#!/usr/bin/env python3
"""LiDAR web dashboard backend.

Serves the dashboard, a 1 Hz status API, the original binary live-cloud
WebSocket, plus incremental global-map and DEM streams.  LiDAR data is received
from the ROS bridge over loopback and never leaves the machine except to the
browser that is viewing it.

Cloudflare Quick Tunnel is started automatically and proxies only this Flask
port.  ROS/SLAM/DEM services are independent processes and are never exposed.
"""

import argparse
import json
import os
import sys
import time

from flask import (Flask, Response, jsonify, render_template, request,
                   send_file, stream_with_context)
from flask_sock import Sock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_hub import FrameHub, FrameIngestServer  # noqa: E402
import dem_hub as dem_hub_module  # noqa: E402
import map_hub as map_hub_module  # noqa: E402
from map_ingest import MapIngestServer  # noqa: E402
from tunnel import CloudflareTunnel  # noqa: E402

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
DEM_DIR = os.environ.get("LIDAR_DEM_DIR", "/tmp/lidar_dem")

app = Flask(__name__, template_folder=os.path.join(BASE_DIR, "templates"),
            static_folder=os.path.join(BASE_DIR, "static"))
app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 300
sock = Sock(app)
hub = FrameHub()
map_hub = map_hub_module.MapHub(
    max_points=int(os.environ.get("LIDAR_MAP_MAX_POINTS", "6000000")),
    max_tiles=int(os.environ.get("LIDAR_MAP_MAX_TILES", "20000")))
dem_hub = dem_hub_module.DemHub(
    max_cells=int(os.environ.get("LIDAR_DEM_MAX_CELLS", "4194304")))
ingest = None
map_ingest = None
tunnel = None


def _local_request():
    return request.remote_addr in ("127.0.0.1", "::1", None)


@app.after_request
def no_store_api(response):
    if request.path.startswith("/api/"):
        response.headers["Cache-Control"] = "no-store"
    return response


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/api/status")
def api_status():
    data = hub.snapshot()
    map_stats = map_hub.stats()
    data["map"] = map_stats
    data["slam"] = {
        "connected": map_stats["slam_connected"],
        "peer": map_stats["slam_peer"],
        "pose": map_stats["pose"],
        "pose_age_s": map_stats["pose_age_s"],
        "status": ("mapping" if map_stats["pose_age_s"] is not None and
                   map_stats["pose_age_s"] < 2.5 else
                   ("connected" if map_stats["slam_connected"] else "offline")),
        "meta": map_stats["slam_meta"],
    }
    data["dem"] = dem_hub.stats()
    data["tunnel"] = tunnel.status() if tunnel is not None else data.get("tunnel")
    return jsonify(data)


# --------------------------------------------------------------------------
# Live point cloud (original protocol, unchanged)
# --------------------------------------------------------------------------
@sock.route("/ws/points")
def points_socket(ws):
    """Stream the newest point-cloud frame as a binary WebSocket message.

    Each client only ever receives the latest frame.  A slow client therefore
    skips frames instead of building a backlog, and cannot slow down ingest.
    """
    hub.client_connected()
    last_seq = 0
    try:
        while True:
            frame, seq = hub.wait_for_frame(last_seq, timeout=1.0)
            if frame is None:
                continue
            last_seq = seq
            ws.send(frame)
            hub.add_bytes_sent(len(frame))
    except Exception as exc:  # noqa: BLE001 - any socket error means the client left
        app.logger.debug("websocket client closed: %r", exc)
    finally:
        hub.client_disconnected()


# --------------------------------------------------------------------------
# Global map / trajectory / pose
# --------------------------------------------------------------------------
@sock.route("/ws/map")
def map_socket(ws):
    """Send a full snapshot, then only incremental tile/pose updates."""
    try:
        messages, last_seq = map_hub.snapshot()
        for message in messages:
            ws.send(message)
        while True:
            messages, new_seq, resync = map_hub.wait_for_updates(
                last_seq, timeout=1.0)
            if resync:
                messages, new_seq = map_hub.snapshot()
            for message in messages:
                ws.send(message)
            last_seq = new_seq
    except Exception as exc:  # noqa: BLE001
        app.logger.debug("map websocket closed: %r", exc)


# --------------------------------------------------------------------------
# DEM raster
# --------------------------------------------------------------------------
@sock.route("/ws/dem")
def dem_socket(ws):
    try:
        messages, last_seq = dem_hub.snapshot()
        for message in messages:
            ws.send(message)
        while True:
            messages, new_seq, resync = dem_hub.wait_for_updates(
                last_seq, timeout=1.0)
            if resync:
                messages, new_seq = dem_hub.snapshot()
            for message in messages:
                ws.send(message)
            last_seq = new_seq
    except Exception as exc:  # noqa: BLE001
        app.logger.debug("dem websocket closed: %r", exc)


@app.route("/internal/dem/patch", methods=["POST"])
def internal_dem_patch():
    if not _local_request():
        return jsonify({"error": "loopback only"}), 403
    data = request.get_data(cache=False)
    ok = dem_hub.apply_message(data)
    return jsonify({"ok": bool(ok)}), (200 if ok else 400)


@app.route("/internal/dem/status", methods=["POST"])
def internal_dem_status():
    if not _local_request():
        return jsonify({"error": "loopback only"}), 403
    try:
        dem_hub.update_status(request.get_json(force=True) or {})
    except (ValueError, TypeError) as exc:
        return jsonify({"error": str(exc)}), 400
    return jsonify({"ok": True})


# --------------------------------------------------------------------------
# Exports and reset
# --------------------------------------------------------------------------
@app.route("/api/map/export.pcd")
def api_map_export_pcd():
    stats = map_hub.stats()
    total = stats["points"]

    def generate():
        header = (
            "# .PCD v0.7 - generated by lidar web dashboard\n"
            "VERSION 0.7\n"
            "FIELDS x y z intensity\n"
            "SIZE 4 4 4 4\n"
            "TYPE F F F F\n"
            "COUNT 1 1 1 1\n"
            "WIDTH %d\n"
            "HEIGHT 1\n"
            "VIEWPOINT 0 0 0 1 0 0 0\n"
            "POINTS %d\n"
            "DATA binary\n" % (total, total)
        ).encode("ascii")
        yield header
        for chunk in map_hub.iter_tile_point_bytes():
            yield chunk

    return Response(
        stream_with_context(generate()),
        mimetype="application/octet-stream",
        headers={"Content-Disposition": 'attachment; filename="global_map.pcd"'})


@app.route("/api/map/reset", methods=["POST"])
def api_map_reset():
    payload = request.get_json(silent=True) or {}
    if payload.get("confirm") != "reset":
        return jsonify({"error": "explicit confirmation required",
                        "hint": 'POST {"confirm": "reset"}'}), 400
    map_hub.reset()
    dem_hub.reset(broadcast=True)
    if map_ingest is not None:
        map_ingest.send_reset()
    return jsonify({"ok": True, "map": map_hub.stats(),
                    "dem": dem_hub.stats()})


@app.route("/api/dem/export.tif")
def api_dem_export_tif():
    path = os.path.join(DEM_DIR, "current.tif")
    if not os.path.isfile(path):
        return jsonify({"error": "DEM GeoTIFF not generated yet",
                        "path": path}), 404
    return send_file(path, mimetype="image/tiff", as_attachment=True,
                     download_name="dem_local.tif")


@app.route("/api/dem/export/ground.pcd")
def api_dem_export_ground():
    path = os.path.join(DEM_DIR, "ground_points.pcd")
    if not os.path.isfile(path):
        return jsonify({"error": "ground-point export not generated yet",
                        "path": path}), 404
    return send_file(path, mimetype="application/octet-stream",
                     as_attachment=True,
                     download_name="dem_ground_points.pcd")


@app.route("/api/dem/export.json")
def api_dem_export_json():
    path = os.path.join(DEM_DIR, "dem_meta.json")
    if not os.path.isfile(path):
        return jsonify({"error": "DEM metadata not generated yet",
                        "path": path}), 404
    return send_file(path, mimetype="application/json", as_attachment=True,
                     download_name="dem_metadata.json")


# --------------------------------------------------------------------------
# Startup
# --------------------------------------------------------------------------
def print_banner(port, tunnel_manager):
    local = "http://127.0.0.1:%d" % port
    status = tunnel_manager.status() if tunnel_manager is not None else {}
    remote = status.get("url") or "starting…" if status.get("enabled") else "disabled"
    state = (status.get("state") or "disabled").upper()
    if not status.get("enabled"):
        state = "DISABLED"
    elif status.get("installed") is False:
        state = "CLOUDFLARED NOT INSTALLED"
    print("")
    print("========================================")
    print("       LIDAR WEB DASHBOARD")
    print("========================================")
    print("")
    print("LOCAL ACCESS:")
    print(local)
    print("")
    print("REMOTE ACCESS:")
    print(remote)
    print("")
    print("CLOUDFLARE STATUS: %s" % state)
    print("")
    print("========================================")
    print("")


def main():
    global ingest, map_ingest, tunnel
    parser = argparse.ArgumentParser(description="LiDAR web dashboard")
    parser.add_argument("--host", default=os.environ.get("LIDAR_WEB_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("LIDAR_WEB_PORT", "5000")))
    parser.add_argument("--ingest-host", default="127.0.0.1")
    parser.add_argument("--ingest-port", type=int,
                        default=int(os.environ.get("LIDAR_INGEST_PORT", "8765")))
    parser.add_argument("--map-port", type=int,
                        default=int(os.environ.get("LIDAR_MAP_PORT", "8766")))
    parser.add_argument("--no-tunnel", action="store_true",
                        help="do not start Cloudflare Quick Tunnel")
    args = parser.parse_args()

    ingest = FrameIngestServer(hub, args.ingest_host, args.ingest_port)
    ingest.start()
    map_ingest = MapIngestServer(map_hub, args.ingest_host, args.map_port)
    map_ingest.start()

    tunnel = CloudflareTunnel(port=args.port, disabled=args.no_tunnel,
                              on_ready=lambda _status: print_banner(args.port, tunnel))
    tunnel.start()

    print("[lidar-web] dashboard:  http://%s:%d" % (args.host, args.port))
    print("[lidar-web] live ingest: tcp://%s:%d (ROS bridge)" %
          (args.ingest_host, args.ingest_port))
    print("[lidar-web] map ingest:  tcp://%s:%d (FAST-LIO2 exporter, loopback only)" %
          (args.ingest_host, args.map_port))
    print("[lidar-web] websocket:  /ws/points, /ws/map, /ws/dem")
    print_banner(args.port, tunnel)
    try:
        app.run(host=args.host, port=args.port, threaded=True, use_reloader=False)
    finally:
        if tunnel is not None:
            tunnel.stop()
        if map_ingest is not None:
            map_ingest.stop()
        if ingest is not None:
            ingest.stop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        time.sleep(0.1)
