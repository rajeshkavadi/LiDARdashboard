#!/usr/bin/env python3
"""LiDAR web dashboard backend.

Serves the dashboard, a 1 Hz status API and the binary point-cloud WebSocket.
LiDAR data is received from the ROS bridge over loopback and never leaves the
machine except to the browser that is viewing it.
"""

import argparse
import os
import sys
import time

from flask import Flask, jsonify, render_template, request
from flask_sock import Sock

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from frame_hub import FrameHub, FrameIngestServer  # noqa: E402

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

app = Flask(__name__, template_folder=os.path.join(BASE_DIR, "templates"),
            static_folder=os.path.join(BASE_DIR, "static"))
app.config["SEND_FILE_MAX_AGE_DEFAULT"] = 300
sock = Sock(app)
hub = FrameHub()
ingest = None


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
    return jsonify(hub.snapshot())


@sock.route("/ws/points")
def points_socket(ws):
    """Stream the newest point-cloud frame as a binary WebSocket message.

    Each client only ever receives the latest frame.  A slow client therefore
    skips frames instead of building a backlog, and cannot slow down ingest.
    """
    hub.client_connected()
    last_version = 0
    try:
        while True:
            frame, version = hub.wait_for_frame(last_version, timeout=1.0)
            if frame is None:
                continue
            last_version = version
            ws.send(frame)
            hub.add_bytes_sent(len(frame))
    except Exception as exc:  # noqa: BLE001 - any socket error means the client left
        app.logger.debug("websocket client closed: %r", exc)
    finally:
        hub.client_disconnected()


def main():
    global ingest
    parser = argparse.ArgumentParser(description="LiDAR web dashboard")
    parser.add_argument("--host", default=os.environ.get("LIDAR_WEB_HOST", "127.0.0.1"))
    parser.add_argument("--port", type=int, default=int(os.environ.get("LIDAR_WEB_PORT", "5000")))
    parser.add_argument("--ingest-host", default="127.0.0.1")
    parser.add_argument("--ingest-port", type=int,
                        default=int(os.environ.get("LIDAR_INGEST_PORT", "8765")))
    args = parser.parse_args()

    ingest = FrameIngestServer(hub, args.ingest_host, args.ingest_port)
    ingest.start()

    print("[lidar-web] dashboard:  http://%s:%d" % (args.host, args.port))
    print("[lidar-web] ingest:     tcp://%s:%d (ROS bridge)" % (args.ingest_host, args.ingest_port))
    print("[lidar-web] websocket:  /ws/points")
    app.run(host=args.host, port=args.port, threaded=True, use_reloader=False)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        if ingest is not None:
            ingest.stop()
        time.sleep(0.1)
