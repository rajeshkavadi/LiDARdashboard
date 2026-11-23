#!/usr/bin/env python3
"""Local three.js viewer for CPU PointPillars -- no RViz required.

Two data sources feed the same browser page:

  bag mode (default, completely ROS-free)
      python3 tools/pointpillars_web_view.py --bag data/demo1.bag
      Reads LiDAR frames straight from a ROS1 bag, runs the CPU detector in
      this process and streams points + boxes to http://127.0.0.1:8080.
      The bag replays in a loop until Ctrl+C, and a Cloudflare quick tunnel
      (https://<random>.trycloudflare.com) is opened for remote viewers.

  ROS mode (replaces RViz when the detector is already running as a node)
      python3 tools/pointpillars_web_view.py --ros
      Subscribes to ``/pointcloud`` and ``/detect_json`` and mirrors them to
      the same page.

The page draws the cloud colourised by range, concentric 5 m radar rings with
distance labels, and the 3D boxes.  Everything it needs is vendored in
``webapp/`` so the viewer works offline.
"""

import argparse
import json
import os
import sys
import threading
import time

import numpy as np


def _project_root():
    """Works from the source tree, a catkin install space and any machine."""
    env = os.environ.get("LIVOX_DETECTION_ROOT")
    if env and os.path.isdir(env):
        return os.path.abspath(env)
    here = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if os.path.isdir(os.path.join(here, "webapp")):
        return here
    try:
        import rospkg
        found = rospkg.RosPack().get_path("livox_detection")
        if found:
            return found
    except Exception:
        pass
    return here


PROJECT_ROOT = _project_root()
sys.path.insert(0, PROJECT_ROOT)

from pointpillars.config import ConfigError, load_config  # noqa: E402
from pointpillars.tunnel import CloudflareTunnel  # noqa: E402
from pointpillars.web_server import (  # noqa: E402
    DEFAULT_WEBAPP_DIR, FrameHub, WebViewerServer, detections_payload, encode_frame,
)


def _existing(path):
    return path if path and os.path.isfile(os.path.expanduser(path)) else None


def resolve_bag(value):
    """Find a bag without hard-coding anyone's home directory."""
    candidates = [
        value,
        os.environ.get("LIVOX_BAG"),
        os.path.join(PROJECT_ROOT, "data", "demo1.bag"),
        os.path.join(os.path.dirname(PROJECT_ROOT), "demo1.bag"),
        os.path.join(os.getcwd(), "demo1.bag"),
    ]
    for candidate in candidates:
        found = _existing(candidate)
        if found:
            return os.path.abspath(os.path.expanduser(found))
    raise SystemExit(
        "bag not found. Pass --bag /path/to/file.bag (looked in: %s)"
        % ", ".join(os.path.abspath(os.path.expanduser(c))
                    for c in candidates if c))


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Local three.js PointPillars viewer (bag or ROS source)")
    parser.add_argument("--bag", default=None, help="ROS1 bag to replay")
    parser.add_argument("--topic", default=None,
                        help="PointCloud2 topic in the bag (auto-detected)")
    parser.add_argument("--start-frame", type=int, default=0)
    parser.add_argument("--max-frames", type=int, default=0,
                        help="stop after N frames (0 = run forever)")
    parser.add_argument("--no-loop", dest="loop", action="store_false", default=True,
                        help="stop at the end of the bag instead of replaying")
    parser.add_argument("--play-rate", type=float, default=1.0,
                        help="pacing for --no-infer replay (1.0 = bag speed)")
    parser.add_argument("--ros", action="store_true",
                        help="mirror /pointcloud + /detect_json instead of a bag")
    parser.add_argument("--points-topic", default=None,
                        help="ROS mode: point cloud topic (default /pointcloud)")
    parser.add_argument("--json-topic", default=None,
                        help="ROS mode: detection JSON topic (default /detect_json)")
    parser.add_argument("--no-infer", dest="infer", action="store_false",
                        default=True, help="bag mode: show the cloud without detections")
    parser.add_argument("--ckpt", default=os.path.join(
        PROJECT_ROOT, "checkpoints", "pointpillars_checkpoint.pth"))
    parser.add_argument("--cfg_file", default=os.path.join(
        PROJECT_ROOT, "configs", "pointpillars.yaml"))
    parser.add_argument("--intensity-mode", dest="intensity_mode", default=None,
                        choices=["zero", "constant", "field"])
    parser.add_argument("--z-offset", dest="z_offset", type=float, default=None)
    parser.add_argument("--score-thresh", dest="score_thresh", type=float, default=None)
    parser.add_argument("--nms-thresh", dest="nms_thresh", type=float, default=None)
    parser.add_argument("--host", default="127.0.0.1",
                        help="bind address (localhost only by default)")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--webapp-dir", default=None,
                        help="override the webapp directory (default: packaged one)")
    parser.add_argument("--no-share", dest="share", action="store_false", default=True,
                        help="keep the viewer local instead of opening a "
                             "Cloudflare quick tunnel")
    parser.add_argument("--cloudflared", default=None,
                        help="path to the cloudflared binary (default: found on PATH)")
    return parser.parse_args(argv)


def empty_detections(n_points):
    return {
        "boxes": np.zeros((0, 7), dtype=np.float32),
        "scores": np.zeros((0,), dtype=np.float32),
        "labels": np.zeros((0,), dtype=np.int64),
        "timings": {},
        "num_original_points": int(n_points),
    }


def publish_frame(hub, detector, frame, seq, class_names):
    """Run inference (if any) on one bag frame and publish it."""
    points = frame["points"]
    started = time.perf_counter()
    if detector is not None:
        det = detector.detect(points, frame["has_intensity"])
    else:
        det = empty_detections(len(points))
    infer_ms = (time.perf_counter() - started) * 1000.0
    boxes = detections_payload(det["boxes"], det["scores"], det["labels"], class_names)
    meta = {
        "infer_ms": round(infer_ms, 1),
        "points_in": int(len(points)),
        "voxels": int(det.get("num_voxels", 0)),
        "intensity": bool(frame.get("has_intensity")),
        "index": int(frame.get("index", seq)),
    }
    payload = encode_frame(seq, frame.get("stamp", time.time()),
                           frame.get("frame_id", "livox_frame"), points, boxes, meta)
    hub.publish(payload, seq, len(points), len(boxes),
                frame.get("stamp", time.time()), frame.get("frame_id", ""), meta)
    return det, boxes


def run_bag(args, hub, stop_event):
    from pointpillars.bag_frame import find_cloud_topic, iter_frames

    bag = resolve_bag(args.bag)
    topic = args.topic or find_cloud_topic(bag)[0]
    print("[web-view] bag:   %s" % bag)
    print("[web-view] topic: %s%s" % (topic, "" if args.infer else " (detections off)"))

    class_names = ["Car", "Pedestrian", "Cyclist"]
    detector = None
    if args.infer:
        overrides = {"DEVICE": "cpu"}
        if args.intensity_mode:
            overrides["INTENSITY_MODE"] = args.intensity_mode
        if args.z_offset is not None:
            overrides["Z_OFFSET"] = args.z_offset
        cfg = load_config(args.cfg_file, overrides)
        if args.score_thresh is not None:
            cfg["post"]["score_thresh"] = float(args.score_thresh)
        if args.nms_thresh is not None:
            cfg["post"]["nms_thresh"] = float(args.nms_thresh)
        class_names = list(cfg["class_names"])
        from pointpillars.pipeline import PointPillarDetector  # imports torch only here
        detector = PointPillarDetector(cfg, args.ckpt, overrides)
        print("[web-view] checkpoint: %s (%s)"
              % (os.path.basename(args.ckpt), detector.checkpoint_status))

    seq = 0
    while not stop_event.is_set():
        produced = 0
        frame_started = None
        for frame in iter_frames(bag, topic, 0):
            if stop_event.is_set():
                return
            if frame["index"] < args.start_frame:
                continue
            if args.max_frames and seq >= args.max_frames:
                return
            frame_started = time.perf_counter()
            det, boxes = publish_frame(hub, detector, frame, seq, class_names)
            seq += 1
            produced += 1
            if args.max_frames and seq >= args.max_frames:
                return
            if produced % 5 == 0 or produced == 1:
                print("[web-view] frame %d | points=%d boxes=%d | %.0f ms"
                      % (seq, len(frame["points"]), len(boxes),
                         (time.perf_counter() - frame_started) * 1000.0),
                      flush=True)
            if detector is None and args.play_rate > 0:
                # Bag frames are 10 Hz; keep a plain cloud view readable.
                target = 0.1 / args.play_rate
                elapsed = time.perf_counter() - frame_started
                if elapsed < target:
                    time.sleep(target - elapsed)
        if not args.loop:
            return
        print("[web-view] end of bag, replaying from the start")


def run_ros(args, hub, stop_event):
    import rospy
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from pointpillars.ros_utils import pointcloud2_to_numpy

    rospy.init_node("pointpillars_web_view", anonymous=True, disable_signals=True)
    points_topic = args.points_topic or rospy.get_param("~points_topic", "/pointcloud")
    json_topic = args.json_topic or rospy.get_param("~json_topic", "/detect_json")
    print("[web-view] ROS in: %s + %s" % (points_topic, json_topic))

    lock = threading.Lock()
    state = {"frame": None, "seq": 0, "boxes": [], "meta": {}, "json_count": 0}
    pending = {"frame": None}

    def on_cloud(msg):
        try:
            points, has_intensity, _fields = pointcloud2_to_numpy(msg)
        except Exception as exc:  # one bad frame must not kill the viewer
            rospy.logerr("web view: bad point cloud: %s", exc)
            return
        with lock:
            state["seq"] += 1
            state["frame"] = {
                "points": points,
                "has_intensity": has_intensity,
                "stamp": msg.header.stamp.to_sec(),
                "frame_id": msg.header.frame_id or "livox_frame",
                "index": state["seq"] - 1,
            }
            pending["frame"] = state["frame"]

    def on_json(msg):
        try:
            doc = json.loads(msg.data)
        except ValueError:
            return
        with lock:
            state["boxes"] = doc.get("boxes") or []
            state["meta"] = doc.get("meta") or {}
            state["json_count"] += 1

    rospy.Subscriber(points_topic, PointCloud2, on_cloud, queue_size=1)
    rospy.Subscriber(json_topic, String, on_json, queue_size=10)

    published_seq = -1
    rate = rospy.Rate(50)
    while not stop_event.is_set() and not rospy.is_shutdown():
        with lock:
            frame = pending["frame"]
            boxes = list(state["boxes"])
            meta = dict(state["meta"])
        if frame is None or frame["index"] <= published_seq:
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                break
            continue
        published_seq = frame["index"]
        meta.update({"points_in": int(len(frame["points"])),
                     "intensity": bool(frame["has_intensity"]),
                     "index": frame["index"]})
        payload = encode_frame(frame["index"], frame["stamp"], frame["frame_id"],
                               frame["points"], boxes, meta)
        hub.publish(payload, frame["index"], len(frame["points"]), len(boxes),
                    frame["stamp"], frame["frame_id"], meta)


def main(argv=None):
    args = parse_args(argv)
    if not args.ros:
        if args.infer and not os.path.isfile(args.ckpt):
            raise SystemExit("checkpoint not found: %s" % args.ckpt)
        if not os.path.isfile(args.cfg_file):
            raise SystemExit("config not found: %s" % args.cfg_file)

    hub = FrameHub(source="ros" if args.ros else "bag")
    server = WebViewerServer(hub, host=args.host, port=args.port,
                            webapp_dir=args.webapp_dir or DEFAULT_WEBAPP_DIR)
    try:
        server.start()
    except OSError as exc:
        raise SystemExit("cannot listen on %s:%d (%s); try --port 8081"
                         % (args.host, args.port, exc))

    tunnel = None
    if args.share:
        print("[web-view] opening a Cloudflare quick tunnel ...", flush=True)
        try:
            tunnel = CloudflareTunnel(args.port, host=args.host,
                                      binary=args.cloudflared)
            tunnel.start()
        except RuntimeError as exc:
            tunnel = None
            print("[web-view] share unavailable: %s" % exc, file=sys.stderr)

    print("", flush=True)
    print("==============================================", flush=True)
    print("   POINTPILLARS WEB VIEWER  (three.js)", flush=True)
    print("==============================================", flush=True)
    print("  open:  %s" % server.url, flush=True)
    if tunnel is not None:
        print("  share: %s" % tunnel.url, flush=True)
        print("         (public Cloudflare link, share with anyone)", flush=True)
    else:
        print("  share: off (local only)", flush=True)
    print("  source: %s" % ("ROS topics" if args.ros else "bag replay"), flush=True)
    print("  rings: 5 m steps, cloud colourised by range", flush=True)
    if not args.ros and args.loop:
        print("  replay: loop until Ctrl+C", flush=True)
    print("==============================================", flush=True)
    print("", flush=True)

    stop_event = threading.Event()
    try:
        if args.ros:
            try:
                run_ros(args, hub, stop_event)
            except Exception as exc:  # roscore missing, bad topic, ...
                print("[web-view] ROS mode failed: %s" % exc, file=sys.stderr)
                return 2
        else:
            run_bag(args, hub, stop_event)
    except ConfigError as exc:
        print("[web-view] configuration error: %s" % exc, file=sys.stderr)
        return 2
    except RuntimeError as exc:
        print("[web-view] error: %s" % exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        if tunnel is not None:
            tunnel.stop()
        server.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
