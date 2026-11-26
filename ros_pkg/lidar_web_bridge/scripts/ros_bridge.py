#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS -> web bridge for live LiDAR point clouds.

Subscribes to a sensor_msgs/PointCloud2 topic and forwards x, y, z, intensity
for every point to the local Flask/WebSocket backend over a loopback TCP
socket.

The topic is auto-detected when ``~topic`` is left empty, so Mid-360
(``/mid360/livox/lidar``), legacy Livox (``/livox/lidar``), Ouster, Avia and
Velodyne bags all work without arguments.  A missing topic now fails loudly
instead of looking like a frozen dashboard.

Runs on Python 2.7 because ROS Melodic's rospy only supports Python 2.
The existing driver, RViz and topic pipeline are left untouched; this is an
additional, read-only subscriber.

Parameters (private):
    ~topic        point-cloud topic            (default: auto-detect)
    ~imu_topic    IMU topic for health display (default: auto-detect)
    ~server_host  web backend ingest host      (default 127.0.0.1)
    ~server_port  web backend ingest port      (default 8765)
    ~point_stride keep every Nth point         (default 1)
    ~max_points   hard cap for a single frame  (default 0 = unlimited)

IMU samples are only counted for the dashboard (rate/age/frame); they are not
forwarded and are not required for the live point-cloud viewer.
"""

import json
import os
import socket
import struct
import threading
import time

import collections

import numpy as np
import rospy
from sensor_msgs.msg import Imu, PointCloud2

MAGIC = b"LVP1"
HELLO_MAGIC = b"LVS1"
HEADER = struct.Struct("<4sIdIH2x")  # 24 bytes, 2 zero pad bytes
POINT_BYTES = 16
ONLY_FIELDS = ("x", "y", "z", "intensity")


def log(message):
    rospy.loginfo("[lidar-web-bridge] " + message)


# Candidate names, most specific first.  Any bag works as long as its cloud
# topic carries sensor_msgs/PointCloud2, so this list is only a hint: the
# keyword and message-type fallbacks in _pick() catch everything else.
CLOUD_CANDIDATES = (
    # Livox (Mid-360, Avia, Horizon, Tele-15, legacy)
    "/livox/lidar", "/livox/lidar_custom", "/livox/lidar_raw",
    "/livox/lidar_front", "/livox/lidar_left", "/livox/lidar_right",
    "/livox/lidar_1", "/livox/lidar_2", "/livox/points",
    "/livox/points_raw", "/livox_cloud", "/mid360/livox/lidar",
    "/mid360/lidar", "/mid360/points", "/avia/livox/lidar",
    "/horizon/livox/lidar", "/tele/livox/lidar",
    # Ouster / Velodyne / Hesai / RoboSense / Leishen
    "/ouster/points", "/os_cloud_node/points", "/os1_cloud_node/points",
    "/velodyne_points", "/velodyne/points", "/velodyne_cloud",
    "/hesai/points", "/rslidar_points", "/rslidar/points",
    "/lslidar_point_cloud", "/velo_cloud", "/sensor_points",
    # generic
    "/points", "/points_raw", "/lidar_points", "/lidar/points",
    "/cloud", "/lidar/cloud", "/scan_cloud", "/points_cloud",
)
IMU_CANDIDATES = (
    "/livox/imu", "/livox/imu_raw", "/mid360/livox/imu", "/mid360/imu",
    "/avia/livox/imu", "/horizon/livox/imu", "/ouster/imu",
    "/imu", "/imu/data", "/imu/data_raw", "/imu_raw", "/imu/raw",
    "/vn100/imu", "/gps/imu", "/vector/imu", "/imu_9dof",
)
# Topic-name fragments used when no candidate matches exactly.
CLOUD_HINTS = ("livox", "mid360", "lidar", "velodyne", "ouster", "hesai",
               "rslidar", "lslidar", "scan", "points", "cloud", "velo")
FLOW_HINTS = ("imu", "ins", "vector", "vn100")


# Topics this project publishes itself.  Picking one as the *input* cloud would
# make the bridge or the odometry feed on its own output.
SELF_TOPICS = ("/cloud_registered", "/Odometry", "/path", "/map",
               "/cloud_registered_body")

# sensor_msgs/PointField datatype codes.
_ROS_FORMATS = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4",
                7: "f4", 8: "f8"}
INTENSITY_NAMES = ("intensity", "intensities", "reflectivity",
                   "reflectance", "refl", "power", "i")


def _field_format(field):
    """numpy format for one PointField, honouring its real datatype."""
    base = _ROS_FORMATS.get(field.datatype)
    if base is None:
        raise ValueError("unsupported datatype %d for field %r"
                         % (field.datatype, field.name))
    if field.count > 1:
        return ("<" + base, (field.count,))
    return "<" + base


def _raw_field(msg, field, count, name="v"):
    dtype = np.dtype({"names": [name], "formats": [_field_format(field)],
                      "offsets": [field.offset], "itemsize": msg.point_step})
    return np.frombuffer(msg.data, dtype=dtype, count=count)[name]


def _intensity(msg, fields, count):
    """Intensity normalised to 0..255, or None when the cloud has none.

    Handles ``intensity``/``reflectivity``-style scalars *and* the packed
    colour float (``rgb``/``rgba``) that Velodyne converters and some Livox
    exports publish instead, so Intensity colouring works for both.
    """
    for name in INTENSITY_NAMES:
        if name not in fields:
            continue
        try:
            values = _raw_field(msg, fields[name], count)
        except ValueError:
            continue
        values = np.asarray(values, dtype=np.float64)
        if values.ndim > 1:
            values = values[:, 0]
        if values.size == 0:
            return None
        peak = float(np.nanmax(values))
        if peak > 0 and peak <= 1.5:          # 0..1 reflectance
            values = values * 255.0
        elif peak > 255.0:                    # 0..65535 or raw counts
            values = values * (255.0 / peak)
        return values
    for name in ("rgb", "rgba"):
        if name not in fields:
            continue
        field = fields[name]
        if field.datatype not in (5, 6, 7):
            continue
        packed = _raw_field(msg, field, count)
        if field.datatype == 7:               # float32 holding a packed uint32
            packed = np.asarray(packed).view(np.uint32)
        else:
            packed = np.asarray(packed).astype(np.uint32)
        red = (packed >> 16) & 0xFF
        green = (packed >> 8) & 0xFF
        blue = packed & 0xFF
        return 0.299 * red + 0.587 * green + 0.114 * blue
    return None


def _published_topics():
    try:
        return dict(rospy.get_published_topics())
    except Exception:  # noqa: BLE001 - mastership may not be ready yet
        return {}


def _pick_many(preferred, candidates, kind, hints=(), exclude=(), limit=6):
    """Every topic that could feed us, best guess first.

    Candidates win, then keyword matches, then any topic of the wanted type.
    An explicit ``~topic`` is tried first but not exclusive, so a stale
    parameter cannot leave the bridge deaf.  All of them are subscribed and the
    first that delivers data wins.
    """
    available = _published_topics()
    ordered = [preferred] if preferred else []
    for name in candidates:
        if name in available and kind in available[name]:
            ordered.append(name)
    for hint in hints:
        for name in sorted(available):
            if name in exclude or hint not in name.lower():
                continue
            if kind in available[name] and name not in ordered:
                ordered.append(name)
    for name in sorted(available):
        if (name not in exclude and kind in available[name]
                and name not in ordered):
            ordered.append(name)
    return ordered[:limit]


def _pick(preferred, candidates, kind, hints=(), exclude=()):
    """Single best topic (used for the optional IMU health topic)."""
    found = _pick_many(preferred, candidates, kind, hints, exclude, limit=1)
    if found:
        return found[0], None
    return None, "no %s topic found; published: %s" % (
        kind, ", ".join(sorted(_published_topics().keys())) or "nothing")


class PointCloudBridge(object):
    def __init__(self):
        self.explicit_topic = rospy.get_param("~topic", "")
        self.topic = None
        self.topic_error = None
        self.subs = {}
        self.rank = {}          # topic -> candidate priority (lower is better)
        candidates = _pick_many(
            self.explicit_topic, CLOUD_CANDIDATES,
            "sensor_msgs/PointCloud2", CLOUD_HINTS, SELF_TOPICS)
        if candidates:
            log("cloud candidates: %s" % ", ".join(candidates))
        else:
            rospy.logwarn("[lidar-web-bridge] no PointCloud2 topic yet; "
                          "waiting for one to appear")
        self.imu_topic, _imu_error = _pick(
            rospy.get_param("~imu_topic", ""), IMU_CANDIDATES,
            "sensor_msgs/Imu", FLOW_HINTS)
        self.server_host = rospy.get_param("~server_host", "127.0.0.1")
        self.server_port = int(rospy.get_param("~server_port", 8765))
        self.point_stride = max(1, int(rospy.get_param("~point_stride", 1)))
        self.max_points = max(0, int(rospy.get_param("~max_points", 0)))

        self.queue = []
        self.queue_lock = threading.Lock()
        self.queue_event = threading.Event()
        self.stop_event = threading.Event()
        self.seq = 0
        self.subscriber_count = 0

        self.sock = None
        self.send_lock = threading.Lock()
        self.imu_arrivals = collections.deque(maxlen=50)
        self.imu_last = 0.0
        self.imu_frame = ""
        self.imu_lock = threading.Lock()

        self.sender = threading.Thread(target=self._sender_loop, name="web-sender")
        self.sender.daemon = True
        self.sender.start()
        self.meta = threading.Thread(target=self._meta_loop, name="web-meta")
        self.meta.daemon = True
        self.meta.start()

        self._subscribe(candidates)
        self.discovery = rospy.Timer(rospy.Duration(3.0), self._discover)
        self.imu_subscriber = None
        if self.imu_topic is not None:
            self.imu_subscriber = rospy.Subscriber(
                self.imu_topic, Imu, self._on_imu, queue_size=10)
        log("subscribed to %s (sensor_msgs/PointCloud2)"
            % (self.topic or "(waiting for a cloud topic)"))
        if self.imu_topic is not None:
            log("monitoring IMU %s (health only)" % self.imu_topic)
        else:
            log("no IMU topic found; IMU health will stay unknown")
        log("forwarding to tcp://%s:%d (stride=%d)"
            % (self.server_host, self.server_port, self.point_stride))

    # -- topic discovery ------------------------------------------------
    def _subscribe(self, candidates):
        for topic in candidates:
            if topic not in self.rank:
                self.rank[topic] = len(self.rank)
            if topic in self.subs:
                continue
            self.subs[topic] = rospy.Subscriber(
                topic, PointCloud2,
                lambda msg, t=topic: self._on_cloud(msg, t),
                queue_size=1, buff_size=2 ** 24)

    def _discover(self, _event=None):
        if self.topic is not None:
            # Several LiDARs may publish; for the first frames a better-ranked
            # topic can still take over, then the extra subscriptions are cut.
            if self.seq >= 5 and len(self.subs) > 1:
                for name, sub in list(self.subs.items()):
                    if name == self.topic:
                        continue
                    try:
                        sub.unregister()
                    except Exception:  # noqa: BLE001 - shutdown race
                        pass
                    self.subs.pop(name, None)
            return
        candidates = _pick_many(self.explicit_topic, CLOUD_CANDIDATES,
                                "sensor_msgs/PointCloud2", CLOUD_HINTS,
                                SELF_TOPICS)
        if candidates:
            log("subscribing to %s" % ", ".join(candidates))
            self._subscribe(candidates)

    # -- ROS callback ---------------------------------------------------
    def _on_cloud(self, msg, topic=None):
        # Keep the first topic that really publishes and drop the rest.
        if self.topic is None:
            self.topic = topic
            if self.topic_error is None:
                log("locked onto cloud topic %s" % topic)
        elif topic is not None and topic != self.topic:
            current = self.rank.get(self.topic, 1 << 30)
            if self.seq < 5 and self.rank.get(topic, 1 << 30) < current:
                log("preferring %s over %s" % (topic, self.topic))
                self.topic = topic
            else:
                return
        try:
            points = self._extract(msg)
        except Exception as exc:  # noqa: BLE001 - never kill the subscriber
            rospy.logwarn_throttle(5.0, "[lidar-web-bridge] bad frame: %s" % exc)
            return

        if points is None or len(points) == 0:
            return

        self.seq += 1
        item = (points, msg.header.stamp.to_sec(), msg.header.frame_id or "",
                self.seq)
        # Keep at most two frames queued; stale frames are dropped.
        with self.queue_lock:
            if len(self.queue) >= 2:
                self.queue.pop(0)
            self.queue.append(item)
        self.queue_event.set()

    def _on_imu(self, msg):
        now = time.time()
        with self.imu_lock:
            self.imu_last = now
            self.imu_frame = msg.header.frame_id or ""
            self.imu_arrivals.append(now)

    def _imu_status(self):
        with self.imu_lock:
            arrivals = list(self.imu_arrivals)
            last = self.imu_last
            frame = self.imu_frame
        fps = 0.0
        if len(arrivals) > 1:
            span = arrivals[-1] - arrivals[0]
            if span > 0:
                fps = (len(arrivals) - 1) / span
        age = (time.time() - last) if last else None
        return {
            "topic": self.imu_topic,
            "frame_id": frame,
            "fps": round(fps, 2),
            "last_sample_age_s": round(age, 3) if age is not None else None,
            "active": bool(age is not None and age < 1.5),
        }

    def _extract(self, msg):
        fields = {}
        for field in msg.fields:
            fields[field.name] = field
        if "x" not in fields or "y" not in fields or "z" not in fields:
            raise ValueError("point cloud has no x/y/z fields")

        count = msg.width * msg.height
        if count <= 0:
            return None
        if self.max_points and count > self.max_points:
            step = int(np.ceil(float(count) / self.max_points))
        else:
            step = self.point_stride

        # Build one list per dtype key.  (The previous version used a
        # comprehension whose loop variable leaked in Python 2, so ``formats``
        # could be shorter than ``names`` and numpy rejected every frame.)
        names = [name for name in ONLY_FIELDS if name in fields]
        offsets = [fields[name].offset for name in names]
        formats = [_field_format(fields[name]) for name in names]
        dtype = np.dtype({
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": msg.point_step,
        })
        cloud = np.frombuffer(msg.data, dtype=dtype, count=count)

        n = (count + step - 1) // step
        out = np.empty((n, 4), dtype="<f4")
        sliced = cloud[::step]
        out[:, 0] = sliced["x"]
        out[:, 1] = sliced["y"]
        out[:, 2] = sliced["z"]
        intensity = _intensity(msg, fields, count)
        if intensity is None:
            out[:, 3] = 0.0
        else:
            np.clip(intensity[::step], 0.0, 255.0, out=out[:, 3])
        return out

    # -- TCP sender -----------------------------------------------------
    def _hello_bytes(self):
        hello = {
            "topic": self.topic or "(waiting)",
            "message_type": "sensor_msgs/PointCloud2",
            "frame_id": "",
            "master_uri": os.environ.get("ROS_MASTER_URI", ""),
            "pid": os.getpid(),
            "stamp": time.time(),
            "imu": self._imu_status(),
        }
        payload = json.dumps(hello).encode("utf-8")
        return HELLO_MAGIC + struct.pack("<I", len(payload)) + payload

    def _connect(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((self.server_host, self.server_port))
        sock.settimeout(None)
        sock.sendall(self._hello_bytes())
        with self.send_lock:
            self.sock = sock
        log("connected to web backend at %s:%d" % (self.server_host, self.server_port))
        return sock

    def _meta_loop(self):
        """Send a 1 Hz hello so IMU health and bridge metadata stay current."""
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            self.stop_event.wait(1.0)
            with self.send_lock:
                sock = self.sock
                if sock is None:
                    continue
                try:
                    sock.sendall(self._hello_bytes())
                except socket.error:
                    # The sender thread owns reconnection; just skip this tick.
                    pass

    def _sender_loop(self):
        sock = None
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            if sock is None:
                try:
                    sock = self._connect()
                except (socket.error, socket.timeout) as exc:
                    rospy.logwarn_throttle(
                        15.0,
                        "[lidar-web-bridge] web backend not reachable (%s); retrying" % exc)
                    time.sleep(1.0)
                    continue

            self.queue_event.wait(0.5)
            self.queue_event.clear()
            item = None
            with self.queue_lock:
                if self.queue:
                    item = self.queue.pop(0)
            if item is None:
                continue

            points, stamp, frame_id, seq = item
            try:
                encoded_id = frame_id.encode("utf-8")[:1024]
                padded = encoded_id + b"\x00" * ((4 - len(encoded_id) % 4) % 4)
                head = HEADER.pack(MAGIC, points.shape[0], stamp, seq,
                                   len(encoded_id))
                with self.send_lock:
                    sock.sendall(head + padded + points.tobytes())
            except socket.error as exc:
                rospy.logwarn("[lidar-web-bridge] send failed (%s); reconnecting" % exc)
                with self.send_lock:
                    self.sock = None
                try:
                    sock.close()
                except socket.error:
                    pass
                sock = None

        if sock is not None:
            try:
                sock.close()
            except socket.error:
                pass
        with self.send_lock:
            self.sock = None

    def shutdown(self):
        self.stop_event.set()
        self.queue_event.set()


def main():
    rospy.init_node("lidar_web_bridge", anonymous=False)
    bridge = PointCloudBridge()
    rospy.on_shutdown(bridge.shutdown)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
