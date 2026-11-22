#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""ROS -> web bridge for live LiDAR point clouds.

Subscribes to a sensor_msgs/PointCloud2 topic (default /livox/lidar) and
forwards x, y, z, intensity for every point to the local Flask/WebSocket
backend over a loopback TCP socket.

Runs on Python 2.7 because ROS Melodic's rospy only supports Python 2.
The existing driver, RViz and topic pipeline are left untouched; this is an
additional, read-only subscriber.

Parameters (private):
    ~topic        point-cloud topic            (default /livox/lidar)
    ~server_host  web backend ingest host      (default 127.0.0.1)
    ~server_port  web backend ingest port      (default 8765)
    ~point_stride keep every Nth point         (default 1)
    ~max_points   hard cap for a single frame  (default 0 = unlimited)
"""

import json
import os
import socket
import struct
import threading
import time

import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2

MAGIC = b"LVP1"
HELLO_MAGIC = b"LVS1"
HEADER = struct.Struct("<4sIdIH2x")  # 24 bytes, 2 zero pad bytes
POINT_BYTES = 16
ONLY_FIELDS = ("x", "y", "z", "intensity")
# sensor_msgs/PointField datatype constants -> numpy type codes.
PF_DTYPES = {1: "i1", 2: "u1", 3: "i2", 4: "u2", 5: "i4", 6: "u4",
             7: "f4", 8: "f8"}


def log(message):
    rospy.loginfo("[lidar-web-bridge] " + message)


class PointCloudBridge(object):
    def __init__(self):
        self.topic = rospy.get_param("~topic", "/livox/lidar")
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

        self.sender = threading.Thread(target=self._sender_loop, name="web-sender")
        self.sender.daemon = True
        self.sender.start()

        self.subscriber = rospy.Subscriber(
            self.topic,
            PointCloud2,
            self._on_cloud,
            queue_size=1,
            buff_size=2 ** 24,
        )
        log("subscribed to %s (sensor_msgs/PointCloud2)" % self.topic)
        log("forwarding to tcp://%s:%d (stride=%d)"
            % (self.server_host, self.server_port, self.point_stride))

    # -- ROS callback ---------------------------------------------------
    def _on_cloud(self, msg):
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

        # Build the structured dtype from each field's declared datatype and
        # the message endianness.  (The old code reused a list-comprehension
        # variable that only leaks in Python 2: on Python 3 / Noetic it raised
        # NameError on every frame, and it assumed every field was float32.)
        endian = ">" if msg.is_bigendian else "<"
        names, formats, offsets = [], [], []
        for fname in ONLY_FIELDS:
            field = fields.get(fname)
            if field is None:
                continue
            code = PF_DTYPES.get(field.datatype)
            if code is None:
                raise ValueError("field %s has unsupported datatype %d"
                                 % (fname, field.datatype))
            if fname in ("x", "y", "z") and code[0] != "f":
                raise ValueError("field %s is not floating point" % fname)
            names.append(fname)
            formats.append(endian + code)
            offsets.append(field.offset)
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
        if "intensity" in names:
            out[:, 3] = np.clip(sliced["intensity"].astype("<f4"), 0.0, 255.0)
        else:
            out[:, 3] = 0.0
        return out

    # -- TCP sender -----------------------------------------------------
    def _connect(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((self.server_host, self.server_port))
        sock.settimeout(None)
        hello = {
            "topic": self.topic,
            "message_type": "sensor_msgs/PointCloud2",
            "frame_id": "",
            "master_uri": os.environ.get("ROS_MASTER_URI", ""),
            "pid": os.getpid(),
            "stamp": time.time(),
        }
        payload = json.dumps(hello).encode("utf-8")
        sock.sendall(HELLO_MAGIC + struct.pack("<I", len(payload)) + payload)
        log("connected to web backend at %s:%d" % (self.server_host, self.server_port))
        return sock

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
                sock.sendall(head + padded + points.tobytes())
            except socket.error as exc:
                rospy.logwarn("[lidar-web-bridge] send failed (%s); reconnecting" % exc)
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
