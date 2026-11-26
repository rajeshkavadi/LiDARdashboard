#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Stream a SLAM backend's global map, pose and trajectory to the dashboard.

Subscribes (read-only) to the registered scan and odometry published by *any*
SLAM backend -- the built-in ``lidar_odometry.py``, FAST-LIO2, Point-LIO,
LIO-SAM, DLIO, ... -- because they all publish the same contract:
``nav_msgs/Odometry`` plus a registered ``sensor_msgs/PointCloud2``.  Empty
topic parameters are auto-detected, so switching backend needs no edits here.

Incoming points are voxel-downsampled per spatial tile and only *new* points
are sent to the loopback map ingest (default 127.0.0.1:8766), so the web
backend and Three.js never receive the whole map per frame.

This is not SLAM itself: poses and the registered cloud come from the backend.
No raw sensor-relative accumulation is performed.

Parameters (private):
    ~backend_name      label shown in the dashboard (default "generic")
    ~odom_topic        default: auto-detect /Odometry
    ~cloud_topic       default: auto-detect /cloud_registered
    ~topic_wait_s      seconds to wait for those topics to appear (default 120)
    ~server_host       default 127.0.0.1
    ~server_port       default 8766
    ~tile_size         default 20.0 m
    ~voxel_size        default 0.15 m
    ~max_voxels        default 2000000
    ~max_tiles         default 20000
    ~trajectory_min_step default 0.01 m
"""

import json
import math
import os
import Queue
import socket
import struct
import sys
import threading
import time

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

# Share the exact protocol constants/functions with the Python 3 backend.
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__),
                                    "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(ROOT, "webapp"))
import protocol  # noqa: E402

HELLO_MAGIC = b"LVS1"


def log_info(message):
    rospy.loginfo("[slam_web_bridge] " + message)


# Hints only: any nav_msgs/Odometry and any registered PointCloud2 topic is
# accepted, so an unfamiliar backend still works without configuration.
ODOM_CANDIDATES = ("/Odometry", "/odometry", "/odom", "/Odometry_high_rate",
                   "/lio_sam/mapping/odometry", "/lio_sam/mapping/odometry_incremental",
                   "/aft_mapped_to_init", "/point_lio/odometry",
                   "/dlio/odom_node/odom", "/fast_lio/odometry",
                   "/slam/odometry", "/odometry/filtered")
CLOUD_CANDIDATES = ("/cloud_registered", "/cloud_registered_body",
                    "/lio_sam/mapping/cloud_registered",
                    "/lio_sam/mapping/cloud_registered_body",
                    "/point_lio/cloud_registered", "/dlio/odom_node/pointcloud",
                    "/fast_lio/cloud_registered", "/registered_scan",
                    "/slam/cloud_registered")
SELF_TOPICS = ("/points",)  # never feed raw input back as the registered map


def _published_topics():
    try:
        return dict(rospy.get_published_topics())
    except Exception:  # noqa: BLE001 - mastership race at startup
        return {}


def _autodetect(preferred, candidates, kind, exclude=()):
    if preferred:
        return preferred
    available = _published_topics()
    for name in candidates:
        if name in available and kind in available[name]:
            return name
    for name in sorted(available):
        if name not in exclude and kind in available[name]:
            return name
    return None


class SlamWebBridge(object):
    def __init__(self):
        self.backend_name = rospy.get_param("~backend_name", "generic")
        self.topic_wait_s = float(rospy.get_param("~topic_wait_s", 120.0))
        if not self._resolve_topics():
            rospy.logerr("[slam_web_bridge] no SLAM topics found after %.0f s. "
                         "Expected an Odometry topic and a registered "
                         "PointCloud2 topic. Published: %s"
                         % (self.topic_wait_s,
                            ", ".join(sorted(_published_topics().keys()))))
            raise SystemExit(1)
        self.server_host = rospy.get_param("~server_host", "127.0.0.1")
        self.server_port = int(rospy.get_param("~server_port", 8766))
        self.tile_size = float(rospy.get_param("~tile_size", 20.0))
        self.voxel_size = float(rospy.get_param("~voxel_size", 0.15))
        self.max_voxels = int(rospy.get_param("~max_voxels", 2000000))
        self.max_tiles = int(rospy.get_param("~max_tiles", 20000))
        self.min_step = float(rospy.get_param("~trajectory_min_step", 0.01))

        self.tiles = {}            # (tx,ty,tz) -> set(voxel_key)
        self.voxel_count = 0
        self.dropped = 0
        self.traj_count = 0
        self.last_traj_xyz = None
        self.lock = threading.Lock()
        self.queue = Queue.Queue(maxsize=4096)
        self.stop = threading.Event()
        self.sock = None

        self.sender = threading.Thread(target=self._sender_loop,
                                       name="slam-web-sender")
        self.sender.daemon = True
        self.sender.start()

        self.odom_sub = rospy.Subscriber(
            self.odom_topic, Odometry, self._on_odom, queue_size=20)
        self.cloud_sub = rospy.Subscriber(
            self.cloud_topic, PointCloud2, self._on_cloud,
            queue_size=2, buff_size=2 ** 24)
        log_info("odometry: %s   cloud: %s" % (self.odom_topic, self.cloud_topic))
        log_info("sending to tcp://%s:%d (tile %.1f m, voxel %.2f m)" %
                 (self.server_host, self.server_port,
                  self.tile_size, self.voxel_size))

        self.stats_timer = rospy.Timer(
            rospy.Duration(10.0), self._log_stats)

    def _resolve_topics(self):
        """Auto-detect the SLAM contract, waiting for the backend to start.

        The SLAM node is launched just before this one (and an external
        backend may take seconds to initialise), so a single look at the
        topic list would often miss it and kill the exporter.
        """
        odom_param = rospy.get_param("~odom_topic", "")
        cloud_param = rospy.get_param("~cloud_topic", "")
        deadline = time.time() + max(0.0, self.topic_wait_s)
        waiting_logged = False
        while True:
            self.odom_topic = _autodetect(odom_param, ODOM_CANDIDATES,
                                          "nav_msgs/Odometry")
            self.cloud_topic = _autodetect(cloud_param, CLOUD_CANDIDATES,
                                           "sensor_msgs/PointCloud2",
                                           SELF_TOPICS)
            if self.odom_topic and self.cloud_topic:
                return True
            if rospy.is_shutdown() or time.time() >= deadline:
                return False
            if not waiting_logged:
                log_info("waiting for the SLAM topics (odometry + registered "
                         "cloud) to be published...")
                waiting_logged = True
            time.sleep(0.5)

    # -- outbound queue -------------------------------------------------
    def _enqueue(self, message):
        try:
            self.queue.put_nowait(message)
        except Queue.Full:
            self.dropped += 1

    def _hello_bytes(self):
        hello = {
            "role": "slam-exporter",
            "backend": self.backend_name,
            "odom_topic": self.odom_topic,
            "cloud_topic": self.cloud_topic,
            "tile_size_m": self.tile_size,
            "voxel_size_m": self.voxel_size,
            "max_voxels": self.max_voxels,
            "pid": os.getpid(),
            "stamp": time.time(),
        }
        payload = json.dumps(hello).encode("utf-8")
        return (HELLO_MAGIC + struct.pack("<I", len(payload)) + payload)

    def _sender_loop(self):
        while not self.stop.is_set() and not rospy.is_shutdown():
            if self.sock is None:
                try:
                    sock = socket.create_connection(
                        (self.server_host, self.server_port), timeout=5.0)
                    sock.settimeout(None)
                    # The backend clears its map for a new session, so the
                    # exporter must forget what it already sent and replay.
                    self._reset()
                    sock.sendall(self._hello_bytes())
                    self.sock = sock
                    log_info("connected to web map ingest")
                    reader = threading.Thread(
                        target=self._reader_loop, args=(sock,),
                        name="slam-web-reader")
                    reader.daemon = True
                    reader.start()
                except (socket.error, socket.timeout) as exc:
                    rospy.logwarn_throttle(
                        15.0, "[slam_web_bridge] web ingest unreachable (%s)"
                        % exc)
                    self.stop.wait(1.0)
                    continue
            try:
                message = self.queue.get(timeout=0.5)
            except Queue.Empty:
                continue
            try:
                self.sock.sendall(message)
            except socket.error as exc:
                rospy.logwarn("[slam_web_bridge] send failed (%s)" % exc)
                try:
                    self.sock.close()
                except socket.error:
                    pass
                self.sock = None

    def _reader_loop(self, sock):
        """Receive RESET from the backend (the only inbound command)."""
        try:
            while not self.stop.is_set():
                head = self._recv_exact(sock, 8)
                if head is None:
                    break
                if head[:4] != protocol.MAP_MAGIC:
                    break
                msg_type = (head[4] if isinstance(head[4], int)
                            else ord(head[4]))
                if msg_type == protocol.MAP_RESET:
                    self._reset()
        except (socket.error, OSError):
            pass
        finally:
            if self.sock is sock:
                self.sock = None

    @staticmethod
    def _recv_exact(sock, size):
        chunks = []
        remaining = size
        while remaining > 0:
            try:
                chunk = sock.recv(remaining)
            except socket.error:
                return None
            if not chunk:
                return None
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _reset(self):
        with self.lock:
            self.tiles = {}
            self.voxel_count = 0
            self.traj_count = 0
            self.last_traj_xyz = None
        while True:
            try:
                self.queue.get_nowait()
            except Queue.Empty:
                break
        log_info("map reset received; cleared local voxel/trajectory state")

    # -- SLAM backend input ----------------------------------------------
    def _on_odom(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        stamp = msg.header.stamp.to_sec()
        pose = protocol.map_pose(stamp, position.x, position.y, position.z,
                                 orientation.x, orientation.y,
                                 orientation.z, orientation.w)
        self._enqueue(pose)

        # The first pose always seeds the trajectory, otherwise the very first
        # message would return early and the path would never start.
        distance = float("inf")
        if self.last_traj_xyz is not None:
            dx = position.x - self.last_traj_xyz[0]
            dy = position.y - self.last_traj_xyz[1]
            dz = position.z - self.last_traj_xyz[2]
            distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        if distance < self.min_step:
            return
        with self.lock:
            start = self.traj_count
            self.traj_count += 1
            self.last_traj_xyz = (position.x, position.y, position.z)
        xyz = struct.pack("<3d", position.x, position.y, position.z)
        self._enqueue(protocol.map_trajectory(start, xyz))

    def _on_cloud(self, msg):
        fields = dict((f.name, f) for f in msg.fields)
        if not all(name in fields for name in ("x", "y", "z")):
            rospy.logwarn_throttle(
                5.0, "[slam_web_bridge] registered cloud has no x/y/z")
            return
        names = [n for n in ("x", "y", "z", "intensity") if n in fields]
        formats = ["<f4" for _ in names]
        dtype = np.dtype({
            "names": names,
            "formats": formats,
            "offsets": [fields[n].offset for n in names],
            "itemsize": msg.point_step,
        })
        count = msg.width * msg.height
        if count <= 0:
            return
        cloud = np.frombuffer(msg.data, dtype=dtype, count=count)

        pending = {}  # (tx,ty,tz) -> list of floats
        xs = cloud["x"]
        ys = cloud["y"]
        zs = cloud["z"]
        intensity = (cloud["intensity"] if "intensity" in names
                     else np.zeros(count, dtype=np.float32))
        voxel = self.voxel_size
        tile_size = self.tile_size
        limit = 1048575  # 21 bits per axis

        with self.lock:
            for i in xrange(count):
                x = float(xs[i])
                y = float(ys[i])
                z = float(zs[i])
                if not (x == x and y == y and z == z):
                    continue
                ix = int(math.floor(x / voxel))
                iy = int(math.floor(y / voxel))
                iz = int(math.floor(z / voxel))
                if (abs(ix) > limit or abs(iy) > limit or abs(iz) > limit):
                    self.dropped += 1
                    continue
                key3 = (int(math.floor(x / tile_size)),
                        int(math.floor(y / tile_size)),
                        int(math.floor(z / tile_size)))
                bucket = self.tiles.get(key3)
                if bucket is None:
                    if len(self.tiles) >= self.max_tiles:
                        self.dropped += 1
                        continue
                    bucket = set()
                    self.tiles[key3] = bucket
                    pending[key3] = []
                elif key3 not in pending:
                    pending[key3] = []
                if self.voxel_count >= self.max_voxels:
                    self.dropped += 1
                    continue
                voxel_key = (((ix & 0x1FFFFF) << 42) |
                             ((iy & 0x1FFFFF) << 21) |
                             (iz & 0x1FFFFF))
                if voxel_key in bucket:
                    continue
                bucket.add(voxel_key)
                self.voxel_count += 1
                value = float(intensity[i])
                if value < 0:
                    value = 0.0
                elif value > 255:
                    value = 255.0
                pending[key3].extend((x, y, z, value))

        for key3, values in pending.items():
            if not values:
                continue
            packed = np.asarray(values, dtype="<f4").tobytes()
            self._enqueue(protocol.map_tile(
                key3[0], key3[1], key3[2], packed))

    def _log_stats(self, _event):
        with self.lock:
            tiles = len(self.tiles)
            voxels = self.voxel_count
            traj = self.traj_count
        log_info("voxels=%d tiles=%d trajectory=%d dropped=%d queue=%d" %
                 (voxels, tiles, traj, self.dropped, self.queue.qsize()))


def main():
    rospy.init_node("slam_web_bridge", anonymous=False)
    bridge = SlamWebBridge()
    rospy.on_shutdown(bridge.stop.set)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
