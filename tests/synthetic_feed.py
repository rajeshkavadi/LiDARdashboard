#!/usr/bin/env python3
"""Synthetic end-to-end feed for testing the web app without ROS.

Sends:
  * LVP1 live frames to 127.0.0.1:8765
  * incremental LVM1 tiles/pose/trajectory to 127.0.0.1:8766
  * LVD1 DEM patches and status to the loopback internal HTTP API

This is a test harness only.  It is never started by the application and is
clearly not SLAM: trajectories are analytic, not estimated.
"""

import argparse
import json
import math
import os
import random
import socket
import struct
import sys
import threading
import time
import urllib.request

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "webapp"))
import protocol  # noqa: E402

LIVE_HEADER = struct.Struct("<4sIdIH2x")


class Feed(object):
    def __init__(self, host, live_port, map_port, http_port):
        self.host = host
        self.live_port = live_port
        self.map_port = map_port
        self.http_port = http_port
        self.stop = threading.Event()
        self.traj_index = 0
        self.pose_step = 0
        self.batch = 0

    def connect_map(self):
        sock = socket.create_connection((self.host, self.map_port))
        hello = json.dumps({
            "role": "synthetic-test",
            "fast_lio": "not-real (test harness)",
            "tile_size_m": 20.0,
            "voxel_size_m": 0.15,
        }).encode("utf-8")
        sock.sendall(b"LVS1" + struct.pack("<I", len(hello)) + hello)
        threading.Thread(target=self._drain, args=(sock,), daemon=True).start()
        return sock

    @staticmethod
    def _drain(sock):
        try:
            while True:
                if not sock.recv(4096):
                    break
        except OSError:
            pass

    def live_loop(self):
        sock = None
        seq = 0
        rng = np.random.RandomState(7)
        while not self.stop.is_set():
            if sock is None:
                try:
                    sock = socket.create_connection((self.host, self.live_port))
                except OSError:
                    self.stop.wait(1.0)
                    continue
            seq += 1
            count = 6000
            points = np.empty((count, 4), dtype="<f4")
            points[:, 0] = rng.uniform(-25, 25, count)
            points[:, 1] = rng.uniform(-25, 25, count)
            points[:, 2] = rng.uniform(-1.0, 3.0, count)
            points[:, 3] = rng.uniform(0, 255, count)
            head = LIVE_HEADER.pack(b"LVP1", count, time.time(), seq, 0)
            try:
                sock.sendall(head + points.tobytes())
            except OSError:
                try:
                    sock.close()
                except OSError:
                    pass
                sock = None
            self.stop.wait(0.1)
        if sock is not None:
            sock.close()

    def map_loop(self):
        sock = None
        rng = np.random.RandomState(11)
        tiles = [(tx, ty) for tx in range(-2, 3) for ty in range(-2, 3)]
        while not self.stop.is_set():
            if sock is None:
                try:
                    sock = self.connect_map()
                except OSError:
                    self.stop.wait(1.0)
                    continue
            # Add a few new points to every tile, like incremental voxel updates.
            for tx, ty in tiles:
                count = 120
                pts = np.empty((count, 4), dtype="<f4")
                pts[:, 0] = rng.uniform(tx * 20, (tx + 1) * 20, count)
                pts[:, 1] = rng.uniform(ty * 20, (ty + 1) * 20, count)
                pts[:, 2] = rng.normal(0.0, 0.8, count)
                pts[:, 3] = rng.uniform(0, 255, count)
                try:
                    sock.sendall(protocol.map_tile(
                        tx, ty, 0, pts.tobytes()))
                except OSError:
                    sock = None
                    break
            if sock is None:
                continue

            t = time.time()
            x = 25 * math.sin(self.pose_step * 0.12)
            y = 25 * (1 - math.cos(self.pose_step * 0.12))
            yaw = self.pose_step * 0.12
            qz = math.sin(yaw / 2)
            qw = math.cos(yaw / 2)
            try:
                sock.sendall(protocol.map_pose(t, x, y, 0.0, 0.0, 0.0, qz, qw))
                traj = struct.pack("<3d", x, y, 0.0)
                sock.sendall(protocol.map_trajectory(self.traj_index, traj))
                self.traj_index += 1
            except OSError:
                sock = None
                continue
            self.pose_step += 1
            self.stop.wait(0.5)

    def dem_loop(self):
        width = height = 160
        resolution = 0.5
        origin_x = origin_y = -40.0
        xs = np.arange(width) * resolution + origin_x
        ys = np.arange(height) * resolution + origin_y
        gx, gy = np.meshgrid(xs, ys)
        terrain = (2.0 + 1.5 * np.sin(gx / 8.0) +
                   0.8 * np.cos(gy / 6.0)).astype("<f4")
        full = protocol.dem_patch(
            origin_x, origin_y, resolution, width, height,
            0, 0, width, height, -9999.0, terrain.tobytes(), full=True)
        self._post("/internal/dem/patch", full, binary=True)
        rng = random.Random(3)
        while not self.stop.is_set():
            self.batch += 1
            x0 = rng.randrange(0, width - 16)
            y0 = rng.randrange(0, height - 16)
            w = h = 16
            patch = (terrain[y0:y0 + h, x0:x0 + w] +
                     rng.uniform(-0.1, 0.1)).astype("<f4")
            msg = protocol.dem_patch(
                origin_x, origin_y, resolution, width, height,
                x0, y0, w, h, -9999.0, patch.tobytes())
            self._post("/internal/dem/patch", msg, binary=True)
            self._post("/internal/dem/status", json.dumps({
                "status": "processing",
                "coordinate_mode": "LOCALIZED",
                "resolution": resolution,
                "width": width,
                "height": height,
                "origin": [origin_x, origin_y],
                "min_elevation": float(np.min(terrain)),
                "max_elevation": float(np.max(terrain)),
                "ground_points": 42000 + self.batch * 37,
                "tiles_completed": 12 + self.batch // 4,
                "batches": self.batch,
                "pdal_version": "synthetic-test",
                "updated_at": time.time(),
            }).encode("utf-8"), binary=False)
            self.stop.wait(0.7)

    def _post(self, path, body, binary):
        headers = {"Content-Type":
                   "application/octet-stream" if binary else "application/json"}
        req = urllib.request.Request(
            "http://%s:%d%s" % (self.host, self.http_port, path),
            data=body, headers=headers)
        try:
            urllib.request.urlopen(req, timeout=3).read()
        except Exception:
            pass

    def run(self):
        threads = [
            threading.Thread(target=self.live_loop, daemon=True),
            threading.Thread(target=self.map_loop, daemon=True),
            threading.Thread(target=self.dem_loop, daemon=True),
        ]
        for thread in threads:
            thread.start()
        try:
            while not self.stop.wait(1.0):
                pass
        except KeyboardInterrupt:
            self.stop.set()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--live-port", type=int, default=8765)
    parser.add_argument("--map-port", type=int, default=8766)
    parser.add_argument("--http-port", type=int, default=5000)
    parser.add_argument("--seconds", type=float, default=120)
    args = parser.parse_args()
    feed = Feed(args.host, args.live_port, args.map_port, args.http_port)
    threading.Thread(target=feed.run, daemon=True).start()
    time.sleep(args.seconds)
    feed.stop.set()
    print("synthetic feed stopped")


if __name__ == "__main__":
    main()
