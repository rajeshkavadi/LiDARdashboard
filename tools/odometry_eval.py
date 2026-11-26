#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Offline bag evaluation for the built-in ICP odometry (slam_core.py).

Runs on Python 2.7 with ROS Melodic (uses ``rosbag`` only; no roscore, no
launch files).  It reads a Livox PointCloud2 bag, feeds each frame to
``IcpOdometry`` exactly like the ROS node does, and prints:

* processing rate (frames/s) and per-frame timing,
* distance travelled and final displacement,
* the same statistics for /gnss_pose when the bag contains it (reference only;
  NOT used by the odometry).

Usage:
    source /opt/ros/melodic/setup.bash
    python2 tools/odometry_eval.py BAG --topic /mid360/livox/lidar [--frames 100]
"""

import argparse
import math
import os
import sys
import time

import numpy as np
import rosbag

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "slam_ws", "src",
                                "lidar_slam_bridge", "scripts"))
import slam_core  # noqa: E402


def cloud_to_xyz(msg):
    """(N, 3) float64 points and (N,) intensity from a PointCloud2 message."""
    fields = dict((f.name, f) for f in msg.fields)
    if not all(name in fields for name in ("x", "y", "z")):
        return None, None
    names = [n for n in ("x", "y", "z", "intensity") if n in fields]
    dtype = np.dtype({
        "names": names,
        "formats": ["<f4" for _ in names],
        "offsets": [fields[n].offset for n in names],
        "itemsize": msg.point_step,
    })
    count = msg.width * msg.height
    if count <= 0:
        return None, None
    cloud = np.frombuffer(msg.data, dtype=dtype, count=count)
    xyz = np.vstack([cloud["x"], cloud["y"], cloud["z"]]).T.astype(np.float64)
    if "intensity" in names:
        intensity = cloud["intensity"].astype(np.float64)
    else:
        intensity = None
    return xyz, intensity


def path_length(points):
    total = 0.0
    for i in range(1, len(points)):
        total += math.sqrt(sum((points[i][k] - points[i - 1][k]) ** 2
                               for k in range(3)))
    return total


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--topic", default="/mid360/livox/lidar")
    parser.add_argument("--frames", type=int, default=0,
                        help="0 = all LiDAR frames")
    parser.add_argument("--voxel", type=float, default=0.35)
    parser.add_argument("--map-voxel", type=float, default=0.5)
    parser.add_argument("--iterations", type=int, default=12)
    args = parser.parse_args()
    if not os.path.isfile(args.bag):
        print("bag not found: %s" % args.bag)
        return 1

    odom = slam_core.IcpOdometry(voxel_size=args.voxel,
                                 map_voxel=args.map_voxel,
                                 max_iterations=args.iterations)
    bag = rosbag.Bag(args.bag)
    try:
        # Reference trajectory (test statistics only, never fed to the odometry).
        gh = []
        for _, msg, _ in bag.read_messages(topics=["/gnss_pose"]):
            p = msg.pose.position
            gh.append((p.x, p.y, p.z))
        if not gh:
            gh = None

        frames = 0
        started = time.time()
        first_pos = None
        last_pos = None
        for _, msg, _ in bag.read_messages(topics=[args.topic]):
            xyz, intensity = cloud_to_xyz(msg)
            if xyz is None:
                print("cloud has no x/y/z fields")
                return 1
            t0 = time.time()
            pose, registered, _, ratio = odom.update(xyz, intensity)
            elapsed = time.time() - t0
            frames += 1
            position = pose[:3, 3]
            if first_pos is None:
                first_pos = position.copy()
            last_pos = position.copy()
            if frames % 10 == 0 or frames == 1:
                print("frame %4d  %6.1f ms  inliers %.2f  map %6d  "
                      "pose (%7.2f, %7.2f, %7.2f)"
                      % (frames, elapsed * 1000.0, ratio, len(odom.map),
                         position[0], position[1], position[2]))
            if args.frames and frames >= args.frames:
                break
        total = time.time() - started
    finally:
        bag.close()

    if frames == 0:
        print("no LiDAR messages on %s" % args.topic)
        return 1

    ours = [tuple(p[:3, 3]) for p in odom.trajectory]
    displacement = (last_pos - first_pos) if first_pos is not None else None
    print("")
    print("frames           : %d" % frames)
    print("wall time        : %.1f s  (%.2f frames/s)"
          % (total, frames / total if total else 0.0))
    print("rejected frames  : %d" % odom.rejected)
    print("map voxels       : %d" % len(odom.map))
    print("path length      : %.2f m" % path_length(ours))
    if displacement is not None:
        print("net displacement : %.2f m" % math.sqrt(sum(
            v * v for v in displacement)))
    if gh:
        print("gnss path length : %.2f m (reference)"
              % path_length(gh))
        print("gnss disp        : %.2f m (reference)" % math.sqrt(sum(
            (gh[-1][k] - gh[0][k]) ** 2 for k in range(3))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
