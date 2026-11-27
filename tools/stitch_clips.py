#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Stitch a sequence of short bag clips into ONE continuous bag.

Why
---
The edge recorder writes the session as a series of 10 s clips.  Replaying
them with one ``rosbag play`` per clip is *not* the same as replaying a single
long bag:

* the SLAM node sees the LiDAR publisher disappear and come back every 10 s,
* every clip restart adds a pause (a few hundred ms of no data at all),
* hall of the recorded timeline is dead time between clips (the recorder stops
  the bag and starts a new one, so ~2 s of every 10 s is lost),
* an IMU-based backend (FAST-LIO, Point-LIO) is integrated across that dead
  gap, which throws the state estimate away and smears the map.

This tool rewrites the clips into one bag on a continuous, strictly increasing
timeline: each clip keeps its own internal timing, and the dead time between
two clips is replaced by a small configurable seam (``--gap``, default 0.02 s).
Every header stamp -- and the Livox per-point ``timestamp`` field -- is shifted
by the same amount, so scan deskewing and IMU fusion stay consistent.

The result behaves like the single long bag the sensor would have produced if
the recorder had never stopped.

Usage
-----
    python2 tools/stitch_clips.py CLIP [CLIP ...]
    python2 tools/stitch_clips.py --cache-dir /var/tmp/stitch CLIP [CLIP ...]
    python2 tools/stitch_clips.py --out /tmp/session.bag --gap 0 CLIP [CLIP ...]

The merged bag path is printed on stdout (and only that), progress and warnings
go to stderr, so it can be used as ``MERGED=$(stitch_clips.py ...)``.
An existing bag in the cache is reused unless ``--force`` is given.
"""

import argparse
import hashlib
import os
import sys

import numpy as np
import rosbag
import rospy

VERSION = 2
DEFAULT_CACHE = os.environ.get("LIDAR_STITCH_CACHE", "/tmp/lidar_stitch")


def warn(message):
    sys.stderr.write("stitch_clips: %s\n" % message)
    sys.stderr.flush()


def info(message):
    sys.stderr.write("stitch_clips: %s\n" % message)
    sys.stderr.flush()


def cache_key(paths, gap):
    """Stable key for a clip set: names, sizes, mtimes, seam and tool version."""
    digest = hashlib.sha1()
    digest.update("v%d|gap=%.6f|" % (VERSION, gap))
    for path in paths:
        try:
            stat = os.stat(path)
            stamp = "%d.%d" % (stat.st_size, int(stat.st_mtime))
        except OSError:
            stamp = "missing"
        digest.update("%s|%s|" % (os.path.abspath(path), stamp))
    return digest.hexdigest()[:16]


def shift_header(msg, shift):
    """Move every absolute time inside one message forward by ``shift`` sec."""
    header = getattr(msg, "header", None)
    if header is not None:
        header.stamp = header.stamp + rospy.Duration.from_sec(shift)

    # livox_ros_driver/CustomMsg: absolute ns timebase + per-point offsets.
    timebase = getattr(msg, "timebase", None)
    if isinstance(timebase, (int, long)):
        msg.timebase = timebase + long(round(shift * 1e9))

    if msg._type != "sensor_msgs/PointCloud2":
        return
    _shift_point_times(msg, shift)


def _shift_point_times(msg, shift):
    """Shift a float64 ``timestamp`` field (absolute ns) in a PointCloud2."""
    field = None
    for candidate in msg.fields:
        if (candidate.name == "timestamp" and candidate.datatype == 8
                and candidate.count == 1):
            field = candidate
            break
    if field is None:
        return
    count = msg.width * msg.height
    if count <= 0 or not msg.data:
        return
    dtype = np.dtype({"names": ["t"], "formats": ["<f8"],
                      "offsets": [field.offset], "itemsize": msg.point_step})
    if field.offset + 8 > msg.point_step:
        return
    delta = shift * 1e9
    values = np.frombuffer(msg.data, dtype=dtype, count=count)["t"].copy()
    values += delta
    buffer = bytearray(msg.data)
    np.frombuffer(buffer, dtype=dtype, count=count)["t"][:] = values
    msg.data = bytes(buffer)


def stitch(paths, out_path, gap):
    """Write every message of every clip into ``out_path`` on one timeline."""
    written = 0
    total = 0
    cursor = None          # bag time the next clip's first message should land on
    previous_end = None    # last message time of the previous clip, original
    total_clips = len(paths)

    bag = rosbag.Bag(out_path, "w")
    try:
        for index, path in enumerate(paths):
            source = rosbag.Bag(path)
            try:
                start = source.get_start_time()
                end = source.get_end_time()
                messages = 0
                if end > start:
                    shift = 0.0 if cursor is None else cursor - start
                    offset = rospy.Duration.from_sec(shift)
                    for topic, message, stamp in source.read_messages():
                        shift_header(message, shift)
                        bag.write(topic, message, stamp + offset)
                        messages += 1
                        written += 1
            finally:
                source.close()
            if not messages:
                warn("clip %d/%d %s is empty; skipped"
                     % (index + 1, total_clips, os.path.basename(path)))
                continue
            dead = 0.0 if previous_end is None else start - previous_end
            previous_end = end
            cursor = (end + shift) + gap
            total += (end - start)
            info("clip %d/%d %s -> %5d msgs, %.1f s, dead time %.2f s -> %.2f s"
                 % (index + 1, total_clips, os.path.basename(path), messages,
                    end - start, dead, gap))
    finally:
        bag.close()
    return written, total


def main(argv):
    parser = argparse.ArgumentParser(
        description="Stitch short bag clips into one continuous bag.")
    parser.add_argument("clips", nargs="+", help="clip bags, in playback order")
    parser.add_argument("--out", help="output bag (default: cache/<key>.bag)")
    parser.add_argument("--cache-dir", default=DEFAULT_CACHE,
                        help="cache directory (default: %s)" % DEFAULT_CACHE)
    parser.add_argument("--gap", type=float, default=0.02,
                        help="seam length in seconds between two clips "
                             "(default: 0.02, 0 = back to back)")
    parser.add_argument("--force", action="store_true",
                        help="rebuild even when the cached bag exists")
    args = parser.parse_args(argv)

    missing = [path for path in args.clips if not os.path.isfile(path)]
    if missing:
        warn("not found: %s" % ", ".join(missing))
        return 2
    if args.gap < 0.0:
        warn("--gap must not be negative")
        return 2

    if args.out:
        out_path = os.path.abspath(os.path.expanduser(args.out))
    else:
        cache_dir = os.path.expanduser(args.cache_dir)
        if not os.path.isdir(cache_dir):
            try:
                os.makedirs(cache_dir)
            except OSError as exc:
                warn("cannot create cache dir %s (%s)" % (cache_dir, exc))
                return 2
        out_path = os.path.join(cache_dir, "%s.bag" % cache_key(args.clips,
                                                               args.gap))

    if os.path.isfile(out_path) and not args.force:
        info("reusing %s" % out_path)
        print(out_path)
        return 0

    needed = sum(os.path.getsize(path) for path in args.clips)
    try:
        stat = os.statvfs(os.path.dirname(out_path))
        free = stat.f_bavail * stat.f_frsize
    except OSError:
        free = None
    if free is not None and free < needed:
        warn("not enough free space in %s: need %.1f GB, have %.1f GB"
             % (os.path.dirname(out_path), needed / 1e9, free / 1e9))
        return 3

    partial = out_path + ".part"
    if os.path.isfile(partial):
        os.unlink(partial)
    info("stitching %d clips (%.2f GB) -> %s"
         % (len(args.clips), needed / 1e9, out_path))
    try:
        written, duration = stitch(args.clips, partial, args.gap)
    except (rosbag.ROSBagException, IOError, OSError) as exc:
        warn("stitch failed: %s" % exc)
        if os.path.isfile(partial):
            os.unlink(partial)
        return 1
    if not written:
        warn("no messages found in the given clips")
        if os.path.isfile(partial):
            os.unlink(partial)
        return 1
    if os.path.isfile(out_path):
        os.unlink(out_path)
    os.rename(partial, out_path)
    info("done: %d messages, %.1f s of data, %.2f GB"
         % (written, duration, os.path.getsize(out_path) / 1e9))
    print(out_path)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main(sys.argv[1:]))
    except KeyboardInterrupt:
        sys.exit(130)
