"""Incremental global-map store fed by the FAST-LIO2 web exporter.

The exporter sends only *new* voxel points per spatial tile.  This store keeps
those increments grouped by tile, bounds total memory, and gives each browser a
snapshot on connect followed by a bounded event log.  A client that falls
further behind than the log is resynchronised from a snapshot instead of being
allowed to build an unbounded send queue.
"""

import collections
import math
import struct
import threading
import time

import protocol


class MapHub(object):
    def __init__(self, max_points=6000000, max_tiles=20000,
                 event_log_size=4096):
        self._cond = threading.Condition()
        self.max_points = int(max_points)
        self.max_tiles = int(max_tiles)
        self._tiles = {}            # (tx,ty,tz) -> {"chunks": [bytes], "count": n}
        self._points = 0
        self._dropped_points = 0
        self._trajectory = []       # list of packed (f64 x,y,z) chunks
        self._traj_points = 0
        self._traj_length = 0.0
        self._last_position = None
        self._pose = None           # encoded pose message
        self._pose_values = None
        self._pose_at = 0.0
        self._events = collections.deque()
        self._event_seq = 0
        self._reset_count = 0
        self._slam_connected = False
        self._slam_peer = None
        self._slam_meta = {}
        self._updated_at = 0.0

    # -- ingest ---------------------------------------------------------
    def append_tile(self, tx, ty, tz, count, points_bytes):
        key = (int(tx), int(ty), int(tz))
        if count <= 0 or len(points_bytes) != count * protocol.POINT_BYTES:
            return False
        with self._cond:
            tile = self._tiles.get(key)
            if tile is None:
                if len(self._tiles) >= self.max_tiles:
                    self._dropped_points += count
                    return False
                tile = {"chunks": [], "count": 0}
                self._tiles[key] = tile
            if self._points + count > self.max_points:
                self._dropped_points += count
                return False
            tile["chunks"].append(points_bytes)
            tile["count"] += count
            self._points += count
            self._updated_at = time.time()
            self._record(protocol.map_tile(tx, ty, tz, points_bytes))
            return True

    def append_trajectory(self, start_index, count, xyz_bytes):
        if count <= 0 or len(xyz_bytes) != count * protocol.TRAJ_POINT_BYTES:
            return False
        with self._cond:
            if start_index != self._traj_points:
                # Duplicate or out-of-order segment: ignore rather than
                # creating a disconnected line.
                return False
            self._trajectory.append(xyz_bytes)
            self._traj_points += count
            view = memoryview(xyz_bytes)
            for i in range(count):
                x, y, z = struct.unpack_from("<3d", view, i * 24)
                if self._last_position is not None:
                    dx = x - self._last_position[0]
                    dy = y - self._last_position[1]
                    dz = z - self._last_position[2]
                    self._traj_length += math.sqrt(dx * dx + dy * dy + dz * dz)
                self._last_position = (x, y, z)
            self._updated_at = time.time()
            self._record(protocol.map_trajectory(start_index, xyz_bytes))
            return True

    def set_pose(self, values):
        encoded = protocol.map_pose(*values)
        with self._cond:
            self._pose = encoded
            self._pose_values = values
            self._pose_at = time.time()
            self._updated_at = self._pose_at
            self._record(encoded)

    def set_slam_connection(self, connected, peer=None, meta=None):
        with self._cond:
            self._slam_connected = bool(connected)
            self._slam_peer = peer if connected else None
            if meta:
                self._slam_meta.update(meta)
            if not connected:
                self._slam_meta["disconnected_at"] = time.time()
            self._cond.notify_all()

    def reset(self):
        with self._cond:
            self._tiles = {}
            self._points = 0
            self._trajectory = []
            self._traj_points = 0
            self._traj_length = 0.0
            self._last_position = None
            self._pose = None
            self._pose_values = None
            self._pose_at = 0.0
            self._reset_count += 1
            self._updated_at = time.time()
            self._record(protocol.map_reset())
            self._cond.notify_all()

    # -- event log ------------------------------------------------------
    def _record(self, message):
        self._event_seq += 1
        self._events.append((self._event_seq, message))
        self._cond.notify_all()

    def wait_for_updates(self, last_seq, timeout=1.0):
        """Return ``(messages, new_seq, resync)`` for one browser client."""
        with self._cond:
            if self._event_seq <= last_seq:
                self._cond.wait(timeout)
            if self._event_seq <= last_seq:
                return [], last_seq, False
            if self._events and last_seq + 1 < self._events[0][0]:
                return [], last_seq, True
            messages = [msg for seq, msg in self._events if seq > last_seq]
            return messages, self._event_seq, False

    def snapshot(self):
        """Return ``(messages, seq)`` describing the complete current map.

        A leading RESET makes the snapshot idempotent for a reconnecting
        client that may still hold tiles from an earlier session.
        """
        with self._cond:
            messages = [protocol.map_reset()]
            for (tx, ty, tz), tile in self._tiles.items():
                for chunk in tile["chunks"]:
                    messages.append(protocol.map_tile(tx, ty, tz, chunk))
            messages.extend(self._trajectory_messages())
            if self._pose is not None:
                messages.append(self._pose)
            return messages, self._event_seq

    def _trajectory_messages(self, chunk_points=4096):
        messages = []
        start = 0
        for chunk in self._trajectory:
            count = len(chunk) // protocol.TRAJ_POINT_BYTES
            step = max(1, chunk_points)
            for offset in range(0, count, step):
                take = min(step, count - offset)
                messages.append(protocol.map_trajectory(
                    start + offset,
                    chunk[offset * protocol.TRAJ_POINT_BYTES:
                          (offset + take) * protocol.TRAJ_POINT_BYTES]))
            start += count
        return messages

    # -- status / export ------------------------------------------------
    def iter_tile_point_bytes(self):
        with self._cond:
            chunks = [chunk for tile in self._tiles.values()
                      for chunk in tile["chunks"]]
        for chunk in chunks:
            yield chunk

    def stats(self):
        now = time.time()
        with self._cond:
            pose_age = (now - self._pose_at) if self._pose_at else None
            pose = None
            if self._pose_values:
                values = self._pose_values
                pose = {
                    "stamp": values[0],
                    "x": values[1], "y": values[2], "z": values[3],
                    "qx": values[4], "qy": values[5],
                    "qz": values[6], "qw": values[7],
                }
            return {
                "points": self._points,
                "tiles": len(self._tiles),
                "trajectory_points": self._traj_points,
                "trajectory_length_m": round(self._traj_length, 3),
                "pose": pose,
                "pose_age_s": round(pose_age, 3) if pose_age is not None else None,
                "slam_connected": self._slam_connected,
                "slam_peer": self._slam_peer,
                "slam_meta": dict(self._slam_meta),
                "dropped_points": self._dropped_points,
                "max_points": self.max_points,
                "max_tiles": self.max_tiles,
                "reset_count": self._reset_count,
                "updated_age_s": (round(now - self._updated_at, 3)
                                  if self._updated_at else None),
            }
