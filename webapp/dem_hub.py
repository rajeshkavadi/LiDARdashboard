"""DEM raster store.

The DEM service (a separate process) owns PDAL processing.  It sends complete
grids when the extent/resolution changes and small dirty rectangles otherwise.
The backend keeps the authoritative copy, serves it to browsers and exposes it
for export.
"""

import collections
import threading
import time

import numpy as np

import protocol


class DemHub(object):
    def __init__(self, max_cells=4194304, event_log_size=1024):
        self._cond = threading.Condition()
        self.max_cells = int(max_cells)
        self._grid = None
        self._meta = None
        self._status = {
            "status": "starting",
            "coordinate_mode": "LOCALIZED",
            "resolution": 0.0,
            "width": 0,
            "height": 0,
            "origin": None,
            "min_elevation": None,
            "max_elevation": None,
            "ground_points": 0,
            "tiles_completed": 0,
            "batches": 0,
            "pdal_version": None,
            "error": None,
            "updated_at": None,
        }
        self._events = collections.deque()
        self._event_seq = 0
        self._reset_count = 0

    # -- ingest ---------------------------------------------------------
    def apply_message(self, data):
        msg_type, meta, heights = protocol.parse_dem_message(data)
        if msg_type is None:
            return False
        if msg_type == protocol.DEM_RESET:
            self.reset(broadcast=True)
            return True
        count = meta["w"] * meta["h"]
        if count <= 0:
            return False
        if meta["width"] * meta["height"] > self.max_cells:
            self._status["error"] = (
                "DEM grid exceeds max cells (%d)" % self.max_cells)
            return False
        values = np.frombuffer(heights, dtype="<f4").reshape(meta["h"], meta["w"])
        with self._cond:
            if msg_type == protocol.DEM_FULL:
                if meta["width"] > 0 and meta["height"] > 0:
                    self._grid = np.full(
                        (meta["height"], meta["width"]), np.nan, dtype=np.float32)
                    self._meta = meta
                    self._grid[:] = values
                    self._record(data)
                    self._cond.notify_all()
                    return True
                return False
            # PATCH
            if (self._grid is None or self._meta is None or
                    self._grid.shape != (meta["height"], meta["width"]) or
                    meta["x0"] < 0 or meta["y0"] < 0 or
                    meta["x0"] + meta["w"] > meta["width"] or
                    meta["y0"] + meta["h"] > meta["height"]):
                return False
            self._grid[meta["y0"]:meta["y0"] + meta["h"],
                       meta["x0"]:meta["x0"] + meta["w"]] = values
            self._record(data)
            self._cond.notify_all()
            return True

    def update_status(self, status):
        with self._cond:
            for key in ("status", "coordinate_mode", "resolution", "width",
                        "height", "ground_points", "tiles_completed", "batches",
                        "pdal_version", "error", "min_elevation",
                        "max_elevation"):
                if key in status:
                    self._status[key] = status[key]
            if "origin" in status:
                self._status["origin"] = status["origin"]
            self._status["updated_at"] = status.get("updated_at", time.time())
            self._cond.notify_all()

    def reset(self, broadcast=True):
        with self._cond:
            self._grid = None
            self._meta = None
            self._reset_count += 1
            if broadcast:
                self._record(protocol.dem_reset())
            self._cond.notify_all()

    # -- fan-out --------------------------------------------------------
    def _record(self, message):
        self._event_seq += 1
        self._events.append((self._event_seq, message))
        self._cond.notify_all()

    def wait_for_updates(self, last_seq, timeout=1.0):
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
        with self._cond:
            messages = [protocol.dem_reset()]
            if self._grid is not None and self._meta is not None:
                meta = self._meta
                messages.append(protocol.dem_patch(
                    meta["origin_x"], meta["origin_y"], meta["resolution"],
                    meta["width"], meta["height"], 0, 0,
                    meta["width"], meta["height"], meta["nodata"],
                    self._grid.astype("<f4", copy=False).tobytes(), full=True))
            return messages, self._event_seq

    def stats(self):
        with self._cond:
            status = dict(self._status)
            if self._grid is not None and np.isfinite(self._grid).any():
                status["min_elevation"] = float(np.nanmin(self._grid))
                status["max_elevation"] = float(np.nanmax(self._grid))
            if self._meta:
                status["width"] = self._meta["width"]
                status["height"] = self._meta["height"]
                status["resolution"] = self._meta["resolution"]
                status["origin"] = [self._meta["origin_x"],
                                    self._meta["origin_y"]]
                status["nodata"] = self._meta["nodata"]
            status["has_grid"] = self._grid is not None
            status["reset_count"] = self._reset_count
            return status

    def grid_copy(self):
        with self._cond:
            if self._grid is None or self._meta is None:
                return None, None
            return self._grid.copy(), dict(self._meta)
