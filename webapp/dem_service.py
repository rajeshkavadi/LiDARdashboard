#!/usr/bin/env python3
"""Incremental DEM service for the LiDAR dashboard.

The service reads the globally transformed point cloud that the SLAM pipeline
already produced (same stream the browser gets on /ws/map) and keeps a local
terrain grid up to date.  It is a separate process, so it can never block LiDAR
acquisition, SLAM, live rendering or WebSocket delivery.

Pipeline per region:
    accumulate points in 20 m spatial tiles
    -> classify ground (PDAL filters.smrf when available, otherwise a numpy
       morphological filter)
    -> commit only the centre tile (the 3x3 neighbourhood is used as a halo, so
       tile borders classify consistently)
    -> aggregate the lowest ground return per DEM cell
    -> send a FULL grid on extent change, PATCH rectangles otherwise

Coordinates are LOCAL metres in the SLAM map frame.  No CRS, no GPS, no
latitude/longitude is created anywhere in this file.

Run:
    python3 webapp/dem_service.py --resolution 0.5
"""

import argparse
import glob
import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
import urllib.error
import urllib.request

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import protocol  # noqa: E402

NODATA = -9999.0
PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---------------------------------------------------------------------------
# Ground classification
# ---------------------------------------------------------------------------
def find_pdal():
    """Locate a usable pdal binary (PATH first, then the vendored copy)."""
    for path in os.environ.get("LIDAR_PDAL", "").split(os.pathsep):
        if path and os.path.isfile(path) and os.access(path, os.X_OK):
            return path
    for candidate in ("pdal",):
        for directory in os.environ.get("PATH", "").split(os.pathsep):
            full = os.path.join(directory, candidate)
            if os.path.isfile(full) and os.access(full, os.X_OK):
                return full
    vendored = os.path.join(PROJECT_ROOT, "vendor", "pdal", "usr", "bin",
                            "pdal")
    if os.path.isfile(vendored) and os.access(vendored, os.X_OK):
        return vendored
    return None


def pdal_env(pdal_path):
    """Environment for the vendored PDAL build (it ships its own libs)."""
    env = dict(os.environ)
    root = os.path.abspath(os.path.join(os.path.dirname(pdal_path), ".."))
    libs = [os.path.join(root, "lib"), os.path.join(root, "lib",
                                                   "x86_64-linux-gnu")]
    existing = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = ":".join(
        [p for p in libs + [existing] if p])
    return env


class GroundClassifier(object):
    """PDAL SMRF with a numpy fallback; never raises into the pipeline."""

    def __init__(self, pdal_bin=None, cell=1.0, slope=0.15, threshold=0.3,
                 window=18.0):
        self.pdal = pdal_bin if pdal_bin is not None else find_pdal()
        self.cell = float(cell)
        self.slope = float(slope)
        self.threshold = float(threshold)
        self.window = float(window)
        self.version = "numpy-fallback"
        self.last_error = None
        self.pdal_failures = 0
        if self.pdal:
            try:
                output = subprocess.check_output(
                    [self.pdal, "--version"], stderr=subprocess.STDOUT,
                    env=pdal_env(self.pdal), timeout=20)
                text = output.decode("utf-8", "replace").splitlines()
                for line in text:
                    if line.strip().startswith("pdal"):
                        self.version = line.strip()
                        break
            except Exception as exc:  # noqa: BLE001 - keep the fallback
                self.last_error = "pdal unavailable: %s" % exc
                self.pdal = None

    # -- public -----------------------------------------------------------
    def classify(self, points):
        """Return a boolean mask of ground points for an (N, 3) array."""
        if points.shape[0] < 30:
            return np.zeros(points.shape[0], dtype=bool)
        if self.pdal and self.pdal_failures < 5:
            mask = self._pdal_mask(points)
            if mask is not None:
                return mask
        return self._numpy_mask(points)

    # -- pdal -------------------------------------------------------------
    def _pdal_mask(self, points):
        tempdir = tempfile.mkdtemp(prefix="lidar_dem_")
        src = os.path.join(tempdir, "points.ply")
        dst = os.path.join(tempdir, "ground.txt")
        pipeline_path = os.path.join(tempdir, "pipeline.json")
        try:
            _write_ply(src, points)
            pipeline = {
                "pipeline": [
                    {"type": "readers.ply", "filename": src},
                    {"type": "filters.smrf",
                     "scalar": 1.2,
                     "slope": self.slope,
                     "threshold": self.threshold,
                     "window": self.window,
                     "cell": self.cell},
                    {"type": "filters.range", "limits": "Classification[2:2]"},
                    {"type": "writers.text", "filename": dst,
                     "order": "X,Y,Z"},
                ]
            }
            with open(pipeline_path, "w") as handle:
                json.dump(pipeline, handle)
            subprocess.check_output(
                [self.pdal, "pipeline", pipeline_path],
                stderr=subprocess.STDOUT, env=pdal_env(self.pdal),
                timeout=120)
            ground = _read_xyz_text(dst)
            if ground is None or ground.size == 0:
                self.pdal_failures += 1
                return np.zeros(points.shape[0], dtype=bool)
            self.last_error = None
            return _match_ground(points, ground)
        except Exception as exc:  # noqa: BLE001 - fall back to numpy
            self.pdal_failures += 1
            self.last_error = "pdal failed: %s" % exc
            return None
        finally:
            for name in ("points.ply", "ground.txt", "pipeline.json"):
                try:
                    os.unlink(os.path.join(tempdir, name))
                except OSError:
                    pass
            try:
                os.rmdir(tempdir)
            except OSError:
                pass

    # -- numpy fallback ---------------------------------------------------
    def _numpy_mask(self, points):
        """Morphological ground filter (lowest surface + slope tolerance)."""
        cell = self.cell
        xy = points[:, :2]
        origin = xy.min(axis=0)
        indices = np.floor((xy - origin) / cell).astype(np.int64)
        width = int(indices[:, 0].max()) + 1
        height = int(indices[:, 1].max()) + 1
        if width * height > 4000000:
            # Too wide: coarsen the support grid to keep memory bounded.
            cell *= math.sqrt(width * height / 4000000.0)
            indices = np.floor((xy - origin) / cell).astype(np.int64)
            width = int(indices[:, 0].max()) + 1
            height = int(indices[:, 1].max()) + 1
        lowest = np.full((height, width), np.nan, dtype=np.float32)
        np.fmin.at(lowest, (indices[:, 1], indices[:, 0]),
                   points[:, 2].astype(np.float32))

        # Neighbourhood minimum: a point is ground if it is close to the lowest
        # surface of its surroundings, allowing a slope over one cell.
        padded = np.pad(lowest, 1, mode="constant", constant_values=np.nan)
        neighbour = lowest.copy()
        for dy in (0, 1, 2):
            for dx in (0, 1, 2):
                candidate = padded[dy:dy + height, dx:dx + width]
                neighbour = np.fmin(neighbour, candidate)
        allowed = neighbour + self.threshold + self.slope * cell
        limit = allowed[indices[:, 1], indices[:, 0]]
        mask = np.zeros(points.shape[0], dtype=bool)
        valid = np.isfinite(limit)
        mask[valid] = points[valid, 2] <= limit[valid]
        return mask


def _write_ply(path, points):
    """Write a binary little-endian PLY, the fastest lossless PLY for PDAL."""
    points = np.ascontiguousarray(points[:, :3], dtype="<f4")
    header = ("ply\nformat binary_little_endian 1.0\n"
              "element vertex %d\n"
              "property float x\nproperty float y\n"
              "property float z\nend_header\n" % points.shape[0])
    with open(path, "wb") as handle:
        handle.write(header.encode("ascii"))
        handle.write(points.tobytes())


def _write_points_text(handle, points, fmt):
    """Append numeric rows to a text handle.

    numpy 1.13 refuses ``savetxt`` on an open file object with a float format
    string, so the rows go through a temporary file and are copied across.
    """
    fd, temp = tempfile.mkstemp(prefix="lidar_dem_pts_")
    os.close(fd)
    try:
        np.savetxt(temp, np.ascontiguousarray(points[:, :3],
                                               dtype=np.float32), fmt=fmt)
        with open(temp, "r") as source:
            while True:
                chunk = source.read(1 << 20)
                if not chunk:
                    break
                handle.write(chunk)
    finally:
        try:
            os.unlink(temp)
        except OSError:
            pass


def _read_xyz_text(path):
    """Read the CSV produced by PDAL's writers.text (X,Y,Z)."""
    if not os.path.isfile(path):
        return None
    rows = []
    with open(path) as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith('"X"') or line.lower().startswith(
                    "x,y"):
                continue
            parts = line.replace('"', "").split(",")
            if len(parts) < 3:
                parts = line.split()
            if len(parts) < 3:
                continue
            try:
                rows.append((float(parts[0]), float(parts[1]), float(parts[2])))
            except ValueError:
                continue
    if not rows:
        return np.zeros((0, 3), dtype=np.float64)
    return np.asarray(rows, dtype=np.float64)


def _match_ground(points, ground, tolerance=0.01):
    """Boolean mask selecting ``points`` that appear in the classified set.

    PDAL only returns the surviving ground coordinates, so they are matched
    back by quantised position.  Both arrays originate from the same values
    (the text round-trip is accurate to 1 mm), so quantising at 1 cm is safe.
    """
    if ground.shape[0] == 0:
        return np.zeros(points.shape[0], dtype=bool)
    scale = 1.0 / max(tolerance, 1e-3)
    ground_keys = np.round(ground[:, :3] * scale).astype(np.int64)
    point_keys = np.round(points[:, :3] * scale).astype(np.int64)
    order = np.lexsort((ground_keys[:, 2], ground_keys[:, 1],
                        ground_keys[:, 0]))
    flat = np.ascontiguousarray(ground_keys[order]).view(
        [("x", np.int64), ("y", np.int64), ("z", np.int64)]).ravel()
    query = np.ascontiguousarray(point_keys).view(
        [("x", np.int64), ("y", np.int64), ("z", np.int64)]).ravel()
    index = np.searchsorted(flat, query)
    index = np.clip(index, 0, flat.size - 1)
    return flat[index] == query


# ---------------------------------------------------------------------------
# DEM grid
# ---------------------------------------------------------------------------
class DemGrid(object):
    """Sparse-in-Y, dense-in-memory elevation grid in LOCAL coordinates."""

    def __init__(self, resolution=0.5, origin_x=0.0, origin_y=0.0):
        self.extent_changed = False
        self.resolution = float(resolution)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.width = 0
        self.height = 0
        self.values = np.zeros((0, 0), dtype=np.float32)

    def _ensure(self, ix_min, ix_max, iy_min, iy_max):
        """Grow the grid (origin shifts west/south) to contain the indices."""
        changed = False
        pad = 32
        if self.width == 0:
            self.width = max(1, ix_max - ix_min + 1 + pad)
            self.height = max(1, iy_max - iy_min + 1 + pad)
            self.origin_x = self.origin_x + ix_min * self.resolution - pad // 2 \
                * self.resolution
            self.origin_y = self.origin_y + iy_min * self.resolution - pad // 2 \
                * self.resolution
            self.values = np.full((self.height, self.width), np.nan,
                                  dtype=np.float32)
            return True
        shift_x = max(0, -ix_min + 4)
        shift_y = max(0, -iy_min + 4)
        grow_x = max(0, ix_max - (self.width - 1) + 4)
        grow_y = max(0, iy_max - (self.height - 1) + 4)
        if shift_x or shift_y or grow_x or grow_y:
            new_width = self.width + shift_x + grow_x
            new_height = self.height + shift_y + grow_y
            new_values = np.full((new_height, new_width), np.nan,
                                 dtype=np.float32)
            new_values[shift_y:shift_y + self.height,
                       shift_x:shift_x + self.width] = self.values
            self.values = new_values
            self.width = new_width
            self.height = new_height
            self.origin_x -= shift_x * self.resolution
            self.origin_y -= shift_y * self.resolution
            changed = True
        return changed

    def update(self, points):
        """Aggregate the lowest ground return per cell. Returns dirty bbox."""
        if points.shape[0] == 0:
            return None
        ix = np.floor((points[:, 0] - self.origin_x) / self.resolution).astype(
            np.int64)
        iy = np.floor((points[:, 1] - self.origin_y) / self.resolution).astype(
            np.int64)
        changed = self._ensure(int(ix.min()), int(ix.max()),
                               int(iy.min()), int(iy.max()))
        ix = np.floor((points[:, 0] - self.origin_x) / self.resolution).astype(
            np.int64)
        iy = np.floor((points[:, 1] - self.origin_y) / self.resolution).astype(
            np.int64)
        valid = ((ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height))
        if not valid.any():
            return None
        ix = ix[valid]
        iy = iy[valid]
        z = points[valid, 2].astype(np.float32)
        np.fmin.at(self.values, (iy, ix), z)
        self.extent_changed = changed
        return (int(ix.min()), int(iy.min()), int(ix.max()), int(iy.max()))

    def stats(self):
        if self.values.size == 0 or not np.isfinite(self.values).any():
            return {"min_elevation": None, "max_elevation": None}
        return {"min_elevation": float(np.nanmin(self.values)),
                "max_elevation": float(np.nanmax(self.values))}

    def to_bytes(self):
        return np.ascontiguousarray(self.values, dtype="<f4").tobytes()


# ---------------------------------------------------------------------------
# Service
# ---------------------------------------------------------------------------
class DemService(object):
    def __init__(self, args):
        self.args = args
        self.grid = DemGrid(args.resolution, args.origin_x, args.origin_y)
        self.classifier = GroundClassifier(
            pdal_bin=args.pdal if args.pdal else None,
            cell=args.smrf_cell, slope=args.smrf_slope,
            threshold=args.smrf_threshold, window=args.smrf_window)
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.work = threading.Event()
        self.pending = {}            # tile key -> list of (N,3) arrays
        self.pending_touched = {}    # tile key -> last ingest time
        self.pending_points = 0
        self.processing = []         # queue of (key, points)
        self.processed_tiles = set()
        self.ground_points = 0
        self.batches = 0
        self.ground_buffer = []      # recent ground points for export
        self.ground_buffer_points = 0
        self.dirty = None
        self.need_full = True
        self.status = "starting"
        self.error = None
        self.started = time.time()
        self.last_export = 0.0
        self.last_broadcast = 0.0
        self.broadcast_lock = threading.Lock()

    # -- backend I/O ------------------------------------------------------
    def _post(self, path, body, content_type):
        request = urllib.request.Request(
            self.args.backend + path, data=body,
            headers={"Content-Type": content_type})
        try:
            urllib.request.urlopen(request, timeout=10).read()
            return 200
        except urllib.error.HTTPError as exc:
            return exc.code
        except Exception as exc:  # noqa: BLE001 - backend may be restarting
            self.error = "backend unreachable: %s" % exc
            return None

    def push_status(self):
        stats = self.grid.stats()
        payload = {
            "status": self.status,
            "coordinate_mode": "LOCALIZED",
            "resolution": self.grid.resolution,
            "width": self.grid.width,
            "height": self.grid.height,
            "origin": [self.grid.origin_x, self.grid.origin_y],
            "min_elevation": stats["min_elevation"],
            "max_elevation": stats["max_elevation"],
            "ground_points": self.ground_points,
            "tiles_completed": len(self.processed_tiles),
            "batches": self.batches,
            "pdal_version": self.classifier.version,
            "error": self.classifier.last_error or self.error,
            "updated_at": time.time(),
        }
        self._post("/internal/dem/status", json.dumps(payload).encode("utf-8"),
                   "application/json")

    def broadcast(self, force=False):
        with self.broadcast_lock:
            now = time.time()
            if not force and now - self.last_broadcast < 0.5:
                return
            self.last_broadcast = now
            with self.lock:
                if self.grid.width == 0:
                    return
                dirty = self.dirty
                self.dirty = None
                full = self.need_full
                self.need_full = False
                origin_x, origin_y = self.grid.origin_x, self.grid.origin_y
                resolution = self.grid.resolution
                width, height = self.grid.width, self.grid.height
                values = self.grid.values
            if full:
                body = protocol.dem_patch(
                    origin_x, origin_y, resolution, width, height,
                    0, 0, width, height, NODATA,
                    np.ascontiguousarray(values, dtype="<f4").tobytes(),
                    full=True)
                code = self._post("/internal/dem/patch", body,
                                  "application/octet-stream")
                if code != 200:
                    self.need_full = True
                return
            if dirty is None:
                return
            x0, y0, x1, y1 = dirty
            x0 = max(0, min(x0, width - 1))
            y0 = max(0, min(y0, height - 1))
            x1 = max(x0, min(x1, width - 1))
            y1 = max(y0, min(y1, height - 1))
            patch = np.ascontiguousarray(
                values[y0:y1 + 1, x0:x1 + 1], dtype="<f4")
            body = protocol.dem_patch(
                origin_x, origin_y, resolution, width, height,
                x0, y0, x1 - x0 + 1, y1 - y0 + 1, NODATA, patch.tobytes())
            code = self._post("/internal/dem/patch", body,
                              "application/octet-stream")
            if code != 200:
                # Backend restarted or grid shape mismatch: resend everything.
                self.need_full = True

    # -- map stream -------------------------------------------------------
    def stream_loop(self):
        decoder = protocol.MapStreamDecoder()
        while not self.stop.is_set():
            try:
                response = urllib.request.urlopen(
                    self.args.backend + "/internal/map/stream", timeout=15)
            except Exception as exc:  # noqa: BLE001
                self.error = "map stream unavailable: %s" % exc
                self.stop.wait(2.0)
                continue
            self.error = None
            try:
                while not self.stop.is_set():
                    chunk = response.read(65536)
                    if not chunk:
                        break
                    for message in decoder.feed(chunk):
                        self._handle_message(message)
            except Exception as exc:  # noqa: BLE001
                self.error = "map stream error: %s" % exc
            finally:
                try:
                    response.close()
                except Exception:  # noqa: BLE001
                    pass
            self.stop.wait(1.0)

    def _handle_message(self, message):
        msg_type, payload = protocol.parse_map_message(message)
        if msg_type == protocol.MAP_RESET:
            self.reset()
            return
        if msg_type != protocol.MAP_TILE_ADD:
            return
        parsed = protocol.parse_tile(payload)
        if parsed is None:
            return
        tx, ty, tz, count, body = parsed
        points = np.frombuffer(body, dtype="<f4").reshape(count, 4)[:, :3]
        points = points[np.isfinite(points).all(axis=1)]
        if points.shape[0] == 0:
            return
        self.ingest(points)

    def ingest(self, points):
        """Accumulate points per spatial tile, queue full tiles for processing."""
        tile = self.args.tile_size
        keys = np.floor(points[:, :2] / tile).astype(np.int64)
        with self.lock:
            for key in np.unique(keys, axis=0):
                mask = (keys[:, 0] == key[0]) & (keys[:, 1] == key[1])
                bucket_key = (int(key[0]), int(key[1]))
                self.pending.setdefault(bucket_key, []).append(points[mask])
                self.pending_touched[bucket_key] = time.time()
                self.pending_points += int(mask.sum())
            self._trim_pending()
            ready = []
            for bucket_key, chunks in list(self.pending.items()):
                total = sum(c.shape[0] for c in chunks)
                if total >= self.args.batch_points:
                    ready.append((bucket_key, np.vstack(chunks)))
                    del self.pending[bucket_key]
                    self.pending_points -= total
            self.processing.extend(ready)
        if ready:
            self.work.set()

    def flush_idle(self, min_points=1500, idle_s=8.0):
        """Process tiles that stopped growing, so no area waits for a batch."""
        now = time.time()
        ready = []
        with self.lock:
            for key, chunks in list(self.pending.items()):
                touched = self.pending_touched.get(key, 0.0)
                total = sum(c.shape[0] for c in chunks)
                if total < min_points or now - touched < idle_s:
                    continue
                if self.processing and self.processing[-1][0] == key:
                    continue
                ready.append((key, np.vstack(chunks)))
                del self.pending[key]
                self.pending_touched.pop(key, None)
                self.pending_points -= total
            self.processing.extend(ready)
        if ready:
            self.work.set()

    def _trim_pending(self):
        """Bound memory when the DEM cannot keep up with the sensor."""
        limit = self.args.max_pending_points
        while self.pending_points > limit and self.pending:
            key = next(iter(self.pending))
            self.pending_points -= sum(c.shape[0] for c in self.pending[key])
            del self.pending[key]
            self.pending_touched.pop(key, None)

    # -- processing -------------------------------------------------------
    def worker_loop(self):
        while not self.stop.is_set():
            if not self.work.wait(1.0):
                continue
            self.work.clear()
            while not self.stop.is_set():
                with self.lock:
                    if not self.processing:
                        break
                    key, points = self.processing.pop(0)
                try:
                    self.process_tile(key, points)
                except Exception as exc:  # noqa: BLE001 - keep the service up
                    self.error = "tile %s failed: %s" % (key, exc)
                    self.status = "error"

    def _halo_points(self, key):
        """Centre tile plus its 3x3 neighbourhood, used for classification."""
        tile = self.args.tile_size
        with self.lock:
            chunks = []
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    neighbour = (key[0] + dx, key[1] + dy)
                    for chunk in self.pending.get(neighbour, []):
                        chunks.append(chunk)
        if not chunks:
            return None
        halo = np.vstack(chunks)
        cap = self.args.max_halo_points
        if halo.shape[0] > cap:
            halo = halo[::int(math.ceil(halo.shape[0] / float(cap)))]
        return halo

    def process_tile(self, key, points):
        halo = self._halo_points(key)
        self.status = "processing"
        if halo is None:
            halo = points
        if halo.shape[0] < 30:
            return
        mask = self.classifier.classify(halo)
        ground = halo[mask]
        tile = self.args.tile_size
        centre = ((np.floor(ground[:, 0] / tile).astype(np.int64) == key[0]) &
                  (np.floor(ground[:, 1] / tile).astype(np.int64) == key[1]))
        ground = ground[centre]
        self.batches += 1
        self.processed_tiles.add(key)
        if ground.shape[0]:
            self.ground_points += ground.shape[0]
            self.ground_buffer.append(ground)
            self.ground_buffer_points += ground.shape[0]
            self._trim_ground_buffer()
            with self.lock:
                bbox = self.grid.update(ground)
                if bbox is None or self.grid.extent_changed:
                    self.need_full = True
                    self.dirty = None
                    self.status = "ready"
                else:
                    if self.dirty is None:
                        self.dirty = bbox
                    else:
                        self.dirty = (min(self.dirty[0], bbox[0]),
                                      min(self.dirty[1], bbox[1]),
                                      max(self.dirty[2], bbox[2]),
                                      max(self.dirty[3], bbox[3]))
                    if self.status != "error":
                        self.status = "ready"
        self.broadcast()
        self._maybe_export()

    def _trim_ground_buffer(self):
        limit = self.args.max_ground_points
        while (self.ground_buffer_points > limit and
               len(self.ground_buffer) > 1):
            removed = self.ground_buffer.pop(0)
            self.ground_buffer_points -= removed.shape[0]

    # -- exports ----------------------------------------------------------
    def _maybe_export(self):
        if self.args.export_interval <= 0:
            return
        now = time.time()
        if now - self.last_export < self.args.export_interval:
            return
        self.last_export = now
        self.write_exports()

    def _ground_points(self):
        if not self.ground_buffer:
            return None
        return np.vstack(self.ground_buffer)

    def write_exports(self):
        """Write GeoTIFF, ground PCD and metadata (best effort, never raises)."""
        outdir = self.args.out_dir
        try:
            if not os.path.isdir(outdir):
                os.makedirs(outdir)
        except OSError:
            return
        ground = self._ground_points()
        stats = self.grid.stats()
        meta = {
            "coordinate_mode": "LOCALIZED",
            "crs": None,
            "origin_local": [self.grid.origin_x, self.grid.origin_y],
            "resolution_m": self.grid.resolution,
            "width": self.grid.width,
            "height": self.grid.height,
            "nodata": NODATA,
            "min_elevation_m": stats["min_elevation"],
            "max_elevation_m": stats["max_elevation"],
            "ground_points": self.ground_points,
            "tiles_completed": len(self.processed_tiles),
            "batches": self.batches,
            "ground_classifier": self.classifier.version,
            "updated_at": time.time(),
            "note": ("Local SLAM map frame; X/Y are relative metres from the "
                     "first map origin. No geographic CRS is assigned."),
        }
        try:
            with open(os.path.join(outdir, "dem_meta.json"), "w") as handle:
                json.dump(meta, handle, indent=2)
        except OSError:
            pass
        if ground is None:
            return
        try:
            _write_pcd(os.path.join(outdir, "ground_points.pcd"), ground)
        except (OSError, IOError):
            pass
        if self.classifier.pdal:
            try:
                _write_geotiff(self.classifier, ground,
                               os.path.join(outdir, "current.tif"),
                               self.args.resolution, 120)
            except Exception as exc:  # noqa: BLE001
                self.classifier.last_error = "geotiff export: %s" % exc

    # -- lifecycle --------------------------------------------------------
    def reset(self):
        with self.lock:
            self.grid = DemGrid(self.args.resolution, self.args.origin_x,
                                self.args.origin_y)
            self.pending = {}
            self.pending_touched = {}
            self.pending_points = 0
            self.processing = []
            self.processed_tiles = set()
            self.ground_points = 0
            self.batches = 0
            self.ground_buffer = []
            self.ground_buffer_points = 0
            self.dirty = None
            self.need_full = True
            self.status = "ready"
        self.broadcast(force=True)
        self.push_status()

    def run(self):
        threads = [
            threading.Thread(target=self.stream_loop, name="dem-stream"),
            threading.Thread(target=self.worker_loop, name="dem-worker"),
            threading.Thread(target=self.status_loop, name="dem-status"),
        ]
        for thread in threads:
            thread.daemon = True
            thread.start()
        print("[dem] backend      : %s" % self.args.backend)
        print("[dem] resolution   : %.2f m" % self.args.resolution)
        print("[dem] tile/batch   : %.1f m / %d points"
              % (self.args.tile_size, self.args.batch_points))
        print("[dem] ground filter: %s" % self.classifier.version)
        if self.classifier.last_error:
            print("[dem] note         : %s" % self.classifier.last_error)
        print("[dem] output dir   : %s" % self.args.out_dir)
        try:
            while not self.stop.is_set():
                self.stop.wait(1.0)
        except KeyboardInterrupt:
            pass

    def status_loop(self):
        while not self.stop.is_set():
            try:
                self.flush_idle()
                self.push_status()
                self.broadcast()
            except Exception as exc:  # noqa: BLE001 - keep reporting
                self.error = "status loop: %s" % exc
            self.stop.wait(2.0)


def _write_pcd(path, points):
    count = points.shape[0]
    with open(path, "w") as handle:
        handle.write(
            "# .PCD v0.7 - DEM ground points (local coordinates)\n"
            "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
            "WIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\n"
            "DATA ascii\n" % (count, count))
        _write_points_text(handle, points, "%.3f")


def _write_geotiff(classifier, ground, path, resolution, timeout):
    tempdir = tempfile.mkdtemp(prefix="lidar_dem_tif_")
    src = os.path.join(tempdir, "ground.ply")
    pipeline_path = os.path.join(tempdir, "pipeline.json")
    try:
        _write_ply(src, ground)
        pipeline = {
            "pipeline": [
                {"type": "readers.ply", "filename": src},
                {"type": "writers.gdal", "filename": path,
                 "resolution": resolution, "output_type": "min",
                 "nodata": NODATA, "gdaldriver": "GTiff"},
            ]
        }
        with open(pipeline_path, "w") as handle:
            json.dump(pipeline, handle)
        subprocess.check_output(
            [classifier.pdal, "pipeline", pipeline_path],
            stderr=subprocess.STDOUT, env=pdal_env(classifier.pdal),
            timeout=timeout)
    finally:
        for path_name in (src, pipeline_path):
            try:
                os.unlink(path_name)
            except OSError:
                pass
        try:
            os.rmdir(tempdir)
        except OSError:
            pass


def main():
    parser = argparse.ArgumentParser(description="Incremental DEM service")
    parser.add_argument("--backend", default=os.environ.get(
        "LIDAR_DEM_BACKEND", "http://127.0.0.1:5000"))
    parser.add_argument("--resolution", type=float, default=0.5,
                        help="DEM cell size in metres (default 0.5)")
    parser.add_argument("--origin-x", type=float, default=0.0)
    parser.add_argument("--origin-y", type=float, default=0.0)
    parser.add_argument("--tile-size", type=float, default=20.0,
                        help="spatial accumulation tile (metres)")
    parser.add_argument("--batch-points", type=int, default=12000,
                        help="points per tile before processing")
    parser.add_argument("--max-pending-points", type=int, default=3000000)
    parser.add_argument("--max-halo-points", type=int, default=200000)
    parser.add_argument("--max-ground-points", type=int, default=1500000)
    parser.add_argument("--export-interval", type=float, default=20.0,
                        help="seconds between GeoTIFF/PCD exports (0 = off)")
    parser.add_argument("--out-dir", default=os.environ.get(
        "LIDAR_DEM_DIR", "/tmp/lidar_dem"))
    parser.add_argument("--pdal", default=os.environ.get("LIDAR_PDAL_BIN", ""),
                        help="path to pdal (default: PATH or vendor/pdal)")
    parser.add_argument("--smrf-cell", type=float, default=1.0)
    parser.add_argument("--smrf-slope", type=float, default=0.15)
    parser.add_argument("--smrf-threshold", type=float, default=0.3)
    parser.add_argument("--smrf-window", type=float, default=18.0)
    args = parser.parse_args()
    service = DemService(args)
    service.run()


if __name__ == "__main__":
    main()
