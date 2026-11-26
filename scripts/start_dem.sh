#!/usr/bin/env bash
# Start the incremental DEM/terrain worker.
#
# It reads the global map from the backend over loopback and classifies ground
# with PDAL (vendored copy used automatically when PDAL is not on PATH).  It is
# a separate process, so a slow DEM can never delay LiDAR, SLAM or the viewer.
#
# Environment:
#   DEM_RESOLUTION  cell size in metres            (default 0.5)
#   LIDAR_DEM_DIR   output directory for exports   (default /tmp/lidar_dem)
#   LIDAR_DEM_BACKEND backend URL                  (default http://127.0.0.1:5000)
#   LIDAR_PDAL_BIN  explicit pdal binary
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

exec python3 -u webapp/dem_service.py \
  --backend "${LIDAR_DEM_BACKEND:-http://127.0.0.1:5000}" \
  --resolution "${DEM_RESOLUTION:-0.5}" \
  --out-dir "${LIDAR_DEM_DIR:-/tmp/lidar_dem}" \
  "$@"
