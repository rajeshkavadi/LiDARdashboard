#!/usr/bin/env bash
# Start the SLAM stack that feeds the dashboard's GLOBAL MAP.
#
# Two processes:
#   1. lidar_odometry.py  - portable ICP odometry (numpy/scipy only, no build)
#   2. slam_web_bridge.py - exports pose/trajectory/tiles to the web backend
#
# The backend itself is a different script (scripts/start_web.sh); start it
# first, or use scripts/start_all.sh.
#
# Environment:
#   SLAM_BACKEND   "builtin" (default) or "external"
#                  external = FAST-LIO2 / Point-LIO / LIO-SAM / DLIO already
#                  running; only the exporter is started, and it auto-detects
#                  the odometry and registered-cloud topics.
#   LIDAR_TOPIC    point cloud topic (default: auto-detect)
#   IMU_TOPIC      IMU topic (default: auto-detect)
#   DESKEW         0 to disable per-point motion compensation (default 1)
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

if [ -f /opt/ros/melodic/setup.bash ]; then
  # ROS's setup.sh forwards "$@" to _setup_util.py, so our own arguments (a bag
  # path, --help, --loop) would be handed to the ROS setup and replace its
  # generated script with something that is then sourced. Hide them while
  # sourcing, then restore them.
  _LIDAR_ARGS=("$@")
  set --
  # shellcheck disable=SC1091
  source /opt/ros/melodic/setup.bash
  set -- "${_LIDAR_ARGS[@]}"
  unset _LIDAR_ARGS
fi

export ROS_PACKAGE_PATH="${HERE}/slam_ws/src:${ROS_PACKAGE_PATH:-}"
BACKEND="${SLAM_BACKEND:-builtin}"
MAP_PORT="${LIDAR_MAP_PORT:-8766}"
DESKEW="${DESKEW:-1}"

if ! command -v rosrun >/dev/null 2>&1; then
  echo "start_slam: ROS (rosrun) not found; source /opt/ros/melodic/setup.bash" >&2
  exit 2
fi

# The bundled odometry needs scipy (cKDTree).  Ubuntu's python2 has no pip, so
# vendor/py2 may hold a Python 2.7 scipy wheel that is used when the distro
# package is not installed.  Check up front instead of letting the import die
# in the middle of the bag playback log.
if [ -d "$HERE/vendor/py2/scipy" ]; then
  export PYTHONPATH="$HERE/vendor/py2:${PYTHONPATH:-}"
fi
if [ "$BACKEND" = "builtin" ] \
   && ! python2 -c "import scipy.spatial" >/dev/null 2>&1; then
  echo "start_slam: python-scipy is missing, so the built-in odometry cannot run." >&2
  echo "start_slam:   sudo apt install python-scipy" >&2
  echo "start_slam:   (or unpack a cp27 scipy wheel into $HERE/vendor/py2)" >&2
  exit 2
fi

PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

if [ "$BACKEND" = "builtin" ]; then
  echo "[slam] backend: built-in ICP odometry (no FAST-LIO2, no build)"
  rosrun lidar_slam_bridge lidar_odometry.py \
    _lidar_topic:="${LIDAR_TOPIC:-}" \
    _imu_topic:="${IMU_TOPIC:-}" \
    _deskew:="$DESKEW" &
  PIDS+=("$!")
  sleep 1
else
  echo "[slam] backend: external SLAM node (expects /Odometry + registered cloud)"
fi

echo "[slam] starting exporter -> tcp://127.0.0.1:${MAP_PORT}"
rosrun lidar_slam_bridge slam_web_bridge.py \
  _server_port:="$MAP_PORT" \
  _backend_name:="${SLAM_NAME:-$BACKEND}" &
PIDS+=("$!")

wait
