#!/usr/bin/env bash
# Start the read-only ROS Python 2 bridge that feeds the web backend.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

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

export ROS_PACKAGE_PATH="${HERE}/ros_pkg:${ROS_PACKAGE_PATH:-}"
# Empty TOPIC = auto-detect (/mid360/livox/lidar, /livox/lidar, /ouster/points, ...)
TOPIC="${TOPIC:-}"
STRIDE="${POINT_STRIDE:-1}"
PORT="${LIDAR_INGEST_PORT:-8765}"

exec rosrun lidar_web_bridge ros_bridge.py \
  _topic:="${TOPIC}" \
  _server_port:="${PORT}" \
  _point_stride:="${STRIDE}"
