#!/usr/bin/env bash
# Start the read-only ROS Python 2 bridge that feeds the web backend.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ -f /opt/ros/melodic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/melodic/setup.bash
fi

export ROS_PACKAGE_PATH="${HERE}/ros_pkg:${ROS_PACKAGE_PATH:-}"
TOPIC="${TOPIC:-/livox/lidar}"
STRIDE="${POINT_STRIDE:-1}"
PORT="${LIDAR_INGEST_PORT:-8765}"

exec rosrun lidar_web_bridge ros_bridge.py \
  _topic:="${TOPIC}" \
  _server_port:="${PORT}" \
  _point_stride:="${STRIDE}"
