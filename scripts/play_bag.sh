#!/usr/bin/env bash
# Replay a recorded bag (testing only; the live driver is unaffected).
#
# Usage: scripts/play_bag.sh [bag] [extra rosbag args...]
# Defaults to the first bag found in the repository root, so the script never
# points at a file that does not exist.
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

if [ -n "$1" ] && [ -f "$1" ]; then
  BAG="$1"
  shift
else
  BAG=""
  for candidate in "$HERE"/*.bag; do
    [ -f "$candidate" ] || continue
    BAG="$candidate"
    break
  done
  if [ -z "$BAG" ]; then
    echo "play_bag: no .bag file found in $HERE" >&2
    echo "usage: $0 <bag> [rosbag play args...]" >&2
    exit 2
  fi
fi

LOOP="${BAG_LOOP:---loop}"

# Which topics to play: BAG_TOPICS if set, otherwise every PointCloud2 and Imu
# topic actually present in the bag -- so any recording works without knowing
# its topic names.  If none are found (unusual message types) the whole bag is
# played rather than nothing.
if [ -n "${BAG_TOPICS:-}" ]; then
  TOPICS="$BAG_TOPICS"
  SOURCE="BAG_TOPICS"
else
  # `rosbag info` puts the first topic on the "topics:" line and the rest on
  # continuation lines, so both layouts are handled.
  TOPICS=$(rosbag info "$BAG" 2>/dev/null \
    | awk '/^topics:/ { if ($0 ~ /sensor_msgs\/(PointCloud2|Imu)/) print $2 }
           /^[ \t]+\// { if ($0 ~ /sensor_msgs\/(PointCloud2|Imu)/) print $1 }' \
    | sort -u | tr '\n' ' ')
  SOURCE="PointCloud2/Imu topics in the bag"
fi

echo "play_bag: $BAG"
if [ -n "${TOPICS// /}" ]; then
  echo "play_bag: topics ($SOURCE): $TOPICS"
  # shellcheck disable=SC2086
  exec rosbag play $LOOP "$BAG" --topics $TOPICS "$@"
else
  echo "play_bag: no PointCloud2/Imu topics detected; playing the whole bag"
  # shellcheck disable=SC2086
  exec rosbag play $LOOP "$BAG" "$@"
fi
