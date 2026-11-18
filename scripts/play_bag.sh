#!/usr/bin/env bash
# Replay the recorded Livox bag (testing only; the live driver is unaffected).
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

if [ -f /opt/ros/melodic/setup.bash ]; then
  # shellcheck disable=SC1091
  source /opt/ros/melodic/setup.bash
fi

BAG="${1:-20261030_221123_mid360s.bag}"
shift || true
exec rosbag play --loop "$BAG" "$@"
