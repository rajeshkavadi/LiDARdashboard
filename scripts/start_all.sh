#!/usr/bin/env bash
# Start everything needed for the full dashboard, in the right order.
#
#   ROS master (roscore, reused when one is already running)
#   Flux/WebSocket dashboard + Cloudflare tunnel  (python3, port 5000)
#   live point-cloud bridge                       (ROS python2, tcp 8765)
#   SLAM (built-in ICP odometry + exporter)       (ROS python2, tcp 8766)
#   incremental DEM worker                        (python3, PDAL)
#
# Nothing is built and nothing is downloaded: if ROS Melodic and Python 3 are
# installed, this works on a fresh Ubuntu 18.04 machine.
#
# Usage:
#   scripts/start_all.sh                  # dashboard + bridge + SLAM + DEM
#   scripts/start_all.sh --bag            # ... and replay the default bag
#   scripts/start_all.sh /path/to/x.bag   # ... and replay that bag
#   scripts/start_all.sh --bag=/path/x.bag
#   scripts/start_all.sh --file /path/dir # ... and replay the clips in dir
#   scripts/start_all.sh --file           # ... from $LIDAR_BAG_DIR
#   NO_TUNNEL=1 scripts/start_all.sh      # skip the Cloudflare tunnel
#   NO_SLAM=1 / NO_DEM=1 / NO_BRIDGE=1 to leave a component out
#
# A bag is played once.  When it reaches the end, every component is stopped
# and the script exits; Ctrl-C stops everything earlier.
#
# A folder of short clips (e.g. 10 s segments uploaded by an edge device) is
# stitched into one bag on a continuous timeline (scripts/play_bag.sh, tools/
# stitch_clips.py) and replayed once, so the dashboard, global map, trajectory
# and DEM accumulate as if the whole run had arrived as one bag -- which is
# what the SLAM and DEM components need.  `--folder` is an alias for `--file`.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

# Folder used by a bare `--file` when LIDAR_BAG_DIR is not set: the directory
# the edge device uploads its short clips into on this machine.
DEFAULT_BAG_DIR="${LIDAR_BAG_DIR:-/home/rajesh/sensor_recording/lidar_data}"

usage() {
  echo "usage: $0 [--bag | --bag=<file> | <file.bag> | --file <dir>] [--help]" >&2
  echo "  (no argument)       live LiDAR" >&2
  echo "  --bag               replay the first .bag in $HERE (once)" >&2
  echo "  <file.bag>          replay that bag (also --bag=<file>)" >&2
  echo "  --file <dir>        stitch the .bag files in <dir> into one continuous run" >&2
  echo "  --file              same, for $DEFAULT_BAG_DIR" >&2
  echo "  --file=<dir>        (--folder is an alias for --file)" >&2
  echo "  env: NO_TUNNEL=1 NO_DEM=1 NO_SLAM=1 NO_BRIDGE=1 BAG_LOOP=--loop" >&2
}

if [ -f /opt/ros/melodic/setup.bash ]; then
  # ROS's setup.sh forwards "$@" to _setup_util.py, so our own arguments
  # ("--bag", a bag path, "--help") would be handed to the ROS setup and
  # replace its generated script with something that is then sourced.
  # Hide them while sourcing, then restore them.
  _LIDAR_ARGS=("$@")
  set --
  # shellcheck disable=SC1091
  source /opt/ros/melodic/setup.bash
  set -- "${_LIDAR_ARGS[@]}"
  unset _LIDAR_ARGS
fi

PLAY_BAG=0
BAG_PATH=""
BAG_DIR=""
DIR_REQUESTED=0
while [ $# -gt 0 ]; do
  case "$1" in
    --bag)
      PLAY_BAG=1
      ;;
    --bag=*)
      PLAY_BAG=1
      BAG_PATH="${1#*=}"
      ;;
    --file|--folder)
      PLAY_BAG=1
      DIR_REQUESTED=1
      # `--file <dir>` takes the next argument when it is a path; a bare
      # `--file` (or one followed by another option) uses the default folder.
      if [ $# -ge 2 ] && [ "${2#-}" = "$2" ]; then
        BAG_DIR="$2"
        shift
      else
        BAG_DIR="${LIDAR_BAG_DIR:-$DEFAULT_BAG_DIR}"
      fi
      ;;
    --file=*|--folder=*)
      PLAY_BAG=1
      DIR_REQUESTED=1
      BAG_DIR="${1#*=}"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --*)
      echo "start_all: unknown option: $1" >&2
      usage
      exit 2
      ;;
    *)
      # A bare argument is the bag to replay.
      PLAY_BAG=1
      BAG_PATH="$1"
      ;;
  esac
  shift
done

BAG_ARGS=()
if [ "$DIR_REQUESTED" = 1 ]; then
  case "$BAG_DIR" in
    "~"/*) BAG_DIR="$HOME/${BAG_DIR#\~/}" ;;
    /*) ;;                      # absolute, use as given
    *) BAG_DIR="$(pwd)/$BAG_DIR" ;;
  esac
  if [ ! -d "$BAG_DIR" ]; then
    echo "start_all: folder not found: $BAG_DIR" >&2
    usage
    exit 2
  fi
  CLIPS=$(find "$BAG_DIR" -maxdepth 1 \( -type f -o -type l \) -name '*.bag' -print | wc -l | tr -d ' ')
  if [ "$CLIPS" = 0 ]; then
    echo "start_all: no .bag files in $BAG_DIR" >&2
    exit 2
  fi
  BAG_PATH=""
  BAG_ARGS+=(--folder "$BAG_DIR")
  echo "start_all: folder playback: $CLIPS clip(s) from $BAG_DIR"
fi
if [ -n "$BAG_PATH" ]; then
  case "$BAG_PATH" in
    "~"/*) BAG_PATH="$HOME/${BAG_PATH#\~/}" ;;
    /*) ;;                      # absolute, use as given
    *) BAG_PATH="$(pwd)/$BAG_PATH" ;;
  esac
  if [ ! -f "$BAG_PATH" ]; then
    echo "start_all: bag not found: $BAG_PATH" >&2
    usage
    exit 2
  fi
  BAG_ARGS+=("$BAG_PATH")
fi

WEB_ARGS=()
if [ "${NO_TUNNEL:-0}" = "1" ]; then
  WEB_ARGS+=(--no-tunnel)
fi

PIDS=()
BAG_PID=""
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  # A component that ignores SIGTERM would keep the port / ROS master busy for
  # the next run, so give everything a moment and then insist.
  sleep 1
  for pid in "${PIDS[@]}"; do
    kill -9 "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

echo "== backend =="
python3 webapp/app.py "${WEB_ARGS[@]}" &
PIDS+=("$!")

WEB_URL="http://127.0.0.1:${LIDAR_WEB_PORT:-5000}"
WEB_PID="${PIDS[0]}"
WEB_READY=0
for _ in $(seq 1 30); do
  if curl -s --max-time 1 "$WEB_URL/api/status" >/dev/null 2>&1; then
    WEB_READY=1
    break
  fi
  # The backend died (bad port, missing dependency, traceback): stop here
  # rather than start three more processes that can only log connection
  # refused for the rest of the session.
  if ! kill -0 "$WEB_PID" 2>/dev/null; then
    break
  fi
  sleep 0.5
done

if [ "$WEB_READY" != "1" ]; then
  echo "[start_all] ERROR: the web backend never answered on $WEB_URL" >&2
  echo "[start_all]        see the traceback above; no other component was started." >&2
  exit 1
fi

# ROS master.  rosbag, the bridge, the odometry node and the exporter are all
# ROS nodes: without a master they exit with "Unable to communicate with
# master", which looks exactly like a dead bridge and an empty dashboard.
# Reuse a master that is already running, otherwise start one.
if [ "${NO_SLAM:-0}" != "1" ] || [ "${NO_BRIDGE:-0}" != "1" ] || [ "$PLAY_BAG" = "1" ]; then
  if ! command -v roscore >/dev/null 2>&1; then
    echo "[start_all] ERROR: ROS not found; source /opt/ros/melodic/setup.bash" >&2
    echo "[start_all]        no ROS component was started." >&2
    exit 1
  fi
  if rosnode list >/dev/null 2>&1; then
    echo "== ROS master (already running at ${ROS_MASTER_URI:-http://localhost:11311}) =="
  else
    echo "== ROS master (roscore) =="
    roscore >"${LIDAR_ROSCORE_LOG:-/tmp/lidar_roscore.log}" 2>&1 &
    PIDS+=("$!")
    ROS_READY=0
    for _ in $(seq 1 60); do
      if rosnode list >/dev/null 2>&1; then
        ROS_READY=1
        break
      fi
      sleep 0.25
    done
    if [ "$ROS_READY" != "1" ]; then
      echo "[start_all] ERROR: roscore never came up; see ${LIDAR_ROSCORE_LOG:-/tmp/lidar_roscore.log}" >&2
      exit 1
    fi
  fi
fi

if [ "${NO_DEM:-0}" != "1" ]; then
  echo "== DEM worker =="
  "$HERE/scripts/start_dem.sh" &
  PIDS+=("$!")
fi

# Playback goes first so its topics exist before the ROS nodes look for them.
if [ "$PLAY_BAG" = "1" ]; then
  if [ "$DIR_REQUESTED" = 1 ]; then
    echo "== clip playback (stitched) =="
  else
    echo "== bag playback =="
  fi
  "$HERE/scripts/play_bag.sh" "${BAG_ARGS[@]}" &
  BAG_PID="$!"
  PIDS+=("$BAG_PID")
  sleep 2
fi

if [ "${NO_SLAM:-0}" != "1" ]; then
  echo "== SLAM =="
  "$HERE/scripts/start_slam.sh" &
  PIDS+=("$!")
fi

if [ "${NO_BRIDGE:-0}" != "1" ]; then
  echo "== live bridge =="
  "$HERE/scripts/start_bridge.sh" &
  PIDS+=("$!")
fi

echo
echo "dashboard: http://127.0.0.1:${LIDAR_WEB_PORT:-5000}/"
if [ -n "$BAG_PID" ]; then
  if [ "$DIR_REQUESTED" = 1 ]; then
    echo "the clips play as one continuous run; everything stops at its end"
  else
    echo "the bag plays once; everything stops when it ends (Ctrl+C to stop early)"
  fi
  # Wait only for the playback: when the recording is over there is nothing
  # left to show, so the whole stack is torn down by the EXIT trap.
  wait "$BAG_PID" 2>/dev/null || true
  echo
  if [ "$DIR_REQUESTED" = 1 ]; then
    echo "[start_all] clip sequence finished; stopping the dashboard, bridge, SLAM and DEM"
  else
    echo "[start_all] bag finished; stopping the dashboard, bridge, SLAM and DEM"
  fi
  exit 0
fi
echo "press Ctrl+C to stop everything"
wait
