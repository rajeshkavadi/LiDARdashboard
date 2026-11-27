#!/usr/bin/env bash
# Replay recorded bags (testing only; the live driver is unaffected).
#
# Usage:
#   scripts/play_bag.sh                          # first bag found in the repo root
#   scripts/play_bag.sh clip.bag [rosbag args]   # that bag, once
#   scripts/play_bag.sh --folder DIR             # every .bag in DIR, in order
#   scripts/play_bag.sh --folder=DIR
#
# Folder mode is for recordings that arrive as a sequence of short clips (e.g.
# an edge device sending 10 s segments).  By default the clips are first
# stitched into ONE bag on a continuous timeline (tools/stitch_clips.py) and
# that single bag is played: the SLAM stack then receives one uninterrupted
# stream, exactly as if the session had been recorded as one long bag.
# BAG_STITCH=0 falls back to playing each clip with its own `rosbag play`, which
# leaves a pause -- and the recorded dead time -- between clips.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

usage() {
  echo "usage: $0 [<bag> [rosbag play args...]] | [--folder <dir> [rosbag play args...]]" >&2
  echo "  <bag>          replay that bag once (default: first .bag in $HERE)" >&2
  echo "  --folder <dir> stitch every .bag in <dir> into one run and play it" >&2
  echo "  env: BAG_LOOP=--loop (repeat), BAG_TOPICS=\"/a /b\" (topic override)" >&2
  echo "       BAG_STITCH=0 (play the clips one by one instead of stitching)" >&2
  echo "       LIDAR_STITCH_GAP=0.02 (seam length in seconds), LIDAR_STITCH_CACHE=DIR" >&2
}

if [ -f /opt/ros/melodic/setup.bash ]; then
  # ROS's setup.sh forwards "$@" to _setup_util.py, so our own arguments (a bag
  # path, --help, --folder, --loop) would be handed to the ROS setup and replace
  # its generated script with something that is then sourced. Hide them while
  # sourcing, then restore them.
  _LIDAR_ARGS=("$@")
  set --
  # shellcheck disable=SC1091
  source /opt/ros/melodic/setup.bash
  set -- "${_LIDAR_ARGS[@]}"
  unset _LIDAR_ARGS
fi

FOLDER=""
POS=()
while [ $# -gt 0 ]; do
  case "$1" in
    --folder|--file|-d)
      if [ $# -lt 2 ]; then
        echo "play_bag: $1 needs a directory" >&2
        usage
        exit 2
      fi
      FOLDER="$2"
      shift 2
      ;;
    --folder=*|--file=*)
      FOLDER="${1#*=}"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      POS+=("$1")
      shift
      ;;
  esac
done

# Which topics to play: BAG_TOPICS if set, otherwise every PointCloud2 and Imu
# topic actually present in the bag -- so any recording works without knowing
# its topic names.  If none are found (unusual message types) the whole bag is
# played rather than nothing.
detect_topics() {
  if [ -n "${BAG_TOPICS:-}" ]; then
    TOPICS="$BAG_TOPICS"
    SOURCE="BAG_TOPICS"
  else
    # `rosbag info` puts the first topic on the "topics:" line and the rest on
    # continuation lines, so both layouts are handled.
    TOPICS=$(rosbag info "$1" 2>/dev/null \
      | awk '/^topics:/ { if ($0 ~ /sensor_msgs\/(PointCloud2|Imu)/) print $2 }
             /^[ \t]+\// { if ($0 ~ /sensor_msgs\/(PointCloud2|Imu)/) print $1 }' \
      | sort -u | tr '\n' ' ')
    SOURCE="PointCloud2/Imu topics in the bag"
  fi
}

CONTINUOUS=0
BAGS=()

if [ -n "$FOLDER" ]; then
  case "$FOLDER" in
    "~"/*) FOLDER="$HOME/${FOLDER#\~/}" ;;
    "~") FOLDER="$HOME" ;;
    /*) ;;                                # absolute, use as given
    *) FOLDER="$(pwd)/$FOLDER" ;;
  esac
  if [ ! -d "$FOLDER" ]; then
    echo "play_bag: not a directory: $FOLDER" >&2
    exit 2
  fi
  # Filename order: the clips of a run are timestamped, and `sort -V` also
  # keeps "clip2" before "clip10" for hand-named segments.
  while IFS= read -r _bag; do
    BAGS+=("$_bag")
  done < <(find "$FOLDER" -maxdepth 1 \( -type f -o -type l \) -name '*.bag' -print \
            | LC_ALL=C sort -V)
  unset _bag
  if [ "${#BAGS[@]}" = 0 ]; then
    echo "play_bag: no .bag files in $FOLDER" >&2
    exit 2
  fi
  CONTINUOUS=1
else
  if [ -n "${POS[0]:-}" ] && [ -f "${POS[0]}" ]; then
    BAGS+=("${POS[0]}")
    POS=("${POS[@]:1}")
  else
    for candidate in "$HERE"/*.bag; do
      [ -f "$candidate" ] || continue
      BAGS+=("$candidate")
      break
    done
    if [ "${#BAGS[@]}" = 0 ]; then
      echo "play_bag: no .bag file found in $HERE" >&2
      usage
      exit 2
    fi
  fi
fi

# Stitch the clips into one continuous bag.  Replaying N files with N `rosbag
# play` runs is not the same as one long bag: the LiDAR publisher disappears at
# every clip boundary, each restart adds a pause, and the dead time recorded
# between two clips makes an inertial backend integrate across a multi-second
# hole -- which is what turns the map into one smeared blob.  The stitched bag
# has a single, gap-free timeline, so SLAM/DEM/trajectory behave exactly like a
# single recording.  Set BAG_STITCH=0 to keep the old clip-by-clip playback.
STITCHED=0
TOTAL="${#BAGS[@]}"
if [ "$CONTINUOUS" = 1 ] && [ "$TOTAL" -gt 1 ] && [ "${BAG_STITCH:-1}" != "0" ]; then
  _args=(--gap "${LIDAR_STITCH_GAP:-0.02}")
  if [ -n "${LIDAR_STITCH_CACHE:-}" ]; then
    _args+=(--cache-dir "$LIDAR_STITCH_CACHE")
  fi
  if [ "${BAG_STITCH_FORCE:-0}" = "1" ]; then
    _args+=(--force)
  fi
  echo "play_bag: stitching $TOTAL clips into one continuous bag ..."
  STITCH_LOG="${TMPDIR:-/tmp}/lidar_stitch.log"
  if MERGED=$(python2 "$HERE/tools/stitch_clips.py" "${_args[@]}" "${BAGS[@]}" 2>"$STITCH_LOG"); then
    BAGS=("$MERGED")
    STITCHED=1
  else
    echo "play_bag: stitching unavailable; falling back to clip-by-clip playback" >&2
    sed 's/^/play_bag:   /' "$STITCH_LOG" 2>/dev/null | tail -5 >&2 || true
  fi
  unset _args
fi

TOTAL="${#BAGS[@]}"

# One pass by default: the recording is played once and the command exits at
# the end of the last bag.  Set BAG_LOOP=--loop to keep replaying it.
LOOP="${BAG_LOOP:-}"

if [ "$STITCHED" = 1 ]; then
  echo "play_bag: folder $FOLDER"
  echo "play_bag: $TOTAL clip(s) stitched into one bag: $(basename "${BAGS[0]}")"
  echo "play_bag: one continuous timeline (no clip boundaries, no dead time)"
  if [ -n "$LOOP" ]; then
    echo "play_bag: looping the whole session ($LOOP); Ctrl+C to stop"
  else
    echo "play_bag: playing once (BAG_LOOP=--loop to repeat the session)"
  fi
elif [ "$CONTINUOUS" = 1 ]; then
  echo "play_bag: folder $FOLDER"
  echo "play_bag: $TOTAL clip(s), played one by one (BAG_STITCH=0)"
  if [ -n "$LOOP" ]; then
    echo "play_bag: looping the whole folder ($LOOP); Ctrl+C to stop"
  else
    echo "play_bag: playing once (BAG_LOOP=--loop to repeat the folder)"
  fi
else
  echo "play_bag: ${BAGS[0]}"
  if [ -n "$LOOP" ]; then
    echo "play_bag: looping ($LOOP); Ctrl+C to stop"
  else
    echo "play_bag: playing once (BAG_LOOP=--loop to repeat)"
  fi
fi

# Ctrl-C during a clip must stop the sequence, not start the next one.  The
# player runs as a supervised child so that a signal aimed at this script only
# (start_all.sh stops the stack that way) also stops rosbag play instead of
# leaving it publishing as an orphan.
INTERRUPTED=0
FAILED=0
CHILD=""
kill_child() {
  INTERRUPTED=1
  if [ -n "$CHILD" ]; then
    kill "$CHILD" 2>/dev/null || true
  fi
}
trap kill_child INT TERM
trap 'kill_child' EXIT

RUN=0
while :; do
  RUN=$((RUN + 1))
  INDEX=0
  for BAG in "${BAGS[@]}"; do
    INDEX=$((INDEX + 1))
    [ "$INTERRUPTED" = 1 ] && break

    detect_topics "$BAG"
    if [ "$CONTINUOUS" = 1 ] && [ "$STITCHED" != 1 ]; then
      echo "play_bag: clip $INDEX/$TOTAL $(basename "$BAG")"
    fi

    if [ -n "${TOPICS// /}" ]; then
      echo "play_bag: topics ($SOURCE): $TOPICS"
      # shellcheck disable=SC2086
      rosbag play $LOOP "$BAG" --topics $TOPICS "${POS[@]}" &
    else
      echo "play_bag: no PointCloud2/Imu topics detected; playing the whole bag"
      rosbag play $LOOP "$BAG" "${POS[@]}" &
    fi
    CHILD=$!
    RC=0
    wait "$CHILD" || RC=$?
    CHILD=""
    [ "$INTERRUPTED" = 1 ] && break

    # A corrupt or truncated clip (a segment still being uploaded) must not
    # abort the rest of the run: report it and carry on with the next clip.
    if [ "$RC" != 0 ]; then
      echo "play_bag: clip $(basename "$BAG") exited with status $RC; continuing" >&2
      FAILED=$((FAILED + 1))
    fi
  done

  [ "$INTERRUPTED" = 1 ] && break
  [ -n "$LOOP" ] || break
done

if [ "$CONTINUOUS" = 1 ] && [ "$FAILED" = 0 ] && [ "$INTERRUPTED" = 0 ] \
   && [ "$STITCHED" != 1 ]; then
  echo "play_bag: folder finished ($TOTAL clips)"
fi
if [ "$STITCHED" = 1 ] && [ "$INTERRUPTED" = 0 ] && [ "$FAILED" = 0 ]; then
  echo "play_bag: stitched session finished"
fi
[ "$FAILED" = 0 ] || exit 1
exit 0
