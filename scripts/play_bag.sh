#!/usr/bin/env bash
# Replay recorded bags (testing only; the live driver is unaffected).
#
# Usage:
#   scripts/play_bag.sh                          # first bag found in the repo root
#   scripts/play_bag.sh clip.bag [rosbag args]   # that bag, once
#   scripts/play_bag.sh --folder DIR             # every .bag in DIR, in order
#   scripts/play_bag.sh --folder=DIR
#
# DIR may be a dated recording folder (~/sensor_recording/2026-11-27): its
# clips are found in DIR/lidar_data when DIR itself has none.
#
# Folder mode is for recordings that arrive as a sequence of short clips (e.g.
# an edge device sending 10 s segments).  Two playback modes:
#
#   * stitched (default): the clips present when playback starts are merged into
#     ONE bag on a continuous timeline (tools/stitch_clips.py) and played once.
#     Best for a recording that is already complete, and the only mode that can
#     remove dead time from older recordings.
#   * follow: play each clip as soon as it is complete and wait for the next
#     one.  This is for a folder that is still being written: the live view
#     starts after the first clip instead of after the whole recording, and the
#     SLAM nodes keep running across clip boundaries.  --follow / LIDAR_FOLLOW=1
#     forces it, --no-follow / LIDAR_FOLLOW=0 disables it; when neither is given
#     it is chosen automatically if the folder received a clip recently.
#
# BAG_STITCH=0 (stitched mode only) plays the clips one by one with a fresh
# `rosbag play` each, including whatever dead time the recording has.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

usage() {
  echo "usage: $0 [<bag> [rosbag play args...]] | [--folder <dir> [rosbag play args...]]" >&2
  echo "  <bag>          replay that bag once (default: first .bag in $HERE)" >&2
  echo "  --folder <dir> stitch every .bag in <dir> into one run and play it" >&2
  echo "  --follow       play clips as they arrive instead of stitching (growing folder)" >&2
  echo "  env: BAG_LOOP=--loop (repeat), BAG_TOPICS=\"/a /b\" (topic override)" >&2
  echo "       LIDAR_FOLLOW=1/0, LIDAR_FOLLOW_IDLE=30 (s without a new clip -> stop)" >&2
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
FOLLOW_REQUEST=""
while [ $# -gt 0 ]; do
  case "$1" in
    --follow)
      FOLLOW_REQUEST=1
      shift
      ;;
    --no-follow|--stitch)
      FOLLOW_REQUEST=0
      shift
      ;;
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
  # A recording day folder (~/sensor_recording/2026-11-27) keeps its clips in
  # lidar_data/, next to gps_data/, so a folder without clips of its own is
  # searched one level down.  Point either at the date folder or at
  # .../lidar_data -- both work.
  list_bags() {
    local out
    out=$(find "$FOLDER" -maxdepth 1 \( -type f -o -type l \) -name '*.bag' -print \
            | LC_ALL=C sort -V)
    if [ -z "$out" ]; then
      out=$(find "$FOLDER" -maxdepth 2 \( -type f -o -type l \) -name '*.bag' -print \
              | LC_ALL=C sort -V)
    fi
    printf '%s\n' "$out"
  }
  # Filename order: the clips of a run are timestamped, and `sort -V` also
  # keeps "clip2" before "clip10" for hand-named segments.
  while IFS= read -r _bag; do
    [ -n "$_bag" ] || continue
    BAGS+=("$_bag")
  done < <(list_bags)
  unset _bag
  if [ "${#BAGS[@]}" = 0 ]; then
    echo "play_bag: no .bag files in $FOLDER (or in its lidar_data/ subfolder)" >&2
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

# Follow mode: a folder that is still being written (an edge device uploading
# 10 s clips, or the recorder running right now) must not wait for a stitch of
# the whole recording before it shows anything.  In this mode every clip is
# played as soon as its file is complete and the SLAM nodes keep running, so the
# live view stays about one clip behind the sensor instead of starting at the
# end of the recording.  What keeps that map continuous is that the clips really
# are pieces of one stream (record_sensors.sh splits a single rosbag session), so
# there is no dead time at the seam -- only the ~0.2 s `rosbag play` needs to
# restart, which no SLAM backend integrates over.
#
# Auto-selected when the newest clip was written less than LIDAR_FOLLOW_WINDOW
# seconds ago, i.e. the folder looks like it is still growing.
FOLLOW="${LIDAR_FOLLOW:-}"
if [ -z "$FOLLOW" ] && [ -n "$FOLLOW_REQUEST" ]; then
  FOLLOW="$FOLLOW_REQUEST"
fi
if [ -z "$FOLLOW" ] && [ "$CONTINUOUS" = 1 ]; then
  _newest=$(find "$FOLDER" -maxdepth 2 \( -type f -o -type l \) -name '*.bag' \
              -printf '%T@\n' 2>/dev/null | sort -n | tail -1)
  if [ -n "$_newest" ] && [ $(( $(date +%s) - ${_newest%%.*} )) -le "${LIDAR_FOLLOW_WINDOW:-120}" ]; then
    FOLLOW=1
  else
    FOLLOW=0
  fi
  unset _newest
fi
FOLLOW="${FOLLOW:-0}"
FOLLOW_IDLE="${LIDAR_FOLLOW_IDLE:-30}"

# Stitch the clips into one continuous bag.  Replaying N files with N `rosbag
# play` runs is not the same as one long bag: the LiDAR publisher disappears at
# every clip boundary, each restart adds a pause, and the dead time recorded
# between two clips makes an inertial backend integrate across a multi-second
# hole -- which is what turns the map into one smeared blob.  The stitched bag
# has a single, gap-free timeline, so SLAM/DEM/trajectory behave exactly like a
# single recording.  Set BAG_STITCH=0 to keep the old clip-by-clip playback.
STITCHED=0
TOTAL="${#BAGS[@]}"
if [ "$CONTINUOUS" = 1 ] && [ "$FOLLOW" != 1 ] && [ "$TOTAL" -gt 1 ] \
   && [ "${BAG_STITCH:-1}" != "0" ]; then
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
elif [ "$FOLLOW" = 1 ]; then
  echo "play_bag: folder $FOLDER"
  echo "play_bag: streaming clips as they arrive (follow mode: $TOTAL clip(s) so far)"
  echo "play_bag: SLAM keeps running across clip boundaries; stops after ${FOLLOW_IDLE}s without a new clip"
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

# Play one bag and wait for it to finish.  A clip that fails is reported and
# skipped so the rest of the run still happens.
PLAYED=0
play_one() {
  local bag="$1" rc=0 loop="$LOOP"
  PLAYED=$((PLAYED + 1))
  # A follow run always moves forward: looping a single clip would never end.
  if [ "$FOLLOW" = 1 ]; then
    loop=""
  fi
  detect_topics "$bag"
  if [ "$CONTINUOUS" = 1 ] && [ "$STITCHED" != 1 ] && [ "$FOLLOW" != 1 ]; then
    echo "play_bag: clip $PLAYED/$TOTAL $(basename "$bag")"
  fi
  if [ -n "${TOPICS// /}" ]; then
    echo "play_bag: topics ($SOURCE): $TOPICS"
    # shellcheck disable=SC2086
    rosbag play $loop "$bag" --topics $TOPICS "${POS[@]}" &
  else
    echo "play_bag: no PointCloud2/Imu topics detected; playing the whole bag"
    rosbag play $loop "$bag" "${POS[@]}" &
  fi
  CHILD=$!
  wait "$CHILD" || rc=$?
  CHILD=""
  if [ "$INTERRUPTED" = 1 ]; then
    return 0
  fi
  if [ "$rc" != 0 ]; then
    echo "play_bag: clip $(basename "$bag") exited with status $rc; continuing" >&2
    FAILED=$((FAILED + 1))
  fi
}

# Follow mode: replay the clips in order, and when the folder runs dry wait for
# the next one instead of exiting.  A file that is still being written (mtime <
# 1 s old) is left for the next pass, so a half-uploaded clip is never played.
follow_loop() {
  local -A seen=()
  local bag found idle_since=$SECONDS
  while :; do
    found=0
    while IFS= read -r bag; do
      [ -n "${seen[$bag]:-}" ] && continue
      [ "$INTERRUPTED" = 1 ] && break
      if [ $(( $(date +%s) - $(stat -c %Y "$bag" 2>/dev/null || echo 0) )) -lt 1 ]; then
        continue
      fi
      seen["$bag"]=1
      found=1
      echo "play_bag: new clip $(basename "$bag")"
      play_one "$bag"
      idle_since=$SECONDS
    done < <(list_bags)
    if [ "$INTERRUPTED" = 1 ]; then
      break
    fi
    if [ "$found" = 0 ] && [ $((SECONDS - idle_since)) -ge "$FOLLOW_IDLE" ]; then
      echo "play_bag: no new clip for ${FOLLOW_IDLE}s; ending the stream"
      break
    fi
    sleep 2
  done
}

if [ "$FOLLOW" = 1 ]; then
  follow_loop
else
  RUN=0
  while :; do
    RUN=$((RUN + 1))
    for BAG in "${BAGS[@]}"; do
      if [ "$INTERRUPTED" = 1 ]; then
        break
      fi
      play_one "$BAG"
    done

    if [ "$INTERRUPTED" = 1 ]; then
      break
    fi
    [ -n "$LOOP" ] || break
  done
fi

if [ "$CONTINUOUS" = 1 ] && [ "$FOLLOW" != 1 ] && [ "$FAILED" = 0 ] \
   && [ "$INTERRUPTED" = 0 ] && [ "$STITCHED" != 1 ]; then
  echo "play_bag: folder finished ($TOTAL clips)"
fi
if [ "$FOLLOW" = 1 ] && [ "$INTERRUPTED" = 0 ] && [ "$FAILED" = 0 ]; then
  echo "play_bag: clip stream finished ($PLAYED clip(s) played)"
fi
if [ "$STITCHED" = 1 ] && [ "$INTERRUPTED" = 0 ] && [ "$FAILED" = 0 ]; then
  echo "play_bag: stitched session finished"
fi
[ "$FAILED" = 0 ] || exit 1
exit 0
