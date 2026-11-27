#!/usr/bin/env bash
#
# record_sensors.sh
# -----------------------------------------------------------------------------
# Segmented recording of:
#   * u-blox 7 raw NMEA      -> ~/sensor_recording/<YYYY-MM-DD>/gps_data/gps_<date_time>.txt
#   * Livox MID-360S LiDAR+IMU -> ~/sensor_recording/<YYYY-MM-DD>/lidar_data/lidar_<date_time>.bag
#
# Everything from one recording day goes into one dated folder, so a new run
# never mixes with the previous day's data:
#
#   ~/sensor_recording/
#     2026-11-27/
#       lidar_data/lidar_2026-11-27-14-03-11_0.bag ...
#       gps_data/gps_20261127_140311.txt ...
#       segments_<HHMMSS>.csv, recording_<HHMMSS>.log
#
# That dated folder is what you hand to scripts/start_all.sh --file later (it
# looks into lidar_data/ itself).
#
# LiDAR/IMU: ONE continuous `rosbag record` session for the whole run, split by
# rosbag itself into SEGMENT_SECONDS (default 10 s) files.  The split files are
# consecutive slices of one uninterrupted stream: nothing is stopped between
# them, so there is no dead time at a seam and replaying them back to back is
# the same as replaying one long bag (the dashboard's global map then stays
# continuous instead of smearing, which is what the old per-segment
# stop/restart of rosbag caused: ~1.7 s of unrecorded motion per file).
#
# GPS: still one file per segment, gps_<YYYYMMDD_HHMMSS>.txt.  Only the GPS
# files follow the segment tag naming; bag names come from rosbag itself,
# lidar_<YYYY-MM-DD-HH-MM-SS>.bag, in the order they were written.
#
# Tested target: Ubuntu 18.04 + ROS Melodic + livox_ros_driver2 + u-blox 7.
#
# Usage:
#   sudo bash record_sensors.sh
#   sudo bash record_sensors.sh --duration 10
#   sudo bash record_sensors.sh --duration 10 --output-dir /data/sensor_recording
#   sudo bash record_sensors.sh --no-gps                 # only LiDAR/IMU
#   sudo bash record_sensors.sh --no-lidar               # only GPS
#   sudo bash record_sensors.sh --launch livox_ros_driver2.launch   # no Rviz
#   sudo bash record_sensors.sh --dry-run                # print config, touch nothing
#   sudo bash record_sensors.sh --help
#
# Environment overrides (upper-case names mirror the long options):
#   SEGMENT_SECONDS, GPS_DEV, NET_IFACE, LIDAR_IP, LOCAL_IP,
#   ROS_SETUP, LIVOX_WS_SETUP, LIVOX_LAUNCH, TOPICS, OUTPUT_DIR, SESSION_DATE
#
# NOTE: run with sudo for the IP/GPSD setup. The output directory is always
# resolved against the *invoking* user's home, even when sudo changes ~ to /root.
# -----------------------------------------------------------------------------

set -o pipefail

# ------------------------------- defaults -----------------------------------
SEGMENT_SECONDS="${SEGMENT_SECONDS:-10}"
GPS_DEV="${GPS_DEV:-/dev/ttyACM0}"
NET_IFACE="${NET_IFACE:-enp0s31f6}"
LIDAR_IP="${LIDAR_IP:-192.168.1.193}"
LOCAL_IP="${LOCAL_IP:-192.168.1.50/24}"
ROS_SETUP="${ROS_SETUP:-/opt/ros/melodic/setup.bash}"
LIVOX_LAUNCH="${LIVOX_LAUNCH:-rviz_MID360s.launch}"
TOPICS="${TOPICS:-/livox/lidar /livox/imu}"
GPS_ENABLED=1
LIDAR_ENABLED=1
DRY_RUN=0
OUTPUT_DIR="${OUTPUT_DIR:-}"

RUNNING=1
GPS_PID=""
BAG_PID=""
DRIVER_PID=""
GPSD_PID=""
SEG_INDEX=0

die()  { echo "[ERROR] $*" >&2; exit 1; }
warn() { echo "[WARN]  $*" >&2; }

usage() {
    cat <<'EOF'
record_sensors.sh -- segmented u-blox GPS + Livox MID-360S recorder

  -d, --duration SEC   length of each segment in seconds (default: 10)
      --gps-dev DEV    GPS serial device (default: /dev/ttyACM0)
      --iface IF       ethernet interface to the LiDAR (default: enp0s31f6)
      --lidar-ip IP    LiDAR IP (default: 192.168.1.193)
      --local-ip CIDR  laptop IP on that interface (default: 192.168.1.50/24)
      --output-dir DIR root folder (default: ~/sensor_recording/<today's date>)
                       runs land in <DIR>/lidar_data and <DIR>/gps_data
      --launch NAME    livox_ros_driver2 launch file (default: rviz_MID360s.launch)
      --topics "..."   space separated topics (default: "/livox/lidar /livox/imu")
      --no-gps         skip GPS recording
      --no-lidar       skip ROS bag recording
      --dry-run        print the resolved configuration and exit
  -h, --help           this help

Press Ctrl+C to stop: the current segment is closed cleanly and a summary is printed.
EOF
}

# --------------------------- argument parsing -------------------------------
while (( $# > 0 )); do
    case "$1" in
        -d|--duration)
            (( $# >= 2 )) || die "missing value for $1"; SEGMENT_SECONDS="$2"; shift 2 ;;
        --gps-dev)
            (( $# >= 2 )) || die "missing value for $1"; GPS_DEV="$2"; shift 2 ;;
        --iface)
            (( $# >= 2 )) || die "missing value for $1"; NET_IFACE="$2"; shift 2 ;;
        --lidar-ip)
            (( $# >= 2 )) || die "missing value for $1"; LIDAR_IP="$2"; shift 2 ;;
        --local-ip)
            (( $# >= 2 )) || die "missing value for $1"; LOCAL_IP="$2"; shift 2 ;;
        --output-dir)
            (( $# >= 2 )) || die "missing value for $1"; OUTPUT_DIR="$2"; shift 2 ;;
        --launch)
            (( $# >= 2 )) || die "missing value for $1"; LIVOX_LAUNCH="$2"; shift 2 ;;
        --topics)
            (( $# >= 2 )) || die "missing value for $1"; TOPICS="$2"; shift 2 ;;
        --no-gps)    GPS_ENABLED=0; shift ;;
        --no-lidar)  LIDAR_ENABLED=0; shift ;;
        --dry-run)   DRY_RUN=1; shift ;;
        -h|--help)   usage; exit 0 ;;
        *) die "unknown option '$1' (try --help)" ;;
    esac
done

[[ "$SEGMENT_SECONDS" =~ ^[1-9][0-9]*$ ]] || die "--duration must be a positive integer (got: '$SEGMENT_SECONDS')"
(( GPS_ENABLED == 1 || LIDAR_ENABLED == 1 )) || die "nothing to do: both --no-gps and --no-lidar were given"

# ---------------------- resolve the REAL user's home ------------------------
if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
    REAL_USER="$SUDO_USER"
    REAL_HOME="$(getent passwd "$SUDO_USER" | cut -d: -f6)"
else
    REAL_USER="${USER:-$(id -un)}"
    REAL_HOME="${HOME:-$(getent passwd "$REAL_USER" | cut -d: -f6)}"
fi
[[ -n "$REAL_HOME" && -d "$REAL_HOME" ]] || die "could not determine the target home directory"

# expand a leading ~ in --output-dir against the real home
OUTPUT_DIR="${OUTPUT_DIR/#\~/$REAL_HOME}"
# One folder per recording day, unless --output-dir says otherwise.
SESSION_DATE="${SESSION_DATE:-$(date +%F)}"
OUTPUT_ROOT="${OUTPUT_DIR:-${REAL_HOME}/sensor_recording/${SESSION_DATE}}"
GPS_DIR="${OUTPUT_ROOT}/gps_data"
BAG_DIR="${OUTPUT_ROOT}/lidar_data"
# Per-session names: recording twice in one day must not truncate the first
# session's log or manifest.
SESSION_STAMP="$(date +%H%M%S)"
LOG_FILE="${OUTPUT_ROOT}/recording_${SESSION_STAMP}.log"
MANIFEST="${OUTPUT_ROOT}/segments_${SESSION_STAMP}.csv"

LIVOX_WS_SETUP="${LIVOX_WS_SETUP:-${REAL_HOME}/ws_livox/devel/setup.bash}"
read -r -a TOPIC_ARR <<< "$TOPICS"
(( ${#TOPIC_ARR[@]} > 0 )) || die "--topics must contain at least one topic"

log() { printf '[%s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "$*" | tee -a "$LOG_FILE"; }

if (( DRY_RUN == 1 )); then
    cat <<EOF
DRY RUN -- no hardware or ROS commands will be executed.

  segment length   : ${SEGMENT_SECONDS} s
  GPS device       : ${GPS_DEV}          (enabled: ${GPS_ENABLED})
  interface        : ${NET_IFACE}
  laptop IP        : ${LOCAL_IP}
  LiDAR IP         : ${LIDAR_IP}
  ROS setup        : ${ROS_SETUP}
  Livox setup      : ${LIVOX_WS_SETUP}
  Livox launch     : ${LIVOX_LAUNCH}
  topics           : ${TOPICS}           (enabled: ${LIDAR_ENABLED})
  output root      : ${OUTPUT_ROOT}      (date folder: ${SESSION_DATE})
    gps files      : ${GPS_DIR}/gps_<YYYYMMDD_HHMMSS>.txt
    bag files      : ${BAG_DIR}/lidar_<YYYY-MM-DD-HH-MM-SS>.bag (one split session)
    manifest       : ${MANIFEST}
    log            : ${LOG_FILE}

Example: 10 minutes at 10 s/segment -> 60 .txt files and 60 .bag files.
EOF
    exit 0
fi

# --------------------------- signal handling --------------------------------
# Ctrl+C -> stop the two active recorders gracefully, then let the main loop
# fall through to the final summary instead of dying mid-segment.
on_signal() {
    RUNNING=0
    echo
    echo "[INFO]  Stop requested -- closing current segment..." >&2
    [[ -n "$GPS_PID" ]] && kill -INT "$GPS_PID" 2>/dev/null
    [[ -n "$BAG_PID" ]] && kill -INT "$BAG_PID" 2>/dev/null
    true
}
trap on_signal INT TERM

stop_pid() {
    local pid="$1" label="$2" i
    [[ -n "$pid" ]] || return 0
    kill -0 "$pid" 2>/dev/null || return 0
    kill -INT "$pid" 2>/dev/null || true
    for i in 1 2 3 4 5 6 7 8 9 10; do
        kill -0 "$pid" 2>/dev/null || { log "$label stopped."; return 0; }
        sleep 0.5
    done
    kill -KILL "$pid" 2>/dev/null || true
    log "$label did not exit cleanly -- force-stopped."
}

STEP=0
TOTAL_STEPS=1                                   # environment / tools / storage
(( GPS_ENABLED   )) && TOTAL_STEPS=$((TOTAL_STEPS + 2))   # GPS device + gpsd/NMEA
(( LIDAR_ENABLED )) && TOTAL_STEPS=$((TOTAL_STEPS + 5))   # ROS env + net + ping + driver + topics

step() {
    STEP=$(( STEP + 1 ))
    printf '\n' | tee -a "$LOG_FILE" >/dev/null
    printf '[%s] ===== STEP %d/%d: %s =====\n' \
        "$(date '+%H:%M:%S')" "$STEP" "$TOTAL_STEPS" "$*" | tee -a "$LOG_FILE"
}
ok()   { log "    [OK]   $*"; }
info() { log "    [....] $*"; }

# ------------------------------ preflight -----------------------------------
(( EUID == 0 )) || die "please run with sudo (IP/GPSD setup needs root): sudo bash $0"

mkdir -p "$GPS_DIR" "$BAG_DIR" || die "cannot create $OUTPUT_ROOT"
printf 'index,timestamp,duration_s,gps_file,gps_bytes,gps_lines,bag_file,bag_bytes\n' > "$MANIFEST"
: > "$LOG_FILE"

# A second run on the same day would land in the same dated folder; the clips
# would then replay as one run, so say it out loud instead of blending them
# silently.  Use --output-dir to keep runs apart.
existing_bags=$(find "$BAG_DIR" -maxdepth 1 -type f -name 'lidar_*.bag' | wc -l)
if (( existing_bags > 0 )); then
    warn "${existing_bags} clip(s) from an earlier run today are already in ${BAG_DIR}"
    warn "they replay together with this run; use --output-dir to keep runs separate"
fi

log "=== recording session started (user: $REAL_USER, segment: ${SEGMENT_SECONDS}s) ==="

# --- STEP: environment / required tools ---
step "Checking environment and required tools"
for cmd in date ip ping timeout stdbuf awk stat wc find df tee; do
    command -v "$cmd" >/dev/null 2>&1 || die "required command not found: $cmd"
done
ok "running as root (sudo) for IP/GPSD setup"
ok "output folder ready: $OUTPUT_ROOT"
ok "target user: $REAL_USER (home: $REAL_HOME)"

free_bytes="$(df -PB1 "$OUTPUT_ROOT" | awk 'NR==2{print $4}')"
info "free storage: $(( free_bytes / 1024 / 1024 )) MB"
(( free_bytes > 600 * 1024 * 1024 )) || warn "less than 600 MB free -- a 10-minute LiDAR session used ~1.7 GB previously"

# --- STEP: ROS environment MUST be sourced before looking for roslaunch ---
# sudo replaces PATH with secure_path, which does not contain /opt/ros/melodic/bin.
if (( LIDAR_ENABLED )); then
    step "Loading ROS Melodic + Livox workspace environment"
    [[ -f "$ROS_SETUP" ]]     || die "ROS setup not found: $ROS_SETUP"
    [[ -f "$LIVOX_WS_SETUP" ]] || die "Livox workspace setup not found: $LIVOX_WS_SETUP"

    # shellcheck disable=SC1090
    source "$ROS_SETUP"
    # shellcheck disable=SC1090
    source "$LIVOX_WS_SETUP"

    for cmd in roslaunch rosbag rostopic; do
        command -v "$cmd" >/dev/null 2>&1 || die "$cmd not found after sourcing $ROS_SETUP"
        ok "$cmd found at $(command -v "$cmd")"
    done
    ok "ROS_MASTER_URI=${ROS_MASTER_URI:-<default>}"
    ok "ROS workspace: $LIVOX_WS_SETUP"

    # Rviz/X11 when launched through sudo from a desktop session.
    export DISPLAY="${DISPLAY:-:0}"
    export XAUTHORITY="${XAUTHORITY:-${REAL_HOME}/.Xauthority}"
fi

# --- STEP: GPS device ---
if (( GPS_ENABLED )); then
    step "Looking for u-blox GPS receiver"
    for cmd in gpsd gpspipe systemctl; do
        command -v "$cmd" >/dev/null 2>&1 || die "required command not found: $cmd"
    done
    info "waiting for $GPS_DEV ..."
    for i in $(seq 1 10); do
        (( RUNNING )) || break
        [[ -e "$GPS_DEV" ]] && break
        sleep 1
    done
    [[ -e "$GPS_DEV" ]] || die "GPS device $GPS_DEV not found (check the USB cable)"
    ok "GPS device found: $GPS_DEV"
fi

# --- STEP: network ---
if (( LIDAR_ENABLED )); then
    step "Configuring ethernet for the Livox MID-360S"
    ip link set "$NET_IFACE" up             || die "cannot bring up $NET_IFACE"
    ip addr flush dev "$NET_IFACE"          || die "cannot flush $NET_IFACE"
    ip addr add "$LOCAL_IP" dev "$NET_IFACE" 2>/dev/null || true   # already set is fine
    addr_line="$(ip -br addr show dev "$NET_IFACE" 2>/dev/null)"
    echo "$addr_line" | grep -q "${LOCAL_IP%/*}" || die "IP $LOCAL_IP is not on $NET_IFACE: $addr_line"
    ok "interface UP: $addr_line"
    ok "laptop IP set to $LOCAL_IP on $NET_IFACE"

    step "Checking LiDAR link ($LIDAR_IP)"
    ping_ok=0
    for i in $(seq 1 5); do
        (( RUNNING )) || break
        info "ping attempt $i/5 ..."
        if ping -c 1 -W 1 "$LIDAR_IP" >/dev/null 2>&1; then ping_ok=1; break; fi
        sleep 1
    done
    (( ping_ok == 1 )) || die "cannot ping LiDAR at $LIDAR_IP (check Ethernet/cable/IP)"
    ping_detail="$(ping -c 4 -W 1 "$LIDAR_IP" 2>&1)"
    ok "LiDAR CONNECTED at $LIDAR_IP -- $(echo "$ping_detail" | grep -o 'time=[0-9.]* ms' | tail -1)"
    ok "ping summary: $(echo "$ping_detail" | grep 'packet loss')"
fi

# --- STEP: gpsd + live NMEA check ---
if (( GPS_ENABLED )); then
    step "Starting gpsd and checking live GPS data"
    systemctl stop gpsd gpsd.socket >/dev/null 2>&1 || true
    gpsd -N -n "$GPS_DEV" &
    GPSD_PID=$!
    sleep 1
    kill -0 "$GPSD_PID" 2>/dev/null || die "gpsd failed to start on $GPS_DEV"
    ok "gpsd started on $GPS_DEV (pid $GPSD_PID)"
    nmea="$(timeout 5 gpspipe -r -n 1 2>/dev/null | head -n 1)"
    if [[ -n "$nmea" ]]; then
        ok "GPS CONNECTED -- receiving NMEA: ${nmea:0:64}"
    else
        warn "no NMEA sentence in 5 s -- keep the antenna outdoors; recording will continue"
    fi
fi

# --- STEP: Livox driver ---
if (( LIDAR_ENABLED )); then
    step "Launching Livox driver (livox_ros_driver2 / $LIVOX_LAUNCH)"
    roslaunch livox_ros_driver2 "$LIVOX_LAUNCH" >>"$LOG_FILE" 2>&1 &
    DRIVER_PID=$!
    info "waiting for the driver to come up ..."
    sleep 3
    if kill -0 "$DRIVER_PID" 2>/dev/null; then
        ok "Livox driver CONNECTED (roslaunch pid $DRIVER_PID)"
    else
        warn "roslaunch exited early -- check $LOG_FILE; will still try to record"
    fi

    step "Verifying LiDAR / IMU topics and message rates"
    topics_ok=0
    for i in $(seq 1 30); do
        (( RUNNING )) || break
        topic_list="$(rostopic list 2>/dev/null)"
        missing=0
        for t in "${TOPIC_ARR[@]}"; do
            echo "$topic_list" | grep -qx "$t" || missing=1
        done
        if (( missing == 0 )); then topics_ok=1; break; fi
        (( i % 5 == 0 )) && info "still waiting for topics ... (${i}s)"
        sleep 1
    done
    if (( topics_ok == 0 )); then
        warn "not all requested topics appeared yet: ${TOPICS}"
        warn "recording anyway -- see $LOG_FILE"
    else
        ok "topics available: ${TOPICS}"
    fi
    for t in "${TOPIC_ARR[@]}"; do
        rate=""
        if command -v script >/dev/null 2>&1; then
            rate="$(timeout 6 script -qec "rostopic hz $t" /dev/null 2>/dev/null \
                    | tr '\r' '\n' | grep -o 'average rate: [0-9.]*' | tail -1)"
        fi
        if [[ -n "$rate" ]]; then
            ok "$t -> $rate Hz"
        else
            info "rate check inconclusive for $t (messages will be verified per segment)"
        fi
    done
fi

(( RUNNING )) || { log "Stopped during setup."; exit 0; }

log "Recording ${SEGMENT_SECONDS}-second segments into $OUTPUT_ROOT"
log "Press Ctrl+C at any time to finish the current segment and stop cleanly."

# ------------------------- LiDAR: one continuous session --------------------
# rosbag keeps ONE stream open for the whole run and writes a new file every
# SEGMENT_SECONDS.  Because the recorder never stops, the next file starts
# exactly where the previous one ended: the clip boundary is seamless, both for
# the SLAM backend (same process, continuous stamps) and for the map.
if (( LIDAR_ENABLED )); then
    log "Starting one continuous rosbag session (new file every ${SEGMENT_SECONDS}s)"
    info "bag files: ${BAG_DIR}/lidar_<date>-<time>.bag"
    rosbag record --split --duration="${SEGMENT_SECONDS}" -o "$BAG_DIR/lidar" \
        "${TOPIC_ARR[@]}" >>"$LOG_FILE" 2>&1 &
    BAG_PID=$!
    sleep 2
    if kill -0 "$BAG_PID" 2>/dev/null; then
        ok "rosbag recording (pid $BAG_PID), splitting every ${SEGMENT_SECONDS}s"
        sleep 1
        (( $(find "$BAG_DIR" -maxdepth 1 -name 'lidar_*' -newermt '-10 seconds' | wc -l) > 0 )) \
            || warn "no bag file written yet -- check $LOG_FILE"
    else
        warn "rosbag record exited immediately -- see $LOG_FILE"
        BAG_PID=""
        RUNNING=0
    fi
fi

# Bag files already reported in the manifest.
declare -A SEEN_BAGS=()
NEW_BAG_NAMES="-"; NEW_BAG_BYTES=0; NEW_BAG_COUNT=0

# Add every bag file that appeared since the last pass to the manifest row.
record_new_bags() {
    local f bytes
    NEW_BAG_NAMES="-"; NEW_BAG_BYTES=0; NEW_BAG_COUNT=0
    for f in "$BAG_DIR"/lidar_*.bag; do
        [[ -e "$f" ]] || continue
        [[ -n "${SEEN_BAGS[$f]:-}" ]] && continue
        SEEN_BAGS[$f]=1
        bytes="$(stat -c %s "$f" 2>/dev/null || echo 0)"
        (( bytes > 0 )) || warn "$(basename "$f") is empty -- check ${TOPICS}"
        NEW_BAG_BYTES=$(( NEW_BAG_BYTES + bytes ))
        NEW_BAG_COUNT=$(( NEW_BAG_COUNT + 1 ))
        if [[ "$NEW_BAG_NAMES" == "-" ]]; then
            NEW_BAG_NAMES="$f"
        else
            NEW_BAG_NAMES="${NEW_BAG_NAMES};${f}"
        fi
        log "  closed bag: $(basename "$f") (${bytes} B)"
    done
}

# --------------------------- recording loop ---------------------------------
# The loop only drives the GPS segments and keeps the manifest up to date; the
# LiDAR recorder runs in the background for the whole session.
while (( RUNNING )); do
    SEG_INDEX=$(( SEG_INDEX + 1 ))
    STAMP="$(date +%Y%m%d_%H%M%S)"

    # avoid overwriting if the configured segment is very short
    suffix=0
    while true; do
        tag="${STAMP}${suffix:+_${suffix}}"
        gps_file="${GPS_DIR}/gps_${tag}.txt"
        [[ -e "$gps_file" ]] || break
        suffix=$(( suffix + 1 ))
    done

    log "segment #${SEG_INDEX} @ ${tag}"

    if (( GPS_ENABLED )); then
        # stdbuf forces line-buffered writes so a SIGINT cannot truncate the tail.
        timeout -s INT -k 2s "${SEGMENT_SECONDS}" \
            stdbuf -oL -eL gpspipe -r >"$gps_file" 2>>"$LOG_FILE" &
        GPS_PID=$!
        wait "$GPS_PID" 2>/dev/null || true
        GPS_PID=""
    else
        # No GPS: just wait out the segment so the manifest has one row per bag.
        sleep "$SEGMENT_SECONDS" &
        GPS_PID=$!
        wait "$GPS_PID" 2>/dev/null || true
        GPS_PID=""
    fi

    if (( LIDAR_ENABLED )); then
        if [[ -n "$BAG_PID" ]] && ! kill -0 "$BAG_PID" 2>/dev/null; then
            warn "rosbag record stopped unexpectedly -- see $LOG_FILE"
            RUNNING=0
        fi
        record_new_bags
    else
        NEW_BAG_NAMES="-"; NEW_BAG_BYTES=0; NEW_BAG_COUNT=0
    fi

    gps_bytes=0; gps_lines=0
    if (( GPS_ENABLED )) && [[ -f "$gps_file" ]]; then
        gps_bytes="$(stat -c %s "$gps_file" 2>/dev/null || echo 0)"
        gps_lines="$(wc -l < "$gps_file" 2>/dev/null || echo 0)"
        (( gps_lines > 0 )) || warn "$gps_file is empty -- is the GPS antenna outside with a clear sky view?"
    fi

    printf '%s,%s,%s,%s,%s,%s,%s,%s\n' \
        "$SEG_INDEX" "$tag" "$SEGMENT_SECONDS" \
        "$([ "$GPS_ENABLED" -eq 1 ] && echo "$gps_file" || echo '-')" "$gps_bytes" "$gps_lines" \
        "$NEW_BAG_NAMES" "$NEW_BAG_BYTES" >> "$MANIFEST"

    log "segment #${SEG_INDEX} done: gps=${gps_lines} lines/${gps_bytes} B, ${NEW_BAG_COUNT} bag file(s)/${NEW_BAG_BYTES} B"
done

# ------------------------------- shutdown -----------------------------------
trap - INT TERM
log "Stopping helpers..."
stop_pid "$BAG_PID"    "rosbag"
stop_pid "$GPS_PID"    "gpspipe"

# The last split file is still .bag.active until rosbag closes it; make sure the
# final clip is usable (and listed) even if rosbag was force-stopped.
if (( LIDAR_ENABLED )); then
    for act in "$BAG_DIR"/lidar_*.bag.active; do
        [[ -e "$act" ]] || continue
        mv "$act" "${act%.active}"
        warn "recovered $(basename "$act") -> $(basename "${act%.active}")"
    done
    record_new_bags
    [[ "$NEW_BAG_COUNT" == 0 ]] || log "final bag file(s): ${NEW_BAG_COUNT}"
fi
stop_pid "$DRIVER_PID" "roslaunch (Livox driver)"
stop_pid "$GPSD_PID"   "gpsd"

gps_count="$(find "$GPS_DIR" -maxdepth 1 -type f -name 'gps_*.txt' | wc -l)"
bag_count="$(find "$BAG_DIR" -maxdepth 1 -type f -name 'lidar_*.bag' | wc -l)"
gps_size="$(du -sh "$GPS_DIR" 2>/dev/null | awk '{print $1}')"
bag_size="$(du -sh "$BAG_DIR" 2>/dev/null | awk '{print $1}')"
leftovers="$(find "$BAG_DIR" -maxdepth 1 -type f -name '*.active' | wc -l)"

echo
echo "==================== recording summary ===================="
echo " output root : $OUTPUT_ROOT  (${SESSION_DATE})"
echo " GPS files   : ${gps_count}  (${gps_size})  in ${GPS_DIR}"
echo " BAG files   : ${bag_count}  (${bag_size})  in ${BAG_DIR}"
echo " manifest    : $MANIFEST"
echo " log         : $LOG_FILE"
(( leftovers > 0 )) && echo " NOTE: ${leftovers} unfinished .bag.active file(s) found in ${BAG_DIR}"
echo "==========================================================="
log "=== session finished: ${gps_count} gps file(s), ${bag_count} bag file(s) ==="
exit 0
