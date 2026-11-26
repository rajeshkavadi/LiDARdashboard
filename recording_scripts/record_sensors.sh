#!/usr/bin/env bash
#
# record_sensors.sh
# -----------------------------------------------------------------------------
# Segmented recording of:
#   * u-blox 7 raw NMEA      -> ~/sensor_recording/gps_data/gps_<date_time>.txt
#   * Livox MID-360S LiDAR+IMU -> ~/sensor_recording/lidar_data/lidar_<date_time>.bag
#
# One segment = SEGMENT_SECONDS (default 10 s). The loop keeps creating new
# timestamped file PAIRS until you press Ctrl+C. Both files of one segment use
# the SAME timestamp tag, so gps_20261126_220000.txt belongs to
# lidar_20261126_220000.bag -- handy for later fusion.
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
#   ROS_SETUP, LIVOX_WS_SETUP, LIVOX_LAUNCH, TOPICS
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
      --output-dir DIR root folder (default: ~/sensor_recording)
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
OUTPUT_ROOT="${OUTPUT_DIR:-${REAL_HOME}/sensor_recording}"
GPS_DIR="${OUTPUT_ROOT}/gps_data"
BAG_DIR="${OUTPUT_ROOT}/lidar_data"
LOG_FILE="${OUTPUT_ROOT}/recording.log"
MANIFEST="${OUTPUT_ROOT}/segments.csv"

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
  output root      : ${OUTPUT_ROOT}
    gps files      : ${GPS_DIR}/gps_<YYYYMMDD_HHMMSS>.txt
    bag files      : ${BAG_DIR}/lidar_<YYYYMMDD_HHMMSS>.bag
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

# --------------------------- recording loop ---------------------------------
while (( RUNNING )); do
    SEG_INDEX=$(( SEG_INDEX + 1 ))
    STAMP="$(date +%Y%m%d_%H%M%S)"

    # avoid overwriting if the configured segment is very short
    suffix=0
    while true; do
        tag="${STAMP}${suffix:+_${suffix}}"
        gps_file="${GPS_DIR}/gps_${tag}.txt"
        bag_file="${BAG_DIR}/lidar_${tag}.bag"
        [[ -e "$gps_file" || -e "$bag_file" || -e "${bag_file}.active" ]] || break
        suffix=$(( suffix + 1 ))
    done

    log "segment #${SEG_INDEX} @ ${tag}"

    # rosbag is started first so it has time to initialise before gpspipe starts.
    if (( LIDAR_ENABLED )); then
        # --duration stops rosbag normally after N seconds; `timeout` is only a
        # safety net in case rosbag hangs (3 s grace before it sends SIGINT).
        timeout -s INT -k 5s $(( SEGMENT_SECONDS + 3 )) \
            rosbag record --duration="${SEGMENT_SECONDS}" -O "$bag_file" "${TOPIC_ARR[@]}" \
            >>"$LOG_FILE" 2>&1 &
        BAG_PID=$!
    fi

    if (( GPS_ENABLED )); then
        # stdbuf forces line-buffered writes so a SIGINT cannot truncate the tail.
        timeout -s INT -k 2s "${SEGMENT_SECONDS}" \
            stdbuf -oL -eL gpspipe -r >"$gps_file" 2>>"$LOG_FILE" &
        GPS_PID=$!
    fi

    # Interrupted by the signal trap; children have already been told to stop.
    if [[ -n "$BAG_PID" ]]; then wait "$BAG_PID" 2>/dev/null; fi
    if [[ -n "$GPS_PID" ]]; then wait "$GPS_PID" 2>/dev/null; fi

    # rosbag should rename .active -> .bag on clean exit; recover it if not.
    if (( LIDAR_ENABLED )) && [[ -f "${bag_file}.active" && ! -f "$bag_file" ]]; then
        mv "${bag_file}.active" "$bag_file"
        warn "recovered ${bag_file}.active -> ${bag_file}"
    fi

    gps_bytes=0; gps_lines=0; bag_bytes=0
    if (( GPS_ENABLED )) && [[ -f "$gps_file" ]]; then
        gps_bytes="$(stat -c %s "$gps_file" 2>/dev/null || echo 0)"
        gps_lines="$(wc -l < "$gps_file" 2>/dev/null || echo 0)"
        (( gps_lines > 0 )) || warn "$gps_file is empty -- is the GPS antenna outside with a clear sky view?"
    fi
    if (( LIDAR_ENABLED )) && [[ -f "$bag_file" ]]; then
        bag_bytes="$(stat -c %s "$bag_file" 2>/dev/null || echo 0)"
        (( bag_bytes > 0 )) || warn "$bag_file is empty -- check /livox/lidar and /livox/imu"
    fi

    printf '%s,%s,%s,%s,%s,%s,%s,%s\n' \
        "$SEG_INDEX" "$tag" "$SEGMENT_SECONDS" \
        "$([ "$GPS_ENABLED" -eq 1 ] && echo "$gps_file" || echo '-')" "$gps_bytes" "$gps_lines" \
        "$([ "$LIDAR_ENABLED" -eq 1 ] && echo "$bag_file" || echo '-')" "$bag_bytes" >> "$MANIFEST"

    log "segment #${SEG_INDEX} done: gps=${gps_lines} lines/${gps_bytes} B, bag=${bag_bytes} B"
    GPS_PID=""; BAG_PID=""
done

# ------------------------------- shutdown -----------------------------------
trap - INT TERM
log "Stopping helpers..."
stop_pid "$BAG_PID"    "rosbag"
stop_pid "$GPS_PID"    "gpspipe"
stop_pid "$DRIVER_PID" "roslaunch (Livox driver)"
stop_pid "$GPSD_PID"   "gpsd"

gps_count="$(find "$GPS_DIR" -maxdepth 1 -type f -name 'gps_*.txt' | wc -l)"
bag_count="$(find "$BAG_DIR" -maxdepth 1 -type f -name 'lidar_*.bag' | wc -l)"
gps_size="$(du -sh "$GPS_DIR" 2>/dev/null | awk '{print $1}')"
bag_size="$(du -sh "$BAG_DIR" 2>/dev/null | awk '{print $1}')"
leftovers="$(find "$BAG_DIR" -maxdepth 1 -type f -name '*.active' | wc -l)"

echo
echo "==================== recording summary ===================="
echo " output root : $OUTPUT_ROOT"
echo " GPS files   : ${gps_count}  (${gps_size})  in ${GPS_DIR}"
echo " BAG files   : ${bag_count}  (${bag_size})  in ${BAG_DIR}"
echo " manifest    : $MANIFEST"
echo " log         : $LOG_FILE"
(( leftovers > 0 )) && echo " NOTE: ${leftovers} unfinished .bag.active file(s) found in ${BAG_DIR}"
echo "==========================================================="
log "=== session finished: ${gps_count} gps file(s), ${bag_count} bag file(s) ==="
exit 0
