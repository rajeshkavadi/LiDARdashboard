#!/usr/bin/env bash
# Start the Flask + WebSocket dashboard (Python 3). Independent of ROS.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$HERE"

HOST="${LIDAR_WEB_HOST:-127.0.0.1}"
PORT="${LIDAR_WEB_PORT:-5000}"

echo "[lidar-web] starting dashboard on http://${HOST}:${PORT}"
exec python3 webapp/app.py --host "$HOST" --port "$PORT" "$@"
