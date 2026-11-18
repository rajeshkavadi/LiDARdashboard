#!/usr/bin/env bash
# Expose the local dashboard through Cloudflare Tunnel (Quick Tunnel).
# This only proxies HTTP/WebSocket; all LiDAR processing stays local.
set -u
PORT="${LIDAR_WEB_PORT:-5000}"
LOG="${LIDAR_TUNNEL_LOG:-/tmp/lidar_cf_tunnel.log}"

if ! command -v cloudflared >/dev/null 2>&1; then
  echo "cloudflared not found. Install it first:" >&2
  echo "  https://developers.cloudflare.com/cloudflare-one/connections/connect-networks/downloads/" >&2
  exit 1
fi

: > "$LOG"
echo "[lidar-web] starting Quick Tunnel to http://localhost:${PORT}"
echo "[lidar-web] public URL is written to ${LOG}"
cloudflared tunnel --url "http://localhost:${PORT}" --no-autoupdate 2>&1 | tee -a "$LOG"
