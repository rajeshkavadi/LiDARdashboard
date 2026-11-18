"""Point-cloud frame hub and ROS bridge ingest server.

The ROS side runs under Python 2.7 (ROS Melodic rospy) and cannot be imported
from this Python 3 web process.  ``ros_bridge.py`` therefore forwards each
LiDAR frame over a loopback TCP socket using a tiny binary protocol.  This
module receives those frames, keeps only the newest one, and fans it out to
connected browsers over WebSocket.

Binary frame layout (identical bridge -> backend and backend -> browser):

    offset  size  type      field
    0       4     char[4]   magic, b"LVP1"
    4       4     uint32    point count
    8       8     float64   header stamp (seconds)
    16      4     uint32    frame sequence number
    20      2     uint16    frame_id length
    22      2     pad       zero padding (keeps points 4-byte aligned)
    24      n     char[n]   frame_id, zero padded to a multiple of 4
    ...     n*16  float32   x, y, z, intensity per point (little endian)

Bridge hello layout (sent once per TCP connection):

    0       4     char[4]   magic, b"LVS1"
    4       4     uint32    JSON length
    8       n     utf-8     JSON metadata
"""

import collections
import json
import os
import re
import socket
import struct
import threading
import time

MAGIC = b"LVP1"
HELLO_MAGIC = b"LVS1"
HEADER = struct.Struct("<4sIdIH2x")  # 24 bytes
POINT_BYTES = 16
MAX_POINTS = 2000000
LIDAR_TIMEOUT_S = 2.0


def _recv_exact(sock, size):
    """Read exactly ``size`` bytes from ``sock`` or raise EOFError."""
    chunks = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(min(remaining, 1 << 20))
        if not chunk:
            raise EOFError("socket closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class FrameHub(object):
    """Holds the latest frame and the live system metadata."""

    def __init__(self):
        self._cond = threading.Condition()
        self._frame = None
        self._seq = 0
        self._count = 0
        self._stamp = 0.0
        self._frame_id = ""
        self._arrivals = collections.deque(maxlen=30)
        self._last_frame_time = 0.0
        self._total_frames = 0
        self._clients = 0
        self._bytes_sent = 0
        self._started_at = time.time()
        self._bridge_connections = 0
        self._bridge = {
            "connected": False,
            "connected_at": None,
            "peer": None,
            "topic": None,
            "message_type": None,
            "master_uri": os.environ.get("ROS_MASTER_URI", ""),
            "pid": None,
        }

    # -- ingest ---------------------------------------------------------
    def publish(self, frame_bytes, count, stamp, seq, frame_id):
        now = time.time()
        with self._cond:
            self._frame = frame_bytes
            self._seq = seq
            self._count = count
            self._stamp = stamp
            self._frame_id = frame_id
            self._last_frame_time = now
            self._total_frames += 1
            self._arrivals.append(now)
            self._cond.notify_all()

    def wait_for_frame(self, last_seq, timeout=1.0):
        """Block until a frame newer than ``last_seq`` exists.

        Returns ``(frame_bytes, seq)`` or ``(None, last_seq)`` on timeout.
        """
        with self._cond:
            if self._seq <= last_seq and self._frame is None:
                self._cond.wait(timeout)
            if self._seq <= last_seq:
                return None, last_seq
            return self._frame, self._seq

    # -- bridge metadata ------------------------------------------------
    def set_bridge_hello(self, meta):
        with self._cond:
            self._bridge["topic"] = meta.get("topic")
            self._bridge["message_type"] = meta.get("message_type")
            self._bridge["master_uri"] = meta.get("master_uri") or self._bridge["master_uri"]
            self._bridge["pid"] = meta.get("pid")

    def set_bridge_connected(self, connected, peer=None):
        with self._cond:
            if connected:
                self._bridge_connections += 1
                self._bridge["peer"] = peer
                self._bridge["connected_at"] = time.time()
            else:
                self._bridge_connections = max(0, self._bridge_connections - 1)
            self._bridge["connected"] = self._bridge_connections > 0
            if not self._bridge["connected"]:
                self._bridge["peer"] = None
                self._bridge["pid"] = None

    def set_subscriber_count(self, count):
        with self._cond:
            self._bridge["subscriber_count"] = count

    # -- websocket clients ----------------------------------------------
    def client_connected(self):
        with self._cond:
            self._clients += 1

    def client_disconnected(self):
        with self._cond:
            self._clients = max(0, self._clients - 1)

    def add_bytes_sent(self, size):
        with self._cond:
            self._bytes_sent += size

    # -- status ---------------------------------------------------------
    def snapshot(self):
        now = time.time()
        with self._cond:
            arrivals = list(self._arrivals)
            age = (now - self._last_frame_time) if self._last_frame_time else None
            fps = 0.0
            if len(arrivals) >= 2:
                span = arrivals[-1] - arrivals[0]
                if span > 0:
                    fps = (len(arrivals) - 1) / span
            frame_bytes = self._frame
            return {
                "server_time": now,
                "uptime_s": now - self._started_at,
                "ros": {
                    "bridge_connected": self._bridge["connected"],
                    "connected_at": self._bridge["connected_at"],
                    "peer": self._bridge["peer"],
                    "master_uri": self._bridge["master_uri"],
                    "topic": self._bridge["topic"],
                    "message_type": self._bridge["message_type"],
                    "pid": self._bridge["pid"],
                    "subscribers": self._bridge.get("subscriber_count"),
                    "lidar_active": bool(age is not None and age < LIDAR_TIMEOUT_S),
                },
                "cloud": {
                    "seq": self._seq,
                    "points": self._count,
                    "stamp": self._stamp,
                    "frame_id": self._frame_id,
                    "fps": round(fps, 2),
                    "last_frame_age_s": round(age, 3) if age is not None else None,
                    "total_frames": self._total_frames,
                    "frame_bytes": len(frame_bytes) if frame_bytes else 0,
                    "bytes_sent": self._bytes_sent,
                },
                "websocket": {"clients": self._clients},
                "tunnel": tunnel_status(),
                "rviz": {"running": process_running("rviz")},
            }


def process_running(name):
    """Best-effort process check via /proc (Linux, no psutil dependency).

    Only the executable basename is compared, so a directory such as
    ``/home/user/rviz_web_app`` cannot be mistaken for a running ``rviz``.
    """
    try:
        pids = os.listdir("/proc")
    except OSError:
        return False
    needle = name.encode("utf-8", "ignore")
    for pid in pids:
        if not pid.isdigit():
            continue
        try:
            with open("/proc/%s/cmdline" % pid, "rb") as handle:
                cmdline = handle.read()
        except (IOError, OSError):
            continue
        for token in cmdline.split(b"\x00"):
            if token and os.path.basename(token) == needle:
                return True
    return False


_QUICK_TUNNEL_RE = re.compile(r"https://[a-z0-9-]+\.trycloudflare\.com")


def tunnel_status():
    """Report Cloudflare Tunnel state without being required for local access."""
    configured_url = os.environ.get("LIDAR_TUNNEL_URL", "").strip() or None
    log_path = os.environ.get("LIDAR_TUNNEL_LOG", "/tmp/lidar_cf_tunnel.log")
    running = process_running("cloudflared")
    url = configured_url
    if url is None and running:
        # Only trust the log while cloudflared is alive, otherwise an old URL
        # from a previous Quick Tunnel would be shown as if it were current.
        try:
            with open(log_path, "r") as handle:
                matches = _QUICK_TUNNEL_RE.findall(handle.read())
            if matches:
                url = matches[-1]
        except (IOError, OSError):
            pass
    return {"running": running, "url": url, "log": log_path}


class FrameIngestServer(object):
    """Loopback TCP server that receives frames from the ROS bridge."""

    def __init__(self, hub, host="127.0.0.1", port=8765):
        self.hub = hub
        self.host = host
        self.port = port
        self._stop = threading.Event()
        self._socket = None

    def start(self):
        thread = threading.Thread(target=self._serve, name="frame-ingest")
        thread.daemon = True
        thread.start()

    def stop(self):
        self._stop.set()
        try:
            if self._socket is not None:
                self._socket.close()
        except OSError:
            pass

    def _serve(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.host, self.port))
        server.listen(4)
        server.settimeout(1.0)
        self._socket = server
        while not self._stop.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            thread = threading.Thread(
                target=self._handle_client, args=(conn, addr), name="bridge-client"
            )
            thread.daemon = True
            thread.start()

    def _handle_client(self, conn, addr):
        peer = "%s:%s" % (addr[0], addr[1])
        self.hub.set_bridge_connected(True, peer)
        conn.settimeout(10.0)
        try:
            while not self._stop.is_set():
                head = _recv_exact(conn, 4)
                if head == HELLO_MAGIC:
                    length = struct.unpack("<I", _recv_exact(conn, 4))[0]
                    if length > 65536:
                        break
                    payload = _recv_exact(conn, length)
                    try:
                        self.hub.set_bridge_hello(json.loads(payload.decode("utf-8")))
                    except ValueError:
                        pass
                    conn.settimeout(None)
                    continue
                if head != MAGIC:
                    break
                header = head + _recv_exact(conn, HEADER.size - 4)
                _, count, stamp, seq, frame_id_len = HEADER.unpack(header)
                if count > MAX_POINTS:
                    break
                padded_id_len = (frame_id_len + 3) & ~3
                frame_id = _recv_exact(conn, padded_id_len)[:frame_id_len]
                payload_size = count * POINT_BYTES
                payload = _recv_exact(conn, payload_size)
                frame = header + frame_id + b"\x00" * (padded_id_len - frame_id_len) + payload
                self.hub.publish(
                    frame, count, stamp, seq, frame_id.decode("utf-8", "replace")
                )
        except (EOFError, socket.timeout, OSError):
            pass
        finally:
            try:
                conn.close()
            except OSError:
                pass
            self.hub.set_bridge_connected(False)
