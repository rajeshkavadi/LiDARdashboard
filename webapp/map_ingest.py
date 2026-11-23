"""Loopback TCP ingest for the FAST-LIO2 exporter (default port 8766).

This port must never be exposed through Cloudflare; it carries the global map,
pose and trajectory updates and accepts a reset instruction for the exporter.
"""

import json
import socket
import struct
import threading

import protocol

HELLO_MAGIC = b"LVS1"
MAX_MESSAGE_POINTS = 2000000
MAX_HELLO_BYTES = 65536


def _recv_exact(sock, size):
    chunks = []
    remaining = size
    while remaining > 0:
        chunk = sock.recv(min(remaining, 1 << 20))
        if not chunk:
            raise EOFError("socket closed")
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class MapIngestServer(object):
    def __init__(self, hub, host="127.0.0.1", port=8766):
        self.hub = hub
        self.host = host
        self.port = port
        self._stop = threading.Event()
        self._socket = None
        self._clients = set()
        self._lock = threading.Lock()

    def start(self):
        thread = threading.Thread(target=self._serve, name="map-ingest")
        thread.daemon = True
        thread.start()

    def stop(self):
        self._stop.set()
        try:
            if self._socket is not None:
                self._socket.close()
        except OSError:
            pass
        with self._lock:
            clients = list(self._clients)
        for conn in clients:
            try:
                conn.close()
            except OSError:
                pass

    def send_reset(self):
        """Ask connected exporters to clear their voxel/trajectory state."""
        message = protocol.map_reset()
        with self._lock:
            clients = list(self._clients)
        for conn in clients:
            try:
                conn.sendall(message)
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
                target=self._handle, args=(conn, addr), name="slam-client")
            thread.daemon = True
            thread.start()

    def _handle(self, conn, addr):
        peer = "%s:%s" % (addr[0], addr[1])
        with self._lock:
            self._clients.add(conn)
        self.hub.set_slam_connection(True, peer)
        conn.settimeout(10.0)
        try:
            while not self._stop.is_set():
                head = _recv_exact(conn, 4)
                if head == HELLO_MAGIC:
                    length = struct.unpack("<I", _recv_exact(conn, 4))[0]
                    if length > MAX_HELLO_BYTES:
                        break
                    try:
                        meta = json.loads(_recv_exact(conn, length).decode("utf-8"))
                    except ValueError:
                        meta = {}
                    meta["peer"] = peer
                    self.hub.set_slam_connection(True, peer, meta)
                    conn.settimeout(None)
                    continue
                if head != protocol.MAP_MAGIC:
                    break
                msg_type = _recv_exact(conn, 1)[0]
                # 3 pad bytes
                _recv_exact(conn, 3)

                if msg_type == protocol.MAP_RESET:
                    # Exporters are not expected to send this; ignore.
                    continue
                if msg_type == protocol.MAP_POSE:
                    payload = _recv_exact(conn, protocol.MAP_POSE_STRUCT.size)
                    values = protocol.parse_pose(memoryview(payload))
                    if values:
                        self.hub.set_pose(values)
                    continue
                if msg_type == protocol.MAP_TRAJECTORY:
                    header = _recv_exact(conn, protocol.MAP_TRAJ_HEADER.size)
                    start, count = struct.unpack("<II", header)
                    if count > MAX_MESSAGE_POINTS:
                        break
                    body = _recv_exact(
                        conn, count * protocol.TRAJ_POINT_BYTES)
                    self.hub.append_trajectory(start, count, body)
                    continue
                if msg_type == protocol.MAP_TILE_ADD:
                    header = _recv_exact(conn, protocol.MAP_TILE_HEADER.size)
                    tx, ty, tz, count = struct.unpack("<iiiI", header)
                    if count > MAX_MESSAGE_POINTS:
                        break
                    body = _recv_exact(conn, count * protocol.POINT_BYTES)
                    self.hub.append_tile(tx, ty, tz, count, body)
                    continue
                break
        except (EOFError, socket.timeout, OSError):
            pass
        finally:
            with self._lock:
                self._clients.discard(conn)
            try:
                conn.close()
            except OSError:
                pass
            self.hub.set_slam_connection(False)
