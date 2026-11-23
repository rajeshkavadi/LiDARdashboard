"""Automatic Cloudflare Quick Tunnel for the Flask dashboard.

Starts ``cloudflared tunnel --url http://127.0.0.1:<port>`` as a child process,
captures the generated public URL from its log, exposes it through
``/api/status`` and stops the child cleanly on shutdown.  Only the Flask port
is proxied; the ROS ingest port (8765/8766) is never part of the tunnel.

The dashboard remains fully usable locally when cloudflared is missing,
offline, or disconnected.
"""

import atexit
import os
import re
import shutil
import subprocess
import threading
import time

QUICK_URL_RE = re.compile(r"https://[a-z0-9-]+\.trycloudflare\.com")
INSTALL_HINT = (
    "Install cloudflared:\n"
    "  https://developers.cloudflare.com/cloudflare-one/connections/"
    "connect-networks/downloads/\n"
    "  Debian/Ubuntu example:\n"
    "    curl -L -o cloudflared https://github.com/cloudflare/cloudflared/"
    "releases/latest/download/cloudflared-linux-amd64\n"
    "    chmod +x cloudflared && sudo mv cloudflared /usr/local/bin/"
)


class CloudflareTunnel(object):
    def __init__(self, port=5000, log_path=None, disabled=False,
                 max_restarts=5, on_ready=None):
        self.port = int(port)
        self.log_path = log_path or os.environ.get(
            "LIDAR_TUNNEL_LOG", "/tmp/lidar_cf_tunnel.log")
        self.disabled = bool(disabled) or os.environ.get(
            "LIDAR_TUNNEL_DISABLE", "").lower() in ("1", "true", "yes")
        self.max_restarts = int(max_restarts)
        self._on_ready = on_ready
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._process = None
        self._reader = None
        self._monitor = None
        self._state = "disabled" if self.disabled else "starting"
        self._url = None
        self._connected = False
        self._error = None
        self._pid = None
        self._started_at = None
        self._restarts = 0
        self._bin = shutil.which("cloudflared")
        self._logged_url = None
        atexit.register(self.stop)

    # -- lifecycle ------------------------------------------------------
    def start(self):
        if self.disabled:
            return
        if not self._bin:
            with self._lock:
                self._state = "unavailable"
                self._error = "cloudflared not found in PATH"
            print("[tunnel] cloudflared is not installed.")
            print(INSTALL_HINT)
            return
        try:
            self._open_log()
        except OSError as exc:
            print("[tunnel] cannot write %s: %s" % (self.log_path, exc))
        self._monitor = threading.Thread(
            target=self._monitor_loop, name="cloudflared-monitor")
        self._monitor.daemon = True
        self._monitor.start()

    def _open_log(self):
        directory = os.path.dirname(self.log_path)
        if directory and not os.path.isdir(directory):
            os.makedirs(directory)
        with open(self.log_path, "w") as handle:
            handle.write("[tunnel] starting cloudflared for 127.0.0.1:%d\n" %
                         self.port)

    def _spawn(self):
        cmd = [self._bin, "tunnel", "--url",
               "http://127.0.0.1:%d" % self.port, "--no-autoupdate"]
        with self._lock:
            if self._stop.is_set():
                return False
            self._state = "starting"
            self._url = None
            self._connected = False
            self._error = None
            self._started_at = time.time()
        try:
            process = subprocess.Popen(
                cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                universal_newlines=True, bufsize=1)
        except OSError as exc:
            with self._lock:
                self._state = "error"
                self._error = str(exc)
            return False
        with self._lock:
            self._process = process
            self._pid = process.pid
        self._reader = threading.Thread(
            target=self._read_output, args=(process,),
            name="cloudflared-log")
        self._reader.daemon = True
        self._reader.start()
        return True

    def _monitor_loop(self):
        attempts = 0
        while not self._stop.is_set():
            if self._process is None:
                if not self._spawn():
                    self._stop.wait(5.0)
                    continue
            process = self._process
            while process.poll() is None and not self._stop.is_set():
                self._stop.wait(0.5)
            if self._stop.is_set():
                break
            code = process.returncode
            with self._lock:
                self._connected = False
                self._url = None
                self._state = "disconnected"
                self._error = "cloudflared exited (code %s)" % code
                self._process = None
                self._pid = None
                ran_for = time.time() - (self._started_at or time.time())
            if ran_for > 60:
                attempts = 0
            attempts += 1
            self._restarts += 1
            if attempts > self.max_restarts:
                with self._lock:
                    self._state = "error"
                    self._error = ("cloudflared keeps exiting; giving up "
                                   "after %d restarts" % self.max_restarts)
                print("[tunnel] WARNING: %s" % self._error)
                return
            delay = min(30, 5 * attempts)
            print("[tunnel] cloudflared stopped (code %s); restarting in %ds"
                  % (code, delay))
            self._stop.wait(delay)

    def _read_output(self, process):
        try:
            with open(self.log_path, "a") as log:
                for line in iter(process.stdout.readline, ""):
                    if not line:
                        break
                    log.write(line)
                    log.flush()
                    self._handle_line(line.rstrip("\n"))
        except (OSError, ValueError):
            pass
        finally:
            try:
                process.stdout.close()
            except (OSError, ValueError):
                pass

    def _handle_line(self, line):
        match = QUICK_URL_RE.search(line)
        if match:
            with self._lock:
                self._url = match.group(0)
                if self._state in ("starting", "connecting"):
                    self._state = "connecting"
            if self._logged_url != match.group(0):
                self._logged_url = match.group(0)
                print("[tunnel] public URL: %s" % match.group(0))
                if self._on_ready:
                    try:
                        self._on_ready(self.status())
                    except Exception as exc:  # noqa: BLE001
                        print("[tunnel] ready callback failed: %s" % exc)
        if "Registered tunnel connection" in line:
            with self._lock:
                self._connected = True
                self._state = "connected"
            if self._url:
                print("[tunnel] Cloudflare status: CONNECTED")
        if ("Unregistered tunnel connection" in line or
                "failed to establish connection" in line or
                "connection refused" in line):
            with self._lock:
                self._connected = False
                if self._state == "connected":
                    self._state = "reconnecting"

    def stop(self):
        self._stop.set()
        process = self._process
        if process is None:
            return
        try:
            process.terminate()
            for _ in range(20):
                if process.poll() is not None:
                    break
                time.sleep(0.1)
            if process.poll() is None:
                process.kill()
        except OSError:
            pass
        with self._lock:
            if self._state != "disabled":
                self._state = "stopped"
            self._connected = False
            self._url = None
            self._process = None
            self._pid = None
        print("[tunnel] cloudflared stopped")

    # -- status ---------------------------------------------------------
    def status(self):
        with self._lock:
            return {
                "installed": bool(self._bin),
                "enabled": not self.disabled,
                "state": self._state,
                "running": bool(self._process is not None and
                                self._process.poll() is None),
                "connected": self._connected,
                "url": self._url,
                "pid": self._pid,
                "started_at": self._started_at,
                "restarts": self._restarts,
                "error": self._error,
                "log": self.log_path,
                "install_hint": None if self._bin else INSTALL_HINT,
            }
