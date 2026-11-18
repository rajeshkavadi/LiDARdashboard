# LiDAR Live — Livox Mid-360 Web Point-Cloud Viewer

Browser-based, real-time, multi-directional visualisation of a Livox LiDAR
point cloud. It runs alongside the existing ROS Melodic driver and RViz and
adds:

- **Dashboard** — LiDAR, ROS bridge, stream, WebSocket, tunnel and RViz status.
- **RViz** — the native RViz workflow, unchanged and monitored from the page.
- **View** — a Three.js point-cloud viewer with FRONT / BACK / LEFT / RIGHT /
  **TOP** / FREE camera modes, 5 m range rings in TOP, Height / Intensity /
  **Distance** point colouring, live FPS and point counts.

This is **visualisation only**. There is no object detection, no
PointPillars, no inference, no bounding boxes and no classification anywhere
in this project. Point-cloud data is never uploaded for cloud processing; the
Cloudflare Tunnel only proxies the local web app.

---

## 1. Verified environment and data source

| Item | Value |
| --- | --- |
| OS | Ubuntu 18.04 |
| ROS | Melodic (`/opt/ros/melodic`, Python 2.7 rospy) |
| Web runtime | Python 3.6.9 |
| LiDAR | Livox Mid-360 |
| ROS topic | `/livox/lidar` |
| Message type | `sensor_msgs/PointCloud2` |
| Frame | `livox_frame` |
| Fields | `x`, `y`, `z`, `intensity` (float32), plus `tag`, `line`, `timestamp` |
| Point step | 26 bytes |
| Rate | 10 Hz |
| Points per frame | ~20,000 |
| Recording | `20261030_221123_mid360s.bag` (3:38, 1.1 GB, 2,187 LiDAR frames) |

The point-cloud topic, message type, field offsets, frame id and rate above
were read from the supplied bag; nothing here assumes a different topic.

**Coordinate frame.** `livox_frame` follows the standard ROS / Livox
convention: **X forward, Y left, Z up**. The camera presets use exactly this
convention. If your sensor is mounted rotated on the robot, change the `VIEWS`
table in `webapp/static/js/viewer.js` (one line per direction) rather than
rotating the data.

---

## 2. Architecture

```
Livox Mid-360
      │  (unchanged)
      ▼
Existing Livox ROS driver ──────────────► RViz (unchanged)
      │
      ▼  /livox/lidar  sensor_msgs/PointCloud2
ros_bridge.py  (ROS Melodic, Python 2.7, read-only subscriber)
      │  binary frames over loopback TCP 127.0.0.1:8765
      ▼
webapp/app.py + frame_hub.py  (Python 3, Flask + WebSocket)
      │  binary WebSocket /ws/points
      ├──────────────► http://localhost:5000   (local browser)
      └── cloudflared ─► https://…trycloudflare.com  (remote browser)
```

### Why a Python 2 bridge?

ROS Melodic's `rospy` only runs on Python 2.7, while the modern web stack
runs on Python 3.6. Instead of installing a second ROS stack, `ros_bridge.py`
is a small Python 2.7 node that subscribes to the existing topic and forwards
frames to the Python 3 web process over a loopback socket. This keeps the
driver, topic and RViz untouched and avoids duplicate drivers or nodes.

The bridge subscribes with `queue_size=1`, so a slow or disconnected browser
can never back up the ROS callback. Only the two newest frames are queued for
transmission; older ones are dropped.

### Binary frame protocol

Identical from bridge → backend and backend → browser:

| Offset | Size | Type | Field |
| --- | --- | --- | --- |
| 0 | 4 | char[4] | magic `LVP1` |
| 4 | 4 | uint32 | point count |
| 8 | 8 | float64 | header stamp (s) |
| 16 | 4 | uint32 | sequence number |
| 20 | 2 | uint16 | frame_id length |
| 22 | 2 | – | zero padding |
| 24 | n | char[n] | frame_id, padded to 4 bytes |
| … | n×16 | float32 | x, y, z, intensity |

No JSON is used for point data. The browser parses the header with
`DataView`, views the payload as a `Float32Array` (4-byte aligned, no copy),
and updates one existing `BufferGeometry` with `updateRange` so only changed
ranges are re-uploaded.

### Streaming behaviour

- Backend keeps **one** latest frame. Every WebSocket client receives the
  newest frame available; slow clients skip frames instead of queueing them.
- Each client is handled in its own thread, so a stuck browser cannot block
  ingest or other viewers.
- The frontend reconnects automatically with exponential backoff (1 s → 10 s)
  and works over `ws://` locally and `wss://` through the tunnel.
- WebSocket URLs are derived from `window.location`; no tunnel URL is hardcoded.

---

## 3. Project structure

```
rviz_web_app/
├── 20261030_221123_mid360s.bag          # recording (unmodified)
├── README.md
├── requirements.txt
├── scripts/
│   ├── start_web.sh                     # Flask + WebSocket backend
│   ├── start_bridge.sh                  # ROS → web bridge (Python 2.7)
│   ├── start_tunnel.sh                  # Cloudflare Quick Tunnel
│   └── play_bag.sh                      # replay the recording for testing
├── webapp/
│   ├── app.py                           # Flask app, /api/status, /ws/points
│   ├── frame_hub.py                     # loopback ingest, latest-frame hub, status
│   ├── templates/
│   │   └── index.html                   # Dashboard / RViz / View tabs
│   └── static/
│       ├── css/style.css
│       ├── js/app.js                    # tabs, status polling, WS client
│       ├── js/viewer.js                 # Three.js viewer + camera presets
│       └── vendor/
│           ├── three.min.js             # Three.js r128 (vendored, offline)
│           └── OrbitControls.js         # r128 OrbitControls
└── ros_pkg/
    └── lidar_web_bridge/
        ├── package.xml
        ├── CMakeLists.txt
        ├── launch/lidar_web.launch
        ├── rviz/lidar_view.rviz
        └── scripts/ros_bridge.py        # Python 2.7 rospy node
```

**Newly created:** everything listed above except the bag.
**Modified existing files:** none — the original directory contained only the
bag. **Unmodified:** `20261030_221123_mid360s.bag`, the Livox driver, and any
existing RViz configuration.

---

## 4. Install dependencies

### 4.1 ROS / bridge (already required by the existing setup)

```bash
sudo apt install ros-melodic-rviz python-numpy
python -c "import rospy, numpy, sensor_msgs; print('bridge deps OK')"
```

### 4.2 Web backend (Python 3.6)

```bash
cd ~/rviz_web_app
python3 -m pip install --user -r requirements.txt
```

Pins are intentional for Ubuntu 18.04's Python 3.6 (Flask 2.0.3 / Werkzeug
2.0.3 / flask-sock 0.7.0). Three.js is vendored under
`webapp/static/vendor/`, so the browser needs no internet access and there is
no Node/npm build step.

### 4.3 Cloudflare Tunnel

`cloudflared` is already present at `~/.local/bin/cloudflared`. If installing
on another machine:

```bash
# Debian package (recommended)
curl -L -o /tmp/cloudflared.deb \
  https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-amd64.deb
sudo dpkg -i /tmp/cloudflared.deb
cloudflared --version
```

---

## 5. Start the system

Run each component in its own terminal.

### 5.0 Quick start — four terminals + a browser

Exact launch order, verified with `20261030_221123_mid360s.bag`. Skip
Terminal 4 and start the real driver (5.1) instead when the sensor is in use.

**Terminal 1 — ROS master**

```bash
source /opt/ros/melodic/setup.bash
roscore
```

**Terminal 2 — web backend**

```bash
cd ~/rviz_web_app
./scripts/start_web.sh
```

Wait for `Running on http://127.0.0.1:5000`.

**Terminal 3 — ROS → web bridge**

```bash
cd ~/rviz_web_app
./scripts/start_bridge.sh
```

Wait for `subscribed to /livox/lidar` and `connected to web backend`.

**Terminal 4 — replay the bag**

```bash
cd ~/rviz_web_app
./scripts/play_bag.sh
```

It loops the bag at 1× (~20,000 points/frame, ~10 FPS).

**Browser — open the dashboard**

```bash
# IMPORTANT: clear the snap-terminal GL variables (see 11. WebGL note)
env -u LIBGL_DRIVERS_PATH -u LD_LIBRARY_PATH firefox http://localhost:5000
```

Launching Firefox from the desktop app menu also works. Hard-refresh with
**Ctrl+Shift+R** after updating the app. Then:

- **Dashboard** — LiDAR `Streaming`, Bridge `online`, WebSocket `Connected`,
  ~20,000 points at ~10 FPS.
- **View** — the cloud appears within a second. Use FRONT / BACK / LEFT /
  RIGHT / **TOP** / FREE VIEW, and the Color dropdown for Height, Intensity
  or **Distance** (see section 7).

Stop in reverse order: Terminal 4 (bag), 3 (bridge), 2 (backend), 1 (roscore).

### 5.1 LiDAR driver (existing, unchanged)

```bash
source /opt/ros/melodic/setup.bash
# Start your existing Livox driver exactly as before, e.g.:
# roslaunch livox_ros_driver livox_lidar_msg.launch
```

Do not start a second driver. RViz can run at the same time as before:

```bash
rviz -d ~/rviz_web_app/ros_pkg/lidar_web_bridge/rviz/lidar_view.rviz
```

### 5.2 Web backend

```bash
cd ~/rviz_web_app
./scripts/start_web.sh
```

Defaults: HTTP + WebSocket on `127.0.0.1:5000`, ingest on `127.0.0.1:8765`.
Environment overrides: `LIDAR_WEB_HOST`, `LIDAR_WEB_PORT`,
`LIDAR_INGEST_PORT`. Local dashboard: **http://localhost:5000**

The dashboard works locally even with no ROS running and with the tunnel off.

### 5.3 ROS bridge

```bash
cd ~/rviz_web_app
./scripts/start_bridge.sh
```

This sources ROS Melodic, adds `ros_pkg/` to `ROS_PACKAGE_PATH`, and runs
`rosrun lidar_web_bridge ros_bridge.py`. Overrides: `TOPIC` (default
`/livox/lidar`), `POINT_STRIDE` (default 1), `LIDAR_INGEST_PORT`.

Equivalent launch file (bridge, plus RViz optionally):

```bash
export ROS_PACKAGE_PATH=$HOME/rviz_web_app/ros_pkg:$ROS_PACKAGE_PATH
roslaunch lidar_web_bridge lidar_web.launch rviz:=true
```

`roslaunch` arguments: `topic:=`, `point_stride:=`, `max_points:=`,
`rviz:=true|false`.

> Run only one bridge node at a time. ROS will shut down an older node when a
> second node with the same name registers.

### 5.4 Testing without the LiDAR

With `roscore` and the bridge running:

```bash
cd ~/rviz_web_app
./scripts/play_bag.sh             # loops the supplied bag at 1×
./scripts/play_bag.sh 20261030_221123_mid360s.bag -r 2 -u 30   # faster, 30 s
```

Replaying the bag does not touch the live driver; stop it when the real
sensor is in use.

---

## 6. Verify the point-cloud topic

```bash
source /opt/ros/melodic/setup.bash

rostopic info /livox/lidar          # type: sensor_msgs/PointCloud2
rostopic hz /livox/lidar            # expect ~10 Hz
rostopic echo -n1 /livox/lidar/header
rostopic echo -n1 /livox/lidar/fields
```

Expected fields: `x`, `y`, `z` (float32 at offsets 0/4/8), `intensity`
(float32 at 12), `tag`, `line`, `timestamp`. If the topic differs, start the
bridge with `TOPIC:=/your/topic` or `roslaunch ... topic:=/your/topic`.

Check the backend sees frames:

```bash
curl -s http://localhost:5000/api/status | python3 -m json.tool
```

Look for `ros.lidar_active: true`, `cloud.points ≈ 20000`, `cloud.fps ≈ 10`.

---

## 7. Using the View tab

| Button | Camera position | Direction |
| --- | --- | --- |
| FRONT | LiDAR origin `(0,0,0)` | +X forward |
| BACK | LiDAR origin `(0,0,0)` | −X backward |
| LEFT | LiDAR origin `(0,0,0)` | +Y left |
| RIGHT | LiDAR origin `(0,0,0)` | −Y right |
| TOP | orthographic overhead map | straight down (−Z) |
| FREE VIEW | orbit camera (pulled back) | user controlled |

All four directional views use the **same** live point cloud and the **same**
camera position; only orientation changes.

TOP is a true overhead map: an orthographic camera looks straight down, with
+X up the screen and +Y to the left. It draws concentric **distance rings on
the ground plane every 5 m (5, 10, 15 … 40 m)**, each labelled with its radius
in metres, so you can read range directly off the scene. Rings are shown in
TOP only and are drawn on top of the points so they stay visible. The default
framing fits the **5 m ring**, keeping the near field readable; scroll down to
zoom out and reveal the 10, 15 … 40 m rings as needed. Switching views never
reconnects the WebSocket or pauses the stream.

FREE VIEW enables OrbitControls: left-drag rotates, wheel zooms, right-drag
pans. When entering FREE VIEW the camera keeps the current look direction but
moves 24 m backwards so it can orbit the origin.

Other controls:

- **Color** — **Height (Z)** with an adaptive height scale, **Intensity**
  (`intensity / 255`), or **Distance** (3-D range `√(x²+y²+z²)`), scaled to the
  99th-percentile range of the frame so outliers do not flatten the ramp. The
  legend switches to `DISTANCE` and shows 0 m → max range; intensity has no
  legend because it is a raw 0–255 value.
- **Zoom** — spin the mouse wheel over the scene. FRONT / BACK / LEFT / RIGHT
  zoom by narrowing the field of view (the camera stays at the LiDAR origin);
  TOP zooms the orthographic scale, so the rings spread out but stay circular.
  Ranges are 0.15× to 8× (0.15× is wide enough to frame the full 40 m ring set)
  and every view switch resets to the 5 m default framing.
  In FREE VIEW the wheel is OrbitControls' dolly, as before.
- **Max points** — 50k / 120k / 250k / unlimited. Frames above the limit are
  decimated with a stride before touching the GPU buffers.
- HUD — view label, FPS, rendered points, WebSocket and LiDAR indicators.
- Scene — colored points, XY reference grid, RGB axes (X red, Y green,
  Z blue), LiDAR origin marker.

### What the colour modes mean

Points arrive as `(x, y, z, intensity)` in the sensor frame `livox_frame`:
**X forward, Y left, Z up**, in metres, origin at the LiDAR.

| Mode | Value used | Notes |
| --- | --- | --- |
| Height (Z) | `z` | Height **relative to the sensor**, not the floor. The min/max adapt to the frame with smoothing, so the ramp does not flicker. |
| Intensity | `intensity` (0–255) | Raw return strength reported by the Livox driver, coloured as `intensity / 255`. No legend, because it is a raw value. |
| Distance | `√(x² + y² + z²)` | True 3-D range in metres. Coloured `range / dMax`, where `dMax` is the frame’s 99th-percentile range (smoothed) so a few stray long returns cannot flatten the ramp. Legend shows `0 m → dMax`, e.g. `0.0 m … 41.4 m`. |

**Reflectance / reflectivity** is a physical surface property — the fraction of
emitted light that returns. In this data the `intensity` field is the
uncalibrated echo strength, not calibrated reflectance: it also drops with
range and with grazing incidence angle. Read it as “how strong was this echo”,
not as a material constant.

Note that the TOP **rings** are **horizontal** distance drawn on the ground
plane (Z=0), while the Distance colour mode uses the true 3-D range. A point
high above the sensor appears slightly outside its ground radius in TOP; the
colour ramp is the exact range.

### Testing the TOP view

1. With the cloud streaming, click **View → TOP**.
2. Confirm the readout reads `TOP · −Z`, +X points up the screen and +Y points
   left, and the scene is a top-down map centred on the LiDAR marker.
3. The default framing shows the 5 m ring; scroll down to reveal `10 m` …
   `40 m` and confirm the rings are evenly spaced and labelled.
4. Scroll up over the map: the rings grow and their labels stay attached and
   readable; scroll back down to see the outer rings again. Confirm the rings
   stay circular at every zoom level.
5. Switch to FRONT: the rings disappear and the zoom resets. Switch back to
   TOP: rings return at the default framing, with the stream still live
   (FPS/points keep updating).

### Testing the Distance colour mode

1. Set **Color → Distance**; the legend title becomes `DISTANCE` and shows
   `0.0 m` to the current maximum around 40 m.
2. Confirm near points are blue/cyan and the furthest returns are yellow/red,
   and that the ramp is stable frame to frame.
3. Switch to **Intensity**: the legend hides. Switch to **Height**: the legend
   returns as `HEIGHT` with signed metres, e.g. `-1.4 m … +18.7 m`.

### Testing the directional views

1. Start the backend, bridge and a live or replayed cloud.
2. Open `http://localhost:5000`, click **View**, select **FRONT**.
3. Confirm the scene in front of the sensor fills the view and the label reads
   `FRONT · +X`; the HUD FPS/points stay live (no freeze or reconnect).
4. Place an asymmetric object ahead of the sensor. FRONT shows it ahead; BACK
   shows it behind; LEFT/RIGHT show it on the respective side. Ensure the view
   is from the sensor's position (points radiate outward from the centre).
5. Switch back and forth while watching the FPS counter: it must not pause.
6. Scroll up/down in FRONT: the scene zooms in/out without the camera moving
   from the origin (near points stay centred); switching views resets it.

### Testing FREE VIEW

1. Click **FREE VIEW**; the button turns amber and the control hint appears.
2. Drag to rotate, wheel to zoom, right-drag to pan.
3. Confirm directional buttons switch back to origin-based views immediately.

---

## 8. Cloudflare Tunnel

### 8.1 Quick Tunnel (current workflow, no Cloudflare account)

With the backend already running:

```bash
cd ~/rviz_web_app
./scripts/start_tunnel.sh
```

The public `https://<random>.trycloudflare.com` URL is printed and written to
`/tmp/lidar_cf_tunnel.log`. The dashboard reads that log and shows the same URL
under **Cloudflare Tunnel**.

- The URL is temporary and changes on every restart — it is never hardcoded.
- WebSocket uses `wss://` automatically because the page is served over HTTPS.
- Start/stop the tunnel independently of ROS and the LiDAR; the local URL keeps
  working when the tunnel is down.
- Nothing is processed in the cloud: cloudflared only proxies HTTP/WebSocket
  to `localhost:5000`.

### 8.2 Named tunnel (stable hostname, optional)

```bash
cloudflared tunnel login
cloudflared tunnel create lidar-web
# ~/.cloudflared/config.yml
#   tunnel: lidar-web
#   credentials-file: /home/<user>/.cloudflared/<tunnel-id>.json
#   ingress:
#     - hostname: lidar.example.com
#       service: http://localhost:5000
#     - service: http_status:404
cloudflared tunnel route dns lidar-web lidar.example.com
cloudflared tunnel run lidar-web
```

Then set `LIDAR_TUNNEL_URL=https://lidar.example.com` when starting the backend
so the dashboard displays the stable URL.

---

## 9. Testing procedures

### WebSocket connection

1. Open DevTools → Network → WS → `ws/points`. Status 101; messages are
   binary, ~300–330 KB each, at ~10/s.
2. Stop the backend: the WS status turns red and the page starts retrying.
   Restart the backend: it reconnects automatically within a few seconds.
3. Stop and restart the tunnel: remote pages reconnect with `wss://`.
4. Optional CLI check on the machine:

```bash
python3 -m pip install --user websocket-client==0.59.0
python3 - <<'PY'
import struct, websocket
ws = websocket.create_connection('ws://127.0.0.1:5000/ws/points', timeout=10)
data = ws.recv()
magic, count, stamp, seq, flen = struct.unpack('<4sIdIH', data[:22])
print(magic, count, seq, data[24:24+flen], len(data))
ws.close()
PY
```

Expected: `b'LVP1' 20064 <seq> b'livox_frame' 321060`.

### Performance

1. Watch the View HUD FPS and `cloud.fps` in `/api/status`; both should track
   the LiDAR rate (~10 fps at 1× playback).
2. Watch CPU with `top -p $(pgrep -f 'webapp/app.py')` and browser GPU memory.
   One geometry and two attributes are reused; capacity is only reallocated if
   a frame exceeds it.
3. Over a slow remote link, set **Max points** to 50k or start the bridge with
   `POINT_STRIDE=2` to halve bandwidth.
4. Confirm `cloud.bytes_sent` grows steadily and `cloud.total_frames` keeps
   increasing with no unbounded memory growth.

---

## 10. Troubleshooting

| Symptom | Check |
| --- | --- |
| Dashboard shows “No frames” | `rostopic hz /livox/lidar`; bridge log; topic name |
| Bridge offline in dashboard | backend running and on ingest port 8765; `scripts/start_bridge.sh` log |
| WS reconnecting | backend process; tunnel logs; firewall |
| Tunnel URL missing | `/tmp/lidar_cf_tunnel.log`; `cloudflared` in `PATH` |
| “3D view unavailable” / empty view | browser has no WebGL; see note below |
| Empty black view | WebGL enabled; browser console; frames arriving |
| Wrong directions | sensor mount rotation → adjust `VIEWS` in `viewer.js` |
| RViz unaffected | this project never modifies the driver or RViz |

---


## 11. WebGL note (important on this machine)

The 3D view needs WebGL. This desktop runs a snap-packaged terminal that
exports `LIBGL_DRIVERS_PATH` and `LD_LIBRARY_PATH` pointing into
`/snap/alacritty/.../dri`; a browser launched from such a terminal inherits
them and cannot create a WebGL context (the app then shows “3D view
unavailable”). Launch the browser from the desktop, or clear those variables:

```bash
env -u LIBGL_DRIVERS_PATH -u LD_LIBRARY_PATH firefox http://localhost:5000
```

Verify with `http://localhost:5000` after launching this way; the View HUD
should show live points and ~10 FPS.

---

## 12. Scope

Intentionally excluded: object detection, PointPillars, ML/inference,
3D bounding boxes, classification, tracking, cloud-side processing, and any
modification of the Livox driver or existing RViz workflow.

