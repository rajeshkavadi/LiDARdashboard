# Project context — LiDAR web dashboard (Livox Mid-360)

Snapshot of the work done on this repo so far. Written 2026-11-18.

## What this project is

A **read-only** web dashboard for a Livox Mid-360 LiDAR:

- **RViz** stays the ground-truth viewer and is left untouched.
- A **ROS Python 2 bridge** (`ros_pkg/lidar_web_bridge/scripts/ros_bridge.py`)
  subscribes to `/livox/lidar` (`sensor_msgs/PointCloud2`) and streams binary
  frames over TCP to the Flask backend.
- **Flask** backend (`webapp/app.py`) relays those frames over a **binary
  WebSocket** to the browser and serves the UI.
- **Three.js** front end (`webapp/static/js/viewer.js`, `app.js`) renders the
  cloud with three fixed camera views and a free orbit view.
- Optional Cloudflare Tunnel proxies **only the web app**.
- Explicit non-goals: object detection / ML, modifying RViz, text/JSON point
  transport (binary only).

Project root: `/home/rajesh/rviz_web_app` (not a git repo).

## How to run everything

Four terminals:

1. `roscore`
2. `./scripts/start_web.sh` — Flask on `http://127.0.0.1:5000`
3. `./scripts/start_bridge.sh` — ROS bridge, TCP ingest on `127.0.0.1:8765`
4. `./scripts/play_bag.sh` — plays `20261030_221123_mid360s.bag` on loop

Then open `http://127.0.0.1:5000` and hard-refresh (**Ctrl+Shift+R**) after
front-end edits. Full details (including tunnel setup and tests) are in
`README.md` §5.0 and §7.

## Files that matter

| File | Role |
| --- | --- |
| `webapp/static/js/viewer.js` | All rendering: cameras, views, point buffers, colour modes, range rings, wheel zoom. |
| `webapp/static/js/app.js` | UI state, view buttons, legend, WebGL fallback, HUD stats, WebSocket handling. |
| `webapp/templates/index.html` | View buttons (FRONT/BACK/LEFT/RIGHT/TOP/FREE VIEW), colour select, max-points select, `legend-title`. |
| `webapp/static/css/style.css` | Layout; `.view-buttons { flex-wrap: wrap }` so six buttons fit. |
| `ros_pkg/lidar_web_bridge/scripts/ros_bridge.py` | Extracts `x, y, z, intensity`; clips intensity to 0–255; optional `~point_stride` / `~max_points`. |
| `README.md` | Full documentation: quick start, architecture, protocol, views, colour semantics, zoom, tests. |

## Features implemented in this session

### TOP view (overhead map)

- Orthographic camera looking straight down (−Z); **+X up the screen, +Y to the
  left**.
- Concentric **distance rings every 5 m** (5 … 40 m) drawn as `THREE.Line`
  loops at z = 0, with canvas-sprite labels at 45° between the axes.
- Rings and labels are visible **only in TOP**.
- `TOP_HALF = 6` (ortho half-height in metres): the **default framing fits the
  5 m ring**, per request. Wider rings are off-screen until the user zooms out.
- `grid` is a 120 × 60 `GridHelper` (2 m cells), rotated into the XY plane.

### Distance colour mode

- Added "Distance" beside Height and Intensity.
- Value = true 3-D range `√(x² + y² + z²)`.
- Scaled to a **smoothed 99th-percentile range** computed from a 1 m histogram,
  so a handful of stray long returns cannot flatten the ramp.
- Legend switches to `DISTANCE`, 0 m → current max range (~40 m typical).
- Height mode: `z` relative to sensor, adaptive min/max smoothed at 0.12.
- Intensity mode: `intensity / 255`, no legend (raw value).

### Wheel zoom in fixed views

- Wheel listener on the canvas. **FREE VIEW is untouched** (OrbitControls dolly).
- **FRONT / BACK / LEFT / RIGHT**: zoom by narrowing the perspective FOV.
  `BASE_FOV = 72`; camera stays exactly at the LiDAR origin.
- **TOP**: scales `topCamera.zoom`.
- `ZOOM_MIN = 0.15` → TOP half-height 40 m, enough to frame the entire ring set.
- `ZOOM_MAX = 8` → TOP half-height 0.75 m.
- Every `setView()` resets zoom to 1, so switching views returns to the default
  framing.
- Ring labels are rescaled as `1.7 / zoom` by `0.85 / zoom` so they keep a
  **constant on-screen size** at all zoom levels.
- View subtitles end with "· scroll to zoom" for all five fixed views.

## Colour / data semantics (answers given to the user)

- **Intensity is not computed by this app.** The Livox driver publishes a
  per-point `intensity` (reflectivity code, ~0–255). The bridge clips it with
  `np.clip(intensity, 0, 255)` and the viewer colours `intensity / 255`.
  It is raw echo strength, not calibrated surface reflectance — it falls off
  with range and grazing incidence, so it is not a material constant.
- **Max points** (50k / 120k / 250k / Unlimited) is a **browser-side per-frame
  cap** (`viewer.setMaxPoints`). Larger frames are decimated by stride before
  the GPU buffers are filled; HUD "pts" shows the rendered count. The test bag
  yields ~20k points/frame, below every option, so changing it makes no visible
  difference. It matters only for denser clouds (multiple sensors, higher
  rates, Unlimited recordings).

## Verification performed

- `node --check` on both JS files after every edit.
- Live end-to-end test with the bag replaying in headless Firefox on CDP 9234:
  - TOP wheel: `topCamera.zoom` 1 → 1.54 after two notches; clamps at 0.4 (now
    0.15) and 8; resets to exactly 1 on view switch.
  - FRONT wheel: FOV 72° → 60.7° after one notch, camera still at origin.
  - Default TOP screenshot: 5 m ring fills the frame, ~20k points, ~16 FPS.
  - After zooming fully out: all rings 5–40 m visible and labelled, ~7–8 FPS.
  - Rings stay circular at every zoom; labels readable and constant-sized.
- No console errors; ~19.9k–20.3k points per frame.

## Testing gotchas (reusable knowledge)

- Firefox here is a **snap** with a polluted environment. Clean launch:
  ```bash
  mkdir -p /home/rajesh/rviz_web_app/fftest_profile
  env -u LIBGL_DRIVERS_PATH -u LD_LIBRARY_PATH -u LIBVA_DRIVERS_PATH \
      -u __EGL_VENDOR_LIBRARY_DIRS MOZ_HEADLESS=1 \
      firefox -headless -no-remote \
      -profile /home/rajesh/rviz_web_app/fftest_profile \
      --remote-debugging-port 9234 http://127.0.0.1:5000
  ```
  (Snap Firefox cannot create profiles under `/tmp` or hidden directories.)
- Use the Python `websocket-client` helper against `http://127.0.0.1:9234/json`,
  with `suppress_origin=True`, and enable `Runtime` + `Page`.
- **`Input.dispatchMouseEvent` with `mouseWheel` does not reach the page** in
  this Firefox CDP build. Dispatch a DOM event instead:
  ```js
  document.getElementById('cloud-canvas').dispatchEvent(
    new WheelEvent('wheel', {deltaY: -120, deltaMode: 0, bubbles: true, cancelable: true}));
  ```
- The viewer instance is module-local (`var viewer`), so it cannot be read from
  the console. To inspect camera state, monkey-patch `THREE` prototypes at
  runtime, e.g. record `this.zoom` inside
  `THREE.OrthographicCamera.prototype.updateProjectionMatrix`, or `this.fov`
  inside the `PerspectiveCamera` equivalent.
- The page must be reloaded with `Page.reload {ignoreCache: true}` after JS
  edits; otherwise the old script stays cached in the running tab.
- Click the **VIEW** tab before clicking a `[data-view=...]` button, and allow
  time for boot.
- Backgrounding with `(cmd &)` inside a SYNC call gets killed by tool cleanup;
  use `process_type: BACKGROUND`.
- When stopping test processes, avoid `pkill -f` patterns that also match the
  shell running the command (the shell's own command line contains the
  pattern). Use PID files / `ss -tlnp` PIDs, or bracket tricks such as
  `rosbag [p]lay`.
- Screenshots: `/tmp/top5_default.png` (5 m framing), `/tmp/top5_zoomout.png`
  (full ring set), `/tmp/d_top_zoom2.png` (zoomed with constant-size labels).

## Current state

- All requested features are implemented and verified; test services have been
  stopped and the Firefox test profile removed.
- `README.md` documents the TOP default framing (5 m), the 0.15×–8× zoom range,
  colour-mode semantics, and the max-points control.
- This file is a working-context snapshot, not user-facing documentation.

## Possible next steps (not done)

- Persist zoom level per view instead of resetting on every switch.
- On-screen +/− zoom buttons and double-click-to-reset.
- Drag-to-pan in TOP when zoomed in.
- Show pre-decimation frame size next to the rendered point count in the HUD.

---

# 2026-11-23 — SLAM backend, global map, trajectory, incremental DEM

## Why this change

The previous attempt "hung": the documented bag and the bridge's default topic
did not exist in this checkout, so no frames ever reached the dashboard, and
DEM patches were rejected with HTTP 400. Also, the old pipeline assumed a
pre-built FAST-LIO2, which is a long network build that does not exist on a
fresh Ubuntu 18.04 machine. Goal: stop depending on FAST-LIO2 specifically,
make SLAM pluggable, and make the whole thing start with one command.

Root causes found and fixed:

1. `scripts/play_bag.sh` defaulted to `20261030_221123_mid360s.bag` (absent).
   It now auto-finds a `.bag` in the repo root.
2. `ros_bridge.py`/`slam_web_bridge.py` defaulted to topics that this bag does
   not publish (`/livox/lidar` vs the real `/mid360/livox/lidar`). Both now
   auto-detect, and a missing topic exits with a clear error instead of
   silently showing a frozen dashboard.
3. No DEM process existed at all. `webapp/dem_service.py` is new.
4. Every DEM PATCH failed with 400 because a lost FULL grid left `dem_hub`
   without a grid. The patch endpoint now answers 409 (`need_full`) so the
   service resends a FULL grid; `/internal/map/stream` is the new loopback feed.
5. A restarted SLAM exporter could never append trajectory/tiles again
   (index mismatch against stale backend state). A new exporter session now
   resets the map (and the DEM) on both sides, and the exporter replays its
   voxel map. The first pose also seeds the trajectory (it previously returned
   early, so the path stayed empty forever).

## Architecture

- `slam_ws/src/lidar_slam_bridge/scripts/slam_core.py` — portable ICP odometry
  core (numpy/scipy only): voxel downsample, Umeyama best-fit, local voxel map,
  deskew from per-point timestamps + IMU gyro.
- `.../lidar_odometry.py` — Python 2 rospy node. Auto-detects LiDAR/IMU topics,
  publishes `/Odometry`, `/cloud_registered`, `/path`, optional TF.
- `.../slam_web_bridge.py` — backend-agnostic exporter (auto-detects odometry
  and registered-cloud topics, reports its `backend` label in the handshake).
- `webapp/dem_service.py` — new Python 3 worker: tile batches → PDAL SMRF →
  per-cell minimum → LVD1 FULL/PATCH, exports GeoTIFF/PCD/JSON.
- `webapp/app.py` — `/internal/map/stream` (loopback LVM1 byte stream), 409 on
  PATCH without a grid, map reset on new SLAM session.
- `webapp/protocol.py` — `MapStreamDecoder` for incremental stream framing.
- `webapp/map_ingest.py` — reset-on-hello session semantics.

## Commands

```bash
./scripts/start_all.sh              # backend+tunnel+bridge+SLAM+DEM
./scripts/start_all.sh --bag        # plus bag replay
SLAM_BACKEND=external SLAM_NAME=fast-lio2 ./scripts/start_slam.sh
./scripts/start_dem.sh --resolution 0.5
./scripts/play_bag.sh               # auto-finds the bag
```

## Tests actually run (2026-11-23)

- `tools/odometry_eval.py` on `OutdoorRoad_cut1.bag`: 453 frames, 41.7 fps
  offline, 0 rejected, path 49.9 m, net displacement 28.2 m.
- Repeatability: same scan 30× → 1.3 mm drift.
- Live: roscore + `rosbag play --loop` + `lidar_odometry.py` + exporter +
  Flask. `/Odometry` at 10.07 Hz, live WS 10 fps / 20,064 points from
  `/mid360/livox/lidar` (auto-detected).
- Map: 800k+ voxel points, 160+ tiles, 400+ trajectory points after ~60 s.
- DEM: `has_grid: true`, grid ~170×170 @0.5 m, 122k ground points, elevation
  span −6.03…1.63 m, `error: null`; `current.tif` is a valid little-endian
  GeoTIFF (205 KB), `ground_points.pcd` 2.5 MB, `dem_meta.json` written.
- Ground classification sanity check on synthetic data: PDAL kept 85% of the
  true ground surface and 0% of the +1.5–3 m layer; numpy fallback kept the
  ground layer only.
- `py_compile` on all Python 2 nodes (win) and Python 3 web modules; `bash -n`
  on all scripts.

## Bugs fixed during verification

- `_match_ground` searched a key set that contained the query points, so every
  point was "ground" (max elevation 54 m). Now only ground keys are searched.
- numpy 1.13 `savetxt` refuses a float format when given an open file object →
  binary PLY for PDAL input, temp-file spill for the ASCII PCD.
- `grid.update()` returned a 2-tuple when the grid grew, crashing the DEM status loop.
- `pkill -f dem_service.py` killed the shell that was starting it (same command
  line); use PIDs.

## Limitations / pending

- Bundled odometry has no loop closure → drift on long loops. FAST-LIO2 is the
  recommended production backend and remains drop-in (`SLAM_BACKEND=external`).
- `/gnss_pose` in this bag is static (~1e-7 m per message), so trajectory
  accuracy is NOT validated against ground truth.
- SMRF runs per 20 m tile with a 3×3 halo: tile seams may differ on steep
  terrain. A global re-classification pass is not implemented.
- GeoTIFF is rewritten in full every `--export-interval` (20 s); fine for the
  45 s bag, slow for very large maps.
- The map/DEM state is in memory only; exports are the only persistence.
- Cloudflare Quick Tunnel URL was not exercised in this environment
  (`cloudflared` is not installed); the tunnel code paths are unchanged.

## 2026-11-23 (follow-up) — UnicodeEncodeError in the startup banner

Reported from a real run: `./scripts/start_all.sh` printed the banner and then
died with

```
UnicodeEncodeError: 'ascii' codec can't encode character '\u2026'
  File "webapp/app.py", line 306, in print_banner
```

Cause: the banner used `"starting…"`, and `tunnel.status()` reports
`enabled: true` whenever the tunnel is not disabled — including when
`cloudflared` is not installed (`installed: false`, `state: unavailable`). So
the `starting…` branch was taken on a root shell whose locale is POSIX/C, where
Python 3.6 encodes stdout as ASCII. One non-ASCII glyph killed the Flask
process before it bound port 5000; the bridge, SLAM and DEM processes then
logged "Connection refused" forever, which looked like the pipeline hanging.

Fixes:
- `print_banner` now has three explicit branches: disabled → `disabled`,
  cloudflared missing → `unavailable (cloudflared not installed)`, otherwise the
  URL or `starting...`. All banner text is ASCII.
- New `_ascii_safe_stdout()` re-wraps `sys.stdout`/`sys.stderr` as UTF-8 with
  `errors="replace"` at startup (Python 3.6 has no `reconfigure()`), so any
  later non-ASCII output (paths, cloudflared text) degrades instead of killing
  the backend.
- `scripts/start_all.sh` now waits up to 15 s for `/api/status`, aborts watch if
  the backend process exits, and prints
  `[start_all] ERROR: the web backend never answered on <url>` and exits 1
  instead of starting three components against a dead backend.

Verified: backend started with `LC_ALL=POSIX LANG= PYTHONIOENCODING=ascii`
answers `/api/status` with 200 in both the `--no-tunnel` (DISABLED) and the
cloudflared-missing (CLOUDFLARED NOT INSTALLED) cases; the banner prints
`unavailable (cloudflared not installed)`; `start_all.sh` with a deliberately
invalid port exits 1 with the new error and starts nothing else.

## 2026-11-23 (follow-up) — bag path argument for start_all.sh

`./scripts/start_all.sh <file.bag>` now replays that bag (previously the path
after `--bag` was ignored and always fell back to the first `.bag` in the repo
root). Accepted forms: bare path, `--bag=<path>`, `--bag` (default bag),
`-h/--help`. Relative paths resolve against the repository root, `~/` expands,
and a missing file exits 2 with `start_all: bag not found: <path>` before any
component starts. The resolved path is forwarded to `play_bag.sh`.

Bug found while adding it: every `scripts/*.sh` sources
`/opt/ros/melodic/setup.bash`, and ROS's `setup.sh` runs
`_setup_util.py "$@"` and sources the result. Script arguments were therefore
handed to the ROS setup — `--help` made `_setup_util.py` print its own usage
text, which was then sourced (`/tmp/setup.sh.*: line 1: usage:: command not
found`, exit 127). All four scripts now save, clear and restore `"$@"` around
the `source` call.

Verified: `--help`/`-h` exit 0 and print the usage block; a missing bag exits 2;
`NO_TUNNEL=1 NO_DEM=1 NO_SLAM=1 ./scripts/start_all.sh OutdoorRoad_cut1.bag`
logged `play_bag: /home/LiDARdashboard/OutdoorRoad_cut1.bag` and the dashboard
reported 9.97 fps / 20,064 pts / 129 frames; `bash -n` clean on all scripts.

## 2026-11-24 — second dataset (`/workspace/datasets/demo1.bag`) did not work

Reported symptom: the dashboard stayed empty for a bag whose topics/fields
differ from `OutdoorRoad_cut1.bag`.

Bag facts: 10 s, 101 frames, `sensor_msgs/PointCloud2` only, single topic
`/livox/lidar`, frame `livox_frame`, 167,616 points/frame, `point_step` 32,
fields `x,y,z,rgb` (packed colour float at offset 16), **no `intensity`**, no
per-point `timestamp`, **no IMU**.

Root causes (in the order they bit):

1. **Every live frame was rejected** —
   `bad frame: 'names', 'formats', 'offsets', and 'titles' dict entries must
   have the same length` in `ros_bridge._extract`. Python 2 leaks comprehension
   variables, so `formats = ["<f4" for _ in ONLY_FIELDS if name in fields]`
   used the *last* value of `name` from the previous comprehension: with
   `x,y,z,intensity` present it accidentally matched the names list, which is
   why the Mid-360 bag worked and this one did not.
2. **No intensity field** — colour modes would have shown zeros.
3. **Topic resolution was a single guess** — with several registered topics
   (a stale publisher, or a bag with Avia/Mid-360/Ouster) the node could pick a
   topic that never publishes; `_lidar_topic:=` values also persist in the ROS
   parameter server, so an old override kept pinning a dead topic.
4. **Bundled ICP could not keep up** — 167k points/frame at stride 1 ran at
   1.6 frames/s, so the map looked frozen.
5. **Startup order** — SLAM and the bridge started before the bag advertised
   its topics.

Fixes:

- `_extract` builds one ordered list of names/formats/offsets from the real
  `PointField` datatypes (int/uint/float32/float64, `count > 1` supported).
- `_intensity()` falls back to `intensity`/`intensities`/`reflectivity`/
  `reflectance`/`refl`/`power`/`i`, and to packed `rgb`/`rgba` luminance, with
  0..1 and 0..65535 ranges normalised to 0..255.
- Wide candidate lists plus keyword and message-type fallbacks in
  `ros_bridge.py`, `lidar_odometry.py` and `slam_web_bridge.py`; every plausible
  topic is subscribed and the first one that really delivers is used. An
  explicit `~topic`/`~lidar_topic` is tried first but not exclusive, and a
  better-ranked topic may take over within the first 5 frames (multi-LiDAR
  bags), after which the extra subscriptions are dropped.
- `slam_core.IcpOdometry(process_stride=0)` decimates dense scans to
  `target_points_per_frame` (12,000) before ICP; `~process_stride` accepts an
  explicit value.
- `start_all.sh` starts bag playback before SLAM/bridge.
- `play_bag.sh` derives its topic list from the bag itself (all
  `sensor_msgs/PointCloud2` + `sensor_msgs/Imu` topics), playing the whole bag
  if none are found; `BAG_TOPICS` still overrides.

Verified:

- `tools/odometry_eval.py /workspace/datasets/demo1.bag --topic /livox/lidar`:
  101 frames, 9.46 frames/s, 0 rejected, 188.23 m path / 188.07 m net, 97%
  inliers (was 1.57 frames/s, identical path within 0.5 m).
- `tools/odometry_eval.py OutdoorRoad_cut1.bag`: unchanged — 453 frames,
  36.78 frames/s, 0 rejected, 49.88 m path (was 49.9 m).
- Full stack on demo1 (`NO_TUNNEL=1 ./scripts/start_all.sh
  /workspace/datasets/demo1.bag`): LIVE 10.04 fps / 152,400 pts, map 1.07M
  points over 644 tiles, 169 trajectory points, DEM `has_grid: true`
  1180x168 with 12,159 ground points, `error: null`, all exports written.
- Full stack on the original bag: live 10.0 fps / 19,968 pts
  (`mid360_frame`), odometry locked `/avia/livox/lidar` then switched to the
  preferred `/mid360/livox/lidar`, 99 frames in 10 s, map 351k points.
- `python2 -m py_compile` and `bash -n` clean on every touched file.
