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
