/* Live LiDAR point-cloud viewer (Three.js r128).
 *
 * One point cloud, one geometry, one GPU buffer set.  Only the camera changes
 * between FRONT / BACK / LEFT / RIGHT / FREE, so switching views never touches
 * the stream or the ROS side.
 *
 * Frame layout must match webapp/frame_hub.py:
 *   4s magic "LVP1" | u32 count | f64 stamp | u32 seq | u16 frameIdLen |
 *   2 pad | frameId (padded to 4) | count * (f32 x, y, z, intensity)
 */
(function (global) {
  'use strict';

  var MAGIC = 0x3150564c; // "LVP1" little endian
  var HEADER_SIZE = 24;
  var POINT_BYTES = 16;
  var RING_STEP = 5;        // metres between range rings
  var RING_COUNT = 8;       // 5 m … 40 m
  var BASE_FOV = 72;        // FRONT/BACK/LEFT/RIGHT field of view at zoom 1
  var TOP_HALF = 6;         // TOP ortho half-height: the 5 m ring fits the frame
  var ZOOM_MIN = 0.15;      // 6 / 0.15 = 40 m half-height: the whole ring set
  var ZOOM_MAX = 8;         // 6 / 8 = 0.75 m half-height at the tightest zoom

  var COLOR_STOPS = [
    [0.00, 0.15, 0.29, 0.86],
    [0.25, 0.13, 0.83, 0.93],
    [0.50, 0.13, 0.77, 0.37],
    [0.75, 0.92, 0.70, 0.03],
    [1.00, 0.94, 0.27, 0.27]
  ];

  var VIEWS = {
    front: { dir: [1, 0, 0], label: 'FRONT · +X' },
    back: { dir: [-1, 0, 0], label: 'BACK · −X' },
    left: { dir: [0, 1, 0], label: 'LEFT · +Y' },
    right: { dir: [0, -1, 0], label: 'RIGHT · −Y' }
  };

  function PointCloudViewer(canvas, options) {
    options = options || {};
    this.canvas = canvas;
    this.onStats = options.onStats || function () {};
    this.mode = 'front';
    this.colorMode = 'height';
    this.maxPoints = 120000;
    this.accumFrames = 10;    // frames merged into the displayed cloud (1 = off)
    this._filled = 0;         // slots currently holding a frame
    this.active = true;
    this.drawn = 0;
    this.zMin = -2;
    this.zMax = 6;
    this.dMin = 0;
    this.dMax = 40;
    this._rangeReady = false;
    this._renderFrames = 0;
    this._renderWindowStart = performance.now();
    this._fps = 0;
    this._lastFrameAt = 0;
    this._frameSeq = 0;
    this._frameCount = 0;
    this.zoom = 1;

    this._initScene();
    this._initBuffers();
    this.setAccumulation(this.accumFrames);
    this._bind();
    this.setView('front');
    this._loop();
  }

  PointCloudViewer.prototype._initScene = function () {
    var width = this.canvas.clientWidth || 800;
    var height = this.canvas.clientHeight || 600;

    this.renderer = new THREE.WebGLRenderer({
      canvas: this.canvas,
      antialias: true,
      alpha: false,
      powerPreference: 'high-performance'
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setSize(width, height, false);
    this.renderer.setClearColor(0x04070c, 1);

    this.scene = new THREE.Scene();

    this.camera = new THREE.PerspectiveCamera(BASE_FOV, width / height, 0.05, 900);
    this.camera.up.set(0, 0, 1); // ROS convention: X fwd, Y left, Z up
    this.viewCamera = this.camera;

    // Orthographic overhead camera for the TOP map: true top-down, so the
    // 5 m range rings stay circular and evenly spaced.
    var half = TOP_HALF;
    this.topCamera = new THREE.OrthographicCamera(
      -half * (width / height), half * (width / height), half, -half, 0.1, 400
    );
    this.topCamera.up.set(1, 0, 0); // +X up the screen
    this.topCamera.position.set(0, 0, 120);
    this.topCamera.lookAt(0, 0, 0);
    this.topCamera.updateMatrixWorld();

    // Ground / reference grid on the XY plane (Z up).
    var grid = new THREE.GridHelper(120, 60, 0x274055, 0x14202e);
    grid.rotation.x = Math.PI / 2;
    grid.material.transparent = true;
    grid.material.opacity = 0.65;
    grid.material.depthWrite = false;
    this.scene.add(grid);

    // ROS-aligned axes: X red (forward), Y green (left), Z blue (up).
    this.axes = new THREE.AxesHelper(2.2);
    this.axes.material.depthTest = false;
    this.axes.renderOrder = 5;
    this.scene.add(this.axes);

    // Sensor origin indicator.
    var origin = new THREE.Mesh(
      new THREE.SphereGeometry(0.1, 16, 12),
      new THREE.MeshBasicMaterial({ color: 0xffc451 })
    );
    origin.renderOrder = 6;
    origin.material.depthTest = false;
    this.scene.add(origin);

    var ring = new THREE.Mesh(
      new THREE.RingGeometry(0.34, 0.4, 40),
      new THREE.MeshBasicMaterial({
        color: 0xffc451,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.55,
        depthWrite: false
      })
    );
    ring.rotation.x = -Math.PI / 2;
    this.scene.add(ring);

    this.controls = new THREE.OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.screenSpacePanning = true;
    this.controls.minDistance = 0.3;
    this.controls.maxDistance = 400;
    this.controls.enabled = false;

    this._initRangeRings();
  };

  /** 5 m concentric distance rings on the ground plane, TOP view only. */
  PointCloudViewer.prototype._initRangeRings = function () {
    var group = new THREE.Group();
    group.visible = false;
    this.ringLabels = [];

    for (var i = 1; i <= RING_COUNT; i++) {
      var radius = i * RING_STEP;
      var points = [];
      for (var a = 0; a <= 128; a++) {
        var theta = (a / 128) * Math.PI * 2;
        points.push(new THREE.Vector3(Math.cos(theta) * radius, Math.sin(theta) * radius, 0));
      }
      var line = new THREE.Line(
        new THREE.BufferGeometry().setFromPoints(points),
        new THREE.LineBasicMaterial({
          color: i % 2 ? 0x3d8fb5 : 0x22536b,
          transparent: true,
          opacity: 0.9,
          depthTest: false,
          depthWrite: false
        })
      );
      line.renderOrder = 3;
      group.add(line);
      var label = this._ringLabel(radius);
      group.add(label);
      this.ringLabels.push(label);
    }

    this.ranges = group;
    this.scene.add(group);
  };

  PointCloudViewer.prototype._ringLabel = function (radius) {
    var canvas = document.createElement('canvas');
    canvas.width = 128;
    canvas.height = 64;
    var ctx = canvas.getContext('2d');
    ctx.fillStyle = 'rgba(4, 7, 12, 0.72)';
    ctx.fillRect(4, 12, 120, 40);
    ctx.font = 'bold 32px Arial, sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'middle';
    ctx.fillStyle = '#6fd0ec';
    ctx.fillText(radius + ' m', 64, 33);

    var texture = new THREE.CanvasTexture(canvas);
    texture.minFilter = THREE.LinearFilter;
    var sprite = new THREE.Sprite(new THREE.SpriteMaterial({
      map: texture,
      transparent: true,
      depthTest: false,
      depthWrite: false
    }));
    var angle = Math.PI / 4; // 45° between +X and +Y, clear of the axes
    sprite.position.set(Math.cos(angle) * radius, Math.sin(angle) * radius, 0.05);
    sprite.scale.set(1.7, 0.85, 1);
    sprite.renderOrder = 4;
    return sprite;
  };

  /**
   * Point storage is a RING of per-frame GPU buffers ("slots"), one slot per
   * accumulated frame.  A new frame overwrites only the oldest slot, so each
   * frame colours and uploads just its own ~20k points instead of re-uploading
   * the whole accumulated cloud.  All slots share one material.
   */
  PointCloudViewer.prototype._initBuffers = function () {
    this.material = new THREE.PointsMaterial({
      size: 0.075,
      sizeAttenuation: true,
      vertexColors: true,
      transparent: false,
      depthWrite: true
    });
    this._slots = [];
    this._next = 0;   // slot that receives the next frame
  };

  PointCloudViewer.prototype._makeSlot = function () {
    var geometry = new THREE.BufferGeometry();
    geometry.setDrawRange(0, 0);
    var points = new THREE.Points(geometry, this.material);
    points.frustumCulled = false;
    points.renderOrder = 2;
    this.scene.add(points);
    return { points: points, data: null, count: 0, capacity: 0, pos: null, col: null };
  };

  PointCloudViewer.prototype._disposeSlot = function (slot) {
    this.scene.remove(slot.points);
    slot.points.geometry.dispose();
  };

  /** Copy one frame (Float32Array of x, y, z, intensity) into a slot. */
  PointCloudViewer.prototype._fillSlot = function (slot, data) {
    var n = data.length / 4;
    if (n > slot.capacity) {
      // Grow: new geometry, and dispose the old one so its GL buffers are freed.
      var cap = Math.max(n, Math.ceil(slot.capacity * 1.5), 1024);
      slot.pos = new Float32Array(cap * 3);
      slot.col = new Float32Array(cap * 3);
      slot.capacity = cap;
      var geometry = new THREE.BufferGeometry();
      var pa = new THREE.BufferAttribute(slot.pos, 3);
      var ca = new THREE.BufferAttribute(slot.col, 3);
      pa.setUsage(THREE.DynamicDrawUsage);
      ca.setUsage(THREE.DynamicDrawUsage);
      geometry.setAttribute('position', pa);
      geometry.setAttribute('color', ca);
      var old = slot.points.geometry;
      slot.points.geometry = geometry;
      old.dispose();
    }
    var pos = slot.pos;
    for (var p = 0, s = 0, d = 0; p < n; p++, s += 4, d += 3) {
      pos[d] = data[s];
      pos[d + 1] = data[s + 1];
      pos[d + 2] = data[s + 2];
    }
    slot.data = data;
    slot.count = n;
    var g = slot.points.geometry;
    var posAttr = g.getAttribute('position');
    posAttr.updateRange.offset = 0;
    posAttr.updateRange.count = n * 3;
    posAttr.needsUpdate = true;
    g.setDrawRange(0, n);
    this._colorSlot(slot);
  };

  /** (Re)compute a slot's colours with the current colour mode and range. */
  PointCloudViewer.prototype._colorSlot = function (slot) {
    if (!slot.data) return;
    var src = slot.data;
    var col = slot.col;
    var n = slot.count;
    var span = this.zMax - this.zMin;
    if (!(span > 0.001)) span = 0.001;
    var invSpan = 1 / span;
    var invDist = 1 / (this.dMax > 0.5 ? this.dMax : 1);
    var heightMode = this.colorMode === 'height';
    var distanceMode = this.colorMode === 'distance';
    for (var p = 0, si = 0, di = 0; p < n; p++, si += 4, di += 3) {
      var t;
      if (heightMode) {
        t = (src[si + 2] - this.zMin) * invSpan;
      } else if (distanceMode) {
        var dx = src[si];
        var dy = src[si + 1];
        var dz = src[si + 2];
        t = Math.sqrt(dx * dx + dy * dy + dz * dz) * invDist;
      } else {
        t = src[si + 3] / 255;
      }
      if (t < 0) t = 0; else if (t > 1) t = 1;
      this._ramp(t, col, di);
    }
    var colAttr = slot.points.geometry.getAttribute('color');
    colAttr.updateRange.offset = 0;
    colAttr.updateRange.count = n * 3;
    colAttr.needsUpdate = true;
  };

  PointCloudViewer.prototype._countDrawn = function () {
    var drawn = 0;
    var filled = 0;
    for (var i = 0; i < this._slots.length; i++) {
      drawn += this._slots[i].count;
      if (this._slots[i].data) filled += 1;
    }
    this.drawn = drawn;
    this._filled = filled;
  };

  PointCloudViewer.prototype._bind = function () {
    var self = this;
    this._onResize = function () { self.resize(); };
    window.addEventListener('resize', this._onResize);
    if (window.ResizeObserver) {
      this._observer = new ResizeObserver(this._onResize);
      this._observer.observe(this.canvas.parentElement || this.canvas);
    }
    this._onWheel = function (ev) { self._wheel(ev); };
    this.canvas.addEventListener('wheel', this._onWheel, { passive: false });
  };

  /** Wheel zoom for the fixed views; FREE VIEW keeps OrbitControls' dolly. */
  PointCloudViewer.prototype._wheel = function (ev) {
    if (!this.active || this.mode === 'free') return;
    ev.preventDefault();
    var delta = ev.deltaMode === 1 ? ev.deltaY * 16 : ev.deltaY;
    this._setZoom(this.zoom * Math.pow(1.0018, -delta));
  };

  PointCloudViewer.prototype._setZoom = function (zoom) {
    this.zoom = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, zoom));
    if (this.mode === 'top') {
      this.topCamera.zoom = this.zoom;
      this.topCamera.updateProjectionMatrix();
      // Keep ring labels the same size on screen at every zoom level.
      var labels = this.ringLabels || [];
      for (var i = 0; i < labels.length; i++) {
        labels[i].scale.set(1.7 / this.zoom, 0.85 / this.zoom, 1);
      }
      return;
    }
    var fov = 2 * Math.atan(Math.tan(BASE_FOV * Math.PI / 360) / this.zoom) * 180 / Math.PI;
    this.camera.fov = Math.min(100, Math.max(6, fov));
    this.camera.updateProjectionMatrix();
  };

  PointCloudViewer.prototype.resize = function () {
    var width = this.canvas.clientWidth;
    var height = this.canvas.clientHeight;
    if (!width || !height) return;
    var aspect = width / height;
    this.camera.aspect = aspect;
    this.camera.updateProjectionMatrix();
    var half = TOP_HALF;
    this.topCamera.left = -half * aspect;
    this.topCamera.right = half * aspect;
    this.topCamera.top = half;
    this.topCamera.bottom = -half;
    this.topCamera.updateProjectionMatrix();
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.setSize(width, height, false);
  };

  PointCloudViewer.prototype.setActive = function (active) {
    this.active = active;
    if (active) {
      this.resize();
      this._renderWindowStart = performance.now();
      this._renderFrames = 0;
    }
  };

  /* --------------------------- camera presets --------------------------- */

  PointCloudViewer.prototype.setView = function (mode) {
    this.mode = mode;
    this.ranges.visible = mode === 'top';
    this.camera.up.set(0, 0, 1);
    this._setZoom(1); // each view starts at its default framing

    if (mode === 'top') {
      // Overhead map: +X up the screen, +Y to the left, distance rings on Z=0.
      this.controls.enabled = false;
      this.viewCamera = this.topCamera;
      return;
    }
    this.viewCamera = this.camera;

    if (mode === 'free') {
      // Keep the current orientation but pull back so orbiting works.
      var dir = new THREE.Vector3();
      this.camera.getWorldDirection(dir);
      this.camera.position.copy(dir).multiplyScalar(-24);
      this.controls.target.set(0, 0, 0);
      this.controls.enabled = true;
      this.controls.update();
      return;
    }

    var preset = VIEWS[mode] || VIEWS.front;
    this.controls.enabled = false;
    // Camera stays exactly at the LiDAR origin; only the orientation changes.
    this.camera.position.set(0, 0, 0);
    this.camera.lookAt(preset.dir[0] * 10, preset.dir[1] * 10, preset.dir[2] * 10);
    this.camera.updateMatrixWorld();
  };

  PointCloudViewer.prototype.setColorMode = function (mode) {
    this.colorMode = mode;
    this._rangeReady = false;
    // Recolour the frames already on screen so the whole cloud switches at once.
    for (var i = 0; i < this._slots.length; i++) this._colorSlot(this._slots[i]);
  };

  PointCloudViewer.prototype.setMaxPoints = function (limit) {
    this.maxPoints = limit;   // applies to frames received from now on
  };

  /**
   * Accumulation window.  The Mid-360 scan pattern is non-repetitive, so one
   * 100 ms frame covers only part of the field of view; showing the last N
   * frames together fills it in.  Points are in the SENSOR frame, so this is
   * only correct while the sensor is stationary — on a moving sensor it smears.
   */
  PointCloudViewer.prototype.setAccumulation = function (frames) {
    var n = Math.max(1, frames | 0);
    // Frames currently held, oldest -> newest.
    var held = [];
    var len = this._slots.length;
    for (var k = 0; k < len; k++) {
      var slot = this._slots[(this._next + k) % len];
      if (slot.data) held.push(slot.data);
    }
    held = held.slice(Math.max(0, held.length - n));
    for (var i = 0; i < len; i++) this._disposeSlot(this._slots[i]);
    this._slots = [];
    for (var j = 0; j < n; j++) this._slots.push(this._makeSlot());
    for (var h = 0; h < held.length; h++) this._fillSlot(this._slots[h], held[h]);
    this._next = held.length % n;
    this.accumFrames = n;
    this._countDrawn();
    this._emitStats();
  };

  PointCloudViewer.prototype.clearAccumulation = function () {
    for (var i = 0; i < this._slots.length; i++) {
      var slot = this._slots[i];
      slot.data = null;
      slot.count = 0;
      slot.points.geometry.setDrawRange(0, 0);
    }
    this._next = 0;
    this._countDrawn();
  };

  /* ------------------------------ data path ----------------------------- */

  PointCloudViewer.prototype.applyFrame = function (buffer) {
    if (!buffer || buffer.byteLength < HEADER_SIZE) return false;
    var view = new DataView(buffer);
    if (view.getUint32(0, true) !== MAGIC) return false;

    var total = view.getUint32(4, true);
    var frameIdLen = view.getUint16(20, true);
    var dataOffset = HEADER_SIZE + ((frameIdLen + 3) & ~3);
    if (!total || dataOffset + total * POINT_BYTES > buffer.byteLength) return false;

    var src = new Float32Array(buffer, dataOffset, total * 4);
    // Max points caps the whole accumulated cloud, so each frame gets an
    // equal share of the budget: maxPoints / accumFrames.
    var budget = this.maxPoints > 0 ? Math.max(1, Math.floor(this.maxPoints / this.accumFrames)) : 0;
    var stride = budget > 0 ? Math.ceil(total / budget) : 1;
    var kept = Math.ceil(total / stride);

    if (this.colorMode !== 'intensity') {
      this._updateRange(src, total, stride);
    }

    // Keep a decimated copy of x, y, z, intensity (the WebSocket buffer is
    // not retained) and write it into the oldest slot of the ring.
    var copy = new Float32Array(kept * 4);
    for (var k = 0, s4 = 0; k < kept; k++, s4 += stride * 4) {
      var c4 = k * 4;
      copy[c4] = src[s4];
      copy[c4 + 1] = src[s4 + 1];
      copy[c4 + 2] = src[s4 + 2];
      copy[c4 + 3] = src[s4 + 3];
    }
    this._fillSlot(this._slots[this._next], copy);
    this._next = (this._next + 1) % this._slots.length;
    this._countDrawn();

    this._frameSeq = view.getUint32(16, true);
    this._frameCount += 1;
    this._lastFrameAt = performance.now();
    this._emitStats();
    return true;
  };

  /** Slowly adapt the colour ranges (height and range) without flicker. */
  PointCloudViewer.prototype._updateRange = function (src, total, stride) {
    var lo = Infinity;
    var hi = -Infinity;
    var hist = new Uint32Array(101); // 1 m bins up to 100 m
    var valid = 0;
    for (var s = 0; s < total; s += stride) {
      var x = src[s * 4];
      var y = src[s * 4 + 1];
      var z = src[s * 4 + 2];
      if (!isFinite(z)) continue;
      if (z < lo) lo = z;
      if (z > hi) hi = z;
      var d = Math.sqrt(x * x + y * y + z * z);
      if (isFinite(d)) {
        valid += 1;
        if (d >= 100) hist[100] += 1;
        else hist[d | 0] += 1;
      }
    }
    if (!isFinite(lo) || !isFinite(hi)) return;

    // 99th percentile range: a few stray long returns must not wash out the
    // colour ramp for the rest of the scene.
    var far = 0;
    if (valid) {
      var target = valid * 0.99;
      var acc = 0;
      for (var b = 0; b < hist.length; b++) {
        acc += hist[b];
        if (acc >= target) { far = b + 1; break; }
      }
    }

    if (!this._rangeReady) {
      this.zMin = lo;
      this.zMax = hi;
      this.dMax = Math.max(far, 5);
      this._rangeReady = true;
    } else {
      this.zMin += (lo - this.zMin) * 0.12;
      this.zMax += (hi - this.zMax) * 0.12;
      this.dMax += (Math.max(far, 5) - this.dMax) * 0.12;
    }
    if (this.zMax - this.zMin < 0.4) {
      var mid = (this.zMax + this.zMin) / 2;
      this.zMin = mid - 0.2;
      this.zMax = mid + 0.2;
    }
  };

  PointCloudViewer.prototype._ramp = function (t, out, i) {
    var k = 0;
    while (k < COLOR_STOPS.length - 2 && t > COLOR_STOPS[k + 1][0]) k++;
    var a = COLOR_STOPS[k];
    var b = COLOR_STOPS[k + 1];
    var f = (t - a[0]) / (b[0] - a[0]);
    out[i] = a[1] + (b[1] - a[1]) * f;
    out[i + 1] = a[2] + (b[2] - a[2]) * f;
    out[i + 2] = a[3] + (b[3] - a[3]) * f;
  };

  PointCloudViewer.prototype._emitStats = function () {
    this.onStats({
      fps: this._fps,
      drawn: this.drawn,
      zMin: this.zMin,
      zMax: this.zMax,
      frames: this._frameCount,
      accumulated: this._filled,
      seq: this._frameSeq,
      age: this._lastFrameAt ? (performance.now() - this._lastFrameAt) / 1000 : null
    });
  };

  PointCloudViewer.prototype._loop = function () {
    var self = this;
    function tick() {
      requestAnimationFrame(tick);
      if (!self.active) return;

      if (self.controls.enabled) self.controls.update();
      self.renderer.render(self.scene, self.viewCamera);

      self._renderFrames += 1;
      var now = performance.now();
      var elapsed = now - self._renderWindowStart;
      if (elapsed >= 1000) {
        self._fps = (self._renderFrames * 1000) / elapsed;
        self._renderFrames = 0;
        self._renderWindowStart = now;
        self._emitStats();
      }
    }
    requestAnimationFrame(tick);
  };

  global.PointCloudViewer = PointCloudViewer;
})(window);
