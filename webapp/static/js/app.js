/* Dashboard controller: tabs, status polling, live/map/DEM WebSockets. */
(function () {
  'use strict';

  var VIEW_META = {
    front: { label: 'FRONT · +X', sub: 'camera at LiDAR origin · looking forward (+X) · scroll to zoom' },
    back: { label: 'BACK · −X', sub: 'camera at LiDAR origin · looking backward (−X) · scroll to zoom' },
    left: { label: 'LEFT · +Y', sub: 'camera at LiDAR origin · looking left (+Y) · scroll to zoom' },
    right: { label: 'RIGHT · −Y', sub: 'camera at LiDAR origin · looking right (−Y) · scroll to zoom' },
    top: { label: 'TOP · −Z', sub: 'overhead map · +X up · 5 m rings · scroll to zoom' },
    free: { label: 'FREE VIEW', sub: 'orbit · zoom · pan (camera may move off origin)' }
  };

  var $ = function (id) { return document.getElementById(id); };

  var state = {
    tab: 'dashboard',
    ws: null,
    wsConnected: false,
    wsAttempts: 0,
    reconnectTimer: null,
    mapWs: null,
    mapConnected: false,
    mapAttempts: 0,
    mapReconnectTimer: null,
    demWs: null,
    demConnected: false,
    demAttempts: 0,
    demReconnectTimer: null,
    lastFrameAt: 0,
    receivedFrames: 0,
    frameTimes: [],
    status: null,
    statusTimer: null,
    activeView: 'front',
    mode: 'live'
  };

  var viewer = null;
  var demViewer = null;

  function unavailableViewer(message) {
    return {
      failed: true,
      error: message,
      zMin: 0,
      zMax: 0,
      dMin: 0,
      dMax: 0,
      colorMode: 'height',
      mapZMin: 0,
      mapZMax: 0,
      setActive: function () {},
      resize: function () {},
      setView: function () {},
      setColorMode: function () {},
      setMaxPoints: function () {},
      setMapMaxPoints: function () {},
      setDisplayMode: function () {},
      applyFrame: function () { return false; },
      applyMapMessage: function () { return false; },
      mapStats: function () { return { points: 0, tiles: 0, trajectory: 0 }; },
      resetMapView: function () {}
    };
  }

  function showViewerFailure(message) {
    var overlay = $('canvas-overlay');
    var text = $('overlay-text');
    if (!overlay || !text) return;
    overlay.classList.remove('hidden');
    overlay.classList.add('is-error');
    text.textContent = '3D view unavailable: ' + String(message).replace(/\.+$/, '') +
      '. The LiDAR stream is still running (see Dashboard). ' +
      'Enable WebGL in this browser or open the page in Chrome/Chromium.';
    var hint = $('overlay-hint');
    if (hint) hint.textContent = 'The cloud is being received; only 3D drawing is unavailable.';
  }

  /* ------------------------------- helpers ------------------------------- */

  function fmtInt(value) {
    if (value === null || value === undefined) return '—';
    return Number(value).toLocaleString('en-US');
  }

  function fmtBytes(bytes) {
    if (!bytes) return '0 B';
    if (bytes < 1024) return bytes + ' B';
    if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB';
    return (bytes / (1024 * 1024)).toFixed(2) + ' MB';
  }

  function fmtAge(seconds) {
    if (seconds === null || seconds === undefined) return '—';
    if (seconds < 1) return (seconds * 1000).toFixed(0) + ' ms ago';
    if (seconds < 60) return seconds.toFixed(2) + ' s ago';
    return (seconds / 60).toFixed(1) + ' min ago';
  }

  function fmtMetres(value, digits) {
    if (value === null || value === undefined || !isFinite(value)) return '—';
    return Number(value).toFixed(digits === undefined ? 2 : digits) + ' m';
  }

  function setBadge(el, text, cls) {
    if (!el) return;
    el.textContent = text;
    el.className = 'badge' + (cls ? ' ' + cls : '');
  }

  function setMetric(el, text, cls) {
    if (!el) return;
    el.textContent = text;
    el.className = 'metric' + (cls ? ' ' + cls : '');
  }

  function setPill(el, on, warn) {
    if (!el) return;
    el.classList.toggle('is-on', !!on);
    el.classList.toggle('is-warn', !on && !!warn);
    el.classList.toggle('is-off', !on && !warn);
  }

  function wsUrl(path) {
    var proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return proto + '//' + window.location.host + path;
  }

  /* -------------------------------- tabs -------------------------------- */

  function switchTab(name) {
    state.tab = name;
    var tabs = document.querySelectorAll('.tab');
    for (var i = 0; i < tabs.length; i++) {
      tabs[i].classList.toggle('active', tabs[i].dataset.tab === name);
    }
    var panels = document.querySelectorAll('.tab-panel');
    for (var j = 0; j < panels.length; j++) {
      panels[j].classList.toggle('active', panels[j].id === 'tab-' + name);
    }
    if (viewer) viewer.setActive(name === 'view');
    if (demViewer) demViewer.setActive(name === 'dem');
    if (name === 'view') {
      requestAnimationFrame(function () { viewer.resize(); });
    }
    if (name === 'dem') {
      requestAnimationFrame(function () { demViewer.resize(); });
    }
  }

  /* ------------------------------ WebSocket ----------------------------- */

  function updateWsIndicators() {
    var connected = state.wsConnected;
    setPill($('pill-ws'), connected);
    var dot = $('view-ws-dot');
    if (dot) {
      dot.classList.toggle('is-on', connected);
      dot.classList.toggle('is-off', !connected);
    }
    var text = $('view-ws-text');
    if (text) text.textContent = connected ? 'WS' : 'WS OFF';
  }

  function scheduleReconnect() {
    if (state.reconnectTimer) return;
    state.wsAttempts += 1;
    var delay = Math.min(1000 * Math.pow(1.7, state.wsAttempts - 1), 10000);
    delay += Math.random() * 250;
    state.reconnectTimer = setTimeout(function () {
      state.reconnectTimer = null;
      connectWs();
    }, delay);
  }

  function connectWs() {
    if (state.ws && (state.ws.readyState === WebSocket.OPEN ||
        state.ws.readyState === WebSocket.CONNECTING)) return;

    var ws;
    try {
      ws = new WebSocket(wsUrl('/ws/points'));
    } catch (err) {
      scheduleReconnect();
      return;
    }
    ws.binaryType = 'arraybuffer';
    state.ws = ws;

    ws.onopen = function () {
      state.wsConnected = true;
      state.wsAttempts = 0;
      updateWsIndicators();
    };

    ws.onmessage = function (event) {
      if (!(event.data instanceof ArrayBuffer)) return;
      if (viewer.applyFrame(event.data)) {
        state.lastFrameAt = performance.now();
        state.receivedFrames += 1;
        var now = performance.now();
        state.frameTimes.push(now);
        while (state.frameTimes.length && now - state.frameTimes[0] > 2000) {
          state.frameTimes.shift();
        }
      }
    };

    ws.onclose = function () {
      state.wsConnected = false;
      state.ws = null;
      updateWsIndicators();
      scheduleReconnect();
    };

    ws.onerror = function () {
      try { ws.close(); } catch (err) { /* ignore */ }
    };
  }

  function scheduleReconnectFor(kind) {
    var timer = kind + 'ReconnectTimer';
    var attempts = kind + 'Attempts';
    if (state[timer]) return;
    state[attempts] += 1;
    var delay = Math.min(1000 * Math.pow(1.7, state[attempts] - 1), 10000);
    delay += Math.random() * 250;
    state[timer] = setTimeout(function () {
      state[timer] = null;
      if (kind === 'map') connectMapWs(); else connectDemWs();
    }, delay);
  }

  function connectMapWs() {
    if (state.mapWs && (state.mapWs.readyState === WebSocket.OPEN ||
        state.mapWs.readyState === WebSocket.CONNECTING)) return;
    var ws;
    try {
      ws = new WebSocket(wsUrl('/ws/map'));
    } catch (err) {
      scheduleReconnectFor('map');
      return;
    }
    ws.binaryType = 'arraybuffer';
    state.mapWs = ws;
    ws.onopen = function () {
      state.mapConnected = true;
      state.mapAttempts = 0;
    };
    ws.onmessage = function (event) {
      if (!(event.data instanceof ArrayBuffer)) return;
      viewer.applyMapMessage(event.data);
      updateMapHud();
    };
    ws.onclose = function () {
      state.mapConnected = false;
      state.mapWs = null;
      scheduleReconnectFor('map');
    };
    ws.onerror = function () {
      try { ws.close(); } catch (err) { /* ignore */ }
    };
  }

  function connectDemWs() {
    if (state.demWs && (state.demWs.readyState === WebSocket.OPEN ||
        state.demWs.readyState === WebSocket.CONNECTING)) return;
    var ws;
    try {
      ws = new WebSocket(wsUrl('/ws/dem'));
    } catch (err) {
      scheduleReconnectFor('dem');
      return;
    }
    ws.binaryType = 'arraybuffer';
    state.demWs = ws;
    ws.onopen = function () {
      state.demConnected = true;
      state.demAttempts = 0;
    };
    ws.onmessage = function (event) {
      if (!(event.data instanceof ArrayBuffer)) return;
      demViewer.applyMessage(event.data);
    };
    ws.onclose = function () {
      state.demConnected = false;
      state.demWs = null;
      scheduleReconnectFor('dem');
    };
    ws.onerror = function () {
      try { ws.close(); } catch (err) { /* ignore */ }
    };
  }

  /* ------------------------------- status ------------------------------- */

  function pollStatus() {
    fetch('/api/status', { cache: 'no-store' })
      .then(function (res) { return res.json(); })
      .then(function (data) {
        state.status = data;
        renderStatus(data);
      })
      .catch(function () {
        state.status = null;
        renderStatus(null);
      });
  }

  function renderStatus(data) {
    var lidar = $('pill-lidar');

    if (!data) {
      setBadge($('badge-lidar'), 'NO SERVER', 'bad');
      setMetric($('metric-lidar'), 'Backend unreachable', 'bad');
      setBadge($('badge-ros'), 'UNKNOWN', 'bad');
      setMetric($('metric-ros'), 'Unknown', 'bad');
      setPill(lidar, false);
      return;
    }

    var cloud = data.cloud;
    var ros = data.ros;
    var ws = data.websocket;
    var slam = data.slam || {};
    var map = data.map || {};
    var dem = data.dem || {};
    var active = ros.lidar_active;

    // --- LiDAR ---
    setPill(lidar, active);
    setBadge($('badge-lidar'), active ? 'LIVE' : 'NO DATA', active ? 'ok' : 'bad');
    setMetric($('metric-lidar'), active ? 'Streaming' : 'No frames', active ? 'ok' : 'bad');
    $('kv-topic').textContent = ros.topic || '—';
    $('kv-topic').title = ros.topic || '';
    $('kv-type').textContent = ros.message_type || '—';
    $('kv-frame').textContent = cloud.frame_id || '—';
    $('kv-age').textContent = fmtAge(cloud.last_frame_age_s);

    // --- IMU ---
    var imu = ros.imu || {};
    var imuActive = !!imu.active;
    setBadge($('badge-imu'), imuActive ? 'LIVE' : 'NO DATA', imuActive ? 'ok' : 'bad');
    setMetric($('metric-imu'), imuActive ? 'Streaming' : 'No samples', imuActive ? 'ok' : 'bad');
    $('kv-imu-topic').textContent = imu.topic || '/livox/imu';
    $('kv-imu-rate').textContent = imu.fps ? imu.fps.toFixed(1) + ' Hz' : '—';
    $('kv-imu-age').textContent = fmtAge(imu.last_sample_age_s);
    $('kv-imu-frame').textContent = imu.frame_id || '—';

    // --- ROS bridge ---
    var bridge = ros.bridge_connected;
    setBadge($('badge-ros'), bridge ? 'CONNECTED' : 'OFFLINE', bridge ? 'ok' : 'bad');
    setMetric($('metric-ros'), bridge ? 'Bridge online' : 'Bridge offline', bridge ? 'ok' : 'bad');
    $('kv-master').textContent = ros.master_uri || '—';
    $('kv-ingest').textContent = bridge ? (ros.peer || 'tcp') : 'not connected';
    $('kv-pid').textContent = ros.pid ? String(ros.pid) : '—';

    // --- stream ---
    setBadge($('badge-stream'), active ? 'STREAMING' : 'IDLE', active ? 'ok' : 'warn');
    $('metric-fps').textContent = (cloud.fps || 0).toFixed(1);
    $('kv-points').textContent = fmtInt(cloud.points);
    $('kv-framesize').textContent = fmtBytes(cloud.frame_bytes);
    $('kv-total').textContent = fmtInt(cloud.total_frames);
    $('kv-sent').textContent = fmtBytes(cloud.bytes_sent);

    // --- websocket ---
    var clients = ws.clients || 0;
    setBadge($('badge-ws'), state.wsConnected ? 'CONNECTED' : 'DISCONNECTED',
      state.wsConnected ? 'ok' : 'bad');
    setMetric($('metric-ws'), state.wsConnected ? 'Connected' : 'Reconnecting…',
      state.wsConnected ? 'ok' : 'bad');
    $('kv-clients').textContent = String(clients);
    $('kv-wsproto').textContent = window.location.protocol === 'https:' ? 'wss (TLS)' : 'ws (local)';

    // --- SLAM ---
    var slamLive = slam.status === 'mapping';
    var pose = slam.pose;
    setPill($('pill-slam'), slamLive || slam.connected,
      slam.connected && !slamLive);
    setBadge($('badge-slam'), slamLive ? 'MAPPING' :
      (slam.connected ? 'CONNECTED' : 'OFFLINE'), slamLive ? 'ok' :
      (slam.connected ? 'warn' : 'bad'));
    setMetric($('metric-slam'), slamLive ? 'Odometry live' :
      (slam.connected ? 'Connected' : 'Offline'), slamLive ? 'ok' :
      (slam.connected ? 'warn' : 'bad'));
    $('kv-slam-status').textContent = slam.status || '—';
    $('kv-slam-pose').textContent = pose ?
      pose.x.toFixed(2) + ' / ' + pose.y.toFixed(2) + ' / ' + pose.z.toFixed(2) + ' m' : '—';
    $('kv-slam-traj').textContent = (map.trajectory_length_m || 0).toFixed(1) + ' m';
    $('kv-slam-age').textContent = fmtAge(slam.pose_age_s);

    var slamDot = $('view-slam-dot');
    if (slamDot) {
      slamDot.classList.toggle('is-on', slamLive);
      slamDot.classList.toggle('is-warn', slam.connected && !slamLive);
      slamDot.classList.toggle('is-off', !slam.connected);
    }

    // --- global map ---
    setBadge($('badge-map'), map.points ? 'ACCUMULATING' : 'EMPTY',
      map.points ? 'ok' : 'warn');
    $('metric-map').innerHTML = fmtInt(map.points) + ' <small>pts</small>';
    $('kv-map-tiles').textContent = fmtInt(map.tiles);
    $('kv-map-traj-pts').textContent = fmtInt(map.trajectory_points);
    $('kv-map-dropped').textContent = fmtInt(map.dropped_points);
    $('kv-map-cap').textContent = fmtInt(map.max_points) + ' pts';

    // --- DEM ---
    var demStatus = (dem.status || 'idle').toLowerCase();
    var demLive = demStatus === 'processing' || demStatus === 'idle';
    setBadge($('badge-dem'), dem.has_grid ? demStatus.toUpperCase() : 'NO DATA',
      dem.has_grid ? (demStatus === 'error' ? 'bad' : 'ok') : 'warn');
    setMetric($('metric-dem'), dem.has_grid ? demStatus : 'Idle',
      demStatus === 'error' ? 'bad' : (dem.has_grid ? 'ok' : 'warn'));
    $('kv-dem-res').textContent = dem.resolution ? dem.resolution.toFixed(2) + ' m' : '—';
    $('kv-dem-ground').textContent = fmtInt(dem.ground_points);
    $('kv-dem-tiles').textContent = fmtInt(dem.tiles_completed);
    $('kv-dem-range').textContent =
      (dem.min_elevation !== null && dem.min_elevation !== undefined &&
       dem.max_elevation !== null && dem.max_elevation !== undefined) ?
        dem.min_elevation.toFixed(2) + ' … ' + dem.max_elevation.toFixed(2) + ' m' : '—';
    $('kv-dem-mode').textContent = dem.coordinate_mode || 'LOCALIZED';

    renderDemStatus(dem);
    demViewer.setStats(dem);

    // --- tunnel ---
    var tunnel = data.tunnel || {};
    var running = tunnel.running;
    var connected = tunnel.connected;
    var stateText = (tunnel.state || (running ? 'starting' : 'stopped')).toUpperCase();
    setBadge($('badge-tunnel'), tunnel.installed === false ? 'NOT INSTALLED' :
      (connected ? 'CONNECTED' : (running ? 'CONNECTING' : 'OFFLINE')),
      connected ? 'ok' : (running ? 'warn' : 'bad'));
    setMetric($('metric-tunnel'), connected ? 'Remote access live' :
      (running ? 'Connecting…' : 'Local only'), connected ? 'ok' : 'warn');
    $('kv-tunnel-state').textContent = stateText;
    var urlCell = $('kv-tunnel-url');
    urlCell.textContent = '';
    if (tunnel.url) {
      var link = document.createElement('a');
      link.href = tunnel.url;
      link.target = '_blank';
      link.rel = 'noopener';
      link.textContent = tunnel.url;
      urlCell.appendChild(link);
    } else {
      urlCell.textContent = tunnel.installed === false ? 'cloudflared missing' :
        (running ? 'starting…' : 'disabled');
    }
    $('kv-local').textContent = window.location.origin;
    $('kv-local').href = window.location.origin;
    var hint = $('tunnel-hint');
    if (hint) {
      hint.textContent = tunnel.installed === false ?
        'cloudflared is not installed. Local access still works; install cloudflared to enable remote access.' :
        'Tunnel is independent of ROS, SLAM and the LiDAR stream. Only port 5000 is exposed.';
    }

    // --- rviz ---
    var rvizRunning = data.rviz && data.rviz.running;
    setBadge($('badge-rviz'), rvizRunning ? 'RUNNING' : 'NOT DETECTED', rvizRunning ? 'ok' : 'warn');
    setMetric($('metric-rviz'), rvizRunning ? 'RViz running' : 'Not detected', rvizRunning ? 'ok' : 'warn');
    setBadge($('badge-rviz-tab'), rvizRunning ? 'RUNNING' : 'NOT DETECTED', rvizRunning ? 'ok' : 'warn');
    setMetric($('metric-rviz-tab'), rvizRunning ? 'RViz running' : 'Not detected', rvizRunning ? 'ok' : 'warn');
  }

  function renderDemStatus(dem) {
    var status = (dem.status || 'idle').toUpperCase();
    var hasGrid = !!dem.has_grid;
    $('dem-status').textContent = status;
    $('dem-resolution').textContent = dem.resolution ? dem.resolution.toFixed(2) + ' m' : '—';
    $('dem-min').textContent = fmtMetres(dem.min_elevation);
    $('dem-max').textContent = fmtMetres(dem.max_elevation);
    $('dem-ground').textContent = fmtInt(dem.ground_points);
    $('dem-tiles').textContent = fmtInt(dem.tiles_completed);
    $('dem-mode').textContent = dem.coordinate_mode || 'LOCALIZED';
    $('dem-hud-ground').textContent = fmtInt(dem.ground_points);
    $('dem-hud-tiles').textContent = fmtInt(dem.tiles_completed);
    $('dem-hud-res').textContent = dem.resolution ? dem.resolution.toFixed(2) + ' m' : '—';
    $('dem-hud-status').textContent = status;
    var dot = $('dem-dot');
    if (dot) {
      dot.classList.toggle('is-on', status === 'PROCESSING' || status === 'IDLE');
      dot.classList.toggle('is-warn', status === 'WAITING' || status === 'STARTING');
      dot.classList.toggle('is-off', status === 'ERROR' || status === 'OFFLINE');
    }
    var overlay = $('dem-overlay');
    if (!overlay) return;
    if (hasGrid) {
      overlay.classList.add('hidden');
      return;
    }
    overlay.classList.remove('hidden');
    var text = $('dem-overlay-text');
    var hint = $('dem-overlay-hint');
    if (status === 'ERROR') {
      text.textContent = 'DEM processing error';
      hint.textContent = dem.error || 'Check the DEM service log.';
    } else if (state.demConnected) {
      text.textContent = 'Waiting for ground-classified tiles…';
      hint.textContent = 'FAST-LIO2 must be mapping; PDAL processes batches asynchronously.';
    } else {
      text.textContent = 'Waiting for the DEM service…';
      hint.textContent = 'Start scripts/start_dem.sh after FAST-LIO2 is mapping.';
    }
  }

  /* -------------------------------- view UI ----------------------------- */

  function setMode(mode) {
    state.mode = mode === 'global' ? 'global' : 'live';
    viewer.setDisplayMode(state.mode);
    var buttons = document.querySelectorAll('.mode-btn');
    for (var i = 0; i < buttons.length; i++) {
      buttons[i].classList.toggle('active', buttons[i].dataset.mode === state.mode);
    }
    var global = state.mode === 'global';
    $('live-max-wrap').classList.toggle('hidden', global);
    $('map-max-wrap').classList.toggle('hidden', !global);
    $('map-tools').classList.toggle('hidden', !global);
    setView(state.activeView);
    updateLegend();
  }

  function setView(mode) {
    state.activeView = mode;
    viewer.setView(mode);
    var meta = VIEW_META[mode];
    $('view-readout').textContent = meta.label;
    var sub = meta.sub;
    if (state.mode === 'global') {
      sub = mode === 'top' ?
        'overhead of accumulated map · centred on SLAM pose · scroll to zoom' :
        (mode === 'free' ? 'orbit the complete global map · wheel zoom · right-drag pan' :
          'camera follows SLAM pose and LiDAR orientation');
    }
    $('view-sub').textContent = sub;
    var buttons = document.querySelectorAll('.view-btn');
    for (var i = 0; i < buttons.length; i++) {
      buttons[i].classList.toggle('active', buttons[i].dataset.view === mode);
    }
    $('free-hint').classList.toggle('visible', mode === 'free');
  }

  function bindViewControls() {
    var buttons = document.querySelectorAll('.view-btn');
    for (var i = 0; i < buttons.length; i++) {
      buttons[i].addEventListener('click', function () {
        setView(this.dataset.view);
      });
    }

    var modes = document.querySelectorAll('.mode-btn');
    for (var j = 0; j < modes.length; j++) {
      modes[j].addEventListener('click', function () {
        setMode(this.dataset.mode);
      });
    }

    $('color-mode').addEventListener('change', function () {
      viewer.setColorMode(this.value);
      updateLegend();
    });

    $('max-points').addEventListener('change', function () {
      viewer.setMaxPoints(parseInt(this.value, 10) || 0);
    });

    $('map-max-points').addEventListener('change', function () {
      viewer.setMapMaxPoints(parseInt(this.value, 10) || 0);
    });

    $('map-reset').addEventListener('click', function () {
      var ok = window.confirm(
        'Reset the accumulated global map and DEM?\n\n' +
        'This clears the web map, trajectory and DEM tiles. FAST-LIO2 keeps ' +
        'running and will re-add the currently visible area. This cannot be undone.');
      if (!ok) return;
      fetch('/api/map/reset', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ confirm: 'reset' })
      }).then(function (res) { return res.json(); }).then(function () {
        viewer.resetMapView();
        demViewer.reset();
      }).catch(function () {});
    });

    setInterval(function () {
      var now = performance.now();
      var times = state.frameTimes;
      var fps = 0;
      if (times.length > 1) {
        var span = (times[times.length - 1] - times[0]) / 1000;
        if (span > 0) fps = (times.length - 1) / span;
      }
      $('view-fps').textContent = fps.toFixed(1);

      var age = state.lastFrameAt ? (now - state.lastFrameAt) / 1000 : null;
      var lidarDot = $('view-lidar-dot');
      var live = age !== null && age < 2.5;
      lidarDot.classList.toggle('is-on', live);
      lidarDot.classList.toggle('is-off', !live);

      updateMapHud();
      updateOverlay(live);
      updateLegend();
    }, 500);
  }

  function updateMapHud() {
    if (!viewer) return;
    var stats = viewer.mapStats();
    var el = $('map-hud-points');
    if (el) el.textContent = fmtInt(stats.points);
  }

  function updateOverlay(live) {
    var overlay = $('canvas-overlay');
    if (!overlay) return;
    if (viewer.failed) {
      overlay.classList.remove('hidden');
      return;
    }
    if (state.mode === 'global') {
      var stats = viewer.mapStats();
      var slam = (state.status && state.status.slam) || {};
      if (stats.points > 0 || slam.connected) {
        overlay.classList.add('hidden');
      } else {
        overlay.classList.remove('hidden');
        overlay.classList.remove('is-error');
        $('overlay-text').textContent = 'Waiting for the FAST-LIO2 global map…';
        $('overlay-hint').textContent =
          'Start SLAM and scripts/start_slam_web.sh. LIVE VIEW keeps working independently.';
      }
      return;
    }
    if (live) {
      overlay.classList.add('hidden');
    } else {
      overlay.classList.remove('hidden');
      $('overlay-text').textContent = state.wsConnected
        ? 'Waiting for live LiDAR frames…'
        : 'Connecting to the point-cloud stream…';
      $('overlay-hint').textContent = 'Check the ROS bridge and /livox/lidar.';
    }
  }

  function updateLegend() {
    if (!viewer) return;
    var mode = viewer.colorMode || 'height';
    var legend = $('color-legend');
    if (!legend) return;
    legend.classList.toggle('hidden', mode === 'intensity');
    var title = $('legend-title');
    if (title) title.textContent = mode === 'distance' ? 'DISTANCE' : 'HEIGHT';

    if (state.mode === 'global') {
      var zMin = viewer.mapZMin;
      var zMax = viewer.mapZMax;
      if (!isFinite(zMin) || !isFinite(zMax)) {
        $('legend-min').textContent = '—';
        $('legend-max').textContent = '—';
      } else if (mode === 'distance') {
        $('legend-min').textContent = '0 m';
        $('legend-max').textContent = 'map extent';
      } else {
        $('legend-min').textContent = zMin.toFixed(1) + ' m';
        $('legend-max').textContent = '+' + zMax.toFixed(1) + ' m';
      }
      return;
    }

    if (viewer.colorMode === 'distance') {
      $('legend-min').textContent = viewer.dMin.toFixed(1) + ' m';
      $('legend-max').textContent = viewer.dMax.toFixed(1) + ' m';
    } else {
      $('legend-min').textContent = viewer.zMin.toFixed(1) + ' m';
      $('legend-max').textContent = '+' + viewer.zMax.toFixed(1) + ' m';
    }
  }

  function onViewerStats(stats) {
    $('view-points').textContent = fmtInt(stats.drawn);
  }

  function onDemStats(stats) {
    if (!stats) return;
    $('dem-legend-min').textContent = fmtMetres(stats.min);
    $('dem-legend-max').textContent = fmtMetres(stats.max);
    if (stats.resolution) $('dem-resolution').textContent = stats.resolution.toFixed(2) + ' m';
  }

  /* -------------------------------- boot -------------------------------- */

  document.addEventListener('DOMContentLoaded', function () {
    var tabs = document.querySelectorAll('.tab');
    for (var i = 0; i < tabs.length; i++) {
      tabs[i].addEventListener('click', function () {
        switchTab(this.dataset.tab);
      });
    }

    try {
      viewer = new PointCloudViewer($('cloud-canvas'), { onStats: onViewerStats });
      viewer.setActive(false);
    } catch (err) {
      var reason = (err && err.message) ? err.message : String(err);
      console.error('[lidar] 3D viewer failed to start:', err);
      viewer = unavailableViewer(reason);
      showViewerFailure(reason);
    }

    try {
      demViewer = new DemViewer($('dem-canvas'), { onStats: onDemStats });
      demViewer.setActive(false);
    } catch (err2) {
      console.error('[lidar] DEM viewer failed to start:', err2);
      demViewer = {
        setActive: function () {}, resize: function () {}, setStats: function () {},
        applyMessage: function () { return false; }, reset: function () {},
        getRange: function () { return null; }
      };
      var demText = $('dem-overlay-text');
      if (demText) demText.textContent = 'DEM 3D view unavailable (WebGL): ' + String(err2);
    }

    bindViewControls();
    updateLegend();
    setMode('live');
    setView('front');
    connectWs();
    connectMapWs();
    connectDemWs();
    pollStatus();
    state.statusTimer = setInterval(pollStatus, 1000);

    document.addEventListener('visibilitychange', function () {
      if (!document.hidden) {
        if (!state.ws || state.ws.readyState > 1) connectWs();
        if (!state.mapWs || state.mapWs.readyState > 1) connectMapWs();
        if (!state.demWs || state.demWs.readyState > 1) connectDemWs();
      }
    });
  });
})();
