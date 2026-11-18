/* Dashboard controller: tabs, status polling, WebSocket stream. */
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
    lastFrameAt: 0,
    receivedFrames: 0,
    frameTimes: [],
    status: null,
    statusTimer: null,
    activeView: 'front'
  };

  var viewer = null;

  /* Fallback used when the browser cannot create a WebGL context.  It keeps
   * the rest of the page (status polling, WebSocket, tab switching) alive so
   * the failure is visible instead of leaving the UI stuck on "Waiting". */
  function unavailableViewer(message) {
    return {
      failed: true,
      error: message,
      zMin: 0,
      zMax: 0,
      dMin: 0,
      dMax: 0,
      colorMode: 'height',
      setActive: function () {},
      resize: function () {},
      setView: function () {},
      setColorMode: function () {},
      setMaxPoints: function () {},
      applyFrame: function () { return false; }
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
    if (name === 'view') {
      requestAnimationFrame(function () { viewer.resize(); });
    }
  }

  /* ------------------------------ WebSocket ----------------------------- */

  function wsUrl() {
    var proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    return proto + '//' + window.location.host + '/ws/points';
  }

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
      ws = new WebSocket(wsUrl());
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

    // --- tunnel ---
    var tunnel = data.tunnel || {};
    setBadge($('badge-tunnel'), tunnel.running ? 'RUNNING' : 'STOPPED',
      tunnel.running ? 'ok' : 'warn');
    setMetric($('metric-tunnel'), tunnel.running ? 'Tunnel active' : 'Local only',
      tunnel.running ? 'ok' : 'warn');
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
      urlCell.textContent = tunnel.running ? 'starting…' : 'not configured';
    }
    $('kv-local').textContent = window.location.origin;
    $('kv-local').href = window.location.origin;

    // --- rviz ---
    var rvizRunning = data.rviz && data.rviz.running;
    setBadge($('badge-rviz'), rvizRunning ? 'RUNNING' : 'NOT DETECTED', rvizRunning ? 'ok' : 'warn');
    setMetric($('metric-rviz'), rvizRunning ? 'RViz running' : 'Not detected', rvizRunning ? 'ok' : 'warn');
    setBadge($('badge-rviz-tab'), rvizRunning ? 'RUNNING' : 'NOT DETECTED', rvizRunning ? 'ok' : 'warn');
    setMetric($('metric-rviz-tab'), rvizRunning ? 'RViz running' : 'Not detected', rvizRunning ? 'ok' : 'warn');
  }

  /* -------------------------------- view UI ----------------------------- */

  function setView(mode) {
    state.activeView = mode;
    viewer.setView(mode);
    var meta = VIEW_META[mode];
    $('view-readout').textContent = meta.label;
    $('view-sub').textContent = meta.sub;
    var buttons = document.querySelectorAll('.view-btn');
    for (var i = 0; i < buttons.length; i++) {
      buttons[i].classList.toggle('active', buttons[i].dataset.view === mode);
    }
    $('free-hint').classList.toggle('visible', mode === 'free');
  }

  var statsTick = null;

  function bindViewControls() {
    var buttons = document.querySelectorAll('.view-btn');
    for (var i = 0; i < buttons.length; i++) {
      buttons[i].addEventListener('click', function () {
        setView(this.dataset.view);
      });
    }

    $('color-mode').addEventListener('change', function () {
      viewer.setColorMode(this.value);
      updateLegend();
    });

    $('max-points').addEventListener('change', function () {
      viewer.setMaxPoints(parseInt(this.value, 10) || 0);
    });

    statsTick = setInterval(function () {
      // Stream FPS measured from actual delivered frames (2 s window).
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

      if (viewer.colorMode === 'distance') {
        $('legend-min').textContent = viewer.dMin.toFixed(1) + ' m';
        $('legend-max').textContent = viewer.dMax.toFixed(1) + ' m';
      } else {
        $('legend-min').textContent = viewer.zMin.toFixed(1) + ' m';
        $('legend-max').textContent = '+' + viewer.zMax.toFixed(1) + ' m';
      }

      var overlay = $('canvas-overlay');
      if (viewer.failed) {
        overlay.classList.remove('hidden');
      } else if (live) {
        overlay.classList.add('hidden');
      } else {
        overlay.classList.remove('hidden');
        $('overlay-text').textContent = state.wsConnected
          ? 'Waiting for live LiDAR frames…'
          : 'Connecting to the point-cloud stream…';
      }
    }, 500);
  }

  function updateLegend() {
    var mode = (viewer && viewer.colorMode) || 'height';
    var legend = $('color-legend');
    if (!legend) return;
    legend.classList.toggle('hidden', mode === 'intensity');
    var title = $('legend-title');
    if (title) title.textContent = mode === 'distance' ? 'DISTANCE' : 'HEIGHT';
  }

  function onViewerStats(stats) {
    $('view-points').textContent = fmtInt(stats.drawn);
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

    bindViewControls();
    updateLegend();
    setView('front');
    connectWs();
    pollStatus();
    state.statusTimer = setInterval(pollStatus, 1000);

    document.addEventListener('visibilitychange', function () {
      if (!document.hidden && (!state.ws || state.ws.readyState > 1)) {
        connectWs();
      }
    });
  });
})();
