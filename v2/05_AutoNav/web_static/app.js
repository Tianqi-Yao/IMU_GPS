// AutoNav Dashboard
// Single WS connection to autonav_bridge :8806
// IMU and RTK raw data are proxied through autonav_status messages

const AUTONAV_PORT = Number(window.location.port || 8805) + 1;  // 8806
const AUTONAV_URL  = `ws://${window.location.hostname}:${AUTONAV_PORT}`;

const JSON_THROTTLE_MS = 300;

// ── DOM refs ─────────────────────────────────────────────────
const statusDot    = document.getElementById('status-dot');
const statusText   = document.getElementById('status-text');
const stateBadge   = document.getElementById('state-badge');
const stateReason  = document.getElementById('state-reason');
const jsonImu      = document.getElementById('json-imu');
const jsonRtk      = document.getElementById('json-rtk');
const jsonCmd      = document.getElementById('json-cmd');
const jsonStatus   = document.getElementById('json-status');
const imuAge       = document.getElementById('imu-age');
const rtkAge       = document.getElementById('rtk-age');
const needleHdg    = document.getElementById('needle-heading');
const needleTgt    = document.getElementById('needle-target');

const mHeading  = document.getElementById('m-heading');
const mTarget   = document.getElementById('m-target');
const mError    = document.getElementById('m-error');
const mDist     = document.getElementById('m-dist');
const mProgress = document.getElementById('m-progress');
const mLinear   = document.getElementById('m-linear');
const mAngular  = document.getElementById('m-angular');
const mGpsAge   = document.getElementById('m-gps-age');
const mImuAge   = document.getElementById('m-imu-age');

// ── State ────────────────────────────────────────────────────
let navWs = null;
const lastUpdate = { imu: 0, rtk: 0, status: 0 };
let currentPauseMode = 'all';

// ── Helpers ──────────────────────────────────────────────────
function fmt(v, d = 1, unit = '') {
  return v != null ? v.toFixed(d) + unit : '—';
}

function setNeedle(el, angleDeg) {
  if (angleDeg == null) return;
  const rad = (angleDeg - 90) * Math.PI / 180;
  el.setAttribute('x2', (100 + 75 * Math.cos(rad)).toFixed(1));
  el.setAttribute('y2', (100 + 75 * Math.sin(rad)).toFixed(1));
}

function setAge(el, ageS, threshold) {
  if (ageS == null || ageS > 999) {
    el.textContent = 'NO DATA';
    el.style.color = 'var(--red)';
  } else {
    el.textContent = ageS.toFixed(2) + 's';
    el.style.color = ageS > threshold ? 'var(--red)' : '';
  }
}

// ── AutoNav WebSocket ─────────────────────────────────────────
function connectAutoNav() {
  navWs = new WebSocket(AUTONAV_URL);

  navWs.onopen = () => {
    statusDot.className = 'connected';
    statusText.textContent = 'CONNECTED';
  };

  navWs.onclose = () => {
    statusDot.className = '';
    statusText.textContent = 'DISCONNECTED';
    navWs = null;
    setTimeout(connectAutoNav, 3000);
  };

  navWs.onmessage = (e) => {
    try {
      const msg = JSON.parse(e.data);
      if (msg.type !== 'autonav_status') return;
      handleNavStatus(msg);
    } catch (_) {}
  };
}

function handleNavStatus(msg) {
  const now = Date.now();

  // State badge
  const state = msg.state || 'idle';
  stateBadge.className = `state-${state}`;
  if (state === 'lifting' && msg.lift_remaining_s > 0) {
    stateBadge.textContent = `LIFTING ${(msg.lift_cmd || '').toUpperCase()} ${msg.lift_remaining_s}s`;
  } else {
    stateBadge.textContent = state.toUpperCase();
  }

  // Auto-pause reason (sensor timeout or robot link loss) — was previously
  // sent by the server but never shown, making auto-pauses look like an
  // unexplained random stop.
  if (state === 'paused' && msg.sensor_block_reason) {
    stateReason.textContent = `paused: ${msg.sensor_block_reason}`;
    stateReason.style.display = '';
  } else {
    stateReason.style.display = 'none';
  }

  // Compass needles
  setNeedle(needleHdg, msg.heading_deg);
  setNeedle(needleTgt, msg.target_bearing_deg);

  // Metrics
  mHeading.textContent  = fmt(msg.heading_deg, 1, '°');
  mTarget.textContent   = fmt(msg.target_bearing_deg, 1, '°');
  const err = msg.bearing_error_deg;
  mError.textContent    = err != null ? (err >= 0 ? '+' : '') + err.toFixed(1) + '°' : '—';
  mDist.textContent     = fmt(msg.dist_to_wp_m, 1, ' m');
  mProgress.textContent = (msg.current_wp_idx != null && msg.total_wp != null)
    ? `${msg.current_wp_idx} / ${msg.total_wp}` : '—';
  mLinear.textContent   = fmt(msg.linear, 2, ' m/s');
  mAngular.textContent  = fmt(msg.angular, 2, ' r/s');
  setAge(mGpsAge, msg.gps_age_s, 2.0);
  setAge(mImuAge, msg.imu_age_s, 2.0);

  // Sync config values from server
  if (msg.manual_speed != null) MANUAL_SPEED = msg.manual_speed;
  if (msg.lift_up_s   != null) document.getElementById('lift-up-s').value   = msg.lift_up_s;
  if (msg.lift_down_s != null) document.getElementById('lift-down-s').value = msg.lift_down_s;
  if (msg.offset_m != null && !offsetSlider.matches(':active')) {
    offsetSlider.value = msg.offset_m;
    offsetValEl.textContent = parseFloat(msg.offset_m).toFixed(1) + ' m';
  }

  // Waypoint arrival banner
  const banner = document.getElementById('wp-arrive-banner');
  if (msg.waiting_at_wp) {
    document.getElementById('wp-arrive-msg').textContent =
      `Arrived at WP #${msg.waiting_wp_idx} — click CONTINUE to proceed`;
    banner.style.display = 'flex';
  } else {
    banner.style.display = 'none';
  }

  // Pause mode button
  if (msg.pause_mode !== undefined && msg.pause_mode !== currentPauseMode) {
    currentPauseMode = msg.pause_mode;
    document.getElementById('btn-pause-mode').textContent =
      msg.pause_mode === 'all' ? 'Stop: All WPs' : 'Stop: Pause WPs';
  }

  // Waypoint window
  updateWpTable(msg.waypoints_window);

  // Heading calibration panel
  updateCalibPanel(msg);

  // Input JSON panels (throttled) — data proxied from 01/02 via bridge
  if (now - lastUpdate.imu >= JSON_THROTTLE_MS && msg.imu_raw) {
    lastUpdate.imu = now;
    jsonImu.textContent = JSON.stringify(msg.imu_raw, null, 2);
  }
  if (now - lastUpdate.rtk >= JSON_THROTTLE_MS && msg.rtk_raw) {
    lastUpdate.rtk = now;
    jsonRtk.textContent = JSON.stringify(msg.rtk_raw, null, 2);
  }

  // Path map SVG
  updatePathMap(msg);

  // Output JSON panels (throttled)
  if (now - lastUpdate.status >= JSON_THROTTLE_MS) {
    lastUpdate.status = now;
    const _HIDE = new Set(['imu_raw', 'rtk_raw', 'path_original', 'path_offset']);
    const statusDisplay = Object.fromEntries(
      Object.entries(msg).filter(([k]) => !_HIDE.has(k))
    );
    jsonStatus.textContent = JSON.stringify(statusDisplay, null, 2);
    jsonCmd.textContent = JSON.stringify({
      type: 'joystick',
      linear: msg.linear,
      angular: msg.angular,
    }, null, 2);
  }
}

// ── Manual drive ─────────────────────────────────────────
let MANUAL_SPEED = 0.4;  // updated from server via autonav_status.manual_speed

function startManual(linear) { sendCmd('manual_drive', { linear }); }
function stopManual()         { sendCmd('manual_drive', { linear: 0.0 }); }

const btnFwd = document.getElementById('btn-fwd');
const btnBwd = document.getElementById('btn-bwd');

btnFwd.addEventListener('pointerdown',  () => startManual( MANUAL_SPEED));
btnBwd.addEventListener('pointerdown',  () => startManual(-MANUAL_SPEED));
btnFwd.addEventListener('pointerup',    stopManual);
btnBwd.addEventListener('pointerup',    stopManual);
btnFwd.addEventListener('pointerleave', stopManual);
btnBwd.addEventListener('pointerleave', stopManual);

document.addEventListener('keydown', (e) => {
  if (e.repeat) return;
  if (e.key === 'w' || e.key === 'W') startManual( MANUAL_SPEED);
  if (e.key === 's' || e.key === 'S') startManual(-MANUAL_SPEED);
});
document.addEventListener('keyup', (e) => {
  if (e.key === 'w' || e.key === 'W' || e.key === 's' || e.key === 'S') stopManual();
});
window.addEventListener('blur', stopManual);
document.addEventListener('visibilitychange', () => { if (document.hidden) stopManual(); });

// ── Heading calibration ───────────────────────────────────
const calibMarked  = document.getElementById('calib-marked');
const calibCurrent = document.getElementById('calib-current');
const calibBearing = document.getElementById('calib-bearing');
const calibOffset  = document.getElementById('calib-offset');

function fmtLatLon(lat, lon) {
  if (lat == null || lon == null) return '—';
  return `${lat.toFixed(6)}, ${lon.toFixed(6)}`;
}

function bearing(lat1, lon1, lat2, lon2) {
  const toRad = d => d * Math.PI / 180;
  const dLon = toRad(lon2 - lon1);
  const x = Math.sin(dLon) * Math.cos(toRad(lat2));
  const y = Math.cos(toRad(lat1)) * Math.sin(toRad(lat2))
          - Math.sin(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.cos(dLon);
  return ((Math.atan2(x, y) * 180 / Math.PI) + 360) % 360;
}

let _calibMark = null;
let _curLat = null, _curLon = null;

function updateCalibPanel(msg) {
  const calib = msg.calib || {};
  const rtk   = msg.rtk_raw || {};

  _curLat = rtk.lat ?? null;
  _curLon = rtk.lon ?? null;
  _calibMark = calib.mark || null;

  calibCurrent.textContent = fmtLatLon(_curLat, _curLon);

  if (_calibMark) {
    calibMarked.textContent = fmtLatLon(_calibMark.lat, _calibMark.lon);
    if (_curLat != null) {
      const b = bearing(_curLat, _curLon, _calibMark.lat, _calibMark.lon);
      calibBearing.textContent = b.toFixed(1) + '°';
    }
  } else {
    calibMarked.textContent = '—';
    calibBearing.textContent = '—';
  }

  if (calib.offset_applied != null) {
    calibOffset.textContent = calib.offset_applied.toFixed(2) + '°';
    calibOffset.className = 'applied';
  } else {
    calibOffset.textContent = '—';
    calibOffset.className = '';
  }
}

document.getElementById('btn-mark').addEventListener('click', () => sendCmd('calib_mark'));
document.getElementById('btn-calibrate').addEventListener('click', () => {
  if (!_calibMark) { alert('先按 MARK POS 记录前方位置'); return; }
  sendCmd('calib_apply');
});

// ── Waypoint table ────────────────────────────────────────
const wpTbody = document.getElementById('wp-tbody');

function updateWpTable(wps) {
  if (!wps || !wps.length) {
    wpTbody.innerHTML = '<tr><td colspan="3" style="color:var(--dim);text-align:center">no waypoints</td></tr>';
    return;
  }
  wpTbody.innerHTML = wps.map(wp => {
    const cls = wp.current ? ' class="wp-current"' : '';
    return `<tr${cls}><td>${wp.idx}</td><td>${wp.lat.toFixed(7)}</td><td>${wp.lon.toFixed(7)}</td></tr>`;
  }).join('');
}

// ── Control buttons ───────────────────────────────────────────
function sendCmd(type, extra = {}) {
  if (navWs && navWs.readyState === WebSocket.OPEN) {
    navWs.send(JSON.stringify({ type, ...extra }));
  }
}

document.getElementById('btn-start').addEventListener('click',   () => sendCmd('start'));
document.getElementById('btn-stop').addEventListener('click',    () => sendCmd('stop'));
document.getElementById('btn-restart').addEventListener('click', () => sendCmd('restart'));

// Keyboard shortcuts for the main controls — the dashboard display is
// mounted on the robot, so precise clicking/remote mousing is often not an
// option. Ignored while typing into a text field, and while any modifier
// key is held so browser shortcuts still work.
const NAV_KEY_COMMANDS = { '1': 'start', '2': 'stop', '3': 'restart' };
document.addEventListener('keydown', (e) => {
  if (e.repeat || e.ctrlKey || e.altKey || e.metaKey) return;
  const tag = document.activeElement && document.activeElement.tagName;
  if (tag === 'INPUT' || tag === 'TEXTAREA' || tag === 'SELECT') return;
  const cmd = NAV_KEY_COMMANDS[e.key];
  if (cmd) sendCmd(cmd);
});

// ── Speed slider ──────────────────────────────────────────────
const speedSlider = document.getElementById('speed-slider');
speedSlider.addEventListener('input', (e) => {
  const pct = parseInt(e.target.value);
  document.getElementById('speed-value').textContent = pct + '%';
  sendCmd('set_speed', { ratio: pct / 100 });
});

// ── Waypoint CONTINUE button ─────────────────────────────────
document.getElementById('btn-continue').addEventListener('click', () => {
  sendCmd('confirm_wp', {});
  const btn = document.getElementById('btn-continue');
  btn.textContent = 'SENT ✓';
  setTimeout(() => { btn.textContent = 'CONTINUE →'; }, 800);
});

// ── Pause mode toggle ────────────────────────────────────────
document.getElementById('btn-pause-mode').addEventListener('click', () => {
  const newMode = currentPauseMode === 'all' ? 'type' : 'all';
  sendCmd('set_pause_mode', { mode: newMode });
  document.getElementById('btn-pause-mode').textContent =
    newMode === 'all' ? 'Stop: All WPs' : 'Stop: Pause WPs';
});

// ── CSV import ───────────────────────────────────────────────
const csvInput    = document.getElementById('csv-input');
const csvFilename = document.getElementById('csv-filename');

csvInput.addEventListener('change', (e) => {
  const file = e.target.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = (ev) => {
    sendCmd('load_csv', { content: ev.target.result });
    csvFilename.textContent = file.name;
  };
  reader.readAsText(file);
  csvInput.value = '';  // allow re-selecting the same file
});

// ── GEN PATH ─────────────────────────────────────────────────
const btnGenPath = document.getElementById('btn-gen-path');
btnGenPath.addEventListener('click', () => {
  if (_curLat == null || _curLon == null) {
    alert('No GPS fix — wait for RTK data.');
    return;
  }
  sendCmd('gen_path', { lat: _curLat, lon: _curLon });
  const orig = btnGenPath.textContent;
  btnGenPath.textContent = 'SENDING…';
  btnGenPath.disabled = true;
  setTimeout(() => {
    btnGenPath.textContent = 'DONE ✓';
    btnGenPath.disabled = false;
    setTimeout(() => { btnGenPath.textContent = orig; }, 1500);
  }, 800);
});

// ── Lateral offset ────────────────────────────────────────────
const offsetSlider = document.getElementById('offset-slider');
const offsetValEl  = document.getElementById('offset-value');

offsetSlider.addEventListener('input', () => {
  offsetValEl.textContent = parseFloat(offsetSlider.value).toFixed(1) + ' m';
  sendCmd('set_offset', { offset_m: parseFloat(offsetSlider.value) });
});

document.getElementById('btn-offset-apply').addEventListener('click', () => {
  sendCmd('set_offset', { offset_m: parseFloat(offsetSlider.value) });
});

document.getElementById('btn-export-csv').addEventListener('click', () => {
  window.location.href = '/export_csv';
});

// ── Path map SVG ──────────────────────────────────────────────
const SVG_W = 260, SVG_H = 200, SVG_PAD = 14;

function _getBounds(...arrays) {
  const all = arrays.flat().filter(p => p && p.lat != null);
  if (!all.length) return null;
  return {
    minLat: Math.min(...all.map(p => p.lat)),
    maxLat: Math.max(...all.map(p => p.lat)),
    minLon: Math.min(...all.map(p => p.lon)),
    maxLon: Math.max(...all.map(p => p.lon)),
  };
}

function _toSvgPts(points, bounds) {
  const { minLat, maxLat, minLon, maxLon } = bounds;
  const dLat = maxLat - minLat || 1e-6;
  const dLon = maxLon - minLon || 1e-6;
  const W = SVG_W - 2 * SVG_PAD, H = SVG_H - 2 * SVG_PAD;
  return points.map(p => {
    const x = SVG_PAD + (p.lon - minLon) / dLon * W;
    const y = SVG_PAD + (1 - (p.lat - minLat) / dLat) * H;
    return `${x.toFixed(1)},${y.toFixed(1)}`;
  }).join(' ');
}

function _setSvgCircle(id, pt, bounds) {
  const el = document.getElementById(id);
  if (pt) {
    const coords = _toSvgPts([pt], bounds).split(',');
    el.setAttribute('cx', coords[0]);
    el.setAttribute('cy', coords[1]);
  } else {
    el.setAttribute('cx', -10);
    el.setAttribute('cy', -10);
  }
}

function updatePathMap(d) {
  const orig  = d.path_original || [];
  const off   = d.path_offset   || [];
  const robot = (d.robot_lat != null) ? { lat: d.robot_lat, lon: d.robot_lon } : null;
  const wpTarget = (d.current_wp_idx != null && off[d.current_wp_idx]) ? off[d.current_wp_idx] : null;

  const bounds = _getBounds(orig, off, robot ? [robot] : []);
  if (!bounds) return;

  document.getElementById('pm-original').setAttribute('points', orig.length ? _toSvgPts(orig, bounds) : '');
  document.getElementById('pm-offset').setAttribute('points',   off.length  ? _toSvgPts(off,  bounds) : '');
  _setSvgCircle('pm-robot',  robot,    bounds);
  _setSvgCircle('pm-target', wpTarget, bounds);

  if (orig.length) {
    document.getElementById('path-map-info').textContent =
      `${orig.length}pts  off:${(d.offset_m || 0).toFixed(1)}m`;
  }
}

// ── Lift control ─────────────────────────────────────────────
function sendLift(cmd) { sendCmd('lift_control', { cmd }); }

document.getElementById('btn-lift-set').addEventListener('click', () => {
  const up_s   = parseFloat(document.getElementById('lift-up-s').value)   || 5;
  const down_s = parseFloat(document.getElementById('lift-down-s').value) || 5;
  sendCmd('set_lift_duration', { up_s, down_s });
});

const liftUp   = document.getElementById('btn-lift-up');
const liftStop = document.getElementById('btn-lift-stop');
const liftDown = document.getElementById('btn-lift-down');

liftUp.addEventListener('pointerdown',   () => sendLift('up'));
liftUp.addEventListener('pointerup',     () => sendLift('stop'));
liftUp.addEventListener('pointerleave',  () => sendLift('stop'));
liftUp.addEventListener('pointercancel', () => sendLift('stop'));

liftDown.addEventListener('pointerdown',   () => sendLift('down'));
liftDown.addEventListener('pointerup',     () => sendLift('stop'));
liftDown.addEventListener('pointerleave',  () => sendLift('stop'));
liftDown.addEventListener('pointercancel', () => sendLift('stop'));

liftStop.addEventListener('click', () => sendLift('stop'));

// ── Boot ─────────────────────────────────────────────────────
connectAutoNav();
