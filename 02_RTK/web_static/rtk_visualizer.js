// Map center used only before the WS connection delivers any frame (or while
// every frame so far is still the server's config.py fallback coordinate,
// source:"default"). Once a real fix arrives, updateByFrame() recenters the
// map — this constant is a startup placeholder, not a synced copy of
// config.py's RTK_DEFAULT_LAT/LON.
const DEFAULT_POS = [38.9412928598587, -92.31884600793728];
const WS_URL = `ws://${window.location.hostname}:${Number(window.location.port || 8775) + 1}`;
const STRINGS = {
  title: 'RTK Path Visualizer',
  disconnected: 'Disconnected',
  connected: 'Connected',
  reconnecting: 'Disconnected, reconnecting in 3s',
  csvLabel: 'CSV Route File',
findMe: 'Find Me',
  centerCurrent: 'Center Current',
  editRoute: 'Edit Route',
  doneEdit: 'Finish Edit',
  undoNode: 'Undo Node',
  startSim: 'Start Simulation',
  stopSim: 'Stop Simulation',
  exportRoute: 'Export Route CSV',
  clearTrack: 'Clear Track',
  exportLog: 'Export Log',
  cardCurrent: 'Current Position',
  latitude: 'Latitude',
  longitude: 'Longitude',
  source: 'Source',
  heading: 'Heading',
  fix: 'Fix',
  satellites: 'Satellites',
  speed: 'Speed',
  cardMission: 'Mission Progress',
  reached: 'Reached Waypoints',
  target: 'Current Target',
  distance: 'Target Distance',
  cardEvents: 'Events',
  allReached: 'All reached',
  missingTiles: 'Satellite unavailable, switched to offline map',
  layerOffline: 'Offline Map (LAN/local)',
  layerSat: 'Satellite (Esri)',
  layerOsm: 'Street (OSM)',
  csvMissingLatLon: 'CSV missing lat/lon columns',
  cleared: 'Track and logs cleared',
  noLogs: 'No logs to export',
  noRoute: 'No route to export',
  routeExported: 'Exported {count} route points',
  reachedWp: 'Waypoint {id} reached ({dist} m)',
  wsConnected: 'WebSocket connected {url}',
  simStopped: 'Simulation stopped',
  simEmpty: 'No simulation path. Import CSV or edit route first',
  simStarted: 'Simulation started, {count} points',
  simDone: 'Simulation completed',
  editOn: 'Edit mode on: click map to add points (tol={tolerance}m, speed={speed}m/s)',
  editOff: 'Edit finished, generated {count} points',
  undoDone: 'Removed latest node #{id}',
  undoEmpty: 'No node to undo',
  geoUnsupported: 'Geolocation is not supported by this browser',
  geoOk: 'Located {lat}, {lon}',
  geoFail: 'Locate failed: {message}',
  centered: 'Centered to current position {lat}, {lon}',
  loadedPoints: 'Loaded {count} route points',
  csvLoadFail: 'CSV load failed: {message}',
  boot1: 'System started, default location loaded',
  boot2: 'Default base map is satellite map',
};

function t(key, vars = {}) {
  const raw = STRINGS[key] ?? key;
  return raw.replace(/\{(\w+)\}/g, (_, k) => vars[k] ?? `{${k}}`);
}

const map = L.map('map').setView(DEFAULT_POS, 19);

const offlineLayer = L.tileLayer('./assets/tiles/{z}/{x}/{y}.jpg', {
  maxZoom: 20,
  attribution: 'Offline tiles (LAN/local)',
});

const osmLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  maxZoom: 20,
  attribution: '&copy; OpenStreetMap contributors'
});

const esriSatLayer = L.tileLayer(
  'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {
    maxZoom: 20,
    attribution: 'Tiles &copy; Esri'
  }
);

let activeBaseLayer = null;
function switchBaseLayer(nextLayer) {
  if (activeBaseLayer === nextLayer) return;
  [offlineLayer, esriSatLayer, osmLayer].forEach((layer) => {
    if (map.hasLayer(layer)) map.removeLayer(layer);
  });
  nextLayer.addTo(map);
  activeBaseLayer = nextLayer;
}

switchBaseLayer(esriSatLayer);

let hasSwitchedToOffline = false;
esriSatLayer.on('tileerror', () => {
  if (hasSwitchedToOffline) return;
  hasSwitchedToOffline = true;
  switchBaseLayer(offlineLayer);
  addEvent(t('missingTiles'), '#b57812');
});

if (!navigator.onLine) {
  switchBaseLayer(offlineLayer);
}

window.addEventListener('offline', () => {
  switchBaseLayer(offlineLayer);
});

window.addEventListener('online', () => {
  if (activeBaseLayer === offlineLayer) {
    switchBaseLayer(esriSatLayer);
  }
});

let baseLayerControl = null;
function renderLayerControl() {
  if (baseLayerControl) map.removeControl(baseLayerControl);
  baseLayerControl = L.control.layers({
    [t('layerOffline')]: offlineLayer,
    [t('layerSat')]: esriSatLayer,
    [t('layerOsm')]: osmLayer,
  }).addTo(map);
}
renderLayerControl();
map.on('baselayerchange', (e) => {
  activeBaseLayer = e.layer;
});

const statusDot = document.getElementById('statusDot');
const statusText = document.getElementById('statusText');
const pageTitle = document.getElementById('pageTitle');
const csvLabel = document.getElementById('csvLabel');
const csvFile = document.getElementById('csvFile');
const btnFindMe = document.getElementById('btnFindMe');
const btnCenterCurrent = document.getElementById('btnCenterCurrent');
const btnEditRoute = document.getElementById('btnEditRoute');
const btnUndoNode = document.getElementById('btnUndoNode');
const editRouteOptions = document.getElementById('editRouteOptions');
const editTolerance = document.getElementById('editTolerance');
const editMaxSpeed = document.getElementById('editMaxSpeed');
const btnStartSim = document.getElementById('btnStartSim');
const btnExportRoute = document.getElementById('btnExportRoute');
const offsetDist      = document.getElementById('offsetDist');
const btnOffsetLeft   = document.getElementById('btnOffsetLeft');
const btnOffsetRight  = document.getElementById('btnOffsetRight');
const btnExportOffset = document.getElementById('btnExportOffset');
const btnClearOffset  = document.getElementById('btnClearOffset');
const btnClearTrack = document.getElementById('btnClearTrack');
const btnExportLog = document.getElementById('btnExportLog');
const cardCurrentTitle = document.getElementById('cardCurrentTitle');
const labelLat = document.getElementById('labelLat');
const labelLon = document.getElementById('labelLon');
const labelSource = document.getElementById('labelSource');
const labelSats = document.getElementById('labelSats');
const labelSpeed = document.getElementById('labelSpeed');
const cardMissionTitle = document.getElementById('cardMissionTitle');
const labelReached = document.getElementById('labelReached');
const labelTarget = document.getElementById('labelTarget');
const labelDistance = document.getElementById('labelDistance');
const cardEventTitle = document.getElementById('cardEventTitle');

let liveSocket = null;
let currentMarker = L.circleMarker(DEFAULT_POS, {
  radius: 8,
  color: '#073b4c',
  weight: 2,
  fillColor: '#118ab2',
  fillOpacity: 0.9,
}).addTo(map);

let waypoints = [];
let waypointMarkers = [];
let plannedPath = null;
let targetCircle = null;
let trackPoints = [];
let trackSegments = [];
let logs = [];
let isEditMode = false;
let insertHandles = [];
let simPath = [];
let simTimer = null;
let hasFirstFix = false;
let connectionState = 'disconnected';
let offsetWaypoints = [];
let offsetPath = null;

function makeWpIcon(state = 'normal', editMode = false) {
  const C = {
    normal:  { bg: '#cbd5e0', bd: '#4a5568' },
    active:  { bg: '#f6ad55', bd: '#b57812' },
    reached: { bg: '#6ee7b7', bd: '#1f8f46' },
  };
  const { bg, bd } = C[state] || C.normal;
  return L.divIcon({
    className: '',
    html: `<div style="width:12px;height:12px;border-radius:50%;background:${bg};border:2px solid ${bd};box-sizing:border-box;${editMode ? 'cursor:grab' : ''}"></div>`,
    iconSize: [12, 12], iconAnchor: [6, 6],
    tooltipAnchor: [6, 0], popupAnchor: [0, -6],
  });
}

function deleteWaypoint(idx) {
  if (idx < 0 || idx >= waypoints.length) return;
  const removed = waypoints.splice(idx, 1)[0];
  map.closePopup();
  redrawWaypoints({ fitView: false });
  addEvent(`Removed WP #${idx + 1} (id:${removed.id})`, '#6b7280');
}

function insertWaypointAt(idx, lat, lon) {
  const { tolerance, maxSpeed } = getEditParams();
  waypoints.splice(idx, 0, {
    id: `${idx}`,
    lat,
    lon,
    tolerance_m: tolerance,
    max_speed: maxSpeed,
    reached: false,
    reached_at: null,
  });
  redrawWaypoints({ fitView: false });
}

function redrawInsertHandles() {
  insertHandles.forEach((h) => h.remove());
  insertHandles = [];
  if (!isEditMode || waypoints.length < 2) return;
  for (let i = 0; i < waypoints.length - 1; i += 1) {
    const midLat = (waypoints[i].lat + waypoints[i + 1].lat) / 2;
    const midLon = (waypoints[i].lon + waypoints[i + 1].lon) / 2;
    const h = L.marker([midLat, midLon], {
      icon: L.divIcon({
        className: '',
        html: '<div style="width:16px;height:16px;border-radius:50%;background:#3182ce;border:2px solid #1a5ea8;box-sizing:border-box;display:flex;align-items:center;justify-content:center;color:#fff;font-size:13px;font-weight:700;line-height:1;cursor:pointer">+</div>',
        iconSize: [16, 16], iconAnchor: [8, 8],
      }),
      interactive: true,
    }).addTo(map);
    h.bindTooltip(`Insert between #${i + 1} and #${i + 2}`);
    const ci = i;
    h.on('click', (e) => { L.DomEvent.stop(e); insertWaypointAt(ci + 1, midLat, midLon); });
    insertHandles.push(h);
  }
}

function waypointTooltipHtml(wp, idx) {
  const reachedText = wp.reached ? 'yes' : 'no';
  const reachedAt = wp.reached_at || '-';
  const maxSpeed = wp.max_speed == null ? '-' : wp.max_speed;
  return [
    `ID: ${wp.id} (#${idx + 1})`,
    `lat: ${Number(wp.lat).toFixed(8)}`,
    `lon: ${Number(wp.lon).toFixed(8)}`,
    `tolerance_m: ${wp.tolerance_m}`,
    `max_speed: ${maxSpeed}`,
    `reached: ${reachedText}`,
    `reached_at: ${reachedAt}`,
  ].join('<br/>');
}

function setField(id, value) {
  const el = document.getElementById(id);
  if (el) el.textContent = value;
}

function setEditOptionsVisible(visible) {
  if (!editRouteOptions) return;
  editRouteOptions.classList.toggle('show', visible);
  editRouteOptions.setAttribute('aria-hidden', visible ? 'false' : 'true');
}

function getEditParams() {
  const tolRaw = Number(editTolerance?.value);
  const speedRaw = Number(editMaxSpeed?.value);
  const tolerance = Number.isFinite(tolRaw) && tolRaw > 0 ? tolRaw : 0.5;
  const maxSpeed = Number.isFinite(speedRaw) && speedRaw >= 0 ? speedRaw : 1;
  if (editTolerance) editTolerance.value = tolerance.toString();
  if (editMaxSpeed) editMaxSpeed.value = maxSpeed.toString();
  return { tolerance, maxSpeed };
}

function updateUndoButtonState() {
  if (!btnUndoNode) return;
  const canUndo = isEditMode && waypoints.length > 0;
  btnUndoNode.disabled = !canUndo;
}

function addEvent(text, color = '#1b6c8d') {
  const ul = document.getElementById('eventList');
  const li = document.createElement('li');
  li.textContent = text;
  li.style.borderLeftColor = color;
  ul.prepend(li);
  while (ul.children.length > 80) ul.removeChild(ul.lastChild);
}

function distanceMeters(lat1, lon1, lat2, lon2) {
  const R = 6371000;
  const toRad = (deg) => deg * Math.PI / 180;
  const dLat = toRad(lat2 - lat1);
  const dLon = toRad(lon2 - lon1);
  const a = Math.sin(dLat / 2) ** 2 +
            Math.cos(toRad(lat1)) * Math.cos(toRad(lat2)) * Math.sin(dLon / 2) ** 2;
  return 2 * R * Math.asin(Math.sqrt(a));
}

function interpolatePath(points, stepMeters = 0.5) {
  if (points.length < 2) return [...points];
  const result = [points[0]];
  for (let i = 1; i < points.length; i += 1) {
    const a = points[i - 1];
    const b = points[i];
    const d = distanceMeters(a[0], a[1], b[0], b[1]);
    const steps = Math.max(1, Math.ceil(d / stepMeters));
    for (let k = 1; k <= steps; k += 1) {
      const t = k / steps;
      result.push([
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
      ]);
    }
  }
  return result;
}

function computeOffsetWaypoints(wps, offsetM) {
  const MLAT = 111320.0;
  const MAX_MITER = 5.0; // cap at 5× to prevent runaway on near-180° hairpins

  return wps.map((w, i) => {
    const cosLat = Math.cos(w.lat * Math.PI / 180);

    // Left normal (CCW 90°) of segment a→b, in ENU meters, unit length
    function segNormal(a, b) {
      const dx = (b.lon - a.lon) * MLAT * cosLat;
      const dy = (b.lat - a.lat) * MLAT;
      const len = Math.sqrt(dx * dx + dy * dy);
      if (len < 1e-9) return null;
      return { x: -dy / len, y: dx / len };
    }

    const n1 = i > 0              ? segNormal(wps[i - 1], w)  : null;
    const n2 = i < wps.length - 1 ? segNormal(w, wps[i + 1]) : null;

    let perpEast, perpNorth;

    if (!n1 && !n2) {
      return { ...w, reached: false, reached_at: null };
    } else if (!n1 || !n2) {
      // Endpoint: simple perpendicular to the single adjacent segment
      const n = n1 || n2;
      perpEast  = n.x * offsetM;
      perpNorth = n.y * offsetM;
    } else {
      // Interior point: miter bisector so both adjacent offset segments are
      // exactly parallel (distance = offsetM) to the original segments.
      const mx = n1.x + n2.x;
      const my = n1.y + n2.y;
      const mlen = Math.sqrt(mx * mx + my * my);
      if (mlen < 1e-9) {
        // 180° hairpin: normals cancel; keep simple perpendicular from n1
        perpEast  = n1.x * offsetM;
        perpNorth = n1.y * offsetM;
      } else {
        const ux = mx / mlen;
        const uy = my / mlen;
        const dot = ux * n1.x + uy * n1.y; // cos(half_angle)
        const scale = dot < 1 / MAX_MITER ? MAX_MITER * offsetM : offsetM / dot;
        perpEast  = ux * scale;
        perpNorth = uy * scale;
      }
    }

    return {
      ...w,
      lat: w.lat + perpNorth / MLAT,
      lon: w.lon + perpEast / (MLAT * cosLat),
      reached: false,
      reached_at: null,
    };
  });
}

function clearOffsetPath() {
  if (offsetPath) { offsetPath.remove(); offsetPath = null; }
  offsetWaypoints = [];
  if (btnExportOffset) btnExportOffset.disabled = true;
  if (btnClearOffset)  btnClearOffset.disabled  = true;
}

function applyOffset(offsetM) {
  if (!waypoints.length) {
    addEvent('No route loaded to offset', '#c23a27');
    return;
  }
  clearOffsetPath();
  offsetWaypoints = computeOffsetWaypoints(waypoints, offsetM);
  offsetPath = L.polyline(
    offsetWaypoints.map((w) => [w.lat, w.lon]),
    { color: '#d97706', weight: 2.5, opacity: 0.9, dashArray: '5, 4' },
  ).addTo(map);
  if (btnExportOffset) btnExportOffset.disabled = false;
  if (btnClearOffset)  btnClearOffset.disabled  = false;
  const dir = offsetM > 0 ? 'left' : 'right';
  addEvent(`Offset path: ${dir} ${Math.abs(offsetM).toFixed(1)} m, ${offsetWaypoints.length} pts`, '#d97706');
}

function exportOffsetCsv() {
  if (!offsetWaypoints.length) return;
  const header = ['id', 'lat', 'lon', 'tolerance_m', 'max_speed'];
  const rows = offsetWaypoints.map((w, idx) => [
    w.id ?? `${idx}`,
    Number(w.lat).toFixed(8),
    Number(w.lon).toFixed(8),
    w.tolerance_m ?? 0.5,
    w.max_speed ?? '',
  ]);
  const csv = [header, ...rows].map((line) => line.join(',')).join('\n');
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `route_offset_${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
  a.click();
  URL.revokeObjectURL(url);
  addEvent(`Offset route exported: ${offsetWaypoints.length} pts`, '#1f8f46');
}

function parseCsvRows(text) {
  const lines = text.split(/\r?\n/).map((l) => l.trim()).filter(Boolean);
  if (!lines.length) return [];

  const splitCommaQuoted = (line) => {
    const out = [];
    let cur = '';
    let inQuotes = false;
    for (let i = 0; i < line.length; i += 1) {
      const ch = line[i];
      if (ch === '"') {
        if (inQuotes && line[i + 1] === '"') {
          cur += '"';
          i += 1;
        } else {
          inQuotes = !inQuotes;
        }
      } else if (ch === ',' && !inQuotes) {
        out.push(cur);
        cur = '';
      } else {
        cur += ch;
      }
    }
    out.push(cur);
    return out;
  };

  const splitLine = (line) => (line.includes('\t') ? line.split('\t') : splitCommaQuoted(line));
  const headers = splitLine(lines[0]).map((h) => h.trim().toLowerCase());

  const col = {
    id: headers.indexOf('id'),
    lat: headers.indexOf('lat'),
    lon: headers.indexOf('lon'),
    tolerance_m: headers.indexOf('tolerance_m'),
    max_speed: headers.indexOf('max_speed'),
  };

  if (col.lat < 0 || col.lon < 0) throw new Error(t('csvMissingLatLon'));

  return lines.slice(1).map((line, idx) => {
    const cells = splitLine(line).map((v) => v.trim());
    const id = col.id >= 0 ? (cells[col.id] || `${idx}`) : `${idx}`;
    const lat = Number(cells[col.lat]);
    const lon = Number(cells[col.lon]);
    const tolerance = col.tolerance_m >= 0 ? Number(cells[col.tolerance_m] || 0.5) : 0.5;
    const maxSpeed = col.max_speed >= 0 ? Number(cells[col.max_speed] || 0) : null;
    if (!Number.isFinite(lat) || !Number.isFinite(lon)) return null;
    return {
      id,
      lat,
      lon,
      tolerance_m: Number.isFinite(tolerance) && tolerance > 0 ? tolerance : 0.5,
      max_speed: Number.isFinite(maxSpeed) ? maxSpeed : null,
      reached: false,
      reached_at: null,
    };
  }).filter(Boolean);
}

function redrawWaypoints(options = {}) {
  const { fitView = true } = options;
  waypointMarkers.forEach((m) => m.remove());
  waypointMarkers = [];
  clearOffsetPath();

  if (plannedPath) {
    plannedPath.remove();
    plannedPath = null;
  }

  if (!waypoints.length) {
    setField('reachVal', '0 / 0');
    setField('targetVal', '-');
    setField('distVal', '-');
    simPath = [];
    updateUndoButtonState();
    return;
  }

  plannedPath = L.polyline(waypoints.map((w) => [w.lat, w.lon]), {
    color: '#2b6cb0',
    weight: 3,
    opacity: 0.85,
    dashArray: '8, 6',
  }).addTo(map);

  waypoints.forEach((w, idx) => {
    const marker = L.marker([w.lat, w.lon], {
      icon: makeWpIcon('normal', isEditMode),
      draggable: isEditMode,
    }).addTo(map);
    marker.bindTooltip(waypointTooltipHtml(w, idx), { permanent: false });

    if (isEditMode) {
      marker.on('drag', (e) => {
        if (!plannedPath) return;
        const ll = plannedPath.getLatLngs();
        ll[idx] = e.target.getLatLng();
        plannedPath.setLatLngs(ll);
      });
      marker.on('dragend', (e) => {
        const { lat, lng } = e.target.getLatLng();
        waypoints[idx].lat = lat;
        waypoints[idx].lon = lng;
        redrawWaypoints({ fitView: false });
      });
      marker.bindPopup(
        `<div style="text-align:center"><b>WP #${idx + 1}</b><br>` +
        `<button onclick="deleteWaypoint(${idx})" ` +
        `style="margin-top:6px;padding:4px 12px;background:#e53e3e;color:#fff;` +
        `border:none;border-radius:4px;cursor:pointer">Delete</button></div>`
      );
    }
    waypointMarkers[idx] = marker;
  });

  simPath = interpolatePath(waypoints.map((w) => [w.lat, w.lon]), 0.4);
  if (fitView) {
    const bounds = L.latLngBounds(waypoints.map((w) => [w.lat, w.lon]));
    bounds.extend(currentMarker.getLatLng());
    map.fitBounds(bounds.pad(0.2));
  }
  updateUndoButtonState();
  redrawInsertHandles();
}

function findCurrentTarget() {
  for (let i = 0; i < waypoints.length; i += 1) {
    if (!waypoints[i].reached) return { wp: waypoints[i], idx: i };
  }
  return null;
}

function updateWaypointStyles(activeIdx) {
  waypointMarkers.forEach((m, idx) => {
    const wp = waypoints[idx];
    let state = 'normal';
    if (wp.reached) state = 'reached';
    else if (idx === activeIdx) state = 'active';
    m.setIcon(makeWpIcon(state, isEditMode));
    m.setTooltipContent(waypointTooltipHtml(wp, idx));
  });
}

function addTrackSegment(a, b, color) {
  const seg = L.polyline([a, b], {
    color,
    weight: 5,
    opacity: 0.85,
    lineCap: 'round',
  }).addTo(map);
  trackSegments.push(seg);
}

function clearTrack() {
  trackSegments.forEach((s) => s.remove());
  trackSegments = [];
  trackPoints = [];
  logs = [];
  addEvent(t('cleared'), '#6b7280');
}

function exportLogs() {
  if (!logs.length) {
    addEvent(t('noLogs'), '#6b7280');
    return;
  }
  const header = [
    'timestamp','lat','lon','source','fix_quality','num_sats','hdop','speed_mps','target_id','target_distance_m','status'
  ];
  const rows = logs.map((r) => [
    r.timestamp, r.lat, r.lon, r.source, r.fix_quality, r.num_sats,
    r.hdop, r.speed_mps, r.target_id, r.target_distance_m, r.status
  ]);
  const csv = [header, ...rows].map((line) => line.join(',')).join('\n');
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `rtk_log_${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
  a.click();
  URL.revokeObjectURL(url);
}

function exportRouteCsv() {
  if (!waypoints.length) {
    addEvent(t('noRoute'), '#6b7280');
    return;
  }
  const header = ['id', 'lat', 'lon', 'tolerance_m', 'max_speed'];
  const rows = waypoints.map((w, idx) => [
    w.id ?? `${idx}`,
    Number(w.lat).toFixed(8),
    Number(w.lon).toFixed(8),
    w.tolerance_m ?? 0.5,
    w.max_speed ?? '',
  ]);
  const csv = [header, ...rows].map((line) => line.join(',')).join('\n');
  const blob = new Blob([csv], { type: 'text/csv;charset=utf-8;' });
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = `route_${new Date().toISOString().replace(/[:.]/g, '-')}.csv`;
  a.click();
  URL.revokeObjectURL(url);
  addEvent(t('routeExported', { count: waypoints.length }), '#1f8f46');
}

function pushFrame(frame) {
  updateByFrame(frame);
}

function updateByFrame(frame) {
  if (simTimer && frame.source !== 'sim') return;

  const lat = Number(frame.lat);
  const lon = Number(frame.lon);
  if (!Number.isFinite(lat) || !Number.isFinite(lon)) return;

  const point = [lat, lon];
  currentMarker.setLatLng(point);

  if (!hasFirstFix && frame.source !== 'default') {
    hasFirstFix = true;
    console.log('First RTK fix, moving map to', point);
    map.setView(point, map.getZoom());
  }

  setField('latVal', lat.toFixed(8));
  setField('lonVal', lon.toFixed(8));
  setField('srcVal', frame.source || 'unknown');
  setField('fixVal', `${frame.fix_quality ?? '-'} `);
  setField('satVal', `${frame.num_sats ?? '-'} `);
  setField('hdopVal', frame.hdop == null ? '-' : Number(frame.hdop).toFixed(2));

  const speedMps = frame.speed_knots == null ? null : Number(frame.speed_knots) * 0.514444;
  setField('spdVal', speedMps == null ? '-' : `${speedMps.toFixed(2)} m/s`);

  if (!trackPoints.length || distanceMeters(trackPoints.at(-1)[0], trackPoints.at(-1)[1], lat, lon) >= 0.2) {
    trackPoints.push(point);

    let status = 'no_target';
    let color = '#c23a27';
    let targetId = '';
    let targetDist = '';
    let activeIdx = -1;

    const targetInfo = findCurrentTarget();
    if (targetInfo) {
      const { wp, idx } = targetInfo;
      activeIdx = idx;
      const d = distanceMeters(lat, lon, wp.lat, wp.lon);
      targetId = wp.id;
      targetDist = d.toFixed(2);

      if (targetCircle) targetCircle.remove();
      targetCircle = L.circle([wp.lat, wp.lon], {
        radius: wp.tolerance_m,
        color: '#b57812',
        fillColor: '#fbd38d',
        fillOpacity: 0.18,
        weight: 2,
      }).addTo(map);

      if (d <= wp.tolerance_m) {
        color = '#1f8f46';
        status = 'reached';
        wp.reached = true;
        wp.reached_at = new Date().toISOString();
        addEvent(t('reachedWp', { id: wp.id, dist: d.toFixed(2) }), '#1f8f46');
      } else if (d <= wp.tolerance_m * 2) {
        color = '#b57812';
        status = 'approaching';
      } else {
        color = '#c23a27';
        status = 'off_path';
      }

      setField('targetVal', wp.id);
      setField('distVal', `${d.toFixed(2)} m`);
    } else {
      setField('targetVal', t('allReached'));
      setField('distVal', '-');
      if (targetCircle) {
        targetCircle.remove();
        targetCircle = null;
      }
      status = 'completed';
      color = '#1f8f46';
    }

    if (trackPoints.length >= 2) {
      addTrackSegment(trackPoints.at(-2), trackPoints.at(-1), color);
    }

    updateWaypointStyles(activeIdx);
    const reachedCount = waypoints.filter((w) => w.reached).length;
    setField('reachVal', `${reachedCount} / ${waypoints.length}`);

    logs.push({
      timestamp: new Date().toISOString(),
      lat: lat.toFixed(8),
      lon: lon.toFixed(8),
      source: frame.source || '',
      fix_quality: frame.fix_quality ?? '',
      num_sats: frame.num_sats ?? '',
      hdop: frame.hdop ?? '',
      speed_mps: speedMps == null ? '' : speedMps.toFixed(3),
      target_id: targetId,
      target_distance_m: targetDist,
      status,
    });
  }
}


function connectWebSocket() {
  const ws = new WebSocket(WS_URL);
  liveSocket = ws;

  ws.onopen = () => {
    connectionState = 'connected';
    statusDot.style.background = '#1f8f46';
    statusText.textContent = t('connected');
    addEvent(t('wsConnected', { url: WS_URL }), '#1f8f46');
  };

  ws.onmessage = (event) => {
    try {
      if (simTimer) return;
      const frame = JSON.parse(event.data);
pushFrame(frame);
    } catch (err) {
      console.error(err);
    }
  };

  ws.onclose = () => {
    if (liveSocket === ws) liveSocket = null;
    connectionState = 'reconnecting';
    statusDot.style.background = '#c23a27';
    statusText.textContent = t('reconnecting');
    setTimeout(connectWebSocket, 3000);
  };

  ws.onerror = () => ws.close();
}

function resetReachState() {
  waypoints.forEach((w) => {
    w.reached = false;
    w.reached_at = null;
  });
  updateWaypointStyles(-1);
  setField('reachVal', `0 / ${waypoints.length}`);
}

function stopSimulation() {
  if (simTimer) {
    clearInterval(simTimer);
    simTimer = null;
  }
  btnStartSim.textContent = t('startSim');
}

function startSimulation() {
  if (simTimer) {
    stopSimulation();
    addEvent(t('simStopped'), '#6b7280');
    return;
  }
  if (simPath.length < 2) {
    addEvent(t('simEmpty'), '#c23a27');
    return;
  }

  clearTrack();
  resetReachState();

  let i = 0;
  btnStartSim.textContent = t('stopSim');
  addEvent(t('simStarted', { count: simPath.length }), '#1b6c8d');

  simTimer = setInterval(() => {
    if (i >= simPath.length) {
      stopSimulation();
      addEvent(t('simDone'), '#1f8f46');
      return;
    }

    const p = simPath[i];
    const prev = i > 0 ? simPath[i - 1] : p;
    const d = distanceMeters(prev[0], prev[1], p[0], p[1]);
    const speedKnots = (d / 0.25) / 0.514444;

    pushFrame({
      lat: p[0],
      lon: p[1],
      source: 'sim',
      fix_quality: 4,
      num_sats: 18,
      hdop: 0.6,
      speed_knots: speedKnots,
    });

    i += 1;
  }, 250);
}

function addWaypointByMapClick(e) {
  if (!isEditMode) return;
  const lat = Number(e.latlng.lat);
  const lon = Number(e.latlng.lng);
  const { tolerance, maxSpeed } = getEditParams();
  waypoints.push({
    id: `${waypoints.length}`,
    lat,
    lon,
    tolerance_m: tolerance,
    max_speed: maxSpeed,
    reached: false,
    reached_at: null,
  });
  redrawWaypoints({ fitView: false });
  setField('reachVal', `0 / ${waypoints.length}`);
}

function undoLastWaypoint() {
  if (!isEditMode || !waypoints.length) {
    addEvent(t('undoEmpty'), '#6b7280');
    return;
  }
  const removed = waypoints.pop();
  redrawWaypoints({ fitView: false });
  setField('reachVal', `0 / ${waypoints.length}`);
  addEvent(t('undoDone', { id: removed.id }), '#6b7280');
}

function toggleEditRoute() {
  isEditMode = !isEditMode;
  if (!isEditMode) {
    insertHandles.forEach((h) => h.remove());
    insertHandles = [];
  }
  setEditOptionsVisible(isEditMode);
  btnEditRoute.textContent = isEditMode ? t('doneEdit') : t('editRoute');
  btnEditRoute.classList.toggle('active', isEditMode);
  updateUndoButtonState();
  if (isEditMode) {
    const { tolerance, maxSpeed } = getEditParams();
    stopSimulation();
    redrawWaypoints();
    addEvent(t('editOn', { tolerance, speed: maxSpeed }), '#b57812');
  } else {
    redrawWaypoints();
    addEvent(t('editOff', { count: waypoints.length }), '#1b6c8d');
  }
}

function findMe() {
  if (!navigator.geolocation) {
    addEvent(t('geoUnsupported'), '#c23a27');
    return;
  }

  navigator.geolocation.getCurrentPosition(
    (pos) => {
      const lat = pos.coords.latitude;
      const lon = pos.coords.longitude;
      const speedMps = Number.isFinite(pos.coords.speed) ? pos.coords.speed : null;
      const speedKnots = speedMps == null ? null : speedMps / 0.514444;
      map.setView([lat, lon], 19);
      pushFrame({
        lat,
        lon,
        source: 'find_me',
        fix_quality: 1,
        num_sats: '',
        hdop: Number.isFinite(pos.coords.accuracy) ? pos.coords.accuracy : '',
        speed_knots: speedKnots,
      });
      addEvent(t('geoOk', { lat: lat.toFixed(7), lon: lon.toFixed(7) }), '#1f8f46');
    },
    (err) => addEvent(t('geoFail', { message: err.message }), '#c23a27'),
    { enableHighAccuracy: true, timeout: 10000 }
  );
}

function centerToCurrent() {
  const p = currentMarker.getLatLng();
  map.setView([p.lat, p.lng], Math.max(map.getZoom(), 19));
  addEvent(t('centered', { lat: p.lat.toFixed(7), lon: p.lng.toFixed(7) }), '#1b6c8d');
}


csvFile.addEventListener('change', async (ev) => {
  const file = ev.target.files?.[0];
  if (!file) return;
  ev.target.value = '';  // allow re-selecting the same file

  try {
    const text = await file.text();
    waypoints = parseCsvRows(text);
    resetReachState();
    redrawWaypoints();
    updateWaypointStyles(0);
    addEvent(t('loadedPoints', { count: waypoints.length }), '#1b6c8d');
  } catch (err) {
    addEvent(t('csvLoadFail', { message: err.message }), '#c23a27');
  }
});

btnFindMe.addEventListener('click', findMe);
btnCenterCurrent.addEventListener('click', centerToCurrent);
btnEditRoute.addEventListener('click', toggleEditRoute);
btnUndoNode.addEventListener('click', undoLastWaypoint);
btnStartSim.addEventListener('click', startSimulation);
btnExportRoute.addEventListener('click', exportRouteCsv);
btnOffsetLeft.addEventListener('click', () => {
  const d = Math.abs(Number(offsetDist?.value) || 1.0);
  applyOffset(d);
});
btnOffsetRight.addEventListener('click', () => {
  const d = Math.abs(Number(offsetDist?.value) || 1.0);
  applyOffset(-d);
});
btnExportOffset.addEventListener('click', exportOffsetCsv);
btnClearOffset.addEventListener('click', clearOffsetPath);
btnClearTrack.addEventListener('click', clearTrack);
btnExportLog.addEventListener('click', exportLogs);

map.on('click', addWaypointByMapClick);


document.title = STRINGS.title;
pageTitle.textContent = STRINGS.title;
statusText.textContent = STRINGS.disconnected;
connectionState = 'disconnected';
statusText.textContent = t('disconnected');
addEvent(t('boot1'));
addEvent(t('boot2'));
connectWebSocket();
