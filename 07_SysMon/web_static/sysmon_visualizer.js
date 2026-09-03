/**
 * sysmon_visualizer.js — WebSocket client + charts for the SysMon dashboard.
 *
 * Two modes:
 *  - live:    buffers sysmon_frame messages from the bridge's WebSocket
 *             (last LIVE_WINDOW_MS), redraws on every message.
 *  - history: a previously-saved sysmon_data/*.jsonl file is loaded via
 *             FileReader (all client-side, no server round trip) and
 *             rendered with the same chart code, so a past FPS-drop
 *             incident can be reviewed without needing the bridge running
 *             at the time.
 */

// ── DOM Elements ─────────────────────────────────────────────────────────────

const statusDot      = document.getElementById("statusDot");
const statusText     = document.getElementById("statusText");
const bannerText     = document.getElementById("bannerText");
const overallBanner  = document.getElementById("overallBanner");
const cardThrottled  = document.getElementById("cardThrottled");
const valThrottled   = document.getElementById("valThrottled");
const valSpeedLimit  = document.getElementById("valSpeedLimit");
const valSchedLimit  = document.getElementById("valSchedLimit");
const valLoadAvg     = document.getElementById("valLoadAvg");
const valCpuCount    = document.getElementById("valCpuCount");
const valSwap        = document.getElementById("valSwap");
const cardMemPressure = document.getElementById("cardMemPressure");
const valMemPressure = document.getElementById("valMemPressure");
const valLastUpdate  = document.getElementById("valLastUpdate");
const rawJson        = document.getElementById("rawJson");
const historyFile    = document.getElementById("historyFile");
const historyInfo    = document.getElementById("historyInfo");
const btnBackToLive  = document.getElementById("btnBackToLive");

const chartLimit = document.getElementById("chartLimit");
const chartLoad  = document.getElementById("chartLoad");
const chartSwap  = document.getElementById("chartSwap");
const chartMem   = document.getElementById("chartMem");

// ── State ────────────────────────────────────────────────────────────────────

const LIVE_WINDOW_MS = 10 * 60 * 1000; // rolling 10-minute live window
const MAX_RENDER_POINTS = 1200;        // downsample history files above this many samples

let mode = "live"; // "live" | "history"
let liveBuffer = []; // [{...frame, ts}]
let historyBuffer = [];

// ── WebSocket ────────────────────────────────────────────────────────────────

const WS_PORT = (parseInt(window.location.port, 10) || 8825) + 1;
const WS_URL  = `ws://${window.location.hostname}:${WS_PORT}`;

let ws = null;
let reconnectTimer = null;

function connect() {
  ws = new WebSocket(WS_URL);

  ws.onopen = () => {
    statusDot.classList.add("connected");
    statusText.textContent = "Connected";
  };

  ws.onclose = () => {
    statusDot.classList.remove("connected");
    statusText.textContent = "Disconnected";
    reconnectTimer = setTimeout(connect, 2000);
  };

  ws.onerror = () => ws.close();

  ws.onmessage = (event) => {
    let frame;
    try {
      frame = JSON.parse(event.data);
    } catch (e) {
      return;
    }
    if (frame.type !== "sysmon_frame") return;

    liveBuffer.push(frame);
    const cutoff = Date.now() / 1000 - LIVE_WINDOW_MS / 1000;
    while (liveBuffer.length && liveBuffer[0].ts < cutoff) liveBuffer.shift();

    updateStatCards(frame);
    if (mode === "live") render(liveBuffer);
  };
}

connect();

// ── Stat cards ───────────────────────────────────────────────────────────────

function fmtLimit(v) {
  return v === null || v === undefined ? "—" : `${v}%`;
}

function memPressureLabel(level) {
  if (level === 1) return "Normal";
  if (level === 2) return "Warn";
  if (level === 4) return "Critical";
  return "—";
}

function setCardState(card, state) {
  card.classList.remove("state-ok", "state-warn", "state-critical");
  if (state) card.classList.add(state);
}

/**
 * Reduces a frame to a single "is anything unusual right now" verdict, so
 * the user doesn't have to read four separate charts to answer that
 * question. `throttled` wins outright since it's macOS's own thermal
 * verdict (pmset -g therm), not a derived proxy like the other checks.
 */
function computeOverallState(frame) {
  if (frame.throttled) {
    return { level: "critical", label: "Throttled", detail: "macOS has reduced CPU speed/scheduling due to thermal limits." };
  }
  if (frame.mem_pressure_level === 4) {
    return { level: "critical", label: "Memory Pressure: Critical", detail: "System is critically low on memory." };
  }
  const reasons = [];
  if (frame.loadavg_1 > frame.cpu_count) reasons.push("High load (1m avg exceeds core count)");
  if (frame.mem_pressure_level === 2) reasons.push("Memory pressure: Warn");
  if (frame.swap_used_mb > 0) reasons.push("Swap in use");
  if (reasons.length) {
    return { level: "warn", label: "Elevated", detail: reasons.join("; ") };
  }
  return { level: "ok", label: "System Normal", detail: "No throttling, load, or memory pressure issues detected." };
}

function updateOverallBanner(frame) {
  const state = computeOverallState(frame);
  overallBanner.classList.remove("state-ok", "state-warn", "state-critical");
  overallBanner.classList.add(`state-${state.level}`);
  bannerText.textContent = `${state.label} — ${state.detail}`;
}

function updateStatCards(frame) {
  updateOverallBanner(frame);

  valThrottled.textContent = frame.throttled ? "Throttled" : "Normal";
  setCardState(cardThrottled, frame.throttled ? "state-critical" : "state-ok");

  valSpeedLimit.textContent = fmtLimit(frame.cpu_speed_limit);
  valSchedLimit.textContent = fmtLimit(frame.cpu_scheduler_limit);
  valLoadAvg.textContent = `${frame.loadavg_1.toFixed(2)} / ${frame.loadavg_5.toFixed(2)} / ${frame.loadavg_15.toFixed(2)}`;
  valCpuCount.textContent = frame.cpu_count;
  valSwap.textContent = frame.swap_used_mb === null || frame.swap_used_mb === undefined
    ? "—" : `${frame.swap_used_mb.toFixed(1)} MB`;

  valMemPressure.textContent = memPressureLabel(frame.mem_pressure_level);
  setCardState(cardMemPressure,
    frame.mem_pressure_level === 4 ? "state-critical" :
    frame.mem_pressure_level === 2 ? "state-warn" : "state-ok");

  valLastUpdate.textContent = new Date(frame.ts * 1000).toLocaleTimeString();
  rawJson.textContent = JSON.stringify(frame, null, 2);
}

// ── History import ───────────────────────────────────────────────────────────

historyFile.addEventListener("change", () => {
  const file = historyFile.files[0];
  if (!file) return;
  const reader = new FileReader();
  reader.onload = () => {
    const lines = reader.result.split("\n").filter((l) => l.trim());
    const frames = [];
    for (const line of lines) {
      try {
        const obj = JSON.parse(line);
        if (obj.type === "sysmon_frame") frames.push(obj);
      } catch (e) {
        // skip malformed lines
      }
    }
    if (!frames.length) {
      historyInfo.textContent = "No valid sysmon_frame records found in this file";
      return;
    }
    historyBuffer = frames;
    mode = "history";
    btnBackToLive.style.display = "inline-block";
    const start = new Date(frames[0].ts * 1000).toLocaleString();
    const end = new Date(frames[frames.length - 1].ts * 1000).toLocaleString();
    historyInfo.textContent = `${file.name} — ${frames.length} samples (${start} ~ ${end})`;
    updateStatCards(frames[frames.length - 1]);
    render(downsample(frames, MAX_RENDER_POINTS));
  };
  reader.readAsText(file);
});

btnBackToLive.addEventListener("click", () => {
  mode = "live";
  btnBackToLive.style.display = "none";
  historyInfo.textContent = "";
  historyFile.value = "";
  render(liveBuffer);
});

function downsample(frames, maxPoints) {
  if (frames.length <= maxPoints) return frames;
  const step = Math.ceil(frames.length / maxPoints);
  const out = [];
  for (let i = 0; i < frames.length; i += step) out.push(frames[i]);
  return out;
}

// ── Charts (hand-rolled canvas line charts — no external dependency) ────────

function setupCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const cssWidth = canvas.clientWidth || canvas.parentElement.clientWidth;
  const cssHeight = canvas.height; // literal height="140" attribute, in CSS px
  canvas.width = cssWidth * dpr;
  canvas.height = cssHeight * dpr;
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { ctx, width: cssWidth, height: cssHeight };
}

/**
 * series: [{ color, points: [[t, v], ...] }]
 * opts: { yMin, yMax, unit, yTicks: [{ value, label }], refLines: [{ value, color }] }
 * `unit` is appended to the default (auto min/mid/max) tick numbers, e.g. "%", " MB".
 * `yTicks`, if given, replaces the default 3-tick grid with lines/labels at
 * specific values (e.g. the memory-pressure chart draws its gridlines at
 * the real 1/2/4 levels, labeled "Normal/Warn/Critical", instead of a
 * generic min/mid/max split).
 */
function drawChart(canvas, series, opts) {
  const { ctx, width, height } = setupCanvas(canvas);
  ctx.clearRect(0, 0, width, height);

  const padL = 56, padR = 8, padT = 8, padB = 16;
  const plotW = Math.max(1, width - padL - padR);
  const plotH = Math.max(1, height - padT - padB);

  const allPoints = series.flatMap((s) => s.points);
  if (!allPoints.length) {
    ctx.fillStyle = "#7d8590";
    ctx.font = "12px sans-serif";
    ctx.fillText("Waiting for data…", padL, height / 2);
    return;
  }

  const ts = allPoints.map((p) => p[0]);
  const tMin = Math.min(...ts), tMax = Math.max(...ts) || tMin + 1;

  let yMin = opts.yMin, yMax = opts.yMax;
  if (yMin === undefined || yMax === undefined) {
    // Include reference-line values in the auto-range, otherwise a refLine
    // sitting outside the data's own min/max (e.g. cpu_count=8 with load
    // averages around 1-3) gets computed right off the top of the canvas
    // and never actually renders.
    const refValues = (opts.refLines || []).map((r) => r.value);
    const vs = allPoints.map((p) => p[1]).concat(refValues);
    yMin = opts.yMin !== undefined ? opts.yMin : Math.min(...vs);
    yMax = opts.yMax !== undefined ? opts.yMax : Math.max(...vs);
    if (yMax - yMin < 1e-6) { yMin -= 1; yMax += 1; }
    const pad = (yMax - yMin) * 0.1;
    yMin -= pad; yMax += pad;
  }

  const x = (t) => padL + ((t - tMin) / (tMax - tMin || 1)) * plotW;
  const y = (v) => padT + plotH - ((v - yMin) / (yMax - yMin || 1)) * plotH;

  // grid + y-axis labels (min/mid/max)
  ctx.strokeStyle = "#30363d";
  ctx.fillStyle = "#7d8590";
  ctx.font = "10px monospace";
  ctx.lineWidth = 1;
  const unit = opts.unit || "";
  const ticks = opts.yTicks || [yMin, (yMin + yMax) / 2, yMax].map((v) => ({ value: v, label: `${v.toFixed(1)}${unit}` }));
  ticks.forEach((tick) => {
    const yy = y(tick.value);
    ctx.beginPath();
    ctx.moveTo(padL, yy);
    ctx.lineTo(padL + plotW, yy);
    ctx.stroke();
    ctx.fillText(tick.label, 2, yy + 3);
  });

  // reference lines (e.g. core count)
  (opts.refLines || []).forEach((ref) => {
    const yy = y(ref.value);
    ctx.strokeStyle = ref.color || "#7d8590";
    ctx.setLineDash([4, 3]);
    ctx.beginPath();
    ctx.moveTo(padL, yy);
    ctx.lineTo(padL + plotW, yy);
    ctx.stroke();
    ctx.setLineDash([]);
  });

  // series lines
  series.forEach((s) => {
    if (s.points.length < 2) return;
    ctx.strokeStyle = s.color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    s.points.forEach((p, i) => {
      const px = x(p[0]), py = y(p[1]);
      if (i === 0) ctx.moveTo(px, py); else ctx.lineTo(px, py);
    });
    ctx.stroke();
  });
}

function render(buffer) {
  if (!buffer.length) return;
  const last = buffer[buffer.length - 1];

  drawChart(chartLimit, [
    { color: "#58a6ff", points: buffer.map((f) => [f.ts, f.cpu_speed_limit ?? 100]) },
    { color: "#d29922", points: buffer.map((f) => [f.ts, f.cpu_scheduler_limit ?? 100]) },
  ], { yMin: 0, yMax: 100, unit: "%" });

  drawChart(chartLoad, [
    { color: "#3fb950", points: buffer.map((f) => [f.ts, f.loadavg_1]) },
    { color: "#58a6ff", points: buffer.map((f) => [f.ts, f.loadavg_5]) },
    { color: "#d29922", points: buffer.map((f) => [f.ts, f.loadavg_15]) },
  ], { yMin: 0, unit: " procs", refLines: [{ value: last.cpu_count, color: "#7d8590" }] });

  drawChart(chartSwap, [
    { color: "#f85149", points: buffer.map((f) => [f.ts, f.swap_used_mb ?? 0]) },
  ], { yMin: 0, unit: " MB" });

  drawChart(chartMem, [
    { color: "#39d0d8", points: buffer.map((f) => [f.ts, f.mem_pressure_level ?? 1]) },
  ], {
    yMin: 0, yMax: 4.5,
    yTicks: [{ value: 1, label: "Normal" }, { value: 2, label: "Warn" }, { value: 4, label: "Critical" }],
  });
}

window.addEventListener("resize", () => render(mode === "live" ? liveBuffer : historyBuffer));
