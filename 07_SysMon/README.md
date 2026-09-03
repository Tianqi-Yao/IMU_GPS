# 07_SysMon — CPU Thermal/Load/Memory Bridge

*[中文版](README_zh.md)*

Samples macOS CPU thermal-throttle status, load average, swap, and memory
pressure on a fixed interval and broadcasts them as JSON over WebSocket.
Built to help diagnose intermittent low camera FPS on an outdoor Mac mini —
the suspicion is thermal throttling from ambient heat, but load saturation
(7 concurrent bridges) and memory pressure are equally plausible and produce
the same symptom, so this module distinguishes between them.

## Architecture

- **CORE** — `SysSampler.sample()`: no accumulated state, just shells out
  to `pmset -g therm` (the no-sudo Apple Silicon signal for CPU speed/
  scheduler throttling), `os.getloadavg()`/`os.cpu_count()`, `sysctl
  vm.swapusage`, and `sysctl kern.memorystatus_vm_pressure_level` (the same
  signal Activity Monitor's memory-pressure gauge uses). Each source is
  parsed independently — a format change in one command degrades only that
  field to `null` instead of crashing the sampler.
- **OUTPUT** —
  - `common.ws_server.BroadcastWsServer` broadcasts `sysmon_frame` payloads
    every `SYSMON_INTERVAL_S` seconds, for the live dashboard.
  - `JsonlWriter` appends every sample to `sysmon_data/sysmon_YYYYMMDD.jsonl`
    (day-rotating), independent of whether `record_all.py` is running — so a
    history exists automatically for whenever an FPS-drop incident actually
    happens, without needing to remember to start a recording session first.
  - `common.http_server.StaticFileServer` serves `web_static/`: a live
    dashboard (stat cards + hand-rolled canvas line charts, no external
    chart library) plus a "load log file" control that renders the same
    charts from any saved `sysmon_data/*.jsonl` file entirely client-side
    (FileReader, no server round trip) — for reviewing a past incident.

## Output contract (WebSocket, `sysmon_frame`, every `SYSMON_INTERVAL_S`)

```json
{
  "type": "sysmon_frame", "version": 1,
  "cpu_speed_limit": 45, "cpu_scheduler_limit": 100, "throttled": true,
  "loadavg_1": 2.05, "loadavg_5": 1.87, "loadavg_15": 2.17, "cpu_count": 8,
  "swap_used_mb": 0.0, "mem_pressure_level": 1,
  "ts": 1788452819.16
}
```

| field | meaning |
|---|---|
| `cpu_speed_limit` / `cpu_scheduler_limit` | from `pmset -g therm`; percent of full speed/schedulable cores, 100 = no limit, `null` if no throttle event has ever been recorded since boot |
| `throttled` | `true` if either limit above is below 100 |
| `loadavg_1/5/15` | `os.getloadavg()` — CPU pressure independent of thermal state |
| `cpu_count` | `os.cpu_count()`, for comparing load average against available cores |
| `swap_used_mb` | from `sysctl vm.swapusage` |
| `mem_pressure_level` | from `sysctl kern.memorystatus_vm_pressure_level`: 1=normal, 2=warn, 4=critical |
| `ts` | bridge-side `time.time()` at sample |

`sysmon_bridge.log` also gets an edge-triggered `WARNING` the moment
`throttled` flips to `true` (and an `INFO` when it clears), so grepping the
log alone is enough to confirm a throttling event even without opening the
dashboard.

## Record / replay

- `sysmon_data/sysmon_YYYYMMDD.jsonl` — always-on, day-rotating log written
  directly by the bridge (see above). This is the primary way to catch an
  unattended incident.
- Also wired into `record_all.py`'s `WS_STREAMS`, so a deliberate recording
  session's `data_log/session_*/sysmon.jsonl` sits timestamp-aligned next to
  that session's `camera_status.jsonl` (which carries `cam1_fps`/`cam2_fps`)
  for direct incident correlation.

## Run

```bash
python sysmon_bridge.py
```

Configure `SYSMON_WS_PORT` / `SYSMON_INTERVAL_S` in the repo-root
`config.py`. HTTP page on `SYSMON_WS_PORT` (default 8825), WebSocket on
`SYSMON_WS_PORT + 1` (default 8826).
