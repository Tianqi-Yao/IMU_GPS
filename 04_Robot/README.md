# 04_Robot — Robot Serial Control Bridge

*[中文版](README_zh.md)*

Drives the robot chassis (Feather M4 + CAN bus to an Amiga VCU) from
browser joystick input or 05_AutoNav, and relays telemetry (odometry,
battery, upstream IMU/RTK) to any WebSocket consumer.

## Architecture

- **INPUT** — `SerialLink._reader_loop` (daemon thread, line-buffered) +
  inbound browser WS messages (`common.ws_server` dispatches to
  `RobotBridge._handle_client_message`).
- **CORE** — four plain objects with no socket/serial handles, unit
  testable standalone:
  - `RobotState` — parses `"O:"`/`"S:"` lines into odometry + the
    auto-active truth (see "State fix" below).
  - `VelocityRamp` — accel-limited setpoint smoothing (`_step_toward`),
    decoupled from WS message arrival rate; runs on its own 20 Hz timer.
  - `Watchdog` — heartbeat-timeout emergency stop (0.5s check period).
  - `Recorder` — rate-limited, per-type-slimmed `.jsonl` logging.
- **OUTPUT** — `SerialLink.write_velocity`/`write_raw`/`write_hbridge`
  (serial); `common.ws_server.BroadcastWsServer` (browser-facing broadcast,
  funneled through `RobotBridge._broadcast` so every message is also fed to
  the `Recorder`); `common.http_server.StaticFileServer` (serves
  `web_static/`, injecting `data-max-linear`/`data-max-angular` attributes
  into `index.html`'s `<html>` tag).

### State fix: single source of truth for `active`

The CAN-bus state integer carried on every `"O:"` line (see
`CIRCUITPY/lib/farm_ng/utils/packet.py::AmigaControlState`,
`STATE_AUTO_ACTIVE = 5`) is the **sole source of truth** for `active`,
re-synced on every `"O:"` line (~20 Hz). `"S:ACTIVE"`/`"S:READY"` lines are
applied immediately too — for a snappier UI response right after a manual
toggle — but they are no longer the *only* thing that can set `active`. If
the hardware silently downgrades to READY without emitting `"S:READY"` (a
CAN safety interlock, for example), the next `"O:"` line corrects the state
within ~50ms instead of leaving the bridge (and the dashboard) stuck
reporting a stale `ACTIVE` until the next manual toggle.

## Hardware protocol

- **Upstream (Pi -> Feather M4)**: `f"V{linear:.2f},{angular:.2f}\n"`,
  values normalized to `[-1.0, 1.0]` (m/s / rad/s). `f"H{U|D|S}\n"` for the
  lift actuator. A single `\r` byte toggles `AUTO_READY` <-> `AUTO_ACTIVE`.
- **Downstream (Feather M4 -> Pi)**: `"O:{v:.3f},{w:.3f},{state:d},{soc:d}\n"`
  at ~20 Hz; `"S:ACTIVE\n"` / `"S:READY\n"` confirmation lines on toggle.

Firmware (`CIRCUITPY/code.py`) is unchanged from the pre-refactor
implementation — the protocol was already verified correct.

## Output contract (WebSocket, multiple `type`s on one connection)

```json
{"type": "odom", "version": 1, "v": 0.31, "w": -0.02, "state": 5, "soc": 78, "ts": 1785049699.19}
{"type": "state_status", "version": 1, "active": true}
{"type": "imu", "version": 1, "...": "full imu_frame payload, forwarded from 01_IMU"}
{"type": "rtk", "version": 1, "available": true, "...": "full rtk_frame payload, forwarded from 02_RTK"}
{"type": "rec_status", "version": 1, "recording": true, "filename": "run_20260726_120000.jsonl"}
```

`rtk.available` is computed here (`source == "rtk"` or `fix_quality > 0`),
not present in the upstream `rtk_frame` itself.

## Control messages (browser -> bridge)

```json
{"type": "joystick", "linear": 0.3, "angular": 0.0}
{"type": "heartbeat"}
{"type": "toggle_state"}
{"type": "lift_control", "cmd": "up"}
{"type": "set_recording", "enabled": true}
```

`heartbeat` and `joystick` both feed the watchdog. Note: the browser
joystick UI's `force` field (nipplejs push magnitude) is not read by the
server — it is purely a client-side display value.

## Record / replay

- `listen_robot_websocket.py` — logs full messages to
  `data_log/robot_raw_{timestamp}.jsonl`.
- `replay_robot_websocket.py` — broadcasts the most recently recorded
  `data_log/*.jsonl` file at 20 Hz. `data_log/robot_raw_v1.jsonl` (carried
  over from the previous repo — its schema matches `Recorder._slim`'s
  odom/imu/rtk output) ships as the baseline sample so replay works out of
  the box.
- `send_robot_only_demo.py` — interactive terminal joystick, no browser
  needed.

## Run

```bash
python robot_bridge.py
```

Configure `ROBOT_SERIAL_PORT` / `ROBOT_SERIAL_BAUD` / `ROBOT_WS_PORT` /
`ROBOT_MAX_LINEAR` / `ROBOT_MAX_ANGULAR` / `ROBOT_WATCHDOG_TIMEOUT` /
`ROBOT_IMU_WS` / `ROBOT_RTK_WS` in the repo-root `config.py` (this module
defines its own `ROBOT_IMU_WS`/`ROBOT_RTK_WS` rather than borrowing
03_Nav's). HTTP page on `ROBOT_WS_PORT` (default 8888), WebSocket on
`ROBOT_WS_PORT + 1` (default 8889).
