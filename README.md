# IMU_GPS

*[中文版](README_zh.md)*

A Raspberry Pi perception and autonomous-navigation stack for a field
robot: a BNO085 IMU and an RTK GPS receiver feed real-time attitude and
centimeter-level position over WebSocket to browser dashboards and to an
autonomous navigation loop, which drives a chassis (Feather M4 over CAN to
an Amiga VCU) along a waypoint path; an OAK-D depth camera streams video
through a hot-swappable image-processing plugin pipeline (including a
gesture-control plugin that can drive the robot directly); a QR launcher
page gives phones on the LAN one-tap access to every module's dashboard.

## Architecture: independent bridge processes

The system is **7 independent processes** ("bridges"), each owning one
piece of hardware or one computation, talking to each other only over
WebSocket/HTTP. Any bridge can be started, debugged, restarted, or swapped
for a different implementation on its own, as long as its external
contract (the WebSocket JSON schema, or the serial protocol) stays the
same — nothing here imports another module's business logic.

| module | entry point | HTTP | WS (HTTP+1) | talks to | role |
|---|---|---|---|---|---|
| `00_QR` | `qr_server.py` | 8700 | — (static page only) | — | LAN QR launcher + WiFi-join page |
| `01_IMU` | `imu_bridge.py` | 8765 | 8766 | BNO085 (serial) | attitude/heading |
| `02_RTK` | `rtk_bridge.py` | 8775 | 8776 | RTK GPS (serial, NMEA) | position fix |
| `03_Nav` | `nav_bridge.py` | 8785 | 8786 | 01_IMU, 02_RTK (WS) | IMU+RTK dashboard passthrough (no fusion) |
| `04_Robot` | `robot_bridge.py` | 8888 | 8889 | Feather M4 (serial) | drive control + telemetry |
| `05_AutoNav` | `autonav_bridge.py` | 8805 | 8806 | 01_IMU, 02_RTK, 04_Robot (WS) | waypoint-following autonomy |
| `06_Camera` | `camera_bridge.py` | 8815 | 8816 | OAK-D (USB) | video streaming + vision plugins |

Every bridge's HTTP port serves its `web_static/` dashboard; its WebSocket
port is always `HTTP_PORT + 1` (`common/ports.py::derive_ws_port`).
MJPEG video itself (06_Camera) rides on separate ports
(`CAM1_STREAM_PORT`/`CAM2_STREAM_PORT`, default 8080/8081) since it isn't a
JSON/WebSocket feed.

## Design rules

- **Black-box contracts.** Modules talk to each other only through their
  WebSocket/HTTP/serial boundary, never by importing each other's code.
  02_RTK could be replaced by a different GPS stack entirely and nothing
  downstream would need to change, as long as it broadcasts the same
  `rtk_frame` JSON on the same port.
- **INPUT / CORE / OUTPUT layering inside each bridge.** Serial/WS/file
  reads (INPUT) are kept separate from parsing/algorithms/state machines
  (CORE — plain functions/objects, no socket handles, unit-testable
  standalone) and from WS broadcast/HTTP responses (OUTPUT). 05_AutoNav is
  the clearest example: all control-law math lives in `autonav_algo.py`,
  and `autonav_bridge.py` only does I/O and sequences the state machine.
- **`common/` is a zero-business-logic framework layer**, shared by every
  bridge: `ws_server.py` (broadcast + inbound dispatch), `http_server.py`
  (static file serving), `logging_setup.py`, `ports.py`. It never imports
  from `00_QR`..`06_Camera`, and no module-specific logic (coordinate
  transforms, PID gains, NMEA parsing, ...) is allowed to live here.
- **Record/replay for hardware-free development.** Every module ships a
  `listen_*.py` (observe and log real WebSocket traffic to
  `data_log/*.jsonl`) and a `replay_*.py` (rebroadcast a recorded file on
  the same port at a fixed rate), so downstream work never strictly
  requires the upstream hardware to be attached.

## Repository layout

```
00_QR/       LAN QR launcher (no README — see qr_server.py)
01_IMU/      BNO085 attitude bridge
02_RTK/      RTK GPS bridge
03_Nav/      IMU+RTK dashboard passthrough
04_Robot/    Robot serial control bridge (Feather M4 / Amiga CAN)
05_AutoNav/  Autonomous waypoint navigation
06_Camera/   OAK-D streaming + image-processing plugins
common/      Shared framework: ws_server, http_server, logging, ports
CIRCUITPY/   Feather M4 firmware (CircuitPython, drives the Amiga VCU over CAN)
config.py    All tunable parameters for every module, grouped by number
start_bridges.sh   Start all 7 bridges, each in its own tmux window
start_helpers.sh   Menu to start listen_*/replay_*/send_demo helper tools
record_all.py      Record every running bridge's WS output into one session
doc/         Design/refactor notes and project write-ups
```

## Getting started

```bash
# Start all 7 bridges, each in its own tmux window
./start_bridges.sh

# Start listen_*/replay_*/send_demo helper tools via a menu
./start_helpers.sh

# Record every running bridge's WS output (+ both camera MJPEG streams)
# into one timestamped session directory
python3 record_all.py
```

See [QUICKSTART.md](QUICKSTART.md) for serial port setup, the dashboard
URLs, and the full record/replay walkthrough.

## Configuration

All tunable parameters (serial ports, baud rates, WebSocket URLs, PID/
control gains, ...) live in the repo-root `config.py`, grouped by module
number. Edit the file and restart the relevant bridge to apply changes —
no CLI arguments are added by default. Each consumer module defines its
own upstream WebSocket URL variables (e.g. `ROBOT_IMU_WS`/`ROBOT_RTK_WS`)
rather than borrowing another module's, so changing one module's upstream
address never accidentally affects another.

## Per-module documentation

Each module's `README.md` (with a `README_zh.md` Chinese counterpart)
documents its real WebSocket/serial contract with a JSON example, its
control messages, and its record/replay workflow:

- [01_IMU](01_IMU/README.md) — wire protocol from the ESP32C3 firmware, `imu_frame` contract, north-offset calibration
- [02_RTK](02_RTK/README.md) — NMEA parsing, `rtk_frame` contract, fix-quality codes
- [03_Nav](03_Nav/README.md) — why this is a passthrough, not sensor fusion
- [04_Robot](04_Robot/README.md) — Feather M4 serial protocol, joystick/telemetry contract
- [05_AutoNav](05_AutoNav/README.md) — the control law, state machine, waypoint file format
- [06_Camera](06_Camera/README.md) — plugin interface, `pose_control` gesture-driving plugin

## Firmware

`CIRCUITPY/` (Feather M4, drives the Amiga VCU over CAN — see
`04_Robot/README.md` for the serial protocol) and
`01_IMU/bno085_esp32c3/bno085_esp32c3.ino` (BNO085 IMU firmware — see
`01_IMU/README.md` for the wire protocol) implement the two serial
protocols this stack depends on; changes to either must stay in sync with
the corresponding bridge's parser.
