# Quick Start

*[中文版](QUICKSTART_zh.md)*

## 1. Environment setup

```bash
pip install pyserial websockets depthai opencv-python numpy qrcode
```

- Python 3.10+
- tmux: `brew install tmux` (Mac) / `apt install tmux` (Linux)

> `qrcode` is only needed by `00_QR`; `depthai`/`opencv-python` are only
> needed by `06_Camera` — skip them if you don't have OAK-D hardware, the
> other modules are unaffected.

---

## 2. Configure serial ports

Open the repo-root `config.py` and edit the three serial device paths
(other parameters usually don't need changing):

| Parameter | Hardware | Mac example | Linux example |
|------|------|---------|-----------|
| `IMU_SERIAL_PORT` | BNO085 (ESP32-C3) | `/dev/cu.usbmodem1201` | `/dev/ttyACM0` |
| `RTK_SERIAL_PORT` | RTK GPS receiver | `/dev/cu.usbmodem1101` | `/dev/ttyACM1` |
| `ROBOT_SERIAL_PORT` | Feather M4 | `/dev/cu.usbmodem1301` | `/dev/ttyACM2` |

Find available serial ports:

```bash
ls /dev/cu.*          # Mac
ls /dev/ttyACM*       # Linux
```

---

## 3. Start all bridges

```bash
./start_bridges.sh
```

Starts all 7 modules as separate windows in a single tmux session
(`bridges`):

```
00_QR      → python3 00_QR/qr_server.py
01_IMU     → python3 01_IMU/imu_bridge.py
02_RTK     → python3 02_RTK/rtk_bridge.py
03_Nav     → python3 03_Nav/nav_bridge.py
04_Robot   → python3 04_Robot/robot_bridge.py
05_AutoNav → python3 05_AutoNav/autonav_bridge.py
06_Camera  → python3 06_Camera/camera_bridge.py
```

**Common tmux operations:**

| Action | Shortcut |
|------|--------|
| Switch window | `Ctrl-B` then the number/name of the window |
| Detach | `Ctrl-B D` |
| Reattach | `tmux attach -t bridges` |
| Kill everything | `tmux kill-session -t bridges` |

---

## 4. Open the browser UIs

| Module | URL | Notes |
|------|------|------|
| 00_QR | http://localhost:8700 | LAN QR launcher page (scan to jump to other modules) |
| 01_IMU | http://localhost:8765 | 3D attitude + sensor data |
| 02_RTK | http://localhost:8775 | Map + waypoint management |
| 03_Nav | http://localhost:8785 | IMU + RTK combined panel (passthrough, not fusion) |
| 04_Robot | http://localhost:8888 | Robot control + telemetry |
| 05_AutoNav | http://localhost:8805 | Autonomous navigation engine |
| 06_Camera | http://localhost:8815 | OAK-D camera stream |

---

## 5. Recording data

### 5.1 One-command recording of everything (recommended)

```bash
python3 record_all.py
```

Records data from every currently running bridge at once, `Ctrl-C` to
stop, output goes into one timestamped session directory:

```
data_log/session_{timestamp}/
    imu.jsonl            # IMU frames (~50 Hz)
    rtk.jsonl             # RTK fixes (~5 Hz)
    nav.jsonl             # IMU+RTK passthrough frames (~10 Hz)
    robot.jsonl           # Robot telemetry (~20 Hz)
    autonav.jsonl         # Autonomous navigation status (~5 Hz)
    camera_status.jsonl   # Camera status (~1 Hz)
    cam1.mp4               # Camera 1 video (needs camera online)
    cam2.mp4               # Camera 2 video (needs camera online)
```

If a bridge isn't running, that path auto-retries without affecting other
recordings. A per-file size summary prints when you stop.

### 5.2 Recording a single module

| Module | Command | Saved to |
|------|------|---------|
| IMU | `python3 01_IMU/listen_imu_websocket.py` | `01_IMU/data_log/imu_raw_{ts}.jsonl` |
| RTK | `python3 02_RTK/listen_rtk_websocket.py` | `02_RTK/data_log/rtk_raw_{ts}.jsonl` |
| Nav | `python3 03_Nav/listen_nav_websocket.py` | `03_Nav/data_log/nav_raw_{ts}.jsonl` |
| Robot | Click **● REC** at http://localhost:8888, or `python3 04_Robot/listen_robot_websocket.py` | `04_Robot/data_log/run_{ts}.jsonl` / `robot_raw_{ts}.jsonl` |
| AutoNav | `python3 05_AutoNav/listen_autonav.py` | `05_AutoNav/data_log/autonav_raw_{ts}.jsonl` |
| Camera | `python3 06_Camera/listen_camera_websocket.py` | `06_Camera/data_log/camera_raw_{ts}.jsonl` |

---

## 6. Offline replay (developing without hardware)

Use previously recorded JSONL files to simulate a data stream in place of
real hardware. Each `replay_*.py` defaults to the **most recent** `.jsonl`
file in that module's `data_log/` directory (record one first with the
`listen_*.py` scripts above, or drop a sample recording in directly):

```bash
# Replay IMU (occupies port 8766)
python3 01_IMU/replay_imu_websocket.py

# Replay RTK (occupies port 8776)
python3 02_RTK/replay_rtk_websocket.py

# Replay Robot (occupies port 8889; 04_Robot/data_log/robot_raw_v1.jsonl
# ships as a baseline sample with the repo)
python3 04_Robot/replay_robot_websocket.py

# Replay IMU + RTK together (feeds 05_AutoNav; reads the fixed
# 01_IMU/data_log/imu_raw_v1.jsonl and 02_RTK/data_log/rtk_raw_v1.jsonl —
# you need to record/place these two files yourself first)
python3 05_AutoNav/replay_imu_rtk.py
```

**Combine these with the debug-tool menu (`./start_helpers.sh`):**

```
D = 02_RTK replay + 01_IMU replay + 03_Nav bridge + 03_Nav listen
E = 05_AutoNav listen
F = 05_AutoNav replay (replays IMU + RTK together)
```

The menu accepts multiple key combos at once (e.g. `234`), automatically
tiling multiple panes in one tmux window.

---

## 7. Command cheat sheet

```bash
# Start all bridges
./start_bridges.sh

# One-command recording of everything (in another terminal)
python3 record_all.py

# Debug tool menu (listen / replay / send_demo)
./start_helpers.sh

# Start a single module (example)
cd 05_AutoNav && python3 autonav_bridge.py

# Stop all bridges
tmux kill-session -t bridges
```

**Data file locations:**

```
data_log/session_{ts}/                       # record_all.py's unified output directory
01_IMU/data_log/imu_raw_{ts}.jsonl           # single-module IMU recording
02_RTK/data_log/rtk_raw_{ts}.jsonl           # single-module RTK recording
03_Nav/data_log/nav_raw_{ts}.jsonl           # single-module Nav recording
04_Robot/data_log/run_{ts}.jsonl             # single-module Robot recording (needs REC clicked)
05_AutoNav/data_log/autonav_raw_{ts}.jsonl   # single-module AutoNav recording
06_Camera/data_log/camera_raw_{ts}.jsonl     # single-module Camera recording
```

> For detailed architecture, the `common/` framework layer design, and
> each module's JSON contract, see [README.md](README.md) and the
> `README.md` under each module directory.
