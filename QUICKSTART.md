# Quick Start

## 1. Prerequisites

```bash
pip install pyserial websockets depthai opencv-python numpy
```

- Python 3.10+
- tmux: `brew install tmux` (Mac) / `apt install tmux` (Linux)

---

## 2. Configure Serial Ports

Open `config.py` and update three serial port paths (other parameters rarely need changing):

| Parameter | Hardware | Mac example | Linux example |
|-----------|----------|-------------|---------------|
| `IMU_SERIAL_PORT` | BNO085 (ESP32-C3) | `/dev/cu.usbmodem1201` | `/dev/ttyACM0` |
| `RTK_SERIAL_PORT` | RTK GPS receiver | `/dev/cu.usbmodem1103` | `/dev/ttyACM1` |
| `ROBOT_SERIAL_PORT` | Feather M4 | `/dev/cu.usbmodem1301` | `/dev/ttyACM2` |

Find available ports:

```bash
ls /dev/cu.*          # Mac
ls /dev/ttyACM*       # Linux
```

---

## 3. Start All Bridges

```bash
./start_bridges.sh
```

Launches 6 modules in a single tmux session (`bridges`), each in its own window:

```
01_IMU     → python 01_IMU/imu_bridge.py
02_RTK     → python 02_RTK/rtk_bridge.py
03_Nav     → python 03_Nav/nav_bridge.py
04_Robot   → python 04_Robot/robot_bridge.py
05_AutoNav → python 05_AutoNav/autonav_bridge.py
06_Camera  → python 06_Camera/camera_bridge.py
```

**tmux cheatsheet:**

| Action | Shortcut |
|--------|----------|
| Switch window | `Ctrl-B` then a number (`1`–`6`) |
| Detach (background) | `Ctrl-B D` |
| Reattach | `tmux attach -t bridges` |
| Kill all | `tmux kill-session -t bridges` |

---

## 4. Open Browser UIs

| Module | URL | Description |
|--------|-----|-------------|
| 01_IMU | http://localhost:8765 | 3D attitude + sensor data |
| 02_RTK | http://localhost:8775 | Map + waypoint management |
| 03_Nav | http://localhost:8785 | Fused navigation dashboard |
| 04_Robot | http://localhost:8888 | Robot control + telemetry |
| 05_AutoNav | http://localhost:8805 | Autonomous navigation engine |
| 06_Camera | http://localhost:8815 | OAK-D camera stream |

---

## 5. Recording Data

### 5.1 Record everything at once (recommended)

```bash
python record_all.py
```

Records all modules simultaneously. Press `Ctrl-C` to stop. Output goes to a single timestamped session directory:

```
data_log/session_{timestamp}/
    imu.jsonl            # IMU frames        ~50 Hz
    rtk.jsonl            # RTK position       ~5 Hz
    nav.jsonl            # Fused nav         ~10 Hz
    robot.jsonl          # Robot telemetry   ~20 Hz
    autonav.jsonl        # AutoNav status     ~5 Hz
    camera_status.jsonl  # Camera status      ~1 Hz
    cam1.mp4             # Camera 1 video    (if camera running)
    cam2.mp4             # Camera 2 video    (if camera running)
```

If a bridge is not running, that stream is skipped silently. A file size summary is printed on exit.

### 5.2 Record a single module

| Module | Command | Output |
|--------|---------|--------|
| IMU | `python 01_IMU/listen_imu_websocket.py` | `01_IMU/data_log/imu_raw_{ts}.jsonl` |
| RTK | `python 02_RTK/listen_rtk_websocket.py` | `02_RTK/data_log/rtk_raw_{ts}.jsonl` |
| Robot | Open http://localhost:8888, click **● REC** | `04_Robot/data_log/run_{ts}.jsonl` |
| AutoNav | `python 05_AutoNav/listen_autonav.py` | `05_AutoNav/data_log/autonav_raw_{ts}.jsonl` |

---

## 6. Offline Replay (no hardware)

Replay recorded JSONL files as live data streams to replace real hardware:

```bash
# Replay IMU (serves on port 8766)
python 01_IMU/replay_imu_websocket.py

# Replay RTK (serves on port 8776)
python 02_RTK/replay_rtk_websocket.py

# Replay IMU + RTK together (feeds 05_AutoNav)
python 05_AutoNav/replay_imu_rtk.py
```

To use a different recording file: open the corresponding `replay_*.py` and update the `INPUT_PATH` variable at the top.

**Full offline nav stack (helpers menu option `D`):**

```
D = IMU replay + RTK replay + Nav bridge + Nav listen
```

---

## 7. Command Reference

```bash
# Start all bridges
./start_bridges.sh

# Record everything at once (separate terminal)
python record_all.py

# Debug tools menu (listen / replay)
./start_helpers.sh

# Start a single module
cd 05_AutoNav && python autonav_bridge.py

# Stop all bridges
tmux kill-session -t bridges
```

**Data file locations:**

```
data_log/session_{ts}/                       # record_all.py unified output
01_IMU/data_log/imu_raw_{ts}.jsonl          # single-module IMU recording
02_RTK/data_log/rtk_raw_{ts}.jsonl          # single-module RTK recording
04_Robot/data_log/run_{ts}.jsonl            # single-module Robot (requires REC)
05_AutoNav/data_log/autonav_raw_{ts}.jsonl  # single-module AutoNav recording
```

> For detailed parameters, architecture diagrams, and plugin development guides see [README.md](README.md).
