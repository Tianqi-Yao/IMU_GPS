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

### 5.1 IMU data

```bash
python 01_IMU/listen_imu_websocket.py
```

Auto-saves while running → `01_IMU/data_log/imu_raw_{timestamp}.jsonl`. Stop with `Ctrl-C`.

### 5.2 RTK data

```bash
python 02_RTK/listen_rtk_websocket.py
```

Auto-saves → `02_RTK/data_log/rtk_raw_{timestamp}.jsonl`.

### 5.3 Robot telemetry

Open http://localhost:8888 and click **● REC** to start recording; click again to stop.

- Saves to `04_Robot/data_log/run_{timestamp}.jsonl`
- Contains: IMU forward, RTK forward, odometry (speed / angular rate), battery

### 5.4 Autonomous navigation status

```bash
python 05_AutoNav/listen_autonav.py
```

Auto-saves → `05_AutoNav/data_log/autonav_raw_{timestamp}.jsonl`, while printing live navigation status to the terminal. Type commands to control navigation: `start` / `stop` / `pause` / `resume`.

### 5.5 Start all listen tools at once

```bash
./start_helpers.sh
```

Type `137` (or `1 3 7`) to launch IMU + RTK + Robot listen in one go:

```
1  01_IMU  listen   → 01_IMU/data_log/imu_raw_{ts}.jsonl
3  02_RTK  listen   → 02_RTK/data_log/rtk_raw_{ts}.jsonl
7  04_Robot listen  → terminal output (use REC button for robot log)
```

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

# Debug tools menu (listen / replay)
./start_helpers.sh

# Start a single module
cd 05_AutoNav && python autonav_bridge.py

# Record IMU + RTK (background)
python 01_IMU/listen_imu_websocket.py &
python 02_RTK/listen_rtk_websocket.py &

# Record autonomous navigation status
python 05_AutoNav/listen_autonav.py

# Stop all bridges
tmux kill-session -t bridges
```

**Data file locations:**

```
01_IMU/data_log/imu_raw_{ts}.jsonl          # IMU frames (~50 Hz)
02_RTK/data_log/rtk_raw_{ts}.jsonl          # RTK position data (~5 Hz)
04_Robot/data_log/run_{ts}.jsonl            # Robot telemetry (requires REC)
05_AutoNav/data_log/autonav_raw_{ts}.jsonl  # Autonomous nav status (~5 Hz)
```

> For detailed parameters, architecture diagrams, and plugin development guides see [README.md](README.md).
