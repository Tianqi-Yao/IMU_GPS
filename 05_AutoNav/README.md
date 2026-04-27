# 05_AutoNav — Autonomous Navigation

Pure Pursuit + PID path tracking module. Reads IMU heading and RTK position, follows waypoints defined in `path.csv`, and sends velocity commands to `04_Robot`.

## File Structure

```
05_AutoNav/
├── autonav_bridge.py   # I/O framework: sensor reading, state machine, waypoint selection, WS/HTTP server
├── autonav_algo.py     # Control algorithm: edit this file to change steering behaviour
├── path.csv            # Waypoint file
├── listen_autonav.py   # Debug tool: prints live navigation status
├── replay_imu_rtk.py   # Offline tool: replays recorded IMU/RTK data without hardware
└── web_static/         # Browser dashboard (opens automatically)
```

## Quick Start

```bash
python autonav_bridge.py
```

A browser tab opens automatically at `http://localhost:8805` showing live navigation status.

Send control commands via the dashboard or WebSocket:

| Command | Effect |
|---------|--------|
| `start` | Begin navigation |
| `stop` | Stop and zero velocity |
| `pause` | Hold in place |
| `resume` | Resume from paused |

## Data Flow

```
imu_bridge  :8766 ──→ ImuWsClient  ─┐
                                     ├─ AutoNavLoop → algo.compute() → joystick cmd
rtk_bridge  :8776 ──→ RtkWsClient  ─┘       │                              │
                                             │                              ↓
path.csv ───────────────────────────────────┘              robot_bridge :8889
                                                                           │
                                              AutoNavWsServer :8806 ◄──────┘
                                              (Dashboard + control commands)
```

## Modifying the Algorithm

**Only edit `autonav_algo.py`** — no need to touch `autonav_bridge.py`.

Parameters at the top of the file:

```python
KP = 0.8          # Steering proportional gain (higher = more aggressive turning)
KI = 0.01         # Integral gain (corrects persistent small offsets)
KD = 0.05         # Derivative gain (dampens oscillation)
LOOKAHEAD_M = 2.0 # Pure Pursuit lookahead distance (higher = smoother path)
REACH_TOL_M = 1.5 # Waypoint arrival radius (m)
MAX_LINEAR  = 1.0 # Max forward speed (m/s)
MAX_ANGULAR = 1.0 # Max angular velocity (rad/s)
```

Core function signature (do not rename):

```python
def compute(heading_deg, target_bearing_deg, dist_to_wp_m, dist_to_final_m, dt_s):
    # heading_deg        : current robot heading, 0=north, clockwise
    # target_bearing_deg : bearing to the lookahead waypoint
    # dist_to_wp_m       : distance to the current waypoint (m)
    # dist_to_final_m    : distance to the last waypoint (m)
    # dt_s               : seconds since last tick
    # returns: (linear m/s, angular rad/s)
    ...
```

## Waypoint File

`path.csv` format (tab or comma separated):

```
id  lat         lon          tolerance_m  max_speed
0   38.94130    -92.31880    0.5          1
1   38.94133    -92.31875    0.5          1
```

| Field | Description |
|-------|-------------|
| `id` | Index, starting from 0 |
| `lat` / `lon` | WGS-84 decimal degrees |
| `tolerance_m` | Arrival radius (m), overrides `REACH_TOL_M` per waypoint |
| `max_speed` | Max speed for this segment (m/s), overrides `MAX_LINEAR` |

## Debugging

**Enable raw data printing** — set in `autonav_algo.py`:

```python
ALGO_DEBUG = True
```

After restart, the terminal prints every tick's raw IMU/RTK dicts, extracted values, and algorithm output:

```
[RAW IMU]   {"type": "imu_frame", "heading": {"deg": 45.2}, ...}
[RAW RTK]   {"type": "rtk_frame", "lat": 38.941, "fix_quality": 1, ...}
[EXTRACTED] heading=45.2 lat=38.941 lon=-92.318 gps_age=0.21s imu_age=0.08s
[CMD OUT]   linear=0.800 angular=0.312 error=+23.1° dist=4.2m wp=1/5
```

**Offline testing without hardware:**

```bash
# Terminal 1: replay recorded IMU + RTK data
python replay_imu_rtk.py

# Terminal 2: start navigation
python autonav_bridge.py
```

**Monitor the output stream:**

```bash
python listen_autonav.py
```

## Ports

| Purpose | Port |
|---------|------|
| HTTP Dashboard | 8805 |
| AutoNav WebSocket (status + control) | 8806 |
| Depends on IMU WS | 8766 |
| Depends on RTK WS | 8776 |
| Depends on Robot WS | 8889 |

## Safety

- **Sensor timeout**: navigation auto-pauses if GPS or IMU data age exceeds `GPS_TIMEOUT_S` (default 5s); auto-resumes when sensors recover
- **GPS fix filter**: frames with `fix_quality == 0` (no fix / default coordinates) do not update GPS age, preventing false "fresh data" readings
- **Watchdog heartbeat**: zero-velocity command sent to robot_bridge every second to prevent runaway on connection loss
