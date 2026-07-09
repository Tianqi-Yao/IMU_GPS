# 05_AutoNav — I/O Workflow

Autonomous navigation module: follows a GPS waypoint path using Pure Pursuit + PID control.

## Architecture

```
INPUT   ImuWsClient      ws://localhost:8766  →  heading_deg
        RtkWsClient      ws://localhost:8776  →  lat, lon
        PathLoader       path.json           →  list[Waypoint]

CORE    GeoMath          haversine, bearing, normalize_angle
        PIDController    angular velocity PID (anti-windup)
        MovingAverage    angular output smoothing
        PurePursuitPlanner  lookahead waypoint selection
        AutoNavController   state machine + velocity computation

OUTPUT  RobotWsClient    ws://localhost:8889  →  joystick JSON
        AutoNavWsServer  ws://localhost:8806  →  autonav_status broadcast
        HttpFileServer   http://localhost:8805
```

## Running

```bash
# 1. Start autonav_bridge
python 05_AutoNav/autonav_bridge.py

# 2. Observe status (in another terminal)
python 05_AutoNav/listen_autonav.py

# 3. Send start command (in listen_autonav terminal, type and press Enter)
start
```

## Offline testing (no hardware)

```bash
# 1. Replay recorded IMU + RTK data
python 05_AutoNav/replay_imu_rtk.py

# 2. Start autonav_bridge (in another terminal)
python 05_AutoNav/autonav_bridge.py

# 3. Observe
python 05_AutoNav/listen_autonav.py
```

## Input contracts

### IMU frame — ws://localhost:8766
```json
{
  "type": "imu_frame",
  "heading": {
    "deg": 135.4,
    "dir": "SE"
  }
}
```
Used field: `heading.deg` (0–360, true north, north_offset applied)

### RTK frame — ws://localhost:8776
```json
{
  "type": "rtk_frame",
  "lat": 38.9412,
  "lon": -92.3188,
  "fix_quality": 4,
  "num_sats": 12
}
```
Used fields: `lat`, `lon`

### path.csv
```
lat,lon,type,lift
38.941083678333,-92.318317141667,alley,
38.941095347133,-92.318317141667,pause,
```
Columns (tab or comma separated, auto-detected; parsed in `autonav_bridge._load_default_waypoints` / `_convert_csv_to_waypoints`):
- `lat`: latitude (decimal degrees, +N) — **required**
- `lon`: longitude (decimal degrees, +E) — **required**
- `type`: free-text label; the value `pause` forces a stop at that waypoint — optional
- `lift`: `up` / `down` triggers the lift actuator (`LIFT_UP_DURATION_S` / `LIFT_DOWN_DURATION_S`) — optional
- Extra columns are ignored; only `lat`/`lon`/`type`/`lift` are read.

The file can be replaced at runtime via the dashboard's "LOAD CSV" upload, which parses the
same schema through `_convert_csv_to_waypoints`.

### Generating path.csv from a manual run

No hardware GPS survey step exists to get precise field coordinates ahead of time, so
`path.csv` is instead produced by driving the planned route once and converting the
recording:

1. Manually drive the route via the `04_Robot` web UI, pressing **REC** before starting
   and stopping it at the end — this writes `04_Robot/data_log/run_<timestamp>.jsonl`
   (RTK/IMU/odom samples; the `Recorder` class already existed for this purpose).
2. Run `python 05_AutoNav/build_waypoints_from_run.py` — it picks the newest `run_*.jsonl`
   by default, filters RTK samples by `WAYPOINT_MIN_FIX_QUALITY`, downsamples them so
   consecutive waypoints are at least `WAYPOINT_MIN_DISTANCE_M` apart (both in `config.py`),
   and writes `05_AutoNav/data_log/path_<run_ts>_<density>m.csv` in the schema above.
3. Adjust `WAYPOINT_MIN_DISTANCE_M` to control waypoint density and re-run step 2 — this
   regenerates the path from the same recording, no re-drive needed.
4. Load the generated CSV via the dashboard "LOAD CSV" button, or copy/rename it to
   `path.csv` and restart `autonav_bridge.py`.

## Output contracts

### → robot_bridge — ws://localhost:8889
```json
{"type": "joystick", "linear": 0.30, "angular": -0.12, "force": 1.0}
```
- `linear`: m/s, range [0.0, MAX_LINEAR_VEL]
- `angular`: rad/s, range [-MAX_ANGULAR_VEL, +MAX_ANGULAR_VEL]
- `force`: always 1.0

### Status broadcast — ws://localhost:8806
```json
{
  "type": "autonav_status",
  "version": 1,
  "state": "running",
  "current_wp_idx": 2,
  "total_wp": 5,
  "dist_to_wp_m": 1.23,
  "target_bearing_deg": 45.0,
  "heading_deg": 42.1,
  "bearing_error_deg": 2.9,
  "linear": 0.30,
  "angular": -0.05,
  "gps_age_s": 0.12,
  "imu_age_s": 0.08,
  "sensor_block_reason": null,
  "robot_connected": true
}
```
- `sensor_block_reason`: `null` unless `state == "paused"` by an automatic condition — one of
  `rtk_fix_quality=<n>`, `rtk_packet_age=<s>s`, `rtk_fix_age=<s>s`, `imu_age=<s>s`,
  `sensor_unavailable`, or `robot_disconnected`.
- `robot_connected`: whether the `robot_bridge` (:8889) WebSocket link is currently up. `false`
  while `sensor_block_reason == "robot_disconnected"`.

### Control commands → ws://localhost:8806
```json
{"type": "start"}    // idle → running (resumes at current_wp_idx unless arrived/out of range)
{"type": "restart"}  // idle | paused | arrived → running, forces current_wp_idx = 0
{"type": "stop"}     // any  → idle (current_wp_idx preserved)
{"type": "pause"}    // running → paused   (kept for scripted/debug clients; no dashboard button)
{"type": "resume"}   // paused → running   (kept for scripted/debug clients; no dashboard button)
```

## State machine

```
idle ──start()────────────────► running ──GPS/IMU timeout─────────► paused ──resume()──► running
     ◄──stop()──                        ──robot link disconnected─►        ──sensors/link recover (auto)──► running
     ◄──restart() [idle|paused|arrived]  ──all WP reached──────────────────────────────► arrived
                                         ──waypoint has lift=up/down─► lifting ──duration elapsed──► running | arrived | waiting-for-confirm
```

- `paused` by timeout or robot-link loss: auto-resumes when the condition clears; reason exposed via `sensor_block_reason`
- `paused` by command (`pause`/`resume`): requires explicit `resume`; only reachable via the WS command, not from the dashboard
- Entering `paused` or `arrived`: immediately sends linear=0, angular=0
- `restart()` is a no-op (logged, ignored) while `state == "running"` — stop first, or wait for it to arrive/auto-pause

## Algorithm

**Pure Pursuit (simplified):**
1. Advance `current_idx` when distance < `REACH_TOLERANCE_M` for `ARRIVE_FRAMES` consecutive ticks
2. Select first waypoint at distance ≥ `LOOKAHEAD_M` as steering target
3. Fallback to last waypoint if all remaining are within lookahead

**Steering:**
```
target_bearing = bearing(current_pos, target_wp)
error          = normalize(target_bearing - heading_deg)  # (-180, 180]
angular        = PID(error) → MovingAverage
```

**Deceleration (near final waypoint):**
```
if dist_to_final < DECEL_RADIUS_M:
    linear = max(0.10, MAX_LINEAR * dist_to_final / DECEL_RADIUS_M)
```

## Key parameters (config.py)

| Parameter | Default | Description |
|---|---|---|
| `AUTONAV_MAX_LINEAR_VEL` | 1.0 m/s | Maximum linear speed |
| `AUTONAV_MAX_ANGULAR_VEL` | 1.0 rad/s | Maximum turn rate |
| `AUTONAV_LOOKAHEAD_M` | 2.0 m | Pure Pursuit lookahead distance |
| `AUTONAV_DECEL_RADIUS_M` | 3.0 m | Deceleration zone near goal |
| `AUTONAV_REACH_TOLERANCE_M` | 1.5 m | (bridge constant) Waypoint arrival radius |
| `AUTONAV_PID_KP/KI/KD` | 0.8/0.01/0.05 | PID gains |
| `AUTONAV_GPS_TIMEOUT_S` | 5.0 s | Sensor timeout → pause |
| `AUTONAV_MA_WINDOW` | 10 | Angular smoothing window |

## Field calibration

If the robot turns in the wrong direction, change `ANGULAR_SIGN = -1` in `autonav_bridge.py` (line ~65).

## Observation boundaries

- **Input (IMU):** `ImuWsClient.run()` — async for loop in `autonav_bridge.py`
- **Input (RTK):** `RtkWsClient.run()` — async for loop in `autonav_bridge.py`
- **Output (Robot):** `RobotWsClient.run()` — `ws.send(msg)` in `autonav_bridge.py`
- **Status log:** `data_log/autonav_raw_<timestamp>.jsonl` (written by `listen_autonav.py`)
