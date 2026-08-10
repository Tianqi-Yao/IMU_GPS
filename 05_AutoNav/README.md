# 05_AutoNav — Autonomous Navigation

*[中文版](README_zh.md)*

Drives the robot along a fixed waypoint path using live IMU heading + RTK
position, controlling 04_Robot over WebSocket. This module has the
project's clearest bridge/algorithm split: **all math lives in
`autonav_algo.py`; `autonav_bridge.py` only does I/O and sequences the
state machine.**

**Read this before assuming the old README's algorithm description:** the
real control law is a proportional (P-only) heading controller with a hard
dead zone. It is **not** a PID (no integral/derivative terms) and **not**
Pure Pursuit (no lookahead point selection — it always aims directly at the
current waypoint). This is a deliberate, validated simplification; the
`config.py` tuning knobs below are what's actually used.

## Architecture

- **INPUT** — `ImuWsClient`/`RtkWsClient` subscribe to 01_IMU/02_RTK;
  `path.csv` waypoint loading; browser WS control messages.
- **CORE** — `autonav_algo.compute()` (geometry + control law, returns a
  `ComputeResult` with `linear/angular/arrived/dist_m/bearing_deg` so the
  bridge never has to recompute distance/bearing itself) and
  `AutoNavLoop` (the idle/running/paused/lifting/arrived state machine —
  waypoint advancement, pause-at-waypoint, and lift-triggering are
  state-machine concerns and live here, not in `autonav_algo.py`).
- **OUTPUT** — `RobotWsClient` (drive/lift commands to 04_Robot);
  `common.ws_server.BroadcastWsServer` (autonav_status to the dashboard);
  `common.http_server.StaticFileServer` (web_static/ + `GET /export_csv`).

## Algorithm (autonav_algo.py, real behavior)

- `fast_distance_m`/`fast_bearing` — equirectangular-projection planar
  geometry (not haversine great-circle), fine at field scale, not for long
  distances. Public API (no leading underscore) — used by both
  `autonav_bridge.py` and `build_waypoints_from_run.py`.
- `compute(lat, lon, heading_deg, waypoints, wp_idx, dt_s, prev_angular=0.0)`
  — heading error -> proportional gain (`AUTONAV_PID_KP`) -> hard dead zone
  (`AUTONAV_DEAD_ZONE_DEG`, below which angular = 0 exactly, not filtered).
  On high-friction ground a bare P command near the dead zone edge is too
  weak to break static friction (the error grows unchecked until a much
  bigger command "breaks free" and overshoots — a field-observed
  stick-slip weave when driving straight), so two feedforward-style terms
  compensate without turning this into a PID or Pure Pursuit controller:
    - `AUTONAV_MIN_ANGULAR_KICK` — deadband/stiction floor: any nonzero
      angular command outside the dead zone is at least this large.
    - `AUTONAV_ANGULAR_SLEW_RATE` — rate-limits the *output* command
      (rad/s², not a derivative of the error) so the kick above doesn't
      itself jump and overshoot; `prev_angular` (this function's own last
      return value, threaded through explicitly by the caller since this
      stays a pure function) is what it slews from.
  Linear velocity: decelerates within `AUTONAV_DECEL_RADIUS_M` of the
  waypoint, and **tapers continuously** (not a hard stop) down to
  `AUTONAV_LINEAR_TURN_FLOOR` as heading error grows past
  `AUTONAV_DEAD_ZONE_DEG` up to `AUTONAV_LINEAR_TAPER_DEG` — turning while
  still rolling only fights kinetic friction, not the static friction of a
  dead-stop pivot turn, which is why turning-while-moving already worked
  better than the old hard turn-in-place cutoff in the field. The taper
  never raises speed above what the waypoint-deceleration logic set.
  Arrival is `dist_m < AUTONAV_REACH_TOL_M`.
- `compute_calibration_offset(cur_lat, cur_lon, mark_lat, mark_lon, heading_raw)`
  — in-field compass calibration: stand at a known point facing a landmark,
  and this returns the north offset to push to 01_IMU.
- `apply_offset_to_waypoints(waypoints, offset_m)` — shifts the whole path
  laterally (miter join at corners) for `set_offset` (e.g. driving a
  parallel lane).
- **Explicitly not reintroduced** (removed in an earlier, intentional
  simplification — do not add back without an explicit product decision):
  integral/derivative terms, moving-average filtering, Pure Pursuit
  lookahead point selection, haversine great-circle distance.

## State machine

`idle -> running <-> paused -> lifting -> arrived`

- **`lifting`**: triggered when the arriving waypoint's `lift` column is
  `up`/`down`; runs the lift actuator for `AUTONAV_LIFT_*` duration seconds,
  then continues to the next waypoint (or `arrived` if it was the last one).
- **Auto-pause**: if `sensors_ok` (IMU+RTK fresh enough, see
  `AUTONAV_GPS_TIMEOUT_S`) or the robot WS link goes down while `running`,
  the loop auto-pauses (`_paused_by_timeout = True`) and auto-resumes once
  both recover — distinct from a user-initiated `pause` command.
- **Fixed bug**: `cmd_resume()` now refuses to resume (returns
  `"sensors_not_ready"`, echoed back to the requesting client as a
  `resume_result` message) if sensors/robot link are still down, instead of
  unconditionally flipping to `running` and immediately being bounced back
  to `paused` by the next control cycle's auto-pause check (a same-cycle
  paused/running flap in the pre-refactor implementation).
- Waypoint arrival can require `AUTONAV_ARRIVE_FRAMES` consecutive
  in-tolerance control cycles before it's confirmed (debounce; default 1 =
  no debounce). Whether the robot stops and waits for user confirmation at
  an intermediate waypoint is controlled by `pause_mode` (`"all"` or
  `"type"`, the latter only pausing at waypoints with `type == "pause"`).

## `config.py` tuning knobs (all real, top-level definitions — none hidden as algorithm-file fallback defaults)

`AUTONAV_PID_KP`, `AUTONAV_DEAD_ZONE_DEG`, `AUTONAV_MIN_ANGULAR_KICK`,
`AUTONAV_ANGULAR_SLEW_RATE`, `AUTONAV_LINEAR_TAPER_DEG`,
`AUTONAV_LINEAR_TURN_FLOOR`, `AUTONAV_DECEL_RADIUS_M`, `AUTONAV_REACH_TOL_M`,
`AUTONAV_MAX_LINEAR_VEL`, `AUTONAV_MIN_LINEAR_VEL`, `AUTONAV_MAX_ANGULAR_VEL`,
`AUTONAV_MANUAL_SPEED`, `AUTONAV_ARRIVE_FRAMES`.

## Output contract (WebSocket, `autonav_status`, ~`AUTONAV_CONTROL_HZ` Hz)

```json
{
  "type": "autonav_status", "version": 1,
  "state": "running", "current_wp_idx": 3, "total_wp": 12,
  "heading_deg": 87.5, "target_bearing_deg": 90.1, "bearing_error_deg": 2.6,
  "dist_to_wp_m": 4.32, "linear": 0.35, "angular": -0.08,
  "gps_age_s": 0.12, "gps_packet_age_s": 0.20, "gps_fix_quality": 4,
  "sensor_block_reason": null, "robot_connected": true, "imu_age_s": 0.05,
  "speed_ratio": 1.0, "manual_speed": 0.4,
  "calib": {"mark": null, "offset_applied": null},
  "waypoints_window": [{"idx": 3, "lat": 0.0, "lon": 0.0, "current": true}],
  "waiting_at_wp": false, "waiting_wp_idx": null, "pause_mode": "all",
  "lift_cmd": "", "lift_remaining_s": 0.0, "lift_up_s": 5.0, "lift_down_s": 5.0,
  "offset_m": 0.0,
  "path_original": [{"lat": 0.0, "lon": 0.0}], "path_offset": [{"lat": 0.0, "lon": 0.0}],
  "robot_lat": 0.0, "robot_lon": 0.0,
  "imu_raw": {"...": "full imu_frame"}, "rtk_raw": {"...": "full rtk_frame"}
}
```

## Control messages (browser -> bridge)

`start`, `restart`, `stop`, `pause`, `resume` (may reply `resume_result`),
`set_speed{ratio}`, `load_csv{content}`, `gen_path{lat,lon}`, `calib_mark`,
`calib_apply`, `manual_drive{linear}`, `confirm_wp`, `set_pause_mode{mode}`,
`lift_control{cmd}`, `set_lift_duration{up_s,down_s}`, `set_offset{offset_m}`.

**Note**: `pause`/`resume`/`resume_result` are fully implemented in
`autonav_bridge.py`, but the bundled `web_static/app.js` dashboard does not
expose any button for them and ignores `resume_result` messages — today
the only way `paused` is entered/exited is the automatic sensor/robot-link
timeout path (see "Auto-pause" above). Sending `pause`/`resume` over the
WebSocket directly (e.g. from a custom client) works as documented; there
is just no UI affordance for it yet.

## Waypoint file (`path.csv`)

Only `lat`/`lon`/`type`/`lift` columns are consumed (`type` is currently
either `""` or `"pause"`; `lift` is `""`/`"up"`/`"down"`). Other columns
(`st lat`, `st lon`, `file`, `row num`, ...) are intermediate fields left
over from `scripts/convert_offsets_to_latlon.py`'s conversion process and
are ignored by the loader.

## Record / replay

- `listen_autonav.py` — logs to `data_log/autonav_raw_{timestamp}.jsonl`.
- `replay_imu_rtk.py` — serves fixed IMU+RTK baseline files on 01_IMU's/
  02_RTK's normal ports, for developing this bridge without hardware. Point
  `IMU_JSONL`/`RTK_JSONL` at a matched recording pair before using it.
- `build_waypoints_from_run.py` — turns a recorded manual drive
  (04_Robot's `data_log/run_*.jsonl`) into a downsampled waypoint CSV.

## Run

```bash
python autonav_bridge.py
```

Requires 01_IMU, 02_RTK, and 04_Robot (real or replayed) already running at
the URLs in `config.py` (`AUTONAV_IMU_WS`/`AUTONAV_RTK_WS`/`AUTONAV_ROBOT_WS`).
HTTP page on `AUTONAV_WS_PORT` (default 8805), WebSocket on
`AUTONAV_WS_PORT + 1` (default 8806).
