# 01_IMU — BNO085 Attitude Bridge

Reads orientation data from a BNO085 IMU (via an ESP32C3 running
`bno085_esp32c3/bno085_esp32c3.ino`) over USB serial, and broadcasts it as
JSON over WebSocket for any downstream consumer (browser dashboard,
03_Nav, 05_AutoNav).

## Architecture

- **INPUT** — `SerialReader._read_loop`: reads newline-delimited compact-JSON
  lines from the serial port in a dedicated daemon thread, retries the
  serial connection every 3s if the device disappears.
- **CORE** — `IMUPipeline.process`: `_parse` -> `_enrich_euler` ->
  `_enrich_heading` -> `_enrich_hz`. Pure functions on plain dicts/dataclasses,
  no socket or serial state — can be unit tested with recorded strings alone.
- **OUTPUT** — `common.ws_server.BroadcastWsServer` broadcasts `imu_frame`
  payloads; `common.http_server.StaticFileServer` serves `web_static/`.

## Wire protocol: firmware -> bridge (serial)

The ESP32C3 firmware emits a **compact, positionally-encoded** JSON line to
fit the 256-byte USB CDC-ACM buffer:

```json
{"t":123456,"r":[qi,qj,qk,qr,acc],"g":[qi,qj,qk,qr],"a":[x,y,z],"l":[x,y,z],"v":[x,y,z],"w":[x,y,z],"m":[x,y,z],"s":0,"c":3}
```

| key | meaning | expands to |
|---|---|---|
| `t` | timestamp (ms) | `ts` |
| `r` | rotation vector quaternion + accuracy | `rot` `{qi,qj,qk,qr,acc}` |
| `g` | game rotation vector (no magnetometer) | `game_rot` `{qi,qj,qk,qr}` |
| `a` | accelerometer | `accel` `{x,y,z}` |
| `l` | linear acceleration | `lin_accel` `{x,y,z}` |
| `v` | gravity vector | `gravity` `{x,y,z}` |
| `w` | gyroscope | `gyro` `{x,y,z}` |
| `m` | magnetometer | `mag` `{x,y,z}` |
| `s` | step counter | `steps` |
| `c` | calibration status (0-3) | `cal` |

This is an implicit contract between the firmware and the bridge based on
**array position, not key names** — if either side changes the array order
independently, values silently misalign. Lines starting with `#` are
firmware debug comments and are skipped by the parser.

## Output contract (WebSocket, `imu_frame`)

```json
{
  "type": "imu_frame", "version": 1,
  "rot": {"qi": 0.0, "qj": 0.0, "qk": 0.0, "qr": 1.0},
  "euler": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "north_offset_deg": 0.0},
  "heading": {"raw": 0.0, "deg": 0.0, "dir": "N"},
  "hz": 50.0,
  "accel": {"x": 0.0, "y": 0.0, "z": 9.8},
  "lin_accel": {"x": 0.0, "y": 0.0, "z": 0.0},
  "gravity": {"x": 0.0, "y": 0.0, "z": 9.8},
  "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
  "mag": {"x": 0.0, "y": 0.0, "z": 0.0},
  "cal": 3, "steps": 0, "ts": 123456
}
```

- `euler.yaw` is the raw compass-agnostic yaw derived directly from the
  rotation-vector quaternion (server convention: `corrected = raw + north_offset_deg`).
- `heading.deg`/`heading.dir` are a separate, web-aligned compass heading
  computed from `game_rot` (falls back to `rot`), with the same BNO085
  Z-up -> Three.js Y-up frame correction the frontend applies
  (`frame_correction = (-sqrt(0.5), 0, 0, sqrt(0.5))`), so backend and
  frontend headings stay numerically consistent.
- **Frontend sign convention differs from the backend**: the browser displays
  `display = raw - north_offset_deg`, while the server computes
  `corrected = raw + north_offset_deg`. When syncing offsets in either
  direction, convert with `frontend_offset = (360 - server_offset) % 360`.

## Control message (browser -> bridge)

```json
{"set_north_offset": 42.5}
```

Sets the server-side `north_offset_deg` used for all subsequent heading/yaw
calculations (in-field compass calibration). Not broadcast back; applies
only to internal pipeline state.

## Record / replay

- `listen_imu_websocket.py` — connects to the live WS, prints a table, and
  logs every frame (with `log_recv_ts`/`log_recv_iso` added) to
  `data_log/imu_raw_{timestamp}.jsonl`.
- `replay_imu_websocket.py` — broadcasts the most recently recorded
  `data_log/*.jsonl` file on the same WebSocket port, at a fixed 50 Hz, so
  downstream modules can be developed without a physical IMU. `data_log/`
  starts empty in this repo — run the bridge (or another replay source)
  with `listen_imu_websocket.py` once to produce a baseline sample.

## Run

```bash
python imu_bridge.py
```

Configure `IMU_SERIAL_PORT` / `IMU_BAUD` / `IMU_WS_PORT` / `IMU_NORTH_OFFSET`
in the repo-root `config.py`. HTTP page on `IMU_WS_PORT` (default 8765),
WebSocket on `IMU_WS_PORT + 1` (default 8766).
