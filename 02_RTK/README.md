# 02_RTK — RTK GPS Bridge

*[中文版](README_zh.md)*

Reads NMEA 0183 sentences (GGA/RMC) from an RTK GPS receiver over serial and
broadcasts a single-source position fix as JSON over WebSocket.

## Architecture

- **INPUT** — `SerialReader._run`: reads raw NMEA lines from the serial port
  in a daemon thread, retries the connection every 3s on error.
- **CORE** — `NMEAPipeline`: verifies each sentence's XOR checksum, parses
  GGA (position/fix/sats/hdop/alt) and RMC (speed/track, only when status is
  `A`=active) into a shared, lock-protected `RTKFrame`, and exposes
  `snapshot()` for a lock-free read of the latest state. **Broadcast rate is
  decoupled from NMEA arrival rate**: `BroadcastLoop` polls `snapshot()` on
  its own fixed-Hz timer rather than pushing a message per incoming sentence.
- **OUTPUT** — `common.ws_server.BroadcastWsServer` broadcasts `rtk_frame`
  payloads; `common.http_server.StaticFileServer` serves `web_static/`
  (Leaflet-based map UI, offline tile fallback under `web_static/assets/tiles/`).

## Output contract (WebSocket, `rtk_frame`, ~`RTK_HZ` Hz)

```json
{
  "type": "rtk_frame", "version": 1,
  "lat": 38.9412928598587, "lon": -92.31884600793728, "alt": 235.2,
  "fix_quality": 4, "num_sats": 14, "hdop": 1.2,
  "speed_knots": 0.04, "track_deg": 287.92,
  "rtk_ts": 1775526995.6776588, "server_ts": 1775526996.288904,
  "source": "rtk"
}
```

| field | meaning |
|---|---|
| `lat`/`lon` | WGS-84 decimal degrees (+N/-S, +E/-W) |
| `alt` | metres MSL |
| `fix_quality` | 0=No fix, 1=GPS, 2=DGPS, 4=RTK Fixed (~1cm), 5=RTK Float (~10cm) |
| `num_sats` | satellites used |
| `hdop` | horizontal dilution of precision |
| `speed_knots`/`track_deg` | from RMC; `null` when RMC status is void (no fix) |
| `rtk_ts` | bridge-side `time.time()` at the last GGA update |
| `server_ts` | bridge-side `time.time()` when this frame was serialized |
| `source` | `"rtk"` = real fix, `"default"` = fallback coords (`RTK_DEFAULT_LAT`/`LON` in config.py) when no fix is available yet |

**This is a single-source contract.** An earlier prototype explored dual-
antenna heading measurement (`rtk_source_frames`, `heading_deg`,
`heading_baseline_m`, ...); that feature was removed from the backend, and
the corresponding dead UI/parsing code has been removed from
`rtk_visualizer.js` and `listen_rtk_websocket.py` in this rewrite. Do not
resurrect these fields without an explicit product decision to bring back
dual-antenna measurement.

## Record / replay

- `listen_rtk_websocket.py` — logs frames to `data_log/rtk_raw_{timestamp}.jsonl`.
- `replay_rtk_websocket.py` — broadcasts the most recently recorded
  `data_log/*.jsonl` file at a fixed 5 Hz. `data_log/` starts empty in this
  repo: the previous repo's `rtk_raw_v1.jsonl`/`v2.jsonl` samples used the
  removed multi-antenna schema and are intentionally **not** carried over —
  record a fresh sample against this bridge before replaying.

## Other tools

- `script/sim_RTK_serial.py` — simulates a slowly-drifting RTK-fixed GPS
  over a virtual serial pair (see the file's `socat` setup comment), for
  testing without hardware.
- `script/download_map.py` — downloads offline OpenStreetMap tiles into
  `web_static/assets/tiles/` for the Leaflet map's offline fallback layer.

## Run

```bash
python rtk_bridge.py
```

Configure `RTK_SERIAL_PORT` / `RTK_BAUD` / `RTK_WS_PORT` / `RTK_HZ` /
`RTK_DEFAULT_LAT` / `RTK_DEFAULT_LON` in the repo-root `config.py`. HTTP page
on `RTK_WS_PORT` (default 8775), WebSocket on `RTK_WS_PORT + 1` (default 8776).
