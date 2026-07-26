# 03_Nav — IMU + RTK Passthrough

**Read this before assuming "Nav" means fusion.** Despite subscribing to
both 01_IMU and 02_RTK, this bridge does **not** perform any sensor fusion,
coordinate-frame unification, filtering, or arrival/heading computation. It
re-broadcasts each upstream's latest frame, unmodified, nested under `imu`/
`rtk` keys in a single envelope. All of that computation currently happens
client-side in the browser dashboard's JavaScript, not here.

If you need real fusion (e.g. combined position+heading estimate, waypoint
distance), it does not exist yet in this module — either add it here (this
is the natural place for it) or use 05_AutoNav, which does its own
independent geometry/control computation directly from the IMU/RTK streams.

## Architecture

- **INPUT** — `ImuWsClient` / `RtkWsClient`: subscribe to 01_IMU / 02_RTK
  WebSockets, each holding only the most recently received frame (3s
  reconnect on disconnect).
- **CORE** — `BroadcastLoop`: packs the two latest frames into a `nav_frame`
  envelope on its own fixed timer (`NAV_HZ`), decoupled from either
  upstream's actual arrival rate.
- **OUTPUT** — `common.ws_server.BroadcastWsServer` broadcasts `nav_frame`;
  `common.http_server.StaticFileServer` serves `web_static/` (Three.js IMU
  view + Leaflet map, both fed from the same WebSocket).

## Output contract (WebSocket, `nav_frame`, ~`NAV_HZ` Hz)

```json
{
  "type": "nav_frame", "version": 1,
  "imu": { "...": "full imu_frame payload from 01_IMU, unmodified" },
  "rtk": { "...": "full rtk_frame payload from 02_RTK, unmodified" }
}
```

There is no `nav` sub-object — no derived distance/heading/fusion fields.
Consumers must read IMU fields under `imu.*` and RTK fields under `rtk.*`.

## Control message (browser -> bridge)

```json
{"set_north_offset": 42.5}
```

Forwarded verbatim to 01_IMU (see `01_IMU/README.md`); this bridge does not
interpret or store it itself.

## Record / replay

- `listen_nav_websocket.py` — logs frames to `data_log/nav_raw_{timestamp}.jsonl`.
- `replay_nav_websocket.py` — broadcasts the most recently recorded
  `data_log/*.jsonl` file at a fixed 10 Hz. `data_log/` starts empty; run
  `listen_nav_websocket.py` against a live (or replayed) bridge first to
  produce a baseline sample.

## Run

```bash
python nav_bridge.py
```

Requires 01_IMU and 02_RTK (real or replayed) to already be running at the
URLs configured by `NAV_IMU_WS`/`NAV_RTK_WS` in the repo-root `config.py`.
HTTP page on `NAV_WS_PORT` (default 8785), WebSocket on `NAV_WS_PORT + 1`
(default 8786).
