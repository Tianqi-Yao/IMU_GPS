# IMU_GPS v2

A robot perception + autonomous-navigation stack for a Raspberry Pi:
IMU (attitude) and RTK GPS (precision position) feed a real-time WebSocket
dashboard; an autonomous navigation module drives a chassis (Feather M4 +
CAN bus to an Amiga VCU); an OAK-D depth camera module streams video
through a hot-swappable image-processing plugin pipeline; a QR launcher
page gives phones on the LAN one-tap access to every module.

This is a from-scratch rewrite following `doc/REFACTOR_PROMPT.md`'s
architecture review of the original project — same black-box contracts,
cleaned-up internals. **This directory is fully self-contained: no code
here imports anything from outside `v2/`.**

## Black-box architecture (read this first)

The system is **7 independent bridge processes**. Each can be started,
debugged, and replaced on its own, as long as its external contract
(WebSocket/HTTP/serial) stays the same:

- Modules never import each other's business code — only `common/`
  (zero-business-logic framework code: WS broadcast, static HTTP serving,
  logging, port derivation) is shared.
- Every bridge is internally layered INPUT (serial/WS/file reads) / CORE
  (parsing, algorithms, state machines — no socket/serial handles, unit
  testable standalone) / OUTPUT (WS broadcast, HTTP responses, file writes).
- Every module ships a `listen_*.py` (observe/record real traffic) and a
  `replay_*.py` (simulate that module's data source from a recording) so
  downstream development never strictly requires the upstream hardware.

## Module / port map

```
module      HTTP port   WS port (HTTP+1)   hardware input
00_QR       8700        (no WS — static page only)
01_IMU      8765        8766                BNO085 (serial)
02_RTK      8775        8776                RTK GPS (serial, NMEA)
03_Nav      8785        8786                (subscribes to 01/02's WS)
04_Robot    8888        8889                Feather M4 (serial)
05_AutoNav  8805        8806                (subscribes to 01/02's WS, drives 04's WS)
06_Camera   8815        8816                OAK-D (USB)
```

Convention: every bridge's HTTP port serves its `web_static/` dashboard;
its WebSocket port is always `HTTP_PORT + 1` (`common/ports.py::derive_ws_port`).

## `common/` — shared framework layer

```
common/
  ws_server.py      BroadcastWsServer: client-set management, broadcast(),
                     inbound-message dispatch via an on_client_message callback
  http_server.py     StaticFileServer: threaded static file serving, with
                     narrow extension hooks (index.html rewriting, extra routes)
  logging_setup.py   setup_logger(name, logfile): stdout + .log file, once
  ports.py           derive_ws_port(http_port) -> http_port + 1
```

`common/` never imports from `00_QR`..`06_Camera`; dependency direction is
one-way. No coordinate transforms, PID gains, NMEA parsing, or any other
module-specific logic is allowed to live here — if you're tempted to put
something module-specific in `common/`, it belongs in that module instead.

## Per-module docs

Each module's `README.md` documents its real (not aspirational) WebSocket/
serial contract with a JSON example, its control messages, and its
record/replay workflow: [00_QR](00_QR/) (no README needed — static page,
see `qr_server.py`), [01_IMU](01_IMU/README.md), [02_RTK](02_RTK/README.md),
[03_Nav](03_Nav/README.md), [04_Robot](04_Robot/README.md),
[05_AutoNav](05_AutoNav/README.md), [06_Camera](06_Camera/README.md).

## Run

```bash
# Start all 7 bridges, each in its own tmux window
./start_bridges.sh

# Start ad-hoc listen_*/replay_*/send_demo helper tools via a menu
./start_helpers.sh

# Record every running bridge's WS output (+ both camera MJPEG streams)
# into one timestamped session directory
python3 record_all.py
```

## Configuration

All tunable parameters live in the repo-root `config.py`, grouped by module
number. Edit the file and restart the relevant bridge to apply changes — no
CLI arguments are added by default. Each consumer module defines its own
upstream WebSocket URL variables (e.g. `ROBOT_IMU_WS`/`ROBOT_RTK_WS`) rather
than borrowing another module's, so changing one module's upstream address
never accidentally affects another.

## Record / replay workflow

1. Run the real bridge (with hardware attached) alongside its `listen_*.py`
   to capture a `data_log/*.jsonl` baseline sample.
2. Commit that sample if it's broadly useful for offline development.
3. Anyone downstream can run `replay_*.py` (which defaults to the most
   recently recorded file in `data_log/`) to get the same WebSocket
   contract without the hardware attached.

## Firmware

`CIRCUITPY/` (Feather M4, drives the Amiga VCU over CAN — see
`04_Robot/README.md` for the serial protocol) and
`01_IMU/bno085_esp32c3/bno085_esp32c3.ino` (BNO085 IMU firmware — see
`01_IMU/README.md` for the wire protocol) are carried over unchanged; both
protocols were already verified correct and are out of scope for this
rewrite.
