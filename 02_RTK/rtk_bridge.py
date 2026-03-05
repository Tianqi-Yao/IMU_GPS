"""
rtk_bridge.py - RTK serial reader to WebSocket bridge + static web map UI.

Runs:
  - RTKReader serial thread (NMEA parser in rtk_reader.py)
  - HTTP server for web_static/
  - WebSocket broadcast of latest RTK snapshot

If RTK has no valid lat/lon, it falls back to DEFAULT_LAT/LON.
"""

import argparse
import asyncio
import json
import logging
import socketserver
import threading
import time
import webbrowser
from http.server import SimpleHTTPRequestHandler
from pathlib import Path

import websockets

from rtk_reader import RTKReader

DEFAULT_LAT = 38.9412928598587
DEFAULT_LON = -92.31884600793728

_py_name = Path(__file__).stem
OUTPUT_PATH = Path(__file__).parent
log_file_name = OUTPUT_PATH / f"{_py_name}.log"

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.FileHandler(log_file_name, encoding="utf-8"),
        logging.StreamHandler(),
    ],
)
logger = logging.getLogger(__name__)

connected_clients: set = set()


def normalize_frame(raw: dict) -> dict:
    """Normalize RTK reader snapshot and ensure a valid position exists."""
    lat = raw.get("lat")
    lon = raw.get("lon")
    use_default = lat is None or lon is None
    if use_default:
        lat = DEFAULT_LAT
        lon = DEFAULT_LON

    return {
        "lat": lat,
        "lon": lon,
        "alt": raw.get("alt"),
        "fix_quality": raw.get("fix_quality", 0),
        "num_sats": raw.get("num_sats", 0),
        "hdop": raw.get("hdop"),
        "speed_knots": raw.get("speed_knots"),
        "track_deg": raw.get("track_deg"),
        "rtk_ts": raw.get("ts", 0.0),
        "server_ts": time.time(),
        "source": "default" if use_default else "rtk",
    }


async def ws_handler(websocket) -> None:
    addr = websocket.remote_address
    logger.info(f"WebSocket client connected: {addr}")
    connected_clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        connected_clients.discard(websocket)
        logger.info(f"WebSocket client disconnected: {addr}")


async def broadcast_loop(reader: RTKReader, hz: float) -> None:
    """Poll RTKReader at fixed rate and broadcast latest snapshot."""
    period = 1.0 / max(hz, 0.5)
    while True:
        frame = normalize_frame(reader.get_data())
        if connected_clients:
            msg = json.dumps(frame)
            dead = set()
            for ws in connected_clients.copy():
                try:
                    await ws.send(msg)
                except Exception:
                    dead.add(ws)
            connected_clients.difference_update(dead)
        await asyncio.sleep(period)


def start_http_server(static_dir: Path, http_port: int) -> None:
    class Handler(SimpleHTTPRequestHandler):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, directory=str(static_dir), **kwargs)

        def log_message(self, fmt, *args):
            logger.debug("HTTP %s - %s", self.address_string(), fmt % args)

    socketserver.TCPServer.allow_reuse_address = True
    with socketserver.TCPServer(("", http_port), Handler) as httpd:
        logger.info("HTTP server serving %s on port %d", static_dir, http_port)
        httpd.serve_forever()


async def main_async(args: argparse.Namespace) -> None:
    reader = RTKReader()
    reader.start()

    static_dir = Path(__file__).parent / "web_static"
    if not static_dir.exists():
        logger.warning("web_static directory not found at %s", static_dir)
    else:
        http_thread = threading.Thread(
            target=start_http_server,
            args=(static_dir, args.ws_port),
            daemon=True,
            name="rtk-http-server",
        )
        http_thread.start()

    ws_actual_port = args.ws_port + 1
    logger.info("Starting WebSocket server on ws://localhost:%d", ws_actual_port)
    logger.info("Open browser at http://localhost:%d", args.ws_port)

    if args.open_browser:
        url = f"http://localhost:{args.ws_port}"
        threading.Timer(1.0, lambda: webbrowser.open(url)).start()

    asyncio.create_task(broadcast_loop(reader, args.hz))

    async with websockets.serve(ws_handler, "0.0.0.0", ws_actual_port):
        logger.info("RTK bridge running. Press Ctrl+C to stop.")
        await asyncio.Future()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="RTK serial-to-WebSocket bridge")
    parser.add_argument(
        "--ws-port",
        type=int,
        default=8775,
        help="HTTP port for web UI (WebSocket uses ws-port+1)",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=5.0,
        help="Broadcast rate in Hz (default: 5)",
    )
    parser.add_argument(
        "--open-browser",
        default=True,
        help="Auto-open browser after startup",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    logger.info(
        "Starting RTK bridge | http=:%d | ws=:%d | broadcast_hz=%.2f",
        args.ws_port,
        args.ws_port + 1,
        args.hz,
    )
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        logger.info("Bridge stopped by user.")
