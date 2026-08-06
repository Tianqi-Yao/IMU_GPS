"""replay_rtk_websocket.py — hardware-free WebSocket source for 02_RTK.

Broadcasts a recorded data_log/*.jsonl file (single-source rtk_frame
contract) on the same WebSocket port rtk_bridge.py normally uses. By
default replays the most recently recorded file.

Note: this repo's older rtk_raw_v1.jsonl/v2.jsonl samples used a since-
removed multi-antenna schema and are intentionally NOT carried over here —
record a fresh sample with listen_rtk_websocket.py against the current
single-source bridge.

Run: python replay_rtk_websocket.py
"""

import asyncio
import json
import logging
import sys
from pathlib import Path
from typing import List, Optional

import websockets

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    import config as _cfg
except ImportError:
    _cfg = None

from common.ports import derive_ws_port

DATA_LOG_DIR = Path(__file__).parent / "data_log"


def _find_latest_jsonl(directory: Path) -> Optional[Path]:
    files = list(directory.glob("*.jsonl"))
    return max(files, key=lambda p: p.stat().st_mtime, default=None)


INPUT_PATH = _find_latest_jsonl(DATA_LOG_DIR)
HOST = "0.0.0.0"
PORT = derive_ws_port(_cfg.RTK_WS_PORT if _cfg else 8775)
HZ = 5.0
LOOP_FOREVER = True

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


def _load_jsonl(path: Path) -> List[str]:
    if not path.exists():
        raise FileNotFoundError(f"Input file not found: {path}")
    rows: List[str] = []
    skipped = 0
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                skipped += 1
                continue
            rows.append(json.dumps(obj, ensure_ascii=False, separators=(",", ":")))
    if skipped:
        logger.warning("Skipped %d malformed line(s) in %s", skipped, path)
    if not rows:
        raise RuntimeError(f"No valid JSON frames found in {path}")
    return rows


async def _run_server(records: List[str], host: str, port: int, hz: float, loop_forever: bool) -> None:
    interval = 1.0 / hz
    clients: set = set()

    async def handler(ws):
        clients.add(ws)
        try:
            async for _ in ws:
                pass
        finally:
            clients.discard(ws)

    async def broadcaster():
        while True:
            for raw in records:
                if clients:
                    dead = set()
                    for ws in list(clients):
                        try:
                            await ws.send(raw)
                        except websockets.exceptions.ConnectionClosed:
                            dead.add(ws)
                    clients.difference_update(dead)
                    await asyncio.sleep(interval)
                else:
                    await asyncio.sleep(0.05)
            if not loop_forever:
                break

    async with websockets.serve(handler, host, port):
        logger.info("Replaying %d frame(s) at %.1f Hz on ws://%s:%d", len(records), hz, host, port)
        await broadcaster()


def main() -> None:
    if INPUT_PATH is None:
        raise SystemExit(
            f"No .jsonl files found in {DATA_LOG_DIR}. "
            "Run listen_rtk_websocket.py against a real bridge first."
        )
    records = _load_jsonl(INPUT_PATH)
    try:
        asyncio.run(_run_server(records, HOST, PORT, HZ, LOOP_FOREVER))
    except KeyboardInterrupt:
        logger.info("Stopped")


if __name__ == "__main__":
    main()
