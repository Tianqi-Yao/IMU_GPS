"""replay_robot_websocket.py — hardware-free WebSocket source for 04_Robot.

Broadcasts a recorded data_log/*.jsonl file on the same WebSocket port
robot_bridge.py normally uses. By default replays the most recently
recorded file (falls back to the bundled robot_raw_v1.jsonl baseline
sample, whose schema matches Recorder._slim's odom/imu/rtk output).

Run: python replay_robot_websocket.py
"""

import asyncio
import json
import logging
from pathlib import Path
from typing import List, Optional

import websockets

DATA_LOG_DIR = Path(__file__).parent / "data_log"


def _find_latest_jsonl(directory: Path) -> Optional[Path]:
    files = sorted(directory.glob("*.jsonl"))
    return files[-1] if files else None


INPUT_PATH = _find_latest_jsonl(DATA_LOG_DIR)
HOST = "0.0.0.0"
PORT = 8889
HZ = 20.0
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
                    for ws in clients:
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
            "Run listen_robot_websocket.py against a real bridge first."
        )
    records = _load_jsonl(INPUT_PATH)
    try:
        asyncio.run(_run_server(records, HOST, PORT, HZ, LOOP_FOREVER))
    except KeyboardInterrupt:
        logger.info("Stopped")


if __name__ == "__main__":
    main()
