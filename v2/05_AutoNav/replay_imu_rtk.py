"""replay_imu_rtk.py — offline IMU+RTK replay for developing autonav_bridge.py
without any hardware attached.

Serves two independent WebSocket endpoints on the exact ports imu_bridge.py
and rtk_bridge.py normally use, so autonav_bridge.py (pointed at
AUTONAV_IMU_WS/AUTONAV_RTK_WS in config.py, which default to these same
ports) can run against replayed data unmodified.

Unlike 01_IMU/replay_imu_websocket.py and 02_RTK/replay_rtk_websocket.py,
this is a fixed pair of baseline files rather than "most recently recorded"
— point IMU_JSONL/RTK_JSONL at a matched pair of recordings from the same
session before using this for repeatable autonav testing. Loading tolerates
a missing file (logs a warning and serves nothing on that endpoint) rather
than crashing, so one replay source can be brought up before the other.

Replay is at a fixed rate (not the original capture's inter-frame timing) —
a known, documented simplification; see 01_IMU/02_RTK READMEs.

Run: python replay_imu_rtk.py
"""

import asyncio
import json
import logging
from pathlib import Path
from typing import List

import websockets

IMU_JSONL = Path(__file__).parent.parent / "01_IMU" / "data_log" / "imu_raw_v1.jsonl"
RTK_JSONL = Path(__file__).parent.parent / "02_RTK" / "data_log" / "rtk_raw_v1.jsonl"
IMU_HOST, IMU_PORT, IMU_HZ = "0.0.0.0", 8766, 20.0
RTK_HOST, RTK_PORT, RTK_HZ = "0.0.0.0", 8776, 5.0

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger(__name__)


def _load_jsonl(path: Path) -> List[str]:
    if not path.exists():
        logger.warning("%s not found; that endpoint will serve no frames.", path)
        return []
    rows: List[str] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                json.loads(line)
            except json.JSONDecodeError:
                continue
            rows.append(line)
    return rows


class ReplayServer:
    def __init__(self, name: str, frames: List[str], host: str, port: int, hz: float) -> None:
        self._name = name
        self._frames = frames
        self._host = host
        self._port = port
        self._period = 1.0 / hz
        self._clients: set = set()

    async def _handler(self, ws) -> None:
        self._clients.add(ws)
        try:
            async for _ in ws:
                pass
        finally:
            self._clients.discard(ws)

    async def _broadcast_loop(self) -> None:
        if not self._frames:
            await asyncio.Future()  # keep the server listening, but never send anything
        idx = 0
        while True:
            raw = self._frames[idx % len(self._frames)]
            dead = set()
            for ws in self._clients:
                try:
                    await ws.send(raw)
                except websockets.exceptions.ConnectionClosed:
                    dead.add(ws)
            self._clients.difference_update(dead)
            idx += 1
            await asyncio.sleep(self._period)

    async def serve(self) -> None:
        async with websockets.serve(self._handler, self._host, self._port):
            logger.info("%s: replaying %d frame(s) on ws://%s:%d", self._name, len(self._frames), self._host, self._port)
            await self._broadcast_loop()


async def _run_both(imu_server: "ReplayServer", rtk_server: "ReplayServer") -> None:
    await asyncio.gather(imu_server.serve(), rtk_server.serve())


def main() -> None:
    imu_server = ReplayServer("IMU", _load_jsonl(IMU_JSONL), IMU_HOST, IMU_PORT, IMU_HZ)
    rtk_server = ReplayServer("RTK", _load_jsonl(RTK_JSONL), RTK_HOST, RTK_PORT, RTK_HZ)
    try:
        asyncio.run(_run_both(imu_server, rtk_server))
    except KeyboardInterrupt:
        logger.info("Stopped")


if __name__ == "__main__":
    main()
