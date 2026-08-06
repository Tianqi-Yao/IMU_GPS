"""nav_bridge.py — IMU + RTK passthrough bridge.

IMPORTANT: despite the module name "Nav", this bridge does NOT fuse IMU and
RTK data. It subscribes to both upstream WebSockets and re-broadcasts their
latest frames, unmodified, nested under "imu"/"rtk" keys in a single
envelope. No coordinate-frame unification, filtering, weighted fusion, or
arrival computation happens here — see README.md for why, and what would be
required to turn this into a real fusion module.

INPUT  : ImuWsClient / RtkWsClient subscribe to 01_IMU / 02_RTK WebSockets.
CORE   : BroadcastLoop packs the two latest frames into a nav_frame on a
         fixed timer, decoupled from either upstream's arrival rate.
OUTPUT : common.ws_server.BroadcastWsServer broadcasts nav_frame payloads;
         common.http_server.StaticFileServer serves web_static/.

Run: python nav_bridge.py
"""

import asyncio
import json
import sys
import threading
import webbrowser
from pathlib import Path
from typing import Optional

import websockets

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    import config as _cfg
except ImportError:
    _cfg = None

from common.http_server import StaticFileServer
from common.logging_setup import setup_logger
from common.ports import derive_ws_port
from common.ws_server import BroadcastWsServer

WS_MSG_VERSION = 1

DEFAULT_HTTP_PORT = _cfg.NAV_WS_PORT if _cfg else 8785
DEFAULT_IMU_WS = _cfg.NAV_IMU_WS if _cfg else "ws://localhost:8766"
DEFAULT_RTK_WS = _cfg.NAV_RTK_WS if _cfg else "ws://localhost:8776"
DEFAULT_HZ = _cfg.NAV_HZ if _cfg else 10.0

logger = setup_logger(__name__, str(Path(__file__).parent / "nav_bridge.log"))


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 1 — I/O ADAPTERS
# ══════════════════════════════════════════════════════════════════════════
class ImuWsClient:
    """Subscribes to 01_IMU and can forward control messages back to it."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, ws_url: str) -> None:
        self._url = ws_url
        self._latest: dict = {}
        self._lock = threading.Lock()
        self._send_queue: "asyncio.Queue[str]" = asyncio.Queue(maxsize=8)

    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    async def send(self, msg: dict) -> None:
        try:
            self._send_queue.put_nowait(json.dumps(msg))
        except asyncio.QueueFull:
            logger.warning("ImuWsClient send queue full, dropping message")

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    sender_task = asyncio.create_task(self._sender(ws))
                    try:
                        async for raw in ws:
                            with self._lock:
                                self._latest = json.loads(raw)
                    finally:
                        sender_task.cancel()
            except Exception as exc:
                logger.warning("ImuWsClient connection error: %s", exc)
            await asyncio.sleep(self.RECONNECT_DELAY_S)

    async def _sender(self, ws) -> None:
        while True:
            msg = await self._send_queue.get()
            await ws.send(msg)


class RtkWsClient:
    """Subscribes to 02_RTK. No downstream control messages needed."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, ws_url: str) -> None:
        self._url = ws_url
        self._latest: dict = {}
        self._lock = threading.Lock()

    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    async for raw in ws:
                        with self._lock:
                            self._latest = json.loads(raw)
            except Exception as exc:
                logger.warning("RtkWsClient connection error: %s", exc)
            await asyncio.sleep(self.RECONNECT_DELAY_S)


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 2 — CORE
# ══════════════════════════════════════════════════════════════════════════
class BroadcastLoop:
    """Packs the two latest upstream frames into a nav_frame at a fixed rate."""

    def __init__(self, imu: ImuWsClient, rtk: RtkWsClient, queue: "asyncio.Queue[dict]", hz: float) -> None:
        self._imu = imu
        self._rtk = rtk
        self._queue = queue
        self._period = 1.0 / max(hz, 0.5)

    async def run(self) -> None:
        while True:
            payload = {
                "type": "nav_frame",
                "version": WS_MSG_VERSION,
                "imu": self._imu.latest,
                "rtk": self._rtk.latest,
            }
            try:
                self._queue.put_nowait(payload)
            except asyncio.QueueFull:
                logger.warning("BroadcastLoop: queue full, dropping frame")
            await asyncio.sleep(self._period)


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 3 — APPLICATION
# ══════════════════════════════════════════════════════════════════════════
class NavBridge:
    def __init__(self, http_port: int, imu_ws_url: str, rtk_ws_url: str, hz: float, static_dir: Path) -> None:
        self._http_port = http_port
        self._ws_port = derive_ws_port(http_port)
        self._imu_ws_url = imu_ws_url
        self._rtk_ws_url = rtk_ws_url
        self._hz = hz
        self._static_dir = static_dir

    def run(self) -> None:
        if self._static_dir.exists():
            StaticFileServer(self._static_dir, self._http_port).start()
        else:
            logger.warning("Static dir %s not found; HTTP page disabled.", self._static_dir)

        threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{self._http_port}")).start()

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("Stopped")

    async def _run_async(self) -> None:
        queue: "asyncio.Queue[dict]" = asyncio.Queue(maxsize=20)
        imu_client = ImuWsClient(self._imu_ws_url)
        rtk_client = RtkWsClient(self._rtk_ws_url)
        broadcast_loop = BroadcastLoop(imu_client, rtk_client, queue, self._hz)

        async def _on_client_message(_websocket, raw: str) -> None:
            try:
                msg = json.loads(raw)
            except json.JSONDecodeError:
                return
            if "set_north_offset" in msg:
                await imu_client.send({"set_north_offset": msg["set_north_offset"]})

        ws_server = BroadcastWsServer("0.0.0.0", self._ws_port, on_client_message=_on_client_message)

        async def _drain_queue() -> None:
            while True:
                payload = await queue.get()
                await ws_server.broadcast(payload)

        await asyncio.gather(
            imu_client.run(), rtk_client.run(), broadcast_loop.run(), _drain_queue(), ws_server.serve()
        )


if __name__ == "__main__":
    NavBridge(
        http_port=DEFAULT_HTTP_PORT,
        imu_ws_url=DEFAULT_IMU_WS,
        rtk_ws_url=DEFAULT_RTK_WS,
        hz=DEFAULT_HZ,
        static_dir=Path(__file__).parent / "web_static",
    ).run()
