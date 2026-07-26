"""rtk_bridge.py — RTK GPS serial (NMEA 0183) -> WebSocket bridge.

INPUT  : SerialReader reads raw NMEA lines from the RTK receiver.
CORE   : NMEAPipeline verifies checksums, parses GGA/RMC sentences into a
         shared, lock-protected RTKFrame, and exposes a snapshot() read
         that is decoupled from the NMEA arrival rate.
OUTPUT : common.ws_server.BroadcastWsServer broadcasts rtk_frame payloads
         polled from the pipeline at a fixed rate (BroadcastLoop);
         common.http_server.StaticFileServer serves web_static/.

This is a single-source contract: lat/lon/fix_quality/... from one RTK
receiver. Dual-antenna heading measurement is not part of this contract
(see README.md).
Run: python rtk_bridge.py
"""

import asyncio
import copy
import json
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import serial

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

WS_MSG_TYPE_RTK_FRAME = "rtk_frame"
WS_MSG_VERSION = 1

DEFAULT_SERIAL_PORT = _cfg.RTK_SERIAL_PORT if _cfg else "/dev/cu.usbmodem1101"
DEFAULT_BAUD = _cfg.RTK_BAUD if _cfg else 115200
DEFAULT_HTTP_PORT = _cfg.RTK_WS_PORT if _cfg else 8775
DEFAULT_HZ = _cfg.RTK_HZ if _cfg else 5.0
DEFAULT_LAT = _cfg.RTK_DEFAULT_LAT if _cfg else 38.9412928598587
DEFAULT_LON = _cfg.RTK_DEFAULT_LON if _cfg else -92.31884600793728

logger = setup_logger(__name__, str(Path(__file__).parent / "rtk_bridge.log"))


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 1 — DATA MODEL
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class RTKFrame:
    lat: Optional[float] = None          # +N/-S decimal degrees
    lon: Optional[float] = None          # +E/-W decimal degrees
    alt: Optional[float] = None          # metres MSL
    fix_quality: int = 0                 # 0=no fix,1=GPS,2=DGPS,4=RTK fixed,5=RTK float
    num_sats: int = 0
    hdop: Optional[float] = None
    speed_knots: Optional[float] = None  # from RMC
    track_deg: Optional[float] = None    # true course over ground, from RMC
    rtk_ts: Optional[float] = None       # bridge time.time() at last GGA update

    def to_dict(self, *, server_ts: float, default_lat: float, default_lon: float) -> dict:
        use_default = self.lat is None or self.lon is None
        return {
            "type": WS_MSG_TYPE_RTK_FRAME,
            "version": WS_MSG_VERSION,
            "lat": default_lat if use_default else self.lat,
            "lon": default_lon if use_default else self.lon,
            "alt": self.alt,
            "fix_quality": self.fix_quality,
            "num_sats": self.num_sats,
            "hdop": self.hdop,
            "speed_knots": self.speed_knots,
            "track_deg": self.track_deg,
            "rtk_ts": self.rtk_ts if self.rtk_ts is not None else 0.0,
            "server_ts": server_ts,
            "source": "default" if use_default else "rtk",
        }


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 2 — CORE PIPELINE
# ══════════════════════════════════════════════════════════════════════════
class NMEAPipeline:
    """Thread-safe NMEA GGA/RMC accumulator with a decoupled snapshot() read."""

    def __init__(self) -> None:
        self._frame = RTKFrame()
        self._lock = threading.Lock()

    def process(self, raw_line: str) -> None:
        validated = self._verify_checksum(raw_line)
        if validated is not None:
            self._dispatch(validated)

    def snapshot(self) -> RTKFrame:
        with self._lock:
            return copy.copy(self._frame)

    @staticmethod
    def _verify_checksum(line: str) -> Optional[str]:
        if not line.startswith("$") or "*" not in line:
            return None
        try:
            star_idx = line.rindex("*")
            expected = int(line[star_idx + 1:star_idx + 3], 16)
        except (ValueError, IndexError):
            return None
        payload = line[1:star_idx]
        computed = 0
        for ch in payload:
            computed ^= ord(ch)
        if computed != expected:
            logger.warning("NMEA checksum mismatch, dropping line: %r", line)
            return None
        return line

    def _dispatch(self, line: str) -> None:
        body = line[1:].split("*")[0]
        fields = body.split(",")
        if len(fields[0]) < 3:
            return
        sentence_type = fields[0][2:]
        if sentence_type == "GGA":
            self._parse_gga(fields)
        elif sentence_type == "RMC":
            self._parse_rmc(fields)

    def _parse_gga(self, fields: list) -> None:
        if len(fields) < 11:
            return
        lat = self._nmea_to_decimal(fields[2], fields[3]) if fields[2] and fields[3] else None
        lon = self._nmea_to_decimal(fields[4], fields[5]) if fields[4] and fields[5] else None
        fix_quality = int(fields[6]) if fields[6] else 0
        num_sats = int(fields[7]) if fields[7] else 0
        hdop = float(fields[8]) if fields[8] else None
        alt = float(fields[9]) if fields[9] else None
        with self._lock:
            if lat is not None:
                self._frame.lat = lat
            if lon is not None:
                self._frame.lon = lon
            self._frame.alt = alt
            self._frame.fix_quality = fix_quality
            self._frame.num_sats = num_sats
            self._frame.hdop = hdop
            self._frame.rtk_ts = time.time()

    def _parse_rmc(self, fields: list) -> None:
        if len(fields) < 9 or fields[2] != "A":
            return
        speed_knots = float(fields[7]) if fields[7] else None
        track_deg = float(fields[8]) if fields[8] else None
        with self._lock:
            self._frame.speed_knots = speed_knots
            self._frame.track_deg = track_deg

    @staticmethod
    def _nmea_to_decimal(raw: str, direction: str) -> float:
        """NMEA lat/lon format: DDMM.MMMM (lat) or DDDMM.MMMM (lon)."""
        dot_idx = raw.index(".")
        deg_str = raw[:dot_idx - 2]
        min_str = raw[dot_idx - 2:]
        degrees = float(deg_str) + float(min_str) / 60.0
        if direction in ("S", "W"):
            degrees = -degrees
        return degrees


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ══════════════════════════════════════════════════════════════════════════
class SerialReader:
    """Reads raw NMEA lines from the RTK serial port in a daemon thread."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, port: str, baud: int, pipeline: NMEAPipeline, timeout: float = 1.0) -> None:
        self._port = port
        self._baud = baud
        self._pipeline = pipeline
        self._timeout = timeout
        self._thread = threading.Thread(target=self._run, name="rtk-serial-reader", daemon=True)

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        while True:
            ser = None
            try:
                ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
                while True:
                    raw = ser.readline()
                    line = raw.decode("ascii", errors="ignore").strip()
                    if line:
                        self._pipeline.process(line)
            except serial.SerialException as exc:
                logger.error("RTK serial error on %s: %s. Retrying in %.0fs.",
                             self._port, exc, self.RECONNECT_DELAY_S)
            except Exception:
                logger.error("Unexpected RTK serial error", exc_info=True)
            finally:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
            time.sleep(self.RECONNECT_DELAY_S)


class BroadcastLoop:
    """Polls pipeline.snapshot() at a fixed rate, decoupled from NMEA arrival rate."""

    def __init__(self, pipeline: NMEAPipeline, queue: "asyncio.Queue[dict]", hz: float,
                 default_lat: float, default_lon: float) -> None:
        self._pipeline = pipeline
        self._queue = queue
        self._period = 1.0 / max(hz, 0.5)
        self._default_lat = default_lat
        self._default_lon = default_lon

    async def run(self) -> None:
        while True:
            frame = self._pipeline.snapshot()
            payload = frame.to_dict(server_ts=time.time(),
                                     default_lat=self._default_lat, default_lon=self._default_lon)
            try:
                self._queue.put_nowait(payload)
            except asyncio.QueueFull:
                logger.warning("BroadcastLoop: queue full, dropping frame")
            await asyncio.sleep(self._period)


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ══════════════════════════════════════════════════════════════════════════
class RTKBridge:
    def __init__(self, serial_port: str, baud: int, http_port: int, hz: float,
                 static_dir: Path, default_lat: float, default_lon: float,
                 open_browser: bool = True) -> None:
        self._serial_port = serial_port
        self._baud = baud
        self._http_port = http_port
        self._ws_port = derive_ws_port(http_port)
        self._hz = hz
        self._static_dir = static_dir
        self._default_lat = default_lat
        self._default_lon = default_lon
        self._open_browser = open_browser

    def run(self) -> None:
        pipeline = NMEAPipeline()
        SerialReader(self._serial_port, self._baud, pipeline).start()

        if self._static_dir.exists():
            StaticFileServer(self._static_dir, self._http_port).start()
        else:
            logger.warning("Static dir %s not found; HTTP page disabled.", self._static_dir)

        if self._open_browser:
            threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{self._http_port}")).start()

        try:
            asyncio.run(self._run_async(pipeline))
        except KeyboardInterrupt:
            logger.info("Stopped")

    async def _run_async(self, pipeline: NMEAPipeline) -> None:
        queue: "asyncio.Queue[dict]" = asyncio.Queue(maxsize=10)
        ws_server = BroadcastWsServer("0.0.0.0", self._ws_port)
        broadcast_loop = BroadcastLoop(pipeline, queue, self._hz, self._default_lat, self._default_lon)

        async def _drain_queue() -> None:
            while True:
                payload = await queue.get()
                await ws_server.broadcast(payload)

        await asyncio.gather(broadcast_loop.run(), _drain_queue(), ws_server.serve())


if __name__ == "__main__":
    RTKBridge(
        serial_port=DEFAULT_SERIAL_PORT,
        baud=DEFAULT_BAUD,
        http_port=DEFAULT_HTTP_PORT,
        hz=DEFAULT_HZ,
        static_dir=Path(__file__).parent / "web_static",
        default_lat=DEFAULT_LAT,
        default_lon=DEFAULT_LON,
    ).run()
