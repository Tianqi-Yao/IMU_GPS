"""imu_bridge.py — BNO085 serial -> WebSocket bridge.

INPUT  : SerialReader reads compact-JSON lines from the ESP32C3 firmware
         (see bno085_esp32c3/bno085_esp32c3.ino for the wire format).
CORE   : IMUPipeline expands the compact protocol, derives roll/pitch/yaw
         and a web-aligned compass heading from the quaternion, and tracks
         instantaneous frame rate. Pure functions/methods, no socket state.
OUTPUT : common.ws_server.BroadcastWsServer broadcasts imu_frame payloads;
         common.http_server.StaticFileServer serves web_static/.

Contract: see README.md for the full imu_frame JSON schema and the
north-offset calibration control message.
Run: python imu_bridge.py
"""

import asyncio
import json
import math
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass, field
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

WS_MSG_TYPE_IMU_FRAME = "imu_frame"
WS_MSG_VERSION = 1

DEFAULT_SERIAL_PORT = _cfg.IMU_SERIAL_PORT if _cfg else "/dev/cu.usbmodem1201"
DEFAULT_BAUD = _cfg.IMU_BAUD if _cfg else 921600
DEFAULT_HTTP_PORT = _cfg.IMU_WS_PORT if _cfg else 8765
DEFAULT_NORTH_OFFSET = _cfg.IMU_NORTH_OFFSET if _cfg else 0.0

logger = setup_logger(__name__, str(Path(__file__).parent / "imu_bridge.log"))


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 1 — DATA MODEL
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class IMUFrame:
    qi: float
    qj: float
    qk: float
    qr: float
    roll: Optional[float] = None
    pitch: Optional[float] = None
    yaw: Optional[float] = None
    heading_raw: Optional[float] = None
    heading: Optional[float] = None
    heading_dir: Optional[str] = None
    hz: Optional[float] = None
    north_offset_deg: float = 0.0
    extra: dict = field(default_factory=dict)

    def to_dict(self) -> dict:
        payload = {
            "type": WS_MSG_TYPE_IMU_FRAME,
            "version": WS_MSG_VERSION,
            "rot": {"qi": self.qi, "qj": self.qj, "qk": self.qk, "qr": self.qr},
        }
        if self.roll is not None:
            payload["euler"] = {
                "roll": self.roll,
                "pitch": self.pitch,
                "yaw": self.yaw,
                "north_offset_deg": self.north_offset_deg,
            }
        if self.hz is not None:
            payload["hz"] = self.hz
        if self.heading is not None:
            payload["heading"] = {
                "raw": self.heading_raw,
                "deg": self.heading,
                "dir": self.heading_dir,
            }
        payload.update(self.extra)
        return payload


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 2 — CORE PIPELINE
# ══════════════════════════════════════════════════════════════════════════
class FrameRateTracker:
    """Rolling-window instantaneous frame-rate estimator."""

    def __init__(self, window: int = 50) -> None:
        self._window = window
        self._times: list = []

    def tick(self) -> float:
        now = time.monotonic()
        self._times.append(now)
        if len(self._times) > self._window:
            self._times = self._times[-self._window:]
        if len(self._times) >= 2:
            return round((len(self._times) - 1) / (self._times[-1] - self._times[0]), 1)
        return 0.0


class IMUPipeline:
    """Parses compact-protocol serial lines into imu_frame JSON strings.

    Holds no socket/serial state; safe to unit test with plain strings.
    """

    def __init__(self, hz_tracker: FrameRateTracker, north_offset_deg: float = 0.0) -> None:
        self._hz_tracker = hz_tracker
        self._north_offset_deg = north_offset_deg
        # Written from the asyncio event loop thread (set_north_offset
        # control message), read from the serial reader thread (_parse).
        self._north_offset_lock = threading.Lock()

    @property
    def north_offset_deg(self) -> float:
        with self._north_offset_lock:
            return self._north_offset_deg

    @north_offset_deg.setter
    def north_offset_deg(self, value: float) -> None:
        with self._north_offset_lock:
            self._north_offset_deg = value

    def process(self, raw_line: str) -> Optional[dict]:
        frame = self._parse(raw_line)
        if frame is None:
            return None
        frame = self._enrich_euler(frame)
        frame = self._enrich_heading(frame)
        frame = self._enrich_hz(frame)
        return frame.to_dict()

    def _parse(self, line: str) -> Optional[IMUFrame]:
        try:
            data = json.loads(line)
        except json.JSONDecodeError:
            logger.debug("Dropping malformed JSON line: %r", line)
            return None

        if "r" in data and isinstance(data["r"], list):
            data = self._expand_compact(data)

        rot = data.get("rot")
        if not rot:
            logger.debug("Frame missing 'rot' field, skipping.")
            return None

        extra = {k: v for k, v in data.items() if k != "rot"}
        return IMUFrame(
            qi=rot.get("qi", 0.0),
            qj=rot.get("qj", 0.0),
            qk=rot.get("qk", 0.0),
            qr=rot.get("qr", 1.0),
            north_offset_deg=self.north_offset_deg,
            extra=extra,
        )

    @staticmethod
    def _expand_compact(data: dict) -> dict:
        """Expand the firmware's single-letter compact keys into full field names.

        Wire format (see bno085_esp32c3.ino outputJSON()):
          t -> ts (scalar)
          r -> rot        {qi, qj, qk, qr, acc}
          g -> game_rot   {qi, qj, qk, qr}
          a -> accel      {x, y, z}
          l -> lin_accel  {x, y, z}
          v -> gravity    {x, y, z}
          w -> gyro       {x, y, z}
          m -> mag        {x, y, z}
          s -> steps      (scalar)
          c -> cal        (scalar, 0-3)
        """
        out: dict = {}
        if "t" in data:
            out["ts"] = data["t"]
        r = data.get("r")
        if isinstance(r, list) and len(r) >= 4:
            out["rot"] = {
                "qi": r[0], "qj": r[1], "qk": r[2], "qr": r[3],
                "acc": r[4] if len(r) > 4 else 0.0,
            }
        g = data.get("g")
        if isinstance(g, list) and len(g) >= 4:
            out["game_rot"] = {"qi": g[0], "qj": g[1], "qk": g[2], "qr": g[3]}
        for key, name in (("a", "accel"), ("l", "lin_accel"), ("v", "gravity"), ("w", "gyro"), ("m", "mag")):
            vec = data.get(key)
            if isinstance(vec, list) and len(vec) >= 3:
                out[name] = {"x": vec[0], "y": vec[1], "z": vec[2]}
        if "s" in data:
            out["steps"] = data["s"]
        if "c" in data:
            out["cal"] = data["c"]
        return out

    @staticmethod
    def _enrich_euler(frame: IMUFrame) -> IMUFrame:
        """ZYX Euler angles from the (qi, qj, qk, qr) = (x, y, z, w) quaternion."""
        qi, qj, qk, qr = frame.qi, frame.qj, frame.qk, frame.qr

        sinr_cosp = 2.0 * (qr * qi + qj * qk)
        cosr_cosp = 1.0 - 2.0 * (qi * qi + qj * qj)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (qr * qj - qk * qi)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (qr * qk + qi * qj)
        cosy_cosp = 1.0 - 2.0 * (qj * qj + qk * qk)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        raw_yaw_deg = math.degrees(yaw)
        frame.roll = round(math.degrees(roll), 2)
        frame.pitch = round(math.degrees(pitch), 2)
        frame.yaw = round((raw_yaw_deg + frame.north_offset_deg) % 360, 2)
        return frame

    @staticmethod
    def _heading_to_direction(heading_deg: float) -> str:
        dirs = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
                "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
        idx = int(round((heading_deg % 360.0) / 22.5)) % 16
        return dirs[idx]

    @staticmethod
    def _quat_mul(a: tuple, b: tuple) -> tuple:
        """Hamilton product, both operands in (x, y, z, w) component order."""
        ax, ay, az, aw = a
        bx, by, bz, bw = b
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    @staticmethod
    def _normalize_quat(q: tuple) -> tuple:
        x, y, z, w = q
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1e-9:
            return (0.0, 0.0, 0.0, 1.0)
        return (x / norm, y / norm, z / norm, w / norm)

    def _enrich_heading(self, frame: IMUFrame) -> IMUFrame:
        """Web-aligned compass heading, independent of the raw `yaw` euler field.

        Prefers game_rot (magnetometer-free, stable indoors) over rot.
        Applies the same BNO085 Z-up -> Three.js Y-up frame correction the
        frontend uses, so backend and frontend heading stay numerically
        consistent.
        """
        source = frame.extra.get("game_rot")
        if source:
            q_src = (source.get("qi", 0.0), source.get("qj", 0.0), source.get("qk", 0.0), source.get("qr", 1.0))
        else:
            q_src = (frame.qi, frame.qj, frame.qk, frame.qr)
        q_src = self._normalize_quat(q_src)

        # BNO085 Z-up -> Three.js Y-up: -90 deg rotation about X axis.
        frame_correction = (-math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))
        qx, qy, qz, qw = self._normalize_quat(self._quat_mul(frame_correction, q_src))

        chip_x_world_x = 1.0 - 2.0 * (qy * qy + qz * qz)
        chip_x_world_z = 2.0 * (qx * qz - qw * qy)
        raw_heading = (math.degrees(math.atan2(chip_x_world_z, chip_x_world_x)) + 360.0) % 360.0
        corrected_heading = (raw_heading + frame.north_offset_deg) % 360.0

        frame.heading_raw = round(raw_heading, 3)
        frame.heading = round(corrected_heading, 3)
        frame.heading_dir = self._heading_to_direction(corrected_heading)
        return frame

    def _enrich_hz(self, frame: IMUFrame) -> IMUFrame:
        frame.hz = self._hz_tracker.tick()
        return frame


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ══════════════════════════════════════════════════════════════════════════
class SerialReader:
    """Reads newline-delimited compact-JSON frames from the IMU serial port.

    Runs in a dedicated daemon thread (pyserial is blocking); pushes parsed
    imu_frame JSON strings into an asyncio.Queue via
    run_coroutine_threadsafe.
    """

    _RETRY_DELAY_S = 3.0

    def __init__(self, port: str, baud: int, pipeline: IMUPipeline,
                 loop: asyncio.AbstractEventLoop, queue: "asyncio.Queue[dict]") -> None:
        self._port = port
        self._baud = baud
        self._pipeline = pipeline
        self._loop = loop
        self._queue = queue

    def start(self) -> None:
        threading.Thread(target=self.run, name="serial-reader", daemon=True).start()

    def run(self) -> None:
        while True:
            try:
                with serial.Serial(self._port, self._baud, timeout=1.0) as ser:
                    self._read_loop(ser)
            except serial.SerialException as exc:
                logger.error("Cannot open serial port %s: %s. Retrying in %.0fs.",
                             self._port, exc, self._RETRY_DELAY_S)
            except Exception:
                logger.error("Unexpected serial error", exc_info=True)
            time.sleep(self._RETRY_DELAY_S)

    def _read_loop(self, ser: serial.Serial) -> None:
        while True:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if line.startswith("#") or not line.startswith("{"):
                continue
            result = self._pipeline.process(line)
            if result is None:
                continue
            asyncio.run_coroutine_threadsafe(self._queue.put(result), self._loop)


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ══════════════════════════════════════════════════════════════════════════
class IMUBridge:
    def __init__(self, serial_port: str, baud: int, http_port: int,
                 static_dir: Path, north_offset_deg: float = 0.0) -> None:
        self._serial_port = serial_port
        self._baud = baud
        self._http_port = http_port
        self._ws_port = derive_ws_port(http_port)
        self._static_dir = static_dir
        self._north_offset_deg = north_offset_deg
        self._pipeline: Optional[IMUPipeline] = None

    def run(self) -> None:
        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("Stopped")

    async def _run_async(self) -> None:
        loop = asyncio.get_running_loop()
        queue: "asyncio.Queue[dict]" = asyncio.Queue(maxsize=200)

        hz_tracker = FrameRateTracker(window=50)
        self._pipeline = IMUPipeline(hz_tracker, north_offset_deg=self._north_offset_deg)

        SerialReader(self._serial_port, self._baud, self._pipeline, loop, queue).start()

        if self._static_dir.exists():
            StaticFileServer(self._static_dir, self._http_port).start()
        else:
            logger.warning("Static dir %s not found; HTTP page disabled.", self._static_dir)

        threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{self._http_port}")).start()

        ws_server = BroadcastWsServer(
            "0.0.0.0", self._ws_port, on_client_message=self._handle_client_message
        )

        async def _drain_queue() -> None:
            while True:
                payload = await queue.get()
                await ws_server.broadcast(payload)

        await asyncio.gather(_drain_queue(), ws_server.serve())

    def _handle_client_message(self, _websocket, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return
        if "set_north_offset" in msg and self._pipeline is not None:
            try:
                self._pipeline.north_offset_deg = float(msg["set_north_offset"])
            except (TypeError, ValueError):
                logger.warning("Ignoring invalid set_north_offset payload: %r", msg)


if __name__ == "__main__":
    IMUBridge(
        serial_port=DEFAULT_SERIAL_PORT,
        baud=DEFAULT_BAUD,
        http_port=DEFAULT_HTTP_PORT,
        static_dir=Path(__file__).parent / "web_static",
        north_offset_deg=DEFAULT_NORTH_OFFSET,
    ).run()
