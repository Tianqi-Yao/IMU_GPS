"""robot_bridge.py — Feather M4 serial control bridge.

INPUT  : SerialLink's reader thread splits the serial stream into lines;
         WS client subscriptions to 01_IMU/02_RTK; browser WS messages.
CORE   : RobotState (parses "O:"/"S:" lines into odom + auto-active state),
         VelocityRamp (accel-limited setpoint smoothing), Watchdog
         (heartbeat timeout -> e-stop), Recorder (rate-limited jsonl log).
         All four are plain objects with no socket/serial handles — testable
         standalone.
OUTPUT : SerialLink's write_* methods (serial); common.ws_server for the
         browser-facing broadcast; common.http_server for web_static/.

Key correctness fix vs. the pre-refactor implementation: the CAN-bus state
integer carried on every "O:" line (see CIRCUITPY/lib/farm_ng/utils/packet.py
AmigaControlState) is now the single source of truth for `active`, re-synced
on every "O:" line (~20 Hz). "S:ACTIVE"/"S:READY" lines are applied
immediately too (for a snappier UI response right after a manual toggle),
but are no longer the *only* thing that can set `active` — if the hardware
silently downgrades to READY without emitting "S:READY" (e.g. a CAN safety
interlock), the next "O:" line corrects the state within ~50ms instead of
leaving the bridge stuck reporting a stale ACTIVE forever.

Run: python robot_bridge.py
"""

import asyncio
import json
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Optional

import serial
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

# CIRCUITPY/lib/farm_ng/utils/packet.py :: AmigaControlState.STATE_AUTO_ACTIVE
STATE_AUTO_ACTIVE = 5

DEFAULT_HTTP_PORT = _cfg.ROBOT_WS_PORT if _cfg else 8888
DEFAULT_SERIAL_PORT = _cfg.ROBOT_SERIAL_PORT if _cfg else "/dev/cu.usbmodem1301"
DEFAULT_SERIAL_BAUD = _cfg.ROBOT_SERIAL_BAUD if _cfg else 115200
DEFAULT_SERIAL_TIMEOUT = _cfg.ROBOT_SERIAL_TIMEOUT if _cfg else 0.05
DEFAULT_MAX_LINEAR = _cfg.ROBOT_MAX_LINEAR if _cfg else 1.0
DEFAULT_MAX_ANGULAR = _cfg.ROBOT_MAX_ANGULAR if _cfg else 1.0
DEFAULT_MAX_LINEAR_ACCEL = _cfg.ROBOT_MAX_LINEAR_ACCEL if _cfg else 1.5
DEFAULT_MAX_ANGULAR_ACCEL = _cfg.ROBOT_MAX_ANGULAR_ACCEL if _cfg else 3.0
DEFAULT_WATCHDOG_TIMEOUT = _cfg.ROBOT_WATCHDOG_TIMEOUT if _cfg else 3.0
DEFAULT_RECORD_INTERVAL = max(0.2, _cfg.ROBOT_RECORD_INTERVAL if _cfg else 1.0)
DEFAULT_IMU_WS_URL = _cfg.ROBOT_IMU_WS if _cfg else "ws://localhost:8766"
DEFAULT_RTK_WS_URL = _cfg.ROBOT_RTK_WS if _cfg else "ws://localhost:8776"

logger = setup_logger(__name__, str(Path(__file__).parent / "robot_bridge.log"))


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 1 — DATA MODEL / RECORDING
# ══════════════════════════════════════════════════════════════════════════
class Recorder:
    """Rate-limited, per-type-slimmed jsonl recorder for manual test runs."""

    LOG_DIR = Path(__file__).parent / "data_log"

    def __init__(self, interval: float = 1.0) -> None:
        self._interval = interval
        self._file = None
        self.filename: str = ""
        self._last_ts: dict = {}
        self._lock = threading.Lock()

    @property
    def active(self) -> bool:
        return self._file is not None

    def start(self) -> str:
        self.LOG_DIR.mkdir(parents=True, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"run_{ts}.jsonl"
        path = self.LOG_DIR / self.filename
        with self._lock:
            self._file = path.open("a", encoding="utf-8")
        self._last_ts = {}
        return self.filename

    def stop(self) -> None:
        with self._lock:
            if self._file:
                self._file.close()
                self._file = None

    def write(self, rec_ts: float, msg_type: str, msg: dict) -> None:
        if rec_ts - self._last_ts.get(msg_type, 0.0) < self._interval:
            return
        row = self._slim(rec_ts, msg_type, msg)
        if row is None:
            return
        self._last_ts[msg_type] = rec_ts
        line = json.dumps(row, ensure_ascii=False) + "\n"
        with self._lock:
            if self._file:
                self._file.write(line)
                self._file.flush()

    @staticmethod
    def _slim(rec_ts: float, msg_type: str, msg: dict) -> Optional[dict]:
        if msg_type == "imu":
            heading = msg.get("heading", {})
            euler = msg.get("euler", {})
            return {
                "type": "imu", "rec_ts": rec_ts,
                "heading_deg": heading.get("deg"), "heading_dir": heading.get("dir"),
                "roll": euler.get("roll"), "pitch": euler.get("pitch"), "yaw": euler.get("yaw"),
            }
        if msg_type == "rtk":
            return {
                "type": "rtk", "rec_ts": rec_ts,
                "lat": msg.get("lat"), "lon": msg.get("lon"),
                "fix_quality": msg.get("fix_quality"), "num_sats": msg.get("num_sats"),
            }
        if msg_type == "odom":
            return {
                "type": "odom", "rec_ts": rec_ts,
                "v": msg.get("v"), "w": msg.get("w"), "soc": msg.get("soc"), "state": msg.get("state"),
            }
        return None


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 2 — CORE
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class RobotEvent:
    kind: str  # "odom" | "state_change"
    odom: Optional[dict] = None
    active: Optional[bool] = None
    active_changed: bool = False


class RobotState:
    """Parses "O:"/"S:" serial lines; owns odom + auto-active truth.

    "O:" line's integer state field (CAN-bus AmigaControlState) is the sole
    source of truth for `active`, re-synced on every line. "S:ACTIVE"/
    "S:READY" lines apply immediately too, for snappier feedback right after
    a manual toggle, but the very next "O:" line will correct them if wrong.
    """

    def __init__(self) -> None:
        self.active: bool = False
        self.last_odom: dict = {}

    def handle_serial_line(self, line: bytes) -> Optional[RobotEvent]:
        if line.startswith(b"O:"):
            return self._handle_odom_line(line)
        if line in (b"S:ACTIVE", b"S:READY"):
            new_active = line == b"S:ACTIVE"
            if new_active != self.active:
                self.active = new_active
                return RobotEvent(kind="state_change", active=self.active)
            return None
        return None

    def _handle_odom_line(self, line: bytes) -> Optional[RobotEvent]:
        try:
            parts = line[2:].decode().split(",")
            v = float(parts[0])
            w = float(parts[1])
            state_int = int(parts[2]) if len(parts) > 2 else None
            soc = int(parts[3]) if len(parts) > 3 else None
        except (ValueError, IndexError):
            return None

        self.last_odom = {"v": v, "w": w, "state": state_int, "soc": soc, "ts": time.time()}

        active_changed = False
        if state_int is not None:
            new_active = state_int == STATE_AUTO_ACTIVE
            if new_active != self.active:
                self.active = new_active
                active_changed = True

        return RobotEvent(kind="odom", odom=dict(self.last_odom),
                           active=self.active, active_changed=active_changed)


class VelocityRamp:
    """Accel-limited setpoint smoothing, decoupled from WS message arrival rate."""

    def __init__(self, max_linear_accel: float, max_angular_accel: float) -> None:
        self._max_linear_accel = max_linear_accel
        self._max_angular_accel = max_angular_accel
        self.target_linear = 0.0
        self.target_angular = 0.0
        self.out_linear = 0.0
        self.out_angular = 0.0

    def set_target(self, linear: float, angular: float) -> None:
        self.target_linear = linear
        self.target_angular = angular

    def stop(self) -> None:
        self.target_linear = self.target_angular = 0.0
        self.out_linear = self.out_angular = 0.0

    def step(self, dt: float) -> None:
        self.out_linear = self._step_toward(self.out_linear, self.target_linear, self._max_linear_accel * dt)
        self.out_angular = self._step_toward(self.out_angular, self.target_angular, self._max_angular_accel * dt)

    @staticmethod
    def _step_toward(current: float, target: float, max_delta: float) -> float:
        diff = target - current
        if diff > max_delta:
            return current + max_delta
        if diff < -max_delta:
            return current - max_delta
        return target


class Watchdog:
    """Tracks time since the last heartbeat/joystick message."""

    def __init__(self, timeout: float) -> None:
        self._timeout = timeout
        self._last_heartbeat = time.time()

    def feed(self) -> None:
        self._last_heartbeat = time.time()

    def is_expired(self) -> bool:
        return (time.time() - self._last_heartbeat) > self._timeout


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O ADAPTERS
# ══════════════════════════════════════════════════════════════════════════
class SerialLink:
    """Owns the physical serial connection: line-buffered reads and writes.

    Reading and writing share one serial.Serial resource by necessity; what
    a line *means* (RobotState) and what to write in response (the bridge's
    control loops) live outside this class.
    """

    def __init__(self, port: str, baud: int, timeout: float) -> None:
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._ser: Optional[serial.Serial] = None
        self._ser_lock = threading.Lock()

    def open(self) -> None:
        try:
            with self._ser_lock:
                self._ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
            logger.info("Opened serial port %s @ %d baud", self._port, self._baud)
        except serial.SerialException as exc:
            logger.error("Cannot open serial port %s: %s", self._port, exc)

    def write_velocity(self, linear: float, angular: float) -> None:
        self._write(f"V{linear:.2f},{angular:.2f}\n".encode())

    def write_raw(self, data: bytes) -> None:
        self._write(data)

    def write_hbridge(self, cmd: str) -> None:
        self._write(f"H{cmd}\n".encode())

    def _write(self, data: bytes) -> None:
        with self._ser_lock:
            ser = self._ser
        if ser is None or not ser.is_open:
            return
        try:
            ser.write(data)
        except serial.SerialException as exc:
            logger.error("Serial write failed: %s", exc)

    def start_reader(self, on_line: Callable[[bytes], None]) -> None:
        threading.Thread(target=self._reader_loop, args=(on_line,), name="serial-reader", daemon=True).start()

    def _reader_loop(self, on_line: Callable[[bytes], None]) -> None:
        buf = b""
        while True:
            try:
                with self._ser_lock:
                    ser = self._ser
                if ser is None or not ser.is_open:
                    buf = b""
                    time.sleep(0.1)
                    continue
                n = ser.in_waiting
                chunk = ser.read(n) if n > 0 else b""
            except serial.SerialException as exc:
                logger.error("Serial read error: %s", exc)
                buf = b""
                time.sleep(0.1)
                continue
            if chunk:
                buf += chunk
                while b"\n" in buf:
                    line, buf = buf.split(b"\n", 1)
                    on_line(line.strip())
            else:
                time.sleep(0.01)


class ImuWsClient:
    """Subscribes to 01_IMU and forwards each frame via on_frame."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, url: str, on_frame: Callable[[dict], "asyncio.Future"]) -> None:
        self._url = url
        self._on_frame = on_frame

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    async for raw in ws:
                        msg = json.loads(raw)
                        await self._on_frame({**msg, "type": "imu"})
            except Exception as exc:
                logger.warning("ImuWsClient connection error: %s", exc)
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class RtkWsClient:
    """Subscribes to 02_RTK, tags each frame with a computed `available` flag."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, url: str, on_frame: Callable[[dict], "asyncio.Future"]) -> None:
        self._url = url
        self._on_frame = on_frame

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    async for raw in ws:
                        msg = json.loads(raw)
                        available = bool(msg.get("source") == "rtk" or (msg.get("fix_quality") or 0) > 0)
                        await self._on_frame({**msg, "type": "rtk", "available": available})
            except Exception as exc:
                logger.warning("RtkWsClient connection error: %s", exc)
            await asyncio.sleep(self.RECONNECT_DELAY_S)


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ══════════════════════════════════════════════════════════════════════════
class RobotBridge:
    def __init__(self, http_port: int, serial_port: str, serial_baud: int, serial_timeout: float,
                 max_linear: float, max_angular: float, max_linear_accel: float, max_angular_accel: float,
                 watchdog_timeout: float, imu_ws_url: str, rtk_ws_url: str,
                 record_interval: float, static_dir: Path) -> None:
        self._http_port = http_port
        self._ws_port = derive_ws_port(http_port)
        self._max_linear = max_linear
        self._max_angular = max_angular
        self._imu_ws_url = imu_ws_url
        self._rtk_ws_url = rtk_ws_url
        self._static_dir = static_dir

        self._serial = SerialLink(serial_port, serial_baud, serial_timeout)
        self._robot_state = RobotState()
        self._velocity_ramp = VelocityRamp(max_linear_accel, max_angular_accel)
        self._watchdog = Watchdog(watchdog_timeout)
        self._recorder = Recorder(interval=record_interval)

        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._ws_server: Optional[BroadcastWsServer] = None

    def run(self) -> None:
        self._serial.open()
        self._serial.start_reader(self._on_serial_line)

        if self._static_dir.exists():
            StaticFileServer(
                self._static_dir, self._http_port, index_transform=self._inject_limits
            ).start()
        else:
            logger.warning("Static dir %s not found; HTTP page disabled.", self._static_dir)

        threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{self._http_port}")).start()

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("Stopped")

    def _inject_limits(self, html_bytes: bytes) -> bytes:
        html = html_bytes.decode("utf-8")
        html = html.replace(
            "<html", f'<html data-max-linear="{self._max_linear}" data-max-angular="{self._max_angular}"', 1
        )
        return html.encode("utf-8")

    async def _run_async(self) -> None:
        self._loop = asyncio.get_running_loop()
        self._ws_server = BroadcastWsServer(
            "0.0.0.0", self._ws_port,
            on_client_message=self._handle_client_message,
            on_client_connect=self._handle_client_connect,
        )

        imu_client = ImuWsClient(self._imu_ws_url, self._broadcast)
        rtk_client = RtkWsClient(self._rtk_ws_url, self._broadcast)

        await asyncio.gather(
            self._odom_broadcast_loop(),
            self._velocity_ramp_loop(),
            self._watchdog_loop(),
            imu_client.run(),
            rtk_client.run(),
            self._ws_server.serve(),
        )

    async def _broadcast(self, payload: dict) -> None:
        rec_ts = time.time()
        if self._recorder.active:
            self._recorder.write(rec_ts, payload.get("type", ""), payload)
        payload.setdefault("version", WS_MSG_VERSION)
        await self._ws_server.broadcast(payload)

    async def _handle_client_connect(self, _websocket) -> None:
        await self._broadcast({"type": "state_status", "active": self._robot_state.active})

    async def _handle_client_message(self, _websocket, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return
        msg_type = msg.get("type")
        loop = asyncio.get_running_loop()

        if msg_type in ("heartbeat", "joystick"):
            self._watchdog.feed()

        if msg_type == "joystick":
            linear = max(-self._max_linear, min(self._max_linear, float(msg.get("linear", 0.0))))
            angular = max(-self._max_angular, min(self._max_angular, float(msg.get("angular", 0.0))))
            self._velocity_ramp.set_target(linear, angular)
        elif msg_type == "toggle_state":
            self._watchdog.feed()
            await loop.run_in_executor(None, self._serial.write_raw, b"\r")
        elif msg_type == "lift_control":
            cmd_map = {"up": "U", "down": "D", "stop": "S"}
            cmd = cmd_map.get(str(msg.get("cmd", "stop")).lower(), "S")
            await loop.run_in_executor(None, self._serial.write_hbridge, cmd)
        elif msg_type == "set_recording":
            if msg.get("enabled"):
                filename = self._recorder.start()
                await self._broadcast({"type": "rec_status", "recording": True, "filename": filename})
            else:
                self._recorder.stop()
                await self._broadcast({"type": "rec_status", "recording": False, "filename": ""})

    def _on_serial_line(self, line: bytes) -> None:
        """Runs on the serial reader thread; schedules async work on the event loop."""
        event = self._robot_state.handle_serial_line(line)
        if event is None or self._loop is None:
            return
        if event.kind == "state_change" or (event.kind == "odom" and event.active_changed):
            asyncio.run_coroutine_threadsafe(
                self._broadcast({"type": "state_status", "active": event.active}), self._loop
            )

    async def _odom_broadcast_loop(self) -> None:
        while True:
            await asyncio.sleep(0.05)  # 20 Hz
            odom = self._robot_state.last_odom
            if odom:
                await self._broadcast({"type": "odom", **odom})

    async def _velocity_ramp_loop(self) -> None:
        dt = 0.05  # 20 Hz
        loop = asyncio.get_running_loop()
        while True:
            await asyncio.sleep(dt)
            self._velocity_ramp.step(dt)
            await loop.run_in_executor(
                None, self._serial.write_velocity, self._velocity_ramp.out_linear, self._velocity_ramp.out_angular
            )

    async def _watchdog_loop(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            await asyncio.sleep(0.5)
            if self._watchdog.is_expired():
                logger.warning("Watchdog: no heartbeat — emergency stop")
                self._velocity_ramp.stop()
                await loop.run_in_executor(None, self._serial.write_velocity, 0.0, 0.0)
                if self._robot_state.active:
                    await loop.run_in_executor(None, self._serial.write_raw, b"\r")
                self._watchdog.feed()


if __name__ == "__main__":
    RobotBridge(
        http_port=DEFAULT_HTTP_PORT,
        serial_port=DEFAULT_SERIAL_PORT,
        serial_baud=DEFAULT_SERIAL_BAUD,
        serial_timeout=DEFAULT_SERIAL_TIMEOUT,
        max_linear=DEFAULT_MAX_LINEAR,
        max_angular=DEFAULT_MAX_ANGULAR,
        max_linear_accel=DEFAULT_MAX_LINEAR_ACCEL,
        max_angular_accel=DEFAULT_MAX_ANGULAR_ACCEL,
        watchdog_timeout=DEFAULT_WATCHDOG_TIMEOUT,
        imu_ws_url=DEFAULT_IMU_WS_URL,
        rtk_ws_url=DEFAULT_RTK_WS_URL,
        record_interval=DEFAULT_RECORD_INTERVAL,
        static_dir=Path(__file__).parent / "web_static",
    ).run()
