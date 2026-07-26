"""autonav_bridge.py — autonomous navigation state machine + I/O.

Data flow:
    imu_bridge (WS)  --> ImuWsClient  --\\
                                          AutoNavLoop -> algo.compute() -> ComputeResult
    rtk_bridge (WS)  --> RtkWsClient  --/        |
                                                   v
    path.csv --------------------------------------                RobotWsClient -> robot_bridge (WS)
                                                   |                       ^
                                        AutoNavWsServer (WS) <-------------+

INPUT  : ImuWsClient/RtkWsClient (subscribe upstream), browser WS control
         messages, path.csv waypoint loading.
CORE   : AutoNavLoop (state machine: idle/running/paused/lifting/arrived)
         calling autonav_algo.compute() for the control law. All math lives
         in autonav_algo.py; this file only sequences it.
OUTPUT : RobotWsClient (drive/lift commands to 04_Robot);
         common.ws_server.BroadcastWsServer (autonav_status to the
         dashboard); common.http_server.StaticFileServer (web_static/ +
         GET /export_csv).

Run: python autonav_bridge.py
"""

import asyncio
import csv
import importlib
import io
import json
import sys
import threading
import time
import webbrowser
from pathlib import Path
from typing import List, Optional, Tuple

import websockets

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    import config as _cfg
except ImportError:
    _cfg = None

import autonav_algo as algo
from common.http_server import StaticFileServer
from common.logging_setup import setup_logger
from common.ports import derive_ws_port
from common.ws_server import BroadcastWsServer


def _c(attr: str, default):
    return getattr(_cfg, attr, default) if _cfg else default


WS_MSG_VERSION = 1

DEFAULT_HTTP_PORT = _c("AUTONAV_WS_PORT", 8805)
IMU_WS_URL = _c("AUTONAV_IMU_WS", "ws://localhost:8766")
RTK_WS_URL = _c("AUTONAV_RTK_WS", "ws://localhost:8776")
ROBOT_WS_URL = _c("AUTONAV_ROBOT_WS", "ws://localhost:8889")
GPS_TIMEOUT_S = _c("AUTONAV_GPS_TIMEOUT_S", 5.0)
CONTROL_HZ = _c("AUTONAV_CONTROL_HZ", 5.0)
MANUAL_SPEED = _c("AUTONAV_MANUAL_SPEED", 0.4)
ARRIVE_FRAMES = _c("AUTONAV_ARRIVE_FRAMES", 1)
HEARTBEAT_INTERVAL = 0.2
ROBOT_SEND_TIMEOUT_S = _c("AUTONAV_ROBOT_SEND_TIMEOUT_S", 2.0)
LIFT_UP_DURATION_S = _c("LIFT_UP_DURATION_S", 5.0)
LIFT_DOWN_DURATION_S = _c("LIFT_DOWN_DURATION_S", 5.0)
PATH_FILE = Path(__file__).parent / "path.csv"

logger = setup_logger(__name__, str(Path(__file__).parent / "autonav_bridge.log"))


# ══════════════════════════════════════════════════════════════════════════
# Waypoint loading
# ══════════════════════════════════════════════════════════════════════════
def _convert_csv_to_waypoints(content: str) -> List[dict]:
    try:
        dialect = csv.Sniffer().sniff(content[:2048], delimiters="\t,")
    except csv.Error:
        dialect = csv.excel
    reader = csv.DictReader(io.StringIO(content), dialect=dialect)
    waypoints: List[dict] = []
    for row in reader:
        try:
            waypoints.append({
                "lat": float(row["lat"]),
                "lon": float(row["lon"]),
                "type": str(row.get("type", "")).strip(),
                "lift": str(row.get("lift", "")).strip().lower(),
            })
        except (KeyError, ValueError, TypeError) as exc:
            logger.warning("Skipping malformed waypoint row %r: %s", row, exc)
    return waypoints


def _load_default_waypoints(path: Path) -> List[dict]:
    if not path.exists():
        logger.warning("Waypoint file %s not found; starting with an empty path.", path)
        return []
    return _convert_csv_to_waypoints(path.read_text(encoding="utf-8"))


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 1 — I/O ADAPTERS
# ══════════════════════════════════════════════════════════════════════════
def _extract_imu_heading(imu_raw: dict) -> Optional[float]:
    return imu_raw.get("heading", {}).get("deg")


def _extract_rtk_sample(rtk_raw: dict) -> Tuple[Optional[float], Optional[float], int]:
    return rtk_raw.get("lat"), rtk_raw.get("lon"), int(rtk_raw.get("fix_quality") or 0)


class ImuWsClient:
    """Subscribes to 01_IMU; can also push set_north_offset back to it."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, url: str) -> None:
        self._url = url
        self._latest: dict = {}
        self._last_ts: float = 0.0
        self._lock = threading.Lock()
        self._send_queue: "asyncio.Queue[str]" = asyncio.Queue(maxsize=8)

    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    @property
    def last_ts(self) -> float:
        with self._lock:
            return self._last_ts

    async def set_north_offset(self, offset_deg: float) -> None:
        try:
            self._send_queue.put_nowait(json.dumps({"set_north_offset": round(offset_deg, 4)}))
        except asyncio.QueueFull:
            logger.warning("ImuWsClient send queue full, dropping set_north_offset")

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    sender_task = asyncio.create_task(self._sender(ws))
                    try:
                        async for raw in ws:
                            with self._lock:
                                self._latest = json.loads(raw)
                                self._last_ts = time.time()
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
    """Subscribes to 02_RTK. Tracks two ages: any-packet age and fix age."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, url: str) -> None:
        self._url = url
        self._latest: dict = {}
        self._last_ts: float = 0.0          # only updated when fix_quality > 0
        self._last_packet_ts: float = 0.0   # updated on any received packet
        self._lock = threading.Lock()

    @property
    def latest(self) -> dict:
        with self._lock:
            return dict(self._latest)

    @property
    def last_ts(self) -> float:
        with self._lock:
            return self._last_ts

    @property
    def last_packet_ts(self) -> float:
        with self._lock:
            return self._last_packet_ts

    async def run(self) -> None:
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    async for raw in ws:
                        msg = json.loads(raw)
                        now = time.time()
                        with self._lock:
                            self._latest = msg
                            self._last_packet_ts = now
                            if (msg.get("fix_quality") or 0) > 0:
                                self._last_ts = now
            except Exception as exc:
                logger.warning("RtkWsClient connection error: %s", exc)
            await asyncio.sleep(self.RECONNECT_DELAY_S)


class RobotWsClient:
    """Sends drive/lift commands to 04_Robot; drains its broadcasts unread."""

    RECONNECT_DELAY_S = 3.0

    def __init__(self, url: str, send_timeout_s: float) -> None:
        self._url = url
        self._send_timeout_s = send_timeout_s
        self._ws = None
        self._last_sent_ts = 0.0

    @property
    def connected(self) -> bool:
        return self._ws is not None

    async def send(self, linear: float, angular: float) -> None:
        if self._ws is None:
            logger.warning("RobotWsClient: not connected, dropping drive command")
            return
        try:
            await asyncio.wait_for(
                self._ws.send(json.dumps({"type": "joystick", "linear": linear, "angular": angular})),
                timeout=self._send_timeout_s,
            )
            self._last_sent_ts = time.time()
        except Exception as exc:
            logger.warning("RobotWsClient send failed: %s", exc)
            await self._disconnect()

    async def send_lift(self, cmd: str) -> None:
        if self._ws is None:
            logger.warning("RobotWsClient: not connected, dropping lift command")
            return
        try:
            await asyncio.wait_for(
                self._ws.send(json.dumps({"type": "lift_control", "cmd": cmd})),
                timeout=self._send_timeout_s,
            )
        except Exception as exc:
            logger.warning("RobotWsClient send_lift failed: %s", exc)
            await self._disconnect()

    async def _disconnect(self) -> None:
        ws, self._ws = self._ws, None
        if ws is not None:
            try:
                await ws.close()
            except Exception:
                pass

    async def run(self) -> None:
        asyncio.create_task(self._heartbeat_loop())
        while True:
            try:
                async with websockets.connect(self._url) as ws:
                    self._ws = ws
                    try:
                        async for _ in ws:
                            pass  # drain odom/state broadcasts; not needed here
                    finally:
                        self._ws = None
            except Exception as exc:
                logger.warning("RobotWsClient connection error: %s", exc)
            await asyncio.sleep(self.RECONNECT_DELAY_S)

    async def _heartbeat_loop(self) -> None:
        while True:
            await asyncio.sleep(HEARTBEAT_INTERVAL)
            if time.time() - self._last_sent_ts >= HEARTBEAT_INTERVAL:
                await self.send(0.0, 0.0)


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 2 — CORE: sensor timeout detection
# ══════════════════════════════════════════════════════════════════════════
def _sensor_ages(now: float, imu_client: ImuWsClient, rtk_client: RtkWsClient) -> Tuple[float, float, float]:
    imu_ts = imu_client.last_ts if imu_client.last_ts > 0 else now - 9999
    gps_ts = rtk_client.last_ts if rtk_client.last_ts > 0 else now - 9999
    gps_packet_ts = rtk_client.last_packet_ts if rtk_client.last_packet_ts > 0 else now - 9999
    return now - imu_ts, now - gps_ts, now - gps_packet_ts


def _sensors_ok(heading: Optional[float], lat: Optional[float], lon: Optional[float], fix_quality: int,
                 gps_age: float, gps_packet_age: float, imu_age: float) -> bool:
    return (
        heading is not None and lat is not None and lon is not None
        and fix_quality > 0
        and gps_age < GPS_TIMEOUT_S and gps_packet_age < GPS_TIMEOUT_S
        and imu_age < GPS_TIMEOUT_S
    )


def _sensor_block_reason(fix_quality: int, gps_age: float, gps_packet_age: float, imu_age: float) -> str:
    if fix_quality <= 0:
        return "rtk_fix_quality"
    if gps_packet_age >= GPS_TIMEOUT_S:
        return "rtk_packet_age"
    if gps_age >= GPS_TIMEOUT_S:
        return "rtk_fix_age"
    if imu_age >= GPS_TIMEOUT_S:
        return "imu_age"
    return "sensor_unavailable"


def _downsample_path(waypoints: List[dict], max_pts: int = 80) -> List[dict]:
    if len(waypoints) <= max_pts:
        return [{"lat": wp["lat"], "lon": wp["lon"]} for wp in waypoints]
    step = max(1, len(waypoints) // max_pts)
    return [{"lat": wp["lat"], "lon": wp["lon"]} for wp in waypoints[::step]]


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 3 — CORE: state machine
# ══════════════════════════════════════════════════════════════════════════
class AutoNavLoop:
    """State machine: idle -> running <-> paused -> lifting -> arrived.

    Waypoint-advancement, pausing-at-waypoint, and lift-triggering are all
    decided here (not in autonav_algo.py) since they are state-machine
    concerns, not control-law math.
    """

    def __init__(self, imu: ImuWsClient, rtk: RtkWsClient, robot: RobotWsClient,
                 waypoints: List[dict], status_queue: "asyncio.Queue[dict]") -> None:
        self._imu = imu
        self._rtk = rtk
        self._robot = robot
        self._waypoints = waypoints
        self._original_waypoints = list(waypoints)
        self._status_queue = status_queue

        self._state = "idle"
        self._paused_by_timeout = False
        self._wp_idx = 0
        self._speed_ratio = 1.0
        self._manual_linear = 0.0
        self._arrive_counter = 0
        self._waiting_at_wp = False
        self._confirm_advance = False
        self._sensor_block_reason: Optional[str] = None
        self._calib_mark: Optional[dict] = None
        self._calib_offset_applied: Optional[float] = None
        self._pause_mode = "all"
        self._offset_m = 0.0
        self._lift_cmd = ""
        self._lift_end_time = 0.0
        self._lift_up_s = LIFT_UP_DURATION_S
        self._lift_down_s = LIFT_DOWN_DURATION_S

    def reset(self) -> None:
        self._arrive_counter = 0
        self._waiting_at_wp = False
        self._confirm_advance = False
        self._sensor_block_reason = None

    # ---- commands -----------------------------------------------------
    def cmd_start(self) -> None:
        if not self._waypoints:
            logger.warning("cmd_start: no waypoints loaded")
            return
        if self._state == "arrived" or self._wp_idx >= len(self._waypoints):
            self._wp_idx = 0
        self._paused_by_timeout = False
        self.reset()
        self._state = "running"

    def cmd_restart(self) -> None:
        if self._state not in ("idle", "paused", "arrived"):
            logger.warning("cmd_restart: refused in state=%s", self._state)
            return
        self._wp_idx = 0
        self.reset()
        self._state = "running"

    async def cmd_stop(self) -> None:
        if self._state == "lifting":
            await self._robot.send_lift("stop")
        self.reset()
        self._state = "idle"
        self._manual_linear = 0.0
        await self._robot.send(0.0, 0.0)

    def cmd_mark_pos(self) -> str:
        rtk = self._rtk.latest
        lat, lon = rtk.get("lat"), rtk.get("lon")
        if lat is None or lon is None:
            return "no_fix"
        self._calib_mark = {"lat": lat, "lon": lon}
        return "ok"

    async def cmd_calibrate(self) -> str:
        if self._calib_mark is None:
            return "no_mark"
        rtk = self._rtk.latest
        cur_lat, cur_lon = rtk.get("lat"), rtk.get("lon")
        if cur_lat is None or cur_lon is None:
            return "no_fix"
        heading_raw = self._imu.latest.get("heading", {}).get("raw")
        if heading_raw is None:
            return "no_imu"

        offset = algo.compute_calibration_offset(
            cur_lat, cur_lon, self._calib_mark["lat"], self._calib_mark["lon"], heading_raw
        )
        await self._imu.set_north_offset(offset)
        self._calib_offset_applied = offset
        return "ok"

    def calib_status(self) -> dict:
        return {"mark": self._calib_mark, "offset_applied": self._calib_offset_applied}

    def cmd_manual(self, linear: float) -> None:
        if self._state == "idle":
            self._manual_linear = linear

    async def cmd_load_waypoints(self, waypoints: List[dict]) -> None:
        await self.cmd_stop()
        self._original_waypoints = list(waypoints)
        self._waypoints = algo.apply_offset_to_waypoints(waypoints, self._offset_m)
        self._wp_idx = 0

    async def cmd_pause(self) -> None:
        if self._state != "running":
            return
        self._state = "paused"
        self._paused_by_timeout = False
        self._manual_linear = 0.0
        await self._robot.send(0.0, 0.0)

    def cmd_resume(self) -> str:
        """Refuses to resume while sensors/robot link are down (fixes a
        pre-refactor bug where an unconditional resume was immediately
        bounced back to paused by the next control cycle's auto-pause
        check, causing paused/running state to flap within one cycle)."""
        if self._state != "paused":
            return "not_paused"

        now = time.time()
        heading = _extract_imu_heading(self._imu.latest)
        lat, lon, fix_quality = _extract_rtk_sample(self._rtk.latest)
        imu_age, gps_age, gps_packet_age = _sensor_ages(now, self._imu, self._rtk)
        ok = _sensors_ok(heading, lat, lon, fix_quality, gps_age, gps_packet_age, imu_age)
        if not ok or not self._robot.connected:
            self._sensor_block_reason = (
                _sensor_block_reason(fix_quality, gps_age, gps_packet_age, imu_age)
                if not ok else "robot_disconnected"
            )
            return "sensors_not_ready"

        self.reset()
        self._state = "running"
        self._paused_by_timeout = False
        return "ok"

    def cmd_set_speed(self, ratio: float) -> None:
        self._speed_ratio = max(0.0, min(1.0, ratio))

    def cmd_set_offset(self, offset_m: float) -> None:
        self._offset_m = max(-5.0, min(5.0, offset_m))
        self._waypoints = algo.apply_offset_to_waypoints(self._original_waypoints, self._offset_m)

    def set_pause_mode(self, mode: str) -> None:
        if mode in ("all", "type"):
            self._pause_mode = mode

    def confirm_wp(self) -> None:
        self._confirm_advance = True

    def set_lift_duration(self, up_s: Optional[float], down_s: Optional[float]) -> None:
        if up_s is not None:
            self._lift_up_s = max(0.1, up_s)
        if down_s is not None:
            self._lift_down_s = max(0.1, down_s)

    # ---- waypoint progress / arrival -----------------------------------
    def _apply_waypoint_progress(self, linear: float, angular: float, arrived: bool) -> Tuple[float, float, bool, bool]:
        if self._waiting_at_wp:
            if self._confirm_advance:
                self._wp_idx += 1
                self._arrive_counter = 0
                self._confirm_advance = False
                self._waiting_at_wp = False
                return 0.0, 0.0, False, True
            return 0.0, 0.0, False, False

        if not arrived:
            self._arrive_counter = 0
            return linear, angular, False, False

        self._arrive_counter += 1
        if self._arrive_counter < ARRIVE_FRAMES:
            return linear, angular, True, False

        self._arrive_counter = 0
        wp = self._waypoints[self._wp_idx]
        is_last = self._wp_idx >= len(self._waypoints) - 1
        lift = wp.get("lift", "")

        if lift in ("up", "down"):
            self._state = "lifting"
            self._lift_cmd = lift
            duration = self._lift_up_s if lift == "up" else self._lift_down_s
            self._lift_end_time = time.monotonic() + duration
            return 0.0, 0.0, False, False

        if is_last:
            self._state = "arrived"
            logger.info("Arrived at destination")
            return 0.0, 0.0, True, False

        should_pause = self._pause_mode == "all" or wp.get("type") == "pause"
        if should_pause:
            self._waiting_at_wp = True
            return 0.0, 0.0, False, False

        self._wp_idx += 1
        self._arrive_counter = 0
        return linear, angular, False, True

    async def _handle_lifting(self) -> Tuple[float, float, bool]:
        if time.monotonic() < self._lift_end_time:
            await self._robot.send_lift(self._lift_cmd)
            return 0.0, 0.0, False

        await self._robot.send_lift("stop")
        self._lift_cmd = ""
        self._state = "running"

        wp = self._waypoints[self._wp_idx]
        is_last = self._wp_idx >= len(self._waypoints) - 1
        if is_last:
            self._state = "arrived"
        else:
            should_pause = self._pause_mode == "all" or wp.get("type") == "pause"
            if should_pause:
                self._waiting_at_wp = True
            else:
                self._wp_idx += 1
        return 0.0, 0.0, False

    @staticmethod
    def _scaled_robot_command(state: str, linear: float, angular: float,
                               manual_linear: float, speed_ratio: float) -> Tuple[float, float]:
        if state == "idle":
            return manual_linear, 0.0
        if state == "running":
            return linear * speed_ratio, angular * speed_ratio
        return 0.0, 0.0

    def _get_wp_window(self, window: int = 7) -> List[dict]:
        n = len(self._waypoints)
        if n == 0:
            return []
        half = window // 2
        start = max(0, self._wp_idx - half)
        end = min(n, start + window)
        start = max(0, end - window)
        return [
            {"idx": i, "lat": self._waypoints[i]["lat"], "lon": self._waypoints[i]["lon"], "current": i == self._wp_idx}
            for i in range(start, end)
        ]

    # ---- main loop ------------------------------------------------------
    async def run(self) -> None:
        period = 1.0 / max(CONTROL_HZ, 0.5)
        while True:
            try:
                await self._tick()
            except Exception:
                logger.exception("AutoNavLoop tick failed")
            await asyncio.sleep(period)

    async def _tick(self) -> None:
        now = time.time()
        imu_raw = self._imu.latest
        rtk_raw = self._rtk.latest
        heading = _extract_imu_heading(imu_raw)
        lat, lon, fix_quality = _extract_rtk_sample(rtk_raw)
        imu_age, gps_age, gps_packet_age = _sensor_ages(now, self._imu, self._rtk)
        sensors_ok = _sensors_ok(heading, lat, lon, fix_quality, gps_age, gps_packet_age, imu_age)
        robot_ok = self._robot.connected

        if self._state == "running" and (not sensors_ok or not robot_ok):
            self._state = "paused"
            self._paused_by_timeout = True
            self._sensor_block_reason = (
                "robot_disconnected" if not robot_ok
                else _sensor_block_reason(fix_quality, gps_age, gps_packet_age, imu_age)
            )
            logger.warning("Auto-pausing: sensors_ok=%s robot_ok=%s reason=%s",
                            sensors_ok, robot_ok, self._sensor_block_reason)

        elif self._state == "paused" and self._paused_by_timeout and sensors_ok and robot_ok:
            self._arrive_counter = 0
            self._confirm_advance = False
            self._state = "running"
            self._paused_by_timeout = False
            self._sensor_block_reason = None
            logger.info("Sensors/robot link recovered — auto-resuming")

        linear = angular = 0.0
        dist_m = bearing_deg = 0.0
        dt_s = 1.0 / max(CONTROL_HZ, 0.5)

        if self._state == "running" and sensors_ok:
            result = algo.compute(lat, lon, heading, self._waypoints, self._wp_idx, dt_s)
            dist_m, bearing_deg = result.dist_m, result.bearing_deg
            linear, angular, arrived, _advanced = self._apply_waypoint_progress(
                result.linear, result.angular, result.arrived
            )
        elif self._state == "lifting":
            linear, angular, _ = await self._handle_lifting()

        send_linear, send_angular = self._scaled_robot_command(
            self._state, linear, angular, self._manual_linear, self._speed_ratio
        )
        await self._robot.send(round(send_linear, 3), round(send_angular, 3))

        status = _build_autonav_status(
            self, heading, lat, lon, gps_age, gps_packet_age, imu_age,
            linear, angular, dist_m, bearing_deg, imu_raw, rtk_raw,
        )
        try:
            self._status_queue.put_nowait(status)
        except asyncio.QueueFull:
            pass


def _build_autonav_status(loop: AutoNavLoop, heading: Optional[float], lat: Optional[float], lon: Optional[float],
                           gps_age: float, gps_packet_age: float, imu_age: float,
                           linear: float, angular: float, dist_m: float, bearing_deg: float,
                           imu_raw: dict, rtk_raw: dict) -> dict:
    heading_val = heading if heading is not None else 0.0
    bearing_error = (bearing_deg - heading_val + 180) % 360 - 180
    return {
        "type": "autonav_status", "version": WS_MSG_VERSION,
        "state": loop._state,
        "current_wp_idx": loop._wp_idx,
        "total_wp": len(loop._waypoints),
        "heading_deg": heading,
        "target_bearing_deg": round(bearing_deg, 1),
        "bearing_error_deg": round(bearing_error, 1),
        "dist_to_wp_m": round(dist_m, 2),
        "linear": round(linear, 3),
        "angular": round(angular, 3),
        "gps_age_s": round(gps_age, 2),
        "gps_packet_age_s": round(gps_packet_age, 2),
        "gps_fix_quality": int(rtk_raw.get("fix_quality") or 0),
        "sensor_block_reason": loop._sensor_block_reason,
        "robot_connected": loop._robot.connected,
        "imu_age_s": round(imu_age, 2),
        "speed_ratio": loop._speed_ratio,
        "manual_speed": MANUAL_SPEED,
        "calib": loop.calib_status(),
        "waypoints_window": loop._get_wp_window(),
        "waiting_at_wp": loop._waiting_at_wp,
        "waiting_wp_idx": loop._wp_idx if loop._waiting_at_wp else None,
        "pause_mode": loop._pause_mode,
        "lift_cmd": loop._lift_cmd if loop._state == "lifting" else "",
        "lift_remaining_s": (
            max(0.0, round(loop._lift_end_time - time.monotonic(), 1)) if loop._state == "lifting" else 0.0
        ),
        "lift_up_s": loop._lift_up_s,
        "lift_down_s": loop._lift_down_s,
        "offset_m": loop._offset_m,
        "path_original": _downsample_path(loop._original_waypoints),
        "path_offset": _downsample_path(loop._waypoints),
        "robot_lat": lat,
        "robot_lon": lon,
        "imu_raw": imu_raw,
        "rtk_raw": rtk_raw,
    }


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ══════════════════════════════════════════════════════════════════════════
class AutoNavBridge:
    def __init__(self, http_port: int, static_dir: Path) -> None:
        self._http_port = http_port
        self._ws_port = derive_ws_port(http_port)
        self._static_dir = static_dir
        self._nav_loop: Optional[AutoNavLoop] = None

    def run(self) -> None:
        if self._static_dir.exists():
            StaticFileServer(
                self._static_dir, self._http_port,
                extra_routes={"/export_csv": self._export_csv_route},
                no_cache=True,
            ).start()
        else:
            logger.warning("Static dir %s not found; HTTP page disabled.", self._static_dir)

        threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{self._http_port}")).start()

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("Stopped")

    def _export_csv_route(self) -> Tuple[bytes, str]:
        body = self._generate_export_csv().encode("utf-8")
        return body, "text/csv; charset=utf-8"

    def _generate_export_csv(self) -> str:
        lines = ["index,orig_lat,orig_lon,offset_lat,offset_lon,type,lift"]
        if self._nav_loop is not None:
            orig = self._nav_loop._original_waypoints
            offset = self._nav_loop._waypoints
            for i, (o, w) in enumerate(zip(orig, offset)):
                lines.append(
                    f"{i},{o['lat']:.8f},{o['lon']:.8f},{w['lat']:.8f},{w['lon']:.8f},"
                    f"{o.get('type', '')},{o.get('lift', '')}"
                )
        return "\n".join(lines) + "\n"

    async def _run_async(self) -> None:
        waypoints = _load_default_waypoints(PATH_FILE)

        imu_client = ImuWsClient(IMU_WS_URL)
        rtk_client = RtkWsClient(RTK_WS_URL)
        robot_client = RobotWsClient(ROBOT_WS_URL, ROBOT_SEND_TIMEOUT_S)
        status_queue: "asyncio.Queue[dict]" = asyncio.Queue(maxsize=10)

        self._nav_loop = AutoNavLoop(imu_client, rtk_client, robot_client, waypoints, status_queue)

        ws_server = BroadcastWsServer("0.0.0.0", self._ws_port, on_client_message=self._handle_message)

        async def _drain_queue() -> None:
            while True:
                status = await status_queue.get()
                await ws_server.broadcast(status)

        await asyncio.gather(
            imu_client.run(), rtk_client.run(), robot_client.run(),
            self._nav_loop.run(), _drain_queue(), ws_server.serve(),
        )

    async def _handle_message(self, _websocket, raw: str) -> Optional[str]:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return None
        loop = self._nav_loop
        if loop is None:
            return None
        t = msg.get("type", "")

        if t == "start":
            loop.cmd_start()
        elif t == "restart":
            loop.cmd_restart()
        elif t == "stop":
            await loop.cmd_stop()
        elif t == "pause":
            await loop.cmd_pause()
        elif t == "resume":
            result = loop.cmd_resume()
            if result != "ok":
                return json.dumps({
                    "type": "resume_result", "version": WS_MSG_VERSION,
                    "ok": False, "reason": result,
                })
        elif t == "set_speed":
            loop.cmd_set_speed(float(msg.get("ratio", 1.0)))
        elif t == "load_csv":
            waypoints = _convert_csv_to_waypoints(msg.get("content", ""))
            await loop.cmd_load_waypoints(waypoints)
        elif t == "gen_path":
            await self._handle_gen_path(msg)
        elif t == "calib_mark":
            loop.cmd_mark_pos()
        elif t == "calib_apply":
            await loop.cmd_calibrate()
        elif t == "manual_drive":
            loop.cmd_manual(float(msg.get("linear", 0.0)))
        elif t == "confirm_wp":
            loop.confirm_wp()
        elif t == "set_pause_mode":
            loop.set_pause_mode(str(msg.get("mode", "all")))
        elif t == "lift_control":
            await self._nav_loop._robot.send_lift(str(msg.get("cmd", "stop")))
        elif t == "set_lift_duration":
            loop.set_lift_duration(msg.get("up_s"), msg.get("down_s"))
        elif t == "set_offset":
            loop.cmd_set_offset(float(msg.get("offset_m", 0.0)))

    async def _handle_gen_path(self, msg: dict) -> None:
        try:
            scripts_dir = str(Path(__file__).parent / "scripts")
            if scripts_dir not in sys.path:
                sys.path.insert(0, scripts_dir)
            convert_module = importlib.import_module("convert_offsets_to_latlon")
            offset_csv = Path(__file__).parent / "scripts" / "offset.csv"
            convert_module.convert(offset_csv, PATH_FILE, float(msg["lat"]), float(msg["lon"]))
            waypoints = _load_default_waypoints(PATH_FILE)
            if self._nav_loop is not None:
                await self._nav_loop.cmd_load_waypoints(waypoints)
        except Exception:
            logger.exception("gen_path failed")


if __name__ == "__main__":
    AutoNavBridge(
        http_port=DEFAULT_HTTP_PORT,
        static_dir=Path(__file__).parent / "web_static",
    ).run()
