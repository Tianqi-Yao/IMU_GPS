"""autonav_algo.py — pure navigation math for 05_AutoNav. No I/O, no state.

This is a proportional (P-only) heading controller with a hard dead zone —
NOT a PID (no integral/derivative terms) and NOT Pure Pursuit (no lookahead
point selection; it always aims directly at the current waypoint). This is
a deliberate, validated simplification carried forward from the v2 tuning
of the original project; do not add PID/Pure Pursuit/haversine/moving-
average filtering back in without an explicit product decision to do so.

fast_distance_m/fast_bearing use an equirectangular (flat-earth) projection,
not a great-circle (haversine) formula — accurate enough at field scale
(waypoints tens to low hundreds of meters apart), not for long distances.

Everything here is a pure function/dataclass: no sockets, no serial, no
threads — safe to unit test with plain floats and dicts.
"""

import math
from dataclasses import dataclass
from typing import List, Optional

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    import config as _cfg
except ImportError:
    _cfg = None


def _c(attr: str, default):
    return getattr(_cfg, attr, default) if _cfg else default


REACH_TOL_M = _c("AUTONAV_REACH_TOL_M", 0.5)
SAFE_STOP_RADIUS_M = _c("AUTONAV_DECEL_RADIUS_M", 1.5)
MAX_LINEAR = _c("AUTONAV_MAX_LINEAR_VEL", 1.0)
MIN_LINEAR = _c("AUTONAV_MIN_LINEAR_VEL", 0.1)
MAX_ANGULAR = _c("AUTONAV_MAX_ANGULAR_VEL", 1.0)
KP = _c("AUTONAV_PID_KP", 0.02)
DEAD_ZONE_DEG = _c("AUTONAV_DEAD_ZONE_DEG", 3.0)
TURN_THRESHOLD_DEG = _c("AUTONAV_TURN_THRESHOLD_DEG", 5.0)

# Not a config.py knob: only relevant if the chassis' rotation sense differs
# from this codebase's convention. Flip to +1 if the robot turns the wrong way.
ANGULAR_SIGN = -1

_METERS_PER_DEGREE_LAT = 111_320
_MAX_MITER = 3.0


@dataclass
class ComputeResult:
    linear: float
    angular: float
    arrived: bool
    dist_m: float
    bearing_deg: float


def fast_distance_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Equirectangular-projection planar distance. Field-scale only (<=~10km)."""
    k = _METERS_PER_DEGREE_LAT
    mean_lat = math.radians((lat1 + lat2) / 2)
    dx = (lon2 - lon1) * k * math.cos(mean_lat)
    dy = (lat2 - lat1) * k
    return math.hypot(dx, dy)


def fast_bearing(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Planar bearing in degrees [0, 360), 0=north. Field-scale only."""
    mean_lat = math.radians((lat1 + lat2) / 2)
    dx = (lon2 - lon1) * math.cos(mean_lat)
    dy = lat2 - lat1
    return (math.degrees(math.atan2(dx, dy)) + 360) % 360


def compute(lat: Optional[float], lon: Optional[float], heading_deg: Optional[float],
            waypoints: List[dict], wp_idx: int, dt_s: float) -> ComputeResult:
    """P-controller + hard dead zone + two-stage deceleration + turn-in-place.

    Returns (linear, angular, arrived, dist_m, bearing_deg) via ComputeResult.
    Waypoint-index advancement, pausing, and lifting are NOT this function's
    job — that state-machine logic lives in autonav_bridge.py.
    """
    if lat is None or lon is None or heading_deg is None:
        return ComputeResult(0.0, 0.0, False, 0.0, 0.0)
    if wp_idx >= len(waypoints):
        return ComputeResult(0.0, 0.0, True, 0.0, 0.0)

    wp = waypoints[wp_idx]
    dist_m = fast_distance_m(lat, lon, wp["lat"], wp["lon"])
    bearing_deg = fast_bearing(lat, lon, wp["lat"], wp["lon"])

    error = (bearing_deg - heading_deg + 180) % 360 - 180

    if abs(error) < DEAD_ZONE_DEG:
        angular = 0.0
    else:
        angular = ANGULAR_SIGN * KP * error
        angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

    if dist_m < SAFE_STOP_RADIUS_M:
        linear = max(MIN_LINEAR, MAX_LINEAR * (dist_m / SAFE_STOP_RADIUS_M))
    else:
        linear = MAX_LINEAR

    if TURN_THRESHOLD_DEG > 0 and abs(error) > TURN_THRESHOLD_DEG:
        linear = 0.0

    arrived = dist_m < REACH_TOL_M
    return ComputeResult(linear, angular, arrived, dist_m, bearing_deg)


def compute_calibration_offset(cur_lat: float, cur_lon: float,
                                mark_lat: float, mark_lon: float, heading_raw: float) -> float:
    """North-offset (degrees) so that heading_raw + offset points at the mark.

    Used for in-field compass calibration: stand at (cur_lat, cur_lon)
    facing a known landmark at (mark_lat, mark_lon), and this returns the
    offset to hand to 01_IMU's set_north_offset control message.
    """
    bearing = fast_bearing(cur_lat, cur_lon, mark_lat, mark_lon)
    return (bearing - heading_raw) % 360.0


def apply_offset_to_waypoints(waypoints: List[dict], offset_m: float) -> List[dict]:
    """Shift a waypoint path laterally by offset_m (miter join at corners)."""
    n = len(waypoints)
    if n < 2 or offset_m == 0.0:
        return [dict(wp) for wp in waypoints]

    def perp_vec(lat1, lon1, lat2, lon2):
        bearing = fast_bearing(lat1, lon1, lat2, lon2)
        rad = math.radians(bearing + 90)
        return math.cos(rad), math.sin(rad)

    result = []
    for i, wp in enumerate(waypoints):
        if i == 0:
            px, py = perp_vec(wp["lat"], wp["lon"], waypoints[1]["lat"], waypoints[1]["lon"])
        elif i == n - 1:
            px, py = perp_vec(waypoints[i - 1]["lat"], waypoints[i - 1]["lon"], wp["lat"], wp["lon"])
        else:
            px1, py1 = perp_vec(waypoints[i - 1]["lat"], waypoints[i - 1]["lon"], wp["lat"], wp["lon"])
            px2, py2 = perp_vec(wp["lat"], wp["lon"], waypoints[i + 1]["lat"], waypoints[i + 1]["lon"])
            sx, sy = px1 + px2, py1 + py2
            mag = math.hypot(sx, sy)
            if mag < 1e-6:
                px, py = px1, py1
            else:
                f = min(2.0 / mag, _MAX_MITER) / mag
                px, py = sx * f, sy * f

        k = _METERS_PER_DEGREE_LAT
        new_lat = wp["lat"] + offset_m * px / k
        new_lon = wp["lon"] + offset_m * py / (k * math.cos(math.radians(wp["lat"])))
        new_wp = dict(wp)
        new_wp["lat"] = new_lat
        new_wp["lon"] = new_lon
        result.append(new_wp)

    return result
