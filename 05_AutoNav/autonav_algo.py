import math
import rootutils
ROOT = rootutils.setup_root(__file__, indicator=".git", pythonpath=True)

# ── Load config ───────────────────────────────────────────────────────────────
try:
    import config as _cfg
except ImportError:
    _cfg = None

def _c(attr, default):
    return getattr(_cfg, attr, default) if _cfg else default

# ── Parameters (all overridable in config.py) ─────────────────────────────────
REACH_TOL_M       = _c("AUTONAV_REACH_TOL_M",       0.5)
SAFE_STOP_RADIUS_M    = _c("AUTONAV_DECEL_RADIUS_M",     1.5)

MAX_LINEAR        = _c("AUTONAV_MAX_LINEAR_VEL",     1.0)
MIN_LINEAR        = _c("AUTONAV_MIN_LINEAR_VEL",     0.1)
MAX_ANGULAR       = _c("AUTONAV_MAX_ANGULAR_VEL",    1.0)

KP                = _c("AUTONAV_PID_KP",             0.02)

DEAD_ZONE_DEG     = _c("AUTONAV_DEAD_ZONE_DEG",      3.0)
TURN_THRESHOLD_DEG = _c("AUTONAV_TURN_THRESHOLD_DEG",  5.0)

ANGULAR_SIGN      = -1  # 如果机器人转向方向相反，就改成 +1。

# ── (no algorithm-internal state) ───────────────────────────────────────────
# This module implements a stateless `compute()`; arrival/waiting logic
# is intended to be handled by the bridge (`autonav_bridge.AutoNavLoop`).


# ── Geometry ──────────────────────────────────────────────────────────────────

def _fast_distance_m(lat1, lon1, lat2, lon2) -> float:
    """快速平面距离近似，单位:米。适合短距离(<= 10km)。"""
    k = 111_320  # meters per degree latitude
    mean_lat = math.radians((lat1 + lat2) / 2)
    dx = (lon2 - lon1) * k * math.cos(mean_lat)
    dy = (lat2 - lat1) * k
    return math.hypot(dx, dy)


def _fast_bearing(lat1, lon1, lat2, lon2) -> float:
    """快速方位角计算，单位是度，范围 [0, 360)。平面近似。"""
    mean_lat = math.radians((lat1 + lat2) / 2)
    dx = (lon2 - lon1) * math.cos(mean_lat)
    dy = (lat2 - lat1)
    bearing = (math.degrees(math.atan2(dx, dy)) + 360) % 360
    # Return the bearing from point A to point B in degrees [0, 360). Planar approximation (valid for short distances)
    return bearing


# ── Main compute ──────────────────────────────────────────────────────────────

def compute(lat, lon, heading_deg, waypoints, wp_idx, dt_s) -> tuple:
    """
    Returns: (linear m/s, angular rad/s, arrived: bool)
    """

    # 如果没有当前坐标或航向信息，无法导航，直接返回零速度。
    if lat is None or lon is None or heading_deg is None:
        return 0.0, 0.0, False

    # 安全：如果索引越界或无航点，直接报告已到达。
    if wp_idx >= len(waypoints):
        return 0.0, 0.0, True

    # 当前航点与距离
    wp = waypoints[wp_idx]
    dis_to_wp = _fast_distance_m(lat, lon, wp["lat"], wp["lon"])

    # 直接追踪当前航点的方位角。
    target_bearing = _fast_bearing(lat, lon, wp["lat"], wp["lon"])

    # 把方位误差归一化到 [-180, 180]，再把小误差抹平，避免机器人已经对准时还抖动。
    error = (target_bearing - heading_deg + 180) % 360 - 180
    # error: [-180, 180], degrees

    # angular: rad/s or robot command unit
    if abs(error) < DEAD_ZONE_DEG:
        angular = 0.0
    else:
        angular = KP * error
        angular = ANGULAR_SIGN * angular
        angular = max(-MAX_ANGULAR, min(MAX_ANGULAR, angular))

    # 接近终点时限制线速度，方便平稳停下。
    if dis_to_wp < SAFE_STOP_RADIUS_M:
        linear = max(MIN_LINEAR, MAX_LINEAR * (dis_to_wp / SAFE_STOP_RADIUS_M))
    else:
        linear = MAX_LINEAR

    # 当航向误差较大时，先停止前进，原地转向。
    if TURN_THRESHOLD_DEG > 0 and abs(error) > TURN_THRESHOLD_DEG:
        linear = 0.0

    # 到达当前航点，标记 arrived=True，外层会进入 _waiting_at_wp 状态
    arrived = dis_to_wp < REACH_TOL_M

    return linear, angular, arrived


def apply_offset_to_waypoints(waypoints: list, offset_m: float) -> list:
    """Apply lateral offset (m) perpendicular to each path segment.
    Positive = right of travel direction, negative = left.
    Uses miter join at corners so the offset distance stays consistent.
    Preserves all extra fields (type, lift, etc.).
    """
    n = len(waypoints)
    if not waypoints or n < 2 or offset_m == 0.0:
        return [dict(wp) for wp in waypoints]

    def _perp_vec(lat1, lon1, lat2, lon2):
        r = math.radians((_fast_bearing(lat1, lon1, lat2, lon2) + 90) % 360)
        return math.cos(r), math.sin(r)  # (north-component, east-component)

    MAX_MITER = 3.0  # cap miter scale for corners sharper than ~38 deg

    result = []
    for i, wp in enumerate(waypoints):
        if i == 0:
            px, py = _perp_vec(wp["lat"], wp["lon"],
                                waypoints[1]["lat"], waypoints[1]["lon"])
        elif i == n - 1:
            px, py = _perp_vec(waypoints[i - 1]["lat"], waypoints[i - 1]["lon"],
                                wp["lat"], wp["lon"])
        else:
            # Miter join: bisector of incoming and outgoing perpendiculars,
            # scaled so that the offset distance from both segments equals offset_m.
            px1, py1 = _perp_vec(waypoints[i - 1]["lat"], waypoints[i - 1]["lon"],
                                  wp["lat"], wp["lon"])
            px2, py2 = _perp_vec(wp["lat"], wp["lon"],
                                  waypoints[i + 1]["lat"], waypoints[i + 1]["lon"])
            sx, sy = px1 + px2, py1 + py2
            mag = math.hypot(sx, sy)
            if mag < 1e-6:
                px, py = px1, py1
            else:
                # miter_len = 2/mag; scale = miter_len/mag so (sx,sy)*scale has that length
                f = min(2.0 / mag, MAX_MITER) / mag
                px, py = sx * f, sy * f

        k = 111_320
        new_wp = dict(wp)
        new_wp["lat"] = wp["lat"] + offset_m * px / k
        new_wp["lon"] = wp["lon"] + offset_m * py / (k * math.cos(math.radians(wp["lat"])))
        result.append(new_wp)
    return result
