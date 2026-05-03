"""
Figure 1: GPS trajectory map — actual vs planned path
Figure 2: Cross-track error along actual trajectory
Outputs: gps_path.png, gps_path_error.png
"""
import csv
import json
import math
from math import hypot
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np

DATA = Path(__file__).parent.parent / "data_log" / "robot_raw_20260502_200501.jsonl"
AUTONAV = Path(__file__).parent.parent / "data_log" / "autonav_raw_20260502_200458.jsonl"
PATH_CSV = Path(__file__).parent.parent / "path.csv"
OUT1 = Path(__file__).parent / "gps_path.png"
OUT2 = Path(__file__).parent / "gps_path_error.png"

# Planned path offset (m)
# Set to None for auto-align to actual track start
# Or set to (offset_x, offset_y) for manual offset
# PLAN_OFFSET = None  
PLAN_OFFSET = (61.21667380173043, 133.10829470372498)  # (offset_x, offset_y) 单位 m


# --- load actual RTK track ---
records = [json.loads(l) for l in DATA.read_text().splitlines()]
rtk = [r for r in records if r["type"] == "rtk"]

lats = np.array([r["lat"] for r in rtk])
lons = np.array([r["lon"] for r in rtk])
hdops = np.array([r["hdop"] for r in rtk])
ts = np.array([r["log_recv_ts"] for r in rtk])

lat0, lon0 = lats[0], lons[0]
k = 111_320.0
cos_lat = math.cos(math.radians(lat0))

def to_enu(lat_arr, lon_arr):
    xs = (lon_arr - lon0) * k * cos_lat
    ys = (lat_arr - lat0) * k
    return xs, ys

xs, ys = to_enu(lats, lons)

# --- crop: use wp_idx 0->1 transition = robot just arrived at plan[0] ---
nav = [json.loads(l) for l in AUTONAV.read_text().splitlines()]
task_start_ts = None
prev_wp = None
for r in nav:
    if r["state"] != "running" or not r.get("rtk_raw"):
        continue
    wp = r["current_wp_idx"]
    if prev_wp == 0 and wp == 1:
        task_start_ts = r["rtk_raw"]["server_ts"]
        break
    prev_wp = wp
crop_idx = int(np.searchsorted(ts, task_start_ts))
xs, ys, hdops, ts = xs[crop_idx:], ys[crop_idx:], hdops[crop_idx:], ts[crop_idx:]

# --- load planned path and align to task start ---
plan_lats, plan_lons = [], []
with open(PATH_CSV, newline="") as f:
    reader = csv.DictReader(f)
    for row in reader:
        plan_lats.append(float(row["lat"]))
        plan_lons.append(float(row["lon"]))

plan_lats = np.array(plan_lats)
plan_lons = np.array(plan_lons)
_pxs, _pys = to_enu(plan_lats, plan_lons)
print(_pxs[0], _pys[0])

# Apply planned path offset
if PLAN_OFFSET is None:
    # Auto-align: shift plan so its start coincides with task start in actual track
    offset_x = xs[0] - _pxs[0]
    offset_y = ys[0] - _pys[0]
else:
    offset_x, offset_y = PLAN_OFFSET

pxs = _pxs + offset_x
pys = _pys + offset_y

# --- cross-track error ---
def point_to_segment_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    if dx == 0 and dy == 0:
        return hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)))
    return hypot(px - (ax + t * dx), py - (ay + t * dy))

errors = []
for px, py in zip(xs, ys):
    min_d = min(
        point_to_segment_dist(px, py, pxs[i], pys[i], pxs[i + 1], pys[i + 1])
        for i in range(len(pxs) - 1)
    )
    errors.append(min_d)
errors = np.array(errors)

cumdist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(xs), np.diff(ys)))])
total_dist = cumdist[-1]
duration = ts[-1] - ts[0]

# ── Figure 1: path comparison ──────────────────────────────────────────────
fig1, ax1 = plt.subplots(figsize=(6.2, 8.4), constrained_layout=True)

ax1.plot(pxs, pys, color="darkorange", lw=1.4, ls="--", zorder=2, label="Planned path")

sc = ax1.scatter(xs, ys, c=hdops, cmap="RdYlGn_r", vmin=1.0, vmax=2.0,
                 s=12, linewidths=0, zorder=3)
cbar = fig1.colorbar(sc, ax=ax1, pad=0.02, shrink=0.92)
cbar.set_label("HDOP", fontsize=10)

ax1.plot(xs[0], ys[0], "^", color="tab:green", ms=10, zorder=5, label="Start")
ax1.plot(xs[-1], ys[-1], "s", color="tab:red", ms=10, zorder=5, label="End")
ax1.plot(xs, ys, color="steelblue", lw=0.8, alpha=0.5, zorder=2)

ax1.margins(x=0.12, y=0.05)
ax1.set_aspect("equal", adjustable="datalim")
ax1.set_anchor("C")
ax1.set_xlabel("East (m)", fontsize=11)
ax1.set_ylabel("North (m)", fontsize=11)
ax1.set_title(
    f"RTK GPS Trajectory  |  {len(rtk)} fixes  |  {total_dist:.1f} m  |  {duration:.0f} s",
    fontsize=11,
)
ax1.legend(fontsize=10, loc="lower left", frameon=True, framealpha=0.9)
ax1.grid(True, alpha=0.3)

fig1.savefig(OUT1, dpi=150, bbox_inches="tight", pad_inches=0.12)
print(f"Saved: {OUT1}")

# ── Figure 2: cross-track error ────────────────────────────────────────────
fig2, ax2 = plt.subplots(figsize=(8, 3.8), constrained_layout=True)

ax2.plot(cumdist, errors, color="steelblue", lw=0.9, alpha=0.8)
ax2.fill_between(cumdist, errors, alpha=0.2, color="steelblue")

mean_e   = errors.mean()
std_e    = errors.std()
median_e = np.median(errors)
max_e    = errors.max()

ax2.axhline(mean_e,   color="tab:red",   lw=1.2, ls="--", label=f"Mean   {mean_e:.2f} m")
ax2.axhline(median_e, color="tab:green", lw=1.0, ls="-.", label=f"Median {median_e:.2f} m")
ax2.axhline(max_e,    color="tab:orange",lw=1.0, ls=":",  label=f"Max    {max_e:.2f} m")

ax2.set_xlabel("Distance along track (m)", fontsize=11)
ax2.set_ylabel("Cross-track error (m)", fontsize=11)
ax2.set_title(
    f"Cross-track Error — Actual vs Planned Path  |  std = {std_e:.3f} m",
    fontsize=11,
)
ax2.legend(
    fontsize=9, frameon=True, framealpha=0.9,
    loc="center left", bbox_to_anchor=(1.01, 0.5),
)
ax2.grid(True, alpha=0.3)

fig2.savefig(OUT2, dpi=150, bbox_inches="tight", pad_inches=0.12)
print(f"Saved: {OUT2}")
