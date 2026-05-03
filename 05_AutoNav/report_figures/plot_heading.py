"""
Figure 3: IMU heading over time + GPS track_deg comparison
- IMU heading (deg, 0-360)
- RTK ground track bearing (resampled to IMU timestamps)
- Highlights heading error (IMU - GPS track)
Output: heading_comparison.png
"""
import json
import numpy as np
from pathlib import Path
import matplotlib.pyplot as plt

DATA = Path(__file__).parent.parent / "data_log" / "robot_raw_20260502_200501.jsonl"
OUT = Path(__file__).parent / "heading_comparison.png"

# Minimum RTK speed (knots) to include a fix — below this track_deg is noise.
MIN_SPEED_KT = 0.3

records = [json.loads(l) for l in DATA.read_text().splitlines()]
imu_recs = [r for r in records if r["type"] == "imu"]
rtk_recs = [
    r for r in records
    if r["type"] == "rtk"
    and r.get("track_deg") is not None
    and (r.get("speed_knots") or 0) >= MIN_SPEED_KT
]

ts0 = records[0]["log_recv_ts"]

imu_t = np.array([r["log_recv_ts"] - ts0 for r in imu_recs])
imu_h = np.array([r["heading"]["deg"] for r in imu_recs])

rtk_t = np.array([r["log_recv_ts"] - ts0 for r in rtk_recs])
rtk_track = np.array([r["track_deg"] for r in rtk_recs])

# Circular interpolation via unit circle — avoids 0/360 wrap artifacts
rtk_cx = np.interp(imu_t, rtk_t, np.cos(np.deg2rad(rtk_track)))
rtk_cy = np.interp(imu_t, rtk_t, np.sin(np.deg2rad(rtk_track)))
rtk_interp = np.rad2deg(np.arctan2(rtk_cy, rtk_cx)) % 360

# Circular error in [-180, 180]
raw_err = (imu_h - rtk_interp + 180) % 360 - 180
rmse = np.sqrt(np.mean(raw_err**2))

def unwrap_deg(a):
    out = np.empty_like(a, dtype=float)
    out[0] = a[0]
    for i in range(1, len(a)):
        out[i] = out[i - 1] + (a[i] - a[i - 1] + 180) % 360 - 180
    return out

# Unwrap IMU independently.
imu_h_uw = unwrap_deg(imu_h)

# Unwrap RTK, then at each point snap it to the nearest 360° multiple of
# the IMU value so the two stay in the same "cycle" and don't diverge.
_rtk_uw = unwrap_deg(rtk_interp)
_rtk_uw += imu_h_uw[0] - _rtk_uw[0]          # align starting value
n = np.round((imu_h_uw - _rtk_uw) / 360)      # how many full turns they differ
rtk_uw = _rtk_uw + n * 360                    # snap to IMU cycle

fig, axes = plt.subplots(2, 1, figsize=(11, 6.8), sharex=True)

# Top: unwrapped — continuous, no 0/360 jumps
axes[0].plot(imu_t, imu_h_uw, lw=1.0, color="#1f4e79", label="IMU heading")
axes[0].plot(imu_t, rtk_uw, lw=1.0, color="#d97706", alpha=0.9, label="RTK ground track")
axes[0].set_ylabel("Unwrapped heading (°)", fontsize=10)
axes[0].legend(fontsize=9, loc="upper right")
axes[0].grid(True, alpha=0.3)

# Bottom: circular difference
axes[1].plot(imu_t, raw_err, lw=0.9, color="crimson", label="Heading error (IMU − track)")
axes[1].axhline(0, color="gray", lw=0.5, ls="--")
axes[1].fill_between(imu_t, raw_err, 0, alpha=0.15, color="crimson")
axes[1].set_ylabel("Error (°)", fontsize=10)
axes[1].set_xlabel("Time (s)", fontsize=10)
axes[1].legend(fontsize=9, loc="upper right")
axes[1].grid(True, alpha=0.3)

fig.suptitle(f"IMU Heading vs RTK Ground Track  |  RMSE = {rmse:.1f}°", fontsize=12)
fig.tight_layout()
fig.savefig(OUT, dpi=150, bbox_inches="tight", pad_inches=0.12)
print(f"Saved: {OUT}  (RMSE = {rmse:.2f}°)")
