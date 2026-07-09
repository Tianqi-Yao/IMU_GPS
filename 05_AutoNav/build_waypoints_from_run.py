"""
build_waypoints_from_run.py — Convert a recorded manual-drive run into an AutoNav waypoint CSV.

Data flow:
    04_Robot/data_log/run_<ts>.jsonl (Recorder output; rtk/imu/odom rows)
        --> filter type=="rtk", fix_quality >= WAYPOINT_MIN_FIX_QUALITY
        --> greedy distance downsample (>= WAYPOINT_MIN_DISTANCE_M apart)
        --> 05_AutoNav/data_log/path_<run_ts>_<density>m.csv  (lat,lon,type,lift)

This is an offline, one-shot conversion tool — it does not connect to any
websocket and is not a long-running bridge. Re-run it with a different
WAYPOINT_MIN_DISTANCE_M (in config.py) to regenerate a denser/sparser path
from the same recorded run, with no need to re-drive the field test.

Usage:
    1. Record a manual run: 04_Robot web UI -> press REC before driving,
       stop it after reaching the end of the planned route.
    2. python build_waypoints_from_run.py
    3. Load the generated CSV via the AutoNav dashboard "LOAD CSV" button,
       or copy/rename it to path.csv and restart autonav_bridge.py.
"""

from __future__ import annotations

import rootutils
ROOT = rootutils.setup_root(__file__, indicator=".git", pythonpath=True)
try:
    import config as _cfg
except ImportError:
    _cfg = None

import csv
import json
from dataclasses import dataclass
from pathlib import Path

import autonav_algo as algo

# ── Parameters (edit here to reconfigure; no CLI args by convention) ─────────
RUN_FILE: Path | None = None  # None -> auto-pick newest 04_Robot/data_log/run_*.jsonl

ROBOT_DATA_LOG_DIR = Path(__file__).parent.parent / "04_Robot" / "data_log"
OUTPUT_DIR = Path(__file__).parent / "data_log"

MIN_DISTANCE_M  = _cfg.WAYPOINT_MIN_DISTANCE_M  if _cfg else 1.0
MIN_FIX_QUALITY = _cfg.WAYPOINT_MIN_FIX_QUALITY if _cfg else 1


@dataclass
class RtkSample:
    rec_ts: float
    lat: float
    lon: float
    fix_quality: int


def _find_latest_run_file() -> Path:
    candidates = sorted(ROBOT_DATA_LOG_DIR.glob("run_*.jsonl"))
    if not candidates:
        raise FileNotFoundError(
            f"No run_*.jsonl found in {ROBOT_DATA_LOG_DIR}. "
            "Record a manual run first (04_Robot web UI REC button)."
        )
    return candidates[-1]


def _load_rtk_samples(path: Path) -> list[RtkSample]:
    samples: list[RtkSample] = []
    with open(path, encoding="utf-8") as f:
        for line_no, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                print(f"Skipping malformed JSON on line {line_no}")
                continue
            if row.get("type") != "rtk":
                continue
            lat, lon = row.get("lat"), row.get("lon")
            fix_quality = int(row.get("fix_quality", 0) or 0)
            if lat is None or lon is None or fix_quality < MIN_FIX_QUALITY:
                continue
            samples.append(RtkSample(float(row.get("rec_ts", 0.0)), float(lat), float(lon), fix_quality))
    samples.sort(key=lambda s: s.rec_ts)
    return samples


def _downsample(samples: list[RtkSample], min_distance_m: float) -> list[RtkSample]:
    if not samples:
        return []
    kept = [samples[0]]
    for sample in samples[1:]:
        last = kept[-1]
        if algo._fast_distance_m(last.lat, last.lon, sample.lat, sample.lon) >= min_distance_m:
            kept.append(sample)
    if kept[-1] is not samples[-1]:
        kept.append(samples[-1])
    return kept


def _path_length_m(samples: list[RtkSample]) -> float:
    return sum(
        algo._fast_distance_m(a.lat, a.lon, b.lat, b.lon)
        for a, b in zip(samples, samples[1:])
    )


def _write_waypoint_csv(waypoints: list[RtkSample], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["lat", "lon", "type", "lift"])
        for wp in waypoints:
            writer.writerow([wp.lat, wp.lon, "", ""])


def main() -> None:
    run_file = RUN_FILE or _find_latest_run_file()
    print(f"Reading run file: {run_file}")

    samples = _load_rtk_samples(run_file)
    print(f"RTK samples read (fix_quality >= {MIN_FIX_QUALITY}): {len(samples)}")
    if not samples:
        print("No usable RTK samples found -- aborting.")
        return

    waypoints = _downsample(samples, MIN_DISTANCE_M)
    length_m = _path_length_m(waypoints)

    run_ts = run_file.stem.replace("run_", "")
    density_tag = f"{MIN_DISTANCE_M:g}m"
    out_path = OUTPUT_DIR / f"path_{run_ts}_{density_tag}.csv"
    _write_waypoint_csv(waypoints, out_path)

    print(f"Waypoints kept (min spacing {MIN_DISTANCE_M:g} m): {len(waypoints)}")
    print(f"Approx path length: {length_m:.1f} m")
    print(f"Output: {out_path}")
    print("Load it via the AutoNav dashboard 'LOAD CSV' button, "
          "or copy/rename it to path.csv and restart autonav_bridge.py.")


if __name__ == "__main__":
    main()
