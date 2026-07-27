"""build_waypoints_from_run.py — generate a waypoint CSV from a recorded manual run.

Reads the most recent 04_Robot/data_log/run_*.jsonl (produced by
robot_bridge.py's own "set_recording" feature — drive the route manually
with recording on, then run this script), keeps only RTK samples meeting
WAYPOINT_MIN_FIX_QUALITY, greedily downsamples them so consecutive
waypoints are at least WAYPOINT_MIN_DISTANCE_M apart, and writes
data_log/path_{run_timestamp}_{density}m.csv (lat,lon,type,lift columns;
type/lift are left blank for manual editing — this script has no way to
infer alley/pause/lift semantics from raw GPS samples).

Run: python build_waypoints_from_run.py
"""

import json
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    import config as _cfg
except ImportError:
    _cfg = None

import autonav_algo as algo

MIN_DISTANCE_M = _cfg.WAYPOINT_MIN_DISTANCE_M if _cfg else 1.0
MIN_FIX_QUALITY = _cfg.WAYPOINT_MIN_FIX_QUALITY if _cfg else 1

ROBOT_DATA_LOG_DIR = Path(__file__).parent.parent / "04_Robot" / "data_log"
OUTPUT_DIR = Path(__file__).parent / "data_log"


@dataclass
class RtkSample:
    rec_ts: float
    lat: float
    lon: float
    fix_quality: int


def _find_latest_run_file() -> Path:
    files = list(ROBOT_DATA_LOG_DIR.glob("run_*.jsonl"))
    if not files:
        raise FileNotFoundError(
            f"No run_*.jsonl files found in {ROBOT_DATA_LOG_DIR}. "
            "Record a manual run first via robot_bridge.py's recording feature."
        )
    return max(files, key=lambda p: p.stat().st_mtime)


def _load_rtk_samples(path: Path) -> List[RtkSample]:
    samples: List[RtkSample] = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError:
                continue
            if row.get("type") != "rtk":
                continue
            fix_quality = row.get("fix_quality") or 0
            lat, lon = row.get("lat"), row.get("lon")
            if fix_quality < MIN_FIX_QUALITY or lat is None or lon is None:
                continue
            samples.append(RtkSample(row.get("rec_ts", 0.0), lat, lon, fix_quality))
    samples.sort(key=lambda s: s.rec_ts)
    return samples


def _downsample(samples: List[RtkSample], min_distance_m: float) -> List[RtkSample]:
    if not samples:
        return []
    kept = [samples[0]]
    for sample in samples[1:]:
        if algo.fast_distance_m(kept[-1].lat, kept[-1].lon, sample.lat, sample.lon) >= min_distance_m:
            kept.append(sample)
    if kept[-1] is not samples[-1]:
        kept.append(samples[-1])
    return kept


def _path_length_m(samples: List[RtkSample]) -> float:
    total = 0.0
    for a, b in zip(samples, samples[1:]):
        total += algo.fast_distance_m(a.lat, a.lon, b.lat, b.lon)
    return total


def _write_waypoint_csv(waypoints: List[RtkSample], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with out_path.open("w", encoding="utf-8") as f:
        f.write("lat,lon,type,lift\n")
        for wp in waypoints:
            f.write(f"{wp.lat:.8f},{wp.lon:.8f},,\n")


def main() -> None:
    run_path = _find_latest_run_file()
    samples = _load_rtk_samples(run_path)
    if not samples:
        raise SystemExit(f"No usable RTK samples (fix_quality >= {MIN_FIX_QUALITY}) found in {run_path}")

    waypoints = _downsample(samples, MIN_DISTANCE_M)
    length_m = _path_length_m(samples)

    run_ts = run_path.stem.replace("run_", "")
    density = f"{MIN_DISTANCE_M:g}"
    out_path = OUTPUT_DIR / f"path_{run_ts}_{density}m.csv"
    _write_waypoint_csv(waypoints, out_path)

    print(f"Source run: {run_path} ({len(samples)} RTK samples, {length_m:.1f} m path length)")
    print(f"Wrote {len(waypoints)} waypoint(s) to {out_path}")
    print("NOTE: 'type'/'lift' columns are blank — edit manually to mark alley/pause/lift waypoints.")


if __name__ == "__main__":
    main()
