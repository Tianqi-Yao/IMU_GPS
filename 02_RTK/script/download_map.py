#!/usr/bin/env python3
"""
Download map tiles around a center point into web_static/assets/tiles for LAN/offline use.

Example:
  python download_map.py
"""

from __future__ import annotations

import math
import time
from pathlib import Path
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

EARTH_RADIUS_M = 6371008.8 # mean Earth radius in meters
LAT = 38.9412928598587
LON = -92.31884600793728
RADIUS_MILES = 0.5
ZOOM_MIN = 1
ZOOM_MAX = 19
URL_TEMPLATE = "https://tile.openstreetmap.org/{z}/{x}/{y}.png"
OUT_DIR = Path(__file__).resolve().parents[1] / "web_static" / "assets" / "tiles"
TIMEOUT = 10.0
RETRIES = 2
SLEEP = 0.1
OVERWRITE = False


def deg2num(lat_deg: float, lon_deg: float, zoom: int) -> tuple[int, int]:
    lat_rad = math.radians(lat_deg)
    n = 2**zoom
    xtile = int((lon_deg + 180.0) / 360.0 * n)
    ytile = int((1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n)
    return xtile, ytile


def clamp_lat(lat: float) -> float:
    return max(min(lat, 85.05112878), -85.05112878)


def bounding_box(center_lat: float, center_lon: float, radius_m: float) -> tuple[float, float, float, float]:
    """Return (min_lat, min_lon, max_lat, max_lon)."""
    dlat = math.degrees(radius_m / EARTH_RADIUS_M)
    # avoid division by zero near poles
    cos_lat = max(math.cos(math.radians(center_lat)), 1e-6)
    dlon = math.degrees(radius_m / (EARTH_RADIUS_M * cos_lat))
    min_lat = clamp_lat(center_lat - dlat)
    max_lat = clamp_lat(center_lat + dlat)
    min_lon = center_lon - dlon
    max_lon = center_lon + dlon
    return min_lat, min_lon, max_lat, max_lon


def fetch_tile(url: str, out_path: Path, timeout: float, retries: int, sleep_s: float) -> bool:
    req = Request(url, headers={"User-Agent": "IMU_GPS offline tile downloader/1.0"})
    for i in range(retries + 1):
        try:
            with urlopen(req, timeout=timeout) as resp:
                data = resp.read()
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_bytes(data)
            return True
        except (HTTPError, URLError, TimeoutError):
            if i >= retries:
                return False
            time.sleep(sleep_s)
    return False


def main() -> None:
    radius_m = RADIUS_MILES * 1609.344

    min_lat, min_lon, max_lat, max_lon = bounding_box(LAT, LON, radius_m)
    print(f"Center=({LAT:.8f}, {LON:.8f}) radius={RADIUS_MILES} miles")
    print(f"BBox lat[{min_lat:.6f},{max_lat:.6f}] lon[{min_lon:.6f},{max_lon:.6f}]")
    print(f"Output: {OUT_DIR}")

    total = 0
    skipped = 0
    ok = 0
    failed = 0

    for z in range(ZOOM_MIN, ZOOM_MAX + 1):
        x0, y1 = deg2num(max_lat, min_lon, z)
        x1, y0 = deg2num(min_lat, max_lon, z)
        xmin, xmax = sorted((x0, x1))
        ymin, ymax = sorted((y0, y1))

        zoom_total = (xmax - xmin + 1) * (ymax - ymin + 1)
        print(f"z={z}: x[{xmin},{xmax}] y[{ymin},{ymax}] tiles={zoom_total}")

        for x in range(xmin, xmax + 1):
            for y in range(ymin, ymax + 1):
                total += 1
                out_path = OUT_DIR / str(z) / str(x) / f"{y}.png"
                if out_path.exists() and not OVERWRITE:
                    skipped += 1
                    continue

                url = URL_TEMPLATE.format(z=z, x=x, y=y)
                if fetch_tile(url, out_path, TIMEOUT, RETRIES, SLEEP):
                    ok += 1
                else:
                    failed += 1

    print("Done")
    print(f"  total={total}")
    print(f"  downloaded={ok}")
    print(f"  skipped={skipped}")
    print(f"  failed={failed}")


if __name__ == "__main__":
    main()
