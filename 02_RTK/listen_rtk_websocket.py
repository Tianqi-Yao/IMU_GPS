#!/usr/bin/env python3
"""
Minimal WebSocket client to listen to RTK data from rtk_bridge.py

Usage:
    python listen_rtk_websocket.py
"""

import asyncio
import json
from pathlib import Path
from datetime import datetime
import websockets

WS_URL = "ws://localhost:8776"


# Fix quality descriptions
FIX_QUALITY = {
    0: "No Fix",
    1: "GPS",
    2: "DGPS",
    4: "RTK Fixed",
    5: "RTK Float",
}


async def listen_rtk(ws_url: str):
    """Connect to rtk_bridge.py WebSocket and listen for RTK data."""
    print(f"Connecting to {ws_url}...")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_log_path = Path(".") / "data_log" / f"rtk_raw_{ts}.jsonl"
    raw_log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"Raw log file: {raw_log_path}")
    
    try:
        async with websockets.connect(ws_url) as websocket:
            print("✓ Connected!")
            print("\nReceiving RTK data...\n")
            print("Active | Heading          | Baseline | RTK Source | Connected | Lat (°)      | Lon (°)       | Alt (m)  | Fix Type      | Sats | HDOP")
            print("-" * 132)
            
            frame_count = 0
            async for message in websocket:
                try:
                    raw_text = message.decode("utf-8", errors="replace") if isinstance(message, bytes) else str(message)
                    recv_dt = datetime.now()
                    recv_ts_iso = recv_dt.isoformat(timespec="milliseconds")
                    recv_ts_epoch = recv_dt.timestamp()

                    data = json.loads(raw_text)
                    data["log_recv_ts"] = recv_ts_epoch
                    data["log_recv_iso"] = recv_ts_iso

                    with raw_log_path.open("a", encoding="utf-8") as raw_log_file:
                        raw_log_file.write(json.dumps(data, ensure_ascii=False))
                        raw_log_file.write("\n")
                    
                    lat = data.get("lat")
                    lon = data.get("lon")
                    alt = data.get("alt")
                    fix_quality = data.get("fix_quality", 0)
                    num_sats = data.get("num_sats", 0)
                    hdop = data.get("hdop")
                    speed_knots = data.get("speed_knots")
                    heading_deg = data.get("heading_deg")
                    heading_valid = data.get("heading_valid", False)
                    heading_dir = data.get("heading_dir")
                    baseline_m = data.get("heading_baseline_m")
                    active_source = data.get("rtk_active_source", data.get("rtk_source", ""))
                    active_label = data.get("rtk_source_label", active_source)
                    source_frames = data.get("rtk_source_frames", [])
                    
                    fix_str = FIX_QUALITY.get(fix_quality, f"Unknown({fix_quality})")
                    
                    if heading_valid and heading_deg is not None:
                        heading_str = f"{heading_deg:.1f}° {heading_dir or ''}".strip()
                    else:
                        heading_str = "N/A"
                    baseline_str = f"{baseline_m:.2f}m" if baseline_m is not None else "N/A"

                    active_summary = f"{active_source or '-'} ({active_label or '-'})"
                    if not source_frames:
                        source_frames = [{
                            "source_id": active_source or "rtk",
                            "label": active_label or "RTK",
                            "connected": True,
                            "lat": lat,
                            "lon": lon,
                            "alt": alt,
                            "fix_quality": fix_quality,
                            "num_sats": num_sats,
                            "hdop": hdop,
                        }]

                    print(f"ACTIVE: {active_summary} | HEADING: {heading_str} | BASELINE: {baseline_str}")
                    for src in source_frames:
                        src_id = src.get("source_id", "?")
                        label = src.get("label", src_id)
                        connected = "yes" if src.get("connected") else "no"
                        src_lat = src.get("lat")
                        src_lon = src.get("lon")
                        src_alt = src.get("alt")
                        src_fix = FIX_QUALITY.get(src.get("fix_quality", 0), f"Unknown({src.get('fix_quality', 0)})")
                        src_sats = src.get("num_sats", 0)
                        src_hdop = src.get("hdop")

                        lat_str = f"{src_lat:.8f}" if src_lat is not None else "N/A"
                        lon_str = f"{src_lon:.8f}" if src_lon is not None else "N/A"
                        alt_str = f"{src_alt:.1f}" if src_alt is not None else "N/A"
                        hdop_str = f"{src_hdop:.2f}" if src_hdop is not None else "N/A"

                        print(
                            f"{label:<6} | {connected:^9} | {lat_str} | {lon_str} | {alt_str:>7} | "
                            f"{src_fix:<13} | {src_sats:>4} | {hdop_str:>5}"
                        )
                    print("-" * 132)
                    
                    frame_count += 1
                    
                except json.JSONDecodeError as e:
                    print(f"JSON decode error: {e}")
                except Exception as e:
                    print(f"Error processing data: {e}")
                    
    except ConnectionRefusedError:
        print(f"✗ Connection refused. Make sure rtk_bridge.py is running on {ws_url}")
    except Exception as e:
        print(f"✗ Connection error: {e}")


if __name__ == "__main__":
    try:
        asyncio.run(listen_rtk(WS_URL))
    except KeyboardInterrupt:
        print("\n✓ Stopped")
