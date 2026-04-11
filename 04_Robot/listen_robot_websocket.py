#!/usr/bin/env python3
"""
listen_robot_websocket.py — Minimal demo for reading robot_bridge data.

Shows the simplest way to:
- connect to ws://localhost:8889
- receive JSON messages
- read fields from IMU / RTK / nav_status / odom messages

Usage:
    python listen_robot_websocket.py
"""

import asyncio
import json
from pathlib import Path
from datetime import datetime
import websockets

HOST = "localhost"
PORT = 8889


async def listen(host: str, port: int) -> None:
    url = f"ws://{host}:{port}/"
    print(f"Connecting to {url} …")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_log_path = Path(".") / "data_log" / f"robot_raw_{ts}.jsonl"
    raw_log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"Raw log file: {raw_log_path}")

    while True:
        try:
            async with websockets.connect(url) as ws:
                print(f"Connected to {url}")

                async for raw in ws:
                    try:
                        raw_text = raw.decode("utf-8", errors="replace") if isinstance(raw, bytes) else str(raw)
                        recv_dt = datetime.now()
                        recv_ts_iso = recv_dt.isoformat(timespec="milliseconds")
                        recv_ts_epoch = recv_dt.timestamp()

                        msg = json.loads(raw_text)
                        msg["log_recv_ts"] = recv_ts_epoch
                        msg["log_recv_iso"] = recv_ts_iso

                        with raw_log_path.open("a", encoding="utf-8") as raw_log_file:
                            raw_log_file.write(json.dumps(msg, ensure_ascii=False))
                            raw_log_file.write("\n")

                        msg_type = msg.get("type")

                        if msg_type == "imu":
                            euler = msg.get("euler", {})
                            print(
                                "IMU  | "
                                f"roll={euler.get('roll')} "
                                f"pitch={euler.get('pitch')} "
                                f"yaw={euler.get('yaw')} "
                                f"heading={msg.get('heading', {}).get('deg')} "
                                f"dir={msg.get('heading', {}).get('dir')} "
                            )

                        elif msg_type == "rtk":
                            print(
                                "RTK  | "
                                f"lat={msg.get('lat')} "
                                f"lon={msg.get('lon')} "
                                f"fix={msg.get('fix_quality')} "
                                f"sats={msg.get('num_sats')} "
                                f"available={msg.get('available')}"
                            )

                        elif msg_type == "nav_status":
                            print(
                                "NAV  | "
                                f"state={msg.get('state')} "
                                f"progress={msg.get('progress')} "
                                f"dist_m={msg.get('distance_m')} "
                                f"heading={msg.get('heading_deg')} "
                                f"dir={msg.get('heading_dir')}"
                            )

                        elif msg_type == "odom":
                            print(
                                "ODOM | "
                                f"v={msg.get('v')} "
                                f"w={msg.get('w')} "
                                f"soc={msg.get('soc')} "
                                f"state={msg.get('state')}"
                            )

                        else:
                            print(f"OTHER| {json.dumps(msg, ensure_ascii=False)}")

                    except json.JSONDecodeError:
                        print(f"[raw] {raw}")

        except (OSError, websockets.exceptions.WebSocketException) as exc:
            print(f"Disconnected: {exc}. Retrying in 2s…")
            await asyncio.sleep(2)


def main() -> None:
    asyncio.run(listen(HOST, PORT))


if __name__ == "__main__":
    main()
