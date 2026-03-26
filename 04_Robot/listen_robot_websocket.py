#!/usr/bin/env python3
"""
listen_robot_websocket.py — Minimal demo for reading robot_bridge data.

Shows the simplest way to:
- connect to ws://localhost:8889
- receive JSON messages
- read fields from IMU / RTK / nav_status / odom messages

Usage:
    python listen_robot_websocket.py
    python listen_robot_websocket.py --host localhost --port 8889
"""

import asyncio
import json
import argparse
import websockets


async def listen(host: str, port: int) -> None:
    url = f"ws://{host}:{port}/"
    print(f"Connecting to {url} …")

    while True:
        try:
            async with websockets.connect(url) as ws:
                print(f"Connected to {url}")

                async for raw in ws:
                    try:
                        msg = json.loads(raw)
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
    parser = argparse.ArgumentParser(description="Minimal Robot WebSocket demo")
    parser.add_argument("--host", default="localhost", help="Bridge hostname (default: localhost)")
    parser.add_argument("--port", type=int, default=8889, help="WS port (default: 8889)")
    args = parser.parse_args()
    asyncio.run(listen(args.host, args.port))


if __name__ == "__main__":
    main()
