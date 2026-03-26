#!/usr/bin/env python3
"""
send_robot_only_demo.py — Minimal sender demo for robot_bridge WebSocket.

Usage:
    python send_robot_only_demo.py
    python send_robot_only_demo.py --host localhost --port 8889

Commands:
    w: forward   s: backward   a: left turn   d: right turn
    x: stop      t: toggle_state
    h: heartbeat j: custom json
    q: quit
"""

import argparse
import asyncio
import json

import websockets


async def run_sender(host: str, port: int) -> None:
    url = f"ws://{host}:{port}/"
    print(f"Connecting to {url} ...")
    async with websockets.connect(url) as ws:
        print("Connected")
        print("Commands: w/s/a/d/x/t/h/j/q")

        while True:
            cmd = (await asyncio.to_thread(input, "> ")).strip().lower()

            if cmd == "q":
                print("Bye")
                return

            if cmd == "w":
                msg = {"type": "joystick", "linear": 0.30, "angular": 0.00, "force": 1.0}
            elif cmd == "s":
                msg = {"type": "joystick", "linear": -0.30, "angular": 0.00, "force": 1.0}
            elif cmd == "a":
                msg = {"type": "joystick", "linear": 0.00, "angular": 0.60, "force": 1.0}
            elif cmd == "d":
                msg = {"type": "joystick", "linear": 0.00, "angular": -0.60, "force": 1.0}
            elif cmd == "x":
                msg = {"type": "joystick", "linear": 0.00, "angular": 0.00, "force": 0.0}
            elif cmd == "t":
                msg = {"type": "toggle_state"}
            elif cmd == "h":
                msg = {"type": "heartbeat"}
            elif cmd == "j":
                raw = await asyncio.to_thread(input, "json> ")
                try:
                    msg = json.loads(raw)
                except json.JSONDecodeError as exc:
                    print(f"JSON error: {exc}")
                    continue
            else:
                print("Unknown command. Use: w/s/a/d/x/t/h/j/q")
                continue

            await ws.send(json.dumps(msg))
            print("sent:", msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Minimal sender demo for robot_bridge")
    parser.add_argument("--host", default="localhost", help="robot_bridge host")
    parser.add_argument("--port", type=int, default=8889, help="robot_bridge ws port")
    args = parser.parse_args()

    try:
        asyncio.run(run_sender(args.host, args.port))
    except KeyboardInterrupt:
        print("\nStopped")


if __name__ == "__main__":
    main()
