"""send_robot_only_demo.py — interactive terminal joystick for 04_Robot.

Sends control messages to robot_bridge.py over WebSocket without needing
the browser dashboard. Useful for quick manual testing.

Keys: w/s = forward/back, a/d = turn left/right, x = stop,
      t = toggle auto-ready/active, h = heartbeat only, j = send raw JSON,
      q = quit.

Run: python send_robot_only_demo.py
"""

import asyncio
import json

import websockets

WS_URL = "ws://localhost:8889"

JOYSTICK_COMMANDS = {
    "w": {"type": "joystick", "linear": 0.30, "angular": 0.00},
    "s": {"type": "joystick", "linear": -0.30, "angular": 0.00},
    "a": {"type": "joystick", "linear": 0.00, "angular": 0.60},
    "d": {"type": "joystick", "linear": 0.00, "angular": -0.60},
    "x": {"type": "joystick", "linear": 0.00, "angular": 0.00},
}


async def main() -> None:
    async with websockets.connect(WS_URL) as ws:
        print(f"Connected to {WS_URL}")
        print("Keys: w/s/a/d/x = drive, t = toggle state, h = heartbeat, j = raw json, q = quit")
        while True:
            cmd = (await asyncio.to_thread(input, "> ")).strip().lower()
            if cmd == "q":
                break
            if cmd in JOYSTICK_COMMANDS:
                await ws.send(json.dumps(JOYSTICK_COMMANDS[cmd]))
            elif cmd == "t":
                await ws.send(json.dumps({"type": "toggle_state"}))
            elif cmd == "h":
                await ws.send(json.dumps({"type": "heartbeat"}))
            elif cmd == "j":
                raw = await asyncio.to_thread(input, "json> ")
                try:
                    json.loads(raw)  # validate before sending
                except json.JSONDecodeError as exc:
                    print(f"Invalid JSON: {exc}")
                    continue
                await ws.send(raw)
            else:
                print(f"Unknown command: {cmd!r}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Stopped")
    except ConnectionRefusedError:
        print(f"Could not connect to {WS_URL} — is robot_bridge.py running?")
