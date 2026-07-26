"""listen_imu_websocket.py — output-boundary observer / recorder for 01_IMU.

Connects to imu_bridge's WebSocket, prints a live formatted table, and logs
every frame (with local receive timestamps added) to
data_log/imu_raw_{timestamp}.jsonl for later replay via replay_imu_websocket.py.

Run: python listen_imu_websocket.py
"""

import asyncio
import json
from datetime import datetime
from pathlib import Path

import websockets

WS_URL = "ws://localhost:8766"
DATA_LOG_DIR = Path(__file__).parent / "data_log"


async def listen(ws_url: str) -> None:
    DATA_LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = DATA_LOG_DIR / f"imu_raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
    print(f"Logging to {log_path}")
    print(f"{'Roll (deg)':>10} | {'Pitch (deg)':>11} | {'Yaw (deg)':>9} | "
          f"{'Heading (deg)':>13} | {'Dir':>4} | {'Hz':>5} | Quaternion")

    with log_path.open("a", encoding="utf-8") as log_file:
        async with websockets.connect(ws_url) as ws:
            async for raw in ws:
                if isinstance(raw, bytes):
                    raw = raw.decode("utf-8")
                data = json.loads(raw)

                recv_dt = datetime.now()
                data["log_recv_ts"] = recv_dt.timestamp()
                data["log_recv_iso"] = recv_dt.isoformat(timespec="milliseconds")
                log_file.write(json.dumps(data, ensure_ascii=False) + "\n")
                log_file.flush()

                euler = data.get("euler", {})
                heading = data.get("heading", {})
                rot = data.get("rot", {})
                print(
                    f"{euler.get('roll', 0):>10.2f} | {euler.get('pitch', 0):>11.2f} | "
                    f"{euler.get('yaw', 0):>9.2f} | {heading.get('deg', 0):>13.2f} | "
                    f"{heading.get('dir', '-'):>4} | {data.get('hz', 0):>5.1f} | "
                    f"({rot.get('qi', 0):.3f}, {rot.get('qj', 0):.3f}, "
                    f"{rot.get('qk', 0):.3f}, {rot.get('qr', 0):.3f})"
                )


def main() -> None:
    try:
        asyncio.run(listen(WS_URL))
    except ConnectionRefusedError:
        print(f"Could not connect to {WS_URL} — is imu_bridge.py running?")
    except KeyboardInterrupt:
        print("Stopped")


if __name__ == "__main__":
    main()
