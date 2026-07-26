"""listen_robot_websocket.py — output-boundary observer / recorder for 04_Robot.

Connects to robot_bridge's WebSocket, prints a live formatted line per
message type, and logs everything (with local receive timestamps added) to
data_log/robot_raw_{timestamp}.jsonl.

Message types actually broadcast by robot_bridge.py: "odom", "state_status",
"imu", "rtk", "rec_status". There is no "nav_status" message type — an
earlier version of this script had a dead branch for it that never fired;
it has been removed here.

Run: python listen_robot_websocket.py
"""

import asyncio
import json
from datetime import datetime
from pathlib import Path

import websockets

HOST = "localhost"
PORT = 8889
DATA_LOG_DIR = Path(__file__).parent / "data_log"


async def listen(host: str, port: int) -> None:
    DATA_LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = DATA_LOG_DIR / f"robot_raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
    print(f"Logging to {log_path}")
    url = f"ws://{host}:{port}/"

    with log_path.open("a", encoding="utf-8") as log_file:
        while True:
            try:
                async with websockets.connect(url) as ws:
                    print(f"Connected to {url}")
                    async for raw in ws:
                        if isinstance(raw, bytes):
                            raw = raw.decode("utf-8")
                        msg = json.loads(raw)

                        recv_dt = datetime.now()
                        msg["log_recv_ts"] = recv_dt.timestamp()
                        msg["log_recv_iso"] = recv_dt.isoformat(timespec="milliseconds")
                        log_file.write(json.dumps(msg, ensure_ascii=False) + "\n")
                        log_file.flush()

                        msg_type = msg.get("type")
                        if msg_type == "imu":
                            euler = msg.get("euler", {})
                            heading = msg.get("heading", {})
                            print(f"IMU  | roll={euler.get('roll')} pitch={euler.get('pitch')} "
                                  f"yaw={euler.get('yaw')} heading={heading.get('deg')} dir={heading.get('dir')}")
                        elif msg_type == "rtk":
                            print(f"RTK  | lat={msg.get('lat')} lon={msg.get('lon')} "
                                  f"fix_quality={msg.get('fix_quality')} num_sats={msg.get('num_sats')} "
                                  f"available={msg.get('available')}")
                        elif msg_type == "odom":
                            print(f"ODOM | v={msg.get('v')} w={msg.get('w')} "
                                  f"soc={msg.get('soc')} state={msg.get('state')}")
                        elif msg_type == "state_status":
                            print(f"STATE| active={msg.get('active')}")
                        elif msg_type == "rec_status":
                            print(f"REC  | recording={msg.get('recording')} filename={msg.get('filename')}")
                        else:
                            print(f"OTHER| {json.dumps(msg, ensure_ascii=False)}")
            except (OSError, websockets.exceptions.WebSocketException) as exc:
                print(f"Connection error: {exc}. Retrying in 2s...")
                await asyncio.sleep(2)


def main() -> None:
    try:
        asyncio.run(listen(HOST, PORT))
    except KeyboardInterrupt:
        print("Stopped")


if __name__ == "__main__":
    main()
