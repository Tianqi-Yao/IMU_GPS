"""listen_camera_websocket.py — output-boundary observer / recorder for 06_Camera.

Connects to camera_bridge's WebSocket, prints a live formatted table, and
logs every camera_status frame to data_log/camera_raw_{timestamp}.jsonl.

Run: python listen_camera_websocket.py
"""

import asyncio
import json
from datetime import datetime
from pathlib import Path

import websockets

WS_URL = "ws://localhost:8816"
DATA_LOG_DIR = Path(__file__).parent / "data_log"


async def listen(ws_url: str) -> None:
    DATA_LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = DATA_LOG_DIR / f"camera_raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
    print(f"Logging to {log_path}")
    print(f"{'Plugin':>14} | {'Cam':>4} | {'Streaming':>9} | {'FPS':>5} | {'Size':>9} | C1 Clients | C2 Clients")

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

                print(
                    f"{data.get('active_plugin', '-'):>14} | {data.get('cam_selection', '-'):>4} | "
                    f"{str(data.get('streaming')):>9} | {data.get('fps', 0):>5.1f} | "
                    f"{data.get('width')}x{data.get('height'):>9} | "
                    f"{data.get('cam1_clients', 0):>10} | {data.get('cam2_clients', 0):>10}"
                )


def main() -> None:
    try:
        asyncio.run(listen(WS_URL))
    except ConnectionRefusedError:
        print(f"Could not connect to {WS_URL} — is camera_bridge.py running?")
    except KeyboardInterrupt:
        print("Stopped")


if __name__ == "__main__":
    main()
