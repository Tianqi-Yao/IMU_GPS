"""listen_nav_websocket.py — output-boundary observer / recorder for 03_Nav.

Connects to nav_bridge's WebSocket, prints a live formatted table, and logs
every frame to data_log/nav_raw_{timestamp}.jsonl.

nav_frame only ever contains "imu" and "rtk" sub-objects (nav_bridge is a
passthrough, not a fusion module — see README.md); there is no independent
"nav" sub-object with derived distance/heading fields, so this listener only
reads euler/heading from "imu" and lat/lon/fix_quality from "rtk".

Run: python listen_nav_websocket.py
"""

import asyncio
import json
from datetime import datetime
from pathlib import Path

import websockets

WS_URL = "ws://localhost:8786"
DATA_LOG_DIR = Path(__file__).parent / "data_log"

FIX_QUALITY = {0: "No Fix", 1: "GPS", 2: "DGPS", 4: "RTK Fixed", 5: "RTK Float"}


async def listen(ws_url: str) -> None:
    DATA_LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = DATA_LOG_DIR / f"nav_raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
    print(f"Logging to {log_path}")
    print(f"{'Roll':>7} | {'Pitch':>7} | {'Yaw':>7} | {'Lat':>12} | {'Lon':>13} | Fix")

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

                euler = data.get("imu", {}).get("euler", {})
                rtk = data.get("rtk", {})
                fix_label = FIX_QUALITY.get(rtk.get("fix_quality", 0), str(rtk.get("fix_quality")))
                print(
                    f"{euler.get('roll', 0):>7.2f} | {euler.get('pitch', 0):>7.2f} | "
                    f"{euler.get('yaw', 0):>7.2f} | {rtk.get('lat', 0):>12.7f} | "
                    f"{rtk.get('lon', 0):>13.7f} | {fix_label}"
                )


def main() -> None:
    try:
        asyncio.run(listen(WS_URL))
    except ConnectionRefusedError:
        print(f"Could not connect to {WS_URL} — is nav_bridge.py running?")
    except KeyboardInterrupt:
        print("Stopped")


if __name__ == "__main__":
    main()
