"""listen_rtk_websocket.py — output-boundary observer / recorder for 02_RTK.

Connects to rtk_bridge's WebSocket, prints a live formatted table, and logs
every frame (with local receive timestamps added) to
data_log/rtk_raw_{timestamp}.jsonl for later replay via replay_rtk_websocket.py.

Only parses the current single-source rtk_frame contract (lat/lon/alt/
fix_quality/num_sats/hdop/speed_knots/track_deg/rtk_ts/server_ts/source).
Dual-antenna/multi-source fields are not part of this contract.

Run: python listen_rtk_websocket.py
"""

import asyncio
import json
from datetime import datetime
from pathlib import Path

import websockets

WS_URL = "ws://localhost:8776"
DATA_LOG_DIR = Path(__file__).parent / "data_log"

FIX_QUALITY = {0: "No Fix", 1: "GPS", 2: "DGPS", 4: "RTK Fixed", 5: "RTK Float"}


async def listen(ws_url: str) -> None:
    DATA_LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = DATA_LOG_DIR / f"rtk_raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
    print(f"Logging to {log_path}")
    print(f"{'Lat':>12} | {'Lon':>13} | {'Alt (m)':>8} | {'Fix':>9} | "
          f"{'Sats':>4} | {'HDOP':>5} | {'Speed (kn)':>10} | {'Track (deg)':>11} | Source")

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

                fix_label = FIX_QUALITY.get(data.get("fix_quality", 0), str(data.get("fix_quality")))
                print(
                    f"{data.get('lat', 0):>12.7f} | {data.get('lon', 0):>13.7f} | "
                    f"{data.get('alt') or 0:>8.2f} | {fix_label:>9} | "
                    f"{data.get('num_sats', 0):>4} | {data.get('hdop') or 0:>5.2f} | "
                    f"{data.get('speed_knots') or 0:>10.2f} | {data.get('track_deg') or 0:>11.2f} | "
                    f"{data.get('source', '-')}"
                )


def main() -> None:
    try:
        asyncio.run(listen(WS_URL))
    except ConnectionRefusedError:
        print(f"Could not connect to {WS_URL} — is rtk_bridge.py running?")
    except KeyboardInterrupt:
        print("Stopped")


if __name__ == "__main__":
    main()
