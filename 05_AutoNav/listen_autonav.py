"""listen_autonav.py — output-boundary observer / recorder for 05_AutoNav.

Connects to autonav_bridge's WebSocket, prints a live one-line status
summary, and logs every autonav_status frame (full payload, including the
embedded imu_raw/rtk_raw passthrough) to
data_log/autonav_raw_{timestamp}.jsonl.

Run: python listen_autonav.py
"""

import asyncio
import json
from datetime import datetime
from pathlib import Path

import websockets

WS_URL = "ws://localhost:8806"
DATA_LOG_DIR = Path(__file__).parent / "data_log"


async def listen(ws_url: str) -> None:
    DATA_LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = DATA_LOG_DIR / f"autonav_raw_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
    print(f"Logging to {log_path}")

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
                    f"state={data.get('state'):>8} wp={data.get('current_wp_idx')}/{data.get('total_wp')} "
                    f"dist_to_wp={data.get('dist_to_wp_m')}m bearing_err={data.get('bearing_error_deg')}deg "
                    f"linear={data.get('linear')} angular={data.get('angular')} "
                    f"sensor_block={data.get('sensor_block_reason')} robot_connected={data.get('robot_connected')}"
                )


def main() -> None:
    try:
        asyncio.run(listen(WS_URL))
    except ConnectionRefusedError:
        print(f"Could not connect to {WS_URL} — is autonav_bridge.py running?")
    except KeyboardInterrupt:
        print("Stopped")


if __name__ == "__main__":
    main()
