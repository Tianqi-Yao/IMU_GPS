#!/usr/bin/env python3
"""
Minimal WebSocket client to listen to Camera status data from camera_bridge.py.

Usage:
    python listen_camera_websocket.py
"""

import asyncio
import json
from datetime import datetime
from pathlib import Path

import websockets

WS_URL = "ws://localhost:8816"


async def listen_camera(ws_url: str) -> None:
    print(f"Connecting to {ws_url}...")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    raw_log_path = Path(".") / "data_log" / f"camera_raw_{ts}.jsonl"
    raw_log_path.parent.mkdir(parents=True, exist_ok=True)
    print(f"Raw log file: {raw_log_path}")

    while True:
        try:
            async with websockets.connect(ws_url) as ws:
                print("✓ Connected!")
                print("\nReceiving Camera status...\n")
                print("Plugin        | Cam | Streaming | FPS   | Size      | C1 Clients | C2 Clients")
                print("-" * 86)

                async for message in ws:
                    try:
                        raw_text = message.decode("utf-8", errors="replace") if isinstance(message, bytes) else str(message)
                        recv_dt = datetime.now()
                        recv_ts_iso = recv_dt.isoformat(timespec="milliseconds")
                        recv_ts_epoch = recv_dt.timestamp()

                        data = json.loads(raw_text)
                        data["log_recv_ts"] = recv_ts_epoch
                        data["log_recv_iso"] = recv_ts_iso

                        with raw_log_path.open("a", encoding="utf-8") as raw_log_file:
                            raw_log_file.write(json.dumps(data, ensure_ascii=False))
                            raw_log_file.write("\n")

                        print(
                            f"{str(data.get('active_plugin', '')):<13} | "
                            f"{str(data.get('cam_selection', '')):>3} | "
                            f"{str(data.get('streaming', '')):>9} | "
                            f"{float(data.get('fps', 0.0)):>5.1f} | "
                            f"{int(data.get('width', 0))}x{int(data.get('height', 0)):<5} | "
                            f"{int(data.get('cam1_clients', 0)):>10} | "
                            f"{int(data.get('cam2_clients', 0)):>10}"
                        )
                    except json.JSONDecodeError as exc:
                        print(f"JSON decode error: {exc}")
                    except Exception as exc:
                        print(f"Error processing message: {exc}")

        except (OSError, websockets.exceptions.WebSocketException) as exc:
            print(f"Disconnected: {exc}. Retrying in 2s...")
            await asyncio.sleep(2)


def main() -> None:
    try:
        asyncio.run(listen_camera(WS_URL))
    except KeyboardInterrupt:
        print("\n✓ Stopped")


if __name__ == "__main__":
    main()
