"""record_all.py — one-shot recorder for every running bridge's WS output
plus both camera MJPEG streams, into a single timestamped session directory.

Connects to whichever bridges are already running (00_QR has no WS output
and is not recorded); each stream that fails to connect is retried every
RECONNECT_DELAY_S seconds independently of the others. Stop with Ctrl+C —
all files are flushed and a size summary is printed before exit.

Run: python record_all.py
"""

import asyncio
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import List, Tuple

import websockets

ROOT = Path(__file__).resolve().parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    import config as _cfg
except ImportError:
    _cfg = None

from common.ports import derive_ws_port


def _c(attr: str, default):
    return getattr(_cfg, attr, default) if _cfg else default


IMU_WS_PORT = _c("IMU_WS_PORT", 8765)
RTK_WS_PORT = _c("RTK_WS_PORT", 8775)
NAV_WS_PORT = _c("NAV_WS_PORT", 8785)
ROBOT_WS_PORT = _c("ROBOT_WS_PORT", 8888)
AUTONAV_WS_PORT = _c("AUTONAV_WS_PORT", 8805)
CAM_WS_PORT = _c("CAM_WS_PORT", 8815)
CAM1_STREAM_PORT = _c("CAM1_STREAM_PORT", 8080)
CAM2_STREAM_PORT = _c("CAM2_STREAM_PORT", 8081)
CAM_FPS = _c("CAM_FPS", 25)
CAM_WIDTH = _c("CAM_WIDTH", 640)
CAM_HEIGHT = _c("CAM_HEIGHT", 400)
SYSMON_WS_PORT = _c("SYSMON_WS_PORT", 8825)

WS_STREAMS: List[Tuple[str, str]] = [
    ("imu", f"ws://localhost:{derive_ws_port(IMU_WS_PORT)}"),
    ("rtk", f"ws://localhost:{derive_ws_port(RTK_WS_PORT)}"),
    ("nav", f"ws://localhost:{derive_ws_port(NAV_WS_PORT)}"),
    ("robot", f"ws://localhost:{derive_ws_port(ROBOT_WS_PORT)}"),
    ("autonav", f"ws://localhost:{derive_ws_port(AUTONAV_WS_PORT)}"),
    ("camera_status", f"ws://localhost:{derive_ws_port(CAM_WS_PORT)}"),
    ("sysmon", f"ws://localhost:{derive_ws_port(SYSMON_WS_PORT)}"),
]
CAM_STREAMS: List[Tuple[str, str]] = [
    ("cam1", f"http://localhost:{CAM1_STREAM_PORT}"),
    ("cam2", f"http://localhost:{CAM2_STREAM_PORT}"),
]
RECONNECT_DELAY_S = 2.0


def _fmt_size(num_bytes: int) -> str:
    size = float(num_bytes)
    for unit in ("B", "KB", "MB", "GB"):
        if size < 1024 or unit == "GB":
            return f"{size:.1f} {unit}"
        size /= 1024
    return f"{size:.1f} TB"


async def _ws_record(name: str, url: str, out_path: Path, stop_event: asyncio.Event) -> None:
    with out_path.open("a", encoding="utf-8") as f:
        while not stop_event.is_set():
            try:
                async with websockets.connect(url, open_timeout=3) as ws:
                    print(f"[{name}] connected to {url}")
                    async for raw in ws:
                        if isinstance(raw, bytes):
                            raw = raw.decode("utf-8")
                        obj = json.loads(raw)
                        obj["log_recv_ts"] = time.time()
                        f.write(json.dumps(obj, ensure_ascii=False, separators=(",", ":")) + "\n")
                        f.flush()
            except asyncio.CancelledError:
                break
            except Exception as exc:
                print(f"[{name}] connection error: {exc}. Retrying in {RECONNECT_DELAY_S}s...")
                try:
                    await asyncio.wait_for(stop_event.wait(), timeout=RECONNECT_DELAY_S)
                except asyncio.TimeoutError:
                    pass


def _cam_record(name: str, url: str, out_path: Path, stop_event: threading.Event) -> None:
    try:
        import cv2
    except ImportError:
        print(f"[{name}] opencv-python not installed; skipping camera recording.")
        return

    cap = cv2.VideoCapture(url)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, CAM_FPS, (CAM_WIDTH, CAM_HEIGHT))
    print(f"[{name}] recording {url} -> {out_path}")
    try:
        while not stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            if frame.shape[1] != CAM_WIDTH or frame.shape[0] != CAM_HEIGHT:
                frame = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT))
            writer.write(frame)
    finally:
        writer.release()
        cap.release()


async def main() -> None:
    session_dir = ROOT / "data_log" / f"session_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    session_dir.mkdir(parents=True, exist_ok=True)
    print(f"Recording session: {session_dir}")

    async_stop = asyncio.Event()
    thread_stop = threading.Event()

    ws_tasks = [
        asyncio.create_task(_ws_record(name, url, session_dir / f"{name}.jsonl", async_stop))
        for name, url in WS_STREAMS
    ]
    cam_threads = [
        threading.Thread(
            target=_cam_record, args=(name, url, session_dir / f"{name}.mp4", thread_stop),
            name=f"cam-record-{name}", daemon=True,
        )
        for name, url in CAM_STREAMS
    ]
    for t in cam_threads:
        t.start()

    try:
        await asyncio.sleep(float("inf"))
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        print("\nStopping...")
        async_stop.set()
        thread_stop.set()

        loop = asyncio.get_running_loop()
        for t in cam_threads:
            await loop.run_in_executor(None, t.join, 5.0)

        for task in ws_tasks:
            task.cancel()
        await asyncio.gather(*ws_tasks, return_exceptions=True)

        print("\nRecorded files:")
        for f in sorted(session_dir.iterdir()):
            print(f"  {f.name:30s} {_fmt_size(f.stat().st_size)}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
