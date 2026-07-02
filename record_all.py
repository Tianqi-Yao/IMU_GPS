"""
record_all.py — One-shot full data recorder.

Connects to all running bridges and saves every stream to a timestamped
session directory.  Missing bridges are skipped gracefully.

Usage:
    python record_all.py

Output:
    data_log/session_YYYYMMDD_HHMMSS/
        imu.jsonl            # IMU frames       ~50 Hz
        rtk.jsonl            # RTK position      ~5 Hz
        nav.jsonl            # Fused nav         ~10 Hz
        robot.jsonl          # Robot telemetry   ~20 Hz
        autonav.jsonl        # AutoNav status     ~5 Hz
        camera_status.jsonl  # Camera status      ~1 Hz
        cam1.mp4             # Camera 1 video    (if running)
        cam2.mp4             # Camera 2 video    (if running)
"""

from __future__ import annotations

import asyncio
import json
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import rootutils
import websockets

# ── Load config (fallback to defaults if running standalone) ─────────────────
ROOT = rootutils.setup_root(__file__, indicator=".git", pythonpath=True)
try:
    import config as _cfg
except ImportError:
    _cfg = None

def _c(attr, default):
    return getattr(_cfg, attr, default) if _cfg else default

# ── Stream endpoints ──────────────────────────────────────────────────────────

WS_STREAMS = [
    ("imu",           f"ws://localhost:{_c('IMU_WS_PORT',     8765) + 1}"),
    ("rtk",           f"ws://localhost:{_c('RTK_WS_PORT',     8775) + 1}"),
    ("nav",           f"ws://localhost:{_c('NAV_WS_PORT',     8785) + 1}"),
    ("robot",         f"ws://localhost:{_c('ROBOT_WS_PORT',   8888) + 1}"),
    ("autonav",       f"ws://localhost:{_c('AUTONAV_WS_PORT', 8805) + 1}"),
    ("camera_status", f"ws://localhost:{_c('CAM_WS_PORT',     8815) + 1}"),
]

CAM_STREAMS = [
    ("cam1", f"http://localhost:{_c('CAM1_STREAM_PORT', 8080)}"),
    ("cam2", f"http://localhost:{_c('CAM2_STREAM_PORT', 8081)}"),
]

CAM_FPS    = _c("CAM_FPS",    25)
CAM_WIDTH  = _c("CAM_WIDTH",  640)
CAM_HEIGHT = _c("CAM_HEIGHT", 400)

RECONNECT_DELAY_S = 2.0


# ── WebSocket recorder (async) ────────────────────────────────────────────────
async def _ws_record(name: str, url: str, out_path: Path,
                     stop_event: asyncio.Event) -> None:
    count = 0
    with out_path.open("w", encoding="utf-8") as f:
        while not stop_event.is_set():
            try:
                async with websockets.connect(url, open_timeout=3) as ws:
                    print(f"  [+] {name:<16} connected  → {out_path.name}")
                    async for raw in ws:
                        if stop_event.is_set():
                            break
                        try:
                            text = raw.decode() if isinstance(raw, bytes) else raw
                            obj  = json.loads(text)
                            obj["log_recv_ts"] = time.time()
                            f.write(json.dumps(obj, ensure_ascii=False, separators=(",", ":")))
                            f.write("\n")
                            f.flush()
                            count += 1
                        except Exception:
                            pass
            except asyncio.CancelledError:
                break
            except Exception as exc:
                if not stop_event.is_set():
                    print(f"  [-] {name:<16} {exc!r}  — retry in {RECONNECT_DELAY_S}s")
                    try:
                        await asyncio.wait_for(stop_event.wait(), timeout=RECONNECT_DELAY_S)
                    except asyncio.TimeoutError:
                        pass
    print(f"  [=] {name:<16} closed  ({count} frames)")


# ── Camera MJPEG recorder (thread) ────────────────────────────────────────────
def _cam_record(name: str, url: str, out_path: Path,
                stop_event: threading.Event) -> None:
    try:
        import cv2
    except ImportError:
        print(f"  [!] {name:<16} opencv not available — skipping video")
        return

    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        print(f"  [!] {name:<16} cannot open {url} — skipping video")
        return

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(out_path), fourcc, CAM_FPS, (CAM_WIDTH, CAM_HEIGHT))
    if not writer.isOpened():
        print(f"  [!] {name:<16} VideoWriter failed — skipping video")
        cap.release()
        return

    print(f"  [+] {name:<16} recording → {out_path.name}")
    frames = 0
    while not stop_event.is_set():
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue
        if frame.shape[1] != CAM_WIDTH or frame.shape[0] != CAM_HEIGHT:
            frame = cv2.resize(frame, (CAM_WIDTH, CAM_HEIGHT))
        writer.write(frame)
        frames += 1

    writer.release()
    cap.release()
    print(f"  [=] {name:<16} closed  ({frames} frames)")


# ── Main ──────────────────────────────────────────────────────────────────────
def _fmt_size(path: Path) -> str:
    if not path.exists():
        return "missing"
    b = path.stat().st_size
    for unit in ("B", "KB", "MB", "GB"):
        if b < 1024:
            return f"{b:.1f} {unit}"
        b /= 1024
    return f"{b:.1f} TB"


async def main() -> None:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = ROOT / "data_log" / f"session_{ts}"
    session_dir.mkdir(parents=True, exist_ok=True)

    print(f"\nrecord_all — session: {session_dir}\n")
    print("Starting recorders…")

    # asyncio stop event (for WS tasks)
    async_stop = asyncio.Event()
    # threading stop event (for camera threads)
    thread_stop = threading.Event()

    # ── WS recorders ─────────────────────────────────────────────────────────
    ws_tasks = []
    ws_paths = []
    for name, url in WS_STREAMS:
        out = session_dir / f"{name}.jsonl"
        ws_paths.append(out)
        ws_tasks.append(asyncio.create_task(_ws_record(name, url, out, async_stop)))

    # ── Camera recorders ─────────────────────────────────────────────────────
    cam_threads = []
    cam_paths   = []
    for name, url in CAM_STREAMS:
        out = session_dir / f"{name}.mp4"
        cam_paths.append(out)
        t = threading.Thread(
            target=_cam_record,
            args=(name, url, out, thread_stop),
            daemon=True,
        )
        t.start()
        cam_threads.append(t)

    print("\nRecording… press Ctrl-C to stop.\n")

    try:
        await asyncio.sleep(float("inf"))
    except (asyncio.CancelledError, KeyboardInterrupt):
        pass

    print("\nStopping…")
    async_stop.set()
    thread_stop.set()

    # Join camera threads without blocking the event loop
    loop = asyncio.get_event_loop()
    await asyncio.gather(*[
        loop.run_in_executor(None, lambda t=t: t.join(timeout=5))
        for t in cam_threads
    ])

    for task in ws_tasks:
        task.cancel()
    await asyncio.gather(*ws_tasks, return_exceptions=True)

    # ── Summary ───────────────────────────────────────────────────────────────
    print(f"\nSession saved to: {session_dir}\n")
    all_paths = ws_paths + cam_paths
    all_names = [n for n, _ in WS_STREAMS] + [n for n, _ in CAM_STREAMS]
    for name, path in zip(all_names, all_paths):
        print(f"  {name:<16} {_fmt_size(path):<10}  {path.name}")
    print()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
