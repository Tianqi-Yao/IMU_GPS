"""sysmon_bridge.py — macOS system-health (CPU thermal/load/memory) -> WebSocket bridge.

INPUT  : SysSampler shells out to `pmset -g therm` (CPU speed/scheduler
         limit — the no-sudo Apple Silicon thermal-throttle signal),
         os.getloadavg()/os.cpu_count() (CPU pressure independent of
         thermal state), `sysctl vm.swapusage` and `vm_stat` (memory
         pressure — a third alternate cause of the same symptom).
CORE   : none — this bridge has no accumulating state, just a periodic
         sample() call (unlike e.g. rtk_bridge's NMEA accumulator).
OUTPUT : common.ws_server.BroadcastWsServer broadcasts sysmon_frame payloads
         at a fixed rate; JsonlWriter appends every sample to a
         day-rotating file under sysmon_data/ so history exists even if
         nobody is running record_all.py; common.http_server.StaticFileServer
         serves web_static/ (live dashboard + historical-log-import viewer).

Written to help track down intermittent low camera FPS on an outdoor
Mac mini, suspected to be thermal throttling — see 07_SysMon/README.md.

Run: python sysmon_bridge.py
"""

import asyncio
import json
import os
import re
import subprocess
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
try:
    import config as _cfg
except ImportError:
    _cfg = None

from common.http_server import StaticFileServer
from common.logging_setup import setup_logger
from common.ports import derive_ws_port
from common.ws_server import BroadcastWsServer

WS_MSG_TYPE_SYSMON_FRAME = "sysmon_frame"
WS_MSG_VERSION = 1

DEFAULT_HTTP_PORT = _cfg.SYSMON_WS_PORT if _cfg else 8825
DEFAULT_INTERVAL_S = _cfg.SYSMON_INTERVAL_S if _cfg else 2.0

logger = setup_logger(__name__, str(Path(__file__).parent / "sysmon_bridge.log"))

DATA_DIR = Path(__file__).parent / "sysmon_data"


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 1 — DATA MODEL
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class SysMonFrame:
    cpu_speed_limit: Optional[int]       # % of full speed; 100 = no limit; from `pmset -g therm`
    cpu_scheduler_limit: Optional[int]   # % of cores schedulable; 100 = no limit
    throttled: bool                      # True if either limit above is below 100
    loadavg_1: float
    loadavg_5: float
    loadavg_15: float
    cpu_count: int
    swap_used_mb: Optional[float]
    mem_pressure_level: Optional[int]    # kern.memorystatus_vm_pressure_level: 1=normal, 2=warn, 4=critical
    ts: float

    def to_dict(self) -> dict:
        return {
            "type": WS_MSG_TYPE_SYSMON_FRAME, "version": WS_MSG_VERSION,
            "cpu_speed_limit": self.cpu_speed_limit,
            "cpu_scheduler_limit": self.cpu_scheduler_limit,
            "throttled": self.throttled,
            "loadavg_1": self.loadavg_1, "loadavg_5": self.loadavg_5, "loadavg_15": self.loadavg_15,
            "cpu_count": self.cpu_count,
            "swap_used_mb": self.swap_used_mb,
            "mem_pressure_level": self.mem_pressure_level,
            "ts": self.ts,
        }


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 2 — CORE: system sampling (no accumulated state)
# ══════════════════════════════════════════════════════════════════════════
class SysSampler:
    """Reads OS-level thermal/load/memory signals. Each source is sampled
    independently so a parsing failure in one (e.g. a macOS version that
    changes `pmset`/`vm_stat` output format) degrades only that field
    instead of the whole frame.
    """

    _THERM_RE = re.compile(r"^(CPU_Speed_Limit|CPU_Scheduler_Limit)\s*=\s*(\d+)")
    _SWAP_RE = re.compile(r"used\s*=\s*([\d.]+)M")
    _MEM_PRESSURE_RE = re.compile(r"kern\.memorystatus_vm_pressure_level:\s*(\d+)")

    def sample(self) -> SysMonFrame:
        speed_limit, scheduler_limit = self._read_therm()
        loadavg_1, loadavg_5, loadavg_15 = os.getloadavg()
        return SysMonFrame(
            cpu_speed_limit=speed_limit,
            cpu_scheduler_limit=scheduler_limit,
            throttled=(speed_limit is not None and speed_limit < 100)
            or (scheduler_limit is not None and scheduler_limit < 100),
            loadavg_1=loadavg_1, loadavg_5=loadavg_5, loadavg_15=loadavg_15,
            cpu_count=os.cpu_count() or 1,
            swap_used_mb=self._read_swap_used_mb(),
            mem_pressure_level=self._read_mem_pressure_level(),
            ts=time.time(),
        )

    @classmethod
    def _read_therm(cls) -> tuple:
        try:
            out = subprocess.run(["pmset", "-g", "therm"], capture_output=True, text=True, timeout=2).stdout
        except Exception:
            return None, None
        values = {}
        for line in out.splitlines():
            m = cls._THERM_RE.match(line.strip())
            if m:
                values[m.group(1)] = int(m.group(2))
        return values.get("CPU_Speed_Limit"), values.get("CPU_Scheduler_Limit")

    @classmethod
    def _read_swap_used_mb(cls) -> Optional[float]:
        try:
            out = subprocess.run(["sysctl", "vm.swapusage"], capture_output=True, text=True, timeout=2).stdout
            m = cls._SWAP_RE.search(out)
            return float(m.group(1)) if m else None
        except Exception:
            return None

    @classmethod
    def _read_mem_pressure_level(cls) -> Optional[int]:
        """1=normal, 2=warn, 4=critical — same signal Activity Monitor's
        memory-pressure gauge uses, so no need to re-derive it from vm_stat
        page counts (which conflate reclaimable disk cache with real
        pressure and read as "critical" almost all the time on macOS).
        """
        try:
            out = subprocess.run(["sysctl", "kern.memorystatus_vm_pressure_level"],
                                  capture_output=True, text=True, timeout=2).stdout
            m = cls._MEM_PRESSURE_RE.search(out)
            return int(m.group(1)) if m else None
        except Exception:
            return None


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 3 — I/O: broadcast loop + real-time JSONL persistence
# ══════════════════════════════════════════════════════════════════════════
class JsonlWriter:
    """Appends every sample to a day-rotating file under sysmon_data/, so a
    history exists purely from the bridge running — no manual recording
    session (record_all.py) required to catch an FPS-drop incident.
    """

    def __init__(self, data_dir: Path) -> None:
        self._data_dir = data_dir
        self._data_dir.mkdir(parents=True, exist_ok=True)

    def write(self, payload: dict) -> None:
        path = self._data_dir / f"sysmon_{datetime.now().strftime('%Y%m%d')}.jsonl"
        try:
            with path.open("a", encoding="utf-8") as f:
                f.write(json.dumps(payload, separators=(",", ":")) + "\n")
        except Exception:
            logger.warning("Failed to append sysmon sample to %s", path, exc_info=True)


class BroadcastLoop:
    """Samples at a fixed rate, pushes each frame to the WS broadcast queue,
    and appends it to the JSONL history. Logs a warning on the rising edge
    of `throttled` (not every sample) so sysmon_bridge.log alone is
    greppable evidence of a thermal-throttle event.
    """

    def __init__(self, sampler: SysSampler, writer: JsonlWriter, queue: "asyncio.Queue[dict]",
                 interval_s: float) -> None:
        self._sampler = sampler
        self._writer = writer
        self._queue = queue
        self._interval_s = interval_s
        self._was_throttled = False

    async def run(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            frame = await loop.run_in_executor(None, self._sampler.sample)
            if frame.throttled and not self._was_throttled:
                logger.warning(
                    "CPU thermal throttling detected: speed_limit=%s scheduler_limit=%s loadavg=%.2f/%.2f/%.2f",
                    frame.cpu_speed_limit, frame.cpu_scheduler_limit,
                    frame.loadavg_1, frame.loadavg_5, frame.loadavg_15,
                )
            elif not frame.throttled and self._was_throttled:
                logger.info("CPU thermal throttling cleared.")
            self._was_throttled = frame.throttled

            payload = frame.to_dict()
            self._writer.write(payload)
            try:
                self._queue.put_nowait(payload)
            except asyncio.QueueFull:
                logger.warning("BroadcastLoop: queue full, dropping frame")
            await asyncio.sleep(self._interval_s)


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 4 — APPLICATION
# ══════════════════════════════════════════════════════════════════════════
class SysMonBridge:
    def __init__(self, http_port: int, interval_s: float, static_dir: Path,
                 open_browser: bool = True) -> None:
        self._http_port = http_port
        self._ws_port = derive_ws_port(http_port)
        self._interval_s = interval_s
        self._static_dir = static_dir
        self._open_browser = open_browser

    def run(self) -> None:
        if self._static_dir.exists():
            StaticFileServer(self._static_dir, self._http_port).start()
        else:
            logger.warning("Static dir %s not found; HTTP page disabled.", self._static_dir)

        if self._open_browser:
            threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{self._http_port}")).start()

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("Stopped")

    async def _run_async(self) -> None:
        queue: "asyncio.Queue[dict]" = asyncio.Queue(maxsize=10)
        ws_server = BroadcastWsServer("0.0.0.0", self._ws_port)
        broadcast_loop = BroadcastLoop(SysSampler(), JsonlWriter(DATA_DIR), queue, self._interval_s)

        async def _drain_queue() -> None:
            while True:
                payload = await queue.get()
                await ws_server.broadcast(payload)

        await asyncio.gather(broadcast_loop.run(), _drain_queue(), ws_server.serve())


if __name__ == "__main__":
    SysMonBridge(
        http_port=DEFAULT_HTTP_PORT,
        interval_s=DEFAULT_INTERVAL_S,
        static_dir=Path(__file__).parent / "web_static",
    ).run()
