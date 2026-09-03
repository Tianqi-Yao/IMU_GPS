"""camera_bridge.py — OAK-D camera WebSocket streaming + processing plugin bridge.

INPUT  : CameraDevice reads frames from the depthai pipeline (hardware I/O).
CORE   : plugins.FrameProcessor.process() transforms frames (stateless-or-
         light-state image processors, hot-swappable without restarting the
         camera — see plugins/__init__.py). FrameStreamServer's capture/
         process/encode are three separate stages (previously one fused loop).
OUTPUT : FrameStreamServer pushes JPEG-encoded frames as WebSocket binary
         messages over its own per-camera WS port — latest-frame-wins, stale
         frames are dropped rather than queued, the same drop-old-for-new
         semantics as UDP adapted to a transport browsers can receive;
         common.ws_server.BroadcastWsServer broadcasts camera_status;
         common.http_server.StaticFileServer serves web_static/ (dashboard +
         snapshot HTML pages).

Requires the depthai SDK and physical OAK-D hardware to actually stream;
camera open failures are logged and do not crash the bridge (other cameras/
the dashboard keep working).

Run: python camera_bridge.py
"""

import asyncio
import base64
import json
import socket
import sys
import threading
import time
import webbrowser
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple, Union

import cv2
import numpy as np
from websockets.exceptions import ConnectionClosed
from websockets.sync.server import serve as ws_serve

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

import plugins  # noqa: E402 — must come after the config/sys.path setup above,
                 # since plugins/disparity_demo.py does its own `import config`.

WS_MSG_TYPE_CAMERA_STATUS = "camera_status"
WS_MSG_VERSION = 1

DEFAULT_HTTP_PORT = _cfg.CAM_WS_PORT if _cfg else 8815
DEFAULT_CAM1_IP = _cfg.CAM1_IP if _cfg else "10.95.76.11"
DEFAULT_CAM2_IP = _cfg.CAM2_IP if _cfg else "10.95.76.10"
DEFAULT_CAM_SELECTION = 1
DEFAULT_CAM1_STREAM_PORT = _cfg.CAM1_STREAM_PORT if _cfg else 8080
DEFAULT_CAM2_STREAM_PORT = _cfg.CAM2_STREAM_PORT if _cfg else 8081
DEFAULT_CAM3_SOURCE = _cfg.CAM3_SOURCE if _cfg else 0
DEFAULT_CAM3_STREAM_PORT = _cfg.CAM3_STREAM_PORT if _cfg else 8082
DEFAULT_FPS = _cfg.CAM_FPS if _cfg else 25
DEFAULT_WIDTH = _cfg.CAM_WIDTH if _cfg else 640
DEFAULT_HEIGHT = _cfg.CAM_HEIGHT if _cfg else 400
DEFAULT_MJPEG_QUALITY = _cfg.CAM_MJPEG_QUALITY if _cfg else 80
DEFAULT_PLUGIN = _cfg.CAM_DEFAULT_PLUGIN if _cfg else "simple_color"
DEFAULT_STEREO = _cfg.CAM_ENABLE_STEREO if _cfg else False
DEFAULT_DISPARITY = _cfg.CAM_ENABLE_DISPARITY if _cfg else False

logger = setup_logger(__name__, str(Path(__file__).parent / "camera_bridge.log"))


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 1 — DATA MODEL
# ══════════════════════════════════════════════════════════════════════════
@dataclass
class CameraFrame:
    cam_selection: int
    streaming: bool
    fps: float
    width: int
    height: int
    stream_url_cam1: str
    stream_url_cam2: str
    stream_url_cam3: str
    cam1_clients: int
    cam2_clients: int
    cam3_clients: int
    cam1_streaming: bool
    cam2_streaming: bool
    cam3_streaming: bool
    cam1_fps: float
    cam2_fps: float
    cam3_fps: float
    active_plugin: str
    active_plugin_config: dict
    available_plugins: List[dict]
    available_streams: List[str]

    def to_dict(self) -> dict:
        return {
            "type": WS_MSG_TYPE_CAMERA_STATUS, "version": WS_MSG_VERSION,
            "cam_selection": self.cam_selection, "streaming": self.streaming,
            "fps": round(self.fps, 1), "width": self.width, "height": self.height,
            "stream_url_cam1": self.stream_url_cam1, "stream_url_cam2": self.stream_url_cam2,
            "stream_url_cam3": self.stream_url_cam3,
            "cam1_clients": self.cam1_clients, "cam2_clients": self.cam2_clients,
            "cam3_clients": self.cam3_clients,
            "cam1_streaming": self.cam1_streaming, "cam2_streaming": self.cam2_streaming,
            "cam3_streaming": self.cam3_streaming,
            "cam1_fps": round(self.cam1_fps, 1), "cam2_fps": round(self.cam2_fps, 1),
            "cam3_fps": round(self.cam3_fps, 1),
            "active_plugin": self.active_plugin, "active_plugin_config": self.active_plugin_config,
            "available_plugins": self.available_plugins, "available_streams": self.available_streams,
        }


def generate_snapshot_html(output_frame: np.ndarray, raw_frames: Dict[str, np.ndarray], plugin_name: str) -> str:
    """Fills web_static/snapshot_template.html with the captured frame + raw
    per-stream sensor data, for offline pixel-level inspection in a browser.
    """
    template_path = Path(__file__).parent / "web_static" / "snapshot_template.html"
    template = template_path.read_text(encoding="utf-8")

    ok, buf = cv2.imencode(".png", output_frame)
    img_b64 = base64.b64encode(buf.tobytes()).decode("ascii") if ok else ""

    raw_data = {}
    for name, frame in raw_frames.items():
        if frame is None:
            continue
        raw_data[name] = {
            "dtype": str(frame.dtype),
            "shape": list(frame.shape),
            "data_b64": base64.b64encode(frame.tobytes()).decode("ascii"),
        }

    html = template
    html = html.replace("__TS__", datetime.now().isoformat(timespec="seconds"))
    html = html.replace("__PLUGIN__", plugin_name)
    html = html.replace("__W__", str(output_frame.shape[1]))
    html = html.replace("__H__", str(output_frame.shape[0]))
    html = html.replace("__IMG_B64__", img_b64)
    html = html.replace("__RAW_JSON__", json.dumps(raw_data))
    return html


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 2 — I/O ADAPTER: hardware (depthai pipeline)
# ══════════════════════════════════════════════════════════════════════════
class CameraDevice:
    """Owns one OAK-D device's depthai pipeline. No image processing here."""

    _RGB_SOCKET = "CAM_A"
    _LEFT_SOCKET = "CAM_B"
    _RIGHT_SOCKET = "CAM_C"

    def __init__(self, ip: str, fps: int, width: int, height: int,
                 enable_stereo: bool, enable_disparity: bool) -> None:
        self._ip = ip
        self._fps = fps
        self._width = width
        self._height = height
        self._enable_stereo = enable_stereo
        self._enable_disparity = enable_disparity
        self._device = None
        self._pipeline = None
        self._queues: Dict[str, object] = {}
        self._last_frames: Dict[str, np.ndarray] = {}
        self._last_raw_frames: Dict[str, np.ndarray] = {}

    def open(self) -> None:
        import depthai as dai

        try:
            self._device = dai.Device(dai.DeviceInfo(self._ip)) if self._ip else dai.Device()
            pipeline = dai.Pipeline(self._device)
            self._pipeline = pipeline

            cam_rgb = pipeline.create(dai.node.Camera).build(getattr(dai.CameraBoardSocket, self._RGB_SOCKET))
            rgb_out = cam_rgb.requestOutput(size=(self._width, self._height), fps=float(self._fps))
            self._queues["rgb"] = rgb_out.createOutputQueue(maxSize=1, blocking=False)

            if self._enable_stereo:
                cam_left = pipeline.create(dai.node.Camera).build(getattr(dai.CameraBoardSocket, self._LEFT_SOCKET))
                cam_right = pipeline.create(dai.node.Camera).build(getattr(dai.CameraBoardSocket, self._RIGHT_SOCKET))
                stereo = pipeline.create(dai.node.StereoDepth)
                stereo.setExtendedDisparity(True)
                left_out = cam_left.requestOutput(size=(640, 400), fps=float(self._fps))
                right_out = cam_right.requestOutput(size=(640, 400), fps=float(self._fps))
                left_out.link(stereo.left)
                right_out.link(stereo.right)
                self._queues["depth"] = stereo.depth.createOutputQueue(maxSize=1, blocking=False)
                if self._enable_disparity:
                    self._queues["disparity"] = stereo.disparity.createOutputQueue(maxSize=1, blocking=False)

            pipeline.start()
        except Exception:
            logger.error("Failed to open camera device (ip=%s)", self._ip, exc_info=True)
            self.close()
            raise

    def close(self) -> None:
        self._queues = {}
        self._last_frames = {}
        self._last_raw_frames = {}
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception:
                pass
            self._pipeline = None
        if self._device is not None:
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None

    def get_frames(self, stream_names: List[str]) -> Tuple[Dict[str, Optional[np.ndarray]], bool]:
        has_new_frame = False
        for name in stream_names:
            q = self._queues.get(name)
            if q is None:
                continue
            try:
                pkt = q.tryGet()
            except Exception:
                logger.warning("Failed to read packet from stream '%s'", name, exc_info=True)
                continue
            if pkt is None:
                continue
            has_new_frame = True
            try:
                raw = pkt.getCvFrame()
            except Exception:
                logger.warning("Failed to decode frame from stream '%s'", name, exc_info=True)
                continue
            self._last_raw_frames[name] = raw
            if name in ("depth", "disparity") and raw.ndim == 2:
                try:
                    normalized = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
                    self._last_frames[name] = cv2.applyColorMap(normalized, cv2.COLORMAP_JET)
                except Exception:
                    logger.warning("Failed to colorize stream '%s'; using raw frame", name, exc_info=True)
                    self._last_frames[name] = raw
            else:
                self._last_frames[name] = raw
        return {name: self._last_frames.get(name) for name in stream_names}, has_new_frame

    def available_streams(self) -> List[str]:
        return list(self._queues.keys())

    def get_raw_snapshot(self) -> Dict[str, np.ndarray]:
        return dict(self._last_raw_frames)


class WebcamDevice:
    """Owns one plain USB/webcam device via OpenCV VideoCapture. Duck-types
    the same interface as CameraDevice so FrameStreamServer/CameraPipeline
    don't need to know which kind of camera they're driving. Only ever
    provides a single "rgb" stream — no depth/disparity — so plugins that
    ask for those just get None for them, same as an OAK-D with stereo off.
    """

    def __init__(self, source, fps: int, width: int, height: int) -> None:
        self._source = source
        self._fps = fps
        self._width = width
        self._height = height
        self._cap = None
        self._last_frame: Optional[np.ndarray] = None

    def open(self) -> None:
        cap = cv2.VideoCapture(self._source)
        if not cap.isOpened():
            cap.release()
            raise RuntimeError(f"Cannot open webcam source {self._source!r}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._height)
        cap.set(cv2.CAP_PROP_FPS, self._fps)
        self._cap = cap

    def close(self) -> None:
        self._last_frame = None
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None

    def get_frames(self, stream_names: List[str]) -> Tuple[Dict[str, Optional[np.ndarray]], bool]:
        if self._cap is None or "rgb" not in stream_names:
            return {name: None for name in stream_names}, False
        ret, frame = self._cap.read()
        if ret:
            self._last_frame = frame
        return {name: (self._last_frame if name == "rgb" else None) for name in stream_names}, ret

    def available_streams(self) -> List[str]:
        return ["rgb"] if self._cap is not None else []

    def get_raw_snapshot(self) -> Dict[str, np.ndarray]:
        return {"rgb": self._last_frame} if self._last_frame is not None else {}


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 3 — CORE + OUTPUT: per-camera WebSocket frame server
# ══════════════════════════════════════════════════════════════════════════
class FrameStreamServer:
    """Runs one camera's capture -> process -> encode pipeline and pushes
    the result as WebSocket binary frames over its own WS port. The three
    pipeline stages are separate methods so each can be reasoned about (and
    tested) alone. Delivery is latest-frame-wins: a client that's still
    sending when newer frames land simply gets the newest one on its next
    turn, stale frames are dropped rather than queued.
    """

    def __init__(self, device: CameraDevice, port: int, quality: int, cam_id: int) -> None:
        self._device = device
        self._port = port
        self._quality = quality
        self._cam_id = cam_id

        self._latest_jpeg = b""
        self._frame_id = 0
        self._frame_lock = threading.Lock()
        self._frame_cond = threading.Condition(self._frame_lock)
        self._running = False

        self._fps_times: List[float] = []
        self._current_fps = 0.0

        self._client_count = 0
        self._client_lock = threading.Lock()

        self._processor = None
        self._processor_lock = threading.Lock()

        self._latest_output_frame: Optional[np.ndarray] = None
        self._output_shape: Optional[Tuple[int, int]] = None  # (height, width) of the last encoded frame
        self._output_lock = threading.Lock()

        self._wsd = None  # Optional[websockets.sync.server.Server]

    @property
    def port(self) -> int:
        return self._port

    @property
    def streaming(self) -> bool:
        return self._running

    @property
    def fps(self) -> float:
        return self._current_fps

    @property
    def client_count(self) -> int:
        with self._client_lock:
            return self._client_count

    def set_processor(self, processor) -> None:
        with self._processor_lock:
            self._processor = processor

    def reconfigure_processor(self, **kwargs) -> None:
        with self._processor_lock:
            proc = self._processor
        if proc is not None:
            proc.reconfigure(**kwargs)

    def get_latest_output_frame(self) -> Optional[np.ndarray]:
        with self._output_lock:
            return self._latest_output_frame

    @property
    def output_shape(self) -> Optional[Tuple[int, int]]:
        """(height, width) of the most recently encoded frame, or None if
        nothing has been encoded yet (e.g. camera not open, or plugin never
        produced output). Reflects the plugin's actual rendered size, which
        can differ from the configured capture resolution (e.g. path_cam's
        composite mode concatenates multiple panels side by side).
        """
        with self._output_lock:
            return self._output_shape

    def start_serving(self) -> None:
        if self._running:
            return
        self._running = True
        threading.Thread(target=self._capture_loop, name=f"stream-capture-{self._port}", daemon=True).start()
        threading.Thread(target=self._run_ws, name=f"stream-ws-{self._port}", daemon=True).start()

    def stop_serving(self) -> None:
        self._running = False
        if self._wsd is not None:
            try:
                self._wsd.shutdown()
            except Exception:
                pass
            self._wsd = None
        with self._frame_cond:
            self._latest_jpeg = b""
            self._frame_cond.notify_all()  # wake any client threads blocked in _handle_ws
        self._current_fps = 0.0
        with self._client_lock:
            self._client_count = 0

    # ---- capture / process / encode -------------------------------------
    def _capture(self) -> Tuple[Dict[str, Optional[np.ndarray]], bool]:
        with self._processor_lock:
            proc = self._processor
        if proc is None:
            return {}, False
        return self._device.get_frames(proc.required_streams())

    def _process(self, frames: Dict[str, Optional[np.ndarray]]) -> Optional[np.ndarray]:
        with self._processor_lock:
            proc = self._processor
        if proc is None:
            return None
        # Tag with this server's own camera id so processors that need to
        # single out one camera among several concurrent capture threads
        # (e.g. pose_control's driver-camera lock) don't have to guess.
        frames["_cam_id"] = self._cam_id
        try:
            return proc.process(frames)
        except Exception:
            logger.warning("Plugin process() failed", exc_info=True)
            return None

    def _encode(self, output: np.ndarray) -> Optional[bytes]:
        ok, buf = cv2.imencode(".jpg", output, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        return buf.tobytes() if ok else None

    def _capture_loop(self) -> None:
        while self._running:
            frames, has_new_frame = self._capture()
            output = self._process(frames)

            if output is None:
                # No processor/frames at all (e.g. camera not open) — nothing
                # to poll for, so back off well below frame-rate cadence.
                time.sleep(0.02)
                continue

            with self._output_lock:
                self._latest_output_frame = output
                self._output_shape = output.shape[:2]

            if not has_new_frame:
                # depthai delivers at most one frame per ~1/fps; polling much
                # faster than that just burns CPU on tryGet() with nothing to
                # return, which is what drove sustained load during camera
                # sessions. Tightened to 5ms (from 8ms) per an explicit
                # max-speed request — still far above the risk zone that
                # caused the earlier system-wide load spike, which came from
                # several bridges cold-starting at once, not from one
                # steady-state poll loop.
                time.sleep(0.005)
                continue

            jpeg = self._encode(output)
            if jpeg is not None:
                with self._frame_cond:
                    self._latest_jpeg = jpeg
                    self._frame_id += 1
                    self._frame_cond.notify_all()  # wake do_GET waiters instantly instead of on their next poll
                self._tick_fps()

    def _tick_fps(self) -> None:
        now = time.monotonic()
        self._fps_times.append(now)
        # Keep only timestamps from the last 2 seconds.
        cutoff = now - 2.0
        self._fps_times = [t for t in self._fps_times if t >= cutoff]
        if len(self._fps_times) >= 2:
            elapsed = self._fps_times[-1] - self._fps_times[0]
            if elapsed >= 0.5:  # require >=0.5s of samples so startup doesn't report an inflated rate
                self._current_fps = (len(self._fps_times) - 1) / elapsed

    def _handle_ws(self, connection) -> None:
        try:
            connection.socket.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError:
            pass
        with self._client_lock:
            self._client_count += 1
        try:
            last_sent_id = -1
            while self._running:
                with self._frame_cond:
                    self._frame_cond.wait_for(
                        lambda: not self._running or self._frame_id != last_sent_id,
                        timeout=1.0,
                    )
                    current_id = self._frame_id
                    jpeg = self._latest_jpeg
                if not self._running:
                    break
                if current_id == last_sent_id or not jpeg:
                    continue
                last_sent_id = current_id
                try:
                    connection.send(jpeg)
                except (ConnectionClosed, OSError):
                    break
        finally:
            with self._client_lock:
                self._client_count -= 1
            try:
                connection.close()
            except Exception:
                pass

    def _run_ws(self) -> None:
        try:
            self._wsd = ws_serve(self._handle_ws, "0.0.0.0", self._port)
            self._wsd.serve_forever()
        except Exception:
            logger.error("WS video server failed on port %d", self._port, exc_info=True)


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 4 — orchestration: N cameras + plugin switching
# ══════════════════════════════════════════════════════════════════════════
class CameraPipeline:
    def __init__(self, cam_configs: Dict[int, dict], default_plugin: str, default_plugin_config: dict) -> None:
        self._devices: Dict[int, Union[CameraDevice, WebcamDevice]] = {}
        self._servers: Dict[int, FrameStreamServer] = {}
        self._active_plugin_name = default_plugin
        self._active_plugin_config = dict(default_plugin_config)
        self._cam_selection = min(cam_configs.keys()) if cam_configs else 1

        first_cfg = next(iter(cam_configs.values()), None)
        self._width = first_cfg["width"] if first_cfg else DEFAULT_WIDTH
        self._height = first_cfg["height"] if first_cfg else DEFAULT_HEIGHT

        for cam_id, cfg in cam_configs.items():
            if cfg["type"] == "webcam":
                self._devices[cam_id] = WebcamDevice(
                    source=cfg["source"], fps=cfg["fps"], width=cfg["width"], height=cfg["height"],
                )
            else:
                self._devices[cam_id] = CameraDevice(
                    ip=cfg["ip"], fps=cfg["fps"], width=cfg["width"], height=cfg["height"],
                    enable_stereo=cfg["enable_stereo"], enable_disparity=cfg["enable_disparity"],
                )
            self._servers[cam_id] = FrameStreamServer(self._devices[cam_id], cfg["stream_port"], cfg["quality"], cam_id=cam_id)

        self._open_all()

    def _open_all(self) -> None:
        proc_cls = plugins.get_processor(self._active_plugin_name)
        processor = proc_cls(cam_id=self._cam_selection, **self._active_plugin_config)
        for cam_id, device in self._devices.items():
            try:
                device.open()
            except Exception:
                logger.error("Camera %d failed to open; continuing without it.", cam_id)
            server = self._servers[cam_id]
            server.set_processor(processor)
            server.start_serving()

    def switch_camera(self, cam_id: int) -> None:
        if cam_id in self._servers:
            self._cam_selection = cam_id

    def switch_plugin(self, name: str, config: dict, cam_id: Optional[int] = None) -> None:
        # cam_id designates which camera drives plugins that must single one
        # out (e.g. pose_control): the camera the browser had selected when
        # it clicked Apply, defaulting to the pipeline's current selection.
        driver_cam_id = cam_id if cam_id is not None else self._cam_selection
        proc_cls = plugins.get_processor(name)
        processor = proc_cls(cam_id=driver_cam_id, **config)
        self._active_plugin_name = name
        self._active_plugin_config = dict(config)
        for server in self._servers.values():
            server.set_processor(processor)

    def update_plugin_config(self, config: dict) -> None:
        self._active_plugin_config = {**self._active_plugin_config, **config}
        for server in self._servers.values():
            server.reconfigure_processor(**config)

    def restart_cameras(self, fps: int, width: int, height: int) -> None:
        for server in self._servers.values():
            server.stop_serving()
        self._width, self._height = width, height
        for device in self._devices.values():
            device._fps, device._width, device._height = fps, width, height
            device.close()
            try:
                device.open()
            except Exception:
                logger.error("Camera failed to reopen during restart", exc_info=True)
        proc_cls = plugins.get_processor(self._active_plugin_name)
        processor = proc_cls(cam_id=self._cam_selection, **self._active_plugin_config)
        for server in self._servers.values():
            server.set_processor(processor)
            server.start_serving()

    def start_serving(self, cam_id: Optional[int] = None) -> None:
        targets = self._servers.values() if cam_id is None else [self._servers[cam_id]]
        for server in targets:
            if not server.streaming:
                server.start_serving()

    def stop_serving(self, cam_id: Optional[int] = None) -> None:
        targets = self._servers.values() if cam_id is None else [self._servers[cam_id]]
        for server in targets:
            if server.streaming:
                server.stop_serving()

    def get_status(self) -> CameraFrame:
        s1, s2, s3 = self._servers.get(1), self._servers.get(2), self._servers.get(3)
        active_server = self._servers.get(self._cam_selection)
        active_fps = active_server.fps if active_server else 0.0
        # Prefer the active plugin's actual rendered output size (e.g. path_cam's
        # composite mode is wider than the capture resolution); fall back to the
        # configured capture resolution before any frame has been encoded.
        output_shape = active_server.output_shape if active_server else None
        width = output_shape[1] if output_shape else self._width
        height = output_shape[0] if output_shape else self._height
        streams: set = set()
        for device in self._devices.values():
            streams.update(device.available_streams())
        return CameraFrame(
            cam_selection=self._cam_selection,
            streaming=any(s.streaming for s in self._servers.values()),
            fps=active_fps, width=width, height=height,
            stream_url_cam1=f"ws://{{host}}:{s1.port}/" if s1 else "",
            stream_url_cam2=f"ws://{{host}}:{s2.port}/" if s2 else "",
            stream_url_cam3=f"ws://{{host}}:{s3.port}/" if s3 else "",
            cam1_clients=s1.client_count if s1 else 0,
            cam2_clients=s2.client_count if s2 else 0,
            cam3_clients=s3.client_count if s3 else 0,
            cam1_streaming=s1.streaming if s1 else False,
            cam2_streaming=s2.streaming if s2 else False,
            cam3_streaming=s3.streaming if s3 else False,
            cam1_fps=s1.fps if s1 else 0.0,
            cam2_fps=s2.fps if s2 else 0.0,
            cam3_fps=s3.fps if s3 else 0.0,
            active_plugin=self._active_plugin_name,
            active_plugin_config=self._active_plugin_config,
            available_plugins=plugins.list_plugins(),
            available_streams=sorted(streams),
        )

    def take_snapshot(self, cam_id: int, snapshots_dir: Path) -> str:
        server = self._servers.get(cam_id)
        device = self._devices.get(cam_id)
        if server is None or device is None:
            raise RuntimeError(f"Unknown camera id: {cam_id}")
        output_frame = server.get_latest_output_frame()
        if output_frame is None:
            raise RuntimeError("No frame available yet")
        raw_frames = device.get_raw_snapshot()

        snapshots_dir.mkdir(parents=True, exist_ok=True)
        filename = f"snap_{datetime.now().strftime('%Y%m%d_%H%M%S')}.html"
        html = generate_snapshot_html(output_frame, raw_frames, self._active_plugin_name)
        (snapshots_dir / filename).write_text(html, encoding="utf-8")
        return f"/snapshots/{filename}"

    def shutdown(self) -> None:
        for server in self._servers.values():
            try:
                server.stop_serving()
            except Exception:
                pass
        for device in self._devices.values():
            try:
                device.close()
            except Exception:
                pass


# ══════════════════════════════════════════════════════════════════════════
# BLOCK 5 — APPLICATION
# ══════════════════════════════════════════════════════════════════════════
class CameraBridge:
    def __init__(self, http_port: int, cam1_ip: str, cam2_ip: str, cam_selection: int,
                 cam1_stream_port: int, cam2_stream_port: int, fps: int, width: int, height: int,
                 quality: int, default_plugin: str, enable_stereo: bool, enable_disparity: bool,
                 static_dir: Path, cam3_source=None, cam3_stream_port: int = 8082) -> None:
        self._http_port = http_port
        self._ws_port = derive_ws_port(http_port)
        self._static_dir = static_dir
        self._snapshots_dir = static_dir / "snapshots"

        cam_configs = {}
        if cam1_ip:
            cam_configs[1] = {"type": "oakd", "ip": cam1_ip, "fps": fps, "width": width, "height": height,
                               "enable_stereo": enable_stereo, "enable_disparity": enable_disparity,
                               "stream_port": cam1_stream_port, "quality": quality}
        if cam2_ip:
            cam_configs[2] = {"type": "oakd", "ip": cam2_ip, "fps": fps, "width": width, "height": height,
                               "enable_stereo": enable_stereo, "enable_disparity": enable_disparity,
                               "stream_port": cam2_stream_port, "quality": quality}
        if cam3_source is not None:
            cam_configs[3] = {"type": "webcam", "source": cam3_source, "fps": fps, "width": width, "height": height,
                               "stream_port": cam3_stream_port, "quality": quality}

        self._pipeline = CameraPipeline(cam_configs, default_plugin, {})
        self._pipeline.switch_camera(cam_selection)

    def run(self) -> None:
        if self._static_dir.exists():
            StaticFileServer(self._static_dir, self._http_port).start()
        else:
            logger.warning("Static dir %s not found; HTTP page disabled.", self._static_dir)

        threading.Timer(1.0, lambda: webbrowser.open(f"http://localhost:{self._http_port}")).start()

        try:
            asyncio.run(self._run_async())
        except KeyboardInterrupt:
            logger.info("Stopped")
        finally:
            self._pipeline.shutdown()

    async def _run_async(self) -> None:
        queue: "asyncio.Queue[dict]" = asyncio.Queue(maxsize=5)
        ws_server = BroadcastWsServer("0.0.0.0", self._ws_port, on_client_message=self._handle_client_message)

        async def _status_loop() -> None:
            while True:
                await asyncio.sleep(1.0)
                await ws_server.broadcast(self._pipeline.get_status().to_dict())

        await asyncio.gather(_status_loop(), ws_server.serve())

    async def _handle_client_message(self, _websocket, raw: str) -> Optional[str]:
        try:
            msg = json.loads(raw)
        except json.JSONDecodeError:
            return None
        t = msg.get("type")

        if t == "start_stream":
            self._pipeline.start_serving(msg.get("cam_id"))
        elif t == "stop_stream":
            self._pipeline.stop_serving(msg.get("cam_id"))
        elif t == "switch_camera":
            self._pipeline.switch_camera(int(msg.get("cam_id", 1)))
        elif t == "switch_plugin":
            try:
                cam_id = msg.get("cam_id")
                self._pipeline.switch_plugin(
                    msg.get("plugin_name", ""), msg.get("config", {}),
                    cam_id=int(cam_id) if cam_id is not None else None,
                )
            except Exception as exc:
                logger.warning("switch_plugin failed: %s", exc, exc_info=True)
                return json.dumps({"type": "switch_plugin_error", "version": WS_MSG_VERSION, "error": str(exc)})
        elif t == "update_plugin_config":
            self._pipeline.update_plugin_config(msg.get("config", {}))
        elif t == "restart_cameras":
            # Closing/reopening the device is a blocking, potentially
            # multi-second operation; run it off the event loop so WS
            # message handling and status broadcasts don't stall.
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(
                None, self._pipeline.restart_cameras,
                int(msg.get("fps", DEFAULT_FPS)), int(msg.get("width", DEFAULT_WIDTH)),
                int(msg.get("height", DEFAULT_HEIGHT)),
            )
        elif t == "snapshot":
            try:
                url = self._pipeline.take_snapshot(int(msg.get("cam_id", 1)), self._snapshots_dir)
                return json.dumps({"type": "snapshot_ready", "version": WS_MSG_VERSION, "url": url})
            except Exception as exc:
                return json.dumps({"type": "snapshot_error", "version": WS_MSG_VERSION, "error": str(exc)})
        return None


if __name__ == "__main__":
    CameraBridge(
        http_port=DEFAULT_HTTP_PORT,
        cam1_ip=DEFAULT_CAM1_IP,
        cam2_ip=DEFAULT_CAM2_IP,
        cam_selection=DEFAULT_CAM_SELECTION,
        cam1_stream_port=DEFAULT_CAM1_STREAM_PORT,
        cam2_stream_port=DEFAULT_CAM2_STREAM_PORT,
        cam3_source=DEFAULT_CAM3_SOURCE,
        cam3_stream_port=DEFAULT_CAM3_STREAM_PORT,
        fps=DEFAULT_FPS,
        width=DEFAULT_WIDTH,
        height=DEFAULT_HEIGHT,
        quality=DEFAULT_MJPEG_QUALITY,
        default_plugin=DEFAULT_PLUGIN,
        enable_stereo=DEFAULT_STEREO,
        enable_disparity=DEFAULT_DISPARITY,
        static_dir=Path(__file__).parent / "web_static",
    ).run()
