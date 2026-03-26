"""
DepthCamSource — OAK-D source plugin with mono/depth visualization options.

Goal:
- Keep a single source plugin that can output RGB / mono / depth / blend views.
- Allow enabling/disabling left/right mono cameras for future depth workflows.
- Provide a stable foundation for higher-level processing on top.
"""

from __future__ import annotations

import logging
from datetime import timedelta

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

try:
    import depthai as dai
except ImportError:
    dai = None

from . import FrameSource, register_plugin

logger = logging.getLogger(__name__)


def _extract_msg(group, key: str):
    try:
        return group[key]
    except Exception:
        return None


def _colorize_depth(frame_depth: np.ndarray) -> np.ndarray:
    if cv2 is None:
        return np.zeros((frame_depth.shape[0], frame_depth.shape[1], 3), dtype=np.uint8)
    invalid_mask = frame_depth == 0
    try:
        valid = frame_depth[frame_depth != 0]
        if valid.size == 0:
            return np.zeros((frame_depth.shape[0], frame_depth.shape[1], 3), dtype=np.uint8)
        min_depth = np.percentile(valid, 3)
        max_depth = np.percentile(valid, 95)
        log_depth = np.log(frame_depth, where=frame_depth != 0)
        log_min = np.log(min_depth)
        log_max = np.log(max_depth)
        np.nan_to_num(log_depth, copy=False, nan=log_min)
        log_depth = np.clip(log_depth, log_min, log_max)
        color = np.interp(log_depth, (log_min, log_max), (0, 255))
        color = np.nan_to_num(color).astype(np.uint8)
        color = cv2.applyColorMap(color, cv2.COLORMAP_JET)
        color[invalid_mask] = 0
        return color
    except Exception:
        return np.zeros((frame_depth.shape[0], frame_depth.shape[1], 3), dtype=np.uint8)


@register_plugin
class DepthCamSource(FrameSource):
    """Depth-capable OAK-D source using DepthAI v3 pipeline nodes."""

    PLUGIN_NAME = "depth_cam"
    PLUGIN_LABEL = "Depth Camera"
    PLUGIN_DESCRIPTION = "RGB/mono/depth stream with optional depth alignment"

    @classmethod
    def config_schema(cls) -> list[dict]:
        return [
            {
                "key": "profile",
                "type": "enum",
                "default": "depth_debug",
                "label": "Preset",
                "options": [
                    {"value": "navigation", "label": "navigation"},
                    {"value": "depth_debug", "label": "depth_debug"},
                    {"value": "stereo_debug", "label": "stereo_debug"},
                ],
            },
            {
                "key": "display_mode",
                "type": "enum",
                "default": "depth",
                "label": "Display Mode",
                "options": [
                    {"value": "rgb", "label": "rgb"},
                    {"value": "depth", "label": "depth"},
                    {"value": "rgb_depth_blend", "label": "rgb+depth"},
                    {"value": "mono_left", "label": "mono_left"},
                    {"value": "mono_right", "label": "mono_right"},
                    {"value": "mono_both", "label": "mono_both"},
                ],
            },
            {"key": "enable_left_mono", "type": "bool", "default": True, "label": "Enable Left Mono"},
            {"key": "enable_right_mono", "type": "bool", "default": True, "label": "Enable Right Mono"},
            {"key": "fps", "type": "int", "default": 25, "label": "FPS"},
            {"key": "rgb_width", "type": "int", "default": 1280, "label": "RGB Width"},
            {"key": "rgb_height", "type": "int", "default": 720, "label": "RGB Height"},
            {"key": "mono_width", "type": "int", "default": 640, "label": "Mono Width"},
            {"key": "mono_height", "type": "int", "default": 400, "label": "Mono Height"},
            {
                "key": "rgb_weight_percent",
                "type": "range",
                "default": 40,
                "label": "RGB Weight % (blend)",
                "min": 0,
                "max": 100,
                "step": 1,
            },
        ]

    def __init__(self, **kwargs) -> None:
        self._device_ip = kwargs.get("device_ip")
        self._profile = kwargs.get("profile", "depth_debug")
        self._display_mode = "depth"
        self._enable_left_mono = True
        self._enable_right_mono = True
        self._fps = int(kwargs.get("fps", 25))
        self._rgb_width = int(kwargs.get("rgb_width", kwargs.get("width", 1280)))
        self._rgb_height = int(kwargs.get("rgb_height", kwargs.get("height", 720)))
        self._mono_width = int(kwargs.get("mono_width", 640))
        self._mono_height = int(kwargs.get("mono_height", 400))
        self._rgb_weight_percent = int(kwargs.get("rgb_weight_percent", 40))

        self._device = None
        self._pipeline = None
        self._queue = None
        self._apply_profile(self._profile)
        if "display_mode" in kwargs:
            self._display_mode = str(kwargs["display_mode"])
        if "enable_left_mono" in kwargs:
            self._enable_left_mono = bool(kwargs["enable_left_mono"])
        if "enable_right_mono" in kwargs:
            self._enable_right_mono = bool(kwargs["enable_right_mono"])

    def reconfigure(self, **kwargs) -> None:
        profile = kwargs.get("profile")
        if profile is not None:
            self._apply_profile(str(profile))
        if "display_mode" in kwargs:
            self._display_mode = str(kwargs["display_mode"])
        if "rgb_weight_percent" in kwargs:
            self._rgb_weight_percent = int(kwargs["rgb_weight_percent"])

    def _apply_profile(self, profile: str) -> None:
        self._profile = profile
        if profile == "navigation":
            self._display_mode = "rgb_depth_blend"
            self._enable_left_mono = True
            self._enable_right_mono = True
            if self._rgb_weight_percent == 40:
                self._rgb_weight_percent = 55
        elif profile == "stereo_debug":
            self._display_mode = "mono_both"
            self._enable_left_mono = True
            self._enable_right_mono = True
        else:
            # depth_debug default
            self._display_mode = "depth"
            self._enable_left_mono = True
            self._enable_right_mono = True

    def open(self) -> None:
        if dai is None:
            raise RuntimeError("depthai library not installed")
        if cv2 is None:
            raise RuntimeError("opencv-python library not installed")

        try:
            if self._device_ip:
                self._device = dai.Device(dai.DeviceInfo(self._device_ip))
            else:
                self._device = dai.Device()

            self._pipeline = dai.Pipeline(self._device)
            sync = self._pipeline.create(dai.node.Sync)
            sync.setSyncThreshold(timedelta(milliseconds=40))

            cam_rgb = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            rgb_out = cam_rgb.requestOutput(
                size=(self._rgb_width, self._rgb_height),
                fps=float(self._fps),
                enableUndistortion=True,
            )
            rgb_out.link(sync.inputs["rgb"])

            left_out = None
            right_out = None
            if self._enable_left_mono:
                cam_left = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
                left_out = cam_left.requestOutput(size=(self._mono_width, self._mono_height), fps=float(self._fps))
                left_out.link(sync.inputs["mono_left"])
            if self._enable_right_mono:
                cam_right = self._pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
                right_out = cam_right.requestOutput(size=(self._mono_width, self._mono_height), fps=float(self._fps))
                right_out.link(sync.inputs["mono_right"])

            if left_out is not None and right_out is not None:
                stereo = self._pipeline.create(dai.node.StereoDepth)
                stereo.setExtendedDisparity(True)
                left_out.link(stereo.left)
                right_out.link(stereo.right)
                try:
                    rgb_out.link(stereo.inputAlignTo)
                except Exception:
                    pass
                stereo.depth.link(sync.inputs["depth"])

            self._queue = sync.out.createOutputQueue(maxSize=1, blocking=False)
            self._pipeline.start()
            logger.info(
                "DepthCamSource: opened (ip=%s, profile=%s, mode=%s, monoL=%s, monoR=%s)",
                self._device_ip,
                self._profile,
                self._display_mode,
                self._enable_left_mono,
                self._enable_right_mono,
            )
        except Exception as exc:
            logger.error("DepthCamSource: failed to open camera: %s", exc)
            self.close()
            raise

    def close(self) -> None:
        if self._pipeline is not None:
            try:
                self._pipeline.stop()
            except Exception as exc:
                logger.warning("DepthCamSource: error stopping pipeline: %s", exc)
            self._pipeline = None
        if self._device is not None:
            try:
                self._device.close()
            except Exception as exc:
                logger.warning("DepthCamSource: error closing device: %s", exc)
            self._device = None
        self._queue = None

    def get_frame(self) -> np.ndarray | None:
        if self._queue is None:
            return None
        try:
            group = self._queue.tryGet()
            if group is None:
                return None
            rgb = self._extract_bgr(group, "rgb")
            mono_left = self._extract_bgr(group, "mono_left")
            mono_right = self._extract_bgr(group, "mono_right")
            depth = self._extract_depth_color(group, "depth")
            return self._compose(rgb, mono_left, mono_right, depth)
        except Exception as exc:
            logger.warning("DepthCamSource: get_frame error: %s", exc)
            return None

    def _compose(
        self,
        rgb: np.ndarray | None,
        mono_left: np.ndarray | None,
        mono_right: np.ndarray | None,
        depth: np.ndarray | None,
    ) -> np.ndarray | None:
        mode = self._display_mode
        fallback = self._first_available(depth, rgb, mono_left, mono_right)

        if mode == "rgb":
            return self._first_available(rgb, fallback)
        if mode == "mono_left":
            return self._first_available(mono_left, rgb, fallback)
        if mode == "mono_right":
            return self._first_available(mono_right, rgb, fallback)
        if mode == "mono_both":
            if mono_left is not None and mono_right is not None:
                return np.concatenate([mono_left, mono_right], axis=1)
            return self._first_available(mono_left, mono_right, rgb, fallback)
        if mode == "rgb_depth_blend":
            if rgb is not None and depth is not None:
                if cv2 is None:
                    return rgb
                depth_resized = cv2.resize(depth, (rgb.shape[1], rgb.shape[0]))
                rgb_w = float(np.clip(self._rgb_weight_percent, 0, 100)) / 100.0
                dep_w = 1.0 - rgb_w
                return cv2.addWeighted(rgb, rgb_w, depth_resized, dep_w, 0)
            return self._first_available(rgb, fallback)

        # default depth mode
        return fallback

    @staticmethod
    def _first_available(*frames: np.ndarray | None) -> np.ndarray | None:
        for frame in frames:
            if frame is not None:
                return frame
        return None

    @staticmethod
    def _extract_bgr(group, key: str) -> np.ndarray | None:
        frame = _extract_msg(group, key)
        if frame is None:
            return None
        cv_frame = frame.getCvFrame()
        if cv_frame is None:
            return None
        if len(cv_frame.shape) == 2:
            if cv2 is None:
                return None
            return cv2.cvtColor(cv_frame, cv2.COLOR_GRAY2BGR)
        return cv_frame

    @staticmethod
    def _extract_depth_color(group, key: str) -> np.ndarray | None:
        frame = _extract_msg(group, key)
        if frame is None:
            return None
        depth_frame = frame.getFrame()
        if depth_frame is None:
            return None
        return _colorize_depth(depth_frame)
