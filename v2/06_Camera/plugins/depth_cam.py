"""
DepthCamProcessor — depth visualization processor.

Receives pre-captured RGB and depth frames from CameraDevice and composes
various display modes.  No depthai code here; camera lifecycle is managed
 by CameraDevice in camera_bridge.py (requires CAM_ENABLE_STEREO=True).
"""

from __future__ import annotations

import logging

import numpy as np

try:
    import cv2
except ImportError:
    cv2 = None

from . import FrameProcessor, register_processor

logger = logging.getLogger(__name__)

@register_processor
class DepthCamProcessor(FrameProcessor):
    """
    Depth visualization processor for OAK-D stereo cameras.

    Requires CameraDevice opened with enable_stereo=True (CAM_ENABLE_STEREO=True).
    Display modes: rgb, depth, rgb+depth blend.
    """

    PROCESSOR_NAME = "depth_cam"
    PROCESSOR_LABEL = "Depth Camera"
    PROCESSOR_DESCRIPTION = "RGB/depth stream (requires CAM_ENABLE_STEREO=True)"

    @classmethod
    def required_streams(cls) -> list[str]:
        return ["rgb", "depth"]

    @classmethod
    def config_schema(cls) -> list[dict]:
        return [
            {
                "key": "display_mode",
                "type": "enum",
                "default": "depth",
                "label": "Display Mode",
                "options": [
                    {"value": "rgb",            "label": "rgb"},
                    {"value": "depth",          "label": "depth"},
                    {"value": "rgb_depth_blend","label": "rgb+depth"},
                ],
            },
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
        self._display_mode = str(kwargs.get("display_mode", "depth"))
        self._rgb_weight_percent = int(kwargs.get("rgb_weight_percent", 40))

    def reconfigure(self, **kwargs) -> None:
        if "display_mode" in kwargs:
            self._display_mode = str(kwargs["display_mode"])
        if "rgb_weight_percent" in kwargs:
            self._rgb_weight_percent = int(kwargs["rgb_weight_percent"])

    def process(self, frames: dict[str, np.ndarray | None]) -> np.ndarray | None:
        rgb   = frames.get("rgb")
        depth = frames.get("depth")

        # depth frame from CameraDevice is already colorized (BGR)
        depth_color = depth  # CameraDevice.get_frames handles colorization

        # If depth is unavailable (CAM_ENABLE_STEREO=False), overlay a warning on rgb
        if depth_color is None and rgb is not None and cv2 is not None:
            out = rgb.copy()
            cv2.putText(
                out,
                "No depth stream set CAM_ENABLE_STEREO=True in config.py",
                (10, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 60, 255),
                2,
                cv2.LINE_AA,
            )
            return out

        mode = self._display_mode

        if mode == "rgb":
            return rgb

        if mode == "depth":
            if depth_color is not None:
                return depth_color
            return rgb  # fallback

        if mode == "rgb_depth_blend":
            if rgb is not None and depth_color is not None:
                if cv2 is None:
                    return rgb
                d_resized = cv2.resize(depth_color, (rgb.shape[1], rgb.shape[0]))
                rgb_w = float(np.clip(self._rgb_weight_percent, 0, 100)) / 100.0
                dep_w = 1.0 - rgb_w
                return cv2.addWeighted(rgb, rgb_w, d_resized, dep_w, 0)
            return rgb

        # fallback
        return rgb if rgb is not None else depth_color
