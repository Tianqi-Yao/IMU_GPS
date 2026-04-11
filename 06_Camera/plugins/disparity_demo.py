"""
DisparityDemoProcessor — minimal disparity demo.

Shows the raw disparity map colourised with COLORMAP_JET.
Higher value (red) = closer object. Lower value (blue) = farther away.

Requires CAM_ENABLE_STEREO=True and CAM_ENABLE_DISPARITY=True in config.py.
"""

from __future__ import annotations

import numpy as np
import cv2

from . import FrameProcessor, register_processor


@register_processor
class DisparityDemoProcessor(FrameProcessor):

    PROCESSOR_NAME = "disparity_demo"
    PROCESSOR_LABEL = "Disparity Demo"
    PROCESSOR_DESCRIPTION = "Raw disparity map — red=near, blue=far (requires CAM_ENABLE_DISPARITY=True)"

    @classmethod
    def required_streams(cls) -> list[str]:
        return ["disparity"]

    @classmethod
    def config_schema(cls) -> list[dict]:
        return []

    def process(self, frames: dict[str, np.ndarray | None]) -> np.ndarray | None:
        disp = frames.get("disparity")
        if disp is None:
            # No stereo stream — return a black frame with a hint
            blank = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(blank, "Please set CAM_ENABLE_DISPARITY = True in config.py",
                        (20, 240), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 60, 255), 2, cv2.LINE_AA)
            return blank
        return disp  # CameraDevice already colourises disparity
