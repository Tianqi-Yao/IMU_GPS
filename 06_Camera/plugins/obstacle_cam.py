"""
ObstacleCamProcessor — near-obstacle warning using raw disparity.

Disparity value is inversely proportional to distance: higher disparity = closer object.
This processor segments the disparity map into near/mid/far zones and overlays
a warning banner on the RGB frame when close obstacles are detected in the center ROI.

display_mode options:
    "rgb_warn"   — RGB with coloured warning overlay (default)
    "disparity"  — raw colorised disparity (COLORMAP_JET)
    "composite"  — side by side: RGB+warn | disparity
"""

from __future__ import annotations

import logging

import numpy as np
import cv2

from . import FrameProcessor, register_processor

logger = logging.getLogger(__name__)

# ── Disparity thresholds (uint8 after normalisation to 0-255) ────────────────
# Higher value = closer object.
# These are tuned for OAK-D with setExtendedDisparity=True (max disp ~190 px).
_NEAR_THRESH = 200   # danger zone  (red)
_MID_THRESH  = 140   # caution zone (yellow)

# ── Centre ROI as fraction of frame ──────────────────────────────────────────
_ROI_X_MIN, _ROI_X_MAX = 0.25, 0.75
_ROI_Y_MIN, _ROI_Y_MAX = 0.20, 0.80

# ── Minimum fraction of ROI pixels that must be "near" to trigger warning ────
_NEAR_TRIGGER_FRAC = 0.05   # 5 % of ROI pixels
_MID_TRIGGER_FRAC  = 0.12   # 12 % of ROI pixels


@register_processor
class ObstacleCamProcessor(FrameProcessor):
    """
    Near-obstacle detector using stereo disparity.

    Requires CameraDevice opened with enable_stereo=True (--stereo flag).
    Segments disparity into near/mid/far zones and paints a warning overlay
    on the live RGB feed.
    """

    PROCESSOR_NAME = "obstacle_cam"
    PROCESSOR_LABEL = "Obstacle Warning"
    PROCESSOR_DESCRIPTION = (
        "Near-obstacle detection via disparity segmentation (requires --stereo)"
    )

    @classmethod
    def required_streams(cls) -> list[str]:
        return ["rgb", "disparity"]

    @classmethod
    def config_schema(cls) -> list[dict]:
        return [
            {
                "key": "display_mode",
                "type": "enum",
                "default": "rgb_warn",
                "label": "Display Mode",
                "options": [
                    {"value": "rgb_warn",   "label": "rgb+warn"},
                    {"value": "disparity",  "label": "disparity"},
                    {"value": "composite",  "label": "composite"},
                ],
            },
            {
                "key": "near_thresh",
                "type": "range",
                "default": _NEAR_THRESH,
                "label": "Near threshold (0-255)",
                "min": 50,
                "max": 255,
                "step": 5,
            },
            {
                "key": "mid_thresh",
                "type": "range",
                "default": _MID_THRESH,
                "label": "Mid threshold (0-255)",
                "min": 30,
                "max": 220,
                "step": 5,
            },
        ]

    def __init__(self, **kwargs) -> None:
        self._display_mode = str(kwargs.get("display_mode", "rgb_warn"))
        self._near_thresh  = int(kwargs.get("near_thresh", _NEAR_THRESH))
        self._mid_thresh   = int(kwargs.get("mid_thresh",  _MID_THRESH))

    def reconfigure(self, **kwargs) -> None:
        if "display_mode" in kwargs:
            self._display_mode = str(kwargs["display_mode"])
        if "near_thresh" in kwargs:
            self._near_thresh = int(kwargs["near_thresh"])
        if "mid_thresh" in kwargs:
            self._mid_thresh = int(kwargs["mid_thresh"])

    # ── helpers ───────────────────────────────────────────────────────────────

    def _no_disparity_warning(self, rgb: np.ndarray) -> np.ndarray:
        out = rgb.copy()
        cv2.putText(
            out,
            "No disparity — restart with --stereo",
            (10, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            (0, 60, 255),
            2,
            cv2.LINE_AA,
        )
        return out

    def _analyse(
        self, disp_gray: np.ndarray
    ) -> tuple[str, tuple[int, int, int], np.ndarray]:
        """
        Returns (level, bgr_colour, roi_mask_full_size).
        level: "CLEAR" | "CAUTION" | "DANGER"
        """
        h, w = disp_gray.shape[:2]
        x1 = int(w * _ROI_X_MIN)
        x2 = int(w * _ROI_X_MAX)
        y1 = int(h * _ROI_Y_MIN)
        y2 = int(h * _ROI_Y_MAX)

        roi = disp_gray[y1:y2, x1:x2]
        roi_pixels = roi.size

        near_frac = float(np.count_nonzero(roi >= self._near_thresh)) / roi_pixels
        mid_frac  = float(np.count_nonzero(roi >= self._mid_thresh))  / roi_pixels

        # Build a full-size highlight mask (near pixels only, in ROI)
        highlight = np.zeros((h, w), dtype=np.uint8)
        near_in_roi = (roi >= self._near_thresh).astype(np.uint8) * 255
        highlight[y1:y2, x1:x2] = near_in_roi

        if near_frac >= _NEAR_TRIGGER_FRAC:
            return "DANGER",  (0, 0, 220),   highlight
        if mid_frac  >= _MID_TRIGGER_FRAC:
            return "CAUTION", (0, 200, 255),  highlight
        return "CLEAR",   (0, 200, 60),   highlight

    def _overlay(
        self,
        rgb: np.ndarray,
        level: str,
        colour: tuple[int, int, int],
        highlight_mask: np.ndarray,
    ) -> np.ndarray:
        out = rgb.copy()
        h, w = out.shape[:2]

        # Tint near pixels on the RGB image
        if level != "CLEAR":
            tint = np.zeros_like(out)
            tint[highlight_mask > 0] = colour
            out = cv2.addWeighted(out, 0.7, tint, 0.3, 0)

        # ROI rectangle
        rx1, rx2 = int(w * _ROI_X_MIN), int(w * _ROI_X_MAX)
        ry1, ry2 = int(h * _ROI_Y_MIN), int(h * _ROI_Y_MAX)
        cv2.rectangle(out, (rx1, ry1), (rx2, ry2), colour, 2)

        # Banner at top
        banner_h = 44
        cv2.rectangle(out, (0, 0), (w, banner_h), colour, cv2.FILLED)
        cv2.putText(
            out,
            f"OBSTACLE: {level}",
            (12, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return out

    # ── main ──────────────────────────────────────────────────────────────────

    def process(self, frames: dict[str, np.ndarray | None]) -> np.ndarray | None:
        rgb      = frames.get("rgb")
        disp_bgr = frames.get("disparity")   # already colorised BGR by CameraDevice

        if disp_bgr is None:
            if rgb is not None:
                return self._no_disparity_warning(rgb)
            return None

        if rgb is None:
            return disp_bgr

        # Convert colorised disparity back to single-channel for analysis.
        # CameraDevice used NORM_MINMAX → applyColorMap(JET).
        # We need the original intensity; recover via grayscale of the colour map.
        disp_gray = cv2.cvtColor(disp_bgr, cv2.COLOR_BGR2GRAY)

        level, colour, highlight = self._analyse(disp_gray)

        if self._display_mode == "disparity":
            # Draw ROI box and level text directly on disparity view
            out = disp_bgr.copy()
            h, w = out.shape[:2]
            rx1, rx2 = int(w * _ROI_X_MIN), int(w * _ROI_X_MAX)
            ry1, ry2 = int(h * _ROI_Y_MIN), int(h * _ROI_Y_MAX)
            cv2.rectangle(out, (rx1, ry1), (rx2, ry2), colour, 2)
            cv2.putText(
                out, level, (rx1 + 4, ry1 + 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, colour, 2, cv2.LINE_AA,
            )
            return out

        rgb_out = self._overlay(rgb, level, colour, highlight)

        if self._display_mode == "composite":
            disp_resized = cv2.resize(disp_bgr, (rgb_out.shape[1], rgb_out.shape[0]))
            return np.concatenate([rgb_out, disp_resized], axis=1)

        return rgb_out  # "rgb_warn"
