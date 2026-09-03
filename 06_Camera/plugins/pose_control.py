"""
PoseControlProcessor — gesture-driven robot control via mediapipe Pose.

  - right wrist above the nose -> drive forward
  - left wrist above the nose  -> drive backward
  - neither raised             -> hold (linear = 0)
  - person off-center          -> angular correction to re-center them in
                                   frame; applied every frame regardless of
                                   the forward/backward gesture above (the
                                   robot can turn to track the person while
                                   also moving forward/backward)
  - no person detected         -> stop (linear = angular = 0); does not
                                   search/rotate on its own

This plugin becomes active the moment the browser selects it via
`switch_plugin` (a fresh instance is constructed then, per the existing
plugin framework in plugins/__init__.py) -- there is no separate start/stop
control message.

Unlike every other plugin here, this one has a side effect beyond the frame
it returns: every call to process() also sends a
`{"type": "joystick", "linear": ..., "angular": ...}` message straight to
04_Robot over its own WebSocket connection (ws://localhost:<ROBOT_WS_PORT+1>),
the exact schema 05_AutoNav's RobotWsClient uses. robot_bridge.py has no
arbitration between concurrent joystick senders -- whichever message arrives
last wins (see 04_Robot/robot_bridge.py's VelocityRamp) -- so do not run this
plugin at the same time as 05_AutoNav's autonomous driving or the 04_Robot
manual joystick page; both would fight this one for control.

If multiple cameras are configured, camera_bridge.py hands all of them
frames from the *same* processor instance (switch_plugin sets one processor
object on every FrameStreamServer), each running its own capture thread --
so process() can be entered concurrently from several threads. Only the
camera that was selected in the browser at the moment Apply was clicked
(passed through as the `cam_id` constructor kwarg -- see switch_plugin's
`cam_id` param in camera_bridge.py) runs pose detection and drives the
robot ("driver" camera); every other camera's frames are passed through
unchanged. This avoids two cameras independently computing and sending
conflicting drive commands, and -- unlike picking whichever capture thread
happens to call process() first -- it's deterministic: it's always the
camera you were looking at when you applied the plugin.

Requires `pip install mediapipe` (not a project-wide dependency; only this
plugin needs it -- see 06_Camera/README.md). The pose landmarker model
bundle (a few MB) is downloaded once on first use and cached under
plugins/models/ (gitignored, a binary asset rather than source); expect a
one-time pause the first time this plugin is selected.
"""

from __future__ import annotations

import json
import logging
import threading
import time
import urllib.request
from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import mediapipe as mp
from mediapipe.tasks.python import vision as mp_vision
from mediapipe.tasks.python.core.base_options import BaseOptions
import websockets.sync.client as ws_sync_client

from . import FrameProcessor, register_processor

try:
    import config as _cfg
    from common.ports import derive_ws_port
    _ROBOT_WS_URL = f"ws://localhost:{derive_ws_port(_cfg.ROBOT_WS_PORT)}"
except ImportError:
    _ROBOT_WS_URL = "ws://localhost:8889"

logger = logging.getLogger(__name__)

_MODEL_URL = (
    "https://storage.googleapis.com/mediapipe-models/pose_landmarker/"
    "pose_landmarker_lite/float16/latest/pose_landmarker_lite.task"
)
_MODEL_PATH = Path(__file__).parent / "models" / "pose_landmarker_lite.task"

_ROBOT_CONNECT_TIMEOUT_S = 1.0

_PL = mp_vision.PoseLandmark


def _ensure_model() -> str:
    """Downloads mediapipe's pose landmarker model bundle on first use."""
    if not _MODEL_PATH.exists():
        _MODEL_PATH.parent.mkdir(parents=True, exist_ok=True)
        logger.info("pose_control: downloading pose landmarker model to %s ...", _MODEL_PATH)
        urllib.request.urlretrieve(_MODEL_URL, _MODEL_PATH)
        logger.info("pose_control: model download complete")
    return str(_MODEL_PATH)


@register_processor
class PoseControlProcessor(FrameProcessor):
    """Gesture control: raise a hand to move, drift off-center to be tracked."""

    PROCESSOR_NAME = "pose_control"
    PROCESSOR_LABEL = "Pose Gesture Control"
    PROCESSOR_DESCRIPTION = (
        "Right hand over head = forward, left hand over head = backward, "
        "auto-recenters the person left/right. Drives 04_Robot directly -- "
        "do not run alongside 05_AutoNav or manual joystick control."
    )

    @classmethod
    def required_streams(cls) -> list[str]:
        return ["rgb"]

    @classmethod
    def config_schema(cls) -> list[dict]:
        return [
            {"key": "move_speed", "type": "range", "default": 0.3, "label": "Move Speed (m/s)",
             "min": 0.05, "max": 1.0, "step": 0.05},
            {"key": "turn_gain", "type": "range", "default": 1.5, "label": "Turn Gain",
             "min": 0.1, "max": 5.0, "step": 0.1},
            {"key": "center_dead_zone_frac", "type": "range", "default": 0.15, "label": "Center Dead Zone (frac of width)",
             "min": 0.05, "max": 0.4, "step": 0.01},
            {"key": "raise_margin_frac", "type": "range", "default": 0.05, "label": "Raise Margin (frac of height)",
             "min": 0.0, "max": 0.2, "step": 0.01},
            {"key": "min_detection_confidence", "type": "range", "default": 0.5, "label": "Min Detection Confidence",
             "min": 0.1, "max": 0.9, "step": 0.05},
            {"key": "swap_hands", "type": "bool", "default": False, "label": "Swap Left/Right (mirrored feed)"},
        ]

    def __init__(self, **kwargs) -> None:
        self._move_speed = float(kwargs.get("move_speed", 0.3))
        self._turn_gain = float(kwargs.get("turn_gain", 1.5))
        self._center_dead_zone_frac = float(kwargs.get("center_dead_zone_frac", 0.15))
        self._raise_margin_frac = float(kwargs.get("raise_margin_frac", 0.05))
        self._swap_hands = bool(kwargs.get("swap_hands", False))

        min_detection_confidence = float(kwargs.get("min_detection_confidence", 0.5))
        options = mp_vision.PoseLandmarkerOptions(
            base_options=BaseOptions(model_asset_path=_ensure_model()),
            running_mode=mp_vision.RunningMode.VIDEO,
            num_poses=1,
            min_pose_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_detection_confidence,
        )
        self._landmarker = mp_vision.PoseLandmarker.create_from_options(options)
        self._next_ts_ms = 0

        self._driver_cam_id: Optional[int] = kwargs.get("cam_id")

        self._robot_ws = None
        self._warned_disconnected = False

    def reconfigure(self, **kwargs) -> None:
        # min_detection_confidence needs a fresh PoseLandmarker to take
        # effect, so it's intentionally left out here -- re-select the
        # plugin (switch_plugin) to apply a new value.
        if "move_speed" in kwargs:
            self._move_speed = float(kwargs["move_speed"])
        if "turn_gain" in kwargs:
            self._turn_gain = float(kwargs["turn_gain"])
        if "center_dead_zone_frac" in kwargs:
            self._center_dead_zone_frac = float(kwargs["center_dead_zone_frac"])
        if "raise_margin_frac" in kwargs:
            self._raise_margin_frac = float(kwargs["raise_margin_frac"])
        if "swap_hands" in kwargs:
            self._swap_hands = bool(kwargs["swap_hands"])

    def __del__(self) -> None:
        try:
            self._landmarker.close()
        except Exception:
            pass
        if self._robot_ws is not None:
            try:
                self._robot_ws.close()
            except Exception:
                pass

    # ── CORE ──────────────────────────────────────────────────────────────

    def process(self, frames: dict[str, np.ndarray | None]) -> np.ndarray | None:
        rgb = frames.get("rgb")
        if rgb is None:
            return None

        if frames.get("_cam_id") != self._driver_cam_id:
            # Non-driver camera: pass through only, so it doesn't also
            # detect+drive and fight the driver camera over the robot's
            # controls.
            return rgb

        h, w = rgb.shape[:2]
        ts_ms = max(self._next_ts_ms + 1, int(time.monotonic() * 1000))
        self._next_ts_ms = ts_ms
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
        result = self._landmarker.detect_for_video(mp_image, ts_ms)

        output = rgb.copy()
        linear = 0.0
        angular = 0.0
        state = "LOST"

        if result.pose_landmarks:
            lm = result.pose_landmarks[0]
            nose_y = lm[_PL.NOSE].y
            r_wrist_y = lm[_PL.RIGHT_WRIST].y
            l_wrist_y = lm[_PL.LEFT_WRIST].y
            if self._swap_hands:
                r_wrist_y, l_wrist_y = l_wrist_y, r_wrist_y

            right_up = r_wrist_y < nose_y - self._raise_margin_frac
            left_up = l_wrist_y < nose_y - self._raise_margin_frac
            if right_up and not left_up:
                linear = self._move_speed
                state = "FORWARD"
            elif left_up and not right_up:
                linear = -self._move_speed
                state = "BACKWARD"
            else:
                state = "HOLD"

            cx_frac = (lm[_PL.LEFT_SHOULDER].x + lm[_PL.RIGHT_SHOULDER].x) / 2.0
            center_err_frac = cx_frac - 0.5  # >0: person is right of frame center
            if abs(center_err_frac) > self._center_dead_zone_frac:
                # Person right of center -> turn the chassis right (clockwise)
                # to follow them back toward center. On this hardware a
                # NEGATIVE angular value turns right (same calibrated
                # convention as 05_AutoNav's ANGULAR_SIGN=-1 -- see
                # autonav_algo.py -- since both go through the same
                # robot_bridge joystick wire schema).
                angular = -self._turn_gain * center_err_frac
                if state == "HOLD":
                    state = "CENTERING"

            self._draw_overlay(output, lm, w, h)

        self._send_drive(linear, angular)
        self._draw_status(output, state, linear, angular)
        return output

    # ── OUTPUT (robot drive commands) ────────────────────────────────────

    def _send_drive(self, linear: float, angular: float) -> None:
        if self._robot_ws is None:
            try:
                self._robot_ws = ws_sync_client.connect(
                    _ROBOT_WS_URL, open_timeout=_ROBOT_CONNECT_TIMEOUT_S,
                )
                self._warned_disconnected = False
                threading.Thread(
                    target=self._drain_robot_ws, args=(self._robot_ws,), daemon=True,
                ).start()
            except Exception as exc:
                if not self._warned_disconnected:
                    logger.warning("pose_control: cannot reach 04_Robot at %s: %s", _ROBOT_WS_URL, exc)
                    self._warned_disconnected = True
                return
        try:
            self._robot_ws.send(json.dumps({
                "type": "joystick", "linear": round(linear, 3), "angular": round(angular, 3),
            }))
        except Exception as exc:
            if not self._warned_disconnected:
                logger.warning("pose_control: robot WS send failed: %s", exc)
                self._warned_disconnected = True
            try:
                self._robot_ws.close()
            except Exception:
                pass
            self._robot_ws = None

    def _drain_robot_ws(self, ws) -> None:
        # robot_bridge broadcasts odom/imu/rtk to every connected client;
        # this connection only sends, so it must still drain incoming
        # traffic or the default max_queue=16 backlog jams the connection's
        # receive thread and eventually kills the connection.
        try:
            for _ in ws:
                pass
        except Exception:
            pass

    # ── OUTPUT (overlay) ──────────────────────────────────────────────────

    def _draw_overlay(self, img: np.ndarray, lm, w: int, h: int) -> None:
        def pt(landmark_id) -> tuple[int, int]:
            p = lm[landmark_id]
            return int(p.x * w), int(p.y * h)

        nose, l_sh, r_sh = pt(_PL.NOSE), pt(_PL.LEFT_SHOULDER), pt(_PL.RIGHT_SHOULDER)
        l_wr, r_wr = pt(_PL.LEFT_WRIST), pt(_PL.RIGHT_WRIST)
        for p in (nose, l_sh, r_sh, l_wr, r_wr):
            cv2.circle(img, p, 6, (0, 255, 0), -1)
        cv2.line(img, l_sh, l_wr, (0, 200, 255), 2)
        cv2.line(img, r_sh, r_wr, (0, 200, 255), 2)
        cv2.line(img, l_sh, r_sh, (0, 200, 255), 2)

        center_x = w // 2
        dead = int(self._center_dead_zone_frac * w)
        cv2.line(img, (center_x, 0), (center_x, h), (255, 255, 255), 1)
        cv2.rectangle(img, (center_x - dead, 0), (center_x + dead, h), (255, 255, 255), 1)

    def _draw_status(self, img: np.ndarray, state: str, linear: float, angular: float) -> None:
        color = {
            "FORWARD": (0, 255, 0), "BACKWARD": (0, 165, 255),
            "CENTERING": (255, 255, 0), "HOLD": (200, 200, 200), "LOST": (0, 0, 255),
        }.get(state, (255, 255, 255))
        cv2.putText(img, state, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2, cv2.LINE_AA)
        cv2.putText(img, f"linear={linear:+.2f}  angular={angular:+.2f}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
