# 06_Camera — OAK-D Camera Streaming + Processing Plugins

Streams live video from one or two OAK-D cameras as MJPEG over HTTP, with a
hot-swappable image-processing plugin pipeline and a WebSocket status feed
for the dashboard.

## Architecture

- **`CameraDevice`** — owns one OAK-D's depthai pipeline (hardware I/O
  only: open/close, pull frames from output queues, colorize depth/disparity).
- **`MJPEGServer`** — one camera's capture -> process -> encode pipeline,
  now three separate methods (`_capture`/`_process`/`_encode`) called in
  sequence from `_capture_loop`, rather than one fused loop body. Streams
  `multipart/x-mixed-replace` MJPEG on its own HTTP port.
- **`CameraPipeline`** — orchestration: manages N `CameraDevice`+`MJPEGServer`
  pairs, plugin switching (never restarts the camera — `set_processor` is
  an atomic swap), and snapshots.
- **`common.ws_server.BroadcastWsServer`** broadcasts `camera_status`
  (~1Hz); **`common.http_server.StaticFileServer`** serves `web_static/`.

## Plugin interface (`plugins/__init__.py::FrameProcessor`)

```python
class FrameProcessor(abc.ABC):
    def required_streams(self) -> list[str]: ...       # e.g. ["rgb", "depth"]
    def config_schema(self) -> list[dict]: ...           # optional, for UI-driven config
    def reconfigure(self, **kwargs) -> None: ...          # optional, in-place config update
    def process(self, frames: dict) -> np.ndarray | None: ...  # transform, return BGR frame
```

Plugins are **stateless-or-light-state image transforms**; they never
manage camera hardware lifecycle (that's exclusively `CameraDevice`'s job).
New plugin files dropped into `plugins/` are auto-discovered at import time
via `@register_processor` — no other code needs to change (**camera_bridge.py
must be restarted** for a newly added plugin file to be picked up; it is not
hot-reloaded into an already-running process). Five reference plugins ship:
`simple_color` (passthrough), `depth_cam` (RGB/depth/blend), `path_cam`
(yellow-tape/path detection), `disparity_demo` (raw disparity visualization),
`pose_control` (gesture-driven robot control, see below).

### `pose_control` — gesture-driven robot control

Selecting this plugin (i.e. sending `switch_plugin` for it — the moment the
browser's dropdown + "Apply Plugin" picks it) starts gesture control
immediately; there is no separate start/stop message. Uses mediapipe Pose
landmarks on the RGB stream:

- right wrist above the nose → drive forward
- left wrist above the nose → drive backward
- neither raised → hold (linear = 0)
- person off-center horizontally → turns to re-center them, applied every
  frame regardless of the forward/backward gesture (can track the person
  while also moving)
- no person detected → **stops and waits** (linear = angular = 0); does not
  search/rotate on its own

Unlike the other plugins, `process()` has a side effect beyond the frame it
returns: every call also sends `{"type": "joystick", "linear": ..., "angular": ...}`
straight to `04_Robot` over its own WebSocket connection
(`ws://localhost:<ROBOT_WS_PORT + 1>`) — the same schema `05_AutoNav`'s
`RobotWsClient` uses. **`robot_bridge.py` has no arbitration between
concurrent joystick senders (last message wins) — do not run this plugin at
the same time as `05_AutoNav`'s autonomous driving or the `04_Robot` manual
joystick page; they will fight each other for control.**

If both `CAM1_IP` and `CAM2_IP` are configured, both cameras' frames are
handed to the same plugin instance and can call `process()` concurrently
from two threads; only the first camera to call `process()` runs pose
detection and drives the robot, the other is passed through unchanged (to
avoid two cameras independently sending conflicting drive commands).

Config knobs (`config_schema`, all live-tunable via `update_plugin_config`
except `min_detection_confidence` which needs a fresh `switch_plugin` to take
effect): `move_speed` (m/s), `turn_gain`, `center_dead_zone_frac`,
`raise_margin_frac`, `min_detection_confidence`, `swap_hands` (flip if the
feed is mirrored and left/right come out backwards).

Requires `pip install mediapipe` in the environment `camera_bridge.py` runs
under (not a project-wide dependency — only this plugin needs it; if it's
missing, `pose_control` just won't appear in the plugin list, logged as a
`_auto_discover` warning rather than crashing the bridge). The pose landmarker
model bundle (a few MB) downloads once on first use to
`plugins/models/pose_landmarker_lite.task` (gitignored — a binary asset, not
source) — expect a one-time pause in the dashboard the first time this
plugin is selected while it downloads.

## Output contract (WebSocket, `camera_status`, ~1Hz)

```json
{
  "type": "camera_status", "version": 1,
  "cam_selection": 1, "streaming": true,
  "fps": 24.5, "width": 640, "height": 400,
  "mjpeg_url_cam1": "http://{host}:8080/", "mjpeg_url_cam2": "http://{host}:8081/",
  "cam1_clients": 1, "cam2_clients": 0,
  "cam1_streaming": true, "cam2_streaming": false,
  "cam1_fps": 24.5, "cam2_fps": 0.0,
  "active_plugin": "simple_color", "active_plugin_config": {},
  "available_plugins": [
    {"name": "simple_color", "label": "RGB Preview", "description": "...", "config_schema": [], "required_streams": ["rgb"]},
    {"name": "depth_cam", "label": "Depth Camera", "description": "...", "config_schema": [...], "required_streams": ["rgb", "depth"]}
  ],
  "available_streams": ["rgb", "depth", "disparity"]
}
```

**`mjpeg_url_cam1`/`mjpeg_url_cam2` contain the literal placeholder
`{host}`** — this is an intentional, existing part of the contract (not a
bug): the bridge doesn't know which of the server's network interfaces the
requesting browser can actually reach, so it leaves host substitution to
the consumer (`camera_visualizer.js` replaces it with `window.location.hostname`).

## Control messages (browser -> bridge)

`start_stream{cam_id?}`, `stop_stream{cam_id?}`, `switch_camera{cam_id}`,
`switch_plugin{plugin_name,config}` (full config replacement, not merged),
`update_plugin_config{config}` (merged into the current config, calls
`reconfigure()` without recreating the plugin), `restart_cameras{fps,width,height}`
(the only path that actually closes/reopens the camera hardware), `snapshot{cam_id}`
(replies `snapshot_ready{url}` or `snapshot_error{error}`).

## Snapshot pages

`GET /snapshots/snap_{timestamp}.html` — a self-contained HTML page (built
from `web_static/snapshot_template.html`) showing the captured frame plus,
on mouseover, the per-pixel **raw** sensor values from every stream that
fed the active plugin (not just the display-processed output) — useful for
checking a plugin's numeric output against ground truth.

## Record / replay

- `listen_camera_websocket.py` — logs `camera_status` frames to
  `data_log/camera_raw_{timestamp}.jsonl`.
- `replay_camera_websocket.py` — broadcasts the most recently recorded
  `data_log/*.jsonl` file at 1 Hz. (The pre-refactor version hardcoded a
  fixed `camera_raw_v1.jsonl` filename that the listener never actually
  produced — fixed here, same as 04_Robot's replay script.) Note this only
  replays the WebSocket status feed, not the MJPEG video itself.

## Run

```bash
python camera_bridge.py
```

Requires the `depthai` SDK and physical OAK-D hardware to actually stream
(camera open failures are logged, not fatal — the rest of the bridge keeps
running). Configure `CAM1_IP`/`CAM2_IP`/`CAM_WS_PORT`/`CAM1_STREAM_PORT`/
`CAM2_STREAM_PORT`/`CAM_FPS`/`CAM_WIDTH`/`CAM_HEIGHT`/`CAM_ENABLE_STEREO`/
`CAM_ENABLE_DISPARITY` in the repo-root `config.py`. HTTP dashboard on
`CAM_WS_PORT` (default 8815), WebSocket on `CAM_WS_PORT + 1` (default
8816), MJPEG streams on `CAM1_STREAM_PORT`/`CAM2_STREAM_PORT` (default
8080/8081).
