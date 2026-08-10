# 06_Camera — OAK-D 摄像头推流 + 处理插件

*[English](README.md)*

将一个或两个 OAK-D 摄像头的实时视频以 MJPEG 形式通过 HTTP 推流,配有一套
可热插拔的图像处理插件管线,以及供仪表盘使用的 WebSocket 状态推送。

## 架构

- **`CameraDevice`** —— 拥有一台 OAK-D 的 depthai pipeline(只做硬件
  I/O:打开/关闭、从输出队列取帧、给深度/视差上色)。
- **`MJPEGServer`** —— 单个摄像头的 capture -> process -> encode 管线,
  现在拆成三个独立方法(`_capture`/`_process`/`_encode`),按顺序从
  `_capture_loop` 调用,而不是一个揉在一起的循环体。在自己的 HTTP 端口上
  推 `multipart/x-mixed-replace` MJPEG。
- **`CameraPipeline`** —— 编排:管理 N 对 `CameraDevice`+`MJPEGServer`、
  插件切换(从不重启摄像头——`set_processor` 是一次原子替换)、以及截图。
- **`common.ws_server.BroadcastWsServer`** 广播 `camera_status`(约 1Hz);
  **`common.http_server.StaticFileServer`** 服务 `web_static/`。

## 插件接口(`plugins/__init__.py::FrameProcessor`)

```python
class FrameProcessor(abc.ABC):
    def required_streams(self) -> list[str]: ...       # 例如 ["rgb", "depth"]
    def config_schema(self) -> list[dict]: ...           # 可选,供 UI 驱动配置
    def reconfigure(self, **kwargs) -> None: ...          # 可选,原地更新配置
    def process(self, frames: dict) -> np.ndarray | None: ...  # 变换,返回 BGR 帧
```

插件是**无状态或轻状态的图像变换**;它们从不管理摄像头硬件的生命周期
(那是 `CameraDevice` 专属的职责)。放进 `plugins/` 的新插件文件会在导入
时通过 `@register_processor` 自动发现——不需要改动其他代码(**新增插件
文件需要重启 `camera_bridge.py`** 才会被识别;不会热加载进已经在运行的
进程)。随仓库自带五个参考插件:`simple_color`(直通)、`depth_cam`
(RGB/深度/混合)、`path_cam`(黄色胶带/路径检测)、`disparity_demo`
(原始视差可视化)、`pose_control`(手势驱动机器人控制,见下文)。

### `pose_control` —— 手势驱动机器人控制

选中这个插件(即发送对应的 `switch_plugin`——浏览器下拉框 +"Apply
Plugin"点下去的那一刻)会立即开始手势控制;没有独立的开始/停止消息。
在 RGB 流上使用 mediapipe Pose 关键点:

- 右手腕高于鼻子 → 前进
- 左手腕高于鼻子 → 后退
- 都没抬起 → 保持(linear = 0)
- 人物水平偏离画面中心 → 转向以重新居中,每帧都会应用,与前进/后退手势
  无关(可以在移动的同时保持追踪)
- 没检测到人 → **停止并等待**(linear = angular = 0);不会自己搜索/旋转

和其他插件不同,`process()` 除了返回帧之外还有一个副作用:每次调用都会
通过自己的 WebSocket 连接(`ws://localhost:<ROBOT_WS_PORT + 1>`)直接向
`04_Robot` 发送 `{"type": "joystick", "linear": ..., "angular": ...}`——
和 `05_AutoNav` 的 `RobotWsClient` 用的是同一套 schema。**`robot_bridge.py`
在并发的摇杆发送方之间没有仲裁(后发消息生效)——不要在跑这个插件的同时
运行 `05_AutoNav` 的自主驾驶或 `04_Robot` 的手动摇杆页面,它们会互相
抢控制权。**

如果 `CAM1_IP` 和 `CAM2_IP` 都配置了,两台摄像头的帧会交给同一个插件
实例,可能从两个线程并发调用 `process()`;只有第一个调用 `process()` 的
摄像头会跑姿态检测并驱动机器人,另一台原样透传(避免两台摄像头各自
独立发出互相冲突的驾驶指令)。

配置旋钮(`config_schema`,除 `min_detection_confidence`(需要重新
`switch_plugin` 才生效)外,都可以通过 `update_plugin_config` 实时调整):
`move_speed`(m/s)、`turn_gain`、`center_dead_zone_frac`、
`raise_margin_frac`、`min_detection_confidence`、`swap_hands`(如果画面
是镜像的、左右搞反了就用这个翻转)。

需要在运行 `camera_bridge.py` 的环境里 `pip install mediapipe`(不是
项目级依赖——只有这个插件需要;缺失时 `pose_control` 只是不会出现在插件
列表里,记一条 `_auto_discover` warning,而不会让 bridge 崩溃)。姿态
landmarker 模型包(几 MB)会在第一次使用时下载一次到
`plugins/models/pose_landmarker_lite.task`(已加入 gitignore——是二进制
资产,不是源码)——第一次选中这个插件时,仪表盘会有一次性的下载暂停。

## 输出契约(WebSocket,`camera_status`,约 1Hz)

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

**`mjpeg_url_cam1`/`mjpeg_url_cam2` 里包含字面量占位符 `{host}`**——这是
契约里刻意保留的一部分(不是 bug):bridge 并不知道请求方浏览器实际能
访问服务端哪个网络接口,所以把 host 替换留给消费方去做
(`camera_visualizer.js` 会用 `window.location.hostname` 替换它)。

## 控制消息(浏览器 -> bridge)

`start_stream{cam_id?}`, `stop_stream{cam_id?}`, `switch_camera{cam_id}`,
`switch_plugin{plugin_name,config}`(整体替换配置,不是合并),
`update_plugin_config{config}`(合并进当前配置,调用 `reconfigure()` 而不
重建插件实例), `restart_cameras{fps,width,height}`(唯一真正会
关闭/重开摄像头硬件的路径), `snapshot{cam_id}`(回复
`snapshot_ready{url}` 或 `snapshot_error{error}`)。

## 截图页面

`GET /snapshots/snap_{timestamp}.html` —— 一个自包含的 HTML 页面(基于
`web_static/snapshot_template.html` 构建),展示截取的帧,鼠标悬停时显示
喂给当前插件的每个数据流的逐像素**原始**传感器数值(不只是展示处理后的
输出)——便于把插件的数值输出与真值做比对。

## Record / replay

- `listen_camera_websocket.py` —— 把 `camera_status` 帧记录到
  `data_log/camera_raw_{timestamp}.jsonl`。
- `replay_camera_websocket.py` —— 以 1 Hz 广播最近一次录制的
  `data_log/*.jsonl` 文件。(重构前的版本硬编码了一个固定的
  `camera_raw_v1.jsonl` 文件名,而 listener 实际上从来没产出过这个文件——
  这里已经修复,和 04_Robot 的 replay 脚本同样的问题。)注意这只回放
  WebSocket 状态推送,不回放 MJPEG 视频本身。

## 运行

```bash
python camera_bridge.py
```

需要 `depthai` SDK 和实体 OAK-D 硬件才能真正推流(摄像头打开失败只会被
记录日志、不是致命错误——bridge 的其余部分照常运行)。在仓库根目录
`config.py` 中配置 `CAM1_IP`/`CAM2_IP`/`CAM_WS_PORT`/`CAM1_STREAM_PORT`/
`CAM2_STREAM_PORT`/`CAM_FPS`/`CAM_WIDTH`/`CAM_HEIGHT`/`CAM_ENABLE_STEREO`/
`CAM_ENABLE_DISPARITY`。HTTP 仪表盘在 `CAM_WS_PORT`(默认 8815),
WebSocket 在 `CAM_WS_PORT + 1`(默认 8816),MJPEG 流在
`CAM1_STREAM_PORT`/`CAM2_STREAM_PORT`(默认 8080/8081)。
