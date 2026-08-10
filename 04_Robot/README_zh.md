# 04_Robot — 机器人串口控制 Bridge

*[English](README.md)*

从浏览器摇杆输入或 05_AutoNav 驱动机器人底盘(Feather M4 + CAN 总线到
Amiga VCU),并把遥测数据(里程计、电量、上游 IMU/RTK)转发给任何
WebSocket 消费者。

## 架构

- **INPUT** —— `SerialLink._reader_loop`(守护线程,按行缓冲)+ 入站的浏览器
  WS 消息(`common.ws_server` 分发给 `RobotBridge._handle_client_message`)。
- **CORE** —— 四个不持有 socket/串口对象、可单独做单元测试的纯对象:
  - `RobotState` —— 解析 `"O:"`/`"S:"` 行为里程计 + auto-active 的真值
    (见下面"状态修复"一节)。
  - `VelocityRamp` —— 限加速度的设定值平滑(`_step_toward`),与 WS 消息
    到达速率解耦;跑在自己的 20 Hz 定时器上。
  - `Watchdog` —— 心跳超时紧急停止(0.5s 检查周期)。
  - `Recorder` —— 限速、按类型瘦身的 `.jsonl` 日志记录。
- **OUTPUT** —— `SerialLink.write_velocity`/`write_raw`/`write_hbridge`
  (串口);`common.ws_server.BroadcastWsServer`(面向浏览器的广播,统一走
  `RobotBridge._broadcast`,这样每条消息也会同时喂给 `Recorder`);
  `common.http_server.StaticFileServer`(服务 `web_static/`,并往
  `index.html` 的 `<html>` 标签注入 `data-max-linear`/`data-max-angular`
  属性)。

### 状态修复:`active` 的单一事实来源

每一条 `"O:"` 行上携带的 CAN 总线状态整数(见
`CIRCUITPY/lib/farm_ng/utils/packet.py::AmigaControlState`,
`STATE_AUTO_ACTIVE = 5`)是 `active` 的**唯一事实来源**,每收到一条
`"O:"` 行(约 20 Hz)就重新同步一次。`"S:ACTIVE"`/`"S:READY"` 行也会
立即生效——这是为了让手动切换后 UI 响应更快——但它们不再是唯一能设置
`active` 的东西。如果硬件在没有发出 `"S:READY"` 的情况下悄悄降级回
READY(比如触发了 CAN 安全联锁),下一条 `"O:"` 行会在约 50ms 内纠正
状态,而不是让 bridge(和仪表盘)一直卡在过期的 `ACTIVE` 状态,直到下次
手动切换才更新。

## 硬件协议

- **上行(Pi -> Feather M4)**:`f"V{linear:.2f},{angular:.2f}\n"`,数值
  归一化到 `[-1.0, 1.0]`(m/s / rad/s)。`f"H{U|D|S}\n"` 控制升降执行器。
  单个 `\r` 字节在 `AUTO_READY` <-> `AUTO_ACTIVE` 之间切换。
- **下行(Feather M4 -> Pi)**:`"O:{v:.3f},{w:.3f},{state:d},{soc:d}\n"`,
  约 20 Hz;切换时会有 `"S:ACTIVE\n"` / `"S:READY\n"` 确认行。

固件(`CIRCUITPY/code.py`)相对重构前保持不变——该协议此前已验证正确。

## 输出契约(WebSocket,一个连接上有多种 `type`)

```json
{"type": "odom", "version": 1, "v": 0.31, "w": -0.02, "state": 5, "soc": 78, "ts": 1785049699.19}
{"type": "state_status", "version": 1, "active": true}
{"type": "imu", "version": 1, "...": "来自 01_IMU 的完整 imu_frame 负载,原样转发"}
{"type": "rtk", "version": 1, "available": true, "...": "来自 02_RTK 的完整 rtk_frame 负载,原样转发"}
{"type": "rec_status", "version": 1, "recording": true, "filename": "run_20260726_120000.jsonl"}
```

`rtk.available` 是在这里计算出来的(`source == "rtk"` 或
`fix_quality > 0`),上游的 `rtk_frame` 本身并不带这个字段。

## 控制消息(浏览器 -> bridge)

```json
{"type": "joystick", "linear": 0.3, "angular": 0.0}
{"type": "heartbeat"}
{"type": "toggle_state"}
{"type": "lift_control", "cmd": "up"}
{"type": "set_recording", "enabled": true}
```

`heartbeat` 和 `joystick` 都会喂给 watchdog。注意:浏览器摇杆 UI 的
`force` 字段(nipplejs 推动幅度)服务端不会读取——它纯粹是客户端展示用
的值。

## Record / replay

- `listen_robot_websocket.py` —— 把完整消息记录到
  `data_log/robot_raw_{timestamp}.jsonl`。
- `replay_robot_websocket.py` —— 以 20 Hz 广播最近一次录制的
  `data_log/*.jsonl` 文件。`data_log/robot_raw_v1.jsonl`(从上一版仓库
  带过来的——其 schema 与 `Recorder._slim` 的 odom/imu/rtk 输出匹配)
  作为基准样本随仓库自带,开箱即可回放。
- `send_robot_only_demo.py` —— 交互式终端摇杆,不需要浏览器。

## 运行

```bash
python robot_bridge.py
```

在仓库根目录 `config.py` 中配置 `ROBOT_SERIAL_PORT` /
`ROBOT_SERIAL_BAUD` / `ROBOT_WS_PORT` / `ROBOT_MAX_LINEAR` /
`ROBOT_MAX_ANGULAR` / `ROBOT_WATCHDOG_TIMEOUT` / `ROBOT_IMU_WS` /
`ROBOT_RTK_WS`(本模块自己定义 `ROBOT_IMU_WS`/`ROBOT_RTK_WS`,而不是借用
03_Nav 的)。HTTP 页面在 `ROBOT_WS_PORT`(默认 8888),WebSocket 在
`ROBOT_WS_PORT + 1`(默认 8889)。
