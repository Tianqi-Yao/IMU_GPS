# IMU_GPS

*[English](README.md)*

面向田间机器人的树莓派感知与自主导航系统:BNO085 IMU 和 RTK GPS 接收器
把实时姿态和厘米级定位通过 WebSocket 送给浏览器仪表盘,以及一个自主导航
循环——它驱动机器人底盘(Feather M4 经 CAN 总线到 Amiga 电控)沿航点路径
行驶;一个 OAK-D 深度相机通过一套可热插拔的图像处理插件推流视频(其中
包含一个可以直接驾驶机器人的手势控制插件);一个二维码启动页让手机在
局域网内一键访问每个模块的仪表盘。

## 架构:相互独立的 bridge 进程

系统由 **7 个独立进程**("bridge")组成,每个进程负责一件硬件或一项计算,
彼此只通过 WebSocket/HTTP 通信。只要对外契约(WebSocket 的 JSON schema,
或串口协议)不变,任何一个 bridge 都可以单独启动、单独调试、单独重启、
甚至整体换成另一套实现——这里的任何代码都不会 import 其他模块的业务逻辑。

| 模块 | 入口脚本 | HTTP | WS(HTTP+1) | 通信对象 | 职责 |
|---|---|---|---|---|---|
| `00_QR` | `qr_server.py` | 8700 | —(纯静态页面) | — | 局域网二维码启动页 + WiFi 加入 |
| `01_IMU` | `imu_bridge.py` | 8765 | 8766 | BNO085(串口) | 姿态/朝向 |
| `02_RTK` | `rtk_bridge.py` | 8775 | 8776 | RTK GPS(串口,NMEA) | 定位 fix |
| `03_Nav` | `nav_bridge.py` | 8785 | 8786 | 01_IMU、02_RTK(WS) | IMU+RTK 仪表盘转发(不做融合) |
| `04_Robot` | `robot_bridge.py` | 8888 | 8889 | Feather M4(串口) | 驱动控制 + 遥测 |
| `05_AutoNav` | `autonav_bridge.py` | 8805 | 8806 | 01_IMU、02_RTK、04_Robot(WS) | 航点自主导航 |
| `06_Camera` | `camera_bridge.py` | 8815 | 8816 | OAK-D(USB) | 视频推流 + 视觉插件 |

每个 bridge 的 HTTP 端口都服务其 `web_static/` 仪表盘;WebSocket 端口
永远是 `HTTP 端口 + 1`(见 `common/ports.py::derive_ws_port`)。MJPEG
视频本身(06_Camera)走独立的端口(`CAM1_STREAM_PORT`/`CAM2_STREAM_PORT`,
默认 8080/8081),因为它不是一路 JSON/WebSocket 数据流。

## 设计原则

- **黑盒契约。** 模块之间只通过 WebSocket/HTTP/串口边界通信,绝不 import
  对方的代码。把 02_RTK 整个换成另一套 GPS 方案也完全可行,只要它在同一
  个端口上广播同样的 `rtk_frame` JSON,下游不需要改一行代码。
- **每个 bridge 内部分 INPUT / CORE / OUTPUT 三层。** 串口/WS/文件读取
  (INPUT)与解析/算法/状态机(CORE——纯函数/纯对象,不持有 socket 句柄,
  可脱离硬件单独做单元测试)以及 WS 广播/HTTP 响应(OUTPUT)严格分开。
  05_AutoNav 是最典型的例子:所有控制律的数学计算都在 `autonav_algo.py`
  里,`autonav_bridge.py` 只做 I/O 并驱动状态机。
- **`common/` 是零业务逻辑的框架层**,被所有 bridge 共享:
  `ws_server.py`(广播 + 入站消息分发)、`http_server.py`(静态文件服务)、
  `logging_setup.py`、`ports.py`。它绝不 import `00_QR`~`06_Camera` 里的
  任何东西,任何模块特定的逻辑(坐标转换、PID 增益、NMEA 解析等)都不允许
  出现在这里。
- **record/replay 支持无硬件开发。** 每个模块都配了一份
  `listen_*.py`(观察并把真实 WebSocket 流量记录到 `data_log/*.jsonl`)
  和一份 `replay_*.py`(在同一个端口上以固定速率重放一份录制文件),这样
  下游开发就不必强依赖上游硬件已经接好。

## 仓库结构

```
00_QR/       局域网二维码启动页(无 README,直接看 qr_server.py)
01_IMU/      BNO085 姿态 bridge
02_RTK/      RTK GPS bridge
03_Nav/      IMU+RTK 仪表盘转发
04_Robot/    机器人串口控制 bridge(Feather M4 / Amiga CAN)
05_AutoNav/  自主航点导航
06_Camera/   OAK-D 推流 + 图像处理插件
common/      共享框架:ws_server、http_server、日志、端口
CIRCUITPY/   Feather M4 固件(CircuitPython,经 CAN 驱动 Amiga 电控)
config.py    所有模块的可调参数,按模块编号分组
start_bridges.sh   启动全部 7 个 bridge,每个在独立 tmux 窗口
start_helpers.sh   通过菜单启动 listen_*/replay_*/send_demo 调试工具
record_all.py      把所有正在运行的 bridge 的 WS 输出录制到统一 session
doc/         设计/重构笔记与项目报告
```

## 快速上手

```bash
# 启动全部 7 个 bridge,每个在独立 tmux 窗口
./start_bridges.sh

# 通过菜单启动 listen_*/replay_*/send_demo 调试工具
./start_helpers.sh

# 把所有正在运行的 bridge 的 WS 输出(+ 两路摄像头 MJPEG 视频)
# 录制到一个带时间戳的统一 session 目录
python3 record_all.py
```

串口配置、各仪表盘地址、以及完整的 record/replay 流程见
[QUICKSTART_zh.md](QUICKSTART_zh.md)。

## 配置

所有可调参数(串口路径、波特率、WebSocket 地址、PID/控制增益等)都在
仓库根目录的 `config.py` 里,按模块编号分组。改完文件后重启对应 bridge
即可生效——默认不新增命令行参数入口。每个消费者模块都显式定义自己的
上游 WebSocket 地址变量(例如 `ROBOT_IMU_WS`/`ROBOT_RTK_WS`),而不是
借用别的模块的,这样改动一个模块的上游地址不会误伤另一个模块。

## 各模块文档

每个模块的 `README.md`(配有中文版 `README_zh.md`)都用真实的 JSON
样例文档化其 WebSocket/串口契约、控制消息、以及 record/replay 工作流:

- [01_IMU](01_IMU/README_zh.md) —— ESP32C3 固件的线协议、`imu_frame` 契约、北向偏移校准
- [02_RTK](02_RTK/README_zh.md) —— NMEA 解析、`rtk_frame` 契约、fix quality 编码
- [03_Nav](03_Nav/README_zh.md) —— 为什么这里是转发而不是传感器融合
- [04_Robot](04_Robot/README_zh.md) —— Feather M4 串口协议、摇杆/遥测契约
- [05_AutoNav](05_AutoNav/README_zh.md) —— 控制律、状态机、航点文件格式
- [06_Camera](06_Camera/README_zh.md) —— 插件接口、`pose_control` 手势驾驶插件

## 固件

`CIRCUITPY/`(Feather M4,经 CAN 总线驱动 Amiga 电控——串口协议见
`04_Robot/README.md`)和 `01_IMU/bno085_esp32c3/bno085_esp32c3.ino`
(BNO085 IMU 固件——线协议见 `01_IMU/README.md`)实现了本系统依赖的两份
串口协议;改动任何一份都必须与对应 bridge 的解析器保持同步。
