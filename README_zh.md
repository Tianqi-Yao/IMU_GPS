# IMU_GPS v2

面向树莓派的机器人感知与自动导航系统:IMU(姿态)和 RTK GPS(高精度定位)把数据实时广播给浏览器仪表盘;自动导航模块驱动机器人底盘(Feather M4 单片机 + CAN 总线到 Amiga 电控);OAK-D 深度相机模块通过一套可热插拔的图像处理插件推流视频;二维码启动页让手机在局域网内一键访问全部模块。

这是根据 `doc/REFACTOR_PROMPT.md` 的架构评审,对原项目做的一次从零重写——对外契约(黑盒边界)保持不变,内部实现全部清理。**本目录完全独立:这里的任何代码都不 import `v2/` 之外的东西。**

## 黑盒优先架构(请先读这节)

系统由 **7 个独立的 bridge 进程**组成。只要对外契约(WebSocket/HTTP/串口)不变,每个模块都可以单独启动、单独调试、整体替换:

- 模块之间不允许互相 import 业务代码——唯一共享的是 `common/`(零业务逻辑的框架代码:WS 广播、静态 HTTP 服务、日志、端口派生)。
- 每个 bridge 内部都分 INPUT(串口/WS/文件读取)/ CORE(解析、算法、状态机——不持有 socket/串口对象,可脱离硬件单独测试)/ OUTPUT(WS 广播、HTTP 响应、文件写入)三层。
- 每个模块都配一份 `listen_*.py`(观察/记录真实流量)和 `replay_*.py`(用录制数据模拟该模块的数据源),下游开发不必强依赖上游硬件。

## 模块与端口一览

```
模块        HTTP 端口   WS 端口(HTTP+1)   硬件输入
00_QR       8700        (无 WS,纯静态页面)
01_IMU      8765        8766                BNO085(串口)
02_RTK      8775        8776                RTK GPS(串口,NMEA)
03_Nav      8785        8786                (订阅 01/02 的 WS)
04_Robot    8888        8889                Feather M4(串口)
05_AutoNav  8805        8806                (订阅 01/02 的 WS,驱动 04 的 WS)
06_Camera   8815        8816                OAK-D(USB)
```

约定:每个 bridge 的 HTTP 端口服务其 `web_static/` 仪表盘;WebSocket 端口永远是 `HTTP 端口 + 1`(见 `common/ports.py::derive_ws_port`)。

## `common/` —— 共享框架层

```
common/
  ws_server.py      BroadcastWsServer:客户端集合管理、broadcast()、
                     通过 on_client_message 回调分发入站消息
  http_server.py     StaticFileServer:多线程静态文件服务,带两个窄扩展点
                     (改写 index.html、额外路由)
  logging_setup.py   setup_logger(name, logfile):stdout + .log 文件双输出
  ports.py           derive_ws_port(http_port) -> http_port + 1
```

`common/` 绝不 import `00_QR`~`06_Camera` 里的任何东西,依赖方向永远是单向的。坐标转换、PID 增益、NMEA 解析等任何模块特定逻辑都不允许出现在这里——如果你觉得某段代码"应该放进 common",先确认它是不是业务逻辑,是的话应该留在对应模块内。

## 各模块文档

每个模块的 `README.md` 都用真实(而非过时想象中)的 JSON 样例文档化其 WebSocket/串口契约、控制消息、以及 record/replay 工作流:[00_QR](00_QR/)(纯静态页面,不需要单独 README,直接看 `qr_server.py`)、[01_IMU](01_IMU/README.md)、[02_RTK](02_RTK/README.md)、[03_Nav](03_Nav/README.md)、[04_Robot](04_Robot/README.md)、[05_AutoNav](05_AutoNav/README.md)、[06_Camera](06_Camera/README.md)。

## 运行

```bash
# 启动全部 7 个 bridge,每个在独立 tmux 窗口
./start_bridges.sh

# 通过菜单启动 listen_*/replay_*/send_demo 等临时调试工具
./start_helpers.sh

# 把所有正在运行的 bridge 的 WS 输出(+ 两路摄像头 MJPEG 视频)
# 录制到一个带时间戳的统一 session 目录
python3 record_all.py
```

## 配置

所有可调参数都在仓库根目录的 `config.py` 里,按模块编号分组。改完文件后重启对应 bridge 即可生效——默认不新增命令行参数入口。每个消费者模块都显式定义自己的上游 WebSocket 地址变量(例如 `ROBOT_IMU_WS`/`ROBOT_RTK_WS`),而不是借用别的模块的,这样改动一个模块的上游地址不会误伤另一个模块。

## record/replay 工作流

1. 接上真实硬件运行对应 bridge,同时跑它的 `listen_*.py`,录制一份 `data_log/*.jsonl` 基准样本。
2. 如果这份样本对离线开发普遍有用,可以把它提交进仓库。
3. 下游任何人都可以运行 `replay_*.py`(默认读取 `data_log/` 下最新的一个文件),在没有硬件的情况下拿到同样的 WebSocket 契约。

## 固件

`CIRCUITPY/`(Feather M4,通过 CAN 总线驱动 Amiga 电控——串口协议见 `04_Robot/README.md`)和 `01_IMU/bno085_esp32c3/bno085_esp32c3.ino`(BNO085 IMU 固件——协议见 `01_IMU/README.md`)均原样保留、未做改动;两份协议此前都已验证正确,不在本次重写范围内。
