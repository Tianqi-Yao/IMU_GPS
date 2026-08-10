# 快速开始

*[English](QUICKSTART.md)*

## 一、环境准备

```bash
pip install pyserial websockets depthai opencv-python numpy qrcode
```

- Python 3.10+
- tmux:`brew install tmux`(Mac)/ `apt install tmux`(Linux)

> `qrcode` 只有 `00_QR` 需要;`depthai`/`opencv-python` 只有 `06_Camera` 需要,没有 OAK-D 硬件也可以跳过安装,其余模块不受影响。

---

## 二、配置串口

打开根目录 `config.py`,修改三处串口路径(其余参数一般不需要改):

| 参数 | 硬件 | Mac 示例 | Linux 示例 |
|------|------|---------|-----------|
| `IMU_SERIAL_PORT` | BNO085(ESP32-C3)| `/dev/cu.usbmodem1201` | `/dev/ttyACM0` |
| `RTK_SERIAL_PORT` | RTK GPS 接收器 | `/dev/cu.usbmodem1101` | `/dev/ttyACM1` |
| `ROBOT_SERIAL_PORT` | Feather M4 | `/dev/cu.usbmodem1301` | `/dev/ttyACM2` |

查找可用串口:

```bash
ls /dev/cu.*          # Mac
ls /dev/ttyACM*       # Linux
```

---

## 三、启动所有 Bridge

```bash
./start_bridges.sh
```

在单个 tmux session(`bridges`)中以独立窗口启动全部 7 个模块:

```
00_QR      → python3 00_QR/qr_server.py
01_IMU     → python3 01_IMU/imu_bridge.py
02_RTK     → python3 02_RTK/rtk_bridge.py
03_Nav     → python3 03_Nav/nav_bridge.py
04_Robot   → python3 04_Robot/robot_bridge.py
05_AutoNav → python3 05_AutoNav/autonav_bridge.py
06_Camera  → python3 06_Camera/camera_bridge.py
```

**tmux 常用操作:**

| 操作 | 快捷键 |
|------|--------|
| 切换窗口 | `Ctrl-B` 然后按数字/名称对应的窗口号 |
| 后台挂起 | `Ctrl-B D` |
| 重新连接 | `tmux attach -t bridges` |
| 结束全部 | `tmux kill-session -t bridges` |

---

## 四、打开浏览器 UI

| 模块 | 地址 | 说明 |
|------|------|------|
| 00_QR | http://localhost:8700 | 局域网二维码启动页(扫码跳转其他模块) |
| 01_IMU | http://localhost:8765 | 3D 姿态 + 传感器数据 |
| 02_RTK | http://localhost:8775 | 地图 + 路径点管理 |
| 03_Nav | http://localhost:8785 | IMU + RTK 汇总面板(转发,非融合) |
| 04_Robot | http://localhost:8888 | 机器人控制 + 遥测 |
| 05_AutoNav | http://localhost:8805 | 自主导航引擎 |
| 06_Camera | http://localhost:8815 | OAK-D 摄像头流 |

---

## 五、录制数据

### 5.1 一键录制全部(推荐)

```bash
python3 record_all.py
```

同时录制所有正在运行的 bridge 的数据,`Ctrl-C` 停止,输出到统一 session 目录:

```
data_log/session_{时间戳}/
    imu.jsonl            # IMU 帧(~50 Hz)
    rtk.jsonl            # RTK 定位(~5 Hz)
    nav.jsonl            # IMU+RTK 转发帧(~10 Hz)
    robot.jsonl          # 机器人遥测(~20 Hz)
    autonav.jsonl        # 自主导航状态(~5 Hz)
    camera_status.jsonl  # 摄像头状态(~1 Hz)
    cam1.mp4             # 摄像头 1 视频(需摄像头在线)
    cam2.mp4             # 摄像头 2 视频(需摄像头在线)
```

某个 bridge 未运行时该路自动重试、不影响其他录制。停止后打印各文件大小汇总。

### 5.2 单独录制某模块

| 模块 | 命令 | 保存位置 |
|------|------|---------|
| IMU | `python3 01_IMU/listen_imu_websocket.py` | `01_IMU/data_log/imu_raw_{ts}.jsonl` |
| RTK | `python3 02_RTK/listen_rtk_websocket.py` | `02_RTK/data_log/rtk_raw_{ts}.jsonl` |
| Nav | `python3 03_Nav/listen_nav_websocket.py` | `03_Nav/data_log/nav_raw_{ts}.jsonl` |
| Robot | http://localhost:8888 点击 **● REC**,或 `python3 04_Robot/listen_robot_websocket.py` | `04_Robot/data_log/run_{ts}.jsonl` / `robot_raw_{ts}.jsonl` |
| AutoNav | `python3 05_AutoNav/listen_autonav.py` | `05_AutoNav/data_log/autonav_raw_{ts}.jsonl` |
| Camera | `python3 06_Camera/listen_camera_websocket.py` | `06_Camera/data_log/camera_raw_{ts}.jsonl` |

---

## 六、离线回放(无硬件联调)

用已录制的 JSONL 文件模拟数据流,替代真实硬件。每个 `replay_*.py` 默认读取对应模块 `data_log/` 目录下**最新**的一个 `.jsonl` 文件(先用上面的 `listen_*.py` 录一份,或直接放一份录制样本进去):

```bash
# 回放 IMU(占用端口 8766)
python3 01_IMU/replay_imu_websocket.py

# 回放 RTK(占用端口 8776)
python3 02_RTK/replay_rtk_websocket.py

# 回放 Robot(占用端口 8889,04_Robot/data_log/robot_raw_v1.jsonl 已作为基准样本随仓库自带)
python3 04_Robot/replay_robot_websocket.py

# 同时回放 IMU + RTK(为 05_AutoNav 供输入;读取固定的
# 01_IMU/data_log/imu_raw_v1.jsonl、02_RTK/data_log/rtk_raw_v1.jsonl,
# 需要自己先录制/放置这两个文件)
python3 05_AutoNav/replay_imu_rtk.py
```

**用调试工具菜单(`./start_helpers.sh`)一键组合:**

```
D = 02_RTK 回放 + 01_IMU 回放 + 03_Nav bridge + 03_Nav listen
E = 05_AutoNav listen
F = 05_AutoNav replay(同时回放 IMU + RTK)
```

菜单支持输入多个键位组合启动(例如 `234`),会自动在一个 tmux 窗口里平铺多个 pane。

---

## 七、命令速查

```bash
# 启动全部 bridge
./start_bridges.sh

# 一键录制全部数据(另开终端)
python3 record_all.py

# 调试工具菜单(listen / replay / send_demo)
./start_helpers.sh

# 单独启动某模块(示例)
cd 05_AutoNav && python3 autonav_bridge.py

# 停止全部 bridge
tmux kill-session -t bridges
```

**数据文件位置:**

```
data_log/session_{ts}/                       # record_all.py 统一输出目录
01_IMU/data_log/imu_raw_{ts}.jsonl           # 单独录制 IMU
02_RTK/data_log/rtk_raw_{ts}.jsonl           # 单独录制 RTK
03_Nav/data_log/nav_raw_{ts}.jsonl           # 单独录制 Nav
04_Robot/data_log/run_{ts}.jsonl             # 单独录制 Robot(需点 REC)
05_AutoNav/data_log/autonav_raw_{ts}.jsonl   # 单独录制 AutoNav
06_Camera/data_log/camera_raw_{ts}.jsonl     # 单独录制 Camera
```

> 详细的架构说明、`common/` 框架层设计、各模块 JSON 契约见 [README_zh.md](README_zh.md) 及各模块目录下的 `README_zh.md`。
