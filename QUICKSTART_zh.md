# 快速开始

## 一、环境准备

```bash
pip install pyserial websockets depthai opencv-python numpy
```

- Python 3.10+
- tmux：`brew install tmux`（Mac）/ `apt install tmux`（Linux）

---

## 二、配置串口

打开 `config.py`，修改三处串口路径（其余参数一般不需改）：

| 参数 | 硬件 | Mac 示例 | Linux 示例 |
|------|------|---------|-----------|
| `IMU_SERIAL_PORT` | BNO085（ESP32-C3）| `/dev/cu.usbmodem1201` | `/dev/ttyACM0` |
| `RTK_SERIAL_PORT` | RTK GPS 接收器 | `/dev/cu.usbmodem1103` | `/dev/ttyACM1` |
| `ROBOT_SERIAL_PORT` | Feather M4 | `/dev/cu.usbmodem1301` | `/dev/ttyACM2` |

查找可用串口：

```bash
ls /dev/cu.*          # Mac
ls /dev/ttyACM*       # Linux
```

---

## 三、启动所有 Bridge

```bash
./start_bridges.sh
```

在单个 tmux session（`bridges`）中以独立窗口启动 6 个模块：

```
01_IMU    → python 01_IMU/imu_bridge.py
02_RTK    → python 02_RTK/rtk_bridge.py
03_Nav    → python 03_Nav/nav_bridge.py
04_Robot  → python 04_Robot/robot_bridge.py
05_AutoNav → python 05_AutoNav/autonav_bridge.py
06_Camera  → python 06_Camera/camera_bridge.py
```

**tmux 常用操作：**

| 操作 | 快捷键 |
|------|--------|
| 切换窗口 | `Ctrl-B` 然后按数字（`1`～`6`） |
| 后台挂起 | `Ctrl-B D` |
| 重新连接 | `tmux attach -t bridges` |
| 结束全部 | `tmux kill-session -t bridges` |

---

## 四、打开浏览器 UI

| 模块 | 地址 | 说明 |
|------|------|------|
| 01_IMU | http://localhost:8765 | 3D 姿态 + 传感器数据 |
| 02_RTK | http://localhost:8775 | 地图 + 路径点管理 |
| 03_Nav | http://localhost:8785 | 融合导航总面板 |
| 04_Robot | http://localhost:8888 | 机器人控制 + 遥测 |
| 05_AutoNav | http://localhost:8805 | 自主导航引擎 |
| 06_Camera | http://localhost:8815 | OAK-D 摄像头流 |

---

## 五、录制数据

### 5.1 IMU 数据

```bash
python 01_IMU/listen_imu_websocket.py
```

运行期间自动保存 → `01_IMU/data_log/imu_raw_{时间戳}.jsonl`，`Ctrl-C` 停止。

### 5.2 RTK 数据

```bash
python 02_RTK/listen_rtk_websocket.py
```

自动保存 → `02_RTK/data_log/rtk_raw_{时间戳}.jsonl`。

### 5.3 机器人遥测

打开 http://localhost:8888，点击 **● REC** 开始录制，再次点击停止。

- 保存到 `04_Robot/data_log/run_{时间戳}.jsonl`
- 内容：IMU 转发、RTK 转发、里程计（速度/角速度）、电量

### 5.4 自主导航状态

```bash
python 05_AutoNav/listen_autonav.py
```

自动保存 → `05_AutoNav/data_log/autonav_raw_{时间戳}.jsonl`，同时终端实时打印导航状态。
可在终端输入命令控制导航：`start` / `stop` / `pause` / `resume`。

### 5.5 一次性启动所有 listen 工具

```bash
./start_helpers.sh
```

菜单中输入 `137`（或 `1 3 7`），同时启动 IMU + RTK + Robot listen：

```
1  01_IMU  listen   → 01_IMU/data_log/imu_raw_{ts}.jsonl
3  02_RTK  listen   → 02_RTK/data_log/rtk_raw_{ts}.jsonl
7  04_Robot listen  → 终端输出（机器人遥测需另开 REC 按钮）
```

---

## 六、离线回放（无硬件联调）

用已录制的 JSONL 文件模拟数据流，替代真实硬件：

```bash
# 回放 IMU（占用端口 8766）
python 01_IMU/replay_imu_websocket.py

# 回放 RTK（占用端口 8776）
python 02_RTK/replay_rtk_websocket.py

# 同时回放 IMU + RTK（为 05_AutoNav 供输入）
python 05_AutoNav/replay_imu_rtk.py
```

修改回放的数据文件：打开对应 `replay_*.py`，改顶部的 `INPUT_PATH` 变量。

**完整离线联调（helpers 菜单选 `D`）：**

```
D = IMU 回放 + RTK 回放 + Nav bridge + Nav listen
```

---

## 七、命令速查

```bash
# 启动全部 bridge
./start_bridges.sh

# 调试工具菜单（listen / replay）
./start_helpers.sh

# 单独启动某模块（示例）
cd 05_AutoNav && python autonav_bridge.py

# 录制 IMU + RTK（后台运行）
python 01_IMU/listen_imu_websocket.py &
python 02_RTK/listen_rtk_websocket.py &

# 录制自主导航状态
python 05_AutoNav/listen_autonav.py

# 停止全部 bridge
tmux kill-session -t bridges
```

**数据文件位置：**

```
01_IMU/data_log/imu_raw_{ts}.jsonl          # IMU 原始帧（~50 Hz）
02_RTK/data_log/rtk_raw_{ts}.jsonl          # RTK 定位数据（~5 Hz）
04_Robot/data_log/run_{ts}.jsonl            # 机器人遥测（需点 REC）
05_AutoNav/data_log/autonav_raw_{ts}.jsonl  # 自主导航状态（~5 Hz）
```

> 详细参数说明、架构图、插件开发指南见 [README_zh.md](README_zh.md)。
