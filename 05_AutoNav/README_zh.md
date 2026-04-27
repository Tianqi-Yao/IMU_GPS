# 05_AutoNav — Autonomous Navigation

Pure Pursuit + PID 路径跟踪模块。读取 IMU 朝向和 RTK 位置，沿 `path.csv` 中的航点自主导航，向 `04_Robot` 发送速度指令。

## 文件结构

```
05_AutoNav/
├── autonav_bridge.py   # I/O 框架：传感器读取、状态机、航点选择、WS/HTTP 服务
├── autonav_algo.py     # 控制算法：修改这里来改变转向行为
├── path.csv            # 航点文件
├── listen_autonav.py   # 调试工具：打印实时导航状态
├── replay_imu_rtk.py   # 离线工具：无硬件时回放 IMU/RTK 数据
└── web_static/         # 浏览器 Dashboard（自动打开）
```

## 快速启动

```bash
python autonav_bridge.py
```

启动后浏览器自动打开 `http://localhost:8805`，显示实时导航状态。

在页面或通过 WebSocket 发送控制指令：

| 指令 | 效果 |
|------|------|
| `start` | 开始导航 |
| `stop` | 停止并归零 |
| `pause` | 原地暂停 |
| `resume` | 从暂停恢复 |

## 数据流

```
imu_bridge  :8766 ──→ ImuWsClient  ─┐
                                     ├─ AutoNavLoop → algo.compute() → joystick cmd
rtk_bridge  :8776 ──→ RtkWsClient  ─┘       │                              │
                                             │                              ↓
path.csv ───────────────────────────────────┘              robot_bridge :8889
                                                                           │
                                              AutoNavWsServer :8806 ◄──────┘
                                              (Dashboard + 控制指令)
```

## 修改算法

**只需要编辑 `autonav_algo.py`**，不需要碰 `autonav_bridge.py`。

顶部参数：

```python
KP = 0.8          # 转向比例增益（越大转越猛）
KI = 0.01         # 积分增益（消除持续小偏差）
KD = 0.05         # 微分增益（防止过冲摆动）
LOOKAHEAD_M = 2.0 # Pure Pursuit 前视距离（越大路径越平滑）
REACH_TOL_M = 1.5 # 航点到达判定半径（米）
MAX_LINEAR  = 1.0 # 最大前进速度（m/s）
MAX_ANGULAR = 1.0 # 最大转向角速度（rad/s）
```

核心函数签名（不要改名）：

```python
def compute(heading_deg, target_bearing_deg, dist_to_wp_m, dist_to_final_m, dt_s):
    # heading_deg        : 当前朝向，0=正北，顺时针
    # target_bearing_deg : 前视目标点的方位角
    # dist_to_wp_m       : 到当前航点的距离（m）
    # dist_to_final_m    : 到终点的距离（m）
    # dt_s               : 距上次调用的时间（s）
    # 返回: (linear m/s, angular rad/s)
    ...
```

## 航点文件

`path.csv` 格式（制表符或逗号分隔）：

```
id  lat         lon          tolerance_m  max_speed
0   38.94130    -92.31880    0.5          1
1   38.94133    -92.31875    0.5          1
```

| 字段 | 说明 |
|------|------|
| `id` | 序号（从 0 开始） |
| `lat` / `lon` | WGS-84 十进制度 |
| `tolerance_m` | 到达判定半径（米），可覆盖 `REACH_TOL_M` |
| `max_speed` | 该段最大速度（m/s），可覆盖 `MAX_LINEAR` |

## 调试

**开启原始数据打印**：将 `autonav_algo.py` 第一行常量改为：

```python
ALGO_DEBUG = True
```

重启后终端输出每个 tick 的原始 IMU/RTK dict、提取值和算法输出：

```
[RAW IMU]   {"type": "imu_frame", "heading": {"deg": 45.2}, ...}
[RAW RTK]   {"type": "rtk_frame", "lat": 38.941, "fix_quality": 1, ...}
[EXTRACTED] heading=45.2 lat=38.941 lon=-92.318 gps_age=0.21s imu_age=0.08s
[CMD OUT]   linear=0.800 angular=0.312 error=+23.1° dist=4.2m wp=1/5
```

**无硬件离线测试**：

```bash
# 终端 1：回放录制的 IMU + RTK 数据
python replay_imu_rtk.py

# 终端 2：启动导航
python autonav_bridge.py
```

**监听输出流**：

```bash
python listen_autonav.py
```

## 端口

| 用途 | 端口 |
|------|------|
| HTTP Dashboard | 8805 |
| AutoNav WS（状态广播 + 控制） | 8806 |
| 依赖 IMU WS | 8766 |
| 依赖 RTK WS | 8776 |
| 依赖 Robot WS | 8889 |

## 安全机制

- **传感器超时**：GPS 或 IMU 数据超过 `GPS_TIMEOUT_S`（默认 5s）自动暂停，恢复后自动续航
- **GPS fix 过滤**：`fix_quality == 0`（无信号/默认坐标）不计入 GPS age，防止虚假"有信号"
- **Watchdog 心跳**：每秒向 robot_bridge 发送零速指令，防止机器人因通信中断失控
