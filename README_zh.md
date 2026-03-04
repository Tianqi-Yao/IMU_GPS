# IMU_GPS — 农业机器人 IMU & GPS 数据采集

BNO085 + ESP32-C3 IMU 数据采集，配套 Web 3D 实时可视化界面。

---

## 目录结构

```
IMU_GPS/
├── 00_IMU/
│   ├── bno085_esp32c3/
│   │   └── bno085_esp32c3.ino   # ESP32-C3 Arduino 固件
│   ├── serial_bridge.py          # Python 串口 → WebSocket 桥接服务
│   ├── requirements.txt          # pyserial>=3.5, websockets>=12.0
│   └── web_static/
│       ├── index.html            # 可视化页面主框架
│       ├── imu_visualizer.js     # Three.js r160 3D 渲染 + WebSocket 客户端
│       └── style.css             # 深色主题样式
├── 02_RTK/
│   └── rtk_reader.py            # RTK GPS 串口读取
└── config.py                    # 共用配置
```

---

## 硬件连接（ESP32-C3）

| 信号  | GPIO |
|-------|------|
| MOSI  | 1    |
| MISO  | 6    |
| SCK   | 7    |
| CS    | 0    |
| INT   | 5    |
| RST   | 2    |
| BOOT  | 9（长按 3 秒保存校准） |

---

## 快速开始

### 1. 烧录固件

用 Arduino IDE 打开 `00_IMU/bno085_esp32c3/bno085_esp32c3.ino`，选择 **ESP32C3 Dev Module**，烧录。

依赖库（Arduino Library Manager 安装）：
- **Adafruit BNO08x**（非 SparkFun 版本）

### 2. 启动桥接服务

```bash
cd 00_IMU
pip install -r requirements.txt
python serial_bridge.py --port /dev/ttyACM0 --baud 921600 --ws-port 8765
```

参数说明：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `--port` | `/dev/ttyACM0` | ESP32-C3 串口设备 |
| `--baud` | `921600` | 串口波特率 |
| `--ws-port` | `8765` | HTTP 静态文件服务端口；WebSocket 自动使用 `ws-port + 1`（8766） |

### 3. 打开可视化界面

浏览器访问：`http://localhost:8765`

手机访问（局域网）：`http://<电脑IP>:8765`

---

## Web 可视化界面功能

### 数据面板

实时显示以下传感器数据：
- **Rotation Vector**（带磁力计四元数）
- **Heading**：航向角 + 罗盘方向（基于 Game Rotation Vector）
- **Euler Angles**：横滚 / 俯仰 / 偏航（由 Python 计算）
- **Accelerometer**、**Linear Accel**、**Gravity**
- **Gyroscope**、**Game Rotation Vector**（无磁力计）、**Magnetometer**
- 计步数、校准等级、数据频率（Hz）

### 工具栏按钮

| 按钮 | 功能 |
|------|------|
| 输入框 + **Set Heading** | 手动输入当前方向角（0–359°），将此方向标记为指定角度 |
| **Clear North** | 清除北向校准偏移，恢复原始 heading |
| **Reset View** | 复位 3D 相机到初始视角 |
| **Lock Yaw** | 切换：固定俯仰/滚转，只保留偏航旋转，方便观察水平朝向 |
| **Pause / Resume** | 切换：冻结 / 恢复实时数据更新 |

---

## 架构说明

```
ESP32-C3 (BNO085 SPI)
    │  串口 JSON 帧 @ 921600 baud
    ▼
serial_bridge.py
    ├── 计算 Euler 角（roll/pitch/yaw）
    ├── HTTP 服务 web_static/ → :8765
    └── WebSocket 广播 → :8766
            │
            ▼
    浏览器 imu_visualizer.js
        ├── Three.js 3D 渲染（SLERP 平滑）
        ├── 罗盘 HUD（Canvas 2D）
        └── 工具栏控件（northOffset / lockYaw / pause）
```

**坐标系说明**：BNO085 使用 Z-up 右手系，Three.js 使用 Y-up 右手系，桥接时乘以旋转修正四元数 `(-√2/2, 0, 0, √2/2)`（绕 X 轴 -90°）。

---

## 校准

1. 上电后传感器会自动加载上次保存的校准数据（DCD）
2. 移动/旋转设备使各轴校准等级达到 **High（3）**（数据面板右下角显示绿色）
3. 长按 **BOOT（GPIO 9）3 秒** 保存当前校准数据到传感器非易失存储

---

## 依赖版本

| 依赖 | 版本 |
|------|------|
| Python | ≥ 3.10 |
| pyserial | ≥ 3.5 |
| websockets | ≥ 12.0 |
| Three.js | r160（CDN） |
| Arduino IDE | 2.x |
| Adafruit BNO08x | 最新版 |

---

## RTK 地图可视化（新增）

RTK 模块目录：

```
02_RTK/
├── rtk_reader.py      # 读取 Emlid RS+ NMEA 数据（GGA/RMC）
├── rtk_bridge.py      # Python RTK -> WebSocket + HTTP 静态页面
└── web_static/
    ├── index.html
    ├── rtk_visualizer.js
    └── style.css
```

启动：

```bash
cd 02_RTK
python rtk_bridge.py --ws-port 8775 --hz 5 --open-browser
```

打开：`http://localhost:8775`

说明：
- 若 RTK 暂无有效 `lat/lon`，页面自动使用默认坐标 `38.9412928598587, -92.31884600793728`。
- 支持导入 CSV（或制表符文本）路径点，至少需要 `lat, lon` 列，`tolerance_m, max_speed` 可选。
- 地图会按移动状态给轨迹段着色：
  - 绿色：进入当前目标容差圈（满足点位）
  - 橙色：接近目标（2x 容差范围内）
  - 红色：偏离目标
- 自动记录行驶日志，可在页面点击“导出日志”下载 CSV。
