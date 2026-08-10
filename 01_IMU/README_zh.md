# 01_IMU — BNO085 姿态 Bridge

*[English](README.md)*

通过 USB 串口从 BNO085 IMU(经由跑 `bno085_esp32c3/bno085_esp32c3.ino` 的
ESP32C3 转发)读取姿态数据,并以 JSON 形式通过 WebSocket 广播给任何下游消费者
(浏览器仪表盘、03_Nav、05_AutoNav)。

## 架构

- **INPUT** —— `SerialReader._read_loop`:在独立的守护线程中从串口逐行读取
  换行分隔的紧凑 JSON,设备断开时每 3 秒重试一次连接。
- **CORE** —— `IMUPipeline.process`:`_parse` -> `_enrich_euler` ->
  `_enrich_heading` -> `_enrich_hz`。全部是作用于普通 dict/dataclass 的纯函数,
  不持有 socket 或串口状态——可以只用录制的字符串单独做单元测试。
- **OUTPUT** —— `common.ws_server.BroadcastWsServer` 广播 `imu_frame` 负载;
  `common.http_server.StaticFileServer` 服务 `web_static/`。

## 线协议:固件 -> bridge(串口)

ESP32C3 固件发出**紧凑、按位置编码**的 JSON 行,以适配 256 字节的 USB
CDC-ACM 缓冲区:

```json
{"t":123456,"r":[qi,qj,qk,qr,acc],"g":[qi,qj,qk,qr],"a":[x,y,z],"l":[x,y,z],"v":[x,y,z],"w":[x,y,z],"m":[x,y,z],"s":0,"c":3}
```

| key | 含义 | 展开为 |
|---|---|---|
| `t` | 时间戳(ms) | `ts` |
| `r` | 旋转矢量四元数 + 精度 | `rot` `{qi,qj,qk,qr,acc}` |
| `g` | game rotation vector(不含磁力计) | `game_rot` `{qi,qj,qk,qr}` |
| `a` | 加速度计 | `accel` `{x,y,z}` |
| `l` | 线性加速度 | `lin_accel` `{x,y,z}` |
| `v` | 重力矢量 | `gravity` `{x,y,z}` |
| `w` | 陀螺仪 | `gyro` `{x,y,z}` |
| `m` | 磁力计 | `mag` `{x,y,z}` |
| `s` | 步数计数 | `steps` |
| `c` | 校准状态(0-3) | `cal` |

这是固件与 bridge 之间的一份**隐式契约**,依赖**数组位置而非字段名**——如果任一
侧独立改动数组顺序,数值会静默错位。以 `#` 开头的行是固件调试注释,解析器会
跳过。

## 输出契约(WebSocket,`imu_frame`)

```json
{
  "type": "imu_frame", "version": 1,
  "rot": {"qi": 0.0, "qj": 0.0, "qk": 0.0, "qr": 1.0},
  "euler": {"roll": 0.0, "pitch": 0.0, "yaw": 0.0, "north_offset_deg": 0.0},
  "heading": {"raw": 0.0, "deg": 0.0, "dir": "N"},
  "hz": 50.0,
  "accel": {"x": 0.0, "y": 0.0, "z": 9.8},
  "lin_accel": {"x": 0.0, "y": 0.0, "z": 0.0},
  "gravity": {"x": 0.0, "y": 0.0, "z": 9.8},
  "gyro": {"x": 0.0, "y": 0.0, "z": 0.0},
  "mag": {"x": 0.0, "y": 0.0, "z": 0.0},
  "cal": 3, "steps": 0, "ts": 123456
}
```

- `euler.yaw` 是直接从旋转矢量四元数推出的、与指南针无关的原始 yaw(服务端约定:
  `corrected = raw + north_offset_deg`)。
- `heading.deg`/`heading.dir` 是另一套面向前端展示的罗盘朝向,由 `game_rot`
  计算得出(回退到 `rot`),使用与前端一致的 BNO085 Z-up -> Three.js Y-up
  坐标系修正(`frame_correction = (-sqrt(0.5), 0, 0, sqrt(0.5))`),使前后端
  的朝向数值保持一致。
- **前端符号约定与后端不同**:浏览器展示的是
  `display = raw - north_offset_deg`,而服务端计算的是
  `corrected = raw + north_offset_deg`。在两个方向之间同步偏移量时,用
  `frontend_offset = (360 - server_offset) % 360` 换算。

## 控制消息(浏览器 -> bridge)

```json
{"set_north_offset": 42.5}
```

设置服务端用于后续所有 heading/yaw 计算的 `north_offset_deg`(用于现场罗盘
校准)。不会被广播回去;只影响内部 pipeline 状态。

## Record / replay

- `listen_imu_websocket.py` —— 连接到实时 WS,打印一张表格,并把每一帧
  (附加 `log_recv_ts`/`log_recv_iso`)记录到
  `data_log/imu_raw_{timestamp}.jsonl`。
- `replay_imu_websocket.py` —— 在同一个 WebSocket 端口上以固定 50 Hz 广播
  最近一次录制的 `data_log/*.jsonl` 文件,让下游模块可以在没有实体 IMU 的
  情况下开发。本仓库中 `data_log/` 初始为空——先用 `listen_imu_websocket.py`
  跑一次 bridge(或其他回放源)以产出一份基准样本。

## 运行

```bash
python imu_bridge.py
```

在仓库根目录 `config.py` 中配置 `IMU_SERIAL_PORT` / `IMU_BAUD` /
`IMU_WS_PORT` / `IMU_NORTH_OFFSET`。HTTP 页面在 `IMU_WS_PORT`(默认
8765),WebSocket 在 `IMU_WS_PORT + 1`(默认 8766)。
