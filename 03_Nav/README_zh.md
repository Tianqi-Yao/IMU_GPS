# 03_Nav — IMU + RTK 透传

*[English](README.md)*

**在假设"Nav"意味着融合之前,请先读这段。** 尽管同时订阅了 01_IMU 和
02_RTK,这个 bridge **不**做任何传感器融合、坐标系统一、滤波,也不计算
到达判断/航向。它只是把每个上游最新的一帧原样重新广播,嵌套在同一个
envelope 的 `imu`/`rtk` 字段下。目前所有这些计算都发生在浏览器仪表盘的
JavaScript 客户端侧,而不是这里。

如果你需要真正的融合(比如组合的位置+航向估计、航点距离),这个模块目前
还没有——要么在这里新增(这是最自然的落地位置),要么使用 05_AutoNav,
它会直接从 IMU/RTK 流做自己独立的几何/控制计算。

## 架构

- **INPUT** —— `ImuWsClient` / `RtkWsClient`:订阅 01_IMU / 02_RTK 的
  WebSocket,各自只保留最近收到的一帧(断线后 3 秒重连)。
- **CORE** —— `BroadcastLoop`:按自己固定的定时器(`NAV_HZ`)把两个最新帧
  打包进一个 `nav_frame` envelope,与任一上游的实际到达速率解耦。
- **OUTPUT** —— `common.ws_server.BroadcastWsServer` 广播 `nav_frame`;
  `common.http_server.StaticFileServer` 服务 `web_static/`(Three.js IMU
  视图 + Leaflet 地图,两者都由同一个 WebSocket 驱动)。

## 输出契约(WebSocket,`nav_frame`,约 `NAV_HZ` Hz)

```json
{
  "type": "nav_frame", "version": 1,
  "imu": { "...": "来自 01_IMU 的完整 imu_frame 负载,未做修改" },
  "rtk": { "...": "来自 02_RTK 的完整 rtk_frame 负载,未做修改" }
}
```

没有 `nav` 子对象——没有任何派生的距离/航向/融合字段。消费方必须从
`imu.*` 读 IMU 字段、从 `rtk.*` 读 RTK 字段。

## 控制消息(浏览器 -> bridge)

```json
{"set_north_offset": 42.5}
```

原样转发给 01_IMU(见 `01_IMU/README.md`);本 bridge 自己不解释也不存储
这个值。

## Record / replay

- `listen_nav_websocket.py` —— 把帧记录到 `data_log/nav_raw_{timestamp}.jsonl`。
- `replay_nav_websocket.py` —— 以固定 10 Hz 广播最近一次录制的
  `data_log/*.jsonl` 文件。`data_log/` 初始为空;先针对一个正在运行(或被
  回放)的 bridge 跑一次 `listen_nav_websocket.py` 以产出一份基准样本。

## 运行

```bash
python nav_bridge.py
```

需要 01_IMU 和 02_RTK(真实或回放)已经在仓库根目录 `config.py` 里
`NAV_IMU_WS`/`NAV_RTK_WS` 配置的地址上运行。HTTP 页面在 `NAV_WS_PORT`
(默认 8785),WebSocket 在 `NAV_WS_PORT + 1`(默认 8786)。
