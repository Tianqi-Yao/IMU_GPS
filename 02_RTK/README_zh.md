# 02_RTK — RTK GPS Bridge

*[English](README.md)*

通过串口从 RTK GPS 接收器读取 NMEA 0183 语句(GGA/RMC),并以 JSON 形式通过
WebSocket 广播单一来源的定位结果。

## 架构

- **INPUT** —— `SerialReader._run`:在守护线程中从串口读取原始 NMEA 行,
  出错时每 3 秒重试一次连接。
- **CORE** —— `NMEAPipeline`:校验每条语句的 XOR 校验和,解析 GGA(位置/
  fix/卫星数/hdop/海拔)和 RMC(速度/航向,仅当 status 为 `A`=有效时),
  写入一个共享的、带锁保护的 `RTKFrame`,并提供 `snapshot()` 做无锁读取
  最新状态。**广播速率与 NMEA 到达速率解耦**:`BroadcastLoop` 按自己固定
  Hz 的定时器轮询 `snapshot()`,而不是每收到一条语句就推一条消息。
- **OUTPUT** —— `common.ws_server.BroadcastWsServer` 广播 `rtk_frame` 负载;
  `common.http_server.StaticFileServer` 服务 `web_static/`(基于 Leaflet
  的地图 UI,离线瓦片回退在 `web_static/assets/tiles/` 下)。

## 输出契约(WebSocket,`rtk_frame`,约 `RTK_HZ` Hz)

```json
{
  "type": "rtk_frame", "version": 1,
  "lat": 38.9412928598587, "lon": -92.31884600793728, "alt": 235.2,
  "fix_quality": 4, "num_sats": 14, "hdop": 1.2,
  "speed_knots": 0.04, "track_deg": 287.92,
  "rtk_ts": 1775526995.6776588, "server_ts": 1775526996.288904,
  "source": "rtk"
}
```

| 字段 | 含义 |
|---|---|
| `lat`/`lon` | WGS-84 十进制度(+N/-S, +E/-W) |
| `alt` | 海拔,单位米(MSL) |
| `fix_quality` | 0=无 fix,1=GPS,2=DGPS,4=RTK Fixed(约 1cm),5=RTK Float(约 10cm) |
| `num_sats` | 使用中的卫星数 |
| `hdop` | 水平精度衰减因子 |
| `speed_knots`/`track_deg` | 来自 RMC;RMC status 为 void(无 fix)时为 `null` |
| `rtk_ts` | 最后一次 GGA 更新时 bridge 侧的 `time.time()` |
| `server_ts` | 这一帧被序列化时 bridge 侧的 `time.time()` |
| `source` | `"rtk"` = 真实 fix,`"default"` = 尚无 fix 时的兜底坐标(config.py 中的 `RTK_DEFAULT_LAT`/`LON`) |

**这是一份单一来源契约。** 早期原型探索过双天线航向测量(`rtk_source_frames`、
`heading_deg`、`heading_baseline_m` 等字段);该功能已从后端移除,本次重写也
从 `rtk_visualizer.js` 和 `listen_rtk_websocket.py` 中删除了对应的死 UI/解析
代码。在没有明确产品决策要恢复双天线测量之前,不要复活这些字段。

## Record / replay

- `listen_rtk_websocket.py` —— 把帧记录到 `data_log/rtk_raw_{timestamp}.jsonl`。
- `replay_rtk_websocket.py` —— 以固定 5 Hz 广播最近一次录制的
  `data_log/*.jsonl` 文件。本仓库中 `data_log/` 初始为空:上一版仓库里的
  `rtk_raw_v1.jsonl`/`v2.jsonl` 样本用的是已移除的多天线 schema,本次重写
  故意**没有**带过来——回放前请先针对本 bridge 录一份新样本。

## 其他工具

- `script/sim_RTK_serial.py` —— 通过一对虚拟串口模拟一个缓慢漂移的
  RTK-fixed GPS(见文件内的 `socat` 设置注释),用于无硬件测试。
- `script/download_map.py` —— 下载离线 OpenStreetMap 瓦片到
  `web_static/assets/tiles/`,供 Leaflet 地图的离线回退图层使用。

## 运行

```bash
python rtk_bridge.py
```

在仓库根目录 `config.py` 中配置 `RTK_SERIAL_PORT` / `RTK_BAUD` /
`RTK_WS_PORT` / `RTK_HZ` / `RTK_DEFAULT_LAT` / `RTK_DEFAULT_LON`。HTTP
页面在 `RTK_WS_PORT`(默认 8775),WebSocket 在 `RTK_WS_PORT + 1`(默认
8776)。
