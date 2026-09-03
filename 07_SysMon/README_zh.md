# 07_SysMon — CPU 降频/负载/内存监测 Bridge

*[English](README.md)*

按固定周期采样 macOS 的 CPU 降频状态、负载平均值、swap 占用和内存压力,
并以 JSON 形式通过 WebSocket 广播。用于排查室外 Mac mini 偶发的相机 FPS
过低问题——怀疑是环境高温导致的 CPU 降频,但 7 个 bridge 同时跑导致的负载
跑满、以及内存压力,同样会产生一模一样的症状,所以这个模块用来区分到底是
哪一种。

## 架构

- **CORE** —— `SysSampler.sample()`:不维护累积状态,每次都直接 shell 出去
  调用 `pmset -g therm`(这台 Apple Silicon Mac mini 上不需要 sudo 就能拿到
  的 CPU 降频信号)、`os.getloadavg()`/`os.cpu_count()`、
  `sysctl vm.swapusage`,以及 `sysctl kern.memorystatus_vm_pressure_level`
  (和 Activity Monitor 的内存压力表用的是同一个信号)。每个数据源单独解析——
  某个命令输出格式变了,只会让对应字段变成 `null`,不会导致整个采样器崩溃。
- **OUTPUT** ——
  - `common.ws_server.BroadcastWsServer` 每隔 `SYSMON_INTERVAL_S` 秒广播一次
    `sysmon_frame` 负载,供实时面板使用。
  - `JsonlWriter` 把每次采样都追加写入
    `sysmon_data/sysmon_YYYYMMDD.jsonl`(按天分文件),和 `record_all.py`
    是否在跑无关——这样不管 FPS 掉的那一刻有没有人记得手动开始录制,历史
    数据都会自动留存下来。
  - `common.http_server.StaticFileServer` 服务 `web_static/`:一个实时面板
    (状态卡片 + 手写的 canvas 折线图,不依赖任何外部图表库),外加一个
    "导入日志文件"控件,可以把任意保存下来的 `sysmon_data/*.jsonl`
    完全在浏览器本地(FileReader,不经过后端)画成同一套图表——方便回顾
    某次已经发生过的事件。

## 输出契约(WebSocket,`sysmon_frame`,每 `SYSMON_INTERVAL_S` 秒一次)

```json
{
  "type": "sysmon_frame", "version": 1,
  "cpu_speed_limit": 45, "cpu_scheduler_limit": 100, "throttled": true,
  "loadavg_1": 2.05, "loadavg_5": 1.87, "loadavg_15": 2.17, "cpu_count": 8,
  "swap_used_mb": 0.0, "mem_pressure_level": 1,
  "ts": 1788452819.16
}
```

| 字段 | 含义 |
|---|---|
| `cpu_speed_limit` / `cpu_scheduler_limit` | 来自 `pmset -g therm`;满速/可调度核心的百分比,100=无限制,如果开机后从未触发过降频事件则为 `null` |
| `throttled` | 上面两个 limit 任一低于 100 时为 `true` |
| `loadavg_1/5/15` | `os.getloadavg()`——和降频无关的 CPU 负载信号 |
| `cpu_count` | `os.cpu_count()`,用来把负载平均值和可用核心数对比 |
| `swap_used_mb` | 来自 `sysctl vm.swapusage` |
| `mem_pressure_level` | 来自 `sysctl kern.memorystatus_vm_pressure_level`:1=正常,2=警告,4=严重 |
| `ts` | 采样时 bridge 侧的 `time.time()` |

`sysmon_bridge.log` 里,`throttled` 从 False 变 True 的那一刻会打一条边沿
触发的 `WARNING`(恢复正常时打一条 `INFO`),所以就算不开面板,单看日志
也足够确认发生过降频。

## 录制 / 回放

- `sysmon_data/sysmon_YYYYMMDD.jsonl` —— bridge 常驻写入的按天日志(见上文),
  是捕捉无人值守时发生的意外事件的主要方式。
- 同时接入了 `record_all.py` 的 `WS_STREAMS`,所以某次专门发起的录制会话里,
  `data_log/session_*/sysmon.jsonl` 会和该会话的 `camera_status.jsonl`
  (带有 `cam1_fps`/`cam2_fps`)时间戳对齐地放在一起,方便直接对照分析。

## 运行

```bash
python sysmon_bridge.py
```

在仓库根目录 `config.py` 中配置 `SYSMON_WS_PORT` / `SYSMON_INTERVAL_S`。
HTTP 页面在 `SYSMON_WS_PORT`(默认 8825),WebSocket 在
`SYSMON_WS_PORT + 1`(默认 8826)。
