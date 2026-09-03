# 05_AutoNav — 自主导航

*[English](README.md)*

用实时 IMU 航向 + RTK 位置驱动机器人沿固定航点路径行驶,通过 WebSocket
控制 04_Robot。这是本项目 bridge/算法拆分最清晰的模块:**所有数学计算都在
`autonav_algo.py` 里;`autonav_bridge.py` 只做 I/O 并驱动状态机。**

**在假设旧 README 里的算法描述之前,请先读这段:** 实际的控制律是一个带硬
死区的比例(P-only)航向控制器。它**不是** PID(没有积分/微分项),也**不是**
Pure Pursuit(没有前视点选择——它始终直接瞄准当前航点)。这是一次刻意的、
经过验证的简化;下面 `config.py` 里的调参旋钮就是实际在用的那些。

## 架构

- **INPUT** —— `ImuWsClient`/`RtkWsClient` 订阅 01_IMU/02_RTK;`path.csv`
  航点加载;浏览器 WS 控制消息。
- **CORE** —— `autonav_algo.compute()`(几何 + 控制律,返回一个带
  `linear/angular/arrived/dist_m/bearing_deg` 的 `ComputeResult`,这样
  bridge 就不需要自己重新计算距离/航向)以及 `AutoNavLoop`(idle/running/
  paused/lifting/arrived 状态机——航点推进、在航点暂停、触发升降都是状态机
  的职责,住在这里而不是 `autonav_algo.py`)。
- **OUTPUT** —— `RobotWsClient`(向 04_Robot 发送驱动/升降指令);
  `common.ws_server.BroadcastWsServer`(向仪表盘广播 autonav_status);
  `common.http_server.StaticFileServer`(`web_static/` + `GET /export_csv`)。

## 算法(autonav_algo.py,真实行为)

- `fast_distance_m`/`fast_bearing` —— 等距圆柱投影的平面几何(不是
  haversine 大圆距离),在田块尺度下够用,不适合远距离。是公开 API(没有
  前导下划线)——被 `autonav_bridge.py` 和 `build_waypoints_from_run.py`
  同时使用。
- `compute(lat, lon, heading_deg, waypoints, wp_idx, dt_s, prev_angular=0.0)`
  —— 航向误差 -> 比例增益(`AUTONAV_PID_KP`)-> 硬死区
  (`AUTONAV_DEAD_ZONE_DEG`,低于此值 angular 精确为 0,不是滤波出来的)。
  在高摩擦地面上,死区边缘附近的纯 P 指令太弱、突破不了静摩擦(误差不断
  增长,直到一个大得多的指令"挣脱"出来并过冲——这是田间实测到的直行时
  的 stick-slip 摆动),所以有两个前馈式的项在补偿,同时避免把这个控制器
  变成 PID 或 Pure Pursuit:
    - `AUTONAV_MIN_ANGULAR_KICK` —— 死区/静摩擦地板:死区之外任何非零的
      angular 指令至少是这么大。
    - `AUTONAV_ANGULAR_SLEW_RATE` —— 限制*输出*指令的变化速率(单位
      rad/s²,不是误差的微分),这样上面的 kick 本身不会跳变过冲;
      `prev_angular`(这个函数自己上一次的返回值,由调用方显式传入,因为
      这个函数本身仍然是纯函数)就是它爬升的起点。
  线速度:在离航点 `AUTONAV_DECEL_RADIUS_M` 范围内减速,并且随着航向误差
  从 `AUTONAV_DEAD_ZONE_DEG` 增长到 `AUTONAV_LINEAR_TAPER_DEG`,**连续地**
  (不是硬停)衰减到 `AUTONAV_LINEAR_TURN_FLOOR`——边走边转只需要克服动
  摩擦,而不是原地掉头那种静摩擦,这也是为什么边走边转在田间实测中比旧的
  原地硬转截止效果更好。这个衰减永远不会把速度抬高到超过航点减速逻辑设定
  的值。到达判定是 `dist_m < AUTONAV_REACH_TOL_M`。
- `compute_calibration_offset(cur_lat, cur_lon, mark_lat, mark_lon, heading_raw)`
  —— 现场罗盘校准:站在一个已知点面对一个地标,这个函数返回要推送给
  01_IMU 的北向偏移量。
- `apply_offset_to_waypoints(waypoints, offset_m)` —— 把整条路径横向平移
  (拐角处斜接)以支持 `set_offset`(例如开出一条平行车道)。
- **明确不会重新引入的**(在此前一次刻意的简化中已移除——没有明确的产品
  决策不要加回来):积分/微分项、移动平均滤波、Pure Pursuit 前视点选择、
  haversine 大圆距离。

## 状态机

`idle -> running <-> paused -> lifting -> arrived`

- **`lifting`**:当正在抵达的航点的 `lift` 列为 `up`/`down` 时触发;
  运行升降执行器 `AUTONAV_LIFT_*` 秒,然后继续下一个航点(如果这是最后
  一个航点则进入 `arrived`)。
- **自动暂停**:如果 `running` 时 `sensors_ok`(IMU+RTK 是否足够新鲜,见
  `AUTONAV_GPS_TIMEOUT_S`)不满足,或机器人 WS 链路断开,循环会自动暂停
  (`_paused_by_timeout = True`),两者恢复后自动继续——这与用户主动发起
  的 `pause` 命令不同。
- **已修复的 bug**:`cmd_resume()` 现在会在传感器/机器人链路仍然处于
  down 状态时拒绝恢复(返回 `"sensors_not_ready"`,作为 `resume_result`
  消息回显给发起请求的客户端),而不是无条件切到 `running`、又立刻被下
  一个控制周期的自动暂停检查弹回 `paused`(重构前实现里的同一周期
  paused/running 抖动问题)。
- 航点到达可以要求连续 `AUTONAV_ARRIVE_FRAMES` 个在容差范围内的控制周期
  才算确认(去抖;默认 1 = 不去抖)。机器人是否在中间航点停下等待用户
  确认,由 `pause_mode`(`"all"` 或 `"type"`,后者只在 `type == "pause"`
  的航点暂停)控制。

## `config.py` 调参旋钮(全部是真实的顶层定义——不是隐藏在算法文件里的
兜底默认值)

`AUTONAV_PID_KP`, `AUTONAV_DEAD_ZONE_DEG`, `AUTONAV_MIN_ANGULAR_KICK`,
`AUTONAV_ANGULAR_SLEW_RATE`, `AUTONAV_LINEAR_TAPER_DEG`,
`AUTONAV_LINEAR_TURN_FLOOR`, `AUTONAV_DECEL_RADIUS_M`, `AUTONAV_REACH_TOL_M`,
`AUTONAV_MAX_LINEAR_VEL`, `AUTONAV_MIN_LINEAR_VEL`, `AUTONAV_MAX_ANGULAR_VEL`,
`AUTONAV_MANUAL_SPEED`, `AUTONAV_ARRIVE_FRAMES`。

## 输出契约(WebSocket,`autonav_status`,约 `AUTONAV_CONTROL_HZ` Hz)

```json
{
  "type": "autonav_status", "version": 1,
  "state": "running", "current_wp_idx": 3, "total_wp": 12,
  "heading_deg": 87.5, "target_bearing_deg": 90.1, "bearing_error_deg": 2.6,
  "dist_to_wp_m": 4.32, "linear": 0.35, "angular": -0.08,
  "gps_age_s": 0.12, "gps_packet_age_s": 0.20, "gps_fix_quality": 4,
  "sensor_block_reason": null, "robot_connected": true, "imu_age_s": 0.05,
  "speed_ratio": 1.0, "manual_speed": 0.4,
  "calib": {"mark": null, "offset_applied": null},
  "waypoints_window": [{"idx": 3, "lat": 0.0, "lon": 0.0, "current": true}],
  "waiting_at_wp": false, "waiting_wp_idx": null, "pause_mode": "all",
  "lift_cmd": "", "lift_remaining_s": 0.0, "lift_up_s": 5.0, "lift_down_s": 5.0,
  "offset_m": 0.0,
  "path_original": [{"lat": 0.0, "lon": 0.0}], "path_offset": [{"lat": 0.0, "lon": 0.0}],
  "robot_lat": 0.0, "robot_lon": 0.0,
  "imu_raw": {"...": "完整 imu_frame"}, "rtk_raw": {"...": "完整 rtk_frame"}
}
```

## 控制消息(浏览器 -> bridge)

`start`, `restart`, `stop`, `pause`, `resume`(可能回复 `resume_result`),
`set_speed{ratio}`, `load_csv{content}`, `gen_path{lat,lon}`, `calib_mark`,
`calib_apply`, `manual_drive{linear}`, `confirm_wp`, `set_pause_mode{mode}`,
`lift_control{cmd}`, `set_lift_duration{up_s,down_s}`, `set_offset{offset_m}`。

**注意**:`pause`/`resume`/`resume_result` 在 `autonav_bridge.py` 里已经
完整实现,但内置的 `web_static/app.js` 仪表盘没有为它们提供任何按钮,也
会忽略 `resume_result` 消息——目前进入/退出 `paused` 唯一的途径是上面
"自动暂停"一节说的传感器/机器人链路超时路径。直接通过 WebSocket 发送
`pause`/`resume`(比如从自定义客户端)按文档描述可以正常工作;只是目前
还没有对应的 UI。

## 航点文件(`path.csv`)

只有 `lat`/`lon`/`type`/`lift` 列会被消费(`type` 目前只有 `""` 或
`"pause"`;`lift` 是 `""`/`"up"`/`"down"`)。其他列(`st lat`、`st lon`、
`file`、`row num` 等)是 `scripts/convert_offsets_to_latlon.py` 转换过程
遗留下来的中间字段,加载器会忽略它们。

## Record / replay

- `listen_autonav.py` —— 记录到 `data_log/autonav_raw_{timestamp}.jsonl`。
- `replay_imu_rtk.py` —— 在 01_IMU/02_RTK 各自正常的端口上提供固定的
  IMU+RTK 基准文件,便于在没有硬件的情况下开发本 bridge。使用前把
  `IMU_JSONL`/`RTK_JSONL` 指向一对匹配的录制文件。
- `build_waypoints_from_run.py` —— 把一次录制的手动驾驶
  (`waypoint_runs/recordings/run_*.jsonl`，由 04_Robot 的 GEN CSV 按钮或
  录制开关生成)转换成一份降采样的航点 CSV。

## 运行

```bash
python autonav_bridge.py
```

需要 01_IMU、02_RTK、04_Robot(真实或回放)已经在 `config.py` 里
(`AUTONAV_IMU_WS`/`AUTONAV_RTK_WS`/`AUTONAV_ROBOT_WS`)配置的地址上运行。
HTTP 页面在 `AUTONAV_WS_PORT`(默认 8805),WebSocket 在
`AUTONAV_WS_PORT + 1`(默认 8806)。
