# v2/ 代码审查报告(第二轮复查)

## 本轮背景

第一轮审查发现的 20 项问题(🔴 bug、🟡 矛盾、🟢 优化建议)已经全部修复并做了针对性验证。用户要求再做一轮独立审查:①确认第一轮的修复本身没有引入新问题;②找出第一轮没有覆盖到的问题。

方法与第一轮一致:3 个独立、互不通气的审查代理分别覆盖 `common/+00_QR+01_IMU+02_RTK`、`03_Nav+04_Robot`、`05_AutoNav+06_Camera`,每组既要复核上一轮改过的具体代码,也要独立通读全部文件找新问题。三份报告汇总后,我对每一条中高严重度发现做了独立复核(`grep`/`Read`/实际运行验证,包括用本机 `depthai==3.5.0`、`node`、构造最小复现脚本),不是照抄审查代理的结论。

**结论先说重点**:第一轮的 20 项修复里,**19 项复核确认修复正确、没有引入新问题**;**1 项(02_RTK 地图首帧居中逻辑)确认是修复本身引入的新回归**。除此之外,本轮独立发现了一批第一轮审查没有覆盖到的问题,其中最重要的一条是 **06_Camera 的 `available_plugins` 字段类型从一开始(不是这两轮修复导致)就传错了,导致插件切换 UI 即使装上正确的字段名也还是不可用**——这说明第一轮修的 `switch_plugin` 字段名问题只是表层症状,根子还没除。另外还发现了一个**影响 5 个模块**的 `replay_*.py` 崩溃 bug(遍历中途被并发修改的 `set`),两个独立审查代理各自发现并交叉确认。

## 严重程度分级

- 🔴 **bug**:会导致功能错误/异常/崩溃,已通过代码 + 运行验证确认。
- 🟡 **矛盾**:前后端契约不一致、或文档与代码不符导致的静默问题。
- 🟢 **优化建议**:不是 bug,但值得改进的代码质量/健壮性问题。

---

## 一、第一轮 20 项修复复核结果

| # | 修复内容 | 复核结论 |
|---|---|---|
| 1 | 06_Camera `cam_selection` 改回 int | ✅ 确认正确,与前端 `=== 1`/`=== 2` 严格比较一致 |
| 2 | 06_Camera `switch_plugin` 字段名改 `plugin_name` | ✅ 字段名本身改对了,但见下文新发现问题 1——插件切换实际仍不可用,根因是 `available_plugins` 类型错误 |
| 3 | 06_Camera depthai `CameraBoardSocket` 改 `getattr` | ✅ 确认正确,全仓库 grep 只有这三处构造调用,无遗漏 |
| 4 | 02_RTK GGA/RMC 解析补 try/except | ✅ 确认正确,try 范围覆盖了全部可能抛异常的语句 |
| 5 | 06_Camera `get_frames()` 补异常保护 | ✅ 基本正确,但见下文新发现问题 8(着色步骤异常时显示帧状态不完全自洽,优化建议级别) |
| 6 | 05_AutoNav `cmd_restart()` 补空航点守卫 | ✅ 确认正确,两个守卫顺序无问题 |
| 7 | 全项目 5 个 replay 脚本 `_find_latest_jsonl` 改用 mtime | ✅ 确认正确,但见下文新发现问题 3——**同一批文件里还有另一个更严重的并发崩溃 bug 没有被第一轮发现** |
| 8 | 06_Camera `CameraPipeline` 补 width/height 状态追踪 | ✅ 确认正确,`cam_configs` 为空时也有合理 fallback |
| 9 | 04_Robot `SerialLink` 补断线重连 | ✅ 确认正确,状态机、加锁范围、重连感知均无问题(仅有一条锁粒度的优化建议,见下文) |
| 10 | `common/ws_server.py` `on_client_connect` 补异常保护 | ✅ 确认正确,`ConnectionClosed` 能被外层正确接住 |
| 11 | 05_AutoNav `cmd_restart()` 补 `_paused_by_timeout` 复位 | ✅ 确认正确 |
| 12 | 05_AutoNav 航点读写加锁 | ✅ 设计成立,`_waypoints_lock` 覆盖了全部真正跨线程访问点;另发现一个和锁无关的 asyncio 协作式调度导致的单帧不一致,极低优先级(见下文新发现问题 12) |
| 13 | 04_Robot odom `state`/`soc` 独立降级解析 | ✅ 确认正确,`len(parts)>N` 判断已排除 IndexError,不存在遗漏 |
| 14 | 01_IMU 删除 `imu_visualizer.js` 死代码 | ✅ JS 层面删除干净,但对应的 CSS 没有同步清理(见下文优化建议) |
| 15 | 01_IMU/02_RTK 内嵌调试脚本改 `location.hostname` | ✅ 改法本身正确,但端口号仍是硬编码字面量,不是真正意义上的"完全动态"(见下文优化建议) |
| 16 | 04_Robot replay schema 重建 | 🟡 **部分问题**:边界情况(字段缺失、已是完整 schema)都处理对了,但① `available` 推导公式与实时公式/README 契约不完全一致,②遇到非 dict 的合法 JSON 行会崩溃——这两点算是这个新函数自身的设计缺口,详见下文新发现问题 4、5 |
| 17 | 02_RTK 地图去硬编码默认坐标,改为首帧居中 | 🔴 **确认是修复引入的新回归**,详见下文新发现问题 2 |
| 18 | 01_IMU/02_RTK listen/replay 脚本改用 config.py | ✅ 确认正确,`sys.path` 插入逻辑和 import 顺序都没问题 |
| 19 | `common/ws_server.py::broadcast()` 改并发发送 | ✅ 确认正确,`message` 不可变、`clients` 快照拷贝后再并发发送,无竞态 |
| 20 | 01_IMU `north_offset_deg` 加锁 | ✅ 确认正确,临界区极小,无死锁/性能问题 |

---

## 二、本轮新发现的问题

### 🔴 bug

#### 1.【最高优先级】06_Camera `available_plugins` 字段类型错误,插件切换 UI 实际仍不可用

- **文件**:`06_Camera/camera_bridge.py:93`(`available_plugins: List[str]`)、`:530`(`available_plugins=[p["name"] for p in plugins.list_plugins()]`)
- **对照前端**:`06_Camera/web_static/camera_visualizer.js:322`(`plugins.map(p => p.name)`)、`:324`(`plugins.some(p => p.name === name)`)、`:357`(`plugins.find(p => p.name === activeName)`)——全部假设 `available_plugins` 是**对象数组**(`{name,label,description,config_schema,required_streams}`),但后端实际发的是**纯字符串数组**。
- **已用 node 实测**:字符串没有 `.name` 属性,`plugins.map(p => p.name)` 全部得到 `undefined`;插件下拉框 4 个选项全部显示成同一个 `"undefined"`,无法区分,`renderPluginConfig` 永远匹配不到当前插件,`depth_cam`/`path_cam` 的配置面板永远不会渲染;点击 Apply Plugin 时发给后端的 `plugin_name` 也是字符串 `"undefined"`,后端必然 `KeyError`。
- **对照旧仓库**:旧代码 `available_plugins=list_plugins()`(直接传完整对象列表),说明这是**我在最初写 v2 时的重写错误**,不是这两轮"审查-修复"引入的,只是两轮都没查到——第一轮修的 `plugin_name` 字段名问题只是表层症状,这个类型错误才是插件切换功能完全不可用的根本原因。
- **触发场景**:打开 Camera Controller 页面即触发,不需要特殊操作。
- **建议修法**:`camera_bridge.py:530` 改成 `available_plugins=plugins.list_plugins()`(直接传完整对象列表),dataclass 字段类型改成 `List[dict]`,同步更新 `06_Camera/README.md` 里的示例 payload。

#### 2. 02_RTK 地图首帧居中逻辑——第一轮"优化建议 #17"的修复本身引入的回归

- **文件**:`02_RTK/web_static/rtk_visualizer.js:665-669`
  ```js
  if (!hasFirstFix) {
      hasFirstFix = true;
      map.setView(point, map.getZoom());
  }
  ```
- **问题描述**:`rtk_bridge.py` 启动时内部状态 `lat`/`lon` 为 `None`,`to_dict()` 里 `source` 恒为 `"default"`(拿 `RTK_DEFAULT_LAT/LON` 兜底坐标),直到收到第一条有效 GGA 定位之前**几乎必然先广播若干帧 `source:"default"`**。第一轮把"只在 `source==='rtk'` 时才居中"改成了"收到任意第一帧就居中",导致地图会在这个**兜底坐标**上就把 `hasFirstFix` 标记为 `true`;等真实 GPS 定位到来时(`source:"rtk"`),不会再触发居中——如果真实坐标和默认坐标相距较远,用户只能看到标记点跑出可视范围,必须手动点 "Center Current"。
- **触发场景**:正常开机流程(GPS 尚未定位时必然先广播若干帧 default),且真实定位坐标与 `config.py` 里的 `RTK_DEFAULT_LAT/LON` 相距较远。
- **建议修法**:恢复"只在拿到真实定位时才首次居中"的判断(`frame.source !== 'default'`),或者保留"任意首帧居中"的同时,额外在检测到 `source` 从 `'default'` 首次切换为 `'rtk'` 时再居中一次。

#### 3. 全项目 5 个 `replay_*.py` 脚本存在"遍历中途被并发修改 set"导致的崩溃 bug(与第一轮修的 `_find_latest_jsonl` 是同一批文件,但这是另一个独立 bug,第一轮没有查到)

- **文件**:`01_IMU/replay_imu_websocket.py:76-92`、`02_RTK/replay_rtk_websocket.py`、`03_Nav/replay_nav_websocket.py:76-91`、`04_Robot/replay_robot_websocket.py:115`、`06_Camera/replay_camera_websocket.py`(5 处结构完全相同,系复制粘贴的同一份模板)
  ```python
  async def handler(ws):
      clients.add(ws)              # 新客户端连接时 mutate 共享 set
      try:
          async for _ in ws: pass
      finally:
          clients.discard(ws)      # 断开时 mutate 共享 set

  async def broadcaster():
      while True:
          for raw in records:
              if clients:
                  for ws in clients:        # 直接遍历共享 set(不是快照)
                      try:
                          await ws.send(raw)  # await 让出控制权
                      except ...: dead.add(ws)
  ```
- **问题描述**:`for ws in clients:` 直接遍历的是共享可变 `set`;循环体内 `await ws.send(raw)` 让出控制权时,如果恰好有新客户端连接或旧客户端断开(浏览器刷新、`listen_*.py` 启动/退出都会触发),会在**正在被迭代的 set 对象上**发生 `add`/`discard`,触发 CPython `RuntimeError: Set changed size during iteration`。这个异常没有被任何 try/except 捕获,会一路冒泡导致整个 replay 服务器进程崩溃退出。**已用最小复现脚本验证该异常必现**。
- **对照**:`common/ws_server.py::BroadcastWsServer.broadcast()` 是正确写法(锁内 `clients = set(self._clients)` 拷贝快照后再遍历),这 5 个 replay 脚本重新发明了广播逻辑,但没有照抄这个关键点。
- **触发场景**:replay 服务器广播过程中,有第二个消费者(浏览器 dashboard、`listen_*.py`)连接或断开——真实调试工作流中很容易触发,直接破坏 CLAUDE.md 要求的"下游开发者应能仅依赖回放数据完成联调"这条基本契约。
- **建议修法**:`for ws in clients:` 改成 `for ws in list(clients):` 或 `for ws in set(clients):`(遍历前拷贝快照),5 处一起改。

#### 4. 04_Robot replay 的 schema 重建函数遇到非 dict 的合法 JSON 行会整体崩溃

- **文件**:`04_Robot/replay_robot_websocket.py::_reconstruct_live_schema`,调用点在 `_load_jsonl` 第 89 行左右
- **问题描述**:`_load_jsonl` 对 `json.JSONDecodeError` 做了捕获并计入 `skipped`,但如果某一行是**合法 JSON 却不是对象**(比如损坏成 `42`、`"foo"`、`[1,2,3]`),`json.loads` 不会报错,`_reconstruct_live_schema(obj)` 内 `obj.get("type")` 对 int/str/list 调用会抛 `AttributeError`,没有被任何 try/except 捕获,导致 `main()` 崩溃退出,而不是像其他畸形行一样被优雅跳过。**已用 `_reconstruct_live_schema(42)` 实测复现该 `AttributeError`**。
- **触发场景**:`data_log/*.jsonl` 文件里混入了非对象类型的合法 JSON 行(人工编辑、磁盘/传输截断)。
- **建议修法**:调用处加 `isinstance(obj, dict)` 判断,非 dict 时计入 `skipped` 并 `continue`。

#### 5. 04_Robot replay 的 `available` 推导公式与实时公式/自身 README 契约不一致

- **文件**:`04_Robot/replay_robot_websocket.py:66`(`obj["available"] = bool((obj.get("fix_quality") or 0) > 0)`)对照 `robot_bridge.py:389`(实时公式是 `source == "rtk" or fix_quality > 0`)和 `04_Robot/README.md:61`(明文写着这个"或"逻辑)
- **问题描述**:`Recorder._slim()` 从未记录 `source` 字段,`_reconstruct_live_schema` 只能退化成单纯 `fix_quality>0` 判断。真实场景里完全可能出现"曾经拿到过真实坐标(`source=="rtk"`,坐标被保留),但当前 `fix_quality` 因信号短暂丢失变成 0"的情况——此时实时广播 `available=True`,回放时却变成 `available=False`,与录制时刻的真实状态不符。
- **触发场景**:用真实硬件录制一段包含"RTK 丢固定解但仍保留最后坐标"的数据,之后回放。
- **建议修法**:二选一——① 在 `Recorder._slim()` 里为 rtk 类型增加 `"source": msg.get("source")` 字段,`_reconstruct_live_schema` 里补上 `source=="rtk"` 判断;② 不想扩展记录 schema 的话,至少在函数注释里写清楚这是"有损近似,无法完全还原 available",避免文档与实际回放行为脱节。

#### 6. 04_Robot 新连接的浏览器客户端收不到当前录制状态

- **文件**:`04_Robot/robot_bridge.py::_handle_client_connect`(473-474 行)对照 `web_static/app.js:29`(`let isRecording = false;` 硬编码初值)
- **问题描述**:新客户端连接时只补发 `state_status`,没有补发 `rec_status`。如果录制已经由另一个客户端开启,此后新打开/刷新的浏览器标签页会一直显示"未在录制",直到有人再次切换录制状态。
- **触发场景**:客户端 A 开启录制后,客户端 B(新连接或刷新页面)加入。
- **建议修法**:在 `_handle_client_connect` 里一并补发当前 `rec_status`(`self._recorder.active`/`self._recorder.filename`)。

#### 7. 04_Robot `Recorder.start()` 不是幂等的,重复调用会泄漏文件句柄

- **文件**:`04_Robot/robot_bridge.py::Recorder.start`(93-101 行)
- **问题描述**:每次调用都无条件创建新文件并覆盖 `self._file` 引用,旧句柄既不会被显式 `close()`,也不会再收到写入,只能等垃圾回收器某个不确定时机隐式关闭。结合上一条("新客户端不知道正在录制"),两个浏览器标签页都误以为未在录制、先后点击"开始录制"就会触发。
- **建议修法**:`start()` 开头检查 `if self.active: return self.filename`(幂等),或先 `self.stop()` 再开新文件。

### 🟡 矛盾

#### 8. 06_Camera README 文档滞后于第一轮的代码修复

- **文件**:`06_Camera/README.md:44`(示例仍是 `"cam_selection": "cam1"` 字符串,应为整数 `1`)、`:65`(`switch_plugin{plugin,config}`,字段名应为 `plugin_name`)
- **问题描述**:第一轮改代码时没有同步更新文档,违反 CLAUDE.md"I/O schema 变更必须更新文档"的规则。
- **建议修法**:随 bug 1 一起把示例 payload 和控制消息说明改成与当前代码一致。

#### 9. 03_Nav 3D 视图被永久锁定为俯视角,但界面提示文案说"可以拖拽旋转"

- **文件**:`03_Nav/web_static/nav_visualizer.js:1020`(启动时无条件 `setTopNorthView(true)`)、`03_Nav/web_static/index.html:28`(提示文案 `"Drag to rotate · Scroll to zoom"`)、`:46`(唯一能关闭俯视锁定的 `#btn-top-north` 按钮被永久藏在 `display:none` 的容器里)
- **问题描述**:`setTopNorthView(true)` 会禁用 `controls.enableRotate`,且没有任何可达的 UI 入口能重新打开。用户看到提示后尝试拖拽会发现完全没有反应,只有滚轮缩放还能用。**这是旧仓库本来就有的问题**,与两轮修复无关。
- **建议修法**:要么把提示文案改成只提示"Scroll to zoom",要么把 `#btn-top-north` 重新暴露成可点击控件。

### 🟢 优化建议

10. **`common/ws_server.py::_handle_client` 消息处理循环里 `ConnectionClosed` 处理不一致**——`on_client_connect` 那段特意把 `ConnectionClosed` 重新抛给外层静默处理,但紧邻的消息循环里同样的异常会被宽泛的 `except Exception` 捕获成一条带完整堆栈的 WARNING 日志。目前对 01_IMU/02_RTK 无实际影响(它们的 handler 从不回发消息),但会给未来任何回发消息的模块产生噪音日志。建议给消息循环也加 `except ConnectionClosed: raise`。
11. **`02_RTK/rtk_bridge.py::_parse_gga` 里 `alt`/`hdop`/`fix_quality`/`num_sats` 无条件覆盖,与 `lat`/`lon` 的"保留旧值"策略不一致**——某些接收机信号抖动时偶发这些字段临时为空,会把已有有效值清空/归零,即便 `lat`/`lon` 仍然有效。建议对齐处理策略,加 `if x is not None` 判断。
12. **01_IMU/02_RTK 内嵌调试脚本的端口仍是硬编码字面量**(`8766`/`8776`),与同页面 `imu_visualizer.js`/`rtk_visualizer.js` 的动态端口推导方式不一致——`config.py` 改端口后调试面板会静默失效而主视图正常。
13. **`02_RTK/web_static/rtk_visualizer.js` 的 `DEFAULT_POS` 仍是硬编码的 `config.py` 默认坐标影子副本**(与第一轮报告里提到的问题相同,第一轮只改了首帧居中逻辑,没有解决这个根源重复维护问题)。
14. **`02_RTK/web_static/index.html` 的 `#langToggle` 按钮完全没有实现**(点击无任何反应,标题写"Switch to Chinese"),以及 `rtk_visualizer.js::STRINGS` 里残留 `cardDual`/`baseline`/`online`/`offline`/`noFix`/`headingInvalid` 等双天线功能遗留的死字符串——均为旧仓库遗留问题。
15. **`01_IMU/web_static/style.css`(约 196-280 行)和 `04_Robot/web_static/style.css`(约 151-408 行)存在大量死 CSS**——前者是第一轮删 JS 死代码时没有同步清理的校准面板样式;后者是与当前 `index.html`/`app.js` 完全无关联的孤立样式(疑似曾经计划接入 05_AutoNav 风格看板但从未清理,约占该文件 2/3)。
16. **`04_Robot/web_static/app.js`(第 6 行)与 `index.html` 内嵌脚本(第 85 行)各自独立计算 WS 端口且算法不完全一致**——`app.js` 没有空端口兜底(会得到 `NaN`),`index.html` 有 `|| 8888` 兜底。当前固定用 8888 访问不会触发,但存在重复实现风险。
17. **`04_Robot/robot_bridge.py::SerialLink.open()` 里 `serial.Serial(...)` 构造调用在锁内执行**,硬件枚举较慢时会让 `_write()` 多等待一小段时间,建议缩小临界区(只在赋值那一刻持锁)。
18. **`05_AutoNav/build_waypoints_from_run.py::_find_latest_run_file`** 存在和第一轮 bug #7 完全相同的"文件名字典序选最新文件"缺陷(`sorted(...)[-1]`),但这个脚本没有被第一轮的 5 处修复覆盖到。目前 `04_Robot/data_log/` 下没有匹配 `run_*.jsonl` 的多个文件所以暂不会选错,建议一并改成按 mtime 排序。
19. **`06_Camera/camera_bridge.py::CameraDevice.get_frames()` 的 try/except 范围把着色逻辑(`cv2.normalize`/`applyColorMap`)也包了进去**——如果着色本身抛异常,`_last_raw_frames` 已经更新但 `_last_frames`(实际显示帧)停留在上一帧,`has_new_frame` 却已置真,导致"有更新但画面没变"的边界不一致。建议把着色逻辑单独包一层 try/except,失败时至少用原始帧兜底。
20. **`06_Camera` 的 `restart_cameras`/设备重开是同步阻塞调用,发生在 asyncio 事件循环线程内**——期间所有 WS 客户端消息处理和状态广播都会卡住。当前 UI 没有暴露触发入口,影响面很小,若以后加入口建议用 `run_in_executor` 挪到线程池。
21. **`05_AutoNav/autonav_bridge.py::_tick()` 存在极低概率的单帧状态不一致**——`await self._robot.send(...)` 期间如果恰好调度到处理航点变更类消息(`load_csv`/`set_offset`)的 task,当次广播的 `autonav_status` 里 `dist_m`/`bearing_deg`(基于旧航点)和 `total_wp`/`path_offset`(基于新航点)会不匹配。这是 asyncio 协作式调度的固有特性,不是加锁能解决的,需要在 `_tick()` 顶部做一次性快照才能根治。
22. **05_AutoNav 网页 UI 没有暴露手动 pause/resume 控制**,`app.js` 完全不处理 `resume_result` 消息(只认 `autonav_status` 类型),与 README 文档描述的"有 pause/resume 命令"存在功能缺口。当前只能靠传感器超时自动触发暂停/恢复。
23. **`06_Camera` 的 `camera_status.width`/`height` 不反映插件实际渲染尺寸**——比如 `path_cam` 的 `composite` 模式会把画面横向拼接成 3 倍宽,但状态字段仍显示采集配置分辨率,前端 `streamRes` 显示与用户实际看到的画面尺寸不符。

---

## 三、更新后的修复优先级建议

1. **立刻修**:新发现问题 1(`available_plugins` 类型错误,插件切换真正的根因)、问题 3(5 个 replay 脚本的 set 并发崩溃)。这两条一个是核心交互功能完全不可用,一个是破坏"离线协作"基本契约的崩溃 bug。
2. **尽快修**:问题 2(RTK 首帧居中回归)、问题 4(replay schema 重建遇非 dict 崩溃)、问题 6/7(录制状态不同步 + 文件句柄泄漏,组合触发)。
3. **有空再修**:问题 5、8、9,以及 🟢 优化建议部分。
4. 第一轮 20 项修复中,**19 项确认无问题,只有 #17(RTK 首帧居中)需要返工**。

本轮同样只做诊断,不修改任何 `v2/` 下的源代码。
