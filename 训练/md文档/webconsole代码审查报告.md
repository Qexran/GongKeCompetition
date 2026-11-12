# GongKeCompetition `tools/webconsole` 控制台代码审查报告

> 审查范围：后端 8 个模块（`server.py` / `bt_link.py` / `bt_serial.py` / `capture.py` / `host_tools.py` / `tuning.py` / `map_codec.py` / `arm_action.py`，约 5800 行）+ 前端（`static/app.js` 4762 行、`static/index.html`）。
> 验证方式：通读全部源码 + 运行完整测试套件（`test_server / test_tuning / test_host_tools / test_bt / test_capture / test_arm_action`）。

## 总体结论

这套代码质量相当高。以下铁律贯穿一致，是多数工程代码做不到的纪律性：

- **无操作保存 = 字节级零 diff**（未编辑条目按原始物理行回吐）
- **写盘前 `.bak` 备份 + tmp + `os.replace` 原子替换**
- **锁挂模块级单例**（`bt_link.LINK` / `capture.CAMERA`），而非每请求一把
- **409 乐观锁 + `WRITE_LOCK` 串行化** 组合防多客户端 TOCTOU
- **id 只增不重用**（预设编号 / 动作编号），宁可编译期硬失败也不静默指向别处
- 测试 417 个、全程沙箱，注释里全是实踩过的坑

但仍有若干真实问题，按严重程度列在下面。

---

## 一、测试现状

417 个测试，416 绿，**1 个失败**：

- `test_server.py:650` `test_parse_map_exposes_id_and_next_id`

**这不是产品代码的 bug，是测试断言写错了**：它对真实 `map.c` 的沙箱副本断言"预设 id 必须连续 1..N、`next_id == N+1`"。但当前 `Application/map.c` 的注册表里 **13 号已被删除**——而 id 按设计"只增不重用"，出现空洞是**合法状态**（`map.h` 里 `SCRIPT_PRESET_NEXT_ID = 20u` 也证明了这一点）。

这个断言与自家设计矛盾，任何带空洞的合法仓库状态都会挂。**建议**：改成断言"id 严格递增且唯一、`next_id > max(ids)`"。

---

## 二、潜在问题（按严重程度）

### P1 — map.c 写回路径对用户输入零清洗，与自家解析器的已知约束冲突

`bt_link.sanitize_tag`（`bt_link.py:134`）的注释写得很清楚：标签里的 `"`、`//`、不配对 `()` 会破坏 C 字符串 / 解析器的括号计数。机械臂路径（`make_arm_snippet`、`_fmt_gripper_step`）确实都过了这层清洗，但**「地图」Tab 自己的写回路径没有**：

1. **步骤 tag 裸插值**：`map_codec.py:325` `'"%s"' % item.get("tag", "")`。「检查器」里的"积木名"输入框（`app.js:649`）没有任何客户端或服务端校验。用户在 tag 里敲一个 `"`，写出的就是 `"我"的"`——不只是编译错，更糟的是**重载时** `_split_call` 的字符串跟踪（`map_codec.py:97`）会被多出来的引号带偏，参数切分错位，下一次保存可能**悄悄改写邻近条目**。
2. **分区标题裸插值**：`map_codec.py:387` `"    /* ===== %s ===== */" % title` 同样裸插值。双击分区名改名（`app.js:486`）输入 `*/` 会提前闭合注释，后面的半行变成**活的 C 代码**——这正是 `_preset_header_label`（`server.py:598`）花了整段注释防的那件事，只防了预设头注释这一处，数组体内的分区标题漏了。

**建议**：在 `_serialize_step` 和 `serialize_map` 的 section 分支写盘前统一过一遍清洗（tag 复用 `sanitize_tag`，分区标题洗掉 `*/`、`/*`、换行），并在前端 tag 输入框做即时提示。

### P1 — `tuning.write_meta` 没拿 WRITE_LOCK

`tuning.py:507-519`。`write_config` 全程持锁（`tuning.py:389`，注释还专门解释了 TOCTOU 窗口），但同文件的 `write_meta` 是裸的"读-改-写"。两个客户端并发时（一个调 `/api/tuning` 改值、一个调 `/api/tuning/meta` 改注释），会交错写 `app_config.h`：轻则丢一次更新，重则写坏文件，且双方都各自覆盖 `.bak`。

**建议**：一行修法——套上模块里现成的 `WRITE_LOCK`。

### P2 — 积木库缓存不感知外部修改，git pull 后多客户端同步"刷新了个寂寞"

`Handler.catalog_cache`（`server.py:1233`）只在控制台自己的保存动作里失效（`server.py:1376 / 1394 / 1403`）。如果有人**手改了 `app_script.h` 或 git pull**（团队协作下这是常态），前端的多客户端同步机制会正确检测到 `script_h` mtime 变化、去拉 `/api/catalog`（`app.js:4431`），并提示"积木库元数据已被其他客户端修改，已刷新"——但服务端吐回来的是**旧缓存**。用户看到提示却看不到任何变化，非常难查。

**建议**：把缓存键做成 `(SCRIPT_H.mtime, SCRIPT_C.mtime, MAP_H.mtime)` 元组，不匹配就重解析（解析本身是纯读，几十毫秒）。

### P2 — `write_map` 对 map.c 无条件重写，与 arm_action 的"内容不变一个字节不写"纪律不一致

`server.py:860-864` 无条件 `copy2(.bak)` + `write_text`，即使 `new_text == text`（无操作保存零 diff 时也是如此）。后果：无操作保存也会 bump mtime → 其他干净客户端被 `/api/rev` 判成"远端改动"而自动重载一轮。`arm_action.py:712-719` 和 `_write_preset_header`（`server.py:736`）都明确守了这条（注释还写明了理由），唯独 map.c 主体漏了。

**建议**：加一个 `if new_text != text` 的守卫即可。

### P2 — 静态文件路径检查有前缀缝隙

`server.py:1263` `str(fp).startswith(str(STATIC_DIR.resolve()))`——没有结尾分隔符，`/static/../staticX/…` 这类指向**同级 `static*` 目录**的路径能通过检查。当前目录布局下恰好没有这样的兄弟目录，实际不可利用，但这是经典的 startswith 陷阱。

**建议**：换 `fp.relative_to(STATIC_DIR.resolve())`（3.9+，包 try）或 `os.path.commonpath` 判断；项目声称支持 3.8 的话注意兼容。

### P2 — 串口 close 与读线程存在 Win32 句柄竞态

`SerialPort.close`（`bt_serial.py:525`）不持 `_io_lock` 直接 `CloseHandle`，而读线程可能正阻塞在同句柄的 `ReadFile`（`bt_serial.py:549`）。`BtLink.disconnect` 先 join(1.0) 再关（`bt_link.py:325-331`），读超时 50ms 时通常来得及，但 join 超时的分支就是裸竞态：句柄值被复用后 ReadFile 可能读到别的东西或报假错。

**建议**：close 里也拿 `_io_lock`，或在关闭前 `CancelIoEx`。

### P3 — 一批小问题

| # | 问题 | 位置 | 说明 |
|---|------|------|------|
| 1 | `WriteFile` 部分写未处理 | `bt_serial.py:559-568` | `nwritten.value < len(data)` 时命令被静默截断，返回值没人看。串口流控下虽罕见，但 pyserial 都是循环写的 |
| 2 | POST 无 Content-Length 上限 | `server.py:1318` | 超大 body 会整个读进内存；缺 body 时 `json.loads("")` 抛异常被兜成 500 而不是 400 |
| 3 | 一次 `$B,POS` 超时即自动踢出编辑模式 | `app.js:2710-2711` → `bt_link.py:409` | 蓝牙偶发一拍抖动（超时 1.5s）就 `btSetEdit(false, true)`，台架上正调到一半被退出。若非有意为之，建议连续 2~3 次失败再退 |
| 4 | 新建分区循环有潜在 bug | `tuning.py:439-447` | `for p in fresh` 迭代过程中重新赋值 `fresh`，若 payload 里带两个同新分区的参数会产生重复的空分区头。当前 UI 一次只发一个参数所以不可达，但是个埋着的雷 |
| 5 | `btWriteParam` 回车+失焦双写 | `app.js:2283-2285` | Enter 写入后输入框仍持焦点且 `p.value` 未更新，随后的 blur 会把同一个值再 SET 一遍。无害但多一次蓝牙往返 |
| 6 | `host_tools._ps` 编码 | `host_tools.py:147-158` | `text=True` 没指定 encoding，中文 Windows 上按 GBK 解码，报错文案可能乱码——`bt_serial._ps_stdout`（`bt_serial.py:169`）已经踩过并修了同样的坑，这里没跟 |
| 7 | 重复注释块 | `server.py:124-130` | 同一段话贴了两遍，纯清理 |

### 安全姿态（已知且文档化，但值得再列一次）

服务端绑 `0.0.0.0`、**无任何鉴权**；Origin 校验（`server.py:1157`）只防浏览器侧的跨站 / DNS rebinding，**非浏览器客户端（curl、任何局域网程序）完全放行**。

暴露面包括：

- 写仓库源码（map.c / app_config.h / app_script.h / app_arm_action.c）
- 改 `~/.ssh/config` 与网络位置
- 拉起 mstsc / Trae / VS Code
- 驱动真实舵机、开摄像头

比赛局域网内可接受，但如果场地网络不可控（观众 / 其他队伍同网段），建议加一个**可选的共享口令**（环境变量 → 请求头校验，几十行就能做），或至少在 CLAUDE.md 里把"非浏览器客户端无防护"这条写显眼。现有的 `GKS_CONSOLE_HOSTS` 环境变量只解决"误拒"，不解决"放太开"。

---

## 三、优化建议

1. **app.js 拆分**：4762 行单文件、全部全局函数，按 Tab 拆成 ES modules（服务端 MIME 已支持，`<script type="module">` 即可）。注意 `srccheck.py` 的契约测试以整文件文本跑 node 探针，拆分要同步改测试——若比赛临近，这条可以放最后。
2. **`parse_catalog` 的失效粒度**：结合 P2 缓存修复顺带做——现在任何一次 map/arm 保存都会把缓存整个打掉，下次 `/api/catalog` 全量重扫 `Application/*.h` + QEPack 头文件；按 mtime 键控后未变的头文件天然命中。
3. **前端 `renderCanvas` 全量重建**：地图画布每次编辑都重画全部 zone/block，条目量大时（>200 步）会有可感延迟。当前路线规模没问题，不必动，留个心眼即可。
4. **三个轮询的合流**：`/api/rev`（5s）、`/api/bt`（3s）、采集状态（2s）各自独立轮询，都是轻请求，现状没问题；若以后加 Tab，考虑合并成一个 `/api/heartbeat` 按需取。

---

## 四、修复优先级建议

| 优先级 | 事项 | 理由 |
|--------|------|------|
| 1 | map 写回路径的输入清洗（P1-1） | 小改动大收益，防自伤式 C 语法错误与静默数据改写 |
| 2 | `tuning.write_meta` 补锁（P1-2） | 一行改动，堵住并发丢更新 |
| 3 | catalog 缓存按 mtime 键控（P2-3） | 团队 git pull 协作下的真实困惑源 |
| 4 | `write_map` 加"内容不变不写"守卫（P2-4） | 消除无操作保存引发的多人重载抖动 |
| 5 | 失败测试的断言修正 | 让测试套件回绿，恢复"全绿=正常"的信号价值 |
| 6 | 其余 P3 逐条顺手修 | 均为小改动 |
