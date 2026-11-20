# CLAUDE.md

本文件为 Claude Code 在 GKS_Web 仓库中工作时提供指导。

## 项目概况

GKS 工科赛的 **Web 控制端**，与触摸屏控制台（`GKS_Competition/control_panel.py`）同等地位，
开机常驻（systemd `gks-web.service`），**独立于主程序 `gks_competition`**。目标是：不开远程桌面，
浏览器访问 `http://<IP>:9000/` 即可完成比赛调试 / 单片机模拟 / 树莓派电源管理。

## 核心原则

- 两个项目、两个 systemd 服务、两个进程，互不影响。
- 本仓库只 import `GKS_Competition` 里**无副作用**的库模块（`protocol`/`camera`/`tasks._qr`/
  `log_buffer`/`config`），绝不 import 有副作用的 `main.py`（会启动服务器、重定向 stdout）。
- 自身配置模块叫 `webconfig.py`（**不叫 config.py**），避免与 `GKS_Competition/config.py`
  在 sys.path 上重名导致 `from config import ...` 解析冲突。

## 文件说明

- `app.py` — HTTP 服务 + 路由 + 内联网页 UI。端口 9000。启动时同时拉起无头单片机模拟器。
- `console.py` — 控制台后端（网络/服务/电源/系统状态），移植 control_panel.py 非 GUI 逻辑。
- `simulator.py` — 无头单片机模拟器（TCP 线程），`main.py` 以 `SIMULATOR_MODE=1` 连接后，
  网页发指令帧给 main.py。
- `webconfig.py` — 本仓库配置（端口、模拟器地址、GKS_Competition 目录、服务名）。
- `gks-web.service` — systemd 服务文件。

## 「网络」页签（WiFi 扫描 / 切换 / 开机优先级）

给「手机热点不常开」准备的：在页面上扫附近 WiFi、需要密码就输、切过去，并把它设为
**开机第一连接点**。逻辑全在 `console.py` 的 `# ==== WiFi ====` 一节，`app.py` 只做参数转手。

- 端点：`GET /api/wifi/status`、`GET /api/wifi/scan`、`POST /api/wifi/connect`、
  `POST /api/wifi/profile`（`{action}` ∈ `priority|prefer|delete|revert`）。
- **切换必须先应答再动手**：请求正是从"即将被切走的网络"进来的，同步等 nmcli 就会发不出响应。
  所以 `POST /api/wifi/connect` 只做「不改网络」的准备（建/改档案、设优先级）后立刻返回，
  真正激活交给后台 worker 线程（开头 `sleep 1` 让响应先发出去 —— 同 `restart_web_service` 的手法）。
- **成功判据是「关联上 + 拿到 IPv4」**，不是"nmcli 返回 0"：密码错和卡在 DHCP 都会让
  `con up` 看似成功。worker 在 `WIFI_VERIFY_GRACE`（20s）内轮询 `_wlan_connected_to()` + `_wlan_ip()`。
- **失败自动切回**：`rc=4/10` 是确定性失败（密码被拒 / 没这个网络），不等宽限期直接回滚；
  回滚前会**删掉本次新建的档案**并把优先级**还原成切换前的快照**（`prev_prios`）——
  别改成"把当前网络设成 100"，那正好是用户想避开的（让热点当首选）。
  全部动作幂等：`wifi_revert()` 会先看目标是否已经活动。
- 开机优先级 = `connection.autoconnect-priority`，**数字越大越先**。开机时 NM 只在**当前范围内**
  的 SSID 里挑，首选不在范围内会自动落到下一个（所以旧档案别删）。`有线连接 1` 是 -999
  且在 `/run`（重启即消失）；⚠ 插网线时 eth0 会与 wlan0 **并行**激活并抢走默认路由（metric
  100 vs 600），优先级管不了它。
- ⚠ `Mi 10 Ultra` 这类「`autoconnect=yes` 但 keyfile 里没有 `psk=`」的档案，开机连它必然失败、
  还会退避重试一轮 —— 页面因此显示「⚠ 未保存密码」列把它暴露出来（`_has_stored_psk` 只
  `grep -c '^psk='`，**不取值**）。
- 页面安全：SSID 是**攻击者可控的广播字符串**（一个叫 `<img src=x onerror=...>` 的热点就能
  打进来，而本页有断电按钮）。所以拼进 `innerHTML` 的 SSID 一律 `esc()`，且**绝不内联进
  `onclick`/属性**（`esc()` 不转义引号）—— 一律传**数组下标**。

### ⚠️ nmcli 相关的两条铁律 + 两个字段坑

1. **一律用 `console._nm()` 的 argv 列表，绝不用 `run_cmd()`**（那是 `shell=True`）：
   SSID 与密码都可能含 shell 元字符（`"; rm -rf ~` 是合法 SSID）；而且 subprocess 超时的
   异常文本**带完整命令行**，会顺着 journal 把密码写进日志。
2. **密码只走 keyfile 的 stdin**（`_write_root_file` 的 `sudo tee` + 0600 root），
   不进 argv、不进日志、不回给浏览器。环境必须是 **`LC_ALL=C.UTF-8`**：用 `C` 会把
   `有线连接 1` 变成 `????`，用默认的 zh_CN 又只拿到中文报错没法匹配。
   ⚠ 别用 `nmcli --show-secrets`；`journalctl -u gks-web` 与页面「日志」tab 是同一份输出。
3. _字段坑（各踩过一次，都是"命令不报错但判断反了"）_：`-f FILENAME` 只在**列表形式**
   （`con show`）有效；`con show id <name>` 的 `-f` 只认**设置字段**
   （`connection.id`、`802-11-wireless.ssid` 那种），给它 `NAME`/`FILENAME` 会
   `invalid field` + **rc=2**。要判断"档案存不存在 / 在哪"一律走列表形式再查行。
4. `dev wifi rescan` 是**异步**的，紧接着 list 只会拿到"扫了一半"的结果（实测只剩当前那一个 AP）
   → 用 `dev wifi list --rescan auto`（会等扫描完成，约 1.5s）。

## ⚠️ 新增接口 / 指令的同步清单

本仓库多处是「一件事要在两个地方登记」，**只改一处不会报错，而是运行时静默失效**，
排查代价很高（页面一直转圈、按钮没反应、日志不回显）。按下面逐项打勾：

**新增一个 HTTP 接口（`app.py`）要动 3 处：**

1. `WebHandler.do_GET` / `do_POST` 开头的**路由白名单元组**（`if path in (...)`）——
   漏加 = 直接落到 `else: self.send_error(404)`
2. 同函数里的**分发分支**（`elif path == ...`）—— 只加分支不加白名单，同样 404
3. 前端调用方（JS）**必须有 try/catch**：`postJSON`/`fetch` 拿到非 JSON 响应（404/500 错误页）
   会抛异常，调用方不接就会**静默卡在「进行中」状态**，看不出是失败
   （`postJSON` 现已把非 JSON 响应转成可读的 `HTTP <status>（<url>）`）

**新增一条模拟器指令（`simulator.py::COMMANDS`）要动 4 处**（详见
`GKS_Competition/CLAUDE.md` 的同名清单）：本表 + `GKS_Competition` 的
`handler.py::_COMMANDS`、`tasks/__init__.py` 导出、`docs/串口协议.md` §6；
跨到单片机仓库还要改 `app_rpi.h::emRpiTaskTdf` + `app_rpi.c::s_apcTaskCmd[]`。

**生效方式**：改 `app.py` / `console.py` / `simulator.py` 后必须重启 `gks-web.service`
（页面上「控制 → Web 控制端自身 → 重启控制端」按钮即可，不必 SSH）。

⚠️⚠️ **重启控制端会顺手杀掉「调试模式」起的 main.py（2026-10-16 现场踩到）**：
`gks-web.service` 的 unit 里没写 `KillMode` ⇒ systemd 默认 `control-group` —— `systemctl
restart` 是给**整个 cgroup** 发 SIGTERM，而页面「启动主程序」的 `start_main_py()` 只是
`Popen(..., start_new_session=True)`：只换了会话/进程组，**cgroup 还是 gks-web.service 的**。
现场：台架上次次点「重启控制端」改代码，`main.py` 就一次次"无缘无故"退出（它的
`logs/run_*/log.txt` 会留下正常的「收到终止信号，正在清理资源...」——不是崩溃）。
· 别在 main.py 正在跑动作/被单片机驱动时点重启控制端（会打断这一轮）。
· 要根治：`sudo systemctl edit gks-web.service` 加
  `[Service]` + `KillMode=process`，再 `sudo systemctl daemon-reload`
  （`process` = 只给主进程（app.py）发信号，它拉起的兄弟进程不受牵连）。
· 主程序用 systemd 跑（`gks.service`）则不受影响：那是另一个 cgroup。

### ⚠️ 「摄像头调试区」的扫码按钮：方法与**相机**两根轴都要对

「模拟器」Tab 的摄像头调试区有三个扫码按钮（`scanQR(method)` → `GET /scan?method=`）：

| 按钮 | method | 相机 | 结果显示 | 服务端先停哪路推流 |
|---|---|---|---|---|
| 扫码二维码(OpenCV) | `opencv` | 扫码副摄 | `box-qr` | `_stop_stream("qr")` |
| 扫码二维码(pyzbar) | `pyzbar` | 扫码副摄 | `box-qr` | `_stop_stream("qr")` |
| **扫码二维码(主摄)** | `wechat_main` | **主摄** | **`box-main`** | **`_stop_stream("main")`** |

**⚠️ 三条一起改，漏一条就静默失效**（`_handle_scan` 里的方法白名单 + 选哪个 run_fn +
**停哪台相机的推流**；前端选 `box`）。最容易漏的是**停推流**那一处：主摄那条若照抄
停 `"qr"`，主摄仍被推流线程占着 → `open_camera` 直接失败，而日志里只有一条
「主摄像头打开失败」，看着像硬件问题。

`wechat_main` 走的就是单片机命令 **`"D"`** 的那条路（`tasks.run_qr_wechat_main`）——
所以台架上在这里看到能识别，就等于「单片机发 `"D"` 能拿到结果」。
模拟器指令按钮 `"D"` 是另一条路（真发给 main.py），它同样要停**主摄**推流、显示在主摄格
（`simSend` 里 `key === 'D'` 那一支）。

⚠️ 两条命令在 Pi 端共用 §6a 的同一种结果帧，也都写同一个 `logs/qr_last.jpg`，
所以 `pollQrFrame(t0, boxId)` 只有一个端点，**靠 boxId 参数决定显示到哪一格**。

### ⚠️ 「靶标识别(pi)」按钮（`scanTarget(1)` → `GET /target?pi=1`）

那颗 CSI（`GKS_Competition/camera_picam.py`）固定在底盘上、几何稳，台架上靠它读靶 cx 去填
单片机的 `tc_cx_aim_pi`（瞄准列）。两个坑都在 2026-10-16 修掉，改这条链路时别再踩回去：

1. **服务端必须回 `cam_is_pi`**（`_handle_target` 的 JSON）。前端那句 `[实际: pi/CSI | 主摄]`
   就是按它显示的 —— 漏了这个字段，**页面永远显示"主摄"**，哪怕这一帧明明是 CSI 出的
   （看着就像"pi 那条路从来没成功过"）。`_handle_hostage` 一直有，`_handle_target` 当时漏了。
2. **必须先 warm + 等就绪再识别**（`PICAM_MANUAL_WAIT_S`，最多 6s，可被 `/abort` 打断）。
   冷启动 CSI 要 ~3s（列相机 + 首帧 ~2s + 丢收敛帧），而 `run_target_detect` 给 CSI 那一次只留
   `config.PICAM_TRY_BUDGET_S`（默认 **2s** —— 那是**给单片机那条路**的：超了就把主摄那次的
   预算吃光）。不等这一下 → **每次点都是"2s 花完 → 退回主摄"**。
   ⚠️ 这是**手动按钮**，不受单片机时限约束，等两三秒完全值得。
3. 不用再手工关 pi 预览了：`camera_picam._open_now()` 起流失败会走「收掉自己上次运行遗留的
   取流进程 → 请 GKS_Web 让出 pi 预览（同进程钩子 `_yield_pi_preview`）」这条链（2026-10-16 加）。
4. ⚠️ 这些行为全在 `GKS_Competition/camera_picam.py` 里 → 改完那份**两个服务都要重启**
   （`gks-web.service` 与 `gks.service`）。只重启 main.py 的话，网页进程用的还是旧模块。

## 「标靶练习」页（`/practice`，2026-10 加）

与 `/parallax` 同款**独立单页**姿态（主页 tabnav 只放跳转按钮；低频标定/采数工具改坏只坏自己）。
用途：OLED 第 3 页「执行任务→激光射靶」（单片机预设 15）打一发真实流程，打完在预览里
点光斑 → 攒「(实际打点, 检测靶心) 误差」数据集，供离线分析落点随机（同位置重复性 /
不同位置视差 / 不同角度几何）。**本页只读数据不触发**——触发在 OLED，数值全部来自
log.txt 的 TGTS 调试串 + `_target_stream.py` 停流时归档的定格帧（见 GKS_Competition 侧 CLAUDE.md）。

- 端点：`GET /practice`、`GET /api/practice/shots[?run=]`（解析活动 run 的 log.txt → 发次列表）、
  `GET /api/practice/records`、`GET /api/practice/recordlog?ts=`、`GET /api/practice/condition`、
  `GET /api/practice/frame?file=`、`POST /api/practice/mark {x,y,run?,time?,mode?}`、
  `POST /api/practice/delete {ts}`、`POST /api/practice/reccond {ts,pos?|angle?|note?}`
  （双击记录表「位置/角度」就地改该条标签）、`POST /api/practice/condition {pos,angle,note}`。
- **`_parse_shots` 是发次解析的唯一实现**（模块级函数）：一发 = 一条 `TGTS cfg` 行开启到下一发
  cfg/EOF；⚠️ 单片机的 `fin/ok` 行与 Pi 的 `[靶标流] 定格帧/终态` 行**谁先谁后不定**
  （两进程并发打印），故按块内 anywhere 解析、不依赖行序；时间戳滚动携带（TeeWriter 同秒行
  只有缩进没有 `[HH:MM:SS]`）。
- **标注与 /parallax 的两点刻意不同**：① 检测失败**不拒绝**（打点本身就是数据，检测只是
  辅助算误差）；② 不做「点击必须贴近某靶」的门（`PARALLAX_PICK_TOL_PX` 那道）—— 打歪了
  正是要量的东西，命中靶取离点击最近的那颗。
- **预览三视图**（切换纯查看、不写数据）：`定格帧·标记前`（默认，本次闭环最后看着的那张，
  经 `/api/logs/frame`）／`实时`（MJPEG，打靶期间会被主程序抢走，正常）／`标记后`（该发
  记录的 dotann 回看）。**定格帧视图里点击 = 标注**（`mode:"frozen"`）：底图无光斑（定格于
  开激光之前），位置由人眼看物理靶映射；车臂都没动、几何与实时帧一致，靶心检测跑在**这张
  图**上（不碰相机、无抢占）。实时视图点击 = `mode:"live"`（抓帧直测光斑）。记录带
  `mark_mode: live|frozen` 供分析端区分打点来源。点完自动切「标记后」。
- **数据落 `GKS_Competition/JiGuangLogs/`**（`shots.jsonl` + `condition.json` +
  `frames/<ts>_{dot,dotann,final,finalann,seqNN}.jpg` 证据拷贝）。⚠️ 定格帧与**过程帧序列**
  （aimseq_*，标注时按「cfg→结束行」时间窗配对拷贝，窗外的属于别发）从 run 目录**拷贝**过来
  而不是引用——run 目录会被「日志」页清理，拷出来的才是稳的证据。改 record 字段名会破坏
  已攒数据集的可读性，慎重。
- **标注是 upsert、不是追加**（键 = run + cfg 行时间）：同一发重复点标注 = **修正**那条记录
  （覆盖写 jsonl、旧证据帧一并清理，历史同键重复行也顺带收拢成一条）——追加式会在手抖微调时
  刷一堆近重复行（真踩过）。页面记录表每行有「删」按钮（`/api/practice/delete`，按 ts 删一条，
  连证据帧）。
- **每一发记录自带完整过程 log**（`rec["log"]` = 该发 cfg→下一发之间的**原始日志行**，含
  TGTS 逐拍串/换挡/流状态/定格帧行，≤800 行；标注时从 run 的 log.txt 固化进 jsonl）——run 目录
  被清理后，这就是追溯那一发全过程的**唯一副本**。列表端点把 `log` 剥成行数 `log_n`（载荷），
  按条取全文走 `GET /api/practice/recordlog?ts=`（text/plain，页面「过程log」列的链接）。
  解析器为此给每发记 `lines`（原样含时间戳前缀）与 `end_time`（ok/fail/abort 行时刻 =
  seq 拷贝窗右界）。
- 打靶期间预览被主程序抢相机（`/abort` 跨进程兜底，正常现象），打完点「刷新预览」。
- 「详细日志/帧序列归档」开关在「参数」页（`TARGET_STREAM_VERBOSE` / `TARGET_STREAM_ARCHIVE_FRAMES`，
  两层门控），改完**要点「重启 main.py」**才对主程序生效（双进程陷阱，见 GKS_Competition 侧文档）。

## 「日志」页的相机帧（分类 / 训练模式 / 失败也留档，2026-10-21 加）

run 目录 `frames/<key>_<序号>.jpg` 的回看网格（`loadRunFrames`）顶部有一排分类 chips：
**[全部] [QR] [Road] [Color] [Target] [Hostage]**。分类是**前端按文件名 key 前缀**推的
（`frameCatOf`），后端只给文件名列表 —— 改归档 key 不用动它，认不出的 key 只进「全部」。
key 一览：`qr`/`qr_<算法>`（`_main` 结尾=主摄）→ QR；`road`/`road_yaw` → Road；
`color`/`rect`（矩形地板在同一区域）→ Color；`target`（含历史键 `aim*`）→ Target；
`hostage` → Hostage。

- **训练模式（`config.TRAIN_MODE_EN`，「参数」页「训练模式」组）**：开 = 每次识别归档
  结果帧时**同序号**再存一张处理前的原始帧 `frames/<key>_before_<n>.jpg`（给离线复跑/
  再训练留"当时的现场"——标注帧会骗人，原图不会）。生效走 `config.live_value()`
  （logstore.archive_frame 里现读 override JSON）：**网页一勾，main.py 与 GKS_Web
  两个进程下一拍同时生效，不用重启**（帧归档一次任务才一次，读盘零负担；高频参数
  别学这条）。before 只在**成功**时配对产出（失败回的本来就是原始帧，再存一份是重复）；
  QR 的 before 是**裁剪/缩放前**的整帧原图（结果帧画在裁剪后的 proc 上）。
  文件名形态 `xxx_before_0001.jpg` 天然过 `_FRAME_RE` 白名单，`archive_frame` 的序号
  扫描是锚定正则、两套序号互不干扰；前端把 before 排在其同序号 after 前面并标〔原始〕。
- **失败也留档（统一记录接口）**：`handler._save_frame(key, result, legacy_path)` 与
  本仓库 `_archive_scan_frame(key, result, legacy_name)` 都收**整个 result dict**，
  成功失败走同一个入口。任务侧的义务（6 个 `tasks/_*.py` 都已做）：失败/打断时把
  **最后的原始帧**放进 `result["frame"]`（一帧都没检到也一样——那才是排障最想要的
  现场）；成功且帧被标注/裁剪过时另给 `result["frame_raw"]`。logstore 收到
  `ok=False` 时在**拷贝**上盖红字 FAIL 印再落盘（回看网格不用点开猜成败；必须在
  拷贝上画——租约 `read()` 返回的是缓存数组引用，原地画会把租约里的帧涂花）。
  相机根本没打开成功的那种失败没有图可存，仍是零帧，属正常。

### 帧右键「验证」：离线复跑当前算法（2026-11-06 加）

「日志」页回看网格里对任意一张帧图**右键 → 🔍 验证**：服务端把那帧喂进与在线
**完全同一套**的单帧检测管线，弹窗（#validateModal）左右并列「输入帧 | 当前代码
标注结果」+ 检测摘要 —— 改完阈值/算法不用再上车重跑任务，对着历史帧就地二验。

- 端点：`POST /api/logs/validate {run, file}`（`_api_logs_validate` + `_vld_run_algo`，
  按「新增接口清单」三处同步：白名单元组 / 分发分支 / 方法本体）。
- **输入选择（`_vld_pick_input`）**：验证要吃**原始帧** —— 同序号有训练模式配对的
  `<key>_before_<n>.jpg` 就自动换成它（meta 行注明「已自动换成配对原始帧」）；
  `aimann_<n>` 的原始孪生是 `aimfinal_<n>`；本就是原始帧（`*_before_*` / aimfinal /
  aimseq）或没有配对时原样验证并如实注明（旧标注框可能轻微干扰检测）。
- **算法映射（`_vld_algo_of`，与前端 FRAME_CATS 同一套归类）**：`qr*` → 新引擎
  `decode_frame`；`road` / `road_yaw` / `color` / `rect` / `hostage` 各回各任务；
  `target` 与历史键 `aim*` → 靶标单帧检测。认不出的 key 拒绝并说明，不猜。
- ⚠️ **纪律：只用各任务的纯函数件**（`_detect_frame` / `_detect_one` / `detect_rect` /
  `_detect` / `decode_frame`），全程不碰相机、串口、租约 —— 与在线任务同时进行也不抢
  设备。各任务的 `_annotate`/`_detect_one` 同名，导入一律用模块别名
  （`from tasks import _road as _t_road, ...`），直接 from-import 会互相覆盖。
- ⚠️ 参数一律 `gks_config` 现读（与在线同源同值，参数页改完下一帧验证就是新值）。
  靶标 cfg 与 `tasks/_target.py::_run_detect_once` 同源（含 `fallback_q_min`）；CSI
  归档帧（宽 2592）按帧宽识别走 pi 那把 `TARGET_R_MAX_PI` 尺 + 白心救回，主摄帧照旧。
- 前端：`#frameCtx` 右键菜单 + `frameMenuCache` 下标引用（与 runCache 同纪律）；
  全局收起监听与 #runCtx 同款（点别处/滚/Esc/别处再右键）。菜单拼接 img 属性串时
  **onclick 的收尾引号别丢** —— 丢一个引号整段属性错位还只是菜单失灵，JS 字符串
  字面量没闭合则整页脚本解析失败（两个都真踩过，`node --check` 抽出的脚本能当场抓到）。
- 验证是同步计算的（ThreadingHTTPServer，不挡别的请求）：靶标满分辨率 ~0.2s、
  人质首帧含 YOLO 加载 ~1s、二维码最坏 ~4s，meta 行都带耗时。

### 主摄参数悬浮窗（模拟器 Tab，2026-10-30 加）

模拟器 Tab 的「主摄参数 · 实时预览（悬浮窗）」按钮弹出一个**可拖动**的浮窗：自带一路
`/stream?cam=main` 预览 + 白平衡（自动 / 固定 2800~6500K）+ 曝光（自动 / 手动 1~5000，
单位 100µs）+「载入档位」（main / road / hostage 的 `config` 默认值）。

- 端点：`GET /api/camctl?cam=main`（读当前四项）、`POST /api/camctl`
  `{cam, auto_wb, wb_temp, auto_exposure, exposure}`（应用）或 `{cam, profile}`
  （载入档位，走 `camera.profile_params`）。
- **只改当前预览流的 V4L2 控件**（`_streams["main"].cap`），不写 `config_override.json`、
  不动 main.py。关预览/重开相机会按档位重新设满四个控件（`open_camera` 的纪律），
  所以这些值**不会**漏进识别任务。
- **没有主摄预览流就报错**（`{ok:false, message}`），**不**临时开设备 —— 主摄跨进程独占，
  临时开会和主程序抢 `/dev/camera_main`。
- 悬浮窗开预览会顶掉 Tab 里那路主摄预览（服务端 `_send_stream` 先 `_stop_stream("main")`）；
  关浮窗时若原来 Tab 里有预览会自动重开（`closeCamTune` 里先 `await /abort?cam=main`）。
  主摄被 main.py 占用时浮窗的 `<img>` 会显示「预览打不开」。
- 主摄是独占设备：**别在 main.py 跑识别任务/动作时开它**；台架调参用模拟器模式。
- 调好的值要落到任务：把读数填到「参数」页的 `CAM_MAIN_*` / `ROAD_*`（或 `HOSTAGE_*`）→
  保存 → 重启 `gks.service`。⚠️ 主摄档改白平衡会平移色球/靶标的 HSV 阈值，改前先验证。

## 相机持有纪律（2026-10 修「拔插 USB 才能恢复」的 bug）

本进程持有相机有**三种形态**，让出时必须全都覆盖：① 预览推流（`_streams` 注册表，
`_stop_stream` 停）；② 就地识别 adopt 的**租约**（`camera_lease.LEASE`，本进程常驻）；
③ `_grab_main_frame`/`_check_camera` 的临时句柄（finally 里自己关，无风险）。
旧版只有 ① 被 `/abort` 覆盖：网页里点一次就地识别 → 相机被本进程租约**永久**攥住
（当时本进程没跑 `lease_watchdog`）→ main.py 的 0.3s×3 抢相机重试永远输 → 只能拔插 USB。
两条修复（都验证过）：启动时拉起 `camera_lease.lease_watchdog` 线程（15s 空闲自释放，
main.py 同款纪律）；`_handle_abort` 里对 `camera_lease.release_lease_if(cam=="qr")`
（`/abort` 语义 = 让出该相机**一切**持有形式）。⚠️ 以后往本进程加任何「会把相机
hold 过请求生命周期」的路径，先想清楚谁负责放。

## 端口约定

| 端口 | 用途 |
|------|------|
| 9000 | GKS Web 控制端（本仓库） |
| 8000 | camera_web（**已废弃**；仅 `python3 camera_web.py --port 8000` 独立调试用） |
| 8080 | web_preview（MJPEG 推流） |
| 9997 | 无头单片机模拟器 TCP |

## 与 GKS_Competition 的契约

- 串口协议定义在 `GKS_Competition/protocol.py`，本仓库 `simulator.py` 通过 sys.path 复用。
- 模拟器指令表 `simulator.py::COMMANDS` 须与 `GKS_Competition/handler.py::_COMMANDS` 保持一致。
- 摄像头设备节点 / 分辨率等参数以 `GKS_Competition/config.py` 为准（本仓库复用同一份）。

## 部署

```bash
sudo cp gks-web.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable gks-web.service   # 开机自启
sudo systemctl start gks-web.service    # 立即启动
```

浏览器访问 `http://<IP>:9000/`。

## 模拟器使用（模拟 / 正常 二选一，无静态开关）

模拟器与真实串口平级：main.py 会**同时等待「真实串口」和「模拟器虚拟串口」两个信号**，
谁先就绪就进哪个模式（无 `SIMULATOR_MODE` 配置）：

1. 网页「单片机模拟器」区点「启动模拟器」→ 模拟器开始监听 127.0.0.1:9997。
2. 正在等待信号的 main.py 检测到模拟器就绪，自动切入**模拟模式**（main.py 日志显示「模拟模式」）。
3. 网页点命令按钮发任务，观察帧交换日志与 main.py 日志。
4. 切回正常模式：点「停止模拟器」，插上真实单片机（串口就绪即自动切正常模式）。
5. 比赛时不要点「启动模拟器」，插上单片机即可自动进入正常模式。

## 约定

- 所有 sudo 命令（systemctl/shutdown）依赖用户 `xixi` 的 NOPASSWD 权限。
- 摄像头按需打开（open_camera/release），与 main.py 共用同一摄像头时二者勿同时推流（EBUSY）。
