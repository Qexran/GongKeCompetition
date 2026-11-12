# 树莓派通信（指令 / 结果帧 / 发任务与重连）

> 由根 `CLAUDE.md` 拆分；章节标题保持原样（源码注释按节名引用）。
> 相关：`docs/uart.md`（链路层）、`docs/vision-tasks.md`（视觉任务码）

## ⚠️ 新增树莓派指令 / 结果帧的同步清单

帧协议是**跨仓库契约**（Pi 端 `GKS_Competition/`，见其 `docs/串口协议.md`），漏改不报错、只是**运行时静默失效**（指令没反应 / 结果帧被当普通帧吞掉）。

**本仓库要动的地方：**

- `Application/app_rpi.h::emRpiTaskTdf` 追加任务码 + `Application/app_rpi.c::s_apcTaskCmd[]` 追加 ASCII 命令串 —— **表下标必须等于枚举值**（表按枚举下标索引，`0` 为占位）
  - ⚠️ **单字符 `"1"`~`"9"` 与 `r`/`end`/`sd`/`rb` 已占用 → 新指令用字母**（如 `"A"` = `RPI_TASK_RECT = 14`；新码会自动进 `STEP_RPI_SEND_TASK` 下拉）
  - ⚠️⚠️ **枚举成员的尾注释必须写在同一行**（不许跨行块注释）：控制台 `server.py::parse_enums` 要求注释在同一行闭合，**跨行成员被静默丢掉、不进任务码下拉**（不报错不告警，只是选不到）。`RPI_TASK_PHOTO`(13) 与 `RPI_TASK_RECT`(14) 曾各跨 3 行 → `emRpiTaskTdf` 只解析出 12 个成员（应用 14 个），拍照/矩形**根本选不到**。**加完新成员一定跑** `python -c "import sys; sys.path.insert(0,'tools/webconsole'); import server; print(len(server.parse_enums()['emRpiTaskTdf']))"`（当前应为 **31**，2026-11 数，含 `RPI_TASK_ROAD_REV` / `RPI_TASK_HEALTH`），别只看编译过。
  - 现有 31 项（除前 8 项外均 2026-09 加）：`"1"`/`"2"`/`"3"`/`"r"`/`"end"`/`"4"`/`"5"`/`"6"` + **`"sd"` 关机 / `"rb"` 重启** + **`"7"` 靶标流起流 / `"8"` 靶标流停流** + **`"9"` 拍照** + **`"A"` 矩形地板识别** + **`"B"` 预热扫码相机 / `"C"` 预热主摄**（⚠️ 可带数值后缀（`"C,2"`）：那个数是**使用次数**不是毫秒，本端唯一吃了 `_MS_COMMANDS` 白名单但语义不同的两条，别照字面抄）+ **`"D"` 主摄二维码识别** + **`"E"`/`"F"` 二维码（自研，副/主摄）**（与 `"1"`/`"D"` **同链同结果帧**，只换解码器）+ **`"G"` 看路**：主摄找地面**黄色区** → **左边缘 x** = Δx（纵向）+ **下边缘 y** = Δy（横向，2026-09-30）+ 回 **0x08 帧**（**12B**，dx/ex/ey 是本协议**唯一一组 i16**；Δy 是 u16） + **`"M"` 反向看路**（2026-10-31 加，`RPI_TASK_ROAD_REV = 30`：与 `"G"` 同链路同 0x08 帧，Pi 侧整帧镜像后量黄区**右边缘** —— 返程机位用，`STEP_ROAD_ALIGN_WAIT` 的 `rev` 勾选框发它，数字是镜像系坐标、ideal 要按反向机位另标；见 `docs/road.md`「反向看路」） + **`"T"` OpenMV 补光临时开关**（只发不等、无参；亮↔灭由 Pi 定，**灭是临时的** —— Pi 不写 config，起流对齐会把灯点回原亮度；见 `_cmd_openmv_led_toggle`）+ **`"Z"` 四路相机健康检查**（2026-11 加，`RPI_TASK_HEALTH = 31`：阻塞，回 **0x0D 帧** 5B；由握手积木的 `health` 勾选框在握手成功后发 `"Z,<ms>"`；Pi 依次确认 主摄/扫码副摄/CSI/OpenMV 能出帧，查完**主动解除四路占用**；见下节「相机健康检查」）。⚠️ **Δx 饱和帧 Pi 会带 bit1 放行**（2026-10-30 加，饱和救援：`dx < ROAD_DX_MIN_PX` 不再整帧判负、强制置 CLIP_X，固件见 bit1 后退重测——绝不消费饱和值；部署**先烧新固件再推 Pi**，见 `docs/road.md`「Δx/Δy 饱和救援」）
    （`vRpiShutdownPi()` / `vRpiRebootPi()` = 2026-09 给 OLED 菜单留的、**至今无调用点**；与 `"r"`（只重启 GKS 主程序）不同——这两个停/重启**整台树莓派**、**只收不回**：发完心跳会停，8s 后本端链路自动判 DISCONNECTED，属预期。Pi 端跑普通用户，要提权 `sudo -n systemctl poweroff|reboot` + 免密 sudoers（写法见 `handler.py` 的 `_system_priv_cmd` 上方）。台架可直接用「地图」Tab 的 `STEP_RPI_SEND_TASK` 选这两项。）
  - ⚠️ `"r"` / `"end"` **不在 `_COMMANDS` 表里**，由 Pi「系统命令（不走任务注册表）」分支处理（`handler.py` 的 `if data == b"end"` / `if data == b"r"`）；别只看那张表。
  - ⚠️ **一致性核对的正确写法**：拿 `s_apcTaskCmd[]` 的**下标**比 `emRpiTaskTdf` 的**值**（不是比注释文字），再拿命令串去 Pi 端查 `_COMMANDS` + 特殊分支 —— 比"数某串出现几次"可靠。
  - ⚠️ **`"sd"` / `"rb"` / `"r"` 三条危险指令都要求「连发两次」才执行**（防误触，2026-09）：门在 Pi 侧 `handler.py::_require_double_send`（第一次只登记+打日志，同一条指令在 `config.SYSTEM_CMD_CONFIRM_S` 内第二次到达才真执行；**执行后等长冷却**，否则连发会打穿确认门（执行完下一帧立刻重新登记、再下一帧就通过）。**两次必须是两次操作**（按两次键各发一帧），一次按键连发两帧等于没有确认。窗口是 Pi Web「设置参数」可调项。
  - ⚠️ **阻塞类指令可以带时限**：`vRpiSendTaskCmdMs(emTask, ms)` 发 `"<命令码>,<毫秒>"`（如 `"F,7500"`），Pi 侧拆出头与时限、减个余量当预算 —— 「谁先放弃」只由本端 `I(0)` 决定。**新加阻塞类指令优先用它**，并且**在 Pi 的 `handler.py::_MS_COMMANDS` 里登记**，否则那个逗号会被判成「形状不认识」而拒掉。详见「二维码：两条相机 × 两种算法 → 超时由单片机说了算」。
- 新增动作步骤按「脚本动作表设计规则」三条走：`app_script.h` 枚举 + 参数映射 DSL 注释、`app_script.c` 的 `case`（**case 体第一个函数调用 = 控制台原语名**）、`map.c` 行尾 `//` 注释。积木表自动派生，无需改 `tools/webconsole/server.py`
- 组合动作（发指令 + 等结果/动作 + 失败音）封成一个原语，对应**一步** `STEP_XXX_WAIT` —— 参见 `ucRpiQrScan()` / `ucRpiHostageScan()`，勿在 map.c 里连写两行
  - ⚠️ 新增这类「发起任务 + 等结果」的组合动作时，**发起帧用 `ucRpiSendTaskCmdWait`**（等链路重连，被顶掉/被中止返回 0），**别用 `vRpiSendTaskCmd`**（照发不等链路，断了就是白丢 + 空等超时）；**收尾帧**（停流/停止类）相反，必须用它。理由见「向 Pi 发任务」。
- 巡线那一步同理：`ucRpiLineConverge()`（`app_rpi.c`）内部发 `RPI_TASK_LINE_START` → 等巡线帧计数变化（**不能用 valid 判起流**：丢线时 valid=0 但帧在流通）→ `vLineFollowConverge()` → 发 `LINE_STOP`。Pi 未起流则拒绝且不动车（不响, 留痕; 旧版拿陈旧快照空跑满 converge_ms，表现为"车不动也不报错"）。`vLineFollowConverge()` 是内层原语、**不自己发指令**
- ⚠️ **握手成功后再执行一次「等待握手」= 判为断开、重新握手**（`ucRpiHandshake`）：本次上电**走到过阶段 2**（`s_bHsDone`）后再进来 → 状态落回 `RPI_LINK_WAITING`、清 `ucTakeHandshakeSeen`，重走阶段 1 + 静默确认窗。① 树莓派须重发 `SYNC_HS_1`；② 阶段 1 重拉 `bt_hs_wait` → **蓝牙机械臂编辑模式又能进**；③ 模拟器与真 Pi 同路。首次握手后再排一步握手即此语义。
- ⚠️ **阶段 2 静默确认窗内按 MAIN 可提前放行**（2026-10-05 加）：Pi 侧「握手成功」（它收到 ACK）只是 MCU 阶段 2 的**开始**，窗口固定 3s 走满才放行 —— 路线表头「等握手 → … → 按主按键开始」的流程里，这就是「握手成功后要等一会才能按 MAIN 启动」的来源。现在窗口循环里查 `ucTakeMenuConfirm`：**吃掉再原样放回**（既结束窗口、又不丢这一按 —— 后面 `STEP_DEAD`/`STEP_STOP` 的等键还要消费它；阶段 1 里按下的 MAIN 走同一条路生效，路线在链路刚通时就放行）。安全性：ACK 本就由 `vRpiTask` 收到 `SYNC_HS_1` 自动回、对端重发自动补答，提前放行最坏只是「ACK 曾丢 → 2.5s 后重发 → 又被补答」，链路自愈；菜单触发的「握手让出」里提前放行也不改变按键生命周期（修复前它同样只是缓冲到下一个等键步骤）。
  ⚠️ 判据是 `s_bHsDone` 而**不是**「链路是 OK」：`vRpiTask` 是独立任务，**对端先上电**时会提前应答并置 `RPI_LINK_OK`（常见开机顺序）—— 那是「对端先握好了」，必须照旧放行；按链路状态判则每次开机都可能卡死等一个再也不会来的握手请求。
  ⚠️ 落回 `WAITING` 而非 `DISCONNECTED`：`vRpiTask` 有「Disconnected 态收到有效帧即恢复 OK」的规则，下一心跳（2s）就翻回 OK，而 `vBtEditPump` 见 OK 立刻把 PC 赶出编辑模式——让位窗就闩不上。
  ⚠️ 本端**不主动请求**对端重发（Pi / 模拟器只在开机阶段发 `SYNC_HS_1`）：要等它重启主程序或重新上电。等待期 OLED 显示 `RPI:Waiting` + `BT:HS WAIT`（不蜂鸣 —— 2026-10-23 全固件撤声后未恢复）。
  ⚠️ 故 `STEP_SCRIPT_RESET` **必须**继续跳过表首那一步（「脚本动作表设计规则」第 6 条）；表面排的握手照旧会等。
- **双闭环**：位置环（帧里的 `steer` → 麦轮 `vx`，环在 Pi 端 LinePID）+ **航向环**（环在固件，2026-09 定案：**一刀切选一个源**，**不融合、不滤波**）。两条环的"目标值"分居两端：横向目标在 Pi（`LINE_SETPOINT`，控制律在 Pi），航向目标在固件 —— 别在一边找另一边。

  **听谁由全局调参宏 `LINE_CONV_SRC` 决定**（2026-10-30 起不再是积木参数；常量 `LINE_ANG_SRC_*` 在 `app_rpi.h`）：

  | `src` | 误差 | 增益 / 死区 | 该源不可用时 |
  |---|---|---|---|
  | `0 相机`（**默认**） | `图像角 − line_angle_target` | `LINE_CAM_TO_W` / `LINE_CAM_DEADBAND_DEG` | 丢线（`valid=0`）→ **这一拍不修航向** |
  | `1 陀螺` | `短弧(line_gyro_target − yaw)` | `LINE_GYRO_TO_W` / `LINE_GYRO_DEADBAND_DEG` | IMU 停摆 → **这一拍不修航向** |
  | `2 不修` | —— | —— | ——（只修位置；过雪弗板/交界被遮挡时用） |
  | `3 融合`（**出厂**） | 两条各自算完自己的环 | 各自一套 | 该源不可用 → 本拍出 0 力，权重**不补给**另一条 |

  - ⚠️ **不叠加、不互相降级**：选中的源不可用 = 这一拍没有航向修正，位置环照旧听 Pi 的 `steer`。**丢线多的段落只能把全局 `LINE_CONV_SRC` 改 1(陀螺) 重编译** —— "一刀切"唯一的代价。（旧"丢线当拍切 100% 陀螺 / IMU 停摆退回纯相机"随融合一起删了。）
  - ⚠️ **两条量纲不同**（图像角 vs 真角度：地面线倾角被透视压缩，差约 3.3 倍）⇒ 增益与死区**各一套**、符号各自独立（折在各自增益里，**负值 = 反向**），数值别互抄。
  - ⚠️ **陀螺那条是"保持航向"不是"跟线"**：目标是**绝对世界航向**（零点 = 开机朝向，同 `STEP_SPIN_TURN_WAIT` 的 `absolute=1`），线与车头不一致时车**斜着平移**（位置环修横向、航向咬在目标角）。要沿线拐弯只能用相机那条，或按段改目标角。
  - ⚠️ **IMU 停摆用姿态帧号判**（`ulGetYawFrameCount()` 停涨，窗口 `LINE_GYRO_STALE_MS`(600ms) ≥ ATK 驱动一次阻塞读的 500ms）。**别用「yaw 有没有变」**：分不清"陀螺停摆"与"车/电机没动"（低出力电机死区、打滑都让 yaw 不变），会掐掉**正在生效**的环。也别借 `ucTakeYawFresh()`（读后清零的一次性信号，`app_pid.c` 三处在用）。⚠️ **别把 `LINE_GYRO_STALE_MS` 设 0**：那是这道保护的总闸，关掉后 yaw 一冻住误差恒定 → 车按常数 `w` 一直转（没有别的源可退）。
  - ⚠️⚠️ **删除记录（2026-09 定案，别再改回去）**：原为两条误差**加权平均**（`line_gyro_w` %，`line_gyro_scale` 换算图像角尺度）。该路有**能翻符号的闭环增益** —— 尺的符号标反时两条从"相加"变"相减"，系数 `k − w(k+|s|)` 过零后**负反馈变正反馈**：车顶在 `LINE_W_MAX` 上**朝一个方向无限转**（实测权重 >50% 就犯，该宏出厂 `1.0`、从没标定过）。一并删的还有 `LINE_GYRO_W_PERCENT` / `LINE_GYRO_SCALE` / `LINE_W_SIGN`、`line_gyro_w` / `line_gyro_scale`、`use_fusion` + 互补滤波（`fEst`/`LINE_FUSION_COAST_MS`）、`STEP_LINE_ANGLE_SET` 积木、`line_yaw_target`/`line_yaw_w`/`LINE_YAW_ALIGN_DEG`。**要"既不漂又贴线"的折中，用全局 `LINE_CONV_SRC`（相机/融合），不要回到加权；分段换源只能改宏重编译。** **删滤波代价**：相机那条用**原始相机角**（不再平滑），角抖时航向环更抖（`Spd` 第三个数字高频跳）—— 处方是调小 `LINE_CAM_TO_W`，不加回滤波。
  - **调参通路（2026-10-06 起全套两层）**：蓝牙 Tab「巡线」组当场可改的不止 `line_gyro_target` / `line_angle_target` / `line_time_scale` —— 增益/死区/限幅/方向也已做成 Beam 运行时量（`line_steer_to_vx` / `line_walk_steer_to_vx` / `line_vx_sign` / `line_vx_max` / `line_cam_to_w` / `line_walk_cam_to_w` / `line_cam_db_deg` / `line_gyro_to_w` / `line_walk_gyro_to_w` / `line_gyro_db_deg` / `line_w_max`，共 11 条）：零重编译，`vLineFollowConverge` 每拍现取（20ms 生效），标好把终值写回宏固化。刻意**不进**蓝牙的：保护类窗口量（`LINE_STALE_TIMEOUT_MS` / `LINE_GYRO_STALE_MS` / `LINE_FIRST_FRAME_TIMEOUT_MS`，安全阀不给无线一键关掉的通路）与收敛判据（`LINE_CONVERGE_*` / `LINE_CE_*`，有"阈值 ≥ 死区"的跨参约束）——这些仍走「调参」Tab（要重编译）。OLED 第 1 行 `ATK-Yaw` + 第 4 行 `Spd:...,w` 就是要看的量；步末自述串 **`LINE: src=.. a=.. err=.. w=.. na=..`**（`a` = 该源读数，相机=图像角/陀螺=yaw；`na` = 本步"无可用源"的拍数，一大就说明几乎没在修航向）—— 排障第一手证据，别删。`src=?` = 全局 `LINE_CONV_SRC` 越界（0~3）。
  - ⚠️ **上机前先确认 IMU 的 yaw 零点真是"开机朝向"**（ATK-MS901M 九轴模块，驱动只读满量程、未做零偏标定）：上电读 OLED `ATK-Yaw` → 断电 → 原地转 90° → 再上电读。若≈0 则整套设计成立（世界系与绝对转弯也建立在此前提上）；若≈90 或别的值，说明模块用了磁参考，`gyro_target` 含义要按实测重解。
  - ⚠️ 陀螺目标角的"绝对"**仅相对开机朝向**：中途转过向的段落要按 OLED 读到的 `ATK-Yaw` 填，别按"我转了 90°"推 —— 弧线/打滑会让路线以为的航向与 IMU 读数差几度。
  - ⚠️ `±180` 是同一朝向、从别处过来时往哪边转**不可复现** —— **指定方向的掉头**请用 `STEP_SPIN_TURN_WAIT`（`dir` 明说方向）。
  - 两环正交互补：`vx` 管横向、`w` 管斜向，一次 `vChassisMove(vx,0,w)` 由麦轮解算叠加（`speed[] = ±vx ±vy ∓w`）。⚠️ 航向环**纯 P、无微分**（相机角分辨率 0.1° 且噪声大，故留死区）—— 摆振只能靠调小**那条源自己的**增益（`LINE_CAM_TO_W` / `LINE_GYRO_TO_W`），调大只会更抖。丢线时 Pi 把 `angle` 也送 0，否则固件会拿陈旧倾角一直转车 —— 固件侧还有 `valid` 那道闸兜着（见上表）。
  ⚠️⚠️ **相机端 angle 的可信度要单独确认，开航向环前先确认它稳**：早期相机脚本用 `b.rotation`（整块区域图像矩主轴角），检测区域是**一大片路面**（接近各向同性）时是病态量 —— **角度时而正时而负、且与远近有关**（路面近时铺满画面，主轴被裁切钉死 ≈0 = 无航向信息）。正解是**直接量交界线斜率**（两侧各找一边界行，`atan2(Δy, Δx)`），见 OpenMV `E:\main.py`（相机盘）—— 固定测量区 + 逐列色心中点穿越那版**已上机验证过**（实测重复性 σ≈0.11°），先确认车上烧的是那一版。
  - ⚠️ 一刀切之后**没有"调权重绕开它"这条退路了**：相机角不可信就把全局 `LINE_CONV_SRC` 改成 **1(陀螺)**（不看相机角，代价是只保航向、不跟线）或 **2(不修)**（只修位置）重编译。不健康的那条源不该被悄悄混进来。
- **丢线的处理在 Pi，不在固件**：`vLineFollowConverge()` 在 `valid=0` 时**照用 Pi 给的 steer** —— 「保持 / 找线 / 停」全由 Pi 决定。**别再改回 "丢线就停车"**：丢线率一高（实测 30Hz 里六成帧丢线）会「横移↔停车」反复跳，车几乎不前进。收敛判据也只在 `valid=1` 时累加（丢线时 Pi 恒发 err=0，否则被骗成已收敛）。
  - 2026-09-26 起 Pi 按 **OpenMV 帧第 5 字段 `state`**（丢线方向分类）**定向找线**，取代原来的死写方向（车偏进路里时它**越找越偏**）：`state=2`（交界远出画面上方 = 车偏进路里）→ 向边界横移；`state=3`（交界压到画面下方 = 骑在边界/黄区上）→ 向路内横移；`state=0/1` 是正常/曲线帧（valid=1）。
  - ⚠️ **找线方向不是标定量**：Pi 从 `LINE_ERR_SIGN`/`LINE_SETPOINT` 那条 err→steer 链**推导**（Pi 侧 `_toward_steer()`：PID 在 offset 最负处的输出方向正是把 offset 推回来的方向），改这两个标定量时方向自动跟着走 —— 本端 `LINE_VX_SIGN` 翻符号也一并被继承。找线的**大小**才取 `|LINE_SEARCH_STEER|`；`LINE_LOST_CONFIRM_S`(0.4s) 是躲单帧毛刺的静默期，`LINE_LOST_HOLD_S + LINE_LOST_SEARCH_S` 是两路共用的总预算，超了停。
  - ⚠️ **`LINE_SEARCH_STEER` 身兼两职（2026-10-05 起）**：丢线找线速度 + **valid=1 满偏/钉边救援速度**。救援触发 = `|err|≥1` **或** offset 钉死 ±100 且 err 指向回程 —— 后者是给「SETPOINT 不在画面中央」补的：SETPOINT=-65 时灰侧到满量程只剩 35 个 offset 单位，err 上限 0.35 → steer 上限 0.246 ≈ 7rpm，车偏得再远误差也不涨（日志 50 帧全钉 +0.246），这就是「**黄区修正快、灰区修正慢**」的根因；钉边触发后灰侧同样满幅 1.0。⚠️ 实际值以 `config_override.json` 为准（Web「设置参数」写的运行时覆盖层，gks 启动时 `load_overrides()` 加载，config.py 里那份是出厂底值**会被盖掉**——排障先读 override 再下结论）。
  - `state=4`（没采到参考色、点和颜色都说不清）**或旧 OpenMV 固件**（无第 5 字段，解析缺省 0）才走旧的「保持 → 固定方向 `LINE_SEARCH_STEER` → 停」，此时那个**符号**才还参与（纯兜底猜测）。
  - ⚠️ **本端一行都不用改**：帧的 err/steer/angle/valid 语义一字未变，`state` 是 Pi 内部信息（不下发给单片机）。要确认车上烧的是本版（`valid=0` 照常执行 steer，见 `app_line.c` 的 `fVx = fSteer * LINE_STEER_TO_VX * LINE_VX_SIGN`）：步末调试串有 `LINE:` 即为新版（2026-09 之前叫 `LINE mix:`，改名是因为内容从"两条误差对照"换成"这一拍在用哪个源"）。
  ⚠️ 代价是单片机完全听 Pi，故 `LINE_STALE_TIMEOUT_MS` 那道**帧号新鲜度**保护是唯一兜底（帧号停涨 = Pi 掉线/停发 → 停车）—— **别删**。它还顺带解掉了"Pi 到自己的 `LINE_MAX_DURATION` 就停发、单片机拿最后一帧继续跑"的跑飞风险。
  ⚠️ **「每局开场车必猛地朝一个方向横移一下」曾来自 Pi 侧的启动盲期**（2026-09 已修在 Pi：`_line.py` 在拿到**第一帧新鲜数据**前一律 `steer=0`）。机理：OpenMV 按需发送（收 `'G'` 才发），刚起线程时 Pi 缓存里是**上一次运行**的快照 → 旧版当丢线 → 下发固定方向满幅 `LINE_SEARCH_STEER` → 本端 `valid=0` 也照做 → 车横移（日志里每次「[巡线] … ⚠数据过期」紧跟的那一下就是它）。**别在本端加"开局头几帧不动"的补丁**：本端分不清"Pi 在报陈帧"与"真丢线"，那是 Pi 的职责。
- **现场标定看哪**: OLED 第 4 行（y=48）平时显示 `Spd:vx,vy,w` —— 最后一次 `vChassisMove` 的**指令值**（Beam `vGetChassisSpdCmd` 发布；巡线时第一个数 = 乘完 `LINE_STEER_TO_VX`/`LINE_VX_SIGN` 并限幅后的最终横移量；丢线时是 Pi 策略给的值(可能非 0)，停车/掉线才 0）。只覆盖**速度模式**；位置模式（`vChassisMoveWorld*`/弧线/色球对位）不发速度，值停在上一次。蓝牙机械臂编辑模式时这一行让位给 `BT:HS WAIT/ARM EDIT`

**新增一种二进制结果帧（COORD 子命令）：**

- 挑一个 `< 0x20` 的 TAG（**0x01 巡线 / 0x02 人质 / 0x03 色球 / 0x04 靶标 / 0x05 靶标流 / 0x06 拍照 / 0x07 矩形地板 / 0x08 看路 / 0x09 看路校准 / 0x0A Pi 参数 / 0x0B 爆爆桶流 / 0x0C 蜂鸣命令 / 0x0D 相机健康检查** 已占用；ASCII 帧首字节 ≥ `'0'`，天然不冲突。`RPI_TYPE_QR = 0x03u` 是 **TYPE** 命名空间，与 COORD 的 DATA 首字节 TAG 无关，同名不冲突但别误读）
- 在 `app_rpi.c::vRpiTask` 的 COORD 分发里加分支，**必须放在通用 `LEN >= 9` 分支之前** —— 否则整帧会被 `vSetLastRpiPacket` 当「9 字节 last packet」吞掉，永远到不了新解析逻辑
- 定长二进制 + **小端**（`p[1] | (p[2] << 8)`），与巡线帧同约定；多字结果经 Beam 临界区快照发布（同帧强一致，见 Beam 规则第 4 条），**不要**塞进 last packet
- 已实装：`0x01` 巡线（`vSetLineData`）、`0x02` 人质（`vSetHostageResult`，§6c）、`0x03` 色球（`vSetColorResult`，§6d；2026-09 起成功帧 LEN 18→21，尾追加 `[yaw i16 LE 0.01°][fit_ok u8]` → Beam `fGetBallYawLine`/`ucGetBallFitOk`（蓝牙遥测 `ball_yaw`/`ball_fit_ok`）。⚠️ yaw 是**图像角不是车体角**：车原地转 θ 图像只斜约 (h/Z)·θ（比例待标定，预期 ~0.3），落地须乘标定系数；fit_ok=0 = 共线残差超限（三球共线⇒透视保直线⇒残差大=至少一个质心认错）。⚠️ **2026-10-01 起 fit_ok 只是诊断量、不影响执行**：`ucBallScanOnce` 原先要求 `fit_ok=1`（NG → 重扫一次 → 仍 NG 就长鸣 + 一步不动、**连平移也不做**），现只看 `status=1 && count=3`，NG 时连线角照用、平移照做（"宁可动错也不拒动"，2026-10-01 用户定）；查可疑帧看 Pi 日志 `共线NG` 或遥测 `ball_fit_ok`。旧 Pi 的 LEN=18 → 兜底 yaw=0/fit_ok=1）、`0x04` 靶标（`vSetTargetResult`，§6e）、`0x05` 靶标**流**帧（`vSetAimFrame` + `ulGetAimFrameCount`，§6f 与「标靶流控闭环」）。⚠️ **`0x05` 与 `0x04` 逐字节同布局**，只是 ~20Hz 连续流；两者**刻意各用独立快照**、别合并（否则闭环停流留下的最后一帧会污染随后的终验）。⚠️ **`0x04` 每条记录 7 字节**（多了 `r` 外圆等效半径），别照抄色球的 5 字节 —— `RPI_TARGET_TGT_LEN` 与 Pi 端 `_target.build_target_frame` 的 `struct.pack` 必须逐字节对齐；`0x09` 看路校准（`vSetRoadYawResult`，8B 定长、**刻意取 8 而不是 9**：9 会落进通用 last-packet 分支）；`0x0A` **Pi 参数帧**（**ASCII 文本，不是定长二进制** —— 见下一条）；`0x0B` 爆爆桶流帧（`vSetBucketFrame`）；`0x0C` **蜂鸣命令帧**（Pi 控制蜂鸣器, 2B, 见文末「蜂鸣命令」节）。
- **`0x0A` Pi 参数帧 / 命令 `"P"` 族（2026-10 加）**：让**单片机现场改树莓派 `config.py` 里的参数**（协议细节见 `docs/串口协议.md` §6k）。
  - `"P?,<名>"` 取参 → `V,<名>,<类型>,<值>[,<选项表>]`；`"P,<名>,<值>"` **临时**设参 → `OK,…` / `ERR,<名>,<原因>`。⚠️ 临时 = **只改 Pi 主程序进程内存**（不写 `config_override.json`）：立刻生效、**Pi 重启还原**；要固化去 GKS_Web 的「参数」页。
  - 本端实现在 `app_rpi.c` 末尾的「Pi 参数缓存」段（`vRpiCfgPoll` / `ucRpiCfgHandleReply` / `vRpiCfgSet` …）。三条纪律：**只查"想要"的项**（`vRpiCfgWant`，OLED 画某一行时标它 —— 一百多项全查会把链路刷爆）、**一次只有一条在飞**（Pi 侧任务是串行的）、**绝不置 ready / 不碰 last packet**（它是旁路回执，对"等结果"的窗口隐形；分支排在通用 `LEN >= 9` 之前）。
  - ⚠️ **查询是按需的（2026-10 改，别改回定时轮询）**：只查「没值」的行 + ERR 重试（3s 间隔、封顶 3 次）+ 链路重连那一拍全表作废重查 —— **不再按时间自动刷新**。原 3s 新鲜期让调参屏一挂就每秒两条 `"P?"`（Pi 日志 `收到指令: b'P?,…'` 刷屏的根源）。代价：Pi 侧/网页改的值要等重查触发才上屏（OLED 改值走 `"P,<名>,<值>"`、Pi 回 `OK` 会顺带刷新缓存）。同 TAG 的 `"QS,…"`（「自动下发二维码」回执，Pi 协议文档 §6l）在 `ucRpiCfgHandleReply` **之前**按前缀分流。
  - 消费者的名字由**两张表**回答：本机 Beam 表（`cpstFindBeamParam`，现取现写）+ Pi 参数表（`cpstRpiCfgFind`，**由 `tools/gen_rpi_cfg.py` 从 Pi 的 `config.py` 生成进 `app_rpi.c` 的生成块** —— 改了 Pi 侧可编辑参数要重跑它）。OLED 那侧是动作码 `OLED_ACT_TUNE_PARAM`（块名「名字调参」），行的第四字段存**参数名**（下标会随表重排漂，名字不会）。
  - 控制台（webconsole「OLED」Tab）的「参数」弹窗（`oled_menu.py::param_catalog()` 现扫上面两张表）就是选名字的地方；保存校验会拦住"名字不存在"的死项。**改 `app_beam.c` / `app_rpi.c` 的生成块后刷新页面即可见新名字**。
- **小球对位 `STEP_BALL_ALIGN_WAIT` → `ucRpiBallAlign`**（app_rpi.c）：发 "5" →（`BALL_ALIGN_YAW_EN` 开时）图像连线角 × `BALL_YAW_SCALE`（≈Z/h≈3.3，**≠1**，图像角≠车体角）经 `vPidSpinTurn` 转正 → **复测一次**再算平移（视差已含在复测坐标里，故**不需要** `BALL_ROT_CX_PER_DEG` 那类常数）→ 中球偏差横移/纵移（`vChassisMoveCameraLateral` + 对偶 `vChassisMoveCameraForward`）。基准=**中球**（同 ARM_ACT_3/4/5 录制位姿）；识别不全失败音放弃（对位链内被压制, 归链统一管）绝不动车。**标定量（含 `ball_yaw_en`）全是 Beam 运行时量**：蓝牙 Tab「小球对位」组当场可改（零重编译，断电回 `app_config.h` 的 BALL_* 上电默认）。`ball_yaw_target`（宏 `BALL_YAW_TARGET_DEG`）= 理想停止位下三球连线角的读数（即 `ball_yaw`）：转正按 `(实测 − 本值)` 而非死按 0°（相机装歪/地面不平/连线不平行车体横轴都会让理想位姿下它非 0）；标定=摆位跑一轮读 `ball_yaw` 填入。`ball_xy_swap`（宏 `BALL_XY_SWAP`）= **横/纵两路「轴+增益」整套对调**（相机相对车体装 90°）：横移改由 cy 驱动、纵移改由 cx 驱动，两路 mm/px 增益跟着换；**方向符号仍各走 `BALL_SIDE_SIGN` / `BALL_FWD_SIGN`**，对调后可能也要翻。预设 9「测试小球对位」= 每按 MAIN 一轮，流程写在 map.c 该预设注释里。过程往 **Pi 日志**打 `[STM-DBG]`（`vRpiSendDebug`）：`BALL meas:` / `BALL turn:` / `BALL fix: dx=..mm(R/L/skip) dy=..mm(F/B/skip)` / `BALL fail:`；浮点走 `app_bt.c::bt_fmt_f2`（已公开，**全工程不用 %f**）。⚠️ 对位失败后脚本照常走下一步（执行器无失败分支）。
- **人质对位 `STEP_HOSTAGE_ALIGN_WAIT` → `ucRpiHostageAlign`**（app_rpi.c，2026-09）：与小球对位**同构**，识别对象换成 `0x02` 人质帧 —— 发 "4" → **三只齐才算数**（`status==1 && count==3`）→ 取**积木参数选定的那一只**（`idx`：0=左/1=中/2=右，下拉自动派生）作基准 → 它的 `cx/cy` 偏差 → 横移/纵移（`vChassisMoveCameraLateral`/`Forward`）。三处**有意**不同，别"顺手"改成和小球一样：
  - **基准由参数给**（小球固定取中球，基准绑死在 ARM_ACT_3/4/5 录制位姿上）。
  - ⚠️ **不做转正**。`0x02` 帧**没有** yaw/fit_ok 扩展字段，而圆柱/圆锥/腰鼓的**质心高度本来就不同**（同排腰鼓比圆锥高），拿三个质心算连线角量到的一半是形状差不是车头歪。要做必须先在 Pi 端给 `0x02` 帧加扩展字段（照抄色球帧那套），**不是本端能单方面完成的**。
  - **标定量独立一套 `HOSTAGE_*`**（人质区停止位姿与球区不同，共用 `BALL_*` 会打架）；多一个 `hs_side_sign`（横向方向），小球那边借 `BALL_SIDE_SIGN`。全做成 Beam 运行时量：蓝牙 Tab「人质对位」组（`hs_cx_center`/`hs_cy_center`/`hs_px_to_mm_x`/`hs_px_to_mm_y`/`hs_dx_db_mm`/`hs_dy_db_mm`/`hs_side_sign`/`hs_fwd_sign`/`hs_xy_swap`/`hs_accel`/`hs_speed`），断电回宏的上电默认。⚠️ `hs_xy_swap` 语义与初值跟 `BALL_XY_SWAP` 对齐（与区域无关），但**两边增益/死区/方向符号各自独立**。
  - `idx` 越界（手写 map.c 或老参数）→ 拒绝 + **不动车**（不响, 留痕），**不**静默夹到 0/2（夹了就是「选的是左、对到的是右」）。识别不全/超时 → 失败音（等待函数投）。
  - ⚠️ **「三只齐」这条判据与 Pi 的 `HOSTAGE_MIN_TARGETS` 绑定**：Pi 正常配置下「报成功」等价于 count=3（`_hostage.py` 凑满 MIN 才成功、回传上限 MAX 也是 3），故本判据不误伤；但 **MIN 在 Pi 的 Web 控制端是可调项**，被调到 <3 时 Pi 会以 count<3 报成功而本步一律拒绝（不响, 留痕） —— 现场现象正是「Pi 说识别成功、单片机不动」，别在这边打补丁，回去把参数改回 3。
  - Pi 日志前缀 `HSTG`（小球是 `BALL`）：`HSTG meas: #1(926,429) shp=2 n=3` / `HSTG fix: #1 dx=..mm(R/L/skip) dy=..mm(F/B/skip)` / `HSTG fail:`。
  - ⚠️ **它不碰机械臂**，故**不进 `c_bIsArmStep`**（与标靶校准那种会动舵机的步骤不同）。
  - ⚠️⚠️ **超时值必须按 Pi 侧算，别照抄小球那套的 6000**：人质检测是**主摄 YOLOv8 ONNX 跑 CPU**（~1.5s/帧，凑满 3 个实测 ~5s），色球是 HSV 掩膜（亚秒级）—— 同一个 6000 对前者根本不够。现场日志：Pi 打印 `人质识别成功（从左到右）: 圆柱(368,889) 腰鼓(785,859) 圆锥(1139,840)` 的**同一刻**，本端打 `HSTG fail: wait(t=6000ms) st=0 cnt=0` + `HSTG fail: scan`，紧接着 `COORD tag=2 len=18` —— 结果帧迟到几百毫秒，正好卡在超时外。下限 = Pi 的 `HOSTAGE_TIMEOUT`（`config.py`，默认 **6.0s**，**Web「设置参数」可改**）+ 摄像头打开（`_open_hostage_with_retry`，实测 ~1s，Web 推流时更久）≈ **7.1s** → 取 **7500**；上限 = `RPI_DISCONNECT_TIMEOUT_MS`(8s)（本命令**阻塞**，期间 Pi 不发心跳，超了本端会闪一下 DISCONNECTED，对结果无害、但会关掉蓝牙臂编辑窗）。两个数是**跨仓库耦合**的一对：Pi 那个被调大，这边必须跟着抬。
  - ⚠️ **三种形状的质心 cy 天生不同，别拿它当"高度"读**：实测同一排 `圆柱 889 / 腰鼓 859 / 圆锥 840`，**相差最多 49px ≈ 20mm**（按 `hs_px_to_mm_y` 换算）—— 这正是"不做转正"的依据，也意味着 `hs_cy_center` 只能按**你要对的那一只的形状**去标；混着标会让纵向稳定偏出去二十来毫米。`hs_cx_center` 没这个问题（横向不受形状影响）。


## 相机健康检查（`"Z"` / 0x0D，2026-11 加）

握手积木（`STEP_WAIT_HANDSHAKE`）新增 `health=健康检查{1=是,0=否}` 勾选框（老路线缺槽补 0 = 不检查，行为不变）。勾选后，**握手成功**（走完静默确认窗）立刻调用 `ucRpiHealthCheck()`：

- 发起帧用 `ucRpiSendTaskCmdWait(RPI_TASK_HEALTH, RPI_HEALTH_TIMEOUT_MS)` —— 等链路、可被中止/顶掉，别改用 `vRpiSendTaskCmd`。
- Pi 侧 `handler.py::_cmd_health`（`"Z,<ms>"`，在 `_MS_COMMANDS` / 阻塞元组 / `_FAIL_FRAME_SENDERS` 四处同挂）依次确认四路：**M 主摄 / Q 扫码副摄 / Pi CSI / MV OpenMV**，各读一帧；回 `TYPE_COORD` + `TAG=0x0D`，`DATA=[0x0D][M][Q][Pi][MV]`（5B，1=通过 0=失败，见 `docs/串口协议.md` §6o）。
- **阻塞**：`ucRpiHealthCheck` 在脚本任务里等结果，检查期间脚本不会往下走 —— 这就是「不能突然发车」的结构性保证。等待循环查 `ucGetScriptAbortReq()`；中止时四项保持"未检查"。Pi 在每台相机之间补一帧 `TYPE_HELLO`，正常不会触发 8s 的 `RPI:Disconnected`（单台超过 8s 才可能闪一下，结果帧一到即自愈）。
- **解除占用**：Pi 每查完一台就释放（USB 租约 `LEASE.release_now()`、CSI `PICAM.release_now()`、OpenMV 补 `'S'`，外层 `finally` 再兜一遍）。`release_camera()` 只是"归还保持打开"，**不能用它代替**。⚠️ 因此健康检查应排在「预热摄像头」积木**之前**（握手本来就在表头）：排在预热之后会把预热窗口一起收掉，那次预热白做。
- **显示**：OLED 第 1 页第 4 行在**秒表首次启动前**显示 `H:M√Q√Pi√MV√`（失败 ×、检查中/未检查 ?），首次启动后永久让位给 `Time:...`。检查中为 `H:M?Q?Pi?MV?`。
- **IF 条件**（瞬时快照，不重新检查）：`SCRIPT_COND_HEALTH_FAIL`（任一项失败）、`_M_FAIL` / `_Q_FAIL` / `_PI_FAIL` / `_MV_FAIL`。没勾/没检查时四项为"未检查"，所有 HEALTH 条件都不成立 = 路线照常继续；读取并处理失败项由路线作者用「如果」自己排。
- **失败处理**：不蜂鸣、不自动中止（用户定）；链路不通/超时按四项全失败写 Beam（显示 ×），并在 Pi 日志/调试串留痕（`HEALTH fail: ...` / `HEALTH: M=.. Q=..`）。



**问题**：`vRpiSendTaskCmd` 原先**完全不看链路状态** —— 链路断了（Pi 掉线/没上电）也照发（帧进 TX 环、无人接收），上层唯一感知是随后 `ucRpiWaitXxx` 超时返回 0。

**做法**：新增**唯一会等链路的入口** `ucRpiSendTaskCmdWait(emTask, ms)`（`app_rpi.c`）—— 链路不是 `OK` 就把任务挂进**单槽**等重连（后写者胜、不排队）；超 `RPI_LINK_RETRY_TIMEOUT_MS` 就发一对 `"r"` 让 Pi 重启客户端，**往复不封顶**（只靠中止位退出）。

- **判据是「链路 != OK」，`WAITING`(0) 也等**。`DISCONNECTED` 只能从 `OK` 进入，心跳救不了 `WAITING`（只有 Pi 重发握手才行）—— 发 `"r"` 重启客户端正是唯一能救回它的手段。
- ⚠️ **等待重连的时间不计入积木自己的 `I(0)`**：该函数在 `ucRpiWaitXxx` 之**前**返回，所以积木超时天然只覆盖「帧真正发出去之后」那一段 —— 一行代码都不用写。
- ✅ **发送由「等待者自己」做，不由 `vRpiTask` 代发**：跨任务交代"发出去了没有"会被 10ms 内的新投递覆盖 → 把**已发出**的任务误判成"被顶掉"、结果帧落进**下一步**、同类步骤整体错位一格且**全程无报错**；`vRpiTask` 也**一行没改**（有 `vTaskDelay` 就是真丢帧）。
- ⚠️ **只有「发起任务且要等结果」的原语用阻塞版**（11 处：二维码 / 人质 / 色球 / 靶标 / 矩形 / 小球对位链、巡线起流、靶标流起流）。**`LINE_STOP` / `AIM_STOP` 与「只发不等」的那几条保持原样**：
  - **收尾帧既不等链路也不顶代次** —— 它们最需要发出去的时刻恰是链路不可靠时（起流失败的兜底路径），一挂起就会让 Pi 一直跑着 30Hz 流。
  - 只发不等的（OLED 拍照 `"9"` / 关机 `"sd"`、`STEP_RPI_SEND_TASK`、`STEP_RPI_WARM_CAM` 预热摄像头、`"C"` 预热主摄）**照发、但会顶掉**挂起的那个（「以新的任务为准」）；**刻意不挂起**：否则链路恢复后会在几分钟后突然开火（几分钟前按的"拍照"、路线早过了那一步却发出一条起巡线流），预热窗口也早过了 —— 迟到的预热只会白占相机。

### ⚠️ 代价与纪律（别当成 bug）

1. ✅ **Pi 侧那道门已读源码核实**（`\\192.168.<网段>.125\shared\Desktop\GKS_Competition`）：`handler.py::_require_double_send` 的门状态是 `_system_cmd_arm` / `_system_cmd_last` 两个**以 payload 为键**的 dict —— `"sd"`/`"rb"`/`"r"` **各记一份**，确认窗与冷却按指令分开（`SYSTEM_CMD_CONFIRM_S` 默认 10s）→ `"r"` **确实也要连发两次**（`handler.py:795` 同样门控 → 发一对**必需**）；「两条指令在确认门上凑单」**不可能发生** → `RPI_SYSCMD_SUPPRESS_MS` 那道互锁当前是**冗余护栏**。
2. ⚠️⚠️ **`vRpiRestartPiClient` 里「第二帧之前重查链路」那条判断不能删。** Pi 收到 `"r"` 时**关串口 → `time.sleep(20)` → `os.execl`**（`serial_comm.py::restart_program`，**阻塞在 handler 主循环里**），重启完成的**那一刻**它就发 `SYNC_HS_1`（`handler.py::_handshake` 是 `run()` 第一步）—— 本端**几毫秒内**就把链路置回 `OK`，故两帧之间那 500ms 里链路很可能已经好了：不重查就照发第二帧 → Pi **再确认一次** → 20s 后又重启一遍、**把刚发出去的任务掐掉**，全程无报错。只发第一帧安全（Pi 只是"登记"，10s 窗口过期即作废）。
3. ⚠️ **一次成功恢复要 20s+，不是几秒** —— 那 20s 是 Pi 侧为「等机械臂/底盘停稳」故意等的。期间**端口是关的**，本端每 6s 发的那一对 `"r"` 全部**丢在线上** → `rpi_link_retry` 涨得比真实重启次数快得多（一轮成功恢复约 +8 帧）。**判「成功了没有」看 `rpi_rx_bytes` 恢复 / `rpi_link` 变 1，别看 `rpi_link_retry` 涨多快。**
3. ⚠️ **代价照实说**：等待期间操作员按了 OLED 的「拍照」/「关机」→ 脚本正在等的那个任务**作废**（脚本返回 0, 不响留痕），而那条只发不等的指令在链路不通时白丢。这是「Pi 永远只收到当下真的想要的任务」换来的。
4. ⚠️ `ucGetBtHsWait()` 那道守卫（**让位窗里绝不自动重启 Pi**）**不能删** —— 台架调臂时链路被**故意**钉在 `WAITING`（见 `vBtArmEditHold`），没有它就是"调一次臂重启一次 Pi"。
5. ⚠️ **`RPI_LINK_RETRY_TIMEOUT_MS` 与 `RPI_DISCONNECT_TIMEOUT_MS` 没有大小关系** —— 计时从**投递任务那一刻**起算；硬下界 `>= RPI_HANDSHAKE_CONFIRM_MS`（小于它会在 Pi 正走握手静默窗时又打断它 → **自激成无限重启**）。
6. ⚠️ **`"r"` 的两帧必须分开发**（间隔 `RPI_RESTART_CONFIRM_GAP_MS`）：Pi 的 `_require_double_send` 要「两次操作各一帧」，一次调用里连发两帧会被**只登记不执行**。
7. ⚠️ `STEP_RPI_SEND_TASK` 选到阻塞类任务码时（`"1"`/`"5"`/`"6"`…）**不挂起、也不进结果等待**（与改造前一致，「只发不等」语义没地方放结果）。路线作者该用专门的 `STEP_*_WAIT` 积木。

**遥测**（蓝牙 Tab「通信」组）：`rpi_pend`（挂起任务码，非 0 = 在等链路）、`rpi_pend_ms`、`rpi_link_retry`（累计 `"r"` **帧**数，奇数=卡在半对）、`rpi_pend_drop`（被顶次数）；配合 `rpi_link` / `rpi_rx_bytes` 判读：

| 现象 | 结论 |
|---|---|
| `rpi_link`≠1 且 `rpi_pend`≠0 且 `rpi_pend_ms` 在涨 | 任务在等重连 |
| `rpi_link_retry` 在涨 | 正在往复重启 Pi |
| `rpi_pend` 由非 0 变 0 而路线没往下走，`rpi_pend_drop`+1 | 被新任务顶掉了 |
| `rpi_rx_bytes` **冻结** + `rpi_link`≠1 | Pi 根本没跑（关机/线断）→ 重启指令发不进去，**重启无用** |
| `rpi_rx_bytes` **在涨** + `rpi_link`≠1 | Pi 活着但从没重新握手 → **这正是"重启客户端"唯一真正有效的场景** |

三条新宏在 `app_config.h` 的「RPI 通信」段（走「调参」Tab，**改完要重编译**）：`RPI_LINK_RETRY_TIMEOUT_MS=6000` / `RPI_RESTART_CONFIRM_GAP_MS=500` / `RPI_SYSCMD_SUPPRESS_MS=60000`。2026-11 加健康检查时同段又加了一条 `RPI_HEALTH_TIMEOUT_MS=12000`（握手积木勾选「健康检查」时等 `"Z"` 结果的上限）。

**台架自证**：`UART_RPI` 的 TX 接 USB-TTL **只收**、Pi 全关，跑含 QR 步骤的路线 → ①链路非 OK 期间**一帧 CMD 都没有**；②每 ~6s **两帧 `"r"`、间隔 ≈500ms**；③Pi 上电 → 链路转 `OK` **那一瞬恰好一帧** CMD、任务码 == **最后投递**的那个；④挂起期间按 OLED「拍照」→ `rpi_pend_drop`+1，恢复时**没有** QR 帧。

## 蜂鸣命令（Pi → 单片机, 2026-10-27 加）

`TYPE_COORD` + TAG=`0x0C`，DATA=`[0x0C][pattern u8]`（2B）—— **树莓派控制蜂鸣器的唯一通道**。

- `pattern` 与固件 `app_beam.h::BUZZER_PAT_*` 同值（**跨仓库契约**, 别改）：`1` = 失败音（响 0.8s → 停 0.5s → 响 0.8s, 共 2.1s）；`2` = 短滴（响 0.2s 一声）；`0`/未知 = 忽略。
- 执行**异步非阻塞**：`vRpiTask` 解析后只投请求（`vSetBuzzerReq`），蜂鸣任务 `vBuzzerTask`（freertos.c 创建, 最低优先级）消费并执行 —— `vRpiTask` 禁阻塞, 失败音整段 2.1s 只能在蜂鸣任务里响。蜂鸣进行中再来的请求**覆盖**待响项（后写者胜, 不排队 —— 报警音只要"最新那个"）。
- Pi 侧发送：`serial_comm.send_beep(pattern)`（帧常量 `protocol.TAG_BEEP` / `protocol.BEEP_PAT_*`）。串口断开抛 `SerialDisconnected`（与其它发送同口径, 调用方自行兜）。
- ⚠️ **固件自己也在失败点投失败音**（同一执行者）: ① 树莓派任务失败/超时（QR/人质/色球/靶标各等待函数, 每次失败一声; **中止不响** —— 停是操作员的意图；**色球对位链内被压制** —— 进「拉回（赌）」与整链终失败才响, 2026-10-27 用户定「不到赌不响」）；② 看路失败（`G` 的失败帧/超时/未标定/无左边缘）；③ 平移射靶精调环**丢靶**（从"跟到"变"丢失"的**边沿**投一声 0.2s 短滴, 不是每帧都响）；④ **陀螺仪损坏**（跑动中姿态帧号停涨超 `BUZZER_YAW_STALE_MS`, 每次损坏事件一声）。**这些事件 Pi 别再发 0x0C, 会双响** —— 该命令留给 Pi 独有的判据（如 Pi 内部重试耗尽、Web/调试手动触发）。
- 部署兼容：旧固件没有 0x0C 分支 → 2B 帧静默丢弃（不炸链路）；新固件 + 旧 Pi = 收不到命令而已, 固件自身的失败音照常。
