# 姿态传感器（双口 × 三设备 + 热备自动切换）

> 由根 `CLAUDE.md` 拆分；章节标题保持原样（源码注释按节名引用）。
> 相关：根 `CLAUDE.md`「跨模块共享状态 —— 统一走 Beam」、`docs/bt-protocol.md`

## 姿态传感器：两口 × 三设备 + 热备自动切换（2026-10 扩展）

车上**两个物理口各挂一颗陀螺**，**挂哪颗由「调参」Tab 的选型决定**（可以互换、也可以两口选同型号）：

| | 口 | 引脚 / 丝印 | 设备码 | 说明 |
|---|---|---|---|---|
| 主口 | USART6 | PC6/PC7（丝印 `HWT101_TX/RX`） | `GYRO_MAIN_DEV` | 0=ATK-901 1=M0 2=HWT101 |
| 副口 | USART2 | PD5/PD6（丝印 `VICE_GYRO`） | `GYRO_SUB_DEV` | 同上 |

装两颗是为了**一颗在场上挂了能马上换**（现在是自动换，见下）。三套协议**完全不同**，别混：
- **ATK-MS901M**（`atk_ms901m_device.c`）：0x55 0x55 帧头 + 变长帧，出 roll/pitch/yaw。
- **M0 单轴 yaw**（`gyro_m0_device.c`，源自台架示例 `Desktop/GyroPID`）：115200 8N1；模块→MCU 5B `0x5A type dataL dataH sum`（`type=0xAA` 角速度 / `0xBB` yaw，小端 int16，yaw = raw/32768×180°，±180°）；MCU→模块 5B `0x55 0xAA reg dataL dataH`（末两字节是**数据**不是校验和）；寄存器 `0x02` 回传频率 / `0x03` 波特率 / `0x0A` 校准 / `0x13` 解锁 / `0x15` yaw 置零。
- **HWT101**（内联在 `app_sensors.c`）：0x55 + 0x53/0x52 + 11B + 累加和 —— 历史还原档，2026-10 起又装回主口当主陀螺。

「调参」Tab「姿态传感器选型开关」段 = **三条下拉 + 两个参数**：`GYRO_MAIN_DEV` / `GYRO_SUB_DEV` / `SENSOR_GYRO_SRC`（依次 0/1/2、0/1/2、0/1 —— 三条都是 `选项:` 渲染的下拉）、`GYRO_AUTO_SWITCH`（勾选框）、`GYRO_AUTO_SWITCH_MS`（停摆判定窗，默认 2000ms）。

### ⚠️⚠️ 「两口选同型号」要求三个解析器都可重入 —— 2026-10 已改造

原来三个解析器的状态都是**函数级 static**，两口同型号 = 两个任务踩着同一个状态机、**静默解出错数**。现在全部改成「按实例传状态」：
- `gyro_m0_parse_byte(stGyroM0RxTdf *pstRx, …)` —— 帧状态机由调用方给（`gyro_m0_device.h` 的 `stGyroM0RxTdf`）。
- `atk_ms901m_bind_rx_buf(uart_id_t, uint8_t*)` —— 那个 512B 接收暂存改成**按口绑定**，没绑定就回退到驱动内的默认缓冲（**单实例用法行为与改造前逐字节一致**）；`app_sensors.c` 两个任务各自把本口的 `aucRx` 绑上去。⚠️ 满量程 `g_atk_ms901m_fsr` 仍是全局单份（只被本项目不调用的 `atk_ms901m_get_gyro_accelerometer` 读）。
- HWT101 的三个 static 挪进 per-port 上下文 `stGyroPortCtxTdf`（`app_sensors.c`，`s_astPortCtx[2]`）。
⚠️ 这只做到「**每个口一份**」：**同一路串口被两个任务并发调用照样坏**（本工程不可能 —— 一个口只归一个任务）。

### 发布门 = **有效源**，不是首选

- `gyro_src`（Beam, 蓝牙可写，上电默认 = `SENSOR_GYRO_SRC`）= **首选**（操作员设的）。
- `gyro_src_eff`（Beam, 只读）= **有效源**（真正在发的那一路）。
- 两个传感器任务**都在跑、都在解析**（与选没选中无关）⇒ 每口的帧号/新鲜度一直可见；但**只有有效源那一支**调 `vSetAttitude`/`vSetYawFresh`。下游（`app_pid`/`app_line`/`app_oled`/`app_rpi`）**一行都没改**（只读 `fGetAttitudeYaw`/`ulGetYawFrameCount`/`ucTakeYawFresh`）。
  ⚠️ **两个都发是错的**：`vSetAttitude` 是一个临界区里的三字快照，两源混发会让 roll/pitch 与 yaw 来自不同模块。
- ⚠️ **别把两个量合成一个**：合成就没有"切回首选"可言（热备复原靠的就是这个区分）。

### ⚠️ 热备自动切换（`vGyroSrcWatchdogTick`，`app_sensors.c`）

由 **`vPidTask` 的 50ms 节拍**调用（工程里唯一常驻的 50ms 周期；只读 Beam + 写 Beam，不发串口、不延时）。三条规则、**幂等、每拍全量重算**：

| # | 条件 | 动作 |
|---|---|---|
| ① | 操作员改了首选（`gyro_src` 与上次观察值不同） | 有效源 := 首选（**手动优先**，立刻跟随；`[man]`） |
| ② | `!live(有效源)` **且** `live(另一路)` | 有效源 := 另一路（**立刻切**，跑路线中也切；`[fail]`） |
| ③ | 有效源 ≠ 首选 **且** 首选**连续两个判定窗**都 live | 有效源 := 首选（**热备复原**；`[auto]`） |

- `live(口)`：**出过帧然后停了** → `now − ulLastFrame < GYRO_AUTO_SWITCH_MS` 才算活（真掉线）；**从没出过帧**（`ulLastFrame == 0`）→ 上电后 `GYRO_STARTUP_GRACE_MS`(3000ms,**app_sensors.c 里的固定常量, 不是标定量**) 之内一律算活、之后才算死。
  ⚠️⚠️ **没有这条开机宽限就会每次上电白跳一轮**：ATK 初始化完 ~0.05s 就出帧，M0 要跑完开机配置（解锁→频率→yaw 置零，每步 100ms）≈0.5s 才出第一帧 ⇒ 头 0.5s 里「M0 还没起来」会被判成「M0 死了」→ 切 ATK（日志 `[fail]`）→ 4s 后复原（日志 `[auto]`）→ OLED 闪一下 `ATK!`、`gyro_auto_sw` 白涨。**别把 `ulLastFrame == 0` 简单当"新鲜"**（`ulNow − 0` 在头两秒小于任何窗口，真故障永远判不出来），也**别一律当死**（就是上面那个误判）。
- ⚠️ **②里"另一路也要 live"是防抖的关键**：两路都死时**不能**动作，否则会在两者间每 2s 来回跳一次、yaw 反复跳零点。⚠️ ③ 要"两个窗"是给首选的恢复去抖。
- **关掉 `gyro_auto_switch` 时只跑 ①** ⇒ 有效源恒等于首选，**行为与加这个功能之前逐字节一致**（回归判据）。
- **每次换源都往 Pi 打一条 DBG**（用户要求写明"从哪颗到哪颗"）：`GYRO: sub/M0 -> main/ATK [fail]`（`[man]`/`[auto]`/`[fail]` 三种原因）。口名 `main`/`sub`，设备名按**该口的选型**查 —— 所以选型填错时这条会照实报出"你以为的那颗"。⚠️ 报文最长 33B（`RPI_DATA_MAX` 64B），别再加字段。
- ⚠️ **代价照实说**：换源瞬间 `fGetAttitudeYaw()` 会跳到另一颗的零点（两颗都以上电朝向为零点，正常只差几度；若 ATK 实际用了磁参考则差值更大）—— 正在跑的 `absolute=1` 绝对航向闭环 / 巡线全局 `LINE_CONV_SRC=1(陀螺)` 档会看到突变。手动切换本来也是这个代价（文档原本要求"先停车再切"），自动切换**刻意不等停车**（用户定：跑动中挂掉也要马上换）。
- ⚠️ **M0 是积分式单轴陀螺**：±180° 回绕、无绝对航向、慢漂。要靠 `absolute=1` 之前，先照 ATK 那套（上电读 OLED → 断电 → 原地转 90° → 再上电读）确认零点真是"开机朝向"。

### 现场判据（蓝牙 Tab「陀螺仪」组 —— 组名 2026-10 从「M0陀螺」改掉）

`gyro_src`（首选, 可写）/ **`gyro_src_eff`（实际在用, 只读）** / `gyro_auto_switch`（开关）/ `gyro_auto_sw`（自动切换次数, >0 = 真发生过）/ **`gyro_main_frames` / `gyro_sub_frames`**（两口各自的帧号, **停涨 = 那颗模块掉线 / 选型填错**）/ `m0_yaw`（**模块原始符号**，未乘 `M0_GYRO_YAW_SIGN`）/ `m0_frames`。
- 前两条在蓝牙 Tab 里是**下拉框**（"0=主口那颗 / 1=副口那颗"）—— 见下面「蓝牙参数下拉」。
- OLED 第 1 页标签 = **有效源**那口的设备名（`ATK-`/`M0-`/`HWT-Yaw=`），**末尾带 `!` = 有效源 ≠ 首选**（自动切换接管了）。宽度上限 16 字符=128px，别再往里加字。
- 现场定位 M0 方向：手转车身 90° 逆时针，`att_yaw` ≈ +90 而原始 `m0_yaw` ≈ −90 → `M0_GYRO_YAW_SIGN = -1` 是对的。

### ⚠️ 蓝牙参数的**可选下拉**（`pcOptions`，2026-10 加）

`stBeamParamTdf` 尾部多了一个可选字段 `pcOptions`（`"0=主口那颗|1=副口那颗"`）：非空 ⇒ 「蓝牙」Tab 把该参数渲染成**下拉框**（可写 → 选完即写；只读 → 置灰但仍显示选项名）。链路：`bt_cmd_list` **只在非空时**追加第 13 段 → `bt_link.parse_param_line` 接受 **12 或 13** 段（⚠️ 别写成 `!= 13`，普通参数会全被判无效）→ `parse_options()` → `app.js::btParamRow` 的下拉分支（优先于勾选框；比对走 `optKey`）。
- ⚠️ 选项串里**不能有逗号**（`$B` 无转义，一个逗号就把记录串行）—— 用 `|` 分隔。表现：`gyro_src` / `gyro_src_eff` 两条。
- ⚠️ `renderBtParams` 的"正在交互就不重绘"判据现在**同时认 INPUT 和 SELECT**（否则下拉正拉开时被重建、点不中）。
- 没填 `pcOptions` 的行**一个字节都不变**（老客户端照旧）。

### 陀螺口频率/波特率（「调参」Tab 段「姿态传感器选型开关」）

**按口**配（`GYRO_MAIN_*` / `GYRO_SUB_*`），作用于**该口当次选的设备**（ATK 没有这两个旋钮, 填了也不理会）：
- `GYRO_x_RATE_HZ`（默认 100）：固件就近匹配该型号的档位表。HWT101 连续档最高 **200Hz**（官方例程 wit_c_sdk 的 REG.h：0x09=100、0x0B=200；**0x0A=125Hz 是 WT931 专属**，别收）；M0 档到 1000 但 ⚠️ **>200Hz 时角速度+角度两帧把 115200 打满**（≈87%，超容量突发被 `usUartDmaTakeCopy` 静默丢）——除非波特率也上 230400，而那只有 HWT101 支持。
- `GYRO_x_BAUD` + `GYRO_x_BAUD_APPLY`（默认 115200 / 不下发）：换挡靠**会合机制** —— 两口在 CubeMX 里固定 115200 **别动**，模块只改 RAM、上电也回 115200，固件每次开机先在 115200 会合（下发频率/置零）→ 发新波特率 → `ucUartWaitTxDone` 排空 TX → `vUartSetBaud` 让 **MCU 本口跟着换**（app_uart.c，只动 BRR、RX 循环 DMA 保活）。填错给模块重新上电即自愈，不必上位机救。档位表：M0 只到 115200（兄弟表里的 230400 是笔误）、HWT101 官方例程上限 230400 —— 越表值由 app_sensors.c 顶部的编译期 `#error` 拦（控制台不校验）。
- 型号专属的 `M0_GYRO_YAW_SIGN`（默认 −1）/ `M0_GYRO_ZERO_ON_BOOT`（默认 1）/ `GYRO_HWT_ZERO_ON_BOOT`（默认 1，CALSW=CALANGLEZ，**⚠️上电时车必须静止**）在「陀螺模块专属参数」段 —— **跟型号不跟口**。两颗都开机置零是热备互切不跳大 yaw 的前提。
- ⚠️ **开机配置整段在该口任务的首次循环里发**（调度器起来之后才有 `vTaskDelay`），中间 100ms 一档。**刻意不发保存帧**（两套协议同一条纪律）。旧的全局 `M0_GYRO_RATE_HZ / M0_GYRO_BAUD_CODE / M0_GYRO_BAUD_APPLY` 已删（拆成按口六条），外部脚本别再引用。
- HWT101 配置命令字节（`FF AA reg dataL dataH`、解锁 `FF AA 69 88 B5`）已逐字节对照官方例程 wit_c_sdk（本机 `Desktop\HWT101_STM32F1`）核对 —— 与 M0 的 `0x55 0xAA` 命令是两套协议，别混。

### 陀螺校准入口（OLED「陀螺校准」→ 脚本任务, 2026-10 加）

「PI管理菜单」页的「陀螺校准」项（`OLED_ACT_GYRO_CAL`, 追加在动作码枚举末尾）投 `SCRIPT_REQ_GYRO_CAL` → `vScriptTask` 调 `app_sensors.c::ucGyroCalActiveSource()` —— 校准**有效源**（`gyro_src_eff`）那颗, 想校另一颗先去切「陀螺源」。三颗设备三种待遇:

| 设备 | 动作 | 阻塞 |
|---|---|---|
| M0 | `gyro_m0_bias_cal`（解锁→零偏自校准→保存, 驱动收口）+ yaw 重新归零 | ~21s |
| HWT101 | 解锁 → CALSW=1 开「陀螺自动校准」→ 静止 5s → CALSW=0 关 → SAVE → CALSW=4 重零 | ~7s |
| ATK | 拒绝: 往 Pi 打 `GYRO cal: ATK has no cal cmd` + OLED 闪「校准失败!」 | 即回 |

- ⚠️⚠️ **原来那两条硬约束现在由结构保证, 别破坏**: 校准函数**只从脚本任务的请求循环**调（校准阻塞期间没人 `vSetYawFresh()` → 陀螺闭环会判停摆 —— 而脚本任务同一时刻不可能在跑路线, 「不发布 yaw 的任务」这个前提天然成立）; **绝不许**挪进 `vSensorTask` / `vSensorTaskM0` / OLED 任务。脚本任务在校准前后占住 `script_running`（同握手让出口径）→ 这段时间蓝牙进不了机械臂编辑模式。
- ⚠️ **HWT101 那条等完必须把 CALSW 关掉**: HWT101 手册里 CALSW=0/1 = 关/开「陀螺自动校准」的**开关**, 且官方对「匀速运行场景」（AGV 小车/舵机/云台 —— 本车的支点转弯原样命中）的建议就是**关闭**, 开着会让匀速旋转被当零偏慢慢吃掉。发的 `SAVE` 是**有意**的: 校准的意义就在跨上电; 「配置不发保存帧」那条纪律管的是频率/波特率这类填错能自愈的东西（`GYRO_HWT_REG_SAVE` 处注释已改口径）。
- ⚠️ **校准期间被校那颗不出帧是预期**（HWT101 官方口径"输出暂停或不更新", M0 忙 21s 同理）→ `vGyroSrcWatchdogTick` 见 `ucGetGyroCalBusy()` 就**整段让路**（不判停摆、不切源）—— 少了这道门, 每次校准都白跳一轮 `[fail]`+`[auto]`、白涨 `gyro_auto_sw`。
- ⚠️ **等待段是协议常数不是标定量**: M0 的 21s 写死在驱动里（模块流程）, HWT101 的 5s = `app_sensors.c` 的 `GYRO_HWT_CAL_HOLD_MS`（官方口径"保持静止几秒钟"取裕量）—— 都**别往「调参」Tab 加**。
- 反馈（OLED 标签三态, 同拍照那条的挂法）: `陀螺校准`（闲）/ `校准中请勿动车`（busy, 7 字 112px —— 车一动零偏就估歪了, 固件没法替操作员按住车）/ `校准完成!`·`校准失败!`（done_tick 起 1s）。busy 期间再按菜单项 = 忽略（别把刚估好的零偏搅掉）。
- 校准完两颗都会把**朝向重新归零**（与"上电朝向=0°"口径对齐; HWT101 重零时车必须仍静止 —— 它本来就在静止等待段末尾, M0 走 `gyro_m0_yaw_zero(...,0)` 不保存）。

### 构建

⚠️ **`Drivers/QEPack/gyro_m0_device.c` 是新增源文件**，已插进本机 `MDK-ARM/GongKeCompetition.uvprojx` 的 Drivers/QEPack 组，但**该工程文件被 `.gitignore` 忽略** → 队友拉下来**必须自己把它加进那个组**，否则一片 `undefined symbol gyro_m0_parse_byte`。（零风险：整个 M0 任务在 `app_sensors.c` 里，没新增 Application 源文件。）
