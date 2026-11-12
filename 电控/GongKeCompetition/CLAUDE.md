# CLAUDE.md

工科赛小车固件：STM32F407 + FreeRTOS（裸 API V11.3.0）+ QEPack 驱动，树莓派/OPENMV 视觉引导，麦轮底盘 + 六轴机械臂。

## 文档索引（改哪块先读哪份）

根文件只留**全局铁律、跨模块约定与目录结构**；具体功能的踩坑细节已按主题拆到 `docs/`（**不会自动加载**，改对应区域前先打开对应那份）。
**章节标题逐字保留**（源码注释里有 30 多处 `CLAUDE.md` 引用、分布在 20 多个文件，多处按 `CLAUDE.md「节名」` 定位），按名字在下面找文件即可。

| 主题 / 关键词 | 文档 |
|---|---|
| 6 路 UART、RX 被 HAL 静默停掉、TX DMA 环、遥测对账 | `docs/uart.md` |
| 树莓派指令/结果帧同步清单、链路不通挂起、自动重启 Pi 客户端 | `docs/pi-link.md` |
| 蓝牙 `$B` 协议、Beam 参数、舵机速度总闸、组动作偏移、臂序列并发 | `docs/bt-protocol.md` |
| 支点转弯→巡线接管、巡线直走、巡线收敛增强 | `docs/line-follow.md` |
| 标靶对位/校准/流控闭环、平移射靶、OLED 射靶测试 | `docs/target.md` |
| 单动作抓取（色球/人质） | `docs/grab-one.md` |
| 相机预热、二维码（两相机×两算法）、色球区多方案对位链 | `docs/vision-tasks.md` |
| 看路（纵向/横向）、看路校准 | `docs/road.md` |
| 雪弗板减速带过坎、EMM 环模式切换 | `docs/chassis-misc.md` |
| OLED 菜单（页面/滚动/二维码页/拍照/字库） | `docs/oled.md` |
| 姿态传感器（双口×三设备、热备自动切换） | `docs/sensors.md` |
| 控制台元数据从源码派生（积木库/参数名/下拉） | `docs/console-metadata.md` |
| 历轮全仓库审查记录、「本轮明确不动」清单 | `docs/reviews.md` |

- **按节名找**：`标靶对位 / 标靶校准 / 标靶流控闭环 / 平移射靶 / 射靶测试` → `docs/target.md`；`支点转弯 → 巡线接管 / 巡线直走 / 巡线收敛增强` → `docs/line-follow.md`；`新增蓝牙指令 / 舵机速度总闸 / 组动作角度偏移 / 机械臂相关指令` → `docs/bt-protocol.md`；`新增树莓派指令 / 向 Pi 发任务` → `docs/pi-link.md`；`看路 / 看路校准` → `docs/road.md`；`OLED 菜单 / 拍照 / 中文字库` → `docs/oled.md`；`姿态传感器` → `docs/sensors.md`；`雪弗板减速带过坎 / 步进电机环模式切换` → `docs/chassis-misc.md`。
- 新增/修改功能细节请直接改 `docs/` 下对应文件；不要把长章节再写回根文件。

## 目录结构（关键部分）

```
GongKeCompetition/
├── Application/      ← 应用层：按模块分任务（app_*.c），跨模块状态统一走 app_beam
│                       串口协议各占一个模块：app_rpi.c（树莓派帧协议）/ app_bt.c（蓝牙 $B 协议）
│                       机械臂动作预设表：app_arm_action.c/.h（控制台生成/维护）
│                       路线预设：map.c（各表 + 预设注册表）+ map.h（id 枚举生成块）
│                       条件分支 = 就地嵌套的「如果/否则/结束如果」（不占预设）
├── Core/             ← CubeMX 生成：HAL 外设初始化 + FreeRTOS 任务创建（freertos.c）
├── Drivers/QEPack/   ← 自研驱动库（servo/key/uart/emm_v5/atk_ms901m），匈牙利命名
├── MDK-ARM/          ← Keil 工程（GongKeCompetition.uvprojx，**被 .gitignore 忽略**）
├── tools/webconsole/ ← 本地 Web 控制台（server.py + host_tools.py + tuning.py +
│                       bt_serial.py + bt_link.py + capture.py + map_codec.py +
│                       arm_action.py + srccheck.py + static/，stdlib-only，
│                       唯一例外是「采集」Tab 的可选 cv2）—— 文档见该目录下的 CLAUDE.md
├── tools/fontgen/    ← OLED 汉字字模生成（gen_font.py + check_coverage.py + glyphs.txt）
│                       造字用 Pillow，**只在造字时用**；固件与控制台都不碰它
├── ThirdParty/       ← FreeRTOS-Kernel
└── vision/           ← 树莓派 YOLOv8n 人质目标检测（三分类 cylinder/cone/drum）
```

- **HAL 时基**：TIM6（`HAL_IncTick`）；业务 1ms 中断：TIM7（舵机平滑）/ TIM1（按键扫描）。角度 PID **不占中断**，由 `vPidTask`（`app_pid.c`）以 `vTaskDelayUntil` 50ms 驱动（TIM5 空闲）。
- **补光灯 PWM 独占 TIM9_CH1 @ PE5**（21kHz，`vFillLightSet` 千分比 0~1000 → CCR ×8）：2026-09 从 TIM3_CH1@PA6（50Hz，与舵机共周期导致肉眼可见频闪）迁出。与 TX DMA 同理**手写在 `app_actuator.c`、不进 CubeMX/.ioc**，重生成冲不掉；TIM3 从此只挂舵机（50Hz），原 PA6 的 MX 配置闲置无害（线已挪到 PE5）。灯现在装在主摄旁（机械臂倒数第二节，6500K）。
  - **任务补光（2026-10 加）**：看路/色球/射靶/人质四类任务 + 主摄二维码扫描（`STEP_QR_SCAN_MAIN_WAIT`；扫码副摄那步**不挂** —— 灯在主摄旁照不到副摄画面）里**靠相机看东西的步骤**（识别/对位/校准/抓取/射击/扫码）全程自动开灯、出步必灭。唯一开关出口 = `vScriptExecStep`（`app_script.c`）：进步查 `ucScriptFillTaskOf()` 的步骤→类别映射表（⚠️ 新增任务的视觉步骤要来挂一行，漏了不报错、只是那步没补光），调 `vFillLightTaskBegin/End`（`app_actuator.c`）；放 switch 之外是为了不脏控制台的「case 体第一个函数调用=积木原语名」。配置在 `app_config.h`「任务补光」区（总开关 + 默认亮度 + 每任务 `*_EN`/`*_PCT`，二维码是 `FILL_TASK_QR_EN`/`FILL_TASK_QR_PCT`；PCT **留空（调参页空框，文件存哨兵 255）= 用默认亮度、填 0 = 开灯不亮**，固件把 >100 一律当「未填」；编译期量，改完重编译）。「调参」Tab 新分区随解析自动出现。与手排积木 `STEP_BUGUANG_KAI/_GUAN` 后写者胜——任务步骤里手动开的灯会被步末灭掉。真开了/灭了会打调试串 **`FILL: on` / `FILL: off`**（`vRpiSendDebug` 只收静态字符串，所以不带任务名/亮度；`vFillLightTaskBegin` 返回实际占空比 ‰ 供判「真开了」，EN=0 不打）——现场分岔点：跑任务步骤没这行 = 车上还是旧固件；有这行而灯不亮 = 查灯/驱动（20kHz PWM 对光耦模块太快的坑，见 `app_actuator.c`）。
- **串口**：6 路 UART。**收** = DMA 循环 + IDLE 中断 + 二值信号量；**发** = DMA 环 + TC 中断链（都在 `app_uart.c`）。已弃用 QEPack `uart_device`。
  - ⚠️ **读串口只有一个入口：`usUartDmaTakeCopy(id, dst, cap, timeout)`** —— 它交付的是「**本次新增字节**」（起始偏移 + 长度），并已把圆环回绕拼成连续数据。**别再写「拿缓冲首地址 + 把它当整帧长度」的解析**：循环 DMA 下 `512 - NDTR` 是**写指针位置**而不是帧长，那个老写法只在帧首恰好落在圆环起点时才碰巧读对（曾让 RPI 链路整体失效）。帧可能跨 take 被切开、也可能几帧粘在一次 take 里 → 消费者要有累加/重同步能力（RPI 见 `vRpiTask` 的字节流扫帧、BT 见 `bt_feed` 的行累加器）。
  - 单缓冲：写者不停覆盖，消费者必须及时取走；取走慢于帧率时中间那些帧会被跳过（ISR 每帧推进读位置，不排队）。需要背压得另加环形队列。
  - ⚠️ **RX 会被 HAL 静默停掉、而 TX 照常**（真故障，修复别当冗余删）：真正兜住的是 `vUartDmaIdleHandler` 的 **IDLE 卡死检测**（六路都有）；另两道是 `HAL_UART_ErrorCallback` 的 RX 边角（保留「`hdmarx->State == BUSY` 就别碰 RX」那道门）与 `vRpiTask` 的 5s 兜底重装（仅 RPI）。判据：`rpi_rx_bytes` 停涨 + TX 正常 = 本端已聋。**三道兜底的完整机理、TX DMA 环、遥测对账见 `docs/uart.md`。**
  - **TX 唯一入口 `vUartSendArray`**：拷进 512B 环即返回（DMA 后台发），**返回 ≠ 已上线**；环满背压。只能在任务上下文调，**绝不能在 `taskENTER_CRITICAL()` 里调**（铁律 5/6）。TX DMA 流手写在 `app_uart.c`、不进 CubeMX。

- **业务动作链**：路线表 `map.c`（纯数据）→ 执行器 `vScriptRun()`（`app_script.c`）→ 底盘原语（`app_chassis.c`）/ 角度闭环（`app_pid.c`）。改路线只动 `map.c`，不动执行逻辑。
- **多预设路线**：`map.c` 里可放多套具名路线 `g_astScriptPresetN[]`（每套前有 `/* ===== 预设: 名字 ===== */` 头注释），末尾一行 `#define SCRIPT_PRESET_ACTIVE <表名>` 决定设备**开机跑哪套**；读数走 `g_pstScriptActive` / `g_usScriptActiveCount`（`map.h`），加预设不影响其它模块。`sizeof(SCRIPT_PRESET_ACTIVE)` 自动取长度，无需维护计数。
  - 另有**预设注册表** `g_astScriptPreset[]`（id → 表指针 + 长度），供「跳到别的预设」那套积木按 id 查表 —— 见「跳到别的预设」一节。注册表（`map.c`）与 `emScriptPresetIdTdf` 枚举（`map.h` 的生成块）**都由控制台生成**，两者必须由同一份列表产出否则会互相对不上。⚠️ **条件分支不走这张表**（就地位置配对），所以嵌套多少层分支都不会多出预设。

## Web 控制台 —— 文档在 `tools/webconsole/CLAUDE.md`

双击仓库根 `webconsole.bat`（`tools/webconsole/server.py`，端口 8410，浏览器自动打开）。**改完服务端代码要重启它**（`allow_reuse_address=False`，双开会直接报端口占用）。已实装十一个 Tab：「地图」「调参」「位移」「自定义」「标定」「采集」「调试助手」「蓝牙」「机械臂」「底盘动作组」「OLED」。

**这个目录的完整文档（含全部 ⚠️ 细节）在 `tools/webconsole/CLAUDE.md`** —— 动 `tools/webconsole/` 下任何文件前，或要改 `map.c` / `map.h` / `app_config.h` / `app_script.h` / `app_arm_action.c/.h` 之前，先读那一份。根文件只留三条前提：

- 绑 `0.0.0.0`（`BIND_HOST`），局域网内队友能一起用；**无鉴权**（见下面「安全姿态」）。
- 地图 / 调参 / 标定 / 机械臂 / 底盘动作组 / OLED 等端点会**回写仓库源文件**，多客户端靠 base_version/409 乐观锁 + `/api/rev` 轮询同步。
- 蓝牙链路用 `ctypes` 直调 Win32 串口，**stdlib-only 是硬约定**（唯一例外是「采集」Tab 的可选 cv2）。

## ⚠️⚠️ 安全姿态（无鉴权）—— 排障与取舍前先读这一段

**Web 控制台没有任何鉴权，而且绑在 `0.0.0.0`。** 谁能在网络上访问到 `8410`，谁就能写仓库源码（`map.c` / `app_config.h` / `app_script.h` / `app_arm_action.c`）、改 `~/.ssh/config` 与网络位置、拉起 `mstsc` / Trae / VS Code、驱动真实舵机、开摄像头、断电重启那台树莓派。

- **Origin 校验（`server.py::origin_ok`）不是鉴权**：它只防**浏览器**侧的跨站请求与 DNS rebinding。**非浏览器客户端（curl、脚本、任意局域网程序）一律放行** —— 它们不发 `Origin`，而判据是「不带 Origin 就放行」（为了让 `curl` 能用）。别把它当访问控制。
- `GKS_CONSOLE_HOSTS` 环境变量只解决**误拒**（队友用 mDNS/路由器域名访问时 403），**不解决「放太开」**。没有「只监听本机」的开关 —— 要收紧就把 `server.BIND_HOST` 改成 `127.0.0.1`（那就只有本机能用）。
- **取舍（2026-09 明确决定，别再当成待办）**：比赛局域网里大家都是自己人，加口令会让队友每次开页面都要输一遍、还会跟「双击 `webconsole.bat` 就自动开浏览器」打架，所以**刻意不加**。代价照实说：场地网络不可控时（观众 / 其他队伍同网段）**任何人都能接管这台车**。真要在那种场地用，就赛前把电脑接入受控网络，或临时把 `BIND_HOST` 改成回环。

## 积木参数注释 DSL

**参数注释 DSL**（`app_script.h` 参数映射表，控制台可**右键积木 → 编辑**并写回）：

```
 *   STEP_RPI_SEND_TASK   anI: task_x=任务码   // 备注(可含逗号分号)
 *   DSL 语法: 名字[=中文名][[单位]][{值=标签,...}][<显隐:参数名=值>]  , 行尾 // 为备注
```
- 不写 `=中文名` 回退用参数名；不写 `[单位]` 按参数自身文本猜（ms/mm/rpm/°/%。**只看参数自身**，不继承同步骤其他参数的单位）
- 写 `{...}` 则显式指定下拉选项，优先级高于枚举自动派生
- 写回会改 `app_script.h`（备份 `.h.bak`），只动该步骤的枚举尾注释与参数映射行
- ⚠️ **行尾 `//` 备注写「排路线的人该怎么做」，不写「为什么」**：它同时出现在控制台三处（积木库条目副标题、右侧检查器「参数说明」、右键「编辑元数据」弹窗），过 ~120 字就是一堵没人读的墙。踩坑史/跨仓库算式/日志形态**一律留在根 `CLAUDE.md` 或 `docs/` 对应章节**（`app_config.h` 同名尾备注同理）。2026-09 精简过一轮：13 条只留「参数含义 + 排表约束 + 关键数值」，最长的 `STEP_HOSTAGE_ALIGN_WAIT` 从 496 字压到 133 字。
- ⚠️ 别顺手精简**枚举尾注释**（`/* 停车, 阻塞等 MAIN 按键继续 */` 那块）：它是控制台的「说明」，且新拖出来的积木默认名就取它（`app.js::newItemFromCatalog` 取 `desc` 首个 `(` 之前的前 16 字）——改它 = 以后新加的积木跟着改名（已有 map.c 行不受影响）。
- 尾部可写 `<显隐:参数名=值>`（语法仍在，但**目前没有任何积木用到** —— 原来只有巡线收敛的 `gyro_w` 用过，2026-09 随 `use_fusion` 一起删了）：**只在该积木的另一个参数读到这个值时**才显示本参数。只做字符串相等（`1` 与 `1u` 算相等）、只看同一个积木、只支持一个条件，值里不能出现 `, ; < >`（那是 DSL 的顶层切分符与标记边界，出现即整条拒识且无损回退）。
  - ⚠️ 它**只影响显示**：写盘照样带这个参数（值一个字符不动），固件按槽位取用。
  - ⚠️ **解析时必须最先剥掉它**（`server.py::RE_SHOW_IF`，在剥 `{选项}`/`[单位]` 与 `=中文名` 那次 `_split_top` **之前**）——选项/单位两个正则都是 `$` 锚定的，而 `_split_top` 不认 `<>` 的深度。
  - ⚠️ `_spec_str` 靠 `meta["show_if"]` 把它补回去，所以 `raw` 里不能残留标记（否则追加两次、第一次保存就产生 diff）；前端 `saveMetaEditor` 是**整行重写**，必须原样透传 `show_if` —— 漏了那条，改一次中文名就把标记无声抹掉（`test_server.py::ShowIfWriteMetaSandbox` 盯着这两条）。

## map.c 编辑与构建

**map.c 的两种禁用写法**（控制台自动切换，勿手改混淆）：
- **单步禁用** = 该行行首加 `//`（保留缩进）
- **分区禁用** = 整段用 `/*` 与 `*/` 各自单独成行包裹；**组内各步原有启停状态保持不变**。包裹时组内独立块注释（如 `/* 跑完全程: ... */`）会自动降级为 `//`，否则其 `*/` 会提前闭合外层包裹注释——取消包裹时再还原。
- ⚠️ 全部步骤都禁用会使数组变空初始化列表，C 编译报错（控制台保存前会警告）。
- **任务优先级**：见 `Application/app_config.h`（`APP_PRIO_*`）。
- **编译**：Keil MDK 打开 `MDK-ARM/GongKeCompetition.uvprojx`（ARMCC V5.06，宏 `USE_HAL_DRIVER,STM32F407xx`）。命令行：`cd MDK-ARM && C:/Users/Administrator/AppData/Local/Keil_v5/UV4/UV4.exe -b GongKeCompetition.uvprojx -j0 -o build_<name>.log` —— UV4 退出码不反映成败，须看 log 末行 `N Error(s)`；新增源文件要手动在 uvprojx 的 Application 组插 `<File>` 块。
  - ⚠️ **`GongKeCompetition.uvprojx` 被 `.gitignore` 忽略**（`.gitignore:7`）：往工程里加源文件只是**本地**改动，推不到仓库，队友拉新 `app_*.c` 后会以 `undefined symbol` 报错且毫无头绪。**新增源文件时必须在 PR/提交说明里写明「本克隆需手动把 xxx.c 加进 Application 组」**（或 `git add -f MDK-ARM/GongKeCompetition.uvprojx` 入库）。
    - **当前欠这一行的有两个：`app_arm_action.c`（2026-09 加）与 `app_chassis_action.c`（2026-10 加）**：都已插进本机 uvprojx，但**别人拉下来必须各自插一次**，否则 `undefined symbol vArmPlayWait` / `vChassisActionPlayWait` 之类一片。
  - **C 编译 MiscControls 必须含 `--locale=C --diag_suppress=870`**（源码 UTF-8，AC5 默认按 GBK 解析会把奇数个汉字的字符串引号吞掉报 missing closing quote；CubeMX 重生成工程后需补回）。
  - ⚠️ 注释里**别写 `Application/*.h` 这种带 `/*` 的字面量**——C 注释不嵌套，AC5 会报 `#9-D: nested comment is not allowed`。写「Application 下的头文件」。

## 脚本动作表设计规则（stScriptStepTdf / map.c）

脚本的价值在「粗颗粒 + 自由组合」：一步 = 一个完整任务，改路线/插断点只动表。新增或改步骤时对照：

1. **颗粒度要粗**：小任务链合成一步（「发二维码指令 + 等解析结果」= `STEP_QR_SCAN_WAIT` → `ucRpiQrScan()`）；要连写几行强相关子步骤就先封成动作大函数。
2. **一动作一函数**：`vScriptRun` 的 case 只调一个动作函数，内部再任意组合（如 `vChassisEnableAll`、`ucRpiQrScan`、`vScriptParkWaitKey`），map.c 行尾 `//` 标原语函数。
   - ⚠️ switch 已抽成 `vScriptExecStep`（「测试」积木复用同一份执行语义），case 体写在那里。
   - ⚠️ **case 体里第一个「标识符(」= 控制台显示的积木原语名**（`server.py::parse_catalog` 正则）——别在它前面放函数调用（`I(`/`F(` 宏也算：`int32_t x = (int32_t)I(0);` 会让积木显示成 `I`。真踩过）。前置校验塞进实参（`vSetQrCode(c_iQrTestSanitize((int32_t)I(0)))`）。
3. **延时归属**：过程性延时（到位等待、S 曲线时长、指令间隔、结果轮询）可写函数内；**动作完成后的沉淀延时（如抓合停稳）必须显式写成 `STEP_DELAY` 放表里**，不藏函数尾部。
4. **阻塞后缀**：过程性动作必须显式标注——阻塞到完成加 `_WAIT`，启动即返回加 `_NO_WAIT`（如 `STEP_ARM_GROUP_WAIT` / `STEP_ARM_GROUP_NO_WAIT`）。免标两类：瞬时指令（发帧/设参数/GPIO/PWM 写）与流程控制（`STEP_DELAY` / `STEP_STOP` / `STEP_DEAD` / `STEP_SCRIPT_RESET` / `STEP_IF` / `STEP_ELSE` / `STEP_END_IF` / `STEP_SCRIPT_CALL` / `STEP_SCRIPT_RETURN`）。⚠️ `STEP_IF` 免标**有代价**：条件选「巡线是否在线上」时会阻塞并跟树莓派对话（见「条件分支」），代价改由积木备注承担。
5. **自带闭环的运动步骤在自己的上下文里跑局部控制循环**（范例 `STEP_PIVOT_TURN_WAIT` → `vPidPivotTurn`：局部 PID 状态 + `PIVOT_TURN_PERIOD_MS` 阻塞节拍），**不要复用 `s_stPid`/`s_stRun`**（归 `vPidTask`；PID 空闲时进不了控制分支，不抢电机/串口，见 `app_pid.c::vPidPivotTurn`）。标定量做成 `app_config.h` 的 `#define` 走「调参」Tab。
6. **「重置脚本」= 索引回跳，不是重启任务**：`STEP_SCRIPT_RESET`（无参）把**当前这一帧**表的步索引跳回「表首第一条实质步骤」——跳过开头**连续**的 `STEP_WAIT_HANDSHAKE` / `STEP_DEAD`。排表要懂三件事：
   - **表首那份等键会被跳过，表中间那份照旧等键** —— 「每圈按一次键」把键排中间；「排表尾即自动循环」不需按键。
   - 表首那份必须跳：`ucTakeHandshakeSeen` 是 6 个一次性信号里**唯一没有周期性生产者**的（Pi 只在启动阶段发 `SYNC_HS_1`）—— 断连时重跑握手卡在 `while(!ucTakeHandshakeSeen())`，链路**已 OK** 时重跑更是**主动断连重来**。`STEP_STOP` 故意不跳（停车+清速, 语义比单纯等键重）。
   - ⚠️ **帧局部、不弹跳转栈**：深度 0（主路线）= 「回到本轮路线开头」，与旧版一字不差；被 `STEP_SCRIPT_CALL` 跳进来的**子预设里排它会自循环、永不返回**（整表都是握手/等键时 `ucScriptResetTarget` 全跳掉 → **不让出的紧循环**，饿死按键/OLED）。
   - ⚠️ 排在**嵌套 IF 分支里**也照旧是「回到**整张表**表头」，**不是**回分支开头 —— 分支只是位置配对、非作用域，也不会让标记失衡。
   - ⚠️ **实现上没有 `continue` 了**：`vScriptRun` 已改成「取到步骤**先 `usIdx++` 再执行**」，跳步只要赋值 + `break`。别再找「循环末尾的 `usIdx++`」（结构已不存在；禁令「别在该 switch 里加 `continue`」随机制作废）。帧索引在 `vScriptRun` 的 static 帧栈里，无外部重置入口。

7. **「支点转弯」与「原地转弯」不是一回事，别混用**：`STEP_PIVOT_TURN_WAIT`（`vPidPivotTurn`）单侧双轮驱动、另一侧持 0 当支点，**车心会平移**；`STEP_SPIN_TURN_WAIT`（`vPidSpinTurn`）四轮对称反向出力（`vChassisMove(0,0,w)`）、绕车身中心自转，车心基本不动。两者参数同形（角度 + 方向位 + **角度模式**）但增益**不可互换**（`PIVOT_TURN_*` vs `SPIN_TURN_*`）—— 四轮对转靠辊子打滑，同轮速下角速度约支点的 2 倍（差速自转 ω=2v/轮距）。两个易错点：① `SPIN_TURN_YAW_SIGN` 同时进「目标角」与「出力」两处，改符号自动一起翻，**不是补偿关系**、不用改两处；② 收尾写世界航向时**只有真收敛才写目标角**，超时/IMU 卡死写实测 —— `vPidPivotTurn` 分不出收敛与超时、一律写目标（往累加器写假值），**别照抄回去**。
   - **`absolute` 参数（2026-09 加）**：`0` = 相对（`angle` = 「从当前朝向转这么多」, 老路线不填就是它）；`1` = 绝对（`angle` = **目标世界航向** = 启动朝向为 0° 的那个航向, 与 `vChassisMoveWorld` 查表同一世界系）；用处：「转到世界 90°」不再受前面积累的朝向误差影响。
     ⚠️ 绝对模式下 `dir` **不起作用**（往哪转、转多少由「目标 − 当前」最短路径决定），也不做小角度早退（已在目标附近时收敛带立刻吃掉）。
     ⚠️ 「绝对」= 相对**启动朝向**，不是地理北 —— IMU 的 yaw 零点就是开机那一刻的车头朝向。
   - **`rev` 反向执行（2026-11-01 加, 第 3 个 int 尾槽勾选框, 默认不勾 = 老行为）**：车倒着放/场地换边跑镜像路线时勾 —— **转向与目标角不变**（「左转」仍逆时针转那么多）, 驱动侧换**对侧**双轮并**倒转**（左转：右侧正转 ⇒ 左侧倒转）：支点换到对侧、车心弧线镜像, 复现正向放置的世界轨迹。⚠️⚠️ **对侧与速度必须同时取反**：对侧倒转的 yaw 响应与正常侧正转**同号**（左轮退 = 车头左偏）, 闭环照旧负反馈, 增益/收敛判据/世界航向记账全不用动；只翻其中一个（同侧倒转 / 对侧正转）= yaw 响应翻号 → **正反馈飞车到超时**。`STEP_PIVOT_LINE_WAIT`（交接版）暂不带 rev —— 反向的「转弯→巡线」要连巡线侧一起镜像, 有需求再透传。

8. **⚠️⚠️ 给已有积木「加参数」时，光看编译过不算数 —— 老行的值是按位置映射的**（2026-09 踩到）：`map_codec` 按「**先所有 float、后所有 int**」读写参数；**新参数插在 float 槽而老行只填了 int 槽**时，老行在控制台被编辑一次，那个 int 值就被塞进 float 槽、原来的第一个 int 变成 0 —— 编译照过、控制台照显示，只在实车表现为「这一步莫名空转」。例：`STEP_LINE_CONVERGE_WAIT` 从「converge_ms, use_fusion, gyro_w」变成「gyro_target(float), converge_ms, gyro_w」时，老行 `SCRIPT_STEP_I1(..., 1000u)` 必须**全部**改写成 `SCRIPT_STEP_F1I2(..., 0.0f, 1000u, 0u)`。
   ⚠️ **反例（对照）**：2026-09 砍融合 `gyro_w`(0-100) → `src`(0相机/1陀螺/2不修) 槽数没变、老值**全是 0** = 新枚举「相机」→ **17 行一个字节都不用改**。**改参数语义先想"老值在新区间是什么意思"，能对齐就别造迁移**。
   三种追加写法：① `STEP_PIVOT_LINE_WAIT` 加「交接后听谁」+目标角（新 float 排**第二 float 槽**、新 int 排**末尾 int 槽**）—— int 末尾追加**不动老值**（缺槽补 `0` = 新枚举「相机」），但**新增 float 槽会让老行整体错位**（老行第一个 int 被当成它读 → 转向错位）；② 零迁移样板 `tail_ms` 排 **int 末尾（I(6)）**、**float 槽没动** → `map.c` **一个字节都不用改**（尾槽空 → 补 `0` = 不盲走）；③ `STEP_BALL_ALIGN_WAIT` 加 m1/m2/m3（纯 int 追加）**老行照样必须重写**（不重写三槽读出全 0 = `ALIGN_NONE`）。
   **规矩**：**能不插 float 就不插**（宁可把角度用十分位整数塞进 int 槽），非插不可就把该积木在 `map.c` 的**所有**行一并重写（那次 3 行）；生成一律用 `map_codec._serialize_step`（别手对齐，`test_server.py` 的真实 map.c 往返测试会验证）。
   ⚠️ **删掉/改名一个积木更必须留意**（2026-09 删 `STEP_IF_ELSE`）：`map.c` 残留老行**编译报 `undefined identifier`** —— **有意的硬失败**，但**提交说明里必须写明**：队友拉下来会编译不过，得把那些行换成新积木。控制台**不会**提示（catalog 已无它，`_serialize_step` 按 0 参数序列化 —— "在控制台编辑那一行"会把参数无声吃掉）。

```c
/* map.c 行格式范例 */
SCRIPT_STEP    (STEP_WAIT_HANDSHAKE,               "等待握手"),                                // ucRpiHandshake()
SCRIPT_STEP_I1 (STEP_QR_SCAN_WAIT,                 "二维码扫描",    8000),                     // ucRpiQrScan()
```

## 条件分支：嵌套容器 IF（`STEP_IF` / `STEP_ELSE` / `STEP_END_IF`）—— 默认写法

「地图」Tab 的「如果」积木，一块**就地**表达 if/else，**不占用预设**：

```c
SCRIPT_STEP_I1 (STEP_IF,   "如果: 二维码识别结果", SCRIPT_COND_QR),   // vScriptIf()
    SCRIPT_STEP_F1I2(STEP_LINE_CONVERGE_WAIT, "巡线收敛", 90.0f, 500u, 0u),  // ucRpiLineConverge()
SCRIPT_STEP    (STEP_ELSE, "否则"),                                  // vScriptElse()
    SCRIPT_STEP_I1(STEP_ARM_PLAY_WAIT, "机械臂待命到正视二维码模式", ARM_ACT_22),  // vArmPlayWait()
SCRIPT_STEP    (STEP_END_IF, "结束如果"),                             // vScriptEndIf()
```

**语义**（三块标记在**扁平数组里靠位置配对**）：
- 条件**成立** → 落进 then，执行到同层的「否则」/「结束如果」；走到「否则」时再收在「结束如果」之后。
- 条件**不成立** → 有「否则」就落到它之后；没有就落到「结束如果」之后。
- 「否则」**可省略**；**可任意层嵌套**。

**不需要栈**：`app_script.c::usScriptMatchMark` 从下一条往后扫、数嵌套深度找配对标记、把本帧 `usIdx` 挪过去 —— **无栈、无跨步骤状态**。由此：`STEP_SCRIPT_RESET`（`usIdx` 指回本表表首）/ `STEP_SCRIPT_CALL`（压帧，返回时父帧 `usIdx` 停在分支内部那条之后）在分支里照常工作；嵌套分支**不占** `SCRIPT_CALL_DEPTH`（与「跳到别的预设」两套东西），⚠️ 也因此**分支不是循环作用域** —— 分支里「重置脚本」仍是「重启**当前这一帧的整张表**」；扫到表尾没配对 → `usIdx = usCount`（本帧结束），**绝不越界**（**兜底**，正常路径由控制台保存前拦下）。

⚠️⚠️ **别把三块标记拆开**（最容易犯）：位置式嵌套下少一块标记 = **后面整段归属都变了** —— 删「否则」把失败分支积木**并进 then**；删「结束如果」让后面剩下的**全部落进分支里**。**两种都不报错**，只是路线照错结构跑。故控制台这样设计：
- 只有「如果」能拖，拖/点一下**自动落三行**；「否则/结束如果」灰的、不可拖不可点（单插一块立刻失衡）。拖「如果」头搬**整块**（`cfg.dragSet: ifSpanIds`），标记行**不挂** `onpointerdown`。删**任一**标记 = 连同**整块**一起删（toast 报清删了几行）；「否则」行的 ✕ = 删该分支**连同内容**。
- 头最左**倒三角 = 折叠**（**默认折叠**，只记「被展开过」的：刷新/重开不变）。折起来只剩头 + 摘要，块内与两块标记行都不渲染、落点锚消失（要放积木先展开）。⚠️ 存 `localStorage`（per 浏览器），**别挪去服务端**（视图偏好：会打到别人页面、还会被 `/api/rev` 当远端改动）。
- 「如果」头上的眼睛 = **整块**启用/禁用（块内每行一起去掉/加上行首 `//`）。⚠️ 一并清掉块内各步**原来各自**的禁用状态（toast 明说）。**不给标记行单独的眼**（只注释「如果」=「永远走 else」、注释「否则」=「永远走 then」，是**静默改变控制流**的开关）。**整块禁用是平衡的**（三块标记一起从数组消失），故「先按 `disabled` 过滤再配对」的判据必须守住，否则正常的整块禁用会被拦下。
- 分支里**可以**放分区标题，画布渲染成**缩进的子标题**而**不开新分区** —— 否则「如果」头与「结束如果」会分居两个 `<section>`，缩进与落点全乱。能罩住它的只有分区包裹（`/* */`），「包裹把配对的一半罩住」由保存校验拦下。
- **折叠只是画布的事**：块在 `map.c` 里一个字节没变，固件**完全不知道**有折叠（只按位置配对执行）。

⚠️ **三块标记上不能排「测试积木」**（`vScriptTestLoop` 长鸣拒绝，不是保险而是**必须**）：它合成的一步表配的是**真实帧**，扫描从真实路线当前位置找配对标记，把正在跑的路线**指到别处去**。

⚠️ **改完 `app_script.h` 记得 Ctrl+F5**：页面上 catalog 是加载时拉的，旧的没这三个积木（或 `STEP_IF` 没有 `cond` 参数 → 条件写成 0 = `SCRIPT_COND_NONE` = **恒成立**，分支看着"没生效"）。服务端 catalog 缓存按 mtime 键控会自失效，**浏览器那份不会**。

**保存前校验**：`map_codec.marker_balance_error`（`server.write_map` 调，错误码 `unbalanced_if`）。判据 = 「**与 `serialize_map` 实际写出的行同一套口径**」—— 被禁用（行首 `//`）与落在分区包裹里的行都不进数组、不参与配对。⚠️ 两处口径漂移会「校验放行、写出来的 C 却不平衡」，改包裹规则时必须同步。⚠️ 崩在 C 层**不会报编译错**（三块标记只是普通数组项），表现是「后半张路线静默不执行」。

**条件判据**（`emScriptCondTdf`，IF 与旧写法共用同一张表）：

- **条件分「快照」和「主动探测」两种**：
  - **快照**（`QR` / `HOSTAGE` / `COLOR` / `HEALTH_*`）：只读**最近一次**结果，瞬时、不阻塞、无副作用 → **IF 要排在对应扫描步骤紧后面**，否则读到旧战况。`HEALTH_*` 读的是握手时那次四路相机体检（`ucRpiHealthCheck`），没勾健康检查时四项为"未检查"→ 条件全部不成立（路线照常继续）。
  - **主动探测**（`LINE`）：**会阻塞并跟树莓派对话**（发 `LINE_START` → 等起流 → 数 `LINE_PROBE_MS` 窗口内有效帧 → 发 `LINE_STOP`，不动底盘）。故 **IF 不保证瞬时**（见设计规则第 4 条）。两段都是硬上界（起流段 ≤ `LINE_FIRST_FRAME_TIMEOUT_MS`，窗口段 ≤ `LINE_PROBE_MS`）；「起流失败」和「丢线」都判失败。
    - ⚠️⚠️ **必须先等起流、再开计时窗口**（`ucRpiLineProbe` 两段式）。Pi 收到 `LINE_START` 后要相机重新取流才发得出第一帧（几百毫秒），一上来就开窗口整窗一帧不来 → 判「丢线」→ IF 静默走失败分支。「Pi 还没起流」与「丢线」是**两个完全不同的状态**（`ucRpiLineRun` 注释专门记过这是最难查的失效）。起流门复用同一个 `LINE_FIRST_FRAME_TIMEOUT_MS` —— 两处用了不同值就会出现「`STEP_LINE_CONVERGE_WAIT` 成功、紧跟的 IF 却说没线」。
    - ⚠️ `LINE_PROBE_MIN_VALID` 计的是**帧数**不是采样次数（只有帧计数推进才累加，轮询 10ms / Pi 30Hz → 150ms 窗口约 5 帧）。设成 6 以上会**永远失败且不报错**。
  - 判据：人质/色球是 `status==1` **且数量达标**（与小球对位链路的「三球齐才算有效」同口径，免得 IF 说成功、下游却算不出对位量）。
  - ⚠️ **别用 `ucTakeHostageReady()` / `ucTakeColorReady()` 实现条件** —— 那是读后清零的一次性信号，IF 读一次就**把信号偷走**，紧接着的 `STEP_HOSTAGE_SCAN_WAIT` 会阻塞到超时。用非阻塞快照 `vGetHostageResult` / `vGetColorResult`。
  - **加一个条件** = `emScriptCondTdf` 加成员 + `app_script.c::s_astScriptCond[]` 加一行（按 `emCond` 字段线性查表，**别用「下标 = 枚举值」的数组**，否则中间插一个成员会把后面所有条件编号挪位）。枚举尾注释就是下拉标签，控制台侧一行不改。⚠️ 别加 `_COUNT` 哨兵成员（`parse_enums` 会当真下拉选项）。

## ⚠️ 跳到别的预设：`STEP_SCRIPT_CALL` / `STEP_SCRIPT_RETURN`（旧机制，仍可用）

**条件分支已经不走跳转了**（见上一节），这两个积木是**无条件的子程序调用** —— 用途是「多处共用同一段路线」（IF 分支是就地展开的，表达不了"共用"）。

```c
SCRIPT_STEP_I1 (STEP_SCRIPT_CALL,   "调用预设", SCRIPT_PRESET_2),   // vScriptCallPreset()
SCRIPT_STEP    (STEP_SCRIPT_RETURN, "返回"),                        // vScriptReturn()
```

- **跳转 = 调用**：子预设跑到表尾**自动跳回触发点的下一行**；也可以在里面排一块「返回」提前跳回（顶层执行「返回」= 本轮路线到此为止）。
- **所有预设都编进固件**（不再只有 `SCRIPT_PRESET_ACTIVE` 那一张），靠 `map.c` 的**预设注册表** `g_astScriptPreset[]` 按 id 查表；注册表与 `map.h` 的 `emScriptPresetIdTdf` **都由控制台生成**（`/* ==== 预设注册表/生成块 ==== */` 之间整段重写，手改会被下次保存覆盖）。控件选的就是那个枚举（`parse_enums` 扫 `Application/*.h`，**前端不用改**）。
- ⚠️ **`map.c` 改动后必须让 `parse_catalog` 缓存失效**（`server.py` 的 `/api/map` 分支做了）：`map.h` 枚举是「目标」下拉的选项来源，不失效则新建预设不进下拉，**直到重启服务端** —— 现象是「这功能没做」。
- ⚠️ **id 只增不重用**（`SCRIPT_PRESET_NEXT_ID` 单调，同 `ARM_ACTION_NEXT_ID`）：删掉预设 8 再新建，新预设拿到 **9**；`app.js::nextPresetIdent` 从服务端 `next_id` 分配，**别再按「现有最大号 +1」算** —— 复用 8 会让残留的 `SCRIPT_PRESET_8` **无声指向另一条路线**，拿到 9 则变 `undefined identifier` 的**编译期硬失败**。服务端另有一道「编号空档」校验 ——⚠️ 它**挡不住**「删最大号再新建同号」（删除要到 phase A 才落地，那刻注册表里那号还在，与「编辑了那个预设」无法区分），别以为服务端兜住了。
- ⚠️ **可被跳入的预设**（目标下拉标 `（不可跳入）`，两条都是真陷阱）：
  - **表首别放 `STEP_WAIT_HANDSHAKE`**：`ucScriptResetTarget` 的跳过逻辑只服务「重置脚本」，跳进去会**把握手重跑一遍** —— 「握手成功之后再执行一次握手」= 判为断开重连，会卡在等一个永远不来的信号上。
  - **表内别放 `STEP_SCRIPT_RESET`**：它是帧局部的，子预设会自循环、永不返回。
- **自跳 = 循环**（`调用 → 本预设`），靠 `SCRIPT_CALL_DEPTH` 兜底：到顶就**拒绝跳转、按「继续」落下去**（不是顶掉最老的帧）。第 0 格是顶层路线，所以真正能嵌套的是 **3 层**、不是 4 层。这是这套积木唯一能表达循环的写法，容易被误写。
  - ⚠️ 深度上限只挡**嵌套深度**，**不挡总工作量**：互相调用（A→B→A）在深度内是合法程序，总步数是各表步长的**乘积**（实测 6 步的表、深度 3 就能执行上千步）。车一遍遍重复动作而按键/OLED 任务照常跑 —— 表现是「车像卡住了但没死」，最难查的那种。**没有环检测**，排路线时自己当心。
    - 2026-09 起有一条**外部中止路径**（OLED 菜单的 `vScriptAbort()` → `s_ucScriptAbortReq`），中止点是**步骤边界**；巡线那一步额外在**自己的闭环循环里**也查同一个位（见 `app_line.c`）。但它**只从菜单手动触发**，没有自动环检测 —— 上面那句「排路线时自己当心」照旧成立。
- 帧栈是**静态数组、不是递归**（脚本任务栈只有 512 字），嵌套再深也不会爆栈。
- **遥测**：`script_preset_id` / `script_call_depth` 两个只读 Beam 量，「蓝牙」Tab 直接显示「现在跑到哪张表、跳到第几层」。⚠️ **嵌套 IF 不改变这两个数**：跑纯分支路线时 `call_depth` 恒 0、`preset_id` 全程不变。

## ⚠️ 中断与并发铁律（违反会死机/失控）

HAL 时基 TIM6 的 NVIC 优先级是 **15**（最低，`TICK_INT_PRIORITY`）。优先级数值 <15 的中断里调 `HAL_Delay` 或带超时的 `HAL_UART_Transmit` 都会**死锁**（TIM6 无法抢占 → `uwTick` 停走，电机保持末速）：

1. **周期业务（控制环/扫描）一律放任务**用 `vTaskDelayUntil` 固定节拍，中断只做 `...FromISR` 通知。调 FreeRTOS ISR API 的中断，优先级数值必须 ≥ `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`(5)。
2. **任务上下文延时用 `vTaskDelay`，禁用 `HAL_Delay`** —— 忙等饿死按键/OLED/传感器任务，跑路线无法急停。仅调度器启动前（`vAppInit`）可用。
3. **EMM 命令（USART3）多任务并发**：脚本与 PID 任务都会发，`vEmmV5SendCmd` 内置 mutex，发送必须经它或其上层（`vChassisMove` 等），别绕过直接调 `vUartSendArray(UART_EMM, ...)`。
4. `app_chassis.c` 全模块含 `vTaskDelay`，**仅任务上下文可调**（脚本任务 / `vPidTask`）。
5. **绝不在临界区里发串口**：`APP_TIM_PRIO_SERVO`(5) 恰等于 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`，`taskENTER_CRITICAL()` 屏蔽 TIM7 舵机与 TIM6 时基（`HAL_GetTick` 停走 → `HAL_MAX_DELAY` 超时永不触发 → **真死锁** + 舵机卡顿）。读一组值再发要**逐条读、出临界区再发**；单字读不需锁（Beam 规则第 4 条）。
   - 2026-09 补：`vUartSendArray` 的入队 `memcpy` 是临界区**唯一允许**的东西（环形缓冲互斥唯一手段，≤256B ≈ 1~2µs），但**必须只有 memcpy + 指针运算** —— 塞入 `vUartTxKick`（调 `HAL_UART_Transmit_DMA`）或背压的 `vTaskDelay` 即上面那个死锁。
   - ⚠️ 故**6 个 TX DMA 流的 NVIC 优先级必须设 6**（`app_uart.c::vUartTxDmaInitOne`）：`taskENTER_CRITICAL()` 只屏蔽**优先级数值 ≥5** 的中断，设 1~4 就能打断那个 memcpy，与 `usTxHead`/`usTxPending` 竞争 → **静默发半截数据，随机极难复现**。是正确性前提非风格问题。（取 6 非 5：与 USART 中断同级才互不抢占，避免 HAL 里非原子的 `ATOMIC_SET_BIT(CR1.TCIE)` 被覆盖。）
6. **`vUartSendArray` 不再是忙等**（2026-09 起走 TX DMA 环，见 `docs/uart.md` 的 TX 部分）：「入队即返回」，环满靠 `vTaskDelay(1)` 背压。它**仍只能在任务上下文调**，但「发大块拆多行 + 行间 `vTaskDelay(1)`」的纪律**已不需要**（背压由环满负责；`app_bt.c::bt_cmd_list` 那 135 处行间延时是历史遗留，只白花 ~130ms）。
   - 余下的忙等 TX 路径只有降级兜底（`ucTxUseDma == 0`）与 `vUartTxDrainBlocking`，都**只在任务上下文**跑。
   - ⚠️ **别新增绕过环的 TX 路径**：直接 `HAL_UART_Transmit` 会因 `gState == BUSY_TX` **静默返回 HAL_BUSY 丢帧**。`vUartPrintf` 也走 `vUartSendArray`。
7. ⚠️⚠️ **prio 6 的 ISR 里绝不能调 `HAL_UART_AbortTransmit` / `HAL_DMA_Abort` / `HAL_UART_AbortTransmit_IT`**（2026-09 加 TX DMA 时的规矩）。依据 HAL 源码：
   - 前两个在 `hdma->State == BUSY` 时进 `while((CR & DMA_SxCR_EN) != RESET){ if (HAL_GetTick() - t0 > HAL_TIMEOUT_DMA_ABORT) ... }`（`stm32f4xx_hal_dma.c:544-560`，超时常量 5ms）。HAL 时基 TIM6 是 **prio 15**，prio 6 的 ISR 里 `uwTick` **永远不涨** → 循环**永不超时 → 整个 MCU 挂死，电机保持末速**。与铁律 1 同病，只是换了个没写「Delay」字样的函数。
   - `HAL_UART_AbortTransmit_IT` 不挂，但把 `hdmatx->State` 置 `ABORT` 后**等那个可能已坏的流中断**收尾；不来则 `State` 永停 ABORT，`HAL_DMA_Abort` 遂因 `State != BUSY` 直接 `return HAL_ERROR`（`hal_dma.c:520`）—— **连任务侧看门狗都救不回来**。
   - TX 侧正确做法：**ISR 只记账 + 置 `ucTxRecoverReq`，完整 HAL 恢复交任务侧 `vUartTxRecoverIfNeeded()`**。
   - RX 侧 `HAL_UART_ErrorCallback` 里那个 `HAL_UART_AbortReceive`（**既有代码**）不违反这条：它只在 `CR3.DMAR` 置着时才调 `HAL_DMA_Abort`，而 2026-09 起那道「`hdmarx->State == HAL_DMA_STATE_BUSY` 就直接跳过 RX」的门保证了**走到它时 `State` 一定不是 BUSY** → `HAL_DMA_Abort` 在 `hal_dma.c:520` 立刻 `return HAL_ERROR`，**那个 5ms 循环可证明不可达**。别删那道门，也别顺手重写 RX 恢复。

## ⚠️ 跨模块共享状态 —— 统一走 Beam

> 移植自 pro 项目 `Function/Beam`。任何**跨模块（跨任务/跨文件）共享的状态量**一律收敛到 `Application/app_beam.c`，**禁止**在各模块头文件加 `extern` 裸全局变量。

### 什么时候必须进 Beam

- 一个量被 **≥2 个模块**（不同任务/文件）读写 —— 进 Beam。例：姿态角（传感器任务写 → OLED 读）、菜单索引/确认位（按键任务写 → OLED 读）。
- 只一个模块内部使用 —— 保持 `static`，**不要**塞进 Beam（避免变成垃圾场）。

### 规则

1. 变量唯一定义在 `app_beam.c`（`static`），外部只能经 `app_beam.h` 的 getter/setter 访问。
2. 命名遵循 QEPack 匈牙利前缀：返回值类型 `v`(void)/`f`(float)/`i`(int32_t)/`uc`(uint8_t)/`em`(enum)… + 语义名；参数 `fVal`/`iVal`/`ucVal`/`emVal`。
3. **一次性信号**用「读后自动清零」getter（如 `ucTakeMenuConfirm()`），写方用 `vSetXxx()`。
4. **并发**：Cortex-M4 单字（32 位对齐）读写原子，跨任务访问经函数调用 + FreeRTOS 任务切换即内存屏障，**当前不加锁**。仅当「多字需强一致」（如同一帧多轴数据）时才加临界区快照 getter，不要提前加锁。
   - **已知的「两个写者」例外，都要在 `.h` 里写明，别当成 bug 去『修』**：`s_ucBtEditMode`（两写者都只写 0，见 `app_beam.c` 变量处注释）、`s_ulBtCmdTick`（`vBtTask` 收指令写；`vArmPlayWait` 长播放期间逐条喂狗也写 —— `$B,PLAY` 阻塞，不喂会被 8s 空闲超时判成「PC 掉线」而退出编辑模式；两写者都写「当前 tick」，语义单调推进）。
   - **机械臂后台序列**（2026-09 加）：`arm_seq_busy` 的**唯一写者**是 `app_arm_action.c::vArmActionPump`（认领写 1、收尾写 0）；`vArmSeqAbort` **刻意不碰它**（中止后当前那一拍仍在动舵机，报忙才诚实）。注册进 `s_astBeamParam[]` 是**只读**遥测（`ucRW=0u`、`pfSet=NULL`），「蓝牙」Tab 能看到。
     ⚠️ 只读量要用 `BEAM_ADAPT_RO_U8` 之类**只读适配宏**，别用 `BEAM_ADAPT_U8` —— 后者生成用不到的 setter，AC5 报 `#177-D: declared but never referenced`。

### 新增一个跨模块量的步骤

1. `app_beam.c` 加 `static <type> s_<name>;`
2. `app_beam.h` 声明 `<前缀>Get<Name>(void)` 和 `vSet<Name>(<type> <参数>)`
3. 生产者调 setter，消费者调 getter。

### 示例（已存在）

```c
/* app_beam.h */
float   fGetAttitudeYaw(void);          /* 读姿态航向 */
void    vSetAttitude(float fRoll, float fPitch, float fYaw);  /* 传感器整帧写入 */
uint8_t ucTakeMenuConfirm(void);        /* 读后自动清零 */
void    vSetMenuConfirm(void);          /* 按键置 1 */
```

## 命名约定

- QEPack 匈牙利前缀：`v`(void 函数)、`em`(枚举)、`st`(结构体)、`pst`(结构体指针)、`f`(float)、`uc`(uint8_t)、`i`(int32_t)、`b`(布尔/标志)、`s_`(static 变量)。跨模块量不用 `g_` 裸全局，统一走 Beam 的 getter/setter。
- 应用层文件 `app_<module>.c/.h`；驱动文件 `<module>_device.c/.h`。
- 中文拼音混英文语义（`fangzhi`=放置、`sehuan`=色环等，参考 pro 项目）。

## 历轮审查记录

2026-10-06 / 2026-10-09 两轮全仓库审查与修复清单，以及「本轮明确不动」的标定值，见 `docs/reviews.md`。
