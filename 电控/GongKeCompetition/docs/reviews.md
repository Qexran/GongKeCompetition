# 历轮全仓库审查记录（2026-10-06 / 2026-10-09 / 2026-10-31）

> 由根 `CLAUDE.md` 拆分；章节标题保持原样（源码注释按节名引用）。
> 相关：根 `CLAUDE.md`（全局铁律）、`docs/` 其余拆分文档

> 📌 **2026-10-06 全仓库代码审查 + 修复**：报告在 `C:\Users\Administrator\Desktop\GKS代码审查报告_2026-10-06.md`，
> 覆盖本仓库与 Pi 端 `\\192.168.<网段>.125\shared\Desktop\GKS_Competition`（那边逐文件的修改前副本在
> `_bak-review-20261006/`，**那台机器上 git 基线是几周前的、不可用来回滚**）。
> 本次改动（都已编译通过 0 error / 0 warning，控制台 6 套测试全绿）：
> ① 标靶流控「距离辅助」永远不动车（`app_rpi.c` 的 `ucMmFromDist`）；
> ② 「测试」积木能测 `SCRIPT_CALL/RETURN` → 帧栈泄漏（`app_script.c` 已加进拒绝名单）；
> ③ 蓝牙编辑模式「脚本没在跑」这条入场路径是死的（`app_bt.c::bt_edit_active` + `vBtTask` 的空闲看门狗）；
> ④ 上电头 60s 不能自动重启 Pi（`s_ulSysCmdTick` 初值）；
> ⑤ 三处「等结果帧」+ 两处「等起流」循环补中止位；⑥ 按键事件先清后响 + `emKeyEvent` 加 volatile；
> ⑦ Beam 的多字快照与「读-清」加临界区 + `vGetLastRpiPacket` 拷贝式读；⑧ TX 看门狗时间戳进临界区；
> ⑨ `sanitize_tag` 幂等性（`a/(/b` 会产出含 `//` 的标签）；⑩ 写盘保住原换行风格（CRLF）；⑪ `write_meta` 校验换算闭集；
> ⑫ `_budget_from_ms` 地板不再越过单片机给的时限（Pi）。
> **只改文档/注释、没动值**的（都是现场标定过的）：`TARGET_SERVO_CHASSIS_EN/…`（见「底盘参与闭环」节）、
> `ARM_SERVO_CURVE=2`（见「S 曲线」节）、`TARGET_CALIB_TIMEOUT_MS=8000`（积木 I(0) 现在真生效，0=用蓝牙量）。

> 📌 **2026-10-09 全仓库第二轮审查 + 修复**（控制台 536 项测试全绿；固件无本地编译器，改动均为
> 局部小改 + 括号配平核对，**上 Keil 后请全量重编译确认**）：
> ① `vChassisMoveWorldDiagLF/RF` 象限符号 3 分支 180° 反向（DiagLF 的 case1/default、DiagRF 的 case1；
>   原来靠"调用点恰在 H≈0"兜住）；
> ② app_uart.c TX 起发失败的两处退账进临界区（与 ErrorCallback 的 ISR 退账互斥，杜绝 pending 下溢→memcpy 越界）；
> ③ RX「ABORT 卡死」重装失败时把 State 写回 ABORT 让下个 IDLE 重试（原来非 RPI 路会永久聋）；
> ④ app_beam.c 8 个 take 型读-清信号补临界区（同 ucTakeMenuConfirm 模式）；chassis_accel 量程 2000→255 + setter 钳位
>   （消费端是 uint8_t，>255 原会静默回绕）；
> ⑤ app_pid.c：fMaxOut=0 的参数组不再继承上次的 230 限幅、vPidSetMotion 五字段装载进临界区；
>   **世界航向记账统一为"永远写指令目标"**（ucPidMove / vPidSpinTurn / 支点转弯三处, 车主确认的设计原则）——
>   全工程只用 90° 档转弯, map 里"80 多度"是标定值（物理结果就是 90°）, 世界航向是 90° 逻辑网格的
>   记账器（存在的意义就是排路线时不必想"现在该沿哪根轴走"）; 写实测反而让每弯残差渗进累加器,
>   而查表按 ±45° 取整、几度残差天然被吸收; IMU 卡死有双陀螺仪热备兜底。⚠️ 例外：巡线收尾的落点
>   仍按**实测**写（app_rpi.c 的 line walk / pivot+line 收尾 —— 线才是航向基准, 不在 90° 网格记账范围）。
> ⑥ servo_device.c 多字段指令用 `ucUpdating` 标志 + ISR 跳一拍（QEPack 不引 RTOS 依赖）；
>   app_rpi.c 待发单槽的"顶掉/登记/终审清槽"三处进临界区；
> ⑦ 控制台：tuning/server 两个 meta 端点的用户文本落盘前过 map_codec 清洗（`*/`/换行原来能写坏 C 源文件）、
>   meta 端点支持可选 base_version 乐观锁（前端已带 /api/rev 戳）、oled_menu.py 标识符白名单 + 事务回滚罩住
>   OLED_Data.c（字节级快照）、arm_action 删预设改"先 commit 后进回收站" + 显式 id 查重、
>   server 500 打 traceback + Handler 30s 读超时、raw 值过滤换行；
> ⑧ P3：bt_cmd_play 整数性校验、测试积木"找不到参数"由长鸣照跑改为拒绝、EXT 暂存 64B→整圈、
>   gyro/atk 超时比较改有符号差值（防 tick 回绕）、atk 帧头重同步不再丢 0x55、read_reg 按容量截断、
>   app_config.h「组动作角度偏移」段头合并为单行（跨行段头会让该组在调参 Tab 消失）。
> **本轮明确不动**（车主确认）：`ARM_SERVO_CURVE=0` 是主动改的（以文件实际值为准，注释可能滞后）；
> `LINE_CE_ANG_TH_GYRO(0.5) < LINE_GYRO_DEADBAND_DEG(1.0)` 已知先不修。

> 📌 **2026-10-31 全仓库第三轮审查 + 修复**（五路并行深查：固件新功能/固件基础设施/控制台
> Python/控制台前端/跨端一致性；**9 套 unittest 全绿 641 项**（新增 4 条回归测试），
> app.js 过 node --check + 顶层桩执行。固件本地无编译器，改动均为局部小改 + 括号配平核对，
> **上 Keil 后请全量重编译确认**）：
> **固件**：
> ① `vChassisActionPlay` 的 ARM_WAIT 误读 `ausDur`（恒 0）→ 表里的「等臂上限 usDur」从没生效过，
>    恒吃 60s 兜底（改读 `p->usDur`，新增 `CH_ACT_ARM_GRACE_MS` 等臂认领宽限防同刻空放）；
> ② 预校验跳过的 MOVE 行（wx/wy=0 新建行最典型）把 NaN 注入平移和 → 整组平移无声归零
>    （④ 叠加循环加 `afPeak==0 continue`，顺带挡掉跳过段的 hold 登记）；
> ③ 三个新「读-清」信号补临界区（`ucTakeChActPreviewReq/StopReq/ucTakeScriptReq`，
>    2026-10-09 ④ 同类缺口；CSTOP 被吞 = 刹车必达承诺破产）；
> ④ 置忙提前到播放器顶 + 预校验拒绝路径退账 + 预览任务先占坑（关掉 resolve→播放之间
>    CPUSH 改写草稿的窗口）；⑤ 转弯防故障兜底钟只给**本拍权威段**计时（排队段不再被扣
>    「4s 没转完」）；⑥ MOVE accel=0 估时与下发同口径（0=跟随全局档，注释同步改）；
> ⑦ 单段估时 uint16 钳满防回绕；⑧ 蜂鸣「陀螺损坏」看门狗豁免陀螺校准期（校准不出帧是
>    预期，原来每次校准必白响一声失败音）；⑨ 支点段输出吃 `PIVOT_TURN_YAW_SIGN`（符号链
>    与 vPidPivotTurn 同源）；⑩ s_ucSpinTurnCore 补回负侧归一化循环；⑪ `ulTxReqBytes +=`
>    进临界区；⑫ CIT 超范围/NaN 前置挡掉（(int) 转换 UB 出路径）；⑬ 不存在的播放 id
>    长鸣+日志（同 .h 承诺），删无人调用的 `ucChActExists`；⑭ 播放器置忙纪律。
> **控制台 Python**：
> ⑮ `chassis_action._render_item` 臂事件渲染出 `ARM_ACT_11u`（非法 C 标识符，Keil 编译错 +
>    自家解析器读不回）—— 保存一条「臂动作」即写坏 `app_chassis_action.c`（改 `%d` + 补
>    edited-arm 往返测试 + bt_link 五 kind 的 `$B,CIT` 行构造测试）；
> ⑯ `_active_line_count`（arm/chact 两份）只扣 `//` 行 —— 手写块注释行被数进 usCount =
>    固件越界读表对垃圾出舵机指令（改按条目宏形状过滤 + 测试）；
> ⑰ oled 显示映射短标签不洗 `"`/`\`/换行/注释符（可写断 C 字符串）；act 白名单扩到
>    「无 raw」条目（原来只查 edited，外部 payload 可绕过）；页面表认不出的行改抛（原来
>    静默删）；⑱ base_version 非数字：write_meta 两处原来**静默跳过比对**（乐观锁被绕过）、
>    write_config/delete_params 原来 500 —— 统一 400 门；⑲ arm/chact/calib 三个 dispatch
>    透传 HTTP 状态码（409 不再被吞成 200 —— 机械臂 Tab 的「↺ 重载」提示条靠它触发）；
> ⑳ moves.add_var 初始值验数字（原来 `abc` 会按 RAW 字面量写进 app_config.h）；moves/switches
>    引用扫描跳过分区包裹行；switches._load 复检名字白名单、int(id) 给 400；bt_link 半包缓冲
>    封顶 + set_param 剥 CR/LF；bt_serial.refresh_ports 的 PowerShell 挪出锁；map_codec
>    write_text_like 改 open(newline="")（3.8 兼容）；calib int 转换给贴切 400 文案；
> **控制台前端（app.js/style.css）**：
> ㉑ 底盘动作组标题/备注**任何路径都存不进去**（change 只 markDirty 不写 state，blur 后被
>    回灌旧值）—— 改 input 直写 state（同 arm/calib）；㉒ 地图积木 ✎ 跳机械臂编辑首次必弹
>    假错误（ARM.loaded 入口即置真，改成共享 loading promise 并 await）；㉓ Ctrl+S 按 Tab
>    分发（原来在任意 Tab 都存 map.c）；㉔ 六个写盘函数加防重入（双击第二个 POST 必吃 409
>    弹误导提示）；㉕ 标定旧预设缺新参数显示 "undefined"（h() 的 value 分支补守卫）；
> ㉖ 杂项：editInPlace 值没变不置脏、位移画布/调参排序拖拽补 pointercancel 与窗口失焦兜底、
>    loadOled/loadArm 补 catch+toast、save/armSave 的 reloadCatalog 单独兜底（保存成功不该
>    报「保存失败」）、删除位移变量成功不用 err 红样式、`#toast.ok`/`#pane-calib .btn.tiny`
>    补 CSS、chact 离开 Tab 时 busy 文案标注、revBlocked 罩住五个弹窗、删 `CHACT_LANE` 死码；
> **跨端/字库/文档**：
> ㉗ app_config.h「爆爆桶对位」段头改单行（16 个 BUCKET_ALIGN_* 原来在调参 Tab 被静默归进
>    「平移射靶」组）；962 行空段头改普通注释（消除「自定义开关」重复段头）；
> ㉘ check_coverage 目标扩到 app_beam.c/app_rpi.c（Beam 参数中文名与 Pi 选项标签都是 OLED
>    取材来源）→ 一次性补 339 字重生成字模（OLED_Data.c 159→498 字，覆盖检查通过）；
> ㉙ map.c/map.h 预设 25 标题孤括号修为「过减速带（倒退顶住）」（全角括号配对才能活着过
>    sanitize —— 半角 `(` 不配对会被控制台保存剥掉，表头注释与注册表行三处同步改）；
> ㉚ CLAUDE.md 工程宏更正为 `USE_HAL_DRIVER,STM32F407xx`；uvprojx 欠账补上
>    app_chassis_action.c（与 app_arm_action.c 同欠）；「九个 Tab」→十一个；引用计数改为
>    量级表述；app_chassis_action.h 的 2026-10-09 ⑤ 指针改指 docs/reviews.md；
>    srccheck 的 DOM 桩补 window.addEventListener。
> **本轮明确不动**：`bt_cmd_list` 135 处行间 `vTaskDelay(1)`（文档已标注删不删都行，留待
> 车主顺手清）；字库里 `名`/`现` 两字暂无界面使用（64 字节，留作余量）；F6 草稿窗口的
> 残余缝隙（resolve→进函数几条指令，两侧写同值 1，同 Beam 规则第 4 条口径）；转弯段交叠
> 排法仍未禁止（兜底钟已改权威计时，是否允许交叠待车主定案）。
>
> 📌 **2026-11-01 附：OLED 菜单「翻到亮度往下卡死 / 从第一项 UP 卡死」——真因与修复**（现场报告）:
> 症状是**整屏完全不刷新**（不是高亮消失）。根因在字模查表，不在菜单引擎：
> `vOledShowChinese()`（`Core/Src/OLED.c`）查表下标原来是 **`uint8_t pIndex`**，而字模表
> 已经涨到 **500 条** —— 只要某个「字」**没在前 256 条里找到**，`for` 循环到 255 就回绕成 0，
> **永远扫不完 → OLED 任务挂死、整屏冻结**。两个现场触发点正好对得上：
> ① 往下翻过「亮度」（第 4 行）后窗口滚出**「理想Δx」**：Δ(U+0394) 是 **2 字节 UTF-8**，
>    `ucOledDrawMixed` 却一律按 3 字节切，查找键变成 `CE 94 78`（Δx）→ 查不到 → 死循环；
> ② 从第 1 项按 UP 回绕到「返回」，窗口滚到底、首次画出**「执行反向看路」**：「反」在
>    字模表**下标 498** → 同样扫不到 → 死循环。
> 修复：① `vOledShowChinese` 的 `pIndex` 改 **`uint16_t`**（两处查表循环共用它）;
> ② `ucOledDrawMixed` 按 UTF-8 **首字节判 2/3/4 字节**（不再一律 3，避免吞掉后面的 ASCII）;
> ③ `Δ` 进 `EXTRA_WIDE` + `glyphs.txt` 并重生成字模（`理想Δx/Δy` 正常显示，不再是方框）。
> 之前那版「固件旧于 5543dac、重烧即可」的判断是**错的**：带「看路调试」的固件必然已经
> 有滚动引擎（表与引擎同属 f260422），且症状是死机级的整屏冻结，不是「高亮消失」。
> ⚠️ 引申纪律: **字库过了 255 条之后**，任何「下标往 0..255 里塞」的写法（`uint8_t` 循环变量、
> 按下标遍历字模表）都是死循环级隐患；另外控制台自动补字是**追加到表尾**，所以常用字
> 很容易落在 ≥256 的下标 —— 别以为「常用字一定在前面」。
> （溯源: 把字模表推过 255 的正是 2026-10-31 那轮 ㉘ 的一次性补 339 字（159→498）——
> 补字前表只有 159 条, uint8 循环恰好扫得到哨兵, 雷一直潜伏。全工程遍历字模表的只有
> `OLED.c` 这两个循环, 已核无第三处。）
>
> 📌 **2026-11-01 附：新建预设「可重入的任务」勾不了**（现场报「执行反向看路」）: 真 bug,
> 在前端。`addPreset`/`dupPreset` 只给 ident（`g_astScriptPresetN`）不给 id, 保存成功的
> 响应也不回填 id, 而勾选框判 `cur.id == null` → 新建的预设从建好到保存后都是灰的, 只有
> 「重载」/刷新从磁盘读回 id 才能勾。修 = 抽 `presetIdOf()`（id 优先, 否则解析 ident 尾号 ——
> 服务端 `_preset_entries` 给合规 ident 分的 id 恰是该号）, 勾选框显示/点击两处与
> `presetToken` 都走它; `test_bt.py::TestPresetIdOf` 钉住（node 真跑 + 源码断言）。
> 服务端回路（勾选→map.h `（可重入）`+注册表 `1u`→重读）沙箱实测本来就是好的。
