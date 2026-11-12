/**
  * @file       app_oled_menu.h
  * @brief      OLED 菜单的数据模型 —— 菜单内容在这里, 菜单行为在 app_oled.c。
  *
  * ============================ 读之前先看这三条 ============================
  *
  * ⚠️⚠️ **「==== 生成块 ====」两个标记之间的整段由 Web 控制台生成/重写**
  *     （tools/webconsole/oled_menu.py）。手改标记之间的内容会在控制台下次保存时
  *     被整段覆盖。标记**之外**是本文件的手写区, 控制台一个字节都不碰。
  *
  * ⚠️⚠️ **表是「结构」, switch 是「动词」**：控制台能编辑的是标签 / 顺序 / 启停 /
  *     指向哪个预设或子表 / 绑哪个调参量; **行为**（取值/步进/量程/离散档位/副作用/
  *     蜂鸣/前置条件）全部留在 app_oled.c 的 switch 里。别把行为往表里搬 ——
  *     三个调参量的行为各不相同, 硬凑成通用注册表会失真且更重。
  *
  * ⚠️ **屏一屏只显示 4 行**（64px / 16px, 驱动 vOledShowImage 在 Y>63 直接 return）——
  *     但一页 / 一个子菜单**最多 32 个启用项**, 放不下的靠 UP/DOWN **滚动**查看（右侧画
  *     竖直滚动条; 菜单标签宽度预算因此是 120px, 最右 8px 让给滚动条）。
  *     子菜单的「返回」由固件补在**最后一条**、随列表一起滚动, **不占**这 16 个额度。
  */

#ifndef __APP_OLED_MENU_H
#define __APP_OLED_MENU_H

#include <stdint.h>

/* ==================== 生成块（控制台整段重写） ==================== */
/* ==== 生成块: 开始 ==== */

/* 菜单项的动作码 —— **控制台「OLED」Tab 左侧积木库里的块就是它**。
 *
 * ⚠️⚠️ **尾注释 = 积木库里那个块的名字, 也就是拖进来之后默认的菜单标签**
 *     （即 OLED 上显示的那行字）。所以它写的是**用户看得见的话**, 不是给自己看的
 *     说明 —— 说明一律写在下面这张表里。
 * ⚠️ 一行一个成员: 控制台 server.py::parse_enums 是**整行锚定**的,
 *    成员跨行会被静默丢掉（不报错, 只是那个块在积木库里消失）。
 * ⚠️ 值**只增、不重排、不重用**（同 SCRIPT_PRESET_n / ARM_ACTION_n 的铁律）——
 *    复用旧值 = 某个菜单项无声地换了含义。
 *
 * 各动作码的行为（**实现全在 app_oled.c 的 switch 里, 表里只放"哪个动作"**）:
 *   NONE            空槽。不是真动作, 不出现在积木库里。
 *   NOP             占位/未实现: 只短鸣, 什么都不做。
 *   CONFIRM_SHUTDOWN 进「是/否」确认框; 框里按**两次**「是」各发一帧 "sd"（防误触）。
 *   POST_HANDSHAKE  投 SCRIPT_REQ_HANDSHAKE, 让脚本停进握手阶段 1 → 蓝牙可进编辑模式。
 *   POST_TASK       投 SCRIPT_REQ_TASK 跑一个预设; **参数 = 预设 id**（地图 Tab 里那张表）;
 *                   有二维码前置门（没设二维码 → 弹确认框）。
 *   LINE_TEST       巡线测试的开关（跑 map.c 里那个 SCRIPT_LINE_TEST_PRESET）。
 *   LASER_TOGGLE    就地翻转激光继电器; 标签运行期带「:开 / :关」。
 *   PHOTO           让 Pi 转告 OpenMV 存一张; 标签可被「拍照 成功! / 失败!」临时占用。
 *   ENTER_SUB       进子菜单; **pstSub 指向子表**（子菜单最多 32 项 —— 「返回」由固件补在
 *                   **最后一条**、随列表滚动, 它不进表, 所以生成表漏了它也退得出来）。
 *   TUNE_QTY        调参行; **参数 = 调参量 id**（见下面那个枚举）; 标签后面由固件拼上 ":值"。
 *   RUN_PRESET      跑预设; **参数 = 预设 id**。与 POST_TASK 只差一件事: **没有二维码前置门**
 *                   （台架上跑测试预设不用先去设个二维码）。同样走 SCRIPT_REQ_TASK,
 *                   所以「机械臂没展开过就先展开」那条行为一样有。
 *   ABORT_SCRIPT    中止当前脚本（走 vScriptAbort, 中止点在步骤边界）。
 *                   ⚠️ 原来菜单上**根本没有"停"的能力** —— 路线跑飞时操作员的手就在车上。
 *   GYRO_SRC        切换主/副陀螺源（ucGetGyroSrc/vSetGyroSrc）。场上的用途是主陀螺挂了
 *                   当场换; 标签运行期带「:主 / :副」。
 *                   ⚠️ 切源瞬间 yaw 会**跳到另一颗的零点** → 做绝对航向闭环的步骤会把它
 *                      当大误差。**先停车再切**（从菜单切等于已经把车交给了操作员）。
 *   SHOOT_TEST      **射靶测试**开关（2026-10-09 加, OLED「射靶测试」页第 1 项）:
 *                   与 LINE_TEST 同构, 但**参数 = 预设 id**（跑哪个"保持"预设, 由表里给）。
 *                   起跑 → 投 SCRIPT_REQ_TASK（于是白送 QR 前置门 + 「臂没展开先展开」
 *                   + 主摄预热）; 标签运行期取设备真值 ucScriptPresetActive(ucArg)。
 *   TIMER           计时: **参数 = 计时动作**（emTimerCmdTdf, 开始/结束/重置）——
 *                   控制第 1 页第 4 行的秒表。与地图 Tab 的 STEP_TIMER 积木同一套实现
 *                   （vTimerCmd）, 瞬时、不动车。
 *   GYRO_CAL        陀螺校准（2026-10 加, 投 SCRIPT_REQ_GYRO_CAL）: 校准**有效源**那颗
 *                   （gyro_src_eff —— 想校另一颗先去切「陀螺源」）。M0≈21s / HWT101≈7s,
 *                   **全程车必须静止**; 标签运行期换「校准中请勿动车」, 走完闪 1s
 *                   「校准完成!/校准失败!」（同拍照那条）; ATK 没有校准协议 = 失败。
 *   FILL_TOGGLE     就地翻转补光灯 PWM（2026-10 加, TIM9 —— 同 LASER_TOGGLE 的挂法）:
 *                   开 = FILL_LIGHT_ON_DUTY（与积木 STEP_BUGUANG_KAI 不勾亮度同一默认）,
 *                   关 = 占空比 0。标签运行期带「:开 / :关」, 真值 = Beam usGetFillDuty()
 *                   （vFillLightSet 是唯一 PWM 出口、顺手写它, 状态永远等于实物）。
 *                   ⚠️ 与任务自动补光是「后写者胜」: 任务步末的 vFillLightTaskEnd 会把
 *                      刚开的灯灭掉、这里也能关掉任务步骤开着的灯 —— 都是既有语义。
 *
 * ⚠️ 加新动作码时**只能往末尾追加**（值只增不重排不重用）。 */
typedef enum
{
    OLED_ACT_NONE = 0,          /* （空槽） */
    OLED_ACT_NOP,               /* 占位 */
    OLED_ACT_CONFIRM_SHUTDOWN,  /* 树莓派关机 */
    OLED_ACT_POST_HANDSHAKE,    /* 握手让出 */
    OLED_ACT_POST_TASK,         /* 执行任务 */
    OLED_ACT_LINE_TEST,         /* 巡线测试 */
    OLED_ACT_LASER_TOGGLE,      /* 开关激光 */
    OLED_ACT_PHOTO,             /* 拍照 */
    OLED_ACT_ENTER_SUB,         /* 子菜单 */
    OLED_ACT_TUNE_QTY,          /* 调参 */
    OLED_ACT_RUN_PRESET,        /* 跑预设 */
    OLED_ACT_ABORT_SCRIPT,      /* 中止脚本 */
    OLED_ACT_GYRO_SRC,          /* 陀螺源 */
    OLED_ACT_SHOOT_TEST,        /* 开始射靶测试 */
    OLED_ACT_TUNE_PARAM,        /* 名字调参 */
    OLED_ACT_PI_MAIN_RESTART,   /* 重启main */
    OLED_ACT_PI_SVC_RESTART,    /* 重启服务 */
    OLED_ACT_PI_LOG_CLEAR,      /* 清屏Log */
    OLED_ACT_TIMER,             /* 计时 */
    OLED_ACT_GYRO_CAL,          /* 陀螺校准 */
    OLED_ACT_FILL_TOGGLE,       /* 补光灯 */
} emOledActTdf;

/* 上面与 `OLED_ACT_TUNE_PARAM` 配套: 那一档不用 ucArg（uint8_t 塞不下参数名, 也扛不住
 * 表重排）, 改用 stOledItemTdf 末尾追加的 `pcParam` —— 一个字符串参数名:
 *   · 先在 Beam 表里找（app_beam.h::cpstFindBeamParam, 两百多项, 含只读遥测）;
 *   · 找不到再在 Pi 参数表里找（app_rpi.h::cpstRpiCfgFind, config.py 生成的那张）。
 * 两张表都不认的名字 = 该行只显示标签 + 鸣一声（**不静默**: 排菜单的人当场看得见）。 */

/* 「调参」块能绑的调参量 —— 控制台里那个「参数」下拉就是它。
 * ⚠️ 同上: 一行一个成员、只增不重用。尾注释同样是**用户看得见的短名**。
 *
 * 每个量的取值/步进/量程/副作用各不相同, **实现全在 app_oled.c 的 switch(qty)**:
 *   GYRO_TARGET  vSet/fGetLineGyroTarget，±180° 的 5 档循环（0/90/180/-180/-90）。
 *   ANGLE_TARGET vSet/fGetLineAngleTarget，±90°、步进 0.1（十分位整数域算，防浮点攒脏值）。
 *   OPENMV_LED   0~100 步进 10；改完**即调即发** "L,<n>" 给 Pi（不是存起来下次用）。
 *   SHOOT_CX     tc_cx_aim（瞄准列）/ tc_cy_aim（瞄准行, swap=0 时）—— **读 ta_xy_swap 选中的那
 *                一根**（本车 swap=1 = 瞄准列）。±5px/格、钳 [0,1920]。它是「射靶保持」环每拍
 *                现取的量 → **按一格车就跟着走到新目标**（前提是射靶测试正在跑）。
 *   SHOOT_CONV_PX tc_conv_px（保持环的收敛阈值）。±0.5px、钳 [2.0, 30.0]。
 *                ⚠️ 下限 2px = 流帧检测噪声带（±2px）+ 1rpm 顶格步长(≈1.7px/拍)的下沿,
 *                   比它们还紧会在带边界反复清零计数、永远判不成收敛。 */
typedef enum
{
    OLED_QTY_GYRO_TARGET = 0,   /* 陀螺目标 */
    OLED_QTY_ANGLE_TARGET,      /* 图像目标 */
    OLED_QTY_OPENMV_LED,        /* 补光亮度 */
    OLED_QTY_SHOOT_CX,          /* 瞄准列 */
    OLED_QTY_SHOOT_CONV_PX,     /* 收敛阈值 */
} emOledTuneQtyTdf;

/* ==== 生成块: 结束 ==== */

/* ==================================================================
 * 手写区 —— 控制台不碰
 * ================================================================== */

/* 页的类型。状态页与二维码页是**代码页**（有各自的绘制/按键逻辑, 数据表表达不了）,
 * 所以它们在页面表里只占一条没有项表的空壳; 只有 MENU 页的内容来自项表。 */
typedef enum
{
    OLED_PAGEKIND_STATUS = 0,   /* 第 1 页: 状态显示（比赛看的那页） */
    OLED_PAGEKIND_QR,           /* 第 2 页: 二维码设定（数字编辑器） */
    OLED_PAGEKIND_MENU,         /* 菜单页: 内容来自 pstList */
} emOledPageKindTdf;

struct stOledListTdf;           /* 前向声明: 项表与表本身互相引用 */

/* 一个菜单项。⚠️ 字段是**位置初始化**（AC5 没有指定初始化器）→
 *    **只能往末尾追加字段, 不许插在中间**, 否则控制台写出的每一行都要跟着改。 */
typedef struct stOledItemTdf
{
    const char *pcLabel;        /* **静态兜底标签**; 运行期标签（激光开/关等）另算 */
    uint8_t     ucAct;          /* emOledActTdf */
    uint8_t     ucEnabled;      /* 0 = 不显示、导航跳过（但仍在表里占位） */
    uint8_t     ucArg;          /* 语义由动作码定: 预设 id / 调参量 id */
    const struct stOledListTdf *pstSub;   /* 仅 OLED_ACT_ENTER_SUB 用, 其余为 0 */
    const char *pcParam;        /* 仅 OLED_ACT_TUNE_PARAM 用: 参数名（见上面那段说明）, 其余为 0 */
    const char *pcStep;         /* 仅 OLED_ACT_TUNE_PARAM 用（2026-10-18 加, **追加在末尾**）:
                                 * 自定义步进 —— 十进制数字串（如 "0.01"/"1"/"100"）,
                                 * 空串/0 = 用系统步进（Beam 按量程/20, Pi 按当前值量级）。
                                 * 存字符串而不是 float: 控制台只回吐文本, 不引入浮点字面量。 */
    const char *pcMap;          /* 仅 OLED_ACT_TUNE_PARAM 用（2026-10-18 加, **追加在末尾**）:
                                 * 下拉类参数的**显示映射/循环顺序**覆盖, 形状与 Beam 的
                                 * pcOptions 同构: "1=手|3=自"（值=短标签, | 分隔）——
                                 * 值必须是真选项表里存在的（写盘校验）; 空串/0 = 用真表。
                                 * 屏上显示短标签、LEFT/RIGHT 按映射顺序循环。 */
} stOledItemTdf;

/* 一张项表 = 项指针 + 项数。**页**与**子菜单**共用这一个类型。 */
typedef struct stOledListTdf
{
    const stOledItemTdf *pstItems;
    uint8_t              ucCount;
} stOledListTdf;

/* 一页。代码页（STATUS/QR）的 pstList 为 0。 */
typedef struct stOledPageTdf
{
    uint8_t                  ucKind;    /* emOledPageKindTdf */
    const stOledListTdf     *pstList;
} stOledPageTdf;

/* ------------------------------------------------------------------
 * 表行宏 —— 控制台按这三个形状写表行。用宏而不是裸大括号, 是为了让往返解析
 * 有可锚定的行首前缀（同 map.c 的 SCRIPT_STEP_* 那套)。
 * ⚠️ 末尾那个 0/NULL 是 **pcParam**（2026-10 加的字段）—— 显式写出来, 免得
 *    "少写一个字段"在 AC5 下无声通过（老行照旧工作, 但表行不再能一眼数出字段数）。
 * ------------------------------------------------------------------ */

/* 无参数项 */
#define OLED_ITEM(act, label, on)          { (label), (uint8_t)(act), (uint8_t)(on), 0u, 0, 0, 0, 0 }

/* 带一个字节参数的项（参数 = 预设 id / 调参量 id） */
#define OLED_ITEM_A(act, label, on, arg)   { (label), (uint8_t)(act), (uint8_t)(on), (uint8_t)(arg), 0, 0, 0, 0 }

/* 进子菜单的项。子表用**直接符号引用**而不是 id: 子表被删/改名会变成
 * `undefined identifier` 的**编译错误** —— 这是有意的硬失败, 胜过「重排一下就
 * 无声指向另一张表」。 */
#define OLED_ITEM_SUB(label, on, sub)      { (label), (uint8_t)OLED_ACT_ENTER_SUB, (uint8_t)(on), 0u, (sub), 0, 0, 0 }

/* 按**参数名**调的调参项（OLED_ACT_TUNE_PARAM）: 名字指向 Beam 表或 Pi 参数表,
 * 找不到就是一条**响亮失败**的行（见 emOledActTdf 上面那段）。控制台的调参块下拉
 * 写的就是这个名字 —— 名字比下标稳: 表重排/重生成后下标会漂, 名字不会。 */
#define OLED_ITEM_P(act, label, on, param) { (label), (uint8_t)(act), (uint8_t)(on), 0u, 0, (param), 0, 0 }

/* 同 OLED_ITEM_P, 再带**自定义步进**（2026-10-18 加; pcStep = 十进制数字串,
 * 空/0 = 系统步进）。控制台在调参行的「参数」旁多一个「步进」输入框, 填了才写这个宏。 */
#define OLED_ITEM_PS(act, label, on, param, step) \
                                           { (label), (uint8_t)(act), (uint8_t)(on), 0u, 0, (param), (step), 0 }

/* 同 OLED_ITEM_PS, 再带**显示映射**（pcMap = "1=手|3=自", 空 = 用真选项表）:
 * 下拉类参数的屏上短标签与 LEFT/RIGHT 循环顺序都按它来（2026-10-18 加）。
 * 数值/布尔行不吃它（控制台只给下拉行出「映射」按钮）。 */
#define OLED_ITEM_PM(act, label, on, param, step, map) \
                                           { (label), (uint8_t)(act), (uint8_t)(on), 0u, 0, (param), (step), (map) }

/* 项表 → stOledListTdf, **长度由 sizeof 派生**。
 * ⚠️ 控制台不写第二份计数宏 —— 手工维护的计数正是漏改之源。
 * ⚠️ 只能用在真数组上（不能用在指针上, 那会算成指针大小）。 */
#define OLED_LIST(arr)                     { (arr), (uint8_t)(sizeof(arr) / sizeof((arr)[0])) }

/* ------------------------------------------------------------------
 * 表本体（定义在 app_oled_menu.c 的「数据区」生成块里）
 * ------------------------------------------------------------------ */
extern const stOledPageTdf g_astOledPage[];
extern const uint8_t       g_ucOledPageNum;

#endif /* __APP_OLED_MENU_H */
