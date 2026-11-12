/**
  * @file       app_script.c
  * @brief      脚本步骤表模块实现（移植自 pro 项目 Motion/script.c）。
  *
  *             vScriptRun() 是核心：遍历步骤表, 按 eType 分发到对应底层原语。
  *             每个 case 只调一个动作大函数（步骤设计规则见 app_script.h 顶部
  *             注释 / CLAUDE.md「脚本动作表设计规则」）。
  *             底盘位置移动原语（vChassisMoveWorld*）内部自带阻塞等待到位，
  *             故执行器无需状态机/异步机制。
  */

#include "app_script.h"
#include "app_config.h"      /* emm_v5_device.h + UART 映射 + 动作参数宏 */
#include "app_chassis.h"     /* vChassisMove / vChassisMoveWorld* / 弧线转弯 */
#include "app_chassis_action.h"  /* vChassisActionPlayWait（STEP_CHASSIS_ACTION_PLAY_WAIT 原语） */
#include "app_pid.h"         /* ucPidMove / vPidTurn / vPidMoveAndTurn 等 */
#include "app_arm.h"         /* vArmGroupMove* / vArmSetZhuaShou */
#include "app_arm_action.h"  /* vArmPlayWait / vArmPlayNoWait / vArmSeqAbort */
#include "app_cali.h"        /* vCaliRun（空壳） */
#include "app_rpi.h"         /* ucRpiHandshake / vRpiSendTaskCmd / ucRpiLineConverge */
#include "app_sensors.h"     /* ucGyroCalActiveSource（SCRIPT_REQ_GYRO_CAL） */
#include "app_actuator.h"    /* vRelayOn / vRelayOff / vLaserShoot / vFillLightSet */
#include "app_beam.h"        /* ucTakeMenuConfirm 一次性信号 */
#include "app_bt.h"          /* vBtArmEditHold（STEP_ARM_EDIT_WAIT 让位手动操纵机械臂） */
#include "map.h"             /* g_pstScriptActive 当前预设路线表（map.c 多预设） */
#include "FreeRTOS.h"
#include "task.h"

/* ==================================================================
 * OLED「执行任务」的机械臂展开前置条件
 * ================================================================== */

/* 「机械臂待命」那条**三段式展开序列**（从初始位置伸出 → 伸展机械臂 → 旋转到待命位置）。
 * 机械臂没展开过时, 跑任务预设之前先阻塞播它一遍。
 *
 * ⚠️ 用 ARM_ACT_1 而不是 ARM_ACT_12（阻塞版本）: 两者是同一串位姿, 而这里显式走
 *    vArmPlayWait（本来就阻塞）, 所以**不需要**靠预设自己的播放方式 ——
 *    也就不会因为有人把 ARM_ACT_1 的播放方式改成什么而改变行为。
 * ⚠️ 换一条展开序列就改这一行; 但记得同步 app_arm_action.c::vArmNoteDeployed 里
 *    那个「哪些动作算已展开」的名单 —— **两处必须对上**, 否则会出现
 *    「播了展开序列却没置位 → 每次都白跑一遍」或者「置了位却没真展开」。 */
#define SCRIPT_ARM_STANDBY_ACT   ARM_ACT_1

/* 「开始巡线测试」（OLED 第 4 页那个开关）跑的路线预设。
 * 默认 = map.c 的 SCRIPT_PRESET_2「测试巡线闭环」—— 走预设而不是直接调
 * ucRpiLineConverge 原语, 是为了能在控制台「地图」Tab 里给这条测试**
 * 自由加步骤**（激光、支点转弯…预设 2 里那几行注释掉的就是干这个的）。
 *
 * ⚠️⚠️ **预设 2 的开头两步必须删掉**（握手 / 等 MAIN）, 否则「开始巡线测试」
 *    按下去只会停在那两步上不动 —— 那正是 2026-09 踩过的「点了没反应」。
 *    在「地图」Tab 里删这两步:
 *      ① `等待握手`        （STEP_WAIT_HANDSHAKE —— Pi 已开机时它永远等不到 SYNC_HS_1）
 *      ② `不动车, 等 MAIN` （STEP_DEAD）
 *    删完记得 Ctrl+F5 再保存, 免得浏览器里那份旧路线把手改冲掉。
 *
 * ⚠️ 另一个后果（改之前想清楚）: 那一步的 **gyro_target 会在每次按下时写回 Beam**
 *    （ucRpiLineConverge 一进函数就写 line_gyro_target —— **仅当全局 LINE_CONV_SRC=1/3**;
 *    相机/不修两档一个字节都不写）——
 *    所以「先在第 4 页把陀螺目标角调好、再按开始」会被预设里的值覆盖; 正确顺序是
 *    **先按开始、再边跑边调**（跑起来之后调的值实时生效, 蓝牙/OLED 都一样）。
 *    ⚠️ src/cam_pct 已全局化（2026-10-30）, 不再有"积木参数覆盖权重"这回事。 */
#define SCRIPT_LINE_TEST_PRESET   SCRIPT_PRESET_2

/**
  * @brief       「巡线测试」那条预设是不是**正在跑**
  * @retval      1 = 是
  *
  * 给 OLED 第 4 页那个「开始/停止巡线测试」开关用。⚠️ 判据必须是
  * 「脚本在跑 **且** 跑的就是这条测试预设」，不能只看 `script_running` ——
  * 那个对**任何**路线都是 1（开机路线、菜单启动的任务预设…），拿它当"巡线测试在跑"
  * 会让标签一直显示「停止巡线测试」而实际跑的是别的，操作员一按就成了
  * 「莫名其妙把当前任务掐掉」（2026-09 现场踩过：标签卡在「停止」、还挡住了
  * 「没设二维码」那道提示）。
  *
  * ⚠️ 预设 id 由 SCRIPT_LINE_TEST_PRESET 决定, 只有本文件知道 —— 所以这个判断
  *    留在这里做成公开函数, 别把常量抄到 app_oled.c 去。
  * ⚠️ `ucGetScriptPresetId()` 是**顶层帧**的 id（`vScriptRun` 进场时置、弹栈时还原），
  *    而菜单这条测试就是当顶层路线跑的, 所以正好对得上。
  */
uint8_t ucScriptLineTestActive(void)
{
    return ucScriptPresetActive((uint8_t)SCRIPT_LINE_TEST_PRESET);
}

/**
  * @brief       某个预设是不是**正在当顶层路线跑**（设备真值, 给 OLED 的开关标签用）
  * @param       ucPresetId  emScriptPresetIdTdf 的值
  * @retval      1 = 是
  *
  * 2026-10-09 从 `ucScriptLineTestActive` 抽出来的通用壳：OLED 那两个「开始/停止XX测试」
  * 开关（巡线测试 / 射靶测试）都要「脚本在跑 **且** 跑的正是我这条预设」这个判据，而
  * **ID 由菜单表项的参数带过来**（`OLED_ITEM_A(..., ucArg)`）—— 所以 app_oled.c 不必
  * 硬编码任何预设号（比巡线那条更干净：那条的常量还留在这里，因为它的 id 是宏）。
  *
  * ⚠️ 判据不能只看 `ucGetScriptRunning()` —— 那对**任何**路线都是 1。
  * ⚠️ `ucGetScriptPresetId()` 是**顶层帧**的 id（`vScriptRun` 进场时置、弹栈时还原）；
  *    菜单启动的测试就是当顶层路线跑的, 所以正好对得上。
  * ⚠️ 函数定义在 `ucScriptLineTestActive` 之后、但被它调用 —— .h 里有原型, 没问题。
  */
uint8_t ucScriptPresetActive(uint8_t ucPresetId)
{
    return (uint8_t)((ucGetScriptRunning() != 0u)
                     && (ucGetScriptPresetId() == ucPresetId));
}

/* ==================================================================
 * 组合小流程（脚本内部 / 单动作抓取原语共用）
 * ================================================================== */

/* 等 MAIN 键确认: 让出式轮询 Beam 一次性信号（读后自动清零）。
 * 公开版（2026-11 加）: 单动作抓取的「单步确定」在 app_rpi.c 里也调它 ——
 * STEP_DEAD/STOP 与抓取阶段边界必须是**同一个按键语义**（读后清零、一个键放一段）。
 *
 * ⚠️⚠️ **除了 MAIN 还要认「中止请求」**，否则这里就是菜单请求的死结: 脚本任务停在
 *    这个循环里等键, **没人去取请求槽**（取请求的是 vScriptTask 的循环, 而它正卡在这里）
 *    → 菜单上点了没反应, 只有回第 1 页按一下 MAIN 才有动静。
 *    2026-09 实测踩过: `默认路线` 的第 2 步就是 `STEP_DEAD`, 开机后一直停在这儿,
 *    「开始巡线测试」/「执行任务」**全部**堵住 —— 现象正是「点了没反应, 回主界面按 MAIN 才动」。
 *    （同一个坑在 `ucRpiHandshake` 阶段 1、`vLineFollowConverge` 里也各踩过一次, 那两处
 *    已经补过出口; 这里是第三个 —— 凡是「等外部事件」的循环都要带上这一条。）
 * ⚠️ 出场后**不用自己收拾**: `vScriptRun` 的循环顶部会看到同一个（电平）中止位,
 *    以「整条路线结束」收尾 —— 与「中止点是步骤边界」那条纪律一致。
 * ⚠️ MAIN 只在**真的按下**时被消费（短路求值让它当第一个操作数）, abort 不会顺手把
 *    操作员那次按键吃掉。 */
uint8_t ucScriptWaitKeyConfirm(void)
{
    while ((!ucTakeMenuConfirm()) && (ucGetScriptAbortReq() == 0u))
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return (ucGetScriptAbortReq() == 0u) ? 1u : 0u;
}

/* 停车等键: 底盘清速, 等 MAIN 键继续 */
static void vScriptParkWaitKey(void)
{
    vChassisMove(0, 0, 0);
    (void)ucScriptWaitKeyConfirm();
}

/* 重置脚本的目标步: 表首第一条「实质」步骤 —— 跳过开头**连续**的 STEP_WAIT_HANDSHAKE / STEP_DEAD。
 *
 * 为什么跳过这两类: 它们只在整机上电后第一次有意义 ——
 *   · 握手: 树莓派只在启动阶段重发 SYNC_HS_1（ACK 到手即停, 见 app_rpi.c::ucRpiHandshake），
 *     而 ucTakeHandshakeSeen 是**唯一没有周期性生产者**的一次性信号（置位点只有 SYNC_HS_1 解析）。
 *     重跑这一步要么落进 while(!ucTakeHandshakeSeen()) 等一个永远不来的信号, 要么（链路
 *     已 OK 时）**主动判为断开重连** —— 两种都会让「每圈回到表首」的路线卡死在等待里,
 *     所以表首那份必须跳过。
 *   · 等键: 那是给操作员把握车放好用的，重跑路线时不该再等一次。
 *
 * ⚠️ 只跳**表首连续**的这两类: 表中间排的 DEAD 照旧会等键 —— 那正是「台架每圈按一次键」的写法。
 * ⚠️ STEP_STOP 不跳: 它会停车+清速, 语义比单纯等键重, 排在哪由路线作者决定。
 * 整表都是这两类（异常表）→ 回 0, 不越界。 */
static uint16_t ucScriptResetTarget(const stScriptStepTdf *pstTable, uint16_t usCount)
{
    uint16_t us = 0;

    while ((us < usCount)
           && ((pstTable[us].eType == STEP_WAIT_HANDSHAKE)
               || (pstTable[us].eType == STEP_DEAD))) {
        us++;
    }

    return (us < usCount) ? us : 0;
}

/* ==================================================================
 * 脚本执行器
 * ================================================================== */

/* 这一步是不是「机械臂」步骤（决定要不要先掐掉后台臂序列）。
 * 单独拎出来是为了让它能放在 switch **之外** —— 见 vScriptRun 里的说明。 */
static uint8_t c_bIsArmStep(emScriptStepTypeTdf eType)
{
    switch (eType)
    {
    case STEP_ARM_GROUP_WAIT:
    case STEP_ARM_GROUP_NO_WAIT:
    case STEP_SET_ZHUASHOU_WAIT:
    case STEP_ARM_PLAY_WAIT:
    case STEP_ARM_PLAY_NO_WAIT:
    case STEP_TARGET_CALIB_WAIT:        /* 末尾会动舵机6/3, 属臂指令 */
    case STEP_TARGET_SERVO_WAIT:        /* 闭环期间**持续**动舵机6/3, 更属臂指令（开了 ts_chassis_en 后还动车） */
    case STEP_ARM_EDIT_WAIT:            /* 让位给 PC 点动: 先掐掉后台臂序列, 否则两边抢舵机 */
    case STEP_BOMB_GRAB_WAIT:           /* 排爆抓取: 末尾要播动作预设, 同样先掐后台序列 */
    case STEP_HOSTAGE_RESCUE_WAIT:      /* 反恐任务: 同上（还要按二维码选预设） */
    case STEP_BALL_GRAB_ONE_WAIT:       /* 色球单动作抓取: 抓取 vArmPlayNoWait（place≠0 时还有放置） */
    case STEP_HOSTAGE_GRAB_ONE_WAIT:    /* 人质单动作抓取: 抓取 vArmPlayNoWait（place≠0 时还有放置） */
        return 1u;

    default:
        return 0u;
    }
}

/* ==================================================================
 * IF 条件注册表（STEP_IF 的 cond 参数）
 * ================================================================== */

/* 「恒成立」: 没有条件 —— 用来表达「无条件跳转」, 也方便台架上单测跳转本身。 */
static uint8_t c_ucCondAlways(void)
{
    return 1u;
}

/* 二维码结果: 判据取 **100~399** —— 不是「只要 iQr >= 0」。
 * 因为二维码的三位数字含义是 `百位=目标色(1红/2绿/3蓝) + 后两位序号`: `007` 之类的
 * 值 ucRpiWaitQrResult 也算扫成功, 但下游根本用不上（对位链路用的就是
 * 同一个 100~399 范围）。这里跟它保持一致, 免得 IF 说「成功」、下游却算不出对位量。
 * -1 = 无结果/失败/超时（ucRpiWaitQrResult 的约定）。
 * 快照型 —— 读的是**上一次扫描**留下的值, 瞬时、不阻塞、无副作用。 */
static uint8_t c_ucCondQr(void)
{
    int32_t iQr = iGetQrCode();

    return (uint8_t)((iQr >= 100) && (iQr <= 399));
}

/* 人质识别: status==1 **且数量达标**才算成立（数量齐了才真的用得上）。
 * ⚠️ 不能拿 ucTakeHostageReady() 判 —— 那是「读后清零」的一次性信号, 唯一合法消费者是
 *    ucRpiWaitHostageResult。在这里读一次就等于**把信号偷走**, 紧接着的
 *    STEP_HOSTAGE_SCAN_WAIT 会一直阻塞到超时。用非阻塞快照 vGetHostageResult。 */
static uint8_t c_ucCondHostage(void)
{
    uint8_t ucStatus = 0u;
    uint8_t ucCount  = 0u;

    vGetHostageResult(&ucStatus, &ucCount, NULL);
    return (uint8_t)((ucStatus == 1u) && (ucCount >= (uint8_t)HOSTAGE_MAX_COUNT));
}

/* 色球识别: 同上, 数量按 COLOR_MAX_COUNT 算 —— 与小球对位链路（ucRpiAlignChain）的
 * 「三球齐才算有效」保持同一口径, 免得 IF 说成功、下游却算不出对位量。 */
static uint8_t c_ucCondColor(void)
{
    uint8_t ucStatus = 0u;
    uint8_t ucCount  = 0u;

    vGetColorResult(&ucStatus, &ucCount, NULL);
    return (uint8_t)((ucStatus == 1u) && (ucCount >= (uint8_t)COLOR_MAX_COUNT));
}

/* 巡线是否在线上: **主动探测** —— 短暂起流 → 采样 → 收流。
 * ⚠️ 这是唯一一个**会阻塞**的条件（LINE_PROBE_MS + 两条指令的发送时间）, 并且会跟
 *    树莓派对话。车不动。所以 STEP_IF 不保证是瞬时的, 见 app_script.h 的说明。 */
static uint8_t c_ucCondLine(void)
{
    return ucRpiLineProbe((uint32_t)LINE_PROBE_MS);
}

/* 相机健康检查结果（2026-11 加）: 只读 Beam 快照, **不阻塞、不重新检查**。
 * 没勾健康检查 / 检查被中止时四项是"未检查"(2), 这些条件全部不成立 ——
 * 于是「不读取」与「读取但没检查」的行为一致（路线照常继续）。 */
static uint8_t c_ucCondHealthFail(void)
{
    uint8_t ucM, ucQ, ucPi, ucMv;

    vGetHealthResult(&ucM, &ucQ, &ucPi, &ucMv);
    return ((ucM  == RPI_HEALTH_FAIL) || (ucQ  == RPI_HEALTH_FAIL)
         || (ucPi == RPI_HEALTH_FAIL) || (ucMv == RPI_HEALTH_FAIL)) ? 1u : 0u;
}

static uint8_t c_ucCondHealthMFail(void)
{
    uint8_t ucV = RPI_HEALTH_UNKNOWN;

    vGetHealthResult(&ucV, NULL, NULL, NULL);
    return (ucV == RPI_HEALTH_FAIL) ? 1u : 0u;
}

static uint8_t c_ucCondHealthQFail(void)
{
    uint8_t ucV = RPI_HEALTH_UNKNOWN;

    vGetHealthResult(NULL, &ucV, NULL, NULL);
    return (ucV == RPI_HEALTH_FAIL) ? 1u : 0u;
}

static uint8_t c_ucCondHealthPiFail(void)
{
    uint8_t ucV = RPI_HEALTH_UNKNOWN;

    vGetHealthResult(NULL, NULL, &ucV, NULL);
    return (ucV == RPI_HEALTH_FAIL) ? 1u : 0u;
}

static uint8_t c_ucCondHealthMvFail(void)
{
    uint8_t ucV = RPI_HEALTH_UNKNOWN;

    vGetHealthResult(NULL, NULL, NULL, &ucV);
    return (ucV == RPI_HEALTH_FAIL) ? 1u : 0u;
}

/* 「自定义」Tab 管理的开关条件求值函数 —— 说明:
 * 每行 = 一个开关: 直接返回 app_config.h「自定义开关」段的 SW_* 宏
 * （0/1, 编译期 —— 控制台上切换只是改宏, Keil 重编译烧录才上车）。
 * 行尾注释 = 开关中文名（与 app_script.h 生成块成员的尾注释一致）。
 * ⚠️ 下面这对**单行**标记之间由控制台整段重写（手改会被覆盖）。 */
/* ==== 自定义条件 getter 生成块: 开始 ==== */
static uint8_t c_ucCondSw108(void)   /* 色球反向 */
{
    return SW_COLOR_TASK_REVERSE;
}
static uint8_t c_ucCondSw109(void)   /* 标靶反向 */
{
    return SW_TARGET_TASK_REVERSE;
}
static uint8_t c_ucCondSw110(void)   /* 人质反向 */
{
    return SW_HOSTAGE_REVERSE;
}
static uint8_t c_ucCondSw111(void)   /* 左边封路 */
{
    return SW_DISABLED_LEFT_ROAD;
}
static uint8_t c_ucCondSw113(void)   /* 色球与人质交换 */
{
    return SW_REVERSE_COLOR_AND_HOSTAGE;
}
static uint8_t c_ucCondSw114(void)   /* 二维码板与人质台交换 */
{
    return SW_REVERSE;
}
static uint8_t c_ucCondSw115(void)   /* 人质放置到爆爆桶+抓取色球 */
{
    return SW_SWAP_HOS_AND_BALL_FUNCTION;
}
/* ==== 自定义条件 getter 生成块: 结束 ==== */

typedef struct {
    emScriptCondTdf emCond;
    uint8_t       (*pfProbe)(void);
} stScriptCondTdf;

/* 查找按 emCond 线性扫, **不用「下标 = 枚举值」的数组** —— 那样中间插一个成员就会把
 * 后面所有条件的编号静默挪位（枚举值一改, 老路线里的条件全变了意思）。
 * 加条件 = emScriptCondTdf 加一个成员 + 这里加一行, 控制台下拉自动跟着变。 */
static const stScriptCondTdf s_astScriptCond[] = {
    { SCRIPT_COND_NONE,    c_ucCondAlways  },
    { SCRIPT_COND_QR,      c_ucCondQr      },
    { SCRIPT_COND_HOSTAGE, c_ucCondHostage },
    { SCRIPT_COND_COLOR,   c_ucCondColor   },
    { SCRIPT_COND_LINE,    c_ucCondLine    },   /* ← 只有这一行会阻塞 */
    /* 相机健康检查结果（2026-11 加, 读握手体检的快照, 瞬时） */
    { SCRIPT_COND_HEALTH_FAIL,    c_ucCondHealthFail   },
    { SCRIPT_COND_HEALTH_M_FAIL,  c_ucCondHealthMFail  },
    { SCRIPT_COND_HEALTH_Q_FAIL,  c_ucCondHealthQFail  },
    { SCRIPT_COND_HEALTH_PI_FAIL, c_ucCondHealthPiFail },
    { SCRIPT_COND_HEALTH_MV_FAIL, c_ucCondHealthMvFail },
    /* ⚠️ 下面这对**单行**标记之间由「自定义」Tab 整段重写（手改会被覆盖）:
     * 每行 = 一个自定义开关的注册（emCond + 上面 getter 生成块里的同名函数）。
     * 查找按 emCond 线性扫 —— 生成块行混在同一个数组里, 查询侧一行不用改。 */
    /* ==== 自定义条件注册生成块: 开始 ==== */
    { SCRIPT_COND_SW_108, c_ucCondSw108 },   /* 色球反向 */
    { SCRIPT_COND_SW_109, c_ucCondSw109 },   /* 标靶反向 */
    { SCRIPT_COND_SW_110, c_ucCondSw110 },   /* 人质反向 */
    { SCRIPT_COND_SW_111, c_ucCondSw111 },   /* 左边封路 */
    { SCRIPT_COND_SW_113, c_ucCondSw113 },   /* 色球与人质交换 */
    { SCRIPT_COND_SW_114, c_ucCondSw114 },   /* 二维码板与人质台交换 */
    { SCRIPT_COND_SW_115, c_ucCondSw115 },   /* 人质放置到爆爆桶+抓取色球 */
    /* ==== 自定义条件注册生成块: 结束 ==== */
};

#define SCRIPT_COND_NUM  (sizeof(s_astScriptCond) / sizeof(s_astScriptCond[0]))

/* 求条件值: 1=成立 0=不成立（IF 的「失败」分支就是 0）。
 * 认不出的条件回 0（保守: 宁可落进「失败→继续」也别乱跳）。 */
static uint8_t ucScriptCondTrue(emScriptCondTdf emCond)
{
    uint8_t ucI;

    for (ucI = 0u; ucI < (uint8_t)SCRIPT_COND_NUM; ucI++) {
        if ((s_astScriptCond[ucI].emCond == emCond)
            && (s_astScriptCond[ucI].pfProbe != NULL)) {
            return s_astScriptCond[ucI].pfProbe();
        }
    }

    return 0u;
}

/* ==================================================================
 * 脚本帧栈 —— 「跳到别的预设执行」= 调用, 跑到表尾/「返回」自动跳回
 * ================================================================== */

typedef struct {
    const stScriptStepTdf *pstTable;
    uint16_t               usCount;
    uint16_t               usIdx;
    uint8_t                ucPresetId;   /* 遥测用; 0 = 未知 / 不在注册表里 */
} stScriptFrameTdf;

/* ⚠️ 静态数组, **不是递归**: 脚本任务的栈只有 512 字（Core/Src/freertos.c），
 * 真递归会把 chassis → EMM 那条本来就很深的调用链吃穿。
 * SCRIPT_CALL_DEPTH 帧 × 12 字节 ≈ 48 字节 BSS, 可忽略。 */
static stScriptFrameTdf s_astScriptStack[SCRIPT_CALL_DEPTH];
static uint8_t          s_ucScriptDepth = 0;

/* 按 id 查注册表（map.c 的 g_astScriptPreset[]）。找不到返回 NULL。 */
static const stScriptPresetTdf *cpstFindScriptPreset(emScriptPresetIdTdf emId)
{
    uint16_t usI;

    /* 手写 map.c 把 target 写超 255 时, (uint8_t) 截断会静默折算到别的预设
     * （如 260 → 4）—— 先按表存储宽度(ucId 是 u8)拒绝, 走「找不到」的拒绝路径。 */
    if (((int)emId < 0) || ((uint32_t)(int)emId > 255u)) {
        return NULL;
    }

    for (usI = 0u; usI < g_usScriptPresetNum; usI++) {
        if (g_astScriptPreset[usI].ucId == (uint8_t)emId) {
            return &g_astScriptPreset[usI];
        }
    }

    return NULL;
}

/* 反查一张表是注册表里的哪一条。顶层预设也要报对 id, 否则遥测永远是 0 看不出所以然;
 * 查不到 = 0, 本身就是诊断信息（注册表漏了正在跑的那张表）。 */
static uint8_t ucScriptPresetIdOf(const stScriptStepTdf *pstTable)
{
    uint16_t usI;

    for (usI = 0u; usI < g_usScriptPresetNum; usI++) {
        if (g_astScriptPreset[usI].pstSteps == pstTable) {
            return g_astScriptPreset[usI].ucId;
        }
    }

    return 0u;
}

/* 跳到目标预设执行（= 调用）。返回 1 = 跳了; 0 = 没跳（「继续」/ 认不出 / 到深度上限）,
 * 调用方按「继续」落下去即可。
 *
 * ⚠️ 到 SCRIPT_CALL_DEPTH 是**拒绝跳转**而不是顶掉最老的帧: 拒绝的后果是「这一步没生效、
 *    流程照旧往下走」, 可预期; 顶帧会让两个帧的进度互相踩, 行为没法描述。
 * ⚠️ 目标表 usCount==0 / 指针为空也拒绝 —— 注册表与 sizeof 万一不同步, 不该变成越界读。 */
/* ---- 「机械臂已展开」前置（2026-10-18 统一入口）----
 * 注册表 ucFlags bit0（「可重入的任务」勾选框 → map.c 注册表第 5 列）说了算:
 *   · 带标记 → 机械臂没展开过就阻塞播一遍「机械臂待命」（同一次上电只播一次,
 *     之后 ucGetArmDeployed 恒 1 直接过）—— OLED「执行任务」的三个任务预设与
 *     被STEP_SCRIPT_CALL 调入的任务预设走**同一条逻辑**, 谁都不用担心
 *     "进来时臂是什么姿势"。
 *   · 没带标记 → 什么都不做（跑的人自己安排臂姿 —— 例如纯看路/巡线类预设）。
 * 返回 1 = 臂已就绪; 0 = 无法就绪（待命动作缺失/播完仍没置位）——调用方自行决定
 * 拒绝还是继续（两处调用点都会把失败报给 Pi 日志, 不静默）。
 * ⚠️ 阻塞播待命前先占住「脚本在跑」位（vScriptRun 要到后面才置位）—— 蓝牙的
 *    编辑模式进场条件（脚本没在跑 ⇒ 允许 $B,MODE,1）不然会把这段判成闲,
 *    操作员一点进来就和正在伸出的臂抢舵机。失败时自己清 0。 */
static uint8_t ucScriptEnsureArmDeployed(const stScriptPresetTdf *pst)
{
    if ((pst->ucFlags & SCRIPT_PRESET_FLAG_REENTRANT) == 0u) {
        return 1u;                         /* 未标记: 不自动展开（保持各表自主） */
    }
    if (ucGetArmDeployed() != 0u) {
        vRpiSendDebug("ARM deploy: skip (deployed=1)");
        return 1u;                         /* 已展开过: 直接过 */
    }
    if (ucArmActionExists(SCRIPT_ARM_STANDBY_ACT) == 0u) {
        vRpiSendDebug("ARM standby action missing");
        return 0u;                         /* 待命动作不在注册表（控制台删过/改过号） */
    }

    vRpiSendDebug("ARM deploy: playing standby");
    vSetScriptRunning(1u);                 /* 阻塞展开期间占住「在跑」位（见上） */
    vArmPlayWait(SCRIPT_ARM_STANDBY_ACT);

    if (ucGetArmDeployed() == 0u) {
        vSetScriptRunning(0u);
        vRpiSendDebug("ARM standby incomplete");
        return 0u;
    }
    vRpiSendDebug("ARM deploy: done");
    return 1u;                             /* 播完动作层自己置位, 调用方接着跑 */
}

static uint8_t c_bScriptCallPreset(emScriptPresetIdTdf emTarget)
{
    const stScriptPresetTdf *pst;

    if (emTarget == SCRIPT_PRESET_NONE) {
        return 0u;                                  /* 「继续」: 显式的不跳 */
    }
    if (s_ucScriptDepth >= (uint8_t)(SCRIPT_CALL_DEPTH - 1)) {
        return 0u;                                  /* 到顶: 按「继续」落下去 */
    }

    pst = cpstFindScriptPreset(emTarget);
    if ((pst == NULL) || (pst->pstSteps == NULL) || (pst->usCount == 0u)) {
        return 0u;
    }

    /* ---- 可重入任务: 调入前先确保机械臂已展开（与 OLED「执行任务」同一条逻辑）----
     * 展不开（待命动作缺失/播完仍没置位）→ 不调入, 按「继续」落下去 —— 与
     * 「到顶拒绝跳转」同一条既有语义, 失败原因已打给 Pi 日志。 */
    if (ucScriptEnsureArmDeployed(pst) == 0u) {
        return 0u;
    }

    s_ucScriptDepth++;
    s_astScriptStack[s_ucScriptDepth].pstTable   = pst->pstSteps;
    s_astScriptStack[s_ucScriptDepth].usCount    = pst->usCount;
    s_astScriptStack[s_ucScriptDepth].usIdx      = 0u;
    s_astScriptStack[s_ucScriptDepth].ucPresetId = pst->ucId;

    vSetScriptPresetId(pst->ucId);
    vSetScriptCallDepth(s_ucScriptDepth);

    return 1u;
}

/* STEP_SCRIPT_CALL 的原语: 无条件跳到目标预设, 跑完自动跳回下一行。 */
static void vScriptCallPreset(emScriptPresetIdTdf emTarget)
{
    (void)c_bScriptCallPreset(emTarget);
}

/* STEP_SCRIPT_RETURN 的原语: 从被跳进来的子预设提前返回。
 * 实现成「跳到本帧末尾」而**不是**自己写一套「弹栈 + 父帧 usIdx++」—— 返回规则只留一份
 * （循环顶部那条唯一的收尾路径）, 抄第二份早晚漂移。
 * 顶层（深度 0）执行时 = 本轮路线到此为止, 与跑到表尾同义。 */
static void vScriptReturn(void)
{
    s_astScriptStack[s_ucScriptDepth].usIdx = s_astScriptStack[s_ucScriptDepth].usCount;
}

/* ==================== 嵌套容器 IF（STEP_IF / STEP_ELSE / STEP_END_IF） ==================
 * 三块标记在**扁平数组**里靠位置配对, 执行器**不需要栈、也不需要任何状态**: 条件不成立时
 * 就从下一条开始往后扫, 数着同层嵌套找到配对的那块标记, 把本帧的 usIdx 挪过去即可。
 *
 * 语义（用户可见的说明在 app_script.h 的参数映射注释里, 这里只写执行规则）:
 *   · 条件成立 → 什么都不做, 落进 then 分支一路执行; 走到同层「否则」时再收在「结束如果」之后。
 *   · 条件不成立 → 有「否则」就落到它之后, 没有就落到「结束如果」之后（两者都是「配对标记 +1」）。
 *
 * ⚠️ 扫到表尾还没找到配对 → usIdx 落在 usCount（或更大）, 主循环的 `usIdx >= usCount` 就
 *    把本帧收掉 —— **绝不越界读**。控制台保存前会拦下不配对的路线（错误码 unbalanced_if）,
 *    所以这里只是兜底（手改 map.c 才会走到）。
 * ⚠️ **位置式配对是刻意的**: 它没有任何跨步骤的状态, 于是分支里排 STEP_SCRIPT_RESET
 *    （只是把 usIdx 指回本表表首）与 STEP_SCRIPT_CALL（压帧; 返回时父帧 usIdx 正好停在
 *    分支内部那条之后）都照常工作。改成「进入时压栈、离开时弹栈」的实现就没有这条免费性质了。
 * ⚠️ 因此这三块标记**不能**用「测试积木」去测: vScriptTestLoop 合成的一步表配的是**真实帧**,
 *    扫描会从真实表的当前位置开始找配对标记, 把正在跑的路线**指到别处去**。已在那边拒绝。 */

/* 从 usFrom（**必须已经越过**发起这次扫描的那块标记, 所以初始深度 0）向后扫, 找同层的第一块
 * 目标标记。emWant 只允许两个取值:
 *   STEP_ELSE    → 同层的「否则」或「结束如果」, 谁先遇到就返回谁（「如果」不成立时用);
 *   STEP_END_IF  → 只认同层的「结束如果」（落在「否则」之后的 then 分支收尾时用)。
 * 找不到（表不平衡）返回 usCount。 */
static uint16_t usScriptMatchMark(const stScriptFrameTdf *pstFr, uint16_t usFrom,
                                  emScriptStepTypeTdf emWant)
{
    uint16_t us = usFrom;
    uint8_t  ucDepth = 0u;

    while (us < pstFr->usCount) {
        emScriptStepTypeTdf emT = pstFr->pstTable[us].eType;

        if (emT == STEP_IF) {
            ucDepth++;                                  /* 内层: 它的标记一律不算数 */
        } else if (emT == STEP_END_IF) {
            if (ucDepth == 0u) {
                return us;                              /* 同层结束: 两个 emWant 都收在这儿 */
            }
            ucDepth--;
        } else if (emT == STEP_ELSE) {
            if ((ucDepth == 0u) && (emWant == STEP_ELSE)) {
                return us;                              /* 同层的「否则」: 只有找 else 时才收 */
            }
        } else {
            /* 普通步骤, 与配对无关 */
        }
        us++;
    }

    return pstFr->usCount;
}

/* STEP_IF 的原语: 条件成立什么都不做（落进 then 分支）; 不成立跳到同层的「否则」之后
 * （没有「否则」就是「结束如果」之后）。⚠️ 主循环在调本函数前已经 usIdx++, 所以
 * s_astScriptStack[..].usIdx 现在正是「如果」的**下一条**。 */
static void vScriptIf(emScriptCondTdf emCond)
{
    stScriptFrameTdf *pstFr = &s_astScriptStack[s_ucScriptDepth];

    if (ucScriptCondTrue(emCond) == 0u) {
        pstFr->usIdx = (uint16_t)(usScriptMatchMark(pstFr, pstFr->usIdx, STEP_ELSE) + 1u);
    }
}

/* STEP_ELSE 的原语: then 分支自然落下来到这里 → 跳过整段 else 分支, 收在「结束如果」之后。 */
static void vScriptElse(void)
{
    stScriptFrameTdf *pstFr = &s_astScriptStack[s_ucScriptDepth];

    pstFr->usIdx = (uint16_t)(usScriptMatchMark(pstFr, pstFr->usIdx, STEP_END_IF) + 1u);
}

/* STEP_END_IF 的原语: 空实现 —— 它只是给上面两处扫描一个配对锚点。
 * 保留成函数（而不是在 case 里直接 break）是为了让控制台能显示原语名 vScriptEndIf,
 * 与另两块标记对称。 */
static void vScriptEndIf(void)
{
}

/* ==================== 步骤执行器（供主循环与「测试」积木共用一份） ==================== */

/* 「调试二维码」的值清洗（STEP_QR_SET_TEST）: 每位必须 1~3
 * （百位=目标色 1红/2绿/3蓝, 二位=靶色, 三位=序号）。
 * 合法 → 原样返回; 非法 → -1（= 无结果的约定值, 下游一律按「没扫到」处理, 绝不会
 * 拿一个没有意义的颜色码当真）。
 * ⚠️ 单独拎成函数是为了让 case 体里**第一个函数调用**仍是 vSetQrCode ——
 *    控制台取积木原语名读的就是它（server.py::parse_catalog 的 `\w+\s*\(`）。 */
static int32_t c_iQrTestSanitize(int32_t iCode)
{
    int32_t iA = (iCode / 100) % 10;
    int32_t iB = (iCode / 10) % 10;
    int32_t iC = iCode % 10;

    if ((iA >= 1) && (iA <= 3)
        && (iB >= 1) && (iB <= 3)
        && (iC >= 1) && (iC <= 3))
    {
        return iCode;
    }

    return -1;
}

static void vScriptExecStep(const stScriptStepTdf *p, stScriptFrameTdf *pstFr);   /* 与下面互调 */

/*
 * STEP_TEST_LOOP_WAIT 的原语: 把**选中的那块积木**反复执行, 每轮跑完两短声并等 MAIN 键
 * 继续 —— 「跑一轮 → 看结果 → 再跑一轮」, 不用为测一个动作去改路线。
 *
 * 实现: 按选中积木的类型**合成一张只有它的一步表**, 借 vScriptExecStep 在其上下文里执行
 * 一次 —— 延时/阻塞等到位/分支/臂序列中止这套语义与前向路线**完全同一份**, 不是第二套
 * 简化版进程。
 *   · 参数整条照抄「已知的那一块」（选的是**类型**, 不带参数）: 先在**当前表**里找,
 *     找不到就**扫整个预设注册表**（含未运行的那些路线）。同类型多处出现时取最靠前的一处。
 *   · ⚠️⚠️ 一处都找不到 → 参数只能是全 0, 而**对带超时/距离/角度的积木, 0 是致命的**:
 *     `STEP_BALL_ALIGN_WAIT(0)` = 发完指令立刻超时返回, 表现成「Pi 明明识别成功、单片机
 *     一直报失败」那种最难查的假故障（真踩过）。所以这种情况**发一条调试串说明并拒绝**,
 *     别让它静默地跑一堆空动作。要正常测就随便在哪张路线里放一块同类型的积木。
 *   · rounds: 0 = 不限（无限循环, 断电/复位退出）; N>0 = 跑满 N 轮后往下走。
 *
 * ⚠️ 防呆（拒绝, 一步都不跑, 调试串留痕）:
 *   · 选「测试」自己 → 合成表里再调本函数, 一层层压脚本任务栈（只有 512 字）→ 必爆栈;
 *   · 选「重置脚本」→ 它是**帧局部**的, 会把索引打到表头, 变成不让出的紧循环;
 *   · 选嵌套 IF 的三块标记（如果/否则/结束如果）→ 它们是**位置配对**的, 扫描用的是
 *     **真实帧**（合成表里只有一块标记, 配不成对）—— 于是会从真实路线当前位置开始找配对
 *     标记, 把正在跑的那张表**指到别处去**。这条**不是保险, 是必须的**: 少了它,
 *     台架上测一次「如果」就能让路线静默跳段;
 *   · 选「调用预设」/「返回」→ 它们是**帧栈**操作, 而弹帧只发生在 `vScriptRun` 的循环里
 *     （本函数走的是 vScriptExecStep, 不弹帧）: 测「调用预设」会压一帧且永不弹回 →
 *     测试跑完后 `vScriptRun` 把**被跳入的那张表**当成当前帧继续跑, 后半张路线被静默
 *     丢弃; 测「返回」更是直接把**真实帧**的 usIdx 打到表尾 = 正在跑的路线就此结束;
 *   · 选哨兵 STEP_COUNT（类型参数的下拉自动从枚举派生, 哨兵也在里面）。
 *   要「每轮等键」不必另配积木: 选中 STEP_DEAD 就行（它自己就等键）。
 * 仅任务上下文调用（内部 vTaskDelay）。 */
static void vScriptTestLoop(emScriptStepTypeTdf emStep, uint32_t ulRounds)
{
    stScriptStepTdf   stOne = { STEP_DELAY, NULL, {0}, {0} };   /* 合成的那一步, 先给全 0 参数 */
    stScriptFrameTdf *pstFr = &s_astScriptStack[s_ucScriptDepth];
    uint16_t us;
    uint16_t usP;
    uint32_t ulRound;
    uint8_t  ucFound = 0u;

    if ((emStep == STEP_TEST_LOOP_WAIT) || (emStep == STEP_SCRIPT_RESET)
        || (emStep == STEP_IF) || (emStep == STEP_ELSE) || (emStep == STEP_END_IF)
        || (emStep == STEP_SCRIPT_CALL) || (emStep == STEP_SCRIPT_RETURN)
        || (emStep >= STEP_COUNT))
    {
        return;
    }

    stOne.eType = emStep;

    /* ① 先在**当前表**里找同类型的那一块（最贴近"我正在编辑的这张路线"） */
    for (us = 0u; us < pstFr->usCount; us++)
    {
        if (pstFr->pstTable[us].eType == emStep)
        {
            stOne = pstFr->pstTable[us];   /* 整条照抄（含参数; 结构体 1:1 复制） */
            ucFound = 1u;
            break;
        }
    }

    /* ② 当前表里没有（典型的: 测试预设只放了「测试」积木自己）→ 扫整个预设注册表。
     *    与「跳到别的预设」同一张表, 含未运行的那些路线 —— 于是「小球对位」能拿到
     *    正式路线里标定好的超时值, 而不是全 0。 */
    if (ucFound == 0u)
    {
        for (usP = 0u; usP < g_usScriptPresetNum && ucFound == 0u; usP++)
        {
            const stScriptStepTdf *pstT = g_astScriptPreset[usP].pstSteps;
            uint16_t usN = g_astScriptPreset[usP].usCount;

            for (us = 0u; us < usN; us++)
            {
                if (pstT[us].eType == emStep)
                {
                    stOne = pstT[us];
                    ucFound = 1u;
                    break;
                }
            }
        }
    }

    /* ③ 整个工程里都没这种积木 → 参数只能全 0。带超时/距离/角度的积木会「立即失败」或
     *    「不动」, 而现象看着完全不像参数问题 —— 调试串报出来并**拒绝执行**
     *    （与 vScriptTask 各拒绝分支的「留痕=拒绝」惯例对齐, 2026-10-09 改; 原来报完
     *    之后照跑全 0 参数, 正是函数头描述的那种假故障形态）。 */
    if (ucFound == 0u)
    {
        static const char acNoParm[] = "TESTLOOP: no params found (all-zero)";
        vRpiSendDebug(acNoParm);
        return;
    }

    for (ulRound = 0u; ; )
    {
        vScriptExecStep(&stOne, pstFr);    /* 本轮: 走的就是路线里那套执行语义 */
        ulRound++;
        if ((ulRounds > 0u) && (ulRound >= ulRounds))
        {
            break;
        }
        (void)ucScriptWaitKeyConfirm();           /* 等 MAIN 键继续下一轮 */
    }
}

/* 步骤 → 任务补光类别（app_actuator.h 的 FILL_TASK_*; 开/关由 vScriptExecStep 统一做,
 * 配置在 app_config.h「任务补光」区）。映射粒度 = **步骤**: 要靠相机看东西的
 * 步骤（识别/对位/校准/抓取/射击/主摄扫码）全在这里, 纯行驶/延时/流程类一概不算 ——
 * 灯跟着「相机要看」走, 不跟着「任务区到了」走（开过去路上的行驶段不亮灯）。
 * ⚠️ 新增视觉步骤时记得来这里挂一行, 漏了只是那一步没补光, 不会有任何报错。 */
static uint8_t ucScriptFillTaskOf(emScriptStepTypeTdf eType)
{
    switch (eType)
    {
    /* 看路: 主摄找地面黄区（对位/校准两步） */
    case STEP_ROAD_ALIGN_WAIT:
    case STEP_ROAD_YAW_WAIT:
        return FILL_TASK_ROAD;

    /* 色球: 识别/对位/单动作抓取; 排爆也是色球区那套识别, 一并算 */
    case STEP_COLOR_SCAN_WAIT:
    case STEP_BALL_ALIGN_WAIT:
    case STEP_BALL_GRAB_ONE_WAIT:
    case STEP_BOMB_GRAB_WAIT:
        return FILL_TASK_BALL;

    /* 射靶: 标靶识别/对位/校准/流控 + 平移射靶/射靶保持（OLED 测试页走同一映射） */
    case STEP_TARGET_SCAN_WAIT:
    case STEP_TARGET_ALIGN_WAIT:
    case STEP_TARGET_CALIB_WAIT:
    case STEP_TARGET_SERVO_WAIT:
    case STEP_TARGET_SHOOT_WAIT:
    case STEP_TARGET_SHOOT_HOLD_WAIT:
        return FILL_TASK_SHOOT;

    /* 人质: 识别/对位/单动作抓取/反恐 */
    case STEP_HOSTAGE_SCAN_WAIT:
    case STEP_HOSTAGE_ALIGN_WAIT:
    case STEP_HOSTAGE_GRAB_ONE_WAIT:
    case STEP_HOSTAGE_RESCUE_WAIT:
        return FILL_TASK_HOSTAGE;

    /* 二维码: **只挂主摄**扫码这一步。扫码副摄（STEP_QR_SCAN_WAIT）朝向另一边,
     * 灯装在机械臂上的主摄旁照不到它的画面 —— 挂上去只会白亮一盏灯, 所以故意不在此表。 */
    case STEP_QR_SCAN_MAIN_WAIT:
        return FILL_TASK_QR;

    default:
        return FILL_TASK_NONE;
    }
}

/* 执行**一个**步骤 —— 从 vScriptRun 的循环里抽出来, 与「测试」积木复用同一份:
 * 延时/阻塞等到位/分支跳转/臂序列中止这套语义只有这一处实现, 不会漂移。
 * ⚠️ 抽出来的唯一目的是复用 —— 别把「只有主循环才该做」的东西（帧栈推进/步进）塞进来。
 * pstFr = 该步骤所在的帧（STEP_SCRIPT_RESET 要改它的 usIdx; 其余步骤只读 p）。 */
static void vScriptExecStep(const stScriptStepTdf *p, stScriptFrameTdf *pstFr)
{
    /* 任务补光: 任务步骤全程点亮补光灯, 出了这一步必灭（见函数收尾）。
     * 放在 switch 之外与 vArmSeqAbort 同理 —— 控制台取积木原语名读的是 case 体里
     * **第一个**函数调用（server.py::parse_catalog）, 写进 case 体积木名就废了;
     * 收尾也集中在这一个出口, 各 case 不散着写, 中止/超时路径一样能灭灯。 */
    uint8_t ucFillTask = ucScriptFillTaskOf(p->eType);
    uint16_t usFillDuty = 0u;   /* 本步实际点亮的占空比（‰, 0=没点）, 收尾对称打调试串用 */

    /* 便捷宏: F(n)=afF[n], I(n)=anI[n] (仅在此 switch 块内有效) */
#define F(_i) (p->afF[_i])
#define I(_i) (p->anI[_i])

    /* 任何机械臂步骤之前, 先掐掉后台的臂序列（vArmPlayNoWait 投递、vArmTask 推进）。
     * 规则 = 后写者胜 + 新指令显式终止旧序列: 不掐的话后台会一边继续写舵机、
     * 一边与本步抢, 动作会烂掉。
     * ⚠️ **必须放在 switch 之外**: 控制台「地图」Tab 取积木原语名的方式是读
     *    case 体里**第一个**函数调用（server.py::parse_catalog），把 abort 写进
     *    case 体第一行会让积木显示成 vArmSeqAbort 而不是真正的动作函数。
     * ⚠️ 也别把 abort 下沉进 app_arm.c 的原语里: 后台泵正是经那些原语推进的,
     *    那样会自己把自己中止掉。 */
    if (c_bIsArmStep(p->eType)) {
        vArmSeqAbort();
    }

    /* 任务补光开灯。vRpiSendDebug 只认静态字符串（拼不了数值）, 这两句是现场排障的
     * 分岔点: 跑任务步骤**没有** "FILL: on" = 车上还是旧固件; 有 "FILL: on" 而灯不亮
     * = 逻辑没问题, 查灯/驱动（20kHz PWM 对光耦模块太快的坑, 见 app_actuator.c）。 */
    if (ucFillTask != FILL_TASK_NONE)
    {
        usFillDuty = vFillLightTaskBegin(ucFillTask);
        if (usFillDuty != 0u)
        {
            vRpiSendDebug("FILL: on");
        }
    }

    switch (p->eType) {

    /* ===== 电机 ===== */
    case STEP_EMM_ENABLE:
        vChassisEnableAll((uint8_t)I(0));
        break;

    case STEP_EMM_CTRL_MODE:
        /* 切换环模式: 下拉只放 1=开环 / 2=闭环（emEmmV5CtrlModeTdf 的 0/3 是"关脉冲输入引脚"
         * 与"En 复用多圈限位", 会改引脚功能, 不给排路线的人碰）。
         * 只发模式帧、**不写驱动器 Flash** → 掉电回原模式（有意, 理由见 app_chassis.c）。
         * 瞬时积木（发帧即完）, 按设计规则第 4 条不带后缀; 建议排在**停车时**。 */
        vChassisSetLoopModeAll((emEmmV5CtrlModeTdf)I(0));
        break;

    /* ===== 底盘 ===== */
    case STEP_CHASSIS_MOVE_NO_WAIT:
        vChassisMove((int)I(0), (int)I(1), (int)I(2));
        break;

    case STEP_CHASSIS_STOP:
        vChassisMove(0, 0, 0);
        break;

    case STEP_CHASSIS_MOVE_WORLD_WAIT:
        /* 世界系平移; I(0)=是否叠加上次「路径残留偏移」(人质单动作抓取留下的世界系位移,
         * 一次性: 用完即清零)。追加在 int 末尾, 老行(纯 F2)缺的尾槽补 0 = 不叠加, 不用重写。 */
        vChassisMoveWorld(F(0), F(1), (uint8_t)I(0));
        break;

    case STEP_CHASSIS_MOVE_WORLD_X_WAIT:
        vChassisMoveWorldX(F(0), (uint8_t)I(0));
        break;

    case STEP_CHASSIS_MOVE_WORLD_Y_WAIT:
        vChassisMoveWorldY(F(0), (uint8_t)I(0));
        break;

    case STEP_CHASSIS_MOVE_WORLD_DIAG_LF_WAIT:
        /* 世界系斜向平移（2026-10 由「斜左上/斜右上」两块合并成一块）:
         * F(0)=位移[mm]（带符号, 负=沿所选方向反向）,
         * I(0)=斜向{0=左上,1=右上,2=左下,3=右下} —— 追加在 int 末尾, 老行(F1)缺槽
         * 补 0 = 斜左上 = 原来那块的正向, map.c 一行不用改。方向分派在
         * vChassisMoveWorldDiag（app_chassis.c）, 那边注释里有象限对账。 */
        vChassisMoveWorldDiag((uint8_t)I(0), F(0));
        break;

    case STEP_CHASSIS_SET_SPEED_ACCEL:
        vChassisSetWorldSpeedAccel((int)I(0), (int)I(1));
        break;

    case STEP_CHASSIS_SET_ACCEL:
        vChassisSetAccel(I(0));
        break;

    case STEP_CROSS_RAMP_WAIT:
        vChassisCrossRampWait(F(0), (int)I(0));   /* 首行即动作函数: 控制台取本体内第一个函数调用作积木原语名 */
        break;

    case STEP_CHASSIS_ACTION_PLAY_WAIT:
        /* 播放底盘动作组(时间轴速度混合): 平移段叠加/陀螺闭环转弯段/臂事件按时间轴
         * 连播, 阻塞到整组走完。I(0)=动作组(CH_ACT_n)。
         * ⚠️ (emChActIdTdf) 强转必须与 I(0) **同一行** —— 控制台靠它给「动作组」派生下拉
         *    （同 STEP_ARM_PLAY_WAIT 的 emArmActionIdTdf）。
         * ⚠️ 刻意**不进** c_bIsArmStep: 预中止会掐掉前面 STEP_ARM_PLAY_NO_WAIT(delay_ms)
         *    排下的延时投递（"边走边等 X ms 后展臂"正是本步骤要并行的东西）, 组内的
         *    臂事件自己走 vArmPlayNoWait 的后写者胜, 不需要外面代管。 */
        vChassisActionPlayWait((emChActIdTdf)I(0));
        break;

    /* ===== PID 闭环运动（边走边转系） ===== */
    case STEP_PID_MOVE_WAIT:
        ucPidMove((int)I(0), (int)I(1), 0, (uint32_t)I(2), F(0), (int)I(3));
        break;

    case STEP_PID_MOVE_AND_TURN_WAIT:
        vPidMoveAndTurn((int)I(0), (int)I(1), F(0));
        break;

    case STEP_PID_TURN_WAIT:
        vPidTurn(F(0));
        break;

    case STEP_BACK_TO_ROAD_WAIT:
        vBackToRoad(F(0));
        break;

    case STEP_PIVOT_TURN_WAIT:
        /* 支点转弯: 只驱动单侧双轮, 陀螺仪闭环到目标角（方向 I(0), 角度 F(0), 绝对 I(1)）。
         * I(2)=rev 反向执行(2026-11-01 加): 转向不变, **驱动侧换对侧且速度取反**（左转:
         * 右侧正转 ⇒ 左侧倒转）—— 支点换边、车心弧线镜像, 车倒着放/场地换边时勾;
         * 对侧与速度同时取反 ⇒ 闭环仍稳定, 见 app_pid.c::s_ucPivotTurnCore 头注释。
         * int 尾槽追加, 老行缺槽补 0=正向。 */
        vPidPivotTurn((int)I(0), F(0), (uint8_t)I(1), (uint8_t)I(2));
        break;

    case STEP_PIVOT_LINE_WAIT:
        /* 支点转弯 → 交接给巡线。槽位（本积木是 **F2I7**: 2 个 float + 7 个 int）:
         * F(0)=angle（转角或绝对航向）, F(1)=gyro_target（交接后若听陀螺的那个目标角）,
         * I(0)=dir, I(1)=absolute, I(2)=fwd_ms（交接前盲走）,
         * I(3)=settle_ms（交接后只对准）, I(4)=line_ms（交接后沿线走）,
         * I(5)=tail_ms（沿线走跑完、发完 LINE_STOP 之后的**尾段盲走**）,
         * I(6)=settle_early（settle 段启动即返回{1=是,0=否}）。
         * ⚠️ 交接后听谁的航向 = 全局调参宏 LINE_CONV_SRC（2026-10-30 起不再是积木参数）;
         *    融合占比 = LINE_CAM_PCT。脚本层一律传 LINE_CAM_PCT_KEEP（不写 line_cam_pct）。
         * **边转边等线流**（重叠）, 误差进交接阈值就交给巡线 —— 见 ucRpiPivotTurnLine 的函数头。
         * 只动车、不碰舵机, 故**不进** c_bIsArmStep。 */
        ucRpiPivotTurnLine((int)I(0), F(0), (uint8_t)I(1), (uint32_t)I(2),
                           (uint32_t)I(3), (uint32_t)I(4), (uint32_t)I(5),
                           (uint8_t)LINE_CONV_SRC, (float)F(1), (uint8_t)I(6),
                           LINE_CAM_PCT_KEEP);
        break;

    case STEP_SPIN_TURN_WAIT:
        /* 原地转弯: 四轮绕车身中心自转, 陀螺仪闭环到目标角（方向 I(0), 角度 F(0)） */
        vPidSpinTurn((int)I(0), F(0), (uint8_t)I(1));
        break;

    case STEP_SPIN_LINE_WAIT:
        /* 原地转弯 → 交接给巡线（STEP_PIVOT_LINE_WAIT 的四轮自转版, 2026-10-30 加）。
         * 槽位与支点版**逐位同形**（F2I7, 见上面那个 case 的槽位表）:
         * F(0)=angle, F(1)=gyro_target, I(0)=dir, I(1)=absolute, I(2)=fwd_ms,
         * I(3)=settle_ms, I(4)=line_ms, I(5)=tail_ms, I(6)=settle_early。
         * 转弯本体换四轮自转（车心不动）, 交接后的全部段与支点版共用
         * 同一份代码（ucRpiSpinTurnLine → s_ucTurnLineTail）—— 语义看支点版的注释。
         * 只动车、不碰舵机, 故**不进** c_bIsArmStep。 */
        ucRpiSpinTurnLine((int)I(0), F(0), (uint8_t)I(1), (uint32_t)I(2),
                          (uint32_t)I(3), (uint32_t)I(4), (uint32_t)I(5),
                          (uint8_t)LINE_CONV_SRC, (float)F(1), (uint8_t)I(6),
                          LINE_CAM_PCT_KEEP);
        break;

    /* ===== 弧线转弯（位置模式） ===== */
    case STEP_ARC_TURN_WAIT:
        vChassisArcTurnPos(F(0), F(1));
        break;

    case STEP_ARC_TURN_L90_WAIT:
        vChassisTurn90Left();
        break;

    case STEP_ARC_TURN_R90_WAIT:
        vChassisTurn90Right();
        break;

    /* ===== 机械臂（本组步骤前置 vArmSeqAbort, 见循环开头的说明） ===== */
    case STEP_ARM_EDIT_WAIT:
        /* 中途让位手动操纵机械臂: 停车 + 强制进编辑模式 + 钉住链路, 等 MAIN 继续
         * （三条注意事项见 app_bt.c::vBtArmEditHold 的函数头） */
        vBtArmEditHold();
        break;

    case STEP_ARM_GROUP_WAIT:
        vArmGroupMoveWait(p->afF);
        break;

    case STEP_ARM_GROUP_NO_WAIT:
        vArmGroupMoveNoWait(p->afF);
        break;

    case STEP_SET_ZHUASHOU_WAIT:
        vArmSetZhuaShou((uint8_t)I(0), (uint8_t)I(1));
        break;

    case STEP_ARM_PLAY_WAIT:
        vArmPlayWait((emArmActionIdTdf)I(0));
        break;

    case STEP_ARM_PLAY_NO_WAIT:
        /* 播放动作预设, 启动即返回。I(0)=动作, I(1)=延迟执行[ms]（2026-10-08 加, 追加在
         * int 末尾 —— 老行缺的尾槽补 0 = 立即 = 老行为, map.c 那些行一个字节不用改）:
         * 0=立即; >0=**后台**计时, 到点才投给臂泵（臂闲时 ≤20ms 精度）, 后续步骤
         * （转弯/巡线）不等它。到点前再投一条=后写者胜; 脚本中止/新臂指令会丢弃未到点
         * 的投递（安全语义见 vArmPlayNoWaitDelay）。
         * ⚠️ I(0) 的枚举强转必须**同一行**传进实参 —— 控制台靠它给「动作」派生下拉。 */
        vArmPlayNoWaitDelay((emArmActionIdTdf)I(0), (uint32_t)I(1));
        break;

    case STEP_ARM_SPEED_SET:
        /* 中途修改舵机速度: 全臂速度闸的运行时版（°/s）, 写 Beam + **当场下发** 6 舵机 ——
         * 影响之后的恒速趋近与 S 曲线削峰, 对**正在播放**的动作当拍生效（下一拍削峰即用
         * 新值, 不重启曲线）。瞬时积木（只写速度, 不动车不动臂, 也不掐后台臂序列 ——
         * 它的意义恰恰是"边播边改"）, 按设计规则第 4 条不带后缀。30~1000 越界钳位在
         * setter 里做。想让某段动作慢/快, 把这块排在那段动作**之前**。 */
        vArmSetSpeedLimit((float)I(0));
        break;

    case STEP_ARM_DEPLOY_MARK:
        /* 改写「机械臂已展开」标记（2026-10-18 加）: 该标记决定 OLED「执行任务」与
         * 可重入预设被调入时**要不要先做一次机械臂待命展开**（见 vScriptTask 的
         * SCRIPT_REQ_TASK 与 c_bScriptCallPreset）。典型用法: 路线末尾把臂折回停放
         * 之后排一块 deployed=0 —— 下次任务进来会自动重做展开; 置 1 = 声明已展开。
         * ⚠️ vSetArmDeployed 必须是本 case 体**第一个**函数调用（控制台取原语名）。 */
        vSetArmDeployed((uint8_t)I(0));
        break;

    /* ===== 执行器 ===== */
    case STEP_LASER_ON:
        vRelayOn();                     /* 打开激光（继电器 JIGUANG 置位） */
        break;

    case STEP_LASER_OFF:
        vRelayOff();                    /* 关闭激光 */
        break;

    case STEP_BUGUANG_KAI:
        /* 补光灯开: I(0)=「自定义亮度」{1=是,0=否}, I(1)=亮度[%]（0~100）—— 两槽都
         * **追加在 int 末尾**, 老行（无参）缺槽补 0 = 不勾 = 用宏 FILL_LIGHT_ON_DUTY
         * = 老行为, map.c 一行不用改。勾了就按 pct 折千分比（×10）下发, 越界钳 0~100;
         * ⚠️ 只管**这一次**开灯 —— 灯关了再开（不勾）又是宏值, 排表的人要知道这点。
         * ⚠️ vFillLightSet 必须是本 case 体**第一个**函数调用（控制台取原语名）。 */
        vFillLightSet((I(0) != 0) ? (uint16_t)(((I(1) < 0) ? 0u : ((I(1) > 100) ? 100u
                                     : (uint32_t)I(1))) * 10u)
                     : (uint16_t)FILL_LIGHT_ON_DUTY);
        break;

    case STEP_BUGUANG_GUAN:
        vFillLightSet(0);                   /* 补光灯关（占空比 0） */
        break;

    case STEP_BEEP:
        /* 蜂鸣器: I(0)=响时长[ms], I(1)=「周期模式」{1=是,0=否},
         * I(2)=次数, I(3)=静音间隔[ms] —— 后三槽只在周期=1 时参与（前端按勾选联动
         * 启用输入框, 槽值本身一直带着）。不勾 = 响一声就走（原语自己就是三参的,
         * 不勾时传 off=0/times=1）。times<1 按 1 次; 阻塞（内部 vTaskDelay）,
         * 整段时长都占着脚本任务 —— 长鸣很多次请排 STEP_DELAY, 别把次数堆上天。 */
        vBuzzerBeep((uint16_t)((I(0) < 0) ? 0 : I(0)),
                    (uint16_t)(((I(1) != 0) && (I(3) > 0)) ? I(3) : 0u),
                    (uint8_t)((I(1) != 0) ? (((I(2) < 1) ? 1u : (uint32_t)I(2)) > 255u
                                            ? 255u : (uint32_t)I(2)) : 1u));
        break;

    /* ===== 校准（空壳, 等树莓派方案） ===== */
    case STEP_CALI_WAIT:
        vCaliRun((emCaliMethodTdf)I(0), (emCaliMethodTdf)I(1));
        break;

    /* ===== 树莓派 ===== */
    case STEP_WAIT_HANDSHAKE:
        /* 无限重试, 成功才返回; 链路已 OK 时再执行 = 判为断开重连。
         * health=1: 握手成功后**阻塞**做一次四路相机体检（发 "Z"）——
         *   I(0) 是 2026-11 追加在 int 末尾的健康检查开关, 老路线缺槽补 0 = 不检查,
         *   行为与加这个参数之前一字不差。
         *   ⚠️ 第一条函数调用必须仍是 ucRpiHandshake: 控制台「原语名」读的是
         *      case 体里第一个「标识符(」, 写成 `if (ucRpiHandshake()...)` 会把这块
         *      积木的原语显示成 "if"（实际踩过）。中止时 ucRpiHandshake 返回 0,
         *      这里用中止位判断, 不再去体检。 */
        ucRpiHandshake();
        if ((I(0) != 0) && (ucGetScriptAbortReq() == 0u))
        {
            (void)ucRpiHealthCheck((uint32_t)RPI_HEALTH_TIMEOUT_MS);
        }
        break;

    case STEP_QR_SCAN_WAIT:
        /* 二维码扫描: 发指令+等结果+失败音一步到位(结果在 Beam iGetQrCode)。
         * ⚠️ I(1) 必须**在同一行**传进去: 控制台取积木原语名读的是 case 体里第一个
         *    「标识符(」, 先把它读进局部变量会让这块积木显示成 "I" */
        ucRpiQrScan((uint32_t)I(0), (emQrAlgoTdf)I(1));
        break;

    case STEP_QR_SCAN_MAIN_WAIT:
        /* 主摄二维码扫描: 与上一步同一个实现, 只是让 Pi 用主摄而非扫码副摄。
         * 结果同样落进 iGetQrCode, 下游(IF 条件/排爆/反恐)照旧读它 */
        ucRpiQrScanMain((uint32_t)I(0), (emQrAlgoTdf)I(1));
        break;

    case STEP_HOSTAGE_SCAN_WAIT:
        /* 人质识别: 发指令+等结果+失败音一步到位(结果在 Beam vGetHostageResult) */
        ucRpiHostageScan((uint32_t)I(0));
        break;

    case STEP_COLOR_SCAN_WAIT:
        /* 色球识别: 发指令+等结果+失败音一步到位(结果在 Beam vGetColorResult; 对位链内被压制) */
        ucRpiColorScan((uint32_t)I(0));
        break;

    case STEP_RPI_SEND_TASK:
        vRpiSendTaskCmd((emRpiTaskTdf)I(0));
        break;

    case STEP_RPI_WARM_CAM:
        /* 预热摄像头: cam 选哪台相机(主摄/扫码副摄/pi/主摄+pi), uses = 这台相机接下来要被
         * 几个任务消费, prof/plan = **每次使用各自的档位**（2026-10-23, 每 2 bit 一次使用,
         * 见 app_rpi.h::WARM_PROF_*; 0 = 全默认）。
         * ⚠️ I(1)/I(2) 必须**在同一行**传进去: 控制台取积木原语名读的是 case 体里第一个
         *    「标识符(」, 先把它读进局部变量会让这块积木显示成 "I"（真踩过）。
         * ⚠️ 三个参数的类型转换**都要留着**: 控制台按 `(<类型>)I(n)` 里的类型名决定那一槽的
         *    控件（server.py::_build_catalog 的 casts 表）。第 3 槽故意是 uint32_t ——
         *    它配的是 DSL 里的 `<控件:warm_profiles>` 按钮, 不是枚举下拉。 */
        vRpiWarmCamera((emWarmCamTdf)I(0), (uint32_t)I(1), (uint32_t)I(2));
        break;

    case STEP_OPENMV_LED:
        /* OpenMV 补光亮度: 发"L,<level>"(0~100) → Pi 的 _cmd_openmv_led → 相机
         * 运行时命令 "L<n>\n", 当场生效。只发不等（无结果帧）。
         * ⚠️ I(0) 必须同一行传（理由同上）; 0=灯灭是**合法值**, 所以走专用
         *    vRpiOpenmvLed —— 通用数值发送器对 0 会退化成裸命令码（Pi 判未知指令）。
         * ⚠️ 与车上补光灯(STEP_BUGUANG_KAI/_GUAN)是两盏灯, 别调错对象。 */
        vRpiOpenmvLed((uint32_t)I(0));
        break;

    case STEP_OPENMV_LED_TOGGLE:
        /* OpenMV 补光**临时**开关: 发 "T"（无参）→ Pi 的 _cmd_openmv_led_toggle。
         * 「谁亮谁灭」由 **Pi** 定（它记着最近一次下发的亮度、也握着持久化的
         * config.OPENMV_LED_BRIGHTNESS）—— 本端不维护任何影子状态, 于是 OLED 调参、
         * 积木 STEP_OPENMV_LED、Pi Web 改值哪条路都不会与本端脱节。
         * ⚠️ 无参所以走得了通用 vRpiSendTaskCmd（"0 会退化成裸命令码"那个坑只影响
         *    **带数值**的发送器, 见上面 STEP_OPENMV_LED 的注释）。只发不等（无结果帧）。
         * ⚠️ Pi 侧**刻意不写** config: 灭灯是临时的, 巡线起流("2")前的自动对齐会把灯点回来。 */
        vRpiSendTaskCmd(RPI_TASK_LED_TOGGLE);
        break;

    case STEP_LINE_CONVERGE_WAIT:
        /* 巡线收敛闭环: 发 LINE_START + 闭环横移/修航向 + 发 LINE_STOP 一步到位。
         * 槽位（本积木是 **F1I2**: 1 个 float + 2 个 int）:
         * F(0) = 陀螺目标角[°]（绝对世界航向, 零点=开机朝向）,
         * I(0) = 收敛时长[ms], I(1) = 启动即返回（{1=是,0=否}）。
         * ⚠️ 航向源 = 全局调参宏 LINE_CONV_SRC（2026-10-30 起不再是积木参数）;
         *    融合占比 = LINE_CAM_PCT。脚本层一律传 LINE_CAM_PCT_KEEP（不写 line_cam_pct）。
         * 目标角由 ucRpiLineConverge 写进 Beam 当**这一步的起点**（src=陀螺/融合 时 —— 融合档
         * 的陀螺腿咬它）; 四种源各自的行为、为什么输出端加权安全, 见 app_line.c 的函数头注释。
         * I(1)=是 的判据（何时算"收敛完成"）在 app_config.h 的「巡线收敛增强」区调。 */
        ucRpiLineConverge((uint32_t)I(0), (uint8_t)LINE_CONV_SRC, (float)F(0),
                          (uint8_t)I(1), LINE_CAM_PCT_KEEP);
        break;

    case STEP_LINE_STRAIGHT_WAIT:
        /* 巡线直走: 沿线**边前进边修**走固定距离 —— F(0)=dist_mm 按位置模式**同一条
         * 梯形估时公式**反推闭环时长(fChassisMoveTimeEst, 体坐标系 AXIS_Y)再乘时间系数
         * k(Beam line_time_scale, 蓝牙「巡线」组当场改 —— 估时搬到速度模式斜坡带系统
         * 偏差, 走过了调小没走到调大)。
         * 槽位（本积木是 **F2I4**: 2 个 float + 4 个 int）:
         * I(0)=accel / I(1)=speed 同时管估时与实际斜坡(accel 钳 1~255),
         * I(2)=use_res 吃「路径残留偏移」(fwd 分量并入距离, side 丢弃),
         * I(3)=settle_ms 起步对准段（起流后先原地收敛把摆歪的起步修掉再沿线走; 0=不对准）。
         * ⚠️ 航向源 = 全局调参宏 LINE_CONV_SRC（2026-10-30 起不再是积木参数）;
         *    融合占比 = LINE_CAM_PCT。脚本层一律传 LINE_CAM_PCT_KEEP。
         * ⚠️ dist/speed/accel 任一 ≤0(新拖出的积木=没配齐) → 拒绝 + 一步不动（不响, 留痕）。 */
        ucRpiLineStraightWalk(F(0), (int)I(0), (int)I(1), (uint8_t)LINE_CONV_SRC,
                              (float)F(1), (uint8_t)I(2), (uint32_t)I(3),
                              LINE_CAM_PCT_KEEP);
        break;

    case STEP_BALL_ALIGN_WAIT:
        /* 色球区对位(多方案链): 按槽序逐个试 emAlignMethodTdf, 第一个成功即停。
         * 槽位: I(0)=每方案超时, I(1..3)=方案1/2/3（0=不试）。
         * 三球方案=中球偏差(标定 BALL_*), 矩形方案=板心偏差(标定 BALL_* + rect_*_center)。
         * 见 app_rpi.c 的 ucRpiAlignChain / ucRpiBallAlign / ucRpiRectAlign。
         * 注: enum 强转必须写在调用实参里 —— 控制台靠它给这三槽派生下拉。 */
        ucRpiAlignChain((uint32_t)I(0), (emAlignMethodTdf)I(1),
                        (emAlignMethodTdf)I(2), (emAlignMethodTdf)I(3));
        break;

    case STEP_HOSTAGE_ALIGN_WAIT:
        /* 人质对位: 识别(三只齐才算数) → 基准人质偏差横/纵移,
         * 见 app_rpi.c::ucRpiHostageAlign; 超时 I(0), 基准人质序号 I(1),
         * 标定在 app_config.h 的 HOSTAGE_*（独立一套, 不共用 BALL_*） */
        ucRpiHostageAlign((uint32_t)I(0), (uint8_t)I(1));
        break;

    case STEP_ROAD_ALIGN_WAIT:
        /* 看路对位: 发"G"（rev=1 时发 "M" 反向看路, Pi 侧镜像后量黄区右边缘）
         * → 等 0x08 帧（Δx=黄区左边缘 x, 以及 Δy=黄区下边缘 y）
         * → (理想 − 实测) × 比例系数 × 方向 → 修车体**纵移**(必做) / **横移**(use_y=1)。
         * 见 app_rpi.c::ucRpiRoadAlign; 超时 I(0) / 是否修横向 I(1) / 反向看路 I(2);
         * 标定在 app_config.h 的 ROAD_*（蓝牙「看路」组可运行时改）。
         * ⚠️ **单次开环**、内部不死循环 —— 要迭代收敛就在 map.c **连排两块**。
         * ⚠️ 只动车、不碰舵机, 故**不进** c_bIsArmStep（同 STEP_TARGET_ALIGN_WAIT）。 */
        ucRpiRoadAlign((uint32_t)I(0), (uint32_t)I(1), (uint32_t)I(2));
        break;

    case STEP_ROAD_SET_IDEAL:
        /* 看路理想值: 把两个理想值写进 Beam 运行时量（与蓝牙「看路」组那一对是同一对,
         * 谁后写谁生效）—— 于是**每条路线预设可以带自己的一套理想值**。
         * 见 app_rpi.c::vRoadSetIdeal; ⚠️ 0 = 这一项不动（0 是"未标定"的哨兵）。
         * ⚠️ 瞬时（只写 Beam, 不发串口、不动车）, 照设计规则第 4 条**免 _WAIT 后缀**。 */
        vRoadSetIdeal(F(0), F(1));
        break;

    case STEP_ROAD_YAW_WAIT:
        /* 看路校准: 发"H" → 等 0x09 帧（ang = 黄区直边的图像方向角）→ **对齐到相机真值**:
         * bias = wrap180(原始航向 − err) ⇒ 发布 yaw ≡ err（每次校准后航向零点 = "放直"）,
         * ROADYAW_APPLY_EN=1 时发布 yaw 统一扣 —— PID/巡线/OLED 全生效。
         * 见 app_rpi.c::ucRpiRoadYaw; 超时 I(0); 标定在 app_config.h 的 ROADYAW_*
         * （蓝牙「看路校准」组可运行时改）。⚠️ 世界坐标系不受影响（世界航向记录换算回原始值）。 */
        ucRpiRoadYaw((uint32_t)I(0));
        break;

    /* ===== 排爆（排爆区抓取） ===== */
    case STEP_BOMB_GRAB_WAIT:
        /* 排爆: 二维码首位(目标色) → 在三球里找它的位置 → 播对应动作预设 → 播复位。
         * 四个"动作"参数都是 emArmActionIdTdf（下拉自动取自该枚举, 换了预设改参数即可）。 */
        ucRpiBombGrab((emArmActionIdTdf)I(0), (emArmActionIdTdf)I(1),
                      (emArmActionIdTdf)I(2), (emArmActionIdTdf)I(3));
        break;

    /* ===== 反恐任务（人质区救援） ===== */
    case STEP_HOSTAGE_RESCUE_WAIT:
        /* 反恐任务: 二维码第三位(目标形状 1圆柱/2圆锥/3腰鼓) → 在已识别的三只人质里找它的
         * 位置 → 播那一位的动作预设, **播完不复位**（复位由路线自己排, 见 app_rpi.c）。
         * 三个"动作"参数都是 emArmActionIdTdf（下拉自动取自该枚举）。 */
        ucRpiHostageRescue((emArmActionIdTdf)I(0), (emArmActionIdTdf)I(1),
                           (emArmActionIdTdf)I(2));
        break;

    /* ===== 色球单动作抓取（2026-10 新方案, 与上面的排爆并存） ===== */
    case STEP_BALL_GRAB_ONE_WAIT:
        /* 只录一个"抓中间球"的动作: 识别定位目标(0左/1中/2右) → 中球偏差叠加 (位置-1)×
         * one_lat_mm 平移过去(目标=新"中间球")。抓取**剩提前量异步并行**: 底盘先动,
         * 估时倒计时到「剩 I(4) 毫秒(垫底pad)」才把抓取动作投给臂泵(锚定到位而非起点,
         * 0=到位后才发), 本步只等它结束。放置改回**回退起点后延时** I(5): 回退开始后
         * I(5) 毫秒才发放置动作(0=回退同时放); 放置播完立即返回, **不做放置后收敛**
         * (2026-10-23 用户定; 需要收敛的路线自己在块后排「巡线收敛」)。
         * 槽位（本积木是 **F1I8**: 1 个 float + 8 个 int）:
         * F(0)=陀螺目标角(src=陀螺/融合时咬死), I(0)=识别超时, I(1)=角度纠正方案
         * (0不/1视觉yaw/2 OpenMV巡线), I(2)=抓取动作, I(3)=放置动作, I(4)=抓取提前量,
         * I(5)=放置延时, I(6)=反向执行(2026-10-30 加; 老行缺槽补 0=正向):
         * 勾上=道具换到路另一边、相机朝另一边看, 本步方向整体镜像(对位/间距/拉回/yaw转正)。
         * I(7)=单步确定(2026-11 加; 老行缺槽补 0=老行为):
         * 勾上=色球对位→等 MAIN→移动→等 MAIN→抓取→等 MAIN→放到爆爆桶,
         * 每段结束先停车; 此时 I(4) 不再起作用（移动已停稳, 抓取在按 MAIN 后立即发）。
         * ⚠️ I(3)=放置动作可填 0（控制台下拉「无」）: 抓完仍只回退间距分量到中间对齐位,
         *    但**不投放置动作**（决赛「色球只抓不放」）；单步确定时最后一段也不存在。
         * ⚠️ 航向源 = 全局调参宏 LINE_CONV_SRC（2026-10-30 起不再是积木参数）;
         *    融合占比 = LINE_CAM_PCT。脚本层一律传 LINE_CAM_PCT_KEEP。
         * ⚠️ 识别段临时压灭 OpenMV 补光再恢复（BALL_LED_OFF_*, 编译期; 场地换边后
         *    LED 热点进主摄视野冲淡 HSV 阈值）—— mode=2 的收敛不受影响（识别前/后灯都亮）。
         * ⚠️ 两个动作参数的枚举强转必须**同一行**传进实参（控制台取 case 体第一个
         * "标识符("当原语名, 枚举下拉靠 (emArmActionIdTdf)I(n) 派生）。 */
        ucRpiBallGrabOne((uint32_t)I(0), (uint8_t)I(1),
                         (emArmActionIdTdf)I(2), (emArmActionIdTdf)I(3),
                         (uint32_t)I(4), (uint32_t)I(5), F(0),
                         (uint8_t)LINE_CONV_SRC, LINE_CAM_PCT_KEEP, (uint8_t)I(6),
                         (uint8_t)I(7));
        break;

    /* ===== 人质单动作抓取（2026-10 新方案, 与上面的反恐并存） ===== */
    case STEP_HOSTAGE_GRAB_ONE_WAIT:
        /* 单个"抓中间人质"动作 + 平移定位; place=0(无) 时不放置: I(3)=keep_res 把平移总位移
         * 记成"路径残留偏移", 由后面的世界系平移积木勾"叠加上次残留偏移"一次性吃掉。
         * place≠0 时抓完等臂空 → 只回退间距分量到中间对齐位(=爆爆桶位) → 从回退起点延时
         * I(8) 投放置动作(0=回退同时放) → 播完返回并清残留(此时 keep_res 不记账)。
         * 抓取**剩提前量异步并行**: I(4)=grab_lead_ms 到位前提前量(锚定到位而非起点, 0=到位后才发)。
         * 槽位（本积木是 **F1I9**: 1 个 float + 9 个 int）:
         * F(0)=陀螺目标角(src=陀螺/融合时咬死), I(0)=识别超时(⚠️7500, 同人质对位),
         * I(1)=角度纠正(0不/1识别前巡线收敛), I(2)=抓取动作, I(3)=是否记录残留偏移,
         * I(4)=抓取提前量, I(5)=反向执行(2026-10-30 加; 老行缺槽补 0=正向):
         * 勾上=道具换到路另一边、相机朝另一边看, 本步方向整体镜像(对位/间距/拉回);
         * 残留记账随实际位移自动翻, 消费端"叠加残留"一字不改。
         * I(6)=单步确定(2026-11 加; 老行缺槽补 0=老行为):
         * 勾上=人质对位→等 MAIN→移动→等 MAIN→抓取; place≠0 再加→等 MAIN→放到爆爆桶,
         * 每段结束先停车; 此时 I(4) 不再起作用（移动已停稳, 抓取在按 MAIN 后立即发）。
         * I(7)=放置动作(2026-11 加; 0=无=不放置=老行为; 控制台下拉可空),
         * I(8)=放置延时(2026-11 加; 从回退起点起算, 0=回退同时放)。
         * ⚠️ 航向源 = 全局调参宏 LINE_CONV_SRC（2026-10-30 起不再是积木参数）;
         *    融合占比 = LINE_CAM_PCT。脚本层一律传 LINE_CAM_PCT_KEEP。 */
        ucRpiHostageGrabOne((uint32_t)I(0), (uint8_t)I(1),
                            (emArmActionIdTdf)I(2), (uint8_t)I(3), (uint32_t)I(4),
                            F(0), (uint8_t)LINE_CONV_SRC, LINE_CAM_PCT_KEEP,
                            (uint8_t)I(5), (uint8_t)I(6),
                            (emArmActionIdTdf)I(7), (uint32_t)I(8));
        break;

    /* ===== 打靶（反恐区） ===== */
    case STEP_TARGET_ALIGN_WAIT:
        /* 标靶对位: 识别(三颗齐才算数) → 基准靶横向偏差横移摆正车,
         * 见 app_rpi.c::ucRpiTargetAlign; 超时 I(0), 基准靶序号 I(1),
         * 标定在 app_config.h 的 TARGET_ALIGN_*（独立一套, 不共用 TARGET_*）。
         * ⚠️ 只横移、不动臂, 故**不进** c_bIsArmStep。 */
        ucRpiTargetAlign((uint32_t)I(0), (uint8_t)I(1));
        break;

    case STEP_TARGET_SCAN_WAIT:
        /* 靶标识别(仅扫描): 就是 ucRpiTargetScan 的积木化 —— 发"6"+等结果+失败音,
         * 结果已在 Beam（vGetTargetResult 读）。**不动车也不动臂**,
         * 故不进 c_bIsArmStep（与 STEP_TARGET_ALIGN_WAIT 同理）。 */
        ucRpiTargetScan((uint32_t)I(0));
        break;

    case STEP_TARGET_CALIB_WAIT:
        /* 标靶校准: 发识别指令+等结果+按二维码第二位选靶+按比例尺微调舵机6/3, 一步到位。
         * I(0) = 等结果帧的超时（0 = 用蓝牙运行量 tg_calib_timeout_ms;
         * ⚠️ 这个值以前**没人消费**, 积木里改它不生效 —— 现在传进去了）;
         * 比例尺/方向/画面中心都在 app_config.h 的 TARGET_*。
         * ⚠️ 增量式 —— 连排两步就是迭代收敛。 */
        vTargetCalibAim((uint32_t)I(0));
        break;

    case STEP_TARGET_SERVO_WAIT:
        /* 标靶流控闭环(**新方案**, 与上面那块开环迭代并存):
         * 起流 → 按二维码第二位选靶 → 10Hz P 环微调舵机6/3 → 收敛/超时/行程超限/中止 → 停流,
         * 见 app_rpi.c::ucRpiTargetServo; 超时 I(0),
         * 瞄准点/比例尺/方向**共用 tg_***（与标靶校准同一套物理标定）, 节拍类见 ts_*。
         * ⚠️ 开了「底盘参与闭环」(ts_chassis_en) 后本步**还会动车**（|dx| 进门限后位置模式
         *    **前后**进给 —— 相机装了 90°, 图像横向对应车体纵移）,
         *    底盘那一拍的实际耗时 ≈ ctrl_ms + 位置模式阻塞 ≈ 2× 名义值 → I(0) 要留余量。 */
        ucRpiTargetServo((uint32_t)I(0));
        break;

    case STEP_TARGET_SHOOT_WAIT:
        /* 平移射靶(**第三套方案**, 激光固定在底盘、全程不动机械臂):
         * 粗调(发"6"识别三颗齐→按二维码第二位选色→沿靶排一次**纵移**) → 精调(发"7"起流,
         * 步进位置模式小步纵移闭环→收敛/超时/行程超限/中止) → 停流,
         * 见 app_rpi.c::ucRpiTargetShootAlign; 超时 I(0), 是否记残留偏移 I(1),
         * I(2)=反向执行(2026-10-30 加; 追加在 int 末尾, 老行缺槽补 0=正向):
         * 勾上=靶排换到路另一边、相机朝另一边看, 沿排方向本步内取反(tc_sign 不动);
         * 残留记账随实际位移自动翻, 消费端"叠加残留"一字不改。
         * ⚠️ rev 时**全程只用主摄**: CSI 固定在底盘、不随臂转向另一边的靶排 ——
         *    细调不再优先 pi、粗调失败也不再拿 CSI 重试(tc_use_picam 不动)。
         * ⚠️ 轴是**车体纵移 v_y**(cx 偏差驱动 —— 靶排沿车体纵向排开, 同色球抓取沿排平移);
         * ⚠️ 只动车、不碰舵机, 故**不进** c_bIsArmStep（同 STEP_TARGET_ALIGN_WAIT）;
         * ⚠️ 激光射击不在本步 —— 紧后排 STEP_LASER_SHOOT。 */
        ucRpiTargetShootAlign((uint32_t)I(0), (uint8_t)I(1), (uint8_t)I(2));
        break;

    case STEP_TARGET_SHOOT_HOLD_WAIT:
        /* 平移射靶**保持**(OLED「射靶测试」页专用, 2026-10-09 加, **无参**):
         * 与上面同一套测量/控制律, 但**永不因超时/收敛/noprog/行程闸退出** —— 只有
         * 中止位（OLED 按「停止」→ vScriptAbort）能停, 用来现场标定 tc_cx_aim（瞄准列）。
         * ⚠️ 粗调 + 等停稳照跑一遍(有界), 之后每拍重读瞄准常数: 在 OLED 上按一格(±5px)
         *    → 偏差出死区 → 车跟到新目标。
         * ⚠️ **不记残留偏移**(keep_res 恒 0) —— 保持测试挂着残留会被后面任何勾"叠加残留"的
         *    世界系平移误吃。
         * ⚠️ **不进 c_bIsArmStep**（只动车、不碰舵机, 同上面那块）；⚠️ **必须排在最后**
         *    （它不会自己结束, 后面排什么都跑不到）。见 app_rpi.c::ucRpiTargetShootHold。 */
        ucRpiTargetShootHold();
        break;

    case STEP_BARREL_ALIGN_WAIT:
        /* 爆爆桶对位(2026-10-24 加): 发"J"起流(OpenMV 检测黑桶 → Pi 纯转发 0x0B 流帧)
         * → 等停稳 → 每拍按 cx 偏差小步**纵移**闭环 → 收敛/超时/行程超限/noprog/中止
         * → 发"K"停流, 见 app_rpi.c::ucRpiBarrelAlign; 超时 I(0), 是否记残留偏移 I(1)。
         * ⚠️ 轴是**车体纵移 v_y**(cx 偏差驱动 —— 同平移射靶定案的轴映射);
         * ⚠️ bc_cx_aim=0(未标定) → 本步**只测不改**: 起流采 ~3s 报中位 cx 后返回
         *    （标定自举: 车摆到理想推桶位姿跑一次, 读 "BKT meas:" 行填蓝牙 bc_cx_aim）;
         * ⚠️ 只动车、不碰舵机, 故**不进** c_bIsArmStep（同 STEP_TARGET_SHOOT_WAIT）;
         * ⚠️ 本步不占主摄（OpenMV 独立串口）, 但 COORD 通道由 Pi 侧互斥;
         * ⚠️ 补光: OpenMV 自带的补光灯上电常亮, 本步无需挂 FILL_TASK（车的补光灯照不到
         *    桶, 挂了也是白亮）; 若现场动过 OpenMV 灯(T/STEP_OPENMV_LED), 排块前置。 */
        ucRpiBarrelAlign((uint32_t)I(0), (uint8_t)I(1));
        break;

    case STEP_LASER_SHOOT:
        /* 激光射击: 开激光 → 等 ms → 关激光（一步到位, 免得路线里排开/延时/关三行） */
        vLaserShoot((uint32_t)I(0));
        break;

    case STEP_QR_SET_TEST:
        /* 调试二维码（测试用）: 直接把结果写成指定三位数, 不经过 Pi。
         * 用途: 没扫码 / 台架上演练后续步骤时, 让下游步骤（标靶校准、色球对位…）
         * 拿到一个"像样的"二维码。⚠️ 会覆盖真实扫码结果。
         * ⚠️ 每位必须 1~3 —— 下拉里只能选到合法组合, 固件另有一道防呆（c_iQrTestSanitize）。
         * ⚠️ 防呆**不能写在 case 体开头**: 控制台取积木原语名 = 读 case 体里第一个
         *    「标识符(」，先写校验语句就会把原语名显示成那个函数（踩过: 显示成 "I"）。 */
        vSetQrCode(c_iQrTestSanitize((int32_t)I(0)));
        /* 「调试」= 人工设的值 → 打上 ucGetQrManual, 后面若还排了二维码扫描那一步,
         * 它会**直接跳过、不再问 Pi**（否则扫描会把刚调试好的值覆盖成 -1 = 没扫到,
         * 台架上等于白调试一遍）。
         * ⚠️ 必须写在 vSetQrCode **之后**: 控制台取积木原语名读的是 case 体里
         *    **第一个**函数调用, 放前面这条积木会显示成 vSetQrManual。
         * ⚠️ 非法组合（清洗成 -1）**不置位** —— 那本来就要报错, 不该顺带把扫码关掉
         *    （关掉之后下游只会看到 -1 而不知道是"没扫"还是"值非法")。 */
        if (iGetQrCode() >= 100) { vSetQrManual(1u); }
        break;

    case STEP_QR_SET_RANDOM:
        /* 随机二维码: 每次执行都写一个**随机的合法值**（三位各 1~3, 共 27 种）并打上
         * 人工预设标记 —— 是「多次扫码识别无果」之后的兜底: 下游（排爆抓取 / 反恐任务 /
         * 标靶校准）都按二维码选目标, 拿不到值就整步不动。
         * 见 app_rpi.c::vSetQrCodeRandom（随机源、调试串都在那里）。
         * ⚠️ 标记一旦打上, 后面**所有**二维码扫描步骤都会跳过（只能复位/断电清除）——
         *    有意如此, 免得随后一次失败的重扫把刚随机好的值覆盖成 -1。
         * ⚠️ 第一句就是唯一的函数调用（控制台取积木原语名读的是 case 体里**第一个**
         *    「标识符(」, 本块显示成 vSetQrCodeRandom, 与「调试二维码」分得开）。 */
        vSetQrCodeRandom();
        break;

    /* ===== 工具 ===== */
    case STEP_TIMER:
        /* 状态页秒表: I(0)=动作（0开始/1结束/2重置）—— 枚举强转必须写在实参里,
         * 控制台靠它给「动作」派生下拉。瞬时积木, 不动车、不阻塞; 显示在第 1 页第 4 行。 */
        vTimerCmd((emTimerCmdTdf)I(0));
        break;

    case STEP_DELAY:
        /* 让出式延时（与 pro 的忙等 vDelayMs1 不同, 不饿死低优先级任务） */
        vTaskDelay(pdMS_TO_TICKS((uint32_t)I(0)));
        break;

    case STEP_STOP:
        /* 阻塞停车: 底盘清速, 等 MAIN 键继续 */
        vScriptParkWaitKey();
        break;

    case STEP_DEAD:
        /* 等 MAIN 按键按下继续, 不动车（pro 版为忙等 PC9 电平） */
        (void)ucScriptWaitKeyConfirm();
        break;

    case STEP_SCRIPT_RESET:
        /* 重启**当前这一帧**所在的表（不弹栈）:
         *   深度 0 = 「回到本轮路线的第一条实质步骤」, 与旧版行为一字不差;
         *   深度 > 0 = 重启被跳进来的那张表本身。
         * ⚠️ 所以子预设里排这一块会**自循环、永不返回** —— 这是有意选的语义:
         *    子预想要「循环自己」就用它, 想「跳出去」得让调用方处理。
         * ⚠️ 退化表（整张表只有握手/等键/重置这几类, ucScriptResetTarget 会全跳掉）
         *    会变成**不让出的紧循环**, 把脚本任务按住的 CPU 全吃掉、饿死按键/OLED。
         *    深度 0 时旧版同样如此, 不是本次引入的; 但子预设让它更容易被写出来。
         * ⚠️ 这里**没有 continue**: 本轮的 usIdx++ 已经发生过了, 赋值 + break 即可
         *    （旧版靠 continue 跳过循环末尾那个 usIdx++ —— 那个结构已不存在）。
         *    这是唯一能让脚本「自己重头跑」的位置: vScriptRun 返回后 vScriptTask
         *    会永久停车, 只能断电/复位重启。 */
        pstFr->usIdx = ucScriptResetTarget(pstFr->pstTable, pstFr->usCount);
        break;

    case STEP_TEST_LOOP_WAIT:
        /* 测试: 把选中的积木反复执行, 每轮跑完等 MAIN 键继续（见 vScriptTestLoop）。
         * 参数 0 是**步骤类型** —— 下拉由 emScriptStepTypeTdf 自动派生（所以固件里
         * 必须自己防呆: 哨兵 STEP_COUNT / 自测 / 重置脚本都会被拒）。 */
        vScriptTestLoop((emScriptStepTypeTdf)I(0), (uint32_t)I(1));
        break;

    /* ===== 流程控制（嵌套分支 / 跳到别的预设 / 重置） ===== */
    case STEP_IF:
        /* 条件成立落进 then 分支; 不成立跳到同层的「否则」之后（没有否则就跳到「结束如果」之后）。
         * ⚠️ 条件选 SCRIPT_COND_LINE 时会阻塞并跟树莓派对话（见条件注册表）。 */
        vScriptIf((emScriptCondTdf)I(0));
        break;

    case STEP_ELSE:
        /* then 分支自然落下来到这里 → 跳过整段 else 分支, 收在「结束如果」之后 */
        vScriptElse();
        break;

    case STEP_END_IF:
        /* 纯配对锚点, 本身不做任何事（原语名 vScriptEndIf 只为了与另两块标记对称） */
        vScriptEndIf();
        break;

    case STEP_SCRIPT_CALL:
        /* 无条件跳到目标预设, 跑完自动跳回下一行 */
        vScriptCallPreset((emScriptPresetIdTdf)I(0));
        break;

    case STEP_SCRIPT_RETURN:
        /* 从被跳进来的子预设提前返回; 顶层执行 = 本轮路线到此为止 */
        vScriptReturn();
        break;

    default:
        /* 未知步骤类型: 跳过（生产版本不应出现） */
        break;
    }

    /* 任务补光收尾: 无论本步成功/超时/被中止（阻塞原语对这些路径都是**返回**,
     * 中止位在步骤边界生效 —— 见 vScriptTask 的说明）, 出了这一步就灭灯。
     * 「任务执行完关灯」唯一出口, 别往各 case 里散着写。 */
    if (ucFillTask != FILL_TASK_NONE)
    {
        vFillLightTaskEnd();
        if (usFillDuty != 0u)
        {
            vRpiSendDebug("FILL: off");
            usFillDuty = 0u;
        }
    }

#undef F
#undef I
}

/**
  * @brief       逐步骤执行脚本表（支持嵌套 IF 分支 / STEP_SCRIPT_CALL 跳到别的预设）
  * @param       pstTable  步骤表数组指针
  * @param       usCount   步骤总数
  * @retval      None
  *
  * 对每个条目按 eType 分发到底层原语调用。位置移动原语阻塞至运动完成，
  * 函数返回即表示该步骤完成。仅任务上下文调用（内部用 vTaskDelay）。
  *
  * 循环结构 = **帧栈 + 先步进再执行**:
  *   取到步骤后立刻 `usIdx++`, 然后才执行 —— 于是跳转时父帧天然停在「跳转指令的下一行」,
  *   弹栈回来不用再补一次 +1（弹栈路径只有一行 `s_ucScriptDepth--`）。
  * ⚠️ 旧版把 `usIdx++` 放在循环末尾、靠 `STEP_SCRIPT_RESET` 里的 `continue` 跳过它。
  *    那个结构已经不存在了: reset 现在只要赋值 + break。CLAUDE.md 设计规则第 6 条
  *    已按此重写, 别再照旧描述找「循环末尾的 usIdx++」。
  */
void vScriptRun(const stScriptStepTdf *pstTable, uint16_t usCount)
{
    /* 进场: 重置帧栈, 顶层帧 = 传进来的这张表。帧栈是文件内 static, 每次进来都重来一遍,
     * 免得将来多调一次就拿到上次的残留。 */
    s_ucScriptDepth = 0u;
    s_astScriptStack[0].pstTable   = pstTable;
    s_astScriptStack[0].usCount    = usCount;
    s_astScriptStack[0].usIdx      = 0u;
    s_astScriptStack[0].ucPresetId = ucScriptPresetIdOf(pstTable);
    vSetScriptPresetId(s_astScriptStack[0].ucPresetId);
    vSetScriptCallDepth(0u);

    /* 路线开始跑 = 收回舵机所有权: 蓝牙编辑模式可能是在「脚本没跑」时被打开并留着的
     * （见 app_bt.c::bt_cmd_mode 的入场条件与 vBtArmEditHold）, 不在这儿清掉, 一按 MAIN
     * 开跑就会出现「PC 点动与路线抢舵机」—— 那条结构性保证就漏了。
     * 同时也给遥测 script_running 置位（脚本没跑时才是 0, 编辑模式才可随时进）。 */
    vSetBtEditMode(0u);
    vSetScriptRunning(1u);

    for (;;) {
        stScriptFrameTdf *pstFr = &s_astScriptStack[s_ucScriptDepth];
        const stScriptStepTdf *p = NULL;   /* ⚠️ AC5 是 C90: 声明必须在块首, 不能落在语句之后 */

        /* 被要求中止（OLED 菜单的「停止巡线测试」/ 菜单改跑别的预设）→ 整个路线就此结束。
         * 中止点是**步骤边界** —— 已经发出去的那一步跑完再说, 不会从半路把底盘/臂掐掉
         * （与后台臂序列的「后写者胜 + 显式终止」同一条纪律）。
         * ⚠️ 巡线那一步还要在**自己的闭环循环里**再查一次同一个位（app_line.c）:
         *    它一步可以跑无限长, 只靠这里就停不下来了。 */
        if (ucGetScriptAbortReq() != 0u) {
            break;
        }

        /* 本帧跑完: 顶层 → 整个脚本结束（同旧版的 while 退出）;
         * 子帧 → 弹栈返回, 父帧的 usIdx 早已越过那条跳转指令, 所以**不用**再 +1。 */
        if (pstFr->usIdx >= pstFr->usCount) {
            if (s_ucScriptDepth == 0u) {
                break;
            }
            s_ucScriptDepth--;
            vSetScriptPresetId(s_astScriptStack[s_ucScriptDepth].ucPresetId);
            vSetScriptCallDepth(s_ucScriptDepth);
            continue;
        }

        p = &pstFr->pstTable[pstFr->usIdx];
        pstFr->usIdx++;              /* ⚠️ 先步进, 再执行（见函数头注释） */

        /* 步骤执行: switch 抽在 vScriptExecStep —— 「测试」积木复用同一条路径 */
        vScriptExecStep(p, pstFr);
        /* ⚠️ 循环末尾**没有** usIdx++ 了 —— 步进已经挪到本步执行之前（见函数头注释）。 */
    }

    /* 跑完了: 任务随后永久停车（只能断电/复位再跑）, 所以必须把「脚本在跑」清掉 ——
     * 它同时是蓝牙编辑模式的入场条件（脚本停了就允许随时进编辑模式点动机械臂）。 */
    vSetScriptRunning(0u);
}

/* ==================================================================
 * 任务入口
 * ================================================================== */

/**
  * @brief       脚本任务：开机跑活动预设 → 之后等菜单/外部投递的请求
  * @param       argument  任务参数（未使用）
  * @retval      None
  *
  * 两个阶段:
  *   ① **开机第一轮**跑编译期选定的 g_pstScriptActive（SCRIPT_PRESET_ACTIVE）——
  *      与改前一字不差。⚠️ 这一支不能删: 原实现就是上电即跑, 改成纯「等请求」会让
  *      比赛时上电不动车。只用 s_bBootedOnce 隔离第一次循环。
  *   ② 之后进**请求循环**, 收 OLED 菜单那三类请求（跑任务预设 / 握手让出 / 无限巡线测试）。
  *      原来是跑完 `while(1) vTaskDelay(100000)` 永久停车（只能断电重跑）, 现在是
  *      50ms 空闲轮询 —— 菜单才能反复启动/停止路线。
  *
  * ⚠️ 请求里那两件「本来该在别处做」的事为什么非在这做: ucRpiHandshake 会无限阻塞
  *    （且 s_bHsDone 只有本任务读写）, 路线/巡线会动底盘 —— 两者都只能在脚本任务上下文跑。
  *    OLED 菜单(最低优先级)碰它们 = 整屏停摆。
  * ⚠️ 「后写者胜」: 每个请求都会先把当前跑着的东西中止掉（vScriptPostReq 置中止位）,
  *    中止点在**步骤边界**, 不是从半路掐底盘/臂。
  */
void vScriptTask(void *argument)
{
    static uint8_t s_bBootedOnce = 0u;   /* 开机那一轮只跑一次（跑完就进请求循环） */
    uint8_t  ucReq;
    uint8_t  ucArg;

    (void)argument;

    for (;;) {
        /* ---- ① 开机第一轮: 跑活动预设 ---- */
        if (s_bBootedOnce == 0u) {
            s_bBootedOnce = 1u;
            vScriptRun(g_pstScriptActive, g_usScriptActiveCount);
            continue;
        }

        /* ---- ② 请求循环 ---- */
        ucReq = ucTakeScriptReq(&ucArg);
        if (ucReq == SCRIPT_REQ_NONE) {
            vTaskDelay(pdMS_TO_TICKS(50));   /* 空闲: 让出 CPU（不是永久停车） */
            continue;
        }

        /* 取到请求 → **先清中止位**: 它是电平量, vScriptPostReq 刚把它置了 1,
         * 不清的话下面一开跑就会立刻被自己掐掉。 */
        vSetScriptAbortReq(0u);

        switch (ucReq) {
        case SCRIPT_REQ_HANDSHAKE:
            /* 握手让出（菜单「握手让出」—— 运行中途想进机械臂编辑模式就走它）。
             * 置 script_running 是为了让「脚本没在跑才能进编辑模式」那条判据保守:
             * 握手期间本来就不该让蓝牙抢舵机（阶段 1 里 bt_hs_wait 才是让位的闸门）。
             * 完成时刻给菜单显示 1s 的「让出成功!」用。 */
            vSetScriptRunning(1u);
            if (ucRpiHandshake() != 0u) {
                vSetHandshakeDoneTick(xTaskGetTickCount());
            }
            vSetScriptRunning(0u);
            break;

        case SCRIPT_REQ_TASK:
            /* 跑一条**需要机械臂已展开**的任务预设（OLED「执行任务」→ 色球放置 /
             * 激光射靶 / 人质抓取）。查不到 id = 控制台删过这个预设 / 编号对不上
             * → 给 Pi 打一条调试串 + 拒绝, **别静默什么都不做**
             * （操作员会以为是按键没反应）。 */
            {
                const stScriptPresetTdf *pstTask =
                    cpstFindScriptPreset((emScriptPresetIdTdf)ucArg);

                if (pstTask == NULL) {
                    vRpiSendDebug("MENU: task preset id not found");
                    break;
                }

                /* 预热主摄（只发不等, 同 vRpiPhotoTake —— 没有结果帧可等）。
                 * 为什么排在这里: OLED 那三条任务预设（色球放置/激光射靶/人质抓取）
                 *   **第一步就是视觉步骤**, 预设内部一点窗口都没有; 唯一的窗口就是下面
                 *   那段「机械臂待命展开」—— 它是阻塞播完的（vArmPlayWait）, 好几秒,
                 *   Pi 正好在这段时间里把相机开好, 视觉步骤进来直接解码, 省掉实测 ~1.3s
                 *   的开相机时间（当场 15 帧预热 + V4L2 协商）。
                 * 已展开过时也照发: 那时脚本立刻进视觉步骤, Pi 侧 acquire 会**等这个预热
                 *   落地**（见 camera_lease 的 warming 事件）, 不会两边同时开同一台设备撞 EBUSY,
                 *   只是省不到时间 —— 所以这里没有分支的必要。 */
                vRpiSendTaskCmd(RPI_TASK_WARM_MAIN);

                /* ---- 前置条件: 机械臂「已展开」（2026-10-18 起统一走勾选逻辑）----
                 * 任务动作（抓球/打靶/抓人质）的内部位姿都假定臂已展开, 从折叠位直接
                 * 摆过去会撞桌子/撞车身。要不要自动做一次「机械臂待命展开」由
                 * **注册表 ucFlags bit0（预设栏「可重入的任务」勾选框）**决定 ——
                 * 与 STEP_SCRIPT_CALL 调入可重入预设是**同一条逻辑**
                 * （ucScriptEnsureArmDeployed: 没展开过才阻塞播一遍待命）。
                 * ⚠️ 所以新建一张 OLED 任务预设后, 记得在预设栏勾上「可重入的任务」,
                 *    否则执行时不会自动展开。14/15/16（色球放置/激光射靶/人质抓取）
                 *    已勾好。展不开（待命动作缺失/播完仍没置位）→ 拒绝执行任务。 */
                if (ucScriptEnsureArmDeployed(pstTask) == 0u) {
                    break;                          /* running 已在失败路径清 0 */
                }

                vScriptRun(pstTask->pstSteps, pstTask->usCount);
            }
            break;

        case SCRIPT_REQ_LINE_TEST:
            /* 「开始巡线测试」= 跑 SCRIPT_LINE_TEST_PRESET 那条预设整张表（见宏处的说明）。
             * 走预设而不是直接调原语, 就是为了能在控制台里给它自由加步骤。
             * ⚠️ 停止走的是 vScriptAbort()（菜单那条）→ 中止位在步骤边界生效, 而且
             *    巡线收敛那一步会在**自己的闭环循环里**再查一次（app_line.c）,
             *    所以就算那一步写了 16.7 分钟也能立刻停, 且来得及给 Pi 发 LINE_STOP。 */
            {
                const stScriptPresetTdf *pstLine = cpstFindScriptPreset(SCRIPT_LINE_TEST_PRESET);

                if (pstLine == NULL)
                {
                    /* 预设被删过/改过号 → 报给 Pi + 拒绝, 别静默什么都不做 */
                    vRpiSendDebug("LINE test: preset id not found");
                    break;
                }

                /* 起跑串: 排障用。菜单点了「开始巡线测试」但 Pi 那边只看到心跳时, 靠它一眼分清
                 * 两种失效 —— ① **没有**这条串 = 脚本任务压根没跑到这儿（请求没投上去 / 任务还堵在
                 * 别处, 例如更早的「握手让出」）；② 有这条串而 Pi 没收到 `"2"` = 发不出去。
                 * 每轮只发一条, 不违反「别在路线正跑时拉 LIST」那条。 */
                vRpiSendDebug("LINE test: start");
                vScriptRun(pstLine->pstSteps, pstLine->usCount);
            }
            break;

        case SCRIPT_REQ_GYRO_CAL:
            /* 陀螺校准（OLED「陀螺校准」, 2026-10 加）: 校准**有效源**那颗 —— M0 阻塞约
             * 21s / HWT101 约 7s, ATK 拒绝（函数内部留痕 + 置失败结果给 OLED 闪）。
             * 为什么非在本任务做: 校准期间没人 vSetYawFresh(), 巡线/陀螺闭环若在跑会判
             * IMU 停摆 —— 而本任务同一时刻不可能在跑路线（请求循环是串行的, 投递方
             * vScriptPostReq 还会先把旧路线中止掉）, 天然满足「不发布 yaw」的前提
             * （docs/sensors.md「零偏校准」一节的两条硬约束）。
             * 占住 script_running: 校准的 21s 里蓝牙别进机械臂编辑模式（同握手让出口径,
             * 保守 —— 校准虽然不动舵机, 但这 21s 也确实不该被任何长动作打扰）。 */
            vSetScriptRunning(1u);
            (void)ucGyroCalActiveSource();
            vSetScriptRunning(0u);
            break;

        default:
            break;
        }
    }
}
