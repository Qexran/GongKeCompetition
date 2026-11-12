/**
  * @file       app_chassis_action.c
  * @brief      底盘「动作组」的数据表 + 时间轴速度混合器播放器。
  *
  *             动作组 = 一段可命名的时间轴序列（世界系平移段 / 陀螺闭环转弯段 /
  *             臂事件 / 等臂事件）, 台架上用 Web 控制台「底盘动作组」Tab 排出来,
  *             存进下面的数据区, 然后作为**一块大积木**出现在「地图」Tab 积木库。
  *             路线里按 ID 引用: STEP_CHASSIS_ACTION_PLAY_WAIT; 蓝牙 $B,CPLAY 预览。
  *
  *             为什么用它代替「位置模式走一步→停→转一步」: 位置模式的精确来自
  *             脉冲数量化（开环）, 每步之间必有停车间隙, 世界系斜向还是先 X 后 Y
  *             的 L 形; 本播放器把整段时间轴连续起来 —— 一次 vChassisMoveA 接一次
  *             （20ms 一拍, 麦轮解算 speed[]=±vx±vy∓w 天然叠加）, 直走+转弯叠加
  *             就是弧线, 而且转弯带陀螺闭环（位置模式弧线纯标定、无反馈）。
  *
  *             ⚠️ 数据区由控制台读写（按行解析、未编辑的行原样回吐, 保证无操作
  *                保存零 diff）。手改可以, 但表头三行会被下次保存规范化。
  *
  *             播放刻意复用比赛脚本同一批原语（vChassisMoveA / vArmPlayNoWait /
  *             vChassisWorldToBodyEx / fChassisMoveTimeEst）—— 台架上预览看到的
  *             运动与路线跑起来逐字节同一条路径。
  *
  *             ==================== 航向权威（一拍只允许一个） ====================
  *             转弯段的闭环、平移段的航向保持, 都往同一个 w 通道写 —— 两个航向环
  *             相加等于增益翻倍（摆振）。所以每拍按优先级**选一个**:
  *               活跃且未收敛的转弯段（表序最低者）> 平移段的航向保持 > 无（w=0）。
  *             转弯段收敛后不再出力（保持层自然接管, 目标=翻转后的累加器）。
  */

#include "app_chassis_action.h"
#include "app_chassis.h"       /* vChassisMoveA / vChassisWorldToBodyEx / fChassisMoveTimeEst / AXIS_Y */
#include "app_arm_action.h"    /* vArmPlayNoWait */
#include "app_beam.h"          /* 航向/姿态/臂忙/中止位/本模块运行时量 */
#include "app_config.h"
#include "app_rpi.h"           /* vRpiSendDebug / RPI_DATA_MAX */
#include "app_bt.h"            /* bt_fmt_f2（浮点格式化, 全工程不用 %f） */
#include "app_actuator.h"      /* vBuzzerBeep（拒绝/超时长鸣） */

#include <stdio.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

/* ==================== 数据区（控制台整段重写） ==================== */
/* ==== 数据区: 开始 ==== */

/* ===== 预设: 示例: 前进末尾叠加右转 ===== */
/* 备注: 验证链路的样例: 前进 500mm, 末段叠加右转 90°（弧线收尾, 无停车间隙） */
const stChActItemTdf g_astChActItems1[] = {
    CH_ACT_MOVE     (0u, -50.0f, 50.0f, 251, 141, 1u),  /* 平移段 */
    CH_ACT_MOVE     (100u, 0.0f, 535.0f, 281, 171, 1u),  /* 平移段 */
    CH_ACT_TURN     (1140u, 0u, 90, 600, 1u, 240u),  /* 转弯段 */
    CH_ACT_MOVE     (1180u, 0.0f, 460.0f, 291, 171, 1u),  /* 平移段 */
    CH_ACT_PIVOT     (2220u, 0, 400, 1u),  /* 转弯段 */
    CH_ACT_MOVE     (2280u, 0.0f, 1020.0f, 80, 200, 1u),  /* 平移段 */
//    CH_ACT_MOVE     (4600u, 0.0f, 130.0f, 281, 171, 1u),  /* 平移段 */
    CH_ACT_TURN     (5100u, 0u, 90, 600, 1u, 240u),  /* 转弯段 */
};

/* ===== 预设: 底盘动作组 ===== */
/* 备注: */
const stChActItemTdf g_astChActItems2[] = {
    CH_ACT_MOVE     (0u, 0.0f, 300.0f, 150, 120, 1u),  /* 前进 */
};

const stChActGroupTdf g_astChActGroup[CH_ACT_GROUP_NUM] = {
    { 1u, "示例: 前进末尾叠加右转", g_astChActItems1, 7u },
    { 2u, "底盘动作组", g_astChActItems2, 1u },
};

/* ==== 数据区: 结束 ==== */

/* ==================== 查表 ==================== */

static const stChActGroupTdf *c_pstChActFind(emChActIdTdf emId)
{
    uint8_t ucI;

    /* 线性查找: 组数是个位数（同 c_pstFind 的理由） */
    for (ucI = 0u; ucI < CH_ACT_GROUP_NUM; ucI++)
    {
        if (g_astChActGroup[ucI].ucId == (uint8_t)emId)
        {
            return &g_astChActGroup[ucI];
        }
    }
    return NULL;
}

/* ==================== 台架草稿（$B,CPUSH 在线推送; 2026-10 加） ====================
 * 预览前由控制台把当前编辑的这一份推进来（API 见 .h）。写侧只有 vBtTask;
 * 播放器忙时直接拒收 —— 草稿不会在播放途中被改写, 预览任务读它是稳定的。
 * ⚠️ 路线/比赛**永远不读草稿**（vChassisActionPlayWait 只走 c_pstChActFind）:
 *    比赛必须跑编译进固件的那一份, 台架草稿断电即失（同 .h 的铁律注释）。 */
static stChActItemTdf  s_astChActTry[CH_ACT_ITEMS_MAX];
static const char      s_acChActTryTitle[] = "台架草稿";
static stChActGroupTdf s_stChActTry = { 0u, s_acChActTryTitle, s_astChActTry, 0u };
static uint8_t s_ucChActTryArmed = 0u;   /* 草稿可用（收满一次置 1; 下一次 Begin 清 0） */
static uint8_t s_ucCpWindow = 0u;        /* 剩余应收行数（0 = 没开窗） */

uint8_t ucChActTryBegin(uint8_t ucId, uint8_t ucCount)
{
    if (ucGetChActBusy() != 0u) { return 0u; }

    s_stChActTry.ucId    = ucId;
    s_stChActTry.usCount = 0u;
    s_ucCpWindow         = ucCount;
    s_ucChActTryArmed    = 0u;
    return 1u;
}

uint8_t ucChActTryStore(const stChActItemTdf *pstItem)
{
    if ((s_ucCpWindow == 0u) || (pstItem == NULL)) { return 0u; }
    if (ucGetChActBusy() != 0u) { return 0u; }

    s_astChActTry[s_stChActTry.usCount] = *pstItem;
    s_stChActTry.usCount++;
    s_ucCpWindow--;
    if (s_ucCpWindow == 0u)
    {
        s_ucChActTryArmed = 1u;              /* 收满 = 武装, CPLAY 可以命中了 */
    }
    return 1u;
}

const stChActGroupTdf *pstChActPreviewResolve(uint8_t ucId)
{
    if ((s_ucChActTryArmed != 0u) && (s_stChActTry.ucId == ucId))
    {
        return &s_stChActTry;
    }
    return c_pstChActFind((emChActIdTdf)ucId);
}

uint8_t ucChActPreviewExists(uint8_t ucId)
{
    return (uint8_t)((ucId != 0u) && (pstChActPreviewResolve(ucId) != NULL));
}

/* 短弧归一化到 ±180°（同 app_line.c / app_pid.c 的处理: 目标 −180、实测 +179
 * 不归一会算成 −359 → 朝错误方向满力甩车） */
static float c_fChActWrap180(float fVal)
{
    while (fVal > 180.0f)  { fVal -= 360.0f; }
    while (fVal < -180.0f) { fVal += 360.0f; }
    return fVal;
}

/* ==================== 播放器核心 ====================
 * 两个上下文共用（脚本任务的 vChassisActionPlayWait / 预览任务的 vChassisActionTask）:
 * 播放器只用局部变量 + const 表 + Beam, 可重入; 互斥靠「busy 标志先检后播」
 * （同刻双起的窗口只有一拍, 且两边写的都是同一个 1, 后果与 Beam 规则第 4 条
 * 的单字口径一致）。ucPreview 只影响中止判据（编辑模式掉线/路线抢跑要不要掐）。 */
static void c_vChActPlay(const stChActGroupTdf *pst, uint8_t ucPreview)
{
    uint16_t  usN = pst->usCount;
    uint8_t   aucFired[CH_ACT_ITEMS_MAX];      /* 事件已触发（MOVE=已激活过, 仅统计用） */
    uint16_t  ausDur[CH_ACT_ITEMS_MAX];        /* MOVE=现算时长; TURN=已转动时长（防故障计数, 见 ③） */
    uint16_t  ausRamp[CH_ACT_ITEMS_MAX];       /* MOVE=收尾降速斜坡[ms]; 其余=0 */
    float     afPeak[CH_ACT_ITEMS_MAX];        /* MOVE=本段峰值速度[rpm]（三角剖面 < 表速度）; 其余=0 */
    uint8_t   aucDone[CH_ACT_ITEMS_MAX];       /* TURN/PIVOT=已收敛/已被防故障兜底收账 */
    float     afTarget[CH_ACT_ITEMS_MAX];      /* TURN/PIVOT=绝对目标航向 */
    uint8_t   aucConv[CH_ACT_ITEMS_MAX];       /* TURN/PIVOT=连续进阈值拍数 */
    float     afLastErr[CH_ACT_ITEMS_MAX];     /* TURN/PIVOT=PID 微分项的上拍误差 */
    float     afInteg[CH_ACT_ITEMS_MAX];       /* TURN/PIVOT=PID 积分累加（±KI_MAX 限幅） */
    int8_t    acPivotSide[CH_ACT_ITEMS_MAX];   /* PIVOT=支点方向（激活时按最短路径定死: +1 驱右侧=左转） */
    uint32_t  ulTotal = 0u;
    uint32_t  ulT = 0u;                        /* 时间轴时钟[ms]（等臂/等转弯收敛期间冻结） */
    uint32_t  ulSkipped = 0u;                  /* 没配齐被跳过的 MOVE 段数（步末对账） */
    uint32_t  ulTurnMiss = 0u;                 /* 被防故障兜底收账的转弯段数（正常=0） */
    uint32_t  ulNoSrc = 0u;                    /* IMU 停摆拍数（航向权威整拍让位） */
    uint32_t  ulTelemAcc = 0u;
    uint8_t   ucWaiting = 0u;                  /* 等臂事件进行中 */
    uint8_t   ucArmBusySeen = 0u;              /* 等臂期间见过 busy=1（ARM 事件同刻起等时臂泵要过一拍才认领） */
    uint8_t   ucTurnPending = 0u;              /* 有转弯/支点段还没收敛 —— 组尾要等它 */
    uint8_t   ucPivotActive = 0u;              /* 有支点段在转 —— 车轮被它独占（平移不出力） */
    uint8_t   ucPivotDr = 0u;                  /* 当拍航向权威=支点段（⑦ 走 vChassisPivotDrive） */
    int8_t    cPivotSide = 0;                  /* 当拍支点方向（权威段的） */
    TickType_t xWaitTick = 0u;
    uint32_t  ulWaitLimitMs = 0u;
    uint8_t   ucAbort = 0u;
    uint8_t   ucI;
    uint8_t   ucJ;
    uint8_t   ucTurnAuth;                      /* 本拍转弯权威 = 表序最低的活跃未收敛段（③ 用） */

    /* 开播先置忙: 把「resolve 取表 → 预校验读完」整段罩住 —— CPUSH 改写草稿的
     * 入口（ucChActTryBegin/Store）都查 busy, 不先占坑的话预校验这个大循环期间
     * 草稿能被改写到一半（读到新旧混合的条目）。预校验的整组拒绝路径各自退账。 */
    vSetChActBusy(1u);
    vSetChActPlayingId(pst->ucId);

    /* ---- IMU 新鲜度（帧号法, 与 app_line.c 同一套）: 停摆那一拍航向权威不出力,
     *      否则误差冻结成常数 w 会一直转。每拍跟踪, 不只转弯段才看。 ---- */
    uint32_t   ulYawCnt   = ulGetYawFrameCount();
    TickType_t xYawTick   = xTaskGetTickCount();
    uint8_t    ucYawStale = 0u;

    /* ---- 预校验 + 时长预算 ---- */
    if (usN > (uint16_t)CH_ACT_ITEMS_MAX)
    {
        /* 条目超上限: 整组拒绝（截着播等于时间轴后半段无声丢失） */
        vBuzzerBeep(1000u, 0u, 1u);
        vRpiSendDebug("CA fail: too many items");
        vSetChActBusy(0u);                    /* 开播时占的坑退账 */
        vSetChActPlayingId(0u);
        return;
    }

    for (ucI = 0u; ucI < (uint8_t)usN; ucI++)
    {
        const stChActItemTdf *p = &pst->pstItems[ucI];

        aucFired[ucI] = 0u;
        aucDone[ucI]  = 0u;
        aucConv[ucI]  = 0u;
        afTarget[ucI] = 0.0f;
        afLastErr[ucI] = 0.0f;
        afInteg[ucI]   = 0.0f;
        acPivotSide[ucI] = 0;
        ausDur[ucI]   = 0u;                    /* MOVE 稍后现算; TURN/PIVOT=防故障计数从 0 起 */
        ausRamp[ucI]  = 0u;
        afPeak[ucI]   = 0.0f;

        if (p->ucKind == CH_ACT_KIND_MOVE)
        {
            float fDist = sqrtf((p->fWx * p->fWx) + (p->fWy * p->fWy));

            if ((fDist < 0.5f) || (p->nSpeed <= 0))
            {
                /* 没配齐: 跳过这一段（标记已触发=永不激活）, 组照播 —— 与
                 * 「巡线直走」整步拒绝不同: 时间轴上其它段不该陪着陪葬。 */
                aucFired[ucI] = 1u;
                ulSkipped++;
                continue;
            }

            /* 时长 = 位置模式同款梯形估时 × 时间系数 k（运行时量, 蓝牙当场改、
             * 下一次播放生效）。估时与斜坡用**同一个**加速度档 —— 实效档 = 表值,
             * 表值 0 = 跟随全局底盘加速度（同 ⑦ 下发口径; 0 直接进估时公式会按
             * 256 算, 距离对不上, 同巡线直走; 全局也为 0 时钳 1 保底）。
             * 对齐到 20ms 网格（本播放器的拍）。
             * 同时算出**速度剖面**（与估时公式同形, 2026-10 加）:
             *   长行程梯形 —— time_fenjie 起步/收尾斜坡 + 全速巡航;
             *   短行程三角 —— 半程爬/半程降, 峰值 = 速度×sqrt(q/qf)。
             * 老行为全程恒速、窗口一到命令 0 速 —— 减速被挤到窗口之外, 观感是
             * "时间一到急刹", 刹车滑行还会掠过目标点。现在窗口两端速度归零, 车滑行
             * 到点停住（剖面内少走的距离由 k 吸收, 短了调大 k 即可）。
             * **交叠接续**: 起步/收尾斜坡对称后, 同速同向交叠 = 降/升互补、合速度
             * 恒定无缝; 异速交叠 = 线性过渡; 异向交叠 = 方向圆滑旋转（弧线）。
             * 交叠量建议 ≥ 斜坡时长（时间轴上把后一段的起点拖进前一段的尾段）。 */
            {
                uint16_t usAcc = (uint16_t)((p->ucAccel != 0u)
                                            ? p->ucAccel
                                            : (uint8_t)iGetChassisAccel());

                if (usAcc < 1u) { usAcc = 1u; }
                float fSp = (float)p->nSpeed;
                float fAc = (float)usAcc;
                float fQ  = fDist * CHASSIS_PULSE_PER_MM_Y;   /* 同估时公式的脉冲记账 */
                float fQf = 320.0f * fSp * fSp / (fAc * 6.0f);
                float fEst = fChassisMoveTimeEst(fDist, usAcc,
                                                 (uint16_t)p->nSpeed, AXIS_Y)
                             * fGetChActTimeScale();
                uint32_t ulMs = (uint32_t)fEst + 1u;

                ulMs -= ulMs % (uint32_t)CH_ACT_LOOP_MS;
                if (ulMs < (uint32_t)CH_ACT_LOOP_MS) { ulMs = (uint32_t)CH_ACT_LOOP_MS; }
                if (ulMs > 65535u) { ulMs = 65535u; }   /* uint16 槽位: 超长慢速段钳满, 别回绕成提前散场 */
                ausDur[ucI] = (uint16_t)ulMs;

                if (fQf >= fQ)        /* 三角: 行程短于"加减速本身就够走完"的量 */
                {
                    ausRamp[ucI] = (uint16_t)(ulMs / 2u);
                    afPeak[ucI]  = fSp * sqrtf(fQ / fQf);
                }
                else                  /* 梯形: 斜坡 time_fenjie, 中段全速巡航 */
                {
                    float fR = 2000.0f * fSp / fAc;

                    if (fR > (float)ulMs) { fR = (float)ulMs; }   /* 退化: 全窗斜坡 */
                    ausRamp[ucI] = (uint16_t)fR;
                    afPeak[ucI]  = fSp;
                }
            }
        }
        else if ((p->ucKind == CH_ACT_KIND_TURN) || (p->ucKind == CH_ACT_KIND_PIVOT))
        {
            /* 非 ±90 倍数 = 会把世界航向累加器带离象限点, 整组拒绝（防呆承诺:
             * 累加器永远落在 90° 网格上, 后续世界系查表才成立）。
             * 相对档不许 0（不转白占一段）, 绝对档**允许 0**（咬回开机朝向是有意义的）。
             * PIVOT（支点转弯）与 TURN 同一套角度/记账纪律。 */
            if (((p->nDelta % 90) != 0) || (p->nDelta > 180) || (p->nDelta < -180)
                || ((p->nDelta == 0) && (p->ucAbs == 0u)))
            {
                vBuzzerBeep(1000u, 0u, 1u);
                vRpiSendDebug("CA fail: turn not x90");
                vSetChActBusy(0u);            /* 开播时占的坑退账 */
                vSetChActPlayingId(0u);
                return;
            }
            /* ⚠️ **转弯段没有段上限**（2026-10 取消, 车主定案）: 闭环的自然语义就是
             * 「转到位为止」, 组尾会等它收敛（见 ⑨ 的 ucTurnPending）; 到点硬记账的
             * 上限只会制造蜂鸣和航向残差。防故障兜底见 ③（CH_ACT_TURN_MS, 只在
             * IMU 新鲜仍无法收敛的真故障时触发, 正常转弯永远碰不到）。 */
        }

        /* 组尾预算: 平移段 = 起点+时长; 转弯/支点段时长未知（转到位为止）,
         * 只把**起点**纳进预算, 收敛等待由 ⑨ 的 ucTurnPending 兜住。 */
        if ((p->ucKind == CH_ACT_KIND_TURN) || (p->ucKind == CH_ACT_KIND_PIVOT))
        {
            if ((uint32_t)p->usStart > ulTotal)
            {
                ulTotal = (uint32_t)p->usStart;
            }
        }
        else if (((uint32_t)p->usStart + (uint32_t)ausDur[ucI]) > ulTotal)
        {
            ulTotal = (uint32_t)p->usStart + (uint32_t)ausDur[ucI];
        }
    }

    /* （置忙已提到函数最顶上 —— 预校验期间也要挡 CPUSH 改写草稿） */

    while (1)
    {
        /* ---- ① 中止判据（每拍逐条, 谁来都立刻零速收尾） ---- */
        if (ucGetScriptAbortReq() != 0u) { ucAbort = 1u; break; }
        if (ucTakeChActStopReq() != 0u)  { ucAbort = 1u; break; }
        /* 预览专属的两条让位判据。⚠️ **不看 ucGetScriptRunning() 本身**: 路线带
         * STEP_SCRIPT_RESET 自动循环 / 上电即跑活动预设的机器, 这个标志从上电起
         * 几乎恒为 1（停在「等握手/按主按键」时也是 1）—— 拿它当判据 = 预览开播
         * 一拍就自停（台架实测SCRIPTRUN/秒退的根因）。真正说明「路线要动了」的是:
         *   · 编辑模式被清 —— 三种来路都会清它: Pi 重连交还（vBtEditPump）、
         *     新一轮 vScriptRun 开跑（进场先 vSetBtEditMode(0)）、PC 掉线看门狗;
         *   · 让位窗关闭**且**路线还活着 —— vBtArmEditHold 按 MAIN 退出时只关窗
         *     （bt_hs_wait=0）**不清**编辑模式位, 路线马上接着跑, 靠这条兜住。
         *   脚本真没跑（idle 进的编辑模式）时 hs_wait 本来就是 0, 这条不误伤。 */
        if (ucPreview != 0u)
        {
            if (ucGetBtEditMode() == 0u)
            {
                ucAbort = 1u;
                break;
            }
            if ((ucGetBtHsWait() == 0u) && (ucGetScriptRunning() != 0u))
            {
                ucAbort = 1u;
                break;
            }
        }

        /* ---- ② 等臂事件: 时间轴冻结, 零速 + 航向保持（车停着但鼻子不许歪） ---- */
        if (ucWaiting != 0u)
        {
            uint32_t ulWaited = (uint32_t)((xTaskGetTickCount() - xWaitTick)
                                           * portTICK_PERIOD_MS);

            if (ucGetArmSeqBusy() != 0u)
            {
                ucArmBusySeen = 1u;                /* 臂泵已认领 */
            }
            /* ARM 事件与 ARM_WAIT 同刻触发时, 臂泵要过一拍（任务节拍对齐前）才把
             * busy 抬起来 —— 没见过 1 就放行会把「等臂」空放成 NO_WAIT。宽限
             * CH_ACT_ARM_GRACE_MS 内不判「跑空」; 超时上限照旧优先。 */
            if ((ucGetArmSeqBusy() == 0u)
                && ((ucArmBusySeen != 0u)
                    || (ulWaited >= (uint32_t)CH_ACT_ARM_GRACE_MS)))
            {
                ucWaiting = 0u;                    /* 臂泵跑空: 恢复时钟 */
            }
            else if (ulWaited >= ulWaitLimitMs)
            {
                vBuzzerBeep(1000u, 0u, 1u);        /* 超时放行但必须响（同臂 drain） */
                vRpiSendDebug("CA arm wait: timeout");
                ucWaiting = 0u;
            }
        }

        if (ucWaiting == 0u)
        {
            ucTurnPending = 0u;
            ucPivotActive = 0u;
            /* 本拍转弯权威 = 表序最低的已触发未收敛段（选取规则同 ⑤）:
             * 防故障兜底钟只给权威段计时 —— 排队等前一段的后段根本没在转,
             * 不该被扣「转了 4s 还没到位」的帽子（前一段耗 3s, 后段就只剩 1s
             * 就被强制收账, 且会把激活时快照的旧目标写进世界航向累加器）。 */
            ucTurnAuth = 0xFFu;
            for (ucJ = 0u; ucJ < (uint8_t)usN; ucJ++)
            {
                const stChActItemTdf *pq = &pst->pstItems[ucJ];

                if (((pq->ucKind == CH_ACT_KIND_TURN)
                     || (pq->ucKind == CH_ACT_KIND_PIVOT))
                    && (aucFired[ucJ] != 0u) && (aucDone[ucJ] == 0u))
                {
                    ucTurnAuth = ucJ;
                    break;
                }
            }
            /* ---- ③ 事件触发/到期收账（ulT >= usStart 的条目逐个处理） ---- */
            for (ucI = 0u; ucI < (uint8_t)usN; ucI++)
            {
                const stChActItemTdf *p = &pst->pstItems[ucI];

                if (aucFired[ucI] != 0u)
                {
                    if (((p->ucKind == CH_ACT_KIND_TURN)
                         || (p->ucKind == CH_ACT_KIND_PIVOT))
                        && (aucDone[ucI] == 0u))
                    {
                        /* 转弯/支点段**没有段上限** —— 转到位为止, 组尾等它
                         * （ucTurnPending）。ausDur 复用为「已转动时长」的防故障
                         * 计数: 只在 IMU 新鲜仍无法收敛的真故障（机械打滑/增益
                         * 配错）时触发 —— 写指令目标 + 长鸣放行（世界航向记账原则:
                         * 永远写指令目标, 残差留给视觉锚点）。正常永远碰不到它。
                         * ⚠️ 只给**本拍权威段**（ucTurnAuth）计时, 见上面的预扫。 */
                        if (ucI == ucTurnAuth)
                        {
                            ausDur[ucI] = (uint16_t)(ausDur[ucI] + (uint16_t)CH_ACT_LOOP_MS);
                            if (ausDur[ucI] >= (uint16_t)CH_ACT_TURN_MS)
                            {
                                vSetWorldHeading(afTarget[ucI]);
                                aucDone[ucI] = 1u;
                                ulTurnMiss++;
                                vBuzzerBeep(1000u, 0u, 1u);
                            }
                        }
                        if (aucDone[ucI] == 0u)
                        {
                            ucTurnPending = 1u;    /* 仍未收敛: 组尾等它（每拍重申） */
                        }
                    }
                    if ((p->ucKind == CH_ACT_KIND_PIVOT) && (aucDone[ucI] == 0u))
                    {
                        ucPivotActive = 1u;        /* 支点在转: 车轮独占（④ 平移不出力） */
                    }
                    continue;
                }
                if (ulT < (uint32_t)p->usStart) { continue; }

                aucFired[ucI] = 1u;
                switch (p->ucKind)
                {
                case CH_ACT_KIND_ARM:
                    /* 到点投臂（后写者胜, 与 STEP_ARM_PLAY_NO_WAIT 同语义）;
                     * 臂自己的泵推进, 本播放器不等它 —— 要等排 ARM_WAIT 事件。 */
                    vArmPlayNoWait((emArmActionIdTdf)p->ucArmId);
                    break;

                case CH_ACT_KIND_ARM_WAIT:
                    ucWaiting    = 1u;
                    xWaitTick    = xTaskGetTickCount();
                    ulWaitLimitMs = (uint32_t)p->usDur;   /* 表里的「等臂上限」usDur（2026-10-31 修:
                                                           * 原来误读 ausDur —— ARM_WAIT 的槽恒 0,
                                                           * 表值从没生效过, 一直在吃宏默认 60s） */
                    if (ulWaitLimitMs == 0u)
                    {
                        ulWaitLimitMs = (uint32_t)CH_ACT_ARM_WAIT_MS;   /* 表里 0 = 用宏默认 */
                    }
                    break;

                case CH_ACT_KIND_TURN:
                case CH_ACT_KIND_PIVOT:
                    /* 激活快照。相对档: 目标 = 当前累加器 ± delta（此刻累加器还在旧象限
                     * —— 转弯进行中平移按旧象限映射 = 车体方向不变 = 弧线几何）;
                     * 绝对档: 目标 = delta 本身（前面的段把记账带偏后用它拉回已知朝向）。
                     * 两档收尾都写指令目标, 记账纪律一致。
                     * PID 状态清零 + 上拍误差**按当前误差初始化**（不然首拍的
                     * D 项 = err*KD*系数, 满力一记, 车会往前抽一下）。
                     * PIVOT 额外定死**支点方向**: 初始误差 ≥0（需左转）→ 驱右侧
                     * （iSide=+1）; <0 → 驱左侧。之后每拍只用带符号驱动速度纠正,
                     * 过冲反转不换支点边（同 vPidPivotTurn 的 proven 行为）。 */
                    if (p->ucAbs != 0u)
                    {
                        afTarget[ucI] = c_fChActWrap180((float)p->nDelta);
                    }
                    else
                    {
                        afTarget[ucI] = c_fChActWrap180(fGetWorldHeading()
                                                        + (float)p->nDelta);
                    }
                    afInteg[ucI]   = 0.0f;
                    afLastErr[ucI] = c_fChActWrap180(afTarget[ucI]
                                                     - fGetAttitudeYaw());
                    acPivotSide[ucI] = (afLastErr[ucI] >= 0.0f) ? 1 : -1;
                    ucTurnPending = 1u;   /* ⚠️ 当拍就要登记: 否则纯转弯组/转弯起点≥
                                           * 其它段总长的组, 会在触发的同一拍满足结束
                                           * 条件直接散场 —— 车一拍都不转（台架实测）。 */
                    if (p->ucKind == CH_ACT_KIND_PIVOT)
                    {
                        ucPivotActive = 1u;
                    }
                    break;

                default:
                    break;     /* MOVE: 窗口判定见 ④, 无激活动作 */
                }
            }
        }

        /* ---- ④ 平移叠加（世界系向量求和 → 象限表映射; 查表函数内部只读一次
         *      累加器, 每拍一次符合「同一动作只查一次」纪律） ---- */
        {
            float fWxSum = 0.0f;
            float fWySum = 0.0f;
            float fSide  = 0.0f;
            float fFwd   = 0.0f;
            uint8_t ucAccUse = 0u;      /* 平移段候选: 活跃段的最高档 */
            uint8_t ucTurnAcc = 0u;     /* 转弯段候选: 表序最低的活跃未收敛段自带档（>0 才算） */
            uint8_t ucAnyHold = 0u;

            for (ucI = 0u; ucI < (uint8_t)usN; ucI++)
            {
                const stChActItemTdf *p = &pst->pstItems[ucI];

                if ((p->ucKind != CH_ACT_KIND_MOVE) || (aucFired[ucI] == 0u))
                {
                    continue;
                }
                /* 预校验判「没配齐」跳过的段 afPeak 恒 0 —— 必须在这里挡掉, 不然
                 * wx=wy=0 的行 fDist=0, (0/0)*0 = NaN 污染下面的平移和（⑥ 的钳与
                 * ⑦ 的整型化对 NaN 全失效 → 整组平移无声归零, 直到走过它的 start）。
                 * 顺带把 hold 登记也挡在门外（跳过段不该申请航向保持）。 */
                if (afPeak[ucI] == 0.0f) { continue; }
                if (ulT >= ((uint32_t)p->usStart + (uint32_t)ausDur[ucI]))
                {
                    continue;                       /* 窗口已过 */
                }
                /* 支点段独占车轮: 平移不出力（支点侧要保持力, 平移会把它推走） */
                if (ucPivotActive != 0u) { continue; }

                /* 等臂冻结期: 不出平移（车要停住）, 但窗口内的段仍登记航向保持
                 * —— 停着等的时候鼻子不许歪（④ 的零速 + 保持正是这个组合）。 */
                if (p->ucHold != 0u) { ucAnyHold = 1u; }
                if (ucWaiting != 0u) { continue; }

                {
                    float fDist = sqrtf((p->fWx * p->fWx) + (p->fWy * p->fWy));

                    /* 速度剖面（见预校验注释）: 起步/收尾斜坡**对称** —— 已过或剩余
                     * 时间不足斜坡时按比例折减, 窗口两端速度 ≈ 0。
                     * 交叠接续（2026-10 加起步斜坡后才成立）: 同速同向交叠 = 两段的
                     * 降/升互补, 合速度**恒定无缝**; 异速交叠 = 两速度间线性过渡;
                     * 不同方向交叠 = 方向圆滑旋转（弧线）。 */
                    float fFrac  = 1.0f;
                    uint32_t ulLeft    = ((uint32_t)p->usStart + (uint32_t)ausDur[ucI])
                                         - ulT;
                    uint32_t ulElapsed = ulT - (uint32_t)p->usStart;

                    if (ulLeft < (uint32_t)ausRamp[ucI])
                    {
                        fFrac = (float)ulLeft / (float)ausRamp[ucI];
                    }
                    if ((uint32_t)ausRamp[ucI] != 0u)
                    {
                        float fUp = (float)ulElapsed / (float)ausRamp[ucI];

                        if (fUp < fFrac) { fFrac = fUp; }
                    }
                    fWxSum += (p->fWx / fDist) * afPeak[ucI] * fFrac;
                    fWySum += (p->fWy / fDist) * afPeak[ucI] * fFrac;
                    if (p->ucAccel > ucAccUse) { ucAccUse = p->ucAccel; }
                }
            }

            /* IMU 新鲜度（每拍跟踪, 采信只在航向权威里） */
            {
                uint32_t ulNowYaw = ulGetYawFrameCount();

                if (ulNowYaw != ulYawCnt)
                {
                    ulYawCnt   = ulNowYaw;
                    xYawTick   = xTaskGetTickCount();
                    ucYawStale = 0u;
                }
                else if ((CH_ACT_GYRO_STALE_MS != 0u)
                         && ((xTaskGetTickCount() - xYawTick)
                             > pdMS_TO_TICKS(CH_ACT_GYRO_STALE_MS)))
                {
                    ucYawStale = 1u;
                }
            }

            /* ---- ⑤ 航向权威（一拍一个: 转弯/支点段[表序最低且未收敛] > 保持 > 无;
             *      支点段权威时**独占车轮**——④ 已让平移让位, ⑦ 走单侧驱动) ---- */
            {
                float fW = 0.0f;
                uint8_t ucAuth = 0u;    /* 遥测: 0=无 1=保持 2=转弯 3=支点 */

                if (ucYawStale == 0u)
                {
                    ucPivotDr = 0u;
                    for (ucI = 0u; ucI < (uint8_t)usN; ucI++)
                    {
                        const stChActItemTdf *p = &pst->pstItems[ucI];
                        float fErr;
                        float fWMax = fGetChActWMax();
                        uint8_t ucIsPivot = (p->ucKind == CH_ACT_KIND_PIVOT) ? 1u : 0u;

                        /* 活跃 = 已触发且未收敛（无段上限, 转到位为止） */
                        if (((p->ucKind != CH_ACT_KIND_TURN)
                             && (p->ucKind != CH_ACT_KIND_PIVOT))
                            || (aucFired[ucI] == 0u)
                            || (aucDone[ucI] != 0u)
                            || (ucWaiting != 0u))
                        {
                            continue;
                        }
                        /* 段内限幅: 表值 >0 = **本段权威**（直接用它, 盖过全局
                         * ch_w_max —— 段上抬速度必须看得见效果, 取 min 的老语义
                         * 会让"抬了段值却动不了"像推送没生效）; ≤0 = 用全局。
                         * PIVOT 的 fWMax 是**驱动侧**轮速, 出厂值 = PIVOT_TURN_MAX_SPEED;
                         * 车轮物理上界仍由 ⑥ 的 ch_wheel_max 兜底。 */
                        if (p->nSpeed > 0)
                        {
                            fWMax = (float)p->nSpeed;
                        }
                        else if (ucIsPivot != 0u)
                        {
                            fWMax = fGetChPivotWMax();
                        }
                        /* 本段自带加速度档（>0 才算, 表序最低的活跃段优先）:
                         * 转弯的斜坡平滑是打滑命门, 设了就盖过平移段/全局。 */
                        if ((p->ucAccel > 0u) && (ucTurnAcc == 0u))
                        {
                            ucTurnAcc = p->ucAccel;
                        }

                        fErr = c_fChActWrap180(afTarget[ucI] - fGetAttitudeYaw());
                        if ((fErr <= (float)CH_ACT_CONV_DEG)
                            && (fErr >= -(float)CH_ACT_CONV_DEG))
                        {
                            if (++aucConv[ucI] >= (uint8_t)CH_ACT_CONV_TICKS)
                            {
                                /* 收敛: 累加器翻到新象限, 本段余下时长出零力
                                 * （保持层自然接管, 目标=翻转后的累加器）。
                                 * PID 状态一并清零（段已结束, 别把积分漏给下一次）。 */
                                vSetWorldHeading(afTarget[ucI]);
                                aucDone[ucI] = 1u;
                                afInteg[ucI]   = 0.0f;
                                afLastErr[ucI] = 0.0f;
                                break;
                            }
                        }
                        else
                        {
                            aucConv[ucI] = 0u;
                        }

                        if (ucIsPivot != 0u)
                        {
                            /* ---- 支点分支 ----
                             * 收敛带内驱动侧**出 0**（带内继续出力必冲过头, 驱动侧有
                             * 惯性, 同 vPidPivotTurn 的 proven 行为）; 出带才算数。
                             * PID 算式与转弯段同一条, 增益换 ch_pivot_*
                             * （出厂值 = PIVOT_TURN_*, 现场标定）。 */
                            if (aucConv[ucI] > 0u)
                            {
                                fW = 0.0f;
                            }
                            else
                            {
                                afInteg[ucI] += fErr * fGetChPivotKi();
                                if (afInteg[ucI] >  fGetChPivotKiMax())
                                {
                                    afInteg[ucI] = fGetChPivotKiMax();
                                }
                                if (afInteg[ucI] < -fGetChPivotKiMax())
                                {
                                    afInteg[ucI] = -fGetChPivotKiMax();
                                }
                                fW = fErr * fGetChPivotKp()
                                   + (fErr - afLastErr[ucI]) * fGetChPivotKd()
                                   + afInteg[ucI];
                                afLastErr[ucI] = fErr;
                                if (fW >  fWMax) { fW =  fWMax; }
                                if (fW < -fWMax) { fW = -fWMax; }
                            }
                            ucAuth = 3u;
                            ucPivotDr = 1u;             /* ⑦ 走单侧驱动 */
                            cPivotSide = acPivotSide[ucI];
                            break;                      /* 表序最低的活跃段独占 */
                        }

                        /* ---- 转弯段: PID → 混合器 w 通道 ----
                         * （与支点/原地转弯**同一条算式与口径**:
                         *  integral += err*KI（±KI_MAX 限幅）,
                         *  out = err*KP + (err−上拍err)*KD + integral ——
                         *  调参直觉可互换。⚠️ 本拍 20ms vs 那边 50ms:
                         *  同名增益出力更猛, 从那边抄数值先除 2~3 试。
                         *  纯 P 在目标附近指数减速拖尾（"感觉转得慢"的根源）,
                         *  到位慢加 KI, 超调配 KD/减 KP。） */
                        afInteg[ucI] += fErr * fGetChActHeadKi();
                        if (afInteg[ucI] >  fGetChActHeadKiMax())
                        {
                            afInteg[ucI] = fGetChActHeadKiMax();
                        }
                        if (afInteg[ucI] < -fGetChActHeadKiMax())
                        {
                            afInteg[ucI] = -fGetChActHeadKiMax();
                        }
                        fW = fErr * fGetChActHeadKp()
                           + (fErr - afLastErr[ucI]) * fGetChActHeadKd()
                           + afInteg[ucI];
                        afLastErr[ucI] = fErr;
                        if (fW >  fWMax) { fW =  fWMax; }
                        if (fW < -fWMax) { fW = -fWMax; }
                        ucAuth = 2u;
                        break;                      /* 表序最低的活跃转弯段独占 */
                    }

                    if ((ucAuth == 0u) && (ucAnyHold != 0u))
                    {
                        /* 航向保持: 咬累加器（转弯翻转之间它是常量, 不必快照）。
                         * 死区内不出力（纯 P + 死区, 同巡线陀螺腿的结构）。 */
                        float fErr = c_fChActWrap180(fGetWorldHeading()
                                                     - fGetAttitudeYaw());
                        float fWMax = fGetChActWMax();
                        float fDb   = fGetChActHeadDb();

                        if ((fErr > fDb) || (fErr < -fDb))
                        {
                            fW = fErr * fGetChActHeadKp();
                            if (fW >  fWMax) { fW =  fWMax; }
                            if (fW < -fWMax) { fW = -fWMax; }
                            ucAuth = 1u;
                        }
                    }
                }
                else
                {
                    ulNoSrc++;       /* 排障先看它: 这个数大 = 整段没在修航向 */
                }

                /* ---- ⑥ 车体系直通（2026-10 改, 车主定案）+ 饱和钳 ----
                 * 段向量按**车体坐标系**理解: fWxSum=左右(正=左), fWySum=前后(正=前),
                 * **不再过世界系象限表** —— "什么方向就是什么方向", 摆位/搬车/转弯
                 * 都不影响平移方向, 最好控制。转弯叠平移 = 车体方向随转弯旋转,
                 * 即"边前进边转弯"的驾驶直觉（弧线）; 世界系语义只存在于转弯目标
                 * 与保持层。 ****/
                fSide = fWxSum;
                fFwd  = fWySum;
                {
                    float fWheelMax = fGetChActWheelMax();
                    float fMag = ((fSide >= 0.0f) ? fSide : -fSide)
                               + ((fFwd  >= 0.0f) ? fFwd  : -fFwd);
                    float fWmag = (fW >= 0.0f) ? fW : -fW;

                    if (fWmag > fWheelMax)
                    {
                        fW = (fW >= 0.0f) ? fWheelMax : -fWheelMax;
                        fWmag = fWheelMax;
                    }
                    if (fMag + fWmag > fWheelMax)
                    {
                        float fRoom = fWheelMax - fWmag;
                        float fScale = (fMag > 0.001f) ? (fRoom / fMag) : 0.0f;

                        if (fScale < 0.0f) { fScale = 0.0f; }
                        fSide *= fScale;
                        fFwd  *= fScale;
                    }
                }

                /* ---- ⑦ 下发（一拍一次） ----
                 * 支点段权威 → **单侧驱动**独占车轮（fW=带符号驱动侧轮速,
                 * ×iPivotSide 换成「该侧前进/后退」—— 过冲反转不换支点边,
                 * 同 vPidPivotTurn; 收敛带内 fW=0 = 驱动侧停转保持支点）;
                 * 否则混合器一次 vChassisMoveA（accel 取档: 活跃转弯段自带档 >
                 * 活跃平移段最高档 > 全局底盘加速度）。 */
                if (ucPivotDr != 0u)
                {
                    /* 驱动侧**不插值**: PID 阶梯本身就是平滑（同 vPidPivotTurn 的
                     * PIVOT_TURN_ACCEL=0 —— EMM 再叠斜坡会双重平滑、响应变钝）。
                     * 拍末恢复用户的全局值, 出支点后平移/转弯斜坡照旧。 */
                    int iAccSaved = iGetChassisAccel();

                    vChassisSetAccel(0);
                    /* × PIVOT_TURN_YAW_SIGN: 与 vPidPivotTurn 的输出侧同一条符号链
                     * （yaw 装反改 -1 时两边一起翻, 别让底盘动作组的支点段掉队） */
                    vChassisPivotDrive(cPivotSide,
                                       (int)(fW * (float)cPivotSide * PIVOT_TURN_YAW_SIGN));
                    vChassisSetAccel(iAccSaved);
                }
                else
                {
                    vChassisMoveA((int)fSide, (int)fFwd, (int)fW,
                                  (ucTurnAcc != 0u) ? (int)ucTurnAcc :
                                  ((ucAccUse != 0u) ? (int)ucAccUse
                                                    : (int)iGetChassisAccel()));
                }

                /* ---- ⑧ 遥测（限速节流; a=0 无权威/1 保持/2 转弯） ---- */
                ulTelemAcc += CH_ACT_LOOP_MS;
                if ((CH_ACT_TELEM_MS != 0u) && (ulTelemAcc >= (uint32_t)CH_ACT_TELEM_MS))
                {
                    char acTlm[RPI_DATA_MAX];
                    char acVx[12];
                    char acVy[12];
                    char acH[12];

                    ulTelemAcc = 0u;
                    bt_fmt_f2(acVx, (int)sizeof(acVx), fSide);
                    bt_fmt_f2(acVy, (int)sizeof(acVy), fFwd);
                    bt_fmt_f2(acH,  (int)sizeof(acH),  c_fChActWrap180(
                              fGetWorldHeading() - fGetAttitudeYaw()));
                    (void)snprintf(acTlm, sizeof(acTlm),
                                   "CA: t=%u.%u vx=%s vy=%s w=%d h=%s a=%u%s",
                                   (unsigned)(ulT / 1000u),
                                   (unsigned)((ulT % 1000u) / 100u),
                                   acVx, acVy, (int)fW, acH, (unsigned)ucAuth,
                                   (ucYawStale != 0u) ? "!" : "");
                    vRpiSendDebug(acTlm);
                }
            }
        }

        /* ---- ⑨ 结束判定 + 节拍（20ms 一拍; 等臂/等转弯收敛期间时钟不走但照常下发零速） ----
         * 转弯段无段上限: 时钟走过组的名义总长后, 只要还有转弯段没收敛就继续转
         * （ucTurnPending）, 直到收敛/防故障兜底 —— 组的"尾段"就是最后一个转弯的
         * 收敛过程。 */
        if ((ucWaiting == 0u) && (ucTurnPending == 0u) && (ulT >= ulTotal)) { break; }
        vTaskDelay(pdMS_TO_TICKS(CH_ACT_LOOP_MS));
        if (ucWaiting == 0u) { ulT += (uint32_t)CH_ACT_LOOP_MS; }
    }

    /* ---- 收尾: 零速 + 清忙 + 步末对账 ---- */
    vChassisMoveA(0, 0, 0, (int)iGetChassisAccel());
    vSetChActPlayingId(0u);
    vSetChActBusy(0u);

    {
        char acEnd[RPI_DATA_MAX];

        (void)snprintf(acEnd, sizeof(acEnd),
                       "CA end: t=%ums lim=%ums sk=%u tn=%u na=%u%s",
                       (unsigned)ulT, (unsigned)ulTotal,
                       (unsigned)ulSkipped, (unsigned)ulTurnMiss,
                       (unsigned)ulNoSrc, (ucAbort != 0u) ? " abort" : "");
        vRpiSendDebug(acEnd);
    }
    if (ucAbort != 0u)
    {
        vRpiSendDebug("CA abort: stop by request");
    }
}

/* ==================== 播放入口 ==================== */

void vChassisActionPlayWait(emChActIdTdf emId)
{
    /* ⚠️ 路线播放**刻意只查 FLASH 编译表**（不走 pstChActPreviewResolve）:
     *    比赛必须跑编译进固件的那一份, 台架草稿绝不渗进路线（见 .h 铁律注释）。
     *    草稿里改好的组要上路线 = 控制台保存 + Keil 重编译 + 烧录。 */
    const stChActGroupTdf *pst = c_pstChActFind(emId);

    if (pst == NULL)
    {
        /* 响出来（同 .h 承诺）: 正常排表拿不到不存在的 id（下拉来自同一份生成块）,
         * 走到这就是表/枚举不同步（半烧录等）—— 静默跳步最难查。 */
        vBuzzerBeep(1000u, 0u, 1u);
        vRpiSendDebug("CA fail: no id");
        return;
    }

    /* 互斥: 预览正在播（或另一条已在播）→ 长鸣拒绝, 不动车 —— 两个速度流同时
     * 写 EMM 等于互抢, 「响出来」比静默顶掉诚实（同臂队列满的哲学）。 */
    if (ucGetChActBusy() != 0u)
    {
        vBuzzerBeep(1000u, 0u, 1u);
        vRpiSendDebug("CA fail: player busy");
        return;
    }

    c_vChActPlay(pst, 0u);
}

/* ==================== 蓝牙预览任务 ====================
 * 为什么不像机械臂 $B,PLAY 那样阻塞在 vBtTask 里播: 底盘在跑的时候必须留一条
 * 随时能进的 $B,CSTOP（vBtTask 阻塞期间收不到任何指令, 机械臂无所谓、车不行）。
 * 于是 vBtTask 只投请求就返回, 播放在本任务里走; PC 端轮询 ch_busy 知道何时
 * 播完。链路断了也不怕: PC 停轮询 → 8s 看门狗退出编辑模式 → 播放器每拍查
 * ucGetBtEditMode, 掉线即停（见 c_vChActPlay ①）。 */
void vChassisActionTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        uint8_t ucReq = ucTakeChActPreviewReq();
        const stChActGroupTdf *pst;

        if (ucReq == 0u)
        {
            /* 空闲时把迟到的 STOP 请求也排掉 —— 不然它会在下一次播放的第一拍
             * 被当成中止信号, 预览「一按就停」。 */
            (void)ucTakeChActStopReq();
            vTaskDelay(pdMS_TO_TICKS(20u));
            continue;
        }

        /* 预览取表: 草稿优先（控制台 CPUSH 推上来的当前编辑版）, 没有才回落
         * FLASH 编译表 —— 「在线编辑的立即能预览」。 */
        pst = pstChActPreviewResolve(ucReq);
        if (pst == NULL)
        {
            continue;                 /* 请求作废, 静默（投递端已校验过） */
        }

        /* 互斥: 播放器忙 → 拒绝。⚠️ **不查 ucGetScriptRunning()**: 路线带自动循环
         * /上电即跑的机器它恒为 1（见 c_vChActPlay ① 的让位判据说明）——
         * 「路线会不会突然动」由预览的逐拍让位判据兜（编辑模式被清/让位窗关闭）,
         * 到这里时 bt_cmd_cplay 的 bt_edit_active 门已经放行过一次了。 */
        if (ucGetChActBusy() != 0u)
        {
            vBuzzerBeep(1000u, 0u, 1u);
            vRpiSendDebug("CA fail: busy");
            continue;
        }

        /* 先占坑再进播放器: 关掉 resolve → 进函数之间 CPUSH 改写草稿的窗口
         * （vBtTask 优先级更高, 能在这个缝隙里整批重推; 播放器顶上还会再置一次）。 */
        vSetChActBusy(1u);
        c_vChActPlay(pst, 1u);
    }
}
