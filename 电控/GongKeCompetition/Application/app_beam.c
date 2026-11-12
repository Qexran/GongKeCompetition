/**
  * @file       app_beam.c
  * @brief      Beam 模块 —— 跨模块全局共享状态的唯一定义点。
  *
  *             所有共享状态在此以 static 变量定义，外部通过 app_beam.h 的
  *             getter/setter 访问（不再有跨模块的 extern 裸全局）。
  */

#include "app_beam.h"
#include "app_config.h"        /* BALL_* / HOSTAGE_* / ARM_GROUP_OFS_*（各标定量的编译期默认值） */
#include "servo_device.h"      /* vServoSetSpeed / vServoDeviceSetCurve（arm_speed_dps / arm_curve
                                * 两个 setter 要**当场下发**, 只存 Beam 驱动不读 —— 见 app_beam.h） */
#include "app_rpi.h"           /* ucGetRpiPendTask 等「等链路发任务」遥测（无循环包含:
                                * app_rpi.h → app_arm_action.h → app_script.h → stdint.h）*/
#include <string.h>            /* strcmp（蓝牙参数表按名查找） */
#include "FreeRTOS.h"
#include "task.h"              /* taskENTER_CRITICAL（巡线快照强一致用）/ xTaskGetTickCount */

/* ==================== 姿态角（°）==================== */
static float   s_fAttRoll  = 0.0f;
static float   s_fAttPitch = 0.0f;
static float   s_fAttYaw   = 0.0f;
static float s_fYawRaw    = 0.0f;   /* 原始(未扣校准偏置)工作航向 —— 世界坐标系专用 */

/* ==================== 菜单状态 ==================== */
static int32_t s_iMenuIndex   = 0;
static int32_t s_iAttPage     = 0;
static uint8_t s_ucMenuConfirm = 0;

/* ==================== 姿态角 ==================== */
float fGetAttitudeRoll(void)
{
    return s_fAttRoll;
}

float fGetAttitudePitch(void)
{
    return s_fAttPitch;
}

void vSetYawRaw(float fVal)   { s_fYawRaw = fVal; }
float fGetAttitudeYawRaw(void)
{
    return s_fYawRaw;   /* 原始航向: 世界坐标系专用, 校准偏置不作用于它 */
}

float fGetAttitudeYaw(void)
{
    return s_fAttYaw;
}

void vSetAttitude(float fRoll, float fPitch, float fYaw)
{
    /* ⚠️ 三字**同帧强一致**（Beam 规则第 4 条里"多字需强一致"那一种）:
     * 不加临界区的话, 跨任务读者可能读到「roll 来自第 N 帧、yaw 来自第 N+1 帧」。
     * 今天实际后果有限（控制侧只消费 fGetAttitudeYaw, roll/pitch 只进蓝牙遥测）,
     * 但 app_beam.h 与 app_sensors.c 的注释都**声明**了这里是临界区快照 ——
     * 声明与实现必须一致, 否则后人据此下判断（"两源混发会被快照挡住"）就是错的。
     * 区内只有 3 次 store, 可以忽略。 */
    taskENTER_CRITICAL();
    s_fAttRoll  = fRoll;
    s_fAttPitch = fPitch;
    s_fAttYaw   = fYaw;
    taskEXIT_CRITICAL();
}

/* ==================== 菜单状态 ==================== */
int32_t iGetMenuIndex(void)
{
    return s_iMenuIndex;
}

void vSetMenuIndex(int32_t iVal)
{
    s_iMenuIndex = iVal;
}

int32_t iGetAttPage(void)
{
    return s_iAttPage;
}

void vSetAttPage(int32_t iVal)
{
    s_iAttPage = iVal;
}

uint8_t ucTakeMenuConfirm(void)
{
    uint8_t uc;
    /* ⚠️ 「读-再清零」是读-改-写, **单字读写原子那条前提不覆盖它**:
     * 生产者是按键任务（prio 2）, 消费者是脚本任务（prio 3）/ OLED（prio 1）——
     * 两者之间被抢占时, 刚 OR/置上去的那一位会被后面的 `= 0` 抹掉, 一次按键被静默吞掉。
     * 窗口只有几条指令, 概率很低, 但代价是"按了没反应", 且两条通道（本级 + OLED 掩码）
     * 同形。用一个极短临界区收掉, 成本可忽略。 */
    taskENTER_CRITICAL();
    uc = s_ucMenuConfirm;
    s_ucMenuConfirm = 0;
    taskEXIT_CRITICAL();
    return uc;
}

void vSetMenuConfirm(void)
{
    s_ucMenuConfirm = 1;
}

/* ==================== 拍照结果（2026-09 加）====================
 * 详见 app_beam.h。写者 = vRpiTask（收到 TAG=0x06 的 COORD 帧）; 读者 = OLED 任务。 */
static uint8_t  s_ucPhotoOk       = 0u;   /* 1=成功 0=失败 */
static uint32_t s_ulPhotoDoneTick = 0u;   /* 0 = 本次上电还没收到过结果 */

void     vSetPhotoResult(uint8_t ucOk) { s_ucPhotoOk = (ucOk != 0u) ? 1u : 0u; }
uint8_t  ucGetPhotoResult(void)        { return s_ucPhotoOk; }
void     vSetPhotoDoneTick(uint32_t ulTick) { s_ulPhotoDoneTick = ulTick; }
uint32_t ulGetPhotoDoneTick(void)           { return s_ulPhotoDoneTick; }

/* ==================== 陀螺校准状态（2026-10 加）====================
 * 详见 app_beam.h。写者 = 脚本任务（ucGyroCalActiveSource 收尾时）;
 * 读者 = OLED 任务（标签）+ vGyroSrcWatchdogTick（busy 让路）。 */
static uint8_t  s_ucGyroCalBusy     = 0u;   /* 1=校准进行中 */
static uint8_t  s_ucGyroCalOk       = 0u;   /* 上一次的结果 */
static uint32_t s_ulGyroCalDoneTick = 0u;   /* 0 = 本次上电还没校过 */

void     vSetGyroCalBusy(uint8_t ucVal) { s_ucGyroCalBusy = (ucVal != 0u) ? 1u : 0u; }
uint8_t  ucGetGyroCalBusy(void)         { return s_ucGyroCalBusy; }
void     vSetGyroCalResult(uint8_t ucOk){ s_ucGyroCalOk = (ucOk != 0u) ? 1u : 0u; }
uint8_t  ucGetGyroCalResult(void)       { return s_ucGyroCalOk; }
void     vSetGyroCalDoneTick(uint32_t ulTick) { s_ulGyroCalDoneTick = ulTick; }
uint32_t ulGetGyroCalDoneTick(void)           { return s_ulGyroCalDoneTick; }

/* ==================== 二维码是人工预设的（2026-09 加）====================
 * 详见 app_beam.h 的说明。写者 = OLED 任务（第 2 页编辑）/ 脚本任务（调试积木）;
 * 读者 = ucRpiQrScan（决定要不要扫码）+ OLED 状态行。 */
static uint8_t s_ucQrManual = 0u;

uint8_t ucGetQrManual(void)         { return s_ucQrManual; }
void    vSetQrManual(uint8_t ucVal) { s_ucQrManual = (ucVal != 0u) ? 1u : 0u; }

/* ==================== 机械臂已展开（2026-09 加）====================
 * 详见 app_beam.h 的说明。这里只强调写者：**机械臂动作层**（播完待命序列）与脚本任务。 */
static uint8_t s_ucArmDeployed = 0u;

uint8_t ucGetArmDeployed(void)         { return s_ucArmDeployed; }
void    vSetArmDeployed(uint8_t ucVal) { s_ucArmDeployed = (ucVal != 0u) ? 1u : 0u; }

/* ==================== 激光（继电器）开关状态（2026-10 加）====================
 * 详见 app_beam.h 的说明。写者只有一对: app_actuator.c 的 vRelayOn / vRelayOff
 * （它们同时写引脚 + 写这里, 所以状态与引脚永远一致）; 读者 = OLED 任务。 */
static uint8_t s_ucLaserOn = 0u;

uint8_t ucGetLaserOn(void)         { return s_ucLaserOn; }
void    vSetLaserOn(uint8_t ucVal) { s_ucLaserOn = (ucVal != 0u) ? 1u : 0u; }

/* ==================== 补光灯占空比（2026-10 加）====================
 * 详见 app_beam.h 的说明。写者只有一个: app_actuator.c::vFillLightSet
 * （全工程唯一的 PWM 占空比出口, 它顺手写这里）; 读者 = OLED 任务。 */
static uint16_t s_usFillDuty = 0u;

void     vSetFillDuty(uint16_t usDuty) { s_usFillDuty = usDuty; }
uint16_t usGetFillDuty(void)           { return s_usFillDuty; }

/* ==================== 任务补光（2026-10 加, 编译期宏 → 运行时量）====================
 * 详见 app_beam.h。初值 = app_config.h「任务补光」区同名宏（上电默认, 断电即回）;
 * 写者 = 蓝牙 $B / OLED 名字调参（都经 s_astBeamParam 表）, 读者 = vFillLightTaskBegin。 */
static uint8_t s_ucFillTaskEn       = FILL_TASK_EN;
static uint8_t s_ucFillTaskPct      = FILL_TASK_PCT;
static uint8_t s_ucFillTaskRoadEn   = FILL_TASK_ROAD_EN;
static uint8_t s_ucFillTaskRoadPct  = FILL_TASK_ROAD_PCT;
static uint8_t s_ucFillTaskBallEn   = FILL_TASK_BALL_EN;
static uint8_t s_ucFillTaskBallPct  = FILL_TASK_BALL_PCT;
static uint8_t s_ucFillTaskShootEn  = FILL_TASK_SHOOT_EN;
static uint8_t s_ucFillTaskShootPct = FILL_TASK_SHOOT_PCT;
static uint8_t s_ucFillTaskHsEn     = FILL_TASK_HOSTAGE_EN;
static uint8_t s_ucFillTaskHsPct    = FILL_TASK_HOSTAGE_PCT;
static uint8_t s_ucFillTaskQrEn     = FILL_TASK_QR_EN;
static uint8_t s_ucFillTaskQrPct    = FILL_TASK_QR_PCT;

uint8_t  ucGetFillTaskEn(void)       { return s_ucFillTaskEn; }
void     vSetFillTaskEn(uint8_t ucVal)       { s_ucFillTaskEn = (ucVal != 0u) ? 1u : 0u; }
uint8_t  ucGetFillTaskPct(void)      { return s_ucFillTaskPct; }
void     vSetFillTaskPct(uint8_t ucVal)      { s_ucFillTaskPct = ucVal; }
uint8_t  ucGetFillTaskRoadEn(void)   { return s_ucFillTaskRoadEn; }
void     vSetFillTaskRoadEn(uint8_t ucVal)   { s_ucFillTaskRoadEn = (ucVal != 0u) ? 1u : 0u; }
uint8_t  ucGetFillTaskRoadPct(void)  { return s_ucFillTaskRoadPct; }
void     vSetFillTaskRoadPct(uint8_t ucVal)  { s_ucFillTaskRoadPct = ucVal; }
uint8_t  ucGetFillTaskBallEn(void)   { return s_ucFillTaskBallEn; }
void     vSetFillTaskBallEn(uint8_t ucVal)   { s_ucFillTaskBallEn = (ucVal != 0u) ? 1u : 0u; }
uint8_t  ucGetFillTaskBallPct(void)  { return s_ucFillTaskBallPct; }
void     vSetFillTaskBallPct(uint8_t ucVal)  { s_ucFillTaskBallPct = ucVal; }
uint8_t  ucGetFillTaskShootEn(void)  { return s_ucFillTaskShootEn; }
void     vSetFillTaskShootEn(uint8_t ucVal)  { s_ucFillTaskShootEn = (ucVal != 0u) ? 1u : 0u; }
uint8_t  ucGetFillTaskShootPct(void) { return s_ucFillTaskShootPct; }
void     vSetFillTaskShootPct(uint8_t ucVal) { s_ucFillTaskShootPct = ucVal; }
uint8_t  ucGetFillTaskHsEn(void)     { return s_ucFillTaskHsEn; }
void     vSetFillTaskHsEn(uint8_t ucVal)     { s_ucFillTaskHsEn = (ucVal != 0u) ? 1u : 0u; }
uint8_t  ucGetFillTaskHsPct(void)    { return s_ucFillTaskHsPct; }
void     vSetFillTaskHsPct(uint8_t ucVal)    { s_ucFillTaskHsPct = ucVal; }
uint8_t  ucGetFillTaskQrEn(void)     { return s_ucFillTaskQrEn; }
void     vSetFillTaskQrEn(uint8_t ucVal)     { s_ucFillTaskQrEn = (ucVal != 0u) ? 1u : 0u; }
uint8_t  ucGetFillTaskQrPct(void)    { return s_ucFillTaskQrPct; }
void     vSetFillTaskQrPct(uint8_t ucVal)    { s_ucFillTaskQrPct = ucVal; }

/* ==================== OLED 菜单（2026-09 加）====================
 * 分三样东西, 各有各的写者/读者:
 *   · 按键位掩码  —— 生产者 = 按键任务（vKeyHandler）, 消费者 = OLED 任务
 *   · 菜单占用标志 —— 生产者 = OLED 任务,            消费者 = 按键任务
 *   · 脚本请求通道 —— 生产者 = OLED 菜单,            消费者 = 脚本任务
 * 全部单字读写（Cortex-M4 上 32 位对齐的单次读写原子）, 且**写者/读者各只有一个方向**,
 * 所以不加临界区 —— 符合 CLAUDE.md 的 Beam 并发规则（多字强一致才加锁）。 */

/* ----- 按键位掩码 -----
 * ⚠️ 为什么不复用 vSetMenuConfirm: ucTakeMenuConfirm 是**读后清零**的, 而它的消费者
 *    是脚本任务（vScriptWaitKeyConfirm → STEP_STOP / STEP_DEAD / STEP_ARM_EDIT_WAIT /
 *    测试积木）。菜单再 take 一次 = 把信号偷走 → 脚本永远等不到键。
 *    所以另开这一条通道: 菜单取自己那份, 脚本那份照旧。 */
static uint8_t s_ucOledKeyMask = 0u;

void vOledKeyPost(uint8_t ucMask)
{
    /* 按位 OR 累积: 一次刷屏周期（50ms）内按了不同的键都不会丢;
     * 同一个键按两次合并成一次 —— 那正是消抖想要的行为。
     * ⚠️ `|=` 也是读-改-写: 生产者是按键任务（prio 2）, 消费者是 OLED 任务（prio 1）,
     *    take 那侧同样会 `= 0` —— 两侧一起进临界区才闭合（与 ucTakeMenuConfirm 同源）。 */
    taskENTER_CRITICAL();
    s_ucOledKeyMask |= ucMask;
    taskEXIT_CRITICAL();
}

uint8_t ucTakeOledKeyMask(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucOledKeyMask;
    s_ucOledKeyMask = 0u;   /* 读后自动清零（一次性信号） */
    taskEXIT_CRITICAL();
    return uc;
}

/* ----- 菜单占用标志 -----
 * 为真 = OLED 不在第 1 页（状态页）= 菜单正占着 MAIN。按键任务据此**不再**置
 * ucTakeMenuConfirm()。
 * 理由: 菜单项要用 MAIN 激活, 而 MAIN 同时又是脚本的「继续」键 —— 不掐掉的话,
 * 「在菜单里按 MAIN 选任务」会顺带把停在 STEP_STOP 的路线放走, 车会突然动。
 * 第 1 页（默认页、比赛看的那页）为假 → 路线行为一字不变。 */
static uint8_t s_ucOledMenuBusy = 0u;

uint8_t ucGetOledMenuBusy(void)         { return s_ucOledMenuBusy; }
void    vSetOledMenuBusy(uint8_t ucVal) { s_ucOledMenuBusy = (ucVal != 0u) ? 1u : 0u; }

/* ----- 路线中止位 -----
 * ⚠️ **电平量, 刻意不做成一次性信号**: 它要被**三处**观察 ——
 *   · vScriptRun 的主循环（在步骤边界退出）;
 *   · vLineFollowConverge 的循环（巡线内层, 见 app_line.c —— 不在这里响应的话
 *     「停止巡线测试」要等整步跑完, 而「巡线测试」那一步是无限时长, 等于停不下来;
 *     而且中途退出还要靠它才来得及给 Pi 发 LINE_STOP）;
 *   · ucRpiHandshake 阶段 1 的等待循环（Pi 不上电时那里是**无限等**,
 *     没有出口就没有任何办法取消一次「握手让出」的等待）。
 * 做成 ucTakeXxx 的话先读的那个会把信号偷走, 其余就看不见了。
 * 由 vScriptTask 在**取到请求之后、开跑之前**清零（不清就会立刻把自己掐掉）。 */
static uint8_t s_ucScriptAbortReq = 0u;

void     vScriptAbort(void)               { s_ucScriptAbortReq = 1u; }
uint8_t  ucGetScriptAbortReq(void)        { return s_ucScriptAbortReq; }
void     vSetScriptAbortReq(uint8_t ucVal){ s_ucScriptAbortReq = (ucVal != 0u) ? 1u : 0u; }

/* ----- 脚本任务请求通道 -----
 * 菜单要「让固件做点事」（跑任务预设 / 握手让出 / 无限巡线测试）, 但**不能自己去做**:
 *   · ucRpiHandshake() 会 while(!ucTakeHandshakeSeen()) **无限阻塞**, 而 OLED 任务是
 *     最低优先级(1) —— 阻塞它 = 整屏停摆, 连操作员唯一的反馈通道(状态页)都没了;
 *     且 app_rpi.c 的 s_bHsDone 写明「只有脚本任务读写」, 多一个调用者会让正常重握
 *     被跳过（症状: 该握手时直接放行）。
 *   · 跑路线/巡线会动底盘, 只能在脚本任务上下文跑。
 * 所以一律「投递请求 → 脚本任务消费」。
 * ⚠️ 打包成一个 uint16_t 的目的是「取回时 req 与 arg 一定来自同一次投递」——
 *    分两个字节写的话, 取方可能读到新 req 配旧 arg。
 * ⚠️ **不排队**: 槽位里还有没被取走的请求时, 新请求被丢弃并返回 0（后写者胜, 与后台
 *    臂序列同一条纪律 —— 确定且看得见）。调用方必须把 0 当失败处理（留痕, 别静默吞掉）。
 * ⚠️ 这样也就**不需要临界区**: 写者见非 0 就跳过, 所以「取方读到 R 再清零」不可能
 *    吃掉一次刚写进去的请求。 */
static uint16_t s_usScriptReq = 0u;   /* (req << 8) | arg; 0 = 无请求 */

uint8_t vScriptPostReq(uint8_t ucReq, uint8_t ucArg)
{
    if ((ucReq == SCRIPT_REQ_NONE) || (s_usScriptReq != 0u)) {
        return 0u;
    }
    /* 每个请求都先中止当前正在跑的东西 —— 「后写者胜 + 新指令显式终止旧的」,
     * 与后台臂序列同一条纪律。于是菜单里「点执行任务」不用先手动停路线,
     * 行为也确定: 路线在**步骤边界**收尾, 不是从半路被掐掉。
     * 顺带把「握手让出卡在那里等 Pi」也解掉 —— 见 ucRpiHandshake 阶段 1 里的中止检查。
     * ⚠️ 中止位是电平量, 由 vScriptTask 在**取到这个请求之后、开跑之前**清零。 */
    s_ucScriptAbortReq = 1u;
    /* 同时取消 Pi 侧正在跑的任务/巡线流/靶标流（"x", 2026-10 加）: 不取消的话被中止
     * 的旧任务会在 Pi 上跑完并回帧, 迟到的结果帧会落进**下一轮**同类型步骤的等待
     * 窗口被当成新结果（「OLED 反复跑同一任务拿到旧数据」的错位来源, 全程无报错）。
     * 只发不等、链路不通就白丢 —— 那种时候 Pi 上本来也没有活着的任务。 */
    vRpiCancelPiTask();
    s_usScriptReq = ((uint16_t)ucReq << 8) | (uint16_t)ucArg;
    return 1u;
}

uint8_t ucTakeScriptReq(uint8_t *pucArg)
{
    uint16_t us;

    /* 读-清进临界区（同 ucTakeMenuConfirm）: 写者 vBtTask(4) 比读者 vScriptTask(3)高,
     * 抢在「读 → 清」之间写的请求会被 `= 0` 吞掉 —— $B,HS 回了 OK 却永不生效。 */
    taskENTER_CRITICAL();
    us = s_usScriptReq;
    s_usScriptReq = 0u;   /* 读后自动清零（一次性信号） */
    taskEXIT_CRITICAL();

    if (us == 0u) {
        return SCRIPT_REQ_NONE;
    }

    if (pucArg != NULL) {
        *pucArg = (uint8_t)(us & 0xFFu);
    }
    return (uint8_t)(us >> 8);
}

/* ----- 上次菜单「握手让出」跑完的时刻（xTaskGetTickCount）-----
 * 0 = 本次上电还没跑过。用**时刻**而不是一次性信号: OLED 要按自己的 50ms 节拍判断
 * 「还在 1 秒内」, 那是「看时间」不是「接事件」—— 信号会被别的读者偷走, 时刻不会。 */
static uint32_t s_ulHandshakeDoneTick = 0u;

void     vSetHandshakeDoneTick(uint32_t ulTick) { s_ulHandshakeDoneTick = ulTick; }
uint32_t ulGetHandshakeDoneTick(void)           { return s_ulHandshakeDoneTick; }

/* ==================== 世界航向 / 运动坐标 ==================== */
static float   s_fWorldHeading   = 0.0f;
static float   s_fDirectX        = 0.0f;
static float   s_fDirectY        = 0.0f;
static float   s_fOriginX        = 0.0f;
static float   s_fOriginY        = 0.0f;
static const float s_fDistWukuai = 150.0f;   /* 物块间距常量 */
static float   s_fVisualServoPxToMmX = 0.30f;
static float   s_fVisualServoPxToMmY = 0.30f;
static int32_t s_iChassisAccel   = 230;
static uint8_t s_ucSehuanStreaming = 0;
static int8_t  s_cSehuanYDiff    = 0;
static uint8_t s_aucLastRpiPacket[9] = {0};
static uint8_t s_ucYawFresh      = 0;

/* ----- 世界航向 ----- */
float fGetWorldHeading(void)
{
    return s_fWorldHeading;
}

void vSetWorldHeading(float fVal)
{
    s_fWorldHeading = fVal;
}

/* ----- 坐标偏移 ----- */
float fGetDirectX(void)
{
    return s_fDirectX;
}

void vSetDirectX(float fVal)
{
    s_fDirectX = fVal;
}

float fGetDirectY(void)
{
    return s_fDirectY;
}

void vSetDirectY(float fVal)
{
    s_fDirectY = fVal;
}

/* ----- 像素原点 ----- */
float fGetOriginX(void)
{
    return s_fOriginX;
}

void vSetOriginX(float fVal)
{
    s_fOriginX = fVal;
}

float fGetOriginY(void)
{
    return s_fOriginY;
}

void vSetOriginY(float fVal)
{
    s_fOriginY = fVal;
}

/* ----- 物块间距(常量) ----- */
float fGetDistanceWukuai(void)
{
    return s_fDistWukuai;
}

/* ----- 大分辨率像素→毫米增益 ----- */
float fGetVisualServoPxToMmX(void)
{
    return s_fVisualServoPxToMmX;
}

void vSetVisualServoPxToMmX(float fVal)
{
    s_fVisualServoPxToMmX = fVal;
}

float fGetVisualServoPxToMmY(void)
{
    return s_fVisualServoPxToMmY;
}

void vSetVisualServoPxToMmY(float fVal)
{
    s_fVisualServoPxToMmY = fVal;
}

/* ----- 底盘加速度 ----- */
int32_t iGetChassisAccel(void)
{
    return s_iChassisAccel;
}

void vSetChassisAccel(int32_t iVal)
{
    /* 消费端是 (uint8_t)（EMM 加速度档 0~255, emm_v5_device.c 注释明确范围）:
     * 不钳的话蓝牙把值写到 256+ 会被静默截断（300→44、2000→208）, 界面数值与
     * 电机实际档位脱钩。注册表的 fMax 也同步收到 255（2026-10-09 修）。 */
    if (iVal < 0)       { iVal = 0; }
    if (iVal > 255)     { iVal = 255; }
    s_iChassisAccel = iVal;
}

/* ----- 色环流式细定标 ----- */
uint8_t ucGetSehuanStreaming(void)
{
    return s_ucSehuanStreaming;
}

void vSetSehuanStreaming(uint8_t ucVal)
{
    s_ucSehuanStreaming = ucVal;
}

/* ----- 底盘动作组（时间轴速度混合播放器, app_chassis_action.c; 2026-10 加） -----
 * 增益/限幅四个量 = 蓝牙「底盘动作组」组的可写运行时量（上电默认 = app_config.h
 * 的 CH_ACT_* 同名宏, 断电回宏）—— 播放器每拍现取, 改完下一拍生效。
 *
 * busy/playing: **唯一写者是播放器**（开播写 1 / 收尾清 0）, 注册进参数表是只读
 * 遥测（PC 端 $B,CPLAY 后轮询 ch_busy 判断播完）—— 与 arm_seq_busy 同一口径。
 *
 * preview/stop 两个是「读后清零」的一次性请求槽: 写方 = vBtTask（$B,CPLAY/CSTOP）,
 * 读方 = vChassisActionTask / 播放器逐拍。单字读写原子, 按 Beam 规则第 4 条不加锁。 */
static float   s_fChActTimeScale = CH_ACT_TIME_SCALE;
static float   s_fChActHeadKp    = CH_ACT_HEAD_KP;
static float   s_fChActHeadKi    = CH_ACT_HEAD_KI;
static float   s_fChActHeadKd    = CH_ACT_HEAD_KD;
static float   s_fChActHeadKiMax = CH_ACT_HEAD_KI_MAX;
static float   s_fChActHeadDb    = CH_ACT_HEAD_DB_DEG;
static float   s_fChActWMax      = CH_ACT_W_MAX;
static float   s_fChActWheelMax  = CH_ACT_WHEEL_MAX;
static float   s_fChPivotKp      = CH_ACT_PIVOT_KP;
static float   s_fChPivotKi      = CH_ACT_PIVOT_KI;
static float   s_fChPivotKd      = CH_ACT_PIVOT_KD;
static float   s_fChPivotKiMax   = CH_ACT_PIVOT_KI_MAX;
static float   s_fChPivotWMax    = CH_ACT_PIVOT_W_MAX;
static uint8_t s_ucChActBusy     = 0u;
static uint8_t s_ucChActPlaying  = 0u;
static uint8_t s_ucChActPreviewReq = 0u;
static uint8_t s_ucChActStopReq    = 0u;

float fGetChActTimeScale(void)              { return s_fChActTimeScale; }
void  vSetChActTimeScale(float fVal)        { s_fChActTimeScale = fVal; }
float fGetChActHeadKp(void)                 { return s_fChActHeadKp; }
void  vSetChActHeadKp(float fVal)           { s_fChActHeadKp = fVal; }
float fGetChActHeadKi(void)                 { return s_fChActHeadKi; }
void  vSetChActHeadKi(float fVal)           { s_fChActHeadKi = fVal; }
float fGetChActHeadKd(void)                 { return s_fChActHeadKd; }
void  vSetChActHeadKd(float fVal)           { s_fChActHeadKd = fVal; }
float fGetChActHeadKiMax(void)              { return s_fChActHeadKiMax; }
void  vSetChActHeadKiMax(float fVal)        { s_fChActHeadKiMax = fVal; }
float fGetChActHeadDb(void)                 { return s_fChActHeadDb; }
void  vSetChActHeadDb(float fVal)           { s_fChActHeadDb = fVal; }
float fGetChActWMax(void)                   { return s_fChActWMax; }
void  vSetChActWMax(float fVal)             { s_fChActWMax = fVal; }
float fGetChActWheelMax(void)               { return s_fChActWheelMax; }
void  vSetChActWheelMax(float fVal)         { s_fChActWheelMax = fVal; }

/* 支点转弯段（CH_ACT_KIND_PIVOT）的 PID 出厂值引用 PIVOT_TURN_*（见 app_config.h） */
float fGetChPivotKp(void)                   { return s_fChPivotKp; }
void  vSetChPivotKp(float fVal)             { s_fChPivotKp = fVal; }
float fGetChPivotKi(void)                   { return s_fChPivotKi; }
void  vSetChPivotKi(float fVal)             { s_fChPivotKi = fVal; }
float fGetChPivotKd(void)                   { return s_fChPivotKd; }
void  vSetChPivotKd(float fVal)             { s_fChPivotKd = fVal; }
float fGetChPivotKiMax(void)                { return s_fChPivotKiMax; }
void  vSetChPivotKiMax(float fVal)          { s_fChPivotKiMax = fVal; }
float fGetChPivotWMax(void)                 { return s_fChPivotWMax; }
void  vSetChPivotWMax(float fVal)           { s_fChPivotWMax = fVal; }

uint8_t ucGetChActBusy(void)                { return s_ucChActBusy; }
void    vSetChActBusy(uint8_t ucVal)        { s_ucChActBusy = ucVal; }
uint8_t ucGetChActPlayingId(void)           { return s_ucChActPlaying; }
void    vSetChActPlayingId(uint8_t ucVal)   { s_ucChActPlaying = ucVal; }

uint8_t ucTakeChActPreviewReq(void)
{
    uint8_t ucVal;

    /* 读-清进临界区（同 ucTakeMenuConfirm）: 写者 vBtTask(4) 比读者播放任务(3)高,
     * 抢在「读 → 清」之间置 1 会被下面的 `= 0` 抹掉 —— CPLAY 回了 OK 却永不播出。 */
    taskENTER_CRITICAL();
    ucVal = s_ucChActPreviewReq;
    s_ucChActPreviewReq = 0u;
    taskEXIT_CRITICAL();
    return ucVal;
}

void vSetChActPreviewReq(uint8_t ucVal)     { s_ucChActPreviewReq = ucVal; }

uint8_t ucTakeChActStopReq(void)
{
    uint8_t ucVal;

    /* 读-清进临界区（同 ucTakeMenuConfirm）: 抢在「读 → 清」之间的 CSTOP 会被
     * `= 0` 吞掉 —— 停车请求静默丢失, 破坏「刹车指令永远合法且必达」的承诺。 */
    taskENTER_CRITICAL();
    ucVal = s_ucChActStopReq;
    s_ucChActStopReq = 0u;
    taskEXIT_CRITICAL();
    return ucVal;
}

void vSetChActStopReq(uint8_t ucVal)        { s_ucChActStopReq = ucVal; }

int8_t cGetSehuanYDiff(void)
{
    return s_cSehuanYDiff;
}

void vSetSehuanYDiff(int8_t cVal)
{
    s_cSehuanYDiff = cVal;
}

/* ----- 最近一帧 RPI 包 -----
 * 写者 = vRpiTask（每收到一条 ≥9B 的 COORD 帧就整包覆盖）, 读者 = 脚本任务
 * （ucRpiWaitQrResult 逐字节读 [0..4]）。9 字节不是单字, 而两条任务优先级不同
 * （vRpiTask 4 > 脚本 3）→ 写者能在读者读到一半时插进来, 读出「半新半旧」的一包。
 * QR 那条最要命: 前缀 "QR" + 3 位数字是**逐字节判**的, 撕裂后仍可能凑出一个
 * **看着合法的错二维码**, 后面排爆选色/标靶选靶全按它走且不报错。
 * 故: 写入进临界区, 并新增拷贝式读入口 vGetLastRpiPacket（推荐消费者用它）。 */
uint8_t *pucGetLastRpiPacket(void)
{
    /* ⚠️ 返回的是**内部缓冲**（为兼容旧的坐标变换调用点保留）。除非你确定没有第二个
     * 写者会插进来, 否则请改用 vGetLastRpiPacket(dst) 一次拷走。 */
    return s_aucLastRpiPacket;
}

void vGetLastRpiPacket(uint8_t *pucDst)
{
    uint8_t i;
    taskENTER_CRITICAL();
    for (i = 0; i < 9; i++) pucDst[i] = s_aucLastRpiPacket[i];
    taskEXIT_CRITICAL();
}

void vSetLastRpiPacket(const uint8_t *pucSrc)
{
    uint8_t i;
    taskENTER_CRITICAL();
    for (i = 0; i < 9; i++) s_aucLastRpiPacket[i] = pucSrc[i];
    taskEXIT_CRITICAL();
}

/* ----- 航向刷新标志(一次性信号) + 姿态帧计数 -----
 * 帧计数与一次性信号**同源**（都在传感器任务真收到一帧时推进, 见 app_sensors.c）,
 * 但语义不同: 那一个是"读后清零"（PID 的三处在用, 谁读走谁得）, 这一个**只增不减、
 * 谁都能读** —— 差值 = IMU 新鲜度（同 ulGetLineFrameCount 的做法, 见 app_beam.h）。
 * 单写者 = 传感器任务, 32 位对齐读写天然原子, 按 Beam 规则第 4 条不加锁。 */
static uint32_t s_ulYawFrameCnt = 0u;

uint8_t ucTakeYawFresh(void)
{
    uint8_t uc = s_ucYawFresh;
    s_ucYawFresh = 0;   /* 读后自动清零 */
    return uc;
}

void vSetYawFresh(void)
{
    s_ucYawFresh = 1;
    s_ulYawFrameCnt++;
}

uint32_t ulGetYawFrameCount(void)
{
    return s_ulYawFrameCnt;
}

/* ----- 陀螺仪选源（两口 × 三设备, 2026-10 扩展）-----
 * `gyro_src`     = **首选**：操作员设的（蓝牙可写, 初值 = app_config.h 的 SENSOR_GYRO_SRC）。
 * `gyro_src_eff` = **有效源**：实际在发的那一路, 由 app_sensors.c::vGyroSrcWatchdogTick 维护
 *                  （热备自动切换；关掉 gyro_auto_switch 时恒等于首选）。
 * 两个传感器任务都在跑、都在解析, 但只有**有效源**那一支调 vSetAttitude/vSetYawFresh ——
 * 理由见 app_beam.h。⚠️ 别把这两个合成一个：合成就没有"切回首选"可言了。
 * m0_yaw / 各口帧计数 / 切换次数是**只读遥测**, 与选源无关（选不选中都更新 —— 备份那颗
 * 的存活必须一直看得见, 那正是热备的判据来源）。 */
static uint8_t  s_ucGyroSrc     = SENSOR_GYRO_SRC;
static uint8_t  s_ucGyroSrcEff  = SENSOR_GYRO_SRC;
static uint8_t  s_ucGyroAutoSw  = GYRO_AUTO_SWITCH;
static int32_t  s_iGyroFrmMain  = 0;
static int32_t  s_iGyroFrmSub   = 0;
static int32_t  s_iGyroAutoSwN  = 0;
static float    s_fM0Yaw        = 0.0f;
static uint32_t s_ulM0FrameCnt  = 0u;

uint8_t  ucGetGyroSrc(void) { return s_ucGyroSrc; }
void     vSetGyroSrc(uint8_t ucVal) { s_ucGyroSrc = ucVal; }

uint8_t  ucGetGyroSrcEff(void) { return s_ucGyroSrcEff; }
void     vSetGyroSrcEff(uint8_t ucVal) { s_ucGyroSrcEff = ucVal; }

uint8_t  ucGetGyroAutoSwitch(void) { return s_ucGyroAutoSw; }
void     vSetGyroAutoSwitch(uint8_t ucVal) { s_ucGyroAutoSw = (ucVal != 0u) ? 1u : 0u; }

int32_t  iGetGyroFramesMain(void) { return s_iGyroFrmMain; }
void     vBumpGyroFramesMain(void) { s_iGyroFrmMain++; }
int32_t  iGetGyroFramesSub(void) { return s_iGyroFrmSub; }
void     vBumpGyroFramesSub(void) { s_iGyroFrmSub++; }

int32_t  iGetGyroAutoSw(void) { return s_iGyroAutoSwN; }
void     vBumpGyroAutoSw(void) { s_iGyroAutoSwN++; }

float    fGetM0Yaw(void) { return s_fM0Yaw; }
void     vSetM0Yaw(float fVal)
{
    s_fM0Yaw = fVal;
    s_ulM0FrameCnt++;
}
uint32_t ulGetM0YawFrameCount(void) { return s_ulM0FrameCnt; }

/* ----- OLED 菜单换行动画开关（2026-10 加）-----
 * 初值 = app_config.h 的 OLED_MENU_ANIM_EN（「调参」Tab 改的是那个, 要重编译）。
 * ⚠️ 读方 app_oled.c **每拍现取** —— 所以这里是"当场生效"的量, 不是 BALL_* 那种标定量。 */
static uint8_t s_ucOledMenuAnim = OLED_MENU_ANIM_EN;

uint8_t  ucGetOledMenuAnim(void) { return s_ucOledMenuAnim; }
void     vSetOledMenuAnim(uint8_t ucVal) { s_ucOledMenuAnim = (ucVal != 0u) ? 1u : 0u; }

/* ----- RPI 帧到达 / 握手可见（一次性信号, 生产者=vRpiTask） ----- */
static uint8_t s_ucRpiFrameReady = 0;
static uint8_t s_ucHandshakeSeen = 0;

uint8_t ucTakeRpiFrameReady(void)
{
    uint8_t uc;
    /* ⚠️ 读-清进临界区（与 ucTakeMenuConfirm 同一模式, 2026-10-09 补）: 生产者
     * vRpiTask（prio 4）高于消费者脚本任务（prio 3）, 抢进 load 与 store 之间会把
     * 刚置上的信号抹掉 → 该步骤空等到超时。本文件 8 个 take 型读-清同修。 */
    taskENTER_CRITICAL();
    uc = s_ucRpiFrameReady;
    s_ucRpiFrameReady = 0;   /* 读后自动清零 */
    taskEXIT_CRITICAL();
    return uc;
}

void vSetRpiFrameReady(void)
{
    s_ucRpiFrameReady = 1;
}

uint8_t ucTakeHandshakeSeen(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucHandshakeSeen;
    s_ucHandshakeSeen = 0;   /* 读后自动清零 */
    taskEXIT_CRITICAL();
    return uc;
}

void vSetHandshakeSeen(void)
{
    s_ucHandshakeSeen = 1;
}

/* ----- 巡线数据快照（同帧多字强一致 → 临界区） ----- */
static float    s_fLineErr       = 0.0f;
static float    s_fLineSteer     = 0.0f;
static float    s_fLineAngleDeg  = 0.0f;
static uint8_t  s_ucLineValid    = 0;
static uint32_t s_ulLineFrameCnt = 0;

void vSetLineData(float fErr, float fSteer, float fAngleDeg, uint8_t ucValid)
{
    taskENTER_CRITICAL();
    s_fLineErr      = fErr;
    s_fLineSteer    = fSteer;
    s_fLineAngleDeg = fAngleDeg;
    s_ucLineValid   = ucValid;
    s_ulLineFrameCnt++;
    taskEXIT_CRITICAL();
}

void vGetLineData(float *pfErr, float *pfSteer, float *pfAngleDeg, uint8_t *pucValid)
{
    taskENTER_CRITICAL();
    *pfErr      = s_fLineErr;
    *pfSteer    = s_fLineSteer;
    *pfAngleDeg = s_fLineAngleDeg;
    *pucValid   = s_ucLineValid;
    taskEXIT_CRITICAL();
}

uint32_t ulGetLineFrameCount(void)
{
    return s_ulLineFrameCnt;
}

/* ----- 巡线航向环的目标角（运行时积木/蓝牙可调, 断电回编译期默认） -----
 * 从 app_config.h 的 LINE_ANGLE_TARGET_DEG 取上电初值 —— 与 BALL_* / HOSTAGE_* 同一套
 * 两层关系: 「调参」Tab 改的是那条宏 = 持久默认, 「蓝牙」Tab 改的是这里的 RAM = 当场生效。
 * 单字、写者都是任务上下文（脚本任务 / vBtTask）, 按 Beam 规则第 4 条不加锁。 */
static float s_fLineAngleTargetDeg = LINE_ANGLE_TARGET_DEG;

void  vSetLineAngleTarget(float fVal)   { s_fLineAngleTargetDeg = fVal; }
float fGetLineAngleTarget(void)         { return s_fLineAngleTargetDeg; }

/* ----- 巡线航向环的**陀螺那条的目标角**（2026-09 定案; 权重那套已随融合删除） -----
 * 同一套两层关系（宏 = 持久默认, 这里的 RAM = 当场生效）; 单字、写者都是任务上下文, 不加锁。
 * 写者有两处: 「巡线收敛」积木进步骤时写一次(当起点)、蓝牙 $B,SET 覆盖。
 * 消费者 app_line.c 每拍现取 → 蓝牙边跑边调下一拍就生效。
 * ⚠️ 它**只给目标**:"这一步要不要听陀螺"是积木的 `src` 参数决定的, 与这里无关。 */
static float   s_fLineGyroTargetDeg = LINE_GYRO_TARGET_DEG;

void   vSetLineGyroTarget(float fVal)  { s_fLineGyroTargetDeg = fVal; }
float  fGetLineGyroTarget(void)        { return s_fLineGyroTargetDeg; }

/* ----- 巡线航向环**融合档的相机占比**[%]（2026-10-08 加, 同一套两层关系） -----
 * 只在全局 `LINE_CONV_SRC=融合` 时被用到: 两条各自走完自己的环后, 输出按 本值:(100−本值) 加权
 * （**输出端**加权, 见 app_config.h「删除记录」段 —— 误差端加权/换算尺仍是禁忌）。
 * 上电默认 = LINE_CAM_PCT; 2026-10-30 起六个脚本调用点一律传 KEEP（不再逐块写）,
 * 写者只剩蓝牙 $B,SET（ucBeamApply 按表钳位）; 消费者 app_line.c 每拍现取。 */
static float   s_fLineCamPct = LINE_CAM_PCT;

void   vSetLineCamPct(float fVal)      { s_fLineCamPct = fVal; }
float  fGetLineCamPct(void)            { return s_fLineCamPct; }

/* ----- 巡线直走的 mm→时间换算系数（2026-10 加, 同一套两层关系） -----
 * 估时(ms) × 本值 = 「巡线直走」实际的闭环时长。估时公式是位置模式的梯形曲线,
 * 搬到速度模式斜坡上带系统偏差（实测偏长）, 现场标定: 走过了调小、没走到调大。
 * 消费者 ucRpiLineStraightWalk 每步现取 → 蓝牙改完下一步就生效。⚠️ 下界 0.01:
 * 上下界经 bt_fmt_f2 报名只有两位小数, 填 0.001 报出去 = "0.00"（界面成"下界 0"）。 */
static float   s_fLineTimeScale = LINE_TIME_SCALE;

void   vSetLineTimeScale(float fVal)   { s_fLineTimeScale = fVal; }
float  fGetLineTimeScale(void)         { return s_fLineTimeScale; }

/* ----- 巡线控制律标定量（2026-10-06 加, 同一套两层关系）-----
 * 增益/死区/限幅/方向原先是纯编译期宏（LINE_STEER_TO_VX / LINE_CAM_TO_W / …）,
 * 现场调一次要重编译烧写一轮 —— 与 BALL_* / ROAD_* 那批「蓝牙当场改、零重编译」的
 * 标定量不一致。「调参」Tab 改的宏 = 上电默认（持久）, 这里的 RAM = 当场生效（易失）。
 * 写方 = vBtTask（$B,SET 经 ucBeamApply 钳位）; 读方 = app_line.c::vLineFollowConverge
 * **每拍现取**（不缓存）→ 蓝牙边跑边改, 下一拍（20ms）就进控制律。
 * 单字、写者任务上下文, 按 Beam 规则第 4 条不加锁。
 * ⚠️ 刻意**不含**保护类窗口量（LINE_STALE_TIMEOUT_MS / LINE_GYRO_STALE_MS /
 *    LINE_FIRST_FRAME_TIMEOUT_MS）与判据类量（LINE_CONVERGE_* / LINE_CE_*）:
 *    前者是安全阀（运行时一键关掉 = 掉线后按最后一帧 steer 跑飞 / 陀螺停摆常转）,
 *    后者与死区有「阈值 ≥ 死区」的跨参约束 —— 都留在编译期, 不给无线一键关掉的通路。 */
static float s_fLineSteerToVx     = LINE_STEER_TO_VX;      /* 对准组: steer→vx */
static float s_fLineWalkSteerToVx = LINE_WALK_STEER_TO_VX; /* 边走组: steer→vx */
static float s_fLineVxSign        = LINE_VX_SIGN;
static float s_fLineVxMax         = LINE_VX_MAX;
static float s_fLineCamToW        = LINE_CAM_TO_W;         /* 对准组: 图像角→w */
static float s_fLineWalkCamToW    = LINE_WALK_CAM_TO_W;    /* 边走组: 图像角→w */
static float s_fLineCamDeadband   = LINE_CAM_DEADBAND_DEG;
static float s_fLineGyroToW       = LINE_GYRO_TO_W;        /* 对准组: yaw→w */
static float s_fLineWalkGyroToW   = LINE_WALK_GYRO_TO_W;   /* 边走组: yaw→w */
static float s_fLineGyroDeadband  = LINE_GYRO_DEADBAND_DEG;
static float s_fLineWMax          = LINE_W_MAX;            /* 两组、两源共用 */

void   vSetLineSteerToVx(float fVal)     { s_fLineSteerToVx = fVal; }
float  fGetLineSteerToVx(void)           { return s_fLineSteerToVx; }
void   vSetLineWalkSteerToVx(float fVal) { s_fLineWalkSteerToVx = fVal; }
float  fGetLineWalkSteerToVx(void)       { return s_fLineWalkSteerToVx; }
void   vSetLineVxSign(float fVal)        { s_fLineVxSign = fVal; }
float  fGetLineVxSign(void)              { return s_fLineVxSign; }
void   vSetLineVxMax(float fVal)         { s_fLineVxMax = fVal; }
float  fGetLineVxMax(void)               { return s_fLineVxMax; }
void   vSetLineCamToW(float fVal)        { s_fLineCamToW = fVal; }
float  fGetLineCamToW(void)              { return s_fLineCamToW; }
void   vSetLineWalkCamToW(float fVal)    { s_fLineWalkCamToW = fVal; }
float  fGetLineWalkCamToW(void)          { return s_fLineWalkCamToW; }
void   vSetLineCamDeadband(float fVal)   { s_fLineCamDeadband = fVal; }
float  fGetLineCamDeadband(void)         { return s_fLineCamDeadband; }
void   vSetLineGyroToW(float fVal)       { s_fLineGyroToW = fVal; }
float  fGetLineGyroToW(void)             { return s_fLineGyroToW; }
void   vSetLineWalkGyroToW(float fVal)   { s_fLineWalkGyroToW = fVal; }
float  fGetLineWalkGyroToW(void)         { return s_fLineWalkGyroToW; }
void   vSetLineGyroDeadband(float fVal)  { s_fLineGyroDeadband = fVal; }
float  fGetLineGyroDeadband(void)        { return s_fLineGyroDeadband; }
void   vSetLineWMax(float fVal)          { s_fLineWMax = fVal; }
float  fGetLineWMax(void)                { return s_fLineWMax; }

/* ----- 底盘速度指令快照（三字强一致 → 临界区） ----- */
static int32_t s_aiChassisSpd[3] = { 0, 0, 0 };

void vSetChassisSpdCmd(int32_t iVx, int32_t iVy, int32_t iW)
{
    taskENTER_CRITICAL();
    s_aiChassisSpd[0] = iVx;
    s_aiChassisSpd[1] = iVy;
    s_aiChassisSpd[2] = iW;
    taskEXIT_CRITICAL();
}

void vGetChassisSpdCmd(int32_t *piVx, int32_t *piVy, int32_t *piW)
{
    taskENTER_CRITICAL();
    if (piVx != NULL) { *piVx = s_aiChassisSpd[0]; }
    if (piVy != NULL) { *piVy = s_aiChassisSpd[1]; }
    if (piW  != NULL) { *piW  = s_aiChassisSpd[2]; }
    taskEXIT_CRITICAL();
}

/* ----- 状态页秒表（运行段起点 tick + 已累计毫秒 + 运行位, 强一致 → 临界区） -----
 * ⚠️ 停止时把当前段**折进累计值**再清 run 位: 只清 run 位的话再按「开始」
 *    会把停止期间也算进去。开始/结束/重置都可能来自脚本任务或 OLED 任务,
 *    读取侧（OLED 每 50ms）也一样, 所以读写都进临界区。 */
static uint8_t    s_ucTimerRun       = 0u;
static uint32_t   s_ulTimerAccumMs   = 0u;
static TickType_t s_xTimerStartTick  = 0;
static uint8_t    s_ucTimerEver      = 0u;   /* 本次上电是否开始过（健康检查显示让位判据） */

void vTimerCmd(emTimerCmdTdf emCmd)
{
    taskENTER_CRITICAL();
    switch (emCmd)
    {
    case TIMER_CMD_START:
        s_ucTimerEver = 1u;                  /* 一旦开始过, 健康检查结果就永久让位给秒表 */
        if (s_ucTimerRun == 0u)
        {
            s_xTimerStartTick = xTaskGetTickCount();
            s_ucTimerRun = 1u;
        }
        break;

    case TIMER_CMD_STOP:
        if (s_ucTimerRun != 0u)
        {
            /* 存 tick 差值再乘周期, 不存绝对 tick*周期: 后者在累计运行几十天后
             * 会先于 tick 计数本身溢出（当前 1kHz 下 tick 差值的乘法很安全）。 */
            s_ulTimerAccumMs += (uint32_t)(xTaskGetTickCount() - s_xTimerStartTick)
                                * (uint32_t)portTICK_PERIOD_MS;
            s_ucTimerRun = 0u;
        }
        break;

    case TIMER_CMD_RESET:
        s_ulTimerAccumMs = 0u;
        s_xTimerStartTick = 0;
        s_ucTimerRun = 0u;
        break;

    default:
        break;
    }
    taskEXIT_CRITICAL();
}

uint32_t ulTimerElapsedMs(void)
{
    uint32_t ulMs;

    taskENTER_CRITICAL();
    ulMs = s_ulTimerAccumMs;
    if (s_ucTimerRun != 0u)
    {
        ulMs += (uint32_t)(xTaskGetTickCount() - s_xTimerStartTick)
                * (uint32_t)portTICK_PERIOD_MS;
    }
    taskEXIT_CRITICAL();
    return ulMs;
}

uint8_t ucTimerEverStarted(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucTimerEver;
    taskEXIT_CRITICAL();
    return uc;
}

/* ----- 相机健康检查结果（TAG=0x0D, 四路相机, 2026-11 加）-----
 * 同帧多字强一致 → 临界区快照（同矩形/看路那几组）。
 * 写方 = ucRpiHealthCheck（开始/超时/链路失败）与 vRpiTask（结果帧）。
 * 写序: 结果帧 → vSetHealthResult → vSetHealthBusy(0) → vSetHealthReady。
 * 读方 = OLED 状态页（每拍）与 IF 条件 getter（任意时刻, 非 take 语义）。 */
static uint8_t s_ucHealthM      = RPI_HEALTH_UNKNOWN;
static uint8_t s_ucHealthQ      = RPI_HEALTH_UNKNOWN;
static uint8_t s_ucHealthPi     = RPI_HEALTH_UNKNOWN;
static uint8_t s_ucHealthMv     = RPI_HEALTH_UNKNOWN;
static uint8_t s_ucHealthBusy   = 0u;
static uint8_t s_ucHealthReady  = 0u;

void vSetHealthBusy(uint8_t ucBusy)
{
    taskENTER_CRITICAL();
    s_ucHealthBusy = (ucBusy != 0u) ? 1u : 0u;
    taskEXIT_CRITICAL();
}

uint8_t ucGetHealthBusy(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucHealthBusy;
    taskEXIT_CRITICAL();
    return uc;
}

void vSetHealthResult(uint8_t ucM, uint8_t ucQ, uint8_t ucPi, uint8_t ucMv)
{
    taskENTER_CRITICAL();
    s_ucHealthM  = ucM;
    s_ucHealthQ  = ucQ;
    s_ucHealthPi = ucPi;
    s_ucHealthMv = ucMv;
    taskEXIT_CRITICAL();
}

void vGetHealthResult(uint8_t *pucM, uint8_t *pucQ, uint8_t *pucPi, uint8_t *pucMv)
{
    taskENTER_CRITICAL();
    if (pucM  != NULL) { *pucM  = s_ucHealthM; }
    if (pucQ  != NULL) { *pucQ  = s_ucHealthQ; }
    if (pucPi != NULL) { *pucPi = s_ucHealthPi; }
    if (pucMv != NULL) { *pucMv = s_ucHealthMv; }
    taskEXIT_CRITICAL();
}

uint8_t ucTakeHealthReady(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucHealthReady;
    s_ucHealthReady = 0u;
    taskEXIT_CRITICAL();
    return uc;
}

void vSetHealthReady(void)
{
    taskENTER_CRITICAL();
    s_ucHealthReady = 1u;
    taskEXIT_CRITICAL();
}

/* ----- 二维码结果 ----- */
static int32_t s_iQrCode = -1;

int32_t iGetQrCode(void)
{
    return s_iQrCode;
}

void vSetQrCode(int32_t iVal)
{
    s_iQrCode = iVal;
}

/* ----- 人质识别结果（同帧多字强一致 → 临界区, 同巡线快照） ----- */
static uint8_t s_ucHostageStatus = 0;    /* 1=成功 0=失败/超时 */
static uint8_t s_ucHostageCount  = 0;
static stHostageTgtTdf s_astHostageTgt[HOSTAGE_MAX_COUNT];
static uint8_t s_ucHostageReady  = 0;

void vSetHostageResult(uint8_t ucStatus, uint8_t ucCount, const stHostageTgtTdf *pstTgt)
{
    uint8_t ucI;

    if (pstTgt == NULL) { ucCount = 0; }
    if (ucCount > HOSTAGE_MAX_COUNT) { ucCount = HOSTAGE_MAX_COUNT; }
    taskENTER_CRITICAL();
    s_ucHostageStatus = ucStatus;
    s_ucHostageCount  = ucCount;
    for (ucI = 0; ucI < ucCount; ucI++)
    {
        s_astHostageTgt[ucI] = pstTgt[ucI];
    }
    taskEXIT_CRITICAL();
}

void vGetHostageResult(uint8_t *pucStatus, uint8_t *pucCount, stHostageTgtTdf *pstTgt)
{
    uint8_t ucI;

    taskENTER_CRITICAL();
    if (pucStatus != NULL) { *pucStatus = s_ucHostageStatus; }
    if (pucCount  != NULL) { *pucCount  = s_ucHostageCount; }
    if (pstTgt    != NULL)
    {
        for (ucI = 0; ucI < s_ucHostageCount && ucI < HOSTAGE_MAX_COUNT; ucI++)
        {
            pstTgt[ucI] = s_astHostageTgt[ucI];
        }
    }
    taskEXIT_CRITICAL();
}

uint8_t ucTakeHostageReady(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucHostageReady;
    s_ucHostageReady = 0;   /* 读后自动清零 */
    taskEXIT_CRITICAL();
    return uc;
}

void vSetHostageReady(void)
{
    s_ucHostageReady = 1;
}

/* ----- 色球识别结果（同帧多字强一致 → 临界区, 同巡线/人质快照） ----- */
static uint8_t s_ucColorStatus = 0;      /* 1=成功 0=失败/超时 */
static uint8_t s_ucColorCount  = 0;
static stColorTgtTdf s_astColorTgt[COLOR_MAX_COUNT];
static uint8_t s_ucColorReady  = 0;

void vSetColorResult(uint8_t ucStatus, uint8_t ucCount, const stColorTgtTdf *pstTgt)
{
    uint8_t ucI;

    if (pstTgt == NULL) { ucCount = 0; }
    if (ucCount > COLOR_MAX_COUNT) { ucCount = COLOR_MAX_COUNT; }
    taskENTER_CRITICAL();
    s_ucColorStatus = ucStatus;
    s_ucColorCount  = ucCount;
    for (ucI = 0; ucI < ucCount; ucI++)
    {
        s_astColorTgt[ucI] = pstTgt[ucI];
    }
    taskEXIT_CRITICAL();
}

void vGetColorResult(uint8_t *pucStatus, uint8_t *pucCount, stColorTgtTdf *pstTgt)
{
    uint8_t ucI;

    taskENTER_CRITICAL();
    if (pucStatus != NULL) { *pucStatus = s_ucColorStatus; }
    if (pucCount  != NULL) { *pucCount  = s_ucColorCount; }
    if (pstTgt    != NULL)
    {
        for (ucI = 0; ucI < s_ucColorCount && ucI < COLOR_MAX_COUNT; ucI++)
        {
            pstTgt[ucI] = s_astColorTgt[ucI];
        }
    }
    taskEXIT_CRITICAL();
}

uint8_t ucTakeColorReady(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucColorReady;
    s_ucColorReady = 0;   /* 读后自动清零 */
    taskEXIT_CRITICAL();
    return uc;
}

void vSetColorReady(void)
{
    s_ucColorReady = 1;
}

/* ----- 色球帧扩展（小球对位, 2026-09）: 三球连线角 + 共线质检 -----
 * 单字、单写者（vRpiTask）, 按 Beam 规则第 4 条不加锁; 与色球结果的同帧一致性
 * 靠写序（先这里、再 vSetColorResult、最后 vSetColorReady）保证, 见 app_beam.h。 */
static float    s_fBallYawLine = 0.0f;   /* 三球连线图像倾角 [°], y 向下左高右低为正 */
static uint8_t  s_ucBallFitOk  = 1u;     /* 1 = 共线残差合格 */
static uint8_t  s_ucBallPullDir = 0u;    /* 失败帧方向字节(2026-10-26): 0=无信息 1=缺球从画面左出界 2=右; 写方=vRpiTask(同帧先于 vSetColorResult), 读方=ucRpiPullbackAlign */

void vSetBallYawLine(float fVal)   { s_fBallYawLine = fVal; }
float fGetBallYawLine(void)        { return s_fBallYawLine; }
void vSetBallFitOk(uint8_t ucVal)  { s_ucBallFitOk = ucVal; }
uint8_t ucGetBallFitOk(void)       { return s_ucBallFitOk; }
void vSetBallPullDir(uint8_t ucVal){ s_ucBallPullDir = ucVal; }
uint8_t ucGetBallPullDir(void)     { return s_ucBallPullDir; }

/* ----- 蜂鸣请求（2026-10-27 加, 语义见 app_beam.h 那段说明）-----
 * 覆盖写是单字节原子; 读-清不原子 → 进临界区（与 ucTakeMenuConfirm 同一模式）。
 * 生产者有多个（vRpiTask 收 0x0C 帧 / app_rpi 失败点 / 蜂鸣任务陀螺看门狗）,
 * 消费者只有蜂鸣任务一个 —— 读-清由它独占, 不会互相偷。 */
static uint8_t s_ucBuzzerReq = 0u;

void vSetBuzzerReq(uint8_t ucVal)
{
    s_ucBuzzerReq = ucVal;
}

uint8_t ucTakeBuzzerReq(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucBuzzerReq;
    s_ucBuzzerReq = 0u;
    taskEXIT_CRITICAL();
    return uc;
}

/* ----- 矩形地板识别结果（TAG=0x07, 色球区缺球时的备选对位基准, 2026-09 加）-----
 * 同帧多字强一致 → 临界区快照（同色球那一组）。
 * ⚠️ ready 是**一次性信号**（读后清零），唯一合法消费者是 ucRpiWaitRectResult；
 *    别的读者要读结果请用 vGetRectResult（非 take 语义）。
 * ⚠️ 写序：vRpiTask 先 vSetRectResult、最后 vSetRectReady —— 按 ready 门消费的一方
 *    拿到的必是同一帧的值（与色球帧同一条纪律）。 */
static uint8_t  s_ucRectStatus = 0u;   /* 1=成功 0=失败/超时 */
static uint16_t s_usRectCx     = 0u;   /* 板心（全分辨率像素） */
static uint16_t s_usRectCy     = 0u;
static uint8_t  s_ucRectFlags  = 0u;   /* bit0=三边估 bit1~3=边数 bit4=主路径 */
static uint8_t  s_ucRectSpread = 0u;   /* 中心跨帧散布[px]，封顶 255 */
static uint8_t  s_ucRectReady  = 0u;

void vSetRectResult(uint8_t ucStatus, uint16_t usCx, uint16_t usCy,
                    uint8_t ucFlags, uint8_t ucSpread)
{
    taskENTER_CRITICAL();
    s_ucRectStatus = ucStatus;
    s_usRectCx     = usCx;
    s_usRectCy     = usCy;
    s_ucRectFlags  = ucFlags;
    s_ucRectSpread = ucSpread;
    taskEXIT_CRITICAL();
}

void vGetRectResult(uint8_t *pucStatus, uint16_t *pusCx, uint16_t *pusCy,
                    uint8_t *pucFlags, uint8_t *pucSpread)
{
    taskENTER_CRITICAL();
    if (pucStatus != NULL) { *pucStatus = s_ucRectStatus; }
    if (pusCx     != NULL) { *pusCx     = s_usRectCx; }
    if (pusCy     != NULL) { *pusCy     = s_usRectCy; }
    if (pucFlags  != NULL) { *pucFlags  = s_ucRectFlags; }
    if (pucSpread != NULL) { *pucSpread = s_ucRectSpread; }
    taskEXIT_CRITICAL();
}

/* 单独的 status getter: 给蓝牙「任务」组的 rect_status 遥测用（同 color_status/road_status
 * 那几条）。单字读, 按 Beam 规则第 4 条不需要临界区。 */
uint8_t ucGetRectStatus(void)
{
    return s_ucRectStatus;
}

uint8_t ucTakeRectReady(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucRectReady;
    s_ucRectReady = 0u;   /* 读后自动清零 */
    taskEXIT_CRITICAL();
    return uc;
}

void vSetRectReady(void) { s_ucRectReady = 1u; }

/* ----- 看路结果（TAG=0x08, 主摄看地面黄色区, 2026-09 加/第二版）-----
 * 同帧多字强一致 → 临界区快照（同矩形那一组）。写方 = vRpiTask,
 * 读方 = app_rpi 的 ucRpiWaitRoadResult（按 ready 门消费）。
 * 写序: 先 vSetRoadResult、最后 vSetRoadReady —— 按 ready 门拿到的必是同一帧的值。
 * ⚠️ **Δx/Δy 都是 Pi 直接量好的**（Δx=黄色区左边缘的 x, Δy=黄区下边缘的 y,
 *    单位都是 0.1px）, 本端不再自己算距离。
 * ⚠️ dx/ex/ey 是 **int16_t**: dx 给方向对调后留符号、ex 可以到 0 附近;
 *    用无符号会被钳成 0 而且**不报错**（与 Pi 侧 build_road_frame 的 i16 逐位对齐,
 *    见 docs/串口协议.md §6i）。dy 反过来: 下边缘的 y 不可能为负 → **u16**。
 * ⚠️ dy10 是**追加在参数表最后**的: 既有实参位置一个不动, 老调用点不会串位。 */
static uint8_t  s_ucRoadStatus = 0u;   /* 1=成功 0=失败/超时 */
static int16_t  s_nRoadDx10    = 0;    /* Δx（黄色区左边缘 x）, 单位 0.1px */
static int16_t  s_nRoadEx      = 0;    /* 左边缘那一点的全分辨率像素坐标（遥测用） */
static int16_t  s_nRoadEy      = 0;
static uint8_t  s_ucRoadFlags  = 0u;   /* bit0=边缘有效 bit1=贴画面左边(饱和) bit2=贴上下边 bit3=面积勉强 bit4=Δy有效 */
static uint8_t  s_ucRoadSpread = 0u;   /* Δx 跨帧散布[px]，封顶 255 */
static uint16_t s_usRoadDy10   = 0u;   /* Δy（黄色区下边缘 y）, 单位 0.1px; 有效与否看 flags 的 bit4 */
static uint8_t  s_ucRoadReady  = 0u;
static float    s_fRoadDxMeas  = 0.0f; /* 同 dx, 换算成 px 的浮点 —— 只做遥测（单写者 = 脚本任务） */
static float    s_fRoadDyMeas  = 0.0f; /* 同 dy, 换算成 px 的浮点 —— 只做遥测（标定就是读它） */

void vSetRoadResult(uint8_t ucStatus, int16_t nDx10, int16_t nEx, int16_t nEy,
                    uint8_t ucFlags, uint8_t ucSpread, uint16_t usDy10)
{
    taskENTER_CRITICAL();
    s_ucRoadStatus = ucStatus;
    s_nRoadDx10    = nDx10;
    s_nRoadEx      = nEx;
    s_nRoadEy      = nEy;
    s_ucRoadFlags  = ucFlags;
    s_ucRoadSpread = ucSpread;
    s_usRoadDy10   = usDy10;
    taskEXIT_CRITICAL();
}

void vGetRoadResult(uint8_t *pucStatus, int16_t *pnDx10, int16_t *pnEx,
                    int16_t *pnEy, uint8_t *pucFlags, uint8_t *pucSpread,
                    uint16_t *pusDy10)
{
    taskENTER_CRITICAL();
    if (pucStatus  != NULL) { *pucStatus  = s_ucRoadStatus; }
    if (pnDx10     != NULL) { *pnDx10     = s_nRoadDx10; }
    if (pnEx       != NULL) { *pnEx       = s_nRoadEx; }
    if (pnEy       != NULL) { *pnEy       = s_nRoadEy; }
    if (pucFlags   != NULL) { *pucFlags   = s_ucRoadFlags; }
    if (pucSpread  != NULL) { *pucSpread  = s_ucRoadSpread; }
    if (pusDy10    != NULL) { *pusDy10    = s_usRoadDy10; }
    taskEXIT_CRITICAL();
}

uint8_t ucTakeRoadReady(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucRoadReady;
    s_ucRoadReady = 0u;   /* 读后自动清零 */
    taskEXIT_CRITICAL();
    return uc;
}

void  vSetRoadReady(void)          { s_ucRoadReady = 1u; }
void  vSetRoadDxMeas(float fVal)   { s_fRoadDxMeas = fVal; }
float fGetRoadDxMeas(void)         { return s_fRoadDxMeas; }
void  vSetRoadDyMeas(float fVal)   { s_fRoadDyMeas = fVal; }
float fGetRoadDyMeas(void)         { return s_fRoadDyMeas; }

/* ----- 看路校准结果（TAG=0x09, 主摄拟合黄区直边图像方向角, 2026-10 加）-----
 * 与看路那组同构: 临界区快照 + ready 门 + 失败帧也置 ready。
 * ang = 黄区直边与**图像竖直方向**的夹角(0.01°, i16)。黄区固定在场上、相机固定在车上
 * ⇒ 它只随**车航向**变 —— 陀螺漂移的视觉基准。消费方 ucRpiRoadYaw **只测不改**。 */
static uint8_t  s_ucRoadYawStatus = 0u;  /* 1=成功 0=失败/超时 */
static int16_t  s_nRoadYawAng100  = 0;   /* 方向角, 单位 0.01°（i16, (−90,+90]） */
static uint8_t  s_ucRoadYawCols   = 0u;  /* 拟合内点数（遥测） */
static uint8_t  s_ucRoadYawRms10  = 0u;  /* 内点残差 RMS, 0.1px（质量门） */
static uint8_t  s_ucRoadYawSpread = 0u;  /* 角度跨帧散布, 0.01°（遥测） */
static uint8_t  s_ucRoadYawFlags  = 0u;  /* bit0=拟合有效 bit1=直段短/点少(勉强) */
static uint8_t  s_ucRoadYawReady  = 0u;
static float    s_fRoadYawMeas    = 0.0f; /* 换算成 ° 的实测方向角 —— 只做遥测 */
static float    s_fRoadYawErr     = 0.0f; /* (实测−ideal)×方向 [°] —— 只做遥测 */

void vSetRoadYawResult(uint8_t ucStatus, int16_t nAng100, uint8_t ucCols,
                       uint8_t ucRms10, uint8_t ucSpread, uint8_t ucFlags)
{
    taskENTER_CRITICAL();
    s_ucRoadYawStatus = ucStatus;
    s_nRoadYawAng100  = nAng100;
    s_ucRoadYawCols   = ucCols;
    s_ucRoadYawRms10  = ucRms10;
    s_ucRoadYawSpread = ucSpread;
    s_ucRoadYawFlags  = ucFlags;
    taskEXIT_CRITICAL();
}

void vGetRoadYawResult(uint8_t *pucStatus, int16_t *pnAng100, uint8_t *pucCols,
                       uint8_t *pucRms10, uint8_t *pucSpread, uint8_t *pucFlags)
{
    taskENTER_CRITICAL();
    if (pucStatus  != NULL) { *pucStatus  = s_ucRoadYawStatus; }
    if (pnAng100   != NULL) { *pnAng100   = s_nRoadYawAng100; }
    if (pucCols    != NULL) { *pucCols    = s_ucRoadYawCols; }
    if (pucRms10   != NULL) { *pucRms10   = s_ucRoadYawRms10; }
    if (pucSpread  != NULL) { *pucSpread  = s_ucRoadYawSpread; }
    if (pucFlags   != NULL) { *pucFlags   = s_ucRoadYawFlags; }
    taskEXIT_CRITICAL();
}

uint8_t ucTakeRoadYawReady(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucRoadYawReady;
    s_ucRoadYawReady = 0u;   /* 读后自动清零 */
    taskEXIT_CRITICAL();
    return uc;
}

void  vSetRoadYawReady(void)       { s_ucRoadYawReady = 1u; }
void  vSetRoadYawMeas(float fVal)  { s_fRoadYawMeas = fVal; }
float fGetRoadYawMeas(void)        { return s_fRoadYawMeas; }
void  vSetRoadYawErr(float fVal)   { s_fRoadYawErr = fVal; }
float fGetRoadYawErr(void)         { return s_fRoadYawErr; }

/* ----- 看路对位标定量（蓝牙可调, ⚠️易失: 断电回 app_config.h 的 ROAD_* 默认）-----
 * 与小球/人质/标靶那几组同一条纪律: **每个区一套自己的标定**, 不共用。
 * 写方 = app_bt; 读方 = app_rpi（ucRpiRoadAlign 每次执行现取现用）。
 * ⚠️ 纵向(road_*) 与横向(road_*_y)是**两套独立的尺**: 图像 x↔车体纵向、图像 y↔车体横向
 *    （本车相机相对车体转了 90°）。横向那一套只在积木参数 use_y=1 时参与运算。 */
static float   s_fRoadDxIdeal  = ROAD_DX_IDEAL_PX;
static float   s_fRoadDxIdealRev = ROAD_DX_IDEAL_REV_PX;   /* 反向看路(rev=1)那把 */
static float   s_fRoadScale    = ROAD_DX_SCALE_MM_PER_PX;
static float   s_fRoadSign     = ROAD_DX_DIR;
static float   s_fRoadDeadband = ROAD_DX_DEADBAND_MM;
static float   s_fRoadMaxMm    = ROAD_DX_MAX_MM;
static int32_t s_iRoadAccel    = ROAD_ALIGN_ACCEL;
static int32_t s_iRoadSpeed    = ROAD_ALIGN_SPEED;
static float   s_fRoadDyIdeal  = ROAD_DY_IDEAL_PX;
static float   s_fRoadDyIdealRev = ROAD_DY_IDEAL_REV_PX;   /* 反向看路(rev=1)那把; 0=未标定→横向跳过 */
static float   s_fRoadScaleY   = ROAD_DY_SCALE_MM_PER_PX;
static float   s_fRoadDySign   = ROAD_DY_DIR;
static float   s_fRoadDyDeadband = ROAD_DY_DEADBAND_MM;
static float   s_fRoadRescueMm = ROAD_RESCUE_MM;

void     vSetRoadDxIdeal(float fVal)  { s_fRoadDxIdeal = fVal; }
float    fGetRoadDxIdeal(void)        { return s_fRoadDxIdeal; }
void     vSetRoadDxIdealRev(float fVal) { s_fRoadDxIdealRev = fVal; }
float    fGetRoadDxIdealRev(void)      { return s_fRoadDxIdealRev; }
void     vSetRoadScale(float fVal)    { s_fRoadScale = fVal; }
float    fGetRoadScale(void)          { return s_fRoadScale; }
void     vSetRoadSign(float fVal)     { s_fRoadSign = fVal; }
float    fGetRoadSign(void)           { return s_fRoadSign; }
void     vSetRoadDeadband(float fVal) { s_fRoadDeadband = fVal; }
float    fGetRoadDeadband(void)       { return s_fRoadDeadband; }
void     vSetRoadMaxMm(float fVal)    { s_fRoadMaxMm = fVal; }
float    fGetRoadMaxMm(void)          { return s_fRoadMaxMm; }
void     vSetRoadAccel(int32_t iVal)  { s_iRoadAccel = iVal; }
int32_t  iGetRoadAccel(void)          { return s_iRoadAccel; }
void     vSetRoadSpeed(int32_t iVal)  { s_iRoadSpeed = iVal; }
int32_t  iGetRoadSpeed(void)          { return s_iRoadSpeed; }
void     vSetRoadDyIdeal(float fVal)  { s_fRoadDyIdeal = fVal; }
float    fGetRoadDyIdeal(void)        { return s_fRoadDyIdeal; }
void     vSetRoadDyIdealRev(float fVal) { s_fRoadDyIdealRev = fVal; }
float    fGetRoadDyIdealRev(void)      { return s_fRoadDyIdealRev; }
void     vSetRoadScaleY(float fVal)   { s_fRoadScaleY = fVal; }
float    fGetRoadScaleY(void)         { return s_fRoadScaleY; }
void     vSetRoadDySign(float fVal)   { s_fRoadDySign = fVal; }
float    fGetRoadDySign(void)         { return s_fRoadDySign; }
void     vSetRoadDyDeadband(float fVal) { s_fRoadDyDeadband = fVal; }
float    fGetRoadDyDeadband(void)     { return s_fRoadDyDeadband; }
void     vSetRoadRescueMm(float fVal) { s_fRoadRescueMm = fVal; }
float    fGetRoadRescueMm(void)       { return s_fRoadRescueMm; }

/* ----- 看路校准标定量（蓝牙可调, ⚠️易失: 断电回 app_config.h 的 ROADYAW_* 默认）-----
 * ideal 的**未标定哨兵是 |值|>90, 不是 0** —— 竖直的直边 ang 本来就 = 0°, 0 是合法值。 */
static float   s_fRoadYawIdeal = ROADYAW_ANGLE_IDEAL_DEG;
static float   s_fRoadYawDir   = ROADYAW_DIR;

void     vSetRoadYawIdeal(float fVal) { s_fRoadYawIdeal = fVal; }
float    fGetRoadYawIdeal(void)       { return s_fRoadYawIdeal; }
void     vSetRoadYawDir(float fVal)   { s_fRoadYawDir = fVal; }
float    fGetRoadYawDir(void)         { return s_fRoadYawDir; }

/* ----- 看路校准的陀螺漂移偏置（2026-10 加）-----
 * 链路见 app_beam.h 那段。写方 = app_rpi（ucRpiRoadYaw mode=1/2）;
 * 读方 = app_sensors（vGyroPublishAttitude 发布 yaw 时扣偏置, gyro_yaw_apply=1 才扣）。
 * ⚠️ 偏置钳位在 app_rpi 侧做（ROADYAW_BIAS_MAX_DEG）, 这里只是存取。 */
static float   s_fRoadYawBias     = 0.0f;   /* 当前对齐偏置 [°], 上电=0 */
static uint8_t s_ucRoadYawCalibOk = 0u;      /* 已定标过一次（首次对齐不限幅, 之后每拍限幅） */

void     vSetRoadYawBias(float fVal)     { s_fRoadYawBias = fVal; }
float    fGetRoadYawBias(void)           { return s_fRoadYawBias; }
void     vSetRoadYawCalibOk(uint8_t ucVal) { s_ucRoadYawCalibOk = ucVal; }
uint8_t  ucGetRoadYawCalibOk(void)       { return s_ucRoadYawCalibOk; }

/* ----- 对位方案链的结果（只读遥测, 2026-09 加）-----
 * 「这一轮到底哪个方案把车对上了」是现场唯一看得见的答案（链内失败统一静音、Pi 日志事后才能翻）。
 * 写方 = app_rpi（ucRpiAlignChain）；读方 = 蓝牙 Tab 遥测 / 排障。 */
static uint8_t s_ucAlignOk     = 0u;   /* 1=链成功 0=全失败/没配方案 */
static uint8_t s_ucAlignMethod = 0u;   /* 成功的那一槽的方案码(emAlignMethodTdf) */

void    vSetAlignOk(uint8_t ucVal)     { s_ucAlignOk = ucVal; }
uint8_t ucGetAlignOk(void)             { return s_ucAlignOk; }
void    vSetAlignMethod(uint8_t ucVal) { s_ucAlignMethod = ucVal; }
uint8_t ucGetAlignMethod(void)         { return s_ucAlignMethod; }

/* ----- 脚本是否正在跑（vScriptRun 在跑 = 1; 返回后 vScriptTask 永久停车 = 0）-----
 * 两个用处: ① 蓝牙编辑模式的入场条件之一 —— 脚本没在跑时路线不可能动舵机,
 *              让操作员进编辑模式点动是安全的, 而那正是台架调臂的常态
 *              （否则「车停着、人围着调臂」却进不去, 只见 NOHANDSHAKE）;
 *           ② 只读遥测: 它回答「现在到底还在不在跑路线」（script_preset_id 只说跑到哪张表）。 */
static uint8_t  s_ucScriptRunning = 0u;

void     vSetScriptRunning(uint8_t ucVal) { s_ucScriptRunning = ucVal; }
uint8_t  ucGetScriptRunning(void)         { return s_ucScriptRunning; }

/* 脚本请求槽是否有待处理的请求（2026-10 加, 蓝牙编辑模式的入场门用）。
 * 投递(vScriptPostReq) → 消费(vScriptTask 取走) 之间有个 running 尚为 0 的窗口
 * （最长一个 50ms 轮询周期）, 只看 ucGetScriptRunning 会让 PC 在这个窗口里进编辑
 * 模式、随后路线照样开跑 —— 点动与脚本抢舵机的「结构性保证」就漏了。
 * 16 位读天然原子（Cortex-M）, 与 vScriptPostReq 的写入无竞争危害（最坏晚一拍）。 */
uint8_t  ucGetScriptReqPending(void)      { return (s_usScriptReq != 0u) ? 1u : 0u; }

/* ----- 小球对位标定量（运行时蓝牙可调, 断电回编译期默认） -----
 * 「调参」Tab 改的是 app_config.h 里的 BALL_* 宏 = **开机默认值（持久）**;
 * 「蓝牙」Tab 改的是这里的 RAM 变量 = **当场生效（易失）**。现场标定走蓝牙零重编译,
 * 标好了把终值写回 app_config.h 固化。写方 = app_bt（$B,SET 经 ucBeamApply 钳位）,
 * 读方 = app_rpi（ucRpiBallAlign 每次执行现取现用, 不缓存）。 */
static float    s_fBallCxCenter    = BALL_CX_CENTER;
static float    s_fBallCyCenter    = BALL_CY_CENTER;
static float    s_fBallPxToMmX     = BALL_PX_TO_MM_X;
static float    s_fBallPxToMmY     = BALL_PX_TO_MM_Y;
static float    s_fBallYawScale    = BALL_YAW_SCALE;
static float    s_fBallYawSign     = BALL_YAW_SIGN;
static float    s_fBallYawTarget   = BALL_YAW_TARGET_DEG;
static float    s_fBallYawDeadband = BALL_YAW_DEADBAND_DEG;
static float    s_fBallDxDeadband  = BALL_DX_DEADBAND_MM;
static float    s_fBallDyDeadband  = BALL_DY_DEADBAND_MM;
static float    s_fBallFwdSign     = BALL_FWD_SIGN;
static uint8_t  s_ucBallYawEn      = BALL_ALIGN_YAW_EN;
static uint8_t  s_ucBallXySwap     = BALL_XY_SWAP;
static int32_t  s_iBallAlignAccel  = BALL_ALIGN_ACCEL;
static int32_t  s_iBallAlignSpeed  = BALL_ALIGN_SPEED;
static float    s_fBallPullbackMm  = BALL_PULLBACK_MM;

/* ----- 矩形板心对位标定量（只有两个中心量; 其余整套复用上面的 BALL_*, 2026-09 加）-----
 * 写方 = app_bt; 读方 = app_rpi（ucRpiRectAlign 每次执行现取现用, 不缓存）。
 * ⚠️ 与 ball_cx/cy_center **同一个停车位姿**标定（两种测量定义差 ~15px≈5mm,
 *    各自被各自中心量吸收, 别再加补偿量）。 */
static float s_fRectCxCenter = RECT_ALIGN_CX;
static float s_fRectCyCenter = RECT_ALIGN_CY;

void  vSetRectCxCenter(float fVal) { s_fRectCxCenter = fVal; }
float fGetRectCxCenter(void)       { return s_fRectCxCenter; }
void  vSetRectCyCenter(float fVal) { s_fRectCyCenter = fVal; }
float fGetRectCyCenter(void)       { return s_fRectCyCenter; }

void     vSetBallCxCenter(float fVal)    { s_fBallCxCenter = fVal; }
float    fGetBallCxCenter(void)          { return s_fBallCxCenter; }
void     vSetBallCyCenter(float fVal)    { s_fBallCyCenter = fVal; }
float    fGetBallCyCenter(void)          { return s_fBallCyCenter; }
void     vSetBallPxToMmX(float fVal)     { s_fBallPxToMmX = fVal; }
float    fGetBallPxToMmX(void)           { return s_fBallPxToMmX; }
void     vSetBallPxToMmY(float fVal)     { s_fBallPxToMmY = fVal; }
float    fGetBallPxToMmY(void)           { return s_fBallPxToMmY; }
void     vSetBallYawScale(float fVal)    { s_fBallYawScale = fVal; }
float    fGetBallYawScale(void)          { return s_fBallYawScale; }
void     vSetBallYawSign(float fVal)     { s_fBallYawSign = fVal; }
float    fGetBallYawSign(void)           { return s_fBallYawSign; }
void     vSetBallYawTarget(float fVal)   { s_fBallYawTarget = fVal; }
float    fGetBallYawTarget(void)         { return s_fBallYawTarget; }
void     vSetBallYawDeadband(float fVal) { s_fBallYawDeadband = fVal; }
float    fGetBallYawDeadband(void)       { return s_fBallYawDeadband; }
void     vSetBallDxDeadband(float fVal)  { s_fBallDxDeadband = fVal; }
float    fGetBallDxDeadband(void)        { return s_fBallDxDeadband; }
void     vSetBallDyDeadband(float fVal)  { s_fBallDyDeadband = fVal; }
float    fGetBallDyDeadband(void)        { return s_fBallDyDeadband; }
void     vSetBallFwdSign(float fVal)     { s_fBallFwdSign = fVal; }
float    fGetBallFwdSign(void)           { return s_fBallFwdSign; }
void     vSetBallYawEn(uint8_t ucVal)    { s_ucBallYawEn = ucVal; }
uint8_t  ucGetBallYawEn(void)            { return s_ucBallYawEn; }
void     vSetBallXySwap(uint8_t ucVal)   { s_ucBallXySwap = ucVal; }
uint8_t  ucGetBallXySwap(void)           { return s_ucBallXySwap; }
void     vSetBallAlignAccel(int32_t iVal) { s_iBallAlignAccel = iVal; }
int32_t  iGetBallAlignAccel(void)         { return s_iBallAlignAccel; }
void     vSetBallAlignSpeed(int32_t iVal) { s_iBallAlignSpeed = iVal; }
int32_t  iGetBallAlignSpeed(void)         { return s_iBallAlignSpeed; }
void     vSetBallPullbackMm(float fVal)   { s_fBallPullbackMm = fVal; }
float    fGetBallPullbackMm(void)         { return s_fBallPullbackMm; }

/* ----- 人质对位标定量（2026-09 加; 语义与用法同上面那组, 只是另一套值） -----
 * 为什么不复用 s_fBallXxx: 人质区与球区的理想停止位姿不同, 一套值到了另一边就是
 * 系统性偏差, 而两边会分别标定 —— 共用意味着"调好了球区就动摇了人质区"。
 * 没有 yaw 相关量: 0x02 人质帧不含连线角, 本步也不做转正（见 app_config.h 段头）。 */
static float    s_fHsCxCenter   = HOSTAGE_CX_CENTER;
static float    s_fHsCyCenter   = HOSTAGE_CY_CENTER;
static float    s_fHsPxToMmX    = HOSTAGE_PX_TO_MM_X;
static float    s_fHsPxToMmY    = HOSTAGE_PX_TO_MM_Y;
static float    s_fHsDxDeadband = HOSTAGE_DX_DEADBAND_MM;
static float    s_fHsDyDeadband = HOSTAGE_DY_DEADBAND_MM;
static float    s_fHsSideSign   = HOSTAGE_SIDE_SIGN;
static float    s_fHsFwdSign    = HOSTAGE_FWD_SIGN;
static uint8_t  s_ucHsXySwap    = HOSTAGE_XY_SWAP;
static int32_t  s_iHsAlignAccel = HOSTAGE_ALIGN_ACCEL;
static int32_t  s_iHsAlignSpeed = HOSTAGE_ALIGN_SPEED;

void     vSetHsCxCenter(float fVal)   { s_fHsCxCenter = fVal; }
float    fGetHsCxCenter(void)         { return s_fHsCxCenter; }
void     vSetHsCyCenter(float fVal)   { s_fHsCyCenter = fVal; }
float    fGetHsCyCenter(void)         { return s_fHsCyCenter; }
void     vSetHsPxToMmX(float fVal)    { s_fHsPxToMmX = fVal; }
float    fGetHsPxToMmX(void)          { return s_fHsPxToMmX; }
void     vSetHsPxToMmY(float fVal)    { s_fHsPxToMmY = fVal; }
float    fGetHsPxToMmY(void)          { return s_fHsPxToMmY; }
void     vSetHsDxDeadband(float fVal) { s_fHsDxDeadband = fVal; }
float    fGetHsDxDeadband(void)       { return s_fHsDxDeadband; }
void     vSetHsDyDeadband(float fVal) { s_fHsDyDeadband = fVal; }
float    fGetHsDyDeadband(void)       { return s_fHsDyDeadband; }
void     vSetHsSideSign(float fVal)   { s_fHsSideSign = fVal; }
float    fGetHsSideSign(void)         { return s_fHsSideSign; }
void     vSetHsFwdSign(float fVal)    { s_fHsFwdSign = fVal; }
float    fGetHsFwdSign(void)          { return s_fHsFwdSign; }
void     vSetHsXySwap(uint8_t ucVal)  { s_ucHsXySwap = ucVal; }
uint8_t  ucGetHsXySwap(void)          { return s_ucHsXySwap; }
void     vSetHsAlignAccel(int32_t iVal) { s_iHsAlignAccel = iVal; }
int32_t  iGetHsAlignAccel(void)         { return s_iHsAlignAccel; }
void     vSetHsAlignSpeed(int32_t iVal) { s_iHsAlignSpeed = iVal; }
int32_t  iGetHsAlignSpeed(void)         { return s_iHsAlignSpeed; }

/* ----- 单动作抓取标定量（2026-10 加; 只录一个"抓中间"的动作, 其余目标靠平移挪过来） -----
 * 色球版(ucRpiBallGrabOne)与人质版(ucRpiHostageGrabOne)共用这一组 —— 两个区
 * 平移的都是"相邻目标间距"这一个物理量, 各自再配一套会多出一堆没人调的旋钮;
 * 真到现场发现两区间距不同, 再拆成两套也不迟。 */
static float    s_fOneLatMm     = ONE_GRAB_LAT_MM;
static uint8_t  s_ucOneLatDir   = ONE_GRAB_LAT_DIR;
static int32_t  s_iOneLineMs    = (int32_t)ONE_GRAB_LINE_MS;

void     vSetOneLatMm(float fVal)        { s_fOneLatMm = fVal; }
float    fGetOneLatMm(void)              { return s_fOneLatMm; }
void     vSetOneLatDir(uint8_t ucVal)    { s_ucOneLatDir = ucVal; }
uint8_t  ucGetOneLatDir(void)            { return s_ucOneLatDir; }
void     vSetOneLineMs(int32_t iVal)     { s_iOneLineMs = iVal; }
int32_t  iGetOneLineMs(void)             { return s_iOneLineMs; }

/* ----- 机械臂舵机运行时量（2026-10 加; 上电初值 = app_config.h, 见 app_beam.h） -----
 * ⚠️ 两个 setter 都是"存 + 当场下发": 驱动在周期执行里读的是**舵机自己的 fSpeed 字段**
 *    和**驱动里的全局曲线**, 只写 Beam 驱动根本不知道 —— 与"存起来、用时再取"的
 *    标定量（BALL_* 那批）刻意不同, 别"顺手统一"成纯存储。 */
static float    s_fArmSpeedDps  = ARM_SERVO_MAX_SPEED_DPS;
static uint8_t  s_ucArmCurve    = ARM_SERVO_CURVE;

void vSetArmSpeedDps(float fVal)
{
    uint8_t ucI;

    if (fVal < 30.0f)   { fVal = 30.0f; }     /* 5V 舵机再慢也别低于 30°/s（等于基本不动） */
    if (fVal > 1000.0f) { fVal = 1000.0f; }   /* 1000 = 实际不限速（空载才 375~500°/s） */
    s_fArmSpeedDps = fVal;
    /* 当场下发全部舵机（与 vAppArmInit 同一遍, 含夹爪）; 对**正在播放**的动作下一拍
     * 削峰就按新值来 —— "中途改速度"的语义就在这。 */
    for (ucI = 0u; ucI < (uint8_t)SERVO_DEV_NUM; ucI++)
    {
        vServoSetSpeed((emServoDevNumTdf)ucI, fVal / 1000.0f);
    }
}

float fGetArmSpeedDps(void) { return s_fArmSpeedDps; }

void vSetArmCurve(uint8_t ucVal)
{
    /* ⚠️ **先钳位再存**: 驱动 setter 内部对越界值是"回五次S", 而 Beam 存的是原值 ——
     * 于是蓝牙写 arm_curve=7 时, 驱动用五次S、`ucGetArmCurve()` 与「蓝牙」Tab 却报 7
     * （UI 滑块的上界只挡得住滑块, 挡不住 $B,SET 直接写）。排障会被这个数骗。
     * 存"驱动实际采用的那一档", 遥测才与硬件一致（同 vSetArmSpeedDps 的口径）。 */
    if (ucVal > (uint8_t)emServoCurve_Linear) { ucVal = (uint8_t)emServoCurve_Hermite5; }
    s_ucArmCurve = ucVal;
    vServoDeviceSetCurve((emServoCurveTdf)ucVal);
}

uint8_t ucGetArmCurve(void) { return s_ucArmCurve; }

/* ----- 抓取任务残留偏移（世界系 mm; 两 float 成对, 临界区保强一致） -----
 * 人质单动作抓取不回中: 它把平移过的总位移按当时指令航向换算成世界系记在这里,
 * 后续世界系平移积木勾"叠加上次残留偏移"时由 vConsumePathResidual 一次性吃掉。 */
static float    s_fPathResX     = 0.0f;
static float    s_fPathResY     = 0.0f;

void vSetPathResidual(float fWx, float fWy)
{
    taskENTER_CRITICAL();
    s_fPathResX = fWx;
    s_fPathResY = fWy;
    taskEXIT_CRITICAL();
}

void vGetPathResidual(float *pfWx, float *pfWy)
{
    if ((pfWx == NULL) || (pfWy == NULL)) { return; }
    taskENTER_CRITICAL();
    *pfWx = s_fPathResX;
    *pfWy = s_fPathResY;
    taskEXIT_CRITICAL();
}

void vConsumePathResidual(float *pfWx, float *pfWy)
{
    if ((pfWx == NULL) || (pfWy == NULL)) { return; }
    taskENTER_CRITICAL();
    *pfWx = s_fPathResX;
    *pfWy = s_fPathResY;
    s_fPathResX = 0.0f;               /* 一次性: 消费即清零, 下次再消费就是 0 */
    s_fPathResY = 0.0f;
    taskEXIT_CRITICAL();
}

float fGetPathResX(void) { return s_fPathResX; }   /* 仅蓝牙遥测表用（单值, 无清零副作用） */
float fGetPathResY(void) { return s_fPathResY; }

/* ----- 标靶校准标定量（2026-09 加; 语义与用法同上面两组, 见 app_beam.h 的说明） -----
 * 为什么必须给标靶补这一组: 它是三个标定环节里**唯一**没有运行时可调量的 —— 标定时
 * 「先定方向、再定比例尺」要反复试, 而原先每试一次都得重编译烧写一轮。
 * 没有 yaw / swap 之外的姿态量: 标靶是**增量式开环微调**（见 app_rpi.c::vTargetCalibAim）,
 * 不闭环、不走底盘, 所以不需要 BALL_/HOSTAGE_ 那套速度/加速度/死区mm。 */
static float    s_fTgCenterX    = TARGET_CENTER_X;
static float    s_fTgCenterY    = TARGET_CENTER_Y;
static float    s_fTgPxToDegX   = TARGET_PX_TO_DEG_X;
static float    s_fTgPxToDegY   = TARGET_PX_TO_DEG_Y;
static float    s_fTgXSign      = TARGET_X_SIGN;
static float    s_fTgYSign      = TARGET_Y_SIGN;
static float    s_fTgDeadbandPx = TARGET_DEADBAND_PX;
static float    s_fTgMaxStepDeg = TARGET_MAX_STEP_DEG;
static int32_t  s_iTgMoveMs     = (int32_t)TARGET_MOVE_MS;
static int32_t  s_iTgMoveSlackMs = (int32_t)TARGET_MOVE_SLACK_MS;
static uint8_t  s_ucTgXySwap    = TARGET_XY_SWAP;
static int32_t  s_iTgCalibTimeoutMs = (int32_t)TARGET_CALIB_TIMEOUT_MS;

void     vSetTgCenterX(float fVal)    { s_fTgCenterX = fVal; }
float    fGetTgCenterX(void)          { return s_fTgCenterX; }
void     vSetTgCenterY(float fVal)    { s_fTgCenterY = fVal; }
float    fGetTgCenterY(void)          { return s_fTgCenterY; }
void     vSetTgPxToDegX(float fVal)   { s_fTgPxToDegX = fVal; }
float    fGetTgPxToDegX(void)         { return s_fTgPxToDegX; }
void     vSetTgPxToDegY(float fVal)   { s_fTgPxToDegY = fVal; }
float    fGetTgPxToDegY(void)         { return s_fTgPxToDegY; }
void     vSetTgXSign(float fVal)      { s_fTgXSign = fVal; }
float    fGetTgXSign(void)            { return s_fTgXSign; }
void     vSetTgYSign(float fVal)      { s_fTgYSign = fVal; }
float    fGetTgYSign(void)            { return s_fTgYSign; }
void     vSetTgDeadbandPx(float fVal) { s_fTgDeadbandPx = fVal; }
float    fGetTgDeadbandPx(void)       { return s_fTgDeadbandPx; }
void     vSetTgMaxStepDeg(float fVal) { s_fTgMaxStepDeg = fVal; }
float    fGetTgMaxStepDeg(void)       { return s_fTgMaxStepDeg; }
void     vSetTgMoveMs(int32_t iVal)   { s_iTgMoveMs = iVal; }
int32_t  iGetTgMoveMs(void)          { return s_iTgMoveMs; }
void     vSetTgMoveSlackMs(int32_t iVal) { s_iTgMoveSlackMs = iVal; }
int32_t  iGetTgMoveSlackMs(void)        { return s_iTgMoveSlackMs; }
void     vSetTgXySwap(uint8_t ucVal)  { s_ucTgXySwap = ucVal; }
uint8_t  ucGetTgXySwap(void)          { return s_ucTgXySwap; }
void     vSetTgCalibTimeoutMs(int32_t iVal) { s_iTgCalibTimeoutMs = iVal; }
int32_t  iGetTgCalibTimeoutMs(void)        { return s_iTgCalibTimeoutMs; }

/* ----- 标靶对位标定量（2026-09 加; 语义与用法同小球/人质那两组, 见 app_beam.h） -----
 * 只做横向: 反恐区的前后距离由路线排定, 本步不动它 —— 所以没有 px_to_mm_y / fwd_sign /
 * dy_deadband 这三个"纵移专用"量（人质那组有）。cy_center 留着是给 xy_swap 用的:
 * 开了对调之后横向改由 cy 驱动, 那时它就需要一个基准行。 */
static float    s_fTaCxCenter   = TARGET_ALIGN_CX_CENTER;
static float    s_fTaCyCenter   = TARGET_ALIGN_CY_CENTER;
static float    s_fTaPxToMmX    = TARGET_ALIGN_PX_TO_MM_X;
static float    s_fTaDxDeadband = TARGET_ALIGN_DX_DEADBAND_MM;
static float    s_fTaSideSign   = TARGET_ALIGN_SIDE_SIGN;
static uint8_t  s_ucTaXySwap    = TARGET_ALIGN_XY_SWAP;
static int32_t  s_iTaAlignAccel = TARGET_ALIGN_ACCEL;
static int32_t  s_iTaAlignSpeed = TARGET_ALIGN_SPEED;

void     vSetTaCxCenter(float fVal)   { s_fTaCxCenter = fVal; }
float    fGetTaCxCenter(void)         { return s_fTaCxCenter; }
void     vSetTaCyCenter(float fVal)   { s_fTaCyCenter = fVal; }
float    fGetTaCyCenter(void)         { return s_fTaCyCenter; }
void     vSetTaPxToMmX(float fVal)    { s_fTaPxToMmX = fVal; }
float    fGetTaPxToMmX(void)          { return s_fTaPxToMmX; }
void     vSetTaDxDeadband(float fVal) { s_fTaDxDeadband = fVal; }
float    fGetTaDxDeadband(void)       { return s_fTaDxDeadband; }
void     vSetTaSideSign(float fVal)   { s_fTaSideSign = fVal; }
float    fGetTaSideSign(void)         { return s_fTaSideSign; }
void     vSetTaXySwap(uint8_t ucVal)  { s_ucTaXySwap = ucVal; }
uint8_t  ucGetTaXySwap(void)          { return s_ucTaXySwap; }
void     vSetTaAlignAccel(int32_t iVal) { s_iTaAlignAccel = iVal; }
int32_t  iGetTaAlignAccel(void)         { return s_iTaAlignAccel; }
void     vSetTaAlignSpeed(int32_t iVal) { s_iTaAlignSpeed = iVal; }
int32_t  iGetTaAlignSpeed(void)         { return s_iTaAlignSpeed; }

/* ----- 标靶流控闭环节拍量（反恐区打靶, 2026-09 加; 见 app_beam.h 的分工说明） -----
 * 单字、写者都是任务上下文（vBtTask 的 $B,SET / 脚本任务读取）, 按 Beam 规则第 4 条不加锁。
 * ⚠️ 瞄准点/比例尺/方向/XY对调**不在本组** —— 那些与「标靶校准」共用 tg_*。 */
static int32_t s_iTsCtrlMs    = (int32_t)TARGET_SERVO_CTRL_MS;
static int32_t s_iTsMoveMs    = (int32_t)TARGET_SERVO_MOVE_MS;
static float   s_fTsStepDeg   = TARGET_SERVO_STEP_DEG;
static float   s_fTsConvPx    = TARGET_SERVO_CONV_PX;
static int32_t s_iTsConvTicks = (int32_t)TARGET_SERVO_CONV_TICKS;
static float   s_fTsTravelDeg = TARGET_SERVO_TRAVEL_DEG;
static int32_t s_iTsStaleMs   = (int32_t)TARGET_SERVO_STALE_MS;
static float   s_fTsKpScale   = TARGET_SERVO_KP_SCALE;
/* 底盘参与闭环（2026-09 加）—— 见 app_beam.h 该组的契约说明 */
static uint8_t s_ucTsChassisEn     = TARGET_SERVO_CHASSIS_EN;
static float   s_fTsChassisPx      = TARGET_SERVO_CHASSIS_PX;
static float   s_fTsChassisPxToMm  = TARGET_SERVO_CHASSIS_PX_TO_MM;
static float   s_fTsChassisSign    = TARGET_SERVO_CHASSIS_SIGN;
static float   s_fTsChassisMm      = TARGET_SERVO_CHASSIS_MM;
/* 阶段化 X→Y + 踢步 + 距离辅助（2026-10-06 加; ts_seq_en=0 整组不参与, 见 app_config.h 段头） */
static uint8_t s_ucTsSeqEn      = TARGET_SERVO_SEQ_EN;
static float   s_fTsConvXPx     = TARGET_SERVO_CONV_X_PX;
static float   s_fTsConvYPx     = TARGET_SERVO_CONV_Y_PX;
static int32_t s_iTsStallTicks  = (int32_t)TARGET_SERVO_STALL_TICKS;
static float   s_fTsKickDeg     = TARGET_SERVO_KICK_DEG;
static int32_t s_iTsKickMax     = (int32_t)TARGET_SERVO_KICK_MAX;
static uint8_t s_ucTsDistEn     = TARGET_SERVO_DIST_EN;
static float   s_fTsDistPxToMm  = TARGET_SERVO_DIST_PX_TO_MM;
static float   s_fTsDistMm      = TARGET_SERVO_DIST_MM;

void     vSetTsCtrlMs(int32_t iVal)    { s_iTsCtrlMs = iVal; }
int32_t  iGetTsCtrlMs(void)            { return s_iTsCtrlMs; }
void     vSetTsMoveMs(int32_t iVal)    { s_iTsMoveMs = iVal; }
int32_t  iGetTsMoveMs(void)            { return s_iTsMoveMs; }
void     vSetTsStepDeg(float fVal)     { s_fTsStepDeg = fVal; }
float    fGetTsStepDeg(void)           { return s_fTsStepDeg; }
void     vSetTsConvPx(float fVal)      { s_fTsConvPx = fVal; }
float    fGetTsConvPx(void)            { return s_fTsConvPx; }
void     vSetTsConvTicks(int32_t iVal) { s_iTsConvTicks = iVal; }
int32_t  iGetTsConvTicks(void)         { return s_iTsConvTicks; }
void     vSetTsTravelDeg(float fVal)   { s_fTsTravelDeg = fVal; }
float    fGetTsTravelDeg(void)         { return s_fTsTravelDeg; }
void     vSetTsStaleMs(int32_t iVal)   { s_iTsStaleMs = iVal; }
int32_t  iGetTsStaleMs(void)           { return s_iTsStaleMs; }
void     vSetTsKpScale(float fVal)     { s_fTsKpScale = fVal; }
float    fGetTsKpScale(void)           { return s_fTsKpScale; }
void     vSetTsChassisEn(uint8_t ucVal) { s_ucTsChassisEn = ucVal; }
uint8_t  ucGetTsChassisEn(void)         { return s_ucTsChassisEn; }
void     vSetTsChassisPx(float fVal)    { s_fTsChassisPx = fVal; }
float    fGetTsChassisPx(void)          { return s_fTsChassisPx; }
void     vSetTsChassisPxToMm(float fVal){ s_fTsChassisPxToMm = fVal; }
float    fGetTsChassisPxToMm(void)      { return s_fTsChassisPxToMm; }
void     vSetTsChassisSign(float fVal)  { s_fTsChassisSign = fVal; }
float    fGetTsChassisSign(void)        { return s_fTsChassisSign; }
void     vSetTsChassisMm(float fVal)    { s_fTsChassisMm = fVal; }
float    fGetTsChassisMm(void)          { return s_fTsChassisMm; }

void     vSetTsSeqEn(uint8_t ucVal)     { s_ucTsSeqEn = ucVal; }
uint8_t  ucGetTsSeqEn(void)             { return s_ucTsSeqEn; }
void     vSetTsConvXPx(float fVal)      { s_fTsConvXPx = fVal; }
float    fGetTsConvXPx(void)            { return s_fTsConvXPx; }
void     vSetTsConvYPx(float fVal)      { s_fTsConvYPx = fVal; }
float    fGetTsConvYPx(void)            { return s_fTsConvYPx; }
void     vSetTsStallTicks(int32_t iVal) { s_iTsStallTicks = iVal; }
int32_t  iGetTsStallTicks(void)         { return s_iTsStallTicks; }
void     vSetTsKickDeg(float fVal)      { s_fTsKickDeg = fVal; }
float    fGetTsKickDeg(void)            { return s_fTsKickDeg; }
void     vSetTsKickMax(int32_t iVal)    { s_iTsKickMax = iVal; }
int32_t  iGetTsKickMax(void)            { return s_iTsKickMax; }
void     vSetTsDistEn(uint8_t ucVal)    { s_ucTsDistEn = ucVal; }
uint8_t  ucGetTsDistEn(void)            { return s_ucTsDistEn; }
void     vSetTsDistPxToMm(float fVal)   { s_fTsDistPxToMm = fVal; }
float    fGetTsDistPxToMm(void)         { return s_fTsDistPxToMm; }
void     vSetTsDistMm(float fVal)       { s_fTsDistMm = fVal; }
float    fGetTsDistMm(void)             { return s_fTsDistMm; }

/* ----- 平移射靶标定量（激光固定在底盘, 2026-10-07 加; 见 app_beam.h 的契约说明） -----
 * 沿靶排**纵移**对齐（cx 偏差驱动 v_y, swap=1 本车）: 瞄准常数 + 自己的比例尺/方向/死区
 * （初值 = ts_chassis 那对"cx→纵移"现场标定值, 刻意独立 —— 见 app_config.h 段头）;
 * 行走加减速复用 ta_accel/ta_speed, 帧新鲜度复用 ts_stale_ms。
 * ⚠️ 注释里别把两个宏前缀写成「ts_ 星杠 tg_」的通配样式 —— 星杠是注释结束符, 提前闭合。 */
static float    s_fTcCxAim     = TARGET_SHOOT_CX_AIM;
static float    s_fTcCyAim     = TARGET_SHOOT_CY_AIM;
static float    s_fTcGapMm     = TARGET_SHOOT_GAP_MM;
static float    s_fTcGapPx     = TARGET_SHOOT_GAP_PX;
static float    s_fTcPxToMm    = TARGET_SHOOT_PX_TO_MM;
static float    s_fTcSign      = TARGET_SHOOT_SIGN;
static float    s_fTcDbMm      = TARGET_SHOOT_DB_MM;
static int32_t  s_iTcAccel     = TARGET_SHOOT_ACCEL;
static int32_t  s_iTcSpeed     = TARGET_SHOOT_SPEED;
static uint8_t  s_ucTcVelMode  = TARGET_SHOOT_VEL_MODE;
static float    s_fTcMaxMm     = TARGET_SHOOT_MAX_MM;
static float    s_fTcConvPx    = TARGET_SHOOT_CONV_PX;
static int32_t  s_iTcConvTicks = (int32_t)TARGET_SHOOT_CONV_TICKS;
static float    s_fTcKpScale   = TARGET_SHOOT_KP_SCALE;
static float    s_fTcStepMm    = TARGET_SHOOT_STEP_MM;
static float    s_fTcTravelMm  = TARGET_SHOOT_TRAVEL_MM;
static int32_t  s_iTcCtrlMs    = (int32_t)TARGET_SHOOT_CTRL_MS;

/* pi(树莓派 CSI)摄像头那一套（2026-10-16 加, 见 app_beam.h 与 app_config.h 段头）。
 * 只分瞄准列/行这一对: CSI 与主摄**相对位置不同**(主摄在机械臂上、CSI 固定在底盘), 理想
 * 停止位下目标该停的像素列不一样。其余比例尺/方向/门限暂时共用。 */
static float    s_fTcCxAimPi   = TARGET_SHOOT_CX_AIM_PI;
static float    s_fTcCyAimPi   = TARGET_SHOOT_CY_AIM_PI;
static uint8_t  s_ucTcUsePicam = TARGET_SHOOT_USE_PICAM;

/* ----- 「射靶保持」运行期遥测（2026-10-09 加, 给 OLED 射靶测试页与蓝牙 Tab 看）-----
 * 唯一写者 = `ucRpiTargetShootHold()` 的保持环（app_rpi.c）, 唯一读者 = OLED 标签 + 遥测。
 * ⚠️ 都**不是**标定量: 断电即失效, 也不进 s_astBeamParam 的可写列（pfSet 传 NULL）。 */
static float    s_fShootErr       = 0.0f;   /* 本拍沿排偏差[px]（>0 = 该往 tc_sign 正向走） */
static uint8_t  s_ucShootState    = 0u;     /* 0空闲 1粗调 2运动中 3稳住 4冻结 */
static uint32_t s_ulShootSettleTick = 0u;   /* 最近一次"运动中→稳住"的 xTaskGetTickCount */
static uint8_t  s_ucShootCam       = 0u;    /* 本步实际在用哪台相机(0=主摄/1=pi CSI) */

void     vSetTcCxAim(float fVal)      { s_fTcCxAim = fVal; }
float    fGetTcCxAim(void)            { return s_fTcCxAim; }
void     vSetTcCyAim(float fVal)      { s_fTcCyAim = fVal; }
float    fGetTcCyAim(void)            { return s_fTcCyAim; }
void     vSetTcGapMm(float fVal)      { s_fTcGapMm = fVal; }
float    fGetTcGapMm(void)            { return s_fTcGapMm; }
void     vSetTcGapPx(float fVal)      { s_fTcGapPx = fVal; }
float    fGetTcGapPx(void)            { return s_fTcGapPx; }
void     vSetTcPxToMm(float fVal)     { s_fTcPxToMm = fVal; }
float    fGetTcPxToMm(void)           { return s_fTcPxToMm; }
void     vSetTcSign(float fVal)       { s_fTcSign = fVal; }
float    fGetTcSign(void)             { return s_fTcSign; }
void     vSetTcDbMm(float fVal)       { s_fTcDbMm = fVal; }
float    fGetTcDbMm(void)             { return s_fTcDbMm; }
void     vSetTcAccel(int32_t iVal)    { s_iTcAccel = iVal; }
int32_t  iGetTcAccel(void)            { return s_iTcAccel; }
void     vSetTcSpeed(int32_t iVal)    { s_iTcSpeed = iVal; }
int32_t  iGetTcSpeed(void)            { return s_iTcSpeed; }
void     vSetTcVelMode(uint8_t ucVal) { s_ucTcVelMode = ucVal; }
uint8_t  ucGetTcVelMode(void)         { return s_ucTcVelMode; }

/* ----- 「射靶保持」运行期遥测（见上面变量处的说明）----- */
void     vSetShootErr(float fVal)          { s_fShootErr = fVal; }
float    fGetShootErr(void)                { return s_fShootErr; }
void     vSetShootState(uint8_t ucVal)     { s_ucShootState = ucVal; }
uint8_t  ucGetShootState(void)             { return s_ucShootState; }
void     vSetShootSettleTick(uint32_t ulVal) { s_ulShootSettleTick = ulVal; }
uint32_t ulGetShootSettleTick(void)        { return s_ulShootSettleTick; }
void     vSetShootCam(uint8_t ucVal)       { s_ucShootCam = ucVal; }
uint8_t  ucGetShootCam(void)               { return s_ucShootCam; }
void     vSetTcMaxMm(float fVal)      { s_fTcMaxMm = fVal; }
float    fGetTcMaxMm(void)            { return s_fTcMaxMm; }
void     vSetTcConvPx(float fVal)     { s_fTcConvPx = fVal; }
float    fGetTcConvPx(void)           { return s_fTcConvPx; }
void     vSetTcConvTicks(int32_t iVal) { s_iTcConvTicks = iVal; }
int32_t  iGetTcConvTicks(void)        { return s_iTcConvTicks; }
void     vSetTcKpScale(float fVal)    { s_fTcKpScale = fVal; }
float    fGetTcKpScale(void)          { return s_fTcKpScale; }
void     vSetTcStepMm(float fVal)     { s_fTcStepMm = fVal; }
float    fGetTcStepMm(void)           { return s_fTcStepMm; }
void     vSetTcTravelMm(float fVal)   { s_fTcTravelMm = fVal; }
float    fGetTcTravelMm(void)         { return s_fTcTravelMm; }
void     vSetTcCtrlMs(int32_t iVal)   { s_iTcCtrlMs = iVal; }
int32_t  iGetTcCtrlMs(void)           { return s_iTcCtrlMs; }
void     vSetTcCxAimPi(float fVal)    { s_fTcCxAimPi = fVal; }
float    fGetTcCxAimPi(void)          { return s_fTcCxAimPi; }
void     vSetTcCyAimPi(float fVal)    { s_fTcCyAimPi = fVal; }
float    fGetTcCyAimPi(void)          { return s_fTcCyAimPi; }
void     vSetTcUsePicam(uint8_t ucVal) { s_ucTcUsePicam = ucVal; }
uint8_t  ucGetTcUsePicam(void)        { return s_ucTcUsePicam; }

/* ----- 爆爆桶对位标定量（2026-10-24 加; 见 app_config.h「爆爆桶对位」段头） -----
 * 闭环 = ucTargetShootCore 精调环的桶版（app_rpi.c::ucRpiBarrelAlign）: 瞄准常数 +
 * 自己的比例尺/方向/死区/节拍（**刻意独立一套** —— 调它不连带改坏射靶, 同 tc 不共用
 * ts_chassis 的理由）; 坐标系是 OpenMV **320×240**, 与 tc/tg 那些全不是一套, 别抄值。 */
static float    s_fBcCxAim     = BUCKET_ALIGN_CX_AIM;
static float    s_fBcCyAim     = BUCKET_ALIGN_CY_AIM;
static uint8_t  s_ucBcSwap     = BUCKET_ALIGN_SWAP;
static float    s_fBcPxToMm    = BUCKET_ALIGN_PX_TO_MM;
static float    s_fBcSign      = BUCKET_ALIGN_SIGN;
static float    s_fBcDbMm      = BUCKET_ALIGN_DB_MM;
static int32_t  s_iBcAccel     = BUCKET_ALIGN_ACCEL;
static int32_t  s_iBcSpeed     = BUCKET_ALIGN_SPEED;
static uint8_t  s_ucBcVelMode  = BUCKET_ALIGN_VEL_MODE;
static float    s_fBcConvPx    = BUCKET_ALIGN_CONV_PX;
static int32_t  s_iBcConvTicks = (int32_t)BUCKET_ALIGN_CONV_TICKS;
static float    s_fBcKpScale   = BUCKET_ALIGN_KP_SCALE;
static float    s_fBcStepMm    = BUCKET_ALIGN_STEP_MM;
static float    s_fBcTravelMm  = BUCKET_ALIGN_TRAVEL_MM;
static int32_t  s_iBcCtrlMs    = (int32_t)BUCKET_ALIGN_CTRL_MS;
static int32_t  s_iBcStaleMs   = (int32_t)BUCKET_ALIGN_STALE_MS;
static int32_t  s_iBcMeasMs    = (int32_t)BUCKET_ALIGN_MEAS_MS;

/* 「爆爆桶对位」运行期遥测（只读; 唯一写者 = ucRpiBarrelAlign 的只测/闭环环）——
 * 蓝牙 Tab 直接看偏差, 标定 bc_cx_aim 不必翻日志。 */
static float    s_fBucketErr   = 0.0f;   /* 本拍 cx 偏差[px]（>0 = 桶在瞄准列右侧） */

void     vSetBcCxAim(float fVal)      { s_fBcCxAim = fVal; }
float    fGetBcCxAim(void)            { return s_fBcCxAim; }
void     vSetBcCyAim(float fVal)      { s_fBcCyAim = fVal; }
float    fGetBcCyAim(void)            { return s_fBcCyAim; }
void     vSetBcSwap(uint8_t ucVal)    { s_ucBcSwap = ucVal; }
uint8_t  ucGetBcSwap(void)            { return s_ucBcSwap; }
void     vSetBcPxToMm(float fVal)     { s_fBcPxToMm = fVal; }
float    fGetBcPxToMm(void)           { return s_fBcPxToMm; }
void     vSetBcSign(float fVal)       { s_fBcSign = fVal; }
float    fGetBcSign(void)             { return s_fBcSign; }
void     vSetBcDbMm(float fVal)       { s_fBcDbMm = fVal; }
float    fGetBcDbMm(void)             { return s_fBcDbMm; }
void     vSetBcAccel(int32_t iVal)    { s_iBcAccel = iVal; }
int32_t  iGetBcAccel(void)            { return s_iBcAccel; }
void     vSetBcSpeed(int32_t iVal)    { s_iBcSpeed = iVal; }
int32_t  iGetBcSpeed(void)            { return s_iBcSpeed; }
void     vSetBcVelMode(uint8_t ucVal) { s_ucBcVelMode = ucVal; }
uint8_t  ucGetBcVelMode(void)         { return s_ucBcVelMode; }
void     vSetBcConvPx(float fVal)     { s_fBcConvPx = fVal; }
float    fGetBcConvPx(void)           { return s_fBcConvPx; }
void     vSetBcConvTicks(int32_t iVal) { s_iBcConvTicks = iVal; }
int32_t  iGetBcConvTicks(void)        { return s_iBcConvTicks; }
void     vSetBcKpScale(float fVal)    { s_fBcKpScale = fVal; }
float    fGetBcKpScale(void)          { return s_fBcKpScale; }
void     vSetBcStepMm(float fVal)     { s_fBcStepMm = fVal; }
float    fGetBcStepMm(void)           { return s_fBcStepMm; }
void     vSetBcTravelMm(float fVal)   { s_fBcTravelMm = fVal; }
float    fGetBcTravelMm(void)         { return s_fBcTravelMm; }
void     vSetBcCtrlMs(int32_t iVal)   { s_iBcCtrlMs = iVal; }
int32_t  iGetBcCtrlMs(void)           { return s_iBcCtrlMs; }
void     vSetBcStaleMs(int32_t iVal)  { s_iBcStaleMs = iVal; }
int32_t  iGetBcStaleMs(void)          { return s_iBcStaleMs; }
void     vSetBcMeasMs(int32_t iVal)   { s_iBcMeasMs = iVal; }
int32_t  iGetBcMeasMs(void)           { return s_iBcMeasMs; }
void     vSetBucketErr(float fVal)    { s_fBucketErr = fVal; }
float    fGetBucketErr(void)          { return s_fBucketErr; }

/* ----- 靶标识别结果（反恐区打靶；同帧多字强一致 → 临界区, 同色球快照） ----- */
static uint8_t s_ucTargetStatus = 0;     /* 1=成功 0=失败/超时（**只收 bit0**） */
static uint8_t s_ucTargetCount  = 0;
static stTargetTgtTdf s_astTargetTgt[TARGET_MAX_COUNT];
static uint8_t s_ucTargetReady  = 0;
static uint8_t s_ucTargetCam    = 0u;    /* 这一帧实际哪台相机测的(0=主摄/1=pi CSI) */

void vSetTargetCam(uint8_t ucVal) { s_ucTargetCam = (ucVal != 0u) ? 1u : 0u; }
uint8_t ucGetTargetCam(void)      { return s_ucTargetCam; }

static uint8_t s_ucTargetPullDir = 0u;   /* 0x04 失败帧方向字节(2026-10-24): 0=无信息
                                          * 1=目标色画面右侧出界 2=左侧; 写方=vRpiTask
                                          * (同帧先于 vSetTargetResult), 读方=ucTargetShootCore
                                          * 的补步（读后即清, 同 BallPullDir 的纪律）。 */
void     vSetTargetPullDir(uint8_t ucVal){ s_ucTargetPullDir = ucVal; }
uint8_t  ucGetTargetPullDir(void)        { return s_ucTargetPullDir; }

void vSetTargetResult(uint8_t ucStatus, uint8_t ucCount, const stTargetTgtTdf *pstTgt)
{
    uint8_t ucI;

    if (pstTgt == NULL) { ucCount = 0; }
    if (ucCount > TARGET_MAX_COUNT) { ucCount = TARGET_MAX_COUNT; }
    taskENTER_CRITICAL();
    s_ucTargetStatus = ucStatus;
    s_ucTargetCount  = ucCount;
    for (ucI = 0; ucI < ucCount; ucI++)
    {
        s_astTargetTgt[ucI] = pstTgt[ucI];
    }
    taskEXIT_CRITICAL();
}

void vGetTargetResult(uint8_t *pucStatus, uint8_t *pucCount, stTargetTgtTdf *pstTgt)
{
    uint8_t ucI;

    taskENTER_CRITICAL();
    if (pucStatus != NULL) { *pucStatus = s_ucTargetStatus; }
    if (pucCount  != NULL) { *pucCount  = s_ucTargetCount; }
    if (pstTgt    != NULL)
    {
        for (ucI = 0; ucI < s_ucTargetCount && ucI < TARGET_MAX_COUNT; ucI++)
        {
            pstTgt[ucI] = s_astTargetTgt[ucI];
        }
    }
    taskEXIT_CRITICAL();
}

uint8_t ucTakeTargetReady(void)
{
    uint8_t uc;
    taskENTER_CRITICAL();
    uc = s_ucTargetReady;
    s_ucTargetReady = 0;   /* 读后自动清零 */
    taskEXIT_CRITICAL();
    return uc;
}

void vSetTargetReady(void)
{
    s_ucTargetReady = 1;
}

/* ----- 靶标流帧快照（反恐区打靶, TAG=0x05, ~20Hz; 同帧多字强一致 → 临界区）-----
 * ⚠️ 与上面的 0x04 快照刻意**分开**（理由见 app_beam.h 那段）: 闭环跑完停流时留下的
 *    最后一帧不能污染随后的终验。
 * ⚠️ **没有 ready 一次性信号**, 只有帧计数 —— 流的语义是"计数还在不在涨",
 *    设 ready 会让读它的一方把信号偷走（同巡线帧的做法, 见 vSetLineData）。 */
/* ⚠️ 容量用 TARGET_MAX_COUNT（本头文件里那个）而不是 app_rpi.h 的 RPI_AIM_MAX_CNT ——
 *    两者都是 3 且都写明了"双端同步改", 但 Beam 不该反向依赖帧模块
 *    （app_beam.c 不 include app_rpi.h）。存储大小按这里来, 帧侧解析时自会钳到
 *    RPI_AIM_MAX_CNT, 两者必须一起改。 */
static uint8_t s_ucAimCount    = 0;
static stTargetTgtTdf s_astAimTgt[TARGET_MAX_COUNT];
static uint32_t s_ulAimFrameCnt = 0;
static uint8_t s_ucAimCam      = 0u;    /* 这一帧实际哪台相机测的(0=主摄/1=pi CSI) */

void vSetAimFrame(uint8_t ucCount, const stTargetTgtTdf *pstTgt, uint8_t ucCam)
{
    uint8_t ucI;

    if (pstTgt == NULL) { ucCount = 0; }
    if (ucCount > TARGET_MAX_COUNT) { ucCount = TARGET_MAX_COUNT; }
    taskENTER_CRITICAL();
    s_ucAimCount = ucCount;
    s_ucAimCam   = (ucCam != 0u) ? 1u : 0u;
    for (ucI = 0; ucI < ucCount; ucI++)
    {
        s_astAimTgt[ucI] = pstTgt[ucI];
    }
    s_ulAimFrameCnt++;      /* ⚠️ count=0（丢目标）也是合法帧, 计数照常推进 */
    taskEXIT_CRITICAL();
}

void vGetAimFrame(uint8_t *pucCount, stTargetTgtTdf *pstTgt, uint8_t *pucCam)
{
    uint8_t ucI;

    taskENTER_CRITICAL();
    if (pucCount != NULL) { *pucCount = s_ucAimCount; }
    if (pucCam   != NULL) { *pucCam   = s_ucAimCam; }
    if (pstTgt != NULL)
    {
        for (ucI = 0; ucI < s_ucAimCount && ucI < TARGET_MAX_COUNT; ucI++)
        {
            pstTgt[ucI] = s_astAimTgt[ucI];
        }
    }
    taskEXIT_CRITICAL();
}

uint32_t ulGetAimFrameCount(void)
{
    return s_ulAimFrameCnt;
}

/* ----- 爆爆桶流帧快照（TAG=0x0B, ~20Hz, 2026-10-24 加; 同帧多字强一致 → 临界区）-----
 * ⚠️ 与上面的 0x05 快照**分开**（同 vSetAimFrame 与 0x04 分开的理由）。
 * ⚠️ 同样**只帧计数、没有 ready 一次性信号**（流的语义 = "计数还在不在涨"）。
 * 单桶定长帧（相机端"挑好再报"）, 不需要数组。 */
static uint8_t  s_ucBucketValid    = 0;
static uint16_t s_usBucketCx       = 0;
static uint16_t s_usBucketCy       = 0;
static uint16_t s_usBucketW        = 0;
static uint16_t s_usBucketH        = 0;
static uint32_t s_ulBucketFrameCnt = 0;

void vSetBucketFrame(uint8_t ucValid, uint16_t usCx, uint16_t usCy,
                     uint16_t usW, uint16_t usH)
{
    taskENTER_CRITICAL();
    s_ucBucketValid = (ucValid != 0u) ? 1u : 0u;
    s_usBucketCx    = usCx;
    s_usBucketCy    = usCy;
    s_usBucketW     = usW;
    s_usBucketH     = usH;
    s_ulBucketFrameCnt++;      /* 无桶帧也是合法帧, 计数照常推进（新鲜度语义） */
    taskEXIT_CRITICAL();
}

void vGetBucketFrame(uint8_t *pucValid, uint16_t *pusCx, uint16_t *pusCy,
                     uint16_t *pusW, uint16_t *pusH)
{
    taskENTER_CRITICAL();
    if (pucValid != NULL) { *pucValid = s_ucBucketValid; }
    if (pusCx  != NULL) { *pusCx  = s_usBucketCx; }
    if (pusCy  != NULL) { *pusCy  = s_usBucketCy; }
    if (pusW   != NULL) { *pusW   = s_usBucketW; }
    if (pusH   != NULL) { *pusH   = s_usBucketH; }
    taskEXIT_CRITICAL();
}

uint32_t ulGetBucketFrameCount(void)
{
    return s_ulBucketFrameCnt;
}

/* ----- RPI 链路状态 ----- */
static emRpiLinkStateTdf s_emRpiLinkState = RPI_LINK_WAITING;

emRpiLinkStateTdf emGetRpiLinkState(void)
{
    return s_emRpiLinkState;
}

void vSetRpiLinkState(emRpiLinkStateTdf emVal)
{
    s_emRpiLinkState = emVal;
}

/* ==================== 蓝牙机械臂编辑模式 ==================== */
static uint8_t  s_ucBtHsWait   = 0;   /* 脚本正停在握手阶段 1（写者: 脚本任务） */
static uint8_t  s_ucBtEditMode = 0;   /* PC 请求的编辑模式（写 1 者: vBtTask） */
static uint32_t s_ulBtCmdTick  = 0;   /* 上一条有效指令的 tick（写者: vBtTask） */

uint8_t ucGetBtHsWait(void)
{
    return s_ucBtHsWait;
}

void vSetBtHsWait(uint8_t ucVal)
{
    s_ucBtHsWait = ucVal;
}

/*
 * bt_edit_mode 的两个写者都只写 0：
 *   vBtTask   收 $B,MODE,0 → 写 0
 *   vBtEditPump 见到握手成功 / 看门狗超时 → 写 0
 * 这是**有意的良性竞态**，不要"修"成单一写者：单字节读写原子，泵本身在循环里
 * 每轮重新判定，丢一次更新下一轮自愈；写 1 的始终只有 vBtTask 一家。
 */
uint8_t ucGetBtEditMode(void)
{
    return s_ucBtEditMode;
}

void vSetBtEditMode(uint8_t ucVal)
{
    s_ucBtEditMode = ucVal;
}

uint32_t ulGetBtCmdTick(void)
{
    return s_ulBtCmdTick;
}

void vSetBtCmdTick(uint32_t ulVal)
{
    s_ulBtCmdTick = ulVal;
}

/* ==================== 机械臂后台序列 ==================== */
/* 「有后台序列在跑」——**只有 vArmTask 的泵（vArmActionPump）一个写者**：
 * 它认领一段序列时写 1、走完（或代次变了交给接手者）时写 0。
 * vArmSeqAbort 刻意**不碰**这个标志: 中止后当前那一拍仍在动舵机, 报 busy 才诚实,
 * 泵退出时自然会清。别为了「让中止立刻显示空闲」在这里加第二个写者。 */
static uint8_t  s_ucArmSeqBusy   = 0;   /* 写者: app_arm_action.c::vArmActionPump */

uint8_t ucGetArmSeqBusy(void)
{
    return s_ucArmSeqBusy;
}

void vSetArmSeqBusy(uint8_t ucVal)
{
    s_ucArmSeqBusy = ucVal;
}

/* ==================== 脚本跳转遥测（只读） ==================== */
/* 有了 IF/跳转之后, 「车现在跑到哪张表了」是现场排障第一个要问的问题 —— 这两个量把
 * 脚本帧栈的状态如实报出来, 「蓝牙」Tab 的参数面板里直接能看到（不用改控制台）。
 * **唯一写者** = 脚本任务（app_script.c::vScriptRun 在进场/压栈/弹栈时写）。
 * 读者 = 蓝牙 / OLED。单字节读写在 Cortex-M4 上原子, 按 Beam 规则第 4 条不加锁。 */
static uint8_t s_ucScriptPresetId  = 0;   /* 当前帧所在的预设 id; 0 = 未知/顶层预设不在注册表 */
static uint8_t s_ucScriptCallDepth = 0;   /* 跳转深度: 0 = 顶层路线, 1+ = 被跳进来的子预设 */

uint8_t ucGetScriptPresetId(void)
{
    return s_ucScriptPresetId;
}

void vSetScriptPresetId(uint8_t ucVal)
{
    s_ucScriptPresetId = ucVal;
}

uint8_t ucGetScriptCallDepth(void)
{
    return s_ucScriptCallDepth;
}

void vSetScriptCallDepth(uint8_t ucVal)
{
    s_ucScriptCallDepth = ucVal;
}

/* ==================== 组动作角度偏移（台架标定, 蓝牙可写） ====================
 * 「编辑模式拖着到位」（恒速趋近）与「组动作 S 曲线播放到位」的**物理落点**不一定一致 ——
 * 舵机死区 + 齿轮回差，底座 S6 带整臂惯量最明显。台架上拖准的位姿一进路线就偏几度，
 * 这里给组动作的每个目标角叠加一个常量来把它修回去（标定法子见 app_config.h 该段注释）。
 *
 * 唯一消费者 = app_arm.c::vArmGroupMoveNoWait。⚠️ 刻意**不**给 $B,JOG 与夹爪用:
 * 编辑模式是标定基准，把它也偏移掉就成了「滑杆读数与物理位置互相追着跑」。
 * 索引 = 组顺序 SERVO1,SERVO2,SERVO3,SERVO4,SERVO6（与 ARM_GROUP_NUM 一致）。
 * 单字读写原子, 按 Beam 规则第 4 条不加锁。 */
static float s_afArmGroupOfs[ARM_GROUP_NUM] = {
    ARM_GROUP_OFS_S1, ARM_GROUP_OFS_S2, ARM_GROUP_OFS_S3, ARM_GROUP_OFS_S4, ARM_GROUP_OFS_S6,
};

float fGetArmGroupOfs(uint8_t ucIdx)
{
    return (ucIdx < (uint8_t)ARM_GROUP_NUM) ? s_afArmGroupOfs[ucIdx] : 0.0f;
}

void vSetArmGroupOfs(uint8_t ucIdx, float fVal)
{
    if (ucIdx < (uint8_t)ARM_GROUP_NUM) { s_afArmGroupOfs[ucIdx] = fVal; }
}

/* ==================== 蓝牙可访问参数注册表 ==================== */
/*
 * 为什么要有这张表：控制台「蓝牙」Tab 的参数面板必须**由设备自述**——
 * 参数名/中文名/分组/单位/类型/范围/当前值全部从设备读，控制台侧不写死默认值，
 * 这样宏改了、初值改了，面板自动跟着变（同「地图」Tab 从源码派生的思路）。
 *
 * 三条硬约束：
 *  1. 一律**排除 ucTakeXxx 一次性信号**——读它们会清零, 会打断真正的消费者
 *     （ucTakeMenuConfirm / ucTakeYawFresh / ucTakeRpiFrameReady /
 *       ucTakeHandshakeSeen / ucTakeHostageReady / ucTakeColorReady /
 *       ucTakeChActPreviewReq / ucTakeChActStopReq, 共 8 个, 全部不在表内）。
 *     pucGetLastRpiPacket 返回内部指针, 签名叫不上 float(*)(void), 同样排除。
 *  2. 字段里**禁止出现 ',' 和 '\n'**——$B 线协议没有转义, 一个逗号就把 $B,P 记录串行。
 *  3. 表是**位置初始化**（AC5 V5.06 是 C89, 不支持指定初始化器）, 顺序必须与
 *     stBeamParamTdf 的字段声明一一对应。
 */

/* --- 非 float 类型的读写适配 thunk ---
 * AC5 不会把 int32_t(*)(void) 隐式转成 float(*)(void)（告警 #167-D）, 故逐个生成。
 * 写侧先取整再交给原 setter; 范围钳位由 ucBeamApply 在调用前完成, thunk 只管转换。 */
static int32_t beam_i32Round(float fVal)
{
    return (fVal >= 0.0f) ? (int32_t)(fVal + 0.5f) : (int32_t)(fVal - 0.5f);
}

#define BEAM_ADAPT_I32(stem)                                            \
    static float _bg_##stem(void) { return (float)iGet##stem(); }        \
    static void  _bs_##stem(float fVal) { vSet##stem(beam_i32Round(fVal)); }

#define BEAM_ADAPT_U8(stem)                                             \
    static float _bg_##stem(void) { return (float)ucGet##stem(); }       \
    static void  _bs_##stem(float fVal) { vSet##stem((uint8_t)fVal); }

#define BEAM_ADAPT_I8(stem)                                             \
    static float _bg_##stem(void) { return (float)cGet##stem(); }        \
    static void  _bs_##stem(float fVal) { vSet##stem((int8_t)fVal); }

/* 只读遥测用: 表里 pfSet 传 NULL, 生成 setter 只会换来一句
 * #177-D "declared but never referenced"。 */
#define BEAM_ADAPT_RO_U8(stem)                                          \
    static float _bg_##stem(void) { return (float)ucGet##stem(); }

#define BEAM_ADAPT_RO_I32(stem)                                         \
    static float _bg_##stem(void) { return (float)iGet##stem(); }

BEAM_ADAPT_I32(ChassisAccel)
BEAM_ADAPT_I32(MenuIndex)
BEAM_ADAPT_I32(AttPage)
BEAM_ADAPT_U8(GyroAutoSwitch)     /* 陀螺热备开关（可读可写） */
BEAM_ADAPT_RO_U8(GyroSrcEff)      /* 有效源（只读遥测） */
BEAM_ADAPT_RO_I32(GyroFramesMain) /* 主口帧号（只读遥测） */
BEAM_ADAPT_RO_I32(GyroFramesSub)  /* 副口帧号（只读遥测） */
BEAM_ADAPT_RO_I32(GyroAutoSw)     /* 自动切换次数（只读遥测） */
BEAM_ADAPT_U8(OledMenuAnim)       /* OLED 菜单换行动画开关（可读可写, 0~1 → 前端勾选框） */
BEAM_ADAPT_U8(SehuanStreaming)
BEAM_ADAPT_I8(SehuanYDiff)
BEAM_ADAPT_U8(BallYawEn)
BEAM_ADAPT_U8(BallXySwap)
BEAM_ADAPT_I32(BallAlignAccel)
BEAM_ADAPT_I32(BallAlignSpeed)
BEAM_ADAPT_U8(HsXySwap)
BEAM_ADAPT_I32(HsAlignAccel)
BEAM_ADAPT_I32(HsAlignSpeed)
BEAM_ADAPT_U8(OneLatDir)
BEAM_ADAPT_I32(OneLineMs)
BEAM_ADAPT_U8(ArmCurve)
BEAM_ADAPT_I32(TgMoveMs)
BEAM_ADAPT_I32(TgMoveSlackMs)
BEAM_ADAPT_U8(TgXySwap)
BEAM_ADAPT_I32(TgCalibTimeoutMs)
BEAM_ADAPT_U8(TaXySwap)
BEAM_ADAPT_I32(TaAlignAccel)
BEAM_ADAPT_I32(TaAlignSpeed)
BEAM_ADAPT_I32(TsCtrlMs)
BEAM_ADAPT_I32(TsMoveMs)
BEAM_ADAPT_I32(TsConvTicks)
BEAM_ADAPT_I32(TsStaleMs)
BEAM_ADAPT_U8(TsChassisEn)
BEAM_ADAPT_U8(TsSeqEn)
BEAM_ADAPT_I32(TsStallTicks)
BEAM_ADAPT_I32(TsKickMax)
BEAM_ADAPT_U8(TsDistEn)
BEAM_ADAPT_I32(TcConvTicks)
BEAM_ADAPT_I32(TcCtrlMs)
BEAM_ADAPT_I32(TcAccel)
BEAM_ADAPT_I32(TcSpeed)
BEAM_ADAPT_U8(TcVelMode)
BEAM_ADAPT_U8(TcUsePicam)       /* 优先 pi(CSI) 摄像头（2026-10-16 加） */
BEAM_ADAPT_U8(BcSwap)           /* 爆爆桶轴映射 1=cx→纵移（2026-10-24 加） */
BEAM_ADAPT_I32(BcAccel)
BEAM_ADAPT_I32(BcSpeed)
BEAM_ADAPT_U8(BcVelMode)
BEAM_ADAPT_I32(BcConvTicks)
BEAM_ADAPT_I32(BcCtrlMs)
BEAM_ADAPT_I32(BcStaleMs)
BEAM_ADAPT_I32(BcMeasMs)
BEAM_ADAPT_RO_U8(ShootCam)      /* 射靶在用哪台相机 0主摄/1pi（只读遥测） */
BEAM_ADAPT_U8(GyroSrc)          /* M0 副陀螺选源（2026-10 加） */
BEAM_ADAPT_RO_U8(QrManual)
BEAM_ADAPT_RO_U8(PhotoResult)
BEAM_ADAPT_RO_U8(ArmDeployed)
BEAM_ADAPT_RO_U8(ArmSeqBusy)
BEAM_ADAPT_RO_U8(ScriptRunning)
BEAM_ADAPT_RO_U8(BallFitOk)
BEAM_ADAPT_RO_U8(AlignOk)
BEAM_ADAPT_RO_U8(AlignMethod)
BEAM_ADAPT_RO_U8(ScriptPresetId)
BEAM_ADAPT_RO_U8(ScriptCallDepth)
BEAM_ADAPT_RO_U8(RectStatus)
BEAM_ADAPT_RO_U8(ShootState)      /* 射靶保持状态 0空闲/1粗调/2运动中/3稳住/4冻结（只读遥测） */
BEAM_ADAPT_I32(RoadAccel)
BEAM_ADAPT_I32(RoadSpeed)
/* 底盘动作组: 增益四个量直接用 float getter/setter（表里直引, 无需 thunk）;
 * busy/playing 是只读遥测（唯一写者 = 播放器, 见变量区注释）。 */
BEAM_ADAPT_RO_U8(ChActBusy)
BEAM_ADAPT_RO_U8(ChActPlayingId)
/* 任务补光（2026-10 加, 12 个 U8 量: 总开关/默认亮度 + 五任务各 EN/PCT） */
BEAM_ADAPT_U8(FillTaskEn)
BEAM_ADAPT_U8(FillTaskPct)
BEAM_ADAPT_U8(FillTaskRoadEn)
BEAM_ADAPT_U8(FillTaskRoadPct)
BEAM_ADAPT_U8(FillTaskBallEn)
BEAM_ADAPT_U8(FillTaskBallPct)
BEAM_ADAPT_U8(FillTaskShootEn)
BEAM_ADAPT_U8(FillTaskShootPct)
BEAM_ADAPT_U8(FillTaskHsEn)
BEAM_ADAPT_U8(FillTaskHsPct)
BEAM_ADAPT_U8(FillTaskQrEn)
BEAM_ADAPT_U8(FillTaskQrPct)

/* 组动作偏移是**数组**, 而注册表每条要一对 float(*)(void)/void(*)(float) —— 按索引生成。 */
#define BEAM_ADAPT_ARM_OFS(n)                                                     \
    static float _bg_ArmOfs##n(void) { return fGetArmGroupOfs((uint8_t)(n)); }     \
    static void  _bs_ArmOfs##n(float fVal) { vSetArmGroupOfs((uint8_t)(n), fVal); }

BEAM_ADAPT_ARM_OFS(0)   /* s1 */
BEAM_ADAPT_ARM_OFS(1)   /* s2 */
BEAM_ADAPT_ARM_OFS(2)   /* s3 */
BEAM_ADAPT_ARM_OFS(3)   /* s4 */
BEAM_ADAPT_ARM_OFS(4)   /* s6 */

/* --- 只读遥测的读适配（float 型只读项直接指向已有 getter, 无需 thunk） --- */

/* 看路遥测（只读）: 标 road_dx_ideal / road_dy_ideal / 现场排障时看这几个。
 * ⚠️ road_dx / road_dy 与 Pi 日志里那行 `Δx=` / `Δy=` 应当一致（差 < 1px, 整数像素化的
 *    固有误差）—— 两边对得上才说明 0x08 帧解析没串位（dy 尤其要看: 它用的是原先那个
 *    恒 0 的保留字段, 串位了会读出一个看着"合理"的错数）。 */
static float _bg_RoadStatus(void) { uint8_t v = 0u; vGetRoadResult(&v, NULL, NULL, NULL, NULL, NULL, NULL); return (float)v; }
static float _bg_RoadEx(void)     { int16_t v = 0;  vGetRoadResult(NULL, NULL, &v, NULL, NULL, NULL, NULL); return (float)v; }
static float _bg_RoadEy(void)     { int16_t v = 0;  vGetRoadResult(NULL, NULL, NULL, &v, NULL, NULL, NULL); return (float)v; }
static float _bg_RoadFlags(void)  { uint8_t v = 0u; vGetRoadResult(NULL, NULL, NULL, NULL, &v, NULL, NULL); return (float)v; }
static float _bg_RoadSpread(void) { uint8_t v = 0u; vGetRoadResult(NULL, NULL, NULL, NULL, NULL, &v, NULL); return (float)v; }
static float _bg_RoadDx(void)     { return fGetRoadDxMeas(); }
static float _bg_RoadDy(void)     { return fGetRoadDyMeas(); }
static float _bg_RoadYawStatus(void) { uint8_t v = 0u; vGetRoadYawResult(&v, NULL, NULL, NULL, NULL, NULL); return (float)v; }
static float _bg_RoadYawCols(void)   { uint8_t v = 0u; vGetRoadYawResult(NULL, NULL, &v, NULL, NULL, NULL); return (float)v; }
static float _bg_RoadYawRms(void)    { uint8_t v = 0u; vGetRoadYawResult(NULL, NULL, NULL, &v, NULL, NULL); return (float)v / 10.0f; }
static float _bg_RoadYawSpread(void) { uint8_t v = 0u; vGetRoadYawResult(NULL, NULL, NULL, NULL, &v, NULL); return (float)v / 100.0f; }
static float _bg_RoadYawFlags(void)  { uint8_t v = 0u; vGetRoadYawResult(NULL, NULL, NULL, NULL, NULL, &v); return (float)v; }
static float _bg_RoadYawAng(void) { return fGetRoadYawMeas(); }
static float _bg_RoadYawErr(void) { return fGetRoadYawErr(); }

static float _bg_RoadYawBias(void)   { return fGetRoadYawBias(); }

/* 串口接收健康（app_uart 的 getter 带通道参数, 不能直接用 BEAM_ADAPT_RO_* 宏） */
static float _bg_RpiRxBytes(void) { return (float)ulUartGetRxBytes(UART_RPI); }
static float _bg_RpiRxErr(void)   { return (float)usUartGetRxErr(UART_RPI); }
static float _bg_RpiRxRearmFail(void) { return (float)usUartGetRxRearmFail(UART_RPI); }

/* 串口发送健康（2026-09 发送改 DMA 环后加）。⚠️ 判读规则:
 *   rpi_tx_bytes 涨 **且** 段计数涨 = 真在走 TX DMA; 只是 tx_bytes 涨 = 在走阻塞兜底
 *   （TX DMA 没起来）。段计数（usTxDmaSegs）只在 HAL_UART_TxCpltCallback 里自增, 是
 *   「DMA→TC 中断链完整」的唯一直接证据 —— 它暂时没进 Beam（会再加一行 $B,LIST）,
 *   排障时在调试器里看 g_astUartCh[UART_ID_RPI].usTxDmaSegs 或 usUartGetTxDmaSegs()。 */
static float _bg_RpiTxBytes(void) { return (float)ulUartGetTxBytes(UART_RPI); }

/* 「等链路发任务」遥测（2026-09 加, 见 app_rpi.c::ucRpiSendTaskCmdWait）。
 * 判据: rpi_pend 非 0 且 rpi_link != 1 = 有任务正等重连;
 *       配合 rpi_rx_bytes —— 【冻结 = Pi 根本没跑, 重启指令发不进去, 重启无用】
 *                        【在涨但没握手 = Pi 活着, 那正是"重启 Pi 客户端"唯一有效的场景】。 */
static float _bg_RpiPend(void)      { return (float)ucGetRpiPendTask(); }
static float _bg_RpiPendMs(void)    { return (float)ulGetRpiPendMs(); }
static float _bg_RpiLinkRetry(void) { return (float)usGetRpiLinkRetry(); }
static float _bg_RpiPendDrop(void)  { return (float)usGetRpiPendDrop(); }

/* M0 副陀螺帧号（只读遥测: 停涨 = 模块掉线 / 波特率不对） */
static float _beamM0Frames(void)
{
    return (float)ulGetM0YawFrameCount();
}

/* 巡线快照: ⚠️ vGetLineData 的四个指针参数是**无条件解引用**的（没有 NULL 判断，
 * 只有 vGetHostageResult 才判空），所以每次都要传齐四个局部变量。 */
static float _beamLineErr(void)
{
    float fErr, fSteer, fAngle;
    uint8_t ucV;
    vGetLineData(&fErr, &fSteer, &fAngle, &ucV);
    return fErr;
}

static float _beamLineSteer(void)
{
    float fErr, fSteer, fAngle;
    uint8_t ucV;
    vGetLineData(&fErr, &fSteer, &fAngle, &ucV);
    return fSteer;
}

static float _beamLineAngle(void)
{
    float fErr, fSteer, fAngle;
    uint8_t ucV;
    vGetLineData(&fErr, &fSteer, &fAngle, &ucV);
    return fAngle;
}

static float _beamLineValid(void)
{
    float fErr, fSteer, fAngle;
    uint8_t ucV;
    vGetLineData(&fErr, &fSteer, &fAngle, &ucV);
    return (float)ucV;
}

static float _beamLineFrames(void){ return (float)ulGetLineFrameCount(); }

static float _beamQrCode(void)      { return (float)iGetQrCode(); }
static float _beamRpiLink(void)     { return (float)emGetRpiLinkState(); }
static float _beamHostageStatus(void){ uint8_t uc; vGetHostageResult(&uc, NULL, NULL); return (float)uc; }
static float _beamHostageCount(void) { uint8_t uc; vGetHostageResult(NULL, &uc, NULL); return (float)uc; }

/* 第 ucIdx 个目标（0 起）的字段: 0=形状码 1=中心x 2=中心y; 该目标不存在时回 0 */
static float beam_hostageField(uint8_t ucIdx, uint8_t ucField)
{
    stHostageTgtTdf astTgt[HOSTAGE_MAX_COUNT];
    uint8_t ucCnt = 0;
    uint8_t ucI;

    for (ucI = 0; ucI < HOSTAGE_MAX_COUNT; ucI++)
    {
        astTgt[ucI].ucShape = 0;
        astTgt[ucI].usCx    = 0;
        astTgt[ucI].usCy    = 0;
    }
    vGetHostageResult(NULL, &ucCnt, astTgt);
    if (ucIdx >= ucCnt) { return 0.0f; }

    if (ucField == 0u) { return (float)astTgt[ucIdx].ucShape; }
    if (ucField == 1u) { return (float)astTgt[ucIdx].usCx; }
    return (float)astTgt[ucIdx].usCy;
}

static float _beamHs1Shape(void) { return beam_hostageField(0u, 0u); }
static float _beamHs1X(void)     { return beam_hostageField(0u, 1u); }
static float _beamHs1Y(void)     { return beam_hostageField(0u, 2u); }
static float _beamHs2Shape(void) { return beam_hostageField(1u, 0u); }
static float _beamHs2X(void)     { return beam_hostageField(1u, 1u); }
static float _beamHs2Y(void)     { return beam_hostageField(1u, 2u); }
static float _beamHs3Shape(void) { return beam_hostageField(2u, 0u); }
static float _beamHs3X(void)     { return beam_hostageField(2u, 1u); }
static float _beamHs3Y(void)     { return beam_hostageField(2u, 2u); }

/* 色球遥测（只读）—— 现场标定 BALL_CX_CENTER / BALL_SIDE_SIGN / BALL_PX_TO_MM_X 时
 * 从「蓝牙」Tab 无线实时看三球 cx/cy，不用反复烧录。
 * ⚠️ 读的是 vGetColorResult（非 take 语义的临界区快照），故可进表;
 *    同模块的 ucTakeColorReady 是 take 型, 读会清零, **绝不进表**（见文件头三条硬约束）。 */
static float _beamColorStatus(void){ uint8_t uc; vGetColorResult(&uc, NULL, NULL); return (float)uc; }

/* 矩形地板遥测（只读）—— 现场标定 rect_cx_center / rect_cy_center 时看这里。
 * ⚠️ 读的是 vGetRectResult（非 take 语义的临界区快照），故可进表；
 *    同模块的 ucTakeRectReady 是 take 型、读会清零，**绝不进表**（见文件头三条硬约束）。 */
/* rect_status 走 BEAM_ADAPT_RO_U8(RectStatus)（挂在 ucGetRectStatus 上, 与 color_status/
 * road_status 同型）—— 原来这里是个返回 float 的私有 adapter + BEAM_VAL_F32,
 * 于是同一个语义的遥测量在控制台上显示成 "1.00"/"0.00" 且认不出是布尔。 */
static float _beamRectCx(void)    { uint16_t v; vGetRectResult(NULL, &v, NULL, NULL, NULL); return (float)v; }
static float _beamRectCy(void)    { uint16_t v; vGetRectResult(NULL, NULL, &v, NULL, NULL); return (float)v; }
static float _beamRectSpread(void){ uint8_t v;  vGetRectResult(NULL, NULL, NULL, NULL, &v); return (float)v; }
static float _beamColorCount(void) { uint8_t uc; vGetColorResult(NULL, &uc, NULL); return (float)uc; }

/* 靶标流帧遥测（只读）—— 台架调闭环时的第一手证据:
 *   aim_frames **停涨** = 流根本没在来（Pi 没起流 / "7" 没发出去 / 链路聋了）;
 *   aim_frames 在涨但 aim_count=0 = 流活着、Pi 只是没找到靶（窗跟丢/掩膜失锁）。
 * 这两种失效在现象上都是"舵机不动", 只有这个数分得开 —— 所以别只加帧数不加靶数。
 * ⚠️ 读的是 vGetAimFrame / ulGetAimFrameCount（非 take 语义）, 故可安全进表。 */
static float _beamAimFrames(void)   { return (float)ulGetAimFrameCount(); }
static float _beamAimCount(void)    { uint8_t uc; vGetAimFrame(&uc, NULL, NULL); return (float)uc; }

/* 第 ucIdx 个色球（0 起）的字段: 0=颜色码 1=中心x 2=中心y; 该球不存在时回 0 */
static float beam_colorField(uint8_t ucIdx, uint8_t ucField)
{
    stColorTgtTdf astTgt[COLOR_MAX_COUNT];
    uint8_t ucCnt = 0;
    uint8_t ucI;

    for (ucI = 0; ucI < COLOR_MAX_COUNT; ucI++)
    {
        astTgt[ucI].ucColor = 0;
        astTgt[ucI].usCx    = 0;
        astTgt[ucI].usCy    = 0;
    }
    vGetColorResult(NULL, &ucCnt, astTgt);
    if (ucIdx >= ucCnt) { return 0.0f; }

    if (ucField == 0u) { return (float)astTgt[ucIdx].ucColor; }
    if (ucField == 1u) { return (float)astTgt[ucIdx].usCx; }
    return (float)astTgt[ucIdx].usCy;
}

static float _beamCl1Color(void) { return beam_colorField(0u, 0u); }
static float _beamCl1X(void)     { return beam_colorField(0u, 1u); }
static float _beamCl1Y(void)     { return beam_colorField(0u, 2u); }
static float _beamCl2Color(void) { return beam_colorField(1u, 0u); }
static float _beamCl2X(void)     { return beam_colorField(1u, 1u); }
static float _beamCl2Y(void)     { return beam_colorField(1u, 2u); }
static float _beamCl3Color(void) { return beam_colorField(2u, 0u); }
static float _beamCl3X(void)     { return beam_colorField(2u, 1u); }
static float _beamCl3Y(void)     { return beam_colorField(2u, 2u); }

/* --- 表本体 ---
 * 单位里的 ° 为 UTF-8（工程已配 --locale=C, 中文/全角字面量原样落盘, 见 map.c 的步骤名）。
 * ucWarn=1 表示「运行中修改会影响控制律」, 界面打 ⚠ 提示（不拦写入——无线边跑边调
 * 正是这条链路存在的意义）。 */
static const stBeamParamTdf s_astBeamParam[] =
{
    /* ---- 视觉 ---- */
    { "visual_px_to_mm_x", "视觉像素增益X",  "视觉", "mm/px", BEAM_VAL_F32, 1u, 1u,  0.01f,  5.00f, fGetVisualServoPxToMmX, vSetVisualServoPxToMmX },
    { "visual_px_to_mm_y", "视觉像素增益Y",  "视觉", "mm/px", BEAM_VAL_F32, 1u, 1u,  0.01f,  5.00f, fGetVisualServoPxToMmY, vSetVisualServoPxToMmY },
    { "origin_x",          "像素原点X",      "视觉", "px",    BEAM_VAL_F32, 1u, 1u,  0.00f, 1920.f, fGetOriginX,            vSetOriginX            },
    { "origin_y",          "像素原点Y",      "视觉", "px",    BEAM_VAL_F32, 1u, 1u,  0.00f, 1080.f, fGetOriginY,            vSetOriginY            },
    { "direct_x",          "视觉偏移X",      "视觉", "mm",    BEAM_VAL_F32, 1u, 1u, -1000.f, 1000.f, fGetDirectX,           vSetDirectX            },
    { "direct_y",          "视觉偏移Y",      "视觉", "mm",    BEAM_VAL_F32, 1u, 1u, -1000.f, 1000.f, fGetDirectY,           vSetDirectY            },
    { "sehuan_streaming",  "色环流式定标",   "视觉", "",      BEAM_VAL_U8,  1u, 0u,  0.00f,    1.f, _bg_SehuanStreaming,   _bs_SehuanStreaming    },
    { "sehuan_y_diff",     "色环Y像素差",    "视觉", "px",    BEAM_VAL_I8,  1u, 0u, -128.f,   127.f, _bg_SehuanYDiff,       _bs_SehuanYDiff        },
    { "distance_wukuai",   "物块间距",       "视觉", "mm",    BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, fGetDistanceWukuai,    NULL                   },

    /* ---- 底盘 ---- */
    { "world_heading",     "世界航向",       "底盘", "°",     BEAM_VAL_F32, 1u, 1u, -360.f,   360.f, fGetWorldHeading,      vSetWorldHeading       },
    { "chassis_accel",     "底盘加速度",     "底盘", "rpm/s", BEAM_VAL_I32, 1u, 1u,  0.00f,  255.f, _bg_ChassisAccel,      _bs_ChassisAccel       },
    /* ---- 底盘动作组（时间轴速度混合播放器; 增益/限幅可写, 状态只读）----
     * ucWarn=1 的都是控制律输入, 播放中改会当场改变车的走向/速度上限;
     * ch_time_scale 改的是下一次播放的 mm→时长折算（排好的时间轴不长个儿变）。 */
    { "ch_time_scale",     "时间系数k",      "底盘动作组", "",    BEAM_VAL_F32, 1u, 1u,  0.01f,   5.00f, fGetChActTimeScale,   vSetChActTimeScale     },
    { "ch_head_kp",        "航向环P",        "底盘动作组", "rpm/°", BEAM_VAL_F32, 1u, 1u,  0.00f,  20.00f, fGetChActHeadKp,      vSetChActHeadKp        },
    { "ch_head_ki",        "航向环I",        "底盘动作组", "",    BEAM_VAL_F32, 1u, 1u,  0.00f,   5.00f, fGetChActHeadKi,      vSetChActHeadKi        },
    { "ch_head_kd",        "航向环D",        "底盘动作组", "",    BEAM_VAL_F32, 1u, 1u,  0.00f,  20.00f, fGetChActHeadKd,      vSetChActHeadKd        },
    { "ch_head_ki_max",    "积分限幅",       "底盘动作组", "",    BEAM_VAL_F32, 1u, 1u,  0.00f,  50.00f, fGetChActHeadKiMax,   vSetChActHeadKiMax     },
    { "ch_head_db_deg",    "保持死区",       "底盘动作组", "°",   BEAM_VAL_F32, 1u, 0u,  0.00f,  10.00f, fGetChActHeadDb,      vSetChActHeadDb        },
    { "ch_w_max",          "旋转限幅",       "底盘动作组", "rpm", BEAM_VAL_F32, 1u, 1u,  0.00f, 200.00f, fGetChActWMax,        vSetChActWMax          },
    { "ch_wheel_max",      "四轮饱和钳",     "底盘动作组", "rpm", BEAM_VAL_F32, 1u, 1u,  0.00f,1000.00f, fGetChActWheelMax,    vSetChActWheelMax      },
    /* 支点转弯段（CH_ACT_KIND_PIVOT）的 PID —— 出厂值引用 PIVOT_TURN_*（现场标定） */
    { "ch_pivot_kp",       "支点P",          "底盘动作组", "rpm/°", BEAM_VAL_F32, 1u, 1u,  0.00f,  20.00f, fGetChPivotKp,        vSetChPivotKp          },
    { "ch_pivot_ki",       "支点I",          "底盘动作组", "",    BEAM_VAL_F32, 1u, 1u,  0.00f,   5.00f, fGetChPivotKi,        vSetChPivotKi          },
    { "ch_pivot_kd",       "支点D",          "底盘动作组", "",    BEAM_VAL_F32, 1u, 1u,  0.00f,  20.00f, fGetChPivotKd,        vSetChPivotKd          },
    { "ch_pivot_ki_max",   "支点积分限幅",   "底盘动作组", "",    BEAM_VAL_F32, 1u, 1u,  0.00f,  50.00f, fGetChPivotKiMax,     vSetChPivotKiMax       },
    { "ch_pivot_w_max",    "支点驱动限幅",   "底盘动作组", "rpm", BEAM_VAL_F32, 1u, 1u,  0.00f, 255.00f, fGetChPivotWMax,      vSetChPivotWMax        },
    { "ch_busy",           "播放器忙",       "底盘动作组", "",    BEAM_VAL_U8,  0u, 0u,  0.00f,    1.f, _bg_ChActBusy,        NULL                   },
    { "ch_playing",        "正在播的组",     "底盘动作组", "",    BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _bg_ChActPlayingId,   NULL                   },

    /* ---- 姿态（只读）---- */
    { "att_roll",          "姿态横滚",       "姿态", "°",     BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, fGetAttitudeRoll,      NULL                   },
    { "att_pitch",         "姿态俯仰",       "姿态", "°",     BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, fGetAttitudePitch,     NULL                   },
    { "att_yaw",           "姿态航向",       "姿态", "°",     BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, fGetAttitudeYaw,       NULL                   },

    /* ---- 巡线（目标角 + 控制律标定可写, 其余只读遥测）----
     * line_angle_target = 航向环的目标（见 app_beam.h）: 现场边跑边调「车该与线成多少度」,
     * 不用重编译; 标好了把终值写回 app_config.h 的 LINE_ANGLE_TARGET_DEG 固化。
     * ucWarn=1: 它是控制律的输入, 运行中改会当场改变车的走向。 */
    { "line_angle_target", "目标图像角",     "巡线", "°",     BEAM_VAL_F32, 1u, 1u, -90.00f,  90.00f, fGetLineAngleTarget,   vSetLineAngleTarget    },
    /* 陀螺那条（2026-09 定案）: 目标角 = 我要的世界航向(零点=开机朝向, 同绝对转弯)。
     * 范围 ±180（世界航向的物理量程）, 与 line_angle_target 的 ±90 不同。
     * ⚠️ 它**只在全局 `LINE_CONV_SRC=陀螺/融合` 时被用到**, 也不决定任何权重（权重/比例尺那套已随融合删除）——
     *    "这一步听谁"是全局宏 LINE_CONV_SRC, 不是这个运行时的开关。ucWarn=1: 控制律输入。 */
    { "line_gyro_target",  "陀螺目标角",     "巡线", "°",     BEAM_VAL_F32, 1u, 1u,-180.00f, 180.00f, fGetLineGyroTarget,   vSetLineGyroTarget     },
    /* 融合档(LINE_CONV_SRC=3)的相机占比[%]: 两个输出 w 按它:(100−它) 加权。0=全陀螺、100=全相机。
     * ⚠️ 只在融合档被读到; 单源档改它毫无作用（"这一步听谁"是全局宏 LINE_CONV_SRC, 不是这个量）。
     * ucWarn=1: 控制律输入, 跑动中改车会当场改变转向力度。 */
    { "line_cam_pct",      "融合相机占比",   "巡线", "%",     BEAM_VAL_F32, 1u, 1u,   0.00f,  100.00f, fGetLineCamPct,        vSetLineCamPct        },
    /* 巡线直走的时间系数: 估时 × 本值 = 实际闭环时长。ucWarn=1: 直接改变行程, 跑动中改车会当场多走/少走。 */
    { "line_time_scale",   "直走时间系数",   "巡线", "",      BEAM_VAL_F32, 1u, 1u,   0.01f,    5.00f, fGetLineTimeScale,     vSetLineTimeScale     },
    /* ---- 巡线控制律标定（2026-10-06 加, 上电默认 = app_config.h 的同名宏, 断电即回）----
     * 与「小球对位」那批同一个用法: 现场边跑边调（app_line.c 每拍现取, 下一拍生效）,
     * 标好了把终值写回对应宏固化。增益分「原地对准 / 边走边修」两组（见 app_config.h
     * 「巡线闭环」段头）, 限幅/方向两组共用。⚠️ 死区、方向这类量纲约束（死区 ≥ 相机角
     * 分辨率）写在宏注释里, 这里只管数值。
     * ucWarn=1 的都是控制律输入, 运行中改会当场改变车的走向/速度上限。 */
    { "line_steer_to_vx",     "对准横移增益",  "巡线", "rpm/单位", BEAM_VAL_F32, 1u, 1u,   0.00f,  200.00f, fGetLineSteerToVx,     vSetLineSteerToVx     },
    { "line_walk_steer_to_vx", "边走横移增益", "巡线", "rpm/单位", BEAM_VAL_F32, 1u, 1u,   0.00f,  200.00f, fGetLineWalkSteerToVx, vSetLineWalkSteerToVx },
    { "line_vx_sign",       "横移方向",       "巡线", "",        BEAM_VAL_F32, 1u, 0u,  -1.00f,    1.00f, fGetLineVxSign,        vSetLineVxSign        },
    { "line_vx_max",        "横移限幅",       "巡线", "rpm",     BEAM_VAL_F32, 1u, 1u,   0.00f,  500.00f, fGetLineVxMax,         vSetLineVxMax         },
    { "line_cam_to_w",      "对准相机增益",   "巡线", "rpm/°",   BEAM_VAL_F32, 1u, 1u,  -5.00f,    5.00f, fGetLineCamToW,        vSetLineCamToW        },
    { "line_walk_cam_to_w", "边走相机增益",   "巡线", "rpm/°",   BEAM_VAL_F32, 1u, 1u,  -8.00f,    8.00f, fGetLineWalkCamToW,    vSetLineWalkCamToW    },
    { "line_cam_db_deg",    "相机死区",       "巡线", "°",       BEAM_VAL_F32, 1u, 0u,   0.00f,   10.00f, fGetLineCamDeadband,   vSetLineCamDeadband   },
    { "line_gyro_to_w",     "对准陀螺增益",   "巡线", "rpm/°",   BEAM_VAL_F32, 1u, 1u,  -5.00f,    5.00f, fGetLineGyroToW,       vSetLineGyroToW       },
    { "line_walk_gyro_to_w", "边走陀螺增益",  "巡线", "rpm/°",   BEAM_VAL_F32, 1u, 1u,  -8.00f,    8.00f, fGetLineWalkGyroToW,   vSetLineWalkGyroToW   },
    { "line_gyro_db_deg",   "陀螺死区",       "巡线", "°",       BEAM_VAL_F32, 1u, 0u,   0.00f,   10.00f, fGetLineGyroDeadband,  vSetLineGyroDeadband  },
    { "line_w_max",         "旋转限幅",       "巡线", "rpm",     BEAM_VAL_F32, 1u, 1u,   0.00f,  200.00f, fGetLineWMax,          vSetLineWMax          },
    { "line_err",          "巡线偏差",       "巡线", "",      BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, _beamLineErr,          NULL                   },
    { "line_steer",        "巡线控制量",     "巡线", "",      BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, _beamLineSteer,        NULL                   },
    { "line_angle",        "巡线航向角",     "巡线", "°",     BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, _beamLineAngle,        NULL                   },
    { "line_valid",        "巡线有效",       "巡线", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamLineValid,        NULL                   },
    { "line_frames",       "巡线帧号",       "巡线", "",      BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamLineFrames,       NULL                   },

    /* ---- 任务结果（只读）---- */
    { "qr_code",           "二维码结果",     "任务", "",      BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamQrCode,           NULL                   },
    { "qr_manual",         "二维码人工预设", "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    1.f, _bg_QrManual,          NULL                   },
    { "photo_ok",          "拍照结果",      "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    1.f, _bg_PhotoResult,       NULL                   },
    { "hostage_status",    "人质识别状态",   "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamHostageStatus,    NULL                   },
    { "hostage_count",     "人质数量",       "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamHostageCount,     NULL                   },
    { "hostage1_shape",    "人质1形状",      "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamHs1Shape,         NULL                   },
    { "hostage1_x",        "人质1中心X",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamHs1X,             NULL                   },
    { "hostage1_y",        "人质1中心Y",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamHs1Y,             NULL                   },
    { "hostage2_shape",    "人质2形状",      "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamHs2Shape,         NULL                   },
    { "hostage2_x",        "人质2中心X",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamHs2X,             NULL                   },
    { "hostage2_y",        "人质2中心Y",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamHs2Y,             NULL                   },
    { "hostage3_shape",    "人质3形状",      "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamHs3Shape,         NULL                   },
    { "hostage3_x",        "人质3中心X",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamHs3X,             NULL                   },
    { "hostage3_y",        "人质3中心Y",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamHs3Y,             NULL                   },
    { "color_status",      "色球识别状态",   "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamColorStatus,      NULL                   },
    { "color_count",       "色球数量",       "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamColorCount,       NULL                   },
    { "color1_color",      "色球1颜色",      "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamCl1Color,         NULL                   },
    { "color1_x",          "色球1中心X",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamCl1X,             NULL                   },
    { "color1_y",          "色球1中心Y",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamCl1Y,             NULL                   },
    { "color2_color",      "色球2颜色",      "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamCl2Color,         NULL                   },
    { "color2_x",          "色球2中心X",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamCl2X,             NULL                   },
    { "color2_y",          "色球2中心Y",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamCl2Y,             NULL                   },
    { "color3_color",      "色球3颜色",      "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamCl3Color,         NULL                   },
    { "color3_x",          "色球3中心X",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamCl3X,             NULL                   },
    { "color3_y",          "色球3中心Y",     "任务", "px",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamCl3Y,             NULL                   },
    /* 靶标流遥测: 帧号停涨 = 流没在来; 帧号在涨但数量=0 = 流活着、Pi 没找到靶。
     * 两种失效的现象都是"舵机不动", 只有这两个数分得开（详见上面读适配的注释）。 */
    { "aim_frames",        "靶标流帧号",     "任务", "",      BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamAimFrames,        NULL                   },
    { "aim_count",         "靶标流靶数",     "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamAimCount,         NULL                   },
    { "ball_yaw",          "三球连线角",     "任务", "°",     BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, fGetBallYawLine,       NULL                   },
    { "ball_fit_ok",       "三球共线合格",   "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    1.f, _bg_BallFitOk,         NULL                   },
    { "rect_status",       "矩形识别状态",   "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    1.f, _bg_RectStatus,        NULL                   },
    { "rect_cx",           "板心列",         "任务", "px",    BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, _beamRectCx,           NULL                   },
    { "rect_cy",           "板心行",         "任务", "px",    BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, _beamRectCy,           NULL                   },
    { "rect_spread",       "板心散布",       "任务", "px",    BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, _beamRectSpread,       NULL                   },
    { "align_ok",          "对位链成功",     "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    1.f, _bg_AlignOk,           NULL                   },
    { "align_method",      "对位用的方案",   "任务", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    2.f, _bg_AlignMethod,       NULL                   },

    /* ---- 任务补光（蓝牙/OLED 可写; 初值 = app_config.h「任务补光」区编译期默认, 断电即回）----
     * 读者 = vFillLightTaskBegin（每个任务步骤现取）—— 改完**下一步就生效**, 不用重编译;
     * 现场调好把终值写回 app_config.h 固化（同小球对位那批的约定）。
     * PCT 口径与固件一致: 0=开灯但 0% 亮（等效灭）; 1~100=单独覆盖;
     * >100（含 255）=跟随「默认亮度」fill_task_pct —— 哨兵语义原样保留（所以 max=255）。 */
    { "fill_task_en",       "补光总开关",   "补光", "",      BEAM_VAL_U8,  1u, 0u,  0.00f,    1.f, _bg_FillTaskEn,        _bs_FillTaskEn        },
    { "fill_task_pct",      "补光默认亮度", "补光", "%",     BEAM_VAL_U8,  1u, 0u,  0.00f,  100.f, _bg_FillTaskPct,       _bs_FillTaskPct       },
    { "fill_task_road_en",  "看路开灯",     "补光", "",      BEAM_VAL_U8,  1u, 0u,  0.00f,    1.f, _bg_FillTaskRoadEn,    _bs_FillTaskRoadEn    },
    { "fill_task_road_pct", "看路亮度",     "补光", "%",     BEAM_VAL_U8,  1u, 0u,  0.00f,  255.f, _bg_FillTaskRoadPct,   _bs_FillTaskRoadPct   },
    { "fill_task_ball_en",  "色球开灯",     "补光", "",      BEAM_VAL_U8,  1u, 0u,  0.00f,    1.f, _bg_FillTaskBallEn,    _bs_FillTaskBallEn    },
    { "fill_task_ball_pct", "色球亮度",     "补光", "%",     BEAM_VAL_U8,  1u, 0u,  0.00f,  255.f, _bg_FillTaskBallPct,   _bs_FillTaskBallPct   },
    { "fill_task_shoot_en", "射靶开灯",     "补光", "",      BEAM_VAL_U8,  1u, 0u,  0.00f,    1.f, _bg_FillTaskShootEn,   _bs_FillTaskShootEn   },
    { "fill_task_shoot_pct","射靶亮度",     "补光", "%",     BEAM_VAL_U8,  1u, 0u,  0.00f,  255.f, _bg_FillTaskShootPct,  _bs_FillTaskShootPct  },
    { "fill_task_hs_en",    "人质开灯",     "补光", "",      BEAM_VAL_U8,  1u, 0u,  0.00f,    1.f, _bg_FillTaskHsEn,      _bs_FillTaskHsEn      },
    { "fill_task_hs_pct",   "人质亮度",     "补光", "%",     BEAM_VAL_U8,  1u, 0u,  0.00f,  255.f, _bg_FillTaskHsPct,     _bs_FillTaskHsPct     },
    { "fill_task_qr_en",    "扫码开灯",     "补光", "",      BEAM_VAL_U8,  1u, 0u,  0.00f,    1.f, _bg_FillTaskQrEn,      _bs_FillTaskQrEn      },
    { "fill_task_qr_pct",   "扫码亮度",     "补光", "%",     BEAM_VAL_U8,  1u, 0u,  0.00f,  255.f, _bg_FillTaskQrPct,     _bs_FillTaskQrPct     },

    /* ---- 小球对位标定（蓝牙可写; 初值 = app_config.h 的 BALL_* 编译期默认, 断电即回） ---- */
    { "ball_cx_center",    "理想中球列",     "小球对位", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1920.f, fGetBallCxCenter,    vSetBallCxCenter    },
    { "ball_cy_center",    "理想中球行",     "小球对位", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1080.f, fGetBallCyCenter,    vSetBallCyCenter    },
    { "ball_px_to_mm_x",   "横向比例",       "小球对位", "mm/px", BEAM_VAL_F32, 1u, 0u, 0.01f,    5.f, fGetBallPxToMmX,     vSetBallPxToMmX     },
    { "ball_px_to_mm_y",   "纵向比例",       "小球对位", "mm/px", BEAM_VAL_F32, 1u, 0u, 0.01f,    5.f, fGetBallPxToMmY,     vSetBallPxToMmY     },
    { "ball_yaw_scale",    "图像角倍率",     "小球对位", "",     BEAM_VAL_F32, 1u, 1u,   0.5f,   10.f, fGetBallYawScale,    vSetBallYawScale    },
    { "ball_yaw_sign",     "转正方向",       "小球对位", "",     BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetBallYawSign,     vSetBallYawSign     },
    { "ball_yaw_target",   "理想角度",       "小球对位", "°",    BEAM_VAL_F32, 1u, 0u, -90.0f,   90.f, fGetBallYawTarget,   vSetBallYawTarget   },
    { "ball_yaw_db_deg",   "转正死区",       "小球对位", "°",    BEAM_VAL_F32, 1u, 0u,   0.0f,   10.f, fGetBallYawDeadband, vSetBallYawDeadband },
    { "ball_dx_db_mm",     "横移死区",       "小球对位", "mm",   BEAM_VAL_F32, 1u, 0u,   0.0f,   50.f, fGetBallDxDeadband,  vSetBallDxDeadband  },
    { "ball_dy_db_mm",     "纵移死区",       "小球对位", "mm",   BEAM_VAL_F32, 1u, 0u,   0.0f,   50.f, fGetBallDyDeadband,  vSetBallDyDeadband  },
    { "ball_fwd_sign",     "纵移方向",       "小球对位", "",     BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetBallFwdSign,     vSetBallFwdSign     },
    { "ball_yaw_en",       "Yaw修正开关",    "小球对位", "",     BEAM_VAL_U8,  1u, 0u,   0.0f,    1.f, _bg_BallYawEn,       _bs_BallYawEn       },
    { "ball_xy_swap",      "XY整体对调",     "小球对位", "",     BEAM_VAL_U8,  1u, 0u,   0.0f,    1.f, _bg_BallXySwap,      _bs_BallXySwap      },
    { "ball_accel",        "对位加速度",     "小球对位", "",     BEAM_VAL_I32, 1u, 0u,   0.0f,  255.f, _bg_BallAlignAccel,  _bs_BallAlignAccel  },
    { "ball_speed",        "对位速度",       "小球对位", "rpm",  BEAM_VAL_I32, 1u, 0u,   0.0f, 65535.f, _bg_BallAlignSpeed,  _bs_BallAlignSpeed  },
    { "ball_pullback_mm",  "缺球拉回距离",   "小球对位", "mm",   BEAM_VAL_F32, 1u, 0u,   0.0f,  500.f, fGetBallPullbackMm,  vSetBallPullbackMm  },
    /* ---- 矩形板心对位标定（蓝牙可写; 只有两个中心量, 其余复用上面的 ball_*）---- */
    { "rect_cx_center",    "理想板心列(待标定)", "矩形对位", "px", BEAM_VAL_F32, 1u, 0u, 0.0f, 1920.f, fGetRectCxCenter, vSetRectCxCenter },
    { "rect_cy_center",    "理想板心行(待标定)", "矩形对位", "px", BEAM_VAL_F32, 1u, 0u, 0.0f, 1080.f, fGetRectCyCenter, vSetRectCyCenter },

    /* ---- 人质对位标定（蓝牙可写; 初值 = app_config.h 的 HOSTAGE_* 编译期默认, 断电即回）
     *      基准人质由积木参数选(0左/1中/2右), 这里只有几何/方向/速度量 —— 本步不做转正。 ---- */
    { "hs_cx_center",      "理想人质列",     "人质对位", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1920.f, fGetHsCxCenter,      vSetHsCxCenter      },
    { "hs_cy_center",      "理想人质行",     "人质对位", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1080.f, fGetHsCyCenter,      vSetHsCyCenter      },
    { "hs_px_to_mm_x",     "横向比例",       "人质对位", "mm/px", BEAM_VAL_F32, 1u, 0u,  0.01f,    5.f, fGetHsPxToMmX,       vSetHsPxToMmX       },
    { "hs_px_to_mm_y",     "纵向比例",       "人质对位", "mm/px", BEAM_VAL_F32, 1u, 0u,  0.01f,    5.f, fGetHsPxToMmY,       vSetHsPxToMmY       },
    { "hs_dx_db_mm",       "横移死区",       "人质对位", "mm",   BEAM_VAL_F32, 1u, 0u,   0.0f,   50.f, fGetHsDxDeadband,    vSetHsDxDeadband    },
    { "hs_dy_db_mm",       "纵移死区",       "人质对位", "mm",   BEAM_VAL_F32, 1u, 0u,   0.0f,   50.f, fGetHsDyDeadband,    vSetHsDyDeadband    },
    { "hs_side_sign",      "横移方向",       "人质对位", "",     BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetHsSideSign,      vSetHsSideSign      },
    { "hs_fwd_sign",       "纵移方向",       "人质对位", "",     BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetHsFwdSign,       vSetHsFwdSign       },
    { "hs_xy_swap",        "XY整体对调",     "人质对位", "",     BEAM_VAL_U8,  1u, 0u,   0.0f,    1.f, _bg_HsXySwap,        _bs_HsXySwap        },
    { "hs_accel",          "对位加速度",     "人质对位", "",     BEAM_VAL_I32, 1u, 0u,   0.0f,  255.f, _bg_HsAlignAccel,    _bs_HsAlignAccel    },
    { "hs_speed",          "对位速度",       "人质对位", "rpm",  BEAM_VAL_I32, 1u, 0u,   0.0f, 65535.f, _bg_HsAlignSpeed,    _bs_HsAlignSpeed    },

    /* ---- 单动作抓取标定（蓝牙可写; 初值 = app_config.h 的 ONE_GRAB_* 编译期默认, 断电即回;
     *      2026-10 加。只录一个"抓中间目标"的动作, 非中间目标靠平移 (位置-1)×one_lat_mm
     *      挪到中间正前方再抓 —— one_lat_mm 现场量相邻目标中心距, one_lat_dir 定分量加在
     *      哪根轴/哪边为正(反了就换一档), one_line_ms 是 mode=OpenMV 那档收敛段的时长） ---- */
    { "one_lat_mm",        "目标间距分量",   "单动作抓取", "mm",  BEAM_VAL_F32, 1u, 0u,   0.0f,  500.f, fGetOneLatMm,        vSetOneLatMm        },
    { "one_lat_dir",       "分量方向",       "单动作抓取", "",    BEAM_VAL_U8,  1u, 0u,   0.0f,    4.f, _bg_OneLatDir,       _bs_OneLatDir       },
    { "one_line_ms",       "巡线收敛时长",   "单动作抓取", "ms",  BEAM_VAL_I32, 1u, 0u,   0.0f, 10000.f, _bg_OneLineMs,      _bs_OneLineMs       },
    /* 只读遥测: 人质单动作抓取不回中, 残留偏移(世界系)记在这, 由世界系平移积木
     * 勾"叠加上次残留偏移"一次性吃掉 —— 吃掉前后看这两个数, 就是核对机制的入口。 */
    { "path_res_x",        "残留偏移X",      "单动作抓取", "mm",  BEAM_VAL_F32, 0u, 0u, -500.f,  500.f, fGetPathResX,         NULL                },
    { "path_res_y",        "残留偏移Y",      "单动作抓取", "mm",  BEAM_VAL_F32, 0u, 0u, -500.f,  500.f, fGetPathResY,         NULL                },

    /* ---- 标靶校准标定（蓝牙可写; 初值 = app_config.h 的 TARGET_* 编译期默认, 断电即回）
     *      ⚠️ 「瞄准点」= **靶心应该停在画面里的哪个像素**（也就是瞄准的设定值）:
     *         闭环干的事是 `Δ = 靶心 − 瞄准点 → 0`, 所以增大瞄准点列 = 靶心最终停在更靠右的位置。
     *         默认 (960,540) 就是画面正中心, 但它不只是"画面中心"——靶心稳定地偏在一边时
     *         （开环增量式的残余偏差, 舵机死区/回差导致走不到指令角度）就填这里把它整体平移掉。
     *         列 = x(0~1919), 行 = y(0~1079)。想改"走多少"改 PX_TO_DEG, 想翻方向改 SIGN。
     *      标定顺序: 先 X/Y_SIGN 看方向对不对 → 再 PX_TO_DEG 看走多少 → 最后 MAX_STEP_DEG 收步长。
     *      ⚠️ calib_timeout 上界卡在 7999: 那条命令是**阻塞**的, 期间 Pi 不发心跳, 超了本端
     *         会闪一下 DISCONNECTED（对结果无害但会关掉蓝牙臂编辑窗）。
     *      ⚠️ 刻意**不做**成"画面X/Y 各填一个舵机号": 两个独立编号能配出"两根轴指同一个舵机"
     *         这种危险组合, 而 xy_swap 只有两个合法状态（与 BALL_/HOSTAGE_XY_SWAP 同惯例）。 ---- */
    { "tg_center_x",       "瞄准点列",       "标靶校准", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1920.f, fGetTgCenterX,       vSetTgCenterX       },
    { "tg_center_y",       "瞄准点行",       "标靶校准", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1080.f, fGetTgCenterY,       vSetTgCenterY       },
    /* ⚠️ 比例尺下界给 0.01 而不是更小: 上下界要经 bt_fmt_f2 报文给控制台, 而它只有
     *    **两位小数** —— 填 0.001 报出去就是 "0.00", 界面上成了"下界 0"、滑块能拉到 0。
     *    0.01 已经等于"100px 偏差才修 1°"（比默认 0.05 还钝 5 倍）, 够当实际下限了。 */
    { "tg_px_to_deg_x",    "横向比例",       "标靶校准", "°/px", BEAM_VAL_F32, 1u, 0u,  0.01f,    1.f, fGetTgPxToDegX,      vSetTgPxToDegX      },
    { "tg_px_to_deg_y",    "纵向比例",       "标靶校准", "°/px", BEAM_VAL_F32, 1u, 0u,  0.01f,    1.f, fGetTgPxToDegY,      vSetTgPxToDegY      },
    { "tg_x_sign",         "横向方向",       "标靶校准", "",     BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetTgXSign,         vSetTgXSign         },
    { "tg_y_sign",         "纵向方向",       "标靶校准", "",     BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetTgYSign,         vSetTgYSign         },
    { "tg_deadband_px",    "瞄准死区",       "标靶校准", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f,  200.f, fGetTgDeadbandPx,    vSetTgDeadbandPx    },
    { "tg_max_step_deg",   "单次微调上限",   "标靶校准", "°",    BEAM_VAL_F32, 1u, 0u,   0.5f,   90.f, fGetTgMaxStepDeg,    vSetTgMaxStepDeg    },
    { "tg_move_ms",        "微调时长",       "标靶校准", "ms",   BEAM_VAL_I32, 1u, 0u,  50.0f, 3000.f, _bg_TgMoveMs,        _bs_TgMoveMs        },
    { "tg_move_slack_ms",  "到位轮询余量",   "标靶校准", "ms",   BEAM_VAL_I32, 1u, 0u,   0.0f, 2000.f, _bg_TgMoveSlackMs,   _bs_TgMoveSlackMs   },
    { "tg_xy_swap",        "XY整体对调",     "标靶校准", "",     BEAM_VAL_U8,  1u, 0u,   0.0f,    1.f, _bg_TgXySwap,        _bs_TgXySwap        },
    { "tg_calib_timeout_ms", "识别超时",     "标靶校准", "ms",   BEAM_VAL_I32, 1u, 0u, 500.0f, 8000.f, _bg_TgCalibTimeoutMs, _bs_TgCalibTimeoutMs },

    /* ---- 标靶对位标定（蓝牙可写; 初值 = app_config.h 的 TARGET_ALIGN_*, 断电即回）
     *      粗调用车、细调用臂: 本组把车横移到位, 再由上面那组用舵机6/3 细修。
     *      标定顺序: SIDE_SIGN 看方向 → PX_TO_MM_X 看走多少 → DX_DEADBAND_MM 收到不抖。
     *      ⚠️ 与「标靶校准」是**两套独立的量**（那边 °/px, 这边 mm/px）, 别混着调。 ---- */
    { "ta_cx_center",      "基准靶列",       "标靶对位", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1920.f, fGetTaCxCenter,      vSetTaCxCenter      },
    { "ta_cy_center",      "基准靶行",       "标靶对位", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1080.f, fGetTaCyCenter,      vSetTaCyCenter      },
    { "ta_px_to_mm_x",     "横向比例",       "标靶对位", "mm/px", BEAM_VAL_F32, 1u, 0u,  0.01f,    5.f, fGetTaPxToMmX,       vSetTaPxToMmX       },
    { "ta_dx_db_mm",       "横移死区",       "标靶对位", "mm",   BEAM_VAL_F32, 1u, 0u,   0.0f,   50.f, fGetTaDxDeadband,    vSetTaDxDeadband    },
    { "ta_side_sign",      "横移方向",       "标靶对位", "",     BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetTaSideSign,      vSetTaSideSign      },
    { "ta_xy_swap",        "XY整体对调",     "标靶对位", "",     BEAM_VAL_U8,  1u, 0u,   0.0f,    1.f, _bg_TaXySwap,        _bs_TaXySwap        },
    { "ta_accel",          "横移加速度",     "标靶对位", "",     BEAM_VAL_I32, 1u, 0u,   0.0f,  255.f, _bg_TaAlignAccel,    _bs_TaAlignAccel    },
    { "ta_speed",          "横移速度",       "标靶对位", "rpm",  BEAM_VAL_I32, 1u, 0u,   0.0f, 65535.f, _bg_TaAlignSpeed,    _bs_TaAlignSpeed    },

    /* ---- 标靶流控（蓝牙可写; 初值 = app_config.h 的 TARGET_SERVO_*, 断电即回）
     *      与「标靶校准」tg_* 的分工: 瞄准点/比例尺/方向/XY对调**共用 tg_***（同一套物理
     *      标定, 开环与闭环两方案共享）; 本组只管闭环节拍类参数（走多快/何时算收敛/何时放弃）。
     *      现场调参顺序: 先沿用已标好的 tg_* → 不收敛/振荡 → 降 ts_kp_scale →
     *      收敛太慢 → 加 ts_step_deg → 误收 → 收 ts_conv_px / 加 ts_conv_ticks。 ---- */
    { "ts_ctrl_ms",        "控制节拍",       "标靶流控", "ms",   BEAM_VAL_I32, 1u, 0u,  20.0f,  500.f, _bg_TsCtrlMs,        _bs_TsCtrlMs        },
    { "ts_move_ms",        "单拍时长",       "标靶流控", "ms",   BEAM_VAL_I32, 1u, 0u,  20.0f, 1000.f, _bg_TsMoveMs,        _bs_TsMoveMs        },
    { "ts_step_deg",       "单拍上限",       "标靶流控", "°",    BEAM_VAL_F32, 1u, 0u,   0.1f,   20.f, fGetTsStepDeg,       vSetTsStepDeg       },
    { "ts_conv_px",        "收敛阈值",       "标靶流控", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f,  100.f, fGetTsConvPx,        vSetTsConvPx        },
    { "ts_conv_ticks",     "收敛拍数",       "标靶流控", "",     BEAM_VAL_I32, 1u, 0u,   1.0f,   50.f, _bg_TsConvTicks,     _bs_TsConvTicks     },
    { "ts_travel_deg",     "行程上限",       "标靶流控", "°",    BEAM_VAL_F32, 1u, 0u,   1.0f,   90.f, fGetTsTravelDeg,     vSetTsTravelDeg     },
    { "ts_stale_ms",       "帧新鲜度",       "标靶流控", "ms",   BEAM_VAL_I32, 1u, 0u,  50.0f, 2000.f, _bg_TsStaleMs,       _bs_TsStaleMs       },
    { "ts_kp_scale",       "增益系数",       "标靶流控", "",     BEAM_VAL_F32, 1u, 1u,   0.1f,    2.f, fGetTsKpScale,       vSetTsKpScale       },

    /* ---- 底盘参与闭环（2026-09 加, 分组同「标靶流控」; 见 app_config.h 的「底盘参与闭环」段头）
     * 语义: |dx| 落进 ts_chassis_px 以内后**舵机6 让位**, 改由**底盘位置模式小步前后进给**继续闭环。
     * ⚠️ **是"前后"不是"左右"**（相机装了 90°, 图像横向 cx 对应车体纵移; 见 app_config.h 段头）,
     *    所以 ts_chassis_px_to_mm 配的是 BALL_PX_TO_MM_X 那把尺, 不是 ta_px_to_mm_x。
     * ⚠️ 标定顺序: ts_chassis_sign 看方向(故意摆偏, 看误差收敛还是发散)
     *    → ts_chassis_px_to_mm 看一步走多少(大了过冲振荡、小了收敛慢; 闭环复测, 偏了不致命)
     *    → ts_chassis_mm 收行程。
     * ⚠️ ts_chassis_en=0 时这四个量一个都不参与, 行为与加这一段之前完全一致。 */
    { "ts_chassis_en",        "底盘参与闭环", "标靶流控", "",      BEAM_VAL_U8,  1u, 1u,   0.0f,    1.f, _bg_TsChassisEn,     _bs_TsChassisEn     },
    { "ts_chassis_px",        "接管门限",     "标靶流控", "px",    BEAM_VAL_F32, 1u, 1u,   1.0f,  100.f, fGetTsChassisPx,     vSetTsChassisPx     },
    { "ts_chassis_px_to_mm",  "前后比例",     "标靶流控", "mm/px", BEAM_VAL_F32, 1u, 1u,  0.01f,    5.f, fGetTsChassisPxToMm, vSetTsChassisPxToMm },
    { "ts_chassis_sign",      "前后方向",     "标靶流控", "",      BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetTsChassisSign,   vSetTsChassisSign   },
    { "ts_chassis_mm",        "行程闸",       "标靶流控", "mm",    BEAM_VAL_F32, 1u, 0u,   1.0f,  500.f, fGetTsChassisMm,     vSetTsChassisMm     },

    /* ---- 阶段化 X→Y + 踢步 + 距离辅助（2026-10-06 加; 语义见 app_config.h「阶段化 X→Y」段头）
     * ts_seq_en=0 整组不参与。现场调参顺序: 先 ts_seq_en=1 验证阶段化本身 → Y 卡小误差
     * → 从 ts_kick_deg=0.2 起试踢（看 TGTS kick 行的 acc/rev/wst 比例）→ 还差最后一两 px
     * 且灵敏度实验已做 → 才开 ts_dist_en。 ---- */
    { "ts_seq_en",         "阶段化X→Y",     "标靶流控", "",      BEAM_VAL_U8,  1u, 1u,   0.0f,    1.f, _bg_TsSeqEn,         _bs_TsSeqEn         },
    { "ts_conv_x_px",      "X收敛阈值",     "标靶流控", "px",    BEAM_VAL_F32, 1u, 0u,   0.0f,  100.f, fGetTsConvXPx,       vSetTsConvXPx       },
    { "ts_conv_y_px",      "Y收敛阈值",     "标靶流控", "px",    BEAM_VAL_F32, 1u, 0u,   0.0f,  100.f, fGetTsConvYPx,       vSetTsConvYPx       },
    { "ts_stall_ticks",    "停滞判定拍数",  "标靶流控", "",      BEAM_VAL_I32, 1u, 0u,   1.0f,   50.f, _bg_TsStallTicks,    _bs_TsStallTicks    },
    { "ts_kick_deg",       "踢步幅度",      "标靶流控", "°",     BEAM_VAL_F32, 1u, 1u,  0.05f,    5.f, fGetTsKickDeg,       vSetTsKickDeg       },
    { "ts_kick_max",       "踢步次数",      "标靶流控", "",      BEAM_VAL_I32, 1u, 0u,   0.0f,   20.f, _bg_TsKickMax,       _bs_TsKickMax       },
    { "ts_dist_en",        "距离辅助",      "标靶流控", "",      BEAM_VAL_U8,  1u, 1u,   0.0f,    1.f, _bg_TsDistEn,        _bs_TsDistEn        },
    { "ts_dist_px_to_mm",  "dy→远近比例",   "标靶流控", "mm/px", BEAM_VAL_F32, 1u, 1u, -50.0f,   50.f, fGetTsDistPxToMm,    vSetTsDistPxToMm    },
    { "ts_dist_mm",        "距离辅助行程",  "标靶流控", "mm",    BEAM_VAL_F32, 1u, 0u,   1.0f,  100.f, fGetTsDistMm,        vSetTsDistMm        },

    /* ---- 平移射靶（激光固定在底盘; 2026-10-07 加, 语义见 app_config.h「平移射靶」段头）
     * 沿靶排**纵移**对齐（cx 偏差驱动 v_y, swap=1 本车）: 瞄准常数 + 自己的比例尺/方向/死区
     * （初值 = ts_chassis 那对"cx→纵移"现场标定值, 刻意独立不共用 —— 翻符号不连带改坏流控）;
     * 行走加减速复用 ta_accel/ta_speed, 帧新鲜度复用 ts_stale_ms。
     * ⚠️ 两个瞄准常数里只有 ta_xy_swap 选中的那个参与运算（本车=tc_cx_aim 瞄准列）;
     * 0=未标定 → 本步拒绝动车（不响, 留痕）。标定: 车摆"激光正打中间靶中心"位姿 → 发"6"读中间靶 cx。 ---- */
    { "tc_cx_aim",         "瞄准列(待标定)", "平移射靶", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1920.f, fGetTcCxAim,         vSetTcCxAim         },
    { "tc_cy_aim",         "瞄准行(swap=0用)", "平移射靶", "px", BEAM_VAL_F32, 1u, 0u,   0.0f, 1080.f, fGetTcCyAim,         vSetTcCyAim         },
    /* pi(CSI)那一套 —— 只有瞄准列/行独立（CSI 固定在底盘, 与机械臂上的主摄相对位置不同）;
     * 0=未标定 → 本步**自动按"只用主摄"发**（惰性, 不蜂鸣）。标定: 车摆"激光正打中间靶中心"
     * 位姿 → 跑一步看回帧/遥测 shoot_cam=1 → 读中间靶 cx 填 tc_cx_aim_pi。
     * ⚠️⚠️ **上界跟主摄那两根不一样**, 别照抄: 坐标系 = **Pi 侧取流的尺寸**（旋转会把
     *    宽高换个个儿）—— 2026-10-16 定稿: 取流 **2592×1944**（传感器原生满分辨率,
     *    帧率高必须 ≤15 ⇒ 现在 10）且 **PICAM_ROTATE=0（不转）** ⇒ 交付给识别的画面就是
     *    **2592×1944**（列 0~2592 / 行 0~1944）, 与主摄的 1920×1080 **不是一套**。
     *    ⚠️ 改 PICAM_ROTATE 或改取流尺寸 = **换坐标系**: 这两行的钳位范围与瞄准常数
     *    全部作废重标（别只改一边）。 */
    { "tc_cx_aim_pi",      "pi瞄准列(待标定)", "平移射靶", "px", BEAM_VAL_F32, 1u, 0u, 0.0f, 2592.f, fGetTcCxAimPi,       vSetTcCxAimPi       },
    { "tc_cy_aim_pi",      "pi瞄准行(swap=0用)", "平移射靶", "px", BEAM_VAL_F32, 1u, 0u, 0.0f, 1944.f, fGetTcCyAimPi,     vSetTcCyAimPi       },
    /* ⚠️ 自 2026-10-18 起这个名字只管**细调相位**（"7" 那股流）: 粗调那步固定用主摄
     *    （格距/瞄准列的尺是主摄像素标定的, pi 的 px/mm 约两倍 —— 见 app_rpi.c 粗调段）。
     *    1 = 细调优先 pi（起不来/掉帧时 Pi 整条退回主摄, 本步按回帧 bit7 换瞄准尺）;
     *    0 = 细调也走主摄（= 老方案, 调主摄参数/台架不动 pi 时用）。 */
    { "tc_use_picam",      "细调优先pi",     "平移射靶", "",     BEAM_VAL_U8,  1u, 1u,   0.0f,    1.f, _bg_TcUsePicam,      _bs_TcUsePicam      },
    { "tc_gap_mm",         "粗调格距",       "平移射靶", "mm",   BEAM_VAL_F32, 1u, 0u,   1.0f,  500.f, fGetTcGapMm,         vSetTcGapMm         },
    { "tc_gap_px",         "每格像素(待标定)", "平移射靶", "px", BEAM_VAL_F32, 1u, 0u,   0.0f, 1920.f, fGetTcGapPx,         vSetTcGapPx         },
    { "tc_px_to_mm",       "精调前移比例",   "平移射靶", "mm/px", BEAM_VAL_F32, 1u, 0u,   0.01f,    5.f, fGetTcPxToMm,        vSetTcPxToMm        },
    { "tc_sign",           "前移方向",       "平移射靶", "",     BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetTcSign,          vSetTcSign          },
    { "tc_db_mm",          "前移死区",       "平移射靶", "mm",   BEAM_VAL_F32, 1u, 0u,   0.0f,   50.f, fGetTcDbMm,          vSetTcDbMm          },
    { "tc_accel",          "平移加速度",     "平移射靶", "",     BEAM_VAL_I32, 1u, 0u,   0.0f,  255.f, _bg_TcAccel,         _bs_TcAccel         },
    { "tc_speed",          "平移速度",       "平移射靶", "rpm",  BEAM_VAL_I32, 1u, 0u,   0.0f, 65535.f, _bg_TcSpeed,        _bs_TcSpeed         },
    { "tc_vel_mode",       "微调模式",       "平移射靶", "",     BEAM_VAL_U8,  1u, 0u,   0.0f,    1.f, _bg_TcVelMode,       _bs_TcVelMode       },
    { "tc_max_mm",         "粗调上限",       "平移射靶", "mm",   BEAM_VAL_F32, 1u, 0u,   1.0f,  500.f, fGetTcMaxMm,         vSetTcMaxMm         },
    { "tc_conv_px",        "收敛阈值",       "平移射靶", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f,  100.f, fGetTcConvPx,        vSetTcConvPx        },
    { "tc_conv_ticks",     "收敛拍数",       "平移射靶", "",     BEAM_VAL_I32, 1u, 0u,   1.0f,   50.f, _bg_TcConvTicks,     _bs_TcConvTicks     },
    { "tc_kp_scale",       "增益系数",       "平移射靶", "",     BEAM_VAL_F32, 1u, 1u,   0.1f,    2.f, fGetTcKpScale,       vSetTcKpScale       },
    { "tc_step_mm",        "单拍上限",       "平移射靶", "mm",   BEAM_VAL_F32, 1u, 0u,   0.5f,  100.f, fGetTcStepMm,        vSetTcStepMm        },
    { "tc_travel_mm",      "行程闸",         "平移射靶", "mm",   BEAM_VAL_F32, 1u, 0u,   1.0f,  500.f, fGetTcTravelMm,      vSetTcTravelMm      },
    { "tc_ctrl_ms",        "控制节拍",       "平移射靶", "ms",   BEAM_VAL_I32, 1u, 0u,  20.0f,  500.f, _bg_TcCtrlMs,        _bs_TcCtrlMs        },
    /* 保持环运行期遥测（只读; 唯一写者 = ucRpiTargetShootHold）—— 射靶测试时在手机/PC 上
     * 能直接看偏差与状态, 不必去翻 Pi 日志的 SHOT n= 行。 */
    { "shoot_err",         "射靶保持偏差",   "平移射靶", "px",   BEAM_VAL_F32, 0u, 0u, -2000.f, 2000.f, fGetShootErr,       NULL                },
    /* ⚠️ 上界 6（不是 4）: AIM_STATE_NOTGT=5 / AIM_STATE_STALE=6 都要报得出来（2026-10-16 修，
     * 原来填 4 会把这两个态钳成"冻结"，屏上看到的是错的病因）。 */
    { "shoot_state",       "射靶保持状态",   "平移射靶", "",     BEAM_VAL_U8,  0u, 0u,   0.00f,    6.f, _bg_ShootState,      NULL                },
    { "shoot_cam",         "在用摄像头",     "平移射靶", "",     BEAM_VAL_U8,  0u, 0u,   0.00f,    1.f, _bg_ShootCam,       NULL                },

    /* ---- 爆爆桶对位标定（蓝牙可写; 初值 = app_config.h 的 BUCKET_ALIGN_*, 断电即回;
     *      2026-10-24 加。坐标系 = OpenMV **320×240**, 与射靶那几套全不是一套, 别抄值。
     *      标定顺序: ①车摆"正对爆爆桶"的理想位姿 → 跑一步(只测模式)读 bucket_err/
     *      "BKT meas:" 日志行 → 填 bc_cx_aim → ②前后挪 ±20mm 看 bucket_err 变大变小
     *      → 定 bc_sign → ③再跑一步应显示 skip（已在死区内）---- */
    { "bc_cx_aim",         "瞄准列(0=只测)",  "爆爆桶", "px",    BEAM_VAL_F32, 1u, 0u,   0.0f,  320.f, fGetBcCxAim,         vSetBcCxAim         },
    { "bc_cy_aim",         "瞄准行(swap=0用)", "爆爆桶", "px",   BEAM_VAL_F32, 1u, 0u,   0.0f,  240.f, fGetBcCyAim,         vSetBcCyAim         },
    { "bc_swap",           "轴映射",         "爆爆桶", "",      BEAM_VAL_U8,  1u, 0u,   0.0f,    1.f, _bg_BcSwap,          _bs_BcSwap          },
    { "bc_px_to_mm",       "比例尺(待标定)",  "爆爆桶", "mm/px", BEAM_VAL_F32, 1u, 0u,  0.01f,    5.f, fGetBcPxToMm,        vSetBcPxToMm        },
    { "bc_sign",           "纵移方向",       "爆爆桶", "",      BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetBcSign,          vSetBcSign          },
    { "bc_db_mm",          "纵移死区",       "爆爆桶", "mm",    BEAM_VAL_F32, 1u, 0u,   0.0f,   50.f, fGetBcDbMm,          vSetBcDbMm          },
    { "bc_accel",          "平移加速度",     "爆爆桶", "",      BEAM_VAL_I32, 1u, 0u,   0.0f,  255.f, _bg_BcAccel,         _bs_BcAccel         },
    { "bc_speed",          "平移速度",       "爆爆桶", "rpm",   BEAM_VAL_I32, 1u, 0u,   0.0f,65535.f, _bg_BcSpeed,         _bs_BcSpeed         },
    { "bc_vel_mode",       "微调模式",       "爆爆桶", "",      BEAM_VAL_U8,  1u, 0u,   0.0f,    1.f, _bg_BcVelMode,       _bs_BcVelMode       },
    { "bc_conv_px",        "收敛阈值",       "爆爆桶", "px",    BEAM_VAL_F32, 1u, 0u,   0.0f,  100.f, fGetBcConvPx,        vSetBcConvPx        },
    { "bc_conv_ticks",     "收敛拍数",       "爆爆桶", "",      BEAM_VAL_I32, 1u, 0u,   1.0f,   50.f, _bg_BcConvTicks,     _bs_BcConvTicks     },
    { "bc_kp_scale",       "增益系数",       "爆爆桶", "",      BEAM_VAL_F32, 1u, 1u,   0.1f,    2.f, fGetBcKpScale,       vSetBcKpScale       },
    { "bc_step_mm",        "单拍上限",       "爆爆桶", "mm",    BEAM_VAL_F32, 1u, 0u,   0.5f,  100.f, fGetBcStepMm,        vSetBcStepMm        },
    { "bc_travel_mm",      "行程闸",         "爆爆桶", "mm",    BEAM_VAL_F32, 1u, 0u,   1.0f,  500.f, fGetBcTravelMm,      vSetBcTravelMm      },
    { "bc_ctrl_ms",        "控制节拍",       "爆爆桶", "ms",    BEAM_VAL_I32, 1u, 0u,  20.0f,  500.f, _bg_BcCtrlMs,        _bs_BcCtrlMs        },
    { "bc_stale_ms",       "帧新鲜度",       "爆爆桶", "ms",    BEAM_VAL_I32, 1u, 0u, 100.0f, 3000.f, _bg_BcStaleMs,       _bs_BcStaleMs       },
    { "bc_meas_ms",        "只测模式时长",   "爆爆桶", "ms",    BEAM_VAL_I32, 1u, 0u, 500.0f,10000.f, _bg_BcMeasMs,        _bs_BcMeasMs        },
    /* 只读遥测 —— 标 bc_cx_aim（只测模式下看它）与排障就看这一个 */
    { "bucket_err",        "桶偏差",         "爆爆桶", "px",    BEAM_VAL_F32, 0u, 0u, -512.f,  512.f, fGetBucketErr,       NULL                },

    /* ---- 看路标定量（蓝牙可写; 初值 = app_config.h 的 ROAD_*, 断电即回）----
     * 标定顺序: ①车摆理想停车位 → 读 road_dx 填进 road_dx_ideal
     *          → ②前后各挪 ±50mm, 看 road_dx 变大还是变小 → 定 road_sign
     *          → ③再跑一步应显示 skip（已在死区内）。 */
    { "road_dx_ideal",     "理想Δx",         "看路",   "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1920.f, fGetRoadDxIdeal,     vSetRoadDxIdeal     },
    { "road_dx_ideal_rev", "反向看路理想Δx", "看路",   "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1920.f, fGetRoadDxIdealRev,  vSetRoadDxIdealRev  },
    { "road_scale",        "比例系数",       "看路",   "mm/px", BEAM_VAL_F32, 1u, 0u,  0.01f,    5.f, fGetRoadScale,       vSetRoadScale       },
    { "road_sign",         "方向",           "看路",   "",     BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetRoadSign,        vSetRoadSign        },
    { "road_db_mm",        "纵移死区",       "看路",   "mm",   BEAM_VAL_F32, 1u, 0u,   0.0f,   50.f, fGetRoadDeadband,    vSetRoadDeadband    },
    { "road_max_mm",       "单次最大位移",   "看路",   "mm",   BEAM_VAL_F32, 1u, 0u,   0.0f,  500.f, fGetRoadMaxMm,       vSetRoadMaxMm       },
    { "road_accel",        "对位加速度",     "看路",   "",     BEAM_VAL_I32, 1u, 0u,   0.0f,  255.f, _bg_RoadAccel,       _bs_RoadAccel       },
    { "road_speed",        "对位速度",       "看路",   "rpm",  BEAM_VAL_I32, 1u, 0u,   0.0f, 65535.f, _bg_RoadSpeed,      _bs_RoadSpeed       },
    /* ---- 横向那一路（Δy = 黄区**下边缘**的 y; 积木参数 use_y=1 才参与运算）----
     * ⚠️ 与上面纵向那一套**各自独立标定**（同一停车位、但两把尺的比例/方向都不同）。
     * 标定顺序: ①车横向挪 ±20mm, 看 road_dy 变大还是变小 → 定 road_sign_y
     *          → ②车摆理想停车位, 读 road_dy 填进 road_dy_ideal
     *          → ③再跑一步应显示 skip（已在死区内）。
     * ⚠️ road_max_mm（单次最大位移）是纵/横**共用**的一个安全钳位, 不另配。 */
    { "road_dy_ideal",     "理想Δy",         "看路",   "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1080.f, fGetRoadDyIdeal,     vSetRoadDyIdeal     },
    { "road_dy_ideal_rev", "反向看路理想Δy", "看路",   "px",   BEAM_VAL_F32, 1u, 0u,   0.0f, 1080.f, fGetRoadDyIdealRev,  vSetRoadDyIdealRev  },
    { "road_scale_y",      "横向比例系数",   "看路",   "mm/px", BEAM_VAL_F32, 1u, 0u,  0.01f,    5.f, fGetRoadScaleY,      vSetRoadScaleY      },
    { "road_sign_y",       "横向方向",       "看路",   "",     BEAM_VAL_F32, 1u, 0u,  -1.0f,    1.f, fGetRoadDySign,      vSetRoadDySign      },
    { "road_db_mm_y",      "横向死区",       "看路",   "mm",   BEAM_VAL_F32, 1u, 0u,   0.0f,   50.f, fGetRoadDyDeadband,  vSetRoadDyDeadband  },
    /* 饱和救援步距（2026-10-30 加）: 测到"尺子没数"的帧（①黄区贴画面左缘=横向全黄 ②use_y=1
     * 时下边缘被裁=蓝牙 road_dy=0）→ 车体系后退这段再重测一次, 不叠加世界坐标系。
     * ⚠️ 方向 = **road_sign**（与正常位移同一条；不再写死负值——写死负值在本车会朝篷布走）;
     *    后退后沉淀 ROAD_RESCUE_SETTLE_MS 再重发 "G"; 0=关闭（Δy 被裁照旧只 skip,
     *    Δx 饱和帧判失败）。 */
    { "road_rescue_mm",    "饱和救援步距",   "看路",   "mm",   BEAM_VAL_F32, 1u, 0u,   0.0f,  500.f, fGetRoadRescueMm,    vSetRoadRescueMm    },
    /* 只读遥测 —— 标 road_dx_ideal / road_dy_ideal 与排障就看这几个 */
    { "road_status",       "识别状态",       "看路",   "",     BEAM_VAL_U8,  0u, 0u,   0.0f,    1.f, _bg_RoadStatus,      NULL                },
    { "road_dx",           "实测Δx",         "看路",   "px",   BEAM_VAL_F32, 0u, 0u,   0.0f, 65535.f, _bg_RoadDx,         NULL                },
    { "road_dy",           "实测Δy",         "看路",   "px",   BEAM_VAL_F32, 0u, 0u,   0.0f, 65535.f, _bg_RoadDy,         NULL                },
    { "road_ex",           "左边缘X",        "看路",   "px",   BEAM_VAL_F32, 0u, 0u, -2000.f, 4000.f, _bg_RoadEx,         NULL                },
    { "road_ey",           "左边缘Y",        "看路",   "px",   BEAM_VAL_F32, 0u, 0u, -2000.f, 4000.f, _bg_RoadEy,         NULL                },
    { "road_flags",        "识别标志位",     "看路",   "",     BEAM_VAL_U8,  0u, 0u,   0.0f,  255.f, _bg_RoadFlags,      NULL                },
    { "road_spread",       "Δx散布",         "看路",   "px",   BEAM_VAL_U8,  0u, 0u,   0.0f,  255.f, _bg_RoadSpread,     NULL                },

    /* ---- 看路校准（2026-10 加; ang = 黄区直边图像方向角, 只随车航向变 —— 陀螺漂移的视觉基准。
     * 语义见 app_config.h「看路校准」段头。⚠️ 只测不改: 不动车不写航向, 先并排验证再谈接环。
     * 标定: 车摆基准位读 roadyaw_ang 填 roadyaw_ideal（哨兵 |值|>90, 不是 0!）
     * → 原地转已知角看 roadyaw_err 变化方向 → 反了翻 roadyaw_dir。 ---- */
    { "roadyaw_ideal",     "理想方向角",     "看路校准", "°",  BEAM_VAL_F32, 1u, 0u, -90.f,   90.f, fGetRoadYawIdeal,   vSetRoadYawIdeal    },
    { "roadyaw_dir",       "方向",           "看路校准", "",   BEAM_VAL_F32, 1u, 0u,  -1.f,    1.f, fGetRoadYawDir,     vSetRoadYawDir      },
    { "roadyaw_status",    "识别状态",       "看路校准", "",   BEAM_VAL_U8,  0u, 0u,   0.0f,    1.f, _bg_RoadYawStatus,  NULL                },
    { "roadyaw_ang",       "实测方向角",     "看路校准", "°",  BEAM_VAL_F32, 0u, 0u, -180.f,  180.f, _bg_RoadYawAng,     NULL                },
    { "roadyaw_err",       "航向偏差",       "看路校准", "°",  BEAM_VAL_F32, 0u, 0u, -180.f,  180.f, _bg_RoadYawErr,     NULL                },
    { "roadyaw_cols",      "拟合内点数",     "看路校准", "",   BEAM_VAL_U8,  0u, 0u,   0.0f,  255.f, _bg_RoadYawCols,    NULL                },
    { "roadyaw_rms",       "拟合残差",       "看路校准", "px", BEAM_VAL_F32, 0u, 0u,   0.0f,   25.5f, _bg_RoadYawRms,    NULL                },
    { "roadyaw_spread",    "角度散布",       "看路校准", "°",  BEAM_VAL_F32, 0u, 0u,   0.0f,    2.55f, _bg_RoadYawSpread, NULL                },
    { "roadyaw_flags",     "识别标志位",     "看路校准", "",   BEAM_VAL_U8,  0u, 0u,   0.0f,  255.f, _bg_RoadYawFlags,   NULL                },
    /* ---- 看路校准→陀螺漂移偏置（2026-10 加; 链路见 app_beam.h「陀螺漂移偏置」段）----
     * 每次跑都对齐到相机真值: bias = wrap180(原始航向 − err), 发布 yaw ≡ err（零点="放直"）。
     * 扣不扣由编译期 ROADYAW_APPLY_EN 定（现场不碰蓝牙）。---- */
    { "roadyaw_bias",      "对齐偏置",       "看路校准", "°",   BEAM_VAL_F32, 0u, 0u,-180.f,  180.f, _bg_RoadYawBias,    NULL                },

    /* ---- 组动作角度偏移（蓝牙可写; 初值 = app_config.h 的 ARM_GROUP_OFS_*, 断电即回）
     *      下发角 = 动作预设里写的角 + 本值。⚠️ 只作用于组动作 —— 拖滑杆($B,JOG)与夹爪不吃它,
     *      这样编辑模式永远是标定基准。s6 量程 0~360, 偏移上限放宽到 ±180。 ---- */
    { "arm_ofs_s1",        "组偏移S1",       "臂偏移", "°",  BEAM_VAL_F32, 1u, 0u, -90.0f,   90.f, _bg_ArmOfs0,          _bs_ArmOfs0            },
    { "arm_ofs_s2",        "组偏移S2",       "臂偏移", "°",  BEAM_VAL_F32, 1u, 0u, -90.0f,   90.f, _bg_ArmOfs1,          _bs_ArmOfs1            },
    { "arm_ofs_s3",        "组偏移S3",       "臂偏移", "°",  BEAM_VAL_F32, 1u, 0u, -90.0f,   90.f, _bg_ArmOfs2,          _bs_ArmOfs2            },
    { "arm_ofs_s4",        "组偏移S4",       "臂偏移", "°",  BEAM_VAL_F32, 1u, 0u, -90.0f,   90.f, _bg_ArmOfs3,          _bs_ArmOfs3            },
    { "arm_ofs_s6",        "组偏移S6",       "臂偏移", "°",  BEAM_VAL_F32, 1u, 0u, -180.0f, 180.f, _bg_ArmOfs4,          _bs_ArmOfs4            },

    /* ---- 机械臂动作（只读遥测）---- */
    { "arm_deployed",      "机械臂已展开",   "机械臂", "",    BEAM_VAL_U8,  0u, 0u,  0.00f,    1.f, _bg_ArmDeployed,       NULL                   },
    { "arm_seq_busy",      "臂序列在跑",     "机械臂", "",    BEAM_VAL_U8,  0u, 0u,  0.00f,    1.f, _bg_ArmSeqBusy,        NULL                   },
    /* 舵机运行时量（2026-10 加, setter 当场下发 —— 见 app_beam.h 那段说明）:
     * arm_speed_dps 调小=整臂更斯文（拖滑杆与 S 曲线削峰都受它管）, arm_curve 换 S 曲线
     * 的进度形状（0五次S/1三次S/2余弦/3线性, 拿一个 1s 的动作当场对比最直观）。 */
    { "arm_speed_dps",     "舵机速度闸",     "机械臂", "°/s", BEAM_VAL_F32, 1u, 0u,  30.0f, 1000.f, fGetArmSpeedDps,      vSetArmSpeedDps       },
    { "arm_curve",         "插值曲线",       "机械臂", "",    BEAM_VAL_U8,  1u, 0u,   0.0f,    3.f, _bg_ArmCurve,          _bs_ArmCurve          },
    { "script_running",    "脚本在跑",       "脚本", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    1.f, _bg_ScriptRunning,    NULL                   },

    /* ---- 脚本跳转（只读遥测）---- */
    { "script_preset_id",  "当前预设ID",     "脚本", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _bg_ScriptPresetId,    NULL                   },
    { "script_call_depth", "跳转深度",       "脚本", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _bg_ScriptCallDepth,   NULL                   },

    /* ---- 串口接收健康（只读遥测; 2026-09 加: 「Pi 说识别成功、单片机全超时」那类失效
     *      只有这两个数看得出来 —— rpi_rx_bytes 停涨而 TX 正常 = 本端 RX 已死）---- */
    { "rpi_rx_bytes",      "RPI收字节数",    "通信", "",      BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _bg_RpiRxBytes,        NULL                   },
    { "rpi_rx_err",        "RPI接收错误",    "通信", "",      BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _bg_RpiRxErr,          NULL                   },
    { "rpi_rx_rearm_fail", "RPI重装失败",    "通信", "",      BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _bg_RpiRxRearmFail,    NULL                   },
    /* ---- 串口发送健康（发送改 TX DMA 环后加）---- */
    { "rpi_tx_bytes",      "RPI发字节数",    "通信", "",      BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _bg_RpiTxBytes,        NULL                   },

    /* ---- 「等链路发任务」遥测（2026-09 加; 判据见 app_rpi.h 里 ucRpiSendTaskCmdWait 的注释）----
     * rpi_link 非 1 且 rpi_pend 非 0 且 rpi_pend_ms 在涨 = 有任务在等重连;
     * rpi_link_retry 涨 = 正在往复重启 Pi（奇数帧=卡在半对）;
     * rpi_pend 由非 0 变 0 而路线没往下走、rpi_pend_drop+1 = 被新任务顶掉了。 */
    { "rpi_pend",          "挂起任务码",     "通信", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _bg_RpiPend,           NULL                   },
    { "rpi_pend_ms",       "已等链路",       "通信", "ms",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _bg_RpiPendMs,         NULL                   },
    { "rpi_link_retry",    "重启Pi帧数",     "通信", "",      BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _bg_RpiLinkRetry,      NULL                   },
    { "rpi_pend_drop",     "挂起被顶次数",   "通信", "",      BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _bg_RpiPendDrop,       NULL                   },

    /* ---- 通信 / 菜单 ---- */
    { "rpi_link",          "RPI链路状态",    "通信", "",      BEAM_VAL_U8,  0u, 0u,  0.00f,    0.f, _beamRpiLink,          NULL                   },
    { "menu_index",        "菜单索引",       "菜单", "",      BEAM_VAL_I32, 1u, 0u,  0.00f,  100.f, _bg_MenuIndex,         _bs_MenuIndex          },
    { "att_page",          "姿态显示页",     "菜单", "",      BEAM_VAL_I32, 1u, 0u,  0.00f,    2.f, _bg_AttPage,           _bs_AttPage            },

    /* ---- 陀螺仪（两口 × 三设备 + 热备自动切换, 2026-10 扩展; 见 app_beam.h 该组说明）----
     * 组名从「M0陀螺」改成「陀螺仪」: 这一组现在管的是"两口三设备 + 选源 + 热备",
     * 拿一颗模块的名字当组名是撒谎。两个 U8 域恰为 0~1 ⇒ 「蓝牙」Tab 自动渲染成勾选框。
     * ⚠️ ucWarn=1: 运行中切源会让 fGetAttitudeYaw() 跳到另一颗的零点,
     *    正在做绝对航向闭环的步骤会把它当大误差 —— 先停车再切。 */
    { "gyro_src",          "陀螺仪首选",     "陀螺仪", "",    BEAM_VAL_U8,  1u, 1u,  0.00f,    1.f, _bg_GyroSrc,           _bs_GyroSrc,           "0=主口那颗|1=副口那颗" },
    { "gyro_src_eff",      "实际在用",       "陀螺仪", "",    BEAM_VAL_U8,  0u, 0u,  0.00f,    1.f, _bg_GyroSrcEff,        NULL,                  "0=主口那颗|1=副口那颗" },
    { "gyro_auto_switch",  "自动切换",       "陀螺仪", "",    BEAM_VAL_U8,  1u, 1u,  0.00f,    1.f, _bg_GyroAutoSwitch,    _bs_GyroAutoSwitch     },
    { "gyro_auto_sw",      "自动切换次数",   "陀螺仪", "",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _bg_GyroAutoSw,        NULL                   },
    { "gyro_main_frames",  "主口帧号",       "陀螺仪", "",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _bg_GyroFramesMain,    NULL                   },
    { "gyro_sub_frames",   "副口帧号",       "陀螺仪", "",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _bg_GyroFramesSub,     NULL                   },
    { "m0_yaw",            "M0原始航向",     "陀螺仪", "°",   BEAM_VAL_F32, 0u, 0u,  0.00f,    0.f, fGetM0Yaw,             NULL                   },
    { "m0_frames",         "M0帧号",         "陀螺仪", "",    BEAM_VAL_I32, 0u, 0u,  0.00f,    0.f, _beamM0Frames,         NULL                   },

    /* ---- OLED 菜单（2026-10 加）----
     * U8 域恰为 0~1 ⇒ 「蓝牙」Tab 自动渲染成**勾选框**（勾/去勾即写, 无"写入"按钮）。
     * ⚠️ 上电初值来自 app_config.h 的 OLED_MENU_ANIM_EN; 这里改了当场生效, 但**易失**
     *    （断电回宏值）—— 想把现场认定值固化就改「调参」Tab 那个宏再重编译。 */
    { "oled_menu_anim",    "换行动画",       "OLED", "",      BEAM_VAL_U8,  1u, 0u,  0.00f,    1.f, _bg_OledMenuAnim,      _bs_OledMenuAnim       },
};

const stBeamParamTdf *cpstGetBeamParamTable(uint16_t *pusCount)
{
    if (pusCount != NULL)
    {
        *pusCount = (uint16_t)(sizeof(s_astBeamParam) / sizeof(s_astBeamParam[0]));
    }
    return s_astBeamParam;
}

const stBeamParamTdf *cpstFindBeamParam(const char *pcName)
{
    uint16_t usI;

    if (pcName == NULL) { return NULL; }

    for (usI = 0; usI < (uint16_t)(sizeof(s_astBeamParam) / sizeof(s_astBeamParam[0])); usI++)
    {
        if (strcmp(s_astBeamParam[usI].pcName, pcName) == 0) { return &s_astBeamParam[usI]; }
    }
    return NULL;
}

uint8_t ucBeamApply(const char *pcName, float fVal)
{
    const stBeamParamTdf *pst = cpstFindBeamParam(pcName);

    if (pst == NULL)          { return 1u; }   /* 未找到 */
    if (pst->ucRW == 0u)      { return 2u; }   /* 只读 */
    /* 范围检查必须在调 pfSet 之前：否则 (uint8_t)300 会静默变成 44，参数被悄悄写坏 */
    if ((fVal < pst->fMin) || (fVal > pst->fMax)) { return 3u; }

    if (pst->pfSet != NULL) { pst->pfSet(fVal); }
    return 0u;
}
