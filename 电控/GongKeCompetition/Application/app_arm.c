/**
  * @file       app_arm.c
  * @brief      机械臂模块：初始化 6 轴舵机（SERVO1-5 为 180°，SERVO6 为 360°底座）。
  */

#include "app_arm.h"
#include "app_arm_action.h"
#include "app_beam.h"       /* 组动作角度偏移 fGetArmGroupOfs（台架标定, 蓝牙可调） */
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

/* ==================== 机械臂组联动（组 = SERVO1~4 + SERVO6 底座） ====================
 * pfAngles[5] 顺序与本表一致; 夹爪 SERVO5 不进组, 由 vArmSetZhuaShou 单独控制。 */
static const emServoDevNumTdf s_aemArmGroup[ARM_GROUP_NUM] = {
    ARM_SERVO1, ARM_SERVO2, ARM_SERVO3, ARM_SERVO4, ARM_SERVO6,
};

/* ==================== 上电默认姿态 ==================== */
/* 断电手摆到期望姿态 → 上电观察哪个关节在动 → 微调对应值，直到上电舵机纹丝不动。
 * 原理：初始化时先把 CCR 写成该角度对应脉宽再启动 PWM，上电第一帧输出的就是
 * 默认姿态的脉宽；手摆位置与表中角度差在舵机死区内（约±2°），舵机就不会动。 */
static const float s_fArmPoseInit[SERVO_DEV_NUM] = {
    88.0f,   /* SERVO1 180° 关节1 */
    14.0f,   /* SERVO2 180° 关节2 */
    11.0f,   /* SERVO3 180° 关节3 */
    90.0f,   /* SERVO4 180° 关节4 */
    120.6f,   /* SERVO5 180° 关节5 */
    12.0f,  /* SERVO6 360° 底座（0~360 中位） */
};

void vAppArmInit(void)
{
    uint8_t ucI;

    /* 180° 角度型舵机（初始化到上电默认姿态，先写 CCR 再启动 PWM，无 0 脉宽毛刺） */
    vServoDeviceInit_AngleAt(ARM_SERVO1, &htim4, TIM_CHANNEL_2, s_fArmPoseInit[0]);  /* PB7 */
    vServoDeviceInit_AngleAt(ARM_SERVO2, &htim2, TIM_CHANNEL_3, s_fArmPoseInit[1]);  /* PA2 */
    vServoDeviceInit_AngleAt(ARM_SERVO3, &htim2, TIM_CHANNEL_2, s_fArmPoseInit[2]);  /* PA1 */
    vServoDeviceInit_AngleAt(ARM_SERVO4, &htim2, TIM_CHANNEL_4, s_fArmPoseInit[3]);  /* PA3 */
    vServoDeviceInit_AngleAt(ARM_SERVO5, &htim3, TIM_CHANNEL_3, s_fArmPoseInit[4]);  /* PB0 */

    /* 360° 角度型舵机（底座，可像 180° 一样定位到 0~360° 任意角度） */
    vServoDeviceInit_Angle360At(ARM_SERVO6, &htim3, TIM_CHANNEL_4, s_fArmPoseInit[5]);  /* PB1 */

    /* 全臂速度闸（运行时量 arm_speed_dps, 上电初值 = ARM_SERVO_MAX_SPEED_DPS;
     * 蓝牙「机械臂」组 / 「舵机速度」积木可中途改, setter 当场下发）。
     * ⚠️ 必须放在上面所有初始化**之后** —— 那些 init 会把 fSpeed 复位成
     *    SERVO_DEFAULT_SPEED(1.0 = 1000°/s, 实际等于不限速)。
     * 它同时管两件事: 拖滑杆时的恒速趋近速度、S 曲线播放时的削峰上限。 */
    for (ucI = 0u; ucI < (uint8_t)SERVO_DEV_NUM; ucI++)
    {
        vServoSetSpeed((emServoDevNumTdf)ucI, fGetArmSpeedDps() / 1000.0f);
    }

    /* S 曲线插值形状（上电初值 = ARM_SERVO_CURVE; 蓝牙 arm_curve 可中途换） */
    vServoDeviceSetCurve((emServoCurveTdf)ucGetArmCurve());
}

/* 中途修改舵机速度（STEP_ARM_SPEED_SET 的原语; 见 app_arm.h） */
void vArmSetSpeedLimit(float fDps)
{
    vSetArmSpeedDps(fDps);        /* Beam setter: 存 + 当场下发全部舵机 */
}

/*
 * 机械臂任务：推进「启动即返回」的动作预设序列。
 *
 * 阻塞式的播放（STEP_ARM_PLAY_WAIT / $B,PLAY）在调用者自己的上下文里跑（脚本任务 /
 * vBtTask），不经过这里；只有投递到后台的序列需要有人替它往下走 —— 这就是
 * 这个任务存在的理由（它原本只是个 TODO 空壳）。
 * 泵内部（app_arm_action.c::vArmActionPump）: 当前那条跑完再看待播队列有没有下一条
 * （排队的两条会一条接一条跑完、`arm_seq_busy` 全程为 1），都跑空了才让出 20ms。
 * 投递来源有两个: vArmPlayNoWait（路线里那块大积木, 后写者胜）与
 * ucArmPlayByMode（排爆抓取/反恐任务里那几条动作, 追加排队 —— 标了「阻塞到位」的
 * 那一档由调用方在原地等队列跑空, 所以两种模式都经这里推进）。
 */
void vArmTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        vArmActionPump();
    }
}

/* ==================== 组动作（类似 pro 塔吊 nArmMoveAllWait/NoWait） ==================== */

/*
 * 5 轴同时启动（各自按角度差自动估时 S 曲线），立即返回。
 * 仿真 pro 塔吊的 NoWait：启动后不等到位，调用方可继续底盘动作，稍后自行等待。
 *
 * ⚠️ 这里是**全部组动作唯一的收口**（vArmGroupMoveWait / $B,ARM / 动作预设播放都经它），
 *    所以「组动作角度偏移」（Beam 的 arm_ofs_s*）也只在**这一处**叠加 —— 一处生效，
 *    避免在脚本、蓝牙、动作预设三处各写一遍而漏掉其中一个。
 *    按组顺序取偏移（0..4 → S1,S2,S3,S4,S6）, 与 pfAngles 一一对应。
 *
 * ⚠️ 刻意**不**给 $B,JOG（编辑模式拖滑杆）与夹爪加偏移：编辑模式拖出来的是标定基准,
 *    让它也吃偏移就成了「滑杆读数与物理位置互相追着跑」, 永远标不准。
 */
void vArmGroupMoveNoWait(const float *pfAngles)
{
    uint8_t ucI;

    for (ucI = 0; ucI < ARM_GROUP_NUM; ucI++)
    {
        const stServoDeviceParamTdf *pstDev =
            c_pstGetServoDeviceParam(s_aemArmGroup[ucI]);
        float fAngle = pfAngles[ucI] + fGetArmGroupOfs(ucI);

        /* 叠加偏移后仍钳在量程内。驱动自己也会钳, 但那样「偏移被截掉多少」是无声的 ——
         * 这里显式钳, 语义与 vArmAimByPixels 一致: 宁可少走一点也不越量程。 */
        if (fAngle < pstDev->stStaticParam.fValueMin) { fAngle = pstDev->stStaticParam.fValueMin; }
        if (fAngle > pstDev->stStaticParam.fValueMax) { fAngle = pstDev->stStaticParam.fValueMax; }

        vServoSetTargetValueAuto(s_aemArmGroup[ucI], fAngle);
    }
}

/* 全臂「最坏行程」耗时上界(ms) = max over 六轴 of 量程 / 速度闸。
 * ⚠️ 为什么要算它: 速度闸(ARM_SERVO_MAX_SPEED_DPS)调小后, 360° 那根轴走满程要的
 *    毫秒数会远超固定的 ARM_GROUP_WAIT_TIMEOUT_MS; 固定超时一到就放行, 等于把
 *    「轮询到位」又变回「估时放行」, 大行程动作会在半路被放掉（与 vArmSetZhuaShou
 *    的 ulBound 同一个道理）。速度闸没设(=0, 不限速)时回 0。 */
static uint32_t ulArmWorstTravelMs(void)
{
    uint32_t ulMax = 0u;
    uint8_t  ucI;

    for (ucI = 0u; ucI < (uint8_t)SERVO_DEV_NUM; ucI++)
    {
        const stServoDeviceParamTdf *pstDev =
            c_pstGetServoDeviceParam((emServoDevNumTdf)ucI);
        float fSpeed = pstDev->stRunningParam.fSpeed;                            /* °/ms */
        float fSpan  = pstDev->stStaticParam.fValueMax
                     - pstDev->stStaticParam.fValueMin;                          /* 全量程 */
        uint32_t ulT;

        if ((fSpeed <= 0.0001f) || (fSpan <= 0.0f)) { continue; }
        ulT = (uint32_t)(fSpan / fSpeed);
        if (ulT > ulMax) { ulMax = ulT; }
    }
    return ulMax;
}

/*
 * 组动作阻塞版: NoWait 启动 + 轮询全部舵机到位标志。
 * 与 pro 的"定时延时估时"不同, 轮询 done 标志天然准确 —— **前提是那个标志真的
 * 表示"到位"**（见 servo_device.c::ucServoIsSmoothStepDone: 被削速后的恒速收尾
 * 阶段必须算"还没到", 否则这里会提前放行, 整条路线跟着提前动）。
 * 超时上界取 max(ARM_GROUP_WAIT_TIMEOUT_MS, 最坏行程 + 余量) 后放行（防某轴卡死
 * 拖住整条路线; 配置值只是**下界**, 不该反过来切掉正常的大行程动作）。
 */
void vArmGroupMoveWait(const float *pfAngles)
{
    uint32_t ulWaited = 0;
    uint32_t ulWorst  = ulArmWorstTravelMs();
    uint32_t ulBound  = ARM_GROUP_WAIT_TIMEOUT_MS;

    if ((ulWorst > 0u) && ((ulWorst + 500u) > ulBound)) { ulBound = ulWorst + 500u; }

    vArmGroupMoveNoWait(pfAngles);

    while (!ucServoIsAllSmoothStepDone() && ulWaited < ulBound)
    {
        vTaskDelay(pdMS_TO_TICKS(5));   /* 让出式轮询，不饿死低优先级任务 */
        ulWaited += 5;
    }
}

/* ==================== 夹爪（SERVO5） ==================== */

/*
 * 夹爪开合: ucKaiHe 1=KAI(开) 0=HE(合); ucDegree 0~100 开合程度。
 * 语义: KAI d = 开到 d% 开度; HE d = 合掉 d% 行程（目标开度 = 100-d）。
 *   即 KAI 100=全开, KAI 0=全闭, HE 100=全闭, HE 30=合三成(剩70%开度)。
 * 角度 = ZHUASHOU_CLOSE_ANGLE..ZHUASHOU_OPEN_ANGLE 按开度线性插值。
 */
void vArmSetZhuaShouNoWait(uint8_t ucKaiHe, uint8_t ucDegree)
{
    float fAngle;
    uint8_t ucOpenPct;

    if (ucDegree > 100) ucDegree = 100;
    ucOpenPct = (ucKaiHe != 0) ? ucDegree : (uint8_t)(100 - ucDegree);

    fAngle = ZHUASHOU_CLOSE_ANGLE
           + (ZHUASHOU_OPEN_ANGLE - ZHUASHOU_CLOSE_ANGLE) * ucOpenPct / 100.0f;

    vServoSetTargetValueTimed(ARM_GRIPPER, fAngle, ZHUASHOU_MOVE_MS);
}

/*
 * 夹爪开合(阻塞版): 定长 S 曲线 + 等到位（抓放时序关键, 同 pro vSetZhuaShou）。
 * 脚本步骤 STEP_SET_ZHUASHOU_WAIT 用这一版; 蓝牙点动用 NoWait 版。
 */
void vArmSetZhuaShou(uint8_t ucKaiHe, uint8_t ucDegree)
{
    uint32_t ulWaited = 0u;
    uint32_t ulBound;

    vArmSetZhuaShouNoWait(ucKaiHe, ucDegree);

    /* 阻塞等到位（带超时保护, 防平滑时基停摆时永久卡死）。
     *
     * ⚠️ 上界**不能**按 ZHUASHOU_MOVE_MS 的固定倍数给。行程被全臂总速度闸
     *    (ARM_SERVO_MAX_SPEED_DPS) 削速时, 五次赫米特中段峰值 1.875·Δ/时长 远超闸值,
     *    驱动会在 S 曲线走完后切**恒速收尾**(vServoDevicePeriodExecute) —— 真实耗时是
     *    max(时长, Δ/速度), 不是时长。夹爪行程由 40° 放大到 180° 后, MOVE_MS*4 = 1200ms
     *    会把上界卡在行程中间, 脚本在夹爪还没合上时就往下走（抓取时序错乱, 表现为物件掉落）。
     * 故按「全量程最坏行程」算上界: 速度闸越小 / 行程越大, 上界自动跟着涨。
     * ⚠️ 上界只是防卡死的**兜底** —— 「什么时候算到位」由 ucServoIsSmoothStepDone 判,
     *    它必须把恒速收尾算成"还没到"（2026-09 修: 老写法只看 emMode, 收尾一开始就报
     *    完成, 上界给多宽都没用, 循环早就退出了）。 */
    {
        float fSpeed = c_pstGetServoDeviceParam(ARM_GRIPPER)->stRunningParam.fSpeed; /* °/ms */
        float fSpan  = ZHUASHOU_CLOSE_ANGLE - ZHUASHOU_OPEN_ANGLE;                   /* 全量程 */
        if (fSpan < 0.0f) { fSpan = -fSpan; }

        if (fSpeed > 0.0001f)
        {
            /* 恒速收尾(Δ/速度) + S 曲线时长 + 余量 */
            ulBound = (uint32_t)(fSpan / fSpeed) + ZHUASHOU_MOVE_MS + 200u;
        }
        else
        {
            ulBound = ZHUASHOU_MOVE_MS * 4u;   /* 没设速度闸时退回原退路 */
        }
    }

    while (!ucServoIsSmoothStepDone(ARM_GRIPPER) && ulWaited < ulBound)
    {
        vTaskDelay(pdMS_TO_TICKS(5));
        ulWaited += 5;
    }
    /* 停稳延时按脚本规则显式写进表里（STEP_DELAY）, 不藏在动作函数尾部 */
}

/* ==================== 靶标校准：按像素偏差微调瞄准轴 ====================
 *
 * 输入是**画面像素偏差**（靶心 − 瞄准点）; 输出是两根瞄准轴的角度增量。
 * 标定量全部走 Beam（「蓝牙」Tab 的「标靶校准」组, 2026-09 加）—— 进门取一次快照,
 * 半途被人从蓝牙改掉也不会算出前后不一致的一步。
 *
 * ⚠️ 两根轴的映射由 tg_xy_swap 决定（与 BALL_/HOSTAGE_XY_SWAP 同惯例: 轴+比例尺整套对调,
 *    方向符号各自仍走 tg_x_sign / tg_y_sign, 不跟着翻）。
 * ⚠️ **Δ角会被 tg_max_step_deg 夹一道**。像素偏差 → 角度是开环线性的, 靶心落在画面边缘时
 *    偏差可达 ±900px（×0.05°/px = 45°）, 而那一路坐标可能来自一次误检。夹住步长后,
 *    误检的代价被限制在"这一小步"; 收敛靠路线里把同一块积木连排几步迭代。
 */

void vArmAimByPixels(float fDxPx, float fDyPx)
{
    /* 画面X 那一路的 (舵机, 比例尺, 符号); 画面Y 那一路同理。开 swap 就整体对调。 */
    emServoDevNumTdf emDevX = ARM_SERVO6;
    emServoDevNumTdf emDevY = ARM_SERVO3;
    const stServoDeviceParamTdf *pstX;
    const stServoDeviceParamTdf *pstY;
    float    fScaleX;
    float    fScaleY;
    float    fSignX;
    float    fSignY;
    float    fStepX;
    float    fStepY;
    float    fCurX;
    float    fCurY;
    float    fAngX;
    float    fAngY;
    int32_t  iMoveMs;
    int32_t  iSlackMs;
    float    fSpan;
    float    fTmp;
    float    fPick;
    uint32_t ulWaited = 0u;
    uint32_t ulBound;

    /* ---- 进门取一次标定量快照（都是蓝牙可改的 Beam 量） ---- */
    if (ucGetTgXySwap() != 0u) { emDevX = ARM_SERVO3; emDevY = ARM_SERVO6; }
    fScaleX  = (emDevX == ARM_SERVO6) ? fGetTgPxToDegX() : fGetTgPxToDegY();
    fScaleY  = (emDevY == ARM_SERVO3) ? fGetTgPxToDegY() : fGetTgPxToDegX();
    fSignX   = (emDevX == ARM_SERVO6) ? fGetTgXSign()    : fGetTgYSign();
    fSignY   = (emDevY == ARM_SERVO3) ? fGetTgYSign()    : fGetTgXSign();
    iMoveMs  = iGetTgMoveMs();
    iSlackMs = iGetTgMoveSlackMs();

    /* ⚠️ 时长下限: 驱动的 `vServoSetTargetValueTimed(dur=0)` 是**零时长旁路** ——
     * 它直接把 fCurrentValue 写成目标（舵机以自身最高速甩过去, 砸齿轮）, 而这正是
     * 工程反复禁止的那条路（$B,JOG 的 ms=0 专门走恒速趋近就是为它）。
     * 蓝牙 / 调参 Tab 都能把 tg_move_ms 写成 0（量程 @(0~)）—— 这里兜住,
     * 同族的 ts_move_ms 在调参 Tab 里限的是 @(20~1000)。 */
    if (iMoveMs < 20) { iMoveMs = 20; }

    pstX = c_pstGetServoDeviceParam(emDevX);
    pstY = c_pstGetServoDeviceParam(emDevY);
    fCurX = pstX->stRunningParam.fCurrentValue;
    fCurY = pstY->stRunningParam.fCurrentValue;

    /* ---- 像素偏差 → 角度增量, 先各自夹到 ±tg_max_step_deg ---- */
    fStepX = fDxPx * fScaleX * fSignX;
    fStepY = fDyPx * fScaleY * fSignY;
    fPick  = fGetTgMaxStepDeg();
    if (fStepX >  fPick) { fStepX =  fPick; }
    if (fStepX < -fPick) { fStepX = -fPick; }
    if (fStepY >  fPick) { fStepY =  fPick; }
    if (fStepY < -fPick) { fStepY = -fPick; }

    fAngX = fCurX + fStepX;
    fAngY = fCurY + fStepY;

    /* 钳位到量程。用**静态参数**而不是 SERVO_DEFAULT_ANGLE_* 宏 —— 静态参数是运行时
     * 单一事实源（同 ucArmFillJointTable 的理由）, 换初始化方式也不会失配。
     * 宁可少走一点也绝不把舵机写到量程外。 */
    if (fAngX < pstX->stStaticParam.fValueMin) { fAngX = pstX->stStaticParam.fValueMin; }
    if (fAngX > pstX->stStaticParam.fValueMax) { fAngX = pstX->stStaticParam.fValueMax; }
    if (fAngY < pstY->stStaticParam.fValueMin) { fAngY = pstY->stStaticParam.fValueMin; }
    if (fAngY > pstY->stStaticParam.fValueMax) { fAngY = pstY->stStaticParam.fValueMax; }

    vArmSeqAbort();     /* 新臂指令: 先掐掉后台的「启动即返回」序列, 免得两边抢舵机 */

    vServoSetTargetValueTimed(emDevX, fAngX, (uint32_t)iMoveMs);
    vServoSetTargetValueTimed(emDevY, fAngY, (uint32_t)iMoveMs);

    /* 阻塞等到位。⚠️ 上界要覆盖「被总速度闸削速后」的真实行程, 不能只按 MOVE_MS
     *    的固定倍数 —— S 曲线中段峰值 1.875·Δ/时长 远超闸值时, 驱动会切**恒速收尾**,
     *    真实耗时是 max(时长, Δ/速度)。（同 vArmSetZhuaShou 那段踩过的坑。） */
    fSpan = fAngX - fCurX;
    if (fSpan < 0.0f) { fSpan = -fSpan; }
    fTmp = fAngY - fCurY;
    if (fTmp < 0.0f) { fTmp = -fTmp; }
    if (fTmp > fSpan) { fSpan = fTmp; }

    if (pstX->stRunningParam.fSpeed > 0.0001f)
    {
        ulBound = (uint32_t)(fSpan / pstX->stRunningParam.fSpeed)
                  + (uint32_t)iMoveMs + (uint32_t)iSlackMs;
    }
    else
    {
        ulBound = (uint32_t)iMoveMs * 4u;
    }

    while ((!ucServoIsSmoothStepDone(emDevX) || !ucServoIsSmoothStepDone(emDevY))
           && (ulWaited < ulBound))
    {
        vTaskDelay(pdMS_TO_TICKS(5));
        ulWaited += 5;
    }
    /* 停稳延时按脚本规则显式写进表里（STEP_DELAY）, 不藏在动作函数尾部 */
}

/* ==================== 点动关节表 ==================== */

/*
 * 点动关节表: 5 项, 顺序 = 组顺序 SERVO1,2,3,4,6（与 afF[0..4] 一一对应）。
 * 范围**从舵机静态参数取**, 不用 SERVO_DEFAULT_ANGLE_* 宏——静态参数是运行时
 * 单一事实源, 换初始化方式(180°/360°)也不会失配。
 * 填进调用方自己的缓冲（模块内不留可变静态量, 免得又变成一处隐藏共享状态）,
 * 返回实际写入条数。
 */
uint8_t ucArmFillJointTable(stArmJointTdf *pastOut, uint8_t ucCapacity)
{
    static const char *const s_apcArmJointName[ARM_GROUP_NUM] =
        { "s1", "s2", "s3", "s4", "s6" };
    uint8_t ucI;

    if ((pastOut == NULL) || (ucCapacity < ARM_GROUP_NUM)) { return 0u; }

    for (ucI = 0; ucI < ARM_GROUP_NUM; ucI++)
    {
        const stServoDeviceParamTdf *pstParam = c_pstGetServoDeviceParam(s_aemArmGroup[ucI]);

        pastOut[ucI].pcName = s_apcArmJointName[ucI];
        pastOut[ucI].emDev  = s_aemArmGroup[ucI];
        pastOut[ucI].fMin   = pstParam->stStaticParam.fValueMin;
        pastOut[ucI].fMax   = pstParam->stStaticParam.fValueMax;
    }
    return ARM_GROUP_NUM;
}
