/**
  * @file       app_pid.c
  * @brief      角度 PID 闭环实现（移植自 pro 项目 Hardware/pid.c）。
  *
  *             适配点（SPL 裸机 → HAL/FreeRTOS）:
  *               TIM4 50ms ISR → vPidTask 以 vTaskDelayUntil 50ms 周期驱动；
  *               反馈 g_fAngle[2] → fGetAttitudeYaw()（Beam, ±180°）；
  *               IMU 卡死检测 ucImuScan 的新鲜标志 → ucTakeYawFresh()；
  *               去掉急停轮询(vEmergencyPoll)/OLED 调试显示；
  *               阻塞等待 HAL_Delay → vTaskDelay（不饿死低优先级任务）。
  *               注意：周期处理内部有阻塞 UART 发送与 1ms 级延时，必须留在
  *               任务上下文（这也是从 TIM5 中断迁出的原因）。
  */

#include "app_pid.h"
#include "app_chassis.h"
#include "app_beam.h"
#include "app_sensors.h"     /* vGyroSrcWatchdogTick（陀螺源热备看门狗）*/
#include "app_config.h"      /* main.h → HAL 外设 */
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* 全局 PID 控制器实例（文件内私有，无跨模块 extern） */
static stPidTdf  s_stPid = {0};

/* ucPidMove() 使用的底盘加速度，vPidSetAccel() 可覆盖 */
static int32_t s_iPidDefaultAccel = 230;

/* PID 运行时状态 —— vPidPeriodic50ms 中频繁读写 */
typedef struct
{
    float    fFeedback;         /* 当前航向角反馈(°) */
    float    fTarget;           /* 目标航向角(°) */
    int      nVx, nVy, nW;      /* 底盘速度分量(rpm) */
    int      nFlag;             /* 完成标志: 0=运行中, 1=已完成 */
    uint32_t ulDelayTime;       /* 本次运动持续时间(ms) */
    uint32_t ulRealTime;        /* 已运行时间(ms) */
    uint8_t  ucMotorEnable;     /* 电机使能 */
    uint8_t  ucPidEnable;       /* PID 闭环使能 */
    uint8_t  ucCircle;          /* 调试计数(未用) */
    float    fImuStallCnt;      /* IMU 卡死计数(连续无新航向次数) */
} stPidRuntimeTdf;

/* PID 收敛模式参数 */
typedef struct
{
    uint8_t  ucStrategy;        /* 0=纯超时, 1=角度收敛 */
    float    fThreshold;        /* 收敛角度阈值(°) */
    uint8_t  ucSamples;         /* 连续稳定采样数 */
    uint8_t  ucCount;           /* 当前连续稳定计数 */
    uint32_t ulTimeout;         /* 收敛模式安全超时(ms) */
} stPidConvergeTdf;

static stPidRuntimeTdf  s_stRun  = {0};
static stPidConvergeTdf s_stConv = {0};

/* ==================== PID 参数组表（下标 = PID_MODE_*） ==================== */

typedef struct
{
    float fKp, fKi, fKd;
    float fMaxOut;      /* 0 = 不覆盖 maxOutput */
} stPidParamTdf;

static const stPidParamTdf s_astPidParam[] =
{
    { PID_CALIBRATE_KP, PID_CALIBRATE_KI, PID_CALIBRATE_KD, 0.0f                 }, /* 0 PID_MODE_CALIBRATE */
    { PID_BACK_OFF_KP,  PID_BACK_OFF_KI,  PID_BACK_OFF_KD,  PID_BACK_OFF_MAX_OUT }, /* 1 PID_MODE_BACK_OFF  */
    { PID_ROUGH_KP,     PID_ROUGH_KI,     PID_ROUGH_KD,     0.0f                 }, /* 2 PID_MODE_ROUGH     */
    { PID_FINE_KP,      PID_FINE_KI,      PID_FINE_KD,      0.0f                 }, /* 3 PID_MODE_FINE      */
    { PID_HOLD_KP,      PID_HOLD_KI,      PID_HOLD_KD,      0.0f                 }, /* 4 PID_MODE_HOLD      */
    { PID_ARC_TURN_KP,  PID_ARC_TURN_KI,  PID_ARC_TURN_KD,  PID_ARC_TURN_MAX_OUT }  /* 5 PID_MODE_ARC_TURN  */
};

/* 按模式号装载 PID 参数（越界回落到 0 号校准组；fMaxOut=0 的组不覆盖限幅） */
static void vPidLoadParams(int iMode)
{
    if (iMode < 0 || iMode >= (int)(sizeof(s_astPidParam) / sizeof(s_astPidParam[0])))
    {
        iMode = PID_MODE_CALIBRATE;
    }
    s_stPid.kp = s_astPidParam[iMode].fKp;
    s_stPid.ki = s_astPidParam[iMode].fKi;
    s_stPid.kd = s_astPidParam[iMode].fKd;
    if (s_astPidParam[iMode].fMaxOut > 0.0f)
    {
        s_stPid.maxOutput = s_astPidParam[iMode].fMaxOut;
    }
    else
    {
        /* fMaxOut=0 的组恢复默认限幅（2026-10-09 修）: ucPidMove 收尾会把 maxOutput 抬到
         * POST_MOVE(230), 不恢复的话 CALIBRATE/ROUGH/FINE/HOLD 四组从此**永久继承 230**
         * —— 同组同误差的运动力度依赖调用历史（首动 180、之后 230）。 */
        s_stPid.maxOutput = PID_DEFAULT_MAX_OUTPUT;
    }
}

/* ==================== getter/setter ==================== */

void vPidSetMotorEnable(uint8_t ucVal)
{
    s_stRun.ucMotorEnable = ucVal;
}

void vPidSetPidEnable(uint8_t ucVal)
{
    s_stRun.ucPidEnable = ucVal;
}

void vPidClearIntegral(void)
{
    s_stPid.integral = 0.0f;
}

void vPidConvergeInit(uint8_t ucStrategy, float fThreshold,
                      uint8_t ucSamples, uint32_t ulTimeout)
{
    s_stConv.ucStrategy = ucStrategy;
    s_stConv.fThreshold = fThreshold;
    s_stConv.ucSamples  = ucSamples;
    s_stConv.ulTimeout  = ulTimeout;
}

void vPidSetAccel(int32_t iVal)
{
    s_iPidDefaultAccel = iVal;
}

/* ==================== PID 核心 ==================== */

static void vPidInitParam(stPidTdf *pstPid, float p, float i, float d, float maxI, float maxOut)
{
    pstPid->kp = p;
    pstPid->ki = i;
    pstPid->kd = d;
    pstPid->maxIntegral = maxI;
    pstPid->maxOutput = maxOut;
}

/* 一次 PID 计算（位置式）。角度误差归一化到 (-180°, +180°]，自动取短弧方向。
 * ucFirstDeriv = 1（本次运动的**第一拍**）→ 微分项强制为 0: 上一次运动**结束时**遗留的
 * error 与本次运动毫无关系, 拿它算微分会给一个与当前误差无关的旋转分量（起步抖一下）。
 * 支点转弯 / 原地转弯都做了同样的首拍抑制, 只有这条老路漏了。 */
static void vPidCalc(stPidTdf *pstPid, float reference, float feedback, uint8_t ucFirstDeriv)
{
    pstPid->lastError = pstPid->error;

    if (reference - feedback < -180) {
        pstPid->error = reference - feedback + 360;
    } else if (reference - feedback > 180) {
        pstPid->error = reference - feedback - 360;
    } else {
        pstPid->error = reference - feedback;
    }

    if (ucFirstDeriv != 0u) { pstPid->lastError = pstPid->error; }

    float dout = (pstPid->error - pstPid->lastError) * pstPid->kd;
    float pout = pstPid->error * pstPid->kp;
    pstPid->integral += pstPid->error * pstPid->ki;

    if (pstPid->integral > pstPid->maxIntegral)
        pstPid->integral = pstPid->maxIntegral;
    else if (pstPid->integral < -pstPid->maxIntegral)
        pstPid->integral = -pstPid->maxIntegral;

    pstPid->output = pout + dout + pstPid->integral;

    if (pstPid->output > pstPid->maxOutput)
        pstPid->output = pstPid->maxOutput;
    else if (pstPid->output < -pstPid->maxOutput)
        pstPid->output = -pstPid->maxOutput;
}

/* 设定单次 PID 运动参数（不启动，由 vPidPeriodic50ms 检测到后驱动） */
static void vPidSetMotion(int v_x, int v_y, int w, uint32_t time, float target)
{
    /* 五字段是一条完整「运动指令」, 读侧是 vPidTask（50ms 节拍）——裸写的话装载窗口里
     * 周期任务可能读到「新 ulDelayTime + 旧 ulRealTime」的组合, 判成"已完成" → 本次
     * 运动一拍没跑就返回。短临界区包成原子（区内只有 store, 2026-10-09 修）。 */
    taskENTER_CRITICAL();
    s_stRun.nVx = v_x;
    s_stRun.nVy = v_y;
    s_stRun.nW  = w;
    s_stRun.ulDelayTime = time;
    s_stRun.ulRealTime  = 0;
    s_stRun.fTarget     = target;
    taskEXIT_CRITICAL();
}

/* 校准偏置换算: 修正系(闭环/显示用) → 原始系(世界航向记录用)。
 * ⚠️⚠️ **世界坐标系刻意不受看路校准影响**（2026-10-08）: vChassisMoveWorld 查表的
 *    world_heading 必须一直活在开机原始坐标系里 —— 校准的瞬间若把世界系一起旋转,
 *    跨校准记录/消费的「路径残留偏移」会错轴。所以闭环咬的航向用修正值（劫持）,
 *    而这里写 world_heading 时换算回原始值。 */
static float fPidWorldYaw(float fCorrected)
{
#if (ROADYAW_APPLY_EN != 0)
    return fCorrected + fGetRoadYawBias();
#else
    return fCorrected;
#endif
}

/* ==================== 初始化 ==================== */

void vPidInit(void)
{
    vPidInitParam(&s_stPid,
             PID_DEFAULT_KP, PID_DEFAULT_KI, PID_DEFAULT_KD,
             PID_DEFAULT_MAX_INTEGRAL, PID_DEFAULT_MAX_OUTPUT);
    vSetWorldHeading(0.0f);
}

void vAppPidInit(void)
{
    vPidSetMotorEnable(1);   /* 先使能电机，防止周期处理误驱动 */
    vPidInit();

#if PID_STRATEGY == PID_STRATEGY_CONVERGE
    vPidConvergeInit(PID_STRATEGY_CONVERGE, PID_CONVERGE_THRESHOLD,
                    PID_CONVERGE_SAMPLES, PID_CONVERGE_SAFETY_MS);
#else
    vPidConvergeInit(PID_STRATEGY_TIMEOUT, 0.0f, 0, 0);
#endif
}

/* ==================== 运动控制接口 ==================== */

uint8_t ucPidMove(int v_x, int v_y, int w, uint32_t time, float target, int pid_choose)
{
    s_stRun.nFlag = 0;
    vPidClearIntegral();
    vChassisSetAccel(s_iPidDefaultAccel);

    if (s_stConv.ucStrategy == PID_STRATEGY_CONVERGE) {
        s_stConv.ucCount = 0;
        time = s_stConv.ulTimeout;
    }

    s_stRun.fImuStallCnt = 0;

    vPidLoadParams(pid_choose);

    /* 目标角度归一化到 (-180°, +180°] */
    if (target > 180) {
        target = target - 360;
    } else if (target < -180) {
        target = target + 360;
    }

    vPidSetMotion(v_x, v_y, w, time, target);

    /* 阻塞等待：vPidTask 周期处理判定到位后置 nFlag=1
     * （vTaskDelay 让出式等待，不饿死按键/OLED/传感器任务） */
    {
        uint32_t timeout_ms = (time > 0) ? (time * 2 + 1000) : 5000;
        uint32_t waited = 0;
        while (s_stRun.nFlag == 0 && waited < timeout_ms) {
            vTaskDelay(pdMS_TO_TICKS(1));
            waited++;
        }
        if (s_stRun.nFlag == 0) {
            vChassisMove(0, 0, 0);
        }
    }
    s_stRun.nFlag = 0;

    s_stPid.maxOutput = PID_POST_MOVE_MAX_OUTPUT;

    /* 记录指令航向（原始系）供 vChassisMoveWorld 查表 —— **永远按指令目标写, 不写实测**
     * （2026-10-09 与车主确认的设计原则, 别再"修"成实测; 自转/支点转弯收尾同款）:
     *   · 全工程只用 90° 档转弯, map 里的"80 多度"是**标定值** —— 物理结果就是 90°。
     *     世界航向是 90° 逻辑网格的记账器: 它存在的意义是排路线时不用想"现在该沿哪根轴走",
     *     按 网格=0/90/180/-90 思考即可;
     *   · 写实测反而让每一弯的真实残差渗进累加器, 逻辑网格被一点点扭歪;
     *   · 查表本身按 ±45° 就近取整, 几度的残差天然被吸收 —— 记目标与记实测落同一个象限;
     *   · IMU 卡死有双陀螺仪热备兜底（app_sensors 的源看门狗）, 不为它改记账语义。 */
    vSetWorldHeading(fPidWorldYaw(target));
    vTaskDelay(pdMS_TO_TICKS(100));
    return 1;
}

void vPidMoveNoWait(int v_x, int v_y, int w, uint32_t time, float target, int pid_choose)
{
    s_stRun.nFlag = 0;
    vPidLoadParams(pid_choose);
    vPidSetMotion(v_x, v_y, w, time, target);
}

/* 底盘直线运动 + 角度闭环：速度按勾股分配到两轴，时间按 PID_MOVE_MS_PER_MM 估算 */
void PID_vChassisMoveDistance(float x, float y, float angel, float speed)
{
    float fDist = sqrtf(x * x + y * y);
    if (fDist < 1e-3f) {
        return;                       /* 零位移：无速度分量可分配 */
    }
    float fenliang = speed / fDist;
    uint32_t time = (uint32_t)(fDist * PID_MOVE_MS_PER_MM);
    vChassisSetAccel(0);
    ucPidMove((int)(x * fenliang), (int)(y * fenliang), 0, time, angel,
              PID_MODE_BACK_OFF);
}

/* ==================== 边走边转包装层（移植 pro Motion.c） ==================== */

/* 原地转到目标航向: CALIBRATE 粗调 → FINE 精调 两段（pro vPidTurn） */
void vPidTurn(float fTargetAngle)
{
    ucPidMove(0, 0, 0, PID_TIMEOUT_CALIBRATE_MS, fTargetAngle, PID_MODE_CALIBRATE);
    ucPidMove(0, 0, 0, PID_TIMEOUT_FINE_MS,      fTargetAngle, PID_MODE_FINE);
}

/* 边走边转: BACK_OFF(限幅30rpm) 平移+纠偏 → FINE 停下消差（pro vPidMoveAndTurn） */
void vPidMoveAndTurn(int v_x, int v_y, float fTargetAngle)
{
    ucPidMove(v_x, v_y, 0, PID_TIMEOUT_TRANSIT_MS,   fTargetAngle, PID_MODE_BACK_OFF);
    ucPidMove(0, 0,  0, PID_TIMEOUT_TRANSIT_FINE_MS, fTargetAngle, PID_MODE_FINE);
}

/* 回退回正: 小速度后退同时把航向拉回目标, 再停稳（pro vBackToRoad） */
void vBackToRoad(float fTargetAngle)
{
    ucPidMove(CHASSIS_BACK_OFF_SPEED, 0, 0, PID_TIMEOUT_BACKOFF_MS,
              fTargetAngle, PID_MODE_BACK_OFF);
    vTaskDelay(pdMS_TO_TICKS(200));
}

/* ==================== 支点转弯（单侧双轮 + 陀螺仪闭环） ==================== */

/**
  * @brief       支点转弯核心: 只驱动一侧双轮、另一侧当支点, IMU 航向闭环转到目标角
  * @param       iDir        >0 = 左转(逆时针), <=0 = 右转; 决定驱动哪一侧
  * @param       fAngleDeg   转角幅值(°), 方向由 iDir 给定（负值按绝对值处理）
  * @param       fHandoffDeg >0 = **交接模式**: |误差| ≤ 本值就提前松手退出（给巡线接管用）;
  *                          0 = 关（= 历史行为, 转到 PIVOT_TURN_CONVERGE_DEG 收敛带）
  * @param       ulLineFrame0 发 LINE_START **之前**取的 ulGetLineFrameCount()。
  *                          ⚠️ **交接必须同时满足「误差进阈值」与「帧计数已推进」** ——
  *                          这一条就是"起流等待与转弯重叠"的全部实现: 转弯照转, 每拍顺带
  *                          看一眼流起没起, 两个条件都齐的当拍才交。若先等满起流再转弯,
  *                          总时长 = 起流等待 + 转弯, **一点没省**（真踩过）。
  *                          fHandoffDeg=0 时本参数无意义（传 0 即可）。
  * @retval      1 = 因交接提前退出; 0 = 正常到位 / 超时 / IMU 故障 / 被中止 / 角度太小
  *
  * 物理方向写死在 vChassisPivotDrive: 左转驱动右侧 {2,4} 正转、左侧 {1,3} 持 0;
  * 右转反之。只有"驱动侧"出力, 另一侧是支点。
  *
  * 与四轮 vPidTurn 的差异（刻意不共用 stPid / s_stRun）:
  *   1) 局部 PID 状态 + 本上下文按 PIVOT_TURN_PERIOD_MS 阻塞循环, 不碰 s_stPid/s_stRun。
  *      空闲的 vPidPeriodic50ms 既进不了控制分支也不写电机（见其 nFlag/ucPidEnable 判据）,
  *      故两者不会抢电机指令; IMU 新鲜标志同理（PID 任务只在控制分支里读它）。
  *   2) 循环退出即状态失效, 不留积分残差给下一次。
  *   3) 空闲的 PID 任务唯一可能写电机的路径是 vPidPeriodic50ms 的"一次性停车"
  *      (s_ucStopSent): 仅 CONVERGE 策略下, 上一个 ucPidMove 提前收敛返回后的那一拍
  *      会补发一次 vChassisMove(0,0,0), 可能与本闭环撞一拍 —— 后果只是该拍驱动丢失
  *      (下一拍自动重发), 且当前 PID_STRATEGY=TIMEOUT 时 ucPidMove 退出前已停车, 不发生。
  *
  * ±180° 突变处理（yaw 到边界会从 +180 跳到 -180）:
  *   目标航向 = 当前航向 + 带符号转角, 先归一化到 (-180°, +180°];
  *   每拍误差再按"短弧"归一化（|err|>180 就 ∓360）—— 与 vPidCalc 同一判据。
  *   于是跨 ±180° 的转弯（如 170°→-170°）误差始终是真实的小角度,
  *   不会出现 359° 的假误差把车朝反方向甩。
  *
  * ==================== 交接模式（fHandoffDeg > 0, 给 STEP_PIVOT_LINE_WAIT 用） ====================
  * 转弯误差进 fHandoffDeg 就提前退出, 由巡线接管（同时修位置 vx 与航向 w）。
  * 与"完整转到目标角"的两处**刻意不同**（别照抄回去）:
  *   1) **不停车**: 交接路径**不调** vChassisMove(0,0,0) —— 驱动侧残留转速由巡线的
  *      第一拍直接覆盖, 省一次"停稳再起步"。但 vChassisSetAccel(iAccSaved) **照旧要调**:
  *      vChassisMove 每个电机指令都现读 iGetChassisAccel(), 不恢复的话巡线段会一直用
  *      PIVOT_TURN_ACCEL(=0, 不插值), 与"转弯跑完再进巡线"的老两步路径行为不一致。
  *   2) 世界航向由**尾段按源**收尾（ucRpiPivotTurnLine → s_ucTurnLineTail）: 陀螺档写
  *      指令目标（90° 网格）, 相机/融合写实测（线才是航向基准）; 转弯本体仍按指令目标
  *      记账（见函数尾）。
  * ⚠️ **交接要先"武装"**: 进循环前先量一次初始误差, 只有 |初值| > fHandoffDeg 才启用 ——
  *    否则「相对转 5°」而阈值 10° 时会首拍就 |err| ≤ 阈值、**一步不转就松手**,
  *    与"相对转 5°"的语义正好相反。绝对模式下"已在目标附近"同理（那属于没事可做,
  *    照旧走收敛带, 代价 200ms）。同时要求 fHandoffDeg > PIVOT_TURN_CONVERGE_DEG,
  *    不然收敛带先于交接触发 → 永不交接（下面那道武装门一并管住）。
  * ⚠️ 这道武装门**只看误差、不看帧计数**: 首拍时流几乎不可能已经起来, 把帧判据混进来
  *    等于这道保护永不生效。
  * ⚠️ 循环里查中止位（收尾 = 停车, 航向仍按指令目标记账, 与 IMU 卡死同语义）。
  *    本步把"转弯"与"等起流"合到了一起, 不可中止的话一次能堵好几秒 —— 同为等外部
  *    事件的循环, 与 vLineFollowConverge 同一条纪律。
  */
static uint8_t s_ucPivotTurnCore(int iDir, float fAngleDeg, uint8_t ucAbsolute,
                                 float fHandoffDeg, uint32_t ulLineFrame0, uint8_t ucRev)
{
    float    fMag = (fAngleDeg >= 0.0f) ? fAngleDeg : -fAngleDeg;
    int      iLeft = (iDir > 0) ? 1 : 0;
    int      iSide = iLeft ? +1 : -1;      /* 左转: 右侧 {2,4}; 右转: 左侧 {1,3} */
    /* 反向执行（ucRev=1, 2026-11-01 加; 用户定案「**对侧驱动 + 速度取反**」）: 目标角与
     * 转向**不变**（"左转"仍逆时针转那么多）, 改成**对侧**双轮**倒转**当驱动 —— 支点随之
     * 换到对侧、车心弧线镜像（左转: 正常右轮前进 ⇒ 反向左轮后退, 车心由前扫变后扫）。
     * 车倒着放/场地换边跑镜像路线时勾, 复现正向放置的世界轨迹。
     * ⚠️ **对侧与速度必须同时取反**: 对侧倒转的 yaw 响应与正常侧正转**同号**（左轮退 =
     * 车头左偏 = yaw 增, 同右轮进）, 闭环照旧负反馈, 增益/收敛判据/世界航向记账全不用动。
     * 只翻其中一个（同侧倒转 / 对侧正转）都会把 yaw 响应翻号 → 正反馈飞车到超时。 */
    const int   iDriveSide = (ucRev != 0u) ? -iSide : iSide;  /* 反向: 驱动侧换对侧 */
    const float fRevSpd    = (ucRev != 0u) ? -1.0f : 1.0f;    /* 反向: 驱动速度取反 */
    float    fDelta, fTarget, fErr, fLastErr = 0.0f, fIntegral = 0.0f, fOut;
    uint32_t ulElapsed = 0;
    int      iStable = 0;
    int      iStall  = 0;
    uint8_t  ucFirst = 1u;                  /* 首拍抑制微分冲击 */
    uint8_t  ucHandoffEn = 0u;              /* 交接已"武装"（本次真的要转、且初值在阈值之外） */
    uint8_t  ucHandoff = 0u;                /* 因交接提前退出（不是到位也不是超时） */
    int32_t  iAccSaved;

    if (ucAbsolute != 0u)
    {
        /* 绝对模式: fAngleDeg 就是**目标 IMU 航向**（归一化到 (-180,180]）。
         * 方向位 iDir 不参与 —— 往哪转、转多少全由「目标 − 当前」的最短路径决定,
         * 于是「转到世界 90°」这种写法不再受前面积累的朝向误差影响。
         * 也不做小角度早退: 那判的是"转角太小", 绝对模式没有转角这个概念;
         * 已在目标附近时下面的收敛带会立刻吃掉它。
         * 反向执行不动目标角 —— 转向不变, 只换驱动侧与速度符号（见上面 fRevSpd 段）。 */
        fTarget = fAngleDeg;
    }
    else
    {
        if (fMag < PIVOT_TURN_MIN_DEG) {
            return 0u;                     /* 相对模式·角度太小: 不动车（也不改世界航向） */
        }

        /* 目标航向 = 当前航向 + 带符号转角（左转方向 × 陀螺符号标定） */
        fDelta  = (iLeft ? +1.0f : -1.0f) * fMag * PIVOT_TURN_YAW_SIGN;
        fTarget = fGetAttitudeYaw() + fDelta;
    }

    while (fTarget > 180.0f)  { fTarget -= 360.0f; }   /* 归一化到 (-180,180] */
    while (fTarget < -180.0f) { fTarget += 360.0f; }

    /* 交接「武装」: 只有本次**真的要转**、且初始误差在阈值**之外**才启用（理由见函数头）。
     * 这一道门同时管住了「阈值 ≤ PIVOT_TURN_CONVERGE_DEG」（收敛带会先触发, 永不交接）
     * 与「相对转角本身小于阈值」两种退化情形。 */
    if (fHandoffDeg > PIVOT_TURN_CONVERGE_DEG)
    {
        float fInit = fTarget - fGetAttitudeYaw();
        if (fInit > 180.0f)       { fInit -= 360.0f; }
        else if (fInit < -180.0f) { fInit += 360.0f; }
        if (fabsf(fInit) > fHandoffDeg) { ucHandoffEn = 1u; }
    }

    iAccSaved = iGetChassisAccel();
    vChassisSetAccel(PIVOT_TURN_ACCEL);    /* 闭环自己平滑, 驱动侧默认不插值 */

    while (ulElapsed < PIVOT_TURN_TIMEOUT_MS)
    {
        /* 中止位: 本步把"转弯"与"等起流"合到了一起, 不可中止能堵好几秒（见函数头）。
         * 放循环体第一行 —— 既与 vLineFollowConverge 同形, 也保证按停之后不再出这一拍力。 */
        if (ucGetScriptAbortReq() != 0u) { break; }

        /* 反馈 → 短弧误差（±180° 突变在这一步被吃掉） */
        fErr = fTarget - fGetAttitudeYaw();
        if (fErr > 180.0f)       { fErr -= 360.0f; }
        else if (fErr < -180.0f) { fErr += 360.0f; }

        /* 交接: **误差进阈值 且 巡线帧已起流**的当拍才松手（见函数头）。
         * **排在收敛带判断之前** —— 阈值必然大于 PIVOT_TURN_CONVERGE_DEG（上面那道武装门
         * 保证）, 于是交接模式下"停下等 4 拍稳定窗"那段**永远不会执行**, 这就是"交接收尾
         * 不停车"。第二个条件（帧计数）就是起流与转弯的重叠: 帧没来就继续转, 不干等。 */
        if ((ucHandoffEn != 0u)
            && (fabsf(fErr) <= fHandoffDeg)
            && (ulGetLineFrameCount() != ulLineFrame0))
        {
            ucHandoff = 1u;
            break;                     /* 本拍**不出力**: 上一拍转速留给巡线第一拍覆盖 */
        }

        if (fabsf(fErr) <= PIVOT_TURN_CONVERGE_DEG) {
            /* 进入收敛带: 停住等稳定窗（带内继续出力必冲过头, 驱动侧有惯性） */
            if (++iStable >= (int)PIVOT_TURN_CONVERGE_N) {
                break;                     /* 到位 */
            }
            vChassisPivotDrive(iDriveSide, 0);
        } else {
            iStable = 0;

            if (ucFirst) { fLastErr = fErr; ucFirst = 0; }   /* 首拍微分项为 0 */
            fIntegral += fErr * PIVOT_TURN_KI;
            if (fIntegral > PIVOT_TURN_MAX_INTEGRAL)       { fIntegral = PIVOT_TURN_MAX_INTEGRAL; }
            else if (fIntegral < -PIVOT_TURN_MAX_INTEGRAL) { fIntegral = -PIVOT_TURN_MAX_INTEGRAL; }

            fOut = fErr * PIVOT_TURN_KP + (fErr - fLastErr) * PIVOT_TURN_KD + fIntegral;
            fLastErr = fErr;
            if (fOut > PIVOT_TURN_MAX_SPEED)       { fOut = PIVOT_TURN_MAX_SPEED; }
            else if (fOut < -PIVOT_TURN_MAX_SPEED) { fOut = -PIVOT_TURN_MAX_SPEED; }

            /* 输出 → 驱动侧前进/后退: 该侧"前进"对 yaw 的作用 = iSide × 陀螺符号,
             * 与误差同号才是在朝目标转（左转 = 右侧正转, 见 vChassisPivotDrive）。
             * 反向执行: 驱动侧换对侧(iDriveSide)且速度取反(fRevSpd) —— yaw 响应与正向
             * 同号（对侧倒转 = 本侧正转的等效转角方向）, 闭环稳定性不受影响。 */
            vChassisPivotDrive(iDriveSide,
                               (int)(fOut * (float)iSide * PIVOT_TURN_YAW_SIGN * fRevSpd));
        }

        /* IMU 卡死检测: 与 vPidPeriodic50ms 同一判据（空闲的 PID 任务不读该信号, 不抢）。
         * 卡死退出也按指令目标记账 —— 双陀螺仪热备下这层只是保险, 见函数尾的收尾注释。 */
        if (!ucTakeYawFresh()) {
            if (++iStall >= PID_IMU_STALL_LIMIT) { break; }
        } else {
            iStall = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(PIVOT_TURN_PERIOD_MS));
        ulElapsed += PIVOT_TURN_PERIOD_MS;
    }

    if (ucHandoff == 0u)
    {
        vChassisMove(0, 0, 0);             /* 四轮清速（含支点侧的保持力, 一次停干净） */
    }
    /* ⚠️ 交接路径**故意不上面那句**: 驱动侧残留转速交给巡线的第一拍覆盖（省一次停稳再起步）。
     * 而加速度**两条路都要恢复** —— vChassisMove 每个电机指令都现读 iGetChassisAccel(),
     * 不恢复的话巡线段会一直用 PIVOT_TURN_ACCEL(=0, 不插值), 与老两步路径行为不一致。 */
    vChassisSetAccel(iAccSaved);

    /* 记指令航向供 vChassisMoveWorld 查表 —— **永远按指令目标写**（90° 逻辑网格的记账器,
     * 完整理由见 ucPidMove 收尾的设计原则注释; 2026-10-09 按该原则统一, 别再改回实测）。
     * 交接路径说明: 交接发生在巡线已接管对准的中途, 最终航向由 ucRpiPivotTurnLine 的
     * 尾段**按源**收尾（陀螺档写指令目标, 相机/融合写实测 —— 线才是航向基准, 不在 90°
     * 网格的记账范围里）; 中止/IMU 卡死后路线已停, 记账值不再被消费。 */
    {
        float fHeading = fPidWorldYaw(fTarget);
        while (fHeading > 180.0f)  { fHeading -= 360.0f; }
        while (fHeading < -180.0f) { fHeading += 360.0f; }
        vSetWorldHeading(fHeading);
    }

    return ucHandoff;
}

/* 公开入口 1: 完整转到目标角（**历史行为, 交接关** —— 既有路线走这条, 一字不变）。
 * fHandoffDeg=0 → 交接资格恒 0, 交接分支全短路, 帧计数参数无意义（传 0）。
 * ucRev=1: 反向执行（对侧驱动+速度取反, 转向/目标角不变 —— 见内核头注释）; 传 0 = 原行为。 */
void vPidPivotTurn(int iDir, float fAngleDeg, uint8_t ucAbsolute, uint8_t ucRev)
{
    (void)s_ucPivotTurnCore(iDir, fAngleDeg, ucAbsolute, 0.0f, 0u, ucRev);
}

/* 公开入口 2: 转到交接阈值就提前松手（STEP_PIVOT_LINE_WAIT 用, 由巡线接管）。
 * ulLineFrame0 = 发 LINE_START **之前**取的帧计数 —— 帧推进了才允许交接（起流与转弯重叠）。
 * 返回 1 = 交接发生了（调用方接着跑巡线闭环）, 0 = 走完了完整转弯 / 被中止 / IMU 故障。
 * ⚠️ 交接版**暂不带反向**（ucRev 恒 0）—— 反向的「转弯→巡线」要连巡线侧一起镜像,
 *    有那个需求时再透传进来, 别只翻转弯半段。 */
uint8_t vPidPivotTurnUntil(int iDir, float fAngleDeg, uint8_t ucAbsolute,
                           float fHandoffDeg, uint32_t ulLineFrame0)
{
    return s_ucPivotTurnCore(iDir, fAngleDeg, ucAbsolute, fHandoffDeg, ulLineFrame0, 0u);
}

/* ==================== 原地转弯（四轮绕中心自转 + 陀螺仪闭环） ==================== */

/**
  * @brief       原地转弯: 四个麦轮对称反向出力, 绕车身中心自转, IMU 航向闭环转到目标角
  * @param       iDir       >0 = 左转(逆时针), <=0 = 右转
  * @param       fAngleDeg  转角幅值(°), 方向由 iDir 给定（负值按绝对值处理）
  * @retval      None
  *
  * 出力走 vChassisMove(0, 0, w) —— 麦轮解算在 vx=vy=0 时给出 speed[1..4]=∓w 两组反向,
  * 配合电机 1/2 反装即绕中心自转（w>0 = 逆时针 = 左转, 权威定义见 app_chassis.h）。
 * 与支点转弯(vPidPivotTurn)的差异:
 *   1) 四轮都出力, 车心基本不动（支点转弯只驱动单侧, 车心会平移）;
 *   2) 四轮对转靠辊子打滑, 同样的轮速下角速度约为支点的 2 倍 → 增益与 PIVOT_TURN_* 不可互换。
 *
 * 收尾航向: 与 ucPidMove/支点转弯统一 —— **永远按指令目标写**。世界航向是 90° 逻辑网格的
 * 记账器（map 的"80 多度"是标定值, 物理结果就是 90°）, 写实测会让每弯残差渗进累加器;
 * 查表按 ±45° 取整, 几度残差天然被吸收。IMU 卡死有双陀螺仪热备兜底。
 * （2026-10-09 按该原则改回; 原先"超时/卡死写实测"的那版是审查期改的, 语义见 ucPidMove 收尾。）
  *
 * 符号: SPIN_TURN_YAW_SIGN 同时进"目标角"与"出力"两处（与支点转弯的用法同构）——
 * 命令"左转"始终映射到固定的物理动作 w>0, SIGN 负责在 IMU 的 yaw 帧与物理帧之间来回翻译;
 * SIGN=+1 断言"左转 → yaw 增大", 装反了改 -1, 两处自动一起翻（不是补偿关系, 不用改两处）。
  *
  * ±180° 突变处理与支点转弯同: 目标角先归一化到 (-180°, +180°], 每拍误差再按短弧归一化,
  * 否则跨 ±180° 会出现 359° 的假误差把车朝反方向甩。
  * 仅任务上下文调用（内部 vTaskDelay + vChassisMove 都要任务上下文）。
  *
  * 「转弯→巡线交接」变体（vPidSpinTurnUntil, 2026-10-30 加, 与支点的 s_ucPivotTurnCore 同构）:
  * 自转闭环里每拍顺带看一眼巡线帧计数, **误差进交接阈值 且 帧已起流**的当拍提前松手
  * （本拍**不出力**, 残留自转转速由巡线第一拍的整体覆盖吃掉 —— 自转是四轮对称的, 巡线环
  * 下发的 vChassisMove(fVx, fY, fW) 覆盖全部四轮, 与支点交接的"驱动侧残留"同理）。于是
  * Pi 起流那几百毫秒被转弯吃掉, 且交接不停车（同 ucRpiPivotTurnLine 的"重叠"理由 ——
  * ⚠️ 别写成"先等满起流再转"的串行版, 那是这一套机制唯一要防的倒退）。
  * ⚠️ 交接要先"武装"（与支点同一道门）: 只有 |初始误差| > fHandoffDeg 才启用 ——
  *    否则「相对转 5°」而阈值 10° 时首拍就满足、一步不转就松手。武装门同时保证
  *    fHandoffDeg > SPIN_TURN_CONVERGE_DEG（收敛带永远晚于交接触发, "停下等稳定窗"
  *    那段在交接模式下不可达 —— 交接收尾不停车）。
  * ⚠️ 交接路径**不停车但恢复加速度**（同支点: vChassisMove 每拍现读 iGetChassisAccel(),
  *    不恢复的话巡线段会一直用 SPIN_TURN_ACCEL(=0, 不插值)）; 世界航向**仍按指令目标写**
  *    —— 交接后最终航向由 ucRpiSpinTurnLine 尾段**按源**收尾（陀螺档=目标, 相机/融合=实测）。
  * ⚠️ 自转的角速度约支点的 2 倍（差速自转 ω=2v/轮距）: 同一个交接阈值下, 交接瞬间的
  *    残留角速度也约是支点的 2 倍 —— 巡线环第一拍要多修一些; 阈值嫌松就调小
  *    PIVOT_LINE_HANDOFF_DEG（刻意共用, 没有为自转单开旋钮 —— 少一个要标定的数）。
  */
static uint8_t s_ucSpinTurnCore(int iDir, float fAngleDeg, uint8_t ucAbsolute,
                                float fHandoffDeg, uint32_t ulLineFrame0)
{
    float    fMag = (fAngleDeg >= 0.0f) ? fAngleDeg : -fAngleDeg;
    int      iLeft = (iDir > 0) ? 1 : 0;
    float    fDelta, fTarget, fErr, fLastErr = 0.0f, fIntegral = 0.0f, fOut;
    uint32_t ulElapsed = 0;
    int      iStable = 0;
    int      iStall  = 0;
    uint8_t  ucFirst = 1;                  /* 首拍抑制微分冲击 */
    uint8_t  ucHandoffEn = 0;              /* 交接已"武装"（本次真的要转、且初值在阈值之外） */
    uint8_t  ucHandoff = 0;                /* 因交接提前退出（不是到位也不是超时） */
    int32_t  iAccSaved;

    if (ucAbsolute != 0u)
    {
        /* 绝对模式: fAngleDeg = 目标 IMU 航向; 方向位不参与（同 vPidPivotTurn 的说明） */
        fTarget = fAngleDeg;
    }
    else
    {
        if (fMag < SPIN_TURN_MIN_DEG) {
            return 0u;                     /* 相对模式·角度太小: 不动车（也不改世界航向） */
        }

        /* 目标航向 = 当前航向 + 带符号转角（左转方向 × 陀螺符号标定） */
        fDelta  = (iLeft ? +1.0f : -1.0f) * fMag * SPIN_TURN_YAW_SIGN;
        fTarget = fGetAttitudeYaw() + fDelta;
    }

    while (fTarget > 180.0f)  { fTarget -= 360.0f; }   /* 归一化到 (-180,180] */
    while (fTarget < -180.0f) { fTarget += 360.0f; }   /* 负侧 2026-10-30 重构时被顶掉, 补回与
                                                        * 支点核同构（当前每拍误差有独立短弧
                                                        * 归一化兜底, 这里是防后人照支点核加
                                                        * 「直接用 fTarget」的逻辑时踩空） */

    /* 交接「武装」: 只有本次**真的要转**、且初始误差在阈值**之外**才启用 ——
     * 与 s_ucPivotTurnCore 的同一道门, 完整理由见那边的函数头（防「相对转 5°」一步不转
     * 就松手; 防收敛带先于交接触发）。只看误差、不看帧计数（首拍时流几乎不可能已起）。 */
    if (fHandoffDeg > SPIN_TURN_CONVERGE_DEG)
    {
        float fInit = fTarget - fGetAttitudeYaw();
        if (fInit > 180.0f)       { fInit -= 360.0f; }
        else if (fInit < -180.0f) { fInit += 360.0f; }
        if (fabsf(fInit) > fHandoffDeg) { ucHandoffEn = 1u; }
    }

    iAccSaved = iGetChassisAccel();
    vChassisSetAccel(SPIN_TURN_ACCEL);     /* 闭环自己平滑, 驱动侧默认不插值 */

    while (ulElapsed < SPIN_TURN_TIMEOUT_MS)
    {
        /* 中止出口（2026-10 补, 工程铁律「等外部事件的循环都要带」—— 这里漏过:
         * 不查的话运行中按菜单停止/投新任务, 最坏要干等 SPIN_TURN_TIMEOUT_MS(4s)
         * 超时才在步骤边界让位。中止后路线已被叫停, 航向记账值不再被消费。 */
        if (ucGetScriptAbortReq() != 0u)
        {
            break;
        }

        /* 反馈 → 短弧误差（±180° 突变在这一步被吃掉） */
        fErr = fTarget - fGetAttitudeYaw();
        if (fErr > 180.0f)       { fErr -= 360.0f; }
        else if (fErr < -180.0f) { fErr += 360.0f; }

        /* 交接: **误差进阈值 且 巡线帧已起流**的当拍才松手 —— 与支点的同一套判据,
         * 排在收敛带判断之前（武装门保证阈值 > SPIN_TURN_CONVERGE_DEG, 于是交接模式下
         * "停下等 4 拍稳定窗"那段不可达 = 交接收尾不停车）。本拍**不出力**: 残留自转
         * 转速留给巡线第一拍的整体覆盖吃掉（四轮对称自转, 覆盖是天然的 —— 见函数头）。 */
        if ((ucHandoffEn != 0u)
            && (fabsf(fErr) <= fHandoffDeg)
            && (ulGetLineFrameCount() != ulLineFrame0))
        {
            ucHandoff = 1u;
            break;
        }

        if (fabsf(fErr) <= SPIN_TURN_CONVERGE_DEG) {
            /* 进入收敛带: 四轮停住等稳定窗（带内继续出力必冲过头, 驱动侧有惯性） */
            if (++iStable >= (int)SPIN_TURN_CONVERGE_N) {
                break;                     /* 到位 */
            }
            vChassisMove(0, 0, 0);
        } else {
            iStable = 0;

            if (ucFirst) { fLastErr = fErr; ucFirst = 0; }   /* 首拍微分项为 0 */
            fIntegral += fErr * SPIN_TURN_KI;
            if (fIntegral > SPIN_TURN_MAX_INTEGRAL)       { fIntegral = SPIN_TURN_MAX_INTEGRAL; }
            else if (fIntegral < -SPIN_TURN_MAX_INTEGRAL) { fIntegral = -SPIN_TURN_MAX_INTEGRAL; }

            fOut = fErr * SPIN_TURN_KP + (fErr - fLastErr) * SPIN_TURN_KD + fIntegral;
            fLastErr = fErr;
            if (fOut > SPIN_TURN_MAX_SPEED)       { fOut = SPIN_TURN_MAX_SPEED; }
            else if (fOut < -SPIN_TURN_MAX_SPEED) { fOut = -SPIN_TURN_MAX_SPEED; }

            /* 输出 → 角速度: 四轮对称反向自转, w>0 = 逆时针。SIGN 把"IMU 误差帧"翻成
             * "物理转动帧"（与支点转弯 fOut × iSide × SIGN 是同一个道理） */
            vChassisMove(0, 0, (int)(fOut * SPIN_TURN_YAW_SIGN));
        }

        /* IMU 卡死检测: 与 vPidPeriodic50ms 同一判据（空闲的 PID 任务不读该信号, 不抢）。
         * 卡死退出也按指令目标记账 —— 双陀螺仪热备下这层只是保险, 见函数尾的收尾注释。 */
        if (!ucTakeYawFresh()) {
            if (++iStall >= PID_IMU_STALL_LIMIT) { break; }
        } else {
            iStall = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(SPIN_TURN_PERIOD_MS));
        ulElapsed += SPIN_TURN_PERIOD_MS;
    }

    if (ucHandoff == 0u)
    {
        vChassisMove(0, 0, 0);                 /* 四轮清速 */
    }
    /* ⚠️ 交接路径**故意不停车**: 残留自转转速交给巡线的第一拍覆盖（省一次停稳再起步）。
     * 而加速度**两条路都要恢复** —— 与支点交接的收尾同一取舍（见 s_ucPivotTurnCore 尾部）。 */
    vChassisSetAccel(iAccSaved);           /* 恢复底盘加速度, 不把 0 留给后面的速度模式步骤 */

    /* 记指令航向供 vChassisMoveWorld 查表 —— 永远按指令目标写（90° 逻辑网格的记账器,
     * 完整理由见 ucPidMove 收尾与本函数头注释; 2026-10-09 按该原则从"超时/卡死写实测"改回。
     * 交接路径说明: 与支点同 —— 交接发生在巡线已接管对准的中途, 最终航向由
     * ucRpiSpinTurnLine 尾段**按源**收尾（陀螺档=目标, 相机/融合=实测）; 中止/IMU 卡死
     * 后路线已停, 记账值不再被消费。） */
    {
        float fHeading = fPidWorldYaw(fTarget);
        while (fHeading > 180.0f)  { fHeading -= 360.0f; }
        while (fHeading < -180.0f) { fHeading += 360.0f; }
        vSetWorldHeading(fHeading);
    }

    return ucHandoff;
}

/* 公开入口 1: 完整转到目标角（**历史行为, 交接关** —— 既有路线走这条, 一字不变）。
 * fHandoffDeg=0 → 交接资格恒 0, 交接分支全短路, 帧计数参数无意义（传 0）。 */
void vPidSpinTurn(int iDir, float fAngleDeg, uint8_t ucAbsolute)
{
    (void)s_ucSpinTurnCore(iDir, fAngleDeg, ucAbsolute, 0.0f, 0u);
}

/* 公开入口 2: 转到交接阈值就提前松手（STEP_SPIN_LINE_WAIT 用, 由巡线接管）。
 * 与 vPidPivotTurnUntil 同形同义 —— ulLineFrame0 = 发 LINE_START **之前**取的帧计数,
 * 帧推进了才允许交接（起流与转弯的重叠就在这个判据里）。
 * 返回 1 = 交接发生了（调用方接着跑巡线闭环）, 0 = 走完了完整转弯 / 被中止 / IMU 故障。 */
uint8_t vPidSpinTurnUntil(int iDir, float fAngleDeg, uint8_t ucAbsolute,
                          float fHandoffDeg, uint32_t ulLineFrame0)
{
    return s_ucSpinTurnCore(iDir, fAngleDeg, ucAbsolute, fHandoffDeg, ulLineFrame0);
}

/* ==================== 周期处理（原 vPidIsrHandler） ==================== */

void vPidPeriodic50ms(void)
{
    /* 始终读取当前航向（无论是否在 PID 运动中） */
    s_stRun.fFeedback = fGetAttitudeYaw();

    static uint8_t s_ucStopSent = 1;

    /* 正常 PID 控制阶段 */
    if (s_stRun.ulDelayTime > 0 && s_stRun.ulRealTime <= s_stRun.ulDelayTime
        && s_stRun.ucMotorEnable) {
        s_ucStopSent = 0;

        /* 本次运动的第一拍（ulRealTime 由 vPidSetMotion 置 0）→ 抑制微分冲击 */
        vPidCalc(&s_stPid, s_stRun.fTarget, s_stRun.fFeedback,
                 (s_stRun.ulRealTime == 0u) ? 1u : 0u);

        /* 收敛模式：误差连续 N 次 < 阈值则提前退出 */
        if (s_stConv.ucStrategy == PID_STRATEGY_CONVERGE) {
            float abs_err = (s_stPid.error >= 0.0f) ? s_stPid.error : -s_stPid.error;
            if (abs_err <= s_stConv.fThreshold) {
                s_stConv.ucCount++;
                if (s_stConv.ucCount >= s_stConv.ucSamples) {
                    vChassisMove(0, 0, 0);
                    s_stRun.nFlag = 1;
                    s_stRun.ulRealTime = s_stRun.ulDelayTime + 1;
                    return;
                }
            } else {
                s_stConv.ucCount = 0;
            }
        }

        /* IMU 卡死检测：传感器任务每帧 vSetYawFresh，这里读后清零 */
        if (!ucTakeYawFresh()) {
            s_stRun.fImuStallCnt += 1.0f;
            if (s_stRun.fImuStallCnt >= PID_IMU_STALL_LIMIT) {
                vChassisMove(0, 0, 0);
                s_stRun.nFlag = 1;
                s_stRun.ulRealTime = s_stRun.ulDelayTime + 1;
                return;
            }
        } else {
            s_stRun.fImuStallCnt = 0.0f;
        }

        /* 驱动底盘：平移速度 + PID 输出角速度 */
        vChassisMove(s_stRun.nVx, s_stRun.nVy, s_stPid.output);

        s_stRun.nFlag = 0;

    /* 停止阶段：到点必停车（仅首次）
     * ⚠️ 2026-10 起不看 ucPidEnable: 旧版 ucPidEnable=1 时控制/停车两个分支都不执行
     *    → 电机保持末速, 直到 ucPidMove 的超时兜底才停（多跑约 time+1s）。
     *    「PID 闭环使能」的语义应是**出力方式**, 不该是「到点要不要停」。 */
    } else {
        if (!s_ucStopSent) {
            vChassisMove(0, 0, 0);
            s_ucStopSent = 1;
        }
        s_stRun.nFlag = 1;
    }

    s_stRun.ulRealTime += 50;
}

/* ==================== 周期任务 ==================== */

/*
 * PID 周期任务：vTaskDelayUntil 50ms 固定节拍驱动 vPidPeriodic50ms。
 * 替代原 TIM5 中断方案 —— 周期处理会调 vChassisMove（阻塞 UART 发送 +
 * 1ms 级任务延时），中断里跑会死锁（HAL 时基 TIM6 优先级 15 无法抢占 TIM5 的 6）。
 */
void vPidTask(void *argument)
{
    TickType_t xLastWakeTime;

    (void)argument;

    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        vPidPeriodic50ms();

        /* 陀螺源热备看门狗（自动切换）——借这个**工程里唯一常驻的 50ms 周期**跑:
         * 它只读 Beam + 写 Beam（换源那一刻才往 Pi 打一条 DBG），不发串口、不延时,
         * 放这儿不影响 50ms 节拍。判据是传感器任务记的"本口最近一帧的时刻",
         * 与 PID 本身无关（见 app_sensors.c::vGyroSrcWatchdogTick）。 */
        vGyroSrcWatchdogTick();

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(PID_PERIOD_MS));
    }
}
