/**
  * @file       servo_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/02/11
  * @brief      舵机驱动（兼容180°角度型/360°旋转型），基于 STM32 HAL 库
  */
#include "servo_device.h"
#if SERVO_IS_ENABLE

static stServoDeviceParamTdf astServoDeviceParam[SERVO_DEV_NUM];

/* S 曲线插值形状（全局, 六轴共用; 默认五次赫米特 = 与加选择前逐字节一致）。
 * 当场生效: 正在播放的 SmoothStep 下一拍就按新曲线算, 进行中的 t 不重置。 */
static emServoCurveTdf s_emServoCurve = emServoCurve_Hermite5;

emServoCurveTdf emServoDeviceGetCurve(void)
{
    return s_emServoCurve;
}

void vServoDeviceSetCurve(emServoCurveTdf emCurve)
{
    if(emCurve > emServoCurve_Linear) { emCurve = emServoCurve_Hermite5; }   // 越界回默认
    s_emServoCurve = emCurve;
}

/// @brief      获取舵机设备参数（只读）
/// @param      emDevNum   ：设备号
/// @note       返回值为只读指针，避免参数被意外修改
const stServoDeviceParamTdf *c_pstGetServoDeviceParam(emServoDevNumTdf emDevNum)
{
    // 设备号边界检查
    if(emDevNum >= SERVO_DEV_NUM) return NULL;
    return &astServoDeviceParam[emDevNum];
}

/// @brief      初始化舵机运行参数（自定义参数）
/// @param      pstInit    ：初始化参数指针
/// @param      emDevNum   ：设备号
void vServoDeviceRunningParamInit(stServoRunningParamTdf *pstInit, emServoDevNumTdf emDevNum)
{
    if(emDevNum >= SERVO_DEV_NUM || pstInit == NULL) return;
    memcpy(&astServoDeviceParam[emDevNum].stRunningParam, pstInit, sizeof(stServoRunningParamTdf));
}

/// @brief      初始化舵机静态参数（自定义参数，硬件配置）
/// @param      pstInit    ：初始化参数指针
/// @param      emDevNum   ：设备号
/// @note       初始化后启动PWM输出
void vServoDeviceInit(stServoStaticParamTdf *pstInit, emServoDevNumTdf emDevNum)
{
    if(emDevNum >= SERVO_DEV_NUM || pstInit == NULL) return;
    
    // 1. 拷贝静态参数
    memcpy(&astServoDeviceParam[emDevNum].stStaticParam, pstInit, sizeof(stServoStaticParamTdf));
    
    // 2. 启动PWM输出
    #if (QEPACK_PLATFORM == TI)
        DL_Timer_startCounter(astServoDeviceParam[emDevNum].stStaticParam.stTimer->timer_inst);
    #else
        HAL_TIM_PWM_Start(astServoDeviceParam[emDevNum].stStaticParam.pstTimHandle,
                        astServoDeviceParam[emDevNum].stStaticParam.ulChannel);
    #endif
}

/// @brief      值转脉冲宽度（内部函数）
/// @param      emDevNum   ：设备号
/// @param      fValue     ：目标值（180°=角度，360°=速度）
/// @return     脉冲宽度(us)
static float fServoValueToPulse(emServoDevNumTdf emDevNum, float fValue)
{
    if(emDevNum >= SERVO_DEV_NUM) return 0.0f;
    
    stServoStaticParamTdf *pstStatic = &astServoDeviceParam[emDevNum].stStaticParam;
    
    // 边界限制
    if(fValue < pstStatic->fValueMin) fValue = pstStatic->fValueMin;
    if(fValue > pstStatic->fValueMax) fValue = pstStatic->fValueMax;
    
    // 180°角度型舵机：角度 -> 脉冲宽度（线性映射）
    if(pstStatic->emType == emServoType_Angle)
    {
        float fRange = pstStatic->fValueMax - pstStatic->fValueMin;
        if (fRange < 0.01f) return pstStatic->fPulseMid;
        return pstStatic->fPulseMin + (fValue - pstStatic->fValueMin) *
               (pstStatic->fPulseMax - pstStatic->fPulseMin) / fRange;
    }
    // 360°连续旋转型舵机：速度 -> 脉冲宽度
    else if(pstStatic->emType == emServoType_360)
    {
        if(fValue == 0) return pstStatic->fPulseMid; // 0速度=停转（中位脉冲）
        
        // 正转（0~100）：中位脉冲 -> 最大脉冲
        else if(fValue > 0)
        {
            return pstStatic->fPulseMid + (fValue / SERVO_360_SPEED_MAX) * 
                   (pstStatic->fPulseMax - pstStatic->fPulseMid);
        }
        // 反转（-100~0）：中位脉冲 -> 最小脉冲
        else
        {
            return pstStatic->fPulseMid - (fabs(fValue) / SERVO_360_SPEED_MAX) *
                   (pstStatic->fPulseMid - pstStatic->fPulseMin);
        }
    }
    
    return pstStatic->fPulseMid; // 默认返回中位脉冲（停转）
}

/// @brief      更新PWM占空比（内部函数）
/// @param      emDevNum   ：设备号
/// @param      fPulseUs   ：脉冲宽度(us)
static void vServoUpdatePwm(emServoDevNumTdf emDevNum, float fPulseUs)
{
    if(emDevNum >= SERVO_DEV_NUM) return;
    
    stServoStaticParamTdf *pstStatic = &astServoDeviceParam[emDevNum].stStaticParam;
    
    #if (QEPACK_PLATFORM == TI )
        if (pstStatic->stTimer == NULL || pstStatic->stTimer->clk_freq == 0) return;
        uint32_t Prescaler = CPUCLK_FREQ / pstStatic->stTimer->clk_freq;
        uint32_t ulArr = (CPUCLK_FREQ / (pstStatic->fPwmFreq * Prescaler)) - 1;
        uint32_t ulCcr = (uint32_t)((fPulseUs / 1000000.0f) * pstStatic->fPwmFreq * ulArr);

        DL_Timer_setCaptureCompareValue(pstStatic->stTimer->timer_inst, ulCcr, pstStatic->emChannel);
    #else
        // 直接读硬件ARR寄存器，避免SystemCoreClock与实际定时器时钟不一致
        uint32_t ulArr = pstStatic->pstTimHandle->Instance->ARR;
        // 计算CCR值：CCR = (脉冲宽度/1秒) * PWM频率 * (ARR+1)
        uint32_t ulCcr = (uint32_t)((fPulseUs / 1000000.0f) * pstStatic->fPwmFreq * (ulArr + 1));
        
        // 设置PWM比较值
        __HAL_TIM_SET_COMPARE(pstStatic->pstTimHandle, pstStatic->ulChannel, ulCcr);
    #endif
}

/// @brief      直接设置舵机值（静态模式）
/// @param      emDevNum   ：设备号
/// @param      fValue     ：目标值（180°=角度(°)；360°=速度(-100~100)）
/// @note       立即生效，无平滑过渡
void vServoSetValue(emServoDevNumTdf emDevNum, float fValue)
{
    if(emDevNum >= SERVO_DEV_NUM) return;

    stServoStaticParamTdf *pstStatic = &astServoDeviceParam[emDevNum].stStaticParam;

    // 1. 钳位到有效范围
    if(fValue < pstStatic->fValueMin) fValue = pstStatic->fValueMin;
    if(fValue > pstStatic->fValueMax) fValue = pstStatic->fValueMax;

    // 2. 更新当前值
    astServoDeviceParam[emDevNum].stRunningParam.fCurrentValue = fValue;
    
    // 2. 值转脉冲并更新PWM
    vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, fValue));
}

/// @brief      设置舵机目标值（平滑模式）
/// @param      emDevNum   ：设备号
/// @param      fValue     ：目标值（180°=角度(°)；360°=速度(-100~100)）
/// @note       需配合 vServoDevicePeriodExecute 实现平滑调速
void vServoSetTargetValue(emServoDevNumTdf emDevNum, float fValue)
{
    if(emDevNum >= SERVO_DEV_NUM) return;
    
    stServoStaticParamTdf *pstStatic = &astServoDeviceParam[emDevNum].stStaticParam;
    
    // 边界限制
    if(fValue < pstStatic->fValueMin) fValue = pstStatic->fValueMin;
    if(fValue > pstStatic->fValueMax) fValue = pstStatic->fValueMax;
    
    // 更新目标值
    astServoDeviceParam[emDevNum].stRunningParam.fTargetValue = fValue;
}

/// @brief      舵机周期执行
/// @param      emDevNum   ：设备号
void vServoDevicePeriodExecute(emServoDevNumTdf emDevNum)
{
    if(emDevNum >= SERVO_DEV_NUM) return;

    stServoRunningParamTdf *pstRunning = &astServoDeviceParam[emDevNum].stRunningParam;

    /* 写侧正在更新多字段指令 → 本拍跳过（PWM 保持上一值, 舵机原地保持一拍无感）。
     * 见 vServoSetTargetValueTimed 头注释的竞态说明, 2026-10-09 加。 */
    if(pstRunning->ucUpdating) return;

    if(pstRunning->emMode == emServoMode_Smooth)
    {
        float fDelta = pstRunning->fTargetValue - pstRunning->fCurrentValue;

        // 差值小于0.1时，认为已到位（避免抖动）
        if(fabs(fDelta) < 0.1f)
        {
            pstRunning->fCurrentValue = pstRunning->fTargetValue;
            vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, pstRunning->fCurrentValue));
            return;
        }

        // 按速度调整当前值（1ms调用一次，速度单位：值/ms）
        if(fDelta > 0)
        {
            pstRunning->fCurrentValue += pstRunning->fSpeed;
            if(pstRunning->fCurrentValue > pstRunning->fTargetValue)
                pstRunning->fCurrentValue = pstRunning->fTargetValue;
        }
        else
        {
            pstRunning->fCurrentValue -= pstRunning->fSpeed;
            if(pstRunning->fCurrentValue < pstRunning->fTargetValue)
                pstRunning->fCurrentValue = pstRunning->fTargetValue;
        }

        vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, pstRunning->fCurrentValue));
    }
    else if(pstRunning->emMode == emServoMode_SmoothStep)
    {
        float fPrev = pstRunning->fCurrentValue;

        if(pstRunning->ulSmoothStepElapsedMs < pstRunning->ulSmoothStepDurationMs)
        {
            /* 进度形状: 按全局选择的插值曲线把 t∈[0,1] 映射到 s∈[0,1]
             * （见 emServoCurveTdf; 默认五次赫米特 = 与加选择前逐字节一致） */
            float s;
            pstRunning->ulSmoothStepElapsedMs++;
            float t = (float)pstRunning->ulSmoothStepElapsedMs / (float)pstRunning->ulSmoothStepDurationMs;
            switch (s_emServoCurve)
            {
            case emServoCurve_Cubic:  s = t * t * (3.0f - 2.0f * t); break;               /* 3t²-2t³ */
            case emServoCurve_Cosine: s = 0.5f * (1.0f - cosf(3.14159265f * t)); break;   /* (1-cos πt)/2 */
            case emServoCurve_Linear: s = t; break;                                        /* 恒速 */
            default:                  s = t * t * t * (t * (6.0f * t - 15.0f) + 10.0f); break;  /* 6t⁵-15t⁴+10t³ */
            }
            pstRunning->fCurrentValue = pstRunning->fStartValue +
                s * (pstRunning->fTargetValue - pstRunning->fStartValue);
        }
        else
        {
            /* S 曲线走完了。若中途被速度上限削过速, 当前位置还没到目标 ——
             * 这时**不能**直接写 fTargetValue（被削掉的那段会凭空跳过去, 等于没限速）,
             * 交给恒速模式收尾。没限速时两者位置相等, 走 Static 与原行为完全一致。 */
            if(fabsf(pstRunning->fTargetValue - pstRunning->fCurrentValue) < 0.1f)
            {
                pstRunning->fCurrentValue = pstRunning->fTargetValue;
                pstRunning->emMode = emServoMode_Static;
            }
            else
            {
                pstRunning->emMode = emServoMode_Smooth;
            }
        }

        /* 总速度闸（vServoSetSpeed 设的）: 把这一拍的变化量夹住。
         * 五次赫米特中段最快（峰值 1.875·Δ/duration）, 大行程短时长时会明显超出
         * 舵机允许的转速 —— 拖滑杆那种"每拍都给新目标"的场景更是完全靠它兜底。 */
        if(pstRunning->fSpeed > 0.0f)
        {
            float fStep = pstRunning->fCurrentValue - fPrev;
            if(fStep >  pstRunning->fSpeed) fStep =  pstRunning->fSpeed;
            if(fStep < -pstRunning->fSpeed) fStep = -pstRunning->fSpeed;
            pstRunning->fCurrentValue = fPrev + fStep;
        }

        vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, pstRunning->fCurrentValue));
    }
}

/// @brief      180°角度型舵机默认参数初始化（简化版）
/// @param      emDevNum   ：设备号
/// @param      pstTimHandle ：TIM句柄
/// @param      ulChannel ：PWM通道
#if (QEPACK_PLATFORM == TI )
    void vServoDeviceDefaultInit_Angle(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel)
    {
        if(emDevNum >= SERVO_DEV_NUM || stTimer == NULL) return;
        
        // 1. 静态参数（使用project_config.h的默认宏定义）
        stServoStaticParamTdf stStaticInit = {
            .stTimer = stTimer,
            .emChannel = emChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_Angle,
            .fValueMin = SERVO_DEFAULT_ANGLE_MIN,
            .fValueMax = SERVO_DEFAULT_ANGLE_MAX,
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,
            .fPulseMid = 0.0f, // 180°舵机无需中位脉冲
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,
        };
        
        // 2. 运行参数（默认配置）
        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = (SERVO_DEFAULT_ANGLE_MIN + SERVO_DEFAULT_ANGLE_MAX) / 2, // 初始90°
            .fTargetValue = (SERVO_DEFAULT_ANGLE_MIN + SERVO_DEFAULT_ANGLE_MAX) / 2,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };
        
        // 3. 执行初始化
        vServoDeviceInit(&stStaticInit, emDevNum);
        vServoDeviceRunningParamInit(&stRunInit, emDevNum);
        
        // 4. 设置初始角度（90°）
        vServoSetValue(emDevNum, stRunInit.fCurrentValue);
    }
#else
    void vServoDeviceDefaultInit_Angle(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel)
    {
        if(emDevNum >= SERVO_DEV_NUM || pstTimHandle == NULL) return;
        
        // 1. 静态参数（使用project_config.h的默认宏定义）
        stServoStaticParamTdf stStaticInit = {
            .pstTimHandle = pstTimHandle,
            .ulChannel = ulChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_Angle,
            .fValueMin = SERVO_DEFAULT_ANGLE_MIN,
            .fValueMax = SERVO_DEFAULT_ANGLE_MAX,
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,
            .fPulseMid = 0.0f, // 180°舵机无需中位脉冲
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,
        };
        
        // 2. 运行参数（默认配置）
        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = (SERVO_DEFAULT_ANGLE_MIN + SERVO_DEFAULT_ANGLE_MAX) / 2, // 初始90°
            .fTargetValue = (SERVO_DEFAULT_ANGLE_MIN + SERVO_DEFAULT_ANGLE_MAX) / 2,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };
        
        // 3. 执行初始化
        vServoDeviceInit(&stStaticInit, emDevNum);
        vServoDeviceRunningParamInit(&stRunInit, emDevNum);
        
        // 4. 设置初始角度（90°）
        vServoSetValue(emDevNum, stRunInit.fCurrentValue);
    }
#endif

/// @brief      270°角度型舵机默认参数初始化（简化版）
/// @param      emDevNum   ：设备号
/// @param      pstTimHandle ：TIM句柄
/// @param      ulChannel ：PWM通道
#if (QEPACK_PLATFORM == TI )
    void vServoDeviceDefaultInit_Angle270(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel)
    {
        if(emDevNum >= SERVO_DEV_NUM || stTimer == NULL) return;

        stServoStaticParamTdf stStaticInit = {
            .stTimer = stTimer,
            .emChannel = emChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_Angle,
            .fValueMin = SERVO_DEFAULT_ANGLE_MIN,
            .fValueMax = SERVO_DEFAULT_ANGLE_270_MAX,
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,
            .fPulseMid = 0.0f,
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,
        };

        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = SERVO_DEFAULT_ANGLE_270_MAX / 2, // 初始135°
            .fTargetValue = SERVO_DEFAULT_ANGLE_270_MAX / 2,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };

        vServoDeviceInit(&stStaticInit, emDevNum);
        vServoDeviceRunningParamInit(&stRunInit, emDevNum);
        vServoSetValue(emDevNum, stRunInit.fCurrentValue);
    }
#else
    void vServoDeviceDefaultInit_Angle270(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel)
    {
        if(emDevNum >= SERVO_DEV_NUM || pstTimHandle == NULL) return;

        stServoStaticParamTdf stStaticInit = {
            .pstTimHandle = pstTimHandle,
            .ulChannel = ulChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_Angle,
            .fValueMin = SERVO_DEFAULT_ANGLE_MIN,
            .fValueMax = SERVO_DEFAULT_ANGLE_270_MAX,
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,
            .fPulseMid = 0.0f,
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,
        };

        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = SERVO_DEFAULT_ANGLE_270_MAX / 2, // 初始135°
            .fTargetValue = SERVO_DEFAULT_ANGLE_270_MAX / 2,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };

        vServoDeviceInit(&stStaticInit, emDevNum);
        vServoDeviceRunningParamInit(&stRunInit, emDevNum);
        vServoSetValue(emDevNum, stRunInit.fCurrentValue);
    }
#endif

/// @brief      360°旋转型舵机默认参数初始化（简化版）
/// @param      emDevNum   ：设备号
/// @param      pstTimHandle ：TIM句柄
/// @param      ulChannel ：PWM通道
#if (QEPACK_PLATFORM == TI )
    void vServoDeviceDefaultInit_360(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel)
    {
        if(emDevNum >= SERVO_DEV_NUM || stTimer == NULL) return;
        
        // 1. 静态参数（使用project_config.h的默认宏定义）
        stServoStaticParamTdf stStaticInit = {
            .stTimer = stTimer,
            .emChannel = emChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_360,
            .fValueMin = SERVO_360_SPEED_MIN,    // -100
            .fValueMax = SERVO_360_SPEED_MAX,    // 100
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,// 500us
            .fPulseMid = SERVO_360_PULSE_MID,    // 1500us（停转）
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,// 2500us
        };
        
        // 2. 运行参数（默认配置）
        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = 0.0f, // 初始停转
            .fTargetValue = 0.0f,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };
        
        // 3. 执行初始化
        vServoDeviceInit(&stStaticInit, emDevNum);
        vServoDeviceRunningParamInit(&stRunInit, emDevNum);
        
        // 4. 设置初始速度（停转）
        vServoSetValue(emDevNum, stRunInit.fCurrentValue);
    }
#else
    void vServoDeviceDefaultInit_360(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel)
    {
        if(emDevNum >= SERVO_DEV_NUM || pstTimHandle == NULL) return;
        
        // 1. 静态参数（使用project_config.h的默认宏定义）
        stServoStaticParamTdf stStaticInit = {
            .pstTimHandle = pstTimHandle,
            .ulChannel = ulChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_360,
            .fValueMin = SERVO_360_SPEED_MIN,    // -100
            .fValueMax = SERVO_360_SPEED_MAX,    // 100
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,// 500us
            .fPulseMid = SERVO_360_PULSE_MID,    // 1500us（停转）
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,// 2500us
        };
        
        // 2. 运行参数（默认配置）
        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = 0.0f, // 初始停转
            .fTargetValue = 0.0f,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };
        
        // 3. 执行初始化
        vServoDeviceInit(&stStaticInit, emDevNum);
        vServoDeviceRunningParamInit(&stRunInit, emDevNum);
        
        // 4. 设置初始速度（停转）
        vServoSetValue(emDevNum, stRunInit.fCurrentValue);
    }
#endif

/// @brief      360°角度型舵机默认参数初始化（简化版）
/// @param      emDevNum   ：设备号
/// @param      pstTimHandle ：TIM句柄
/// @param      ulChannel ：PWM通道
#if (QEPACK_PLATFORM == TI )
    void vServoDeviceDefaultInit_Angle360(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel)
    {
        if(emDevNum >= SERVO_DEV_NUM || stTimer == NULL) return;

        stServoStaticParamTdf stStaticInit = {
            .stTimer = stTimer,
            .emChannel = emChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_Angle,
            .fValueMin = SERVO_DEFAULT_ANGLE_MIN,
            .fValueMax = SERVO_DEFAULT_ANGLE_360_MAX,
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,
            .fPulseMid = 0.0f,
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,
        };

        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = SERVO_DEFAULT_ANGLE_360_MAX / 2, // 初始180°
            .fTargetValue = SERVO_DEFAULT_ANGLE_360_MAX / 2,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };

        vServoDeviceInit(&stStaticInit, emDevNum);
        vServoDeviceRunningParamInit(&stRunInit, emDevNum);
        vServoSetValue(emDevNum, stRunInit.fCurrentValue);
    }
#else
    void vServoDeviceDefaultInit_Angle360(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel)
    {
        if(emDevNum >= SERVO_DEV_NUM || pstTimHandle == NULL) return;

        stServoStaticParamTdf stStaticInit = {
            .pstTimHandle = pstTimHandle,
            .ulChannel = ulChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_Angle,
            .fValueMin = SERVO_DEFAULT_ANGLE_MIN,
            .fValueMax = SERVO_DEFAULT_ANGLE_360_MAX,
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,
            .fPulseMid = 0.0f,
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,
        };

        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = SERVO_DEFAULT_ANGLE_360_MAX / 2, // 初始180°
            .fTargetValue = SERVO_DEFAULT_ANGLE_360_MAX / 2,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };

        vServoDeviceInit(&stStaticInit, emDevNum);
        vServoDeviceRunningParamInit(&stRunInit, emDevNum);
        vServoSetValue(emDevNum, stRunInit.fCurrentValue);
    }
#endif

/// @brief      180°角度型舵机初始化（指定上电初始角度）
/// @param      emDevNum     ：设备号
/// @param      pstTimHandle ：TIM句柄
/// @param      ulChannel    ：PWM通道
/// @param      fInitAngle   ：上电初始角度(°)，应与手摆物理位置一致，避免上电大幅偏转
/// @note       先写CCR再启动PWM：上电第一帧即输出初始角度脉宽，无 0 脉宽毛刺
#if (QEPACK_PLATFORM == TI )
    void vServoDeviceInit_AngleAt(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel, float fInitAngle)
    {
        if(emDevNum >= SERVO_DEV_NUM || stTimer == NULL) return;

        // 1. 静态参数（使用project_config.h的默认宏定义）
        stServoStaticParamTdf stStaticInit = {
            .stTimer = stTimer,
            .emChannel = emChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_Angle,
            .fValueMin = SERVO_DEFAULT_ANGLE_MIN,
            .fValueMax = SERVO_DEFAULT_ANGLE_MAX,
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,
            .fPulseMid = 0.0f,
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,
        };
        memcpy(&astServoDeviceParam[emDevNum].stStaticParam, &stStaticInit, sizeof(stServoStaticParamTdf));

        // 2. 初始角度钳位到有效范围
        if(fInitAngle < stStaticInit.fValueMin) fInitAngle = stStaticInit.fValueMin;
        if(fInitAngle > stStaticInit.fValueMax) fInitAngle = stStaticInit.fValueMax;

        // 3. 先写CCR再启动PWM（先PWM_Start后写CCR会有0脉宽窗口，导致上电满偏）
        vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, fInitAngle));
        DL_Timer_startCounter(astServoDeviceParam[emDevNum].stStaticParam.stTimer->timer_inst);

        // 4. 运行参数以初始角度初始化
        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = fInitAngle,
            .fTargetValue = fInitAngle,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };
        memcpy(&astServoDeviceParam[emDevNum].stRunningParam, &stRunInit, sizeof(stServoRunningParamTdf));
    }
#else
    void vServoDeviceInit_AngleAt(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel, float fInitAngle)
    {
        if(emDevNum >= SERVO_DEV_NUM || pstTimHandle == NULL) return;

        // 1. 静态参数（使用project_config.h的默认宏定义）
        stServoStaticParamTdf stStaticInit = {
            .pstTimHandle = pstTimHandle,
            .ulChannel = ulChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_Angle,
            .fValueMin = SERVO_DEFAULT_ANGLE_MIN,
            .fValueMax = SERVO_DEFAULT_ANGLE_MAX,
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,
            .fPulseMid = 0.0f,
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,
        };
        memcpy(&astServoDeviceParam[emDevNum].stStaticParam, &stStaticInit, sizeof(stServoStaticParamTdf));

        // 2. 初始角度钳位到有效范围
        if(fInitAngle < stStaticInit.fValueMin) fInitAngle = stStaticInit.fValueMin;
        if(fInitAngle > stStaticInit.fValueMax) fInitAngle = stStaticInit.fValueMax;

        // 3. 先写CCR再启动PWM（先PWM_Start后写CCR会有0脉宽窗口，导致上电满偏）
        vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, fInitAngle));
        HAL_TIM_PWM_Start(astServoDeviceParam[emDevNum].stStaticParam.pstTimHandle,
                        astServoDeviceParam[emDevNum].stStaticParam.ulChannel);

        // 4. 运行参数以初始角度初始化
        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = fInitAngle,
            .fTargetValue = fInitAngle,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };
        memcpy(&astServoDeviceParam[emDevNum].stRunningParam, &stRunInit, sizeof(stServoRunningParamTdf));
    }
#endif

/// @brief      360°角度型舵机初始化（指定上电初始角度）
/// @param      emDevNum     ：设备号
/// @param      pstTimHandle ：TIM句柄
/// @param      ulChannel    ：PWM通道
/// @param      fInitAngle   ：上电初始角度(°，0~360)，应与手摆物理位置一致
/// @note       先写CCR再启动PWM：上电第一帧即输出初始角度脉宽，无 0 脉宽毛刺
#if (QEPACK_PLATFORM == TI )
    void vServoDeviceInit_Angle360At(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel, float fInitAngle)
    {
        if(emDevNum >= SERVO_DEV_NUM || stTimer == NULL) return;

        // 1. 静态参数（使用project_config.h的默认宏定义）
        stServoStaticParamTdf stStaticInit = {
            .stTimer = stTimer,
            .emChannel = emChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_Angle,
            .fValueMin = SERVO_DEFAULT_ANGLE_MIN,
            .fValueMax = SERVO_DEFAULT_ANGLE_360_MAX,
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,
            .fPulseMid = 0.0f,
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,
        };
        memcpy(&astServoDeviceParam[emDevNum].stStaticParam, &stStaticInit, sizeof(stServoStaticParamTdf));

        // 2. 初始角度钳位到有效范围
        if(fInitAngle < stStaticInit.fValueMin) fInitAngle = stStaticInit.fValueMin;
        if(fInitAngle > stStaticInit.fValueMax) fInitAngle = stStaticInit.fValueMax;

        // 3. 先写CCR再启动PWM
        vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, fInitAngle));
        DL_Timer_startCounter(astServoDeviceParam[emDevNum].stStaticParam.stTimer->timer_inst);

        // 4. 运行参数以初始角度初始化
        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = fInitAngle,
            .fTargetValue = fInitAngle,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };
        memcpy(&astServoDeviceParam[emDevNum].stRunningParam, &stRunInit, sizeof(stServoRunningParamTdf));
    }
#else
    void vServoDeviceInit_Angle360At(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel, float fInitAngle)
    {
        if(emDevNum >= SERVO_DEV_NUM || pstTimHandle == NULL) return;

        // 1. 静态参数（使用project_config.h的默认宏定义）
        stServoStaticParamTdf stStaticInit = {
            .pstTimHandle = pstTimHandle,
            .ulChannel = ulChannel,
            .fPwmFreq = SERVO_DEFAULT_PWM_FREQ,
            .emType = emServoType_Angle,
            .fValueMin = SERVO_DEFAULT_ANGLE_MIN,
            .fValueMax = SERVO_DEFAULT_ANGLE_360_MAX,
            .fPulseMin = SERVO_DEFAULT_PULSE_MIN,
            .fPulseMid = 0.0f,
            .fPulseMax = SERVO_DEFAULT_PULSE_MAX,
        };
        memcpy(&astServoDeviceParam[emDevNum].stStaticParam, &stStaticInit, sizeof(stServoStaticParamTdf));

        // 2. 初始角度钳位到有效范围
        if(fInitAngle < stStaticInit.fValueMin) fInitAngle = stStaticInit.fValueMin;
        if(fInitAngle > stStaticInit.fValueMax) fInitAngle = stStaticInit.fValueMax;

        // 3. 先写CCR再启动PWM
        vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, fInitAngle));
        HAL_TIM_PWM_Start(astServoDeviceParam[emDevNum].stStaticParam.pstTimHandle,
                        astServoDeviceParam[emDevNum].stStaticParam.ulChannel);

        // 4. 运行参数以初始角度初始化
        stServoRunningParamTdf stRunInit = {
            .emMode = emServoMode_Static,
            .fCurrentValue = fInitAngle,
            .fTargetValue = fInitAngle,
            .fSpeed = SERVO_DEFAULT_SPEED,
        };
        memcpy(&astServoDeviceParam[emDevNum].stRunningParam, &stRunInit, sizeof(stServoRunningParamTdf));
    }
#endif

/// @brief      设置舵机模式
/// @param      emDevNum   ：设备号
/// @param      newemMode ：新的模式
void vServoSetMode(emServoDevNumTdf emDevNum, emServoModeTdf newemMode)
{
    if(emDevNum >= SERVO_DEV_NUM) return;
    astServoDeviceParam[emDevNum].stRunningParam.emMode = newemMode;
}

/// @brief      设置目标值（S 曲线模式，指定运动时长）
/// @param      emDevNum     ：设备号
/// @param      fTarget      ：目标值
/// @param      ulDurationMs ：运动总时长(ms)
void vServoSetTargetValueTimed(emServoDevNumTdf emDevNum, float fTarget, uint32_t ulDurationMs)
{
    if(emDevNum >= SERVO_DEV_NUM) return;

    stServoDeviceParamTdf *pstDev = &astServoDeviceParam[emDevNum];
    stServoRunningParamTdf *pstRun = &pstDev->stRunningParam;
    stServoStaticParamTdf *pstStatic = &pstDev->stStaticParam;

    // 边界限制
    if(fTarget < pstStatic->fValueMin) fTarget = pstStatic->fValueMin;
    if(fTarget > pstStatic->fValueMax) fTarget = pstStatic->fValueMax;

    /* ⚠️ 下面的多字段写是一条「运动指令」, 读侧是 1ms 的 vServoDevicePeriodExecute
     *    (TIM7, prio 5)。裸写时 ISR 可插在任意两条 store 之间, 按「新 start + 旧
     *    target/duration」的错配组合算一拍位置 → 偶发 1ms PWM 毛刺（高频 JOG 时叠加,
     *    是"舵机偶尔抖一下"的合理根因）。QEPack 保持平台无关、不引 RTOS 临界区,
     *    改用 ucUpdating 标志: 先置 1 再写字段, ISR 见 1 跳过本轴一拍; 单核下 ISR
     *    要么在置 1 前进入（读到的还是完整旧指令）, 要么置 1 后进入（跳过）——
     *    不会读到半新半旧的组合。（2026-10-09 加） */
    pstRun->ucUpdating = 1u;

    if(ulDurationMs == 0)
    {
        // 时长为零，直接跳到目标
        pstRun->fCurrentValue = fTarget;
        pstRun->fTargetValue = fTarget;
        pstRun->emMode = emServoMode_Static;
        pstRun->ucUpdating = 0u;
        vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, fTarget));
        return;
    }

    pstRun->fStartValue = pstRun->fCurrentValue;
    pstRun->fTargetValue = fTarget;
    pstRun->ulSmoothStepDurationMs = ulDurationMs;
    pstRun->ulSmoothStepElapsedMs = 0;
    pstRun->emMode = emServoMode_SmoothStep;
    pstRun->ucUpdating = 0u;
}

/// @brief      设置「恒速趋近」的最大速度（每 1ms 的最大变化量）
/// @param      emDevNum   ：设备号
/// @param      fSpeedPerMs：°/ms（<=0 表示不限速）
/// @note       这个值同时是 S 曲线模式的**速度上限**（见 vServoDevicePeriodExecute）,
///             所以它是一条总速度闸: 曲线比它慢就听曲线的, 曲线要的更快就削到它。
void vServoSetSpeed(emServoDevNumTdf emDevNum, float fSpeedPerMs)
{
    if(emDevNum >= SERVO_DEV_NUM) return;
    if(fSpeedPerMs < 0.0f) fSpeedPerMs = 0.0f;
    astServoDeviceParam[emDevNum].stRunningParam.fSpeed = fSpeedPerMs;
}

/// @brief      恒速趋近目标（不带 S 曲线, 每 1ms 走 fSpeed 那么多）
/// @param      emDevNum   ：设备号
/// @param      fTarget    ：目标值
/// @note       与 vServoSetTargetValueTimed 的关键区别: **不重启任何曲线**。
///             拖滑杆时每一拍都来一个新目标, 走 S 曲线的话每拍都会把 t 打回 0 从头缓入
///             （那正是"拖不跟手"的根因）; 恒定速度则是一条干净的一阶跟随 ——
///             手指移得比速度上限慢时它就是跟手的, 移得太快也只是被削速, 不会跳。
void vServoSetTargetValueSlew(emServoDevNumTdf emDevNum, float fTarget)
{
    if(emDevNum >= SERVO_DEV_NUM) return;

    stServoDeviceParamTdf *pstDev = &astServoDeviceParam[emDevNum];
    stServoRunningParamTdf *pstRun = &pstDev->stRunningParam;
    stServoStaticParamTdf *pstStatic = &pstDev->stStaticParam;

    // 边界限制
    if(fTarget < pstStatic->fValueMin) fTarget = pstStatic->fValueMin;
    if(fTarget > pstStatic->fValueMax) fTarget = pstStatic->fValueMax;

    pstRun->ucUpdating = 1u;              /* 同 vServoSetTargetValueTimed 的竞态说明 */

    /* 已经到位就别切模式: 否则每拍一次 JOG 都会把完成标志压成"运动中",
     * PC 侧轮询 $B,POS 的 done 永远等不到。 */
    if(fabsf(fTarget - pstRun->fCurrentValue) < 0.1f)
    {
        pstRun->fCurrentValue = fTarget;
        pstRun->fTargetValue = fTarget;
        pstRun->emMode = emServoMode_Static;
        pstRun->ucUpdating = 0u;
        vServoUpdatePwm(emDevNum, fServoValueToPulse(emDevNum, fTarget));
        return;
    }

    pstRun->fTargetValue = fTarget;
    pstRun->emMode = emServoMode_Smooth;
    pstRun->ucUpdating = 0u;
}

/// @brief      查询这一轴**到位了没有**
/// @param      emDevNum   ：设备号
/// @return     1=已到位（Static, 或恒速模式已追上目标），0=还在走
/// @note       ⚠️ 判据是「到没到目标」而**不是**「S 曲线的时长走完没有」——
///             被速度闸(fSpeed)削速时, vServoDevicePeriodExecute 会在曲线时长走完、
///             位置还没到目标时切 emServoMode_Smooth 恒速收尾。老写法只看
///             `emMode != emServoMode_SmoothStep`, 于是**收尾一开始就报"完成"**,
///             而臂还在以 fSpeed 往目标爬几百毫秒(大行程可到 1s+)。
///             后果: 脚本的 STEP_ARM_GROUP_WAIT / STEP_SET_ZHUASHOU_WAIT、
///             蓝牙 $B,PLAY 与控制台「播放整个序列」的 $B,POS done 全部提前放行 ——
///             底盘在臂还没到位时就动、连续位姿被压成"提前播放"。别再改回去。
uint8_t ucServoIsSmoothStepDone(emServoDevNumTdf emDevNum)
{
    stServoRunningParamTdf *pstRun;

    if(emDevNum >= SERVO_DEV_NUM) return 1;
    pstRun = &astServoDeviceParam[emDevNum].stRunningParam;

    if(pstRun->emMode == emServoMode_SmoothStep) return 0;      /* S 曲线进行中 */

    if(pstRun->emMode == emServoMode_Smooth)                    /* 恒速趋近/收尾 */
    {
        return (fabsf(pstRun->fTargetValue - pstRun->fCurrentValue) < 0.1f) ? 1u : 0u;
    }
    return 1;                                                   /* Static = 到位 */
}

/// @brief      按角度差 + 名义转速自动估算运动时长（内部函数）
/// @param      emDevNum   ：设备号
/// @param      fTarget    ：目标值（已钳位）
/// @return     运动时长(ms)
/// @note       同 pro 项目方案：分段基准 × diff / 名义转速，差值越大转速利用率越高
static uint32_t ulServoAutoTimeMs(emServoDevNumTdf emDevNum, float fTarget)
{
    float fCur = astServoDeviceParam[emDevNum].stRunningParam.fCurrentValue;
    float fDiff = fabsf(fTarget - fCur);

    float fBase;
    if      (fDiff <= 28)  fBase = 120;
    else if (fDiff <= 50)  fBase = 100;
    else if (fDiff <= 100) fBase = 85;
    else if (fDiff <= 150) fBase = 70;
    else if (fDiff <= 250) fBase = 60;
    else                   fBase = 50;

    float fTime = fBase * fDiff / SERVO_AUTO_MOVE_SPEED;
    if(fTime < (float)SERVO_AUTO_MOVE_MIN_MS) fTime = (float)SERVO_AUTO_MOVE_MIN_MS;
    return (uint32_t)fTime;
}

/// @brief      设置目标值（S 曲线模式，自动估算运动时长）
/// @param      emDevNum   ：设备号
/// @param      fTarget    ：目标值（180°=角度(°)）
/// @note       时长按当前值与目标的角度差 + 名义转速（SERVO_AUTO_MOVE_SPEED）估算
void vServoSetTargetValueAuto(emServoDevNumTdf emDevNum, float fTarget)
{
    stServoStaticParamTdf *pstStatic;

    if(emDevNum >= SERVO_DEV_NUM) return;
    pstStatic = &astServoDeviceParam[emDevNum].stStaticParam;

    // 先钳位，保证估时与实际行程一致
    if(fTarget < pstStatic->fValueMin) fTarget = pstStatic->fValueMin;
    if(fTarget > pstStatic->fValueMax) fTarget = pstStatic->fValueMax;

    vServoSetTargetValueTimed(emDevNum, fTarget, ulServoAutoTimeMs(emDevNum, fTarget));
}

/// @brief      查询所有舵机是否都**到位**
/// @return     1=六轴全部到位, 0=任一还在走（含恒速收尾阶段, 见 ucServoIsSmoothStepDone）
uint8_t ucServoIsAllSmoothStepDone(void)
{
    uint8_t i;
    for(i = 0; i < SERVO_DEV_NUM; i++)
    {
        if(!ucServoIsSmoothStepDone((emServoDevNumTdf)i)) return 0;
    }
    return 1;
}

#endif /* SERVO_IS_ENABLE */
