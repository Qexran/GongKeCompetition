/**
  * @file       servo_device.h
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/02/11
  * @brief      舵机驱动（兼容180°角度型/360°旋转型），基于 STM32 HAL 库
  * @note       编码风格与LED驱动模块保持一致
  */
#include "project_config.h"
#if SERVO_IS_ENABLE

#ifndef _SERVO_DEVICE_H_
#define _SERVO_DEVICE_H_

#include "string.h"
#include "math.h"

#if (QEPACK_PLATFORM == TI)
	#include "ti_platform.h"
#endif

/// @brief          舵机设备号枚举
/// @note           与 SERVO_DEV_NUM 匹配，可扩展至更多设备
typedef enum
{
    emServoDevNum0        = 0,
    emServoDevNum1,
    emServoDevNum2,
    emServoDevNum3,
    emServoDevNum4,
    emServoDevNum5,
    emServoDevNum6,
    emServoDevNum7,
} emServoDevNumTdf;

/// @brief          舵机工作模式枚举
typedef enum
{
    emServoMode_Static        = 0,            // 静态模式（直接设置角度/速度）
    emServoMode_Smooth,                       // 平滑调速模式（按指定速度调整到目标值）
    emServoMode_SmoothStep,                   // S 曲线平滑模式（基于时间的插值, 形状可选, 见 emServoCurveTdf）
} emServoModeTdf;

/// @brief          S 曲线（SmoothStep 播放）的**插值曲线形状** —— 全局一份, 六轴共用（2026-10 加）。
/// @note           都把归一化时间 t∈[0,1] 映射到进度 s∈[0,1], 区别在两端/中段的快慢:
///                 Hermite5 五次S（默认, 与加选择前**逐字节一致**）: 两端零速, 中段峰速 1.875×平均
///                 Cubic    三次S: 两端零速, 峰速 1.5×平均 —— 同时长下中段更"匀", 前段更快
///                 Cosine   余弦: 两端零速, 峰速 π/2≈1.571×平均, 减速段比五次S长
///                 Linear   线性: 恒速（两端速度不连续; 实际感觉 = fSpeed 削峰后的梯形）
typedef enum
{
    emServoCurve_Hermite5     = 0,
    emServoCurve_Cubic        = 1,
    emServoCurve_Cosine       = 2,
    emServoCurve_Linear       = 3,
} emServoCurveTdf;

/// @brief          舵机类型枚举
typedef enum
{
    emServoType_Angle        = 0,    // 180°角度型舵机（位置控制）
    emServoType_360,                // 360°连续旋转型舵机（速度/方向控制）
} emServoTypeTdf;

/// @brief          舵机静态参数（硬件相关，初始化后不修改）
/// @note           包含PWM通道、硬件参数、角度/速度-脉冲映射关系
typedef struct
{
	#if (QEPACK_PLATFORM == TI)				// TIM句柄
		stTimerTdf 			*stTimer;
		DL_TIMER_CC_INDEX 	emChannel;           // PWM通道（如TIM_CHANNEL_2）
	#else
   		TIM_HandleTypeDef   *pstTimHandle;
		uint32_t            ulChannel;           // PWM通道（如TIM_CHANNEL_2）
    #endif

    float               fPwmFreq;            // PWM频率(Hz)
    emServoTypeTdf      emType;              // 舵机类型（180°/360°）
    float               fValueMin;           // 最小可控值（180°=角度，360°=速度）
    float               fValueMax;           // 最大可控值（180°=角度，360°=速度）
    float               fPulseMin;           // 最小脉冲宽度(us)
    float               fPulseMid;           // 中位脉冲宽度(us)（360°舵机停转）
    float               fPulseMax;           // 最大脉冲宽度(us)
} stServoStaticParamTdf;

/// @brief          舵机运行参数（状态/动态参数，运行时可修改）
typedef struct
{
    emServoModeTdf      emMode;              // 工作模式
    float               fCurrentValue;       // 当前值（180°=角度，360°=速度）
    float               fTargetValue;        // 目标值（180°=角度，360°=速度）
    float               fSpeed;              // 平滑调速时的速度（°/ms 或 速度值/ms）
    float               fStartValue;         // S 曲线起始值
    uint32_t            ulSmoothStepDurationMs;  // S 曲线运动总时长(ms)
    uint32_t            ulSmoothStepElapsedMs;   // S 曲线已用时间(ms)
    volatile uint8_t    ucUpdating;          // 写侧多字段更新中: 1ms 周期 ISR 见 1 就跳过本轴一拍
                                             // （写-读竞态见 vServoSetTargetValueTimed 的注释;
                                             //   驱动保持平台无关, 不引 RTOS 临界区, 2026-10-09 加）
} stServoRunningParamTdf;

/// @brief          舵机设备参数（整合静态+运行参数）
typedef struct
{
    stServoStaticParamTdf     stStaticParam;  // 静态参数
    stServoRunningParamTdf    stRunningParam; // 运行参数
} stServoDeviceParamTdf;

/* ==================== 对外公开函数声明 ==================== */
// 获取当前舵机的设备参数（只读）
const stServoDeviceParamTdf *c_pstGetServoDeviceParam(emServoDevNumTdf emDevNum);

// 运行参数初始化（自定义参数）
void vServoDeviceRunningParamInit(stServoRunningParamTdf *pstInit, emServoDevNumTdf emDevNum);

// 静态参数初始化（自定义参数，硬件配置）
void vServoDeviceInit(stServoStaticParamTdf *pstInit, emServoDevNumTdf emDevNum);

// 更改舵机模式
void vServoSetMode(emServoDevNumTdf emDevNum, emServoModeTdf newemMode);

// 直接设置舵机值（静态模式）
// 180°舵机：fValue=角度(°)；360°舵机：fValue=速度(-100~100)
void vServoSetValue(emServoDevNumTdf emDevNum, float fValue);

// 设置舵机目标值（平滑模式）
// 180°舵机：fValue=角度(°)；360°舵机：fValue=速度(-100~100)
void vServoSetTargetValue(emServoDevNumTdf emDevNum, float fValue);

// 舵机平滑模式周期执行函数
void vServoDevicePeriodExecute(emServoDevNumTdf emDevNum);

// 设置目标值（S 曲线模式，指定运动时长）
void vServoSetTargetValueTimed(emServoDevNumTdf emDevNum, float fTarget, uint32_t ulDurationMs);

// 查询这一轴**到位了没有**（函数名是历史包袱: 判据是「到没到目标」,
// 不是「S 曲线时长走完没有」—— 被速度闸削速后的恒速收尾阶段算"还没到"。
// 详见 servo_device.c 的实现注释, 别按名字改成只看 emMode）。
uint8_t ucServoIsSmoothStepDone(emServoDevNumTdf emDevNum);

// 设置目标值（S 曲线模式，按角度差+名义转速自动估算运动时长）
void vServoSetTargetValueAuto(emServoDevNumTdf emDevNum, float fTarget);

// 最大速度（每 1ms 的最大变化量，°/ms；<=0 = 不限速）。
// **这是全臂的总速度闸**: 它既是恒速趋近的速度, 也是 S 曲线模式的削峰上限。
void vServoSetSpeed(emServoDevNumTdf emDevNum, float fSpeedPerMs);

// S 曲线插值形状（全局, 六轴共用; 默认五次赫米特）。setter 越界值回默认。
// 当场生效: 对**正在播放**的 SmoothStep 动作下一拍就按新曲线算（进行中的 t 不重置）。
emServoCurveTdf emServoDeviceGetCurve(void);
void vServoDeviceSetCurve(emServoCurveTdf emCurve);

// 恒速趋近目标（不重启任何曲线）——拖滑杆/连续跟随用。
// 与 Timed 的区别: 每拍给新目标时 Timed 会把 S 曲线的 t 打回 0 从头缓入（拖不跟手）,
// 而它是一条干净的一阶跟随, 只受 vServoSetSpeed 的速度上限约束。
void vServoSetTargetValueSlew(emServoDevNumTdf emDevNum, float fTarget);

// 查询所有舵机（SERVO_DEV_NUM 全部, 含夹爪）是否都到位
uint8_t ucServoIsAllSmoothStepDone(void);

#if (QEPACK_PLATFORM == TI)
	// 180°角度型舵机默认参数初始化（极速版）
	void vServoDeviceDefaultInit_Angle(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel);

	// 270°角度型舵机默认参数初始化（极速版）
	void vServoDeviceDefaultInit_Angle270(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel);

	// 360°旋转型舵机默认参数初始化（极速版）
	void vServoDeviceDefaultInit_360(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel);

	// 360°角度型舵机默认参数初始化（极速版）
	void vServoDeviceDefaultInit_Angle360(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel);

	// 180°角度型舵机初始化（指定上电初始角度，先写CCR再启动PWM，避免上电偏转）
	void vServoDeviceInit_AngleAt(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel, float fInitAngle);

	// 360°角度型舵机初始化（指定上电初始角度，先写CCR再启动PWM，避免上电偏转）
	void vServoDeviceInit_Angle360At(emServoDevNumTdf emDevNum, stTimerTdf *stTimer, DL_TIMER_CC_INDEX emChannel, float fInitAngle);

#else
	// 180°角度型舵机默认参数初始化（极速版）
	void vServoDeviceDefaultInit_Angle(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel);

	// 270°角度型舵机默认参数初始化（极速版）
	void vServoDeviceDefaultInit_Angle270(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel);

	// 360°旋转型舵机默认参数初始化（极速版）
	void vServoDeviceDefaultInit_360(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel);

	// 360°角度型舵机默认参数初始化（极速版）
	void vServoDeviceDefaultInit_Angle360(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel);

	// 180°角度型舵机初始化（指定上电初始角度，先写CCR再启动PWM，避免上电偏转）
	void vServoDeviceInit_AngleAt(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel, float fInitAngle);

	// 360°角度型舵机初始化（指定上电初始角度，先写CCR再启动PWM，避免上电偏转）
	void vServoDeviceInit_Angle360At(emServoDevNumTdf emDevNum, TIM_HandleTypeDef *pstTimHandle, uint32_t ulChannel, float fInitAngle);
#endif

#endif /* _SERVO_DEVICE_H_ */
#endif /* SERVO_IS_ENABLE */

/*
	1.0 舵机如何工作？
		舵机通过接收PWM（脉冲宽度调制）信号来控制角度。这个信号有三个关键特征：

		频率固定：通常为50Hz（周期20ms）
		脉宽可变：高电平持续时间在0.5ms-2.5ms之间变化
		脉宽与角度对应：
			0.5ms → 0度
			1.5ms → 90度
			2.5ms → 180度
	

	1.1 接线方式
		舵机线缆	颜色			STM32连接点
		红色		电源+		5V电源
		棕色		地线			GND
		黄色		信号线		PWM输出引脚（如PA6）
		
	1.2 
		类型			控制逻辑
		180° 		脉冲宽度对应角度定位（如 500us=0°，2500us=180°），脉冲固定则角度固定
		360° 		脉冲宽度对应旋转方向 / 速度（如 500us = 最大速正转，1500us = 停转，2500us = 最大速反转）
	
	1.3 参考默认初始化示例
		// 初始化180°舵机0（TIM2，通道2）
		vServoDeviceDefaultInit_Angle(SERVO0, &htim2, TIM_CHANNEL_2);
		// 或指定上电初始角度45°（先写CCR再启动PWM，上电无偏转毛刺）
		vServoDeviceInit_AngleAt(SERVO0, &htim2, TIM_CHANNEL_2, 45.0f);
		// 控制180°舵机转到45°
		vServoSetValue(SERVO0, 45.0f);

		// 初始化360°舵机1（TIM3，通道1）
		vServoDeviceDefaultInit_360(SERVO1, &htim3, TIM_CHANNEL_1);
		// 控制360°舵机正转（速度50）
		vServoSetValue(SERVO1, 50.0f);
		// 控制360°舵机停转
		vServoSetValue(SERVO1, 0.0f);
	
	1.4 参考自定义参数初始化示例
	void ServoDeviceConfig(void)
	{
		stServoStaticParamTdf stServo0Static;
		stServo0Static.pstTimHandle = &htim2;
		stServo0Static.ulChannel = TIM_CHANNEL_2;
		stServo0Static.fPwmFreq = 50.0f;
		stServo0Static.emType = emServoType_Angle;
		stServo0Static.fValueMin = 0.0f;
		stServo0Static.fValueMax = 180.0f;
		stServo0Static.fPulseMin = 500.0f;
		stServo0Static.fPulseMid = 1500.0f;
		stServo0Static.fPulseMax = 2500.0f;
		
		// 初始化舵机0
		vServoDeviceInit(&stServo0Static, SERVO0);
	}

	// 3. 主循环
	int main(void)
	{
		HAL_Init();
		SystemClock_Config();
		MX_TIM2_Init(); // 初始化TIM2 PWM
		ServoDeviceConfig(); // 初始化舵机设备
		
		while(1)
		{
			// 示例1：静态模式直接设角度
			vServoSetValue(SERVO0, 45.0f);
			HAL_Delay(1000);
			vServoSetValue(SERVO0, 135.0f);
			HAL_Delay(1000);
			
			// 示例2：平滑模式调速（需确保1ms调用一次vServoDevicePeriodExecute）
			vServoSetMode(SERVO0, emServoMode_Smooth);
			vServoSetTargetValue(SERVO0, 0.0f);
			for(int i=0; i<1000; i++) // 模拟1ms循环
			{
				vServoDevicePeriodExecute(SERVO0);
				HAL_Delay(1);
			}
		}
	}
*/
