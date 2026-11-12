/**
  * @file       app_arm.h
  * @brief      机械臂模块：6 轴舵机控制。
  */

#ifndef __APP_ARM_H
#define __APP_ARM_H

#include "app_config.h"

void vAppArmInit(void);
void vArmTask(void *argument);

/* 中途修改舵机速度（STEP_ARM_SPEED_SET 的原语; 瞬时, 不动车不动臂）:
 * 写 Beam 运行时量 arm_speed_dps 并**当场下发**全部舵机（°/s, 30~1000 钳位）——
 * 影响之后的恒速趋近与 S 曲线削峰, 对**正在播放**的动作当拍生效（下一拍削峰即用新值）。
 * 上电回 app_config.h 的 ARM_SERVO_MAX_SPEED_DPS。 */
void vArmSetSpeedLimit(float fDps);

/* ==================== 组动作（组 = SERVO1~4 + SERVO6, 夹爪 SERVO5 不进组） ====================
 * pfAngles[5] 顺序: SERVO1, SERVO2, SERVO3, SERVO4, SERVO6
 *
 * ⚠️ 两个函数都会给每个目标角叠加「组动作角度偏移」（Beam 的 arm_ofs_s*, 蓝牙可调）——
 *    台架标定用: 拖滑杆的落点与 S 曲线播放的落点差几度时, 靠它把播放整体挪回去。
 *    偏移植只加在这里（组动作唯一收口）, 拖滑杆（$B,JOG）与夹爪**不吃**它。 */
void vArmGroupMoveNoWait(const float *pfAngles);   /* 5 轴同时启动, 立即返回 */
void vArmGroupMoveWait (const float *pfAngles);    /* 启动后轮询到位, 超时 ARM_GROUP_WAIT_TIMEOUT_MS 放行 */

/* 夹爪开合: ucKaiHe 1=KAI 0=HE; ucDegree 0~100 (KAI d=开到 d%, HE d=合掉 d%) */
void vArmSetZhuaShou(uint8_t ucKaiHe, uint8_t ucDegree);

/* 夹爪开合(立即返回版): 只下发目标 + 定长 S 曲线, 不等到位。
 * 蓝牙点动/遥控用——拖滑块时不能把 vBtTask 卡住 300ms。脚本步骤仍用阻塞版。 */
void vArmSetZhuaShouNoWait(uint8_t ucKaiHe, uint8_t ucDegree);

/* ==================== 靶标校准：按像素偏差微调瞄准轴 ====================
 * 舵机6 = 画面横向(X), 舵机3 = 画面纵向(Y) —— 映射与比例尺/方向都是 app_config.h 的
 * TARGET_* 标定量（现场标定）。
 *
 * ⚠️ **增量式**：以两个舵机的**当前角度**为基准加偏移, 不是走到某个绝对角。
 *    所以同一个动作可以反复调用来迭代收敛（连排两步 = 两步逼近）。
 * ⚠️ 目标角按舵机静态参数的 fValueMin/fValueMax **钳位**（宁可少走一点也不越量程）。
 * 阻塞到两个轴都到位（带超时兜底, 算法同 vArmSetZhuaShou）。仅任务上下文调用。 */
void vArmAimByPixels(float fDxPx, float fDyPx);

/* ==================== 点动关节表（蓝牙编辑界面用） ====================
 * 关节序号 0..4 **按组顺序** SERVO1, SERVO2, SERVO3, SERVO4, SERVO6 —— 与
 * vArmGroupMoveWait/NoWait 的 pfAngles[0..4] 以及 STEP_ARM_GROUP_WAIT 的 afF[0..4]
 * 一一对应, 所以界面上记下来的 5 个值可以无损转回脚本步骤。
 * 夹爪(SERVO5)不进表: 它不是角度参数, 走 $B,GRIP 的「开/合 + 程度%」,
 * 与 STEP_SET_ZHUASHOU_WAIT 的语义完全对齐。
 *
 * 表里**不带中文名**——中文名复用「地图」Tab 从 app_script.h 参数 DSL 派生的
 * 「舵机1..舵机6」标签, 同一份关节标签只留一个来源。 */
typedef struct
{
    const char        *pcName;   /* "s1".."s4","s6"（与脚本步骤参数名一致） */
    emServoDevNumTdf   emDev;    /* 对应舵机设备号 */
    float              fMin;     /* 可动范围（取自舵机静态参数, 运行时单一事实源） */
    float              fMax;
} stArmJointTdf;

/* 填充点动关节表到调用方缓冲（模块内不留可变静态量）, 返回实际写入条数（当前 5）。
 * ucCapacity 小于 ARM_GROUP_NUM 时返回 0。 */
uint8_t ucArmFillJointTable(stArmJointTdf *pastOut, uint8_t ucCapacity);

/* 舵机运动控制直接用 QEPack 接口：
 *   vServoSetTargetValueTimed(ARM_SERVOx, fAngle, ulMs)   指定时长的 S 曲线
 *   vServoSetTargetValueAuto(ARM_SERVOx, fAngle)          按角度差自动估时
 *   ucServoIsAllSmoothStepDone()                          全部舵机是否到位 */

#endif /* __APP_ARM_H */
