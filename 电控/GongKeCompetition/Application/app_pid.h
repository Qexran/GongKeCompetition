/**
  * @file       app_pid.h
  * @brief      角度 PID 闭环模块（移植自 pro 项目 Hardware/pid.c）。
  *
  *             用 IMU 航向角(fGetAttitudeYaw)做反馈，vPidTask 以 vTaskDelayUntil
  *             50ms(PID_PERIOD_MS) 节拍驱动 vPidPeriodic50ms；ucPidMove 阻塞等待到位。
  */

#ifndef __APP_PID_H
#define __APP_PID_H

#include <stdint.h>
#include "app_motion_types.h"

/* PID 周期（ms），vPidTask 节拍 = vPidPeriodic50ms 的调用间隔 */
#define PID_PERIOD_MS   50u

/* PID 控制器结构体（位置式） */
typedef struct
{
    float kp, ki, kd;           /* 比例/积分/微分系数 */
    float error, lastError;     /* 当前误差 / 上次误差（微分项用） */
    float integral, maxIntegral;/* 积分累加 / 积分限幅 */
    float output, maxOutput;    /* 输出 / 输出限幅(rpm) */
} stPidTdf;

/* 应用入口：使能电机 + 装默认 PID 参数 + 世界航向归零 + 收敛策略初始化 */
void vAppPidInit(void);

/* 装默认 PID 参数 + 世界航向归零（TIM 全归 app_tick，本模块不再碰定时器） */
void vPidInit(void);

/* PID 闭环运动（阻塞）：v_x/v_y=平移(rpm), w=角速度基值(rpm), time=最大时长(ms),
 * target=目标航向(°), pid_choose=PID_MODE_*。完成时写入世界航向。 */
uint8_t ucPidMove(int v_x, int v_y, int w, uint32_t time, float target, int pid_choose);

/* 非阻塞 PID 运动（只装参数，由周期处理驱动） */
void vPidMoveNoWait(int v_x, int v_y, int w, uint32_t time, float target, int pid_choose);

/* 底盘直线运动 + 角度闭环（边走边转） */
void PID_vChassisMoveDistance(float x, float y, float angel, float speed);

/* 边走边转包装层（移植 pro Motion.c, 超时/速度宏见 app_config.h） */
void vPidTurn(float fTargetAngle);                              /* 两段原地转 */
void vPidMoveAndTurn(int v_x, int v_y, float fTargetAngle);     /* 平移+纠偏→消差 */
void vBackToRoad(float fTargetAngle);                           /* 回退回正 */

/* 支点转弯（阻塞, 陀螺仪闭环）: 只驱动一侧双轮、另一侧当支点。
 * iDir>0=左转(驱动右侧 {2,4} 正转), <=0=右转(驱动左侧 {1,3} 正转);
 * fAngleDeg=转角幅值(°)。标定宏见 app_config.h 的 PIVOT_TURN_*。 */
/* 支点转弯（单侧双轮 + 陀螺仪闭环, 阻塞到到位/超时）。
 * iDir:      1=左转 / 0=右转 —— **只在相对模式下有意义**（绝对模式往哪转由最短路径定）。
 * fAngleDeg: 相对模式 = 转角[°]; 绝对模式 = **目标 IMU 航向**[°]。
 * ucAbsolute: 0 = 相对（转到「当前航向 ± 转角」, 与历史行为一致, 老路线不填这个参数就是它）;
 *             1 = 绝对（转到 fAngleDeg 这个航向, 不受当前朝向影响）。
 * ucRev:     0 = 正向（默认/老行为, 逐字节不变）; 1 = **反向执行**（2026-11-01 加, 车倒着放/
 *             场地换边跑镜像路线）—— **转向与目标角不变**（"左转"仍逆时针转那么多）, 驱动侧
 *             换**对侧**双轮并**倒转**（左转: 右侧正转 ⇒ 左侧倒转）: 支点换到对侧、车心弧线
 *             镜像, 复现正向放置的世界轨迹。对侧与速度**同时**取反 ⇒ yaw 响应与正向同号,
 *             闭环/增益/世界航向记账全不用动（只翻其中一个 = yaw 响应翻号 → 正反馈飞车,
 *             见 app_pid.c 内核头注释）。
 * ⚠️ 绝对模式**不做** PIVOT_TURN_MIN_DEG 的小角度早退（那里判的是"转角太小",
 *    绝对模式没有转角这个概念）; 已在目标附近时闭环自己会在收敛带里停住。 */
void vPidPivotTurn(int iDir, float fAngleDeg, uint8_t ucAbsolute, uint8_t ucRev);

/* 支点转弯·**交接模式**（STEP_PIVOT_LINE_WAIT 用）: 与上面同形, 多两个参数 ——
 * fHandoffDeg: |误差| ≤ 它就提前松手, 把位置与航向交给巡线闭环（陀螺退出）;
 * ulLineFrame0: 发 LINE_START **之前**取的 ulGetLineFrameCount(), 帧推进了才允许交接。
 * ⚠️ **两个条件都齐才交** —— 这一条就是"等起流"与"转弯"的重叠: 转弯照转, 每拍顺带看一眼
 *    流起没起。先等满起流再转弯的话总时长 = 起流等待 + 转弯, 一点没省（真踩过）。
 * 返回 1 = 交接发生了（调用方接着跑巡线）; 0 = 走完了完整转弯 / 被中止 / IMU 故障
 * （fHandoffDeg ≤ PIVOT_TURN_CONVERGE_DEG、初值已在阈值内、角度太小 也走 0）。
 * ⚠️ 交接路径**不停车**（驱动侧残留转速由巡线的第一拍覆盖）, 但加速度照旧恢复 ——
 *    否则巡线段会一直用 PIVOT_TURN_ACCEL(=0, 不插值), 见 app_pid.c 函数头。 */
uint8_t vPidPivotTurnUntil(int iDir, float fAngleDeg, uint8_t ucAbsolute,
                           float fHandoffDeg, uint32_t ulLineFrame0);

/* 原地转弯（阻塞, 陀螺仪闭环）: 四个麦轮对称反向出力, 绕**车身中心**自转（车心基本不动）。
 * 与支点转弯的区别: 支点只驱动单侧双轮、车心会平移; 本动作四轮都出力。
 * iDir>0=左转(逆时针), <=0=右转; fAngleDeg=转角幅值(°)。负值按绝对值处理。
 * 标定宏见 app_config.h 的 SPIN_TURN_*（⚠️四轮增益与 PIVOT_TURN_* 不可互换）。 */
/* 原地转弯（四轮对转 + 陀螺仪闭环）。参数与 vPidPivotTurn **同形同义**
 * （iDir 只在相对模式用; ucAbsolute=0 相对 / 1 绝对航向）—— 两者只有增益与
 * 早退阈值不同, 别把参数含义搞混。 */
void vPidSpinTurn(int iDir, float fAngleDeg, uint8_t ucAbsolute);

/* 原地转弯·**交接模式**（STEP_SPIN_LINE_WAIT 用, 2026-10-30 加）: 与 vPidPivotTurnUntil
 * 同形同义 —— fHandoffDeg = |误差| ≤ 它且巡线帧已起流就提前松手;
 * ulLineFrame0 = 发 LINE_START **之前**取的 ulGetLineFrameCount()。
 * ⚠️ 交接路径**不停车**（四轮对称的自转残速由巡线第一拍的 vChassisMove 整体覆盖吃掉 ——
 *    覆盖是天然的, 与支点"驱动侧残留"同理）, 但加速度照旧恢复（否则巡线段会一直用
 *    SPIN_TURN_ACCEL=0, 见 app_pid.c 函数头）。阈值共用 PIVOT_LINE_HANDOFF_DEG ——
 *    自转角速度约支点的 2 倍, 交接瞬间残留也约 2 倍, 嫌松就调小那个宏（刻意不单开旋钮）。
 * 返回 1 = 交接发生了（调用方接着跑巡线）; 0 = 走完了完整转弯 / 被中止 / IMU 故障
 * （fHandoffDeg ≤ SPIN_TURN_CONVERGE_DEG、初值已在阈值内、角度太小 也走 0）。 */
uint8_t vPidSpinTurnUntil(int iDir, float fAngleDeg, uint8_t ucAbsolute,
                          float fHandoffDeg, uint32_t ulLineFrame0);

/* 周期处理（每 PID_PERIOD_MS 调用一次，由 vPidTask 驱动） */
void vPidPeriodic50ms(void);

/* PID 周期任务入口（freertos.c 注册，优先级 APP_PRIO_PID） */
void vPidTask(void *argument);

/* 电机使能 / PID 闭环使能 */
void vPidSetMotorEnable(uint8_t ucVal);
void vPidSetPidEnable(uint8_t ucVal);

/* 清除积分累加 */
void vPidClearIntegral(void);

/* 收敛模式参数初始化 */
void vPidConvergeInit(uint8_t ucStrategy, float fThreshold, uint8_t ucSamples, uint32_t ulTimeout);

/* 设置 ucPidMove 使用的底盘加速度(rpm/s)，默认 230 */
void vPidSetAccel(int32_t iVal);

#endif /* __APP_PID_H */
