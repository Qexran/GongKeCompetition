/**
  * @file       app_sensors.h
  * @brief      传感器模块：姿态传感器。
  *
  *             **两个物理口 × 三种设备, 谁在哪个口由 app_config.h 的选型决定**（可以互换,
  *             也可以两口选同型号）：
  *               主口 USART6 (PC6/PC7, 板上丝印 HWT101_TX/RX) → 按 GYRO_MAIN_DEV 分派 (vSensorTask)
  *               副口 USART2 (PD5/PD6, 板上丝印 VICE_GYRO)    → 按 GYRO_SUB_DEV  分派 (vSensorTaskM0)
  *             设备（三套完全不同的协议, 见各自驱动）：
  *               ATK-MS901M 九轴 → atk_ms901m_device.c（0x55 0x55 变长帧, roll/pitch/yaw）
  *               M0 单轴 yaw     → gyro_m0_device.c    （0x5A 5 字节帧, ±180° 回绕慢漂）
  *               HWT101 Z 轴     → 本模块内联状态机     （0x55 帧头 + 0x53/0x52 + 11 字节）
  *
  *             两个任务都在跑（各自独占一路串口、各自解析），**谁发布由 Beam 决定**：
  *             `gyro_src` = 首选（操作员设的）, `gyro_src_eff` = 实际在发的那一路 ——
  *             两者不同 = 热备自动切换接管了（见 app_config.h 的 GYRO_AUTO_SWITCH 段
  *             与 vGyroSrcWatchdogTick）。
  *             下游（app_pid / app_line / app_oled / app_rpi）只读 fGetAttitudeYaw /
  *             ulGetYawFrameCount / ucTakeYawFresh, 一行都不用改。
  *             ⚠️ 两个都发是错的：vSetAttitude 是一个临界区里的三字快照, 两源混发会让
  *                roll/pitch 与 yaw 来自不同模块 —— 发布门必须只有一个赢家。
  */

#ifndef __APP_SENSORS_H
#define __APP_SENSORS_H

#include "app_config.h"

/* 设备码 —— 与 app_config.h 的「选项:0=ATK-901 1=M0 2=HWT101」文案一一对应。
 * ⚠️ 码值定义在这里而不是 app_config.h：那边**只能写裸整数**（写标识符会让控制台
 *    把该行判成 raw、下拉框消失, 见 tuning.py::classify_value + app.js::tuneInput）。
 *    改这里的顺序必须同步改 app_config.h 两条宏的「选项:」文案。 */
#define GYRO_DEV_ATK     0u   /* ATK-MS901M 九轴（roll/pitch/yaw） */
#define GYRO_DEV_M0      1u   /* M0 单轴 yaw（±180° 回绕、慢漂） */
#define GYRO_DEV_HWT101  2u   /* HWT101 Z 轴（频率/波特率开机由固件自动下发, 见 app_config.h 选型段） */

/* 端口别名（底层仍是 app_uart.h 的历史名; 丝印/引脚名沿用 HWT101 / VICE_GYRO）。 */
#define UART_GYRO_MAIN   UART_ID_HWT101   /* 主口 USART6 PC6/PC7 */
#define UART_GYRO_SUB    UART_ID_OPENMV   /* 副口 USART2 PD5/PD6 */

void vAppSensorInit(void);
void vSensorTask(void *argument);     /* 主口（USART6）：按 GYRO_MAIN_DEV 分派 */
void vSensorTaskM0(void *argument);   /* 副口（USART2）：按 GYRO_SUB_DEV 分派（名字里的 M0 是历史名） */

/* 校准「有效源」（gyro_src_eff）那颗陀螺（2026-10 加, OLED「陀螺校准」→ SCRIPT_REQ_GYRO_CAL）。
 * M0 = gyro_m0_bias_cal（解锁→零偏自校准→**阻塞约 21s**→保存）; HWT101 = CALSW=1 开陀螺
 * 自动校准→静止等待→关闭→保存（**阻塞约 7s**）; ATK 没接校准协议 → 拒绝并往 Pi 打调试串。
 * 返回 1 = 校准动作走完, 0 = 没法校。
 * ⚠️⚠️ **阻塞调用它的任务**, 且期间被校那颗不出帧 —— 只允许从**不发布 yaw** 的任务调
 *    （vScriptTask 的请求循环）, 并由 Beam 的 gyro_cal_busy 让热备看门狗在这段时间不判
 *    停摆不切源。调用前车必须静止, 期间不跑路线（脚本任务串行, 天然成立）。 */
uint8_t ucGyroCalActiveSource(void);

/* 陀螺源热备看门狗（自动切换）—— 由 **vPidTask 的 50ms 节拍**调用, 见 .c 里的规则表。
 * 只读 Beam + 写 Beam（换源时才往 Pi 打一条 DBG）, 不发串口、不延时。 */
void vGyroSrcWatchdogTick(void);

#endif /* __APP_SENSORS_H */
