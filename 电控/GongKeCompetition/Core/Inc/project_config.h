/**
  * @file       project_config.h
  * @author     Qe_xr (adapted for STM32F407 by Claude)
  * @version    V1.0.0
  * @date       2026/09/06
  * @brief      QEPack 驱动库统一配置文件（STM32F4 移植版）
  *
  *             原本为 STM32F1 平台的 project_config.h，此处仅保留本项目用到的
  *             模块开关与设备数量，并把 HAL 头换成 stm32f4xx_hal.h。
  */

#ifndef _PROJECT_CONFIG_H_
#define _PROJECT_CONFIG_H_

/* 平台占位符（保持与 QEPack 驱动 #if 分支兼容） */
#define GPIO                		0
#define TIM                 		1
#define TI                  		0
#define ST                  		1

/* ########################### System Configuration ######################### */
#define QEPACK_PLATFORM     ST

#if (QEPACK_PLATFORM == ST)
    #include "stm32f4xx_hal.h"
#endif

#define PI                  		3.141592653
#define SYSTEM_CORE_CLOCK   		168000000U     /* STM32F407 @ 168MHz */

/* ########################### KEY 按键 ############################### */
#define KEY_IS_ENABLE       		1
#define KEY_DEV_NUM         		5               /* MAIN/LEFT/RIGHT/UP/DOWN */
#define KEY0                		emKeyDevNum0
#define KEY1                		emKeyDevNum1
#define KEY2                		emKeyDevNum2
#define KEY3                		emKeyDevNum3
#define KEY4                		emKeyDevNum4

/* ########################### SERVO 舵机 ############################# */
#define SERVO_IS_ENABLE     		1
#define SERVO_DEV_NUM       		6               /* 6 轴机械臂 */
#define SERVO0              		emServoDevNum0
#define SERVO1              		emServoDevNum1
#define SERVO2              		emServoDevNum2
#define SERVO3              		emServoDevNum3
#define SERVO4              		emServoDevNum4
#define SERVO5              		emServoDevNum5

/* 舵机默认参数（180° 角度型 / 360° 旋转型） */
#define SERVO_DEFAULT_PWM_FREQ      50.0f
#define SERVO_DEFAULT_ANGLE_MIN     0.0f
#define SERVO_DEFAULT_ANGLE_MAX     180.0f
#define SERVO_DEFAULT_ANGLE_270_MAX 270.0f
#define SERVO_DEFAULT_ANGLE_360_MAX 360.0f
#define SERVO_DEFAULT_PULSE_MIN     500.0f
#define SERVO_DEFAULT_PULSE_MAX     2500.0f
#define SERVO_DEFAULT_SPEED         1.0f
#define SERVO_AUTO_MOVE_SPEED       13.0f   /* 自动估时名义转速（越小越慢，同 pro 项目 SERVO_SPEED） */
#define SERVO_AUTO_MOVE_MIN_MS      120u    /* 自动估时最短运动时长(ms) */
#define SERVO_360_PULSE_MID         1500.0f
#define SERVO_360_SPEED_MIN         -100.0f
#define SERVO_360_SPEED_MAX         100.0f

/* ########################### UART 串口 ############################# */
#define UART_IS_ENABLE      		0               /* 已弃用 QEPack uart_device，改用 app_uart.c 的 DMA+IDLE+信号量方案 */
#define UART_DEV_NUM        		6               /* USART1/2/3 + UART4/5 + USART6 */
#define UART_BUF_MAX_LEN    		256
#define UART_FRAME_MAX_LEN  		128
#define UART_TX_QUEUE_MAX_LEN 		256
#define UART_TX_BUF_MAX_LEN 		512
#define UART_IS_USE_DMA     		0               /* 0=中断单字节接收, 1=DMA */

/* ########################### EMM_V5 闭环步进电机 #################### */
#define EMM_V5_IS_ENABLE    		1

/* ########################### ATK_MS901M 九轴姿态传感器 ############# */
/* ⚠️ 必须保持 1：ATK 现在是**运行期可选**的（app_config.h 的 GYRO_MAIN_DEV / GYRO_SUB_DEV
 *    可以把任一口选成 ATK），app_sensors.c 无条件引用 atk_ms901m_* —— 填 0 会让它编不过
 *    （原来那条 `#if SENSOR_USE_ATK` 保护随编译期选型一起删了）。 */
#define ATK_MS901M_IS_ENABLE   		1

#endif /* _PROJECT_CONFIG_H_ */
