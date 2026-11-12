/**
  * @file       app_main.c
  * @brief      应用层总入口：初始化所有模块（在启动调度器之前调用）。
  */

#include "app_main.h"
#include "app_uart.h"
#include "app_key.h"
#include "app_oled.h"
#include "app_arm.h"
#include "app_motor.h"
#include "app_actuator.h"
#include "app_sensors.h"
#include "app_tick.h"
#include "app_pid.h"
#include "app_rpi.h"

void vAppInit(void)
{
    vAppUartInit();      /* 串口框架（6 路） */
    vAppKeyInit();       /* 按键 */
    vAppArmInit();       /* 机械臂舵机 */
    vAppMotorInit();     /* EMM V5 底盘电机 */
    vAppActuatorInit();  /* 蜂鸣器/继电器/补光 */
    vAppOledInit();      /* OLED */
    vAppSensorInit();    /* 姿态传感器（主口 USART6 / 副口 USART2, 各挂哪颗由 GYRO_MAIN_DEV/GYRO_SUB_DEV 选） */
    vAppTickInit();      /* 1ms 时基（硬件定时器） */
    vAppPidInit();       /* 角度 PID（默认参数 + 世界航向归零） */
    vAppRpiInit();       /* 树莓派通信（发送互斥） */
}
