/**
  * @file       app_comm.h
  * @brief      通信模块：外部串口 的接收任务。
  *             树莓派接收任务在 app_rpi.h（vRpiTask）；
  *             蓝牙接收任务在 app_bt.h（vBtTask）；
  *             USART2（原 OPENMV 口）现由 M0 副陀螺仪独占，见 app_sensors.h。
  */

#ifndef __APP_COMM_H
#define __APP_COMM_H

#include "app_config.h"

void vExtTask(void *argument);

#endif /* __APP_COMM_H */
