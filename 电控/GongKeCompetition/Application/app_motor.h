/**
  * @file       app_motor.h
  * @brief      底盘电机模块：EMM V5 闭环步进电机。
  */

#ifndef __APP_MOTOR_H
#define __APP_MOTOR_H

#include "app_config.h"

void vAppMotorInit(void);
void vMotorTask(void *argument);

#endif /* __APP_MOTOR_H */
