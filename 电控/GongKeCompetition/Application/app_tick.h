/**
  * @file       app_tick.h
  * @brief      应用时基：2 个硬件定时器（舵机平滑 / 服务层）。
  *
  *              优先级：舵机平滑调速 > 服务层（数值越小优先级越高，
  *              均 >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY(5)）。
  *              视觉 PID 不再用定时器中断，见 app_pid.c 的 vPidTask。
  */

#ifndef __APP_TICK_H
#define __APP_TICK_H

#include "app_config.h"

/* 定时器中断优先级 */
#define APP_TIM_PRIO_SERVO     5
#define APP_TIM_PRIO_SERVICE   7

void vAppTickInit(void);      /* 初始化 2 个硬件定时器 */

void vAppServoTick(void);     /* TIM7：舵机平滑调速 */
void vAppServiceTick(void);   /* TIM1：服务层（按键扫描） */

#endif /* __APP_TICK_H */
