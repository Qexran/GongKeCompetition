/**
  * @file       app_key.h
  * @brief      按键模块：5 键扫描 + 菜单导航状态。
  */

#ifndef __APP_KEY_H
#define __APP_KEY_H

#include "app_config.h"
#include "app_actuator.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

void vAppKeyInit(void);
void vKeyTask(void *argument);

#endif /* __APP_KEY_H */
