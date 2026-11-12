/**
  * @file       app_oled.h
  * @brief      OLED 显示模块：菜单/状态显示。
  */

#ifndef __APP_OLED_H
#define __APP_OLED_H

#include "app_config.h"

void vAppOledInit(void);
void vOledTask(void *argument);

#endif /* __APP_OLED_H */
