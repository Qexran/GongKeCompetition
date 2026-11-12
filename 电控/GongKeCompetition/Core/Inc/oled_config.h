/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    oled_config.h
  * @brief   Minimal configuration for the QEPack OLED driver (STM32F4 port).
  *
  *          The OLED driver (OLED.h / OLED.c) originally depended on
  *          project_config.h. This shim supplies only the macros that driver
  *          actually needs, targeting STM32F4 + software (bit-bang) I2C.
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __OLED_CONFIG_H
#define __OLED_CONFIG_H

/* Platform selection (kept for compatibility with the driver's #if branches). */
#define ST                  1
#define TI                  0
#define QEPACK_PLATFORM     ST

/* OLED module switch and device count. */
#define OLED_IS_ENABLE      1
#define OLED_DEV_NUM        1
#define OLED_IS_USE_HARDWARE 0   /* 1 = hardware I2C (I2C2), 0 = software bit-bang */

/* Device alias used by application code. */
#define OLED0               emOledDevNum0

#endif /* __OLED_CONFIG_H */
