/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    freertos_app.h
  * @brief   Header for FreeRTOS application tasks.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FREERTOS_APP_H
#define __FREERTOS_APP_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"

/* Exported functions prototypes ---------------------------------------------*/
void MX_FREERTOS_Init(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __FREERTOS_APP_H */
