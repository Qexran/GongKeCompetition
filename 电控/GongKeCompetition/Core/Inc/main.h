/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define OLED_SDA_GPIO_Pin GPIO_PIN_0
#define OLED_SDA_GPIO_GPIO_Port GPIOF
#define OLED_SCL_GPIO_Pin GPIO_PIN_1
#define OLED_SCL_GPIO_GPIO_Port GPIOF
#define MAIN_BUTTON_Pin GPIO_PIN_6
#define MAIN_BUTTON_GPIO_Port GPIOF
#define LEFT_BUTTON_Pin GPIO_PIN_8
#define LEFT_BUTTON_GPIO_Port GPIOF
#define EXTERNAL_UART_TX_Pin GPIO_PIN_0
#define EXTERNAL_UART_TX_GPIO_Port GPIOA
#define SERVO3_PWM_Pin GPIO_PIN_1
#define SERVO3_PWM_GPIO_Port GPIOA
#define SERVO2_PWM_Pin GPIO_PIN_2
#define SERVO2_PWM_GPIO_Port GPIOA
#define SERVO4_PWM_Pin GPIO_PIN_3
#define SERVO4_PWM_GPIO_Port GPIOA
#define RIGHT_BUTTON_Pin GPIO_PIN_5
#define RIGHT_BUTTON_GPIO_Port GPIOA
#define BU_GUANG_PWM_Pin GPIO_PIN_6
#define BU_GUANG_PWM_GPIO_Port GPIOA
#define DOWN_BUTTON_Pin GPIO_PIN_7
#define DOWN_BUTTON_GPIO_Port GPIOA
#define UP_BUTTON_Pin GPIO_PIN_4
#define UP_BUTTON_GPIO_Port GPIOC
#define SERVO5_PWM_Pin GPIO_PIN_0
#define SERVO5_PWM_GPIO_Port GPIOB
#define SERVO6_PWM_Pin GPIO_PIN_1
#define SERVO6_PWM_GPIO_Port GPIOB
#define EMM_MOTOR_TX_Pin GPIO_PIN_10
#define EMM_MOTOR_TX_GPIO_Port GPIOB
#define EMM_MOTOR_RX_Pin GPIO_PIN_11
#define EMM_MOTOR_RX_GPIO_Port GPIOB
#define JIGUANG_Pin GPIO_PIN_10
#define JIGUANG_GPIO_Port GPIOD
#define HWT101_TX_Pin GPIO_PIN_6
#define HWT101_TX_GPIO_Port GPIOC
#define HWT101_RX_Pin GPIO_PIN_7
#define HWT101_RX_GPIO_Port GPIOC
#define RB_TX_Pin GPIO_PIN_9
#define RB_TX_GPIO_Port GPIOA
#define RV_RX_Pin GPIO_PIN_10
#define RV_RX_GPIO_Port GPIOA
#define EXTERNAL_UART_RX_Pin GPIO_PIN_11
#define EXTERNAL_UART_RX_GPIO_Port GPIOC
#define BLUETOOTH_TX_Pin GPIO_PIN_12
#define BLUETOOTH_TX_GPIO_Port GPIOC
#define BLUETOOTH_RX_Pin GPIO_PIN_2
#define BLUETOOTH_RX_GPIO_Port GPIOD
#define VICE_GYRO_TX_Pin GPIO_PIN_5
#define VICE_GYRO_TX_GPIO_Port GPIOD
#define VICE_GYRO_RX_Pin GPIO_PIN_6
#define VICE_GYRO_RX_GPIO_Port GPIOD
#define BEEP_CONTORL_Pin GPIO_PIN_13
#define BEEP_CONTORL_GPIO_Port GPIOB
#define SERVO1_PWM_Pin GPIO_PIN_7
#define SERVO1_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
