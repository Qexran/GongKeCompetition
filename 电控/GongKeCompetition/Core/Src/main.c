/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "freertos_app.h"
#include "app_main.h"
#include "emm_v5_device.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  不依赖中断的忙等延时（DWT 周期计数器）—— 备用，未启用
  * @note   HAL_Delay 依赖 TIM6 中断递增 uwTick；本函数直接数 CPU 周期，
  *         在中断异常（tick 停走）的场景下也能按时返回，用于隔离定位问题。
  */
//static void vTestBusyDelayMs(uint32_t ulMs)
//{
//  uint32_t ulStart;
//  uint32_t ulCycles = ulMs * (SystemCoreClock / 1000u);
//
//  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   /* 使能 DWT */
//  DWT->CYCCNT = 0;
//  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
//  ulStart = DWT->CYCCNT;
//  while ((DWT->CYCCNT - ulStart) < ulCycles)
//  {
//    __NOP();
//  }
//}

/**
  * @brief  测试函数：关闭 EMM V5 四个电机的"上电自动触发回零"功能（一次性运行）
  * @note   对应手册命令 0x4C 0xAE（修改原点回零参数），整包写入 14 字节参数：
  *           - 其余回零参数按手册默认值写入（单圈就近/CW/30RPM/超时10s/碰撞检测300RPM/800mA/60ms）
  *           - bSave=1   写入 ROM，掉电保存
  *           - bPotFlag=0 关闭"上电自动触发回零"，只保留手动触发回零
  *         跑完一次后注释掉 main 里的调用即可（本函数可留作备用）。
  */
//static void vTestDisableEmmPowerOnHoming(void)
//{
//  uint8_t aucAddr[4] = {4, 3, 2, 1};    /* 四个 EMM V5 电机地址（USART3 总线） */
//  uint8_t ucIdx;

//  for (ucIdx = 0; ucIdx < 4; ucIdx++)
//  {
//    vEmmV5OriginModifyParams(aucAddr[ucIdx],
//                             1,                              /* 是否存储：存 ROM */
//                             emEmmV5OrgMode_SingleNear,      /* 回零模式：单圈就近 */
//                             emEmmV5Dir_CW,                  /* 回零方向：顺时针 */
//                             30,                             /* 回零转速 30 RPM */
//                             10000,                          /* 回零超时 10000 ms */
//                             300,                            /* 碰撞检测转速 300 RPM */
//                             800,                            /* 碰撞检测电流 800 mA */
//                             60,                             /* 碰撞检测时间 60 ms */
//                             0);                             /* 上电自动回零：关闭 */
//    vTestBusyDelayMs(200);                /* 写 ROM 需时间，错开发下一台 */
//  }
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */

  /* 初始化应用层各模块（串口/按键/舵机/电机/执行器/OLED/时基） */
  vAppInit();
	

  /* 一次性测试：关闭 EMM V5 四个电机的上电自动回零（跑完注释掉） */
  //vTestDisableEmmPowerOnHoming();
  /* 创建 FreeRTOS 任务并启动调度器（此后不再返回） */
  MX_FREERTOS_Init();
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  extern void vAppServoTick(void);
  extern void vAppServiceTick(void);

  if (htim->Instance == TIM7)
  {
    vAppServoTick();      /* 舵机平滑调速 */
  }
  else if (htim->Instance == TIM1)
  {
    vAppServiceTick();    /* 服务层（按键扫描） */
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
