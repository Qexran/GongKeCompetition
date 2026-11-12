/**
  * @file       app_tick.c
  * @brief      应用时基：2 个硬件定时器，分别驱动舵机平滑 / 服务层。
  *
  *              均 1kHz（1ms）。TIM7 在 APB1（84MHz），TIM1 在 APB2（168MHz）。
  *              注意：视觉 PID 已移出中断，由 vPidTask（app_pid.c）按 50ms 节拍驱动
  *              —— 周期处理内部有阻塞 UART 发送与延时，不能放中断上下文。
  *              OLED 的 I2C 刷新不在中断里做，只在这里做快速按键扫描，
  *              OLED 刷新仍由 OLED 任务按 50ms 节拍完成。
  */

#include "app_tick.h"
#include "tim.h"

/* 两个硬件定时器句柄 */
TIM_HandleTypeDef htim7;   /* 舵机平滑调速 */
TIM_HandleTypeDef htim1;   /* 服务层（按键） */

/* 中断频率：1kHz（1ms） */
#define TICK_FREQ_HZ        1000u

void vAppTickInit(void)
{
    /* ---- TIM7：舵机平滑调速 ---- */
    __HAL_RCC_TIM7_CLK_ENABLE();
    htim7.Instance = TIM7;
    htim7.Init.Prescaler = 84 - 1;                  /* 84MHz / 84 = 1MHz */
    htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim7.Init.Period = (1000000u / TICK_FREQ_HZ) - 1;   /* 1MHz / 1000 = 1kHz */
    htim7.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim7);
    HAL_NVIC_SetPriority(TIM7_IRQn, APP_TIM_PRIO_SERVO, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    HAL_TIM_Base_Start_IT(&htim7);

    /* ---- TIM1：服务层（按键扫描） ---- */
    __HAL_RCC_TIM1_CLK_ENABLE();
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 168 - 1;                 /* 168MHz / 168 = 1MHz */
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = (1000000u / TICK_FREQ_HZ) - 1;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim1);
    HAL_NVIC_SetPriority(TIM1_UP_TIM10_IRQn, APP_TIM_PRIO_SERVICE, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
    HAL_TIM_Base_Start_IT(&htim1);
}

/* TIM7 中断回调：舵机平滑调速 */
void vAppServoTick(void)
{
    uint8_t i;
    for (i = 0; i < SERVO_DEV_NUM; i++)
    {
        vServoDevicePeriodExecute((emServoDevNumTdf)i);
    }
}

/* TIM1 中断回调：服务层（按键扫描，快速、非阻塞） */
void vAppServiceTick(void)
{
    uint8_t i;
    for (i = 0; i < KEY_DEV_NUM; i++)
    {
        vKeyDevicePeriodExecute((emKeyDevNumTdf)i);
    }
}
