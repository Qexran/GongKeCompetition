/**
  * @file       app_motor.c
  * @brief      底盘电机模块：EMM V5 闭环步进电机（USART3 总线）。
  */

#include "app_motor.h"
#include "FreeRTOS.h"
#include "task.h"

#define EMM_MOTOR_ADDR  0x01    /* EMM V5 电机地址（按实际接线修改） */

void vAppMotorInit(void)
{
    stEmmV5StaticParamTdf stInit;

    stInit.emUartDevNum = UART_EMM;
    stInit.vCallbackFcn = NULL;
    vEmmV5DeviceInit(&stInit);

    /* 上电使能（示例，可按需放到任务里） */
    /* vEmmV5EnControl(EMM_MOTOR_ADDR, 1, 0); */
}

void vMotorTask(void *argument)
{
    (void)argument;

    for (;;)
    {
        /* 处理 EMM V5 接收到的响应帧（接收在串口中断里完成） */
        vEmmV5DevicePeriodExecute();

        /* TODO: 底盘运动控制，调用 vEmmV5VelControl / vEmmV5PosControl */
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
