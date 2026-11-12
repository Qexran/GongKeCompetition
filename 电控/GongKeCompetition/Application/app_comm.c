/**
  * @file       app_comm.c
  * @brief      通信模块：各外部串口的接收任务。
  *
  *             各任务阻塞等待对应串口的一帧数据（DMA + IDLE + 二值信号量），
  *             拿到整帧后按各自协议解析（TODO）。
  *             树莓派（USART1）的接收/协议已迁至 app_rpi.c（vRpiTask）；
  *             蓝牙（UART5）的接收/协议已迁至 app_bt.c（vBtTask）。
  *             USART2（原 OPENMV 口）现由**副陀螺口**独占（app_sensors.c::vSensorTaskM0,
  *             挂哪颗由 GYRO_SUB_DEV 选）—— 一个串口只能有一个消费者,
  *             原来的 vOpenmvTask 空 stub 已删。
  */

#include "app_comm.h"
#include "FreeRTOS.h"
#include "task.h"

/* 外部串口（UART4）：预留 */
void vExtTask(void *argument)
{
    /* static 而不上栈: 取满一圈要 512B（usUartDmaTakeCopy 对超容量的 take 是「丢弃
     * 不重发」, 64B 暂存会让 >64B 的帧整帧静默蒸发, 2026-10-09 改; 同
     * gyro_m0_device.c::gyro_m0_read_yaw 的做法——本函数只被单线程调用）。 */
    static uint8_t aucRx[UART_DMA_BUF_SIZE];

    (void)argument;
    for (;;)
    {
        uint16_t usLen = usUartDmaTakeCopy(UART_EXT, aucRx, (uint16_t)sizeof(aucRx),
                                          portMAX_DELAY);

        /* TODO: 解析外部设备数据（aucRx[0..usLen)） */
        (void)usLen;
    }
}
