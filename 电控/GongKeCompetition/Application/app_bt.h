/**
  * @file       app_bt.h
  * @brief      蓝牙模块（UART5）：$B 行协议 + Beam 运行时参数读写 + 机械臂编辑模式。
  *
  *             硬件：PC12=UART5_TX / PD2=UART5_RX, 115200 8N1, DMA 循环 + IDLE 中断,
  *             已在 app_uart.c 注册为 UART_ID_BT（UART_BT）。本模块是它的**唯一读者**。
  *
  *             ⚠️ 新增一条 $B 指令要同步改三处：本文件 + app_bt.c 的分发 +
  *                CLAUDE.md 的「新增蓝牙指令」清单。这是**宿主（PC）协议**，
  *                与树莓派的帧协议（app_rpi.c）无关——**Pi 端不需要任何改动**。
  */

#ifndef __APP_BT_H
#define __APP_BT_H

#include "app_config.h"

/* 蓝牙接收/协议任务（在 freertos.c 创建, 优先级 APP_PRIO_COMM）。
 * 阻塞等 UART5 的一帧 → 行累积 → 按 $B 协议分发。 */
void vBtTask(void *argument);

/* 握手等待让位泵：脚本任务在 ucRpiHandshake 的**阶段 1** 循环里每轮调一次。
 *
 * 单次调用语义，**不是**嵌套循环——嵌套循环会让阶段 1 的 ulWaited 计数冻住，
 * 操作员最需要提示时反而没有提示。
 *   返回 1 = 编辑模式生效, 已在此停住且内部 vTaskDelay(10ms) 过了 10ms 节拍,
 *            调用方**不要再**自己延时（否则变成 20ms 一轮）;
 *   返回 0 = 未停住（编辑模式没开 / 已到退出条件）, 调用方照常自己延时。
 *
 * 自动退出条件见 app_bt.c：树莓派握手成功、或看门狗超时（PC 掉线）。 */
uint8_t vBtEditPump(void);

/* 强制让位给机械臂编辑模式并**停住**, 直到操作员按 MAIN 键（或从 PC 侧退出编辑模式）。
 * 给路线中途插一块「我要手动操纵机械臂」用（脚本动作 STEP_ARM_EDIT_WAIT 的原语）:
 * 停车清了速度, 六个关节交给「机械臂」Tab 点动。细节与坑见 app_bt.c 的实现注释。 */
void vBtArmEditHold(void);

/* 定点格式化 float → "-12.34"（两位小数, 四舍五入, NaN/inf 兜底）。
 * 全工程**不用 %f**（MicroLib 下输出空白, 还多链一坨浮点格式化库）——
 * 别处要往字符串里写 float（如 app_rpi 的 [STM-DBG] 调试串）就用它, 别再抄一份。 */
void bt_fmt_f2(char *pcOut, int iSize, float fVal);

#endif /* __APP_BT_H */
