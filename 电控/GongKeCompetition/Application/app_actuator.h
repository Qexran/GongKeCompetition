/**
  * @file       app_actuator.h
  * @brief      执行器控制：蜂鸣器、继电器（激光）、补光 PWM。
  */

#ifndef __APP_ACTUATOR_H
#define __APP_ACTUATOR_H

#include "app_config.h"

void vAppActuatorInit(void);

/* 蜂鸣器（PB13, **高电平触发** —— 随 2026-09-30 板 V1.2 换脚一起改的, 见 app_actuator.c）
 * vBuzzer* 三条是**阻塞式**直用口（vBuzzerBeep 内部 vTaskDelay, 仅任务上下文）;
 * 禁阻塞的上下文（vRpiTask / 平移精调环）请投**非阻塞**请求: vSetBuzzerReq
 * （模式常量在 app_beam.h, 执行者 = vBuzzerTask）。 */
void vBuzzerOn(void);
void vBuzzerOff(void);
void vBuzzerBeep(uint16_t usOnMs, uint16_t usOffMs, uint8_t ucTimes);

/* 蜂鸣任务（freertos.c 创建, 优先级 APP_PRIO_OLED）: 消费 Beam 的蜂鸣请求槽
 * （Pi 0x0C 命令帧 / 固件各失败点投递）并执行对应鸣叫; 顺带跑「陀螺仪损坏」
 * 看门狗（跑动中姿态帧号停涨 → 失败音, 见本文件 BUZZER_* 段）。 */
void vBuzzerTask(void *argument);

/* 继电器（JIGUANG TRIG） */
void vRelayOn(void);
void vRelayOff(void);

/* 激光射击: 开激光 → 等 ulMs → 关激光（一步到位, 免得路线里排开/延时/关三行）。
 * ⚠️ 内部是 vTaskDelay 让出式延时, 仅任务上下文调用（脚本任务）。 */
void vLaserShoot(uint32_t ulMs);

/* 补光 PWM（TIM9_CH1 @ PE5，频率见 app_actuator.c 的 FILL_LIGHT_PWM_HZ），usDuty 范围 0~1000（千分比） */
void vFillLightSet(uint16_t usDuty);

/* 任务补光的任务类别（vFillLightTaskBegin 的实参; 步骤→类别映射表在 app_script.c） */
#define FILL_TASK_NONE              0u
#define FILL_TASK_ROAD              1u  /* 看路（主摄看黄区） */
#define FILL_TASK_BALL              2u  /* 色球（识别/对位/抓取/排爆） */
#define FILL_TASK_SHOOT             3u  /* 射靶（识别/对位/流控/射击/保持） */
#define FILL_TASK_HOSTAGE           4u  /* 人质（识别/对位/抓取/反恐） */
#define FILL_TASK_QR                5u  /* 二维码（主摄扫码; 扫码副摄不挂 —— 灯照的是主摄那边） */

/* 任务补光: 按任务类别查 app_config.h「任务补光」区（总开关/分任务开关/亮度;
 * 亮度**留空（文件存哨兵 255）= 用默认亮度**、填 0 = 开灯不亮、1~100 = 单独覆盖）
 * 决定开不开、开多亮; 不开就直接返回、一个寄存器都不碰。
 * 返回**实际点亮的占空比**（‰, 0=没开灯）—— 调用方好打「真开了」的调试串, 不被 EN=0 骗。
 * vFillLightTaskEnd 无条件灭灯 —— 「任务执行完关灯」只有这一个出口, 谁开谁收。 */
uint16_t vFillLightTaskBegin(uint8_t ucTask);
void vFillLightTaskEnd(void);

#endif /* __APP_ACTUATOR_H */
