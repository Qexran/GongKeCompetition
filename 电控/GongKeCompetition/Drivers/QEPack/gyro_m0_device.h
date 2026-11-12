/**
  * @file       gyro_m0_device.h
  * @brief      M0 副陀螺仪（单轴 Z 航向）驱动 —— 5 字节帧 / 5 字节命令。
  *
  *             模块是维特系单轴 yaw 陀螺，接在 USART2（PD5/PD6，板上丝印 VICE_GYRO），
  *             115200 8N1。协议出自台架示例工程，与 pro 兄弟工程的同名驱动逐字节一致。
  *
  *             ⚠️ 与 USART6 上的 ATK-MS901M / HWT101 是**两套完全不同的协议**：
  *                ATK 是 0x55 0x55 帧头 + 变长帧；HWT101 是 0x55 帧头 + 0x53/0x52 +
  *                11 字节。本模块是 0x5A 帧头 + 0xAA/0xBB + 5 字节。别混。
  *
  *             ⚠️ 模块**顺时针为正**，本工程**逆时针为正** → 调用方要乘 -1
  *                （见 app_config.h 的 M0_GYRO_YAW_SIGN）。驱动本身**不取负** ——
  *                符号是调用方的策略，不是协议的一部分。
  */

#ifndef __GYRO_M0_DEVICE_H
#define __GYRO_M0_DEVICE_H

#include "main.h"
#include "app_uart.h"        /* uart_id_t / vUartSendArray / usUartDmaTakeCopy */
#include <stdint.h>

/* ==================== 上报帧（模块 -> MCU），5 字节 ====================
 *   byte0  byte1  byte2  byte3      byte4
 *   0x5A   type   dataL  dataH      sum = (b0+b1+b2+b3) & 0xFF
 *   dataL/dataH 是**小端** int16。 */
#define GYRO_M0_FRAME_HEAD   0x5Au
#define GYRO_M0_FRAME_LEN    5u
#define GYRO_M0_TYPE_RATE    0xAAu   /* 角速度：raw / 32768 * 2000  [°/s] */
#define GYRO_M0_TYPE_YAW     0xBBu   /* 航向角：raw / 32768 * 180   [°], -180 ~ +180 */

/* ==================== 下发帧（MCU -> 模块），5 字节 ====================
 *   0x55  0xAA  reg  dataL  dataH
 *   ⚠️ 末两字节是**数据**，不是校验和 —— 解锁帧的 0x8E 0x5F 只是看着像校验和。 */
#define GYRO_M0_CMD_HEAD1    0x55u
#define GYRO_M0_CMD_HEAD2    0xAAu
#define GYRO_M0_CMD_LEN      5u

/* 寄存器 */
#define GYRO_M0_REG_SAVE     0x00u   /* data: 0x0000=保存 0x0001=恢复出厂 0x00FF=重启 */
#define GYRO_M0_REG_OUT_RATE 0x02u   /* 回传频率档码 */
#define GYRO_M0_REG_BAUD     0x03u   /* 波特率档码 */
#define GYRO_M0_REG_READ_ST  0x04u   /* 读寄存器, 模块回 0x5A 0xCC XX XX SUM */
#define GYRO_M0_REG_BIAS_CAL 0x0Au   /* data: 0x0001=零偏自校准 0x0003=手动刻度标定 */
#define GYRO_M0_REG_UNLOCK   0x13u   /* 写 0x8E5F 解锁；解锁后 20s 内必须改完并保存 */
#define GYRO_M0_REG_YAW_ZERO 0x15u   /* yaw 置零 */

/* 波特率档码（写 0x03 寄存器）。⚠️ 兄弟头文件里的 230400 项与 115200 同为 0x06，
 * 是笔误，别照抄。真实映射只到 115200。 */
#define GYRO_M0_BAUD_4800    0x01u
#define GYRO_M0_BAUD_9600    0x02u
#define GYRO_M0_BAUD_19200   0x03u
#define GYRO_M0_BAUD_38400   0x04u
#define GYRO_M0_BAUD_57600   0x05u
#define GYRO_M0_BAUD_115200  0x06u

/* ==================== 接口 ==================== */

/* 发一条 5 字节命令帧（内部走 vUartSendArray, 不绕过 TX 环）。
 * ⚠️ 仅任务上下文（vUartSendArray 的契约）。 */
void     gyro_m0_send_cmd(uart_id_t emUart, uint8_t ucReg, uint8_t ucDataL, uint8_t ucDataH);

/* 解析器上下文（帧组装状态）——**每个消费者自己持一份**，解析器因此可按实例重入。
 * 调用方把它当不透明状态即可（清零初始化 = 空闲态, static 变量天然满足）。 */
typedef struct
{
    uint8_t aucBuf[GYRO_M0_FRAME_LEN];
    uint8_t ucIdx;
} stGyroM0RxTdf;

/* 纯字节流解析器：喂一个字节，凑满一帧且校验通过时返回帧类型
 * （GYRO_M0_TYPE_YAW / GYRO_M0_TYPE_RATE 并把数值写进 *pfVal），否则返回 0。
 *
 * ⚠️ 状态**全部在 *pstRx 里**（不再是函数级 static）：同一份上下文只能被一个
 *    任务喂（两条流共用一个上下文 = 半帧交错、解出垃圾）。主/副两个陀螺口各持
 *    一份自己的，所以两口都挂 M0 也互不干扰。
 * ⚠️ 返回值**不做符号翻转**：pfVal 是模块原始读数。
 * 重同步策略：帧头之后收到非 0xAA/0xBB 时，若该字节本身就是 0x5A 则当作新帧头
 * 重启状态机，否则整帧丢弃重新等帧头。 */
uint8_t  gyro_m0_parse_byte(stGyroM0RxTdf *pstRx, uint8_t ucByte, float *pfVal);

/* 阻塞读一帧航向角（便利封装：自检 / 开机确认模块在不在）。
 * 返回 1 = 读到, 0 = 超时。⚠️ 仅任务上下文。
 * ⚠️ 它**自带一份私有上下文**（与 parse_byte 的调用方不共享）⇒ 本函数仍只允许
 *    单任务调用, 且不能与喂同一路串口的那个任务并发（本工程 = 无调用点）。 */
uint8_t  gyro_m0_read_yaw(uart_id_t emUart, float *pfYaw, uint32_t ulTimeoutMs);

/* 把「回传频率 Hz」就近映射到模块支持的 15 个档位码。
 * 返回 1 = 找到（写进 *pucCode），0 = *pucCode 不是合法输入（>0 才有意义）。 */
uint8_t  gyro_m0_rate_code_from_hz(float fHz, uint8_t *pucCode);

/* ==================== 配置命令（都会先发解锁帧）====================
 * ⚠️ 全部**只改模块 RAM、不发保存帧** —— 断电即回模块自己的默认值。
 *    这是有意的：配置项里有一个「波特率」，填错会让本路直接变聋；不写 Flash
 *    就意味着给模块重新上电就能自愈，不必拿上位机去救。
 * ⚠️ 全部仅任务上下文（内部有 vTaskDelay 间隔）。 */

void     gyro_m0_set_rate_hz(uart_id_t emUart, float fHz);        /* 解锁 -> 写 0x02 */
void     gyro_m0_set_baud_code(uart_id_t emUart, uint8_t ucCode); /* 解锁 -> 写 0x03。⚠️ 改它必须同步改 MCU 侧波特率 */
void     gyro_m0_yaw_zero(uart_id_t emUart, uint8_t ucSave);      /* 解锁 -> 置零 [-> 保存] */

/* 开机一次性配置：解锁 -> 写频率 [-> 写波特率] [-> yaw 置零]，全程不保存。
 * ucApplyBaud=0 时不下发波特率（默认，免得误改把自己弄聋）；
 * ucZeroOnBoot=1 时发一次 yaw 置零，让「开机朝向 = 0°」的前提成立。 */
void     gyro_m0_apply_boot_config(uart_id_t emUart, float fHz, uint8_t ucBaudCode,
                                   uint8_t ucApplyBaud, uint8_t ucZeroOnBoot);

/* ==================== 维护命令（本次无调用点，留给将来）====================
 * ⚠️⚠️ gyro_m0_bias_cal 要模块静止约 21 秒，期间**阻塞调用它的任务**。
 *    **绝不能从 vSensorTask / vSensorTaskM0 里调** —— 那 21s 里没人 vSetYawFresh()，
 *    ulGetYawFrameCount() 停涨 → app_line.c 的 LINE_GYRO_STALE_MS(600ms) 会判 IMU
 *    停摆，路线上每一条陀螺闭环的步骤都会失去航向。
 *    只能放在不发布 yaw 的任务里（vBtTask 的某个 $B 分支，或 OLED 菜单投递给
 *    脚本任务的那条路）。 */
void     gyro_m0_bias_cal(uart_id_t emUart);      /* 解锁 -> 校准 -> 等 21s -> 保存 */
void     gyro_m0_factory_reset(uart_id_t emUart); /* 解锁 -> 恢复出厂 -> 保存 */
void     gyro_m0_reboot(uart_id_t emUart);        /* 解锁 -> 重启 */

#endif /* __GYRO_M0_DEVICE_H */
