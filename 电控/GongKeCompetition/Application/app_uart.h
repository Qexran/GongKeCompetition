/**
  * @file       app_uart.h
  * @brief      串口 DMA 层：接收 DMA 循环 + IDLE 中断 + 二值信号量；发送 DMA 环 + TC 中断链。
  *
  *             接收：DMA 直接把数据搬进缓冲区（不占 CPU），USART 检测到总线
  *             空闲（一帧结束）触发 IDLE 中断，中断里算出**本次新增字节数**并释放
  *             二值信号量，任务通过 usUartDmaTakeCopy() 阻塞等待后整段拷走
  *             （数据跨圆环末尾时的两段拼接由驱动吸收，消费者见到的永远连续）。
  *
  *             发送：任务把字节**拷进每路的 TX 环形缓冲**后立刻返回，DMA 后台搬运
  *             （见下面 vUartSendArray 的契约）。硬件侧 TX DMA 流由 app_uart.c 自己配
  *             （不在 CubeMX 的 usart.c 里），详见 .c 里的流映射表。
  */

#ifndef __APP_UART_H
#define __APP_UART_H

#include "main.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* ==================== 串口设备号 ==================== */
typedef enum
{
    UART_ID_RPI    = 0,   /* USART1 树莓派        PA9/PA10  */
    UART_ID_OPENMV = 1,   /* USART2 OPENMV        PD5/PD6   */
    UART_ID_EMM    = 2,   /* USART3 EMM V5 电机   PB10/PB11 */
    UART_ID_EXT    = 3,   /* UART4  外部          PA0/PC11  */
    UART_ID_BT     = 4,   /* UART5  蓝牙          PC12/PD2  */
    UART_ID_HWT101 = 5,   /* USART6 姿态传感器    PC6/PC7   */
    UART_ID_NUM    = 6
} uart_id_t;

/* 每路 DMA 接收缓冲大小（字节） */
#define UART_DMA_BUF_SIZE   512

/* ==================== 发送侧参数 ==================== */
/* 每路 TX 环形缓冲大小（字节）。**必须是 2 的幂** —— 进队/出队用 `& (SIZE-1)` 取模
 * （下面有编译期断言挡着）。统一 512：单次最大调用是 bt_sendf 的 256B；蓝牙
 * `$B,LIST` 那 ~9.8KB 的突发靠「环满背压」消化，不靠把环做大。6 路共 +3KB RAM。 */
#define UART_TX_BUF_SIZE          512u

/* 入队等空间的上限（ms）。超时才丢弃并计数 —— 这是背压，不是失败路径：
 * 正常长突发（$B,LIST）每一段都在这个时间内排空。 */
#define UART_TX_ENQ_TIMEOUT_MS    500u

/* 「在途段卡死」判定（ms）。最长合法段 = min(pending, SIZE-tail) ≤ 512B = 44ms @115200,
 * 300ms 是 6.8× 余量。触发后会被拦腰截断一帧 —— **对端全部有帧界定 + 校验**
 * （RPI 的 SOF/CRC8、蓝牙的 $B...\r\n 行、EMM 的 CRC-8、HWT101 的帧内校验和），
 * 截断帧会被对端丢掉而不会误解析。 */
#define UART_TX_STALL_TIMEOUT_MS  300u

#if ((UART_TX_BUF_SIZE & (UART_TX_BUF_SIZE - 1u)) != 0u)
#error "UART_TX_BUF_SIZE 必须是 2 的幂（进队/出队用 & (SIZE-1) 取模）"
#endif

/* ==================== 接口 ==================== */

/* 应用入口：初始化 6 路串口（被 vAppInit 调用） */
void vAppUartInit(void);

/* 初始化 6 路串口：创建信号量 + 使能 IDLE 中断 + 启动 DMA 循环接收 + 配 TX DMA 流 */
void vUartDmaInit(void);

/* 运行时改某路的波特率（只动 BRR/CR1.UE, RX 循环 DMA 与 IDLE 中断原样保活）。
 * 设计给「陀螺口开机配置」用: 与模块约定的会合波特率上完成寄存器下发后, 让 MCU
 * 本口跟上模块的新波特率。⚠️ 仅任务上下文, 且**只在链路空闲时调**（调用前先
 * ucUartWaitTxDone 把 TX 环排空）—— 调用瞬间在途/在收的字节会变成乱码,
 * 由上层解析器的帧头重同步兜底。 */
void vUartSetBaud(uart_id_t emId, uint32_t ulBaud);

/* IDLE 中断处理（在各 USART/UART 的 IRQ handler 里调用） */
void vUartDmaIdleHandler(UART_HandleTypeDef *pstHuart);

/* TX DMA 流中断统一分发（在 stm32f4xx_it.c 的 USER CODE BEGIN 1 段里被 6 个
 * DMAx_Streamy_IRQHandler 调用）。句柄数组保持 static, 不对外暴露裸全局。 */
void vUartTxDmaIrqHandler(uart_id_t emId);

/* 阻塞等一帧, 并把**本次新增**的字节整段拷进 pucDst（自动处理圆环回绕）:
 *   返回 = 实际拷贝字节数; 超时无数据 / 本次新增超过 usDstCap → 返回 0（丢弃）。
 * ⚠️ 这是读串口的唯一入口 —— 驱动交付的永远是「本次新增字节」。
 *    不要再写「拿缓冲首地址 + 把它当整帧长度」的解析: 那个老写法在圆环回绕之后
 *    必然错位（缓冲首地址装的是本圈最老的数据, 而新数据在写指针处）。
 * ⚠️ 单缓冲: 写者不停覆盖, 消费者必须及时取走; 取走慢于帧率时中间那些帧会被跳过
 *    （ISR 每帧推进读位置, 不做排队）—— 需要背压时得另加环形队列。
 * 调用方缓冲至少要有「单次突发最大字节数」那么大（帧协议的建议 ≥ 最大帧长）。 */
uint16_t usUartDmaTakeCopy(uart_id_t emId, uint8_t *pucDst, uint16_t usDstCap, TickType_t xTimeout);

/* 重新 arm 循环接收 DMA（RX 被 HAL 判为阻塞性错误停机后的恢复手段, 幂等）。
 * 平时不用调 —— `HAL_UART_ErrorCallback`（app_uart.c）已经会自动恢复;
 * 这是给「链路已判掉线」时的兜底自愈（见 app_rpi.c::vRpiTask）。 */
void     vUartReArmRx(uart_id_t emId);

/* ==================== 发送（唯一入口） ====================
 * 把 ulLen 字节拷进该路的 TX 环形缓冲并让 DMA 起发, **返回时数据已入队但未必在线**。
 *
 * 契约（2026-09 由阻塞 HAL_UART_Transmit 改成 DMA 环, 是本次唯一的行为契约变更）:
 *   · 返回 = 「已交给驱动」, 不再是「已在线上」。协议层不受影响 —— 对端不可能在收全
 *     请求之前就先回帧, 所以「发完再等回应」的调用方（RPI / 蓝牙 / ATK 姿态）行为不变。
 *   · 环满时**阻塞背压**（vTaskDelay(1) 轮询, 上限 UART_TX_ENQ_TIMEOUT_MS）,
 *     超时才丢弃并计入 usTxDrop。⚠️ 所以本函数**只能在任务上下文调**, 且
 *     **绝不能在 taskENTER_CRITICAL() 里调**（临界区屏蔽 TIM6/TIM7, 会让背压等待
 *     变成硬挂）。
 *   · 单次 ulLen 超过 UART_TX_BUF_SIZE-1 → 直接丢 + 计数（不进等待循环, 永远等不到）。
 *   · 某个通道的 TX DMA 起不来（HAL_DMA_Init 失败）→ 该路整条退回原来的阻塞发送,
 *     行为与改造前完全一致。
 *   · 真要「等数据真的发完」用 ucUartWaitTxDone()。
 *
 * 顺序保证: 同一路的多次调用按 FIFO 顺序上线; 进队动作在临界区内完成, 两帧不会交错。
 * （vRpiSendFrame 的 s_xRpiTxMutex / vEmmV5SendCmd 的 s_xEmmBusMutex 仍然保留 ——
 *   它们保证的是帧边界与调用方的语义, 只是持锁时间从「整帧线上时间」降到了「memcpy 时间」。） */
void vUartSendArray(uart_id_t emId, const uint8_t *pucData, uint32_t ulLen);

/* 等该路 TX 环排空且没有在途段（= 最后一个字节已经移位出去）。
 * 返回 1 = 已发完; 0 = 超时。⚠️ 仅任务上下文。本次没有任何调用点, 是给将来
 * 「发完必须确认」的场景留的出口（大多数协议不需要, 见上面的契约说明）。 */
uint8_t ucUartWaitTxDone(uart_id_t emId, TickType_t xTimeout);

/* 格式化发送（内部走 vUartSendArray, 不绕过环）。当前无调用点。 */
void vUartPrintf(uart_id_t emId, const char *pcFormat, ...);

/* ==================== 遥测（只读） ====================
 * TX 侧天生是静默失效的重灾区: 加了 DMA 之后「发出去了没有」不再能从函数返回值看出来。
 * 四个数的判读规则（见 CLAUDE.md）:
 *   · ulTxBytes 涨 **且** usTxDmaSegs 涨          → 真在走 TX DMA, TC 链完整;
 *   · ulTxBytes 涨 **但** usTxDmaSegs 不动        → **在走阻塞兜底路径**（TX DMA 没起来）;
 *   · usTxStall > 0                                → 看门狗触发过 / ISR 退过账（TC 中断丢过）;
 *   · usTxDrop  > 0                                → 有字节被丢（环满超时, 或看门狗截断）。
 * 对账不变量: ulTxReqBytes == ulTxBytes + usTxDrop（不成立 = 记账漏了）。 */
uint32_t ulUartGetTxReqBytes(uart_id_t emId);   /* 累计「请求发送」字节 */
uint32_t ulUartGetTxBytes(uart_id_t emId);      /* 累计「真的发出去」字节（含兜底路径） */
uint16_t usUartGetTxDmaSegs(uart_id_t emId);    /* DMA 段完成次数（只由 TxCplt 自增） */
uint16_t usUartGetTxStall(uart_id_t emId);      /* 看门狗/ISR 记账次数 */
uint16_t usUartGetTxDrop(uart_id_t emId);       /* 累计丢弃字节 */
uint16_t usUartGetTxErr(uart_id_t emId);        /* 累计 TX DMA 错误次数 */

/* ==================== 接收遥测（只读） ====================
 * **字节数停涨而 TX 照常 = 本端 RX 已死**, 这是「Pi 说识别成功、单片机全超时」
 * 那类失效唯一看得见的指标（2026-09 加）。 */
uint32_t ulUartGetRxBytes(uart_id_t emId);
uint16_t usUartGetRxErr(uart_id_t emId);
uint16_t usUartGetRxRearmFail(uart_id_t emId);   /* 重装 RX DMA 失败的次数（>0 = 自愈也没救回来, 得查硬件） */

#endif /* __APP_UART_H */
