/**
  * @file       app_uart.c
  * @brief      串口 DMA 层实现：接收 DMA 循环 + IDLE 中断 + 二值信号量；
  *                                  发送 DMA 环 + TC 中断链续发。
  *
  *             接收思路（江科大 FreeRTOS·USART-DMA 方案，HAL 适配）：
  *               - HAL_UART_Receive_DMA 以循环模式把数据搬进缓冲区（只启动一次，永不重装）；
  *               - USART 检测到总线空闲触发 IDLE 中断，在中断里把当前写指针位置
  *                 与「上次产出的位置」相减，得到**本次新增字节数**并释放信号量；
  *               - 任务 usUartDmaTakeCopy() 阻塞等待，把这段字节整段拷走（回绕由驱动吸收）。
  *             ⚠️ 循环模式下「缓冲大小 - DMA 剩余计数」是**写指针位置**而不是帧长，
  *                拿它当帧长会让消费者只在圆环起点对齐时才碰巧读对（老 bug，2026-09 修）。
  *             注意：单缓冲，任务应尽快取走数据，避免下一帧覆盖。
  *
  *             发送思路（2026-09 加，替换原来的忙等 HAL_UART_Transmit）：
  *               - 每路一个 TX 环形缓冲，vUartSendArray 把字节**拷进去**就返回（见 .h 契约）；
  *               - vUartTxKick 从 tail 取一段**连续**内存（DMA 不会绕回）交给
  *                 HAL_UART_Transmit_DMA（NORMAL 模式）；
  *               - 段发完 → DMA 流 TC 中断 → UART_DMATransmitCplt 置 CR1.TCIE →
  *                 **USART TC 中断** → HAL_UART_TxCpltCallback（本文件）→ 推进 tail 并续发下一段。
  *             ⚠️ 上面那条链里 DMA 流 IRQHandler 是**必需的**：NORMAL 模式的
  *                UART_DMATransmitCplt（hal_uart.c:3015-3030）只置 TCIE、**不调用户回调**，
  *                没有流中断就永远等不到 TxCplt。6 个流 IRQ 在 stm32f4xx_it.c 的
  *                USER CODE BEGIN 1 段里分发到本文件的 vUartTxDmaIrqHandler()。
  *             ⚠️ TX DMA 流由本文件自己配（g_astUartTxMap），**不在 CubeMX 生成的
  *                usart.c 里** —— 这样 CubeMX 重生成冲不掉，.ioc 也不用改。
  */

#include "app_uart.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/* ==================== 通道描述 ==================== */
typedef struct
{
    UART_HandleTypeDef *pstHuart;                          /* HAL 句柄 */

    /* ---------------- 接收侧 ---------------- */
    __attribute__((aligned(4))) uint8_t aucBuf[UART_DMA_BUF_SIZE]; /* DMA 接收缓冲 */
    volatile uint16_t usStart;                             /* 本次新增数据的起始偏移（ISR 写） */
    volatile uint16_t usLen;                               /* **本次新增**字节数（ISR 写） */
    uint16_t usLastPos;                                    /* 上次产出时的圆环写指针（仅 ISR 推进） */
    volatile uint32_t ulRxBytes;                           /* 累计收字节数（遥测用: 停涨 = 本端变聋） */
    volatile uint16_t usRxErr;                             /* 累计接收错误次数（遥测用） */
    volatile uint16_t usRxRearmFail;                       /* 累计「重装 RX DMA 失败」次数（遥测用） */
    volatile uint16_t usRxUnstuck;                         /* 累计「ABORT 卡死被强制复位」次数（遥测用;
                                                              非零 = 踩到过 hal_dma 循环流 Abort_IT 死链, 见
                                                              vUartDmaIdleHandler 的卡死检测） */
    SemaphoreHandle_t xSem;                                /* 帧到达信号量 */

    /* ---------------- 发送侧（DMA 环） ----------------
     * 不变量: huart->gState == HAL_UART_STATE_READY  ⟺  ucTxBusy == 0  ⟺  无在途段。
     * 两者不一致就是 bug 现场。
     * 对账不变量: ulTxReqBytes == ulTxBytes + usTxDrop。 */
    __attribute__((aligned(4))) uint8_t aucTxBuf[UART_TX_BUF_SIZE]; /* TX 环形缓冲 */
    volatile uint16_t usTxHead;                            /* 任务侧写指针 */
    volatile uint16_t usTxTail;                            /* 已被 DMA 取走的段末（ISR 推） */
    volatile uint16_t usTxPending;                         /* 环内未发出字节数 */
    volatile uint16_t usTxCurrentLen;                      /* 本次在途段长（ISR 用它推进 tail） */
    volatile uint8_t  ucTxBusy;                            /* 1 = 有在途段（兼作 TX 引擎的所有权锁） */
    volatile uint8_t  ucTxUseDma;                          /* 0 = 该路没起 TX DMA → 走原阻塞发送 */
    volatile uint8_t  ucTxRecoverReq;                      /* ISR 请求任务侧做 HAL 恢复 */
    volatile uint32_t ulTxStartTick;                       /* 在途段起始 tick（看门狗用） */
    volatile uint32_t ulTxReqBytes;                        /* 累计「请求发送」字节 */
    volatile uint32_t ulTxBytes;                           /* 累计「真的发出去」字节 */
    volatile uint16_t usTxDmaSegs;                         /* DMA 段完成次数（只由 TxCplt 自增） */
    volatile uint16_t usTxStall;                           /* 看门狗触发 / ISR 退账次数 */
    volatile uint16_t usTxDrop;                            /* 累计丢弃字节 */
    volatile uint16_t usTxErr;                             /* 累计 TX DMA 错误次数 */
} stUartDmaChTdf;

static stUartDmaChTdf g_astUartCh[UART_ID_NUM];

/* ==================== TX DMA 流映射 ====================
 * 与 6 路 RX 已占的流（DMA2_S2 / DMA1_S5 / DMA1_S1 / DMA1_S2 / DMA1_S0 / DMA2_S1）
 * **完全不相交**。改了这张表就要同步改 stm32f4xx_it.c 里那 6 个 IRQHandler 的名字。 */
typedef struct
{
    DMA_Stream_TypeDef *pstStream;
    uint32_t            ulChannel;
    IRQn_Type           emIrq;
} stUartTxDmaMapTdf;

static const stUartTxDmaMapTdf g_astUartTxMap[UART_ID_NUM] =
{
    { DMA2_Stream7, DMA_CHANNEL_4, DMA2_Stream7_IRQn },  /* 0 UART_ID_RPI    USART1_TX */
    { DMA1_Stream6, DMA_CHANNEL_4, DMA1_Stream6_IRQn },  /* 1 UART_ID_OPENMV USART2_TX */
    { DMA1_Stream3, DMA_CHANNEL_4, DMA1_Stream3_IRQn },  /* 2 UART_ID_EMM    USART3_TX */
    { DMA1_Stream4, DMA_CHANNEL_4, DMA1_Stream4_IRQn },  /* 3 UART_ID_EXT    UART4_TX  */
    { DMA1_Stream7, DMA_CHANNEL_4, DMA1_Stream7_IRQn },  /* 4 UART_ID_BT     UART5_TX  */
    { DMA2_Stream6, DMA_CHANNEL_5, DMA2_Stream6_IRQn },  /* 5 UART_ID_HWT101 USART6_TX */
};

static DMA_HandleTypeDef g_astUartTxDma[UART_ID_NUM];

/* ==================== 发送侧内部函数 ==================== */
static void vUartTxDrainBlocking(stUartDmaChTdf *pstCh);
static void vUartTxRecoverIfNeeded(stUartDmaChTdf *pstCh);
static uint8_t ucUartDmaUnstickRx(UART_HandleTypeDef *pstHuart);   /* RX ABORT 卡死解卡（见其定义处注释） */

/* TX DMA 单路初始化。
 * ⚠️ 放在这里而不是 CubeMX 生成的 usart.c:
 *    ① CubeMX 重生成冲不掉（本工程 .ioc 里只有 RX 的 6 条 Dma.Request, 一条 TX 都没有）;
 *    ② 句柄数组得以保持 static, 不新增跨模块裸全局。
 * HAL_DMA_Init 是**自包含**的（hal_dma.c:170: 自己算 StreamBaseAddress/StreamIndex、
 * 自己清标志、不调 HAL_DMA_MspInit），只要求 DMA1/DMA2 时钟已开 —— MX_DMA_Init（dma.c）已开,
 * 这里再幂等开一次, 是为了挡住「哪天 CubeMX 把 MX_DMA_Init 从 main 里拿掉」那种情况。
 * 任一步失败 → ucTxUseDma = 0 → vUartSendArray 整条退回阻塞发送（行为 = 改造前）。 */
static void vUartTxDmaInitOne(uart_id_t emId)
{
    const stUartTxDmaMapTdf *pstMap = &g_astUartTxMap[emId];
    DMA_HandleTypeDef       *pstDma = &g_astUartTxDma[emId];

    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    pstDma->Instance                 = pstMap->pstStream;
    pstDma->Init.Channel             = pstMap->ulChannel;
    pstDma->Init.Direction           = DMA_MEMORY_TO_PERIPH;
    pstDma->Init.PeriphInc           = DMA_PINC_DISABLE;
    pstDma->Init.MemInc              = DMA_MINC_ENABLE;
    pstDma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    pstDma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
    /* ⚠️ 必须是 NORMAL: CIRCULAR 会在发完之后自动重头再来一遍, 等于无限重发同一段。 */
    pstDma->Init.Mode                = DMA_NORMAL;
    pstDma->Init.Priority            = DMA_PRIORITY_MEDIUM;
    pstDma->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;

    if (HAL_DMA_Init(pstDma) != HAL_OK)
    {
        g_astUartCh[emId].ucTxUseDma = 0u;
        return;
    }

    /* ⚠️ 必须在 HAL_DMA_Init 之后: 本宏会写 pstDma->Parent, 而 UART_DMATransmitCplt 要经
     *    hdma->Parent 拿回 huart —— 写反了那里就是 NULL 解引用（HardFault）。 */
    __HAL_LINKDMA(g_astUartCh[emId].pstHuart, hdmatx, g_astUartTxDma[emId]);

    /* ⚠️ 优先级必须是 6。理由**不是** FreeRTOS 的 FromISR 约束（本 ISR 里没有任何
     *    FromISR 调用）, 而是: taskENTER_CRITICAL() 只屏蔽**优先级数值 ≥5** 的中断 ——
     *    设成 1~4 就能打断任务侧临界区里的环形缓冲 memcpy, 与 usTxHead/usTxPending 竞争,
     *    **静默发出半截/重复数据, 随机且极难复现**。
     *    取 6 而不是 5: 与 USART 中断同级 → 互不抢占, 避免 HAL 里非原子的
     *    ATOMIC_SET_BIT(CR1.TCIE) 与 HAL_UART_IRQHandler 自己的寄存器写互相覆盖。 */
    HAL_NVIC_SetPriority(pstMap->emIrq, 6u, 0u);
    HAL_NVIC_EnableIRQ(pstMap->emIrq);

    g_astUartCh[emId].ucTxUseDma = 1u;
}

/* TX DMA 流中断统一分发（stm32f4xx_it.c 的 USER CODE BEGIN 1 段里 6 个
 * DMAx_Streamy_IRQHandler 各调一次, 只传语义 id —— 句柄数组不外露）。 */
void vUartTxDmaIrqHandler(uart_id_t emId)
{
    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return;
    }
    HAL_DMA_IRQHandler(&g_astUartTxDma[emId]);
}

/* 起发下一段。test-and-set 在临界区里, **所有 HAL 调用都在临界区外**。
 * 可从任务（ucFromIsr=0）与 TxCplt 中断（ucFromIsr=1）两处调用。
 * ⚠️ ISR 分支不需要临界区: 任务不可能在 ISR 执行期间恢复执行, 而任务侧的所有读写
 *    都在 taskENTER_CRITICAL() 里（它屏蔽优先级 ≥5 的本 ISR）。 */
static void vUartTxKick(stUartDmaChTdf *pstCh, uint8_t ucFromIsr)
{
    uint16_t usLen    = 0u;
    uint8_t *pucSrc   = NULL;
    uint8_t  ucStart  = 0u;

    if (ucFromIsr == 0u)
    {
        taskENTER_CRITICAL();
    }

    if ((pstCh->ucTxBusy == 0u) && (pstCh->usTxPending > 0u))
    {
        /* 只能发「从 tail 到环尾」这一段连续内存（DMA 不会绕回）; 剩下的等下一段。 */
        usLen = (uint16_t)(UART_TX_BUF_SIZE - pstCh->usTxTail);
        if (usLen > pstCh->usTxPending)
        {
            usLen = pstCh->usTxPending;
        }
        pstCh->ucTxBusy       = 1u;
        pstCh->usTxCurrentLen = usLen;
        pstCh->ulTxStartTick  = HAL_GetTick();      /* 只是读 uwTick, 不会循环, 临界区里安全 */
        pucSrc                = &pstCh->aucTxBuf[pstCh->usTxTail];
        ucStart               = 1u;
    }

    if (ucFromIsr == 0u)
    {
        taskEXIT_CRITICAL();
    }

    if (ucStart == 0u)
    {
        return;
    }

    if (HAL_UART_Transmit_DMA(pstCh->pstHuart, pucSrc, usLen) == HAL_OK)
    {
        return;                                     /* 成功: 等 TC 中断推进 tail */
    }

    /* ---------------- 起发失败 ---------------- */
    if (ucFromIsr != 0u)
    {
        /* ISR 分支: 只退记账, **绝不做 HAL 恢复、绝不阻塞发送**
         * （见 HAL_UART_ErrorCallback 里「prio 6 的 ISR 里为什么不能调 AbortTransmit」）。 */
        pstCh->usTxDrop      += usLen;
        pstCh->usTxTail       = (uint16_t)((pstCh->usTxTail + usLen) & (UART_TX_BUF_SIZE - 1u));
        pstCh->usTxPending   -= usLen;
        pstCh->usTxCurrentLen = 0u;
        pstCh->ucTxBusy       = 0u;
        pstCh->usTxErr++;
        pstCh->ucTxRecoverReq = 1u;                 /* 任务侧下次进 vUartSendArray 时做完整恢复 */
        return;
    }

    /* 任务分支: 先证明「没有任何 DMA 段在途」, 才允许走阻塞兜底。
     * ⚠️ 不能写成「起发失败就直接阻塞发」: HAL_UART_Transmit_DMA 的失败面里
     *    「gState != READY（上一段还在飞）→ HAL_BUSY」这一种**有 DMA 在途**, 那时阻塞发
     *    会被 gState==BUSY_TX 挡成 HAL_BUSY → **整环数据静默丢掉**。
     * HAL_UART_AbortTransmit 返回 HAL_OK 就意味着流已停、State/Lock/gState 都回了 READY,
     * 此时阻塞发是安全的。 */
    if (HAL_UART_AbortTransmit(pstCh->pstHuart) == HAL_OK)
    {
        /* 在途那一段已经被 abort 截断了, 帧完整性已失 → 丢掉它（对端会按 CRC/行尾丢弃
         * 截断帧）, 再从 tail 把剩下的整段阻塞发完。
         * ⚠️ 退账必须进临界区（2026-10-09 修）: HAL_UART_ErrorCallback 的 TX 分支（prio 6）
         *    对同一组字段做**完全相同的退账** —— abort 成功后 gState 已回 READY 而 ucTxBusy
         *    仍置着, 那道 ISR 门（ucTxBusy && gState!=BUSY_TX）恰好放行; 不互斥的话错误
         *    中断插进「读 usTxCurrentLen → 逐字段更新」之间就是双退账: usTxPending 双减
         *    下溢 → vUartSendArray 的 usFree 同样下溢 → 512B 环上 memcpy 越界写。 */
        uint16_t usLost;

        taskENTER_CRITICAL();
        usLost                = pstCh->usTxCurrentLen;
        pstCh->usTxDrop      += usLost;
        pstCh->usTxTail       = (uint16_t)((pstCh->usTxTail + usLost) & (UART_TX_BUF_SIZE - 1u));
        pstCh->usTxPending   -= usLost;
        pstCh->usTxCurrentLen = 0u;
        pstCh->ucTxBusy       = 0u;
        pstCh->usTxErr++;
        taskEXIT_CRITICAL();

        vUartTxDrainBlocking(pstCh);
        return;
    }

    /* Abort 也失败（内部 5ms 超时）: 丢掉在途段、清 busy, 交看门狗重试; **绝不阻塞发**
     * —— 流可能还在跑, 阻塞发的数据会与 DMA 在线上交错。（退账同样进临界区, 理由同上） */
    {
        uint16_t usLost2;

        taskENTER_CRITICAL();
        usLost2               = pstCh->usTxCurrentLen;
        pstCh->usTxDrop      += usLost2;
        pstCh->usTxTail       = (uint16_t)((pstCh->usTxTail + usLost2) & (UART_TX_BUF_SIZE - 1u));
        pstCh->usTxPending   -= usLost2;
        pstCh->usTxCurrentLen = 0u;
        pstCh->ucTxBusy       = 0u;
        pstCh->usTxStall++;
        taskEXIT_CRITICAL();
    }
}

/* 任务上下文的兜底: 按「tail 到环尾」的连续段用阻塞 HAL_UART_Transmit 把环排空。
 * ⚠️ 只在**已确认没有 DMA 段在途**之后调用（ucTxUseDma==0, 或 AbortTransmit 成功）。
 * ⚠️ 绝不能被 ISR 调到 —— HAL_UART_Transmit 是 HAL_MAX_DELAY 忙等。 */
static void vUartTxDrainBlocking(stUartDmaChTdf *pstCh)
{
    for (;;)
    {
        uint16_t usLen;
        uint16_t usTail;

        taskENTER_CRITICAL();
        usTail = pstCh->usTxTail;
        usLen  = (uint16_t)(UART_TX_BUF_SIZE - usTail);
        if (usLen > pstCh->usTxPending)
        {
            usLen = pstCh->usTxPending;
        }
        if (usLen == 0u)
        {
            taskEXIT_CRITICAL();
            return;
        }
        taskEXIT_CRITICAL();

        /* 必须在临界区外: 这是忙等, 临界区里会屏蔽 TIM6/TIM7（同铁律 5/6）。 */
        HAL_UART_Transmit(pstCh->pstHuart, &pstCh->aucTxBuf[usTail], usLen, HAL_MAX_DELAY);

        taskENTER_CRITICAL();
        pstCh->usTxTail     = (uint16_t)((pstCh->usTxTail + usLen) & (UART_TX_BUF_SIZE - 1u));
        pstCh->usTxPending -= usLen;
        pstCh->ulTxBytes   += usLen;
        taskEXIT_CRITICAL();
    }
}

/* 任务上下文: 把 TX 状态机从两类故障里救回来。
 *   ① ISR 已记好账、只差把 HAL 拉回 READY（ucTxRecoverReq）
 *   ② TC 中断丢失 → ucTxBusy 永远卡住（看门狗按 ulTxStartTick 判）
 * ⚠️ HAL_UART_AbortTransmit 必须在**临界区外**调: 它内部会走 HAL_DMA_Abort 的
 *    `while((CR & DMA_SxCR_EN) != RESET){ if (HAL_GetTick() - t0 > HAL_TIMEOUT_DMA_ABORT) ... }`
 *    超时循环（hal_dma.c:544-560, 超时常量 5ms）, 而临界区把 BASEPRI 抬到 5 会连 TIM6
 *    （优先级 15 = HAL 时基）一起屏蔽 → uwTick 停走 → 那个循环**永不超时** → 整机挂死。 */
static void vUartTxRecoverIfNeeded(stUartDmaChTdf *pstCh)
{
    uint8_t  ucNeed = 0u;
    uint32_t ulNow;

    if (pstCh->ucTxUseDma == 0u)
    {
        return;
    }
    /* ⚠️ 时间戳读进临界区再比: ulTxStartTick 由 **prio 6 的 TxCplt ISR** 在新段认领时写,
     * 而临界区外的 HAL_GetTick() 与随后进临界区之间**能被那个 ISR 抢占** ——
     * 若这期间某段完成并认领了新段, 且 uwTick 恰好 +1, 就有
     * `ulTxStartTick == ulNow + 1`, 无符号相减得 0xFFFFFFFF → **误判"TC 中断丢失"**、
     * 把一段完全健康的在途数据丢掉并**拦腰截断**（对端收到半截帧, 且 usTxStall 说谎）。
     * 区内只有一次读 + 两次比较, 代价可忽略。 */
    taskENTER_CRITICAL();
    ulNow = HAL_GetTick();                          /* 在区内取, 与 ulTxStartTick 同源同时刻 */
    if (pstCh->ucTxRecoverReq != 0u)
    {
        pstCh->ucTxRecoverReq = 0u;
        ucNeed = 1u;                                /* ISR 已退好账, 只差把 HAL 拉回 READY */
    }
    else if ((pstCh->ucTxBusy != 0u) &&
             ((uint32_t)(ulNow - pstCh->ulTxStartTick) > UART_TX_STALL_TIMEOUT_MS))
    {
        uint16_t usLost = pstCh->usTxCurrentLen;    /* TC 中断丢失: 丢掉在途段 */
        pstCh->usTxDrop      += usLost;
        pstCh->usTxTail       = (uint16_t)((pstCh->usTxTail + usLost) & (UART_TX_BUF_SIZE - 1u));
        pstCh->usTxPending   -= usLost;
        pstCh->usTxCurrentLen = 0u;
        pstCh->ucTxBusy       = 0u;
        pstCh->usTxStall++;
        ucNeed = 1u;
    }
    taskEXIT_CRITICAL();

    if (ucNeed != 0u)
    {
        /* 这是 TX 故障的**唯一解药**, 同时治好两种:
         *   ① TC 中断丢失留下的 ucTxBusy 卡死;
         *   ② DME/FE 错误把 hdmatx 留在 State=BUSY + Lock=LOCKED 的「中毒」状态
         *      —— 此后 HAL_DMA_Start_IT 的 __HAL_LOCK 永远返回 HAL_BUSY, 该路 TX 永久哑掉。
         * 它只碰 TX（CR1.TXEIE|TCIE / CR3.DMAT / hdmatx / gState）, **不碰 CR3.DMAR /
         * hdmarx / RxState**（hal_uart.c:1953-1990）—— 所以在 RX 循环 DMA 跑着的同一 UART
         * 上调它, RX 一个字节都不受影响。
         * ⚠️ 它返回 HAL_TIMEOUT 时**不执行**结尾的 gState = READY（hal_uart.c:1977 提前
         *    return）→ 通道仍是死的, 所以这里不假装恢复, 由下次 vUartSendArray 再战。 */
        (void)HAL_UART_AbortTransmit(pstCh->pstHuart);
    }
}

/* ==================== 初始化 ==================== */
void vAppUartInit(void)
{
    vUartDmaInit();
}

void vUartDmaInit(void)
{
    g_astUartCh[UART_ID_RPI].pstHuart    = &huart1;
    g_astUartCh[UART_ID_OPENMV].pstHuart = &huart2;
    g_astUartCh[UART_ID_EMM].pstHuart    = &huart3;
    g_astUartCh[UART_ID_EXT].pstHuart    = &huart4;
    g_astUartCh[UART_ID_BT].pstHuart     = &huart5;
    g_astUartCh[UART_ID_HWT101].pstHuart = &huart6;

    for (uint8_t ucIdx = 0; ucIdx < UART_ID_NUM; ucIdx++)
    {
        /* ---- 接收侧 ---- */
        g_astUartCh[ucIdx].usStart   = 0;
        g_astUartCh[ucIdx].usLen     = 0;
        g_astUartCh[ucIdx].usLastPos = 0;
        g_astUartCh[ucIdx].xSem  = xSemaphoreCreateBinary();
        configASSERT(g_astUartCh[ucIdx].xSem != NULL);

        /* ---- 发送侧: .bss 本来就是 0, 这里显式写一遍让不变量一眼可见 ---- */
        g_astUartCh[ucIdx].usTxHead       = 0u;
        g_astUartCh[ucIdx].usTxTail       = 0u;
        g_astUartCh[ucIdx].usTxPending    = 0u;
        g_astUartCh[ucIdx].usTxCurrentLen = 0u;
        g_astUartCh[ucIdx].ucTxBusy       = 0u;
        g_astUartCh[ucIdx].ucTxRecoverReq = 0u;
        g_astUartCh[ucIdx].ucTxUseDma     = 0u;     /* 先置 0, InitOne 成功后才置 1 */

        vUartTxDmaInitOne((uart_id_t)ucIdx);

        /* 使能 IDLE 空闲中断（判断一帧接收完成） */
        __HAL_UART_ENABLE_IT(g_astUartCh[ucIdx].pstHuart, UART_IT_IDLE);

        /* 启动 DMA 循环接收（Circular 已在 HAL_UART_MspInit 配置好） */
        if (HAL_UART_Receive_DMA(g_astUartCh[ucIdx].pstHuart,
                                 g_astUartCh[ucIdx].aucBuf,
                                 UART_DMA_BUF_SIZE) != HAL_OK)
        {
            Error_Handler();
        }
    }
}

/* ==================== 运行时改波特率 ====================
 * 只为「陀螺口开机配置」设计: 与模块在 115200（会合波特率）上完成寄存器下发、并告知
 * 模块新波特率之后, 让 MCU 本口跟着换挡。
 *
 * ⚠️ 刻意走 HAL_UART_Init 而不是手写 BRR/DeInit:
 *   · CR1/CR2/CR3 在 UART_SetConfig 里都是**读-改-写**（只动本次配置的位）——
 *     RX 循环 DMA 的 CR3.DMAR、IDLE 中断的 CR1.IDLEIE 原样保活, DMA 环不用重装;
 *   · gState 是 READY（≠RESET）⇒ 不会重入 MspInit, 引脚/时钟不被二次初始化;
 *   · 副作用 = gState/RxState 被写回 READY。本工程对这两者的全部判据都在
 *     hdmarx->State 上（vUartDmaIdleHandler 卡死检测 / ErrorCallback 重装 / 重装本身）,
 *     与 RxState 无耦合; RX 循环 DMA 硬件上仍在跑, 语义不受影响。
 * ⚠️ 仅任务上下文 + 链路空闲时调（见 .h 契约）。换挡瞬间在途字节变乱码,
 *    由陀螺解析器的帧头重同步兜底 —— 这是开机一次性动作, 不值得为此做更多。 */
void vUartSetBaud(uart_id_t emId, uint32_t ulBaud)
{
    UART_HandleTypeDef *pstHuart;

    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return;
    }
    pstHuart = g_astUartCh[emId].pstHuart;

    pstHuart->Init.BaudRate = ulBaud;       /* 句柄里的镜像同步改, 后续重装/恢复链读到的是新值 */
    (void)HAL_UART_Init(pstHuart);
}

/* ==================== IDLE 中断处理 ==================== */
void vUartDmaIdleHandler(UART_HandleTypeDef *pstHuart)
{
    if (__HAL_UART_GET_FLAG(pstHuart, UART_FLAG_IDLE) == RESET)
    {
        return;
    }

    /* 清 IDLE 标志（读 SR 再读 DR） */
    __HAL_UART_CLEAR_IDLEFLAG(pstHuart);

    for (uint8_t ucIdx = 0; ucIdx < UART_ID_NUM; ucIdx++)
    {
        if (g_astUartCh[ucIdx].pstHuart == pstHuart)
        {
            /* ---- RX 卡死检测（2026-10 加, 六路串口的兜底自愈）----
             * 死链: USART 侧错误(FE/NE/ORE)在 DMA 接收态下被 HAL 判为阻塞性错误 →
             * HAL_UART_IRQHandler 把 ErrorCallback **推迟**到 HAL_DMA_Abort_IT 的完成回调
             * (hal_uart.c:2421-2446), 而那个回调只在 DMA **TC 中断**里执行
             * (hal_dma.c:851 的 ABORT 分支) —— 循环模式的流在环中间被停掉后 NDTR 永远
             * 到不了 0, TC 永远不来 → ErrorCallback 一行不跑、DMA State 永久卡 ABORT。
             * 之后连 vUartReArmRx 都救不回: HAL_UART_Receive_DMA → HAL_DMA_Start_IT 因
             * State != READY 返回 HAL_BUSY。现场表现 = 该路只发不收（"识别成功但一直超时"）。
             * 检测判据精确无假阳性: 正常收流时 hdmarx 恒为 BUSY; 只有踩进上面那条死链
             * 才会出现 ABORT。此时 IDLEIE 仍开着（错误路径不清它）, 噪声一停线 IDLE 就来,
             * 正好当我们的触发器。恢复动作与 ErrorCallback 的 RX 分支完全相同 ——
             * ⚠️ 安全性与那里同源: 错误路径已把 CR3.DMAR 清掉, AbortReceive 不会走进
             *    hal_dma.c 那个 5ms 忙等循环（本函数 prio 6, TIM6 prio 15, 见上面铁律）。 */
            if ((pstHuart->hdmarx != NULL) &&
                (pstHuart->hdmarx->State == HAL_DMA_STATE_ABORT))
            {
                (void)ucUartDmaUnstickRx(pstHuart);
                g_astUartCh[ucIdx].usRxUnstuck++;
                g_astUartCh[ucIdx].usRxErr++;
                g_astUartCh[ucIdx].usStart   = 0u;
                g_astUartCh[ucIdx].usLen     = 0u;
                g_astUartCh[ucIdx].usLastPos = 0u;
                (void)HAL_UART_AbortReceive(pstHuart);
                if (HAL_UART_Receive_DMA(pstHuart, g_astUartCh[ucIdx].aucBuf,
                                         UART_DMA_BUF_SIZE) != HAL_OK)
                {
                    g_astUartCh[ucIdx].usRxRearmFail++;
                    /* ⚠️ 重装失败（多半是某任务恰好持着 huart->Lock, HAL_BUSY）→ 把
                     * hdmarx->State 写回 ABORT: 否则 State 已被上面的解卡改成 READY,
                     * 本分支的进入条件从此永不成立, 这路 RX **永久聋**（非 RPI 路没有
                     * 5s 任务级兜底）。写回 ABORT 让下一个 IDLE（对端还在发数据就有）
                     * 再走一遍「解卡 + 重装」——整条链幂等, 撞锁窗口亚毫秒级, 下次
                     * 基本必成。（2026-10-09 修） */
                    pstHuart->hdmarx->State = HAL_DMA_STATE_ABORT;
                }
                break;
            }

            /* 圆环写指针位置（0~511）: DMA 循环模式下 NDTR 到 0 由硬件自动重装,
             * 所以「缓冲大小 - 剩余计数」= 本圈已写到的位置 = 累计字节数 mod 512。
             * 它是位置而不是帧长 —— 帧长要拿它和上次产出的位置相减（含回绕）。 */
            uint16_t usPos = (uint16_t)(UART_DMA_BUF_SIZE
                                        - __HAL_DMA_GET_COUNTER(pstHuart->hdmarx));

            g_astUartCh[ucIdx].usStart = g_astUartCh[ucIdx].usLastPos;
            g_astUartCh[ucIdx].usLen   = (usPos >= g_astUartCh[ucIdx].usLastPos)
                ? (uint16_t)(usPos - g_astUartCh[ucIdx].usLastPos)
                : (uint16_t)(UART_DMA_BUF_SIZE - g_astUartCh[ucIdx].usLastPos + usPos);
            g_astUartCh[ucIdx].usLastPos = usPos;

            if (g_astUartCh[ucIdx].usLen > 0)
            {
                BaseType_t xHigherPriorityTaskWoken = pdFALSE;
                g_astUartCh[ucIdx].ulRxBytes += g_astUartCh[ucIdx].usLen;   /* 遥测累加 */
                xSemaphoreGiveFromISR(g_astUartCh[ucIdx].xSem, &xHigherPriorityTaskWoken);
                portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            }
            break;
        }
    }
}

/* ==================== 串口错误恢复 ====================
 * ⚠️ 本函数跑在 **prio 6 的 ISR** 里（HAL_UART_IRQHandler 的阻塞性错误分支、
 *    以及 UART_DMAError → HAL_UART_ErrorCallback）。因此这里**绝对禁止**调
 *    HAL_UART_AbortTransmit / HAL_DMA_Abort / HAL_UART_AbortTransmit_IT:
 *      · 前两个内部是 `while((CR & DMA_SxCR_EN) != RESET){ if (HAL_GetTick()-t0 > 5) ... }`
 *        （hal_dma.c:544-560）, 而 HAL 时基 TIM6 是 prio 15 —— prio 6 的 ISR 里 uwTick
 *        永远不涨, 那个循环**永不超时 → 整个 MCU 挂死, 电机保持末速**;
 *      · 最后一个把 hdmatx->State 置成 ABORT 后要等**那个可能已经坏掉的流中断**来收尾,
 *        一旦不来 State 永久停在 ABORT, 此后 HAL_DMA_Abort 会因 State != BUSY 直接返回
 *        HAL_ERROR（hal_dma.c:520）—— 连任务侧的看门狗都救不回来。
 *     TX 的完整 HAL 恢复交给任务侧 vUartTxRecoverIfNeeded()。
 *
 * ---- RX 侧（⚠️ 这一段是必须的, 删了链路会静默死一半）----
 * F4 HAL 在 **DMA 接收态**下把任何 PE/FE/NE/ORE 都判为**阻塞性错误**(blocking_error):
 *   HAL_UART_IRQHandler → UART_EndRxTransfer() 把 RX 状态拉回 READY → 调 HAL_UART_ErrorCallback
 * 而 HAL 里那个回调是**弱定义**(默认空实现) —— 于是 UART 从那一刻起**永久变聋**:
 *   · TX 照常（对端看到本端一切正常, Pi 侧日志毫无异常）;
 *   · RX 再也不进一个字节 → 所有「等结果帧」的动作(色球/人质/靶标/巡线起流/握手)全部超时。
 * **实测踩过**: Pi 的 USB 串口适配器掉线时(其日志 `Input/output error` → `/dev/ttyUSB_GKS
 * 不存在`), 线上的噪声/断帧给了 MCU 一个噪声错误 → RPI 链路只发不收, 现场表现为
 * 「单片机一直报 BALL fail: scan, 而树莓派显示识别成功」, 唯一恢复手段是复位单片机。
 * 这里只做两件事: ① 清标志; ② 重新 arm 循环 DMA。
 * ⚠️ 必须同时把 usLastPos 归零: 重装后 DMA 写指针从 0 重新开始, 若仍按旧位置算
 *    「本次新增」长度, 会得出一段虚假的超长数据 → 被 usUartDmaTakeCopy 直接丢弃(它还
 *    会连带吃掉一个信号量), 帧就凭空少一截。 */
/* ---- RX DMA「ABORT 卡死」强制复位（2026-10, ISR/任务两侧都安全）----
 * 死链机理见 vUartDmaIdleHandler 的卡死检测段: HAL_DMA_Abort_IT 的完成回调只在
 * DMA TC 中断里执行, 循环流环中被停后 TC 永不来 → State 永久 ABORT、Lock 未释放,
 * 之后 HAL_UART_Receive_DMA 必然 HAL_BUSY。此时流早已停死（EN 已清、TC/TE 中断
 * 都不会再触发）, 就地把状态复位是安全的 —— 不会与任何在跑的 DMA 竞争。
 * 三个调用点: IDLE 卡死检测（ISR）、HAL_UART_ErrorCallback 的 RX 分支（ISR,
 * 防 TC 恰好 pending 时走到一半的边角）、vUartReArmRx（任务, 5s 兜底也要能成功）。 */
static uint8_t ucUartDmaUnstickRx(UART_HandleTypeDef *pstHuart)
{
    DMA_HandleTypeDef *pstDma = pstHuart->hdmarx;

    if ((pstDma != NULL) && (pstDma->State == HAL_DMA_STATE_ABORT))
    {
        pstDma->State = HAL_DMA_STATE_READY;
        __HAL_UNLOCK(pstDma);
        pstDma->XferAbortCallback = NULL;   /* 甩掉那个永远等不到时机的迟来回调 */
        return 1u;
    }
    return 0u;
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    uint8_t ucIdx;

    for (ucIdx = 0u; ucIdx < (uint8_t)UART_ID_NUM; ucIdx++)
    {
        if (g_astUartCh[ucIdx].pstHuart != huart)
        {
            continue;
        }

        (void)huart->ErrorCode;                 /* 诊断用（调试器里能看） */

        /* ---- TX 侧: 最小恢复（纯内存记账, 见本函数开头那段禁令） ----
         * TX DMA 出错也会走到这里（UART_DMAError → HAL_UART_ErrorCallback）: HAL 已把
         * gState 置回 READY, 但我们的 ucTxBusy 会**永远卡住** → 该路 TX 永久哑掉。
         * ⚠️ 2026-10 加 gState 判别: RX-only 错误（FE/NE/ORE, 走 hal_uart.c 阻塞分支）
         *    **完全不碰 TX** —— gState 仍是 BUSY_TX、TX DMA 流还在健康地跑。旧版无条件
         *    记账会把在途段判死: tail 前移 + 提前释放环空间（生产者可覆盖在途段 → 线上
         *    乱码尾）+ ucTxRecoverReq 让下一次发送把健康流拦腰截断（线上半截帧）。 */
        if ((g_astUartCh[ucIdx].ucTxBusy != 0u) &&
            (huart->gState != HAL_UART_STATE_BUSY_TX))
        {
            uint16_t usLost = g_astUartCh[ucIdx].usTxCurrentLen;
            g_astUartCh[ucIdx].usTxDrop      += usLost;
            g_astUartCh[ucIdx].usTxTail       = (uint16_t)((g_astUartCh[ucIdx].usTxTail + usLost)
                                                           & (UART_TX_BUF_SIZE - 1u));
            g_astUartCh[ucIdx].usTxPending   -= usLost;
            g_astUartCh[ucIdx].usTxCurrentLen = 0u;
            g_astUartCh[ucIdx].ucTxBusy       = 0u;
            g_astUartCh[ucIdx].usTxErr++;
            g_astUartCh[ucIdx].ucTxRecoverReq = 1u;
        }

        /* ---- RX 侧 ----
         * ⚠️ 判据是「RX 的 DMA 引擎还在不在跑」, **不是** RxState:
         *    所有进本回调的路径 HAL 都会把 RxState 置成 READY（UART_EndRxTransfer 里做的），
         *    所以拿 RxState 当门等于没有门。真正要防的是**这种情况**:
         *    TX DMA 出错时 UART_DMAError（hal_uart.c:3183-3189）会顺手调 UART_EndRxTransfer
         *    把 RxState 置 READY, 但**不动 RX 的 DMA 流、也不清 CR3.DMAR** —— 此时 RX 其实
         *    完全健康、IDLE 中断也还开着（ReceptionType 是 STANDARD, IDLEIE 不会被清）。
         *    若照旧重装, 就会把一个正在跑的循环 RX 拆掉、把 usLastPos 归零, **静默丢掉在途
         *    那一帧**; 而且会新引入一条 prio 6 的 ISR 里 `HAL_DMA_Abort(忙流)` 的路径
         *    （见铁律 7 —— CR3.DMAR 还置着时 AbortReceive 会走进那个 5ms tick 循环）。
         * 判据: hdmarx->State == BUSY ⟺ 循环 DMA 还在搬 ⟺ 本次错误与 RX 无关 → 一个字节别碰。
         *   · TX DMA 出错 → hdmarx 未被碰过 → BUSY → 跳过 ✓
         *   · RX DMA 出错 → HAL_DMA_IRQHandler 的 TE 分支已把 State 置 READY → 重装 ✓
         *   · FE/NE/ORE → AbortReceive 已被 HAL 走过, State 是 READY/ABORT → 重装 ✓ */
        if ((huart->hdmarx != NULL) &&
            (huart->hdmarx->State == HAL_DMA_STATE_BUSY))
        {
            break;
        }

        g_astUartCh[ucIdx].usRxErr++;

        /* ABORT 卡死先解掉（见 ucUartDmaUnstickRx）: 本回调经「TC 恰好 pending」这条
         * 边角路径进来时 State 可能正是 ABORT —— 不解的话下面的 Receive_DMA 会
         * HAL_BUSY, 重装等于没做。 */
        if (ucUartDmaUnstickRx(huart) != 0u)
        {
            g_astUartCh[ucIdx].usRxUnstuck++;
        }

        __HAL_UART_CLEAR_PEFLAG(huart);         /* 读 SR 再读 DR: 一并清 PE/FE/NE/ORE */

        g_astUartCh[ucIdx].usStart   = 0u;
        g_astUartCh[ucIdx].usLen     = 0u;
        g_astUartCh[ucIdx].usLastPos = 0u;

        /* 先把状态机拉回 READY（HAL_UART_AbortReceive 幂等, 出错也无妨）, 再重装。
         * 只调 Receive_DMA 也够 —— 但若 RxState 不是 READY, 它会**静默返回 HAL_BUSY**
         * 而什么都不做, 链路就再也起不来了; Abort 先把门打开, 顺手失败还记一笔数。
         * （现场不会挂死: 走到这里时 HAL 已经把 CR3.DMAR 清掉或流已停, AbortTransmit
         *   里那个 5ms 循环第一轮就退出 —— 见 vUartTxRecoverIfNeeded 的说明。） */
        (void)HAL_UART_AbortReceive(huart);
        if (HAL_UART_Receive_DMA(huart, g_astUartCh[ucIdx].aucBuf,
                                 UART_DMA_BUF_SIZE) != HAL_OK)
        {
            g_astUartCh[ucIdx].usRxRearmFail++;
        }
        break;
    }
}

/* ==================== 发送段完成（ISR） ====================
 * 只在「DMA 流 TC 中断 → UART_DMATransmitCplt 置 CR1.TCIE → USART TC 中断 →
 * UART_EndTransmit_IT」这条**完整链路**走通时才会被调到（NORMAL 模式的
 * UART_DMATransmitCplt 只置 TCIE、不调用户回调 —— hal_uart.c:3015-3030）,
 * 所以 usTxDmaSegs 是「TX DMA 真的在工作」的唯一直接证据。
 * ⚠️ 进来时 gState 已在 UART_EndTransmit_IT 里置回 READY（hal_uart.c:3609）, 且
 *    hdmatx 的 Lock/State 都已释放（hal_dma.c:908-911）, 所以下面直接续发是合法的
 *    —— 这是 HAL 官方支持的链式 DMA 用法。
 * 本函数在 ISR 里, 只做内存记账 + 起发（vUartTxKick 的 HAL 调用无 tick 循环）。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    uint8_t ucIdx;

    for (ucIdx = 0u; ucIdx < (uint8_t)UART_ID_NUM; ucIdx++)
    {
        if (g_astUartCh[ucIdx].pstHuart != huart)
        {
            continue;
        }

        g_astUartCh[ucIdx].ulTxBytes     += g_astUartCh[ucIdx].usTxCurrentLen;
        g_astUartCh[ucIdx].usTxTail       = (uint16_t)((g_astUartCh[ucIdx].usTxTail
                                                        + g_astUartCh[ucIdx].usTxCurrentLen)
                                                       & (UART_TX_BUF_SIZE - 1u));
        g_astUartCh[ucIdx].usTxPending   -= g_astUartCh[ucIdx].usTxCurrentLen;
        g_astUartCh[ucIdx].usTxCurrentLen = 0u;
        g_astUartCh[ucIdx].usTxDmaSegs++;
        g_astUartCh[ucIdx].ucTxBusy       = 0u;      /* ⚠️ 必须在 kick 之前: 它是 kick 的 test-and-set 判据 */
        break;
    }

    if (ucIdx < (uint8_t)UART_ID_NUM)
    {
        vUartTxKick(&g_astUartCh[ucIdx], 1u);
    }
}

/* ==================== 接收接口 ==================== */

/* 重装循环 DMA（幂等）: 清残状态 + 把 ISR 的位置游标归零, 再 arm。
 * ⚠️ usLastPos 必须归零 —— 重装后写指针回到 0, 不归零会让下一次 IDLE 算出一段
 *    虚假的超长「新增数据」（见 HAL_UART_ErrorCallback 的说明）。 */
void vUartReArmRx(uart_id_t emId)
{
    stUartDmaChTdf *pstCh;

    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return;
    }
    pstCh = &g_astUartCh[emId];

    taskENTER_CRITICAL();
    pstCh->usStart   = 0u;
    pstCh->usLen     = 0u;
    pstCh->usLastPos = 0u;
    taskEXIT_CRITICAL();

    /* ABORT 卡死先解掉（见 ucUartDmaUnstickRx）—— 否则 5s 兜底也会因 HAL_BUSY
     * 重装失败（死链里 State 卡 ABORT, HAL_DMA_Start_IT 直接拒发）。 */
    if (ucUartDmaUnstickRx(pstCh->pstHuart) != 0u)
    {
        pstCh->usRxUnstuck++;
    }

    /* 同 HAL_UART_ErrorCallback: 先 Abort 把 RxState 拉回 READY, 再重装;
     * 否则 RxState 若停在 BUSY, Receive_DMA 会静默返回 HAL_BUSY, 等于没恢复。 */
    (void)HAL_UART_AbortReceive(pstCh->pstHuart);
    if (HAL_UART_Receive_DMA(pstCh->pstHuart, pstCh->aucBuf,
                             UART_DMA_BUF_SIZE) != HAL_OK)
    {
        pstCh->usRxRearmFail++;
    }
}

uint32_t ulUartGetRxBytes(uart_id_t emId)
{
    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return 0u;
    }
    return g_astUartCh[emId].ulRxBytes;
}

uint16_t usUartGetRxErr(uart_id_t emId)
{
    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return 0u;
    }
    return g_astUartCh[emId].usRxErr;
}

uint16_t usUartGetRxRearmFail(uart_id_t emId)
{
    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return 0u;
    }
    return g_astUartCh[emId].usRxRearmFail;
}

/*
 * 取帧的唯一入口（见 app_uart.h 的契约说明）:
 *   阻塞等信号量 → 锁存本次的 (起始偏移, 长度) → 整段拷进 pucDst。
 *   拷贝分两段: 数据跨圆环末尾时, 先拷 [usStart, 512) 再拷 [0, 剩余) ——
 *   消费者因此永远拿到连续字节, 不必自己处理回绕。
 * 本次新增超过 usDstCap 视为调用方缓冲不够: 返回 0 丢弃（消费者缓冲应 ≥ 最大帧长）。
 */
uint16_t usUartDmaTakeCopy(uart_id_t emId, uint8_t *pucDst, uint16_t usDstCap, TickType_t xTimeout)
{
    stUartDmaChTdf *pstCh;
    uint16_t usStart;
    uint16_t usLen;
    uint16_t usFirst;

    if (((uint8_t)emId >= (uint8_t)UART_ID_NUM) || (pucDst == NULL) || (usDstCap == 0u))
    {
        return 0;
    }
    pstCh = &g_astUartCh[emId];

    if (xSemaphoreTake(pstCh->xSem, xTimeout) != pdTRUE)
    {
        return 0;                                   /* 超时: 无新数据 */
    }

    /* 先锁存再拷贝: 拷贝期间 ISR 若为更新的帧改写了这两个字段, 本次也不受影响。
     * ⚠️ 双读校验（2026-10）: 两次 16 位读之间可被 IDLE ISR 打断, 拿到「新 start +
     *    旧 len」的错位组合 → 拷出一段错位数据（靠对端 CRC/行界定丢弃, 表现为偶发
     *    坏帧）。重读直到两字段都稳定 —— ISR 写完即走, 冲突后重读几乎必然成功。 */
    do
    {
        usStart = pstCh->usStart;
        usLen   = pstCh->usLen;
    }
    while ((usStart != pstCh->usStart) || (usLen != pstCh->usLen));

    if ((usLen == 0u) || (usLen > usDstCap))
    {
        return 0;
    }

    usFirst = (uint16_t)(UART_DMA_BUF_SIZE - usStart);      /* 到圆环末尾还有多少字节 */
    if (usFirst >= usLen)
    {
        memcpy(pucDst, &pstCh->aucBuf[usStart], usLen);
    }
    else
    {
        memcpy(pucDst, &pstCh->aucBuf[usStart], usFirst);                   /* 圆环尾段 */
        memcpy(&pucDst[usFirst], &pstCh->aucBuf[0],
               (uint16_t)(usLen - usFirst));                                /* 圆环首段 */
    }
    return usLen;
}

/* ==================== 发送接口 ==================== */
void vUartSendArray(uart_id_t emId, const uint8_t *pucData, uint32_t ulLen)
{
    stUartDmaChTdf *pstCh;
    uint32_t        ulDeadline;

    if ((pucData == NULL) || (ulLen == 0u) || ((uint8_t)emId >= (uint8_t)UART_ID_NUM))
    {
        return;
    }
    pstCh = &g_astUartCh[emId];

    /* ⚠️ 对账计数进临界区: `+=` 是读-改-写（理由同下面 usTxDrop 那条 —— 对账不变量的
     * 另一半写者在 prio 6 ISR 里; 将来同口出现第二个直调发送任务也会与它交错）。
     * 目前六个口的并发发送都被上层串行化（rpi/emm 走 mutex、其余单任务）, 这里是把
     * 前提收进函数本体。区内只有一条加法, 符合临界区纪律。 */
    taskENTER_CRITICAL();
    pstCh->ulTxReqBytes += ulLen;
    taskEXIT_CRITICAL();

    /* 单次超过环容量 → 直接丢。**绝不进等待循环** —— 环里永远腾不出这么多空间,
     * 只会白等一个 UART_TX_ENQ_TIMEOUT_MS 再丢。本工程单次最大是 bt_sendf 的 256B。 */
    if (ulLen > (uint32_t)(UART_TX_BUF_SIZE - 1u))
    {
        /* ⚠️ 计数进临界区: usTxDrop 的另一半写者（TX DMA 错误退账 / 看门狗退账）在
         * **prio 6 的 ISR** 里, 而 `+=` 是读-改-写 —— 不管它, 一次自增会被 ISR 吞掉,
         * CLAUDE.md 那条对账不变量 `ulTxReqBytes == ulTxBytes + usTxDrop` 会静默不成立
         * （现场照它判断"记账漏了"就会被骗）。区内只有一条加法。 */
        taskENTER_CRITICAL();
        pstCh->usTxDrop += (uint16_t)ulLen;
        taskEXIT_CRITICAL();
        return;
    }

    /* 没起 TX DMA 的通道（HAL_DMA_Init 失败）→ 原样走阻塞发送, 行为与改造前一致。
     * ⚠️ 这道分流不能省: hdmatx == NULL 时 HAL_UART_Transmit_DMA **不是**返回 HAL_BUSY,
     *    而是在 hal_uart.c:1399 无条件解引用 huart->hdmatx → **HardFault**。 */
    if (pstCh->ucTxUseDma == 0u)
    {
        HAL_UART_Transmit(pstCh->pstHuart, (uint8_t *)pucData, (uint16_t)ulLen, HAL_MAX_DELAY);
        pstCh->ulTxBytes += ulLen;
        return;
    }

    vUartTxRecoverIfNeeded(pstCh);                  /* 动环之前先把故障状态清掉 */

    ulDeadline = HAL_GetTick() + UART_TX_ENQ_TIMEOUT_MS;
    for (;;)
    {
        uint16_t usFree;
        uint16_t usHead;
        uint16_t usFirst;

        taskENTER_CRITICAL();
        usFree = (uint16_t)((UART_TX_BUF_SIZE - 1u) - pstCh->usTxPending);
        if (usFree >= (uint16_t)ulLen)
        {
            usHead  = pstCh->usTxHead;
            /* ⚠️ 临界区里**只有 memcpy + 指针运算**。绝不在这里调任何 HAL 串口函数:
             *    临界区屏蔽 TIM6/TIM7, 而 HAL_MAX_DELAY 的超时靠 HAL_GetTick 停走 → 真死锁。
             *    512B 的 memcpy 约 1~2µs, 对 1kHz 的舵机节拍是 0.2% 抖动, 可忽略。 */
            usFirst = (uint16_t)(UART_TX_BUF_SIZE - usHead);
            if (usFirst > (uint16_t)ulLen)
            {
                usFirst = (uint16_t)ulLen;
            }
            memcpy(&pstCh->aucTxBuf[usHead], pucData, usFirst);
            if (usFirst < (uint16_t)ulLen)
            {
                memcpy(&pstCh->aucTxBuf[0], &pucData[usFirst],
                       (uint16_t)((uint16_t)ulLen - usFirst));
            }
            pstCh->usTxHead    = (uint16_t)((usHead + (uint16_t)ulLen) & (UART_TX_BUF_SIZE - 1u));
            pstCh->usTxPending = (uint16_t)(pstCh->usTxPending + (uint16_t)ulLen);
            taskEXIT_CRITICAL();

            vUartTxKick(pstCh, 0u);                 /* 临界区外: 里面会调 HAL_UART_Transmit_DMA */
            return;
        }
        taskEXIT_CRITICAL();

        if ((int32_t)(HAL_GetTick() - ulDeadline) >= 0)
        {
            /* 环满超时: 丢弃并计数（临界区理由同上面那条） */
            taskENTER_CRITICAL();
            pstCh->usTxDrop += (uint16_t)ulLen;
            taskEXIT_CRITICAL();
            return;
        }

        /* ⚠️ 调度器未启动时不能 vTaskDelay（pxCurrentTCB 是 NULL → 上电 HardFault）。
         *    vAppInit 阶段目前没有发送, 这两行是防将来有人在调度器启动前发。 */
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        {
            vTaskDelay(1);                          /* 背压: 让 DMA 把环排空一点 */
        }
        else
        {
            /* 调度器没跑: 不能背压, 直接丢（临界区理由同上面那条） */
            taskENTER_CRITICAL();
            pstCh->usTxDrop += (uint16_t)ulLen;
            taskEXIT_CRITICAL();
            return;
        }
    }
}

uint8_t ucUartWaitTxDone(uart_id_t emId, TickType_t xTimeout)
{
    stUartDmaChTdf *pstCh;
    TickType_t      xDeadline;

    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return 1u;
    }
    pstCh = &g_astUartCh[emId];

    if (pstCh->ucTxUseDma == 0u)
    {
        return 1u;                                  /* 走的是阻塞发送, 返回即已发完 */
    }

    xDeadline = xTaskGetTickCount() + xTimeout;
    while ((pstCh->ucTxBusy != 0u) || (pstCh->usTxPending != 0u))
    {
        if ((int32_t)(xTaskGetTickCount() - xDeadline) >= 0)
        {
            return 0u;
        }
        vTaskDelay(1);
    }
    return 1u;
}

void vUartPrintf(uart_id_t emId, const char *pcFormat, ...)
{
    char acBuf[256];
    va_list stArg;
    int lLen;

    va_start(stArg, pcFormat);
    lLen = vsnprintf(acBuf, sizeof(acBuf), pcFormat, stArg);
    va_end(stArg);

    /* ⚠️ vsnprintf 返回的是「本应写入的长度」而非实际写入量: 内容被截断时它仍返回
     * 原长, 直接拿去发会从 acBuf 越界读（栈上读到垃圾, 甚至硬件异常）。
     * 必须夹到缓冲实际容量以内。 */
    if (lLen >= (int)sizeof(acBuf))
    {
        lLen = (int)sizeof(acBuf) - 1;
    }
    if (lLen > 0)
    {
        /* 走唯一原语（内部会拷进 TX 环）—— 别再直接调 HAL_UART_Transmit:
         * 那是第二条绕过环的 TX 路径, 会因为 gState==BUSY_TX 静默丢帧。 */
        vUartSendArray(emId, (const uint8_t *)acBuf, (uint32_t)lLen);
    }
}

/* ==================== 发送遥测 ==================== */
uint32_t ulUartGetTxReqBytes(uart_id_t emId)
{
    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return 0u;
    }
    return g_astUartCh[emId].ulTxReqBytes;
}

uint32_t ulUartGetTxBytes(uart_id_t emId)
{
    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return 0u;
    }
    return g_astUartCh[emId].ulTxBytes;
}

uint16_t usUartGetTxDmaSegs(uart_id_t emId)
{
    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return 0u;
    }
    return g_astUartCh[emId].usTxDmaSegs;
}

uint16_t usUartGetTxStall(uart_id_t emId)
{
    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return 0u;
    }
    return g_astUartCh[emId].usTxStall;
}

uint16_t usUartGetTxDrop(uart_id_t emId)
{
    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return 0u;
    }
    return g_astUartCh[emId].usTxDrop;
}

uint16_t usUartGetTxErr(uart_id_t emId)
{
    if ((uint8_t)emId >= (uint8_t)UART_ID_NUM)
    {
        return 0u;
    }
    return g_astUartCh[emId].usTxErr;
}
