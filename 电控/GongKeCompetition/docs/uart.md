# 串口（6 路 UART：DMA 环形收发）

> 由根 `CLAUDE.md` 拆分；章节标题保持原样（源码注释按节名引用）。
> 相关：根 `CLAUDE.md`「中断与并发铁律」、`docs/pi-link.md`

  - ⚠️⚠️ **RX 会被 HAL 静默停掉、而 TX 照常（2026-09 修，别把修复删了）**：F4 HAL 在 **DMA 接收态**下把任何 PE/FE/NE/ORE 都判为**阻塞性错误** → `UART_EndRxTransfer()` 停掉 RX DMA → 调 `HAL_UART_ErrorCallback`（HAL 里是**弱定义**，工程原先没有实现 → 什么都不做）。于是该 UART **永久变聋**：TX 一切正常、对端日志毫无异常，本端所有「等结果帧」的动作（色球/人质/靶标/巡线起流/握手）全部超时。**实测踩过**：Pi 的 USB 串口适配器掉线（其日志 `Input/output error` → `/dev/ttyUSB_GKS 不存在`）→ 线上噪声给 MCU 一个噪声错误 → 表现是「Pi 显示色球识别成功、单片机一直 `BALL fail: scan`」，只有复位能恢复。⚠️⚠️ **真正兜住这条链的是「IDLE 卡死检测」，不是 `HAL_UART_ErrorCallback`**（2026-10 审查核实，根 `CLAUDE.md` 旧版把两者写反了）：
`HAL_UART_IRQHandler` 把 `ErrorCallback` **推迟**到 `HAL_DMA_Abort_IT` 的完成回调（`hal_uart.c:2421-2446`），而 `HAL_DMA_Abort_IT` 对 BUSY 的流是「置 `State=ABORT` + 清 EN + **返回 HAL_OK**」（`hal_dma.c:580-597`）⇒ 那个 `!= HAL_OK` 分支不成立、回调**不会被直接调用**；它只可能从 **DMA TC 中断**里来，而**六路 RX 流的中断从未在 NVIC 使能**（`dma.c` 只开时钟、`stm32f4xx_it.c` 只手写了 6 个 **TX** 流）。所以 **RX 首次错误时 `HAL_UART_ErrorCallback` 一行都不会跑**（`app_uart.c:410-422` 的注释把这条链写得很清楚）。
现在实际有三道：
① **`vUartDmaIdleHandler` 的卡死检测**（`app_uart.c:423-439`，**六路都有**）: 错误路径把 `hdmarx` 停在 `State==ABORT`，而它**不动 IDLEIE** ⇒ 噪声一停线 IDLE 就来，正好当触发器 → 解卡 + 重装 + `usLastPos` 归零（不归零会算出一段虚假超长「新增数据」被 take-copy 丢掉）；
② `HAL_UART_ErrorCallback` 的 RX 分支（**只覆盖 TX 错误与 `State==ABORT` 的边角**，那道「`hdmarx->State == BUSY` 就别碰 RX」的门必须留着）；
③ `vRpiTask` 在链路已判掉线时每 5s 兜底重装一次（**只有 RPI 这一路**有，健康链路永不触发）。
⚠️ 所以：**别把 ① 当"冗余"删掉** —— 删了这条故障会原样复活（文档旧版正是把 ① 漏掉了）。
遥测 `rpi_rx_bytes` / `rpi_rx_err`（蓝牙 Tab「通信」组）就是为这个加的：**字节数停涨而 TX 正常 = 本端已经聋了**（⚠️ 首次 RX 错误不会进 `rpi_rx_err` —— 它只在 ② 里自增，而 ② 在这条路上不跑；判据看 `rpi_rx_bytes`）。
    - ⚠️ Pi 端 2026-09 也补了对应的一半（`GKS_Competition/serial_comm.py`）：那次掉线时 Pi 的 `handler.run()` 把链路故障当「收到中断信号」**直接退出了进程**，而调试模式（GKS_Web 控制台 Popen 拉起）没有任何东西会把它重新拉起来 —— 现场是「Pi 掉一次串口，车从此不响应，只能人工重启」。现在链路故障（`OSError` / `SerialDisconnected`）改为 `restart_process()` **原地 `os.execl` 重启**（连续重启延时 3s→60s 递增，握手成功即清零），只有真被叫停（Ctrl+C/SIGTERM/菜单）才退出。改 Pi 那个仓库时别把这条改回去。
  - **TX 侧 2026-09 也改成 DMA 了（与 RX 对齐）**：唯一入口仍是 `vUartSendArray(id, data, len)`，但它现在把字节**拷进该路的 TX 环形缓冲**（`UART_TX_BUF_SIZE` = 512/路）就返回，DMA 后台搬运。**返回 ≠ 已在线上**。已经逐个核对过 4 个发送出口（`vRpiSendFrame` / `bt_sendf` / `vEmmV5SendCmd` / `atk_ms901m_*_reg_by_id`），**没有调用方依赖「返回时已在线上」** —— 对端不可能在收全请求之前就先回帧，所以「发完再等回应」的调用方行为不变。
    - **环满 → 阻塞背压**（`vTaskDelay(1)` 轮询，上限 `UART_TX_ENQ_TIMEOUT_MS` 500ms，超时才丢并计入 `usTxDrop`）。单次超过 511B 直接丢（不进等待循环）。`$B,LIST` 那 ~9.8KB 突发就是靠背压消化的。
    - ⚠️ **只能在任务上下文调**，且**绝不能在 `taskENTER_CRITICAL()` 里调**（见铁律 5/6）。真要「等数据真的发完」用 `ucUartWaitTxDone()`（目前无调用点）。
    - **TX DMA 流全部配在 `Application/app_uart.c` 里手动做**（`g_astUartTxMap` + `HAL_DMA_Init` + `__HAL_LINKDMA` + NVIC），**不在 CubeMX 生成的 `usart.c` / `.ioc` 里** —— 这样 CubeMX 重生成冲不掉，`.ioc` 也不用动。改映射表要同步改 `stm32f4xx_it.c` `USER CODE BEGIN 1` 段里那 6 个 `DMAx_Streamy_IRQHandler`（它们只是 `vUartTxDmaIrqHandler(UART_ID_*)` 的一行分发器）。

      | id | 外设 | **TX 流** | 通道 | 现有 RX 流 |
      |---|---|---|---|---|
      | 0 RPI | USART1 | `DMA2_Stream7` | CH4 | DMA2_S2 |
      | 1 **副陀螺口**（原 OPENMV） | USART2 | `DMA1_Stream6` | CH4 | DMA1_S5 |
      | 2 EMM | USART3 | `DMA1_Stream3` | CH4 | DMA1_S1 |
      | 3 EXT | UART4 | `DMA1_Stream4` | CH4 | DMA1_S2 |
      | 4 BT | UART5 | `DMA1_Stream7` | CH4 | DMA1_S0 |
      | 5 **主陀螺口**（丝印 HWT101） | USART6 | `DMA2_Stream6` | CH5 | DMA2_S1 |

      （DMA1 的 4 个空闲流至此**全部用满**；DMA2 还剩 S0/S3/S4/S5。ADC 与其它外设都不用 DMA。）
    - ⚠️ **DMA 流 IRQ 是必需的**，不是可选优化：NORMAL 模式下 `UART_DMATransmitCplt`（`hal_uart.c:3015-3030`）只置 `CR1.TCIE`、**不调用户回调**，得等 USART TC 中断才轮得到 `HAL_UART_TxCpltCallback` 续发下一段。少了流中断，TX 只能发出第一段。
    - **遥测**：`rpi_tx_bytes`（蓝牙 Tab「通信」组）与 `rpi_rx_bytes` 对称。更细的几个只在源码里（未引用会被链接器裁掉，但计数器本身活在 `g_astUartCh[]` 里，调试器能看）：`ulTxReqBytes` / `ulTxBytes` / `usTxDmaSegs` / `usTxStall` / `usTxDrop` / `usTxErr`。**判读**：`ulTxBytes` 涨 **且** `usTxDmaSegs` 涨 = 真在走 TX DMA；**`ulTxBytes` 涨但 `usTxDmaSegs` 不动 = 在走阻塞兜底**（TX DMA 没起来）；`usTxStall > 0` = 看门狗触发过；`usTxDrop > 0` = 有字节被丢。对账不变量 `ulTxReqBytes == ulTxBytes + usTxDrop`（不成立 = 记账漏了）。
    - **三道故障兜底**（都在 `app_uart.c`）：① TX DMA 出错 → `HAL_UART_ErrorCallback` 里纯记账，完整 HAL 恢复交任务侧；② 在途段卡死（TC 中断丢失，判据是 `ucTxBusy` 置着超过 `UART_TX_STALL_TIMEOUT_MS` 300ms）→ 任务侧看门狗 `HAL_UART_AbortTransmit` + 丢在途段 + 重发；③ `HAL_UART_Transmit_DMA` 起发失败且**已确认无段在途** → 退回阻塞发送。
    - ⚠️⚠️ **`HAL_UART_ErrorCallback` 里的 RX 重装必须有一道「RX 引擎还在跑就别碰」的门**（判据 `huart->hdmarx->State == HAL_DMA_STATE_BUSY`）。**别用 `RxState` 当判据** —— 所有进这个回调的路径 HAL 都把 `RxState` 置成 READY 了（`UART_EndRxTransfer` 干的），拿它当门等于没有门。要防的是这一种：**TX DMA 出错时 `UART_DMAError`（`hal_uart.c:3183-3189`）会顺手把 `RxState` 置 READY，却不动 RX 的 DMA 流、也不清 `CR3.DMAR`** —— 那时 RX 其实完全健康（连 IDLE 中断都还开着，因为 `ReceptionType` 是 STANDARD、`IDLEIE` 不会被清）。照旧重装就会把一个正在跑的循环 RX 拆掉、`usLastPos` 归零 → **静默丢掉在途那一帧**；而且 `CR3.DMAR` 还置着时 `HAL_UART_AbortReceive` 会走进 `HAL_DMA_Abort(忙流)` —— 正是铁律 7 说的那条 prio 6 ISR 里不该有的路径。
      ⚠️ 某个通道的 TX DMA 起不来时（`ucTxUseDma == 0`）该路整条退回原来的阻塞发送，行为与改造前完全一致 —— 所以「TX DMA 没生效」是**良性降级**，不是故障。
