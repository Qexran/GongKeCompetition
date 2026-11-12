/**
  * @file       app_sensors.c
  * @brief      传感器模块：姿态传感器（两口 × 三设备, 按 app_config.h 的选型分派）。
  *
  *             **两个物理口各跑一个任务, 谁在哪个口、是哪颗设备, 全由选型决定**：
  *               主口 USART6 (PC6/PC7, 丝印 HWT101_TX/RX) → vSensorTask  , GYRO_MAIN_DEV
  *               副口 USART2 (PD5/PD6, 丝印 VICE_GYRO)    → vSensorTaskM0, GYRO_SUB_DEV
  *             两口的设备类型可以互换、也可以**选同型号** —— 三套解析器的状态都在
  *             「每口一份」的上下文里（stGyroPortCtxTdf），不是函数级 static。
  *
  *             **两个任务都在跑、都在解析**（与选没选中无关）, 但只有"有效源"那一路
  *             调 vSetAttitude/vSetYawFresh。有效源 = Beam `gyro_src_eff`, 由热备看门狗
  *             （vGyroSrcWatchdogTick）维护; `gyro_src` 是操作员设的**首选**。
  *             关掉自动切换时有效源恒等于首选, 行为与加这个功能之前一致。
  *
  *             M0 / HWT101 的回传频率与波特率也是**按口**配的（app_config.h 选型段
  *             GYRO_x_RATE_HZ / GYRO_x_BAUD / GYRO_x_BAUD_APPLY）: 开机在会合波特率
  *             115200 上由固件自动下发（vGyroDevInit）, 要改波特率时 MCU 本口跟着换挡
  *             —— 不碰 CubeMX、不碰上位机, 模块不写 Flash、填错重新上电即自愈。
  *
  *             ⚠️ 两个都发是错的：vSetAttitude 是一个临界区里的三字快照, 两源混发会让
  *                roll/pitch 与 yaw 来自不同模块。
  */

#include "app_sensors.h"
#include "app_beam.h"
#include "app_uart.h"        /* usUartDmaTakeCopy / UART_DMA_BUF_SIZE */
#include "app_rpi.h"         /* vRpiSendDebug（换源时那条联调串）*/
#include "gyro_m0_device.h"
#include "atk_ms901m_device.h"   /* 无条件包含: ATK 现在可能挂在任一口上 */
#include "FreeRTOS.h"
#include "task.h"

/* ⚠️ 设备码越界（控制台里手敲了 9 之类）→ 编译期硬失败。控制台的 @(0~2) 只是黄框提示、
 *    不拦保存, 所以这道门必须在固件侧。
 * ⚠️ 消息**故意用 ASCII**：AC5 会把 #error 里的中文打成 `???`（实测），那就等于没提示。 */
#if ((GYRO_MAIN_DEV) > 2) || ((GYRO_SUB_DEV) > 2)
#error "GYRO_MAIN_DEV / GYRO_SUB_DEV must be 0(ATK-901) / 1(M0) / 2(HWT101)"
#endif

/* ---- 波特率×设备的编译期交叉校验（控制台的 @() 与下拉只是提示, 这道门必须在固件侧）----
 * 档位表出处: M0 = Drivers/QEPack/gyro_m0_device.h（只到 115200, 兄弟表里的 230400 是笔误）;
 * HWT101 = 官方例程 wit_c_sdk 的 REG.h / WitSetUartBaud（CheckRange 上限 = 230400, 档码 7）。
 * ATK 没有波特率旋钮, 但值本身仍必须是表内值 —— 免得日后把某口换成 M0/HWT101 时埋雷。 */
#if (GYRO_MAIN_BAUD != 4800u) && (GYRO_MAIN_BAUD != 9600u) && (GYRO_MAIN_BAUD != 19200u) && \
    (GYRO_MAIN_BAUD != 38400u) && (GYRO_MAIN_BAUD != 57600u) && (GYRO_MAIN_BAUD != 115200u) && \
    (GYRO_MAIN_BAUD != 230400u)
#error "GYRO_MAIN_BAUD must be one of 4800 / 9600 / 19200 / 38400 / 57600 / 115200 / 230400"
#endif
#if ((GYRO_MAIN_DEV) == 1) && ((GYRO_MAIN_BAUD) > 115200u)
#error "main port = M0: GYRO_MAIN_BAUD max is 115200 (M0 baud table has no 230400)"
#endif
#if (GYRO_SUB_BAUD != 4800u) && (GYRO_SUB_BAUD != 9600u) && (GYRO_SUB_BAUD != 19200u) && \
    (GYRO_SUB_BAUD != 38400u) && (GYRO_SUB_BAUD != 57600u) && (GYRO_SUB_BAUD != 115200u) && \
    (GYRO_SUB_BAUD != 230400u)
#error "GYRO_SUB_BAUD must be one of 4800 / 9600 / 19200 / 38400 / 57600 / 115200 / 230400"
#endif
#if ((GYRO_SUB_DEV) == 1) && ((GYRO_SUB_BAUD) > 115200u)
#error "sub port = M0: GYRO_SUB_BAUD max is 115200 (M0 baud table has no 230400)"
#endif

/* HWT101 帧格式（维特智能 Z 轴陀螺, 与 ATK/M0 两套协议无关）：
 * 0x55 帧头 + 0x53/0x52 + 11 字节, 末字节是前 10 字节的累加和。只有 0x53 出角度。 */
#define GYRO_HWT_HEAD        0x55u
#define GYRO_HWT_TYPE_ANGLE  0x53u
#define GYRO_HWT_TYPE_ANGLE2 0x52u
#define GYRO_HWT_FRAME_LEN   11u

/* HWT101 配置命令（维特标准寄存器协议, 与上面帧协议同一本手册; 已逐字节对照官方例程
 * wit_c_sdk 的 REG.h / wit_c_sdk.c）: FF AA + 寄存器 + 16 位数据**低字节在前**, 模块无应答。
 * ⚠️ 与 Drivers 里 gyro_m0 的「0x55 0xAA reg dataL dataH」是**两套完全不同**的命令协议, 别混。 */
#define GYRO_HWT_REG_SAVE    0x00u   /* 写 0x0000=保存。**配置路径刻意不用**（只改 RAM, 断电自愈）;
                                          唯一例外 = 陀螺校准落零偏（ucGyroCalActiveSource ——
                                          校准的意义就在跨上电, 不保存等于白校） */
#define GYRO_HWT_REG_CALSW   0x01u   /* 校准动作寄存器 */
#define GYRO_HWT_CAL_OFF     0x00u   /* CALSW = 关闭陀螺自动校准 / 退出校准（老固件 = NORMAL） */
#define GYRO_HWT_CAL_GYRON   0x01u   /* CALSW = 打开陀螺自动校准（官方 HWT101 手册口径; 老固件的
                                          CALGYROACC 同值 = 一键陀螺校准 —— 两种语义下「开→静止
                                          →关」都是安全的同一套动作, 见 ucGyroCalActiveSource） */
#define GYRO_HWT_CAL_ZZERO   0x04u   /* CALSW = Z轴角度置零（官方表 CALANGLEZ; 上电时车须静止） */
#define GYRO_HWT_REG_RRATE   0x03u   /* 回传频率档码（0x09=100Hz, 0x0B=200Hz; 0x0A=125Hz 是 WT931 专属） */
#define GYRO_HWT_REG_BAUD    0x04u   /* 波特率档码（6=115200, 7=230400; 官方例程 CheckRange 上限即 230400） */
#define GYRO_HWT_REG_KEY     0x69u   /* 解锁寄存器 */
#define GYRO_HWT_KEY_UNLOCK  0xB588u /* KEY 写入值 = 解锁 */

/* 每条配置命令之间的间隔。官方 SDK 正常协议只等 1ms, 这里取 M0 驱动同款 100ms ——
 * 开机一次性动作, 裕量比省下的 0.2s 值钱, 且两型号节奏一致好排障。 */
#define GYRO_HWT_CFG_GAP_MS  100u

/* 写一个寄存器（任务上下文; 入 TX 环即返回, 模块不回帧）。 */
static void vGyroHwtWriteReg(uart_id_t emUart, uint8_t ucReg, uint16_t usData)
{
    uint8_t aucCmd[5];

    aucCmd[0] = 0xFFu;
    aucCmd[1] = 0xAAu;
    aucCmd[2] = ucReg;
    aucCmd[3] = (uint8_t)(usData & 0xFFu);      /* 低字节在前 —— 官方 SDK 同序 */
    aucCmd[4] = (uint8_t)(usData >> 8);
    vUartSendArray(emUart, aucCmd, 5u);
}

/* 回传频率就近取档（官方 REG.h 的 RRATE 表）。返回 0 = 表里没有（fHz 非法, 别下发）。 */
static uint8_t ucGyroHwtRateCodeFromHz(float fHz)
{
    static const float c_afHz[]  = { 0.2f, 0.5f, 1.0f, 2.0f, 5.0f, 10.0f, 20.0f, 50.0f, 100.0f, 200.0f };
    static const uint8_t c_aucCode[] = { 0x01u, 0x02u, 0x03u, 0x04u, 0x05u, 0x06u, 0x07u, 0x08u, 0x09u, 0x0Bu };
    uint8_t ucBest = 0u;
    uint8_t i;
    float   fBestDiff;

    if (fHz <= 0.0f)
    {
        return 0u;
    }
    fBestDiff = (c_afHz[0] > fHz) ? (c_afHz[0] - fHz) : (fHz - c_afHz[0]);
    for (i = 1u; i < (uint8_t)(sizeof(c_afHz) / sizeof(c_afHz[0])); i++)
    {
        float fDiff = (c_afHz[i] > fHz) ? (c_afHz[i] - fHz) : (fHz - c_afHz[i]);
        if (fDiff < fBestDiff)
        {
            fBestDiff = fDiff;
            ucBest = i;
        }
    }
    return c_aucCode[ucBest];
}

/* 开机出场宽限 [ms]：上电后这段时间内, **还没出过帧的那一口一律当"活的"**。
 * 两颗模块的启动时间不一样 —— ATK 初始化完 ~0.05s 就出帧, M0/HWT101 要先跑完开机配置
 * （解锁→频率→[yaw 置零]→[换波特率], 每步 100ms）≈ 0.5~0.6s 才出第一帧。不宽限的话每次上电
 * 都会把「M0 还没起来」判成「M0 死了」→ 切到 ATK → 4s 后再复原：日志里白多一对
 * `[fail]`+`[auto]`、OLED 闪一下 `ATK!`、`gyro_auto_sw` 白涨一次。
 * ⚠️ 这是**开机出场**的固定余量（取模块配置耗时 ~0.6s 的 5 倍），与停摆判定窗
 *    `GYRO_AUTO_SWITCH_MS` 无关, 也不是标定量 —— 别往「调参」Tab 里加。
 * ⚠️ 只在 `ulLastFrame == 0`（本上电周期还没出过帧）时起作用; 真掉线是"出过帧然后停了",
 *    走的是另一条判据, 不受宽限影响。 */
#define GYRO_STARTUP_GRACE_MS  3000u

/* ==================== 每口一份运行上下文 ====================
 * 两个任务各持一份、**互不共享**：三套解析器的状态都挂在这里, 于是"两口选同型号"
 * 也互不干扰（它们原来都是函数级 static, 两个任务会踩坏对方正在拼的半帧）。
 * 静态分配, 不占任务栈（传感器任务栈只有 256 字）。 */
typedef struct
{
    uint8_t       aucRx[UART_DMA_BUF_SIZE];  /* take-copy 暂存; 同时绑给 ATK 驱动当接收缓冲 */
    uint8_t       aucHwt[GYRO_HWT_FRAME_LEN];/* HWT101 帧组装 */
    uint8_t       ucHwtIdx;
    uint8_t       ucHwtNeed;
    stGyroM0RxTdf stM0Rx;                    /* M0 帧状态机（驱动提供, 按实例）*/
    uint32_t      ulLastFrame;               /* 本口最近解出一帧姿态的时刻; 0 = 从没解出 */
} stGyroPortCtxTdf;

static stGyroPortCtxTdf s_astPortCtx[2];     /* [0] = 主口 / [1] = 副口 */

/* 某口配的是哪颗设备（本文件 + 联调串用） */
static uint8_t ucGyroDevOfSlot(uint8_t ucIsSub)
{
    return (ucIsSub != 0u) ? (uint8_t)GYRO_SUB_DEV : (uint8_t)GYRO_MAIN_DEV;
}

/* 本口是不是"实际在发"的那一路 —— 判据是**有效源**, 不是操作员设的首选。 */
static uint8_t ucGyroSlotIsActive(uint8_t ucIsSub)
{
    return ((ucGetGyroSrcEff() != 0u) == (ucIsSub != 0u)) ? 1u : 0u;
}

/* 发布一帧姿态（只有有效源那一支会调到）
 * ⚠️ 2026-10 起这里统一扣「看路校准」的陀螺漂移偏置（编译期 ROADYAW_APPLY_EN=1 才扣）:
 *    看路校准步骤在到点时算出累计漂移写进 Beam 的 roadyaw_bias, 本函数是全场
 *    **唯一**的工作航向出口 —— 在这里扣, PID/巡线陀螺腿/支点转弯/OLED 显示全部一致
 *    受益, 不用每个消费者自己修。⚠️ 偏置变化的时机 = 看路校准步骤（车停着）,
 *    不存在"闭环咬着航向时 yaw 被拽走"的窗口。⚠️ 偏置上电=0 → 开着也无副作用。 */
static void vGyroPublishAttitude(float fRoll, float fPitch, float fYaw)
{
    float fYawPub = fYaw;

#if (ROADYAW_APPLY_EN != 0)
    if (fGetRoadYawBias() != 0.0f)
    {
        fYawPub = fYaw - fGetRoadYawBias();
        /* |bias| 钳在 ±ROADYAW_BIAS_MAX_DEG、|yaw| ≤ 180 ⇒ 越界最多一圈的零头,
         * 修一次就回 ±180 内 */
        if (fYawPub > 180.0f)       { fYawPub -= 360.0f; }
        else if (fYawPub < -180.0f) { fYawPub += 360.0f; }
    }
#endif
    vSetYawRaw(fYaw);        /* 原始航向存档: 世界坐标系专用（校准不影响世界系） */
    vSetAttitude(fRoll, fPitch, fYawPub);
    vSetYawFresh();          /* 供角度 PID 的 IMU 卡死检测 */
}

/* ==================== 换源联调串 ==================== */
/* 「从哪颗切到哪颗」——用户要求每次换源都发一条。口名 main/sub, 设备名按**该口的选型**
 * 查（所以选型填错时这条会照实报出"你以为的那颗"）。
 * ⚠️ 报文最长 "GYRO: main/ATK -> sub/HWT [fail]" = 33B（RPI_DATA_MAX 64B）—— 别再加字段。 */
static const char *c_pcGyroPortName[2] = { "main", "sub" };

static const char *c_pcGyroDevName(uint8_t ucIsSub)
{
    switch (ucGyroDevOfSlot(ucIsSub))
    {
        case GYRO_DEV_M0:     return "M0";
        case GYRO_DEV_HWT101: return "HWT";
        default:              return "ATK";
    }
}

static char *c_pcAppendStr(char *pcDst, const char *pcSrc)
{
    while (*pcSrc != '\0')
    {
        *pcDst++ = *pcSrc++;
    }
    *pcDst = '\0';
    return pcDst;
}

static void vGyroSendSwitchDebug(uint8_t ucFromIsSub, uint8_t ucToIsSub, const char *pcWhy)
{
    char  acBuf[48];
    char *pc = acBuf;

    pc = c_pcAppendStr(pc, "GYRO: ");
    pc = c_pcAppendStr(pc, c_pcGyroPortName[ucFromIsSub & 1u]);
    pc = c_pcAppendStr(pc, "/");
    pc = c_pcAppendStr(pc, c_pcGyroDevName(ucFromIsSub));
    pc = c_pcAppendStr(pc, " -> ");
    pc = c_pcAppendStr(pc, c_pcGyroPortName[ucToIsSub & 1u]);
    pc = c_pcAppendStr(pc, "/");
    pc = c_pcAppendStr(pc, c_pcGyroDevName(ucToIsSub));
    pc = c_pcAppendStr(pc, " [");
    pc = c_pcAppendStr(pc, pcWhy);
    (void)c_pcAppendStr(pc, "]");

    vRpiSendDebug(acBuf);
}

/* ==================== HWT101 解析（0x55 + 0x53/0x52 + 11B + 累加和）====================
 * 凑满一帧角度帧返回 1 并把 yaw 写进 *pfYaw, 否则返回 0。
 * ⚠️ 状态在 *pstCtx 里（每口一份）—— 别改回函数级 static。 */
static uint8_t ucHwtParseByte(stGyroPortCtxTdf *pstCtx, uint8_t ucByte, float *pfYaw)
{
    if (pstCtx->ucHwtIdx == 0u)
    {
        if (ucByte != GYRO_HWT_HEAD)
        {
            return 0u;                          /* 等待帧头 */
        }
        pstCtx->aucHwt[pstCtx->ucHwtIdx++] = ucByte;
        return 0u;
    }

    if (pstCtx->ucHwtIdx == 1u)
    {
        pstCtx->aucHwt[pstCtx->ucHwtIdx++] = ucByte;
        if (ucByte == GYRO_HWT_TYPE_ANGLE || ucByte == GYRO_HWT_TYPE_ANGLE2)
        {
            pstCtx->ucHwtNeed = GYRO_HWT_FRAME_LEN;
        }
        else
        {
            pstCtx->ucHwtIdx = 0u;              /* 未知类型，重新等帧头 */
        }
        return 0u;
    }

    pstCtx->aucHwt[pstCtx->ucHwtIdx++] = ucByte;
    if (pstCtx->ucHwtIdx < pstCtx->ucHwtNeed)
    {
        return 0u;
    }

    /* 一帧收齐，先复位状态（无论校验成败） */
    pstCtx->ucHwtIdx = 0u;

    /* 累加和校验（前 N-1 字节之和） */
    {
        uint8_t ucSum = 0u;
        uint8_t i;

        for (i = 0u; i < (uint8_t)(pstCtx->ucHwtNeed - 1u); i++)
        {
            ucSum += pstCtx->aucHwt[i];
        }
        if (ucSum != pstCtx->aucHwt[pstCtx->ucHwtNeed - 1u])
        {
            return 0u;
        }
    }

    /* 只有 0x53 是角度帧（0x52 同长度但不出角度） */
    if (pstCtx->aucHwt[1] != GYRO_HWT_TYPE_ANGLE)
    {
        return 0u;
    }

    {
        int16_t sYaw = (int16_t)(((uint16_t)pstCtx->aucHwt[7] << 8) | (uint16_t)pstCtx->aucHwt[6]);

        if (pfYaw != NULL)
        {
            /* WitMotion 常用比例：/32768*180（°），如与实际不符请按产品手册校准 */
            *pfYaw = (float)sYaw / 32768.0f * 180.0f;
        }
    }
    return 1u;
}

/* ==================== 各设备的一次性初始化（任务上下文）==================== */

/* 会合波特率 = CubeMX 里两口写死的上电波特率 = 模块 RAM 的上电默认值。三者都在 115200
 * 对齐, 这是「固件开机自动配波特率 / 不碰 CubeMX / 填错重新上电自愈」三条性质共同的前提
 * —— 所以 CubeMX 里这两口的波特率**保持 115200 别动**, 模块侧也**别用上位机保存**波特率。 */
#define GYRO_BOOT_BAUD  115200u

/* 波特率 → 该设备型号的档位码。返回 0 = 表里没有（编译期 #error 已拦, 这里是双保险:
 * 万一还是漏进来, 就**不下发、不换挡**, 两端都留在会合波特率上 —— 链路照样活着）。 */
static uint8_t ucGyroBaudCodeOf(uint8_t ucDev, uint32_t ulBaud)
{
    uint8_t ucCode;

    switch (ulBaud)
    {
        case 4800u:   ucCode = 1u; break;
        case 9600u:   ucCode = 2u; break;
        case 19200u:  ucCode = 3u; break;
        case 38400u:  ucCode = 4u; break;
        case 57600u:  ucCode = 5u; break;
        case 115200u: ucCode = 6u; break;
        case 230400u: ucCode = 7u; break;
        default:      return 0u;
    }
    if ((ucDev == (uint8_t)GYRO_DEV_M0) && (ucCode > 6u))
    {
        return 0u;                              /* M0 档位表只到 115200 */
    }
    if (ucDev == (uint8_t)GYRO_DEV_ATK)
    {
        return 0u;                              /* ATK 没有波特率旋钮 */
    }
    return ucCode;
}

/* 在**当前**波特率下写「回传频率 [+ yaw 置零]」, 唯独不碰波特率 —— 波特率必须留给
 * 最后一步, 否则后面还有命令要发、两端却已经对不上挡。
 * ⚠️ 全程不发保存帧（M0/HWT101 两套协议同一条纪律）: 只改模块 RAM, 断电自愈。 */
static void vGyroSendRateAndZero(uart_id_t emUart, uint8_t ucDev, float fRate, uint8_t ucZero)
{
    uint8_t ucCode;

    if (ucDev == (uint8_t)GYRO_DEV_M0)
    {
        /* 开机配置必须在调度器起来之后（内部有 vTaskDelay）。中间 100ms 一档 ——
         * 模块要求解锁后 20s 内改完, 绰绰有余。波特率那步由调用方编排（见下）。 */
        gyro_m0_apply_boot_config(emUart, fRate, (uint8_t)GYRO_M0_BAUD_115200, 0u, ucZero);
        return;
    }

    /* HWT101: 解锁 → 写回传频率 → [CALSW=Z轴置零]。 */
    vGyroHwtWriteReg(emUart, GYRO_HWT_REG_KEY, GYRO_HWT_KEY_UNLOCK);
    vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CFG_GAP_MS));

    ucCode = ucGyroHwtRateCodeFromHz(fRate);
    if (ucCode != 0u)
    {
        vGyroHwtWriteReg(emUart, GYRO_HWT_REG_RRATE, ucCode);
        vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CFG_GAP_MS));
    }

    if (ucZero != 0u)
    {
        vGyroHwtWriteReg(emUart, GYRO_HWT_REG_CALSW, GYRO_HWT_CAL_ZZERO);
        vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CFG_GAP_MS));
    }
}

/* 告诉模块新波特率（发完这条模块**立刻**换挡说话, 所以这是换挡流程的第一步）。 */
static void vGyroSendBaudCmd(uart_id_t emUart, uint8_t ucDev, uint8_t ucCode)
{
    if (ucCode == 0u)
    {
        return;
    }
    if (ucDev == (uint8_t)GYRO_DEV_M0)
    {
        gyro_m0_set_baud_code(emUart, ucCode);  /* 内部自带解锁 + 100ms 步距 */
        return;
    }
    vGyroHwtWriteReg(emUart, GYRO_HWT_REG_KEY, GYRO_HWT_KEY_UNLOCK);
    vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CFG_GAP_MS));
    vGyroHwtWriteReg(emUart, GYRO_HWT_REG_BAUD, ucCode);
    vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CFG_GAP_MS));
}

static void vGyroDevInit(uart_id_t emUart, uint8_t ucDev, uint8_t ucIsSub)
{
    float    fRate;
    uint32_t ulBaud;
    uint8_t  ucApplyBaud;
    uint8_t  ucZero;

    if (ucDev == (uint8_t)GYRO_DEV_ATK)
    {
        /* 读一次满量程备用，失败不影响姿态角显示，忽略返回值。
         * ATK 的协议没有频率/波特率旋钮 —— 选型段里那几个宏对它一律不理会。 */
        (void)atk_ms901m_init(emUart);
        return;
    }

    /* 本口的三个旋钮（app_config.h 选型段, 按口取名） */
    if (ucIsSub != 0u)
    {
        fRate      = (float)GYRO_SUB_RATE_HZ;
        ulBaud     = (uint32_t)GYRO_SUB_BAUD;
        ucApplyBaud = (uint8_t)GYRO_SUB_BAUD_APPLY;
    }
    else
    {
        fRate      = (float)GYRO_MAIN_RATE_HZ;
        ulBaud     = (uint32_t)GYRO_MAIN_BAUD;
        ucApplyBaud = (uint8_t)GYRO_MAIN_BAUD_APPLY;
    }
    /* 置零开关跟**型号**走, 不跟口 */
    ucZero = (ucDev == (uint8_t)GYRO_DEV_M0) ? (uint8_t)M0_GYRO_ZERO_ON_BOOT
                                             : (uint8_t)GYRO_HWT_ZERO_ON_BOOT;

    vGyroSendRateAndZero(emUart, ucDev, fRate, ucZero);

    if ((ucApplyBaud == 0u) || (ulBaud == GYRO_BOOT_BAUD) ||
        (ucGyroBaudCodeOf(ucDev, ulBaud) == 0u))
    {
        return;                                 /* 不换挡: 两端都留在会合波特率上, 与自配之前一致 */
    }

    /* 换挡三步, 顺序不能换:
     * ① 把新波特率告诉模块 —— 模块写完寄存器立刻换挡说话;
     * ② 等最后一个字节真的移出移位寄存器（TX 走 DMA 环, 不等就换挡会拦腰截断命令）;
     * ③ MCU 本口换挡。换挡后模块发来的头几帧可能是乱码（两端切换非原子, 正常现象）,
     *    解析器按帧头重同步 —— 这与「MCU 先换挡 = 自己聋到模块也换完为止」相比,
     *    是无害方向。 */
    vGyroSendBaudCmd(emUart, ucDev, ucGyroBaudCodeOf(ucDev, ulBaud));
    (void)ucUartWaitTxDone(emUart, pdMS_TO_TICKS(100u));
    vUartSetBaud(emUart, ulBaud);
}

/* ==================== 陀螺校准（OLED「陀螺校准」→ SCRIPT_REQ_GYRO_CAL）==================== */

/* HWT101「打开自动校准」后的静止等待。官方口径是"保持模块静止几秒钟"（期间角度/角速度
 * 输出暂停或不更新）; 短了平均不够, 长了车被摁着不动的时间难接受 —— 取 5s, 真觉得零偏
 * 还大就再校一次。⚠️ 这是**协议常数**不是标定量（同 GYRO_STARTUP_GRACE_MS 的口径）,
 * 别往「调参」Tab 里加。 */
#define GYRO_HWT_CAL_HOLD_MS  5000u

/**
  * @brief       校准「有效源」那颗陀螺（阻塞式, 详见 app_sensors.h）
  * @retval      1 = 校准动作走完; 0 = 没法校（有效源是 ATK, 本工程没接它的校准协议）
  *
  * 三颗设备三种待遇:
  *   M0     → gyro_m0_bias_cal（解锁→零偏自校准→等 21s→保存, 驱动收口）+ yaw 重新归零;
  *   HWT101 → 解锁→CALSW=1 开「陀螺自动校准」→静止 5s→CALSW=0 关→保存→CALSW=4 重零;
  *   ATK    → 拒绝（往 Pi 打一条调试串留痕, OLED 闪「校准失败!」）。
  *
  * ⚠️⚠️ HWT101 那条**等完必须把 CALSW 关掉**, 不留在开的状态: 官方手册对「匀速运行场景」
  *    （AGV 小车/舵机/云台, 本车原样命中 —— 支点转弯就是匀速旋转）的建议就是关闭自动校准,
  *    否则匀速旋转会被当零偏慢慢吃掉, 航向漂。发的 SAVE 是**有意**的（见 REG_SAVE 处注释）:
  *    校准的意义就在跨上电, 不落 Flash 等于白校 —— 与「配置不发保存帧」那条纪律不冲突,
  *    那条管的是频率/波特率这类填错能自愈的配置, 校准零偏不属于那类。
  * ⚠️ 收尾的「Z轴重新归零」把"校准时的朝向"定成 0°, 与两颗模块"上电朝向=0°"的口径对齐
  *    （不归零的话 HWT101 的角度输出带着漂移继续走, M0 的 yaw 保留旧累计值）。
  * ⚠️ 校准期间被校那颗不出帧是**预期**: Beam 的 gyro_cal_busy 在本函数进出时置/清,
  *    vGyroSrcWatchdogTick 见它就让路（不判停摆、不自动切源）。
  */
uint8_t ucGyroCalActiveSource(void)
{
    uint8_t   ucIsSub = (ucGetGyroSrcEff() != 0u) ? 1u : 0u;
    uint8_t   ucDev   = ucGyroDevOfSlot(ucIsSub);
    uart_id_t emUart  = (ucIsSub != 0u) ? UART_GYRO_SUB : UART_GYRO_MAIN;
    uint8_t   ucOk    = 0u;

    vSetGyroCalBusy(1u);   /* 先占位（watchdog 让路 + OLED 换「校准中」）, 再开始慢慢发 */

    switch (ucDev)
    {
    case (uint8_t)GYRO_DEV_M0:
        gyro_m0_bias_cal(emUart);          /* 解锁→校准→等 21s→保存（驱动收口, 内部有节拍） */
        gyro_m0_yaw_zero(emUart, 0u);      /* 朝向重新归零（只改 RAM, 同上电口径） */
        ucOk = 1u;
        break;

    case (uint8_t)GYRO_DEV_HWT101:
        /* 节奏与 vGyroSendRateAndZero 一致（命令间 100ms 一档）。⚠️ 命令**无应答帧**,
         * 写失败是静默的 —— 而本序列在 5s 静止等待之后还要写, 所以**等待之后的每个
         * 写组前都重新解锁**（一次 5 字节 + 100ms, 已解锁时是无害的重复）: 维特的
         * 解锁窗口虽 ≥20s, 但 SAVE 之后模块可能自动回锁, 别赌。 */
        vGyroHwtWriteReg(emUart, GYRO_HWT_REG_KEY, GYRO_HWT_KEY_UNLOCK);
        vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CFG_GAP_MS));
        vGyroHwtWriteReg(emUart, GYRO_HWT_REG_CALSW, GYRO_HWT_CAL_GYRON);
        vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CAL_HOLD_MS));   /* **车必须静止**的等待段 */

        vGyroHwtWriteReg(emUart, GYRO_HWT_REG_KEY, GYRO_HWT_KEY_UNLOCK);  /* 关+存 同组 */
        vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CFG_GAP_MS));
        vGyroHwtWriteReg(emUart, GYRO_HWT_REG_CALSW, GYRO_HWT_CAL_OFF);
        vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CFG_GAP_MS));
        vGyroHwtWriteReg(emUart, GYRO_HWT_REG_SAVE, 0x0000u);
        vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CFG_GAP_MS));

        vGyroHwtWriteReg(emUart, GYRO_HWT_REG_KEY, GYRO_HWT_KEY_UNLOCK);  /* SAVE 可能回锁 */
        vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CFG_GAP_MS));
        vGyroHwtWriteReg(emUart, GYRO_HWT_REG_CALSW, GYRO_HWT_CAL_ZZERO);
        vTaskDelay(pdMS_TO_TICKS(GYRO_HWT_CFG_GAP_MS));
        ucOk = 1u;
        break;

    default:                               /* ATK: 协议里没有我们能发的校准命令 */
        vRpiSendDebug("GYRO cal: ATK has no cal cmd");
        break;
    }

    vSetGyroCalBusy(0u);
    vSetGyroCalResult(ucOk);
    vSetGyroCalDoneTick(xTaskGetTickCount());
    return ucOk;
}

/* ==================== 按设备分派的主循环（不返回）==================== */
static void vGyroPortTask(uart_id_t emUart, uint8_t ucDev, uint8_t ucIsSub, stGyroPortCtxTdf *pstCtx)
{
    vGyroDevInit(emUart, ucDev, ucIsSub);

    for (;;)
    {
        if (ucDev == (uint8_t)GYRO_DEV_ATK)
        {
            atk_ms901m_attitude_data_t stAtt;

            /* 阻塞等一帧姿态角（内部自己 take-copy 到本口绑定的缓冲）；超时重试 */
            if (atk_ms901m_get_attitude(emUart, &stAtt, 500) == ATK_MS901M_EOK)
            {
                pstCtx->ulLastFrame = (uint32_t)xTaskGetTickCount();
                if (ucIsSub != 0u) { vBumpGyroFramesSub(); } else { vBumpGyroFramesMain(); }

                if (ucGyroSlotIsActive(ucIsSub) != 0u)
                {
                    vGyroPublishAttitude(stAtt.roll, stAtt.pitch, stAtt.yaw);
                }
            }
            else
            {
                vTaskDelay(pdMS_TO_TICKS(1));   /* 模块未就绪/未连接时降频，避免空转占满 CPU */
            }
        }
        else
        {
            /* M0 / HWT101 同构：阻塞等一段新字节 → 逐字节喂状态机。
             * ⚠️ 保持 portMAX_DELAY：本任务不需要自己醒来看停摆 —— 热备判据是
             *    `ulLastFrame` 的时间戳（看门狗在 vPidTask 里读）, 与这里醒不醒无关。 */
            uint16_t usLen = usUartDmaTakeCopy(emUart, pstCtx->aucRx,
                                               (uint16_t)sizeof(pstCtx->aucRx),
                                               portMAX_DELAY);
            uint16_t i;

            for (i = 0u; i < usLen; i++)
            {
                float   fYaw = 0.0f;
                uint8_t ucGot = 0u;

                if (ucDev == (uint8_t)GYRO_DEV_M0)
                {
                    ucGot = (gyro_m0_parse_byte(&pstCtx->stM0Rx, pstCtx->aucRx[i], &fYaw)
                             == GYRO_M0_TYPE_YAW) ? 1u : 0u;
                    if (ucGot != 0u)
                    {
                        /* 遥测：选没选中都更新（确认模块在不在 / 方向对不对的第一手证据）*/
                        vSetM0Yaw(fYaw);
                    }
                }
                else
                {
                    ucGot = ucHwtParseByte(pstCtx, pstCtx->aucRx[i], &fYaw);
                }

                if (ucGot != 0u)
                {
                    pstCtx->ulLastFrame = (uint32_t)xTaskGetTickCount();
                    if (ucIsSub != 0u) { vBumpGyroFramesSub(); } else { vBumpGyroFramesMain(); }

                    if (ucGyroSlotIsActive(ucIsSub) != 0u)
                    {
                        /* M0 模块顺时针为正、本工程逆时针为正 ⇒ 乘方向符号（HWT101 不用）*/
                        vGyroPublishAttitude(0.0f, 0.0f,
                                             (ucDev == (uint8_t)GYRO_DEV_M0)
                                                 ? (fYaw * M0_GYRO_YAW_SIGN) : fYaw);
                    }
                }
            }
        }
    }
}

/* ==================== 应用入口 / 两个任务 ==================== */

void vAppSensorInit(void)
{
    /* 两种初始化都要阻塞等应答帧 / 有延时, 依赖 FreeRTOS 信号量与 vTaskDelay,
       已移到各自任务首次执行（调度器启动之后）。此处留空。 */
}

void vSensorTask(void *argument)
{
    (void)argument;

    /* ⚠️ 接收缓冲绑定必须在进循环之前：ATK 的接收暂存是按口取的, 不绑就会两个口
     *    共用驱动内部那份默认缓冲（= 退回改造前的互相踩踏）。不挂 ATK 的口绑了也无害。 */
    atk_ms901m_bind_rx_buf(UART_GYRO_MAIN, s_astPortCtx[0].aucRx);
    vGyroPortTask(UART_GYRO_MAIN, (uint8_t)GYRO_MAIN_DEV, 0u, &s_astPortCtx[0]);
}

void vSensorTaskM0(void *argument)
{
    (void)argument;

    atk_ms901m_bind_rx_buf(UART_GYRO_SUB, s_astPortCtx[1].aucRx);
    vGyroPortTask(UART_GYRO_SUB, (uint8_t)GYRO_SUB_DEV, 1u, &s_astPortCtx[1]);
}

/* ==================== 陀螺源热备看门狗（自动切换）====================
 * 由 vPidTask 的 50ms 节拍调用（见 app_sensors.h）。
 *
 * 前提：两个口**都在解析**（各自的任务一直在跑, 与选没选中无关）⇒ 每口的 `ulLastFrame`
 * 一直反映"那颗模块现在还在不在说话"。
 *
 * 规则（幂等, 每拍全量重算）：
 *   ① 操作员改了首选（蓝牙 gyro_src / 上电默认）→ 有效源立刻跟随（**手动优先**）；
 *   ② 有效源停摆 **且** 另一路还活着 → 有效源 := 另一路（立刻切, 跑路线中也切）；
 *   ③ 有效源 ≠ 首选 **且** 首选连续两个判定窗都健康 → 有效源 := 首选（**热备复原**）。
 * 关掉 gyro_auto_switch 时只跑 ① —— 有效源恒等于首选, 行为与加这个功能之前一致。
 *
 * ⚠️ 规则 ② 里"另一路也要活着"是防抖的关键：两路都死时**不能**动作, 否则会在两者间
 *    每 GYRO_AUTO_SWITCH_MS 来回跳一次、yaw 反复跳零点。
 * ⚠️ 规则 ③ 要"两个窗"是给首选的恢复去抖：只在窗口里冒一帧不算数。
 * ⚠️ **开机头 GYRO_STARTUP_GRACE_MS 内, "还没出过帧"不算死**（两颗模块出场时间不同,
 *    见该宏的说明）—— 少了这条, 每次上电都会白跳一轮 ATK↔M0。
 * ⚠️ 换源瞬间 fGetAttitudeYaw() 会跳到另一颗的零点（两颗都以上电朝向为零点, 正常只差
 *    几度）—— 正在跑的绝对航向闭环 / 巡线陀螺档会看到这个突变。这是"立刻切"的已知代价。
 * ⚠️ 它**不写** `gyro_src`（那是操作员的量）, 只写有效源。合成一个变量就没有"切回首选"了。
 */
static uint8_t ucGyroPortLive(uint8_t ucIsSub, uint32_t ulNow, uint32_t ulWin);

void vGyroSrcWatchdogTick(void)
{
    static uint8_t  s_ucLastPref    = 0xFFu;  /* 上次看到的首选（0xFF = 还没见过）*/
    static uint32_t s_ulPrefStaleAt = 0u;     /* 首选最近一次被判"不健康"的时刻 */

    uint32_t ulNow  = (uint32_t)xTaskGetTickCount();
    uint32_t ulWin  = (uint32_t)GYRO_AUTO_SWITCH_MS;
    uint8_t  ucPref = ucGetGyroSrc();
    uint8_t  ucEff  = ucGetGyroSrcEff();
    uint8_t  ucPrefLive;

    /* ---- 规则 ①：操作员改了首选 → 立刻跟随（与自动切换开关无关）---- */
    if (s_ucLastPref != ucPref)
    {
        s_ucLastPref    = ucPref;
        s_ulPrefStaleAt = 0u;                 /* 重新计"连续健康" */

        if (ucEff != ucPref)
        {
            vSetGyroSrcEff(ucPref);
            vGyroSendSwitchDebug(ucEff, ucPref, "man");
        }
        return;                               /* 本拍不下停摆结论, 免得拿旧时刻判 */
    }

    if (ucGetGyroAutoSwitch() == 0u)
    {
        return;                               /* 关掉 = 只保留手动（上面那段已处理）*/
    }

    /* ---- 陀螺校准进行中 → 整段让路 ----
     * 校准（ucGyroCalActiveSource）期间被校那颗**不出帧是预期动作**（HWT101 官方口径
     * "校准期间输出暂停或不更新"; M0 那边模块忙 21s 同理）—— 按停摆判就是白切一轮:
     * 切去另一颗、校准完再切回来, 白涨 gyro_auto_sw + 白打两条换源 DBG。 */
    if (ucGetGyroCalBusy() != 0u)
    {
        return;
    }

    /* ---- 首选健康度（顺带维护"连续健康"的起点）---- */
    ucPrefLive = ucGyroPortLive(ucPref, ulNow, ulWin);
    if (ucPrefLive == 0u)
    {
        s_ulPrefStaleAt = ulNow;
    }

    /* ---- 规则 ②：有效源停摆 + 另一路活着 → 立刻换 ---- */
    if (ucGyroPortLive(ucEff, ulNow, ulWin) == 0u)
    {
        uint8_t ucOther = (uint8_t)(ucEff ^ 1u);

        if (ucGyroPortLive(ucOther, ulNow, ulWin) != 0u)
        {
            vSetGyroSrcEff(ucOther);
            vBumpGyroAutoSw();
            vGyroSendSwitchDebug(ucEff, ucOther, "fail");
        }
        return;
    }

    /* ---- 规则 ③：有效源 ≠ 首选, 且首选连续两个窗健康 → 复原 ---- */
    if ((ucEff != ucPref) && (ucPrefLive != 0u)
        && ((uint32_t)(ulNow - s_ulPrefStaleAt) >= (2u * ulWin)))
    {
        vSetGyroSrcEff(ucPref);
        vBumpGyroAutoSw();
        vGyroSendSwitchDebug(ucEff, ucPref, "auto");
    }
}

/* 某口最近还在不在出帧。无符号减法天然处理 tick 回绕。
 * ⚠️ 两种"不出帧"要分开判：
 *    ① **从没出过帧**（`ulLastFrame == 0`）—— 开机头几秒是正常的（两模块出场时间不同,
 *       见 GYRO_STARTUP_GRACE_MS），所以宽限期内一律算活; 过了宽限期才算真死。
 *       ⚠️ 不能简单地把 0 当"刚刚还很新鲜"（`ulNow - 0` 在开机头两秒小于任何窗口,
 *          那样真故障永远判不出来）, 也不能一律当死（那样每次开机都误判一轮）。
 *    ② **出过帧然后停了** —— 真掉线, 直接用时间差判, 不受开机宽限影响。 */
static uint8_t ucGyroPortLive(uint8_t ucIsSub, uint32_t ulNow, uint32_t ulWin)
{
    uint32_t ulLast = s_astPortCtx[ucIsSub & 1u].ulLastFrame;

    if (ulLast == 0u)
    {
        return (ulNow < GYRO_STARTUP_GRACE_MS) ? 1u : 0u;
    }
    return ((uint32_t)(ulNow - ulLast) < ulWin) ? 1u : 0u;
}
