/**
  * @file       gyro_m0_device.c
  * @brief      M0 副陀螺仪（单轴 Z 航向）驱动实现。
  *
  *             协议、接线、以及「为什么符号要由调用方翻」见 gyro_m0_device.h。
  *             收发都走上层 app_uart（usUartDmaTakeCopy / vUartSendArray），
  *             不直接碰 HAL —— 与 atk_ms901m_device.c 同一条路子。
  *
  *             ⚠️ 本文件里所有会阻塞的函数（凡是带 vTaskDelay 的）**只能从任务上下文调**。
  */

#include "gyro_m0_device.h"
#include "FreeRTOS.h"
#include "task.h"

/* ==================== 命令字节 ====================
 * 都是「0x55 0xAA reg dataL dataH」。解锁用的是魔数 0x8E5F（不是校验和）。 */
#define GYRO_M0_UNLOCK_L     0x8Eu
#define GYRO_M0_UNLOCK_H     0x5Fu

/* 解锁后到下一次写之间的间隔（示例驱动用 100ms 忙等；这里让出 CPU） */
#define GYRO_M0_CFG_GAP_MS   100u

/* 发送帧的取模取整（vUartSendArray 的 ulLen 形参） */
#define GYRO_M0_CMD_SEND_LEN ((uint32_t)GYRO_M0_CMD_LEN)

void gyro_m0_send_cmd(uart_id_t emUart, uint8_t ucReg, uint8_t ucDataL, uint8_t ucDataH)
{
    uint8_t aucCmd[GYRO_M0_CMD_LEN];

    aucCmd[0] = GYRO_M0_CMD_HEAD1;
    aucCmd[1] = GYRO_M0_CMD_HEAD2;
    aucCmd[2] = ucReg;
    aucCmd[3] = ucDataL;
    aucCmd[4] = ucDataH;

    /* 入 TX 环即返回; 本函数不长占串口 */
    vUartSendArray(emUart, aucCmd, GYRO_M0_CMD_SEND_LEN);
}

/* ==================== 接收解析 ====================
 * ⚠️ 状态**全在调用方给的上下文里**（`pstRx`），不再是函数级 static —— 解析器因此
 *    可按实例重入：主/副两个陀螺口各持一份自己的上下文，两口都挂 M0 也互不干扰。
 *    同一份上下文仍**只允许一个任务喂**（两条流共用一个 = 半帧交错、解出垃圾）。 */
uint8_t gyro_m0_parse_byte(stGyroM0RxTdf *pstRx, uint8_t ucByte, float *pfVal)
{
    uint8_t *aucBuf = pstRx->aucBuf;

    if (pstRx->ucIdx == 0u)
    {
        if (ucByte != GYRO_M0_FRAME_HEAD)
        {
            return 0u;                          /* 等帧头 */
        }
        aucBuf[pstRx->ucIdx++] = ucByte;
        return 0u;
    }

    if (pstRx->ucIdx == 1u)
    {
        if (ucByte != GYRO_M0_TYPE_RATE && ucByte != GYRO_M0_TYPE_YAW)
        {
            /* 未知类型：这一字节本身可能就是下一个帧头，别白丢 */
            pstRx->ucIdx = 0u;
            if (ucByte == GYRO_M0_FRAME_HEAD)
            {
                aucBuf[pstRx->ucIdx++] = ucByte;
            }
            return 0u;
        }
        aucBuf[pstRx->ucIdx++] = ucByte;
        return 0u;
    }

    aucBuf[pstRx->ucIdx++] = ucByte;
    if (pstRx->ucIdx < GYRO_M0_FRAME_LEN)
    {
        return 0u;
    }

    /* 一帧收齐，先复位状态（无论校验成败） */
    pstRx->ucIdx = 0u;

    /* 校验和 = 前 4 字节之和（8 位自然溢出） */
    if ((uint8_t)(aucBuf[0] + aucBuf[1] + aucBuf[2] + aucBuf[3]) != aucBuf[4])
    {
        return 0u;
    }

    /* 小端 int16 -> 物理量 */
    {
        int16_t sRaw = (int16_t)((uint16_t)aucBuf[2] | ((uint16_t)aucBuf[3] << 8));
        float   fScale;

        if (aucBuf[1] == GYRO_M0_TYPE_YAW)
        {
            fScale = 180.0f / 32768.0f;         /* 航向角 ±180° */
        }
        else
        {
            fScale = 2000.0f / 32768.0f;        /* 角速度 ±2000°/s */
        }

        if (pfVal != NULL)
        {
            *pfVal = (float)sRaw * fScale;
        }
    }

    /* 刻意不在这里乘方向符号：那是调用方的策略，见 .h */
    return aucBuf[1];
}

uint8_t gyro_m0_read_yaw(uart_id_t emUart, float *pfYaw, uint32_t ulTimeoutMs)
{
    TickType_t    xDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(ulTimeoutMs);
    /* static 是为了不占任务栈（传感器任务栈 256 字）; 本函数只被单线程调用 */
    static uint8_t aucRx[UART_DMA_BUF_SIZE];
    /* 本函数**自带一份私有解析上下文** —— 与 parse_byte 的调用方（app_sensors.c 的
     * 两个陀螺口任务）不共享, 所以两者不能并发喂同一路串口。本次无调用点。 */
    static stGyroM0RxTdf s_stRx;

    for (;;)
    {
        TickType_t xNow = xTaskGetTickCount();
        uint16_t   usLen;
        uint16_t   i;

        /* 有符号差值比较: 裸的 xNow >= xDeadline 在 tick 回绕（~49.8 天 @1kHz）那一刻
         * 会把"早到期"误判成"还有很久"。±24 天窗口内两种写法等价, 取工程惯用那种。 */
        if ((int32_t)(xNow - xDeadline) >= 0)
        {
            return 0u;
        }

        usLen = usUartDmaTakeCopy(emUart, aucRx, (uint16_t)sizeof(aucRx), xDeadline - xNow);
        if (usLen == 0u)
        {
            return 0u;                          /* 超时无新数据 */
        }

        for (i = 0u; i < usLen; i++)
        {
            float fVal = 0.0f;

            if (gyro_m0_parse_byte(&s_stRx, aucRx[i], &fVal) == GYRO_M0_TYPE_YAW)
            {
                if (pfYaw != NULL)
                {
                    *pfYaw = fVal;
                }
                return 1u;
            }
        }
    }
}

/* ==================== 回传频率档位表 ====================
 * 模块只认这 15 个档码；调用方填 Hz，这里取最近的一档。 */
typedef struct
{
    float   fHz;
    uint8_t ucCode;
} stGyroM0RateTdf;

static const stGyroM0RateTdf s_astRateTable[] =
{
    {    0.1f, 0x00u }, {    0.2f, 0x01u }, {    0.5f, 0x02u },
    {    1.0f, 0x03u }, {    2.0f, 0x04u }, {    5.0f, 0x05u },
    {   10.0f, 0x06u }, {   20.0f, 0x07u }, {   50.0f, 0x08u },
    {  100.0f, 0x09u }, {  125.0f, 0x0Au }, {  200.0f, 0x0Bu },
    {  250.0f, 0x0Cu }, {  500.0f, 0x0Du }, { 1000.0f, 0x0Eu },
};

#define GYRO_M0_RATE_TABLE_NUM (sizeof(s_astRateTable) / sizeof(s_astRateTable[0]))

uint8_t gyro_m0_rate_code_from_hz(float fHz, uint8_t *pucCode)
{
    uint32_t ulBest = 0u;
    uint32_t i;
    float    fBestDiff;

    if (pucCode == NULL || fHz <= 0.0f)
    {
        return 0u;
    }

    fBestDiff = (s_astRateTable[0].fHz > fHz) ? (s_astRateTable[0].fHz - fHz)
                                              : (fHz - s_astRateTable[0].fHz);

    for (i = 1u; i < (uint32_t)GYRO_M0_RATE_TABLE_NUM; i++)
    {
        float fDiff = (s_astRateTable[i].fHz > fHz) ? (s_astRateTable[i].fHz - fHz)
                                                    : (fHz - s_astRateTable[i].fHz);

        if (fDiff < fBestDiff)
        {
            fBestDiff = fDiff;
            ulBest = i;
        }
    }

    *pucCode = s_astRateTable[ulBest].ucCode;
    return 1u;
}

/* ==================== 配置命令 ==================== */

void gyro_m0_set_rate_hz(uart_id_t emUart, float fHz)
{
    uint8_t ucCode;

    if (gyro_m0_rate_code_from_hz(fHz, &ucCode) == 0u)
    {
        return;
    }

    gyro_m0_send_cmd(emUart, GYRO_M0_REG_UNLOCK, GYRO_M0_UNLOCK_L, GYRO_M0_UNLOCK_H);
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_OUT_RATE, ucCode, 0x00u);
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
}

void gyro_m0_set_baud_code(uart_id_t emUart, uint8_t ucCode)
{
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_UNLOCK, GYRO_M0_UNLOCK_L, GYRO_M0_UNLOCK_H);
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_BAUD, ucCode, 0x00u);
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
}

void gyro_m0_yaw_zero(uart_id_t emUart, uint8_t ucSave)
{
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_UNLOCK, GYRO_M0_UNLOCK_L, GYRO_M0_UNLOCK_H);
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_YAW_ZERO, 0x00u, 0x00u);
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));

    if (ucSave != 0u)
    {
        gyro_m0_send_cmd(emUart, GYRO_M0_REG_SAVE, 0x00u, 0x00u);
        vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
    }
}

void gyro_m0_apply_boot_config(uart_id_t emUart, float fHz, uint8_t ucBaudCode,
                               uint8_t ucApplyBaud, uint8_t ucZeroOnBoot)
{
    uint8_t ucCode;

    /* 解锁。之后模块要求 20s 内把配置改完 —— 下面每一步最多 100ms，绰绰有余。 */
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_UNLOCK, GYRO_M0_UNLOCK_L, GYRO_M0_UNLOCK_H);
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));

    if (gyro_m0_rate_code_from_hz(fHz, &ucCode) != 0u)
    {
        gyro_m0_send_cmd(emUart, GYRO_M0_REG_OUT_RATE, ucCode, 0x00u);
        vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
    }

    if (ucApplyBaud != 0u)
    {
        gyro_m0_send_cmd(emUart, GYRO_M0_REG_BAUD, ucBaudCode, 0x00u);
        vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
    }

    if (ucZeroOnBoot != 0u)
    {
        gyro_m0_send_cmd(emUart, GYRO_M0_REG_YAW_ZERO, 0x00u, 0x00u);
        vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
    }

    /* 刻意不发保存帧：以上全部只改模块 RAM。断电模块即回它自己的默认值 ——
     * 这正是「波特率填错也能自愈」的依据，别顺手补一句保存。 */
}

/* ==================== 维护命令（本次无调用点）==================== */

void gyro_m0_bias_cal(uart_id_t emUart)
{
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_UNLOCK, GYRO_M0_UNLOCK_L, GYRO_M0_UNLOCK_H);
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_BIAS_CAL, 0x01u, 0x00u);   /* 0x0001 = 零偏自校准 */

    /* 模块要求静止约 21 秒。例程是忙等 21000ms；这里必须让出 CPU，
     * 且调用方必须是「不发布 yaw 的任务」（见 .h 的警告）。 */
    vTaskDelay(pdMS_TO_TICKS(21000u));

    gyro_m0_send_cmd(emUart, GYRO_M0_REG_SAVE, 0x00u, 0x00u);
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
}

void gyro_m0_factory_reset(uart_id_t emUart)
{
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_UNLOCK, GYRO_M0_UNLOCK_L, GYRO_M0_UNLOCK_H);
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_SAVE, 0x01u, 0x00u);       /* 0x0001 = 恢复出厂 */
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
}

void gyro_m0_reboot(uart_id_t emUart)
{
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_UNLOCK, GYRO_M0_UNLOCK_L, GYRO_M0_UNLOCK_H);
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
    gyro_m0_send_cmd(emUart, GYRO_M0_REG_SAVE, 0xFFu, 0x00u);       /* 0x00FF = 重启 */
    vTaskDelay(pdMS_TO_TICKS(GYRO_M0_CFG_GAP_MS));
}
