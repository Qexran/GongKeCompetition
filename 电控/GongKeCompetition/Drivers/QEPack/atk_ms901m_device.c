/**
  * @file       adc_device.h
  * @author     正点原子团队(ALIENTEK) & Qe_xr
  * @version    V1.2.0
  * @date       2026/2/17
  * @brief      ATK-MS901M 模块驱动，基于 STM32 HAL 库
  * 
  */


#include "atk_ms901m_device.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#if ATK_MS901M_IS_ENABLE

/* ATK-MS901M UART通讯帧头 */
#define ATK_MS901M_FRAME_HEAD_L             0x55
#define ATK_MS901M_FRAME_HEAD_UPLOAD_H      0x55    /* 高位主动上传帧头 */
#define ATK_MS901M_FRAME_HEAD_ACK_H         0xAF    /* 高位应答帧头 */

/* ATK-MS901M读写寄存器ID */
#define ATK_MS901M_READ_REG_ID(id)         (id | 0x80)
#define ATK_MS901M_WRITE_REG_ID(id)        (id)

typedef struct
{
    uint8_t head_l;                                 /* 低位帧头 */
    uint8_t head_h;                                 /* 高位帧头 */
    uint8_t id;                                     /* 帧ID */
    uint8_t len;                                    /* 数据长度 */
    uint8_t dat[ATK_MS901M_FRAME_DAT_MAX_SIZE];     /* 数据 */
    uint8_t check_sum;                              /* 校验和 */
} atk_ms901m_frame_t;                               /* ATK-MS901M UART通讯帧结构体 */

typedef enum
{
    wait_for_head_l = 0x00,                         /* 等待低位帧头 */
    wait_for_head_h = 0x01,                         /* 等待高位帧头 */
    wait_for_id     = 0x02,                         /* 等待帧ID */
    wait_for_len    = 0x04,                         /* 等待数据长度 */
    wait_for_dat    = 0x08,                         /* 等待数据 */
    wait_for_sum    = 0x16,                         /* 等待校验和 */
} atk_ms901m_handle_state_t;                        /* 帧处理状态机状态枚举 */

/* 陀螺仪、加速度计满量程表 */
static const uint16_t g_atk_ms901m_gyro_fsr_table[4] = {250, 500, 1000, 2000};
static const uint8_t g_atk_ms901m_accelerometer_fsr_table[4] = {2, 4, 8, 16};

static struct
{
    uint8_t gyro;                                   /* 陀螺仪满量程 */
    uint8_t accelerometer;                          /* 加速度计满量程 */
} g_atk_ms901m_fsr;                                 /* ATK-MS901M满量程数据 */

/* ==================== 接收暂存: 按 UART 口绑定（让驱动可按实例重入）====================
 * 原来这里是 get_frame_by_id 里的一个函数级 static —— 两个口同时挂 ATK 时, 两个任务
 * 会互相踩坏对方正在解析的那一段（静默解出错数）。现在按 **口** 取缓冲: 一路串口只
 * 归一个任务, 所以按口分开就够。没绑定的口回退到本文件内的默认缓冲 ⇒ **单实例用法
 * 与改造前逐字节一致**。
 * ⚠️ 这只解决"两个口各一份缓冲"; 驱动**仍不是线程安全的**: 同一个口被两个任务并发
 *    调用一样坏（本工程不可能 —— 一路串口只由一个传感器任务消费）。
 * ⚠️ 满量程 g_atk_ms901m_fsr 仍是全局单份（只被 atk_ms901m_get_gyro_accelerometer
 *    读, 本工程不调用）—— 将来要用它得连它一起做成按实例。 */
static uint8_t  s_aucRxDefault[UART_DMA_BUF_SIZE];
static uint8_t *s_apucRx[UART_ID_NUM];

void atk_ms901m_bind_rx_buf(uart_id_t emDevNum, uint8_t *pucBuf)
{
    if ((uint32_t)emDevNum < (uint32_t)UART_ID_NUM)
    {
        s_apucRx[emDevNum] = pucBuf;
    }
}

static uint8_t *atk_ms901m_rx_buf_of(uart_id_t emDevNum)
{
    if (((uint32_t)emDevNum < (uint32_t)UART_ID_NUM) && (s_apucRx[emDevNum] != NULL))
    {
        return s_apucRx[emDevNum];
    }
    return s_aucRxDefault;
}

/* 帧头重同步（2026-10-09 加）: 某字节不合预期时, 它本身可能恰是**下一帧的帧头**(0x55) ——
 * 直接丢掉要再错一整帧。上传流一帧丢了还有下一帧, ACK 应答帧却只有一帧, 值得救。
 * 与 gyro_m0_device.c::gyro_m0_parse_byte 的「未知类型: 这一字节可能是下一个帧头」同款。 */
static atk_ms901m_handle_state_t atk_resync_byte(atk_ms901m_frame_t *frame, uint8_t dat)
{
    if (dat == ATK_MS901M_FRAME_HEAD_L)
    {
        frame->head_l = dat;
        frame->check_sum = frame->head_l;
        return wait_for_head_h;
    }
    return wait_for_head_l;
}

/**
 * @brief       通过指定帧ID获取接收到的数据帧
 * @param       emDevNum: UART设备号
 * @param       frame  : 接收到的数据帧
 * @param       id     : 指定帧ID
 * @param       id_type: 帧ID类型
 *                       ATK_MS901M_FRAME_ID_TYPE_UPLOAD: ATK-MS901M主动上传帧ID
 *                       ATK_MS901M_FRAME_ID_TYPE_ACK   : ATK-MS901M应答帧ID
 * @param       timeout: 等待数据帧最大超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK     : 获取指定数据帧成功
 *              ATK_MS901M_EINVAL  : 错误函数参数，获取指定数据帧失败
 *              ATK_MS901M_ETIMEOUT: 接收数据帧超时，获取指定数据帧失败
 */
static uint8_t atk_ms901m_get_frame_by_id(uart_id_t emDevNum, atk_ms901m_frame_t *frame, uint8_t id, uint8_t id_type, uint32_t timeout)
{
    atk_ms901m_handle_state_t handle_state = wait_for_head_l;
    uint8_t dat_index = 0;
    TickType_t xDeadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout);
    /* 本次新增字节暂存: 取满一圈, 任何一次 take 都放得下（不会因容量丢帧）。
     * 按 **口** 取（见 atk_ms901m_bind_rx_buf 的说明）——不给本函数的调用方白占栈,
     * 同时让两个口各解各的。状态机变量 handle_state 本就是函数内局部量。 */
    uint8_t *aucRx = atk_ms901m_rx_buf_of(emDevNum);

    for (;;)
    {
        TickType_t xNow = xTaskGetTickCount();
        uint16_t usLen;
        uint16_t i;

        /* 有符号差值比较: 防裸比较在 tick 回绕（~49.8 天）时的误判, 同 gyro_m0_device.c */
        if ((int32_t)(xNow - xDeadline) >= 0)
        {
            return ATK_MS901M_ETIMEOUT;
        }

        /* 阻塞等待新字节（DMA + IDLE + 二值信号量; 起始偏移/回绕由驱动吸收） */
        usLen = usUartDmaTakeCopy(emDevNum, aucRx, (uint16_t)UART_DMA_BUF_SIZE, xDeadline - xNow);
        if (usLen == 0)
        {
            return ATK_MS901M_ETIMEOUT;
        }

        /* 在本次字节内遍历解析（状态机跨 take 保持, 半帧留到下次） */
        for (i = 0; i < usLen; i++)
        {
            uint8_t dat = aucRx[i];

        switch (handle_state)
        {
            case wait_for_head_l:
            {
                if (dat == ATK_MS901M_FRAME_HEAD_L)
                {
                    frame->head_l = dat;
                    frame->check_sum = frame->head_l;
                    handle_state = wait_for_head_h;
                }
                else
                {
                    handle_state = wait_for_head_l;
                }
                break;
            }
            case wait_for_head_h:
            {
                switch (id_type)
                {
                    case ATK_MS901M_FRAME_ID_TYPE_UPLOAD:
                    {
                        if (dat == ATK_MS901M_FRAME_HEAD_UPLOAD_H)
                        {
                            frame->head_h = dat;
                            frame->check_sum += frame->head_h;
                            handle_state = wait_for_id;
                        }
                        else
                        {
                            handle_state = atk_resync_byte(frame, dat);
                        }
                        break;
                    }
                    case ATK_MS901M_FRAME_ID_TYPE_ACK:
                    {
                        if (dat == ATK_MS901M_FRAME_HEAD_ACK_H)
                        {
                            frame->head_h = dat;
                            frame->check_sum += frame->head_h;
                            handle_state = wait_for_id;
                        }
                        else
                        {
                            handle_state = atk_resync_byte(frame, dat);
                        }
                        break;
                    }
                    default:
                    {
                        return ATK_MS901M_EINVAL;
                    }
                }
                break;
            }
            case wait_for_id:
            {
                if (dat == id)
                {
                    frame->id = dat;
                    frame->check_sum += frame->id;
                    handle_state = wait_for_len;
                }
                else
                {
                    handle_state = atk_resync_byte(frame, dat);
                }
                break;
            }
            case wait_for_len:
            {
                if (dat > ATK_MS901M_FRAME_DAT_MAX_SIZE)
                {
                    handle_state = atk_resync_byte(frame, dat);
                }
                else
                {
                    frame->len = dat;
                    frame->check_sum += frame->len;
                    if (frame->len == 0)
                    {
                        handle_state = wait_for_sum;
                    }
                    else
                    {
                        handle_state = wait_for_dat;
                    }
                }
                break;
            }
            case wait_for_dat:
            {
                frame->dat[dat_index] = dat;
                frame->check_sum += frame->dat[dat_index];
                dat_index++;
                if (dat_index == frame->len)
                {
                    dat_index = 0;
                    handle_state = wait_for_sum;
                }
                break;
            }
            case wait_for_sum:
            {
                if (dat == frame->check_sum)
                {
                    return ATK_MS901M_EOK;
                }
                handle_state = atk_resync_byte(frame, dat);
                break;
            }
            default:
            {
                handle_state = wait_for_head_l;
                break;
            }
        }
        }   /* for (i < usLen) */
    }   /* for (;;) */
}

/**
 * @brief       通过帧ID读取ATK-MS901M寄存器
 * @param       emDevNum: UART设备号
 * @param       id     : 寄存器对应的通讯帧ID
 * @param       dat    : 读取到的数据
 * @param       ucDatCap: dat 缓冲容量（字节）。设备回报的 len 只受帧内上限(28)约束,
 *                       与调用方缓冲无关 —— 异常回报时按容量截断, 不越界写（2026-10-09 加）。
 * @param       timeout: 等待数据的最大超时时间，单位：毫秒（ms）
 * @retval      0: 读取失败
 *              其他值: 读取到数据的长度
 */
uint8_t atk_ms901m_read_reg_by_id(uart_id_t emDevNum, uint8_t id, uint8_t *dat, uint8_t ucDatCap, uint32_t timeout)
{
    uint8_t buf[7];
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    uint8_t dat_index;

    buf[0] = ATK_MS901M_FRAME_HEAD_L;
    buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
    buf[2] = ATK_MS901M_READ_REG_ID(id);
    buf[3] = 1;
    buf[4] = 0;
    buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
    vUartSendArray(emDevNum, buf, 6);
    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, id, ATK_MS901M_FRAME_ID_TYPE_ACK, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return 0;
    }

    for (dat_index=0; (dat_index<frame.len) && (dat_index<ucDatCap); dat_index++)
    {
        dat[dat_index] = frame.dat[dat_index];
    }

    return frame.len;
}

/**
 * @brief       通过帧ID写入ATK-MS901M寄存器
 * @param       emDevNum: UART设备号
 * @param       id     : 寄存器对应的通讯帧ID
 * @param       len    : 待写入数据长度（1或2）
 * @param       dat    : 待写入的数据
 * @retval      ATK_MS901M_EOK   : 寄存器写入成功
 *              ATK_MS901M_EINVAL: 函数参数len有误
 */
uint8_t atk_ms901m_write_reg_by_id(uart_id_t emDevNum, uint8_t id, uint8_t len, uint8_t *dat)
{
    uint8_t buf[7];
    
    buf[0] = ATK_MS901M_FRAME_HEAD_L;
    buf[1] = ATK_MS901M_FRAME_HEAD_ACK_H;
    buf[2] = ATK_MS901M_WRITE_REG_ID(id);
    buf[3] = len;
    if (len == 1)
    {
        buf[4] = dat[0];
        buf[5] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4];
        vUartSendArray(emDevNum, buf, 6);
    }
    else if (len == 2)
    {
        buf[4] = dat[0];
        buf[5] = dat[1];
        buf[6] = buf[0] + buf[1] + buf[2] + buf[3] + buf[4] + buf[5];
        vUartSendArray(emDevNum, buf, 7);
    }
    else
    {
        return ATK_MS901M_EINVAL;
    }
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       ATK-MS901M初始化
 * @param       emDevNum: UART设备号
 * @retval      ATK_MS901M_EOK  : ATK-MS901M初始化成功
 *              ATK_MS901M_ERROR: ATK-MS901M初始化失败
 */
uint8_t atk_ms901m_init(uart_id_t emDevNum)
{
    uint8_t ret;
    
    /* 获取ATK-MS901M陀螺仪满量程 */
    ret = atk_ms901m_read_reg_by_id(emDevNum, ATK_MS901M_FRAME_ID_REG_GYROFSR, &g_atk_ms901m_fsr.gyro,
                            (uint8_t)sizeof(g_atk_ms901m_fsr.gyro), 200);
    if (ret == 0)
    {
        return ATK_MS901M_ERROR;
    }
    
    /* 获取ATK-MS901M加速度计满量程 */
    ret = atk_ms901m_read_reg_by_id(emDevNum, ATK_MS901M_FRAME_ID_REG_ACCFSR, &g_atk_ms901m_fsr.accelerometer,
                            (uint8_t)sizeof(g_atk_ms901m_fsr.accelerometer), 200);
    if (ret == 0)
    {
        return ATK_MS901M_ERROR;
    }
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M姿态角数据
 * @param       emDevNum: UART设备号
 * @param       attitude_dat: 姿态角数据结构体
 * @param       timeout     : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M姿态角数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M姿态角数据失败
 */
uint8_t atk_ms901m_get_attitude(uart_id_t emDevNum, atk_ms901m_attitude_data_t *attitude_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    
    if (attitude_dat == NULL)
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_ATTITUDE, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    attitude_dat->roll = (float)((int16_t)(frame.dat[1] << 8) | frame.dat[0]) / 32768 * 180;
    attitude_dat->pitch = (float)((int16_t)(frame.dat[3] << 8) | frame.dat[2]) / 32768 * 180;
    attitude_dat->yaw = (float)((int16_t)(frame.dat[5] << 8) | frame.dat[4]) / 32768 * 180;
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M四元数数据
 * @param       emDevNum: UART设备号
 * @param       quaternion_dat: 四元数数据结构体
 * @param       timeout       : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M四元数数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M四元数数据失败
 */
uint8_t atk_ms901m_get_quaternion(uart_id_t emDevNum, atk_ms901m_quaternion_data_t *quaternion_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    
    if (quaternion_dat == NULL)
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_QUAT, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    quaternion_dat->q0 = (float)((int16_t)(frame.dat[1] << 8) | frame.dat[0]) / 32768;
    quaternion_dat->q1 = (float)((int16_t)(frame.dat[3] << 8) | frame.dat[2]) / 32768;
    quaternion_dat->q2 = (float)((int16_t)(frame.dat[5] << 8) | frame.dat[4]) / 32768;
    quaternion_dat->q3 = (float)((int16_t)(frame.dat[7] << 8) | frame.dat[6]) / 32768;
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M陀螺仪、加速度计数据
 * @param       emDevNum: UART设备号
 * @param       gyro_dat         : 陀螺仪数据结构体
 * @param       accelerometer_dat: 加速度计数据结构体
 * @param       timeout          : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M陀螺仪、加速度计数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M陀螺仪、加速度计数据失败
 */
uint8_t atk_ms901m_get_gyro_accelerometer(uart_id_t emDevNum, atk_ms901m_gyro_data_t *gyro_dat, atk_ms901m_accelerometer_data_t *accelerometer_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    
    if ((gyro_dat == NULL) && (accelerometer_dat == NULL))
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_GYRO_ACCE, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    if (gyro_dat != NULL)
    {
        gyro_dat->raw.x = (int16_t)(frame.dat[7] << 8) | frame.dat[6];
        gyro_dat->raw.y = (int16_t)(frame.dat[9] << 8) | frame.dat[8];
        gyro_dat->raw.z = (int16_t)(frame.dat[11] << 8) | frame.dat[10];
        
        gyro_dat->x = (float)gyro_dat->raw.x / 32768 * g_atk_ms901m_gyro_fsr_table[g_atk_ms901m_fsr.gyro];
        gyro_dat->y = (float)gyro_dat->raw.y / 32768 * g_atk_ms901m_gyro_fsr_table[g_atk_ms901m_fsr.gyro];
        gyro_dat->z = (float)gyro_dat->raw.z / 32768 * g_atk_ms901m_gyro_fsr_table[g_atk_ms901m_fsr.gyro];
    }
    
    if (accelerometer_dat != NULL)
    {
        accelerometer_dat->raw.x = (int16_t)(frame.dat[1] << 8) | frame.dat[0];
        accelerometer_dat->raw.y = (int16_t)(frame.dat[3] << 8) | frame.dat[2];
        accelerometer_dat->raw.z = (int16_t)(frame.dat[5] << 8) | frame.dat[4];
        
        accelerometer_dat->x = (float)accelerometer_dat->raw.x / 32768 * g_atk_ms901m_accelerometer_fsr_table[g_atk_ms901m_fsr.accelerometer];
        accelerometer_dat->y = (float)accelerometer_dat->raw.y / 32768 * g_atk_ms901m_accelerometer_fsr_table[g_atk_ms901m_fsr.accelerometer];
        accelerometer_dat->z = (float)accelerometer_dat->raw.z / 32768 * g_atk_ms901m_accelerometer_fsr_table[g_atk_ms901m_fsr.accelerometer];
    }
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M磁力计数据
 * @param       emDevNum: UART设备号
 * @param       magnetometer_dat: 磁力计数据结构体
 * @param       timeout         : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M磁力计数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M磁力计数据失败
 */
uint8_t atk_ms901m_get_magnetometer(uart_id_t emDevNum, atk_ms901m_magnetometer_data_t *magnetometer_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    
    if (magnetometer_dat == NULL)
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_MAG, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    magnetometer_dat->x = (int16_t)(frame.dat[1] << 8) | frame.dat[0];
    magnetometer_dat->y = (int16_t)(frame.dat[3] << 8) | frame.dat[2];
    magnetometer_dat->z = (int16_t)(frame.dat[5] << 8) | frame.dat[4];
    magnetometer_dat->temperature = (float)((int16_t)(frame.dat[7] << 8) | frame.dat[6]) / 100;
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M气压计数据
 * @param       emDevNum: UART设备号
 * @param       barometer_dat: 气压计数据结构体
 * @param       timeout      : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M气压计数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M气压计数据失败
 */
uint8_t atk_ms901m_get_barometer(uart_id_t emDevNum, atk_ms901m_barometer_data_t *barometer_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    
    if (barometer_dat == NULL)
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_BARO, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    barometer_dat->pressure = (int32_t)(frame.dat[3] << 24) | (frame.dat[2] << 16) | (frame.dat[1] << 8) | frame.dat[0];
    barometer_dat->altitude = (int32_t)(frame.dat[7] << 24) | (frame.dat[6] << 16) | (frame.dat[5] << 8) | frame.dat[4];
    barometer_dat->temperature = (float)((int16_t)(frame.dat[9] << 8) | frame.dat[8]) / 100;
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M端口数据
 * @param       emDevNum: UART设备号
 * @param       port_dat: 端口数据结构体
 * @param       timeout : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M端口数据成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M端口数据失败
 */
uint8_t atk_ms901m_get_port(uart_id_t emDevNum, atk_ms901m_port_data_t *port_dat, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_frame_t frame = {0};
    
    if (port_dat == NULL)
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_get_frame_by_id(emDevNum, &frame, ATK_MS901M_FRAME_ID_PORT, ATK_MS901M_FRAME_ID_TYPE_UPLOAD, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    port_dat->d0 = (uint16_t)(frame.dat[1] << 8) | frame.dat[0];
    port_dat->d1 = (uint16_t)(frame.dat[3] << 8) | frame.dat[2];
    port_dat->d2 = (uint16_t)(frame.dat[5] << 8) | frame.dat[4];
    port_dat->d3 = (uint16_t)(frame.dat[7] << 8) | frame.dat[6];
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M LED灯状态
 * @param       emDevNum: UART设备号
 * @param       state   : LED灯状态
 * @param       timeout : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M LED灯状态成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M LED灯状态失败
 */
uint8_t atk_ms901m_get_led_state(uart_id_t emDevNum, atk_ms901m_led_state_t *state, uint32_t timeout)
{
    uint8_t ret;
    
    ret = atk_ms901m_read_reg_by_id(emDevNum, ATK_MS901M_FRAME_ID_REG_LEDOFF, (uint8_t *)state,
                            (uint8_t)sizeof(*state), timeout);
    if (ret == 0)
    {
        return ATK_MS901M_ERROR;
    }
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       设置ATK-MS901M LED灯状态
 * @param       emDevNum: UART设备号
 * @param       state   : LED灯状态
 * @param       timeout : 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 设置ATK-MS901M LED灯状态成功
 *              ATK_MS901M_ERROR: 设置ATK-MS901M LED灯状态失败
 */
uint8_t atk_ms901m_set_led_state(uart_id_t emDevNum, atk_ms901m_led_state_t state, uint32_t timeout)
{
    uint8_t ret;
    atk_ms901m_led_state_t state_recv;
    
    ret = atk_ms901m_write_reg_by_id(emDevNum, ATK_MS901M_FRAME_ID_REG_LEDOFF, 1, (uint8_t *)&state);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_get_led_state(emDevNum, &state_recv, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    if (state_recv != state)
    {
        return ATK_MS901M_ERROR;
    }
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M指定端口模式
 * @param       emDevNum: UART设备号
 * @param       port   : 指定端口
 * @param       mode   : 端口的模式
 * @param       timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M指定端口模式成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M指定端口模式失败
 */
uint8_t atk_ms901m_get_port_mode(uart_id_t emDevNum, atk_ms901m_port_t port, atk_ms901m_port_mode_t *mode, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    
    if (port == ATK_MS901M_PORT_D0)
    {
        id = ATK_MS901M_FRAME_ID_REG_D0MODE;
    }
    else if (port == ATK_MS901M_PORT_D1)
    {
        id = ATK_MS901M_FRAME_ID_REG_D1MODE;
    }
    else if (port == ATK_MS901M_PORT_D2)
    {
        id = ATK_MS901M_FRAME_ID_REG_D2MODE;
    }
    else if (port == ATK_MS901M_PORT_D3)
    {
        id = ATK_MS901M_FRAME_ID_REG_D3MODE;
    }
    else
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_read_reg_by_id(emDevNum, id, (uint8_t *)mode,
                            (uint8_t)sizeof(*mode), timeout);
    if (ret == 0)
    {
        return ATK_MS901M_ERROR;
    }
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       设置ATK-MS901M指定端口模式
 * @param       emDevNum: UART设备号
 * @param       port   : 指定端口
 * @param       mode   : 端口的模式
 * @param       timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 设置ATK-MS901M指定端口模式成功
 *              ATK_MS901M_ERROR: 设置ATK-MS901M指定端口模式失败
 */
uint8_t atk_ms901m_set_port_mode(uart_id_t emDevNum, atk_ms901m_port_t port, atk_ms901m_port_mode_t mode, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    atk_ms901m_port_mode_t mode_recv;
    
    if (port == ATK_MS901M_PORT_D0)
    {
        if (mode == ATK_MS901M_PORT_MODE_OUTPUT_PWM)
        {
            return ATK_MS901M_ERROR;
        }
        id = ATK_MS901M_FRAME_ID_REG_D0MODE;
    }
    else if (port == ATK_MS901M_PORT_D1)
    {
        id = ATK_MS901M_FRAME_ID_REG_D1MODE;
    }
    else if (port == ATK_MS901M_PORT_D2)
    {
        if (mode == ATK_MS901M_PORT_MODE_OUTPUT_PWM)
        {
            return ATK_MS901M_ERROR;
        }
        id = ATK_MS901M_FRAME_ID_REG_D2MODE;
    }
    else if (port == ATK_MS901M_PORT_D3)
    {
        id = ATK_MS901M_FRAME_ID_REG_D3MODE;
    }
    else
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_write_reg_by_id(emDevNum, id, 1, (uint8_t *)&mode);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_get_port_mode(emDevNum, port, &mode_recv, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    else
    {
        if (mode_recv != mode)
        {
            return ATK_MS901M_ERROR;
        }
    }
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M指定端口PWM高电平的宽度
 * @param       emDevNum: UART设备号
 * @param       port   : 指定端口
 * @param       pulse  : 端口PWM高电平的宽度，单位：纳秒（ns）
 * @param       timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M指定端口PWM高电平的宽度成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M指定端口PWM高电平的宽度失败
 */
uint8_t atk_ms901m_get_port_pwm_pulse(uart_id_t emDevNum, atk_ms901m_port_t port, uint16_t *pulse, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    
    if (port == ATK_MS901M_PORT_D0)
    {
        return ATK_MS901M_ERROR;
    }
    else if (port == ATK_MS901M_PORT_D1)
    {
        id = ATK_MS901M_FRAME_ID_REG_D1PULSE;
    }
    else if (port == ATK_MS901M_PORT_D2)
    {
        return ATK_MS901M_ERROR;
    }
    else if (port == ATK_MS901M_PORT_D3)
    {
        id = ATK_MS901M_FRAME_ID_REG_D3PULSE;
    }
    else
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_read_reg_by_id(emDevNum, id, (uint8_t *)pulse,
                            (uint8_t)sizeof(*pulse), 100);
    if (ret == 0)
    {
        return ATK_MS901M_ERROR;
    }
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       设置ATK-MS901M指定端口PWM高电平的宽度
 * @param       emDevNum: UART设备号
 * @param       port   : 指定端口
 * @param       pulse  : 端口PWM高电平的宽度，单位：纳秒（ns）
 * @param       timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 设置ATK-MS901M指定端口PWM高电平的宽度成功
 *              ATK_MS901M_ERROR: 设置ATK-MS901M指定端口PWM高电平的宽度失败
 */
uint8_t atk_ms901m_set_port_pwm_pulse(uart_id_t emDevNum, atk_ms901m_port_t port, uint16_t pulse, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    uint16_t pulse_recv;
    
    if (port == ATK_MS901M_PORT_D0)
    {
        return ATK_MS901M_ERROR;
    }
    else if (port == ATK_MS901M_PORT_D1)
    {
        id = ATK_MS901M_FRAME_ID_REG_D1PULSE;
    }
    else if (port == ATK_MS901M_PORT_D2)
    {
        return ATK_MS901M_ERROR;
    }
    else if (port == ATK_MS901M_PORT_D3)
    {
        id = ATK_MS901M_FRAME_ID_REG_D3PULSE;
    }
    else
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_write_reg_by_id(emDevNum, id, 2, (uint8_t *)&pulse);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_get_port_pwm_pulse(emDevNum, port, &pulse_recv, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    if (pulse_recv != pulse)
    {
        return ATK_MS901M_ERROR;
    }
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       获取ATK-MS901M指定端口PWM周期
 * @param       emDevNum: UART设备号
 * @param       port   : 指定端口
 * @param       period : 端口PWM周期，单位：纳秒（ns）
 * @param       timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 获取ATK-MS901M指定端口PWM周期成功
 *              ATK_MS901M_ERROR: 获取ATK-MS901M指定端口PWM周期失败
 */
uint8_t atk_ms901m_get_port_pwm_period(uart_id_t emDevNum, atk_ms901m_port_t port, uint16_t *period, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    
    if (port == ATK_MS901M_PORT_D0)
    {
        return ATK_MS901M_ERROR;
    }
    else if (port == ATK_MS901M_PORT_D1)
    {
        id = ATK_MS901M_FRAME_ID_REG_D1PERIOD;
    }
    else if (port == ATK_MS901M_PORT_D2)
    {
        return ATK_MS901M_ERROR;
    }
    else if (port == ATK_MS901M_PORT_D3)
    {
        id = ATK_MS901M_FRAME_ID_REG_D3PERIOD;
    }
    else
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_read_reg_by_id(emDevNum, id, (uint8_t *)period,
                            (uint8_t)sizeof(*period), timeout);
    if (ret == 0)
    {
        return ATK_MS901M_ERROR;
    }
    
    return ATK_MS901M_EOK;
}

/**
 * @brief       设置ATK-MS901M指定端口PWM周期
 * @param       emDevNum: UART设备号
 * @param       port   : 指定端口
 * @param       period : 端口PWM周期，单位：纳秒（ns）
 * @param       timeout: 获取数据最大等待超时时间，单位：毫秒（ms）
 * @retval      ATK_MS901M_EOK  : 设置ATK-MS901M指定端口PWM周期成功
 *              ATK_MS901M_ERROR: 设置ATK-MS901M指定端口PWM周期失败
 */
uint8_t atk_ms901m_set_port_pwm_period(uart_id_t emDevNum, atk_ms901m_port_t port, uint16_t period, uint32_t timeout)
{
    uint8_t ret;
    uint8_t id;
    uint16_t period_recv;
    
    if (port == ATK_MS901M_PORT_D0)
    {
        return ATK_MS901M_ERROR;
    }
    else if (port == ATK_MS901M_PORT_D1)
    {
        id = ATK_MS901M_FRAME_ID_REG_D1PERIOD;
    }
    else if (port == ATK_MS901M_PORT_D2)
    {
        return ATK_MS901M_ERROR;
    }
    else if (port == ATK_MS901M_PORT_D3)
    {
        id = ATK_MS901M_FRAME_ID_REG_D3PERIOD;
    }
    else
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_write_reg_by_id(emDevNum, id, 2, (uint8_t *)&period);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    ret = atk_ms901m_get_port_pwm_period(emDevNum, port, &period_recv, timeout);
    if (ret != ATK_MS901M_EOK)
    {
        return ATK_MS901M_ERROR;
    }
    
    if (period_recv != period)
    {
        return ATK_MS901M_ERROR;
    }
    
    return ATK_MS901M_EOK;
}

#endif
