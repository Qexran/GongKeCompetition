/**
  * @file       emm_v5_device.c
  * @author     Qe_xr
  * @version    V1.0.0
  * @date       2026/1/24
  * @brief      Emm_V5步进闭环电机驱动，基于 STM32 HAL 库
  *
  */
#include "emm_v5_device.h"
#include <string.h>
#include "FreeRTOS.h"
#include "semphr.h"

#if EMM_V5_IS_ENABLE

static stEmmV5DeviceParamTdf g_stEmmV5Device;

/* EMM 总线互斥：脚本任务(位置指令)与 PID 任务(50ms 速度指令)并发发送时,
 * 保证帧完整性 —— HAL_UART_Transmit 非重入, 并发调用会返回 HAL_BUSY 静默丢帧 */
static SemaphoreHandle_t s_xEmmBusMutex = NULL;

/**
 * @brief  CRC-8 查找表（电机菜单 Checksum 选 CRC-8 时使用）
 * @note   算法：初值取首字节，逐字节 crc = table[crc ^ byte]，与 EMM V5 上位机一致
 */
static const uint8_t s_aucCrc8Table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35
};

/**
 * @brief  计算 CRC-8 校验值（初值取首字节）
 * @param  pucData ：数据首地址（含地址字节）
 * @param  ucLen   ：数据长度（不含校验字节）
 * @return CRC-8 校验值
 * @note   通用 CRC-8（查表）, app_rpi 帧协议校验复用此函数
 */
uint8_t ucEmmV5Crc8(const uint8_t *pucData, uint8_t ucLen)
{
    uint8_t ucCrc = pucData[0];
    uint8_t ucIdx;

    for (ucIdx = 1; ucIdx < ucLen; ucIdx++)
    {
        ucCrc = s_aucCrc8Table[ucCrc ^ pucData[ucIdx]];
    }
    return ucCrc;
}

/**
 * @brief  Emm_V5 命令统一发送出口（CRC-8 校验模式）
 * @param  pucCmd ：命令帧缓冲区，末字节为 0x6B 占位
 * @param  usLen  ：帧总长度（含占位校验字节）
 * @note   电机菜单 Checksum 配置为 CRC-8 时，每条命令最后一个字节必须为
 *         前 N-1 字节的 CRC-8 计算值（0x6B 会被电机判为校验错误而丢弃）。
 *         各命令函数装载的 0x6B 仅为占位，此函数统一替换后发送。
 */
static void vEmmV5SendCmd(uint8_t *pucCmd, uint16_t usLen)
{
    if (usLen < 2)
    {
        return;
    }

    pucCmd[usLen - 1] = ucEmmV5Crc8(pucCmd, (uint8_t)(usLen - 1));

    /* 互斥发送（vEmmV5DeviceInit 已创建；判空防御初始化前误调用） */
    if (s_xEmmBusMutex != NULL) { xSemaphoreTake(s_xEmmBusMutex, portMAX_DELAY); }
    vUartSendArray(g_stEmmV5Device.stStaticParam.emUartDevNum, pucCmd, usLen);
    if (s_xEmmBusMutex != NULL) { xSemaphoreGive(s_xEmmBusMutex); }
}

/**
 * @brief  Emm_V5初始化
 * @param  pstInit ：静态参数指针
 */
void vEmmV5DeviceInit(stEmmV5StaticParamTdf *pstInit)
{
    if (s_xEmmBusMutex == NULL)
    {
        s_xEmmBusMutex = xSemaphoreCreateMutex();
        configASSERT(s_xEmmBusMutex != NULL);
    }

    g_stEmmV5Device.stStaticParam = *pstInit;
    g_stEmmV5Device.stRunningParam.ucAddr = 0;
    g_stEmmV5Device.stRunningParam.usRxFrameLen = 0;
    g_stEmmV5Device.stRunningParam.ucRxFrameComplete = 0;
    memset(g_stEmmV5Device.stRunningParam.aucRxFrameBuf, 0, sizeof(g_stEmmV5Device.stRunningParam.aucRxFrameBuf));
}

/**
 * @brief  Emm_V5周期执行
 */
void vEmmV5DevicePeriodExecute(void)
{
    uint16_t usLen;

    /* 非阻塞取一帧（DMA + IDLE + 二值信号量）: 直接拷进收帧缓冲,
     * 圆环回绕/起始偏移由 usUartDmaTakeCopy 吸收, 所以 [0] 必是这一帧的首字节
     * （回调按「[0]=电机地址」用它, 错位会静默认错电机）。超容量时驱动返回 0 丢弃。 */
    usLen = usUartDmaTakeCopy(g_stEmmV5Device.stStaticParam.emUartDevNum,
                              g_stEmmV5Device.stRunningParam.aucRxFrameBuf,
                              (uint16_t)sizeof(g_stEmmV5Device.stRunningParam.aucRxFrameBuf),
                              0);
    if (usLen == 0)
    {
        return;
    }

    g_stEmmV5Device.stRunningParam.usRxFrameLen = usLen;

    if (g_stEmmV5Device.stStaticParam.vCallbackFcn != NULL)
    {
        g_stEmmV5Device.stStaticParam.vCallbackFcn(
            g_stEmmV5Device.stRunningParam.aucRxFrameBuf[0],
            g_stEmmV5Device.stRunningParam.aucRxFrameBuf,
            g_stEmmV5Device.stRunningParam.usRxFrameLen
        );
    }
}

/**
 * @brief  将当前位置清零
 * @param  ucAddr ：电机地址
 */
void vEmmV5ResetCurPosToZero(uint8_t ucAddr)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x0A;
    aucCmd[2] = 0x6D;
    aucCmd[3] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 4);
}

/**
 * @brief  解除堵转保护
 * @param  ucAddr ：电机地址
 */
void vEmmV5ResetClogPro(uint8_t ucAddr)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x0E;
    aucCmd[2] = 0x52;
    aucCmd[3] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 4);
}

/**
 * @brief  读取系统参数
 * @param  ucAddr ：电机地址
 * @param  emSysParam ：系统参数类型
 */
void vEmmV5ReadSysParams(uint8_t ucAddr, emEmmV5SysParamTdf emSysParam)
{
    uint8_t aucCmd[16] = {0};
    uint8_t ucIndex = 0;
    
    aucCmd[ucIndex++] = ucAddr;
    
    switch(emSysParam)
    {
        case emEmmV5SysParam_Ver   : aucCmd[ucIndex++] = 0x1F; break;
        case emEmmV5SysParam_RL    : aucCmd[ucIndex++] = 0x20; break;
        case emEmmV5SysParam_PID   : aucCmd[ucIndex++] = 0x21; break;
        case emEmmV5SysParam_VBus  : aucCmd[ucIndex++] = 0x24; break;
        case emEmmV5SysParam_Cpha  : aucCmd[ucIndex++] = 0x27; break;
        case emEmmV5SysParam_Encl  : aucCmd[ucIndex++] = 0x31; break;
        case emEmmV5SysParam_TPos  : aucCmd[ucIndex++] = 0x33; break;
        case emEmmV5SysParam_Vel   : aucCmd[ucIndex++] = 0x35; break;
        case emEmmV5SysParam_CPos  : aucCmd[ucIndex++] = 0x36; break;
        case emEmmV5SysParam_PErr  : aucCmd[ucIndex++] = 0x37; break;
        case emEmmV5SysParam_Flag  : aucCmd[ucIndex++] = 0x3A; break;
        case emEmmV5SysParam_Org   : aucCmd[ucIndex++] = 0x3B; break;
        case emEmmV5SysParam_Conf  : aucCmd[ucIndex++] = 0x42; aucCmd[ucIndex++] = 0x6C; break;
        case emEmmV5SysParam_State : aucCmd[ucIndex++] = 0x43; aucCmd[ucIndex++] = 0x7A; break;
        default: break;
    }
    
    aucCmd[ucIndex++] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, ucIndex);
}

/**
 * @brief  修改开环/闭环控制模式
 * @param  ucAddr ：电机地址
 * @param  bSave ：存储标志
 * @param  emCtrlMode ：控制模式
 */
void vEmmV5ModifyCtrlMode(uint8_t ucAddr, uint8_t bSave, emEmmV5CtrlModeTdf emCtrlMode)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x46;
    aucCmd[2] = 0x69;
    aucCmd[3] = bSave;
    aucCmd[4] = emCtrlMode;
    aucCmd[5] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 6);
}

/**
 * @brief  电机使能控制
 * @param  ucAddr ：电机地址
 * @param  bEnable ：使能状态，true为使能电机，false为关闭电机
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmV5EnControl(uint8_t ucAddr, uint8_t bEnable, uint8_t bSyncFlag)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0xF3;
    aucCmd[2] = 0xAB;
    aucCmd[3] = bEnable;
    aucCmd[4] = bSyncFlag;
    aucCmd[5] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 6);
}

/**
 * @brief  速度模式控制
 * @param  ucAddr ：电机地址
 * @param  emDir ：方向
 * @param  usVel ：速度，范围0 - 5000RPM
 * @param  ucAcc ：加速度，范围0 - 255，注意：0是直接启动
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmV5VelControl(uint8_t ucAddr, emEmmV5DirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint8_t bSyncFlag)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0xF6;
    aucCmd[2] = emDir;
    aucCmd[3] = (uint8_t)(usVel >> 8);
    aucCmd[4] = (uint8_t)(usVel >> 0);
    aucCmd[5] = ucAcc;
    aucCmd[6] = bSyncFlag;
    aucCmd[7] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 8);
}

/**
 * @brief  位置模式控制
 * @param  ucAddr ：电机地址
 * @param  emDir ：方向
 * @param  usVel ：速度(RPM)，范围0 - 5000RPM
 * @param  ucAcc ：加速度，范围0 - 255，注意：0是直接启动
 * @param  ulClk ：脉冲数，范围0- (2^32 - 1)个
 * @param  bAbsFlag ：相位/绝对标志，false为相对运动，true为绝对值运动
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmV5PosControl(uint8_t ucAddr, emEmmV5DirTdf emDir, uint16_t usVel, uint8_t ucAcc, uint32_t ulClk, uint8_t bAbsFlag, uint8_t bSyncFlag)
{
    uint8_t aucCmd[32] = {0};
    
    aucCmd[0]  = ucAddr;
    aucCmd[1]  = 0xFD;
    aucCmd[2]  = emDir;
    aucCmd[3]  = (uint8_t)(usVel >> 8);
    aucCmd[4]  = (uint8_t)(usVel >> 0);
    aucCmd[5]  = ucAcc;
    aucCmd[6]  = (uint8_t)(ulClk >> 24);
    aucCmd[7]  = (uint8_t)(ulClk >> 16);
    aucCmd[8]  = (uint8_t)(ulClk >> 8);
    aucCmd[9]  = (uint8_t)(ulClk >> 0);
    aucCmd[10] = bAbsFlag;
    aucCmd[11] = bSyncFlag;
    aucCmd[12] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 13);
}

/**
 * @brief  让电机立即停止运动
 * @param  ucAddr ：电机地址
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmV5StopNow(uint8_t ucAddr, uint8_t bSyncFlag)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0xFE;
    aucCmd[2] = 0x98;
    aucCmd[3] = bSyncFlag;
    aucCmd[4] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 5);
}

/**
 * @brief  触发多机同步开始运动
 * @param  ucAddr ：电机地址
 */
void vEmmV5SynchronousMotion(uint8_t ucAddr)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0xFF;
    aucCmd[2] = 0x66;
    aucCmd[3] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 4);
}

/**
 * @brief  设置单圈回零的零点位置
 * @param  ucAddr ：电机地址
 * @param  bSave ：是否存储标志，false为不存储，true为存储
 */
void vEmmV5OriginSetO(uint8_t ucAddr, uint8_t bSave)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x93;
    aucCmd[2] = 0x88;
    aucCmd[3] = bSave;
    aucCmd[4] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 5);
}

/**
 * @brief  修改回零参数
 * @param  ucAddr ：电机地址
 * @param  bSave ：是否存储标志，false为不存储，true为存储
 * @param  emOrgMode ：回零模式
 * @param  emDir ：回零方向
 * @param  usOrgVel ：回零速度，单位：RPM（转/分钟）
 * @param  ulOrgTm ：回零超时时间，单位：毫秒
 * @param  usSlVel ：无限位碰撞回零检测转速，单位：RPM（转/分钟）
 * @param  usSlMa ：无限位碰撞回零检测电流，单位：Ma（毫安）
 * @param  usSlMs ：无限位碰撞回零检测时间，单位：Ms（毫秒）
 * @param  bPotFlag ：上电自动触发回零，false为不使能，true为使能
 */
void vEmmV5OriginModifyParams(uint8_t ucAddr, uint8_t bSave, emEmmV5OrgModeTdf emOrgMode, emEmmV5DirTdf emDir, 
                               uint16_t usOrgVel, uint32_t ulOrgTm, uint16_t usSlVel, uint16_t usSlMa, 
                               uint16_t usSlMs, uint8_t bPotFlag)
{
    uint8_t aucCmd[32] = {0};
    
    aucCmd[0]  = ucAddr;
    aucCmd[1]  = 0x4C;
    aucCmd[2]  = 0xAE;
    aucCmd[3]  = bSave;
    aucCmd[4]  = emOrgMode;
    aucCmd[5]  = emDir;
    aucCmd[6]  = (uint8_t)(usOrgVel >> 8);
    aucCmd[7]  = (uint8_t)(usOrgVel >> 0);
    aucCmd[8]  = (uint8_t)(ulOrgTm >> 24);
    aucCmd[9]  = (uint8_t)(ulOrgTm >> 16);
    aucCmd[10] = (uint8_t)(ulOrgTm >> 8);
    aucCmd[11] = (uint8_t)(ulOrgTm >> 0);
    aucCmd[12] = (uint8_t)(usSlVel >> 8);
    aucCmd[13] = (uint8_t)(usSlVel >> 0);
    aucCmd[14] = (uint8_t)(usSlMa >> 8);
    aucCmd[15] = (uint8_t)(usSlMa >> 0);
    aucCmd[16] = (uint8_t)(usSlMs >> 8);
    aucCmd[17] = (uint8_t)(usSlMs >> 0);
    aucCmd[18] = bPotFlag;
    aucCmd[19] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 20);
}

/**
 * @brief  触发回零
 * @param  ucAddr ：电机地址
 * @param  emOrgMode ：回零模式
 * @param  bSyncFlag ：多机同步标志，false为不启用，true为启用
 */
void vEmmV5OriginTriggerReturn(uint8_t ucAddr, emEmmV5OrgModeTdf emOrgMode, uint8_t bSyncFlag)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x9A;
    aucCmd[2] = emOrgMode;
    aucCmd[3] = bSyncFlag;
    aucCmd[4] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 5);
}

/**
 * @brief  强制中断并退出回零
 * @param  ucAddr ：电机地址
 */
void vEmmV5OriginInterrupt(uint8_t ucAddr)
{
    uint8_t aucCmd[16] = {0};
    
    aucCmd[0] = ucAddr;
    aucCmd[1] = 0x9C;
    aucCmd[2] = 0x48;
    aucCmd[3] = 0x6B;
    
    vEmmV5SendCmd(aucCmd, 4);
}

#endif
