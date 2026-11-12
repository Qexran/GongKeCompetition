/**
  * @file       app_transfer.c
  * @brief      ⚠️ **DEPRECATED（2026-10 审查标记, 勿再接线）** —— pro 项目遗留死代码:
  *             vTransferPosWuKuai/vTransferPosSeHuan 全工程无调用点, 且其坐标语义
  *             （「9 字节 ASCII last packet 的 X/Y」）与现行 RPI 帧协议（TAG=0x03/0x04
  *             整帧快照 + 各对位函数的 PX_TO_MM 标定）完全不兼容。需要像素→毫米时
  *             走现行链路, 别复活本文件。
  *
  *             视觉像素坐标 → 毫米 坐标变换实现（移植自 pro 项目 Function/Function.c）。
  *
  *             适配点：pro 的全局 g_aucSerialRxPacket[9] → Beam 的 pucGetLastRpiPacket()。
  *             串口数据格式(9 字节): [0..3]=X(ASCII) [4..7]=Y(ASCII) [8]=颜色/状态。
  */

#include "app_transfer.h"
#include "app_beam.h"

/*
 * 按目标颜色与识别颜色做横向偏移：三物块横向排列, 间距 fGetDistanceWukuai。
 * 偏移系数表 [目标][识别]（行/列顺序均为 红/绿/蓝），单位 = 物块间距，
 * 0 = 识别到目标色不偏移。未识别(>=4)/流式(0) 由调用方处理，不在表内。
 */
static const int8_t s_cOffsetCoef[3][3] = {
    /* 识别:  红   绿   蓝  */
    /* 目标绿 */ { +1,  0, -1 },
    /* 目标红 */ {  0, -1, -2 },
    /* 目标蓝 */ { +2, +1,  0 },
};

/* 查表计算颜色偏移并叠加到 DirectX（目标或识别色非法时不偏移） */
static void vApplyColorOffsetX(color_t emTarget, uint8_t ucDetected)
{
    int iTarget = (int)emTarget - 1;      /* COLOR_RED=1 → 下标 0 */
    int iDetect = (int)ucDetected - 1;

    if (iTarget < 0 || iTarget > 2) return;
    if (iDetect < 0 || iDetect > 2) return;

    float fOffset = (float)s_cOffsetCoef[iTarget][iDetect] * fGetDistanceWukuai();
    if (fOffset != 0.0f) {
        vSetDirectX(fGetDirectX() + fOffset);
    }
}

/* 合法性校验: 前 8 字节为 ASCII 数字; ucWithXor=1 时还要求低 4 位 XOR 校验和匹配 */
static uint8_t ucCheckRpiPacket(const uint8_t *pucPkt, uint8_t ucWithXor)
{
    uint8_t ucI;

    for (ucI = 0; ucI < 8; ucI++) {
        if (pucPkt[ucI] < '0' || pucPkt[ucI] > '9') { return 0; }
    }

    if (ucWithXor) {
        uint8_t ucXor = 0;
        for (ucI = 0; ucI < 8; ucI++) { ucXor ^= pucPkt[ucI]; }
        if ((ucXor & 0x0F) != (pucPkt[8] & 0x0F)) { return 0; }
    }
    return 1;
}

/* 解析 X/Y 像素坐标（各 4 位 ASCII 拼接）写入 Beam 像素原点 */
static void vParseRpiPixel(const uint8_t *pucPkt)
{
    vSetOriginX((pucPkt[0] - 0x30) * 1000 + (pucPkt[1] - 0x30) * 100
              + (pucPkt[2] - 0x30) * 10    + (pucPkt[3] - 0x30));
    vSetOriginY((pucPkt[4] - 0x30) * 1000 + (pucPkt[5] - 0x30) * 100
              + (pucPkt[6] - 0x30) * 10    + (pucPkt[7] - 0x30));
}

/*
 * 物块位置转换（转盘 TASK_TURNTABLE / 码垛 TASK_MA_DUO）
 * 校验 → 解析像素坐标 → 按物块高度算比例系数 → 减原点偏移 → 乘比例得毫米 → 颜色间距修正。
 */
void vTransferPosWuKuai(task_type_t task_choose, float distance, color_t color)
{
    uint8_t *pucPkt = pucGetLastRpiPacket();

    /* 1. 合法性校验（先校验后解析，坏包不污染 Beam） */
    if (!ucCheckRpiPacket(pucPkt, 1)) {
        vSetDirectX(0);
        vSetDirectY(0);
        return;
    }

    /* 2. 解析 X/Y 像素坐标 */
    vParseRpiPixel(pucPkt);

    /* 提取颜色码 (高 4 位, 1=红 2=绿 3=蓝 4=无) */
    uint8_t detected_color = ucGetDetectedColor();

    float bilichi = 0;  /* 像素→毫米比例系数 */

    /* 3. 按任务类型算比例系数和原点偏移 */
    if (task_choose == TASK_TURNTABLE) {
        bilichi = -0.0025f * (distance + 160.0f) + 1.1967f;
        vSetOriginX(fGetOriginX() - 312);
        vSetOriginY(fGetOriginY() - 155);
    } else if (task_choose == TASK_MA_DUO) {
        bilichi = -0.0023f * (distance) + 1.1655f;
        vSetOriginX(fGetOriginX() - 314);
        vSetOriginY(fGetOriginY() - 238);
    }

    /* 4. 像素坐标 → 毫米 */
    vSetDirectX(fGetOriginX() * bilichi);
    vSetDirectY(fGetOriginY() * bilichi);

    /* 5. 按颜色做物块间间距偏移 */
    vApplyColorOffsetX(color, detected_color);

    /* 6. 未看到物块时微调，让小车微动后重试 */
    if (detected_color >= 4) {
        vSetDirectY(5);
        vSetDirectX(-5);
    }
}

/*
 * 色环位置转换（粗定标 RESOLUTION_COARSE / 精定标 RESOLUTION_FINE）
 * 粗定位用通用中心对准任意色环再偏移到绿色锚点，精定位用颜色精确中心对准。
 */
void vTransferPosSeHuan(resolution_t fenbian_choose, color_t task_num)
{
    uint8_t *pucPkt = pucGetLastRpiPacket();

    /* 合法性校验（流式模式下跳过 XOR 校验，byte[8] 承载 y_diff 数据） */
    if (!ucCheckRpiPacket(pucPkt, ucGetSehuanStreaming() ? 0 : 1)) {
        vSetDirectX(0);
        vSetDirectY(0);
        return;
    }

    /* 解析 X/Y 像素坐标 */
    vParseRpiPixel(pucPkt);

    /* 颜色码（流式模式下 byte[8] 是原始 y_diff） */
    uint8_t detected_color = ucGetSehuanStreaming() ? 0 : ucGetDetectedColor();

    if (fenbian_choose == RESOLUTION_COARSE) {
        float x_center = 296;
        float y_center = 238;
        vSetOriginX(fGetOriginX() - x_center);
        vSetOriginY(fGetOriginY() - y_center);
        vSetDirectX(fGetOriginX() * 1.1943f);
        vSetDirectY(fGetOriginY() * 1.1943f);

    } else if (fenbian_choose == RESOLUTION_FINE) {
        float x_center = 940;
        float y_center = 558;
        vSetOriginX(fGetOriginX() - x_center);
        vSetOriginY(fGetOriginY() - y_center);
        vSetDirectX(fGetOriginX() * fGetVisualServoPxToMmX());
        vSetDirectY(fGetOriginY() * fGetVisualServoPxToMmY());

        /* 流式细定标: byte[8] 是 int8 y_diff，非颜色码 */
        if (ucGetSehuanStreaming()) {
            vSetSehuanYDiff((int8_t)pucPkt[8]);
            return;  /* 跳过颜色偏移逻辑 */
        }
    }

    /* 按目标颜色偏移（三个色环横向排列，间距 distance_wukuai） */
    vApplyColorOffsetX(task_num, detected_color);

    /* 未识别到物块时小幅度微调 */
    if (detected_color >= 4) {
        vSetDirectY(3);
        vSetDirectX(-3);
    }
}
