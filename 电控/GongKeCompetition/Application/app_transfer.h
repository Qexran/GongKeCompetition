/**
  * @file       app_transfer.h
  * @brief      视觉像素坐标 → 毫米 坐标变换模块（移植自 pro 项目 Function/Function.c 的纯函数）。
  *
  *             输入为 Beam 缓存的"最近一帧树莓派 9 字节包" pucGetLastRpiPacket()，
  *             输出写入 Beam 的 DirectX/DirectY(mm)。串口协议(握手/收包/校验)不在此处。
  */

#ifndef __APP_TRANSFER_H
#define __APP_TRANSFER_H

#include "app_motion_types.h"

/* 物块位置转换：解析像素坐标 → 毫米偏移(写入 DirectX/Y)，并按颜色做物块间距修正 */
void vTransferPosWuKuai(task_type_t task_choose, float distance, color_t color);

/* 色环位置转换：粗/精两阶段分辨率，解析像素坐标 → 毫米偏移(写入 DirectX/Y) */
void vTransferPosSeHuan(resolution_t fenbian_choose, color_t task_num);

#endif /* __APP_TRANSFER_H */
