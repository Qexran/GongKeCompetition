/**
  * @file       app_cali.h
  * @brief      视觉校准模块（结构移植自 pro Motion.c 的 vSeHuanCalibrate 方法枚举分发）。
  *
  *             子校准动作可任意组合（NONE=跳过），实现目前为空壳，
  *             等树莓派上位机方案确定后逐个填充。
  */

#ifndef __APP_CALI_H
#define __APP_CALI_H

#include <stdint.h>

/* ==================== 校准方法枚举 ==================== */
/* 4~ 预留扩展位（pro 还有 ANGLE_LIU_SHI 流式角度 / ANGLE_XI 单次角度） */
typedef enum {
    CALI_METHOD_NONE     = 0,   /* 空方法, 跳过 */
    CALI_METHOD_CU       = 1,   /* 粗定标: 低分辨率快速靠近 */
    CALI_METHOD_XI       = 2,   /* 单次细定标: 高分辨率单帧精移 */
    CALI_METHOD_LIU_SHI  = 3,   /* 流式细定标: 像素闭环伺服 */
} emCaliMethodTdf;

/* 顺序执行两个校准方法（任意组合, NONE=跳过该段） */
void vCaliRun(emCaliMethodTdf emMethod1, emCaliMethodTdf emMethod2);

#endif /* __APP_CALI_H */
