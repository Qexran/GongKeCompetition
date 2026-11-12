/**
  * @file       app_cali.c
  * @brief      ⚠️ **DEPRECATED（2026-10 审查标记, 勿再接线）** —— pro 项目遗留空壳:
  *             三个子方法均为 TODO 空实现且全工程无有效调用链, 其设计依赖的
  *             app_transfer 坐标链路已被 RPI 帧协议取代（见 app_transfer.c 头）。
  *
  *             视觉校准模块实现（分发结构来自 pro Motion.c vSeHuanCalibrate）。
  *
  *             ⚠️ 三个子方法目前均为空壳（TODO），仅保证调用链与超时/返回
  *             结构就位；具体实现等树莓派上位机方案确定后填充：
  *               CU      —— 发粗定标指令 → 等坐标帧 → vTransferPosSeHuan(COARSE)
  *                          → vChassisMoveCameraDistance 靠近, 循环至识别有效
  *               XI      —— 发细定标指令 → 等坐标帧 → vTransferPosSeHuan(FINE)
  *                          → 小速度精移
  *               LIU_SHI —— 发流式指令 → 像素闭环伺服(PI-D) → 收敛判定 → 退出通知
  */

#include "app_cali.h"
#include "FreeRTOS.h"
#include "task.h"

/* ==================== 子校准动作（空壳） ==================== */

/* 粗定标: 低分辨率快速靠近目标（循环: 发指令→取坐标→按像素移动, 至识别有效） */
static void vCaliCu(void)
{
    /* TODO: 等树莓派方案后实现（参考 pro vSeHuanCalibrateOneTimeCu） */
}

/* 单次细定标: 高分辨率单帧精移（一次: 发指令→取坐标→精移到位） */
static void vCaliXi(void)
{
    /* TODO: 等树莓派方案后实现（参考 pro vSeHuanCalibrateOneTimeXi） */
}

/* 流式细定标: 像素闭环伺服（持续帧流 + PI-D 收敛判定 + 退出通知） */
static void vCaliLiuShi(void)
{
    /* TODO: 等树莓派方案后实现（参考 pro vSeHuanCalibrateLiuShi） */
}

/* ==================== 组合入口 ==================== */

/*
 * 顺序执行两个校准方法, NONE=跳过（可任意组合, 如 CU+XI / NONE+LIU_SHI）。
 * 仅任务上下文调用（子方法将包含阻塞等待）。
 */
void vCaliRun(emCaliMethodTdf emMethod1, emCaliMethodTdf emMethod2)
{
    switch (emMethod1)
    {
    case CALI_METHOD_CU:      vCaliCu();     break;
    case CALI_METHOD_XI:      vCaliXi();     break;
    case CALI_METHOD_LIU_SHI: vCaliLiuShi(); break;
    default:                                  break;   /* NONE/未知: 跳过 */
    }

    switch (emMethod2)
    {
    case CALI_METHOD_CU:      vCaliCu();     break;
    case CALI_METHOD_XI:      vCaliXi();     break;
    case CALI_METHOD_LIU_SHI: vCaliLiuShi(); break;
    default:                                  break;
    }
}
