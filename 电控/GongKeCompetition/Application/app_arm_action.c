/**
  * @file       app_arm_action.c
  * @brief      机械臂「动作预设」的数据表 + 播放实现。
  *
  *             预设 = 一段可命名的位姿/延时序列, 台架上用 Web 控制台「机械臂」Tab
  *             录出来（拖滑杆 → 记录位姿 → 加延时 → 保存）, 存进下面的数据区,
  *             然后作为**一块大积木**出现在「地图」Tab 的积木库里。
  *             路线里按 ID 引用它: STEP_ARM_PLAY_WAIT / STEP_ARM_PLAY_NO_WAIT。
  *
  *             ⚠️ 数据区由控制台读写（按行解析、未编辑的行原样回吐, 保证无操作保存
  *                零 diff）。手改可以, 但表头三行（预设/播放方式/备注）会被下次保存
  *                规范化; 步骤表内部的行则原样保留。
  *             ⚠️ 聚合初始化里的第 3 段 = 播放方式（stArmActionTdf::ucNoWait）,
  *                这是**运行时唯一**的真相源 —— 表头那行「播放方式」注释由控制台按同一
  *                份数据渲染。改聚合行的段数/顺序必须连控制台 arm_action.py 一起改。
  *
  *             播放刻意复用比赛脚本同一批原语（vArmGroupMoveWait / vArmSetZhuaShou /
  *             vTaskDelay）—— 台架上试跑看到的运动与路线跑起来是逐字节同一条路径,
  *             不存在「调试用的实现和比赛用的不一样」。
  */

#include "app_arm_action.h"
#include "app_arm.h"
#include "app_beam.h"

#include "FreeRTOS.h"
#include "task.h"

/* ==================== 数据区（控制台整段重写） ==================== */
/* ==== 数据区: 开始 ==== */

/* ===== 预设: 机械臂待命 ===== */
/* 播放方式: 启动即返回 */
/* 备注: 台架上录的样例: 验证大积木链路 */
const stScriptStepTdf g_astArmActionSteps1[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从初始位置伸出", 88.0f, 14.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从初始位置伸出-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "伸展机械臂", 88.0f, 37.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "伸展机械臂-夹爪", 0u, 67u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 62u),  // vArmSetZhuaShou()
};

/* ===== 预设: 抓取最右边的球 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps3[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 32.0f, 25.0f, 90.0f, 90.0f, 246.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓球", 32.0f, 25.0f, 90.0f, 90.0f, 246.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓球-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "转到爆破桶", 66.0f, 38.0f, 36.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "转到爆破桶-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "降低高度", 59.0f, 30.0f, 26.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "降低高度-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "放球", 59.0f, 33.0f, 23.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "放球-夹爪", 0u, 51u),  // vArmSetZhuaShou()
};

/* ===== 预设: 抓取中间的球 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps4[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 36.0f, 41.0f, 38.0f, 90.0f, 284.1f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓球", 36.0f, 41.0f, 38.0f, 90.0f, 284.1f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓球-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "转到爆破桶", 66.0f, 38.0f, 36.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "转到爆破桶-夹爪", 0u, 69u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "降低机械臂高度到桶边沿", 59.0f, 30.0f, 26.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "降低机械臂高度到桶边沿-夹爪", 0u, 69u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "放球", 59.0f, 33.0f, 26.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "放球-夹爪", 0u, 46u),  // vArmSetZhuaShou()
};

/* ===== 预设: 抓取左边的球 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps5[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 25.0f, 59.0f, 49.0f, 90.0f, 314.8f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿8", 25.0f, 59.0f, 45.0f, 90.0f, 315.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿8-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿7", 47.0f, 38.0f, 45.0f, 90.0f, 315.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿7-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "转到爆破桶", 66.0f, 38.0f, 36.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "转到爆破桶-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "下到爆破桶", 59.0f, 30.0f, 29.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "下到爆破桶-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "放球", 59.0f, 30.0f, 29.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "放球-夹爪", 1u, 72u),  // vArmSetZhuaShou()
};

/* ===== 预设: 抓取最右边人质 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps6[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "识别人质", 168.0f, 12.0f, 83.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "识别人质-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "切换准备识别人质", 122.0f, 24.0f, 83.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "切换准备识别人质-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 57.0f, 41.0f, 37.0f, 90.0f, 251.4f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 74u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓", 57.0f, 41.0f, 37.0f, 90.0f, 251.4f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓-夹爪", 1u, 34u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从下到上", 94.0f, 56.0f, 41.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从下到上-夹爪", 1u, 34u),  // vArmSetZhuaShou()
};

/* ===== 预设: 抓取中间人质 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps7[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "识别人质", 168.0f, 12.0f, 83.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "识别人质-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "切换准备识别人质", 122.0f, 24.0f, 83.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "切换准备识别人质-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 67.0f, 31.6f, 29.0f, 90.0f, 283.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓", 67.0f, 31.6f, 29.0f, 90.0f, 283.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓-夹爪", 0u, 68u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从下往上", 94.0f, 56.0f, 41.0f, 90.0f, 281.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从下往上-夹爪", 1u, 34u),  // vArmSetZhuaShou()
};

/* ===== 预设: 抓取最左边人质 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps8[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "识别人质", 168.0f, 12.0f, 83.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "识别人质-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "切换准备识别人质", 122.0f, 24.0f, 83.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "切换准备识别人质-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上到下", 54.0f, 31.0f, 63.0f, 90.0f, 322.9f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上到下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓人质", 54.0f, 31.0f, 63.0f, 90.0f, 322.9f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓人质-夹爪", 1u, 36u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从下到上", 94.0f, 56.0f, 41.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从下到上-夹爪", 1u, 35u),  // vArmSetZhuaShou()
};

/* ===== 预设: 识别小球 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps9[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿1", 94.0f, 46.0f, 13.0f, 90.0f, 4.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿1-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿2", 88.0f, 14.0f, 11.0f, 90.0f, 8.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿2-夹爪", 0u, 67u),  // vArmSetZhuaShou()
};

/* ===== 预设: 看靶子 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps10[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿1", 94.0f, 46.0f, 116.0f, 90.0f, 276.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿1-夹爪", 0u, 67u),  // vArmSetZhuaShou()
};

/* ===== 预设: 复位回待命 ===== */
/* 播放方式: 启动即返回 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps11[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "待命", 94.0f, 46.0f, 66.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "待命-夹爪", 0u, 38u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿2", 94.0f, 46.0f, 66.0f, 108.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿2-夹爪", 1u, 62u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿3", 94.0f, 46.0f, 66.0f, 73.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿3-夹爪", 1u, 62u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿4", 94.0f, 46.0f, 66.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿4-夹爪", 1u, 62u),  // vArmSetZhuaShou()
};

/* ===== 预设: 机械臂待命阻塞版本 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: 台架上录的样例: 验证大积木链路 */
const stScriptStepTdf g_astArmActionSteps12[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从初始位置伸出", 88.0f, 14.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从初始位置伸出-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "伸展机械臂", 88.0f, 37.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "伸展机械臂-夹爪", 0u, 67u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 62u),  // vArmSetZhuaShou()
};

/* ===== 预设: 切换准备识别人质 ===== */
/* 播放方式: 启动即返回 */
/* 备注: 台架上录的样例: 验证大积木链路 */
const stScriptStepTdf g_astArmActionSteps13[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 168.0f, 12.0f, 83.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 62u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "切换准备识别人质", 122.0f, 24.0f, 83.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "切换准备识别人质-夹爪", 1u, 72u),  // vArmSetZhuaShou()
};

/* ===== 预设: 切换准备识别人质阻塞版本 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: 台架上录的样例: 验证大积木链路 */
const stScriptStepTdf g_astArmActionSteps14[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 168.0f, 12.0f, 83.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 62u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "切换准备识别人质", 122.0f, 24.0f, 83.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "切换准备识别人质-夹爪", 1u, 72u),  // vArmSetZhuaShou()
};

/* ===== 预设: 切换到识别标靶 ===== */
/* 播放方式: 启动即返回 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps15[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "A", 94.0f, 46.0f, 66.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "A-夹爪", 1u, 62u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "B", 21.0f, 158.0f, 79.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "B-夹爪", 1u, 62u),  // vArmSetZhuaShou()
};

/* ===== 预设: 切换到识别标靶阻塞版本 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps16[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "A", 94.0f, 46.0f, 66.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "A-夹爪", 1u, 62u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "B", 21.0f, 158.0f, 79.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "B-夹爪", 1u, 62u),  // vArmSetZhuaShou()
};

/* ===== 预设: 复位回待命阻塞版本 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps17[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "待命", 94.0f, 46.0f, 66.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "待命-夹爪", 0u, 38u),  // vArmSetZhuaShou()
};

/* ===== 预设: 机械臂待命到斜视二维码模式 ===== */
/* 播放方式: 启动即返回 */
/* 备注: 台架上录的样例: 验证大积木链路 */
const stScriptStepTdf g_astArmActionSteps19[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从初始位置伸出", 88.0f, 14.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从初始位置伸出-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "伸展机械臂", 88.0f, 37.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "伸展机械臂-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿4", 88.0f, 53.0f, 48.0f, 90.0f, 7.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿4-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 41.0f, 66.0f, 117.0f, 90.0f, 50.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 0u, 67u),  // vArmSetZhuaShou()
};

/* ===== 预设: 机械臂待命到正视二维码模式 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: 台架上录的样例: 验证大积木链路 */
const stScriptStepTdf g_astArmActionSteps22[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从初始位置伸出", 88.0f, 14.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从初始位置伸出-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "伸展机械臂", 88.0f, 37.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "伸展机械臂-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到正视", 88.0f, 61.0f, 14.0f, 90.0f, 193.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到正视-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "开始正视", 11.0f, 121.0f, 132.0f, 8.0f, 193.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "开始正视-夹爪", 0u, 51u),  // vArmSetZhuaShou()
};

/* ===== 预设: 从正视二维码回收到初始位置 ===== */
/* 播放方式: 启动即返回 */
/* 备注: 台架上录的样例: 验证大积木链路 */
const stScriptStepTdf g_astArmActionSteps23[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "开始正视", 88.0f, 37.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "开始正视-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到正视", 88.0f, 61.0f, 14.0f, 90.0f, 193.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到正视-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "伸展机械臂", 88.0f, 37.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "伸展机械臂-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从初始位置伸出", 88.0f, 14.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从初始位置伸出-夹爪", 0u, 67u),  // vArmSetZhuaShou()
};

/* ===== 预设: 看路 ===== */
/* 播放方式: 启动即返回 */
/* 备注: 台架上录的样例: 验证大积木链路 */
const stScriptStepTdf g_astArmActionSteps26[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 73.0f, 55.0f, 64.0f, 90.0f, 101.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 62u),  // vArmSetZhuaShou()
};

/* ===== 预设: 机械臂待命到看路 ===== */
/* 播放方式: 启动即返回 */
/* 备注: 台架上录的样例: 验证大积木链路 */
const stScriptStepTdf g_astArmActionSteps27[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从初始位置伸出", 88.0f, 14.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从初始位置伸出-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "伸展机械臂", 88.0f, 37.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "伸展机械臂-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 73.0f, 55.0f, 64.0f, 90.0f, 101.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 62u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 62u),  // vArmSetZhuaShou()
};

/* ===== 预设: 机械臂待命到看路阻塞版本 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: 台架上录的样例: 验证大积木链路 */
const stScriptStepTdf g_astArmActionSteps28[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从初始位置伸出", 88.0f, 14.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从初始位置伸出-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "伸展机械臂", 88.0f, 37.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "伸展机械臂-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 73.0f, 55.0f, 64.0f, 90.0f, 101.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 62u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 62u),  // vArmSetZhuaShou()
};

/* ===== 预设: 抓取中间的球(单动作) ===== */
/* 播放方式: 启动即返回 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps29[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 36.0f, 41.0f, 38.0f, 90.0f, 284.1f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 36.0f, 40.0f, 39.4f, 90.0f, 281.5f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓球", 36.0f, 41.0f, 38.0f, 90.0f, 284.1f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓球-夹爪", 1u, 35u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓球", 36.0f, 40.0f, 39.0f, 90.0f, 281.5f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓球-夹爪", 0u, 65u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿8", 53.0f, 40.0f, 39.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿8-夹爪", 0u, 65u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿7", 36.0f, 40.0f, 39.0f, 90.0f, 281.5f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿7-夹爪", 1u, 35u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "转到爆破桶", 66.0f, 38.0f, 36.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "转到爆破桶-夹爪", 0u, 69u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "降低机械臂高度到桶边沿", 59.0f, 30.0f, 26.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "降低机械臂高度到桶边沿-夹爪", 0u, 69u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "放球", 59.0f, 33.0f, 26.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "放球-夹爪", 0u, 46u),  // vArmSetZhuaShou()
};

/* ===== 预设: 放置到爆爆桶(单动作) ===== */
/* 播放方式: 启动即返回 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps30[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 72u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 36.0f, 41.0f, 39.4f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓球", 36.0f, 41.0f, 39.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓球-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "转到爆破桶", 66.0f, 38.0f, 20.0f, 90.0f, 104.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "转到爆破桶-夹爪", 0u, 69u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "降低机械臂高度到桶边沿", 61.0f, 33.0f, 20.0f, 90.0f, 104.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "降低机械臂高度到桶边沿-夹爪", 0u, 69u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "放球", 61.0f, 33.0f, 20.0f, 90.0f, 104.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "放球-夹爪", 0u, 46u),  // vArmSetZhuaShou()
};

/* ===== 预设: 动作25 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps31[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿2", 21.0f, 158.0f, 76.0f, 90.0f, 303.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿2-夹爪", 1u, 62u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "你该调的", 21.0f, 158.0f, 74.0f, 90.0f, 306.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "你该调的-夹爪", 1u, 62u),  // vArmSetZhuaShou()
};

/* ===== 预设: 从抓取人质回到上电前位置 ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps32[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从下往上", 94.0f, 56.0f, 41.0f, 90.0f, 282.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从下往上-夹爪", 1u, 34u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "伸展机械臂", 88.0f, 37.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "伸展机械臂-夹爪", 0u, 67u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从初始位置伸出", 88.0f, 14.0f, 11.0f, 90.0f, 12.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从初始位置伸出-夹爪", 0u, 67u),  // vArmSetZhuaShou()
};

/* ===== 预设: 切换准备识别人质(反向) ===== */
/* 播放方式: 启动即返回 */
/* 备注: 台架上录的样例: 验证大积木链路 */
const stScriptStepTdf g_astArmActionSteps33[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 168.0f, 12.0f, 83.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 62u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "切换准备识别人质", 122.0f, 24.0f, 83.0f, 90.0f, 97.1f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "切换准备识别人质-夹爪", 1u, 72u),  // vArmSetZhuaShou()
};

/* ===== 预设: 复位回待命(反向) ===== */
/* 播放方式: 启动即返回 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps34[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "待命", 94.0f, 46.0f, 66.0f, 90.0f, 100.6f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "待命-夹爪", 0u, 38u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿2", 94.0f, 46.0f, 66.0f, 108.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿2-夹爪", 1u, 62u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿3", 94.0f, 46.0f, 66.0f, 73.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿3-夹爪", 1u, 62u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿4", 94.0f, 46.0f, 66.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿4-夹爪", 1u, 62u),  // vArmSetZhuaShou()
};

/* ===== 预设: 抓取中间的球(反向) ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps35[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 101.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 36.0f, 41.0f, 39.4f, 90.0f, 104.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓球", 36.0f, 41.0f, 39.0f, 90.0f, 104.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓球-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "位姿7", 53.0f, 41.0f, 39.0f, 90.0f, 104.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "位姿7-夹爪", 1u, 35u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "转到爆破桶", 66.0f, 38.0f, 36.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "转到爆破桶-夹爪", 0u, 69u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "降低机械臂高度到桶边沿", 59.0f, 30.0f, 26.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "降低机械臂高度到桶边沿-夹爪", 0u, 69u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "放球", 59.0f, 33.0f, 26.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "放球-夹爪", 0u, 46u),  // vArmSetZhuaShou()
};

/* ===== 预设: 放置到爆爆桶(反向) ===== */
/* 播放方式: 启动即返回 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps36[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 72u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 36.0f, 41.0f, 39.4f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓球", 36.0f, 41.0f, 39.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓球-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "转到爆破桶", 66.0f, 38.0f, 20.0f, 90.0f, 288.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "转到爆破桶-夹爪", 0u, 69u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "降低机械臂高度到桶边沿", 61.0f, 33.0f, 20.0f, 90.0f, 288.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "降低机械臂高度到桶边沿-夹爪", 0u, 69u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "放球", 61.0f, 33.0f, 20.0f, 90.0f, 288.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "放球-夹爪", 0u, 46u),  // vArmSetZhuaShou()
};

/* ===== 预设: 切换到识别标靶（反向） ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps37[] = {
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "B", 21.0f, 158.0f, 79.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "B-夹爪", 1u, 62u),  // vArmSetZhuaShou()
};

/* ===== 预设: 抓取中间人质(反向) ===== */
/* 播放方式: 阻塞到位 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps38[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "识别人质", 168.0f, 12.0f, 83.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "识别人质-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "切换准备识别人质", 122.0f, 24.0f, 83.0f, 90.0f, 100.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "切换准备识别人质-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 67.0f, 31.6f, 29.0f, 90.0f, 94.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓", 67.0f, 31.6f, 29.0f, 90.0f, 94.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓-夹爪", 0u, 68u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从下往上", 94.0f, 56.0f, 41.0f, 90.0f, 94.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从下往上-夹爪", 1u, 34u),  // vArmSetZhuaShou()
};

/* ===== 预设: 放置人质到爆爆桶(人质单动作) ===== */
/* 播放方式: 启动即返回 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps40[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 72u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 36.0f, 41.0f, 39.4f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓球", 36.0f, 41.0f, 39.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓球-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从下往上", 94.0f, 56.0f, 41.0f, 90.0f, 104.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从下往上-夹爪", 1u, 34u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "转到爆破桶", 66.0f, 38.0f, 20.0f, 90.0f, 104.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "转到爆破桶-夹爪", 0u, 69u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "降低机械臂高度到桶边沿", 61.0f, 38.0f, 20.0f, 90.0f, 104.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "降低机械臂高度到桶边沿-夹爪", 0u, 69u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "放球", 61.0f, 33.0f, 20.0f, 90.0f, 104.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "放球-夹爪", 0u, 46u),  // vArmSetZhuaShou()
};

/* ===== 预设: 放置人质到爆爆桶(人质单动作反向) ===== */
/* 播放方式: 启动即返回 */
/* 备注: */
const stScriptStepTdf g_astArmActionSteps41[] = {
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "旋转到待命位置", 94.0f, 46.0f, 66.0f, 90.0f, 280.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "旋转到待命位置-夹爪", 1u, 72u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从上往下", 36.0f, 41.0f, 39.4f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从上往下-夹爪", 1u, 72u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "抓球", 36.0f, 41.0f, 39.0f, 90.0f, 279.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "抓球-夹爪", 1u, 35u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "从下往上", 94.0f, 56.0f, 41.0f, 90.0f, 288.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "从下往上-夹爪", 1u, 34u),  // vArmSetZhuaShou()
//    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "转到爆破桶", 66.0f, 38.0f, 20.0f, 90.0f, 104.0f),  // vArmGroupMoveWait()
//    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "转到爆破桶-夹爪", 0u, 69u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "降低机械臂高度到桶边沿", 61.0f, 38.0f, 20.0f, 90.0f, 288.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "降低机械臂高度到桶边沿-夹爪", 0u, 69u),  // vArmSetZhuaShou()
    SCRIPT_STEP_F5     (STEP_ARM_GROUP_WAIT,                  "放球", 61.0f, 33.0f, 20.0f, 90.0f, 288.0f),  // vArmGroupMoveWait()
    SCRIPT_STEP_I2     (STEP_SET_ZHUASHOU_WAIT,               "放球-夹爪", 0u, 46u),  // vArmSetZhuaShou()
};

const stArmActionTdf g_astArmAction[ARM_ACTION_NUM] = {
    { 1u, "机械臂待命", 1u, g_astArmActionSteps1, 4u },
    { 3u, "抓取最右边的球", 0u, g_astArmActionSteps3, 12u },
    { 4u, "抓取中间的球", 0u, g_astArmActionSteps4, 12u },
    { 5u, "抓取左边的球", 0u, g_astArmActionSteps5, 14u },
    { 6u, "抓取最右边人质", 0u, g_astArmActionSteps6, 8u },
    { 7u, "抓取中间人质", 0u, g_astArmActionSteps7, 8u },
    { 8u, "抓取最左边人质", 0u, g_astArmActionSteps8, 8u },
    { 9u, "识别小球", 0u, g_astArmActionSteps9, 4u },
    { 10u, "看靶子", 0u, g_astArmActionSteps10, 2u },
    { 11u, "复位回待命", 1u, g_astArmActionSteps11, 2u },
    { 12u, "机械臂待命阻塞版本", 0u, g_astArmActionSteps12, 4u },
    { 13u, "切换准备识别人质", 1u, g_astArmActionSteps13, 2u },
    { 14u, "切换准备识别人质阻塞版本", 0u, g_astArmActionSteps14, 2u },
    { 15u, "切换到识别标靶", 1u, g_astArmActionSteps15, 4u },
    { 16u, "切换到识别标靶阻塞版本", 0u, g_astArmActionSteps16, 4u },
    { 17u, "复位回待命阻塞版本", 0u, g_astArmActionSteps17, 2u },
    { 19u, "机械臂待命到斜视二维码模式", 1u, g_astArmActionSteps19, 8u },
    { 22u, "机械臂待命到正视二维码模式", 0u, g_astArmActionSteps22, 8u },
    { 23u, "从正视二维码回收到初始位置", 1u, g_astArmActionSteps23, 6u },
    { 26u, "看路", 1u, g_astArmActionSteps26, 2u },
    { 27u, "机械臂待命到看路", 1u, g_astArmActionSteps27, 6u },
    { 28u, "机械臂待命到看路阻塞版本", 0u, g_astArmActionSteps28, 6u },
    { 29u, "抓取中间的球(单动作)", 1u, g_astArmActionSteps29, 6u },
    { 30u, "放置到爆爆桶(单动作)", 1u, g_astArmActionSteps30, 6u },
    { 31u, "动作25", 0u, g_astArmActionSteps31, 4u },
    { 32u, "从抓取人质回到上电前位置", 0u, g_astArmActionSteps32, 6u },
    { 33u, "切换准备识别人质(反向)", 1u, g_astArmActionSteps33, 2u },
    { 34u, "复位回待命(反向)", 1u, g_astArmActionSteps34, 2u },
    { 35u, "抓取中间的球(反向)", 0u, g_astArmActionSteps35, 6u },
    { 36u, "放置到爆爆桶(反向)", 1u, g_astArmActionSteps36, 6u },
    { 37u, "切换到识别标靶（反向）", 0u, g_astArmActionSteps37, 2u },
    { 38u, "抓取中间人质(反向)", 0u, g_astArmActionSteps38, 8u },
    { 40u, "放置人质到爆爆桶(人质单动作)", 1u, g_astArmActionSteps40, 6u },
    { 41u, "放置人质到爆爆桶(人质单动作反向)", 1u, g_astArmActionSteps41, 6u },
};

/* ==== 数据区: 结束 ==== */

/* ==================== 后台序列状态 ====================
 * 只有 vArmTask 的泵（vArmActionPump）会「在跑」, 别的任务只是投递请求/中止。
 * 跨任务共享三个单字, 按 Beam 规则第 4 条不需要加锁（Cortex-M4 单字读写原子,
 * 函数调用 + 任务切换即内存屏障）; 真正的正确性靠**代次号**: `vArmSeqAbort()`
 * 把它加一, 正在跑的那一轮在下一个步骤边界发现代次变了就退出,
 * 于是不会把新序列的状态当成自己的清掉。
 *
 * ⚠️⚠️ **代次号只有 `vArmSeqAbort()` 一家加一 —— 投递（含追加进队列）千万别加。**
 *   泵靠「代次变了就在下一步骤边界退出」响应中止; 投递时加一 = 把正在跑的那条
 *   当成"被中止"掐断（症状: 「抓取只走了几步就跳去复位」）。
 *   （本注释以前写成"任何一次投递/中止都会把代次加一", 与代码相反、也违反上面这条 —— 已改正。）
 *
 * 「有没有序列在跑」要跨模块读（蓝牙 Tab 遥测）→ 那个走 Beam（arm_seq_busy）,
 * 且**只有泵一个写者**（Abort 不碰它 —— 中止后当前这一拍还在动舵机, 报 busy
 * 才是诚实的; 泵退出时自然会清）。 */
static const stArmActionTdf *s_pstRunning = NULL;   /* 泵当前在跑的那条（**只有泵写**: 认领时置、收尾/中止时清） */
static uint16_t              s_usStep     = 0;      /* 泵的进度（只有泵写） */
static uint32_t              s_ulSeqGen   = 0;      /* 代次（**只有 vArmSeqAbort 加一**, 见上） */

/* ----- 「延时待投」槽位（2026-10-08 加, STEP_ARM_PLAY_NO_WAIT 的 delay_ms 参数） -----
 * 单槽: 记「到点后要把哪条动作按 vArmPlayNoWait 同款（先中止再入队）投出去」。
 * 计时**完全在后台**: 投递的步骤立即返回, 泵每 ≤20ms 醒来看一眼到没到点 —— 后续步骤
 * （转弯/巡线照常先走）。到点前再投一条 = **后写者胜**（旧的那条作废, 与 vArmPlayNoWait
 * 的覆盖语义同一口径）。
 * 写者 = 脚本任务（vArmPlayNoWaitDelay）; 清除 = vArmSeqAbort（任何新臂指令/中止掐序列
 * 时一并丢, 防止「按了停/切编辑模式之后臂过一会儿又自己动一下」）与泵自己（到点投出/
 * 脚本中止电平置位时丢弃）。指针 + 两个数不是单字原子, 写/清/取**一律进临界区**
 * （同下面队列的口径: 只有几条内存指令, 绝不碰串口/延时）。 */
static const stArmActionTdf *s_pstDelayPend  = NULL;   /* 到点待投（NULL = 无） */
static TickType_t            xDelayPostTick  = 0;      /* 投递时刻（算耗时用） */
static uint32_t              s_ulDelayMs     = 0;      /* 延迟时长[ms] */

/* ==================== 后台待播队列（2026-09 加） ====================
 * 单条的 s_pstRunning 装不下「两条都要在后台依次跑完」这件事: 排爆抓取 / 反恐任务里
 * 选中的动作若标了「启动即返回」, **积木本身要立刻返回**, 而选中的动作（排爆还有那条
 * 复位）得一条接一条在后台跑 —— 后投的那条不能把前一条掐掉（那就是 vArmSeqAbort 的
 * 「后写者胜」, 用在这里等于把刚发起的抓取砍断）。于是加一个小的 FIFO。
 *
 * ⚠️ 遥测 `arm_seq_busy` 仍是**泵一个写者**, 队列里刚投进去还没被认领的那 ≲20ms 会显示
 *    0（泵空闲时在 20ms 让出）—— 那是诚实的「还没开跑」, 别为它去加第二个写者。
 *
 * ⚠️ 三个上下文会碰它: 脚本任务 / vBtTask 投递（push）、任何任务调 vArmSeqAbort（clear）、
 *    vArmTask 的泵 pop。读写头尾两个下标**不是单字原子**的那种情形, 所以 push/pop/clear
 *    一律 taskENTER_CRITICAL 包住 —— 里面只有几条内存指令、**绝不碰串口/延时**
 *    （临界区里不能发串口那条铁律针对的是忙等, 见 CLAUDE.md「中断与并发铁律」第 5 条）。 */
#define ARM_SEQ_QUEUE_LEN 4u
static const stArmActionTdf *s_apstQueue[ARM_SEQ_QUEUE_LEN];
static uint8_t               s_ucQHead  = 0u;   /* 出队位置（泵写） */
static uint8_t               s_ucQTail  = 0u;   /* 入队位置（投递者写） */
static uint8_t               s_ucQCount = 0u;   /* 待播条数 */

/* 阻塞档等后台跑空的采样节拍与上界（上界是兜底护栏, 不是节拍）。
 * 正常操作远小于上界: 每条动作的每一步本身都有上界（vArmGroupMoveWait 的 ulBound、
 * 夹爪的 ZHUASHOU_MOVE_MS、STEP_DELAY 的毫秒数）, 所以队列总会跑空; 上界只为
 * 「泵被饿死 / 舵机卡住」那种异常兜底 —— 超时**放行但必须响**（见 c_ucSeqDrain）。 */
#define ARM_SEQ_DRAIN_POLL_MS    10u
#define ARM_SEQ_DRAIN_TIMEOUT_MS 60000u

/* 清空待播队列（只用在自己的临界区里, 见 vArmSeqAbort）。 */
static void c_vQueueClearLocked(void)
{
    s_ucQHead  = 0u;
    s_ucQTail  = 0u;
    s_ucQCount = 0u;
}

/* 追加到队尾（自持锁）。返回 0 = 队列满, **队列一字不动**。
 *
 * ⚠️ 满时丢的是**这一条新的**、不是最旧的那条: 已经在队列里的都是"已经承诺过要播"的
 *    （排爆那两步就是这么排进来的）, 悄悄删一条等于让它凭空不执行; 而新的这条被拒时
 *    调用方拿得到 0, 可以响。深度 4 对「进这一步前已 abort 清空、本步最多排 2 条」
 *    来说有 2 倍余量 —— 真满了说明有 bug, 那就该响。 */
static uint8_t c_ucQueueAppend(const stArmActionTdf *pst)
{
    uint8_t ucOk = 0u;

    taskENTER_CRITICAL();
    if (s_ucQCount < (uint8_t)ARM_SEQ_QUEUE_LEN)
    {
        s_apstQueue[s_ucQTail] = pst;
        s_ucQTail = (uint8_t)((s_ucQTail + 1u) % (uint8_t)ARM_SEQ_QUEUE_LEN);
        s_ucQCount++;
        ucOk = 1u;
    }
    taskEXIT_CRITICAL();
    return ucOk;
}

/* 出队 + 认领当前 = **一个原子动作**。
 * ⚠️ 这两件事必须同处一个临界区: 否则「队列已空、s_pstRunning 还没写上」那一瞬间
 *    会被 c_bSeqPending / drain 看成「后台跑空了」而提前放行（复位还没播完车就开走,
 *    随机、极难复现）。认领时顺手置忙, 于是「忙」的语义 = 泵手上有活。
 * ⚠️⚠️ **代次号也必须在这个临界区里一起取走**（pulGenOut）: 原来泵是认领**之后**在
 *    临界区外读 s_ulSeqGen —— taskEXIT_CRITICAL 恢复 BASEPRI 是个天然抢占点, 若一次
 *    vArmSeqAbort()（代次 +1、清队列、s_pstRunning=NULL）刚好落在那儿, 泵就会认领到
 *    **中止之后**的新代次, 把被中止的那条动作**整条跑完** —— 正是「新指令显式终止旧序列」
 *    要防的两边抢舵机。取在区内, 认领与代次就是同一瞬间的。 */
static const stArmActionTdf *c_pstQueueClaim(uint32_t *pulGenOut)
{
    const stArmActionTdf *pst = NULL;

    taskENTER_CRITICAL();
    *pulGenOut = s_ulSeqGen;
    if (s_ucQCount > 0u)
    {
        pst = s_apstQueue[s_ucQHead];
        s_ucQHead = (uint8_t)((s_ucQHead + 1u) % (uint8_t)ARM_SEQ_QUEUE_LEN);
        s_ucQCount--;
        s_pstRunning = pst;
        s_usStep     = 0;
        vSetArmSeqBusy(1u);
    }
    taskEXIT_CRITICAL();
    return pst;
}

/* 还有没有「在跑 / 待播」的（drain 的判据）。
 * ⚠️ 必须在临界区里**同时**读两处: 只读一处都可能在「泵刚跑完当前那条、正要取队列」
 *    的那一瞬间看到「空」而提前放行。 */
static uint8_t c_bSeqPending(void)
{
    uint8_t ucRet;

    taskENTER_CRITICAL();
    ucRet = (uint8_t)((s_ucQCount > 0u) || (s_pstRunning != NULL));
    taskEXIT_CRITICAL();
    return ucRet;
}

static uint8_t c_bQueueEmpty(void)
{
    uint8_t ucRet;

    taskENTER_CRITICAL();
    ucRet = (uint8_t)(s_ucQCount == 0u);
    taskEXIT_CRITICAL();
    return ucRet;
}

/* ==================== 查表 ==================== */

static const stArmActionTdf *c_pstFind(emArmActionIdTdf emId)
{
    uint8_t ucI;

    /* 线性查找: 预设张数是个位数, 比二分省事且不怕表被手改乱序 */
    for (ucI = 0; ucI < ARM_ACTION_NUM; ucI++)
    {
        if (g_astArmAction[ucI].ucId == (uint8_t)emId)
        {
            return &g_astArmAction[ucI];
        }
    }
    return NULL;
}

uint8_t ucArmActionExists(emArmActionIdTdf emId)
{
    return (uint8_t)((emId != (emArmActionIdTdf)0) && (c_pstFind(emId) != NULL));
}

/* 「机械臂已展开」标志的唯一置位点（2026-09 加, 见 app_beam.h）。
 * 所有播放路径（vArmPlayWait / vArmPlayNoWait / ucArmPlayByMode）—— 也就是路线里的
 * 两块大积木、$B,PLAY、排爆/反恐那两处 —— 播完都会走到这儿。
 *
 * ⚠️ **只认这两条待命序列**（ARM_ACT_1「机械臂待命」/ ARM_ACT_12「待命阻塞版本」）:
 *    它们第一个动作是「从初始位置伸出」, 是**从折叠位安全展开**的那条路。
 *    别的臂动作（**含「复位回待命」ARM_ACT_11/17**）都是直接摆到目标位姿 ——
 *    从折叠位起摆有撞桌子/撞车身的风险, 所以它们不构成「已安全展开」。
 *    加新的待命序列时记得把它也列进来。
 *
 * ⚠️ 记的是「**展开过**」不是「现在停在待命」: 用户口径是「展开过就够了, 之后臂挪到
 *    哪儿都无所谓」。所以臂干完活停在别处时标志仍然保持 1。
 * ⚠️ 只在**完整走完**时置位, 半途而废的展开不算展开 —— 那正是本标志要防的
 *    「以为臂出去了、其实还折着」。两个调用点各自保证这点, 但机制不同:
 *      · vArmPlayWait: 表一定会逐步走完（那个循环**不查代次号**, vArmSeqAbort 停不了它）,
 *        所以调用点放循环之后即可。
 *      · vArmActionPump: 放在「代次没变」的收尾分支里 —— 被中止/被新序列顶掉时就**不置位**。 */
static void vArmNoteDeployed(const stArmActionTdf *pst)
{
    if (pst == NULL)
    {
        return;
    }
    if ((pst->ucId == (uint8_t)ARM_ACT_1) || (pst->ucId == (uint8_t)ARM_ACT_12))
    {
        vSetArmDeployed(1u);
    }
}

/* ==================== 单步执行 ====================
 * 与 app_script.c 里 STEP_ARM_GROUP_WAIT / STEP_SET_ZHUASHOU_WAIT / STEP_DELAY
 * 三个 case **同一批调用**。白名单外的步骤跳过（不猜语义）。 */
static void vArmExecStep(const stScriptStepTdf *pstStep)
{
    switch (pstStep->eType)
    {
    case STEP_ARM_GROUP_WAIT:
        vArmGroupMoveWait(pstStep->afF);
        break;

    case STEP_ARM_GROUP_NO_WAIT:
        vArmGroupMoveNoWait(pstStep->afF);
        break;

    case STEP_SET_ZHUASHOU_WAIT:
        vArmSetZhuaShou((uint8_t)pstStep->anI[0], (uint8_t)pstStep->anI[1]);
        break;

    case STEP_DELAY:
        vTaskDelay(pdMS_TO_TICKS((uint32_t)pstStep->anI[0]));
        break;

    default:
        break;
    }
}

/* ==================== 播放 ==================== */

void vArmSeqAbort(void)
{
    /* 代次 +1 让正在跑的那一轮在下一个**步骤边界**退出（当前那一拍仍在动舵机, 所以 busy
     * 不由这里清 —— 见 app_beam.h）; 队列表**一起清**: 不清的话 abort 之后那些待播项还会
     * 被泵取出来播, 那就是「中止不掉」（现象: 点了一下点动, 臂过一会儿又自己动一下）。 */
    taskENTER_CRITICAL();
    s_ulSeqGen++;
    s_pstRunning = NULL;
    s_usStep     = 0;
    c_vQueueClearLocked();
    /* 「延时待投」的一并丢 —— 同一个道理再往前延伸一步: 中止/换臂指令之后, 还没到点的
     * 延时投递更不许到点后再动臂。vArmPlayNoWaitDelay 的文档与这行是一体的, 别单删。 */
    s_pstDelayPend = NULL;
    taskEXIT_CRITICAL();
}

void vArmPlayWait(emArmActionIdTdf emId)
{
    const stArmActionTdf *pst = c_pstFind(emId);
    uint16_t usI;

    vArmSeqAbort();                 /* 后写者胜: 先终止在跑的后台序列 */

    if (pst == NULL)
    {
        return;
    }

    for (usI = 0; usI < pst->usCount; usI++)
    {
        /* 逐条喂蓝牙编辑模式看门狗: $B,PLAY 是阻塞的, 这段时间 vBtTask 收不到
         * 新指令, 不喂的话 8s 空闲超时会把编辑模式判成「PC 掉线」而退出。
         * 这是 s_ulBtCmdTick 的第二个写者（见 app_beam.h 的登记）。 */
        vSetBtCmdTick(xTaskGetTickCount());
        vArmExecStep(&pst->pstSteps[usI]);
    }

    /* 走完了才记「机械臂已展开」（只对待命序列生效, 见 vArmNoteDeployed）。
     * 这里不需要额外的「完整走完」判据: 上面那个循环不查代次号, 表一定会逐步走完,
     * 所以执行到这一行就等价于「整条序列跑完了」。 */
    vArmNoteDeployed(pst);
}

void vArmPlayNoWait(emArmActionIdTdf emId)
{
    const stArmActionTdf *pst = c_pstFind(emId);

    vArmSeqAbort();                 /* 同一时刻只允许一个后台序列（队列也一起清 = 后写者胜） */

    if (pst == NULL)
    {
        return;
    }

    /* ⚠️ 必须**入队**投递, 不能只写 s_pstRunning —— 泵（vArmActionPump）唯一取活的口子
     *    是 c_pstQueueClaim(), 只写 s_pstRunning 等于投进了一个没人看的信箱: 序列永远
     *    不播, 而且**完全静默**（不响、不报错, 现象就是「路线里那块大积木点了没反应」）。
     *    真踩过 —— 默认路线里那四块 STEP_ARM_PLAY_NO_WAIT 全都不动。
     *    又因为上面刚 abort 过（队列已清空）, 这里必然落进空队, 「后写者胜」一字不变。 */
    (void)c_ucQueueAppend(pst);     /* 投递; 由 vArmTask 的泵认领 */
}

/* 延时投递（2026-10-08 加, STEP_ARM_PLAY_NO_WAIT 的 delay_ms 参数; 声明见 app_arm_action.h）:
 * ulDelayMs = 0 → 直接走 vArmPlayNoWait（逐字节老路径, 立即入队）;
 * ≠ 0 → 记进「延时待投」槽位**立即返回** —— 计时由 vArmTask 的泵在后台走（≤20ms 精度,
 *        与后续步骤完全并行）, 到点才按 vArmPlayNoWait 同款（先中止旧序列再入队）投出。
 * 用途: 「边转弯边等 X ms 后展开臂」这类时序 —— 老写法只能排 STEP_DELAY, 车就得停着等。
 * ⚠️ 到点前再投一条 = 后写者胜（旧的那条作废）; 任何 vArmSeqAbort()（新臂指令/脚本中止
 *    掐序列）或脚本中止电平都会**丢弃**还没到点的投递 —— 按了停之后臂不许事后自己动。
 * ID 不存在则直接返回（与 vArmPlayNoWait 同款静默）。仅任务上下文可调。 */
void vArmPlayNoWaitDelay(emArmActionIdTdf emId, uint32_t ulDelayMs)
{
    const stArmActionTdf *pst = c_pstFind(emId);

    if (pst == NULL)
    {
        return;
    }

    if (ulDelayMs == 0u)
    {
        vArmPlayNoWait(emId);       /* 缺省 = 老行为, 一字节不偏 */
        return;
    }

    taskENTER_CRITICAL();
    s_pstDelayPend = pst;
    xDelayPostTick = xTaskGetTickCount();
    s_ulDelayMs    = ulDelayMs;
    taskEXIT_CRITICAL();
}

/* 等后台序列（含待播队列）跑空 —— **不中止**。返回 1 = 已跑空, 0 = 超时放弃。
 * 用途: ucArmPlayByMode 的阻塞那一档靠它把「排在前面的（含自己这条, 由泵推进）」等到播完,
 *       而不是像 vArmPlayWait 那样先 abort —— 那会把刚投的 ③ 掐断。
 * 仅任务上下文可调（内部 vTaskDelay）。 */
static uint8_t c_ucSeqDrain(void)
{
    uint32_t ulWaited = 0u;

    while (c_bSeqPending())
    {
        if (ulWaited >= (uint32_t)ARM_SEQ_DRAIN_TIMEOUT_MS)
        {
            /* 兜底出口: 泵被饿死/舵机卡住时硬等下去会把整条路线挂死（车不动、无任何提示、
             * 只能断电）, 放行至少让路线继续走 —— 由调用方报出去留痕
             * （臂还在动、车却开走了, 那是比冻死更危险的状态）。正常绝不该到这里:
             * 泵的每一步都自带上界, 任何 Abort 也会立刻清空队列让判据成立。 */
            return 0u;
        }
        /* 台架演练时这一段常常跑在**编辑模式**里（PC 就停在排爆那一步看着）,
         * 所以和 vArmPlayWait 一样逐拍喂看门狗 —— 否则 8s 空闲超时会把 PC 判成掉线。 */
        vSetBtCmdTick(xTaskGetTickCount());
        vTaskDelay(pdMS_TO_TICKS(ARM_SEQ_DRAIN_POLL_MS));
        ulWaited += (uint32_t)ARM_SEQ_DRAIN_POLL_MS;
    }
    return 1u;
}

uint8_t ucArmPlayByMode(emArmActionIdTdf emId)
{
    const stArmActionTdf *pst = c_pstFind(emId);

    if (pst == NULL)
    {
        return 1u;                  /* 与 vArmPlayWait 同款: 不存在的 ID 静默（调用方另有防呆） */
    }

    /* ⚠️ 先入队, 两种模式共用一个落点 —— 于是「什么时候播」完全由泵统一推进。
     *    这里**刻意不调 vArmSeqAbort**: 追加的语义正是「前一条还在跑, 这条接在后面」,
     *    中止它就把刚发起的抓取砍断了（那正是本改动要修的老毛病）。
     *    代次号也不动 —— 动了会让正在跑的那条在下一个步骤边界被自己投的这条截断。 */
    if (c_ucQueueAppend(pst) == 0u)
    {
        return 0u;                  /* 队列满（不可达, 见 c_ucQueueAppend 的说明）*/
    }

    if (pst->ucNoWait != 0u)
    {
        return 1u;                  /* 启动即返回: 交给泵, 本步立刻往下走 */
    }

    /* 阻塞到位: 等整条队列（含我前面排的那些, 还有正在跑的）跑空。
     * ⚠️ 到这里队列是空的吗? 不一定 —— ③ 标了「启动即返回」时它就还排在前面。 */
    return c_ucSeqDrain();
}

void vArmActionPump(void)
{
    const stArmActionTdf *pst;
    uint32_t ulGen;

    /* 「延时待投」到点检查（2026-10-08, 见 s_pstDelayPend 处的说明）:
     * vArmTask 每轮都调本函数 —— 空闲时 20ms 一醒, 所以**臂闲着时**投递精度 ≤20ms;
     * 有序列在跑时本钩子要等那条播完（下一轮）才路过, 到点的投递顺延到臂空 ——
     * 这是有意的: 到点就掐正在播的动作, 等于让延时段落砍掉上一段, 不如等它空。
     * 到点 = 按 vArmPlayNoWait 同款投出（先中止再入队, 后写者胜; 此刻槽位已取空,
     * 里面那次 vArmSeqAbort 清槽清的是空, 无害）。脚本中止电平置位 = 直接丢弃。 */
    if (s_pstDelayPend != NULL)
    {
        const stArmActionTdf *pstDue = NULL;

        taskENTER_CRITICAL();
        if (s_pstDelayPend != NULL)
        {
            if (ucGetScriptAbortReq() != 0u)
            {
                s_pstDelayPend = NULL;                       /* 按了停 = 丢弃 */
            }
            else if ((uint32_t)((xTaskGetTickCount() - xDelayPostTick)
                                * portTICK_PERIOD_MS) >= s_ulDelayMs)
            {
                pstDue         = s_pstDelayPend;
                s_pstDelayPend = NULL;
            }
        }
        taskEXIT_CRITICAL();

        if (pstDue != NULL)
        {
            vArmSeqAbort();
            (void)c_ucQueueAppend(pstDue);
        }
    }

    /* 出队 + 认领 + 置忙 + **取代次** = 一个原子动作（见 c_pstQueueClaim）。
     * 没有活干才让出。 */
    pst = c_pstQueueClaim(&ulGen);
    if (pst == NULL)
    {
        /* 没有在跑也没有待播的: 清忙 + 让出 20ms（vArmTask 的循环体）。
         * ⚠️ 这一行是「被 Abort 中止之后」唯一的清零点: 中止路径的尾判被代次号挡掉了
         *    （见下）, 不清的话 arm_seq_busy 会一直停在 1 直到下一条序列跑完 ——
         *    遥测就一直在谎报「有序列在跑」。Abort 本身仍然不碰它。 */
        vSetArmSeqBusy(0u);
        vTaskDelay(pdMS_TO_TICKS(20));
        return;
    }

    while ((s_usStep < pst->usCount) && (s_ulSeqGen == ulGen))
    {
        vArmExecStep(&pst->pstSteps[s_usStep]);
        s_usStep++;
    }

    /* 代次没变 = 没有被中止也没被新序列顶掉, 才由我收尾。
     * 变了就什么都别碰 —— 那是接手者的状态（清 busy 的活由下面那次「没活干」的分支
     * 或下一条序列的收尾负责）。 */
    if (s_ulSeqGen == ulGen)
    {
        /* 走完了才记「机械臂已展开」（只对待命序列生效, 见 vArmNoteDeployed）。
         * ⚠️ 放在这个分支里是**有意的**: 代次变了说明这条被中止/顶掉了, 臂只走了一半,
         *    那时置位就是谎报「已展开」—— 而本标志存在的意义正是防这个。 */
        vArmNoteDeployed(pst);

        s_pstRunning = NULL;
        s_usStep     = 0;
        /* 队列里还有待播的就先别清 busy —— vArmTask 是紧循环, 下一轮立刻接着认领,
         * 于是「排了两条」对遥测是一个连续的 busy=1; 都跑完了才报 0。 */
        if (c_bQueueEmpty())
        {
            vSetArmSeqBusy(0u);
        }
    }
}
