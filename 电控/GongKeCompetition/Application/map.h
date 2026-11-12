/**
  * @file       map.h
  * @brief      路线表对外声明：map.c 集中管理整套流程的动作顺序。
  */

#ifndef __MAP_H
#define __MAP_H

#include "app_script.h"    /* stScriptStepTdf / SCRIPT_STEP_* 宏 */

/* 多预设路线表（map.c 定义）：
 * map.c 里每张 g_astScriptPresetN[] 是一套完整路线,
 * 文件末尾的 #define SCRIPT_PRESET_ACTIVE 选当前跑哪张（控制台可切换）。
 * 外部只认 g_pstScriptActive / g_usScriptActiveCount 这两个符号, 不必关心具体有几套预设。
 *
 * 「跳到别的预设」那套积木（STEP_SCRIPT_CALL / STEP_SCRIPT_RETURN）
 * 还需要一张 id → 表 的注册表, 见下面 g_astScriptPreset[] 与 emScriptPresetIdTdf。
 * ⚠️ 条件分支（STEP_IF / STEP_ELSE / STEP_END_IF）**不走这张表** —— 它就在本表内部靠位置配对,
 *    所以嵌套多少层分支都不会多出预设。 */

/* ==================================================================
 * 预设 id 枚举 —— 控制台「地图」Tab 生成, 标记之间由控制台整段重写。
 * ⚠️ 手改会被下次保存覆盖; 要加预设请用控制台（或同步改 map.c 的注册表）。
 * ⚠️ id 只增不重用（SCRIPT_PRESET_NEXT_ID 单调）: 删掉预设 7 再新建,
 *    新预设拿到的是 9 而不是 7 —— 否则 map.c 里残留的 SCRIPT_PRESET_7
 *    会**无声地指向另一条路线**（同 ARM_ACTION_NEXT_ID 那个坑）。
 * 成员尾注释 = 控制台目标下拉里的标签, 由预设标题派生;
 * 「不可跳入」是控制台算出来的警告（表首带握手 / 表内含重置脚本）。
 * ================================================================== */
/* ==== 生成块: 开始 ==== */
typedef enum {
    SCRIPT_PRESET_NONE = 0,   /* 继续（不跳转） */
    SCRIPT_PRESET_1    = 1,   /* 默认路线（不可跳入） */
    SCRIPT_PRESET_2    = 2,   /* 测试巡线闭环 */
    SCRIPT_PRESET_3    = 3,   /* 测试减速带 */
    SCRIPT_PRESET_4    = 4,   /* 测试原地转弯 */
    SCRIPT_PRESET_5    = 5,   /* 测试脚本重置（不可跳入） */
    SCRIPT_PRESET_6    = 6,   /* 测试激光 */
    SCRIPT_PRESET_7    = 7,   /* 测试机械臂（不可跳入） */
    SCRIPT_PRESET_8    = 8,   /* 测试标靶（不可跳入） */
    SCRIPT_PRESET_9    = 9,   /* 测试小球对位（不可跳入） */
    SCRIPT_PRESET_10   = 10,   /* 测试积木 */
    SCRIPT_PRESET_11   = 11,   /* 测试人质对位（不可跳入） */
    SCRIPT_PRESET_12   = 12,   /* 测试标靶流控（不可跳入） */
    SCRIPT_PRESET_14   = 14,   /* OLED色球放置（可重入） */
    SCRIPT_PRESET_15   = 15,   /* OLED激光射靶（可重入） */
    SCRIPT_PRESET_16   = 16,   /* OLED人质抓取（可重入） */
    SCRIPT_PRESET_19   = 19,   /* 测试主摄像头识别二维码 */
    SCRIPT_PRESET_22   = 22,   /* 测试支点顺畅转弯（不可跳入） */
    SCRIPT_PRESET_23   = 23,   /* OLED仅看路（可重入） */
    SCRIPT_PRESET_24   = 24,   /* 过减速带(缓慢顶住) */
    SCRIPT_PRESET_25   = 25,   /* 过减速带（倒退顶住） */
    SCRIPT_PRESET_27   = 27,   /* 测试单动作色球抓取 */
    SCRIPT_PRESET_29   = 29,   /* 测试巡线走（不可跳入） */
    SCRIPT_PRESET_30   = 30,   /* 测试OpenMV补光 */
    SCRIPT_PRESET_31   = 31,   /* 测试纯陀螺仪巡线收敛（不可跳入） */
    SCRIPT_PRESET_32   = 32,   /* 测试平移射靶（可重入） */
    SCRIPT_PRESET_33   = 33,   /* 测试看路校准（不可跳入） */
    SCRIPT_PRESET_34   = 34,   /* 测试射靶保持（可重入） */
    SCRIPT_PRESET_35   = 35,   /* 测试补光灯 */
    SCRIPT_PRESET_36   = 36,   /* OLED看路转弯（可重入） */
    SCRIPT_PRESET_37   = 37,   /* 安全扫码 */
    SCRIPT_PRESET_39   = 39,   /* 常规扫码 */
    SCRIPT_PRESET_40   = 40,   /* 走左边 */
    SCRIPT_PRESET_41   = 41,   /* 走右边 */
    SCRIPT_PRESET_42   = 42,   /* 复位回起点 */
    SCRIPT_PRESET_43   = 43,   /* 执行任务一 */
    SCRIPT_PRESET_44   = 44,   /* 执行任务二 */
    SCRIPT_PRESET_45   = 45,   /* 执行任务三 */
    SCRIPT_PRESET_46   = 46,   /* 测试爆爆桶对位（可重入） */
    SCRIPT_PRESET_47   = 47,   /* 反向路线（不可跳入） */
    SCRIPT_PRESET_48   = 48,   /* 执行反向看路（可重入） */
    SCRIPT_PRESET_49   = 49,   /* 执行任务一(反向) */
    SCRIPT_PRESET_50   = 50,   /* 执行任务三（反向） */
    SCRIPT_PRESET_53   = 53,   /* 默认路线切换标靶和色球（不可跳入） */
    SCRIPT_PRESET_54   = 54,   /* 安全扫码_PLUS */
} emScriptPresetIdTdf;
#define SCRIPT_PRESET_NEXT_ID    55u
/* ==== 生成块: 结束 ==== */

/* ==================================================================
 * 预设注册表（map.c 定义, 控制台生成）
 * ================================================================== */

typedef struct
{
    uint8_t                ucId;      /* 稳定 id（= SCRIPT_PRESET_n, 0 表示未注册） */
    const char            *pcTitle;   /* 标题（= 头注释里的预设名） */
    const stScriptStepTdf *pstSteps;
    uint16_t               usCount;
    uint8_t                ucFlags;   /* 2026-10-18 加（**追加在末尾**, 4 字段老行缺省 = 0）:
                                       * bit0 = SCRIPT_PRESET_FLAG_REENTRANT（「可重入的任务」,
                                       * 预设栏勾选框写进注册表的第 5 列）—— STEP_SCRIPT_CALL
                                       * 调入这张表前先查机械臂展开标记, 没展开过就先阻塞
                                       * 播一遍「机械臂待命」（同 OLED「执行任务」的前置）。 */
} stScriptPresetTdf;

/* ucFlags 的位定义（map.c 注册表第 5 列, 控制台从「可重入」勾选状态生成） */
#define SCRIPT_PRESET_FLAG_REENTRANT  0x01u

extern const stScriptPresetTdf g_astScriptPreset[];
extern const uint16_t          g_usScriptPresetNum;

/* 表长度从表本身取, 不另存数字 —— 控制台生成的是这个表达式而不是字面量,
 * 所以手改 map.c 增删了步骤也不会与 usCount 脱节。
 * ⚠️ 这也正是注册表必须待在 map.c（数组的定义处）而不能搬进本头文件的原因:
 *    extern 声明的不完整数组类型 sizeof 不了。 */
#define SCRIPT_PRESET_COUNT(t)  ((uint16_t)(sizeof(t) / sizeof((t)[0])))

/* 外部只认这两个（当前跑的表与它的长度） */
extern const stScriptStepTdf *const g_pstScriptActive;
extern const uint16_t               g_usScriptActiveCount;

#endif /* __MAP_H */
