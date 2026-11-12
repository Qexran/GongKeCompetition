/**
  * @file       app_arm_action.h
  * @brief      机械臂「动作预设」表 —— 一段可命名的位姿/延时序列, 由路线里的
  *             STEP_ARM_PLAY_WAIT / _NO_WAIT 按 ID 引用。
  *
  *             数据表在 app_arm_action.c, 由 Web 控制台「机械臂」Tab 生成与维护
  *             （台架上拖滑杆调位姿 → 记录 → 保存 → 自动出现在「地图」积木库里）。
  *             本头文件的「生成块」与控制台的解析器是一对, 改格式要两边一起改。
  *             ⚠️ 聚合初始化里带了每个预设的播放方式（stArmActionTdf::ucNoWait）——
  *                控制台同时按它渲染数据区里那行「播放方式」注释, 两边不会漂移;
  *                改聚合行的字段顺序/个数要连控制台 arm_action.py 一起改。
  *
  *             播放复用比赛脚本同一批原语（vArmGroupMoveWait / vArmSetZhuaShou /
  *             vTaskDelay）, 所以台架上看到的运动与比赛跑路线时逐字节同一条路径。
  */

#ifndef __APP_ARM_ACTION_H
#define __APP_ARM_ACTION_H

#include <stdint.h>
#include "app_script.h"     /* stScriptStepTdf / SCRIPT_STEP_* 宏 */

/* ==================== 生成块 ====================
 * 下面这一段由 Web 控制台「机械臂」Tab **整段重写**, 手改会被下次保存覆盖。
 * 文件其余部分（结构体、API 声明）是手写的, 控制台不碰。 */
/* ==== 生成块: 开始 ==== */
/* 下一个可用 ID。只增不减、删除不回收 —— 保证已经发给控制台的 id 不会在下次
 * 保存后指向另一个动作（那会让路线里的引用于无声中换目标）。 */
#define ARM_ACTION_NEXT_ID    42u

/* 动作 ID: 路线表的 STEP_ARM_PLAY_WAIT / _NO_WAIT 用它引用动作; 控制台「动作」
 * 参数的下拉选项**取自本枚举**, 成员尾注释 = 用户起的标题。
 * 绑定条件 = app_script.c 里该步骤的 case 体写了 (emArmActionIdTdf)I(0)。
 *
 * ⚠️ 本枚举是**有意**被 server.py::parse_enums 扫到的（它的 glob 覆盖 Application
 *    下的头文件）。别因为「怕污染参数下拉」把它改成 #define —— 那样「动作」参数
 *    就没有下拉了, 只能手敲 ARM_ACT_n。 */
typedef enum {
    ARM_ACT_1 = 1,   /* 机械臂待命 */
    ARM_ACT_3 = 3,   /* 抓取最右边的球 */
    ARM_ACT_4 = 4,   /* 抓取中间的球 */
    ARM_ACT_5 = 5,   /* 抓取左边的球 */
    ARM_ACT_6 = 6,   /* 抓取最右边人质 */
    ARM_ACT_7 = 7,   /* 抓取中间人质 */
    ARM_ACT_8 = 8,   /* 抓取最左边人质 */
    ARM_ACT_9 = 9,   /* 识别小球 */
    ARM_ACT_10 = 10,   /* 看靶子 */
    ARM_ACT_11 = 11,   /* 复位回待命 */
    ARM_ACT_12 = 12,   /* 机械臂待命阻塞版本 */
    ARM_ACT_13 = 13,   /* 切换准备识别人质 */
    ARM_ACT_14 = 14,   /* 切换准备识别人质阻塞版本 */
    ARM_ACT_15 = 15,   /* 切换到识别标靶 */
    ARM_ACT_16 = 16,   /* 切换到识别标靶阻塞版本 */
    ARM_ACT_17 = 17,   /* 复位回待命阻塞版本 */
    ARM_ACT_19 = 19,   /* 机械臂待命到斜视二维码模式 */
    ARM_ACT_22 = 22,   /* 机械臂待命到正视二维码模式 */
    ARM_ACT_23 = 23,   /* 从正视二维码回收到初始位置 */
    ARM_ACT_26 = 26,   /* 看路 */
    ARM_ACT_27 = 27,   /* 机械臂待命到看路 */
    ARM_ACT_28 = 28,   /* 机械臂待命到看路阻塞版本 */
    ARM_ACT_29 = 29,   /* 抓取中间的球(单动作) */
    ARM_ACT_30 = 30,   /* 放置到爆爆桶(单动作) */
    ARM_ACT_31 = 31,   /* 动作25 */
    ARM_ACT_32 = 32,   /* 从抓取人质回到上电前位置 */
    ARM_ACT_33 = 33,   /* 切换准备识别人质(反向) */
    ARM_ACT_34 = 34,   /* 复位回待命(反向) */
    ARM_ACT_35 = 35,   /* 抓取中间的球(反向) */
    ARM_ACT_36 = 36,   /* 放置到爆爆桶(反向) */
    ARM_ACT_37 = 37,   /* 切换到识别标靶（反向） */
    ARM_ACT_38 = 38,   /* 抓取中间人质(反向) */
    ARM_ACT_40 = 40,   /* 放置人质到爆爆桶(人质单动作) */
    ARM_ACT_41 = 41,   /* 放置人质到爆爆桶(人质单动作反向) */
} emArmActionIdTdf;

/* 预设张数（= g_astArmAction[] 的长度, 控制台同步维护） */
#define ARM_ACTION_NUM        34u
/* ==== 生成块: 结束 ==== */

/* 一个动作预设: 稳定 ID + 标题 + 播放方式 + 那串步骤表 */
typedef struct
{
    uint8_t                ucId;      /* 稳定 ID（= 枚举值） */
    const char            *pcTitle;   /* 标题（也进积木库与路线表的标签） */
    uint8_t                ucNoWait;  /* 播放方式: 0=阻塞到位, 1=启动即返回。
                                       *
                                       * ⚠️ 这是**运行时唯一**的播放方式真相源（2026-09 加）:
                                       *    数据区里那行「播放方式」注释由控制台按本字段渲染,
                                       *    两者不会漂移。控制台生成聚合初始化时写它。
                                       *    路径里那块大积木（STEP_ARM_PLAY_WAIT/_NO_WAIT）不看它
                                       *    —— 那一步用自己的类型决定; 看它的是 ucArmPlayByMode()
                                       *    （排爆抓取 / 反恐任务内部按所选动作播那两处）。 */
    const stScriptStepTdf *pstSteps;
    uint16_t               usCount;
} stArmActionTdf;

extern const stArmActionTdf g_astArmAction[ARM_ACTION_NUM];

/* ==================== 播放 ====================
 * 两者内部都逐条走 vArmGroupMoveWait / vArmSetZhuaShou / vTaskDelay ——
 * 与 STEP_ARM_GROUP_WAIT / STEP_SET_ZHUASHOU_WAIT / STEP_DELAY 同一批函数。 */

/* 整段播放并阻塞到走完（含表内 STEP_DELAY）。路线里的 STEP_ARM_PLAY_WAIT
 * 与蓝牙的 $B,PLAY 都走这一条。ID 不存在则直接返回。仅任务上下文可调。
 * ⚠️ 播完会置 Beam 的「机械臂已展开」标志（仅当播的是待命序列, 见 .c 的 vArmNoteDeployed）。 */
void vArmPlayWait(emArmActionIdTdf emId);

/* 整段交给 vArmTask 后台推进, 立即返回。路线里的 STEP_ARM_PLAY_NO_WAIT 用它。
 * ⚠️ 同一时刻只允许一个后台序列: 再发一条会先终止上一条（**队列也一起清**）。 */
void vArmPlayNoWait(emArmActionIdTdf emId);

/* 延时投递（2026-10-08 加, STEP_ARM_PLAY_NO_WAIT 的 delay_ms 参数）:
 * ulDelayMs = 0 → 与 vArmPlayNoWait 完全同路径（立即入队, 老行为）;
 * ≠ 0 → 记「延时待投」槽位**立即返回**, 计时由 vArmTask 的泵在后台走, 到点才按
 *        vArmPlayNoWait 同款（先中止再入队）投出 —— **后续步骤不等它**（转弯/巡线照常先走）。
 * 精度: 臂空闲时 ≤20ms; 有序列在播时顺延到臂空（不掐正在播的）。
 * ⚠️ 后写者胜: 到点前再投一条, 旧的那条作废。⚠️ 任何 vArmSeqAbort()（新臂指令/脚本中止）
 *    或脚本中止电平都会**丢弃**未到点的投递 —— 按了停之后臂不许事后自己动。
 * ID 不存在则直接返回（同款静默）。仅任务上下文可调。 */
void vArmPlayNoWaitDelay(emArmActionIdTdf emId, uint32_t ulDelayMs);

/* 按**预设自己标的播放方式**播（2026-09 加）:
 *   ucNoWait == 0（阻塞到位）  → 先把后台序列（含待播队列）**等空**（不中止）再同步播完;
 *   ucNoWait == 1（启动即返回）→ **追加**到后台待播队列, 立即返回。
 * 用途: 排爆抓取 / 反恐任务里「播选中的那个动作」—— 那两块积木的阻塞与否应当由所选
 * 动作自己的模式决定, 而不是由积木名里的 _WAIT 决定。
 *
 * ⚠️ 与 vArmPlayNoWait 的区别只在「投递时要不要先中止」:
 *    这个函数是**追加**（让「抓取→复位」两条依次在后台跑完, 积木立刻返回）;
 *    那个函数是**后写者胜**（路线里一步顶掉前一步）。别把两者混用。
 * ⚠️ 阻塞那一档是真的**阻塞整个调用者**: 「③启动即返回 + ④阻塞到位」这组下, 路线
 *    仍然要等「抓取 + 复位」两段跑完（④ 的"到位"以它自己排队的位置为准, 不会去掐 ③）。
 * 返回 1 = 正常受理; 0 = 降级（待播队列满 / 等后台超时）——**调用方必须把它响出来**
 * （本文件没有蜂鸣/调试串口, 见 app_rpi.c 的用法）: 0 意味着「这条没排上」或
 * 「臂可能还在动、路线已经往下走了」, 静默通过比冻住更危险。
 * ID 不存在则返回 1（与 vArmPlayWait 同款静默）。仅任务上下文可调。 */
uint8_t ucArmPlayByMode(emArmActionIdTdf emId);

/* 终止正在跑的后台序列**并清空待播队列**（幂等）。
 * **任何新的臂指令都应先调它** —— 组动作/夹爪/两种播放/蓝牙点动都算,
 * 否则后台序列会继续写舵机, 与新指令抢。规则是「后写者胜 + 新指令显式终止旧序列」,
 * 于是不需要排表纪律: 排得不合理只是运动被截断, 行为确定且看得见。
 * ⚠️ 队列必须一起清（不然中止不掉: 泵过一会儿又把待播的取出来播了）。 */
void vArmSeqAbort(void);

/* vArmTask 的循环体: 有后台序列就阻塞推进它, 没有就让出。
 * 从 app_arm.c 的 vArmTask 里调用（那个任务原来是个空壳）。 */
void vArmActionPump(void);

uint8_t ucArmActionExists(emArmActionIdTdf emId);

#endif /* __APP_ARM_ACTION_H */
