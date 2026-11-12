/**
  * @file       map.c
  * @brief      路线表：小车整套流程的动作顺序集中在这一个文件里，一眼看清路线排布。
  *
  *             步骤类型与参数含义见 app_script.h 顶部注释；执行器 vScriptRun()
  *             在 app_script.c。改路线只改这张表，不动执行逻辑。
  *             每行末尾 // 注释 = 该步骤调用的原语函数（快速跳转看实现/改逻辑）。
  *
  *             方向约定（世界系, app_chassis.h）: +Y=启动前方, +X=启动左方。
 *
 *             支点转弯 STEP_PIVOT_TURN_WAIT: 只驱动一侧双轮、另一侧双轮持 0 当支点,
 *             靠陀螺仪闭环转到目标角。左转驱动右侧轮 2 与 4（1、3 当支点）, 右转驱动 1 与 3。
 *             角度参数无符号, 方向由 dir(1=左转 / 0=右转) 给; 标定宏 PIVOT_TURN_*（app_config.h）。
 *
 *             原地转弯 STEP_SPIN_TURN_WAIT: 四个麦轮对称反向出力, 绕**车身中心**自转
 *             （车心基本不动）, 同样靠陀螺仪闭环。参数与支点转弯同形（角度 + 方向位）,
 *             但两者**不可互换**: 支点转弯车心会平移, 且四轮自转是另一套增益（SPIN_TURN_*）。
 *
 *             重置脚本 STEP_SCRIPT_RESET: 无参, 回到**当前这张表**的第一条「实质」步骤 ——
 *             跳过表首连续的 STEP_WAIT_HANDSHAKE / STEP_DEAD（它们只在开机后第一次有意义）。
 *             排在表尾就是**自动循环**; 想在台架上每圈按一次键, 就把等键那步排在表**中间**
 *             （表首那份会被跳过, 中间那份照旧等键）。
 *             ⚠️ 它**不弹跳转栈**: 被跳进来的子预设里排它 = 该子预设自循环、永不返回。
 *                深度 0（主路线）时行为与旧版完全一致, 现有路线都不受影响。
 *
 *             条件分支 STEP_IF / STEP_ELSE / STEP_END_IF —— **就地嵌套, 不占预设**:
 *             三块标记在下面这张扁平表里靠**位置**配对: 「如果」开一层, 同层的「否则」把这一层
 *             切成两半, 同层的「结束如果」收口。可以任意层嵌套, 「否则」可省略（省略 = 条件不成立
 *             就直接往下走）。固件执行时不需要栈 —— 条件不成立就从下一条开始往后扫, 数着嵌套
 *             找到配对的那块标记跳过去（app_script.c 的 usScriptMatchMark）。
 *             ⚠️ 三块标记**必须成对**: 控制台保存前会拦（错误码 unbalanced_if）。不配对的表
 *                **C 编译期不报错** —— 固件扫到表尾就把本帧收掉, 表现成「后半张路线静默不执行」。
 *             ⚠️ 在控制台里只有「如果」能拖（拖一下自动落三行）; 「否则/结束如果」是结构行,
 *                不能单独拖/禁用/删除（删任一标记都会让后面整段的归属变掉）。
 *
 *             跳到别的预设 STEP_SCRIPT_CALL / STEP_SCRIPT_RETURN:
 *             跳转 = **调用**, 子预设跑到表尾自动跳回触发点的下一行; 也可以显式排「返回」提前跳回。
 *             ⚠️ 可被跳入的预设: **表首别放 STEP_WAIT_HANDSHAKE、表内别放 STEP_SCRIPT_RESET** ——
 *                前者会重跑握手（= 判为断开重连, 可能永久卡住）, 后者会让子预设永不返回。
 *                控制台会把这类预设的目标下拉标成「不可跳入」。
 *             ⚠️ 嵌套 IF 的分支**不是**调用: 不占 SCRIPT_CALL_DEPTH, 也不受上面那条约束。
 *
 * ------------------------------------------------------------------
 * 预设「测试射靶保持」(id 34, 表 `g_astScriptPreset34`) —— 2026-10-09 加。
 * ⚠️ 说明只能写在这里: **预设块内部的注释会被控制台重写掉**（它按「预设: X」那一行
 *    的等号花括号注释整段重生成标题行与表体）。
 *
 * ⚠️ 它由 OLED 第 5 页「射靶测试」的第 1 项启动（动作码 `OLED_ACT_SHOOT_TEST`,
 *    参数 = 本预设 id）, 走的是 `SCRIPT_REQ_TASK` 那条投递 —— 于是白送三样:
 *    二维码前置门 + 主摄预热 + 「机械臂没展开过就先阻塞播 ARM_ACT_1」。
 * ⚠️ **表首只放 ARM_ACT_16**（识别标靶位姿）: 待命/展开由上面那条兜住。
 *    **别在这里再排一块待命** —— 展开序列是"伸出→伸展→旋转", 臂已展开时重播会从
 *    伸展态再"伸出"一次, 那才有可能撞坏周围器件。
 * ⚠️ 最后一步 `STEP_TARGET_SHOOT_HOLD_WAIT` **永不自己结束**（只有 OLED 按「停止」
 *    → `vScriptAbort()` 能停）→ **后面排什么都跑不到**, 要加步骤只能加在它前面。
 * ⚠️ 不排激光: 激光开关是那一页的第 2 项, 由操作员手动控制。
  */

#include "map.h"
/* 路线位移变量（MOVE_*）是 app_config.h 的**编译期宏** —— 参数槽里写的就是宏名,
 * 而上面的包含链（app_script.h 等）全是纯枚举/类型, 够不到 app_config.h, 必须在这里直接包。
 * 删一个 MOVE_* 前先确认本表没有引用（控制台「位移」Tab 删除时会查）。 */
#include "app_config.h"
#include "app_cali.h"       /* CALI_METHOD_*（树莓派联调示例用） */
#include "app_rpi.h"        /* RPI_TASK_*（树莓派联调示例用） */
/* ⚠️ 用了 STEP_ARM_PLAY_WAIT/_NO_WAIT 就**必须**包含这个头: 那一步的参数类型
 *    emArmActionIdTdf（ARM_ACT_n）定义在这里, 而 app_script.h 只声明了步骤类型本身。
 *    少了它 → 编译报 `identifier "ARM_ACT_n" is undefined`（看着像代码写错了, 其实只是缺 include）。 */
#include "app_arm_action.h"
/* 同上: STEP_CHASSIS_ACTION_PLAY_WAIT 的参数类型 emChActIdTdf（CH_ACT_n）在这里。
 *    底盘动作组的数据表在 app_chassis_action.c（控制台「底盘动作组」Tab 生成）。 */
#include "app_chassis_action.h"

/* ==================================================================
 * 多预设路线表：每张表 = 一套完整路线, 文件末尾的 SCRIPT_PRESET_ACTIVE
 * 决定当前跑哪一张（控制台「地图」Tab 切换运行改的就是那一行）。
 * 速度 80rpm / 加速度 100 为保守首测值, 方向不对直接翻表内符号。
 * ================================================================== */

/* ===== 预设: 默认路线 ===== */
const stScriptStepTdf g_astScriptPreset1[] = {

    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 170u, 210u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_I1     (STEP_WAIT_HANDSHAKE,                  "等待握手", 0u),  // ucRpiHandshake()
    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热副摄", WARM_CAM_QR, 1u, 0u),  // vRpiWarmCamera()
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_TIMER,                           "计时", TIMER_CMD_START),  // vTimerCmd()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_DIAG_LF_WAIT, "斜线到出发区中央", MOVE_OUT_DIAG, 0u),  // vChassisMoveWorldDiag()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "安全扫码", SCRIPT_PRESET_54),  // vScriptCallPreset()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_111),  // vScriptIf()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "走右边", SCRIPT_PRESET_41),  // vScriptCallPreset()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "走左边", SCRIPT_PRESET_40),  // vScriptCallPreset()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前进", MOVE_ROAD_X, 0u),  // vChassisMoveWorldX()
    SCRIPT_STEP_I2     (STEP_ROAD_ALIGN_WAIT,                 "看路对位", 6000u, 1u),  // ucRpiRoadAlign()
    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "右转", 0.0f, 0u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "继续前进准备到灯泡", MOVE_BULB_Y, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_I2     (STEP_ROAD_ALIGN_WAIT,                 "看路对位", 6000u, 1u),  // ucRpiRoadAlign()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_115),  // vScriptIf()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_108),  // vScriptIf()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_33, 0u),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_13, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_108),  // vScriptIf()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_34, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_11),  // vArmPlayNoWait()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
//    SCRIPT_STEP_I2     (STEP_RPI_WARM_CAM,                    "预热摄像头: 主摄", WARM_CAM_MAIN, 1u),  // vRpiWarmCamera()
    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "右转", -90.0f, 0u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前往灯泡处", MOVE_LAMP_X),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -90.0f, 800u, 0u),  // ucRpiLineConverge()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "执行任务一", SCRIPT_PRESET_43),  // vScriptCallPreset()
//    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -90.0f, 1000u, 0u),  // ucRpiLineConverge()//    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "看路", ARM_ACT_26, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_11, 0),  // vArmPlayNoWaitDelay()
//    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_108),  // vScriptIf()
//    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_11),  // vArmPlayNoWait()
//    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热主摄+pi", WARM_CAM_MAIN_PI, 1u, 0),  // vRpiWarmCamera()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "走到激光半程", MOVE_LASER_X, 0u),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -90.0f, 500u, 0u),  // ucRpiLineConverge()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "切换到识别标靶", ARM_ACT_15, 0),  // vArmPlayNoWaitDelay()
//    SCRIPT_STEP_I1     (STEP_IF,                              "机械臂复位", SCRIPT_COND_SW_109),  // vScriptIf()
//    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "切换到识别标靶", ARM_ACT_15),  // vArmPlayNoWait()
//    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前往人像处", MOVE_FACE_X, 0u),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -90.0f, 1000u, 0u),  // ucRpiLineConverge()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "执行任务二", SCRIPT_PRESET_44),  // vScriptCallPreset()
//    SCRIPT_STEP_I1     (STEP_TARGET_SERVO_WAIT,               "标靶流控闭环: 起流→10Hz", 12000u),  // ucRpiTargetServo()
//    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热主摄", WARM_CAM_MAIN, 2u, 9u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "看路", ARM_ACT_26),  // vArmPlayNoWait()
//    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 80u, 150u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "离开激光准备转弯", MOVE_LEAVE_X, 1u),  // vChassisMoveWorldX()
    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 170u, 210u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_I2     (STEP_ROAD_ALIGN_WAIT,                 "看路对位", 6000u, 1u),  // ucRpiRoadAlign()
    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热主摄", WARM_CAM_MAIN, 1u, 2u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_115),  // vScriptIf()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_110),  // vScriptIf()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "切换准备识别人质", ARM_ACT_34, 0u),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "切换准备识别人质", ARM_ACT_11, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_110),  // vScriptIf()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "切换准备识别人质", ARM_ACT_33, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "切换准备识别人质", ARM_ACT_13),  // vArmPlayNoWait()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "右转", -180.0f, 0u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "走到人质", MOVE_HOSTAGE_Y, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "执行任务三", SCRIPT_PRESET_45),  // vScriptCallPreset()// ucRpiHostageGrabOne()
    SCRIPT_STEP_F2I1   (STEP_CHASSIS_MOVE_WORLD_WAIT,         "一步走到世界尽头", 0.0f, MOVE_END1_Y, 1u),  // vChassisMoveWorld()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -180.0f, 500u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP_F1I2   (STEP_SPIN_TURN_WAIT,                  "四轮绕中心原地自转", 1.0f, 0u, 0u),  // vPidSpinTurn()
    SCRIPT_STEP_F2I1   (STEP_CHASSIS_MOVE_WORLD_WAIT,         "一步走到世界尽头", 0.0f, MOVE_END2_Y, 0u),  // vChassisMoveWorld()
//    SCRIPT_STEP_F2I1   (STEP_CHASSIS_MOVE_WORLD_WAIT,         "一步走到世界尽头", 0.0f, -1640.0f, 1u),  // vChassisMoveWorld()
    SCRIPT_STEP_I1     (STEP_TIMER,                           "计时", TIMER_CMD_STOP),  // vTimerCmd()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "复位回起点", SCRIPT_PRESET_42),  // vScriptCallPreset()
    SCRIPT_STEP        (STEP_SCRIPT_RESET,                    "重置脚本"),  // ucScriptResetTarget()
};

/* ===== 预设: 测试巡线闭环 ===== */
const stScriptStepTdf g_astScriptPreset2[] = {
//    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 0.0f, 1000000u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 0.0f, 1000u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 0.0f, 1000u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 0.0f, 1000u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 0.0f, 1000u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 0.0f, 1000u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP        (STEP_LASER_ON,                        "打开激光（继电器 JIGUANG"),  // vRelayOn()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP        (STEP_LASER_OFF,                       "关闭激光"),  // vRelayOff()
//    SCRIPT_STEP_F1I1   (STEP_PIVOT_TURN_WAIT,                 "支点左转90度", 90.0f, 1),  // vPidPivotTurn()
//    SCRIPT_STEP_F1I1   (STEP_PIVOT_TURN_WAIT,                 "支点右转90度", 90.0f, 0),  // vPidPivotTurn()
};

/* ===== 预设: 测试减速带 ===== */
const stScriptStepTdf g_astScriptPreset3[] = {
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_F1I1   (STEP_CROSS_RAMP_WAIT,                 "过减速带: 低速顶住坎面被动对齐", 600.0f, 5000u),  // vChassisCrossRampWait()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
};

/* ===== 预设: 测试原地转弯 ===== */
/* 零代码标定用: 标 SPIN_TURN_*（app_config.h, 控制台「调参」Tab 改）时反复跑这个预设,
 * 不用改路线也不用重新烧录。看 OLED 第 4 行 Spd 的第一个数应为 0、第三个数是自转角速度。 */
const stScriptStepTdf g_astScriptPreset4[] = {
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_F1I1   (STEP_SPIN_TURN_WAIT,                  "原地左转90度", 90.0f, 1u),  // vPidSpinTurn()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_F1I1   (STEP_SPIN_TURN_WAIT,                  "原地右转90度", 90.0f, 0u),  // vPidSpinTurn()
    /* ↓ 世界航向交叉验证: 右转 90° 后车头已不朝世界 +Y, 这一步若**仍沿世界 +Y 平移**,
     *   就证明 vPidSpinTurn 结尾的 vSetWorldHeading 写对了（写错会斜着走或走反）。
     *   标定完可以删掉这两行。 */
    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "验证航向: 世界系前进300mm", 300.0f),  // vChassisMoveWorldY()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
};

/* ===== 预设: 测试脚本重置 ===== */
/* 每按一次 MAIN 键走一圈（世界系前进 300mm）—— 靠「表首 DEAD 被重置跳过、表中间 DEAD 照旧等键」
 * 这个特性做成: 圈数肉眼可数, 且不会跑飞。若第二圈不等键就直接又走 300mm, 说明跳过范围错了。 */
const stScriptStepTdf g_astScriptPreset5[] = {
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "第N圈: 世界系前进300mm", 300.0f),  // vChassisMoveWorldY()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP        (STEP_SCRIPT_RESET,                    "重置脚本: 回到本圈开头"),  // ucScriptResetTarget()
};

/* ===== 预设: 测试激光 ===== */
const stScriptStepTdf g_astScriptPreset6[] = {
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP        (STEP_LASER_ON,                        "打开激光（继电器 JIGUANG"),  // vRelayOn()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP        (STEP_LASER_OFF,                       "关闭激光"),  // vRelayOff()
};

/* ===== 预设: 测试机械臂 ===== */
const stScriptStepTdf g_astScriptPreset7[] = {
    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命", ARM_ACT_1),  // vArmPlayWait()
};

/* ===== 预设: 测试标靶 ===== */
/* 台架上演练打靶链，**不动车**：调试一个二维码 → 两次标靶校准（第一次是开环落点，
 * 第二次是拿第一次的结果迭代收敛，因为 TARGET_* 是**增量式**的）→ 激光射击。
 * 用控制台「地图」Tab 的「设备运行此预设」切到这一套即可（或改下面那行 #define）。 */
const stScriptStepTdf g_astScriptPreset8[] = {
    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP_I1     (STEP_QR_SET_TEST,                     "调试二维码 121 绿靶", 111u),  // vSetQrCode()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命阻塞版本", ARM_ACT_12),  // vArmPlayWait()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "切换到识别标靶", ARM_ACT_16),  // vArmPlayWait()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_TARGET_CALIB_WAIT,               "标靶校准 #1 开环落点", 6000u),  // vTargetCalibAim()
    SCRIPT_STEP_I1     (STEP_TARGET_CALIB_WAIT,               "标靶校准 #1 开环落点", 6000u),  // vTargetCalibAim()
    SCRIPT_STEP_I1     (STEP_TARGET_CALIB_WAIT,               "标靶校准 #1 开环落点", 6000u),  // vTargetCalibAim()
//    SCRIPT_STEP_I2     (STEP_TARGET_ALIGN_WAIT,               "标靶对位: 识别", 5000u, 1u),  // ucRpiTargetAlign()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP        (STEP_LASER_ON,                        "打开激光（继电器 JIGUANG"),  // vRelayOn()
//    SCRIPT_STEP_I2     (STEP_TEST_LOOP_WAIT,                  "反复执行选中的积木, 每轮等 M", STEP_TARGET_CALIB_WAIT, 1000u),  // vScriptTestLoop()
    SCRIPT_STEP_I1     (STEP_LASER_SHOOT,                     "激光射击 1000ms", 1000u),  // vLaserShoot()
//    SCRIPT_STEP_I1     (STEP_TARGET_CALIB_WAIT,               "标靶校准 #2 迭代收敛", 6000u),  // vTargetCalibAim()
    SCRIPT_STEP_I1     (STEP_DELAY,                           "沉淀延时", 500u),  // vTaskDelay()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
};

/* 现场标定/试跑用: 每按一次 MAIN 键做一轮完整对位（识别 → BALL_ALIGN_YAW_EN 开时
 * 转正+复测 → 中球偏差横/纵移）, 不抓球。失败音: 进「拉回（赌）」/整链终失败才响, 其余失败静音; 成功不响。
 * 标定顺序: ① 车摆理想停止位发一轮, 蓝牙 Tab 读 color2_x/y → 填 BALL_CX/CY_CENTER;
 *   ② 故意摆歪再跑, 方向反了翻 BALL_YAW_SIGN / BALL_SIDE_SIGN / BALL_FWD_SIGN;
 *   ③ 走多了/少了调 BALL_PX_TO_MM_X/Y 与 BALL_YAW_SCALE（垫转已知角读 ball_yaw 反算）。
 * ⚠️ 下面那行表头必须**紧贴**数组行 —— map_codec._locate_presets 只往回看 5 行。 */
/* ===== 预设: 测试小球对位 ===== */
const stScriptStepTdf g_astScriptPreset9[] = {
    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命阻塞版本", ARM_ACT_12),  // vArmPlayWait()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I4     (STEP_BALL_ALIGN_WAIT,                 "小球对位: 识别→", 6000u, ALIGN_BALL_MID, 0u, 0u),  // ucRpiAlignChain()
    SCRIPT_STEP_I2     (STEP_TEST_LOOP_WAIT,                  "反复执行选中的积木, 每轮等 M", STEP_BALL_ALIGN_WAIT, 1000u),  // vScriptTestLoop()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "复位回待命", ARM_ACT_11),  // vArmPlayWait()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "抓取左边的球", ARM_ACT_5),  // vArmPlayWait()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "复位回待命", ARM_ACT_11),  // vArmPlayWait()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "抓取中间的球", ARM_ACT_4),  // vArmPlayWait()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "复位回待命", ARM_ACT_11),  // vArmPlayWait()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "抓取最右边的球", ARM_ACT_3),  // vArmPlayWait()
//    SCRIPT_STEP        (STEP_ARM_EDIT_WAIT,                   "让位手动操纵: 停车+强制编辑模"),  // vBtArmEditHold()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
};

/* 台架上「反复测同一个积木」的沙盒: 把中间那块「测试」积木的**被测试的积木**下拉换成
 * 想测的那个, 每按一次 MAIN 跑一轮（两短声=本轮完, 三短声=跑满轮数后往下走）。
 * 默认测 1s 延时（不动车, 最安全）—— 轮数改成 0 就是不限次。
 * 例: 想反复试小球对位 → 下拉选「小球对位」(STEP_BALL_ALIGN_WAIT), 轮数 0,
 *     于是「对位 → 等键 → 对位 → 等键」不用再手工铺 N 行。
 * ⚠️ 别选「测试」自己和「重置脚本」: 固件里拒绝（爆栈 / 紧循环, 见 app_script.c）。
 * ⚠️ 新预设一律**排在文件最后**（预设表顺序 = 注册表/枚举顺序, 控制台按文件顺序解析;
 *    插在中间会让 id 顺序错位, test_server 的 TestPresetIds 会拦）。 */
/* ===== 预设: 测试积木 ===== */
const stScriptStepTdf g_astScriptPreset10[] = {
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I2     (STEP_TEST_LOOP_WAIT,                  "反复测: 1s延时(换下拉即换被测积木)", STEP_DELAY, 3u),  // vScriptTestLoop()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
};

/* 现场标定/试跑人质对位用: 每按一次 MAIN 键做一轮完整对位（发4识别 → 基准人质的
 * cx/cy 偏差 → 横/纵移）, 不抓人质。失败音: 识别失败/超时各响一声; 成功不响。
 * 标定顺序: ① 车摆理想停止位发一轮, 蓝牙 Tab「任务」组读 hostage{1,2,3}_x/y →
 *   把**中间那只**的数填进 HOSTAGE_CX_CENTER / HOSTAGE_CY_CENTER（中间那只是
 *   下面那块积木的默认基准, idx=1）;
 *   ② 故意摆歪再跑, 方向反了翻 HOSTAGE_SIDE_SIGN / HOSTAGE_FWD_SIGN
 *   （⚠️是独立的一套, 不是 BALL_SIDE_SIGN / BALL_FWD_SIGN）;
 *   ③ 走多了/少了调 HOSTAGE_PX_TO_MM_X/Y。
 * 四个量都能在蓝牙 Tab「人质对位」组当场改（零重编译）, 标好把终值写回 app_config.h 固化。
 * 想测左边/右边那只 → 把下面那块积木的「对位基准」下拉改掉;
 * 下面那块「测试」积木也可以改成别的积木反复跑（同预设 10 的用法）。
 * ⚠️ 下面那行表头必须**紧贴**数组行 —— map_codec._locate_presets 只往回看 5 行。
 * ⚠️ 新预设一律**排在文件最后**（预设表顺序 = 注册表/枚举顺序, 控制台按文件顺序解析;
 *    插在中间会让 id 顺序错位, test_server 的 TestPresetIds 会拦）。 */
/* ===== 预设: 测试人质对位 ===== */
const stScriptStepTdf g_astScriptPreset11[] = {
    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命阻塞版本", ARM_ACT_12),  // vArmPlayWait()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "切换准备识别人质阻塞版本", ARM_ACT_14),  // vArmPlayWait()
//    SCRIPT_STEP_I2     (STEP_TEST_LOOP_WAIT,                  "反复执行选中的积木, 每轮等 M", STEP_HOSTAGE_ALIGN_WAIT, 1000u),  // vScriptTestLoop()
//    SCRIPT_STEP_I2     (STEP_HOSTAGE_ALIGN_WAIT,              "人质对位: 中间那只为基准", 7500u, 1u),  // ucRpiHostageAlign()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_I2     (STEP_HOSTAGE_ALIGN_WAIT,              "人质对位: 中间那只为基准", 7500u, 1u),  // ucRpiHostageAlign()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_I2     (STEP_HOSTAGE_ALIGN_WAIT,              "人质对位: 中间那只为基准", 7500u, 1u),  // ucRpiHostageAlign()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_I2     (STEP_HOSTAGE_ALIGN_WAIT,              "人质对位: 中间那只为基准", 7500u, 1u),  // ucRpiHostageAlign()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I2     (STEP_HOSTAGE_ALIGN_WAIT,              "人质对位: 中间那只为基准", 7500u, 1u),  // ucRpiHostageAlign()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "抓取最左边人质", ARM_ACT_8),  // vArmPlayWait()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
};

/* 台架上演练「流控闭环」打靶链, 与预设 8（测试标靶 / 老方案）对照用, 不动车:
 * 调试二维码 → 机械臂待命/识别位姿 → 等 MAIN → [可选]仅识别预检 → 流控闭环
 * → [可选]终验+终补 → 激光。
 *
 * 三块新链路积木都可单独删, 删掉就是另一种编法:
 *   · 删「靶标识别(预检)」= 不预检, 直接闭环（Pi 端会自己锁定）;
 *   · 删「标靶校准(终验)」= 不要全分辨率终验, 闭环一收敛就开激光（最快）;
 *   · **想复现老方案** = 删掉流控闭环和预检, 换成连排三块标靶校准（那就是预设 8）。
 *
 * 跑之前先确认设备跑的是本预设（「地图」Tab 勾「设备运行此预设」）;
 * 调参顺序见 CLAUDE.md「标靶流控闭环」一节: tg_* 已有标定直接沿用 →
 * 不收敛/振荡先降 ts_kp_scale → 收敛太慢加 ts_step_deg → 误收再收紧 ts_conv_px/ticks。
 * 现场排障第一手: 蓝牙 Tab 看 aim_frames 涨不涨（停涨=流没来; 涨着但 aim_count=0
 * =流活着、Pi 没找到靶）, 以及 Pi 日志里的 TGTS ok/fail 行。
 * ⚠️ 下面那行表头必须**紧贴**数组行 —— map_codec._locate_presets 只往回看 5 行。
 * ⚠️ 新预设一律**排在文件最后**（预设表顺序 = 注册表/枚举顺序, 控制台按文件顺序解析;
 *    插在中间会让 id 顺序错位, test_server 的 TestPresetIds 会拦）。 */
/* ===== 预设: 测试标靶流控 ===== */
const stScriptStepTdf g_astScriptPreset12[] = {
    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP_I1     (STEP_QR_SET_TEST,                     "调试二维码 111 红靶", 111u),  // vSetQrCode()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命阻塞版本", ARM_ACT_12),  // vArmPlayWait()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "切换到识别标靶", ARM_ACT_16),  // vArmPlayWait()
    SCRIPT_STEP        (STEP_LASER_ON,                        "打开激光（继电器 JIGUANG"),  // vRelayOn()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_TARGET_SCAN_WAIT,                "靶标识别(预检, 可删)", 6000u),  // ucRpiTargetScan()
    SCRIPT_STEP_I2     (STEP_TEST_LOOP_WAIT,                  "反复执行选中的积木, 每轮等 M", STEP_TARGET_SERVO_WAIT, 1000u),  // vScriptTestLoop()
    SCRIPT_STEP_I1     (STEP_TARGET_SERVO_WAIT,               "标靶流控闭环", 10000u),  // ucRpiTargetServo()
//    SCRIPT_STEP_I1     (STEP_TARGET_CALIB_WAIT,               "终验+终补(可删)", 6000u),  // vTargetCalibAim()
//    SCRIPT_STEP_I1     (STEP_LASER_SHOOT,                     "激光射击 1000ms", 1000u),  // vLaserShoot()
    SCRIPT_STEP        (STEP_LASER_ON,                        "打开激光（继电器 JIGUANG"),  // vRelayOn()
    SCRIPT_STEP_I1     (STEP_DELAY,                           "沉淀延时", 500u),  // vTaskDelay()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
};

/* ===== 预设: OLED色球放置 ===== */
const stScriptStepTdf g_astScriptPreset14[] = {
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "复位回待命阻塞版本", ARM_ACT_17),  // vArmPlayWait()
//    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 0.0f, 1000u, 0u),  // ucRpiLineConverge()
    SCRIPT_STEP_F1I8   (STEP_BALL_GRAB_ONE_WAIT,              "色球单动作抓取: 只录'抓中间球", 0.0f, 7500u, 2u, ARM_ACT_29, ARM_ACT_30, 300u, 0u, 0u, 0u),  // ucRpiBallGrabOne()
//    SCRIPT_STEP_I4     (STEP_BALL_ALIGN_WAIT,                 "小球对位: 识别→", 6000u, ALIGN_BALL_MID, 0u, 0u),  // ucRpiAlignChain()
//    SCRIPT_STEP_I4     (STEP_BOMB_GRAB_WAIT,                  "排爆: 按二维码首位选球 → 播", ARM_ACT_5, ARM_ACT_4, ARM_ACT_3, ARM_ACT_11),  // ucRpiBombGrab()
};

/* ===== 预设: OLED激光射靶 ===== */
const stScriptStepTdf g_astScriptPreset15[] = {
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "切换到识别标靶阻塞版本", ARM_ACT_16),  // vArmPlayWait()
//    SCRIPT_STEP_I1     (STEP_TARGET_SCAN_WAIT,                "靶标识别", 6000u),  // ucRpiTargetScan()
//    SCRIPT_STEP_I1     (STEP_TARGET_SERVO_WAIT,               "标靶流控闭环: 起流→10Hz ", 10000u),  // ucRpiTargetServo()
    /* ⚠️ keep_res=0: 本预设只打靶、后面没有消费残留的世界系平移 —— 记了账留着
     *    会被之后任何一块勾"叠加残留"的平移**误吃**（不重上电就一直挂着）。 */
    SCRIPT_STEP_I2     (STEP_TARGET_SHOOT_WAIT,               "平移射靶", 10000u, 0u),  // ucRpiTargetShootAlign()
    SCRIPT_STEP_I1     (STEP_LASER_SHOOT,                     "激光射击: 开激光, 等一段时间", 1000u),  // vLaserShoot()
};

/* ⚠️ 表首那一步**必须是阻塞的** `STEP_ARM_PLAY_WAIT`: OLED「执行任务」跑本预设前会先发
 *    一条 `"C"` 预热主摄（app_script.c 的 SCRIPT_REQ_TASK 分支）, 唯一的预热窗口就是
 *    这段摆臂 —— 改成 _NO_WAIT 的话视觉指令几乎与预热同时到达 Pi, 预热等于白做
 *    （acquire 会原地等预热落地, 总耗时 ≈ 冷开, 实测就是这样）。 */
/* ===== 预设: OLED人质抓取 ===== */
const stScriptStepTdf g_astScriptPreset16[] = {
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "切换准备识别人质", ARM_ACT_33),  // vArmPlayWait()
//    SCRIPT_STEP_F1I9   (STEP_HOSTAGE_GRAB_ONE_WAIT,           "人质单动作抓取: 单个'抓中间'", 0.0f, 8000u, 1u, ARM_ACT_38, 0u, 700u, 1u, 0u, 0u, 0u),  // ucRpiHostageGrabOne()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "切换准备识别人质", ARM_ACT_13),  // vArmPlayWait()
    SCRIPT_STEP_F1I9   (STEP_HOSTAGE_GRAB_ONE_WAIT,           "人质单动作抓取: 单个'抓中间'", 0.0f, 8000u, 1u, ARM_ACT_7, 1u, 700u, 0u, 0u, 0u, 0u),  // ucRpiHostageGrabOne()
//    SCRIPT_STEP_I2     (STEP_HOSTAGE_ALIGN_WAIT,              "人质对位: 识别→按参数选定的基", 6000u, 1u),  // ucRpiHostageAlign()
//    SCRIPT_STEP_I3     (STEP_HOSTAGE_RESCUE_WAIT,             "反恐任务: 按二维码第三位选人质", ARM_ACT_8, ARM_ACT_7, ARM_ACT_6),  // ucRpiHostageRescue()
};

/* ===== 预设: 测试主摄像头识别二维码 ===== */
const stScriptStepTdf g_astScriptPreset19[] = {
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命到二维码模式", ARM_ACT_19),  // vArmPlayWait()
    SCRIPT_STEP_I2     (STEP_QR_SCAN_MAIN_WAIT,               "主摄二维码扫描: 同上, 但相机", 6000u, QR_ALGO_AUTO),  // ucRpiQrScanMain()
    SCRIPT_STEP        (STEP_STOP,                            "停车, 阻塞等 MAIN "),  // vScriptParkWaitKey()
};

/* ===== 预设: 测试支点顺畅转弯 ===== */
const stScriptStepTdf g_astScriptPreset22[] = {
    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_F2I7   (STEP_PIVOT_LINE_WAIT,                 "支点转弯→误差进阈值交给巡线", 90.0f, 0.0f, 0u, 0u, 100u, 0u, 1000u, 0u, 0u),  // ucRpiPivotTurnLine()
};

/* ===== 预设: OLED仅看路 ===== */
const stScriptStepTdf g_astScriptPreset23[] = {
//    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命阻塞版本", ARM_ACT_12),  // vArmPlayWait()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "看路", ARM_ACT_26),  // vArmPlayNoWait()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I2     (STEP_ROAD_ALIGN_WAIT,                 "看路对位: 主摄量黄色区左边缘→", 6000u, 1u),  // ucRpiRoadAlign()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", 90.0f, 0u, 0u),  // vPidPivotTurn()
};

/* ===== 预设: 过减速带(缓慢顶住) ===== */
const stScriptStepTdf g_astScriptPreset24[] = {
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_EMM_CTRL_MODE,                   "切换步进电机环模式", 1u),  // vChassisSetLoopModeAll()
    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "vChassisSetWorld", 5u, 8u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", 100.0f),  // vChassisMoveWorldY()
    SCRIPT_STEP_I1     (STEP_EMM_CTRL_MODE,                   "切换步进电机环模式", 2u),  // vChassisSetLoopModeAll()
    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "vChassisSetWorld", 80u, 100u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", 590.0f),  // vChassisMoveWorldY()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "回正", 0.0f, 1u, 1u),  // vPidPivotTurn()
};

/* ===== 预设: 过减速带（倒退顶住） ===== */
const stScriptStepTdf g_astScriptPreset25[] = {
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "vChassisSetWorld", 80u, 100u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "vChassisSetWorld", 80u, 100u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", MOVE_RAMP_Y, 0u),  // vChassisMoveWorldY()
//    SCRIPT_STEP_I1     (STEP_EMM_CTRL_MODE,                   "切换步进电机环模式", 1u),  // vChassisSetLoopModeAll()
//    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "vChassisSetWorld", 5u, 10u),  // vChassisSetWorldSpeedAccel()
//    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", -50.0f),  // vChassisMoveWorldY()
//    SCRIPT_STEP_I1     (STEP_EMM_CTRL_MODE,                   "切换步进电机环模式", 2u),  // vChassisSetLoopModeAll()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "回正", 0.0f, 1u, 1u),  // vPidPivotTurn()
};

/* ===== 预设: 测试单动作色球抓取 ===== */
/* ⚠️ 2026-10-06 加预热两步: 台架实测（04:17 那次 5s 内三色 0/3）的根因是「5」来时
 * 相机是冷的且租约刚被网页推流占过 —— 重开吃满了 5s 预算, 一帧都没采到。
 * 预热排在等键之前, 按一次 MAIN 的间隔正好给预热落地（≥2s）。 */
const stScriptStepTdf g_astScriptPreset27[] = {
    SCRIPT_STEP_I2     (STEP_RPI_WARM_CAM,                    "预热摄像头: 主摄", WARM_CAM_MAIN, 1u),  // vRpiWarmCamera()
    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_F1I8   (STEP_BALL_GRAB_ONE_WAIT,           "色球单动作抓取: 只录'抓中间球", 0.0f, 6000u, 1u, ARM_ACT_29, ARM_ACT_30, 0u, 0u, 0u, 0u),  // ucRpiBallGrabOne(); grab_lead=0=到位后即发(台架无平移, 等价老 0 延时)
};

/* ===== 预设: 测试巡线走 ===== */
const stScriptStepTdf g_astScriptPreset29[] = {
    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_F2I7   (STEP_PIVOT_LINE_WAIT,                 "支点转弯→误差进阈值交给巡线", 0.0f, 0.0f, 1u, 0u, 0u, 0u, 1500u, 0u, 0u),  // ucRpiPivotTurnLine()
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_F2I4   (STEP_LINE_STRAIGHT_WAIT,              "巡线直走", 1000.0f, 0.0f, 120u, 210u, 0u, 500u),  // ucRpiLineStraightWalk()
//    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "vChassisMoveWorl", 100.0f, 1u),  // vChassisMoveWorldX()
};

/* ===== 预设: 测试OpenMV补光 ===== */
const stScriptStepTdf g_astScriptPreset30[] = {
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP        (STEP_OPENMV_LED_TOGGLE,               "OpenMV 补光临时开关"),  // vRpiSendTaskCmd()
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP        (STEP_OPENMV_LED_TOGGLE,               "OpenMV 补光临时开关"),  // vRpiSendTaskCmd()
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
};

/* ===== 预设: 测试纯陀螺仪巡线收敛 ===== */
/* ⚠️ 2026-10-30: src 已提升为全局宏 LINE_CONV_SRC（出厂 3=融合）—— 本预设不再强制纯陀螺;
 *    想复现纯陀螺请把 LINE_CONV_SRC 临时改 1 重编译。 */
const stScriptStepTdf g_astScriptPreset31[] = {
    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 0.0f, 100000u, 0u),  // ucRpiLineConverge()
};

/* 平移射靶台架沙盒（2026-10-07 加, 第三套打靶方案: 激光固定在底盘、全程不动机械臂）。
 * 流程: 摆臂给相机位姿(ARM_ACT_16=识别标靶) → 平移射靶(粗调"6"沿排一次纵移 +
 * 精调"7"流帧位置模式闭环, keep_res=1 把累计纵移记进残留偏移) → 激光射击。
 * ⚠️ 轴是**车体纵移(前后)**: 靶排沿车体纵向排开, cx 偏差驱动 vChassisMoveCameraForward
 *    (同色球抓取沿排平移那根轴; 首版错用了横移, 已修)。
 * ⚠️ 标定前提: tc_cx_aim(瞄准列)与 tc_gap_px(每格像素, 发"6"看相邻靶 cx 差)都已在
 *    蓝牙「平移射靶」组标好, 否则第一步就拒绝动车(打 SHOT fail: aim not calibrated);
 *    打边靶走多大 = tc_gap_mm(粗调格距, 默认150, 可改160, 只影响粗调)。
 * ⚠️ 本预设**会动车**(沿排纵移可达 ±150mm+), 台架先架起轮子或留足场地。 */
/* ===== 预设: 测试平移射靶 ===== */
const stScriptStepTdf g_astScriptPreset32[] = {
//    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
//    SCRIPT_STEP_I1     (STEP_QR_SET_TEST,                     "调试二维码 111 红靶", 111u),  // vSetQrCode()
//    SCRIPT_STEP_I2     (STEP_RPI_WARM_CAM,                    "预热摄像头: 主摄", WARM_CAM_MAIN, 1u),  // vRpiWarmCamera()
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命阻塞版本", ARM_ACT_12),  // vArmPlayWait()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "切换到识别标靶", ARM_ACT_16),  // vArmPlayWait()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I2     (STEP_TARGET_SHOOT_WAIT,               "平移射靶: 粗调+精调对准", 10000u, 1u),  // ucRpiTargetShootAlign()
    SCRIPT_STEP_I1     (STEP_LASER_SHOOT,                     "激光射击 1000ms", 1000u),  // vLaserShoot()
//    SCRIPT_STEP        (STEP_DEAD,                            "不动车, 等 MAIN 按键继续"),  // vScriptWaitKeyConfirm()
};

/* ===== 预设: 测试看路校准 ===== */
const stScriptStepTdf g_astScriptPreset33[] = {
    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命阻塞版本", ARM_ACT_12),  // vArmPlayWait()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "看路", ARM_ACT_26, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_STOP,                            "停车, 阻塞等 MAIN"),  // vScriptParkWaitKey()
    SCRIPT_STEP_I1     (STEP_ROAD_YAW_WAIT,                   "看路校准: 拟合黄区直边图像方向", 6000u),  // ucRpiRoadYaw()
    SCRIPT_STEP        (STEP_STOP,                            "停车, 阻塞等 MAIN"),  // vScriptParkWaitKey()
    SCRIPT_STEP_F1I2   (STEP_SPIN_TURN_WAIT,                  "四轮绕中心原地自转", 0.0f, 1u, 1u),  // vPidSpinTurn()
};

/* ===== 预设: 测试射靶保持 ===== */
const stScriptStepTdf g_astScriptPreset34[] = {
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "切换到识别标靶", ARM_ACT_16),  // vArmPlayWait()
    SCRIPT_STEP        (STEP_TARGET_SHOOT_HOLD_WAIT,          "平移射靶保持: 跟到瞄准列为止"),  // ucRpiTargetShootHold()
};

/* ===== 预设: 测试补光灯 ===== */
const stScriptStepTdf g_astScriptPreset35[] = {
//    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP        (STEP_BUGUANG_KAI,                     "打开补光灯"),  // vFillLightSet()
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP        (STEP_BUGUANG_GUAN,                    "关闭补光灯"),  // vFillLightSet()
};

/* ===== 预设: OLED看路转弯 ===== */
const stScriptStepTdf g_astScriptPreset36[] = {
//    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命阻塞版本", ARM_ACT_12),  // vArmPlayWait()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "看路", ARM_ACT_26),  // vArmPlayNoWait()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I3     (STEP_ROAD_ALIGN_WAIT,                 "看路对位: 主摄量黄色区左边缘→", 6000u, 1u, 0u),  // ucRpiRoadAlign()
    SCRIPT_STEP_F1I3   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", 90.0f, 0u, 0u, 0u),  // vPidPivotTurn()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
//    SCRIPT_STEP_I3     (STEP_ROAD_ALIGN_WAIT,                 "看路对位: 主摄量黄色区左边缘→", 6000u, 1u, 1u),  // ucRpiRoadAlign()
//    SCRIPT_STEP_F1I3   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", 90.0f, 1u, 1u, 1u),  // vPidPivotTurn()
};

/* ===== 预设: 安全扫码 ===== */
const stScriptStepTdf g_astScriptPreset37[] = {
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "一步前进到二维码位", MOVE_QR_SAFE_Y, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_I2     (STEP_QR_SCAN_WAIT,                    "副摄二维码扫描", 5000u, QR_ALGO_AUTO),  // ucRpiQrScan()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_QR),  // vScriptIf()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I2     (STEP_RPI_WARM_CAM,                    "预热摄像头", WARM_CAM_MAIN, 1u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命到正视二维码模式", ARM_ACT_22),  // vArmPlayWait()
    SCRIPT_STEP_I2     (STEP_QR_SCAN_MAIN_WAIT,               "主摄二维码扫描", 8000u, QR_ALGO_AUTO),  // ucRpiQrScanMain()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "从正视二维码回收到初始位置", ARM_ACT_23, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
//    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "退回准备转弯", -180.0f, 0u),  // vChassisMoveWorldY()
};

/* ===== 预设: 常规扫码 ===== */
const stScriptStepTdf g_astScriptPreset39[] = {
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", MOVE_QR_NORM_Y, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_I2     (STEP_QR_SCAN_WAIT,                    "副摄二维码扫描", 5000u, QR_ALGO_AUTO),  // ucRpiQrScan()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_QR),  // vScriptIf()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I2     (STEP_RPI_WARM_CAM,                    "预热摄像头", WARM_CAM_QR, 1u),  // vRpiWarmCamera()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "再靠近一点", MOVE_QR_NEAR_Y, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_I2     (STEP_QR_SCAN_WAIT,                    "副摄二维码扫描", 5000u, QR_ALGO_AUTO),  // ucRpiQrScan()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_QR),  // vScriptIf()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I2     (STEP_RPI_WARM_CAM,                    "预热摄像头", WARM_CAM_MAIN, 1u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命到正视二维码模式", ARM_ACT_22),  // vArmPlayWait()
    SCRIPT_STEP_I2     (STEP_QR_SCAN_MAIN_WAIT,               "主摄二维码扫描", 8000u, QR_ALGO_AUTO),  // ucRpiQrScanMain()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "从正视二维码回收到初始位置", ARM_ACT_23, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "退回准备转弯", MOVE_QR_BACK_Y, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
};

/* ===== 预设: 走左边 ===== */
const stScriptStepTdf g_astScriptPreset40[] = {
    SCRIPT_STEP_F2I7   (STEP_SPIN_LINE_WAIT,                  "四轮绕中心原地自转", 90.0f, 90.0f, 1u, 1u, 0u, 1000u, 0u, 0u, 0u),  // ucRpiSpinTurnLine()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", 25.0f, 0u),  // vChassisMoveWorldY()
//    SCRIPT_STEP_F1I2   (STEP_SPIN_TURN_WAIT,                  "四轮绕中心原地自转", 90.0f, 1u, 1u),  // vPidSpinTurn()
//    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", 90.0f, 1u, 0u),  // vPidPivotTurn()
//    SCRIPT_STEP_F2I7   (STEP_PIVOT_LINE_WAIT,                 "支点转弯", 90.0f, 90.0f, 1u, 1u, 0u, 500u, 0u, 0u, 0u),  // ucRpiPivotTurnLine()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前进", MOVE_LEFT_X, 0u),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "右转弯准备过减速带", 0.0f, 0u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热摄像头", WARM_CAM_MAIN, 3u, 5u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "过减速带", SCRIPT_PRESET_25),  // vScriptCallPreset()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "机械臂待命到看路", ARM_ACT_27, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_I1     (STEP_ARM_DEPLOY_MARK,                 "改写「机械臂已展开」标记", 1u),  // vSetArmDeployed()
    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 180u, 210u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_F2I7   (STEP_PIVOT_LINE_WAIT,                 "支点转弯", 90.0f, 90.0f, 1u, 1u, 0u, 1000u, 0u, 0u, 0u),  // ucRpiPivotTurnLine()
//    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 二维码识别结果", SCRIPT_COND_QR),  // vScriptIf()
//    SCRIPT_STEP        (STEP_ELSE,                            "否则"),  // vScriptElse()
//    SCRIPT_STEP_I2     (STEP_RPI_WARM_CAM,                    "预热摄像头: 主摄", WARM_CAM_MAIN, 1u),  // vRpiWarmCamera()
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "机械臂待命到斜视二维码模式", ARM_ACT_19),  // vArmPlayNoWait()
//    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "退到主摄扫码位", -660.0f),  // vChassisMoveWorldX()
//    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "微调前后位置", -30.0f),  // vChassisMoveWorldY()
//    SCRIPT_STEP_I2     (STEP_QR_SCAN_MAIN_WAIT,               "主摄二维码扫描: 自研算法", 10000u, QR_ALGO_DIFF),  // ucRpiQrScanMain()
//    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 二维码识别结果", SCRIPT_COND_QR),  // vScriptIf()
//    SCRIPT_STEP        (STEP_ELSE,                            "否则"),  // vScriptElse()
//    SCRIPT_STEP_I2     (STEP_QR_SCAN_MAIN_WAIT,               "主摄二维码扫描: 自研算法加长", 15000u, QR_ALGO_DIFF),  // ucRpiQrScanMain()
//    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 二维码识别结果", SCRIPT_COND_QR),  // vScriptIf()
//    SCRIPT_STEP        (STEP_ELSE,                            "否则"),  // vScriptElse()
//    SCRIPT_STEP        (STEP_QR_SET_RANDOM,                   "随机二维码（不经过 Pi）"),  // vSetQrCodeRandom()
//    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
//    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_11),  // vArmPlayNoWait()
//    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "前后归位", 30.0f),  // vChassisMoveWorldY()
//    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "横向归位", 660.0f),  // vChassisMoveWorldX()
//    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛: 回到线上", 90.0f, 1000u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
};

/* ===== 预设: 走右边 ===== */
const stScriptStepTdf g_astScriptPreset41[] = {
    SCRIPT_STEP_F2I7   (STEP_SPIN_LINE_WAIT,                  "原地转弯→误差进阈值交给巡线", -90.0f, -90.0f, 0u, 1u, 0u, 0u, 0u, 0u, 0u),  // ucRpiSpinTurnLine()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", 20.0f, 0u),  // vChassisMoveWorldY()
//    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", -90.0f, 0u, 1u),  // vPidPivotTurn()
//    SCRIPT_STEP_F2I7   (STEP_PIVOT_LINE_WAIT,                 "支点转弯", -90.0f, -90.0f, 0u, 1u, 0u, 0u, 0u, 0u, 0u),  // ucRpiPivotTurnLine()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前进", MOVE_RIGHT_X, 0u),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "左转弯准备过减速带", 0.0f, 1u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热摄像头", WARM_CAM_MAIN, 3u, 5u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "过减速带", SCRIPT_PRESET_25),  // vScriptCallPreset()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "机械臂待命到看路", ARM_ACT_27, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_I1     (STEP_ARM_DEPLOY_MARK,                 "改写「机械臂已展开」标记", 1u),  // vSetArmDeployed()
    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 170u, 210u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_F2I7   (STEP_PIVOT_LINE_WAIT,                 "支点转弯", 90.0f, 90.0f, 1u, 1u, 0u, 0u, 0u, 0u, 0u),  // ucRpiPivotTurnLine()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前进", MOVE_RIGHT2_X, 0u),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环", 90.0f, 500u, 0u),  // ucRpiLineConverge()
};

/* ===== 预设: 复位回起点 ===== */
const stScriptStepTdf g_astScriptPreset42[] = {
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键回到原点"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_TIMER,                           "计时", TIMER_CMD_RESET),  // vTimerCmd()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位机械臂", ARM_ACT_32, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 50u, 100u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "回到出发区", MOVE_HOME_X),  // vChassisMoveWorldX()
    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 180u, 250u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_F1I2   (STEP_SPIN_TURN_WAIT,                  "转回原点", 0.0f, 0u, 1u),  // vPidSpinTurn()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 0.0f, 600u, 0u),  // ucRpiLineConverge()
};

/* ===== 预设: 执行任务一 ===== */
const stScriptStepTdf g_astScriptPreset43[] = {
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_115),  // vScriptIf()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_108),  // vScriptIf()
    SCRIPT_STEP_F1I9   (STEP_HOSTAGE_GRAB_ONE_WAIT,           "人质单动作抓取: 单个'抓中间'", -90.0f, 6000u, 1u, ARM_ACT_38, 1u, 800u, 1u, 0u, ARM_ACT_41, 0u),  // ucRpiHostageGrabOne()
//    SCRIPT_STEP_F1I8   (STEP_BALL_GRAB_ONE_WAIT,              "色球单动作抓取", -90.0f, 7500u, 2u, ARM_ACT_35, ARM_ACT_36, 800u, 0u, 1u, 0u),  // ucRpiBallGrabOne()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_F1I9   (STEP_HOSTAGE_GRAB_ONE_WAIT,           "人质单动作抓取: 单个'抓中间'", -90.0f, 6000u, 1u, ARM_ACT_7, 1u, 800u, 0u, 0u, ARM_ACT_40, 0u),  // ucRpiHostageGrabOne()
//    SCRIPT_STEP_F1I8   (STEP_BALL_GRAB_ONE_WAIT,              "色球单动作抓取", -90.0f, 7500u, 2u, ARM_ACT_29, ARM_ACT_30, 800u, 0u, 0u, 0u),  // ucRpiBallGrabOne()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_108),  // vScriptIf()
    SCRIPT_STEP_F1I8   (STEP_BALL_GRAB_ONE_WAIT,              "色球单动作抓取", -90.0f, 7500u, 2u, ARM_ACT_35, ARM_ACT_36, 800u, 0u, 1u, 0u),  // ucRpiBallGrabOne()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_F1I8   (STEP_BALL_GRAB_ONE_WAIT,              "色球单动作抓取", -90.0f, 7500u, 2u, ARM_ACT_29, ARM_ACT_30, 800u, 0u, 0u, 0u),  // ucRpiBallGrabOne()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
};

/* ===== 预设: 执行任务二 ===== */
const stScriptStepTdf g_astScriptPreset44[] = {
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_109),  // vScriptIf()
    SCRIPT_STEP_F1I2   (STEP_SPIN_TURN_WAIT,                  "四轮绕中心原地自转", 90.0f, 1u, 1u),  // vPidSpinTurn()
    SCRIPT_STEP_I3     (STEP_TARGET_SHOOT_WAIT,               "平移射靶", 12000u, 1u, 0u),  // ucRpiTargetShootAlign()
    SCRIPT_STEP_I1     (STEP_LASER_SHOOT,                     "激光射击", 800u),  // vLaserShoot()
    SCRIPT_STEP_F1I2   (STEP_SPIN_TURN_WAIT,                  "四轮绕中心原地自转", -90.0f, 0u, 1u),  // vPidSpinTurn()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I2     (STEP_TARGET_SHOOT_WAIT,               "平移射靶", 12000u, 1u),  // ucRpiTargetShootAlign()
    SCRIPT_STEP_I1     (STEP_LASER_SHOOT,                     "激光射击", 800u),  // vLaserShoot()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
};

/* ===== 预设: 执行任务三 ===== */
const stScriptStepTdf g_astScriptPreset45[] = {
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_115),  // vScriptIf()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_110),  // vScriptIf()
    SCRIPT_STEP_F1I8   (STEP_BALL_GRAB_ONE_WAIT,              "色球单动作抓取: 只录'抓中间球", -180.0f, 6000u, 2u, ARM_ACT_35, 0u, 700u, 0u, 1u, 0u),  // ucRpiBallGrabOne()
//    SCRIPT_STEP_F1I9   (STEP_HOSTAGE_GRAB_ONE_WAIT,           "人质单动作抓取", -180.0f, 8000u, 1u, ARM_ACT_38, 1u, 800u, 1u, 0u, 0u, 0u),  // ucRpiHostageGrabOne()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_F1I8   (STEP_BALL_GRAB_ONE_WAIT,              "色球单动作抓取: 只录'抓中间球", -180.0f, 6000u, 2u, ARM_ACT_29, 0u, 700u, 0u, 0u, 0u),  // ucRpiBallGrabOne()
//    SCRIPT_STEP_F1I9   (STEP_HOSTAGE_GRAB_ONE_WAIT,           "人质单动作抓取", -180.0f, 8000u, 1u, ARM_ACT_7, 1u, 800u, 0u, 0u, 0u, 0u),  // ucRpiHostageGrabOne()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_110),  // vScriptIf()
    SCRIPT_STEP_F1I9   (STEP_HOSTAGE_GRAB_ONE_WAIT,           "人质单动作抓取", -180.0f, 8000u, 1u, ARM_ACT_38, 1u, 800u, 1u, 0u, 0u, 0u),  // ucRpiHostageGrabOne()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_F1I9   (STEP_HOSTAGE_GRAB_ONE_WAIT,           "人质单动作抓取", -180.0f, 8000u, 1u, ARM_ACT_7, 1u, 800u, 0u, 0u, 0u, 0u),  // ucRpiHostageGrabOne()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
};

/* 爆爆桶对位台架沙盒（2026-10-24 加）。
 * 流程: 等握手 → 爆爆桶对位（发"J"起流 + 0x0B 流帧纵移闭环 → 发"K"停流）。
 * ⚠️ 标定前提: bc_cx_aim(瞄准列)已在蓝牙「爆爆桶」组标好 —— 没标的话本步自动进
 *    **只测不改**模式（起流采 ~3s, "BKT meas:" 行报中位 cx + 蓝牙遥测 bucket_err,
 *    一格不动）: 车摆到理想推桶位姿跑一次本预设 → 读数填 bc_cx_aim → 再跑就是闭环。
 *    bc_px_to_mm 是占位值, 第一次闭环前按"已知位移实测"重标（见 app_config.h 段头）。
 * ⚠️ 本预设**会动车**（纵移闭环, 行程闸 bc_travel_mm=150 兜底）, 台架留足场地。 */
/* ===== 预设: 测试爆爆桶对位 ===== */
const stScriptStepTdf g_astScriptPreset46[] = {
    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP_I2     (STEP_BARREL_ALIGN_WAIT,               "爆爆桶对位: 流帧纵移闭环", 20000u, 0u),  // ucRpiBarrelAlign()
};

/* ===== 预设: 反向路线 ===== */
const stScriptStepTdf g_astScriptPreset47[] = {

    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 180u, 210u),  // vChassisSetWorldSpeedAccel()
//    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
    SCRIPT_STEP        (STEP_WAIT_HANDSHAKE,                  "等待握手"),  // ucRpiHandshake()
//    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热副摄", WARM_CAM_QR, 1u, 0u),  // vRpiWarmCamera()
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热摄像头", WARM_CAM_MAIN_PI, 2u, 0u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_TIMER,                           "计时", TIMER_CMD_START),  // vTimerCmd()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_DIAG_LF_WAIT, "斜线到出发区中央", 90.0f, 2u),  // vChassisMoveWorldDiag()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", -800.0f, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 0.0f, 500u, 0u),  // ucRpiLineConverge()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", -800.0f, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 0.0f, 600u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "vArmPlayNoWait", ARM_ACT_27, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_I1     (STEP_ARM_DEPLOY_MARK,                 "改写「机械臂已展开」标记", 1u),  // vSetArmDeployed()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", -750.0f, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_QR),  // vScriptIf()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_QR_SET_TEST,                     "调试二维码（测试用, 每位 1~", 321u),  // vSetQrCode()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I3     (STEP_ROAD_ALIGN_WAIT,                 "看路对位: 主摄量黄色区左边缘→", 6000u, 1u, 1u),  // ucRpiRoadAlign()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "切换到识别标靶", ARM_ACT_15, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_F1I3   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", 90.0f, 1u, 1u, 1u),  // vPidPivotTurn()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "靠近激光", MOVE_LEAVE_X, 1u),  // vChassisMoveWorldX()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 90.0f, 600u, 0u),  // ucRpiLineConverge()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "执行任务二", SCRIPT_PRESET_44),  // vScriptCallPreset()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 90.0f, 600u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_11, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前往人像处", -600.0f, 1u),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 90.0f, 1000u, 0u),  // ucRpiLineConverge()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "走到激光半程", -1090.0f, 0u),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", 90.0f, 1000u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "执行任务一", SCRIPT_PRESET_49),  // vScriptCallPreset()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "看路", ARM_ACT_26),  // vArmPlayNoWait()
    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前往灯泡处", MOVE_LAMP_X),  // vChassisMoveWorldX()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I3     (STEP_ROAD_ALIGN_WAIT,                 "看路对位", 6000u, 1u, 1u),  // ucRpiRoadAlign()
    SCRIPT_STEP_F1I3   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", 180.0f, 1u, 1u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "继续前进准备到灯泡", MOVE_BULB_Y, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_I3     (STEP_ROAD_ALIGN_WAIT,                 "看路对位", 6000u, 1u, 1u),  // ucRpiRoadAlign()
    SCRIPT_STEP_F1I3   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", -90.0f, 1u, 1u, 1u),  // vPidPivotTurn()
//    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前进", MOVE_ROAD_X, 0u),  // vChassisMoveWorldX()
//    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "安全扫码", SCRIPT_PRESET_37),  // vScriptCallPreset()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "切换准备识别人质", ARM_ACT_33, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", -90.0f, 1000u, 0u),  // ucRpiLineConverge()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "走到人质", 840.0f, 0u),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", -20.0f, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "执行任务三", SCRIPT_PRESET_50),  // vScriptCallPreset()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "离开人质", 450.0f, 1u),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I3   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", -180.0f, 0u, 1u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "vChassisSetWorld", 80u, 100u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", 665.0f, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_F1I3   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", 90.0f, 0u, 1u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "准备回到起点", -470.0f, 0u),  // vChassisMoveWorldX()
//    SCRIPT_STEP_F1I2   (STEP_SPIN_TURN_WAIT,                  "四轮绕中心原地自转", 0.0f, 1u, 0u),  // vPidSpinTurn()
//    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛闭环: 发LINE_ST", -90.0f, 500u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP_F1I3   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", 180.0f, 1u, 1u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "vChassisMoveWorl", 600.0f, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_111),  // vScriptIf()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "走右边", SCRIPT_PRESET_41),  // vScriptCallPreset()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "走左边", SCRIPT_PRESET_40),  // vScriptCallPreset()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_I2     (STEP_ROAD_ALIGN_WAIT,                 "看路对位", 6000u, 1u),  // ucRpiRoadAlign()
    SCRIPT_STEP_F1I3   (STEP_PIVOT_TURN_WAIT,                 "右转", 0.0f, 0u, 1u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_108),  // vScriptIf()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_34, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_11),  // vArmPlayNoWait()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
//    SCRIPT_STEP_I2     (STEP_RPI_WARM_CAM,                    "预热摄像头: 主摄", WARM_CAM_MAIN, 1u),  // vRpiWarmCamera()
    SCRIPT_STEP_F1I3   (STEP_PIVOT_TURN_WAIT,                 "右转", -90.0f, 0u, 1u, 1u),  // vPidPivotTurn()
//    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -90.0f, 1000u, 0u),  // ucRpiLineConverge()//    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "看路", ARM_ACT_26, 0),  // vArmPlayNoWaitDelay()
//    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_108),  // vScriptIf()
//    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_11),  // vArmPlayNoWait()
//    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
//    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热主摄+pi", WARM_CAM_MAIN_PI, 1u, 0),  // vRpiWarmCamera()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -90.0f, 500u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP_I1     (STEP_IF,                              "机械臂复位", SCRIPT_COND_SW_109),  // vScriptIf()
//    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "切换到识别标靶", ARM_ACT_15),  // vArmPlayNoWait()
//    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -90.0f, 1000u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP_I1     (STEP_TARGET_SERVO_WAIT,               "标靶流控闭环: 起流→10Hz", 12000u),  // ucRpiTargetServo()
//    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热主摄", WARM_CAM_MAIN, 2u, 9u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_TIMER,                           "计时", TIMER_CMD_STOP),  // vTimerCmd()
//    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 80u, 150u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 170u, 210u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_I2     (STEP_ROAD_ALIGN_WAIT,                 "看路对位", 6000u, 1u),  // ucRpiRoadAlign()
    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热主摄", WARM_CAM_MAIN, 1u, 2u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_110),  // vScriptIf()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "切换准备识别人质", ARM_ACT_33, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "右转", -180.0f, 0u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_F2I1   (STEP_CHASSIS_MOVE_WORLD_WAIT,         "一步走到世界尽头", 0.0f, MOVE_END1_Y, 1u),  // vChassisMoveWorld()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -180.0f, 500u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP_F1I2   (STEP_SPIN_TURN_WAIT,                  "四轮绕中心原地自转", 1.0f, 0u, 0u),  // vPidSpinTurn()
    SCRIPT_STEP_F2I1   (STEP_CHASSIS_MOVE_WORLD_WAIT,         "一步走到世界尽头", 0.0f, MOVE_END2_Y, 0u),  // vChassisMoveWorld()
//    SCRIPT_STEP_F2I1   (STEP_CHASSIS_MOVE_WORLD_WAIT,         "一步走到世界尽头", 0.0f, -1640.0f, 1u),  // vChassisMoveWorld()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "复位回起点", SCRIPT_PRESET_42),  // vScriptCallPreset()
    SCRIPT_STEP        (STEP_SCRIPT_RESET,                    "重置脚本"),  // ucScriptResetTarget()
};

/* ===== 预设: 执行反向看路 ===== */
const stScriptStepTdf g_astScriptPreset48[] = {
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "看路", ARM_ACT_26, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_I3     (STEP_ROAD_ALIGN_WAIT,                 "看路对位: 主摄量黄色区左边缘→", 6000u, 1u, 1u),  // ucRpiRoadAlign()
    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "单侧双轮支点转弯", 90.0f, 0u, 1u),  // vPidPivotTurn()
};

/* ===== 预设: 执行任务一(反向) ===== */
const stScriptStepTdf g_astScriptPreset49[] = {
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_108),  // vScriptIf()
    SCRIPT_STEP_F1I8   (STEP_BALL_GRAB_ONE_WAIT,              "色球单动作抓取", 90.0f, 7500u, 2u, ARM_ACT_35, ARM_ACT_36, 700u, 0u, 1u, 0u),  // ucRpiBallGrabOne()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_F1I8   (STEP_BALL_GRAB_ONE_WAIT,              "色球单动作抓取", 90.0f, 7500u, 2u, ARM_ACT_29, ARM_ACT_30, 700u, 0u, 0u, 0u),  // ucRpiBallGrabOne()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
};

/* ===== 预设: 执行任务三（反向） ===== */
const stScriptStepTdf g_astScriptPreset50[] = {
//    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_110),  // vScriptIf()
//    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
//    SCRIPT_STEP_F1I9   (STEP_HOSTAGE_GRAB_ONE_WAIT,           "人质单动作抓取", -90.0f, 8000u, 1u, ARM_ACT_7, 1u, 800u, 0u, 0u, 0u, 0u),  // ucRpiHostageGrabOne()
//    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_F1I9   (STEP_HOSTAGE_GRAB_ONE_WAIT,           "人质单动作抓取", -90.0f, 8000u, 0u, ARM_ACT_38, 1u, 800u, 1u, 0u, 0u, 0u),  // ucRpiHostageGrabOne()
};

/* ===== 预设: 默认路线切换标靶和色球 ===== */
const stScriptStepTdf g_astScriptPreset53[] = {

    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 180u, 210u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_I1     (STEP_WAIT_HANDSHAKE,                  "等待握手", 0u),  // ucRpiHandshake()
    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热副摄", WARM_CAM_QR, 1u, 0u),  // vRpiWarmCamera()
    SCRIPT_STEP        (STEP_DEAD,                            "按主按键开始"),  // vScriptWaitKeyConfirm()
    SCRIPT_STEP_I1     (STEP_TIMER,                           "计时", TIMER_CMD_START),  // vTimerCmd()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_DIAG_LF_WAIT, "斜线到出发区中央", MOVE_OUT_DIAG, 0u),  // vChassisMoveWorldDiag()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "安全扫码", SCRIPT_PRESET_37),  // vScriptCallPreset()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_111),  // vScriptIf()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "走右边", SCRIPT_PRESET_41),  // vScriptCallPreset()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "走左边", SCRIPT_PRESET_40),  // vScriptCallPreset()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前进", MOVE_ROAD_X, 0u),  // vChassisMoveWorldX()
    SCRIPT_STEP_I2     (STEP_ROAD_ALIGN_WAIT,                 "看路对位", 6000u, 1u),  // ucRpiRoadAlign()
    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "右转", 0.0f, 0u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "继续前进准备到灯泡", MOVE_BULB_Y, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_I2     (STEP_ROAD_ALIGN_WAIT,                 "看路对位", 6000u, 1u),  // ucRpiRoadAlign()
//    SCRIPT_STEP_I2     (STEP_RPI_WARM_CAM,                    "预热摄像头: 主摄", WARM_CAM_MAIN, 1u),  // vRpiWarmCamera()
    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热主摄+pi", WARM_CAM_MAIN_PI, 1u, 0),  // vRpiWarmCamera()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_11, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "右转", -90.0f, 0u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "切换到识别标靶", ARM_ACT_15, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_F1     (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前往灯泡处", MOVE_LAMP_X),  // vChassisMoveWorldX()
//    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -90.0f, 1000u, 0u),  // ucRpiLineConverge()//    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "看路", ARM_ACT_26, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "执行任务二", SCRIPT_PRESET_44),  // vScriptCallPreset()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_11, 0),  // vArmPlayNoWaitDelay()
//    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_108),  // vScriptIf()
//    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_11),  // vArmPlayNoWait()
//    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "走到激光半程", MOVE_LASER_X, 0u),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -90.0f, 500u, 0u),  // ucRpiLineConverge()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_115),  // vScriptIf()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_108),  // vScriptIf()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_33, 0u),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_13, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_108),  // vScriptIf()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_34, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "复位回待命", ARM_ACT_11),  // vArmPlayNoWait()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
//    SCRIPT_STEP_I1     (STEP_IF,                              "机械臂复位", SCRIPT_COND_SW_109),  // vScriptIf()
//    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
//    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "切换到识别标靶", ARM_ACT_15),  // vArmPlayNoWait()
//    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "前往人像处", -820.0f, 0u),  // vChassisMoveWorldX()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -90.0f, 1000u, 0u),  // ucRpiLineConverge()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "执行任务一", SCRIPT_PRESET_43),  // vScriptCallPreset()
//    SCRIPT_STEP_I1     (STEP_TARGET_SERVO_WAIT,               "标靶流控闭环: 起流→10Hz", 12000u),  // ucRpiTargetServo()
//    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热主摄", WARM_CAM_MAIN, 2u, 9u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "看路", ARM_ACT_26),  // vArmPlayNoWait()
//    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 80u, 150u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_X_WAIT,       "离开激光准备转弯", MOVE_LEAVE_X, 1u),  // vChassisMoveWorldX()
    SCRIPT_STEP_I2     (STEP_CHASSIS_SET_SPEED_ACCEL,         "设置速度加速度", 170u, 210u),  // vChassisSetWorldSpeedAccel()
    SCRIPT_STEP_I2     (STEP_ROAD_ALIGN_WAIT,                 "看路对位", 6000u, 1u),  // ucRpiRoadAlign()
    SCRIPT_STEP_I3     (STEP_RPI_WARM_CAM,                    "预热主摄", WARM_CAM_MAIN, 1u, 2u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_115),  // vScriptIf()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_110),  // vScriptIf()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "切换准备识别人质", ARM_ACT_34, 0u),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "切换准备识别人质", ARM_ACT_11, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_SW_110),  // vScriptIf()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "切换准备识别人质", ARM_ACT_33, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_NO_WAIT,                "切换准备识别人质", ARM_ACT_13),  // vArmPlayNoWait()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
    SCRIPT_STEP_F1I2   (STEP_PIVOT_TURN_WAIT,                 "右转", -180.0f, 0u, 1u),  // vPidPivotTurn()
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "走到人质", MOVE_HOSTAGE_Y, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "执行任务三", SCRIPT_PRESET_45),  // vScriptCallPreset()// ucRpiHostageGrabOne()
    SCRIPT_STEP_F2I1   (STEP_CHASSIS_MOVE_WORLD_WAIT,         "一步走到世界尽头", 0.0f, MOVE_END1_Y, 1u),  // vChassisMoveWorld()
    SCRIPT_STEP_F1I2   (STEP_LINE_CONVERGE_WAIT,              "巡线收敛", -180.0f, 500u, 0u),  // ucRpiLineConverge()
//    SCRIPT_STEP_F1I2   (STEP_SPIN_TURN_WAIT,                  "四轮绕中心原地自转", 1.0f, 0u, 0u),  // vPidSpinTurn()
    SCRIPT_STEP_F2I1   (STEP_CHASSIS_MOVE_WORLD_WAIT,         "一步走到世界尽头", 0.0f, MOVE_END2_Y, 0u),  // vChassisMoveWorld()
//    SCRIPT_STEP_F2I1   (STEP_CHASSIS_MOVE_WORLD_WAIT,         "一步走到世界尽头", 0.0f, -1640.0f, 1u),  // vChassisMoveWorld()
    SCRIPT_STEP_I1     (STEP_TIMER,                           "计时", TIMER_CMD_STOP),  // vTimerCmd()
    SCRIPT_STEP_I1     (STEP_SCRIPT_CALL,                     "复位回起点", SCRIPT_PRESET_42),  // vScriptCallPreset()
    SCRIPT_STEP        (STEP_SCRIPT_RESET,                    "重置脚本"),  // ucScriptResetTarget()
};

/* ===== 预设: 安全扫码_PLUS ===== */
const stScriptStepTdf g_astScriptPreset54[] = {
    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "一步前进到二维码位", 600.0f, 0u),  // vChassisMoveWorldY()
    SCRIPT_STEP_I2     (STEP_QR_SCAN_WAIT,                    "副摄二维码扫描", 5000u, QR_ALGO_AUTO),  // ucRpiQrScan()
    SCRIPT_STEP_I1     (STEP_IF,                              "如果: 条件不成立走否则分支", SCRIPT_COND_QR),  // vScriptIf()
    SCRIPT_STEP        (STEP_ELSE,                            "否则: 条件不成立时从这里继续"),  // vScriptElse()
    SCRIPT_STEP_I2     (STEP_RPI_WARM_CAM,                    "预热摄像头", WARM_CAM_MAIN, 1u),  // vRpiWarmCamera()
    SCRIPT_STEP_I1     (STEP_ARM_PLAY_WAIT,                   "机械臂待命到正视二维码模式", ARM_ACT_22),  // vArmPlayWait()
    SCRIPT_STEP_I2     (STEP_QR_SCAN_MAIN_WAIT,               "主摄二维码扫描", 8000u, QR_ALGO_AUTO),  // ucRpiQrScanMain()
    SCRIPT_STEP_I2     (STEP_ARM_PLAY_NO_WAIT,                "从正视二维码回收到初始位置", ARM_ACT_23, 0),  // vArmPlayNoWaitDelay()
    SCRIPT_STEP        (STEP_END_IF,                          "结束如果"),  // vScriptEndIf()
//    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "一步前进到二维码位", 30.0f, 0u),  // vChassisMoveWorldY()
//    SCRIPT_STEP_F1I1   (STEP_CHASSIS_MOVE_WORLD_Y_WAIT,       "退回准备转弯", -180.0f, 0u),  // vChassisMoveWorldY()
};

/* ==================================================================
 * 预设注册表 —— 控制台「地图」Tab 生成（标记之间整段重写, 手改会被下次保存覆盖）。
 * 供 STEP_SCRIPT_CALL 按 id 查表跳转; 条目顺序 = 上面各表的顺序。
 * （嵌套 IF 的分支不查这张表 —— 它就在本表内部靠位置配对, 见文件头的说明。）
 * usCount 写成 SCRIPT_PRESET_COUNT(表名) 而不是数字 —— 手改 map.c 增删步骤时
 * 不用回来同步维护计数。
 * ================================================================== */
/* ==== 预设注册表: 开始 ==== */
const stScriptPresetTdf g_astScriptPreset[] = {
    { 1u, "默认路线", g_astScriptPreset1, SCRIPT_PRESET_COUNT(g_astScriptPreset1), 0u },
    { 2u, "测试巡线闭环", g_astScriptPreset2, SCRIPT_PRESET_COUNT(g_astScriptPreset2), 0u },
    { 3u, "测试减速带", g_astScriptPreset3, SCRIPT_PRESET_COUNT(g_astScriptPreset3), 0u },
    { 4u, "测试原地转弯", g_astScriptPreset4, SCRIPT_PRESET_COUNT(g_astScriptPreset4), 0u },
    { 5u, "测试脚本重置", g_astScriptPreset5, SCRIPT_PRESET_COUNT(g_astScriptPreset5), 0u },
    { 6u, "测试激光", g_astScriptPreset6, SCRIPT_PRESET_COUNT(g_astScriptPreset6), 0u },
    { 7u, "测试机械臂", g_astScriptPreset7, SCRIPT_PRESET_COUNT(g_astScriptPreset7), 0u },
    { 8u, "测试标靶", g_astScriptPreset8, SCRIPT_PRESET_COUNT(g_astScriptPreset8), 0u },
    { 9u, "测试小球对位", g_astScriptPreset9, SCRIPT_PRESET_COUNT(g_astScriptPreset9), 0u },
    { 10u, "测试积木", g_astScriptPreset10, SCRIPT_PRESET_COUNT(g_astScriptPreset10), 0u },
    { 11u, "测试人质对位", g_astScriptPreset11, SCRIPT_PRESET_COUNT(g_astScriptPreset11), 0u },
    { 12u, "测试标靶流控", g_astScriptPreset12, SCRIPT_PRESET_COUNT(g_astScriptPreset12), 0u },
    { 14u, "OLED色球放置", g_astScriptPreset14, SCRIPT_PRESET_COUNT(g_astScriptPreset14), 1u },
    { 15u, "OLED激光射靶", g_astScriptPreset15, SCRIPT_PRESET_COUNT(g_astScriptPreset15), 1u },
    { 16u, "OLED人质抓取", g_astScriptPreset16, SCRIPT_PRESET_COUNT(g_astScriptPreset16), 1u },
    { 19u, "测试主摄像头识别二维码", g_astScriptPreset19, SCRIPT_PRESET_COUNT(g_astScriptPreset19), 0u },
    { 22u, "测试支点顺畅转弯", g_astScriptPreset22, SCRIPT_PRESET_COUNT(g_astScriptPreset22), 0u },
    { 23u, "OLED仅看路", g_astScriptPreset23, SCRIPT_PRESET_COUNT(g_astScriptPreset23), 1u },
    { 24u, "过减速带(缓慢顶住)", g_astScriptPreset24, SCRIPT_PRESET_COUNT(g_astScriptPreset24), 0u },
    { 25u, "过减速带（倒退顶住）", g_astScriptPreset25, SCRIPT_PRESET_COUNT(g_astScriptPreset25), 0u },
    { 27u, "测试单动作色球抓取", g_astScriptPreset27, SCRIPT_PRESET_COUNT(g_astScriptPreset27), 0u },
    { 29u, "测试巡线走", g_astScriptPreset29, SCRIPT_PRESET_COUNT(g_astScriptPreset29), 0u },
    { 30u, "测试OpenMV补光", g_astScriptPreset30, SCRIPT_PRESET_COUNT(g_astScriptPreset30), 0u },
    { 31u, "测试纯陀螺仪巡线收敛", g_astScriptPreset31, SCRIPT_PRESET_COUNT(g_astScriptPreset31), 0u },
    { 32u, "测试平移射靶", g_astScriptPreset32, SCRIPT_PRESET_COUNT(g_astScriptPreset32), 1u },
    { 33u, "测试看路校准", g_astScriptPreset33, SCRIPT_PRESET_COUNT(g_astScriptPreset33), 0u },
    { 34u, "测试射靶保持", g_astScriptPreset34, SCRIPT_PRESET_COUNT(g_astScriptPreset34), 1u },
    { 35u, "测试补光灯", g_astScriptPreset35, SCRIPT_PRESET_COUNT(g_astScriptPreset35), 0u },
    { 36u, "OLED看路转弯", g_astScriptPreset36, SCRIPT_PRESET_COUNT(g_astScriptPreset36), 1u },
    { 37u, "安全扫码", g_astScriptPreset37, SCRIPT_PRESET_COUNT(g_astScriptPreset37), 0u },
    { 39u, "常规扫码", g_astScriptPreset39, SCRIPT_PRESET_COUNT(g_astScriptPreset39), 0u },
    { 40u, "走左边", g_astScriptPreset40, SCRIPT_PRESET_COUNT(g_astScriptPreset40), 0u },
    { 41u, "走右边", g_astScriptPreset41, SCRIPT_PRESET_COUNT(g_astScriptPreset41), 0u },
    { 42u, "复位回起点", g_astScriptPreset42, SCRIPT_PRESET_COUNT(g_astScriptPreset42), 0u },
    { 43u, "执行任务一", g_astScriptPreset43, SCRIPT_PRESET_COUNT(g_astScriptPreset43), 0u },
    { 44u, "执行任务二", g_astScriptPreset44, SCRIPT_PRESET_COUNT(g_astScriptPreset44), 0u },
    { 45u, "执行任务三", g_astScriptPreset45, SCRIPT_PRESET_COUNT(g_astScriptPreset45), 0u },
    { 46u, "测试爆爆桶对位", g_astScriptPreset46, SCRIPT_PRESET_COUNT(g_astScriptPreset46), 1u },
    { 47u, "反向路线", g_astScriptPreset47, SCRIPT_PRESET_COUNT(g_astScriptPreset47), 0u },
    { 48u, "执行反向看路", g_astScriptPreset48, SCRIPT_PRESET_COUNT(g_astScriptPreset48), 1u },
    { 49u, "执行任务一(反向)", g_astScriptPreset49, SCRIPT_PRESET_COUNT(g_astScriptPreset49), 0u },
    { 50u, "执行任务三（反向）", g_astScriptPreset50, SCRIPT_PRESET_COUNT(g_astScriptPreset50), 0u },
    { 53u, "默认路线切换标靶和色球", g_astScriptPreset53, SCRIPT_PRESET_COUNT(g_astScriptPreset53), 0u },
    { 54u, "安全扫码_PLUS", g_astScriptPreset54, SCRIPT_PRESET_COUNT(g_astScriptPreset54), 0u },
};
const uint16_t g_usScriptPresetNum = SCRIPT_PRESET_COUNT(g_astScriptPreset);
/* ==== 预设注册表: 结束 ==== */

/* ==================================================================
 * 当前运行的预设 —— 控制台「地图」Tab 的"切换运行"只改下面这一行。
 * sizeof(SCRIPT_PRESET_ACTIVE) 直接取所选表的长度, 无需同步维护计数。
 * ================================================================== */
#define SCRIPT_PRESET_ACTIVE  g_astScriptPreset1

const stScriptStepTdf *const g_pstScriptActive = SCRIPT_PRESET_ACTIVE;
const uint16_t g_usScriptActiveCount =
    (uint16_t)(sizeof(SCRIPT_PRESET_ACTIVE) / sizeof(SCRIPT_PRESET_ACTIVE[0]));
