/**
  * @file       app_chassis.h
  * @brief      底盘运动学 + 世界坐标系模块（移植自 pro 项目 Motion/yyb_move.c 的底盘部分）。
  *
  *             麦轮底盘 4 电机速度/位置解算、世界系平移、运动时间估算。
  *             机械臂三轴（升降/平推/云台）不在此模块（见 app_arm / servo_device）。
  */

#ifndef __APP_CHASSIS_H
#define __APP_CHASSIS_H

#include <stdint.h>
#include "app_motion_types.h"   /* axis_t, ABS */

/* 世界系平移符号标定常量（移植自 pro yyb_move.h）
 * 世界系约定: 偏航角 0°=启动正前方(+Y), 逆时针为正; +X=启动正左方 */
#define WORLD_SIDE_SIGN  (+1.0f)
#define WORLD_FWD_SIGN   (+1.0f)

/* 麦轮速度模式运动（开环）：v_x>0=右移, v_y>0=前进, w>0=逆时针，单位 rpm */
void vChassisMove(int v_x, int v_y, int w);
/* 加速度**按实参**的版本（EMM 单字节档位 0~255, 不取全局量; 2026-10 加）:
 * 巡线直走要「估时公式的加速度 == 实际斜坡的加速度」, mm→时间反推才成立 */
void vChassisMoveA(int v_x, int v_y, int w, int iAccel);

/* 底盘四电机(地址1~4)统一使能/失能（瞬时指令, 内含 1ms 命令间隔） */
void vChassisEnableAll(uint8_t ucEnable);

/* 底盘四电机(地址1~4)统一切换开环/闭环控制模式（瞬时指令, 内含 1ms 命令间隔）。
 * ucMode 取 emEmmV5CtrlModeTdf 的 1=开环 / 2=闭环（越界值等同"关脉冲输入"，别传 0/3）。
 * ⚠️ **不写驱动器 Flash**: 模式当场生效, 掉电自动回原模式（有意如此, 理由见 .c） */
void vChassisSetLoopModeAll(uint8_t ucMode);

/* 像素偏差 → 位置模式旋转 */
void vChassisRotateByPixelError(float err_px, float px_to_pulse,
                                uint16_t speed, uint8_t accel);

/* 弧线转弯（位置模式写死标定, 不走 PID/不依赖 IMU）:
 * fDeltaAngle>0=左转, fFwdMm=弧线期间前进量; 精度由 ARC_TURN_* 标定宏保证 */
void vChassisArcTurnPos(float fDeltaAngle, float fFwdMm);
void vChassisTurn90Left(void);    /* = ArcTurnPos(+90°, ARC_TURN90_FWD_MM) */
void vChassisTurn90Right(void);   /* = ArcTurnPos(-90°, ARC_TURN90_FWD_MM) */

/* 支点转弯的单侧驱动原语（速度模式, 开环, 瞬时下发）。
 * iSide:  +1 = 驱动右侧双轮 {2,4}(RF/RB); -1 = 驱动左侧双轮 {1,3}(LF/LB)
 * iSpeed: 驱动侧轮速(rpm), >0=该侧前进, <0=该侧后退; 另一侧双轮显式发 0 速。
 * 闭环在 app_pid.c::vPidPivotTurn（陀螺仪）, 本函数只管发速度、不管角度。 */
void vChassisPivotDrive(int iSide, int iSpeed);

/* 设定麦轮速度模式加速度（写入 Beam 底盘加速度） */
void vChassisSetAccel(int32_t a_in);

/* 过减速带（阻塞, 仅任务上下文）: 低速位置模式顶住坎面, **纯延时**等车被接触反力
 * 被动转正（速度低到上不去）→ 显式停车 → 位置模式大力冲过（标定量 RAMP_* 见 app_config.h）。
 * **全程不碰陀螺仪**: 不靠它判断"什么时候该冲"（顶住时长由 iAlignMs 给足即可）, 也不用它修正航向
 * —— 顶住期间被动转正的那几度**不会**并进 fGetWorldHeading(), 会留作后续世界系平移的误差（有意选的）。
 * 方向取世界系前进, 按当前指令航向映射到车体轴（三段共用同一次查表）。
 * fDistMm 冲坡前进量(mm, 正=世界前方), ≤0 不动车; iAlignMs 顶住时长(ms), ≤0 跳过顶住段。
 *
 * ⚠️ 冲坡距离是**开环脉冲**: 轮子打滑时位置目标照样跑完而车没走到, 且应用层没有任何 EMM
 *    应答解析 → 失败是**静默的**。所以：① 路线里前一步须把车停在坎前 10~20mm 且车头朝坎;
 *    ② dist 给足余量保证车完全离开坎面; ③ 后面**必须**接视觉重同步
 *    （map.c 排一块 `src=不修` 的 STEP_LINE_CONVERGE_WAIT）。详见 CLAUDE.md「雪弗板减速带过坎」。 */
void vChassisCrossRampWait(float fDistMm, int iAlignMs);

/* 摄像头引导移动: body-frame（摄像头反装，内部交换 XY） */
void vChassisMoveCameraDistance(float dx, float dy, int accel, int speed);

/* 车体横向位置移动（视觉对位用）: fMm>0=右移（与 vChassisMove 的 v_x 同向）,
 * 走 X 轴脉冲系数 CHASSIS_PULSE_PER_MM_X, 位置模式阻塞到位（内部含估时等待）。
 * ⚠️ 不要用 vChassisMoveCameraDistance 代替: 它的 XY 交换是 pro 项目 90° 反装相机
 *    的约定, 本车相机装法不同, 直接用会把横移打到前后轴上。 */
void vChassisMoveCameraLateral(float fMm, int iAccel, int iSpeed);

/* 车体纵向位置移动（小球对位用）: vChassisMoveCameraLateral 的纵向对偶,
 * fMm>0=前进（与 vChassisMove 的 v_y 同向）, 走 Y 轴脉冲系数, 位置模式阻塞到位。 */
void vChassisMoveCameraForward(float fMm, int iAccel, int iSpeed);

/* 平移**启动**（只发位置指令, 不等待到位; 2026-10 单动作抓取"底盘与机械臂并行"用）:
 * 到位等待由调用方自己补 —— 先 fChassisMoveTimeEst 估时, 再按需切片等待;
 * ⚠️ 在飞的位置移动被下一条位置指令覆盖前必须等完（同 vChassisMoveCameraLateral 那条警告）,
 * ⚠️ 双轴平移仍是先 X 后 Y 串行启动（同时发两轴位置指令会互相覆盖, 见 vChassisMoveDistance）。 */
void vChassisMoveCameraLateralStart(float fMm, int iAccel, int iSpeed);
void vChassisMoveCameraForwardStart(float fMm, int iAccel, int iSpeed);

/* 世界系移动速度/加速度（独立保存，只在需要变速时调用一次） */
void vChassisSetWorldSpeedAccel(int accel, int speed);

/* 车体系分量 → 世界位移（vChassisWorldToBody 的逆映射, 同一张象限表, 2026-10 加）。
 * 用途: 视觉对位/单动作抓取那类**车体系**平移的累计量要折进世界系"路径残留偏移"
 * （app_beam 的 path_res_x/y）时, 按当前指令航向换算一次。世界系约定同下:
 * 偏航角 0°=启动正前方(+Y), 逆时针为正; +X=启动正左方。
 * ⚠️ 与 vChassisWorldToBody 同一条纪律: 同一段动作里要多次换算必须**只查一次**航向并缓存。 */
void vChassisBodyToWorld(float fSide, float fFwd, float *pfWx, float *pfWy);

/* 世界位移 → 车体系（2026-10 公开, 与内部 vChassisMoveWorld 的 use_res 同一张象限表）:
 * 用途 = 「巡线直走」的 use_res 档把世界系「路径残留偏移」折到车体前向分量。
 * 出参 (side, fwd): side 走 X 轴(v_x>0=右移)、fwd 走 Y 轴; 世界系约定同下。
 * ⚠️ fGetWorldHeading() 是航位推算量 —— 同一段动作里多次换算必须只查一次并缓存。 */
void vChassisWorldToBodyEx(float wx, float wy, float *pfSide, float *pfFwd);

/* 麦轮底盘"世界坐标系"平移：按启动朝向把 (wx,wy) 映射为车体前后/左右。
 * ucUseRes != 0 时先把 Beam 里的「路径残留偏移」(世界系 mm, 单动作人质抓取留下的)
 * 并进本次位移: 本次 = (wx,wy) − 残留（车已偏过去多少就少走多少, 落回路线原定终点;
 * 抓取时后撤过则表现为"多走"）。读出即清零 —— 一次性, 消费走。
 *
 * 2026-10-23 加「补偿后短行程保护」（只对 use_res=1 生效, 阈值见 app_config.h 的
 * WORLD_RES_*）: 补偿后车体位移 ≤ WORLD_RES_SKIP_MM 直接视为到位（不发指令）;
 * < WORLD_RES_SHORT_MM 时自动把加速度/速度压到短行程那一对（只降不升）。
 * 目的: 打右靶后残留把下一步缩得很短时, 别用默认高加速度档冲出「走一下/偏一下/急刹」;
 * 补偿后该走多少还是多少, 终点不变。 */
void vChassisMoveWorld(float wx, float wy, uint8_t ucUseRes);
void vChassisMoveWorldX(float wx, uint8_t ucUseRes);   /* 仅世界 X 方向 */
void vChassisMoveWorldY(float wy, uint8_t ucUseRes);   /* 仅世界 Y 方向 */
void vChassisMoveWorldDiagLF(float wd);         /* 世界斜向 LF+LB 对, 正=斜左上 */
void vChassisMoveWorldDiagRF(float wd);         /* 世界斜向 RF+RB 对, 正=斜右上 */
void vChassisMoveWorldDiag(uint8_t ucDir, float wd);
                                /* 世界斜向**合并档**（积木 STEP_CHASSIS_MOVE_WORLD_DIAG_LF_WAIT 的原语,
                                 * 2026-10 由"斜左上/斜右上"两块积木合并而来）:
                                 * ucDir 0=斜左上(默认=老行为) 1=斜右上 2=斜左下 3=斜右下;
                                 * wd 带符号（负=沿所选方向反向）。分派见 app_chassis.c */

/* 纯估时（**不等待**; 2026-10 加）: fChassisMoveDelay 的"只算不等"一半 ——
 * 平移与机械臂并行时, 调用方要按估时自己切片等待（期间还要插延时发臂）,
 * 阻塞版没法用（它一等就是全程）。返回估算耗时 ms（与 fChassisMoveDelay 同一公式）。 */
float fChassisMoveTimeEst(float distance, uint16_t a_of_car, uint16_t speed_of_car, axis_t axis);

/* 底盘运动时间估算（梯形加减速）并执行让出式等待，返回估算耗时 ms（仅任务上下文） */
float fChassisMoveDelay(float distance, uint16_t a_of_car, uint16_t speed_of_car, axis_t axis);

#endif /* __APP_CHASSIS_H */
