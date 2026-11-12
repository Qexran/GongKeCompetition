/**
  * @file       app_chassis.c
  * @brief      底盘运动学 + 世界坐标系模块实现（移植自 pro 项目 Motion/yyb_move.c 底盘部分
  *             + Function.c 的 u32ChassisMoveDelay）。
  *
  *             适配点（SPL 裸机 → HAL/FreeRTOS）:
  *               vEmmV5VelControl/PosControl 的 dir 由裸 uint8 改为枚举 emEmmV5DirTdf；
  *               vEmmV5SyncMotion → vEmmV5SynchronousMotion；
  *               vDelayMs → vTaskDelay（让出式，忙等会饿死按键/显示/传感器任务）；
  *               nGetChassisAccel → iGetChassisAccel；
  *               运动估时 pow(double) → sqrtf（M4F 单精度 FPU，double 为软件模拟）。
  *
  *             本模块所有函数仅在任务上下文调用（脚本任务 / vPidTask），
  *             内部含 vTaskDelay，禁止从中断调用。
  *
  *             电机地址 1~4 = 麦轮(LF/RF/LB/RB)。机械臂的升降/平推(地址 5/6)不在本模块。
  *             安装注意: 电机 1 与 2 是反着装的（旋转方向定义互逆）。代码正速度
  *             方向位为 电机1/4=CCW、电机2/3=CW，即 {1,4} 与 {2,3} 两组互逆，
  *             速度模式与位置模式的符号表均已按此实测校准，改动前先实车核对。
  */

#include "app_chassis.h"
#include "app_beam.h"
#include "app_config.h"      /* emm_v5_device.h, main.h */
#include "FreeRTOS.h"
#include "task.h"
#include <math.h>

/* 世界系移动速度/加速度(rpm, rpm/s)，vChassisSetWorldSpeedAccel 设定 */
static int32_t s_iWorldMoveAccel = 150;
static int32_t s_iWorldMoveSpeed = 180;

/* 底盘四电机(地址1~4)统一使能/失能: 瞬时指令, 电机需短暂间隔处理命令 */
void vChassisEnableAll(uint8_t ucEnable)
{
    uint8_t ucAddr;
    for (ucAddr = 1; ucAddr <= 4; ucAddr++)
    {
        vEmmV5EnControl(ucAddr, ucEnable, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*
 * 底盘四电机(地址1~4)统一切换开环/闭环控制模式（EMM 指令 0x46 0x69）。
 * ucMode 取 emEmmV5CtrlModeTdf: 1=开环, 2=闭环（0/3 是脉冲/多圈限位复用, 不是"环模式",
 * 路线积木的下拉只放 1/2 —— 那两种会改引脚功能, 误点会静默改掉整台车的输入行为）。
 *
 * ⚠️ 存储标志恒为 **0 = 不存储**: 模式当场生效, 但**掉电回原模式**。
 *    这是刻意的 —— 环模式是"这次想试一下"的东西, 写进驱动器 Flash 之后就变成整台车的
 *    持久默认行为, 而路线里没有任何一步能保证一定被跑到（中途急停/换预设/改二维码分支
 *    都会跳过"改回来"那一步）, 一旦写坏只能拿上位机去救。要固化就手动用上位机改并存储。
 * ⚠️ 建议在**停车时**切换（本步只发这 4 条模式帧, 不碰速度/位置指令）。
 */
void vChassisSetLoopModeAll(uint8_t ucMode)
{
    uint8_t ucAddr;

    for (ucAddr = 1; ucAddr <= 4; ucAddr++)
    {
        vEmmV5ModifyCtrlMode(ucAddr, 0, (emEmmV5CtrlModeTdf)ucMode);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

/*
 * 麦轮底盘速度模式运动（开环）—— **加速度按实参**（EMM 单字节档位 0~255）。
 * 四个麦轮速度由 v_x/v_y/w 叠加解算，各轮独立发速度指令。
 * 实测: 正 v_x=右移, 正 v_y=前进 (XY 坐标系: X=左右, Y=前后)
 *
 * ⚠️ 为什么要有"加速度按实参"的版本: 巡线直走（STEP_LINE_STRAIGHT_WAIT）用「位置模式
 *    同款估时」把 mm 反推成时长, 估时公式里的加速度必须就是**实际斜坡用的那个** ——
 *    若实际斜坡取全局量而被别人改过, 反推就不成立。普通路径继续走 vChassisMove
 *    （取 Beam 底盘加速度）, 行为与拆分前一字不差。
 *
 * ⚠️⚠️ **下标 = 电机地址**（speed[n] 发给电机 n）: 1=LF 左前, 2=RF 右前, 3=LB 左后, 4=RB 右后。
 *    2026-09 修过一次**交叉**的笔误（speed[1] 发给电机 2、speed[2] 发给电机 1）——
 *    症状是「速度模式前进时**只有电机 1 和 2 转反**」（台架反馈）。
 *    为什么横移一直没暴露: v_x>0 时电机 1 与 2 拿到的方向字节**本来就一样**（都是 CW），
 *    交叉了看不出; 而前进时两者相反, 交叉立刻显形。
 *    ⚠️ **改这里必对照**: 前进的方向字节要与 `vChassisMoveDistanceY` 那张久经验证的**位置模式**
 *    表逐位相同 —— 下标=电机地址 1..4 时 v_y>0 为 {1:0(CW), 2:1(CCW), 3:0(CW), 4:1(CCW)}
 *    （也就是 app_chassis.c 里那张 s_aucFwdDir）。
 */
void vChassisMoveA(int v_x, int v_y, int w, int iAccel)
{
    int speed[5];

    /* 发布最后一次速度指令（OLED 显示 / 遥测用, 见 app_beam.h）。
     * 发布的是**指令值**（调用方给的 vx/vy/w），不是解算后的四轮转速:
     * 巡线闭环调参要看的就是它算完限幅后交给底盘的那个数。 */
    vSetChassisSpdCmd((int32_t)v_x, (int32_t)v_y, (int32_t)w);

    speed[1] = +v_x + v_y - w;      /* → 电机 1 (LF) */
    speed[2] = -v_x + v_y + w;      /* → 电机 2 (RF) */
    speed[3] = -v_x + v_y - w;      /* → 电机 3 (LB) */
    speed[4] = +v_x + v_y + w;      /* → 电机 4 (RB) */

    /* 阻塞发送保证字节完整性, 电机需短暂间隔处理命令, 保留 1ms 间隙
     * （vTaskDelay 让出式，不饿死低优先级任务） */
    if (speed[1] >= 0) {
        vEmmV5VelControl(1, emEmmV5Dir_CW, speed[1], (uint8_t)iAccel, 0);
    } else {
        vEmmV5VelControl(1, emEmmV5Dir_CCW, -speed[1], (uint8_t)iAccel, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    if (speed[2] >= 0) {
        vEmmV5VelControl(2, emEmmV5Dir_CCW, speed[2], (uint8_t)iAccel, 0);
    } else {
        vEmmV5VelControl(2, emEmmV5Dir_CW, -speed[2], (uint8_t)iAccel, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    if (speed[3] >= 0) {
        vEmmV5VelControl(3, emEmmV5Dir_CW, speed[3], (uint8_t)iAccel, 0);
    } else {
        vEmmV5VelControl(3, emEmmV5Dir_CCW, -speed[3], (uint8_t)iAccel, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
    if (speed[4] >= 0) {
        vEmmV5VelControl(4, emEmmV5Dir_CCW, speed[4], (uint8_t)iAccel, 0);
    } else {
        vEmmV5VelControl(4, emEmmV5Dir_CW, -speed[4], (uint8_t)iAccel, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(1));
}

/* 麦轮底盘速度模式运动（Beam 底盘加速度版）: vChassisMoveA 的常用壳 ——
 * 加速度取全局量（vChassisSetAccel / 「底盘」蓝牙组 chassis_accel 那个）。 */
void vChassisMove(int v_x, int v_y, int w)
{
    vChassisMoveA(v_x, v_y, w, (int)iGetChassisAccel());
}

/*
 * 支点转弯的单侧驱动（速度模式, 开环）
 * iSide:  +1 = 驱动右侧双轮 {2,4}(RF/RB), -1 = 驱动左侧双轮 {1,3}(LF/LB)
 * iSpeed: 驱动侧轮速(rpm), >0=该侧前进, <0=该侧后退。
 *
 * 另一侧双轮显式发 0 速 —— EMM 停在原位带保持力, 这就是"支点"的含义: 该侧轮组
 * 不主动转、抵抗车体滑动, 单侧出力使车体绕该侧前后轮连线旋转。麦轮下这个动作
 * 会附带少量前向平移（速度分解 vy=w=V/2, 见 vChassisMove 的麦轮模型）, 属该
 * 动作固有特性, 不是 bug; 转弯过程中车头会朝支点侧"蹭"一点。
 *
 * 方向字节沿用 Y 轴前进表（同 vChassisMoveDistanceY: 前进时 m1/m3=CW, m2/m4=CCW）,
 * 后退整组翻转。加速度取 Beam 底盘加速度（vChassisSetAccel 写入）, 调用方限幅。
 */
void vChassisPivotDrive(int iSide, int iSpeed)
{
    /* 下标 = 电机地址 1..4; 前进方向字节（0=CW, 1=CCW） */
    static const uint8_t s_aucFwdDir[5] = {0, 0, 1, 0, 1};
    int iDrvA  = (iSide > 0) ? 2 : 1;      /* 驱动侧两轮 */
    int iDrvB  = (iSide > 0) ? 4 : 3;
    int iIdleA = (iSide > 0) ? 1 : 2;      /* 支点侧两轮（不主动转） */
    int iIdleB = (iSide > 0) ? 3 : 4;
    int iMag   = (iSpeed >= 0) ? iSpeed : -iSpeed;
    uint8_t ucRev = (iSpeed >= 0) ? 0 : 1;
    uint8_t ucAcc = (uint8_t)iGetChassisAccel();

    /* 先把支点侧显式归零: 防止上一步残留速度把它带跑（每拍都发, 幂等） */
    vEmmV5VelControl((uint8_t)iIdleA, emEmmV5Dir_CW, 0, ucAcc, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    vEmmV5VelControl((uint8_t)iIdleB, emEmmV5Dir_CW, 0, ucAcc, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    /* 再驱动动力侧双轮 */
    vEmmV5VelControl((uint8_t)iDrvA, (emEmmV5DirTdf)(s_aucFwdDir[iDrvA] ^ ucRev),
                     (uint16_t)iMag, ucAcc, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    vEmmV5VelControl((uint8_t)iDrvB, (emEmmV5DirTdf)(s_aucFwdDir[iDrvB] ^ ucRev),
                     (uint16_t)iMag, ucAcc, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
}

/* 麦轮底盘 X 轴位置移动（同步启动），脉冲系数 CHASSIS_PULSE_PER_MM_X */
static void vChassisMoveDistanceX(float x, int accel, int speed)
{
    if (x == 0.0f) return;
    int quanshu = ABS(x) * CHASSIS_PULSE_PER_MM_X;
    int v_x = speed * (x > 0 ? 1 : -1);
    int dt = 5;

    vEmmV5PosControl(2, (emEmmV5DirTdf)((v_x >= 0) ? 0 : 1), ABS(v_x), accel, quanshu, 0, 1);  /* RF */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5PosControl(1, (emEmmV5DirTdf)((v_x >= 0) ? 0 : 1), ABS(v_x), accel, quanshu, 0, 1);  /* LF */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5PosControl(3, (emEmmV5DirTdf)((v_x >= 0) ? 1 : 0), ABS(v_x), accel, quanshu, 0, 1);  /* LB */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5PosControl(4, (emEmmV5DirTdf)((v_x >= 0) ? 1 : 0), ABS(v_x), accel, quanshu, 0, 1);  /* RB */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5SynchronousMotion(0x00);
}

/* 麦轮底盘 Y 轴位置移动（同步启动），脉冲系数 CHASSIS_PULSE_PER_MM_Y */
static void vChassisMoveDistanceY(float y, int accel, int speed)
{
    if (y == 0.0f) return;
    int quanshu = ABS(y) * CHASSIS_PULSE_PER_MM_Y;
    int v_y = speed * (y > 0 ? 1 : -1);
    int dt = 5;

    vEmmV5PosControl(2, (emEmmV5DirTdf)((v_y >= 0) ? 1 : 0), ABS(v_y), accel, quanshu, 0, 1);  /* RF */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5PosControl(1, (emEmmV5DirTdf)((v_y >= 0) ? 0 : 1), ABS(v_y), accel, quanshu, 0, 1);  /* LF */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5PosControl(3, (emEmmV5DirTdf)((v_y >= 0) ? 0 : 1), ABS(v_y), accel, quanshu, 0, 1);  /* LB */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5PosControl(4, (emEmmV5DirTdf)((v_y >= 0) ? 1 : 0), ABS(v_y), accel, quanshu, 0, 1);  /* RB */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5SynchronousMotion(0x00);
}

/* 斜向位置移动（对角电机对 2=RF + 3=LB），d>0=车体斜左上, d<0=车体斜右下 */
static void vChassisMoveDistanceLF(float d, int accel, int speed)
{
    if (d == 0.0f) return;
    int quanshu = ABS(d) * CHASSIS_PULSE_PER_MM_Y;
    int abs_speed = ABS(speed);
    int dt = 5;

    vEmmV5PosControl(2, (emEmmV5DirTdf)((d > 0) ? 1 : 0), abs_speed, accel, quanshu, 0, 1);  /* RF */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5PosControl(3, (emEmmV5DirTdf)((d > 0) ? 0 : 1), abs_speed, accel, quanshu, 0, 1);  /* LB */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5SynchronousMotion(0x00);
    fChassisMoveDelay(d, accel, abs_speed, AXIS_Y);
}

/* 斜向位置移动（对角电机对 1=LF + 4=RB），d>0=车体斜右上, d<0=车体斜左下 */
static void vChassisMoveDistanceRF(float d, int accel, int speed)
{
    if (d == 0.0f) return;
    int quanshu = ABS(d) * CHASSIS_PULSE_PER_MM_Y;
    int abs_speed = ABS(speed);
    int dt = 5;

    vEmmV5PosControl(1, (emEmmV5DirTdf)((d > 0) ? 0 : 1), abs_speed, accel, quanshu, 0, 1);  /* LF */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5PosControl(4, (emEmmV5DirTdf)((d > 0) ? 1 : 0), abs_speed, accel, quanshu, 0, 1);  /* RB */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5SynchronousMotion(0x00);
    fChassisMoveDelay(d, accel, abs_speed, AXIS_Y);
}

/* 像素偏差 → 位置模式旋转 (共用: 角度定标 + 像素偏差测试)
 * 电机分组 (2,3) vs (1,4), 两组方向相反 (与 vChassisMove 速度模型对齐) */
void vChassisRotateByPixelError(float err_px, float px_to_pulse,
                                uint16_t speed, uint8_t accel)
{
    if (ABS(err_px) < 0.01f) return;
    uint32_t pulses = (uint32_t)(ABS(err_px) * px_to_pulse);
    if (pulses == 0) pulses = 1;
    int dt = 5;

    /* 速度模式 CW(w<0): 四轮dir全0 → 因 mot1/4 方向约定反转,
     * 物理上(2,3)与(1,4)互逆 → 旋转。位置模式无此反转, 需显式互逆 */
    uint8_t dir_23 = (err_px > 0) ? 1 : 0;
    uint8_t dir_14 = (err_px > 0) ? 1 : 0;

    vEmmV5PosControl(2, (emEmmV5DirTdf)dir_23, speed, accel, pulses, 0, 1);  /* RF */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5PosControl(3, (emEmmV5DirTdf)dir_23, speed, accel, pulses, 0, 1);  /* LB */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5PosControl(1, (emEmmV5DirTdf)dir_14, speed, accel, pulses, 0, 1);  /* LF */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5PosControl(4, (emEmmV5DirTdf)dir_14, speed, accel, pulses, 0, 1);  /* RB */
    vTaskDelay(pdMS_TO_TICKS(dt));
    vEmmV5SynchronousMotion(0x00);
}

/*
 * 弧线转弯（位置模式, 写死标定, 不走 PID / 不依赖 IMU）
 * fDeltaAngle>0=左转(逆时针), fFwdMm=弧线期间前进量(mm)。
 * 每轮净脉冲 = 前进分量 ± 旋转差动分量(外侧多走、内侧少走):
 *   左转: 右侧轮 {2,4} +rot, 左侧轮 {1,3} -rot; 右转反之。
 * 方向字节取 Y 平移前进表(v>0: m1=0,m2=1,m3=0,m4=1), 净脉冲<0 时翻转。
 * 结束按指令角累加世界航向(Beam)——无 IMU 修正, 靠标定保证精度。
 */
void vChassisArcTurnPos(float fDeltaAngle, float fFwdMm)
{
    int fwd = (int)(fFwdMm * CHASSIS_PULSE_PER_MM_Y);
    int rot = (int)(ABS(fDeltaAngle) * ARC_TURN_PULSE_PER_DEG);
    int iLeft = (fDeltaAngle > 0.0f);          /* 左转: 右侧 {2,4} 为外侧 */

    /* Y+ 前进方向字节表(下标=电机地址 1..4), 同 vChassisMoveDistanceY */
    static const uint8_t s_aucFwdDir[5] = {0, 0, 1, 0, 1};
    int anNet[5];
    int iMax = 0;
    uint8_t ucAddr;
    int dt = 5;

    if (fwd == 0 && rot == 0) return;

    anNet[1] = fwd + (iLeft ? -rot : +rot);    /* LF 左侧 */
    anNet[2] = fwd + (iLeft ? +rot : -rot);    /* RF 右侧 */
    anNet[3] = fwd + (iLeft ? -rot : +rot);    /* LB 左侧 */
    anNet[4] = fwd + (iLeft ? +rot : -rot);    /* RB 右侧 */

    for (ucAddr = 1; ucAddr <= 4; ucAddr++)
    {
        int net = anNet[ucAddr];
        uint8_t dir = s_aucFwdDir[ucAddr];

        if (net < 0) { dir ^= 1; net = -net; }
        if (net > iMax) iMax = net;
        if (net == 0) continue;                /* 该轮无位移, 不发命令 */

        vEmmV5PosControl(ucAddr, (emEmmV5DirTdf)dir, ARC_TURN_SPEED,
                         ARC_TURN_ACCEL, (uint32_t)net, 0, 1);
        vTaskDelay(pdMS_TO_TICKS(dt));
    }
    vEmmV5SynchronousMotion(0x00);

    /* 阻塞等待: 以行程最大的轮折算毫米估时 */
    fChassisMoveDelay((float)iMax / CHASSIS_PULSE_PER_MM_Y,
                      ARC_TURN_ACCEL, ARC_TURN_SPEED, AXIS_Y);

    /* 指令航向累加(归一化到 ±180°), 供后续世界系平移查表 */
    {
        float fNew = fGetWorldHeading() + fDeltaAngle;
        if (fNew > 180.0f)       fNew -= 360.0f;
        else if (fNew < -180.0f) fNew += 360.0f;
        vSetWorldHeading(fNew);
    }
}

/* 原地/弧线 90° 转弯（位置模式写死标定） */
void vChassisTurn90Left(void)  { vChassisArcTurnPos(+90.0f, ARC_TURN90_FWD_MM); }
void vChassisTurn90Right(void) { vChassisArcTurnPos(-90.0f, ARC_TURN90_FWD_MM); }

/* 先 X 后 Y 两段位置移动 */
static void vChassisMoveDistance(float x, float y, int a_in, int speed_in)
{
    vTaskDelay(pdMS_TO_TICKS(10));
    vChassisMoveDistanceX(x, a_in, speed_in);
    fChassisMoveDelay(x, a_in, speed_in, AXIS_X);

    vTaskDelay(pdMS_TO_TICKS(10));
    vChassisMoveDistanceY(y, a_in, speed_in);
    fChassisMoveDelay(y, a_in, speed_in, AXIS_Y);
}

/* 摄像头引导移动: body-frame, DirectX/Y → 底盘位移 (不依赖世界航向)
 * 摄像头反装，应将 xy 颠倒 */
void vChassisMoveCameraDistance(float dx, float dy, int accel, int speed)
{
    vChassisMoveDistance(dy, dx, accel, speed);
}

/* 车体横向位置移动（视觉对位用）: fMm>0=右移（与 vChassisMove 的 v_x 同向）
 * ⚠️ vChassisMoveDistanceX 只发指令不等待, 到位等待必须自己补 fChassisMoveDelay,
 *    否则函数立刻返回而电机还在走。
 * ⚠️ 不要用 vChassisMoveCameraDistance 代替: 它的 XY 交换是 pro 项目 90° 反装相机的
 *    约定, 本车相机装法不同, 直接用会把横移打到前后轴上。 */
void vChassisMoveCameraLateral(float fMm, int iAccel, int iSpeed)
{
    vChassisMoveDistanceX(fMm, iAccel, iSpeed);
    fChassisMoveDelay(fMm, (uint16_t)iAccel, (uint16_t)iSpeed, AXIS_X);
}

/* 车体纵向位置移动（小球对位用）: vChassisMoveCameraLateral 的纵向对偶, fMm>0=前进 */
void vChassisMoveCameraForward(float fMm, int iAccel, int iSpeed)
{
    vChassisMoveDistanceY(fMm, iAccel, iSpeed);
    fChassisMoveDelay(fMm, (uint16_t)iAccel, (uint16_t)iSpeed, AXIS_Y);
}

/* 平移**启动**（只发位置指令, 不等待到位; 2026-10 单动作抓取"底盘与机械臂并行"用）:
 * 到位等待由调用方补（fChassisMoveTimeEst 估时 + 自己切片等待）—— 在飞的位置移动
 * 被下一条位置指令覆盖前必须等完（EMM 位置指令会互相覆盖, 同上面的警告）。 */
void vChassisMoveCameraLateralStart(float fMm, int iAccel, int iSpeed)
{
    vChassisMoveDistanceX(fMm, iAccel, iSpeed);
}

void vChassisMoveCameraForwardStart(float fMm, int iAccel, int iSpeed)
{
    vChassisMoveDistanceY(fMm, iAccel, iSpeed);
}

/*
 * 世界位移 (wx,wy) → 车体系分量 (side=横向, fwd=前后)
 * 按指令航向 fGetWorldHeading() 就近归到 4 个象限（硬编码查表, 不用三角函数）。
 * 世界系约定: 偏航角 0°=启动正前方(+Y), 逆时针为正; +X=启动正左方。
 *
 * 映射表 (H 就近归到 0°/90°/180°/-90°):
 *   H=0°:    fwd=+wy, side=-wx
 *   H=90°:   fwd=+wx, side=+wy
 *   H=±180°: fwd=-wy, side=+wx
 *   H=-90°:  fwd=-wx, side=-wy
 *
 * ⚠️ 调用方拿到的是 **(side, fwd)** 这个顺序: side 走 X 轴(v_x>0=右移)、fwd 走 Y 轴,
 *    与 vChassisMove / vChassisMoveDistance 的实参顺序一致。写反会在 H≠0 时把前进打到
 *    横移轴上（同 app_chassis.h 里 vChassisMoveCameraDistance 那条警告）。
 * ⚠️ fGetWorldHeading() 是**航位推算**量（只有 ucPidMove / 支点转弯 / 弧线写它）。
 *    同一个动作里要多次查表时必须**只查一次**并缓存 —— 车体真实朝向一变而累加量没跟上,
 *    象限会在中途翻掉。
 */
static void vChassisWorldToBody(float wx, float wy, float *pfSide, float *pfFwd)
{
    switch (((int)(fGetWorldHeading() + 360.0f + 45.0f) / 90) % 4) {
    case 0:  /* H≈0°   */ *pfFwd =  wy; *pfSide = -wx; break;
    case 1:  /* H≈90°  */ *pfFwd =  wx; *pfSide =  wy; break;
    case 2:  /* H≈±180°*/ *pfFwd = -wy; *pfSide =  wx; break;
    default: /* H≈-90° */ *pfFwd = -wx; *pfSide = -wy; break;
    }
}

/* 世界位移 → 车体系（vChassisWorldToBody 的公开壳, 2026-10 加）:
 * 「巡线直走」的 use_res 档要把世界系「路径残留偏移」折到车体前向分量里 ——
 * 与 vChassisMoveWorld 的 use_res 同一张象限表, 别抄第二份。
 * ⚠️ 调用方拿到的是 (side, fwd) 顺序; 航向缓存纪律同上。 */
void vChassisWorldToBodyEx(float wx, float wy, float *pfSide, float *pfFwd)
{
    vChassisWorldToBody(wx, wy, pfSide, pfFwd);
}

/* 车体系 (side=横向右正, fwd=前正) → 世界 (wx, wy)。上面那张表的逐象限逆映射:
 * H=0°: wx=-side, wy=fwd | H=90°: wx=fwd, wy=side | H=±180°: wx=side, wy=-fwd | H=-90°: wx=-fwd, wy=-side */
void vChassisBodyToWorld(float fSide, float fFwd, float *pfWx, float *pfWy)
{
    if ((pfWx == NULL) || (pfWy == NULL)) { return; }

    switch (((int)(fGetWorldHeading() + 360.0f + 45.0f) / 90) % 4) {
    case 0:  /* H≈0°   */ *pfWx = -fSide; *pfWy =  fFwd; break;
    case 1:  /* H≈90°  */ *pfWx =  fFwd;  *pfWy = fSide; break;
    case 2:  /* H≈±180°*/ *pfWx =  fSide; *pfWy = -fFwd; break;
    default: /* H≈-90° */ *pfWx = -fFwd;  *pfWy = -fSide; break;
    }
}

/*
 * 麦轮底盘"世界坐标系"平移（场向控制）
 * 把以"启动朝向"为参考的世界位移(wx,wy)按指令航向映射为车体前后/左右分量后,
 * 先 X 后 Y 两段位置移动（映射后必有一轴为 0, 见 vChassisWorldToBody）。
 * ucUseRes != 0 时先把「路径残留偏移」(Beam, 世界系 mm) 并进本次位移的计算:
 * 人质单动作抓取不回中, 车停在 路线账面位置+残留 处 —— 想落回路线原定的终点,
 * 本次就该 **计划位移 − 残留**（车已经偏过去多少就少走多少; 抓取时后撤过则
 * 表现为"多走"）。残留读出即清零（一次性, 消费走; 之后再勾一次就是 −0）。
 *
 * 2026-10-23 加「补偿后短行程保护」（只对 use_res=1 生效, 参数见 app_config.h）:
 *   补偿后车体位移 ≤ WORLD_RES_SKIP_MM → 视为已到位, 不发位置指令直接返回;
 *   补偿后车体位移 < WORLD_RES_SHORT_MM → 加速度/速度取 min(当前值, 短行程那一对)。
 * 背景: 打右靶后残留会把下一步 -460mm 缩到 ~270mm, 位置模式短行程只有加速/减速段,
 * 默认高加速度档容易冲一下/偏一下再急刹; 不是终点算错, 是短行程的出力瞬态。
 * ⚠️ **不作用于 use_res=0**（普通计划短平移保持原行为）; 也不改变终点 ——
 *    补偿后该走多少还是多少, 只是走得柔一点/极小距离不再发脉冲。
 */
void vChassisMoveWorld(float wx, float wy, uint8_t ucUseRes)
{
    float fwd, side;
    float fBodyDist = 0.0f;
    int   iAccel = (int)s_iWorldMoveAccel;
    int   iSpeed = (int)s_iWorldMoveSpeed;

    if (ucUseRes != 0u)
    {
        float fResWx, fResWy;

        vConsumePathResidual(&fResWx, &fResWy);
        wx -= fResWx;
        wy -= fResWy;
    }

    vChassisWorldToBody(wx, wy, &side, &fwd);

    if (ucUseRes != 0u)
    {
        fBodyDist = sqrtf((side * side) + (fwd * fwd));

        if (fBodyDist <= WORLD_RES_SKIP_MM)
        {
            return;
        }
        if (fBodyDist < WORLD_RES_SHORT_MM)
        {
            if (iAccel > (int)WORLD_RES_SHORT_ACCEL)
            {
                iAccel = (int)WORLD_RES_SHORT_ACCEL;
            }
            if (iSpeed > (int)WORLD_RES_SHORT_SPEED)
            {
                iSpeed = (int)WORLD_RES_SHORT_SPEED;
            }
        }
    }

    vChassisMoveDistance(side * WORLD_SIDE_SIGN, fwd * WORLD_FWD_SIGN, iAccel, iSpeed);
}

/* 仅世界 X(启动左右)方向平移，正=启动左方 */
void vChassisMoveWorldX(float wx, uint8_t ucUseRes) { vChassisMoveWorld(wx, 0.0f, ucUseRes); }

/* 仅世界 Y(启动前后)方向平移，正=启动前方 */
void vChassisMoveWorldY(float wy, uint8_t ucUseRes) { vChassisMoveWorld(0.0f, wy, ucUseRes); }

/* 世界坐标系斜向平移，正=世界斜左上（+X 左 与 +Y 前的对角）。
 * 象限映射与 vChassisWorldToBody 同源推导: 世界(+wd,+wd) 按 H 折到车体对角轴 ——
 *   H≈0°→左前 LF+ / H≈90°→右前 RF+ / H≈±180°→右后 LF- / H≈-90°→左后 RF-
 * （2026-10-09 审查修正: case1/default 原来符号写反, H 落进去会朝 180° 反向的对角
 *  走全量位移; 当时没暴露只因两处调用点都排在开局 H≈0 的正确分支） */
void vChassisMoveWorldDiagLF(float wd)
{
    switch (((int)(fGetWorldHeading() + 360.0f + 45.0f) / 90) % 4) {
    case 0:  vChassisMoveDistanceLF(+wd, s_iWorldMoveAccel, s_iWorldMoveSpeed); break;
    case 1:  vChassisMoveDistanceRF(+wd, s_iWorldMoveAccel, s_iWorldMoveSpeed); break;
    case 2:  vChassisMoveDistanceLF(-wd, s_iWorldMoveAccel, s_iWorldMoveSpeed); break;
    default: vChassisMoveDistanceRF(-wd, s_iWorldMoveAccel, s_iWorldMoveSpeed); break;
    }
}

/* 世界坐标系斜向平移，正=世界斜右上（-X 右 与 +Y 前的对角）。
 * 同一张象限表: 世界(-wd,+wd) 折到 ——
 *   H≈0°→右前 RF+ / H≈90°→右后 LF- / H≈±180°→左后 RF- / H≈-90°→左前 LF+
 * （2026-10-09 审查修正: case1 原来写的 LF+ 是 180° 反向） */
void vChassisMoveWorldDiagRF(float wd)
{
    switch (((int)(fGetWorldHeading() + 360.0f + 45.0f) / 90) % 4) {
    case 0:  vChassisMoveDistanceRF(+wd, s_iWorldMoveAccel, s_iWorldMoveSpeed); break;
    case 1:  vChassisMoveDistanceLF(-wd, s_iWorldMoveAccel, s_iWorldMoveSpeed); break;
    case 2:  vChassisMoveDistanceRF(-wd, s_iWorldMoveAccel, s_iWorldMoveSpeed); break;
    default: vChassisMoveDistanceLF(+wd, s_iWorldMoveAccel, s_iWorldMoveSpeed); break;
    }
}

/* 世界坐标系斜向平移 —— **合并档**（2026-10 加, 方向由参数选）:
 *   ucDir: 0/默认=斜左上（= 原 DIAG_LF 正向, 老行缺槽补 0 正好落在它上）
 *          1 = 斜右上（= 原 DIAG_RF 正向）  2 = 斜左下（= DIAG_LF 负向）
 *          3 = 斜右下（= DIAG_RF 负向）
 * 给积木 STEP_CHASSIS_MOVE_WORLD_DIAG_LF_WAIT 用（那块的下拉就这四档）;
 * 原 STEP_CHASSIS_MOVE_WORLD_DIAG_RF_WAIT 积木已删, 但车体原语 LF/RF 两只保留 ——
 * 合并档的四档全靠它们拼出来。 */
void vChassisMoveWorldDiag(uint8_t ucDir, float wd)
{
    switch (ucDir) {
    case 1u:  vChassisMoveWorldDiagRF(+wd); break;   /* 右上 */
    case 2u:  vChassisMoveWorldDiagLF(-wd); break;   /* 左下 */
    case 3u:  vChassisMoveWorldDiagRF(-wd); break;   /* 右下 */
    default:  vChassisMoveWorldDiagLF(+wd); break;   /* 0/越界 = 左上（安全默认） */
    }
}

/*
 * 过减速带（麦轮专用, 阻塞, 仅任务上下文）
 *   ① 低速位置模式顶住坎面, **纯延时**等车被接触反力被动转正（速度低到上不去）;
 *   ② 显式停车 + 解堵转保护;
 *   ③ 位置模式大力冲过。
 *
 * **全程不碰陀螺仪**: 不用它判断"什么时候该冲"（被动转正发生在接触后的头几百毫秒,
 * 顶住时长由调用方给足即可, 多顶一会儿只是原地磨轮胎, 比加一套判据简单也可靠），
 * 也不用它修正航向。
 * ⚠️ 代价: 顶住期间的被动转正**真实改变了车头朝向**, 而 fGetWorldHeading() 是航位推算量
 *    （只由 ucPidMove / 支点转弯 / 弧线写）, 本步不改它 → 那次转正的几度会作为航向误差
 *    留在累加器里, 影响后续世界系平移。这是**有意选的**（换取本步完全不依赖 IMU）,
 *    别再"顺手补上"—— 要治就在路线层用带 IMU 闭环的步骤（见 CLAUDE.md）。
 *
 * 为什么"顶住"用位置模式而不是速度模式: 位置模式发一条**推进量大于停车间隙**的相对位移,
 * 被坎顶住时剩余脉冲永远完不成 = 电机一直顶住 —— "顶住"是这套写法的自然产物, 不需要在
 * 循环里反复重发速度指令（那样每拍要发 4 条 EMM 帧、各含一次 ~1.1ms 忙等, 加 4 次 1ms
 * 延时, 在 20ms 节拍里占掉四成; 还要动 Beam 的底盘加速度并恢复）。
 *
 * 方向: 取**世界系前进**(0,+1) 经 vChassisWorldToBody 映射到车体轴, 三段共用同一次查表
 * 结果（push 段车会被动转正, 中途重算象限会翻）。映射后必有一轴为 0, 所以是纯单轴平移,
 * 不会走出 L 形折线。
 *
 * 标定量见 app_config.h 的 RAMP_*; 路线级约束（停车间隙、后面必须接视觉重同步）见
 * CLAUDE.md「雪弗板减速带过坎」。
 *
 * fDistMm  冲坡前进量(mm, 正=世界前方); ≤0 直接返回（防控制台新建积木默认参数全 0 空转一步）
 * iAlignMs 顶住时长(ms, 纯延时); ≤0 = 跳过顶住段只冲坡（A/B 对照用）
 */
void vChassisCrossRampWait(float fDistMm, int iAlignMs)
{
    float    fSide, fFwd, fSign;
    uint8_t  ucUseY, ucIdx;

    if (fDistMm <= 0.0f) {
        return;
    }

    /* 世界系前进方向 → 车体轴。只查这一次（见函数头注释）。 */
    vChassisWorldToBody(0.0f, 1.0f, &fSide, &fFwd);
    if (fFwd != 0.0f) { fSign = fFwd * WORLD_FWD_SIGN;  ucUseY = 1; }
    else              { fSign = fSide * WORLD_SIDE_SIGN; ucUseY = 0; }

    /* ---------- ① 低速顶住: 纯延时等被动转正 ---------- */
    if (iAlignMs > 0)
    {
        if (ucUseY) { vChassisMoveDistanceY(fSign * RAMP_ALIGN_MM, RAMP_ALIGN_ACCEL, RAMP_ALIGN_SPEED); }
        else        { vChassisMoveDistanceX(fSign * RAMP_ALIGN_MM, RAMP_ALIGN_ACCEL, RAMP_ALIGN_SPEED); }

        /* 位置模式发出去就不管了: 被坎顶住时剩余脉冲永远完不成 = 电机一直顶住。
         * 到点直接进 ②。vTaskDelay 是让出式, 顶住期间按键/OLED/传感器任务照常跑
         * （不能换成 HAL_Delay: 那是忙等, 跑路线时急停会失效）。 */
        vTaskDelay(pdMS_TO_TICKS((uint32_t)iAlignMs));
    }

    /* ---------- ② 交接: 显式停车 + 解堵转保护 ---------- */
    /* 为什么不直接发冲坡指令: 那要求"新位置指令能取代进行中的位置指令", 而这是本仓库
     * **从未验证过**的行为（既有位置移动都靠 fChassisMoveDelay 等到位后才发下一条）。
     * 显式停一下就把这个未知量消掉, 代价 8ms。
     * ResetClogPro: 本方案的核心就是故意堵转, 而堵转保护在应用层**完全不可观测**
     * （app_motor.c 把 vCallbackFcn 传成 NULL, 没有任何 EMM 应答解析）。这条命令不需要
     * 应答, 保护没触发时是空操作。 */
    for (ucIdx = 1; ucIdx <= 4; ucIdx++) {
        vEmmV5StopNow(ucIdx, 0);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    for (ucIdx = 1; ucIdx <= 4; ucIdx++) {
        vEmmV5ResetClogPro(ucIdx);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    /* ---------- ③ 大力冲坡 ---------- */
    if (ucUseY) { vChassisMoveDistanceY(fSign * fDistMm, RAMP_CROSS_ACCEL, RAMP_CROSS_SPEED); }
    else        { vChassisMoveDistanceX(fSign * fDistMm, RAMP_CROSS_ACCEL, RAMP_CROSS_SPEED); }

    /* 估时按"能达到设定转速"的梯形曲线算, 被坎挡住时实际耗时无上界 → 它是**下界**,
     * 所以必须补 RAMP_CROSS_EXTRA_MS。
     * ⚠️ 这里**不能**用 vChassisMove(0,0,0) 收尾: 那是速度模式指令, 会主动中止还在进行的
     *    位置移动, 车会骑在坎上, 紧接着的巡线横移就正对着坎横移 —— 比原来打滑更糟。 */
    fChassisMoveDelay(fDistMm, (uint16_t)RAMP_CROSS_ACCEL, (uint16_t)RAMP_CROSS_SPEED,
                      ucUseY ? AXIS_Y : AXIS_X);
    vTaskDelay(pdMS_TO_TICKS(RAMP_CROSS_EXTRA_MS));
}

/* 设定世界系移动速度/加速度，vChassisMoveWorld/X/Y 一直沿用 */
void vChassisSetWorldSpeedAccel(int accel, int speed)
{
    s_iWorldMoveAccel = accel;
    s_iWorldMoveSpeed = speed;
}

/* 设定麦轮速度模式加速度（写入 Beam 底盘加速度） */
void vChassisSetAccel(int32_t a_in)
{
    vSetChassisAccel(a_in);
}

/*
 * 麦轮底盘运动时间估算（纯计算, **不等待**; 2026-10 从 fChassisMoveDelay 拆出）:
 * fChassisMoveDelay 一等就是全程, 而"平移 × 延时异步臂"要按估时自己切片等待
 * （期间还插到点发臂）, 所以把公式拆出来公用。axis: AXIS_X / AXIS_Y。
 * 返回估算耗时(ms)。
 */
float fChassisMoveTimeEst(float distance, uint16_t a_of_car, uint16_t speed_of_car, axis_t axis)
{
    float quanshu;
    float speed;
    float accel;
    float quanshu_fenjie;
    float time_fenjie;
    float time;

    if (a_of_car == 0) { a_of_car = 256; }
    if (speed_of_car == 0)
    {
        return 200.0f;                    /* 速度 0 电机不动, 与阻塞版同口径 */
    }

    quanshu = (axis == AXIS_X) ? (ABS(distance) * CHASSIS_PULSE_PER_MM_X)
                               : (ABS(distance) * CHASSIS_PULSE_PER_MM_Y);
    speed = (float)speed_of_car;
    accel = (float)a_of_car;
    quanshu_fenjie = 320.0f * speed * speed / (accel * 6.0f);
    time_fenjie = 2000.0f * speed / accel;
    if (quanshu_fenjie >= quanshu) {
        time = time_fenjie * sqrtf(quanshu / quanshu_fenjie);
    } else {
        time = 1000.0f * (quanshu - quanshu_fenjie) / (320.0f * speed / 6.0f) + time_fenjie;
    }
    if (ABS(distance) <= 2.0f) { time = 200.0f; }
    return time;
}

/*
 * 麦轮底盘运动时间估算（并执行阻塞等待）
 * 根据梯形加减速曲线计算运动耗时，用于两个连续位置指令之间防止覆盖。
 * axis: AXIS_X=CHASSIS_PULSE_PER_MM_X, AXIS_Y=CHASSIS_PULSE_PER_MM_Y
 * 返回估算耗时(ms)；等待为 vTaskDelay 让出式，等待期间低优先级任务正常调度。
 */
float fChassisMoveDelay(float distance, uint16_t a_of_car, uint16_t speed_of_car, axis_t axis)
{
    float time = fChassisMoveTimeEst(distance, a_of_car, speed_of_car, axis);

    vTaskDelay(pdMS_TO_TICKS((uint32_t)time));
    return time;
}
