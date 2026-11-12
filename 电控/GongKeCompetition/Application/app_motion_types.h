/**
  * @file       app_motion_types.h
  * @brief      运动/导航子系统共享类型、枚举与参数（移植自 pro 项目 Motion/TypeDef.h）。
  *
  *             仅保留底盘运动学 + 角度 PID + 坐标变换用到的子集；视觉伺服 PI-D、
  *             舵机插值、机械臂动作序列等宏/枚举仍留在 pro 项目，不在此处。
  */

#ifndef __APP_MOTION_TYPES_H
#define __APP_MOTION_TYPES_H

/* ==================== 坐标系约定（移植自 pro Motion/TypeDef.h） ====================
 *
 *   像素坐标系 (树莓派摄像头):
 *     原点: 图像左上角；X+ 向右；Y+ 向下
 *
 *   小车坐标系 (麦轮底盘, vChassisMove):
 *     原点: 车体中心；v_x > 0 = 右移；v_y > 0 = 前进；w > 0 = 逆时针(俯视)
 *
 *   世界坐标系 (启动朝向, vChassisMoveWorld):
 *     偏航角 0° = 启动正前方(+Y)，逆时针为正；+X = 启动正左方。
 *     航向由 ucPidMove 完成后写入世界航向(Beam)，非实时 IMU。
 * ============================================================================= */

#define ABS(x)  ((x) > 0 ? (x) : -(x))

/* 分辨率选择 (fenbian_choose 参数) */
typedef enum {
    RESOLUTION_COARSE = 0x01,  /* 小分辨率/粗定位 */
    RESOLUTION_FINE   = 0x02   /* 大分辨率/精定位 */
} resolution_t;

/* 定标任务类型 (task_choose 参数) */
typedef enum {
    TASK_TURNTABLE = 0x01,  /* 转盘处定标 */
    TASK_MA_DUO    = 0x02   /* 码垛处定标 */
} task_type_t;

/* 颜色代码 —— 内部使用 */
typedef enum {
    COLOR_NONE  = 0,
    COLOR_RED   = 1,
    COLOR_GREEN = 2,
    COLOR_BLUE  = 3
} color_t;

/* 转盘槽位 */
typedef enum {
    POSITION_CENTER = 1,  /* 底部中间 */
    POSITION_LEFT   = 2,  /* 左上 */
    POSITION_RIGHT  = 3   /* 右上 */
} yp_slot_t;

/* 坐标轴选择 (底盘移动方向) */
typedef enum {
    AXIS_X = 0,
    AXIS_Y = 1
} axis_t;

/* 串口颜色标记 —— 树莓派返回包第 8 字节 */
typedef enum {
    SERIAL_COLOR_RED   = 0x31,
    SERIAL_COLOR_GREEN = 0x32,
    SERIAL_COLOR_BLUE  = 0x33,
    SERIAL_COLOR_NONE  = 0x34
} serial_color_t;

/* ==================================================================
 * PID 参数组 —— 五种控制策略的 Kp / Ki / Kd / MaxOutput
 * 移植自 pro Motion/TypeDef.h，调试时只需改此处。
 * ================================================================== */
#define PID_CALIBRATE_KP          2.0f
#define PID_CALIBRATE_KI          0.00f
#define PID_CALIBRATE_KD          0.8f

#define PID_BACK_OFF_KP           3.3f
#define PID_BACK_OFF_KI           0.00f
#define PID_BACK_OFF_KD           1.8f
#define PID_BACK_OFF_MAX_OUT      30.0f

#define PID_ROUGH_KP              5.0f
#define PID_ROUGH_KI              0.00f
#define PID_ROUGH_KD              5.0f

#define PID_FINE_KP               2.0f
#define PID_FINE_KI               0.05f
#define PID_FINE_KD               0.4f

#define PID_HOLD_KP               2.0f
#define PID_HOLD_KI               0.07f
#define PID_HOLD_KD               0.4f

#define PID_ARC_TURN_KP           2.5f
#define PID_ARC_TURN_KI           0.00f
#define PID_ARC_TURN_KD           2.5f
#define PID_ARC_TURN_MAX_OUT      80.0f

/* PID 默认参数 —— vPidInit() 初始化和 ucPidMove() 退出时恢复 */
#define PID_DEFAULT_KP             1.5f
#define PID_DEFAULT_KI             0.001f
#define PID_DEFAULT_KD             2.0f
#define PID_DEFAULT_MAX_INTEGRAL   7.0f
#define PID_DEFAULT_MAX_OUTPUT     180.0f

/* ucPidMove() 退出后恢复的 maxOutput */
#define PID_POST_MOVE_MAX_OUTPUT   230.0f

/* PID 模式选择 (pid_choose 参数) */
#define PID_MODE_CALIBRATE        0
#define PID_MODE_BACK_OFF         1
#define PID_MODE_ROUGH            2
#define PID_MODE_FINE             3
#define PID_MODE_HOLD             4
#define PID_MODE_ARC_TURN         5

/* IMU 卡死检测: 连续多少次(×50ms)无新航向数据则判定传感器故障停车 */
#define PID_IMU_STALL_LIMIT        20

/* ==================================================================
 * PID 完成策略 —— 决定 ucPidMove() 何时退出等待
 *   TIMEOUT  (0): 纯超时, 各阶段跑满指定时间
 *   CONVERGE (1): 误差连续 N 次 < 阈值提前退出
 * ================================================================== */
#define PID_STRATEGY_TIMEOUT    0
#define PID_STRATEGY_CONVERGE   1
#define PID_STRATEGY            PID_STRATEGY_TIMEOUT

#define PID_CONVERGE_THRESHOLD      0.05f
#define PID_CONVERGE_SAMPLES        5
#define PID_CONVERGE_SAFETY_MS      3000

#endif /* __APP_MOTION_TYPES_H */
