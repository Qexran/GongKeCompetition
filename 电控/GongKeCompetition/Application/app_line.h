/**
  * @file       app_line.h
  * @brief      巡线闭环模块：树莓派端 steer → 麦轮横移，收敛到线上。
  *
  *             类似流式细定标（视觉误差闭环伺服）——Pi 端 LinePID 已把 OpenMV
  *             横向偏差 err 算成控制量 steer（巡线帧），单片机端只做「速度映射 +
  *             横移执行」，闭环智能在 Pi 端（PID 参数 Web 在线可调）。
  *
  *             依赖：Pi 已在转发巡线帧（见 app_rpi.c 的 vRpiTask）——脚本侧请走
  *              app_rpi.c 的组合动作 ucRpiLineConverge()——它负责发 LINE_START /
 *             LINE_STOP 并等起流。本文件的 vLineFollowConverge() 是其中的内层
 *             原语, **不自己发指令**, 直接调它会一直拿不到帧（车不动也不报错）。
  */

#ifndef __APP_LINE_H
#define __APP_LINE_H

#include <stdint.h>

/* 巡线收敛闭环（内层原语, 假定帧已在流）：读 Pi 端 steer → 横移 vx，
 * **外加航向环**：由一个源（见 emSrc）算出航向误差 → 麦轮旋转 w，
 * 两环正交、一次 vChassisMove 下发（见 app_config.h 的「航向环」那组）。
 * 在 ulConvergeMs 内让横向偏差收敛。
 * **丢线(valid=0)不停车** —— 照用 Pi 给的 steer（保持/找线/停由 Pi 的三段策略决定，
 * 见 Pi 端 tasks/_line.py）；只有「帧号超过 LINE_STALE_TIMEOUT_MS 没变」= Pi 停发/
 * 掉线才停车（防拿最后一帧跑飞）。
 * err 连续 LINE_CONVERGE_SAMPLES 次 < 阈值提前退出（该宏为 0 = 不提前退出, 跑满窗口），
 * 且只在 valid=1 时累加。
 *
 * ============ ucSrc = **航向来源**（单源三选一, 或 src=3 融合加权） ============
 * 调用方指定源（常量 `LINE_ANG_SRC_*` 定义在 app_rpi.h;
 * 为什么 2026-09 砍过一次加权平均: 见 app_config.h 的删除记录 —— 旧的**误差端**加权
 * 需要跨量纲换算尺, 尺的符号标反就会变成正反馈把车转飞。2026-10-08 加回的融合档(src=3)
 * 改在**输出端**加权, 没有任何跨源换算, 详见下面 ④ 与 app_line.c 的融合分支）:
 *   ① `LINE_ANG_SRC_CAM`  —— 相机那条: `err = 图像角 − line_angle_target`
 *      （Beam 运行时量, 上电默认 LINE_ANGLE_TARGET_DEG）。属**安装标定**（相机装歪几度
 *      填几度）, 只走蓝牙/宏两条路, 没有积木参数。**丢线(valid=0)时本拍不修航向**。
 *   ② `LINE_ANG_SRC_GYRO` —— 陀螺那条: `err = 短弧(line_gyro_target − yaw)`, 目标由
 *      「巡线收敛」积木每步给一个（src=陀螺/融合 两档都写; 蓝牙可当场覆盖）。**IMU 停摆时本拍不修航向**。
 *      它的语义是"车头咬死一个绝对世界航向", **线与航向不一致时车会斜着平移** ——
 *      要沿线拐弯只能用 ①, 或者按段改目标角。
 *   ③ `LINE_ANG_SRC_NONE` —— 不修航向（fW 恒 0）。用于「交界被遮挡」的场合（如过雪弗板
 *      减速带: 板子与交界相切, 会把一部分列的色阶行推走, 那一刻相机给的 angle 不可信）。
 *      道理: 横向环是位置环（误差有界、会自回正）, 航向环的误差却会一直把车带偏, 所以
 *      短暂不可信时只该掐掉航向那一半。
 *   ④ `LINE_ANG_SRC_FUSE` —— **融合**: 相机/陀螺两条**各自完整地走一遍自己的环**
 *      （误差 → 各自死区 → 各自增益 → w_cam / w_gyro）, 最后只把两个**输出 w** 按
 *      `相机:陀螺 = line_cam_pct : (100−line_cam_pct)` 加权（占比是 Beam 运行时量,
 *      上电默认 = 全局 LINE_CAM_PCT(50); 2026-10-30 起积木无 cam_pct 参数, 蓝牙可当场覆盖）。
 *      动机: 陀螺仪会漂、相机量的是场地的角度 —— 现场要"信相机多些、陀螺兜底"。
 *      陀螺腿咬的目标角 = 积木的 angle 槽（src=融合 也写入, 与陀螺档同一份 ±180 校验）。
 *      ⚠️ 两条的符号各自折在各自增益里（单源档各自标定过）, 凸组合**翻不了符号**;
 *      ⚠️ 某一源不可用（丢线/IMU 停摆）该源**本拍出 0 力**, 另一源仍按原权重出力 ——
 *      刻意**不**把空出的权重补给对方（增益台阶 40%↔100% 跳变会抖; 位置环自会兜住）;
 *      ⚠️ 两源量纲不同（图像角/真角度）, 占比没有"配多少等效于单源"的换算, 按 LV: 现场试。
 * ⚠️ 参数是 `uint8_t` 而非枚举: 枚举类型会逼着 case 体写 `(emXxx)I(n)` 转换, 而控制台收
 *    那一处转换时下标会错位（详见 app_rpi.h 的 LINE_ANG_SRC_* 那段, 别改）。
 * ⚠️ **任何一源不可用都只是"本拍不修"（融合档=该源出 0 力）, 绝不换源**: 两个源各有一套
 *    增益/死区（`line_cam_to_w`/`line_cam_db_deg` 与 `line_gyro_to_w`/`line_gyro_db_deg`,
 *    Beam 运行时量, 上电默认 = app_config.h 同名宏; 量纲一个是图像角、一个是真角度,
 *    数值不可互抄）, 符号也各自独立。
 *
 * iFwdRpm = **前进分量[rpm]**（体坐标系, 正=前进）, 与横移/转正一次下发:
 *   0 = 只横移 + 转正、**不往前走**（「对准」, `STEP_LINE_CONVERGE_WAIT` 走这条, 老行为）;
 *   ≠0 = **边前进边修**（「沿线走」, `STEP_PIVOT_LINE_WAIT` 的 `line_ms` 段 /
 *       `STEP_LINE_STRAIGHT_WAIT` 走这条）。
 *   ⚠️ ≠0 时"收敛时长"的含义从**对准**变成**走多远**; ⚠️ 它还会被全局的
 *   `LINE_CONVERGE_SAMPLES` 提前退出**截断行程**（要"走满时长"就得让那个宏保持 0）。
 *
 * iAccelRpmS = **本次斜坡的 EMM 加速度档位**（1~255, 每条速度指令都带它; 2026-10 加）:
 *   巡线直走要「估时公式的加速度 == 实际斜坡的加速度」mm→时间反推才成立, 所以加速度
 *   跟着调用方走（普通路径传 iGetChassisAccel() = 老行为逐字节不变）。停车/掉线停车
 *   也用同一个值 —— 估时假设的是对称梯形, 减速段不一致同样吃距离。
 *
 * ucEarlyRet = **「启动即返回」开关**（2026-10 加, 目前只有「巡线收敛」积木会传 1）:
 *   0 = 老行为, 跑满 ulConvergeMs（提前退出只归全局宏 LINE_CONVERGE_SAMPLES 管, 必须 0）;
 *   1 = 逐拍检查「收敛完成」判据（app_config.h「巡线收敛增强」区: 位置误差 + 该积木 src
 *       选中的源的航向误差 —— 融合档则**两源各判各的**（阈值 0=不判该源; 一源不可用 =
 *       该拍不满足）, 连续满足 LINE_CE_STABLE_MS）→ 达标即提前返回, 实际耗时落在
 *       0 < t < ulConvergeMs; 没达标则再给 LINE_CONV_OVERTIME_MS 的超时收敛, 期间判到
 *       随时走。⚠️ 判据总闸 LINE_CE_POS_TH = 0 时本参数整体失效（仍跑满窗口）。
 *
 * 过程遥测: 闭环**运行中**按 LINE_LIVE_TELEM_MS（app_config.h, 0=关）节拍往 Pi 日志发
 * `LV: t= e= h= vx= w= f= v=`（本拍横向偏差/航向误差/三通道指令/是否在线）——
 * 「巡线直走」边走边修时的过程录像, 光看步末的 LINE:/LW: 看不出纠正在哪。
 *
 * 仅任务上下文调用（内部 vTaskDelay）。 */
void vLineFollowConverge(uint32_t ulConvergeMs, uint8_t ucSrc, int iFwdRpm, int iAccelRpmS,
                         uint8_t ucEarlyRet);

#endif /* __APP_LINE_H */
