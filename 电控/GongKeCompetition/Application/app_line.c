/**
  * @file       app_line.c
  * @brief      巡线闭环模块实现（视觉误差闭环伺服, 类似流式细定标）。
  *
  *             Pi 端 tasks/_line.py 已做 LinePID(err→steer), 本模块不再重复 PID,
  *             直接读巡线帧的 steer 映射为麦轮横移速度 vx（vChassisMove(vx,0,0)）,
 *             **外加航向环**：由一个源（相机图像角 / 陀螺世界航向 / 不修）算出航向
 *             误差 → 麦轮旋转 w, 在收敛时间内让 err 归零。
 *             控制律参数分两层（2026-10-06 起）: app_config.h 的同名宏 = 上电默认,
 *             Beam 运行时量（蓝牙「巡线」组, 断电回宏）= 当场生效 —— 闭环内**每拍现取**。
 *             航向环默认**一刀切选一个源**; 2026-10-08 加回融合档（src=3, **输出端**加权,
 *             与 2026-09 删除的误差端加权不是一条路, 见下方长注释与 app_config.h 删除记录）。
 */

#include "app_line.h"
#include "app_chassis.h"     /* vChassisMoveA（斜坡加速度按实参的版本, 见函数头 iAccelRpmS） */
#include "app_beam.h"        /* vGetLineData / fGetAttitudeYaw / fGetLineAngleTarget / fGetLineGyroTarget / fGetLineCamPct */
#include "app_config.h"      /* LINE_* 参数宏 */
#include "app_rpi.h"         /* vRpiSendDebug（步末自述串）+ RPI_DATA_MAX + LINE_ANG_SRC_* */
#include "app_bt.h"          /* bt_fmt_f2（浮点格式化, 全工程不用 %f） */

#include <stdio.h>           /* snprintf（调试串） */

#include "FreeRTOS.h"
#include "task.h"

/* 源名字（只服务步末自述串）—— 认不出来的值打 `?`, **别默认成 "cam"**:
 * 那会把"积木参数写错了"伪装成正常工况, 正是排障时最不该骗人的地方。 */
static const char *c_pcLineSrcName(uint8_t ucSrc)
{
    switch (ucSrc)
    {
    case LINE_ANG_SRC_CAM:  return "cam";
    case LINE_ANG_SRC_GYRO: return "gyro";
    case LINE_ANG_SRC_NONE: return "none";
    case LINE_ANG_SRC_FUSE: return "fuse";
    default:                return "?";
    }
}

/*
 * 巡线收敛闭环（阻塞, 仅任务上下文）—— **双闭环: 位置环(在 Pi) + 航向环(单源三选一, 或 src=3 融合)**:
 *   ① 位置环: steer(横幅) × 系数 → vx, 消横向偏差（控制律在 Pi 端 LinePID）;
 *   ② 航向环: 由调用方指定的源算出航向误差 → w（单源 = 一刀切只认它; src=3 融合 = 两条
 *      各自走完自己的环、**输出端**按占比加权, 2026-10-08 加回, 见下面融合段）。
 *   与位置环正交: vx 管"横着偏多少", w 管"斜了多少"; 一次 vChassisMove 把两者一起下发。
 *   **丢线(valid=0)也照用 Pi 给的 steer**（保持/搜索/停由 Pi 决定, 见下）;
 *   帧号超过 LINE_STALE_TIMEOUT_MS 没变 = Pi 掉线/停发 → 停车;
 *   err 连续 N 次 < 阈值提前收敛退出（只在 valid=1 时累加）; 超时停车退出。
 *
 * 职责划分: 本模块只做「steer → vx 映射 + 限幅」, **丢线策略全在 Pi** —— 只有 Pi
 * 知道线丢了多久、往哪边丢、相机看没看见（见 Pi 端 tasks/_line.py: AI 保持期
 * LINE_LOST_HOLD_S → 找线期 LINE_LOST_SEARCH_S → 停）。所以 valid=0 时不再无条件
 * 停车: 那会让 Pi 的策略形同虚设, 而且丢线率一高就变成「横移↔停车」反复跳, 车几乎
 * 不前进（实测 30Hz 里六成帧丢线时, 车基本不动）。
 * 代价: 单片机完全听 Pi, 所以必须有下面那道**帧新鲜度**保护 —— 别删。
 *
 * ============ 航向环: **单源一刀切**（2026-09 定案, 替代原来的误差端加权）; src=3 融合（2026-10-08） ============
 * 调用方（ucRpiLineConverge / ucRpiPivotTurnLine 等）在 ucSrc 里说明这一次听谁:
 *   ① `LINE_ANG_SRC_CAM`  —— 相机那条: `err = 图像角 − line_angle_target`（**图像角**尺度, 贴着线走）;
 *   ② `LINE_ANG_SRC_GYRO` —— 陀螺那条: `err = 短弧(gyro目标 − yaw)`（**真角度**, 咬住一个世界航向）;
 *   ③ `LINE_ANG_SRC_NONE` —— **不修航向**（fW 恒 0）: 交界被遮挡时用（过雪弗板减速带 —— 板子与
 *      交界相切会把一部分列的色阶行推走, 那一刻相机给的 angle 不可信）。道理: 横向环是位置环
 *      （误差有界、会自回正）, 航向环的误差却会一直把车带偏, 所以短暂不可信时只该掐掉航向那一半。
 *   ④ `LINE_ANG_SRC_FUSE` —— **融合**: 相机/陀螺两条**各自完整地走一遍自己的环**
 *      （误差 → 各自死区 → 各自增益 → w_cam / w_gyro）, 最后只把两个**输出 w** 按
 *      `相机:陀螺 = line_cam_pct : (100−line_cam_pct)` 加权、再统一钳 ±line_w_max。
 *      动机: 陀螺仪会漂（累积误差）, 相机量的是**场地的角度**（不漂但要看得见线）——
 *      现场要"信相机多些、陀螺兜底"的折中, 占比由全局 LINE_CAM_PCT(出厂 50)/蓝牙给。
 *      陀螺腿咬的目标角 = 积木的 angle 槽（src=融合 也写入, 同陀螺档 ±180 校验）。
 * 每拍（增益/死区/目标/占比都**现取**, 于是蓝牙边跑边改下一拍就生效）:
 *   ① 选中的源**不可用 → 这一拍就是没有它的航向修正**（单源档 fW 保持 0; 融合档该源出 0 力、
 *      另一源照常按原权重出力, **不把空出的权重补给对方** —— 补给 = 增益台阶跳变, 会抖）:
 *        · 相机那条看 `valid`（丢线时相机算不出旋转角; 旧 Pi 丢线时仍发旧角, 少这道门
 *          就会拿陈旧倾角一直转车）;
 *        · 陀螺那条看**姿态帧号新鲜度**（见下）;
 *      自述串里的 `na=` 就是"本步有多少拍没有任何可用源" —— 排障先看它（融合档某源单独
 *      不可用不计入 na, 它的后果是 w 里少了一路分量）。
 *   ② `|err| < 本源的死区` → 不修正（融合档: 该源那条出 0 力, 另一条不受牵连）;
 *   ③ `w = err × 本源的增益` → 夹 ±line_w_max（单源档在共用段钳; 融合档加权后钳一次 ——
 *      两路同号时加权不损失满力, 异号相消本就是融合要的效果）。
 * ⚠️⚠️ **为什么 2026-09 的融合被删、2026-10-08 的融合敢加回来**（删除记录, 别混淆）:
 *    旧法是两条**误差**加权平均（权重 line_gyro_w %, 靠 line_gyro_scale 把真角度换算成图像角
 *    尺度, 再共用一个增益）。那条路有一个**能翻符号的闭环增益** —— 换算尺的符号标反时两条从
 *    "相加"变成"相减", 加权系数 `k − w(k+|s|)` 越过零点之后**负反馈变正反馈**, 车顶在
 *    LINE_W_MAX 上朝一个方向无限转（现场实测: 权重开到 50% 以上就犯, 而那个宏是出厂 1.0、
 *    从来没标定过）。
 *    新法（④）没有任何跨源换算/共用增益: 两条各自的符号折在各自增益里（line_cam_to_w /
 *    line_gyro_to_w, 单源档各自标定过、各自跑过）, 权重只作用在两个输出上 —— 对同一个物理
 *    偏角, fW = p·w_cam + (1−p)·w_gyro 是两个**已验证负反馈**的凸组合, 找不到可翻的符号。
 *    要排障时把融合档拆回单源验证: 把占比临时拨 0（=纯陀螺）或 100（=纯相机）即可, 不用改积木。
 *    **误差端加权/跨源换算尺仍然禁止加回来。**
 * ⚠️ 陀螺目标角的"绝对"**只相对开机朝向**（与 STEP_SPIN_TURN_WAIT 的 absolute=1 同口径,
 *    零点 = 开机那一刻的车头朝向）。中途转过向的段落要按 OLED 读到的 Yaw 值填（标签随源变）,
 *    别按"我转了 90° 所以现在是 90"推 —— 弧线/打滑会让路线以为的航向与 IMU 读数差几度。
 * ⚠️ 陀螺那条是**保持航向, 不是跟线**: 线与车头不一致时车会**斜着平移**（位置环在修横向偏差,
 *    但航向咬在目标角上）—— 要沿线拐弯只能用相机那条, 或者按段改目标角。
 * ⚠️ IMU 停摆为什么用**姿态帧号**而不是"yaw 有没有变": 后者分不清"陀螺停摆"与"车/电机没动"
 *    （低出力时电机死区、打滑都让 yaw 不变）, 会把**正在生效**的环掐掉。帧计数是
 *    ulGetYawFrameCount()（与 Pi 巡线帧号同一套判法）。**跟踪每拍都做**（与选了哪个源无关:
 *    免得切进陀螺模式的首拍把"刚初始化"误判成"停摆"）, 只是**只在陀螺/融合两档被采信**。
 *
 * ==================== iFwdRpm: 前进分量（2026-09 加） ====================
 * 第三通道, 与横移/转正在麦轮解算里叠加（speed[] = ±vx ±fwd ∓w, **一次下发**）:
 *   0 = 只横移 + 转正、**不往前走**（「对准」语义 —— STEP_LINE_CONVERGE_WAIT 走这条,
 *       与加这个参数之前逐字节等价）;
 *   ≠0 = **边前进边修**（「沿线走」语义 —— STEP_PIVOT_LINE_WAIT 的 line_ms 段与
 *       STEP_LINE_STRAIGHT_WAIT 走这条）。
 * ⚠️ ≠0 时"收敛时长"的含义就从**对准**变成了**走多远** —— 想走固定距离就调时长或换速度。
 * ⚠️ ≠0 的段走**「边走边修」增益组**（line_walk_steer_to_vx / line_walk_cam_to_w /
 *    line_walk_gyro_to_w）, =0 走**「原地对准」组**（line_steer_to_vx / line_cam_to_w /
 *    line_gyro_to_w）—— 前进会让横向偏差发散、歪头要靠旋转环扳平, 两组分开各调各的
 *    （2026-10-05 拆）。限幅（line_vx_max/line_w_max）与 Pi 端 LINE_KP 两组共用。
 *    以上全是 Beam 运行时量（2026-10-06 起, 上电默认 = app_config.h 同名宏）:
 *    蓝牙「巡线」组当场改, 本闭环下一拍（20ms）就生效。
 * ⚠️ 与 `LINE_CONVERGE_SAMPLES` 的交互: 那个提前退出判据（|err|≤阈值连续 N 拍）在前进模式下
 *    会**截断行程**（对准得早 → 提前结束 → 少走一段）。要"走满时长"就得让它保持 0（当前
 *    默认值就是 0）—— 它是**全局宏**, 别为其它积木把它调大。
 *
 * ==================== iAccelRpmS: 斜坡加速度（2026-10 加） ====================
 * 每条速度指令都带 EMM 的单字节加速度档位。巡线直走（STEP_LINE_STRAIGHT_WAIT）按
 * 位置模式同款估时把 mm 反推成时长 —— 估时公式里的加速度必须就是实际斜坡用的那个,
 * 所以它成了参数（普通调用方传 iGetChassisAccel() = 老行为逐字节不变）。停车
 * （正常收尾 / Pi 停发兜底）也用同一个值: 估时假设的是对称梯形, 减速斜坡不一致同样吃距离。
 *
 * ==================== ucEarlyRet: 「启动即返回」（2026-10 加） ====================
 * 0 = 老行为: 跑满 ulConvergeMs, 提前退出只归全局宏 LINE_CONVERGE_SAMPLES（必须保持 0）。
 * 1 = 「巡线收敛」积木勾了"启动即返回": 逐拍检查「收敛完成」判据（app_config.h
 *   「巡线收敛增强」区）—— 位置误差达标 **且** 该积木 src 选中的源航向误差达标（阈值 0
 *   = 不判该源; src=不修 就只判位置）, **连续**满足 LINE_CE_STABLE_MS 才算收敛 →
 *   提前返回（实际耗时 0 < t < converge_ms, "一开始就对得很齐"就几乎不等）。
 *   收敛时长内没判到 → 再给 LINE_CONV_OVERTIME_MS 的**超时收敛**（总上限 =
 *   converge_ms + 超时收敛时间）, 期间判到随时走, 到点仍未收敛才强制结束。
 * 判据只在 valid=1 的帧上累加, 丢线 / IMU 停摆 / Pi 停发那一拍直接算"不满足"并清零
 * 稳定窗 —— Pi 起流盲期发的就是 valid=0 的帧, 不会被上一次运行的陈旧快照骗成"已收敛"。
 * ⚠️ 判据总闸 LINE_CE_POS_TH = 0 → 整套增强失效, 勾了"是"也按老行为跑满（防呆）。
 * ⚠️ 与老判据（LINE_CONVERGE_SAMPLES）**互斥**: 增强启用时老判据不参与, 两套阈值别混着调。
 * 步末自述: 启用时会多发一条 `LINE conv: t=<实际> lim=<上限> r=<0|1|2>`（r=1 窗口内
 * 提前收敛 / 2 超时段收敛 / 0 跑满仍未收敛 —— 常态化 r=0 就去查阈值是否 ≥ 死区、或真没对齐）。
 *
 * ==================== 行内遥测 LV:（2026-10-05 加, LINE_LIVE_TELEM_MS=0 关） ====================
 * 过程中按节拍发 `LV: t=<s> e=<横向偏差> h=<航向误差> vx=<横移指令> w=<旋转指令>
 * f=<前进分量> v=<是否在线>` —— 「巡线直走」这类边走边修的步骤在 Pi 日志里的**过程录像**,
 * 与步末汇总（LINE:/LW:）互补: 误差怎么变、横移/旋转各打了多少、哪一拍丢的线, 全在这条里。
 * vx 符号即 vChassisMove 约定（v_x>0=右移）; w 同麦轮解算的旋转通道。丢线(v=0)的拍也发。
 */
void vLineFollowConverge(uint32_t ulConvergeMs, uint8_t ucSrc, int iFwdRpm, int iAccelRpmS,
                         uint8_t ucEarlyRet)
{
    uint32_t ulElapsed = 0;
    uint8_t  ucStable  = 0;
    float    fErr, fSteer, fAngle;
    uint8_t  ucValid;
    uint32_t ulFrameCnt   = ulGetLineFrameCount();   /* 新鲜度: 帧号变了 = Pi 还在发 */
    TickType_t xFrameTick = xTaskGetTickCount();

    /* --- 收敛增强（ucEarlyRet=1 才武装; 见函数头「启动即返回」段） ---
     * ulLimitMs 是循环的真正上限: 增强关闭时 == ulConvergeMs（老行为）, 启用时再加
     * LINE_CONV_OVERTIME_MS 的超时收敛余量。溢出保护: converge_ms 给到极大值
     * （「巡线测试」那条 1000000u 的近亲）时别让加法绕回变小。 */
    uint8_t  ucEnhance    = 0u;
    uint32_t ulLimitMs    = ulConvergeMs;
    uint32_t ulConvHold   = 0u;      /* 判据连续满足的累计时长[ms]（按 20ms 拍累加） */
    uint8_t  ucConvResult = 0u;      /* 步末自述 r=: 0 跑满未收敛 / 1 窗口内收敛 / 2 超时段收敛 */
    uint32_t ulTelemAcc   = 0u;      /* 行内遥测限速累加[ms]（节拍 = LINE_LIVE_TELEM_MS） */

    if ((ucEarlyRet != 0u) && (LINE_CE_POS_TH > 0.0f))
    {
        ucEnhance = 1u;
        ulLimitMs = (ulConvergeMs > (0xFFFFFFFFu - (uint32_t)LINE_CONV_OVERTIME_MS))
                    ? 0xFFFFFFFFu
                    : ulConvergeMs + (uint32_t)LINE_CONV_OVERTIME_MS;
    }

    /* --- 陀螺那条的 IMU 新鲜度（帧号法, 与上面 Pi 巡线帧新鲜度同一套） ---
     * ⚠️ **每拍都跟踪**, 与这一次选了哪个源无关 —— 只在陀螺模式里跟踪的话, 切进陀螺
     * 模式的首拍会把"刚在函数入口初始化"误判成"停摆"（xYawTick 还是上一个模式留下的）。
     * 跟踪照做, **采信**只在陀螺模式那一支里。 */
    uint32_t   ulYawCnt   = ulGetYawFrameCount();
    TickType_t xYawTick   = xTaskGetTickCount();
    uint8_t    ucYawStale = 0u;
    /* 自述用（步末那条 `LINE:` 串）: 本步"选的源不可用"的拍数 + 最后一次的源读数/误差/输出。
     * 三样都按**本源的量纲**读（相机=图像角, 陀螺=真角度）。 */
    uint32_t   ulNoSrc   = 0u;
    float      fDbgVal   = 0.0f;
    float      fDbgErr   = 0.0f;
    float      fDbgW     = 0.0f;
    uint8_t    ucAborted = 0u;   /* 是不是被中止请求掐停的（见循环里那条检查） */

    while (ulElapsed < ulLimitMs)    /* 增强启用时上限含超时收敛余量, 否则 == converge_ms（老行为） */
    {
        uint32_t ulNowCnt;
        uint8_t  ucNewFrame;

        /* 路线中止请求（OLED 菜单的「停止巡线测试」）—— ⚠️ 这个检查**必须在这里**,
         * 不能只靠 vScriptRun 在步骤边界中止:
         *   ① 「巡线测试」那一步的时长是 0xFFFFFFFFu（实为不限）, 只在步骤边界响应
         *      就等于**停不下来**;
         *   ② 即使时长有限, 中途把脚本任务掐掉也会让 ucRpiLineConverge() **来不及给 Pi
         *      发 LINE_STOP** —— Pi 会继续以 30Hz 推巡线帧, 白占相机 + 拖累 50ms 的
         *      vPidTask。在这里 break 之后控制权回到 ucRpiLineConverge(), 它照常收流。
         * ⚠️ 读的是**电平量**不是一次性信号: 同一个位 vScriptRun 也要看, 做成 ucTakeXxx
         *    会被这里先偷走（见 app_beam.c 该变量处的注释）。 */
        if (ucGetScriptAbortReq() != 0u)
        {
            ucAborted = 1u;
            break;
        }

        vGetLineData(&fErr, &fSteer, &fAngle, &ucValid);

        /* Pi 丢线时也照样 30Hz 发帧（只是 valid=0）, 所以「帧号停涨」只可能是
         * Pi 掉了 / 停了转发（例如到了它自己的 LINE_MAX_DURATION）。这时绝不能再
         * 用快照里的 steer —— 那是停发前的最后一帧, 车会朝那个方向一直跑。 */
        ulNowCnt   = ulGetLineFrameCount();
        ucNewFrame = (ulNowCnt != ulFrameCnt) ? 1u : 0u;   /* 新鲜度只看帧计数 */
        if (ucNewFrame != 0u)
        {
            ulFrameCnt = ulNowCnt;
            xFrameTick = xTaskGetTickCount();
        }

        /* IMU 停摆判定（只服务「世界航向」那条误差, 见函数头）: 姿态帧号停涨 = 陀螺没在
         * 喂数 → yaw 冻在旧值 → 那条误差恒定 → 车会按一个常数 w 一直转。帧回来自动续。 */
        {
            uint32_t ulNowYaw = ulGetYawFrameCount();
            if (ulNowYaw != ulYawCnt)
            {
                ulYawCnt   = ulNowYaw;
                xYawTick   = xTaskGetTickCount();
                ucYawStale = 0u;
            }
            else if ((LINE_GYRO_STALE_MS != 0u)
                     && ((xTaskGetTickCount() - xYawTick) > pdMS_TO_TICKS(LINE_GYRO_STALE_MS)))
            {
                ucYawStale = 1u;
            }
        }

        if ((xTaskGetTickCount() - xFrameTick) > pdMS_TO_TICKS(LINE_STALE_TIMEOUT_MS))
        {
            /* Pi 掉线/停发: 停车（窗口内 Pi 回来即自动恢复闭环） */
            vChassisMoveA(0, 0, 0, iAccelRpmS);
            ucStable = 0;
            ulConvHold = 0u;     /* 没有新鲜帧就谈不上"收敛中", 稳定窗从头再攒 */
        }
        else
        {
            /* steer 控制量 → 横移速度 vx（vChassisMove 约定 v_x>0=右移）。
             * valid=0 时也照做: Pi 在丢线时给的是它三段策略的输出。
             * ⚠️ 横向增益按「原地对准 / 边走边修」**分两组**（2026-10-05 拆）:
             *   iFwdRpm≠0（直走/接管沿线段）用 line_walk_steer_to_vx —— 前进本身会让
             *   横向偏差发散, 同样的 err 需要更强的横移才修得住;
             *   iFwdRpm=0（收敛/接管 settle 段）用 line_steer_to_vx, 行为不变。
             *   限幅 line_vx_max 两组共用; Pi 端 LINE_KP 也共用 —— 直走要更猛调本组增益。
             * 增益/方向/限幅全是 Beam 运行时量（2026-10-06 起, 上电默认 = 同名宏）:
             * **每拍现取** → 蓝牙「巡线」组边跑边改, 下一拍（20ms）就生效, 零重编译。 */
            float fVxLim = fGetLineVxMax();
            float fVx = fSteer * ((iFwdRpm != 0) ? fGetLineWalkSteerToVx()
                                                 : fGetLineSteerToVx()) * fGetLineVxSign();
            if (fVx >  fVxLim) fVx =  fVxLim;
            else if (fVx < -fVxLim) fVx = -fVxLim;

            /* 第二闭环: 航向环 —— 单源档**一刀切只认一个源**, src=3 走融合（ucSrc 由调用方给;
             * 为什么误差端加权不能复活见函数头删除记录）。增益/死区/目标/占比都**每拍现取**
             * （不缓存）: 于是「巡线收敛」积木写进去的起点、以及蓝牙当场改的值, 下一拍就生效。 */
            float   fW       = 0.0f;
            float   fSrcVal  = 0.0f;    /* 本源自己的读数（相机=图像角, 陀螺=yaw） */
            float   fSrcErr  = 0.0f;    /* 误差（与本源同量纲） */
            float   fGain    = 0.0f;    /* 本源的 P 增益（**有符号**, 符号折在里面） */
            float   fDb      = 0.0f;    /* 本源的死区（两源量纲不同 → 各一个宏） */
            uint8_t ucHaveSrc = 0u;     /* 这一拍有没有算出一个可用的航向误差 */
            /* 融合档专用（2026-10-08）: 两条各自走完自己的环, 输出端加权 —— 见下面分支与函数头。 */
            float   fFuseCamErr  = 0.0f;
            float   fFuseGyroErr = 0.0f;
            uint8_t ucFuseCamOk  = 0u;
            uint8_t ucFuseGyroOk = 0u;
            uint8_t ucFuseDone   = 0u;  /* 融合档已在本分支算好 fW → 跳过下面共用的死区/增益段 */

            if (ucSrc == LINE_ANG_SRC_CAM)
            {
                /* 相机那条: **只看帧有效**。丢线时相机算不出旋转角, 这一拍就是没有航向修正
                 * —— 刻意**不**切给陀螺（那是旧融合的降级路径, 见函数头的删除记录）。
                 * ⚠️ 绝不能继续用陈旧倾角: 旧 Pi 丢线时仍发旧角, 少这道门就会拿旧角一直转车。 */
                if (ucValid != 0u)
                {
                    fSrcVal   = fAngle;
                    fSrcErr   = fAngle - fGetLineAngleTarget();
                    /* 航向增益同样按「原地对准 / 边走边修」分两组（2026-10-05 拆）:
                     * 歪头前进时旋转环是把车扳平的唯一手段, 边走边修给独立的（更强的）增益。
                     * 增益/死区是 Beam 运行时量（line_cam_to_w / line_walk_cam_to_w /
                     * line_cam_db_deg, 上电默认 = 同名宏）, 每拍现取 → 蓝牙改完下一拍生效 */
                    fGain     = (iFwdRpm != 0) ? fGetLineWalkCamToW() : fGetLineCamToW();
                    fDb       = fGetLineCamDeadband();
                    ucHaveSrc = 1u;
                }
            }
            else if (ucSrc == LINE_ANG_SRC_GYRO)
            {
                /* 陀螺那条: 目标 = 人给的世界航向(零点 = 开机朝向, 同绝对转弯)。
                 * IMU 停摆 → 读数冻住 → 那条误差恒定 → 车会按常数 w 一直转 → 本拍不修
                 * （同样**不换源**: 选了它就是要它）。 */
                if (ucYawStale == 0u)
                {
                    /* 短弧归一化 —— 目标与实测都在 ±180, 不归一的话「目标 −180、实测 +179」会
                     * 算成 −359 → 照样通过死区判 → 朝错误方向满力甩车（同 app_pid.c 的处理）。 */
                    float fRaw = fGetLineGyroTarget() - fGetAttitudeYaw();

                    if (fRaw > 180.0f)       { fRaw -= 360.0f; }
                    else if (fRaw < -180.0f) { fRaw += 360.0f; }

                    fSrcVal   = fGetAttitudeYaw();
                    fSrcErr   = fRaw;               /* **真角度** —— 不再乘任何换算尺 */
                    fGain     = (iFwdRpm != 0) ? fGetLineWalkGyroToW() : fGetLineGyroToW();
                    fDb       = fGetLineGyroDeadband();
                    ucHaveSrc = 1u;
                }
            }
            else if (ucSrc == LINE_ANG_SRC_FUSE)
            {
                /* 融合档（2026-10-08 加）: 两条**各自完整地走一遍自己的环**（误差 → 死区 →
                 * 各自增益 → w_cam / w_gyro）, 最后只把两个**输出 w** 按占比加权、钳一次幅。
                 * ⚠️⚠️ 权重加在**输出端**, 不是误差端 —— 与 2026-09 删除的那条路唯一但本质
                 *    的区别: 旧法要先用换算尺把两源误差折成同一量纲（尺的符号标反 = 两条相减 =
                 *    正反馈, 见函数头删除记录）; 本分支没有任何跨源换算, 两条的符号各自折在
                 *    各自的增益里（单源档各自标定过）, 凸组合翻不了符号。
                 * 可用性: 谁可用谁出力, **不把空出的权重补给另一个源** —— 补给会让增益在
                 *    40%↔100% 间跳变（丢线恢复的瞬间又跳回去）, 位置环是位置环、短时少一路
                 *    航向出力无害。占比**每拍现取**: 蓝牙「巡线」组改完下一拍生效。 */
                float fWCam  = 0.0f;
                float fWGyro = 0.0f;

                if (ucValid != 0u)
                {
                    float fCamDb;

                    fFuseCamErr = fAngle - fGetLineAngleTarget();
                    ucFuseCamOk = 1u;
                    fCamDb      = fGetLineCamDeadband();
                    if ((fFuseCamErr >= fCamDb) || (fFuseCamErr <= -fCamDb))
                    {
                        /* 增益跟单源档同一套「原地对准 / 边走边修」分组与取值 */
                        fWCam = fFuseCamErr * ((iFwdRpm != 0) ? fGetLineWalkCamToW()
                                                              : fGetLineCamToW());
                    }
                }

                if (ucYawStale == 0u)
                {
                    float fRaw    = fGetLineGyroTarget() - fGetAttitudeYaw();
                    float fGyroDb;

                    if (fRaw > 180.0f)       { fRaw -= 360.0f; }
                    else if (fRaw < -180.0f) { fRaw += 360.0f; }

                    fFuseGyroErr = fRaw;
                    ucFuseGyroOk = 1u;
                    fGyroDb      = fGetLineGyroDeadband();
                    if ((fRaw >= fGyroDb) || (fRaw <= -fGyroDb))
                    {
                        fWGyro = fRaw * ((iFwdRpm != 0) ? fGetLineWalkGyroToW()
                                                        : fGetLineGyroToW());
                    }
                }

                if ((ucFuseCamOk != 0u) || (ucFuseGyroOk != 0u))
                {
                    float fPct  = fGetLineCamPct();       /* 0~100, resolve 已钳 */
                    float fWMax = fGetLineWMax();

                    ucHaveSrc = 1u;
                    fW = (fWCam * fPct + fWGyro * (100.0f - fPct)) / 100.0f;
                    if (fW > fWMax) fW = fWMax;
                    else if (fW < -fWMax) fW = -fWMax;
                    ucFuseDone = 1u;
                }

                /* 遥测读数报**相机**那条（权重一般偏向它; 相机丢线时报陀螺那条）——
                 * 两源量纲不同, 加权平均一个误差数没有物理意义, 所以只报一路。 */
                if (ucFuseCamOk != 0u)
                {
                    fSrcVal = fAngle;
                    fSrcErr = fFuseCamErr;
                }
                else if (ucFuseGyroOk != 0u)
                {
                    fSrcVal = fGetAttitudeYaw();
                    fSrcErr = fFuseGyroErr;
                }
            }
            /* LINE_ANG_SRC_NONE（以及任何认不出来的值）→ ucHaveSrc 保持 0 = 本拍不修航向 */

            if (ucHaveSrc == 0u)
            {
                ulNoSrc++;                          /* 自述串的 `na=`: 排障第一个看的数 */
            }
            else if (ucFuseDone == 0u)
            {
                /* 单源档: 统一在这里走死区 → 增益 → 限幅（融合档已自带, 见上）。 */
                if ((fSrcErr >= fDb) || (fSrcErr <= -fDb))
                {
                    float fWMax = fGetLineWMax();   /* Beam 运行时量（上电默认 = LINE_W_MAX） */

                    fW = fSrcErr * fGain;
                    if (fW >  fWMax) fW =  fWMax;
                    else if (fW < -fWMax) fW = -fWMax;
                }
            }

            fDbgVal = fSrcVal;      /* 供步末自述串（这一拍实际在用的源 / 读数 / 误差 / 输出） */
            fDbgErr = fSrcErr;
            fDbgW   = fW;

            /* iFwdRpm ≠ 0 = **边前进边修**（"沿线走"）, 走的就是麦轮解算的第三个通道;
             * = 0 = 只有横移 + 转正（老行为, STEP_LINE_CONVERGE_WAIT 走这条）。
             * 斜坡加速度走 iAccelRpmS（见函数头）—— 估时与实际必须是同一个值。 */
            vChassisMoveA((int)fVx, iFwdRpm, (int)fW, iAccelRpmS);

            /* --- 行内遥测（限速, LINE_LIVE_TELEM_MS=0 关）---
             * 「巡线直走」这类边走边修的步骤, 光看步末的 LW:/LINE: 根本看不出
             * **过程中**纠正在哪（误差怎么变、横移/旋转各打了多少、哪拍丢的线）
             * —— 这里按固定节拍把本拍的误差与三通道指令发进 Pi 日志（LV: 前缀）,
             * 与步末汇总互补。丢线(v=0)的拍也发: 找线/保持期间车在按 Pi 策略动,
             * 这正是最需要看见的时候。 */
            ulTelemAcc += LINE_LOOP_PERIOD_MS;
            if ((LINE_LIVE_TELEM_MS != 0u) && (ulTelemAcc >= (uint32_t)LINE_LIVE_TELEM_MS))
            {
                char acTlv[RPI_DATA_MAX];
                char acE[12];
                char acH[12];

                ulTelemAcc = 0u;
                bt_fmt_f2(acE, (int)sizeof(acE), fErr);        /* 横向偏差（Pi 归一化） */
                bt_fmt_f2(acH, (int)sizeof(acH), fSrcErr);     /* 航向误差（本源量纲; src=不修恒 0） */
                (void)snprintf(acTlv, sizeof(acTlv),
                               "LV: t=%u.%u e=%s h=%s vx=%d w=%d f=%d v=%u",
                               (unsigned)(ulElapsed / 1000u),
                               (unsigned)((ulElapsed % 1000u) / 100u),
                               acE, acH, (int)fVx, (int)fW, iFwdRpm, (unsigned)ucValid);
                vRpiSendDebug(acTlv);
            }

            /* --- 老判据（全局宏 LINE_CONVERGE_SAMPLES; 增强启用时**不参与**, 见函数头） ---
             * 收敛判据只在「真看到线」时累加: 丢线时 Pi 恒发 err=0, 少这道门就会被
             * 丢线骗成"已收敛"（LINE_CONVERGE_SAMPLES ≥ 1 时会提前退出）。
             * LINE_CONVERGE_SAMPLES = 0 表示**不提前退出**: 无论 err 多小都跑满
             * converge_ms —— 标定时用来看完整闭环过程（设 ≥1 则比赛省时间）。
             * ⚠️ 关掉提前退出后整段窗口没有中途退出口 (Pi 掉线也只是停车等),
             *    只能等 converge_ms 走完或断电。 */
            if (ucValid)
            {
                float fAbsErr = (fErr >= 0.0f) ? fErr : -fErr;
                if (fAbsErr <= LINE_CONVERGE_ERR_THRESHOLD)
                {
                    ucStable++;
                    if ((ucEnhance == 0u)
                        && (LINE_CONVERGE_SAMPLES != 0u)
                        /* (int) 别删: 宏=0 时 uint8_t>=0 是恒真比较, 会触发 #186-D 告警 */
                        && ((int)ucStable >= LINE_CONVERGE_SAMPLES))
                    {
                        break;
                    }
                }
                else
                {
                    ucStable = 0;
                }
            }
            else
            {
                ucStable = 0;
            }

            /* --- 收敛增强判据（「启动即返回」勾了"是"才参与; 定义见 app_config.h 那一区） ---
             * 位置误差达标 且 该积木 src 选中的源航向误差达标, **连续**满足
             * LINE_CE_STABLE_MS 才算"收敛完成" → 提前返回。丢线 / 源不可用那一拍 =
             * 不满足（fSrcErr 只在 ucHaveSrc=1 时是本拍真值, 别拿 0 或旧值充数）。 */
            if (ucEnhance != 0u)
            {
                uint8_t ucConvNow = 0u;

                if (ucValid != 0u)
                {
                    float fAbsPos = (fErr >= 0.0f) ? fErr : -fErr;

                    ucConvNow = (fAbsPos <= LINE_CE_POS_TH) ? 1u : 0u;

                    /* 航向判据只查本积木选中的那一个源（融合档: 两源各判各的）, 阈值 0 = 不判该源;
                     * src=不修就只判位置（它本来就不修航向, 无从谈起）。 */
                    if ((ucConvNow != 0u) && (ucSrc == LINE_ANG_SRC_FUSE))
                    {
                        /* 融合档: 两源**各判各的**（阈值 0=不判该源; 一源不可用 = 该拍不满足）
                         * —— 融合是"两个都信一点", 判收敛自然也要两个都点头, 而两源误差量纲
                         * 不同、各回各的阈值（LINE_CE_ANG_TH_CAM / _GYRO）。 */
                        if (LINE_CE_ANG_TH_CAM > 0.0f)
                        {
                            float fAbsAng = (fFuseCamErr >= 0.0f) ? fFuseCamErr : -fFuseCamErr;
                            ucConvNow = ((ucFuseCamOk != 0u) && (fAbsAng <= LINE_CE_ANG_TH_CAM)) ? 1u : 0u;
                        }
                        if ((ucConvNow != 0u) && (LINE_CE_ANG_TH_GYRO > 0.0f))
                        {
                            float fAbsAng = (fFuseGyroErr >= 0.0f) ? fFuseGyroErr : -fFuseGyroErr;
                            ucConvNow = ((ucFuseGyroOk != 0u) && (fAbsAng <= LINE_CE_ANG_TH_GYRO)) ? 1u : 0u;
                        }
                    }
                    else if ((ucConvNow != 0u) && (ucSrc == LINE_ANG_SRC_CAM)
                        && (LINE_CE_ANG_TH_CAM > 0.0f))
                    {
                        float fAbsAng = (fSrcErr >= 0.0f) ? fSrcErr : -fSrcErr;
                        ucConvNow = ((ucHaveSrc != 0u) && (fAbsAng <= LINE_CE_ANG_TH_CAM)) ? 1u : 0u;
                    }
                    else if ((ucConvNow != 0u) && (ucSrc == LINE_ANG_SRC_GYRO)
                             && (LINE_CE_ANG_TH_GYRO > 0.0f))
                    {
                        float fAbsAng = (fSrcErr >= 0.0f) ? fSrcErr : -fSrcErr;
                        ucConvNow = ((ucHaveSrc != 0u) && (fAbsAng <= LINE_CE_ANG_TH_GYRO)) ? 1u : 0u;
                    }
                }

                if (ucConvNow != 0u)
                {
                    ulConvHold += LINE_LOOP_PERIOD_MS;
                    if (ulConvHold >= (uint32_t)LINE_CE_STABLE_MS)
                    {
                        ucConvResult = (ulElapsed < ulConvergeMs) ? 1u : 2u;
                        break;
                    }
                }
                else
                {
                    ulConvHold = 0u;     /* 一拍不满足就从头再攒 */
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LINE_LOOP_PERIOD_MS));
        ulElapsed += LINE_LOOP_PERIOD_MS;
    }

    vChassisMoveA(0, 0, 0, iAccelRpmS);      /* 收敛结束/超时: 停车（斜坡同本步加速度, 见函数头） */

    /* 收敛增强的步末对账（只在该次「启动即返回」真正武装时发, 见函数头）:
     *   t   = 实际耗时   lim = 本次退出上限（收敛时长 + 超时收敛时间）
     *   r   = 0 跑满仍未收敛 / 1 收敛时长内提前收敛 / 2 超时段才收敛
     * r=1/2 就是"提早结束"在干活; **常态化 r=0** = 每次都吃满上限 —— 先查航向阈值是否
     * ≥ 对应源死区（压不进死区的误差判不到收敛）, 再谈车是不是真没对齐。 */
    if (ucEnhance != 0u)
    {
        char acConv[RPI_DATA_MAX];

        (void)snprintf(acConv, sizeof(acConv), "LINE conv: t=%ums lim=%ums r=%u",
                       (unsigned)ulElapsed, (unsigned)ulLimitMs, (unsigned)ucConvResult);
        vRpiSendDebug(acConv);
    }

    /* **步末自述串**（每步一条, 进 Pi 日志的 [STM-DBG]）—— 航向环唯一的运行期可见性:
     *   src = 这一次在听谁（cam/gyro/none/fuse; `?` = 积木参数写错了）;
     *   a   = 该源自己的读数（cam=图像角 line_angle, gyro=yaw; 融合档报相机那条, 丢线报陀螺）
     *         —— 相机那条抖多大、陀螺那条离目标还差多少, 都从这两个数直接看, 不必再连蓝牙;
     *   err = 本源的误差（与 a 同量纲）; w = 输出转速[rpm]（融合档=加权后的最终值）;
     *   na  = 本步"选的源不可用"的拍数（丢线 / IMU 停摆）—— 这个数一大就说明这一步
     *         几乎没在修航向, 排障先看它;
     *   p   = 融合相机占比[%]（**仅 src=fuse 才有**的字段: 验证"我填的占比生效没有"就在这）。
     * ⚠️ **每步都发**（原来按 gyro_w != 0 才发）: 相机模式以前一条日志都不留, 而它恰恰是
     *    默认档、也是最需要标定的那个。短串忙等无害: 此刻车已停、PID 任务空闲, 且每步只发
     *    一条（支点转弯那步会调两次本函数 → 两条 `LINE:`, 外加它自己的 `PVL:`）。
     * ⚠️ RPI_DATA_MAX=64 是**静默截断**: 单源档最长约 54 字符; 融合档多一个 p= 后最长约 62
     *    字符, 仍在线内 —— 别再往 fuse 档加字段了。 */
    {
        char acDbg[RPI_DATA_MAX];
        char acVal[12];
        char acErr[12];
        char acW[12];

        bt_fmt_f2(acVal, (int)sizeof(acVal), fDbgVal);
        bt_fmt_f2(acErr, (int)sizeof(acErr), fDbgErr);
        bt_fmt_f2(acW,   (int)sizeof(acW),   fDbgW);
        if (ucSrc == LINE_ANG_SRC_FUSE)
        {
            (void)snprintf(acDbg, sizeof(acDbg), "LINE: src=%s a=%s err=%s w=%s na=%u p=%u",
                           c_pcLineSrcName(ucSrc), acVal, acErr, acW, (unsigned)ulNoSrc,
                           (unsigned)(fGetLineCamPct() + 0.5f));
        }
        else
        {
            (void)snprintf(acDbg, sizeof(acDbg), "LINE: src=%s a=%s err=%s w=%s na=%u",
                           c_pcLineSrcName(ucSrc), acVal, acErr, acW, (unsigned)ulNoSrc);
        }
        vRpiSendDebug(acDbg);
    }

    /* --- 步末 Yaw 记录（LINE_END_YAW_DBG, 「调参」Tab 开关; 不产生任何其它行为） ---
     * 本闭环（「巡线收敛」以及共用它的支点接管沿线段 / 巡线直走 / 单动作抓取里那次并行收敛）
     * **结束**时记一条当前世界航向: 一条路线跑下来, 每段收尾的 Yaw 排在一起就能看出陀螺漂移
     * 的趋势、以及"这一段把车头转歪了多少"（下一段的 absolute 航向目标该不该跟着改）。
     * ⚠️ 读的是 fGetAttitudeYaw() = **工作航向**（ROADYAW_APPLY_EN=1 时已扣看路校准偏置,
     *    与 OLED 第 1 行、各航向闭环看到的是同一个数）。
     * ⚠️ 与 `LINE:` 的关系: 那条报"这一拍在修什么", 这条只报"收尾时的航向" —— 两者互补,
     *    也**不合并**（`LINE:` 已接近 RPI_DATA_MAX(64) 的静默截断线）。
     * ⚠️ 走 `#if` 而不是运行时 if: 关掉时连字符串带调用一起消失, 零 flash/零日志开销。 */
#if (LINE_END_YAW_DBG != 0)
    {
        char acYawDbg[RPI_DATA_MAX];
        char acYaw[12];

        bt_fmt_f2(acYaw, (int)sizeof(acYaw), fGetAttitudeYaw());
        (void)snprintf(acYawDbg, sizeof(acYawDbg), "LINE yaw: %s", acYaw);
        vRpiSendDebug(acYawDbg);
    }
#endif

    /* 被中止掐停时单独报一条 —— ⚠️ 刻意**不加进上面那条 `LINE:` 的字段**: 那条串已经接近
     * RPI_DATA_MAX(64) 的静默截断线, 加字段会把别的字段挤掉、看起来像"没生效"。这条只在
     * 真的中止时出现, 所以它一出现就说明是操作员按了「停止巡线测试」，而不是跑满时长 / Pi 掉线。 */
    if (ucAborted != 0u)
    {
        vRpiSendDebug("LINE abort: stop by request");
    }
}
