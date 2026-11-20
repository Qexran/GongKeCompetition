# tasks/_line.py — OpenMV 寻线转发任务（中转站核心）
#
# 收到单片机巡线命令（"2"）后启动（独立线程，见 handler.py::_cmd_line_start）：
#   读 openmv_reader 最新数据 → 归一化 → PID → TYPE_COORD 帧实时发单片机。
# 单片机只执行转向量，视觉/PID 全在树莓派端。
#
# 树莓派 → 单片机 巡线帧（复用 TYPE_COORD=0x02，DATA 首字节 0x01 做子命令标记，
# 与 ASCII 坐标帧（首字节可打印字符 ≥0x20）天然不冲突）：
#   DATA = [0x01][err i16 LE ×10000][steer i16 LE ×1000][angle i16 LE 0.1°][valid u8]  共 8B
#   err   = 归一化偏差 [-1,1]
#   steer = PID 输出
#   valid = 0 时 err/steer 无意义，单片机自行处理（保持/停车）
#
# ⚠ 参数每帧从 config 模块动态读取（**不能** `from config import LINE_KP` —— 那是导入时
#   快照）。⚠️ 2026-10 勘误: Web 设置页保存的是 GKS_Web 进程的副本, 对跑在本进程(main.py,
#   gks.service)里的这条循环**不即时生效** —— 改完要 `sudo systemctl restart gks`。
#
# 2026-09 丢线策略升级：OpenMV 帧新增第 5 字段 state（丢线分类），丢线时按它
# **定向找线**，取代旧版「一律按踩到右边界往固定方向找」的死写方向；
# state=4/旧固件 走旧三段兜底。
#
# ⚠️⚠️ state 的语义**以相机盘 main.py 头部为准**，别再照抄「2=边界远出画面 / 3=压到黄区」
#   —— 那两代语义在 main.py 头部「第三次↔第四次」之间翻过一次（2026-09-27），现在
#   **st = 车在哪一面**：2 = 压灰区（off 相对「直线基线 base_off」衰减 Δ<−15）、
#   3 = 压黄区（off 抬升 Δ>+15）、4 = 说不清。锚点同样由相机盘给出：
#   **st=2 → 让 off 增大（回边界）；st=3 → 让 off 减小（回路面）**。
#   ⚠️ 数字沿用、含义换过 —— 所以「数字」本身不能当方向用，方向一律由
#   _toward_steer() 推导，再乘上 LINE_SEARCH_STEER 的极性（见下）。
#
# ⚠️⚠️ 定向找线的方向**同时**受两个东西管，少一个方向就会反（2026-10-01 修：
#   「OpenMV 已经没有边界远出这个概念了，但收到方向 2 时车一直朝一个方向靠」）：
#     ① _toward_steer() —— LINE_ERR_SIGN / LINE_SETPOINT 推导出的「让 off 增大」方向;
#     ② LINE_SEARCH_STEER 的**符号** = 整体极性（正=沿用①, 负=整体反过来）。
#   ②以前被一次 abs() 吃掉了，于是现场在 Web 上翻这个符号一点作用都没有（而标签却写着
#   「符号=方向」）—— 现在它是真的极性开关，翻它就能把找线方向整个倒过来，
#   不用改相机盘、也不用动 LINE_ERR_SIGN（那会把 PID 一起翻）。
#
# ⚠⚠ 启动盲期（2026-09 修，别再当成"丢线"）：OpenMV 是**按需发送**的（收到 'G' 才开始
#   发），所以刚起线程那几帧 openmv_reader 的缓存里是**上一次运行**留下的快照
#   （fresh=False → 下发 valid=0）。旧版把它当丢线处理，而此刻 last_steer is None
#   （本次还没见过线）+ lost_s≈0 ≤ 预算 → 直接落到「方向不明的兜底找线」，下发**固定
#   方向的满幅 LINE_SEARCH_STEER**。单片机在 valid=0 时**照常执行 steer**（app_line.c），
#   于是**每一局开场车都会朝一个方向猛横移一下**（实测 ~2 帧，100ms 量级），OpenMV
#   起流后才正常 —— 就是日志里每次「[巡线] …⚠数据过期」紧跟的那一下。
#   现在「本次运行还没拿到任何新鲜数据」单独一档：steer 恒 0（帧照发、valid=0）。
#   ⚠ 判据是**有没有拿到过新鲜数据**，不是 data["valid"]: 缓存里那个 valid 是陈的
#     （实测有 有效=1 却是陈帧的），只有 fresh 才说明传感器真的在发。
import struct
import threading
import time

import config
from openmv_serial import openmv_reader
from protocol import TYPE_COORD
from serial_comm import serial_comm, SerialDisconnected

# 巡线帧子命令标记（TYPE_COORD DATA 首字节）
LINE_TAG = 0x01


class LinePID:
    """位置式 PID，积分状态跨迭代保持。"""

    def __init__(self):
        self.reset()

    def reset(self):
        self.integ = 0.0
        self.prev_err = None    # None=首帧（微分置 0）
        self._d_filtered = 0.0

    def update(self, err, kp, ki, kd, i_limit, d_filter):
        """单步更新，返回控制量 steer。

        i_limit: 积分限幅（绝对值）; d_filter: 微分一阶低通系数（0=不滤波）。
        """
        self.integ += err
        self.integ = max(-i_limit, min(i_limit, self.integ))

        if self.prev_err is None:
            d_raw = 0.0
        else:
            d_raw = err - self.prev_err
        # 一阶低通：新值 = α·旧值 + (1-α)·原始值，α 越大越平滑
        self._d_filtered = d_filter * self._d_filtered + (1.0 - d_filter) * d_raw
        self.prev_err = err

        return kp * err + ki * self.integ + kd * self._d_filtered


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _toward_steer():
    """「让 off 增大」的 steer 符号（+1/-1）—— 也就是 PID 在 offset=-100 那一端
    会输出的方向（那条链是稳的，所以它的输出必然把 off 拉回 setpoint 一侧）。
    相机盘的锚点是「st=2 → 让 off 增大（回边界）/ st=3 → 让 off 减小（回路面）」，
    所以 st=2 直接用它、st=3 取反；LINE_ERR_SIGN / LINE_SETPOINT 改了这里自动跟着变，
    找线方向不再死写（旧版丢线一律按「踩到右边界」往一个方向找，偏进路里时越找越偏）。

    ⚠️ 它只给出「相对方向」，**符号（整体极性）由调用方再乘一次 LINE_SEARCH_STEER 的
    正负**（见调用处的 `pol`）—— 两处都要对，方向才对。"""
    return 1.0 if config.LINE_ERR_SIGN * ((-100.0 - config.LINE_SETPOINT) / 100.0) > 0 else -1.0


def _build_frame(err, steer, angle, valid):
    """组 8B 巡线帧 DATA。err/steer 为 float，定点化后打包。

    ⚠️ 2026-10 加两道防线（配置被毒化时的最后一道闸, 入口侧的拒绝见 config._coerce）:
      ① steer 显式钳位 ±1.0 —— 协议语义上 steer 就是归一化转向量, LINE_KP 被改大后
         无钳位会一直放大到定点化才静默饱和, 中间是异常转速;
      ② 任一量非有限（nan/inf: 参数毒化的典型产物）→ 整帧按 valid=0、量全 0 下发,
         决不让 int(round(nan)) 把巡线线程炸死（旧版抛 ValueError 直接杀死线程, 车失导引）。"""
    import math
    if not (math.isfinite(err) and math.isfinite(steer) and math.isfinite(angle)):
        return struct.pack("<BhhhB", LINE_TAG, 0, 0, 0, 0)
    steer = _clamp(steer, -1.0, 1.0)
    err_q = _clamp(int(round(err * 10000)), -32768, 32767)
    steer_q = _clamp(int(round(steer * 1000)), -32768, 32767)
    angle_q = _clamp(int(angle), -32768, 32767)
    return struct.pack("<BhhhB", LINE_TAG, err_q, steer_q, angle_q, 1 if valid else 0)


def run_line_follow(abort_event=None):
    """巡线转发主循环。由 handler 的巡线线程调用，正常不应被主循环直接同步调用。

    退出条件：abort_event 置位 / 串口断开（SerialDisconnected）/ 超时保护。
    返回 dict（与 QR 任务返回约定一致）。
    """
    abort_event = abort_event or threading.Event()
    pid = LinePID()
    t_start = time.time()
    sent = 0
    lost_frames = 0
    last_log = 0.0          # 上次打印原始值的时刻（0 = 首帧立即打印）

    last_steer   = None       # 最后一次「有效帧」算出的 steer; None = 还没见过线
                              # （丢线保持期沿用它; 从没见过线时保持期无事可做）
    last_valid_t = t_start    # 最后一次有效帧的时刻（丢线时长从这里算）
    first_fresh_t = None      # 本次运行第一帧**新鲜**数据的时刻; None = OpenMV 还没开始发
                              # （启动盲期, 那一档 steer 恒 0 —— 见文件头与循环里那一段）

    print("[INFO] 巡线转发启动")
    try:
        while not abort_event.is_set():
            if time.time() - t_start > config.LINE_MAX_DURATION:
                print(f"[WARN] 巡线超过最大时长 {config.LINE_MAX_DURATION}s，自动停止")
                return {"success": True, "reason": f"max_duration ({config.LINE_MAX_DURATION}s)",
                        "aborted": False, "frames": sent}

            data = openmv_reader.get_latest()
            now = time.time()
            # 数据过期或未连接 → 这一帧不算有效（valid=0 仍发帧，让单片机知情）。
            # ⚠ 本次运行**一帧新鲜数据都还没拿到**时是「启动盲期」（缓存里是上一次运行的
            #   快照）→ 下面丢线分支里单独一档，steer 恒 0，别按丢线去下发找线。
            fresh = data["connected"] and (now - data["timestamp"]) <= config.LINE_STALE_TIMEOUT
            valid = 1 if (fresh and data["valid"]) else 0
            if fresh and first_fresh_t is None:
                first_fresh_t = now     # 传感器时钟起点（启动盲期到此结束）

            # state 只在数据新鲜时可信（OpenMV 断流时缓存是陈的）—— 提前算好，下面的
            # 定向找线与末尾的打印共用同一份（打印挪到 steer 算完之后，见那里）。
            st = data.get("state", 0) if fresh else 0
            lost_src = ''      # 本帧 steer 的来源（打印用）：定向/确认/保持/兜底/停/盲期

            if valid:
                # OpenMV offset 名义 [-100,100] → err 归一化 [-1,1]
                # 目标位置 = 相机零点 + LINE_SETPOINT（单位同 offset, 名义 ±100;
                # 0 = 停在相机认为的"正中", 非 0 = 稳定偏在线某一侧）
                err = config.LINE_ERR_SIGN * ((data["offset"] - config.LINE_SETPOINT) / 100.0)
                steer = pid.update(err, config.LINE_KP, config.LINE_KI, config.LINE_KD,
                                   config.LINE_I_LIMIT, config.LINE_D_FILTER)
                angle = data["angle"]
                # 满偏/钉边：offset 顶到 ±100，说明交界线已经贴到画面边缘（相机装得偏/车
                # 停在边界旁），此时 PID 只剩「方向」这点信息、输出被 KP 压得很小 —— **工作
                # 点不在画面中央时灰侧尤其如此**（SETPOINT=-65 现场: 灰侧从目标到钉死只有
                # 35 个 offset 单位, err 上限 0.35 → steer 上限 0.246 ≈ 7rpm, 车偏得再远
                # 误差也不涨 —— 2026-10-05 日志实测「灰区修正慢」的根因, 50 帧全钉 +0.246）。
                # 配了搜索速度就在两种情形下**直接改用搜索速度**（方向跟 err 符号 = PID 会
                # 走的方向）:
                #   ① |err| ≥ 1 —— err 量程打满（黄侧: err 可到 1.65, 原有路径）;
                #   ② offset 钉在 ±100 且 err 指向回程 —— 灰侧「钉死但 err 到不了 1」的情形。
                #     ⚠️ 要求 err 指向回程: SETPOINT 恰好标在 ±100 上时 err≈0, 没有"回程方向"
                #     可言, 不能凭空满幅推（防止把停在目标上的车推走）。
                # ⚠️ 这里**有意不吃** LINE_SEARCH_STEER 的极性（pol）—— 它的方向来自**本帧
                #   实时的 err 符号**（= PID 自己会走的方向），是对的；乘了 pol 反而会反。
                #   pol 只管丢线那一支（相机瞎着、没有 err 可看的时候）。
                # ⚠️ LINE_SEARCH_STEER 从此身兼两职: 丢线找线速度 + 满偏/钉边救援速度
                #   （幅度共用, 现值 1.0 = 满幅; 嫌猛在 Web 上调小, 重启 gks 后生效）。
                _off_pin = data["offset"]
                _at_rail = (abs(_off_pin) >= 100.0
                            and ((_off_pin < 0.0 and err > 0.0)
                                 or (_off_pin > 0.0 and err < 0.0)))
                if (abs(err) >= 1.0 or _at_rail) and config.LINE_SEARCH_STEER != 0.0:
                    mag = abs(config.LINE_SEARCH_STEER)
                    steer = _toward_steer() * mag if err > 0 else -_toward_steer() * mag
                last_steer   = steer          # 留给丢线保持期用
                last_valid_t = now
            else:
                # 丢线。err/angle 均置 0（协议约定：valid=0 时二者无意义；航向环在
                # 单片机侧自动切陀螺咬住目标航向），PID 内部状态冻结不清零（恢复后无
                # 积分跳变）。steer 分三档：
                #   【启动盲期】本次运行**一帧新鲜数据都还没拿到** → steer 恒 0, 见下。
                #   【方向明确】OpenMV 报 state=2（压灰区）/ 3（压黄区）：先静默
                #     LINE_LOST_CONFIRM_S 躲单帧毛刺，其后按 state 定向找线 —— 这是
                #     2026-09 的主路径，取代旧版「一律按踩到右边界」的死写方向
                #     （偏进路里时旧版会越找越偏）。方向 = _toward_steer() × 极性(见下)。
                #   【方向不明】state=4 或找线关闭：沿用旧三段（保持 → 固定方向找线 → 停），
                #     此时 LINE_SEARCH_STEER 的**符号**才是那个兜底猜测的方向。
                # 两种路都受 LINE_LOST_HOLD_S + LINE_LOST_SEARCH_S 总预算封顶，超时停
                # （防跑飞）。⚠️ 找线段是**开环**的（相机正瞎着），所以方向一错就是
                # 一路推到底 —— 预算别给太长，方向先用日志里的 steer= / 找线= 核实。
                # 单片机在 valid=0 时也照常执行 steer（app_line.c），只是航向环切陀螺
                # —— 所以找线帧不用伪造 valid=1。
                lost_frames += 1
                err, angle = 0.0, 0.0
                if first_fresh_t is None:
                    # 启动盲期: 这次运行还**没见过任何新鲜数据**, 谈丢线为时过早
                    # （缓存里那组是上一次运行留下的, 拿它当现状 = 每局开场朝一个
                    #  凭空的固定方向猛横移一下, 见文件头）。
                    # ⚠ 帧照发（valid=0, steer=0）: 单片机靠**帧计数推进**判「Pi 起流了」,
                    #   不发帧会被判成「Pi 没起流」（长鸣 + 之后整段不动车）; 而 steer=0
                    #   意味着它这一刻只是原地停着等, 不会朝任何方向动。
                    # ⚠ OpenMV 真没接/没上电时也会一直停在这儿（旧版是朝一个凭空的固定
                    #   方向找线 10s）—— 这是有意的: 一点传感器信息都没有时不动车,
                    #   日志里每秒一条 ⚠OpenMV未连接 / ⚠数据过期 就是提示。
                    steer = 0.0
                    lost_src = '盲期'
                else:
                    # 丢线时长: 从最后一次有效帧算起, 但**不把启动盲期算进去** ——
                    # 传感器时钟从第一帧新鲜数据才起算, 否则刚起流时 lost_s 已经吃掉了
                    # 起流耗时, 保持/确认窗会被白扣一截。
                    lost_s = now - max(last_valid_t, first_fresh_t)
                    steer_mag = abs(config.LINE_SEARCH_STEER)
                    # 定向找线的**整体极性** = LINE_SEARCH_STEER 的符号（2026-10-01 起真的
                    # 生效了）：正 = 沿用 _toward_steer() 推导的方向，负 = 整个反过来。
                    # 以前这一支只取 |本值|（abs），符号被吞掉 → 现场在 Web 上翻符号完全
                    # 没反应（标签却写着「符号=方向」），症状「收到方向 2 车一直朝一边靠」。
                    # 要临时反一下方向就把它改成负数，不必动 LINE_ERR_SIGN（那会连 PID
                    # 一起翻）也不必改相机盘。
                    pol = -1.0 if config.LINE_SEARCH_STEER < 0.0 else 1.0
                    budget_s = config.LINE_LOST_HOLD_S + config.LINE_LOST_SEARCH_S
                    if st in (2, 3) and steer_mag != 0.0 and lost_s <= budget_s:
                        if lost_s < config.LINE_LOST_CONFIRM_S:
                            # 确认期：沿用车丢线前正在执行的 steer（它本就朝修正方向打）
                            steer = last_steer if last_steer is not None else 0.0
                            lost_src = '确认'
                        else:
                            # 相机盘锚点: st=2 → 让 off 增大（回边界）/ st=3 → 让 off 减小
                            toward = _toward_steer() * pol
                            steer = toward * steer_mag if st == 2 else -toward * steer_mag
                            last_steer = steer
                            lost_src = f'定向(st={st})'
                    elif last_steer is not None and lost_s <= config.LINE_LOST_HOLD_S:
                        steer = last_steer       # 旧三段①: 短丢线不打断动作
                        lost_src = '保持'
                    elif steer_mag != 0.0 and lost_s <= budget_s:
                        steer = config.LINE_SEARCH_STEER   # 旧三段②: 方向不明的兜底找线
                        lost_src = '兜底'
                    else:
                        steer = 0.0              # 旧三段③: 停（防跑飞）
                        lost_src = '停'

            # OpenMV 原始值 + **本端这一帧实际下发的 steer** 打印。
            # ⚠️ 挪到 steer 算完之后（原来打在分支之前，所以看不到 steer）。
            # 为什么要打 steer：丢线时车往哪边动以前**完全看不见**，只能跑去 OLED 读
            #   Spd:vx —— 排「找线方向反了」全靠这一列；再配 找线= 那格就能一眼分清
            #   是「定向」在推、还是旧三段的「保持/兜底/停」。
            # 限流（config.LINE_LOG_HZ，0=关）——转发循环 30Hz，逐帧打印会刷爆日志文件。
            # ⚠ 打的是缓存快照：data["valid"] 是 OpenMV 自己报的，未必等于下发的 valid，
            #   故未连接/过期时额外标一句，避免日志里一串好数字其实是陈的。
            if config.LINE_LOG_HZ > 0 and now - last_log >= 1.0 / config.LINE_LOG_HZ:
                last_log = now
                if not data["connected"]:
                    note = " ⚠OpenMV未连接"
                elif not fresh:
                    note = " ⚠数据过期"
                else:
                    note = ""
                # 2/3 的语义见相机盘 main.py 头部（st = 车在哪一面，不是旧的「出画方向」）
                st_note = "" if data["valid"] else f" 丢线方向={st} (2=压灰区/3=压黄区/4=不明)"
                lost_note = f" 找线={lost_src}" if lost_src else ""
                print(f"[巡线] 偏差={data['offset']} 航向角={data['angle']}(0.1度) "
                      f"有效={data['valid']} steer={steer:+.3f}{st_note}{lost_note}{note}")

            serial_comm.send_frame(TYPE_COORD, _build_frame(err, steer, angle, valid))
            sent += 1
            time.sleep(1.0 / max(config.LINE_SEND_HZ, 1.0))
    except SerialDisconnected as e:
        print(f"[WARN] 巡线中止: 单片机串口断开（{e}）")
        return {"success": False, "reason": f"serial_disconnected: {e}", "aborted": False,
                "frames": sent}
    finally:
        print(f"[INFO] 巡线转发结束: 共发 {sent} 帧（丢线 {lost_frames}）")

    return {"success": True, "reason": "stopped", "aborted": True, "frames": sent}
