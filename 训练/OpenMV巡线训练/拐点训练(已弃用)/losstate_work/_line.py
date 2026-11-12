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
# ⚠ 参数每帧从 config 模块动态读取（Web 设置页调 PID 即时生效），
#   不能 `from config import LINE_KP`（那是导入时快照）。
#
# 2026-09 丢线策略升级：OpenMV 帧新增第 5 字段 state（丢线方向分类），丢线时按它
# **定向找线**（state=2 边界远出画面 → 向边界横移；state=3 压到黄区 → 向路内横移），
# 取代旧版「一律按踩到右边界往固定方向找」的死写方向；state=4/旧固件 走旧三段兜底。
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
    """「向边界方向横移」的 steer 符号（+1/-1），由 err→steer 同一条标定链推导：
    offset=-100 = 边界远在画面上方（车偏进路里），此刻 err =
    LINE_ERR_SIGN*(-100-SETPOINT)/100，其符号正是 PID 会输出的、把车带向边界的
    steer 方向。LINE_ERR_SIGN / LINE_SETPOINT 改了这里自动跟着变 —— 找线方向
    不再死写（旧版丢线一律按「踩到右边界」往一个方向找，偏进路里时越找越偏）。"""
    return 1.0 if config.LINE_ERR_SIGN * ((-100.0 - config.LINE_SETPOINT) / 100.0) > 0 else -1.0


def _build_frame(err, steer, angle, valid):
    """组 8B 巡线帧 DATA。err/steer 为 float，定点化后打包。"""
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

    print("[INFO] 巡线转发启动")
    try:
        while not abort_event.is_set():
            if time.time() - t_start > config.LINE_MAX_DURATION:
                print(f"[WARN] 巡线超过最大时长 {config.LINE_MAX_DURATION}s，自动停止")
                return {"success": True, "reason": f"max_duration ({config.LINE_MAX_DURATION}s)",
                        "aborted": False, "frames": sent}

            data = openmv_reader.get_latest()
            now = time.time()
            # 数据过期或未连接 → 按丢线处理（valid=0 仍发帧，让单片机知情）
            fresh = data["connected"] and (now - data["timestamp"]) <= config.LINE_STALE_TIMEOUT
            valid = 1 if (fresh and data["valid"]) else 0

            # OpenMV 原始值打印：归一化/PID 之前的传感器读数，用来确认「数据到底有没有在流」。
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
                st_note = ("" if data["valid"]
                           else f" 丢线方向={data.get('state', 0)}"
                                " (2=边界远出/3=压黄区/4=不明)")
                print(f"[巡线] 偏差={data['offset']} 航向角={data['angle']}(0.1度) "
                      f"有效={data['valid']}{st_note}{note}")

            if valid:
                # OpenMV offset 名义 [-100,100] → err 归一化 [-1,1]
                # 目标位置 = 相机零点 + LINE_SETPOINT（单位同 offset, 名义 ±100;
                # 0 = 停在相机认为的"正中", 非 0 = 稳定偏在线某一侧）
                err = config.LINE_ERR_SIGN * ((data["offset"] - config.LINE_SETPOINT) / 100.0)
                steer = pid.update(err, config.LINE_KP, config.LINE_KI, config.LINE_KD,
                                   config.LINE_I_LIMIT, config.LINE_D_FILTER)
                angle = data["angle"]
                # 满偏：offset 顶到 ±100，说明交界线已经贴到画面边缘（相机装得偏/车停在
                # 边界旁），此时 PID 只剩「方向」这点信息、输出被 KP 压得很小。配了搜索速度
                # 就**直接改用搜索速度**（方向跟 err 符号），别再指望那点 PID 输出把车挪回来。
                if abs(err) >= 1.0 and config.LINE_SEARCH_STEER != 0.0:
                    mag = abs(config.LINE_SEARCH_STEER)
                    steer = _toward_steer() * mag if err > 0 else -_toward_steer() * mag
                last_steer   = steer          # 留给丢线保持期用
                last_valid_t = now
            else:
                # 丢线。err/angle 均置 0（协议约定：valid=0 时二者无意义；航向环在
                # 单片机侧自动切陀螺咬住目标航向），PID 内部状态冻结不清零（恢复后无
                # 积分跳变）。steer 分两路：
                #   【方向明确】OpenMV 报 state=2（边界远出画面, 车偏进路里 → 向边界横移）
                #     或 state=3（压到黄区 → 向路内横移）：先静默 LINE_LOST_CONFIRM_S
                #     躲单帧毛刺，其后按 state 定向找线 —— 这是 2026-09 的主路径，取代
                #     旧版「一律按踩到右边界」的死写方向（偏进路里时旧版会越找越偏）。
                #   【方向不明】state=4 或找线关闭：沿用旧三段（保持 → 固定方向找线 → 停），
                #     此时的固定方向才是真正的兜底猜测。
                # 两种路都受 LINE_LOST_HOLD_S + LINE_LOST_SEARCH_S 总预算封顶，超时停
                # （防跑飞）。单片机在 valid=0 时也照常执行 steer（app_line.c），只是
                # 航向环切陀螺 —— 所以找线帧不用伪造 valid=1。
                lost_frames += 1
                lost_s = now - last_valid_t
                err, angle = 0.0, 0.0
                # state 只在数据新鲜时可信（OpenMV 断流时缓存是陈的）
                st = data.get("state", 0) if fresh else 0
                steer_mag = abs(config.LINE_SEARCH_STEER)
                budget_s = config.LINE_LOST_HOLD_S + config.LINE_LOST_SEARCH_S
                if st in (2, 3) and steer_mag != 0.0 and lost_s <= budget_s:
                    if lost_s < config.LINE_LOST_CONFIRM_S:
                        # 确认期：沿用车丢线前正在执行的 steer（它本就朝修正方向打）
                        steer = last_steer if last_steer is not None else 0.0
                    else:
                        toward = _toward_steer()
                        steer = toward * steer_mag if st == 2 else -toward * steer_mag
                        last_steer = steer
                elif last_steer is not None and lost_s <= config.LINE_LOST_HOLD_S:
                    steer = last_steer       # 旧三段①: 短丢线不打断动作
                elif steer_mag != 0.0 and lost_s <= budget_s:
                    steer = config.LINE_SEARCH_STEER   # 旧三段②: 方向不明的兜底找线
                else:
                    steer = 0.0              # 旧三段③: 停（防跑飞）

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
