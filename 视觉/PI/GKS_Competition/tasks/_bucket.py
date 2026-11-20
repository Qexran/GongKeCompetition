# tasks/_bucket.py — 爆爆桶坐标转发（OpenMV → Pi → 单片机，2026-10-24 加）
#
# 指令 "J" 起流 / "K" 停流（handler._cmd_bucket_start/_cmd_bucket_stop）：
# **独立阶段任务**（对位时车不巡线，用户 2026-10-24 定案），与巡线流/靶标流互斥
# —— 起流前 handler 已把另两条流停掉（同 "7" 靶标流的调度纪律）。
#
# 数据源：OpenMV v3.4 的 "B,<valid>,<cx>,<cy>,<w>,<h>" 文本帧
# （openmv_serial._parse_bucket_line 解析、get_latest()["bucket"] 快照暴露）。
# 本循环只做「取 → 判新鲜 → 打包 → 发」，**检测全在相机端** —— Pi 是纯转发。
#
# 回传帧（TYPE_COORD TAG=0x0B，见 docs/串口协议.md §6m）：
#   DATA = [0x0B][status u8][cx u16LE][cy u16LE][w u16LE][h u16LE] 共 10 字节
#   status bit0 = valid；valid=0 时 cx..h 是上一帧的沿用值（相机端语义）。
#   无桶/陈旧也照发（**流活性靠帧计数**，同 0x05 靶标流语义；valid 由单片机判）。
import struct
import threading
import time

import config
from openmv_serial import openmv_reader
from protocol import TYPE_COORD
from serial_comm import serial_comm, SerialDisconnected

BUCKET_TAG = 0x0B            # COORD 帧子命令标记（须与 app_rpi.h::RPI_BUCKET_TAG 一致）


def _clamp_u16(v):
    """钳到 u16（cx/cy 名义 0..319/0..239，w/h 更小；负值/异常值钳 0）。"""
    return max(0, min(65535, int(v)))


def build_bucket_frame(valid, cx, cy, w, h):
    """打包 0x0B 流帧 DATA。

    ⚠️ 函数名必须唯一 —— build_* 家族经 tasks/__init__.py 重新导出会**互相覆盖**
    （那六条注释就是这么来的），本文件的名字刻意不与任何既有 build_* 重名。
    ⚠️ status 刻意恒 bit0 按 valid 走、**没有** count 字段：相机端每帧只报一个桶
    （"OpenMV 挑好再报"，用户 2026-10-24 定案），定长 10 字节对齐 0x08 看路帧的布局。
    """
    status = 0x01 if valid else 0x00
    return struct.pack("<BBHHHH", BUCKET_TAG, status,
                       _clamp_u16(cx), _clamp_u16(cy),
                       _clamp_u16(w), _clamp_u16(h))


def _sleep_rest(t_iter):
    """按 BUCKET_SEND_HZ 限流：让「本轮开始 → 下一轮开始」不短于 1/hz（0 = 不限）。

    ⚠ 限的是**总周期**（同 _target_stream._sleep_rest 的理由：处理慢时固定睡
    1/hz 会把帧率叠乘着掉，日志里还看不出是谁吃掉的）。
    """
    hz = float(config.BUCKET_SEND_HZ)
    if hz <= 0:
        return
    rest = 1.0 / hz - (time.time() - t_iter)
    if rest > 0:
        time.sleep(min(rest, 0.5))


def run_bucket_stream(timeout=None, abort_event=None):
    """爆爆桶坐标转发主循环（独立线程，由 handler._bucket_worker 调）。

    新鲜度：`config.BUCKET_STALE_TIMEOUT` 内没有新 B 帧（或相机未连接）→ 本拍按
    valid=0 发（坐标仍取缓存沿用值 —— 单片机靠 status bit0 判有效性、靠帧计数判
    流活性，语义与相机端一致）。

    退出条件：abort_event 置位（"K"/"x"）/ 串口断开 / 超过 timeout
    （None = config.BUCKET_MAX_S；单片机 "J,<ms>" 会把预算传进来）。
    返回 dict（与其它任务函数的返回约定一致）。
    """
    abort_event = abort_event or threading.Event()
    if timeout is None:
        timeout = float(config.BUCKET_MAX_S)
    deadline = time.time() + float(timeout)
    t_start = time.time()
    sent = 0
    stale = 0
    last_log = t_start
    sent_at_log = 0
    print("[INFO] 爆爆桶流启动")
    try:
        while not abort_event.is_set():
            t_iter = time.time()
            if t_iter > deadline:
                print(f"[WARN] 爆爆桶流超过安全时长 {timeout:.0f}s，自动停止")
                break
            snap = openmv_reader.get_latest()
            b = snap.get("bucket") or {}
            fresh = bool(snap.get("connected")) and (
                t_iter - float(b.get("timestamp", 0.0))
                <= float(config.BUCKET_STALE_TIMEOUT))
            valid = int(b.get("valid", 0)) if fresh else 0
            if not fresh:
                stale += 1
            data = build_bucket_frame(valid, b.get("cx", 0), b.get("cy", 0),
                                      b.get("w", 0), b.get("h", 0))
            try:
                serial_comm.send_frame(TYPE_COORD, data)
            except (SerialDisconnected, ValueError, OSError) as e:
                print(f"[WARN] 爆爆桶流回传失败: {e}")
                break
            sent += 1
            if config.BUCKET_LOG_HZ > 0 and t_iter - last_log >= 1.0 / config.BUCKET_LOG_HZ:
                span = t_iter - last_log
                fps = (sent - sent_at_log) / span if span > 0 else 0.0
                last_log, sent_at_log = t_iter, sent
                print(f"[爆爆桶流] fps={fps:.1f} v={valid} "
                      f"cx={b.get('cx', 0)} cy={b.get('cy', 0)} "
                      f"w={b.get('w', 0)} h={b.get('h', 0)} 陈旧拍={stale}")
            _sleep_rest(t_iter)
    finally:
        el = max(time.time() - t_start, 1e-6)
        print(f"[INFO] 爆爆桶流结束: 共发 {sent} 帧（{sent / el:.1f}fps，陈旧 {stale} 拍）")
    return {"success": True, "reason": "stopped", "frames": sent}
