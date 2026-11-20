# tasks/_target_stream.py — 靶标流式跟踪（反恐区打靶瞄准闭环的测量源）
#
# 指令 "7" 起流 / "8" 停流：锁定靶盘后，在**小跟踪窗**内高频回传每颗靶的靶心，
# 供单片机做 10Hz 的 P 环微调舵机 6/3（见 GongKeCompetition 仓库的
# app_rpi.c::ucRpiTargetServo 与 STEP_TARGET_SERVO_WAIT）。
#
# 回传帧（TYPE_COORD TAG=0x05，见 docs/串口协议.md §6f）：
#   DATA = [TAG=0x05][status u8][count u8] + count×[color u8][cx u16 LE][cy u16 LE][r u16 LE]
#   与指令 "6" 的 0x04 帧**逐字节同布局**，区别只在它是 ~20Hz 的连续流。
#   count=0 = 本帧没找到（**流还活着**，单片机靠帧计数判新鲜度，不是失败帧）。
#
# ⚠ 与指令 "6"（一次性全画面多帧投票）是**两套并存**的方案，本文件不碰 _target.py：
#   · "6"    → 0x04 一次性结果 → 单片机 STEP_TARGET_CALIB_WAIT（开环迭代，老方案）
#   · "7/8"  → 0x05 连续流     → 单片机 STEP_TARGET_SERVO_WAIT（闭环，新方案）
#   阈值/色相族/尺寸判据全部复用 _target.py，**不另开一套**（"什么算红"只留一个出处）。
#
# ⚠ 为什么能提速而**不降分辨率**：靶外圆 r≈55px、内孔 r≈27px，精度就靠全分辨率
#   （TARGET_PROC_SCALE 的教训：降到 0.5 时遮挡用例整颗漏检）。所以提速手段是**裁**
#   （只看跟踪窗），不是**缩**。窗随目标走，丢失时逐帧扩张、自然退化成全画面重搜。
#
# ⚠⚠ 2026-10-06 帧率重构（读帧线程双缓冲）: 旧实现「cap.read() → 处理 → 发送」串行,
#   read 要等相机的下一帧（30fps 节拍 ~33ms）, 处理又排在后面, 一帧 ~45ms → 实测
#   ~22Hz, 25 的上限永远贴不到。现在后台线程**持续读、只留最新一帧**, 主循环
#   「等新帧 → 处理最新帧」: 读与处理重叠, 帧率 ≈ 1/max(相机节拍, 处理耗时),
#   贴近 30fps 的相机天花板; 且主循环拿到的永远是**最新**帧 —— 旧串行版处理慢一拍
#   时读到的是 V4L2 队列里的陈帧, 测量时效反而差。行为约定不变: 无新帧 = 不发
#   （与旧版 read 失败一致, 单片机靠帧计数判新鲜度）。
#
# ⚠ 参数每帧从 config 模块动态读取（Web「设置参数」改 TARGET_STREAM_* 即时生效），
#   不能 `from config import TARGET_STREAM_HZ`（那是导入时快照）—— 同 _line.py。
#   例外是 _detect_cfg() 里那批**检测阈值**（SAT/VAL/MIN_AREA/TOL/Q_*）：它们与 "6"
#   同源同值，起流时取一次即可（改它们本来就要重新起流才谈得上生效）。
import struct
import threading
import time

import cv2

import config
import logstore                     # 定格帧/过程帧归档（run 目录 frames/; 本模块无副作用）
from camera_lease import release_camera
from protocol import TYPE_COORD
from serial_comm import serial_comm, SerialDisconnected

# ⚠ 这里**直接 import 私有的几何件**是刻意的：_target.py 是这条链路的唯一实现来源，
#   另抄一份"小窗版"就会有两套阈值/两套判据，迟早漂。
#   若哪天这些名字被改名，表现是 ImportError（**响亮地坏掉**，不是静默失效）。
from tasks._target import (_detect_frame, _color_candidates, _locate_one,
                           _open_with_retry, _annotate, _clamp_u16, _COLOR_EN)

AIM_TAG = 0x05            # 流帧子命令标记（TYPE_COORD DATA 首字节）
COLOR_UNKNOWN = 0xFF      # 无色模式哨兵（单片机据此退化成"取离瞄准点最近的那颗"）
_HUE_FULL = [(0, 179)]    # 无色模式：色相满量程 = 只按 S/V 判据取掩膜（见 config 的说明）
_MIN_WIN_PX = 320         # 跟踪窗最小边长[px]（见 _window_of 的说明）

# 锁定一个颜色至少要命中几帧（单个 tracker 的判据）
_LOCK_MIN_VOTES = 2


def _detect_cfg():
    """跟踪/锁定共用的检测参数（与 run_target_detect 里那个 dict 同源、同值）。

    ⚠ 刻意**不含 use_roi**：锁定阶段要看**全画面**（此时还不知道靶在哪，ROI 是给
    "已经瞄上了"用的，配错会静默地什么都找不到）；跟踪阶段则用**动态跟踪窗**代替，
    比静态 ROI 更贴（见 _window_of）。
    """
    return {"sat_min": config.TARGET_SAT_MIN, "val_min": config.TARGET_VAL_MIN,
            "min_area": config.TARGET_MIN_AREA, "tol": config.TARGET_TOL,
            "q_min": config.TARGET_RING_Q_MIN, "q_max": config.TARGET_RING_Q_MAX,
            "r_max": config.TARGET_R_MAX,
            "families": config.COLOR_HUE_FAMILIES,
            "use_roi": False, "roi": (0.0, 0.0, 1.0, 1.0)}


def build_stream_frame(targets):
    """打包 0x05 流帧 DATA（见 docs/串口协议.md §6f）。

    ⚠ 函数名必须是 build_stream_frame —— 不能叫 build_result_frame /
    build_target_frame：那两个名字已被 _hostage / _color / _target 占用，
    经 tasks/__init__.py 重新导出会**互相覆盖**（那两条注释就是这么来的）。
    ⚠ 每颗靶恒 7 字节（与 0x04 逐字节同布局）；count 由 len(targets) 决定，
    故调用方必须自己保证 ≤ 单片机端的上限（3）。
    """
    data = struct.pack("<BBB", AIM_TAG, 1, len(targets))
    for t in targets:
        data += struct.pack("<BHHH", t["color"], _clamp_u16(t["cx"]),
                            _clamp_u16(t["cy"]), _clamp_u16(t["r"]))
    return data


def _send(data, what=""):
    """回传一帧；串口断开时静默丢弃（流帧丢失不致命，单片机靠帧计数判新鲜度）。"""
    try:
        serial_comm.send_frame(TYPE_COORD, data)
        return True
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 靶标流{what}回传失败: {e}")
        return False


def _send_count0(what="(未锁定)"):
    """发一帧 count=0。

    ⚠ 起流失败/还没锁上时**必须**照发，不能干脆不发：单片机的"流是否活着"判据是
    **帧计数有没有涨**，一帧都不发的话它会一直空等到超时（默认 6s）——
    而"相机打不开"或"还没锁定"明明是可以立刻报告的。
    """
    _send(build_stream_frame([]), what)


def _med(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return 0.0
    return float(xs[n // 2]) if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0


def _fmt_ts(targets):
    """靶列表 → 日志友好的一串（"红(925,524)r60 蓝(..)"）。锁定行/终态行/详细日志共用。

    trackers 与 targets 的元素同键名（color/cx/cy/r）, 两边都能喂。空列表 → "无"。
    """
    s = " ".join(f"{_COLOR_EN.get(t['color'], '?')}({int(t['cx'])},{int(t['cy'])})"
                 f"r{int(t['r'])}" for t in targets)
    return s or "无"


def _lock(frame, cfg, use_color):
    """锁定：连续采 TARGET_STREAM_INIT_FRAMES 帧全画面检测，每色取中位位置。

    判据是**该色出现 ≥_LOCK_MIN_VOTES 次**（单帧命中不算 —— 一帧误检就锁死会把整个
    闭环引到杂块上，而这正是"锁定阶段"唯一要防的事）。

    → tracker 列表，每项 = {"color": 回传用的颜色码, "cx","cy","r","lost"}。

    ⚠ 跟踪器用**列表**而不是"按颜色码索引的字典"，是为了让无色模式成立：那时所有
    跟踪器回传的颜色码都是 0xFF（同一个键），字典装不下三个 —— 见下面那段。
    """
    n = max(1, int(config.TARGET_STREAM_INIT_FRAMES))
    votes = {}
    for _ in range(n):
        try:
            found, _small = _detect_frame(frame, cfg, config.TARGET_PROC_SCALE)
        except Exception as e:                        # noqa: BLE001
            print(f"[WARN] 靶标流锁定单帧异常: {type(e).__name__}: {e}")
            found = []
        for t in found:
            votes.setdefault(t["color"], []).append((t["cx"], t["cy"], t["r"]))

    trackers = []
    for color, pts in votes.items():
        if len(pts) < _LOCK_MIN_VOTES:
            continue
        trackers.append({"color": color, "lost": 0,
                         "cx": _med([p[0] for p in pts]),
                         "cy": _med([p[1] for p in pts]),
                         "r": _med([p[2] for p in pts])})
    trackers.sort(key=lambda t: t["cx"])

    if not use_color and trackers:
        # ---- 无色模式: 只跟**一颗**，回传 color=0xFF（单片机退化成"取离瞄准点最近的那颗"）
        #
        # ⚠ 为什么必须只留一颗、且必须**换掉颜色码**：
        #   · 若照原样把三颗都报成 0xFF，单片机那条"取离瞄准点最近的"就要在**同一帧内
        #     三颗同色候选**里选 —— 它做得到，但闭环里"正在跟的那颗"会随偏差大小在
        #     三颗之间跳（瞄准点离 A 近就追 A，A 被推远一点又变成 B 最近），**会打架**；
        #   · 而 Pi 这边更是无从判断单片机在跟哪一颗，窗就没法收窄（等于退回全画面）。
        #   所以无色模式 = 单目标跟踪，这是它比带色模式差的地方之一。
        # ⚠ 挑哪一颗: 取**离画面中心最近**的。单片机那条兜底规则是"取离瞄准点最近的",
        #   而瞄准点（tg_center_x/y）默认就在画面中心附近（925,524），两者口径一致。
        #   ⚠ 这是**代理量**, 不是真值: 瞄准点若标得离画面中心很远, 两者会对不上
        #     （那就别用无色模式 —— 带色模式本来也没这个问题）。
        h, w = frame.shape[:2]
        best = min(trackers, key=lambda t: (t["cx"] - w / 2.0) ** 2
                                       + (t["cy"] - h / 2.0) ** 2)
        best["color"] = COLOR_UNKNOWN
        trackers = [best]

    return trackers


def _window_of(trackers, w, h, margin_px):
    """跟踪窗 = 各跟踪目标"上一帧位置 ± 边距"的并集 bbox（→ (x0,y0,x1,y1)）。

    ⚠ 必须是**动窗**：舵机动一下，靶心在画面里就走几百 px，锁定时的静态框立刻失效。
    ⚠ 丢失越久窗越大（margin ×(1+连续丢失帧数)，封顶半幅）→ 自然退化成全画面重搜，
    不必另写一套"重新锁定"逻辑。
    ⚠ 窗**不能**小于一个下限：目标正好停在窗边缘时，下一帧窗缩到它身上会裁掉半边轮廓，
    拟合直接崩（而崩掉又会让窗继续缩 → 死循环）。
    """
    if not trackers:
        return (0, 0, w, h)

    xs, ys = [], []
    for t in trackers:
        m = min(margin_px * (1.0 + t.get("lost", 0)), max(w, h) / 2.0)
        xs.append((t["cx"] - m, t["cx"] + m))
        ys.append((t["cy"] - m, t["cy"] + m))

    x0 = max(0, int(min(a for a, _b in xs)))
    x1 = min(w, int(max(b for _a, b in xs)))
    y0 = max(0, int(min(a for a, _b in ys)))
    y1 = min(h, int(max(b for _a, b in ys)))

    # 下限兜底：向外扩到 _MIN_WIN_PX（居中扩，再钳回画面）
    if x1 - x0 < _MIN_WIN_PX:
        cx = (x0 + x1) // 2
        x0, x1 = cx - _MIN_WIN_PX // 2, cx + _MIN_WIN_PX // 2
    if y1 - y0 < _MIN_WIN_PX:
        cy = (y0 + y1) // 2
        y0, y1 = cy - _MIN_WIN_PX // 2, cy + _MIN_WIN_PX // 2
    x0, y0 = max(0, x0), max(0, y0)
    x1, y1 = min(w, x1), min(h, y1)

    if x1 - x0 < 8 or y1 - y0 < 8:
        return (0, 0, w, h)                           # 退化：宁可慢也不要空窗
    return (x0, y0, x1, y1)


def _pick_tracked(cands, last, x0, y0, r_tol):
    """从候选里挑出"还是刚才那颗"的候选（距离门 + 半径门）。→ cand 或 None

    两道门的分工：
      · **半径门**：车/臂都不动、只有舵机在转，靶盘半径在闭环尺度内几乎不变，
        r 突变说明这是另一个东西（同色干扰块）。这是**最有力**的一道门 ——
        无色模式下更是唯一的实质判据（那个模式下没有颜色可用）。
      · **距离门**：取离上一帧位置最近的那个。窗本身就是位置门的粗版，
        这里再收一道，防的是"窗里同时出现两个同色候选"。

    ⚠ `cand["r"]` 与 `last["r"]` 是两个近似量：前者是 minEnclosingCircle(色环外轮廓)，
    后者是 _locate_one 的内边界拟合半径 ÷ 0.55（外圆等效）。两者都≈外圆半径，
    R_TOL 默认 0.35 足够吸收这点差异；**别把 R_TOL 收到 0.1 以下** ——
    那样会把真靶自己判掉（内/外边界对同一颗靶本来就能差几个百分点）。
    ⚠ 距离用**全帧坐标**比：cand 的 cx/cy 是窗内坐标（_color_candidates 已经加过
    它自己 bbox 的 ox/oy），这里要再加窗偏移 x0/y0 才和 last 同系。
    """
    best = None
    best_d2 = 0.0
    for c in cands:
        if last is not None and last.get("r", 0.0) > 0.0:
            r_ref = last["r"]
            if abs(c["r"] - r_ref) > r_ref * r_tol:
                continue
        ddx = (c["cx"] + x0) - last["cx"]
        ddy = (c["cy"] + y0) - last["cy"]
        d2 = ddx * ddx + ddy * ddy
        if best is None or d2 < best_d2:
            best, best_d2 = c, d2
    return best


def _track_frame(frame, trackers, cfg, r_tol, margin_px, use_color):
    """跟踪一帧 → (本帧回传的 targets 列表, 窗)。**就地更新** trackers。

    targets 里的坐标一律**全分辨率全帧**（与 0x04 帧同口径，固件的 tg_* 标定直接复用）。
    ⚠ 本帧没找到的跟踪器**不回传**（不是回传上一帧的陈旧值）—— 陈旧值会让单片机
    继续照着它推舵机，而那颗靶可能已经不在那了。位置冻结在 tracker 里只是为了
    "下一帧窗前瞻"，不是拿来当测量结果用的。
    """
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = _window_of(trackers, w, h, margin_px)
    sub = frame[y0:y1, x0:x1]
    if sub.size == 0:
        return [], (x0, y0, x1, y1)

    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)      # 小窗内只转一次
    out = []

    for t in trackers:
        # 带色 → 该色自己的色相族; 无色 → 满量程（只按 S/V 取掩膜）
        families = cfg["families"].get(t["color"], []) if use_color else _HUE_FULL
        cands = _color_candidates(hsv, t["color"], cfg["min_area"], cfg["sat_min"],
                                  cfg["val_min"], families,
                                  r_max_full=cfg.get("r_max"), scale=1.0)
        pick = _pick_tracked(cands, t, x0, y0, r_tol)
        loc = _locate_one(pick) if pick is not None else None
        if loc is None:
            t["lost"] = t.get("lost", 0) + 1         # 位置冻结不动, 窗下一帧扩张
            continue
        # 直接替换、**不平滑**：单片机端自己有死区与收敛判据，
        # 这里再滤一道就成了两级滤波串联，只会白加迟延（闭环最怕的就是迟延）。
        t["cx"], t["cy"], t["r"] = loc[0] + x0, loc[1] + y0, loc[2]
        t["lost"] = 0
        out.append({"color": t["color"], "cx": t["cx"], "cy": t["cy"], "r": t["r"]})

    out.sort(key=lambda d: d["cx"])                  # 按 cx 升序（与 0x04 帧同约定）
    return out, (x0, y0, x1, y1)


def _write_preview(frame, targets):
    """周期性把标注帧落到 TARGET_STREAM_LAST_FRAME_PATH（Web 预览用）。

    ⚠ 整段 try/except：落帧是**调试便利**，任何失败都不该影响流本身
    （磁盘满/路径不可写时，流必须照跑）。
    """
    try:
        import os
        path = config.TARGET_STREAM_LAST_FRAME_PATH
        os.makedirs(os.path.dirname(path), exist_ok=True)
        cv2.imwrite(path, _annotate(frame, targets, extra="stream"))
    except Exception as e:                            # noqa: BLE001
        print(f"[WARN] 靶标流落帧失败: {e}")


def _write_preview_async(frame, targets):
    """_write_preview 的异步版（2026-10-06）: imwrite+annotate 一口能吃 10~20ms,
    原来同步跑在流循环里 —— 1Hz 一次, 每秒都有一帧被它拖出尖刺。丢后台线程,
    主循环照常跑; 1Hz 的写入节奏远大于单次耗时, 不会出现两个线程写同一文件。"""
    threading.Thread(target=_write_preview, args=(frame, targets), daemon=True).start()


class _FrameFeeder:
    """后台持续 cap.read()、只留**最新一帧**的读帧线程（双缓冲, 2026-10-06 加）。

    ⚠ 只有线程自己碰 cap —— 主循环从此只经 wait_new() 拿帧, 这保证了 cap.read()
      不会被两个线程并发调用（V4L2 不答应）。
    ⚠ 时序约定: **先放帧、后推 seq** —— 主循环见 seq 变了再取 frame, 拿到的必是
      新帧（反过来就会拿到上一帧）。
    ⚠ 不需要锁: seq/frame 都是 GIL 下的原子引用赋值; frame 数组由 cap.read() 新建,
      没人就地改它。
    ⚠ 失败处理与旧版 read 不同: 不在这里 sleep+重试（那会把"读不到"变成定时阻塞）,
      只计数照常转 —— 相机拔了 seq 停涨, 主循环自然走"无新帧不发"的老路。
    """

    def __init__(self, cap):
        self._cap = cap
        self._stop = threading.Event()
        self._seq = 0            # 已成功读到的帧数
        self._frame = None       # 最新一帧
        self._fails = 0          # 连续失败数（只作诊断, 不参与控制）
        self._t = threading.Thread(target=self._loop, daemon=True)
        self._t.start()

    def _loop(self):
        while not self._stop.is_set():
            t_read = time.time()
            ret, frame = self._cap.read()
            if not ret:
                self._fails += 1
                time.sleep(0.02)
                continue
            self._fails = 0
            self._frame = frame
            self._seq += 1
            # ~40fps 节流（2026-10-06 加）: 相机本来就 30fps, 这里的空转只在
            # 「cap.read() 不阻塞」的情形出现 —— 比如接管成租约后的 LeaseHandle
            # （read 只是取最新帧的引用, 立即返回）, 那样会变成一个不眠的 Python
            # 循环, 白吃 GIL 与另一台读线程的锁。25ms 上限 ≈ 40fps > 相机 30fps,
            # 对真 cap 毫无损失（read 本身 ~33ms）, 对 LeaseHandle 则掐掉空转。
            rest = 0.025 - (time.time() - t_read)
            if rest > 0:
                time.sleep(rest)

    def wait_new(self, last_seq, abort_event, timeout=0.1):
        """等 seq 前进（或超时/中止）→ (frame|None, 当前 seq)。

        返回 frame=None 的两种情况在调用方是同一条路: 「这轮没新帧 → 不发送」。
        timeout 0.1s = 中止响应的上限（"8"停流最多 0.1s 后生效）。
        """
        deadline = time.time() + timeout
        while time.time() < deadline:
            if abort_event.is_set():
                return None, last_seq
            if self._seq != last_seq:
                return self._frame, self._seq
            time.sleep(0.005)
        if self._seq != last_seq:
            return self._frame, self._seq
        return None, last_seq

    def stop(self):
        """停线程并**等它退出**——必须在 release_camera 之前调, 否则线程还在 read
        一个已释放的 cap（段错误级别的坏法）。join 超时 1s 只是兜底。"""
        self._stop.set()
        self._t.join(timeout=1.0)


def run_target_stream(timeout=None, abort_event=None):
    """靶标流式跟踪主循环（命令 "7" 起流 / "8" 停流）。

    由 handler 的独立线程调用（`handler.py::_aim_worker`），**不同步阻塞主循环** ——
    这正是不用"6"那种阻塞式识别的理由：流期间单片机心跳照发，链路比阻塞式更健康。

    退出条件：abort_event 置位（"8"）/ 串口断开 / 超过 TARGET_STREAM_MAX_S。
    返回 dict（与其它任务函数的返回约定一致）。
    """
    abort_event = abort_event or threading.Event()
    cfg = _detect_cfg()
    cap = _open_with_retry(abort_event)
    if cap is None:
        print("[WARN] 靶标流：主摄像头打开失败，立刻回一帧 count=0 让单片机知情")
        _send_count0("(相机打不开)")
        return {"success": False, "reason": "主摄像头打开失败", "frames": 0}

    timeout = config.TARGET_STREAM_MAX_S if timeout is None else timeout
    deadline = time.time() + timeout
    t_start = time.time()
    sent = 0
    blanks = 0        # 空窗/异常计数（只用于收尾日志）
    last_log = 0.0
    sent_at_log = 0   # 上次状态日志时的 sent（用来算**区间**帧率, 见下）
    trackers = []
    # 「定格帧」候选（2026-10 加, 标靶练习数据源）: 闭环最后看着的那帧 + 它的检测结果。
    # 停流时无论收敛/超时/中止都把它归档 —— 超时的样本恰是练习最想看的。
    last_frame = None
    last_targets = []

    print("[INFO] 靶标流启动（小窗跟踪 + 门控, 读帧线程双缓冲）")
    feeder = None
    try:
        # ---- 读帧线程: 从这一刻起 cap 只归它碰（主循环经 feeder.wait_new 拿帧）。
        #      finally 里必须先 stop 再 release —— 线程还活着就释放 cap 是段错误级别的坏法。
        feeder = _FrameFeeder(cap)

        # ---- 阶段1: 锁定（全画面）。锁不到不退出: 每 _LOCK_RETRY_EVERY 帧重试一次,
        #      期间照发 count=0 帧（让单片机知道"流活着、只是还没锁上"）。----
        lock_try = 0
        seq_seen = 0
        while not abort_event.is_set() and not trackers:
            t_iter = time.time()
            if t_iter > deadline:
                print(f"[WARN] 靶标流超过安全时长 {timeout}s（一直没锁定），自动停止")
                return {"success": False, "reason": "max_duration", "frames": sent}

            if lock_try % _LOCK_RETRY_EVERY == 0:
                frame, seq_seen = feeder.wait_new(seq_seen, abort_event)
                if frame is not None:
                    last_frame = frame          # 锁定阶段退出（超时/中止）也要有定格帧
                    try:
                        trackers = _lock(frame, cfg, bool(config.TARGET_STREAM_USE_COLOR))
                    except Exception as e:                # noqa: BLE001
                        print(f"[WARN] 靶标流锁定异常: {type(e).__name__}: {e}")
                        trackers = []
                    if trackers:
                        print("[INFO] 靶标流已锁定: " + _fmt_ts(trackers))
                        break
            lock_try += 1
            _send_count0()
            sent += 1
            blanks += 1
            _sleep_rest(t_iter)

        # ---- 阶段2: 小窗跟踪 ----
        # ⚠ 计时的两个基准必须在**进入跟踪循环时**重新锚定：锁定阶段耗时 ~1s 且期间发的
        #   是空窗帧，把它们算进来会让第一行状态日志的"区间帧率"算成一个荒唐的数。
        last_log = time.time()
        sent_at_log = sent
        seq_seen = 0
        t_read_ms = 0.0    # 区间累计: 等新帧的耗时（诊断分解, 见 1Hz 日志）
        t_track_ms = 0.0   # 区间累计: 小窗检测+打包的耗时
        n_timed = 0        # 区间计的帧数（算平均值用）

        while not abort_event.is_set():
            t_iter = time.time()
            t_now = t_iter
            if t_now > deadline:
                print(f"[WARN] 靶标流超过安全时长 {timeout}s，自动停止")
                return {"success": False, "reason": "max_duration", "frames": sent}

            t0 = time.time()
            frame, seq_seen = feeder.wait_new(seq_seen, abort_event)
            if frame is None:
                continue        # 无新帧（相机没来帧/中止）: 不发 —— 与旧版 read 失败同路
            t_read_ms += (time.time() - t0) * 1000.0
            last_frame = frame

            t1 = time.time()
            try:
                targets, win = _track_frame(frame, trackers, cfg,
                                            config.TARGET_STREAM_R_TOL,
                                            config.TARGET_STREAM_MARGIN_PX,
                                            bool(config.TARGET_STREAM_USE_COLOR))
            except Exception as e:                # noqa: BLE001
                # ⚠ 单帧异常必须吞掉：一帧坏图不该杀掉整个流线程（那会让单片机看到
                # 帧计数停涨、冻结不动 —— 查起来像链路故障，其实是这里崩了）。
                print(f"[WARN] 靶标流单帧检测异常: {type(e).__name__}: {e}")
                targets, win = [], (0, 0, 0, 0)
            t_track_ms += (time.time() - t1) * 1000.0
            n_timed += 1
            last_targets = targets

            if not targets:
                blanks += 1

            if not _send(build_stream_frame(targets[:config.TARGET_MAX_TARGETS])):
                break                                 # 串口断了: 直接收摊
            sent += 1

            # 详细日志（标靶练习调试用, 默认关）: 逐帧一行, 靶心/半径/窗全量可见。
            # 与 1Hz 状态行互补 —— 那条看帧率, 这条看每颗靶的轨迹细节。
            if config.TARGET_STREAM_VERBOSE:
                print(f"[靶标流#{sent}] {_fmt_ts(targets)} "
                      f"窗={win[2] - win[0]}x{win[3] - win[1]}")

            # 1Hz 状态日志 + 落帧（Web 预览）。限流是为了不刷爆日志文件。
            if config.TARGET_STREAM_LOG_HZ > 0 and \
                    t_now - last_log >= 1.0 / config.TARGET_STREAM_LOG_HZ:
                # ⚠ 这里打的是**区间帧率**（自上次日志以来），不是"累计帧数 ÷ 总耗时"。
                #   后者在起流阶段被锁定期的空窗帧和 ~1s 的锁定耗时拖着，头几行会显示成
                #   个位数 fps（实测 1.7），看着像"帧率根本达不到" —— 而真正要判断的是
                #   "稳态下跑多少"。调 TARGET_STREAM_HZ 时看的就是这个数。
                # ⚠ 2026-10-06 加 读/算 耗时分解（区间平均）: fps 不够时一眼分清是
                #   "相机没来帧"（read 大）还是"检测太慢"（track 大）——
                #   read 大到 ~33ms = 相机 30fps 节拍, 正常; track 大 = 窗涨太大/机器忙。
                span = t_now - last_log
                fps = (sent - sent_at_log) / span if span > 0 else 0.0
                avg_read = t_read_ms / n_timed if n_timed else 0.0
                avg_track = t_track_ms / n_timed if n_timed else 0.0
                last_log, sent_at_log = t_now, sent
                t_read_ms = t_track_ms = 0.0
                n_timed = 0
                print(f"[靶标流] fps={fps:.1f} n={len(targets)} "
                      f"窗={win[2] - win[0]}x{win[3] - win[1]} "
                      f"读={avg_read:.0f}ms 算={avg_track:.0f}ms "
                      f"丢={[_COLOR_EN.get(t['color'], '?') for t in trackers if t['lost']]}")
                _write_preview_async(frame, targets)
                # 帧序列归档（标靶练习回放, aimseq_*）: 两层门控 ——
                # TARGET_STREAM_ARCHIVE_FRAMES 只在 VERBOSE=1 时才生效（联动是用户定的:
                # 详细日志关了, 磁盘归档必须连带禁用）。异步线程写盘, 不拖流循环。
                if config.TARGET_STREAM_VERBOSE and config.TARGET_STREAM_ARCHIVE_FRAMES:
                    threading.Thread(target=logstore.archive_frame,
                                     args=("aimseq", frame), daemon=True).start()

            _sleep_rest(t_iter)

    except SerialDisconnected as e:
        print(f"[WARN] 靶标流中止: 单片机串口断开（{e}）")
        return {"success": False, "reason": f"serial_disconnected: {e}", "frames": sent}
    finally:
        if feeder is not None:
            feeder.stop()       # 先停读帧线程（它还占着 cap）, 再归还相机
        release_camera(cap)     # 租约句柄 → 归还（保持打开）; 普通 cap → 真释放
        # ---- 定格帧 + 终态（2026-10 加, GKS_Web「标靶练习」页的数据源）----
        # 「闭环最后看着的那帧」: 收敛/超时/中止/串口断全都归档 —— 超时的样本恰是
        # 练习最想看的。aimfinal=原始帧, aimann=标注帧（画环与十字, 给人看）。
        # ⚠️ 文件名打进行里（frames/aimfinal_NNNN.jpg）: 练习页靠它把帧与这一发
        #    TGTS 记录精确配对, 不靠时间戳猜。任何失败只打 WARN, 不影响流收尾。
        if last_frame is not None:
            try:
                _fin = logstore.archive_frame("aimfinal", last_frame)
                logstore.archive_frame("aimann",
                                       _annotate(last_frame, last_targets, extra="final"))
                print(f"[靶标流] 定格帧: frames/{_fin}" if _fin
                      else "[WARN] 定格帧归档失败（imwrite 未成功）")
            except Exception as e:                # noqa: BLE001
                print(f"[WARN] 定格帧归档失败: {e}")
        else:
            print("[靶标流] 定格帧: 无（锁定阶段即退出, 没处理过任何帧）")
        # 终态 = trackers 最后一次的位置/半径（丢失的靶位置冻结, 见 _track_frame）。
        print(f"[靶标流] 终态: {_fmt_ts(trackers)}")
        el = max(time.time() - t_start, 1e-6)
        print(f"[INFO] 靶标流结束: 共发 {sent} 帧（{sent / el:.1f}fps，空窗 {blanks} 次）")

    return {"success": True, "reason": "stopped", "frames": sent}


_LOCK_RETRY_EVERY = 5     # 锁定阶段每几帧做一次全画面搜索（每帧都搜是白烧 CPU）


def _sleep_rest(t_iter):
    """按 TARGET_STREAM_HZ 限流：让「本轮开始 → 下一轮开始」不短于 1/hz（0 = 不限）。

    ⚠ 限的是**总周期**（处理 + 等待），不是"处理完再固定睡 1/hz" —— 后者会让实际帧率
    变成 1/(处理耗时 + 1/hz)，处理慢的时候（比如 40ms/帧）会把 25Hz 拖成 15Hz，
    而日志里的 fps 又看不出是谁吃掉的。上限 0.5s 只是防 hz 被配成 0.几 时的长眠。
    """
    hz = config.TARGET_STREAM_HZ
    if hz <= 0:
        return
    rest = 1.0 / hz - (time.time() - t_iter)
    if rest > 0:
        time.sleep(min(rest, 0.5))
