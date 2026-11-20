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
# ⚠ 参数每帧从 config 模块动态读取（Web「设置参数」改 TARGET_STREAM_* 即时生效），
#   不能 `from config import TARGET_STREAM_HZ`（那是导入时快照）—— 同 _line.py。
#   例外是 _detect_cfg() 里那批**检测阈值**（SAT/VAL/MIN_AREA/TOL/Q_*）：它们与 "6"
#   同源同值，起流时取一次即可（改它们本来就要重新起流才谈得上生效）。
import struct
import threading
import time

import cv2

import config
import camera_picam      # pi(CSI) 那条路（优先看靶, 起不来/中断由 run_target_stream 退回主摄）
from camera import CameraManager
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
_WIN_R_FACTOR = 1.30      # 半窗 ≥ 靶半径×本值（见 _window_of: CSI 满分辨率 r≈180~238,
                          # 固定 150 的边距会把靶环裁掉 → 丢帧振荡）。1.3 给运动/拟合留余量。
_MAX_MOVE_PX = 400.0      # 单帧可信位移上限[px, 全分辨率]（见 _track_frame）。
                          # 舵机一次修正实测 ≤250px/拍、流 6~12fps → 正常帧间位移远小于它;
                          # 超过它的"位移"是误测（跳到同色干扰块/窗扩带来的假运动），
                          # 既不喂速度也不做跨色补偿，免得一个坏点把三色一起带偏。

# 锁定最多采几帧（第 1 帧够 2 色就早退, 见 _lock）
_LOCK_MAX_FRAMES = 2
# 彩色模式下"差色"时的帧数上限（2026-11-02 加, 见 _lock 的现场修正注释）:
# 三色齐 → 第 1 帧就收工; 只差一色 → 最多读到第 6 帧（每帧 ~43ms 降采样）。
_LOCK_PARTIAL_FRAMES = 6
# 连续锁定失败这么多次（按 lock_try 计, 每 _LOCK_RETRY_EVERY 次一条）之后改用全分辨率
_LOCK_FULL_SCALE_AFTER = 2 * 5
# CSI **没预热**时起流最多等多久[s]（见 _picam_up_wait）
_PICAM_COLD_WAIT_S = 0.3


def _detect_cfg(pi=False):
    """跟踪/锁定共用的检测参数（与 run_target_detect 里那个 dict 同源、同值）。

    ⚠ 刻意**不含 use_roi**：锁定阶段要看**全画面**（此时还不知道靶在哪，ROI 是给
    "已经瞄上了"用的，配错会静默地什么都找不到）；跟踪阶段则用**动态跟踪窗**代替，
    比静态 ROI 更贴（见 _window_of）。
    ⚠ `pi` **只影响半径上限那一个值**（两台相机的取流分辨率不同 ⇒ 同一距离下真靶的 r
      不同: 主摄 r≈90~110px、CSI 满分辨率 r≈238px）。这条路上**主摄保持 TARGET_R_MAX
      原值** = 与改造前逐字节相同, 放宽只放宽 CSI 那一把（TARGET_R_MAX_PI）。
    """
    return {"sat_min": config.TARGET_SAT_MIN, "val_min": config.TARGET_VAL_MIN,
            "min_area": config.TARGET_MIN_AREA, "tol": config.TARGET_TOL,
            "q_min": config.TARGET_RING_Q_MIN, "q_max": config.TARGET_RING_Q_MAX,
            "fallback_q_min": config.TARGET_FALLBACK_Q_MIN,
            "r_max": (getattr(config, "TARGET_R_MAX_PI", config.TARGET_R_MAX)
                      if pi else config.TARGET_R_MAX),
            # 「白心救回」也只给 pi 开（与 run_target_detect 那条路同源同义）: 靶整颗贴在
            # 同色背景上时两者合并成巨块, 靠内孔（白心）救回; 主摄保持不变。
            "rescue_hole": bool(pi),
            "families": config.TARGET_HUE_FAMILIES,
            "use_roi": False, "roi": (0.0, 0.0, 1.0, 1.0)}


def build_stream_frame(targets, cam_is_pi=False):
    """打包 0x05 流帧 DATA（见 docs/串口协议.md §6f）。

    ⚠ 函数名必须是 build_stream_frame —— 不能叫 build_result_frame /
    build_target_frame：那两个名字已被 _hostage / _color / _target 占用，
    经 tasks/__init__.py 重新导出会**互相覆盖**（那两条注释就是这么来的）。
    ⚠ 每颗靶恒 7 字节（与 0x04 逐字节同布局）；count 由 len(targets) 决定，
    故调用方必须自己保证 ≤ 单片机端的上限（3）。

    ⚠️ **status 是位域（2026-10-16 加, 与 0x04 同一套）**: bit0 = 帧有效（恒 1, 老语义）,
    **bit7 = 本次测量来自 pi(树莓派 CSI)摄像头**。单片机**每一拍**都按它选瞄准列
    （CSI 与主摄各一把尺）, 因为流可能**中途**从 CSI 退回主摄（见 run_target_stream）。
    ⚠️ 帧长没变: 流帧本来就恒带 status=1, 老单片机根本不读它 ⇒ 加位等于白送。
    """
    status = 0x01 | (0x80 if cam_is_pi else 0)
    data = struct.pack("<BBB", AIM_TAG, status, len(targets))
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


def _send_count0(what="(未锁定)", cam_is_pi=False):
    """发一帧 count=0。

    ⚠ 起流失败/还没锁上时**必须**照发，不能干脆不发：单片机的"流是否活着"判据是
    **帧计数有没有涨**，一帧都不发的话它会一直空等到超时（默认 6s）——
    而"相机打不开"或"还没锁定"明明是可以立刻报告的。
    """
    _send(build_stream_frame([], cam_is_pi), what)


def _med(xs):
    xs = sorted(xs)
    n = len(xs)
    if n == 0:
        return 0.0
    return float(xs[n // 2]) if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0


def _read_frame(src, cap, pi):
    """取一帧 → (ok, frame)。pi 那路返回的是 CSI 的最新 BGR 帧（陈帧会被判成失败）。

    ⚠ 两台相机在这里被抹平成同一个接口（同 _target.py 的做法）:
      检测/跟踪的全部判据一行都不用知道自己在看哪台相机的图。
    """
    if pi:
        return src.read()
    frame = CameraManager.read(cap, "Main Camera", max_retries=3)
    return (frame is not None), frame


def _lock(read_one, cfg, use_color, scale=1.0):
    """锁定：在**不同的帧**上做全画面检测，每色取中位位置。→ tracker 列表

    ⚠️⚠️ 2026-10-23 修了一个**纯浪费**：原来这里是「把**同一帧**连续检测
    `TARGET_STREAM_INIT_FRAMES` 次」（调用方只取了一帧就传进来）—— 三份结果
    **逐像素相同**，"投票"一直是自投自票，白烧 2/3 的锁定时间。5MP 全分辨率下
    一次检测 ~0.6~1.0s，这正是「平移射靶」那一步 3s 锁定的主要来源。
    现在：第 1 帧采到 **≥2 色**就收工（这一步的目标就是三色并排，2 色足够进跟踪）；
    只采到 1 色 / 0 色才再取一帧合并（最多 `_LOCK_MAX_FRAMES` 帧）。中位仍然保留。

    ⚠️ 锁定用**降采样**（`TARGET_STREAM_LOCK_SCALE`，缺省 0.5）：这一步只要"靶大概
    在哪"，精定位留给跟踪阶段（`_track_frame` 的窗内不降采样）。0.5 下 5MP→1.25MP，
    实测单帧 164ms→43ms（台架），命中数不变。缩太狠会漏小靶 —— 所以调用方在连续
    锁定失败后会切回 1.0 重试（见 _run_stream_once）。

    ⚠ 判据从"该色出现 ≥2 次（原来的 _LOCK_MIN_VOTES）"改成"**至少出现一次**"：配合
      上面的多帧语义，单帧命中现在就是全部信息（以前"2 票"其实是同一帧的 2 次复制）。
      误锁的兜底在跟踪阶段（半径门 + 距离门 + 窗跟随），不在这里。
    """
    # ⚠️⚠️ 假锁过滤（2026-10-30 加）：`_detect_frame` 的兜底路径只要「有孔 + q≥地板」
    #    就收，现场实测会锁到**角落里的同色小色块**（日志实锤 `R(1151,33)r15`），
    #    之后跟踪阶段每帧都丢、还要等 5 帧才重试锁定。锁定期宁缺毋滥：
    #    ① 半径地板 min_r（按相机尺 r_max 的 12%：pi 42px / 主摄 24px，真靶 ≥90px）;
    #    ② 同一色若有"像靶环"（q 接近设计值 0.55 / 白心救回的巨块）的票，只用那些票。
    min_r = max(20.0, 0.12 * float(cfg.get("r_max") or 0.0))
    q_ring = float(cfg.get("q_min", 0.35))

    votes = {}
    frame = None            # 无色模式下面要用它算画面中心；一帧都没读到时别 NameError
    # ⚠️ 只锁到 1 色时**多给一帧**（2026-10-30）：锁定只会为"出现在票里的颜色"建跟踪器，
    #    跟踪阶段**不会**去搜没建跟踪器的颜色 —— 只锁到 1 色就意味着另外两色整条流都不跟。
    #    三色并排的场地上 1 色几乎都是这一帧漏检（遮挡/运动/白心被窗边切到），再看一帧
    #    多半能凑齐。多这一帧 ~50~160ms，比"整条流只跟一颗靶"便宜。
    # ⚠️⚠️ 2026-11-02 现场修正（射靶精调超时）: 彩色模式下"**差一色**"也照此多读几帧
    #    （上限 `_LOCK_PARTIAL_FRAMES`）。原判据是"≥2 色就早退"，而现场那次锁定发生在
    #    粗调 112mm 位移**刚发完、车还在动**的时候（`SHOT fix` 与 `"7"` 同一秒），第 1 帧
    #    漏了绿靶 → 只锁 R/B 就收工 → 整条流没有绿跟踪器 → 精调永远选不到瞄准色 →
    #    `SHOT settle s=0` → 超时判失败（日志 16:47:38 `G(170,1360)r21` 是那一帧里
    #    角落里的小绿块, 被半径地板正确剔除; 事后同机位静止帧 22/22 三色齐, 算法没问题）。
    #    成本: 只有"确实差色"时每帧 ~43ms（降采样）, 最多 4 帧; 三色齐时仍是第 1 帧收工。
    need = 3 if use_color else 2            # 彩色: 三色齐; 无色: 两色就够（老行为）
    extra_max = _LOCK_PARTIAL_FRAMES if use_color else (_LOCK_MAX_FRAMES + 1)
    frame_idx = 0
    while (frame_idx < _LOCK_MAX_FRAMES
           or (0 < len(votes) < need and frame_idx < extra_max)):
        frame_idx += 1
        ok, frame = read_one()
        if not ok or frame is None:
            continue
        try:
            found, _small = _detect_frame(frame, cfg, scale)
        except Exception as e:                        # noqa: BLE001
            print(f"[WARN] 靶标流锁定单帧异常: {type(e).__name__}: {e}")
            found = []
        for t in found:
            if float(t.get("r", 0.0)) < min_r:
                print(f"[WARN] 靶标流锁定忽略疑似假锁: "
                      f"{_COLOR_EN.get(t['color'], '?')}({int(t['cx'])},{int(t['cy'])})"
                      f"r{int(t['r'])} < 半径地板 {min_r:.0f}px")
                continue
            votes.setdefault(t["color"], []).append(
                (t["cx"], t["cy"], t["r"], float(t.get("q", 0.0)),
                 bool(t.get("merged", False))))
        if len(votes) >= need:                        # 凑齐（彩色 3 色 / 无色 2 色）就早退
            break

    trackers = []
    for color, pts in votes.items():
        # 同色有"像靶环"的票时，别让兜底捞上来的杂块（q 很低）参与中位
        ring_like = [p for p in pts if p[3] >= q_ring * 0.75 or p[4]]
        use = ring_like or pts
        trackers.append({"color": color, "lost": 0, "vx": 0.0, "vy": 0.0,
                         "cx": _med([p[0] for p in use]),
                         "cy": _med([p[1] for p in use]),
                         "r": _med([p[2] for p in use])})
    trackers.sort(key=lambda t: t["cx"])

    if not use_color and trackers and frame is not None:
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

    ⚠️ 必须是**动窗**：舵机动一下，靶心在画面里就走几百 px，锁定时的静态框立刻失效。
    ⚠️ 丢失越久窗越大（margin ×(1+连续丢失帧数)，封顶半幅）→ 自然退化成全画面重搜，
    不必另写一套"重新锁定"逻辑。
    ⚠️ 窗**不能**小于一个下限：目标正好停在窗边缘时，下一帧窗缩到它身上会裁掉半边轮廓，
    拟合直接崩（而崩掉又会让窗继续缩 → 死循环）。

    ⚠️⚠️ 2026-10-23 **边距必须跟着靶半径走**（本次实测的死锁）：
    原来半窗恒等于 margin_px（默认 150）。那套数是**按主摄**标的（主摄 r≈90~110，
    150 够放下整颗靶）。改用 pi(CSI) 满分辨率看靶后同一个靶 r≈180~238 —— **半窗比靶
    半径还小**，于是窗把靶环四边全裁掉: 内孔还在但外缘成了四条对角弧，`_locate_one`
    要么拟合出错误的圆、要么被圆度判据拒 → 判"丢失" → 窗按 lost 扩张 → 重新捕获
    (lost 归零) → 窗又缩回 150 → **再裁再丢**。这是一个**自激振荡**:
    日志里 `窗=1364x320 丢=['B']` 与 `窗=1704x1944 丢=['G']` 交替出现, 帧率被
    5MP 全幅帧拖到 2~3fps（2026-10-23 02:31/02:38 两次射击）。
    修法: 半窗 = max(固定边距, r × _WIN_R_FACTOR)。主摄 r=93 → 仍用 150（行为不变）;
    CSI r=190 → 247，整颗靶(±190)加运动余量全在窗内。
    ⚠️ 半径取**上一帧测量值**（_locate_one 已经按内孔反推成外径口径, 与 _pick_tracked
    的半径门同源）；本轮还没测过的（刚锁定）用锁定值。

    ⚠️⚠️ 2026-10-30 加**速度前瞻**（窗心 = 上一帧位置 + 该跟踪器上一帧的位移）：
    舵机 6/3 一次修正可达 250px，10fps 下靶可能在**两帧之间**整体平移两三百 px。
    只靠固定边距时，靶正好压在窗边 → 色环被窗边裁断 → 白心不再是**闭合孔** →
    `_color_candidates` 的白心救回/内孔拟合全都失效 → 该色整帧判丢。丢一帧后窗
    才扩张，追得慢一拍，串起来就是「红/蓝靶一丢就是好几帧」。
    前瞻把窗**先推到靶将要去的地方**，比放大边距便宜（放大 = 每帧都多算像素）。
    ⚠️ 速度夹在 ±base 内：误测出的大速度不会把窗甩出画面。丢失期间 `_track_frame`
       把 vx/vy 逐帧减半，窗口自然刹住，不会一直跑。
    """
    if not trackers:
        return (0, 0, w, h)

    xs, ys = [], []
    for t in trackers:
        # 半径-aware 的基准半窗（见 docstring）；r 缺失/为 0 时退回固定边距
        base = max(float(margin_px), float(t.get("r", 0.0)) * _WIN_R_FACTOR)
        base = min(base, max(w, h) / 4.0)      # 防异常 r（误测成巨块）把窗撑到全幅
        vx = max(-base, min(base, float(t.get("vx", 0.0))))
        vy = max(-base, min(base, float(t.get("vy", 0.0))))
        px, py = t["cx"] + vx, t["cy"] + vy
        m = min(base * (1.0 + t.get("lost", 0)), max(w, h) / 2.0)
        xs.append((px - m, px + m))
        ys.append((py - m, py + m))

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


def _size_of(c):
    """候选参与**半径门**的尺寸：内孔反推的外圆等效半径优先（无内孔才用外接圆）。

    ⚠️⚠️ 2026-10-30 这是本文件最重要的一处修正（现场「带补光灯时红/蓝靶反复丢」的根因）:
    跟踪器里存的 `r` 是 `_locate_one` 给的**内孔反推外径**（基准稳定），而候选的
    `cand["r"]` 是 `minEnclosingCircle(色环掩膜)` —— **靶与同色背景粘连时它会被撑大**
    （蓝靶贴蓝柜子/红靶贴纸箱; 实测同一颗蓝靶的 minEnclosingCircle 在窗口里
    139~158px 之间随背景连通程度跳，等效外径恒 184px）。拿前者当门去卡后者 =
    同一个靶**自己把自己判丢**，丢一帧后窗扩大、粘连更多 → 更过不了 → 自激振荡。
    补光灯把背景打亮后粘连更厉害, 所以现象是「开灯才坏」。
    `r_eq` 由 `_target.py::_color_candidates` 提供（内孔半径 ÷ 0.55，与回传 r 同口径）。
    """
    r_eq = c.get("r_eq")
    return float(r_eq) if r_eq else float(c["r"])


def _pick_tracked(cands, last, x0, y0, r_tol):
    """从候选里挑出"还是刚才那颗"的候选（半径门 + 距离门）。→ cand 或 None

    三道门的分工：
      · **半径门**：车/臂都不动、只有舵机在转，靶盘半径在闭环尺度内几乎不变，
        尺寸突变说明这是另一个东西（同色干扰块）。这是**最有力**的一道门 ——
        无色模式下更是唯一的实质判据（那个模式下没有颜色可用）。
        ⚠️ 比的是 `_size_of()`（内孔优先），不是 minEnclosingCircle —— 见 _size_of。
      · **距离门**：窗本身就是位置门的粗版；这里再给一个上限（`4×r`），
        防的是**丢帧后窗扩到全幅**时，把画面另一头一个"碰巧同尺寸"的同色块捞上来。
        4 倍而不是更紧：舵机一拍最多把靶推 ~250px，远靶 r≈90 时 4×r≈360 才罩得住。
      · **内孔优先**：过了前两道的候选里，有白心孔的先挑（真靶盘的直接证据）。
        只有再没有带孔候选时才退回无孔候选（外缘被强光/遮挡切成弧时仍有救）。

    ⚠ `last["r"]` 与 `_size_of(c)` 都是"外圆等效"口径（内孔 ÷ 0.55），R_TOL 默认 0.35
    足够吸收两者在遮挡/透视下的十几个百分点差；**别把 R_TOL 收到 0.1 以下**。
    ⚠ 距离用**全帧坐标**比：cand 的 cx/cy 是窗内坐标（_color_candidates 已经加过
    它自己 bbox 的 ox/oy），这里要再加窗偏移 x0/y0 才和 last 同系。
    """
    if last is None:
        return None
    # ⚠️⚠️ 半径门在「已丢帧」时**小幅**放宽（2026-10-10 加、2026-10-30 收紧）：
    #    原意是锁定那帧若车在动/半遮挡，记下的 r 偏小会让真靶被连拒。但 0.15/帧、
    #    封顶 1.6 倍太松 —— 丢 4 帧后 r=84 的干扰块也能过门（|84−174|=90 < 174×0.56），
    #    这正是固件日志 `SHOT warn: color 1 r=82px rejected` 那个假测量的来源
    #    （固件按半径范围拒掉之后该色继续丢，越丢越松）。
    #    改用内孔口径后基准稳定，放宽只需要吸收**遮挡/透视**（≤±15%），
    #    故 0.10/帧、封顶 1.30 倍：真靶稳过，半尺寸干扰块（差 50%+）永远过不了。
    eff_tol = r_tol * (1.0 + 0.10 * min(int(last.get("lost", 0)), 3))
    r_ref = float(last.get("r", 0.0))
    passed = []
    for c in cands:
        if r_ref > 0.0 and abs(_size_of(c) - r_ref) > r_ref * eff_tol:
            continue
        ddx = (c["cx"] + x0) - last["cx"]
        ddy = (c["cy"] + y0) - last["cy"]
        d2 = ddx * ddx + ddy * ddy
        if r_ref > 0.0:
            # 4×r 是**安全阀**不是精定位：舵机一拍最多把靶推 ~250px，远靶 r≈90 时
            # 4×r=360 仍罩得住；真正排除干扰靠半径门 + 内孔优先。
            d_max = 4.0 * r_ref
            if d2 > d_max * d_max:
                continue
        passed.append((d2, c))
    if not passed:
        return None
    holed = [p for p in passed if p[1].get("hole_fit") is not None]
    return min(holed or passed, key=lambda p: p[0])[1]


def _track_frame(frame, trackers, cfg, r_tol, margin_px, use_color):
    """跟踪一帧 → (本帧回传的 targets 列表, 窗)。**就地更新** trackers。

    targets 里的坐标一律**全分辨率全帧**（与 0x04 帧同口径，固件的 tg_* 标定直接复用）。
    ⚠ 本帧没找到的跟踪器**不回传**（不是回传上一帧的陈旧值）—— 陈旧值会让单片机
    继续照着它推舵机，而那颗靶可能已经不在那了。位置冻结在 tracker 里只是为了
    "下一帧窗前瞻"，不是拿来当测量结果用的。

    ⚠️⚠️ 2026-10-24 加 `TARGET_STREAM_TRACK_SCALE`（缺省 1.0 = 不降采样）:
    细调闭环只要**靶心位置**（±2px 就够，死区 7px），不需要全分辨率。窗口 1588×518
    在 Pi 4 上"3 色 × (inRange+开+闭) + 连通域"要 300~660ms → 只有 1.5~3.6fps，
    而闭环在 1fps 下必然过冲（一拍要走 1 秒）。降到 0.5 后同一窗口的像素数只有 1/4，
    检测快约 4 倍（台架实测 37.5ms → 9ms），**才是真正能提帧率的那一刀**。
    坐标口径不变: 窗内检测在降采样空间做, 结果乘回 1/sc 再加窗偏移 → 回传仍是全分辨率。

    ⚠️ 跟踪器（上一帧位置/半径）也换算进降采样空间再参与半径门/距离门 —— 两道门
       必须和候选同一套尺度, 否则半径门会把真靶全判掉（r 差 2 倍）。

    ⚠️⚠️ 2026-10-30 加**速度 + 跨色运动补偿**（现场「带补光灯时红/蓝靶丢一串」的修法）:
    三种靶在画面里是**一起动**的（动的来源是舵机 6/3 / 车，不是靶自己跑）。于是：
      · 每个跟踪器记 (vx, vy) = 上一帧的位移，`_window_of` 用它前瞻窗心；
      · 某一色本帧丢了 → 用**本帧测到的那几色**的中位位移把它的冻结位置一起平移，
        下一帧的窗仍然罩着它（否则窗停在旧位置，靶已经走了几百 px，越丢越远）。
    这两条都只在**跟踪器内部**做，回传的仍是本帧实测值（丢的色照旧不回传）。
    """
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = _window_of(trackers, w, h, margin_px)
    sub = frame[y0:y1, x0:x1]
    if sub.size == 0:
        return [], (x0, y0, x1, y1)

    sc = float(getattr(config, "TARGET_STREAM_TRACK_SCALE", 1.0))
    if sc <= 0.0 or sc > 1.0:
        sc = 1.0
    if abs(sc - 1.0) > 1e-6:
        sub = cv2.resize(sub, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)

    hsv = cv2.cvtColor(sub, cv2.COLOR_BGR2HSV)      # 小窗内只转一次
    out = []
    debug = bool(_cfg_int("TARGET_STREAM_DEBUG", 0))
    prev = {t["color"]: (t["cx"], t["cy"]) for t in trackers}
    moved = []                    # 本帧**连续跟上**的颜色的位移（全分辨率 px）

    for t in trackers:
        # 带色 → 该色自己的色相族; 无色 → 满量程（只按 S/V 取掩膜）
        families = cfg["families"].get(t["color"], []) if use_color else _HUE_FULL
        cands = _color_candidates(hsv, t["color"], cfg["min_area"], cfg["sat_min"],
                                  cfg["val_min"], families,
                                  r_max_full=cfg.get("r_max"), scale=sc,
                                  rescue_hole=cfg.get("rescue_hole", False))
        if abs(sc - 1.0) > 1e-6:
            # 上一帧的参考量换进降采样空间（门在同一尺度上比才成立），窗偏移置 0
            t_ref = {"color": t["color"], "lost": t.get("lost", 0),
                     "cx": (t["cx"] - x0) * sc, "cy": (t["cy"] - y0) * sc,
                     "r": t["r"] * sc}
            pick = _pick_tracked(cands, t_ref, 0, 0, r_tol)
        else:
            pick = _pick_tracked(cands, t, x0, y0, r_tol)
        loc = _locate_one(pick) if pick is not None else None
        was_lost = int(t.get("lost", 0))
        if loc is None:
            t["lost"] = was_lost + 1                 # 位置冻结（下面按他色位移补偿）
            t["vx"] = float(t.get("vx", 0.0)) * 0.5  # 丢失期间速度衰减 → 窗刹住
            t["vy"] = float(t.get("vy", 0.0)) * 0.5
            if debug and t["lost"] in (1, 2, 5, 10):
                print("[靶标流/DBG] 丢[%s] lost=%d 窗=(%d,%d,%d,%d) sc=%.2f 候选=%d "
                      "末位=(%.0f,%.0f)r%.0f | %s"
                      % (_COLOR_EN.get(t["color"], "?"), t["lost"], x0, y0, x1, y1,
                         sc, len(cands), t["cx"], t["cy"], t["r"],
                         _dbg_cands(cands)))
            continue
        nx = loc[0] / sc + x0
        ny = loc[1] / sc + y0
        pcx, pcy = prev[t["color"]]
        mvx, mvy = nx - pcx, ny - pcy
        # 只有**连续跟上**的色才喂速度/位移：丢了几帧后重新捕获时，位移里混着
        # "靶不在时窗自己扩"的假运动，喂进去会把别的色的窗带偏。
        if was_lost == 0 and abs(mvx) <= _MAX_MOVE_PX and abs(mvy) <= _MAX_MOVE_PX:
            t["vx"] = 0.5 * float(t.get("vx", 0.0)) + 0.5 * mvx
            t["vy"] = 0.5 * float(t.get("vy", 0.0)) + 0.5 * mvy
            moved.append((mvx, mvy))
        elif was_lost:
            t["vx"] = t["vy"] = 0.0                  # 重新捕获: 速度从零重新估
            if debug and was_lost >= 2:
                print("[靶标流/DBG] 复得[%s] 丢了 %d 帧 → (%.0f,%.0f)r%.0f"
                      % (_COLOR_EN.get(t["color"], "?"), was_lost, nx, ny, loc[2] / sc))
        # 直接替换、**不平滑**：单片机端自己有死区与收敛判据，
        # 这里再滤一道就成了两级滤波串联，只会白加迟延（闭环最怕的就是迟延）。
        t["cx"] = nx
        t["cy"] = ny
        t["r"]  = loc[2] / sc
        t["lost"] = 0
        out.append({"color": t["color"], "cx": t["cx"], "cy": t["cy"], "r": t["r"]})

    # ---- 跨色运动补偿：本帧丢的色，按"跟上的色"的中位位移一起平移 ----
    if moved:
        dx = _med([m[0] for m in moved])
        dy = _med([m[1] for m in moved])
        dx = max(-_MAX_MOVE_PX, min(_MAX_MOVE_PX, dx))
        dy = max(-_MAX_MOVE_PX, min(_MAX_MOVE_PX, dy))
        for t in trackers:
            if t.get("lost", 0) > 0:
                t["cx"] += dx
                t["cy"] += dy

    out.sort(key=lambda d: d["cx"])                  # 按 cx 升序（与 0x04 帧同约定）
    return out, (x0, y0, x1, y1)


def _cfg_int(name, default):
    """读一个"有没有都行"的 config 整数（旧 config.py 没有这个名字时用默认）。

    ⚠️ 走 getattr 而不是 `from config import ...`: 进程里的 config 副本可能比本文件旧
    （改了 tasks/ 但没重启时），少一个名字会让整个模块 **ImportError** —— 靶标流直接崩。
    """
    try:
        return int(getattr(config, name, default) or 0)
    except (TypeError, ValueError):
        return int(default)


def _dbg_cands(cands, n=4):
    """把候选压成一行给丢帧诊断日志（窗内降采样坐标; 默认不开, 见 TARGET_STREAM_DEBUG）。"""
    parts = []
    for c in cands[:n]:
        parts.append("r%.0f/eq%.0f q%.2f hole%d d(%.0f,%.0f)" % (
            c["r"], _size_of(c), c.get("q", 0.0),
            1 if c.get("hole_fit") is not None else 0, c["cx"], c["cy"]))
    return " ; ".join(parts) or "-"


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


def _picam_up_wait():
    """CSI 起流最多等多久[s]（2026-10-23 加）。

    流**已经在跑**（预热过）→ 用 config 的 PICAM_STREAM_UP_S（3s）; 其实 ready() 会让
    `acquire` 走快路径瞬间返回, 这个 3s 只是兜底。
    **没预热过** → 只等 0.3s 就退回主摄: 冷启动要 2~3s, 而「平移射靶」这一步总共只有
    3s 预算 —— 白等 3s 再去开主摄（多 1s）就是 4s 打水漂，而主摄租约刚从 "6" 用过来、
    **就在手上**。真要能起来的冷流也会在 0.3s 内就绪。
    """
    try:
        if camera_picam.PICAM.ready():
            return float(config.PICAM_STREAM_UP_S)
    except Exception:                                 # noqa: BLE001
        pass
    return _PICAM_COLD_WAIT_S


def run_target_stream(timeout=None, abort_event=None, use_pi=False):
    """靶标流式跟踪主循环（命令 "7" 起流 / "8" 停流）。

    由 handler 的独立线程调用（`handler.py::_aim_worker`），**不同步阻塞主循环** ——
    这正是不用"6"那种阻塞式识别的理由：流期间单片机心跳照发，链路比阻塞式更健康。

    ⚠️ **优先 pi(树莓派 CSI)摄像头**（2026-10-16 加）: 理由同 _target.run_target_detect
    （主摄在机械臂上会漂, CSI 固定在底盘上稳）。`use_pi` 时先拿 CSI 起流;
    **起不来 / 中途掉线** → 整条流改用主摄重跑一遍（单片机那边只是帧计数卡一下,
    之后照常; 回帧的 cam 位跟着变, 它自己会换瞄准列）。
    ⚠️ 退回是**单向**的: 一旦回到主摄就不再试 CSI（否则一根接触不良的排线会让 cam 位
    在 20Hz 上反复跳, 单片机每拍重置计数器 —— 永远收不敛）。

    退出条件：abort_event 置位（"8"）/ 串口断开 / 超过 TARGET_STREAM_MAX_S。
    返回 dict（与其它任务函数的返回约定一致）。
    """
    abort_event = abort_event or threading.Event()
    if use_pi:
        res = _run_stream_once(timeout, abort_event, pi=True, up_wait=_picam_up_wait())
        if res.get("reason") != "cam_fail":
            return res
        print("[WARN] pi(CSI) 靶标流起不来/中断 → 退回主摄重跑一遍")
        _send_count0("(pi摄像头退回, 正在换成主摄)", False)
    return _run_stream_once(timeout, abort_event, pi=False)


def _run_stream_once(timeout, abort_event, pi, up_wait=None):
    """流的一次完整尝试（锁定 + 跟踪）。返回 dict; reason=="cam_fail" = 相机这一路不行。

    ⚠️ 调用方（run_target_stream）据 reason 决定要不要换另一台相机重跑 —— 所以
       "相机打不开"与"跑到超时/被停"必须用**不同的 reason** 区分开。
    up_wait = CSI 起流最多等多久[s]（None → config 的 PICAM_STREAM_UP_S）;
              没预热过时由 _picam_up_wait() 压到 0.3s, 见那里的说明。
    """
    cfg = _detect_cfg(pi)
    src = None
    cap = None
    cam_is_pi = bool(pi)
    if pi:
        if up_wait is None:
            up_wait = float(config.PICAM_STREAM_UP_S)
        src = camera_picam.acquire_picam(float(up_wait))
        if src is None:
            print("[WARN] 靶标流：pi(CSI) 起流失败")
            return {"success": False, "reason": "cam_fail", "frames": 0}
    else:
        cap = _open_with_retry(abort_event)
        if cap is None:
            print("[WARN] 靶标流：主摄像头打开失败，立刻回一帧 count=0 让单片机知情")
            _send_count0("(相机打不开)", cam_is_pi)
            return {"success": False, "reason": "cam_fail", "frames": 0}

    timeout = config.TARGET_STREAM_MAX_S if timeout is None else timeout
    deadline = time.time() + timeout
    t_start = time.time()
    sent = 0
    blanks = 0        # 空窗/异常计数（只用于收尾日志）
    last_log = 0.0
    sent_at_log = 0   # 上次状态日志时的 sent（用来算**区间**帧率, 见下）
    trackers = []

    print("[INFO] 靶标流启动（小窗跟踪 + 门控）")
    try:
        # ---- 阶段1: 锁定（全画面）。锁不到不退出: 每 _LOCK_RETRY_EVERY 帧重试一次,
        #      期间照发 count=0 帧（让单片机知道"流活着、只是还没锁上"）。----
        lock_try = 0
        while not abort_event.is_set() and not trackers:
            t_iter = time.time()
            if t_iter > deadline:
                print(f"[WARN] 靶标流超过安全时长 {timeout}s（一直没锁定），自动停止")
                return {"success": False, "reason": "max_duration", "frames": sent}

            if pi and not src.alive():
                # 掉了 → 交回主摄重跑（调一次锁定的钱, 比"帧计数停涨让单片机冻结"划算）
                print("[WARN] 靶标流：pi(CSI) 中断（不再出帧）")
                return {"success": False, "reason": "cam_fail", "frames": sent}

            if lock_try % _LOCK_RETRY_EVERY == 0:
                # 锁定尺度: 先试降采样（快 4 倍）, 连续失败后再退回全分辨率（防漏小靶）
                lock_scale = (float(getattr(config, "TARGET_STREAM_LOCK_SCALE", 0.5))
                              if lock_try < _LOCK_FULL_SCALE_AFTER
                              else float(config.TARGET_PROC_SCALE))
                try:
                    trackers = _lock(lambda: _read_frame(src, cap, pi), cfg,
                                     bool(config.TARGET_STREAM_USE_COLOR), lock_scale)
                except Exception as e:                # noqa: BLE001
                    print(f"[WARN] 靶标流锁定异常: {type(e).__name__}: {e}")
                    trackers = []
                if trackers:
                    print("[INFO] 靶标流已锁定: " + " ".join(
                        f"{_COLOR_EN.get(t['color'], '?')}({int(t['cx'])},{int(t['cy'])})"
                        f"r{int(t['r'])}" for t in trackers))
                    break
            lock_try += 1
            _send_count0(cam_is_pi=cam_is_pi)
            sent += 1
            blanks += 1
            _sleep_rest(t_iter)

        # ---- 阶段2: 小窗跟踪 ----
        # ⚠ 计时的两个基准必须在**进入跟踪循环时**重新锚定：锁定阶段耗时 ~1s 且期间发的
        #   是空窗帧，把它们算进来会让第一行状态日志的"区间帧率"算成一个荒唐的数
        #   （last_log=0.0 时 span 更是直接等于 Unix 纪元到现在的秒数）。
        last_log = time.time()
        sent_at_log = sent

        while not abort_event.is_set():
            t_iter = time.time()
            t_now = t_iter
            if t_now > deadline:
                print(f"[WARN] 靶标流超过安全时长 {timeout}s，自动停止")
                return {"success": False, "reason": "max_duration", "frames": sent}

            if pi and not src.alive():
                print("[WARN] 靶标流：pi(CSI) 中断（不再出帧）→ 交回主摄重跑")
                return {"success": False, "reason": "cam_fail", "frames": sent}

            ok, frame = _read_frame(src, cap, pi)
            if not ok or frame is None:
                time.sleep(0.05)
                continue

            try:
                targets, win = _track_frame(frame, trackers, cfg,
                                            config.TARGET_STREAM_R_TOL,
                                            config.TARGET_STREAM_MARGIN_PX,
                                            bool(config.TARGET_STREAM_USE_COLOR))
            except Exception as e:                    # noqa: BLE001
                # ⚠ 单帧异常必须吞掉：一帧坏图不该杀掉整个流线程（那会让单片机看到
                #   帧计数停涨、冻结不动 —— 查起来像链路故障，其实是这里崩了）。
                print(f"[WARN] 靶标流单帧检测异常: {type(e).__name__}: {e}")
                targets, win = [], (0, 0, 0, 0)

            if not targets:
                blanks += 1

            if not _send(build_stream_frame(targets[:config.TARGET_MAX_TARGETS], cam_is_pi)):
                break                                 # 串口断了: 直接收摊
            sent += 1

            # 1Hz 状态日志 + 落帧（Web 预览）。限流是为了不刷爆日志文件。
            if config.TARGET_STREAM_LOG_HZ > 0 and \
                    t_now - last_log >= 1.0 / config.TARGET_STREAM_LOG_HZ:
                # ⚠ 这里打的是**区间帧率**（自上次日志以来），不是"累计帧数 ÷ 总耗时"。
                #   后者在起流阶段被锁定期的空窗帧和 ~1s 的锁定耗时拖着，头几行会显示成
                #   个位数 fps（实测 1.7），看着像"帧率根本达不到" —— 而真正要判断的是
                #   "稳态下跑多少"。调 TARGET_STREAM_HZ 时看的就是这个数。
                span = t_now - last_log
                fps = (sent - sent_at_log) / span if span > 0 else 0.0
                last_log, sent_at_log = t_now, sent
                print(f"[靶标流] fps={fps:.1f} n={len(targets)} "
                      f"窗={win[2] - win[0]}x{win[3] - win[1]} "
                      f"丢={[_COLOR_EN.get(t['color'], '?') for t in trackers if t['lost']]}")
                _write_preview(frame, targets)

            _sleep_rest(t_iter)

    except SerialDisconnected as e:
        print(f"[WARN] 靶标流中止: 单片机串口断开（{e}）")
        return {"success": False, "reason": f"serial_disconnected: {e}", "frames": sent}
    finally:
        if pi:
            src.release()       # 只续期、**不关设备**（CSI 由 picam_watchdog 按空闲超时关）
        else:
            release_camera(cap)     # 租约句柄 → 归还（保持打开）; 普通 cap → 真释放
        el = max(time.time() - t_start, 1e-6)
        print(f"[INFO] 靶标流结束[{('pi/CSI' if pi else '主摄')}]: 共发 {sent} 帧"
              f"（{sent / el:.1f}fps，空窗 {blanks} 次）")

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
