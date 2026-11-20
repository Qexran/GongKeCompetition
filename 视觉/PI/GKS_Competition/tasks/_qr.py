# tasks/_qr.py — 二维码检测（两种方法：OpenCV / pyzbar）
#
# 与 GCS(cv_python/tasks/_qr.py) 的识别方式一致（pyzbar 灰度化后 decode），
# 额外提供 OpenCV QRCodeDetector 作为第二种方法。两种方法各自独立成一个任务
# 函数，将来由单片机串口指令决定调用哪个（对应 GCS 的 run_erweima 同构）。
#
# 二维码内容应为三位数字（如 "211"）：解码结果不是三位数则排除、继续寻找
# 其他二维码；若整个区域只有一个非三位数二维码，超时后返回失败。
#
# ---- 两台相机（2026-09 加）----
# 解码链（_decode_*/_scan_qr/_annotate...）只有一条，两条入口的区别**只在开哪台相机**：
#   命令 "1" → run_qr_wechat()      扫码副摄 /dev/camera_Qr   （is_qr=True，  QR_SCAN_*     档）
#   命令 "D" → run_qr_wechat_main() 主摄     /dev/camera_main（is_qr=False， QR_MAIN_SCAN_* 档）
# ⚠ 预处理参数**两套独立**：QR_SCAN_CENTER_CROP=0.3 是为副摄的近距离取景调的，
#   主摄对着场地时二维码多半不在画面中心 30% 内，所以 QR_MAIN_CENTER_CROP 默认 0.0。
# ⚠ "D" 与 "1" 回传的是**同一种帧**（9B ASCII "QR<ddd>"/"QRF"），单片机侧
#   ucRpiWaitQrResult 与下游（IF 条件/排爆/反恐）一个字都不用改 —— 所以这里
#   **不要在任务层加任何"按相机区分"的返回值**（保持两条路对外完全同构）。
import os
import time
import re
import threading

import cv2
import numpy as np
from pyzbar.pyzbar import decode

from camera import CameraManager
# 相机租约（预热复用，见 camera_lease.py）：命中租约时 acquire_camera 返回一个
# LeaseHandle（read() 与 cv2.VideoCapture 同形，_scan_qr 一行都不用改），
# 没命中就退回 open_camera 的旧行为。归还**必须**走 release_camera ——
# 租约命中的句柄要「还」不要「关」，否则下次任务还得重新预热。
from camera_lease import acquire_camera, release_camera

# 三位数字校验（整串必须是 3 个数字字符，如 "211"）
_VALID_RE = re.compile(r"^\d{3}$")

# 赛题二维码的取值域: **每一位都是 1~3**（即 111~333, 共 27 种）。
# 与固件侧 STEP_QR_SET_TEST 的 c_iQrTestSanitize 同一口径, 也与 QR_DIFF_KNOWN="123"
# （那三个字符是**每一位的取值范围**）一致。
_QR_DIGITS = "123"

# 方法显示名（日志用）
_METHOD_DISPLAY = {"opencv": "OpenCV", "pyzbar": "pyzbar", "wechat": "WeChat",
                   "diff": "自研", "auto": "自动"}

# 需要**前置粗矫正**的方法。wechat 自带超分/几何矫正、自研自带一整套几何重建，
# 给它们先过一遍 _perspective_correct 只会在杂乱背景上误检定位角、把整帧 warp 坏。
# 所以这里用**白名单**而不是 `method != "wechat"` 那种黑名单：新加算法默认不吃粗矫正。
_NEEDS_DESKEW = ("opencv", "pyzbar")


def _is_valid_code(data):
    """三位数**且每一位都在 1~3**（= 111~333）。

    ⚠️ 原来只判「三个数字字符」（`^\\d{3}$`）—— 于是场地里任何内容是三位数字的码
    （`000` / `999` / 别的贴纸）都会被判**成功**并回传 `QR999`：
    单片机 `ucRpiWaitQrResult` 认的就是「QR + 3 位数字」，照单全收并置位 `iGetQrCode()`，
    而**下游一律不合法**（IF 条件 100~399、排爆抓取首位 1~3、反恐任务第三位 1~3）
    → 现场表现正是「Pi 报二维码识别成功、后面整链按旧值/不动」。
    收紧之后越界的码当成"不是我们的码"继续找，并在失败原因里报出来（见 _scan_qr）。
    """
    return bool(_VALID_RE.match(data)) and all(ch in _QR_DIGITS for ch in data)


def _decode_pyzbar(frame):
    """pyzbar 解码一帧，返回 [{"data": str, "poly": [(x,y),...]|None}]。

    与 GCS 一致：灰度化后 pyzbar.decode(gray)，取 obj.data 与 obj.polygon。
    """
    results = []
    try:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    except Exception:
        return results
    for obj in decode(gray):
        try:
            data = obj.data.decode("utf-8").strip()
        except Exception:
            continue
        poly = None
        if obj.polygon:
            poly = [(p.x, p.y) for p in obj.polygon]
        results.append({"data": data, "poly": poly})
    return results


def _decode_opencv(frame):
    """OpenCV QRCodeDetector 解码一帧，返回与 _decode_pyzbar 同构的列表。

    优先 detectAndDecodeMulti（支持一帧多个二维码），为空时回退
    detectAndDecode（单码），以满足"排除非三位数、寻找其他二维码"的需求。
    """
    results = []
    det = cv2.QRCodeDetector()
    try:
        ok, infos, points, _ = det.detectAndDecodeMulti(frame)
        if ok and infos:
            for info, pts in zip(infos, points):
                if info:
                    info = info.strip()
                if not info:
                    continue
                poly = None
                if pts is not None and len(pts):
                    poly = [(int(float(p[0])), int(float(p[1])))
                            for p in pts.reshape(-1, 2)]
                results.append({"data": info, "poly": poly})
    except Exception:
        results = []

    # 多码解码为空时回退单码解码
    if not results:
        try:
            data, pts, _ = det.detectAndDecode(frame)
            if data:
                data = data.strip()
                if not data:
                    return results
                poly = None
                if pts is not None and len(pts):
                    poly = [(int(float(p[0])), int(float(p[1])))
                            for p in pts.reshape(-1, 2)]
                results.append({"data": data, "poly": poly})
        except Exception:
            pass
    return results


def _annotate(frame, r, code, method):
    """在识别帧上画出二维码边框 + 结果文字。"""
    out = frame.copy()
    poly = r.get("poly")
    if poly:
        pts = np.array(poly, dtype=np.int32)
        cv2.polylines(out, [pts], True, (0, 255, 0), 2)
        x = int(min(p[0] for p in poly))
        y = int(min(p[1] for p in poly))
    else:
        x, y = 10, 30
    cv2.putText(out, f"{code} ({method})", (max(0, x), max(20, y - 10)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    return out


def _center_crop(frame, ratio):
    """裁取画面中心的正方形区域，去掉四周杂乱背景。

    微信引擎检测网络输入固定 384x384，整帧会被等比缩到 384 再检测；二维码在
    整图中的占比越大越易检出。放大无效（整帧仍会被缩回 384），只有「裁」能让
    二维码占比变大。ratio<=0 时返回原图。
    """
    if ratio is None or ratio <= 0:
        return frame
    h, w = frame.shape[:2]
    side = max(1, int(min(h, w) * ratio))
    x0 = (w - side) // 2
    y0 = (h - side) // 2
    return frame[y0:y0 + side, x0:x0 + side]


def _preprocess(frame, scale, max_size=960):
    """预处理：先降采样到 max_size 内（保证动态识别速度），再对「小图」按 scale 放大。"""
    h, w = frame.shape[:2]
    m = max(h, w)
    # 先降采样：大图（如 1920x1080）降到合理尺寸，避免逐帧处理过慢
    if m > max_size:
        s = max_size / float(m)
        frame = cv2.resize(frame, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
        m = max_size
    # 再放大：只对「放大后仍不超过 max_size」的小图放大，提升小码识别率
    if scale and scale > 1.0 and m * scale <= max_size:
        frame = cv2.resize(frame, (int(frame.shape[1] * scale), int(frame.shape[0] * scale)),
                           interpolation=cv2.INTER_CUBIC)
    return frame


# ---- 透视校正（找 3 个定位角 → 推出第 4 角 → 矫正畸变）----
def _check_finder_ratio(bw, cx, cy, half=45):
    """检查过 (cx,cy) 的水平/垂直/对角扫描线是否符合 1:1:3:1:1（定位角特征）。"""
    H, W = bw.shape

    def runs(dx, dy):
        prev = None
        ln = 0
        rs = []
        for t in range(-half, half + 1):
            x = int(cx + dx * t)
            y = int(cy + dy * t)
            if x < 0 or y < 0 or x >= W or y >= H:
                continue
            v = int(bw[y, x])
            if prev is None:
                prev, ln = v, 1
            elif v == prev:
                ln += 1
            else:
                rs.append(ln)
                prev, ln = v, 1
        if ln:
            rs.append(ln)
        return rs

    def ok(rs):
        for i in range(len(rs) - 4):
            seg = rs[i:i + 5]
            if any(s == 0 for s in seg):
                continue
            total = sum(seg)
            ratio = [s / (total / 7.0) for s in seg]
            if sum(abs(ratio[j] - [1, 1, 3, 1, 1][j]) for j in range(5)) < 2.5:
                return True
        return False

    return any(ok(runs(dx, dy)) for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)))


def _find_finder_centers(gray):
    """找二维码定位角中心：自适应阈值 + 轮廓 + 1:1:3:1:1 比例校验。

    返回 [(cx, cy, area), ...]。1:1:3:1:1 是定位角（黑-白-黑-白-黑）的专属特征，
    比「有孔方块」更能过滤掉屏幕上的 UI 元素等误检。
    """
    th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY, 51, 10)
    bw = (th > 0).astype("uint8")
    contours, hierarchy = cv2.findContours(cv2.bitwise_not(th), cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return []
    hierarchy = hierarchy[0]
    candidates = []
    for i in range(len(contours)):
        if hierarchy[i][2] == -1:
            continue  # 无孔洞 → 不是定位角
        peri = cv2.arcLength(contours[i], True)
        approx = cv2.approxPolyDP(contours[i], 0.05 * peri, True)
        if len(approx) != 4:
            continue
        M = cv2.moments(contours[i])
        if M["m00"] < 1e-6:
            continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        if not _check_finder_ratio(bw, cx, cy):
            continue
        candidates.append((cx, cy, cv2.contourArea(contours[i])))
    # 去重（同一位置多次检出），按面积从大到小
    dedup = []
    for c in sorted(candidates, key=lambda c: -c[2]):
        if all((c[0] - d[0]) ** 2 + (c[1] - d[1]) ** 2 > 225 for d in dedup):
            dedup.append(c)
    return dedup


def _finder_centers_to_quad(centers):
    """从定位角里找最佳「等腰直角三角形」三元组 → 二维码 4 个角点。

    二维码 3 个定位角构成等腰直角三角形（两直角边等长）。不足 3 个或误差太大返回 None。
    """
    import itertools
    if len(centers) < 3:
        return None
    pts = [np.array([c[0], c[1]], dtype=np.float32) for c in centers]
    best = None
    best_err = float("inf")
    for i, j, k in itertools.combinations(range(len(pts)), 3):
        a, b, c = sorted([np.linalg.norm(pts[i] - pts[j]),
                          np.linalg.norm(pts[j] - pts[k]),
                          np.linalg.norm(pts[k] - pts[i])])
        if a < 20:
            continue
        err = abs(b - a) / a + abs(c - a * 1.414) / a
        if err < best_err:
            best_err = err
            best = (i, j, k)
    if best is None or best_err > 1.5:
        return None
    i, j, k = best
    tri = [pts[i], pts[j], pts[k]]
    d = [np.linalg.norm(tri[0] - tri[1]), np.linalg.norm(tri[1] - tri[2]),
         np.linalg.norm(tri[2] - tri[0])]
    hyp = max(range(3), key=lambda m: d[m])  # 最长边 = 斜边
    tl = tri[(hyp + 2) % 3]  # 斜边对面的顶点 = 直角顶点
    other = [tri[m] for m in range(3) if m != (hyp + 2) % 3]
    tr, bl = other[0], other[1]
    br = tr + bl - tl  # 平行四边形补全第 4 角
    return np.array([tl, tr, br, bl], dtype=np.float32)


def _expand_quad(quad, factor=1.5):
    """把四边形绕中心外扩 factor 倍（定位角中心在二维码内侧，需外扩到外缘）。"""
    c = quad.mean(axis=0)
    return (quad - c) * factor + c


def _order_points(pts):
    """把 4 个角点排成 左上/右上/右下/左下 顺序。"""
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]   # 左上（x+y 最小）
    rect[2] = pts[np.argmax(s)]   # 右下（x+y 最大）
    diff = np.diff(pts, axis=1).ravel()
    rect[1] = pts[np.argmin(diff)]  # 右上（y-x 最小）
    rect[3] = pts[np.argmax(diff)]  # 左下（y-x 最大）
    return rect


def _perspective_correct(frame):
    """找二维码定位角 → 推出第 4 角 → 透视校正成正视视角。找不到则返回原图。"""
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    quad = _finder_centers_to_quad(_find_finder_centers(gray))
    if quad is None:
        return frame
    quad = _expand_quad(quad)   # 定位角中心 → 外扩到二维码外缘
    pts = _order_points(quad)
    tl, tr, br, bl = pts
    w = int(max(np.linalg.norm(tr - tl), np.linalg.norm(br - bl)))
    h = int(max(np.linalg.norm(bl - tl), np.linalg.norm(br - tr)))
    if w < 20 or h < 20:
        return frame
    dst = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(pts, dst)
    return cv2.warpPerspective(frame, M, (w, h))


def _qr_params(use_main, method=None):
    """取这一路要用的预处理参数 —— (scale, max_size, center_crop, perspective)。

    **每次调用现取 config**（不在 import 时固化）：这是本模块既有的约定，
    为的是 Pi Web「设置参数」改完下一次识别就生效，不用重启进程。
    ⚠ 两套参数**刻意分开**（见文件头）：主摄与扫码副摄的取景差得很远，
      共用一套会让"为副摄调好的中心裁剪"把主摄那条路直接裁没。

    ⚠ 自研那条（method="diff"）**再独立一套**，而且 perspective 恒为 False：
      它的几何重建比 _perspective_correct 强得多，前置粗矫正只会帮倒忙；另外它的
      定位角检测是**按分辨率调过的**（块大小 51/75/101、轮廓面积门限 400），
      960 降采样会把小码的定位角压到门限以下 —— 所以这条路的 max_size 要放宽。
    """
    import config
    if method == "diff":
        return (config.QR_DIFF_SCALE, config.QR_DIFF_MAX_SIZE,
                config.QR_DIFF_CENTER_CROP, False)
    if use_main:
        return (config.QR_MAIN_SCAN_SCALE, config.QR_MAIN_SCAN_MAX_SIZE,
                config.QR_MAIN_CENTER_CROP, config.QR_MAIN_SCAN_PERSPECTIVE)
    return (config.QR_SCAN_SCALE, config.QR_SCAN_MAX_SIZE,
            config.QR_SCAN_CENTER_CROP, config.QR_SCAN_PERSPECTIVE)


def _cam_label(use_main):
    """日志用的相机名（只影响 CameraManager.read 打的日志）。"""
    return "Main Camera" if use_main else "QR Camera"


# 本轮扫描的总 deadline（绝对时间戳），由 _scan_qr 在进入循环前设、_run_task 收尾时清。
# 慢解码器（自研一次尝试实测 4.5s）必须拿到它，否则一轮就能把总时长顶穿预算 ——
# 详见 _scan_qr 里的注释。
_SCAN_DEADLINE = None

# 一轮尝试最少值这么多秒才起跑（见 _scan_qr 里的 min_slice）。
# 自研那个值在下面（_DIFF_MIN_SLICE_S）；这里是 wechat/opencv/pyzbar 的档：
# 一次尝试 = 读帧(~33ms) + 预处理 + 推理，实测约 1s，取 1.0 与它对齐。
_MIN_ATTEMPT_S = 1.0


def _scan_qr(cap, decode_fn, timeout, method, abort_event, use_main=False):
    """扫描循环：读帧 → 放大预处理 → 解码 → 三位数校验 → 命中即返回，超时/打断返回失败。

    返回 dict: {"success", "code", "method", "reason", "frame",
                "frame_raw"(仅成功时: 裁剪/缩放前的相机原图)}。
    非三位数的二维码被排除，继续读下一帧寻找其他二维码。

    use_main=False（默认）= 扫码副摄；True = 主摄。**只影响预处理参数与日志标签** ——
    解码、校验、返回结构两条路完全一致（单片机侧因此能用同一套解析）。
    """
    (scale, max_size, center_crop, perspective) = _qr_params(use_main, method)
    cam = _cam_label(use_main)

    deadline = time.time() + timeout
    last_frame = None
    seen_invalid = False  # 是否见过"有二维码但不是三位数"的情况（用于失败原因）
    seen_out_of_range = None  # 记下第一个"三位数但每位不都在 1~3"的读数（如 "999"）

    # 把**本轮的总 deadline** 交给解码函数（`_decode_diff` 会读它，当作自己内部
    # 那次几何重建的止损点）。
    # ⚠️ 不能只靠下面 while 的开头那一查：慢解码器（自研一次尝试实测 4.5s）只要
    # 一轮就吃掉大部分预算，而 `while` 只在**下一轮开头**才发现超时 —— 预算 6s 时
    # 会跑成 ~9s，顶穿「Pi 必须比固件先放弃」那条契约（见 `_get_diff_timeout` 注释：
    # 固件先放弃的话，这次的结果帧会落进下一次重扫的等待窗口里）。
    global _SCAN_DEADLINE
    _SCAN_DEADLINE = deadline

    # 一轮尝试至少值这么多秒才起跑（按算法分档）。
    # ⚠️ 为什么不靠下面那句 `while time.time() < deadline`：解码器/解码函数只在**半路**
    #    才查 deadline，已经跑起来的那一轮一定会跑完 —— 在 deadline 前 0.1s 起跑，
    #    照样把这整轮的时间顶到预算外面。实测 wechat 一轮 ~1s、自研一轮 ~1.35s 起，
    #    于是「预算 6s 实际跑了 6.56s」，再叠上开相机 1.3s 就顶穿了固件愿意等的那个数
    #    （而迟到的结果帧会落进下一次重扫的等待窗口里被当成那一次的结果）。
    min_slice = _DIFF_MIN_SLICE_S if method == "diff" else _MIN_ATTEMPT_S

    while time.time() < deadline:
        # ⚠️⚠️ **中止判定必须排在 min_slice 提前退出之前**（2026-10 修）。
        #    反过来的话: 预算只剩不到 1.0s（wechat）/1.6s（自研）时被打断 → `break`
        #    落到函数末尾的**普通失败**出口 → 回一条 `QRF` —— 而那正是 `"x"` 机制要根治的
        #    「被中止的旧任务回帧、落进下一轮等待窗被当成新结果」。
        #    按 `_MIN_ATTEMPT_S`/`_DIFF_MIN_SLICE_S` 相对 5~6s 预算算, 这个窗口不小。
        if abort_event is not None and abort_event.is_set():
            # 主动放弃（单片机/web 打断），不算失败：仅标记 aborted，不填 reason
            return {"success": False, "aborted": True, "code": None,
                    "method": method, "reason": "", "frame": last_frame}
        # 剩余时间不够跑完一轮就别起跑 —— 这条把「超出预算」关在一个可预期的范围内
        if deadline - time.time() < min_slice:
            break

        frame = CameraManager.read(cap, cam, max_retries=3)
        if frame is None:
            time.sleep(0.05)
            continue
        last_frame = frame

        # 预处理：中心裁剪（裁掉四周杂乱背景，提升二维码占比）→ 降采样 →
        # 透视校正（仅几何方法；微信引擎自带超分/几何矫正，且杂乱背景下易误检
        # 定位角导致整帧被错误 warp，故对 wechat 跳过）
        proc = _center_crop(frame, center_crop)
        proc = _preprocess(proc, scale, max_size)
        if perspective and method in _NEEDS_DESKEW:
            proc = _perspective_correct(proc)
        results = decode_fn(proc)
        if results:
            # 依次校验本帧所有二维码，命中三位数立即返回
            for r in results:
                data = r["data"]
                if _is_valid_code(data):
                    annotated = _annotate(proc, r, data, method)
                    # frame_raw = **裁剪/缩放之前的相机原图**: 成功帧画在 proc（裁剪+
                    # 降采样后）上, 坐标参数调坏时它才是"当时现场"的唯一证据
                    # （训练模式开 = logstore 里与结果帧同序号配对归档, 见 archive_frame）。
                    return {"success": True, "code": data, "method": method,
                            "reason": "", "frame": annotated, "frame_raw": frame}
            seen_invalid = True  # 有二维码但都不是三位数 → 排除，继续找
            if seen_out_of_range is None:
                # 记下**第一个越界的读数**（如 "000"/"999"）: 失败时它是唯一能把
                # 「码读得出来但不是我们的码」与「压根没读到码」分开的证据。
                seen_out_of_range = next((r["data"] for r in results
                                          if _VALID_RE.match(r["data"])), None)

        time.sleep(0.01)  # 避免空转刷满 CPU

    if seen_out_of_range is not None:
        reason = (f"{method} 超时 ({timeout:.0f}s)：读到二维码 {seen_out_of_range}，"
                  f"但每位必须是 1~3（赛题取值域 111~333）")
    elif seen_invalid:
        reason = f"{method} 超时 ({timeout:.0f}s)：区域中二维码均非三位数"
    else:
        reason = f"{method} 超时 ({timeout:.0f}s)：未检测到二维码"
    return {"success": False, "code": None, "method": method,
            "reason": reason, "frame": last_frame}


def _open_with_retry(abort_event, attempts=5, delay=0.3, use_main=False):
    """打开扫码用的摄像头（副摄或主摄），失败时少量重试。

    Web 触发扫码前会先断开推流，但推流线程释放设备需要一小段时间，
    这里重试等待设备空闲（EBUSY）。串口直接调用时同样受益。

    ⚠️ 走 acquire_camera 而不是 open_camera：单片机发过 "B"（预热扫码相机）/
    "C"（预热主摄）时这里直接拿到租约句柄 —— 相机早已开好、读线程一直在更新
    最新帧，省掉实测 ~1.3s 的开相机时间。没预热过时 acquire 返回 None →
    退回原位 open_camera，行为与加租约之前**完全一致**（所以路线里删掉预热步骤
    不会坏，只是慢）。

    ⚠️ 主摄那条（use_main=True）传 profile="qr"：**曝光档是关于目标的**
    （二维码的对比度/光照），不是关于相机的，所以主摄扫二维码照样该吃 QR 档。
    租约命中时 acquire 会**就地重设曝光**（不是重开设备）—— 所以「先按 main 档
    预热，再进来扫码」是纯赚的：只损失曝光收敛期那几帧，不重付开相机的时间。
    """
    cam = "主摄" if use_main else "二维码摄像头"
    for i in range(attempts):
        if abort_event is not None and abort_event.is_set():
            return None
        cap = acquire_camera(is_qr=not use_main,
                             profile="qr" if use_main else None)
        if cap is not None:
            return cap
        print(f"[WARN] {cam}打开失败 (第{i + 1}/{attempts}次)，稍后重试...")
        time.sleep(delay)
    return None


def _run_task(method, decode_fn, timeout, abort_event, use_main=False):
    """打开摄像头（副摄/主摄）→ 扫描 → 释放，返回结果 dict。"""
    cap = _open_with_retry(abort_event, use_main=use_main)
    if cap is None:
        # 打开阶段被打断同样视为「主动放弃」，而非失败
        if abort_event is not None and abort_event.is_set():
            print(f"[INFO] 二维码（{_METHOD_DISPLAY.get(method, method)}）任务被打断")
            return {"success": False, "aborted": True, "code": None,
                    "method": method, "reason": "", "frame": None}
        return {"success": False, "code": None, "method": method,
                "reason": "二维码摄像头打开失败", "frame": None}
    try:
        print(f"[INFO] 开始二维码检测 ({method})，限时 {timeout:.0f}s"
              f"{'，主摄' if use_main else ''}...")
        t0 = time.time()
        result = _scan_qr(cap, decode_fn, timeout, method, abort_event, use_main)
        # 实际耗时——「自研到底几秒」只能从这里读（⚠️ 路线里那块 STEP_RPI_SEND_TASK
        # 是非阻塞的，它拿不到结果帧，所以没法从固件侧计时）
        result["elapsed"] = time.time() - t0
        if result["success"]:
            print(f"[INFO] 检测到二维码 [{method}]: {result['code']}"
                  f"（用时 {result['elapsed']:.2f}s）")
            # 结果回传由 handler.py 统一发送（_cmd_qr = 命令 "1" / _cmd_qr_main = 命令 "D"，
            # 两条都走同一个 _cmd_qr_common）。此处仅返回结果 dict，
            # 不在任务层直接碰串口（保持任务函数纯净可测）。
        elif result.get("aborted"):
            # 单片机/web 主动放弃，不算失败，仅日志记录
            print(f"[INFO] 二维码（{_METHOD_DISPLAY.get(method, method)}）任务被打断")
        else:
            print(f"[WARN] {result['reason']}（用时 {result['elapsed']:.2f}s）")
        return result
    finally:
        global _SCAN_DEADLINE, _DIFF_SKIP_LOGGED
        _SCAN_DEADLINE = None   # 收尾必清：别让上一次的预算泄漏给下一次扫描
        _DIFF_SKIP_LOGGED = False
        release_camera(cap)     # 租约句柄 → 归还（保持打开）; 普通 cap → 真释放


def _get_timeout(timeout):
    """timeout 为 None 时从 config 取当前值（支持 Web「设置参数」即时生效）。"""
    if timeout is None:
        from config import QR_SCAN_TIMEOUT
        return QR_SCAN_TIMEOUT
    return timeout


def run_qr_opencv(timeout=None, abort_event=None):
    """OpenCV 识别二维码（限时 timeout 秒，失败/被打断返回失败）。

    Returns dict: {"success", "code", "method", "reason", "frame"}。
    """
    return _run_task("opencv", _decode_opencv, _get_timeout(timeout), abort_event)


def run_qr_pyzbar(timeout=None, abort_event=None):
    """pyzbar 识别二维码（与 GCS 一致，限时 timeout 秒，失败/被打断返回失败）。

    Returns dict: {"success", "code", "method", "reason", "frame"}。
    """
    return _run_task("pyzbar", _decode_pyzbar, _get_timeout(timeout), abort_event)


# ---- 微信二维码引擎（OpenCV wechat_qrcode，CNN 检测，比 QRCodeDetector 更鲁棒）----
_WECHAT_DET = None
_WECHAT_DET_ERR = None          # 加载失败的原因（只打印一次，见下）


def _get_wechat_detector():
    """懒加载微信二维码检测器（CNN 模型加载较慢，只建一次并缓存）。

    ⚠️⚠️ **失败必须吼出来，不能静默返回 None/空**（2026-09 实测踩过）：
    WeChat 引擎在 `opencv-contrib-python` 里，而普通 `opencv-python` **没有**
    `cv2.wechat_qrcode` —— 缺了的话 `_decode_wechat` 每次返回空列表，
    表现是**扫码永远超时**、日志里一条线索都没有（"解不出来"和"根本没这个解码器"
    在现象上完全一样）。而斜视扫码这条路（主摄/"D"）**主力就是它**，
    所以这里必须把原因打成一行日志。
    """
    global _WECHAT_DET, _WECHAT_DET_ERR
    if _WECHAT_DET is None:
        model_dir = "?"
        try:
            from config import WECHAT_QR_MODEL_DIR
            model_dir = WECHAT_QR_MODEL_DIR
            _WECHAT_DET = cv2.wechat_qrcode_WeChatQRCode(
                os.path.join(model_dir, "detect.prototxt"),
                os.path.join(model_dir, "detect.caffemodel"),
                os.path.join(model_dir, "sr.prototxt"),
                os.path.join(model_dir, "sr.caffemodel"),
            )
        except Exception as e:
            _WECHAT_DET_ERR = f"{type(e).__name__}: {e}"
            print(f"[WARN] 微信二维码检测器**不可用** —— 主摄扫码会一直超时却没有其它线索。"
                  f"原因: {_WECHAT_DET_ERR}；"
                  f"多半是缺 opencv-contrib-python（普通 opencv-python 没有 "
                  f"cv2.wechat_qrcode）。模型目录: {model_dir}")
    return _WECHAT_DET


def _decode_wechat(frame):
    """微信引擎解码一帧，返回与 _decode_opencv/_decode_pyzbar 同构的列表。

    detectAndDecode 返回 (解码字符串列表, 四边形顶点列表)；无码返回空列表。
    ⚠️ 检测器拿不到时**返回空列表但已在 `_get_wechat_detector` 打过日志** ——
    这里不要再吞异常（吞了就等于把"没装检测器"伪装成"没解出来"）。
    """
    results = []
    det = _get_wechat_detector()
    if det is None:
        return results
    try:
        res, points = det.detectAndDecode(frame)
    except Exception as e:
        print(f"[WARN] 微信引擎解码异常: {type(e).__name__}: {e}")
        return results
    if not res:
        return results
    for data, pts in zip(res, points):
        data = (data or "").strip()
        if not data:
            continue
        poly = None
        try:
            arr = np.asarray(pts, dtype=np.float32).reshape(-1, 2)
            if len(arr):
                poly = [(int(x), int(y)) for x, y in arr]
        except Exception:
            poly = None
        results.append({"data": data, "poly": poly})
    return results


def run_qr_wechat(timeout=None, abort_event=None):
    """微信二维码引擎识别（OpenCV wechat_qrcode，限时 timeout 秒）。**扫码副摄**。

    Returns dict: {"success", "code", "method", "reason", "frame"}。
    """
    return _run_task("wechat", _decode_wechat, _get_timeout(timeout), abort_event)


def run_qr_wechat_main(timeout=None, abort_event=None):
    """微信二维码引擎识别，但用**主摄**（/dev/camera_main）—— 命令 "D"。

    与 run_qr_wechat 是**同一条解码链**，只换了相机与配套的预处理参数
    （config 的 QR_MAIN_SCAN_*，默认不中心裁剪，见文件头）。
    返回结构与 run_qr_wechat 完全一致 —— 单片机侧因此能用同一套结果帧解析。
    """
    return _run_task("wechat", _decode_wechat, _get_timeout(timeout), abort_event,
                     use_main=True)


# ---- 自研难度算法（候选集模式，见 tasks/_qr_diff.py）----
#
# 治的是 wechat 治不了的图：斜视 / 模块像素太少 / 强透视。代价是**慢**
# （PC 上约 1s/帧，Pi 上按 3~5× 估是数秒），所以它是手动选的第二档，不做自动兜底。
#
# ⚠️ 模块级**不 import** `_qr_diff`：那个文件缺了/坏了会让整个 tasks 包 import 失败
#    → 主程序直接起不来，而症状只是「扫码不行」。改成懒加载 + 一次性响亮报错，
#    与 `_get_wechat_detector` 同一条纪律：**「解码器没装」必须和「没解出来」分得开**。
_DIFF_MOD = None
_DIFF_MOD_ERR = None

# 最近一次自研解码的完整结果（含 warp/采样矩阵），失败留档用
_DIFF_LAST = {}

# 一轮自研尝试最少要留这么多秒才值得起跑（实测最便宜的一轮 ~1.35s，见 _decode_diff）
_DIFF_MIN_SLICE_S = 1.6

# 「剩余不足、不再起新一轮」只报一次（那一判每帧都会走）
_DIFF_SKIP_LOGGED = False

# 候选集解析缓存（按 spec 字符串键控；config 改了下次调用就重算）
_KNOWN_CACHE = {}


def _get_diff_module():
    global _DIFF_MOD, _DIFF_MOD_ERR
    if _DIFF_MOD is None and _DIFF_MOD_ERR is None:
        try:
            # ⚠️ 必须走**包路径**，不能写 `import _qr_diff`：本目录（tasks/）不在
            # sys.path 上，只有仓库根在。在子模块里写顶层名会
            # `ModuleNotFoundError: No module named '_qr_diff'`，而症状只是
            # 「选自研的扫码步骤一直超时」（已在 Pi 上实测踩到）。
            from tasks import _qr_diff
            _DIFF_MOD = _qr_diff
        except Exception as e:
            _DIFF_MOD_ERR = f"{type(e).__name__}: {e}"
            print(f"[WARN] 自研二维码解码器**不可用** —— 选「自研」的扫码步骤会一直失败"
                  f"却没有其它线索。原因: {_DIFF_MOD_ERR}；"
                  f"检查 tasks/_qr_diff.py 是否存在、以及 numpy/cv2 是否可用")
    return _DIFF_MOD


def _get_diff_known():
    """候选集（**每次现取 config**，Web 调参改完下一帧就生效）。"""
    import config
    spec = config.QR_DIFF_KNOWN
    v = _KNOWN_CACHE.get(spec)
    if v is None:
        mod = _get_diff_module()
        if mod is None:
            return []
        try:
            v = mod.parse_known_spec(spec)
        except Exception as e:
            # 配错了就吼出来：配错的自研模式会**每次都失败**，而失败原因只有一条
            # 「候选集未命中」，不说清的话现场会以为是算法不行
            print(f"[WARN] 自研候选集配置 QR_DIFF_KNOWN={spec!r} 非法: {e}")
            v = []
        _KNOWN_CACHE.clear()
        _KNOWN_CACHE[spec] = v
        print(f"[INFO] 自研候选集: {spec!r} -> {len(v)} 个候选")
    return v


def _get_diff_timeout(timeout):
    """自研**自己的**预算。**

    ⚠️ 此刻它只是**兜底**：从串口来的指令由单片机随命令下发时限（`"F,7500"`），
       handler 用 CMD_MARGIN_S 减完余量再传进来（见 handler._budget_from_ms）。
       这个函数管的是没给时限的场合 —— 主要是网页 /scan 就地扫码。
    ⚠️ 不能用 QR_SCAN_TIMEOUT：那是 wechat 那条路共用的（默认 5s），为自研抬高
       会连带把 wechat 的超时也拉长。
    """
    if timeout is not None:
        return timeout
    from config import QR_DIFF_TIMEOUT
    return QR_DIFF_TIMEOUT


def _decode_diff(frame):
    """自研难度算法解码一帧（候选集模式），返回与其它 _decode_* 同构的列表。

    ⚠️ 一次调用 = **一次完整的几何重建 + 候选集匹配**（PC 上约 1s、Pi 上数秒），
       不是「读一帧判一帧」。没命中就返回空列表，让 _scan_qr 的循环进入下一轮
       （换一帧再试 —— 换帧通常比在同一帧上重试有用：曝光/抖动都不一样）。

    ⚠️ 故意**不返回 poly**：它的几何建立在自有的 1320×1320 画布上，坐标要经
       Hcur⁻¹ 才能映射回 proc 空间（而且中间还可能整图放大过 k 倍）。_annotate 对
       poly=None 已有分支，画个角标就够 —— 别为了画个框去凑一个会画错的变换。
    """
    mod = _get_diff_module()
    if mod is None:
        return []
    contents = _get_diff_known()
    if not contents:
        return []
    # 止损点优先用**本轮扫描的总 deadline**（_scan_qr 设的），拿不到才退回自己的预算：
    # 用 "now + QR_DIFF_TIMEOUT" 的话，循环里的第二次尝试会从头再拿一整个预算。
    if _SCAN_DEADLINE is not None:
        deadline = _SCAN_DEADLINE
    else:
        deadline = time.time() + _get_diff_timeout(None)

    # ⚠️ 剩余时间不够跑一轮就**别再起跑**。解码器只在各阶段**边界**查 deadline，
    #    而最便宜的一轮（定位角就没找着）也要 ~1.35s —— 在 deadline 前 0.1s 起跑
    #    照样会把这 1.35s 跑完。实测：预算 6s 的那次扫描实际跑了 7.13s，顶穿了
    #    「Pi 必须比固件先放弃」的契约（见 _get_diff_timeout 注释）。
    #    跳过时**保留本轮的上一轮结果**（_DIFF_LAST 不动）——失败留档里那份比空着有用。
    #    ⚠️ 但它的有效范围**只在同一次扫描内**（_run_diff 进扫描前会 clear）: 不 clear 的话
    #       一次"因为剩余时间不足而根本没起跑"的扫描, 会把**上一次扫描**（可能是成功那次）
    #       的 frame/采样矩阵当成这一次的产物写进 logs/Difficult/<新时间戳>/, 排障时
    #       会把错误位姿的结论当成这一次的。
    if deadline - time.time() < _DIFF_MIN_SLICE_S:
        # 只报一次：这段是「每读一帧就判一次」的，不打标记会刷满日志环
        global _DIFF_SKIP_LOGGED
        if not _DIFF_SKIP_LOGGED:
            _DIFF_SKIP_LOGGED = True
            print(f"[INFO] 自研: 剩余不足 {_DIFF_MIN_SLICE_S:.1f}s，不再起新一轮"
                  f"（本轮已试过的结论见上一条）")
        return []

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    r = mod.decode_gray(gray, contents=contents, deadline=deadline,
                        want_artifacts=True)
    _DIFF_LAST.clear()
    _DIFF_LAST.update(r)
    _DIFF_LAST["frame"] = frame          # 喂进解码器的那一帧（留档用）
    if not r["ok"]:
        # 失败原因打到日志：它是唯一能区分「定位不到」「几何没收敛」「采样错」
        # 「不在候选集里」的东西（见 _qr_diff 的 stage 字段）
        print(f"[INFO] 自研: %s (%.2fs, score=%s) -> %s"
              % (r["stage"], r["elapsed"], r["score"], r["reason"]))
        return []
    return [{"data": r["text"], "poly": None}]


def _dump_diff_failure_bg(snapshot, frame):
    """失败留档的后台执行体（异常一律吞掉 —— 留档失败不许影响任务结果）。"""
    try:
        mod = _get_diff_module()
        import config
        stamp = time.strftime("%Y%m%d_%H%M%S")
        p = mod.dump_failure(snapshot, frame=frame,
                             out_dir=os.path.join(config.QR_DIFF_DEBUG_DIR, stamp),
                             keep=config.QR_DIFF_DEBUG_KEEP)
        if p:
            print(f"[INFO] 自研扫码失败留档: {p}")
    except Exception as e:                            # noqa: BLE001
        print(f"[WARN] 自研失败留档出错: {type(e).__name__}: {e}")


def _run_diff(timeout, abort_event, use_main):
    """自研扫码：跑一次，失败则把中间产物留档（logs/Difficult/<时间戳>/）。

    ⚠️⚠️ **留档必须异步**（2026-10 修）。它原来是同步做的，而它排在**结果帧之前**：
    handler._cmd_qr_common 要等 run_fn 返回才发 "QRF"，而留档要写 4 个文件
    （frame.jpg / warp.png / matrix.npy+.png / hits.txt）+ `_prune` 删旧目录
    （SD 卡上每次 50~300ms）—— 实测量级 **0.3~1.5s**。
    于是「收指令 → QRF」= 开相机 + 扫描满预算(6.0s) + 留档，冷开时能到 **8~11s**，
    超过单片机等的 7500ms → **迟到的 QRF 落进下一次重扫的等待窗口**被当成那一次的结果
    （协议 §6 警告的那个错位；CLAUDE.md「实测把 7.13s 收到 6.04s」测的是扫描段，
    没算这一段）。
    丢进守护线程后：结果帧先出去，留档在后台慢慢写（快照一份, 免得下一次扫描把
    _DIFF_LAST 覆盖掉）。
    """
    _DIFF_LAST.clear()      # 本次扫描从零开始: 别让上一次扫描的产物冒充这一次的（见上）
    result = _run_task("diff", _decode_diff, timeout, abort_event, use_main=use_main)
    if result.get("success") or result.get("aborted") or not _DIFF_LAST:
        return result
    try:
        snapshot = dict(_DIFF_LAST)     # 快照: 后台读的时候下一次扫描可能正在写它
        frame = snapshot.get("frame")
        threading.Thread(target=_dump_diff_failure_bg, args=(snapshot, frame),
                         name="qr-diff-archive", daemon=True).start()
    except Exception as e:                            # noqa: BLE001
        print(f"[WARN] 自研失败留档线程启动失败: {type(e).__name__}: {e}")
    return result


def run_qr_diff(timeout=None, abort_event=None):
    """自研难度算法识别二维码。**扫码副摄** —— 命令 "E"。

    与 wechat 那条**同一条链路**（相机/重试/结果帧都一致），只有解码器不同，
    所以单片机侧用同一套 ucRpiWaitQrResult / iGetQrCode 就够了。
    """
    return _run_diff(_get_diff_timeout(timeout), abort_event, use_main=False)


def run_qr_diff_main(timeout=None, abort_event=None):
    """自研难度算法识别二维码，用**主摄** —— 命令 "F"（斜视扫码那条路）。

    与 run_qr_diff 是同一个实现，只换相机与预处理参数。
    """
    return _run_diff(_get_diff_timeout(timeout), abort_event, use_main=True)


# ==================== 重构版扫码（2026-10）: 定位→紧裁→强放大 ====================
#
# 为什么加这条（现场实测数据见 tasks/_qr_auto.py 文件头）:
#   旧链路「固定中心裁剪 0.30 + 微信引擎」在现场真帧上的基线只有 **2/10**，且平移
#   6% 就全灭 —— 根因是固定裁剪把码的位置写死了。新引擎改成「先定位、再按检测框
#   紧裁放大」，同一批真帧上基线 10/10、旋转/平移全稳、合计 95%，耗时还更短。
#
# ⚠️ 对外**完全同构**: 返回结构、`QR<ddd>`/`QRF` 结果帧、111~333 取值域校验都与
#    wechat 那条一字不差 —— 单片机侧的 ucRpiWaitQrResult 与下游一个字都不用改。
# ⚠️ 解码引擎在 `_qr_auto.py`（独立模块，只吃一帧）; 这里只负责「相机循环 + 预算 +
#    结果帧 + 归档」这些与硬件相关的事。**不要**把定位算法搬回本文件。

# 一次尝试最少要留这么多秒才起跑。新引擎一轮 = 读帧 + 定位(~17ms) + 放大解码，
# 命中路径实测 57ms、全阶梯失败路径 ~0.5s（含 2x 上采样档）。
# 取 0.30: 够跑完一次「命中」, 又不至于在 deadline 前 0.1s 起跑把预算顶穿。
_AUTO_MIN_ATTEMPT_S = 0.30


def _annotate_auto(frame, code, info, method):
    """结果帧标注：有检测框就画框 + 文字，没有就只写文字。"""
    out = frame.copy()
    bb = info.get("bbox")
    if bb:
        cx, cy, bw, bh = bb
        x0 = int(cx - bw / 2)
        y0 = int(cy - bh / 2)
        cv2.rectangle(out, (x0, y0), (x0 + int(bw), y0 + int(bh)), (0, 255, 0), 2)
        tx, ty = max(0, x0), max(18, y0 - 8)
    else:
        tx, ty = 10, 30
    stage = info.get("stage", "")
    cv2.putText(out, f"{code} ({method}/{stage})", (tx, ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
    return out


def _scan_qr_auto(cap, timeout, abort_event, use_main=False, qe=None):
    """新引擎的扫描循环：读帧 → decode_frame → 三位数校验 → 命中即返回。

    与 `_scan_qr` 的**接口与出口完全一致**（同样的 dict、同样的 aborted 语义），
    差别只在「每帧怎么解」：不再做固定中心裁剪，而是交给 `_qr_auto.decode_frame`
    在整帧上定位后按检测框放大。
    """
    if qe is None:
        from tasks import _qr_auto as qe
    cam = _cam_label(use_main)
    deadline = time.time() + timeout
    last_frame = None
    info_best = {}
    n_attempt = 0
    t_start = time.time()

    while time.time() < deadline:
        # ⚠️ 中止判定排在 min_slice 之前（理由同 _scan_qr 的同名注释：被中止的旧任务
        #    不能落到「普通失败」出口去回一条 QRF，那会顶替下一轮的结果）。
        if abort_event is not None and abort_event.is_set():
            return {"success": False, "aborted": True, "code": None,
                    "method": "auto", "reason": "", "frame": last_frame}
        # 剩余时间不够跑完一轮就别起跑（命中路径 ~60ms, 但失败路径可能 0.5s）
        if deadline - time.time() < _AUTO_MIN_ATTEMPT_S:
            break

        frame = CameraManager.read(cap, cam, max_retries=3)
        if frame is None:
            time.sleep(0.03)
            continue
        last_frame = frame
        n_attempt += 1

        code, info = qe.decode_frame(frame, deadline=deadline,
                                     wechat_getter=_get_wechat_detector)
        info_best = info
        if code:
            info["attempts"] = n_attempt
            annotated = _annotate_auto(frame, code, info, "auto")
            return {"success": True, "code": code, "method": "auto", "reason": "",
                    "frame": annotated, "frame_raw": frame, "info": info}
        time.sleep(0.005)

    oob = (info_best or {}).get("oob")
    if oob is not None:
        reason = (f"auto 超时 ({timeout:.0f}s)：读到 {oob}，但每位必须是 1~3"
                  f"（赛题取值域 111~333）")
    elif (info_best or {}).get("cands"):
        reason = (f"auto 超时 ({timeout:.0f}s)：定位到二维码"
                  f"（{info_best.get('qr_px')}px）但解不出三位码")
    else:
        reason = f"auto 超时 ({timeout:.0f}s)：未检测到二维码"
    return {"success": False, "code": None, "method": "auto",
            "reason": reason, "frame": last_frame, "info": info_best}


def _run_auto(timeout, abort_event, use_main):
    """新引擎的统一入口：开相机 → 扫描 → 释放（结构照抄 _run_task）。"""
    from tasks import _qr_auto as qe
    # ⚠️ 缺 zxingcpp 轮子时**不能**在这里静默失败 —— 新引擎全靠它。退回旧的 wechat
    #    链路并**响亮报一行**，否则现场只会看到「扫码突然变回老样子」而查不出原因。
    if not getattr(qe, "ZXING_OK", False):
        print("[WARN] tasks/_qr_auto 依赖 zxing-cpp 但未安装 —— 本次退回旧的微信链路。"
              "装法: python3 -m pip install --user --break-system-packages zxing-cpp")
        return _run_task("wechat", _decode_wechat, timeout, abort_event, use_main=use_main)

    cap = _open_with_retry(abort_event, use_main=use_main)
    if cap is None:
        if abort_event is not None and abort_event.is_set():
            print("[INFO] 二维码（auto）任务被打断")
            return {"success": False, "aborted": True, "code": None,
                    "method": "auto", "reason": "", "frame": None}
        return {"success": False, "code": None, "method": "auto",
                "reason": "二维码摄像头打开失败", "frame": None}
    try:
        print(f"[INFO] 开始二维码检测 (auto)，限时 {timeout:.0f}s"
              f"{'，主摄' if use_main else ''}...")
        t0 = time.time()
        result = _scan_qr_auto(cap, timeout, abort_event, use_main=use_main, qe=qe)
        result["elapsed"] = time.time() - t0
        info = result.get("info") or {}
        if result["success"]:
            print(f"[INFO] 检测到二维码 [auto/{info.get('stage')}]: {result['code']}"
                  f"（用时 {result['elapsed']:.2f}s, {info.get('attempts')} 次尝试, "
                  f"码 {info.get('qr_px')}px）")
        elif result.get("aborted"):
            print("[INFO] 二维码（auto）任务被打断")
        else:
            print(f"[WARN] {result['reason']}（用时 {result['elapsed']:.2f}s）")
        return result
    finally:
        release_camera(cap)


def run_qr_auto(timeout=None, abort_event=None):
    """新引擎扫码（定位→紧裁→强放大）。**扫码副摄** —— 命令 "1"。

    与 run_qr_wechat 对外完全同构（同样的结果结构、同样的 QRddd/QRF 帧），
    只是每帧的解法换成了「先定位再放大」。
    """
    return _run_auto(_get_timeout(timeout), abort_event, use_main=False)


def run_qr_auto_main(timeout=None, abort_event=None):
    """新引擎扫码，用**主摄** —— 命令 "D"。与 run_qr_auto 同一实现，只换相机。"""
    return _run_auto(_get_timeout(timeout), abort_event, use_main=True)
