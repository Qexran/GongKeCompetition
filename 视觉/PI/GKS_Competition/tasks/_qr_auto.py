# tasks/_qr_auto.py — 二维码「先定位、再放大解码」引擎（2026-10 重构）
#
# 为什么重做（2026-10-23 现场实测, 副摄 1920x1080）:
#   现场停车位上二维码在画面里只有 **46x46 px**（V1 含静默区约 29 模块 → ~1.6 px/模块,
#   已在通用解码器的理论下限附近）。旧链路「固定中心裁剪 0.30 + 微信引擎」在这批真帧上
#   **基线只有 2/10**，并且对纸张位置/微小角度极敏感 —— 平移 6% 就 0/10。
#   根因不是解码器弱，而是**固定裁剪把「码在画面哪个位置」写死**了：码一旦偏离裁剪窗
#   就整块丢出画面，再强的解码器也无力。
#
# 换成「先定位、再按检测框紧裁放大」之后（同一批真帧、同一硬件）:
#   基线 10/10; 旋转 0~25° 10/10; 平移 0~16% 10/10; 缩放 0.8~0.6 10/10; 合计 95%。
#   耗时还与旧链路相当（基线 57ms vs 112ms）—— 因为定位只花 ~17ms，且命中即返回。
#
# 三个关键机制（缺一个效果就掉档, 改之前先读完）:
#   1. **zxing 的「解不出也给位置」当检测器用**：`read_barcodes(return_errors=True)` 在
#      解不出时会返回 ErrorType.Checksum 的 Barcode, 但它带的 `position` 是**正确**的
#      码位置（现场实测每帧都是 46x46@(987,548)）。所以全帧只需跑一次 ~17ms 就有了
#      检测结果 —— 这比自己写定位角/轮廓检测稳得多, 也不受背景杂色影响。
#   2. **紧裁 + 强放大**：按检测框 1.6 倍边长裁出, 再放大到 960（码 46px → 约 290px）。
#      实测边距 1.6 明显优于 2.3/3.0 —— 放大倍数越大, 模块边界越干净。
#   3. **CLAHE 必须作用在「放大后」的裁剪图上**：整体偏亮/偏暗时只有它救得回来;
#      作用在全帧上无效（网格尺度不对, 实测 +30 亮度档 1/12）。
#
# ⚠️ 本模块**不做相机/串口**, 只吃一帧 BGR 返回三位码 —— 便于台架与离线跑分。
# ⚠️ 语义与旧链路严格一致：内容必须三位数字**且每位 1~3**（赛题取值域 111~333,
#    见 _qr.py::_is_valid_code 的注释）; 不合法的一律当"不是我们的码"继续找,
#    但把第一个越界读数带回 info，供失败原因使用。
import time

import cv2
import numpy as np

try:
    import zxingcpp
    ZXING_OK = True
except Exception:                     # pragma: no cover - 部署环境缺轮子时
    zxingcpp = None
    ZXING_OK = False

# 赛题取值域（与 _qr.py/_QR_DIGITS、固件 STEP_QR_TEST 同口径）
_QR_DIGITS = "123"

# 放大目标边长。960 在 Pi 上 zxing 只需 ~15ms；1280 约 28ms, 收益不明显（实测同分）。
_TARGET = 960

# 裁剪边距（相对检测框边长）。1.6 = 实测最优; 2.3 = 检测框偏小时的补救。
_MARGINS = (1.6, 2.3)

# 定位失败时的位置兜底阶梯（中心裁剪比例）。覆盖「码比 zxing 检测下限还小」。
_LADDER = (0.10, 0.14, 0.19, 0.26, 0.36, 0.50)

# 2x 上采样定位档：只在预算够（>0.6s）时才跑。实测把「码缩到 0.6 倍」从 2/10 救到 10/10。
_UPSCALE2_MIN_BUDGET_S = 0.6
_UPSCALE2_FACTOR = 2.0

_CLAHE = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

_BINARIZERS = None


def _binarizers():
    """二值化器，按性价比排序：GlobalHistogram 最便宜（Pi 上 6.9ms vs LocalAverage 15ms）。"""
    global _BINARIZERS
    if _BINARIZERS is None:
        b = zxingcpp.Binarizer
        _BINARIZERS = (b.GlobalHistogram, b.LocalAverage, b.FixedThreshold)
    return _BINARIZERS


def _text_of(b):
    t = b.text
    if isinstance(t, (bytes, bytearray)):
        try:
            t = t.decode("utf-8", "ignore")
        except Exception:
            return ""
    return (t or "").strip()


def _is_digits3(t):
    return bool(t) and len(t) == 3 and t.isdigit()


def _is_valid(t):
    """三位数字**且每位 1~3**（= 111~333）。"""
    return _is_digits3(t) and all(ch in _QR_DIGITS for ch in t)


def _bbox_of(b):
    """Barcode.position → (cx, cy, bw, bh)；拿不到返回 None。"""
    p = getattr(b, "position", None)
    if p is None:
        return None
    try:
        xs = (p.top_left.x, p.top_right.x, p.bottom_right.x, p.bottom_left.x)
        ys = (p.top_left.y, p.top_right.y, p.bottom_right.y, p.bottom_left.y)
    except Exception:
        return None
    return (sum(xs) / 4.0, sum(ys) / 4.0, max(xs) - min(xs), max(ys) - min(ys))


def _zxing_scan(gray, want_errors=True):
    """跑一遍 zxing，返回 (code|None, cands, oob_text)。

    cands = [(cx,cy,bw,bh)]（含解不出来的那些 —— 正是它们提供了位置）。
    一旦某个二值化器给出了候选就直接用它的结果，不再换下一个（省时间）。
    """
    cands = []
    oob = None
    for bz in _binarizers():
        try:
            res = zxingcpp.read_barcodes(
                gray, formats=zxingcpp.BarcodeFormat.QRCode,
                binarizer=bz, return_errors=want_errors)
        except Exception:
            continue
        for b in res:
            t = _text_of(b)
            if _is_valid(t):
                return t, cands, oob
            if _is_digits3(t) and oob is None:
                oob = t
            bb = _bbox_of(b)
            if bb is not None:
                cands.append(bb)
        if cands or oob is not None:
            break
    return None, cands, oob


def _crop_around(gray, cx, cy, side):
    H, W = gray.shape[:2]
    side = int(max(32, min(side, min(H, W))))
    x0 = max(0, min(W - side, int(round(cx - side / 2.0))))
    y0 = max(0, min(H - side, int(round(cy - side / 2.0))))
    return gray[y0:y0 + side, x0:x0 + side]


def _crop_center(gray, ratio):
    H, W = gray.shape[:2]
    return _crop_around(gray, W / 2.0, H / 2.0, int(min(H, W) * ratio))


def _try_tile(tile, deadline, target=_TARGET):
    """一张裁剪 → 放大 → (原图 + CLAHE) × 多个二值化器。返回 (code|None, oob)。

    ⚠️ CLAHE 作用在**放大后**的图上（见文件头第 3 条）；对原尺寸裁剪做 CLAHE 没用。
    """
    if tile is None or tile.size < 64:
        return None, None
    up = cv2.resize(tile, (target, target), interpolation=cv2.INTER_LINEAR)
    oob = None
    for v in (up, _CLAHE.apply(up)):
        for bz in _binarizers():
            if deadline is not None and time.time() > deadline:
                return None, oob
            try:
                res = zxingcpp.read_barcodes(
                    v, formats=zxingcpp.BarcodeFormat.QRCode, binarizer=bz)
            except Exception:
                continue
            for b in res:
                t = _text_of(b)
                if _is_valid(t):
                    return t, oob
                if _is_digits3(t) and oob is None:
                    oob = t
    return None, oob


def _remaining(deadline):
    return float("inf") if deadline is None else (deadline - time.time())


def decode_frame(bgr, deadline=None, wechat_det=None, wechat_getter=None,
                 use_upscale2=True):
    """一帧 → 三位码。返回 (code|None, info)。

    info = {"stage", "ms", "cands", "qr_px", "oob"}
      stage: 命中的阶段（zxing-full / zoom / ladder / upscale2 / wechat）或 miss/deadline
      qr_px: 检测到的码边长（px）—— 现场调机位的唯一量化依据
      oob  : 读到过但不在 111~333 的读数（失败原因用）

    ⚠️ 每一阶段前都查 deadline：本函数被 `_scan_qr_auto` 的循环反复调用，
       单次超预算会让整轮扫描顶穿固件愿意等的时间（见 _qr.py 的同名纪律）。
    """
    t0 = time.time()
    info = {"stage": "no-frame", "ms": 0.0, "cands": 0, "qr_px": 0, "oob": None,
            "bbox": None}
    if bgr is None:
        return None, info
    gray = bgr if bgr.ndim == 2 else cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    def done(code, stage, oob=None):
        info["stage"] = stage
        info["ms"] = (time.time() - t0) * 1000.0
        if oob is not None and info["oob"] is None:
            info["oob"] = oob
        return code, info

    # ---- 阶段 1: 全帧定位（顺带命中; 约 17ms）----
    code, cands, oob = _zxing_scan(gray, want_errors=True)
    src = gray
    if code:
        return done(code, "zxing-full", oob)
    if not cands:
        # 定位不到时换 CLAHE 全帧再试一次（偏亮/偏暗场景; 约 +17ms）
        gc = _CLAHE.apply(gray)
        code, cands2, oob2 = _zxing_scan(gc, want_errors=True)
        if code:
            return done(code, "zxing-full-clahe", oob2)
        if cands2:
            cands, src = cands2, gc
    info["cands"] = len(cands)
    if cands:
        info["qr_px"] = int(max(cands[0][2], cands[0][3]))

    # ---- 阶段 2: 候选处紧裁 + 放大解码 ----
    # 按检测框面积从大到小（赛场上最大的通常就是我们的码; 也倾向选更近、更清晰的那个）。
    for (cx, cy, bw, bh) in sorted(cands, key=lambda c: -(c[2] * c[3]))[:3]:
        for margin in _MARGINS:
            if _remaining(deadline) <= 0:
                return done(None, "deadline", oob)
            tile = _crop_around(src, cx, cy, int(max(bw, bh, 24) * margin))
            code, o = _try_tile(tile, deadline)
            if o is not None and oob is None:
                oob = o
            if code:
                info["bbox"] = (cx, cy, bw, bh)   # 只用于结果帧标注
                return done(code, "zoom", oob)

    # ---- 阶段 3: 位置兜底阶梯（定位彻底失败: 码太小/太糊）----
    for ratio in _LADDER:
        if _remaining(deadline) <= 0:
            return done(None, "deadline", oob)
        code, o = _try_tile(_crop_center(gray, ratio), deadline)
        if o is not None and oob is None:
            oob = o
        if code:
            return done(code, "ladder", oob)

    # ---- 阶段 4: 2x 上采样再定位（停得更远/码更小的场合）----
    # ⚠️ 这一步最贵（~0.5s）。只在预算够时才跑, 否则会顶穿本轮 deadline。
    if (use_upscale2 and ZXING_OK
            and _remaining(deadline) > _UPSCALE2_MIN_BUDGET_S):
        g2 = cv2.resize(gray, None, fx=_UPSCALE2_FACTOR, fy=_UPSCALE2_FACTOR,
                        interpolation=cv2.INTER_CUBIC)
        code, cands2, oob2 = _zxing_scan(g2, want_errors=True)
        if oob2 is not None and oob is None:
            oob = oob2
        if code:
            return done(code, "upscale2", oob)
        if not cands2:
            g2c = _CLAHE.apply(g2)
            code, cands2, oob2 = _zxing_scan(g2c, want_errors=True)
            if oob2 is not None and oob is None:
                oob = oob2
            if code:
                return done(code, "upscale2", oob)
            if cands2:
                g2 = g2c
        for (cx, cy, bw, bh) in sorted(cands2, key=lambda c: -(c[2] * c[3]))[:2]:
            for margin in _MARGINS:
                if _remaining(deadline) <= 0:
                    return done(None, "deadline", oob)
                tile = _crop_around(g2, cx, cy, int(max(bw, bh, 32) * margin))
                code, o = _try_tile(tile, deadline)
                if code:
                    return done(code, "upscale2-zoom", oob)
        for ratio in (0.10, 0.15, 0.22):
            if _remaining(deadline) <= 0:
                break
            code, o = _try_tile(_crop_center(g2, ratio), deadline)
            if code:
                return done(code, "upscale2-ladder", oob)

    # ---- 阶段 5: 微信引擎复核（CNN, 抗旋转最强; Pi 上 ~88ms/次）----
    # 定位/放大都对它没用时, 它偶尔还能靠超分救回来 —— 只在还有预算时试几档。
    # ⚠️ 检测器**不在扫描开头就加载**（CNN 模型加载要几百 ms, 会白白吃掉本轮预算）:
    #    传进来的是 getter, 只有走到这一步且预算够时才真正加载。
    det = wechat_det
    if det is None and wechat_getter is not None and _remaining(deadline) > 1.5:
        try:
            det = wechat_getter()
        except Exception:
            det = None
    if det is not None and _remaining(deadline) > 0.2:
        for ratio in (0.22, 0.30, 0.45):
            if _remaining(deadline) <= 0:
                break
            try:
                res, _ = det.detectAndDecode(_crop_center(gray, ratio))
            except Exception:
                res = []
            for t in res:
                t = (t or "").strip()
                if _is_valid(t):
                    return done(t, "wechat", oob)
                if _is_digits3(t) and oob is None:
                    oob = t

    return done(None, "miss", oob)
