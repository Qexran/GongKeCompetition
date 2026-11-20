# -*- coding: utf-8 -*-
"""自研难度二维码解码器（候选集模式）—— 移植自桌面项目 DiffcultQR。

原项目(`decode_qr.py` + `qrdecode.py`)是个命令行工具,专治屏幕拍摄/强透视/摩尔纹这类
「微信能扫、zxing/pyzbar/OpenCV 全失败」的硬图:先用三次形变场把二维码几何重建出来,
再在**已知候选集**里用 ECC 字节精确匹配定出内容。

移植时做了四件事,每一件都有理由:

1. **抽成库**(`decode_gray(gray, ...) -> dict`):原版读 `sys.argv`、四处 `sys.exit()`,
   只 `print` 不返回,任务层没法用。现在失败也返回,不抛异常。
2. **只留候选集模式**。赛题二维码内容固定是三位数 `111~333`(共 27 种),而通用暴破要
   枚举 15.5 万个候选、每个还按 4 档 ECC 各做一次 RS 编码 —— 实测 ~33s,其中 93% 花在
   这个编码上。**在线链路里根本跑不完**,所以整段砍掉。
3. **零新依赖**。Pi 上没装 reedsolo,而原版在模块级 `from reedsolo import ...`(缺了就
   import 失败)。这里自带一个极小的纯 Python GF(256)/RS 编码器,只用来在 import 时
   预生成那 27×4 条候选码字表(108 次,几十毫秒,不进热路径),于是解码只需要
   `cv2 + numpy`。用 `python -c "import reedsolo"` 能导入时,还额外提供 `try_rs_decode`
   作为离线诊断用的一般解码(在线链路不走它)。
4. **加了四个安全阀**:放大倍数封顶(原版无上限,小图会 resize 出 ~400MB 直接 OOM)、
   版本守卫(全流程写死 `N=21`,不是 version 1 就早退而不是拿 21 的网格去采样出垃圾)、
   候选规格非法时响亮报错(原版会静默丢弃并悄悄回落到 33s 暴破)、以及 deadline 取消。

⚠️ 本模块**刻意不含快速通道**(zxing/pyzbar/OpenCV) —— 那是 wechat 那条路的职责。
自研模式要的是「wechat 解不出来时能不能补上」,混进别的解码器会让 A/B 对比失去意义。

⚠️ 全流程对 **version 1(21×21)** 是硬编码的:定位角之间的模块间距、`ECC_INFO` 的
26 个码字、`function_map` 的暗模块位置都按 V1 写死。喂进 V2+ 的码不会报错,只会采样出
垃圾 —— 所以 `est_n` 守卫必须留着。
"""
import os
import time

import cv2
import numpy as np

# ---------- 常量（V1 硬编码，见模块 docstring）----------
N = 21          # 模块数（version 1）
PPM = 40        # 矫正画布上每模块的像素数
QZ = 6          # 静默区模块数
S = (N + 2 * QZ) * PPM      # 画布边长 = 1320
OFF = QZ * PPM              # 画布原点到模块(0,0)的像素偏移 = 240

# 定位角中心之间相隔的模块数（V1: 21-7=14）
_FINDER_SPAN = N - 7

# 放大倍数上限。原版 `k = ceil(14/msize)` 无上限，小码会 resize 出上亿像素 → OOM。
_MAX_UPSCALE = 4

# est_n（模块数估计）守卫的容差带。
#
# ⚠️ 这个估计量**尺度无关但有一个未标定的固定偏差**：在种子样张上（含两张模块像素
#    差 27% 的图）它稳定给出 18.68~18.84，而真值是 21。既然不能用同一块码板的 7 张
#    去标定那个常数，这里就把带子放得很宽 —— 它要抓的是「**显然不是 V1**」（比如
#    V5 的 37 模块、或者把非二维码的黑块误当成定位角），不是精确判版本。
#
# ⚠️ 真正防「拿 21 的网格去采样 V2+ 产出垃圾」的其实**不是**这道门，而是候选集解码
#    本身：它要求 26 个码字里至少 6 个校验字节**逐位相同**（误命中概率可忽略），
#    所以几何错位只会导致「解不出来」，不会导致「解出一个错的数」。这道门的作用是
#    把「一秒白跑 + 一句含糊的候选集未命中」换成「一句说清原因的早退」。
_EST_N_LO = 13.0
_EST_N_HI = 30.0


# ============================================================
# 快速百分位（唯一的性能关键点）
# ============================================================
def _pct(a, q):
    """等价于 np.percentile(a, q)（默认 'linear' 插值法），但快得多。

    为什么值得单独写：`first_fall_from_outside` 每调用一次就要算两个百分位，而它被
    调用上万次（每条 finder 边每 2px 采一次样）。`np.percentile` 单次调用约 80µs，
    几乎全是 numpy 的调度开销（`_lerp`/`_unique1d`/`issubdtype`），实测占了整条管线
    一半的时间；换成「排序 + 手工线性插值」后同样元素数只要约 3µs。
    """
    n = a.size
    if n == 0:
        return 0.0
    if n == 1:
        return float(a[0])
    s = np.sort(a)
    idx = (n - 1) * (float(q) / 100.0)
    lo = int(idx)
    if lo >= n - 1:
        return float(s[n - 1])
    f = idx - lo
    return float(s[lo]) + (float(s[lo + 1]) - float(s[lo])) * f


# ============================================================
# 阶段 A：定位角
# ============================================================
def find_finder_centers(gray):
    """三个定位角中心。返回 (centers[3], clusters[3]) 或 None。

    3×3×2 = 18 组二值化（块大小 × 常数 × 两种极性），靠 49:25:9 的嵌套面积比筛出定位角，
    再按中心距 25px 聚类、按平均尺寸取最大的三个（真定位角最大）。
    """
    found = []
    for blk in [51, 75, 101]:
        for C in [5, 10, 15]:
            bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY, blk, C)
            for pol in [0, 1]:
                b = bw if pol == 0 else 255 - bw
                cnts, hier = cv2.findContours(b, cv2.RETR_TREE,
                                              cv2.CHAIN_APPROX_SIMPLE)
                if hier is None:
                    continue
                hier = hier[0]
                for i, c in enumerate(cnts):
                    a = cv2.contourArea(c)
                    if a < 400:
                        continue
                    ch = hier[i][2]
                    if ch < 0:
                        continue
                    ca = cv2.contourArea(cnts[ch])
                    if ca <= 0 or not (0.4 < ca / a < 0.65):
                        continue
                    gh = hier[ch][2]
                    if gh < 0:
                        continue
                    ga = cv2.contourArea(cnts[gh])
                    if ga / ca < 0.2:
                        continue
                    M = cv2.moments(c)
                    if M['m00'] == 0:
                        continue
                    found.append((M['m10'] / M['m00'], M['m01'] / M['m00'],
                                  np.sqrt(a), c))
    if not found:
        return None
    clusters = []
    for x, y, w, cnt in found:
        placed = False
        for cl in clusters:
            if np.hypot(x - cl[0] / cl[3], y - cl[1] / cl[3]) < 25:
                cl[0] += x
                cl[1] += y
                cl[2] += w
                cl[3] += 1
                if cv2.contourArea(cnt) > cv2.contourArea(cl[4]):
                    cl[4] = cnt
                placed = True
                break
        if not placed:
            clusters.append([x, y, w, 1, cnt])
    clusters.sort(key=lambda c: -c[2] / c[3])      # 按平均尺寸（真定位角最大）
    centers = [np.array([c[0] / c[3], c[1] / c[3]]) for c in clusters]
    if len(centers) < 3:
        return None
    return centers[:3], clusters[:3]


def orient_centers(centers):
    """三个中心 → (TL, TR, BL)：直角在 TL，叉积定手性。"""
    dots = []
    for i in range(3):
        a = centers[(i + 1) % 3] - centers[i]
        b = centers[(i + 2) % 3] - centers[i]
        dots.append(abs(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))))
    i = int(np.argmin(dots))
    v = centers[i]
    a, b = centers[(i + 1) % 3], centers[(i + 2) % 3]
    cross = (b[0] - v[0]) * (a[1] - v[1]) - (b[1] - v[1]) * (a[0] - v[0])
    if cross > 0:
        return v, b, a
    return v, a, b


def quad_from_contour(cnt):
    """外轮廓 → 4 角点（凸包 + 逐步放宽的逼近）。"""
    hull = cv2.convexHull(cnt)
    peri = cv2.arcLength(hull, True)
    for eps in [0.02, 0.03, 0.05, 0.08, 0.12]:
        ap = cv2.approxPolyDP(hull, eps * peri, True)
        if len(ap) == 4:
            return ap.reshape(4, 2).astype(np.float64)
    return None


def order_quad(quad, center, xhat, yhat):
    """按二维码自身坐标系把 4 角点排成 tl,tr,br,bl。"""
    rel = quad - center
    a = rel @ xhat
    b = rel @ yhat
    s = a + b
    d = a - b
    tl = quad[int(np.argmin(s))]
    br = quad[int(np.argmax(s))]
    tr = quad[int(np.argmax(d))]
    bl = quad[int(np.argmin(d))]
    return np.array([tl, tr, br, bl])


def homography_from_quads(clusters3, TL, TR, BL):
    """三个定位角的外四边形 → 初始单应性（模块坐标系）。"""
    xhat = TR - TL
    xhat = xhat / (np.linalg.norm(xhat) + 1e-9)
    yhat = BL - TL
    yhat = yhat / (np.linalg.norm(yhat) + 1e-9)
    canon = {0: [(0, 0), (7, 0), (7, 7), (0, 7)],
             1: [(N - 7, 0), (N, 0), (N, 7), (N - 7, 7)],
             2: [(0, N - 7), (7, N - 7), (7, N), (0, N)]}
    srcp, dstp = [], []
    for i, cl in enumerate(clusters3):
        quad = quad_from_contour(cl[4])
        if quad is None:
            return None
        center = np.array([cl[0] / cl[3], cl[1] / cl[3]])
        q = order_quad(quad, center, xhat, yhat)
        for j in range(4):
            srcp.append(list(q[j]))
            dstp.append(list(canon[i][j]))
    H, _ = cv2.findHomography(np.array(srcp, np.float32),
                              np.array(dstp, np.float32), cv2.RANSAC, 1.0)
    return None if H is None else H.astype(np.float64)


def affine_from_centers(TL, TR, BL):
    """三个中心 → 仿射（模块坐标系）。"""
    ex = (TR - TL) / _FINDER_SPAN
    ey = (BL - TL) / _FINDER_SPAN
    A = np.array([[ex[0], ey[0]], [ex[1], ey[1]]])
    t = TL - 3.5 * ex - 3.5 * ey
    Ainv = np.linalg.inv(A)
    return np.array([[Ainv[0, 0], Ainv[0, 1], -(Ainv @ t)[0]],
                     [Ainv[1, 0], Ainv[1, 1], -(Ainv @ t)[1]],
                     [0, 0, 1]])


# ---------- A2：亚像素边缘 ----------
def first_fall_from_outside(prof, out_dir):
    """从外侧往里扫，找灰度第一次跌破中位线的位置（线性插值到亚像素）。

    ⚠️ 两个 `_pct` 是本模块最热的调用点（上万次），别换回 `np.percentile`。
    """
    L = len(prof)
    p = prof.astype(np.float64)
    outer = p[:max(4, L // 3)] if out_dir < 0 else p[-max(4, L // 3):]
    hi = _pct(outer, 80)
    lo = _pct(p, 10)
    if hi - lo < 30:
        return None
    mid = (hi + lo) / 2
    seq = p if out_dir < 0 else p[::-1]
    below = np.where(seq < mid)[0]
    if len(below) == 0 or below[0] == 0:
        return None
    i = below[0] - 1
    a, b = seq[i], seq[i + 1]
    if b >= a:
        return None
    x = i + (mid - a) / (b - a)
    return x if out_dir < 0 else L - 1 - x


def fit_edge(warp, axis, a0, a1, expect, out_dir):
    """沿一条边采样 → 直线拟合 (斜率, 截距) + 残差标准差。"""
    pos = []
    y_lo = int(max(0, expect - 1.5 * PPM))
    y_hi = int(min(warp.shape[0], expect + 1.5 * PPM))
    x_lo = int(max(0, expect - 1.5 * PPM))
    x_hi = int(min(warp.shape[1], expect + 1.5 * PPM))
    for t in np.arange(a0, a1, 2.0):
        ti = int(round(t))
        if axis == 'h':
            if ti < 0 or ti >= warp.shape[1]:
                continue
            prof = warp[y_lo:y_hi, ti]
        else:
            if ti < 0 or ti >= warp.shape[0]:
                continue
            prof = warp[ti, x_lo:x_hi]
        e = first_fall_from_outside(prof, out_dir)
        if e is None:
            continue
        pos.append((ti, (y_lo if axis == 'h' else x_lo) + e))
    if len(pos) < 12:
        return None, 0.0
    arr = np.array(pos)
    med = np.median(arr[:, 1])
    arr = arr[np.abs(arr[:, 1] - med) < 0.5 * PPM]
    if len(arr) < 10:
        return None, 0.0
    A = np.stack([arr[:, 0], np.ones_like(arr[:, 0])], 1)
    coef, *_ = np.linalg.lstsq(A, arr[:, 1], rcond=None)
    return coef, float(np.std(arr[:, 1] - A @ coef))


def finder_quad(warp, mx, my):
    """矫正图上某个定位角的 4 个外角点（模块坐标）。"""
    X0, Y0 = OFF + mx * PPM, OFF + my * PPM
    X7, Y7 = X0 + 7 * PPM, Y0 + 7 * PPM
    top = fit_edge(warp, 'h', X0 + 6, X7 - 6, Y0, -1)
    bot = fit_edge(warp, 'h', X0 + 6, X7 - 6, Y7, +1)
    left = fit_edge(warp, 'v', Y0 + 6, Y7 - 6, X0, -1)
    right = fit_edge(warp, 'v', Y0 + 6, Y7 - 6, X7, +1)
    if any(e[0] is None for e in [top, bot, left, right]):
        return None

    def inter(hl, vl):
        ah, bh = hl
        av, bv = vl
        y = (ah * bv + bh) / (1 - ah * av + 1e-9)
        x = av * y + bv
        return np.array([(x - OFF) / PPM, (y - OFF) / PPM])

    return dict(tl=inter(top[0], left[0]), tr=inter(top[0], right[0]),
                br=inter(bot[0], right[0]), bl=inter(bot[0], left[0]))


def timing_edge_points(warp):
    """时序图案（第 6 行/列交替黑白）的跳变点 → 锚点。"""
    pts = []
    strip = warp[int(OFF + 5.6 * PPM):int(OFF + 7.4 * PPM), :]
    p = strip.mean(axis=0).astype(np.float64)
    hi, lo = _pct(p, 85), _pct(p, 15)
    mid = (hi + lo) / 2
    d = np.diff(p)
    for k in range(6, 14):
        idx = np.where((np.arange(len(d)) >= OFF + (k - 0.7) * PPM) &
                       (np.arange(len(d)) < OFF + (k + 0.7) * PPM))[0]
        if len(idx) == 0:
            continue
        i = idx[int(np.argmin(d[idx]))] if k % 2 == 0 else idx[int(np.argmax(d[idx]))]
        jump = d[i]
        if (k % 2 == 0 and jump > -20) or (k % 2 == 1 and jump < 20):
            continue
        a, b = p[i], p[i + 1]
        if b == a:
            continue
        x = (i + (mid - a) / (b - a) - OFF) / PPM
        if abs(x - k) < 0.5:
            pts.append(((x, 6.5), (float(k), 6.5)))
    strip = warp[:, int(OFF + 5.6 * PPM):int(OFF + 7.4 * PPM)]
    p = strip.mean(axis=1).astype(np.float64)
    hi, lo = _pct(p, 85), _pct(p, 15)
    mid = (hi + lo) / 2
    d = np.diff(p)
    for k in range(6, 14):
        idx = np.where((np.arange(len(d)) >= OFF + (k - 0.7) * PPM) &
                       (np.arange(len(d)) < OFF + (k + 0.7) * PPM))[0]
        if len(idx) == 0:
            continue
        i = idx[int(np.argmin(d[idx]))] if k % 2 == 0 else idx[int(np.argmax(d[idx]))]
        jump = d[i]
        if (k % 2 == 0 and jump > -20) or (k % 2 == 1 and jump < 20):
            continue
        a, b = p[i], p[i + 1]
        if b == a:
            continue
        y = (i + (mid - a) / (b - a) - OFF) / PPM
        if abs(y - k) < 0.5:
            pts.append(((6.5, y), (6.5, float(k))))
    return pts


def cubic_design(u, v):
    """三次多项式基（10 项）的设计矩阵。"""
    return np.stack([np.ones_like(u), u, v, u * u, u * v, v * v,
                     u ** 3, u * u * v, u * v * v, v ** 3], 1)


def robust_fit(Amat, b, w0, iters=5):
    """IRLS（5 轮，Tukey 型权重）。"""
    w = w0.copy()
    coef = np.zeros(Amat.shape[1])
    for _ in range(iters):
        W = w[:, None]
        coef = np.linalg.lstsq(Amat * W, b * w, rcond=None)[0]
        r = b - Amat @ coef
        sc = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-6
        w = w0 * np.clip(1 - (np.abs(r) / (4.5 * sc)) ** 2, 0, 1) ** 2
    return coef


def collect_anchors(warp, coef_x, coef_y):
    """三定位角 12 条边 + 时序图案 → 亚像素锚点集（≈170 个）。"""
    anchors = []
    pos = {'TL': (0, 0), 'TR': (N - 7, 0), 'BL': (0, N - 7)}
    for k, (mx, my) in pos.items():
        X0, Y0 = OFF + mx * PPM, OFF + my * PPM
        X7, Y7 = X0 + 7 * PPM, Y0 + 7 * PPM
        edges = [('h', X0 + 6, X7 - 6, Y0, -1, my),
                 ('h', X0 + 6, X7 - 6, Y7, +1, my + 7),
                 ('v', Y0 + 6, Y7 - 6, X0, -1, mx),
                 ('v', Y0 + 6, Y7 - 6, X7, +1, mx + 7)]
        for (axis, a0, a1, expect, od, line_k) in edges:
            coef, std = fit_edge(warp, axis, a0, a1, expect, od)
            if coef is None:
                continue
            w = 1.0 / (1.0 + std * std)
            if axis == 'h':
                for u in np.arange(mx + 0.5, mx + 7.0, 0.5):
                    bx = cubic_design(np.array([u]), np.array([float(line_k)]))
                    xw = OFF + u * PPM + float((bx @ coef_x)[0])
                    anchors.append((u, float(line_k), xw, coef[0] * xw + coef[1], w))
            else:
                for v in np.arange(my + 0.5, my + 7.0, 0.5):
                    by = cubic_design(np.array([float(line_k)]), np.array([v]))
                    yw = OFF + v * PPM + float((by @ coef_y)[0])
                    anchors.append((float(line_k), v, coef[0] * yw + coef[1], yw, w))
    qd = finder_quad(warp, 0, 0)
    if qd:
        for key, (cu, cv) in [('tl', (0, 0)), ('tr', (7, 0)), ('br', (7, 7)), ('bl', (0, 7))]:
            anchors.append((cu, cv, OFF + qd[key][0] * PPM, OFF + qd[key][1] * PPM, 3.0))
    for (a, b), (c, d) in timing_edge_points(warp):
        anchors.append((c, d, OFF + a * PPM, OFF + b * PPM, 1.5))
    return anchors


def pred_x(coef_x, u, v):
    return OFF + u * PPM + float((cubic_design(np.array([u]), np.array([v])) @ coef_x)[0])


def pred_y(coef_y, u, v):
    return OFF + v * PPM + float((cubic_design(np.array([u]), np.array([v])) @ coef_y)[0])


def sample_matrix(warp, coef_x, coef_y):
    """沿形变场取 21×21 模块中心（5×5 中值）→ 全局 Otsu → 0/1 矩阵。"""
    vals = np.zeros((N, N), np.float32)
    for r in range(N):
        for c in range(N):
            xc = pred_x(coef_x, c + 0.5, r + 0.5)
            yc = pred_y(coef_y, c + 0.5, r + 0.5)
            pts = []
            for dy in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                for dx in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                    xi, yi = int(xc + dx), int(yc + dy)
                    if 0 <= xi < S and 0 <= yi < S:
                        pts.append(warp[yi, xi])
            vals[r, c] = np.median(pts) if pts else 128.0
    _, bw = cv2.threshold(np.clip(vals, 0, 255).astype(np.uint8), 0, 255,
                          cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return (bw < 127).astype(np.uint8)


def local_residuals(warp, coef_x, coef_y):
    """逐网格线、逐半模块带，在 Sobel 边缘图上找局部峰值 → 位置修正量。"""
    sm = cv2.GaussianBlur(warp, (0, 0), 1.2)
    gx = np.abs(cv2.Sobel(sm, cv2.CV_32F, 1, 0, ksize=3))
    gy = np.abs(cv2.Sobel(sm, cv2.CV_32F, 0, 1, ksize=3))
    nb = 2 * N
    dvx = np.zeros((N + 1, nb))
    dvy = np.zeros((N + 1, nb))
    for k in range(N + 1):
        for b in range(nb):
            v = (b + 0.5) / 2
            obs = []
            for y in np.arange(OFF + (b / 2) * PPM + 4,
                               OFF + ((b + 1) / 2) * PPM - 3, 3):
                y = int(y)
                if y < 1 or y >= S - 1:
                    continue
                xpred = pred_x(coef_x, k, v)
                lo, hi = int(xpred - 0.35 * PPM), int(xpred + 0.35 * PPM)
                if lo < 1 or hi >= S - 1:
                    continue
                seg = gx[y, lo:hi]
                if len(seg) < 5:
                    continue
                j = int(np.argmax(seg))
                if seg[j] < 120:
                    continue
                a, b2, c2 = seg[j - 1], seg[j], seg[j + 1] if j + 1 < len(seg) else seg[j]
                den = a - 2 * b2 + c2
                dj = 0.5 * (a - c2) / den if den != 0 else 0
                obs.append(lo + j + float(np.clip(dj, -1, 1)) - xpred)
            if len(obs) >= 4:
                med = float(np.median(obs))
                mad = float(np.median(np.abs(np.array(obs) - med)))
                if mad < 0.12 * PPM:
                    dvx[k, b] = med
    for k in range(N + 1):
        for b in range(nb):
            u = (b + 0.5) / 2
            obs = []
            for x in np.arange(OFF + (b / 2) * PPM + 4,
                               OFF + ((b + 1) / 2) * PPM - 3, 3):
                x = int(x)
                if x < 1 or x >= S - 1:
                    continue
                ypred = pred_y(coef_y, u, k)
                lo, hi = int(ypred - 0.35 * PPM), int(ypred + 0.35 * PPM)
                if lo < 1 or hi >= S - 1:
                    continue
                seg = gy[lo:hi, x]
                if len(seg) < 5:
                    continue
                j = int(np.argmax(seg))
                if seg[j] < 120:
                    continue
                a, b2, c2 = seg[j - 1], seg[j], seg[j + 1] if j + 1 < len(seg) else seg[j]
                den = a - 2 * b2 + c2
                dj = 0.5 * (a - c2) / den if den != 0 else 0
                obs.append(lo + j + float(np.clip(dj, -1, 1)) - ypred)
            if len(obs) >= 4:
                med = float(np.median(obs))
                mad = float(np.median(np.abs(np.array(obs) - med)))
                if mad < 0.12 * PPM:
                    dvy[k, b] = med
    return dvx, dvy


def sample_with_field(warp, coef_x, coef_y, dvx, dvy):
    """按（三次场 + 逐线修正）采样，模块中心取相邻网格线中点。"""
    nb = 2 * N
    vals = np.zeros((N, N), np.float32)
    for r in range(N):
        for c in range(N):
            bx = 2 * r + 1
            xc = 0.5 * (pred_x(coef_x, c, r + 1.0) + dvx[c, bx] +
                        pred_x(coef_x, c + 1, r + 1.0) + dvx[c + 1, bx])
            by = 2 * c + 1
            yc = 0.5 * (pred_y(coef_y, c + 1.0, r) + dvy[r, by] +
                        pred_y(coef_y, c + 1.0, r + 1) + dvy[r + 1, by])
            pts = []
            for dy in np.linspace(-0.12 * PPM, 0.12 * PPM, 3):
                for dx in np.linspace(-0.12 * PPM, 0.12 * PPM, 3):
                    xi, yi = int(xc + dx), int(yc + dy)
                    if 0 <= xi < S and 0 <= yi < S:
                        pts.append(warp[yi, xi])
            vals[r, c] = np.median(pts) if pts else 128.0
    _, bw = cv2.threshold(np.clip(vals, 0, 255).astype(np.uint8), 0, 255,
                          cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return (bw < 127).astype(np.uint8)


def structure_score(m):
    """与 V1 的已知结构（定位角/分隔符/时序/暗模块）比对，返回命中率 [0,1]。"""
    dark = np.zeros((N, N), bool)
    light = np.zeros((N, N), bool)

    def fr_(r0, c0):
        for r in range(7):
            for c in range(7):
                dd = max(abs(r - 3), abs(c - 3))
                (dark if dd != 1 else light)[r0 + r, c0 + c] = True

    fr_(0, 0)
    fr_(0, N - 7)
    fr_(N - 7, 0)
    for c in range(8):
        light[7, c] = light[c, 7] = light[7, N - 1 - c]
        light[N - 8, c] = light[N - 1 - c, 7] = True
    for i in range(8, N - 8):
        (dark if i % 2 == 0 else light)[6, i] = True
        (dark if i % 2 == 0 else light)[i, 6] = True
    dark[13, 8] = True
    good = (m.astype(bool) & dark).sum() + (~m.astype(bool) & light).sum()
    return good / (dark.sum() + light.sum())


# ============================================================
# QR 位级原语（内联自 qrdecode.py；只保留候选集模式用得到的）
# ============================================================
def function_map(n=N):
    fm = np.zeros((n, n), bool)
    fm[0:9, 0:9] = True
    fm[0:9, n - 8:] = True
    fm[n - 8:, 0:9] = True
    fm[6, :] = True
    fm[:, 6] = True
    fm[13, 8] = True          # V1 的暗模块
    return fm


# 模块是否参与数据（非功能模块）
_FREE = ~function_map()

# 8 个掩码，预先算成布尔矩阵（原来是每调用一次遍历 21×21 逐点求值）
_MASKS = [
    lambda r, c: (r + c) % 2 == 0,
    lambda r, c: r % 2 == 0,
    lambda r, c: c % 3 == 0,
    lambda r, c: (r + c) % 3 == 0,
    lambda r, c: (r // 2 + c // 3) % 2 == 0,
    lambda r, c: (r * c) % 2 + (r * c) % 3 == 0,
    lambda r, c: ((r * c) % 2 + (r * c) % 3) % 2 == 0,
    lambda r, c: ((r + c) % 2 + (r * c) % 3) % 2 == 0,
]
_RR, _CC = np.mgrid[0:N, 0:N]
_MASK_ARR = [np.array([[mf(int(r), int(c)) for c in range(N)] for r in range(N)], bool)
             for mf in _MASKS]
# 掩码只作用于数据模块
_MASK_FREE = [(_FREE & mk) for mk in _MASK_ARR]

# 之字形读取顺序（行、列索引），在 import 时算一次
def _bit_order():
    rr, cc = [], []
    col = N - 1
    up = True
    while col > 0:
        if col == 6:
            col -= 1
        rows = range(N - 1, -1, -1) if up else range(N)
        for r in rows:
            for c in (col, col - 1):
                if not function_map()[r, c]:
                    rr.append(r)
                    cc.append(c)
        up = not up
        col -= 2
    return np.array(rr), np.array(cc)


_ORDER_R, _ORDER_C = _bit_order()
NBITS = len(_ORDER_R)          # V1 = 208
NCW = NBITS // 8               # V1 = 26 个码字

ECC_INFO = {
    'L': dict(total=26, data=19),
    'M': dict(total=26, data=16),
    'Q': dict(total=26, data=13),
    'H': dict(total=26, data=9),
}

_POW7 = (1 << np.arange(7, -1, -1)).astype(np.uint8)


def codewords_of(base, mi):
    """一个候选朝向/掩码下的 26 个码字（去掩码 → 之字形收集 → 打包成字节）。"""
    un = base.copy()
    un[_MASK_FREE[mi]] ^= 1
    bits = un[_ORDER_R, _ORDER_C].astype(np.uint8)
    return (bits.reshape(NCW, 8) * _POW7).sum(axis=1).astype(np.uint8)


def dih(x):
    """8 个二面体姿态（4 旋转 + 4 镜像旋转）—— 覆盖旋转与镜像。"""
    return ([np.rot90(x, k) for k in range(4)] +
            [np.fliplr(np.rot90(x, k)) for k in range(4)])


# ---------- 纯 Python RS 编码（只为预生成候选码字表）----------
_GF_EXP = [0] * 512
_GF_LOG = [0] * 256
_x = 1
for _i in range(255):
    _GF_EXP[_i] = _x
    _GF_LOG[_x] = _i
    _x <<= 1
    if _x & 0x100:
        _x ^= 0x11D
for _i in range(255, 512):
    _GF_EXP[_i] = _GF_EXP[_i - 255]


def _gf_mul(a, b):
    if a == 0 or b == 0:
        return 0
    return _GF_EXP[_GF_LOG[a] + _GF_LOG[b]]


def _poly_mul(p, q):
    r = [0] * (len(p) + len(q) - 1)
    for i, a in enumerate(p):
        if a == 0:
            continue
        for j, b in enumerate(q):
            if b:
                r[i + j] ^= _gf_mul(a, b)
    return r


def _rs_generator(nsym):
    g = [1]
    for i in range(nsym):
        g = _poly_mul(g, [1, _GF_EXP[i]])
    return g


def rs_encode(data, nsym):
    """QR 标准 RS 系统码：返回 data + nsym 个校验字节（GF(256), 本原多项式 0x11D）。

    ⚠️ 最后那行 `msg[:len(data)] = data` 不能省：合成除法过程中信息位会被写坏，
    reedsolo 也是这么还原的。少了它，预生成的码字表会全错、而症状只是「解不出来」。
    """
    gen = _rs_generator(nsym)
    msg = bytearray(data) + bytearray(nsym)
    for i in range(len(data)):
        coef = msg[i]
        if coef:
            for j in range(1, len(gen)):
                if gen[j]:
                    msg[i + j] ^= _gf_mul(gen[j], coef)
    msg[:len(data)] = data
    return bytes(msg)


# ---------- 候选集 ----------
def parse_known_spec(spec):
    """配候选集。三种写法（与原项目一致）:

        '123'       → 字母表 {1,2,3} × 3 位 = 111..333 共 27 个（赛题就是这个）
        '123:2'     → 字母表 {1,2,3} × 2 位 = 9 个
        '111,133'   → 显式列表（逗号形式，逐项原文使用）

    ⚠️ 注意 `'123'` 里那三个字符是**每一位的取值范围**，不是「内容从 123 开始」——
    所以 `'1234'` 会得到 {1,2,3,4} 的 64 个三位数（111..444），不是「111..234」。
    要精确指定就用显式列表。

    ⚠️ 与原版不同的一点：原版对长度 >3 的候选（如 `0123:4`）会在 `numeric_first_bits`
    里**静默丢弃**，于是候选集变空、悄悄回落到 33s 的通用暴破。这里改成抛 ValueError，
    让配错的候选集当场暴露。
    """
    if spec is None:
        return []
    spec = spec.strip()
    if not spec:
        return []
    if ',' in spec:
        return [s.strip() for s in spec.split(',') if s.strip()]
    if ':' in spec:
        alpha, ln = spec.split(':', 1)
        ln = int(ln)
    else:
        alpha, ln = spec, 3
    alpha = alpha.strip()
    if ln < 1 or ln > 3:
        raise ValueError(
            '候选集长度只能是 1~3 位（数字模式单块最多 3 位），得到长度 %d: %r' % (ln, spec))
    out = []

    def rec(prefix):
        if len(prefix) == ln:
            out.append(prefix)
            return
        for ch in alpha:
            rec(prefix + ch)

    rec('')
    return out


def numeric_first_bits(content):
    """三位以内数字串 → 前 24 位载荷（模式 0001 + 10 位长度 + 10 位数字组）。"""
    if not content.isdigit() or not (1 <= len(content) <= 3):
        return None
    bits = [0, 0, 0, 1]
    ln = len(content)
    bits += [(ln >> (9 - i)) & 1 for i in range(10)]
    grp = int(content)
    bits += [(grp >> (9 - i)) & 1 for i in range(10)]
    return bits


def build_data_stream(first_bits, ndata):
    """按 QR 规范补成完整数据流：4 位终止符 + 补零到字节 + 0xEC/0x11 交替填充。"""
    bits = list(first_bits) + [0, 0, 0, 0]
    if len(bits) % 8:
        bits += [0] * (8 - len(bits) % 8)
    pads = [[1, 1, 1, 0, 1, 1, 0, 0], [0, 0, 0, 1, 0, 0, 0, 1]]
    i = 0
    while len(bits) < ndata * 8:
        bits += pads[i % 2]
        i += 1
    out = []
    for k in range(ndata):
        v = 0
        for b in bits[k * 8:(k + 1) * 8]:
            v = (v << 1) | b
        out.append(v)
    return bytes(out)


def build_candidate_table(contents):
    """候选集 → {ndata: (texts, data_arr, ecc_arr)}，numpy 数组便于向量化比对。

    27 个候选 × 4 档 ECC = 108 次 RS 编码，纯 Python 也就几十毫秒，且只在候选集
    变化时算一次（见 `_table_for`）。
    """
    cands = []
    for ctext in contents:
        fb = numeric_first_bits(ctext)
        if fb is None:
            raise ValueError('候选 %r 不是 1~3 位数字串，数字模式下无法表示' % (ctext,))
        cands.append((ctext, fb))
    if not cands:
        raise ValueError('候选集为空')
    table = {}
    for ndata in (19, 16, 13, 9):
        nsym = 26 - ndata
        texts, datas, eccs = [], [], []
        for ctext, fb in cands:
            data = build_data_stream(fb, ndata)
            full = rs_encode(data, nsym)
            texts.append(ctext)
            datas.append(list(data))
            eccs.append(list(full[ndata:]))
        table[ndata] = (texts,
                        np.array(datas, np.uint8),
                        np.array(eccs, np.uint8))
    return table


_TABLE_CACHE = {}


def _table_for(contents):
    key = tuple(contents)
    t = _TABLE_CACHE.get(key)
    if t is None:
        t = build_candidate_table(contents)
        _TABLE_CACHE.clear()          # 只留一份（候选集本来就只有一套）
        _TABLE_CACHE[key] = t
    return t


def known_set_decode(mat, contents):
    """候选集解码：8 姿态 × 4 档 ECC × 8 掩码，按 ECC 字节精确命中数评分。

    返回 (text, info)。`info` 里带各姿态/ECC 下的最高命中数，供失败留档判断
    「是采样错了还是不在候选集里」。
    """
    table = _table_for(contents)
    best_hits = 0
    best_text = None
    best_md = None
    per_ecc = {}
    for ecc_name, info in ECC_INFO.items():
        ndata = info['data']
        nsym = 26 - ndata
        texts, data_arr, ecc_arr = table[ndata]
        ecc_best = 0
        for base in dih(mat):
            for mi in range(8):
                ucw = codewords_of(base, mi)
                me = (ecc_arr == ucw[ndata:ndata + nsym]).sum(axis=1)
                top = int(me.max())
                if top > ecc_best:
                    ecc_best = top
                if top > best_hits or (top == best_hits and best_text is None):
                    # 同分时用数据字节命中数做 tie-break（与逻辑无关，只求稳定）
                    for idx in np.where(me >= top)[0]:
                        md = int((data_arr[idx] == ucw[:ndata]).sum())
                        if (top > best_hits or best_md is None or md > best_md):
                            best_hits = top
                            best_text = texts[int(idx)]
                            best_md = md
        per_ecc[ecc_name] = ecc_best
    info_out = {
        'hits': best_hits,
        'hits_per_ecc': per_ecc,
        'threshold': 6,
        'matched': best_text if best_hits >= 6 else None,
    }
    # 阈值 6：26 个码字里至少 6 个校验字节逐位相同才算命中
    # （对 27 个候选，误命中概率约 256^-6 * 27 * 256，可忽略）
    if best_hits >= 6:
        return best_text, info_out
    return None, info_out


# ---------- 可选：一般解码（只在装得上 reedsolo 时可用，供离线诊断）----------
def try_rs_decode(mat):
    """对不在候选集里的内容做一般解码。⚠️ 在线链路**不调用**（只用于离线排障）。"""
    try:
        import reedsolo
        from reedsolo import RSCodec
    except Exception:
        return None
    for ori, base in enumerate(dih(mat)):
        for ecc_name, info in ECC_INFO.items():
            nsym = info['total'] - info['data']
            for mi in range(8):
                ucw = codewords_of(base, mi)
                try:
                    dec, full, errata = RSCodec(nsym).decode(bytes(ucw))
                except Exception:
                    continue
                if all(v == 0 for v in dec):
                    continue
                t = _parse_payload(list(dec))
                if t and not t.startswith('<mode'):
                    return (t, ori, ecc_name, mi, len(errata))
    return None


def _parse_payload(cw):
    """数字/字母/字节模式的最小解析（只为离线诊断，够用就行）。"""
    bits = []
    for v in cw:
        bits += [(v >> (7 - i)) & 1 for i in range(8)]
    pos = 0

    def take(n):
        nonlocal pos
        v = 0
        for i in range(n):
            v = (v << 1) | (bits[pos + i] if pos + i < len(bits) else 0)
        pos += n
        return v

    mode = take(4)
    if mode == 0b0001:
        n = take(10)
        s = ''
        while n >= 3:
            s += '%03d' % take(10)
            n -= 3
        if n == 2:
            s += '%02d' % take(7)
        elif n == 1:
            s += '%d' % take(4)
        return s
    if mode == 0b0010:
        n = take(9)
        chars = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:'
        s = ''
        while n >= 2:
            v = take(11)
            s += chars[v // 45] + chars[v % 45]
            n -= 2
        if n == 1:
            s += chars[take(6)]
        return s
    if mode == 0b0100:
        n = take(8)
        bs = bytes(take(8) for _ in range(n))
        try:
            return bs.decode('utf-8')
        except UnicodeDecodeError:
            return bs.decode('latin1')
    return '<mode %04b>' % mode


# ============================================================
# 主入口
# ============================================================
def decode_gray(gray, contents=None, deadline=None, want_artifacts=False):
    """在灰度图上跑一次完整的自研解码。**不抛异常**（超时/失败都从返回值里看）。

    参数
      gray            : HxW uint8 灰度图（BGR 由调用方转好）
      contents        : 候选内容列表，如 111..333 共 27 个；None → 只跑一般解码
      deadline        : time.time() 时间戳，超过就在阶段边界止损
      want_artifacts  : True 时把 warp / 采样矩阵 / 命中数一并返回（失败留档用）

    返回 dict：
      text      成功时的内容，失败为 None
      ok        bool
      stage     卡在哪一层：fast/finder/geometry/sampled/decode/timeout
      reason    一句话原因（给日志）
      score     structure_score；hits 候选集最高命中数；msize 模块像素；est_n 模块数估计
      elapsed   本阶段耗时（秒）
      warp/matrix  仅 want_artifacts 时非 None
    """
    t0 = time.time()
    out = {
        'ok': False, 'text': None, 'stage': 'init', 'reason': '',
        'score': None, 'hits': None, 'hits_per_ecc': None, 'est_n': None,
        'msize': None, 'elapsed': 0.0, 'warp': None, 'matrix': None,
    }

    def _expired():
        return deadline is not None and time.time() > deadline

    def _done(stage, reason):
        out['stage'] = stage
        out['reason'] = reason
        out['elapsed'] = time.time() - t0
        return out

    if gray is None or gray.size == 0:
        return _done('finder', '空图像')

    # ---- 1) 定位角 ----
    found = find_finder_centers(gray)
    if found is None:
        return _done('finder', '没找到三个定位角')
    centers, clusters3 = found
    TL, TR, BL = orient_centers(centers)

    # ---- 版本守卫：从定位角尺寸估模块数 ----
    dmin = min(np.linalg.norm(TR - TL), np.linalg.norm(BL - TL))
    msize = dmin / _FINDER_SPAN
    # 定位角外轮廓 ≈ 49 个模块²（7×7 实心），故 msize ≈ sqrt(area)/7 = w/7
    w_avg = float(np.mean([c[2] / c[3] for c in clusters3]))
    msize_from_area = w_avg / 7.0
    out['msize'] = float(msize)
    if msize_from_area > 0:
        out['est_n'] = dmin / msize_from_area + 7.0
    if out['est_n'] is not None and not (_EST_N_LO <= out['est_n'] <= _EST_N_HI):
        # 不是 V1 就早退：拿 21 的网格去采样 V2+ 只会产出垃圾，而且不报错
        return _done('finder',
                     '不是 version 1(21x21)：估计 %d 个模块' % int(round(out['est_n'])))
    if msize <= 0:
        return _done('finder', '定位角退化（模块尺寸为 0）')

    if _expired():
        return _done('timeout', '定位阶段就超时了')

    # ---- 2) 小码放大（带封顶）----
    if msize < 14:
        k = int(np.ceil(14.0 / msize))
        # ⚠️ 原版这里没有上限：msize≈1px 时 k=14，1920×1080 会被 resize 成 ~400MB
        if k > _MAX_UPSCALE:
            k = _MAX_UPSCALE
        if k > 1:
            gray = cv2.resize(gray, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC)
            TL, TR, BL = TL * k, TR * k, BL * k
            centers = [c * k for c in centers]
            clusters3 = [[c[0] * k, c[1] * k, c[2] * k, c[3], c[4] * k]
                         for c in clusters3]
            msize *= k
            out['msize'] = float(msize)

    T = np.array([[PPM, 0, OFF], [0, PPM, OFF], [0, 0, 1]], np.float64)
    canon = {'TL': [(0, 0), (7, 0), (7, 7), (0, 7)],
             'TR': [(N - 7, 0), (N, 0), (N, 7), (N - 7, 7)],
             'BL': [(0, N - 7), (7, N - 7), (7, N), (0, N)]}
    pos = {'TL': (0, 0), 'TR': (N - 7, 0), 'BL': (0, N - 7)}

    # ---- 3) 初始单应性候选 + 迭代细化 ----
    cl_by_center = []
    for p in [TL, TR, BL]:
        bi = min(range(len(clusters3)),
                 key=lambda i: np.hypot(centers[i][0] - p[0], centers[i][1] - p[1]))
        cl_by_center.append(clusters3[bi])
    cands = []
    Hq = homography_from_quads(cl_by_center, TL, TR, BL)
    if Hq is not None:
        cands.append(T @ Hq)
    cands.append(T @ affine_from_centers(TL, TR, BL))
    cands.append(T @ affine_from_centers(TL, BL, TR))
    Hcur = None
    for Htry in cands:
        warp0 = cv2.warpPerspective(gray, Htry.astype(np.float32), (S, S),
                                    flags=cv2.INTER_CUBIC)
        if all(finder_quad(warp0, *pos[k]) is not None for k in pos):
            Hcur = Htry
            break
    if Hcur is None:
        Hcur = cands[0]

    for _it in range(3):
        if _expired():
            return _done('timeout', '几何细化阶段超时了')
        warp = cv2.warpPerspective(gray, Hcur.astype(np.float32), (S, S),
                                   flags=cv2.INTER_CUBIC)
        src, dst = [], []
        for k in ['TL', 'TR', 'BL']:
            qd = finder_quad(warp, *pos[k])
            if qd is None:
                continue
            for key, ci in [('tl', 0), ('tr', 1), ('br', 2), ('bl', 3)]:
                src.append(list(qd[key]))
                dst.append(list(canon[k][ci]))
        if len(src) < 8:
            return _done('geometry', '定位角边缘拟合点不足')
        Hm, _ = cv2.findHomography(np.array(src, np.float32),
                                   np.array(dst, np.float32),
                                   cv2.RANSAC, 0.3)
        Hcur = (T @ Hm @ np.linalg.inv(T)) @ Hcur
    warp = cv2.warpPerspective(gray, Hcur.astype(np.float32), (S, S),
                               flags=cv2.INTER_CUBIC)

    # ---- 4) 亚像素锚点 + 三次形变场（3 轮）----
    coef_x = np.zeros(10)
    coef_y = np.zeros(10)
    for _rnd in range(3):
        if _expired():
            return _done('timeout', '形变场拟合阶段超时了')
        anchors = collect_anchors(warp, coef_x, coef_y)
        if len(anchors) < 20:
            return _done('geometry', '锚点太少(%d)' % len(anchors))
        au = np.array([a[0] for a in anchors], np.float64)
        av = np.array([a[1] for a in anchors], np.float64)
        ax = np.array([a[2] for a in anchors], np.float64)
        ay = np.array([a[3] for a in anchors], np.float64)
        aw = np.array([a[4] for a in anchors], np.float64)
        A = cubic_design(au, av)
        coef_x = robust_fit(A, ax - (OFF + au * PPM), aw)
        coef_y = robust_fit(A, ay - (OFF + av * PPM), aw)

    # ---- 5) 采样（两种方案取更优），选一个结构分更高的 ----
    mat = sample_matrix(warp, coef_x, coef_y)
    sc = structure_score(mat)
    try:
        dvx, dvy = local_residuals(warp, coef_x, coef_y)
        mat2 = sample_with_field(warp, coef_x, coef_y, dvx, dvy)
        sc2 = structure_score(mat2)
        if sc2 > sc:
            mat, sc = mat2, sc2
    except Exception:
        pass
    out['score'] = float(sc)
    if want_artifacts:
        out['warp'] = warp
        out['matrix'] = mat

    if _expired():
        return _done('timeout', '采样完就超时了')

    # ---- 6) 解码 ----
    if contents:
        text, info = known_set_decode(mat, contents)
        out['hits'] = info['hits']
        out['hits_per_ecc'] = info['hits_per_ecc']
        if text:
            out['ok'] = True
            out['text'] = text
            return _done('ok', '候选集命中')
        # 候选集没中：给离线诊断留一条路（在线链路里这个分支就是「失败」）
        r = try_rs_decode(mat)
        if r:
            out['ok'] = True
            out['text'] = r[0]
            return _done('ok', '一般 RS 解出（不在候选集里）')
        return _done('decode',
                     '候选集未命中（最高命中 %d/%d）' % (info['hits'], 26))
    r = try_rs_decode(mat)
    if r:
        out['ok'] = True
        out['text'] = r[0]
        return _done('ok', '一般 RS 解出')
    return _done('decode', '未解出（没配候选集，且一般解码失败）')


# ============================================================
# 失败留档
# ============================================================
def dump_failure(result, gray=None, frame=None, out_dir=None, keep=20):
    """把一次失败的全部中间产物写进 out_dir（默认 logs/Difficult/<时间戳>/）。

    ⚠️ 自研算法的失败可能卡在四层中的任一层，而 wechat 的失败只有「没检到」一种 ——
    没有这些产物就只能猜。只在失败时调用（成功不留，避免刷 SD 卡）。

    keep: 只保留最近 N 个目录，多了就删最老的（Pi 的 logs/ 是只增不减的）。
    """
    if out_dir is None:
        return None
    try:
        os.makedirs(out_dir, exist_ok=True)
        if frame is not None:
            cv2.imwrite(os.path.join(out_dir, 'frame.jpg'), frame)
        if gray is not None:
            cv2.imwrite(os.path.join(out_dir, 'gray.png'), gray)
        warp = result.get('warp')
        if warp is not None:
            cv2.imwrite(os.path.join(out_dir, 'warp.png'), warp)
        mat = result.get('matrix')
        if mat is not None:
            np.save(os.path.join(out_dir, 'matrix.npy'), mat)
            big = np.where(mat.astype(bool), 0, 255).astype(np.uint8)
            cv2.imwrite(os.path.join(out_dir, 'matrix.png'),
                        cv2.resize(big, (N * 20, N * 20), interpolation=cv2.INTER_NEAREST))
        with open(os.path.join(out_dir, 'hits.txt'), 'w', encoding='utf-8') as f:
            f.write('stage      = %s\n' % result.get('stage'))
            f.write('reason     = %s\n' % result.get('reason'))
            f.write('elapsed    = %.3fs\n' % result.get('elapsed', 0.0))
            f.write('score      = %s\n' % result.get('score'))
            f.write('msize      = %s px\n' % result.get('msize'))
            f.write('est_n      = %s modules\n' % result.get('est_n'))
            f.write('hits       = %s (阈值 6)\n' % result.get('hits'))
            f.write('hits_per_ecc = %s\n' % (result.get('hits_per_ecc'),))
        _prune(out_dir, keep)
        return out_dir
    except Exception:
        return None


def _prune(out_dir, keep):
    """只留最近 keep 个同级目录。"""
    try:
        parent = os.path.dirname(os.path.abspath(out_dir))
        names = sorted(d for d in os.listdir(parent)
                       if os.path.isdir(os.path.join(parent, d)))
        for d in names[:-keep] if len(names) > keep else []:
            import shutil
            shutil.rmtree(os.path.join(parent, d), ignore_errors=True)
    except Exception:
        pass
