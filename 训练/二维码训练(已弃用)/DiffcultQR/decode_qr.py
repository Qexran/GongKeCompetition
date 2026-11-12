# -*- coding: utf-8 -*-
"""
decode_qr.py - 高难度二维码照片解码器(屏幕拍摄/强透视/摩尔纹场景)

管线:
  A. 快速通道: zxing-cpp / pyzbar / OpenCV 直接解码
  B. 几何重建: finder轮廊检测 -> 仿射warp -> 亚像素边缘锚点 -> 三次多项式形变场
  C. 模块采样: 网格中心中值采样 -> Otsu -> 21x21 模块矩阵
  D. 鲁棒解码: 8姿态 x 4 ECC x 8掩码 RS直接解码;
               失败则进入短载荷暴力补全(候选前缀+标准填充, 用完好ECC字节验证)

用法: python decode_qr.py <图片路径> [--debug]
依赖: opencv-python numpy reedsolo  (可选增强: zxing-cpp pyzbar)
"""
import sys
import cv2
import numpy as np

N = 21
PPM = 40
QZ = 6
S = (N + 2 * QZ) * PPM
OFF = QZ * PPM


def imread_u(path):
    img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise IOError('cannot read: ' + path)
    return img


def fast_decode(gray):
    try:
        import zxingcpp
        for kw in [dict(try_harder=True, try_rotate=True),
                   dict(try_harder=True, try_rotate=True, try_downscale=True)]:
            try:
                res = zxingcpp.read_barcodes(gray, **kw)
            except Exception:
                res = []
            if res:
                return res[0].text
    except ImportError:
        pass
    try:
        from pyzbar import pyzbar
        r = pyzbar.decode(gray)
        if r:
            return r[0].data.decode('utf-8', 'replace')
    except ImportError:
        pass
    s = cv2.QRCodeDetector().detectAndDecode(gray)[0]
    return s if s else None


def find_finder_centers(gray):
    found = []
    for blk in [51, 75, 101]:
        for C in [5, 10, 15]:
            bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY, blk, C)
            for pol in [0, 1]:
                b = bw if pol == 0 else 255 - bw
                cnts, hier = cv2.findContours(b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
    clusters.sort(key=lambda c: -c[2] / c[3])  # 按平均尺寸(真finder最大)
    centers = [np.array([c[0] / c[3], c[1] / c[3]]) for c in clusters]
    if len(centers) < 3:
        return None
    return centers[:3], clusters[:3]


def orient_centers(centers):
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
    """外轮廓 -> 4角点(凸包+逼近)"""
    hull = cv2.convexHull(cnt)
    peri = cv2.arcLength(hull, True)
    for eps in [0.02, 0.03, 0.05, 0.08, 0.12]:
        ap = cv2.approxPolyDP(hull, eps * peri, True)
        if len(ap) == 4:
            return ap.reshape(4, 2).astype(np.float64)
    return None


def order_quad(quad, center, xhat, yhat):
    """按QR自身坐标系(xhat=TL->TR, yhat=TL->BL)排序4角点为 tl,tr,br,bl"""
    rel = quad - center
    a = rel @ xhat   # 沿x的坐标
    b = rel @ yhat   # 沿y的坐标
    s = a + b
    d = a - b
    tl = quad[int(np.argmin(s))]
    br = quad[int(np.argmax(s))]
    tr = quad[int(np.argmax(d))]
    bl = quad[int(np.argmin(d))]
    return np.array([tl, tr, br, bl])


def homography_from_quads(clusters3, TL, TR, BL):
    """三个finder外四边形 -> 精确初始单应性(module坐标). 失败返回None"""
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
    ex = (TR - TL) / (N - 7)
    ey = (BL - TL) / (N - 7)
    A = np.array([[ex[0], ey[0]], [ex[1], ey[1]]])
    t = TL - 3.5 * ex - 3.5 * ey
    Ainv = np.linalg.inv(A)
    return np.array([[Ainv[0, 0], Ainv[0, 1], -(Ainv @ t)[0]],
                     [Ainv[1, 0], Ainv[1, 1], -(Ainv @ t)[1]],
                     [0, 0, 1]])

# ---------- B2: subpixel edges ----------

def first_fall_from_outside(prof, out_dir):
    L = len(prof)
    p = prof.astype(np.float64)
    outer = p[:max(4, L // 3)] if out_dir < 0 else p[-max(4, L // 3):]
    hi = np.percentile(outer, 80)
    lo = np.percentile(p, 10)
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
    pts = []
    strip = warp[int(OFF + 5.6 * PPM):int(OFF + 7.4 * PPM), :]
    p = strip.mean(axis=0).astype(np.float64)
    hi, lo = np.percentile(p, 85), np.percentile(p, 15)
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
    hi, lo = np.percentile(p, 85), np.percentile(p, 15)
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
    return np.stack([np.ones_like(u), u, v, u * u, u * v, v * v,
                     u ** 3, u * u * v, u * v * v, v ** 3], 1)


def robust_fit(Amat, b, w0, iters=5):
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
    """逐线逐带检测网格位置修正(紧窗口+中值+MAD一致性)"""
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
    """按(三次场+逐线修正)网格采样, 模块中心取相邻线中点"""
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

# ---------- D: decoding ----------

import qrdecode as q
from reedsolo import RSCodec, ReedSolomonError


def bit_positions():
    fm = q.function_map(N)
    bits = []
    col = N - 1
    up = True
    while col > 0:
        if col == 6:
            col -= 1
        c1, c0 = col, col - 1
        rows = range(N - 1, -1, -1) if up else range(N)
        for r in rows:
            for c in (c1, c0):
                if not fm[r, c]:
                    bits.append((r, c))
        up = not up
        col -= 2
    return bits

BITS = bit_positions()


def dih(x):
    return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]


def dih_pairs(x, m):
    return ([(np.rot90(x, k), np.rot90(m, k)) for k in range(4)] +
            [(np.fliplr(np.rot90(x, k)), np.fliplr(np.rot90(m, k))) for k in range(4)])


def codewords_of(base, mi):
    fm = q.function_map(N)
    mf = q.mask_funcs()[mi]
    un = base.copy()
    for r in range(N):
        for c in range(N):
            if not fm[r, c] and mf(r, c):
                un[r, c] ^= 1
    return np.array(q.bits_to_codewords(q.extract_bits(un, N)))


def try_rs_decode(mat):
    for ori, base in enumerate(dih(mat)):
        for ecc_name, info in q.ECC_INFO.items():
            nsym = info['total'] - info['data']
            for mi in range(8):
                ucw = codewords_of(base, mi)
                try:
                    dec, full, errata = RSCodec(nsym).decode(bytes(ucw))
                except Exception:
                    continue
                if all(v == 0 for v in dec):
                    continue
                try:
                    t = q.parse_payload(list(dec))
                except Exception:
                    continue
                if t and not t.startswith('<mode'):
                    return (t, ori, ecc_name, mi, len(errata))
    return None


def build_data_stream(first_bits, ndata):
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


def candidates24():
    yield [0] * 24
    for ln in range(1, 35):
        for g in range(1024):
            yield [0, 0, 0, 1] + [(ln >> (9 - i)) & 1 for i in range(10)] + \
                  [(g >> (9 - i)) & 1 for i in range(10)]
    for ln in range(1, 26):
        for g in range(2048):
            yield [0, 0, 1, 0] + [(ln >> (8 - i)) & 1 for i in range(9)] + \
                  [(g >> (10 - i)) & 1 for i in range(11)]
    for ln in range(1, 18):
        for g in range(4096):
            yield [0, 1, 0, 0] + [(ln >> (7 - i)) & 1 for i in range(8)] + \
                  [(g >> (11 - i)) & 1 for i in range(12)]


def try_brute_complete(mat, damage_own=None):
    """无损坏模型版: 预计算候选RS编码, 按ECC字节精确匹配数评分"""
    from collections import defaultdict
    # 预计算: 各 ndata 下的候选编码矩阵
    pre = {}
    for ndata in [19, 16, 13, 9]:
        nsym = 26 - ndata
        rsc = RSCodec(nsym)
        datas = []
        encs = []
        for cand in candidates24():
            data = build_data_stream(cand, ndata)
            datas.append(data)
            encs.append(bytes(rsc.encode(data))[ndata:])
        pre[ndata] = (datas, np.frombuffer(b''.join(encs), np.uint8)
                      .reshape(len(datas), nsym))

    best_hits = defaultdict(set)
    for ori, base in enumerate(dih(mat)):
        for ecc_name, info in q.ECC_INFO.items():
            ndata = info['data']
            nsym = 26 - ndata
            datas, encm = pre[ndata]
            for mi in range(8):
                ucw = codewords_of(base, mi)
                obs = ucw[ndata:ndata + nsym]
                match = (encm == obs[None, :]).sum(axis=1)
                mx = int(match.max())
                if mx >= max(6, nsym - 3):
                    for idx in np.where(match >= mx)[0]:
                        try:
                            t = q.parse_payload(list(datas[idx]))
                        except Exception:
                            continue
                        if t and not t.startswith('<mode'):
                            best_hits[(mx, ecc_name)].add(t)
    if not best_hits:
        return []
    top = max(k[0] for k in best_hits)
    out = set()
    for (mx, ecc), ts in best_hits.items():
        if mx >= top - 1:
            out |= ts
    return sorted(out)


def parse_known_spec(spec):
    """'123' -> 111..333 共27个; '123:2' -> 两位; '111,133,222' -> 显式列表"""
    if ',' in spec:
        return [s.strip() for s in spec.split(',') if s.strip()]
    if ':' in spec:
        alpha, ln = spec.split(':', 1)
        ln = int(ln)
    else:
        alpha, ln = spec, 3
    alpha = alpha.strip()
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
    """数字模式载荷 -> 前24位 (mode+len+数字组)"""
    if not content.isdigit() or not (1 <= len(content) <= 3):
        return None
    bits = [0, 0, 0, 1]
    ln = len(content)
    bits += [(ln >> (9 - i)) & 1 for i in range(10)]
    grp = int(content)
    bits += [(grp >> (9 - i)) & 1 for i in range(10)]
    return bits


def known_set_decode(mat, contents):
    """已知候选集解码: 候选数少, 直接用ECC字节精确匹配评分"""
    cands = []
    for ctext in contents:
        fb = numeric_first_bits(ctext)
        if fb is None:
            continue
        cands.append((ctext, fb))
    if not cands:
        return None
    pre = {}
    for ndata in [19, 16, 13, 9]:
        nsym = 26 - ndata
        rsc = RSCodec(nsym)
        rows = []
        for ctext, fb in cands:
            data = build_data_stream(fb, ndata)
            rows.append((ctext, data, bytes(rsc.encode(data))[ndata:]))
        pre[ndata] = rows
    best = (0, None)
    for ori, base in enumerate(dih(mat)):
        for ecc_name, info in q.ECC_INFO.items():
            ndata = info['data']
            nsym = 26 - ndata
            for mi in range(8):
                ucw = codewords_of(base, mi)
                for (ctext, data, ecc) in pre[ndata]:
                    me = sum(1 for j in range(nsym) if ecc[j] == ucw[ndata + j])
                    md = sum(1 for j in range(ndata) if data[j] == ucw[j])
                    if me > best[0] or (me == best[0] and md < best[2] if len(best) > 2 else False):
                        best = (me, ctext, md)
    if best[0] >= 6:
        return best[1]
    return None


# ---------- main ----------

def run():
    debug = '--debug' in sys.argv
    known_contents = None
    pos_args = []
    argv = sys.argv[1:]
    i = 0
    while i < len(argv):
        if argv[i] == '--known' and i + 1 < len(argv):
            known_contents = parse_known_spec(argv[i + 1])
            i += 2
            continue
        if argv[i] and not argv[i].startswith('--'):
            pos_args.append(argv[i])
        i += 1
    if not pos_args:
        print(__doc__)
        sys.exit(1)
    path = pos_args[0]
    img = imread_u(path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    print('[1/4] fast path (zxing/pyzbar/OpenCV)...')
    r = fast_decode(gray)
    if r:
        print('DECODED: %r' % r)
        return

    print('[2/4] finder detection + perspective warp...')
    result = find_finder_centers(gray)
    if result is None:
        print('FAIL: three finder patterns not found')
        sys.exit(2)
    centers, clusters3 = result
    TL, TR, BL = orient_centers(centers)
    # 小码自动放大: 较短边模块 < 8px 时放大, 保障亚像素边缘检测
    dmin = min(np.linalg.norm(TR - TL), np.linalg.norm(BL - TL))
    msize = dmin / (N - 7)
    if msize < 14:
        k = int(np.ceil(14 / msize))
        print('  module ~%.1fpx, upscaling x%d' % (msize, k))
        gray = cv2.resize(gray, None, fx=k, fy=k, interpolation=cv2.INTER_CUBIC)
        TL, TR, BL = TL * k, TR * k, BL * k
        centers = [c * k for c in centers]
        clusters3 = [[c[0] * k, c[1] * k, c[2] * k, c[3], c[4] * k]
                     for c in clusters3]
    T = np.array([[PPM, 0, OFF], [0, PPM, OFF], [0, 0, 1]], np.float64)

    canon = {'TL': [(0, 0), (7, 0), (7, 7), (0, 7)],
             'TR': [(N - 7, 0), (N, 0), (N, 7), (N - 7, 7)],
             'BL': [(0, N - 7), (7, N - 7), (7, N), (0, N)]}
    pos = {'TL': (0, 0), 'TR': (N - 7, 0), 'BL': (0, N - 7)}

    print('[3/4] subpixel edge anchors + cubic deformation field...')
    # 初始H候选: 轮廓四边形单应性(极端透视更准) + 两种仿射定向
    cl_by_center = []
    for p in [TL, TR, BL]:
        bi = min(range(len(clusters3)),
                 key=lambda i: np.hypot(centers[i][0] - p[0],
                                        centers[i][1] - p[1]))
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

    for it in range(3):
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
            print('FAIL: finder edge fitting insufficient')
            sys.exit(3)
        Hm, _ = cv2.findHomography(np.array(src, np.float32),
                                   np.array(dst, np.float32),
                                   cv2.RANSAC, 0.3)
        Hcur = (T @ Hm @ np.linalg.inv(T)) @ Hcur
    warp = cv2.warpPerspective(gray, Hcur.astype(np.float32), (S, S),
                               flags=cv2.INTER_CUBIC)

    coef_x = np.zeros(10)
    coef_y = np.zeros(10)
    for rnd in range(3):
        anchors = collect_anchors(warp, coef_x, coef_y)
        au = np.array([a[0] for a in anchors], np.float64)
        av = np.array([a[1] for a in anchors], np.float64)
        ax = np.array([a[2] for a in anchors], np.float64)
        ay = np.array([a[3] for a in anchors], np.float64)
        aw = np.array([a[4] for a in anchors], np.float64)
        A = cubic_design(au, av)
        coef_x = robust_fit(A, ax - (OFF + au * PPM), aw)
        coef_y = robust_fit(A, ay - (OFF + av * PPM), aw)
    if debug:
        cv2.imwrite('debug_warp.png', warp)

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
    print('  module matrix structure score: %.3f' % sc)
    if debug:
        np.save('debug_matrix.npy', mat)

    print('[4/4] RS decoding...')
    r = try_rs_decode(mat)
    if r:
        t, ori, ecc, mi, ne = r
        print('DECODED: %r  (orientation %d, ECC=%s, mask=%d, RS fixed %d codewords)'
              % (t, ori, ecc, mi, ne))
        return

    if known_contents:
        print('  trying known candidate set (%d candidates)...' % len(known_contents))
        t0 = __import__('time').time()
        kt = known_set_decode(mat, known_contents)
        if kt:
            print('DECODED (known set): %r  [%.2fs]' % (kt, __import__('time').time() - t0))
            return
        print('  known set failed, falling back to generic brute...')
    print('  direct RS failed, trying short-payload brute completion...')
    ac = np.array([[3.5, 3.5], [N - 3.5, 3.5], [3.5, N - 3.5]])
    damage = np.zeros((N, N), bool)
    for r_ in range(N):
        for c_ in range(N):
            d = min(np.hypot(c_ + 0.5 - a[0], r_ + 0.5 - a[1]) for a in ac)
            damage[r_, c_] = d > 6.5
    res = try_brute_complete(mat, damage)
    if res:
        for t in res:
            print('DECODED (brute completion): %r' % t)
        return
    print('FAIL: all methods exhausted. use --debug for artifacts.')
    sys.exit(4)


def main():
    import time
    t0 = time.time()
    try:
        run()
    except SystemExit:
        print('[耗时] 解码管线 %.2f 秒' % (time.time() - t0))
        raise
    print('[耗时] 解码管线 %.2f 秒' % (time.time() - t0))


if __name__ == '__main__':
    main()
