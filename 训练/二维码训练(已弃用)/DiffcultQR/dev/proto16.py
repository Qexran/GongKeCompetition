# -*- coding: utf-8 -*-
"""proto16: BR象限 RANSAC 线提取 -> 重拟合三次场 -> 采样解码"""
import cv2
import numpy as np

N, PPM, QZ = 21, 40, 6
S = (N + 2 * QZ) * PPM
OFF = QZ * PPM

import proto12 as p12
from proto12 import collect_anchors, cubic_design, robust_fit
from reedsolo import RSCodec, ReedSolomonError
import qrdecode as q


def pred_x(coef_x, u, v):
    return OFF + u * PPM + float((cubic_design(np.array([u]), np.array([v])) @ coef_x)[0])


def pred_y(coef_y, u, v):
    return OFF + v * PPM + float((cubic_design(np.array([u]), np.array([v])) @ coef_y)[0])


def ransac_line(points, thr=3.0, iters=60, rng=None):
    """points: [(y, x)] 拟合 x = a*y+b. 返回 (a, b, 内点mask) 或 None"""
    if len(points) < 6:
        return None
    rng = rng or np.random.default_rng(0)
    pts = np.array(points, np.float64)
    best_inliers = None
    best_cnt = 0
    n = len(pts)
    for _ in range(iters):
        i, j = rng.choice(n, 2, replace=False)
        y1, x1 = pts[i]
        y2, x2 = pts[j]
        if abs(y2 - y1) < 20:
            continue
        a = (x2 - x1) / (y2 - y1)
        b = x1 - a * y1
        if abs(a) > 0.2:
            continue
        resid = np.abs(pts[:, 1] - (a * pts[:, 0] + b))
        inl = resid < thr
        cnt = inl.sum()
        if cnt > best_cnt:
            best_cnt = cnt
            best_inliers = (a, b, inl)
    if best_inliers is None or best_cnt < max(6, n * 0.25):
        return None
    a, b, inl = best_inliers
    # 最小二乘精化
    yy, xx = pts[inl, 0], pts[inl, 1]
    A = np.stack([yy, np.ones_like(yy)], 1)
    coef, *_ = np.linalg.lstsq(A, xx, rcond=None)
    return coef[0], coef[1], inl


def extract_br_lines(warp, coef_x, coef_y):
    """返回新增锚点列表"""
    sm = cv2.GaussianBlur(warp, (0, 0), 1.2)
    gx = np.abs(cv2.Sobel(sm, cv2.CV_32F, 1, 0, ksize=3))
    gy = np.abs(cv2.Sobel(sm, cv2.CV_32F, 0, 1, ksize=3))
    rng = np.random.default_rng(3)
    anchors = []
    # 垂直线 u=8..21 在行 8..21
    for k in range(8, N + 1):
        pts = []
        for y in range(int(OFF + 8 * PPM), int(OFF + 21 * PPM), 2):
            xp = pred_x(coef_x, k, (y - OFF) / PPM)
            lo, hi = int(xp - 0.45 * PPM), int(xp + 0.45 * PPM)
            if lo < 1 or hi >= S - 1:
                continue
            seg = gx[y, lo:hi + 1]
            j = int(np.argmax(seg))
            if seg[j] < 140:
                continue
            a, b2, c2 = seg[j - 1], seg[j], seg[j + 1] if j + 1 < len(seg) else seg[j]
            den = a - 2 * b2 + c2
            dj = 0.5 * (a - c2) / den if den != 0 else 0
            pts.append((float(y), lo + j + float(np.clip(dj, -1, 1))))
        res = ransac_line(pts, thr=3.0, rng=rng)
        if res:
            a, b, inl = res
            pts_arr = np.array(pts)
            for (yy, xx) in pts_arr[inl]:
                # 锚点: canonical (k, v) -> (x=xx 观测, y=yy)
                anchors.append((float(k), (yy - OFF) / PPM, float(xx), float(yy), 2.0))
    # 水平线 k=8..21 在列 8..21
    for k in range(8, N + 1):
        pts = []
        for x in range(int(OFF + 8 * PPM), int(OFF + 21 * PPM), 2):
            yp = pred_y(coef_y, (x - OFF) / PPM, k)
            lo, hi = int(yp - 0.45 * PPM), int(yp + 0.45 * PPM)
            if lo < 1 or hi >= S - 1:
                continue
            seg = gy[lo:hi + 1, x]
            j = int(np.argmax(seg))
            if seg[j] < 140:
                continue
            a, b2, c2 = seg[j - 1], seg[j], seg[j + 1] if j + 1 < len(seg) else seg[j]
            den = a - 2 * b2 + c2
            dj = 0.5 * (a - c2) / den if den != 0 else 0
            pts.append((float(x), lo + j + float(np.clip(dj, -1, 1))))
        res = ransac_line(pts, thr=3.0, rng=rng)
        if res:
            a, b, inl = res
            pts_arr = np.array(pts)
            for (xx, yy) in pts_arr[inl]:
                anchors.append(((xx - OFF) / PPM, float(k), float(xx), float(yy), 2.0))
    return anchors


def try_decode2(matrix):
    Nn = matrix.shape[0]
    fm = q.function_map(Nn)
    masks = q.mask_funcs()
    results = []
    for ecc, info in q.ECC_INFO.items():
        nsym = info['total'] - info['data']
        rsc = RSCodec(nsym)
        for mi, mf in enumerate(masks):
            un = matrix.copy()
            for r in range(Nn):
                for c in range(Nn):
                    if not fm[r, c] and mf(r, c):
                        un[r, c] ^= 1
            ucw = q.bits_to_codewords(q.extract_bits(un, Nn))
            if len(ucw) < info['total']:
                continue
            try:
                dec, full, errata = rsc.decode(bytes(ucw))
            except ReedSolomonError:
                continue
            if all(v == 0 for v in dec):
                continue
            try:
                text = q.parse_payload(list(dec))
            except Exception:
                continue
            if text and not text.startswith('<mode'):
                results.append((len(errata), ecc, mi, text))
    results.sort()
    return results


def sample_field(warp, coef_x, coef_y):
    vals = np.zeros((N, N), np.float32)
    for r in range(N):
        for c in range(N):
            xs = [pred_x(coef_x, k, r + 1.0) for k in (c, c + 1)]
            ys = [pred_y(coef_y, c + 1.0, k) for k in (r, r + 1)]
            xc, yc = (xs[0] + xs[1]) / 2, (ys[0] + ys[1]) / 2
            pts = []
            for ddy in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                for ddx in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                    xi, yi = int(xc + ddx), int(yc + ddy)
                    if 0 <= xi < S and 0 <= yi < S:
                        pts.append(warp[yi, xi])
            vals[r, c] = np.median(pts)
    return vals


def main():
    gray, Hp2w = p12.load_warp()
    warp = cv2.warpPerspective(gray, Hp2w.astype(np.float32), (S, S), flags=cv2.INTER_CUBIC)
    coef_x = np.zeros(10)
    coef_y = np.zeros(10)
    for rnd in range(3):
        anchors = collect_anchors(warp, coef_x, coef_y, cubic_design)
        au = np.array([a[0] for a in anchors], np.float64)
        av = np.array([a[1] for a in anchors], np.float64)
        ax = np.array([a[2] for a in anchors], np.float64)
        ay = np.array([a[3] for a in anchors], np.float64)
        aw = np.array([a[4] for a in anchors], np.float64)
        A = cubic_design(au, av)
        coef_x = robust_fit(A, ax - (OFF + au * PPM), aw)
        coef_y = robust_fit(A, ay - (OFF + av * PPM), aw)

    gt = np.load('gt.npy')
    def dih(x):
        return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]

    # 迭代加入 BR 线锚点
    for it in range(3):
        vals = sample_field(warp, coef_x, coef_y)
        _, bw = cv2.threshold(np.clip(vals, 0, 255).astype(np.uint8), 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        mat = (bw < 127).astype(np.uint8)
        best = min((int(np.count_nonzero(mat != d)), i) for i, d in enumerate(dih(gt)))
        res = try_decode2(mat)
        print(f'iter{it}: vs133={best[0]} 解码={res[:3] if res else "fail"}')
        if res:
            np.save('matrix_win.npy', mat)
            print('*** 成功 ***')
            return
        br = extract_br_lines(warp, coef_x, coef_y)
        base = collect_anchors(warp, coef_x, coef_y, cubic_design)
        allA = base + br
        print(f'  BR线锚点: {len(br)}, 总锚点: {len(allA)}')
        au = np.array([a[0] for a in allA], np.float64)
        av = np.array([a[1] for a in allA], np.float64)
        ax = np.array([a[2] for a in allA], np.float64)
        ay = np.array([a[3] for a in allA], np.float64)
        aw = np.array([a[4] for a in allA], np.float64)
        A = cubic_design(au, av)
        coef_x = robust_fit(A, ax - (OFF + au * PPM), aw)
        coef_y = robust_fit(A, ay - (OFF + av * PPM), aw)


if __name__ == '__main__':
    main()
