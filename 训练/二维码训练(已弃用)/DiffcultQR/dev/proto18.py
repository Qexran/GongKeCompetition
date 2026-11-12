# -*- coding: utf-8 -*-
"""proto18: BR象限逐线坐标下降(双峰性目标)"""
import cv2
import numpy as np
import os

N, PPM, QZ = 21, 40, 6
S = (N + 2 * QZ) * PPM
OFF = QZ * PPM
D = os.path.dirname(os.path.abspath(__file__))

import proto12 as p12
from proto12 import collect_anchors, cubic_design, robust_fit
from reedsolo import RSCodec, ReedSolomonError
import qrdecode as q


def pred_x(coef_x, u, v):
    return OFF + u * PPM + float((cubic_design(np.array([u]), np.array([v])) @ coef_x)[0])


def pred_y(coef_y, u, v):
    return OFF + v * PPM + float((cubic_design(np.array([u]), np.array([v])) @ coef_y)[0])


def sample_pos(coef_x, coef_y, sx, sy, r, c):
    """sx: dict k->shift(px), sy: dict k->shift, 权重在 r>=8,c>=12 内渐变"""
    u, v = c + 0.5, r + 0.5
    xc = pred_x(coef_x, u, v)
    yc = pred_y(coef_y, u, v)
    if r >= 8 and c >= 12:
        # x 修正: 相邻两条线的 shift 平均
        k0, k1 = c, c + 1
        w1 = min(1.0, (c - 11) / 4.0)
        dxs = (sx.get(k0, 0.0) + sx.get(k1, 0.0)) / 2 * w1
        xc += dxs
        j0, j1 = r, r + 1
        h1 = min(1.0, (r - 7) / 4.0)
        dys = (sy.get(j0, 0.0) + sy.get(j1, 0.0)) / 2 * h1
        yc += dys
    return xc, yc


def sample_all(warp, coef_x, coef_y, sx, sy, rows=None, cols=None):
    rows = rows or range(N)
    cols = cols or range(N)
    vals = np.zeros((N, N), np.float32)
    for r in rows:
        for c in cols:
            xc, yc = sample_pos(coef_x, coef_y, sx, sy, r, c)
            pts = []
            for ddy in np.linspace(-0.12 * PPM, 0.12 * PPM, 3):
                for ddx in np.linspace(-0.12 * PPM, 0.12 * PPM, 3):
                    xi, yi = int(xc + ddx), int(yc + ddy)
                    if 0 <= xi < S and 0 <= yi < S:
                        pts.append(warp[yi, xi])
            vals[r, c] = np.median(pts) if pts else 128.0
    return vals


def bimod(arr):
    arr = np.asarray(arr, np.float64).ravel()
    if len(arr) < 8 or arr.max() - arr.min() < 1e-6:
        return -1e9
    best = -1
    for t in np.linspace(arr.min() + 1e-3, arr.max() - 1e-3, 50):
        a = arr[arr < t]; b = arr[arr >= t]
        if len(a) < 2 or len(b) < 2:
            continue
        w0, w1 = len(a) / len(arr), len(b) / len(arr)
        sc = w0 * w1 * (a.mean() - b.mean()) ** 2
        if sc > best:
            best = sc
    return best


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

    gt = np.load(os.path.join(D, 'gt.npy'))
    def dih(x):
        return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]

    sx = {}
    sy = {}
    # 坐标下降
    for sweep in range(4):
        for k in range(14, N + 1):  # 垂直线
            best = (None, -1e18)
            for sh in np.arange(-0.7, 0.71, 0.08):
                sx[k] = sh * PPM
                vals = sample_all(warp, coef_x, coef_y, sx, sy,
                                  rows=range(8, N), cols=range(max(12, k - 2), min(N, k + 2)))
                sc = bimod(vals[8:N, max(12, k - 2):min(N, k + 2)])
                if sc > best[1]:
                    best = (sh * PPM, sc)
            sx[k] = best[0]
        for j in range(9, N + 1):  # 水平线
            best = (None, -1e18)
            for sh in np.arange(-0.7, 0.71, 0.08):
                sy[j] = sh * PPM
                vals = sample_all(warp, coef_x, coef_y, sx, sy,
                                  rows=range(max(8, j - 2), min(N, j + 2)), cols=range(12, N))
                sc = bimod(vals[max(8, j - 2):min(N, j + 2), 12:N])
                if sc > best[1]:
                    best = (sh * PPM, sc)
            sy[j] = best[0]
        vals = sample_all(warp, coef_x, coef_y, sx, sy)
        _, bw = cv2.threshold(np.clip(vals, 0, 255).astype(np.uint8), 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        mat = (bw < 127).astype(np.uint8)
        best = min((int(np.count_nonzero(mat != d)), i) for i, d in enumerate(dih(gt)))
        res = try_decode2(mat)
        print(f'sweep{sweep}: vs133={best[0]} 解码={res[:2] if res else "fail"}')
        if res:
            np.save(os.path.join(D, 'matrix_win.npy'), mat)
            print('*** 成功:', res[0], '***')
            return
    np.save(os.path.join(D, 'matrix_cd.npy'), mat)


if __name__ == '__main__':
    main()
