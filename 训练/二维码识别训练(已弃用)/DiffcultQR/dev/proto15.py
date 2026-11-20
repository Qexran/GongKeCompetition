# -*- coding: utf-8 -*-
"""proto15: 逐线迭代跟踪网格 + 解码 + 编码假设检验"""
import cv2
import numpy as np

N, PPM, QZ = 21, 40, 6
S = (N + 2 * QZ) * PPM
OFF = QZ * PPM

import proto12 as p12
from reedsolo import RSCodec, ReedSolomonError
import qrdecode as q


def pred_x(coef_x, u, v):
    return OFF + u * PPM + float((p12.cubic_design(np.array([u]), np.array([v])) @ coef_x)[0])


def pred_y(coef_y, u, v):
    return OFF + v * PPM + float((p12.cubic_design(np.array([u]), np.array([v])) @ coef_y)[0])


def trace_lines(warp, coef_x, coef_y, rounds=4, win=0.28):
    """返回 X[k][b], Y[k][b]: 线k在band b(0..2N-1, 半band)的位置"""
    sm = cv2.GaussianBlur(warp, (0, 0), 1.2)
    gx = np.abs(cv2.Sobel(sm, cv2.CV_32F, 1, 0, ksize=3))
    gy = np.abs(cv2.Sobel(sm, cv2.CV_32F, 0, 1, ksize=3))
    nb = 2 * N  # 半band
    X = np.zeros((N + 1, nb))
    Y = np.zeros((N + 1, nb))
    for k in range(N + 1):
        for b in range(nb):
            v = (b + 0.5) / 2
            X[k, b] = pred_x(coef_x, k, v)
            Y[k, b] = pred_y(coef_y, k, v)
    for rnd in range(rounds):
        for k in range(N + 1):
            obs = [None] * nb
            for b in range(nb):
                v = (b + 0.5) / 2
                # band 内的 y 像素范围
                y_lo = int(OFF + (b / 2) * PPM + 3)
                y_hi = int(OFF + ((b + 1) / 2) * PPM - 3)
                best = None
                for y in range(y_lo, y_hi, 2):
                    if y < 1 or y >= S - 1:
                        continue
                    xpred = X[k, b]
                    lo = int(xpred - win * PPM)
                    hi = int(xpred + win * PPM)
                    if lo < 1 or hi >= S - 1:
                        continue
                    seg = gx[y, lo:hi + 1]
                    j = int(np.argmax(seg))
                    if seg[j] < 130:
                        continue
                    a, b2, c2 = seg[j - 1], seg[j], seg[j + 1] if j + 1 < len(seg) else seg[j]
                    den = a - 2 * b2 + c2
                    dj = 0.5 * (a - c2) / den if den != 0 else 0
                    dj = float(np.clip(dj, -1, 1))
                    if best is None or seg[j] > best[1]:
                        best = (lo + j + dj, seg[j])
                if best is not None:
                    obs[b] = best[0]
            # 沿线平滑: 三点中值 -> 均值
            vals = []
            for b in range(nb):
                w = [obs[i] for i in (b - 1, b, b + 1) if 0 <= i < nb and obs[i] is not None]
                if len(w) >= 2:
                    vals.append((b, float(np.median(w))))
                else:
                    vals.append((b, None))
            newb = X[k].copy()
            last = None
            nxt_vals = {}
            # 简单插值填充
            known = {b: v for b, v in vals if v is not None}
            if len(known) >= nb * 0.3:
                ks = sorted(known)
                for b in range(nb):
                    if b in known:
                        # 与预测加权(观测为主)
                        newb[b] = 0.8 * known[b] + 0.2 * X[k, b]
                    else:
                        newb[b] = X[k, b]
            X[k] = newb
        # 水平线
        for k in range(N + 1):
            obs = [None] * nb
            for b in range(nb):
                u = (b + 0.5) / 2
                x_lo = int(OFF + (b / 2) * PPM + 3)
                x_hi = int(OFF + ((b + 1) / 2) * PPM - 3)
                best = None
                for x in range(x_lo, x_hi, 2):
                    if x < 1 or x >= S - 1:
                        continue
                    ypred = Y[k, b]
                    lo = int(ypred - win * PPM)
                    hi = int(ypred + win * PPM)
                    if lo < 1 or hi >= S - 1:
                        continue
                    seg = gy[lo:hi + 1, x]
                    j = int(np.argmax(seg))
                    if seg[j] < 130:
                        continue
                    a, b2, c2 = seg[j - 1], seg[j], seg[j + 1] if j + 1 < len(seg) else seg[j]
                    den = a - 2 * b2 + c2
                    dj = 0.5 * (a - c2) / den if den != 0 else 0
                    dj = float(np.clip(dj, -1, 1))
                    if best is None or seg[j] > best[1]:
                        best = (lo + j + dj, seg[j])
                if best is not None:
                    obs[b] = best[0]
            known = {b: o for b, o in enumerate(obs) if o is not None}
            newb = Y[k].copy()
            if len(known) >= nb * 0.3:
                for b in range(nb):
                    if b in known:
                        newb[b] = 0.8 * known[b] + 0.2 * Y[k, b]
            Y[k] = newb
    return X, Y


def sample_grid(warp, X, Y):
    vals = np.zeros((N, N), np.float32)
    nb = 2 * N
    for r in range(N):
        for c in range(N):
            bx = 2 * r + 1  # band 索引: 模块中心在 band 2r+1
            xc = (X[c, bx] + X[c + 1, bx]) / 2
            by = 2 * c + 1
            yc = (Y[r, by] + Y[r + 1, by]) / 2
            pts = []
            for ddy in np.linspace(-0.12 * PPM, 0.12 * PPM, 5):
                for ddx in np.linspace(-0.12 * PPM, 0.12 * PPM, 5):
                    xi, yi = int(xc + ddx), int(yc + ddy)
                    if 0 <= xi < S and 0 <= yi < S:
                        pts.append(warp[yi, xi])
            vals[r, c] = np.median(pts)
    return vals


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
    from proto12 import collect_anchors, cubic_design, robust_fit
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

    X, Y = trace_lines(warp, coef_x, coef_y, rounds=4)
    np.save('X_lines.npy', X)
    np.save('Y_lines.npy', Y)
    vals = sample_grid(warp, X, Y)
    _, bw = cv2.threshold(np.clip(vals, 0, 255).astype(np.uint8), 0, 255,
                          cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mat = (bw < 127).astype(np.uint8)
    np.save('matrix_trace.npy', mat)

    ideal = np.array([[1, 1, 1, 1, 1, 1, 1], [1, 0, 0, 0, 0, 0, 1], [1, 0, 1, 1, 1, 0, 1],
                      [1, 0, 1, 1, 1, 0, 1], [1, 0, 1, 1, 1, 0, 1], [1, 0, 0, 0, 0, 0, 1],
                      [1, 1, 1, 1, 1, 1, 1]])
    print('TL:', (mat[0:7, 0:7] == ideal).all(), ' TR:', (mat[0:7, 14:21] == ideal).all(),
          ' BL:', (mat[14:21, 0:7] == ideal).all())
    print('timing:', mat[6, 8:13].tolist(), mat[8:13, 6].tolist())
    gt = np.load('gt.npy')
    def dih(x):
        return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]
    best = min((int(np.count_nonzero(mat != d)), i) for i, d in enumerate(dih(gt)))
    print('vs 133:', best)
    res = try_decode2(mat)
    print('解码:', res[:5] if res else 'fail')
    if res:
        np.save('matrix_win.npy', mat)
        print('*** 成功 ***')


if __name__ == '__main__':
    main()
