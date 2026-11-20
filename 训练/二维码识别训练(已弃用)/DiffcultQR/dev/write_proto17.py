# -*- coding: utf-8 -*-
"""proto17: BR象限双峰性目标优化 + 全矩阵解码"""
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


def quad2(u, v):
    return np.stack([np.ones_like(u), u, v, u * u, u * v, v * v], 1)


def sample_quad(warp, coef_x, coef_y, px, py, r0, r1, c0, c1, frac=0.3):
    vals = {}
    for r in range(r0, r1):
        for c in range(c0, c1):
            u = c + 0.5
            v = r + 0.5
            bx = quad2(np.array([u]), np.array([v]))
            du = float((bx @ px)[0])
            dv = float((bx @ py)[0])
            xc = pred_x(coef_x, u, v) + du
            yc = pred_y(coef_y, u, v) + dv
            pts = []
            h = frac * PPM / 2
            for ddy in np.linspace(-h, h, 3):
                for ddx in np.linspace(-h, h, 3):
                    xi, yi = int(xc + ddx), int(yc + ddy)
                    if 0 <= xi < S and 0 <= yi < S:
                        pts.append(warp[yi, xi])
            vals[(r, c)] = float(np.median(pts)) if pts else 128.0
    return vals


def bimodality(vals_dict):
    arr = np.array(list(vals_dict.values()), np.float64)
    if len(arr) < 8:
        return -1e9
    lo, hi = arr.min(), arr.max()
    if hi - lo < 1e-6:
        return -1e9
    best = -1
    for t in np.linspace(lo + 1e-3, hi - 1e-3, 60):
        a = arr[arr < t]
        b = arr[arr >= t]
        if len(a) < 2 or len(b) < 2:
            continue
        w0, w1 = len(a) / len(arr), len(b) / len(arr)
        score = w0 * w1 * (a.mean() - b.mean()) ** 2
        if score > best:
            best = score
    return best


def nelder_mead(f, x0, step=2.0, maxiter=400):
    n = len(x0)
    sim = [np.array(x0, np.float64)]
    for i in range(n):
        x = np.array(x0, np.float64)
        x[i] += step
        sim.append(x)
    sim = np.array(sim)
    fv = np.array([f(x) for x in sim])
    for it in range(maxiter):
        order = np.argsort(fv)
        sim, fv = sim[order], fv[order]
        if np.max(np.abs(sim[1:] - sim[0])) < 0.05 and np.std(fv) < 1e-3:
            break
        cen = sim[:-1].mean(axis=0)
        xr = cen + (cen - sim[-1])
        fr = f(xr)
        if fv[0] <= fr < fv[-2]:
            sim[-1], fv[-1] = xr, fr
        elif fr < fv[0]:
            xe = cen + 2 * (cen - sim[-1])
            fe = f(xe)
            sim[-1], fv[-1] = (xe, fe) if fe < fr else (xr, fr)
        else:
            xc = cen + 0.5 * (sim[-1] - cen)
            fc = f(xc)
            if fc < fv[-1]:
                sim[-1], fv[-1] = xc, fc
            else:
                sim[1:] = sim[0] + 0.5 * (sim[1:] - sim[0])
                for i in range(1, n + 1):
                    fv[i] = f(sim[i])
    order = np.argsort(fv)
    return sim[order][0], fv[order][0]


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

    gt = np.load('gt.npy')
    def dih(x):
        return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]

    R0, R1, C0, C1 = 8, 21, 12, 21

    def score_pxpy(p12vec):
        px = p12vec[:6]
        py = p12vec[6:]
        vd = sample_quad(warp, coef_x, coef_y, px, py, R0, R1, C0, C1)
        return -bimodality(vd)

    best = None
    rng = np.random.default_rng(11)
    starts = [np.zeros(12)] + [rng.normal(0, 3, 12) for _ in range(3)]
    for s in starts:
        x, fv = nelder_mead(score_pxpy, s, step=3.0, maxiter=350)
        if best is None or fv < best[1]:
            best = (x, fv)
    p12vec = best[0]
    print('BR优化收敛, 双峰得分:', -best[1])
    print('参数:', np.round(p12vec, 2))

    vals = np.zeros((N, N), np.float32)
    for r in range(N):
        for c in range(N):
            xc = pred_x(coef_x, c + 0.5, r + 0.5)
            yc = pred_y(coef_y, c + 0.5, r + 0.5)
            if r >= R0 and c >= C0:
                bx = quad2(np.array([c + 0.5]), np.array([r + 0.5]))
                xc += float((bx @ p12vec[:6])[0])
                yc += float((bx @ p12vec[6:])[0])
            pts = []
            for ddy in np.linspace(-0.12 * PPM, 0.12 * PPM, 5):
                for ddx in np.linspace(-0.12 * PPM, 0.12 * PPM, 5):
                    xi, yi = int(xc + ddx), int(yc + ddy)
                    if 0 <= xi < S and 0 <= yi < S:
                        pts.append(warp[yi, xi])
            vals[r, c] = np.median(pts)
    _, bw = cv2.threshold(np.clip(vals, 0, 255).astype(np.uint8), 0, 255,
                          cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mat = (bw < 127).astype(np.uint8)
    np.save('matrix_bropt.npy', mat)

    bestc = min((int(np.count_nonzero(mat != d)), i) for i, d in enumerate(dih(gt)))
    print('vs 133:', bestc)
    res = try_decode2(mat)
    print('解码:', res[:5] if res else 'fail')
    if res:
        np.save('matrix_win.npy', mat)
        print('*** 成功 ***')


if __name__ == '__main__':
    main()
