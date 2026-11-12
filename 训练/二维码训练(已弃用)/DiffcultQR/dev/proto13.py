# -*- coding: utf-8 -*-
"""proto13: 三次场 + 逐线局部残差修正 + 解码"""
import cv2
import numpy as np

N, PPM, QZ = 21, 40, 6
S = (N + 2 * QZ) * PPM
OFF = QZ * PPM
PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'
ORIG = r'C:\Users\Administrator\Desktop\QRCodePNG\133.png'


def imread_u(path):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


import proto12 as p12

load_warp = p12.load_warp
collect_anchors = p12.collect_anchors
cubic_design = p12.cubic_design
robust_fit = p12.robust_fit
timing_edge_points = p12.NS['timing_edge_points']

from reedsolo import RSCodec, ReedSolomonError
import qrdecode as q


def fit_cubic(warp):
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
    return coef_x, coef_y


def pred_x(coef_x, u, v):
    return OFF + u * PPM + float((cubic_design(np.array([u]), np.array([v])) @ coef_x)[0])


def pred_y(coef_y, u, v):
    return OFF + v * PPM + float((cubic_design(np.array([u]), np.array([v])) @ coef_y)[0])


def local_residuals(warp, coef_x, coef_y):
    """逐线逐带检测修正量, 返回 (dvx[k][band], dvy[k][band])"""
    sm = cv2.GaussianBlur(warp, (0, 0), 1.2)
    gx = np.abs(cv2.Sobel(sm, cv2.CV_32F, 1, 0, ksize=3))
    gy = np.abs(cv2.Sobel(sm, cv2.CV_32F, 0, 1, ksize=3))
    bands = [np.arange(max(0, b - 2), min(N, b + 3)) for b in range(0, N + 1)]
    dvx = np.zeros((N + 1, N + 1))  # [k, band]
    dvy = np.zeros((N + 1, N + 1))
    for k in range(N + 1):
        for bi, br in enumerate(bands):
            obs = []
            for v in br:
                # 该行带中, 沿若干y采样 |gx| 峰
                for y in np.arange(OFF + v * PPM + 4, OFF + (v + 1) * PPM - 4, 4):
                    if y < 2 or y >= S - 2:
                        continue
                    xpred = pred_x(coef_x, k, v + 0.5)
                    lo = int(xpred - 0.35 * PPM)
                    hi = int(xpred + 0.35 * PPM)
                    if lo < 0 or hi >= S:
                        continue
                    seg = gx[int(y), lo:hi]
                    if len(seg) < 5:
                        continue
                    j = int(np.argmax(seg))
                    if seg[j] < 120:
                        continue
                    a, b2, c2 = seg[j - 1], seg[j], seg[j + 1] if j + 1 < len(seg) else seg[j]
                    den = a - 2 * b2 + c2
                    dj = 0.5 * (a - c2) / den if den != 0 else 0
                    dj = float(np.clip(dj, -1, 1))
                    obs.append(lo + j + dj - xpred)
            if len(obs) >= 4:
                med = float(np.median(obs))
                mad = float(np.median(np.abs(np.array(obs) - med)))
                if mad < 0.12 * PPM:
                    dvx[k, bi] = med
    for k in range(N + 1):
        for bi, bc in enumerate(bands):
            obs = []
            for u in bc:
                for x in np.arange(OFF + u * PPM + 4, OFF + (u + 1) * PPM - 4, 4):
                    if x < 2 or x >= S - 2:
                        continue
                    ypred = pred_y(coef_y, u + 0.5, k)
                    lo = int(ypred - 0.35 * PPM)
                    hi = int(ypred + 0.35 * PPM)
                    if lo < 0 or hi >= S:
                        continue
                    seg = gy[lo:hi, int(x)]
                    if len(seg) < 5:
                        continue
                    j = int(np.argmax(seg))
                    if seg[j] < 120:
                        continue
                    a, b2, c2 = seg[j - 1], seg[j + 1] if j + 1 < len(seg) else seg[j], seg[j]
                    den = a - 2 * b2 + c2
                    dj = 0.5 * (a - c2) / den if den != 0 else 0
                    dj = float(np.clip(dj, -1, 1))
                    obs.append(lo + j + dj - ypred)
            if len(obs) >= 4:
                med = float(np.median(obs))
                mad = float(np.median(np.abs(np.array(obs) - med)))
                if mad < 0.12 * PPM:
                    dvy[k, bi] = med
    return dvx, dvy


def sample_with_field(warp, coef_x, coef_y, dvx, dvy, du_extra=0.0, dv_extra=0.0):
    bands_v = np.arange(N + 1, dtype=float)
    vals = np.zeros((N, N), np.float32)
    for r in range(N):
        for c in range(N):
            vrow = r + 1.0
            ucol = c + 1.0
            xs = []
            for k in (c, c + 1):
                base = pred_x(coef_x, k, vrow)
                # 双线性插值 dvx
                bi = min(N, max(0, int(round(vrow))))
                xs.append(base + dvx[k, bi] + (du_extra if k >= c + 1 else 0))
            ys = []
            for k in (r, r + 1):
                base = pred_y(coef_y, ucol, k)
                bi = min(N, max(0, int(round(ucol))))
                ys.append(base + dvy[k, bi] + (dv_extra if k >= r + 1 else 0))
            xc, yc = (xs[0] + xs[1]) / 2, (ys[0] + ys[1]) / 2
            pts = []
            for dy in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                for dx in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                    xi, yi = int(xc + dx), int(yc + dy)
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
    gray, Hp2w = load_warp()
    warp = cv2.warpPerspective(gray, Hp2w.astype(np.float32), (S, S), flags=cv2.INTER_CUBIC)
    coef_x, coef_y = fit_cubic(warp)
    dvx, dvy = local_residuals(warp, coef_x, coef_y)
    print('dvx 非零行数:', (np.abs(dvx) > 0.05 * PPM).sum(), '/', dvx.size)
    print('dvy 非零行数:', (np.abs(dvy) > 0.05 * PPM).sum(), '/', dvy.size)
    np.save('coef_x.npy', coef_x)
    np.save('coef_y.npy', coef_y)
    np.save('dvx.npy', dvx)
    np.save('dvy.npy', dvy)

    vals = sample_with_field(warp, coef_x, coef_y, dvx, dvy)
    _, bw = cv2.threshold(np.clip(vals, 0, 255).astype(np.uint8), 0, 255,
                          cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mat = (bw < 127).astype(np.uint8)
    np.save('matrix_final.npy', mat)

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

    allres = []
    for di, d in enumerate(dih(mat)):
        for r in try_decode2(d):
            allres.append((r[0], di, r[1], r[2], r[3]))
    allres.sort()
    for r in allres[:10]:
        print(f'RS纠{r[0]}个错 姿态{r[1]} ECC={r[2]} mask={r[3]} 内容={r[4]!r}')
    if not allres:
        print('解码失败, 尝试offset搜索...')
        # 对残差区域尝试平移
        bestm, bestsc = None, 999
        for du in np.arange(-0.4, 0.41, 0.1):
            for dv in np.arange(-0.4, 0.41, 0.1):
                vv = sample_with_field(warp, coef_x, coef_y, dvx, dvy, du, dv)
                _, bw2 = cv2.threshold(np.clip(vv, 0, 255).astype(np.uint8), 0, 255,
                                       cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                mm = (bw2 < 127).astype(np.uint8)
                res = []
                for d in dih(mm):
                    res += try_decode2(d)
                if res:
                    res.sort()
                    print(f'offset ({du:.1f},{dv:.1f}) 解码成功:', res[0])
                    return
                sc = min(int(np.count_nonzero(mm != dd)) for dd in dih(gt))
                if sc < bestsc:
                    bestsc, bestm = sc, mm
        print('offset搜索后仍失败, 最佳错误数:', bestsc)


if __name__ == '__main__':
    main()
