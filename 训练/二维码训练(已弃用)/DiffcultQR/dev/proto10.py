# -*- coding: utf-8 -*-
"""proto10: 修正版 —— 外向内扫描取第一个下降沿，迭代精化，最终解码"""
import cv2
import numpy as np

PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'
ORIG = r'C:\Users\Administrator\Desktop\QRCodePNG\133.png'
N = 21
PPM = 40
QZ = 6
S = (N + 2 * QZ) * PPM
OFF = QZ * PPM


def imread_u(path):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


def ground_truth():
    g = cv2.cvtColor(imread_u(ORIG), cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(g, 128, 255, cv2.THRESH_BINARY)
    m, q = 20, 4
    mat = np.zeros((N, N), np.uint8)
    for r in range(N):
        for c in range(N):
            cell = bw[(q + r) * m + 6:(q + r + 1) * m - 6, (q + c) * m + 6:(q + c + 1) * m - 6]
            mat[r, c] = 1 if np.mean(cell < 127) > 0.5 else 0
    return mat


def first_fall_from_outside(prof, out_dir, span_unused=None):
    """out_dir=-1: 外侧在数组前部(正向扫描); +1: 外侧在数组尾部(反向扫描)"""
    L = len(prof)
    p = prof.astype(np.float64)
    outer = p[:max(4, L // 3)] if out_dir < 0 else p[-max(4, L // 3):]
    hi_lvl = np.percentile(outer, 80)
    lo_lvl = np.percentile(p, 10)
    if hi_lvl - lo_lvl < 30:
        return None
    mid = (hi_lvl + lo_lvl) / 2
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


def fit_edge(warp, axis, a0, a1, expect_px, out_dir):
    """axis='h': 水平边（沿 x 采样，剖面为 y 方向）；out_dir=-1 外侧在上方(y小)"""
    pos = []
    y_lo = int(max(0, expect_px - 1.5 * PPM))
    y_hi = int(min(warp.shape[0], expect_px + 1.5 * PPM))
    x_lo = int(max(0, expect_px - 1.5 * PPM))
    x_hi = int(min(warp.shape[1], expect_px + 1.5 * PPM))
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
        v = (y_lo if axis == 'h' else x_lo) + e
        pos.append((ti, v))
    if len(pos) < 12:
        return None, 0
    arr = np.array(pos)
    med = np.median(arr[:, 1])
    keep = np.abs(arr[:, 1] - med) < 0.5 * PPM
    arr = arr[keep]
    if len(arr) < 10:
        return None, 0
    A = np.stack([arr[:, 0], np.ones_like(arr[:, 0])], 1)
    coef, *_ = np.linalg.lstsq(A, arr[:, 1], rcond=None)
    resid = arr[:, 1] - A @ coef
    return coef, float(np.std(resid))


def timing_edge_points(warp):
    """利用时序图案(行6/列6)的跳变沿作为单应性约束点。返回 [(warp模块x, warp模块y), ...] """
    pts = []
    # 水平时序: 行6中心 y=6.5, 边沿 x=6..13
    strip = warp[int(OFF + 5.6 * PPM):int(OFF + 7.4 * PPM), :]
    prof = strip.mean(axis=0)
    p = prof.astype(np.float64)
    hi, lo = np.percentile(p, 85), np.percentile(p, 15)
    mid = (hi + lo) / 2
    d = np.diff(p)
    for k in range(6, 14):
        win = (np.arange(len(d)) >= OFF + (k - 0.7) * PPM) & (np.arange(len(d)) < OFF + (k + 0.7) * PPM)
        idx = np.where(win)[0]
        if len(idx) == 0:
            continue
        want_fall = (k % 2 == 0)
        seg = d[idx]
        i = idx[int(np.argmin(seg)) if want_fall else int(np.argmax(seg))]
        jump = d[i]
        if (want_fall and jump > -20) or (not want_fall and jump < 20):
            continue
        a, b = p[i], p[i + 1]
        if b == a:
            continue
        x = (i + (mid - a) / (b - a) - OFF) / PPM
        if abs(x - k) < 0.5:
            pts.append(((x, 6.5), (float(k), 6.5)))
    # 垂直时序: 列6中心 x=6.5
    strip = warp[:, int(OFF + 5.6 * PPM):int(OFF + 7.4 * PPM)]
    prof = strip.mean(axis=1)
    p = prof.astype(np.float64)
    hi, lo = np.percentile(p, 85), np.percentile(p, 15)
    mid = (hi + lo) / 2
    d = np.diff(p)
    for k in range(6, 14):
        win = (np.arange(len(d)) >= OFF + (k - 0.7) * PPM) & (np.arange(len(d)) < OFF + (k + 0.7) * PPM)
        idx = np.where(win)[0]
        if len(idx) == 0:
            continue
        want_fall = (k % 2 == 0)
        seg = d[idx]
        i = idx[int(np.argmin(seg)) if want_fall else int(np.argmax(seg))]
        jump = d[i]
        if (want_fall and jump > -20) or (not want_fall and jump < 20):
            continue
        a, b = p[i], p[i + 1]
        if b == a:
            continue
        y = (i + (mid - a) / (b - a) - OFF) / PPM
        if abs(y - k) < 0.5:
            pts.append(((6.5, y), (6.5, float(k))))
    return pts


def finder_quad(warp, mx, my):
    """finder 左上角模块坐标 (mx,my)，返回4角(模块坐标)"""
    X0, Y0 = OFF + mx * PPM, OFF + my * PPM
    X7, Y7 = X0 + 7 * PPM, Y0 + 7 * PPM
    top = fit_edge(warp, 'h', X0 + 8, X7 - 8, Y0, -1)
    bot = fit_edge(warp, 'h', X0 + 8, X7 - 8, Y7, +1)
    left = fit_edge(warp, 'v', Y0 + 8, Y7 - 8, X0, -1)
    right = fit_edge(warp, 'v', Y0 + 8, Y7 - 8, X7, +1)
    edges = [top, bot, left, right]
    if any(e[0] is None for e in edges):
        return None
    def inter(hl, vl):
        ah, bh = hl
        av, bv = vl
        y = (ah * bv + bh) / (1 - ah * av + 1e-9)
        x = av * y + bv
        return np.array([(x - OFF) / PPM, (y - OFF) / PPM])
    q = dict(tl=inter(top[0], left[0]), tr=inter(top[0], right[0]),
             br=inter(bot[0], right[0]), bl=inter(bot[0], left[0]))
    return q, [round(e[1], 2) for e in edges]


def sample_matrix(gray, Hp2m, K=5, frac=0.5):
    Hinv = np.linalg.inv(Hp2m)
    xs = ((np.arange(K) / (K - 1) - 0.5) * frac + 0.5)
    rows, cols = np.meshgrid(np.arange(N), np.arange(N), indexing='ij')
    rr, cc = rows.ravel(), cols.ravel()
    Uf = np.tile(cc[:, None] + xs[None, :], (1, K)).ravel()
    Vf = np.repeat(rr[:, None] + xs[None, :], K, axis=1).ravel()
    pts = np.stack([Uf, Vf, np.ones_like(Uf)], 0)
    imgp = Hinv @ pts
    imgp = imgp[:2] / imgp[2:3]
    n = len(rr)
    v = cv2.remap(gray, imgp[0].astype(np.float32).reshape(n, K * K),
                  imgp[1].astype(np.float32).reshape(n, K * K), cv2.INTER_LINEAR).mean(axis=1)
    return v.reshape(N, N)


def structure_score(m):
    dark = np.zeros((N, N), bool)
    light = np.zeros((N, N), bool)

    def fr(r0, c0):
        for r in range(7):
            for c in range(7):
                d = max(abs(r - 3), abs(c - 3))
                (dark if d != 1 else light)[r0 + r, c0 + c] = True
    fr(0, 0); fr(0, N - 7); fr(N - 7, 0)
    for c in range(8):
        light[7, c] = light[c, 7] = light[7, N - 1 - c] = light[c, N - 8] = light[N - 8, c] = light[N - 1 - c, 7] = True
    for i in range(8, N - 8):
        (dark if i % 2 == 0 else light)[6, i] = True
        (dark if i % 2 == 0 else light)[i, 6] = True
    dark[13, 8] = True
    good = (m.astype(bool) & dark).sum() + (~m.astype(bool) & light).sum()
    return good / (dark.sum() + light.sum())


def main():
    img = imread_u(PHOTO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    TLc = np.array([582.07, 507.66])
    TRc = np.array([713.56, 678.29])
    BLc = np.array([839.12, 289.30])
    ex = (TRc - TLc) / (N - 7)
    ey = (BLc - TLc) / (N - 7)
    A = np.array([[ex[0], ey[0]], [ex[1], ey[1]]], np.float64)
    t = TLc - 3.5 * ex - 3.5 * ey
    Ainv = np.linalg.inv(A)
    H0 = np.array([[Ainv[0, 0], Ainv[0, 1], -(Ainv @ t)[0]],
                   [Ainv[1, 0], Ainv[1, 1], -(Ainv @ t)[1]],
                   [0, 0, 1]], np.float64)
    T = np.array([[PPM, 0, OFF], [0, PPM, OFF], [0, 0, 1]], np.float64)
    Hcur = T @ H0  # photo -> warp px

    canon = {
        'TL': [(0, 0), (7, 0), (7, 7), (0, 7)],
        'TR': [(N - 7, 0), (N, 0), (N, 7), (N - 7, 7)],
        'BL': [(0, N - 7), (7, N - 7), (7, N), (0, N)],
    }
    pos = {'TL': (0, 0), 'TR': (N - 7, 0), 'BL': (0, N - 7)}

    for it in range(4):
        warp = cv2.warpPerspective(gray, Hcur.astype(np.float32), (S, S), flags=cv2.INTER_CUBIC)
        src, dst = [], []
        for k in ['TL', 'TR', 'BL']:
            mx, my = pos[k]
            res = finder_quad(warp, mx, my)
            if res is None:
                print(f'it{it} {k} FAILED')
                continue
            q, stds = res
            for key, ci in [('tl', 0), ('tr', 1), ('br', 2), ('bl', 3)]:
                src.append(list(q[key]))
                dst.append(list(canon[k][ci]))
            print(f'it{it} {k} quad(mod):', np.round([q['tl'], q['tr'], q['br'], q['bl']], 2).tolist(),
                  'stds:', stds)
        tp = timing_edge_points(warp)
        for (a, b), (c, d) in tp:
            src.append([a, b])
            dst.append([c, d])
        print(f'it{it} timing约束点: {len(tp)}')
        Hm, _ = cv2.findHomography(np.array(src, np.float32), np.array(dst, np.float32), cv2.RANSAC, 0.3)
        Hcur = (T @ Hm.astype(np.float64) @ np.linalg.inv(T)) @ Hcur

    Hp2m = np.linalg.inv(T) @ Hcur  # photo -> module
    np.save('H10.npy', Hp2m)
    # --- 局部网格混合: 消除镜头畸变残差 ---
    # 用最后一次迭代检测的三个 finder quad 构建 4 个局部仿射
    warp = cv2.warpPerspective(gray, Hcur.astype(np.float32), (S, S), flags=cv2.INTER_CUBIC)
    quads = {}
    for k in ['TL', 'TR', 'BL']:
        mx, my = pos[k]
        res = finder_quad(warp, mx, my)
        quads[k] = res[0] if res else None
    if all(q is not None for q in quads.values()):
        def local_affine(q, canon4):
            # 规范坐标 -> 测量坐标 (供采样使用)
            src3 = np.array([canon4[0], canon4[1], canon4[3]], np.float64)
            dst3 = np.array([q['tl'], q['tr'], q['bl']], np.float64)
            A = np.stack([src3[:, 0], src3[:, 1], np.ones(3)], 1)
            uc, *_ = np.linalg.lstsq(A, dst3[:, 0], rcond=None)
            vc, *_ = np.linalg.lstsq(A, dst3[:, 1], rcond=None)
            return np.concatenate([uc, vc])  # x = c0*u+c1*v+c2; y = c3*u+c4*v+c5

        aff = {}
        for k in ['TL', 'TR', 'BL']:
            aff[k] = local_affine(quads[k], canon[k])
        # BR: 平行四边形外推 G_BR = G_TR + G_BL - G_TL
        cTL, cTR, cBL = aff['TL'], aff['TR'], aff['BL']
        cBR = cTR + cBL - cTL
        aff['BR'] = cBR

        us = np.arange(N) + 0.5
        vs = np.arange(N) + 0.5
        UU, VV = np.meshgrid(us, vs)
        s = UU / N
        t = VV / N
        def g(coef):
            xu = coef[0] * UU + coef[1] * VV + coef[2]
            yu = coef[3] * UU + coef[4] * VV + coef[5]
            return xu, yu
        xtl, ytl = g(cTL); xtr, ytr = g(cTR); xbl, ybl = g(cBL); xbr, ybr = g(cBR)
        Xu = (1-s)*(1-t)*xtl + s*(1-t)*xtr + (1-s)*t*xbl + s*t*xbr
        Yu = (1-s)*(1-t)*ytl + s*(1-t)*ytr + (1-s)*t*ybl + s*t*ybr
        # 在 warp 图上直接采样（warp 像素 = (module+QZ)*PPM）
        wx = ((Xu + QZ) * PPM).astype(np.float32)
        wy = ((Yu + QZ) * PPM).astype(np.float32)
        vals = cv2.remap(warp, wx, wy, cv2.INTER_LINEAR)
        med = vals
        print('局部混合网格采样完成')
    else:
        med = sample_matrix(gray, Hp2m)
    med_u8 = np.clip(med, 0, 255).astype(np.uint8)
    cv2.imwrite('med10.png', med_u8)
    _, bw = cv2.threshold(med_u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    m = (bw < 127).astype(np.uint8)
    np.save('matrix10.npy', m)
    print('结构得分:', round(structure_score(m), 3))
    print('timing row:', m[6, 8:13].tolist(), ' col:', m[8:13, 6].tolist())
    print('TL:'); print(m[0:7, 0:7])
    print('TR:'); print(m[0:7, 14:21])
    print('BL:'); print(m[14:21, 0:7])

    gt = ground_truth()
    def dih(x):
        return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]
    best = min(((int(np.count_nonzero(m != d)), i) for i, d in enumerate(dih(gt))))
    print('vs 133.png: 错误', best[0], '/441 姿态', best[1])


if __name__ == '__main__':
    main()
