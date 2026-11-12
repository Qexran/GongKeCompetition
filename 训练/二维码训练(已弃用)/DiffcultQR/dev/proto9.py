# -*- coding: utf-8 -*-
"""原型9：带静区 warp + 两轮边缘精化"""
import cv2
import numpy as np

PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'
ORIG = r'C:\Users\Administrator\Desktop\QRCodePNG\133.png'
N = 21
PPM = 40
QZ = 6  # warp 含静区模块数（两侧）
S = (N + 2 * QZ) * PPM


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


def subpixel_fall(prof, expect, win=1.6):
    lo = max(0, int(expect - win * PPM))
    hi = min(len(prof) - 1, int(expect + win * PPM))
    if hi - lo < 5:
        return None
    p = prof[lo:hi + 1].astype(np.float64)
    d = np.diff(p)
    i = int(np.argmin(d))
    if d[i] > -20:
        return None
    a, b = p[i], p[i + 1]
    top = np.percentile(p, 90)
    bot = np.percentile(p, 10)
    if top - bot < 40:
        return None
    mid = (top + bot) / 2
    if b == a:
        return None
    x = lo + i + (mid - a) / (b - a)
    if abs(x - expect) > win * PPM:
        return None
    return x


def fit_edge(warp, axis, c0, c1, expect):
    pos = []
    for c in range(int(c0), int(c1), 2):
        prof = warp[:, c] if axis == 'h' else warp[c, :]
        e = subpixel_fall(prof, expect)
        if e is not None:
            pos.append((c, e))
    if len(pos) < 12:
        return None, None
    arr = np.array(pos)
    med = np.median(arr[:, 1])
    keep = np.abs(arr[:, 1] - med) < 0.5 * PPM
    arr = arr[keep]
    if len(arr) < 10:
        return None, None
    A = np.stack([arr[:, 0], np.ones_like(arr[:, 0])], 1)
    coef, *_ = np.linalg.lstsq(A, arr[:, 1], rcond=None)
    resid = arr[:, 1] - A @ coef
    return coef, float(np.std(resid))


def corners_from_edges(warp, x0, y0, shift=QZ * PPM):
    """x0,y0: finder 左上角模块坐标(可为浮点), 返回4角点(warp像素)"""
    X0, Y0 = (x0 + QZ) * PPM, (y0 + QZ) * PPM
    X7, Y7 = X0 + 7 * PPM, Y0 + 7 * PPM
    top, st_t = fit_edge(warp, 'h', X0 + 10, X7 - 10, Y0)
    bot, st_b = fit_edge(warp, 'h', X0 + 10, X7 - 10, Y7)
    left, st_l = fit_edge(warp, 'v', Y0 + 10, Y7 - 10, X0)
    right, st_r = fit_edge(warp, 'v', Y0 + 10, Y7 - 10, X7)
    edges = [top, bot, left, right]
    stds = [st_t, st_b, st_l, st_r]
    if any(e is None for e in edges):
        return None, stds

    def inter(hl, vl):
        ah, bh = hl
        av, bv = vl
        y = (ah * bv + bh) / (1 - ah * av + 1e-9)
        x = av * y + bv
        return np.array([x, y])

    tl = inter(top, left)
    tr = inter(top, right)
    br = inter(bot, right)
    bl = inter(bot, left)
    return dict(tl=tl, tr=tr, br=br, bl=bl), stds


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
    T = np.array([[PPM, 0, QZ * PPM], [0, PPM, QZ * PPM], [0, 0, 1]], np.float64)
    Hp2w = T @ H0

    gt = ground_truth()
    canon = {
        'TL': [(0, 0), (7, 0), (7, 7), (0, 7)],
        'TR': [(N - 7, 0), (N, 0), (N, 7), (N - 7, 7)],
        'BL': [(0, N - 7), (7, N - 7), (7, N), (0, N)],
    }
    pos = {'TL': (0, 0), 'TR': (N - 7, 0), 'BL': (0, N - 7)}

    Hcur = Hp2w.copy()
    for it in range(3):
        warp = cv2.warpPerspective(gray, Hcur.astype(np.float32), (S, S), flags=cv2.INTER_CUBIC)
        cv2.imwrite(f'warp9_{it}.png', warp)
        src, dst = [], []
        ok_cnt = 0
        for k in ['TL', 'TR', 'BL']:
            x0, y0 = pos[k]
            q, stds = corners_from_edges(warp, x0, y0)
            if q is None:
                print(f'it{it} {k} FAILED stds={stds}')
                continue
            ok_cnt += 1
            for key, cname in [('tl', 0), ('tr', 1), ('br', 2), ('bl', 3)]:
                # warp像素 -> module
                src.append([(q[key][0] - QZ * PPM) / PPM, (q[key][1] - QZ * PPM) / PPM])
                dst.append(list(canon[k][cname]))
            print(f'it{it} {k} ok stds={[round(s,2) for s in stds]}')
        if ok_cnt < 2:
            break
        Hm, _ = cv2.findHomography(np.array(src, np.float32), np.array(dst, np.float32), cv2.RANSAC, 0.3)
        # src 是当前 warp 中的 module 坐标, dst 是规范 module 坐标
        # Hcur: photo->warp像素; 下一轮 warp: photo -> T@Hm@inv(T)@Hcur
        Hcur = (T @ Hm.astype(np.float64) @ np.linalg.inv(T)) @ Hcur
        print(f'it{it}: 复合完成, 可用finder {ok_cnt}/3')

    # 最终采样: Hcur: photo->warp, inv(T): warp->module
    Hmod = np.linalg.inv(T) @ Hcur
    Hinv = np.linalg.inv(Hmod)
    K = 7
    xs = ((np.arange(K) / (K - 1) - 0.5) * 0.36 + 0.5)
    rows, cols = np.meshgrid(np.arange(N), np.arange(N), indexing='ij')
    rr, cc = rows.ravel(), cols.ravel()
    Uf = np.tile(cc[:, None] + xs[None, :], (1, K)).ravel()
    Vf = np.repeat(rr[:, None] + xs[None, :], K, axis=1).ravel()
    n = len(rr)
    pts = np.stack([Uf, Vf, np.ones_like(Uf)], 0)
    imgp = Hinv @ pts
    imgp = imgp[:2] / imgp[2:3]
    vals = cv2.remap(gray, imgp[0].astype(np.float32).reshape(n, K * K),
                     imgp[1].astype(np.float32).reshape(n, K * K), cv2.INTER_LINEAR).mean(axis=1)
    med = vals.reshape(N, N)
    med_u8 = np.clip(med, 0, 255).astype(np.uint8)
    cv2.imwrite('med9.png', med_u8)
    _, bw = cv2.threshold(med_u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    matrix = (bw < 127).astype(np.uint8)
    np.save('matrix9.npy', matrix)

    def dih(m):
        return [np.rot90(m, k) for k in range(4)] + [np.fliplr(np.rot90(m, k)) for k in range(4)]
    best = min(((int(np.count_nonzero(matrix != d)), i) for i, d in enumerate(dih(gt))))
    print('错误模块:', best[0], '/441 变换idx', best[1])
    for r in range(N):
        print(''.join('##' if matrix[r, c] else '  ' for c in range(N)))


if __name__ == '__main__':
    main()
