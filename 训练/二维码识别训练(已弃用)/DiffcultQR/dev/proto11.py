# -*- coding: utf-8 -*-
"""proto11: 曲线网格追踪 + 二阶多项式形变场 + 精采样"""
import cv2
import numpy as np

N, PPM, QZ = 21, 40, 6
S = (N + 2 * QZ) * PPM
OFF = QZ * PPM
PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'
ORIG = r'C:\Users\Administrator\Desktop\QRCodePNG\133.png'


def imread_u(path):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


def load_warp():
    img = imread_u(PHOTO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    TLc = np.array([582.07, 507.66])
    TRc = np.array([713.56, 678.29])
    BLc = np.array([839.12, 289.30])
    ex = (TRc - TLc) / (N - 7)
    ey = (BLc - TLc) / (N - 7)
    A = np.array([[ex[0], ey[0]], [ex[1], ey[1]]])
    t = TLc - 3.5 * ex - 3.5 * ey
    Ainv = np.linalg.inv(A)
    H0 = np.array([[Ainv[0, 0], Ainv[0, 1], -(Ainv @ t)[0]],
                   [Ainv[1, 0], Ainv[1, 1], -(Ainv @ t)[1]],
                   [0, 0, 1]], np.float64)
    T = np.array([[PPM, 0, OFF], [0, PPM, OFF], [0, 0, 1]], np.float64)
    return gray, T @ H0


def quad_design(u, v):
    """二阶多项式基"""
    return np.stack([np.ones_like(u), u, v, u * u, u * v, v * v], 1)


def robust_fit(Amat, b, w0, iters=4):
    w = w0.copy()
    for _ in range(iters):
        W = w[:, None]
        coef = np.linalg.lstsq(Amat * W, b * w, rcond=None)[0]
        r = b - Amat @ coef
        s = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-6
        w = w0 * np.clip(1 - (np.abs(r) / (4.5 * s)) ** 2, 0, 1) ** 2
    return coef


def row_peaks(prof, thr=100, minsep=6):
    """一维剖面上的局部极大(亚像素)"""
    out = []
    n = len(prof)
    for i in range(2, n - 2):
        v = prof[i]
        if v < thr or v < prof[i - 1] or v < prof[i + 1]:
            continue
        # 局部极大且与已选峰距离足够
        if out and i - out[-1][0] < minsep:
            if v > out[-1][1]:
                out.pop()
            else:
                continue
        a, b, c = prof[i - 1], v, prof[i + 1]
        den = a - 2 * b + c
        dj = 0.5 * (a - c) / den if den != 0 else 0
        dj = float(np.clip(dj, -1, 1))
        out.append((i, v, i + dj))
    return out


def trace_grid(warp, rounds=3):
    sm = cv2.GaussianBlur(warp, (0, 0), 1.2)
    gx = np.abs(cv2.Sobel(sm, cv2.CV_32F, 1, 0, ksize=3))
    gy = np.abs(cv2.Sobel(sm, cv2.CV_32F, 0, 1, ksize=3))
    coef_x = np.zeros(6)
    coef_y = np.zeros(6)
    for rnd in range(rounds):
        vobs = []  # (k, v_row, x, strength)
        hobs = []
        win = 0.65 if rnd == 0 else 0.45
        for y in range(4, S - 4, 2):
            for (i, v, xp) in row_peaks(gx[y]):
                # 预测: 该峰最近的垂直线
                vrow = (y - OFF) / PPM
                best_k, best_d = None, 1e9
                for k in range(N + 1):
                    pred = OFF + k * PPM + quad_design(np.array([float(k)]), np.array([vrow]))[0] @ coef_x
                    d = abs(xp - pred)
                    if d < best_d:
                        best_d, best_k = d, k
                if best_k is not None and best_d < win * PPM:
                    vobs.append((best_k, vrow, xp, v))
        for x in range(4, S - 4, 2):
            for (i, v, yp) in row_peaks(gx_col(gy, x)):
                vcol = (x - OFF) / PPM
                best_k, best_d = None, 1e9
                for k in range(N + 1):
                    pred = OFF + k * PPM + quad_design(np.array([float(k)]), np.array([vcol]))[0] @ coef_y
                    d = abs(yp - pred)
                    if d < best_d:
                        best_d, best_k = d, k
                if best_k is not None and best_d < win * PPM:
                    hobs.append((best_k, vcol, yp, v))
        vk = np.array([o[0] for o in vobs], np.float64)
        vv = np.array([o[1] for o in vobs], np.float64)
        vx = np.array([o[2] for o in vobs], np.float64)
        vw = np.array([o[3] for o in vobs], np.float64)
        if len(vobs) < 50:
            break
        Amat = quad_design(vk, vv)
        target = vx - (OFF + vk * PPM)
        coef_x = robust_fit(Amat, target, vw / vw.max())
        hk = np.array([o[0] for o in hobs], np.float64)
        hv = np.array([o[1] for o in hobs], np.float64)
        hy = np.array([o[2] for o in hobs], np.float64)
        hw = np.array([o[3] for o in hobs], np.float64)
        if len(hobs) < 50:
            break
        Amath = quad_design(hk, hv)
        targeth = hy - (OFF + hk * PPM)
        coef_y = robust_fit(Amath, targeth, hw / hw.max())
        rx = target - Amat @ coef_x
        ry = targeth - Amath @ coef_y
        print(f'round{rnd}: vobs={len(vobs)} rms={np.sqrt((rx**2).mean()):.2f}px | '
              f'hobs={len(hobs)} rms={np.sqrt((ry**2).mean()):.2f}px')
    return coef_x, coef_y


def gx_col(gy, x):
    return gy[:, x]


def sample(warp, coef_x, coef_y):
    vals = np.zeros((N, N), np.float32)
    for r in range(N):
        for c in range(N):
            # 模块中心的四条边界线在该处的位置
            vrow = (r + 1)  # 以模块下边界行评估
            ucol = (c + 1)
            # x 方向: 线 c 和 c+1 在行 v=r+1 处
            xs = []
            for k in (c, c + 1):
                basis = quad_design(np.array([k], np.float64), np.array([r + 1.0], np.float64))
                xk = OFF + k * PPM + float((basis @ coef_x)[0])
                xs.append(xk)
            ys = []
            for k in (r, r + 1):
                basis = quad_design(np.array([c + 1.0], np.float64), np.array([k], np.float64))
                yk = OFF + k * PPM + float((basis @ coef_y)[0])
                ys.append(yk)
            xc, yc = (xs[0] + xs[1]) / 2, (ys[0] + ys[1]) / 2
            # 中心区域 5x5 采样中值
            pts = []
            for dy in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                for dx in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                    xi, yi = int(xc + dx), int(yc + dy)
                    if 0 <= xi < S and 0 <= yi < S:
                        pts.append(warp[yi, xi])
            vals[r, c] = np.median(pts)
    return vals


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


def main():
    gray, Hp2w = load_warp()
    warp = cv2.warpPerspective(gray, Hp2w.astype(np.float32), (S, S), flags=cv2.INTER_CUBIC)
    coef_x, coef_y = trace_grid(warp)
    vals = sample(warp, coef_x, coef_y)
    _, bw = cv2.threshold(np.clip(vals, 0, 255).astype(np.uint8), 0, 255,
                          cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mat = (bw < 127).astype(np.uint8)
    np.save('matrix_poly.npy', mat)
    cv2.imwrite('warp_poly.png', warp)

    ideal = np.array([[1, 1, 1, 1, 1, 1, 1], [1, 0, 0, 0, 0, 0, 1], [1, 0, 1, 1, 1, 0, 1],
                      [1, 0, 1, 1, 1, 0, 1], [1, 0, 1, 1, 1, 0, 1], [1, 0, 0, 0, 0, 0, 1],
                      [1, 1, 1, 1, 1, 1, 1]])
    print('TL:', (mat[0:7, 0:7] == ideal).all(), ' TR:', (mat[0:7, 14:21] == ideal).all(),
          ' BL:', (mat[14:21, 0:7] == ideal).all())
    print('timing:', mat[6, 8:13].tolist(), mat[8:13, 6].tolist())

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
    print('结构得分:', round(structure_score(mat), 3))

    gt = ground_truth()
    def dih(x):
        return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]
    best = min((int(np.count_nonzero(mat != d)), i) for i, d in enumerate(dih(gt)))
    print('vs 133:', best)


if __name__ == '__main__':
    main()
