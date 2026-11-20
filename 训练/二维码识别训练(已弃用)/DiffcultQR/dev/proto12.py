# -*- coding: utf-8 -*-
"""proto12: 可信锚点(finder边线+时序)驱动的三次形变场"""
import cv2
import numpy as np

N, PPM, QZ = 21, 40, 6
S = (N + 2 * QZ) * PPM
OFF = QZ * PPM
PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'
ORIG = r'C:\Users\Administrator\Desktop\QRCodePNG\133.png'


def imread_u(path):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


# ---- 复用 proto10 的边缘拟合 ----
SRC10 = open('proto10.py', encoding='utf-8').read().split("if __name__")[0]
SRC10 = SRC10.replace("def main():", "def _unused():")
NS = {}
exec(SRC10, NS)
first_fall_from_outside = NS['first_fall_from_outside']
fit_edge = NS['fit_edge']
finder_quad = NS['finder_quad']


def load_warp():
    img = imread_u(PHOTO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    TLc = np.array([582.07, 507.66]); TRc = np.array([713.56, 678.29]); BLc = np.array([839.12, 289.30])
    ex = (TRc - TLc) / (N - 7); ey = (BLc - TLc) / (N - 7)
    A = np.array([[ex[0], ey[0]], [ex[1], ey[1]]]); t = TLc - 3.5 * ex - 3.5 * ey
    Ainv = np.linalg.inv(A)
    H0 = np.array([[Ainv[0, 0], Ainv[0, 1], -(Ainv @ t)[0]],
                   [Ainv[1, 0], Ainv[1, 1], -(Ainv @ t)[1]], [0, 0, 1]], np.float64)
    T = np.array([[PPM, 0, OFF], [0, PPM, OFF], [0, 0, 1]], np.float64)
    return gray, T @ H0


def collect_anchors(warp, coef_x, coef_y, basis_fn):
    """锚点: (u_canon, v_canon, x_warp_obs, y_warp_obs, weight)"""
    anchors = []
    pos = {'TL': (0, 0), 'TR': (N - 7, 0), 'BL': (0, N - 7)}
    canon_c = {'TL': (0, 0), 'TR': (N - 7, 0), 'BL': (0, N - 7)}
    for k, (mx, my) in pos.items():
        X0, Y0 = OFF + mx * PPM, OFF + my * PPM
        X7, Y7 = X0 + 7 * PPM, Y0 + 7 * PPM
        # 4条边: (axis, a0, a1, expect, out_dir, canon_line_k)
        edges = [
            ('h', X0 + 6, X7 - 6, Y0, -1, my),      # top -> v = my
            ('h', X0 + 6, X7 - 6, Y7, +1, my + 7),  # bottom -> v = my+7
            ('v', Y0 + 6, Y7 - 6, X0, -1, mx),      # left -> u = mx
            ('v', Y0 + 6, Y7 - 6, X7, +1, mx + 7),  # right -> u = mx+7
        ]
        for (axis, a0, a1, expect, out_dir, line_k) in edges:
            res = fit_edge(warp, axis, a0, a1, expect, out_dir)
            if res[0] is None:
                continue
            coef_line, std = res
            w = 1.0 / (1.0 + std * std)
            if axis == 'h':
                # y = a*x + b; 沿 canonical u 取点
                for u in np.arange(canon_c[k][0] + 0.5, canon_c[k][0] + 7.0, 0.5):
                    # u -> warp x (当前模型)
                    bx = basis_fn(np.array([u]), np.array([float(line_k)]))
                    xw = OFF + u * PPM + float((bx @ coef_x)[0])
                    yw = coef_line[0] * xw + coef_line[1]
                    anchors.append((u, float(line_k), xw, yw, w))
            else:
                for v in np.arange(canon_c[k][1] + 0.5, canon_c[k][1] + 7.0, 0.5):
                    by = basis_fn(np.array([float(line_k)]), np.array([v]))
                    yw = OFF + v * PPM + float((by @ coef_y)[0])
                    xw = coef_line[0] * yw + coef_line[1]
                    anchors.append((float(line_k), v, xw, yw, w))
    # 角点锚点
    for k, (mx, my) in pos.items():
        res = finder_quad(warp, mx, my)
        if res is None:
            continue
        q = res[0]
        for key, (cu, cv) in [('tl', (mx, my)), ('tr', (mx + 7, my)),
                              ('br', (mx + 7, my + 7)), ('bl', (mx, my + 7))]:
            anchors.append((cu, cv, OFF + q[key][0] * PPM, OFF + q[key][1] * PPM, 3.0))
    # 时序锚点
    tp = NS['timing_edge_points'](warp)
    for (a, b), (c, d) in tp:
        anchors.append((c, d, OFF + a * PPM, OFF + b * PPM, 1.5))
    return anchors


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
        s = 1.4826 * np.median(np.abs(r - np.median(r))) + 1e-6
        w = w0 * np.clip(1 - (np.abs(r) / (4.5 * s)) ** 2, 0, 1) ** 2
    return coef


def main():
    gray, Hp2w = load_warp()
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
        cx = robust_fit(A, ax - (OFF + au * PPM), aw)
        cy = robust_fit(A, ay - (OFF + av * PPM), aw)
        rx = ax - (OFF + au * PPM) - A @ cx
        ry = ay - (OFF + av * PPM) - A @ cy
        print(f'round{rnd}: anchors={len(anchors)} rms x={np.sqrt((rx**2).mean()):.2f} y={np.sqrt((ry**2).mean()):.2f}')
        coef_x, coef_y = cx, cy

    # 采样
    vals = np.zeros((N, N), np.float32)
    for r in range(N):
        for c in range(N):
            xs = []
            for k in (c, c + 1):
                bx = cubic_design(np.array([float(k)]), np.array([r + 1.0]))
                xs.append(OFF + k * PPM + float((bx @ coef_x)[0]))
            ys = []
            for k in (r, r + 1):
                by = cubic_design(np.array([c + 1.0]), np.array([float(k)]))
                ys.append(OFF + k * PPM + float((by @ coef_y)[0]))
            xc, yc = (xs[0] + xs[1]) / 2, (ys[0] + ys[1]) / 2
            pts = []
            for dy in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                for dx in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                    xi, yi = int(xc + dx), int(yc + dy)
                    if 0 <= xi < S and 0 <= yi < S:
                        pts.append(warp[yi, xi])
            vals[r, c] = np.median(pts)
    _, bw = cv2.threshold(np.clip(vals, 0, 255).astype(np.uint8), 0, 255,
                          cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    mat = (bw < 127).astype(np.uint8)
    np.save('matrix_cubic.npy', mat)

    ideal = np.array([[1, 1, 1, 1, 1, 1, 1], [1, 0, 0, 0, 0, 0, 1], [1, 0, 1, 1, 1, 0, 1],
                      [1, 0, 1, 1, 1, 0, 1], [1, 0, 1, 1, 1, 0, 1], [1, 0, 0, 0, 0, 0, 1],
                      [1, 1, 1, 1, 1, 1, 1]])
    print('TL:', (mat[0:7, 0:7] == ideal).all(), ' TR:', (mat[0:7, 14:21] == ideal).all(),
          ' BL:', (mat[14:21, 0:7] == ideal).all())
    print('timing:', mat[6, 8:13].tolist(), mat[8:13, 6].tolist())

    def structure_score(m):
        dark = np.zeros((N, N), bool); light = np.zeros((N, N), bool)
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

    g = cv2.cvtColor(imread_u(ORIG), cv2.COLOR_BGR2GRAY)
    _, bwp = cv2.threshold(g, 128, 255, cv2.THRESH_BINARY)
    gt = np.zeros((N, N), np.uint8)
    for r in range(N):
        for c in range(N):
            cell = bwp[(4 + r) * 20 + 6:(4 + r + 1) * 20 - 6, (4 + c) * 20 + 6:(4 + c + 1) * 20 - 6]
            gt[r, c] = 1 if np.mean(cell < 127) > 0.5 else 0
    def dih(x):
        return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]
    best = min((int(np.count_nonzero(mat != d)), i) for i, d in enumerate(dih(gt)))
    print('vs 133:', best)


if __name__ == '__main__':
    main()
