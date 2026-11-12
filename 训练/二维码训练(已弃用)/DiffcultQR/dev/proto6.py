# -*- coding: utf-8 -*-
"""原型6：单应性非线性优化（QR结构约束）→ 精确采样 → 重建解码"""
import cv2
import numpy as np

PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'
ORIG = r'C:\Users\Administrator\Desktop\QRCodePNG\133.png'
N = 21


def imread_u(path):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


def known_modules(N):
    """返回 dict {(r,c): 0/1} 仅含版本通用结构（finder/分隔/时序/暗模块）"""
    k = {}

    def finder(r0, c0):
        for r in range(7):
            for c in range(7):
                d = max(abs(r - 3), abs(c - 3))
                k[(r0 + r, c0 + c)] = 1 if d != 1 else 0  # d=2 ring dark, d=1 white, d<=0 dark

    finder(0, 0)
    finder(0, N - 7)
    finder(N - 7, 0)
    # 分隔带
    for c in range(8):
        k[(7, c)] = 0
        k[(c, 7)] = 0
        k[(7, N - 1 - c)] = 0
        k[(c, N - 8)] = 0
        k[(N - 8, c)] = 0
        k[(N - 1 - c, 7)] = 0
    # 时序
    for i in range(8, N - 8):
        k[(6, i)] = 1 if i % 2 == 0 else 0
        k[(i, 6)] = 1 if i % 2 == 0 else 0
    # 暗模块
    k[(4 * 1 + 9, 8)] = 1
    return k


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


# ---------- 采样核心 ----------
def make_sampler(gray, N, K=5, frac=0.4):
    """返回 sample(Hinv, coords) -> 强度。预先把 KxK 采样点打包"""
    xs = ((np.arange(K) / (K - 1) - 0.5) * frac + 0.5)

    def sample_module(Hinv, rs, cs):
        """rs, cs: 模块坐标数组 -> 强度数组（中心区域均值）"""
        U, V = np.meshgrid(cs[:, None] + xs[None, :], rs[:, None] + xs[None, :])
        U = np.tile(U.ravel(), len(rs))
        V = np.tile(V.ravel(), len(rs))
        n = len(rs)
        pts = np.stack([U, V, np.ones_like(U)], 0)
        imgp = Hinv @ pts
        imgp = imgp[:2] / imgp[2:3]
        vals = cv2.remap(gray, imgp[0].astype(np.float32).reshape(n, -1),
                         imgp[1].astype(np.float32).reshape(n, -1), cv2.INTER_LINEAR)
        return vals.mean(axis=1)

    return sample_module


def objective_factory(gray, N, T):
    kn = known_modules(N)
    coords = np.array(list(kn.keys()))
    vals_ref = np.array(list(kn.values()), np.float32)
    K = 5
    xs = ((np.arange(K) / (K - 1) - 0.5) * 0.4 + 0.5)
    rr, cc = coords[:, 0], coords[:, 1]
    Uf = np.tile(cc[:, None] + xs[None, :], (1, K)).ravel()
    Vf = np.repeat(rr[:, None] + xs[None, :], K, axis=1).ravel()

    def objective(p):
        D = np.array([[1 + p[0], p[1], p[2]],
                      [p[3], 1 + p[4], p[5]],
                      [p[6], p[7], 1.0]], np.float64)
        try:
            Hinv = np.linalg.inv(D)
        except np.linalg.LinAlgError:
            return 1e9
        pts = np.stack([Uf, Vf, np.ones_like(Uf)], 0)
        imgp = Hinv @ pts
        w = imgp[2:3]
        w[np.abs(w) < 1e-12] = 1e-12
        imgp = imgp[:2] / w
        n = len(coords)
        vals = cv2.remap(gray, imgp[0].astype(np.float32).reshape(n, K * K),
                         imgp[1].astype(np.float32).reshape(n, K * K), cv2.INTER_LINEAR).mean(axis=1)
        # 结构一致性
        s = np.tanh((T - vals) / 35.0) * (2 * vals_ref - 1) * -1
        # vals_ref=1(暗): tanh((T-v)/35) 大 -> 好; vals_ref=0(亮): -tanh(...) 
        score = s.sum()
        score -= 2.0 * np.sum(np.array(p) ** 2)
        return -score  # minimize

    return objective


def nelder_mead(f, x0, step=0.2, maxiter=3000, tol=1e-6):
    """纯 numpy Nelder-Mead"""
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
        if np.std(fv) < tol and np.max(np.abs(sim[1:] - sim[0])) < 1e-4:
            break
        cen = sim[:-1].mean(axis=0)
        xr = cen + 1.0 * (cen - sim[-1])
        fr = f(xr)
        if fv[0] <= fr < fv[-2]:
            sim[-1], fv[-1] = xr, fr
        elif fr < fv[0]:
            xe = cen + 2.0 * (cen - sim[-1])
            fe = f(xe)
            if fe < fr:
                sim[-1], fv[-1] = xe, fe
            else:
                sim[-1], fv[-1] = xr, fr
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
    return sim[order][0], fv[order][0], it


def optimize(gray, H0, N, T):
    obj = objective_factory(gray, N, T)
    p0 = np.zeros(8)
    best = None
    starts = [p0]
    rng = np.random.default_rng(42)
    for _ in range(4):
        starts.append(rng.normal(0, 0.12, 8))
    for s in starts:
        x, fval, it = nelder_mead(obj, s, step=0.25, maxiter=2500)
        if best is None or fval < best[1]:
            best = (x, fval, it)
    p = best[0]
    D = np.array([[1 + p[0], p[1], p[2]],
                  [p[3], 1 + p[4], p[5]],
                  [p[6], p[7], 1.0]], np.float64)
    return D @ H0, best


def main():
    img = imread_u(PHOTO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gt = ground_truth()

    # Otsu 中间阈值（QR 区域）
    roi = gray[150:800, 440:950]
    T, _ = cv2.threshold(roi, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    print('Otsu T =', T)

    # 初始 H：用三个 finder 中心的仿射基（可靠）
    TL = np.array([582.07, 507.66])
    TR = np.array([713.56, 678.29])  # 按上次输出 TL/TR/BL
    BL = np.array([839.12, 289.30])
    # 判定直角顶点
    cs = [TL, TR, BL]
    dots = [abs(np.dot(cs[(i + 1) % 3] - cs[i], cs[(i + 2) % 3] - cs[i]) /
                (np.linalg.norm(cs[(i + 1) % 3] - cs[i]) * np.linalg.norm(cs[(i + 2) % 3] - cs[i])))
           for i in range(3)]
    i = int(np.argmin(dots))
    v = cs[i]
    a, b = cs[(i + 1) % 3], cs[(i + 2) % 3]

    def cross2(x, y):
        return x[0] * y[1] - x[1] * y[0]

    if cross2(b - v, a - v) > 0:
        TRc, BLc = b, a
    else:
        TRc, BLc = a, b
    TLc = v
    print('TLc', TLc, 'TRc', TRc, 'BLc', BLc)
    ex = (TRc - TLc) / (N - 7)
    ey = (BLc - TLc) / (N - 7)
    # photo -> module
    A = np.array([np.concatenate([ex, [0]]), np.concatenate([ey, [0]]),
                  [0, 0, 0]])
    A = np.array([[ex[0], ey[0], 0], [ex[1], ey[1], 0], [0, 0, 1]], np.float64)
    t = TLc - 3.5 * ex - 3.5 * ey
    Ainv = np.linalg.inv(A)
    # module -> photo: p = A @ m + t ; photo->module: m = Ainv(p - t)
    H0 = np.array([[Ainv[0, 0], Ainv[0, 1], -(Ainv[0, 0] * t[0] + Ainv[0, 1] * t[1])],
                   [Ainv[1, 0], Ainv[1, 1], -(Ainv[1, 0] * t[0] + Ainv[1, 1] * t[1])],
                   [0, 0, 1]], np.float64)

    Hopt, res = optimize(gray, H0, N, T)
    print('优化收敛: fval=', res[1], 'iters=', res[2])
    # 除零保护

    np.save('Hopt.npy', Hopt)

    # 采样整个矩阵
    K = 7
    xs = ((np.arange(K) / (K - 1) - 0.5) * 0.36 + 0.5)
    Hinv = np.linalg.inv(Hopt)
    rows, cols = np.meshgrid(np.arange(N), np.arange(N), indexing='ij')
    rr, cc = rows.ravel(), cols.ravel()
    n = len(rr)
    Uf = np.tile(cc[:, None] + xs[None, :], (1, K)).ravel()
    Vf = np.repeat(rr[:, None] + xs[None, :], K, axis=1).ravel()
    pts = np.stack([Uf, Vf, np.ones_like(Uf)], 0)
    imgp = Hinv @ pts
    imgp = imgp[:2] / imgp[2:3]
    vals = cv2.remap(gray, imgp[0].astype(np.float32).reshape(n, K * K),
                     imgp[1].astype(np.float32).reshape(n, K * K), cv2.INTER_LINEAR).mean(axis=1)
    med = vals.reshape(N, N)
    med_u8 = np.clip(med, 0, 255).astype(np.uint8)
    _, bw = cv2.threshold(med_u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    matrix = (bw < 127).astype(np.uint8)

    # 与 gt 对比（8种姿态）
    def dih(m):
        return [np.rot90(m, k) for k in range(4)] + [np.fliplr(np.rot90(m, k)) for k in range(4)]

    best = min(((int(np.count_nonzero(matrix != d)), i) for i, d in enumerate(dih(gt))))
    print('错误模块:', best[0], '/441 变换idx', best[1])
    for r in range(N):
        print(''.join('##' if matrix[r, c] else '  ' for c in range(N)))
    np.save('matrix_opt.npy', matrix)
    cv2.imwrite('med_opt.png', med_u8)


if __name__ == '__main__':
    main()
