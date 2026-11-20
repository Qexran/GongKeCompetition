# -*- coding: utf-8 -*-
"""原型8：仿射warp后在规范空间做亚像素边缘检测"""
import cv2
import numpy as np

PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'
ORIG = r'C:\Users\Administrator\Desktop\QRCodePNG\133.png'
N = 21
PPM = 40  # warp 后每模块像素
S = N * PPM


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


def subpixel_fall(profile, expect_idx):
    """找亮->暗下落沿（在 expect_idx 附近）, 返回亚像素位置"""
    p = profile.astype(np.float64)
    d = np.diff(p)
    lo = max(0, expect_idx - PPM)
    hi = min(len(d), expect_idx + PPM)
    if hi <= lo:
        return None
    seg = d[lo:hi]
    i = lo + int(np.argmin(seg))  # 最大负跳变
    if seg.min() > -15:
        return None
    a, b = p[i], p[i + 1]
    mid = (p.max() + p.min()) / 2
    if b == a:
        return None
    x = i + (mid - a) / (b - a)
    if abs(x - expect_idx) > PPM:
        return None
    return x


def edge_line_from_profiles(warp, axis, c0, c1, expect, bright_side):
    """axis='h': 水平边（沿x采样，剖面为列向量）; expect: 预期行位置(像素)
    bright_side: 亮侧在 expect 的 -1/ +1 方向
    返回拟合直线的 (均值位置, 法向残差有效性)——这里边都是横/竖的，直接收集亚像素位置再均值+鲁棒筛选"""
    pos_list = []
    rng = range(int(c0), int(c1), 2)
    for c in rng:
        if axis == 'h':
            prof = warp[:, c]
            e = subpixel_fall(prof, expect, )
            if e is not None and bright_side < 0:
                pass
            if e is not None:
                pos_list.append((c, e))
        else:
            prof = warp[c, :]
            e = subpixel_fall(prof, expect)
            if e is not None:
                pos_list.append((c, e))
    if len(pos_list) < 10:
        return None
    arr = np.array(pos_list)
    med = np.median(arr[:, 1])
    mad = np.median(np.abs(arr[:, 1] - med)) + 1e-9
    keep = np.abs(arr[:, 1] - med) < 3 * mad * 1.5 + 1.0
    arr = arr[keep]
    if len(arr) < 8:
        return None
    # 拟合一次直线（位置 ~ 斜率*c + 截距）处理小倾斜
    A = np.stack([arr[:, 0], np.ones_like(arr[:, 0])], 1)
    coef, *_ = np.linalg.lstsq(A, arr[:, 1], rcond=None)
    resid = arr[:, 1] - A @ coef
    return coef, float(np.std(resid)), len(arr)


def main():
    img = imread_u(PHOTO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # H0: photo -> module（3中心仿射）
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
                   [0, 0, 1]])
    # photo -> warp像素 = (module)*PPM
    Sc = np.diag([PPM, PPM, 1.0])
    Hphoto2warp = Sc @ H0
    warp = cv2.warpPerspective(gray, Hphoto2warp.astype(np.float32), (S, S),
                               flags=cv2.INTER_CUBIC)
    cv2.imwrite('warp8.png', warp)

    # finder 规范位置（warp像素）
    specs = {
        'TL': dict(x0=0, y0=0),
        'TR': dict(x0=N - 7, y0=0),
        'BL': dict(x0=0, y0=N - 7),
    }
    src, dst = [], []
    canon = {
        'TL': [(0, 0), (7, 0), (7, 7), (0, 7)],
        'TR': [(N - 7, 0), (N, 0), (N, 7), (N - 7, 7)],
        'BL': [(0, N - 7), (7, N - 7), (7, N), (0, N)],
    }
    for k, sp in specs.items():
        x0, y0 = sp['x0'] * PPM, sp['y0'] * PPM
        x7, y7 = (sp['x0'] + 7) * PPM, (sp['y0'] + 7) * PPM
        # 4条边: top(y0) bottom(y7) left(x0) right(x7)
        top = edge_line_from_profiles(warp, 'h', x0 + 8, x7 - 8, y0, bright_side=-1)
        bot = edge_line_from_profiles(warp, 'h', x0 + 8, x7 - 8, y7, bright_side=+1)
        left = edge_line_from_profiles(warp, 'v', y0 + 8, y7 - 8, x0, bright_side=-1)
        right = edge_line_from_profiles(warp, 'v', y0 + 8, y7 - 8, x7, bright_side=+1)
        print(k, 'edges:', top is not None, bot is not None, left is not None, right is not None)
        if any(e is None for e in [top, bot, left, right]):
            continue
        def inter(hline, vline):
            # hline: y = a*hx+b ; vline: x = a*vy+b
            ha, hb = hline[0]
            va, vb = vline[0]
            # vline 给出 x = va*y + vb -> y = (x - vb)/va
            x_at = lambda y: ha * y + hb  # 参数是沿采样坐标...简化: 直接求交点
            # hline 参数化 pos(c)=a*c+b, c 是 x 坐标 -> y(x)=a*x+b
            # vline: x(y)=a*y+b
            # y = a_h*(a_v*y+b_v)+b_h -> y(1-a_h*a_v)=a_h*b_v+b_h
            ah, bh = ha, hb
            av, bv = va, vb
            y = (ah * bv + bh) / (1 - ah * av + 1e-12)
            x = av * y + bv
            return np.array([x, y])
        tl = inter(top, left)
        tr = inter(top, right)
        br = inter(bot, right)
        bl = inter(bot, left)
        for pt, cn in zip([tl, tr, br, bl], canon[k]):
            src.append(pt)
            dst.append(cn)
        print('   ', k, 'corners(px):', np.round([tl, tr, br, bl]).astype(int).tolist(),
              'stds:', [round(top[1],2), round(bot[1],2), round(left[1],2), round(right[1],2)])

    src = np.array(src, np.float64)
    dst = np.array(dst, np.float64)
    # warp像素 -> module
    Hw2m, _ = cv2.findHomography(src.astype(np.float32), dst.astype(np.float32), cv2.RANSAC, 0.3)
    H = Hw2m @ Hphoto2warp
    H = H.astype(np.float64)
    np.save('H8.npy', H)

    # 采样
    K = 7
    xs = ((np.arange(K) / (K - 1) - 0.5) * 0.36 + 0.5)
    Hinv = np.linalg.inv(H)
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
    cv2.imwrite('med8.png', med_u8)
    _, bw = cv2.threshold(med_u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    matrix = (bw < 127).astype(np.uint8)
    np.save('matrix8.npy', matrix)

    gt = ground_truth()
    def dih(m):
        return [np.rot90(m, k) for k in range(4)] + [np.fliplr(np.rot90(m, k)) for k in range(4)]
    best = min(((int(np.count_nonzero(matrix != d)), i) for i, d in enumerate(dih(gt))))
    print('错误模块:', best[0], '/441 变换idx', best[1])
    for r in range(N):
        print(''.join('##' if matrix[r, c] else '  ' for c in range(N)))


if __name__ == '__main__':
    main()
