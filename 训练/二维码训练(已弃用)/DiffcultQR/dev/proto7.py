# -*- coding: utf-8 -*-
"""原型7：finder 外边缘亚像素检测 + 直线拟合 -> 12角点精确单应性"""
import cv2
import numpy as np

PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'
ORIG = r'C:\Users\Administrator\Desktop\QRCodePNG\133.png'
N = 21


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


def subpixel_edge(profile, dark_side):
    """在1D剖面中找暗亮过渡的亚像素位置。dark_side=+1 表示剖面起点侧是暗"""
    p = profile.astype(np.float64)
    lo, hi = p.min(), p.max()
    if hi - lo < 30:
        return None
    mid = (lo + hi) / 2
    # 找最大的上升/下降跳变
    d = np.diff(p)
    if dark_side > 0:  # 暗->亮, 找最大正跳变
        i = int(np.argmax(d))
        if d[i] <= 0:
            return None
    else:
        i = int(np.argmin(d))
        if d[i] >= 0:
            return None
    # 线性插值过中线
    a, b = p[i], p[i + 1]
    if b == a:
        return i + 0.5
    return i + (mid - a) / (b - a)


def edge_points_along_line(gray, p0, p1, normal_dir, halfwidth, npts=25):
    """沿 p0->p1 的线，每隔一定距离取垂直方向的剖面做亚像素边缘检测"""
    pts = []
    for t in np.linspace(0.1, 0.9, npts):
        base = p0 + t * (p1 - p0)
        # 剖面: base ± halfwidth*normal
        L = np.linspace(-halfwidth, halfwidth, int(halfwidth * 2 * 2) + 1)
        samples = []
        for l in L:
            q = base + l * normal_dir
            xi, yi = int(round(q[0])), int(round(q[1]))
            if 0 <= xi < gray.shape[1] and 0 <= yi < gray.shape[0]:
                samples.append(gray[yi, xi])
            else:
                samples.append(-1)
        samples = np.array(samples)
        if (samples < 0).any():
            continue
        # 外侧（远离finder中心）应为亮，内侧为暗
        e = subpixel_edge(samples, dark_side=+1)  # 剖面从外(亮)到内(暗)? 方向由调用方定
        if e is None:
            continue
        pos = base + (L[0] + e) * normal_dir - L[0] * normal_dir
        # L[0]=-halfwidth, L[0]+e = -halfwidth + e
        pos = base + (-halfwidth + e) * normal_dir
        pts.append(pos)
    return np.array(pts) if pts else None


def fit_line_tls(pts):
    """全最小二乘拟合直线, 返回 (点, 方向)"""
    c = pts.mean(axis=0)
    u, s, vt = np.linalg.svd(pts - c)
    d = vt[0]
    return c, d / np.linalg.norm(d)


def finder_quad(gray, center, ex, ey, m):
    """估计 finder 外框4条边的亚像素边缘并拟合直线求4角点
    ex, ey: 该 finder 处的模块向量（照片像素/模块）; m: 模块尺寸标量"""
    center = np.asarray(center, np.float64)
    ex = np.asarray(ex, np.float64)
    ey = np.asarray(ey, np.float64)
    ux = ex / (np.linalg.norm(ex) + 1e-9)
    uy = ey / (np.linalg.norm(ey) + 1e-9)
    nx, ny = -uy, ux  # 法向
    lines = {}
    hw = max(2.0, 0.8 * np.linalg.norm(ex))
    span = 5.0  # 沿边采样跨度（模块）
    for name, sgn, uvec, nvec in [
        ('top', -1, ux, uy),    # 上边: 法向朝上(uy负方向)
        ('bottom', +1, ux, uy),
        ('left', -1, uy, ux),
        ('right', +1, uy, ux),
    ]:
        # 边中心位置
        edge_c = center + sgn * 3.5 * (ex if name in ('left', 'right') else ey)
        # 边方向: top/bottom 沿 ux; left/right 沿 uy
        along = ux if name in ('top', 'bottom') else uy
        p0 = edge_c - 3.0 * along
        p1 = edge_c + 3.0 * along
        # 法向（指向 finder 外部）
        out_n = -sgn * (uy if name in ('top', 'bottom') else ux)
        # 剖面起点在外侧(亮)，结束在内侧(暗)
        pts = edge_points_along_line(gray, p0, p1, -out_n, hw, npts=30)
        if pts is None or len(pts) < 8:
            return None
        lines[name] = fit_line_tls(pts)
    # 角点 = 线交点
    def intersect(l1, l2):
        c1, d1 = l1
        c2, d2 = l2
        A = np.array([d1, -d2]).T
        b = c2 - c1
        t, s = np.linalg.lstsq(A, b, rcond=None)[0]
        return c1 + t * d1

    tl = intersect(lines['top'], lines['left'])
    tr = intersect(lines['top'], lines['right'])
    br = intersect(lines['bottom'], lines['right'])
    bl = intersect(lines['bottom'], lines['left'])
    return dict(tl=tl, tr=tr, br=br, bl=bl, lines=lines)


def main():
    img = imread_u(PHOTO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gt = ground_truth()

    # finder 中心（此前验证过的）
    TLc = np.array([582.07, 507.66])
    TRc = np.array([713.56, 678.29])
    BLc = np.array([839.12, 289.30])
    centers = {'TL': TLc, 'TR': TRc, 'BL': BLc}
    # 注意：这里 TL/TR/BL 命名对应矩阵姿态；先按 proto3 的组合（TL=582,507 直角点）
    # 模块向量估计（仿射）
    ex = (TRc - TLc) / (N - 7)
    ey = (BLc - TLc) / (N - 7)

    canon_corners = {
        'TL': [(0, 0), (7, 0), (7, 7), (0, 7)],
        'TR': [(N - 7, 0), (N, 0), (N, 7), (N - 7, 7)],
        'BL': [(0, N - 7), (7, N - 7), (7, N), (0, N)],
    }
    src_pts, dst_pts = [], []
    quads = {}
    for k in ['TL', 'TR', 'BL']:
        m = np.linalg.norm(ex if k != 'TL' else ex)
        # 每个 finder 用全局 ex,ey（一次近似）
        q = finder_quad(gray, centers[k], ex, ey, m)
        if q is None:
            print(k, 'quad FAILED')
            continue
        quads[k] = q
        for key, cname in [('tl', 0), ('tr', 1), ('br', 2), ('bl', 3)]:
            src_pts.append(q[key])
            dst_pts.append(canon_corners[k][cname])
        print(k, 'quad ok')

    src = np.array(src_pts, np.float32)
    dst = np.array(dst_pts, np.float32)
    H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 0.6)
    print('H =', H, 'inliers:', mask.sum(), '/', len(mask))
    np.save('H7.npy', H)

    # 采样
    K = 7
    xs = ((np.arange(K) / (K - 1) - 0.5) * 0.36 + 0.5)
    Hinv = np.linalg.inv(H.astype(np.float64))
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
    cv2.imwrite('med7.png', med_u8)
    _, bw = cv2.threshold(med_u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    matrix = (bw < 127).astype(np.uint8)

    def dih(m):
        return [np.rot90(m, k) for k in range(4)] + [np.fliplr(np.rot90(m, k)) for k in range(4)]

    best = min(((int(np.count_nonzero(matrix != d)), i) for i, d in enumerate(dih(gt))))
    print('错误模块:', best[0], '/441 变换idx', best[1])
    for r in range(N):
        print(''.join('##' if matrix[r, c] else '  ' for c in range(N)))
    np.save('matrix7.npy', matrix)


if __name__ == '__main__':
    main()
