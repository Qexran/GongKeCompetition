# -*- coding: utf-8 -*-
"""原型5：加入与原图 ground truth 的对比诊断 + warp 空间迭代精化"""
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
    # 580px / 29 modules(21+8 quiet) = 20px 模块，quiet=4
    m = 20
    q = 4
    mat = np.zeros((N, N), np.uint8)
    for r in range(N):
        for c in range(N):
            cell = bw[(q + r) * m + 6:(q + r + 1) * m - 6, (q + c) * m + 6:(q + c + 1) * m - 6]
            mat[r, c] = 1 if np.mean(cell < 127) > 0.5 else 0
    return mat


def dihedral(mat):
    outs = []
    for k in range(4):
        r = np.rot90(mat, k)
        outs.append(r)
        outs.append(np.fliplr(r))
    return outs


def compare(sampled, gt):
    best = None
    for i, d in enumerate(dihedral(gt)):
        diff = np.int32(sampled) - np.int32(d)
        nerr = int(np.count_nonzero(diff))
        if best is None or nerr < best[0]:
            best = (nerr, i, d)
    return best


def finder_contours(gray):
    found = []
    for blk in [51, 75, 101]:
        for C in [5, 10, 15]:
            bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY, blk, C)
            for polarity in [0, 1]:
                b = bw if polarity == 0 else 255 - bw
                cnts, hier = cv2.findContours(b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if hier is None:
                    continue
                hier = hier[0]
                for i, c in enumerate(cnts):
                    a = cv2.contourArea(c)
                    if a < 400:
                        continue
                    child = hier[i][2]
                    if child < 0:
                        continue
                    ca = cv2.contourArea(cnts[child])
                    if ca <= 0 or not (0.4 < ca / a < 0.65):
                        continue
                    gchild = hier[child][2]
                    if gchild < 0:
                        continue
                    ga = cv2.contourArea(cnts[gchild])
                    if ga / ca < 0.2:
                        continue
                    found.append(c)
    return found


def cluster_finders(contours):
    items = []
    for c in contours:
        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        items.append((np.array([M['m10'] / M['m00'], M['m01'] / M['m00']]), c))
    clusters = []
    for ctr, c in items:
        placed = False
        for cl in clusters:
            if np.hypot(ctr[0] - cl['sx'] / cl['n'], ctr[1] - cl['sy'] / cl['n']) < 20:
                cl['sx'] += ctr[0]
                cl['sy'] += ctr[1]
                cl['n'] += 1
                cl['contours'].append(c)
                placed = True
                break
        if not placed:
            clusters.append(dict(sx=ctr[0], sy=ctr[1], n=1, contours=[c]))
    for cl in clusters:
        cl['center'] = np.array([cl['sx'] / cl['n'], cl['sy'] / cl['n']])
    clusters.sort(key=lambda cl: -np.mean([cv2.contourArea(c) for c in cl['contours']]))
    return clusters[:3]


def inner_corner(contour, center, qr_center):
    d = qr_center - center
    d = d / (np.linalg.norm(d) + 1e-9)
    pts = contour.reshape(-1, 2).astype(np.float64)
    proj = (pts - center) @ d
    return pts[np.argmax(proj)]


def fit_homography(finders, N):
    canon = {
        'TL': ((3.5, 3.5), (7.0, 7.0)),
        'TR': ((N - 3.5, 3.5), (N - 7.0, 7.0)),
        'BL': ((3.5, N - 3.5), (7.0, N - 7.0)),
    }
    src, dst = [], []
    for k in ['TL', 'TR', 'BL']:
        c, ic = finders[k]
        src += [c, ic]
        dst += [canon[k][0], canon[k][1]]
    H, _ = cv2.findHomography(np.array(src, np.float32), np.array(dst, np.float32))
    return H


def refine_in_warp(gray, H, N, iters=3):
    """在 warp 图内重新定位 finder 中心，修正 H"""
    S = N * 30
    dst = np.array([[0, 0], [S, 0], [S, S], [0, S]], np.float32)
    for it in range(iters):
        M = H.astype(np.float64)
        warp = cv2.warpPerspective(gray, M, (S, S), flags=cv2.INTER_CUBIC)
        # 期望的 finder 中心（warp 后应为正方形位置）
        m = S / N
        expect = {'TL': (3.5 * m, 3.5 * m), 'TR': ((N - 3.5) * m, 3.5 * m), 'BL': (3.5 * m, (N - 3.5) * m)}
        # 模板匹配精定位
        meas = {}
        tpl_half = int(3.5 * m)
        tpl = np.zeros((2 * tpl_half, 2 * tpl_half), np.uint8)
        cv2.rectangle(tpl, (int(0.5 * m), int(0.5 * m)), (int(6.5 * m), int(6.5 * m)), 255, -1)
        cv2.rectangle(tpl, (int(1.5 * m), int(1.5 * m)), (int(5.5 * m), int(5.5 * m)), 0, -1)
        cv2.rectangle(tpl, (int(2.5 * m), int(2.5 * m)), (int(4.5 * m), int(4.5 * m)), 255, -1)
        for k, (ex, ey) in expect.items():
            x0, y0 = int(ex - tpl_half - 2 * m), int(ey - tpl_half - 2 * m)
            side = 2 * tpl_half + int(4 * m)
            if x0 < 0 or y0 < 0 or x0 + side > S or y0 + side > S:
                meas[k] = np.array([ex, ey])
                continue
            patch = warp[y0:y0 + side, x0:x0 + side]
            _, pb = cv2.threshold(patch, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            if pb.mean() < 128:
                pb = 255 - pb
            res = cv2.matchTemplate(pb, tpl, cv2.TM_CCOEFF_NORMED)
            _, _, _, loc = cv2.minMaxLoc(res)
            meas[k] = np.array([x0 + loc[0] + tpl_half, y0 + loc[1] + tpl_half], np.float64)
        # 用三个实测中心 + 三个内角重新拟合 -> 但warp里内角就是 (7m,7m) 等
        canon = {
            'TL': ((3.5, 3.5), (7.0, 7.0)),
            'TR': ((N - 3.5, 3.5), (N - 7.0, 7.0)),
            'BL': ((3.5, N - 3.5), (7.0, N - 7.0)),
        }
        src, dstp = [], []
        for k in ['TL', 'TR', 'BL']:
            mc = meas[k]
            # 内角估计: 中心 + 3.5m 朝 QR 中心方向的对角
            cen = np.array([S / 2, S / 2], np.float64)
            dd = cen - mc
            dd /= (np.linalg.norm(dd) + 1e-9)
            mic = mc + dd * 3.5 * m * np.sqrt(2) / np.sqrt(2)  # 对角距离 = 3.5*sqrt(2)*m 沿单位对角
            mic = mc + dd * 3.5 * m * 1.4142 / 1.4142  # 简化: 沿对角方向 3.5*sqrt(2)*m
            src += [mc, mic]
            dstp += [canon[k][0], canon[k][1]]
        Hc, _ = cv2.findHomography(np.array(src, np.float32), np.array(dstp, np.float32))
        if Hc is None:
            break
        # Hc: warp像素 -> module坐标；M: photo -> warp像素
        H = Hc @ M
        print(f'  iter{it}: meas TL={meas["TL"].astype(int)} TR={meas["TR"].astype(int)} BL={meas["BL"].astype(int)} expect {int(3.5*m)},{int(3.5*m)}...')
    return H


def sample_matrix(gray, H, N, upscale=8):
    xs = (np.arange(upscale) / (upscale - 1) - 0.5) * 0.4 + 0.5
    samples = np.zeros((N, N, upscale, upscale), np.float32)
    Hinv = np.linalg.inv(H)
    for r in range(N):
        for c in range(N):
            U, V = np.meshgrid(c + xs, r + xs)
            ones = np.ones_like(U)
            pts = np.stack([U, V, ones], -1).reshape(-1, 3)
            imgp = (Hinv @ pts.T).T
            imgp = imgp[:, :2] / imgp[:, 2:3]
            mapx = imgp[:, 0].astype(np.float32).reshape(upscale, upscale)
            mapy = imgp[:, 1].astype(np.float32).reshape(upscale, upscale)
            samples[r, c] = cv2.remap(gray, mapx, mapy, cv2.INTER_LINEAR)
    med = np.median(samples, axis=(2, 3))
    med_u8 = np.clip(med, 0, 255).astype(np.uint8)
    _, bw = cv2.threshold(med_u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    return (bw < 127).astype(np.uint8), med_u8


def main():
    img = imread_u(PHOTO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gt = ground_truth()
    np.save('gt.npy', gt)

    contours = finder_contours(gray)
    three = cluster_finders(contours)
    centers = [cl['center'] for cl in three]
    qr_c = np.mean(centers, axis=0)
    dots = []
    for i in range(3):
        a = centers[(i + 1) % 3] - centers[i]
        b = centers[(i + 2) % 3] - centers[i]
        dots.append(abs(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)))
    tl_i = int(np.argmin(dots))
    rest = [i for i in range(3) if i != tl_i]

    def cross2(a, b):
        return a[0] * b[1] - a[1] * b[0]

    TLc = centers[tl_i]
    if cross2(centers[rest[1]] - TLc, centers[rest[0]] - TLc) > 0:
        tr_i, bl_i = rest
    else:
        bl_i, tr_i = rest
    finders = {
        'TL': (centers[tl_i], inner_corner(max(three[tl_i]['contours'], key=cv2.contourArea), centers[tl_i], qr_c)),
        'TR': (centers[tr_i], inner_corner(max(three[tr_i]['contours'], key=cv2.contourArea), centers[tr_i], qr_c)),
        'BL': (centers[bl_i], inner_corner(max(three[bl_i]['contours'], key=cv2.contourArea), centers[bl_i], qr_c)),
    }
    H = fit_homography(finders, N)
    H = refine_in_warp(gray, H, N, iters=3)
    np.save('H.npy', H)

    matrix, med = sample_matrix(gray, H, N)
    cv2.imwrite('med.png', med)
    nerr, dih, aligned = compare(matrix, gt)
    print(f'与原图对比: 错误模块数 = {nerr}/441, 最佳变换 idx={dih}')
    errmap = (np.int32(matrix) - np.int32(aligned)) != 0
    for r in range(N):
        print(''.join('XX' if errmap[r, c] else ('##' if matrix[r, c] else '  ') for c in range(N)))
    np.save('matrix.npy', matrix)


if __name__ == '__main__':
    main()
