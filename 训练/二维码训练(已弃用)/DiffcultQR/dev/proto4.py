# -*- coding: utf-8 -*-
"""原型4：6点单应性拟合 + remap 密集采样 + 中值重建"""
import cv2
import numpy as np

PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'
N = 21


def imread_u(path):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


def finder_contours(gray):
    """返回 [(outer_contour, ...)] 三个 finder 的外轮廓"""
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
                    found.append((c, cnts[child], cnts[gchild], blk, C, polarity))
    return found


def pick_three(found):
    """从候选中聚类出三个 finder 的外轮廓"""
    items = []
    for c, ch, gc, *_ in found:
        M = cv2.moments(c)
        if M['m00'] == 0:
            continue
        items.append((M['m10'] / M['m00'], M['m01'] / M['m00'], c, gc))
    if not items:
        return None
    # 聚类
    clusters = []
    for it in items:
        placed = False
        for cl in clusters:
            if np.hypot(it[0] - cl['cx'] / cl['n'], it[1] - cl['cy'] / cl['n']) < 20:
                cl['cx'] += it[0]
                cl['cy'] += it[1]
                cl['n'] += 1
                cl['contours'].append(it[2])
                cl['cores'].append(it[3])
                placed = True
                break
        if not placed:
            clusters.append(dict(cx=it[0], cy=it[1], n=1, contours=[it[2]], cores=[it[3]]))
    for cl in clusters:
        cl['center'] = np.array([cl['cx'] / cl['n'], cl['cy'] / cl['n']])
    # 取面积最大的三个（真 finder 比误检大）
    clusters.sort(key=lambda cl: -np.mean([cv2.contourArea(c) for c in cl['contours']]))
    return clusters[:3]


def inner_corner(contour, center, qr_center):
    """finder 外轮廓上朝向 QR 中心的角点"""
    d = qr_center - center
    d = d / (np.linalg.norm(d) + 1e-9)
    pts = contour.reshape(-1, 2).astype(np.float64)
    proj = (pts - center) @ d
    return pts[np.argmax(proj)]


def fit_homography(finders, N):
    """finders: {'TL': (center, inner_corner), ...}"""
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


def sample_matrix(gray, H, N, upscale=8):
    """用 remap 把每个模块中心 5x5 采样点拉出来，中值 -> Otsu -> 矩阵"""
    scale = upscale  # 每模块 upscale^2 个采样点（中心40%区域内）
    half = int(upscale * 0.2)
    xs = (np.arange(upscale) / upscale - 0.5) * 0.4 + 0.5  # 0.3..0.7
    samples = np.zeros((N, N, upscale, upscale), np.float32)
    Hinv = np.linalg.inv(H)
    for r in range(N):
        for c in range(N):
            uu = c + xs
            vv = r + xs
            U, V = np.meshgrid(uu, vv)
            ones = np.ones_like(U)
            pts = np.stack([U, V, ones], -1).reshape(-1, 3)
            imgp = (Hinv @ pts.T).T
            imgp = imgp[:, :2] / imgp[:, 2:3]
            mapx = imgp[:, 0].astype(np.float32).reshape(upscale, upscale)
            mapy = imgp[:, 1].astype(np.float32).reshape(upscale, upscale)
            samples[r, c] = cv2.remap(gray, mapx, mapy, cv2.INTER_LINEAR)
    med = np.median(samples, axis=(2, 3))  # N×N 中值图
    med_u8 = np.clip(med, 0, 255).astype(np.uint8)
    _, bw = cv2.threshold(med_u8, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    matrix = (bw < 127).astype(np.uint8)  # 1=黑
    return matrix, med_u8


def render_qr(matrix, scale=20, quiet=4):
    N = matrix.shape[0]
    S = (N + quiet * 2) * scale
    img = np.ones((S, S), np.uint8) * 255
    for r in range(N):
        for c in range(N):
            if matrix[r, c]:
                img[(quiet + r) * scale:(quiet + r + 1) * scale,
                    (quiet + c) * scale:(quiet + c + 1) * scale] = 0
    return img


def decode_any(img):
    import zxingcpp
    res = zxingcpp.read_barcodes(img)
    if res:
        return f'zxing: {res[0].text}'
    from pyzbar import pyzbar
    r = pyzbar.decode(img)
    if r:
        return f'pyzbar: {r[0].data.decode()}'
    s = cv2.QRCodeDetector().detectAndDecode(img)[0]
    return f'ocv: {s}' if s else None


def main():
    img = imread_u(PHOTO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    found = finder_contours(gray)
    print('candidates:', len(found))
    three = pick_three(found)
    assert three and len(three) == 3, f'got {len(three) if three else 0} finders'
    centers = [cl['center'] for cl in three]
    qr_c = np.mean(centers, axis=0)
    # TL = 直角顶点
    v01 = centers[1] - centers[0]
    v02 = centers[2] - centers[0]
    dots = []
    for i in range(3):
        a = centers[(i + 1) % 3] - centers[i]
        b = centers[(i + 2) % 3] - centers[i]
        dots.append(abs(np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-9)))
    def cross2(a, b):
        return a[0] * b[1] - a[1] * b[0]

    tl_i = int(np.argmin(dots))
    rest = [i for i in range(3) if i != tl_i]
    TLc = centers[tl_i]
    # TR/BL 定向: cross>0
    if cross2(centers[rest[1]] - TLc, centers[rest[0]] - TLc) > 0:
        tr_i, bl_i = rest
    else:
        bl_i, tr_i = rest
    TRc, BLc = centers[tr_i], centers[bl_i]
    print('TL', TLc, 'TR', TRc, 'BL', BLc)

    # 每个 finder 的内角：用其聚类里最大的轮廓
    def biggest(cl):
        return max(cl['contours'], key=cv2.contourArea)

    finders = {
        'TL': (TLc, inner_corner(biggest(three[tl_i]), TLc, qr_c)),
        'TR': (TRc, inner_corner(biggest(three[tr_i]), TRc, qr_c)),
        'BL': (BLc, inner_corner(biggest(three[bl_i]), BLc, qr_c)),
    }
    H = fit_homography(finders, N)
    print('H=', H)

    matrix, med = sample_matrix(gray, H, N)
    np.save('matrix.npy', matrix)
    cv2.imwrite('med.png', med)
    for row in matrix:
        print(''.join('##' if v else '  ' for v in row))
    rebuilt = render_qr(matrix)
    cv2.imwrite('rebuilt.png', rebuilt)
    print('decode:', decode_any(rebuilt))


if __name__ == '__main__':
    main()
