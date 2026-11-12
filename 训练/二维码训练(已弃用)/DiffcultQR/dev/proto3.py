# -*- coding: utf-8 -*-
"""原型3：仿射基矫正 + 迭代精化 + 网格采样 + 重建解码"""
import cv2
import numpy as np

PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'


def imread_u(path):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


def binarize_multi(gray):
    """多种二值化，供投票"""
    outs = []
    outs.append(cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1])
    for blk in [25, 41, 61]:
        for C in [5, 12]:
            outs.append(cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                              cv2.THRESH_BINARY, blk, C))
    return outs


def sample_grid(warped, N, votes_bg_white=True):
    """中心区域中值采样 N×N 模块矩阵。返回 0/1 矩阵（1=黑）"""
    S = warped.shape[0]
    m = S / N
    binims = binarize_multi(warped)
    acc = np.zeros((N, N), dtype=np.float32)
    frac = 0.36  # 采样中心 60% 区域
    for b in binims:
        for r in range(N):
            for c in range(N):
                x0 = int((c + 0.5 - frac / 2) * m)
                x1 = int((c + 0.5 + frac / 2) * m)
                y0 = int((r + 0.5 - frac / 2) * m)
                y1 = int((r + 0.5 + frac / 2) * m)
                cell = b[y0:y1, x0:x1]
                dark_ratio = np.mean(cell < 127)
                acc[r, c] += dark_ratio
    acc /= len(binims)
    matrix = (acc > 0.5).astype(np.uint8)
    if votes_bg_white is False:
        matrix = 1 - matrix
    return matrix, acc


def render_qr(matrix, scale=20, quiet=4):
    N = matrix.shape[0]
    S = (N + quiet * 2) * scale
    img = np.ones((S, S), dtype=np.uint8) * 255
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
        return f'zxing:{res[0].text}'
    try:
        from pyzbar import pyzbar
        r = pyzbar.decode(img)
        if r:
            return f'pyzbar:{r[0].data.decode()}'
    except ImportError:
        pass
    d = cv2.QRCodeDetector()
    s = d.detectAndDecode(img)[0]
    if s:
        return f'ocv:{s}'
    return None


def refine_warp(gray, corners, N, S):
    """warp 后在图内重新定位 finder，校正残差，二次 warp"""
    M = cv2.getPerspectiveTransform(corners.astype(np.float32),
                                    np.array([[0, 0], [S, 0], [S, S], [0, S]], dtype=np.float32))
    warp = cv2.warpPerspective(gray, M, (S, S), flags=cv2.INTER_CUBIC)
    return warp


def main():
    img = imread_u(PHOTO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    TL = np.array([582.1, 507.7])
    TR = np.array([839.1, 289.3])
    BL = np.array([713.6, 678.3])
    N = 21
    S = 630  # 21*30

    ex = (TR - TL) / (N - 7)
    ey = (BL - TL) / (N - 7)
    k = 3.5
    c00 = TL - k * ex - k * ey
    c10 = TR + k * ex - k * ey
    c11 = TL + (N - k) * ex + (N - k) * ey
    c01 = BL - k * ex + k * ey
    corners = np.array([c00, c10, c11, c01], dtype=np.float32)
    print('corners:\n', corners)

    warp = refine_warp(gray, corners, N, S)
    cv2.imwrite('warp.png', warp)

    for inv in [False, True]:
        matrix, acc = sample_grid(warp, N, votes_bg_white=not inv)
        print(f'--- inv={inv} ---')
        # 打印矩阵
        for row in matrix:
            print(''.join('##' if v else '  ' for v in row))
        cv2.imwrite(f'rebuilt_{"inv" if inv else "norm"}.png', render_qr(matrix))
        r = decode_any(render_qr(matrix))
        print('decode:', r)
        if r:
            return


if __name__ == '__main__':
    main()
