# -*- coding: utf-8 -*-
"""原型：透视矫正 + 网格重采样 + 重建清晰二维码"""
import cv2
import numpy as np

PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'


def imread_u(path):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


def order_corners(pts):
    """按 TL, TR, BR, BL 排序"""
    pts = pts.reshape(-1, 2).astype(np.float32)
    s = pts.sum(axis=1)
    d = np.diff(pts, axis=1).ravel()  # y-x
    tl = pts[np.argmin(s)]
    br = pts[np.argmax(s)]
    tr = pts[np.argmin(d)]
    bl = pts[np.argmax(d)]
    return np.array([tl, tr, br, bl], dtype=np.float32)


def main():
    img = imread_u(PHOTO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    det = cv2.QRCodeDetector()
    ok, pts = det.detect(img)
    assert ok, 'detect failed'
    corners = order_corners(pts)
    print('ordered corners:\n', corners)

    # 第一次粗矫正到 420px
    N = 21
    S = 420
    dst = np.array([[0, 0], [S, 0], [S, S], [0, S]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(corners, dst)
    warp = cv2.warpPerspective(gray, M, (S, S), flags=cv2.INTER_CUBIC)
    cv2.imwrite('warp1.png', warp)

    # 在矫正图上精确定位三个 finder 的中心与模块尺寸
    # 用 run-length 分析左上 finder：从 (0,0) 开始
    def finder_center(box):  # box=(x0,y0,x1,y1) 大致区域
        sub = warp[box[1]:box[3], box[0]:box[2]]
        _, b = cv2.threshold(sub, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        b = 255 - b  # 黑模块 -> 白
        Mch, cnts, _ = cv2.findContours(b, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        # 取嵌套结构：外环 + 内核的公共中心
        best = None
        for c in cnts:
            (cx, cy), r = cv2.minEnclosingCircle(c)
            area = cv2.contourArea(c)
            if area < 50:
                continue
            if best is None or area < best[2]:  # 最小的实心核
                pass
        # 简化：用矩
        Mm = cv2.moments(b)
        # 这里改为分层质心平均
        return sub

    # 更稳的做法：直接分析行/列 run
    _, bw = cv2.threshold(warp, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    # 反色使黑模块为白(255)，方便统计
    bwq = 255 - bw if bw[0, 0] > 127 else bw  # 边角应为白，保证背景为0
    if bwq[0, 0] > 0:
        bwq = 255 - bw

    # 顶部 timing 行在 y=6*mod 处；先估模块尺寸：找第一列的 run
    col = bwq[:, 3]  # 第3列接近左边缘但避开
    runs = []
    cur = col[0] > 127
    n = 1
    for v in col[1:] > 127:
        if v == cur:
            n += 1
        else:
            runs.append((cur, n))
            cur = v
            n = 1
    runs.append((cur, n))
    print('col runs:', runs[:10])
    dark_runs = [n for v, n in runs if v]
    print('dark run lengths:', dark_runs[:10])


if __name__ == '__main__':
    main()
