# -*- coding: utf-8 -*-
"""原型2：基于 finder 图案轮廓检测的精确定位"""
import cv2
import numpy as np

PHOTO = r'C:\Users\Administrator\Desktop\跟顺眼的二维码.jpg'


def imread_u(path):
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


def find_finder_centers(gray):
    """通过嵌套轮廓结构找 finder 图案（1:1:3:1:1 环结构）"""
    results = []
    for blk in [51, 75, 101]:
        for C in [5, 10, 15]:
            bw = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                       cv2.THRESH_BINARY, blk, C)
            for inv in [False, True]:
                b = bw if not inv else 255 - bw
                cnts, hier = cv2.findContours(b, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
                if hier is None:
                    continue
                hier = hier[0]
                for i, c in enumerate(cnts):
                    a = cv2.contourArea(c)
                    if a < 100:
                        continue
                    # 找子轮廓面积比 ~ 25/49 的嵌套对
                    child = hier[i][2]
                    if child < 0:
                        continue
                    cc = cnts[child]
                    ca = cv2.contourArea(cc)
                    if ca <= 0:
                        continue
                    ratio = ca / a
                    if 0.4 < ratio < 0.65:  # 25/49 = 0.51
                        # 进一步检查孙轮廓 9/25 = 0.36
                        gchild = hier[child][2]
                        if gchild >= 0:
                            ga = cv2.contourArea(cnts[gchild])
                            if ga / ca > 0.2:
                                M = cv2.moments(c)
                                if M['m00'] > 0:
                                    cx, cy = M['m10'] / M['m00'], M['m01'] / M['m00']
                                    w = np.sqrt(a)
                                else:
                                    continue
                                gM = cv2.moments(cnts[gchild])
                                gx, gy = gM['m10'] / gM['m00'], gM['m01'] / gM['m00']
                                # 外环与内核中心应几乎重合
                                if np.hypot(cx - gx, cy - gy) < w * 0.15:
                                    results.append((cx, cy, w))
    # 聚合
    if not results:
        return []
    arr = np.array([(r[0], r[1], r[2]) for r in results])
    clustered = []
    used = np.zeros(len(arr), bool)
    for i in range(len(arr)):
        if used[i]:
            continue
        grp = [i]
        used[i] = True
        for j in range(i + 1, len(arr)):
            if used[j]:
                continue
            if np.hypot(arr[i, 0] - arr[j, 0], arr[i, 1] - arr[j, 1]) < 15:
                grp.append(j)
                used[j] = True
        sub = arr[grp]
        clustered.append((sub[:, 0].mean(), sub[:, 1].mean(), sub[:, 2].mean()))
    return clustered


def main():
    img = imread_u(PHOTO)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    centers = find_finder_centers(gray)
    print(f'found {len(centers)} finder candidates')
    for c in sorted(centers, key=lambda c: -c[2])[:12]:
        print(f'  center=({c[0]:.1f},{c[1]:.1f}) size={c[2]:.1f}')
    vis = img.copy()
    for c in centers:
        cv2.circle(vis, (int(c[0]), int(c[1])), 6, (0, 0, 255), 2)
    cv2.imwrite('finders.png', vis)


if __name__ == '__main__':
    main()
