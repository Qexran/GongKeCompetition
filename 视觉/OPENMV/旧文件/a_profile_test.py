# -*- coding: utf-8 -*-
"""
A 改法（上下参考色距离剖面过零）的台架验证 —— 用实拍帧对照新旧两种逐列判别
================================================================
用法: python a_profile_test.py [帧路径]
      缺省用 C:\\Users\\Administrator\\Desktop\\uuu.bmp（2026-09-25 那张问题帧）

做什么:
  复刻 main.py（旧）与 main_a.py（新）的逐列判别，在同一帧的同 8 列上各跑一遍:
    旧法  单判别通道(a或b)穿过 own_mid，邻域落差最大者胜
    新法  每条横条的 (a,b) 到该列上/下参考色的距离²之差 f 过零
  参照真值: 该列 95~155 行里 L 均值最小的那一行（黑条最黑处）。

注意:
  - PC 端 LAB 用标准 sRGB→CIELAB(D65) 近似 OpenMV 的转换，绝对值有几单位出入，
    但穿越位置的机理结论不受影响。
  - 帧是画完标记后落盘的: 黄十字/红十字/绿线/青框先按颜色剔除再取均值，
    某条全被标记盖住时退回不剔除。
"""

import sys
from PIL import Image

PATH = sys.argv[1] if len(sys.argv) > 1 else r"C:\Users\Administrator\Desktop\uuu.bmp"

FRAME_W, FRAME_H = 320, 240
TOP_BAND_H, BOT_BAND_H = 60, 35
NCOL, COL_W, STRIP = 8, 9, 12
NSTRIP = FRAME_H // STRIP                     # 20 —— 与 main.py 实际一致
CROSS_MIN_SWING_FRAC = 0.5
CROSS_SWING_DEPTH = 2

im = Image.open(PATH).convert("RGB")
assert im.size == (FRAME_W, FRAME_H), "帧尺寸不对: %s" % (im.size,)
px = im.load()


def is_marker(r, g, b):
    """画上去的四种标记色（RGB565 落盘后有过量化，放宽一点）"""
    return ((r > 170 and g > 170 and b < 90) or    # 黄十字
            (r < 90 and g > 170 and b > 170) or    # 青框
            (r < 90 and g > 170 and b < 90) or     # 绿拟合线
            (r > 170 and g < 90 and b < 90))       # 红十字


def rgb2lab(r, g, b):
    """sRGB → CIELAB(D65)，近似 OpenMV 的转换"""
    def lin(c):
        c /= 255.0
        return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4
    R, G, B = lin(r), lin(g), lin(b)
    X = (0.4124564 * R + 0.3575761 * G + 0.1804375 * B) / 0.95047
    Y = 0.2126729 * R + 0.7151522 * G + 0.0721750 * B
    Z = (0.0193339 * R + 0.1191920 * G + 0.9503041 * B) / 1.08883
    def f(t):
        return t ** (1.0 / 3.0) if t > 0.008856 else 7.787 * t + 16.0 / 116.0
    fx, fy, fz = f(X), f(Y), f(Z)
    return 116 * fy - 16, 500 * (fx - fy), 200 * (fy - fz)


def strip_lab(x, y0, h):
    """该列一条横带的 (L, a, b) 均值（剔除标记像素）"""
    x0 = x - COL_W // 2
    sl = sa = sb = n = 0
    for yy in range(y0, y0 + h):
        for xx in range(x0, x0 + COL_W):
            r, g, b = px[xx, yy]
            if is_marker(r, g, b):
                continue
            L, a, bb = rgb2lab(r, g, b)
            sl += L; sa += a; sb += bb; n += 1
    if n == 0:                       # 整条都被标记盖住 → 退回不剔除
        for yy in range(y0, y0 + h):
            for xx in range(x0, x0 + COL_W):
                L, a, bb = rgb2lab(*px[xx, yy])
                sl += L; sa += a; sb += bb; n += 1
    return sl / n, sa / n, sb / n


def band_medians(y0, h):
    """整幅某横带的 (a, b) 中位数（剔除标记像素）—— 近似 get_statistics 的 median"""
    la, lb = [], []
    for yy in range(y0, y0 + h):
        for xx in range(FRAME_W):
            r, g, b = px[xx, yy]
            if is_marker(r, g, b):
                continue
            _, a, bb = rgb2lab(r, g, b)
            la.append(a); lb.append(bb)
    la.sort(); lb.sort()
    return la[len(la) // 2], lb[len(lb) // 2]


def darkest_row(x, lo=95, hi=155):
    """参照真值: 该列 [lo,hi) 行里 L 均值最小的行（黑条最黑处）"""
    x0 = x - COL_W // 2
    best, brow = None, None
    for yy in range(lo, hi):
        s = n = 0
        for xx in range(x0, x0 + COL_W):
            r, g, b = px[xx, yy]
            if is_marker(r, g, b):
                continue
            s += 0.2126 * r + 0.7152 * g + 0.0722 * b
            n += 1
        if n and (best is None or s / n < best):
            best, brow = s / n, yy
    return brow


def cross_row(prof, mid, min_swing):
    """与 main.py / main_a.py 逐字一致（prof=[(行中心, 值), ...]）"""
    best_row, best_swing = None, 0.0
    n = len(prof)
    for j in range(n - 1):
        b0, b1 = prof[j][1], prof[j + 1][1]
        if (b0 - mid) * (b1 - mid) <= 0 and abs(b1 - b0) > 1e-6:
            seg = [prof[k][1] for k in range(max(0, j - CROSS_SWING_DEPTH),
                                             min(n, j + 2 + CROSS_SWING_DEPTH))]
            swing = max(seg) - min(seg)
            if swing > best_swing:
                best_swing = swing
                best_row = prof[j][0] + (mid - b0) / (b1 - b0) * STRIP
    if best_row is None or best_swing < min_swing:
        return None
    return best_row


# ---- 全局两侧色心 → margin / 通道（旧法要用；新法只用 margin 当闸的下限） ----
a_up, b_up = band_medians(0, TOP_BAND_H)
a_dn, b_dn = band_medians(FRAME_H - BOT_BAND_H, BOT_BAND_H)
margin = max(abs(a_up - a_dn), abs(b_up - b_dn))
is_a = abs(a_up - a_dn) >= abs(b_up - b_dn)
print("top band a=%+.1f b=%+.1f | bottom band a=%+.1f b=%+.1f | margin=%.1f -> sim channel=%c"
      % (a_up, b_up, a_dn, b_dn, margin, 'a' if is_a else 'b'))
print("(真机上 margin/通道以 OpenMV 的 LAB 为准，此处数值仅供对照)")
print()
print("col   old_row  new_row  darkest(ref)")
olds, news, refs = [], [], []
prof180 = None
for i in range(NCOL):
    x = int(FRAME_W * (i + 0.5) / NCOL)
    prof = []
    for j in range(NSTRIP):
        L, a, b = strip_lab(x, j * STRIP, STRIP)
        prof.append((j * STRIP + STRIP / 2.0, a, b, L))
    if x == 180:
        prof180 = prof

    # ---- 旧法: 单通道过 own_mid ----
    cv = [p[1] if is_a else p[2] for p in prof]
    b_top = (cv[0] + cv[1] + cv[2]) / 3.0
    b_bot = (cv[-1] + cv[-2] + cv[-3]) / 3.0
    own_mid = (b_top + b_bot) / 2.0
    own_spread = abs(b_top - b_bot)
    r_old = cross_row([(p[0], v) for p, v in zip(prof, cv)], own_mid,
                      CROSS_MIN_SWING_FRAC * max(own_spread, margin))

    # ---- 新法: (a,b) 到上下参考色的距离²之差 f 过零 ----
    ta = (prof[0][1] + prof[1][1] + prof[2][1]) / 3.0
    tb = (prof[0][2] + prof[1][2] + prof[2][2]) / 3.0
    ba = (prof[-1][1] + prof[-2][1] + prof[-3][1]) / 3.0
    bb = (prof[-1][2] + prof[-2][2] + prof[-3][2]) / 3.0
    da, db = ba - ta, bb - tb
    d2 = da * da + db * db
    f = [((p[1] - ta) ** 2 + (p[2] - tb) ** 2)
         - ((p[1] - ba) ** 2 + (p[2] - bb) ** 2) for p in prof]
    r_new = cross_row([(p[0], f[j]) for j, p in enumerate(prof)], 0.0,
                      CROSS_MIN_SWING_FRAC * max(2.0 * d2, margin * margin))

    ref = darkest_row(x)
    fm = lambda r: ("%6.1f" % r) if r is not None else "  rej"
    print("%3d   %s    %s     %d" % (x, fm(r_old), fm(r_new), ref))
    if r_old is not None: olds.append(r_old)
    if r_new is not None: news.append(r_new)
    if ref is not None: refs.append(ref)

sp = lambda v: (max(v) - min(v)) if v else 0
print()
print("old: %d/%d pts, row spread=%.1f px | new: %d/%d pts, row spread=%.1f px | ref spread=%.1f px"
      % (len(olds), NCOL, sp(olds), len(news), NCOL, sp(news), sp(refs)))

if prof180:
    print()
    print("x=180 column profile (strip_y, a, b, f) -- 褶皱阴影边在 a/b 里可见, f 必须全程 <0:")
    ta = (prof180[0][1] + prof180[1][1] + prof180[2][1]) / 3.0
    tb = (prof180[0][2] + prof180[1][2] + prof180[2][2]) / 3.0
    ba = (prof180[-1][1] + prof180[-2][1] + prof180[-3][1]) / 3.0
    bb = (prof180[-1][2] + prof180[-2][2] + prof180[-3][2]) / 3.0
    for p in prof180:
        fv = ((p[1] - ta) ** 2 + (p[2] - tb) ** 2) - ((p[1] - ba) ** 2 + (p[2] - bb) ** 2)
        print("  y=%5.1f  a=%+6.2f  b=%+6.2f  f=%+8.1f" % (p[0], p[1], p[2], fv))
