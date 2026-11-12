# -*- coding: utf-8 -*-
"""靶心定位算法 —— **这份就是要搬到 Pi 的 tasks/_target.py 的核心**：纯算法、不碰串口/相机。

四个候选方法，为的是横向对比（evaluate.py 一起量误差）：

| 方法 | 怎么定圆心 | 谁在用 |
|---|---|---|
| `m_centroid` | 外环**色块质心** | **explorer 省一队**（`find_blobs` → `blob.cx()`） |
| `m_hough` | `cv2.HoughCircles` | 两个物流搬运参考项目 |
| `m_outfit` | **外边界**（色环外缘）最小二乘拟合 | — |
| `m_inner` | **内边界**（色环↔白心）最小二乘拟合 | 实拍支持的**最强单特征** |
| `m_shared` | 以内边界为起点、**共享圆心**联合细化 | 我们主张的 |
| `m_shared_sub` | 上者 + 径向剖面亚像素 | 我们主张的+ |

**实拍定下的几何（2026-09，现场帧，见 synth.py 的说明）**：手工/3D 打印样件
**白心上没有 8/9/10 环**，可用圆边只有两条 —— 外缘（手工贴的，**不是圆**，拟合椭圆残差
std 2.15px/最大 5.8px）和内边界（**干净的圆**，残差 0.44px，像素化下限约 0.3px）。
所以「以哪条边为圆心依据」是个真问题，不是风格偏好 —— 这一组方法就是为回答它而排的。

为什么是「对比」而不是直接上最复杂的：**如果简单方法在真实条件下已经够用，就不该上复杂的。**
explorer 拿省一说明质心法可能够 —— 但这要用数据回答，不能靠直觉。evaluate.py 会给出
每个方法在各退化条件下的误差，以及换算到靶面上的毫米数（对着 10 环半径 7.5mm 判）。

坐标一律是**全分辨率像素**（与固件 BALL_CX_CENTER / 串口协议同一约定），
所以将来 Pi 侧降采样提速时必须按 1/scale 还原，别把降采样后的坐标发出去。
"""
import math

import cv2
import numpy as np

# 三色 HSV 阈值带（H 0~179）。红色跨 0 度所以有两段。
# 带宽给得宽是**故意的**：合成图的印刷色和实拍会有偏差，带宽紧的阈值在实拍上必炸。
COLOR_BANDS = {
    "red":   (((0, 9), (171, 179)), 70, 55),
    "green": (((42, 82),),          55, 45),
    "blue":  (((95, 132),),         55, 45),
}

# 靶盘设计比例：色环内边界 Ø55 / 外缘 Ø100（命题图 7）。
# 只用来把「内圆」的拟合半径折算成**外圆等效半径**，好让各方法返回的 r_px 同口径 ——
# 视差公式 `offset = h·r_px/50` 是按外缘半径定义的（见 tools/target_offline/README）。
R_INNER_OVER_OUTER = 27.5 / 50.0


# ==================== 粗定位：按颜色找靶盘 ====================

def color_mask(img, color_name):
    """按颜色找外环区域。返回 uint8 掩膜（255 = 外环色）。

    ⚠️ 必须在 int16 里比：H 是 uint8，`h >= 171` 之后再 `h <= 179` 没问题，但
    `(h >= 0)` 这种写法在回绕时会被静默判错，所以统一升位再算。
    """
    bands, s_min, v_min = COLOR_BANDS[color_name]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h = hsv[:, :, 0].astype(np.int16)
    s = hsv[:, :, 1].astype(np.int16)
    v = hsv[:, :, 2].astype(np.int16)

    m = np.zeros(h.shape, bool)
    for lo, hi in bands:
        m |= ((h >= lo) & (h <= hi))
    m &= (s >= s_min) & (v >= v_min)

    mask = (m.astype(np.uint8) * 255)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)    # 去零散噪点
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)   # 补盘面高光/贴纸缝
    return mask


def coarse_target(img, color_name):
    """→ {'cx','cy','r','mask','contour'}；找不到回 None。

    ⚠️ `r` 是**外缘视半径**的粗估（外接圆），只用来给后续方法定 ROI 和半径带；
    它本身有 1~2px 的正偏差（外接圆必然包含全部边界像素）。
    """
    mask = color_mask(img, color_name)
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    if cv2.contourArea(c) < 50.0:
        return None
    (cx, cy), r = cv2.minEnclosingCircle(c)
    return {"cx": float(cx), "cy": float(cy), "r": float(r),
            "mask": mask, "contour": c}


# ==================== 单圆最小二乘拟合（Kåsa） ====================

def fit_circle_lsq(pts):
    """点集 → (cx, cy, r)。最小化 Σ(‖p−c‖² − r²)²（Kåsa 代数解，闭式）。

    代数解对离群点敏感，所以外面套一层按半径残差 MAD 的剔除。
    """
    p = np.asarray(pts, np.float64).reshape(-1, 2)
    if len(p) < 3:
        return None
    x, y = p[:, 0], p[:, 1]
    A = np.column_stack([x, y, np.ones(len(p))])
    b = x * x + y * y
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return None
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    r2 = sol[2] + cx * cx + cy * cy
    if r2 <= 0:
        return None
    return float(cx), float(cy), float(np.sqrt(r2))


def fit_circle_robust(pts, rounds=3):
    """Kåsa + 按半径残差 MAD 剔除离群点, 迭代 rounds 轮。"""
    p = np.asarray(pts, np.float64).reshape(-1, 2)
    best = None
    for _ in range(max(1, rounds)):
        if len(p) < 3:
            break
        f = fit_circle_lsq(p)
        if f is None:
            break
        cx, cy, r = f
        best = f
        d = np.abs(np.hypot(p[:, 0] - cx, p[:, 1] - cy) - r)
        mad = np.median(np.abs(d - np.median(d))) + 1e-6
        keep = d < max(2.5 * 1.4826 * mad, 1.0)
        if keep.all() or keep.sum() < 3:
            break
        p = p[keep]
    return best


# ==================== 共享圆心同心环拟合 ====================

def _edge_points(gray, c0, r0, pct=90.0, max_pts=30000, pad=1.12, radial_cos=0.75):
    """在 (c0, r0) 的邻域里取边缘点（梯度幅值 + 同心环距离带过滤）。

    为什么用**梯度幅值**而不是 Canny/Hough：同心环里既有"黑线↔灰环"的强边、也有
    盘缘"色块↔背景"的边，Canny 的双阈值会按局部情况挑掉一部分，而同心拟合恰恰
    **不关心是哪个环的边**，只要"到中心的距离是常数"就成立。所以宁滥勿缺，去噪交给
    后面的鲁棒迭代。

    ⚠️ **先裁 ROI 再做 Sobel**：全幅 1920×1080 的梯度有 99% 是被扔掉的背景，
    在 Pi 上这是纯粹浪费（实测全幅做一次 ~40ms，裁完只剩几毫秒）。返回的坐标已加回偏移。
    """
    h, w = gray.shape
    x0 = max(0, int(c0[0] - pad * r0))
    x1 = min(w, int(c0[0] + pad * r0) + 1)
    y0 = max(0, int(c0[1] - pad * r0))
    y1 = min(h, int(c0[1] + pad * r0) + 1)
    if (x1 - x0) < 8 or (y1 - y0) < 8:
        return None
    sub = gray[y0:y1, x0:x1]

    gx = cv2.Sobel(sub, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(sub, cv2.CV_32F, 0, 1, ksize=3)
    mag = cv2.magnitude(gx, gy)

    sh, sw = mag.shape
    yy, xx = np.mgrid[0:sh, 0:sw].astype(np.float32)
    dist = np.hypot(xx + x0 - c0[0], yy + y0 - c0[1])

    band = (dist > 0.12 * r0) & (dist < 1.10 * r0)   # 丢掉中心块与远处背景
    if not band.any():
        return None

    vals = mag[band]
    thr = max(np.percentile(vals, pct), 0.15 * float(vals.max()) + 1e-6)
    sel = band & (mag >= thr)
    if sel.sum() < 30:
        sel = band & (mag >= np.percentile(vals, 70.0))

    ys, xs = np.nonzero(sel)

    # ---- 只保留**梯度方向沿径向**的边缘点 ----
    # 同心环的边缘，其梯度必然指向（或背离）圆心；地面纹理、JPEG 块效应的梯度方向是任意的。
    # ⚠️ 但它**拦不住矩形遮挡**：遮挡物靠近水平轴的那条竖边，其法线方向恰好就是径向的，
    #    方向判据会原样放过。所以真正管用的去噪是下面的「按半径分布裁剪」，
    #    这一道只是先把明显不相干的（纹理/块效应）清掉。
    rx = xs.astype(np.float64) + x0 - c0[0]
    ry = ys.astype(np.float64) + y0 - c0[1]
    rn = np.hypot(rx, ry)
    rn[rn < 1e-6] = 1e-6
    gxs = gx[ys, xs].astype(np.float64)
    gys = gy[ys, xs].astype(np.float64)
    gn = np.hypot(gxs, gys)
    gn[gn < 1e-6] = 1e-6
    cosang = np.abs((gxs * rx + gys * ry) / (gn * rn))
    keep = cosang >= radial_cos
    xs, ys = xs[keep], ys[keep]
    if len(xs) < 30:
        return None

    pts = np.column_stack([xs + x0, ys + y0]).astype(np.float64)
    if len(pts) > max_pts:                       # 均匀抽稀，别让强边扎堆主导
        idx = np.linspace(0, len(pts) - 1, max_pts).astype(int)
        pts = pts[idx]
    return pts


def trim_by_radius(pts, c, bin_px=2.0, tol=2.5, min_frac=0.04, min_cnt=6.0):
    """按**半径分布**裁剪：只保留落在「密集半径带」上的点。

    同心环的合法边缘点必然聚成少数几个尖峰（每圈一个半径）；而遮挡物的直边、噪声、
    块效应点的半径是**连续铺开**的。所以「半径直方图里的尖峰」就是同心结构的指纹 ——
    这个判据比梯度方向强，因为它抓的是同心性本身，而不是边缘的局部朝向。
    """
    if len(pts) < 30:
        return pts
    d = np.hypot(pts[:, 0] - c[0], pts[:, 1] - c[1])
    nb = max(8, int(d.max() / bin_px))
    hist, edges = np.histogram(d, bins=nb)
    centers = 0.5 * (edges[:-1] + edges[1:])
    thr = max(min_cnt, min_frac * len(d))
    peaks = centers[hist >= thr]
    if len(peaks) == 0:
        return pts
    keep = np.zeros(len(d), bool)
    for g in peaks:
        keep |= np.abs(d - g) <= max(tol, bin_px)
    return pts[keep] if keep.sum() >= 20 else pts


def shared_center_fit(pts, c0, iters=12, max_step=2.0):
    """**核心**：最小化「各边缘点到中心的距离」的方差 —— 即同心环共享圆心。

    目标函数  F(c) = Var_i( ‖p_i − c‖ )
    梯度       ∇F = -(2/N)·Σ_i (d_i − r̄)·û_i      （û_i = (p_i−c)/d_i, r̄ = mean(d_i)）
    下降       c ← c + η·(2/N)·Σ(d_i − r̄)·û_i，步长上限 max_step，并做单调性回退。

    为什么这个目标对：同心环的每一圈边缘点到真圆心的距离都是常数（各自的半径），
    所以真圆心处**半径的方差取全局最小**。它：
      · **不需要知道有几个环、环半径多少**（题目没给、实拍也未必测得准）
      · 天然抗遮挡/抗截断（半张靶盘照样收敛，只要边缘点还在）
      · 把 4 圈的信息**一起**用来定一个点，比只拟外圆多了 3 组约束
    """
    p = np.asarray(pts, np.float64).reshape(-1, 2)
    if len(p) < 12:
        return None

    c = np.asarray(c0, np.float64).copy()

    def cost(cc):
        return float(np.var(np.hypot(p[:, 0] - cc[0], p[:, 1] - cc[1])))

    cur = cost(c)
    step = 1.0
    for _ in range(iters):
        d = np.hypot(p[:, 0] - c[0], p[:, 1] - c[1])
        d = np.maximum(d, 1e-6)
        ux = (p[:, 0] - c[0]) / d
        uy = (p[:, 1] - c[1]) / d
        rbar = d.mean()
        gx = (2.0 / len(p)) * float(np.sum((d - rbar) * ux))
        gy = (2.0 / len(p)) * float(np.sum((d - rbar) * uy))
        g = np.hypot(gx, gy)
        if g < 1e-9:
            break
        move = np.array([gx, gy]) / g * min(max_step * step, g)
        cand = c + move
        cc = cost(cand)
        if cc <= cur:                 # 单调才接受
            c, cur = cand, cc
            step = min(1.0, step * 1.4)
        else:
            step *= 0.4               # 回退并把步长收一半
            if step < 1e-3:
                break
    return float(c[0]), float(c[1])


def radial_refine(gray, c0, r0, n_rays=72, n_samp=None):
    """径向剖面亚像素细化：过圆心打若干条射线，找环边界的**线性插值过零点**，
    拿相反两条射线的边界中点求平均 → 新的圆心估计。

    这一步买的是「亚像素」：梯度峰的整数像素定位有 ±0.5px 的量化误差，而灰度
    剖面上的过零点是连续的，插值能把它压到 0.1px 量级。只在**已收敛**的 c0 上用。
    """
    h, w = gray.shape
    r0 = float(r0)
    n_samp = n_samp or int(max(32, min(200, r0 * 1.2)))
    rs = np.linspace(0.10 * r0, 1.06 * r0, n_samp)

    mids = []
    for k in range(n_rays):
        a = 2.0 * np.pi * k / n_rays
        ca, sa = np.cos(a), np.sin(a)
        xs = c0[0] + rs * ca
        ys = c0[1] + rs * sa
        ok = (xs >= 1) & (xs < w - 1) & (ys >= 1) & (ys < h - 1)
        if ok.sum() < 8:
            continue
        prof = cv2.remap(gray, xs[ok].astype(np.float32).reshape(1, -1),
                         ys[ok].astype(np.float32).reshape(1, -1),
                         cv2.INTER_LINEAR).ravel().astype(np.float64)

        # 一阶差分找过零点（暗线在剖面上是一个下凹）
        dif = np.diff(prof)
        zc = []
        for i in range(len(dif) - 1):
            if dif[i] == 0:
                continue
            if dif[i] > 0 >= dif[i + 1]:          # 由升转降 = 亮→暗的坡顶, 取下降过零
                t = dif[i] / (dif[i] - dif[i + 1] + 1e-9)
                zc.append(rs[ok][i] + t * (rs[ok][i + 1] - rs[ok][i]))
        if len(zc) >= 2:
            mids.append((a, 0.5 * (zc[0] + zc[-1])))   # 该方向上最外两条边界的中点

    if len(mids) < 8:
        return None

    # 中点应落在以真圆心为心的圆上 → 再用一次圆拟合收口
    pts = np.array([[c0[0] + m * np.cos(a), c0[1] + m * np.sin(a)] for a, m in mids])
    f = fit_circle_robust(pts, rounds=2)
    if f is None:
        return None
    return float(f[0]), float(f[1])


# ==================== 四个方法 ====================

def m_centroid(img, color_name, **_):
    """外环色块质心 —— explorer 的做法（OpenMV 的 blob.cx() 就是填实后求质心）。

    ⚠️ 用**填实掩膜的矩**（m10/m00）而不是外轮廓点的均值：轮廓点是**边界曲线**上的
    采样，疏密受像素化影响，透视下还会一头稀一头密；质心是面积加权，才是 OpenMV 那个量。
    """
    co = coarse_target(img, color_name)
    if co is None:
        return None
    M = cv2.moments(co["mask"], binaryImage=True)
    if M["m00"] <= 0:
        return None
    return {"cx": M["m10"] / M["m00"], "cy": M["m01"] / M["m00"], "r_px": co["r"]}


def m_hough(img, color_name, band=0.22, **_):
    """cv2.HoughCircles —— 两个参考视觉项目的做法（半径带卡死）。

    参考项目能这么用是因为他们的工位距离固定、环半径已知；这里按粗定位半径
    ±band 自适应一下，否则换个距离就完全失效（这也是那条路的根本局限）。
    """
    co = coarse_target(img, color_name)
    if co is None:
        return None
    r = co["r"]
    # 裁到粗定位邻域再做 Hough：全幅 1920×1080 的累加器又慢又容易被背景干扰带偏
    h, w = img.shape[:2]
    pad = 1.30 * r
    x0 = max(0, int(co["cx"] - pad)); x1 = min(w, int(co["cx"] + pad) + 1)
    y0 = max(0, int(co["cy"] - pad)); y1 = min(h, int(co["cy"] + pad) + 1)
    gray = cv2.cvtColor(img[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.2)
    circles = cv2.HoughCircles(
        gray, cv2.HOUGH_GRADIENT, dp=1.5, minDist=max(20, r),
        param1=120, param2=40,
        minRadius=max(4, int(r * (1.0 - band))), maxRadius=int(r * (1.0 + band)))
    if circles is None:
        return None
    c = np.asarray(circles[0], np.float64)
    c[:, 0] += x0
    c[:, 1] += y0
    # 取离粗定位最近的那个（Hough 会返回一堆）
    i = int(np.argmin(np.hypot(c[:, 0] - co["cx"], c[:, 1] - co["cy"])))
    return {"cx": float(c[i, 0]), "cy": float(c[i, 1]), "r_px": float(c[i, 2])}


def m_outfit(img, color_name, **_):
    """只拟合外圆（盘缘 Ø100）—— 用色块外轮廓的边界点做鲁棒最小二乘。"""
    co = coarse_target(img, color_name)
    if co is None:
        return None
    f = fit_circle_robust(co["contour"].reshape(-1, 2), rounds=4)
    if f is None:
        return None
    return {"cx": f[0], "cy": f[1], "r_px": f[2]}


def _hole_contour(mask):
    """色环**内边界**（掩膜里的孔洞）—— 实拍确认这是整幅图里最干净的一条圆边。

    实测（2026-09 现场帧）：内边界拟合椭圆的残差 std **0.44px**，
    而外边界是手工贴的波浪边，残差 std **2.15px、最大 5.8px**（像素化只能解释 ~0.3px）。
    所以「圆心」应当以这条内边界为主特征。

    ⚠️ 但孔洞**会消失**：只要有任何东西（遮挡物、阴影、画面边缘）切到白心的边，
    白心就和外部背景连成一片，孔洞没了。所以用它的方法必须有回退路径。
    """
    cnts, hier = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if hier is None:
        return None
    hier = hier[0]
    holes = [c for i, c in enumerate(cnts) if hier[i][3] != -1]   # 有父轮廓 = 是个孔
    if not holes:
        return None
    return max(holes, key=cv2.contourArea)


def fit_hole_checked(mask, min_fill=0.75):
    """拟合内边界，并**检查它是不是一个可信的圆**（不是一段弧）。→ (cx,cy,r) 或 None。

    孔洞可能是被切掉一块的月牙，这时候圆拟合看着有解、其实中心已经飞了。
    判据用「轮廓面积 / 拟合圆面积」：完整圆 ≈ 1.0，缺一大块的月牙会明显偏小。
    min_fill=0.75 相当于允许缺失约 1/4。
    """
    hole = _hole_contour(mask)
    if hole is None:
        return None
    f = fit_circle_robust(hole.reshape(-1, 2), rounds=4)
    if f is None or f[2] < 3.0:
        return None
    if cv2.contourArea(hole) / (math.pi * f[2] * f[2]) < min_fill:
        return None
    return f


def fit_hole_in(cnts, hier, i0, min_fill=0.75):
    """同 `fit_hole_checked`，但复用**已在手上**的 CCOMP 轮廓树（省一遍 findContours）。

    `i0` 是该候选的顶层轮廓下标，内孔 = 它的直接子轮廓。判据与 `fit_hole_checked`
    逐条一致（最大孔、r≥3、fill≥min_fill），只是不再重跑 findContours ——
    `color_candidates` 本来就要那棵树来数 holes。
    """
    kids = [j for j in range(len(cnts)) if hier[j][3] == i0]
    if not kids:
        return None
    hole = max((cnts[j] for j in kids), key=cv2.contourArea)
    f = fit_circle_robust(hole.reshape(-1, 2), rounds=4)
    if f is None or f[2] < 3.0:
        return None
    if cv2.contourArea(hole) / (math.pi * f[2] * f[2]) < min_fill:
        return None
    return f


def m_inner(img, color_name, **_):
    """只拟合**色环内边界**（色环↔白心那个圆）。

    实拍数据说这条边最干净，所以它应当是最强的一个单特征法 —— 把它放进对比里。
    返回的 r_px 已按靶盘设计（Ø55 内圆 ↔ Ø100 外圆）折算成**外圆等效半径**，
    好跟别的方法的 r_px 以及视差公式 `offset = h·r_px/50` 保持同一口径。
    """
    co = coarse_target(img, color_name)
    if co is None:
        return None
    f = fit_hole_checked(co["mask"])
    if f is None:
        return None
    return {"cx": f[0], "cy": f[1], "r_px": f[2] / R_INNER_OVER_OUTER}


def m_shared(img, color_name, sub=False, trim_rounds=0, **_):
    """**共享圆心同心环拟合**（本方案主张）。sub=True 时再接一段径向亚像素细化。

    起点用**内边界鲁棒拟合**（`_hole_contour` + 鲁棒圆拟合）而不是色块质心，也不再拿外轮廓
    当主力：实拍确认外缘是手工贴的波浪边，而内边界是干净的圆。所谓"共享圆心"在这里就是
    「内边界定初值 → 用 ROI 内**所有沿径向的**边缘点（两条边都算）联合收敛」。

    `trim_rounds` 默认 **0=不裁剪**。裁剪（trim_by_radius）是实验过的：它假设"合法点聚成
    少数几个半径尖峰"，但遮挡物的直边在半径直方图上同样会造出密集格，于是**该留的被丢掉、
    该丢的留下来** —— 实测在遮挡用例上把误差从 1.0mm 拉到 3.7mm，是**负收益**，故默认关。
    （那条路要真做对得用 RANSAC/IRLS 而不是硬阈值，留到有实拍数据再定。）
    """
    co = coarse_target(img, color_name)
    if co is None:
        return None
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    c = (co["cx"], co["cy"])
    f_in = fit_hole_checked(co["mask"])
    if f_in is not None:
        c = (f_in[0], f_in[1])            # 起点用**内边界**拟合（实拍最干净的那条边）

    pts = _edge_points(gray, c, co["r"])
    if pts is None:
        return None
    n0 = len(pts)

    for _ in range(max(1, trim_rounds)):
        f = shared_center_fit(pts, c)
        if f is None:
            break
        c = f
        nxt = trim_by_radius(pts, c)
        if len(nxt) < 20:
            break
        growth = len(nxt) == len(pts)
        pts = nxt
        if growth:                        # 裁剪没去掉东西 = 已稳定
            break

    cx, cy = c
    if sub:
        r2 = radial_refine(gray, (cx, cy), co["r"])
        if r2 is not None:
            cx, cy = r2
    return {"cx": cx, "cy": cy, "r_px": co["r"], "n_edge": len(pts), "n_edge0": n0}


def color_candidates(img, color_name, min_area=120.0, max_area_frac=1.0):
    """列出该颜色的**所有**候选连通域（不是只取最大）。

    ⚠️ 「取最大色块」在真场地是错的：现场背景是一整面蓝柜子，蓝靶在蓝色候选里排第 30 名
    开外，取最大只会拿到柜子。所以要**保留全部候选**，交给跨色尺寸一致性去挑。

    ⚠️⚠️ `max_area_frac` 默认 **1.0（等于不设上限）**。曾经设成 0.25，结果柜子和蓝靶
    **挨在一起被连通域合并**成一个大块 → 被上限整个滤掉 → 蓝色一个候选都不剩、直接全灭。
    「排除大块」这个思路本身是错的：靶和干扰物可能连通，排掉大块等于排掉靶
    （而那个合并块的**内孔仍在**，本来是可以救回来的）。
    """
    mask = color_mask(img, color_name)
    h, w = mask.shape
    nlab, labels, stats, _cents = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    for lab in range(1, nlab):
        a = float(stats[lab, cv2.CC_STAT_AREA])
        if a < min_area or a > max_area_frac * h * w:
            continue
        # ⚠️ 每个候选要拿到**只含自己**的掩膜。曾经这里是拿整幅色掩膜去找孔，结果
        #    画面里另一个更大的同色带孔物（蓝桶口）的内孔比靶的大，**孔的拟合直接抓了桶口**
        #    —— 看着是"选错候选"，其实是掩膜没隔离。真踩过。
        comp = (labels == lab).astype(np.uint8) * 255
        cnts, hier = cv2.findContours(comp, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        if not cnts:
            continue
        hier = hier[0]
        tops = [i for i in range(len(cnts)) if hier[i][3] == -1]
        if not tops:
            continue
        i0 = max(tops, key=lambda i: cv2.contourArea(cnts[i]))
        c = cnts[i0]
        (cx, cy), r = cv2.minEnclosingCircle(c)
        if r < 4.0:
            continue
        nholes = sum(1 for j in range(len(cnts)) if hier[j][3] == i0)
        # q = 内孔拟合半径 ÷ 本候选外接圆半径 —— 「像不像靶环」的判据②'（2026-09 加）。
        # 靶盘设计值 R_INNER_OVER_OUTER = 0.55：真靶环实测 0.52~0.55，而现场一切杂块
        # （蓝柜子/蓝箱/木椅腿/背景碎块）实测 ≤0.23，两者不重叠。
        # 轮廓树已经在手上，顺手拟合，不另开一遍 findContours。
        hf = fit_hole_in(cnts, hier, i0)
        out.append({"color": color_name, "area": a, "r": r, "cx": cx, "cy": cy,
                    "contour": c, "mask": comp, "holes": nholes, "hole_fit": hf,
                    "q": (hf[2] / r) if (hf and r > 0) else 0.0})
    out.sort(key=lambda d: -d["area"])
    return out


def pick_consistent(cands_by_color, tol=2.0, per_color=10, min_colors=2, ring_q=None):
    """**跨色尺寸一致性选靶** —— 每色最多选一个，且要求选出来的尺寸互相匹配。

    这是探子给的方向，但做法必须是「**在候选里挑一组匹配的**」而不是「各取最大再校验」：
    现场实测蓝柜子外接圆半径 455px、绿/红靶 55px 左右，差 8.3 倍 —— 按最大取，蓝选出来
    就是柜子；**校验只能说明"这个不对"，给不出正确的蓝靶**（真蓝靶在 85 个候选里排 30 名开外）。

    做法（对应「某一个突然特别大/特别小就该怀疑」）：
      1. 每色先取自己最大的那个；
      2. 算这一组尺寸的中位数，把偏离超过 tol 倍的**颜色**记为可疑；
      3. 对可疑的颜色，改挑它自己候选里尺寸最接近中位数的那个；
      4. 第 2~3 步再来一轮（换掉一个可能让中位数变，需要收敛）。

    ⚠️⚠️ **锚点不能是「各色最大」**：真场地帧上直接失败过 —— 红靶旁边的木椅子是 r=311
    的红块、蓝柜子是 r=448 的蓝块，都盖过真靶（r≈62/79/65）→ 中位数被拉到 311 →
    **真靶反倒成了"离群"被剔掉**（症状：红靶检不出、蓝靶检成了柜子一块）。
    「取最大」在干扰物比目标大时**必然**错，而"哪个是干扰物"恰恰是这里要判的事。

    ⚠️⚠️ **还要先按"有没有内孔"筛一遍**：靶盘是「色环 + 白心」，一定带孔；木椅子、柜子、
    背景碎块都不带孔。实测不加这道筛时，共识会选出一组 r≈30~34 的**背景小碎块**
    （它们的尺寸比真靶更整齐，在覆盖颜色数相同时按 spread 排序反而赢了）。
    **尺寸一致性只是必要条件，不是充分条件。**

    tol 给到 2.0：三个靶并排，但相机凑近某一个时近的会明显大。同距实测三靶半径差 <10%。

    `ring_q=(lo,hi)` 给定时（**判据②'**，2026-09 加），进池子前先按
    `q = 内孔半径 ÷ 外接圆半径` 筛一遍；`None` = 旧行为（只按"有没有内孔"筛）。
    为什么非它不可：现场同色干扰块够多时，**三个杂块凑巧一样大**就能拼出一个合法的
    "尺寸一致组"把真靶挤掉 —— 尺寸一致性只是必要条件、远不是充分条件。
    而 q 只看**单个候选自己的形状**，跟别的候选没关系，所以骗不过它。
    实测（现场帧、蓝靶）：真靶环 q=0.52~0.55，一切杂块 q≤0.23。
    """
    if ring_q is None:
        work = {c: [x for x in lst if x.get("holes", 0) >= 1]
                for c, lst in cands_by_color.items()}
    else:
        lo_q, hi_q = ring_q
        ring = {c: [x for x in lst if lo_q <= x.get("q", 0.0) <= hi_q]
                for c, lst in cands_by_color.items()}
        # ⚠️⚠️ 像靶环的凑不齐两色时：退回 **「有内孔的候选」**，**不是**退回全部
        #    （2026-10-16 修, 与 Pi 侧 tasks/_target.py 同一处; 改一处必须同步另一处）。
        #    退回全部 = 把判据②' 整个关掉, 而它当初正是为「三个杂块凑巧一样大」加的。
        #    现场（CSI 帧）: 蓝靶与蓝柜子合并被 r_max 剔掉 ⇒ 只剩红色像靶环 ⇒ 退回全部
        #    ⇒ 地面三个 q=0.00/无孔的小杂块凑成一组被选中, 真红靶反被挤掉、标记画到地上。
        #    「有内孔」是不依赖同伴的靶盘证据（白心）, 当降级门最稳。
        work = ring if sum(1 for v in ring.values() if v) >= min_colors else \
            {c: [x for x in lst if x.get("holes", 0) >= 1]
             for c, lst in cands_by_color.items()}
    # 连"有内孔的"都凑不齐两色 → 才真的退回全部（遮挡到连白心都没了: 保旧行为, 别把路堵死）
    if sum(1 for v in work.values() if v) < min_colors:
        work = cands_by_color

    pool = []
    for col, lst in work.items():
        for c in lst[:per_color]:
            if c["r"] > 0:
                pool.append((c["r"], col, c))
    if not pool:
        return {}

    best = None
    for s, _col, _c in pool:
        lo, hi = s / tol, s * tol
        covered = {}
        for r, col, c in pool:
            if lo <= r <= hi:
                if (col not in covered) or (abs(r - s) < abs(covered[col]["r"] - s)):
                    covered[col] = c
        if len(covered) < min_colors:
            continue
        sizes = [c["r"] for c in covered.values()]
        score = (-len(covered), max(sizes) / min(sizes))
        if best is None or score < best[0]:
            best = (score, covered)

    picked = dict(best[1]) if best else {}
    if not picked:
        return {}

    # 没被覆盖的颜色：挑尺寸跟共识最接近的（而不是"最大的那个" = 通常就是干扰物）
    ref = float(np.median([c["r"] for c in picked.values()]))
    for col, lst in work.items():
        if col in picked or not lst:
            continue
        cand = min(lst[:per_color], key=lambda c: abs(c["r"] - ref))
        if (cand["r"] / ref <= tol) and (ref / cand["r"] <= tol):
            picked[col] = cand
    return picked


def find_target(img, want_color, tol=2.0, ring_q=None):
    """**选靶主入口**：三色一起看，跨色尺寸一致性选出一组，再返回 want_color 那一颗。

    返回 dict（含 contour / mask / cx / cy / r / holes）或 None。

    只用一个颜色时（另外两色都不在画面里）失去尺寸基准，此时退化成"取最大候选" ——
    所以调用方还应当有**不依赖同伴**的判据兜底（内孔/圆度），见 `m_triple`。
    """
    cands = {c: color_candidates(img, c) for c in ("red", "green", "blue")}
    picked = pick_consistent(cands, tol=tol, ring_q=ring_q)
    if want_color in picked:
        return picked[want_color]
    # 目标色在画面里，但没有同伴可比 → 退回该色最大候选
    lst = cands.get(want_color) or []
    return lst[0] if lst else None


def m_triple(img, color_name, tol=2.0, ring_q=None, **_):
    """**跨色尺寸一致性选靶 + 内孔优先/外缘兜底** —— 目前主张的做法。

    两层判据互补（这是现场帧逼出来的，不是设计出来的）：
      · **跨色尺寸一致性** —— 治「同色大块」。现场背景整面蓝柜子，蓝靶与它同色；
        尺寸差 8.3 倍，一致性判据直接把它排掉（见 pick_consistent）。
      · **内孔存在性** —— 治「同色小块」，而且**不依赖同伴**（瞄准时凑近某一个靶，
        另外两个可能出画）。靶盘有色环↔白心的内孔，桌上的蓝色杂物没有。

    `ring_q=(lo,hi)` 再叠一层**判据②'**「像不像靶环」（见 pick_consistent）。
    现场实测它才是治「蓝靶指向画面角落杂块」的那一道 —— 尺寸一致性单独用会被
    "三个杂块凑巧一样大"骗过。Pi 侧 `tasks/_target.py` 用的就是开着的这一版。
    """
    t = find_target(img, color_name, tol=tol, ring_q=ring_q)
    if t is None:
        return None
    f_in = t.get("hole_fit") or fit_hole_checked(t["mask"])
    if f_in is not None:
        return {"cx": f_in[0], "cy": f_in[1], "r_px": f_in[2] / R_INNER_OVER_OUTER}

    # 兜底：外缘。但**必须过圆度这一关** —— 否则柜子/桌角这种大体量矩形也会被
    # fit_circle_robust 拟出一个"圆"来（它不会返回 None，只是残差大）。
    f_out = fit_circle_robust(t["contour"].reshape(-1, 2), rounds=4)
    if f_out is None or f_out[2] < 4.0:
        return None
    # 圆度关卡用**稳健离散度**（半径的 MAD），不能用全部点的 RMS —— 遮挡物的直边正是
    # 鲁棒拟合故意剔掉的离群点，算进来必然超标（实测一个拟合误差 0.2px 的正确圆被判成
    # rms/r=0.277 丢弃）。MAD 只看主体分布。这条与 Pi 侧 tasks/_target.py 保持一致。
    p = t["contour"].reshape(-1, 2).astype(np.float64)
    d = np.hypot(p[:, 0] - f_out[0], p[:, 1] - f_out[1])
    mad = float(np.median(np.abs(d - np.median(d))))
    if mad / max(f_out[2], 1e-6) > 0.06:
        return None
    return {"cx": f_out[0], "cy": f_out[1], "r_px": f_out[2]}


def m_both(img, color_name, **_):
    """**内边界优先、外边界兜底** —— 实拍数据直接推出来的组合，不是折中。

      · 内边界最准（中位 0.26mm）但整条边被切到就没了（实测 5/53 个用例检不出）
      · 外边界糙（中位 0.40mm）却几乎总能拟合上（实测只丢 1 个）
      · 两条边**同心**，所以「能拿内边就拿、拿不到退回外边」既不丢精度也不丢可用率

    ⚠️ 试过「两条边并起来做共享圆心联合拟合」，**实测更差**（中位 0.84mm、最大 17.7mm）：
    波浪外缘不是零均值噪声，在"半径方差最小"这个目标下它会**系统性地**把圆心拖走，
    而 `fit_circle_robust` 的 MAD 裁剪之所以扛得住，是因为它按各自那条边的残差剔点。
    结论：分开拟合、按可用性选，而不是强行合并。
    """
    co = coarse_target(img, color_name)
    if co is None:
        return None

    f_in = fit_hole_checked(co["mask"])
    if f_in is not None:
        return {"cx": f_in[0], "cy": f_in[1], "r_px": f_in[2] / R_INNER_OVER_OUTER}

    f_out = fit_circle_robust(co["contour"].reshape(-1, 2), rounds=4)
    if f_out is None:
        return None
    return {"cx": f_out[0], "cy": f_out[1], "r_px": f_out[2]}


# 靶环「内孔/外径」比的可接受区间（判据②'）。靶盘设计值 R_INNER_OVER_OUTER = 0.55；
# 现场实测真靶环 0.52~0.55、一切杂块 ≤0.23 —— 0.35 卡在中间的空白带里。
# ⚠️ 与 Pi 侧 config.TARGET_RING_Q_MIN / MAX 是**同一对值**，改一处必须同步另一处。
RING_Q_MIN = 0.35
RING_Q_MAX = 0.85


METHODS = {
    "m_centroid": m_centroid,
    "m_hough": m_hough,
    "m_outfit": m_outfit,
    "m_inner": m_inner,
    "m_shared": m_shared,
    "m_shared_sub": lambda img, color, **kw: m_shared(img, color, sub=True, **kw),
    "m_both": m_both,
    "m_triple": m_triple,
    # 判据②' 开关版：m_triple + 「像不像靶环」的 q 门槛。**这才是 Pi 侧
    # tasks/_target.py 现在跑的那一版**（2026-09 现场实测：蓝靶误识别被它治好，
    # 且 27/27 次现场识别全中）。
    "m_triple_q": lambda img, color, **kw: m_triple(
        img, color, ring_q=(RING_Q_MIN, RING_Q_MAX), **kw),
}


def detect(img, color_name, method="m_shared", roi=None, **kw):
    """统一入口 → {'cx','cy','r_px',...} 或 None（没找到）。坐标一律**全分辨率**。

    `roi = (x0, y0, x1, y1)` 是**相对坐标**（0~1），给了就只在这个窗口里找，返回坐标自动
    加回偏移。这是**第一道、也是最省事的一道**过滤：

      · 瞄准闭环本来就会把靶心拉到画面中心附近 → "靶心应该在的地方"是个**已知的小窗口**
      · 窗口外的干扰（现场背景那面蓝柜子）**根本不用去看**，而不是"看了再去排除"
      · 全部参考项目（explorer 的 OpenMV 脚本、两个物流搬运项目）用的都是固定 ROI

    代价是它假设靶心落在窗口内；这由**上一个步骤**（粗对准 / 臂的预备位姿）保证。
    真机上它就是一组 `TARGET_ROI_*` 标定量（走「调参」Tab）。
    """
    fn = METHODS.get(method)
    if fn is None:
        raise KeyError("未知方法: %s" % method)

    if roi is None:
        r = fn(img, color_name, **kw)
        return r

    h, w = img.shape[:2]
    x0 = max(0, int(roi[0] * w))
    x1 = min(w, int(roi[2] * w))
    y0 = max(0, int(roi[1] * h))
    y1 = min(h, int(roi[3] * h))
    if (x1 - x0) < 8 or (y1 - y0) < 8:
        return None
    r = fn(img[y0:y1, x0:x1], color_name, **kw)
    if r is None:
        return None
    r = dict(r)
    r["cx"] += x0
    r["cy"] += y0
    return r
