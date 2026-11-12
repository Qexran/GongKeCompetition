# -*- coding: utf-8 -*-
"""合成靶渲染 —— 按命题图（图 7 固定靶示意图）的真实尺寸生成靶盘图像 + **精确真值圆心**。

为什么要有这个：真靶盘还没在相机前摆过，而「圆心算法准不准」是整个打靶方案的地基。
合成图的真值圆心是**算出来的**（不是标的），所以能把误差量到亚像素级 —— 实拍图做不到。

几何（全部来自命题图 7 的三视图，单位 mm，靶心为原点）:

        Ø100 ┌─────────────┐ 外环：红/绿/蓝三色之一（比赛用来标识是哪个靶）
             │  Ø55 ┌─────┐│
             │      │ Ø30 ││  8 环 = Ø30..Ø55
             │  Ø15 │┌───┐││
             │      ││10 │││  9 环 = Ø15..Ø30
             │      │└───┘││  10 环 = Ø15（要打的就是它，半径 7.5mm）
             └──────┴─────┘┘
   环之间是黑色圆线；靶心离地 115mm（本文件只关心盘面，高度在结构上保证）。

相机模型是**物理正确**的：给内参 K、靶盘平面位姿 (R, t)，单应 H = K·[r1 r2 t]。
所以透视收缩、离轴变形、距离-比例关系都是真的，不是贴图。
"""
import math

import cv2
import numpy as np

# ---------------- 靶盘几何（mm） ----------------
# 命题图 7 画的是：外环 Ø55~Ø100 着色，里面 8/9/10 环。
# ⚠️ **2026-09 实测更正**：现场用的是手工/3D 打印样件，白色圆心上**没有印 8/9/10 环**
#    （队伍也不确定正式靶有没有，按「没有」设计）。所以真实的可用几何特征只有两条圆边：
#      外边界 = 色环外缘（样件是手工贴的，实测**不是圆**，拟合椭圆残差 std 2.15px）
#      内边界 = 色环↔白心交界（实测是**干净的圆**，残差 0.44px）
#    算法必须以内边界为主特征 —— 这是实拍量出来的结论，不是推测。
R_OUTER = 50.0     # Ø100 色环外缘
R_INNER = 27.5     # Ø55 色环内缘 = 白色圆心盘的边
RIM_MM = 1.5       # 外缘那圈白背板的可见宽度

# 兼容旧名（命题图上 8/9/10 环的位置，仅在 inner_rings=True 时才画）
R_DISC = R_OUTER
R_8 = 27.5
R_9 = 15.0
R_10 = 7.5

WHITE_CENTER = (238, 238, 236)
RIM_COLOR = (245, 245, 245)
RING_LINE = (60, 60, 60)      # 仅在 inner_rings=True 时用的内环线颜色

# 外环三色，用 **HSV 定义**再转 BGR —— 这样检测端的阈值带和渲染端是同一套语言，
# 不会出现「渲染出来的红恰好落在阈值带外」这种自欺欺人的假通过。
# (H 0~179, S 0~255, V 0~255)
COLOR_HSV = {
    "red":   (0, 210, 175),
    "green": (60, 190, 150),
    "blue":  (110, 210, 175),
}


def hsv_to_bgr(h, s, v):
    """单个 HSV → BGR（uint8）。cv2.cvtColor 要整图，这里只要一个像素。"""
    px = np.uint8([[[h, s, v]]])
    b, g, r = cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0]
    return int(b), int(g), int(r)


def target_color_bgr(name, v_scale=1.0, s_scale=1.0):
    """外环色的 BGR。v_scale/s_scale 用来扫「印刷色深浅 / 现场光照」的鲁棒性。"""
    h, s, v = COLOR_HSV[name]
    return hsv_to_bgr(h, int(min(255, s * s_scale)), int(min(255, v * v_scale)))


def _wobble_polygon(c, r_base_mm, scale, lumpy, rng, n=720, harm=5):
    """半径按角度做几次谐波扰动 → 一条"手工贴出来的"波浪轮廓。

    lumpy 是**半径的相对标准差**。实测样件外缘的残差 std ≈ 半径的 4.6%（r≈47px 上 2.15px），
    所以 lumpy=0.05 与现场量到的量级对得上。lumpy=0 就退化成正圆。
    """
    th = np.linspace(0.0, 2.0 * np.pi, n, endpoint=False)
    if lumpy > 0:
        w = np.zeros_like(th)
        for k in range(1, harm + 1):
            w += rng.normal(0.0, 1.0 / k) * np.cos(k * th + rng.uniform(0.0, 2.0 * np.pi))
        sd = float(w.std())
        if sd > 1e-9:
            w = w / sd * lumpy
    else:
        w = np.zeros_like(th)
    r = r_base_mm * (1.0 + w) * scale
    xs = np.round(c + r * np.cos(th)).astype(np.int32)
    ys = np.round(c + r * np.sin(th)).astype(np.int32)
    return np.column_stack([xs, ys]).reshape(-1, 1, 2)


def render_target_canvas(px_per_mm, color_name, v_scale=1.0, s_scale=1.0,
                         margin_mm=6.0, ss=4, lumpy=0.0, seed=0, inner_rings=False):
    """把靶盘画成一张画布（正面、无透视），返回 (canvas_bgr, 圆心像素坐标, alpha)。

    结构照实拍（不是命题图）：**外圈白背板 → 色环（外缘波浪）→ 白色圆心盘**。
    默认**不画 8/9/10 内环** —— 现场样件上没有，队伍也按"没有"设计（inner_rings=True 可开）。

    先按 ss 倍超采样再缩回来 —— 边缘不做抗锯齿的话会有台阶，拟合会被这个假象带偏
    （真相机是有光学抗锯齿的，台阶是渲染器的伪影，不是被测对象）。

    ⚠️⚠️ **必须同时返回 alpha（靶盘剪影的实心掩膜）**。画布在靶盘以外是黑的，如果合成时
    拿"整块画布"当遮罩，就会把靶盘**连同一块黑方**一起贴进画面 —— 那块黑方的四条直边
    是极强的灰度边缘，会把拟合整个带偏。这个坑真踩过：`m_shared` 在遮挡用例上偏 20mm，
    查到最后不是算法问题，是这里贴了个黑方块。
    """
    rng = np.random.default_rng(seed)
    half = R_OUTER + RIM_MM + margin_mm
    n = int(round(2.0 * half * px_per_mm * ss))
    c = n / 2.0
    ci = int(round(c))

    def rpx(mm):
        return mm * px_per_mm * ss

    img = np.zeros((n, n, 3), np.uint8)
    alpha = np.zeros((n, n), np.uint8)

    # 白背板（自己的波浪），再叠色环（自己的波浪），最后白色圆心盘（正圆）
    rim_poly = _wobble_polygon(c, R_OUTER + RIM_MM, ss * px_per_mm, lumpy, rng)
    outer_poly = _wobble_polygon(c, R_OUTER, ss * px_per_mm, lumpy, rng)
    cv2.fillPoly(img, [rim_poly], RIM_COLOR)
    cv2.fillPoly(img, [outer_poly], target_color_bgr(color_name, v_scale, s_scale))
    cv2.circle(img, (ci, ci), int(round(rpx(R_INNER))), WHITE_CENTER, -1)

    if inner_rings:
        # 命题图 7 的 8/9/10 环（默认不画，见函数头）
        for r_mm in (R_8, R_9, R_10):
            cv2.circle(img, (ci, ci), int(round(rpx(r_mm))),
                       RING_LINE, max(1, int(round(rpx(1.2)))))

    cv2.fillPoly(alpha, [rim_poly], 255)

    n_out = int(round(2.0 * half * px_per_mm))
    img = cv2.resize(img, (n_out, n_out), interpolation=cv2.INTER_AREA)
    alpha = cv2.resize(alpha, (n_out, n_out), interpolation=cv2.INTER_AREA)
    return img, n_out / 2.0, alpha


def make_homography(f_px, frame_wh, dist_mm, offset_mm=(0.0, 0.0),
                    roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    """靶盘平面（z=0）→ 图像 的单应 H = K·[r1 r2 t]。

    靶盘坐标系: x 右, y **下**（跟图像同向，画布坐标直接对应）, z 指向相机外。
    t = 靶心在相机系中的位置（相机系 x 右 / y 下 / z 前）。
    dist_mm = 靶心到相机的深度；pitch/yaw 是靶盘相对正对方向的偏转角（度）。
    """
    cx0, cy0 = frame_wh[0] / 2.0, frame_wh[1] / 2.0
    K = np.array([[f_px, 0.0, cx0],
                  [0.0, f_px, cy0],
                  [0.0, 0.0, 1.0]], float)

    a, b, g = (math.radians(pitch_deg), math.radians(yaw_deg), math.radians(roll_deg))

    def rx(t):
        c, s = math.cos(t), math.sin(t)
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)

    def ry(t):
        c, s = math.cos(t), math.sin(t)
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)

    def rz(t):
        c, s = math.cos(t), math.sin(t)
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)

    R = rz(g) @ ry(b) @ rx(a)
    t = np.array([offset_mm[0], offset_mm[1], dist_mm], float)
    return K @ np.column_stack([R[:, 0], R[:, 1], t])


def make_background(frame_wh, base_bgr=(148, 148, 148), grad=0.0,
                    rng=None, noise_sigma=0.0, vignette=0.0):
    """场地背景：基准灰 + 可选亮度梯度（现场补光不均）+ 噪声 + 暗角。"""
    w, h = frame_wh
    rng = rng or np.random.default_rng(0)
    img = np.zeros((h, w, 3), np.float32)
    img[:, :] = base_bgr

    if grad:
        ramp = np.linspace(-grad, grad, w, dtype=np.float32)[None, :, None]
        img += ramp
    if vignette:
        yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
        d = np.sqrt(((xx - w / 2) / (w / 2)) ** 2 + ((yy - h / 2) / (h / 2)) ** 2)
        img *= (1.0 - vignette * np.clip(d, 0, 1.4))[:, :, None]
    if noise_sigma:
        img += rng.normal(0.0, noise_sigma, img.shape).astype(np.float32)

    return np.clip(img, 0, 255).astype(np.uint8)


def apply_lens_distortion(img, f_px, k1, k2=0.0):
    """按 OpenCV 的径向畸变模型，把理想图变成"相机实际拍到的样子"。

    模型（归一化坐标）: r² = x²+y²,  x_d = x·(1 + k1·r² + k2·r⁴)
    k1 < 0 = 桶形（广角镜头的常态）, k1 > 0 = 枕形。

    要**造**畸变图得反向采样：对畸变图上的每个像素，反解它来自理想图的哪里。
    反解用定点迭代（畸变不大时 4~6 轮收敛到亚像素），全程向量化。

    ⚠️ 这条是**质心法最大的系统性风险**：畸变把离轴的圆"挤"成非圆，而质心是按面积
    加权的，于是离轴越远偏得越多 —— 而且**方向固定**，不是随机噪声，多帧平均消不掉。
    """
    h, w = img.shape[:2]
    cx0, cy0 = w / 2.0, h / 2.0
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    xd = (xx - cx0) / f_px
    yd = (yy - cy0) / f_px
    x, y = xd.copy(), yd.copy()
    for _ in range(6):
        r2 = x * x + y * y
        x = xd / (1.0 + k1 * r2 + k2 * r2 * r2)
        y = yd / (1.0 + k1 * r2 + k2 * r2 * r2)
    mapx = (x * f_px + cx0).astype(np.float32)
    mapy = (y * f_px + cy0).astype(np.float32)
    return cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_REPLICATE)


def apply_shadow(img, frac, factor):
    """把画面的一块矩形压暗 —— 模拟补光不均 / 被人挡住光。

    为什么这条最能区分方法：外环被压暗的那一段会**掉出色带阈值**，色块因此不再对称，
    质心立刻偏向还亮着的那侧；而同心环拟合只当"少了一段边缘点"，中心不动。

    frac = (x0, x1, y0, y1) 都是 0~1 的相对坐标。
    """
    h, w = img.shape[:2]
    x0, x1 = int(frac[0] * w), int(frac[1] * w)
    y0, y1 = int(frac[2] * h), int(frac[3] * h)
    out = img.astype(np.float32)
    out[y0:y1, x0:x1] *= factor
    return np.clip(out, 0, 255).astype(np.uint8)


def apply_occluders(img, rects, color=(38, 38, 38)):
    """图像坐标的实心矩形遮挡（机械臂自己挡、靶架立柱、光电接收器…）。"""
    out = img.copy()
    for (x, y, w_, h_,) in rects:
        cv2.rectangle(out, (int(x), int(y)), (int(x + w_), int(y + h_)), color, -1)
    return out


def apply_specular(img, spots):
    """镜面高光：亮斑会把外环的一块像素顶出色带（V 过饱和 → 饱和度反降）。"""
    out = img.copy()
    for (x, y, r) in spots:
        layer = out.copy()
        cv2.circle(layer, (int(x), int(y)), int(r), (250, 250, 250), -1)
        k = int(2 * round(r * 0.6) + 1)
        layer = cv2.GaussianBlur(layer, (k, k), r * 0.35)
        m = np.zeros(out.shape[:2], np.float32)
        cv2.circle(m, (int(x), int(y)), int(r), 1.0, -1)
        m = cv2.GaussianBlur(m, (k, k), r * 0.35)[:, :, None]
        out = np.clip(out * (1 - m) + layer * m, 0, 255).astype(np.uint8)
    return out


def apply_distractors(img, items):
    """同色大块干扰物。

    现场背景是**一整面蓝色柜子**（实拍确认），而蓝靶和它同色 —— 纯色阈值在真实场景里
    会被这种大块同色区域骗走（实测：全帧跑检测时五个方法全锁到了 r=224 的笔记本屏幕上）。
    合成用例里必须带上这一类，否则测出来的"鲁棒性"是假的。

    items = [(颜色名, (x0,y0,x1,y1) 相对坐标, 亮度系数), ...]
    """
    out = img.copy()
    h, w = out.shape[:2]
    for color_name, box, v_scale in items:
        bgr = target_color_bgr(color_name, v_scale)
        cv2.rectangle(out, (int(box[0] * w), int(box[1] * h)),
                      (int(box[2] * w), int(box[3] * h)), bgr, -1)
    return out


def apply_ring_distractors(img, items):
    """**同色、圆形、还带孔**的干扰物（蓝水桶口、圆牌、圆形周转箱…）。

    为什么单列一类：矩形柜子没有内孔，靠"靶盘有色环内孔"这条判据就能排掉；
    但一个**也带孔的圆形同色物**能骗过内孔判据 —— 只有「跨色尺寸一致性」能治它。
    这一类用例是用来量那条判据到底值不值它的开销的。

    items = [(颜色名, cx 相对, cy 相对, 外半径相对, 壁厚相对, 亮度系数), ...]
    """
    out = img.copy()
    h, w = out.shape[:2]
    for color_name, cxf, cyf, rf, tf, v_scale in items:
        bgr = target_color_bgr(color_name, v_scale)
        cx, cy = int(cxf * w), int(cyf * h)
        r = int(rf * min(w, h))
        t = max(2, int(tf * min(w, h)))
        cv2.circle(out, (cx, cy), r, bgr, -1)
        cv2.circle(out, (cx, cy), max(1, r - t), WHITE_CENTER, -1)
    return out


def compose(canvas, canvas_c, alpha, px_per_mm, H, bg):
    """按单应 H 把画布贴进背景（遮罩用**alpha 圆盘**，不是整块画布 —— 见 render 那条警告）。

    画布像素 → 靶盘 mm → H → 图像像素，走的是**同一个 H**，所以真值圆心
    （靶盘原点）与贴图位置在数学上必然一致，不存在「贴歪了但真值没跟着歪」。

    边缘用软混合而不是 `mask > 127` 硬切：硬切会在盘缘留一圈锯齿，那圈锯齿本身就是
    强梯度，会被边缘点提取当成"第五个环"。
    """
    h, w = canvas.shape[:2]

    def to_mm(px, py):
        return (px - canvas_c) / px_per_mm, (py - canvas_c) / px_per_mm

    corners = [(0.0, 0.0), (w - 1.0, 0.0), (w - 1.0, h - 1.0), (0.0, h - 1.0)]
    src = np.float32(corners)
    dst = []
    for px, py in corners:
        u, v = to_mm(px, py)
        p = H @ np.array([u, v, 1.0])
        dst.append([p[0] / p[2], p[1] / p[2]])
    Hw = cv2.getPerspectiveTransform(src, np.float32(dst))

    fh, fw = bg.shape[:2]
    warped = cv2.warpPerspective(canvas, Hw, (fw, fh), flags=cv2.INTER_LINEAR)
    amask = cv2.warpPerspective(alpha, Hw, (fw, fh), flags=cv2.INTER_LINEAR)

    a = (amask.astype(np.float32) / 255.0)[:, :, None]
    out = bg.astype(np.float32) * (1.0 - a) + warped.astype(np.float32) * a
    return np.clip(out, 0, 255).astype(np.uint8)


def make_case(frame_wh=(1920, 1080), hfov_deg=70.0, dist_mm=1000.0,
              color_name="red", offset_mm=(0.0, 0.0), roll_deg=0.0,
              pitch_deg=0.0, yaw_deg=0.0, v_scale=1.0, s_scale=1.0,
              bg_bgr=(148, 148, 148), grad=0.0, noise_sigma=0.0, vignette=0.0,
              blur_sigma=0.0, jpeg_q=0, rng=None,
              k1=0.0, k2=0.0, shadow=None, occlude=(), specular=None,
              lumpy=0.0, tgt_seed=0, inner_rings=False, distractors=(),
              distractor_rings=(), targets=None):
    """渲染一个用例 → (bgr, truth)。

    truth = {cx, cy, r_px, mm_per_px, dist_mm, color, 以及各退化参数}
      · cx/cy = **真值圆心**（全分辨率像素，亚像素浮点）
      · r_px  = 靶盘外缘的视半径（= f·R_DISC/D，正对时精确；有倾角时是沿该方向的）
      · mm_per_px = D/f —— 把像素误差换成靶面上的毫米，用来对着 10 环半径 7.5mm 判
    """
    w, h = frame_wh
    f_px = (w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)

    bg = make_background(frame_wh, bg_bgr, grad, rng, noise_sigma, vignette)
    if distractors:
        bg = apply_distractors(bg, distractors)
    if distractor_rings:
        bg = apply_ring_distractors(bg, distractor_rings)

    # targets = [(颜色, 靶心偏移mm, 距离mm), ...] —— 给了就摆一排（比赛现场就是三个并排）。
    # 跨色尺寸一致性判据**必须**有多个靶才谈得上，所以用例得能造出这种场面。
    tlist = list(targets) if targets else [(color_name, offset_mm, dist_mm)]
    img = bg
    for (cname, off, dist) in tlist:
        Ht = make_homography(f_px, frame_wh, dist, off, roll_deg, pitch_deg, yaw_deg)
        ppm = max(0.5, f_px / dist)
        canvas, cc, alpha = render_target_canvas(ppm, cname, v_scale, s_scale,
                                                 lumpy=lumpy, seed=tgt_seed,
                                                 inner_rings=inner_rings)
        img = compose(canvas, cc, alpha, ppm, Ht, img)

    # 真值只对**第一个**目标计误差；其余靶只作尺寸基准/干扰
    color_name, offset_mm, dist_mm = tlist[0]
    H = make_homography(f_px, frame_wh, dist_mm, offset_mm,
                        roll_deg, pitch_deg, yaw_deg)

    # 退化顺序贴近物理：光学模糊 → 镜头畸变 → 遮挡类（外部物体） → 传感器噪声(在背景里) → 压缩
    if blur_sigma > 0:
        k = int(2 * round(3 * blur_sigma) + 1)
        img = cv2.GaussianBlur(img, (k, k), blur_sigma)
    if k1 or k2:
        img = apply_lens_distortion(img, f_px, k1, k2)
    if shadow is not None:
        img = apply_shadow(img, shadow[0], shadow[1])
    if occlude:
        img = apply_occluders(img, occlude)
    if specular:
        img = apply_specular(img, specular)
    if jpeg_q:
        ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_q)])
        if ok:
            img = cv2.imdecode(enc, cv2.IMREAD_COLOR)

    p = H @ np.array([0.0, 0.0, 1.0])
    cx, cy = p[0] / p[2], p[1] / p[2]

    # ⚠️ 真值也要过一遍**同一个畸变映射**。否则量出来的是「畸变把靶心自己挪走的那一段」，
    #    而那段是**固定、可标定**的（会被瞄准点 (u0,v0) 吸收，不是算法误差）。
    #    真正该量的是「在畸变后的像里，算法推断出的圆心 与 靶心的畸变像 差多少」——
    #    也就是「环被挤成非圆之后，推断中心偏了多少」，那才是标定吃不掉的残差。
    if k1 or k2:
        px0, py0 = frame_wh[0] / 2.0, frame_wh[1] / 2.0
        xn = (cx - px0) / f_px
        yn = (cy - py0) / f_px
        r2 = xn * xn + yn * yn
        s = 1.0 + k1 * r2 + k2 * r2 * r2
        cx, cy = px0 + f_px * xn * s, py0 + f_px * yn * s

    truth = {
        "cx": float(cx), "cy": float(cy),
        "r_px": f_px * R_DISC / dist_mm,
        "mm_per_px": dist_mm / f_px,
        "dist_mm": dist_mm, "color": color_name,
        "roll_deg": roll_deg, "pitch_deg": pitch_deg, "yaw_deg": yaw_deg,
    }
    return img, truth
