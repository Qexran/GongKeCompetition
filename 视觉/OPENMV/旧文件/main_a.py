# -*- coding: utf-8 -*-
"""
单线巡线 —— 侧装摄像头视觉程序（固定测量区 + 上下参考色距离剖面过零）
================================================================
2026-09-13 换测量核心。旧法（find_blobs 取最大色块 → b.rotation 主轴当航向角、
bbox 极值边当交界行）已弃用：它读的是"像素云的形状"，块随光照/裁切一变就翻号。
新法见"测量核心"一段；它的验证记录与逐步推导在 angle_test.py 里。
2026-09-25 判别量再换（距离剖面）：单通道过中点在"阴影/褶皱 + 黑条"的场上会把
阴影边当成边界（uuu.bmp 帧 x≈180 锁到褶皱阴影边、x≈20/300 锁到黑条下沿），绿线跳。
改为每条横条的 (a,b) 到该列上/下参考色的距离²之差 f 过零 —— 阴影把色度压向原点
也不再翻面，黑条不再把落差劈成两半（推导见主循环第 3 步注释）。交界行定义挪了
几 px，上车后若车摆正时 offset 不为 0，按 angle_test.py 的流程重标 TARGET_OFFSET。
（本文件是 2026-09-25 的 A 改法试验版：只换判别量，其余与 main.py 一字不差。）

场地是「道路 + 边界区」的二分，摄像头侧装、镜头朝下，交界线横向（沿图像 X 轴）。

输出两个量（**帧格式与旧版一字不差，树莓派侧不需要任何改动**）：
  1. offset —— 横向位置偏差，归一化 [-100, 100]，0 = 交界线在目标行
  2. angle  —— 航向角偏差，单位 0.1°，0 = 与道路平行（"水平"）

串口文本帧（树莓派端 openmv_serial.py 解析）：
  "L,<offset>,<angle>,<valid>\r\n"   @115200
  按需发送：默认不发送，收到树莓派写来的控制字节才开始/停止：
    'G'(0x47) = 开始发送    'S'(0x53) = 停止发送
    'P'(0x50) = 拍一张照    存到相机盘根目录的「拍摄照片」（落盘逻辑见 photo.py）
  摄像头采样循环始终在跑（保持曝光/白平衡热状态），收到 'G' 后
  下一轮采样即开始发帧，延迟约一个采样周期（~60ms）。
  valid=0 时沿用上一帧的 offset/angle —— 丢线策略在树莓派侧（其 _line.py）。

⚠️ 新法**不需要任何 LAB 阈值**：两侧颜色每帧现算，所以换场地/换光照不用重标阈值。
"""

import sensor, image, time, math, sys
from pyb import UART
from machine import Pin, PWM

# 拍照落盘（存到相机盘根的「拍摄照片」）。
# ⚠️ 包 try: 导入失败**只关掉拍照**, 巡线照跑 —— 不能因为一个可选功能把整条链路带下去。
#    photo.py 就在盘根（与 main.py 同级）; 中文目录名能不能用由它自己探测（见 photo.py 顶部）。
try:
    import photo
except Exception as _e:
    photo = None
    print("[拍照] photo 模块导入失败, 拍照功能关闭（巡线不受影响）:", _e)

# ---------------- 传感器初始化 ----------------
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)      # 320x240
# 固定曝光/增益/白平衡，保证亮度确定、两侧色心稳定
sensor.set_auto_exposure(False, exposure_us=6000)   # 固定曝光时间(us)，需实测调
sensor.set_auto_gain(False, gain_db=10)              # 固定增益(dB)
sensor.set_auto_whitebal(False)                     # 固定白平衡
sensor.skip_frames(50)                              # 等设置生效
# 视实际安装方向，可能需要取消下面某几行注释（先实测再定）：
# sensor.set_hmirror(True)
# sensor.set_vflip(True)

LENS_CORR_STRENGTH = 2.0   # 镜头畸变校正强度：1.0~3.0 越大校正越强；0 表示关闭
LED_BRIGHTNESS    = 50     # LED 补光亮度 0~100（0=灭，100=最亮）

# ---------------- 测量区（固定在图像坐标里，别跟着线跑） ----------------
# 为什么必须固定：相机有倾角，同一个真实航向角在画面不同行读出的斜率不同、跨整幅能差
# 20%~3 倍；测量区固定后这个增益退化成**一个常数**，一次标定就吸收干净 → 与远近无关。
# 布局：顶部一条带隔离出来只做色心取样，其余全宽 + 一直到画面最底都是测量区。
FRAME_W, FRAME_H = 320, 240
MEAS_X     = (0, 320)      # 采样列范围（全宽 → 基线 320px）
MEAS_Y     = (0, 240)     # 测量区上下界：顶部隔离带之下 → 画面最底
TOP_BAND_H = 60            # 顶部隔离带高度（只取远侧色心，不参与找交界）
BOT_BAND_H = 35            # 底部色心带高度（贴画面最底取近侧色心）
NCOL       = 8             # 采样列数
COL_W      = 9             # 每列条带宽度(px)
STRIP      = 12            # 剖面的横条高度(px) → 每列 13 条

MARGIN_MIN = 20.0          # 两侧色心判别通道距离 < 此 → 测量区里没有交界 → valid=0
                           # （实测正常交界 32~40，故 20 是保守线；给太松会把"全灰/
                           #   全黄"那种情况放过去，吐出噪声角度）
CROSS_MIN_SWING_FRAC = 0.5 # 穿越处邻域(±2条=48行)总落差至少要到 margin 的这么多才认。
                           #   刻意不看"单步"：交界跨在两条分界上时会被劈成两小步，
                           #   卡单步会把真交界扔掉；远处那道弧/反光的假穿越总落差小
CROSS_SWING_DEPTH = 2      # 邻域取几条（2 → ±2 条）
RANSAC_TOL_PX     = 8.0    # 离群剔除的**固定**门槛(px)≈0.7×STRIP。刻意不用 MAD/3σ
                           #   那种"从数据自己算"的门槛：8 列里塞 2 个离群点就能把它
                           #   撑大、门槛放宽，结果一个都剔不掉（实测过）

CHECK_BLACK   = True       # 黑线交叉验证（只在练习地图有意义，标准地图给 0 个点，无害）
BLACK_MAX_L   = 38         # 黑线亮度上限
BLACK_MIN_DIP = 3.0        # 要比左右相邻条都暗这么多才算黑线

DRAW = True                # 画标记（调试看每列十字用）。独立跑想省几 ms 可关掉
PRINT_EVERY_S = 1.0        # 终端打印限速（原来每帧打，16fps 没人看得过来还占 USB）

# ---------------- 控制参数 ----------------
CTRL_X = FRAME_W // 2      # 控制列：offset 在这个 x 上求值（= 画面中心列）
TARGET_OFFSET = 44.0       # 目标横向偏移（相对画面中线=第120行，像素）。
                           # 44 → 目标交界行 = 164：2026-09-13 在实际场地把车摆正时
                           #   angle_test.py 读到的 行@160=164（六帧一致），故取 44。
                           # 想让车离交界更远 → 调小（目标交界在画面里更靠上）。
                           # ⚠️ 2026-09-25 换距离剖面后交界行定义挪了 ~3-5px（落在黑条
                           #   蓝色一侧的边缘），摆正时 offset 会系统性偏 +3~4，
                           #   上车按 angle_test.py 流程重校一次（预计 44 → 47 左右）。
ERR_GAIN = 200             # 误差映射系数：满偏(±半幅)对应 ±100（沿用旧版，Pi 侧不改）

# ---------------- 串口（双向：收控制字节 + 发文本帧） ----------------
uart = UART(3, 115200, timeout_char=1000)

# 按需发送开关：默认不发，等树莓派写 'G' 才开始，写 'S' 停止。
# 树莓派端巡线命令("2")触发 'G'，停止命令("3")/超时/断线触发 'S'。
SENDING = False

# ---------------- LED 补光（P6，PWM，常亮） ----------------
led = PWM(Pin("P6"), freq=1000, duty_u16=0)
led.duty_u16(int(LED_BRIGHTNESS / 100 * 65535))   # 按 LED_BRIGHTNESS 固定亮度

# 派生几何
WIN_X  = MEAS_X
WIN_Y  = MEAS_Y
NSTRIP = int((WIN_Y[1] - WIN_Y[0]) // STRIP)      # 13
TOP_BAND = (MEAS_X[0], 0, MEAS_X[1] - MEAS_X[0], TOP_BAND_H)
BOT_BAND = (MEAS_X[0], FRAME_H - BOT_BAND_H, MEAS_X[1] - MEAS_X[0], BOT_BAND_H)


# ==================== 测量核心 ====================
# 思路：逐列把测量区切成横条，用 get_statistics 取每条的 (a,b,L) **均值**，算各条到
# 该列"上/下参考色"的距离²之差 f，找 f 过零的那一行（相邻条线性插值），再对 8 个点
# 做抗离群拟合。2026-09-25 前是"单判别通道穿过两侧色心中点"，弃用原因见主循环第 3 步。
# 这里的 API 都是本机实测通过的：snapshot / lens_corr / get_statistics / draw_*
# （find_blobs 在这台机器上报 can't convert tuple to int，形态未定，故不用它）


def sval(st, *names):
    """按顺序找第一个存在的统计字段；OpenMV 5.0 里是属性、4.x 里是方法 —— 两种都吃。"""
    for nm in names:
        try:
            v = getattr(st, nm)
            return v() if callable(v) else v
        except Exception:
            pass
    raise Exception("统计字段都没有: %s" % (names,))


def lsq(pts):
    """最小二乘。线接近水平，row 当因变量才良态。返回 (斜率, 截距, 残差RMS)。"""
    n = len(pts)
    sx = sum(p[0] for p in pts)
    sy = sum(p[1] for p in pts)
    sxx = sum(p[0] * p[0] for p in pts)
    sxy = sum(p[0] * p[1] for p in pts)
    d = n * sxx - sx * sx
    if d == 0:
        return None
    m = (n * sxy - sx * sy) / d
    c = (sy * sxx - sx * sxy) / d
    e = math.sqrt(sum((p[1] - (m * p[0] + c)) ** 2 for p in pts) / n)
    return m, c, e


def col_profile(img, x):
    """该列的纵向剖面: [(行中心, a均值, b均值, L均值), ...]，从上到下。
    a、b 都要：判别在调用侧用"到上下参考色的距离²之差"做（见主循环第 3 步）——
    固定曝光下阴影会把单通道和黑条一起压向 0，单通道分不开"阴影边"和"真边界"。
    用 mean 不用 median：穿过交界那条横条里 median 会偏向多数那一侧，交点会"跳格"。"""
    out = []
    x0 = x - COL_W // 2
    for j in range(NSTRIP):
        y0 = WIN_Y[0] + j * STRIP
        st = img.get_statistics(roi=(x0, y0, COL_W, STRIP))
        out.append((y0 + STRIP / 2.0,
                    sval(st, 'a_mean', 'a_median'),
                    sval(st, 'b_mean', 'b_median'),
                    sval(st, 'l_mean', 'l_median')))
    return out


def cross_row(prof, mid, min_swing):
    """找剖面穿过 mid 的那一行（主循环喂的是"到上下参考色距离²之差 f"，mid=0）。
    一列里可能穿越多次（远处那道弧、反光、褶皱阴影边），
    取**邻域总落差最大**的那一次 —— 真正的材质交界落差最大。
    落差不足 min_swing 的一律不认（宁可这一列没点，也别塞个假的）。"""
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


def robust_fit(pts):
    """抗离群拟合：穷举 RANSAC —— 所有点对连成线，取"内点数最多"的那条，
    再用内点做 LSQ 精修、并按同一门槛复筛一次。8 列只有 28 对，开销可忽略。
    返回 (斜率, 截距, RMS, 保留点数, 剔除点数)；点数不够返回 None。"""
    n = len(pts)
    if n < 6:                      # 候选列少于 6 列不硬给角度（实测 5 点帧能差 8.6°）
        return None
    best = None
    for i in range(n - 1):
        for k in range(i + 1, n):
            dx = pts[k][0] - pts[i][0]
            if dx == 0:
                continue
            m = (pts[k][1] - pts[i][1]) / float(dx)
            c = pts[i][1] - m * pts[i][0]
            inl = [p for p in pts if abs(p[1] - (m * p[0] + c)) <= RANSAC_TOL_PX]
            if best is None or len(inl) > len(best):
                best = inl
    if best is None or len(best) < 5:
        return None
    f = lsq(best)
    if f is None:
        return None
    m2, c2 = f[0], f[1]
    inl2 = [p for p in pts if abs(p[1] - (m2 * p[0] + c2)) <= RANSAC_TOL_PX]
    if len(inl2) >= 5:
        f2 = lsq(inl2)
        if f2 is not None:
            return f2[0], f2[1], f2[2], len(inl2), n - len(inl2)
    return f[0], f[1], f[2], len(best), n - len(best)


def black_row(prof, near):
    """色阶行附近 ±3 条里找 L 最小的那条（练习地图的黑线），抛物线插值取行。"""
    k = int((near - WIN_Y[0]) / STRIP)
    lo, hi = max(0, k - 3), min(len(prof), k + 4)
    bi, best = None, None
    for j in range(lo, hi):
        if best is None or prof[j][3] < best:   # 剖面改成 (y,a,b,L) 四元组，L 在第 3 列
            best, bi = prof[j][3], j
    if bi is None or best > BLACK_MAX_L:
        return None
    if 0 < bi < len(prof) - 1:
        l, c, r = prof[bi - 1][3], prof[bi][3], prof[bi + 1][3]
        if c > l - BLACK_MIN_DIP or c > r - BLACK_MIN_DIP:
            return None                      # 不够"暗下去"→ 不是黑线
        d = l - 2 * c + r
        if d > 0:
            off = 0.5 * (l - r) / d
            if -1.0 <= off <= 1.0:
                return prof[bi][0] + off * STRIP
    return prof[bi][0]


# ---------------- 绘制：5.0 的 draw_* 要元组还是散参数，探一次就固定 ----------------
DRAW_STYLE = None


def probe_draw(im):
    """探一次 draw_* 形态。'tuple'=传元组，'flat'=散参数；都不行就不画（测量照跑）。"""
    global DRAW_STYLE
    forms = [('tuple', lambda: (im.draw_rectangle((8, 8, 6, 6), color=(255, 0, 0)),
                                im.draw_line((8, 20, 28, 20), color=(255, 0, 0)),
                                im.draw_cross((20, 30), color=(255, 0, 0), size=2))),
             ('flat', lambda: (im.draw_rectangle(8, 8, 6, 6, color=(255, 0, 0)),
                               im.draw_line(8, 20, 28, 20, color=(255, 0, 0)),
                               im.draw_cross(20, 30, color=(255, 0, 0), size=2)))]
    for name, fn in forms:
        try:
            fn()
            print("自检: draw_* 用 '%s' 形态" % name)
            DRAW_STYLE = name
            return name
        except Exception as e:
            print("自检: draw_* '%s' 形态不可用: %s" % (name, e))
    return None


def d_rect(im, r, col):
    if DRAW_STYLE == 'tuple':
        im.draw_rectangle(r, color=col)
    elif DRAW_STYLE == 'flat':
        im.draw_rectangle(r[0], r[1], r[2], r[3], color=col)


def d_line(im, p, col):
    if DRAW_STYLE == 'tuple':
        im.draw_line(p, color=col)
    elif DRAW_STYLE == 'flat':
        im.draw_line(p[0], p[1], p[2], p[3], color=col)


def d_cross(im, x, y, col, sz=4):
    if DRAW_STYLE == 'tuple':
        im.draw_cross((x, y), color=col, size=sz)
    elif DRAW_STYLE == 'flat':
        im.draw_cross(x, y, color=col, size=sz)


# ==================== 主循环 ====================

clock = time.clock()
last_offset = 0
last_angle  = 0
last_print  = time.time()
want_photo  = False        # 收到 'P' 置起, 本帧处理完存图后清掉（见第 6.5 步）

img = sensor.snapshot()
print("DEBUG %dx%d 测量区 x=%s y=%s 上带=%s 下带=%s 列数%d 列宽%d 横条%d 目标行%d"
      % (img.width(), img.height(), WIN_X, WIN_Y, TOP_BAND, BOT_BAND, NCOL, COL_W,
         NSTRIP, int(FRAME_H // 2 + TARGET_OFFSET)))
if DRAW:
    probe_draw(img)

while True:
    # 按需发送控制：轮询树莓派写来的控制字节（'G'=开始发, 'S'=停止发）。
    # 放循环最前，保证触发后本周期即生效（延迟 ≤ 一个采样周期）。
    if uart.any():
        c = uart.read(1)
        if c == b'G':
            SENDING = True
        elif c == b'S':
            SENDING = False
        elif c == b'P':
            want_photo = True      # 只置标志: 本轮这帧处理完再存（要存"刚测过的那一帧"）

    clock.tick()
    img = sensor.snapshot()
    img = img.lens_corr(strength=LENS_CORR_STRENGTH)   # 镜头畸变校正

    # valid=0 时沿用上一帧（丢线策略在树莓派侧）
    valid  = False
    offset = last_offset
    angle  = last_angle
    stage  = '0'

    try:
        # ---- 1) 两侧色心：用中位数（对"交界跑进色心带"这种污染不敏感）----
        stage = '1 色心'
        su = img.get_statistics(roi=TOP_BAND)
        sd = img.get_statistics(roi=BOT_BAND)
        a_up, b_up = sval(su, 'a_median', 'a_mean'), sval(su, 'b_median', 'b_mean')
        a_dn, b_dn = sval(sd, 'a_median', 'a_mean'), sval(sd, 'b_median', 'b_mean')
        l_up = sval(su, 'l_median', 'l_mean')
        l_dn = sval(sd, 'l_median', 'l_mean')
        margin = max(abs(a_up - a_dn), abs(b_up - b_dn))

        # ---- 2) 自动选判别通道：a 和 b 里分得更开的那个 ----
        stage = '2 通道'
        is_a = abs(a_up - a_dn) >= abs(b_up - b_dn)
        ch = 'a' if is_a else 'b'
        # 有效条件：两侧确实分得开（"全灰/全黄"时 margin 会塌掉）
        enough = margin >= MARGIN_MIN

        # ---- 3) 逐列取剖面 → 找交界行 ----
        # 判别量（2026-09-25 起）：每条横条的 (a,b) 到"上参考色"和"下参考色"的距离²
        # 之差 f，f<0 离上侧近、f>0 离下侧近，过零即交界。
        # 为什么弃掉"单通道过中点"：固定曝光下阴影=欠曝，L 和 |a|,|b| 一起塌向 0，
        # 黑条也是 0 —— 单通道剖面里"阴影边"和"真边界"是同一种特征，落差大的那个赢，
        # 就锁错（uuu.bmp 帧实证：x≈180 锁到褶皱阴影边、x≈20/300 锁到黑条下沿）。
        # 距离剖面下，阴影/褶皱只是把色点往原点压：下侧饱和、上侧弱色时，压向黑的点
        # 仍离上侧参考近 → 阴影边不过零；色→黑→色的 f 全程单调，落差不再被劈成两半。
        # 且 f 不再有 own_mid 这个可漂移的标量中点 —— 判别面是上下参考色连线的中垂面，
        # own_mid 被 LED 热点/阴影推着跑而让褶皱边翻面的失效模式整个不存在了。
        stage = '3 逐列'
        pts, bpts = [], []
        for i in range(NCOL):
            x = int(WIN_X[0] + (WIN_X[1] - WIN_X[0]) * (i + 0.5) / NCOL)
            prof = col_profile(img, x)
            # 参考色仍取"该列自己"顶/底各 3 条的均值 —— 延续旧 own_mid 的实测结论：
            # 补光/白平衡的横向渐变（空间不均）被逐列抵消，不丢列、不把角度带偏。
            ta = (prof[0][1] + prof[1][1] + prof[2][1]) / 3.0
            tb = (prof[0][2] + prof[1][2] + prof[2][2]) / 3.0
            ba = (prof[-1][1] + prof[-2][1] + prof[-3][1]) / 3.0
            bb = (prof[-1][2] + prof[-2][2] + prof[-3][2]) / 3.0
            da, db = ba - ta, bb - tb
            d2 = da * da + db * db              # 该列两侧参考色距离² ≈ f 满程的一半
            f = [((p[1] - ta) ** 2 + (p[2] - tb) ** 2)
                 - ((p[1] - ba) ** 2 + (p[2] - bb) ** 2) for p in prof]
            # 闸与旧版同语义：取"满程落差估计"的 CROSS_MIN_SWING_FRAC 倍。满程估计 =
            # max(2×该列参考距², margin²) —— 后者是绝对下限：整列都在一侧时 d2 会很小，
            # 不能被噪声糊过去。margin² 只当满程的保守一半用（真实满程 ≥ 2×margin²），
            # 直接当满程会把 LED 热点列（参考距偏小）的好点误杀（uuu.bmp 实测差 8%）
            row = cross_row([(prof[j][0], f[j]) for j in range(NSTRIP)], 0.0,
                            CROSS_MIN_SWING_FRAC * max(2.0 * d2, margin * margin))
            if row is not None and WIN_Y[0] <= row <= WIN_Y[1]:
                pts.append((x, row))
                if CHECK_BLACK:
                    brow = black_row(prof, row)
                    if brow is not None and WIN_Y[0] <= brow <= WIN_Y[1]:
                        bpts.append((x, brow))

        # ---- 4) 抗离群拟合 ----
        stage = '4 拟合'
        fa = robust_fit(pts)
        fb = robust_fit(bpts) if CHECK_BLACK else None

        # ---- 5) 出量：同一个拟合同时给 offset 和 angle（同源，不会互相打架）----
        stage = '5 出量'
        if fa and enough:
            m, c, e, nk, nd = fa
            lat_span = FRAME_H
            row_ctrl = m * CTRL_X + c                      # 控制列处的交界行
            error_px = (row_ctrl - lat_span / 2.0) - TARGET_OFFSET
            offset = int(error_px / lat_span * 2 * ERR_GAIN)
            offset = max(-100, min(100, offset))

            angle = int(math.degrees(math.atan(m)) * 10)   # 0.1° 单位
            angle = max(-900, min(900, angle))

            valid = True
            last_offset = offset
            last_angle = angle

        # ---- 6) 可视化（调试用，不影响出量）----
        if DRAW:
            try:
                d_rect(img, TOP_BAND, (0, 255, 255))
                d_rect(img, BOT_BAND, (0, 255, 255))
                d_line(img, (WIN_X[0], WIN_Y[0], WIN_X[1], WIN_Y[0]), (0, 255, 255))
                for p in pts:
                    d_cross(img, p[0], int(p[1]), (255, 255, 0), 4)
                for p in bpts:
                    d_cross(img, p[0], int(p[1]), (255, 0, 0), 2)
                if fa:
                    d_line(img, (WIN_X[0], int(fa[0] * WIN_X[0] + fa[1]), WIN_X[1],
                                 int(fa[0] * WIN_X[1] + fa[1])), (0, 255, 0))
            except Exception as de:
                print("[巡线] 绘制失败(不再尝试):", de)
                DRAW = False

        # ---- 6.5) 拍照（收到 'P' 时把**刚测过的这一帧**存到相机盘）----
        #    放在绘制之后: 存下来的照片与 IDE 里看到的一致（带黄十字/绿拟合线）,
        #    "当时每列读到什么"一眼可见 —— 这正是不对劲时最想留的证据。
        #    想要干净原图, 把这三行挪到第 6 步之前即可（位置就是开关）。
        #    ⚠️ photo.take 自己吞掉所有异常, 存图失败不会影响本帧出量。
        if want_photo:
            want_photo = False
            if photo is not None:
                photo.take(img)
            else:
                print("[拍照] photo 模块不可用, 忽略本次 'P'")

        # ---- 7) 状态输出（限速；只在 SENDING 之外也打，方便台架上不上位机时看）----
        if time.time() - last_print > PRINT_EVERY_S:
            last_print = time.time()
            msg = "valid=%d off=%4d ang=%+6.1f 点%d剔%d 列%d/%d 通道%c 距%.0f %.0ffps" % (
                1 if valid else 0, offset, angle / 10.0,
                fa[3] if fa else 0, fa[4] if fa else 0, len(pts), NCOL,
                ch, margin, clock.fps())
            if fb:
                msg += " |黑线%+6.1f" % math.degrees(math.atan(fb[0]))
            print(msg)

    except Exception as ex:
        print("[巡线] 出错 @%s: %s" % (stage, ex))
        try:
            sys.print_exception(ex)
        except Exception:
            pass
        valid = False                    # 出错也要继续发帧（valid=0），别让链路哑掉

    # ---- 8) 串口输出（文本帧，树莓派端 openmv_serial.py 解析）----
    #    格式: L,<offset>,<angle>,<valid>\r\n   —— 仅 SENDING 时发送
    if SENDING:
        uart.write("L,%d,%d,%d\r\n" % (offset, angle, 1 if valid else 0))
