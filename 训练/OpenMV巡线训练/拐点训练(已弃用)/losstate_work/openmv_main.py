# -*- coding: utf-8 -*-
"""
单线巡线 —— 侧装摄像头视觉程序（固定测量区 + 两侧色心中点穿越）
================================================================
2026-09-13 换测量核心。旧法（find_blobs 取最大色块 → b.rotation 主轴当航向角、
bbox 极值边当交界行）已弃用：它读的是"像素云的形状"，块随光照/裁切一变就翻号。
新法见"测量核心"一段；它的验证记录与逐步推导在 angle_test.py 里。

场地是「道路 + 边界区」的二分，摄像头侧装、镜头朝下，交界线横向（沿图像 X 轴）。

输出两个量（**帧格式与旧版一字不差，树莓派侧不需要任何改动**）：
  1. offset —— 横向位置偏差，归一化 [-100, 100]，0 = 交界线在目标行
  2. angle  —— 航向角偏差，单位 0.1°，0 = 与道路平行（"水平"）

串口文本帧（树莓派端 openmv_serial.py 解析）：
  "L,<offset>,<angle>,<valid>,<state>\r\n"   @115200
  第 5 字段 state 是 2026-09 加的丢线分类（旧端只取前 4 个也能跑）：
    0 = 正常直线测量        1 = 有效但交界是曲线（路口/弯道，已用近半段重拟合）
    2 = 丢线：全画面是路面色/交界点全在画面上方（边界远出视野 → 该向边界横移）
    3 = 丢线：全画面是边界区色/交界点全在画面下方（压到黄区 → 该向路内横移）
    4 = 丢线：方向不明（还没采到两侧参考色 / 点和颜色都说不清）
  按需发送：默认不发送，收到树莓派写来的控制字节才开始/停止：
    'G'(0x47) = 开始发送    'S'(0x53) = 停止发送
    'P'(0x50) = 拍一张照    存到相机盘根目录的 photos/（落盘逻辑见 photo.py）
                            存的是**干净原图**: 在全部图像处理之后、绘制调试标记之前
  回执（本端 → 树莓派）：存盘后回写一行 "P,<ok>"（1=成功 0=失败）——
    树莓派据此回传单片机（COORD TAG=0x06）, OLED 才闪「拍照 成功!/失败!」。
    ⚠️ 这是本端**唯一**主动发的非巡线帧; 与 'G'/'S' 的发送开关无关（那两个管巡线帧）。
  摄像头采样循环始终在跑（保持曝光/白平衡热状态），收到 'G' 后
  下一轮采样即开始发帧，延迟约一个采样周期（~60ms）。
  valid=0 时沿用上一帧的 offset/angle —— 丢线策略在树莓派侧（其 _line.py），
  它按 state 决定往哪边找线（不再是死写的固定方向）。

⚠️ 新法**不需要任何 LAB 阈值**：两侧颜色每帧现算，所以换场地/换光照不用重标阈值。
"""

import sensor, image, time, math, sys
from pyb import UART
from machine import Pin, PWM

# 拍照落盘（存到相机盘根的 photos/）。
# ⚠️ 包 try: 导入失败**只关掉拍照**, 巡线照跑 —— 不能因为一个可选功能把整条链路带下去。
#    photo.py 就在盘根（与 main.py 同级）; 目录名必须 ASCII（这台固件的 FatFS 是 CP437
#    模式, 中文名会变成乱码孪生目录 —— 实测记录见 photo.py 顶部「坑 1②」）。
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
sensor.set_auto_exposure(False, exposure_us=4000)   # 固定曝光时间(us)，需实测调
sensor.set_auto_gain(False, gain_db=15)              # 固定增益(dB)
sensor.set_auto_whitebal(False)                     # 固定白平衡
sensor.skip_frames(50)                              # 等设置生效
# 视实际安装方向，可能需要取消下面某几行注释（先实测再定）：
# sensor.set_hmirror(True)
# sensor.set_vflip(True)

LENS_CORR_STRENGTH = 3.0   # 镜头畸变校正强度：1.0~3.0（实车确认要开, 别动）
LED_BRIGHTNESS    = 50     # LED 补光亮度 0~100（0=灭，100=最亮）

# ---------------- 测量区（固定在图像坐标里，别跟着线跑） ----------------
# 为什么必须固定：相机有倾角，同一个真实航向角在画面不同行读出的斜率不同、跨整幅能差
# 20%~3 倍；测量区固定后这个增益退化成**一个常数**，一次标定就吸收干净 → 与远近无关。
# 布局：顶部一条带隔离出来只做色心取样，其余全宽 + 一直到画面最底都是测量区。
FRAME_W, FRAME_H = 320, 240
MEAS_X     = (0, 320)      # 采样列范围（全宽 → 基线 320px）
MEAS_Y     = (80, 240)     # 测量区上下界：顶部隔离带之下 → 画面最底
TOP_BAND_H = 40            # 顶部隔离带高度（只取远侧色心，不参与找交界）
                           # 2026-09-22 从 60 砍到 40: 带越高像素越多; 边界最高也就到
                           # 测量区上沿(80), 40 行的"上方纯色带"足够, 且中位数抗污染
BOT_BAND_H = 24            # 底部色心带高度（贴画面最底取近侧色心）, 同上从 35 砍到 24
NCOL       = 8             # 采样列数
COL_W      = 9             # 每列条带宽度(px)
STRIP      = 20            # 剖面的横条高度(px) → 每列 8 条（2026-09-22 从 12/13 条砍过来:
                           # get_statistics 每帧 106→66 次, 这是调用数大头。交界的行靠条间
                           # 线性插值, 不靠条数堆; 20px 条对"褶皱假穿越"反而更不容易采到）

MARGIN_MIN = 20.0          # 两侧色心判别通道距离 < 此 → 测量区里没有交界 → valid=0
                           # （实测正常交界 32~40，故 20 是保守线；给太松会把"全灰/
                           #   全黄"那种情况放过去，吐出噪声角度）
CROSS_MIN_SWING_FRAC = 0.5 # 穿越处邻域(±2条=48行)总落差至少要到 margin 的这么多才认。
                           #   刻意不看"单步"：交界跨在两条分界上时会被劈成两小步，
                           #   卡单步会把真交界扔掉；远处那道弧/反光的假穿越总落差小
CROSS_SWING_DEPTH = 1      # 邻域取几条（1 → ±1 条）。条带从 12px 变 20px 后 ±1 条
                           # 的邻域宽度(±20px) 与旧版 ±2×12px 基本等效, 别跟着改回 2
RANSAC_TOL_PX     = 8.0    # 离群剔除的**固定**门槛(px)≈0.7×STRIP。刻意不用 MAD/3σ
                           #   那种"从数据自己算"的门槛：8 列里塞 2 个离群点就能把它
                           #   撑大、门槛放宽，结果一个都剔不掉（实测过）

CHECK_BLACK   = True       # 黑线交叉验证（只在练习地图有意义，标准地图给 0 个点，无害）
BLACK_MAX_L   = 38         # 黑线亮度上限
BLACK_MIN_DIP = 3.0        # 要比左右相邻条都暗这么多才算黑线

# ---- 丢线分类（state 2/3/4，见文件头帧格式）与曲线重拟合（state 1）----
# 为什么要分类：交界跑出画面后只能吐 valid=0，旧端分不清「往哪边丢的」，找线方向只能
# 死写。现在两级分类，先用证据硬的：
#   ① 还有零星交界点（不足拟合的 6 个）→ 看点的**行位置**：全在画面上方 = 边界远出上边
#      （车偏进路里）；全在下方 = 边界压到下边（车骑到黄区上）。
#   ② 一个点都没有 → 看整帧**颜色**更像哪一侧：每帧有效时把上/下带色心 EMA 记下来当
#      参考色，丢线帧拿上/下带各自的 (a,b) 色度距离去比（2026-09-22 用实拍 7 张验过：
#      两面色心距离 ≈65，压黄区整帧到黄区参考仅 ≈10，故门槛 25 有两倍多余量）。
# 只比 a/b 不比 L：LED 常亮 + 固定曝光下两面亮度本来就差一大截、阴影主要动 L，
# 色度（黄 vs 蓝）才是稳的判别。
LOST_EMA        = 0.25     # 参考色 EMA 系数（新帧占比）
LOST_CLASS_GATE = 25.0     # 色心到参考的 (a,b) 距离超过它 = 这个带"哪面都不像" → 不投票
FAR_ROW  = 155             # 零星交界点的中位行 < 它 → 边界远出画面上方（state 2）
NEAR_ROW = 175             # 零星交界点的中位行 > 它 → 边界压到画面下方（state 3）
                           # （目标行 164，±~10 是"其实还挺准"的区间，落在中间就报 4）
NEAR_HALF_ROW = (80 + 240) / 2.0   # 曲线重拟合取"近半段"的分界行（=160，靠车这半边）

CURVE_RMS_PX = 5.0         # 拟合残差 RMS 超它 → 判为曲线/路口（state 1），并用**近半段**
                           # 内点重拟合：航向该跟"车马上要进的这段"的切线，而不是整条弧的
                           # 平均切线（lens_corr 开着, 直线 RMS 只有 1~2px, 此分支基本不进）。

# ---- 褶皱/换色鲁棒性（2026-09-22 第二轮，实车灰地图反馈）----
# 实车两个症状: ① 黄区→灰区过渡时斜率跳变 ② 路口/灰地图上地板褶皱产生假交界点。
# 四道对策, 全部只动"选点/出量", 不动拟合核心, 且都带兜底（门杀过头自动退回旧行为）:
CH_HYST       = 1.3        # ① 判别通道滞回: 换通道要 1.3 倍优势。过渡区 a/b 两通道分离度
                           #   接近, 逐帧取优会在两通道间抖 → 交界行整列跳 → 斜率跳变
TRACK_GATE_PX = 35.0       # ② 追踪门: 与上一有效帧拟合线的偏差超过它的穿越不认
                           #   （实拍褶皱假点离真线 80~120px, 一抓一个准; 无轨迹时不设门）
TRACK_GATE_T  = 0.3        #    上帧拟合的新鲜度(秒), 超过视为丢了轨迹, 不设门
ANGLE_SLEW    = 120        # ③ 航向角每帧限速(0.1°/帧): 12°/帧 ≈ 每秒190°, 真车到不了,
                           #   残余的任何跳变源(换色瞬间/点群翻转)都被挡在输出端
                           # ④ 颜色身份门在 classify_lost 一节（side_color_ok）

DRAW = True                # 画标记（调试看每列十字用）。独立跑想省几 ms 可关掉
PRINT_EVERY_S = 1.0        # 终端打印限速（原来每帧打，16fps 没人看得过来还占 USB）
PROF_EVERY_S = 2.0         # 分阶段耗时剖析（秒, 0=关）: 每隔这么久打一行 snap/corr/带/列/
                           # 拟合/其余 的均值 —— 提帧率先看它, 谁大砍谁, 别凭感觉砍

# ---------------- 控制参数 ----------------
CTRL_X = FRAME_W // 2      # 控制列：offset 在这个 x 上求值（= 画面中心列）
TARGET_OFFSET = 44.0       # 目标横向偏移（相对画面中线=第120行，像素）。
                           # 44 → 目标交界行 = 164：2026-09-13 在实际场地把车摆正时
                           # angle_test.py 读到的 行@160=164（六帧一致），故取 44。
                           # 想让车离交界更远 → 调小（目标交界在画面里更靠上）。
ERR_GAIN = 200             # 误差映射系数：满偏(±半幅)对应 ±100（沿用旧版，Pi 侧不改）

# ---------------- 串口（双向：收控制字节 + 发文本帧） ----------------
uart = UART(3, 115200, timeout_char=1000)

# 按需发送开关：默认不发，等树莓派写 'G' 才开始，写 'S' 停止。
# 树莓派端巡线命令("2")触发 'G'，停止命令("3")/超时/断线触发 'S'。
SENDING = False

# ---------------- LED 补光（P6，PWM，常亮） ----------------
# ⚠️ 频率必须在可听域之外（≥20kHz）: 1kHz 时驱动管/电感每秒开关一千次,
#    会发出人耳可闻的"小蜂鸣"（实车确认）。20kHz 周期 50µs ≪ 曝光 4ms,
#    一帧平均 80 个开关周期 —— 光强依旧稳, 相机侧无任何影响, 只是听不见了。
led = PWM(Pin("P6"), freq=20000, duty_u16=0)
led.duty_u16(int(LED_BRIGHTNESS / 100 * 65535))   # 按 LED_BRIGHTNESS 固定亮度

# 派生几何
WIN_X  = MEAS_X
WIN_Y  = MEAS_Y
NSTRIP = int((WIN_Y[1] - WIN_Y[0]) // STRIP)      # 13
TOP_BAND = (MEAS_X[0], 0, MEAS_X[1] - MEAS_X[0], TOP_BAND_H)
BOT_BAND = (MEAS_X[0], FRAME_H - BOT_BAND_H, MEAS_X[1] - MEAS_X[0], BOT_BAND_H)


# ==================== 测量核心 ====================
# 思路：逐列把测量区切成横条，用 get_statistics 取判别通道的**均值**剖面，找"通道穿过
# 两侧色心中点"的那一行（相邻条线性插值），再对 8 个点做抗离群拟合。
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
    用 mean 不用 median：穿过交界那条横条里 median 会偏向多数那一侧，交点会"跳格"。
    a/b 两个通道都存：判别通道每帧现选（带滞回），颜色身份门两侧也都要用。"""
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


def crossings(prof, ch_i, mid, min_swing):
    """该列**全部**的 mid 穿越候选 [(swing, row, j), ...]，按 swing 降序。
    一列里可能穿越多次（远处那道弧、反光、地板缝、褶皱），选哪个交给调用方：
    颜色身份门/追踪门先挑，全被挑掉还有整帧兜底（退回旧行为取 swing 最大者）。
    落差不足 min_swing 的不收（宁可这一列没点，也别塞个假的）。"""
    out = []
    n = len(prof)
    for j in range(n - 1):
        b0, b1 = prof[j][ch_i], prof[j + 1][ch_i]
        if (b0 - mid) * (b1 - mid) <= 0 and abs(b1 - b0) > 1e-6:
            seg = [prof[k][ch_i] for k in range(max(0, j - CROSS_SWING_DEPTH),
                                                min(n, j + 2 + CROSS_SWING_DEPTH))]
            swing = max(seg) - min(seg)
            if swing >= min_swing:
                row = prof[j][0] + (mid - b0) / (b1 - b0) * STRIP
                out.append((swing, row, j))
    out.sort(key=lambda t: -t[0])
    return out


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
        if best is None or prof[j][3] < best:
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


# ---------------- 丢线分类（state 2/3/4）：参考色 + 两级判据 ----------------
# 参考色只在**有效帧**里更新：上带色心 = 边界区那面，下带色心 = 路面那面。
# 用中位数色心（stage 1 已算好）再做 EMA， lighting 慢漂能跟住，单帧污染进不来。
ref_bnd   = None            # (a, b) 边界区参考色；None = 还没见过有效帧
ref_road  = None            # (a, b) 路面参考色
last_is_a = None            # 上帧的判别通道（通道滞回用）
last_fit  = None            # (m, c, t) 上一有效帧的拟合线 —— 追踪门的轨迹基准


def ema_ref(old, new):
    if old is None:
        return new
    k = LOST_EMA
    return (old[0] + (new[0] - old[0]) * k, old[1] + (new[1] - old[1]) * k)


def chrom_d(p, r):
    da = p[0] - r[0]
    db = p[1] - r[1]
    return math.sqrt(da * da + db * db)


def side_color_ok(prof, j):
    """穿越 prof[j]→prof[j+1] 的颜色身份门：上侧该像边界区色、下侧该像路面色。
    没有参考色（开机还没见过有效帧）一律放行。取样刻意跳过紧贴穿越的两条
    （它们被交界本身污染），取 j-1 / j+2。褶皱、黄灰换色缝这类"同侧内伤"
    两侧同色，身份必有一边对不上 —— 要拦的就是它。"""
    if ref_bnd is None or ref_road is None:
        return True
    n = len(prof)
    pa = prof[j - 1] if j > 0 else prof[j]
    pb = prof[j + 2] if j + 2 < n else prof[j + 1]
    ca = (pa[1], pa[2])
    cb = (pb[1], pb[2])
    return (chrom_d(ca, ref_bnd) < chrom_d(ca, ref_road) and
            chrom_d(cb, ref_road) < chrom_d(cb, ref_bnd))


def classify_lost(pts, a_up, b_up, a_dn, b_dn):
    """丢线帧定方向 → 2（边界远出画面）/3（压到黄区）/4（说不清）。
    两级：先看零星交界点的行位置（证据硬，不看颜色），点也没有才比参考色。
    颜色票由上、下带**各自**投，两带一致才认 —— 交界其实还在画面里（只是没测到）
    时两带必是一面一色，硬猜不如报 4 交回旧策略。"""
    if pts:
        rows = sorted(p[1] for p in pts)
        med = rows[len(rows) // 2]
        if med < FAR_ROW:
            return 2
        if med > NEAR_ROW:
            return 3
    if ref_bnd is None or ref_road is None:
        return 4                                   # 开机以来还没有效帧，没有参考色
    d_ut, d_ur = chrom_d((a_up, b_up), ref_bnd), chrom_d((a_up, b_up), ref_road)
    d_dt, d_dr = chrom_d((a_dn, b_dn), ref_bnd), chrom_d((a_dn, b_dn), ref_road)
    ok_t = min(d_ut, d_ur) <= LOST_CLASS_GATE
    ok_b = min(d_dt, d_dr) <= LOST_CLASS_GATE
    if ok_t and ok_b:
        top_road = d_ur < d_ut
        if top_road == (d_dr < d_dt):
            return 2 if top_road else 3
    return 4


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
want_photo  = False        # 收到 'P' 置起, 本帧处理完存图后清掉（见第 5.5 步）
_prof   = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]   # 各阶段耗时累计(ms): snap/corr/带/列/拟合/其余
_prof_n = 0
_prof_t = time.ticks_us()

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
    _t0 = time.ticks_us()
    img = sensor.snapshot()
    _t1 = time.ticks_us()
    if LENS_CORR_STRENGTH >= 0.5:      # 0 = 跳过整帧重映射（最大的单块开销, 见常量处注释）
        img = img.lens_corr(strength=LENS_CORR_STRENGTH)   # 镜头畸变校正
    _t2 = time.ticks_us()

    # valid=0 时沿用上一帧（丢线策略在树莓派侧）
    valid   = False
    offset  = last_offset
    angle   = last_angle
    state_v = 4                   # 默认"方向不明"; 下面各步给出更准的值, 出错也落在这里
    stage   = '0'

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

        # ---- 2) 自动选判别通道：a 和 b 里分得更开的那个（带滞回）----
        stage = '2 通道'
        a_m, b_m = abs(a_up - a_dn), abs(b_up - b_dn)
        is_a = a_m >= b_m
        if last_is_a is not None and is_a != last_is_a:
            # 换道要 1.3 倍优势才准换：黄区↔灰区过渡时两通道的分离度接近，
            # 逐帧取优会在两通道间抖动 → 交界行整列跳 → 斜率跳变（实车症状①）
            is_a = last_is_a if max(a_m, b_m) < CH_HYST * min(a_m, b_m) else is_a
        last_is_a = is_a
        ch = 'a' if is_a else 'b'
        ch_i = 1 if is_a else 2
        # 有效条件：两侧确实分得开（"全灰/全黄"时 margin 会塌掉）
        enough = margin >= MARGIN_MIN
        _t2b = time.ticks_us()

        # ---- 3) 逐列取剖面 → 挑交界点（颜色身份门 ∨ 追踪门，整帧兜底）----
        stage = '3 逐列'
        pts_ok, pts_any, bpts = [], [], []
        now_t = time.time()
        for i in range(NCOL):
            x = int(WIN_X[0] + (WIN_X[1] - WIN_X[0]) * (i + 0.5) / NCOL)
            prof = col_profile(img, x)
            # 每列用自己的中点：拿该列剖面上下两端的均值当"该列自己的两侧色心"。
            # 这样判别通道的横向渐变（补光/白平衡在空间上不均）被抵消 —— 实测对渐变
            # 完全免疫；用全局中点时渐变既丢列又把角度带偏（ramp=30 时漂 8.8°）。
            b_top = (prof[0][ch_i] + prof[1][ch_i] + prof[2][ch_i]) / 3.0
            b_bot = (prof[-1][ch_i] + prof[-2][ch_i] + prof[-3][ch_i]) / 3.0
            own_mid = (b_top + b_bot) / 2.0
            own_spread = abs(b_top - b_bot)
            # 闸取"该列自身落差的 0.5 倍"与"全局 margin 的 0.5 倍"里更大的那个 ——
            # 后者是绝对下限：整列都在一侧时 own_spread 很小，不能被噪声糊过去
            cands = crossings(prof, ch_i, own_mid,
                              CROSS_MIN_SWING_FRAC * max(own_spread, margin))
            # 追踪门基准：上一有效帧的拟合线在这列的行（只在轨迹新鲜时设门）
            track = None
            if last_fit is not None and now_t - last_fit[2] <= TRACK_GATE_T:
                track = last_fit[0] * x + last_fit[1]
            row_g = row_a = None
            for _swing, row, j in cands:          # 门后点: 颜色身份 ∨ 追踪门
                if not (WIN_Y[0] <= row <= WIN_Y[1]):
                    continue
                if side_color_ok(prof, j) or (
                        track is not None and abs(row - track) <= TRACK_GATE_PX):
                    row_g = row
                    break
            for _swing, row, j in cands:          # 兜底点: 旧行为(swing 最大者)
                if WIN_Y[0] <= row <= WIN_Y[1]:
                    row_a = row
                    break
            if row_g is not None:
                pts_ok.append((x, row_g))
            if row_a is not None:
                pts_any.append((x, row_a))
            if CHECK_BLACK and (row_g is not None or row_a is not None):
                brow = black_row(prof, row_g if row_g is not None else row_a)
                if brow is not None and WIN_Y[0] <= brow <= WIN_Y[1]:
                    bpts.append((x, brow))
        if len(pts_ok) >= 6:
            pts = pts_ok        # 门后点够拟合 → 用净的（褶皱假点已在源头被拦）
        elif len(pts_any) >= 6:
            pts = pts_any       # 门后不够 → 退回旧行为，让 RANSAC 自己抗离群
        else:
            pts = pts_ok if len(pts_ok) >= len(pts_any) else pts_any
        _t3 = time.ticks_us()

        # ---- 4) 抗离群拟合（+ 曲线判别与近半段重拟合）----
        stage = '4 拟合'
        fa = robust_fit(pts)
        fb = robust_fit(bpts) if CHECK_BLACK else None
        if fa and fa[2] > CURVE_RMS_PX:
            # 残差大 = 交界在这窗里是条弧（路口/弯道）。航向该跟**车马上要进的近段**
            # 的切线 —— 取全局拟合内点里行 > 160 的那半重拟合; 近半段没凑够 4 个点
            # 或没更直就保留全局拟合（尽力而为）。偏移仍按控制列求, 但行被夹回测量窗。
            near = [p for p in pts
                    if p[1] > NEAR_HALF_ROW
                    and abs(p[1] - (fa[0] * p[0] + fa[1])) <= RANSAC_TOL_PX]
            if len(near) >= 4:
                fn = lsq(near)
                if fn is not None and fn[2] < fa[2]:
                    fa = (fn[0], fn[1], fn[2], len(near), len(pts) - len(near))

        # ---- 5) 出量：同一个拟合同时给 offset 和 angle（同源，不会互相打架）----
        stage = '5 出量'
        if fa and enough:
            m, c, e, nk, nd = fa
            lat_span = FRAME_H
            row_ctrl = m * CTRL_X + c                      # 控制列处的交界行
            # 夹回测量窗：直线上行跑不出去, 曲线帧近段切线外推到控制列可能甩到窗外,
            # 不夹会把放大成满偏的 offset 交给 Pi —— 方向不会错, 大小会虚
            if row_ctrl < WIN_Y[0]:
                row_ctrl = WIN_Y[0]
            elif row_ctrl > WIN_Y[1]:
                row_ctrl = WIN_Y[1]
            error_px = (row_ctrl - lat_span / 2.0) - TARGET_OFFSET
            offset = int(error_px / lat_span * 2 * ERR_GAIN)
            offset = max(-100, min(100, offset))

            angle_raw = int(math.degrees(math.atan(m)) * 10)   # 0.1° 单位
            # 每帧限速：换色瞬间/点群翻转这类残余跳变被挡在输出端（实车症状①兜底）。
            # 12°/帧 ≈ 每秒 190°，真实动态远够用，大台阶只是摊成两三帧的斜坡。
            angle = max(last_angle - ANGLE_SLEW, min(last_angle + ANGLE_SLEW, angle_raw))
            angle = max(-900, min(900, angle))

            valid = True
            state_v = 1 if e > CURVE_RMS_PX else 0
            # 有效帧顺带刷新两侧参考色（丢线分类/颜色身份门的"路面长什么样"都靠它）
            ref_bnd = ema_ref(ref_bnd, (a_up, b_up))
            ref_road = ema_ref(ref_road, (a_dn, b_dn))
            last_fit = (m, c, time.time())     # 给下一帧的追踪门当轨迹基准
            last_offset = offset
            last_angle = angle
        else:
            # 丢线：offset/angle 沿用旧值, 但方向要报准（Pi 定向找线的依据）
            state_v = classify_lost(pts, a_up, b_up, a_dn, b_dn)
        _t4 = time.ticks_us()

        # ---- 分阶段耗时累计（PROF_EVERY_S>0 时周期性打印一行均值）----
        if PROF_EVERY_S > 0:
            _prof[0] += time.ticks_diff(_t1, _t0) / 1000.0    # snapshot(含等帧)
            _prof[1] += time.ticks_diff(_t2, _t1) / 1000.0    # lens_corr
            _prof[2] += time.ticks_diff(_t2b, _t2) / 1000.0   # 上下带色心+选通道
            _prof[3] += time.ticks_diff(_t3, _t2b) / 1000.0   # 逐列剖面+选点
            _prof[4] += time.ticks_diff(_t4, _t3) / 1000.0    # 拟合+出量
            _prof[5] += time.ticks_diff(time.ticks_us(), _t4) / 1000.0  # 其余(绘制等)
            _prof_n += 1
            if time.ticks_diff(time.ticks_us(), _prof_t) >= PROF_EVERY_S * 1000:
                print("prof: snap=%.1f corr=%.1f 带=%.1f 列=%.1f 拟合=%.1f 其余=%.1f | 合=%.1fms (%.0ffps n=%d)"
                      % (_prof[0]/_prof_n, _prof[1]/_prof_n, _prof[2]/_prof_n,
                         _prof[3]/_prof_n, _prof[4]/_prof_n, _prof[5]/_prof_n,
                         sum(_prof)/_prof_n, clock.fps(), _prof_n))
                _prof = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                _prof_n = 0
                _prof_t = time.ticks_us()

        # ---- 5.5) 拍照（收到 'P' 时存这一帧）----
        #    ⚠️⚠️ **位置就是语义, 别挪**: 放在第 6 步（绘制）**之前** —— 存下来的是
        #    **干净原图**, 不带黄十字/绿拟合线那些调试标记（需求如此）。
        #    它在 lens_corr 之后, 所以**保留镜头畸变校正** —— 照片与算法看到的是同一幅几何。
        #    ⚠️ 第 1~5 步只"读"像素（get_statistics / 拟合）, 不改变图像,
        #       所以放这儿与"全部图像处理做完之后"是等价的。
        #    ⚠️ 想要带标记的版本（排障时看"每列读到什么"）: 把这段挪到第 6 步之后即可,
        #       位置就是开关, 不需要额外参数。
        #    ⚠️ photo.take 自己吞掉所有异常, 存图失败不影响本帧出量。
        if want_photo:
            want_photo = False
            if photo is not None:
                # 存盘 + **回执**给树莓派: 它再作为 COORD TAG=0x06 回传给单片机,
                # OLED 才会闪一句「拍照 成功!」。
                # ⚠️ 树莓派**只认这一行**, 所以成/败都要发（失败 = "P,0"）——
                #    不发的话单片机要一直等到 Pi 的超时兜底（PHOTO_ACK_TIMEOUT_S）才有回话。
                # ⚠️ 与 'G'/'S' 的发送开关**无关**: 那两个管巡线帧, 回执无条件发。
                _ok = photo.take(img)
                try:
                    uart.write("P,%d\r\n" % (1 if _ok else 0))
                except Exception as _e:
                    print("[拍照] 回执发送失败:", _e)
            else:
                print("[拍照] photo 模块不可用, 本次不拍")
                try:
                    uart.write("P,0\r\n")      # 也要回话, 别让单片机白等超时
                except Exception:
                    pass

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

        # ---- 7) 状态输出（限速；只在 SENDING 之外也打，方便台架上不上位机时看）----
        if time.time() - last_print > PRINT_EVERY_S:
            last_print = time.time()
            msg = "valid=%d off=%4d ang=%+6.1f st=%d 点%d剔%d 列%d/%d 闸%d 通道%c 距%.0f %.0ffps" % (
                1 if valid else 0, offset, angle / 10.0, state_v,
                fa[3] if fa else 0, fa[4] if fa else 0, len(pts), NCOL, len(pts_ok),
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
    #    格式: L,<offset>,<angle>,<valid>,<state>\r\n   —— 仅 SENDING 时发送
    if SENDING:
        uart.write("L,%d,%d,%d,%d\r\n" % (offset, angle, 1 if valid else 0, state_v))
