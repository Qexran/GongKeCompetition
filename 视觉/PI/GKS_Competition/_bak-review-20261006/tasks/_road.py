# tasks/_road.py — 「看路」：主摄看地面，量**黄色区左边缘的 x**（2026-09 加，第二版；
#                   2026-10 改版：判据 R−B → **Y−B**、阈值改**自适应**、投票**早停**提速）
#
# 用途：车过弯/到点靠开环里程会**走过头**。地面上的黄色区（比赛是黄胶带，练习地图上是
# 那块米黄篷布）**一般落在画面右上角**，车越往前开它越大、**左边缘越往左跑** ——
# 于是"黄色区左边缘的 x"本身就是一把**测距尺**：Δx 越小 = 车离得越远。
#
# 测量定义（用户口径，别自己改）：
#   Δx = **黄色区左边缘的 x**（全分辨率像素；"原点" = 画面左边 x=0）。
#   单片机按 (理想Δx − 实测Δx) × 比例系数 × 方向(±1) 修一次**车体纵向**。
#   Δy = **黄色区下边缘的 y**（同样全分辨率像素；2026-09-30 加）—— 同一块色块的**另一个轴**，
#        单片机按同一套算式修一次**车体横向**（本车相机相对车体转了 90° ⇒ 图像 y↔车体横向）。
#        ⚠️ 用**下**边缘不用上边缘: 黄区固定在画面右上角, 上边缘永远被画面顶边裁掉
#        （实测现场帧 bbox top 全是 0、flags 全是 0x05，那个 bit2 就是上边缘触发的）,
#        下边缘实测 278~576px 全在窗内且随机位变化。
#
# ==================== 2026-10 改版：为什么判据从 R−B 换成 Y−B ====================
# 换了更浅的新地图后 R−B 判据崩了。实测（现场帧 road_failed/1.jpg、2.jpg，固定曝光
# 166 + 固定 WB 6500K）：**黄区 R−B ≈ +11~+19、灰地 R−B ≈ +2~+8** —— 灰地自己变暖了，
# 余量从旧图的 18~20 级掉到 **5~8 级**；标定阈值 20（旧比赛图口径）直接把整块黄区关在
# 门外，掩膜只剩顶部噪声碎片（area 4092、fl=0x1D）→ 锁错/失败。
#   换 **Y−B = (R+G)/2 − B**（黄度，蓝黄对立色度）后实测同一批帧：
#     黄区 **+12~+17** / 灰地 **+2~+6** / 过曝热点 **−6~−10**（偏蓝，天然排除）
#     / 白斑马线 **+1~+2** / 车身黑件 **大负值** —— 余量 ≈ **10~13 级**，是 R−B 的两倍。
#   为什么 Y−B 对各种黄都稳: 中性黄(R≈G)时 Y−B = R−B（旧图口径不变）；**黄绿**(新地图
#   篷布 G>R)时它取两通道均值 ≈ G−B，信号翻倍；**橙黄**(胶带 R>G)时它 ≈ R−B 不吃亏。
#   纯红/纯绿的干扰物则会被另一半拉低 —— 比单通道差更能代表"黄"。
#
# ==================== 2026-10 改版：阈值为什么改自适应 ====================
# 固定阈值的死穴是"换图/换光就得重标，标在分布中段就碎成小岛"（config.py 里 2026-09-30
# 的头号坑）。现在阈值每帧现算（`_adaptive_threshold`，直方图 1/16 采样，全耗 <2ms）：
#   ① **谷底**：D 直方图主峰（灰地）与右边第一个过**显著性检验**的峰（黄区：峰高 ≥ 主峰
#      5% 且两峰间鞍点沉到较矮峰的 50% 以下）之间取谷底 —— 阈值自动落在两分布的空隙里，
#      地图换色/光照漂移都跟着走。显著性检验滤掉地面褶皱/光照梯度造出的"分布肩部假峰"
#      （2026-10-02 旧 log 回归实测：没这检验时阈值会被骗到地面分布内部 → 褶皱泄漏 → 假数）。
#   ② **黄区即主峰**（车很近，黄区占画面大头）：阈值 = 主峰 − 6。
#   ③ **黄区太小构不成峰**（车远）：阈值 = clamp(主峰+4, p99.5−3)，地板钳到
#      ROAD_WARM_FLOOR_YB —— p99.5 锚的是"最黄的那一小撮"。
#   三段都过不了（没有比主峰黄 ≥5 级的像样亮块）→ 直接判"画面里没有黄色区"，
#   **不出巨块**（旧版阈值失手时会整片灰地过阈 → Δx 贴死 ROI 左缘 → 单片机拿假数修车）。
#   ROAD_WARM_FLOOR_YB 是最后一道保险（"黄区再浅也必须比白的多几点黄"），正常别动。
#
# ==================== 2026-10 改版：速度 ====================
#   ① 投票**早停**：凑满 min_votes 且已见 Δx 极差 ≤ 离群下限(5px) → 提前收工。
#      静止机位下后面的帧只会重复同一答案 —— 实测 9 帧全采极差 0~2px，通常 **3 帧即回**
#      （单片机侧从 ~2~3s 降到 ~1~2s， chiefly 省在少读 6 帧上）。早停条件比最终判据
#      **严格**（极差 ≤ MAD 剔离群下限 ⇒ 早停结果与采满 9 帧逐比特同判），不会"早停的
#      成功换成采满就失败"。
#   ② V 通道用 max(B,G,R) 直接算（HSV 的 V 本来就是它），免掉整幅 cvtColor。
#   ③ 阈值直方图只吃 1/16 像素。降采样(ROAD_PROC_SCALE)照旧。
#
# ⚠️⚠️ 判据/阈值口径变了 ⇒ 掩膜边缘跟着挪 ⇒ **Δx 有 10~30px 级平移** —— 到场第一步重标
#    单片机的 ROAD_DX_IDEAL_PX / ROAD_DY_IDEAL_PX（把车摆到理想停车位，读蓝牙 Tab 的
#    road_dx/road_dy 填进去），旧标定作废！
# ⚠️ 已知局限：补光热点若正好压住黄区**顶部**的左边界，那一段掩膜会被挖穿、Δx 偏大
#    （实测机位热点在画面中部偏下、黄区左边界走画面右上，不挡；Web /road 预览一眼可查）。
#
# ⚠️ 只读一帧时车必须静止（多帧投票要求同一机位，与色球/矩形一致）。
# ⚠️ 坐标一律回**全分辨率 1920×1080**（与色球/人质/矩形帧同口径，单片机按它标定）。
# ⚠️ 任务层只返回结果 dict，串口回传/页面显示由调用方负责（同 _rect.py，保持纯净可测）。
#
# 历史（别把第一版捡回来）：第一版做的是"拟合弯道两条直边→求交→圆弧相切→取右切点"
#   （`看路.jpg` 上那两条红线就是那么画的）。它在**合成本例上完全正确**（Δ 几乎等于圆角
#   半径 R），但在真实机位上过不去：练习地图那条边界是篷布褶皱/软阴影边，Hough 会把圆弧的
#   弦当直线、折线化又因边界碎片化只给出短边 → 永远配不成"两条腿"。那一版留在
#   `tasks/_road.py.bak-curve`，完整复盘与证据见 `Desktop/road_offline/README.md`。
import struct
import time

import cv2
import numpy as np

from camera import CameraManager
from camera_lease import acquire_camera, release_camera

ROAD_TAG = 0x08   # 结果帧子命令标记（TYPE_COORD DATA 首字节，<0x20 与 ASCII 帧不冲突）

# 帧内 flags 位（与固件 Application/app_rpi.h 的 RPI_ROAD_FLAG_* 逐位对齐）
FLAG_EDGE_OK = 0x01   # bit0: 左边缘有效（**Δx 全靠它**）
FLAG_CLIP_X = 0x02    # bit1: 黄色区触到画面左/右边 → Δx 可能饱和，不可信
FLAG_CLIP_Y = 0x04    # bit2: 黄色区触到画面上/下边（只提示，不影响用）
FLAG_SMALL = 0x08     # bit3: 面积刚够门限（勉强），建议核对
FLAG_Y_EDGE_OK = 0x10  # bit4: **Δy 有效**（黄区**下边缘**没被裁掉）—— 单片机的横向修正靠它

# D 直方图（Y−B 的值域足够宽：车身黑件 −32、热点 −10、灰地 −11~+8、黄区 +7~+23）
_HIST_LO, _HIST_HI, _HIST_BINS = -60, 80, 141
_MIN_SEP_BINS = 4     # 主峰与黄区峰的最小间隔[bin=1级]
_PROMINENCE = 0.5     # 谷底显著性：鞍点高度须 ≤ 两边较矮峰高的 50%（滤地面分布肩部假峰）

_I16_MIN, _I16_MAX = -32768, 32767
_U16_MAX = 65535


def _clamp_i16(v):
    return max(_I16_MIN, min(_I16_MAX, int(round(v))))


def _clamp_u16(v):
    return max(0, min(_U16_MAX, int(round(v))))


def build_road_frame(ok, dx=0.0, ex=0.0, ey=0.0, flags=0, spread=0.0, dy=0.0):
    """打包回传帧 DATA（见 docs/串口协议.md §6i）：

    [TAG=0x08][status u8][dx i16 LE /10][ex i16 LE][ey i16 LE][flags u8][spread u8][dy u16 LE /10]
    = 12 B

      status : 1 = 成功（dx 有效）；0 = 失败（没找到黄区，全部填 0）
      dx     : **Δx = 黄色区左边缘的 x**，单位 **0.1px**（i16 → ±3276.7px，够用）→ 车体**纵向**
      ex/ey  : 左边缘那一点的全分辨率像素坐标（遥测/画图用，单片机不用）
      flags  : 见文件顶部 FLAG_*
      spread : Δx 跨帧散布[px]（封顶 255；只做遥测，单片机不据此判失败）
      dy     : **Δy = 黄色区下边缘的 y**，单位 **0.1px**（u16 → 0~6553.5px）→ 车体**横向**
               （本车相机相对车体转了 90° ⇒ 图像 y 那条轴就是车体横向）。**原先这里是
               "保留 u16" 恒 0** —— ⚠️ 填了它就必须同时置 FLAG_Y_EDGE_OK，否则单片机
               会把 0 读成"没有这个量"（那是它区分新旧 Pi 的唯一依据）。

    ⚠️ **失败帧也要发**（status=0）—— 单片机靠它立刻解锁，不然要空等满整个超时。
    ⚠️ 长度 12（**不是 9**）：固件里"9 字节 = last packet"那条通用分支会把这帧整个吞掉。
       **加了 dy 也刻意不改长度**（占原来那个保留字段）：改成 14 的话，按 `>= 14` 判的
       新固件会把旧 Pi 发来的 12B 帧当 last packet 吞掉（静默失效）。
    ⚠️ dx 用 **i16**：方向对调后误差可以为负，且黄区贴到画面左边时 x 可以到 0 附近 ——
       用 u16 会被钳成 0 而且**不报错**。dy 相反：下边缘的 y 不可能为负 → **u16**。
    """
    return struct.pack("<BBhhhBBH", ROAD_TAG, 1 if ok else 0,
                       _clamp_i16(dx * 10.0), _clamp_i16(ex), _clamp_i16(ey),
                       int(flags) & 0xFF,
                       max(0, min(255, int(round(spread)))), _clamp_u16(dy * 10.0))

# 标注用色（BGR）
_MASK_BGR = (0, 200, 0)
_EDGE_BGR = (0, 255, 255)
_DY_BGR = (255, 128, 255)      # 黄区下边缘那条横线（横向那把尺）


def _params():
    """参数每次调用现取（Web 参数页改完即时生效，同 _rect.py 的做法）。"""
    from config import (ROAD_PROC_SCALE, ROAD_ROI_TOP, ROAD_ROI_BOTTOM,
                        ROAD_ROI_LEFT, ROAD_ROI_RIGHT, ROAD_WARM_FLOOR_YB,
                        ROAD_PEAK_FRAC, ROAD_WARM_V_MIN, ROAD_WARM_OPEN_K,
                        ROAD_WARM_CLOSE_K, ROAD_WARM_MIN_AREA, ROAD_VOTE_FRAMES,
                        ROAD_MIN_VOTES, ROAD_MAX_SPREAD_PX, ROAD_OUTLIER_K,
                        ROAD_OUTLIER_MIN_PX, ROAD_DX_MIN_PX, ROAD_DX_MAX_PX)
    return dict(proc_scale=ROAD_PROC_SCALE,
                roi_top=ROAD_ROI_TOP, roi_bottom=ROAD_ROI_BOTTOM,
                roi_left=ROAD_ROI_LEFT, roi_right=ROAD_ROI_RIGHT,
                floor=ROAD_WARM_FLOOR_YB, peak_frac=ROAD_PEAK_FRAC,
                v_min=ROAD_WARM_V_MIN, open_k=ROAD_WARM_OPEN_K,
                close_k=ROAD_WARM_CLOSE_K, min_area=ROAD_WARM_MIN_AREA,
                vote_frames=ROAD_VOTE_FRAMES, min_votes=ROAD_MIN_VOTES,
                max_spread_px=ROAD_MAX_SPREAD_PX,
                outlier_k=ROAD_OUTLIER_K, outlier_min_px=ROAD_OUTLIER_MIN_PX,
                dx_min=ROAD_DX_MIN_PX, dx_max=ROAD_DX_MAX_PX)


# ==================== 单帧检测 ====================
def _roi_rect(shape, p):
    """分数式 ROI → 像素 (x0, y0, x1, y1)。**车身/机械臂全在画面下半**，靠它切掉。"""
    h, w = shape[:2]
    x0 = max(0, min(w - 1, int(round(p["roi_left"] * w))))
    y0 = max(0, min(h - 1, int(round(p["roi_top"] * h))))
    x1 = max(x0 + 1, min(w, int(round(p["roi_right"] * w))))
    y1 = max(y0 + 1, min(h, int(round(p["roi_bottom"] * h))))
    return x0, y0, x1, y1


def _adaptive_threshold(D, valid, p):
    """在 D（Y−B）直方图上**每帧现算**阈值。返回 (thr, note) 或 (None, 失败原因)。

    三段式（详见文件顶部"阈值为什么改自适应"）：
      ① 谷底  ② 黄区即主峰 → 主峰−6  ③ 极小黄区 → p99.5 锚定。都过不了 → None。
    ⚠️ 直方图只吃 1/16 像素（D[::4,::4]）：分布足够稳，耗时 <2ms。
    """
    d = D[::4, ::4][valid[::4, ::4]]
    if d.size < 2000:                     # ROI 里几乎没有有效像素（对错墙了？）
        return None, "有效像素太少"
    hist, _ = np.histogram(d, bins=_HIST_BINS, range=(_HIST_LO, _HIST_HI))
    k = np.array([1, 2, 3, 2, 1], np.float32)
    hs = np.convolve(hist.astype(np.float32), k / k.sum(), "same")
    hs[-1] = hs[-2]                       # 卷积右边缘塌掉，用前一 bin 补
    main = int(np.argmax(hs))
    # ① 向右找黄区峰：候选 = 主峰右侧第一个显著局部极大（峰高 ≥ peak_frac×主峰），
    #    但必须过**显著性检验**—— 主峰与它之间的鞍点要沉到两边较矮峰的 50% 以下。
    #    ⚠️ 没有这道检验会栽在"地面分布的肩部涟漪"上：地面褶皱/光照梯度让灰地直方图
    #    拖着带小鼓包的长尾，第一个鼓包就能把谷底骗进**地面分布内部**（阈值过低 → 褶皱
    #    泄漏 → 掩膜贴到 ROI 左缘 → Δx=192 假数；2026-10-02 旧 log 回归实测踩过，
    #    那批帧黄区 Y−B 其实 +28~+34、地面 +4~5，分离度极好，纯纯是扫描被肩部截停）。
    #    ⚠️ 也别直接用全局 argmax 找黄区：褶皱会在黄区内部造出子峰，argmax 可能跳过
    #    黄区抓到更远的噪声峰，同样把谷底卡进黄区分布中段（碎岛坑，2026-09-30 实测）。
    pk2 = None
    for b in range(main + _MIN_SEP_BINS, _HIST_BINS - 1):
        if hs[b] >= p["peak_frac"] * hs[main] and hs[b] >= hs[b - 1] and hs[b] >= hs[b + 1]:
            saddle = float(hs[main:b + 1].min())
            if saddle <= _PROMINENCE * min(hs[main], hs[b]):
                pk2 = b
                break
    if pk2 is not None:
        valley = main + int(np.argmin(hs[main:pk2 + 1]))
        thr = float(max(valley, main + 2) + _HIST_LO)   # 谷底贴着主峰时至少抬 2 级
        return max(thr, float(p["floor"])), f"谷底 主峰{main + _HIST_LO:+d}/黄区{pk2 + _HIST_LO:+d}"
    main_d = main + _HIST_LO
    if main_d >= p["floor"] + 4:          # ② 主峰本身就是黄区（车很近，黄区占大头）
        return float(max(main_d - 6, p["floor"])), f"黄区即主峰({main_d:+d})"
    p995 = float(np.percentile(np.clip(d, _HIST_LO, _HIST_HI), 99.5))
    # ⚠️ 分位数必须钳到直方图值域：图上若有饱和黄污染（烘焙标注/车上黄线标过曝,
    #    Y−B 可到 200+），不钳的话 p99.5 会飞到分布外、锚定完全失效（实测 p99.5=208）。
    if p995 - main_d >= 5:                # ③ 黄区太小构不成峰 → 高分位锚定
        thr = max(p["floor"], min(main_d + 4, p995 - 3))
        return float(thr), f"小黄区 主峰{main_d:+d}/p99.5={p995:.0f}"
    return None, "画面里没有比地面黄的像样亮块（没有黄色区）"


def _warm_mask(sub, p):
    """黄色区掩膜：**Y−B = (R+G)/2 − B > 自适应阈值**（黄度色差 + 直方图谷底）。

    ⚠️ 别改回单帧固定阈值、也别改回 R−B：新浅色地图上灰地自己 R−B≈+6，R−B 余量只剩
       5 级（实测 road_failed/1.jpg），固定阈值标在哪儿都两头漏。Y−B + 谷底自适应
       对"地图换色/光照漂移"免疫，阈值永远落在灰地与黄区分布的空隙里。
    ⚠️ V 用 max(B,G,R) 直接算（HSV 的 V 就是它），省一整幅 cvtColor。
    返回 (mask, thr, note)；算不出阈值时 mask 为 None。
    """
    b = sub[:, :, 0].astype(np.int16)
    g = sub[:, :, 1].astype(np.int16)
    r = sub[:, :, 2].astype(np.int16)
    D = (r + g) // 2 - b
    valid = sub.max(axis=2) >= int(p["v_min"])     # 挡全黑噪声（车身已被 Y−B 排除，双保险）
    thr, note = _adaptive_threshold(D, valid, p)
    if thr is None:
        return None, thr, note
    m = ((D > thr) & valid).astype(np.uint8) * 255
    ko = int(p["open_k"]) | 1
    kc = int(p["close_k"]) | 1
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ko, ko)))
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (kc, kc)))
    return m, thr, note


def _detect_one(frame, p):
    """单帧：Y−B 自适应抠黄色区 → 取**最大连通块** → 最左那一列 → Δx; bbox 下边 → Δy。坐标回全分辨率。"""
    fail = dict(success=False, reason="", dx=0.0, dy=0.0, edge=None, flags=0, mask=None,
                roi=None, scale=1.0, area=0.0, thr=None, note="")
    x0, y0, x1, y1 = _roi_rect(frame.shape, p)
    sub = frame[y0:y1, x0:x1]
    sc = float(p["proc_scale"])
    if abs(sc - 1.0) > 1e-6:
        sub = cv2.resize(sub, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
    hh, ww = sub.shape[:2]

    mask, thr, note = _warm_mask(sub, p)
    if mask is None:
        return dict(fail, reason=note, roi=(x0, y0, x1, y1), scale=sc, thr=None, note=note)
    n, lab, st, _ce = cv2.connectedComponentsWithStats(mask, 8)
    if n < 2:
        return dict(fail, reason="画面里没有黄色区（自适应阈值下无亮块）", mask=mask,
                    roi=(x0, y0, x1, y1), scale=sc, thr=thr, note=note)
    # ⚠️ 取**最大连通块**：车上的黄色线标/另一条胶带是小的，取最大就天然躲开它们
    #    （多帧投票里还有双峰判据兜底：真锁错时会分两簇、直接不给数）
    # ⚠️ 2026-09-30 试过"按帧间共识重挑块"来治"掩膜碎成小岛 → 最大块逐帧换人"，**撤了**:
    #    掩膜碎了就没有正确答案 —— 多数小岛占多数时共识会稳定地锁在**错块**上（实测真报过
    #    1515 而真相是 870），而势均力敌时它会把"分成两簇 → 明确不给数"悄悄变成"多数决"。
    #    2026-10 起谷底自适应阈值把"阈值卡在分布中段 → 碎岛"这个根源掐掉了，这条纪律继续。
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    a_full = float(st[i, cv2.CC_STAT_AREA]) / (sc * sc)
    if a_full < p["min_area"]:
        return dict(fail, reason="黄色区太小（%.0f < %.0f px²）"
                                 % (a_full, p["min_area"]),
                    mask=mask, roi=(x0, y0, x1, y1), scale=sc, area=a_full,
                    thr=thr, note=note)

    ys, xs = np.nonzero(lab == i)
    xmin = int(xs.min())
    ymed = int(np.median(ys[xs == xmin]))           # 最左那一列的代表 y
    bx, by = st[i, cv2.CC_STAT_LEFT], st[i, cv2.CC_STAT_TOP]
    bw, bh = st[i, cv2.CC_STAT_WIDTH], st[i, cv2.CC_STAT_HEIGHT]

    flags = FLAG_EDGE_OK
    # ⚠️ CLIP_X 只看**左边**：Δx 是"左边缘的 x"，只有它贴到画面左边才会饱和。
    #    黄区**右侧**跑出画面是常态（它本来就铺到画面右边），不该报"不可信"。
    if bx <= 2:
        flags |= FLAG_CLIP_X
    if by <= 2 or by + bh >= hh - 2:
        flags |= FLAG_CLIP_Y
    if a_full < 2.0 * p["min_area"]:
        flags |= FLAG_SMALL

    # ---- Δy = 黄区**下边缘**的 y（喂单片机的横向修正）----
    # ⚠️ 为什么是**下**边缘而不是上边缘: 黄区固定在画面**右上角** ⇒ 上边缘永远被画面顶边
    #    裁掉（实测四个现场帧的 bbox top 全是 0、flags 全是 0x05 —— 那个 bit2 就是**上**
    #    边缘触发的），拿它当尺子等于量一个恒为 0 的常数。下边缘实测 278~576px 全在窗内。
    # ⚠️ 有效性只看**下边**有没有贴到 ROI/画面下沿（贴了就饱和）; 上边被裁无所谓。
    dy = y0 + (by + bh - 1) / sc                    # bbox 是 [top, top+height-1], 回全分辨率
    if by + bh < hh - 2:
        flags |= FLAG_Y_EDGE_OK

    return dict(success=True, reason="", flags=flags, mask=mask, scale=sc,
                roi=(x0, y0, x1, y1), area=a_full, thr=thr, note=note,
                dx=x0 + xmin / sc,                       # 回全分辨率
                dy=dy,
                edge=(x0 + xmin / sc, y0 + ymed / sc))


# ==================== 多帧投票 ====================
def _spread(vals):
    return (max(vals) - min(vals)) if vals else 0.0


def _median(vals):
    xs = sorted(vals)
    n = len(xs)
    return xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0


def _ambiguous_gap(vals, p):
    """帧值是否分成**两簇**（两簇都够票数、间距超散布闸）→ 返回间距，否则 0。

    ⚠️ 必须在剔离群**之前**判（`_rect.py` 踩过同一个坑）：偶尔会有几帧锁到**另一块**
       黄区（车上的黄线标、另一条胶带），于是 5 帧读 A、4 帧读 B。MAD 会把少数那一簇
       当离群剔掉，剩下的看着又紧又稳 → 报出一个错值而 spread 还是 0。先说是哪一簇。
    """
    n = len(vals)
    if n < 2 * p["min_votes"]:
        return 0.0
    order = sorted(vals)
    for k in range(p["min_votes"], n - p["min_votes"] + 1):
        gap = order[k] - order[k - 1]
        if gap > p["max_spread_px"]:
            return gap
    return 0.0


def _inlier_mask(vals, p):
    """MAD 离群判据：|v − 中位| ≤ max(下限, k×1.4826×MAD)（抖得小就一个不剔）。"""
    med = _median(vals)
    dev = [abs(v - med) for v in vals]
    thr = max(p["outlier_min_px"], p["outlier_k"] * 1.4826 * _median(dev))
    return [d <= thr for d in dev]


def _annotate(frame, res):
    """画 ROI、黄色区轮廓、左边缘竖线（全部 ASCII 文本，cv2 画不了中文）。"""
    if frame is None:
        return None
    out = frame.copy()
    res = res or {}
    roi = res.get("roi")
    if roi is not None:
        cv2.rectangle(out, (int(roi[0]), int(roi[1])), (int(roi[2]), int(roi[3])),
                      (128, 128, 128), 2)
    mask = res.get("mask")
    if mask is not None:
        sc = res.get("scale", 1.0) or 1.0
        ox, oy = (roi[0], roi[1]) if roi is not None else (0, 0)
        off = np.array([ox, oy], float)
        for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            q = (c.reshape(-1, 2).astype(float) / sc + off).astype(np.int32)
            cv2.polylines(out, [q], True, _MASK_BGR, 3)
    e = res.get("edge")
    if e is not None:
        x, y = int(e[0]), int(e[1])
        cv2.line(out, (x, 0), (x, out.shape[0]), _EDGE_BGR, 2)
        cv2.circle(out, (x, y), 10, _EDGE_BGR, -1)
        # 黄区**下边缘**（横向那把尺）也画出来 + 标出它有没有被裁
        dy = res.get("dy")
        if dy:
            cv2.line(out, (0, int(dy)), (out.shape[1], int(dy)), _DY_BGR, 2)
        cv2.putText(out, "dx=%.1fpx edge(%d,%d) area=%.0f thr=%.0f fl=0x%02X"
                    % (res.get("dx", 0.0), x, y, res.get("area", 0.0),
                       res.get("thr") or 0.0, res.get("flags", 0)),
                    (20, 40), 0, 1.1, _EDGE_BGR, 3)
        if dy:
            cv2.putText(out, "dy=%.1fpx%s"
                        % (dy, "" if res.get("flags", 0) & FLAG_Y_EDGE_OK else " (clipped)"),
                        (20, 80), 0, 1.1, _DY_BGR, 3)
    return out


def run_road_detect(timeout=None, abort_event=None):
    """主摄「看路」：量黄色区左边缘的 x（纵向）与**下边缘的 y**（横向）。
    限时 timeout 秒，默认 config.ROAD_TIMEOUT。

    逐帧 `_detect_one` → 多帧去离群（顺序不能换：**先判双峰，再 MAD 剔跳档**）→ 内点取均值。
    ⚠️ 2026-10 早停：凑满 min_votes 且已见 Δx 极差 ≤ MAD 剔离群**下限**就提前收工 ——
       条件比最终判据严格（此时双峰判不着、MAD 一个不剔、内点=全部），所以早停出的数
       与采满 vote_frames 逐比特同判；抖动稍大就自动退回采满的纪律。

    Returns dict: {"success", "reason", "frame"(标注帧), "dx", "dy", "edge"(ex,ey),
                   "area", "flags", "votes", "inliers", "dropped", "spread",
                   "dy_spread", "frames", "dy_frames", "thr"}
        `dx` = **Δx（黄色区左边缘的 x）**，全分辨率像素 → 单片机修车体纵向。
        `dy` = **Δy（黄色区下边缘的 y）**，全分辨率像素 → 单片机修车体横向
              （有效性看 flags 的 bit4 `FLAG_Y_EDGE_OK`）。
        aborted=True 只在被打断时出现。
    """
    from config import ROAD_TIMEOUT

    timeout = ROAD_TIMEOUT if timeout is None else timeout
    fail = {"success": False, "reason": "", "frame": None, "dx": 0.0, "dy": 0.0,
            "edge": None, "area": 0.0, "flags": 0, "votes": 0, "inliers": 0,
            "dropped": 0, "spread": 0.0, "dy_spread": 0.0, "frames": [], "thr": None}

    cap = acquire_camera(is_qr=False, profile="road")
    if cap is None:
        if abort_event is not None and abort_event.is_set():
            print("[INFO] 看路任务被打断")
            return dict(fail, aborted=True)
        return dict(fail, reason="主摄像头打开失败")

    try:
        p = _params()
        print(f"[INFO] 开始看路（黄色区左边缘，最多 {p['vote_frames']} 帧，"
              f"限时 {timeout:.0f}s）...")
        deadline = time.time() + timeout
        hits = []
        frames_seen = 0
        while time.time() < deadline and frames_seen < p["vote_frames"]:
            if abort_event is not None and abort_event.is_set():
                print("[INFO] 看路任务被打断")
                return dict(fail, aborted=True)
            frame = CameraManager.read(cap, "Main Camera", max_retries=3)
            if frame is None:
                time.sleep(0.05)
                continue
            frames_seen += 1
            r = _detect_one(frame, p)
            if r["success"]:
                hits.append((r, frame))
                # ---- 早停（见 docstring）：静止机位下后面的帧只会重复同一答案 ----
                if len(hits) >= p["min_votes"] and \
                        _spread([h[0]["dx"] for h in hits]) <= p["outlier_min_px"]:
                    print(f"[INFO] 看路早停: {len(hits)} 帧极差 "
                          f"{_spread([h[0]['dx'] for h in hits]):.1f}px 已够稳")
                    break
            else:
                print(f"[WARN] 看路第 {frames_seen} 帧失败: {r['reason']}")
            time.sleep(0.01)

        if len(hits) < p["min_votes"]:
            reason = (f"看路失败：{frames_seen} 帧里只有 {len(hits)} 帧检到黄区 "
                      f"(需≥{p['min_votes']})")
            print(f"[WARN] {reason}")
            best = hits[0] if hits else None
            return dict(fail, reason=reason, votes=len(hits),
                        frame=_annotate(best[1], best[0]) if best else None)

        dxs = [h[0]["dx"] for h in hits]
        # ① **先判双峰**（顺序反了会把少数那一簇当离群剔掉、静默报错值）
        gap = _ambiguous_gap(dxs, p)
        if gap > 0.0:
            reason = (f"Δx 分成两簇（差 {gap:.0f}px 且两边都够票数）—— "
                      f"可能锁到了另一块黄区，不给数")
            print(f"[WARN] {reason}  各帧 Δx={[round(v) for v in dxs]}")
            return dict(fail, reason=reason, votes=len(hits),
                        frame=_annotate(hits[0][1], hits[0][0]),
                        frames=[round(v, 1) for v in dxs])

        # ② MAD 剔跳档
        keep = _inlier_mask(dxs, p)
        idx = [i for i in range(len(hits)) if keep[i]]
        dropped = len(hits) - len(idx)
        if len(idx) < p["min_votes"]:
            reason = (f"看路抖动过大：{len(hits)} 帧里剔除 {dropped} 个离群帧后只剩 "
                      f"{len(idx)} 帧")
            print(f"[WARN] {reason}  各帧 Δx={[round(v) for v in dxs]}")
            return dict(fail, reason=reason, votes=len(hits), inliers=len(idx),
                        dropped=dropped, frame=_annotate(hits[0][1], hits[0][0]))

        # ③ 内点取均值
        dx = sum(dxs[i] for i in idx) / len(idx)
        spread = _spread([dxs[i] for i in idx])
        if spread > p["max_spread_px"]:
            reason = f"Δx 跨帧散布 {spread:.0f}px 超限（车在动？）"
            print(f"[WARN] 看路{reason}")
            return dict(fail, reason=reason, votes=len(hits), inliers=len(idx),
                        dropped=dropped, spread=spread)
        if not (p["dx_min"] <= dx <= p["dx_max"]):
            reason = (f"Δx={dx:.0f}px 不在 [{p['dx_min']:.0f},{p['dx_max']:.0f}] 内"
                      f"（黄区几乎看不到 / 认错了东西？）")
            print(f"[WARN] 看路{reason}")
            return dict(fail, reason=reason, votes=len(hits), inliers=len(idx),
                        dropped=dropped, spread=spread)

        edge = (sum(hits[i][0]["edge"][0] for i in idx) / len(idx),
                sum(hits[i][0]["edge"][1] for i in idx) / len(idx))
        rep = min(idx, key=lambda j: abs(hits[j][0]["dx"] - dx))
        flags = 0
        for i in idx:                       # 任何一帧贴边都算（保守）
            flags |= hits[i][0]["flags"]
        # Δy 跟着**同一批内点帧**取均值（双峰/MAD 仍只判 Δx）—— 同一块黄区、同一机位,
        # 拆成两套内点只会让两边都不一致。散布只进日志（**不对 dy 判失败**:
        # 横向那一轴单片机会自己按死区/钳位兜底, 这里再拦一次只会让整步白跑）。
        dys = [h[0]["dy"] for h in hits]
        dy = sum(dys[i] for i in idx) / len(idx)
        dy_spread = _spread([dys[i] for i in idx])
        res = dict(success=True, reason="", dx=dx, dy=dy, edge=edge,
                   area=hits[rep][0]["area"], flags=flags, votes=len(hits),
                   inliers=len(idx), dropped=dropped, spread=spread,
                   dy_spread=dy_spread, thr=hits[rep][0]["thr"],
                   frames=[round(v, 1) for v in dxs],
                   dy_frames=[round(v, 1) for v in dys])
        print("[INFO] 看路成功: Δx=%.1fpx 左边缘=(%.0f,%.0f) 面积=%.0f "
              "帧%d(剔%d) 散布%.1fpx fl=0x%02X thr=%.0f(%s)"
              % (dx, edge[0], edge[1], res["area"], len(hits), dropped, spread,
                 flags, res["thr"] or 0.0, hits[rep][0]["note"]))
        print("[INFO]   Δy=%.1fpx 散布%.1fpx %s"
              % (dy, dy_spread, "(有)" if flags & FLAG_Y_EDGE_OK else "(无效, 下边缘被裁)"))
        print("[INFO]   各帧 Δx=%s" % [round(v) for v in dxs])
        print("[INFO]   各帧 Δy=%s" % [round(v) for v in dys])
        # 画在**最接近最终值的那一帧**上（预览里看到的数就是返回的数）
        res["frame"] = _annotate(hits[rep][1],
                                 dict(hits[rep][0], dx=dx, dy=dy, edge=edge, flags=flags))
        return res
    finally:
        release_camera(cap)     # 租约句柄 → 归还（保持打开）；普通 cap → 真释放
