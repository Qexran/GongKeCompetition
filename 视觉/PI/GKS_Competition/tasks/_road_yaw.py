# tasks/_road_yaw.py — 「看路校准」：主摄拟合**黄色区直边的方向角**（2026-10 加）
#
# 用途：陀螺仪 yaw 会漂（零偏随温度/时间慢变），过弯全靠它咬航向，漂了整条路线斜着走。
# 本任务给一个**不依赖陀螺仪的航向读数**：黄区（比赛黄胶带 / 练习地图米黄篷布）**固定在
# 场上**，而相机固定在车上 ⇒ 黄区直边在**图像里的方向**只随**车航向**变（平移几乎不动它
# —— 直线方向对平移不敏感，比「看路」那两把尺 Δx/Δy 干净：那两把尺把平移和航向混在一起）。
# 于是：车停在**基准停车位**标一次 ideal，之后每次到点测 ang，err = (ang − ideal)×方向
# 就是「陀螺漂了多少」的现场读数。
#
# ⚠️⚠️ **本任务只测不改**：结果只进遥测（roadyaw_*）与调试串，**不动车、不写航向** ——
#      先在现场把 roadyaw_err 与陀螺 yaw 并排记录、确认相关性，之后再接慢速互补修正
#      （yaw_bias += k×err 那一类）。**别硬置 yaw**（会踢到正在咬航向的巡线闭环）。
#
# 测量定义（与固件 app_rpi.h 的 RPI_ROADYAW_TAG 注释逐字对齐）：
#   ang = 黄区直边的**图像方向角**：拟合直线与**图像竖直方向（y 轴）**的夹角，
#         竖直 = 0°，向图像 x 正方向（画面右）倒为正，归一化到 **(−90, +90]**。
#         （约定本身没有物理含义 —— 转向对不对得上由蓝牙 roadyaw_dir 现场定，见
#         app_config.h 的 ROADYAW_* 段；这里只要**稳定、单调、可复现**。）
#   ⚠️ **未标定哨兵是 999 不是 0**（单片机侧）：竖直的直边 ang 本来就 = 0°，0 是合法值。
#   ⚠️ **方向角是 mod 180° 的量**：+89.9° 与 −89.9° 是**同一个朝向**（差 0.2° 不是 180°）。
#      多帧投票按**圆均值**（0.5·atan2(⟨sin2θ⟩,⟨cos2θ⟩)）折参考系后再跑线性判据 ——
#      否则贴着 ±90° 边界的近水平直边（本机位篷布下边缘正是）会被误判成"两簇/散布 180°"
#      而识别失败（2026-10-08 实测）。单片机侧 err 同样按 mod 180 折回（fmodf 那三行）。
#
# ==================== 算法：为什么是「窗 + Huber 直线拟合」====================
#   ① ROI 选段（ROADYAW_ROI_*，分数式子窗）：**人工把窗罩在一段直边上**，别把拐角/
#      圆弧罩进来。自动识别"哪段是直的"（曲率断点检测之类）在这个场景是过度设计 ——
#      现场机位固定，直边在哪一次标定就定了。
#   ② 掩膜**整体复用看路那套**（tasks/_road.py 的 _warm_mask：Y−B 色度差 + 每帧自适应
#      阈值）—— 「什么算黄区」只有一个来源，两边阈值永远不会各漂各的。掩膜参数
#      （ROAD_WARM_* / ROAD_PEAK_FRAC）也直接吃 ROAD_* 那份，本任务**没有**自己的阈值。
#   ③ 取最大连通块的边界点（CHAIN_APPROX_NONE，处理尺度下逐像素），**先折线化分段、
#      再按段拟合**（2026-10-08 改）：窗罩着整块黄区时（ROI 全宽的常态），最大块的边界
#      是个**闭环** —— 真直边只是其中一段，撕边/褶皱/弧段混在同一份点集里。旧版对全部
#      边界点做一次全局 Huber，撕边也有几百点时就在「直边」「撕边」「折中线」三种形态间
#      **逐帧翻转**（实测同一机位 40 帧方向角扫了 7.8°，多帧投票直接两簇/散布超限）。
#      现在把每段有序边界弧 approxPolyDP 折线化（弦偏差 ≤ _SEG_EPS_PX）：撕边碎成短弦
#      过不了 MIN_SPAN 门，真直边保持一整条；候选段各自跑两遍 Huber（压野点 → 按
#      |残差| ≤ ROADYAW_INLIER_PX 剔点 → 内点重拟合）+ 质量门（RMS / 直段长 / 内点数），
#      **拟合后跨度最长者胜** —— 角度只由那条真直边决定。两条真直边都够长时仍由多帧
#      投票的双簇判据兜底（那时才真是"该收窄窗了"）。宁可不给数，不报假角 —— 不变。
#   ④ 多帧投票与看路同一套（先双峰后 MAD、早停），只是量从 px 换成了 °。
#
# ⚠️ 只读一帧时车必须静止（多帧投票要求同一机位，与看路/色球一致）。
# ⚠️ 坐标/角度口径：ang 是**本任务自己的量**（0.01°，i16），与巡线帧的 angle（交界线倾角）
#    **不是同一个东西**，别互相抄标定值。
# ⚠️ 任务层只返回结果 dict，串口回传/页面显示由调用方负责（同 _road.py，保持纯净可测）。
import math
import struct
import time

import cv2
import numpy as np

from camera import CameraManager
from camera_lease import acquire_camera, release_camera
# 掩膜/ROI/投票全套复用看路的 —— 这两个 import 是本任务的根基, 别为它复制一份
from tasks._road import (_adaptive_threshold, _roi_rect, _warm_mask, _warm_contrast,
                         _ambiguous_gap, _inlier_mask, _spread)

ROADYAW_TAG = 0x09   # 结果帧子命令标记（TYPE_COORD DATA 首字节，<0x20 与 ASCII 帧不冲突）

# 帧内 flags 位（与固件 Application/app_rpi.h 的 RPI_ROADYAW_FLAG_* 逐位对齐）
FLAG_FIT_OK = 0x01    # bit0: 方向角有效（ang 全靠它）
FLAG_FEW_PTS = 0x02   # bit1: 直段偏短/内点少（勉强够门限，只提示）

_I16_MIN, _I16_MAX = -32768, 32767


def _clamp_i16(v):
    return max(_I16_MIN, min(_I16_MAX, int(round(v))))


def _clamp_u8(v):
    return max(0, min(255, int(round(v))))


def build_road_yaw_frame(ok, angle=0.0, cols=0, rms=0.0, spread=0.0, flags=0):
    """打包回传帧 DATA（见 docs/串口协议.md §6j）：

    [TAG=0x09][status u8][ang i16 LE /100][cols u8][rms u8 /10][spread u8 /100][flags u8]
    = 8 B

      status : 1 = 成功（ang 有效）；0 = 失败（没找到黄区/拟合不过关，全部填 0）
      ang    : **黄区直边的图像方向角**，单位 **0.01°**（i16 → ±327.67°，够 ±90°）
               竖直=0°、向画面右倒为正、归一化 (−90,+90] → 车航向的图像观测量
      cols   : 拟合内点数（封顶 255；只做遥测）
      rms    : 内点拟合残差 RMS，单位 **0.1px**（封顶 255 = 25.5px；质量门，越小越直）
      spread : ang 跨帧散布，单位 **0.01°**（封顶 255 = 2.55°；只做遥测）
      flags  : 见文件顶部 FLAG_*

    ⚠️ **失败帧也要发**（status=0）—— 单片机靠它立刻解锁，不然要空等满整个超时。
    ⚠️ 长度 **8（不是 9）**：固件通用分支按 `LEN >= 9` 把 9B 帧当 last packet；
       8B 天然落不进去。旧固件没有 0x09 分支 → 8B 帧被**静默丢弃**（不炸链路），
       所以部署顺序**先 Pi 后固件**（同预热指令 "B"/"C" 那条纪律）。
    """
    return struct.pack("<BBhBBBB", ROADYAW_TAG, 1 if ok else 0,
                       _clamp_i16(angle * 100.0), _clamp_u8(cols),
                       _clamp_u8(rms * 10.0), _clamp_u8(spread * 100.0),
                       int(flags) & 0xFF)


# 标注用色（BGR）
_MASK_BGR = (0, 200, 0)
_LINE_BGR = (0, 128, 255)      # 拟合出的直边方向线
_ROI_BGR = (128, 128, 128)


def _params():
    """参数每次调用现取（Web 参数页改完即时生效，同 _road.py 的做法）。

    ⚠️ 掩膜那几个键（floor/peak_frac/v_min/open_k/close_k/min_area/proc_scale）
    **故意吃 ROAD_* 那份** —— 「什么算黄区」只有一个来源。本任务自己的旋钮只有
    ROI（选直段）、质量门（span/rms/inlier/cols）与投票三件套（° 口径）。
    """
    from config import (ROAD_PROC_SCALE, ROAD_WARM_FLOOR_YB, ROAD_PEAK_FRAC,
                        ROAD_WARM_V_MIN, ROAD_WARM_OPEN_K, ROAD_WARM_CLOSE_K,
                        ROAD_WARM_MIN_AREA, ROAD_WARM_CONTRAST_MIN,
                        ROAD_WARM_MAX_AREA_FRAC,
                        ROADYAW_ROI_TOP, ROADYAW_ROI_BOTTOM,
                        ROADYAW_ROI_LEFT, ROADYAW_ROI_RIGHT,
                        ROADYAW_VOTE_FRAMES, ROADYAW_MIN_VOTES,
                        ROADYAW_MAX_SPREAD_DEG, ROADYAW_OUTLIER_K,
                        ROADYAW_OUTLIER_MIN_DEG, ROADYAW_MIN_SPAN_PX,
                        ROADYAW_MIN_COLS, ROADYAW_MAX_RMS_PX,
                        ROADYAW_INLIER_PX, ROADYAW_EDGE_MARGIN_PX)
    return dict(# ---- 复用看路的掩膜键（_warm_mask/_adaptive_threshold 认这几个名字）----
                proc_scale=ROAD_PROC_SCALE,
                floor=ROAD_WARM_FLOOR_YB, peak_frac=ROAD_PEAK_FRAC,
                v_min=ROAD_WARM_V_MIN, open_k=ROAD_WARM_OPEN_K,
                close_k=ROAD_WARM_CLOSE_K, min_area=ROAD_WARM_MIN_AREA,
                contrast_min=ROAD_WARM_CONTRAST_MIN,
                max_area_frac=ROAD_WARM_MAX_AREA_FRAC,
                # ---- 本任务自己的键 ----
                roi_top=ROADYAW_ROI_TOP, roi_bottom=ROADYAW_ROI_BOTTOM,
                roi_left=ROADYAW_ROI_LEFT, roi_right=ROADYAW_ROI_RIGHT,
                vote_frames=ROADYAW_VOTE_FRAMES, min_votes=ROADYAW_MIN_VOTES,
                max_spread_deg=ROADYAW_MAX_SPREAD_DEG,
                outlier_k=ROADYAW_OUTLIER_K, outlier_min_deg=ROADYAW_OUTLIER_MIN_DEG,
                min_span_px=ROADYAW_MIN_SPAN_PX, min_cols=ROADYAW_MIN_COLS,
                max_rms_px=ROADYAW_MAX_RMS_PX, inlier_px=ROADYAW_INLIER_PX,
                edge_margin=ROADYAW_EDGE_MARGIN_PX)


def _angle_of(vx, vy):
    """拟合直线方向 → 图像方向角 [°]：竖直=0、向画面右倒为正、归一化 (−90,+90]。

    ⚠️ 直线方向是 mod 180° 的 —— atan2 出来落在 (−180,180]，>90 减 180、≤−90 加 180
    折回 (−90,+90]，否则"接近竖直的线"会在 ±90 边界上跳号。
    """
    ang = math.degrees(math.atan2(vx, vy))
    if ang > 90.0:
        ang -= 180.0
    elif ang <= -90.0:
        ang += 180.0
    return ang


def _wrap180(a):
    """把角度折到 [−90, +90)。方向角是 **mod 180°** 的量 —— ±180° 是同一个朝向。"""
    return (a + 90.0) % 180.0 - 90.0


def _ang_diff(a, b):
    """有符号最小差 a−b ∈ [−90, +90)（mod 180 意义下）。"""
    return _wrap180(a - b)


def _circular_mean_deg(angles):
    """mod 180° 角度的圆均值: 0.5·atan2(⟨sin2θ⟩,⟨cos2θ⟩) ∈ [−90, +90)。

    ⚠️ 不能直接算术平均: 贴着 ±90° 边界的直线（比如本机位上篷布的近水平下边缘）
    会在 +89.9 与 −89.9 之间跳, 算术平均会得出 0° 这种荒谬答案 —— 圆均值永远落在
    真实朝向上（+89.9/−89.9 是同一近水平朝向隔着边界线的两种读法, 圆均值 = ±90°,
    不是 0）。
    """
    s = c = 0.0
    for a in angles:
        r = math.radians(2.0 * a)
        s += math.sin(r)
        c += math.cos(r)
    if s == 0.0 and c == 0.0:            # 均匀分布（有票数门挡着, 正常到不了）
        return 0.0
    return _wrap180(0.5 * math.degrees(math.atan2(s, c)))


# ==================== 直段分段（2026-10-08 加） ====================
# 折线化容差[px, 处理尺度]：点离开所在段的弦超过它就该断开。处理尺度(0.5)下掩膜
# 边界本身有 ±1px 级的锯齿, 容差太小会把直边切碎、太大又贴着撕边走 —— 取锯齿的
# 2 倍左右, 让"真直边"整段通过、"撕边/褶皱"按曲率断开。
_SEG_EPS_PX = 2.5
# 相邻两折线段并回一段的方向差上限[°]（mod 180 口径）：微弯的长边（篷布自重下垂）
# 会被折线化切成几段近共线的短段, 并回去才能跟"一条真直边"比跨度。
_SEG_MERGE_DEG = 3.0
# 有序边界点里序号断开超过它算另一条弧（贴窗边剔除带挖出的洞不另起弧, 掩膜碎洞才另起）
_ARC_GAP = 3
# 双簇判据的**间隔下限**[°]：小于它的"两簇"比本方法在篷布机位自身的帧间噪声
# （std ~0.15°, 偶发尾巴凑够 min_votes 票 → 实测 0.3~0.45° 的假两簇, 概率 ~10%）
# 还小 —— 簇选哪边对 err 的影响都在噪声里, 不值得报废一次测量, 交给 MAD+散布闸处置。
# 真正要拦的「窗里两条真直边」至少差几度, 1° 起步才叫"两条边"。（只抬本任务的调用
# 参数, _ambiguous_gap 本体是看路/矩形共用的, 别动。）
_TWO_CLUSTER_MIN_GAP_DEG = 1.0


def _straight_segments(arc):
    """把一条**有序**边界弧折线化成直线段候选，返回闭点区间 [(i0, i1), ...]（含端点）。

    approxPolyDP（弦偏差 ≤ _SEG_EPS_PX, 非闭合）输出保持输入顺序的顶点子集, 把顶点
    映射回弧内下标（单调扫描）→ 相邻顶点即一段; 方向差 ≤ _SEG_MERGE_DEG 的相邻段
    并回一段。⚠️ 返回的是**下标区间**不是点集 —— 调用方还要按段跑拟合与质量门。
    """
    q = cv2.approxPolyDP(arc.reshape(-1, 1, 2), _SEG_EPS_PX, False).reshape(-1, 2)
    if len(q) < 2:
        return [(0, len(arc) - 1)] if len(arc) else []
    idxs, j = [], 0
    for v in q:                       # 顶点 → 弧内下标（顺序保持, 单调前进）
        k = j + int(np.argmin(np.abs(arc[j:] - v).sum(axis=1)))
        idxs.append(k)
        j = k
    runs, s = [], 0                   # s = 当前合并组的起点（idxs 下标）
    for t in range(1, len(idxs)):
        seg_v = arc[idxs[t]] - arc[idxs[t - 1]]      # 新段的方向
        merged_v = arc[idxs[t]] - arc[idxs[s]]       # 合并组首→当前顶点的整弦方向
        if (t - 1 > s
                and np.hypot(*seg_v) > 0 and np.hypot(*merged_v) > 0
                and abs(_ang_diff(_angle_of(seg_v[0], seg_v[1]),
                                  _angle_of(merged_v[0], merged_v[1]))) > _SEG_MERGE_DEG):
            runs.append((idxs[s], idxs[t - 1]))
            s = t - 1
    runs.append((idxs[s], idxs[-1]))
    return runs


# ==================== 单帧检测 ====================
def _detect_one(frame, p):
    """单帧：复用看路的 Y−B 自适应掩膜抠黄区 → 最大连通块边界点 → Huber 拟合 +
    剔点重拟合 → 质量门（直段长/RMS/内点数）→ 图像方向角。角度口径见文件头。"""
    fail = dict(success=False, reason="", angle=0.0, cols=0, rms=0.0, span=0.0,
                flags=0, mask=None, roi=None, scale=1.0, thr=None, note="",
                line=None, pts=None)
    x0, y0, x1, y1 = _roi_rect(frame.shape, p)
    sub = frame[y0:y1, x0:x1]
    sc = float(p["proc_scale"])
    if abs(sc - 1.0) > 1e-6:
        sub = cv2.resize(sub, None, fx=sc, fy=sc, interpolation=cv2.INTER_AREA)
    hh, ww = sub.shape[:2]

    mask, D, valid, thr, note = _warm_mask(sub, p)
    if mask is None:
        return dict(fail, reason=note, roi=(x0, y0, x1, y1), scale=sc, thr=None,
                    note=note)
    n, lab, st, _ce = cv2.connectedComponentsWithStats(mask, 8)
    if n < 2:
        return dict(fail, reason="窗内没有黄色区（自适应阈值下无亮块）", mask=mask,
                    roi=(x0, y0, x1, y1), scale=sc, thr=thr, note=note)
    # 取最大连通块（同 _road.py 的纪律：车上黄线标/另一条胶带是小的，取最大天然躲开）
    i = 1 + int(np.argmax(st[1:, cv2.CC_STAT_AREA]))
    a_full = float(st[i, cv2.CC_STAT_AREA]) / (sc * sc)
    if a_full < p["min_area"]:
        return dict(fail, reason="黄色区太小（%.0f < %.0f px²）" % (a_full, p["min_area"]),
                    mask=mask, roi=(x0, y0, x1, y1), scale=sc, area=a_full,
                    thr=thr, note=note)
    # 掩膜泛滥闸门（同 _road.py 2026-10-09 午加：整幅染黄时直边拟合没有任何意义）
    roi_area_full = float((x1 - x0) * (y1 - y0))
    if a_full > float(p["max_area_frac"]) * roi_area_full:
        return dict(fail, reason="黄区面积占 ROI %.0f%% > %d%%（掩膜泛滥）—— "
                                 "多半白平衡/曝光没锁住、整幅染黄，不给数"
                                 % (100.0 * a_full / roi_area_full,
                                    int(100.0 * p["max_area_frac"])),
                    mask=mask, roi=(x0, y0, x1, y1), scale=sc, area=a_full,
                    thr=thr, note=note)
    # 黄度对比度闸门（同 _road.py 2026-10-09：阴影/杂色块不够"比背景黄"，不给数）
    contrast, _n_out = _warm_contrast(D, valid, lab, i)
    if contrast is not None and contrast < float(p["contrast_min"]):
        return dict(fail, reason="黄区不够黄（块内比背景仅黄 %.0f 级 < %.0f）—— "
                                 "锁到的多半是阴影/杂色块，不给数"
                                 % (contrast, p["contrast_min"]),
                    mask=mask, roi=(x0, y0, x1, y1), scale=sc, area=a_full,
                    thr=thr, note=note)

    # 最大块的边界点（处理尺度、逐像素）：CHAIN_APPROX_NONE 才是"沿着边界走的点列"，
    # 拟合直线要的就是它们（APPROX_SIMPLE 会把直线段压成两个端点，点数直接没了）。
    cnt = max(cv2.findContours((lab == i).astype(np.uint8) * 255,
                               cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0],
              key=cv2.contourArea)
    pts = cnt.reshape(-1, 2).astype(np.float32)
    # ⚠️⚠️ **先剔掉贴窗边的点再拟合** —— 黄区通常比窗大, 掩膜被窗矩形裁断的地方,
    #    轮廓会**沿着窗边框走一段**; 窗边框是**车固连**的直线, 它在图像里永远不动,
    #    混进拟合 = 测到一个常数（正是"用车身当基准"那个死胡同的缩影）。
    #    只留离窗边 ≥ EDGE_MARGIN 的点, 剩下的才是真正**穿过**窗的黄区边界。
    #    剔完没点了 = 窗完全罩在黄区内部（或外面）, 边界根本没穿过窗 —— 让用户挪窗。
    m = float(p["edge_margin"])
    keep_border = ((pts[:, 0] > m) & (pts[:, 0] < ww - 1 - m)
                   & (pts[:, 1] > m) & (pts[:, 1] < hh - 1 - m))
    idx = np.nonzero(keep_border)[0]
    if len(idx) < p["min_cols"]:
        return dict(fail, reason="边界点太少（%d < %d，掩膜碎了？或窗罩在黄区"
                                 "内部/外部、边界没穿过窗 —— 挪窗）"
                                 % (len(idx), p["min_cols"]),
                    mask=mask, roi=(x0, y0, x1, y1), scale=sc, thr=thr, note=note)

    # ---- 分段 → 最长直段候选 → 逐段两遍拟合（2026-10-08 改, 治"两条边轮流当直线"）----
    # 窗罩着**整块**黄区时（ROADYAW_ROI 全宽的常态）, 最大块的边界是个**闭环**:
    # 真直边只是其中一段, 另有撕边/褶皱/弧段混在里面。旧版对全部边界点做一次全局
    # Huber —— 直边占压倒性多数时没事, 撕边也有几百点时 Huber 就在「直边」「撕边」
    # 「两者的折中线」三种形态间**逐帧翻转**（实测同一机位 40 帧方向角扫了 7.8°,
    # 多帧投票直接两簇/散布超限）。现在把每段有序边界弧**折线化**（approxPolyDP,
    # 弦偏差容差 _SEG_EPS_PX）: 撕边碎成一段段短弦过不了 MIN_SPAN 门, 真直边保持
    # 一整条 —— 候选段各自跑原有的两遍 Huber + 质量门, 按**拟合后跨度最长**择优,
    # 角度只由那条真直边决定（两条真直边都够长时仍由多帧投票的双簇判据兜底）。
    best = None          # (span, rms, n_ip, flags, line, ip) —— 拟合后跨度最长者胜
    fail_note = "窗里没有像样的直边（最长的分段连 MIN_SPAN/MIN_COLS 都够不着 —— 挪窗罩住直边）"
    brk = np.nonzero(np.diff(idx) > _ARC_GAP)[0]
    for s, e in zip(np.r_[0, brk + 1], np.r_[brk, len(idx) - 1]):
        arc = pts[idx[s]:int(idx[e]) + 1]
        for i0, i1 in _straight_segments(arc):
            seg = arc[i0:i1 + 1]
            chord = float(np.hypot(*(arc[i1] - arc[i0]))) / sc      # 回全分辨率
            if chord < p["min_span_px"] or len(seg) < p["min_cols"]:
                continue
            # 两遍拟合：Huber 压野点 → 按残差剔点 → 内点重拟合（段端点沾到拐角时甩掉）
            vx, vy, px, py = cv2.fitLine(seg, cv2.DIST_HUBER, 0, 0.01, 0.01).flatten()
            res = np.abs(vx * (seg[:, 1] - py) - vy * (seg[:, 0] - px))
            keep = res <= p["inlier_px"]
            if int(np.count_nonzero(keep)) < p["min_cols"]:
                continue
            ip = seg[keep]
            vx, vy, px, py = cv2.fitLine(ip, cv2.DIST_HUBER, 0, 0.01, 0.01).flatten()
            res2 = np.abs(vx * (ip[:, 1] - py) - vy * (ip[:, 0] - px))
            rms = float(np.sqrt(np.mean(res2 * res2)))
            t = (ip[:, 0] - px) * vx + (ip[:, 1] - py) * vy         # 内点在直线上的投影
            span = float(t.max() - t.min()) / sc                     # 回全分辨率
            if span < p["min_span_px"]:
                fail_note = "直段太短（%.0f < %.0f px，窗开大点或换个位置）" % (span, p["min_span_px"])
                continue
            if rms > p["max_rms_px"]:
                fail_note = ("边界不够直（RMS %.1f > %.1f px，窗里混了弧段/褶皱？）"
                             % (rms, p["max_rms_px"]))
                continue
            flags = FLAG_FIT_OK
            if len(ip) < 2 * p["min_cols"] or span < 2.0 * p["min_span_px"]:
                flags |= FLAG_FEW_PTS      # 刚够门限（勉强）—— 遥测提示, 不判失败
            cand = (span, rms, len(ip), flags,
                    (float(vx), float(vy), float(px), float(py)), ip)
            if best is None or (span, -rms) > (best[0], -best[1]):
                best = cand
    if best is None:
        return dict(fail, reason=fail_note, mask=mask,
                    roi=(x0, y0, x1, y1), scale=sc, thr=thr, note=note)

    span, rms, n_ip, flags, line, ip = best
    ang = _angle_of(line[0], line[1])
    return dict(success=True, reason="", angle=ang, cols=min(n_ip, 255),
                rms=rms, span=span, flags=flags, mask=mask,
                roi=(x0, y0, x1, y1), scale=sc, thr=thr, note=note,
                line=line, pts=ip)


def _annotate(frame, res):
    """画 ROI、掩膜轮廓、拟合直线（全线延长）与结果文本（全部 ASCII，cv2 画不了中文）。"""
    if frame is None:
        return None
    out = frame.copy()
    res = res or {}
    roi = res.get("roi")
    if roi is not None:
        cv2.rectangle(out, (int(roi[0]), int(roi[1])), (int(roi[2]), int(roi[3])),
                      _ROI_BGR, 2)
    mask = res.get("mask")
    if mask is not None:
        sc = res.get("scale", 1.0) or 1.0
        ox, oy = (roi[0], roi[1]) if roi is not None else (0, 0)
        for c in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
            q = (c.reshape(-1, 2).astype(float) / sc + np.array([ox, oy])).astype(np.int32)
            cv2.polylines(out, [q], True, _MASK_BGR, 3)
    line = res.get("line")
    if line is not None and roi is not None:
        vx, vy, px, py = line
        # 处理尺度 ROI 坐标 → 全分辨率: 除 scale + ROI 原点; 沿方向延长到超出 ROI 为止
        cx, cy = px / sc + roi[0], py / sc + roi[1]
        diag = float(np.hypot(roi[2] - roi[0], roi[3] - roi[1]))
        p_a = (int(cx - vx * diag), int(cy - vy * diag))
        p_b = (int(cx + vx * diag), int(cy + vy * diag))
        cv2.line(out, p_a, p_b, _LINE_BGR, 2)
        ang = res.get("angle", 0.0)
        cv2.putText(out, "ya=%.2f deg cols=%d rms=%.1fpx fl=0x%02X"
                    % (ang, res.get("cols", 0), res.get("rms", 0.0),
                       res.get("flags", 0)),
                    (20, 40), 0, 1.1, _LINE_BGR, 3)
    elif res.get("reason"):
        cv2.putText(out, "road_yaw fail", (20, 40), 0, 1.1, (0, 0, 255), 3)
    return out


# ==================== 多帧投票 ====================
def run_road_yaw_detect(timeout=None, abort_event=None):
    """主摄「看路校准」：拟合黄区直边的图像方向角（只测不改 —— 详见文件头）。
    限时 timeout 秒，默认 config.ROADYAW_TIMEOUT。

    流程与 run_road_detect 同构：逐帧 _detect_one → 先判双峰 → MAD 剔离群 →
    内点取均值；凑满 min_votes 且极差 ≤ 离群下限就早停（早停条件比最终判据严格，
    早停结果与采满逐比特同判）。差别只在量纲：px → °。

    Returns dict: {"success", "reason", "frame", "angle", "cols", "rms", "span",
                   "flags", "votes", "inliers", "dropped", "spread", "frames", "thr"}
        `angle` = 图像方向角 [°]（竖直=0、向画面右倒为正、(−90,+90]）→ 单片机遥测 roadyaw_ang,
                  err = (angle − ideal)×dir。
        `spread`/`frames` = 跨帧散布与各帧读数 [°]（遥测）。
        aborted=True 只在被打断时出现。
    """
    from config import ROADYAW_TIMEOUT, ROADYAW_ATTEMPTS

    timeout = ROADYAW_TIMEOUT if timeout is None else timeout
    fail = {"success": False, "reason": "", "frame": None, "angle": 0.0,
            "cols": 0, "rms": 0.0, "span": 0.0, "flags": 0, "votes": 0,
            "inliers": 0, "dropped": 0, "spread": 0.0, "frames": [], "thr": None}

    cap = acquire_camera(is_qr=False, profile="road")
    if cap is None:
        if abort_event is not None and abort_event.is_set():
            print("[INFO] 看路校准任务被打断")
            return dict(fail, aborted=True)
        return dict(fail, reason="主摄像头打开失败")

    try:
        p = _params()
        p["attempts"] = max(1, int(ROADYAW_ATTEMPTS))
        print(f"[INFO] 开始看路校准（黄区直边方向角，最多 {p['vote_frames']} 帧/轮 × "
              f"{p['attempts']} 轮，限时 {timeout:.0f}s）...")
        deadline = time.time() + timeout

        # ---- 瞬态失败自动重采（2026-10-08 加; 轮数 = ROADYAW_ATTEMPTS, 默认 3）----
        # 篷布机位的单帧方向角噪声 ~0.15°（正赛黄胶带更小），绝大多数轮次投票稳过；
        # 偶发一帧曝光/掩膜抖动会让某一轮内点散布撞上 MAX_SPREAD 闸（40 帧离线回放
        # 实测 ~10% 的 9 帧窗）。这种瞬态**重采就好了**，不该把"再按一次 H"的成本
        # 甩给现场 —— 同一预算内（超时照旧卡死）自动重采, 最多 ROADYAW_ATTEMPTS 轮。
        # 闸门一个不放松：车真在动时轮轮都散，照样响亮失败；双簇（真两条直边）不重采。
        attempt = 0
        last_raw = None    # 最近读到的原始帧 —— 打断/失败也带帧（统一记录接口,
                           # 见 handler._save_frame: 失败样本恰是排障最想看的）
        while True:
            hits = []
            frames_seen = 0
            while time.time() < deadline and frames_seen < p["vote_frames"]:
                if abort_event is not None and abort_event.is_set():
                    print("[INFO] 看路校准任务被打断")
                    return dict(fail, aborted=True, frame=last_raw)
                frame = CameraManager.read(cap, "Main Camera", max_retries=3)
                if frame is None:
                    time.sleep(0.05)
                    continue
                last_raw = frame
                frames_seen += 1
                r = _detect_one(frame, p)
                if r["success"]:
                    hits.append((r, frame))
                    # ---- 早停（° 口径, mod 180 圆极差, 见 run_road_detect 同名段）----
                    angs = [h[0]["angle"] for h in hits]
                    if len(hits) >= p["min_votes"]:
                        ref0 = _circular_mean_deg(angs)
                        rel0 = [_ang_diff(a, ref0) for a in angs]
                        if _spread(rel0) <= p["outlier_min_deg"]:
                            print(f"[INFO] 看路校准早停: {len(hits)} 帧圆极差 "
                                  f"{_spread(rel0):.3f}° 已够稳")
                            break
                else:
                    print(f"[WARN] 看路校准第 {frames_seen} 帧失败: {r['reason']}")
                time.sleep(0.01)

            def _retry(reason_dict, why):
                """瞬态失败 → 还剩 ≥0.3s 且没打满轮数上限就重采一轮；轮数耗尽才认。"""
                nonlocal attempt
                if attempt < p["attempts"] - 1 and time.time() < deadline - 0.3:
                    attempt += 1
                    print(f"[INFO] 看路校准{why}，第 {attempt + 1}/{p['attempts']} "
                          f"轮重采（限时内）...")
                    return None
                return reason_dict

            if len(hits) < p["min_votes"]:
                reason = (f"看路校准失败：{frames_seen} 帧里只有 {len(hits)} 帧检到直边 "
                          f"(需≥{p['min_votes']})")
                print(f"[WARN] {reason}")
                best = hits[0] if hits else None
                out = _retry(dict(fail, reason=reason, votes=len(hits),
                                  frame=_annotate(best[1], best[0])
                                  if best else last_raw),
                             "有效帧不足")
                if out is not None:
                    return out
                continue

            angs = [h[0]["angle"] for h in hits]
            # ---- 方向角是 mod 180° 的量: 先折到"圆均值参考系"再投票 ----
            # ⚠️⚠️ 贴着 ±90° 边界的直线（本机位上篷布下边缘就是近水平的）, 相邻两帧读数在
            #    +89.9 与 −89.9 之间跳 —— 数值差 180°, 物理只差 0.2°。直接拿原始读数做
            #    双簇/散布判据, 会把同一簇判成「两簇差 180°」或「散布 179.9° 超限」
            #    （画面明明很干净 —— 2026-10-08 实测踩过）。处方: 圆均值 ref → 全体帧
            #    折成对 ref 的有符号小差 rel（都在 0 附近）→ 双簇/MAD/散布照旧跑在线性量上。
            ref = _circular_mean_deg(angs)
            rel = [_ang_diff(a, ref) for a in angs]
            # ① 先判双峰（少数簇会被 MAD 当离群剔掉 → 静默错值, 顺序不能反）
            #    ⚠️ 间隔不足 _TWO_CLUSTER_MIN_GAP_DEG 的"两簇"是噪声尾巴凑票, 不算 ——
            #    交给 ②的 MAD（剔尾）+ ③的散布闸, 别为 0.3~0.5° 的分缝报废一次测量。
            gap = _ambiguous_gap(rel, {"min_votes": p["min_votes"],
                                       "max_spread_px": max(p["max_spread_deg"],
                                                            _TWO_CLUSTER_MIN_GAP_DEG)})
            if gap > 0.0:
                reason = (f"方向角分成两簇（差 {gap:.2f}° 且两边都够票数）—— "
                          f"窗里有两条都够长的直边在轮流赢「最长」？收窄窗只留一条")
                print(f"[WARN] {reason}  各帧 ang={[round(v, 2) for v in angs]}")
                return dict(fail, reason=reason, votes=len(hits),
                            frame=_annotate(hits[0][1], hits[0][0]),
                            frames=[round(v, 2) for v in rel])

            # ② MAD 剔跳档（° 口径的离群判据, 跑在折过的 rel 上）
            keep = _inlier_mask(rel, {"outlier_k": p["outlier_k"],
                                      "outlier_min_px": p["outlier_min_deg"]})
            idx = [k for k in range(len(hits)) if keep[k]]
            dropped = len(hits) - len(idx)
            if len(idx) < p["min_votes"]:
                reason = (f"看路校准抖动过大：{len(hits)} 帧里剔除 {dropped} 个离群帧后只剩 "
                          f"{len(idx)} 帧")
                print(f"[WARN] {reason}  各帧 ang={[round(v, 2) for v in angs]}")
                out = _retry(dict(fail, reason=reason, votes=len(hits), inliers=len(idx),
                                  dropped=dropped, frame=_annotate(hits[0][1], hits[0][0]),
                                  frames=[round(v, 2) for v in rel]),
                             "剔离群后有效帧不足")
                if out is not None:
                    return out
                continue

            # ③ 内点取均值: 圆均值 ref + 内点线性均值, 再折回 [−90,+90)
            angle = _wrap180(ref + sum(rel[k] for k in idx) / len(idx))
            spread = _spread([rel[k] for k in idx])
            if spread > p["max_spread_deg"]:
                reason = f"方向角跨帧散布 {spread:.2f}° 超限（车在动？）"
                print(f"[WARN] 看路校准{reason}")
                out = _retry(dict(fail, reason=reason, votes=len(hits), inliers=len(idx),
                                  dropped=dropped, spread=spread,
                                  frame=_annotate(hits[0][1], hits[0][0]),
                                  frames=[round(v, 2) for v in rel]),
                             f"散布 {spread:.2f}° 超限")
                if out is not None:
                    return out
                continue
            break

        # 遥测量跟着内点帧取均值; 标注画在"最接近均值"的那一帧上
        cols = int(round(sum(hits[k][0]["cols"] for k in idx) / len(idx)))
        rms = sum(hits[k][0]["rms"] for k in idx) / len(idx)
        span = sum(hits[k][0]["span"] for k in idx) / len(idx)
        rep = min(idx, key=lambda k: abs(hits[k][0]["angle"] - angle))
        flags = 0
        for k in idx:                       # 任何一帧"勉强"都算（保守）
            flags |= hits[k][0]["flags"]
        res = dict(success=True, reason="", angle=angle, cols=cols, rms=rms,
                   span=span, flags=flags, votes=len(hits), inliers=len(idx),
                   dropped=dropped, spread=spread,
                   frames=[round(v, 2) for v in rel])   # 对圆均值的偏差（°）
        print("[INFO] 看路校准成功: ang=%.2f° cols=%d rms=%.1fpx 直段=%.0fpx "
              "帧%d(剔%d) 散布%.2f° fl=0x%02X"
              % (angle, cols, rms, span, len(hits), dropped, spread, flags))
        print("[INFO]   各帧 ang=%s" % [round(v, 2) for v in angs])
        res["frame"] = _annotate(hits[rep][1],
                                 dict(hits[rep][0], angle=angle))
        res["frame_raw"] = hits[rep][1]     # 未标注的原始帧（训练模式成对归档用）
        return res
    finally:
        release_camera(cap)     # 租约句柄 → 归还（保持打开）；普通 cap → 真释放
