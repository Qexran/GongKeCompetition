# tasks/_target.py — 靶标识别 + 靶心定位（反恐区打靶，主摄像头，一次性）
#
# 指令 "6"：识别场上的三色靶盘（红/绿/蓝外环），定位**每一颗的圆心**，连同颜色一起
# 回传。任务层只返回结果 dict，串口回传由 handler.py::_cmd_target 统一发送
# （TYPE_COORD TAG=0x04 二进制帧，见 docs/串口协议.md §6e），保持任务函数纯净可测
# （与 _color.py / _hostage.py 同构）。
#
# ⚠️ **回传三颗而不是一颗**：哪一颗是目标由二维码第二位决定，而二维码结果是**单片机**
#    在解 —— Pi 侧不掺和任务语义（与色球那边完全一致：Pi 给三颗，单片机按目标色挑）。
#
# 算法来自 GongKeCompetition 仓库 tools/target_offline/ 的离线评估（61 个合成用例，
# 参数照现场实拍标定）。四条判据，各自治一类干扰，互补：
#
#   ⓪ **像不像靶环**（_pick_consistent 的 q 门槛 + _fit_hole_in）—— 2026-09 加，最先执行
#      q = 内孔拟合半径 ÷ 该候选外接圆半径，靶盘设计值 0.55。**只看单个候选自己的形状**，
#      所以"几个杂块凑巧一样大"骗不过它。跨色尺寸一致性（判据①）单独用是不够的：
#      现场同色干扰块够多时，三个杂块就能拼出一个合法的"尺寸一致组"把真靶挤掉
#      （症状：蓝靶有时指向画面角落一个 r≈50 的杂块）。实测真靶环 q=0.52~0.55、
#      一切杂块 q≤0.23，两者不重叠，详见 config.TARGET_RING_Q_*。
#
#   ① **跨色尺寸一致性**（_pick_consistent）—— 治「同色大块」
#      现场背景是一整面蓝柜子，蓝靶与它同色。实测蓝柜子外接圆半径 455px、红/绿靶 55px，
#      差 8.3 倍。⚠️ 做法必须是「在候选里挑一组尺寸匹配的」，**不能**是「各色取最大再校验」:
#      按最大取，蓝选出来就是柜子，而校验只能说"这个不对"、给不出正确的蓝靶
#      （真蓝靶在那 85 个候选里排第 30 名开外）。
#      离线的「三靶+同色圆环干扰」用例上，其余 7 个方法全部偏 354mm，只有它 0.45mm。
#      代价：必须有 ≥2 个靶同时在画面里（瞄准时凑近某一个靶、另外两个出画就失效）。
#
#   ② **内边界优先、外边界兜底**（_locate_best）—— 不依赖同伴，只看到一个靶也成立
#      现场样件是手工/3D 打印的，外缘是手工贴的波浪边。实拍量得：
#        外边界（色环外缘）    拟合椭圆残差 std 2.15px（最大 5.8px）→ **物理上不是圆**
#        内边界（色环↔白心）   残差 std 0.44px（像素化下限约 0.3px）→ 干净的圆
#      所以圆心以内边界为准；内边界被遮挡/裁断时退回外边界（外缘糙但几乎总能拟合上）。
#
#   ③ **ROI 裁切**（可关，见 config.TARGET_USE_ROI）—— 最省事的一道
#      瞄准闭环本来就会把靶心拉到画面中心附近，「靶心应该在的地方」是个已知的小窗口，
#      窗口外的干扰根本不用去看。⚠️ 默认**关**：窗口位置得看着真实瞄准位姿定，
#      配错了会静默地什么都找不到。
#
# ⚠️ 多帧投票要求**小车/机械臂静止**：所有票必须来自同一机位。
# ⚠️ 坐标一律**全分辨率**（与固件的标定量同一口径）。降采样提速时按 1/scale 还原。
import math
import struct
import time

import cv2
import numpy as np

from camera import CameraManager
from camera_lease import acquire_camera, release_camera

TARGET_TAG = 0x04   # 结果帧子命令标记（TYPE_COORD DATA 首字节）

# 颜色码 → 画面标注用英文短名（cv2.putText 画不了中文）
_COLOR_EN = {1: "R", 2: "G", 3: "B"}
_COLOR_BGR = {1: (0, 0, 255), 2: (0, 255, 0), 3: (255, 0, 0)}
_BOX_BGR = (0, 255, 255)   # 框/十字统一黄色（画在靶自己的颜色上看不见）

# 靶盘设计比例：色环内边界 Ø55 / 外缘 Ø100（命题图 7）。只用来把内圆的拟合半径折算成
# **外圆等效半径**，让回传的 r 与固件那条视差公式 `offset = h·r_px/50` 同一口径。
_R_INNER_OVER_OUTER = 27.5 / 50.0


def _clamp_u16(v):
    return max(0, min(65535, int(round(v))))


# ==================== 基础几何 ====================

def _fit_circle_lsq(pts):
    """点集 → (cx, cy, r)。Kåsa 代数解（闭式）。对离群点敏感，外面套鲁棒剔除。"""
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
    return float(cx), float(cy), float(math.sqrt(r2))


def _fit_circle_robust(pts, rounds=4):
    """Kåsa + 按半径残差 MAD 迭代剔离群点。

    ⚠️ `rounds` 别调小：外缘有波浪/缺一块时，剔除要跑满几轮才收敛
    （实测 rounds=3 时在遮挡用例上偏到 r=58（真值 68），rounds=4 才回到 67.8）。
    """
    p = np.asarray(pts, np.float64).reshape(-1, 2)
    best = None
    for _ in range(max(1, rounds)):
        if len(p) < 3:
            break
        f = _fit_circle_lsq(p)
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


def _fit_hole_in(cnts, hier, i0, min_fill=0.75):
    """从**已在手上的** CCOMP 轮廓树里挑最大内孔并拟合 → (cx,cy,r) 或 None。

    `i0` 是该候选的顶层轮廓下标；内孔 = 它的直接子轮廓。

    ⚠️ 只认**这一个候选**的孔：轮廓树来自「只含这一个候选」的掩膜
    （见 _color_candidates 的说明），所以这里天然不会跨候选抓错。

    ⚠️ 内孔半径的用途有两个，都别丢：① 判据② 拿它当圆心（内边界比外缘干净得多）；
    ② 判据②' 拿 `内孔半径 ÷ 候选外接圆半径`（即 config.TARGET_RING_Q_*）判
    「这个候选到底像不像一个靶环」。

    孔洞可能是被切掉一块的月牙，这时圆拟合看着有解、中心已经飞了。判据用
    「轮廓面积 / 拟合圆面积」：完整圆 ≈ 1.0，缺一大块会明显偏小（0.75 ≈ 允许缺 1/4）。
    """
    kids = [j for j in range(len(cnts)) if hier[j][3] == i0]
    if not kids:
        return None
    hole = max((cnts[j] for j in kids), key=cv2.contourArea)
    f = _fit_circle_robust(hole.reshape(-1, 2))
    if f is None or f[2] < 3.0:
        return None
    if cv2.contourArea(hole) / (math.pi * f[2] * f[2]) < min_fill:
        return None
    return f


# ==================== 找候选（按颜色，且**全部**保留）====================

def _color_mask(hsv, color_code, s_min, v_min, families):
    """按色相族从 **HSV 图**取色环掩膜 → uint8（255 = 该色）。

    色相族直接复用 config.COLOR_HUE_FAMILIES —— 「什么算红」这件事**只留一个来源**，
    色球和靶标共用，免得两处阈值各自漂。

    ⚠️ 入参是**已经转好的 HSV 图**（不是 BGR 原图）：三色各转一次 HSV 是白烧
    （1920×1080 下一次 cvtColor ~13ms），由 _detect_frame 转一次传进来。
    ⚠️ 用 cv2.inRange（C 实现、无中间临时数组）而不是 numpy 比较：
    后者要先把三个通道 astype(np.int16)，1920×1080 下每次调用 churn 掉 ~12MB，
    而本函数**每帧每色都要调一次**（3 次）。
    """
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in families:
        # (H∈[lo,hi]) & (S≥s_min) & (V≥v_min)
        mask |= cv2.inRange(hsv, (int(lo), int(s_min), int(v_min)),
                            (int(hi), 255, 255))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)    # 去零散噪点
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)   # 补盘面高光/贴纸缝
    return mask


def _color_candidates(hsv, color_code, min_area, s_min, v_min, families,
                      r_max_full=None, scale=1.0):
    """列出该颜色的**所有**候选连通域（不是只取最大）。

    ⚠️ 「取最大色块」在真场地是错的：蓝靶在蓝色候选里排第 30 名开外，取最大只会拿到柜子。

    ⚠️⚠️ 每个候选要拿到**只含自己**的掩膜（用连通域标号分量），否则后续找孔会跨候选
    抓错（见 _fit_hole_in）。这里不能用 max_area 上限去"排除大块"：靶和干扰物
    可能**挨在一起被合并**成一个大块，排掉大块等于排掉靶（而那个合并块的内孔仍在，
    本来可以救回来的）—— 真踩过，设了 0.25 上限后蓝色一个候选都不剩、直接全灭。

    ⚠️⚠️ **逐候选的掩膜只在它自己的包围盒里做**（本函数 2026-09 的性能重写）。
    原先对每个候选做一次整帧 1920×1080 的 `(labels==lab)` 再 findContours：
    现场一帧有 15~30 个候选 × 3 色，光这一步就吃掉 ~450ms（整帧的 65%）。
    改成按 bbox 切片后同样的像素集合、同样一棵 CCOMP 轮廓树，只是坐标基准变成
    候选自己的左下角 —— **所以候选里额外带 `ox`/`oy`，由 _locate_one 加回全帧坐标**。

    r_max_full（**全分辨率** px，配 scale 换算）是候选级**绝对尺寸上限**：靶在
    设计射击距离上 r≈90~110、凑到最近 ~150，背景挡板/柜子的连通域 r 数百 ——
    用「靶就这么大」这条与颜色无关的物理事实把背景大块在候选级就扔掉（2026-10
    蓝靶跨帧散布 1272px 作废的根因之一）。它和上面被否掉的 max_area 上限不同：
    那是「怕大」怕误杀**合并块**（相对量、随场景变），这是**绝对**尺寸线，
    合并块（靶+椅腿级别）远够不着 200px。
    """
    mask = _color_mask(hsv, color_code, s_min, v_min, families)
    nlab, labels, stats, _cents = cv2.connectedComponentsWithStats(mask, 8)
    out = []
    # 面积降序（连通域标号 lab = argsort 下标 + 1）；面积已降序 → 遇到不够大的可整批跳出
    areas = stats[1:, cv2.CC_STAT_AREA]
    for k in np.argsort(-areas):
        lab = int(k) + 1
        area = float(stats[lab, cv2.CC_STAT_AREA])
        if area < min_area:
            break
        ox = int(stats[lab, cv2.CC_STAT_LEFT])
        oy = int(stats[lab, cv2.CC_STAT_TOP])
        w = int(stats[lab, cv2.CC_STAT_WIDTH])
        h = int(stats[lab, cv2.CC_STAT_HEIGHT])
        # ⚠️⚠️ 绝对尺寸上限的 **bbox 几何预检**（2026-10-06 加, 排在一切昂贵工作之前）:
        #    bbox 是紧的（min/max 都由真实像素取得）→ 外接圆半径必然 ≥ max(w,h)/2。
        #    所以 max(w,h)/2/scale 已超 r_max_full 的候选**必**过不了下面的 r_max 门,
        #    可以在抽掩膜/findContours 之前就跳过 —— 这不是近似, 是零误杀的等价改写。
        #    台架实测的病根（06:59 那次 658ms/帧 → 流 2fps → 单片机新鲜度闸冻结）:
        #    跟踪窗一扩张, 蓝柜子进窗 → 旧顺序先对柜子做全套掩膜+轮廓（数百 ms）
        #    再用 r_max 把它扔掉 —— 每帧白干一遍重活。
        if r_max_full is not None and (max(w, h) * 0.5 / scale) > r_max_full:
            continue
        comp = (labels[oy:oy + h, ox:ox + w] == lab).astype(np.uint8) * 255
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
        # 绝对尺寸上限（全分辨率口径，换算回本候选所在的降采样坐标系再比）。
        # 背景大块在这里就出局，进不了候选表 → 共识、兜底两条路都碰不到它。
        if r_max_full is not None and (r / scale) > r_max_full:
            continue
        # 内孔（色环↔白心）就地拟合 —— 轮廓树已经在手上，不另开一遍 findContours。
        # q = 内孔半径 ÷ 本候选外接圆半径：靶盘设计值 0.55，是「像不像靶环」的判据
        # （见 config.TARGET_RING_Q_* 与 _pick_consistent）。
        hole_fit = _fit_hole_in(cnts, hier, i0)
        out.append({"color": color_code, "area": area, "r": float(r),
                    "cx": float(cx) + ox, "cy": float(cy) + oy,
                    "ox": ox, "oy": oy,
                    "contour": c, "mask": comp, "hole_fit": hole_fit,
                    "q": (hole_fit[2] / r) if (hole_fit and r > 0) else 0.0,
                    "holes": sum(1 for j in range(len(cnts)) if hier[j][3] == i0)})
    out.sort(key=lambda d: -d["area"])
    return out


# ==================== 判据①：跨色尺寸一致性 ====================

def _pick_consistent(cands_by_color, tol, per_color=12, min_colors=2,
                     q_min=0.35, q_max=0.85):
    """**在各色的候选里找一组尺寸互相匹配的**。→ {color: cand}

    做法：拿每个候选当种子尺寸 s，数一数有多少个**不同的颜色**在 `[s/tol, s·tol]`
    里都有候选；取覆盖颜色最多的那个 s，再让每个颜色挑最接近 s 的那个候选。

    ⚠️⚠️ **锚点不能是「各色最大」**。曾经就是这么写的（每色取最大 → 按中位数剔离群色），
    在**真实场地帧上直接失败**：红靶旁边的木椅子是个 r=311 的红块、蓝柜子是 r=448 的蓝块，
    两个都盖过了真靶（真靶 r≈62/79/65）→ 「中位数」被拉到 311 → **真靶反倒成了"离群"
    被剔掉**。症状：红靶检不出、蓝靶检成了柜子的一块。
    「取最大」在干扰物比目标大时是**必然**错的；而"哪个是干扰物"恰恰是这里要判的事。

    ⚠️ tol 不能太紧：三个靶并排站着，但相机凑近某一个时近的会明显大一些。
    实测同距时三靶半径差 <10%；留到 2.0 是给「凑近照某一个」留余量。

    ⚠️ 只有 1 个颜色可见时（覆盖数够不上 min_colors）返回空 —— 调用方要兜底
    （这时没有尺寸基准，只能退回该色最大候选，再由内孔/圆度判据把关）。

    ⚠️⚠️ **进池子之前先按「像不像靶环」筛一遍（判据②'，2026-09 加）**，这一步比尺寸
    判据根本得多。使用方 q = `内孔拟合半径 ÷ 该候选外接圆半径`，靶盘设计值 0.55。
    为什么非它不可：**尺寸一致性只是必要条件、远不是充分条件**——现场同色的干扰块
    （蓝柜子/蓝箱/木椅腿/背景碎块）够多时，"三个杂块凑巧一样大"就会拼出一个合法的
    跨色组，把真靶挤掉。实测（现场帧、蓝靶）：
      真靶环 绿/蓝 q = 0.52~0.55（连续 8 帧都稳定）；一切杂块 q ≤ 0.23
      （蓝柜子 0.00~0.03、蓝箱 0.00~0.06、其余 0.00~0.23）—— 两者完全不重叠。
    而 q 只看**单个候选自己的形状**，跟别的候选没关系，所以"凑巧一样大"骗不过它。
    曾经只按「有没有内孔」筛（Q>0 的弱化版），那还不够：杂块的孔也常常过得了
    fill≥0.75 那一关（只是**长得不像**靶环），于是仍能进池子。症状正是
    「蓝靶有时指向画面角落一个 r≈50 的杂块」。

    ⚠️ 被 q 判掉 ≠ 丢弃：`_detect_frame` 末尾还有「没被共识覆盖的颜色退回该色最大候选」
    那条兜底，**靶与干扰物合并成一个连通域**时（红靶与木椅腿，q 掉到 0.13）靠它救回来。
    两道关分工不同，别合成一道。
    """
    ring = {c: [x for x in lst if q_min <= x.get("q", 0.0) <= q_max]
            for c, lst in cands_by_color.items()}
    # 像靶环的凑不齐两色就退回全部（靶被遮挡时外轮廓会撑大、q 会掉出去，别把路堵死）
    work = ring if sum(1 for v in ring.values() if v) >= min_colors else cands_by_color

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

    # 没被覆盖的颜色：挑一个尺寸跟共识最接近的（而不是"最大的那个" = 通常就是干扰物）
    ref = float(np.median([c["r"] for c in picked.values()])) if picked else 0.0
    for col, lst in work.items():
        if col in picked or not lst:
            continue
        cand = min(lst[:per_color], key=lambda c: abs(c["r"] - ref))
        if ref > 0 and (cand["r"] / ref <= tol) and (ref / cand["r"] <= tol):
            picked[col] = cand
    return picked


# ==================== 判据②：内边界优先、外边界兜底 ====================

def _locate_one(cand):
    """候选 → 圆心 (cx, cy, r_outer_equiv_px)。内边界优先，外边界兜底。→ tuple 或 None

    ⚠️ `cand["mask"]/["contour"]` 都是**候选自己包围盒里**的局部坐标，
    这里统一用 `cand["ox"]/["oy"]` 加回全帧坐标再返回（细节见 _color_candidates）。
    """
    ox, oy = cand.get("ox", 0), cand.get("oy", 0)
    f_in = cand.get("hole_fit")
    if f_in is not None:
        return f_in[0] + ox, f_in[1] + oy, f_in[2] / _R_INNER_OVER_OUTER

    # 兜底：外缘。**必须过圆度这一关** —— 否则柜子/桌角那种大体量矩形也会被
    # _fit_circle_robust 拟出一个"圆"来（它不会返回 None，只是残差大）。
    f_out = _fit_circle_robust(cand["contour"].reshape(-1, 2))
    if f_out is None or f_out[2] < 4.0:
        return None

    # 圆度关卡：**必须用稳健离散度（半径的 MAD）**，不能用全部点的 RMS。
    # ⚠️ 用 RMS 是错的：遮挡物的直边正是鲁棒拟合**故意剔掉**的离群点，把它们算进来
    #    必然超标 —— 实测一个拟合误差只有 0.2px 的**正确**圆被判成 rms/r=0.277 而丢弃
    #    （「遮挡左1/4」用例因此整颗靶不回传）。MAD 只看主体分布，不受少数离群点左右。
    p = cand["contour"].reshape(-1, 2).astype(np.float64)
    d = np.hypot(p[:, 0] - f_out[0], p[:, 1] - f_out[1])
    mad = float(np.median(np.abs(d - np.median(d))))
    if mad / max(f_out[2], 1e-6) > 0.06:      # 半径的中位绝对偏差超半径 6% → 不是圆
        return None
    return f_out[0] + ox, f_out[1] + oy, f_out[2]


def _detect_frame(img, cfg, scale=1.0, debug_cands=None):
    """单帧 → [{"color","cx","cy","r","holes"}, ...]（全分辨率像素）。

    scale<1 时先降采样提速，**返回坐标已按 1/scale 还原回全分辨率** ——
    协议与固件的 TARGET_* 标定量全按全分辨率定义，混用会让标定全废。

    debug_cands（可变 list，传了就往里追加）是离线回放器（tools/replay_target_tune.py）
    的观察口：每帧记一条 {color, area, r, q, cx, cy, hole, picked} ——
    picked 取 "consensus"（共识选中）/ "fallback"（兜底救回）/ ""（没被选上），
    供调参时看「候选到底死在哪一道关」。默认 None 时零开销、行为不变。
    """
    if scale != 1.0:
        small = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    else:
        small = img

    # ⚠️ cfg["families"] 是**整张** {颜色码: [(lo,hi),...]}；_color_mask 要的是该颜色的
    #    那一段，得按颜色取出来再传（传整张会 `for lo, hi in dict` → 解包到一个 int 上）。
    # HSV **只转一次**给三个颜色共用（原先每个颜色各转一次，白烧两遍）
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    cands = {c: _color_candidates(hsv, c, cfg["min_area"], cfg["sat_min"],
                                  cfg["val_min"], cfg["families"].get(c, []),
                                  r_max_full=cfg.get("r_max"), scale=scale)
             for c in (1, 2, 3)}
    picked = _pick_consistent(cands, tol=cfg["tol"],
                              q_min=cfg["q_min"], q_max=cfg["q_max"])

    # ⚠️ 尺寸一致性只会**剔除**颜色（挑不出匹配的就整个不要），但那一色可能本来有好候选。
    #    所以这里要**兜底**：被剔除的颜色退回它自己的候选，交给 _locate_one 的
    #    内孔/圆度判据做第二道关。
    #    真踩过：少了这一步，「三靶+同色柜子」里蓝靶因为柜子太大被剔除 → 直接不回传，
    #    而实际上靶和柜子**挨在一起合并成一个连通域**，那个合并块的内孔就是靶的白心，
    #    本来完全能救回来。
    #    ⚠️ 2026-10 改「取最大」为「**优先最大且有内孔的**」：兜底原本闭着眼睛取面积
    #       最大候选，可粘连/高光帧让真靶掉出共识后，蓝色面积最大的往往是**背景挡板块**
    #       —— 内孔拟合失败、_locate_one 走外缘兜底，圆心直接甩到背景上，跨帧散布
    #       爆到上千 px、整色作废（r_max 已把 r 数百的挡掉，这里再保证剩下的候选
    #       至少「有个像白心的孔」才配被兜底）。内孔（hole_fit）就是白心的直接证据：
    #       红靶合并块靠它救回（q=0.13 过不了共识的 q 门，但孔拟合得上），
    #       背景块靠它挡住。一个不剩时才退回旧语义（取最大），最坏情况=旧行为。
    consensus = dict(picked)          # 兜底前快照（值是同一批 cand 对象，用 is 区分来源）
    for col in (1, 2, 3):
        if col not in picked and cands.get(col):
            spare = [c for c in cands[col] if c["hole_fit"] is not None]
            picked[col] = spare[0] if spare else cands[col][0]

    if debug_cands is not None:
        for col, lst in cands.items():
            for c in lst:
                if picked.get(col) is c:
                    tag = "consensus" if consensus.get(col) is c else "fallback"
                else:
                    tag = ""
                debug_cands.append({
                    "color": col, "area": c["area"], "r": c["r"] / scale,
                    "q": c["q"], "cx": c["cx"] / scale, "cy": c["cy"] / scale,
                    "hole": c["hole_fit"] is not None, "picked": tag,
                })

    out = []
    for col, cand in picked.items():
        loc = _locate_one(cand)
        if loc is None:
            continue
        cx, cy, r = loc
        out.append({"color": col, "cx": cx / scale, "cy": cy / scale,
                    "r": r / scale, "holes": cand["holes"]})
    out.sort(key=lambda t: t["cx"])     # 按图像从左到右（与色球帧同约定）
    return out, small


def _annotate(frame, targets, extra=""):
    """框住每颗靶 + 标颜色与半径；完整读数排一行放画面顶部。

    ⚠️ 每颗只画**短标签**（"G r54"）居中在框上方。原来画的是
    "G (730,604) r=54" 这种长标签，三颗靶只隔 ~170px、标签却有 ~180px 宽，
    **三个标签会糊在一起完全读不出来**（现场帧上真踩过）。坐标改放顶部整行。
    ⚠️ 顶部那行只能是 ASCII —— cv2.putText 的内置字体画不了中文，
    直接塞 color_name（红/绿/蓝）会打出乱码。
    """
    out = frame.copy()
    for t in targets:
        cx, cy = int(t["cx"]), int(t["cy"])
        ri = max(6, int(t["r"]))
        cv2.rectangle(out, (cx - ri, cy - ri), (cx + ri, cy + ri), _BOX_BGR, 4)
        cv2.drawMarker(out, (cx, cy), _BOX_BGR, cv2.MARKER_CROSS, 28, 3)
        label = "%s r%d" % (_COLOR_EN.get(t["color"], "?"), ri)
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.8, 2)
        org = (max(0, cx - tw // 2), max(th + 6, cy - ri - 10))
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 5)
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    _COLOR_BGR.get(t["color"], (255, 0, 255)), 2)

    # 顶部整行读数（已按图像从左到右排序，与回传帧的顺序一致）
    line = "  ".join("%d.%s(%d,%d)r%d" % (i + 1, _COLOR_EN.get(t["color"], "?"),
                                          int(t["cx"]), int(t["cy"]), int(t["r"]))
                     for i, t in enumerate(targets)) or extra
    if line:
        cv2.putText(out, line, (16, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 0), 6)
        cv2.putText(out, line, (16, 38), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                    (255, 255, 255), 2)
    return out


def build_target_frame(ok, targets):
    """打包回传帧 DATA（见 docs/串口协议.md §6e）：
    [TAG=0x04][status u8][count u8] + count×([color u8][cx u16 LE][cy u16 LE][r u16 LE])。
    失败帧 = [0x04][00][00]（count=0，让单片机立刻解锁，不空烧等待超时）。

    ⚠️ 函数名必须与 _color.build_color_frame / _hostage.build_result_frame 区分：
    同名再经 tasks/__init__.py 重新导出会互相覆盖，单片机会收到错帧且不报错。

    r 是**外圆等效半径**（内边界拟合的半径已按 Ø55/Ø100 折算）—— 固件那条
    视差公式 offset = h·r_px/50 就是按外缘半径定义的。
    """
    data = struct.pack("<BBB", TARGET_TAG, 1 if ok else 0, len(targets))
    for t in targets:
        data += struct.pack("<BHHH", t["color"], _clamp_u16(t["cx"]),
                            _clamp_u16(t["cy"]), _clamp_u16(t["r"]))
    return data


def _open_with_retry(abort_event, attempts=5, delay=0.3):
    """打开主摄像头，失败时少量重试。

    Web 触发识别前会先断开主摄推流，但推流线程释放设备需要一小段时间，
    这里重试等待设备空闲（EBUSY）。串口直接调用时同样受益。
    """
    for i in range(attempts):
        if abort_event is not None and abort_event.is_set():
            return None
        cap = acquire_camera(is_qr=False)
        if cap is not None:
            return cap
        print(f"[WARN] 主摄像头打开失败 (第{i + 1}/{attempts}次)，稍后重试...")
        time.sleep(delay)
    return None


def _median(xs):
    n = len(xs)
    if n == 0:
        return 0.0
    if n % 2:
        return float(xs[n // 2])
    return (xs[n // 2 - 1] + xs[n // 2]) / 2.0


def _roi_of(cfg, img):
    """按 config 给出 ROI 相对坐标（没开就返回 None）。"""
    if not cfg["use_roi"]:
        return None
    x0, y0, x1, y1 = cfg["roi"]
    h, w = img.shape[:2]
    if (x1 - x0) * w < 16 or (y1 - y0) * h < 16:
        return None
    return (x0, y0, x1, y1)


def _crop(img, roi):
    """→ (子图, x0, y0)。roi 为 None 时原样返回。"""
    if roi is None:
        return img, 0, 0
    h, w = img.shape[:2]
    x0, y0 = max(0, int(roi[0] * w)), max(0, int(roi[1] * h))
    x1, y1 = min(w, int(roi[2] * w)), min(h, int(roi[3] * h))
    return img[y0:y1, x0:x1], x0, y0


def run_target_detect(timeout=None, abort_event=None):
    """主摄像头靶标识别 + 靶心定位（限时 timeout 秒，默认取 config.TARGET_TIMEOUT）。

    逐帧检测（_detect_frame）→ 按**颜色**分色做多帧投票：出现帧数 ≥ TARGET_MIN_VOTES
    才算该色存在，坐标取跨帧**中位数**（抗单帧误检/抖动）；同色 cx 跨帧散布超过
    TARGET_MAX_SPREAD_PX 说明机位在动或误检，该色作废。

    ⚠️ 坐标按**颜色**投票（不是按下标）—— 否则漏检一帧就会把左靶当成中靶。

    Returns dict: {"success","reason","frame","targets","aborted"(仅打断时)}；
        targets = [{"color","color_name","cx","cy","r","votes"}, ...]
        （cx/cy/r 为**全分辨率**像素）。失败时 targets 为「最好成绩」（仅 Web 显示用，
        单片机端收失败帧）。
    """
    from config import (TARGET_TIMEOUT, TARGET_PROC_SCALE, TARGET_SAT_MIN,
                        TARGET_VAL_MIN, TARGET_MIN_AREA, TARGET_TOL,
                        TARGET_VOTE_FRAMES, TARGET_MIN_VOTES, TARGET_MAX_SPREAD_PX,
                        TARGET_MIN_TARGETS, TARGET_MAX_TARGETS, TARGET_COLOR_NAMES,
                        TARGET_USE_ROI, TARGET_ROI_X0, TARGET_ROI_Y0,
                        TARGET_ROI_X1, TARGET_ROI_Y1, COLOR_HUE_FAMILIES,
                        TARGET_RING_Q_MIN, TARGET_RING_Q_MAX, TARGET_R_MAX)

    cfg = {"sat_min": TARGET_SAT_MIN, "val_min": TARGET_VAL_MIN,
           "min_area": TARGET_MIN_AREA, "tol": TARGET_TOL,
           "q_min": TARGET_RING_Q_MIN, "q_max": TARGET_RING_Q_MAX,
           "r_max": TARGET_R_MAX,
           "families": COLOR_HUE_FAMILIES,
           "use_roi": bool(TARGET_USE_ROI),
           "roi": (TARGET_ROI_X0, TARGET_ROI_Y0, TARGET_ROI_X1, TARGET_ROI_Y1)}

    timeout = TARGET_TIMEOUT if timeout is None else timeout
    cap = _open_with_retry(abort_event)
    if cap is None:
        if abort_event is not None and abort_event.is_set():
            print("[INFO] 靶标识别任务被打断")
            return {"success": False, "aborted": True, "reason": "",
                    "frame": None, "targets": []}
        return {"success": False, "reason": "主摄像头打开失败",
                "frame": None, "targets": []}

    try:
        print(f"[INFO] 开始靶标识别（三色尺寸一致性 + 内边界定位，每色≥{TARGET_MIN_VOTES}帧，"
              f"限时 {timeout:.0f}s）...")
        deadline = time.time() + timeout
        votes = {1: [], 2: [], 3: []}     # color → [(cx, cy, r), ...]
        frames_seen = 0
        best_frame = None
        t_start = time.time()

        while time.time() < deadline and frames_seen < TARGET_VOTE_FRAMES:
            if abort_event is not None and abort_event.is_set():
                print("[INFO] 靶标识别任务被打断")
                return {"success": False, "aborted": True, "reason": "",
                        "frame": None, "targets": []}

            frame = CameraManager.read(cap, "Main Camera", max_retries=3)
            if frame is None:
                time.sleep(0.05)
                continue

            roi = _roi_of(cfg, frame)
            sub, x0, y0 = _crop(frame, roi)
            try:
                found, _small = _detect_frame(sub, cfg, TARGET_PROC_SCALE)
            except Exception as e:                    # noqa: BLE001
                print(f"[WARN] 靶标单帧检测异常: {type(e).__name__}: {e}")
                found = []
            for t in found:
                # ROI 裁切的坐标要加回偏移，否则回读全乱
                votes[t["color"]].append((t["cx"] + x0, t["cy"] + y0, t["r"]))
            frames_seen += 1
            if found:
                best_frame = frame
            time.sleep(0.01)                          # 避免空转刷满 CPU

        # 实测帧率：config.py 的 TARGET_TIMEOUT / TARGET_VOTE_FRAMES 就是按它定的，
        # 调那两个数之前先看这一行（2026-09 提速前是 ~1s/帧，8 帧根本跑不完 6s 超时）。
        if frames_seen:
            elapsed = time.time() - t_start
            print(f"[INFO] 靶标采帧 {frames_seen} 帧，用时 {elapsed:.1f}s"
                  f"（{elapsed / frames_seen * 1000:.0f}ms/帧）")

        targets = []
        for c in (1, 2, 3):
            pts = votes[c]
            if len(pts) < TARGET_MIN_VOTES:
                continue
            xs = sorted(p[0] for p in pts)
            ys = sorted(p[1] for p in pts)
            rs = sorted(p[2] for p in pts)
            if xs[-1] - xs[0] > TARGET_MAX_SPREAD_PX:
                print(f"[WARN] 靶[{TARGET_COLOR_NAMES.get(c, c)}] 跨帧散布 "
                      f"{xs[-1] - xs[0]:.0f}px 超限（机位在动或误检），作废")
                continue
            targets.append({"color": c,
                            "color_name": TARGET_COLOR_NAMES.get(c, str(c)),
                            "cx": _median(xs), "cy": _median(ys), "r": _median(rs),
                            "votes": len(pts)})

        targets.sort(key=lambda t: t["cx"])

        ok = len(targets) >= TARGET_MIN_TARGETS
        if ok and len(targets) > TARGET_MAX_TARGETS:
            targets = targets[:TARGET_MAX_TARGETS]

        if ok:
            print("[INFO] 靶标识别成功: " + " ".join(
                f"{t['color_name']}({int(t['cx'])},{int(t['cy'])})r{int(t['r'])}"
                for t in targets))
            annotated = _annotate(best_frame, targets) if best_frame is not None else None
            return {"success": True, "reason": "", "frame": annotated, "targets": targets}

        reason = (f"靶标识别超时/不足 ({timeout:.0f}s)：找到 {len(targets)}/"
                  f"{TARGET_MIN_TARGETS} 颗")
        print(f"[WARN] {reason}")
        annotated = _annotate(best_frame, targets) if best_frame is not None else None
        return {"success": False, "reason": reason,
                "frame": annotated, "targets": targets}
    finally:
        release_camera(cap)     # 租约句柄 → 归还（保持打开）; 普通 cap → 真释放
