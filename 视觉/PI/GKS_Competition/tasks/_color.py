# tasks/_color.py — 色球顺序识别（排爆区，主摄像头 HSV，一次性）
#
# 指令 "5"：识别场地上一行三球（红/绿/蓝各一个，顺序抽签决定）的**颜色顺序**与
# 中心坐标，凑满三色即成功，按图像从左到右（中心 x 升序）排序。任务层只返回结果
# dict，串口回传由 handler.py::_cmd_color 统一发送（TYPE_COORD TAG=0x03 二进制帧，
# 见 docs/串口协议.md §6d），保持任务函数纯净可测（与 _hostage.py 同构）。
#
# 判定分两步（见 _detect_frame）: **只看饱和度找球 → 球内比哪一族色相像素最多**。
# 这是相对判定, 所以色相族可以放宽到对色相环的无空隙划分, 见 config.COLOR_HUE_FAMILIES。
# 为什么不用 YOLO：颜色是像素属性，这套在 Pi 上比 YOLOv8n(1~5fps) 快一个数量级；
# YOLO 留给形状任务（人质三分类）。
#
# ⚠️ 三色各一个（比赛规则），故用「逐色取最大连通域」——哪块是哪色天然无歧义。
#    若日后规则改成允许同色重复，要改成「先连通域聚类成球，再逐球判色」。
#
# ---- 无色轮廓补救（2026-10-26）：轮廓在、颜色丢 → 排除法/概率猜色 ----
# _detect_frame 过了面积/圆度筛但「球内获胜族像素不足」的轮廓, 旧版连实测坐标一起丢;
# 现在坐标单独跨帧投票, 收尾时与缺失色配对:
#   · 缺 1 色 → 排除法: 三色必缺这一色, 零歧义, 坐标是实测的（比缺球推断外推更准）;
#   · 缺 ≥2 色 → 按球内三族色相像素占比全局指派（概率猜色）; 最优与次优分差小于
#     COLOR_GUESS_MARGIN 时日志标「赌」但**照给**——"哪怕决定不了最后猜一个"
#     （2026-10-26 用户定: 不抓=0分, 赌错也比空跑强）。
# 与缺球推断（A1/A2/B, 见下节）的分工: 本节管「轮廓在、颜色丢」（坐标实测）;
# A 层管「轮廓没有」（坐标靠板子坐标系/两球外推）。两节接力, 补不齐 A 层接着推。
# ⚠️ 多帧投票要求**小车静止**：所有票必须来自同一机位，车在动会让同一颜色跨帧散布
#    过大（COLOR_MAX_SPREAD_PX 会作废该色）。单片机必须在底盘停稳后才发 "5"。
import math
import struct
import time

import cv2
import numpy as np

from camera import CameraManager
from camera_lease import acquire_camera, release_camera

COLOR_TAG = 0x03   # 结果帧子命令标记（TYPE_COORD DATA 首字节，<0x20 与 ASCII 帧不冲突）

# 颜色码 → 画面标注用英文短名（cv2.putText 画不了中文）
_COLOR_EN = {1: "R", 2: "G", 3: "B"}
# 文字用球自己的颜色（BGR 顺序！color3=蓝 → (255,0,0)）
_COLOR_BGR = {1: (0, 0, 255), 2: (0, 255, 0), 3: (255, 0, 0)}
# ⚠️ 框选/十字**不能**用球自己的颜色 —— 蓝框画在蓝球上完全看不见（红框/绿框同理，
#    只是没蓝那么彻底）。统一用黄色：对红/绿/蓝球和木纹/浅灰地面都高对比。
_BOX_BGR = (0, 255, 255)


def _clamp_u16(v):
    """坐标落进 u16（struct.pack('<H') 越界会抛异常）。"""
    return max(0, min(65535, int(round(v))))


def _detect_frame(img, scale=1.0):
    """单帧检测 → (balls, small, colorless)。balls = [{"color","cx","cy","r","area","circ","hits"}, ...]
    colorless = [{"cx","cy","r","area","hits"}, ...] —— 过了几何筛但定色不足的轮廓
    （2026-10-26 加; 坐标是实测的, 颜色由收尾的排除法/概率猜色补, 见文件头）。

    **先找球、再定色**（判定方式移植自 pro 项目
    cv2_python/zongchengxu20250724chusai.py::detect_stacking_and_color）:
      1) 只看「饱和且够亮」找候选区域, **完全不看色相**（COLOR_SAT_MIN/VAL_MIN）
         —— 用三色掩膜找球会把球在桌面上的**彩色反光**也并进来, 轮廓碎成非圆,
         实测绿球圆度只有 0.29 直接被剔掉; 只看饱和度, 球是紧密整体（0.74）
      2) 轮廓按面积/圆度筛 → 候选球（色相漂移不影响能不能找到球）
      3) 球内逐族数色相像素 → **最多的那一族 = 这个球的颜色**
      4) 每色只留最大的一颗（比赛规则: 红/绿/蓝各一个），按 cx 升序返回

    这是**相对判定**而非绝对阈值: 不要求「必须落进红色区间才算红」，只要求
    「三个族里红的最多」。所以色相族可以放宽到**对色相环的无空隙划分**，
    光照/白平衡把色相推走了也只会影响判成哪一族，不会整颗球消失。

    scale<1 时先降采样（1920×1080 下提速数倍），**返回的 cx/cy/r 已按 1/scale 还原回
    全分辨率 1920×1080 像素** —— 协议与单片机端 COLOR_ALIGN_CX / COLOR_PX_TO_MM_X
    全按全分辨率定义，混用会让标定全废。

    ⚠️ 坐标空间不统一，别混用: cx/cy/r = **全分辨率**(给协议与画框用);
       area/circ/hits = **降采样后**坐标系下的值(阈值 COLOR_MIN_AREA / COLOR_MIN_COLOR_PIXELS 按此定标)。
    """
    from config import (COLOR_HUE_FAMILIES, COLOR_SAT_MIN, COLOR_VAL_MIN,
                        COLOR_MIN_AREA, COLOR_MAX_AREA, COLOR_MIN_CIRCULARITY,
                        COLOR_MIN_COLOR_PIXELS)

    if scale != 1.0:
        small = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    else:
        small = img
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    hu = hsv[:, :, 0].astype(np.int16)
    sa = hsv[:, :, 1].astype(np.int16)
    va = hsv[:, :, 2].astype(np.int16)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))

    # 1) 找球: 只看「饱和且够亮」, 完全不看色相
    #    （用三色掩膜求并集找球会把球周围的彩色反光并进来, 轮廓碎成非圆 → 被圆度剔掉）
    smask = (((sa >= COLOR_SAT_MIN) & (va >= COLOR_VAL_MIN))
             .astype(np.uint8) * 255)
    smask = cv2.morphologyEx(smask, cv2.MORPH_OPEN, k)    # 去零散噪点
    smask = cv2.morphologyEx(smask, cv2.MORPH_CLOSE, k)   # 补球面高光打出的洞

    contours = cv2.findContours(smask, cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)[-2]   # [-2] 兼容 OpenCV 3/4
    balls = []
    colorless = []
    for c in contours:            # ⚠️ 不能写 max(contours,...)：空列表会抛 ValueError
        area = cv2.contourArea(c)
        if area < COLOR_MIN_AREA or area > COLOR_MAX_AREA:
            continue
        peri = cv2.arcLength(c, True)
        if peri <= 0.0:
            continue
        circ = 4.0 * math.pi * area / (peri * peri)   # 1.0 = 正圆
        if circ < COLOR_MIN_CIRCULARITY:
            continue

        # 2) 定色: 球轮廓内逐族数色相像素 → 最多的那一族胜出（相对判定, 见函数头注释）
        cm = np.zeros(smask.shape, np.uint8)
        cv2.drawContours(cm, [c], -1, 255, cv2.FILLED)
        inside = (cm > 0)
        hits = {}
        for col, fams in COLOR_HUE_FAMILIES.items():
            in_fam = np.logical_or.reduce([(hu >= lo) & (hu <= hi)
                                           for lo, hi in fams])
            hits[col] = int(np.count_nonzero(in_fam & inside))
        color = max(hits, key=lambda c2: hits[c2])    # dict 恒有 3 项, 不会空
        if hits[color] < COLOR_MIN_COLOR_PIXELS:
            # 哪一族都太少: 定色不成立。但轮廓已过面积/圆度筛 —— 几何上就是颗球,
            # 坐标留进无色候选（颜色交给收尾的排除法/概率猜色, 见文件头「无色轮廓
            # 补救」）, 不再像旧版连实测坐标一起丢（2026-10-26, "宁可赌也不空跑"）。
            (cxu, cyu), ru = cv2.minEnclosingCircle(c)
            colorless.append({"cx": cxu / scale, "cy": cyu / scale, "r": ru / scale,
                              "area": area, "hits": hits})
            continue

        (cx, cy), r = cv2.minEnclosingCircle(c)
        balls.append({"color": color,
                      "cx": cx / scale, "cy": cy / scale, "r": r / scale,
                      "area": area, "circ": circ, "hits": hits})

    # 每色只留最大的一颗（比赛规则: 红/绿/蓝各一个）。
    # ⚠️ 必须在这里去重: 同一帧里某色若出现两个连通域（真球 + 远处噪点/反光），
    #    两块都会被记进 votes[该色]；跨帧算中位数前要查散布 COLOR_MAX_SPREAD_PX，
    #    两者 cx 相差几百像素 → **把整颗球作废**。实测踩过: 蓝球 cx=923 被一个
    #    cx=1625、仅 94 像素的噪点连坐，蓝球整帧丢失。
    best = {}
    for b in balls:
        if b["color"] not in best or b["area"] > best[b["color"]]["area"]:
            best[b["color"]] = b
    return sorted(best.values(), key=lambda b: b["cx"]), small, colorless


def _annotate(frame, targets, extra=""):
    """框选每个球 + 标从左到右序号.颜色 + 中心像素，供 Web 预览/现场核对。

    框的大小来自检测到的实际半径 r（全分辨率像素），不是固定值——
    这样「框住没框住」一眼就能看出阈值和圆度筛得对不对。
    targets 里的 r 可缺（标定工具等外部调用），缺时退化为不画框只画十字。
    """
    out = frame.copy()
    for rank, t in enumerate(targets, 1):
        cx, cy = int(t["cx"]), int(t["cy"])
        bgr = _COLOR_BGR.get(t["color"], (255, 0, 255))   # 文字用球色
        r = t.get("r")
        if r:
            ri = max(6, int(r))
            cv2.rectangle(out, (cx - ri, cy - ri), (cx + ri, cy + ri), _BOX_BGR, 4)
        else:
            cv2.circle(out, (cx, cy), 10, _BOX_BGR, 3)
        cv2.drawMarker(out, (cx, cy), _BOX_BGR, cv2.MARKER_CROSS, 28, 3)
        label = f"{rank}.{_COLOR_EN.get(t['color'], '?')} ({cx},{cy})"
        off = (max(6, int(r)) + 10) if r else 24
        org = (max(0, cx - 110), max(30, cy - off))
        # 文字先描黑边再上球色：球色文字压在球上时（尤其蓝字压蓝球）才看得清
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 1.3, (0, 0, 0), 7)
        cv2.putText(out, label, org, cv2.FONT_HERSHEY_SIMPLEX, 1.3, bgr, 3)
    if extra:
        cv2.putText(out, extra, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.2,
                    (255, 255, 255), 3)
    return out


def build_color_frame(ok, targets, yaw_deg=None, fit_ok=False, pull_dir=0):
    """打包回传帧 DATA（见 docs/串口协议.md §6d）：
    [TAG=0x03][status u8][count u8] + count×([color u8][cx u16 LE][cy u16 LE])。
    失败帧 = [0x03][00][00][pull_dir u8]（count=0，让单片机立刻解锁，不空烧等待超时）。

    小球对位扩展（2026-09）: 成功帧末尾追加 [yaw i16 LE 0.01°][fit_ok u8] →
    LEN = 21（yaw/fit_ok 来自 fit_line_yaw, 见其 docstring）。yaw_deg 传 None
    （失败帧/旧调用方）则不追加, 单片机按 LEN<21 兜底为 0/合格, 双向向后兼容。

    失败帧方向字节（2026-10-26, 单片机 ALIGN_PULLBACK 用）: LEN=4, pull_dir
    0=无信息 1=缺的球从画面左出界(内容整体左移=车沿+cx方向过冲) 2=右出界。
    旧固件对第 4 字节无视（失败帧仍按 count=0 解锁）, 向后兼容; 成功帧不带。

    ⚠️ 函数名必须与 _hostage.build_result_frame 区分：两者同名再经
    tasks/__init__.py 重新导出会互相覆盖，单片机会收到错帧且不报错。
    """
    data = struct.pack("<BBB", COLOR_TAG, 1 if ok else 0, len(targets))
    for t in targets:
        data += struct.pack("<BHH", t["color"],
                            _clamp_u16(t["cx"]), _clamp_u16(t["cy"]))
    if not ok:
        data += struct.pack("<B", int(pull_dir) & 3)
    if yaw_deg is not None:
        y01 = max(-32768, min(32767, int(round(yaw_deg * 100.0))))
        data += struct.pack("<hB", y01, 1 if fit_ok else 0)
    return data


def _open_color_with_retry(abort_event, attempts=5, delay=0.3):
    """打开主摄像头，失败时少量重试。

    Web 触发识别前会先断开主摄推流，但推流线程释放设备需要一小段时间，
    这里重试等待设备空闲（EBUSY）。串口直接调用时同样受益。
    """
    for i in range(attempts):
        if abort_event is not None and abort_event.is_set():
            return None
        cap = acquire_camera(is_qr=False)   # 主摄像头（命中租约则复用预热好的）
        if cap is not None:
            return cap
        print(f"[WARN] 主摄像头打开失败 (第{i + 1}/{attempts}次)，稍后重试...")
        time.sleep(delay)
    return None


def _median(xs):
    """已排序列表的中位数（偶数个取中间两个的均值）。"""
    n = len(xs)
    if n == 0:
        return 0.0
    if n % 2:
        return float(xs[n // 2])
    return (xs[n // 2 - 1] + xs[n // 2]) / 2.0


def fit_line_yaw(targets):
    """三球质心 fitLine → (yaw_deg, fit_ok)（小球对位扩展, 2026-09）。

    yaw = 三球**中位数坐标**拟合直线的倾角（图像系 y 向下, 左高右低 = 左球 cy 小于
    右球 cy 为正）。fitLine 返回的方向向量正负号不稳定, 必须先归一到 +x（从左到右）,
    否则同一姿态两次调用的符号会随机翻转 180°。
    ⚠️ 这是**图像角**, 不是车体航向角: 车原地转 θ, 等距球列在图像里只斜约 (h/Z)·θ
    （相机俯视几何, 实测标定该比例, 预期 ~0.3 即车转 3° 图像只斜 ~1°）——
    单片机端拿它换算车体角, 别当 1:1 用。
    fit_ok: 各点到拟合直线的最大距离 < config.COLOR_FIT_MAX_DIST_PX 才算三点
    共线合格 —— 某球质心被反光/遮挡拉偏时残差会明显变大, 单片机可据此要求重扫。
    三球不齐（len != 3）直接判 (0, False), 不做两点拟合。
    """
    from config import COLOR_FIT_MAX_DIST_PX

    if len(targets) != 3:
        return 0.0, False
    pts = np.float32([[t["cx"], t["cy"]] for t in targets])
    # fitLine 返回 4×1 数组, reshape(-1) 拉平后逐个 float()（直接 float() 形状 (1,)
    # 的元素在新版 numpy 会 TypeError, 本地实测踩过）
    vx, vy, x0, y0 = (float(v) for v in
                      cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).reshape(-1))
    if vx < 0:                      # 方向归一到从左到右（+x）, 否则符号随机翻转
        vx, vy = -vx, -vy
    yaw = math.degrees(math.atan2(vy, vx))
    maxd = 0.0
    for t in targets:               # 残差 = 点到直线的距离（叉积 / |方向向量|）
        d = abs((t["cx"] - x0) * vy - (t["cy"] - y0) * vx)
        maxd = max(maxd, d)
    return yaw, (maxd < COLOR_FIT_MAX_DIST_PX)


# ==================== 无色轮廓补救（2026-10-26）：轮廓在、颜色丢 ====================
#
# 与下面「缺球推断」节的分工（见文件头详注）: 本节坐标是**实测的**（轮廓过了几何筛）,
# 只补颜色; 缺球推断是轮廓都没有时的**坐标外推**。收尾顺序: 先本节、后缺球推断 ——
# 本节补不齐（还缺坐标）时 targets 可能正好剩 2 颗, 缺球推断接着推第三颗的坐标。


def _cluster_unc_votes(votes, min_gap_px, max_spread_px, min_votes):
    """跨帧无色候选票 → 按 cx 聚成"球簇"（与有色票的散布闸同一条纪律）。

    贪心: cx 升序逐票, 与当前簇**最新票** cx 差 < max_spread_px 归同簇 ——
    同球票的散布闸就是 max_spread_px（≤它必是同一颗）, 而真球间距实测 ~500px
    远大于它, 分界无需再引入别的数。簇内 cx/cy 极差超限说明混进了别的球/噪点
    → 整簇作废（宁缺勿假, 与有色票同口径）。
    纯函数（不碰图像/串口）, 离线可单独验。
    返回 [{"cx","cy","r","votes","hits"(成员求和)}, ...] 按 cx 升序。
    """
    vs = sorted(votes, key=lambda v: v["cx"])
    groups = []
    for v in vs:
        if groups and (v["cx"] - groups[-1]["last_cx"]) < max_spread_px:
            groups[-1]["members"].append(v)
            groups[-1]["last_cx"] = v["cx"]
        else:
            groups.append({"members": [v], "last_cx": v["cx"]})
    out = []
    for g in groups:
        ms = g["members"]
        if len(ms) < min_votes:
            continue
        xs = sorted(m["cx"] for m in ms)
        ys = sorted(m["cy"] for m in ms)
        if (xs[-1] - xs[0]) > max_spread_px or (ys[-1] - ys[0]) > max_spread_px:
            continue
        hits = {1: 0, 2: 0, 3: 0}
        for m in ms:
            for k, n in (m.get("hits") or {}).items():
                ki = int(k)
                hits[ki] = hits.get(ki, 0) + int(n)
        out.append({"cx": _median(xs), "cy": _median(ys),
                    "r": _median(sorted(m["r"] for m in ms)),
                    "votes": len(ms), "hits": hits})
    return out


def _assign_unc_colors(clusters, missing, guess_margin):
    """无色簇 × 缺失色 → 指派（每簇一色）。返回 [(cluster, color, note), ...]。

    · 缺 1 色 = **排除法**: 三色各一是赛题前提, 剩那颗必是缺色 —— 零歧义,
      这是「知道一色且够用」之外最稳的一层（坐标实测 + 颜色演绎）。
    · 缺 ≥2 色 = **概率猜色**: 全排列各算总分 score = Σ(簇内该色 hits/簇内三色
      hits 总和)（比例制, 簇间像素数不均时公平）, 最优赢。
      簇少于缺色时枚举 permutations(missing, len(clusters)) —— 每簇一色、色不
      重复的**子集指派**, 剩下的缺色留给缺球推断去补坐标（两节接力）。
      最优与次优分差 < guess_margin → 歧义: **照给最优**（"哪怕决定不了最后猜
      一个", 2026-10-26 用户定: 不抓=0分, 赌错也比空跑强）, note 标「赌」。
      排列枚举顺序 = 缺色升序（先验序）, 全平分时稳定排序自然取先验序 ——
      连"赌"都是确定性的, 同一输入永远同一输出, 方便复盘。
    hits 全 0 的簇（色相彻底崩掉）分数恒 0, 落进歧义分支按先验序给。
    纯函数, 离线可单独验。
    """
    from itertools import permutations

    if not clusters or not missing:
        return []
    if len(missing) == 1:
        # 排除法只对第一簇成立（调用方已把簇数截到 ≤ len(missing)）
        return [(clusters[0], missing[0], "排除法: 三色必缺这一色, 零歧义")]

    def _score(assign):
        s = 0.0
        for cl, col in zip(clusters, assign):
            tot = sum(cl["hits"].values())
            if tot > 0:
                s += cl["hits"].get(col, 0) / tot
        return s

    cands = sorted(permutations(missing, len(clusters)), key=_score, reverse=True)
    sb = _score(cands[0])
    s2 = _score(cands[1]) if len(cands) > 1 else -1.0
    if (sb - s2) < guess_margin:
        note = "概率猜色(赌): 分差%.2f不足, 按先验/最优给" % (sb - s2)
    else:
        note = "概率猜色: 最优分差%.2f" % (sb - s2)
    return [(cl, col, note) for cl, col in zip(clusters, cands[0])]


def _pull_dir(targets, cx_center, tol_px, mid_gap_px):
    """识别失败时（补救/推断都没成, 剩实测两球）→ 缺的球从画面哪侧出界。

    ⚠️ 先过**间距门**（2026-10-28 修, 现场真踩过: 过曝白球缺中间, 中点偏左 173px
       被误判"缺左端出画"、给错拉回方向——真拉就朝错方向走 180mm）:
       两球间距 ≥ mid_gap_px → 缺的是**中间**（识别问题, 在画面内）→ 返回 0
       **绝不给方向**（车是正的或只偏一点, 盲拉反而把端球拉出画）。
    间距正常（缺端部）才用中点偏移判出界侧: 中点偏左 = 内容左移 = 车朝 +cx 过冲
    = 缺左端球、左出界 → 1; 反之 2。返回 0 = 无信息/缺中间/信号不明。
    纯函数, 离线可单独验。单片机 ALIGN_PULLBACK 读失败帧的这个方向开环倒回。
    """
    if len(targets) != 2:
        return 0
    (x1, _y1), (x2, _y2) = sorted((t["cx"], t["cy"]) for t in targets)
    if (x2 - x1) >= mid_gap_px:
        return 0          # 缺中间: 识别问题不在画面外, 不拉
    mid = (x1 + x2) / 2.0
    if mid < cx_center - tol_px:
        return 1      # 中点偏左 = 内容左移 = 缺左端球, 从画面左侧出界
    if mid > cx_center + tol_px:
        return 2      # 中点偏右 = 内容右移 = 缺右端球, 从画面右侧出界
    return 0


# ==================== 缺球推断（2026-09）：只认出两颗时把第三颗推回来 ====================
#
# 赛题必定给三颗球（红/绿/蓝各一），所以「只认出两颗」基本是**识别问题**而不是缺球。
# 用已检出的两颗 + 板子坐标系把缺的那颗的**颜色与位置**推回来 —— 下游（单片机对位/抓取）
# 一行都不用改：它拿到的仍是一个正常的三色结果（刻意**不打推断标记**）。
#
# 为什么是"用板子当坐标系"而不是"把矩形等分三份"：三球坐在板子这个平面上，
# 平面 ↔ 像素之间差一个单应变换；用板子四角标定出来之后，球位在**板归一化坐标 (u,v)**
# 里是固定的、与车停在哪/相机怎么歪都无关。实测本机位 u = 0.159 / 0.500 / 0.842
# （等距 s=0.341、对称于 u=0.5、v≈0.375 平行于板子长边）——**不是 1/6、1/2、5/6**，
# 等分会差约 1% 板宽（≈12px）。
#
# 三层判据逐级降级，**任何一层自洽性不过就判失败（不推）**，绝不瞎给：
#   A1 球位表  : 最近槽匹配（标定一次；同时给 A2 当间距先验）
#   A2 等距自洽: 三种占位假设各算两遍（中线先验 / 间距外推）→ 四条判据全过才算数
#   B  两球外推: 板子看不到时用两球间距 + 方向；缺中球取中点（稳），缺端球靠弱先验
# 实测精度（用 ref_color_frame 真值 + ±0.01u 噪声试算）：**缺中球（=对位基准）误差 0~9px
# ≈0~3mm**；缺端球 1~13px（端球位只影响"抓哪颗/移到哪"）；三类对抗用例都被判据挡下。


def _infer_slot_u(u1, u2, c, s_prior, tol_s, tol_center, u_min, u_max, agree_tol):
    """A2 等距自洽：两个已检球的板坐标 u（u1<u2）→ 缺的那个槽的 u。

    模型：三个槽在板坐标里等距（间距 s），且球行横跨板子中线（中球位 = c ≈ 0.5）。
    对"这两个点占哪两个槽"的三种假设，各自用**两条独立路线**推缺的那个槽：
      路线① 中线先验（缺端球时用 c±s，缺中球时就是 c）
      路线② 间距外推（用观测到的间距往外延）
    四条判据全过才算数，且两条路线必须互相同意 —— 对不上就返回 None（不推）。

    纯函数（不碰图像/串口），离线可单独验。
    返回 (u, 说明) 或 (None, 拒绝原因)。
    """
    cands = []
    for name, mid_obs, s, u_a, u_b in (
            ("相邻偏左", u2,            c - u1,        2 * c - u1,  2 * u2 - u1),
            ("相邻偏右", u1,            u2 - c,        2 * c - u2,  2 * u1 - u2),
            ("两端",    (u1 + u2) / 2.0, (u2 - u1) / 2.0, c,          (u1 + u2) / 2.0)):
        res_center = abs(mid_obs - c)
        if s <= 0.0:
            continue
        ok = (abs(s - s_prior) <= tol_s                       # ① 与标定间距相符
              and res_center <= tol_center                     # ② 中线残差
              and u_min <= u_a <= u_max and u_min <= u_b <= u_max   # ③ 落在板内不贴边
              and abs(u_a - u_b) <= agree_tol)                 # ④ 两条路线互相同意
        if ok:
            cands.append((abs(u_a - u_b) + res_center, name, s, (u_a + u_b) / 2.0))
    if not cands:
        return None, "三种占位假设都不自洽（间距/中线/板内/两路一致 至少一条不过）"
    cands.sort()
    if len(cands) > 1 and (cands[1][0] - cands[0][0]) < 0.01:
        return None, "两种占位假设几乎同样自洽 → 有歧义"
    _, name, s, u_miss = cands[0]
    return u_miss, "占位=%s, 间距=%.3f" % (name, s)


def _infer_by_board(frame, targets, cfg):
    """A（主）：用板子坐标系推断缺的那颗。返回 (cx, cy, 说明) 或 (None, None, 原因)。"""
    # 板子识别（同一帧、同一只主摄）。⚠️ 用**包限定名**：任务模块是 tasks 包的一部分，
    # 裸 `import _rect` 在树莓派上会 ImportError（sys.path 里是仓库根、不是 tasks/）。
    from tasks._rect import detect_rect, _params

    r = detect_rect(frame, _params())
    if not r.get("success"):
        return None, None, "板子识别失败(%s)" % r.get("reason", "")
    src = np.float32(r["corners"])                  # 左上,右上,右下,左下
    dst = np.float32([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]])
    H = cv2.getPerspectiveTransform(src, dst)       # 图像 → 板归一化坐标

    # A1: 先按标定表最近槽匹配（最准，实测残差 ~1px）
    tbl = cfg["slot_u"]
    uv = []
    for t in targets:
        p = H @ np.array([t["cx"], t["cy"], 1.0])
        uv.append((p[0] / p[2], p[1] / p[2]))
    hit = [min(range(3), key=lambda k: abs(u - tbl[k])) for u, _ in uv]
    res = max(abs(uv[i][0] - tbl[hit[i]]) for i in range(len(uv)))
    if len(set(hit)) == 2 and res <= cfg["table_tol"]:
        miss = ({0, 1, 2} - set(hit)).pop()
        p = np.linalg.inv(H) @ np.array([tbl[miss], (uv[0][1] + uv[1][1]) / 2.0, 1.0])
        return (float(p[0] / p[2]), float(p[1] / p[2]),
                "A1 球位表: 缺槽%d(u=%.3f) 槽残差%.3f" % (miss, tbl[miss], res))

    # A2: 表对不上 → 等距自洽（用观测到的 u 自己解）
    (u1, v1), (u2, v2) = sorted(uv, key=lambda x: x[0])
    u_miss, why = _infer_slot_u(u1, u2, cfg["u_center"], cfg["s"],
                                cfg["s_tol"], cfg["center_tol"],
                                cfg["u_min"], cfg["u_max"], cfg["agree_tol"])
    if u_miss is None:
        return None, None, "A2 %s（A1 表残差 %.3f 也超限）" % (why, res)
    p = np.linalg.inv(H) @ np.array([u_miss, (v1 + v2) / 2.0, 1.0])
    return (float(p[0] / p[2]), float(p[1] / p[2]), "A2 %s（A1 表残差 %.3f，故走自洽解）"
            % (why, res))


def _infer_by_two_balls(targets, cfg, frame_w=None):
    """B（兜底）：板子看不到时，用两球的图像间距 + 方向外推。

    缺中球（两点是两端）取中点最稳；缺端球只能靠"中球位≈理想列 COLOR_BALL_CX_CENTER"
    这个弱先验消歧（车停得偏就会判错，所以自洽性不过一律不推）。
    返回 (cx, cy, 说明) 或 (None, None, 原因)。

    ⚠️⚠️ 光有"中球位≈理想列"这一条判据不够 —— 它会放出一个**画面外**的坐标。
    反例（实测算式）: 只认出左右两颗、车停偏成 x1=51 / x2=986 →
      · "相邻偏左" 假设 mid=x2=986, |986-926|=60 ≤ 60 过关 → x_miss = 2·986-51 = **1921**（出画面）
      · 真正对的"两端"假设 mid=519 因 |519-926|=407 被否
    于是返回一个画面外的"第三颗球", 而它**不带任何标记**地被当成正常三色结果回传
    （单片机 ucBallScanOnce 认 success, 且中球=对位基准就是这个假坐标 → 横移/抓取全错）。
    所以这里补两道（frame_w 给了才判）:
      ① 观测间距不能大过画面宽（两颗球不可能比画面还远）;
      ② 推出来的第三颗必须**落在画面内**。
    """
    (c1, x1, y1), (c2, x2, y2) = sorted(((t["color"], t["cx"], t["cy"]) for t in targets),
                                        key=lambda v: v[1])
    gap = x2 - x1
    if gap <= 1.0:
        return None, None, "B 两球 cx 几乎重合"
    if frame_w and gap >= frame_w:
        return None, None, "B 观测间距 %.0fpx ≥ 画面宽 %.0fpx（这两颗不可能是同一排）" % (
            gap, frame_w)
    cands = []
    for name, mid, x_miss in (("相邻偏左", x2, 2 * x2 - x1),
                              ("相邻偏右", x1, 2 * x1 - x2),
                              ("两端", (x1 + x2) / 2.0, (x1 + x2) / 2.0)):
        if abs(mid - cfg["cx_center"]) > cfg["cx_tol_px"]:
            continue
        if frame_w and not (0.0 <= x_miss < frame_w):
            print(f"[WARN] 缺球推断: 假设「{name}」推出的第三颗 x={x_miss:.0f} 在画面外"
                  f"（宽 {frame_w:.0f}），丢弃该假设")
            continue
        cands.append((abs(mid - cfg["cx_center"]), name, x_miss))
    if not cands:
        return None, None, "B 三种占位假设都不自洽（都在理想列 %.0f±%.0f 之外）" % (
            cfg["cx_center"], cfg["cx_tol_px"])
    cands.sort()
    if len(cands) > 1 and (cands[1][0] - cands[0][0]) < 20.0:
        return None, None, "B 两种占位假设几乎同样自洽 → 有歧义"
    _, name, x_miss = cands[0]
    return float(x_miss), float((y1 + y2) / 2.0), "B 两球外推: 占位=%s, 间距=%.0fpx" % (name, gap)


def _infer_missing(frame, targets, names):
    """只检到两颗 → 推第三颗（颜色=三色里缺的那个）。

    返回 (新 targets, 说明) 或 (原 targets, None)。推不出来时**不推**（由调用方判失败）。
    ⚠️ 刻意不返回任何"推断标记"：赛题必给三颗，只认出两颗基本是识别问题，下游照常当三色用。
    """
    from config import (COLOR_BALL_U_CENTER, COLOR_BALL_S, COLOR_BALL_S_TOL,
                        COLOR_BALL_CENTER_TOL, COLOR_BALL_U_MIN, COLOR_BALL_U_MAX,
                        COLOR_BALL_AGREE_TOL, COLOR_BALL_CX_CENTER,
                        COLOR_BALL_CX_TOL_PX, COLOR_INFER_MID_GAP_PX)
    cfg = dict(u_center=COLOR_BALL_U_CENTER, s=COLOR_BALL_S, s_tol=COLOR_BALL_S_TOL,
               center_tol=COLOR_BALL_CENTER_TOL, u_min=COLOR_BALL_U_MIN,
               u_max=COLOR_BALL_U_MAX, agree_tol=COLOR_BALL_AGREE_TOL,
               cx_center=COLOR_BALL_CX_CENTER, cx_tol_px=COLOR_BALL_CX_TOL_PX,
               slot_u=[COLOR_BALL_U_CENTER - COLOR_BALL_S, COLOR_BALL_U_CENTER,
                       COLOR_BALL_U_CENTER + COLOR_BALL_S],
               table_tol=COLOR_BALL_S_TOL / 2.0)
    have = {t["color"] for t in targets}
    miss_color = [c for c in (1, 2, 3) if c not in have]
    if len(have) != 2 or len(miss_color) != 1:
        return targets, None
    cx = cy = None
    note = ""
    if frame is not None:
        try:
            cx, cy, note = _infer_by_board(frame, targets, cfg)
        except Exception as e:                      # 推断出问题绝不能把整条任务带崩
            print(f"[WARN] 缺球推断(板子坐标系)异常: {type(e).__name__}: {e}")
            cx = None
    if cx is None:
        # 间距判据（2026-10-28 用户定, 现场日志实证: 两球间距 1124px vs 球距 554）:
        # 两球间距 ≥ COLOR_INFER_MID_GAP_PX → 这两颗必是**两端**, 缺的是中间 ——
        # 间距对车平移**不变**, 是 A1/A2(板系先验)/B(理想列先验)全被车偏打穿时
        # 唯一仍成立的几何约束（车偏 ~173px 时那三层全挂, 真踩过 2026-10-28）。
        # 颜色=排除法零歧义; 坐标=连线中点(cx/cy 都插值)——色球排几乎严格等距
        # (实测 554/557), 中点误差小, 且插值点本身承载当前车偏量, 对位拿它算
        # (cx_center − 中点) 正好把车修回去。不依赖缺球那颗的轮廓——它可能连
        # 饱和度掩膜都没过（过曝白球, 2026-10-28 color_0004 实拍）, 谁都救不了它。
        gap = float(targets[1]["cx"]) - float(targets[0]["cx"])
        if gap >= COLOR_INFER_MID_GAP_PX:
            mid_x = (targets[0]["cx"] + targets[1]["cx"]) / 2.0
            mid_y = (targets[0]["cy"] + targets[1]["cy"]) / 2.0
            c = miss_color[0]
            out = sorted(targets + [{"color": c,
                                     "color_name": names.get(c, str(c)),
                                     "cx": mid_x, "cy": mid_y, "r": 0.0,
                                     "votes": 0}], key=lambda t: t["cx"])
            return out, ("间距判据: 两球间距%.0fpx ≥ %d → 缺中间, 中点插值补上"
                         % (gap, COLOR_INFER_MID_GAP_PX))
        # frame_w: 给 B 层做「推出来的位置必须在画面内」那道检查（拿不到帧就跳过）
        cx2, cy2, why = _infer_by_two_balls(targets, cfg,
                                            frame_w=(float(frame.shape[1]) if frame is not None else None))
        if cx2 is None:
            print(f"[WARN] 缺球推断失败: {note or why}")
            return targets, None
        cx, cy, note = cx2, cy2, why
    c = miss_color[0]
    out = sorted(targets + [{"color": c, "color_name": names.get(c, str(c)),
                             "cx": cx, "cy": cy, "r": 0.0,
                             "votes": 0}], key=lambda t: t["cx"])
    return out, note


def run_color_detect(timeout=None, abort_event=None):
    """主摄像头色球识别（限时 timeout 秒，默认取 config.COLOR_TIMEOUT）。

    逐帧检测（先找球再定色，见 _detect_frame）→ 对每种颜色做**多帧投票**：出现帧数 ≥
    COLOR_MIN_VOTES 才算该色存在（众数语义），坐标取跨帧**中位数**（抗单帧误检/抖动）；
    同色 cx 跨帧散布超过 COLOR_MAX_SPREAD_PX 说明车在动或误检，该色作废。
    凑满三色、且两两 cx 间隔 ≥ COLOR_MIN_GAP_PX（确认是三个球而非一个球裂成两块）
    即成功，按 cx 升序排序。

    坐标按**颜色**分别投票（不是按下标！）——否则漏检一帧就会把左球当成中间球。

    Returns dict: {"success", "reason", "frame", "targets", "aborted"(仅打断时),
                   "frame_raw"(仅成功时: 未标注的原始帧)}；
        targets = [{"color","color_name","cx","cy","votes"}, ...]（cx/cy 为全分辨率像素）
        失败时 targets 为「最好成绩」（仅 Web 显示用，单片机端收失败帧）；
        失败/打断时 frame 尽力给「最后的原始帧」（统一记录接口, 失败也留档）。
    """
    # ⚠️⚠️ 这张 import 表就是本函数的**全部** config 名字。失败出口那句
    #    `_pull_dir(..., COLOR_INFER_MID_GAP_PX)` 也在这张表里取 —— 2026-10-30 修过一次
    #    漏掉它：**识别一失败就 NameError**（未捕获异常 → 串口 I/O error → 重连 5~8s
    #    → 整步白等十几秒），成功路径却一点事没有，日志里只看到"色球变慢"。
    from config import (COLOR_TIMEOUT, COLOR_VOTE_FRAMES, COLOR_MIN_VOTES,
                        COLOR_MAX_SPREAD_PX, COLOR_MIN_GAP_PX, COLOR_PROC_SCALE,
                        COLOR_MIN_TARGETS, COLOR_MAX_TARGETS, COLOR_COLOR_NAMES,
                        COLOR_INFER_EN, COLOR_INFER_RETRY, COLOR_INFER_MIN_BALLS,
                        COLOR_UNC_EN, COLOR_GUESS_MARGIN,
                        COLOR_BALL_CX_CENTER, COLOR_PULL_TOL_PX,
                        COLOR_INFER_MID_GAP_PX)

    timeout = COLOR_TIMEOUT if timeout is None else timeout
    cap = _open_color_with_retry(abort_event)
    if cap is None:
        # 打开阶段被打断同样视为「主动放弃」，而非失败
        if abort_event is not None and abort_event.is_set():
            print("[INFO] 色球识别任务被打断")
            return {"success": False, "aborted": True, "reason": "",
                    "frame": None, "targets": []}
        return {"success": False, "reason": "主摄像头打开失败",
                "frame": None, "targets": []}

    try:
        print(f"[INFO] 开始色球识别（三色投票, 每色≥{COLOR_MIN_VOTES}帧, "
              f"限时 {timeout:.0f}s）...")

        # ---- AE 收敛等待（2026-10-10 加）----
        # 相机租约从别的档位切回 main 时（典型：看路的手动曝光 166 → main 的自动），
        # 自动曝光要 ~1s 从旧亮度爬到新亮度；5 帧投票若落在爬坡期，红/绿球心逐帧漂
        # → 跨帧散布超限 → 整色作废（2026-10-10 21:30:55 实锤：红 48px / 绿 70px
        # > COLOR_MAX_SPREAD_PX=40，三色只剩蓝）。投票前先"排空"：连读两帧平均亮度
        # 差 <3% 才开始采票（最多等 1.2s —— 灯光真在变的话也不能卡死任务）。
        # 靶标（每帧 ~220ms、8 帧）摊得开不需要这招；色球帧快、5 帧全挤在爬坡前段。
        def _mean_v(f):
            small = cv2.resize(f, (48, 27), interpolation=cv2.INTER_AREA)
            return float(small.max(axis=2).mean())

        t_settle = time.time() + 1.2
        v_prev = None
        last_frame = None    # 最近读到的一帧 —— 失败/打断也要留档（统一记录接口,
                             # 见 handler._save_frame: 失败样本恰是排障最想看的）
        while time.time() < t_settle:
            if abort_event is not None and abort_event.is_set():
                print("[INFO] 色球识别任务被打断")
                return {"success": False, "aborted": True, "reason": "",
                        "frame": last_frame, "targets": []}
            f = CameraManager.read(cap, "Main Camera", max_retries=3)
            if f is None:
                time.sleep(0.05)
                continue
            last_frame = f
            v_now = _mean_v(f)
            if v_prev is not None and abs(v_now - v_prev) <= 0.03 * max(v_prev, 1.0):
                break
            v_prev = v_now

        deadline = time.time() + timeout
        votes = {1: [], 2: [], 3: []}     # color → [(cx, cy), ...]
        unc_votes = []                    # 无色候选票（帧内已过滤, 见循环内注释）
        frames_seen = 0
        best_frame = None

        # 采样分「轮」（pass）：第一轮只认出两色时再采一轮再谈推断 ——
        # 既然赛题必定给三颗，先多要点票数比直接开推更划算（票是同一机位、可累加）。
        pas, pass_limit = 0, 1
        while pas < pass_limit:
            while time.time() < deadline and frames_seen < COLOR_VOTE_FRAMES * (pas + 1):
                # 可随时打断：每一轮先检查中断信号
                if abort_event is not None and abort_event.is_set():
                    print("[INFO] 色球识别任务被打断")
                    return {"success": False, "aborted": True, "reason": "",
                            "frame": last_frame, "targets": []}

                frame = CameraManager.read(cap, "Main Camera", max_retries=3)
                if frame is None:
                    time.sleep(0.05)
                    continue
                last_frame = frame
                balls, _small, unc_blobs = _detect_frame(frame, COLOR_PROC_SCALE)
                frames_seen += 1
                if balls or unc_blobs:      # 无色候选在场时这帧对补救/推断同样有用
                    best_frame = frame
                for b in balls:
                    votes[b["color"]].append((b["cx"], b["cy"], b["r"]))

                # 无色候选票: 先丢掉与有色球挨太近的（同球裂块/反光, cx 差 <
                # MIN_GAP 的必不是另一颗球）, 再按"面积最接近本帧有色球"排序取前 3
                # —— 无色场景里真球与已定色球尺寸相仿, 远处小噪点圆排后面。
                if unc_blobs and COLOR_UNC_EN:
                    colored_cx = [b["cx"] for b in balls]
                    keep = [u for u in unc_blobs
                            if all(abs(u["cx"] - x) >= COLOR_MIN_GAP_PX
                                   for x in colored_cx)]
                    if balls:
                        ref = _median(sorted(b["area"] for b in balls))
                        keep.sort(key=lambda u: (abs(math.log2(
                            max(u["area"], 1.0) / ref)) if ref > 0 else -u["area"]))
                    else:
                        keep.sort(key=lambda u: -u["area"])
                    unc_votes.extend(keep[:3])

                time.sleep(0.01)  # 避免空转刷满 CPU

            n_ready = sum(1 for c in (1, 2, 3) if len(votes[c]) >= COLOR_MIN_VOTES)
            # 有一/两色但不满 → 再采一轮（推断之前先多要票; 一色场景同样值得:
            # 无色轮廓/第二色的票都可能再涨, 票是同一机位、可累加）
            if pas == 0 and n_ready in (1, 2) and COLOR_INFER_RETRY:
                print("[INFO] 未凑满三色 → 再采一轮（补救之前先多要票）")
                pass_limit = 2
            pas += 1

        # 众数（出现帧数）判存在 + 中位数取坐标（按颜色分别投票）
        targets = []
        for c in (1, 2, 3):
            pts = votes[c]
            if len(pts) < COLOR_MIN_VOTES:
                continue
            xs = sorted(p[0] for p in pts)
            ys = sorted(p[1] for p in pts)
            rs = sorted(p[2] for p in pts)
            if xs[-1] - xs[0] > COLOR_MAX_SPREAD_PX:
                print(f"[WARN] 色球[{COLOR_COLOR_NAMES.get(c, c)}] 跨帧散布 "
                      f"{xs[-1] - xs[0]:.0f}px 超限（车在动或误检），作废")
                continue
            # ⚠️⚠️ **cy 也要查同样一道闸**。它是回传字段（§6d），而且本车
            # BALL_XY_SWAP=1 ⇒ 单片机端「**横移 ← cy**、纵移 ← cx」（见固件 CLAUDE.md）。
            # 只查 cx 的话: 车沿**横向**挪动（减速尾巴/臂振动，"已停未稳"）时 cx 基本不动,
            # 闸不响, 而 cy 逐帧漂 → 取中位数得到一个**没人见过的位姿**的坐标,
            # 横移量静默偏掉（40px ≈ 13mm 以内不会被任何判据拦下）, 日志里"散布"还看着正常。
            if ys[-1] - ys[0] > COLOR_MAX_SPREAD_PX:
                print(f"[WARN] 色球[{COLOR_COLOR_NAMES.get(c, c)}] 跨帧 cy 散布 "
                      f"{ys[-1] - ys[0]:.0f}px 超限（车在横移或误检），作废")
                continue
            targets.append({"color": c,
                            "color_name": COLOR_COLOR_NAMES.get(c, str(c)),
                            "cx": _median(xs), "cy": _median(ys),
                            "r": _median(rs),
                            "votes": len(pts)})

        targets.sort(key=lambda t: t["cx"])

        # 无色轮廓补救（2026-10-26）: 轮廓在、颜色丢 → 排除法/概率猜色补**颜色**
        # （坐标实测, 优先于下面的缺球推断——那是坐标外推）。补不齐时 targets 可能
        # 正好剩 2 颗, 缺球推断接着推**坐标**, 两节接力。
        if len(targets) < COLOR_MIN_TARGETS and COLOR_UNC_EN and unc_votes:
            clusters = _cluster_unc_votes(unc_votes, COLOR_MIN_GAP_PX,
                                          COLOR_MAX_SPREAD_PX, COLOR_MIN_VOTES)
            missing = [c for c in (1, 2, 3)
                       if c not in {t["color"] for t in targets}]
            if clusters and missing:
                if len(clusters) > len(missing):
                    # 簇多（混进噪点簇）: 按票数×尺寸取前 N —— 无色簇没有色可投,
                    # 以"票多且球大"当可信度
                    clusters = sorted(clusters,
                                      key=lambda k: (-k["votes"], -k["r"]))[:len(missing)]
                for cl, col, note in _assign_unc_colors(clusters, missing,
                                                        COLOR_GUESS_MARGIN):
                    targets.append({"color": col,
                                    "color_name": COLOR_COLOR_NAMES.get(col, str(col)),
                                    "cx": cl["cx"], "cy": cl["cy"], "r": cl["r"],
                                    "votes": cl["votes"]})
                    print(f"[INFO] 无色轮廓补救: 补上 "
                          f"{COLOR_COLOR_NAMES.get(col, str(col))} "
                          f"({cl['cx']:.0f},{cl['cy']:.0f}) ← {note}")
                targets.sort(key=lambda t: t["cx"])
            else:
                # 诊断（2026-10-28 加）: 补救不成立不能静默——第三颗可能连饱和度掩膜
                # 都没过（过曝白球, color_0004 实拍）, 无色候选根本产生不了, 没这行
                # 日志就只能猜"为什么三个轮廓还失败"。
                print(f"[INFO] 无色补救不成立: 候选票 {len(unc_votes)}"
                      f" 有效簇 {len(clusters)} 缺色 {len(missing)}")
        elif len(targets) < COLOR_MIN_TARGETS and COLOR_UNC_EN:
            print("[INFO] 无色补救不成立: 全程无候选票（缺的球没过饱和度/几何筛）")

        # 缺球推断：只认出两颗 → 用板子坐标系（或两球外推）把第三颗推回来。
        # 推不出来就**不推**，照旧判失败（宁可长鸣也不给假坐标）。
        if len(targets) == COLOR_INFER_MIN_BALLS and COLOR_INFER_EN:
            before = {t["color"] for t in targets}
            targets, note = _infer_missing(best_frame, targets, COLOR_COLOR_NAMES)
            if note:
                new = [t for t in targets if t["color"] not in before]
                t = new[0] if new else targets[1]
                print(f"[INFO] 缺球推断: 补上 {t['color_name']} "
                      f"({t['cx']:.0f},{t['cy']:.0f}) ← {note}")
            else:
                print("[INFO] 缺球推断: 不成立（判据不过），这一步按失败处理")

        ok = (len(targets) >= COLOR_MIN_TARGETS)
        if ok and len(targets) > COLOR_MAX_TARGETS:
            targets = targets[:COLOR_MAX_TARGETS]
        if ok:
            gaps = [targets[i + 1]["cx"] - targets[i]["cx"]
                    for i in range(len(targets) - 1)]
            if gaps and min(gaps) < COLOR_MIN_GAP_PX:
                ok = False
                print(f"[WARN] 色球中心间隔过小 {min(gaps):.0f}px，疑似同一球裂成两块")

        if ok:
            yaw, fit_ok = fit_line_yaw(targets)
            order = " ".join(t["color_name"] for t in targets)
            print(f"[INFO] 色球识别成功（从左到右）: {order} "
                  f"连线角 {yaw:+.2f}° 共线{'OK' if fit_ok else 'NG'}")
            # 结果回传由 handler.py::_cmd_color 统一发送，任务层不碰串口
            annotated = (_annotate(best_frame, targets,
                                   extra=f"line {yaw:+.2f}deg fit{int(fit_ok)}")
                         if best_frame is not None else None)
            return {"success": True, "reason": "", "frame": annotated,
                    "frame_raw": best_frame,
                    "targets": targets, "yaw": yaw, "fit_ok": fit_ok}

        reason = f"色球识别超时/不足 ({timeout:.0f}s)：三色齐备 {len(targets)}/3"
        print(f"[WARN] {reason}")
        annotated = _annotate(best_frame, targets) if best_frame is not None else None
        # 失败也要给单片机一个"往哪边找"的方向（2026-10-26, ALIGN_PULLBACK 用）:
        # 能到失败路径说明补救/推断都没成, targets 全是实测有色球 —— 两颗的中点
        # 相对标定中球列明显偏一侧 = 缺的端球在画面外那一侧, 方向明确才值得倒回。
        pdir = _pull_dir(targets, COLOR_BALL_CX_CENTER, COLOR_PULL_TOL_PX,
                         COLOR_INFER_MID_GAP_PX)
        if pdir:
            mid = (targets[0]["cx"] + targets[1]["cx"]) / 2.0
            print(f"[INFO] 拉回方向: {pdir}({'左' if pdir == 1 else '右'}出界) "
                  f"两球中点 {mid:.0f} vs 理想列 {COLOR_BALL_CX_CENTER:.0f}")
        # 一帧都没检到时退回最后的原始帧 —— 失败也要留档（统一记录接口）
        return {"success": False, "reason": reason,
                "frame": annotated if annotated is not None else last_frame,
                "targets": targets, "yaw": 0.0, "fit_ok": False,
                "pull_dir": pdir}
    finally:
        release_camera(cap)     # 租约句柄 → 归还（保持打开）; 普通 cap → 真释放
