# tasks/_rect.py — 色球区**矩形地板**识别（主摄像头，一次性；2026-09 加）
#
# 用途：给一个**不依赖色球**的横向基准。缺一颗球时 `bColorCalcOffset` 那套
# （三球齐 → 按 cx 升序取中球做基准）整段拒动，而板子本身还在原地、位置每次一样，
# 所以「板心在画面里的坐标」同样可以当对位基准 —— 标定一次即可长期用。
#
# 检测对象：场地板上那圈**黑胶带**围出来的矩形（板面可能是白的、也可能是黄的，
# 所以**不看板面颜色**，只看黑胶带）。两条路，结果里的 `path` 说明走的哪条：
#
# 【主路径 enclosed】四边齐全时：暗像素掩膜 → 闭运算桥接断线 →
#   「自由像素」的连通域里**没与画面外连通**的那个大块 = 被胶带围住的板面 →
#   拟合四边形 → 对角线交点（世界矩形中心的射影，精确；estimated=False）。
#   ⚠️ 这条路**不会被"与胶带近乎共线但没把板子围起来"的暗线骗走**（地面缝、阴影、
#      另一块板的边）。真踩过：底边胶带旁边 ~20px 有一条贯穿整个画面的地面缝，
#      按"先求边线再求交"会把两条当成同一条线拟合 → 中心抖 ±25px，还会把真正的
#      底边误判成"同一条胶带的外沿"而丢掉，最后估到车体暗区上去。
#
# 【回退路径 lines】胶带环没围住（缺一条边、或板子伸出画面）时：
#   轮廓折线化 → 直线片段 → **胶带判据**（_is_tape：胶带是细暗条、两侧都亮；车体/
#   轮子/机械臂是大片暗区，剪影边界只有一侧亮 —— 没有这一道，车体轮廓会混进来当"边线"）
#   → 共线片段聚类最小二乘重拟合 → 最长的那条是**长边**（画面里 1000+px，短边 ~300px），
#   与它近平行的是另一条长边、横交(≥45°)的是短边（取"跨过长边中点 + 张得够开 +
#   胶带最长"的一对）→ 四边齐就两两求交（精确），缺一条边则按 RECT_HW_RATIO
#   （长边:短边，四边齐全时实测）仿射补出来再算中心（estimated=True）。
#
# ⚠️ **三边估出来的中心与四边精确中心差 15~30px（主要在横向）**：
#    仿射补边不吃透视（板子近端比远端大，短边左右也不一样长），这是方法本身的偏差、
#    不是标定能补掉的量。**标定就用你实际会跑的那个形态**：如果比赛时车一停就只有
#    三边，就把三边那次的读数当目标值；四边齐全时另测一次。两者别混用。
# ⚠️ 三边估中心只在"能看到两条完整短边 + 一条长边"（缺远端/近端长边，最常见：车顶得近、
#    近端那条边被车身/画面切掉）或"两条长边 + 一条短边"（缺左/右边）时可用；
#    只剩两条边就报失败（定不出尺寸），绝不猜。
# ⚠️ 只读一帧时车必须静止（和色球一样，多帧投票要求同一机位）。
#
# 任务层只返回结果 dict，串口回传/页面显示由调用方负责（与 _color.py 同构，保持纯净可测）。
import math
import struct
import time

import cv2
import numpy as np

from camera import CameraManager
from camera_lease import acquire_camera, release_camera

RECT_TAG = 0x07   # 结果帧子命令标记（TYPE_COORD DATA 首字节，<0x20 与 ASCII 帧不冲突）


def _clamp_u16(v):
    """坐标落进 u16（struct.pack('<H') 越界会抛异常）。"""
    return max(0, min(65535, int(round(v))))


def build_rect_frame(ok, cx=0.0, cy=0.0, estimated=False, n_sides=0,
                     enclosed=False, spread=0.0):
    """打包回传帧 DATA（见 docs/串口协议.md §6h）：

    [TAG=0x07][status u8][cx u16 LE][cy u16 LE][flags u8][spread u8][保留 u16] = 10 B

      status : 1 = 成功（cx/cy 有效）；0 = 失败（板子没识别出来/超时，cx/cy 填 0）
      flags  : bit0 = 三边估（estimated）、bit1~3 = 用了几条边、bit4 = 走的是主路径(量内沿)
      spread : 中心跨帧散布 [px]（封顶 255；只做遥测/排障，单片机不据此判失败）

    ⚠️ **失败帧也要发**（status=0）—— 单片机靠它立刻解锁，不然要空等满整个超时。
    ⚠️ 长度特意取 10（**不是 9**）：固件里"9 字节 = last packet 分包"那条通用分支
       （`ucDataLen >= 9`）虽然排在 0x07 分支之后，但长度撞上会让人误判，避开它。
    ⚠️ 坐标是**全分辨率 1920×1080** 像素（与色球/人质帧同口径，单片机按全分辨率标定）。
    """
    flags = (1 if estimated else 0) | ((int(n_sides) & 0x07) << 1) \
        | (0x10 if enclosed else 0)
    return struct.pack("<BBHHBBH", RECT_TAG, 1 if ok else 0,
                       _clamp_u16(cx), _clamp_u16(cy), flags,
                       max(0, min(255, int(round(spread)))), 0)

# 标注统一黄色：板面可能是白的/黄的、胶带是黑的，黄色对比度最好
_RECT_BGR = (0, 255, 255)
_CENTER_BGR = (255, 0, 255)


# ==================== 几何/掩膜小件 ====================
def _dark_mask(hsv, p):
    """黑胶带掩膜（低亮度 + 低饱和度: 挡掉彩色的暗部/反光）。"""
    m = ((hsv[:, :, 2] < p["v_max"]) & (hsv[:, :, 1] < p["s_max"])).astype(np.uint8) * 255
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE,
                            cv2.getStructuringElement(cv2.MORPH_RECT,
                                                      (p["close_k"],) * 2),
                            iterations=p["close_iter"])


def _sample(p0, p1, step=8.0):
    """沿片段等距取点（直线拟合用：点集越密，角度越稳）。"""
    L = float(np.linalg.norm(p1 - p0))
    n = max(2, int(L / step))
    t = np.linspace(0.0, 1.0, n)[:, None]
    return p0[None, :] * (1 - t) + p1[None, :] * t


def _segments(mask, p):
    """暗掩膜轮廓 → 折线近似 → 直线片段 [(p0, p1, len), ...]。"""
    min_len = p["min_seg_len"] * mask.shape[1]
    out = []
    for c in cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]:
        peri = cv2.arcLength(c, True)
        if peri < min_len:
            continue
        ap = cv2.approxPolyDP(c, 0.01 * peri, True).reshape(-1, 2)
        if len(ap) < 2:
            continue
        pts = np.vstack([ap, ap[:1]])
        for i in range(len(pts) - 1):
            a, b = pts[i].astype(float), pts[i + 1].astype(float)
            d = float(np.linalg.norm(b - a))
            if d >= min_len:
                out.append((a, b, d))
    return out


def _is_tape(v, p0, p1, p):
    """胶带判据：沿线采样，**两侧**垂直偏移处都应"不暗"。

    胶带是贴在地上的一条细暗条 → 垂直方向两侧都是板面/地面（亮）；
    车体、轮子、机械臂是大片暗区 → 它的一侧是暗的（剪影不是线）。
    没有这一道，车身轮廓会被当成"边线"。
    """
    d = p1 - p0
    L = float(np.linalg.norm(d))
    if L < 1:
        return False
    u = d / L
    nrm = np.array([-u[1], u[0]])
    h, w = v.shape
    n = max(3, int(L / 40))
    good = 0
    for t in np.linspace(0.1, 0.9, n):
        base = p0 + t * d
        ok_sides = 0
        for sgn in (1, -1):
            for o in (p["tape_off1"], p["tape_off2"]):
                q = base + sgn * o * nrm
                x, y = int(round(q[0])), int(round(q[1]))
                if 0 <= x < w and 0 <= y < h and v[y, x] > p["v_max"]:
                    ok_sides += 1
                    break
        if ok_sides == 2:
            good += 1
    return good >= 0.7 * n


def _angdiff(a, b):
    d = abs(a - b) % math.pi
    return min(d, math.pi - d)


def _fit_line(pts):
    """总体最小二乘拟合 → (theta ∈ [-π/2, π/2), rho)。theta/rho 定义同 _normal。"""
    mean = pts.mean(axis=0)
    # 主方向 = 协方差矩阵最大特征向量
    _u, _s, vt = np.linalg.svd(pts - mean, full_matrices=False)
    d = vt[0]
    th = _canon_angle(math.atan2(d[1], d[0]))
    nrm = np.array([math.sin(th), -math.cos(th)])
    return th, float(np.dot(mean, nrm))


def _canon_angle(th):
    """方向角归一到 [-π/2, π/2)。

    ⚠️ 必须归一：`angdiff` 按 mod π 比角度，但 **rho = <p, n(θ)> 会随 θ 跨 ±π/2 翻号**
    —— 不归一的话，「同一条胶带的两条边」里朝左画的那条算出 θ≈178.7°、rho=+645，
    朝右画的算出 θ≈0.4°、rho=−646，两条明明是**一条线**却永远聚不到一起
    （真踩过：底边胶带裂成两条"长边"，第二条被当成同一条胶带的外沿丢掉 → 只有三边时直接失败）。
    """
    if th < -math.pi / 2:
        return th + math.pi
    if th >= math.pi / 2:
        return th - math.pi
    return th


def _cluster_lines(segs, p):
    """共线片段聚成一条边线（角度 + 垂距都在容差内）；支撑长度取**投影跨度**。"""
    lines = []
    for s in segs:
        p0, p1, _L = s
        d = p1 - p0
        th = _canon_angle(math.atan2(d[1], d[0]))
        nrm = np.array([math.sin(th), -math.cos(th)])
        rho = float(np.dot(p0, nrm))
        hit = None
        for ln in lines:
            if _angdiff(th, ln["theta"]) > math.radians(p["angle_tol"]):
                continue
            if abs(rho - ln["rho"]) > p["dist_tol"]:
                continue
            hit = ln
            break
        if hit is None:
            lines.append(dict(theta=th, rho=rho, segs=[(p0, p1)], pts=_sample(p0, p1)))
        else:
            hit["segs"].append((p0, p1))
            hit["pts"] = np.vstack([hit["pts"], _sample(p0, p1)])
    for ln in lines:
        ln["theta"], ln["rho"] = _fit_line(ln["pts"])
        u = np.array([math.cos(ln["theta"]), math.sin(ln["theta"])])
        proj = ln["pts"] @ u
        ln["span"] = float(proj.max() - proj.min())
        ln["mid"] = ln["pts"].mean(axis=0)
    return lines


def _normal(ln):
    return np.array([math.sin(ln["theta"]), -math.cos(ln["theta"])])


def _inter(l1, l2):
    """两条边线的交点；近平行（<1.2°）返回 None。"""
    A = np.vstack([_normal(l1), _normal(l2)])
    if abs(np.linalg.det(A)) < 2e-2:
        return None
    return np.linalg.solve(A, np.array([l1["rho"], l2["rho"]]))


def _dist_to(pt, ln):
    return abs(float(np.dot(pt, _normal(ln))) - ln["rho"])


def _far_dir(from_ln, other_ln):
    """沿 from_ln **离开** other_ln 的方向 = "往板子内部"的方向。

    用片段端点判：from_ln 的胶带从贴着 other_ln 的那一头，往板子里面延伸。
    """
    far, fd = None, -1.0
    near, nd = None, 1e9
    for p0, p1 in from_ln["segs"]:
        for pt in (p0, p1):
            d = _dist_to(pt, other_ln)
            if d > fd:
                far, fd = pt, d
            if d < nd:
                near, nd = pt, d
    if far is None:
        return None
    vec = far - near
    n = float(np.linalg.norm(vec))
    return vec / n if n > 1 else None


def _order4(pts):
    """四角排成 左上 → 右上 → 右下 → 左下。

    ⚠️ `reshape(-1, 2)` 不能省：既会收到交点的 (4,2) 列表，也会收到 approxPolyDP 出来的
    (4,1,2) 轮廓 —— 后者不整形的话 `pts[:, 1]` 取的是"第二行"而不是 y，直接 IndexError。
    """
    pts = np.array(pts, dtype=np.float32).reshape(-1, 2)
    c = pts.mean(axis=0)
    pts = pts[np.argsort(np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0]))]
    return np.roll(pts, -int(np.argmin(pts.sum(axis=1))), axis=0)


def _quad_center(q):
    """对角线交点 = 世界矩形中心的射影（比四角均值更正确：透视下两者不重合）。"""
    (x1, y1), (x2, y2), (x3, y3), (x4, y4) = q
    d1 = np.array([x3 - x1, y3 - y1])
    d2 = np.array([x4 - x2, y4 - y2])
    A = np.column_stack([d1, -d2])
    if abs(np.linalg.det(A)) < 1e-6:
        return q.mean(axis=0)
    t = np.linalg.solve(A, np.array([x2 - x1, y2 - y1]))
    return np.array(q[0] + t[0] * d1)


# ==================== 单帧检测 ====================
def _enclosed_quads(hsv, mask, p):
    """**被黑胶带围住的亮区** → 四边形候选（四边齐全时的主路径）。

    做法：取"自由像素"（非暗）的连通域，**与画面边界连通的那个 = 板子外面**（板面
    没被围住时就是它自己），剩下的大块就是被胶带围起来的板面 → 拟合四边形。
    围不住的线在这里天然不产生候选，所以**骗不过它**（地面缝/阴影/另一块板的边）
    —— 这正是主路径存在的理由，见文件头。

    返回按可见面积降序；每项 {quad, area_frac, inner_v, fill}。
    """
    h, w = hsv.shape[:2]
    v = hsv[:, :, 2]
    frame_area = float(h * w)
    n, labels, stats, _ = cv2.connectedComponentsWithStats(
        (1 - (mask > 0).astype(np.uint8)).astype(np.uint8), connectivity=4)
    on_border = set(labels[0, :].tolist()) | set(labels[-1, :].tolist()) \
        | set(labels[:, 0].tolist()) | set(labels[:, -1].tolist())
    med_v = float(np.median(v))

    out = []
    for lab in range(1, n):
        if lab in on_border:                        # 与画面外连通 = 没被围住
            continue
        if stats[lab, cv2.CC_STAT_AREA] < p["min_area_frac"] * frame_area:
            continue
        comp = (labels == lab).astype(np.uint8) * 255
        # 闭运算吞掉球影/胶带毛刺（否则轮廓上全是小锯齿，approxPolyDP 出不了 4 个角）
        comp = cv2.morphologyEx(comp, cv2.MORPH_CLOSE,
                                cv2.getStructuringElement(cv2.MORPH_RECT,
                                                          (p["inner_close"],) * 2))
        px = v[comp > 0]
        if px.size == 0:
            continue
        mean_v = float(px.mean())
        # 框内亮度两关：绝对下限 + 比画面中位亮（车体/轮子围出的暗腔过不去；
        # 板面是白是黄都无所谓 —— 只有 V 进这个判据）
        if mean_v < p["min_inner_v"] or mean_v < p["min_inner_ratio"] * med_v:
            continue
        cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        c = max(cnts, key=cv2.contourArea)
        a = cv2.contourArea(c)
        if a < p["min_area_frac"] * frame_area:
            continue
        ap = cv2.approxPolyDP(c, p["eps_frac"] * cv2.arcLength(c, True), True)
        if len(ap) != 4 or not cv2.isContourConvex(ap):
            continue
        # 边拟合必须用**逐像素**轮廓（NONE）：CHAIN_APPROX_SIMPLE 会把一条 1000px 的直边
        # 压成 2 个点，那样拟合出来的是"毛刺"而不是"边"（真踩过：角点反而偏了 17px）。
        dense = max(cv2.findContours(comp, cv2.RETR_EXTERNAL,
                                     cv2.CHAIN_APPROX_NONE)[0], key=cv2.contourArea)
        quad = _refine_quad(dense, ap, p)           # 边拟合求交 → 亚像素的角点
        # 四条边外侧都必须紧贴黑胶带 —— 板面被一条暗纹（机械臂影子/曝光分界）切成两半时，
        # 其中**一半**也照样是"被围住的亮区"，面积/填充/亮度闸全过得了，只是中心偏 200px。
        # 实测这种帧占 10~30%（多帧离群能剔掉，但万一成了多数就会报错），所以从几何上堵死。
        if not _quad_edges_on_tape(v,
                                   (stats[lab, cv2.CC_STAT_LEFT]
                                    + stats[lab, cv2.CC_STAT_WIDTH] * 0.5,
                                    stats[lab, cv2.CC_STAT_TOP]
                                    + stats[lab, cv2.CC_STAT_HEIGHT] * 0.5),
                                   quad, p):
            continue
        (_c, (rw, rh), _ang) = cv2.minAreaRect(quad.astype(np.float32))
        if rw < 1.0 or rh < 1.0:
            continue
        fill = a / (rw * rh)
        _asp = max(rw, rh) / min(rw, rh)
        # 长宽比**上下限都判**：下限是治"板子被暗纹切成一块"的兜底 —— 切掉一截的碎块
        # 长宽比会明显变小（整块 3.8 → 切掉 40% 只剩 2.3），而它四条边照样都贴着暗线、
        # 亮度/填充也照过。留 2.5 的余量（本机位实测整块 3.5~3.8）。
        if fill < p["min_fill"] or not (p["min_aspect"] <= _asp <= p["max_aspect"]):
            continue
        out.append(dict(quad=_order4(quad),
                        area_frac=float((comp > 0).sum()) / frame_area,
                        inner_v=mean_v, fill=float(fill)))
    out.sort(key=lambda q: -q["area_frac"])
    return out


def _quad_edges_on_tape(v, cen, quad, p):
    """四边形的四条边，外侧都要紧贴黑胶带（被围住的板面边界就该是胶带内沿）。

    判据：沿边采样，取**背离质心**的方向偏移 `RECT_TAPE_NEAR_PX` 像素处应是暗的（胶带）。
    用质心定"外侧"而不是靠顶点顺序 —— 顺序在拟合后可能被 `_order4` 重排过。

    ⚠️ 这是治"板面被一条暗纹切成两半"的那一道：半块板也是"被围住的亮区"，
    面积/填充/亮度三关全过，中心却偏 ~200px（实测占 10~30% 的帧）。它那条**分界线**
    外侧是亮的板面而不是胶带，所以在这里被挡下。
    """
    c = np.asarray(cen, dtype=np.float64)
    h, w = v.shape
    near = p["tape_near_px"]
    q = np.asarray(quad, dtype=np.float64).reshape(-1, 2)
    for i in range(4):
        a, b = q[i], q[(i + 1) % 4]
        ab = b - a
        L = float(np.linalg.norm(ab))
        if L < 40.0:
            return False
        u = ab / L
        nrm = np.array([-u[1], u[0]])
        mid = 0.5 * (a + b)
        if float(np.dot(nrm, mid - c)) < 0:        # 让法向指向**外侧**
            nrm = -nrm
        n = max(4, int(L / 40))
        good = 0
        for t in np.linspace(0.12, 0.88, n):       # 两端让开，躲角上的胶带接缝
            base = a + t * ab
            for off in (near, near * 1.8):
                pt = base + off * nrm
                x, y = int(round(pt[0])), int(round(pt[1]))
                if 0 <= x < w and 0 <= y < h and v[y, x] < p["v_max"]:
                    good += 1
                    break
        if good < 0.7 * n:
            return False
    return True


def _refine_quad(contour, quad, p):
    """四条边分别做直线拟合、再两两求交 —— 取代 approxPolyDP 的顶点。

    ⚠️ approxPolyDP 返回的顶点是**轮廓上现成的点**：一条 1000px 的直边上有
    eps(=RECT_EPS_FRAC×周长, 默认 ~60px) 的活动空间 —— 边界每帧抖 1px，顶点就能沿边
    滑动几十 px（实测：同一个角相邻两帧差 25px，画出来的框肉眼可见地跳）。
    对整条边的**几百个点**做最小二乘，等于把这点噪声平均掉 → 角点稳定到亚像素。
    拟合不成（点太少/两线近平行）就原样退回 approxPolyDP 的结果，不硬凑。
    """
    pts = contour.reshape(-1, 2).astype(np.float64)
    q = np.array(quad, dtype=np.float64).reshape(-1, 2)
    if len(q) != 4 or len(pts) < 8:
        return q
    band = p["edge_band_frac"] * cv2.arcLength(contour, True)
    sides = []
    for i in range(4):
        a = q[i]
        ab = q[(i + 1) % 4] - a
        L2 = float(ab @ ab)
        if L2 < 1.0:
            return q
        t = ((pts - a) @ ab) / L2
        # 点到该边所在直线的垂距 = |叉积| / 边长
        perp = np.abs(ab[0] * (pts[:, 1] - a[1]) - ab[1] * (pts[:, 0] - a[0])) / math.sqrt(L2)
        # 只收"贴着这条边、且不在角附近"的点：两端各让出 5%（躲开圆角/胶带转角毛刺）
        m = (t > 0.05) & (t < 0.95) & (perp < band)
        if int(m.sum()) < p["edge_min_pts"]:
            return q
        th, rho = _fit_line(pts[m])
        sides.append(dict(theta=th, rho=rho))
    out = []
    for i in range(4):
        pt = _inter(sides[(i - 1) % 4], sides[i])   # 第 i 个角 = 相邻两条边线的交点
        if pt is None:
            return q
        out.append(pt)
    return _order4(out)


def detect_rect(img, p):
    """单帧检测 → dict（纯函数，无串口/摄像头依赖）。

    成功: {"success", "center"(cx,cy 全分辨率), "corners"(4×2 左上起顺时针),
           "estimated"(True=三边估出), "n_sides", "inner_v", "area_frac", "hw_ratio"}
    失败: {"success": False, "reason": "...", "dbg": {...}}
    """
    h, w = img.shape[:2]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)     # 只转一次, V/S 与暗掩膜共用
    v = hsv[:, :, 2]
    mask = _dark_mask(hsv, p)

    # ---- 主路径: 被胶带围住的亮区（四边齐全时走这条，最稳也最准）----
    eq = _enclosed_quads(hsv, mask, p)
    if eq:
        b = eq[0]
        quad, center = b["quad"], _quad_center(b["quad"])
        dbg = dict(path="enclosed", n_quad=len(eq), area_frac=b["area_frac"],
                   inner_v=b["inner_v"], fill=b["fill"], center=center.tolist())
        if 0 <= center[0] < w and 0 <= center[1] < h:
            return dict(success=True, reason="", dbg=dbg, est=False, path="enclosed",
                        center=(float(center[0]), float(center[1])),
                        corners=[(float(x), float(y)) for x, y in quad],
                        area_frac=b["area_frac"], inner_v=b["inner_v"], n_sides=4)

    # ---- 回退路径: 边线求交（胶带环没围住: 缺一条边 / 板子伸出画面）----
    segs = [s for s in _segments(mask, p) if _is_tape(v, s[0], s[1], p)]
    lines = sorted(_cluster_lines(segs, p), key=lambda ln: -ln["span"])
    dbg = dict(n_seg=len(segs), n_line=len(lines),
               lines=[dict(th=round(math.degrees(ln["theta"]), 1),
                           span=int(ln["span"])) for ln in lines])
    if not lines:
        return dict(success=False, reason="未找到黑胶带边线", dbg=dbg)

    # 长度闸分两档：**长边**必须够长（比它长的才算"板子的长边"），
    # 短边天生就短（~0.16 画面宽），被画面切掉一截还可能只剩几十 px —— 用大闸会
    # 直接把"缺左边"这种情形判死（真踩过：左边被裁掉一半后再也进不了候选）。
    min_long = p["min_line_len"] * w
    min_short = p["min_short_line"] * w
    anchor = lines[0]
    if anchor["span"] < min_long:
        return dict(success=False, reason="最长的黑胶带边线只有 %dpx（太短）"
                    % int(anchor["span"]), dbg=dbg)
    lines = lines[:p["max_lines"]]

    longs, shorts_cand = [anchor], []
    for ln in lines[1:]:
        da = math.degrees(_angdiff(ln["theta"], anchor["theta"]))
        if da <= p["par_tol"]:
            if ln["span"] >= min_long * 0.7:
                longs.append(ln)
        elif da >= p["trans_min"] and ln["span"] >= min_short:
            shorts_cand.append(ln)

    # 短边 = 板子的左右两条边：在锚线上交点**跨过锚线中点**、张开宽度够、且
    # **胶带最长**的那一对。三条都要：
    #   · 跨中点 —— 板子的中段必在这两条边之间。少了它，"只有三边"时会把同侧两条脏线
    #     （都在左端）当成左右边，拼出盖在车身暗区上的假四边形。
    #   · 张开够宽（min_width_frac）—— 两条边不可能贴在一起。
    #   · 取**总支撑长度最大**而不是"张开最宽" —— 板子的边是画面上最长的横向胶带；
    #     按最宽挑，左边一条更靠外缘的短脏线会赢过真正的左边线（真踩过：中心被拽左 60px）。
    au = np.array([math.cos(anchor["theta"]), math.sin(anchor["theta"])])
    aproj = anchor["pts"] @ au
    amid = 0.5 * float(aproj.max() + aproj.min())
    proj = []
    for ln in shorts_cand:
        pt = _inter(anchor, ln)
        if pt is not None:
            proj.append((ln, pt, float(np.dot(pt, au)) - amid))
    pair, sep, score = None, 0.0, -1.0
    for i in range(len(proj)):
        for j in range(i + 1, len(proj)):
            if proj[i][2] * proj[j][2] >= 0:            # 同侧 → 不是板子的左右边
                continue
            d = float(np.linalg.norm(proj[i][1] - proj[j][1]))
            if d < p["min_width_frac"] * anchor["span"]:
                continue
            s = proj[i][0]["span"] + proj[j][0]["span"]
            if s > score:
                pair, sep, score = (proj[i][0], proj[j][0]), d, s
    shorts = list(pair) if pair else []
    dbg["sep"] = int(sep)
    if not shorts and proj:                # 只剩一条短边（缺左边或右边）
        shorts = [sorted(proj, key=lambda x: -x[0]["span"])[0][0]]

    # 长边定案，三道判据（顺序有讲究：先按"像不像板子的边"筛，再限条数）：
    #   ① **端点判据**: 板子的长边必定经过短边胶带的两端（那里就是两个角）。
    #      没有这一道，"缺左边/右边"时会拿画面底部那条贯穿全幅的地面缝当长边
    #      （它离角点几百 px，却因为更长而在②③里赢）→ 直接失败。（真踩过）
    #   ② **相对位置**: 两条长边不能贴在一起 —— 那是**同一条胶带的内外沿**
    #      （胶带宽 ~10px，两条长边间距却是几百 px），会拼出细长条四边形。
    #   ③ 支撑长度：越长越可能是板子的长边。
    if shorts:
        ends = [pt for s in shorts for p0, p1 in s["segs"] for pt in (p0, p1)]
        keep = [ln for ln in longs
                if min(_dist_to(e, ln) for e in ends) <= p["end_tol"]]
        if keep:
            longs = keep
    longs = sorted(longs, key=lambda ln: -ln["span"])[:2]
    if len(longs) > 1:
        need = p["min_gap_frac"] * (sep if sep > 0 else anchor["span"])
        if _dist_to(longs[1]["mid"], longs[0]) < need:
            longs = longs[:1]
    dbg["n_long"], dbg["n_short"] = len(longs), len(shorts)

    est = False
    if len(longs) == 2 and len(shorts) == 2:
        pts = [_inter(a, b) for a in longs for b in shorts]
        if any(x is None for x in pts):
            return dict(success=False, reason="边线投影退化（疑似重合）", dbg=dbg)
        quad = _order4(pts)
    elif len(shorts) == 2:                 # 缺一条长边（最常见：近端那条被车身/画面切掉）
        base = longs[0]                    # 唯一/较长的那条长边（可能是被端点判据换过的）
        i1, i2 = _inter(base, shorts[0]), _inter(base, shorts[1])
        if i1 is None or i2 is None:
            return dict(success=False, reason="短边与长边交不出来", dbg=dbg)
        hpx = float(np.linalg.norm(i2 - i1)) / p["hw_ratio"]
        u1, u2 = _far_dir(shorts[0], base), _far_dir(shorts[1], base)
        if u1 is None or u2 is None:
            return dict(success=False, reason="短边朝向判不出", dbg=dbg)
        if float(np.dot(u1, u2)) < 0:
            u2 = -u2
        quad = _order4([i1, i2, i2 + hpx * u2, i1 + hpx * u1])
        est = True
    elif len(longs) == 2 and shorts:         # 缺一条短边（左边或右边出画）—— ⚠️ shorts 必须非空
        # 2026-10 修: 板子两条短边都出画时 shorts==[] 也会落进本分支, 旧版直接
        # shorts[0] → IndexError 一路抛穿 run_rect_detect（它没有单帧 try/except）→
        # 连失败帧都不发、单片机空烧整超时, 违反 §6h「失败帧也一定回」与文件头
        # 「只剩两条边直接报失败, 绝不猜」的约定。现在空则掉到下面的 else 报失败。
        a = _inter(longs[0], shorts[0])
        d = _inter(longs[1], shorts[0])
        if a is None or d is None:
            return dict(success=False, reason="长边与短边交不出来", dbg=dbg)
        wpx = float(np.linalg.norm(d - a)) * p["hw_ratio"]
        ua, ud = _far_dir(longs[0], shorts[0]), _far_dir(longs[1], shorts[0])
        if ua is None or ud is None:
            return dict(success=False, reason="长边朝向判不出", dbg=dbg)
        if float(np.dot(ua, ud)) < 0:
            ud = -ud
        quad = _order4([a, a + wpx * ua, d + wpx * ud, d])
        est = True
    else:
        return dict(success=False, reason="边线不足以定出矩形（只见 %d 条）"
                    % (len(longs) + len(shorts)), dbg=dbg)

    center = _quad_center(quad)
    inner = np.zeros((h, w), np.uint8)
    cv2.fillPoly(inner, [quad.astype(np.int32)], 255)
    px = v[cv2.erode(inner, np.ones((15, 15), np.uint8)) > 0]
    inner_v = float(px.mean()) if px.size else 0.0
    area_frac = float((inner > 0).sum()) / (w * h)
    med_v = float(np.median(v))
    dbg["center"], dbg["area_frac"], dbg["inner_v"] = center.tolist(), area_frac, inner_v

    # 板心落在画面外 = 这个四边形不可能是板子（估歪了/误检）
    if not (0 <= center[0] < w and 0 <= center[1] < h):
        return dict(success=False, reason="矩形中心落在画面外", dbg=dbg)
    if area_frac < p["min_area_frac"]:
        return dict(success=False, reason="可见面积太小(%.1f%%)" % (area_frac * 100),
                    dbg=dbg)
    # 框内亮度：既要过绝对下限，也要**比画面中位亮度亮**（板面比地面亮，车体/机械臂
    # 那种大片暗区过不去）。板面是白是黄都无所谓 —— 只有 V（亮度）进这个判据。
    if inner_v < p["min_inner_v"] or inner_v < p["min_inner_ratio"] * med_v:
        return dict(success=False,
                    reason="框内偏暗(V=%.0f, 画面中位 %.0f)，不像板面" % (inner_v, med_v),
                    dbg=dbg)

    return dict(success=True, reason="", dbg=dbg, est=est, path="lines",
                center=(float(center[0]), float(center[1])),
                corners=[(float(x), float(y)) for x, y in quad],
                area_frac=area_frac, inner_v=inner_v,
                n_sides=len(longs) + len(shorts))


# ==================== 任务（多帧投票） ====================
def _params():
    """参数每次调用现取（Web 参数页改完即时生效，同 _target_stream 的做法）。"""
    from config import (RECT_V_MAX, RECT_S_MAX, RECT_CLOSE_K, RECT_CLOSE_ITER,
                        RECT_MIN_VOTES, RECT_MAX_SPREAD_PX,
                        RECT_MIN_SEG_LEN, RECT_ANGLE_TOL_DEG, RECT_DIST_TOL_PX,
                        RECT_MIN_LINE_LEN, RECT_MIN_SHORT_LINE, RECT_TAPE_OFF1,
                        RECT_TAPE_OFF2, RECT_PAR_TOL_DEG, RECT_TRANS_MIN_DEG,
                        RECT_MIN_GAP_FRAC, RECT_MIN_WIDTH_FRAC, RECT_MIN_AREA_FRAC,
                        RECT_MIN_INNER_V, RECT_MIN_INNER_RATIO, RECT_HW_RATIO,
                        RECT_END_TOL_PX, RECT_INNER_CLOSE, RECT_EPS_FRAC,
                        RECT_MIN_FILL, RECT_MAX_ASPECT, RECT_MIN_ASPECT,
                        RECT_EDGE_BAND_FRAC,
                        RECT_EDGE_MIN_PTS, RECT_TAPE_NEAR_PX,
                        RECT_OUTLIER_K, RECT_OUTLIER_MIN_PX)
    return dict(v_max=RECT_V_MAX, s_max=RECT_S_MAX, close_k=RECT_CLOSE_K,
                close_iter=RECT_CLOSE_ITER, min_seg_len=RECT_MIN_SEG_LEN,
                angle_tol=RECT_ANGLE_TOL_DEG, dist_tol=RECT_DIST_TOL_PX,
                min_line_len=RECT_MIN_LINE_LEN, min_short_line=RECT_MIN_SHORT_LINE,
                tape_off1=RECT_TAPE_OFF1, tape_off2=RECT_TAPE_OFF2,
                par_tol=RECT_PAR_TOL_DEG, trans_min=RECT_TRANS_MIN_DEG,
                min_gap_frac=RECT_MIN_GAP_FRAC, min_width_frac=RECT_MIN_WIDTH_FRAC,
                min_area_frac=RECT_MIN_AREA_FRAC, min_inner_v=RECT_MIN_INNER_V,
                min_inner_ratio=RECT_MIN_INNER_RATIO, hw_ratio=RECT_HW_RATIO,
                end_tol=RECT_END_TOL_PX,
                inner_close=RECT_INNER_CLOSE, eps_frac=RECT_EPS_FRAC,
                min_fill=RECT_MIN_FILL, max_aspect=RECT_MAX_ASPECT,
                min_aspect=RECT_MIN_ASPECT, min_votes=RECT_MIN_VOTES,
                max_spread_px=RECT_MAX_SPREAD_PX,
                edge_band_frac=RECT_EDGE_BAND_FRAC, edge_min_pts=RECT_EDGE_MIN_PTS,
                tape_near_px=RECT_TAPE_NEAR_PX,
                outlier_k=RECT_OUTLIER_K, outlier_min_px=RECT_OUTLIER_MIN_PX,
                max_lines=10)


def _median(xs):
    xs = sorted(xs)
    n = len(xs)
    return xs[n // 2] if n % 2 else (xs[n // 2 - 1] + xs[n // 2]) / 2.0


def _spread(xs, ys):
    """一组中心点的最大散布（取两轴里大的那个）。"""
    if not xs:
        return 0.0
    return max(max(xs) - min(xs), max(ys) - min(ys))


def _ambiguous_gap(xs, ys, p):
    """帧中心是否分成**两簇**（两簇都够票数、间距超过散布闸）→ 返回间距，否则 0。

    ⚠️ 为什么必须在剔离群**之前**判：把板子切成两块时（暗纹/机械臂影子横穿板面），
    5 帧读到半块（中心偏 200px）、4 帧读到整块 —— "取中位 + MAD 剔离群"会**把少数那 4 帧
    当成离群剔掉**，然后信心满满地报出错 200px 的数（`spread` 还是 0，看着特别稳）。
    实测过（`synth_split.jpg`：合成一条暗线把板子切开 → 报 1132 而真值是 932）。
    所以先判双峰：说不清是哪块板就**不给数**，宁可失败。
    """
    n = len(xs)
    if n < 2 * p["min_votes"]:
        return 0.0
    for vals in (xs, ys):
        order = sorted(vals)
        for k in range(p["min_votes"], n - p["min_votes"] + 1):
            gap = order[k] - order[k - 1]
            if gap > p["max_spread_px"]:
                return gap
    return 0.0


def _inlier_mask(vals, p):
    """MAD 离群判据：|v − 中位| ≤ max(下限, k×1.4826×MAD)。

    · 抖得小（MAD 小）→ 阈值被 `outlier_min_px` 兜住，**一个都不剔**（不误伤）
    · 抖得野（个别帧跳了几十 px）→ 阈值跟着 MAD 放大，但跳得离谱的那些仍然出局
    1.4826 是把 MAD 折算成正态标准差系数的老常数，k=3 即"3σ"，用在这里只是取个量级。
    """
    med = _median(vals)
    dev = [abs(v - med) for v in vals]
    thr = max(p["outlier_min_px"], p["outlier_k"] * 1.4826 * _median(dev))
    return [d <= thr for d in dev]


def _annotate(frame, res):
    """画框 + 四个角 + 中心十字（黄框/品红十字：板面可能是白的或黄的）。

    ⚠️ 入参既可能是 run_rect_detect 的结果，也可能是**单帧** detect_rect 的结果
    （帧数不够时回显用）—— 两者的键不一样（单帧里是 `est`，不是 `estimated`），
    所以一律 `.get` 取，别下标：下标会在"2 帧成功 1 帧失败"这种半成功路径上
    抛 KeyError，把一次本该干净报失败的情况变成崩。（离线测试抓到的）
    """
    if frame is None:
        return None
    out = frame.copy()
    res = res or {}
    if res.get("center") is None or res.get("corners") is None:
        return out
    q = np.array(res["corners"], np.int32)
    cv2.polylines(out, [q], True, _RECT_BGR, 4)
    for i, (x, y) in enumerate(res["corners"]):
        cv2.circle(out, (int(x), int(y)), 10, _RECT_BGR, -1)
        cv2.putText(out, "P%d" % i, (int(x) + 12, int(y)), 0, 1.2, _RECT_BGR, 3)
    cx, cy = res["center"]
    cv2.drawMarker(out, (int(cx), int(cy)), _CENTER_BGR, cv2.MARKER_CROSS, 60, 4)
    est = res.get("estimated", res.get("est", False))
    # ⚠️ 标注一律 ASCII：cv2.putText 画不了中文（写进去就是一排 "?"，真踩过）
    cv2.putText(out, "C(%d,%d) %s" % (cx, cy, "EST-3side" if est else "4side"),
                (int(cx) + 18, int(cy) - 20), 0, 1.2, _CENTER_BGR, 3)
    return out


def run_rect_detect(timeout=None, abort_event=None):
    """主摄像头矩形地板识别（限时 timeout 秒，默认取 config.RECT_TIMEOUT）。

    逐帧检测（detect_rect）→ 多帧**去离群**：
      ① 按量模型分组（path/estimated 不同的帧是另一种测法，不混着平均）
      ② MAD 剔除跳档的帧（`RECT_OUTLIER_K`/`RECT_OUTLIER_MIN_PX`）
      ③ 内点取**均值**（比单帧稳 √N 倍；也因为均值才值得多采几帧）
    内点里还剩的散布超过 RECT_MAX_SPREAD_PX（车在动）作废，报失败。

    Returns dict: {"success", "reason", "frame"(标注帧), "center"(cx,cy 全分辨率),
                   "corners"(4×2), "estimated", "n_sides", "path", "votes",
                   "inliers", "dropped", "spread", "spread_all", "frames"}
        中心/四角 = 内点均值；`frames` 是**各帧原始中心**（排障/标定时看抖动用）。
        center/corners 全部是**全分辨率 1920×1080**像素（与单片机侧标定同口径）。
        aborted=True 只在被打断时出现。
    """
    from config import (RECT_TIMEOUT, RECT_VOTE_FRAMES, RECT_MIN_VOTES,
                        RECT_MAX_SPREAD_PX, RECT_OUTLIER_K, RECT_OUTLIER_MIN_PX)

    timeout = RECT_TIMEOUT if timeout is None else timeout
    fail = {"success": False, "reason": "", "frame": None, "center": None,
            "corners": None, "estimated": False, "n_sides": 0, "votes": 0,
            "inliers": 0, "dropped": 0, "spread": 0.0, "spread_all": 0.0,
            "frames": []}

    cap = acquire_camera(is_qr=False)          # 主摄像头（命中租约则复用预热好的）
    if cap is None:
        if abort_event is not None and abort_event.is_set():
            print("[INFO] 矩形地板识别任务被打断")
            return dict(fail, aborted=True)
        return dict(fail, reason="主摄像头打开失败")

    try:
        print(f"[INFO] 开始矩形地板识别（黑胶带边框, 最多 {RECT_VOTE_FRAMES} 帧, "
              f"限时 {timeout:.0f}s）...")
        p = _params()
        deadline = time.time() + timeout
        hits = []                              # 每帧: (center, corners, est, frame)
        frames_seen = 0
        while time.time() < deadline and frames_seen < RECT_VOTE_FRAMES:
            if abort_event is not None and abort_event.is_set():
                print("[INFO] 矩形地板识别任务被打断")
                return dict(fail, aborted=True)
            frame = CameraManager.read(cap, "Main Camera", max_retries=3)
            if frame is None:
                time.sleep(0.05)
                continue
            frames_seen += 1
            r = detect_rect(frame, p)
            if r["success"]:
                hits.append((r, frame))
            else:
                print(f"[WARN] 矩形地板识别第 {frames_seen} 帧失败: {r['reason']} "
                      f"{r.get('dbg', {})}")
            time.sleep(0.01)

        if len(hits) < RECT_MIN_VOTES:
            reason = (f"矩形地板识别失败：{frames_seen} 帧里只有 {len(hits)} 帧检到 "
                      f"(需≥{RECT_MIN_VOTES})")
            print(f"[WARN] {reason}")
            best = hits[0][1] if hits else None
            base = hits[0][0] if hits else None
            return dict(fail, reason=reason,
                        frame=_annotate(best, base) if base else None,
                        votes=len(hits))

        # ① **按量模型分组**：path/estimated 不同的帧是"另一种测法"（精确 vs 三边估，
        #    两者本就有 ~20px 系统差），混在一起平均会得到一个谁也不是的数。
        #    取样本最多的那一组，其余记在 groups 里备查。
        groups = {}
        for h in hits:
            groups.setdefault((h[0].get("path", ""), bool(h[0]["est"])), []).append(h)
        n_other = len(hits) - max(len(g) for g in groups.values())
        key = max(groups, key=lambda k: len(groups[k]))
        grp = groups[key]
        if len(grp) < RECT_MIN_VOTES:
            reason = (f"矩形地板识别失败：{len(hits)} 帧里同一种测法只有 {len(grp)} 帧"
                      f"(需≥{RECT_MIN_VOTES}，另 {n_other} 帧走了另一条路)")
            print(f"[WARN] {reason}")
            return dict(fail, reason=reason, frame=_annotate(grp[0][1], grp[0][0]),
                        votes=len(hits))

        # ② **先判双峰，再剔离群**（顺序反了会静默报错，见 _ambiguous_gap 的注释）
        xs = [h[0]["center"][0] for h in grp]
        ys = [h[0]["center"][1] for h in grp]
        gap = _ambiguous_gap(xs, ys, p)
        if gap > 0.0:
            reason = (f"帧中心分成两簇（差 {gap:.0f}px 且两边都够票数）—— 可能板子被暗纹/"
                      f"影子切成两块或有误检，不给数")
            print(f"[WARN] {reason}  各帧={[[int(a), int(b)] for a, b in zip(xs, ys)]}")
            return dict(fail, reason=reason, frame=_annotate(grp[0][1], grp[0][0]),
                        votes=len(grp), dropped=0,
                        spread_all=_spread(xs, ys),
                        frames=[[round(a, 1), round(b, 1)] for a, b in zip(xs, ys)])

        # ③ **MAD 离群剔除**：抖动偶尔会"跳一档"（个别帧误检），这种帧不能参与平均。
        #    判据 |v−中位| ≤ max(RECT_OUTLIER_MIN_PX, RECT_OUTLIER_K×1.4826×MAD)：
        #    抖得小就不剔（下限兜底），抖得野才剔。双峰已在②拦下，这里只管"少数几个跳档"。
        keep_x, keep_y = _inlier_mask(xs, p), _inlier_mask(ys, p)
        keep = [i for i in range(len(grp)) if keep_x[i] and keep_y[i]]
        dropped = len(grp) - len(keep)
        if len(keep) < RECT_MIN_VOTES:
            reason = (f"矩形地板抖动过大：{len(grp)} 帧里剔除 {dropped} 个离群帧后"
                      f"只剩 {len(keep)} 帧")
            print(f"[WARN] {reason}  原始中心={[[int(a), int(b)] for a, b in zip(xs, ys)]}")
            return dict(fail, reason=reason, frame=_annotate(grp[0][1], grp[0][0]),
                        votes=len(grp), inliers=len(keep), dropped=dropped,
                        spread_all=_spread(xs, ys))

        # ③ 内点取**均值**（比中位更省噪声；离群帧已在②剔掉），四角同法。
        kx = [xs[i] for i in keep]
        ky = [ys[i] for i in keep]
        center = (sum(kx) / len(kx), sum(ky) / len(ky))
        corners = []
        for i in range(4):
            cx = [grp[j][0]["corners"][i][0] for j in keep]
            cy = [grp[j][0]["corners"][i][1] for j in keep]
            corners.append((sum(cx) / len(cx), sum(cy) / len(cy)))
        spread = _spread(kx, ky)                     # 内点里的散布 = 这个数的可信度
        if spread > RECT_MAX_SPREAD_PX:
            print(f"[WARN] 矩形地板内点散布 {spread:.0f}px 超限（车在动？）")
            return dict(fail, reason=f"跨帧散布 {spread:.0f}px 超限（车在动？）",
                        frame=_annotate(grp[keep[0]][1], grp[keep[0]][0]),
                        votes=len(grp), inliers=len(keep), dropped=dropped,
                        spread=spread, spread_all=_spread(xs, ys),
                        frames=[[round(a, 1), round(b, 1)] for a, b in zip(xs, ys)])
        est = key[1]
        # 画在**最靠近最终中心的那一帧**上，并把框画成"平均后的框" —— 预览里看到的
        # 就是返回的那个数，不会出现"框和坐标对不上"。
        rep = min(keep, key=lambda j: (xs[j] - center[0]) ** 2 + (ys[j] - center[1]) ** 2)
        res = dict(success=True, reason="", center=center, corners=corners,
                   estimated=est, n_sides=grp[rep][0]["n_sides"],
                   path=key[0], votes=len(grp), inliers=len(keep), dropped=dropped,
                   spread=spread, spread_all=_spread(xs, ys),
                   frames=[[round(a, 1), round(b, 1)] for a, b in zip(xs, ys)],
                   area_frac=grp[rep][0]["area_frac"],
                   inner_v=grp[rep][0]["inner_v"])
        print("[INFO] 矩形地板识别成功: 中心=(%.1f,%.1f) %s %s %d帧(剔%d) 散布%.1fpx"
              % (center[0], center[1], "三边估" if est else "四边精确", key[0],
                 len(grp), dropped, spread))
        print("[INFO]   各帧中心=%s" % [[int(a), int(b)] for a, b in zip(xs, ys)])
        res["frame"] = _annotate(grp[rep][1], res)
        return res
    finally:
        release_camera(cap)     # 租约句柄 → 归还（保持打开）; 普通 cap → 真释放
