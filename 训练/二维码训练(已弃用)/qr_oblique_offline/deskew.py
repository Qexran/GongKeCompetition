# -*- coding: utf-8 -*-
"""核心实验：用三个定位角做透视矫正（复刻 tasks/_qr.py::_perspective_correct 的
全部逻辑，逐行照抄），把斜视二维码掰成正视图，再交给解码器。

如果这条能解出来 → 结论就是「解码器没坏，是**没喂给它正视图**」，
修复 = 在解码链前面加一级 deskew（对 wechat 也要开）。
"""
import glob
import itertools
import cv2
import numpy as np

FRAMES = sorted(glob.glob("live_main_*.jpg"))


# ---------------- 以下四个函数逐行照抄 tasks/_qr.py ----------------
def _check_finder_ratio(bw, cx, cy, half=45):
    H, W = bw.shape

    def runs(dx, dy):
        prev = None
        ln = 0
        rs = []
        for t in range(-half, half + 1):
            x = int(cx + dx * t)
            y = int(cy + dy * t)
            if x < 0 or y < 0 or x >= W or y >= H:
                continue
            v = int(bw[y, x])
            if prev is None:
                prev, ln = v, 1
            elif v == prev:
                ln += 1
            else:
                rs.append(ln)
                prev, ln = v, 1
        if ln:
            rs.append(ln)
        return rs

    def ok(rs):
        for i in range(len(rs) - 4):
            seg = rs[i:i + 5]
            if any(s == 0 for s in seg):
                continue
            total = sum(seg)
            ratio = [s / (total / 7.0) for s in seg]
            if sum(abs(ratio[j] - [1, 1, 3, 1, 1][j]) for j in range(5)) < 2.5:
                return True
        return False

    return any(ok(runs(dx, dy)) for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)))


def _find_finder_centers(gray):
    th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY, 51, 10)
    bw = (th > 0).astype("uint8")
    contours, hierarchy = cv2.findContours(cv2.bitwise_not(th), cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return []
    hierarchy = hierarchy[0]
    candidates = []
    for i in range(len(contours)):
        if hierarchy[i][2] == -1:
            continue
        peri = cv2.arcLength(contours[i], True)
        approx = cv2.approxPolyDP(contours[i], 0.05 * peri, True)
        if len(approx) != 4:
            continue
        M = cv2.moments(contours[i])
        if M["m00"] < 1e-6:
            continue
        cx = M["m10"] / M["m00"]
        cy = M["m01"] / M["m00"]
        if not _check_finder_ratio(bw, cx, cy):
            continue
        candidates.append((cx, cy, cv2.contourArea(contours[i])))
    dedup = []
    for c in sorted(candidates, key=lambda c: -c[2]):
        if all((c[0] - d[0]) ** 2 + (c[1] - d[1]) ** 2 > 225 for d in dedup):
            dedup.append(c)
    return dedup


def _finder_centers_to_quad(centers):
    if len(centers) < 3:
        return None
    pts = [np.array([c[0], c[1]], dtype=np.float32) for c in centers]
    best = None
    best_err = float("inf")
    for i, j, k in itertools.combinations(range(len(pts)), 3):
        a, b, c = sorted([np.linalg.norm(pts[i] - pts[j]),
                          np.linalg.norm(pts[j] - pts[k]),
                          np.linalg.norm(pts[k] - pts[i])])
        if a < 20:
            continue
        err = abs(b - a) / a + abs(c - a * 1.414) / a
        if err < best_err:
            best_err = err
            best = (i, j, k)
    if best is None or best_err > 1.5:
        return None
    i, j, k = best
    tri = [pts[i], pts[j], pts[k]]
    d = [np.linalg.norm(tri[0] - tri[1]), np.linalg.norm(tri[1] - tri[2]),
         np.linalg.norm(tri[2] - tri[0])]
    hyp = max(range(3), key=lambda m: d[m])
    tl = tri[(hyp + 2) % 3]
    other = [tri[m] for m in range(3) if m != (hyp + 2) % 3]
    tr, bl = other[0], other[1]
    br = tr + bl - tl
    return np.array([tl, tr, br, bl], dtype=np.float32)


def _expand_quad(quad, factor=1.5):
    c = quad.mean(axis=0)
    return (quad - c) * factor + c


def _order_points(pts):
    rect = np.zeros((4, 2), dtype=np.float32)
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    diff = np.diff(pts, axis=1).ravel()
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    return rect


def _perspective_correct(frame):
    gray = frame if frame.ndim == 2 else cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    quad = _finder_centers_to_quad(_find_finder_centers(gray))
    if quad is None:
        return frame
    quad = _expand_quad(quad)
    pts = _order_points(quad)
    tl, tr, br, bl = pts
    w = int(max(np.linalg.norm(tr - tl), np.linalg.norm(br - bl)))
    h = int(max(np.linalg.norm(bl - tl), np.linalg.norm(br - tr)))
    if w < 20 or h < 20:
        return frame
    dst = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], dtype=np.float32)
    M = cv2.getPerspectiveTransform(pts, dst)
    return cv2.warpPerspective(frame, M, (w, h))


# ---------------- 实验 ----------------
def try_decode(img):
    det = cv2.QRCodeDetector()
    try:
        ok, infos, points, _ = det.detectAndDecodeMulti(img)
        if ok and infos:
            good = [i.strip() for i in infos if i and i.strip()]
            if good:
                return good[0]
    except Exception:
        pass
    try:
        data, pts, _ = det.detectAndDecode(img)
        if data and data.strip():
            return data.strip()
    except Exception:
        pass
    return None


print("=" * 72)
print("D. deskew（三定位角透视矫正）之后再解 —— 主摄斜视")
print("=" * 72)
for f in FRAMES:
    img = cv2.imread(f)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    centers = _find_finder_centers(gray)
    quad = _finder_centers_to_quad(centers)
    tag = "无三元组" if quad is None else "有三元组"
    rect = _perspective_correct(img)
    changed = rect is not img and rect.shape != img.shape
    out = try_decode(rect)
    h, w = rect.shape[:2]
    print(f"  {f}: 定位角候选={len(centers):2d} {tag}  矫正后={w}x{h}"
          f" ({'已 warp' if changed else '未变'})  →  {out if out else '-'}")
    if out:
        cv2.imwrite(f.replace('.jpg', '_rectified.png'), rect)

print()
print("=" * 72)
print("E. 矫正图上再叠各档预处理（灰度/CLAHE/otsu/自适应）")
print("=" * 72)
for f in FRAMES:
    img = cv2.imread(f)
    rect = _perspective_correct(img)
    if rect is img:
        print(f"  {f}: 未矫正，跳过")
        continue
    g = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY)
    variants = {
        "gray": g,
        "clahe": cv2.createCLAHE(2.0, (8, 8)).apply(g),
        "otsu": cv2.threshold(cv2.createCLAHE(2.0, (8, 8)).apply(g), 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1],
    }
    res = {k: (try_decode(v) or "-") for k, v in variants.items()}
    print(f"  {f}: " + "  ".join(f"{k}={v}" for k, v in res.items()))
