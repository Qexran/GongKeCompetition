# -*- coding: utf-8 -*-
"""主摄斜视二维码：先摸清"到底能不能解、卡在哪一步"。
只用本地有的 cv2.QRCodeDetector（没有 wechat/pyzbar），逐级排除：
  全分辨率 / Pi 的 960 降采样 / 各档降采样 / 中心裁剪 / 透视校正
"""
import glob
import cv2
import numpy as np

FRAMES = sorted(glob.glob("live_main_*.jpg"))


def try_decode(img, tag):
    """QRCodeDetector 多码 → 单码回退。返回 (ok, text)"""
    det = cv2.QRCodeDetector()
    try:
        ok, infos, points, _ = det.detectAndDecodeMulti(img)
        if ok and infos:
            good = [i.strip() for i in infos if i and i.strip()]
            if good:
                return True, good[0]
    except Exception as e:
        pass
    try:
        data, pts, _ = det.detectAndDecode(img)
        if data and data.strip():
            return True, data.strip()
    except Exception:
        pass
    return False, None


def resize_max(img, m):
    h, w = img.shape[:2]
    mx = max(h, w)
    if mx <= m:
        return img
    s = m / float(mx)
    return cv2.resize(img, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)


print("=" * 70)
print("A. 各档分辨率下 QRCodeDetector 的原始解码能力（不做任何预处理）")
print("=" * 70)
for m in (0, 1920, 1280, 960, 720, 540):
    res = []
    for f in FRAMES:
        img = cv2.imread(f)
        if m:
            img = resize_max(img, m)
        ok, txt = try_decode(img, f)
        res.append(txt if ok else "-")
    size = "全分辨率" if m == 0 else f"max={m}"
    print(f"  {size:>10}  " + "  ".join(f"{r:>5}" for r in res))

print()
print("=" * 70)
print("B. 灰度 / CLAHE / 二值化 —— 看预处理是否救得回来（全分辨率 + 960）")
print("=" * 70)


def preprocess_variants(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
    # 强光斑：先做一次大核顶帽/闭运算压高光再二值
    _, otsu = cv2.threshold(clahe, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    adap = cv2.adaptiveThreshold(clahe, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY, 51, 10)
    return {"gray": gray, "clahe": clahe, "otsu": otsu, "adaptive": adap}


for m in (0, 960):
    print(f"  --- max={m or '全'} ---")
    for var in ("gray", "clahe", "otsu", "adaptive"):
        res = []
        for f in FRAMES:
            img = cv2.imread(f)
            if m:
                img = resize_max(img, m)
            ok, txt = try_decode(preprocess_variants(img)[var], f)
            res.append(txt if ok else "-")
        print(f"    {var:>9}  " + "  ".join(f"{r:>5}" for r in res))

print()
print("=" * 70)
print("C. 找定位角 + 透视矫正后再解（复刻 Pi 的 _perspective_correct 思路）")
print("=" * 70)


def find_finder_centers(gray):
    th = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                               cv2.THRESH_BINARY, 51, 10)
    bw = (th > 0).astype("uint8")
    cnts, hier = cv2.findContours(cv2.bitwise_not(th), cv2.RETR_TREE,
                                  cv2.CHAIN_APPROX_SIMPLE)
    if hier is None:
        return []
    hier = hier[0]
    cands = []
    for i in range(len(cnts)):
        if hier[i][2] == -1:
            continue
        peri = cv2.arcLength(cnts[i], True)
        approx = cv2.approxPolyDP(cnts[i], 0.05 * peri, True)
        if len(approx) != 4:
            continue
        M = cv2.moments(cnts[i])
        if M["m00"] < 1e-6:
            continue
        cands.append((M["m10"] / M["m00"], M["m01"] / M["m00"], cv2.contourArea(cnts[i])))
    out = []
    for c in sorted(cands, key=lambda c: -c[2]):
        if all((c[0] - d[0]) ** 2 + (c[1] - d[1]) ** 2 > 225 for d in out):
            out.append(c)
    return out


for f in FRAMES[:2]:
    img = cv2.imread(f)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    c = find_finder_centers(gray)
    print(f"  {f}: 找到 {len(c)} 个候选定位角 {[(int(a),int(b)) for a,b,_ in c[:6]]}")
    # 检测器自己给的角点
    det = cv2.QRCodeDetector()
    try:
        ok, pts = det.detectMulti(gray)
        if ok:
            print(f"       detectMulti 命中 {len(pts)} 个码，首个四边形: {pts[0].reshape(-1,2).astype(int).tolist()}")
        else:
            ok2, pts2 = det.detect(gray)
            print(f"       detectMulti 未命中; detect={ok2}" + (f" 四边形 {pts2.reshape(-1,2).astype(int).tolist()}" if ok2 and pts2 is not None else ""))
    except Exception as e:
        print("       detect 异常:", e)
