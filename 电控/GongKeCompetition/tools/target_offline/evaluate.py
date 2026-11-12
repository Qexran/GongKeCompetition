# -*- coding: utf-8 -*-
"""第 1 步交付物：合成靶 → 五套圆心算法 → 误差报告。

    python evaluate.py            全量（约 40 个用例 × 5 方法）
    python evaluate.py --quick    抽稀版，改算法时快速看趋势
    python evaluate.py --save     额外存一张对照图到 out/cases.png

判据：**10 环半径 7.5mm、9 环 15mm**（命题图 7）。
误差一律先算像素、再乘真值的 mm_per_px 换算成**靶面上的毫米** ——
同一个算法在不同距离下像素误差不同，而"能不能打中"只认毫米。

⚠️ 合成图的真值圆心是**算出来的**（单应 K·[r1 r2 t] 把靶心投到像平面），
不是标的、也不是凑的，所以这里量出来的误差是**算法误差本身**，不含标注误差。
"""
import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

# Windows 控制台默认 GBK, 中文报告会变成乱码 —— 显式把 stdout 钉成 UTF-8
try:
    sys.stdout.reconfigure(encoding="utf-8")
except Exception:                                     # noqa: BLE001
    pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import detect   # noqa: E402
import synth    # noqa: E402

# 判据取**命题图 7 的环半径**（10 环 Ø15 / 9 环 Ø30），与合成靶画了什么无关 ——
# 现场样件上没印内环，但比赛打分是按环数的，所以这条严格线不能放松。
RING10_MM = 7.5
RING9_MM = 15.0
OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

METHODS = ["m_centroid", "m_hough", "m_outfit", "m_inner",
           "m_shared", "m_shared_sub", "m_both", "m_triple", "m_triple_q"]

# 画对照图用的颜色（BGR）。真值一律白色。
MCOLOR = {
    "m_centroid":   (0, 255, 255),
    "m_hough":      (255, 0, 255),
    "m_outfit":     (0, 160, 255),
    "m_inner":      (0, 255, 0),
    "m_shared":     (255, 255, 0),
    "m_shared_sub": (0, 0, 255),
    "m_both":       (255, 128, 0),
    "m_triple":     (128, 0, 128),
    "m_triple_q":   (0, 100, 0),
}
TRUTH_COLOR = (255, 255, 255)


# ==================== 用例集 ====================

def build_cases(quick=False):
    """→ [(组名, 参数)]。每组只覆盖关心的一项, 其余走默认（标称 1m 正对、红靶、浅灰背景）。"""
    C = [("标称 1m 正对", {})]

    dists = (1000.0, 2000.0) if quick else (500.0, 750.0, 1000.0, 1500.0, 2000.0)
    for d in dists:
        C.append(("距离 %gmm" % d, {"dist_mm": d}))

    for c in ("red", "green", "blue"):
        C.append(("颜色 %s" % c, {"color_name": c}))

    # 离轴：靶心不在画幅中心 → 透视 + 镜头边缘畸变（质心法在这里最容易露馅）
    for ox, oy in (((420.0, 0.0), (-480.0, 260.0)) if quick
                   else ((420.0, 0.0), (-480.0, 260.0), (900.0, -380.0))):
        C.append(("离轴 %.0f,%.0f px" % (ox, oy), {"offset_mm": (ox, oy)}))

    # 面内旋转：靶盘装歪。对称圆环的质心**不该**受它影响 —— 用来验证这个前提
    for r in (45.0,) if quick else (18.0, 45.0, 90.0):
        C.append(("面内旋转 %g°" % r, {"roll_deg": r}))

    # 靶盘不垂直于视线（车没停正 / 从斜侧打）—— 透视下"环"变成椭圆
    tilts = ((12.0, 0.0),) if quick else ((8.0, 0.0), (0.0, 10.0),
                                         (12.0, 8.0), (20.0, 14.0))
    for p, y in tilts:
        C.append(("倾角 pitch%g yaw%g" % (p, y),
                  {"pitch_deg": p, "yaw_deg": y}))

    # 印刷色深浅 / 曝光：外环色偏离标称值
    for v in (0.6, 1.45) if quick else (0.5, 0.7, 1.3, 1.6):
        C.append(("曝光/色深 x%.2f" % v, {"v_scale": v, "s_scale": min(1.0, v)}))

    # 背景与补光不均（现场补光把一侧打亮）
    for bg, gr in (((210, 210, 210), 0.0), ((110, 110, 110), 0.0), ((148, 148, 148), 55.0)):
        C.append(("背景 %d 梯度 %g" % (bg[0], gr), {"bg_bgr": bg, "grad": gr}))

    # 相机退化：离焦 / 噪声 / JPEG（UVC 出来就是压缩过的）
    for bs in (1.2,) if quick else (0.8, 1.5, 2.2):
        C.append(("离焦 σ=%g" % bs, {"blur_sigma": bs}))
    for ns in (5.0,) if quick else (3.0, 8.0, 14.0):
        C.append(("噪声 σ=%g" % ns, {"noise_sigma": ns}))
    for q in (75,) if quick else (55, 75, 92):
        C.append(("JPEG q=%d" % q, {"jpeg_q": q}))

    # ==================== 下面这组才是**真正区分方法**的地方 ====================
    # 上面那些（干净、完整、正对）五套方法都能过 —— 那说明不了选型。
    # 能分开它们的只有三类：畸变把圆挤歪、局部掉色让色块不再对称、盘面被挡住一块。

    # ① 镜头径向畸变（70° 视场必然有）。离轴越远"挤"得越狠, 而质心是按面积加权的,
    #    于是偏移**方向固定**、多帧平均消不掉 —— 这是质心法最大的系统性风险。
    for k1 in (-0.12,) if quick else (-0.08, -0.16, -0.28):
        C.append(("畸变 k1=%g 离轴" % k1,
                  {"k1": k1, "offset_mm": (420.0, 200.0)}))

    # ② 阴影压暗外环的一段 → 那段掉出色带, 色块不再对称（同心拟合只当"少了几个点"）
    for f, x0, x1 in (((0.42,), 0.28, 0.47),) if quick else (((0.42,), 0.28, 0.47),
                                                              ((0.30,), 0.22, 0.52)):
        C.append(("阴影盖外环 %.0f%%" % ((x1 - x0) * 100),
                  {"shadow": ((x0, x1, 0.0, 1.0), f[0])}))
    C.append(("阴影+畸变", {"shadow": ((0.28, 0.47, 0.0, 1.0), 0.42),
                            "k1": -0.16, "offset_mm": (300.0, 150.0)}))

    # ③ 遮挡：机械臂自己挡、靶架立柱、光电接收器
    C.append(("遮挡左 1/4 盘面", {"occlude": [(893, 470, 34, 140)]}))
    C.append(("遮挡左下角块", {"occlude": [(840, 560, 150, 120)]}))
    C.append(("遮挡+畸变", {"occlude": [(893, 470, 34, 140)], "k1": -0.12,
                           "offset_mm": (300.0, -180.0)}))

    # ④ 靶盘被画幅边缘裁掉一块（臂伸长时很常见）
    C.append(("出画 40%", {"offset_mm": (690.0, 0.0), "k1": -0.12}))

    # ⑤ 镜面高光把外环顶出色带（V 过饱和 → 饱和度反降）
    C.append(("高光压外环", {"specular": [(1030.0, 480.0, 26.0)]}))

    # ==================== ⑥ 照现场实测补的两类 ====================
    # 手工/3D 打印样件的外缘是**手工贴的波浪边**（实测残差 std ≈ 半径的 4.6%）。
    # 这条专门打「外圆拟合」和「质心法」这两个依赖外边缘的方法。
    for lp in (0.03,) if quick else (0.03, 0.05, 0.08):
        C.append(("波浪外缘 %.0f%%" % (lp * 100), {"lumpy": lp, "tgt_seed": 7}))

    # 现场背景是**一整面蓝色柜子**，蓝靶和它同色 —— 纯色阈值会被大块同色区域骗走
    cab = [("blue", (0.0, 0.0, 1.0, 0.45), 0.55)]
    C.append(("同色大块干扰", {"color_name": "blue", "distractors": cab}))
    C.append(("干扰+波浪", {"color_name": "blue", "distractors": cab,
                            "lumpy": 0.05, "tgt_seed": 7}))
    C.append(("干扰+近+波浪", {"color_name": "blue", "distractors": cab,
                               "lumpy": 0.05, "tgt_seed": 3, "dist_mm": 600.0}))

    # ==================== ⑦ 三靶并排 + 同色大块干扰 ====================
    # 现场就是三个靶并排站在架子上。跨色尺寸一致性判据**只有在这种场面才谈得上**，
    # 所以用例必须造得出来。主目标取 blue —— 因为骗人的那个干扰物也是蓝的。
    trio = [("blue",  (0.0, 0.0),    1200.0),
            ("green", (-500.0, 0.0), 1200.0),
            ("red",   (500.0, 0.0),  1200.0)]
    C.append(("三靶并排", {"color_name": "blue", "targets": trio}))
    C.append(("三靶+同色柜子", {"color_name": "blue", "targets": trio,
                                "distractors": cab}))
    C.append(("三靶+柜子+波浪", {"color_name": "blue", "targets": trio,
                                 "distractors": cab, "lumpy": 0.05, "tgt_seed": 7}))
    # 只看到一个靶（另外两个出画）—— 尺寸基准没了，必须靠内孔兜底
    C.append(("单靶无可比(近)", {"color_name": "blue", "dist_mm": 500.0,
                                 "distractors": cab}))

    # ==================== ⑧ ROI 裁切（探子提的"先裁到常见范围"） ====================
    # ⚠️ ROI 与「跨色尺寸一致性」是**互相拉扯**的：窗口收得越紧，同色的干扰越进不来，
    #    但另外两个靶也一起被切掉了、尺寸基准就没了。所以窗口形状要按现场排布定 ——
    #    三个靶是**水平并排**、柜子在**上方背景**，那就该用「全宽 + 下移的横带」，
    #    而不是居中正方形。
    C.append(("三靶+柜子+ROI横带", {"color_name": "blue", "targets": trio,
                                    "distractors": cab,
                                    "roi": (0.0, 0.40, 1.0, 0.98)}))
    C.append(("单靶+柜子+ROI窗口", {"color_name": "blue", "dist_mm": 500.0,
                                    "distractors": cab,
                                    "roi": (0.20, 0.20, 0.80, 0.90)}))

    # ==================== ⑨ 同色 + 圆形 + 带孔 的干扰物 ====================
    # 这一类**能骗过"内孔"判据**（它自己也有孔），是「跨色尺寸一致性」唯一不可替代的地方。
    # 蓝桶口/圆牌尺寸做成靶盘的 ~2.6 倍 —— 比 tol=2.0 大, 该被尺寸判据排掉。
    ring_big = [("blue", 0.30, 0.62, 0.115, 0.030, 0.75)]
    C.append(("三靶+同色圆环干扰", {"color_name": "blue", "targets": trio,
                                    "distractor_rings": ring_big}))
    C.append(("单靶+同色圆环干扰", {"color_name": "blue", "dist_mm": 1200.0,
                                    "distractor_rings": ring_big}))

    # ==================== ⑩ 干扰物比靶大（现场帧实测的失效模式）====================
    # 真实一帧里：红靶旁边是**木椅子**（r=311 的红块）、蓝靶背后是**蓝柜子**（r=448）。
    # 两个都盖过真靶（r≈62/79/65）。当时的「每色取最大 + 中位数剔离群」被它们带偏，
    # 真靶反倒被当成离群剔掉 → 红靶消失、蓝靶变成柜子一块。这一组专门盯这个。
    trio_r = [("red",   (500.0, 0.0),  1200.0),
              ("green", (-500.0, 0.0), 1200.0),
              ("blue",  (0.0, 0.0),    1200.0)]
    big = [("red",  (0.05, 0.05, 0.35, 0.40), 0.65),    # 木椅子那种大红块
           ("blue", (0.60, 0.05, 1.00, 0.42), 0.55)]    # 蓝柜子
    C.append(("三靶+大红块+蓝柜子", {"color_name": "red", "targets": trio_r,
                                     "distractors": big}))
    C.append(("三靶+大红块+蓝柜子+波浪", {"color_name": "red", "targets": trio_r,
                                          "distractors": big, "lumpy": 0.05, "tgt_seed": 7}))

    # 组合恶劣：远 + 斜 + 暗 + 糊 + 畸变 + 阴影 + 压缩（现场最糟的一帧大概长这样）
    C.append(("组合恶劣", {"dist_mm": 1800.0, "pitch_deg": 10.0, "yaw_deg": 8.0,
                           "v_scale": 0.7, "blur_sigma": 1.0, "noise_sigma": 5.0,
                           "jpeg_q": 75, "grad": 40.0, "k1": -0.14,
                           "shadow": ((0.26, 0.48, 0.0, 1.0), 0.45)}))
    return C


# ==================== 跑 ====================

def run_one(params, seed=0):
    p = dict(params)
    p.setdefault("rng", np.random.default_rng(seed))
    roi = p.pop("roi", None)          # roi 是**检测端**的参数, 不是渲染端的
    img, truth = synth.make_case(**p)
    res = {}
    for m in METHODS:
        t0 = time.perf_counter()
        try:
            r = detect.detect(img, p.get("color_name", "red"), m, roi=roi)
        except Exception as exc:                      # noqa: BLE001
            r, err = None, str(exc)
        else:
            err = ""
        dt = (time.perf_counter() - t0) * 1000.0
        if r is None:
            res[m] = {"miss": True, "ms": dt, "err": err}
        else:
            epx = math.hypot(r["cx"] - truth["cx"], r["cy"] - truth["cy"])
            res[m] = {"miss": False, "cx": r["cx"], "cy": r["cy"],
                      "err_px": epx, "err_mm": epx * truth["mm_per_px"], "ms": dt}
    return img, truth, res


def stats(vals):
    if not vals:
        return None
    a = np.asarray(vals, float)
    return {"n": len(a), "med": float(np.median(a)),
            "p95": float(np.percentile(a, 95)), "max": float(a.max())}


def fmt_cell(r):
    """一格里就一个数：该用例的**毫米**误差（超出 10 环的标 *）。没检出写「未检出」。"""
    if r.get("miss"):
        return "%9s" % "未检出"
    return "%8.2f%s" % (r["err_mm"], "*" if r["err_mm"] > RING10_MM else " ")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--save", action="store_true")
    args = ap.parse_args()

    cases = build_cases(args.quick)
    print("=== 靶心定位离线评估 ===  用例 %d 个 × %d 方法   判据: 10环 %.1fmm / 9环 %.1fmm"
          % (len(cases), len(METHODS), RING10_MM, RING9_MM))
    hdr = "%-22s" % "组" + "".join("%9s" % m.replace("m_shared_sub", "shared_sub")
                                   .replace("m_", "") for m in METHODS)
    print("\n格子 = 该用例的**毫米**误差；带 * = 超出 10 环半径 %.1fmm\n" % RING10_MM)
    print(hdr)
    print("-" * len(hdr))

    agg = {m: {"mm": [], "px": [], "ms": [], "miss": 0} for m in METHODS}
    montage_rows = []

    for ci, (name, params) in enumerate(cases):
        # 种子按**下标**给, 不用 hash(name) —— Python 的字符串哈希每进程随机化,
        # 用 hash 会让「噪声/JPEG 用例」每次跑出来的数都不一样, 测量数据不可复现。
        img, truth, res = run_one(params, seed=ci * 7 + 1)
        print("%-22s" % name + "".join(fmt_cell(res[m]) for m in METHODS))
        for m in METHODS:
            r = res[m]
            if r.get("miss"):
                agg[m]["miss"] += 1
            else:
                agg[m]["mm"].append(r["err_mm"])
                agg[m]["px"].append(r["err_px"])
            agg[m]["ms"].append(r["ms"])
        montage_rows.append((name, img, truth, res))

    n = len(cases)
    print("-" * len(hdr))
    print("\n=== 汇总（%d 个用例） ===" % n)
    print("%-14s %7s %7s %7s %7s %7s %7s %7s"
          % ("方法", "中位mm", "p95mm", "最大mm", "≤7.5mm", "≤15mm", "漏检", "均耗时"))
    for m in METHODS:
        a = agg[m]
        st = stats(a["mm"])
        if st is None:
            print("%-14s  全部漏检" % m)
            continue
        ok10 = sum(1 for v in a["mm"] if v <= RING10_MM)
        ok9 = sum(1 for v in a["mm"] if v <= RING9_MM)
        print("%-14s %7.2f %7.2f %7.2f %6.0f%% %6.0f%% %7d %6.0fms"
              % (m, st["med"], st["p95"], st["max"],
                 100.0 * ok10 / n, 100.0 * ok9 / n, a["miss"],
                 float(np.mean(a["ms"]))))

    if args.save:
        save_montage(montage_rows)
    return 0


def save_montage(rows, tile_w=520, cols=3):
    """每个用例画一层：真值白圈 + 各方法的圆心（按 MCOLOR）。存到 out/cases.png。

    ⚠️ 标注文字只能用 ASCII —— cv2.putText 的内置字体画不了中文（和「采集」Tab 那条
    「预览是单帧轮询+canvas」是同一类约束）。所以这里用组序号，图例打在控制台。
    """
    os.makedirs(OUT_DIR, exist_ok=True)
    tiles = []
    for idx, (name, img, truth, res) in enumerate(rows):
        vis = img.copy()
        tx, ty = int(round(truth["cx"])), int(round(truth["cy"]))
        cv2.drawMarker(vis, (tx, ty), TRUTH_COLOR, cv2.MARKER_CROSS, 34, 2)
        cv2.circle(vis, (tx, ty), int(round(truth["r_px"])), TRUTH_COLOR, 2)
        for m in METHODS:
            r = res[m]
            if r.get("miss"):
                continue
            cv2.drawMarker(vis, (int(round(r["cx"])), int(round(r["cy"]))),
                           MCOLOR[m], cv2.MARKER_TILTED_CROSS, 22, 2)
        s = tile_w / float(vis.shape[1])
        vis = cv2.resize(vis, (tile_w, int(round(vis.shape[0] * s))))
        cv2.putText(vis, "#%d" % idx, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 0, 0), 5)
        cv2.putText(vis, "#%d" % idx, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (255, 255, 255), 2)
        tiles.append(vis)

    h = max(t.shape[0] for t in tiles)
    tiles = [cv2.copyMakeBorder(t, 0, h - t.shape[0], 0, 0,
                                cv2.BORDER_CONSTANT, value=(30, 30, 30)) for t in tiles]
    rows_img = []
    for i in range(0, len(tiles), cols):
        chunk = tiles[i:i + cols]
        while len(chunk) < cols:
            chunk.append(np.full_like(tiles[0], 30))
        rows_img.append(np.hstack(chunk))
    out = np.vstack(rows_img)
    path = os.path.join(OUT_DIR, "cases.png")
    cv2.imwrite(path, out)
    print("\n对照图 → %s" % path)
    print("  白十字+白圈 = 真值；黄=centroid 品红=hough 橙=outfit 绿=shared 红=shared_sub")
    for idx, (name, _i, _t, _r) in enumerate(rows):
        print("  #%-3d %s" % (idx, name))


if __name__ == "__main__":
    sys.exit(main())
