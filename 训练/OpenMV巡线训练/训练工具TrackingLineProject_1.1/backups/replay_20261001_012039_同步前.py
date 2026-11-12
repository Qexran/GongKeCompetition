# -*- coding: utf-8 -*-
"""
巡线算法离线回放测试台
======================
输入: OpenMV IDE 录制的运行视频（320x240, 相机帧率内容存于 30fps 容器, 隔帧重复）
做法: 去重出真实帧序列 → 逐帧喂给**与工作主副本同一份**的算法代码（常量与纯函数
      直接从主文件 exec 加载, 单一事实源）→ 输出相机同款调试行 / CSV / 分段时间线 /
      异常报告（丢线期内标签翻转、offset 跳变）/ 抽样拼板图。
不模拟: lens_corr（IDE 录的已是校正后画面）、串口/拍照/绘制副作用。
已知偏差: H.264 压缩会稍微移动颜色绝对值; 算法是帧内自参考的(距离剖面/参考色),
          逻辑稳定性结论不受影响, 绝对 margin 数值与相机实测会有小差。

用法:
  python replay.py                       # 跑两个视频, 报告 + 拼板
  python replay.py --video ../校赛地图运行视频/校赛地图1.mp4
  python replay.py --set DELTA_GATE=20   # 参数扫掠(不改主文件, 仅本次运行覆盖)
"""
import cv2
import numpy as np
import os
import sys
import math
import argparse
import csv

HERE = os.path.dirname(os.path.abspath(__file__))
MAIN_FILE = os.path.join(HERE, "..", "main.py")

# ---------------- 从主文件加载常量与纯函数（单一事实源） ----------------

def load_from_main():
    with open(MAIN_FILE, encoding="utf-8") as f:
        src = f.read().splitlines(keepends=True)

    def region(start_pred, end_pred):
        s = next(i for i, l in enumerate(src) if start_pred(l))
        e = next(i for i, l in enumerate(src) if i > s and end_pred(l))
        return "".join(src[s:e])

    ns = {"math": math}
    # 常量三段: 测量区+判据+出量源+滤波(到 DRAW 前) / 控制参数 / 派生几何
    exec(region(lambda l: l.startswith("FRAME_W, FRAME_H"),
                lambda l: l.startswith("DRAW =")), ns)
    exec(region(lambda l: l.startswith("CTRL_X"),
                lambda l: l.startswith("# ---------------- 串口")), ns)
    exec(region(lambda l: l.startswith("# 派生几何"),
                lambda l: l.startswith("# ==================== 测量核心")), ns)
    # 纯函数两段: lsq(到 col_profile 前) + cross_row(到 绘制 前, 含丢线分类)
    exec(region(lambda l: l.startswith("def lsq"),
                lambda l: l.startswith("def col_profile")), ns)
    exec(region(lambda l: l.startswith("def cross_row"),
                lambda l: l.startswith("# ---------------- 绘制")), ns)
    return ns


# ---------------- PC 侧图像 → LAB 统计 ----------------

def make_lab_converter():
    # sRGB → CIELAB, 口径同 OpenMV: L 0~100, a/b -128~127
    def to_lab(rgb_u8):
        a = rgb_u8.astype(np.float32) / 255.0
        lin = np.where(a <= 0.04045, a / 12.92, ((a + 0.055) / 1.055) ** 2.4)
        M = np.array([[0.4124564, 0.3575761, 0.1804375],
                      [0.2126729, 0.7151522, 0.0721750],
                      [0.0193339, 0.1191920, 0.9503041]], dtype=np.float32)
        xyz = lin @ M.T / np.array([0.95047, 1.0, 1.08883], dtype=np.float32)
        f = np.where(xyz > 0.008856, np.cbrt(xyz), 7.787 * xyz + 16.0 / 116.0)
        L = 116 * f[..., 1] - 16
        A = 500 * (f[..., 0] - f[..., 1])
        B = 200 * (f[..., 1] - f[..., 2])
        return L, A, B
    return to_lab


# ---------------- 帧提取与去重 ----------------

def load_frames(path, dedup_thr=0.7):
    cap = cv2.VideoCapture(path)
    frames = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        frames.append(fr)
    cap.release()
    if not frames:
        raise SystemExit(f"读不到帧: {path}")
    kept, last = [], None
    for fr in frames:
        if last is None or np.abs(fr.astype(np.int16) - last).mean() > dedup_thr:
            kept.append(fr)
            last = fr
    return frames, kept


# ---------------- 主循环复刻（统计量用 numpy, 逻辑与主文件逐句对齐） ----------------

class Replay:
    def __init__(self, ns, to_lab):
        self.ns = ns
        self.to_lab = to_lab
        g = ns
        self.state = dict(
            last_offset=0, last_angle=0, last_row_ctrl=None, base_off=None,
            filt_buf_o=[], filt_buf_a=[], filt_o=0.0, filt_a=0.0, filt_init=True,
        )
        g["ref_bnd"] = None
        g["ref_road"] = None

    def roi_mean(self, lab_ch, x0, y0, w, h):
        return float(lab_ch[y0:y0 + h, x0:x0 + w].mean())

    def roi_median(self, lab_ch, x0, y0, w, h):
        return float(np.median(lab_ch[y0:y0 + h, x0:x0 + w]))

    def col_profile(self, lab, x):
        g = self.ns
        L, A, B = lab
        out = []
        x0 = x - g["COL_W"] // 2
        for j in range(g["NSTRIP"]):
            y0 = g["WIN_Y"][0] + j * g["STRIP"]
            out.append((y0 + g["STRIP"] / 2.0,
                        self.roi_mean(A, x0, y0, g["COL_W"], g["STRIP"]),
                        self.roi_mean(B, x0, y0, g["COL_W"], g["STRIP"]),
                        self.roi_mean(L, x0, y0, g["COL_W"], g["STRIP"])))
        return out

    def step(self, frame_bgr):
        """喂一帧(去重后的), 返回 record dict。逻辑对齐主文件第 1~5 步 + classify。"""
        g = self.ns
        st = self.state
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        lab = self.to_lab(rgb)

        valid = False
        offset, angle = st["last_offset"], st["last_angle"]
        state_v = 4
        src_ch = "-"

        # ---- 1) 两侧色心（median 口径同主文件）----
        tx, ty, tw, th = g["TOP_BAND"]
        bx, by, bw, bh = g["BOT_BAND"]
        L, A, B = lab
        a_up = self.roi_median(A, tx, ty, tw, th)
        b_up = self.roi_median(B, tx, ty, tw, th)
        a_dn = self.roi_median(A, bx, by, bw, bh)
        b_dn = self.roi_median(B, bx, by, bw, bh)
        margin = max(abs(a_up - a_dn), abs(b_up - b_dn))
        ch = 'a' if abs(a_up - a_dn) >= abs(b_up - b_dn) else 'b'
        enough = margin >= g["MARGIN_MIN"]

        # ---- 3) 逐列距离剖面 → 交界行 ----
        pts, bpts = [], []
        for i in range(g["NCOL"]):
            x = int(g["WIN_X"][0] + (g["WIN_X"][1] - g["WIN_X"][0]) * (i + 0.5) / g["NCOL"])
            prof = self.col_profile(lab, x)
            ta = (prof[0][1] + prof[1][1] + prof[2][1]) / 3.0
            tb = (prof[0][2] + prof[1][2] + prof[2][2]) / 3.0
            ba = (prof[-1][1] + prof[-2][1] + prof[-3][1]) / 3.0
            bb = (prof[-1][2] + prof[-2][2] + prof[-3][2]) / 3.0
            da, db = ba - ta, bb - tb
            d2 = da * da + db * db
            f = [((p[1] - ta) ** 2 + (p[2] - tb) ** 2)
                 - ((p[1] - ba) ** 2 + (p[2] - bb) ** 2) for p in prof]
            row = g["cross_row"]([(prof[j][0], f[j]) for j in range(g["NSTRIP"])], 0.0,
                                 g["CROSS_MIN_SWING_FRAC"] * max(2.0 * d2, margin * margin))
            if row is not None:
                r2 = g["refine_local"](prof, row)
                if r2 is not None:
                    row = r2
            if row is not None and g["WIN_Y"][0] <= row <= g["WIN_Y"][1]:
                pts.append((x, row))
                if g["CHECK_BLACK"]:
                    brow = g["black_row"](prof, row)
                    if brow is not None and g["WIN_Y"][0] <= brow <= g["WIN_Y"][1]:
                        bpts.append((x, brow))

        # ---- 4) 拟合 ----
        fa = g["robust_fit"](pts)
        fb = g["robust_fit"](bpts) if g["CHECK_BLACK"] else None

        # ---- 5) 出量 ----
        if fa and enough:
            if g["USE_BLACK_PRIMARY"] and fb is not None and fb[2] <= g["BLACK_RMS_MAX"]:
                fit, src_pts, src_ch = fb, bpts, 'B'
            else:
                fit, src_pts, src_ch = fa, pts, 'C'
            is_curve = fit[2] > g["CURVE_RMS_PX"]
            if is_curve:
                near = [p for p in src_pts
                        if p[1] > g["NEAR_HALF_ROW"]
                        and abs(p[1] - (fit[0] * p[0] + fit[1])) <= g["RANSAC_TOL_PX"]]
                if len(near) >= 4:
                    fn = g["lsq"](near)
                    if fn is not None and fn[2] < fit[2]:
                        fit = (fn[0], fn[1], fn[2], len(near), len(src_pts) - len(near))
            m, c, e, nk, nd = fit
            lat_span = g["FRAME_H"]
            row_ctrl = m * g["CTRL_X"] + c
            if row_ctrl < g["WIN_Y"][0]:
                row_ctrl = g["WIN_Y"][0]
            elif row_ctrl > g["WIN_Y"][1]:
                row_ctrl = g["WIN_Y"][1]
            error_px = (row_ctrl - lat_span / 2.0) - g["TARGET_OFFSET"]
            raw_off = max(-100, min(100, int(error_px / lat_span * 2 * g["ERR_GAIN"])))
            raw_ang = max(-900, min(900, int(math.degrees(math.atan(m)) * 10)))

            fb_o, fb_a = st["filt_buf_o"], st["filt_buf_a"]
            fb_o.append(raw_off)
            fb_a.append(raw_ang)
            if len(fb_o) > g["FILT_MED_N"]:
                del fb_o[0]
                del fb_a[0]
            med_o = sorted(fb_o)[len(fb_o) // 2]
            med_a = sorted(fb_a)[len(fb_a) // 2]
            if st["filt_init"]:
                st["filt_o"], st["filt_a"] = float(med_o), float(med_a)
                st["filt_init"] = False
            else:
                st["filt_o"] += g["FILT_EMA_A"] * (med_o - st["filt_o"])
                st["filt_a"] += g["FILT_EMA_A"] * (med_a - st["filt_a"])
            offset, angle = int(st["filt_o"]), int(st["filt_a"])

            valid = True
            state_v = 1 if is_curve else 0
            if g["REF_SAFE_ROW"][0] <= row_ctrl <= g["REF_SAFE_ROW"][1]:
                g["ref_bnd"] = g["ema_ref"](g["ref_bnd"], (a_up, b_up))
                g["ref_road"] = g["ema_ref"](g["ref_road"], (a_dn, b_dn))
            if state_v == 0:
                if st["base_off"] is None:
                    st["base_off"] = float(offset)
                elif abs(offset - st["base_off"]) <= g["BASE_GATE"]:
                    st["base_off"] += g["BASE_RATE"] * (offset - st["base_off"])
            st["last_row_ctrl"] = row_ctrl
            st["last_offset"], st["last_angle"] = offset, angle
        else:
            g["last_offset"] = st["last_offset"]      # classify_lost 读全局
            g["base_off"] = st["base_off"]
            state_v = g["classify_lost"](pts, a_up, b_up, a_dn, b_dn)
            st["lost_near"] = g["lost_near"]

        d = (offset if valid else st["last_offset"]) - st["base_off"] if st["base_off"] is not None else None
        return dict(valid=1 if valid else 0, offset=offset, angle=angle, state=state_v,
                    margin=margin, npts=len(pts), nk=fa[3] if fa else 0,
                    nd=fa[4] if fa else 0, src=src_ch, base=st["base_off"],
                    delta=d, near=st.get("lost_near", "?"), pts=pts,
                    fit=fa, row_ctrl=st["last_row_ctrl"])


# ---------------- 报告 ----------------

LABEL = {0: "直线", 1: "路口弧", 2: "压灰区", 3: "压黄区", 4: "不明"}


def analyze(name, recs, fps_content):
    print(f"\n================ {name} | 内容帧 {len(recs)} 帧 = {len(recs)/fps_content:.1f}s ================")
    # 分段时间线
    segs = []
    for i, r in enumerate(recs):
        key = (r["valid"], r["state"])
        if segs and segs[-1][0] == key:
            segs[-1][2] = i
        else:
            segs.append([key, i, i])
    print("-- 分段时间线 (起-止 帧: 标签, off 均值) --")
    for (v, s), a, b in segs:
        offs = [recs[i]["offset"] for i in range(a, b + 1)]
        tag = LABEL[s] if v == 0 else (LABEL[s] + "(有效)")
        print(f"  #{a:4d}-{b:4d}: [{tag}]  off均值={np.mean(offs):+6.1f}  长{b-a+1}")
    # 异常检测
    print("-- 异常检测 --")
    bad = 0
    # ① 丢线段内标签翻转（历史 bug 类）
    run = []
    for i, r in enumerate(recs):
        if r["valid"] == 0:
            run.append(i)
        else:
            if len(run) >= 3:
                states = {recs[j]["state"] for j in run if recs[j]["state"] != 4}
                if len(states) > 1:
                    print(f"  !! 丢线段 #{run[0]}-#{run[-1]} 内标签翻转: {sorted(states)}")
                    bad += 1
            run = []
    if len(run) >= 3:
        states = {recs[j]["state"] for j in run if recs[j]["state"] != 4}
        if len(states) > 1:
            print(f"  !! 丢线段 #{run[0]}-#{run[-1]} 内标签翻转: {sorted(states)}")
            bad += 1
    # ② 有效帧间 offset 跳变
    pv = [(i, r) for i, r in enumerate(recs) if r["valid"] == 1]
    for (i1, r1), (i2, r2) in zip(pv, pv[1:]):
        if abs(r2["offset"] - r1["offset"]) > 25 and i2 - i1 <= 2:
            print(f"  !! 有效帧跳变 #{i1}->{i2}: off {r1['offset']} -> {r2['offset']}")
            bad += 1
    # ③ 丢线但 st=4 连续过长（找线策略会失效）
    for (v, s), a, b in segs:
        if v == 0 and s == 4 and b - a + 1 >= 10:
            print(f"  ?? 长不明段 #{a}-#{b} ({b-a+1}帧) —— 核对该处画面")
    if bad == 0:
        print("  (无标签翻转 / 无 offset 跳变)")
    # 汇总
    nv = sum(1 for r in recs if r["valid"])
    from collections import Counter
    cnt = Counter((r["valid"], r["state"]) for r in recs)
    print(f"-- 汇总: 有效率 {nv}/{len(recs)} = {nv/len(recs)*100:.0f}% | " +
          " ".join(f"[{LABEL[s]}{'V' if v else ''} ×n]" for (v, s), n in
                   sorted(cnt.items())))
    return segs


def contact_sheet(name, frames, recs, every, out_png):
    picks = list(range(0, len(frames), every))[:36]
    cols, tile, pad = 6, 160, 4
    rows = (len(picks) + cols - 1) // cols
    sheet = np.full((rows * (tile + pad + 14) , cols * (tile + pad), 3), 24, np.uint8)
    for k, idx in enumerate(picks):
        fr = cv2.resize(frames[idx], (tile, tile * 3 // 4))
        r = recs[idx]
        vis = fr.copy()
        for (x, row) in r["pts"]:
            cv2.circle(vis, (int(x * tile / 320), int(row * vis.shape[0] / 240)), 2, (0, 255, 255), -1)
        if r["fit"]:
            m, c0 = r["fit"][0], r["fit"][1]
            y1 = int((m * 0 + c0) * vis.shape[0] / 240);  y2 = int((m * 320 + c0) * vis.shape[0] / 240)
            cv2.line(vis, (0, y1), (tile, y2), (0, 255, 0), 1)
        tag = f"{'V' if r['valid'] else 'X'}{r['state']} off{r['offset']:+d}"
        gy, gx = k // cols, k % cols
        y0 = gy * (tile + pad + 14)
        x0 = gx * (tile + pad)
        h = vis.shape[0]
        sheet[y0:y0 + h, x0:x0 + tile] = vis
        cv2.putText(sheet, f"#{idx} {tag}", (x0, y0 + h + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (220, 220, 220), 1, cv2.LINE_AA)
    # cv2.imwrite 不吃 Windows 非 ASCII 路径 → imencode + 文件对象写
    ok, buf = cv2.imencode(".png", sheet)
    with open(out_png, "wb") as f:
        f.write(buf.tobytes())
    print(f"拼板已写: {out_png}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", default=None)
    ap.add_argument("--set", action="append", default=[],
                    help="覆盖常量, 如 --set DELTA_GATE=20")
    ap.add_argument("--sheet-every", type=int, default=8)
    args = ap.parse_args()

    ns = load_from_main()
    for kv in args.set:
        k, v = kv.split("=")
        ns[k.strip()] = float(v) if "." in v or "e" in v.lower() else int(v)
        print(f"[覆盖] {k} = {ns[k.strip()]}")
    to_lab = make_lab_converter()

    videos = [args.video] if args.video else [
        os.path.join(HERE, "..", "校赛地图运行视频", "校赛地图1.mp4"),
        os.path.join(HERE, "..", "校赛地图运行视频", "校赛地图2.mp4"),
    ]
    outdir = os.path.join(HERE, "out")
    os.makedirs(outdir, exist_ok=True)

    for vid in videos:
        name = os.path.splitext(os.path.basename(vid))[0]
        allf, frames = load_frames(vid)
        rp = Replay(ns, to_lab)
        recs = [rp.step(fr) for fr in frames]
        print(f"\n[{name}] 容器帧 {len(allf)} -> 去重后真实帧 {len(frames)} "
              f"(≈{len(frames)/14.0:.1f}s @14fps)")
        # 相机同款调试行（每 10 帧 1 行, 模拟 1s 限速打印）
        for i in range(0, len(recs), 10):
            r = recs[i]
            d_txt = (" d=%+d" % int(r["delta"])) if (r["valid"] == 0 and r["delta"] is not None) else ""
            print("valid=%d[%s] off=%4d ang=%+6.1f st=%d%s 点%d剔%d 列?/8 通道? 距%.0f" % (
                r["valid"], LABEL[r["state"]] + ("(有效)" if r["valid"] else ""),
                r["offset"], r["angle"] / 10.0, r["state"], d_txt,
                r["nk"], r["nd"], r["margin"]))
        analyze(name, recs, 14.0)
        # CSV
        csv_path = os.path.join(outdir, f"{name}_replay.csv")
        with open(csv_path, "w", newline="", encoding="utf-8") as f:
            w = csv.writer(f)
            w.writerow(["frame", "valid", "offset", "angle_x01deg", "state",
                        "margin", "npts", "nk", "nd", "src", "base_off", "delta", "near", "row_ctrl"])
            for i, r in enumerate(recs):
                w.writerow([i, r["valid"], r["offset"], r["angle"], r["state"],
                            round(r["margin"], 1), r["npts"], r["nk"], r["nd"], r["src"],
                            None if r["base"] is None else round(r["base"], 1),
                            None if r["delta"] is None else round(r["delta"], 1),
                            r["near"],
                            None if r["row_ctrl"] is None else round(r["row_ctrl"], 1)])
        print(f"CSV 已写: {csv_path}")
        contact_sheet(name, frames, recs, args.sheet_every,
                      os.path.join(outdir, f"{name}_sheet.png"))


if __name__ == "__main__":
    main()
