# -*- coding: utf-8 -*-
"""第十九次实验：列内照度包络归一化（envelope normalization）。
对每列的 L(row) 做 ±2 条滑动均值包络（下限 ENV_FLOOR），色度按 k=min(L_REF/env, K_MAX)
缩放后再走参考色/f/细化/台阶全流程 —— 照度梯度从色度信号里剥离，暗列材料边被公平放大。
用法: python harness/exp18.py [L_REF] [K_MAX] [ENV_FLOOR]
"""
import sys, os, math, json
HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(PROJ, "labeltool"))
import cv2
import numpy as np
import replay as R
import server as S

ns, to_lab = R.load_from_main(), R.make_lab_converter()
L_REF = float(sys.argv[1]) if len(sys.argv) > 1 else 70.0
K_MAX = float(sys.argv[2]) if len(sys.argv) > 2 else 2.5
ENV_FLOOR = float(sys.argv[3]) if len(sys.argv) > 3 else 20.0
ENV_WIN = 2          # 包络平滑半径（条）


def make_cc_env():
    g = ns

    def cc(prof, margin):
        n = len(prof)
        # ---- 照度包络（第十九次实验）：±ENV_WIN 条滑动均值，下限 ENV_FLOOR ----
        env = []
        for j in range(n):
            lo = max(0, j - ENV_WIN)
            hi = min(n, j + ENV_WIN + 1)
            env.append(sum(prof[k][3] for k in range(lo, hi)) / (hi - lo))
        prof_n = []
        for j in range(n):
            k = min(L_REF / max(env[j], ENV_FLOOR), K_MAX)
            prof_n.append((prof[j][0], prof[j][1] * k, prof[j][2] * k, prof[j][3]))

        l_near = (prof_n[-1][3] + prof_n[-2][3] + prof_n[-3][3]) / 3.0
        thr = max(g["L_DARK_ABS"], g["DARK_FRAC"] * l_near)
        dark = [p[3] < thr for p in prof_n]
        ndead = 0
        for dd in dark:
            if not dd:
                break
            ndead += 1
        dead = [j < ndead for j in range(n)]
        tj = [j for j in range(n) if not dark[j]][:2]
        bj = []
        for j in range(n - 1, n - 9, -1):
            if len(bj) >= 3:
                break
            if dark[j]:
                continue
            if not bj:
                bj.append(j)
                continue
            if ((prof_n[j][1] - prof_n[bj[0]][1]) ** 2
                    + (prof_n[j][2] - prof_n[bj[0]][2]) ** 2) > g["REF_GROW_STEP"] ** 2:
                break
            bj.append(j)
        if len(tj) < 2 or len(bj) < 2:
            return None
        ta = sum(prof_n[j][1] for j in tj) / len(tj)
        tb = sum(prof_n[j][2] for j in tj) / len(tj)
        ba = sum(prof_n[j][1] for j in bj) / len(bj)
        bb = sum(prof_n[j][2] for j in bj) / len(bj)
        da, db = ba - ta, bb - tb
        d2 = da * da + db * db
        f = [((p[1] - ta) ** 2 + (p[2] - tb) ** 2)
             - ((p[1] - ba) ** 2 + (p[2] - bb) ** 2) for p in prof_n]
        fs = g["smooth3"](f)
        row = g["cross_row"]([(prof_n[j][0], fs[j]) for j in range(n)], 0.0,
                             g["CROSS_MIN_SWING_FRAC"] * max(2.0 * d2, margin * margin), dead)
        for _ in range(g["REFINE_ITERS"]):
            if row is None:
                break
            r2 = g["refine_local"](prof_n, row, dark, dead)
            if r2 is None or abs(r2 - row) < 1.0:
                if r2 is not None:
                    row = r2
                break
            row = r2
        if row is not None:
            top_lim = g["WIN_Y"][0] + (tj[-1] + 1) * g["STRIP"]
            bad = row < top_lim
            if not bad:
                eps = 0.3 * d2
                for j in range(tj[-1] + 1, min(bj)):
                    if dead[j] or abs(prof_n[j][0] - row) < 0.75 * g["STRIP"]:
                        continue
                    if ((prof_n[j][0] < row and fs[j] > eps)
                            or (prof_n[j][0] > row and fs[j] < -eps)):
                        bad = True
                        break
            if bad:
                row = None
        # ---- 渐变重锚定 + 边缘强度门（第十六次；第十九次起台阶用归一化色度）----
        sa = [p[1] for p in prof_n]
        sb = [p[2] for p in prof_n]
        stepv = [math.sqrt((sa[j + 1] - sa[j]) ** 2 + (sb[j + 1] - sb[j]) ** 2)
                 for j in range(n - 1)]
        j_lo = tj[-1]
        j_hi = min(bj[0], n - 2) - 1
        if j_hi < j_lo or d2 < 1e-6:
            return None
        nvec = math.sqrt(d2)
        va, vb = (ba - ta) / nvec, (bb - tb) / nvec
        if row is not None:
            k = max(0, min(n - 2, int((row - (g["WIN_Y"][0] + g["STRIP"] / 2.0)) / g["STRIP"])))
            locs = [stepv[j] for j in (k - 1, k, k + 1) if j_lo <= j <= j_hi]
            if locs and max(locs) >= g["EDGE_STEP_SHARP"]:
                return (row, ta, tb, ba, bb)
        best_j, best_v = None, g["EDGE_STEP_SHARP"]
        for j in range(j_lo, j_hi + 1):
            if dead[j] or dead[j + 1] or stepv[j] <= best_v:
                continue
            if (sa[j + 1] - sa[j]) * va + (sb[j + 1] - sb[j]) * vb <= 0.0:
                continue
            best_v, best_j = stepv[j], j
        if best_j is None:
            return None
        r2 = (prof_n[best_j][0] + prof_n[best_j + 1][0]) / 2.0
        if 0 < best_j < n - 2:
            lft, ctr, rgt = stepv[best_j - 1], stepv[best_j], stepv[best_j + 1]
            den = lft - 2.0 * ctr + rgt
            if den < -1e-6:
                offq = 0.5 * (lft - rgt) / den
                if -1.0 <= offq <= 1.0:
                    r2 += offq * g["STRIP"] / 2.0
        if r2 < g["WIN_Y"][0] + (tj[-1] + 1) * g["STRIP"]:
            return None
        return (r2, ta, tb, ba, bb)
    return cc


def run_video(name):
    path = os.path.join(PROJ, "labeltool", "uploads", name + ".mp4")
    _, frames = R.load_frames(path)
    rp = R.Replay(ns, to_lab)
    recs = [rp.step(fr) for fr in frames]
    nv = sum(1 for r in recs if r["valid"])
    ssteps = sorted(abs(recs[i]["angle"] - recs[i - 1]["angle"])
                    for i in range(1, len(recs))
                    if recs[i]["valid"] and recs[i - 1]["valid"]
                    and abs(recs[i]["offset"] - recs[i - 1]["offset"]) <= 2)
    flips = 0
    run = []
    for i, r in enumerate(recs):
        if r["valid"] == 0:
            run.append(i)
        else:
            if len(run) >= 3:
                ss = {recs[j]["state"] for j in run if recs[j]["state"] != 4}
                if len(ss) > 1:
                    flips += 1
            run = []
    if ssteps:
        print("  %s: 有效率 %.0f%% | 静态ang步 n=%d max=%.2f° p95=%.2f° median=%.2f° 超0.3°=%.0f%% | 翻转%d" % (
            name, nv / len(recs) * 100, len(ssteps), ssteps[-1] / 10.0,
            ssteps[int(len(ssteps) * 0.95)] / 10.0, ssteps[len(ssteps) // 2] / 10.0,
            100.0 * sum(1 for v in ssteps if v > 3) / len(ssteps), flips))
    else:
        print("  %s: 有效率 %.0f%% (无静态对) 翻转%d" % (name, nv / len(recs) * 100, flips))


def run_task(name):
    tdir = os.path.join(PROJ, "labeltool", "tasks", name)
    ann = json.load(open(os.path.join(tdir, "annotations.json"), encoding="utf-8"))["frames"]
    meta = json.load(open(os.path.join(tdir, "meta.json"), encoding="utf-8"))
    rp = R.Replay(ns, to_lab)
    off_errs, ang_errs, st_ok, st_tot, n_valid = [], [], 0, 0, 0
    for i in range(meta["n_frames"]):
        fr = S.imread_u(os.path.join(tdir, "frames", "%04d.jpg" % i))
        if fr is None:
            break
        rec = rp.step(fr)
        a = ann.get(str(i), {})
        if a.get("line") and rec["valid"]:
            x1, y1, x2, y2 = a["line"]
            m = 1e6 if abs(x2 - x1) < 1e-6 else (y2 - y1) / (x2 - x1)
            row = m * ns["CTRL_X"] + (y1 - m * x1)
            g_off = max(-100, min(100, int(((row - 120.0) - ns["TARGET_OFFSET"]) / 240.0 * 2 * ns["ERR_GAIN"])))
            g_ang = max(-900, min(900, int(math.degrees(math.atan(m)) * 10)))
            off_errs.append(abs(rec["offset"] - g_off))
            ang_errs.append(abs(rec["angle"] - g_ang) / 10.0)
        if a.get("state") is not None:
            st_tot += 1
            st_ok += 1 if rec["state"] == a["state"] else 0
        n_valid += 1 if rec["valid"] else 0
    print("  %s: off %.2f/%d | ang %.3f | state %d/%d | nv %d" % (
        name, np.mean(off_errs), max(off_errs), np.mean(ang_errs), st_ok, st_tot, n_valid))
    return np.mean(off_errs), np.mean(ang_errs), st_ok, st_tot


if __name__ == '__main__':
    print("== 现状基线(18th): B[21:0.90/2.30/13% 22:0.20/1.30/5% 23:0.90/2.80/14% 24:0.30/0.90/4%] "
          "GT: A ang2.893 B 1.409 E 2.130 I 2.147 J 1.672 K 1.514 T 2.022 state 2188/2403 ==")
    ns["column_crossing"] = make_cc_env()
    print("== 包络归一化 L_REF=%.0f K_MAX=%.1f FLOOR=%.0f (含18th med5) ==" % (L_REF, K_MAX, ENV_FLOOR))
    for v in ("视频21", "视频22", "视频23", "视频24"):
        run_video(v)
    tot = [0, 0]
    for t in ("A", "B", "C", "D", "E", "I", "J", "K", "P", "T", "U", "V"):
        try:
            _, _, ok, n = run_task(t)
            tot[0] += ok
            tot[1] += n
        except Exception as e:
            print("  %s: [error] %s" % (t, e))
    print("  全任务 state: %d/%d = %.1f%% (基线 2188/2403 = 91.1%%)" % (tot[0], tot[1], tot[0] / tot[1] * 100))
