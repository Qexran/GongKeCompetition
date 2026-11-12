# -*- coding: utf-8 -*-
"""B1~B4（视频21~24）无真值诊断：静态角度噪声量化 + B2 爆破桶段叠加图。
用法（任何终端，任意目录）:  python harness/analyze_B.py
结果: harness/out/B_report.txt + harness/out/B_*.png（给 ZCode 读）
"""
import os, sys, io, math, contextlib
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(PROJ, "labeltool"))
import cv2
import replay as R

OUT = os.path.join(HERE, "out")
os.makedirs(OUT, exist_ok=True)
rep = open(os.path.join(OUT, "B_report.txt"), "w", encoding="utf-8")


def P(*a):
    s = " ".join(str(x) for x in a)
    print(s)
    rep.write(s + "\n")


ns = R.load_from_main()
to_lab = R.make_lab_converter()

VIDEOS = ("视频21", "视频22", "视频23", "视频24")


def overlay(frame, rec, tag):
    vis = frame.copy()
    for (x, row) in rec["pts"]:
        cv2.circle(vis, (int(x), int(row)), 3, (0, 255, 255), -1)
    if rec["fit"]:
        m, c0 = rec["fit"][0], rec["fit"][1]
        cv2.line(vis, (0, int(m * 0 + c0)), (320, int(m * 320 + c0)), (0, 255, 0), 1)
    cv2.putText(vis, tag, (3, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
    return vis


def save_png(name, img):
    ok, buf = cv2.imencode(".png", img)
    if ok:
        with open(os.path.join(OUT, name), "wb") as f:
            f.write(buf.tobytes())


for name in VIDEOS:
    path = os.path.join(PROJ, "labeltool", "uploads", name + ".mp4")
    if not os.path.exists(path):
        P(f"[skip] {path} 不存在")
        continue
    try:
        allf, frames = R.load_frames(path)
        rp = R.Replay(ns, to_lab)
        recs = [rp.step(fr) for fr in frames]
        P(f"\n===== {name}: 容器帧 {len(allf)} → 内容帧 {len(frames)} =====")
        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            R.analyze(name, recs, 14.0)
        P(buf.getvalue().rstrip())

        # 角度帧间步（连续有效帧）
        steps = []
        for i in range(1, len(recs)):
            if recs[i]["valid"] and recs[i - 1]["valid"]:
                doff = recs[i]["offset"] - recs[i - 1]["offset"]
                dang = recs[i]["angle"] - recs[i - 1]["angle"]
                steps.append((i, doff, dang))
        asteps = sorted(abs(d[2]) for d in steps)
        if asteps:
            P("角度帧间步: n=%d max=%.2f° p95=%.2f° median=%.2f° 超0.3°帧占比=%.0f%%" % (
                len(asteps), asteps[-1] / 10.0, asteps[int(len(asteps) * 0.95)] / 10.0,
                asteps[len(asteps) // 2] / 10.0,
                100.0 * sum(1 for v in asteps if v > 3) / len(asteps)))
            # 静态近似 = |Δoff|<=2 的帧对里的角度步
            st = sorted(abs(d[2]) for d in steps if abs(d[1]) <= 2)
            if st:
                P("静态段(|Δoff|<=2)角度步: n=%d max=%.2f° p95=%.2f° median=%.2f° 超0.3°=%.0f%%" % (
                    len(st), st[-1] / 10.0, st[int(len(st) * 0.95)] / 10.0,
                    st[len(st) // 2] / 10.0, 100.0 * sum(1 for v in st if v > 3) / len(st)))
            # 最大 |Δoff| 与 |Δang| 的帧（叠加图 + 原图，各取前4，供看桶/黑区）
            offrank = sorted(steps, key=lambda s: -abs(s[1]))[:4]
            angrank = sorted(steps, key=lambda s: -abs(s[2]))[:4]
            picks = sorted(set([s[0] for s in offrank] + [s[0] for s in angrank]))
            for i in picks:
                fr = frames[i]
                tag = "#%d V%d%s off%+d ang%+.1f" % (
                    i, recs[i]["valid"], R.LABEL[recs[i]["state"]],
                    recs[i]["offset"], recs[i]["angle"] / 10.0)
                save_png("B_%s_f%04d_overlay.png" % (name, i), overlay(fr, recs[i], tag))
                save_png("B_%s_f%04d_raw.png" % (name, i), fr)
            P("叠加图已存: " + ", ".join("f%04d" % i for i in picks))
    except Exception as e:
        import traceback
        P(f"[error] {name}: {e}")
        P(traceback.format_exc())

P("\nDONE")
rep.close()
print("报告: harness/out/B_report.txt")
