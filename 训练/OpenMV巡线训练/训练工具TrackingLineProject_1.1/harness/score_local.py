# -*- coding: utf-8 -*-
"""
离线评分：任务帧 + 人工真值 → /compare 同款指标（不出 HTTP）
=============================================================
用途：训练迭代时批量/快速评估，免起 Flask。指标口径与 labeltool/server.py::compare
完全一致（off_mae/off_max/ang_mae_deg/state_match/jitter），另加状态混淆矩阵
（gt→pred），定位"压灰区/正常 判成 压黄区"这类误判。

用法:
  python score_local.py                       # 全部任务
  python score_local.py --tasks A,B,E         # 指定任务
  python score_local.py --set FILT_EMA_A=0.4  # 参数扫掠(不改主文件, 同 replay.py)
  python score_local.py --dump out/rows_A.json --tasks A   # 导出逐帧行供分析

红线遵守：算法常量/纯函数一律 load_from_main() exec 加载（单一事实源），本文件
只做"跑算法 + 对真值算数"。gt_from_line 的换算公式与 server.py::gt_from_line
同源（常量取自主文件 ns）——改主文件出量映射时两边要一起看。
"""
import os
import sys
import json
import math
import argparse

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from replay import load_from_main, make_lab_converter, Replay  # noqa: E402

PROJ = os.path.dirname(HERE)
TASKS_DIR = os.path.join(PROJ, "labeltool", "tasks")


def imread_u(path):
    """cv2.imread 读不到 Windows 非 ASCII 路径 —— np.fromfile + imdecode（同 server.py）。"""
    try:
        buf = np.fromfile(path, dtype=np.uint8)
    except OSError:
        return None
    if buf.size == 0:
        return None
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def gt_from_line(line, g):
    """真值线 → (gt_off, gt_ang)。公式同 server.py::gt_from_line，常量取自主文件 ns。"""
    x1, y1, x2, y2 = line
    if abs(x2 - x1) < 1e-6:
        m = 1e6
        row = (y1 + y2) / 2.0
    else:
        m = (y2 - y1) / float(x2 - x1)
        row = m * g["CTRL_X"] + (y1 - m * x1)
    error_px = (row - g["FRAME_H"] / 2.0) - g["TARGET_OFFSET"]
    off = max(-100, min(100, int(error_px / g["FRAME_H"] * 2 * g["ERR_GAIN"])))
    ang = max(-900, min(900, int(math.degrees(math.atan(m)) * 10)))
    return off, ang


def _std(xs):
    return float(np.std(xs)) if len(xs) > 1 else None


def res_jitter(rows):
    """残差抖动指标 —— 同 server.py::_res_jitter。台阶只在帧号相邻的残差点对间算。"""
    out = {}
    for axis, kp, kg, kr, unit, suf in (("off", "p_off", "g_off", "off_err", 1.0, ""),
                                        ("ang", "p_ang", "g_ang", "ang_err", 0.1, "_deg")):
        pts = [(r["i"], r[kp] * unit, r[kg] * unit, r[kr] * unit)
               for r in rows if r.get(kr) is not None]
        if len(pts) < 2:
            out[axis] = None
            continue
        pairs = [(a, b) for a, b in zip(pts, pts[1:]) if b[0] == a[0] + 1]
        st_res = [b[3] - a[3] for a, b in pairs]
        out[axis] = {
            "n": len(pts), "n_stairs": len(pairs),
            "res_std" + suf: _std([p[3] for p in pts]),
            "stair_std" + suf: _std(st_res),
            "stair_max" + suf: max((abs(v) for v in st_res), default=None),
            "pred_stair_std" + suf: _std([b[1] - a[1] for a, b in pairs]),
            "gt_stair_std" + suf: _std([b[2] - a[2] for a, b in pairs]),
        }
    return out


def score_task(name, ns, to_lab, dump=None):
    tdir = os.path.join(TASKS_DIR, name)
    meta_path = os.path.join(tdir, "meta.json")
    if not os.path.exists(meta_path):
        return None
    with open(meta_path, encoding="utf-8") as f:
        meta = json.load(f)
    ann_path = os.path.join(tdir, "annotations.json")
    ann = {"frames": {}}
    if os.path.exists(ann_path):
        with open(ann_path, encoding="utf-8") as f:
            ann = json.load(f)

    rp = Replay(ns, to_lab)
    rows, off_errs, ang_errs, st_ok, st_tot = [], [], [], 0, 0
    conf = {}   # (gt_state, pred_state) -> n，仅统计有真值 state 的帧
    for i in range(meta["n_frames"]):
        fr = imread_u(os.path.join(tdir, "frames", "%04d.jpg" % i))
        if fr is None:
            break
        r = rp.step(fr)
        a = ann["frames"].get(str(i), {})
        row = {"i": i, "valid": r["valid"], "p_off": r["offset"], "p_ang": r["angle"],
               "p_st": r["state"], "g_st": a.get("state"), "g_off": None, "g_ang": None,
               "off_err": None, "ang_err": None, "st_ok": None,
               "npts": r["npts"], "nk": r["nk"], "margin": round(r["margin"], 1),
               "reject": r.get("reject", 0)}
        if a.get("line"):
            g_off, g_ang = gt_from_line(a["line"], ns)
            row["g_off"], row["g_ang"] = g_off, g_ang
            if r["valid"]:
                row["off_err"] = r["offset"] - g_off
                row["ang_err"] = r["angle"] - g_ang
                off_errs.append(abs(row["off_err"]))
                ang_errs.append(abs(row["ang_err"] / 10.0))
        if a.get("state") is not None:
            st_tot += 1
            row["st_ok"] = r["state"] == a["state"]
            conf[(a["state"], r["state"])] = conf.get((a["state"], r["state"]), 0) + 1
            if row["st_ok"]:
                st_ok += 1
        rows.append(row)

    if dump:
        os.makedirs(os.path.dirname(dump) or ".", exist_ok=True)
        with open(dump, "w", encoding="utf-8") as f:
            json.dump(rows, f, ensure_ascii=False)

    # 有效帧间跳变（同 replay.analyze ② 的口径：相邻有效帧 |Δoff|>25 且间隔 ≤2）
    pv = [(r["i"], r["p_off"]) for r in rows if r["valid"] == 1]
    jumps = [(i1, o1, i2, o2) for (i1, o1), (i2, o2) in zip(pv, pv[1:])
             if abs(o2 - o1) > 25 and i2 - i1 <= 2]
    jit = res_jitter(rows)
    summary = {
        "task": name,
        "n_frames": len(rows),
        "n_line_gt": len(off_errs),
        "n_state_gt": st_tot,
        "off_mae": round(float(np.mean(off_errs)), 2) if off_errs else None,
        "off_max": int(max(off_errs)) if off_errs else None,
        "ang_mae_deg": round(float(np.mean(ang_errs)), 3) if ang_errs else None,
        "state_match": "%d/%d" % (st_ok, st_tot),
        "state_rate": round(st_ok / st_tot, 3) if st_tot else None,
        "n_valid": sum(1 for r in rows if r["valid"]),
        "n_jump": len(jumps),
        "jitter": jit,
    }
    return summary, rows, conf, jumps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tasks", default=None, help="逗号分隔任务名, 缺省=全部")
    ap.add_argument("--set", action="append", default=[], help="覆盖常量, 如 --set DELTA_GATE=20")
    ap.add_argument("--dump", default=None, help="导出逐帧 rows JSON 到此路径（单任务时）")
    ap.add_argument("--confusion", action="store_true", help="打印状态混淆明细")
    args = ap.parse_args()

    ns = load_from_main()
    for kv in args.set:
        k, v = kv.split("=")
        ns[k.strip()] = float(v) if "." in v or "e" in v.lower() else int(v)
        print("[覆盖] %s = %s" % (k.strip(), ns[k.strip()]))
    to_lab = make_lab_converter()

    if args.tasks:
        tasks = [t.strip() for t in args.tasks.split(",") if t.strip()]
    else:
        tasks = sorted(d for d in os.listdir(TASKS_DIR)
                       if os.path.exists(os.path.join(TASKS_DIR, d, "meta.json")))

    hdr = ("task    frames  lineGT  offMAE  offMax  angMAE°  state      valid  jump  "
           "off抖动(stair_std/max)      ang抖动°(stair_std/max)")
    print(hdr)
    print("-" * len(hdr))
    agg = []
    for t in tasks:
        out = score_task(t, ns, to_lab, dump=args.dump if len(tasks) == 1 else None)
        if out is None:
            print("%-6s  (无 meta.json, 跳过)" % t)
            continue
        s, rows, conf, jumps = out
        agg.append(s)
        jo = s["jitter"].get("off")
        ja = s["jitter"].get("ang")
        jo_s = ("%.2f/%.1f" % (jo["stair_std"], jo["stair_max"])) if jo else "-"
        ja_s = ("%.2f/%.1f" % (ja["stair_std_deg"], ja["stair_max_deg"])) if ja else "-"
        print("%-6s  %4d    %4d    %5s    %4s   %5s   %-8s  %4d   %4d   %-16s    %s" % (
            s["task"], s["n_frames"], s["n_line_gt"], s["off_mae"], s["off_max"],
            s["ang_mae_deg"], s["state_match"], s["n_valid"], s["n_jump"], jo_s, ja_s))
        if args.confusion and conf:
            bad = {k: v for k, v in conf.items() if k[0] != k[1]}
            if bad:
                print("        混淆(gt→pred): " + ", ".join(
                    "%d→%d×%d" % (k[0], k[1], v) for k, v in sorted(bad.items())))
        if args.confusion and jumps:
            for i1, o1, i2, o2 in jumps[:8]:
                print("        跳变 #%d(%+d)→#%d(%+d)" % (i1, o1, i2, o2))

    # 汇总（按帧数加权）
    if len(agg) > 1:
        n = sum(s["n_state_gt"] for s in agg)
        sok = sum(int(s["state_match"].split("/")[0]) for s in agg)
        nline = sum(s["n_line_gt"] for s in agg)
        mae = (sum((s["off_mae"] or 0) * s["n_line_gt"] for s in agg) / nline) if nline else 0
        amae = (sum((s["ang_mae_deg"] or 0) * s["n_line_gt"] for s in agg) / nline) if nline else 0
        print("-" * len(hdr))
        print("合计    state %d/%d (%.1f%%)  offMAE(加权) %.2f  angMAE° %.3f" % (
            sok, n, sok / n * 100 if n else 0, mae, amae))


if __name__ == "__main__":
    main()
