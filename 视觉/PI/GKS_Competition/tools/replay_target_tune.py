#!/usr/bin/env python3
# tools/replay_target_tune.py — 靶标检测离线回放调参器（2026-10 加）
#
# 干什么：把 logs/run_*/frames/ 里落下的真实帧喂进**与在线完全同一套**检测管线
# （tasks/_target.py::_detect_frame，含 r_max 过滤与「有孔优先」兜底），离线回答
# 两个问题，不用上车：
#   ① 某次"作废/丢靶"到底是哪一道关杀的 —— 每个候选的死因都在候选明细里
#      （--csv 导出 CSV，用 Excel/pandas 看 q/r/hole/picked 列）；
#   ② TARGET_R_MAX 取多少合适 —— --scan-rmax 扫一条曲线，看「真靶最大 r」与
#      「背景块最小 r」之间的空档在哪，取空档中间。
#
# 用法（在 GKS_Competition 目录下，Pi 或任何有 cv2 的机器）：
#   python3 tools/replay_target_tune.py                          # 最近一次 run 的 frames
#   python3 tools/replay_target_tune.py logs/run_20261002_061100/frames
#   python3 tools/replay_target_tune.py dir1 dir2 --csv out.csv
#   python3 tools/replay_target_tune.py ... --scan-rmax 100,300,25
#   python3 tools/replay_target_tune.py ... --rmax 180           # 固定某个 r_max 复跑
#
# 判读口径（与 run_target_detect 的作废判据同源）：
#   每色按"命中帧数 ≥ TARGET_MIN_VOTES 且跨帧 cx 散布 ≤ TARGET_MAX_SPREAD_PX"
#   判存活，超限即"作废" —— 输出里的 ABORT 就是在线会打到日志里的那类事件。
import argparse
import csv
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2  # noqa: E402

import config  # noqa: E402
from tasks._target import _detect_frame  # noqa: E402


def collect_images(roots):
    out = []
    for root in roots:
        if os.path.isfile(root):
            out.append(root)
            continue
        for pat in ("*.jpg", "*.png"):
            out.extend(glob.glob(os.path.join(root, "**", pat), recursive=True))
    return sorted(out)


def build_cfg(r_max):
    """与 run_target_detect / _target_stream._detect_cfg 同源同值的检测参数。
    ⚠ 两处若再改字段，这里同步 —— 回放的公信力就在"与在线同一套"上。"""
    return {"sat_min": config.TARGET_SAT_MIN, "val_min": config.TARGET_VAL_MIN,
            "min_area": config.TARGET_MIN_AREA, "tol": config.TARGET_TOL,
            "q_min": config.TARGET_RING_Q_MIN, "q_max": config.TARGET_RING_Q_MAX,
            "r_max": r_max,
            "families": config.COLOR_HUE_FAMILIES,
            "use_roi": False, "roi": (0.0, 0.0, 1.0, 1.0)}


def run_once(images, r_max, csv_path=None):
    """跑一遍 → (每帧检出色表, 每色统计, 作废事件表, 候选明细)。"""
    cfg = build_cfg(r_max)
    per_frame = []          # [(name, {color: (cx, cy)})]
    cands_all = []          # 候选明细（debug_cands 的行）
    for path in images:
        img = cv2.imread(path)
        if img is None:
            print(f"[跳过] 读不出: {path}")
            continue
        cands = []
        targets, _ = _detect_frame(img, cfg, config.TARGET_PROC_SCALE,
                                   debug_cands=cands)
        for row in cands:
            row["frame"] = os.path.basename(path)
        cands_all.extend(cands)
        per_frame.append((os.path.basename(path),
                          {t["color"]: (t["cx"], t["cy"]) for t in targets}))

    # ---- 每色统计（复现投票/作废判据）----
    stats = {}      # color -> {"hits", "xs", "abort", "fallback_n", "holeness_n"}
    events = []     # 作废事件
    for name, found in per_frame:
        for col, (cx, _cy) in found.items():
            st = stats.setdefault(col, {"hits": 0, "xs": [], "abort": False,
                                        "fallback_n": 0, "holeness_n": 0})
            st["hits"] += 1
            st["xs"].append(cx)
    for row in cands_all:
        col = row["color"]
        if col in stats:
            if row["picked"] == "fallback":
                stats[col]["fallback_n"] += 1
            if not row["hole"]:
                stats[col]["holeness_n"] += 1
    for col, st in sorted(stats.items()):
        xs = sorted(st["xs"])
        spread = (xs[-1] - xs[0]) if xs else 0
        if st["hits"] < config.TARGET_MIN_VOTES:
            st["abort"] = f"命中 {st['hits']} 帧 < {config.TARGET_MIN_VOTES}"
            events.append((col, f"票数不足（{st['hits']}<{config.TARGET_MIN_VOTES}）"))
        elif spread > config.TARGET_MAX_SPREAD_PX:
            st["abort"] = f"散布 {spread:.0f}px > {config.TARGET_MAX_SPREAD_PX:.0f}"
            events.append((col, f"跨帧散布 {spread:.0f}px 超限"))
        else:
            st["abort"] = False

    if csv_path:
        with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.DictWriter(f, fieldnames=[
                "frame", "color", "picked", "r", "q", "hole",
                "cx", "cy", "area"])
            w.writeheader()
            w.writerows(cands_all)
        print(f"[CSV] 候选明细 × {len(cands_all)} → {csv_path}")
    return per_frame, stats, events, cands_all


def scan_rmax(images, lo, hi, step):
    """对 r_max 扫一条曲线。看什么：
    - ABORT_n 降到 0 的最小 r_max ≈ 「背景块被挡干净」的点；
    - found_3（三色齐帧数）不再随 r_max 下降的平台 ≈ 「真靶无一误杀」；
    两者之间的空档取中就是建议值。"""
    print(f"\n{'r_max':>7} {'候选数':>7} {'兜底':>6} {'无孔兜底':>8} "
          f"{'作废次数':>8} {'三色齐帧':>8}")
    for r_max in range(lo, hi + 1, step):
        per_frame, _st, events, cands = run_once(images, float(r_max))
        full = sum(1 for _n, fnd in per_frame if len(fnd) >= 3)
        holeless_fb = sum(1 for r in cands
                          if r["picked"] == "fallback" and not r["hole"])
        print(f"{r_max:>7} {len(cands):>7} "
              f"{sum(1 for r in cands if r['picked'] == 'fallback'):>6} "
              f"{holeless_fb:>8} {len(events):>8} {full:>8}/{len(per_frame)}")


def main():
    ap = argparse.ArgumentParser(description="靶标检测离线回放调参器")
    ap.add_argument("roots", nargs="*",
                    help="图片目录/文件；缺省= logs/ 下最新的 run_*/frames")
    ap.add_argument("--csv", default=None, help="候选明细导出 CSV 路径")
    ap.add_argument("--rmax", type=float, default=None,
                    help=f"固定 r_max 复跑（缺省= config.TARGET_R_MAX="
                         f"{getattr(config, 'TARGET_R_MAX', 0)}）")
    ap.add_argument("--scan-rmax", default=None,
                    help="扫描『起始,结束,步进』，如 100,300,25")
    args = ap.parse_args()

    roots = args.roots
    if not roots:
        logs = sorted(glob.glob(os.path.join("logs", "run_*", "frames")))
        if not logs:
            sys.exit("没给目录，logs/ 下也没找到 run_*/frames")
        roots = [logs[-1]]
    images = collect_images(roots)
    if not images:
        sys.exit(f"目录里没有 jpg/png: {roots}")
    print(f"[回放] {len(images)} 帧 ← {roots}")

    if args.scan_rmax:
        lo, hi, step = (int(x) for x in args.scan_rmax.split(","))
        scan_rmax(images, lo, hi, step)
        return

    r_max = args.rmax if args.rmax is not None else config.TARGET_R_MAX
    per_frame, stats, events, _ = run_once(images, r_max, csv_path=args.csv)

    print(f"\n== r_max={r_max}，每帧检出 ==")
    for name, found in per_frame:
        tag = ",".join(f"{config.TARGET_COLOR_NAMES.get(c, c)}" for c in sorted(found))
        print(f"  {name:<20} [{tag}]")
    print("\n== 每色统计 ==")
    for col, st in sorted(stats.items()):
        verdict = ("OK" if not st["abort"] else f"作废：{st['abort']}")
        print(f"  {config.TARGET_COLOR_NAMES.get(col, col)}: "
              f"命中 {st['hits']} 帧, 兜底 {st['fallback_n']} 次"
              f"（其中无孔候选 {st['holeness_n']}） → {verdict}")
    if events:
        print("\n== 作废事件（在线会打进日志的那类）==")
        for col, why in events:
            print(f"  {config.TARGET_COLOR_NAMES.get(col, col)}: {why}")
    else:
        print("\n无作废事件。")


if __name__ == "__main__":
    main()
