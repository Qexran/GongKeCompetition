#!/usr/bin/env python3
# color_calib.py — 色球 HSV 阈值标定工具（排爆区）
#
# 阈值必须在**赛场实际光照**下标定（命题明说"场地会有一定色差，以现场为准"）。
# 本机（树莓派）通常没有 X11，cv2.imshow / createTrackbar 用不了，所以做成
# 「打印统计 + 存标注图」，并支持**离线单图模式**——没有摄像头也能在笔记本上调阈值。
#
# 复用 tasks/_color.py::_detect_frame（**同一个检测函数**，保证标定与实跑一致，
# 不复制第二份阈值逻辑）。改阈值时是临时改写 config 的模块属性再还原。
#
# 用法:
#   python3 color_calib.py --image logs/sample.jpg            # 离线: 每色候选块统计
#   python3 color_calib.py --image logs/sample.jpg --sweep    # 离线: 粗扫 S/V 下限给建议
#   python3 color_calib.py --camera --frames 30               # 在线: 连拍统计 + 存标注图
#
# 标定流程建议:
#   1) 现场光照下拍一张三球照片（或摆好球用 --camera）
#   2) --sweep 找到「每色恰好一个合格圆」的 S/V 下限区间
#   3) 把结果填进 config.py 的 COLOR_SAT_MIN / COLOR_VAL_MIN
#   4) 再用 --image 复核一遍，看 cx 顺序与肉眼是否一致
import argparse
import os
import sys
import time

import cv2

import config
from tasks._color import _detect_frame, _annotate, _COLOR_EN

_NAMES = config.COLOR_COLOR_NAMES

# sweep 的候选下限（S/V 同时扫；H 是颜色身份，不扫）
_SWEEP_VALS = [40, 60, 80, 100, 120, 140]


def _px2mm_note(cx_list, align_cx, px2mm, spacing_mm):
    """按单片机的公式预演一遍横移量（标定 COLOR_PX_TO_MM_X 时对照实际落点）。"""
    if len(cx_list) < 3:
        return "(三球未齐, 无法预演位移)"
    mid = cx_list[1]
    out = []
    for idx, cx in enumerate(cx_list):
        dx = (mid - align_cx) * px2mm + (idx - 1) * spacing_mm
        out.append(f"目标={_NAMES.get(idx + 1, '?')}[{idx}] -> {dx:+.1f}mm")
    return " | ".join(out)


def _print_dets(balls, hsv, label=""):
    """打印每个球：位置/半径/面积/圆度 + **球内三色像素数**。

    球内三色像素数是重点——判定是「哪个颜色最多」（相对判定），
    所以要看的是三者的**相对大小**，而不是某一色够不够阈值。
    """
    print(f"--- {label} ---" if label else "---")
    if not balls:
        print("  (无合格色球)")
        return
    for b in sorted(balls, key=lambda x: x["cx"]):
        hits = b.get("hits") or {}
        hs = "  ".join(f"{_NAMES.get(k, k)}={v}" for k, v in sorted(hits.items()))
        win = _NAMES.get(b["color"], b["color"])
        print(f"  [{win}] cx={b['cx']:7.1f} cy={b['cy']:7.1f} r={b['r']:6.1f} "
              f"面积={b['area']:8.0f}(降采样) 圆度={b['circ']:.3f}")
        print(f"        球内: {hs}   ← 胜出: {win}")
    if len(balls) < 3:
        print(f"  ⚠️ 只找到 {len(balls)} 个球（要 3 个）。若画面里确实有三个: "
              f"调 COLOR_MIN_AREA({config.COLOR_MIN_AREA}) / COLOR_MIN_CIRCULARITY"
              f"({config.COLOR_MIN_CIRCULARITY}) / COLOR_MIN_COLOR_PIXELS"
              f"({config.COLOR_MIN_COLOR_PIXELS})；或看是不是球挨在一起被并成一个连通域了")


def _sweep_one(img, which):
    """扫 SAT_MIN 或 VAL_MIN（另一个固定在 config 当前值），打印找到几颗球。

    直接改写 config 的属性再调 _detect_frame —— 复用同一条检测链路，
    于是这里给出的建议就是实跑时的行为，不存在「标定工具和实跑两套逻辑」的偏差。
    """
    attr = "COLOR_SAT_MIN" if which == "SAT" else "COLOR_VAL_MIN"
    saved = getattr(config, attr)
    other = "COLOR_VAL_MIN" if which == "SAT" else "COLOR_SAT_MIN"
    print(f"\n{which} 下限扫描（{other}={getattr(config, other)} 固定；"
          f"色相族与实跑一致，本步不动）")
    try:
        for v in _SWEEP_VALS:
            setattr(config, attr, v)
            balls, _small, _unc = _detect_frame(img, config.COLOR_PROC_SCALE)
            found = "  ".join(f"{_NAMES[b['color']]}@cx={b['cx']:.0f}"
                              f"(圆度{b['circ']:.2f})"
                              for b in sorted(balls, key=lambda x: x["cx"]))
            mark = "<= 3颗" if len(balls) == 3 else "      "   # 不用 ✓ 等非 ASCII: 控制台可能是 GBK
            print(f"  {which}>={v:>3}  找到 {len(balls)} 颗 {mark}  {found}")
    finally:
        setattr(config, attr, saved)


def _run_sweep(img):
    """粗扫 SAT/VAL 下限。

    新判定下这两个只影响**找不找得到球**（找球只看饱和+亮度），不影响**判成哪一色**
    （定色是球内逐族比像素数）。

    ⚠️ 反直觉: **不是越低越好**。下限调低会把球周围的彩色反光/背景也并进来，
    连通域变大变碎、圆度掉下去，反而被 COLOR_MIN_CIRCULARITY 剔掉 ——
    实测同一帧 SAT>=40 只找到 2 颗、SAT>=100 才 3 颗全中。
    所以要挑的是「三颗都找到 **且圆度最高**」的那一档，不是最小的那一档。
    """
    _sweep_one(img, "SAT")
    _sweep_one(img, "VAL")
    print("\n挑「三颗都找到、且圆度普遍最高」的那一档填进 config.py —— 不是最小的那档。")
    print("若某一行三颗齐但**颜色判错**: 那是色相族（COLOR_HUE_FAMILIES）的问题，"
          "本步没扫它 —— 看 _print_dets 打的「球内 R/G/B」三数，胜出族应该是压倒性的。")
    print("若某颗球死活找不到: 看它的圆度，多半是被反光/阴影切得不圆了，"
          f"可下调 COLOR_MIN_CIRCULARITY（现 {config.COLOR_MIN_CIRCULARITY}）。")


def _process(img, args, label):
    balls, small, _unc = _detect_frame(img, config.COLOR_PROC_SCALE)
    hsv = cv2.cvtColor(small, cv2.COLOR_BGR2HSV)
    _print_dets(balls, hsv, label)

    targets = [{"color": b["color"], "color_name": _NAMES.get(b["color"], str(b["color"])),
                "cx": b["cx"], "cy": b["cy"], "r": b["r"]}
               for b in balls]
    targets.sort(key=lambda t: t["cx"])
    if targets:
        order = " ".join(t["color_name"] for t in targets)
        print(f"  从左到右: {order}")
        print(f"  预演位移: {_px2mm_note([t['cx'] for t in targets], args.align_cx, args.px2mm, args.spacing_mm)}")
        annotated = _annotate(img, targets, extra=order)
        if args.save:
            os.makedirs(os.path.dirname(os.path.abspath(args.save)), exist_ok=True)
            cv2.imwrite(args.save, annotated)
            print(f"  标注图已存: {args.save}")
    else:
        print("  三球未齐 —— 若现场确实摆了三个球，说明阈值需要调（先跑 --sweep）")
    return targets


def main():
    ap = argparse.ArgumentParser(description="色球 HSV 阈值标定（排爆区）")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--image", help="离线单图路径")
    src.add_argument("--camera", action="store_true", help="在线: 用主摄像头")
    ap.add_argument("--frames", type=int, default=30, help="在线模式连拍帧数（默认 30）")
    ap.add_argument("--sweep", action="store_true", help="离线: 粗扫 S/V 下限，给建议区间")
    ap.add_argument("--px2mm", type=float, default=0.30,
                    help="单片机的 COLOR_PX_TO_MM_X（默认 0.30，用于预演横移量）")
    ap.add_argument("--align-cx", type=float, default=960.0,
                    help="单片机的 COLOR_ALIGN_CX（默认 960，用于预演横移量）")
    ap.add_argument("--spacing-mm", type=float, default=100.0,
                    help="单片机的 COLOR_BALL_SPACING_MM（默认 100，用于预演横移量）")
    ap.add_argument("--save", default=os.path.join(config.LOG_DIR, "color_calib_last.jpg"),
                    help="标注图输出路径")
    args = ap.parse_args()

    print(f"当前阈值: PROC_SCALE={config.COLOR_PROC_SCALE} MIN_AREA={config.COLOR_MIN_AREA} "
          f"MIN_CIRC={config.COLOR_MIN_CIRCULARITY}")
    print(f"画面分辨率: {config.FRAME_WIDTH}x{config.FRAME_HEIGHT}"
          f"（回传坐标按全分辨率, 检测在 {config.COLOR_PROC_SCALE} 降采样下做）")

    if args.image:
        img = cv2.imread(args.image)
        if img is None:
            raise SystemExit(f"读不到图片: {args.image}")
        print(f"图片: {args.image}  {img.shape[1]}x{img.shape[0]}")
        if args.sweep:
            _run_sweep(img)
        _process(img, args, "离线检测")
        return

    # ---- 在线模式 ----
    from camera import CameraManager, open_camera
    cap = open_camera(is_qr=False)
    if cap is None:
        raise SystemExit("主摄像头打开失败（Web 推流可能还占着，或设备号不对）")

    saved_n = 0
    try:
        print(f"连拍 {args.frames} 帧, 统计三色检出率（Ctrl+C 退出）...")
        for i in range(args.frames):
            frame = CameraManager.read(cap, "Main Camera", max_retries=3)
            if frame is None:
                time.sleep(0.05)
                continue
            det, _small, _unc = _detect_frame(frame, config.COLOR_PROC_SCALE)
            if len(det) == 3:
                saved_n += 1
            missing = [ _NAMES[c] for c in (1, 2, 3) if c not in det ]
            print(f"  第{i + 1:>3}帧: 三色齐={len(det) == 3}"
                  + (f"  缺: {' '.join(missing)}" if missing else ""))
            if i == args.frames - 1 or (i == 0 and len(det) == 3):
                _process(frame, args, f"最后一帧 (第{i + 1}帧)")
        print(f"\n三色齐的帧: {saved_n}/{args.frames}")
        if saved_n == 0:
            print("一帧都没凑齐 —— 先跑离线 --sweep 把阈值调对，再回来。")
    except KeyboardInterrupt:
        print("\n已中断")
    finally:
        CameraManager.release(cap)


if __name__ == "__main__":
    sys.exit(main())
