#!/usr/bin/env python3
"""测试两个摄像头：主摄像头 + 扫码摄像头（QR）

用法:
    python3 test_camera.py                    # 浏览器实时预览两个摄像头（web 推流）
    python3 test_camera.py --main             # 仅预览主摄像头
    python3 test_camera.py --qr               # 仅预览扫码摄像头
    python3 test_camera.py --port 8080        # 指定推流端口
    python3 test_camera.py --no-preview       # 无预览，自动读取 --frames 帧后输出统计
    python3 test_camera.py --no-preview --frames 100

诊断用（符号链接失效时直接指定真实节点）:
    python3 test_camera.py --main --main-index /dev/video0
    python3 test_camera.py --qr --qr-index /dev/video2
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2

from config import (
    CAM_MAIN_INDEX, CAM_QR_INDEX,
    CAM_MAIN_FOURCC, CAM_QR_FOURCC,
    FRAME_WIDTH, FRAME_HEIGHT, WEB_PREVIEW_PORT,
)
from camera import CameraManager
from web_preview import preview


def _fourcc_str(val):
    return "".join(chr((int(val) >> i) & 0xFF) for i in range(0, 32, 8)).strip("\x00")


def _open_with_fallback(index, width, height, fourcc):
    """依次尝试配置的 fourcc → YUYV → 默认，返回第一个成功打开的 cap。"""
    attempts = [fourcc] if fourcc else [""]
    if "YUYV" not in attempts:
        attempts.append("YUYV")
    attempts.append("")
    for fcc in attempts:
        print(f"\n--- 尝试 FOURCC='{fcc or '(默认)'}' ---")
        cap = CameraManager.open(index, width, height, fourcc=fcc)
        if cap is not None:
            return cap
    return None


def _read_frames(name, cap, frames):
    """无预览模式：自动读取 N 帧后统计成功率。"""
    success = fail = 0
    t0 = time.time()
    for i in range(frames):
        frame = CameraManager.read(cap, name, max_retries=3)
        if frame is not None:
            success += 1
        else:
            fail += 1
            if fail <= 3 or fail % 30 == 0:
                print(f"  帧{i}: 读取失败 (成功={success}, 失败={fail})")
        if time.time() - t0 > 15:
            print(f"  15秒超时, 已读 {success + fail} 帧")
            break
    dt = time.time() - t0
    total = success + fail
    fps = success / dt if dt > 0 else 0
    CameraManager.release(cap)
    ok = success > total * 0.5
    print(f"结果: {success}/{total} 成功 ({success / max(total, 1) * 100:.1f}%), "
          f"有效帧率 {fps:.1f}fps")
    print("✅ 正常" if ok else "❌ 异常")
    return ok


def _test_camera(name, index, fourcc, width, height, frames):
    """--no-preview 读帧统计：返回是否正常。"""
    print(f"\n{'=' * 60}")
    print(f"测试 {name}: {index}")
    print(f"  期望: {width}x{height}, FOURCC={fourcc}")
    print('=' * 60)

    if not os.path.exists(index):
        print(f"❌ 设备节点不存在: {index}")
        print("  检查: USB 连接、udev 规则（ls -la /dev/camera_*）")
        print("  可用节点: " + ", ".join(
            [f"/dev/video{i}" for i in range(10)
             if os.path.exists(f"/dev/video{i}")]))
        return False

    cap = _open_with_fallback(index, width, height, fourcc)
    if cap is None:
        print(f"❌ {name} 无法打开")
        return False

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    afps = cap.get(cv2.CAP_PROP_FPS)
    afcc = _fourcc_str(cap.get(cv2.CAP_PROP_FOURCC))
    print(f"实际: {aw}x{ah} @ {afps:.1f}fps, FOURCC={afcc}")
    return _read_frames(name, cap, frames)


def _web_preview(test_main, test_qr, args):
    """web 推流预览：打开摄像头 → 注册到框架 → 阻塞服务直到 Ctrl+C。"""
    preview.start(port=args.port)  # 先按指定端口启动，避免 add_camera 用默认端口提前启动
    caps = []
    if test_main:
        print(f"\n打开主摄像头: {args.main_index}")
        cap = _open_with_fallback(args.main_index, FRAME_WIDTH, FRAME_HEIGHT, CAM_MAIN_FOURCC)
        if cap is not None:
            caps.append(cap)
            preview.add_camera("main", cap, label="主摄像头")
    if test_qr:
        print(f"\n打开扫码摄像头: {args.qr_index}")
        cap = _open_with_fallback(args.qr_index, FRAME_WIDTH, FRAME_HEIGHT, CAM_QR_FOURCC)
        if cap is not None:
            caps.append(cap)
            preview.add_camera("qr", cap, label="扫码摄像头")
    if not caps:
        print("❌ 没有可用的摄像头，无法启动 web 预览")
        return
    try:
        preview.serve(port=args.port)
    finally:
        for c in caps:
            CameraManager.release(c)


def main():
    parser = argparse.ArgumentParser(description="测试主摄像头与扫码摄像头")
    parser.add_argument("--main", action="store_true", help="仅测试主摄像头")
    parser.add_argument("--qr", action="store_true", help="仅测试扫码摄像头")
    parser.add_argument("--no-preview", action="store_true",
                        help="无预览，自动读取帧后输出统计（不做 web 推流）")
    parser.add_argument("--frames", type=int, default=100,
                        help="--no-preview 模式的读取帧数")
    parser.add_argument("--port", type=int, default=WEB_PREVIEW_PORT,
                        help="web 推流端口")
    parser.add_argument("--main-index", default=CAM_MAIN_INDEX,
                        help="主摄像头设备节点（默认 /dev/camera_main）")
    parser.add_argument("--qr-index", default=CAM_QR_INDEX,
                        help="扫码摄像头设备节点（默认 /dev/camera_Qr）")
    args = parser.parse_args()

    # 默认两个都测；--main / --qr 可单独指定
    test_main = args.main or not (args.main or args.qr)
    test_qr = args.qr or not (args.main or args.qr)

    if args.no_preview:
        results = []
        if test_main:
            results.append(("主摄像头", _test_camera(
                "主摄像头", args.main_index, CAM_MAIN_FOURCC,
                FRAME_WIDTH, FRAME_HEIGHT, args.frames)))
        if test_qr:
            results.append(("扫码摄像头", _test_camera(
                "扫码摄像头", args.qr_index, CAM_QR_FOURCC,
                FRAME_WIDTH, FRAME_HEIGHT, args.frames)))
        print("\n" + "=" * 60)
        print("测试总结")
        print("=" * 60)
        for name, ok in results:
            print(f"  {name}: {'✅ 正常' if ok else '❌ 异常'}")
        sys.exit(0 if all(ok for _, ok in results) else 1)

    # 默认：web 推流预览
    _web_preview(test_main, test_qr, args)


if __name__ == "__main__":
    main()
