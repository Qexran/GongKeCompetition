#!/usr/bin/env python3
# app.py — GKS Web 控制端 主入口
#
# 独立于 GKS_Competition/main.py 的开机常驻 Web 控制台（systemd: gks-web.service）。
# ⚠ 本页是**唯一入口**：GKS_Competition 自己的 camera_web.py 页面（:8000）已废弃，
#   main.py 不再 start_server()（那个文件只保留 /scan 前的 _stop_stream 复用与独立调试用）。
# 功能（Tab 分页）：控制（网络/服务/系统/电源）、日志（终端实时）、单片机模拟器、
# 参数（改 GKS_Competition/config.py 的可编辑项，见 EDITABLE_CONFIG）。
# 摄像头调试区（在「模拟器」Tab 内）可就地跑识别任务：二维码（/scan；扫码副摄 或
# 主摄，见 _handle_scan）/ 人质形状（/hostage，主摄 YOLO）。
#
# 全部用标准库 http.server，零第三方依赖；摄像头/扫码/协议复用 GKS_Competition 的
# 无副作用库模块（protocol/camera/tasks/log_buffer），绝不 import 有副作用的 main.py。

import base64
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime, timedelta
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from webconfig import (BASE_DIR, GKS_COMPETITION_DIR, WEB_HOST, WEB_PORT,
                       SIM_HOST, SIM_PORT, GKS_SERVICE, WEB_SERVICE,
                       SIM_LOG_MAX_LINES, WEB_KEY)

# 复用 GKS_Competition 的库模块（无副作用）
sys.path.append(GKS_COMPETITION_DIR)
import cv2
import config as gks_config          # GKS_Competition/config.py（摄像头/二维码参数）
import log_buffer                     # GKS_Competition/log_buffer.py
import camera_lease                   # GKS_Competition/camera_lease.py（acquire/release 走它）
import camera_picam                   # GKS_Competition/camera_picam.py（CSI 摄像头，目前只做预览）
import logstore                       # GKS_Competition/logstore.py（run 目录/帧归档/清理/回看）
from camera import CameraManager, open_camera, profile_params
from tasks import (run_qr_opencv, run_qr_pyzbar, run_qr_wechat, run_qr_wechat_main,
                   run_qr_diff, run_qr_diff_main,
                   run_qr_auto, run_qr_auto_main,
                   run_hostage_detect, run_color_detect, run_target_detect, run_rect_detect,
                   run_road_detect, run_road_yaw_detect)

import console                        # GKS_Web/console.py
from simulator import SimulatorServer, COMMANDS

JPEG_QUALITY = 80
BOUNDARY = "frame"

# ---- 网页「靶标识别(pi)」按钮等 CSI 起流的上限[秒]（2026-10-16 加）----
# 为什么需要它: 这个按钮**不受单片机时限约束**（人手点的），而冷启动 CSI 要 ~3s
# （列相机 + 首帧 ~2s + 丢收敛帧）。可它在 GKS_Web 进程里调的 `run_target_detect`
# 给 CSI 那一次只留 `config.PICAM_TRY_BUDGET_S`（默认 2s —— 那是**给单片机那条路**的:
# 超了就把随后主摄那次的预算吃光），于是每次点都是"2s 花完 → 退回主摄",
# 页面上看着就是「永远用主摄」。这里先 warm + 等就绪, 把"冷"变"热", 随后的 acquire 瞬时完成。
PICAM_MANUAL_WAIT_S = 6.0

# ---- 手动扫描给 CSI 那一次的子预算 / 总预算[秒]（2026-10-16 加）----
# 起流等好了还不够: 满分辨率 2592×1944 采一帧要 0.6~1.8s（降频时更慢 — 实测最低
# 1819ms/帧 ⇒ 2s 只采到 2 帧）, 而判定一颗靶要 ≥TARGET_MIN_VOTES(3) 票 ⇒ 2s 子预算
# **凑不够票**, 每次都被判"超时/不足"退回主摄 —— 页面上看着又是「永远用主摄」,
# 而主摄是 1920×1080 的像素空间, 拿它的 cx 去填 tc_cx_aim_pi 会填错一整列。
# 手动按钮不受单片机时限约束, 所以这里给足: CSI 那一次 8s（够 4~12 帧, 上限 8 帧凑满即返回）,
# 总预算 14s（8s 花完还剩 6s = 主摄那次的原预算, 退回那条路行为不变）。
PICAM_MANUAL_TRY_BUDGET_S = 8.0
TARGET_MANUAL_TIMEOUT_S = 14.0

# ---- 视差标定（2026-10 加，/parallax 页）----
# 干什么：把「激光光斑停在物理圆心」时检测到的靶心 (cx, cy) 连同当时的靶半径 r
# 一起存档；≥2 个距离点后按 r 分段线性生成 MCU 端的 center 锚点表 ——
# 补偿「相机-激光不同轴的视差把瞄准点烤死在 tg_center 一个固定像素里」的距离项
# （锚点表在 app_config.h 的 TARGET_CENTER_ANCHOR_*，TARGET_CENTER_RCOMP_EN=1 生效）。
# 存档是进程旁的 JSON，重启不丢；MCU 那头的宏由页面「生成 C 片段」输出后手工粘贴。
from tasks._target import (_detect_frame, _annotate, _roi_of, _crop)
import numpy as np

# ==================== 「帧离线验证」（日志页帧右键 → 验证, 2026-11-06 加） ====================
# 干什么: 把 logs/run_*/frames 的归档帧（训练模式的 <key>_before_<n> 原始帧等）喂进
# 与在线**完全同一套**的单帧检测管线, 弹窗看「当前代码会把这帧判成什么样」——
# 改完阈值/算法不用再上车重跑任务, 对着历史帧就地二验。
# ⚠️ 纪律: 只用各任务的**纯函数件**（_detect_frame / _detect_one / detect_rect /
#    _detect / decode_frame）, 全程不碰相机、串口、租约 —— 与在线任务同时跑也不抢设备。
# ⚠️ 参数一律 gks_config 现读（与在线同源同值, 参数页改完下一帧验证就是新值）。
# ⚠️ 靶标 cfg 与 tasks._target._run_detect_once 同源（含 fallback_q_min; CSI 归档帧
#    宽 2592 按帧宽识别, 走 pi 那把 r_max 尺 + 白心救回）。
#    模块用别名导入: 各任务的 _annotate/_detect_one 同名, 直接 from-import 会互相覆盖。
from tasks import (_color as _t_color, _hostage as _t_hostage, _qr as _t_qr,
                   _qr_auto as _t_qr_auto, _rect as _t_rect, _road as _t_road,
                   _road_yaw as _t_road_yaw)

_VALIDATE_QR_BUDGET_S = 10.0    # 二维码单帧解码预算[秒]（在线 5s 级; 手动离线验证放宽）
_VLD_CN = {1: "红", 2: "绿", 3: "蓝"}   # 色码 → 中文摘要用（图内文字仍 ASCII, cv2 画不了中文）


def _vld_key_seq(fname):
    """帧文件名 → (key, 序号)。与前端 frameKeyOf / logstore._FRAME_RE 同一套约定:
    <key>_<序号>.jpg, key 段可含 _before（训练模式配对帧）。不匹配回 None。"""
    m = re.match(r"^(?P<key>[a-z_]+)_(?P<n>\d{3,5})\.jpg$", fname)
    return (m.group("key"), m.group("n")) if m else None


def _vld_algo_of(key):
    """帧 key → 验证算法名（与前端 FRAME_CATS 同一套归类; 认不出回 None, 不猜）。"""
    if key.endswith("_before"):
        key = key[: -len("_before")]
    if key == "qr" or key.startswith("qr_"):
        return "qr"
    if key in ("road", "road_yaw", "color", "rect", "hostage"):
        return key
    if key == "target" or key.startswith("aim"):
        return "target"
    return None


def _vld_pick_input(run_name, fname):
    """右键的帧 → (algo, 输入帧名, note, 错误)。原则: 验证要吃**原始帧** ——
    同序号有配对原始帧（训练模式 <key>_before_<n>）就自动换成它; aimann 的原始
    孪生是 aimfinal; 本就是原始帧（*_before_* / aimfinal / aimseq）或没有配对时
    原样验证并如实注明（旧标注框可能轻微干扰检测, 注明让结果可信）。"""
    ks = _vld_key_seq(fname)
    if not ks:
        return None, None, None, "文件名不符合 <key>_<序号>.jpg 约定（帧白名单外）"
    key, n = ks
    algo = _vld_algo_of(key)
    if algo is None:
        return None, None, None, "认不出的帧类型: " + key + "（这种帧没有对应的离线算法）"
    if key.endswith("_before"):
        return algo, fname, "", None          # 本来就是原始帧
    cands = [key + "_before_" + n + ".jpg"]
    if key == "aimann":
        cands.append("aimfinal_" + n + ".jpg")
    for c in cands:
        if logstore.frame_path(run_name, c):
            return algo, c, "（已自动换成配对原始帧 " + c + "）", None
    return algo, fname, "（没有配对原始帧, 直接验证本帧; 若它是标注帧, 旧框线可能干扰检测）", None


def _vld_run_algo(algo, img):
    """一帧 BGR → (标注图 | None, {"ok": bool, "text": 摘要})。只跑纯函数件,
    不碰相机/串口/租约 —— 这是「离线验证」的全部安全边界。"""
    if algo == "target":
        pi = img.shape[1] > 2000        # CSI 归档帧 2592 宽; 主摄 1920 —— 按帧宽选尺
        r_max_pi = float(getattr(gks_config, "TARGET_R_MAX_PI", gks_config.TARGET_R_MAX))
        cfg = {"sat_min": gks_config.TARGET_SAT_MIN, "val_min": gks_config.TARGET_VAL_MIN,
               "min_area": gks_config.TARGET_MIN_AREA, "tol": gks_config.TARGET_TOL,
               "q_min": gks_config.TARGET_RING_Q_MIN, "q_max": gks_config.TARGET_RING_Q_MAX,
               "fallback_q_min": gks_config.TARGET_FALLBACK_Q_MIN,
               "r_max": (r_max_pi if pi else gks_config.TARGET_R_MAX),
               "rescue_hole": bool(pi),
               "families": gks_config.TARGET_HUE_FAMILIES,
               "use_roi": bool(gks_config.TARGET_USE_ROI),
               "roi": (gks_config.TARGET_ROI_X0, gks_config.TARGET_ROI_Y0,
                       gks_config.TARGET_ROI_X1, gks_config.TARGET_ROI_Y1)}
        roi = _roi_of(cfg, img)
        sub, x0, y0 = _crop(img, roi)
        targets, _small = _detect_frame(sub, cfg, gks_config.TARGET_PROC_SCALE)
        if x0 or y0:                    # ROI 坐标加回全帧（与在线 votes 同一步）
            for t in targets:
                t["cx"] += x0
                t["cy"] += y0
        txt = ("检出 %d 颗: " % len(targets)
               + " ".join("%s(%d,%d)r%d"
                          % (gks_config.TARGET_COLOR_NAMES.get(t["color"], t["color"]),
                             int(t["cx"]), int(t["cy"]), int(t["r"]))
                          for t in targets)) or "本帧没有任何靶盘过检"
        return _annotate(img, targets, extra="" if targets else "validate: no target"), \
            {"ok": bool(targets), "text": txt}

    if algo == "color":
        balls, _small, unc = _t_color._detect_frame(
            img, float(getattr(gks_config, "COLOR_PROC_SCALE", 1.0)))
        txt = ("检出 %d 球: " % len(balls)
               + " ".join("%s(%d,%d)r%d"
                          % (_VLD_CN.get(b["color"], b["color"]),
                             int(b["cx"]), int(b["cy"]), int(b.get("r") or 0))
                          for b in balls)) or "本帧没有过几何筛的球"
        if unc:
            txt += "；另有 %d 个无色轮廓（收尾排除法/猜色的池子）" % len(unc)
        return _t_color._annotate(img, balls, extra="" if balls else "validate: none"), \
            {"ok": bool(balls), "text": txt}

    if algo == "rect":
        res = _t_rect.detect_rect(img, _t_rect._params())
        ok = bool(res.get("success"))
        if ok:
            cx, cy = res["center"]
            txt = "板心(%d,%d) %s n_sides=%s" % (
                int(cx), int(cy),
                "三边估出" if res.get("estimated") else "四边精确", res.get("n_sides"))
        else:
            txt = "矩形识别失败: " + str(res.get("reason", ""))
        return _t_rect._annotate(img, res), {"ok": ok, "text": txt}

    if algo == "road":
        res = _t_road._detect_one(img, _t_road._params())
        ok = bool(res.get("success"))
        if ok:
            txt = "dx=%.1fpx dy=%.1fpx edge=%s area=%.0f" % (
                res.get("dx", 0.0), res.get("dy", 0.0),
                res.get("edge"), res.get("area", 0.0))
        else:
            txt = "看路单帧失败: " + str(res.get("reason", ""))
        return _t_road._annotate(img, res), {"ok": ok, "text": txt}

    if algo == "road_yaw":
        res = _t_road_yaw._detect_one(img, _t_road_yaw._params())
        ok = bool(res.get("success"))
        if ok:
            txt = "角度=%.2f° cols=%d rms=%.1fpx" % (
                res.get("angle", 0.0), res.get("cols", 0), res.get("rms", 0.0))
        else:
            txt = "看路校准单帧失败: " + str(res.get("reason", ""))
        return _t_road_yaw._annotate(img, res), {"ok": ok, "text": txt}

    if algo == "hostage":
        net = _t_hostage._get_net()
        if net is None:
            return None, {"ok": False, "text": "YOLO 模型缺失（gks_yolo_model/best.onnx）"}
        dets = _t_hostage._detect(net, img, gks_config.HOSTAGE_INPUT_SIZE,
                                  gks_config.HOSTAGE_CONF_THR, gks_config.HOSTAGE_IOU_THR)
        if dets is None:
            return None, {"ok": False, "text": "类名文件缺失（classes.names）"}
        hi = [d for d in dets if not d.get("lo")]
        if len(hi) > gks_config.HOSTAGE_MAX_TARGETS:
            hi = sorted(hi, key=lambda d: -d["conf"])[:gks_config.HOSTAGE_MAX_TARGETS]
        hi, elim = _t_hostage._apply_elimination(hi)
        hi.sort(key=lambda d: d["cx"])   # 成功目标按中心 x 升序（与在线 run_hostage_detect 同约定）
        txt = ("检出 %d 只: " % len(hi)
               + " ".join("%s(%d,%d)conf%.2f"
                          % (d["shape_name"], int(d["cx"]), int(d["cy"]), d["conf"])
                          for d in hi)) or "本帧没有过阈值的检出"
        if elim:
            txt += "（排除法: %s）" % elim
        return _t_hostage._annotate(img, hi), {"ok": bool(hi), "text": txt}

    if algo == "qr":
        if not getattr(_t_qr_auto, "ZXING_OK", False):
            return None, {"ok": False, "text": "zxing-cpp 未安装 —— 新扫码引擎离线解码不可用"}
        code, qinfo = _t_qr_auto.decode_frame(img, deadline=time.time() + _VALIDATE_QR_BUDGET_S)
        if code:
            txt = "解出 %s（stage=%s, 码宽 %dpx, %.0fms）" % (
                code, qinfo.get("stage"), qinfo.get("qr_px", 0), qinfo.get("ms", 0.0))
        else:
            txt = "未解出（stage=%s, cands=%d, 越界读数=%s）" % (
                qinfo.get("stage"), qinfo.get("cands", 0), qinfo.get("oob"))
        return _t_qr._annotate_auto(img, code or "?", qinfo, "auto"), \
            {"ok": code is not None, "text": txt}

    return None, {"ok": False, "text": "未支持的算法: " + algo}

PARALLAX_POINTS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "parallax_points.json")
# 采点时点击坐标 → 检出靶心的最大容忍距离[px]：点得离任何检出靶都太远 = 大概率
# 点在空地上/想采的靶没检出来，宁可报错让用户重点，也不要静默存一个错锚点。
PARALLAX_PICK_TOL_PX = 150.0


def _parallax_load():
    try:
        with open(PARALLAX_POINTS_PATH, "r", encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return []


def _parallax_save(pts):
    tmp = PARALLAX_POINTS_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(pts, f, ensure_ascii=False, indent=1)
    os.replace(tmp, PARALLAX_POINTS_PATH)


# ---- 「自动下发二维码」设置（实时日志页的调试开关，2026-10-11 加）----
# 写进 GKS_Competition 目录（不是 GKS_Web 自己的）: 消费者是 main.py（gks.service，
# 另一个进程）—— ① 握手成功后立刻推一帧 QR<值>；② 期间单片机发来的扫码指令
# （"1"/"D"/"E"/"F"）也直接回预设值、跳过相机（固件 ucRpiWaitQrResult 只在等待窗
# 内收 QR 帧，握手时推的那帧可能被它清掉，②才是可靠送达路径）。
# ⚠ 调试功能：勾选期间真实扫码全被替换 —— 页面上有黄字提醒，比赛前记得取消。
AUTO_QR_PATH = os.path.join(GKS_COMPETITION_DIR, "web_auto_qr.json")


def _auto_qr_valid(value):
    """值域校验（与 main.py handler._web_auto_qr 同一套）：三位数字、**每位 1~3**
    （与固件二维码语义同域：百位=目标色/十位=靶色/个位=形状，各 1~3，共 27 种）。"""
    v = str(value or "").strip()
    return v if (len(v) == 3 and all(c in "123" for c in v)) else None


def _auto_qr_load():
    try:
        with open(AUTO_QR_PATH, "r", encoding="utf-8") as f:
            d = json.load(f)
        return {"enabled": bool(d.get("enabled")), "value": str(d.get("value", ""))}
    except Exception:
        return {"enabled": False, "value": ""}


def _auto_qr_save(enabled, value):
    tmp = AUTO_QR_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump({"enabled": bool(enabled), "value": str(value or "").strip()},
                  f, ensure_ascii=False)
    os.replace(tmp, AUTO_QR_PATH)


def _build_target_cfg():
    """靶标检测参数 —— 与 tasks._target.run_target_detect / _target_stream._detect_cfg
    同源同值（TARGET_* 全套）。三处若再改，同步改。"""
    return {"sat_min": gks_config.TARGET_SAT_MIN, "val_min": gks_config.TARGET_VAL_MIN,
            "min_area": gks_config.TARGET_MIN_AREA, "tol": gks_config.TARGET_TOL,
            "q_min": gks_config.TARGET_RING_Q_MIN, "q_max": gks_config.TARGET_RING_Q_MAX,
            "r_max": gks_config.TARGET_R_MAX,
            "families": gks_config.COLOR_HUE_FAMILIES,
            "use_roi": False, "roi": (0.0, 0.0, 1.0, 1.0)}


def _grab_main_frame():
    """开主摄抓一帧（全分辨率 ndarray）或 None。调用方负责 release。
    ⚠ 先停该相机的推流线程再 open —— 同 _send_stream 的防 EBUSY 纪律。"""
    _stop_stream("main")
    time.sleep(0.3)
    cap = open_camera(is_qr=False)
    if cap is None:
        return None
    try:
        for _ in range(10):
            frame = CameraManager.read(cap, "main", max_retries=3)
            if frame is not None:
                return frame
            time.sleep(0.1)
        return None
    finally:
        CameraManager.release(cap)


# ---- 标靶练习（2026-10 加, /practice 页）----
# 与 /parallax 的分工: 视差标定是「人把光斑微调到物理圆心再采锚点」; 标靶练习是
# 「OLED 触发一发真实流程（识别→流控闭环→激光）, 打完点一下光斑在哪」—— 攒
# (实际打点, 检测靶心) 误差数据集, 供离线分析落点随机的原因（同位置重复性 /
# 不同位置视差 / 不同角度几何）。数值来自活动 run 的 log.txt（TGTS 调试串 +
# [靶标流] 定格帧/终态行, 见 GongKeCompetition 的 _target_stream.py）。
# 数据落 GKS_Competition/JiGuangLogs/（SMB share 直通, PC 侧可直接读去分析）。
PRACTICE_DIR = os.path.join(GKS_COMPETITION_DIR, "JiGuangLogs")
PRACTICE_FRAMES_DIR = os.path.join(PRACTICE_DIR, "frames")
PRACTICE_RECORDS_PATH = os.path.join(PRACTICE_DIR, "shots.jsonl")
PRACTICE_COND_PATH = os.path.join(PRACTICE_DIR, "condition.json")
# JiGuangLogs/frames 里允许回看的文件名（防目录穿越, 同 logstore 的白名单思路）
_RE_PRACTICE_FRAME = re.compile(r"^[A-Za-z0-9_\-]+\.jpg$")
# TeeWriter 只在秒变化时打 [HH:MM:SS], 同秒的后续行只有缩进 —— 解析时滚动携带
_RE_LOG_TIME = re.compile(r"^\[(\d{2}:\d{2}:\d{2})\]\s")


def _practice_cond_load():
    try:
        with open(PRACTICE_COND_PATH, "r", encoding="utf-8") as f:
            c = json.load(f)
        return {"pos": str(c.get("pos", "")), "angle": str(c.get("angle", "")),
                "note": str(c.get("note", ""))}
    except Exception:
        return {"pos": "", "angle": "", "note": ""}


def _practice_cond_save(c):
    os.makedirs(PRACTICE_DIR, exist_ok=True)
    tmp = PRACTICE_COND_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(c, f, ensure_ascii=False, indent=1)
    os.replace(tmp, PRACTICE_COND_PATH)


def _practice_unlink_files(rec):
    """尽力删掉一条记录名下的证据帧（files 里的相对名, 含 seq 列表）。

    删除/覆盖记录时调用, 免得 JiGuangLogs/frames 积孤儿文件。失败静默 ——
    少删一个文件不值得报错。名字里带路径分隔符的直接跳过（防目录穿越）。"""
    f = rec.get("files") or {}
    names = [v for v in f.values() if isinstance(v, str)]
    names += [v for v in (f.get("seq") or []) if isinstance(v, str)]
    for n in names:
        if not _RE_PRACTICE_FRAME.match(n):
            continue
        try:
            os.remove(os.path.join(PRACTICE_FRAMES_DIR, n))
        except OSError:
            pass


def _practice_records():
    """全部已标注记录（时间升序）。坏行跳过 —— jsonl 是 append-only, 手改坏一行
    不该让整页 500。"""
    out = []
    try:
        with open(PRACTICE_RECORDS_PATH, "r", encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if not line:
                    continue
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    except OSError:
        pass
    return out


def _parse_shots(text):
    """log.txt 文本 → 发次列表（dict, 时间序）。一发 = 一条 `TGTS cfg` 行开启,
    到下一发 cfg / 文本结束为止; 收敛/超时/中止/还在跑 都算一发。

    采集的行（哪来的见固件 ucRpiTargetServo 与 Pi _target_stream.py 的 finally）:
      [STM-DBG] TGTS cfg/ok/fail/abort/fin   ← 单片机侧: 结果/耗时/舵机与底盘累计/r
      [靶标流] 定格帧: frames/aimfinal_NNNN.jpg ← Pi 侧: 闭环最后看着的那帧（配对键）
      [靶标流] 终态: R(..,..)r.. G(..)…      ← Pi 侧: trackers 最后的位置与半径
    ⚠️ 停流那刻单片机的 fin/ok 行与 Pi 的定格帧行谁先谁后不定（两进程并发打印）,
       故按「块内 anywhere」解析, 不依赖行序。"""
    shots, cur, t = [], None, ""
    for orig in text.splitlines():
        m = _RE_LOG_TIME.match(orig)
        if m:
            t = m.group(1)
            raw = orig[m.end():]
        else:
            raw = orig
        line = raw.strip()
        if not line:
            continue
        if "TGTS cfg" in line:                     # 新一发
            cur = {"time": t, "cfg": line, "lines": [orig.rstrip()]}
            shots.append(cur)
            continue
        if cur is None:
            continue
        # 整段原样留档（含时间戳前缀, 上限防病态膨胀）: 标注落盘时随记录固化进
        # JiGuangLogs —— run 目录会被「日志」页清理, 清理后这是唯一的过程 log。
        if len(cur["lines"]) < 1000:
            cur["lines"].append(orig.rstrip())
        if line.startswith("[靶标流] 定格帧:"):
            if "frames/" in line:
                cur["final_frame"] = line.split("frames/", 1)[1].strip()
            else:
                cur["final_frame"] = None          # 无/归档失败 —— 也记下来, 别静默
        elif line.startswith("[靶标流] 终态:"):
            cur["final_state"] = line.partition(":")[2].strip()
        elif "TGTS fin:" in line:
            m2 = re.search(r"trX=(-?\d+) trY=(-?\d+) ch=(-?\d+)mm r=(\d+)px", line)
            if m2:
                cur.update(trX=int(m2.group(1)), trY=int(m2.group(2)),
                           ch_mm=int(m2.group(3)), r_px=int(m2.group(4)))
        elif ("TGTS ok:" in line or "TGTS fail:" in line or "TGTS abort:" in line):
            cur["end_time"] = t                    # 过程帧序列拷贝的时间窗右界
            if "TGTS ok:" in line:
                cur["outcome"] = "ok"
            elif "TGTS abort:" in line:
                cur["outcome"] = "abort"
            else:
                cur["outcome"] = line.split("TGTS fail: ", 1)[1].split(" ", 1)[0]
            for key, pat in (("dx", r"dx=(-?\d+)px"), ("dy", r"dy=(-?\d+)px"),
                             ("trX", r"trX=(-?\d+)"), ("trY", r"trY=(-?\d+)"),
                             ("ch_mm", r"tr=(-?\d+)mm"), ("n_ms", r"n=(\d+)ms")):
                m2 = re.search(pat, line)
                if m2:
                    cur[key] = int(m2.group(1))
    return shots


# 扫码任务状态（复用 camera_web.py 的模式）
_scan_lock = threading.Lock()
_scan_abort = threading.Event()

# /scan 的方法表：方法名 → (任务函数, 是否主摄)。
# ⚠️ 方法名是**算法 + 相机**的组合，与单片机命令码一一对应（见 docs/串口协议.md §6）：
#    wechat="1" / diff="E"（扫码副摄）；wechat_main="D" / diff_main="F"（主摄）。
#    opencv/pyzbar 只有页面用得到，没有对应命令码。
# ⚠️ 页面上的 `wechat` vs `diff`（副摄）、`wechat_main` vs `diff_main`（主摄）是
#    **同相机**下的算法 A/B —— 跨相机比出来的差异说明不了算法好坏。
# ⚠️ 以 `_main` 结尾 = 用主摄：既决定结果落在页面哪一格，也决定停哪台相机的推流。
_SCAN_METHODS = {
    "opencv": (run_qr_opencv, False),
    "pyzbar": (run_qr_pyzbar, False),
    "wechat": (run_qr_wechat, False),
    "diff": (run_qr_diff, False),
    "wechat_main": (run_qr_wechat_main, True),
    "diff_main": (run_qr_diff_main, True),
    # 2026-10 新引擎（全帧定位→紧裁→强放大）= 单片机命令 "1" / "D" 真正走的那条。
    # 页面上留这两个按钮是为了**现场 A/B**: 与 wechat/diff 摆在同一机位点一下就能比。
    "auto": (run_qr_auto, False),
    "auto_main": (run_qr_auto_main, True),
}

# 摄像头推流注册表：cam_key -> _CameraStream
_streams = {}
_streams_lock = threading.Lock()

# 无头单片机模拟器（后台线程常驻，默认不监听；由网页「启动/停止模拟器」开关控制）
SIMULATOR = SimulatorServer(SIM_HOST, SIM_PORT, SIM_LOG_MAX_LINES)
SIMULATOR.start()

# 2026-10（修「拔插 USB 才能恢复」的相机 bug）: 本页的就地识别会把冷开的相机
# adopt 成租约留在**本进程**（acquire_camera 第三条腿, 同 main.py）, 而空闲看门狗
# 以前只在 main.py 里跑 —— 网页上点一次识别, 相机就被 gks-web 攥到天荒地老,
# main.py 的 /abort 又只停推流动不了租约, 只能拔插 USB（2026-10-04 07:52 的日志）。
# 纪律: **每个会持租约的进程都必须跑看门狗**, 一台一份, 别想着共用谁的。
threading.Thread(target=camera_lease.lease_watchdog, daemon=True,
                 name="cam-lease-watchdog").start()


# ==================== 欠压 / 供电监控（2026-10 加, 「控制」Tab 系统状态里看）====================
# 为什么要常驻轮询、而不是打开页面时问一次：
#   `vcgencmd get_throttled` 的「已发生」位(16~19)是**自开机起锁存**的 —— 它只回答
#   「发生过没有」，不回答「什么时候」。而欠压往往是几毫秒的瞬时跌落（USB 设备反复
#   掉线重连、相机打不开、掉盘的常见元凶），事后问什么都问不到。所以这里每
#   POWER_POLL_S 秒采一次，把**变化沿**带时间戳记成事件。
#
# ⚠ 位含义: 0=当前欠压 1=当前 ARM 频率被限 2=当前正在限频 3=当前软温度限；
#   16/17/18/19 = 上面四条「自开机以来发生过」（sticky，掉电才清）。
# ⚠ 光靠采样会漏掉「两次采样之间发生又恢复」的瞬时跌落 —— 那一类由 sticky 位的
#   上升沿兜住（代价是时间戳精度 = 轮询周期，写进事件文字里了）。
# ⚠ 事件同时落盘（POWER_LOG_FILE）：欠压最常见的后果就是**重启**，那时内存里这份
#   日志正好没了，而它恰恰是查这个问题的唯一线索。文件只在内存那份（最近 200 条）
#   上滚动重写，反复抖动也写不爆盘。
POWER_POLL_S = 2.0
POWER_LOG_FILE = os.path.join(BASE_DIR, "power_watch.log")

_PWR_NOW_BITS = {0: "电压不足", 1: "ARM 频率被限", 2: "正在限频", 3: "软温度限触发"}
_PWR_WAS_BITS = {16: "电压不足", 17: "ARM 频率被限", 18: "限频", 19: "软温度限"}


class _PowerMonitor(threading.Thread):
    """后台每 POWER_POLL_S 秒采一次供电/限频状态，把变化沿记成带时间戳的事件。"""

    def __init__(self):
        super().__init__(daemon=True, name="power-monitor")
        self._lock = threading.Lock()
        self._throttled = None      # 最近一次读数（None = 还没采到过）
        self._prev_now = 0
        self._prev_was = 0
        self._ok = False
        self._writes = 0
        self._events = deque(maxlen=200)
        self._uv_count = 0          # 本控制端运行期间记到的欠压事件数
        self._uv_last = None
        self._load_tail()

    # ---- 落盘 / 回读：让日志活过一次重启 ----
    def _load_tail(self):
        try:
            with open(POWER_LOG_FILE, "r", encoding="utf-8", errors="replace") as f:
                for line in f.read().splitlines()[-200:]:
                    if line.strip():
                        self._events.append(line)
        except OSError:
            pass
        # ⚠ 计数与「最近时刻」也要从文件恢复 —— 否则重启一次控制端，界面就变成
        #   「欠压次数: 0」而日志框里明明列着一堆事件（自相矛盾）。
        for line in self._events:
            if "电压不足" in line and ("：开始" in line or "首次出现" in line):
                self._uv_count += 1
                self._uv_last = line[:14]     # "MM-DD HH:MM:SS"

    def _log(self, text, undervolt=False, persist=True):
        line = "%s  %s" % (datetime.now().strftime("%m-%d %H:%M:%S"), text)
        self._events.append(line)
        if undervolt:
            self._uv_count += 1
            self._uv_last = line[:14]     # "MM-DD HH:MM:SS"
        if not persist:
            # 「启动时已锁存」那行每次起服务都会重新生成，落盘只会在文件里堆一串
            # 一模一样的行 —— 它只该活在内存里。
            return
        self._writes += 1
        try:
            with open(POWER_LOG_FILE, "a", encoding="utf-8") as f:
                f.write(line + "\n")
            if self._writes % 200 == 0:
                with open(POWER_LOG_FILE, "w", encoding="utf-8") as f:
                    f.write("\n".join(self._events) + "\n")
        except OSError:
            pass

    @staticmethod
    def _read():
        """vcgencmd 读数（int）；不可用时 None。"""
        try:
            out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True,
                                 text=True, timeout=5).stdout
            return int(out.strip().split("=", 1)[1], 16)
        except Exception:
            return None

    def _apply(self, val):
        """只允许持锁调用。"""
        now = val & 0xF
        was = (val >> 16) & 0xF

        if self._throttled is None:
            # 首次采样：只播种，**不**逐位记事件 —— 否则本控制端每次重启都会把
            # 已经锁存的「已发生」位当成新事件重记一遍。但要把启动时的状态说清楚。
            self._throttled, self._prev_now, self._prev_was = val, now, was
            if was:
                labels = "、".join(_PWR_WAS_BITS[b] for b in sorted(_PWR_WAS_BITS)
                                   if (was >> (b - 16)) & 1)
                self._log("（控制端启动时已锁存：%s —— 发生在本次开机早些时候）" % labels,
                          persist=False)
            return

        self._throttled = val
        logged = set()
        # 先看「当前」位：语义精确（开始 / 恢复）
        for bit, label in _PWR_NOW_BITS.items():
            cur, old = (now >> bit) & 1, (self._prev_now >> bit) & 1
            if cur and not old:
                self._log("⚠ %s：开始" % label, undervolt=(bit == 0))
                logged.add(label)
            elif old and not cur:
                self._log("%s：恢复" % label)
        # 再看「已发生」位：兜住两次采样之间发生又恢复的瞬时跌落
        # ⚠ was 已经是右移过 16 位的 4 位值，这里要按 (bit - 16) 取 —— 直接 >> bit
        #   恒为 0，整段静默失效（单测抓到过：sticky 单独出现时一个事件都不记）。
        for bit, label in _PWR_WAS_BITS.items():
            if (was >> (bit - 16)) & 1 and not (self._prev_was >> (bit - 16)) & 1:
                if label in logged:
                    continue          # 同一拍已经记过「开始」了，别重复
                self._log("⚠ %s：本次开机首次出现（瞬时，采样没抓到持续态）" % label,
                          undervolt=(bit == 16))
        self._prev_now = now
        self._prev_was = was

    def run(self):
        while True:
            val = self._read()
            with self._lock:
                self._ok = val is not None
                if self._ok:
                    self._apply(val)
            time.sleep(POWER_POLL_S)

    def status(self):
        with self._lock:
            val = self._throttled or 0
            return {
                "ok": self._ok,
                "throttled": val,
                "undervolt_now": bool(val & 1),
                "undervolt_was": bool(val & (1 << 16)),
                "throttled_now": bool(val & 4),
                "uv_count": self._uv_count,
                "uv_last": self._uv_last,
                "events": list(self._events)[-40:],
            }

    def clear_log(self):
        """清空欠压/限频事件日志与计数（「系统状态」卡片的清除按钮）。

        ⚠️ **只清软件记录，不动硬件锁存位**：`vcgencmd get_throttled` 的 16~19 位
        是「本次开机以来发生过」，只有重启树莓派才会复位。所以清完后 `uv_count`
        归 0、日志框为空，但 `undervolt_was` 仍可能是 True —— 那是硬件在说
        「本次开机确实发生过」，不是日志没清干净。
        ⚠️ 必须和监控线程用同一把锁：`_log` 的追加/滚动重写都在持锁的 `_apply`
        里调用；这里先截断文件再清内存，避免清完立刻被下一拍写回。
        ⚠️ **不重置 `_prev_was`/`_throttled`**：sticky 位还锁存着，重置的话下一拍
        会把同一批位当成新事件重新记一遍，等于没清。
        """
        with self._lock:
            try:
                with open(POWER_LOG_FILE, "w", encoding="utf-8"):
                    pass
            except OSError as exc:
                return False, "清除失败（日志文件不可写）: %s" % exc
            self._events.clear()
            self._uv_count = 0
            self._uv_last = None
            self._writes = 0
            return True, "已清除欠压/限频日志与计数（硬件锁存位需重启才复位）"


POWER = _PowerMonitor()
POWER.start()


class _CameraStream:
    def __init__(self, cam_key, cap, conn):
        self.cam_key = cam_key
        self.cap = cap
        self.conn = conn
        self.stop = threading.Event()
        # 主摄参数悬浮窗的 /api/camctl 会从 HTTP 线程对这个 cap 做 V4L2 控制 ioctl，
        # 推流线程同时在 read()。S_CTRL 与 DQBUF/QBUF 是独立 ioctl（不破坏取流），
        # 这把锁只把多个控制请求串起来；读帧不等锁 —— 等锁会让预览卡顿。
        self.ctrl_lock = threading.Lock()


def _stop_stream(cam_key):
    with _streams_lock:
        s = _streams.get(cam_key)
    if s is not None:
        s.stop.set()
        try:
            s.conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass


def _stop_all_streams():
    with _streams_lock:
        keys = list(_streams.keys())
    for k in keys:
        _stop_stream(k)


# ---- Pi CSI 摄像头预览（libcamera 那颗，与上面两个 USB 摄像头无关）----
# ⚠ 刻意**不进** _streams 注册表：它是独立硬件（走 libcamera/ISP，既不占 /dev/camera_main
#   也不进 camera_lease），不该被 _stop_all_streams() 顺手掐掉 —— 那个函数是「检测摄像头 /
#   就地识别要腾出 USB 相机」时调的，把 CSI 预览一起停了只会让画面冻在最后一帧。
#   它的生命周期单独管：/stream?cam=pi 开、stopCam() 或再次开流时收。
_pi_stream = None
_pi_lock = threading.Lock()


def _ascii(s):
    """HTTP 状态行只装得下 latin-1 —— `send_error(code, "中文…")` 会抛
    UnicodeEncodeError，后果是**失败时连响应都发不出去**（实测踩过：连开两路预览，
    第二次客户端一个字节都收不到，日志里只有一行 codec 异常）。
    凡是要进 send_error 的动态文本，都先过这里（中文会被换成 ?，日志里有原文）。"""
    return str(s).encode("ascii", "replace").decode("ascii")


def _stop_pi_stream():
    """停 CSI 预览：踢掉 HTTP 连接 + **立刻**收 rpicam-vid 子进程。

    ⚠ 必须当场收进程（不能只 set stop 等推流线程收尾）：libcamera 独占相机，
    下一个 rpicam-vid / 自检抓帧要等它真的退了才开得起来。
    """
    global _pi_stream
    with _pi_lock:
        s = _pi_stream
        _pi_stream = None
    if s is not None:
        s.stop.set()
        try:
            s.conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass
        s.close()


# ---- 进程内「让出相机」钩子（2026-10 加, 治「已取消」玄学）----
# 本页的就地识别走 GKS_Competition 的 acquire_camera 冷开兜底: 开相机失败时它会对
# http://127.0.0.1:9000/abort 发请求 —— 在**本进程**里那一枪打的是自己: /abort 会
# set _scan_abort, 把正在跑的就地识别当场取消, 页面莫名显示「已取消」。注册本钩子后,
# camera_lease 在本进程内改走它（只停推流, 不动识别）—— 冷开重试自己分出胜负,
# 打不开时如实报「识别失败: 主摄像头打开失败」而不是「已取消」。
# ⚠️ 只影响**本进程**的调用路径; main.py 那个进程打的 /abort 照旧会取消页面识别
#    （车优先语义不变——它抢不到相机时, 页面识别让位是应该的）。
def _yield_camera_stream(cam_key):
    _stop_stream(cam_key)
    return True


camera_lease.set_abort_hook(_yield_camera_stream)

# ---- 同款钩子，给 pi(CSI) 那颗（2026-10-16 加）----
# 同一个道理的另一半: 本页的 pi 预览（rpicam-vid）与 gks.service 的识别都走 libcamera,
# **独占**。识别那边开失败会打 /abort?cam=pi 请求让出 —— 在本进程里那一枪同样打的是自己
# （会 set _scan_abort，把正在跑的就地识别取消掉），所以也走钩子: 只收预览、不动识别。
# ⚠️ 与 USB 那条不同: CSI 没有"租约"这一层（不在 camera_lease 里）, 所以只需收推流。
def _yield_pi_preview(cam_key=None):
    _stop_pi_stream()
    return True


camera_picam.set_abort_hook(_yield_pi_preview)

# ---- 本进程的 pi(CSI) 识别流也要有**空闲看门狗**（2026-10-16 加）----
# 为什么: 「靶标识别(pi)」在这条进程里跑, 它会把 CSI 流**留着热用**（同 main.py 那条路,
# `PICAM.acquire` 的语义就是"用完不关"), 而 main.py 的看门狗**管不到另一个进程**的流 ——
# 没有本看门狗, 这条流会**永久占着 libcamera**: main.py 那侧的 CSI 识别从此起不来
# （`failed to acquire camera`, 只能人工 kill）, 而本进程的流还在空转烧 CPU。
# ⚠️ 与 main.py 里那份是**各自进程一份**（PICAM 单例是进程内的）, 互不干扰 ✓。
# ⚠️ 顺带每 10s 收一次"自己留下来的"（`reap_stale`）—— 它现在只认"父进程已经没了的",
#    活着的兄弟进程（main.py）在用的流一律不碰, 见 camera_picam._own_stream_pids。
threading.Thread(target=camera_picam.picam_watchdog,
                 name="picam-watchdog-web", daemon=True).start()


def _scan_begin():
    """进入就地识别的公共前奏: 先请上一个退出, 再抢互斥锁（**最多等 30s**）。

    返回 True = 已持锁且 _scan_abort 已清零, 调用方接着跑任务并在 finally 里
    `_scan_lock.release()`; False = 等锁超时 —— 多半是上一个识别卡在阻塞的相机
    调用里, 调用方应直接回一条「忙」的 JSON（旧版 acquire() 无限等, 前端 fetch
    挂死表现为「点什么都不响应, 等很久才恢复」）。
    """
    _scan_abort.set()          # 先请上一个识别尽快退出（它会在下一个 abort 检查点返回）
    if not _scan_lock.acquire(timeout=30.0):
        print("[WARN] 就地识别: 上一个任务 30s 内没结束（可能卡在相机调用里）, 放弃本次请求")
        return False
    _scan_abort.clear()
    return True


def _scan_busy_json_reason():
    return "上一个识别任务仍未结束（已等 30s），请稍后重试或刷新页面"


def _archive_scan_frame(key, result, legacy_name=None):
    """GKS_Web 就地识别的帧也归档进活动 run 目录（2026-10, 与 main.py 的任务帧同处）。

    key: qr_wechat / hostage / color / ...（归档文件名区分来源与算法, 序号递增不覆盖）。
    legacy_name: 镜像到 logs/<legacy_name>（与 main.py 写的固定路径同名, /api/*/frame
    轮询端点读它）。失败只打 WARN —— 归档是旁路。
    2026-10-21 起收**整个 result dict**（与 main.py 侧 handler._save_frame 同一约定）:
      · result["success"] 为 False → logstore 给帧盖红字 FAIL 印再落盘;
      · result["frame_raw"]（处理前原始帧）+ 训练模式开（Web 参数页 TRAIN_MODE_EN,
        经 logstore 里 live_value 两个进程即时生效）→ 同序号配对归档 <key>_before_<n>.jpg;
      · 失败也归档（任务侧保证失败把最后的原始帧放进 result["frame"]）。
    """
    if not isinstance(result, dict) or result.get("frame") is None:
        return
    try:
        legacy = os.path.join(logstore.LOG_DIR, legacy_name) if legacy_name else None
        logstore.archive_frame(key, result.get("frame"), legacy_path=legacy,
                               before_frame=result.get("frame_raw"),
                               ok=result.get("success"))
    except Exception as e:
        print(f"[WARN] 归档页面识别帧失败({key}): {e}")


def _local_ips():
    try:
        out = console.run_cmd("hostname -I")[0]
        return [ip for ip in out.split()
                if ip.count(".") == 3 and not ip.startswith("127.")]
    except Exception:
        return []


def _newest_mainpy_log():
    """返回 main.py 当前日志文件路径，无则 None。

    2026-10 起优先「活动 run 目录的 log.txt」（logs/run_*/log.txt, 由 logstore
    的 main.pid 定位）; 回退旧式平铺 logs/*_log.txt（旧版 main.py 写的）。
    """
    try:
        d = logstore.active_run_dir()
        if d:
            p = os.path.join(d, "log.txt")
            if os.path.exists(p):
                return p
        files = [f for f in os.listdir(logstore.LOG_DIR) if f.endswith("_log.txt")]
        if not files:
            return None
        files.sort()
        return os.path.join(logstore.LOG_DIR, files[-1])
    except Exception:
        return None


# ─────────────────────────── 网页 UI ───────────────────────────
# 注意：用原始字符串 r"""...""" 保留 JS 里的反斜杠（\n、\' 等），
# 否则 Python 会把 \n 转成换行、\' 转成引号，破坏整段 <script>。
HTML_PAGE = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>GKS Web 控制端</title>
<style>
  :root { color-scheme: dark; }
  * { box-sizing: border-box; }
  body { font-family: sans-serif; background:#1e1e2e; color:#e0e0e0; margin:0; padding:16px; }
  h1 { font-size:20px; color:#89b4fa; margin:0 0 12px; }
  .tabnav { display:flex; gap:6px; margin-bottom:12px; flex-wrap:wrap; }
  .tabnav .tab { background:#2a2a3c; color:#a6adc8; border-radius:6px; padding:9px 18px;
                 font-size:14px; font-weight:bold; cursor:pointer; }
  .tabnav .tab.active { background:#1e66f5; color:#fff; }
  .panel { display:none; }
  .panel.active { display:block; }
  .card { background:#262636; border:1px solid #3a3a4e; border-radius:8px; padding:12px; margin-bottom:12px; }
  .card h2 { font-size:15px; color:#cdd6f4; margin:0 0 8px; }
  .row { display:flex; flex-wrap:wrap; gap:8px; align-items:center; }
  button { background:#1e66f5; color:#fff; border:none; border-radius:6px; padding:9px 14px;
           font-size:13px; font-weight:bold; cursor:pointer; }
  button.g { background:#40a02b; } button.r { background:#d20f39; }
  button.o { background:#df8e1d; } button.p { background:#8839ef; }
  button:hover { opacity:0.9; }
  .kv { font-size:13px; color:#a6adc8; line-height:1.6; }
  .kv b { color:#e0e0e0; }
  /* ---- 参数 Tab（照单片机端控制台的「调参」Tab 抄的一套: 分组卡片 + 行内控件）---- */
  .cfg-group { background:#262636; border:1px solid #3a3a4e; border-radius:8px;
               padding:10px 12px 12px; margin-bottom:12px; }
  .cfg-group-head { display:flex; align-items:baseline; gap:8px;
                    border-bottom:1px dashed #3a3a4e; padding-bottom:6px; margin-bottom:7px; }
  .cfg-group-title { font-weight:bold; font-size:13px; color:#cdd6f4; }
  .cfg-group-n { font-size:11px; color:#a6adc8; margin-left:auto; }
  .cfg-item { display:flex; align-items:center; gap:10px; padding:3px 6px; border-radius:5px; }
  .cfg-item:hover { background:#2a2a3c; }
  /* 改过的行: 左边一道蓝条（与调参 Tab 的 .tune-item.changed 同一意思） */
  .cfg-item.changed { box-shadow: inset 3px 0 0 #1e66f5; }
  /* 「中文名 [宏名]」那一格。flex 而不是纯文本: 中文名要让位（ellipsis）, 宏名不能让位。
     ⚠ min-width:0 不能省 —— flex 项的自动最小尺寸是 min-content, nowrap 文本的
       min-content 就是整条文字宽, 不加的话长中文名根本不缩, 把宏名挤出可视区。 */
  .cfg-name { flex:1; min-width:0; font-size:13px; display:flex; align-items:baseline; gap:6px; }
  .cfg-name .cfg-disp { min-width:0; overflow:hidden; text-overflow:ellipsis; white-space:nowrap; }
  .cfg-name .cfg-macro { flex:none; font-family:monospace; font-size:10px; color:#6c7086; cursor:help; }
  .cfg-input { display:flex; align-items:center; gap:5px; flex:none; }
  .cfg-input input[type=text] { width:96px; margin:0; padding:3px 7px; font-family:monospace;
                                font-size:12px; text-align:right; }
  .cfg-input select { margin:0; padding:3px 7px; font-size:12px; min-width:96px; }
  /* 勾选框: 文本框那套宽度/右对齐/等宽字体对它是错的, 这里收窄 */
  .cfg-input input[type=checkbox] { width:auto; margin:0; padding:0; accent-color:#1e66f5;
                                    cursor:pointer; }
  .cfg-opts { font-size:11px; color:#6c7086; min-width:22px; white-space:nowrap; }
  .status { font-size:13px; min-height:18px; margin-top:6px; }
  .ok { color:#40a02b; font-weight:bold; } .bad { color:#d20f39; font-weight:bold; }
  .warn { color:#df8e1d; font-weight:bold; }
  .box { width:100%; aspect-ratio:4/3; background:#111; border-radius:4px; overflow:hidden; }
  .box img { width:100%; height:100%; object-fit:contain; display:block; }
  .ph { width:100%; height:100%; display:flex; align-items:center; justify-content:center;
        color:#6c7086; font-size:13px; background:#181825; }
  .cam-grid { display:flex; flex-wrap:wrap; gap:12px; }
  .cam { flex:1 1 320px; }
  .cam h3 { font-size:13px; color:#cdd6f4; margin:0 0 6px; }
  /* ---- 日志页「相机帧」分类筛选 chips（2026-10-21 加）---- */
  .fchips { display:flex; flex-wrap:wrap; gap:6px; margin:0 0 10px; }
  .fchip { background:#2a2a3c; color:#a6adc8; border:1px solid #3a3a4e; border-radius:999px;
           padding:4px 12px; font-size:12px; font-weight:bold; cursor:pointer; }
  .fchip.on { background:#1e66f5; color:#fff; border-color:#1e66f5; }
  .fchip small { font-weight:normal; opacity:0.8; }
  /* 摄像头预览区固定两列：第三个预览框（pi摄像头）自动落到第二排（主/扫码在上）。
     别改回 .cam-grid 的 flex 自动排 —— 宽屏下三个 320px 的格子会挤成一行。
     ⚠ 只作用于预览区；「回看」的 #framesGrid 仍走 .cam-grid 的流式换行。 */
  .cam-grid.preview { display:grid; grid-template-columns:1fr 1fr; }
  @media (max-width: 820px) { .cam-grid.preview { grid-template-columns:1fr; } }
  pre { margin:8px 0 0; background:#11111b; border:1px solid #3a3a4e; border-radius:6px;
        padding:8px; height:180px; overflow-y:auto; font-family:monospace; font-size:12px;
        color:#cdd6f4; white-space:pre-wrap; word-break:break-all; }
  pre.terminal { background:#0c0c0c; color:#c5e0c5; height:460px; white-space:pre; word-break:normal; }

  /* 日志页：左侧历史运行侧边栏 + 右侧内容（2026-10） */
  .logwrap { display:flex; gap:12px; align-items:flex-start; }
  .logside { flex:0 0 250px; max-height:78vh; overflow-y:auto; }
  .logmain { flex:1 1 auto; min-width:0; }
  .sidelist { margin-top:6px; }
  .logside-item { padding:8px 10px; border-radius:6px; cursor:pointer; margin-top:4px;
                  border:1px solid #3a3a4e; background:#11111b; }
  .logside-item:hover { border-color:#1e66f5; }
  .logside-item.active { background:#1e66f5; border-color:#1e66f5; }
  .logside-item.active .kv { color:#e0e0e0; }
  /* 历史运行右键菜单 */
  .ctxmenu { position:fixed; z-index:80; min-width:160px; padding:4px;
             background:#262636; border:1px solid #3a3a4e; border-radius:8px;
             box-shadow:0 4px 20px rgba(0,0,0,0.6); font-size:13px; }
  .ctxmenu a { display:block; padding:7px 10px; border-radius:6px; cursor:pointer; }
  .ctxmenu a:hover { background:#1e66f5; }
  .ctxmenu .kv { display:block; padding:7px 10px; cursor:default; }
  /* 帧右键「验证」结果弹窗（2026-11-06 加）: 宽版 modal, 输入帧与结果帧并列 */
  .modal-box.vld { width:min(1180px, 96vw); max-height:92vh; overflow:auto; }
  .vld-imgs { display:flex; gap:10px; flex-wrap:wrap; margin-top:6px; }
  .vld-imgs > div { flex:1 1 380px; min-width:260px; }
  .vld-imgs img { width:100%; display:block; border-radius:6px; background:#11111b;
                  border:1px solid #3a3a4e; min-height:48px; }
  @media (max-width: 900px) {
    .logwrap { flex-direction:column; }
    .logside { flex:none; width:100%; max-height:32vh; }
  }

  /* 弹窗（日志清理等） */
  .modal-mask { position:fixed; inset:0; background:rgba(0,0,0,.55); z-index:60;
                display:flex; align-items:center; justify-content:center; }
  .modal-box { width:min(480px, 92vw); margin:0; }
  input[type=text] { background:#11111b; color:#e0e0e0; border:1px solid #3a3a4e;
        border-radius:6px; padding:8px 10px; font-size:13px; }
  .log { color:#a6e3a1; } .log .tx { color:#a6e3a1; } .log .rx { color:#89b4fa; }
  .log .sys { color:#6c7086; }
  select { background:#11111b; color:#e0e0e0; border:1px solid #3a3a4e; border-radius:6px; padding:8px; }
  #toast { position:fixed; top:14px; right:14px; background:#262636; border:1px solid #3a3a4e;
           padding:10px 16px; border-radius:8px; font-size:13px; display:none; max-width:360px; z-index:10; }
  #floatLog { position:fixed; top:56px; right:16px; width:560px; max-width:92vw;
              background:#0c0c0c; border:1px solid #3a3a4e; border-radius:8px; z-index:100;
              box-shadow:0 4px 20px rgba(0,0,0,0.6); display:none; }
  .float-head { display:flex; align-items:center; justify-content:space-between;
                padding:6px 12px; background:#262636; border-radius:8px 8px 0 0;
                cursor:move; user-select:none; }
  .float-head span { font-size:13px; font-weight:bold; color:#cdd6f4; }
  .float-head button { background:transparent; padding:0 8px; font-size:18px; line-height:1; }
  .float-bar { display:flex; align-items:center; gap:8px; padding:6px 12px;
               background:#1e1e2e; border-bottom:1px solid #3a3a4e; font-size:12px; color:#a6adc8; }
  .float-bar input[type=range] { flex:1; }
  #floatLog .terminal { margin:0; border:none; border-radius:0 0 8px 8px; height:380px; }

  /* ---- 主摄参数悬浮窗（模拟器 Tab 的「主摄参数 · 实时预览」按钮弹出, 2026-10-30 加）----
     与 #floatLog 同款悬浮/拖动；自带一路 /stream?cam=main 预览 + 曝光/白平衡控件。
     z-index 比日志悬浮高（120 > 100），两者同时开时参数窗在上面。 */
  #camTune { position:fixed; top:64px; right:20px; width:620px; max-width:94vw;
             max-height:92vh; overflow:auto;
             background:#262636; border:1px solid #3a3a4e; border-radius:8px; z-index:120;
             box-shadow:0 8px 30px rgba(0,0,0,0.7); display:none; }
  #camTune .box { aspect-ratio:16/9; background:#111; border-radius:6px; overflow:hidden;
                  margin:8px 12px; }
  #camTune .box img { width:100%; height:100%; object-fit:contain; display:block; }
  #camTune .ctl-row { display:flex; align-items:center; gap:8px; padding:5px 12px;
                      font-size:13px; color:#cdd6f4; }
  #camTune .ctl-row input[type=range] { flex:1; }
  #camTune .ctl-row input[type=number] { width:88px; margin:0; padding:3px 7px;
                                         font-family:monospace; font-size:12px; text-align:right;
                                         background:#11111b; color:#e0e0e0; border:1px solid #3a3a4e;
                                         border-radius:6px; }
  #camTune .ctl-label { width:78px; color:#a6adc8; }
  #camTune .ctl-val { min-width:86px; font-family:monospace; color:#a6e3a1; }
  #camTune .hint { padding:2px 12px 10px; font-size:11px; color:#6c7086; line-height:1.55; }
</style>
</head>
<body>
<h1>GKS Web 控制端</h1>

<div class="tabnav">
  <button class="tab active" data-tab="control" onclick="showTab('control')">控制</button>
  <button class="tab" data-tab="wifi" onclick="showTab('wifi')">网络</button>
  <button class="tab" data-tab="logs" onclick="showTab('logs')">日志</button>
  <button class="tab" data-tab="sim" onclick="showTab('sim')">模拟器</button>
  <button class="tab" data-tab="config" onclick="showTab('config')">参数</button>
  <button class="tab" onclick="location.href='/parallax'+location.search">视差标定</button>
  <button class="tab" onclick="location.href='/practice'+location.search">标靶练习</button>
</div>

<!-- 控制 -->
<div class="panel active" id="panel-control">
  <div class="card">
    <h2>网络信息</h2>
    <div class="row">
      <div class="kv" id="netInfo">IP: 检测中... | 主机名: -</div>
      <button class="p" onclick="copySsh()">复制 SSH 命令</button>
      <button class="g" onclick="showTab('wifi')">切换 WiFi</button>
    </div>
    <div class="kv" id="netWifi">WiFi: 检测中...</div>
  </div>

  <div class="card">
    <h2>服务控制 (gks.service = GKS 主程序)</h2>
    <div class="status" id="svcStatus">检测中...</div>
    <div class="kv" id="autoStatus">开机自启: 检测中...</div>
    <div class="row" style="margin-top:8px">
      <button class="g" onclick="svc('start')">启动服务</button>
      <button class="r" onclick="svc('stop')">停止服务</button>
      <button onclick="svc('enable')">开启自启</button>
      <button class="o" onclick="svc('disable')">关闭自启</button>
      <button class="g" onclick="mainpy('start')">启动 main.py</button>
      <button class="r" onclick="mainpy('kill')">杀死 main.py</button>
    </div>
  </div>

  <div class="card">
    <h2>Web 控制端自身 (gks-web.service)</h2>
    <div class="status" id="webSvcStatus">检测中...</div>
    <div class="kv" id="webAutoStatus">开机自启: 检测中...</div>
    <div class="kv">改了 app.py / console.py / simulator.py 等源码后，点「重启控制端」才会生效</div>
    <div class="row" style="margin-top:8px">
      <button class="g" onclick="websvc('restart')">重启控制端</button>
      <button onclick="websvc('enable')">开启自启</button>
      <button class="o" onclick="websvc('disable')">关闭自启</button>
    </div>
  </div>

  <div class="card">
    <h2>设备固定 (udev 符号链接)</h2>
    <div class="kv">重新生成 udev 规则，把设备绑成固定节点：
      <b>/dev/camera_main</b>、<b>/dev/camera_Qr</b>、<b>/dev/ttyUSB_GKS</b></div>
    <div class="kv warn">⚠ 脚本按「检测到的先后顺序」给摄像头命名（第一个→camera_main，
      第二个→camera_Qr）——<b>必须主摄与扫码摄像头都已插入</b>才能继续，少插一个会导致角色对调</div>
    <div class="row" style="margin-top:8px">
      <button class="p" onclick="fixDevices()">固定设备</button>
      <span class="kv" id="fixStatus"></span>
    </div>
    <pre id="fixOutput" style="margin-top:8px;max-height:240px;overflow:auto;background:#11111b;
border:1px solid #3a3a4e;border-radius:4px;padding:8px;font-size:12px;
color:#cdd6f4;white-space:pre-wrap;word-break:break-all">（未运行）</pre>
  </div>

  <div class="card">
    <h2>系统状态</h2>
    <div class="kv" id="sysInfo">检测中...</div>
    <div class="kv" id="pwrInfo" style="margin-top:6px">供电: 检测中...</div>
    <div class="row" style="margin-top:8px;align-items:center">
      <button class="o" onclick="clearPowerLog()">清除欠压日志</button>
      <span class="kv">只清事件与计数；本次开机已发生的硬件锁存位需重启树莓派才复位</span>
    </div>
    <pre id="pwrLog" style="height:120px;margin-top:8px;background:#11111b;border:1px solid #3a3a4e;
border-radius:4px;padding:8px;font-size:12px;color:#cdd6f4;white-space:pre-wrap;word-break:break-all">（欠压/限频事件日志）</pre>
  </div>

  <div class="card">
    <h2>电源管理</h2>
    <div class="row">
      <button class="o" onclick="power('reboot')">重启</button>
      <button class="r" onclick="power('shutdown')">关机</button>
    </div>
  </div>
</div>

<!-- 日志 -->
<!-- 网络 / WiFi -->
<div class="panel" id="panel-wifi">
  <div class="card">
    <h2>当前连接</h2>
    <div class="kv" id="wifiNow">检测中...</div>
    <div class="status" id="wifiSwitchState"></div>
    <div class="row" style="margin-top:8px">
      <button class="p" onclick="loadWifi()">刷新</button>
      <button onclick="copyWifiUrl()">复制本页地址（xixi.local）</button>
      <button class="o" onclick="wifiRevert()">退回上一个网络</button>
    </div>
  </div>

  <div class="card">
    <h2>搜索 WiFi</h2>
    <div class="row">
      <button class="g" id="wifiScanBtn" onclick="wifiScan()">扫描</button>
      <span class="kv" id="wifiScanInfo">点「扫描」列出附近网络（约 2 秒）</span>
    </div>
    <div id="wifiList"></div>
  </div>

  <div class="card">
    <h2>已保存的网络（开机自动连接顺序）</h2>
    <div class="kv warn">数字<b>越大越优先</b>。开机时只会在<b>当前范围内</b>的网络里按优先级挑第一个；
      首选网络不在范围内（比如手机热点没开）会自动落到下一个 —— 所以别删掉能用的那些。
      <br>⚠ 「未保存密码」的档案开机连它必然失败（会白等一轮再退到下一个），建议补密码或删掉。</div>
    <div id="wifiProfiles"></div>
  </div>
</div>

<div class="panel" id="panel-logs">
  <div class="logwrap">
    <div class="card logside">
      <h2>历史运行</h2>
      <div class="row">
        <button onclick="loadRuns()">刷新</button>
        <span class="kv" id="runsInfo"></span>
      </div>
      <div id="runsList" class="sidelist"></div>
      <div id="runCtx" class="ctxmenu" style="display:none"></div>
    </div>

    <div class="logmain">
      <div class="card">
        <h2>实时日志（终端同步）</h2>
        <div class="row">
          <select id="logUnit" onchange="startLogStream()">
            <option value="mainpy">main.py 日志（文件）</option>
            <option value="gks.service">gks.service（服务）</option>
            <option value="gks-web.service">gks-web.service（本控制端）</option>
            <option value="sim">模拟器日志（帧交换）</option>
          </select>
          <button class="g" onclick="mainpy('restart')">开始/重启 main.py</button>
          <button class="r" onclick="mainpy('kill')">停止 main.py</button>
          <button class="g" onclick="svc('restart')">开始/重启服务</button>
          <button class="r" onclick="svc('stop')">停止服务</button>
          <button class="p" onclick="clearLogArea()">清屏</button>
          <button onclick="floatLog()">悬浮</button>
          <button class="o" onclick="showLogClean()">清理日志</button>
        </div>
        <div class="kv">「清屏」会把 main.py / 模拟器的日志<b>文件/缓冲一并清空</b>（不止是本页显示）；
          gks.service / gks-web 两类是 systemd journal，只能本页不回显。</div>
        <div class="row" style="margin-top:6px; align-items:center; gap:8px; flex-wrap:wrap">
          <label style="display:inline-flex; align-items:center; gap:4px; cursor:pointer">
            <input type="checkbox" id="autoQrChk" onchange="autoQrApply()"> 自动下发二维码（调试）
          </label>
          <input id="autoQrVal" placeholder="每位1~3" maxlength="3" size="6"
                 style="width:7em; text-align:center" inputmode="numeric" autocomplete="off"
                 oninput="autoQrApply()" onfocus="this.select()" onclick="this.select()"
                 title="点击后原有数字全选，直接重打即替换">
          <span class="kv" id="autoQrHint"></span>
        </div>
        <pre id="logArea" class="terminal"></pre>
        <div class="row" style="margin-top:6px">
          <button class="p" onclick="backToLive()" id="backLiveBtn" style="display:none">← 返回实时日志</button>
          <span class="kv" id="viewingRunInfo"></span>
        </div>
      </div>

      <div class="card" id="framesCard" style="display:none">
        <h2 id="framesTitle">相机帧</h2>
        <!-- 分类筛选（2026-10-21 加）: chips 由 loadRunFrames 按现有帧现搭, 空分类置灰 -->
        <div id="frameFilters" class="fchips" style="display:none"></div>
        <div id="framesGrid" class="cam-grid"></div>
        <!-- 帧右键菜单（「验证」= 离线复跑当前算法, 2026-11-06）; 纪律同 #runCtx -->
        <div id="frameCtx" class="ctxmenu" style="display:none"></div>
      </div>
    </div>
  </div>
</div>

<!-- 模拟器（含摄像头测试） -->
<div class="panel" id="panel-sim">
  <div class="card">
    <h2>单片机模拟器</h2>
    <div class="status" id="simStatus">检测中...</div>
    <div class="row" style="margin-top:8px">
      <button class="g" onclick="simStart()">启动模拟器</button>
      <button class="r" onclick="simStop()">停止模拟器</button>
    </div>
    <div class="row" style="margin-top:8px" id="simButtons"></div>
    <div class="row" style="margin-top:8px">
      <input type="text" id="simCustom" placeholder="自定义指令 (如 '1' / 'end')" style="flex:1"
             onkeydown="if(event.key==='Enter')simSendCustom()">
      <button class="g" onclick="simSendCustom()">发送</button>
    </div>
    <hr style="border:none;border-top:1px solid #3a3a4e;margin:14px 0">
    <div class="row">
      <button class="g" onclick="checkCamera('main')">测试主摄</button>
      <button class="g" onclick="checkCamera('qr')">测试扫码</button>
      <button class="g" onclick="checkCamera('pi')">测试pi</button>
      <button onclick="checkCamera('all')">三路一起测</button>
      <button class="r" onclick="stopCam()">退出</button>
    </div>
    <div class="row" style="margin-top:8px">
      <button onclick="scanQR('opencv')">扫码二维码(OpenCV)</button>
      <button onclick="scanQR('pyzbar')">扫码二维码(pyzbar)</button>
      <button onclick="scanQR('wechat')">副摄·微信</button>
      <button onclick="scanQR('auto')">副摄·自动(新)</button>
      <button onclick="scanQR('diff')">副摄·自研</button>
      <button onclick="scanQR('wechat_main')">主摄·微信</button>
      <button onclick="scanQR('auto_main')">主摄·自动(新)</button>
      <button onclick="scanQR('diff_main')">主摄·自研</button>
      <button class="p" onclick="scanHostage()">人质识别(YOLO)</button>
      <button class="p" onclick="scanColor()">色球识别(HSV)</button>
      <button class="p" onclick="scanTarget()">靶标识别(靶心)</button>
      <button class="p" onclick="scanTarget(1)">靶标识别(pi)</button>
      <button class="p" onclick="scanRect()">色球区矩形(黑框)</button>
      <button class="p" onclick="scanRoad()">看路(弯道角点)</button>
      <button class="p" onclick="scanRoadYaw()">看路校准(直边角)</button>
    </div>
    <div class="status" id="camStatus">就绪</div>
    <div class="cam-grid preview" style="margin-top:8px">
      <div class="cam"><h3>主摄像头 /dev/camera_main</h3>
        <div class="box" id="box-main"><div class="ph">等待开始...</div></div></div>
      <div class="cam"><h3>扫码摄像头 /dev/camera_Qr</h3>
        <div class="box" id="box-qr"><div class="ph">等待开始...</div></div></div>
      <div class="cam"><h3>pi摄像头</h3>
        <div class="box" id="box-pi"><div class="ph">等待开始...</div></div></div>
    </div>
    <div class="row" style="margin-top:8px">
      <button class="p" onclick="openCamTune()">主摄参数 · 实时预览（悬浮窗）</button>
      <span class="kv">悬浮窗可拖动；只改预览流，不写 config</span>
    </div>
  </div>
</div>

<!-- 参数（改 GKS_Competition/config.py 的可编辑项） -->
<div class="panel" id="panel-config">
  <div class="card">
    <h2>参数设置（覆盖 GKS_Competition/config.py）</h2>
    <div class="kv">改动写入 <b>config_override.json</b>，<b>重启 gks.service 后</b>对主程序生效
      —— 本控制端与主程序是两个进程，各在启动时读这份文件，所以保存后必须重启。</div>
    <div class="row" style="margin-top:8px">
      <button class="g" onclick="saveConfig()">保存所有修改</button>
      <button class="p" onclick="loadConfig()">重新读取</button>
      <button class="g" onclick="svc('restart')">重启服务</button>
      <span class="status" id="cfgStatus">未加载</span>
    </div>
    <div id="cfgList"></div>
  </div>
</div>

<div id="toast"></div>

<!-- 日志清理弹窗（日志 Tab「清理日志」按钮, 2026-10）: 点遮罩空白处也可关闭 -->
<div id="logCleanModal" class="modal-mask" style="display:none" onclick="if(event.target===this)closeLogClean()">
  <div class="card modal-box">
    <h2>清理日志（logs/ 目录）</h2>
    <div class="kv">清理对象 = 各次运行的目录（日志 + 相机帧归档）与旧式日志文件；
      <b>正在运行的那一次不会被删</b>（文件正被 main.py 写着）。删除后不可恢复。</div>
    <div class="row" style="margin-top:10px">
      <button class="o" onclick="doCleanLogs('today')">只保留今天的</button>
      <button class="o" onclick="doCleanLogs('week')">只保留最近 7 天</button>
      <button class="r" onclick="doCleanLogs('all')">全部清空</button>
    </div>
    <div class="row" style="margin-top:8px">
      <button onclick="closeLogClean()">取消</button>
      <span class="kv" id="logCleanState"></span>
    </div>
  </div>
</div>

<!-- 帧离线验证弹窗（日志页帧右键「验证」, 2026-11-06）: 输入帧 | 当前代码标注结果 并列 -->
<div id="validateModal" class="modal-mask" style="display:none" onclick="if(event.target===this)closeValidate()">
  <div class="card modal-box vld">
    <h2 id="vldTitle">离线验证</h2>
    <div class="kv" id="vldMeta"></div>
    <div class="vld-imgs">
      <div><div class="kv">输入帧（喂给算法的图）</div><img id="vldIn" alt=""></div>
      <div><div class="kv">验证结果（当前代码标注）</div><img id="vldOut" alt=""></div>
    </div>
    <div class="kv" id="vldInfo" style="margin-top:8px; white-space:pre-wrap"></div>
    <div class="row" style="margin-top:10px">
      <button onclick="closeValidate()">关闭</button>
      <button class="o" id="vldOpenBtn" style="display:none" onclick="window.open(el('vldOut').src)">结果图新窗口打开</button>
      <span class="kv" id="vldState"></span>
    </div>
  </div>
</div>

<div id="floatLog">
  <div class="float-head"><span>日志悬浮</span><button onclick="closeFloat()">×</button></div>
  <div class="float-bar">
    <select id="floatUnit" onchange="floatUnitChange(this.value)">
      <option value="mainpy">main.py 日志（文件）</option>
      <option value="gks.service">gks.service（服务）</option>
      <option value="gks-web.service">gks-web.service（本控制端）</option>
      <option value="sim">模拟器日志（帧交换）</option>
    </select>
    <button class="p" onclick="clearFloatArea()">清屏</button>
    <span>透明度</span><input type="range" id="floatOpacity" min="20" max="100" value="100" oninput="setOpacity(this.value)">
  </div>
  <pre id="floatLogArea" class="terminal"></pre>
</div>

<!-- 主摄参数悬浮窗（模拟器 Tab 的「主摄参数 · 实时预览」按钮弹出; 可拖动） -->
<div id="camTune">
  <div class="float-head"><span>主摄参数 · 实时预览</span><button onclick="closeCamTune()">×</button></div>
  <div class="float-bar">
    <button class="g" onclick="camTuneOpenStream()">打开预览</button>
    <button onclick="camTuneRead()">读取当前</button>
    <select id="camTuneProfile" title="载入哪个档位的默认值">
      <option value="main">主摄档</option>
      <option value="road">看路档</option>
      <option value="hostage">人质档</option>
    </select>
    <button class="p" onclick="camTuneLoadProfile()">载入档位</button>
    <span class="status" id="camTuneState" style="margin:0;font-size:11px">未连接</span>
  </div>
  <div class="box" id="camTuneBox"><div class="ph">点「打开预览」...</div></div>
  <div class="ctl-row">
    <span class="ctl-label">白平衡</span>
    <select id="camTuneWbMode" onchange="camTuneWbModeChange()">
      <option value="1">自动 AWB</option><option value="0">固定色温</option></select>
    <input type="range" id="camTuneWbRange" min="2800" max="6500" step="10" value="6500"
           oninput="camTuneWbSlider()">
    <input type="number" id="camTuneWbVal" min="2800" max="6500" step="10" value="6500"
           onchange="camTuneWbInput()">
    <span class="ctl-val" id="camTuneWbNow">—</span>
  </div>
  <div class="ctl-row">
    <span class="ctl-label">曝光</span>
    <select id="camTuneExpMode" onchange="camTuneExpModeChange()">
      <option value="3">自动 AE</option><option value="1">手动</option></select>
    <input type="range" id="camTuneExpRange" min="1" max="5000" step="1" value="166"
           oninput="camTuneExpSlider()">
    <input type="number" id="camTuneExpVal" min="1" max="5000" step="1" value="166"
           onchange="camTuneExpInput()">
    <span class="ctl-val" id="camTuneExpNow">—</span>
  </div>
  <div class="ctl-row">
    <span class="kv">拖动即应用（防抖 300ms）；自动档要 1~2s 收敛。</span>
    <button class="g" onclick="camTuneApplyNow()">立即应用</button>
  </div>
  <div class="hint">只改这个预览流的 V4L2 控件，不写 config；关预览/重开相机会按档位重设。
    调好后把读数填到「参数」页的 CAM_MAIN_* / ROAD_*，保存并重启 gks.service 才对主程序生效。
    主摄被主程序占用时预览会打不开（先停任务/退出预览）。</div>
</div>

<script>
// 访问令牌（服务端在 _send_html 里替换占位符注入; WEB_KEY 为空时为空串, 头会被服务端忽略）。
// 所有 fetch 统一补 X-GKS-Key 头 —— POST 的认证只认它或 ?key=（cookie 挡不住 CSRF,
// 见服务端 _auth_post_ok）; EventSource 发不了自定义头, 靠首次 ?key= 进入时种的 cookie。
const GKS_KEY = "__GKS_KEY__";
(function(){
  const _fetch = window.fetch;
  window.fetch = function(u, o){
    try { o = o || {}; o.headers = Object.assign({}, o.headers || {},
             {"X-GKS-Key": GKS_KEY}); } catch(e) {}
    return _fetch(u, o);
  };
})();

let state = { ip: [], hostname: '-', service_active: false, service_enabled: false, mainpy_running: false };
let simCommands = [];
let logSource = null;
let clearedUnits = {};   // unit -> true：已清屏，重连时不回显历史（见 clearLogArea）

function el(id){ return document.getElementById(id); }
function ph(t){ return '<div class="ph">' + (t||'等待开始...') + '</div>'; }
function esc(s){ return String(s).replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;'); }

// ---- 轻提示 ----
function toast(msg){
  const t = el('toast'); t.textContent = msg; t.style.display = 'block';
  clearTimeout(toast._t); toast._t = setTimeout(()=>{ t.style.display='none'; }, 2500);
}

// ---- Tab 切换 ----
function showTab(name){
  document.querySelectorAll('.tabnav .tab').forEach(b => b.classList.toggle('active', b.dataset.tab === name));
  document.querySelectorAll('.panel').forEach(p => p.classList.toggle('active', p.id === 'panel-' + name));
  // 回看历史时（viewingRun 非 null）不重启实时流 —— 否则切个 Tab 回来, 回看内容
  // 就被实时流冲掉了; 「返回实时日志」按钮才是退出回看的唯一出口。
  if (name === 'logs') { if (!viewingRun) startLogStream(); loadRuns(); }
  else stopLogStream();
  // 参数只在进入该 tab 时拉一次（切走不拉；改完点「重新读取」也能刷新）
  if (name === 'config') loadConfig();
  // 网络同理；另外只在网络 tab 上才轮询切换进度（换网时其它 tab 不用管）
  if (name === 'wifi') loadWifi();
}

// ---- 状态轮询 ----
async function refreshStatus(){
  try {
    const r = await fetch('/api/status');
    state = await r.json();
    el('netInfo').innerHTML = 'IP: <b>' + esc(state.ip.join(', ')) + '</b> | 主机名: <b>' + esc(state.hostname) + '</b>';
    if (el('netWifi')) el('netWifi').innerHTML = 'WiFi: <b>' + (lastWifiSsid ? esc(lastWifiSsid) : '未连接') + '</b>';
    if (state.service_active) { el('svcStatus').innerHTML = '<span class="ok">● 运行中</span> (服务模式)'; }
    else if (state.mainpy_running) { el('svcStatus').innerHTML = '<span class="warn">● 运行中</span> (main.py 直接运行，非服务模式)'; }
    else { el('svcStatus').innerHTML = '<span class="bad">● 已停止</span>'; }
    el('autoStatus').innerHTML = '开机自启: <b>' + (state.service_enabled ? '已启用' : '已禁用') + '</b>';
    // Web 控制端自身：active=false 但页面还能打开 → 多半是手动跑的（非 systemd 托管）
    el('webSvcStatus').innerHTML = state.web_service_active
      ? '<span class="ok">● 服务运行中</span>（本页面即由它提供）'
      : '<span class="warn">● systemd 未托管</span>（页面可访问 = 手动启动中）';
    el('webAutoStatus').innerHTML = '开机自启: <b>' + (state.web_service_enabled ? '已启用' : '已禁用') + '</b>';
    const s = state.system || {};
    let sys = '';
    sys += 'CPU温度: <b>' + (s.cpu_temp ?? '-') + '℃</b>　';
    if (s.load) sys += '负载: <b>' + s.load['1min'] + ' / ' + s.load['5min'] + ' / ' + s.load['15min'] + '</b>　';
    if (s.mem) sys += '内存: <b>' + Math.round(s.mem.used_kb/1024) + ' / ' + Math.round(s.mem.total_kb/1024) + ' MB</b>　';
    if (s.disk) sys += '磁盘: <b>' + s.disk.used + ' / ' + s.disk.size + ' (' + s.disk['use%'] + ')</b>　';
    if (s.uptime_sec != null) sys += '运行: <b>' + fmtUptime(s.uptime_sec) + '</b>';
    el('sysInfo').innerHTML = sys || '无数据';
    renderPower(state.power || {});
  } catch(e) { el('sysInfo').innerHTML = '状态获取失败: ' + e; }
}
function fmtUptime(sec){
  const d = Math.floor(sec/86400), h = Math.floor(sec%86400/3600), m = Math.floor(sec%3600/60);
  return (d?d+'天':'') + h + '时' + m + '分';
}

// ---- 供电/欠压（服务端 _PowerMonitor 常驻采样，这里只渲染）----
// 为什么要看这个：欠压常是几毫秒的瞬时跌落，症状是「USB 设备反复掉线重连 / 相机突然
// 打不开 / 掉盘」，而事后去问 vcgencmd 只剩一个「发生过」的位、没有时间。这里给的是
// 带时间戳的事件流水，正好用来跟日志里的故障时刻对齐。
function renderPower(p){
  if (!p.ok) {
    el('pwrInfo').innerHTML = '供电: <span class="bad">读取失败</span>' +
      '<span class="kv">（vcgencmd 不可用，无法监控欠压）</span>';
    return;
  }
  let t = '供电: ';
  if (p.undervolt_now) t += '<span class="bad">⚠ 正在欠压</span>';
  else if (p.undervolt_was) t += '<span class="warn">本次开机发生过欠压（当前正常）</span>';
  else t += '<span class="ok">正常</span>';
  t += '　欠压次数: <b>' + (p.uv_count|0) + '</b>';
  if (p.uv_last) t += '　最近: <b>' + esc(p.uv_last) + '</b>';
  if (p.throttled_now) t += '　<span class="warn">⚠ 正在限频</span>';
  el('pwrInfo').innerHTML = t;

  const log = el('pwrLog');
  if (log) {
    const evs = p.events || [];
    const txt = evs.length ? evs.join('\n') : '（暂无欠压/限频事件）';
    if (log.textContent !== txt) { log.textContent = txt; log.scrollTop = log.scrollHeight; }
  }
}

// 清除欠压/限频事件日志（只清软件记录；硬件锁存位是「本次开机发生过」,
// 只有重启树莓派才复位 —— 清完 undervolt_was 仍可能显示 True, 不是没清干净）。
async function clearPowerLog(){
  if (!confirm('确定清除全部欠压/限频日志与计数？\n（本次开机已发生的硬件锁存位不会复位，重启树莓派后才清）')) return;
  try {
    const d = await postJSON('/api/power', {action:'clear_uv_log'});
    toast(d.message);
    refreshStatus();
  } catch(e) {
    toast('清除失败: ' + e);
  }
}

// ---- 服务 / main.py / 电源 ----
async function postJSON(url, body){
  const r = await fetch(url, { method:'POST', headers:{'Content-Type':'application/json'}, body: JSON.stringify(body) });
  // 非 JSON 响应（多为路由不存在/服务端异常返回的错误页）要抛出可读错误，
  // 否则 r.json() 抛 SyntaxError，调用方若无 try/catch 会静默卡在「进行中」状态
  const text = await r.text();
  try {
    return JSON.parse(text);
  } catch(e) {
    throw new Error('HTTP ' + r.status + '（' + url + '）' +
                    (r.status === 404 ? '：接口不存在，多半是改完源码没重启控制端' : ''));
  }
}
async function svc(action){ const d = await postJSON('/api/service', {action}); toast(d.message); refreshStatus(); }
async function mainpy(action){ const d = await postJSON('/api/mainpy', {action}); toast(d.message); refreshStatus(); }

// ---- 自动下发二维码（跨进程：POST /api/autoqr → GKS_Competition/web_auto_qr.json，
//      main.py 在握手成功/收到扫码指令时读它下发；值域 111~333 与服务端同校验） ----
function autoQrValid(){
  const v = el('autoQrVal').value.trim();
  return /^[1-3]{3}$/.test(v) ? v : null;   // 每位 1~3（色/靶/形，与固件同域）
}
function autoQrUi(){
  const chk = el('autoQrChk'), inp = el('autoQrVal'), hint = el('autoQrHint');
  const raw = inp.value.trim(), v = autoQrValid();
  inp.style.borderColor = ''; inp.style.background = ''; inp.style.borderWidth = '';
  hint.style.color = '';
  if (raw === '') {
    hint.textContent = '未启用（勾选并填三位数、每位 1~3（如 213）后，握手成功即自动下发到单片机）';
  } else if (raw.length < 3) {
    // 输到一半（1~2 位）只给中性进度提示 —— 这时弹黄框会让人以为输入坏了
    hint.textContent = '再输入 ' + (3 - raw.length) + ' 位…';
  } else if (!v) {      // 满 3 位但有数字不在 1~3：才是真正该黄框提醒"不会下发"的时刻
    inp.style.borderColor = '#e0b400'; inp.style.background = '#fff9db'; inp.style.borderWidth = '2px';
    hint.textContent = '⚠ 每位数字须为 1~3（百/十/个位各是 1红2绿3蓝 一类的编码）：当前值不会下发（点击输入框重打可整体替换）';
    hint.style.color = '#b38f00';
  } else if (chk.checked) {
    hint.textContent = '已启用：握手成功即下发 QR' + v + '；之后在这里改成新值会立即同步到单片机（OLED 随即更新）⚠ 调试用，赛前记得取消';
  } else {
    hint.textContent = '数值有效但未勾选';
  }
}
async function autoQrApply(){
  // 输入法可能塞进非数字（中文/全角/粘贴文本）—— 只留数字、封顶 3 位，坏字符当场清掉
  const inpEl = el('autoQrVal');
  const cleaned = inpEl.value.replace(/\D/g, '').slice(0, 3);
  if (inpEl.value !== cleaned) inpEl.value = cleaned;
  autoQrUi();
  const enabled = el('autoQrChk').checked;
  const value = cleaned;
  if (enabled && !autoQrValid()) return;    // 勾了但值非法：不写文件，黄框已提示
  try { await postJSON('/api/autoqr', {enabled, value}); }
  catch(e) { toast('自动二维码保存失败: ' + e.message); }
}
async function autoQrRefresh(){
  // ⚠️ 正在被编辑就跳过这一轮（activeElement 判据, 与单片机控制台蓝牙参数表同一套）:
  //    2026-10-18 起本函数由 2s 定时轮询调用 —— 单片机 OLED 第 2 页也能改这份设置
  //    （"QA/QE/QU/QC" 指令族）, 不轮询的话勾选框永远停在页面加载时的旧状态;
  //    而无脑覆盖会把"正打到一半的三位数"清掉。保存仍只由 autoQrSave（change 事件）触发,
  //    轮询只读不写 —— 但若用户看到旧勾选就顺手去点一下, 旧状态会被写回去,
  //    所以轮询本身就是在治"陈旧勾选反写"那个坑。
  const a = document.activeElement;
  if (a && (a.id === 'autoQrVal' || a.id === 'autoQrChk')) return;
  try {
    const r = await fetch('/api/autoqr');
    const d = await r.json();
    el('autoQrChk').checked = !!d.enabled;
    el('autoQrVal').value = d.value || '';
    autoQrUi();
  } catch(e) {}
}

// ---- 参数设置（GKS_Competition/config.py 的 EDITABLE_CONFIG）----
// GET /config → [{name,type,desc,opts,value}]；POST /config ← {参数名: 值} → {ok,message}。
// 两个端点在本文件的服务端早已存在，这里只是补 UI（勿再往路由里加一份）。
// ⚠ 保存只改 config_override.json + **本控制端进程**的内存，主程序是另一个进程、
//   下次启动才读 → 改完必须重启 gks.service（下面的「重启服务」按钮 = svc('restart')）。
// 渲染刻意照单片机端控制台的「调参」Tab 抄（同一个信息密度下的可读性）：
// 分组卡片 + 「中文名 [宏名]」一行 + 值控件；布尔项画勾选框、带选项表的画下拉框，
// 依据就是后端第 4 字段 opts（"0=关|1=开" 这种，config.py 里写死的）。
let cfgItems = [];
/* 前缀 → 组名。⚠️ 必须覆盖 EDITABLE_CONFIG 的**全部**前缀: 漏一个不会报错,
 * 只会静默落进「其它」—— 原来只有 8 条, ROAD_/ROADYAW_ 那 41 项全在「其它」里。
 * 数组顺序 = 显示顺序（config.py 里的顺序另有一份, 这里按任务重排更好找）。 */
const CFG_GROUPS = [
  ['QR_',      '二维码'],
  ['CMD_',     '指令'],
  ['HOSTAGE_', '人质识别'],
  ['COLOR_',   '色球识别'],
  ['RECT_',    '色球区矩形'],
  ['TARGET_',  '靶标识别'],
  ['ROAD_',    '看路'],
  ['ROADYAW_', '看路方向'],
  ['LINE_',    '巡线'],
  ['OPENMV_',  'OpenMV'],
  ['PICAM_',   'pi摄像头'],
  ['CAM_',     '摄像头'],
  ['FRAME_',   '画面尺寸'],
  ['LEASE_',   '相机租约'],
  ['TRAIN_',   '训练模式'],
  ['SYSTEM_',  '系统'],
];
function cfgGroup(name){
  for (const [p, label] of CFG_GROUPS) if (name.startsWith(p)) return label;
  return '其它';
}
/* 选项表 → [{value,label}]。opts 形如 "0=关|1=开"; 只有一段（没有 '='）时
 * 标签就用值本身。空表返回 null（= 自由数值, 画输入框）。 */
function cfgOptions(opts){
  if (!opts) return null;
  const out = [];
  for (const seg of String(opts).split('|')) {
    const i = seg.indexOf('=');
    if (i < 0) out.push({ value: seg.trim(), label: seg.trim() });
    else out.push({ value: seg.slice(0, i).trim(), label: seg.slice(i + 1).trim() });
  }
  return out.length ? out : null;
}
/* 布尔项: 选项表恰好 {0,1} 两项（"0=关|1=开"/"0=固定|1=自动"/"0=无色|1=带色" 都算）。
 * 判据与单片机端调参 Tab 同源: 看的是选项表的键, 不是标签文字。 */
function cfgIsBool(opts){
  const o = cfgOptions(opts);
  return !!o && o.length === 2 && o[0].value === '0' && o[1].value === '1';
}
/* 值控件（勾选框 / 下拉框 / 文本框）。统一挂 data-orig = 进入页面时的值,
 * saveConfig 靠它挑「改动过的项」; 勾选框/下拉框的变更也要自己同步 .changed 高亮。 */
function cfgControl(it){
  const cur = String(it.value);
  const opts = cfgOptions(it.opts);
  if (cfgIsBool(it.opts)) {
    const chk = document.createElement('input');
    chk.type = 'checkbox'; chk.id = 'cfg-' + it.name;
    chk.value = '1';                       // 勾选框的 .value 恒为 '1', 读的时候看 checked
    chk.checked = (cur === '1');
    chk.title = '勾选 = 1（开），不勾 = 0（关）';
    chk.addEventListener('change', () => cfgMarkDirty(it, chk.checked ? '1' : '0'));
    return chk;
  }
  if (opts) {
    const sel = document.createElement('select');
    sel.id = 'cfg-' + it.name;
    let matched = false;
    for (const o of opts) {
      const opt = document.createElement('option');
      opt.value = o.value; opt.textContent = o.label;
      if (o.value === cur) { opt.selected = true; matched = true; }
      sel.appendChild(opt);
    }
    /* 当前值不在选项表里（手改过 config_override.json / 枚举外加的值）:
     * 补一个"(枚举外)"项如实显示, 不悄悄换成某个合法值 —— 与调参 Tab 同一套。 */
    if (!matched) {
      const opt = document.createElement('option');
      opt.value = cur; opt.textContent = cur + '（枚举外）'; opt.selected = true;
      sel.appendChild(opt);
    }
    sel.title = it.name + '（' + it.type + '）';
    sel.addEventListener('change', () => cfgMarkDirty(it, sel.value));
    return sel;
  }
  const inp = document.createElement('input');
  inp.type = 'text'; inp.id = 'cfg-' + it.name; inp.value = cur;
  inp.title = it.name + '（' + it.type + '）';
  inp.addEventListener('input', () => cfgMarkDirty(it, inp.value));
  return inp;
}
/* 控件当前值（字符串形式）。勾选框读 checked, 其余读 value。 */
function cfgValueOf(it){
  const c = el('cfg-' + it.name);
  if (!c) return null;
  return (c.type === 'checkbox') ? (c.checked ? '1' : '0') : c.value;
}
function cfgMarkDirty(it, val){
  const row = document.querySelector('.cfg-item[data-name="' + it.name + '"]');
  if (row) row.classList.toggle('changed', val !== String(it.value));
}
async function loadConfig(){
  el('cfgStatus').innerHTML = '<span class="warn">读取中...</span>';
  try {
    const r = await fetch('/config');
    if (!r.ok) throw new Error('HTTP ' + r.status + '（/config）');
    cfgItems = await r.json();
  } catch(e) {
    el('cfgStatus').innerHTML = '<span class="bad">● 读取失败</span>';
    el('cfgList').innerHTML = '';
    toast('参数读取失败: ' + e);
    return;
  }
  // 按前缀分组；组内保持后端返回的顺序（= config.py 里 EDITABLE_CONFIG 的顺序，便于对照源码）
  const buckets = new Map();
  for (const it of cfgItems) {
    const g = cfgGroup(it.name);
    if (!buckets.has(g)) buckets.set(g, []);
    buckets.get(g).push(it);
  }
  // 组序按 CFG_GROUPS（未知前缀的「其它」排最后），不按首次出现顺序 —— 否则组序会随
  // config.py 里随手插入新参数而乱跳。
  const order = CFG_GROUPS.map(([, label]) => label).filter(l => buckets.has(l));
  if (buckets.has('其它')) order.push('其它');

  const root = el('cfgList');
  root.textContent = '';
  for (const g of order) {
    const items = buckets.get(g);
    const card = document.createElement('section');
    card.className = 'cfg-group';
    card.dataset.group = g;
    const head = document.createElement('div');
    head.className = 'cfg-group-head';
    head.appendChild(el2('span', 'cfg-group-title', g));
    head.appendChild(el2('span', 'cfg-group-n', items.length + ' 项'));
    card.appendChild(head);
    for (const it of items) {
      const row = document.createElement('div');
      row.className = 'cfg-item';
      row.dataset.name = it.name;
      const name = document.createElement('div');
      name.className = 'cfg-name';
      name.title = it.desc + '\n' + it.name + '（' + it.type + '）';
      name.appendChild(el2('span', 'cfg-disp', it.desc || it.name));
      name.appendChild(el2('code', 'cfg-macro', it.name));
      row.appendChild(name);
      const box = document.createElement('div');
      box.className = 'cfg-input';
      box.appendChild(cfgControl(it));
      const tail = document.createElement('span');
      tail.className = 'cfg-opts';
      tail.textContent = cfgIsBool(it.opts) ? '0/1' : (cfgOptions(it.opts) ? it.opts.replace(/\|/g, ' ') : '');
      box.appendChild(tail);
      row.appendChild(box);
      card.appendChild(row);
    }
    root.appendChild(card);
  }
  if (!cfgItems.length) root.textContent = '(无可调参数)';
  el('cfgStatus').innerHTML = '<span class="ok">已加载 ' + cfgItems.length + ' 项</span>';
}
/* 建元素的小工具: el2(tag, className, text) —— text 走 textContent, 不拼 HTML */
function el2(tag, cls, text){
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined && text !== null) e.textContent = text;
  return e;
}
async function saveConfig(){
  // 只提交改动过的项：后端逐项 save_override，全量提交会把没改的也重写一遍
  const changed = {};
  for (const it of cfgItems) {
    const v = cfgValueOf(it);
    if (v !== null && v !== String(it.value)) changed[it.name] = v;
  }
  const n = Object.keys(changed).length;
  if (!n) { el('cfgStatus').innerHTML = '<span class="warn">无修改</span>'; toast('无修改'); return; }
  el('cfgStatus').innerHTML = '<span class="warn">保存中... (' + n + ' 项)</span>';
  try {
    const d = await postJSON('/config', changed);
    if (d.ok) {
      el('cfgStatus').innerHTML = '<span class="ok">● 已写入 config_override.json（' + n
        + ' 项），点「重启服务」生效</span>';
    } else {
      el('cfgStatus').innerHTML = '<span class="bad">● 保存失败</span>';
    }
    toast(d.message || '(无消息)');
    loadConfig();   // 回读：把类型转换失败/被拒绝的项还原成真实生效值
  } catch(e) {
    // postJSON 对非 JSON 响应抛错（如 404），这里必须接住，否则静默卡在「保存中」
    el('cfgStatus').innerHTML = '<span class="bad">● 请求失败</span>';
    toast('保存失败: ' + e);
  }
}
async function fixDevices(){
  // 必须两个摄像头都插着：脚本按检测顺序命名，少插一个会让 camera_main / camera_Qr 角色对调
  if (!confirm('固定设备前请确认两个设备都已插入：\n\n' +
               '① 主摄像头（将绑为 /dev/camera_main）\n' +
               '② 扫码摄像头（将绑为 /dev/camera_Qr）\n\n' +
               '脚本按「检测到的先后顺序」命名摄像头，少插一个会导致两者角色对调，\n' +
               '必须都插好才能继续。\n\n' +
               '确定两个设备都已插入？')) return;
  el('fixStatus').innerHTML = '<span class="warn">正在重新检测并生成 udev 规则...</span>';
  el('fixOutput').textContent = '';
  let d;
  try {
    d = await postJSON('/api/fixdevices', {});
  } catch(e) {
    el('fixStatus').innerHTML = '<span class="bad">● 请求失败</span>';
    el('fixOutput').textContent = String(e);
    toast('固定设备请求失败: ' + e);
    return;
  }
  el('fixStatus').innerHTML = d.ok ? '<span class="ok">● 完成</span>'
                                   : '<span class="bad">● 失败</span>';
  el('fixOutput').textContent = d.output || '(无输出)';
  toast(d.ok ? '设备固定完成，见下方输出与验证结果' : '设备固定失败，见下方输出');
}
async function websvc(action){
  if (action === 'restart' && !confirm('重启 Web 控制端？\n页面会短暂断开，约 2-3 秒后自动恢复（不必手动刷新）。')) return;
  const d = await postJSON('/api/webservice', {action});
  toast(d.message);
  if (d.ok && d.restarting) { await waitWebBack(); return; }
  refreshStatus();
}
// 重启后轮询等待服务恢复，一旦恢复自动刷新页面（免去手动强刷）
async function waitWebBack(){
  el('webSvcStatus').innerHTML = '<span class="warn">● 正在重启，等待服务恢复...</span>';
  for (let i = 0; i < 30; i++) {
    await sleep(1000);
    try {
      const r = await fetch('/api/status', {cache:'no-store'});
      if (r.ok) { location.reload(); return; }
    } catch(e) {}   // 重启期间连接被拒是正常的，继续等
  }
  el('webSvcStatus').innerHTML = '<span class="bad">● 30s 内未恢复</span>';
  toast('未在 30 秒内检测到服务恢复，请检查 gks-web.service');
}
async function power(action){
  const msg = action==='reboot' ? '确定重启树莓派？' : '确定关闭树莓派电源？';
  if (!confirm(msg)) return;
  const d = await postJSON('/api/power', {action});
  toast(d.message);
}
function copySsh(){
  const ip = state.ip[0] || 'unknown';
  const cmd = 'ssh xixi@' + ip;
  if (navigator.clipboard) navigator.clipboard.writeText(cmd).then(()=>toast('已复制: '+cmd)).catch(()=>toast(cmd));
  else toast(cmd);
}

// ---- 网络 / WiFi ----
// ⚠ SSID 是**攻击者可控的广播字符串**（一个叫 <img src=x onerror=...> 的热点就能打进来，
//   而本页有断电按钮）。所以: ① 任何拼进 innerHTML 的 SSID 一律 esc();
//   ② 绝不把 SSID 内联进 onclick/属性 —— esc() 不转义引号! 一律传**数组下标**。
let wifiNets = [], wifiProfs = [], wifiPollTimer = null, wifiFailCount = 0;
let lastWifiSsid = '', wifiHintUrl = 'http://xixi.local:9000/';

async function loadWifi(){
  try {
    const r = await fetch('/api/wifi/status');
    const s = await r.json();
    renderWifi(s);
  } catch(e){ toast('读取网络状态失败: ' + e.message); }
}

function renderWifi(s){
  const box = el('wifiNow');
  if (!box) return;
  if (s.available === false) {
    box.innerHTML = '<span class="bad">● 不可用</span> ' + esc(s.error || '');
    return;
  }
  lastWifiSsid = s.ssid || '';
  wifiHintUrl = s.hint_url || wifiHintUrl;
  const others = (s.ips || []).filter(x => x !== s.ip);
  box.innerHTML =
    '<div>当前网络: ' + (s.ssid ? '<b>' + esc(s.ssid) + '</b>' : '<span class="bad">未连接 WiFi</span>') +
      (s.active_name ? '　<span class="ok">● 已连接</span>' : '') + '</div>' +
    '<div>wlan0 IP: <b>' + esc(s.ip || '(无)') + '</b>' + (s.gateway ? '　网关: <b>' + esc(s.gateway) + '</b>' : '') + '</div>' +
    (others.length ? '<div>其它 IP: <b>' + esc(others.join(', ')) + '</b></div>' : '') +
    '<div>主机名: <b>' + esc(s.hostname) + '</b></div>' +
    '<div class="kv warn">换网后本页会断开 —— 用 <b>' + esc(s.hint_url) + '</b> 打开（mDNS 名字，换了 IP 也不变）</div>';
  if (el('netWifi')) el('netWifi').innerHTML = 'WiFi: <b>' + (s.ssid ? esc(s.ssid) : '未连接') + '</b>';
  wifiProfs = s.profiles || [];
  renderWifiProfs();
  renderWifiSwitch(s.switch || {});
}

function renderWifiSwitch(st){
  const box = el('wifiSwitchState');
  if (!box) return;
  const ph = st.phase || 'idle';
  if (ph === 'idle') { box.innerHTML = ''; return; }
  const tgt = esc(st.target || '');
  const prev = esc(st.prev || '');
  if (ph === 'activating') {
    box.innerHTML = '<span class="warn">● 正在切换到「' + tgt + '」…</span> 本页随时可能断开'
      + (prev ? '；失败会在约 1~1.5 分钟内自动退回「' + prev + '」' : '') + '。';
  } else if (ph === 'ok') {
    box.innerHTML = '<span class="ok">● 已切换到「' + tgt + '」</span>'
      + (st.ip ? '　新地址: <b>' + esc(st.ip) + '</b>' : '') + '（本页能刷新说明网络已通）';
  } else if (ph === 'reverting' || ph === 'reverted') {
    box.innerHTML = '<span class="warn">● 切换没成功，已自动切回「' + prev + '」</span>'
      + (st.reason ? '　原因: ' + esc(st.reason) : '');
  } else if (ph === 'failed') {
    box.innerHTML = '<span class="bad">● 切换失败</span>'
      + (st.reason ? '　' + esc(st.reason) : '') + '　—— 需要物理接触这台设备重新配网。';
  }
}

function wifiBars(sig){
  const n = sig >= 80 ? 4 : sig >= 60 ? 3 : sig >= 40 ? 2 : 1;
  const cls = sig >= 70 ? 'ok' : sig >= 45 ? 'warn' : 'bad';
  return '<span class="' + cls + '">' + '▮'.repeat(n) + '</span>'
       + '<span style="opacity:.25">' + '▮'.repeat(4 - n) + '</span>';
}

async function wifiScan(){
  const btn = el('wifiScanBtn');
  btn.disabled = true;
  el('wifiScanInfo').textContent = '扫描中…（约 2 秒）';
  try {
    const r = await fetch('/api/wifi/scan');
    const d = await r.json();
    if (!d.ok) { el('wifiScanInfo').textContent = '扫描失败'; toast(d.message || '扫描失败'); return; }
    wifiNets = d.networks || [];
    el('wifiScanInfo').innerHTML = '找到 <b>' + wifiNets.length + '</b> 个网络'
      + (d.rescan_limited ? '（刚扫过，本次用缓存）' : '');
    renderWifiNets();
  } catch(e){
    el('wifiScanInfo').textContent = '扫描失败';
    toast('扫描失败: ' + e.message);
  } finally { btn.disabled = false; }
}

function renderWifiNets(){
  const box = el('wifiList');
  if (!wifiNets.length) { box.innerHTML = '<div class="ph">点「扫描」列出附近网络</div>'; return; }
  let h = '';
  wifiNets.forEach(function(n, i){
    h += '<div class="row" style="margin-top:6px">'
       +   '<span class="kv" style="flex:0 0 60px">' + wifiBars(n.signal) + '</span>'
       +   '<span class="kv" style="flex:0 0 250px">' + esc(n.ssid) + '</span>'
       +   '<span class="kv" style="flex:0 0 150px">'
       +     (n.auth === 'open' ? '<span class="warn">开放(无密码)</span>' : '🔒 ' + esc(n.security || ''))
       +     (n.in_use ? ' <span class="ok">● 已连接</span>' : '')
       +   '</span>'
       +   '<button class="g" onclick="wifiPick(' + i + ')">连接</button>'
       + '</div>'
       + '<div class="row" id="wpw-' + i + '" style="display:none">'
       +   '<input type="password" id="wpwi-' + i + '" placeholder="WPA 密码" style="flex:1">'
       +   '<button onclick="togglePw(' + i + ')">显示</button>'
       +   '<button class="p" onclick="wifiJoin(' + i + ')">确定切换</button>'
       + '</div>';
  });
  box.innerHTML = h;
}

function wifiPick(i){
  const n = wifiNets[i];
  if (!n) return;
  if (n.auth === 'open') {
    if (confirm('「' + n.ssid + '」是开放网络（没有密码），确定连接？')) wifiJoin(i);
    return;
  }
  if (n.auth === 'enterprise' || n.auth === 'wep') {
    toast('暂不支持企业级认证 / WEP 网络');
    return;
  }
  const row = el('wpw-' + i);
  if (row) { row.style.display = 'flex'; const inp = el('wpwi-' + i); if (inp) inp.focus(); }
}

function togglePw(i){
  const inp = el('wpwi-' + i);
  if (!inp) return;
  inp.type = (inp.type === 'password') ? 'text' : 'password';
  toast(inp.type === 'password' ? '密码已隐藏' : '密码已显示');
}

async function wifiJoin(i){
  const n = wifiNets[i];
  if (!n) return;
  const inp = el('wpwi-' + i);
  const pw = inp ? inp.value : '';
  if (n.secure && !pw) { toast('这个网络需要密码'); if (inp) inp.focus(); return; }
  if (!confirm('切换到「' + n.ssid + '」？\n\n当前连接是「' + (lastWifiSsid || '未知') + '」。\n'
             + '切换后本页会断开，请用 ' + wifiHintUrl + ' 重新打开本页。\n'
             + '失败会在约 1~1.5 分钟内自动退回原来的网络。')) return;
  await wifiConnect({ ssid: n.ssid, secure: n.secure, password: pw });
}

async function wifiConnect(body){
  try {
    const d = await postJSON('/api/wifi/connect', body);
    toast(d.message || (d.ok ? '已开始切换' : '切换失败'));
    if (d.ok) enterSwitching(d); else loadWifi();
  } catch(e){ toast('切换请求失败: ' + e.message); }
}

function enterSwitching(d){
  wifiFailCount = 0;
  if (d.hint_url) wifiHintUrl = d.hint_url;
  const box = el('wifiSwitchState');
  box.innerHTML = '<span class="warn">● 正在切换…</span> 本页随时可能断开。失败会在 30 秒内自动退回。'
    + '请记住重新打开的地址: <b>' + esc(wifiHintUrl) + '</b>';
  document.querySelectorAll('#panel-wifi button').forEach(b => { b.disabled = true; });
  if (wifiPollTimer) clearInterval(wifiPollTimer);
  wifiPollTimer = setInterval(pollWifiOnce, 2000);
}

// ⚠ 旧 IP 失效后 fetch 会**一直挂着不 reject** —— 必须自己加超时，否则页面永远卡在"切换中"
async function pollWifiOnce(){
  const ctl = new AbortController();
  const to = setTimeout(() => ctl.abort(), 2500);
  try {
    const r = await fetch('/api/wifi/status', { signal: ctl.signal });
    const s = await r.json();
    clearTimeout(to);
    wifiFailCount = 0;
    renderWifi(s);
    const ph = (s.switch || {}).phase;
    if (ph === 'ok' || ph === 'reverted' || ph === 'failed') stopWifiSwitching();
  } catch(e){
    clearTimeout(to);
    wifiFailCount++;
    if (wifiFailCount >= 2) showWifiLostHint();
  }
}

function showWifiLostHint(){
  stopWifiSwitching();
  const box = el('wifiSwitchState');
  box.innerHTML = '<span class="warn">● 本页与树莓派的连接已断开</span>（换网时的正常现象）。'
    + '<br>请连上新网络后，用 <b>' + esc(wifiHintUrl) + '</b> 重新打开本页。'
    + '<br><span class="kv">若新网络也上不去，树莓派会在约 1~1.5 分钟内自动退回原来的网络（45s 激活 + 20s 验证 + 30s 回切）—— 那时用旧地址再打开即可。</span>';
}

function stopWifiSwitching(){
  if (wifiPollTimer) { clearInterval(wifiPollTimer); wifiPollTimer = null; }
  document.querySelectorAll('#panel-wifi button').forEach(b => { b.disabled = false; });
}

function renderWifiProfs(){
  const box = el('wifiProfiles');
  if (!box) return;
  if (!wifiProfs.length) { box.innerHTML = '<div class="ph">没有已保存的 WiFi</div>'; return; }
  let h = '';
  wifiProfs.forEach(function(p, i){
    h += '<div class="row" style="margin-top:6px">'
       +   '<span class="kv" style="flex:0 0 250px">' + esc(p.ssid)
       +     (p.active ? ' <span class="ok">● 当前</span>' : '') + '</span>'
       +   '<span class="kv" style="flex:0 0 90px">优先级 <b>' + p.priority + '</b></span>'
       +   '<input type="text" id="prio-' + i + '" value="' + p.priority + '" style="flex:0 0 60px">'
       +   '<button onclick="wifiSetPrio(' + i + ')">保存优先级</button>'
       +   '<button class="p" onclick="wifiPrefer(' + i + ')">设为开机首选</button>'
       +   (p.active ? '<button disabled title="正在使用中，不能删">删除</button>'
                     : '<button class="r" onclick="wifiDel(' + i + ')">删除</button>')
       +   (p.has_secret ? '' : ' <span class="warn">⚠ 未保存密码</span>')
       +   (p.persistent ? '' : ' <span class="warn">⚠ 重启后消失</span>')
       + '</div>';
  });
  box.innerHTML = h;
}

async function wifiProfileAction(body){
  try {
    const d = await postJSON('/api/wifi/profile', body);
    toast(d.message || '完成');
    loadWifi();
  } catch(e){ toast('操作失败: ' + e.message); }
}

function wifiSetPrio(i){
  const p = wifiProfs[i];
  if (!p) return;
  wifiProfileAction({ action: 'priority', name: p.name, priority: el('prio-' + i).value });
}

function wifiPrefer(i){
  const p = wifiProfs[i];
  if (!p) return;
  if (confirm('把「' + p.ssid + '」设为开机首选？\n\n其余 WiFi 的优先级会归零；开机时优先连它，'
            + '它不在范围内才退到下一个。')) {
    wifiProfileAction({ action: 'prefer', name: p.name });
  }
}

function wifiDel(i){
  const p = wifiProfs[i];
  if (!p) return;
  if (confirm('删除「' + p.ssid + '」？\n\n密码和配置一起丢掉，以后要用得重新输密码。')) {
    wifiProfileAction({ action: 'delete', name: p.name });
  }
}

function wifiRevert(){
  if (confirm('切回上一个网络？')) wifiProfileAction({ action: 'revert' });
}

function copyWifiUrl(){
  const u = wifiHintUrl;
  if (navigator.clipboard) navigator.clipboard.writeText(u).then(() => toast('已复制: ' + u)).catch(() => toast(u));
  else toast(u);
}

// ---- 实时日志（SSE，终端同步）----
// 清屏要「真清」：服务端每次重连都会回显历史（journalctl -n 200 / 日志文件末 200 行 /
// 模拟器全量缓冲），只清 DOM 的话切换日志类型或重连又会被刷回来。故记下「已清屏」的
// 单位，重连时带 tail=0，让服务端不回显历史、只推新行。
function attachLogStream(pre, src){
  src.onmessage = function(ev){
    pre.textContent += ev.data + '\n';
    pre.scrollTop = pre.scrollHeight;
    const lines = pre.textContent.split('\n');
    if (lines.length > 3000) pre.textContent = lines.slice(-3000).join('\n');
  };
}
function logStreamUrl(unit){
  return '/api/log/stream?unit=' + unit + '&tail=' + (clearedUnits[unit] ? 0 : 200);
}
function startLogStream(){
  if (logSource) logSource.close();
  const unit = el('logUnit').value;
  const pre = el('logArea');
  pre.textContent = '';
  logSource = new EventSource(logStreamUrl(unit));
  logSource.onopen = function(){
    // 重连（含服务端重启）会重新开始推流：已清屏的单位只收新行，未清屏的回显历史
    if (!clearedUnits[unit]) pre.textContent = '';
  };
  attachLogStream(pre, logSource);
}
function stopLogStream(){ if (logSource){ logSource.close(); logSource = null; } }
function clearLogArea(){
  el('logArea').textContent = '';
  clearedUnits[el('logUnit').value] = true;   // 记住已清屏，切换类型/重连都不回显
  serverClearLog(el('logUnit').value);        // 真清：文件/缓冲一并清（2026-10）
}
// 服务端真清屏：mainpy=让活动 main.py 自己截断 log.txt；sim=清模拟器缓冲。
// journalctl 类不支持 → 只维持上面的本地不回显（页面提示见日志页说明文字）。
async function serverClearLog(unit){
  if (unit !== 'mainpy' && unit !== 'sim') return;
  try {
    const d = await postJSON('/api/log/clear', {unit});
    if (!d.ok) toast('清屏: ' + d.message);
  } catch(e) { /* 清屏失败不拦显示 */ }
}

// ---- 日志悬浮窗口 ----
let floatSource = null;
function floatLog(){
  stopLogStream();                       // 关闭 tab 内流，由悬浮窗口接管
  el('floatUnit').value = el('logUnit').value;   // 悬浮窗默认与 tab 当前类型一致
  el('floatLog').style.display = 'block';
  startFloatStream(el('floatUnit').value);
}
// 悬浮窗切换日志类型：同步回 tab 下拉，关闭悬浮后 tab 接着看同一类型
function floatUnitChange(v){ el('logUnit').value = v; startFloatStream(v); }
function startFloatStream(unit){
  if (floatSource) floatSource.close();
  const pre = el('floatLogArea');
  pre.textContent = '';
  floatSource = new EventSource(logStreamUrl(unit));
  floatSource.onopen = function(){
    if (!clearedUnits[unit]) pre.textContent = '';
  };
  attachLogStream(pre, floatSource);
}
function clearFloatArea(){
  el('floatLogArea').textContent = '';
  const u = el('floatUnit').value;
  clearedUnits[u] = true;
  serverClearLog(u);                          // 悬浮窗的清屏同样走服务端真清
}
function closeFloat(){
  if (floatSource){ floatSource.close(); floatSource = null; }
  el('floatLog').style.display = 'none';
  startLogStream();                      // 悬浮关闭，tab 内流恢复（同一类型）
}
function setOpacity(v){ el('floatLog').style.opacity = v / 100; }
function initFloatDrag(){
  const f = el('floatLog');
  const head = f.querySelector('.float-head');
  let drag = false, ox = 0, oy = 0;
  head.addEventListener('mousedown', function(e){
    drag = true; ox = e.clientX - f.offsetLeft; oy = e.clientY - f.offsetTop; e.preventDefault();
  });
  document.addEventListener('mousemove', function(e){
    if (!drag) return;
    f.style.left = (e.clientX - ox) + 'px';
    f.style.top = (e.clientY - oy) + 'px';
    f.style.right = 'auto';
  });
  document.addEventListener('mouseup', function(){ drag = false; });
}

// ---- 摄像头测试（连接检查 + 预览；二维码识别走模拟器「1」命令）----
// 三路**各自单独测**：单测只让出/重开自己那一路的推流、**不动别的格**（一起停会把别人
// 冻住）。结果记进 camState，状态行按路渲染，互不覆盖 —— 单测主摄不会把扫码那格的结果清掉。
let camState = {};   // 'main'|'qr'|'pi' -> {ok, resolution|reason}
const CAM_NAME = { main: '主摄像头', qr: '扫码摄像头', pi: 'pi摄像头' };
function camLine(cam){
  const s = camState[cam];
  if (!s) return CAM_NAME[cam] + ': <span class="kv">未测试</span>';
  return CAM_NAME[cam] + ': ' + (s.ok
    ? '<span class="ok">正常 ' + esc(s.resolution) + '</span>'
    : '<span class="bad">失败（' + esc(s.reason) + '）</span>');
}
function renderCamStatus(){
  el('camStatus').innerHTML = camLine('main') + '　' + camLine('qr') + '　' + camLine('pi');
}
// cam = 'main' | 'qr' | 'pi' | 'all'（省略 = all，保持老「测试摄像头」的语义）
async function checkCamera(cam){
  cam = cam || 'all';
  const targets = (cam === 'all') ? ['main', 'qr', 'pi'] : [cam];
  el('camStatus').textContent = '正在检测 ' + targets.map(t => CAM_NAME[t]).join(' / ') + ' ...';
  // ⚠ 这里**必须 await**。stopScan() 里那发 /abort 会停推流，而它是异步发出的 ——
  //   不等它落地的话，它可能晚于下面新开的预览流到达、把**刚起来的**流掐掉：表现正是
  //   「检测显示正常、画面却冻在最后一帧」，而直接访问 /stream 又是实时的（那条路不
  //   经过 /abort）。2026-10 现场症状，重复点「测试摄像头」必现。
  await stopScan(cam);
  // 主摄要重新检测/开 Tab 内预览时，先收掉参数悬浮窗（同一台主摄，别两路抢）
  if (cam === 'all' || cam === 'main') camTuneForceClose();
  // 先检测、后开预览：检测要独占设备，若先开推流会在同进程内与自己抢摄像头（EBUSY）
  targets.forEach(t => { el('box-' + t).innerHTML = ph('检测中...'); camState[t] = null; });
  try {
    const r = await fetch('/api/camera/check?cam=' + cam);
    const d = await r.json();
    // 检测完设备已释放，此时再开预览（预览窗口在按钮下方）
    // ⚠ 带时间戳：URL 不变时浏览器可能复用一个「还挂着」的上一次 MJPEG 连接，
    //   新 <img> 就永远等不到自己的流（同样是「画面冻住」）。
    const bt = Date.now();
    targets.forEach(function(t){
      const one = d[t];
      if (!one) { return; }
      camState[t] = one;
      el('box-' + t).innerHTML = one.ok
        ? '<img src="/stream?cam=' + t + '&t=' + bt + '">'
        : ph('未通过检测');
    });
    renderCamStatus();
  } catch(e){ el('camStatus').textContent = '检测失败: ' + e; }
}
function stopCam(){
  stopScan();
  camTuneForceClose();
  ['main', 'qr', 'pi'].forEach(t => { el('box-' + t).innerHTML = ph(); });
  camState = {};
  el('camStatus').textContent = '已停止预览';
}

// ---- 主摄参数悬浮窗（2026-10-30 加）----
// 设计: 悬浮窗自带一路 /stream?cam=main 预览，控件直接 cap.set 到那一路的
// VideoCapture 上（服务端 /api/camctl），下一帧就能看到效果。
// 只影响这路预览流，不写 config —— 关预览/重开相机会按档位重新设满，不会漏进任务。
// ⚠️ 主摄是独占设备: 悬浮窗开预览会顶掉 Tab 里那路主摄预览（服务端 _send_stream 先
//    _stop_stream("main")）；关悬浮窗时如果原来 Tab 里有预览，自动重开。
let camTunePrevMain = false;
let camTuneTimer = null;
let camTuneStreamLive = false;   // 悬浮窗这路 /stream 是否真的起来了（决定关闭时要不要 /abort）
function camTuneImg(){
  return '<img src="/stream?cam=main&t=' + Date.now() + '" ' +
         'onerror="el(\'camTuneState\').textContent=\'预览打不开（主摄被占用？先停任务/退出）\'">';
}
function openCamTune(){
  const shown = el('camTune').style.display === 'block';
  el('camTune').style.display = 'block';
  if (shown) camTuneRead(); else camTuneOpenStream();
}
function camTuneOpenStream(){
  // 只记「Tab 里原本有主摄 MJPEG」，扫描结果图（data:）不算
  camTunePrevMain = !!(el('box-main') &&
                       el('box-main').querySelector('img[src^="/stream"]'));
  if (camTunePrevMain) el('box-main').innerHTML = ph('主摄预览在参数悬浮窗');
  camTuneStreamLive = false;
  el('camTuneBox').innerHTML = camTuneImg();
  el('camTuneState').textContent = '正在打开预览...';
  setTimeout(camTuneRead, 700);        // 等 /stream 把 _streams["main"] 注册上
}
async function closeCamTune(){
  clearTimeout(camTuneTimer);
  const reopen = camTunePrevMain;
  const live = camTuneStreamLive;
  camTunePrevMain = false;
  camTuneStreamLive = false;
  el('camTune').style.display = 'none';
  el('camTuneBox').innerHTML = ph('点「打开预览」...');
  el('camTuneState').textContent = '已关闭';
  // 只有悬浮窗这路流真的起来过才让它让出主摄 —— 否则可能误伤正在跑的主摄识别任务。
  // ⚠️ 必须等 /abort 落地再重开 Tab 预览（晚到的 abort 会掐掉刚起来的流，见 stopScan 注释）
  if (live) { try { await fetch('/abort?cam=main'); } catch(e) {} }
  if (reopen) {
    el('box-main').innerHTML = '<img src="/stream?cam=main&t=' + Date.now() + '">';
  }
}
function camTuneForceClose(){
  clearTimeout(camTuneTimer);
  camTunePrevMain = false;
  camTuneStreamLive = false;
  el('camTune').style.display = 'none';
  el('camTuneBox').innerHTML = ph('点「打开预览」...');
  el('camTuneState').textContent = '已关闭';
}
function camTuneSyncEnabled(){
  const wbAuto = el('camTuneWbMode').value === '1';
  el('camTuneWbRange').disabled = wbAuto;
  el('camTuneWbVal').disabled = wbAuto;
  const expAuto = el('camTuneExpMode').value === '3';
  el('camTuneExpRange').disabled = expAuto;
  el('camTuneExpVal').disabled = expAuto;
}
function camTuneWbModeChange(){ camTuneSyncEnabled(); camTuneQueue(); }
function camTuneExpModeChange(){ camTuneSyncEnabled(); camTuneQueue(); }
function camTuneWbSlider(){
  el('camTuneWbVal').value = el('camTuneWbRange').value; camTuneQueue();
}
function camTuneWbInput(){
  el('camTuneWbRange').value = el('camTuneWbVal').value; camTuneQueue();
}
function camTuneExpSlider(){
  el('camTuneExpVal').value = el('camTuneExpRange').value; camTuneQueue();
}
function camTuneExpInput(){
  el('camTuneExpRange').value = el('camTuneExpVal').value; camTuneQueue();
}
function camTuneQueue(){
  clearTimeout(camTuneTimer); camTuneTimer = setTimeout(camTuneApplyNow, 300);
}
function camTuneApplyReadback(d){
  if (d.auto_wb !== undefined && d.auto_wb !== null)
    el('camTuneWbMode').value = String(d.auto_wb ? 1 : 0);
  if (d.wb_temp) {
    el('camTuneWbRange').value = d.wb_temp; el('camTuneWbVal').value = d.wb_temp;
  }
  if (d.auto_exposure !== undefined && d.auto_exposure !== null)
    el('camTuneExpMode').value = String(d.auto_exposure);
  if (d.exposure) {
    el('camTuneExpRange').value = d.exposure; el('camTuneExpVal').value = d.exposure;
  }
  camTuneSyncEnabled();
  el('camTuneWbNow').textContent = d.auto_wb ? '自动' : (d.wb_temp + 'K');
  el('camTuneExpNow').textContent = (d.auto_exposure === 3) ? '自动' : ('手动 ' + d.exposure);
}
async function camTuneRead(){
  try {
    const r = await fetch('/api/camctl?cam=main');
    const d = await r.json();
    if (!d.ok) {
      camTuneStreamLive = false;
      el('camTuneState').textContent = d.message || '未连接';
      return;
    }
    camTuneStreamLive = true;
    camTuneApplyReadback(d);
    el('camTuneState').textContent = '已连接';
  } catch(e){ el('camTuneState').textContent = '读取失败: ' + e; }
}
async function camTuneApplyNow(){
  clearTimeout(camTuneTimer);
  const body = {cam:'main',
                auto_wb: +el('camTuneWbMode').value,
                wb_temp: +el('camTuneWbVal').value,
                auto_exposure: +el('camTuneExpMode').value,
                exposure: +el('camTuneExpVal').value};
  try {
    const d = await postJSON('/api/camctl', body);
    if (!d.ok) {
      if (d.message && d.message.indexOf('没有在运行') >= 0) camTuneStreamLive = false;
      el('camTuneState').textContent = d.message || '应用失败';
      return;
    }
    camTuneStreamLive = true;
    camTuneApplyReadback(d);
    el('camTuneState').textContent = '已应用';
  } catch(e){ el('camTuneState').textContent = '应用失败: ' + e; }
}
async function camTuneLoadProfile(){
  const p = el('camTuneProfile').value;
  try {
    const d = await postJSON('/api/camctl', {cam:'main', profile:p});
    if (!d.ok) {
      if (d.message && d.message.indexOf('没有在运行') >= 0) camTuneStreamLive = false;
      el('camTuneState').textContent = d.message || '载入失败';
      return;
    }
    camTuneStreamLive = true;
    camTuneApplyReadback(d);
    el('camTuneState').textContent = '已载入 ' + p + ' 档';
  } catch(e){ el('camTuneState').textContent = '载入失败: ' + e; }
}
function initCamTuneDrag(){
  const f = el('camTune'); if (!f) return;
  const head = f.querySelector('.float-head');
  let drag = false, ox = 0, oy = 0;
  head.addEventListener('mousedown', function(e){
    drag = true; ox = e.clientX - f.offsetLeft; oy = e.clientY - f.offsetTop; e.preventDefault();
  });
  document.addEventListener('mousemove', function(e){
    if (!drag) return;
    f.style.left = (e.clientX - ox) + 'px';
    f.style.top = (e.clientY - oy) + 'px';
    f.style.right = 'auto';
  });
  document.addEventListener('mouseup', function(){ drag = false; });
}

// ---- 摄像头识别任务（就地检测：在 GKS_Web 进程内直接跑，不经单片机）----
// 与 Pi 端 camera_web.py 同构。⚠ 摄像头是**跨进程独占**设备：本页的推流与 main.py 侧
// 的识别任务（人质/色球）会互相抢占，而 main.py 的 _stop_main_stream() 只管得了它自己
// 进程内的推流、管不到这里 —— 台架上要「先停预览（退出）再让单片机跑识别步骤」。
// （camera_web.py 的 :8000 页面已废弃，不再是争抢方。）
let scanCtrl = null;   // 当前识别请求的 AbortController，退出/重开时用于取消
// cam 省略 / 'all' = 让出全部（识别任务收尾用这个语义）；给了具体路 = **只让出那一路**。
// ⚠ 返回 promise：能等的调用方（checkCamera）**一定要等它落地** —— /abort 的语义是
//   「把这台相机的一切持有形式让出来」，它连**随后**新开的推流也会一起停（它是异步
//   发出的，晚到就掐掉刚起来的流）。不等也不会立刻出错，但会随机冻画面。
function stopScan(cam){
  if (scanCtrl) { scanCtrl.abort(); scanCtrl = null; }
  // pi 不走 /abort：它在服务端不在 _streams 表里（_stop_stream("pi") 是空操作），
  // 而 _handle_abort 会顺手 `release_lease_if()` 把**主摄租约**也让出去 —— 测 pi
  // 不该有这种副作用。预览在跑时自检直接判 OK（见服务端 _check_picam）。
  if (cam === 'pi') { return Promise.resolve(); }
  const q = (cam && cam !== 'all') ? ('?cam=' + cam) : '';
  return fetch('/abort' + q).catch(() => {});
}
async function scanQR(method){
  // 方法名以 `_main` 结尾 = 用**主摄**扫码（wechat_main="D" / diff_main="F"）：
  // 结果落在**主摄那一格**，服务端也会改停主摄推流（见 /scan 的 _handle_scan 与
  // 服务端 _SCAN_METHODS）。这里**按后缀判**而不是穷举方法名 —— 加新算法时
  // （比如再来一条 xxx_main）不用回来改这里，也就不会漏。
  const toMain = method.endsWith('_main');
  const box = toMain ? 'box-main' : 'box-qr';
  const label = toMain ? '主摄' : '扫码副摄';
  if (scanCtrl) scanCtrl.abort();
  const ctrl = new AbortController(); scanCtrl = ctrl;
  el(box).innerHTML = ph('扫码中...');
  el('camStatus').textContent = '正在扫码 (' + label + '/' + method + ')...';
  try {
    const resp = await fetch('/scan?method=' + method, {signal: ctrl.signal});
    const d = await resp.json();
    if (ctrl.signal.aborted) return;
    if (d.aborted) { el('camStatus').textContent = '已取消'; return; }
    el(box).innerHTML = d.image ? '<img src="data:image/jpeg;base64,' + d.image + '">' : ph('识别完成');
    el('camStatus').textContent = d.success ? ('识别成功: ' + d.code + ' (' + d.method + ', ' + label + ')')
                                            : ('识别失败: ' + d.reason);
  } catch(e){ if (!ctrl.signal.aborted) el('camStatus').textContent = '请求失败: ' + e; }
}
async function scanHostage(){
  if (scanCtrl) scanCtrl.abort();
  const ctrl = new AbortController(); scanCtrl = ctrl;
  el('box-main').innerHTML = ph('人质识别中...');
  el('camStatus').textContent = '人质识别中（凑满 3 个目标，限时 ~6s）...';
  try {
    const resp = await fetch('/hostage', {signal: ctrl.signal});
    const d = await resp.json();
    if (ctrl.signal.aborted) return;
    if (d.aborted) { el('camStatus').textContent = '已取消'; return; }
    el('box-main').innerHTML = d.image ? '<img src="data:image/jpeg;base64,' + d.image + '">' : ph('识别完成');
    if (d.success) {
      // 目标已按图像从左到右排序：1.圆锥(640,512) 2.圆柱(960,508) ...
      const order = d.targets.map((t, i) => (i + 1) + '.' + t.shape_name
                                             + '(' + t.cx + ',' + t.cy + ')').join(' ');
      el('camStatus').textContent = '识别成功（从左到右）: ' + order;
    } else {
      el('camStatus').textContent = '识别失败: ' + d.reason;
    }
  } catch(e){ if (!ctrl.signal.aborted) el('camStatus').textContent = '请求失败: ' + e; }
}

// ---- 色球识别（就地检测：在 GKS_Web 进程内跑，不经单片机）----
async function scanColor(){
  if (scanCtrl) scanCtrl.abort();
  const ctrl = new AbortController(); scanCtrl = ctrl;
  el('box-main').innerHTML = ph('色球识别中...');
  el('camStatus').textContent = '色球识别中（凑满红/绿/蓝三色，限时 ~5s）...';
  try {
    const resp = await fetch('/color', {signal: ctrl.signal});
    const d = await resp.json();
    if (ctrl.signal.aborted) return;
    if (d.aborted) { el('camStatus').textContent = '已取消'; return; }
    el('box-main').innerHTML = d.image ? '<img src="data:image/jpeg;base64,' + d.image + '">' : ph('识别完成');
    el('camStatus').textContent = d.success ? colorOrderText(d.targets, true)
                                            : ('识别失败: ' + d.reason);
  } catch(e){ if (!ctrl.signal.aborted) el('camStatus').textContent = '请求失败: ' + e; }
}

// ---- 色球区矩形地板（就地跑，不走串口）----
// 识别黑胶带框出的板子，给出**板心**在画面里的坐标（缺球时做横向基准用）。
// 四边齐全 = 精确（对角交点）；只有三边 = 按长宽比估（EST，页面标出来）。
async function scanRect(){
  if (scanCtrl) scanCtrl.abort();
  const ctrl = new AbortController(); scanCtrl = ctrl;
  el('box-main').innerHTML = ph('矩形地板识别中...');
  el('camStatus').textContent = '矩形地板识别中（黑胶带边框，最多 5 帧）...';
  try {
    const resp = await fetch('/rect', {signal: ctrl.signal});
    const d = await resp.json();
    if (ctrl.signal.aborted) return;
    if (d.aborted) { el('camStatus').textContent = '已取消'; return; }
    el('box-main').innerHTML = d.image ? '<img src="data:image/jpeg;base64,' + d.image + '">' : ph('识别完成');
    if (d.success) {
      const corners = d.corners.map((p, i) => 'P' + i + '(' + p[0] + ',' + p[1] + ')').join(' ');
      const raw = (d.frames || []).map(p => '(' + p[0] + ',' + p[1] + ')').join(' ');
      el('camStatus').innerHTML = '矩形中心 <b>(' + d.center[0] + ',' + d.center[1] + ')</b>'
        + ' · ' + (d.estimated ? '<span class="warn">三边估出</span>' : '<span class="ok">四边精确</span>')
        + ' · ' + d.votes + '帧 剔' + (d.dropped || 0) + ' 散布' + d.spread + 'px'
        + ' · ' + (d.path === 'enclosed' ? '量内沿' : '量中线') + ' · ' + corners
        + (raw ? '<br><span class="kv">各帧原始中心: ' + raw + '</span>' : '');
    } else {
      el('camStatus').textContent = '识别失败: ' + d.reason;
    }
  } catch(e){ if (!ctrl.signal.aborted) el('camStatus').textContent = '请求失败: ' + e; }
}

// 看路（就地跑，不走串口）。找地面上的黄色区 → 取它的**左边缘 x** 作 Δx。
// 单片机侧消费同一条链路（指令 "G" → 0x08 帧），这条按钮只是让台架上先看见测量值。
async function scanRoad(){
  if (scanCtrl) scanCtrl.abort();
  const ctrl = new AbortController(); scanCtrl = ctrl;
  el('box-main').innerHTML = ph('看路识别中...');
  el('camStatus').textContent = '看路识别中（最多 9 帧，需要小车静止）...';
  try {
    const resp = await fetch('/road', {signal: ctrl.signal});
    const d = await resp.json();
    if (ctrl.signal.aborted) return;
    if (d.aborted) { el('camStatus').textContent = '已取消'; return; }
    el('box-main').innerHTML = d.image ? '<img src="data:image/jpeg;base64,' + d.image + '">' : ph('识别完成');
    if (d.success) {
      const raw = (d.frames || []).map(v => v.toFixed(1)).join(' ');
      el('camStatus').innerHTML = '<b>Δx=' + d.dx + 'px</b>'
        + ' · 左边缘 <b>(' + d.edge[0] + ',' + d.edge[1] + ')</b>'
        + ' · 黄区面积 ' + Math.round(d.area) + 'px²'
        + ' · ' + d.votes + '帧 剔' + (d.dropped || 0) + ' 散布' + d.spread + 'px'
        + ((d.flags & 2) ? ' · <span class="warn">贴画面左边(饱和)</span>' : '')
        + ((d.flags & 8) ? ' · <span class="warn">面积勉强</span>' : '')
        + (raw ? '<br><span class="kv">各帧 Δx: ' + raw + '</span>' : '');
    } else {
      el('camStatus').textContent = '看路失败: ' + d.reason;
    }
  } catch(e){ if (!ctrl.signal.aborted) el('camStatus').textContent = '请求失败: ' + e; }
}

// 看路校准（就地跑，不走串口）。拟合**黄区直边的图像方向角** —— 车航向的视觉观测量
// （黄区固定在场上、相机固定在车上 ⇒ 方向只随车航向变），陀螺漂移的基准读数。
// 单片机侧消费同一条链路（指令 "H" → 0x09 帧），这条按钮只是让台架上先看见测量值。
// ⚠️ 只测不改：这条链只读数，不动车、不写航向。标定：车摆基准位记下 ang 填进
//    单片机 roadyaw_ideal，之后 err = (ang − ideal)×dir 就是「陀螺漂了多少」。
async function scanRoadYaw(){
  if (scanCtrl) scanCtrl.abort();
  const ctrl = new AbortController(); scanCtrl = ctrl;
  el('box-main').innerHTML = ph('看路校准中...');
  el('camStatus').textContent = '看路校准中（拟合黄区直边方向角，最多 9 帧，需要小车静止）...';
  try {
    const resp = await fetch('/road_yaw', {signal: ctrl.signal});
    const d = await resp.json();
    if (ctrl.signal.aborted) return;
    if (d.aborted) { el('camStatus').textContent = '已取消'; return; }
    el('box-main').innerHTML = d.image ? '<img src="data:image/jpeg;base64,' + d.image + '">' : ph('识别完成');
    if (d.success) {
      const raw = (d.frames || []).map(v => v.toFixed(2)).join(' ');
      el('camStatus').innerHTML = '<b>ang=' + d.ang + '°</b>'
        + ' · 直段 ' + d.span + 'px · RMS ' + d.rms + 'px · 内点 ' + d.cols
        + ' · ' + d.votes + '帧 剔' + (d.dropped || 0) + ' 散布' + d.spread + '°'
        + ((d.flags & 2) ? ' · <span class="warn">直段偏短(勉强)</span>' : '')
        + (raw ? '<br><span class="kv">各帧 ang: ' + raw + '</span>' : '');
    } else {
      el('camStatus').textContent = '看路校准失败: ' + d.reason;
    }
  } catch(e){ if (!ctrl.signal.aborted) el('camStatus').textContent = '请求失败: ' + e; }
}

// 靶标识别 + 靶心定位（就地跑，不走串口）。与 scanColor 同构。
async function scanTarget(pi){
  /* pi=1 → /target?pi=1 = 优先用树莓派 CSI 那颗识别（失败自动退回主摄）。
     台架上就是靠它读 CSI 的靶 cx 去填单片机的 tc_cx_aim_pi（瞄准列）。 */
  if (scanCtrl) scanCtrl.abort();
  const ctrl = new AbortController(); scanCtrl = ctrl;
  el('box-main').innerHTML = ph('靶标识别中...');
  el('camStatus').textContent = '靶标识别中（' + (pi ? '优先pi/CSI（首次要等它起流 ~3s），' : '')
    + '跨色尺寸一致性定靶心，限时 ~' + (pi ? '14' : '6') + 's）...';
  try {
    const resp = await fetch('/target' + (pi ? '?pi=1' : ''), {signal: ctrl.signal});
    const d = await resp.json();
    if (ctrl.signal.aborted) return;
    if (d.aborted) { el('camStatus').textContent = '已取消'; return; }
    el('box-main').innerHTML = d.image ? '<img src="data:image/jpeg;base64,' + d.image + '">' : ph('识别完成');
    const cam = pi ? ('[实际: ' + (d.cam_is_pi ? 'pi/CSI' : '主摄') + '] ') : '';
    el('camStatus').textContent = cam + (d.success ? targetOrderText(d.targets, true)
                                                   : ('识别失败: ' + d.reason));
  } catch(e){ if (!ctrl.signal.aborted) el('camStatus').textContent = '请求失败: ' + e; }
}

// 目标已按图像从左到右排序：1.红(400,500)r57 2.绿(800,505)r58 3.蓝(1200,510)r57
// r = 靶盘外圆等效半径[px]，可拿去粗估距离（同时是固件视差补偿的输入）
function targetOrderText(targets, withOk){
  const order = targets.map((t, i) => (i + 1) + '.' + t.color_name
                                       + '(' + t.cx + ',' + t.cy + ')r' + t.r).join(' ');
  return (withOk ? '识别成功（从左到右）: ' : '（从左到右）: ') + order;
}

// 目标已按图像从左到右排序：1.红(400,500) 2.绿(800,505) 3.蓝(1200,510)
function colorOrderText(targets, withOk){
  const order = targets.map((t, i) => (i + 1) + '.' + t.color_name
                                       + '(' + t.cx + ',' + t.cy + ')').join(' ');
  return (withOk ? '识别成功（从左到右）: ' : '（从左到右）: ') + order;
}

// ---- 历史日志侧边栏 / 帧查看 / 日志清理弹窗（2026-10）----
let viewingRun = null;   // 非 null = 正在回看某次运行（实时流暂停, 见 viewRun/backToLive）
let runCache = null;     // loadRuns 拉到的列表（侧边栏 onclick 用下标引用, 见 esc 纪律）
async function loadRuns(){
  try {
    const r = await fetch('/api/logs/list');
    const d = await r.json();
    runCache = d.runs;                 // 下标引用: 注入面在本端收口（与 SSID 同纪律）
    let html = '';
    for (let i = 0; i < d.runs.length; i++) {
      const run = d.runs[i];
      const act = run.active ? ' <span class="ok">● 本次</span>' : '';
      const cur = (viewingRun === run.name) ? ' active' : '';
      html += '<div class="logside-item' + cur + '" onclick="viewRunIdx(' + i + ')"'
        + ' oncontextmenu="return runMenu(event,' + i + ')">'
        + '<div><b>' + esc(run.time) + '</b>' + act + '</div>'
        + '<div class="kv">' + run.size_mb + ' MB' + (run.frames ? ' · 帧 ' + run.frames : '')
        + (run.is_run_dir ? '' : ' · 旧式') + '</div></div>';
    }
    el('runsList').innerHTML = html || '<span class="kv">还没有任何运行的日志</span>';
    el('runsInfo').textContent = d.runs.length + ' 次 · ' + d.total_mb + ' MB';
  } catch(e) { el('runsInfo').textContent = '加载失败: ' + e; }
}
function viewRunIdx(i){ if (runCache && runCache[i]) viewRun(runCache[i].name); }

// ---- 历史运行右键菜单: 删除这一次（2026-10）----
// 菜单 DOM 现搭（#runCtx 常驻但空着）; 下标引用 runCache, 与 onclick 同一套纪律。
let ctxRunIdx = null;
function runMenu(ev, i){
  ev.preventDefault();
  const run = runCache && runCache[i];
  if (!run) return false;
  ctxRunIdx = i;
  const m = el('runCtx');
  m.innerHTML = run.active
    ? '<span class="kv">本次运行中, 不能删</span>'
    : '<a onclick="deleteRun()">🗑 删除这次运行</a>';
  m.style.display = 'block';
  // 贴光标, 但不越出视口右下
  const w = m.offsetWidth, h = m.offsetHeight;
  m.style.left = Math.max(4, Math.min(ev.clientX, window.innerWidth - w - 8)) + 'px';
  m.style.top  = Math.max(4, Math.min(ev.clientY, window.innerHeight - h - 8)) + 'px';
  return false;                    // 挡掉浏览器自带的右键菜单
}
function hideRunCtx(){ const m = el('runCtx'); if (m) m.style.display = 'none'; }
async function deleteRun(){
  const run = runCache && runCache[ctxRunIdx];
  hideRunCtx();
  if (!run) return;
  if (!confirm('删除这次运行的日志和归档帧?\n\n' + run.time + '（' + run.size_mb + ' MB）\n\n'
               + '删了就找不回来了。')) return;
  try {
    const d = await postJSON('/api/logs/delete', {run: run.name});
    toast(d.message);
    if (d.ok) {
      if (viewingRun === run.name) backToLive();   // 正在回看这一条 → 先回实时流
      loadRuns();
    }
  } catch(e) { toast('删除失败: ' + e); }
}
// ---- 帧右键「验证」: 离线复跑当前算法（2026-11-06 加）----
// 与 runMenu 同一套纪律: 菜单 DOM 常驻空壳（#frameCtx）, 下标引用 frameMenuCache。
// 结果弹 #validateModal: 左 = 喂给算法的输入帧, 右 = 当前代码标注的结果, 摘要在下。
let ctxFrameIdx = null;
function frameMenu(ev, i){
  ev.preventDefault(); ev.stopPropagation();
  const f = frameMenuCache && frameMenuCache[i];
  if (!f) return false;
  ctxFrameIdx = i;
  const m = el('frameCtx');
  m.innerHTML = '<span class="kv">' + esc(f.file) + '</span>'
    + '<a onclick="runFrameValidate()">🔍 验证（离线复跑当前算法）</a>';
  m.style.display = 'block';
  const w = m.offsetWidth, h = m.offsetHeight;
  m.style.left = Math.max(4, Math.min(ev.clientX, window.innerWidth - w - 8)) + 'px';
  m.style.top  = Math.max(4, Math.min(ev.clientY, window.innerHeight - h - 8)) + 'px';
  return false;                    // 挡掉浏览器自带的右键菜单
}
function hideFrameCtx(){ const m = el('frameCtx'); if (m) m.style.display = 'none'; }
function closeValidate(){ el('validateModal').style.display = 'none'; }
async function runFrameValidate(){
  hideFrameCtx();
  const f = frameMenuCache && frameMenuCache[ctxFrameIdx];
  if (!f || !frameView.run) return;
  el('vldTitle').textContent = '离线验证 · ' + f.file;
  el('vldMeta').textContent = '正在离线复跑当前算法（纯算帧, 不碰相机/串口）...';
  el('vldInfo').textContent = '';
  el('vldState').textContent = '计算中...';
  el('vldOpenBtn').style.display = 'none';
  el('vldIn').removeAttribute('src');
  el('vldOut').removeAttribute('src');
  el('validateModal').style.display = 'flex';
  try {
    const d = await postJSON('/api/logs/validate', {run: frameView.run, file: f.file});
    if (!d.ok) { el('vldState').textContent = ''; el('vldMeta').textContent = d.message || '验证失败'; return; }
    el('vldMeta').textContent = '算法: ' + d.algo + ' · 输入: ' + d.input_file
      + (d.note || '') + ' · 耗时 ' + d.elapsed_ms + 'ms';
    el('vldIn').src = '/api/logs/frame?run=' + encodeURIComponent(frameView.run)
                    + '&file=' + encodeURIComponent(d.input_file);
    if (d.image) el('vldOut').src = d.image;
    el('vldInfo').textContent = d.info_text || '';
    el('vldState').textContent = d.info_ok === false ? '⚠ 检测未成功（摘要见下）' : '✓ 完成';
    if (d.image) el('vldOpenBtn').style.display = '';
  } catch(e) {
    el('vldState').textContent = '';
    el('vldMeta').textContent = '验证失败: ' + e;
  }
}
async function viewRun(name){
  try {
    const r = await fetch('/api/logs/read?run=' + encodeURIComponent(name));
    const d = await r.json();
    if (!d.ok) { toast(d.message); return; }
    viewingRun = name;
    stopLogStream();
    el('logArea').textContent = (d.truncated ? '（日志过大，仅显示末尾 256KB）\n' : '') + d.text;
    el('logArea').scrollTop = 0;
    el('backLiveBtn').style.display = '';
    el('viewingRunInfo').textContent = '正在回看: ' + name;
    loadRuns();                 // 重渲染侧边栏, 让当前条目高亮（渲染按 viewingRun 判 active）
    loadRunFrames(name);
  } catch(e) { toast('读取失败: ' + e); }
}
function backToLive(){
  viewingRun = null;
  document.querySelectorAll('#runsList .logside-item').forEach(function(it){ it.classList.remove('active'); });
  el('backLiveBtn').style.display = 'none';
  el('viewingRunInfo').textContent = '';
  el('framesCard').style.display = 'none';
  el('frameFilters').style.display = 'none';
  startLogStream();
}
// ---- 相机帧分类筛选（2026-10-21 加）----
// 文件名 = <key>_<序号>.jpg（logstore.archive_frame 的命名契约）; key → 分类:
//   qr / qr_<算法>（wechat/diff/... , _main 结尾=主摄）→ QR
//   road / road_yaw（road_yaw_* 与 *_before 变体都算）  → Road
//   color / rect（rect = 色球区的矩形地板, 同一个区域的事）→ Color
//   target（aim* 是靶标流/练习的定格历史键, 认得就归进来）  → Target
//   hostage → Hostage
// 认不出的 key 只进「全部」, 不猜。
const FRAME_CATS = [
  ['QR',      function(k){ return k.indexOf('qr') === 0; }],
  ['Road',    function(k){ return k.indexOf('road') === 0; }],
  ['Color',   function(k){ return k === 'color' || k === 'rect'; }],
  ['Target',  function(k){ return k === 'target' || k.indexOf('aim') === 0; }],
  ['Hostage', function(k){ return k === 'hostage'; }],
];
function frameKeyOf(file){ return String(file).replace(/_\d{3,5}\.jpg$/, ''); }
function frameCatOf(file){
  const k = frameKeyOf(file).replace(/_before$/, '');
  for (const c of FRAME_CATS) if (c[1](k)) return c[0];
  return '';
}
// 训练模式的 before 帧排在其**同序号**结果帧前面（先看原始、再看标注, 正好一对）
function frameSortKey(file){ return String(file).replace(/_before(?=_\d{3,5}\.jpg$)/, ''); }
let frameView = { run: null, frames: [], cat: '全部' };
let frameMenuCache = [];   // renderFrameGrid 按显示顺序缓存可见帧（右键菜单下标引用, 与 runCache 同纪律）
async function loadRunFrames(name){
  try {
    const r = await fetch('/api/logs/frames?run=' + encodeURIComponent(name));
    const d = await r.json();
    if (!d.ok || !d.frames.length) { el('framesCard').style.display = 'none'; el('frameFilters').style.display = 'none'; return; }
    frameView = { run: name, frames: d.frames, cat: '全部' };
    el('framesTitle').textContent = '相机帧 · ' + name + '（按执行先后, 序号不覆盖）';
    renderFrameFilters();
    renderFrameGrid();
    el('framesCard').style.display = '';
    el('frameFilters').style.display = '';
  } catch(e) { el('framesCard').style.display = 'none'; }
}
function renderFrameFilters(){
  const counts = {};
  for (const f of frameView.frames) {
    const c = frameCatOf(f.file) || '其它';
    counts[c] = (counts[c] || 0) + 1;
  }
  const cats = ['全部'];
  for (const c of FRAME_CATS) cats.push(c[0]);
  if (counts['其它']) cats.push('其它');
  let html = '';
  for (const c of cats) {
    const n = (c === '全部') ? frameView.frames.length : (counts[c] || 0);
    html += '<button class="fchip' + (frameView.cat === c ? ' on' : '') + '"'
      + (n ? '' : ' style="opacity:0.45"')
      + ' onclick="frameSetCat(\'' + c + '\')">' + esc(c) + ' <small>' + n + '</small></button>';
  }
  el('frameFilters').innerHTML = html;
}
function frameSetCat(c){
  frameView.cat = c;
  renderFrameFilters();
  renderFrameGrid();
}
function renderFrameGrid(){
  const run = frameView.run;
  const frames = frameView.frames.slice().sort(function(a, b){
    const ka = frameSortKey(a.file), kb = frameSortKey(b.file);
    if (ka !== kb) return ka < kb ? -1 : 1;
    const ba = a.file.indexOf('_before_') >= 0, bb = b.file.indexOf('_before_') >= 0;
    if (ba !== bb) return ba ? -1 : 1;
    return a.file < b.file ? -1 : 1;
  });
  let html = '';
  frameMenuCache = [];
  for (const f of frames) {
    const cat = frameCatOf(f.file);
    if (frameView.cat === '其它') { if (cat) continue; }
    else if (frameView.cat !== '全部' && frameView.cat !== cat) continue;
    const fidx = frameMenuCache.length;      // 右键菜单按下标引用（与 runCache 同纪律）
    frameMenuCache.push(f);
    const src = '/api/logs/frame?run=' + encodeURIComponent(run) + '&file=' + encodeURIComponent(f.file);
    const badge = f.file.indexOf('_before_') >= 0 ? ' <span class="kv">〔原始〕</span>' : '';
    html += '<div class="cam"><h3>' + esc(f.file) + badge + ' <span class="kv">' + f.time + '</span></h3>'
      + '<div class="box"><img loading="lazy" src="' + src + '" onclick="window.open(\'' + src + '\')"'
      + ' oncontextmenu="return frameMenu(event,' + fidx + ')" title="左键放大 · 右键验证"></div></div>';
  }
  el('framesGrid').innerHTML = html || '<div class="kv">这个分类下没有帧。</div>';
}
// 日志清理：入口在日志 Tab 按钮行（悬浮后面）, 弹窗选范围 —— 弹窗代替 confirm,
// 误触时可以取消; 点遮罩空白处同样关闭。
function showLogClean(){
  el('logCleanState').textContent = '';
  el('logCleanModal').style.display = 'flex';
}
function closeLogClean(){ el('logCleanModal').style.display = 'none'; }
async function doCleanLogs(scope){
  el('logCleanState').textContent = '清理中...';
  try {
    const d = await postJSON('/api/logs/clean', {scope});
    el('logCleanState').textContent = d.message || (d.ok ? '完成' : '失败');
    loadRuns();                              // 侧边栏列表同步刷新
    if (d.ok) setTimeout(closeLogClean, 1200);
  } catch(e) { el('logCleanState').textContent = '清理失败: ' + e; }
}

// ---- 单片机模拟器 ----
async function refreshSim(){
  try {
    const r = await fetch('/api/sim/status');
    const d = await r.json();
    if (JSON.stringify(d.commands) !== JSON.stringify(simCommands)) { simCommands = d.commands; renderSimButtons(); }
    let st;
    if (!d.enabled) {
      st = '<span class="bad">● 模拟器未启动</span>　' +
           '<span class="kv">点「启动模拟器」监听 ' + d.host + ':' + d.port + '，等待中的 main.py 会自动切入模拟模式</span>';
    } else if (d.client_connected) {
      st = '<span class="ok">● main.py 已连接（模拟模式）</span>';
    } else {
      st = '<span class="warn">● 监听 ' + d.host + ':' + d.port + '，等待 main.py 连接...</span>';
    }
    el('simStatus').innerHTML = st;
  } catch(e) {}
}
async function simStart(){
  // 启动模拟器前检查 main.py 是否运行，未运行则提示是否先启动
  if (!state.service_active && !state.mainpy_running) {
    const go = confirm('main.py 未运行。\n\n点「确定」= 先启动 main.py（调试模式）再启动模拟器\n点「取消」= 只启动模拟器（稍后手动启动 main.py 也能连上）');
    if (go) { const d = await postJSON('/api/mainpy', {action:'start'}); toast(d.message); }
  }
  await fetch('/api/sim/start', {method:'POST'});
  refreshSim(); refreshStatus();
}
async function simStop(){ await fetch('/api/sim/stop', {method:'POST'}); refreshSim(); }
function renderSimButtons(){
  let html = '';
  for (const c of simCommands) html += '<button onclick="simSend(\'' + c[0] + '\')" title="' + esc(c[2]) + '">' + esc(c[1]) + '</button>';
  el('simButtons').innerHTML = html;
}
async function simSend(key){
  const t0 = Date.now() / 1000;  // 记录发送时刻，用于判断结果帧是否为新
  // 先同步等 /abort 落地再发指令（2026-10 修）: 旧版是 fire-and-forget, /abort 可能
  // 晚于用户随后点开的就地识别的 _scan_abort.clear() 落地, 把新任务莫名取消
  // （症状: 点任务显示「已取消」, 与 main.py 是否在跑无关）。
  try {
    if (key === '1') {
      // 扫码走 main.py：先停掉二维码推流释放摄像头，再发指令
      await fetch('/abort?cam=qr');
      el('box-qr').innerHTML = ph('扫码中...');
    }
    if (key === 'D') {
      // 主摄扫码走 main.py：**它用主摄**，所以停的是主摄推流（不是二维码推流），
      // 结果也落在主摄那一格。结果帧与 "1" 同一种（§6a），
      // 所以照样轮询 /api/qr/frame（_cmd_qr_common 两条命令都写同一个 qr_last.jpg）。
      await fetch('/abort?cam=main');
      el('box-main').innerHTML = ph('扫码中(主摄)...');
      el('camStatus').textContent = '扫码中（由 main.py 用主摄执行，对应命令 "D"）...';
    }
    if (key === '4') {
      // 人质识别走 main.py：先停掉主摄推流释放摄像头，再发指令
      await fetch('/abort?cam=main');
      el('box-main').innerHTML = ph('人质识别中...');
      el('camStatus').textContent = '人质识别中（由 main.py 执行，凑满 3 个目标，限时 ~6s）...';
    }
    if (key === '5') {
      // 色球识别走 main.py：同上
      await fetch('/abort?cam=main');
      el('box-main').innerHTML = ph('色球识别中...');
      el('camStatus').textContent = '色球识别中（由 main.py 执行，凑满红/绿/蓝三色，限时 ~5s）...';
    }
    if (key === '6') {
      // 靶标识别走 main.py：同上
      await fetch('/abort?cam=main');
      el('box-main').innerHTML = ph('靶标识别中...');
      el('camStatus').textContent = '靶标识别中（由 main.py 执行，跨色尺寸一致性定靶心，限时 ~6s）...';
    }
    if (key === 'G') {
      // 看路走 main.py：它用主摄（先停主摄推流释放设备），回 0x08 帧。
      // ⚠️ 要求小车**静止**（多帧投票要同一机位）；限时由单片机那一步给。
      await fetch('/abort?cam=main');
      el('box-main').innerHTML = ph('看路识别中...');
      el('camStatus').textContent = '看路识别中（由 main.py 用主摄执行，最多 9 帧，需要小车静止）...';
    }
    if (key === 'H') {
      // 看路校准走 main.py：它用主摄（先停主摄推流释放设备），回 0x09 帧。
      // ⚠️ 要求小车**静止**；只测不改 —— 结果看 roadyaw_* 遥测与标注帧。
      await fetch('/abort?cam=main');
      el('box-main').innerHTML = ph('看路校准中...');
      el('camStatus').textContent = '看路校准中（由 main.py 用主摄执行，最多 9 帧，需要小车静止）...';
    }
  } catch(e) { /* /abort 失败不拦指令发送 */ }
  const d = await postJSON('/api/sim/command', {key});
  if (!d.ok) toast(d.message);
  if (key === '1') pollQrFrame(t0, 'box-qr');
  if (key === 'D') pollQrFrame(t0, 'box-main');   // 主摄扫码的标注帧显示在主摄那一格
  if (key === '4') pollHostageFrame(t0);
  if (key === '5') pollColorFrame(t0);
  if (key === '6') pollTargetFrame(t0);
  if (key === 'G') pollRoadFrame(t0);
  if (key === 'H') pollRoadYawFrame(t0);
}
// 靶标识别走 main.py：轮询等 main.py 存下的标注帧（logs/target_last.jpg），
// 拿到后显示「框选好的」画面 + 从模拟器最近收到的靶标结果帧取各靶坐标与半径
async function pollTargetFrame(t0){
  for (let i = 0; i < 15; i++) {
    await sleep(1000);
    try {
      const r = await fetch('/api/target/frame?t=' + Date.now());
      if (r.ok) {
        const mtime = parseFloat(r.headers.get('X-Frame-Mtime') || '0');
        if (mtime >= t0) {
          el('box-main').innerHTML = '<img src="/api/target/frame?t=' + Date.now() + '">';
          await showTargetResult();
          return;
        }
      }
    } catch(e) {}
  }
  el('camStatus').innerHTML = '<span class="warn">未获取到靶标识别结果帧</span>' +
                              '（main.py 侧可能超时/未执行，看模拟器日志确认）';
}

// 看路（命令 "G"）：等 main.py 写出 logs/road_last.jpg，再显示 0x08 帧里的测量值。
// ⚠️ 与 scanRoad() 的区别：scanRoad 是**页面就地**跑（不走串口），
//    这里走的是**真链路**（页面 → 模拟器 → main.py → 回 0x08 帧）——
//    台架上验串口这一侧就该按这个按钮。
async function pollRoadFrame(t0){
  for (let i = 0; i < 15; i++) {
    await sleep(1000);
    try {
      const r = await fetch('/api/road/frame?t=' + Date.now());
      if (r.ok) {
        const mtime = parseFloat(r.headers.get('X-Frame-Mtime') || '0');
        if (mtime >= t0) {
          el('box-main').innerHTML = '<img src="/api/road/frame?t=' + Date.now() + '">';
          await showRoadResult();
          return;
        }
      }
    } catch(e) {}
  }
  el('camStatus').innerHTML = '<span class="warn">未获取到看路结果帧</span>' +
                              '（main.py 侧可能超时/未执行，看模拟器日志确认）';
}

// 读模拟器记住的那一帧 0x08（structured），把测量值写到状态行。
// ⚠️ Δ 是**页面自己按两点距离算的**，与单片机上那一个应当一致：
//    对不上就说明帧解析串位了（这是跨仓库契约最直接的自检）。
async function showRoadResult(){
  try {
    const r = await fetch('/api/sim/status');
    const d = await r.json();
    const rd = d.road;
    if (!rd) { el('camStatus').innerHTML = '<span class="ok">已显示看路最后一帧</span>（未收到 0x08 帧）'; return; }
    if (!rd.status) {
      el('camStatus').innerHTML = '<span class="warn">看路帧 status=0</span>'
        + '（没找到黄区/超时）· ' + rd.ts;
      return;
    }
    const fl = rd.flags || 0;
    el('camStatus').innerHTML = '看路: <b>Δx=' + rd.dx + 'px</b>'
      + ' · 左边缘 <b>(' + rd.ex + ',' + rd.ey + ')</b>'
      + ' · ' + (fl & 1 ? '边缘有效' : '<span class="warn">边缘缺失</span>')
      + (fl & 2 ? ' · <span class="warn">贴画面左边(饱和)</span>' : '')
      + (fl & 4 ? ' · 贴上下边' : '')
      + (fl & 8 ? ' · <span class="warn">面积勉强</span>' : '')
      + ' · 散布' + rd.spread + 'px · ' + rd.ts;
  } catch(e) {
    el('camStatus').innerHTML = '<span class="ok">已显示看路最后一帧</span>';
  }
}
// 看路校准（命令 "H"）：等 main.py 写出 logs/roadyaw_last.jpg，再显示 0x09 帧里的方向角。
// ⚠️ 与 scanRoadYaw() 的区别：scanRoadYaw 是**页面就地**跑（不走串口），
//    这里走的是**真链路**（页面 → 模拟器 → main.py → 回 0x09 帧）——
//    台架上验串口这一侧就该按这个按钮。
async function pollRoadYawFrame(t0){
  for (let i = 0; i < 15; i++) {
    await sleep(1000);
    try {
      const r = await fetch('/api/roadyaw/frame?t=' + Date.now());
      if (r.ok) {
        const mtime = parseFloat(r.headers.get('X-Frame-Mtime') || '0');
        if (mtime >= t0) {
          el('box-main').innerHTML = '<img src="/api/roadyaw/frame?t=' + Date.now() + '">';
          await showRoadYawResult();
          return;
        }
      }
    } catch(e) {}
  }
  el('camStatus').innerHTML = '<span class="warn">未获取到看路校准结果帧</span>' +
                              '（main.py 侧可能超时/未执行，看模拟器日志确认）';
}

// 读模拟器记住的那一帧 0x09（structured），把方向角写到状态行。
async function showRoadYawResult(){
  try {
    const r = await fetch('/api/sim/status');
    const d = await r.json();
    const ry = d.roadyaw;
    if (!ry) { el('camStatus').innerHTML = '<span class="ok">已显示看路校准最后一帧</span>（未收到 0x09 帧）'; return; }
    if (!ry.status) {
      el('camStatus').innerHTML = '<span class="warn">看路校准帧 status=0</span>'
        + '（没找到黄区/拟合不过关/超时）· ' + ry.ts;
      return;
    }
    const fl = ry.flags || 0;
    el('camStatus').innerHTML = '看路校准: <b>ang=' + ry.ang + '°</b>'
      + ' · 内点 ' + ry.cols + ' · RMS ' + ry.rms + 'px'
      + ' · ' + (fl & 1 ? '拟合有效' : '<span class="warn">拟合缺失</span>')
      + (fl & 2 ? ' · <span class="warn">直段偏短(勉强)</span>' : '')
      + ' · 散布' + ry.spread + '° · ' + ry.ts;
  } catch(e) {
    el('camStatus').innerHTML = '<span class="ok">已显示看路校准最后一帧</span>';
  }
}
async function showTargetResult(){
  try {
    const r = await fetch('/api/sim/status');
    const d = await r.json();
    const t = d.target;
    if (!t || !t.count) {
      el('camStatus').innerHTML = '<span class="ok">已显示靶标识别最后一帧</span>（未检出靶）';
      return;
    }
    // 已由 Pi 端按中心 x 升序排好。r 是**外圆等效半径**（内边界拟合值折算过），
    // 固件的视差补偿 offset = h·r_px/50 就用它，现场可拿它粗估距离。
    const order = t.targets.map((x, i) => (i + 1) + '.' + x.name
                                           + '(' + x.cx + ',' + x.cy + ')r' + x.r).join(' ');
    el('camStatus').innerHTML = '识别成功（从左到右）: ' + order;
  } catch(e) {
    el('camStatus').innerHTML = '<span class="ok">已显示靶标识别最后一帧</span>';
  }
}
async function simSendCustom(){ const k = el('simCustom').value.trim(); if (k) { await simSend(k); el('simCustom').value=''; } }
function sleep(ms){ return new Promise(res => setTimeout(res, ms)); }
// boxId: 标注帧显示到哪一格 —— '1'（扫码副摄）→ box-qr；'D'（主摄）→ box-main。
// ⚠️ 两条命令都写同一个 logs/qr_last.jpg（_cmd_qr_common），所以端点只有一个。
async function pollQrFrame(t0, boxId){
  const box = boxId || 'box-qr';
  for (let i = 0; i < 12; i++) {
    await sleep(1000);
    try {
      const r = await fetch('/api/qr/frame?t=' + Date.now());
      if (r.ok) {
        const mtime = parseFloat(r.headers.get('X-Frame-Mtime') || '0');
        if (mtime >= t0) {
          el(box).innerHTML = '<img src="/api/qr/frame?t=' + Date.now() + '">';
          el('camStatus').innerHTML = '<span class="ok">已显示扫码最后一帧</span>';
          return;
        }
      }
    } catch(e) {}
  }
  el('camStatus').innerHTML = '<span class="warn">未获取到扫码结果帧</span>';
}

// 人质识别走 main.py：轮询等 main.py 存下的标注帧（logs/hostage_last.jpg），
// 拿到后显示「框选好的」画面 + 从模拟器最近收到的人质结果帧取顺序与坐标
async function pollHostageFrame(t0){
  for (let i = 0; i < 15; i++) {
    await sleep(1000);
    try {
      const r = await fetch('/api/hostage/frame?t=' + Date.now());
      if (r.ok) {
        const mtime = parseFloat(r.headers.get('X-Frame-Mtime') || '0');
        if (mtime >= t0) {
          el('box-main').innerHTML = '<img src="/api/hostage/frame?t=' + Date.now() + '">';
          await showHostageResult();
          return;
        }
      }
    } catch(e) {}
  }
  el('camStatus').innerHTML = '<span class="warn">未获取到人质识别结果帧</span>' +
                              '（main.py 侧可能超时/未执行，看模拟器日志确认）';
}
async function showHostageResult(){
  try {
    const r = await fetch('/api/sim/status');
    const d = await r.json();
    const h = d.hostage;
    if (!h || !h.count) {
      el('camStatus').innerHTML = '<span class="ok">已显示人质识别最后一帧</span>（未检出目标）';
      return;
    }
    // 目标已由 Pi 端按中心 x 升序排好：1.圆锥(640,512) 2.圆柱(960,508) ...
    const order = h.targets.map((t, i) => (i + 1) + '.' + t.name
                                           + '(' + t.cx + ',' + t.cy + ')').join(' ');
    el('camStatus').innerHTML = '识别成功（从左到右）: ' + order;
  } catch(e) {
    el('camStatus').innerHTML = '<span class="ok">已显示人质识别最后一帧</span>';
  }
}

// 色球识别走 main.py：轮询等 main.py 存下的标注帧（logs/color_last.jpg），
// 拿到后显示「框选好的」画面 + 从模拟器最近收到的色球结果帧取顺序与坐标
async function pollColorFrame(t0){
  for (let i = 0; i < 15; i++) {
    await sleep(1000);
    try {
      const r = await fetch('/api/color/frame?t=' + Date.now());
      if (r.ok) {
        const mtime = parseFloat(r.headers.get('X-Frame-Mtime') || '0');
        if (mtime >= t0) {
          el('box-main').innerHTML = '<img src="/api/color/frame?t=' + Date.now() + '">';
          await showColorResult();
          return;
        }
      }
    } catch(e) {}
  }
  el('camStatus').innerHTML = '<span class="warn">未获取到色球识别结果帧</span>' +
                              '（main.py 侧可能超时/未执行，看模拟器日志确认）';
}
async function showColorResult(){
  try {
    const r = await fetch('/api/sim/status');
    const d = await r.json();
    const c = d.color;
    if (!c || !c.count) {
      el('camStatus').innerHTML = '<span class="ok">已显示色球识别最后一帧</span>（未检出三色）';
      return;
    }
    // 目标已由 Pi 端按中心 x 升序排好：1.红(400,500) 2.绿(800,505) 3.蓝(1200,510)
    const order = c.targets.map((t, i) => (i + 1) + '.' + t.name
                                           + '(' + t.cx + ',' + t.cy + ')').join(' ');
    el('camStatus').innerHTML = '识别成功（从左到右）: ' + order;
  } catch(e) {
    el('camStatus').innerHTML = '<span class="ok">已显示色球识别最后一帧</span>';
  }
}

setInterval(refreshStatus, 2000);
setInterval(refreshSim, 1000);
setInterval(autoQrRefresh, 2000);   // 2026-10-18 加: 单片机 OLED 也能改自动下发二维码, 勾选框要跟着文件走
refreshStatus(); refreshSim();
autoQrRefresh();
initFloatDrag();
initCamTuneDrag();      // 主摄参数悬浮窗拖动（2026-10-30 加）
// 历史运行右键菜单: 点别处 / 滚 / Esc / 在别处再右键 都收起
document.addEventListener('click', hideRunCtx);
document.addEventListener('click', hideFrameCtx);
document.addEventListener('scroll', function(){ hideRunCtx(); hideFrameCtx(); }, true);
window.addEventListener('resize', function(){ hideRunCtx(); hideFrameCtx(); });
document.addEventListener('keydown', function(e){ if (e.key === 'Escape') { hideRunCtx(); hideFrameCtx(); } });
document.addEventListener('contextmenu', function(e){
  if (!e.target.closest || !e.target.closest('#runsList')) hideRunCtx();
  if (!e.target.closest || !e.target.closest('#framesGrid')) hideFrameCtx();
});
</script>
</body>
</html>
"""

# ─────────────────────────── 视差标定页（/parallax，2026-10 加）───────────────────────────
# 刻意**独立于**上面的 HTML_PAGE 单页：主页面 1500 行前端一根线牵全身，标定工具
# 是低频功能，单独一页改坏也只坏自己 —— 从主页面 tabnav 只放一个跳转按钮进来。
PARALLAX_PAGE = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>GKS 视差标定</title>
<style>
  body { font-family: sans-serif; background:#1e1e2e; color:#e0e0e0; margin:0; padding:16px; }
  h1 { font-size:20px; color:#89b4fa; margin:0 0 12px; }
  .card { background:#262636; border:1px solid #3a3a4e; border-radius:8px; padding:12px; margin-bottom:12px; }
  .card h2 { font-size:15px; color:#cdd6f4; margin:0 0 8px; }
  .kv { font-size:13px; color:#a6adc8; line-height:1.7; }
  .kv b { color:#e0e0e0; }
  button { background:#1e66f5; color:#fff; border:none; border-radius:6px; padding:9px 14px;
           font-size:13px; font-weight:bold; cursor:pointer; }
  button.g { background:#40a02b; } button.r { background:#d20f39; }
  button.o { background:#df8e1d; }
  #wrap { position:relative; display:inline-block; max-width:100%; }
  #pv { max-width:100%; display:block; border-radius:6px; cursor:crosshair; }
  #mark { position:absolute; width:26px; height:26px; margin:-13px 0 0 -13px; display:none;
          pointer-events:none; }
  #mark::before, #mark::after { content:""; position:absolute; background:#f5c211; }
  #mark::before { left:12px; top:0; width:2px; height:26px; }
  #mark::after { left:0; top:12px; width:26px; height:2px; }
  table { border-collapse:collapse; font-size:13px; width:100%; }
  th,td { border:1px solid #3a3a4e; padding:5px 8px; text-align:right; }
  th { color:#a6adc8; font-weight:normal; }
  pre { background:#11111b; border:1px solid #3a3a4e; border-radius:6px; padding:10px;
        font-size:12px; overflow:auto; white-space:pre; }
  #msg { font-size:13px; min-height:18px; margin:6px 0; }
  .ok { color:#40a02b; } .err { color:#d20f39; }
</style>
</head>
<body>
<h1>视差标定 <span style="font-size:12px;color:#a6adc8">center 随靶距 r 的锚点表</span></h1>
<div class="card kv">
  <b>原理</b>：相机与激光不同轴 —— 车与靶的<b>距离</b>一变，"光斑打到圆心时靶心的图像位置"
  就跟着漂（视差），而闭环把靶心拉向的是一个<b>固定像素点</b> tg_center。本页把若干个
  距离下的"正确瞄准点"采下来，生成按 r 分段插值的锚点表，MCU 端 TARGET_CENTER_RCOMP_EN=1
  后按当帧 r 实时取 center。<br>
  <b>每个距离的采法</b>：① 车摆定一个距离 → ② 开激光，微调舵机把光斑<b>稳定停在物理圆心</b>
  → ③ 在下面画面里<b>点那颗靶的靶心</b>（自动抓当帧检测值，同距离重复采 3~5 次更好）。
  换距离重复 ①~③，≥2 个距离后即可生成 C 片段。
</div>
<div class="card">
  <h2>预览（主摄）</h2>
  <div id="wrap">
    <img id="pv" src="/stream?cam=main&key=__GKS_KEY__" alt="主摄预览">
    <div id="mark"></div>
  </div>
  <div id="msg"></div>
  <button class="o" onclick="refresh()">刷新预览</button>
</div>
<div class="card">
  <h2>已采点 <span id="cnt" style="color:#a6adc8;font-weight:normal"></span></h2>
  <table id="tbl"><tr><th>时间</th><th>色</th><th>r(px)</th><th>cx</th><th>cy</th><th></th></tr></table>
  <div style="margin-top:10px">
    <button class="g" onclick="fit()">生成 C 片段</button>
    <button class="r" onclick="delAll()">清空全部</button>
  </div>
</div>
<div class="card" id="fitcard" style="display:none">
  <h2>锚点表 C 片段（粘进 app_config.h 的「视差锚点表」区，重编译烧录生效）</h2>
  <pre id="csnip"></pre>
  <button onclick="copyC()">复制</button>
  <div class="kv" id="linref" style="margin-top:8px"></div>
</div>
<script>
const KEY = "__GKS_KEY__";
const $ = id => document.getElementById(id);
function hdr() { return {"X-GKS-Key": KEY, "Content-Type": "application/json"}; }
function msg(t, ok) { $("msg").innerHTML = `<span class="${ok ? "ok" : "err"}">${t}</span>`; }

$("pv").addEventListener("click", async ev => {
  const img = $("pv"), mark = $("mark");
  const sx = img.naturalWidth / img.clientWidth;
  const sy = img.naturalHeight / img.clientHeight;
  const x = ev.offsetX * sx, y = ev.offsetY * sy;   // 换算回原始帧坐标（检测在全分辨率口径）
  mark.style.left = ev.offsetX + "px"; mark.style.top = ev.offsetY + "px";
  mark.style.display = "block";
  msg("采点中（抓帧+检测约 2s）…");
  try {
    const r = await fetch("/api/parallax/capture?key=" + KEY,
                          {method: "POST", headers: hdr(), body: JSON.stringify({x, y})});
    const d = await r.json();
    if (d.ok) {
      msg(`已采: 色${d.point.color} r=${d.point.r} cx=${d.point.cx} cy=${d.point.cy}`, true);
      loadList();
    } else { msg(d.message || "采点失败"); }
  } catch (e) { msg("请求失败: " + e); }
});

async function loadList() {
  const r = await fetch("/api/parallax/list?key=" + KEY);
  const d = await r.json();
  const tb = $("tbl");
  tb.innerHTML = "<tr><th>时间</th><th>色</th><th>r(px)</th><th>cx</th><th>cy</th><th></th></tr>";
  (d.points || []).forEach((p, i) => {
    const tr = document.createElement("tr");
    tr.innerHTML = `<td>${p.ts || ""}</td><td>${p.color}</td><td>${p.r}</td>` +
                   `<td>${p.cx}</td><td>${p.cy}</td>` +
                   `<td><button class="r" style="padding:2px 8px" onclick="del(${i})">删</button></td>`;
    tb.appendChild(tr);
  });
  $("cnt").textContent = `（${(d.points || []).length} 个，同距离建议 3~5 次）`;
}
async function del(i) {
  await fetch("/api/parallax/delete?key=" + KEY,
              {method: "POST", headers: hdr(), body: JSON.stringify({index: i})});
  loadList();
}
async function delAll() {
  const r = await fetch("/api/parallax/list?key=" + KEY);
  const d = await r.json();
  for (let i = (d.points || []).length - 1; i >= 0; i--) await del(i);
}
async function fit() {
  try {
    const r = await fetch("/api/parallax/fit?key=" + KEY,
                          {method: "POST", headers: hdr(), body: "{}"});
    const d = await r.json();
    if (!d.ok) { msg(d.message || "生成失败"); return; }
    $("csnip").textContent = d.c_snippet;
    const lr = d.linref || {};
    const warn = a => (a && a.rms > 2.0) ? " ⚠ rms 偏大，建议再多采 1~2 个距离" : "";
    $("linref").innerHTML =
      `线性度参考（直线拟合残差，仅作检查，锚点表本身分段插值）：<br>` +
      `center_x ≈ ${lr.x ? lr.x.a : "?"}·r + ${lr.x ? lr.x.b : "?"}，rms=${lr.x ? lr.x.rms : "?"}px${warn(lr.x)}<br>` +
      `center_y ≈ ${lr.y ? lr.y.a : "?"}·r + ${lr.y ? lr.y.b : "?"}，rms=${lr.y ? lr.y.rms : "?"}px${warn(lr.y)}`;
    $("fitcard").style.display = "block";
    msg("C 片段已生成", true);
  } catch (e) { msg("请求失败: " + e); }
}
function copyC() {
  navigator.clipboard.writeText($("csnip").textContent)
    .then(() => msg("已复制", true), () => msg("复制失败，请手动选择文本"));
}
function refresh() { const p = $("pv").src.split("&_")[0]; $("pv").src = p + "&_=" + Date.now(); }
loadList();
</script>
</body>
</html>
"""


# ─────────────────────────── 标靶练习页（/practice，2026-10 加）───────────────────
# 与 /parallax 同款「独立单页」姿态: 主页面 1500 行前端一根线牵全身, 采数据工具是
# 低频功能, 单独一页改坏也只坏自己。数据面（数值+帧）全部从 log/frames 读, 不新造
# 采集通道 —— 打靶本身由 OLED 第 3 页「执行任务→激光射靶」（预设 15）触发, 本页只做
# 三件事: 列发次 / 点光斑标注 / 把记录与帧拷进 JiGuangLogs。
PRACTICE_PAGE = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>GKS 标靶练习</title>
<style>
  body { font-family: sans-serif; background:#1e1e2e; color:#e0e0e0; margin:0; padding:16px; }
  h1 { font-size:20px; color:#89b4fa; margin:0 0 12px; }
  .card { background:#262636; border:1px solid #3a3a4e; border-radius:8px; padding:12px; margin-bottom:12px; }
  .card h2 { font-size:15px; color:#cdd6f4; margin:0 0 8px; }
  .kv { font-size:13px; color:#a6adc8; line-height:1.7; }
  .kv b { color:#e0e0e0; }
  button { background:#1e66f5; color:#fff; border:none; border-radius:6px; padding:9px 14px;
           font-size:13px; font-weight:bold; cursor:pointer; }
  button.g { background:#40a02b; } button.r { background:#d20f39; }
  button.o { background:#df8e1d; }
  input { background:#11111b; color:#e0e0e0; border:1px solid #3a3a4e; border-radius:6px;
          padding:7px 9px; font-size:13px; margin:0 4px 0 12px; width:120px; }
  #wrap { position:relative; display:inline-block; max-width:100%; }
  #pv { max-width:100%; display:block; border-radius:6px; cursor:crosshair; }
  #mark { position:absolute; width:26px; height:26px; margin:-13px 0 0 -13px; display:none;
          pointer-events:none; }
  #mark::before, #mark::after { content:""; position:absolute; background:#f5c211; }
  #mark::before { left:12px; top:0; width:2px; height:26px; }
  #mark::after { left:0; top:12px; width:26px; height:2px; }
  table { border-collapse:collapse; font-size:13px; width:100%; }
  th,td { border:1px solid #3a3a4e; padding:5px 8px; text-align:right; white-space:nowrap; }
  th { color:#a6adc8; font-weight:normal; }
  th:first-child, td:first-child { text-align:left; }
  tr.sel td { background:#31314a; }
  tr.shot { cursor:pointer; }
  td a { color:#89b4fa; }
  td.ed { cursor:text; text-decoration:underline dotted #5b5b78; }
  #views button.on { background:#f5c211; color:#11111b; }
  #msg { font-size:13px; min-height:18px; margin:6px 0; }
  .ok { color:#40a02b; } .err { color:#d20f39; }
</style>
</head>
<body>
<h1>标靶练习 <span style="font-size:12px;color:#a6adc8">真实流程打靶 → 点光斑攒落点误差数据</span></h1>
<div class="card kv">
  <b>用法</b>：① 车摆好（先填「本批条件」），OLED 第 3 页「执行任务 → 激光射靶」打一发
  （闭环超时也会照打——失败的样本正是要看的）→ ② OLED「开关激光」点亮激光（预设跑完臂保持
  终姿，光斑就是实际落点）→ ③ 在预览里<b>点光斑位置</b>。点完自动切「标记后」回看，可切回
  「定格帧·标记前」对照。<br>
  <b>定格帧</b>（默认）= 闭环最后看着的那张图，<b>直接在上面点</b>（对照物理靶上的光斑点位置；
  不碰相机、画面稳，推荐）；<b>实时</b> = MJPEG 里直接点光斑（图像直测）。两种来源记进
  mark_mode（映射/直测），切换视图本身不写任何数据。同一位置建议连打 3~5 发。
</div>
<div class="card">
  <h2>预览/标注（定格帧与实时里点击 = 标注；标记后为纯回看）</h2>
  <span id="views">
    <button class="o on" data-v="frozen" onclick="setView('frozen')">定格帧·标记前</button>
    <button class="o" data-v="live" onclick="setView('live')">实时</button>
    <button class="o" data-v="marked" onclick="setView('marked')">标记后</button>
  </span>
  <button class="o" onclick="refresh()">刷新</button>
  <button onclick="loadAll()">⟳ 刷新发次</button>
  <div id="wrap">
    <img id="pv" alt="预览">
    <div id="mark"></div>
  </div>
  <div id="msg"></div>
  <span id="active" class="kv"></span>
</div>
<div class="card">
  <h2>本批条件（随每发标注一起落盘，换位置/角度时改）</h2>
  <span class="kv">位置编号</span><input id="cpos" placeholder="如 A3">
  <span class="kv">车头角度</span><input id="cang" placeholder="如 0° / -15°">
  <span class="kv">备注</span><input id="cnote" style="width:220px" placeholder="如 远距/垫了纸">
  <button class="g" onclick="saveCond()">保存条件</button>
</div>
<div class="card">
  <h2>发次（活动运行 log.txt 解析；点一行 = 选为标注对象，默认最新未标注）</h2>
  <table id="tbl"></table>
</div>
<div class="card">
  <h2>已标注记录（最新在前，存 JiGuangLogs/shots.jsonl）</h2>
  <table id="rtbl"></table>
</div>
<script>
const KEY = "__GKS_KEY__";
const $ = id => document.getElementById(id);
const COLOR = {1: "R", 2: "G", 3: "B"};
function hdr() { return {"X-GKS-Key": KEY, "Content-Type": "application/json"}; }
function msg(t, ok) { $("msg").innerHTML = `<span class="${ok ? "ok" : "err"}">${t}</span>`; }
let activeShot = null;
let viewMode = "frozen";   // frozen = 定格帧标注(默认) | live = 实时标注 | marked = 回看

function setView(m) {
  const img = $("pv"), cur = activeShot;
  if (m === "frozen" && !(cur && cur.final_frame)) {
    msg("这一发没有定格帧 —— 切到「实时」标注", false);
    m = "live";
  } else if (m === "marked") {
    const dn = cur && cur.record && cur.record.dotann;
    if (!dn) { msg("这一发还没标注过，没有「标记后」可看", false); m = (cur && cur.final_frame) ? "frozen" : "live"; }
    else img.src = "/api/practice/frame?file=" + encodeURIComponent(dn) + "&_=" + Date.now();
  }
  if (m === "frozen")
    img.src = "/api/logs/frame?run=" + encodeURIComponent(cur.run) +
              "&file=" + encodeURIComponent(cur.final_frame) + "&_=" + Date.now();
  else if (m === "live")
    img.src = "/stream?cam=main&key=" + KEY;
  viewMode = m;
  $("mark").style.display = "none";
  document.querySelectorAll("#views button").forEach(b => b.classList.toggle("on", b.dataset.v === m));
  img.style.cursor = (m === "marked") ? "default" : "crosshair";
}

$("pv").addEventListener("click", async ev => {
  const img = $("pv"), mark = $("mark");
  if (viewMode === "marked") { msg("这是回看 —— 要标注请切「定格帧·标记前」或「实时」", false); return; }
  const x = ev.offsetX * img.naturalWidth / img.clientWidth;
  const y = ev.offsetY * img.naturalHeight / img.clientHeight;  // 原始帧坐标, 与检测同口径
  mark.style.left = ev.offsetX + "px"; mark.style.top = ev.offsetY + "px";
  mark.style.display = "block";
  if (!activeShot) { msg("还没有可标注的发次（先打一发）", false); return; }
  msg((viewMode === "frozen" ? "定格帧标注中（检测 ~1s）" : "实时标注中（抓帧+检测约 2s）")
      + ` → 发次 ${activeShot.time} …`);
  try {
    const r = await fetch("/api/practice/mark?key=" + KEY, {method: "POST", headers: hdr(),
                          body: JSON.stringify({x, y, run: activeShot.run, time: activeShot.time,
                                                mode: viewMode})});
    const d = await r.json();
    if (d.ok) {
      const rec = d.record;
      msg(`${d.replaced ? `已修正（覆盖了该发之前的 ${d.replaced} 条标注）` : "已记录"}`
        + `[${rec.mark_mode === "frozen" ? "定格帧·映射" : "实时·直测"}]: `
        + `打点(${rec.click[0]},${rec.click[1]}) 命中${rec.hit ? COLOR[rec.hit.color] || "?" : "?"}靶 `
        + `误差 dx=${rec.err ? rec.err[0] : "?"} dy=${rec.err ? rec.err[1] : "?"}px `
        + (rec.err_px != null ? `(|${rec.err_px}|px)` : "（本帧没检出靶, 只有打点帧）"), true);
      await loadAll();
      const s = allShots.find(t => t.run === activeShot.run && t.time === activeShot.time);
      if (s) setActive(s);
      setView("marked");       // 点完直接回看标记结果, 对照「定格帧」即为标记前
    } else { msg(d.message || "标注失败", false); }
  } catch (e) { msg("请求失败: " + e, false); }
});

function setActive(s) {
  activeShot = s;
  document.querySelectorAll("#tbl tr").forEach(tr => tr.classList.remove("sel"));
  const row = s ? document.getElementById("shot-" + s.idx) : null;
  if (row) row.classList.add("sel");
  $("active").textContent = s ? `→ 标注对象: ${s.time} ${s.outcome || "…"}` : "";
  // 换发次: 冻结/回看视图跟着换图; 实时视图保持不动（别打断正在标注的人）
  if (s && viewMode !== "live") setView(s.final_frame ? "frozen" : "live");
  if (!s) setView("live");
}

let allShots = [];          // 升序全量发次（loadShots 填, 标注后按 run+time 找回当前发）

async function loadShots() {
  try {
    const r = await fetch("/api/practice/shots?key=" + KEY);
    const d = await r.json();
    const tb = $("tbl");
    tb.innerHTML = "<tr><th>时间</th><th>结果</th><th>耗时s</th><th>dx,dy px</th>"
                 + "<th>舵机X,Y°</th><th>底盘mm</th><th>r px</th><th>定格帧</th><th>标注误差px</th></tr>";
    allShots = d.shots || [];
    allShots.forEach(s => { s.run = d.run; });
    allShots.slice().reverse().forEach((s, ri) => {
      s.idx = allShots.length - 1 - ri;            // 原始下标（升序）
      const tr = document.createElement("tr");
      tr.className = "shot"; tr.id = "shot-" + s.idx;
      const rec = s.record;
      tr.innerHTML = `<td>${s.time || "?"}</td><td>${s.outcome || "（进行中/未收尾）"}</td>`
        + `<td>${s.n_ms != null ? (s.n_ms / 1000).toFixed(1) : "?"}</td>`
        + `<td>${s.dx != null || s.dy != null ? `${s.dx ?? "?"},${s.dy ?? "?"}` : "?"}</td>`
        + `<td>${s.trX != null ? `${s.trX},${s.trY}` : "?"}</td>`
        + `<td>${s.ch_mm != null ? s.ch_mm : "?"}</td>`
        + `<td>${s.r_px != null ? s.r_px : "?"}</td>`
        + `<td>${s.final_frame
              ? `<a href="/api/logs/frame?run=${encodeURIComponent(d.run)}&file=${encodeURIComponent(s.final_frame)}" target="_blank">看</a>`
              : "无"}</td>`
        + `<td>${rec ? `dx=${rec.err ? rec.err[0] : "?"} dy=${rec.err ? rec.err[1] : "?"} (${rec.err_px ?? "?"})`
                     + ` ${rec.mark_mode === "frozen" ? "映射" : "直测"}` : ""}</td>`;
      tr.onclick = () => setActive(s);
      tb.appendChild(tr);
    });
    // 默认选最新未标注的那发; 都标注过就选最新一发（换发次时视图跟着换图）
    let pick = null;
    for (let i = allShots.length - 1; i >= 0; i--) { if (!allShots[i].record) { pick = allShots[i]; break; } }
    if (!pick && allShots.length) pick = allShots[allShots.length - 1];
    setActive(pick);                                 // pick=null 也会落到实时视图
  } catch (e) { console.error(e); }
}

async function loadRecords() {
  try {
    const r = await fetch("/api/practice/records?key=" + KEY);
    const d = await r.json();
    const tb = $("rtbl");
    tb.innerHTML = "<tr><th>时间</th><th>位置</th><th>角度</th><th>结果</th><th>命中</th>"
                 + "<th>误差 x,y px</th><th>|err| px</th><th>r px</th><th>打点帧</th><th>定格帧</th>"
                 + "<th>过程log</th><th></th></tr>";
    (d.records || []).forEach(rec => {
      const tr = document.createElement("tr");
      const c = rec.cond || {};
      const f = rec.files || {};
      const lnk = n => n ? `<a href="/api/practice/frame?file=${encodeURIComponent(n)}" target="_blank">看</a>` : "";
      const seq = (f.seq || []).length;
      tr.innerHTML = `<td>${rec.ts || ""}</td>`
        + `<td class="ed" data-ts="${rec.ts || ""}" data-k="pos" title="双击修改">${c.pos || ""}</td>`
        + `<td class="ed" data-ts="${rec.ts || ""}" data-k="angle" title="双击修改">${c.angle || ""}</td>`
        + `<td>${rec.outcome || "?"}</td><td>${rec.hit ? (COLOR[rec.hit.color] || "?") : "—"}</td>`
        + `<td>${rec.err ? `${rec.err[0]},${rec.err[1]}` : "—"}</td>`
        + `<td>${rec.err_px != null ? rec.err_px : "—"}</td>`
        + `<td>${rec.r_px != null ? rec.r_px : "—"}</td>`
        + `<td>${lnk(f.dot)}${f.dotann ? "/" : ""}${lnk(f.dotann)}${seq ? ` +${seq}帧` : ""}</td>`
        + `<td>${lnk(f.final)}${f.finalann ? "/" : ""}${lnk(f.finalann)}</td>`
        + `<td>${rec.log_n ? `<a href="/api/practice/recordlog?ts=${encodeURIComponent(rec.ts)}&key=${KEY}" target="_blank">${rec.log_n} 行</a>` : "—"}</td>`
        + `<td><button class="r" style="padding:2px 8px" data-ts="${rec.ts || ""}" `
        + `onclick="delRec(this.dataset.ts)">删</button></td>`;
      tb.appendChild(tr);
    });
  } catch (e) { console.error(e); }
}

async function loadCond() {
  try {
    const r = await fetch("/api/practice/condition?key=" + KEY);
    const d = await r.json();
    $("cpos").value = d.cond ? d.cond.pos : "";
    $("cang").value = d.cond ? d.cond.angle : "";
    $("cnote").value = d.cond ? d.cond.note : "";
  } catch (e) { console.error(e); }
}
async function saveCond() {
  try {
    await fetch("/api/practice/condition?key=" + KEY, {method: "POST", headers: hdr(),
                body: JSON.stringify({pos: $("cpos").value, angle: $("cang").value,
                                      note: $("cnote").value})});
    msg("条件已保存（之后每发标注都会带上）", true);
  } catch (e) { msg("保存失败: " + e, false); }
}
async function delRec(ts) {
  if (!ts || !confirm("删除这条记录（连同它的打点帧/定格帧/过程帧）？")) return;
  try {
    const r = await fetch("/api/practice/delete?key=" + KEY, {method: "POST", headers: hdr(),
                          body: JSON.stringify({ts})});
    const d = await r.json();
    if (d.ok) { msg("已删除", true); loadAll(); } else { msg(d.message || "删除失败", false); }
  } catch (e) { msg("删除失败: " + e, false); }
}

// 双击「位置/角度」就地改这一条的标签（回车/失焦提交, Esc 反悔）—— 只改该条 cond
$("rtbl").addEventListener("dblclick", ev => {
  const td = ev.target.closest("td.ed");
  if (!td || !td.dataset.ts) return;
  if (td.querySelector("input")) return;               // 已在编辑中
  const old = td.textContent, key = td.dataset.k, ts = td.dataset.ts;
  td.textContent = "";
  const inp = document.createElement("input");
  inp.value = old; inp.style.width = "96px";
  td.appendChild(inp); inp.focus(); inp.select();
  let done = false;
  const commit = async () => {
    if (done) return;
    done = true;
    const v = inp.value.trim();
    td.textContent = v;
    if (v === old) return;
    try {
      const r = await fetch("/api/practice/reccond?key=" + KEY, {method: "POST", headers: hdr(),
                            body: JSON.stringify({ts, [key]: v})});
      const d = await r.json();
      msg(d.ok ? `已把该条的${key === "pos" ? "位置" : "角度"}标签改为「${v}」` : (d.message || "改标签失败"),
          !!d.ok);
    } catch (e) { msg("改标签失败: " + e, false); }
    loadRecords();
  };
  inp.addEventListener("keydown", e => {
    if (e.key === "Enter") commit();
    else if (e.key === "Escape") { done = true; td.textContent = old; }
  });
  inp.addEventListener("blur", commit);
});
function refresh() {
  if (viewMode === "live") { const p = $("pv").src.split("&_")[0]; $("pv").src = p + "&_=" + Date.now(); }
  else setView(viewMode);      // 静态视图 = 重新拉一次图（定格帧/标注帧刚更新时用）
}
async function loadAll() { await Promise.all([loadShots(), loadRecords()]); }
loadCond(); loadAll();
</script>
</body>
</html>
"""


# ─────────────────────────── HTTP Handler ───────────────────────────
class WebHandler(BaseHTTPRequestHandler):
    # 2026-10: 30s socket 超时 —— 治「客户端静默消失（合盖/WiFi 断/NAT 超时）后 MJPEG/SSE
    # 线程阻塞在 wfile.write 永不返回、相机被它占死」的问题; 也顺带给 _read_json 的
    # 半截 body 一个上限（30s 后 read 超时返回）。正常请求（含 MJPEG 大帧、慢 WiFi）
    # 单次读写远用不到 30s。
    timeout = 30

    def _cam_key(self):
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)
        return "qr" if qs.get("cam", ["main"])[0] == "qr" else "main"

    # ==================== 访问令牌（2026-10, 见 webconfig.WEB_KEY）====================
    def _req_key(self):
        """从 ?key= / X-GKS-Key 头 / cookie 三处取调用方出示的令牌。"""
        qs = parse_qs(urlparse(self.path).query)
        k = qs.get("key", [None])[0]
        if k:
            return k
        k = self.headers.get("X-GKS-Key")
        if k:
            return k
        cookie = self.headers.get("Cookie", "") or ""
        for part in cookie.split(";"):
            name, _, val = part.strip().partition("=")
            if name == "gks_key":
                return val
        return None

    def _authed(self):
        return (not WEB_KEY) or (self._req_key() == WEB_KEY)

    def _auth_post_ok(self):
        """POST 的认证门槛比 GET 严: 只认 ?key= 或 X-GKS-Key 头, **不认 cookie**。

        cookie 会被浏览器对任何站点自动携带 —— 只认 cookie 挡不住 CSRF; 而自定义头
        跨站发不出（预检会拦）, 攻击者网页里又不知道密钥, 两头都堵死。
        页面 JS 已统一 monkey-patch fetch 自动带 X-GKS-Key（见 HTML_PAGE 头部）。"""
        if not WEB_KEY:
            return True
        qs = parse_qs(urlparse(self.path).query)
        if qs.get("key", [None])[0] == WEB_KEY:
            return True
        return self.headers.get("X-GKS-Key") == WEB_KEY

    def _serve_login_page(self):
        body = ("<!DOCTYPE html><html lang='zh'><head><meta charset='utf-8'>"
                "<title>GKS 控制端 - 需要令牌</title>"
                "<body style='background:#1e1e2e;color:#e0e0e0;font-family:sans-serif;"
                "display:flex;align-items:center;justify-content:center;height:100vh;margin:0'>"
                "<form method='get' style='text-align:center'>"
                "<h2 style='color:#89b4fa'>GKS 控制端</h2>"
                "<p>本页面启用了访问令牌（webconfig.WEB_KEY）。</p>"
                "<input name='key' type='password' placeholder='访问令牌' autofocus "
                "style='padding:10px;border-radius:6px;border:1px solid #3a3a4e;"
                "background:#11111b;color:#e0e0e0'>"
                "<button style='padding:10px 16px;margin-left:8px;border:none;"
                "border-radius:6px;background:#1e66f5;color:#fff;font-weight:bold'>进入</button>"
                "</form></body></html>").encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _deny_post(self):
        body = json.dumps({"ok": False, "message": "令牌缺失或错误（需要 ?key= 或 X-GKS-Key 头）"}).encode("utf-8")
        self.send_response(403)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_html(self):
        # 页面只在认证通过后被请求 —— 就地嵌入令牌供 JS 用（fetch 补 X-GKS-Key 头）。
        body = HTML_PAGE.replace("__GKS_KEY__", WEB_KEY or "").encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Frame-Options", "DENY")     # 反点击劫持（本页不该被任何站 iframe）
        self.send_header("Cache-Control", "no-store")
        # 首次带 ?key= 进入 → 种 cookie（SSE/EventSource 发不了自定义头, 靠它）
        if WEB_KEY:
            k = parse_qs(urlparse(self.path).query).get("key", [None])[0]
            if k == WEB_KEY:
                self.send_header("Set-Cookie",
                                 f"gks_key={WEB_KEY}; Path=/; Max-Age=2592000; SameSite=Strict")
        self.end_headers()
        self.wfile.write(body)

    def _send_parallax_page(self):
        # 与 _send_html 同款令牌处理：嵌入 key 供 JS 用；带 ?key= 进来时种 cookie。
        body = PARALLAX_PAGE.replace("__GKS_KEY__", WEB_KEY or "").encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Cache-Control", "no-store")
        if WEB_KEY:
            k = parse_qs(urlparse(self.path).query).get("key", [None])[0]
            if k == WEB_KEY:
                self.send_header("Set-Cookie",
                                 f"gks_key={WEB_KEY}; Path=/; Max-Age=2592000; SameSite=Strict")
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, code, obj):
        body = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, body):
        b = body.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def _read_json(self):
        # 2026-10: 长度上限 1MB（畸形/恶意 Content-Length 不至于吃内存）; int() 解析
        # 加护（畸形头原来直接 500 掐断连接）; 半截 body 由类级 timeout=30 兜底。
        try:
            length = int(self.headers.get("Content-Length", 0) or 0)
        except ValueError:
            return {}
        if length <= 0:
            return {}
        length = min(length, 1048576)
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except Exception:
            return {}

    # ---- 控制台 API ----
    def _api_status(self):
        self._send_json(200, {
            "ip": console.get_ip_addresses(),
            "hostname": console.get_hostname(),
            "service_active": console.is_service_active(),
            "service_enabled": console.is_service_enabled(),
            "web_service_active": console.is_web_service_active(),
            "web_service_enabled": console.is_web_service_enabled(),
            "mainpy_running": console.is_main_py_running(),
            "system": console.get_system_status(),
            "power": POWER.status(),
            "simulator": {
                "enabled": SIMULATOR.is_enabled(),
                "client_connected": SIMULATOR.client_connected(),
                "host": SIM_HOST, "port": SIM_PORT,
            },
        })

    def _api_autoqr_get(self):
        """读「自动下发二维码」当前设置（页面加载时回显用）。"""
        self._send_json(200, _auto_qr_load())

    def _api_autoqr_post(self, body):
        """保存「自动下发二维码」设置 → GKS_Competition/web_auto_qr.json。

        服务端再校验一遍值域（111~333）：勾选开启但值非法 → 400 拒存（页面的黄框
        只是第一道，别让坏值落到文件里让 main.py 每次握手都打 WARN）。
        """
        enabled = bool(body.get("enabled"))
        value = str(body.get("value", "")).strip()
        if enabled and not _auto_qr_valid(value):
            self._send_json(400, {"ok": False,
                                  "message": "二维码每位数字须为 1~3（如 213），未保存"})
            return
        try:
            _auto_qr_save(enabled, value)
        except OSError as e:
            self._send_json(500, {"ok": False, "message": f"保存失败: {e}"})
            return
        self._send_json(200, {"ok": True, "enabled": enabled, "value": value})

    def _api_service(self, body):
        action = body.get("action", "")
        table = {
            "start": console.start_service,
            "stop": console.stop_service,
            "restart": console.restart_service,
            "enable": console.enable_service,
            "disable": console.disable_service,
        }
        fn = table.get(action)
        if fn is None:
            self._send_json(400, {"ok": False, "message": f"未知 action: {action}"})
            return
        ok, err = fn()
        self._send_json(200, {"ok": ok, "message": "操作成功" if ok else f"操作失败: {err}"})

    def _api_fix_devices(self):
        """运行设备固定脚本（重新生成 + 应用 udev 规则），返回脚本完整输出。

        输出含检测详情与符号链接验证结果（哪个绑定成功、哪个不存在），直接展示给用户。
        两个摄像头是否都已插入由前端 confirm 把关（脚本按检测顺序命名，少插会对调）。
        """
        ok, out = console.fix_devices()
        self._send_json(200, {"ok": ok, "output": out})

    def _api_webservice(self, body):
        """控制本 Web 控制端自身（gks-web.service）。

        重启走 console.restart_web_service()：延迟 1s + 脱离会话执行，先让本响应
        发出去再杀进程；前端收到 restarting 后轮询 /api/status，服务恢复即自动刷新。
        """
        action = body.get("action", "")
        if action == "restart":
            ok, msg = console.restart_web_service()
        elif action == "enable":
            ok, err = console.enable_web_service()
            msg = "已开启开机自启" if ok else f"操作失败: {err}"
        elif action == "disable":
            ok, err = console.disable_web_service()
            msg = "已关闭开机自启" if ok else f"操作失败: {err}"
        else:
            self._send_json(400, {"ok": False, "message": f"未知 action: {action}"})
            return
        self._send_json(200, {"ok": ok, "message": msg,
                              "restarting": (action == "restart" and ok)})

    # ---- WiFi（逻辑全在 console.py；这里只做参数转手）----
    def _api_wifi_status(self):
        self._send_json(200, console.wifi_status())

    def _api_wifi_scan(self):
        ok, data = console.wifi_scan()
        if not ok:
            self._send_json(200, {"ok": False, "message": data, "networks": []})
            return
        self._send_json(200, {"ok": True, "networks": data["networks"],
                              "rescan_limited": data["rescan_limited"]})

    def _api_wifi_connect(self, body):
        """切换 WiFi。**本接口会先应答再动手**（真正的切换在后台线程），
        因为请求正是从"即将被切走的那个网络"进来的，同步等就会发不出响应。"""
        ok, msg = console.begin_wifi_switch(body.get("ssid", ""),
                                            body.get("password", ""),
                                            bool(body.get("secure", True)))
        self._send_json(200, {"ok": ok, "message": msg, "switching": ok,
                              "hint_url": console.wifi_hint_url()})

    def _api_wifi_profile(self, body):
        action = body.get("action", "")
        name = (body.get("name") or "").strip()
        if action == "priority":
            ok, msg = console.set_wifi_priority(name, body.get("priority"))
        elif action == "prefer":
            ok, msg = console.prefer_wifi(name)
        elif action == "delete":
            ok, msg = console.delete_wifi_profile(name)
        elif action == "revert":
            ok, msg = console.wifi_revert()
        else:
            self._send_json(400, {"ok": False, "message": f"未知 action: {action}"})
            return
        self._send_json(200, {"ok": ok, "message": msg})

    def _api_mainpy(self, body):
        action = body.get("action", "")
        if action == "start":
            ok, msg = console.start_main_py()
        elif action == "kill":
            ok, msg = console.kill_main_py()
        elif action == "restart":
            ok, msg = console.restart_main_py()
        else:
            self._send_json(400, {"ok": False, "message": f"未知 action: {action}"})
            return
        self._send_json(200, {"ok": ok, "message": msg})

    def _api_power(self, body):
        action = body.get("action", "")
        if action == "reboot":
            ok, err = console.reboot()
        elif action == "shutdown":
            ok, err = console.shutdown()
        elif action == "clear_uv_log":
            ok, msg = POWER.clear_log()
            self._send_json(200, {"ok": ok, "message": msg})
            return
        else:
            self._send_json(400, {"ok": False, "message": f"未知 action: {action}"})
            return
        self._send_json(200, {"ok": ok, "message": "指令已发送" if ok else f"失败: {err}"})

    def _api_parallax_capture(self, body):
        """采一个视差锚点：body={x,y}（**原始帧坐标**，页面点击时已按 naturalWidth 换算）。
        流程：抓一帧 → 跑与在线同一套的靶标检测 → 取离点击最近的靶 →
        把它的 (r, cx, cy)（检测值，不是点击值 —— 精度来自检测器，点击只负责选哪颗）
        追加进 parallax_points.json。"""
        try:
            x = float(body.get("x")); y = float(body.get("y"))
        except (TypeError, ValueError):
            self._send_json(400, {"ok": False, "message": "缺少点击坐标 x/y"})
            return
        frame = _grab_main_frame()
        if frame is None:
            self._send_json(503, {"ok": False, "message": "主摄打开失败（被占用或不存在）"})
            return
        cands = []
        targets, _small = _detect_frame(frame, _build_target_cfg(),
                                        gks_config.TARGET_PROC_SCALE,
                                        debug_cands=cands)
        if not targets:
            self._send_json(200, {"ok": False,
                                  "message": "这一帧没检出任何靶（对好光斑再点一次试试）"})
            return
        best = min(targets, key=lambda t: (t["cx"] - x) ** 2 + (t["cy"] - y) ** 2)
        dist = ((best["cx"] - x) ** 2 + (best["cy"] - y) ** 2) ** 0.5
        if dist > PARALLAX_PICK_TOL_PX:
            near = ", ".join(f"({int(t['cx'])},{int(t['cy'])})" for t in targets)
            self._send_json(200, {"ok": False,
                                  "message": f"点击点离最近靶 {dist:.0f}px —— 点在靶心上。检出: {near}"})
            return
        pts = _parallax_load()
        pt = {"r": round(float(best["r"]), 1), "cx": round(float(best["cx"]), 1),
              "cy": round(float(best["cy"]), 1), "color": int(best["color"]),
              "pick": [round(x, 1), round(y, 1)], "ts": time.strftime("%m-%d %H:%M:%S")}
        pts.append(pt)
        _parallax_save(pts)
        self._send_json(200, {"ok": True, "point": pt, "count": len(pts)})

    def _api_parallax_list(self):
        self._send_json(200, {"ok": True, "points": _parallax_load()})

    def _api_parallax_delete(self, body):
        try:
            idx = int(body.get("index"))
        except (TypeError, ValueError):
            self._send_json(400, {"ok": False, "message": "缺少 index"})
            return
        pts = _parallax_load()
        if not (0 <= idx < len(pts)):
            self._send_json(400, {"ok": False, "message": "index 越界"})
            return
        removed = pts.pop(idx)
        _parallax_save(pts)
        self._send_json(200, {"ok": True, "removed": removed, "count": len(pts)})

    def _api_parallax_fit(self):
        """按 r 升序把采点整成 MCU 锚点表：分段线性插值用「采点本身」当锚（不外推、
        不假设全局线性），另给一条最小二乘直线作**线性度参考** —— 两轴 rms 大于
        ~2px 说明该多采几个距离点了。输出 C 片段直接粘进 app_config.h。"""
        pts = sorted(_parallax_load(), key=lambda p: p["r"])
        # 同一距离重复采的点（r 相邻差 <3px）归一组、组内取中位 —— 同距离多次
        # 采点取中位是标定纪律；⚠ 分组比较用组内**原始点**的 r，别用滚动的中位值。
        groups = []
        for p in pts:
            if groups and abs(groups[-1][-1]["r"] - p["r"]) < 3.0:
                groups[-1].append(p)
            else:
                groups.append([p])
        anchors = []
        for grp in groups:
            med = lambda k, g=grp: sorted(x[k] for x in g)[len(g) // 2]
            anchors.append({"r": med("r"), "cx": med("cx"), "cy": med("cy")})
        merged = anchors
        if len(merged) < 2:
            self._send_json(200, {"ok": False,
                                  "message": f"至少 2 个不同距离的采点（现在 {len(merged)}），"
                                             f"先把车挪个位置再采"})
            return
        ref = {}
        try:
            arr = np.array([[p["r"], p["cx"], p["cy"]] for p in merged])
            for axis, col in (("x", 1), ("y", 2)):
                a, b = np.polyfit(arr[:, 0], arr[:, col], 1)
                rms = float(np.sqrt(np.mean(
                    (arr[:, col] - (a * arr[:, 0] + b)) ** 2)))
                ref[axis] = {"a": round(float(a), 4), "b": round(float(b), 1),
                             "rms": round(rms, 2)}
        except Exception:
            ref = {}
        n = len(merged)
        r_list = ", ".join(f"{p['r']:.1f}" for p in merged)
        cx_list = ", ".join(f"{p['cx']:.1f}" for p in merged)
        cy_list = ", ".join(f"{p['cy']:.1f}" for p in merged)
        c_snippet = (
            f"#define TARGET_CENTER_RCOMP_EN   1      /* 视差补偿总开关 */\n"
            f"#define TARGET_CENTER_ANCHOR_N     {n}\n"
            f"#define TARGET_CENTER_ANCHOR_R     {{ {r_list} }}   /* **升序**! */\n"
            f"#define TARGET_CENTER_ANCHOR_CX    {{ {cx_list} }}\n"
            f"#define TARGET_CENTER_ANCHOR_CY    {{ {cy_list} }}")
        self._send_json(200, {"ok": True, "anchors": anchors,
                              "linref": ref, "c_snippet": c_snippet})

    # ---- 标靶练习（2026-10 加, /practice 页）----

    def _send_practice_page(self):
        # 与 _send_parallax_page 同款令牌处理
        body = PRACTICE_PAGE.replace("__GKS_KEY__", WEB_KEY or "").encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Cache-Control", "no-store")
        if WEB_KEY:
            k = parse_qs(urlparse(self.path).query).get("key", [None])[0]
            if k == WEB_KEY:
                self.send_header("Set-Cookie",
                                 f"gks_key={WEB_KEY}; Path=/; Max-Age=2592000; SameSite=Strict")
        self.end_headers()
        self.wfile.write(body)

    def _practice_read_log(self, run_dir):
        """读 run 目录 log.txt 的**尾部 2MB**（read_log 的 256KB 不够: 详细日志开着时
        一发就有几百行）。返回文本, 失败返回空串。"""
        try:
            log_path = os.path.join(run_dir, "log.txt")
            with open(log_path, "r", encoding="utf-8", errors="replace") as f:
                f.seek(0, 2)
                size = f.tell()
                f.seek(max(0, size - 2 * 1024 * 1024))
                return f.read()
        except OSError:
            return ""

    def _practice_shots_of(self, run):
        """run 名 → (run_dir|None, 解析出的发次列表)。"""
        run_dir = logstore.run_dir_of(run) if run else logstore.active_run_dir()
        if not run_dir:
            return None, []
        return run_dir, _parse_shots(self._practice_read_log(run_dir))

    def _api_practice_shots(self):
        """GET /api/practice/shots[?run=] —— 活动 run（或指定 run）的发次列表。
        每发带上已标注记录的摘要（run+cfg 行时间配对, 后写的记录胜）。"""
        run = parse_qs(urlparse(self.path).query).get("run", [""])[0]
        run_dir, shots = self._practice_shots_of(run)
        run = os.path.basename(run_dir) if run_dir else ""
        recs = {}
        for r in _practice_records():                 # 升序遍历 → 同键后者覆盖
            recs[(r.get("run"), r.get("shot_time"))] = r
        for s in shots:
            s.pop("lines", None)                   # 过程 log 只进记录, 不进列表载荷
            s["run"] = run
            r = recs.get((run, s.get("time")))
            if r:
                s["record"] = {"err": r.get("err"), "err_px": r.get("err_px"),
                               "hit": r.get("hit"), "ts": r.get("ts"),
                               "mark_mode": r.get("mark_mode"),
                               "dotann": (r.get("files") or {}).get("dotann")}
        self._send_json(200, {"ok": True, "run": run, "shots": shots})

    def _api_practice_records(self):
        """GET /api/practice/records —— JiGuangLogs/shots.jsonl（最新在前, ≤200 条）。
        ⚠️ 剥掉 `log`（每发几十~几百行, 200 条会撑爆载荷）, 换成行数 `log_n`;
        过程 log 按条取: /api/practice/recordlog?ts=。"""
        recs = _practice_records()[-200:]
        recs.reverse()
        for r in recs:
            r["log_n"] = len(r.pop("log", []) or [])
        self._send_json(200, {"ok": True, "records": recs})

    def _api_practice_recordlog(self):
        """GET /api/practice/recordlog?ts= —— 某条已标注记录的**完整过程 log**
        （标注时从 run 的 log.txt 固化进来的那一发整段, text/plain 直接可读）。"""
        want = parse_qs(urlparse(self.path).query).get("ts", [""])[0]
        for r in reversed(_practice_records()):
            if r.get("ts") == want and r.get("log"):
                body = "\n".join(r["log"]).encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/plain; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)
                return
        self.send_error(404, "not found")

    def _api_practice_condition(self, body=None):
        """GET 读 / POST 写「本批条件」（JiGuangLogs/condition.json, 随每发标注落盘）。"""
        if body is None:
            self._send_json(200, {"ok": True, "cond": _practice_cond_load()})
            return
        c = {"pos": str(body.get("pos", "")).strip(),
             "angle": str(body.get("angle", "")).strip(),
             "note": str(body.get("note", "")).strip()}
        try:
            _practice_cond_save(c)
        except OSError as e:
            self._send_json(200, {"ok": False, "message": f"写 condition.json 失败: {e}"})
            return
        self._send_json(200, {"ok": True, "cond": c})

    def _api_practice_frame(self):
        """GET /api/practice/frame?file= —— 回看一张 JiGuangLogs/frames 里的帧。
        文件名过白名单（无路径分隔符）, 防目录穿越 —— 同 /api/logs/frame 的纪律。"""
        fname = parse_qs(urlparse(self.path).query).get("file", [""])[0]
        if not _RE_PRACTICE_FRAME.match(fname):
            self.send_error(404, "not found")
            return
        path = os.path.join(PRACTICE_FRAMES_DIR, fname)
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError:
            self.send_error(404, "not found")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _practice_copy_seq(self, run, shot, ts):
        """把该发时间窗内的 aimseq_* 过程帧拷进 JiGuangLogs/frames/ → 拷贝文件名列表。

        没开归档开关时窗内一个文件都没有, 返回 []（常态, 不是错误）。时间窗 = 文件
        mtime 落在「cfg 行时刻 ~ 结束行时刻」各放宽 3s 内（run 名带日期 + 日志行带
        时分秒拼出绝对时刻; 跨午夜的发按 end<start → +1 天）。窗外的不拷 —— 别人的。
        """
        if not (run.startswith("run_") and shot.get("time") and shot.get("end_time")):
            return []
        try:
            day = run[4:12]                          # run_YYYYMMDD_HHMMSS
            start = datetime.strptime(day + shot["time"], "%Y%m%d%H:%M:%S") - timedelta(seconds=3)
            end = datetime.strptime(day + shot["end_time"], "%Y%m%d%H:%M:%S") + timedelta(seconds=3)
            if end < start:
                end += timedelta(days=1)
        except ValueError:
            return []
        frames_dir = os.path.join(logstore.LOG_DIR, run, "frames")
        try:
            names = sorted(n for n in os.listdir(frames_dir)
                           if n.startswith("aimseq_") and n.endswith(".jpg"))
        except OSError:
            return []
        out = []
        for n in names:
            try:
                mtime = os.path.getmtime(os.path.join(frames_dir, n))
            except OSError:
                continue
            if not (start.timestamp() <= mtime <= end.timestamp()):
                continue
            dst = f"{ts}_seq{len(out) + 1:02d}.jpg"
            try:
                shutil.copyfile(os.path.join(frames_dir, n),
                                os.path.join(PRACTICE_FRAMES_DIR, dst))
                out.append(dst)
            except OSError:
                pass
        return out

    def _api_practice_reccond(self, body):
        """POST /api/practice/reccond {ts, pos?|angle?|note?} —— 改某条已标注记录的条件标签。

        只动请求里给出的字段（记录按 ts 定位）。改标签不影响该发的任何其它数据
        （数值/帧/过程 log 原样）—— 标错位置不删记录重标, 双击改过来就行。"""
        want = str(body.get("ts", "") or "")
        upd = {k: str(body.get(k, "")).strip()
               for k in ("pos", "angle", "note") if k in body}
        if not want or not upd:
            self._send_json(400, {"ok": False, "message": "缺少 ts 或要改的字段"})
            return
        recs = _practice_records()
        hit = False
        for r in recs:
            if not hit and r.get("ts") == want:
                c = r.get("cond") or {}
                c.update(upd)
                r["cond"] = c
                hit = True
        if not hit:
            self._send_json(200, {"ok": False, "message": f"没有 ts={want} 的记录"})
            return
        try:
            tmp = PRACTICE_RECORDS_PATH + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                for r in recs:
                    f.write(json.dumps(r, ensure_ascii=False) + "\n")
            os.replace(tmp, PRACTICE_RECORDS_PATH)
        except OSError as e:
            self._send_json(200, {"ok": False, "message": f"写回 shots.jsonl 失败: {e}"})
            return
        self._send_json(200, {"ok": True, "ts": want})

    def _api_practice_delete(self, body):
        """POST /api/practice/delete {ts} —— 删一条已标注记录（连同其证据帧）。"""
        want = str(body.get("ts", "") or "")
        if not want:
            self._send_json(400, {"ok": False, "message": "缺少 ts"})
            return
        recs = _practice_records()
        kept, removed = [], None
        for r in recs:
            if removed is None and r.get("ts") == want:
                removed = r
            else:
                kept.append(r)
        if removed is None:
            self._send_json(200, {"ok": False, "message": f"没有 ts={want} 的记录"})
            return
        _practice_unlink_files(removed)
        try:
            tmp = PRACTICE_RECORDS_PATH + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                for r in kept:
                    f.write(json.dumps(r, ensure_ascii=False) + "\n")
            os.replace(tmp, PRACTICE_RECORDS_PATH)
        except OSError as e:
            self._send_json(200, {"ok": False, "message": f"写回 shots.jsonl 失败: {e}"})
            return
        self._send_json(200, {"ok": True, "message": "已删除", "ts": want})

    def _api_practice_mark(self, body):
        """POST /api/practice/mark {x, y, run?, time?} —— 标注一发。
        流程: 定位发次（给了 run/time 用之, 否则活动 run 最新一发）→ 抓主摄一帧
        （此刻激光应已点亮, 光斑=实际落点）→ 跑靶标检测 → 命中靶 = 离点击最近那颗,
        误差 = 点击 − 靶心 → 打点帧/标注帧/定格帧全部拷进 JiGuangLogs/frames/,
        记录追加 shots.jsonl。
        ⚠️ 与 /api/parallax/capture 不同, 检测失败**不拒绝**（打点本身就是数据,
        检测只是辅助算误差）; 也**不做**「点击必须贴近某靶」的门 —— 打歪了正是
        要量的东西, 拿最近的算归属即可。"""
        try:
            x = float(body.get("x")); y = float(body.get("y"))
        except (TypeError, ValueError):
            self._send_json(400, {"ok": False, "message": "缺少点击坐标 x/y"})
            return
        run = str(body.get("run", "") or "")
        want_time = str(body.get("time", "") or "")
        run_dir, shots = self._practice_shots_of(run)
        if not run_dir or not shots:
            self._send_json(200, {"ok": False,
                                  "message": "活动运行里还没有任何 TGTS 发次（先打一发）"})
            return
        run = os.path.basename(run_dir)
        shot = None
        if want_time:
            shot = next((s for s in shots if s.get("time") == want_time), None)
            if shot is None:
                self._send_json(200, {"ok": False,
                                      "message": f"run {run} 里找不到 {want_time} 那一发"})
                return
        else:
            shot = shots[-1]
        mode = "frozen" if str(body.get("mode", "")) == "frozen" else "live"
        if mode == "frozen":
            # 定格帧标注: 底图 = 该发的 aimfinal（闭环最后看着的那帧; 车与臂都没动,
            # 几何与实时帧一致）。光斑不在这张图里（定格于开激光之前）—— 位置由人
            # 眼看物理靶上的光斑映射过来。好处: ①不碰相机（无抢占、无 ~2s 抓帧）;
            # ②靶心检测与点击落在**同一张图**上, 误差向量内自洽。记录带 mark_mode
            # 供分析端区分打点来源（人眼映射 vs 图像直测）。
            path = logstore.frame_path(run, shot.get("final_frame")) \
                if shot.get("final_frame") else None
            if path is None or not os.path.isfile(path):
                self._send_json(200, {"ok": False,
                                      "message": "这一发没有定格帧（或 run 已清理），请用实时预览标注"})
                return
            frame = cv2.imread(path)
            if frame is None:
                self._send_json(200, {"ok": False, "message": "定格帧读取失败，请用实时预览标注"})
                return
        else:
            frame = _grab_main_frame()
            if frame is None:
                self._send_json(503, {"ok": False, "message": "主摄打开失败（被占用或不存在）"})
                return
        targets = []
        try:
            targets, _small = _detect_frame(frame, _build_target_cfg(),
                                            gks_config.TARGET_PROC_SCALE)
        except Exception as e:                        # noqa: BLE001
            print(f"[WARN] 标靶练习标注帧检测异常: {type(e).__name__}: {e}")
        # 命中靶 = 离点击最近的检出靶（打歪也算它; 没检出靶就没有误差, 只存打点）
        hit, err, err_px = None, None, None
        if targets:
            hit = min(targets, key=lambda t: (t["cx"] - x) ** 2 + (t["cy"] - y) ** 2)
            err = [round(x - hit["cx"], 1), round(y - hit["cy"], 1)]
            err_px = round(((x - hit["cx"]) ** 2 + (y - hit["cy"]) ** 2) ** 0.5, 1)
        # ---- 落盘: JiGuangLogs/frames/<ts>_*.jpg（run 目录会被清理, 证据拷出来才稳）----
        ts = time.strftime("%Y%m%d_%H%M%S")
        files = {}
        try:
            os.makedirs(PRACTICE_FRAMES_DIR, exist_ok=True)
            files["dot"] = f"{ts}_dot.jpg"
            cv2.imwrite(os.path.join(PRACTICE_FRAMES_DIR, files["dot"]), frame)
            ann = _annotate(frame, targets, extra="practice") if targets else frame.copy()
            cv2.circle(ann, (int(x), int(y)), 14, (245, 194, 17), 2)
            cv2.line(ann, (int(x) - 20, int(y)), (int(x) + 20, int(y)), (245, 194, 17), 2)
            cv2.line(ann, (int(x), int(y) - 20), (int(x), int(y) + 20), (245, 194, 17), 2)
            files["dotann"] = f"{ts}_dotann.jpg"
            cv2.imwrite(os.path.join(PRACTICE_FRAMES_DIR, files["dotann"]), ann)
            for key, fname in (("final", shot.get("final_frame")),
                               ("finalann", (shot.get("final_frame") or "").replace("aimfinal", "aimann"))):
                if not fname:
                    continue
                src = logstore.frame_path(run, fname)
                if src and os.path.isfile(src):
                    dst = f"{ts}_{key}.jpg"
                    shutil.copyfile(src, os.path.join(PRACTICE_FRAMES_DIR, dst))
                    files[key] = dst
            # ---- 过程帧序列拷贝（当时开着 ARCHIVE 开关才有）: 该发时间窗内落盘的
            #      aimseq_*。run 目录会被清理, 拷进 JiGuangLogs 才是稳的证据 —— 与
            #      过程 log 固化同一个理由。窗界 = cfg 行时刻 ~ 结束行时刻, 各放宽 3s
            #      （seq 帧按 1Hz 落盘, 尾帧可能比结束行晚 ~1s; 起点侧锁定期也先于首帧）。
            files["seq"] = self._practice_copy_seq(run, shot, ts)
        except Exception as e:                        # noqa: BLE001
            print(f"[WARN] 标靶练习落盘失败: {e}")
        rec = {
            "ts": time.strftime("%Y-%m-%d %H:%M:%S"),
            "run": run, "shot_time": shot.get("time"),
            "cond": _practice_cond_load(),
            "outcome": shot.get("outcome"), "n_ms": shot.get("n_ms"),
            "dx": shot.get("dx"), "dy": shot.get("dy"),
            "trX": shot.get("trX"), "trY": shot.get("trY"),
            "ch_mm": shot.get("ch_mm"), "r_px": shot.get("r_px"),
            "final_state": shot.get("final_state"),
            "mark_mode": mode,                      # live=实时帧直测光斑 / frozen=定格帧上人眼映射
            # 该发的**完整过程 log**（cfg→下一发之间的原始行, 含 TGTS 逐拍串/换挡/
            # 流状态/定格帧行）。标注时从 run 的 log.txt 固化进记录 —— run 目录会被
            # 「日志」页清理, 清掉后这就是追溯那一发全过程的唯一副本。
            "log": (shot.get("lines") or [])[:800],
            "click": [round(x, 1), round(y, 1)],
            "targets": [{"color": int(t["color"]), "cx": round(float(t["cx"]), 1),
                         "cy": round(float(t["cy"]), 1), "r": round(float(t["r"]), 1)}
                        for t in targets],
            "hit": ({"color": int(hit["color"]), "cx": round(float(hit["cx"]), 1),
                     "cy": round(float(hit["cy"]), 1)} if hit else None),
            "err": err, "err_px": err_px,
            "files": files,
        }
        # ---- 写记录: **同一发只留一条**（键 = run + cfg 行时间）。
        # 重复点标注 = 对该发实际落点的**修正**, 覆盖旧记录并清掉旧证据帧 ——
        # 追加式会在手抖/微调时刷出一堆近重复行, 攒数据没法用（真踩过）。
        # 顺带把历史上已攒下的同键重复行一并收拢成本次这一条。
        try:
            os.makedirs(PRACTICE_DIR, exist_ok=True)
            kept, replaced = [], 0
            for r in _practice_records():
                if r.get("run") == run and r.get("shot_time") == shot.get("time"):
                    replaced += 1
                    _practice_unlink_files(r)
                else:
                    kept.append(r)
            kept.append(rec)
            tmp = PRACTICE_RECORDS_PATH + ".tmp"
            with open(tmp, "w", encoding="utf-8") as f:
                for r in kept:
                    f.write(json.dumps(r, ensure_ascii=False) + "\n")
            os.replace(tmp, PRACTICE_RECORDS_PATH)
        except OSError as e:
            self._send_json(200, {"ok": False, "message": f"写 shots.jsonl 失败: {e}"})
            return
        self._send_json(200, {"ok": True, "record": rec, "replaced": replaced})

    def _api_log(self):
        qs = parse_qs(urlparse(self.path).query)
        unit = qs.get("unit", [GKS_SERVICE])[0]
        if unit not in (GKS_SERVICE, WEB_SERVICE):
            unit = GKS_SERVICE
        self._send_text(console.get_log(unit))

    def _api_log_stream(self):
        """SSE 实时日志流：main.py 日志文件 / journalctl -f / 模拟器帧日志，终端同步。

        tail=首次回显的历史行数（默认 200；0 = 不回显历史）。前端「清屏」后重连会带
        tail=0 —— 否则切换日志类型/重连会把刚清掉的历史又刷回来（假清屏）。
        """
        qs = parse_qs(urlparse(self.path).query)
        unit = qs.get("unit", [GKS_SERVICE])[0]
        try:
            tail = int(qs.get("tail", ["200"])[0])
        except ValueError:
            tail = 200
        tail = max(0, min(tail, 2000))
        if unit == "mainpy":
            self._stream_mainpy(tail)
            return
        if unit == "sim":
            self._stream_sim(tail)
            return
        if unit not in (GKS_SERVICE, WEB_SERVICE):
            unit = GKS_SERVICE
        self._stream_journal(unit, tail)

    def _stream_journal(self, unit, tail=200):
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        proc = subprocess.Popen(
            ["journalctl", "-u", unit, "-n", str(tail), "-f", "--no-pager", "--output=cat"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1)
        try:
            for line in iter(proc.stdout.readline, ""):
                line = line.rstrip("\r\n")
                self.wfile.write(f"data: {line}\n\n".encode("utf-8"))
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass  # 客户端断开
        finally:
            try:
                proc.terminate()
            except Exception:
                pass
            try:
                proc.wait(timeout=3)   # 2026-10: 不 wait 会留僵尸（反复切换日志类型时堆积）
            except Exception:
                try:
                    proc.kill()
                except Exception:
                    pass

    def _sse_line(self, line):
        self.wfile.write(f"data: {line.rstrip(chr(10) + chr(13))}\n\n".encode("utf-8"))
        self.wfile.flush()

    def _stream_sim(self, tail=200):
        """SSE 流：模拟器帧交换日志（rx/tx/sys）。

        tail=首次回显的**行数**（0 = 不回显, 清屏后重连用）。2026-10 修语义: 旧版
        `sent = 0 if tail > 0 else len(...)` —— 只要 tail>0 就从第 0 行**全量**重放
        （≤500 行）, 与「回显最近 N 行」不符; 现在真的只回显末尾 tail 行。
        """
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        buf0 = SIMULATOR.recent_log()
        sent = max(0, len(buf0) - max(0, tail)) if tail > 0 else len(buf0)
        try:
            while True:
                log = SIMULATOR.recent_log()
                new = log[sent:]
                for ts, direction, text in new:
                    arrow = "← " if direction == "tx" else ("→ " if direction == "rx" else "─ ")
                    self._sse_line(f"[{ts}] {arrow}{text}")
                sent = len(log)
                time.sleep(0.3)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass  # 客户端断开

    def _stream_mainpy(self, tail=200):
        """跟随 main.py 最新日志文件（logs/*_log.txt），重启换新文件时自动切换。

        tail=首次回显的行数；0 = 直接跳到文件末尾只跟进新行（清屏后重连用）。
        """
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        current = None
        pos = 0
        first = True
        try:
            while True:
                newest = _newest_mainpy_log()
                if newest is not None and newest != current:
                    current = newest
                    pos = 0
                    first = True
                if current is not None:
                    try:
                        with open(current, "r", encoding="utf-8", errors="replace") as f:
                            if first:
                                # 首次连接 / 换文件：回显最近 tail 行，再跟进。
                                # 2026-10: 只 seek 到文件尾部约 tail*300B 处取行 ——
                                # 旧版 readlines() 整读文件（可达数 MB）再切片, 重连
                                # 频繁时白耗内存/CPU。
                                if tail > 0:
                                    f.seek(0, os.SEEK_END)
                                    size = f.tell()
                                    f.seek(max(0, size - tail * 300))
                                    lines = f.read().splitlines()
                                    if size > tail * 300 and lines:
                                        lines = lines[1:]   # 起点可能落在半行中间, 丢首行
                                    for line in lines[-tail:]:
                                        self._sse_line(line)
                                else:
                                    f.seek(0, os.SEEK_END)   # 只跟进新行
                                pos = f.tell()
                                first = False
                            else:
                                # 文件变小 = 被「真清屏」截断了（logstore 的标志文件
                                # 机制）: 从头跟上, 否则旧 pos 悬在 EOF 之外会漏掉
                                # 截断后重写的全部新内容（另一个已连接的悬浮窗就靠它）。
                                try:
                                    if os.path.getsize(current) < pos:
                                        self._sse_line("--- 日志已被清空 ---")
                                        pos = 0
                                except OSError:
                                    pass
                                f.seek(pos)
                                for line in f:
                                    self._sse_line(line)
                                pos = f.tell()
                    except Exception:
                        pass
                time.sleep(0.3)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass  # 客户端断开

    # ---- 日志：真清屏 / 回看 / 帧图 / 清理（2026-10 加, 见 logstore.py）----

    def _api_log_clear(self, body):
        """POST /api/log/clear {unit} —— **真清屏**：显示与文件同步清空。

        · mainpy: 写 logs/.clear_flag → 活动 main.py 的轮询线程消费并**自己**截断
          log.txt（外部截断会让写者 fd 偏移悬空、产生稀疏空洞, 必须由写者来）。
          2.5s 内没被消费（main.py 没跑 / 是旧版）→ 若 pid 不活着就直接截断文件
          （没有活写者, 安全）; pid 活着则是旧版 main.py, 提示重启 gks 服务。
        · sim: 清模拟器帧日志缓冲（服务端真清, 重连不再回放）。
        · journalctl 类（gks/gks-web）: 不支持服务端清空, 维持前端的 tail=0 行为。
        """
        unit = str(body.get("unit", "mainpy"))
        if unit == "mainpy":
            if not logstore.request_clear():
                self._send_json(200, {"ok": False, "message": "清屏标志写入失败（logs/ 可写?）"})
                return
            for _ in range(25):          # 等活动 main.py 消费（轮询周期 0.5s, 给 2.5s）
                if not logstore.clear_requested():
                    self._send_json(200, {"ok": True, "message": "已清空（文件与显示同步）"})
                    return
                time.sleep(0.1)
            # 标志没被消费 = main.py 没跑, 或者跑的是旧版本（不认这个标志）。
            # 旧版本正开着日志文件写 —— 外部截断会让它的 fd 偏移悬空、写出空洞文件,
            # 宁可不清也不干这种事, 提示重启服务后再清。
            if console.is_main_py_running():
                self._send_json(200, {"ok": False,
                                      "message": "main.py 是旧版本（不响应清屏），重启 gks 服务或 main.py 后可用"})
                return
            pid, _run = logstore.read_pid_file()
            pid_alive = False
            try:
                if pid:
                    os.kill(pid, 0)
                    pid_alive = True
            except OSError:
                pid_alive = False
            if pid_alive:
                self._send_json(200, {"ok": False,
                                      "message": "main.py 是旧版本（不响应清屏），重启 gks 服务后可用"})
                return
            # main.py 没在跑: 没有活写者, 直接截断当前日志文件
            log_path = _newest_mainpy_log()
            if log_path:
                try:
                    with open(log_path, "w", encoding="utf-8"):
                        pass
                    self._send_json(200, {"ok": True, "message": "已清空文件（main.py 未运行）"})
                except OSError as e:
                    self._send_json(200, {"ok": False, "message": f"清空失败: {e}"})
            else:
                self._send_json(200, {"ok": True, "message": "当前没有日志文件"})
            return
        if unit == "sim":
            SIMULATOR.clear_log()
            self._send_json(200, {"ok": True, "message": "模拟器日志已清空"})
            return
        self._send_json(200, {"ok": False,
                              "message": f"该日志类型（{unit}）不支持服务端清空"})

    def _api_logs_list(self):
        """GET /api/logs/list —— 回看列表：全部运行（新→旧）, 含旧式平铺日志。"""
        runs = logstore.list_runs()
        out = []
        for r in runs:
            out.append({
                "name": r["name"],
                "time": time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(r["mtime"])),
                "mtime": r["mtime"],
                "size_mb": round(r["size_bytes"] / 1048576.0, 2),
                "frames": r["frames"],
                "active": r["active"],
                "is_run_dir": r["is_run_dir"],
            })
        total = sum(r["size_bytes"] for r in runs)
        self._send_json(200, {"runs": out,
                              "total_mb": round(total / 1048576.0, 2),
                              "logs_dir": logstore.LOG_DIR})

    def _api_logs_read(self):
        """GET /api/logs/read?run= —— 读一次运行的 log.txt 尾部（≤256KB, 防撑爆浏览器）。"""
        qs = parse_qs(urlparse(self.path).query)
        run = qs.get("run", [""])[0]
        data = logstore.read_log(run)
        if data is None:
            self._send_json(404, {"ok": False, "message": f"没有这个运行的日志: {run}"})
            return
        self._send_json(200, {"ok": True, "text": data["text"],
                              "truncated": data["truncated"],
                              "total_bytes": data["total_bytes"], "run": run})

    def _api_logs_frames(self):
        """GET /api/logs/frames?run= —— 列出该运行归档的全部任务帧（时间序）。"""
        qs = parse_qs(urlparse(self.path).query)
        run = qs.get("run", [""])[0]
        frames = logstore.frames_of(run)
        if frames is None:
            self._send_json(404, {"ok": False, "message": f"没有这个运行: {run}"})
            return
        for f in frames:
            f["time"] = time.strftime("%H:%M:%S", time.localtime(f["mtime"]))
        self._send_json(200, {"ok": True, "run": run, "frames": frames})

    def _api_logs_frame(self):
        """GET /api/logs/frame?run=&file= —— 回看一张归档帧（路径经白名单校验）。"""
        qs = parse_qs(urlparse(self.path).query)
        run = qs.get("run", [""])[0]
        fname = qs.get("file", [""])[0]
        path = logstore.frame_path(run, fname)
        if path is None:
            self.send_error(404, "not found")
            return
        try:
            with open(path, "rb") as f:
                data = f.read()
        except OSError:
            self.send_error(404, "not found")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(data)

    def _api_logs_validate(self, body):
        """POST /api/logs/validate {run, file} —— 帧离线验证（日志页帧右键「验证」）。

        把归档帧喂进与在线**同一套**的单帧检测管线（只跑纯函数件, 不碰相机/串口/
        租约）, 回标注后的 JPEG（data URL）+ 检测摘要, 前端弹窗展示。
        输入帧优先取同序号配对的原始帧（<key>_before_<n> / aimann→aimfinal,
        见 _vld_pick_input）—— 验证要喂原图, 旧标注框会干扰检测。
        """
        run = str(body.get("run", ""))
        fname = str(body.get("file", ""))
        algo, inf, note, err = _vld_pick_input(run, fname)
        if algo is None:
            self._send_json(200, {"ok": False, "message": err})
            return
        path = logstore.frame_path(run, inf)
        if path is None:
            self._send_json(200, {"ok": False, "message": "没有这帧: %s / %s" % (run, inf)})
            return
        img = cv2.imread(path)
        if img is None:
            self._send_json(200, {"ok": False, "message": "图片读不出（损坏?）: " + inf})
            return
        t0 = time.time()
        try:
            ann, info = _vld_run_algo(algo, img)
        except Exception as e:                         # noqa: BLE001
            import traceback
            traceback.print_exc()
            self._send_json(200, {"ok": False, "message": "验证算法异常: %s: %s"
                                  % (type(e).__name__, e)})
            return
        ms = int((time.time() - t0) * 1000)
        if ann is None:
            self._send_json(200, {"ok": True, "algo": algo, "input_file": inf,
                                  "note": note or "", "elapsed_ms": ms, "image": None,
                                  "info_ok": False, "info_text": info.get("text", "")})
            return
        okenc, buf = cv2.imencode(".jpg", ann, [int(cv2.IMWRITE_JPEG_QUALITY), 85])
        if not okenc:
            self._send_json(200, {"ok": False, "message": "结果图 JPEG 编码失败"})
            return
        self._send_json(200, {
            "ok": True, "algo": algo, "run": run, "file": fname, "input_file": inf,
            "note": note or "", "elapsed_ms": ms,
            "info_ok": bool(info.get("ok")), "info_text": info.get("text", ""),
            "image": "data:image/jpeg;base64,"
                     + base64.b64encode(buf.tobytes()).decode("ascii"),
        })

    def _api_logs_clean(self, body):
        """POST /api/logs/clean {scope: today|week|all} —— 控制台日志清理。

        活动运行（main.pid 的 pid 活着）一律保留: 正在写的文件删了, main.py 的 fd
        还攥着, 后续日志会写进孤儿 inode 里丢掉。
        """
        scope = str(body.get("scope", ""))
        result = logstore.cleanup(scope)
        if "error" in result:
            self._send_json(200, {"ok": False, "message": result["error"]})
            return
        kept = ", ".join(result.get("kept", [])) or "无"
        self._send_json(200, {
            "ok": True,
            "message": f"已清理 {result['deleted']} 项, 释放 "
                       f"{result['freed_bytes'] / 1048576.0:.1f} MB"
                       + (f"（保留活动运行: {kept}）" if result.get("kept") else ""),
        })

    def _api_logs_delete(self, body):
        """POST /api/logs/delete {run: <name>} —— 删除一次运行（活动运行拒绝）。

        与 cleanup 同一条纪律: 正在写的那次留着, 否则 main.py 的 fd 会写进孤儿 inode。
        """
        run = str(body.get("run", ""))
        result = logstore.delete_run(run)
        if not result.get("ok"):
            self._send_json(200, {"ok": False, "message": result.get("error", "删除失败")})
            return
        self._send_json(200, {
            "ok": True,
            "message": f"已删除 {run}（释放 {result['freed_bytes'] / 1048576.0:.1f} MB）",
        })

    # ---- 模拟器 API ----
    def _api_sim_status(self):
        self._send_json(200, {
            "enabled": SIMULATOR.is_enabled(),
            "client_connected": SIMULATOR.client_connected(),
            "host": SIM_HOST, "port": SIM_PORT,
            "commands": COMMANDS,
            "hostage": SIMULATOR.last_hostage(),   # 最近一帧人质结果（网页显示顺序/坐标）
            "color": SIMULATOR.last_color(),       # 最近一帧色球结果（同上）
            "target": SIMULATOR.last_target(),     # 最近一帧靶标结果（同上，含半径）
            "rect": SIMULATOR.last_rect(),         # 最近一帧矩形地板结果（同上）
            "road": SIMULATOR.last_road(),         # 最近一帧看路结果（同上，0x08）
            "roadyaw": SIMULATOR.last_road_yaw(),  # 最近一帧看路校准结果（同上，0x09）
            "stream": SIMULATOR.last_stream(),     # 最近一帧靶标流帧（协议 §6f，命令 7/8）
        })

    def _api_sim_log(self):
        self._send_json(200, {"log": SIMULATOR.recent_log()})

    def _api_sim_clear(self):
        SIMULATOR.clear_log()
        self._send_json(200, {"ok": True})

    def _api_sim_command(self, body):
        key = str(body.get("key", "")).strip()
        if not key:
            self._send_json(400, {"ok": False, "message": "指令为空"})
            return
        ok = SIMULATOR.send_command(key)
        self._send_json(200, {"ok": ok, "message": "已发送" if ok else "无客户端连接"})

    # ---- 摄像头 / 扫码（复用 camera_web.py 逻辑）----
    def _handle_scan(self, method):
        """就地跑二维码识别，返回 JSON + 标注帧（只给页面看，不回传单片机）。

        方法名 = **算法 + 相机**，与单片机的命令码一一对应（见 _SCAN_METHODS）：

            opencv / pyzbar          → 扫码副摄（只有页面用，没有对应命令码）
            wechat                   → 扫码副摄，对应命令 "1"
            diff                     → 扫码副摄 + 自研难度算法，对应命令 "E"
            wechat_main              → 主摄，对应命令 "D"
            diff_main                → 主摄 + 自研难度算法，对应命令 "F"

        ⚠️ 这几条走的**就是**单片机那条路（同一个 run_qr_*，见
          GKS_Competition/tasks/_qr.py）—— 台架上用它**预演**命令会看到什么：
          同一条解码链、同一套预处理参数。结果帧在 Pi 端是共用的 §6a，
          所以"页面看到能识别的码"就等于"单片机发那条命令能拿到"。
        ⚠️ 必须停**它自己要用的那台**相机的推流（方法名以 `_main` 结尾 = 主摄），
          停错了设备仍被推流线程占着 → open_camera 直接失败。
        ⚠️ 页面是**同一台相机**下做 A/B 的唯一入口：`wechat` vs `diff`（副摄）、
          `wechat_main` vs `diff_main`（主摄）。跨相机的对比说明不了算法好坏。
        """
        entry = _SCAN_METHODS.get(method)
        if entry is None:
            self._send_json(400, {"success": False, "reason": f"未知方法: {method}"})
            return
        fn, _use_main = entry
        if not _scan_begin():
            self._send_json(200, {"success": False, "reason": _scan_busy_json_reason()})
            return
        try:
            _stop_stream("main" if method.endswith("_main") else "qr")
            result = fn(abort_event=_scan_abort)
            _archive_scan_frame("qr_" + method, result, "qr_last.jpg")
            if result.get("aborted"):
                self._send_json(200, {"aborted": True, "method": result["method"]})
                return
            image_b64 = None
            frame = result.get("frame")
            if frame is not None:
                ok, buf = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    image_b64 = base64.b64encode(buf.tobytes()).decode("ascii")
            self._send_json(200, {
                "success": result["success"], "code": result["code"],
                "method": result["method"], "reason": result["reason"],
                "image": image_b64,
            })
        finally:
            _scan_lock.release()

    def _handle_hostage(self):
        """就地跑人质形状识别（主摄像头 YOLO），返回 JSON + 标注帧。

        与 Pi 端 camera_web.py::_handle_hostage 同构：仅页面显示，不回传单片机
        （串口链路由单片机发指令 '4' 触发 main.py::handler._cmd_hostage 完成）。
        共用 _scan_lock/_scan_abort，与扫码任务互斥、可互相打断。
        """
        if not _scan_begin():
            self._send_json(200, {"success": False, "reason": _scan_busy_json_reason()})
            return
        try:
            _stop_stream("main")     # 识别前停主摄推流，释放设备
            result = run_hostage_detect(abort_event=_scan_abort)
            _archive_scan_frame("hostage", result, "hostage_last.jpg")
            if result.get("aborted"):
                self._send_json(200, {"aborted": True})
                return
            image_b64 = None
            frame = result.get("frame")
            if frame is not None:
                ok, buf = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    image_b64 = base64.b64encode(buf.tobytes()).decode("ascii")
            self._send_json(200, {
                "success": result["success"],
                "reason": result["reason"],
                "cam_is_pi": bool(result.get("cam_is_pi", False)),
                "used_pi": bool(use_pi),
                "targets": [{k: t[k] for k in ("shape", "shape_name", "cx", "cy", "conf")}
                            for t in result.get("targets", [])],
                "image": image_b64,
            })
        finally:
            _scan_lock.release()

    def _handle_color(self):
        """就地跑色球顺序识别（主摄像头 HSV），返回 JSON + 标注帧。

        与 _handle_hostage 同构：仅页面显示，不回传单片机
        （串口链路由单片机发指令 '5' 触发 main.py::handler._cmd_color 完成）。
        共用 _scan_lock/_scan_abort，与扫码/人质任务互斥、可互相打断。
        """
        if not _scan_begin():
            self._send_json(200, {"success": False, "reason": _scan_busy_json_reason()})
            return
        try:
            _stop_stream("main")     # 识别前停主摄推流，释放设备
            result = run_color_detect(abort_event=_scan_abort)
            _archive_scan_frame("color", result, "color_last.jpg")
            if result.get("aborted"):
                self._send_json(200, {"aborted": True})
                return
            image_b64 = None
            frame = result.get("frame")
            if frame is not None:
                ok, buf = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    image_b64 = base64.b64encode(buf.tobytes()).decode("ascii")
            self._send_json(200, {
                "success": result["success"],
                "reason": result["reason"],
                "targets": [{k: t.get(k) for k in ("color", "color_name", "cx", "cy", "votes")}
                            for t in result.get("targets", [])],
                "image": image_b64,
            })
        finally:
            _scan_lock.release()

    def _handle_rect(self):
        """就地跑色球区**矩形地板**识别（主摄像头，黑胶带边框），返回 JSON + 标注帧。

        与 _handle_color 同构：仅页面显示，**不回传单片机**（本任务目前没有对应的
        串口指令 —— 单片机侧还没有消费"板心"的步骤；将来要回传就照 _cmd_color 加）。
        共用 _scan_lock/_scan_abort，与扫码/人质/色球/靶标任务互斥、可互相打断。

        返回:
          center   = [cx, cy] 板心（= 四边形对角线交点。世界矩形中心的射影，
                     透视下**不等于**四角均值；缺球时靠它做横向基准）
          corners  = 四个角（P0 左上起顺时针）
          estimated= True 表示只有三边、缺的那条按长宽比估出来的（偏差 ~20px 量级，
                     标定目标值时**别和四边精确的值混用**，见 config.RECT_HW_RATIO 注释）
          n_sides / votes / spread = 用了几条边 / 成功帧数 / 中心跨帧散布
        """
        if not _scan_begin():
            self._send_json(200, {"success": False, "reason": _scan_busy_json_reason()})
            return
        try:
            _stop_stream("main")     # 识别前停主摄推流，释放设备
            result = run_rect_detect(abort_event=_scan_abort)
            _archive_scan_frame("rect", result, "rect_last.jpg")
            if result.get("aborted"):
                self._send_json(200, {"aborted": True})
                return
            image_b64 = None
            frame = result.get("frame")
            if frame is not None:
                ok, buf = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    image_b64 = base64.b64encode(buf.tobytes()).decode("ascii")
            center = ([int(round(result["center"][0])), int(round(result["center"][1]))]
                      if result.get("center") else None)
            corners = ([[int(round(x)), int(round(y))] for x, y in result["corners"]]
                       if result.get("corners") else None)
            self._send_json(200, {
                "success": result["success"],
                "reason": result["reason"],
                "center": center,
                "corners": corners,
                "estimated": bool(result.get("estimated")),
                "n_sides": result.get("n_sides", 0),
                "path": result.get("path", ""),
                "votes": result.get("votes", 0),
                # 稳定性三件套: 剔了几个离群帧 / 内点散布 / **各帧原始中心**
                # （页面把原始值也列出来 —— 稳不稳一眼看得见，别只给一个数）
                "inliers": result.get("inliers", 0),
                "dropped": result.get("dropped", 0),
                "spread": round(float(result.get("spread", 0.0)), 1),
                "frames": [[round(p[0], 1), round(p[1], 1)]
                           for p in result.get("frames", [])],
                "image": image_b64,
            })
        except Exception as e:                      # noqa: BLE001
            # ⚠️ 兜住任何异常**也要回 JSON**：不回的话连接被直接关掉，前端只看到
            # "请求失败/一直转圈"，看不出是服务端炸了（GKS_Web/CLAUDE.md 里那条纪律）。
            import traceback
            traceback.print_exc()
            self._send_json(500, {"success": False, "reason": f"服务端异常: {e}",
                                  "center": None, "corners": None, "estimated": False,
                                  "votes": 0, "inliers": 0, "dropped": 0,
                                  "spread": 0, "frames": [], "path": "",
                                  "image": None})
        finally:
            _scan_lock.release()

    def _handle_road(self):
        """就地跑「看路」识别（主摄像头看地面），返回 JSON + 标注帧。

        与 _handle_rect 同构：这是**标定/排障用的就地预览**（不回传单片机）；
        单片机上那条链是另一条路（指令 "G" → handler._cmd_road → 回 0x08 帧），
        两者调的是**同一个** tasks/_road.py，所以这里看到的数就是车上那个数。

        返回:
          dx    = **Δx = 黄色区左边缘的 x**（全分辨率 px；用户口径，原点 = 画面左边）
          edge  = [ex, ey] 左边缘那一点的像素坐标（画图/排障用）
          area  = 黄区面积[px²]（太小说明没抠住，flags 里也会打"面积勉强"）
          flags = 与 0x08 帧同一套位（bit0 边缘有效 / bit1 贴画面左边(饱和) /
                  bit2 贴上下边 / bit3 面积勉强）
          votes / inliers / dropped / spread = 稳定性：采了几帧 / 内点 / 剔了几个 / 散布
        """
        if not _scan_begin():
            self._send_json(200, {"success": False, "reason": _scan_busy_json_reason()})
            return
        try:
            _stop_stream("main")     # 识别前停主摄推流，释放设备
            result = run_road_detect(abort_event=_scan_abort)
            _archive_scan_frame("road", result, "road_last.jpg")
            if result.get("aborted"):
                self._send_json(200, {"aborted": True})
                return
            image_b64 = None
            frame = result.get("frame")
            if frame is not None:
                ok, buf = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    image_b64 = base64.b64encode(buf.tobytes()).decode("ascii")

            def _pt(p):
                return ([int(round(p[0])), int(round(p[1]))] if p else None)

            self._send_json(200, {
                "success": result["success"],
                "reason": result["reason"],
                "dx": round(float(result.get("dx", 0.0)), 1),
                "edge": _pt(result.get("edge")),
                "area": round(float(result.get("area", 0.0))),
                "flags": int(result.get("flags", 0)),
                "votes": result.get("votes", 0),
                "inliers": result.get("inliers", 0),
                "dropped": result.get("dropped", 0),
                "spread": round(float(result.get("spread", 0.0)), 1),
                # 各帧原始 Δx：稳不稳一眼看得见，别只给一个数
                "frames": [round(float(v), 1) for v in result.get("frames", [])],
                "image": image_b64,
            })
        except Exception as e:                      # noqa: BLE001
            # ⚠️ 兜住任何异常**也要回 JSON**：不回的话连接被直接关掉，前端只看到
            # "请求失败/一直转圈"，看不出是服务端炸了（GKS_Web/CLAUDE.md 里那条纪律）。
            import traceback
            traceback.print_exc()
            self._send_json(500, {"success": False, "reason": f"服务端异常: {e}",
                                  "dx": 0.0, "edge": None, "area": 0, "flags": 0,
                                  "votes": 0, "inliers": 0, "dropped": 0, "spread": 0,
                                  "frames": [], "image": None})
        finally:
            _scan_lock.release()

    def _handle_road_yaw(self):
        """就地跑看路校准（主摄像头），返回 JSON + 标注帧。

        与 _handle_road 同构（不走串口：网页 → 本进程直接调 tasks/_road_yaw.py）。
        单片机上那条链是另一条路（指令 "H" → handler._cmd_road_yaw → 回 0x09 帧），
        两者调的是**同一个** tasks/_road_yaw.py，所以这里看到的角就是车上那个数。

        返回:
          ang    = **黄区直边的图像方向角** [°]（竖直=0°、向画面右倒为正、(−90,+90]，
                   竖直 = 0°，向画面右倒为正）—— 车航向的图像观测量（陀螺漂移基准）
          cols   = 拟合内点数
          rms    = 拟合残差 RMS [px]（质量门，越小越直）
          span   = 拟合直段长度 [px]
          flags  = 与 0x09 帧同一套位（bit0 方向角有效 / bit1 直段偏短(勉强)）
          votes / inliers / dropped / spread = 稳定性（° 口径）
          frames = 各帧原始 ang（稳不稳一眼看得见）
        ⚠️ **只测不改**：这条链只读数，不动车、不写航向。
        """
        if not _scan_begin():
            self._send_json(200, {"success": False, "reason": _scan_busy_json_reason()})
            return
        try:
            _stop_stream("main")     # 识别前停主摄推流，释放设备
            result = run_road_yaw_detect(abort_event=_scan_abort)
            _archive_scan_frame("road_yaw", result, "roadyaw_last.jpg")
            if result.get("aborted"):
                self._send_json(200, {"aborted": True})
                return
            image_b64 = None
            frame = result.get("frame")
            if frame is not None:
                ok, buf = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    image_b64 = base64.b64encode(buf.tobytes()).decode("ascii")

            self._send_json(200, {
                "success": result["success"],
                "reason": result["reason"],
                "ang": round(float(result.get("angle", 0.0)), 2),
                "cols": int(result.get("cols", 0)),
                "rms": round(float(result.get("rms", 0.0)), 1),
                "span": round(float(result.get("span", 0.0))),
                "flags": int(result.get("flags", 0)),
                "votes": result.get("votes", 0),
                "inliers": result.get("inliers", 0),
                "dropped": result.get("dropped", 0),
                "spread": round(float(result.get("spread", 0.0)), 2),
                "frames": [round(float(v), 2) for v in result.get("frames", [])],
                "image": image_b64,
            })
        except Exception as e:                      # noqa: BLE001
            # ⚠️ 兜住任何异常**也要回 JSON**（同 _handle_road 那条纪律）。
            import traceback
            traceback.print_exc()
            self._send_json(500, {"success": False, "reason": f"服务端异常: {e}",
                                  "ang": 0.0, "cols": 0, "rms": 0.0, "span": 0,
                                  "flags": 0, "votes": 0, "inliers": 0, "dropped": 0,
                                  "spread": 0.0, "frames": [], "image": None})
        finally:
            _scan_lock.release()

    def _handle_target(self):
        """就地跑靶标识别 + 靶心定位（主摄像头），返回 JSON + 标注帧。

        与 _handle_color 同构：仅页面显示，不回传单片机
        （串口链路由单片机发指令 '6' 触发 main.py::handler._cmd_target 完成）。
        共用 _scan_lock/_scan_abort，与扫码/人质/色球任务互斥、可互相打断。

        ⚠️ 这里是**在 GKS_Web 进程内**跑视觉 —— 它与 gks.service 抢同一个 /dev/camera_main，
        所以进来先 _stop_stream("main") 释放推流。两个进程同时开主摄会 EBUSY。

        `?pi=1`（2026-10-16 加）= 优先用树莓派 CSI 那颗（它固定在底盘上、几何稳），
        打不开/看不到靶时 `run_target_detect` 自己退回主摄 —— 台架上就是靠它读 CSI 的
        靶 cx 去填单片机的 `tc_cx_aim_pi`（瞄准列）。返回里 `cam_is_pi` 是**实际**用的那台。
        ⚠️ CSI 走 libcamera（不是 USB），与主摄不抢设备；**网页那个 pi 预览**占着它时
        `_open_now` 会先请它让位（同进程钩子 `_yield_pi_preview`，2026-10-16），所以
        不再要求"先把预览关掉"。`?pi=1` 进来还会**先等它起好**（见 PICAM_MANUAL_WAIT_S）——
        手动按钮不受单片机时限约束, 冷启动那 ~3s 等得起, 别让它"永远退回主摄"。
        """
        qs = parse_qs(urlparse(self.path).query)
        use_pi = str(qs.get("pi", ["0"])[0]).lower() in ("1", "true", "yes")
        if not _scan_begin():
            self._send_json(200, {"success": False, "reason": _scan_busy_json_reason()})
            return
        try:
            _stop_stream("main")     # 识别前停主摄推流，释放设备（退回主摄那一路要用它）
            if use_pi:
                # ⚠️ 先把 CSI 起好再识别（2026-10-16 加）: 冷启动 ~3s > run_target_detect 留给
                #    CSI 的那 2s（PICAM_TRY_BUDGET_S 是给单片机那条路的, 见 PICAM_MANUAL_WAIT_S
                #    的注释）—— 不等这一下, 这个按钮永远会"退回主摄"。
                #    ⚠️ warm() 只保证"起流线程已发车", 就绪要靠下面的轮询; 整段都看着 _scan_abort,
                #       页面上的 /abort（点别的按钮/取消）随时能打断。
                t_wait = time.time() + PICAM_MANUAL_WAIT_S
                camera_picam.PICAM.warm(1)
                while time.time() < t_wait and not camera_picam.PICAM.ready():
                    if _scan_abort.is_set():
                        break
                    time.sleep(0.1)
            # ⚠️ 手动扫描要显式给预算（2026-10-16 加）: 默认走 config 那套（CSI 只 2s、总 6s）——
            #    满分辨率下 CSI 采一帧就要 0.6~1.8s, 2s 凑不够 TARGET_MIN_VOTES 票, 结果
            #    "每次都退回主摄"。理由与取值见 PICAM_MANUAL_TRY_BUDGET_S / TARGET_MANUAL_TIMEOUT_S。
            result = run_target_detect(timeout=TARGET_MANUAL_TIMEOUT_S, abort_event=_scan_abort,
                                       use_pi=use_pi, pi_budget=PICAM_MANUAL_TRY_BUDGET_S)
            _archive_scan_frame("target", result, "target_last.jpg")
            if result.get("aborted"):
                self._send_json(200, {"aborted": True})
                return
            image_b64 = None
            frame = result.get("frame")
            if frame is not None:
                ok, buf = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    image_b64 = base64.b64encode(buf.tobytes()).decode("ascii")
            self._send_json(200, {
                "success": result["success"],
                "reason": result["reason"],
                # ⚠️ **必须回**（2026-10-16 补）: 前端 `scanTarget(1)` 就是按它显示
                #    「[实际: pi/CSI | 主摄]」的 —— 之前这条漏了 → 页面**永远**显示"主摄",
                #    哪怕这一帧明明是 CSI 的（看着就像"pi 那条路从来没成功过"）。
                "cam_is_pi": bool(result.get("cam_is_pi", False)),
                "used_pi": bool(use_pi),
                # r 是**外圆等效半径**（内边界拟合值已按 Ø55/Ø100 折算），
                # 固件的视差补偿 offset = h·r_px/50 就用它，现场也可拿它粗估距离
                "targets": [{k: t.get(k) for k in ("color", "color_name",
                                                   "cx", "cy", "r", "votes")}
                            for t in result.get("targets", [])],
                "image": image_b64,
            })
        finally:
            _scan_lock.release()

    def _handle_abort(self):
        _scan_abort.set()
        qs = parse_qs(urlparse(self.path).query)
        cam = qs.get("cam", [None])[0]
        # /abort 的语义 = 「把这台相机的一切持有形式让出来」: 推流 **和** 本进程的租约
        # （2026-10 补第二条 —— 只停推流时, 网页进程 adopt 的租约会把相机一直攥着,
        #  main.py 那边 0.3s×3 的重试窗永远赢不了, 症状=拔插 USB 前一直打不开）。
        if cam:
            _stop_stream(cam)
            # ⚠️ cam=pi 要单独收（2026-10-16 加）: CSI 那颗**不在 _streams 里**（它走
            #    libcamera/rpicam-vid, 见 _send_stream_pi），_stop_stream 够不着它 ——
            #    而 main.py 那边的识别任务打开 CSI 失败时正是打这一枪（camera_picam.
            #    _abort_gks_web_pi）。libcamera 独占, 不真收掉那个子进程就谁都开不了。
            if cam == "pi":
                _stop_pi_stream()
                print("[INFO] /abort: 已让出 pi(CSI) 预览")
            elif camera_lease.release_lease_if(cam == "qr"):
                print(f"[INFO] /abort: 已让出本进程的{'扫码' if cam == 'qr' else '主'}相机租约")
        else:
            _stop_all_streams()
            _stop_pi_stream()
            camera_lease.release_lease_if(False)
            camera_lease.release_lease_if(True)
        self._send_json(200, {"ok": True})

    def _api_camera_check(self):
        """检查摄像头能否正常打开并读到帧（仅连接检查）。

        `?cam=main|qr|pi` → **只测这一路**（只让出它自己的推流，不动别的格 ——
            一起停会把别人的预览冻住）；省略 = 三路全测（= 加这个参数之前的行为）。

        ⚠ 测 USB 那两路必须先停掉**它们自己**的推流：V4L2 设备独占，若页面上正开着
        /stream?cam=main，本进程再 open 同一设备会 EBUSY，检查就会误报「打开失败」
        （前端会在检查通过后再重新开预览，见 JS checkCamera）。
        ⚠ 与就地识别共用 _scan_lock（超时 30s）—— 一个正在跑的识别占着相机时,
        不抢锁的检查会对着**自己进程**吃 EBUSY 误报「设备被占用」。
        ⚠ pi(CSI) **不抢这个锁**：它独立于那两个 USB 摄像头、也不进相机租约，识别
        任务占着 USB 相机照样能测，没必要被它堵最多 30s。
        """
        cam = (parse_qs(urlparse(self.path).query).get("cam") or [""])[0]

        if cam == "pi":
            self._send_json(200, {"pi": self._check_picam()})
            return

        if cam in ("main", "qr"):
            if not _scan_begin():
                self._send_json(200, {cam: {"ok": False,
                                            "reason": "有识别任务正在进行，无法检测"}})
                return
            try:
                _stop_stream(cam)         # ⚠ 只停这一路，别动另一格
                time.sleep(0.3)
                self._send_json(200, {cam: self._check_camera(cam == "qr")})
            finally:
                _scan_lock.release()
            return

        if not _scan_begin():
            self._send_json(200, {
                "main": {"ok": False, "reason": "有识别任务正在进行，无法检测"},
                "qr": {"ok": False, "reason": "有识别任务正在进行，无法检测"},
                # pi(CSI) 是独立硬件、不进相机租约 —— 识别任务占着 USB 相机也照样能测
                "pi": self._check_picam(),
            })
            return
        try:
            _stop_all_streams()
            # ⚠ 刻意**不**停 CSI 预览：它跟两个 USB 摄像头是独立硬件，停了对本检查
            #   没帮助，反而制造竞态（前端 <img> 会立刻重连、抢在自检前面）。
            #   预览在跑时 _check_picam() 直接判 OK，见那里。
            time.sleep(0.3)     # 给两个推流线程一点时间退出循环并 release 设备
            self._send_json(200, {
                "main": self._check_camera(False),
                "qr": self._check_camera(True),
                "pi": self._check_picam(),
            })
        finally:
            _scan_lock.release()

    @staticmethod
    def _check_camera(is_qr):
        name = "扫码摄像头" if is_qr else "主摄像头"
        # 重试几次：上一路推流/识别任务刚释放设备时会有一小段 EBUSY 窗口
        cap = None
        for _ in range(5):
            cap = open_camera(is_qr=is_qr)
            if cap is not None:
                break
            time.sleep(0.3)
        if cap is None:
            node = gks_config.CAM_QR_INDEX if is_qr else gks_config.CAM_MAIN_INDEX
            reason = "打开失败（设备被占用）" if os.path.exists(node) \
                     else f"打开失败（{node} 不存在）"
            return {"ok": False, "reason": reason}
        try:
            frame = None
            for _ in range(10):
                frame = CameraManager.read(cap, name, max_retries=3)
                if frame is not None:
                    break
                time.sleep(0.1)
            if frame is None:
                return {"ok": False, "reason": "读不到帧"}
            h, w = frame.shape[:2]
            return {"ok": True, "resolution": f"{w}x{h}"}
        finally:
            CameraManager.release(cap)

    @staticmethod
    def _check_picam():
        """CSI(libcamera) 那颗的自检 —— 与 _check_camera 是**两条不同的路**：
        它不走 open_camera/CameraManager（cv2 打不开 unicam 原始 Bayer 节点），
        交给 camera_picam.probe() 用 rpicam-still 抓一帧。

        ⚠ 预览正在跑时**直接判 OK、不去抓帧**：前端那个 <img> 会在旧流被停掉后
        立刻重连 /stream?cam=pi，抢上来的 rpicam-vid 会让自检的 rpicam-still 撞
        「Pipeline handler in use by another process」—— 报成假故障。
        （libcamera 是独占的，一次只有一路能拿相机。）
        """
        with _pi_lock:
            s = _pi_stream
        if s is not None and s.alive():
            return {"ok": True, "resolution": "%s（预览中）" % s.camera["model"]}
        try:
            ok, info = camera_picam.probe()
        except Exception as e:
            return {"ok": False, "reason": f"自检异常: {e}"}
        return {"ok": True, "resolution": info} if ok else {"ok": False, "reason": info}

    # ---- 主摄实时调参（模拟器 Tab 的「主摄参数 · 实时预览」悬浮窗, 2026-10-30 加）----
    # 只改**当前主摄预览流**的 V4L2 控件（不写 config、不动 main.py）。关预览/重开相机
    # 会按档位重新设满四个控件，所以这些值不会悄悄漏进识别任务。
    # 消费方是 /stream?cam=main 那条推流线程持有的 cap；没有预览流就明确报错，**不**临时
    # 开设备（会和主程序/识别任务抢 /dev/camera_main）。
    @staticmethod
    def _camctl_apply(cap, auto_exposure=None, exposure=None, auto_wb=None, wb_temp=None):
        """按 camera.py 同一套 V4L2 语义设控件并读回。

        ⚠️ 先设模式再设值：自动档下值控件 inactive（驱动忽略），与 _apply_* 一致。
        ⚠️ 读回值以驱动为准：不支持/超量程时 cap.set 返回 False，下面的 get 暴露真值。
        """
        if auto_wb is not None:
            CameraManager._v4l2_set(cap, cv2.CAP_PROP_AUTO_WB, int(auto_wb))
            if int(auto_wb) == 0 and wb_temp is not None:
                CameraManager._v4l2_set(cap, cv2.CAP_PROP_WB_TEMPERATURE, int(wb_temp))
        if auto_exposure is not None:
            CameraManager._v4l2_set(cap, cv2.CAP_PROP_AUTO_EXPOSURE, int(auto_exposure))
            if int(auto_exposure) == 1 and exposure is not None:
                CameraManager._v4l2_set(cap, cv2.CAP_PROP_EXPOSURE, int(exposure))
        return {"auto_wb": int(round(cap.get(cv2.CAP_PROP_AUTO_WB))),
                "wb_temp": int(round(cap.get(cv2.CAP_PROP_WB_TEMPERATURE))),
                "auto_exposure": int(round(cap.get(cv2.CAP_PROP_AUTO_EXPOSURE))),
                "exposure": int(round(cap.get(cv2.CAP_PROP_EXPOSURE)))}

    @staticmethod
    def _camctl_stream(timeout=1.5):
        """等 /stream?cam=main 把 _CameraStream 注册进 _streams（浏览器刚设 img.src
        时接口可能先到）。超时返回 None。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with _streams_lock:
                s = _streams.get("main")
            if s is not None:
                return s
            time.sleep(0.05)
        return None

    def _api_camctl_get(self):
        s = self._camctl_stream()
        if s is None:
            self._send_json(200, {"ok": False,
                                  "message": "主摄预览没有在运行：先在悬浮窗点「打开预览」"})
            return
        try:
            with s.ctrl_lock:
                vals = self._camctl_apply(s.cap)
            self._send_json(200, dict(vals, ok=True))
        except Exception as e:
            self._send_json(200, {"ok": False, "message": "读取失败: %s" % e})

    def _api_camctl_post(self, body):
        s = self._camctl_stream()
        if s is None:
            self._send_json(200, {"ok": False,
                                  "message": "主摄预览没有在运行：先在悬浮窗点「打开预览」"})
            return
        try:
            with s.ctrl_lock:
                prof = body.get("profile")
                if prof:
                    if prof not in ("main", "road", "hostage"):
                        self._send_json(200, {"ok": False,
                                              "message": "未知档位: %s" % prof})
                        return
                    pp = profile_params(prof)
                    (a_e, v_e), (a_w, v_w) = pp["exposure"], pp["wb"]
                    vals = self._camctl_apply(s.cap, auto_exposure=a_e, exposure=v_e,
                                              auto_wb=a_w, wb_temp=v_w)
                else:
                    vals = self._camctl_apply(
                        s.cap,
                        auto_exposure=body.get("auto_exposure"),
                        exposure=body.get("exposure"),
                        auto_wb=body.get("auto_wb"),
                        wb_temp=body.get("wb_temp"))
            self._send_json(200, dict(vals, ok=True))
        except Exception as e:
            self._send_json(200, {"ok": False, "message": "应用失败: %s" % e})

    def _api_qr_frame(self):
        """返回 main.py 扫码任务保存的最后一帧（logs/qr_last.jpg），带 mtime 供前端判新。"""
        path = os.path.join(GKS_COMPETITION_DIR, "logs", "qr_last.jpg")
        try:
            mtime = os.path.getmtime(path)
            with open(path, "rb") as f:
                data = f.read()
        except Exception:
            self.send_error(404, "no frame")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Frame-Mtime", str(mtime))
        self.end_headers()
        self.wfile.write(data)

    def _api_hostage_frame(self):
        """返回 main.py 人质识别任务保存的最后一帧（logs/hostage_last.jpg）。

        与 _api_qr_frame 同构：带 X-Frame-Mtime 供前端判断是否为新一帧。
        该帧已由 Pi 端 _hostage.py::_annotate 画好检测框与从左到右序号。
        """
        path = os.path.join(GKS_COMPETITION_DIR, "logs", "hostage_last.jpg")
        try:
            mtime = os.path.getmtime(path)
            with open(path, "rb") as f:
                data = f.read()
        except Exception:
            self.send_error(404, "no frame")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Frame-Mtime", str(mtime))
        self.end_headers()
        self.wfile.write(data)

    def _api_color_frame(self):
        """返回 main.py 色球识别任务保存的最后一帧（logs/color_last.jpg）。

        与 _api_hostage_frame 同构：带 X-Frame-Mtime 供前端判断是否为新一帧。
        该帧已由 Pi 端 _color.py::_annotate 框选好每个球并标了从左到右序号。
        """
        path = os.path.join(GKS_COMPETITION_DIR, "logs", "color_last.jpg")
        try:
            mtime = os.path.getmtime(path)
            with open(path, "rb") as f:
                data = f.read()
        except Exception:
            self.send_error(404, "no frame")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Frame-Mtime", str(mtime))
        self.end_headers()
        self.wfile.write(data)

    def _api_road_frame(self):
        """返回 main.py 看路任务保存的最后一帧（logs/road_last.jpg）。

        与 _api_target_frame 同构：带 X-Frame-Mtime 供前端判断是否为新一帧。
        该帧已由 Pi 端 _road.py::_annotate 画好两条腿、交点、右切点与 Δ 连线
        （⚠️ 帧上文字是 ASCII —— cv2.putText 画不了中文，写进去就是一排 "?"）。
        """
        path = os.path.join(GKS_COMPETITION_DIR, "logs", "road_last.jpg")
        try:
            mtime = os.path.getmtime(path)
            with open(path, "rb") as f:
                data = f.read()
        except Exception:
            self.send_error(404, "no frame")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Frame-Mtime", str(mtime))
        self.end_headers()
        self.wfile.write(data)

    def _api_road_yaw_frame(self):
        """返回 main.py 看路校准任务保存的最后一帧（logs/roadyaw_last.jpg）。

        与 _api_road_frame 同构：带 X-Frame-Mtime 供前端判断是否为新一帧。
        该帧已由 Pi 端 _road_yaw.py::_annotate 画好拟合直线、ROI 窗与角度文本。
        """
        path = os.path.join(GKS_COMPETITION_DIR, "logs", "roadyaw_last.jpg")
        try:
            mtime = os.path.getmtime(path)
            with open(path, "rb") as f:
                data = f.read()
        except Exception:
            self.send_error(404, "no frame")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Frame-Mtime", str(mtime))
        self.end_headers()
        self.wfile.write(data)

    def _api_target_frame(self):
        """返回 main.py 靶标识别任务保存的最后一帧（logs/target_last.jpg）。

        与 _api_color_frame 同构：带 X-Frame-Mtime 供前端判断是否为新一帧。
        该帧已由 Pi 端 _target.py::_annotate 框选好每颗靶并标了颜色/圆心/半径。
        """
        path = os.path.join(GKS_COMPETITION_DIR, "logs", "target_last.jpg")
        try:
            mtime = os.path.getmtime(path)
            with open(path, "rb") as f:
                data = f.read()
        except Exception:
            self.send_error(404, "no frame")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Frame-Mtime", str(mtime))
        self.end_headers()
        self.wfile.write(data)

    def _api_stream_frame(self):
        """返回靶标**流**跟踪落下的最后一帧（logs/target_stream_last.jpg，协议 §6f）。

        与 _api_target_frame 同构（带 X-Frame-Mtime 供前端判断新帧），只是来源换成了
        命令 "7"/"8" 的流式跟踪 —— 那张图由 Pi 端 _target_stream.py::_write_preview
        按 TARGET_STREAM_LOG_HZ（默认 1Hz）落盘，所以刷新率也是 1Hz 级，不是 20Hz。
        ⚠️ 与 "6" 的 target_last.jpg 是**两个文件**：那条链路的图在流跑过之后不会被覆盖，
        别把两者当成同一个预览（"6" 只在自己跑的时候才更新）。
        """
        path = os.path.join(GKS_COMPETITION_DIR, "logs", "target_stream_last.jpg")
        try:
            mtime = os.path.getmtime(path)
            with open(path, "rb") as f:
                data = f.read()
        except Exception:
            self.send_error(404, "no frame")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-cache")
        self.send_header("X-Frame-Mtime", str(mtime))
        self.end_headers()
        self.wfile.write(data)

    def _send_snapshot(self, cam_key):
        cap = open_camera(is_qr=(cam_key == "qr"))
        if cap is None:
            self.send_error(503, f"{cam_key} camera open failed")
            return
        try:
            frame = None
            for _ in range(10):
                frame = CameraManager.read(cap, cam_key, max_retries=3)
                if frame is not None:
                    break
                time.sleep(0.1)
            if frame is None:
                self.send_error(500, "no frame")
                return
            ok, buf = cv2.imencode('.jpg', frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok:
                self.send_error(500, "encode failed")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(buf)))
            self.end_headers()
            self.wfile.write(buf.tobytes())
        finally:
            CameraManager.release(cap)

    def _send_stream(self, cam_key):
        _stop_stream(cam_key)
        time.sleep(0.3)   # 2026-10: 给旧推流线程一点时间退出循环并 release 设备（同
                          # _api_camera_check 的做法）—— 旧版立刻 open, 快速刷新预览页时
                          # 偶发 EBUSY → 503
        cap = open_camera(is_qr=(cam_key == "qr"))
        if cap is None:
            self.send_error(503, f"{cam_key} camera open failed")
            return
        stream = _CameraStream(cam_key, cap, self.connection)
        with _streams_lock:
            _streams[cam_key] = stream
        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        try:
            blank_since = None
            while not stream.stop.is_set():
                frame = CameraManager.read(cap, cam_key, max_retries=3)
                if frame is None:
                    # 「画面冻住但检测显示正常」有两种成因，这条日志把它们分开：
                    #   * 本行刷出来 = 相机**打开了但读不到帧**（USB 掉线/带宽不足/
                    #     被别的进程攥着），流还在、只是没东西可画；
                    #   * 没有本行却冻住 = 流被**停掉**了（多半是晚到的 /abort，
                    #     见 JS stopScan 的注释）。
                    now = time.time()
                    if blank_since is None:
                        blank_since = now
                    elif now - blank_since > 2.0:
                        print(f"[WARN] /stream?cam={cam_key}: 已 {now - blank_since:.1f}s "
                              f"读不到帧 —— 这个预览会冻住（相机没出图）")
                        blank_since = now
                    time.sleep(0.05)
                    continue
                blank_since = None
                ok, buf = cv2.imencode('.jpg', frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if not ok:
                    continue
                self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(buf.tobytes())
                self.wfile.write(b"\r\n")
                time.sleep(0.03)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass
        finally:
            with _streams_lock:
                if _streams.get(cam_key) is stream:
                    del _streams[cam_key]
            CameraManager.release(cap)

    def _send_stream_pi(self):
        """Pi CSI 摄像头预览：转发 rpicam-vid 出的 MJPEG。

        为什么另起一条路而不是复用 _send_stream：
          * 那颗在 libcamera 管线后面，/dev/videoN(unicam) 是原始 Bayer 节点，
            open_camera/cv2 根本打不开；
          * 它与两个 USB 摄像头**设备独立**，所以没有「先腾设备再开」那套互斥，
            也不必进 _scan_lock。
        ⚠ 失败就 503（前端会显示成「失败」），别静默黑屏 —— 取不到流的原因
          （没插好 / 被别的 rpicam 占用 / 参数不对）都写在 rpicam-vid 的 stderr 里。
        """
        global _pi_stream
        _stop_pi_stream()
        time.sleep(0.3)   # 给上一个 rpicam-vid 退干净（同 _send_stream 的 0.3s）
        # 重试几次：连开两路预览时，上一路 rpicam-vid 可能还没完全释放 libcamera。
        # ⚠ 每次重试都先清相机索引缓存 —— 失败也可能是「索引变了」（相机拔过）。
        stream, last = None, None
        for attempt in range(3):
            try:
                stream = camera_picam.PicamStream()
            except Exception as e:
                last, stream = e, None
            if stream is not None and stream.wait_ready(2.5):
                break
            if stream is not None:
                stream.close()
                stream = None
                last = "rpicam-vid started but produced no frame (camera busy?)"
            camera_picam.forget_camera()
            if attempt < 2:
                time.sleep(0.8)
        if stream is None:
            print(f"[WARN] pi 摄像头预览打开失败: {last}")     # 日志留中文原文
            self.send_error(503, "pi camera open failed: %s" % _ascii(last))
            return
        stream.conn = self.connection
        with _pi_lock:
            _pi_stream = stream

        self.send_response(200)
        self.send_header("Content-Type",
                         f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        seq = -1
        try:
            while not stream.stop.is_set():
                jpg, seq = stream.get_frame(last_seq=seq, timeout=2.0)
                if jpg is None:
                    if not stream.alive():
                        break     # 子进程起来就退了（参数不对等）→ 收工，别死转
                    continue
                self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
            pass
        finally:
            with _pi_lock:
                if _pi_stream is stream:
                    _pi_stream = None
            stream.close()

    # ---- 配置参数（复用 GKS_Competition 的 EDITABLE_CONFIG）----
    # EDITABLE_CONFIG 是 4 元组 (name, type, desc, opts)：第 4 字段是选项表
    # （"0=关|1=开" 这种），前端据此把布尔项画成勾选框、枚举项画成下拉框
    # （与单片机端控制台的调参 Tab 同一套规则，见 loadConfig 的 cfgControl）。
    def _handle_config_get(self):
        items = [{"name": name, "type": typ, "desc": desc, "opts": opt,
                  "value": getattr(gks_config, name)}
                 for name, typ, desc, opt in gks_config.EDITABLE_CONFIG]
        self._send_json(200, items)

    def _handle_config_post(self):
        body = self._read_json()
        editable = {name for name, _t, _d, _o in gks_config.EDITABLE_CONFIG}
        msgs, ok = [], True
        for name, val in body.items():
            if name not in editable:
                ok = False
                msgs.append(f"未知参数 {name}")
                continue
            saved, msg = gks_config.save_override(name, val)
            msgs.append(msg)
            if not saved:
                ok = False
        self._send_json(200, {"ok": ok, "message": "; ".join(msgs) if msgs else "无修改"})

    # ---- 路由 ----
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)
        # ---- 令牌门（2026-10）: 未认证的 GET 一律只给登录页（见 _authed/_serve_login_page）。
        #      healthcheck 类的本机探活不受影响（WEB_KEY 为空 = 全放行）。
        if not self._authed():
            self._serve_login_page()
            return
        if path in ("/", "/index.html"):
            self._send_html()
        elif path == "/api/status":
            self._api_status()
        elif path == "/api/autoqr":
            self._api_autoqr_get()
        elif path == "/api/log":
            self._api_log()
        elif path == "/api/log/stream":
            self._api_log_stream()
        elif path == "/api/camera/check":
            self._api_camera_check()
        elif path == "/api/camctl":
            self._api_camctl_get()
        elif path == "/api/qr/frame":
            self._api_qr_frame()
        elif path == "/api/hostage/frame":
            self._api_hostage_frame()
        elif path == "/api/color/frame":
            self._api_color_frame()
        elif path == "/api/target/frame":
            self._api_target_frame()
        elif path == "/api/roadyaw/frame":
            self._api_road_yaw_frame()
        elif path == "/api/road/frame":
            self._api_road_frame()
        elif path == "/api/stream/frame":
            self._api_stream_frame()
        elif path == "/api/sim/status":
            self._api_sim_status()
        elif path == "/api/sim/log":
            self._api_sim_log()
        elif path == "/stream":
            # cam=pi → CSI(libcamera) 那颗，走另一条取流路径（见 _send_stream_pi）；
            # 其余照旧交给 _cam_key() 的 main/qr（那两个 key 的语义别动, camera_lease 也读）
            if qs.get("cam", ["main"])[0] == "pi":
                self._send_stream_pi()
            else:
                self._send_stream(self._cam_key())
        # /snapshot.jpg 已删（2026-10）: JS 从未调用; 且它不停推流/不抢 _scan_lock,
        # 识别或推流占着相机时直接 EBUSY 503, 是误导排查的死端点。
        elif path == "/scan":
            self._handle_scan(qs.get("method", ["opencv"])[0])
        elif path == "/hostage":
            self._handle_hostage()
        elif path == "/target":
            self._handle_target()
        elif path == "/color":
            self._handle_color()
        elif path == "/rect":
            self._handle_rect()
        elif path == "/road":
            self._handle_road()
        elif path == "/road_yaw":
            self._handle_road_yaw()
        elif path == "/abort":
            self._handle_abort()
        # GET /log 已删（2026-10）: 它返回 log_buffer.recent(), 但本进程从未安装过
        # Writer（那是 main.py 进程的事）→ 恒为空串, 误导排查的死端点。实时日志走
        # /api/log/stream?unit=mainpy（tail 文件）。
        elif path == "/config":
            self._handle_config_get()
        elif path == "/parallax":
            self._send_parallax_page()
        elif path == "/api/parallax/list":
            self._api_parallax_list()
        elif path == "/practice":
            self._send_practice_page()
        elif path == "/api/practice/shots":
            self._api_practice_shots()
        elif path == "/api/practice/records":
            self._api_practice_records()
        elif path == "/api/practice/recordlog":
            self._api_practice_recordlog()
        elif path == "/api/practice/frame":
            self._api_practice_frame()
        elif path == "/api/practice/condition":
            self._api_practice_condition()
        elif path == "/api/wifi/status":
            self._api_wifi_status()
        elif path == "/api/wifi/scan":
            self._api_wifi_scan()
        elif path == "/api/logs/list":
            self._api_logs_list()
        elif path == "/api/logs/read":
            self._api_logs_read()
        elif path == "/api/logs/frames":
            self._api_logs_frames()
        elif path == "/api/logs/frame":
            self._api_logs_frame()
        else:
            self.send_error(404, "not found")

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        # ---- 令牌门（2026-10, POST 比 GET 严）: 只认 ?key= / X-GKS-Key 头, 不认 cookie ——
        #      cookie 会被浏览器跨站自动携带（挡不住 CSRF）, 自定义头跨站发不出。
        if not self._auth_post_ok():
            self._deny_post()
            return
        if path in ("/api/service", "/api/mainpy", "/api/power",
                    "/api/sim/command", "/api/webservice", "/api/fixdevices",
                    "/api/wifi/connect", "/api/wifi/profile",
                    "/api/log/clear", "/api/logs/clean", "/api/logs/delete",
                    "/api/logs/validate",
                    "/api/autoqr", "/api/camctl"):
            body = self._read_json()
            if path == "/api/service":
                self._api_service(body)
            elif path == "/api/wifi/connect":
                self._api_wifi_connect(body)
            elif path == "/api/wifi/profile":
                self._api_wifi_profile(body)
            elif path == "/api/webservice":
                self._api_webservice(body)
            elif path == "/api/fixdevices":
                self._api_fix_devices()
            elif path == "/api/mainpy":
                self._api_mainpy(body)
            elif path == "/api/power":
                self._api_power(body)
            elif path == "/api/log/clear":
                self._api_log_clear(body)
            elif path == "/api/logs/clean":
                self._api_logs_clean(body)
            elif path == "/api/logs/delete":
                self._api_logs_delete(body)
            elif path == "/api/logs/validate":
                self._api_logs_validate(body)
            elif path == "/api/autoqr":
                self._api_autoqr_post(body)
            elif path == "/api/camctl":
                self._api_camctl_post(body)
            else:
                self._api_sim_command(body)
        elif path == "/api/sim/clear":
            self._api_sim_clear()
        elif path == "/api/sim/start":
            SIMULATOR.enable()
            self._send_json(200, {"ok": True, "message": "模拟器已启动"})
        elif path == "/api/sim/stop":
            SIMULATOR.disable()
            self._send_json(200, {"ok": True, "message": "模拟器已停止"})
        elif path == "/config":
            self._handle_config_post()
        elif path in ("/api/parallax/capture", "/api/parallax/delete",
                      "/api/parallax/fit"):
            body = self._read_json() if path != "/api/parallax/fit" else {}
            if path == "/api/parallax/capture":
                self._api_parallax_capture(body)
            elif path == "/api/parallax/delete":
                self._api_parallax_delete(body)
            else:
                self._api_parallax_fit()
        elif path == "/api/practice/mark":
            self._api_practice_mark(self._read_json())
        elif path == "/api/practice/delete":
            self._api_practice_delete(self._read_json())
        elif path == "/api/practice/reccond":
            self._api_practice_reccond(self._read_json())
        elif path == "/api/practice/condition":
            self._api_practice_condition(self._read_json())
        else:
            self.send_error(404, "not found")

    def log_message(self, fmt, *args):
        pass  # 静音访问日志


def start_server(host=WEB_HOST, port=WEB_PORT):
    server = ThreadingHTTPServer((host, port), WebHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[INFO] GKS Web 控制端已启动: http://{host}:{port}/")
    for ip in _local_ips():
        print(f"        局域网: http://{ip}:{port}/")
    print(f"[INFO] 单片机模拟器已启动: {SIM_HOST}:{SIM_PORT}")
    return server


def main():
    print("[INFO] GKS Web 控制端启动中...")
    server = start_server()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INFO] 收到中断，停止服务")
    finally:
        server.shutdown()
        server.server_close()
        SIMULATOR.shutdown()


if __name__ == "__main__":
    main()
