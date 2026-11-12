# -*- coding: utf-8 -*-
"""
巡线标注/验证平台 后端
======================
职责（AI 闭环不在本服务内, 见 AI_闭环训练说明.md）:
  1) 标注: 从视频切帧建任务, 网页画真值线(理想交界线) + 标状态(0直线/1路口弧/2压灰区/
     3压黄区/4不明), 存 annotations.json
  2) 验证: 任务(或新视频)逐帧跑**与工作主副本同一份**的算法(harness.replay),
     把预测点/拟合线/状态标签烧进画面 → 帧序列 + mp4, 网页翻页播放器预览
  3) 比对: 有真值的任务算逐帧误差(预测 vs 真值) → 汇总 + 最差帧清单, 供 AI 定位问题

启动: python server.py  →  http://127.0.0.1:8765
"""
import json
import os
import sys
import io
import math
import threading

import cv2
import numpy as np
from flask import Flask, request, jsonify, send_file, send_from_directory
from PIL import Image, ImageDraw, ImageFont

HERE = os.path.dirname(os.path.abspath(__file__))
PROJ = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(PROJ, "harness"))
import replay as R  # noqa: E402  (load_from_main / Replay / load_frames / LABEL)

TASKS_DIR = os.path.join(HERE, "tasks")
UPLOAD_DIR = os.path.join(HERE, "uploads")   # 视频唯一入口=用户上传, 项目自身不捆绑视频
VIDEO_DIRS = [UPLOAD_DIR]
os.makedirs(TASKS_DIR, exist_ok=True)
os.makedirs(UPLOAD_DIR, exist_ok=True)

app = Flask(__name__, static_folder="static", static_url_path="/static")

TO_LAB = R.make_lab_converter()
_font_cache = {}

# 算法命名空间：可热重载（改完 main.py 不必重启服务, 见 /api/reload_algo）。
_NS = None
_NS_ERR = None


def load_algo():
    """从主文件(重新) exec 常量与纯函数。成功返回 None, 失败返回错误字符串并记进 _NS_ERR。

    失败时**保留上一次成功的命名空间**：宁可用旧算法继续跑, 也不要因为 main.py 里一个
    手误让整个平台起不来（改算法的人正需要一边试一边看帧）。
    """
    global _NS, _NS_ERR
    try:
        ns = R.load_from_main()
    except Exception as e:
        _NS_ERR = "%s: %s" % (type(e).__name__, e)
        return _NS_ERR
    _NS, _NS_ERR = ns, None
    return None


load_algo()


def reload_algo():
    """重载 main.py 的常量/函数并清空回放会话 —— 改完算法后点'重跑算法'即生效,
    不用重启服务。帧序列不变, 清缓存只为让后续请求按新代码重算。
    返回 None(成功) 或错误字符串（主文件载入失败时**继续用上一次的算法**）。"""
    err = load_algo()
    _ALGO_SESS.clear()
    return err


@app.post("/api/reload_algo")
def api_reload_algo():
    err = reload_algo()
    # 不用 500：主文件手误时前端要能把错误原文显示出来，同时界面继续可用（跑旧算法）
    return jsonify({"ok": err is None, "error": err, "stale": err is not None})

# ---------------- 小工具 ----------------

def _font(size=16):
    if size not in _font_cache:
        for p in (r"C:\Windows\Fonts\msyh.ttc", r"C:\Windows\Fonts\simhei.ttf"):
            if os.path.exists(p):
                _font_cache[size] = ImageFont.truetype(p, size)
                break
        else:
            _font_cache[size] = ImageFont.load_default()
    return _font_cache[size]


def tdir(name):
    return os.path.join(TASKS_DIR, name)


def read_json(path, default):
    try:
        with open(path, encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return default


def write_json(path, obj):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1)


def imread_u(path):
    """读图（Windows 非 ASCII 路径安全）。

    cv2.imread 把路径交给 OpenCV 的 ANSI 文件 API，中文路径一律读不到（返回 None，
    日志里出现把 UTF-8 字节当 GBK 的乱码路径，如 tasks/娴嬭瘯/）—— 任务名允许中文，
    tasks/测试 就这么挂过。np.fromfile 走 Python 的 Unicode 路径 API，取字节后 imdecode，
    结果与 imread 完全一致。
    注意 cv2.VideoCapture / VideoWriter **不受影响**（FFmpeg 后端吃 UTF-8），别顺手也改。
    """
    try:
        buf = np.fromfile(path, dtype=np.uint8)
    except OSError:
        return None
    if buf.size == 0:
        return None
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


def gt_from_line(line):
    """真值线 [x1,y1,x2,y2](320x240 坐标) → (gt_off, gt_ang)。公式与主文件出量一致,
    常量也来自主文件 —— 真值口径和算法口径永远同源。"""
    g = _NS
    x1, y1, x2, y2 = line
    if abs(x2 - x1) < 1e-6:            # 竖线: 角度顶格, 行取两端均值
        m = 1e6
        row = (y1 + y2) / 2.0
    else:
        m = (y2 - y1) / float(x2 - x1)
        row = m * g["CTRL_X"] + (y1 - m * x1)
    error_px = (row - g["FRAME_H"] / 2.0) - g["TARGET_OFFSET"]
    off = max(-100, min(100, int(error_px / g["FRAME_H"] * 2 * g["ERR_GAIN"])))
    ang = max(-900, min(900, int(math.degrees(math.atan(m)) * 10)))
    return off, ang


def run_algo_on_task(name):
    """逐帧跑算法(与 replay.py 完全同一套代码), 返回 records。"""
    meta = read_json(os.path.join(tdir(name), "meta.json"), None)
    if not meta:
        return None
    rp = R.Replay(_NS, TO_LAB)
    recs = []
    for i in range(meta["n_frames"]):
        p = os.path.join(tdir(name), "frames", "%04d.jpg" % i)
        fr = imread_u(p)
        if fr is None:
            break
        recs.append(rp.step(fr))
    return recs


# ---------------- 任务管理 ----------------

@app.get("/api/videos")
def list_videos():
    vids = []
    for d in VIDEO_DIRS:
        if os.path.isdir(d):
            for f in sorted(os.listdir(d)):
                if os.path.splitext(f)[1].lower() in (".mp4", ".avi", ".mov", ".mkv"):
                    vids.append({"name": f, "path": os.path.join(d, f)})
    return jsonify(vids)


@app.post("/api/videos/upload")
def upload_video():
    f = request.files.get("file")
    if not f:
        return jsonify({"error": "no file"}), 400
    dest = os.path.join(UPLOAD_DIR, f.filename)
    f.save(dest)
    return jsonify({"ok": True, "path": dest})


@app.post("/api/tasks")
def create_task():
    q = request.get_json(force=True)
    name = q.get("name", "").strip()
    video = q.get("video", "")
    dedup = bool(q.get("dedup", True))
    every = max(1, int(q.get("every", 1)))
    if not name or not os.path.exists(video):
        return jsonify({"error": "need name & existing video"}), 400
    if not name.replace("_", "").isalnum():
        return jsonify({"error": "任务名只能字母数字下划线"}), 400
    d = tdir(name)
    if os.path.exists(d):
        return jsonify({"error": "任务已存在: " + name}), 400
    os.makedirs(os.path.join(d, "frames"))
    _all, frames = R.load_frames(video) if dedup else (None, None)
    if frames is None:
        cap = cv2.VideoCapture(video)
        frames = []
        while True:
            ok, fr = cap.read()
            if not ok:
                break
            frames.append(fr)
        cap.release()
    frames = frames[::every]
    for i, fr in enumerate(frames):
        p = os.path.join(d, "frames", "%04d.jpg" % i)
        ok, buf = cv2.imencode(".jpg", fr, [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(p, "wb") as f:
            f.write(buf.tobytes())
    write_json(os.path.join(d, "meta.json"),
               {"name": name, "source_video": video, "n_frames": len(frames),
                "dedup": dedup, "every": every, "fps": 14})
    write_json(os.path.join(d, "annotations.json"), {"frames": {}})
    return jsonify({"ok": True, "n_frames": len(frames)})


@app.get("/api/tasks")
def list_tasks():
    out = []
    if os.path.isdir(TASKS_DIR):
        for name in sorted(os.listdir(TASKS_DIR)):
            meta = read_json(os.path.join(tdir(name), "meta.json"), None)
            if not meta:
                continue
            ann = read_json(os.path.join(tdir(name), "annotations.json"), {"frames": {}})
            n_line = sum(1 for v in ann["frames"].values() if v.get("line"))
            n_state = sum(1 for v in ann["frames"].values() if v.get("state") is not None)
            n_ver = sum(1 for v in ann["frames"].values() if v.get("verified"))
            out.append({"name": name, "n_frames": meta["n_frames"],
                        "n_line": n_line, "n_state": n_state, "n_verified": n_ver})
    return jsonify(out)


@app.delete("/api/task/<name>")
def delete_task(name):
    """删除任务（帧/标注/缓存全删）。名字校验防路径穿越；同时清掉算法回放会话。"""
    if not name.replace("_", "").isalnum():
        return jsonify({"error": "bad name"}), 400
    d = tdir(name)
    if not os.path.isdir(d):
        return jsonify({"error": "no task"}), 404
    import shutil
    shutil.rmtree(d)
    _ALGO_SESS.pop(name, None)
    return jsonify({"ok": True})


@app.get("/api/task/<name>/meta")
def task_meta(name):
    meta = read_json(os.path.join(tdir(name), "meta.json"), None)
    if not meta:
        return jsonify({"error": "no task"}), 404
    return jsonify(meta)


@app.get("/api/task/<name>/frame/<int:i>.jpg")
def task_frame(name, i):
    return send_from_directory(os.path.join(tdir(name), "frames"), "%04d.jpg" % i)


@app.get("/api/task/<name>/annotations")
def get_ann(name):
    return jsonify(read_json(os.path.join(tdir(name), "annotations.json"), {"frames": {}}))


def _norm_line(line):
    """防御性规整真值线：接受扁平 [x1,y1,x2,y2] 或嵌套 [[x1,y1],[x2,y2]]（前端旧
    bug 存过嵌套），统一成扁平 4 元数字组；无法解析返回 None。"""
    if not line:
        return None
    try:
        if len(line) == 2 and isinstance(line[0], (list, tuple)):
            line = [line[0][0], line[0][1], line[1][0], line[1][1]]
        flat = [float(v) for v in line[:4]]
        if any(v != v for v in flat):        # NaN 检查
            return None
        return flat
    except (TypeError, ValueError, IndexError):
        return None


@app.post("/api/task/<name>/annotations/<int:i>")
def post_ann(name, i):
    path = os.path.join(tdir(name), "annotations.json")
    ann = read_json(path, {"frames": {}})
    q = request.get_json(force=True)
    ann["frames"][str(i)] = {"line": _norm_line(q.get("line")), "state": q.get("state"),
                             "note": q.get("note", ""),
                             "verified": bool(q.get("verified", False))}
    write_json(path, ann)
    return jsonify({"ok": True})


# ---------------- 比对(预测 vs 真值) ----------------

def _std(xs):
    """总体标准差；少于 2 个点返回 None（没有"抖动"可言）。"""
    return float(np.std(xs)) if len(xs) > 1 else None


def _res_jitter(rows):
    """残差抖动指标 —— 抓"角度在小范围内高频跳"这类 MAE / worst-20 排序看不见的失效。

    残差 = 预测 − 真值（直接用 row 里的 off_err/ang_err = "有真值线且预测有效"的帧）。
    台阶只在**帧号相邻**的两个残差点之间算：跨过丢线/未标的缺口不算一步 ——
    那不是算法的抖动，是序列断了。预测侧与真值侧取**同一批点对**，才可比。
    判读：真值台阶平顺而预测台阶大 = 算法噪声；两边都大 = 车真的在动。
    """
    out, worst = {}, []
    for axis, kp, kg, kr, unit, suf in (("off", "p_off", "g_off", "off_err", 1.0, ""),
                                        ("ang", "p_ang", "g_ang", "ang_err", 0.1, "_deg")):
        pts = [(r["i"], r[kp] * unit, r[kg] * unit, r[kr] * unit)
               for r in rows if r.get(kr) is not None]
        if len(pts) < 2:
            out[axis] = None
            continue
        pairs = [(a, b) for a, b in zip(pts, pts[1:]) if b[0] == a[0] + 1]
        st_res = [b[3] - a[3] for a, b in pairs]
        # 键名自带单位（off=归一化单位同 off_mae, ang=度）：别让两个轴的数字混用
        out[axis] = {
            "n": len(pts), "n_stairs": len(pairs),
            "res_std" + suf: _std([p[3] for p in pts]),   # 残差离散度（含系统性偏差）
            "stair_std" + suf: _std(st_res),              # 逐帧台阶 std（纯抖动）
            "stair_max" + suf: max((abs(v) for v in st_res), default=None),
            "pred_stair_std" + suf: _std([b[1] - a[1] for a, b in pairs]),  # 算法输出多跳
            "gt_stair_std" + suf: _std([b[2] - a[2] for a, b in pairs]),    # 车真的在动
        }
        # 下面几项都用该轴自己的单位（ang=度, off=归一化单位），"unit" 字段显式标出，
        # 免得前端拿着一个"0.9"猜它是度还是 0.1° 单位
        for (a, b), v in zip(pairs, st_res):
            worst.append({"i": b[0], "axis": axis, "unit": "deg" if suf else "norm",
                          "stair": round(v, 3),
                          "p": round(b[1], 3), "g": round(b[2], 3), "res": round(b[3], 3)})
    worst.sort(key=lambda w: -abs(w["stair"]))
    return out, worst[:20]


@app.post("/api/task/<name>/compare")
def compare(name):
    recs = run_algo_on_task(name)
    if recs is None:
        return jsonify({"error": "no task"}), 404
    ann = read_json(os.path.join(tdir(name), "annotations.json"), {"frames": {}})
    rows, off_errs, ang_errs, st_ok, st_tot = [], [], [], 0, 0
    for i, r in enumerate(recs):
        a = ann["frames"].get(str(i), {})
        row = {"i": i, "valid": r["valid"], "p_off": r["offset"], "p_ang": r["angle"],
               "p_st": r["state"], "g_st": a.get("state"),
               "g_off": None, "g_ang": None, "off_err": None, "ang_err": None,
               "st_ok": None}
        if a.get("line"):
            g_off, g_ang = gt_from_line(a["line"])
            row["g_off"], row["g_ang"] = g_off, g_ang
            if r["valid"]:
                row["off_err"] = r["offset"] - g_off
                row["ang_err"] = r["angle"] - g_ang
                off_errs.append(abs(row["off_err"]))
                ang_errs.append(abs(row["ang_err"] / 10.0))
        if a.get("state") is not None:
            st_tot += 1
            row["st_ok"] = (r["state"] == a["state"]) if r["valid"] == 0 else \
                           (r["state"] == a["state"])
            # 有效帧标注 0/1, 丢线帧标注 2/3/4 —— 直接比 state 数字
            if row["st_ok"]:
                st_ok += 1
        rows.append(row)
    worst = sorted([r for r in rows if r["off_err"] is not None],
                   key=lambda r: -abs(r["off_err"]))[:20]
    st_bad = [r for r in rows if r["st_ok"] is False]
    # 抖动指标（检验 FILT_EMA_A / 黑线优先这类改动的正确指标, 不是 MAE）
    jitter, jit_bad = _res_jitter(rows)
    summary = {
        "n_frames": len(recs),
        "n_line_gt": len(off_errs),
        "n_state_gt": st_tot,
        "off_mae": float(np.mean(off_errs)) if off_errs else None,
        "off_max": int(max(off_errs)) if off_errs else None,
        "ang_mae_deg": float(np.mean(ang_errs)) if ang_errs else None,
        "state_match": f"{st_ok}/{st_tot}",
        "n_valid": sum(1 for r in recs if r["valid"]),
        "jitter": jitter,
    }
    write_json(os.path.join(tdir(name), "report.json"),
               {"summary": summary, "rows": rows, "worst": worst, "st_bad": st_bad,
                "jit_bad": jit_bad})
    return jsonify({"summary": summary, "worst": worst, "st_bad": st_bad,
                    "jit_bad": jit_bad})


# ---------------- 实时单帧计算（验证页点击即算） ----------------
# 算法是有状态的（EMA/参考色/基线逐帧演化），算第 i 帧必须从 0 跑到 i。
# 会话缓存：前向浏览只增量推进（点过的帧瞬时返回）；回退跳看则从头重跑一遍
# （帧序列不可变 → 结果确定，重跑只为恢复状态）。真值线每次请求现读，改标注即生效。
_ALGO_SESS = {}   # task -> {"rp": Replay, "done": int, "recs": {i: record}}


def _algo_record(name, i, fresh=False):
    """算第 i 帧的算法结果。fresh=True → 丢掉缓存、从第 0 帧重放（验证页"重跑算法"用：
    算法有状态，要让改动生效就必须从头发车前重放一遍，不能只重算这一帧）。"""
    meta = read_json(os.path.join(tdir(name), "meta.json"), None)
    if not meta or i < 0 or i >= meta["n_frames"]:
        return None
    fdir = os.path.join(tdir(name), "frames")
    sess = _ALGO_SESS.get(name)
    if fresh or sess is None or i < sess["done"]:
        sess = _ALGO_SESS[name] = {"rp": R.Replay(_NS, TO_LAB), "done": -1, "recs": {}}
    if i not in sess["recs"]:
        for k in range(sess["done"] + 1, i + 1):
            fr = imread_u(os.path.join(fdir, "%04d.jpg" % k))
            if fr is None:
                break
            sess["recs"][k] = sess["rp"].step(fr)
            sess["done"] = k
    return sess["recs"].get(i)


@app.get("/api/task/<name>/frame_algo/<int:i>.jpg")
def frame_algo(name, i):
    rec = _algo_record(name, i, fresh=request.args.get("fresh") == "1")
    if rec is None:
        return jsonify({"error": "bad frame"}), 404
    fr = imread_u(os.path.join(tdir(name), "frames", "%04d.jpg" % i))
    if fr is None:                       # 帧文件缺失/损坏：404, 别让 _draw_overlay 炸成 500
        return jsonify({"error": "no frame jpg"}), 404
    ann = read_json(os.path.join(tdir(name), "annotations.json"), {"frames": {}})
    out = _draw_overlay(fr, rec, ann["frames"].get(str(i)))
    ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 90])
    return send_file(io.BytesIO(buf.tobytes()), mimetype="image/jpeg")


# ---------------- 验证渲染(整段导出 mp4 用) ----------------

def _draw_overlay(frame_bgr, rec, gt=None):
    """预测(黄点/绿线/标签) + 真值(青线) 烧进画面。返回 BGR。"""
    vis = frame_bgr.copy()
    g = _NS
    # 真值线(青, 画在底层)
    if gt and gt.get("line"):
        x1, y1, x2, y2 = [int(v) for v in gt["line"]]
        cv2.line(vis, (x1, y1), (x2, y2), (255, 255, 0), 1)
    # 预测交界点(黄点)
    for (x, row) in rec["pts"]:
        cv2.circle(vis, (int(x), int(row)), 2, (0, 255, 255), -1)
    # 预测拟合线(绿)
    if rec["fit"]:
        m, c0 = rec["fit"][0], rec["fit"][1]
        cv2.line(vis, (0, int(m * 0 + c0)), (320, int(m * 320 + c0)), (0, 255, 0), 1)
    # 标签条(PIL 中文)
    pil = Image.fromarray(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
    d = ImageDraw.Draw(pil)
    tag = "V%d[%s] off=%+d ang=%+.1f" % (rec["valid"], R.LABEL[rec["state"]],
                                         rec["offset"], rec["angle"] / 10.0)
    if gt and gt.get("line"):
        g_off, g_ang = gt_from_line(gt["line"])
        tag += " | 真值 off%+d ang%+.1f st%s" % (g_off, g_ang / 10.0,
                                                 "" if gt.get("state") is None else str(gt["state"]))
    d.rectangle([0, 0, 320, 15], fill=(0, 0, 0))
    d.text((3, 1), tag, font=_font(12), fill=(255, 255, 255))
    return cv2.cvtColor(np.array(pil), cv2.COLOR_RGB2BGR)


@app.post("/api/task/<name>/render")
def render(name):
    d = tdir(name)
    meta = read_json(os.path.join(d, "meta.json"), None)
    if not meta:
        return jsonify({"error": "no task"}), 404
    recs = run_algo_on_task(name)
    ann = read_json(os.path.join(d, "annotations.json"), {"frames": {}})
    vdir = os.path.join(d, "verif")
    os.makedirs(vdir, exist_ok=True)
    for f in os.listdir(vdir):
        os.remove(os.path.join(vdir, f))
    vw = cv2.VideoWriter(os.path.join(d, "verif.mp4"),
                         cv2.VideoWriter_fourcc(*"mp4v"), meta["fps"], (320, 240))
    for i, r in enumerate(recs):
        fr = imread_u(os.path.join(d, "frames", "%04d.jpg" % i))
        out = _draw_overlay(fr, r, ann["frames"].get(str(i)))
        vw.write(out)
        ok, buf = cv2.imencode(".jpg", out, [cv2.IMWRITE_JPEG_QUALITY, 88])
        with open(os.path.join(vdir, "%04d.jpg" % i), "wb") as f:
            f.write(buf.tobytes())
    vw.release()
    write_json(os.path.join(d, "verif_manifest.json"),
               {"n": len(recs), "fps": meta["fps"]})
    return jsonify({"ok": True, "n": len(recs)})


@app.get("/api/task/<name>/verif_manifest")
def verif_manifest(name):
    """验证页打开时先问这个：有缓存的渲染结果就不用重算（返回 404 = 未渲染过）。"""
    man = read_json(os.path.join(tdir(name), "verif_manifest.json"), None)
    if not man:
        return jsonify({"error": "not rendered"}), 404
    return jsonify(man)


@app.get("/api/task/<name>/verif/<int:i>.jpg")
def verif_frame(name, i):
    return send_from_directory(os.path.join(tdir(name), "verif"), "%04d.jpg" % i)


@app.get("/api/task/<name>/verif.mp4")
def verif_mp4(name):
    return send_file(os.path.join(tdir(name), "verif.mp4"))


@app.get("/")
def index():
    return send_from_directory(os.path.join(HERE, "static"), "index.html")


if __name__ == "__main__":
    # 打印 PID：服务跑在启动它的那个 cmd 窗口里, 关掉窗口 = 干净地杀掉这个进程
    # （也可以 停止标注平台.bat 按端口杀, 或 taskkill /PID 下面这个号 /T）。
    # 刻意不开 debug：Flask 的 reloader 会另起子进程, 窗口/PID 就对不上了。
    print("=" * 60)
    print("标注/验证平台: http://127.0.0.1:8765")
    print("PID %d —— 关掉本窗口 = 停止服务" % os.getpid())
    print("改了 main.py 不必重启：验证页点 ⟳ 重跑算法（或按 R）热重载")
    print("=" * 60)
    app.run(host="127.0.0.1", port=8765, threaded=True)
