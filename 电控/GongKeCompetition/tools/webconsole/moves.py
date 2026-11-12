#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""位移变量示意图 —— 「位移」Tab 的后端。

变量本体（#define）与中文译名（DSL 尾注释）都住在 Application/app_config.h 的
「路线位移变量」段, 解析/改值/改名/新增直接复用 tuning.py —— 本模块只管三件事:

  · moves.json: 示意图配置（图片路径 / 起点标记 / 线段绑定表）。⚠️ **纯可视化**:
    线段唯一的语义是「这个变量对应图上这一段路」, 不做任何比例尺/投影换算 ——
    图上摆错位置最多难看, 不影响任何数据; 数值修改走 /api/tuning（位移 Tab 左栏
    或「调参」Tab 都是它）。
  · 变量清单下发（GET /api/moves 的 variables 段 —— 「地图」Tab 的参数徽章
    （宏名 → 中文译名, 只读点击跳转）靠它, 前端不写死任何变量名。
  · 删除变量前的 map.c 引用检查: 删掉一个被引用的宏 = `undefined identifier`
    （故意的硬失败, 同 ARM_ACT_n 删除的口径）—— 启用行引用直接拦, 仅禁用行
    （行首 //, 不进编译）引用放行但带警告。

与 tuning.json / phrases.json 同侧: moves.json 是共享内容, **入库**。
"""

import json
import os
import re
import threading
from pathlib import Path

import tuning                          # 变量的解析/改值/改名/新增/删除全在那边

APP_DIR = Path(__file__).resolve().parent
ROOT = tuning.ROOT
MAP_C = ROOT / "Application" / "map.c"
MOVES_JSON = APP_DIR / "moves.json"
WRITE_LOCK = threading.RLock()         # 可重入: delete_var 末段持锁改 moves.json, save_config 内部重入同一把

SECTION_TITLE = "路线位移变量"          # 必须与 app_config.h 里的段头一字不差
DEFAULT_IMAGE = "docs/DefaultMap.JPG"   # 相对仓库根; 不存在时界面提示、不影响变量管理
MAX_SEGMENTS = 64

RE_VAR_NAME = re.compile(r"^[A-Z][A-Z0-9_]*$")


# =====================================================================
# moves.json: 示意图配置
# =====================================================================

def _default_config():
    return {"image": DEFAULT_IMAGE, "start": None, "segments": []}


def _seg_name(v):
    v = str(v or "").strip()
    return v if RE_VAR_NAME.match(v) else ""


def _sanitize_config(data):
    """外来 JSON → 合法配置副本（坏条目丢弃而不是整份拒绝 —— 图示数据坏了不该把 Tab 搞挂）。"""
    cfg = _default_config()
    if not isinstance(data, dict):
        return cfg
    img = data.get("image")
    if isinstance(img, str) and img.strip():
        cfg["image"] = img.strip().replace("\\", "/")
    start = data.get("start")
    if isinstance(start, dict):
        try:
            cfg["start"] = {"x": int(start.get("x")), "y": int(start.get("y"))}
        except (TypeError, ValueError):
            pass
    segs = data.get("segments")
    if isinstance(segs, list):
        out = []
        for s in segs[:MAX_SEGMENTS]:
            if not isinstance(s, dict):
                continue
            try:
                out.append({"name": _seg_name(s.get("name")),
                            "x1": float(s.get("x1")), "y1": float(s.get("y1")),
                            "x2": float(s.get("x2")), "y2": float(s.get("y2"))})
            except (TypeError, ValueError):
                continue
        cfg["segments"] = out
    return cfg


def load_config():
    if not MOVES_JSON.exists():
        return _default_config()
    try:
        return _sanitize_config(json.loads(MOVES_JSON.read_text(encoding="utf-8")))
    except (ValueError, OSError):
        return _default_config()


def version():
    return MOVES_JSON.stat().st_mtime if MOVES_JSON.exists() else 0.0


def save_config(config, base_version=None):
    """整份写 moves.json（临时文件 + 原子替换）。内容等价 → 不写盘, 版本原样返回。"""
    cfg = _sanitize_config(config)
    with WRITE_LOCK:
        ver = version()
        if base_version is not None and abs(base_version - ver) > 1e-6:
            return False, {"error": "conflict",
                           "message": "示意图已被其他客户端修改, 请先重载再保存"}, 409
        if MOVES_JSON.exists():
            try:
                if _sanitize_config(json.loads(MOVES_JSON.read_text(encoding="utf-8"))) == cfg:
                    return True, {"ok": True, "version": ver}, 200
            except (ValueError, OSError):
                pass
        tmp = MOVES_JSON.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(cfg, ensure_ascii=False, indent=2) + "\n",
                       encoding="utf-8")
        os.replace(tmp, MOVES_JSON)
        return True, {"ok": True, "version": version()}, 200


def resolve_image(cfg):
    """配置里的相对路径 → 仓库内绝对路径; 绝对路径/出界/不存在 → None。

    ⚠️ image 会进服务端的文件读取, 必须关死「指到仓库外面」的门（`..` 与盘符路径）。"""
    rel = (cfg or {}).get("image") or ""
    if not rel:
        return None
    p = Path(rel.replace("\\", "/"))
    if p.is_absolute():
        return None
    try:
        full = (ROOT / p).resolve()
        full.relative_to(ROOT.resolve())
    except (ValueError, OSError):
        return None
    return full if full.is_file() else None


def image_response():
    """GET /api/moves/image → (status, bytes, ctype)。没图回 404（前端画占位提示）。"""
    img = resolve_image(load_config())
    if img is None:
        return 404, b"", "text/plain; charset=utf-8"
    ext = img.suffix.lower()
    ctype = "image/jpeg" if ext in (".jpg", ".jpeg") else \
            "image/png" if ext == ".png" else "application/octet-stream"
    return 200, img.read_bytes(), ctype


# =====================================================================
# 变量清单（「路线位移变量」段）与删除
# =====================================================================

def variables():
    """段内全部变量 → [{name, display, num, unit, kind, conv, range, note}]。

    给位移 Tab 左栏与地图徽章。⚠️ conv/range/note 必须带全: 改译名走 /api/tuning/meta
    是**整条尾注释重写**, 前端要全字段透传, 缺哪个哪个就被无声抹掉。"""
    out = []
    for g in tuning.parse_config()["groups"]:
        if g["title"] != SECTION_TITLE:
            continue
        for it in g["items"]:
            pub = tuning.public_item(it)
            out.append({"name": pub["name"], "display": pub["display"],
                        "num": pub["num"], "unit": pub["unit"] or "mm",
                        "kind": pub["kind"], "conv": pub["conv"],
                        "range": pub["range"], "note": pub["note"]})
    return out


def var_refs(name):
    """map.c 里引用了该宏 → (启用行引用数, [启用它的预设名…], [仅禁用行引用的预设名…])。

    行级扫描而非 parse_map: 禁用行只是注释、不进编译 —— 它们的引用只提醒不拦。
    识别预设标题的正则与 map_codec._locate_presets 同形状（单行表头）。"""
    active_n, active_where, disabled_where = 0, [], []
    try:
        lines = MAP_C.read_text(encoding="utf-8").split("\n")
    except OSError:
        return 0, [], []
    pat = re.compile(r"\b%s\b" % re.escape(name))
    cur = "(未命名预设)"
    in_block = False
    for ln in lines:
        hm = re.match(r"^\s*/\*\s*=+\s*预设:\s*(.+?)\s*=+\s*\*/\s*$", ln)
        if hm:
            cur = hm.group(1).strip()
            continue
        body = ln.lstrip()
        # 分区包裹（整段 /* */ 禁用）里的行不进编译 —— 引用只提醒不拦, 同禁用行口径
        if in_block:
            if "*/" in body:
                in_block = False
            continue
        if body.startswith("/*"):
            if "*/" not in body:
                in_block = True
            continue
        if not pat.search(ln) or not body.startswith(("SCRIPT_STEP", "//")):
            continue
        if body.startswith("//"):
            if cur not in disabled_where:
                disabled_where.append(cur)
        else:
            active_n += 1
            if cur not in active_where:
                active_where.append(cur)
    return active_n, active_where, disabled_where


def add_var(name, display, value, base_version=None):
    """新增一个变量（float, mm, @(-2000~2000)）。段在 → 追加段尾; 段没了 → 在 #endif 前重建。"""
    name = str(name or "").strip()
    if not RE_VAR_NAME.match(name):
        return False, {"error": "bad_name",
                       "message": "宏名只能是大写字母/数字/下划线, 且以字母开头"}, 400
    if any(v["name"] == name for v in variables()):
        return False, {"error": "exists", "message": "%s 已经存在" % name}, 400
    val = str(value if value is not None else "").strip() or "0"
    try:
        float(val)
    except ValueError:
        # 契约是 DSL 固定 float —— 非数字按 RAW 字面量写进 #define 行会写坏
        # app_config.h（还可能吞掉行尾 DSL 注释）, 必须在这里拦下
        return False, {"error": "bad_value",
                       "message": "初始值必须是数字（float, mm, 例: -180 或 120.5）"}, 400
    has_section = any(g["title"] == SECTION_TITLE
                      for g in tuning.parse_config()["groups"])
    param = {"name": name, "value": val, "kind": "float",
             "meta": {"display": (display or "").strip() or name, "unit": "mm",
                      "range": {"lo": "-2000", "hi": "2000"}},
             "target": SECTION_TITLE if has_section else None,
             "new_section": None if has_section else SECTION_TITLE}
    return tuning.write_config(None, [param], base_version)


def delete_var(name, base_version=None):
    """删一个变量: 查引用 → 删 app_config.h 整行（tuning.delete_params）→ 解绑线段。"""
    name = str(name or "").strip()
    if not RE_VAR_NAME.match(name):
        return False, {"error": "bad_name", "message": "变量名不合法"}, 400
    if not any(v["name"] == name for v in variables()):
        return False, {"error": "not_found",
                       "message": "app_config.h 里没有 %s" % name}, 404
    active_n, active_where, disabled_where = var_refs(name)
    if active_n:
        return False, {"error": "referenced",
                       "message": "map.c 还有 %d 步在用 %s（%s）—— 先把那些行改回数字"
                                  "或删掉, 再来删变量" % (active_n, name,
                                                        "、".join(active_where))}, 400
    ok, resp, status = tuning.delete_params([name], base_version)
    if not ok:
        return ok, resp, status
    # 解绑线段持锁（RLock, save_config 内部重入同一把）: 否则「读 cfg → 过滤 → 写」
    # 之间另一客户端可保存示意图, 解绑结果被覆盖
    with WRITE_LOCK:
        cfg = load_config()
        before = len(cfg["segments"])
        cfg["segments"] = [s for s in cfg["segments"] if s["name"] != name]
        removed = before - len(cfg["segments"])
        if removed:
            save_config(cfg, None)
    msg = "已删除 %s（Keil 重编译前 map.c 那些宏名引用会先编译失败 —— 确认你已经处理）" % name
    if disabled_where:
        msg += "; 注意: 「%s」的禁用行里还有引用, 启用前记得改回数字" % "、".join(disabled_where)
    if removed:
        msg += "; 解绑了 %d 条线段" % removed
    resp["message"] = msg
    return True, resp, status


def state():
    """GET /api/moves 的响应体。"""
    cfg = load_config()
    img = resolve_image(cfg)
    return {
        "version": version(),
        "tuning_version": tuning.version(),
        "config": cfg,
        "image": {"path": cfg["image"], "url": "/api/moves/image",
                  "mtime": int(img.stat().st_mtime) if img else 0,
                  "exists": img is not None},
        "variables": variables(),
    }
