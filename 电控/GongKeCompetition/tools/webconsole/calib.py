#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
「标定」Tab 的后端 —— 主摄标定预设的管理中心。

背景: 新路线用到的四个模块（看路/色球/人质/射靶粗调）各有一批**随主摄个体变**的
标定量（图像列/行设定点 + px↔mm 比例尺）。换一颗同型号但视场不同的相机, 这批值
必须整套重标 —— 本模块按「一台主摄一套预设」把它们管理起来:

  · 预设本体只存 tools/webconsole/calib.json（**入库**, 同 tuning.json 纪律）;
  · 「保存预设」只写 calib.json, 绝不碰 app_config.h;
  · 「设为当前」(apply) 才写 app_config.h 的宏 —— 复用 tuning.write_config 的
    行级回写（.bak / 类型保真 / 零 diff）, 之后要在 Keil 全量重编译烧录才上车;
  · 这批参数在「调参」Tab 里**不再出现**（tuning.state() 按 CALIB_PARAMS 剔除）,
    元数据（显示名/单位/范围/备注）仍从 app_config.h 尾注释解析, 两边同源。

参数清单的唯一来源 = tuning.CALIB_GROUPS（放 tuning.py 是为了避免循环导入）。
预设必须是**完整快照**: 每个参数都有值（「套一半新相机的标定、剩一半旧相机的」
比「不生效」更危险, save 侧硬拦）。
"""

import json
import os
import re
import shutil
import threading
from pathlib import Path

import tuning

APP_DIR = Path(__file__).resolve().parent

CALIB_JSON = APP_DIR / "calib.json"

# calib.json 的写盘串行化（一次只有一个写者在「读 mtime 比对 base_version → 写回」里跑）。
# app_config.h 那边由 tuning.WRITE_LOCK 自己串行（apply 里借用）, 两把锁互不嵌套对端。
WRITE_LOCK = threading.Lock()

MAX_PRESETS = 12          # 一台车备几颗主摄也就到头了; 拦手滑连环新建
MAX_NAME_LEN = 24
MAX_NOTE_LEN = 120

REBUILD_MSG = "已写回 app_config.h —— 需在 Keil µVision 全量重编译并烧录后车上才生效"


class CalibError(Exception):
    """可直接展示给用户的错误。"""


# =====================================================================
# calib.json 读写
# =====================================================================

def _defaults():
    return {"active": None, "next_id": 1, "presets": []}


def _load():
    """读 calib.json; 不存在/损坏 → 空骨架（损坏不抛: 页面要能打开去修）。"""
    if not CALIB_JSON.exists():
        return _defaults()
    try:
        data = json.loads(CALIB_JSON.read_text(encoding="utf-8"))
        if not isinstance(data, dict):
            return _defaults()
        presets = data.get("presets")
        if not isinstance(presets, list):
            presets = []
        return {"active": data.get("active"),
                "next_id": int(data.get("next_id") or 1),
                "presets": presets}
    except Exception:                                             # noqa: BLE001
        return _defaults()


def _dump(data):
    return json.dumps(data, ensure_ascii=False, indent=2)


def version():
    """calib.json 的 mtime（= /api/rev 的 calib 戳与保存端点的 base_version, 同源）。"""
    try:
        return CALIB_JSON.stat().st_mtime
    except OSError:
        return None


def _write_json(data):
    """落盘（tmp + os.replace 原子写, 同 tuning.save_prefs）。调用方已持 WRITE_LOCK。
    备份路径从**当前的** CALIB_JSON 现算 —— 测试沙箱会 patch 它, 模块级常量会跟丢。"""
    if CALIB_JSON.exists():
        shutil.copy2(CALIB_JSON, CALIB_JSON.parent / (CALIB_JSON.name + ".bak"))
    tmp = CALIB_JSON.with_suffix(".json.tmp")
    tmp.write_text(_dump(data), encoding="utf-8")
    os.replace(tmp, CALIB_JSON)


def _version_ok(base_version):
    """乐观锁: base_version 与当前 mtime 同源比对（None = 文件还不存在, 双方都 None 才过）。"""
    if base_version is None:
        return version() is None
    try:
        bv = float(base_version)
    except (TypeError, ValueError):
        return False
    cur = version()
    if cur is None:
        return False
    return abs(bv - cur) < 1e-6


# =====================================================================
# 校验 / 清洗
# =====================================================================

def _clean_text(s, cap, what):
    """预设名/备注 → 单行、无控制字符、限长（只进 JSON, 不进 C; 轻清洗即可）。"""
    s = "".join(ch for ch in str(s or "") if ch >= " ")
    s = re.sub(r"\s+", " ", s).strip()[:cap]
    if what and not s:
        raise CalibError("%s不能为空" % what)
    return s


def _calib_items():
    """app_config.h 里 CALIB_PARAMS 的条目索引。→ (by_name, missing)。"""
    parsed = tuning.parse_config()
    by_name = {}
    for g in parsed["groups"]:
        for it in g["items"]:
            if it["name"] in tuning.CALIB_PARAMS:
                by_name[it["name"]] = it
    missing = [n for n in tuning.CALIB_PARAMS if n not in by_name]
    return by_name, missing


def _validate_values(values, by_name):
    """预设值表必须是**完整快照**且每个值都能过 from_display。→ 清洗后的 {名: 字符串}。"""
    if not isinstance(values, dict):
        raise CalibError("values 必须是对象")
    lack = [n for n in tuning.CALIB_PARAMS if n not in values]
    if lack:
        raise CalibError("预设缺参数: %s（预设必须是完整快照, 缺的请照当前宏值补上）"
                         % ", ".join(lack))
    bad = []
    for n in tuning.CALIB_PARAMS:
        try:
            tuning.from_display(by_name[n], values[n])
        except (ValueError, KeyError) as e:
            bad.append("%s（%s）" % (n, e))
    if bad:
        raise CalibError("以下值不合法: %s" % "; ".join(bad))
    return {n: str(values[n]).strip() for n in tuning.CALIB_PARAMS}


def _assign_ids(presets, next_id):
    """id=None 的新预设由服务端发号; next_id 只增不减（同 map/arm/chact 的纪律 ——
    复用旧号会让别的客户端缓存的预设指向另一台相机）。"""
    seen = set()
    for p in presets:
        pid = p.get("id")
        if pid is None:
            continue
        pid = int(pid)
        if pid in seen:
            raise CalibError("预设 id 重复: %d" % pid)
        seen.add(pid)
        next_id = max(next_id, pid + 1)
    for p in presets:
        if p.get("id") is None:
            p["id"] = next_id
            next_id += 1
    return next_id


# =====================================================================
# 动作
# =====================================================================

def _save(payload, base_version):
    """整份预设列表写 calib.json。只写这个文件, 绝不碰 app_config.h。"""
    if not _version_ok(base_version):
        return False, {"error": "conflict",
                       "message": "calib.json 已被其他客户端修改, 请先「重载」"}, 409

    presets_in = payload.get("presets")
    if not isinstance(presets_in, list):
        raise CalibError("presets 必须是列表")
    if len(presets_in) > MAX_PRESETS:
        raise CalibError("预设最多 %d 套" % MAX_PRESETS)

    by_name, missing = _calib_items()
    if missing:
        raise CalibError("app_config.h 缺少标定参数: %s —— 先把宏补回来再保存"
                         % ", ".join(missing))

    clean = []
    for p in presets_in:
        if not isinstance(p, dict):
            raise CalibError("预设必须是对象")
        pid = p.get("id")
        clean.append({
            "id": None if pid is None else int(pid),
            "name": _clean_text(p.get("name"), MAX_NAME_LEN, "预设名"),
            "note": _clean_text(p.get("note"), MAX_NOTE_LEN, None),
            "values": _validate_values(p.get("values"), by_name),
        })
    next_id = _load()["next_id"]          # 以已存值为下限: 客户端发来旧值不许让号回退
    try:
        next_id = max(next_id, int(payload.get("next_id") or 0))
    except (TypeError, ValueError):
        pass
    next_id = _assign_ids(clean, next_id)

    active = payload.get("active")
    if active is not None:
        try:
            active = int(active)
        except (TypeError, ValueError):
            raise CalibError("active 必须是数字（预设 id）")
        if active not in [p["id"] for p in clean]:
            raise CalibError("active 指向的预设不存在 —— 当前生效的预设不能删, "
                             "要换先对别套「设为当前」")

    data = {"active": active, "next_id": next_id, "presets": clean}
    old_text = CALIB_JSON.read_text(encoding="utf-8") if CALIB_JSON.exists() else None
    new_text = _dump(data)
    changed = old_text != new_text
    if changed:
        _write_json(data)
    return True, {"ok": True, "version": version(), "changed": changed,
                  "presets": clean, "next_id": next_id, "active": active}, 200


def _apply(payload):
    """「设为当前」: 把预设值写回 app_config.h 的宏（经 tuning.write_config）, 并在
    calib.json 里标记 active。⚠️ 只改宏不驱动任何硬件 —— 生效要等 Keil 重编译烧录。"""
    try:
        pid = int(payload.get("id"))
    except (TypeError, ValueError):
        raise CalibError("id 必须是数字（预设 id）")
    if not _version_ok(payload.get("base_version")):
        return False, {"error": "conflict",
                       "message": "calib.json 已被其他客户端修改, 请先「重载」"}, 409

    data = _load()
    preset = next((p for p in data["presets"] if int(p.get("id") or 0) == pid), None)
    if preset is None:
        return False, {"error": "not found", "message": "没有这套预设"}, 404

    by_name, missing = _calib_items()
    if missing:
        raise CalibError("app_config.h 缺少标定参数: %s —— 宏被改名/删行了, 先修源码"
                         % ", ".join(missing))

    # 与当前宏值比一遍: 只把真有差异的行交给 write_config（全等 → 一个字节都不写,
    # 连 .bak 都不建 —— app_config.h 是「调参」Tab 的版本戳, 白写一次会让他那边吃 409）。
    values = _validate_values(preset.get("values"), by_name)
    changes = {}
    for n in tuning.CALIB_PARAMS:
        it = by_name[n]
        new_val = tuning.from_display(it, values[n])
        if new_val == it["value"] or tuning._same_number(it, new_val):
            continue
        changes[n] = values[n]

    active_changed = data.get("active") != pid
    changed_cfg = bool(changes)
    warnings = []
    if changes:
        ok, resp, status = tuning.write_config(changes, [], payload.get("base_cfg"))
        if not ok:
            return ok, resp, status
        warnings = resp.get("warnings") or []
        cfg_version = resp.get("version")
    else:
        cfg_version = tuning.version()

    if active_changed:
        data["active"] = pid
        _write_json(data)

    note = REBUILD_MSG if changed_cfg else "预设值与 app_config.h 现值一致, 没有写盘"
    return True, {"ok": True, "changed_cfg": changed_cfg,
                  "cfg_version": cfg_version, "version": version(),
                  "active": pid, "warnings": warnings, "note": note}, 200


# =====================================================================
# 分发 / 状态
# =====================================================================

def run_calib_action(action, payload):
    """POST /api/calib 的分发（save/apply）。契约同 chassis_action.run_chact_action
    （返回 (ok, data, error, status), status 是失败时回的 HTTP 状态码）。"""
    with WRITE_LOCK:
        try:
            if action == "save":
                ok, data, status = _save(payload, payload.get("base_version"))
            elif action == "apply":
                ok, data, status = _apply(payload)
            else:
                return False, None, "未知动作: %s" % action, 400
            if ok:
                return True, data, "", 200
            return False, data, data.get("message") or data.get("error") or "操作失败", status
        except CalibError as exc:
            return False, None, str(exc), 400
        except Exception as exc:                                 # noqa: BLE001
            return False, None, "内部错误: %s" % exc, 500


def state():
    """GET /api/calib 的响应体: 四组参数元数据 + 当前宏值 + 预设列表。

    宏缺失（被改名/删行）不静默: error 字段点名, 前端横幅提示; 预设照发,
    但 apply 会被拦（保存也会 —— 都过不了 _calib_items 那道门）。
    """
    by_name, missing = _calib_items()
    groups = []
    for title, names in tuning.CALIB_GROUPS:
        items = [tuning.public_item(by_name[n]) for n in names if n in by_name]
        if items:
            groups.append({"title": title, "items": items})
    data = _load()
    out = {"version": version(), "cfg_version": tuning.version(),
           "groups": groups, "missing": missing,
           "active": data["active"], "next_id": data["next_id"],
           "presets": data["presets"]}
    if missing:
        out["error"] = "app_config.h 缺少标定参数: " + ", ".join(missing)
    return out
