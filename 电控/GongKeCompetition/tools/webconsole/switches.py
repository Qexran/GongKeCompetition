#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""自定义开关 —— 「自定义」Tab 的后端: 让用户在控制台上动态新增 STEP_IF 的条件判据
（编译层 —— 每个开关 = 一个 0/1 编译期宏, 改完 Keil 重编译烧录才上车）。

数据分三处, 本模块负责让它们**一起变**:
  · tools/webconsole/switches.json（**入库**共享）: 清单 —— {id, 宏名, 中文名, 分类, 备注}
    与 next_id（**只增不回退**, 同预设/ARM_ACT 的纪律 —— 复用旧号会让 map.c 里残留的
    SCRIPT_COND_SW_n 无声指向别的开关）。分类只存这里, 不进固件（纯 UI 分组）。
  · Application/app_config.h「自定义开关」段: 开关的**状态**（SW_* 宏, 0/1, 带 DSL 尾注释
    → 「调参」Tab 自动可见）。增删/改中文名/备注复用 tuning.py（add/delete_params/write_meta）。
  · app_script.h 枚举尾部 + app_script.c 两处「生成块」: 条件下拉的成员 + 求值 getter
    + 条件注册表行 —— 标记之间由本模块整段重写（手改会被覆盖）。

顺序约定（add）: 先写宏（tuning）→ 再重写生成块（getter 引用宏, 反了会留编译错窗口）→
最后写 JSON（失败重试可续建: 宏已在段里而清单没有 → 接管补齐, 不重复建宏）。
app_script.h/.c 的生成块写盘**必须持 server.WRITE_LOCK**（与 /api/meta 的 write_meta
写同一个文件）—— 那把锁在 server.py 的端点处理层拿, 本模块只持 JSON 自己的锁。
"""

import json
import os
import re
import threading
from pathlib import Path

import map_codec
import tuning

APP_DIR = Path(__file__).resolve().parent
ROOT = tuning.ROOT
MAP_C = ROOT / "Application" / "map.c"
SCRIPT_H = ROOT / "Application" / "app_script.h"
SCRIPT_C = ROOT / "Application" / "app_script.c"
SWITCHES_JSON = APP_DIR / "switches.json"
WRITE_LOCK = threading.Lock()          # 只串行化 switches.json 自己的「读版本→比对→写」

SECTION_TITLE = "自定义开关"            # 必须与 app_config.h 里的段头一字不差
SW_BASE = 100                          # 条件枚举 id 从这里起（手写成员 0..4, 留足间隔）
RE_SW_NAME = re.compile(r"^[A-Z][A-Z0-9_]*$")
MAX_SWITCHES = 64                      # 条件下拉一屏选不完是小事, 注册表膨胀没意义

# 三处生成块的标记（必须与源文件里的单行标记一字不差 —— find_marked 行锚定）
RE_H_BEGIN = re.compile(r"^\s*/\*\s*=+\s*自定义条件生成块\s*[:：]\s*开始\s*=+\s*\*/\s*$")
RE_H_END = re.compile(r"^\s*/\*\s*=+\s*自定义条件生成块\s*[:：]\s*结束\s*=+\s*\*/\s*$")
RE_GET_BEGIN = re.compile(r"^\s*/\*\s*=+\s*自定义条件 getter 生成块\s*[:：]\s*开始\s*=+\s*\*/\s*$")
RE_GET_END = re.compile(r"^\s*/\*\s*=+\s*自定义条件 getter 生成块\s*[:：]\s*结束\s*=+\s*\*/\s*$")
RE_REG_BEGIN = re.compile(r"^\s*/\*\s*=+\s*自定义条件注册生成块\s*[:：]\s*开始\s*=+\s*\*/\s*$")
RE_REG_END = re.compile(r"^\s*/\*\s*=+\s*自定义条件注册生成块\s*[:：]\s*结束\s*=+\s*\*/\s*$")


# =====================================================================
# 清单（switches.json）
# =====================================================================

def _default_data():
    return {"next_id": SW_BASE, "groups": [], "switches": []}


def _load():
    if not SWITCHES_JSON.exists():
        return _default_data()
    try:
        data = json.loads(SWITCHES_JSON.read_text(encoding="utf-8"))
    except (ValueError, OSError):
        return _default_data()
    if not isinstance(data, dict):
        return _default_data()
    out = _default_data()
    try:
        out["next_id"] = int(data.get("next_id", SW_BASE))
    except (TypeError, ValueError):
        pass
    if isinstance(data.get("groups"), list):
        out["groups"] = [str(g) for g in data["groups"] if str(g).strip()][:32]
    if isinstance(data.get("switches"), list):
        for s in data["switches"][:MAX_SWITCHES]:
            if not isinstance(s, dict):
                continue
            # 名字复检（add 有白名单, 读回路径也要有）: switches.json 是入库共享文件,
            # 手改/合并损坏的名字会经 _getter_block 内插进 app_script.c 的 return 语句
            if not RE_SW_NAME.match(str(s.get("name", ""))):
                continue
            try:
                out["switches"].append({
                    "id": int(s["id"]), "name": str(s.get("name", "")),
                    "display": str(s.get("display", "")),
                    "group": str(s.get("group", "")),
                    "note": str(s.get("note", "")),
                })
            except (KeyError, TypeError, ValueError):
                continue
    return out


def _save(data):
    tmp = SWITCHES_JSON.with_suffix(".json.tmp")
    tmp.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    os.replace(tmp, SWITCHES_JSON)


def version():
    return SWITCHES_JSON.stat().st_mtime if SWITCHES_JSON.exists() else 0.0


def _by_id(data, sw_id):
    for s in data["switches"]:
        if s["id"] == sw_id:
            return s
    return None


# =====================================================================
# app_config.h「自定义开关」段: 宏的现读与写入（复用 tuning.py）
# =====================================================================

def _section_items():
    """段内宏 → {name: tuning item}（现读, 每次现查 —— app_config.h 可能被手改过）。"""
    out = {}
    for g in tuning.parse_config()["groups"]:
        if g["title"] == SECTION_TITLE:
            for it in g["items"]:
                out[it["name"]] = it
    return out


def _macro_value(name):
    it = _section_items().get(name)
    if it is None:
        return None
    return int(tuning.to_number(it) or 0)


def sw_refs(sw_id):
    """map.c 里引用了该条件 → (启用行引用数, [预设名…], [仅禁用行引用的预设名…])。

    删掉一个被 IF 引用的条件 = `undefined identifier`（故意的硬失败, 同删 ARM_ACT_n）。
    行级扫描, 与 moves.var_refs 同一套口径（禁用行是注释、不进编译 → 只提醒不拦）。"""
    token = "SCRIPT_COND_SW_%d" % int(sw_id)
    active_n, active_where, disabled_where = 0, [], []
    try:
        lines = MAP_C.read_text(encoding="utf-8").split("\n")
    except OSError:
        return 0, [], []
    pat = re.compile(r"\b%s\b" % re.escape(token))
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


# =====================================================================
# 生成块重写（app_script.h 枚举成员 / app_script.c getter + 注册表）
# =====================================================================

def _enum_member(sw):
    d = map_codec.sanitize_block_comment(sw["display"], fallback=sw["name"])
    return "    SCRIPT_COND_SW_%-3d = %d,   /* 自定义:%s */" % (sw["id"], sw["id"], d)


def _getter_block(sw):
    d = map_codec.sanitize_block_comment(sw["display"], fallback=sw["name"])
    return ("static uint8_t c_ucCondSw%d(void)   /* %s */\n"
            "{\n"
            "    return %s;\n"
            "}" % (sw["id"], d, sw["name"]))


def _reg_row(sw):
    d = map_codec.sanitize_block_comment(sw["display"], fallback=sw["name"])
    return "    { SCRIPT_COND_SW_%-3d, c_ucCondSw%-3d },   /* %s */" % (
        sw["id"], sw["id"], d)


def _rewrite_marked(path, begin_re, end_re, new_body_lines, backup_suffix):
    """把 path 里 begin/end 标记之间的行整段替换。⚠️ 必须持 server.WRITE_LOCK（写 app_script.h）
    ⚠️ find_marked 返回 (begin+1, end) —— span[1] 就是**结束标记行本身**, tail 从它起切
    （把标记保住）; 切成 span[1]+1 会把结束标记吃掉, 下一次重写就找不到块了。"""
    lines = path.read_text(encoding="utf-8").split("\n")
    span = map_codec.find_marked(lines, begin_re, end_re)
    if span is None:
        raise RuntimeError("%s 里找不到生成块标记" % path.name)
    head = lines[:span[0]]
    tail = lines[span[1]:]
    import shutil
    shutil.copy2(path, path.parent / (path.name + backup_suffix))
    new_lines = head + new_body_lines + tail
    map_codec.write_text_like(path, "\n".join(new_lines))


def write_generated(switches):
    """按清单顺序重写三处生成块。⚠️ 调用方必须持 server.WRITE_LOCK（app_script.h 与
    /api/meta 共写）。顺序 = 清单数组顺序（下拉顺序/注册表顺序都跟它）。"""
    _rewrite_marked(SCRIPT_H, RE_H_BEGIN, RE_H_END,
                    [_enum_member(s) for s in switches], ".bak")
    _rewrite_marked(SCRIPT_C, RE_GET_BEGIN, RE_GET_END,
                    [b for s in switches for b in _getter_block(s).split("\n")] or [""],
                    ".bak")
    _rewrite_marked(SCRIPT_C, RE_REG_BEGIN, RE_REG_END,
                    [_reg_row(s) for s in switches], ".bak")


# =====================================================================
# 动作（server.py 的 /api/switches 分发到这里; 返回 (ok, resp, status)）
# =====================================================================

def state():
    """GET /api/switches: 清单 + 每个开关的当前宏值（现读 app_config.h）。"""
    data = _load()
    items = _section_items()
    switches = []
    for s in data["switches"]:
        it = items.get(s["name"])
        num = int(tuning.to_number(it) or 0) if it is not None else None
        switches.append({
            "id": s["id"], "name": s["name"], "display": s["display"],
            "group": s["group"], "note": s["note"], "value": num,
            "macro_missing": it is None,     # 宏被人手删了 → 界面要显式标出来
        })
    return {"version": version(), "tuning_version": tuning.version(),
            "groups": data["groups"], "switches": switches,
            "section_exists": any(g["title"] == SECTION_TITLE
                                  for g in tuning.parse_config()["groups"])}


def _unique_check(name, display, data, ignore_id=None):
    if not RE_SW_NAME.match(name or ""):
        return "宏名只能是大写字母/数字/下划线, 且以字母开头"
    for s in data["switches"]:
        if s["id"] != ignore_id and s["name"] == name:
            return "宏名 %s 已被别的开关占用" % name
    for s in data["switches"]:
        if s["id"] != ignore_id and s["display"] == display:
            return "中文名「%s」已占用 —— 条件下拉里会重名" % display
    return ""


def add(name, display, group, note, base_version=None):
    name = str(name or "").strip()
    display = map_codec.sanitize_block_comment(str(display or "").strip(),
                                               fallback=name)
    group = map_codec.sanitize_block_comment(str(group or "").strip(), fallback="")
    note = map_codec.sanitize_block_comment(str(note or "").strip(), fallback="")
    with WRITE_LOCK:
        data = _load()
        err = _unique_check(name, display, data)
        if err:
            return False, {"error": "bad_input", "message": err}, 400
        if len(data["switches"]) >= MAX_SWITCHES:
            return False, {"error": "too_many",
                           "message": "开关太多了（上限 %d 个）—— 条件下拉选不完" % MAX_SWITCHES}, 400
        existing = _section_items().get(name)
        if existing is not None:
            # 续建: 宏已在段里（上次 add 中途断掉）→ 不重复建宏, 直接接管进清单
            sw_id = data["next_id"]
            data["next_id"] = sw_id + 1
        else:
            sw_id = data["next_id"]
            has_section = any(g["title"] == SECTION_TITLE
                              for g in tuning.parse_config()["groups"])
            param = {"name": name, "value": "1", "kind": "int", "indent": "",
                     "meta": {"display": display or name, "unit": "",
                              "range": {"lo": "0", "hi": "1"}, "note": note},
                     "target": SECTION_TITLE if has_section else None,
                     "new_section": None if has_section else SECTION_TITLE}
            ok, resp, status = tuning.write_config(None, [param], base_version)
            if not ok:
                return ok, resp, status
            data["next_id"] = sw_id + 1
        data["switches"].append({"id": sw_id, "name": name,
                                 "display": display or name,
                                 "group": group, "note": note})
        write_generated(data["switches"])      # ⚠️ 需外层持 server.WRITE_LOCK
        _save(data)
        return True, {"ok": True, "version": version(),
                      "id": sw_id,
                      "message": "已新增「%s」（SCRIPT_COND_SW_%d）—— Keil 重编译烧录后, "
                                 "IF 积木的条件下拉里才会出现它" % (display or name, sw_id)}, 200


def rename(sw_id, display=None, group=None, note=None, base_version=None):
    try:
        sw_id = int(sw_id)
    except (TypeError, ValueError):
        return False, {"error": "bad_id", "message": "开关 id 必须是数字"}, 400
    with WRITE_LOCK:
        data = _load()
        sw = _by_id(data, sw_id)
        if sw is None:
            return False, {"error": "not_found", "message": "清单里没有这个开关"}, 404
        if display is not None:
            display = map_codec.sanitize_block_comment(str(display).strip(),
                                                       fallback=sw["name"])
            err = _unique_check(sw["name"], display, data, ignore_id=sw_id)
            if err:
                return False, {"error": "bad_input", "message": err}, 400
        # DSL 全字段透传（write_meta 是整条尾注释重写, 缺哪个哪个被抹掉）
        it = _section_items().get(sw["name"])
        if (display is not None or note is not None) and it is not None:
            m = it["meta"]
            ok, resp, status = tuning.write_meta(sw["name"], {
                "display": display if display is not None else m.get("display", ""),
                "unit": m.get("unit", ""), "conv": m.get("conv", ""),
                "range": m.get("range") or {"lo": "0", "hi": "1"},
                "note": map_codec.sanitize_block_comment(str(note), fallback="")
                        if note is not None else m.get("note", ""),
            }, base_version)
            if not ok:
                return ok, resp, status
        if display is not None:
            sw["display"] = display
        if note is not None:
            sw["note"] = map_codec.sanitize_block_comment(str(note), fallback="")
        if group is not None:
            sw["group"] = map_codec.sanitize_block_comment(str(group).strip(), fallback="")
        if display is not None:
            write_generated(data["switches"])   # 下拉标签 = 中文名 → 枚举尾注释要跟
        _save(data)
        return True, {"ok": True, "version": version()}, 200


def toggle(sw_id, value, base_version=None):
    try:
        sw_id = int(sw_id)
    except (TypeError, ValueError):
        return False, {"error": "bad_id", "message": "开关 id 必须是数字"}, 400
    with WRITE_LOCK:
        data = _load()
        sw = _by_id(data, sw_id)
        if sw is None:
            return False, {"error": "not_found", "message": "清单里没有这个开关"}, 404
        v = str(value).strip()
        if v not in ("0", "1"):
            return False, {"error": "bad_value", "message": "开关值只能是 0/1"}, 400
        ok, resp, status = tuning.write_config({sw["name"]: v}, None, base_version)
        if not ok:
            return ok, resp, status
        return True, {"ok": True, "version": version(),
                      "message": "已写回 %s = %s（编译层 —— Keil 重编译烧录后才上车）"
                                 % (sw["name"], v)}, 200


def delete(sw_id, base_version=None):
    try:
        sw_id = int(sw_id)
    except (TypeError, ValueError):
        return False, {"error": "bad_id", "message": "开关 id 必须是数字"}, 400
    with WRITE_LOCK:
        data = _load()
        sw = _by_id(data, sw_id)
        if sw is None:
            return False, {"error": "not_found", "message": "清单里没有这个开关"}, 404
        active_n, active_where, disabled_where = sw_refs(sw_id)
        if active_n:
            return False, {"error": "referenced",
                           "message": "map.c 还有 %d 处 IF 在用这个条件（%s）—— 先把那些"
                                      "积木换成别的条件, 再来删" % (active_n, "、".join(active_where))}, 400
        ok, resp, status = tuning.delete_params([sw["name"]], base_version)
        if not ok:
            return ok, resp, status
        data["switches"] = [s for s in data["switches"] if s["id"] != sw_id]
        write_generated(data["switches"])      # ⚠️ 需外层持 server.WRITE_LOCK
        _save(data)
        msg = "已删除「%s」" % sw["display"]
        if disabled_where:
            msg += "; 注意: 「%s」的禁用行里还有引用, 启用前记得换条件" % "、".join(disabled_where)
        return True, {"ok": True, "version": version(), "message": msg}, 200


def set_switches(order_ids, groups=None):
    """批量保存顺序/分组（拖动排序、分类整理）。顺序 = 生成块的书写顺序（下拉顺序跟着变）。"""
    with WRITE_LOCK:
        data = _load()
        by_id = {s["id"]: s for s in data["switches"]}
        try:
            ids = [int(x) for x in (order_ids or [])]
        except (TypeError, ValueError):
            return False, {"error": "bad_input", "message": "order_ids 必须是 id 列表"}, 400
        if sorted(ids) != sorted(by_id):
            return False, {"error": "bad_input",
                           "message": "order_ids 与现有清单对不上（先刷新再试）"}, 400
        data["switches"] = [by_id[i] for i in ids]
        if groups is not None:
            data["groups"] = [str(g) for g in groups if str(g).strip()][:32]
        write_generated(data["switches"])      # ⚠️ 需外层持 server.WRITE_LOCK
        _save(data)
        return True, {"ok": True, "version": version()}, 200
