#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
app_config.h 解析 / 序列化引擎 —— 「调参」Tab 的后端。

与 server.py（map.c 领域）、host_tools.py（Windows 宿主交互）并列，本模块只管
Application/app_config.h。

沿用 map.c 的两条铁律:
  · 只改必要行, 未修改行按原始物理行回写 → 无操作保存 = 字节级 diff 为空
  · 保存前备份 app_config.h.bak

======================================================================
注释 DSL —— 源码是唯一数据源（与 app_script.h 的参数 DSL 同风格）
======================================================================

    #define NAME  VALUE  /* <显示名> [<显示单位>] {<转换>} @(<范围>) // <备注> */

  显示名      [ 之前的所有文本; 不写就打回宏名
  [X]         界面显示的单位
  {A->B}      转换: 底层存 A, 界面显示 B（查本模块 CONVERSIONS 表）
  @(lo~hi)    范围, 以**显示单位**计; 任一端可省（@(2500~) / @(~100)）
  //          备注

值与类型**不进 DSL** —— 从值本身推断（见 classify_value）: `3000u` 是 uint、`45.0f`
是 float、`(-30)` 带括号的 int、`80` 是纯 int、`UART_ID_RPI` 是 raw。编辑后按原类型与
原风格回写（u 后缀 / N.0f / 纯数字 / 是否带括号），所以用户不用自己敲 u/f。

只想改注释（显示名/备注等）时, **值那一段连空格一起原样保留**, 只换 /* */ 里的内容,
diff 里就只看得到注释在变。
"""

import json
import math
import os
import re
import shutil
import sys
import threading
from pathlib import Path

import map_codec                     # 只为 write_text_like（行尾风格保护; 见 map_codec 该函数）

APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parents[1]                       # tools/webconsole/ → 仓库根
APP_CONFIG = ROOT / "Application" / "app_config.h"

# 写盘串行化（一次只有一个写者在「校验版本 + 整文件写回」里跑）。
# 见 server.py 的 WRITE_LOCK: 单机单用户时这个窗口不可能撞上, 绑 0.0.0.0 多人同时用之后可以。
WRITE_LOCK = threading.Lock()
PREFS_FILE = APP_DIR / "tuning.json"

RE_SECTION = re.compile(r"^\s*/\*\s*=+\s*(.+?)\s*=+\s*\*/\s*$")
RE_DEFINE = re.compile(r"^(\s*)#define\s+(\w+)(\s+)(\S+)(\s*)(/\*.*\*/\s*)?$")
RE_COMMENT = re.compile(r"/\*(.*)\*/", re.S)

# 「标定」Tab 托管的参数（calib.py 消费; 定义在这里而非 calib.py 是为了避免循环导入 ——
# calib 要用本模块的解析/回写）。这些宏**不再出现在「调参」Tab**（state() 里剔除）,
# 但 write_config 照常可写（「设为当前」走的就是它）。
#
# 语义: 随**主摄个体**变的标定量（图像列/行设定点 + px↔mm/每格px 比例尺, 换一颗
# 同型号但视场不同的相机就要整套重标的那批）, 按四组分块。分组是 app_config.h
# 段落归属的固化快照 —— 不依赖段头文本（段标题改字不应把标定 Tab 搞挂）。
# ⚠️ 加/减名字要连 calib.py 的保存校验、test_calib.py 的 fixture 一起改。
CALIB_GROUPS = [
    ("看路", ("ROAD_DX_IDEAL_PX", "ROAD_DX_IDEAL_REV_PX", "ROAD_DY_IDEAL_PX",
              "ROAD_DY_IDEAL_REV_PX",
              "ROAD_DX_SCALE_MM_PER_PX", "ROAD_DY_SCALE_MM_PER_PX")),
    ("色球", ("BALL_CX_CENTER", "BALL_CY_CENTER",
              "BALL_PX_TO_MM_X", "BALL_PX_TO_MM_Y",
              "BALL_YAW_SCALE", "BALL_YAW_TARGET_DEG")),
    ("人质", ("HOSTAGE_CX_CENTER", "HOSTAGE_CY_CENTER",
              "HOSTAGE_PX_TO_MM_X", "HOSTAGE_PX_TO_MM_Y")),
    ("射靶粗调", ("TARGET_SHOOT_CX_AIM", "TARGET_SHOOT_CY_AIM",
                  "TARGET_SHOOT_GAP_PX")),
]
CALIB_PARAMS = [n for _, names in CALIB_GROUPS for n in names]
_CALIB_NAME_SET = frozenset(CALIB_PARAMS)

KIND_UINT, KIND_INT, KIND_FLOAT, KIND_RAW = "uint", "int", "float", "raw"

# 转换表（闭集）。to_display / to_stored 都可能抛 ValueError —— 调用方必须接住,
# 不能把一个算不出来的值写进头文件（ms->Hz 在 0 处会除零）。
CONVERSIONS = {
    "ms->Hz": {"to_display": lambda v: 1000.0 / v,
               "to_stored": lambda v: 1000.0 / v,
               "unit": "Hz"},
    "‰->%": {"to_display": lambda v: v / 10.0,
             "to_stored": lambda v: v * 10.0,
             "unit": "%"},
}


# =====================================================================
# 值: 类型推断 / 数值解析 / 回写格式化
# =====================================================================

def classify_value(raw):
    """'3000u' / '45.0f' / '(-30)' / '80' / 'UART_ID_RPI' → {kind, paren, plus, num}。

    num 是剥掉后缀与括号后的纯数字文本（raw 类型下为 None）。
    plus 记下作者写没写那个显式的 '+'（如 `(+1.0f)` 与 `(-1.0f)` 成对的方向标志），
    回写时照原样保留, 免得把 + 吃掉后在 diff 里看着像改了语义。"""
    s = raw.strip()
    paren = False
    if s.startswith("(") and s.endswith(")") and s.count("(") == 1:
        paren, s = True, s[1:-1].strip()
    plus = s.startswith("+")

    def num_of(pat):
        return bool(re.fullmatch(pat, s))

    if num_of(r"[+-]?\d+[uU]"):
        return {"kind": KIND_UINT, "paren": paren, "plus": plus, "num": s[:-1]}
    if num_of(r"[+-]?(?:\d+\.\d*|\.\d+|\d+)[fF]"):
        return {"kind": KIND_FLOAT, "paren": paren, "plus": plus, "num": s[:-1]}
    if num_of(r"[+-]?(?:\d+\.\d*|\.\d+|\d+)"):
        kind = KIND_FLOAT if "." in s else KIND_INT
        return {"kind": kind, "paren": paren, "plus": plus, "num": s}
    return {"kind": KIND_RAW, "paren": paren, "plus": False, "num": None}


def to_number(item):
    """条目的数值（raw 类型或解析失败 → None）。"""
    num = item.get("num")
    if num is None:
        return None
    try:
        return float(num)
    except ValueError:
        return None


def _fmt_float(v):
    s = "%.6f" % v
    s = s.rstrip("0")
    return s + "0" if s.endswith(".") else s


def format_value(kind, paren, value, plus=False):
    """按原类型/风格把数值写回字面量（用户不用自己敲 u/f）。"""
    if kind == KIND_UINT:
        body = "%du" % int(round(value))
    elif kind == KIND_FLOAT:
        body = "%sf" % _fmt_float(value)
    elif kind == KIND_INT:
        body = "%d" % int(round(value))
    else:
        body = str(value)
    if plus and kind in (KIND_INT, KIND_FLOAT) and value >= 0:
        body = "+" + body
    return "(%s)" % body if paren else body


# =====================================================================
# 注释 DSL
# =====================================================================

def parse_dsl(text):
    """/* */ 之间的文本 → {display, unit, conv, range:{lo,hi}, note}。

    先切 // 备注: 备注里可能带 [] 或 {}（比如「afF[6]」这类说明），
    留在前半段会被误当成单位/转换。"""
    text = (text or "").strip()
    note = ""
    i = text.find("//")
    if i >= 0:
        note, text = text[i + 2:].strip(), text[:i].strip()

    rng = {"lo": "", "hi": ""}
    m = re.search(r"@\(\s*([^)~]*?)\s*~\s*([^)]*?)\s*\)\s*$", text)
    if m:
        rng = {"lo": m.group(1).strip(), "hi": m.group(2).strip()}
        text = text[:m.start()].strip()

    conv = ""
    m = re.search(r"\{\s*([^}]*?)\s*\}\s*$", text)
    if m:
        conv, text = m.group(1).strip(), text[:m.start()].strip()

    unit = ""
    m = re.search(r"\[\s*([^\]]*?)\s*\]\s*$", text)
    if m:
        unit, text = m.group(1).strip(), text[:m.start()].strip()

    # 「选项:0=五次S 1=三次S …」（写在备注里）→ 下拉框选项（2026-10 加, ARM_SERVO_CURVE 首用）。
    # ⚠️ 只**派生**一个 options 字段, note 本身一个字符不动 —— note 原样往返,
    #    format_dsl 不需要知道选项的存在, 「逐参数回写零 diff」的测试才不会炸。
    options = None
    m = re.search(r"选项[:：]\s*(.+)$", note)
    if m:
        options = [{"value": v, "label": l} for v, l in
                   re.findall(r"(\d+)\s*=\s*([^\s=,，;；]+)", m.group(1))]
        if not options:
            options = None

    # 「留空=255 默认取FILL_TASK_PCT」(写在备注里) → 空值哨兵参数（2026-10 加, FILL_TASK_*_PCT
    # 首用）: 界面把哨兵态渲染成**空输入框 + 「默认」placeholder**, 空串提交由 from_display
    # 写回哨兵字面量（数值与文件现值相同 → write_config 判「没变」→ 零 diff）。
    # ⚠️ 与 options 同一条纪律: 只**派生**字段, note 一个字符不动, format_dsl 不需要知道哨兵。
    empty, empty_from = None, None
    m = re.search(r"留空[=：:]\s*(\d+)(?:\s*默认取\s*([A-Za-z_]\w*))?", note)
    if m:
        empty, empty_from = int(m.group(1)), m.group(2)

    return {"display": text.strip(), "unit": unit, "conv": conv,
            "range": rng, "note": note, "options": options,
            "empty": empty, "empty_from": empty_from}


def format_dsl(meta):
    """meta → DSL 文本（写回头文件用）。"""
    s = (meta.get("display") or "").strip()
    if meta.get("unit"):
        s += (" " if s else "") + "[%s]" % meta["unit"]
    if meta.get("conv"):
        s += (" " if s else "") + "{%s}" % meta["conv"]
    rng = meta.get("range") or {}
    if rng.get("lo") or rng.get("hi"):
        s += " @(%s~%s)" % (rng.get("lo", ""), rng.get("hi", ""))
    if meta.get("note"):
        s += " // " + meta["note"].strip()
    return s


def check_range(meta, display_value):
    """超出 @(范围) → 提示文本（None = 没超/没范围/无法比较）。只提示, 不拦保存。"""
    rng = meta.get("range") or {}
    lo, hi = rng.get("lo"), rng.get("hi")
    if not lo and not hi:
        return None
    try:
        v = float(display_value)
    except (TypeError, ValueError):
        return None
    if lo:
        try:
            if v < float(lo):
                return "低于下限 %s" % lo
        except ValueError:
            pass
    if hi:
        try:
            if v > float(hi):
                return "高于上限 %s" % hi
        except ValueError:
            pass
    return None


# =====================================================================
# 解析 app_config.h
# =====================================================================

def _parse_line(line, line_no):
    """一行 #define → 条目; 不是 #define 返回 None。"""
    m = RE_DEFINE.match(line)
    if not m:
        return None
    indent, name, gap1, value, gap2, comment = m.groups()
    info = classify_value(value)
    meta = parse_dsl(RE_COMMENT.search(comment).group(1)) if comment else \
        {"display": "", "unit": "", "conv": "", "range": {"lo": "", "hi": ""}, "note": "",
         "empty": None, "empty_from": None}
    return {
        "name": name,
        "value": value,
        "kind": info["kind"],
        "paren": info["paren"],
        "plus": info["plus"],
        "num": info["num"],
        "meta": meta,
        "line": line,
        "line_no": line_no,
        "indent": indent,
        "gap1": gap1,
        "comment_col": line.find("/*") if comment else None,
        "has_comment": bool(comment),
    }


def parse_config(text=None):
    """→ {version, groups:[{title, items:[…]}]}。未分组的前导 define 归 '(未分组)'。"""
    if text is None:
        text = APP_CONFIG.read_text(encoding="utf-8")
    lines = text.split("\n")

    groups, cur = [], {"title": None, "items": [], "header_line": None}
    for i, line in enumerate(lines):
        sm = RE_SECTION.match(line)
        if sm:
            groups.append(cur)
            cur = {"title": sm.group(1), "items": [], "header_line": i}
            continue
        item = _parse_line(line, i)
        if item:
            cur["items"].append(item)
    groups.append(cur)

    groups = [g for g in groups if g["items"] or g["title"]]
    for g in groups:
        if g["title"] is None:
            g["title"] = "(未分组)"
    return {"groups": groups}


def version():
    return APP_CONFIG.stat().st_mtime


def public_item(item):
    """条目 → 给前端的 JSON（附上显示值）。

    显示值**一律是裸数字**（不带 u/f/括号）: 输入框里出现 `300u` 会让用户
    以为要连后缀一起填, 那是早期版本出现「改动被复原」的根因（详见 _strip_literal）。"""
    meta, conv = item["meta"], item["meta"].get("conv") or ""
    num = to_number(item)
    empty = meta.get("empty")
    display_value = item["value"]                       # raw: 符号/表达式原样
    conv_error = None
    if num is not None:
        if conv and conv in CONVERSIONS:
            try:
                display_value = _num_out(CONVERSIONS[conv]["to_display"](num))
            except (ValueError, ZeroDivisionError):
                conv_error = "按 %s 换算时出错（值不能为 0）" % conv
                display_value = item["value"]
        else:
            display_value = _plain_num(num)
    if num is not None and empty is not None and abs(num - empty) < 1e-9:
        display_value = ""      # 哨兵态: 界面渲染空框 + 「默认」placeholder, 不把 255 露给用户
    return {
        "name": item["name"], "value": item["value"], "num": num,
        "kind": item["kind"], "paren": item["paren"],
        "display": meta.get("display") or item["name"],
        "unit": meta.get("unit", ""), "conv": conv,
        "range": meta.get("range", {}), "note": meta.get("note", ""),
        "options": meta.get("options"),
        "empty": empty, "empty_from": meta.get("empty_from"),
        "display_value": display_value, "conv_error": conv_error,
        "range_warning": check_range(meta, display_value) if conv_error is None else None,
        "has_comment": item["has_comment"],
    }


def _num_out(v):
    """换算结果出一道人类可读的数（50 而不是 50.0；30.33 保留两位）。"""
    if abs(v - round(v)) < 1e-9:
        return int(round(v))
    return round(v, 2)


def _plain_num(v):
    """非换算参数的显示值: 裸数字, **不四舍五入**（舍入会让用户重填时悄悄改掉精度）。"""
    return int(v) if abs(v - round(v)) < 1e-9 else v


def _strip_literal(s):
    """容忍用户把字面量原样填回来: 剥掉外层括号与 u/U/f/F 后缀。

    ⚠️ 这条是给一个真实的坑兜底: 早期版本对**没有换算**的参数直接把存储字面量
    （`300u` / `45.0f` / `(-30)`）显示在输入框里, 用户改数字时理所当然会连后缀一起改,
    而 float('300u') 会抛异常 → 该参数被静默跳过 → 界面重新加载后看起来像「改动被复原」。
    现在展示层已经改成只显示裸数字, 这里再兜一层, 两种写法都收。"""
    s = (s or "").strip()
    if s.startswith("(") and s.endswith(")") and len(s) > 2:
        s = s[1:-1].strip()
    if len(s) > 1 and s[-1] in "uUfF" and (s[-2].isdigit() or s[-2] == "."):
        s = s[:-1]
    return s


def to_display(item):
    """存储值 → 显示值（数值型）。无换算时就是原数值。"""
    num = to_number(item)
    if num is None:
        return None
    conv = item["meta"].get("conv") or ""
    if conv in CONVERSIONS:
        try:
            return CONVERSIONS[conv]["to_display"](num)
        except (ValueError, ZeroDivisionError):
            return None
    return num


def from_display(item, display_value):
    """界面填的值（显示单位）→ 存储字面量。算不出来 → 抛 ValueError。

    例外: 空值哨兵参数（备注 DSL「留空=255」）交来**空串** = 「没填」→ 写回哨兵字面量
    （数值与文件现值相同 → write_config 的 _same_number 判「没变」→ 零 diff;
    check_range 对空串也天然跳过, 不会把哨兵当「超范围」告警）。"""
    if str(display_value).strip() == "" and item["meta"].get("empty") is not None:
        return format_value(item["kind"], item["paren"],
                            float(item["meta"]["empty"]), item.get("plus"))

    kind = item["kind"]
    if kind == KIND_RAW:
        # 符号/表达式: 原样写回, 不做数值处理。
        # 只杀换行: 它会拆掉「一行一参数」的行结构（2026-10-09）; 其余字符是
        # #define 值的合法内容, 不多管。
        return re.sub(r"[\r\n]+", " ", str(display_value)).strip()

    conv = item["meta"].get("conv") or ""
    try:
        v = float(_strip_literal(str(display_value)))
    except (TypeError, ValueError):
        raise ValueError("不是合法数字: %r" % display_value)

    if conv:
        if conv not in CONVERSIONS:
            raise ValueError("未知转换: %s" % conv)
        if v == 0:
            raise ValueError("按 %s 换算时 %s 不能为 0" % (conv, display_value))
        v = CONVERSIONS[conv]["to_stored"](v)
    if kind == KIND_UINT and v < 0:
        # 带 u 后缀的无符号槽位: 负值会绕回, 直接拒掉让用户改。
        # （纯 int 是**有符号**的, `(-30)` 这类负值合法, 不能一起拦。）
        raise ValueError("该参数是无符号整数, 不能是负值")
    return format_value(kind, item["paren"], v, item.get("plus"))


# =====================================================================
# 序列化回写
# =====================================================================

def _rebuild_line(item, value_text=None, meta=None):
    """重建一行 #define, 尽量保持原有对齐。

    值未变时逐字节复现原行（零 diff）; 值变长变短时按原注释列重新补空格,
    贴不到就至少留一个空格 —— 只影响观感, 不影响编译。"""
    value = item["value"] if value_text is None else value_text
    head = item["indent"] + "#define " + item["name"] + item["gap1"] + value
    dsl = format_dsl(meta if meta is not None else item["meta"])
    if not dsl:
        return head.rstrip()
    col = item.get("comment_col")
    pad = " " * max(1, (col - len(head)) if col else 1)
    return head + pad + "/* " + dsl + " */"


def write_config(changes, new_params, base_version=None):
    """回写 app_config.h。→ (ok, resp, status)

    changes:    {NAME: 界面填的值(显示单位)}  —— 只改这些行
    new_params: [{name, value, kind, meta, target|new_section}]

    ⚠️ 整段在 WRITE_LOCK 里跑: 「读 mtime 比对 base_version → 整文件写回」之间有
       TOCTOU 窗口, 两个客户端同时保存会**都通过**校验再交错写同一个文件
       （server.py 的 WRITE_LOCK 处有完整说明）。多客户端之后这个窗口才可达。"""
    with WRITE_LOCK:
        return _write_config_locked(changes, new_params, base_version)


def _write_config_locked(changes, new_params, base_version=None):
    cur = version()
    # base_version 只该是数字（write_map 同款门）: 字符串进来 abs() 会 TypeError → 500
    if base_version is not None and not isinstance(base_version, (int, float)):
        return False, {"error": "bad_base_version",
                       "message": "base_version 必须是数字"}, 400
    if base_version is not None and abs(base_version - cur) > 1e-6:
        return False, {"error": "conflict",
                       "message": "app_config.h 已在外部被修改, 请先「重载」再保存"}, 409

    text = APP_CONFIG.read_text(encoding="utf-8")
    lines = text.split("\n")
    warnings = []

    # ---- 1) 改值（按行号自下而上, 避免行号位移） ----
    parsed = parse_config(text)
    by_name = {}
    for g in parsed["groups"]:
        for it in g["items"]:
            by_name[it["name"]] = it
    for it in by_name.values():
        it["_group"] = next(g["title"] for g in parsed["groups"] if it in g["items"])

    for name, shown in (changes or {}).items():
        it = by_name.get(name)
        if it is None:
            warnings.append({"name": name, "msg": "未知参数, 已跳过", "saved": False})
            continue
        try:
            new_val = from_display(it, shown)
        except ValueError as e:
            # saved=False → 前端要把用户填的值留在框里, 不能让它看着像被默默改回去
            warnings.append({"name": name, "msg": str(e), "saved": False})
            continue
        # 数值相等也算没变: 换算参数来回取整（如 30.3Hz → 33ms）不该产生一次写盘
        if new_val == it["value"] or _same_number(it, new_val):
            continue                                  # 没变 → 原行原样保留（零 diff）
        w = check_range(it["meta"], shown)
        if w:
            warnings.append({"name": name, "msg": "超出范围（%s）, 已按填的值写入" % w,
                             "saved": True})
        lines[it["line_no"]] = _rebuild_line(it, value_text=new_val)

    # ---- 2) 新增参数 ----
    if new_params:
        # 2a) 新建分区: 统一插到 #endif 之前
        fresh = [p for p in new_params if p.get("new_section")]
        if fresh:
            at = _endif_index(lines)
            block, done = [], set()
            for p in fresh:
                # 同一个新分区只写一次。⚠️ 用 seen 集合而不是「在循环里重新赋值 fresh」:
                # 后者绑不上 —— `for p in fresh` 的迭代器在循环开始时就已经指向原列表,
                # 重新赋值那个**名字**不影响迭代, 于是同一分区出现两次时会写出两个空分区头。
                sec = p["new_section"]
                if sec in done:
                    continue
                done.add(sec)
                block.append("")
                block.append("/* ==================== %s ==================== */" % sec)
                for q in fresh:
                    if q["new_section"] == sec:
                        block.append(_new_line(q))
            lines[at:at] = block

        # 2b) 插到已有分区末尾
        # ⚠️ `parsed` 只算一次 → 其中每个分区的 `line_no` 都是**插入前**的行号。
        #    在一个循环里连插多条（同一分区或跨分区）时, 必须把已插进去的行数累加进去,
        #    否则后插的会落在**前一条前面**（同一 at 反复插入 = 后插的挤到最前）,
        #    跨分区的还会落到目标分区倒数第二条之前。一次只加一条的接口看不出来。
        parsed = parse_config("\n".join(lines))
        shift = 0
        for p in [q for q in new_params if not q.get("new_section")]:
            g = next((g for g in parsed["groups"] if g["title"] == p.get("target")), None)
            if g is None:
                warnings.append({"name": p["name"],
                                 "msg": "找不到分区 %r, 未新建" % p.get("target"),
                                 "saved": False})
                continue
            if g["items"]:
                at = g["items"][-1]["line_no"] + 1 + shift  # 该分区最后一个 #define 之后
            else:
                # 空分区（只有段头 —— 如「自定义开关」段刚建好还没加过宏）: 插到段头之后。
                # ⚠️ 别 g["items"][-1] —— IndexError（2026-10-30 自定义开关首条 add 真踩）。
                at = (g.get("header_line") or 0) + 1 + shift
            added = [_new_line(p)]
            lines[at:at] = added
            shift += len(added)

    shutil.copy2(APP_CONFIG, APP_CONFIG.parent / (APP_CONFIG.name + ".bak"))
    new_text = "\n".join(lines)
    if text.endswith("\n") and not new_text.endswith("\n"):
        new_text += "\n"
    map_codec.write_text_like(APP_CONFIG, new_text)     # 按原换行风格写（见该函数注释）
    return True, {"ok": True, "version": version(), "warnings": warnings}, 200


def _same_number(item, new_literal):
    """新字面量的数值与当前值相同吗（只对数值型有意义）。"""
    cur = to_number(item)
    new = to_number(classify_value(new_literal))
    if cur is None or new is None:
        return False
    return abs(cur - new) < 1e-9


def _endif_index(lines):
    for i, ln in enumerate(lines):
        if re.match(r"^\s*#endif\b", ln):
            return i
    return len(lines)


def _new_line(p):
    """新建参数的整行文本。"""
    indent = "    " if p.get("indent") is None else p["indent"]
    name, value = p["name"], p.get("value", "0")
    value = re.sub(r"[\r\n]+", " ", str(value))       # 同 from_display 的 raw 分支: 值里不许带换行
    info = classify_value(value)
    kind = p.get("kind") or info["kind"]
    num = to_number(info)
    try:
        literal = format_value(kind, info["paren"], float(num),
                               info["plus"]) if num is not None else str(value)
    except (TypeError, ValueError):
        literal = str(value)
    head = "%s#define %-28s %s" % (indent, name, literal)
    dsl = format_dsl(_sanitize_user_meta(p.get("meta") or {}))
    return (head + "  /* " + dsl + " */") if dsl else head.rstrip()


# =====================================================================
# 改注释（显示名 / 单位 / 转换 / 范围 / 备注）
# =====================================================================

def _sanitize_user_meta(meta):
    """用户可写的 meta 自由文本（display/unit/note/range）→ 清洗副本（不改调用方 dict）。

    ⚠️ 2026-10-09: 这条路径原来**整条漏掉清洗** —— display 里一个 `*/` 会提前闭合
       注释、把后面半个字符串变成**活的 C 代码**（保存 200、编译才爆）; note 里的换行
       会拆掉「一行一参数」的行结构。清洗规则唯一来源 = map_codec.sanitize_block_comment
       （map/arm/oled 三条写盘路早就在用, 幂等）。刻意**不在** format_dsl 里洗:
       format_dsl 还承担「无操作保存零 diff」的逐字节回放, 在入口洗一次即可。"""
    _s = map_codec.sanitize_block_comment

    def _copy(v):
        return _s(v, fallback="")

    out = dict(meta or {})
    for k in ("display", "unit", "note"):
        if out.get(k):
            out[k] = _copy(out[k])
    rng = out.get("range")
    if isinstance(rng, dict):
        rng = dict(rng)
        for k in ("lo", "hi"):
            if rng.get(k):
                rng[k] = _copy(rng[k])
        out["range"] = rng
    return out


def write_meta(name, meta, base_version=None):
    """只重写某参数那一行的 /* */ —— 值那一段连空格一起原样保留。

    ⚠️ 整段在 WRITE_LOCK 里跑, 与 write_config 同一把锁（server.write_meta 也是这个样式）:
       它是「读文件 → 改一行 → 整文件写回」, 与 write_config 的改值路径**写的是同一个文件**。
       不加锁的话, 一个客户端调 /api/tuning 改值、另一个调 /api/tuning/meta 改注释, 会交错
       写 app_config.h —— 轻则丢一次更新, 重则写坏文件, 而且双方各自覆盖 .bak。
    ⚠️ 2026-10-09 起支持可选 base_version（= /api/rev 的 tuning 戳）: 与 map 端点同款
       乐观锁, 两客户端并发改元数据不再「后写静默覆盖先写」。
    """
    # ⚠️ 换算必须是闭集里的那几种。`{...}` 原来**只在改值路径**（from_display）校验,
    #    而这条路径直接 format_dsl 落盘 —— 写进一个非法换算之后, 该参数在「调参」Tab 里
    #    **再也存不进去**（写盘抛"未知转换"、warnings 带 saved:false; 用户只看到一句 ⚠
    #    而值没落盘 —— 这正是根 CLAUDE.md 花大段警告的那个症状）。
    #    前端 convSelect 只给闭集内的选项, 但本控制台**无鉴权**, 任意 API 客户端都能发这一条;
    #    而且显示名末尾自带 `{...}` 也会被解析成换算而自伤。所以在这里挡住。
    conv = (meta or {}).get("conv")
    if conv and conv not in CONVERSIONS:
        return False, {"error": "未知转换: %s（可选: %s）"
                                % (conv, "/".join(sorted(CONVERSIONS)))}, 400
    with WRITE_LOCK:
        # base_version 只该是数字（write_map 同款门）: 字符串落进 isinstance 的
        # else 分支 = 不比对直接写, 乐观锁被无声绕过
        if base_version is not None and not isinstance(base_version, (int, float)):
            return False, {"error": "bad_base_version",
                           "message": "base_version 必须是数字"}, 400
        if isinstance(base_version, (int, float)) \
                and abs(base_version - APP_CONFIG.stat().st_mtime) > 1e-6:
            return False, {"error": "conflict",
                           "message": "app_config.h 已被其他客户端修改, "
                                      "请关闭本对话框重新打开（你的改动不会被覆盖）"}, 409
        return _write_meta_locked(name, _sanitize_user_meta(meta))


def _write_meta_locked(name, meta):
    text = APP_CONFIG.read_text(encoding="utf-8")
    lines = text.split("\n")
    for i, line in enumerate(lines):
        it = _parse_line(line, i)
        if not it or it["name"] != name:
            continue
        lines[i] = _rebuild_line(it, meta=meta)
        shutil.copy2(APP_CONFIG, APP_CONFIG.parent / (APP_CONFIG.name + ".bak"))
        map_codec.write_text_like(APP_CONFIG, "\n".join(lines))   # 按原换行风格写
        return True, {"ok": True, "version": version()}, 200
    return False, {"error": "app_config.h 里找不到 %s" % name}, 404


# =====================================================================
# 删除参数（整行 #define 连同 DSL 尾注释一起拿掉）
# =====================================================================

def delete_params(names, base_version=None):
    """删除若干参数的整行。→ (ok, resp, status)

    ⚠️ 引用检查是**调用方**的事（moves.py 删位移变量前查 map.c）—— 本函数只负责
       干净地删行: 未列出的行一个字节不动, 删前照常备份 .bak。
    ⚠️ 整段在 WRITE_LOCK 里跑（同 write_config/write_meta: 三条路写同一个文件）。"""
    want = set(names or [])
    if not want:
        return False, {"error": "no_names", "message": "没有给出要删除的参数名"}, 400
    with WRITE_LOCK:
        cur = version()
        # base_version 只该是数字（write_map 同款门）, 同 _write_config_locked
        if base_version is not None and not isinstance(base_version, (int, float)):
            return False, {"error": "bad_base_version",
                           "message": "base_version 必须是数字"}, 400
        if base_version is not None and abs(base_version - cur) > 1e-6:
            return False, {"error": "conflict",
                           "message": "app_config.h 已在外部被修改, 请先重载再删"}, 409
        text = APP_CONFIG.read_text(encoding="utf-8")
        lines = text.split("\n")
        drop, found = set(), []
        for i, line in enumerate(lines):
            it = _parse_line(line, i)
            if it and it["name"] in want:
                drop.add(i)
                found.append(it["name"])
        missing = sorted(want - set(found))
        if not found:
            return False, {"error": "not_found",
                           "message": "app_config.h 里没有: %s" % ", ".join(sorted(want))}, 404
        lines = [ln for i, ln in enumerate(lines) if i not in drop]
        shutil.copy2(APP_CONFIG, APP_CONFIG.parent / (APP_CONFIG.name + ".bak"))
        new_text = "\n".join(lines)
        if text.endswith("\n") and not new_text.endswith("\n"):
            new_text += "\n"
        map_codec.write_text_like(APP_CONFIG, new_text)
        resp = {"ok": True, "version": version(), "removed": found}
        if missing:
            resp["missing"] = missing
        return True, resp, 200


# =====================================================================
# 界面偏好（只存可见性与顺序; 元数据在头文件注释里, 不重复存）
# =====================================================================

def load_prefs():
    if PREFS_FILE.exists():
        try:
            data = json.loads(PREFS_FILE.read_text(encoding="utf-8"))
            return {"visible": data.get("visible") or {},
                    "order": data.get("order") or {}}
        except Exception:                                         # noqa: BLE001
            pass
    return {"visible": {}, "order": {}}


def save_prefs(prefs):
    tmp = PREFS_FILE.with_suffix(".json.tmp")
    tmp.write_text(json.dumps({"visible": prefs.get("visible") or {},
                               "order": prefs.get("order") or {}},
                              ensure_ascii=False, indent=2), encoding="utf-8")
    os.replace(tmp, PREFS_FILE)


def default_visible(item):
    """没被用户显式设过时的默认: 数值型可见, 符号/表达式(raw)不可见。"""
    return item["kind"] != KIND_RAW


def state():
    """GET /api/tuning 的响应体: 分区树 + 每个参数的展示元数据 + 可见性 + 已排顺序。

    「标定」Tab 托管的参数（CALIB_PARAMS）在这里**整条剔除**（真移动, 不是 visible:false
    —— 参数选择器/排序都不该再碰它们; 元数据与改值走 calib.py, 仍基于同一份 parse_config）。
    整组被剔空的分区不再下发。"""
    parsed = parse_config()
    prefs = load_prefs()
    groups = []
    for g in parsed["groups"]:
        items = []
        for it in g["items"]:
            if it["name"] in _CALIB_NAME_SET:
                continue
            pub = public_item(it)
            pub["visible"] = prefs["visible"].get(it["name"], default_visible(it))
            items.append(pub)
        if not items:
            continue
        # 排过序的按 order 在前, 没排过的按文件顺序跟在后面（稳定排序）
        order = prefs["order"].get(g["title"]) or []
        items.sort(key=lambda p: order.index(p["name"]) if p["name"] in order else len(order))
        groups.append({"title": g["title"], "items": items})
    return {"version": version(), "groups": groups}
