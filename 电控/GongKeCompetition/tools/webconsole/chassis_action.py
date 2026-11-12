#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""底盘「动作组」表 —— app_chassis_action.c / .h 的解析、序列化、回收站。

一个「动作组」= 一条时间轴上的若干条目（世界系平移段 / 陀螺闭环转弯段 /
臂动作事件 / 等臂事件），固件的速度混合器按 20ms 一拍连续播放（叠加=向量求和,
直走+转弯叠加=弧线）。保存后它在「地图」积木库里显示成**一块大积木**
（server.py 的 _chact_entries 虚拟出来），拖进路线就是一步
STEP_CHASSIS_ACTION_PLAY_WAIT。

文件布局（两个文件都由控制台维护, 但格式固定、可手改）:

  app_chassis_action.h   生成块（`/* ==== 生成块: 开始 ==== */` 之间）
                         = CH_ACT_NEXT_ID + emChActIdTdf + CH_ACT_GROUP_NUM
                         其余（结构体、API 声明）是手写的, 控制台不碰
  app_chassis_action.c   数据区（`/* ==== 数据区: 开始 ==== */` 之间）
                         = 每组一个条目表 + 末尾 g_astChActGroup[] 聚合初始化
                         其余（播放器实现）是手写的, 控制台不碰

铁律（与 arm_action.py / map.c 同款, 本项目对「回写源码」的统一要求）:
  · **未编辑的条目按原始物理行回吐** → 无操作保存 = 两个文件都字节级 diff 为空
  · 内容没变就**一个字节都不写**（连 .bak 都不建）
  · 写盘前 `.bak` 备份; 写盘按原换行风格（map_codec.write_text_like）

⚠️ 预设的**表头两行**（`预设:` / `备注:`）是无条件重新生成的, 手改排版会被
   规范回去; 条目表内部（数据行）按原始行保留。

⚠️ 子条目的「眼睛」（禁用）= 行首加 `//`, 与 map.c 的单步禁用同一套规则
   （map_codec._toggle_disable）。被禁用的行是注释、不进数组, 所以聚合初始化里
   的条目数用 _active_line_count 扣掉它们。

⚠️ 本模块**不 import server**（循环）。臂动作下拉的合法性校验由调用方传
   arm_action.parse_arm 的 id 集合进来（save 时 server 会带上）。
"""

import functools
import json
import math
import os
import re
import shutil
import threading
import time
from pathlib import Path

import map_codec

APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parents[1]

ACTION_C = ROOT / "Application" / "app_chassis_action.c"
ACTION_H = ROOT / "Application" / "app_chassis_action.h"
CONFIG_H = ROOT / "Application" / "app_config.h"
TRASH_JSON = APP_DIR / "chact_trash.json"

# 数据区/生成块标记（控制台只替换这两个标记之间的内容）
RE_DATA_BEGIN = re.compile(r"^\s*/\*\s*=+\s*数据区\s*[:：]\s*开始\s*=+\s*\*/\s*$")
RE_DATA_END = re.compile(r"^\s*/\*\s*=+\s*数据区\s*[:：]\s*结束\s*=+\s*\*/\s*$")
RE_GEN_BEGIN = re.compile(r"^\s*/\*\s*=+\s*生成块\s*[:：]\s*开始\s*=+\s*\*/\s*$")
RE_GEN_END = re.compile(r"^\s*/\*\s*=+\s*生成块\s*[:：]\s*结束\s*=+\s*\*/\s*$")

RE_TABLE_DEF = re.compile(r"^\s*const\s+stChActItemTdf\s+(\w+)\s*\[\s*\]\s*=\s*\{\s*$")
RE_AGGREGATE = re.compile(
    r"^\s*const\s+stChActGroupTdf\s+g_astChActGroup\s*\[\s*CH_ACT_GROUP_NUM\s*\]"
    r"\s*=\s*\{\s*$")
RE_ENTRY = re.compile(
    r'^\s*\{\s*(\d+)u\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*(\w+)\s*,\s*(\d+)u\s*\}\s*,?\s*$')
RE_NOTE_LINE = re.compile(r"^\s*/\*\s*备注\s*[:：]\s*(.*?)\s*\*/\s*$")
RE_NEXT_ID = re.compile(r"^\s*#define\s+CH_ACT_NEXT_ID\s+(\d+)u\s*$")

# 条目行: 宏名 + 括号参数 + 可选尾注释（// 或 /* */ 两种都认, 手写时两种都合法）
RE_ITEM = re.compile(
    r"^\s*(CH_ACT_MOVE|CH_ACT_TURN|CH_ACT_PIVOT|CH_ACT_ARM|CH_ACT_ARM_WAIT)\s*\((.*)\)"
    r"\s*,?\s*(?://\s*(.*)|/\*\s*(.*?)\s*\*/)?\s*$")
RE_ARM_ID = re.compile(r"^\s*ARM_ACT_(\d+)\s*$")

# app_config.h 的常量（估时镜像 / 时间轴网格 / 条目上限 / 调参卡的离线默认值;
# 解析不出来用保守默认）
RE_CFG_FLOAT = {
    "time_scale": re.compile(r"#define\s+CH_ACT_TIME_SCALE\s+([-\d.fF]+)"),
    "pulse_per_mm_y": re.compile(r"#define\s+CHASSIS_PULSE_PER_MM_Y\s+([-\d.fF]+)"),
    # ⚠️ 键名 = 蓝牙参数名去掉 ch_ 前缀（前端调参卡的离线默认值按这个约定查）
    "head_kp": re.compile(r"#define\s+CH_ACT_HEAD_KP\s+([-\d.fF]+)"),
    "head_ki": re.compile(r"#define\s+CH_ACT_HEAD_KI\s+([-\d.fF]+)"),
    "head_kd": re.compile(r"#define\s+CH_ACT_HEAD_KD\s+([-\d.fF]+)"),
    "head_ki_max": re.compile(r"#define\s+CH_ACT_HEAD_KI_MAX\s+([-\d.fF]+)"),
    "head_db_deg": re.compile(r"#define\s+CH_ACT_HEAD_DB_DEG\s+([-\d.fF]+)"),
    "w_max": re.compile(r"#define\s+CH_ACT_W_MAX\s+([-\d.fF]+)"),
    "wheel_max": re.compile(r"#define\s+CH_ACT_WHEEL_MAX\s+([-\d.fF]+)"),
    # 支点转弯段（固件出厂值**引用** PIVOT_TURN_* —— 这里解析引用源）
    "pivot_kp": re.compile(r"#define\s+PIVOT_TURN_KP\s+([-\d.fF]+)"),
    "pivot_ki": re.compile(r"#define\s+PIVOT_TURN_KI\s+([-\d.fF]+)"),
    "pivot_kd": re.compile(r"#define\s+PIVOT_TURN_KD\s+([-\d.fF]+)"),
    "pivot_ki_max": re.compile(r"#define\s+PIVOT_TURN_MAX_INTEGRAL\s+([-\d.fF]+)"),
    "pivot_w_max": re.compile(r"#define\s+PIVOT_TURN_MAX_SPEED\s+([-\d.fF]+)"),
}
RE_CFG_INT = {
    "loop_ms": re.compile(r"#define\s+CH_ACT_LOOP_MS\s+(\d+)"),
    "items_max": re.compile(r"#define\s+CH_ACT_ITEMS_MAX\s+(\d+)"),
}

MAX_TITLE_LEN = 24
TRASH_MAX = 50
SPEED_MAX = 500          # rpm; 与固件/物理上限对齐的软上限（防手滑输 2100）
ACCEL_MAX = 255          # EMM 单字节档位


class ChActError(Exception):
    """可直接展示给用户的错误。"""


# ==================== 纯函数: 数值 / 文本 ====================

def _to_float(text, default=0.0):
    try:
        return float(text)
    except (TypeError, ValueError):
        return default


def _param_float(token):
    """'500.0f' / '1600u' / '-90' → float（剥掉 C 的 f/F 与 u/U/l/L 后缀）。"""
    t = (token or "").strip()
    try:
        return float(t)
    except ValueError:
        pass
    return _to_float(t.rstrip("fFuUlL"))


def _param_int(token):
    return int(round(_param_float(token)))


def sanitize_comment(text):
    """标题会进 ① `/* 备注: */` 行注释 ② 枚举成员尾注释 —— 清洗规则同 arm。"""
    return map_codec.sanitize_comment_text(text, MAX_TITLE_LEN, "底盘动作组")


def _sanitize_note(text):
    return map_codec.sanitize_block_comment(text)


RE_RESTORE_SUFFIX = re.compile(r"恢复(?:\(\d+\))?$")


def restore_title(original, live_titles):
    stem = RE_RESTORE_SUFFIX.sub("", sanitize_comment(original)).strip() or "动作组"
    base = "%s恢复" % stem
    if base not in live_titles:
        return base
    k = 1
    while "%s(%d)" % (base, k) in live_titles:
        k += 1
    return "%s(%d)" % (base, k)


# ==================== 常量镜像（app_config.h 派生, 不在前端写死） ====================

def config_consts():
    """→ {loop_ms, time_scale, pulse_per_mm_y, items_max, head_kp … wheel_max}。

    前端的时间轴吸附网格 / mm→时长折算 / **调参卡的离线默认值**（未连蓝牙时显示
    app_config.h 的宏, 连上后被设备实时值替换）全用这一份; 蓝牙运行时的 ch_*
    若被当场改过, 设备上的实际值会与这里的宏默认不同 —— 那是**有意**的
    （现场标定量, 宏只是出厂值）, 调参卡会注明。
    """
    out = {"loop_ms": 20, "time_scale": 1.0, "pulse_per_mm_y": 10.3,
           "items_max": 16, "head_kp": 3.0, "head_ki": 0.0, "head_kd": 1.0,
           "head_ki_max": 7.0, "head_db_deg": 1.0, "w_max": 60.0,
           "wheel_max": 300.0, "pivot_kp": 3.0, "pivot_ki": 2.0,
           "pivot_kd": 1.0, "pivot_ki_max": 7.0, "pivot_w_max": 130.0}
    try:
        text = CONFIG_H.read_text(encoding="utf-8")
    except OSError:
        return out
    for key, rx in RE_CFG_FLOAT.items():
        m = rx.search(text)
        if m:
            out[key] = _param_float(m.group(1))
    for key, rx in RE_CFG_INT.items():
        m = rx.search(text)
        if m:
            out[key] = int(m.group(1))
    return out


def estimate_ms(dist_mm, accel, speed, time_scale=None, pulse_per_mm=None):
    """固件 fChassisMoveTimeEst 的 python 镜像（AXIS_Y 档）—— 时间轴排版的依据。

    ⚠️ 只是**估时**: 固件播放时用蓝牙运行时 ch_time_scale 现算, 这里用宏默认。
    两边公式必须逐字符同源（改固件那条公式要连这里一起改）, 否则时间轴上的
    段长与实际播放对不上。
    """
    if time_scale is None or pulse_per_mm is None:
        cc = config_consts()
        time_scale = time_scale if time_scale is not None else cc["time_scale"]
        pulse_per_mm = pulse_per_mm if pulse_per_mm is not None else cc["pulse_per_mm_y"]

    accel = int(accel or 0)
    speed = int(speed or 0)
    if accel == 0:
        accel = 256
    if speed == 0:
        return 200.0
    dist = abs(float(dist_mm or 0.0))
    if dist <= 2.0:
        return 200.0

    quanshu = dist * pulse_per_mm
    quanshu_fenjie = 320.0 * speed * speed / (accel * 6.0)
    time_fenjie = 2000.0 * speed / accel
    if quanshu_fenjie >= quanshu:
        ms = time_fenjie * math.sqrt(quanshu / quanshu_fenjie)
    else:
        ms = 1000.0 * (quanshu - quanshu_fenjie) / (320.0 * speed / 6.0) + time_fenjie
    return ms * time_scale


def snap_ms(value, loop_ms=None):
    """吸附到播放器节拍网格（固件按 20ms 一拍推进, 落在网格外等于白排）。"""
    if loop_ms is None:
        loop_ms = config_consts()["loop_ms"]
    v = int(round(_to_float(value, 0.0)))
    if v < 0:
        v = 0
    return int(round(v / float(loop_ms)) * loop_ms)


# ==================== 解析 ====================

def _find_region(lines, begin_re, end_re):
    begin = end = None
    for i, ln in enumerate(lines):
        if begin is None and begin_re.match(ln):
            begin = i
        elif begin is not None and end_re.match(ln):
            end = i
            break
    if begin is None or end is None:
        raise ChActError("app_chassis_action 文件缺少数据区/生成块标记（被手改坏了？）")
    return begin + 1, end


def _strip_disabled(line):
    """行首 `//`（禁用眼睛）→ (内容行, disabled)。"""
    stripped = line.lstrip()
    if stripped.startswith("//"):
        return stripped[2:], True
    return line, False


def _items_from_body(body):
    """数组体行 → 条目列表。认不出的行原样带上去（raw, 只读显示）。"""
    items = []
    for idx, raw in enumerate(body):
        if not raw.strip():
            continue
        content, disabled = _strip_disabled(raw)
        m = RE_ITEM.match(content)
        if not m:
            items.append({"id": "r%d" % idx, "kind": "raw",
                          "lines": [raw], "edited": False})
            continue

        macro, argstr = m.group(1), m.group(2)
        tag = (m.group(3) or m.group(4) or "").strip()
        args = [a.strip() for a in argstr.split(",")]

        def _f(i):
            return _param_float(args[i]) if i < len(args) else 0.0

        def _i(i):
            return _param_int(args[i]) if i < len(args) else 0

        iid = "i%d" % idx
        if macro == "CH_ACT_MOVE":
            hold = _i(5) if len(args) > 5 else 0
            items.append({"id": iid, "kind": "move", "lines": [raw], "tag": tag,
                          "start": _i(0), "wx": _f(1), "wy": _f(2),
                          "speed": _i(3), "accel": _i(4),
                          "hold": bool(hold), "disabled": disabled, "edited": False})
        elif macro == "CH_ACT_TURN":
            n = len(args)
            items.append({"id": iid, "kind": "turn", "lines": [raw], "tag": tag,
                          "start": _i(0), "dur": _i(1), "delta": _i(2),
                          "speed": _i(3),
                          # 第 5 参「绝对」/第 6 参「加速度」(2026-10 加, 都追加在末尾):
                          # 老行缺省 abs=0=相对、accel=0=自动。
                          # ⚠️ 参个数不足 6 的旧行标 edited=True: 固件宏现在要 6 参,
                          #    原样回吐编译不过 —— 下一次保存时按当前字段重渲染,
                          #    无损迁移（abs/accel 缺省值 = 原行为）。
                          "abs": (_i(4) if n > 4 else 0),
                          "accel": (_i(5) if n > 5 else 0),
                          "edited": n < 6,
                          "disabled": disabled})
        elif macro == "CH_ACT_PIVOT":
            # 支点转弯段（2026-10 加）: 与 turn 同形（delta/speed/abs）,
            # dur 恒 0（无段上限）、accel 恒 0（驱动侧不插值, 闭环自己平滑）
            items.append({"id": iid, "kind": "pivot", "lines": [raw], "tag": tag,
                          "start": _i(0), "dur": 0, "delta": _i(1),
                          "speed": _i(2), "abs": (_i(3) if len(args) > 3 else 0),
                          "accel": 0, "disabled": disabled, "edited": False})
        elif macro == "CH_ACT_ARM":
            am = RE_ARM_ID.match(args[1]) if len(args) > 1 else None
            arm_id = int(am.group(1)) if am else 0
            items.append({"id": iid, "kind": "arm", "lines": [raw], "tag": tag,
                          "start": _i(0), "arm_id": arm_id,
                          "disabled": disabled, "edited": False})
        else:   # CH_ACT_ARM_WAIT
            items.append({"id": iid, "kind": "arm_wait", "lines": [raw], "tag": tag,
                          "start": _i(0), "dur": _i(1),
                          "disabled": disabled, "edited": False})
    return items


def parse_chact():
    """→ {version, next_id, groups, trash}。groups 顺序 = 聚合初始化的顺序。"""
    c_text = ACTION_C.read_text(encoding="utf-8")
    h_text = ACTION_H.read_text(encoding="utf-8")
    lines = c_text.split("\n")

    begin, end = _find_region(lines, RE_DATA_BEGIN, RE_DATA_END)
    region = lines[begin:end]

    # 1) 每张条目表: 表名前几行里的备注
    groups_by_table = {}
    i = 0
    while i < len(region):
        m = RE_TABLE_DEF.match(region[i])
        if not m:
            i += 1
            continue
        table = m.group(1)
        j = i + 1
        while j < len(region) and not region[j].strip().startswith("};"):
            j += 1
        note = ""
        for k in range(i - 1, max(-1, i - 3), -1):
            nn = RE_NOTE_LINE.match(region[k])
            if nn:
                note = nn.group(1).strip()
        groups_by_table[table] = {"body": region[i + 1:j], "note": note}
        i = j + 1

    # 2) 聚合初始化: (id, 标题, 表名, 条目数) 按顺序。匹配不上必须报错（同 arm 的理由:
    #    静默跳过 = 那个组从界面消失, 保存时数据区整段重生成 → 表被无声删掉）
    entries, k = [], 0
    while k < len(region):
        if not RE_AGGREGATE.match(region[k]):
            k += 1
            continue
        k += 1
        while k < len(region) and not region[k].strip().startswith("};"):
            line = region[k]
            k += 1
            if not line.strip():
                continue
            em = RE_ENTRY.match(line)
            if not em:
                raise ChActError(
                    "无法解析聚合初始化里的这一行（期望 { id, \"标题\", 表名, 数量 },）: %s"
                    % line.strip())
            entries.append({"id": int(em.group(1)), "title": em.group(2),
                            "table": em.group(3), "count": int(em.group(4))})
        break

    # 3) 组装: 聚合条目 × 条目表
    groups = []
    for e in entries:
        tbl = groups_by_table.pop(e["table"], None)
        if tbl is None:
            raise ChActError("聚合初始化引用了不存在的表: %s" % e["table"])
        groups.append({
            "id": e["id"], "title": e["title"], "note": tbl["note"],
            "table": e["table"], "count": e["count"],
            "items": _items_from_body(tbl["body"]),
        })
    if groups_by_table:
        # 聚合里没有的孤儿表: 保留成 raw 组会在保存时丢掉 —— 报出来让人处理
        raise ChActError("数据区里有表没进聚合初始化: %s" % ", ".join(groups_by_table))

    # 4) 头部生成块: 下一个可用 ID
    next_id = 1
    for ln in h_text.split("\n"):
        nm = RE_NEXT_ID.match(ln)
        if nm:
            next_id = int(nm.group(1))
            break

    return {
        "version": {"c": ACTION_C.stat().st_mtime, "h": ACTION_H.stat().st_mtime},
        "next_id": next_id,
        "groups": groups,
        "trash": load_trash(),
    }


# ==================== 序列化 ====================

def _table_of(group):
    return group.get("table") or ("g_astChActItems%d" % int(group.get("id") or 1))


def _fmt_f(value):
    """float → map.c 同款字面量（一位小数 + f 后缀）。"""
    return "%.1ff" % float(value)


def _render_item(it):
    """一条条目 → C 行。"""
    kind = it.get("kind")
    if kind == "move":
        row = "CH_ACT_MOVE     (%du, %s, %s, %d, %d, %uu)" % (
            int(it.get("start", 0)), _fmt_f(it.get("wx", 0.0)), _fmt_f(it.get("wy", 0.0)),
            int(it.get("speed", 0)), int(it.get("accel", 0)),
            1 if it.get("hold") else 0)
    elif kind == "turn":
        # dur 槽已废弃（2026-10 取消段上限, 转到位为止）—— 渲染恒写 0,
        # 固件忽略; 老行里残留的非 0 值在未编辑行上原样保留（零 diff）,
        # 编辑过一次即被冲成 0。
        row = "CH_ACT_TURN     (%du, 0u, %d, %d, %uu, %du)" % (
            int(it.get("start", 0)),
            int(it.get("delta", 0)), int(it.get("speed", 0)),
            1 if it.get("abs") else 0,
            int(it.get("accel", 0) or 0))
    elif kind == "arm":
        # ARM_ACT_<n> 是**枚举成员**（无 u 后缀 —— map.c 全部引用皆如此）; 渲染成
        # ARM_ACT_11u 是另一个标识符 = Keil 报 undefined identifier（2026-10-31 修）
        row = "CH_ACT_ARM      (%du, ARM_ACT_%d)" % (
            int(it.get("start", 0)), int(it.get("arm_id", 0)))
    elif kind == "pivot":
        # 支点转弯段（2026-10 加）: dur/accel 恒 0（无段上限、驱动侧不插值）
        row = "CH_ACT_PIVOT     (%du, %d, %d, %uu)" % (
            int(it.get("start", 0)),
            int(it.get("delta", 0)), int(it.get("speed", 0)),
            1 if it.get("abs") else 0)
    elif kind == "arm_wait":
        row = "CH_ACT_ARM_WAIT (%du, %du)" % (
            int(it.get("start", 0)), int(it.get("dur", 0)))
    else:
        return list(it.get("lines") or [])

    tag = (it.get("tag") or "").strip()
    if tag:
        return ["    %s,  /* %s */" % (row, map_codec.sanitize_comment_text(tag, 60, "备注"))]
    return ["    %s," % row]


def _serialize_items(items):
    """条目列表 → 数组体行。未编辑的按原始物理行回吐（零 diff 的来源）;
    禁用 = 行首 `//`（map_codec._toggle_disable, 与 map.c 单步禁用同一规则）。
    raw 不套 toggle（没有眼睛, 行本身可能就是手写注释）。"""
    out = []
    for it in items or []:
        if it.get("kind") == "raw":
            out.extend(list(it.get("lines") or []))
            continue
        if not it.get("edited") and it.get("lines"):
            lines = list(it["lines"])
        else:
            lines = _render_item(it)
        out.extend(map_codec._toggle_disable(lines, bool(it.get("disabled"))))
    return out


def _active_line_count(body):
    """数组体里真正的初始化项行数 = 固件的 usCount（禁用行是注释, 不进数组）。

    只认条目宏形状（RE_ITEM）: 手写的普通注释行会被解析器收成 raw 条目原样回吐,
    但它们不是初始化项 —— 数进去固件会按 usCount **越界读表**（2026-10-31 修,
    与 arm_action._active_line_count 同一条纪律）。"""
    return sum(1 for ln in body if RE_ITEM.match(ln))


def serialize_group(group):
    """一个组 → (完整行块, C 表行数)。"""
    body = _serialize_items(group.get("items"))
    note = _sanitize_note(group.get("note", ""))
    lines = [
        "/* ===== 预设: %s ===== */" % sanitize_comment(group.get("title", "")),
        "/* 备注: %s */" % note if note else "/* 备注: */",
        "const stChActItemTdf %s[] = {" % _table_of(group),
    ]
    lines.extend(body)
    lines.append("};")
    lines.append("")
    return lines, _active_line_count(body)


def _serialize_aggregate(groups, counts):
    out = ["const stChActGroupTdf g_astChActGroup[CH_ACT_GROUP_NUM] = {"]
    for g, n in zip(groups, counts):
        out.append('    { %du, "%s", %s, %du },'
                   % (g["id"], sanitize_comment(g.get("title", "")), _table_of(g), n))
    out.append("};")
    return out


def serialize_data_region(groups):
    out, counts = [""], []
    for g in groups:
        block, n = serialize_group(g)
        out.extend(block)
        counts.append(n)
    out.extend(_serialize_aggregate(groups, counts))
    out.append("")
    return out


def serialize_header_block(groups, next_id):
    """生成块内容（不含标记行）。next_id 只增不减（同 ARM_ACTION_NEXT_ID 的理由）。"""
    next_id = max(int(next_id or 1), max([g["id"] for g in groups] + [0]) + 1)
    out = [
        "/* 下一个可用 ID。只增不减、删除不回收 —— 保证已经发给控制台的 id 不会在下次",
        " * 保存后指向另一个动作组（同 ARM_ACTION_NEXT_ID 的坑）。 */",
        "#define CH_ACT_NEXT_ID    %du" % next_id,
        "",
        "/* 动作组 ID: 路线表的 STEP_CHASSIS_ACTION_PLAY_WAIT 用它引用; 控制台「动作组」",
        " * 参数的下拉选项**取自本枚举**, 成员尾注释 = 用户起的标题。",
        " * 绑定条件 = app_script.c 里该步骤的 case 体写了 (emChActIdTdf)I(0)。",
        " *",
        " * ⚠️ 本枚举是**有意**被 server.py::parse_enums 扫到的。别改成 #define ——",
        " *    那样「动作组」参数就没有下拉了。 */",
        "typedef enum {",
    ]
    for g in groups:
        out.append("    CH_ACT_%d = %d,   /* %s */"
                   % (g["id"], g["id"], sanitize_comment(g.get("title", ""))))
    out.append("} emChActIdTdf;")
    out.append("")
    out.append("/* 预设张数（= g_astChActGroup[] 的长度, 控制台同步维护） */")
    out.append("#define CH_ACT_GROUP_NUM        %du" % len(groups))
    return out


# ==================== 回收站（同 arm_trash.json 的口径） ====================

def load_trash():
    try:
        data = json.loads(TRASH_JSON.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    items = data.get("items") if isinstance(data, dict) else None
    return items if isinstance(items, list) else []


def save_trash(items):
    items = (items or [])[:TRASH_MAX]
    tmp = TRASH_JSON.with_suffix(".json.tmp")
    TRASH_JSON.parent.mkdir(parents=True, exist_ok=True)
    tmp.write_text(json.dumps({"items": items}, ensure_ascii=False, indent=2),
                   encoding="utf-8")
    os.replace(tmp, TRASH_JSON)


def _next_trash_id(items):
    used = set()
    for it in items:
        m = re.match(r"^t(\d+)$", str(it.get("trash_id") or ""))
        if m:
            used.add(int(m.group(1)))
    n = 1
    while n in used:
        n += 1
    return "t%d" % n


def _trash_payload(group):
    items = []
    for it in group.get("items") or []:
        if it.get("kind") == "raw":
            continue
        copy = {k: v for k, v in it.items() if k != "lines"}
        copy["edited"] = True
        items.append(copy)
    return items


# ==================== 校验 / 发号 ====================

def _check_groups(groups, arm_ids):
    """C 与固件的硬约束（固件播放器还有一层防呆, 这里拦在写盘前）。"""
    if not groups:
        return "至少需要保留一个动作组（空的是空初始化列表, C 编译不过）"
    seen = {}
    for g in groups:
        gid = g.get("id")
        if gid:
            if gid in seen:
                return ("两条动作组用了同一个 id %s（「%s」与「%s」）"
                        % (gid, seen[gid], g.get("title") or "?"))
            seen[gid] = g.get("title") or "?"
    consts = config_consts()
    for g in groups:
        title = g.get("title") or "?"
        items = g.get("items") or []
        if not items:
            return "动作组「%s」没有任何条目（空数组体编译不过）" % title
        if _active_line_count(_serialize_items(items)) == 0:
            return ("动作组「%s」的条目**全被禁用**了（禁用的行是注释, 不进数组, "
                    "空数组体编译不过）—— 至少留一条启用" % title)
        if len(items) > consts["items_max"]:
            return ("动作组「%s」条目数 %d 超过固件上限 %d（CH_ACT_ITEMS_MAX）"
                    % (title, len(items), consts["items_max"]))
        for it in items:
            kind = it.get("kind")
            name = it.get("tag") or kind
            if kind not in ("move", "turn", "pivot", "arm", "arm_wait", "raw"):
                return "未知的条目类型: %s" % kind
            if kind == "raw":
                if not it.get("lines"):
                    return "只读块必须带原始行, 否则会丢失"
                continue
            if it.get("start", 0) < 0:
                return "「%s」的起点不能是负数" % name
            if kind == "move":
                if abs(_to_float(it.get("wx"))) + abs(_to_float(it.get("wy"))) < 0.5:
                    return "平移段「%s」没填位移（wx/wy 至少一个 ≥0.5mm）" % name
                if not (0 < int(it.get("speed") or 0) <= SPEED_MAX):
                    return "平移段「%s」的速度须在 1~%d rpm" % (name, SPEED_MAX)
                if not (0 < int(it.get("accel") or 0) <= ACCEL_MAX):
                    return "平移段「%s」的加速度须在 1~%d（EMM 单字节档）" % (name, ACCEL_MAX)
            elif kind in ("turn", "pivot"):
                delta = int(it.get("delta") or 0)
                is_abs = bool(it.get("abs"))
                kind_name = "转弯段" if kind == "turn" else "支点转弯段"
                if delta % 90 != 0 or abs(delta) > 180:
                    return ("%s「%s」的转角必须是 ±90 的倍数且不超 ±180"
                            "（世界航向是 90° 网格的记账器）" % (kind_name, name))
                if delta == 0 and not is_abs:
                    return ("%s「%s」相对模式不允许 0°（不转就别排）; "
                            "要咬回开机朝向请选「绝对」模式" % (kind_name, name))
                acc = int(it.get("accel") or 0)
                if acc < 0 or acc > ACCEL_MAX:
                    return ("%s「%s」的加速度须在 0~%d（0=自动: 沿用平移段/全局档）"
                            % (kind_name, name, ACCEL_MAX))
            elif kind == "arm":
                if int(it.get("arm_id") or 0) not in arm_ids:
                    return ("臂事件「%s」引用的动作 ARM_ACT_%s 不存在"
                            "（先去「机械臂」Tab 建好再排）"
                            % (name, it.get("arm_id") or 0))
    return ""


def _assign_ids(groups, next_id):
    used = set(int(g["id"]) for g in groups if g.get("id"))
    next_id = max(int(next_id or 1), max(used) + 1 if used else 1)
    for g in groups:
        if not g.get("id"):
            while next_id in used:
                next_id += 1
            g["id"] = next_id
            used.add(next_id)
            next_id += 1
    return groups, next_id


# ==================== 写盘 ====================

def _replace_region(lines, begin_re, end_re, body):
    b, e = _find_region(lines, begin_re, end_re)
    return lines[:b] + list(body) + lines[e:]


WRITE_LOCK = threading.RLock()


def _serialized(fn):
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        with WRITE_LOCK:
            return fn(*args, **kwargs)
    return wrapper


def _write_files_locked(groups, next_id):
    c_text = ACTION_C.read_text(encoding="utf-8")
    h_text = ACTION_H.read_text(encoding="utf-8")

    new_c_text = "\n".join(_replace_region(
        c_text.split("\n"), RE_DATA_BEGIN, RE_DATA_END,
        serialize_data_region(groups)))
    new_h_text = "\n".join(_replace_region(
        h_text.split("\n"), RE_GEN_BEGIN, RE_GEN_END,
        serialize_header_block(groups, next_id)))

    if new_c_text != c_text:
        shutil.copy2(ACTION_C, ACTION_C.with_suffix(".c.bak"))
        map_codec.write_text_like(ACTION_C, new_c_text)
    if new_h_text != h_text:
        shutil.copy2(ACTION_H, ACTION_H.with_suffix(".h.bak"))
        map_codec.write_text_like(ACTION_H, new_h_text)

    changed = (new_c_text != c_text) or (new_h_text != h_text)
    return changed, {"c": ACTION_C.stat().st_mtime, "h": ACTION_H.stat().st_mtime}


def _version_ok(base_version):
    if base_version is None:
        return True
    try:
        return (abs(float(base_version.get("c", 0)) - ACTION_C.stat().st_mtime) < 1e-6
                and abs(float(base_version.get("h", 0)) - ACTION_H.stat().st_mtime) < 1e-6)
    except (AttributeError, TypeError, ValueError, OSError):
        return False


CONFLICT = (False, {"error": "conflict",
                    "message": "app_chassis_action 已在外部被修改, 请先「重新加载」再保存"},
            409)


def _commit(groups, next_id, extra):
    groups, next_id = _assign_ids(list(groups), next_id)
    err = _check_groups(groups, _arm_ids())
    if err:
        return False, {"error": "invalid", "message": err}, 400
    changed, version = _write_files_locked(groups, next_id)
    resp = {"ok": True, "version": version, "changed": changed}
    resp.update(extra or {})
    return True, resp, 200


def _find_group(groups, gid):
    for g in groups:
        if int(g.get("id") or 0) == int(gid or 0):
            return g
    return None


def _arm_ids():
    """当前机械臂动作预设的 id 集合（校验 ARM 事件引用用）。延迟 import 防循环。"""
    import arm_action
    try:
        return set(int(p["id"]) for p in arm_action.parse_arm().get("presets", []))
    except Exception:                                        # noqa: BLE001
        return set()


# ==================== 动作分发 ====================

@_serialized
def save(payload, base_version):
    """整份组列表写盘。未编辑条目靠原始行回吐保证零 diff。"""
    if not _version_ok(base_version):
        return CONFLICT
    groups = payload.get("groups")
    if not isinstance(groups, list):
        return False, {"error": "invalid", "message": "groups 必须是列表"}, 400

    consts = config_consts()
    clean = []
    for g in groups:
        q = dict(g)
        q.pop("table", None)
        q["title"] = sanitize_comment(q.get("title", ""))
        items = []
        for it in (q.get("items") or []):
            kind = it.get("kind")
            if kind not in ("move", "turn", "pivot", "arm", "arm_wait", "raw"):
                return False, {"error": "invalid",
                               "message": "未知的条目类型: %s" % kind}, 400
            if kind == "raw":
                if not it.get("lines"):
                    return False, {"error": "invalid",
                                   "message": "只读块必须带原始行, 否则会丢失"}, 400
                items.append(dict(it))
                continue
            c = dict(it)
            c["tag"] = map_codec.sanitize_comment_text(c.get("tag", ""), 60, "备注")
            c["start"] = snap_ms(c.get("start"), consts["loop_ms"])   # 吸附到节拍网格
            items.append(c)
        q["items"] = items
        clean.append(q)

    cur = parse_chact()
    ok, resp, status = _commit(clean, cur["next_id"], {})
    if ok:
        resp["groups"] = parse_chact()["groups"]
    return ok, resp, status


@_serialized
def delete_group(gid, base_version):
    """删组: 先 commit 后进回收站（同 arm 2026-10-09 的调序理由）。

    ⚠️ map.c 里若还有 CH_ACT_<id> 的引用, 删完会编译期报错 —— 故意的（胜过静默
    指向另一个组）。调用方应先问引用数（server 的 chact_refs）弹确认。
    """
    if not _version_ok(base_version):
        return CONFLICT
    cur = parse_chact()
    groups = cur["groups"]
    g = _find_group(groups, gid)
    if g is None:
        return False, {"error": "not found", "message": "没有这个动作组"}, 404
    if len(groups) <= 1:
        return False, {"error": "last", "message": "至少要留一个动作组"}, 400

    ok, resp, status = _commit([x for x in groups if x["id"] != g["id"]],
                               cur["next_id"], {})
    if not ok:
        return ok, resp, status

    trash = load_trash()
    entry = {"trash_id": _next_trash_id(trash), "id": g["id"], "title": g["title"],
             "note": g["note"],
             "deleted_at": time.strftime("%Y-%m-%d %H:%M"),
             "items": _trash_payload(g)}
    save_trash([entry] + trash)
    resp["trash_id"] = entry["trash_id"]
    return ok, resp, status


@_serialized
def restore_group(trash_id, base_version):
    """从回收站恢复: 发**新 ID** + 「原名恢复(x)」, 追加到末尾（同 arm 的理由）。"""
    if not _version_ok(base_version):
        return CONFLICT
    trash = load_trash()
    entry = next((t for t in trash if str(t.get("trash_id")) == str(trash_id)), None)
    if entry is None:
        return False, {"error": "not found", "message": "回收站里没有这一条"}, 404

    cur = parse_chact()
    groups = cur["groups"]
    if not entry.get("items"):
        return False, {"error": "empty", "message": "这一条没有条目, 恢复不了"}, 400

    new_g = {
        "title": restore_title(entry.get("title", ""),
                               set(g["title"] for g in groups)),
        "note": entry.get("note", ""),
        "items": [dict(it) for it in entry["items"]],
    }
    ok, resp, status = _commit(groups + [new_g], cur["next_id"], {})
    if ok:
        save_trash([t for t in trash if str(t.get("trash_id")) != str(trash_id)])
        resp["groups"] = parse_chact()["groups"]
        resp["title"] = new_g["title"]
    return ok, resp, status


@_serialized
def purge_trash(trash_id):
    trash = load_trash()
    left = [t for t in trash if str(t.get("trash_id")) != str(trash_id)]
    save_trash(left)
    return True, {"ok": True, "trash": left}, 200


def run_chact_action(action, payload):
    """POST /api/chact 的分发（save/delete/restore/purge）。契约同 run_arm_action。"""
    base = payload.get("base_version")
    try:
        if action == "save":
            ok, data, status = save(payload, base)
        elif action == "delete":
            ok, data, status = delete_group(payload.get("id"), base)
        elif action == "restore":
            ok, data, status = restore_group(payload.get("trash_id"), base)
        elif action == "purge":
            ok, data, status = purge_trash(payload.get("trash_id"))
        else:
            return False, None, "未知动作: %s" % action, 400
        # HTTP 状态码透传（同 run_arm_action; 前端靠 HTTP 409 触发重载提示条）
        if ok:
            return True, data, "", 200
        return False, data, data.get("message") or data.get("error") or "操作失败", status
    except ChActError as exc:
        return False, None, str(exc), 400
    except Exception as exc:                                 # noqa: BLE001
        return False, None, "内部错误: %s" % exc, 500


def state():
    """GET /api/chact 的响应体。手改坏时降级成带 error 的空壳, 不让页面 500。"""
    try:
        st = parse_chact()
    except (ChActError, OSError) as exc:
        st = {"error": str(exc), "groups": [], "trash": [],
              "next_id": 1, "version": {"c": 0, "h": 0}}
    st["consts"] = config_consts()
    try:
        import arm_action
        st["arm_options"] = [{"id": int(p["id"]), "title": p.get("title", "")}
                             for p in arm_action.parse_arm().get("presets", [])]
    except Exception:                                        # noqa: BLE001
        st["arm_options"] = []
    return st
