#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂「动作预设」表 —— app_arm_action.c / .h 的解析、序列化、回收站。

一个「动作预设」= 一串子积木（位姿 / 夹爪 / 延时）= 台架上录的一段臂动作。
保存后它在「地图」积木库里显示成**一块大积木**（服务端在 parse_catalog 里虚拟出来,
见 server.py），拖进路线就是一步 STEP_ARM_PLAY_WAIT / _NO_WAIT。

文件布局（两个文件都由控制台维护, 但格式固定、可手改）:

  app_arm_action.h   生成块（`/* ==== 生成块: 开始 ==== */` 之间）
                     = ARM_ACTION_NEXT_ID + emArmActionIdTdf + ARM_ACTION_NUM
                     其余（结构体、API 声明）是手写的, 控制台不碰
  app_arm_action.c   数据区（`/* ==== 数据区: 开始 ==== */` 之间）
                     = 每预设一个步骤表 + 末尾 g_astArmAction[] 聚合初始化
                     其余（包含、播放实现）是手写的, 控制台不碰

铁律（与 map.c 同款, 也是本项目对「回写源码」的统一要求）:
  · **未编辑的条目按原始物理行回吐** → 无操作保存 = 两个文件都字节级 diff 为空
  · 内容没变就**一个字节都不写**（连 .bak 都不建）—— 这里同时管两个文件、又拿 mtime
    当版本戳, 白写一次会让另一个 Tab 的下次保存吃 409
  · 写盘前 `.bak` 备份

⚠️ 预设的**表头三行**（`预设:` / `播放方式:` / `备注:`）是无条件重新生成的,
   手改这三行的排版会在下次保存时被规范回去; 步骤表内部（数据行）则按原始行保留。

⚠️ 播放方式的**权威是聚合初始化行里的第 3 段**（= 固件 `stArmActionTdf::ucNoWait`）,
   表头那行 `播放方式:` 注释由同一个值渲染 —— 手改注释不会改变实际行为（固件只认字段）,
   下次保存会被规范回字段值。旧文件（4 段聚合行）仍能读: 那时 `nowait` 从注释取
   （= 老行为）, 保存一次即迁移成 5 段。

⚠️ 子积木的「眼睛」（禁用）= 行首加 `//`, 与 map.c 的单步禁用同一套规则
   （map_codec._toggle_disable）。被禁用的行是**注释、不进数组**, 所以聚合初始化里
   的条目数必须用 `_active_line_count` 扣掉它们（map.c 靠 sizeof 自动取长度, 没这问题）。

⚠️ 本模块**不 import server**（server 要 import 本模块, 会成为循环）。catalog 与
   map.c 的引用计数都由调用方传进来 —— 那是 server 的地盘。
"""

import functools
import json
import os
import re
import shutil
import threading
import time
from pathlib import Path

import bt_link
import map_codec

APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parents[1]

# 模块级常量: 测试用 mock.patch.object 沙箱化（照 test_tuning.py::Sandbox）
ACTION_C = ROOT / "Application" / "app_arm_action.c"
ACTION_H = ROOT / "Application" / "app_arm_action.h"
TRASH_JSON = APP_DIR / "arm_trash.json"

# 「离线关节表」的源码来源（见 offline_joints）—— 元数据一律从源码派生, 前端不写死
ARM_C = ROOT / "Application" / "app_arm.c"
ARM_CONFIG_H = ROOT / "Application" / "app_config.h"
PROJECT_CONFIG_H = ROOT / "Core" / "Inc" / "project_config.h"

# 数据区/生成块标记（控制台只替换这两个标记之间的内容）
RE_DATA_BEGIN = re.compile(r"^\s*/\*\s*=+\s*数据区\s*[:：]\s*开始\s*=+\s*\*/\s*$")
RE_DATA_END = re.compile(r"^\s*/\*\s*=+\s*数据区\s*[:：]\s*结束\s*=+\s*\*/\s*$")
RE_GEN_BEGIN = re.compile(r"^\s*/\*\s*=+\s*生成块\s*[:：]\s*开始\s*=+\s*\*/\s*$")
RE_GEN_END = re.compile(r"^\s*/\*\s*=+\s*生成块\s*[:：]\s*结束\s*=+\s*\*/\s*$")

RE_STEPS_DEF = re.compile(r"^\s*const\s+stScriptStepTdf\s+(\w+)\s*\[\s*\]\s*=\s*\{\s*$")
RE_AGGREGATE = re.compile(
    r"^\s*const\s+stArmActionTdf\s+g_astArmAction\s*\[\s*ARM_ACTION_NUM\s*\]\s*=\s*\{\s*$")
RE_ENTRY = re.compile(
    r'^\s*\{\s*(\d+)u\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*(\d+)u\s*,\s*(\w+)\s*,\s*(\d+)u\s*\}\s*,?\s*$')
# 旧格式（没有播放方式字段）: { id, "标题", 表名, 数量 }, —— 仍读得进来, 保存时迁移成上面那种
RE_ENTRY_LEGACY = re.compile(
    r'^\s*\{\s*(\d+)u\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*(\w+)\s*,\s*(\d+)u\s*\}\s*,?\s*$')
RE_MODE_LINE = re.compile(r"^\s*/\*\s*播放方式\s*[:：]\s*(.+?)\s*\*/\s*$")
RE_NOTE_LINE = re.compile(r"^\s*/\*\s*备注\s*[:：]\s*(.*?)\s*\*/\s*$")
RE_NEXT_ID = re.compile(r"^\s*#define\s+ARM_ACTION_NEXT_ID\s+(\d+)u\s*$")
# 条目行形状（SCRIPT_STEP_* 宏开头的初始化行）—— _active_line_count 用它把
# 手写的普通注释行/raw 行排除在 usCount 之外（数进去 = 固件越界读表）
RE_ITEM_LINE = re.compile(r"^\s*SCRIPT_STEP\w*\s*\(")

MODE_WAIT = "阻塞到位"
MODE_NOWAIT = "启动即返回"

# 动作序列里允许出现的步骤（白名单）。写进 C 的东西必须是 vArmPlayWait 认得的,
# 否则那条会在固件里被静默跳过。
ALLOWED_STEPS = {
    "STEP_ARM_GROUP_WAIT": "pose",
    "STEP_ARM_GROUP_NO_WAIT": "pose",
    "STEP_SET_ZHUASHOU_WAIT": "grip",
    "STEP_DELAY": "delay",
}

MAX_TITLE_LEN = 24
TRASH_MAX = 50


class ArmError(Exception):
    """可直接展示给用户的错误。"""


# ==================== 纯函数: 数值 / 文本 ====================

def _to_float(text, default=0.0):
    try:
        return float(text)
    except (TypeError, ValueError):
        return default


def _param_float(token):
    """'90.0f' / '300u' / '-30' → float（剥掉 C 的 f/F 与 u/U/l/L 后缀）。

    先试原样解析, 失败才剥后缀 —— 否则 '0x1f' 会被剥成 '0x1' 读错。
    """
    t = (token or "").strip()
    try:
        return float(t)
    except ValueError:
        pass
    return _to_float(t.rstrip("fFuUlL"))


def _param_int(token):
    return int(round(_param_float(token)))


def sanitize_comment(text):
    """洗掉会破坏 C 注释的字符。

    标题/备注会同时进: ① `/* 备注: xxx */` 行注释  ② 枚举成员尾注释 `/* xxx */`。
    规则本体已收敛到 map_codec.sanitize_comment_text（**唯一来源**, 那边有完整理由）,
    这里只补上 arm 这条路的两个约定: 长度上限 MAX_TITLE_LEN, 与空标题的兜底
    （清空标题动作会得到一个空名字, 列表里那一行就成了空白 —— 沿用旧行为）。
    """
    return map_codec.sanitize_comment_text(text, MAX_TITLE_LEN, "臂组动作")


def _sanitize_note(text):
    """备注不清洗长度（可以长一点）, 但同样不能破坏注释与行结构。"""
    return map_codec.sanitize_block_comment(text)


RE_RESTORE_SUFFIX = re.compile(r"恢复(?:\(\d+\))?$")


# ==================== 离线关节表（没连设备时把滑杆画成灰色） ====================
# 关节滑杆的**真值来自设备上报**（$B,LIST 的关节表: 5 个轴的角度范围 + 当前角度）。
# 但没连设备时给一块空面板会让人以为「这功能没做」—— 所以这里从**源码**派生一份
# 占位表, 前端照着把滑杆画出来但**禁用**。
#
# 数据来源（都不在前端写死）:
#   app_arm.c        s_aemArmGroup[]（组顺序）/ s_apcArmJointName[]（关节名）/
#                    s_fArmPoseInit[]（上电默认角）/ 哪个轴走 vServoDeviceInit_Angle360At
#   app_config.h     #define ARM_SERVOn emServoDevNumN（servo → 上电角表的下标）
#   project_config.h SERVO_DEFAULT_ANGLE_MIN / _MAX(180) / _360_MAX

RE_ARM_GROUP = re.compile(r"s_aemArmGroup\s*\[\s*ARM_GROUP_NUM\s*\]\s*=\s*\{([^}]*)\}")
RE_JOINT_NAMES = re.compile(r"s_apcArmJointName\s*\[\s*ARM_GROUP_NUM\s*\]\s*=\s*\{([^}]*)\}")
RE_POSE_INIT = re.compile(r"s_fArmPoseInit\s*\[\s*SERVO_DEV_NUM\s*\]\s*=\s*\{([^}]*)\}")
RE_INIT_360 = re.compile(r"vServoDeviceInit_Angle360At\s*\(\s*(\w+)\s*,")
RE_SERVO_DEV = re.compile(r"#define\s+(ARM_SERVO\d)\s+emServoDevNum(\d+)")
RE_ANGLE_DEF = re.compile(r"#define\s+(SERVO_DEFAULT_ANGLE\w*)\s+([-\d.]+)")
RE_ZHUASHOU_ANGLE = re.compile(r"#define\s+(ZHUASHOU_(?:OPEN|CLOSE)_ANGLE)\s+([-\d.]+)")


def offline_joints():
    """→ [{index, name, min, max, angle}]（5 个组轴, 与设备上报的关节表同形）。

    解析不出来就回 []（前端会退回那句提示文字）。**这不是设备真值** —— 一连上就会被
    设备上报的表整体替换, 所以它只求「形状与量程对得上」, 免得连线前后滑杆跳一下。
    """
    try:
        arm_c = ARM_C.read_text(encoding="utf-8")
        cfg_h = ARM_CONFIG_H.read_text(encoding="utf-8")
        pc_h = PROJECT_CONFIG_H.read_text(encoding="utf-8")
    except OSError:
        return []

    grp = RE_ARM_GROUP.search(arm_c)
    names_m = RE_JOINT_NAMES.search(arm_c)
    pose_m = RE_POSE_INIT.search(arm_c)
    if not (grp and names_m and pose_m):
        return []

    servos = re.findall(r"ARM_SERVO\d+", grp.group(1))
    names = re.findall(r'"([^"]*)"', names_m.group(1))
    if not servos or len(servos) != len(names):
        return []

    # ⚠️ 必须用 _param_float: 表里是 C 字面量 90.0f, 裸 float() 会失败并静默回退成 0
    #    （那样 5 个滑杆的初始位置全是 0, 看着像「位姿全归零」）
    pose = [_param_float(t) for t in
            re.sub(r"/\*.*?\*/", "", pose_m.group(1), flags=re.S).split(",") if t.strip()]
    dev_index = {m.group(1): int(m.group(2)) for m in RE_SERVO_DEV.finditer(cfg_h)}
    is360 = set(RE_INIT_360.findall(arm_c))
    consts = {m.group(1): _to_float(m.group(2)) for m in RE_ANGLE_DEF.finditer(pc_h)}

    lo = consts.get("SERVO_DEFAULT_ANGLE_MIN", 0.0)
    hi_180 = consts.get("SERVO_DEFAULT_ANGLE_MAX", 180.0)
    hi_360 = consts.get("SERVO_DEFAULT_ANGLE_360_MAX", 360.0)

    out = []
    for i, dev in enumerate(servos):
        k = dev_index.get(dev)
        out.append({
            "index": i,
            "name": names[i],
            "min": lo,
            "max": hi_360 if dev in is360 else hi_180,
            "angle": pose[k] if (k is not None and k < len(pose)) else lo,
        })
    return out


def gripper_span():
    """→ {'open', 'close', 'min', 'max'} 或 None。

    夹爪**不进**关节表（设备上报的 5 轴里没有它），所以「获取当前位姿」拿到的是一个
    **角度**，而控件是「开合 + 程度%」—— 反解需要 OPEN/CLOSE 两个端点。它们是编译进
    固件的常量、设备不上报，于是和 joints_offline 同一条路：从源码派生。

    解析不出来回 None（前端会跳过夹爪那一步并留一句提示），不抛异常。
    """
    try:
        cfg_h = ARM_CONFIG_H.read_text(encoding="utf-8")
        pc_h = PROJECT_CONFIG_H.read_text(encoding="utf-8")
    except OSError:
        return None

    vals = {m.group(1): _param_float(m.group(2))
            for m in RE_ZHUASHOU_ANGLE.finditer(cfg_h)}
    if ("ZHUASHOU_OPEN_ANGLE" not in vals) or ("ZHUASHOU_CLOSE_ANGLE" not in vals):
        return None

    consts = {m.group(1): _to_float(m.group(2)) for m in RE_ANGLE_DEF.finditer(pc_h)}
    return {
        "open": vals["ZHUASHOU_OPEN_ANGLE"],
        "close": vals["ZHUASHOU_CLOSE_ANGLE"],
        "min": consts.get("SERVO_DEFAULT_ANGLE_MIN", 0.0),
        "max": consts.get("SERVO_DEFAULT_ANGLE_MAX", 180.0),
    }


def restore_title(original, live_titles):
    """被删预设恢复时的取名: 原名恢复 / 原名恢复(1) / 原名恢复(2) …

    对**当前活标题集合**取唯一名（不是对回收站）, 所以连恢复两次也不会撞名。
    原名本身就带「恢复(x)」尾巴的先剥掉再缀 —— 否则「A恢复」再删再恢复会变成
    「A恢复恢复」, 复几次叠几次。
    """
    stem = RE_RESTORE_SUFFIX.sub("", sanitize_comment(original)).strip() or "动作"
    base = "%s恢复" % stem
    if base not in live_titles:
        return base
    k = 1
    while "%s(%d)" % (base, k) in live_titles:
        k += 1
    return "%s(%d)" % (base, k)


# ==================== 解析 ====================

def _find_region(lines, begin_re, end_re):
    """找标记区间, 返回 (begin+1, end) 的 0-based 半开区间; 找不到抛 ArmError。"""
    begin = end = None
    for i, ln in enumerate(lines):
        if begin is None and begin_re.match(ln):
            begin = i
        elif begin is not None and end_re.match(ln):
            end = i
            break
    if begin is None or end is None:
        raise ArmError("app_arm_action 文件缺少数据区/生成块标记（被手改坏了？）")
    return begin + 1, end


def _items_from_body(body):
    """数组体行 → 子积木列表。

    map_codec._parse_body 先把物理行切成 step 条目; 这里再把「位姿 + 紧跟的夹爪」
    合并成一条（界面上看到的一行 = 一个位姿, 夹爪是它的属性）。

    ⚠️ **每条都必须带唯一 `id`**，前端拿它当 DOM id（`blk-arm-<id>`）、选中判据、
       拖拽落点计算的 skip 判据。曾漏掉过 —— 后果是 `data-id` 属性根本没被设上
       （`h()` 会跳过 undefined），于是「点一块选中全部」+「拖动其它块不让位」+
       「删除永远删第一块」，三个症状都来自这一个字段。id 只在**一个预设内**唯一即可。

    `disabled`（行首 `//` 注释掉）由 map_codec._parse_body 认出来（它按条目算
    `all_commented`），这里原样带上去。位姿 + 夹爪合并成一条时取两者的**与**：
    两行都注释了才算这条积木被禁用（与 C 里「这一个初始化项不存在」等价）。
    """
    items = []
    for it in map_codec._parse_body(body):
        iid = it.get("id") or ("i%d" % len(items))
        if it.get("kind") != "step":
            items.append({"id": iid, "kind": "raw", "lines": it.get("lines", []),
                          "edited": False})
            continue

        step = it.get("step", "")
        tag = it.get("tag", "")
        params = it.get("params") or []
        kind = ALLOWED_STEPS.get(step)
        off = bool(it.get("disabled"))

        if kind == "pose" and len(params) >= 5:
            items.append({
                "id": iid, "kind": "pose", "lines": list(it["lines"]), "tag": tag,
                "angles": [_param_float(p) for p in params[:5]],
                "gripper": {"enabled": False, "kai_he": 1, "degree": 100},
                "disabled": off,
                "edited": False,
            })
        elif kind == "grip" and len(params) >= 2:
            prev = items[-1] if items else None
            if prev and prev["kind"] == "pose" and not prev["gripper"]["enabled"]:
                # 位姿自带的夹爪: 并进上一条, 不再单列（id 仍归位姿那条）
                prev["gripper"] = {"enabled": True,
                                   "kai_he": _param_int(params[0]),
                                   "degree": _param_int(params[1])}
                prev["lines"] = prev["lines"] + list(it["lines"])
                # 取**与**: 两行**都**注释了才算这块积木被禁用
                # （test_arm_action.py::TestDisable::test_pose_and_grip_are_disabled_together
                #   钉着这条语义 —— 别改成 or）。
                # ⚠️ 已知的规范化行为（**有意保留**, 改之前先看这里）: 手改 app_arm_action.c
                #    把两行**只注释一行**时, 这块被判成"没禁用", 保存会把另一行也放开
                #    （= 这块变成完全启用）。理由: 半注释的块在 C 里是"只有夹爪没有位姿"的
                #    残缺动作, 本来就不该存在; 而"未编辑条目按原样回吐"要成立, 就得让眼睛
                #    也置 edited（否则眼睛点了不生效 —— 那是更危险的失效方向）。
                #    想让一块不执行, 请把两行一起注释（界面上的眼睛做的就是这件事）。
                prev["disabled"] = prev.get("disabled", False) and off
            else:
                items.append({"id": iid, "kind": "grip", "lines": list(it["lines"]),
                              "tag": tag, "kai_he": _param_int(params[0]),
                              "degree": _param_int(params[1]), "disabled": off,
                              "edited": False})
        elif kind == "delay" and params:
            items.append({"id": iid, "kind": "delay", "lines": list(it["lines"]),
                          "tag": tag, "ms": _param_int(params[0]), "disabled": off,
                          "edited": False})
        else:
            # 白名单外的步骤 / 参数个数不对: 原样留着, 保存时按原行回吐,
            # 界面上当只读块显示（不猜、不丢）。
            items.append({"id": iid, "kind": "raw", "lines": list(it["lines"]),
                          "edited": False, "step": step, "tag": tag})
    return items


def _parse_c_data(presets_by_table, aggregate_entries):
    """把「表名 → 表内容」与「聚合初始化里的条目顺序」拼成预设列表。

    聚合初始化 g_astArmAction[] 是**顺序与 ID 的权威**（固件按它遍历）;
    每个预设块的备注按表名去认领。

    ⚠️ 播放方式的权威是**聚合行的第 3 段**（固件只认它）—— 注释行仅在读旧格式
       （没有该字段）时兜底。手改注释改变不了行为, 保存时会被规范回字段值。
    """
    presets = []
    for ent in aggregate_entries:
        meta = presets_by_table.get(ent["table"])
        if meta is None:
            raise ArmError("g_astArmAction[] 引用了找不到的步骤表 %s" % ent["table"])
        presets.append({
            "id": ent["id"],
            "title": ent["title"],
            "note": meta["note"],
            # 新格式以**字段**为准（固件读的就是它）; 旧格式（None）才回退到表头注释
            "nowait": meta["nowait"] if ent["nowait"] is None else ent["nowait"],
            "table": ent["table"],
            "items": _items_from_body(meta["body"]),
        })
    return presets


def parse_arm():
    """→ {version, next_id, presets, trash}。

    presets 顺序 = g_astArmAction[] 聚合初始化的顺序（不按 ID 排: 界面上拖动排序后,
    顺序就是保存时的顺序）。
    """
    c_text = ACTION_C.read_text(encoding="utf-8")
    h_text = ACTION_H.read_text(encoding="utf-8")
    lines = c_text.split("\n")

    begin, end = _find_region(lines, RE_DATA_BEGIN, RE_DATA_END)
    region = lines[begin:end]

    # 1) 每张步骤表: 表名前几行里的播放方式/备注
    presets_by_table = {}
    i = 0
    while i < len(region):
        m = RE_STEPS_DEF.match(region[i])
        if not m:
            i += 1
            continue
        table = m.group(1)
        j = i + 1
        while j < len(region) and not region[j].strip().startswith("};"):
            j += 1
        nowait, note = False, ""
        for k in range(i - 1, max(-1, i - 5), -1):
            mm = RE_MODE_LINE.match(region[k])
            if mm:
                nowait = (mm.group(1).strip() == MODE_NOWAIT)
                continue
            nn = RE_NOTE_LINE.match(region[k])
            if nn:
                note = nn.group(1).strip()
        presets_by_table[table] = {"body": region[i + 1:j], "nowait": nowait, "note": note}
        i = j + 1

    # 2) 聚合初始化: (id, 标题, 播放方式, 表名, 条目数) 按顺序
    #
    # ⚠️ 匹配不上**必须报错**, 不能静默跳过（原来没有 else）: 少一段/多一段的行会被整条丢掉
    #    → 那个预设从界面上消失, 而保存时数据区是整段重生成的 → 步骤表被删掉, 无声无息。
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
            if em:
                entries.append({"id": int(em.group(1)), "title": em.group(2),
                                "nowait": (em.group(3) != "0"),
                                "table": em.group(4), "count": int(em.group(5))})
                continue
            em = RE_ENTRY_LEGACY.match(line)
            if em:
                # 旧格式（没有播放方式字段）: nowait 留 None, 由步骤表上方那行注释补上
                # （= 老行为）, 保存时统一迁移成带字段的新格式。
                entries.append({"id": int(em.group(1)), "title": em.group(2),
                                "nowait": None,
                                "table": em.group(3), "count": int(em.group(4))})
                continue
            if line.strip().startswith("{"):
                raise ArmError(
                    "无法解析聚合初始化里的这一行（期望 { id, \"标题\", 0u|1u, 表名, 数量 },）: %s"
                    % line.strip())
        break

    # 3) 头部生成块: 下一个可用 ID
    next_id = 1
    for ln in h_text.split("\n"):
        nm = RE_NEXT_ID.match(ln)
        if nm:
            next_id = int(nm.group(1))
            break

    return {
        "version": {"c": ACTION_C.stat().st_mtime, "h": ACTION_H.stat().st_mtime},
        "next_id": next_id,
        "presets": _parse_c_data(presets_by_table, entries),
        "trash": load_trash(),
    }


# ==================== 序列化 ====================

def _table_of(preset):
    return preset.get("table") or ("g_astArmActionSteps%d" % int(preset.get("id") or 1))


def _render_item(it, cat_by_type):
    """一条子积木 → C 行。复用现成生成器, 与手写的 map.c 行逐字符同款。"""
    kind = it.get("kind")
    if kind == "pose":
        grip = it.get("gripper") or {}
        return bt_link.make_arm_snippet([{
            "name": it.get("tag", ""),
            "angles": list(it.get("angles") or [])[:5],
            "gripper": grip if grip.get("enabled") else None,
        }], wait=True)
    if kind == "grip":
        return [bt_link._fmt_gripper_step(it.get("tag", ""),
                                          it.get("kai_he"), it.get("degree"))]
    if kind == "delay":
        return map_codec._serialize_step(
            {"step": "STEP_DELAY", "tag": it.get("tag", ""),
             "params": [str(int(it.get("ms", 0)))]}, cat_by_type)
    return list(it.get("lines") or [])


def _serialize_items(items, cat_by_type):
    """子积木列表 → 数组体行。未编辑的按原始物理行回吐（零 diff 的来源）。

    禁用（眼睛点掉）= 每物理行行首加 `//`，复用 map_codec._toggle_disable ——
    与 map.c 的单步禁用**同一套规则**（所以手写的注释风格在两边都一样）。
    位姿带了夹爪就是两行，两行一起注释：它们在界面上是一条积木，在 C 里也必须
    一起进出数组。

    ⚠️ raw（只读块）不套 _toggle_disable：它没有眼睛，那些行可能是手写的普通
       注释行，`_toggle_disable(.., False)` 会把行首的 `//` 剥掉、无端改字节。
       与 map_codec.serialize_map 一致（那里也只有 step 条目走 toggle）。
    """
    out = []
    for it in items or []:
        if it.get("kind") == "raw":
            out.extend(list(it.get("lines") or []))
            continue
        if not it.get("edited") and it.get("lines"):
            lines = list(it["lines"])
        else:
            lines = _render_item(it, cat_by_type)
        out.extend(map_codec._toggle_disable(lines, bool(it.get("disabled"))))
    return out


def _active_line_count(body):
    """数组体里**真正的初始化项**行数 = 固件的 `usCount`。

    只数**条目宏形状**（RE_ITEM_LINE, 即 SCRIPT_STEP_* 开头）的行：被禁用的条目
    行首有 `//`、raw 条目可能是手写的普通注释行 —— 两者都不是初始化项, 数进去
    会让 `vArmPlayWait` 按 usCount **越界读到表尾别的东西, 对垃圾数据出舵机指令**
    （2026-10-31 修: 旧版只扣 `//` 行, 手写 `/* */` 注释行会把 count 顶大）。
    map.c 靠 `sizeof(SCRIPT_PRESET_ACTIVE)` 自动取长度，没有这个问题；
    这里是显式计数，所以必须自己扣掉。
    """
    return sum(1 for ln in body if RE_ITEM_LINE.match(ln))


def serialize_preset(preset, cat_by_type):
    """一个预设 → (完整行块, C 表行数)。

    行块 = 表头三行 + 步骤表 + 尾随空行。
    ⚠️ 返回的**行数**是给聚合初始化用的 —— 固件按 usCount 遍历 pstSteps[],
    必须等于步骤表里**进得了数组的**物理行数（既不是界面上看到的子积木条数 ——
    一个位姿带夹爪就是两行; 也不是物理行总数 —— 被眼睛禁用的行是注释, 不存在）。
    数错了要么少跑几步, 要么读越界。
    """
    body = _serialize_items(preset.get("items"), cat_by_type)
    note = _sanitize_note(preset.get("note", ""))
    lines = [
        "/* ===== 预设: %s ===== */" % sanitize_comment(preset.get("title", "")),
        "/* 播放方式: %s */" % (MODE_NOWAIT if preset.get("nowait") else MODE_WAIT),
        "/* 备注: %s */" % note if note else "/* 备注: */",
        "const stScriptStepTdf %s[] = {" % _table_of(preset),
    ]
    lines.extend(body)
    lines.append("};")
    lines.append("")
    return lines, _active_line_count(body)


def _serialize_aggregate(presets, counts):
    """g_astArmAction[] 聚合初始化 —— 顺序 / 播放方式 / 条目数都在这里, 固件按它遍历。

    第 3 段 = `stArmActionTdf::ucNoWait`（1=启动即返回）—— 固件运行时**唯一**的播放方式
    来源（排爆抓取/反恐任务按它决定要不要等这条动作走完）。表头那行 `播放方式:` 注释由
    同一个值渲染, 所以两者永不漂移。"""
    out = ["const stArmActionTdf g_astArmAction[ARM_ACTION_NUM] = {"]
    for p, n in zip(presets, counts):
        out.append('    { %du, "%s", %du, %s, %du },'
                   % (p["id"], sanitize_comment(p.get("title", "")),
                      1 if p.get("nowait") else 0, _table_of(p), n))
    out.append("};")
    return out


def serialize_data_region(presets, cat_by_type):
    """数据区内容（不含标记行本身）。"""
    out, counts = [""], []
    for p in presets:
        block, n = serialize_preset(p, cat_by_type)
        out.extend(block)
        counts.append(n)
    out.extend(_serialize_aggregate(presets, counts))
    out.append("")
    return out


def serialize_header_block(presets, next_id):
    """生成块内容（不含标记行本身）。

    `next_id` 取「文件里记的」与「最大 ID + 1」的较大者 —— **只增不减**。
    用 max(ids)+1 会让删掉最大 ID 后新预设复用那个号, 而 map.c 里可能还留着
    ARM_ACT_n 的引用 → 于无声中指向另一个动作。这是必须避免的静默失效。
    """
    next_id = max(int(next_id or 1), max([p["id"] for p in presets] + [0]) + 1)
    out = [
        "/* 下一个可用 ID。只增不减、删除不回收 —— 保证已经发给控制台的 id 不会在下次",
        " * 保存后指向另一个动作（那会让路线里的引用于无声中换目标）。 */",
        "#define ARM_ACTION_NEXT_ID    %du" % next_id,
        "",
        "/* 动作 ID: 路线表的 STEP_ARM_PLAY_WAIT / _NO_WAIT 用它引用动作; 控制台「动作」",
        " * 参数的下拉选项**取自本枚举**, 成员尾注释 = 用户起的标题。",
        " * 绑定条件 = app_script.c 里该步骤的 case 体写了 (emArmActionIdTdf)I(0)。",
        " *",
        " * ⚠️ 本枚举是**有意**被 server.py::parse_enums 扫到的（它的 glob 覆盖 Application",
        " *    下的头文件）。别因为「怕污染参数下拉」把它改成 #define —— 那样「动作」参数",
        " *    就没有下拉了, 只能手敲 ARM_ACT_n。 */",
        "typedef enum {",
    ]
    for p in presets:
        out.append("    ARM_ACT_%d = %d,   /* %s */"
                   % (p["id"], p["id"], sanitize_comment(p.get("title", ""))))
    out.append("} emArmActionIdTdf;")
    out.append("")
    out.append("/* 预设张数（= g_astArmAction[] 的长度, 控制台同步维护） */")
    out.append("#define ARM_ACTION_NUM        %du" % len(presets))
    return out


# ==================== 回收站 ====================

def load_trash():
    """读 arm_trash.json → [{trash_id, id, title, note, nowait, items, deleted_at}]。

    放仓库内（入库）, 与 phrases.json 同侧 —— 被删的动作是**项目内容**,
    队友拉下来应该也在回收站里; 而 bt.json / cap.json 那种各机私有硬件偏好才不入库。
    """
    try:
        data = json.loads(TRASH_JSON.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return []
    items = data.get("items") if isinstance(data, dict) else None
    return items if isinstance(items, list) else []


def save_trash(items):
    """原子写（tmp + os.replace），同 bt_link.save_prefs。"""
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


def _trash_payload(preset):
    """预设 → 回收站条目。丢掉 raw 行、全部标 edited —— 恢复时按当前格式重新生成。"""
    items = []
    for it in preset.get("items") or []:
        if it.get("kind") == "raw":
            continue
        copy = {k: v for k, v in it.items() if k != "lines"}
        copy["edited"] = True
        items.append(copy)
    return items


# ==================== 写盘 ====================

def _check_presets(presets, cat_by_type):
    """C 的硬约束。早点拦下来 —— 否则是编译期报错, 用户看不到原因。"""
    if not presets:
        return "至少需要保留一个动作预设（空的是空初始化列表, C 编译不过）"
    # 显式带 id 的预设必须互不重复（2026-10-09 加）: 重复 id 会生成两个同名枚举成员,
    # 报错指向生成块、用户根本看不出是 payload 的锅。_assign_ids 只给**没号**的发新号,
    # 不会查显式重复 —— 在这里拦。
    seen = {}
    for p in presets:
        pid = p.get("id")
        if pid:
            if pid in seen:
                return ("两条预设用了同一个 id %s（「%s」与「%s」）"
                        % (pid, seen[pid], p.get("title") or "?"))
            seen[pid] = p.get("title") or "?"
    for p in presets:
        if not (p.get("items") or []):
            return "动作「%s」没有任何子积木（空数组体编译不过）" % (p.get("title") or "?")
        # 全部被禁用 = 注释掉的是初始化项 → 数组体为空, 同样编译不过
        if _active_line_count(_serialize_items(p.get("items"), cat_by_type)) == 0:
            return ("动作「%s」的子积木**全被禁用**了（禁用的行是注释, 不进数组, "
                    "空数组体编译不过）—— 至少留一条启用" % (p.get("title") or "?"))
    return ""


def _assign_ids(presets, next_id):
    """没有 id 的预设（界面上新建的）在这里拿号。返回 (presets, next_id)。"""
    used = set(int(p["id"]) for p in presets if p.get("id"))
    next_id = max(int(next_id or 1), max(used) + 1 if used else 1)
    for p in presets:
        if not p.get("id"):
            while next_id in used:
                next_id += 1
            p["id"] = next_id
            used.add(next_id)
            next_id += 1
    return presets, next_id


def _replace_region(lines, begin_re, end_re, body):
    b, e = _find_region(lines, begin_re, end_re)
    return lines[:b] + list(body) + lines[e:]


def write_files(presets, cat_by_type, next_id):
    """把预设列表写回 .c 与 .h。返回 (changed, version)。

    **内容没变就一个字节都不写** —— 连 .bak 都不建。见模块 docstring 的理由。

    读-改-写整段在 WRITE_LOCK 里（RLock: 公开写入口已经拿着了, 这里再进一次是安全的）。
    """
    with WRITE_LOCK:
        return _write_files_locked(presets, cat_by_type, next_id)


def _write_files_locked(presets, cat_by_type, next_id):
    c_text = ACTION_C.read_text(encoding="utf-8")
    h_text = ACTION_H.read_text(encoding="utf-8")

    new_c_text = "\n".join(_replace_region(
        c_text.split("\n"), RE_DATA_BEGIN, RE_DATA_END,
        serialize_data_region(presets, cat_by_type)))
    new_h_text = "\n".join(_replace_region(
        h_text.split("\n"), RE_GEN_BEGIN, RE_GEN_END,
        serialize_header_block(presets, next_id)))

    if new_c_text != c_text:
        shutil.copy2(ACTION_C, ACTION_C.with_suffix(".c.bak"))
        map_codec.write_text_like(ACTION_C, new_c_text)     # 按原换行风格写
    if new_h_text != h_text:
        shutil.copy2(ACTION_H, ACTION_H.with_suffix(".h.bak"))
        map_codec.write_text_like(ACTION_H, new_h_text)     # 按原换行风格写

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
                    "message": "app_arm_action 已在外部被修改, 请先「重新加载」再保存"}, 409)


# 写盘串行化。这里的麻烦在于「校验版本」与「写文件」分居两个函数（各入口先
# `_version_ok` 再 `_commit`）, 所以锁必须由**公开写入口**持有整段 —— 只锁
# write_files 挡不住「两个请求都通过校验, 然后一前一后各写一遍」（丢更新）。
# 用 RLock: 入口拿着锁时 write_files 会再进一次。
# ⚠️ 新增写入口时记得也加 @_serialized（漏了不会报错, 只是那道门又开了）。
WRITE_LOCK = threading.RLock()


def _serialized(fn):
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        with WRITE_LOCK:
            return fn(*args, **kwargs)
    return wrapper


def _commit(presets, cat_by_type, next_id, extra):
    """共用的落盘路径: 发号 → 校验 → 写两个文件。"""
    presets, next_id = _assign_ids(list(presets), next_id)
    err = _check_presets(presets, cat_by_type)
    if err:
        return False, {"error": "invalid", "message": err}, 400
    changed, version = write_files(presets, cat_by_type, next_id)
    resp = {"ok": True, "version": version, "changed": changed}
    resp.update(extra or {})
    return True, resp, 200


def _find_preset(presets, pid):
    for p in presets:
        if int(p.get("id") or 0) == int(pid or 0):
            return p
    return None


# ==================== 动作分发 ====================

@_serialized
def save(payload, base_version, cat_by_type):
    """整份预设列表写盘（「显式保存」是文档级操作, 一次全写）。

    未编辑的条目靠 _serialize_items 的原始行回吐保证零 diff。
    表名由 ID 派生, 不接受外部指定 —— 否则换个表名就等于换了个数组, 白 diff 一片。
    """
    if not _version_ok(base_version):
        return CONFLICT
    presets = payload.get("presets")
    if not isinstance(presets, list):
        return False, {"error": "invalid", "message": "presets 必须是列表"}, 400

    clean = []
    for p in presets:
        q = {k: v for k, v in p.items() if k != "table"}
        for it in (q.get("items") or []):
            kind = it.get("kind")
            if kind not in ("pose", "grip", "delay", "raw"):
                return False, {"error": "invalid",
                               "message": "未知的子积木类型: %s" % kind}, 400
            if kind == "raw" and (it.get("step") or "") not in ALLOWED_STEPS \
                    and not it.get("lines"):
                return False, {"error": "invalid",
                               "message": "只读块必须带原始行, 否则会丢失"}, 400
        clean.append(q)

    cur = parse_arm()
    ok, resp, status = _commit(clean, cat_by_type, cur["next_id"], {})
    if ok:
        resp["presets"] = parse_arm()["presets"]
    return ok, resp, status


@_serialized
def delete_preset(pid, base_version, cat_by_type):
    """删预设: 先把整条塞进回收站, 再从 C 里删掉。

    ⚠️ map.c 里若还有 ARM_ACT_<id> 的引用, 删完会**编译期报错** —— 那是故意的
    （胜过静默指向另一个动作）。调用方应先 POST {"action":"refs"} 拿引用数弹确认。
    """
    if not _version_ok(base_version):
        return CONFLICT
    cur = parse_arm()
    presets = cur["presets"]
    p = _find_preset(presets, pid)
    if p is None:
        return False, {"error": "not found", "message": "没有这个动作预设"}, 404
    if len(presets) <= 1:
        return False, {"error": "last", "message": "至少要留一个动作预设"}, 400

    # 先 commit 再进回收站（2026-10-09 调序）: commit 被校验拦下（400）时 C 一个字节
    # 没动, 回收站也不该先多出一条「幽灵条目」（C 里动作还在, 回收站却显示已删）。
    ok, resp, status = _commit([x for x in presets if x["id"] != p["id"]],
                               cat_by_type, cur["next_id"], {})
    if not ok:
        return ok, resp, status

    trash = load_trash()
    entry = {"trash_id": _next_trash_id(trash), "id": p["id"], "title": p["title"],
             "note": p["note"], "nowait": p["nowait"],
             "deleted_at": time.strftime("%Y-%m-%d %H:%M"),
             "items": _trash_payload(p)}
    save_trash([entry] + trash)
    resp["trash_id"] = entry["trash_id"]
    return ok, resp, status


@_serialized
def restore_preset(trash_id, base_version, cat_by_type):
    """从回收站恢复: 发**新 ID** + 「原名恢复(x)」, 追加到末尾。

    ⚠️ 不复用旧 ID: 旧 ID 可能还被 map.c 引用着, 复用它会让那条引用复活并指向
    新内容。发新号则那条引用保持「未定义标识符」的编译期硬失败, 看得见。
    """
    if not _version_ok(base_version):
        return CONFLICT
    trash = load_trash()
    entry = next((t for t in trash if str(t.get("trash_id")) == str(trash_id)), None)
    if entry is None:
        return False, {"error": "not found", "message": "回收站里没有这一条"}, 404

    cur = parse_arm()
    presets = cur["presets"]
    if not entry.get("items"):
        return False, {"error": "empty", "message": "这一条没有子积木, 恢复不了"}, 400

    new_p = {
        "title": restore_title(entry.get("title", ""), set(p["title"] for p in presets)),
        "note": entry.get("note", ""),
        "nowait": entry.get("nowait", False),
        "items": [dict(it) for it in entry["items"]],
    }
    ok, resp, status = _commit(presets + [new_p], cat_by_type, cur["next_id"], {})
    if ok:
        save_trash([t for t in trash if str(t.get("trash_id")) != str(trash_id)])
        resp["presets"] = parse_arm()["presets"]
        resp["title"] = new_p["title"]
    return ok, resp, status


@_serialized
def purge_trash(trash_id):
    """彻底删（只动回收站 JSON, 不碰 C, 所以不需要版本戳）。"""
    trash = load_trash()
    left = [t for t in trash if str(t.get("trash_id")) != str(trash_id)]
    save_trash(left)
    return True, {"ok": True, "trash": left}, 200


def run_arm_action(action, payload, cat_by_type=None):
    """POST /api/arm 的分发（save/delete/restore/purge）。

    契约同 run_bt_action: 异常全消化成 (ok, data, error, status), 绝不外抛;
    status 是失败时要回的 HTTP 状态码（409/400, server 透传 —— 前端的
    「↺ 重载」提示条靠 HTTP 409 触发）。
    `refs` 不在这里 —— 它要读 map.c, 是 server 的地盘。
    """
    cat = cat_by_type or {}
    base = payload.get("base_version")
    try:
        if action == "save":
            ok, data, status = save(payload, base, cat)
        elif action == "delete":
            ok, data, status = delete_preset(payload.get("id"), base, cat)
        elif action == "restore":
            ok, data, status = restore_preset(payload.get("trash_id"), base, cat)
        elif action == "purge":
            ok, data, status = purge_trash(payload.get("trash_id"))
        else:
            return False, None, "未知动作: %s" % action, 400
        if ok:
            return True, data, "", 200
        return False, data, data.get("message") or data.get("error") or "操作失败", status
    except ArmError as exc:
        return False, None, str(exc), 400
    except Exception as exc:                                 # noqa: BLE001
        return False, None, "内部错误: %s" % exc, 500


def state():
    """GET /api/arm 的响应体。文件被手改坏时降级成带 error 的空壳, 不让页面 500。

    `joints_offline` 是给「机械臂」Tab 在**没连设备**时画灰色滑杆用的占位表
    （见 offline_joints）—— 放在这里是因为它只服务那个 Tab, 而不是设备状态。
    """
    try:
        st = parse_arm()
    except (ArmError, OSError) as exc:
        st = {"error": str(exc), "presets": [], "trash": [],
              "next_id": 1, "version": {"c": 0, "h": 0}}
    st["joints_offline"] = offline_joints()
    st["gripper_span"] = gripper_span()
    return st
