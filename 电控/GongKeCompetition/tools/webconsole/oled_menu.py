"""OLED 菜单 Tab 的服务端逻辑 —— 解析/序列化 ``Application/app_oled_menu.c`` 的「数据区」。

============================ 三条纪律 ============================

⚠️⚠️ **只重写「数据区」两个标记之间的整段, 标记外一个字节都不动**。表本体由本模块
   **整段生成**（含块标题注释）—— 与 ``arm_action.py`` 的取舍一致（那边也是「表头三行
   无条件重新生成」）。想写长期说明就写在标记**之外**。

⚠️ **无操作保存 = 字节级零 diff**：模型里每条目都带 ``raw``（原物理行）与 ``edited``
   标志, 没动过的行**按原样回吐**。所以这个模块的 ``serialize_region`` 必须先能被
   ``parse_region`` 喂出来的模型**原样复现**——测试盯着这条（照
   ``test_arm_action.py::TestGeneratedFileIsStable``）。

⚠️ **本模块不 import server**（server 要 import 本模块, 会循环）。动作码/调参量的下拉
   选项由调用方从 ``server.parse_enums()`` 取好传进来 —— 元数据一律以源码为数据源。

============================ 与固件的契约 ============================

* 一页 / 一个子菜单最多 **32** 个启用项（放不下的靠 UP/DOWN **滚动**查看 —— 屏一屏只
  显示 4 行, 右侧画竖直滚动条）。子菜单的「返回」由固件补在**最后一条**, 走同一个滚动
  列表, **不占**这 32 个额度（它不进表, 所以"生成表漏了返回 → 操作员卡死"这个失效模式
  在构造上不存在）。
* 菜单标签宽度预算 = **120px**（一行 128px, 最右 8px 是滚动条的预留带）; 调参行仍是 64px。
* 子表用**直接符号引用**（``&s_stListXxx``）：引用了不存在的子表 = C 编译错误，
  这是**有意的硬失败**（胜过重排后无声指向另一张表）。
* ``app_oled_menu.h`` 的「生成块」**本模块只读不写**（动作码是固件行为, 加一个动作码
  必然要改 app_oled.c 的 switch, 不是控制台能生成的）。它只参与版本戳。
"""

import functools
import os
import re
import shutil
import subprocess
import sys
import threading
from pathlib import Path

import map_codec

APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parents[1]

# 模块级常量: 测试用 mock.patch.object 沙箱化（照 test_arm_action.py::Sandbox）
MENU_C = ROOT / "Application" / "app_oled_menu.c"
MENU_H = ROOT / "Application" / "app_oled_menu.h"
GLYPHS_TXT = ROOT / "tools" / "fontgen" / "glyphs.txt"
GEN_FONT_PY = ROOT / "tools" / "fontgen" / "gen_font.py"
CHECK_COVERAGE_PY = ROOT / "tools" / "fontgen" / "check_coverage.py"
# gen_font.py 的写盘目标 —— 事务回滚也要把它罩住（写一半的字库文件 = 编译都过不了的坏文件）
DATA_C = ROOT / "Core" / "Src" / "OLED_Data.c"
# 「参数」弹窗（OLED_ACT_TUNE_PARAM 的参数名从哪来）的两个数据源:
#   本机参数 = app_beam.c 的 s_astBeamParam 表（静态初始化行, 直接正则扫）;
#   Pi 参数 = app_rpi.c 里 tools/gen_rpi_cfg.py 生成的块（同上）。
BEAM_C = ROOT / "Application" / "app_beam.c"
RPI_C = ROOT / "Application" / "app_rpi.c"

# 数据区标记（控制台只替换这两个标记之间的内容）
RE_DATA_BEGIN = re.compile(r"^\s*/\*\s*=+\s*数据区\s*[:：]\s*开始\s*=+\s*\*/\s*$")
RE_DATA_END = re.compile(r"^\s*/\*\s*=+\s*数据区\s*[:：]\s*结束\s*=+\s*\*/\s*$")

# ---- 数据区里的行形状 ----
RE_TITLE = re.compile(r"^\s*/\*\s*----\s*(.+?)\s*----\s*\*/\s*$")
RE_ARRAY_DEF = re.compile(r"^\s*static\s+const\s+stOledItemTdf\s+(\w+)\s*\[\s*\]\s*=\s*\{\s*$")
RE_CLOSE = re.compile(r"^\s*\}\s*;\s*$")
RE_LIST_DEF = re.compile(r"^\s*static\s+const\s+stOledListTdf\s+(\w+)\s*=\s*OLED_LIST\s*\(\s*(\w+)\s*\)\s*;\s*$")
RE_PAGES_DEF = re.compile(r"^\s*const\s+stOledPageTdf\s+g_astOledPage\s*\[\s*\]\s*=\s*\{\s*$")
RE_PAGE_ENTRY = re.compile(
    r"^\s*\{\s*\(uint8_t\)\s*(OLED_PAGEKIND_\w+)\s*,\s*(?:0|&\s*(\w+))\s*\}\s*,?\s*$")
RE_PAGE_NUM = re.compile(r"^\s*const\s+uint8_t\s+g_ucOledPageNum\s*=")

# 表行宏（顺序要紧: ITEM 是 ITEM_A / ITEM_P / ITEM_SUB 的前缀, 用完整括号锚定）
RE_ITEM = re.compile(r'^\s*OLED_ITEM\s*\(\s*(\w+)\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*(\d+)u\s*\)\s*,?\s*$')
RE_ITEM_A = re.compile(
    r'^\s*OLED_ITEM_A\s*\(\s*(\w+)\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*(\d+)u\s*,\s*(\w+)\s*\)\s*,?\s*$')
RE_ITEM_SUB = re.compile(
    r'^\s*OLED_ITEM_SUB\s*\(\s*"((?:[^"\\]|\\.)*)"\s*,\s*(\d+)u\s*,\s*&\s*(\w+)\s*\)\s*,?\s*$')
# 按**参数名**调的调参行（OLED_ACT_TUNE_PARAM, 2026-10 加）: 第四个字段是**带引号的名字**
# （参数名是运行期查表的**数据**, 不是标识符宏 —— Beam 表里是小写、Pi 表里全大写,
#  写成裸符号就得给每个参数配一个 #define）。名字本身过 RE_PARAM_TOKEN 白名单。
RE_ITEM_P = re.compile(
    r'^\s*OLED_ITEM_P\s*\(\s*(\w+)\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*(\d+)u'
    r'\s*,\s*"([A-Za-z_][A-Za-z0-9_]*)"\s*\)\s*,?\s*$')
# 同上, 再带第五个字段 = **自定义步进**（2026-10-18 加, 宏 OLED_ITEM_PS）:
# 十进制正数字符串（如 "0.01"/"100"）, 固件按它当 LEFT/RIGHT 的步长;
# 空串/解析不出 = 系统步进。只认数字与小数点 —— 它会被原样内插进 C 字符串,
# 白名单是防注入的那道门（同 RE_PARAM_TOKEN）。
RE_ITEM_PS = re.compile(
    r'^\s*OLED_ITEM_PS\s*\(\s*(\w+)\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*(\d+)u'
    r'\s*,\s*"([A-Za-z_][A-Za-z0-9_]*)"\s*,\s*"([0-9.]+)"\s*\)\s*,?\s*$')
# 同上再带第六个字段 = 下拉参数的**显示映射**（2026-10-18 加, 宏 OLED_ITEM_PM）:
# "1=手|3=自" —— 值=屏上短标签, | 分隔; 值必须落在真选项表里（_check 对着
# param_opts 验）, 短标签过 sanitize_tag。step 槽允许空串（只配映射不配步进）。
RE_ITEM_PM = re.compile(
    r'^\s*OLED_ITEM_PM\s*\(\s*(\w+)\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*(\d+)u'
    r'\s*,\s*"([A-Za-z_][A-Za-z0-9_]*)"\s*,\s*"([0-9.]*)"\s*,\s*"([^"\\]*)"\s*\)\s*,?\s*$')

# 固定在最前面的两个「代码页」（状态页 / 二维码页）—— 内容由 app_oled.c 画, 不可编辑
KIND_STATUS = "OLED_PAGEKIND_STATUS"
KIND_QR = "OLED_PAGEKIND_QR"
KIND_MENU = "OLED_PAGEKIND_MENU"

# ---- payload 标识符白名单（2026-10-09 加）----
# 这些字段会被**原样内插**进生成的 C（对照 arm_action 的 ALLOWED_STEPS 先例）:
# label 有 sanitize_tag, 其余标识符字段此前全部裸奔 —— payload 里塞一个
# `OLED_ACT_NOP), 0u }; int evil(void)...` 就能把任意 C 代码写进 app_oled_menu.c。
RE_C_IDENT  = re.compile(r"^[A-Za-z_]\w*$")            # 表名 / 数组名 / sub 引用
RE_ACT_CODE = re.compile(r"^OLED_ACT_[A-Za-z0-9_]+$")  # 动作码
RE_PAGE_KIND = re.compile(r"^OLED_PAGEKIND_[A-Za-z0-9_]+$")
RE_ARG_TOKEN = re.compile(r"^[A-Za-z0-9_]+$")          # 参数槽: 枚举成员或数字, 不带符号
# 通用调参行的**参数名**（OLED_ITEM_P 的第 4 个字段）: 字母开头 + 字母/数字/下划线。
# 它就是固件要拿去查表的名字, 会原样内插进 C 字符串 —— 白名单是防注入的那道门。
RE_PARAM_TOKEN = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")

ACT_ENTER_SUB = "OLED_ACT_ENTER_SUB"
ACT_TUNE_PARAM = "OLED_ACT_TUNE_PARAM"

# 哪些动作码需要「参数」、参数取自哪个枚举 —— 这是控制台侧的元数据（固件的 switch 才是
# 权威实现）。前端据此决定「参数」那一格渲染成下拉还是禁用。
ACT_ARG_ENUM = {
    "OLED_ACT_POST_TASK": "emScriptPresetIdTdf",
    "OLED_ACT_RUN_PRESET": "emScriptPresetIdTdf",
    # 射靶测试: 参数 = 跑哪个「保持」预设（固件拿它投 SCRIPT_REQ_TASK,
    # 也拿它当标签真值 ucScriptPresetActive(arg)）。漏了这一行 → 控制台生成
    # `OLED_ITEM(...)`（无 arg）→ 固件 ucArg=0 → 查不到预设, 屏上按了只打
    # `MENU: task preset id not found`（2026-10-23 现场踩过）。
    "OLED_ACT_SHOOT_TEST": "emScriptPresetIdTdf",
    "OLED_ACT_TUNE_QTY": "emOledTuneQtyTdf",
    "OLED_ACT_TIMER": "emTimerCmdTdf",
}

# 这些动作码的 `arg` **必须**是一个真预设 —— 枚举里的 `SCRIPT_PRESET_NONE` 是
# 「不跳转」的哨兵, 选到它固件只会长鸣 + 打一条调试串（不静默, 但对操作员就是个死项）。
ACT_NEEDS_PRESET = ("OLED_ACT_POST_TASK", "OLED_ACT_RUN_PRESET",
                    "OLED_ACT_SHOOT_TEST")

MAX_ROWS = 4          # 一屏**同时**能看到几行（窗口高度; 64px / 16px）
MAX_ITEMS = 32        # 一张表最多几个启用项 —— **页面与子菜单同一条上限**
SUB_MAX_ITEMS = MAX_ITEMS   # 子菜单同名上限（「返回」是固件补的最后一条, 不占这个额度）
SCROLLBAR_PX = 8      # 右侧竖直滚动条预留（与固件 OLED_SCROLLBAR_W 对齐）
ROW_PX = 128          # 一行物理像素宽
LABEL_PX = ROW_PX - SCROLLBAR_PX      # 120 —— 文字实际能用的宽度
TUNE_LABEL_PX = 64    # 调参行标签（还要拼上 ":值"）
HAN_PX, ASC_PX = 16, 8


class OledMenuError(Exception):
    """可直接展示给用户的错误。"""


# ==================== 文本度量 ====================

def _is_han(ch):
    o = ord(ch)
    return (0x4E00 <= o <= 0x9FFF or 0x3400 <= o <= 0x4DBF
            or 0x3000 <= o <= 0x303F or 0xFF00 <= o <= 0xFFEF)


def text_width(s):
    """按屏幕像素算宽度（汉字 16px、其余 8px）—— 与固件的 ucOledDrawMixed 同一套判据。"""
    return sum(HAN_PX if _is_han(c) else ASC_PX for c in s)


def collect_han(texts):
    """取一组字符串里用到的全部汉字（去重保序）—— 给字表补字用。"""
    seen = {}
    for t in texts:
        for c in t:
            if _is_han(c) and c not in seen:
                seen[c] = None
    return list(seen)


# ==================== 解析 ====================

def _find_region(lines, what="app_oled_menu.c"):
    """找数据区, 返回 0-based 半开区间 (begin+1, end); 找不到抛 OledMenuError。"""
    rng = map_codec.find_marked(lines, RE_DATA_BEGIN, RE_DATA_END)
    if rng is None:
        raise OledMenuError("%s 缺少「数据区」标记（被手改坏了？）" % what)
    return rng


def _parse_items(body_lines):
    """一组表行 → item dict 列表。

    注释/空行**直接丢弃**（数据区整段重生成, 不保位置）—— 手写进表里的游离注释
    保存即消失, 这是设计内的规范化。认不出的**代码行**仍抛 OledMenuError（那是
    数据区被手改坏了, 静默吞掉会丢积木）。
    """
    items = []
    for ln in body_lines:
        s = ln.strip()
        if not s or s.startswith("/*") or s.startswith("//"):
            continue
        m = RE_ITEM_SUB.match(ln)
        if m:
            items.append({"label": m.group(1), "act": ACT_ENTER_SUB,
                          "enabled": m.group(2) != "0", "arg": None,
                          "sub": m.group(3), "param": None, "step": None,
                          "map": None, "raw": ln, "edited": False})
            continue
        m = RE_ITEM_PM.match(ln)
        if m:
            items.append({"label": m.group(2), "act": m.group(1),
                          "enabled": m.group(3) != "0", "arg": None,
                          "sub": None, "param": m.group(4), "step": m.group(5) or None,
                          "map": m.group(6), "raw": ln, "edited": False})
            continue
        m = RE_ITEM_PS.match(ln)
        if m:
            items.append({"label": m.group(2), "act": m.group(1),
                          "enabled": m.group(3) != "0", "arg": None,
                          "sub": None, "param": m.group(4), "step": m.group(5),
                          "map": None, "raw": ln, "edited": False})
            continue
        m = RE_ITEM_P.match(ln)
        if m:
            items.append({"label": m.group(2), "act": m.group(1),
                          "enabled": m.group(3) != "0", "arg": None,
                          "sub": None, "param": m.group(4), "step": None,
                          "map": None, "raw": ln, "edited": False})
            continue
        m = RE_ITEM_A.match(ln)
        if m:
            items.append({"label": m.group(2), "act": m.group(1),
                          "enabled": m.group(3) != "0", "arg": m.group(4),
                          "sub": None, "param": None, "step": None,
                          "map": None, "raw": ln, "edited": False})
            continue
        m = RE_ITEM.match(ln)
        if m:
            items.append({"label": m.group(2), "act": m.group(1),
                          "enabled": m.group(3) != "0", "arg": None,
                          "sub": None, "param": None, "step": None,
                          "map": None, "raw": ln, "edited": False})
            continue
        raise OledMenuError("认不出的表行（数据区被手改坏了？）: %s" % s)
    return items


def parse_region(text):
    """解析 app_oled_menu.c 的整份文本 → 模型 dict。

    模型（也是前端 JSON 的形状）::

        {
          "order": ["s_stListTaskExec", ...],          # 文件里的书写顺序（子表在前）
          "lists": {"s_stListTaskExec": {"title": ..., "items": [...]}, ...},
          "pages": [{"kind": "OLED_PAGEKIND_STATUS", "list": None}, ...,
                    {"kind": "OLED_PAGEKIND_MENU", "list": "s_stListTask"}],
          "page_num_raw": "const uint8_t g_ucOledPageNum = ...;",
        }

    ⚠️ ``order`` 必须照**文件顺序**保留: 生成时按它书写, 才能做到无操作保存零 diff。
    """
    lines = text.split("\n")
    b, e = _find_region(lines)
    body = lines[b:e]

    order, lists = [], {}
    pages, page_num_raw = [], ""

    i, pending_title = 0, ""
    while i < len(body):
        ln = body[i]
        m = RE_TITLE.match(ln)
        if m:
            pending_title = m.group(1)
            i += 1
            continue

        m = RE_ARRAY_DEF.match(ln)
        if m:
            arr = m.group(1)
            j = i + 1
            while j < len(body) and not RE_CLOSE.match(body[j]):
                j += 1
            if j >= len(body):
                raise OledMenuError("数组 %s 没有闭合的 };（数据区被手改坏了？）" % arr)
            items = _parse_items(body[i + 1:j])
            # 紧随其后的 OLED_LIST(...) 行给出 list 符号名
            k = j + 1
            while k < len(body) and not body[k].strip():
                k += 1
            lm = RE_LIST_DEF.match(body[k]) if k < len(body) else None
            if lm is None or lm.group(2) != arr:
                raise OledMenuError("数组 %s 后面没有对应的 OLED_LIST(...) 行" % arr)
            name = lm.group(1)
            lists[name] = {"title": pending_title, "items": items,
                           "array": arr, "raw_def": body[k], "edited": False}
            order.append(name)
            pending_title = ""
            i = k + 1
            continue

        m = RE_PAGES_DEF.match(ln)
        if m:
            j = i + 1
            while j < len(body) and not RE_CLOSE.match(body[j]):
                em = RE_PAGE_ENTRY.match(body[j])
                if em:
                    pages.append({"kind": em.group(1), "list": em.group(2),
                                  "raw": body[j], "edited": False})
                else:
                    # 与 _parse_items 同口径: 空行/注释跳过, 认不出的行**抛** ——
                    # 静默跳过 = 手改坏的行下次保存被无声删除
                    t = body[j].strip()
                    if t and not t.startswith(("//", "/*", "*")):
                        raise OledMenuError(
                            "页面表里有认不出的行: %r（数据区被手改坏了？）" % t)
                j += 1
            if j >= len(body):
                raise OledMenuError("页面表没有闭合的 };（数据区被手改坏了？）")
            k = j + 1
            while k < len(body) and not RE_PAGE_NUM.match(body[k]):
                k += 1
            page_num_raw = body[k] if k < len(body) else ""
            i = k + 1
            continue

        i += 1

    if not pages:
        raise OledMenuError("页面表是空的（数据区被手改坏了？）")
    return {"order": order, "lists": lists, "pages": pages, "page_num_raw": page_num_raw}


# ==================== 序列化 ====================

def _fmt_item(it):
    """一个 item → 一行表行（规范格式; 只在条目被编辑/新增时用）。"""
    label = map_codec.sanitize_tag(it.get("label") or "", fallback="菜单")
    on = "1u" if it.get("enabled") else "0u"
    if it.get("sub"):
        return '    OLED_ITEM_SUB("%s", %s, &%s),' % (label, on, it["sub"])
    if it.get("param"):
        step = str(it.get("step") or "").strip()
        mp = str(it.get("map") or "").strip()
        if mp:
            return '    OLED_ITEM_PM(%s, "%s", %s, "%s", "%s", "%s"),' % (
                it["act"], label, on, it["param"], step, mp)
        if step:
            return '    OLED_ITEM_PS(%s, "%s", %s, "%s", "%s"),' % (
                it["act"], label, on, it["param"], step)
        return '    OLED_ITEM_P(%s, "%s", %s, "%s"),' % (it["act"], label, on, it["param"])
    if it.get("arg"):
        return '    OLED_ITEM_A(%s, "%s", %s, %s),' % (it["act"], label, on, it["arg"])
    return '    OLED_ITEM(%s, "%s", %s),' % (it["act"], label, on)


def serialize_region(model):
    """模型 → 数据区**内部**的行列表（不含两个标记本身）。

    ⚠️ 未编辑的条目回吐 ``raw``、未编辑的 ``OLED_LIST`` 行也回吐 —— 这是
    「无操作保存 = 零 diff」的唯一保证。
    """
    out = [""]
    for name in model["order"]:
        lst = model["lists"][name]
        # title 进 /* ---- x ---- */ 注释: 用户可改, 过一遍块注释清洗（2026-10-09）
        title = map_codec.sanitize_block_comment(lst.get("title") or name, fallback=name)
        out.append("/* ---- %s ---- */" % title)
        out.append("static const stOledItemTdf %s[] = {" % lst["array"])
        for it in lst["items"]:
            if not it.get("edited") and it.get("raw"):
                out.append(it["raw"])
            else:
                out.append(_fmt_item(it))
        out.append("};")
        if not lst.get("edited") and lst.get("raw_def"):
            out.append(lst["raw_def"])
        else:
            out.append("static const stOledListTdf %s = OLED_LIST(%s);"
                       % (name, lst["array"]))
        out.append("")

    out.append("/* ---- 页面表 ---- */")
    out.append("const stOledPageTdf g_astOledPage[] = {")
    for p in model["pages"]:
        if not p.get("edited") and p.get("raw"):
            out.append(p["raw"])
        elif p["list"]:
            out.append("    { (uint8_t)%s, &%s }," % (p["kind"], p["list"]))
        else:
            out.append("    { (uint8_t)%s, 0 }," % p["kind"])
    out.append("};")
    out.append(model.get("page_num_raw")
               or "const uint8_t g_ucOledPageNum = "
                  "(uint8_t)(sizeof(g_astOledPage) / sizeof(g_astOledPage[0]));")
    out.append("")
    return out


def _replace_region(lines, region_lines):
    """把 lines 里两个标记之间的整段换成 region_lines（``find_marked`` 给的是开区间）。"""
    rng = map_codec.find_marked(lines, RE_DATA_BEGIN, RE_DATA_END)
    if rng is None:
        raise OledMenuError("app_oled_menu.c 缺少「数据区」标记")
    b, e = rng
    return lines[:b] + region_lines + lines[e:]


# ==================== 校验 ====================

def _stable_topo_order(order, lists):
    """把 `order` 稳定地重排成「子表一定排在引用它的表之前」。

    C 要求符号先声明（`OLED_ITEM_SUB` 里是直接取地址）, 但**不该让排菜单的人操心这件事**
    —— 于是保存时自动排。做法是按原顺序做 Kahn: 每次从「依赖都已就位」的表里挑
    **原顺序最靠前**的那个。

    ⚠️ 对**已经合法**的文件这是**恒等变换**（原始顺序本身就满足约束）—— 无操作保存
    仍然字节级零 diff, 这条由 `TestNoOpSave` 盯着。
    返回 (新顺序, 成环的表名列表)；成环时原序返回, 由 `_check` 报错。
    """
    deps = {n: set() for n in order}
    for n in order:
        for it in ((lists.get(n) or {}).get("items") or []):
            sub = it.get("sub")
            if sub and sub in deps and sub != n:
                deps[n].add(sub)

    done, out, remaining = set(), [], list(order)
    while remaining:
        pick = next((n for n in remaining if deps[n] <= done), None)
        if pick is None:                       # 成环: 谁也排不到前面
            cycle = [n for n in remaining if deps[n] - done]
            return list(order), cycle
        out.append(pick)
        done.add(pick)
        remaining.remove(pick)
    return out, []


def _check(model, valid_params=None, param_opts=None):
    """保存前的硬校验。返回错误串（空 = 通过）。全部是「控制台能拦住」的错。

    ``valid_params``（可选）= 通用调参行允许引用的参数名集合（Beam 表 ∪ Pi 参数表,
    由调用方从源码解析后传进来）。传 None 就跳过"名字存不存在"那条 —— 形状校验照做。
    ``param_opts``（可选, 2026-10-18 加）= {参数名: 真选项表字符串}（形状同 Beam 的
    "0=关|1=开"）—— 显示映射（OLED_ITEM_PM 的 step 之后的 map 段）的值必须落在
    它的选项值集合里, 否则固件按映射循环会写出一个真表不认的值（Pi 那边回 ERR）。
    """
    pages = model["pages"]
    if len(pages) < 2 or pages[0].get("kind") != KIND_STATUS or pages[1].get("kind") != KIND_QR:
        return ("页面表的前两条必须是状态页与二维码页（它们是代码页, 不能删/改/换序）")

    # ---- 形状白名单（先于语义校验: 这些字段会内插进生成的 C, 见 RE_C_IDENT 处注释）----
    for p in pages:
        if not isinstance(p, dict):
            return "页面表里有不是对象的条目"
        kind = p.get("kind") or ""
        lst = p.get("list") or ""
        if not RE_PAGE_KIND.match(kind):
            return "页面类型 %r 不是合法的 OLED_PAGEKIND_* 标识符" % kind
        if lst and not RE_C_IDENT.match(lst):
            return "页面引用的表名 %r 不是合法 C 标识符" % lst
    for name in model["order"]:
        if not RE_C_IDENT.match(name):
            return "表名 %r 不是合法 C 标识符" % name
        lst = model["lists"][name]
        if not isinstance(lst, dict):
            return "表 %s 的定义不是对象" % name
        arr = lst.get("array") or ""
        if not RE_C_IDENT.match(arr):
            return "表 %s 的数组名 %r 不是合法 C 标识符" % (name, arr)
        if not isinstance(lst.get("items"), list):
            return "表 %s 的 items 不是列表" % name
    for name in model["order"]:
        for it in model["lists"][name]["items"]:
            if not isinstance(it, dict):
                return "表 %s 里有不是对象的条目" % name
            act = it.get("act") or ""
            arg = it.get("arg") or ""
            sub = it.get("sub") or ""
            param = it.get("param") or ""
            if not isinstance(it.get("label"), str):
                return "表 %s 里有条目的 label 不是字符串" % name
            # 会写进 C 的是「edited 或没有 raw」的条目（其余按 raw 行原样回吐）——
            # 白名单必须罩住整个集合: 只查 edited 的话, 外部客户端发
            # {act: <C 形状 payload>, enabled: 1}（无 raw、未 edited）就能绕过
            if (not RE_ACT_CODE.match(act)) and (it.get("edited") or not it.get("raw")):
                return "动作码 %r 不是合法的 OLED_ACT_* 标识符" % act
            if arg and not RE_ARG_TOKEN.match(str(arg)):
                return "参数 %r 只能是枚举成员或数字" % arg
            if sub and not RE_C_IDENT.match(sub):
                return "子表引用 %r 不是合法 C 标识符" % sub
            # ---- 通用调参行（OLED_ACT_TUNE_PARAM）: 名字与动作码必须成对 ----
            if param and not RE_PARAM_TOKEN.match(str(param)):
                return "参数名 %r 不是合法标识符（字母开头, 字母/数字/下划线）" % param
            if act == ACT_TUNE_PARAM and not param:
                return "「%s」是通用调参行但没选参数（用「参数」那一格的下拉选一个）" % (
                    it.get("label") or "?")
            if param and act != ACT_TUNE_PARAM:
                return ("「%s」带了参数名 %r, 但动作码是 %s —— 只有 OLED_ACT_TUNE_PARAM "
                        "才吃参数名" % (it.get("label") or "?", param, act))
            if param and (valid_params is not None) and (param not in valid_params):
                return ("「%s」的参数名 %r 在 Beam 表与 Pi 参数表里都查不到 —— "
                        "固件会显示 ? 并长鸣（先用「参数」下拉重新选一个）"
                        % (it.get("label") or "?", param))
            # ---- 自定义步进（OLED_ITEM_PS 的 step, 2026-10-18 加）----
            # 只在通用调参行上有意义; 必须是能被固件解析的正十进制数（≤12 字符,
            # 防 "0.0000001" 这种凑长度塞垃圾 —— 固件解析器只认 [0-9.], 这里再限长）。
            step = str(it.get("step") or "").strip()
            if step and act != ACT_TUNE_PARAM:
                return ("「%s」带了步进 %r, 但动作码是 %s —— 步进只有通用调参行吃得下"
                        % (it.get("label") or "?", step, act))
            if step and not param:
                return "「%s」带了步进但没选参数 —— 先选参数再配步进" % (it.get("label") or "?")
            if step:
                try:
                    f = float(step)
                except ValueError:
                    return "「%s」的步进 %r 不是数字（只认 [0-9.] 组成的正数, 如 0.01）" % (
                        it.get("label") or "?", step)
                if (not step.replace(".", "", 1).isdigit()) or f <= 0:
                    return "「%s」的步进 %r 不是正数（只认 [0-9.] 组成的正数, 如 0.01）" % (
                        it.get("label") or "?", step)
                if len(step) > 12:
                    return "「%s」的步进 %r 太长（≤12 字符）" % (it.get("label") or "?", step)
            # ---- 显示映射（OLED_ITEM_PM 的 map 段, 2026-10-18 加）----
            # 只在下拉类调参行上有意义; 形状 = "值=短标签|值=短标签", 值唯一且必须
            # 落在该参数的真选项表里（真表没有的值固件循环时会写出去 → Pi 回 ERR）。
            mapstr = str(it.get("map") or "").strip()
            if mapstr and act != ACT_TUNE_PARAM:
                return ("「%s」带了显示映射 %r, 但动作码是 %s —— 映射只有通用调参行吃得下"
                        % (it.get("label") or "?", mapstr, act))
            if mapstr and not param:
                return "「%s」带了显示映射但没选参数 —— 先选参数再配映射" % (it.get("label") or "?")
            if mapstr:
                real_vals = None
                if param_opts and param in param_opts:
                    real_vals = {seg.split("=", 1)[0].strip()
                                 for seg in str(param_opts[param]).split("|") if "=" in seg}
                seen_vals = set()
                for seg in mapstr.split("|"):
                    if "=" not in seg:
                        return ("「%s」的映射段 %r 缺 '='（形状: 值=短标签, 用 | 分隔）"
                                % (it.get("label") or "?", seg))
                    v, s = (x.strip() for x in seg.split("=", 1))
                    if not v or not s:
                        return ("「%s」的映射段 %r 值/短标签有空 —— 短标签留空就删掉整段"
                                % (it.get("label") or "?", seg))
                    # 两段都会内插进 C 字符串字面量 —— 引号/反斜杠/换行/注释符能写坏
                    # app_oled_menu.c（sanitize_tag 家族防的就是这类事故, 本字段漏网过）
                    for part in (v, s):
                        if any(ch in part for ch in ('"', "\\", "\n", "/*", "*/")):
                            return ("「%s」的映射段 %r 含非法字符（引号/反斜杠/换行/注释符）"
                                    % (it.get("label") or "?", seg))
                    if v in seen_vals:
                        return "「%s」的映射里值 %r 重复了 —— 一个值只映射一个短标签" % (
                            it.get("label") or "?", v)
                    seen_vals.add(v)
                    if (real_vals is not None) and (v not in real_vals):
                        return ("「%s」的映射值 %r 不在参数 %s 的选项表里（可选值: %s）"
                                % (it.get("label") or "?", v, param, ", ".join(sorted(real_vals))))
                    if text_width(s) > 48:
                        return ("「%s」的映射短标签 %r 太宽（>48px, 3 个汉字封顶 —— 太长会把"
                                "「标签:值」挤出行）" % (it.get("label") or "?", s))
    raw_line = (model.get("page_num_raw") or "").strip()
    if raw_line and (not RE_PAGE_NUM.match(raw_line)
                     or any(ch in raw_line for ch in ('"', "/*", "*/", "\n"))):
        return "page_num_raw 不是合法的 g_ucOledPageNum 定义行"

    # 页引用完整性（「子表先于引用者」由 save() 的稳定拓扑排序自动保证, 不在这里判）
    for p in pages:
        if p.get("list") and p["list"] not in model["lists"]:
            return "页面引用了不存在的表 %s" % p["list"]

    page_lists = set(p["list"] for p in pages if p.get("list"))

    for name in model["order"]:
        lst = model["lists"][name]
        items = lst["items"]
        if not items:
            # C 不允许空初始化列表
            return "表 %s 一项都没有（C 不允许空初始化列表）" % name
        enabled = [it for it in items if it["enabled"]]
        if len(enabled) > MAX_ITEMS:
            return ("表 %s 有 %d 个启用项, 超过 %d 项上限"
                    "（一屏只画 4 行, 放不下的靠 UP/DOWN 滚动; 第 %d 项起在车上既看不见也选不到）"
                    % (name, len(enabled), MAX_ITEMS, MAX_ITEMS + 1))
        for it in items:
            w = text_width(it["label"])
            if it["act"] in ("OLED_ACT_TUNE_QTY", ACT_TUNE_PARAM):
                # 调参行要拼上 ":值"（最宽 "-180.0" = 6 ASCII = 48px + ':' 8px）
                if w > TUNE_LABEL_PX:
                    return ("调参行「%s」太宽（%dpx）: 还要拼上「:值」, 上限 %dpx（4 个汉字）"
                            % (it["label"], w, TUNE_LABEL_PX))
            elif w > LABEL_PX:
                return ("菜单项「%s」太宽（%dpx > %dpx）: 右侧 %dpx 要让给滚动条"
                        % (it["label"], w, LABEL_PX, SCROLLBAR_PX))

            # ---- 「能配出来但语义不成立」的组合, 拦掉免得变成死项 ----
            # 选到 `SCRIPT_PRESET_NONE`（"不跳转"的哨兵）→ 固件只会长鸣。
            if it["act"] in ACT_NEEDS_PRESET:
                if (not it.get("arg")) or str(it["arg"]).endswith("_NONE"):
                    return ("「%s」还没选预设（`SCRIPT_PRESET_NONE` 是「不跳转」的哨兵, "
                            "跑起来只会长鸣）" % it["label"])
            # ---- 「调参」行放页面 / 放子菜单都行（2026-10-09 起）----
            # 两种位置行为不同, 但都**能用**:
            #   · 页面上: 光标停在这一行时 LEFT/RIGHT 是**调值**（翻页暂时让位 ——
            #     想翻页先按 UP/DOWN 把光标挪开。这是明说的代价）;
            #   · 子菜单: LEFT/RIGHT 一贯是调值（子菜单本来就不翻页）。
            # ⚠️ 原来这里直接拒掉「页面上的调参行」, 理由是那时固件在页面上只认翻页
            #    → 放上去就是个点不动的死项。固件那边（app_oled.c::vOledPageMenu 的
            #    bTune 判据）已经改成就地调值, 所以这条校验跟着放开。
            # ⚠️ **两边必须一起改**: 只放开这里而不改固件 = 页面上真出死项;
            #    只改固件而不放开这里 = 这一页在控制台里根本存不进去。
            # 子菜单只支持**一层**（固件只存一个 s_pstSubList, 套两层会丢掉上一层,
            # 退出时直接弹回页面）。
            if it["act"] == "OLED_ACT_ENTER_SUB" and name not in page_lists:
                return ("子菜单「%s」里不能再放「子菜单」块（只支持一层）"
                        % (lst.get("title") or name))

            if it.get("sub"):
                if it["sub"] not in model["lists"]:
                    return "菜单项「%s」引用了不存在的子表 %s" % (it["label"], it["sub"])
                # ⚠️ 注: `SUB_MAX_ITEMS == MAX_ITEMS` 时下面这条其实轮不到 —— 子表本身也在
                #    `order` 里, 上面「每表最多 32 项」那条会先命中。留着它 (a) 说明子菜单的
                #    上限来源, (b) 万一以后两条上限分道扬镳还能兜住。
                n_sub = len([x for x in model["lists"][it["sub"]]["items"] if x["enabled"]])
                if n_sub > SUB_MAX_ITEMS:
                    return ("子表 %s 有 %d 个启用项: 子菜单最多 %d 项"
                            "（「返回」由固件补在最后一条、随列表滚动, 不占这个额度）"
                            % (it["sub"], n_sub, SUB_MAX_ITEMS))
    return ""


# ==================== 参数清单（通用调参行「参数」弹窗的数据源） ====================

# 表行形状（两张表各一套正则; 都是**静态初始化行, 一行一项** —— 与菜单那套"行首宏"
# 解析同一个思路: 从源码派生, 控制台里不写死任何默认值）。
#
# Beam（app_beam.c::s_astBeamParam, 结构见 app_beam.h::stBeamParamTdf）:
#   { "name", "label", "group", "unit", BEAM_VAL_X, rw, warn, min, max, getter, setter[, "选项"] },
# ⚠️ 只读项（rw=0）也收 —— 屏上"看遥测"是这一档的正经用法（app_oled.c 对只读项长鸣拒绝写）。
RE_BEAM_ROW = re.compile(
    r'^\s*\{\s*"([A-Za-z_]\w*)"\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*"((?:[^"\\]|\\.)*)"\s*,'
    r'\s*"((?:[^"\\]|\\.)*)"\s*,\s*(BEAM_VAL_\w+)\s*,\s*(\d+)u\s*,\s*(\d+)u\s*,'
    r'\s*(-?[\d.]+)f?\s*,\s*(-?[\d.]+)f?\s*,\s*(\w+)\s*,\s*(\w+)'
    r'(?:\s*,\s*"((?:[^"\\]|\\.)*)")?\s*\}\s*,?\s*$')

# Pi（app_rpi.c 的生成块, 结构见 app_rpi.h::stRpiCfgParamTdf）:
#   { "NAME", "中文名", "分组", RPI_CFG_T_X, NULL|s_acRpiCfgOptN },
RE_RPI_ROW = re.compile(
    r'^\s*\{\s*"([A-Za-z_]\w*)"\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*"((?:[^"\\]|\\.)*)"\s*,'
    r'\s*(RPI_CFG_T_\w+)\s*,\s*(NULL|\w+)\s*\}\s*,?\s*$')
# 去重的选项表常量（生成块头部）: static const char s_acRpiCfgOpt0[] = "0=关|1=开";
RE_OPT_STR = re.compile(r'^\s*static\s+const\s+char\s+(\w+)\s*\[\s*\]\s*=\s*"((?:[^"\\]|\\.)*)"\s*;\s*$')

# 类型 → 界面粗分类。前端只关心三件事: 数值/字符串、能不能写、是不是 0-1 勾选。
BEAM_KIND = {"BEAM_VAL_F32": "float", "BEAM_VAL_I32": "int",
             "BEAM_VAL_U8": "uint", "BEAM_VAL_I8": "int"}
RPI_KIND = {"RPI_CFG_T_F32": "float", "RPI_CFG_T_I32": "int", "RPI_CFG_T_STR": "str"}


def parse_beam_params():
    """扫 app_beam.c 的注册表 → [{name,label,group,unit,kind,ro,warn,min,max,options,src}]。"""
    out, seen = [], set()
    try:
        lines = Path(BEAM_C).read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError):
        return out
    for ln in lines:
        m = RE_BEAM_ROW.match(ln)
        if not m:
            continue
        name = m.group(1)
        if name in seen:      # 同名只认第一处（表本体; 表外若也有同形状行不重复收）
            continue
        seen.add(name)
        out.append({
            "name": name, "label": m.group(2), "group": m.group(3), "unit": m.group(4),
            "kind": BEAM_KIND.get(m.group(5), "float"), "ro": m.group(6) == "0",
            "warn": m.group(7) == "1", "min": m.group(8), "max": m.group(9),
            "options": m.group(12) or "", "src": "beam",
        })
    return out


def parse_rpi_params():
    """扫 app_rpi.c 的生成块 → 同上形状（值在树莓派上: 没有 min/max, 选项来自 config.py）。"""
    out, opts = [], {}
    try:
        lines = Path(RPI_C).read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError):
        return out
    for ln in lines:
        om = RE_OPT_STR.match(ln)
        if om:
            opts[om.group(1)] = om.group(2)
    for ln in lines:
        m = RE_RPI_ROW.match(ln)
        if not m:
            continue
        out.append({
            "name": m.group(1), "label": m.group(2), "group": m.group(3), "unit": "",
            "kind": RPI_KIND.get(m.group(4), "str"),
            # Pi 侧一律可写; str 型例外（它是"看"的 —— 屏上按左右键只会长鸣）
            "ro": m.group(4) == "RPI_CFG_T_STR", "warn": False, "min": "", "max": "",
            "options": "" if m.group(5) == "NULL" else opts.get(m.group(5), ""),
            "src": "pi",
        })
    return out


def param_catalog():
    """「参数」弹窗的整份清单: {"beam": [...], "pi": [...], "names": [全部名字]}。

    分组信息就在每项的 ``group`` 里（前端按它归类, 保序）。``names`` 给保存校验用
    （``_check(valid_params=...)``: 不在清单里的名字 = 固件会显示 ? 的死项）。

    ⚠️ 每次现扫（两份源码, 几百行正则）—— 不做缓存: 缓存会让「刚在 app_beam.c 里加
       的参数」选不到, 那种"页面看着对、就是选不着"最难查。
    """
    beam = parse_beam_params()
    pi = parse_rpi_params()
    return {"beam": beam, "pi": pi,
            "names": [it["name"] for it in beam] + [it["name"] for it in pi]}


# ==================== 字表（自动补字） ====================

def read_glyphs():
    p = Path(GLYPHS_TXT)
    if not p.exists():
        return ""
    return p.read_text(encoding="utf-8")


def listed_han(text):
    """glyphs.txt 里已经列出的汉字集合。"""
    out = set()
    for line in text.split("\n"):
        line = line.split("#", 1)[0]
        for c in line:
            if _is_han(c):
                out.add(c)
    return out


def missing_han(model):
    """模型里用到、但 glyphs.txt 还没列出的汉字（去重保序）。

    ⚠️ 2026-10-18 起把**显示映射的短标签**也算进"用到"（OLED_ITEM_PM 的 map 段,
    如 "1=手|3=自" 的 手/自）—— 它们是运行期字符串, 跟菜单标签一样会上屏;
    不收进来就是下一个「白平衡:?定」。"""
    labels = []
    for name in model["order"]:
        for it in model["lists"][name]["items"]:
            labels.append(it["label"])
            mp = str(it.get("map") or "")
            if mp:
                labels += [seg.split("=", 1)[1] for seg in mp.split("|") if "=" in seg]
    return [c for c in collect_han(labels) if c not in listed_han(read_glyphs())]


def _append_glyphs(chars):
    """把新汉字**每个单独成行**追加到 glyphs.txt 末尾（每行 = 屏幕上一行, 见 gen_font）。"""
    if not chars:
        return False
    text = read_glyphs()
    if text and not text.endswith("\n"):
        text += "\n"
    text += "\n# ==== 由 Web 控制台「OLED」Tab 自动补字 ====\n"
    text += "".join(c + "\n" for c in chars)
    _backup_then_write(Path(GLYPHS_TXT), text)
    return True


# ==================== 外部工具（字模生成 / 覆盖校验） ====================

def _run_py(script, *args):
    """跑 tools/fontgen 下的脚本, 返回 (returncode, stdout, stderr)。

    ⚠️⚠️ **必须注入 PYTHONUTF8=1**: 本机控制台是 GBK, ``check_coverage.py`` 会打印
    ``✓``/``✗``(U+2713/U+2717), 编码不出去就抛 UnicodeEncodeError 并以**退出码 1**
    结束 —— 即使覆盖其实是完整的。不注入就会把「成功」误判成「失败」。
    """
    env = dict(os.environ)
    env["PYTHONUTF8"] = "1"
    env["PYTHONIOENCODING"] = "utf-8"
    try:
        p = subprocess.run([sys.executable, str(script)] + list(args),
                           capture_output=True, env=env, cwd=str(ROOT), timeout=180)
    except (OSError, subprocess.SubprocessError) as exc:
        return 1, "", "无法执行 %s: %s" % (script, exc)
    dec = lambda b: (b or b"").decode("utf-8", "replace")
    return p.returncode, dec(p.stdout), dec(p.stderr)


def _backup_then_write(path, text):
    path = Path(path)
    if path.exists():
        shutil.copy2(path, path.with_suffix(path.suffix + ".bak"))
    map_codec.write_text_like(path, text)


# ==================== 对外: 读 / 写 ====================

def _mt(path):
    try:
        return Path(path).stat().st_mtime
    except OSError:
        return None


def version():
    return {"c": _mt(MENU_C), "h": _mt(MENU_H), "glyphs": _mt(GLYPHS_TXT)}


def state():
    """→ {version, order, lists, pages, missing_glyphs}（只读, 给 GET /api/oled）。"""
    model = parse_region(Path(MENU_C).read_text(encoding="utf-8"))
    model["version"] = version()
    model["missing_glyphs"] = missing_han(model)
    return model


WRITE_LOCK = threading.RLock()


def _serialized(fn):
    @functools.wraps(fn)
    def wrapper(*args, **kwargs):
        with WRITE_LOCK:
            return fn(*args, **kwargs)
    return wrapper


def _version_ok(base_version):
    if base_version is None:
        return True
    try:
        cur = version()
        return all(abs(float(base_version.get(k, 0)) - (cur[k] or 0)) < 1e-6
                   for k in ("c", "h", "glyphs"))
    except (AttributeError, TypeError, ValueError):
        return False


CONFLICT = (False, {"error": "conflict",
                    "message": "app_oled_menu 已在外部被修改, 请先「重新加载」再保存"}, 409)


@_serialized
def save(payload, base_version, valid_params=None, param_opts=None):
    """写回菜单。payload = {order, lists, pages}。返回 (ok, resp, status)。

    ``valid_params`` 透传给 _check（通用调参行的参数名白名单, 由 server 从
    Beam 表 / Pi 参数表解析后给; 不给就跳过"名字存不存在"那条校验）。
    ``param_opts``（2026-10-18 加）同样透传: {参数名: 真选项表} —— 显示映射的
    值域校验用（见 _check）。

    流程（**事务性**: 任一步失败就把已写的文件回滚）::

        ① 校验（失败 400, 一个字节不写）
        ② 写 app_oled_menu.c（只有字节真变才写 + .bak）
        ③ 补字: 差集追加进 glyphs.txt（含映射短标签的字, 见 missing_han）
        ④ 跑 gen_font.py 重写 Core/Src/OLED_Data.c
        ⑤ 失败 → 回滚 ②③
        ⑥ 跑 check_coverage.py, 非 0 作为 warning 回给前端（不阻断）
    """
    if not _version_ok(base_version):
        return CONFLICT

    model = {"order": list(payload.get("order") or []),
             "lists": payload.get("lists") or {},
             "pages": payload.get("pages") or [],
             "page_num_raw": payload.get("page_num_raw") or ""}
    if not model["order"] or not model["pages"]:
        return False, {"error": "invalid", "message": "菜单是空的"}, 400
    for name in model["order"]:
        if name not in model["lists"]:
            return False, {"error": "invalid", "message": "order 里的 %s 没有对应表" % name}, 400

    # 自动把子表排到引用它的表之前（合法顺序下是恒等变换, 见 _stable_topo_order）
    model["order"], cycle = _stable_topo_order(model["order"], model["lists"])
    if cycle:
        return False, {"error": "invalid",
                       "message": "这些表互相引用成环了: %s" % "、".join(cycle)}, 400

    err = _check(model, valid_params, param_opts)
    if err:
        return False, {"error": "invalid", "message": err}, 400

    # ---- 备份现场（回滚用）----
    # ⚠️ snap 必须罩住 DATA_C（2026-10-09 补）: ④ 会重写 Core/Src/OLED_Data.c,
    #    gen_font 中途失败（写一半/磁盘满）会留下半个字库文件 —— 不回滚它, 下次
    #    编译直接坏。测试此前把 _run_py 整个 mock 掉, 看不见这条。
    # ⚠️ 快照用**字节**而不是 read_text: read_text 的通用换行会把 CRLF 规范化成 LF,
    #    回滚就不再是「原样恢复」（write_text_like 保 CRLF 的纪律在回滚路径上漏了个洞）。
    vc, vg = MENU_C, GLYPHS_TXT
    snap = {p: (Path(p).read_bytes() if Path(p).exists() else None)
            for p in (vc, vg, DATA_C)}
    c_text = Path(vc).read_text(encoding="utf-8")
    new_c = "\n".join(_replace_region(c_text.split("\n"), serialize_region(model)))

    changed = (new_c != c_text)
    added = missing_han(model)

    # 真·无操作保存: 菜单没变、也没新汉字 → 一个字节都不写、一个子进程都不起。
    # （这也是「无操作保存 mtime 不动」那条纪律能成立的前提。）
    if not changed and not added:
        return True, {"ok": True, "version": version(), "changed": False,
                      "glyphs_added": [], "warnings": []}, 200

    try:
        if changed:
            _backup_then_write(Path(vc), new_c)
        if added:
            _append_glyphs(added)

        # ⚠️ 只在**真加了新字**时才重跑字模生成 —— 否则每次保存都白重写一遍
        #    Core/Src/OLED_Data.c（多一次写盘 + 多一次 Pillow 调用）。
        if added:
            rc, out, errout = _run_py(GEN_FONT_PY)
            if rc != 0:
                raise OledMenuError("字模生成失败（gen_font.py 退出码 %d）:\n%s"
                                    % (rc, (errout or out).strip()[-800:]))
    except Exception as exc:                                   # 回滚后再抛
        for p, data in snap.items():
            if data is None:
                continue
            Path(p).write_bytes(data)
        raise OledMenuError("保存失败, 已回滚: %s" % exc)

    rc2, out2, err2 = _run_py(CHECK_COVERAGE_PY)
    warnings = []
    if rc2 != 0:
        warnings.append("字库覆盖校验未通过（check_coverage.py 退出码 %d）:\n%s"
                        % (rc2, (err2 or out2).strip()[-800:]))

    return True, {"ok": True, "version": version(), "changed": changed,
                  "glyphs_added": added, "warnings": warnings}, 200
