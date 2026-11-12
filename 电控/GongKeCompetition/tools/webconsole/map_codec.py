#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""map.c 行级编解码引擎 —— 纯函数, 零文件 I/O, 零项目路径常量。

从 server.py 原样搬出（不重写一行逻辑）, 目的是让「机械臂」Tab 的
app_arm_action.c 解析器（arm_action.py）能复用同一套行解析/序列化规则,
而不必把 server.py 变成公共库, 也不必复制第二份。

server.py 仍以 `from map_codec import (...)` 把这批名字绑回自己的模块命名空间,
所以 server._parse_body / server._serialize_step / server.serialize_map
等既有引用（含 test_server.py / test_bt.py）一位不变。

数组序列化铁律（与 server.py 顶部 docstring 一致）:
  · 只替换数组体, 文件其余字节原样保留; 未修改条目按原始物理行回写
    → 无操作保存 = 字节级 diff 为空
  · 单步 disable = 每物理行行首加 "//"（保留缩进）
  · 分区 disable = 整段用 /* ... */ 各自单独成行包裹, 内部各步启停状态不变;
    包裹时标题行 /* ===== x ===== */ 转成 // ===== x ===== 避免内层 */ 提前闭合

⚠️ 这里只放**与具体文件无关**的东西。要写某个 .c 文件请留在调用方
（server.py 的 parse_map/write_map, arm_action.py 的 parse_arm/write_arm）——
本项目要求「无操作保存字节级零 diff」, 而零 diff 的前提是调用方保留原始行。
    唯一的例外是 write_text_like()（见下）: 它不属于"某个具体文件", 而是三个写盘
    模块（server / tuning / arm_action）**必须共用同一份**的行尾保护 —— 放在这里
    就是为了只有一处实现可漂移。
"""
import re
from pathlib import Path


# 多预设: 每张表 = 一个 stScriptStepTdf 数组; 末尾 #define 选当前跑哪张
RE_ARRAY_DEF  = re.compile(r"^\s*const\s+stScriptStepTdf\s+(\w+)\s*\[\s*\]\s*=\s*\{\s*$")
RE_ACTIVE_DEF = re.compile(r"^(\s*#define\s+SCRIPT_PRESET_ACTIVE\s+)(\w+)(\s*)$")
RE_PRESET_HDR = re.compile(r"^\s*/\*\s*=+\s*预设\s*[:：]\s*(.+?)\s*=+\s*\*/\s*$")


def write_text_like(path, text):
    """把 text 写回 path, **按该文件原有的换行风格**（CRLF 文件仍写 CRLF）。

    ⚠️ 为什么必须有这一层: 读用的是 `read_text()` 默认的 universal newlines
    （`\\r\\n` → `\\n`）, 写用的是 `write_text(..., newline="")`（不做任何转换）——
    于是**整份文件的每一行行尾都会被静默改写**。本机受管文件现在都是 LF 所以不发作,
    但仓库没有 `.gitattributes`, Windows 队友 `core.autocrlf=true` 检出即 CRLF:
    那时只要有一次**真实编辑**（内容没变时不会走到这里）, 「只替换数组体、其余字节
    原样保留」这条核心契约就静默失效, diff 变成整文件, `.bak` 与工作区全线不一致。

    写盘前现读一次原文件探测风格（受管文件都是几十 KB, 可忽略）。
    """
    p = Path(path)
    try:
        crlf = b"\r\n" in p.read_bytes()
    except OSError:
        crlf = False                   # 文件不存在/读不到: 按 LF 写（首次创建）
    # 用 open(newline="") 而不是 Path.write_text(newline=): 后者 3.10 才有该参数,
    # 文档承诺 3.8+（3.8/3.9 第一次写盘就 TypeError）
    with open(p, "w", encoding="utf-8", newline="") as fh:
        fh.write(text if not crlf else text.replace("\n", "\r\n"))


def _split_top(s, sep):
    """按括号深度 0 的 sep 切分（字符串内/括号内不切）。"""
    out, depth, in_str, cur = [], 0, False, []
    for ch in s:
        if ch == '"':
            in_str = not in_str
        elif not in_str:
            if ch in "([{":
                depth += 1
            elif ch in ")]}":
                depth -= 1
            elif ch == sep and depth == 0:
                out.append("".join(cur))
                cur = []
                continue
        cur.append(ch)
    if "".join(cur).strip():
        out.append("".join(cur))
    return out


def _locate_presets(lines):
    """扫描全部 stScriptStepTdf 数组体 → [{ident, label, hdr, open, close}]。
    label 取自数组前的 /* ===== 预设: xxx ===== */ 注释; hdr 为该注释行索引(无则 None)。"""
    out, i = [], 0
    while i < len(lines):
        m = RE_ARRAY_DEF.match(lines[i])
        if not m:
            i += 1
            continue
        ident = m.group(1)
        j = i + 1
        while j < len(lines) and not lines[j].strip().startswith("};"):
            j += 1
        label, hdr = ident, None
        # 往回最多看 5 行找表头, 但**撞到上一张表的收尾 `};` 就停** —— 否则
        # 「上一张表的表体很短（≤3 行）+ 这张表没有自己的表头」时, 会把**上一张表的表头**
        # 认成自己的: 那时删/改这张表会连着把上一张表的表头一起删/改名（静默换名）。
        for k in range(i - 1, max(-1, i - 6), -1):
            if lines[k].strip().startswith("};"):
                break
            hm = RE_PRESET_HDR.match(lines[k])
            if hm:
                label, hdr = hm.group(1), k
                break
        out.append({"ident": ident, "label": label, "hdr": hdr, "open": i, "close": j})
        i = j + 1
    return out


def _strip_comment_marks(line):
    """剥掉行首（可多重）的 // 注释标记, 返回剩余内容。"""
    out = line.lstrip()
    while out.startswith("//"):
        out = out[2:].lstrip()
    return out


def _split_call(s):
    """'MACRO(args)[,] [// comment]' → (macro, args, comment)。"""
    s = s.strip()
    p0 = s.find("(")
    if p0 < 0:
        return None
    depth, in_str = 0, False
    for i in range(p0, len(s)):
        ch = s[i]
        if ch == '"':
            in_str = not in_str
            continue
        if in_str:
            continue
        if ch == "(":
            depth += 1
        elif ch == ")":
            depth -= 1
            if depth == 0:
                tail = s[i + 1:].strip().lstrip(",").strip()
                comment = tail[2:].strip() if tail.startswith("//") else ""
                return s[:p0].strip(), s[p0 + 1:i], comment
    return None


def _split_args(argstr):
    """按括号深度 0 的逗号切参数（字符串内逗号不切）。"""
    return [a.strip() for a in _split_top(argstr, ",")]


def _is_wrap_marker(line, ch):
    """该物理行是否为独立的 /* 或 */ 包裹标记行。"""
    return line.strip() == ch


def _parse_section_header(line):
    """分区标题行 → (title, full_text); 非标题行返回 None。
    两种形式都认:  /* ===== 标题 ===== */  和(被包裹时的)  // ===== 标题 ====="""
    s = _strip_comment_marks(line).strip()
    s = s.strip("/*").strip()
    if "=====" not in s:
        return None
    parts = [p.strip(" =") for p in s.split("=====") if p.strip(" =")]
    if not parts:
        return None
    return parts[0], " ".join(parts)


def _parse_body(body):
    """数组体行列表 → 条目列表。条目保留原始物理行。"""
    items, pend, depth = [], [], 0
    pending_wrap = False

    def flush():
        nonlocal pend, depth, pending_wrap
        if not pend:
            return
        item = {"id": "i%d" % len(items)}
        all_commented = all(ln.startswith("//") for ln in pend if ln.strip())
        content0 = _strip_comment_marks(pend[0])
        sec = _parse_section_header(pend[0]) if len(pend) == 1 else None

        if sec:
            item.update(kind="section", lines=list(pend),
                        disabled=all_commented or pending_wrap,
                        wrap=pending_wrap,
                        title=sec[0], text=sec[1])
        elif content0.startswith("SCRIPT_STEP"):
            content = "\n".join(_strip_comment_marks(ln) if ln.startswith("//")
                                else ln.lstrip() for ln in pend)
            call = _split_call(content.strip())
            if call:
                macro, argstr, primitive = call
                args = _split_args(argstr)
                item.update(kind="step", lines=list(pend), disabled=all_commented,
                            macro=macro, step=args[0] if args else "",
                            tag=(args[1].strip('"') if len(args) > 1 else ""),
                            params=args[2:], primitive=primitive)
            else:
                item.update(kind="raw", lines=list(pend))
        else:
            item.update(kind="raw", lines=list(pend))
        items.append(item)
        pend, depth, pending_wrap = [], 0, False

    for ln in body:
        # 分区包裹标记: 吸收进相邻 section 的 wrap 标志, 不作为独立条目
        if not pend and _is_wrap_marker(ln, "/*"):
            pending_wrap = True
            continue
        if not pend and _is_wrap_marker(ln, "*/"):
            continue                                        # 包裹收尾标记, 由 wrap 标志重新生成
        if not pend and not ln.strip():
            items.append({"id": "i%d" % len(items), "kind": "raw", "lines": [ln]})
            continue

        pend.append(ln)
        work = ln
        while work.lstrip().startswith("//"):
            work = work.lstrip()[2:]
        work = work.split("//")[0]
        depth += work.count("(") - work.count(")")
        if depth <= 0:
            depth = 0
            stripped = work.strip()
            # 收尾条件: ①步骤行以 ,/)/ */ 结尾 ②分区标题行(以 = 结尾)
            #           ③独立注释行(不以 SCRIPT_STEP 开头, 不可与后续步骤行合并成一条)
            if (work.rstrip().endswith((",", ")", "*/")) or _parse_section_header(ln)
                    or (len(pend) == 1 and stripped and not stripped.startswith("SCRIPT_STEP"))):
                flush()
    flush()

    return items


def _to_line_comments(lines):
    """块注释 → 行注释。分区被 /* */ 包裹时用:
    组内若残留独立 /* ... */ , 其 */ 会提前闭合外层包裹注释, 使后续行变成有效代码。"""
    out, in_block = [], False
    for ln in lines:
        s = ln.strip()
        indent = ln[:len(ln) - len(ln.lstrip())]
        if not s:
            out.append(ln)
            continue
        if in_block:
            if s.endswith("*/"):
                in_block = False
                body = s[:-2].strip().lstrip("*").strip()
            else:
                body = s.lstrip("*").strip()
            out.append(indent + ("// " + body if body else "//"))
        elif s.startswith("/*") and s.endswith("*/") and len(s) > 4:
            out.append(indent + "// " + s[2:-2].strip())
        elif s.startswith("/*"):
            body = s[2:].strip()
            if body.endswith("*/"):
                body = body[:-2].strip()
            else:
                in_block = True
            out.append(indent + ("// " + body if body else "//"))
        else:
            out.append(ln)
    return out


def _to_block_comments(lines):
    """行注释 → 块注释（分区取消包裹时还原, 与 _to_line_comments 对称）。"""
    out = []
    for ln in lines:
        s = ln.strip()
        indent = ln[:len(ln) - len(ln.lstrip())]
        if s.startswith("//"):
            body = s[2:].strip()
            out.append((indent + "/* " + body + " */") if body else (indent + "/* */"))
        else:
            out.append(ln)
    return out


def _toggle_disable(lines, disabled):
    """单步 disable = 每物理行行首加 '//'（保留缩进）; enable = 去行首 '//'。"""
    out = []
    for ln in lines:
        currently = ln.startswith("//")
        if disabled and not currently:
            out.append("//" + ln)
        elif not disabled and currently:
            rest = ln[2:]
            if rest.lstrip().startswith("//"):          # 历史多重注释: 全剥并规范缩进
                out.append("    " + _strip_comment_marks(rest))
            else:
                out.append(rest)
        else:
            out.append(ln)
    return out


# =====================================================================
# 写回路径的输入清洗 —— **唯一来源**（bt_link / arm_action / server 都从这里取）
# =====================================================================
# 用户起的名字（积木名 tag / 分区标题 / 预设名 / 备注）会插进 map.c 的两种上下文:
#   ① C 字符串字面量 `"标签"`
#      · `"` / `\` 破坏字面量本身;
#      · `//` 会让 _ensure_comma 在行内**第一个** `//` 处插尾逗号 → 逗号插进字符串里;
#      · 不成对的 ASCII `()` 让 _parse_body 的括号计数错位 → 条目被错误刷出 ——
#        表现为**重载时悄悄改写邻近条目**, 比编译报错难查得多。
#        （全角 `（）` 安全, map.c 里已有先例。）
#   ② C 注释 `/* ===== X ===== */` / `// 备注`
#      · `*/` 提前闭合注释 → 后面半行变成**活的 C 代码**;
#      · `/*` 是嵌套注释, AC5 报 #9-D;
#      · 换行把一行拆成两行, 解析错位。
# 所以有两套规则。以前它们散在 bt_link / arm_action / server 里（共四份）——
# 规则漂移过一次就会在某个入口漏掉一类字符, 收敛到这里。
#
# ⚠️ 这几个函数都是**幂等**的, 这不是洁癖: make_arm_snippet 洗一遍、_serialize_step
#    再洗一遍是常态（两条路都要能单独用）, 不幂等就会洗出两个不同结果 ——
#    test_bt.py::test_matches_serializer_byte_for_byte 逐字节比对这两条路。
TAG_MAX_LEN = 24        # 与旧的 bt_link.TAG_MAX_LEN / arm_action.MAX_TITLE_LEN 同值


def sanitize_tag(name, max_len=None, fallback=""):
    """用户起的标签 → 能安全放进 map.c 字符串字面量的文本。

    ⚠️ **不洗顶层逗号**: _split_top 跟踪 in_str, 引号内的逗号本来就不切, 所以逗号是安全的。
       别「顺手」把它剥掉 —— 那会改掉标签里正常的标点。
    ⚠️ **先截断、再配平括号**。反过来（老写法）会让截断把刚配好的括号重新打断:
       app_script.h 的积木 desc 里就有 7 条因此产出不成对的 `(`, 而积木 desc 正是新拖出来的
       积木名的默认来源（app.js::newItemFromCatalog）—— 真实数据可达, 不只是恶意输入。
    ⚠️ `//` 要**循环**折: 割一次只少一个斜杠, `'a///b'` 割完还是 `'a//b'`, 而
       _ensure_comma 照样会在那个 `//` 处插逗号。幂等性也依赖这一步。
    """
    t = (name or "").strip()
    t = t.replace('"', "'").replace("\\", "/")
    if max_len is not None and len(t) > max_len:
        t = t[:max_len]
    # ⚠️⚠️ **括号配平与 `//` 折叠必须交替跑到不动点**, 不能各跑一遍就完:
    #    删掉一个**夹在两条斜杠之间**的 `(` 会把那两条斜杠**重新拼成一个 `//`**
    #    —— `'a/(/b'` 就是最小例子: 配平括号后得到 `'a//b'`。
    #    原来的顺序是「先折 `//`、后配平」, 于是 `'a/(/b'` 出去的是 `'a//b'` ——
    #    ① 函数**不幂等**（再洗一次变成 `'a/b'`）, 而本文件的契约明确写着"必须幂等"
    #       （make_arm_snippet 洗一遍、_serialize_step 再洗一遍是常态）;
    #    ② 残留的 `//` 会让 `_ensure_comma` 把尾逗号插进**字符串字面量里**、
    #       让 `_parse_body` 从 `//` 处截断 → **下一条步骤被并进这一条**（静默少一步）。
    #    每一轮要么折掉一个 `//`、要么删掉一个不配对的括号, 都让 t 严格变短,
    #    所以这个循环必然收敛（不设轮次上限）。
    while True:
        before = t
        while "//" in t:
            t = t.replace("//", "/")
        # 去掉不配对的 ASCII 括号（成对的留着, 全角的不受影响）
        while t.count("(") > t.count(")") and "(" in t:
            i = t.rfind("(")
            t = t[:i] + t[i + 1:]
        while t.count(")") > t.count("(") and ")" in t:
            i = t.rfind(")")
            t = t[:i] + t[i + 1:]
        if t == before:
            break
    t = t.strip()
    return t or fallback


def sanitize_comment_text(text, max_len=None, fallback=""):
    """用户起的标题/备注 → 能同时安全放进块注释与枚举成员尾注释的文本。

    比 sanitize_tag 多一层（`*/` / 换行 —— 它还会进 `/* xxx */`）, 少一层（不在字符串
    字面量里, 双引号与反斜杠其实无害）; 不过照样套用 sanitize_tag 那一套替换, 免得两处的
    产物长得不一样。

    ⚠️ 与它最早的版本（arm_action.sanitize_comment 自己那一份）**只有一处不同**: 那里
       `//` 只折一次, 于是 `'a///b'` 会留下 `'a//b'`; 这里折干净。那是**有意的修复**
       —— 留着的 `//` 会让 _ensure_comma 把尾逗号插进字符串里（同 sanitize_tag 那条理由）。
       其余输入逐字节相同（test_arm_action.py::TestSanitize 钉着那几条性质）。
    ⚠️ 末尾那次 `[:max_len]` 可能把配好的括号重新截断 → 结果括号不成对。**这在注释里无害**
       （问题只出在字符串字面量与 _parse_body 的括号计数上）, 所以不修。
    """
    t = sanitize_tag(text, None, "")
    t = t.replace("*/", "* /").replace("/*", "/ *")
    t = t.replace("\r", " ").replace("\n", " ")
    t = re.sub(r"\s+", " ", t).strip()
    if max_len is not None:
        t = t[:max_len]
    return t or fallback


def sanitize_block_comment(text, fallback=""):
    """只落进 C 注释的文本（预设头注释 / 备注）→ 只洗会破坏注释与行结构的字符。

    ⚠️ **不截断、也不洗字符串字面量字符**: 预设名要**原样**回吐到
       `/* ===== 预设: X ===== */`, 而 RE_PRESET_HDR 得把它原样读回来 ——
       截断 = 把用户起的长预设名悄悄砍短（server._preset_header_label 的注释专门解释了这条）;
       而双引号、反斜杠、`//` 在注释里本来无害。与「无操作保存零 diff」相容: phase C 只在该预设
       **改过名**时才重写那一行, 洗过之后写回的是稳定值。
    """
    t = (text or "").replace("*/", "* /").replace("/*", "/ *")
    t = t.replace("\r", " ").replace("\n", " ")
    return re.sub(r"\s+", " ", t).strip() or fallback


def _normalize_param(val, is_float):
    """空槽补默认值; float 槽纯数字自动补 .0f, 整数槽非负纯数字自动补 u。

    用户不用自己敲 u/f。⚠️ 只对被编辑过的条目的值生效; 未改动的条目仍按原始行回写,
    所以 map.c 里会出现「有的带 u 有的不带」—— 这是「无操作保存零 diff」的代价, 不是 bug。

    负整数**不补 u**（`-30u` 是无符号取负语义, 放进 int32_t 初始化是坑）。

    ⚠️ **val 可能是 None**: 浏览器里老行参数数组比新 catalog 短时, 直接改尾槽会让
    中间出现**稀疏洞**（it.params[9]=v 跳过 i=8）, JSON.stringify 把洞序列化成 null ——
    2026-10 真崩过（'NoneType' object has no attribute 'strip', 保存 500）。None 按空槽
    处理（补 0 = 「未配置」的默认语义）, 别让它往下漏。"""
    if val is None:
        val = ""
    v = val.strip()
    if not v:
        return "0.0f" if is_float else "0"
    if is_float and re.match(r"^[+-]?\d+$", v):
        return v + ".0f"
    if not is_float and re.match(r"^\d+$", v):
        return v + "u"
    return v


def _ensure_comma(lines):
    """保证 step 条目末行代码以 ',' 结尾（逗号插在行尾 // 注释之前）。"""
    last = lines[-1]
    i = last.find("//")
    code, tail = (last[:i], last[i:]) if i >= 0 else (last, "")
    if code.rstrip().endswith(","):
        return lines
    lines[-1] = code.rstrip() + "," + ((" " + tail.lstrip()) if tail.strip() else "")
    return lines


# `SCRIPT_STEP_*` 宏名里每个家族的参数个数上限 —— **必须等于 app_script.h 里该家族的最大编号**
# （那边 1..上限 一个都不能缺, 见 app_script.h 宏表上方那段警告）。
# 提成模块常量是为了让「生成器」与「测试」同源: test_server.py 直接引用它们去核对头文件,
# 以后加宏只改这两处数字, 不会再出现「上限悄悄失配 → 编出的行实参过多 → Keil 才报 #55-D」。
MACRO_CAP_I = 6      # SCRIPT_STEP_I1..I6（2026-10: 4→6, 单动作抓取的「延时异步臂」延时参数
                     #   追加在 int 末尾, 老行 I4 缺的尾槽补 0 = 与底盘同时开始, 不必重写）
MACRO_CAP_F = 5      # SCRIPT_STEP_F1..F5
MACRO_CAP_F1I = 9    # SCRIPT_STEP_F1I1..F1I9（2026-10-07: 5→7, 单动作抓取加「收敛航向来源」—— float 槽
                     #   gyro_target 插在 int 之前, 老行已全量重写成 F1I6/F1I7; 2026-10-08: 7→8,
                     #   BALL_GRAB_ONE 追加 int 尾槽 cam_pct(融合相机占比), 老行不错位不必重写;
                     #   2026-10-30: 8→9, BALL_GRAB_ONE 再追加 int 尾槽 rev「反向执行」（场地换边镜像,
                     #   SCRIPT_MAX_INT_PARAMS=9 顶格）, 老行缺槽补 0=正向不必重写;
                     #   2026-11: HOSTAGE_GRAB_ONE 追加 place/place_delay → 9 个 int（F1I9, 顶格））
MACRO_CAP_F2I = 9    # SCRIPT_STEP_F2I1..F2I9（**2 个 float + n 个 int**, 2026-09 为支点转弯巡线接管新增;
                     #   2026-10 加尾段盲走 tail_ms → 7、settle 段「启动即返回」settle_early → 8;
                     #   2026-10-08 加融合相机占比 cam_pct → 9。**新槽追加在 int 末尾**, 老行不错位、不必重写）


def _fam_macro(fam, n, cap):
    """`SCRIPT_STEP_<fam><n>` —— **n 超出该家族上限时抛错, 绝不悄悄截到上限**。

    截到上限会编出一行「宏名对、实参多」的行, 一路写进 `map.c`, 直到 Keil 编译才报
    `#55-D: too many arguments in invocation of macro`。2026-09 真踩过: 控制台进程还带着
    旧的 cap 表（`app_script.h` 已经给积木加了参数, 但服务端代码没重启）→ 8 个实参配了
    `SCRIPT_STEP_F1I5` 的宏名 → 两份 map.c 行编译不过。抛错则那次保存**当场失败、盘上文件
    一个字节都不动**, 原因也直接写在报错里。
    """
    if n > cap:
        raise ValueError(
            "SCRIPT_STEP_%s 家族上限 %d, 这次要 %d 个参数 —— 要么同时抬 app_script.h 的宏族"
            "与 map_codec.MACRO_CAP_*，要么**重启控制台**（服务端代码比 app_script.h 旧时"
            "就会这样）。本次保存已放弃, map.c 未被改动。" % (fam, cap, n))
    return "SCRIPT_STEP_%s%d" % (fam, n)


def _serialize_step(item, cat_by_type):
    """编辑过/新增的 step 条目 → 标准行（对齐现行 map.c 风格）。

    ⚠️ 宏名按上面几个上限取 —— 参数个数超过上限时**抛错**而不是降级成小宏（见 `_fam_macro`）。
    ⚠️ float 的个数直接进宏名（1 → `F1I*`, 2 → `F2I*`）—— 现在只存在这两族。
    `test_server.py` 有一条测试拿**真实 catalog 的每个积木**跑一遍这个函数, 核对
    它编出来的宏名在 app_script.h 里真有定义（漏抬家族/上限只会在 Keil 才炸）。
    """
    cat = cat_by_type.get(item.get("step", ""), {})
    n_f, n_i = len(cat.get("params_f", [])), len(cat.get("params_i", []))
    if n_f == 0 and n_i == 0:
        macro = "SCRIPT_STEP"
    elif n_f == 0:
        macro = _fam_macro("I", n_i, MACRO_CAP_I)
    elif n_i == 0:
        macro = _fam_macro("F", n_f, MACRO_CAP_F)
    elif n_f == 2:
        macro = _fam_macro("F2I", n_i, MACRO_CAP_F2I)
    else:
        # n_f == 1 走这里（**现行 catalog 里的"有 float 又有 int"的积木都是 1 个 float**）。
        # ⚠️ n_f ≥ 3 的"又有 float 又有 int"的积木**目前不存在** —— 真出现时这里会抛错
        #    （家族名对不上就装不下）, 得先给 app_script.h 加对应家族并在这儿分支。
        macro = _fam_macro("F1I", n_i, MACRO_CAP_F1I)

    params = item.get("params", [])
    vals = []
    for idx in range(n_f + n_i):
        vals.append(_normalize_param(params[idx] if idx < len(params) else "", idx < n_f))

    # tag 是用户自由文本, 必须过清洗（见 sanitize_tag 上方那段）—— 不过它只影响
    # **edited** 条目, 未改动的条目仍按原始物理行回吐, 所以「无操作保存零 diff」不受影响。
    # ⚠️ 这里**不传 max_len**: 截断不是任何 C/解析约束要求的（一行长一点无所谓）,
    #    而把用户起的名字悄悄砍短比留一行长行糟得多。TAG_MAX_LEN 只约束机械臂那边的
    #    「臂组动作」名字（那里的长度是 bt_link 的对外约定）。
    line = "    %s (%s %s" % (macro.ljust(18), (item.get("step", "") + ",").ljust(37),
                              ('"%s"' % sanitize_tag(item.get("tag", ""))))
    if vals:
        line += ", " + ", ".join(vals)
    line += "),"
    prim = cat.get("primitive") or item.get("primitive") or ""
    if prim:
        line += "  // %s()" % prim
    return [line]


def serialize_map(items, cat_by_type, report=None):
    """条目列表 → 数组体行列表。未修改条目回写原始行（含 disable 切换）。

    report: 可选列表。清洗**真改写了**某个条目的文本时往里追加
            {"what": …, "was": …, "now": …} —— 调用方拿它给用户一句提醒
            （「静默改写用户输入」比不改更糟; 见 sanitize_tag 上方那段）。
            在这里收集而不是让调用方自己再判一遍: 判据（哪个条目算「被编辑过」）
            只有一处, 免得两边漂移。
    """
    last_active_step = -1
    for i, it in enumerate(items):
        if it.get("kind") == "step" and not it.get("disabled"):
            last_active_step = i

    # 分区包裹: 每个 wrap 分区在"下一个分区之前"或数组末尾收尾 → */
    close_at = set()              # 需要在其后补 */ 的条目索引
    cur = None
    for i, it in enumerate(items):
        if it.get("kind") == "section":
            if cur is not None:
                close_at.add(i - 1)
            cur = i if (it.get("wrap") and it.get("disabled")) else None
    if cur is not None:
        close_at.add(len(items) - 1)

    # 被切换过的分区: 组内块注释 ⇄ 行注释 的降级/还原
    #   True  = 转成行注释(包裹态, 防内层 */ 提前闭合外层)
    #   False = 还原块注释(取消包裹态)
    comment_mode = {}
    cur_sec = None
    for i, it in enumerate(items):
        if it.get("kind") == "section":
            cur_sec = None
            if it.get("edited"):
                cur_sec = bool(it.get("wrap") and it.get("disabled"))
                comment_mode[i] = cur_sec
        elif cur_sec is not None and it.get("kind") == "raw":
            comment_mode[i] = cur_sec

    body = []
    for i, item in enumerate(items):
        kind = item.get("kind", "raw")

        if kind == "wrapclose":
            continue                                        # 由 wrap 标志重新生成

        if kind == "section" and item.get("wrap") and item.get("disabled"):
            body.append("    /*")

        if kind == "raw":
            lines = item.get("lines", [])
            if i in comment_mode:
                lines = _to_line_comments(lines) if comment_mode[i] else _to_block_comments(lines)
            body.extend(lines)
        else:
            if item.get("edited") or not item.get("lines"):
                if kind == "section":
                    # 标题是双击改名得来的用户文本, 两种形态都落在注释里 —— 不过清洗的话
                    # 一个 `*/` 就让后面半行变成活的 C 代码（见 sanitize_block_comment）。
                    raw_title = item.get("title", "")
                    title = sanitize_block_comment(raw_title)
                    if report is not None and title != raw_title:
                        report.append({"what": "分区标题", "was": raw_title, "now": title})
                    lines = ["    // ===== %s =====" % title if item.get("wrap")
                             else "    /* ===== %s ===== */" % title]
                else:
                    raw_tag = item.get("tag", "")
                    if report is not None and sanitize_tag(raw_tag) != raw_tag:
                        report.append({"what": "积木名", "was": raw_tag,
                                       "now": sanitize_tag(raw_tag)})
                    lines = _serialize_step(item, cat_by_type)
                if item.get("disabled") and kind != "section":
                    lines = _toggle_disable(lines, True)
            else:
                lines = item["lines"]
                if kind == "step":
                    lines = _toggle_disable(lines, item.get("disabled", False))
            if kind == "step" and not item.get("disabled") and i < last_active_step:
                lines = _ensure_comma(lines)
            body.extend(lines)

        if i in close_at:
            body.append("    */")
    return body


def _make_preset_block(label, ident, body):
    """新预设的完整行块（前置空行与头注释）。"""
    return (["", "/* ===== 预设: %s ===== */" % label,
             "const stScriptStepTdf %s[] = {" % ident] + body + ["};"])


# =====================================================================
# 预设注册表 ——「跳到别的预设」那套积木（STEP_IF_ELSE / STEP_SCRIPT_CALL）的落地点
# =====================================================================
# 固件里没有「预设」这个运行时概念（g_pstScriptActive 是编译期烘死的）, 跳转需要一张
# id → 表 的注册表。控制台管两段生成内容, **必须由同一份列表产出**否则会互相对不上:
#   map.c  注册表区块（`/* ==== 预设注册表: 开始/结束 ==== */` 之间）→ 供固件查表
#   map.h  生成块  （`/* ==== 生成块: 开始/结束 ==== */` 之间）   → emScriptPresetIdTdf
#           + SCRIPT_PRESET_NEXT_ID; 枚举成员尾注释就是「地图」Tab 目标下拉的标签
#           （server.py::parse_enums glob 到 map.h, 前端一行都不用改）。
#
# ⚠️ 两段标记都**不会**被 _locate_presets / RE_PRESET_HDR 误认成预设表:
#    RE_ARRAY_DEF 要求 `const stScriptStepTdf`;
#    RE_PRESET_HDR 要求 `预设` 紧跟 `[:：]`, 而「预设注册表」的 `注` 不是。

RE_PRESET_REG_BEGIN = re.compile(r"^\s*/\*\s*=+\s*预设注册表\s*[:：]\s*开始\s*=+\s*\*/\s*$")
RE_PRESET_REG_END = re.compile(r"^\s*/\*\s*=+\s*预设注册表\s*[:：]\s*结束\s*=+\s*\*/\s*$")
RE_PRESET_GEN_BEGIN = re.compile(r"^\s*/\*\s*=+\s*生成块\s*[:：]\s*开始\s*=+\s*\*/\s*$")
RE_PRESET_GEN_END = re.compile(r"^\s*/\*\s*=+\s*生成块\s*[:：]\s*结束\s*=+\s*\*/\s*$")

RE_PRESET_NEXT_ID = re.compile(r"^\s*#define\s+SCRIPT_PRESET_NEXT_ID\s+(\d+)u\s*$")
RE_PRESET_IDENT = re.compile(r"^g_astScriptPreset(\d+)$")

# 注册表行: { 3u, "标题", g_astScriptPreset3, SCRIPT_PRESET_COUNT(g_astScriptPreset3), 0u },
# 第 5 列 = ucFlags（2026-10-18 加, bit0 = 可重入; 4 字段老行也认 —— 缺省 = 0）。
RE_PRESET_REG_ENTRY = re.compile(
    r'^\s*\{\s*(\d+)u\s*,\s*"((?:[^"\\]|\\.)*)"\s*,\s*(\w+)\s*,\s*'
    r'SCRIPT_PRESET_COUNT\(\s*\w+\s*\)\s*(?:,\s*(\d+)u)?\s*\}\s*,?\s*$')

# 跳转目标参数写成 SCRIPT_PRESET_n —— 用来查「引用了不存在的预设」
RE_PRESET_REF = re.compile(r"^SCRIPT_PRESET_(\d+)$")

# 目标下拉标签里的「不可跳入」标记（全角括号: 与 map.c 的既有风格一致, 也避开解析器
# 对 ASCII 括号的括号计数约束）
PRESET_UNSAFE_MARK = "（不可跳入）"
# 「可重入」标记（2026-10-18 加, 用户口径）: 预设栏勾选「可重入的任务」后写进 map.h
# 枚举注释 —— 作者声明"这张表可以被 STEP_SCRIPT_CALL 当子程序调"（比如 看路/看路转弯
# 这类 OLED 直跑预设, 外面包一层一次性的机械臂初始化再来调用）。带它的表即使内容里有
# 握手/重置（本来会被打上 不可跳入）也不再打 —— 作者的显式声明压过启发式猜测。
PRESET_REENTRANT_MARK = "（可重入）"


def find_marked(lines, begin_re, end_re):
    """找标记区间, 返回 0-based 半开区间 (begin+1, end); 找不到返回 None。

    与 arm_action._find_region 同形, 但**找不到返回 None 而不是抛异常** —— 预设注册表
    对老文件是可选的（首次保存时补上）, 不是硬要求。
    """
    begin = end = None
    for i, ln in enumerate(lines):
        if begin is None and begin_re.match(ln):
            begin = i
        elif begin is not None and end_re.match(ln):
            end = i
            break
    if begin is None or end is None:
        return None
    return begin + 1, end


def parse_preset_registry(lines):
    """map.c 的注册表区块 → {ident: {"id": n, "title": s}}; 没有区块返回 {}。"""
    reg = {}
    span = find_marked(lines, RE_PRESET_REG_BEGIN, RE_PRESET_REG_END)
    if span is None:
        return reg
    for ln in lines[span[0]:span[1]]:
        m = RE_PRESET_REG_ENTRY.match(ln)
        if m:
            reg[m.group(3)] = {"id": int(m.group(1)), "title": m.group(2)}
    return reg


def parse_next_preset_id(lines):
    """map.h 里记录的 SCRIPT_PRESET_NEXT_ID; 没有返回 0。"""
    for ln in lines:
        m = RE_PRESET_NEXT_ID.match(ln)
        if m:
            return int(m.group(1))
    return 0


def preset_call_unsafe(items):
    """这张预设表适不适合被跳入? 返回原因（进下拉标签）或 ""。

    ⚠️ 握手要查**全表**、不是只查表首: 被跳入的帧是从 `usIdx = 0` 起步的, 跳进去
       **不会**跳过表头（`ucScriptResetTarget` 的跳过逻辑只服务「重置脚本」）。
       所以表中间排的握手同样会重跑 —— 而「握手成功之后再执行一次握手」= 判为断开
       重连, 会卡在等一个永远不来的信号上。
    ⚠️ 「重置脚本」也是全表查: 它重启当前帧那张表, 子预设里排它就是自循环、永不返回。
    ⚠️ 禁用（行首 `//`）的条目是注释、不进数组, 不算数 —— 不排除的话会给出假警告。
    """
    active = [it for it in (items or [])
              if it.get("kind") == "step" and not it.get("disabled")]

    if any(it.get("step") == "STEP_WAIT_HANDSHAKE" for it in active):
        return "表内有握手"
    if any(it.get("step") == "STEP_SCRIPT_RESET" for it in active):
        return "表内有重置"
    return ""


def preset_refs(items):
    """一张表里引用到的预设 id 集合（扫所有参数槽 —— IF 的两个去向 + CALL 的目标）。

    ⚠️ 跳过禁用条目: 它们是注释、不进数组, 引用了不存在的预设也**编译得过** ——
       算进来的话 `_preset_warnings` 会对着一个不存在的问题喊「编译会报 undefined
       identifier」, 是假警告。
    """
    out = set()
    for it in (items or []):
        if it.get("disabled"):
            continue
        for v in (it.get("params") or []):
            m = RE_PRESET_REF.match(str(v).strip())
            if m:
                out.add(int(m.group(1)))
    return out


def _wrap_commented(items):
    """返回「被分区包裹（`/* */`）注释掉」的条目下标集合。

    ⚠️ 判据必须与 `serialize_map` 的 `close_at` **逐条一致**: 它是按「每个 wrap 且 disabled
       的分区, 从自己的标题行罩到下一个分区标题之前（或数组末尾）」来补 `/*` 与 `*/` 的。
       这里算出来的与那边不一致, 就会出现最难查的一种错 —— **校验说平衡、写出来的 C 数组
       却不平衡**（一个被罩住的「结束如果」在文件里根本不存在, 而校验没看见）。
       所以改 serialize_map 的包裹规则时, 必须回来同步这个函数。"""
    covered = set()
    start = None
    for i, it in enumerate(items or []):
        if it.get("kind") == "section":
            if start is not None:
                covered.update(range(start, i))     # 上一个包裹区到此为止（不含本分区）
                start = None
            if it.get("wrap") and it.get("disabled"):
                start = i                            # `/*` 撒在本分区标题行**之前**
    if start is not None:
        covered.update(range(start, len(items or [])))
    return covered


def marker_balance_error(items, label=""):
    """嵌套容器 IF（STEP_IF / STEP_ELSE / STEP_END_IF）的配对校验。

    通过返回 ""; 不通过返回一句给用户看的中文说明。调用方是 server.write_map
    （保存前拦下, 错误码 unbalanced_if）—— **必须在写盘之前**, 不配对的表写进去就是
    C 编译错误或者（更糟）固件里那套位置配对扫到表尾把本帧收掉、后半张表静默不执行。

    ⚠️ 数哪些行「真的进数组」的判据, 必须和 serialize_map 实际写出来的行一致:
        · 被禁用（行首 `//`）的步骤是注释, 不进数组;
        · 落在 wrap 分区里的行被 `/* */` 罩住, 也不进数组。
       两者都不数进来 —— 否则「整块禁用」这种正常操作会被判成不配对。
       （⚠️ 这与 `preset_refs` / `preset_call_unsafe` 只看 disabled 不同: 那两个是**提醒**,
          多一句假警告不致命; 这个是**硬拦保存**, 判错就是拦下一条能编译的路线。）
    ⚠️ 分区（section）不进数组、也不是标记, 所以分支里放分区标题是**合法**的 ——
       前面「前端把分区切分限制在深度 0」是**渲染**纪律（否则 IF 头与它的结束标记会分居
       两个 <section>）, 不是 C 层的约束。要拦的只是「分区包裹把配对的一半罩住了」,
       而那已经由 _wrap_commented 算进「不进数组」里了。
    """
    commented = _wrap_commented(items)
    open_stack = []          # 未闭合的「如果」: [[条目下标, 是否已见过「否则」], ...]

    def where(i):
        tag = (items[i].get("tag") or "").strip()
        return '第 %d 项%s' % (i + 1, ('（%s）' % tag) if tag else '')

    for i, it in enumerate(items or []):
        if it.get("kind") != "step" or it.get("disabled") or (i in commented):
            continue
        step = it.get("step", "")

        if step == "STEP_IF":
            open_stack.append([i, False])
        elif step == "STEP_ELSE":
            if not open_stack:
                return ('预设「%s」%s 是「否则」, 但它上面没有配对的「如果」—— '
                        '「如果/否则/结束如果」三块标记必须成对。' % (label, where(i)))
            if open_stack[-1][1]:
                return ('预设「%s」%s 又出现一块「否则」—— 一个「如果」只能有一个「否则」。'
                        % (label, where(i)))
            open_stack[-1][1] = True
        elif step == "STEP_END_IF":
            if not open_stack:
                return ('预设「%s」%s 是「结束如果」, 但它上面没有配对的「如果」。'
                        % (label, where(i)))
            open_stack.pop()

    if open_stack:
        i = open_stack[-1][0]
        return ('预设「%s」%s 的「如果」没有配对的「结束如果」—— 分支没封口, '
                '固件会把这半张表剩下的一起跳过。⚠️ 被禁用/被分区注释掉的标记不算数。'
                % (label, where(i)))
    return ""


def serialize_preset_registry(entries):
    """map.c 注册表区块的行（**不含**标记行）。entries 按**文件顺序**给。

    ⚠️ **每一张表都必须有一行** —— 一个都不能少。曾经把「标识不是 g_astScriptPresetN」
       的表排除在外, 结果表名全不合规时生成了一个**空初始化列表** `= { };`:
       AC5 报 `#1514: an empty initializer is invalid for an array with unspecified bound`,
       而固件里 app_script.c 无条件引用 g_astScriptPreset / g_usScriptPresetNum ——
       整个工程编译不过, 报错还指在控制台自己生成的区块上。
       表名是什么形式**不影响**它能不能进注册表: 注册表才是 id 的真值来源。
    行内刻意**不按列对齐**: 标题是中文, 按字符数补空格对不齐, 不如不补。
    第 5 列 = ucFlags（2026-10-18 加）: bit0 = 可重入（来自「可重入的任务」勾选,
    与 map.h 枚举注释的 （可重入） 标记同源 —— 都出自 entries 的 reentrant 字段）。
    """
    out = ["const stScriptPresetTdf g_astScriptPreset[] = {"]
    for e in entries:
        if e.get("id") is None:
            # 上游漏配 id = 会写出空数组/短数组（数组里少一行 → 固件按 id 查不到表）。
            # 宁可抛出来, 也别写一个编译不过或行为不对的文件。
            raise ValueError("预设 %s 没有分到 id —— 每张表都必须进注册表" % e.get("ident"))
        flags = 1 if e.get("reentrant") else 0
        out.append('    { %du, "%s", %s, SCRIPT_PRESET_COUNT(%s), %uu },'
                   % (e["id"], e["title"], e["ident"], e["ident"], flags))
    if len(entries) == 0:
        raise ValueError("预设注册表为空 —— 至少要有一条（空初始化列表是 C 语法错误）")
    out.append("};")
    out.append("const uint16_t g_usScriptPresetNum = SCRIPT_PRESET_COUNT(g_astScriptPreset);")
    return out


def serialize_preset_header(entries, next_id):
    """map.h 生成块的行（**不含**标记行）。entries 按 **id 升序**给。

    按 id 排而不是按文件顺序: 这个顺序就是下拉里的顺序, 按 id（= 创建顺序）排的话
    在 map.c 里上下搬动预设表不会让下拉跳来跳去。

    条目可带 ``reentrant``（2026-10-18 加）: 预设栏「可重入的任务」勾选的状态,
    落成枚举注释里的 PRESET_REENTRANT_MARK; 带它就**不再**追加 不可跳入 标记
    （作者显式声明可被 STEP_SCRIPT_CALL 调入, 压过"表内有握手/重置"的启发式）。
    """
    out = ["typedef enum {",
           "    SCRIPT_PRESET_NONE = 0,   /* 继续（不跳转） */"]
    for e in sorted([e for e in entries if e.get("id") is not None],
                    key=lambda x: x["id"]):
        label = e["title"]
        if e.get("reentrant"):
            label += PRESET_REENTRANT_MARK
        elif e.get("unsafe"):
            label += PRESET_UNSAFE_MARK
        out.append("    %-18s = %d,   /* %s */"
                   % ("SCRIPT_PRESET_%d" % e["id"], e["id"], label))
    out.append("} emScriptPresetIdTdf;")
    out.append("#define SCRIPT_PRESET_NEXT_ID    %du" % next_id)
    return out


def parse_preset_reentrant(lines):
    """map.h 的生成块枚举行 → 带「可重入」标记的预设 id 集合。

    ⚠️ 逐行锚定 SCRIPT_PRESET_n = … 的枚举成员形状（生成块里才有这种行）。
    这是「可重入」标记的**持久化读回**口: map.h 是入库文件, 重启/队友拉取都在。
    """
    rx = re.compile(
        r'^\s*SCRIPT_PRESET_(\d+)\s*=\s*\d+\s*,\s*/\*.*?'
        + re.escape(PRESET_REENTRANT_MARK) + r'.*?\*/\s*$')
    out = set()
    for ln in lines:
        m = rx.match(ln)
        if m:
            out.add(int(m.group(1)))
    return out
