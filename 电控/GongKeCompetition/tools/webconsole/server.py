#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GongKeCompetition Web 控制台 — 本地服务 + map.c 解析/序列化引擎。

绑 0.0.0.0（见 BIND_HOST）: 队友用局域网地址就能打开同一份控制台、编辑同一份源码。
  · 启动时打印「局域网」那一条地址, 直接发给队友; 本机仍自动开回环地址。
  · 跨站防护照旧: 受保护的 POST 只放行「本机地址 + Host 一致」的 Origin（见 origin_ok）。
  · 多人同时编辑靠既有的 base_version/409 乐观锁 —— 别削弱它; 写盘另加锁串行化
    （见 WRITE_LOCK）。多人使用的注意事项写在 CLAUDE.md「多客户端」一节。

Tab 1「地图」: 把 Application/map.c 的 g_astScriptTest[] 路线表变成可视化积木。
  GET  /                → static/index.html
  GET  /static/<file>   → 静态资源
  GET  /api/rev         → 三个 Tab 的版本戳 + app_script.h（前端轮询它发现「队友改了」）
  GET  /api/catalog     → 积木库元数据（完全从源码派生, 无硬编码）
  GET  /api/map         → 解析 map.c 数组体 → JSON 条目列表 + 版本戳(mtime)
  POST /api/map         → 序列化回写 map.c（.bak 备份; base_version 不匹配回 409）
  POST /api/meta        → 元数据写回 app_script.h（右键编辑积木说明/参数中文名等）

Tab「调参」: 把 Application/app_config.h 的 #define 做成可调参数（解析/序列化在 tuning.py）。
  GET  /api/tuning         → 分区树 + 每个参数的显示名/单位/转换/范围/备注 + 可见性/顺序
                             （标定 Tab 托管的 CALIB_PARAMS 不在这里下发, 见下）
  POST /api/tuning         → 回写改过的值与新增参数（.bak 备份; base_version 不匹配回 409）
  POST /api/tuning/meta    → 只重写某参数的尾注释（显示名/单位/转换/范围/备注）
  POST /api/tuning/prefs   → 存 tuning.json（只存可见性与顺序; 元数据在头文件注释里）

Tab「标定」: 主摄标定预设的管理中心（看路/色球/人质/射靶粗调的 17 个宏, 逻辑在 calib.py）。
  GET  /api/calib          → 四组参数元数据 + 当前宏值 + 预设列表（calib.json）
  POST /api/calib          → {action} ∈ save | apply
     save  = 整份预设写 calib.json（不碰 app_config.h）
     apply = 「设为当前」: 预设值写回 app_config.h 的宏（复用 tuning.write_config,
             值全等零写盘）+ calib.json 记 active —— 生效要 Keil 重编译烧录

Tab「位移」: 路线位移变量（app_config.h「路线位移变量」段）的可视化管理（逻辑在 moves.py）。
  变量本体与译名都在 app_config.h, 增删改查复用 tuning.py; 本 Tab 另管一张示意图。
  GET  /api/moves          → 变量清单（地图 Tab 徽章的宏名→译名来源）+ 示意图配置（moves.json）
  GET  /api/moves/image    → 示意图文件本体（路径在 moves.json, 只许仓库内相对路径）
  POST /api/moves          → action 缺省 = 保存示意图（起点/线段绑定, 纯可视化不碰数值）;
                             add_var = 新增变量; delete_var = 删变量（先查 map.c 引用,
                             启用行引用拒绝 —— 删了就是 undefined identifier）

Tab「调试助手」: 宿主侧联机调试（移植自 pro/调试助手/调试助手.py, 逻辑在 host_tools.py）。
  GET  /api/tools          → 本机/树莓派 IP、SSH 串、常用语列表
  POST /api/tools/action   → {action, dry_run?, phrase?} 执行一键动作 → {ok, log[], text?}
                             action ∈ update|rdp|folder|trae|phrase
  POST /api/tools/phrases  → 整表覆盖写 phrases.json
  这两个 POST 会起外部进程/改 ~/.ssh/config, 故加了一层 Origin 校验（见 _origin_ok）。

Tab「蓝牙」: 走蓝牙串口（UART5）读写单片机运行时参数、点动机械臂。
  传输在 bt_serial.py（ctypes 直调 Win32, 保 stdlib-only）, 协议/会话在 bt_link.py。
  GET  /api/bt             → 串口列表 + 连接状态 + 日志尾巴
  POST /api/bt             → {action, ...} → {ok, log[], data?, error?}
     action ∈ refresh | connect | disconnect | list | set | mode | jog | grip | pos
              | arm | play | snippet
       arm  = 播放一组位姿（$B,ARM, 5 轴同时启 S 曲线 + 夹爪, 立即返回）; 到位靠 pos 轮询
       play = 播固件里已编译的动作预设整段（$B,PLAY, 阻塞几秒; 与路线同一个固件函数）
       jog 的 ms=0 = 跟手模式（拖滑杆用）, 见 bt_link.py 顶部
  · 「Beam 参数」面板**完全由设备上报**（$B,LIST）驱动: 参数名/中文名/分组/单位/类型/
    范围/当前值全来自固件 app_beam.c 的注册表, 这里不写死任何默认值。
  · 机械臂编辑模式只在「脚本正停在握手等待阶段 1」时可用（见 app_bt.c），
    所以点动与路线抢不了舵机; 界面据此给出 NOTEDIT/NOHANDSHAKE 的明确提示。
  · 会驱动真实硬件, 故同样加 Origin 校验。

Tab「采集」: 摄像头预览 + 分类采集数据集（逻辑在 capture.py）。
  GET  /api/capture           → 摄像头状态 + 各类别已有张数 + 偏好
  GET  /api/capture/frame.jpg → 当前帧（缩放过的 JPEG；无帧回 404）
  POST /api/capture           → {action, ...} → {ok, log[], data?, error?}
     action ∈ scan | open | close | capture | counts | prefs | reveal
     （close = 释放摄像头设备; 前端「启用摄像头」开关关掉时调它, 见 cap.json 的 enabled）
  · ⚠️ 控制台只有这一个 Tab 带第三方依赖: capture.py 顶部 try import cv2 / except → None。
    opencv-python 是**可选**的 —— 没装时本文件与其余 Tab 一位不受影响, 采集 Tab 自己显示
    pip 提示。别把 cv2 变硬依赖, 也别为了它引 Flask。
  · ⚠️ 预览是**单帧轮询**（前端 canvas 定时换 src）, 不是 MJPEG: 全文件只有 _send 一个
    响应写入口, 它必写 Content-Length。任何流式响应一旦有异常逃到 do_GET 外层的
    `except → _send(500)`, 就会往半个流里插第二个状态行把响应弄坏 —— 轮询在结构上
    没有这个可能。要改流式请先把这条想清楚。
  · 会开真实摄像头、往仓库写图, 故同样加 Origin 校验。

Tab「机械臂」: 台架上录机械臂动作预设（逻辑在 arm_action.py）。
  右栏拖滑杆调位姿 → 左栏记成子积木（位姿/延时）→ 保存 → 这一段合成为**一块大积木**
  出现在「地图」的积木库里, 拖进路线就是一步 STEP_ARM_PLAY_WAIT / _NO_WAIT。
  GET  /api/arm            → 预设列表 + 回收站 + 版本戳({c,h} 两个 mtime)
  POST /api/arm            → {action, ...} → {ok, data?, error?}
     action ∈ save | delete | restore | purge | refs
     （refs = 问 map.c 里有几步引用了这个动作, 删除前弹确认用; 它要读 map.c 故放在这里）
  · 数据在 Application/app_arm_action.c(步骤表) 与 .h(动作 ID 枚举), 解析/回写规则同 map.c:
    未编辑的条目按原始物理行回吐 → **无操作保存两个文件都字节零 diff**。
  · 会写仓库源文件, 故与 /api/tuning 同级做 Origin 校验。

设计要点 —— 一切元数据以源码为唯一数据源（改源码 → 控制台自动跟着变）:
  · 积木清单/分组  ← app_script.h 的 emScriptStepTypeTdf 枚举 + 分组注释
  · 积木说明       ← 枚举成员的尾注释   /* 打开激光（继电器 JIGUANG） */
  · 原语函数       ← app_script.c 每个 case 体第一个函数调用
  · 参数名/中文名/单位/选项 ← app_script.h 参数映射注释的 DSL:
        *   STEP_RPI_SEND_TASK   anI: task_x=任务码{RPI_TASK_QR=二维码识别, ...}
    DSL 语法: 名字[=中文名][[单位]][{值=标签,值=标签}][<显隐:参数名=值>][<启用:参数名=值>][<可空>] , 行尾 // 备注
  · 枚举下拉选项   ← 头文件 typedef enum 的成员 + 成员尾注释作标签
                    + app_script.c 里 (emXxxTdf)I(n) 强制转换确定参数属于哪个枚举
  · 布尔下拉选项   ← 参数提示里的 "1=X 0=Y" 形式, 或 DSL 显式 {1=开,0=关}
  · 参数显隐       ← DSL 尾部 <显隐:参数名=值>: **只在这个积木的另一个参数读到该值时**
                    才在界面上显示本参数。**目前一处都没用到**（语法仍在, 机制仍有效;
                    原来只有巡线收敛那一对用过, 随 2026-09 砍掉融合一起去掉了）。
                    只做字符串相等（1 与 1u 视为相等, 见 app.js::optKey）、只看同一个积木、
                    只支持一个条件; ⚠️ 只影响**显示**, 写盘照样带这个参数（值一个字符不动）。
  · 参数启用       ← DSL 尾部 <启用:参数名=值>（2026-10 加, 与显隐同一套机器）:
                    本参数**永远渲染**, 但只在另一个参数读到该值时输入框可用, 否则禁用
                    （灰的、看得见点不动）。给「勾选框 + 从属输入框」那类积木用:
                    STEP_BUGUANG_KAI 的 pct、STEP_BEEP 的 times/gap_ms。
                    同样只做字符串相等、只看同一个积木、只支持一个条件;
                    ⚠️ 只影响 **disabled**, 写盘照样带这个参数。
  · 可选「无」       ← DSL 尾部 <可空>（2026-11 加）: 本参数的下拉在枚举选项前**追加 0=无**
                    （值 0 = 不执行动作）。只给「放置动作」这类允许缺省的枚举槽用 ——
                    不加标记的动作参数仍然**没有** 0 选项, 避免抓取动作被误选成无。

map.c 序列化铁律:
  · 只替换数组体, 文件其余字节原样保留; 未修改条目按原始物理行回写
    → 无操作保存 = 字节级 diff 为空
  · 单步 disable = 每物理行行首加 "//"
  · 分区 disable = 整段用 /* ... */ 单独成行包裹, 内部各步启停状态不变;
    包裹时标题行 /* ===== x ===== */ 转成 // ===== x ===== 避免内层 */ 提前闭合

仅 stdlib（唯一例外:「采集」Tab 的 opencv-python 是可选依赖, 见 capture.py），Python 3.8+。
启动后自动打开浏览器。
"""

import json
import os
import re
import shutil
import socket
import sys
import threading
import time
import traceback
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit

sys.path.insert(0, str(Path(__file__).resolve().parent))     # 让 host_tools / tuning 可导入（脚本运行时本就在 sys.path[0], 这里兜住被 import 的情况）
import host_tools                                            # noqa: E402
import tuning                                                # noqa: E402
import moves                                                 # noqa: E402
import switches                                              # noqa: E402
import bt_link                                               # noqa: E402
import bt_serial                                             # noqa: E402
import capture                                               # noqa: E402
import map_codec                                             # noqa: E402
import arm_action                                            # noqa: E402
import chassis_action                                        # noqa: E402
import oled_menu                                             # noqa: E402
import calib                                                 # noqa: E402


# map.c 行级编解码的纯函数在 map_codec.py; 在这里重新绑定, 让
# server._parse_body / server._serialize_step / server.serialize_map 等既有引用保持不变。
from map_codec import (                                      # noqa: E402
    RE_ARRAY_DEF,
    RE_ACTIVE_DEF,
    RE_PRESET_HDR,
    _split_top,
    _locate_presets,
    _strip_comment_marks,
    _split_call,
    _split_args,
    _is_wrap_marker,
    _parse_section_header,
    _parse_body,
    _to_line_comments,
    _to_block_comments,
    _toggle_disable,
    _normalize_param,
    _ensure_comma,
    _serialize_step,
    serialize_map,
    _make_preset_block,
    find_marked,
    parse_preset_registry,
    parse_next_preset_id,
    parse_preset_reentrant,
    preset_call_unsafe,
    preset_refs,
    marker_balance_error,
    serialize_preset_registry,
    serialize_preset_header,
    RE_PRESET_REG_BEGIN,
    RE_PRESET_REG_END,
    RE_PRESET_GEN_BEGIN,
    RE_PRESET_GEN_END,
    RE_PRESET_IDENT,
    RE_PRESET_REF,
)


ROOT = Path(__file__).resolve().parents[2]          # tools/webconsole/server.py → 仓库根
MAP_C = ROOT / "Application" / "map.c"
MAP_H = ROOT / "Application" / "map.h"
SCRIPT_H = ROOT / "Application" / "app_script.h"
SCRIPT_C = ROOT / "Application" / "app_script.c"
STATIC_DIR = Path(__file__).resolve().parent / "static"
PORT = host_tools.CONSOLE_PORT     # 端口常量的唯一来源在 host_tools（那里要拼控制台地址）

# 绑 0.0.0.0 = 局域网内队友也能打开（见 _origin_ok 的放行规则与 CLAUDE.md「多客户端」一节）。
# 回环地址照旧自动开浏览器: 本机不该绕一圈网卡。
BIND_HOST = "0.0.0.0"
LOOPBACK_URL = "http://127.0.0.1:%d/" % PORT

# 额外放行的主机名（逗号分隔）。兜住 mDNS / 路由器分配的域名 / 多网卡名 —— 那些名字
# 探测不出来, 只能由使用者显式给。IP 与机器名是自动的, 不用填。
ORIGIN_HOSTS_ENV = "GKS_CONSOLE_HOSTS"

# 多预设: 每张路线表 = 一个 stScriptStepTdf 数组; 文件末尾 #define 选当前跑哪张

# 扫描枚举的头文件（积木参数下拉选项的数据源）
ENUM_GLOBS = ("Application/*.h", "Drivers/QEPack/*.h", "Drivers/QEPack/**/*.h")

# POST body 上限。真实的载荷最大就是整份 map.c 的 JSON 回写（map.c 约 37KB, 带上预设
# 元数据与 JSON 开销约 150KB）—— 8MiB 是「挡住明显异常的大体量」而不是紧贴用量,
# 别为了省内存把它压到接近实际值, 那样加一条路线就可能 413。
MAX_BODY_BYTES = 8 * 1024 * 1024

# 单位人性化: 步骤级提示文本 → 输入框单位后缀（参数级未标 [单位] 时的兜底）
UNIT_PATTERNS = [
    (re.compile(r"ms\b|毫秒"), "ms"),
    (re.compile(r"\bmm\b|毫米"), "mm"),
    (re.compile(r"\brpm\b"), "rpm"),
    (re.compile(r"°|\bdeg\b|角度"), "°"),
    (re.compile(r"%|百分比|程度"), "%"),
]


# =====================================================================
# 枚举自动发现
# =====================================================================

def _parse_catalog_sources():
    """parse_catalog / parse_enums 真正会读的文件集合（按路径排序, 含不存在的）。

    加文件时**必须**同步这里, 否则积木库缓存不会因它变化而失效（症状见 catalog_key）。
    """
    paths = set()
    for pattern in ENUM_GLOBS:
        paths.update(ROOT.glob(pattern))
    paths.update((SCRIPT_H, SCRIPT_C,
                  arm_action.ACTION_C, arm_action.ACTION_H, arm_action.TRASH_JSON))
    return sorted(paths, key=lambda p: str(p))


def catalog_key():
    """积木库缓存键 = 它所依赖的那批文件的**磁盘状态**。

    为什么不能只在保存动作里 `catalog_cache = None`: 那只覆盖「本进程自己写」。
    队友 git pull / 手改 app_script.h 之后, 前端的多客户端同步会正确地检测到
    script_h 的 mtime 变了、去拉 /api/catalog, 并提示「积木库已被其他客户端修改, 已刷新」
    —— 但服务端吐回的是**旧缓存**, 于是用户看到提示却看不到任何变化, 非常难查。

    ⚠️ 键里**同时**带路径名与 mtime/size, 不只是 mtime:
       新增或删掉一个 Application/*.h 会改变 parse_enums 的输出, 却不动任何既有文件的 mtime。
    ⚠️ 用 st_mtime_ns 而不是 st_mtime: 后者是有限精度的 float, 同一秒内的两次修改可能
       取到同一个值（测试与其余模块统一用 ns）。
    读不到的文件记 (path, None, None) —— 文件从无到有 / 从有到无同样要算一次变化。
    """
    stamp = []
    for p in _parse_catalog_sources():
        try:
            st = p.stat()
            stamp.append((str(p), st.st_mtime_ns, st.st_size))
        except OSError:
            stamp.append((str(p), None, None))
    return tuple(stamp)


def parse_enums():
    """扫描头文件 → {枚举类型名: [{value, label}]}（label 取成员尾注释）。"""
    enums = {}
    for pattern in ENUM_GLOBS:
        for path in sorted(ROOT.glob(pattern)):
            try:
                text = path.read_text(encoding="utf-8")
            except (OSError, UnicodeDecodeError):
                continue
            for m in re.finditer(r"typedef\s+enum\s*\{(.*?)\}\s*(\w+)\s*;", text, re.S):
                members = []
                for line in m.group(1).splitlines():
                    s = line.strip()
                    if not s or s.startswith("#"):
                        continue
                    lm = re.match(r"([A-Za-z_]\w*)\s*(?:=\s*([^,]+?))?\s*,?\s*(?:/\*(.*?)\*/)?\s*$", s)
                    if not lm:
                        continue
                    label = (lm.group(3) or "").strip()
                    members.append({"value": lm.group(1), "label": label or lm.group(1)})
                if members:
                    enums.setdefault(m.group(2), members)
    return enums


def enum_of_member(enums):
    """成员名 → 枚举类型名 反查表。"""
    rev = {}
    for name, members in enums.items():
        for mem in members:
            rev.setdefault(mem["value"], name)
    return rev


# =====================================================================
# 参数注释 DSL
#   name[=中文名][[单位]][{值=标签,...}][<显隐:参数名=值>]
# =====================================================================

# 「显隐」后缀: <显隐:另一个参数名=值> —— 只有那个参数读到这个值时, 本参数才显示。
# 值是**字符串相等**（前端用 app.js::optKey 归一化, 所以 1 与 1u 算相等）, 不做数值比较。
# 值里不允许 , ; < > —— 前两个是 DSL 上层的顶层切分符, 后两个是本标记自己的边界;
# 出现了就整条拒识, 退化回「整段当参数名」（不报错、不丢字符, 见下面解析末尾的降级路径）。
RE_SHOW_IF = re.compile(r"<\s*显隐\s*:\s*([A-Za-z_]\w*)\s*=\s*([^<>,;]*)\s*>\s*$")

# 「启用」后缀: <启用:另一个参数名=值>（2026-10 加, 与显隐同一套机器）——
# 本参数**永远渲染**, 但只有那个参数读到这个值时输入框才可用（enabled）,
# 否则禁用（灰的、看得见点不动）。给「勾选框 + 从属输入框」那类积木用
# （STEP_BUGUANG_KAI 的自定义亮度、STEP_BEEP 的周期模式）。
# 与显隐的区别就一句话: 显隐改的是**要不要渲染**, 启用改的是 **disabled**。
RE_ENABLE_IF = re.compile(r"<\s*启用\s*:\s*([A-Za-z_]\w*)\s*=\s*([^<>,;]*)\s*>\s*$")

# 「控件」后缀: <控件:名字[:依赖参数名]>（2026-10-23 加）——
# 把这一槽换成一个**专用的前端控件**，而不是按数值/枚举渲染。前端只认名字，
# 参数名/取值仍然全部来自 DSL（不在 app.js 里硬编码任何参数名）。
#   例: `prof=每次档位<控件:warm_profiles:uses>` = 用「每次使用档位」按钮,
#       它要读的 uses（次数）来自同一积木的 `uses` 参数。
# ⚠️ 与显隐/启用同一套解析纪律: 必须在 {选项} / [单位] / 顶层 '=' 之前剥掉。
RE_WIDGET = re.compile(r"<\s*控件\s*:\s*([A-Za-z_]\w*)\s*(?::\s*([A-Za-z_]\w*)\s*)?>\s*$")

# 「可空」后缀: <可空>（2026-11 加）—— 下拉选项在枚举前**追加 0=无**（不执行动作）。
# 只给「放置动作」这类允许缺省的枚举槽用; 不加标记的动作参数仍然没有 0 选项,
# 避免抓取动作被误选成「无」。与显隐/启用/控件同一套解析纪律: 必须在
# {选项} / [单位] / 顶层 '=' 之前剥掉（它挂在行尾, 两个 $ 锚定的正则都认不出）。
RE_ALLOW_NONE = re.compile(r"<\s*可空\s*>\s*$")


def _parse_param_spec(spec, fallback_unit):
    """DSL 单参数 → {name, raw, label, unit, tip, options, show_if?/enable_if?/widget?/allow_none?}。"""
    spec = spec.strip()
    options = None
    show_if = None
    enable_if = None
    widget = None
    allow_none = False

    # ⚠️ 显隐/启用/控件/可空标记必须**最先**剥掉, 早于 {选项} / [单位] / _split_top(spec, "="):
    #   · 选项与单位那两个正则都是 $ 锚定的 —— 标记挂在尾部会让它们双双失配;
    #   · _split_top 只跟踪 ()[]{} 的深度, '<' '>' 不算深度 → 标记里那个 '=' 会被
    #     当成**顶层** '=' 切一刀, 中文名整段拼不回来。
    # 可空先剥: 它是**无参**标记, 可能与其他带参标记叠加（写回时它总在最后）。
    nm = RE_ALLOW_NONE.search(spec)
    if nm:
        allow_none = True
        spec = spec[:nm.start()].strip()
    wm = RE_WIDGET.search(spec)
    if wm:
        widget = {"name": wm.group(1)}
        if wm.group(2):
            widget["depends"] = wm.group(2)
        spec = spec[:wm.start()].strip()
    em = RE_ENABLE_IF.search(spec)
    if em:
        enable_if = {"name": em.group(1), "value": em.group(2).strip()}
        spec = spec[:em.start()].strip()
    sm = RE_SHOW_IF.search(spec)
    if sm:
        show_if = {"name": sm.group(1), "value": sm.group(2).strip()}
        spec = spec[:sm.start()].strip()

    om = re.search(r"\{([^}]*)\}\s*$", spec)
    if om:
        options = []
        for pair in om.group(1).split(","):
            pair = pair.strip()
            if not pair:
                continue
            if "=" in pair:
                v, l = pair.split("=", 1)
                options.append({"value": v.strip(), "label": l.strip()})
            else:
                options.append({"value": pair, "label": pair})
        spec = spec[:om.start()].strip()

    unit = ""
    um = re.search(r"\[([^\]]*)\]\s*$", spec)
    if um:
        unit = um.group(1).strip()
        spec = spec[:um.start()].strip()

    label = ""
    eq = _split_top(spec, "=")                       # 只认顶层 '='; 括号内的(如 kai_he(1=KAI 0=HE))不算
    if len(eq) == 2:
        spec, label = eq[0], eq[1].strip()

    raw = spec.strip()
    name = raw.split("(")[0].strip()
    tip = raw[len(name):].strip("()")
    out = {"name": name, "raw": raw, "label": label or name,
           "unit": unit or fallback_unit, "tip": tip, "options": options}
    if show_if:
        out["show_if"] = show_if       # 只在真有标记时出现, 不为每个参数塞个 null
    if enable_if:
        out["enable_if"] = enable_if   # 同上
    if widget:
        out["widget"] = widget         # 同上: 专用前端控件（见 RE_WIDGET 的说明）
    if allow_none:
        out["allow_none"] = True       # 下拉前追加 0=无（见 RE_ALLOW_NONE 的说明）
    return out


def _parse_params_part(part, fallback_unit):
    """'vx=横移速度[rpm], vy=前进速度[rpm]   // 备注' → ([spec…], note)。
    备注须在逗号切分前剥离, 否则备注里的逗号会被当成参数。"""
    part = part.strip()
    note = ""
    ci = part.find("//")                             # DSL 新格式: 行尾 // 备注
    if ci >= 0:
        note = part[ci + 2:].strip()
        part = part[:ci].strip()
    else:
        m = re.search(r"\s+\(([^()]*)\)\s*$", part)  # 旧格式: 尾部括号备注(前有空白)
        if m:
            note = m.group(1).strip()
            part = part[:m.start()].strip()
    specs = [_parse_param_spec(t, fallback_unit) for t in _split_top(part, ",") if t.strip()]
    return specs, note


def _spec_str(meta):
    """meta → DSL 文本（写回 app_script.h 用）。

    ⚠️ 2026-10-09: label/unit/选项 label 都是用户可写文本, 而参数映射那几行躺在
       app_script.h 的大块 /* */ 文档注释**内部** —— 一个 `*/` 会把整块注释拦腰截断、
       后半变成活代码; 换行会拆掉「一行一积木」的锚定结构。落盘前统一过
       map_codec.sanitize_block_comment（幂等, 与 map/arm/oled 三条写盘路同源）。
       raw/value 不洗: value 来自枚举成员值, raw 是解析产物。"""
    _s = map_codec.sanitize_block_comment
    s = meta.get("raw") or meta.get("name", "")
    label = _s(meta.get("label", ""))
    if label and label != s:
        s += "=" + label
    if meta.get("unit"):
        s += "[" + _s(meta["unit"]) + "]"
    opts = meta.get("options")
    if opts:
        s += "{" + ",".join("%s=%s" % (o["value"], _s(o["label"]))
                            for o in opts) + "}"
    # 显隐/启用/控件标记**永远补在最外一层**（与 _parse_param_spec 里最先剥掉对称）。
    # ⚠️ 前提是 raw 里没有残留标记 —— 剥的那一步必须在 `=中文名` 那次 _split_top 之前,
    # 否则这里会追加第二次, 第一次保存就产生 diff（有测试盯着只出现一次）。
    sw = meta.get("show_if")
    if sw and sw.get("name"):
        s += "<显隐:%s=%s>" % (sw["name"], sw.get("value", ""))
    en = meta.get("enable_if")
    if en and en.get("name"):
        s += "<启用:%s=%s>" % (en["name"], en.get("value", ""))
    # 控件标记同理: 解析时剥掉、回写时补回 —— 漏了这条, 右键编辑一次积木元数据,
    # `<控件:move_var>` / `<控件:warm_profiles>` 就无声消失, 专用控件退化成普通输入框。
    wg = meta.get("widget")
    if wg and wg.get("name"):
        dep = (":" + wg["depends"]) if wg.get("depends") else ""
        s += "<控件:%s%s>" % (wg["name"], dep)
    # 「可空」同理: 无参标记, 总在最外层（解析时最先剥掉）。漏了它 = 改一次中文名,
    # 「放置动作」下拉里的 0=无 就消失 —— 那是本标记存在的唯一理由。
    if meta.get("allow_none"):
        s += "<可空>"
    return s


# =====================================================================
# 积木库 catalog（全部从源码派生）
# =====================================================================

def _expand_step_names(names_field):
    """'STEP_LASER_ON / _OFF' → ['STEP_LASER_ON', 'STEP_LASER_OFF']。"""
    names_field = names_field.strip().rstrip(",")
    if "/" not in names_field:
        return [names_field]
    left, right = [s.strip() for s in names_field.split("/", 1)]
    base = left.rsplit("_", 1)[0]
    return [left, base + right]


def _bool_options_from_tip(tip):
    """提示里 '1=KAI 0=HE' 形式 → 布尔下拉选项; 否则 None。"""
    m = re.search(r"1\s*=\s*(\w+)\s+0\s*=\s*(\w+)", tip or "")
    if m:
        return [{"value": "1", "label": m.group(1)}, {"value": "0", "label": m.group(2)}]
    return None


def parse_catalog():
    """→ [{type, group, desc, hint, note, primitive, unit, params_meta, names}]。"""
    h_text = SCRIPT_H.read_text(encoding="utf-8")
    enums = parse_enums()

    # 1) 参数映射注释块（DSL + 兼容旧格式）
    mapping = {}
    for m in re.finditer(r"^\s*\*\s{1,3}(STEP_\w+(?:\s*/\s*_\w+)?)\s+(.*)$", h_text, re.M):
        names_field = m.group(1).strip()
        rest = m.group(2).strip()
        params_f, params_i, notes = [], [], []

        # 先剥离整行 // 备注: 备注里可能有 ';' 或 ',', 不能参与后续切分
        ci = rest.find("//")
        if ci >= 0:
            if rest[ci + 2:].strip():
                notes.append(rest[ci + 2:].strip())
            rest = rest[:ci].strip()

        if rest.startswith("(无参"):
            notes.append(rest.strip("()"))
        else:
            for seg in _split_top(rest, ";"):        # 顶层分号; 括号内的分号(如备注)不切
                seg = seg.strip()
                pm = re.match(r"(afF|anI):\s*(.*)$", seg, re.S)
                if not pm:
                    continue
                specs, note = _parse_params_part(pm.group(2), "")
                (params_f if pm.group(1) == "afF" else params_i).extend(specs)
                if note:
                    notes.append(note)

        entry = {"params_f": params_f, "params_i": params_i,
                 "note": "；".join(n for n in notes if n), "names": names_field}
        for n in _expand_step_names(names_field):
            mapping.setdefault(n, entry)

    # 2) enum: 分组注释 + 成员尾注释(说明)
    enum_m = re.search(r"typedef enum\s*\{(.*?)\}\s*emScriptStepTypeTdf;", h_text, re.S)
    catalog, group = [], "其他"
    for line in enum_m.group(1).splitlines():
        gm = re.search(r"/\*\s*----\s*(.+?)\s*----\s*\*/", line)
        if gm:
            group = gm.group(1)
            continue
        em = re.match(r"\s*(STEP_\w+),?\s*(?:/\*(.*?)\*/)?", line)
        if not em or em.group(1) == "STEP_COUNT":
            continue
        info = mapping.get(em.group(1), {"params_f": [], "params_i": [], "note": "", "names": em.group(1)})
        catalog.append({
            "type": em.group(1),
            "group": group,
            "desc": (em.group(2) or "").strip(),          # 积木说明 = 枚举尾注释
            "hint": info["note"],
            "note": info["note"],
            "names": info["names"],
            "primitive": "",
            "unit": "",
            # 名字列表: 前端渲染参数标签用; 服务端 _serialize_step 靠它算宏与参数个数
            "params_f": [m["name"] for m in info["params_f"]],
            "params_i": [m["name"] for m in info["params_i"]],
            "params_meta": list(info["params_f"]) + list(info["params_i"]),
        })

    # 3) 原语函数 + 参数枚举类型（case 体里的 (emXxxTdf)I(n) 强制转换）
    c_text = SCRIPT_C.read_text(encoding="utf-8")
    prim_by_step, enum_by_step = {}, {}
    for m in re.finditer(r"case\s+(STEP_\w+)\s*:\s*(.*?)(?=case\s+STEP_\w+\s*:|default:)",
                         c_text, re.S):
        body = re.sub(r"/\*.*?\*/", "", m.group(2), flags=re.S)
        fm = re.search(r"([a-zA-Z_]\w*)\s*\(", body)
        prim_by_step[m.group(1)] = fm.group(1) if fm else ""
        casts = {}
        for cm in re.finditer(r"\(\s*([A-Za-z_]\w*)\s*\)\s*[IF]\((\d+)\)", body):
            casts[int(cm.group(2))] = cm.group(1)
        enum_by_step[m.group(1)] = casts

    # 4) 参数元数据: 单位兜底 + 下拉选项派生
    for entry in catalog:
        entry["primitive"] = prim_by_step.get(entry["type"], "")
        blob = " ".join(p.get("raw", "") for p in entry["params_meta"]) + " " + entry["hint"]
        entry["unit"] = next((u for pat, u in UNIT_PATTERNS if pat.search(blob)), "")
        casts = enum_by_step.get(entry["type"], {})
        n_f = len(entry["params_f"])
        for idx, meta in enumerate(entry["params_meta"]):
            if not meta.get("unit"):
                # 单位兜底只看参数自身文本, 避免误继承同步骤其他参数的单位(如 time_ms 的 ms)
                local = "%s %s" % (meta.get("raw", ""), meta.get("tip", ""))
                meta["unit"] = next((u for pat, u in UNIT_PATTERNS if pat.search(local)), "")
            if meta.get("options"):
                continue                                   # DSL 显式指定优先
            # casts 的键是 case 体里 `I(n)` 的 n（**int 槽序号**）, 而 idx 是
            # 「先所有 float、后所有 int」的扁平下标 —— 有 float 槽时必须减掉 float 个数,
            # 否则下拉整体错一位: STEP_BALL_GRAB_ONE_WAIT 的 place 曾因此退化成数字框
            # （grab 借了 place 的转换才看起来正常）。无 float 的积木两者相等, 行为不变。
            etype = casts.get(idx - n_f) if idx >= n_f else None
            if etype and etype in enums:
                # 深拷: STEP_IF_ELSE 的两个目标参数是**同一个枚举类型**, 按引用赋的话
                # 两个下拉会共享同一个 list 对象 —— 前端右键改一个选项会连带改另一个,
                # 而且同一步骤的两个下拉本该各自独立。
                opts = [dict(o) for o in enums[etype]]
                if meta.get("allow_none"):
                    # <可空>: 枚举前追加 0=无（值 0 = 不执行动作）。只影响**本参数**,
                    # 同积木的抓取动作等枚举槽不带这个选项。
                    opts.insert(0, {"value": "0", "label": "无"})
                meta["options"] = opts
                meta["enum_type"] = etype
            else:
                bo = _bool_options_from_tip(meta.get("tip"))
                if bo:
                    meta["options"] = bo                   # 布尔下拉

    # 5) 「机械臂动作」大积木（虚拟条目, 见 _arm_action_entries）
    catalog.extend(_arm_action_entries(catalog))
    # 6) 「底盘动作组」大积木（虚拟条目, 见 _chact_entries; 2026-10 加）
    catalog.extend(_chact_entries(catalog))
    return catalog


def _arm_action_entries(catalog):
    """每个已保存的机械臂动作预设, 在积木库里虚拟出一块「大积木」。

    它们**不是新的步骤类型** —— 共用真的 STEP_ARM_PLAY_WAIT / _NO_WAIT, 只是把
    「动作」参数预填好（synth_params）。这样:
      · 积木库里一眼就能看到自己起的标题, 而不是一个叫「机械臂动作」的通用积木;
      · 拖到画布上落地的仍是一步普通步骤, _serialize_step 照常工作;
      · 画布上那一步的「动作」下拉仍能改到别的动作（有意保留的逃生门）。

    ⚠️ `type` 用合成键（ARM_PLAY_<id>）而不是真步骤名 —— cat_by_type 按 type 索引,
       用真名会把真条目覆盖掉, 于是「原语名」「参数个数」全都取决于遍历顺序。
       前端必须用 `synth_step` 当落地的步骤名（newItemFromCatalog 已如此）。

    文件缺失/解析失败时返回空表 —— 不能让「地图」Tab 整个 500。
    """
    by_type = {c["type"]: c for c in catalog}
    try:
        presets = arm_action.parse_arm()["presets"]
    except Exception:                                        # noqa: BLE001
        return []

    out = []
    for p in presets:
        step = "STEP_ARM_PLAY_NO_WAIT" if p.get("nowait") else "STEP_ARM_PLAY_WAIT"
        base = by_type.get(step)
        if base is None:
            continue
        entry = dict(base)
        entry.update({
            "type": "ARM_PLAY_%d" % p["id"],
            "synth_step": step,
            "synth_params": ["ARM_ACT_%d" % p["id"]],
            "action_id": p["id"],
            "group": "机械臂动作",
            "desc": p["title"] or ("动作%d" % p["id"]),
            "hint": p["note"],
            # 参数元数据要深拷: 前端会就地改（右键编辑），共享同一份会把真条目也改脏
            "params_f": list(base.get("params_f") or []),
            "params_i": list(base.get("params_i") or []),
            "params_meta": [dict(m) for m in (base.get("params_meta") or [])],
        })
        out.append(entry)
    return out


def _chact_entries(catalog):
    """每个已保存的底盘动作组, 在积木库里虚拟出一块「大积木」（2026-10 加）。

    与 _arm_action_entries 同一套路: 不是新的步骤类型 —— 共用真的
    STEP_CHASSIS_ACTION_PLAY_WAIT, 把「动作组」参数预填好（synth_params）;
    type 用合成键（CH_ACT_<id>）避免覆盖真条目; 前端用 synth_step 落地。

    文件缺失/解析失败时返回空表 —— 不能让「地图」Tab 整个 500。
    """
    by_type = {c["type"]: c for c in catalog}
    try:
        groups = chassis_action.parse_chact()["groups"]
    except Exception:                                        # noqa: BLE001
        return []

    out = []
    for g in groups:
        base = by_type.get("STEP_CHASSIS_ACTION_PLAY_WAIT")
        if base is None:
            continue
        entry = dict(base)
        entry.update({
            "type": "CH_ACT_%d" % g["id"],
            "synth_step": "STEP_CHASSIS_ACTION_PLAY_WAIT",
            "synth_params": ["CH_ACT_%d" % g["id"]],
            "action_id": g["id"],
            "group": "底盘动作组",
            "desc": g["title"] or ("动作组%d" % g["id"]),
            "hint": g["note"],
            "params_f": list(base.get("params_f") or []),
            "params_i": list(base.get("params_i") or []),
            "params_meta": [dict(m) for m in (base.get("params_meta") or [])],
        })
        out.append(entry)
    return out


# =====================================================================
# map.c 解析 / 序列化
# =====================================================================


def parse_map():
    """解析 map.c → {version, active, presets:[…], next_id, reentrant:[id…]}。

    `id` = 这张表在预设注册表里的编号（「IF / 跳转」的目标下拉用它）;
    `None` = 没进注册表（手写的非 g_astScriptPresetN 标识, 不能当跳转目标）。
    `next_id` = 新建预设该用的下一个编号（**只增不重用**, 见 _preset_entries）。
    `reentrant`（2026-10-18 加）= 带「可重入」标记的预设 id 集合（存于 map.h 枚举
    注释, 预设栏的「可重入的任务」勾选框读写它）。条目保留原始物理行; 未修改条目
    按原始行回写 → 无操作保存字节级无 diff。"""
    lines = MAP_C.read_text(encoding="utf-8").split("\n")
    blocks = _locate_presets(lines)
    if not blocks:
        raise ValueError("map.c 中找不到任何 stScriptStepTdf 数组")

    reg = parse_preset_registry(lines)
    presets = [{"ident": b["ident"], "label": b["label"],
                "id": (reg.get(b["ident"]) or {}).get("id"),
                "items": _parse_body(lines[b["open"] + 1:b["close"]])}
               for b in blocks]

    reentrant = sorted(parse_preset_reentrant(_map_h_lines()))

    active = blocks[0]["ident"]                     # 兜底 = 第一张表
    for ln in lines:
        am = RE_ACTIVE_DEF.match(ln)
        if am:
            active = am.group(2)
            break

    # next_id 必须盖过**这张文件里所有已用编号** —— 光看注册表 + map.h 的 NEXT_ID 不够:
    # 注册表缺失时（老 map.c）ids 是空的, 而 map.h 的 NEXT_ID 也可能被删掉, 那时只按
    # 那两处算会回退到一个已经用掉的号, 前端就拿它建新预设 → 撞名。
    ids = [p["id"] for p in presets if p["id"] is not None]
    idents_num = []
    for p in presets:
        m = RE_PRESET_IDENT.match(p["ident"])
        if m is not None:
            idents_num.append(int(m.group(1)))
    next_id = max([_recorded_preset_next_id()] + [i + 1 for i in ids]
                  + [n + 1 for n in idents_num] + [1])

    return {"version": MAP_C.stat().st_mtime, "active": active,
            "presets": presets, "next_id": next_id, "reentrant": reentrant}


def _map_h_lines():
    """map.h 的行（读不到/异常 → 空列表: parse_preset_reentrant 对空输入回空集）。"""
    try:
        return MAP_H.read_text(encoding="utf-8").split("\n")
    except OSError:
        return []


def _map_include_warning(incoming, new_text):
    """map.c 用了 STEP_ARM_PLAY_* 却没 #include "app_arm_action.h" → 编译期报
    `identifier "ARM_ACT_n" is undefined`。

    那一步的参数类型 emArmActionIdTdf（ARM_ACT_n）定义在 app_arm_action.h 里, 而
    app_script.h 只声明了步骤类型本身 —— 所以用到大积木就必须带上这个头。
    **只提醒、不拦**（缺 include 是编译期错误, 早一步说比让人对着报错猜半天强;
    而且控制台从不动 include 区, 拦下来也没法自动修）。
    """
    uses = any(it.get("step") in ARM_PLAY_STEPS
               for p in incoming for it in (p.get("items") or []))
    if uses and "app_arm_action.h" not in new_text:
        return ('map.c 里有「机械臂动作」步骤, 但文件没有 #include "app_arm_action.h" '
                '—— 编译会报 ARM_ACT_n is undefined')
    uses_ch = any(it.get("step") in CHACT_PLAY_STEPS
                  for p in incoming for it in (p.get("items") or []))
    if uses_ch and "app_chassis_action.h" not in new_text:
        return ('map.c 里有「底盘动作组」步骤, 但文件没有 #include "app_chassis_action.h" '
                '—— 编译会报 CH_ACT_n is undefined')
    return ""


def _preset_title(label, ident):
    """预设标题 → 能安全放进 map.c 注册表字符串 / map.h 枚举尾注释的文本。

    ⚠️ **只用于这两处生成内容**。map.c 的 `/* ===== 预设: X ===== */` 头注释必须原样
       回吐用户输入 —— 拿净化过的文本写回去会破坏「无操作保存零 diff」。
    ⚠️ 借用 arm_action.sanitize_comment（洗 `"` `\\` `//` `*/` 换行与不配对的括号）:
       标题以前只落进 map.c 的块注释, 现在还要进**生成的头文件**, 是新的注入面。
    """
    t = (label or "").strip()
    return arm_action.sanitize_comment(t) if t else ident


def _preset_header_label(label):
    """预设名 → 能安全放进 map.c 头注释 `/* ===== 预设: X ===== */` 的文本。

    规则本体在 map_codec.sanitize_block_comment（**唯一来源**, 那边有完整理由）。

    ⚠️ **它不截断、也不洗字符串字面量字符**, 正是这里要的: 这是用户起的名字, 不是标签 ——
       拿 `_preset_title`（走 sanitize_comment, 按 MAX_TITLE_LEN 截断）来写这里会把长预设名
       悄悄砍掉。与「无操作保存零 diff」不冲突: 只在该预设**改过名**（或**新建**）时才写这一行,
       洗过之后写回的是稳定值, 再存一次就是同一个字节。
    """
    return map_codec.sanitize_block_comment(label, "预设")


def _recorded_preset_next_id():
    """map.h 里记的 SCRIPT_PRESET_NEXT_ID; map.h 缺失/被改坏就返回 0。"""
    try:
        return parse_next_preset_id(MAP_H.read_text(encoding="utf-8").split("\n"))
    except OSError:
        return 0


def _preset_entries(lines, incoming, reentrant_ids=None):
    """给每个预设定 id。返回 (entries, next_id, error_message)。

    entries 按**文件顺序**, 每项 {ident, title, id, unsafe, reentrant}。
    ``reentrant_ids``（2026-10-18 加, 可选）= 带「可重入」标记的预设 id 集合
    （payload 的 reentrant_ids 优先; 没带就沿用 map.h 里已落盘的标记 —— 老客户端
    不发这个字段也不会把别人勾的标记抹掉）。

    ⚠️ **每一张表都必须拿到 id**: 注册表里少一行, 固件按 id 查表就查不到它; 一张都给不出
       还会写出空初始化列表 —— C 语法错误（AC5 #1514）。表名是不是 g_astScriptPresetN
       形式与「能不能进注册表」**无关**, 注册表才是 id 的真值来源; 标识里那个数字只是
       「这个编号能不能复用」的依据。

    两条水位线要分清:
      · watermark = max(记录在 map.h 的 NEXT_ID, 注册表里最大 id + 1) —— 判「编号回收」用;
      · alloc     = max(watermark, 本次 payload 里所有合规标识的最大编号 + 1)
                    —— 给**手写表名**分配新号用。不这样先抬一下, 手写表会分到一个已经
                    属于某张合规表的号（枚举里出现重复成员, 编译报错）。

    ⚠️ id **只增不重用**（同 ARM_ACTION_NEXT_ID 那个坑）: 新预设必须拿到比所有历史 id 都
       大的号 —— 否则 map.c 里残留的 SCRIPT_PRESET_n 会**无声地指向另一条路线**。
    ⚠️ **这道服务端校验挡不住最主要的那种情形**: 「删掉预设 8 → 再新建一个」时, 文件里的
       注册表**仍然列着 8**（删除要到下面 phase A 才落地）, 于是新来的 8 被判成「老预设保留」,
       与「编辑了预设 8 的步骤」完全无法区分 —— 服务端拿不到任何额外信息。
       真正的防线在**前端**: nextPresetIdent 从 S.nextId（= NEXT_ID）分配, 于是新预设拿到 9,
       map.c 里残留的 SCRIPT_PRESET_8 变成 undefined → **编译期硬失败**。
       这里能挡的是「编号空档」: 中间某号被删过、文件注册表里已经没它了, 客户端却还拿它建新预设。
    ⚠️ 回收校验只在**已有注册表**时做: 注册表还没有（老 map.c 首次保存）时每个标识都是第一次
       登记, 照单全收 —— 否则会把老文件整个卡死存不进去。判据用 watermark 而不是 alloc:
       alloc 含本次 payload 的新号, 拿它比会把合规的新标识自己拦下来。
    """
    reg = parse_preset_registry(lines)
    known = {ident: e["id"] for ident, e in reg.items()}

    used = set(known.values())
    for p in incoming:
        m = RE_PRESET_IDENT.match(p["ident"])
        if m is not None:
            used.add(int(m.group(1)))

    watermark = max([_recorded_preset_next_id()] + [i + 1 for i in known.values()] + [1])
    alloc = max([watermark] + [i + 1 for i in used] + [1])

    entries = []
    for p in incoming:
        ident = p["ident"]
        title = _preset_title(p.get("label"), ident)
        unsafe = bool(preset_call_unsafe(p.get("items")))

        if ident in known:
            pid = known[ident]                      # 老预设: 保留原号
        else:
            m = RE_PRESET_IDENT.match(ident)
            if m is None:
                pid = alloc                         # 手写表名: 用下一个可用号
            else:
                pid = int(m.group(1))
                if known and pid < watermark:
                    return None, None, (
                        "预设标识 %s 的编号 %d 已被回收（当前下一个可用是 %d）—— 直接用它会让"
                        "路线里残留的 SCRIPT_PRESET_%d 无声指向另一条路线。"
                        "请「从磁盘重载」后再新建预设。" % (ident, pid, watermark, pid))

        entries.append({"ident": ident, "title": title, "id": pid, "unsafe": unsafe,
                        "reentrant": bool(reentrant_ids and (pid in reentrant_ids))})
        alloc = max(alloc, pid + 1)

    return entries, alloc, ""


def _replace_preset_registry(lines, entries):
    """把 map.c 的注册表区块换成 entries; 区块不存在就补一个。

    老副本（还没有标记）走补建分支: 插在 `#define SCRIPT_PRESET_ACTIVE` 之前。
    不补的话, 加完跳转积木固件会因为 g_astScriptPreset 未定义而编译不过, 且毫无头绪。
    """
    body = serialize_preset_registry(entries)
    span = find_marked(lines, RE_PRESET_REG_BEGIN, RE_PRESET_REG_END)
    if span is not None:
        return lines[:span[0]] + body + lines[span[1]:]

    at = len(lines)
    for i, ln in enumerate(lines):
        if RE_ACTIVE_DEF.match(ln):
            at = i
            break
    return (lines[:at]
            + ["/* ==== 预设注册表: 开始 ==== */"] + body
            + ["/* ==== 预设注册表: 结束 ==== */"]
            + lines[at:])


def _registry_marker_error(lines):
    """注册表标记必须**要么两个都没有**（老副本, 首次保存时补上）, **要么各一个**。

    只坏一半时再插一个区块, map.c 里就会出现两个 `g_astScriptPreset[]` /
    两个 `g_usScriptPresetNum` → redefinition, 而且保存还报 200 —— 当场拦下来说清楚。
    """
    n_begin = sum(1 for ln in lines if RE_PRESET_REG_BEGIN.match(ln))
    n_end = sum(1 for ln in lines if RE_PRESET_REG_END.match(ln))
    if (n_begin, n_end) in ((0, 0), (1, 1)):
        return ""
    return ("map.c 的预设注册表标记不配对（开始 %d 个 / 结束 %d 个）—— 请手工把它们各补成"
            "一行再保存（不补的话控制台会再插一个区块, 变成重复定义）" % (n_begin, n_end))


def _write_preset_header(entries, next_id):
    """回写 map.h 的生成块。返回 "" = 成功或无需改动, 否则是**没能维护**的原因。

    **内容没变就一个字节都不写**（连 .bak 都不建）—— 与 arm_action.write_files 同款:
    map.c 的 mtime 是版本戳, 但白写 map.h 会让另一个 Tab 的下次保存吃 409。
    ⚠️ 失败必须**报出来**, 不能静默返回: map.h 是 id 单调性唯一的持久化处
       （SCRIPT_PRESET_NEXT_ID）, 也是「目标」下拉的选项来源。维护不了却不说, 用户会看到
       「注册表里有 SCRIPT_PRESET_9、枚举里没有」这种编译错误, 而 id 会开始被回收。
    """
    try:
        text = MAP_H.read_text(encoding="utf-8")
    except OSError:
        return "Application/map.h 读不到"

    lines = text.split("\n")
    span = find_marked(lines, RE_PRESET_GEN_BEGIN, RE_PRESET_GEN_END)
    if span is None:
        # ⚠️ 不自己造标记: 生成块是入库的, 缺了说明 map.h 被改坏了
        return "map.h 的生成块标记缺失/被改坏"

    new_text = "\n".join(lines[:span[0]]
                         + serialize_preset_header(entries, next_id)
                         + lines[span[1]:])
    if new_text == text:
        return ""

    shutil.copy2(MAP_H, MAP_H.with_suffix(".h.bak"))
    map_codec.write_text_like(MAP_H, new_text)      # 按原换行风格写（见该函数注释）
    return ""


def _preset_warnings(incoming, entries):
    """「只提醒不拦」的那些 —— 都会在编译期爆掉, 但对着报错看不出原因。"""
    out = []

    have = {e["id"] for e in entries}
    bad = set()
    for p in incoming:
        bad |= (preset_refs(p.get("items")) - have)
    if bad:
        out.append("路线引用了不存在的预设 %s —— map.h 里不会有对应的 SCRIPT_PRESET_n, "
                   "编译会报 undefined identifier"
                   % ", ".join("SCRIPT_PRESET_%d" % i for i in sorted(bad)))
    return out


def write_map(payload, cat_by_type):
    """POST /api/map: 校验版本 → 备份 → 回写全部预设。返回 (ok, resp, http_status)。
    只改动必要区域: 未修改的数组体按原始行回写 → 无操作保存字节级无 diff。

    整段在 WRITE_LOCK 里跑（「校验版本 → 写盘」之间的 TOCTOU 窗口见该常量处注释）。"""
    with WRITE_LOCK:
        return _write_map_locked(payload, cat_by_type)


def _write_map_locked(payload, cat_by_type):
    base_version = payload.get("base_version")
    # base_version 只该是数字: 给个字符串进来 abs() 会 TypeError → 落到外层变 500
    # （tuning/arm/oled 的 _version_ok 都有 try, 这里补齐同样的门, 2026-10-09）。
    if isinstance(base_version, (int, float)):
        cur = MAP_C.stat().st_mtime
        if abs(base_version - cur) > 1e-6:
            return False, {"error": "conflict",
                           "message": "map.c 已在外部被修改, 请先「从磁盘重载」再保存"}, 409
    elif base_version is not None:
        return False, {"error": "bad_base_version",
                       "message": "base_version 必须是数字"}, 400

    text = MAP_C.read_text(encoding="utf-8")
    lines = text.split("\n")
    incoming = payload.get("presets") or []
    if not incoming:
        return False, {"error": "至少需要保留一个预设"}, 400

    # 「机械臂动作」大积木的 type 是合成键（ARM_PLAY_<id>）, 只能出现在积木库里;
    # 落到 map.c 的必须是真步骤名（synth_step）。写错了不报错的话, 会生成一行
    # `ARM_PLAY_3,` —— C 编译期报 undefined identifier 还算好的, 要是撞上同名符号
    # 就是静默跑错。这里直接拒掉。
    for p in incoming:
        for it in (p.get("items") or []):
            if str(it.get("step", "")).startswith("ARM_PLAY_"):
                return False, {"error": "bad_step",
                               "message": "虚拟积木 %s 不能直接写进 map.c, 落地应用它的"
                                          " synth_step（前端对照 newItemFromCatalog）"
                                          % it.get("step")}, 400

    # 重复 ident: 正常前端不会产生（它从 S.presets 来）, 但**另一个浏览器标签页**拿着
    # 旧状态提交就可能 —— 而 by_ident 会静默丢掉一个, 保存结果与屏幕上看到的对不上。
    seen_idents = set()
    for p in incoming:
        if p["ident"] in seen_idents:
            return False, {"error": "dup_ident",
                           "message": "预设标识 %s 重复提交（有另一个页面拿着旧状态在保存？）"
                                      "请「从磁盘重载」后重试。" % p["ident"]}, 400
        seen_idents.add(p["ident"])

    # 嵌套容器 IF 的三块标记必须配对。放在这里 = 任何一条不过都在**写盘之前**返回
    #   —— test_server 有测试锁住「400 时两个文件 mtime 都不变」。
    # ⚠️ 判据在 map_codec.marker_balance_error（与 serialize_map 实际写出的行同一套口径）,
    #    别在这儿重写一份: 两处漂移的后果是「校验放行、写出来的 C 却不平衡」。
    # ⚠️ 不配对的表**不会**在 C 编译期报错 —— 固件那套位置配对会扫到表尾把本帧收掉,
    #    表现成「后半张路线静默不执行」。所以这道理必须硬拦。
    for p in incoming:
        bal_err = marker_balance_error(p.get("items") or [], p.get("label") or p["ident"])
        if bal_err:
            return False, {"error": "unbalanced_if", "message": bal_err}, 400

    # 分配 / 校验预设 id（「跳到别的预设」积木的目标编号）。任何一条不过都在**写盘之前**
    # 返回 —— test_server 有测试锁住「400 时两个文件 mtime 都不变」。
    marker_err = _registry_marker_error(lines)
    if marker_err:
        return False, {"error": "bad_markers", "message": marker_err}, 400

    # 「可重入」标记（2026-10-18 加）: payload 带了 reentrant_ids（新前端保存时随
    # 预设栏勾选框一起发）就用它; 没带（老页面/外部客户端）沿用 map.h 已落盘的
    # 标记 —— 不然老客户端的每次保存都会把别人勾的可重入抹掉。
    reentrant_ids = payload.get("reentrant_ids")
    if not isinstance(reentrant_ids, list) or not all(
            isinstance(x, int) for x in reentrant_ids):
        reentrant_ids = parse_preset_reentrant(_map_h_lines())

    entries, next_id, ident_err = _preset_entries(lines, incoming, set(reentrant_ids))
    if ident_err:
        return False, {"error": "recycled_id", "message": ident_err}, 400

    by_ident = {p["ident"]: p for p in incoming}

    # 收集「清洗真的改写了用户文本」的条目 —— 静默改写用户输入比不改更糟
    # （他会在下次重载时看到一个自己没打过的名字）。B/C 两个 phase 都往里追加。
    cleansed = []

    # A) 删除不在入参里的预设（连同其后空行）
    while True:
        extra = [b for b in _locate_presets(lines) if b["ident"] not in by_ident]
        if not extra:
            break
        b = extra[-1]
        start = b["hdr"] if b["hdr"] is not None else b["open"]
        end = b["close"]
        while end + 1 < len(lines) and not lines[end + 1].strip():
            end += 1
        del lines[start:end + 1]

    # B) 新增预设: 追加到最后一张现有表之后
    existing = {b["ident"] for b in _locate_presets(lines)}
    for p in incoming:
        if p["ident"] in existing:
            continue
        blocks = _locate_presets(lines)
        at = blocks[-1]["close"] + 1 if blocks else len(lines)
        # ⚠️ 标签**必须**过 _preset_header_label: 这条是「新建」路径, 而 phase C（改名）走的是
        #    同一个函数 —— 原先这里传的是原始 label, 于是新建一个名叫 `坏名 */ x` 的预设会
        #    直接写坏 map.c 的头注释（`*/` 提前闭合 → 后面半行变成活的 C 代码）。
        label = _preset_header_label(p["label"])
        if label != (p["label"] or "").strip():
            cleansed.append({"what": "预设名", "was": p["label"], "now": label})
        lines[at:at] = _make_preset_block(label, p["ident"],
                                          serialize_map(p["items"], cat_by_type, cleansed))
        existing.add(p["ident"])

    # C) 改名 + 更新数组体（自下而上, 避免行号位移）
    for b in sorted(_locate_presets(lines), key=lambda x: -x["open"]):
        p = by_ident[b["ident"]]
        if b["hdr"] is not None and p["label"] != b["label"]:
            lines[b["hdr"]] = "/* ===== 预设: %s ===== */" % _preset_header_label(p["label"])
        lines[b["open"] + 1:b["close"]] = serialize_map(p["items"], cat_by_type, cleansed)

    # D) 切换当前运行的预设（只改 #define 那一行）
    idents = [p["ident"] for p in incoming]
    active = payload.get("active")
    if active not in idents:
        active = idents[0]
    for i, ln in enumerate(lines):
        am = RE_ACTIVE_DEF.match(ln)
        if am:
            lines[i] = am.group(1) + active + am.group(3)
            break

    # E) 预设注册表（跳转目标表）—— 与 map.h 的枚举由**同一份 entries** 产出,
    #    分成两次算就会互相对不上（注册表说 id 3 是 A, 枚举说 3 是 B）。
    lines = _replace_preset_registry(lines, entries)

    new_text = "\n".join(lines)
    if text.endswith("\n") and not new_text.endswith("\n"):
        new_text += "\n"

    # ⚠️ **内容没变就一个字节都不写**（连 .bak 都不建）—— 与 _write_preset_header、
    #    arm_action._write_files_locked 同款。map.c 的 mtime 是版本戳: 无条件重写会让
    #    「什么都没改的一次保存」也 bump 它, 于是另一个干净客户端的 /api/rev 判成
    #    「远端改动」而白自动重载一轮（还附一句「已被其他客户端修改」的提示）。
    #    跳过写之后 resp 里的 version 仍读 MAP_C.stat() —— 那时它就等于 base_version, 语义正确。
    if new_text != text:
        shutil.copy2(MAP_C, MAP_C.with_suffix(".c.bak"))
        map_codec.write_text_like(MAP_C, new_text)  # 按原换行风格写（见该函数注释）
    h_err = _write_preset_header(entries, next_id)

    resp = {"ok": True, "version": MAP_C.stat().st_mtime, "active": active,
            "next_id": next_id}
    warns = []
    if h_err:
        warns.append("map.h 没能更新（%s）—— 新预设不会出现在「目标」下拉里, 而且 "
                     "SCRIPT_PRESET_NEXT_ID 没落盘, 编号可能被回收。"
                     "请检查 Application/map.h 的生成块标记" % h_err)
    inc = _map_include_warning(incoming, new_text)
    if inc:
        warns.append(inc)
    warns.extend(_preset_warnings(incoming, entries))
    if cleansed:
        # 服务端改写了用户打的字 —— 必须说出来。不说的话他会在下次重载时看到
        # 一个自己没打过的名字, 只会觉得「控制台把我的东西弄丢了」。
        warns.append("下列文字含会破坏 map.c 的字符, 已自动替换: "
                     + "、".join("%s「%s」→「%s」" % (c["what"], c["was"], c["now"])
                                for c in cleansed[:3])
                     + ("…（共 %d 处）" % len(cleansed) if len(cleansed) > 3 else ""))
    if warns:
        resp["warning"] = "；".join(warns)
    return True, resp, 200


# =====================================================================
# 元数据写回 app_script.h（右键编辑积木说明 / 参数中文名）
# =====================================================================

ARM_PLAY_STEPS = ("STEP_ARM_PLAY_WAIT", "STEP_ARM_PLAY_NO_WAIT")


def arm_refs(pid):
    """map.c 里有几步引用了这个动作 ID → (n, [预设名…])。

    删除一个动作预设会让这些行变成编译期的 `undefined identifier` —— 那是**故意的
    硬失败**（胜过静默指向另一个动作）, 但用户得先知道他要弄坏几条路线, 所以界面
    在删除前调这个。
    """
    token = "ARM_ACT_%d" % int(pid or 0)
    try:
        mp = parse_map()
    except (OSError, ValueError):
        return 0, []
    n, where = 0, []
    for preset in mp.get("presets", []):
        hit = sum(1 for it in preset.get("items", [])
                  if it.get("step") in ARM_PLAY_STEPS
                  and token in (it.get("params") or []))
        if hit:
            n += hit
            where.append(preset.get("label") or preset.get("ident"))
    return n, where


CHACT_PLAY_STEPS = ("STEP_CHASSIS_ACTION_PLAY_WAIT",)


def chact_refs(gid):
    """map.c 里有几步引用了这个底盘动作组 ID → (n, [预设名…])。同 arm_refs 的理由。"""
    token = "CH_ACT_%d" % int(gid or 0)
    try:
        mp = parse_map()
    except (OSError, ValueError):
        return 0, []
    n, where = 0, []
    for preset in mp.get("presets", []):
        hit = sum(1 for it in preset.get("items", [])
                  if it.get("step") in CHACT_PLAY_STEPS
                  and token in (it.get("params") or []))
        if hit:
            n += hit
            where.append(preset.get("label") or preset.get("ident"))
    return n, where


def write_meta(payload):
    """按 step 重写 enum 尾注释与参数映射注释行。返回 (ok, resp, status)。

    ⚠️ 整段在 WRITE_LOCK 里跑: 它是 app_script.h 的**唯一**写者, 但读-改-写不是原子的,
       两个客户端同时点保存会交错（WRITE_LOCK 处有完整说明）。
    ⚠️ 2026-10-09 起支持可选 base_version（= /api/rev 的 script_h 戳）: 两客户端基于
       各自旧状态编辑元数据时, 后写不再静默覆盖先写 —— 与 map/tuning/arm 端点同款
       乐观锁; 不带该字段的调用（curl/旧前端）行为不变。"""
    with WRITE_LOCK:
        base_version = payload.get("base_version")
        # base_version 只该是数字（write_map 同款门）: 字符串落进 isinstance 的
        # else 分支 = 不比对直接写, 乐观锁被无声绕过
        if base_version is not None and not isinstance(base_version, (int, float)):
            return False, {"error": "bad_base_version",
                           "message": "base_version 必须是数字"}, 400
        if isinstance(base_version, (int, float)) \
                and abs(base_version - SCRIPT_H.stat().st_mtime) > 1e-6:
            return False, {"error": "conflict",
                           "message": "app_script.h 已被其他客户端修改, "
                                      "请关掉编辑器重新打开（你的改动不会被覆盖）"}, 409
        return _write_meta_locked(payload)


def _write_meta_locked(payload):
    step = (payload.get("step") or "").strip()
    if not step:
        return False, {"error": "step required"}, 400

    text = SCRIPT_H.read_text(encoding="utf-8")
    lines = text.split("\n")
    hit_enum = hit_param = False

    # 1) enum 成员尾注释 = 积木说明
    if "desc" in payload:
        desc = map_codec.sanitize_block_comment(payload.get("desc") or "")
        for i, ln in enumerate(lines):
            m = re.match(r"^(\s*)" + re.escape(step) + r"(\s*,)(?:\s*/\*.*?\*/)?\s*$", ln)
            if m:
                lines[i] = "%s%s%s" % (m.group(1), step + m.group(2),
                                       ("   /* %s */" % desc) if desc else "")
                hit_enum = True
                break

    # 2) 参数映射注释行 = 参数元数据
    if "params" in payload:
        metas = payload.get("params") or []
        note = map_codec.sanitize_block_comment(payload.get("note") or "")
        n_f = int(payload.get("n_f", 0))
        for i, ln in enumerate(lines):
            m = re.match(r"^(\s*\*\s+)(" + re.escape(step) +
                         r"(?:\s*/\s*_\w+)?)(\s+)(.*)$", ln)
            if not m:
                continue
            names_field = m.group(2).strip()
            spec_f = [_spec_str(x) for x in metas[:n_f]]
            spec_i = [_spec_str(x) for x in metas[n_f:]]
            if not spec_f and not spec_i:
                body_txt = "(无参" + (", " + note if note else "") + ")"
            else:
                segs = []
                if spec_f:
                    segs.append("afF: " + ", ".join(spec_f))
                if spec_i:
                    segs.append("anI: " + ", ".join(spec_i))
                body_txt = "; ".join(segs)
                if note:
                    body_txt += "   // " + note
            lines[i] = " *   %s %s" % (names_field.ljust(36), body_txt)
            hit_param = True
            break

    if not (hit_enum or hit_param):
        return False, {"error": "在 app_script.h 未找到 %s 的注释位置" % step}, 404

    shutil.copy2(SCRIPT_H, SCRIPT_H.with_suffix(".h.bak"))
    map_codec.write_text_like(SCRIPT_H, "\n".join(lines))   # 按原换行风格写
    return True, {"ok": True, "enum": hit_enum, "params": hit_param}, 200


# =====================================================================
# 蓝牙 Tab（Tab「蓝牙」的后端：串口列表 + Beam 参数 + 机械臂编辑）
# =====================================================================

def bt_state():
    """GET /api/bt 的响应体：串口列表 + 连接状态 + 日志尾巴。"""
    link = bt_link.LINK
    state = link.state()
    prefs = bt_link.load_prefs()

    try:
        ports = bt_serial.refresh_ports()
    except Exception:                                        # noqa: BLE001
        ports = []

    last = str(prefs.get("port") or "")
    # 上次用过、且这次还在列表里 → 自动选中（下次进来不用再挑一遍）
    selected = last if any(p["port"] == last for p in ports) else ""

    state.update({
        "available": bt_serial.AVAILABLE,
        "ports": ports,
        "selected": selected,
        "last_port": last,
        "baud": int(prefs.get("baud") or bt_link.DEFAULT_BAUD),
        "baud_choices": list(bt_link.BAUD_CHOICES),
    })
    return state


def run_bt_action(action, payload):
    """执行一个蓝牙动作 → (ok, log[], data, error)。

    契约照 host_tools.run_action：内部把异常全消化成 (False, ..., 错误文案)，不外抛。
    log 是 [{'text','kind'}]，kind ∈ ok|bad，前端据此上色。
    """
    link = bt_link.LINK
    log = []

    def note(text, kind="ok"):
        log.append({"text": text, "kind": kind})

    try:
        if action == "refresh":
            ports = bt_serial.refresh_ports(force=True)
            note("刷新到 %d 个串口" % len(ports))
            return True, log, {"ports": ports}, ""

        if action == "connect":
            port = (payload.get("port") or "").strip()
            if not port:
                return False, log, None, "请先选择一个串口"
            baud = int(payload.get("baud") or bt_link.DEFAULT_BAUD)
            note("连接 %s @ %d ..." % (port, baud))
            link.connect(port, baud)
            note("已连接，设备应答 PONG")
            return True, log, {"port": port, "baud": baud}, ""

        if action == "disconnect":
            link.disconnect()
            note("已断开")
            return True, log, None, ""

        if action == "list":
            data = link.list_params()
            note("读到 %d 个参数、%d 个关节（全部来自设备自述）"
                 % (len(data["params"]), len(data["joints"])))
            return True, log, data, ""

        if action == "set":
            name = (payload.get("name") or "").strip()
            if not name:
                return False, log, None, "缺少参数名"
            value = payload.get("value")
            lines = link.set_param(name, value)
            note("已写入 %s = %s" % (name, value))
            return True, log, {"lines": lines}, ""

        if action == "get":
            # 读单个参数（轻量; 「底盘动作组」预览后的 ch_busy 轮询用它, 别去拉全表）
            name = (payload.get("name") or "").strip()
            if not name:
                return False, log, None, "缺少参数名"
            value = link.get_param(name)
            return True, log, {"name": name, "value": value}, ""

        if action == "mode":
            on = bool(payload.get("on"))
            lines = link.set_mode(on)
            note("已进入机械臂编辑模式（设备已让位）" if on else "已退出机械臂编辑模式")
            return True, log, {"lines": lines}, ""

        if action == "hs":
            # 2026-10 加: 投一次「握手让出」（$B,HS），给「机械臂」Tab 连接后的自动让位用。
            # 只投递不等待 —— 真正生效要等路线当前这一步跑完，所以这里只报"已投递"。
            lines = link.handshake_yield()
            note("已请求握手让出（等脚本走到让位窗后即可进编辑模式）")
            return True, log, {"lines": lines}, ""

        if action == "jog":
            # ⚠️ 不能用 `payload.get("ms") or 250`: ms=0 是**跟手模式**的有意义取值,
            #    会被 or 静默换成 250（那样拖滑杆永远走完整 S 曲线, 就是不跟手）。
            ms = payload.get("ms")
            lines = link.jog(payload.get("index"), payload.get("angle"),
                             250 if ms is None else ms)
            return True, log, {"lines": lines}, ""

        if action == "grip":
            lines = link.grip(payload.get("kai_he"), payload.get("degree"))
            return True, log, {"lines": lines}, ""

        if action == "pos":
            return True, log, link.get_pos(), ""

        if action == "arm":
            # 播放一组位姿（5 轴同时启 S 曲线 + 夹爪），立即返回 —— 到位靠 pos 轮询
            lines = link.arm_pose(payload.get("angles"), payload.get("kai_he"),
                                  payload.get("degree"))
            return True, log, {"lines": lines}, ""

        if action == "play":
            # 让固件播已编译的预设整段（阻塞几秒）；与路线里的 STEP_ARM_PLAY_WAIT
            # 是同一个固件函数, 所以这条是「台架试跑 == 比赛跑路线」的验证路径
            idx = payload.get("id")
            if idx is None:
                return False, log, None, "缺少动作 ID"
            lines = link.play_action(idx)
            return True, log, {"lines": lines}, ""

        if action == "cplay":
            # 让固件播已编译的底盘动作组（**非阻塞**）; 播完没有由前端轮询 ch_busy
            idx = payload.get("id")
            if idx is None:
                return False, log, None, "缺少动作组 ID"
            lines = link.play_chassis_action(idx)
            note("已投递预览（车会动! 实速, 不限速）—— 播放中可随时停止")
            return True, log, {"lines": lines}, ""

        if action == "cpush":
            # 在线推送: 把控制台**当前这份**动作组推进固件 RAM 草稿区（预览播这份,
            # 不用重编译; 路线/比赛仍跑编译版 —— 改完上比赛记得保存+重编译+烧录）
            gid = payload.get("id")
            items = payload.get("items")
            if gid is None:
                return False, log, None, "缺少动作组 ID"
            if not isinstance(items, list) or not items:
                return False, log, None, "这一组没有条目"
            raw_n = sum(1 for it in items if it.get("kind") == "raw")
            off_n = sum(1 for it in items
                        if it.get("kind") != "raw" and it.get("disabled"))
            n = len(items) - raw_n - off_n
            if n == 0:
                return False, log, None, ("没有可推送的条目（手写行/被「启」禁用的行 "
                                          "都不进预览草稿; 它们只在编译版里按原样生效）")
            lines = link.push_chassis_action(gid, items)
            skip = ""
            if raw_n:
                skip += ", 跳过 %d 条手写行" % raw_n
            if off_n:
                skip += ", 跳过 %d 条已禁用" % off_n
            note("草稿已推送（%d 条%s）—— 预览播这份; 路线/比赛仍跑编译版, "
                 "改完记得保存 + Keil 重编译 + 烧录" % (n, skip))
            return True, log, {"lines": lines, "skipped_raw": raw_n,
                               "skipped_disabled": off_n, "pushed": n}, ""

        if action == "cstop":
            # 停底盘动作组预览（置停止位 + 掐臂序列, ≤20ms 生效）
            lines = link.stop_chassis_action()
            note("已请求停止")
            return True, log, {"lines": lines}, ""

        if action == "chr0":
            # 航向关系重置: 世界航向累加器 ← 当前 IMU 工作航向（摆好车再按）
            lines = link.reset_chassis_heading()
            new_ref = ""
            for ln in lines:
                if ln.startswith("$B,CHR0,"):
                    new_ref = ln.split(",", 1)[1]
                    break
            note("航向已归零: 当前车头方向 = 新的 0° 基准"
                 + ("（设备报 " + new_ref + "°）" if new_ref else ""))
            return True, log, {"lines": lines, "ref": new_ref}, ""

        if action == "snippet":
            code = bt_link.make_arm_snippet(payload.get("items") or [],
                                            wait=bool(payload.get("wait", True)))
            return True, log, {"code": "\n".join(code), "lines": code}, ""

        return False, log, None, "未知动作: %s" % action

    except (bt_link.BtError, bt_serial.SerialError) as exc:
        note("✗ %s" % exc, "bad")
        return False, log, None, str(exc)
    except Exception as exc:                                 # noqa: BLE001
        note("✗ %s" % exc, "bad")
        return False, log, None, str(exc)


# =====================================================================
# 局域网 / 多客户端
# =====================================================================

# 写盘串行化。三个写盘端点都是「读 mtime 比对 base_version → 整文件写回」, 中间有
# TOCTOU 窗口: 两个客户端同一瞬间保存会**都通过**校验, 然后交错写同一个文件 ——
# 轻则丢一次更新, 重则写出两半拼起来的坏源码。单机单用户时几乎不可能, 绑 0.0.0.0
# 多人同时用之后变成可达的。各模块用自己的锁包住「校验 + 落盘」整段
# （tuning.py::WRITE_LOCK / arm_action.py::WRITE_LOCK）—— 三份文件互不相干,
# 不做跨模块的大锁, 免得平白多一层耦合。
WRITE_LOCK = threading.Lock()

_HOST_CACHE = {"names": None, "at": 0.0}
_HOST_CACHE_TTL = 30.0


def local_host_names(force=False):
    """本机自己的主机名/IP 集合（Origin 放行白名单，全部小写）。

    回环 + 机器名 + 各网卡 IP 自动收集；mDNS / 路由器分配的域名那种探测不出来的,
    由使用者写进环境变量 GKS_CONSOLE_HOSTS（逗号分隔）。

    ⚠️ 带 TTL 缓存: 这个判断跑在**每个受保护 POST** 上, 而拖滑杆点动是 5Hz,
       getaddrinfo 进热路径会拖慢拖动。换网后最多错一次 —— 缓存未命中会强制刷一遍再判。
    """
    now = time.monotonic()
    if (not force and _HOST_CACHE["names"] is not None
            and now - _HOST_CACHE["at"] < _HOST_CACHE_TTL):
        return _HOST_CACHE["names"]
    names = {"127.0.0.1", "localhost", "::1"}
    try:
        names.add(socket.gethostname().lower())
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None):
            names.add(str(info[4][0]).lower())
    except OSError:
        pass
    try:
        names.add(host_tools.get_local_ip().lower())
    except OSError:
        pass
    names.update(h.strip().lower()
                 for h in os.environ.get(ORIGIN_HOSTS_ENV, "").split(",") if h.strip())
    _HOST_CACHE["names"] = names
    _HOST_CACHE["at"] = now
    return names


def origin_ok(origin, host_header):
    """这个 Origin 算不算「本控制台自己的页面」（见 Handler._origin_ok 的调用点）。

    绑 0.0.0.0 之后 Origin 不再是恒定的 localhost, 但**不能**放宽成「host 是任意
    IP」或「端口对上就行」—— 那等于把所有网页都放进来。两道都要过:

      ① Origin 的 host 属于本机地址集合（local_host_names）, 且 scheme/port 就是本控制台;
      ② Origin 的 authority 与请求的 `Host` 头**逐字相等**。

    ② 是防 DNS rebinding: 光看①的话, 攻击者把自己域名解析到本机 IP, 浏览器就会
    `fetch('http://evil.com:8410/api/bt')` 带上 `Origin: http://evil.com:8410` ——
    host 不是本机地址, ① 已经拦掉了; ② 再把「Host 与 Origin 不一致」这类代理/
    转发场景一并堵死, 两道合起来才等于「确实是我们自己的页面发的」。
    """
    if not origin:
        return True                       # 非浏览器客户端(curl)不带 Origin → 放行
    try:
        parts = urlsplit(origin)
        port = parts.port                 # 非法端口会在此抛 ValueError
    except ValueError:
        return False
    if parts.scheme != "http" or port != PORT:
        return False
    if parts.netloc.lower() != (host_header or "").lower():
        return False
    name = (parts.hostname or "").lower()
    if name in local_host_names():
        return True
    # 缓存里的网卡信息可能是换网前的 → 刷一遍再判, 免得误拒（真被拒的会再走一次这里）
    return name in local_host_names(force=True)


def revisions():
    """→ 三个 Tab 的版本戳 + 积木库头文件（前端 /api/rev 轮询它判断「队友改了没有」）。

    ⚠️ 与各 Tab 保存时用的版本戳**必须是同一个来源**（都是 st_mtime）, 否则自己保存完
       会被前端误判成「远端改动」而白重载一次。
    ⚠️ script_h 不能省: 「右键积木 → 编辑元数据」走 POST /api/meta 改 app_script.h,
       那条路一个字节都不碰 map.c, 少这个键就永远同步不到别人改的积木库。
    """
    def mt(path):
        try:
            return path.stat().st_mtime
        except OSError:
            return None

    return {"map": mt(MAP_C), "tuning": mt(tuning.APP_CONFIG), "script_h": mt(SCRIPT_H),
            "arm": {"c": mt(arm_action.ACTION_C), "h": mt(arm_action.ACTION_H)},
            "chact": {"c": mt(chassis_action.ACTION_C), "h": mt(chassis_action.ACTION_H)},
            "oled": oled_menu.version(),
            "calib": mt(calib.CALIB_JSON)}


# 「OLED」Tab 要用到的枚举 —— 下拉选项一律**从源码派生**（本模块是唯一能调
# parse_enums 的地方, oled_menu.py 不 import server）。
# emTimerCmdTdf = 「计时」块的参数（开始/结束/重置, 定义在 app_script.h）。
OLED_ENUMS = ("emOledActTdf", "emOledTuneQtyTdf", "emScriptPresetIdTdf",
              "emTimerCmdTdf")

_ENUM_CACHE = []      # [(catalog_key(), enums)] —— 键控缓存, 与 Handler.catalog_cache 同思路


def cached_enums():
    """parse_enums() 的键控缓存（每次 GET /api/oled 都全量扫一遍头文件太浪费）。
    ⚠️ 带路径名 + mtime_ns + size 的键保证「新增/删除头文件」也能让缓存失效。"""
    key = catalog_key()
    if not _ENUM_CACHE or _ENUM_CACHE[0][0] != key:
        _ENUM_CACHE[:] = [(key, parse_enums())]
    return _ENUM_CACHE[0][1]


def oled_state():
    """GET /api/oled → 菜单模型 + 三个下拉的选项 + 「参数」弹窗的清单。

    ``params`` = oled_menu.param_catalog()（Beam 表 ∪ Pi 参数表, 每次现扫）——
    通用调参行（OLED_ACT_TUNE_PARAM）的参数名从这里选, 前端按 ``group`` 归类。
    """
    st = oled_menu.state()
    enums = cached_enums()
    st["options"] = {name: enums.get(name, []) for name in OLED_ENUMS}
    st["arg_enum"] = dict(oled_menu.ACT_ARG_ENUM)
    st["params"] = oled_menu.param_catalog()
    return st


def _static_file(rel):
    """把 `/static/<rel>` 解析成 STATIC_DIR 下的真实文件; 越界或不存在 → None。

    ⚠️ 判据必须用 `relative_to`，**不能**用 `str(fp).startswith(str(base))`:
       后者没有结尾分隔符, `/static/../staticX/f` 这类指向**同级 static 打头的目录**的路径
       能通过检查（经典的 startswith 陷阱 —— 当前目录布局下恰好没有这样的兄弟目录,
       所以实际不可利用, 但别把这条改回去）。
    rel 里带 `..` 或绝对路径时: `Path` 的拼接语义会让绝对路径替换掉 base,
       两种都由 relative_to 抛 ValueError 兜掉。
    """
    base = STATIC_DIR.resolve()
    try:
        fp = (base / rel).resolve()
        fp.relative_to(base)
    except (ValueError, OSError):
        return None
    return fp if fp.is_file() else None


# =====================================================================
# HTTP
# =====================================================================

MIME = {".html": "text/html; charset=utf-8",
        ".js": "application/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8",
        ".svg": "image/svg+xml", ".png": "image/png", ".ico": "image/x-icon"}


class Handler(BaseHTTPRequestHandler):
    # 连接级读超时(秒): 没有它, 客户端声明大 Content-Length 却不把 body 发完时,
    # rfile.read 会永久挂住该请求线程（daemon_threads 只保证进程能退出, 不清运行期
    # 堆积的线程）。正常请求毫秒级完成, 30s 绰绰有余。（2026-10-09 加）
    timeout = 30

    # (catalog_key(), catalog) —— None 或键不匹配都要重解析。写盘端点里的
    # `Handler.catalog_cache = None` 仍然有效（显式失效, 与键不匹配等价）。
    catalog_cache = None

    def log_message(self, fmt, *args):        # 安静模式
        pass

    def _send(self, status, data, ctype="application/json; charset=utf-8"):
        body = data if isinstance(data, bytes) else \
            json.dumps(data, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _catalog(self):
        """积木库（带缓存）。**按 catalog_key() 判是否还要用缓存** —— 见那边的说明:
        缓存不感知外部修改的话, 队友 git pull 之后前端提示「积木库已刷新」而内容纹丝不动。"""
        key = catalog_key()
        cached = Handler.catalog_cache
        if cached is None or cached[0] != key:
            cached = (key, parse_catalog())
            Handler.catalog_cache = cached
        return cached[1]

    def _catalog_dict(self):
        """积木库按 type 索引 —— 序列化 step 时用它算宏与参数个数。"""
        return {c["type"]: c for c in self._catalog()}

    def _origin_ok(self):
        """拒绝来自其它页面的跨站 POST。

        /api/tools/* 会拉起 mstsc/资源管理器/Trae 并改 ~/.ssh/config, 影响比写 map.c
        大得多, 而浏览器里任意页面都能 POST 到本机端口。跨站 fetch 浏览器必带 Origin,
        非浏览器客户端(如 curl)不带 Origin → 放行。

        ⚠️ 本控制台绑 0.0.0.0, 队友用局域网地址打开时 Origin 是那个地址而不是 localhost
        —— 所以判据是 origin_ok() 的「本机地址 + Host 一致」两道, 不是一串写死的字符串。
        别退回去写死 localhost: 那会让队友一保存就 403（页面看着完全正常, 极难查）。"""
        return origin_ok(self.headers.get("Origin"), self.headers.get("Host"))

    def do_GET(self):
        # 路由是精确匹配: 剥掉查询串再比, 否则 /api/capture/frame.jpg?t=… 这类破缓存请求
        # 会 404（对现有无查询串的路径是恒等变换, 不影响任何既有路由）
        self.path = self.path.partition("?")[0]
        try:
            if self.path in ("/", "/index.html"):
                self._send(200, (STATIC_DIR / "index.html").read_bytes(), MIME[".html"])
            elif self.path.startswith("/static/"):
                fp = _static_file(self.path[len("/static/"):])
                if fp is None:
                    return self._send(404, {"error": "not found"})
                self._send(200, fp.read_bytes(),
                           MIME.get(fp.suffix, "application/octet-stream"))
            elif self.path == "/api/catalog":
                self._send(200, self._catalog())
            elif self.path == "/api/rev":
                self._send(200, revisions())
            elif self.path == "/api/map":
                self._send(200, parse_map())
            elif self.path == "/api/tools":
                state = host_tools.host_info()          # IP 每次实时探测: 换网后无需重启
                state["phrases"] = host_tools.load_phrases()
                self._send(200, state)
            elif self.path == "/api/tuning":
                self._send(200, tuning.state())
            elif self.path == "/api/bt":
                self._send(200, bt_state())
            elif self.path == "/api/arm":
                self._send(200, arm_action.state())
            elif self.path == "/api/chact":
                self._send(200, chassis_action.state())
            elif self.path == "/api/calib":
                self._send(200, calib.state())
            elif self.path == "/api/moves":
                self._send(200, moves.state())
            elif self.path == "/api/moves/image":
                status, body, ctype = moves.image_response()
                self._send(status, body, ctype)
            elif self.path == "/api/switches":
                self._send(200, switches.state())
            elif self.path == "/api/oled":
                self._send(200, oled_state())
            elif self.path == "/api/capture":
                self._send(200, capture.capture_state())
            elif self.path == "/api/capture/frame.jpg":
                status, body, ctype = capture.frame_response()
                self._send(status, body, ctype)
            else:
                self._send(404, {"error": "not found"})
        except Exception as e:                                    # noqa: BLE001
            traceback.print_exc()          # 500 不能只回一句 str(e): 排障全靠服务端栈
            self._send(500, {"error": str(e)})

    def do_POST(self):
        self.path = self.path.partition("?")[0]             # 同 do_GET: 精确匹配前先剥查询串
        try:
            if self.path not in ("/api/map", "/api/meta",
                                 "/api/tools/action", "/api/tools/phrases",
                                 "/api/tuning", "/api/tuning/meta", "/api/tuning/prefs",
                                 "/api/bt", "/api/capture", "/api/arm", "/api/chact",
                                 "/api/oled", "/api/calib", "/api/moves", "/api/switches"):
                return self._send(404, {"error": "not found"})
            # /api/bt 会打开串口、驱动真实舵机, /api/capture 会开摄像头并往仓库写图,
            # /api/arm 与 /api/chact 会回写仓库源文件, /api/calib 会写 calib.json
            # 并经 apply 改 app_config.h, /api/moves 会写 moves.json / 删 app_config.h
            # 的变量行, /api/switches 会写 switches.json / 生成块（app_script.h/.c）
            # / 改 app_config.h 的开关宏 —— 与 /api/tools 同级, 一并做 Origin 校验
            if self.path.startswith(("/api/tools", "/api/tuning", "/api/bt",
                                     "/api/capture", "/api/arm", "/api/chact",
                                     "/api/map", "/api/meta",
                                     "/api/oled", "/api/calib", "/api/moves",
                                     "/api/switches")) and not self._origin_ok():
                # 被拒在浏览器里只是一句「请求失败」, 看不出为什么。所以两边都留线索:
                # 服务端 stdout 打出实际的 Origin/Host（服务器是唯一能同时看到两者的地方），
                # 响应里给出自救办法。局域网访问最常见的误拒是用了 mDNS/路由器域名登录。
                origin = self.headers.get("Origin")
                print("[webconsole] 403 %s  Origin=%s  Host=%s —— 若这是队友的正常访问, "
                      "把该主机名加进环境变量 %s（逗号分隔）后重启服务端"
                      % (self.path, origin, self.headers.get("Host"), ORIGIN_HOSTS_ENV))
                return self._send(403, {
                    "error": "forbidden",
                    "message": "来源 %s 不在允许列表。若这是队友的正常访问, 把它的主机名"
                               "写进环境变量 %s（逗号分隔）后重启服务端。" % (origin, ORIGIN_HOSTS_ENV)})

            # body 解析: 三种坏输入各自给出贴切的 4xx, 别让它们落到外层 except 变成 500
            # （缺 body / 非 JSON 时 json.loads 会抛, 从前一律 500, 前端只能报「HTTP 500」）。
            try:
                length = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                return self._send(400, {"error": "bad_content_length",
                                        "message": "Content-Length 不是数字"})
            if length < 0:
                return self._send(400, {"error": "bad_content_length"})
            if length > MAX_BODY_BYTES:
                return self._send(413, {"error": "body_too_large",
                                        "message": "请求体超过 %d 字节" % MAX_BODY_BYTES})
            try:
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
            except (ValueError, UnicodeDecodeError) as e:
                return self._send(400, {"error": "bad_json", "message": str(e)})

            if self.path == "/api/capture":
                ok, log, data, error = capture.run_capture_action(
                    payload.get("action", ""), payload)
                resp = {"ok": ok, "log": log}
                if data is not None:
                    resp["data"] = data
                if error:
                    resp["error"] = error
                return self._send(200, resp)

            if self.path == "/api/bt":
                ok, log, data, error = run_bt_action(payload.get("action", ""), payload)
                resp = {"ok": ok, "log": log}
                if data is not None:
                    resp["data"] = data
                if error:
                    resp["error"] = error
                return self._send(200, resp)

            if self.path == "/api/tuning":
                ok, resp, status = tuning.write_config(
                    payload.get("values"), payload.get("new_params"),
                    payload.get("base_version"))
                return self._send(status, resp)

            if self.path == "/api/tuning/meta":
                name = (payload.get("name") or "").strip()
                if not name:
                    return self._send(400, {"error": "name required"})
                ok, resp, status = tuning.write_meta(name, payload.get("meta") or {},
                                                     payload.get("base_version"))
                return self._send(status, resp)

            if self.path == "/api/tuning/prefs":
                prefs = {"visible": payload.get("visible") or {},
                         "order": payload.get("order") or {}}
                if not isinstance(prefs["visible"], dict) or not isinstance(prefs["order"], dict):
                    return self._send(400, {"error": "visible/order 必须是对象"})
                tuning.save_prefs(prefs)
                return self._send(200, {"ok": True, "prefs": prefs})

            if self.path == "/api/tools/action":
                log = []
                ok, text = host_tools.run_action(payload.get("action", ""), payload, log.append)
                return self._send(200, {"ok": ok, "log": log, "text": text})

            if self.path == "/api/tools/phrases":
                phrases = payload.get("phrases")
                if not isinstance(phrases, list):
                    return self._send(400, {"error": "phrases 必须是列表"})
                host_tools.save_phrases(phrases)
                return self._send(200, {"ok": True, "phrases": phrases})

            if self.path == "/api/meta":
                ok, resp, status = write_meta(payload)
                if ok:
                    Handler.catalog_cache = None      # 失效缓存, 下次 GET 重新派生
                return self._send(status, resp)

            if self.path == "/api/arm":
                action = payload.get("action", "")
                if action == "refs":
                    # 删除动作前问一句「map.c 里有几步引用它」。放在这里是因为
                    # 它要读 map.c —— 那是本模块的地盘, 不是 arm_action.py 的。
                    n, where = arm_refs(payload.get("id"))
                    return self._send(200, {"ok": True,
                                            "data": {"referenced": n, "where": where}})
                ok, data, error, status = arm_action.run_arm_action(
                    action, payload, self._catalog_dict())
                resp = {"ok": ok}
                if data is not None:
                    resp["data"] = data
                if error:
                    resp["error"] = error
                if ok and action in ("save", "delete", "restore"):
                    Handler.catalog_cache = None      # 积木库的虚拟条目跟着变
                # 409/400 透传: 机械臂 Tab 的「↺ 重载」提示条靠 HTTP 409 触发
                return self._send(200 if ok else status, resp)

            if self.path == "/api/chact":
                action = payload.get("action", "")
                if action == "refs":
                    # 删除动作组前问引用数（读 map.c, 同 /api/arm 的 refs 理由）
                    n, where = chact_refs(payload.get("id"))
                    return self._send(200, {"ok": True,
                                            "data": {"referenced": n, "where": where}})
                ok, data, error, status = chassis_action.run_chact_action(action, payload)
                resp = {"ok": ok}
                if data is not None:
                    resp["data"] = data
                if error:
                    resp["error"] = error
                if ok and action in ("save", "delete", "restore"):
                    Handler.catalog_cache = None      # 积木库的虚拟条目跟着变
                # 409/400 透传（同 /api/arm）
                return self._send(200 if ok else status, resp)

            if self.path == "/api/moves":
                action = payload.get("action", "")
                if action == "add_var":
                    ok, resp, status = moves.add_var(
                        payload.get("name"), payload.get("display"),
                        payload.get("value", "0"), payload.get("base_version"))
                    return self._send(status, resp)
                if action == "delete_var":
                    ok, resp, status = moves.delete_var(
                        payload.get("name"), payload.get("base_version"))
                    return self._send(status, resp)
                # 缺省 action = 保存示意图（moves.json; 纯可视化, 不碰 app_config.h）
                ok, resp, status = moves.save_config(
                    payload.get("config"), payload.get("base_version"))
                return self._send(status, resp)

            if self.path == "/api/switches":
                # 「自定义」Tab: 开关 = STEP_IF 的编译期自定义条件。
                # 生成块写 app_script.h/.c —— 与 /api/meta 的 write_meta 同一文件,
                # 整段必须持 WRITE_LOCK（含 add/rename/delete/set 里调 tuning 的写盘,
                # 锁序固定 server→tuning, 与反向嵌套不会同时出现 → 无死锁）。
                action = payload.get("action", "")
                with WRITE_LOCK:
                    if action == "add":
                        ok, resp, status = switches.add(
                            payload.get("name"), payload.get("display"),
                            payload.get("group"), payload.get("note"),
                            payload.get("base_version"))
                    elif action == "rename":
                        ok, resp, status = switches.rename(
                            payload.get("id"), payload.get("display"),
                            payload.get("group"), payload.get("note"),
                            payload.get("base_version"))
                    elif action == "toggle":
                        ok, resp, status = switches.toggle(
                            payload.get("id"), payload.get("value"),
                            payload.get("base_version"))
                    elif action == "delete":
                        ok, resp, status = switches.delete(
                            payload.get("id"), payload.get("base_version"))
                    elif action == "set_switches":
                        ok, resp, status = switches.set_switches(
                            payload.get("order_ids"), payload.get("groups"))
                    else:
                        ok, resp, status = False, {"error": "bad_action",
                                                   "message": "未知 action: %r" % action}, 400
                if ok and action in ("add", "rename", "delete", "set_switches"):
                    Handler.catalog_cache = None      # emScriptCondTdf 变了 → 条件下拉的来源
                    _ENUM_CACHE[:] = []               # cached_enums 的键控缓存也显式失效
                return self._send(status, resp)

            if self.path == "/api/calib":
                # 「标定」Tab: save 只写 calib.json; apply 会改 app_config.h 的宏
                # （那是「调参」Tab 的地盘 —— 版本戳/重载由 /api/rev 的 tuning 键走,
                #  自己保存的自己用响应里的 cfg_version 回填, 不会被误判成远端改动）。
                ok, data, error, status = calib.run_calib_action(
                    payload.get("action", ""), payload)
                resp = {"ok": ok}
                if data is not None:
                    resp["data"] = data
                if error:
                    resp["error"] = error
                # 409/400 透传（同 /api/arm）
                return self._send(200 if ok else status, resp)

            if self.path == "/api/oled":
                # 保存会写 app_oled_menu.c（+ 必要时补字重生成字模）。校验不过回 400、
                # 版本戳对不上回 409；字模生成失败会在 oled_menu 里回滚并抛出 → 外层 500。
                # ⚠️ valid_params 现扫（Beam 表 ∪ Pi 参数表）: 通用调参行的参数名必须在
                # 那两张表里真的存在 —— 名字写错的行在车上只会显示 "?" 并长鸣, 是
                # 「配得出来但跑不起来」的死项, 正是这一层该校验的东西。
                # ⚠️ param_opts（2026-10-18 加）: {名字: 真选项表} —— 显示映射
                # （OLED_ITEM_PM 的 map 段）的值域校验用。
                _pcat = oled_menu.param_catalog()
                ok, resp, status = oled_menu.save(
                    payload, payload.get("base_version"),
                    valid_params=set(_pcat["names"]),
                    param_opts={it["name"]: it.get("options") or ""
                                for it in _pcat["beam"] + _pcat["pi"]})
                if ok:
                    # emOledActTdf / emOledTuneQtyTdf 是「动作」「参数」两个下拉的选项来源
                    Handler.catalog_cache = None
                    _ENUM_CACHE[:] = []           # 上面的键控缓存也要显式失效
                return self._send(status, resp)

            cat_by_type = self._catalog_dict()
            ok, resp, status = write_map(payload, cat_by_type)
            if ok:
                # ⚠️ 必须失效: save_map 会重生成 map.h 的 emScriptPresetIdTdf —— 那是
                # 「IF / 跳转」目标下拉的**选项来源**（parse_enums 扫 map.h）。不失效的话
                # 新建的预设不会出现在目标下拉里, 直到重启服务端 —— 现象是「这功能没做」。
                Handler.catalog_cache = None
            self._send(status, resp)
        except Exception as e:                                    # noqa: BLE001
            traceback.print_exc()          # 同 do_GET: 500 必须在服务端留栈
            self._send(500, {"error": str(e)})


def main():
    ThreadingHTTPServer.allow_reuse_address = False   # Windows 下禁止重复绑定, 双开直接报错
    # 绑 0.0.0.0 = 局域网内队友也能连（Origin 放行规则见 origin_ok）。
    server = ThreadingHTTPServer((BIND_HOST, PORT), Handler)
    # 本机仍然走回环打开浏览器 —— 别让自己绕一圈网卡
    threading.Timer(0.8, lambda: webbrowser.open(LOOPBACK_URL)).start()
    print("[webconsole] 本机     %s" % LOOPBACK_URL)
    lan = host_tools.console_url()
    if lan:
        print("[webconsole] 局域网   %s   ← 发给队友" % lan)
        print("[webconsole]          队友的机器首次连不上时: Windows 防火墙放行入站 TCP %d" % PORT)
    else:
        print("[webconsole] 局域网   探测不到本机 IP（没连网络?）")
    print("[webconsole] map.c      = %s" % MAP_C)
    print("[webconsole] app_script.h = %s" % SCRIPT_H)
    print("[webconsole] 采集落盘    = %s" % capture.RAW_DIR)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
