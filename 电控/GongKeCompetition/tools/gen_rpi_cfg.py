# -*- coding: utf-8 -*-
"""从 Pi 的 config.py 生成单片机侧的 Pi 参数表（写进 Application/app_rpi.c 的生成块）。

单片机要能在 OLED 调参块里选「树莓派端的参数」（巡线PID-Kp 这类），就得本地有一张
元数据表（名字/中文名/分组/类型/选项表）—— 值不落表，运行时用 "P" 命令现取（见
docs/串口协议.md §6k）。表由本脚本生成，**别手改**：改 config.py 后重跑即可。

    python tools/gen_rpi_cfg.py [config.py 路径]

默认读树莓派共享目录下的真身；本地开发可传 .pi_work/GKS_Competition/config.py。
生成块写在 app_rpi.c 的两个标记之间（同 oled_menu.py 对 app_oled_menu.c 的做法），
重复跑是幂等的。
"""
import io
import os
import re
import sys
import importlib.util

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

DEFAULT_CFG = r"\\192.168.1.125\shared\Desktop\GKS_Competition\config.py"  # 示例网段, 按现场 Pi 实际地址改（最后一段固定 .125）
TARGET_C = os.path.join(ROOT, "Application", "app_rpi.c")

MARK_BEGIN = "/* ==== Pi 参数表: 生成块 开始（tools/gen_rpi_cfg.py, 勿手改） ==== */"
MARK_END = "/* ==== Pi 参数表: 生成块 结束 ==== */"

# 前缀 → 分组中文名。与 GKS_Web/app.py 的 CFG_GROUPS 保持一致（那边是控制台渲染,
# 这边是 OLED 里的分组名，两处错开只会"看起来不一致"，不会坏功能）。
GROUPS = [
    ("QR_", "二维码"), ("CMD_", "指令"), ("HOSTAGE_", "人质"), ("COLOR_", "色球"),
    ("RECT_", "矩形"), ("TARGET_", "标靶"), ("ROADYAW_", "看路向"), ("ROAD_", "看路"),
    ("LINE_", "巡线"), ("OPENMV_", "OpenMV"), ("CAM_", "相机"), ("PICAM_", "pi摄像头"),
    ("FRAME_", "画面"), ("LEASE_", "租约"), ("SYSTEM_", "系统"),
]
GROUP_ORDER = [g for _, g in GROUPS]

# config.py 的 type → 单片机的类型常量
TYPE_MAP = {"int": "RPI_CFG_T_I32", "float": "RPI_CFG_T_F32", "str": "RPI_CFG_T_STR"}


def group_of(name):
    for p, g in GROUPS:
        if name.startswith(p):
            return g
    return "其它"


def c_str(s):
    """Python 字符串 → C 字符串字面量（转义反斜杠/引号；非 ASCII 原样保留，工程是 UTF-8）。"""
    return '"%s"' % s.replace("\\", "\\\\").replace('"', '\\"')


def main():
    cfg_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_CFG
    if not os.path.exists(cfg_path):
        print("找不到 config.py: %s" % cfg_path)
        return 2
    spec = importlib.util.spec_from_file_location("gks_config_gen", cfg_path)
    cfg = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cfg)

    items = []
    for entry in cfg.EDITABLE_CONFIG:
        name, typ, desc, opts = entry
        if typ not in TYPE_MAP:
            print("跳过未映射的类型: %s (%s)" % (name, typ))
            continue
        items.append({
            "name": name, "label": desc or name, "group": group_of(name),
            "type": TYPE_MAP[typ], "opts": (opts or "").strip(),
        })

    # 组序固定（OLED 里按这个顺序出现），组内保持 config.py 的顺序
    items.sort(key=lambda x: (GROUP_ORDER.index(x["group"])
                              if x["group"] in GROUP_ORDER else len(GROUP_ORDER),))

    # 选项表串去重成 static const char[]
    opt_vals = []
    for it in items:
        if it["opts"] and it["opts"] not in opt_vals:
            opt_vals.append(it["opts"])
    opt_sym = {}
    lines = []
    for i, v in enumerate(opt_vals):
        sym = "s_acRpiCfgOpt%d" % i
        opt_sym[v] = sym
        lines.append("static const char %s[] = %s;" % (sym, c_str(v)))
    if lines:
        lines.append("")

    lines.append("/* 名字/中文名/分组/类型/选项表。⚠️ 值不在这里 —— 运行时用 \"P?,<名>\" 现取")
    lines.append(" * （见 docs/串口协议.md §6k），表里只放**元数据**（Pi 掉线时也要能画菜单）。 */")
    lines.append("const stRpiCfgParamTdf g_astRpiCfgParam[] =")
    lines.append("{")
    cur_group = None
    for it in items:
        if it["group"] != cur_group:
            cur_group = it["group"]
            lines.append("    /* ---- %s ---- */" % cur_group)
        opts_ref = opt_sym[it["opts"]] if it["opts"] else "NULL"
        lines.append("    { %-24s %-22s %-10s %-14s %s }," % (
            c_str(it["name"]) + ",", c_str(it["label"]) + ",",
            c_str(it["group"]) + ",", it["type"] + ",", opts_ref))
    lines.append("};")
    lines.append("const uint16_t g_usRpiCfgParamNum = (uint16_t)(sizeof(g_astRpiCfgParam)"
                 " / sizeof(g_astRpiCfgParam[0]));")
    block = "\n".join(lines)

    src = io.open(TARGET_C, encoding="utf-8").read()
    if MARK_BEGIN in src:
        pat = re.compile(re.escape(MARK_BEGIN) + ".*?" + re.escape(MARK_END), re.S)
        if not pat.search(src):
            print("生成块标记不配对（只有开始没有结束）")
            return 2
        src = pat.sub(MARK_BEGIN + "\n" + block + "\n" + MARK_END, src)
    else:
        # 首次生成: 追加到文件末尾（app_rpi.c 以 vRpiTask 收尾, 没有 #endif 之类的锚点）
        if not src.endswith("\n"):
            src += "\n"
        src += "\n" + MARK_BEGIN + "\n" + block + "\n" + MARK_END + "\n"

    io.open(TARGET_C, "w", encoding="utf-8", newline="").write(src)
    print("已写 %s：%d 项（%d 种选项表）" % (TARGET_C, len(items), len(opt_vals)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
