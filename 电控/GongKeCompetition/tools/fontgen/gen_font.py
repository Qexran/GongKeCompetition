#!/usr/bin/env python3
"""汉字字模生成器 —— 给 OLED 驱动的 OLED_CF16x16[] 造 16x16 字模。

为什么有这个工具
================
Core/Src/OLED_Data.c 里那张 OLED_CF16x16[] 要手写 32 个十六进制字节/字，纯手工
填一定会错、而且改一个菜单文案就得重填一遍。这个工具把「汉字列 -> 字模 C 片段」
自动化，只重写 OLED_Data.c 里两个标记之间的整段，**标记以外的字节一个不动**。

渲染参数是**反推出来的、不是拍脑袋定的**
=========================================
江协科技原版字库只留了 6 个示例字（你/好/世/界/，/。）。本工具的渲染参数就是照着
让这 6 个字模**逐字节相符**反解出来的：

    字体   C:/Windows/Fonts/simsun.ttc   （宋体；TTF 集合取 index 0）
    字号   16
    落点   (0, 0)
    阈值   128
    版式   2 页 x 16 列，bit0 = 页内顶行

`--check` 会断言这一点 —— 它是本工具**唯一的硬回归锚点**：谁改了渲染参数或打包
顺序，这条立刻红，而不是等到屏幕上一堆乱码才发现。真正上机前请先跑 `--check`。

依赖
====
Pillow（`pip install pillow`）。**只在造字时用**：固件不依赖、tools/webconsole/
也不依赖，所以这不违反那边的 stdlib-only 约定（同 capture.py 对待 cv2 的做法）。
没装 PIL 时本工具给出明确的安装提示而不是崩栈。

用法
====
    python tools/fontgen/gen_font.py --check      # 只校验（推荐先跑这个）
    python tools/fontgen/gen_font.py              # 生成并写回 OLED_Data.c
    python tools/fontgen/gen_font.py --font <ttf> --size 16 --threshold 128
    python tools/fontgen/gen_font.py --stdout     # 只打印 C 片段, 不写盘

字表来自同目录的 glyphs.txt（UTF-8，`#` 起头是注释）。
"""

from __future__ import annotations

import argparse
import os
import re
import sys

# ---------------------------------------------------------------------------
# 常量
# ---------------------------------------------------------------------------

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA_C = os.path.join(REPO_ROOT, "Core", "Src", "OLED_Data.c")
GLYPHS_TXT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glyphs.txt")

BEGIN_MARK = "/* ==== 汉字字模: 开始 ==== */"
END_MARK = "/* ==== 汉字字模: 结束 ==== */"

DEFAULT_FONT = r"C:/Windows/Fonts/simsun.ttc"
DEFAULT_SIZE = 16
DEFAULT_THRESHOLD = 128

# 非「汉字/全角区间」、但按 16px 全角渲染进字表的符号（见 is_han 的说明）。
# °(U+00B0, 2 字节 UTF-8): Beam 角度类参数的单位（蓝牙「巡线」组的
#   line_angle_target / line_*_db_deg 等）—— OLED 通用调参行会把单位拼在值后面,
#   不进字表就是那个「方框问号」（2026-10-18 实测: `角度目标:-1.5[?]`）。
# Δ(U+0394, 2 字节 UTF-8): 看路调试菜单的「理想Δx / 理想Δy」（2026-11-01 加）。
#   ⚠️ 2 字节字符要同时满足两处: ① 本表收录（否则画默认方框）;
#   ② 调用方按 UTF-8 长度切字（app_oled.c::ucOledDrawMixed, 一律取 3 字节会把
#   后面的 x 吞掉）。只收进本表而不修切字, 屏上是「方框」+ 丢 x。
EXTRA_WIDE = "√×°Δ"

# 江协原版字库里保留的示例字模 —— 逐字节的黄金样本（见模块 docstring）。
# 渲染参数或打包顺序一旦改动，这里就会失配。
GOLDEN = {
    "你": [0x00, 0x80, 0x60, 0xF8, 0x07, 0x40, 0x20, 0x18, 0x0F, 0x08, 0xC8, 0x08, 0x08, 0x28, 0x18, 0x00,
           0x01, 0x00, 0x00, 0xFF, 0x00, 0x10, 0x0C, 0x03, 0x40, 0x80, 0x7F, 0x00, 0x01, 0x06, 0x18, 0x00],
    "好": [0x10, 0x10, 0xF0, 0x1F, 0x10, 0xF0, 0x00, 0x80, 0x82, 0x82, 0xE2, 0x92, 0x8A, 0x86, 0x80, 0x00,
           0x40, 0x22, 0x15, 0x08, 0x16, 0x61, 0x00, 0x00, 0x40, 0x80, 0x7F, 0x00, 0x00, 0x00, 0x00, 0x00],
    "世": [0x20, 0x20, 0x20, 0xFE, 0x20, 0x20, 0xFF, 0x20, 0x20, 0x20, 0xFF, 0x20, 0x20, 0x20, 0x20, 0x00,
           0x00, 0x00, 0x00, 0x7F, 0x40, 0x40, 0x47, 0x44, 0x44, 0x44, 0x47, 0x40, 0x40, 0x40, 0x00, 0x00],
    "界": [0x00, 0x00, 0x00, 0xFE, 0x92, 0x92, 0x92, 0xFE, 0x92, 0x92, 0x92, 0xFE, 0x00, 0x00, 0x00, 0x00,
           0x08, 0x08, 0x04, 0x84, 0x62, 0x1E, 0x01, 0x00, 0x01, 0xFE, 0x02, 0x04, 0x04, 0x08, 0x08, 0x00],
    "，": [0x00] * 16 + [0x00, 0x00, 0x58, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
    "。": [0x00] * 16 + [0x00, 0x00, 0x18, 0x24, 0x24, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
}

# 字库表尾那个「方框问号」默认图形 —— 驱动靠「Index 为空串」判「找完了」，
# 所以它**必须**在数组最末尾，不能挪。数据与江协原版一致。
DEFAULT_BOX_INDEX = ""
DEFAULT_BOX_DATA = [
    0xFF, 0x01, 0x01, 0x01, 0x31, 0x09, 0x09, 0x09, 0x09, 0x89, 0x71, 0x01, 0x01, 0x01, 0x01, 0xFF,
    0xFF, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x96, 0x81, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0xFF,
]


class FontGenError(Exception):
    """本工具的可读错误（不打印栈）。"""


# ---------------------------------------------------------------------------
# 字表
# ---------------------------------------------------------------------------

def is_han(ch: str) -> bool:
    """是不是「一个 16x16 全角字符」。

    CJK 统一表意文字（含扩展 A）+ 全角标点（，。「」等）。半角 ASCII 不走这张表
    —— 它们由 OLED_F8x16 负责，混进来只会把字库撑大。
    ⚠️ 例外（2026-10 加）: EXTRA_WIDE 里那几个符号 —— 不是汉字, 但界面要用
    （OLED 第 2 页「自动下发QR:√/×」的勾/叉）, 且宋体 16px 渲染效果正好按全角算,
    所以与汉字同表同宽。只加**真的会显示**的: 多一个字白占 32 字节 flash。
    """
    o = ord(ch)
    return (
        ch in EXTRA_WIDE           # √ ×（界面符号, 见上）
        or 0x4E00 <= o <= 0x9FFF      # CJK 统一表意文字
        or 0x3400 <= o <= 0x4DBF   # 扩展 A
        or 0x3000 <= o <= 0x303F   # CJK 标点（、。「」等）
        or 0xFF00 <= o <= 0xFFEF   # 全角形式（含，。？！）
    )


def rows_of(path: str) -> list[str]:
    """读 glyphs.txt -> 每行一条「屏幕上的一行」（含 ASCII，`#` 是注释）。

    与 load_glyphs 的区别：这里**保留 ASCII** —— 用来算宽度。
    约定：glyphs.txt 里每条非注释行 = 屏幕上一行，所以同一行上并排的东西
    （如确认框的「>是  否」）要写在**同一行**里，宽度才算得对。
    """
    with open(path, encoding="utf-8") as fh:
        lines = fh.read().splitlines()
    return [s for s in (ln.split("#", 1)[0].strip() for ln in lines) if s]


def text_width(s: str) -> int:
    """一行文本的像素宽度：汉字 16、其余 8。"""
    return sum(16 if is_han(c) else 8 for c in s)


def load_glyphs(path: str) -> list[str]:
    """读 glyphs.txt -> 去重保序的字表（只保留汉字/全角，排序稳定）。"""
    try:
        with open(path, encoding="utf-8") as fh:
            raw = fh.read()
    except OSError as exc:
        raise FontGenError(f"读不到字表 {path}: {exc}") from exc

    seen: dict[str, None] = {}
    skipped: list[str] = []
    for line in raw.splitlines():
        line = line.split("#", 1)[0]           # 行内注释
        for ch in line:
            if ch.isspace():
                continue
            if not is_han(ch):
                skipped.append(ch)
                continue
            seen.setdefault(ch, None)

    if skipped:
        uniq = "".join(dict.fromkeys(skipped))
        print(f"提示: 已跳过 {len(uniq)} 个非汉字/全角字符: {uniq!r}", file=sys.stderr)

    if not seen:
        raise FontGenError(f"{path} 里没有任何汉字")
    return list(seen)


# ---------------------------------------------------------------------------
# 渲染
# ---------------------------------------------------------------------------

def _require_pil():
    try:
        from PIL import Image, ImageDraw, ImageFont  # noqa: F401
    except ImportError as exc:
        raise FontGenError(
            "需要 Pillow 才能生成字模（只在造字时用，固件不依赖它）。\n"
            "    安装: pip install pillow\n"
            f"    原始错误: {exc}"
        ) from exc
    from PIL import Image, ImageDraw, ImageFont
    return Image, ImageDraw, ImageFont


def render_glyph(ch: str, font_path: str, size: int, threshold: int) -> list[int]:
    """渲一个汉字 -> 32 字节（版式见模块 docstring）。

    ⚠️ 这里每一行都对应一个**被 --check 锁住**的决定，改之前先想清楚：
      · 画布 32x32 再裁左上 16x16 —— 字超过 em 框时宁可截掉，也不要整体偏移
        （偏移会让所有字模一起歪，且 golden 立刻失配）。
      · 落点必须是 (0,0)：PIL 的 text() 已按 ascender 定位，宋体 16px 正好占满
        0..15 行，这也正是江协那张表的口径。
      · fill=255 + threshold=128 是对抗锯齿的二值化。阈值调一点，笔画粗细就变，
        golden 会失配（说明确实动到了渲染结果）。
    """
    Image, ImageDraw, ImageFont = _require_pil()
    img = Image.new("L", (32, 32), 0)
    ImageDraw.Draw(img).text(
        (0, 0), ch, font=ImageFont.truetype(font_path, size), fill=255
    )
    px = img.crop((0, 0, 16, 16)).load()

    data: list[int] = []
    for page in range(2):          # 上半 8 行、下半 8 行
        for col in range(16):
            byte = 0
            for row in range(8):
                if px[col, page * 8 + row] >= threshold:
                    byte |= 1 << row        # bit0 = 页内顶行
            data.append(byte)
    return data


# ---------------------------------------------------------------------------
# 序列化
# ---------------------------------------------------------------------------

def _hex_block(data: list[int], indent: str) -> list[str]:
    """32 字节排成两行、每行 16 个，同江协原版排版（便于人工比对）。"""
    out = []
    for i in range(0, len(data), 16):
        row = ",".join(f"0x{b:02X}" for b in data[i:i + 16])
        out.append(f"{indent}{row},")
    return out


def build_table(glyphs: list[str], font_path: str, size: int, threshold: int) -> str:
    """生成 OLED_CF16x16[] 整段 C 文本（含标记）。"""
    lines = [
        BEGIN_MARK,
        "/* 本段由 tools/fontgen/gen_font.py 生成（字表: tools/fontgen/glyphs.txt）。",
        " * 手改会被下次生成覆盖。版式: 2 页 x 16 列, bit0 = 页内顶行。",
        " * ⚠️ 表尾那个 Index 为空串的「方框问号」默认图形不能删、不能挪 ——",
        " *    vOledShowChinese() 靠它判「扫完了」，缺了它会越界读。 */",
        "const ChineseCell_t OLED_CF16x16[] = {",
        "",
    ]
    for ch in glyphs:
        lines.append(f'\t"{ch}",')
        lines.extend(_hex_block(render_glyph(ch, font_path, size, threshold), "\t"))
        lines.append("")

    lines.append("\t/*未找到指定汉字时显示的默认图形（一个方框，内部一个问号）*/")
    lines.append("\t/*请确保其位于数组最末尾*/")
    lines.append(f'\t"{DEFAULT_BOX_INDEX}",')
    lines.extend(_hex_block(DEFAULT_BOX_DATA, "\t"))
    lines.append("")
    lines.append("};")
    lines.append(END_MARK)
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# 写回 OLED_Data.c（只动两个标记之间）
# ---------------------------------------------------------------------------

def splice(text: str, table: str) -> str:
    """把 table 换进 text 的两个标记之间。

    标记不存在 -> 追加到文件末尾（自举：第一次跑时给它建出来）。
    只有一个标记 / 顺序颠倒 -> 抛错。宁可不写，也不要猜着改（猜错就是静默损坏源码）。
    """
    nb, ne = text.count(BEGIN_MARK), text.count(END_MARK)
    if nb == 0 and ne == 0:
        sep = "" if text.endswith("\n") else "\n"
        return f"{text}{sep}\n{table}\n"
    if nb != 1 or ne != 1:
        raise FontGenError(
            f"{DATA_C} 里的标记不成对（开始 x{nb}, 结束 x{ne}）—— 请手工修好再跑。"
        )
    i = text.index(BEGIN_MARK)
    j = text.index(END_MARK)
    if j < i:
        raise FontGenError(f"{DATA_C} 里结束标记在开始标记之前 —— 请手工修好再跑。")
    return text[:i] + table + text[j + len(END_MARK):]


def read_spliced(text: str) -> str:
    """取出当前两个标记之间的内容（供 --check 比对漂移）。"""
    if text.count(BEGIN_MARK) != 1 or text.count(END_MARK) != 1:
        return ""
    i = text.index(BEGIN_MARK)
    j = text.index(END_MARK)
    return text[i:j + len(END_MARK)]


# ---------------------------------------------------------------------------
# 校验
# ---------------------------------------------------------------------------

def check_golden(font_path: str, size: int, threshold: int) -> list[str]:
    """断言参考字模逐字节相符。返回失败描述列表（空 = 全过）。"""
    bad = []
    for ch, exp in GOLDEN.items():
        try:
            got = render_glyph(ch, font_path, size, threshold)
        except FontGenError:
            raise
        if got != exp:
            diff = [i for i, (a, b) in enumerate(zip(exp, got)) if a != b]
            bad.append(f"{ch}: {len(diff)} 个字节不符, 首个在 byte{diff[0]} "
                       f"(期望 0x{exp[diff[0]]:02X}, 实得 0x{got[diff[0]]:02X})")
    return bad


# ---------------------------------------------------------------------------
# main
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description="生成 OLED_CF16x16[] 汉字字模并写回 Core/Src/OLED_Data.c",
    )
    ap.add_argument("--check", action="store_true",
                    help="只校验（黄金字模 + 与 OLED_Data.c 是否漂移），不写盘")
    ap.add_argument("--stdout", action="store_true", help="只打印 C 片段，不写盘")
    ap.add_argument("--font", default=DEFAULT_FONT, help=f"TTF/TTC 路径（默认 {DEFAULT_FONT}）")
    ap.add_argument("--size", type=int, default=DEFAULT_SIZE, help=f"字号（默认 {DEFAULT_SIZE}）")
    ap.add_argument("--threshold", type=int, default=DEFAULT_THRESHOLD,
                    help=f"二值化阈值（默认 {DEFAULT_THRESHOLD}）")
    ap.add_argument("--glyphs", default=GLYPHS_TXT, help="字表文件（默认同目录 glyphs.txt）")
    ap.add_argument("--widths", action="store_true",
                    help="逐行报告屏幕宽度（汉字 16px / ASCII 8px），不写盘")
    args = ap.parse_args(argv)

    # 宽度预算：屏幕上每行只有 128px，超了会被静默截掉（不是报错，是难看）。
    # 按「额外留一个 8px 光标位」算 —— 选中的那一项前面要画 '>'。
    if args.widths:
        try:
            rows = rows_of(args.glyphs)
        except OSError as exc:
            print(f"错误: 读不到 {args.glyphs}: {exc}", file=sys.stderr)
            return 2
        over = 0
        for s in rows:
            w = text_width(s) + 8          # +8 = 光标位
            mark = "超出!" if w > 128 else ("偏满" if w > 112 else "ok")
            if w > 128:
                over += 1
            print(f"{mark:>5}  {w:>3}px  {s}")
        print(f"\n{len(rows)} 行, {over} 行超 128px（含 8px 光标位）" if over
              else f"\n{len(rows)} 行, 全部在 128px 以内（含 8px 光标位）")
        return 1 if over else 0

    # 0) 先校验黄金字模 —— 参数不对就没必要往下走
    try:
        bad = check_golden(args.font, args.size, args.threshold)
    except FontGenError as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 2
    if bad:
        print("黄金字模校验失败 —— 渲染参数或打包顺序被动过：", file=sys.stderr)
        for line in bad:
            print(f"  · {line}", file=sys.stderr)
        print(f"\n对照基准是 tools/fontgen/gen_font.py 的 GOLDEN（江协原版的 6 个示例字）。\n"
              f"当前参数: font={args.font} size={args.size} threshold={args.threshold}", file=sys.stderr)
        return 1
    print(f"黄金字模校验通过: {len(GOLDEN)}/{len(GOLDEN)} 字逐字节相符")

    # 1) 字表 -> 全部字模
    try:
        glyphs = load_glyphs(args.glyphs)
        table = build_table(glyphs, args.font, args.size, args.threshold)
    except FontGenError as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 2
    print(f"字表 {args.glyphs}: {len(glyphs)} 字 -> {len(glyphs) * 32} 字节字模"
          f"（+ 表尾默认图形 32 字节）")

    if args.stdout:
        print(table)
        return 0

    # 2) 与 OLED_Data.c 比一比
    try:
        with open(DATA_C, encoding="utf-8") as fh:
            text = fh.read()
    except OSError as exc:
        print(f"错误: 读不到 {DATA_C}: {exc}", file=sys.stderr)
        return 2

    current = read_spliced(text)
    drift = (current != table)

    if args.check:
        if not current:
            print(f"漂移: {DATA_C} 里还没有汉字字模标记段 —— 先跑一次不带 --check 的生成",
                  file=sys.stderr)
            return 1
        if drift:
            print(f"漂移: {DATA_C} 里的字模段与字表生成结果不一致"
                  f"（手改过？或字表变了没重新生成）", file=sys.stderr)
            return 1
        print(f"无漂移: {DATA_C} 的字模段与字表一致")
        return 0

    if not drift:
        print(f"{DATA_C} 字模段已是最新，无需改动")
        return 0

    try:
        new_text = splice(text, table)
    except FontGenError as exc:
        print(f"错误: {exc}", file=sys.stderr)
        return 2

    with open(DATA_C, "w", encoding="utf-8", newline="") as fh:
        fh.write(new_text)
    print(f"已写回 {DATA_C}（只改了标记之间的整段）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
