#!/usr/bin/env python3
"""字库覆盖率检查 —— 界面上用到的汉字，字库里到底有没有？

为什么需要它
============
`vOledShowChinese()` 找不到某个字时**不报错**，而是画那个「方框问号」默认图形。
于是「加了句菜单文案、忘了重新生成字模」在编译期、运行期都一声不吭，只表现成
屏幕上多了几个方框 —— 而那种现象太容易被当成「屏坏了 / 字库没加载」。

本脚本把这件事变成一条**退出码**：
  · 源码里用到、字库里没有的字 → **报错**（退出码 1）—— 屏幕上会是方框
  · 字库里有、源码用不到的字   → 只提示（白占 32 字节 flash / 字）

只扫「字符串字面量里」的汉字，所以代码注释里的中文不会误报。

用法
====
    python tools/fontgen/check_coverage.py                 # 扫默认的界面源文件
    python tools/fontgen/check_coverage.py Application/*.c # 指定文件
    python tools/fontgen/check_coverage.py --quiet         # 只在有问题时输出

建议加进提交前的手检流程：改完菜单文案 / 动了 glyphs.txt 之后跑一次。
"""

from __future__ import annotations

import argparse
import glob
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
# ⚠️ is_han **只从 gen_font.py 导入**、绝不抄第二份 —— 那边 2026-10 加了 EXTRA_WIDE
# （√× 按 16px 全角进字库）之后, 本文件自己那份旧判据不认它们: 读 glyphs.txt 时把
# √× 跳过、读字库段时又收进来 → 误报「字库里有 2 个字不在 glyphs.txt 里」且退出码 1
# （2026-10-18 实测）。判据漂移过的坑, 别再挖第二个。
from gen_font import is_han  # noqa: E402

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DATA_C = os.path.join(REPO_ROOT, "Core", "Src", "OLED_Data.c")
GLYPHS_TXT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "glyphs.txt")

BEGIN_MARK = "/* ==== 汉字字模: 开始 ==== */"
END_MARK = "/* ==== 汉字字模: 结束 ==== */"

# 默认扫这些文件 —— 就是「会往 OLED 上写字」的那几个。
# app_oled_menu.c 是菜单的**数据表**（标签都在那张表的字符串字面量里, 由 Web 控制台
# 「OLED」Tab 生成/回写）。⚠️ 漏了它 = 菜单标签完全不在覆盖检查里, 改文案后屏上出
# 「方框问号」而**这条检查一声不吭** —— 那正是它存在的意义。
# app_beam.c / app_rpi.c（2026-10-31 加）: 「名字调参」行画值时走 ucOledOptLabel
# 读**选项标签**, 那些字面量住在 Beam 注册表（app_beam.c）与 Pi 参数表
# （app_rpi.c 的 s_acRpiCfgOpt*）—— 不扫它们, 给这两个来源的参数挂菜单行后
# 屏上出「方框问号」而检查一声不吭。
DEFAULT_TARGETS = ["Application/app_oled.c", "Application/app_oled_menu.c",
                   "Application/app_beam.c", "Application/app_rpi.c"]

# 匹配 C 字符串字面量（只到行尾，够用 —— 本工程的字面量都不换行）
RE_STRING = re.compile(r'"((?:[^"\\]|\\.)*)"')


def load_font_chars(path: str = DATA_C) -> tuple[set[str], str | None]:
    """从 OLED_Data.c 的字模段里取所有 Index（返回 (字集合, 错误信息)）。"""
    try:
        with open(path, encoding="utf-8") as fh:
            text = fh.read()
    except OSError as exc:
        return set(), f"读不到 {path}: {exc}"

    if text.count(BEGIN_MARK) != 1 or text.count(END_MARK) != 1:
        return set(), f"{path} 里的汉字字模标记不成对 —— 先跑 gen_font.py"

    seg = text[text.index(BEGIN_MARK):text.index(END_MARK)]
    # 每条记录 = 缩进 + "汉字" + 逗号
    cells = re.findall(r'^\t"(.*)",\s*$', seg, re.M)
    return {c for c in cells if c}, None


def load_glyph_list(path: str = GLYPHS_TXT) -> set[str]:
    """glyphs.txt 里列的字（真值来源；字库应与它一致）。"""
    try:
        with open(path, encoding="utf-8") as fh:
            raw = fh.read()
    except OSError:
        return set()

    out: set[str] = set()
    for line in raw.splitlines():
        for ch in line.split("#", 1)[0]:
            if is_han(ch):
                out.add(ch)
    return out


def scan_sources(patterns: list[str]) -> dict[str, str]:
    """扫源文件 -> {汉字: 它出现在哪个字符串字面量里}（只取第一次出现）。"""
    used: dict[str, str] = {}
    for pat in patterns:
        for path in glob.glob(os.path.join(REPO_ROOT, pat)) or [pat]:
            try:
                with open(path, encoding="utf-8") as fh:
                    src = fh.read()
            except OSError as exc:
                print(f"警告: 读不到 {path}: {exc}", file=sys.stderr)
                continue

            # 先去注释，避免注释里的中文被算成"用到"（行注释 + 块注释都去掉）
            src = re.sub(r"/\*.*?\*/", "", src, flags=re.S)
            src = re.sub(r"//[^\n]*", "", src)

            for lit in RE_STRING.findall(src):
                for ch in lit:
                    if is_han(ch):
                        used.setdefault(ch, lit)
    return used


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="检查 OLED 字库对界面文案的覆盖")
    ap.add_argument("targets", nargs="*", default=DEFAULT_TARGETS,
                    help=f"要扫的源文件（默认 {' '.join(DEFAULT_TARGETS)}）")
    ap.add_argument("--quiet", action="store_true", help="只在有问题时输出")
    args = ap.parse_args(argv)

    font, err = load_font_chars()
    if err:
        print(f"错误: {err}", file=sys.stderr)
        return 2

    listed = load_glyph_list()
    used = scan_sources(args.targets)

    missing = {c: l for c, l in used.items() if c not in font}
    unused = sorted(font - set(used))
    # 字库与 glyphs.txt 不一致（改了字表没重新生成 / 手改了字库段）
    not_in_list = sorted(font - listed) if listed else []
    not_in_font = sorted(listed - font)

    if not args.quiet:
        print(f"字库: {len(font)} 字 | 扫了 {len(args.targets)} 个源文件, 用到 {len(used)} 字")

    rc = 0

    if missing:
        rc = 1
        print(f"\n✗ 字库缺 {len(missing)} 个字 —— 屏幕上会显示成「方框问号」:", file=sys.stderr)
        for c, lit in sorted(missing.items()):
            print(f"    {c}   出现在 {lit!r}", file=sys.stderr)
        print("\n  修法: 把缺的字补进 tools/fontgen/glyphs.txt, 再跑",
              file=sys.stderr)
        print("        python tools/fontgen/gen_font.py", file=sys.stderr)

    if not_in_list:
        rc = 1
        print(f"\n✗ 字库里有 {len(not_in_list)} 个字不在 glyphs.txt 里"
              f"（手改了字模段?）: {''.join(not_in_list)}", file=sys.stderr)
        print("  下次跑 gen_font.py 会把这几个字**删掉**。", file=sys.stderr)

    if not_in_font:
        rc = 1
        print(f"\n✗ glyphs.txt 里有 {len(not_in_font)} 个字没进字库"
              f"（忘了重新生成?）: {''.join(not_in_font)}", file=sys.stderr)
        print("  修法: python tools/fontgen/gen_font.py", file=sys.stderr)

    if unused:
        # 只提示：不算错（但 32 字节/字，值得知道）
        print(f"\n提示: 字库里有 {len(unused)} 个字界面没用到, 白占 "
              f"{len(unused) * 32} 字节 flash: {''.join(unused)}")

    if rc == 0:
        print("\n✓ 覆盖完整: 界面用到的字全部在字库里, 且字库与 glyphs.txt 一致")
    return rc


if __name__ == "__main__":
    sys.exit(main())
