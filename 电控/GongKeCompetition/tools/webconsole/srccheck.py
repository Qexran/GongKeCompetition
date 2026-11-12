#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""测试用的源码契约检查小工具（**只给测试用, 不参与服务运行**）。

本项目的控制台有几个「漏改不会报错、只会在运行时静默失效」的跨文件契约
（前端契约、路由登记、Origin 校验元组…）。既有做法是读源码文本做字符串断言
（见 test_bt.py::TestFrontendContract、test_capture.py::TestFrontendContract）——
便宜且有效, 但**别再用「某个子串出现几次」那种数法**: 加第三个端点时它会失配,
却不告诉你漏了哪一个地方。

这里把「把元组抠出来再查成员」固定成一个帮手, 两个测试文件共用一份。
"""

import re
import shutil
import subprocess
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent


def read_source(name):
    """读 tools/webconsole 下的一个源文件（UTF-8）。"""
    return (HERE / name).read_text(encoding="utf-8")


# 让 app.js 能在 node 里跑起来的**最小** DOM 桩。
# 只在真正调用到某个 API 时才需要更细的实现; 顶层构造（对象字面量、makeDnd 注册
# 监听器…）用这些就够了。
_JS_STUB = r"""
const __listeners = {};
globalThis.document = {
  addEventListener: (t, f) => { (__listeners[t] = __listeners[t] || []).push(f); },
  getElementById: () => null, querySelector: () => null, querySelectorAll: () => [],
  createElement: () => ({ classList: { add(){}, remove(){}, toggle(){} }, style: {},
                          appendChild(){}, addEventListener(){}, removeAttribute(){} }),
  body: { appendChild(){} }, activeElement: null, documentElement: {},
};
globalThis.window = globalThis;
globalThis.addEventListener = globalThis.document.addEventListener;   /* window 级监听器（app.js 的 blur 兜底在用）走同一个登记表 */
globalThis.Image = function () {};
globalThis.URL = { createObjectURL: () => '' };
globalThis.requestAnimationFrame = (f) => setTimeout(f, 0);
globalThis.fetch = () => Promise.reject(new Error('stub'));
globalThis.setInterval = () => 0;
globalThis.clearInterval = () => {};
globalThis.confirm = () => false;
globalThis.alert = () => {};
"""


def node_available():
    return shutil.which("node") is not None


def run_js_top_level(js_text, probe="", timeout=60):
    """在 node 里执行脚本的**顶层**（不触发 DOMContentLoaded）。

    抓的是「页面一加载就炸」的引用错误 —— 服务端测试看不见, `node --check` 只看语法
    也看不见, 只有真的打开页面才会发现。probe 里的表达式结果会打到 stdout。
    返回 (退出码, 输出文本)；没装 node 时抛 RuntimeError。
    """
    if not node_available():
        raise RuntimeError("没有 node")
    with tempfile.TemporaryDirectory(prefix="gks_js_") as td:
        f = Path(td) / "probe.js"
        f.write_text(_JS_STUB + js_text + "\n" + probe, encoding="utf-8")
        p = subprocess.run([shutil.which("node"), str(f)],
                           capture_output=True, text=True, encoding="utf-8",
                           errors="replace", timeout=timeout)
    return p.returncode, (p.stdout or "") + (p.stderr or "")


def guard_tuples(server_src):
    """从 server.py 源码里抠出 (POST 白名单元组, Origin 前缀元组) 的文本。

    do_POST 开头那两处是「新增端点必须同步登记」的地方, 漏一处就静默失效:
      · 白名单漏了  → 该端点的 POST 一律 404
      · Origin 漏了 → 任意网页都能把它当跳板（会驱动硬件/写仓库文件的那种最危险）

    ⚠️⚠️ 必须 `^\\s*if`（MULTILINE）—— 别退回裸 `if self\\.path\\.startswith\\(`:
        `do_GET` 里的 `elif self.path.startswith("/static/"):` **含有**子串
        "if self.path.startswith("，于是 re.search 从那儿开始、一路吃到 do_POST 的
        `)) and`, 抠出来的是一大段**跨越 do_GET 的文本**。那样两条 assertIn
        会因为「do_GET 的 elif 里也出现过这个路径」而**永远通过** —— 加了新路径
        照样绿, 漏了也照样绿, 正是这只帮手要消灭的假安全感。

    ⚠️ `.*?` 到第一个 `)` 就停 → 被测的元组里不能出现嵌套括号
       （`startswith(tuple(X))` 这种写法会抠出半截）。调用方保持内联字面量元组即可。
    """
    whitelist = re.search(r"^\s*if self\.path not in \((.*?)\):",
                          server_src, re.S | re.M)
    origin = re.search(r"^\s*if self\.path\.startswith\((.*?)\)\s*and",
                       server_src, re.S | re.M)
    return (whitelist.group(1) if whitelist else "",
            origin.group(1) if origin else "")
