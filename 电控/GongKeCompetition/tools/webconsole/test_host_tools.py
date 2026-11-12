#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""host_tools 的离线测试 —— 全部在 tempfile 沙箱里跑, 绝不碰真实的 ~/.ssh/config
与资源管理器「网络位置」。

    python tools/webconsole/test_host_tools.py

建 .lnk 的两条用例需要 Windows + PowerShell（非 Windows 上自动跳过）。
"""

import sys
import tempfile
import shutil
import unittest
from pathlib import Path
from unittest import mock

sys.path.insert(0, str(Path(__file__).resolve().parent))
import host_tools as ht                                              # noqa: E402

ON_WINDOWS = sys.platform == "win32"
UNC = "\\\\192.0.2.125\\shared"


class SandboxCase(unittest.TestCase):
    """把模块里所有会落盘的路径改指临时目录。"""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gks_test_"))
        self.ssh = self.tmp / "ssh" / "config"
        self.net = self.tmp / "Network Shortcuts"
        self.phrases = self.tmp / "phrases.json"
        self.ssh.parent.mkdir(parents=True)
        for name, val in (("SSH_CONFIG", self.ssh),
                          ("SSH_CONFIG_BAK", self.ssh.parent / (self.ssh.name + ".bak")),
                          ("NET_SHORTCUTS", self.net),
                          ("PHRASES_FILE", self.phrases)):
            p = mock.patch.object(ht, name, val)
            p.start()
            self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)
        self.ORIG = "Host pi\n    HostName 192.0.2.125\n    User pi\n"

    def _make_old_shortcut(self):
        """造一个「旧网段」的网络位置, 返回其目录。"""
        if not ON_WINDOWS:
            self.skipTest("建 .lnk 需要 Windows + PowerShell")
        d = self.net / "shared (192.0.2.125 (Samba))"
        self.assertTrue(ht._create_net_shortcut(None, d.name, UNC, lambda m: None))
        return d


class TestLnk(SandboxCase):
    """.lnk 生成与回读。这两条覆盖本次修的那个坑: 现代 Windows 的 .lnk 只存 UTF-16,
    只扫 ANSI 会恒返回 None。"""

    def test_roundtrip(self):
        d = self._make_old_shortcut()
        self.assertTrue((d / "desktop.ini").exists())
        self.assertEqual(ht.extract_unc_from_lnk(d / "target.lnk"), UNC)

    def test_matches_pywin32(self):
        """.lnk 用 PowerShell 建 == 用 pywin32 建（证明替换 COM 调用没改行为）。"""
        if not ON_WINDOWS:
            self.skipTest("需要 Windows")
        try:
            from win32com.client import Dispatch
        except ImportError:
            self.skipTest("没装 pywin32, 无从对比")

        a = self.tmp / "a.lnk"
        sc = Dispatch("WScript.Shell").CreateShortcut(str(a))
        sc.TargetPath = UNC
        sc.Save()

        self.net.mkdir(parents=True, exist_ok=True)
        b = self.net / "b"
        ht._create_net_shortcut(None, "b", UNC, lambda m: None)

        self.assertEqual(a.read_bytes(), (b / "target.lnk").read_bytes())

    def test_unreadable_lnk_is_none(self):
        junk = self.tmp / "junk.lnk"
        junk.write_bytes(b"\x00" * 64)
        self.assertIsNone(ht.extract_unc_from_lnk(junk))
        self.assertIsNone(ht.extract_unc_from_lnk(self.tmp / "nope.lnk"))


class TestUpdate(SandboxCase):

    def test_dry_run_changes_nothing(self):
        old = self._make_old_shortcut()
        self.ssh.write_text(self.ORIG, encoding="utf-8")
        log = []
        ht.do_update("203.0.113", log.append, dry_run=True)

        self.assertEqual(self.ssh.read_text(encoding="utf-8"), self.ORIG)
        self.assertFalse((self.ssh.parent / "config.bak").exists())
        self.assertTrue(old.exists())
        self.assertIn("DRY-RUN", "".join(log))

    def test_real_run_renames_and_backs_up(self):
        old = self._make_old_shortcut()
        self.ssh.write_text(self.ORIG, encoding="utf-8")
        log = []
        ht.do_update("203.0.113", log.append, dry_run=False)
        joined = "".join(log)

        # 认出了旧目录 → 走「改名」而不是「新建」
        self.assertIn("已更新", joined)
        self.assertNotIn("将创建", joined)

        self.assertEqual(self.ssh.read_text(encoding="utf-8"),
                         "Host pi\n    HostName 203.0.113.125\n    User pi\n")
        self.assertEqual((self.ssh.parent / "config.bak").read_text(encoding="utf-8"),
                         self.ORIG)
        self.assertFalse(old.exists(), "旧网络位置目录没删掉")

        new = self.net / "shared (203.0.113.125 (Samba))"
        self.assertTrue(new.exists())
        # 大小写不敏感: 目标主机**可达**时 Windows 会把共享名规范化成真实名字
        # (实测 \\203.0.113.125\shared → 存成 \SHARED), 不可达时原样保留。
        # do_update 只改 IP 那一段, 共享名大小写与它无关。
        self.assertEqual(ht.extract_unc_from_lnk(new / "target.lnk").lower(),
                         "\\\\203.0.113.125\\shared")

    def test_idempotent(self):
        self._make_old_shortcut()
        self.ssh.write_text(self.ORIG, encoding="utf-8")
        ht.do_update("203.0.113", lambda m: None, dry_run=False)

        log = []
        ht.do_update("203.0.113", log.append, dry_run=False)
        joined = "".join(log)
        self.assertIn("无需更新", joined)
        self.assertNotIn("已更新", joined)
        self.assertEqual(len(list(self.net.iterdir())), 1, "重复跑不该多出目录")

    def test_missing_ssh_config_is_skipped(self):
        log = []
        ht.do_update("203.0.113", log.append, dry_run=False)
        self.assertIn("SSH config 不存在", "".join(log))

    def test_every_ip_gets_renumbered(self):
        """⚠️ 原工具既有行为, 非本次引入: 文件里**每个** IPv4 的第三段都会被换成当前
        本机网段（末段保持）。这里把它钉住, 免得哪天被误当 bug「顺手修」而改变现场行为。"""
        self.ssh.write_text("Host a\n  HostName 10.1.2.125\n"
                            "Host b\n  HostName 192.0.2.9\n", encoding="utf-8")
        ht.do_update("203.0.113", lambda m: None, dry_run=False)
        got = self.ssh.read_text(encoding="utf-8")
        self.assertIn("203.0.113.125", got)
        self.assertIn("203.0.113.9", got)


class TestPure(unittest.TestCase):

    def test_replace_ip_in_path(self):
        self.assertEqual(ht.replace_ip_in_path("\\\\192.0.2.125\\shared", "10.0.0"),
                         "\\\\10.0.0.125\\shared")

    def test_detect_pi_ip(self):
        self.assertEqual(ht.detect_pi_ip("203.0.113"), "203.0.113.125")

    def test_host_info_shape(self):
        info = ht.host_info()
        self.assertEqual(set(info),
                         {"local_ip", "prefix", "pi_ip", "ssh", "share", "console",
                          "console_url"})
        self.assertEqual(info["pi_ip"], ht.detect_pi_ip(info["prefix"]))
        self.assertTrue(info["ssh"].startswith("pi@"))
        # 共享地址/控制台地址都由 pi_ip 拼出, 第三段跟着本机网段走
        self.assertEqual(info["share"], "\\\\%s\\shared" % info["pi_ip"])
        self.assertEqual(info["console"], "http://%s:%d/" % (info["pi_ip"], 9000))

    def test_console_url_is_this_machine_not_the_pi(self):
        """「本控制台」地址用的是**本机 IP + 本控制台端口** —— 发给队友的就是它。

        ⚠️ 别和 pi_console_url 搞混（那是树莓派的 :9000）。服务端绑 0.0.0.0,
           所以这个地址必须真的能连上: 它拼的就是 get_local_ip()。"""
        with mock.patch.object(ht, "get_local_ip", return_value="192.0.2.20"):
            self.assertEqual(ht.console_url(), "http://192.0.2.20:8410/")

    def test_pi_share_unc(self):
        self.assertEqual(ht.pi_share_unc("203.0.113.125"), "\\\\203.0.113.125\\shared")

    def test_pi_console_url(self):
        self.assertEqual(ht.pi_console_url("203.0.113.125"),
                         "http://203.0.113.125:9000/")

    def test_resolve_phrase_static(self):
        self.assertEqual(ht.resolve_phrase_text({"text": "看 {pi_ip} 这台"}, "1.2.3.4"),
                         "看 1.2.3.4 这台")

    def test_resolve_phrase_dynamic_falls_back(self):
        """连不上共享时给通配符兜底文案, 而不是抛异常。"""
        with mock.patch.object(ht, "find_latest_log", return_value=None):
            out = ht.resolve_phrase_text({"text": "x", "dynamic": "latest_log"}, "1.2.3.4")
        self.assertIn("logcat_*.log", out)
        self.assertIn("无法自动定位最新", out)

    # ---- PowerShell 调用的编码（中文 Windows 上是真会踩的坑, 见 _ps 的注释）----

    def test_ps_forces_utf8_on_both_ends(self):
        """`text=True` 不指定 encoding 会按系统码页（本机 GBK）解码 —— 中文报错文案变乱码。

        bt_serial._ps_stdout 已经踩过并修了同一个坑（那边还专门注明了「拿不到查询结果」）,
        这里对齐: ① 让 PowerShell 按 UTF-8 输出 ② subprocess 显式 encoding/errors。
        """
        done = mock.Mock(returncode=0, stdout="", stderr="")
        with mock.patch.object(ht.subprocess, "run", return_value=done) as run:
            ht._ps("Get-Date")
        argv = run.call_args[0][0]
        kw = run.call_args[1]
        self.assertIn("powershell", argv[0])
        self.assertIn("UTF8", " ".join(argv), "没让 PowerShell 自己按 UTF-8 输出")
        self.assertEqual(kw.get("encoding"), "utf-8")
        self.assertEqual(kw.get("errors"), "replace")

    def test_ps_reports_stderr_and_survives_garbage_bytes(self):
        """stderr 原样带出来（调用方拿它当失败原因展示）。"""
        done = mock.Mock(returncode=1, stdout="", stderr="  出错了  ")
        with mock.patch.object(ht.subprocess, "run", return_value=done):
            rc, err = ht._ps("boom")
        self.assertEqual(rc, 1)
        self.assertEqual(err, "出错了")


class TestPhrases(SandboxCase):

    def test_save_load_roundtrip(self):
        data = [{"name": "a", "text": "b"}, {"name": "c", "text": "d", "dynamic": "latest_log"}]
        ht.save_phrases(data)
        self.assertEqual(ht.load_phrases(), data)
        self.assertFalse((self.tmp / "phrases.json.tmp").exists(), "临时文件残留")

    def test_missing_file_is_empty(self):
        self.assertEqual(ht.load_phrases(), [])

    def test_corrupt_file_is_empty(self):
        self.phrases.write_text("{ not json", encoding="utf-8")
        self.assertEqual(ht.load_phrases(), [])


class TestRunAction(SandboxCase):

    def test_unknown_action(self):
        log = []
        ok, text = ht.run_action("nope", {}, log.append)
        self.assertFalse(ok)
        self.assertEqual(text, "")
        self.assertIn("未知动作", "".join(log))

    def test_phrase_action_returns_text(self):
        log = []
        ok, text = ht.run_action("phrase", {"phrase": {"text": "ip={pi_ip}"}}, log.append)
        self.assertTrue(ok)
        self.assertIn("ip=", text)
        self.assertNotIn("{pi_ip}", text)

    def test_missing_trae_reports_not_raises(self):
        with mock.patch.object(ht, "TRAE_EXE", self.tmp / "nope.exe"):
            log = []
            ok, _ = ht.run_action("trae", {}, log.append)
        self.assertFalse(ok)
        self.assertIn("未找到 Trae", "".join(log))

    def test_vscode_launch_args(self):
        """--remote ssh-remote+<user>@<pi> <home>, 与 Trae 同一套参数。"""
        with mock.patch.object(ht, "find_vscode", return_value=Path("/fake/Code.exe")), \
             mock.patch.object(ht, "_spawn", return_value=(True, "")) as sp:
            log = []
            ok, _ = ht.run_action("vscode", {}, log.append)
        self.assertTrue(ok)
        argv = sp.call_args[0][0]
        self.assertEqual(argv[1:], ["--remote", "ssh-remote+pi@" + ht.host_info()["pi_ip"],
                                    "/home/pi"])

    def test_missing_vscode_reports_not_raises(self):
        with mock.patch.object(ht, "VSCODE_CANDIDATES", (self.tmp / "nope.exe",)), \
             mock.patch.object(ht.shutil, "which", return_value=None):
            log = []
            ok, _ = ht.run_action("vscode", {}, log.append)
        self.assertFalse(ok)
        self.assertIn("未找到 VS Code", "".join(log))

    def test_console_opens_pi_url(self):
        with mock.patch.object(ht.webbrowser, "open") as op:
            log = []
            ok, _ = ht.run_action("console", {}, log.append)
        self.assertTrue(ok)
        self.assertEqual(op.call_args[0][0],
                         "http://%s:9000/" % ht.host_info()["pi_ip"])
        self.assertIn("9000", "".join(log))


if __name__ == "__main__":
    unittest.main(verbosity=2)
