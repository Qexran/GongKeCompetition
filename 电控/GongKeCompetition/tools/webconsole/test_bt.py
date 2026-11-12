#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""蓝牙 Tab 的单元测试：python tools/webconsole/test_bt.py

全程沙箱: 不打开真实 COM 口（用 FakePort 顶替 bt_serial.SerialPort）、不碰真实
bt.json（mock.patch.object 换掉 bt_link.BT_JSON）。非 Windows 上也能跑。

覆盖四层:
  1. 纯函数      —— $B 行解析、标签清洗、动作片段生成（含与 map.c 序列化器的往返）
  2. 偏好文件    —— bt.json 读写/损坏兜底/原子写
  3. 会话        —— 拆包/粘包/AF 结束符、超时、错误码翻译、读线程独占 ReadFile
  4. 服务端动作  —— run_bt_action 的分发与异常兜底、bt_state 的自动选中
"""

import ctypes
import io
import json
import re
import os
import shutil
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path
from unittest import mock

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import bt_link as B        # noqa: E402
import bt_serial as BS     # noqa: E402
import server as S         # noqa: E402
import srccheck            # noqa: E402


# =====================================================================
# 1. 纯函数
# =====================================================================

class TestParse(unittest.TestCase):

    def test_param_line(self):
        p = B.parse_param_line(
            "$B,P,chassis_accel,底盘加速度,底盘,I32,1,1,0.00,2000.00,rpm/s,230")
        self.assertEqual(p["name"], "chassis_accel")
        self.assertEqual(p["label"], "底盘加速度")
        self.assertEqual(p["group"], "底盘")
        self.assertEqual(p["type"], "I32")
        self.assertTrue(p["rw"])
        self.assertTrue(p["warn"])
        self.assertEqual(p["min"], 0.0)
        self.assertEqual(p["max"], 2000.0)
        self.assertEqual(p["unit"], "rpm/s")
        self.assertEqual(p["value"], "230")
        # 没填 pcOptions 的普通参数**行尾少一个字段** —— 12 段必须照样解析出来,
        # 且 options 为 None（写成 `!= 13` 会让整张参数面板空掉）。
        self.assertIsNone(p["options"])

    def test_param_line_with_options(self):
        """末尾可选 options 字段（固件 pcOptions 非空才出现）→ 前端据此渲染下拉框。"""
        p = B.parse_param_line(
            "$B,P,gyro_src,陀螺仪首选,陀螺仪,U8,1,1,0.00,1.00,,1,0=主口那颗|1=副口那颗")
        self.assertIsNotNone(p)
        self.assertEqual(p["value"], "1")
        self.assertEqual(p["options"], [{"value": "0", "label": "主口那颗"},
                                        {"value": "1", "label": "副口那颗"}])
        # 选项串**不能**含逗号（$B 无转义, 一个逗号就把记录串行）—— 这条锁住分隔符是 `|`。
        self.assertNotIn(",", p["options"][0]["label"])

    def test_parse_options_garbage(self):
        self.assertIsNone(B.parse_options(""))
        self.assertIsNone(B.parse_options("没有等号"))
        self.assertEqual(B.parse_options("0=开"), [{"value": "0", "label": "开"}])

    def test_param_line_empty_unit_keeps_field_count(self):
        """单位为空时字段数不能塌 —— 否则逗号切分整体错位。"""
        p = B.parse_param_line("$B,P,x,名字,组,F32,1,0,0.00,1.00,,0.50")
        self.assertIsNotNone(p)
        self.assertEqual(p["unit"], "")
        self.assertEqual(p["value"], "0.50")

    def test_param_line_rejects_wrong_shape(self):
        self.assertIsNone(B.parse_param_line("$B,OK"))
        self.assertIsNone(B.parse_param_line("$B,J,0,s1,0.00,180.00,90.00"))
        self.assertIsNone(B.parse_param_line("$B,P,a,b,c,d,1,0,0,1,u"))
        self.assertIsNone(B.parse_param_line("garbage"))

    def test_joint_line(self):
        j = B.parse_joint_line("$B,J,2,s3,0.00,180.00,135.50")
        self.assertEqual(j, {"index": 2, "name": "s3",
                             "min": 0.0, "max": 180.0, "angle": 135.5})

    def test_pos_line(self):
        p = B.parse_pos_line("$B,POS,1.00,2.00,3.00,4.00,180.00,45.00,1")
        self.assertEqual(p["angles"], [1.0, 2.0, 3.0, 4.0, 180.0])
        self.assertEqual(p["gripper"], 45.0)
        self.assertTrue(p["done"])
        self.assertIsNone(B.parse_pos_line("$B,POS,1,2,3"))


class TestSanitizeTag(unittest.TestCase):
    """标签要能安全进 map.c —— 「地图」Tab 的解析器对 // 和括号敏感。"""

    def test_strips_double_slash(self):
        out = B.sanitize_tag("抓取//A")
        self.assertNotIn("//", out)

    def test_strips_quotes_and_backslash(self):
        out = B.sanitize_tag('a"b\\c')
        self.assertNotIn('"', out)
        self.assertNotIn("\\", out)

    def test_drops_unbalanced_ascii_parens(self):
        self.assertEqual(B.sanitize_tag("抓取(A"), "抓取A")
        self.assertEqual(B.sanitize_tag("抓取A)"), "抓取A")
        # 配对的留着
        self.assertEqual(B.sanitize_tag("抓取(A)"), "抓取(A)")

    def test_keeps_fullwidth_parens(self):
        """全角括号是安全的（map.c 里已有先例），不能误删。"""
        self.assertEqual(B.sanitize_tag("打开激光（继电器 JIGUANG）"),
                         "打开激光（继电器 JIGUANG）")

    def test_empty_gets_fallback(self):
        self.assertEqual(B.sanitize_tag(""), "臂组动作")
        self.assertEqual(B.sanitize_tag("   "), "臂组动作")
        self.assertEqual(B.sanitize_tag(None), "臂组动作")

    def test_truncates(self):
        self.assertLessEqual(len(B.sanitize_tag("x" * 200)), B.TAG_MAX_LEN)

    # ---- 以下三条钉的是「规则本体」在 map_codec 里的性质, 两条写入路径都依赖它 ----

    def test_repeated_slashes_all_collapse(self):
        """`//` 必须**折干净**, 不能只折一次。

        只折一次的话 `'a///b'` 会剩下 `'a//b'` —— 而 _ensure_comma 在行内**第一个**
        `//` 处插尾逗号, 那个逗号就落进字符串里了（正是本函数存在的理由）。
        """
        for raw in ("a//b", "a///b", "a////b", "a//b//c"):
            out = B.sanitize_tag(raw)
            self.assertNotIn("//", out, "%s 没折干净 → %s" % (raw, out))

    def test_is_idempotent(self):
        """幂等是**硬要求**: make_arm_snippet 洗一遍、_serialize_step 再洗一遍是常态,
        不幂等就会洗出两个不同结果（test_matches_serializer_byte_for_byte 逐字节比对）。"""
        for raw in ("抓取///A", 'a"b\\c', "抓取(A", "x" * 200, "", None,
                    "打开激光（继电器 JIGUANG）"):
            once = B.sanitize_tag(raw)
            self.assertEqual(B.sanitize_tag(once), once, "再洗一次结果变了: %r" % raw)

    def test_truncation_never_leaves_unbalanced_parens(self):
        """先截断、再配平 —— 反过来会让截断把刚配好的括号重新打断。

        ⚠️ 真实数据可达: app_script.h 的积木 desc（新积木名的默认来源）里有 7 条因此
           产出不成对的 `(`; 不成对的括号会让 _parse_body 的括号计数错位, 条目被错误刷出。
        """
        for raw in ("vChassisMoveWorld(wx, wy) 后面还有一长串说明文字",
                    "重置脚本: 重启**当前这一帧**所在的表(不弹出跳转栈)并且继续往下讲",
                    "((((((((((x"):
            out = B.sanitize_tag(raw)
            self.assertEqual(out.count("("), out.count(")"),
                             "截断后括号不成对: %r → %r" % (raw, out))


_DWORD_P = ctypes.POINTER(ctypes.c_uint)


class FakeKernel:
    """最小 kernel32 替身（只实现 SerialPort 用到的那几个调用）。

    ⚠️ 必须用 `ctypes.WINFUNCTYPE` 造**真的** ctypes 可调用对象, 不能拿普通 Python 函数顶替:
       SerialPort 传进来的是 `ctypes.byref(nwritten)`, 只有真的 ctypes 函数才会把它当指针,
       我才能把「实际写出去几个字节」写回去。普通函数收到的是个 CArgObject, 写不进去 ——
       那样测出来的只是我自己造的数。

    write_chunks: 每次 WriteFile 最多写几个字节（模拟部分写）; 用空就一次写完。
    read_chunks:  每次 ReadFile 返回什么（用空就返回 0 字节 = 超时无数据）。
    """

    def __init__(self, write_chunks=None, read_chunks=None):
        self.write_chunks = list(write_chunks or [])
        self.read_chunks = list(read_chunks or [])
        self.written = []
        self.closed = []
        self.read_calls = 0
        self.WriteFile = ctypes.WINFUNCTYPE(
            ctypes.c_int, ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint,
            _DWORD_P, ctypes.c_void_p)(self._write)
        self.ReadFile = ctypes.WINFUNCTYPE(
            ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint,
            _DWORD_P, ctypes.c_void_p)(self._read)
        self.CloseHandle = ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.c_void_p)(self._close)

    def _write(self, h, buf, n, pwritten, ov):
        take = min(n, self.write_chunks.pop(0)) if self.write_chunks else n
        self.written.append(bytes(buf[:take]))
        pwritten[0] = take
        return 1

    def _read(self, h, buf, size, pnread, ov):
        self.read_calls += 1
        data = self.read_chunks.pop(0) if self.read_chunks else b""
        data = data[:size]
        if data:
            ctypes.memmove(buf, data, len(data))
        pnread[0] = len(data)
        return 1

    def _close(self, h):
        self.closed.append(h)
        return 1


class TestSerialPortIo(unittest.TestCase):
    """`bt_serial.SerialPort` 的 Win32 那一层。

    ⚠️ 这一层原本**零覆盖** —— 其余测试都用 FakePort 整个顶替掉 SerialPort, 于是
       read/write/close 里 ctypes 相关的逻辑（部分写、句柄竞态）从来没被执行过。
       这里用 FakeKernel 顶替 `_k32`（不是顶替 SerialPort）, 补上那一段。
    """

    def _port(self):
        p = BS.SerialPort("COM_TEST")
        p._h = 0x1234                     # 假装已打开; 句柄值本身无意义（kernel 是假的）
        return p

    def test_write_loops_until_every_byte_is_sent(self):
        """WriteFile 允许只写一部分（串口流控 / 驱动缓冲满时会发生）。

        老写法只看第一次的返回值、且调用方不看它 —— 命令被**静默截断**, 而
        BtLink._cmd_lock 那套请求/响应配对还在等一个永远不会来的应答
        （现场现象是「某条指令偶尔不生效」, 最难查的那种）。
        """
        payload = b"$B,PLAY,3\n"
        k = FakeKernel(write_chunks=[2, 3, 1, 100])
        with mock.patch.object(BS, "_k32", k):
            n = self._port().write(payload)
        self.assertEqual(n, len(payload))
        self.assertEqual(b"".join(k.written), payload, "命令被截断了")

    def test_zero_byte_write_raises_instead_of_spinning(self):
        """报告成功却一个字节没写 —— 再循环就是死转, 必须报出来。"""
        k = FakeKernel(write_chunks=[0])
        with mock.patch.object(BS, "_k32", k):
            with self.assertRaises(BS.SerialError):
                self._port().write(b"abc")

    def test_write_after_close_raises(self):
        k = FakeKernel()
        with mock.patch.object(BS, "_k32", k):
            p = self._port()
            p.close()
            with self.assertRaises(BS.SerialError):
                p.write(b"abc")
        self.assertEqual(k.written, [])

    def test_read_after_close_raises_before_touching_win32(self):
        """句柄必须**在锁内**取并判空。

        在锁外判 `self._h is None`、锁内又读一次的话, 中间的 close() 会把 None 塞进去 ——
        ReadFile(None) 报的是「无效句柄」, 看着像掉线, 其实只是关闭竞态, 会把排障带偏。
        """
        k = FakeKernel()
        with mock.patch.object(BS, "_k32", k):
            p = self._port()
            p.close()
            with self.assertRaises(BS.SerialError):
                p.read()
        self.assertEqual(k.read_calls, 0, "已经关掉了还去调 ReadFile")

    def test_read_timeout_returns_empty_not_error(self):
        """读超时无数据返回 b''（不是错误）—— 上层靠这个区分「安静」与「掉线」。"""
        k = FakeKernel(read_chunks=[b"", b"$B,OK\n"])
        with mock.patch.object(BS, "_k32", k):
            p = self._port()
            self.assertEqual(p.read(), b"")
            self.assertEqual(p.read(), b"$B,OK\n")

    def test_close_waits_for_an_in_flight_io(self):
        """close() 必须持 _io_lock。

        读线程可能正阻塞在同一个句柄的 ReadFile 里; 不持锁就 CloseHandle 会撞上
        「I/O 在飞时关句柄」—— 句柄值被复用后那次 ReadFile 可能读到别的东西或报假错。
        （BtLink.disconnect 先 join(1.0) 再 close, 正常路径撞不上; join 超时那一支才是裸竞态。）
        """
        k = FakeKernel()
        with mock.patch.object(BS, "_k32", k):
            p = self._port()
            p._io_lock.acquire()                  # 冒充「读线程正拿着锁做 ReadFile」
            t = threading.Thread(target=p.close)
            t.start()
            t.join(0.2)
            self.assertTrue(t.is_alive(), "close() 没等 _io_lock —— 关闭竞态还在")
            p._io_lock.release()
            t.join(2.0)
            self.assertFalse(t.is_alive(), "close() 在锁释放后没能完成")
        self.assertEqual(k.closed, [0x1234])

    def test_close_is_idempotent(self):
        k = FakeKernel()
        with mock.patch.object(BS, "_k32", k):
            p = self._port()
            p.close()
            p.close()                             # 第二次不该再关一次（句柄会被复用）
        self.assertEqual(k.closed, [0x1234])


class TestSnippet(unittest.TestCase):

    ITEM = {"name": "抓取A", "angles": [15, 30, 45, 0, 180],
            "gripper": {"enabled": True, "kai_he": 1, "degree": 100}}

    def test_wait_variants(self):
        self.assertIn("STEP_ARM_GROUP_WAIT", B.make_arm_snippet([self.ITEM], wait=True)[0])
        self.assertIn("STEP_ARM_GROUP_NO_WAIT", B.make_arm_snippet([self.ITEM], wait=False)[0])
        self.assertIn("vArmGroupMoveNoWait()", B.make_arm_snippet([self.ITEM], wait=False)[0])

    def test_matches_serializer_byte_for_byte(self):
        """生成的 F5/I2 行必须与「地图」Tab 的 _serialize_step 完全一致 ——
        这样贴回 map.c 后，控制台自己再写一遍也不会产生 diff。"""
        cat = {c["type"]: c for c in S.parse_catalog()}
        snip = B.make_arm_snippet([self.ITEM], wait=True)

        ref = S._serialize_step({"step": "STEP_ARM_GROUP_WAIT", "tag": "抓取A",
                                 "params": ["15", "30", "45", "0", "180"],
                                 "edited": True}, cat)
        self.assertEqual(snip[0], ref[0])

        refg = S._serialize_step({"step": "STEP_SET_ZHUASHOU_WAIT", "tag": "抓取A-夹爪",
                                  "params": ["1", "100"], "edited": True}, cat)
        self.assertEqual(snip[1], refg[0])

    def test_survives_map_roundtrip(self):
        """真正要保证的：贴进 map.c → 解析 → 重新序列化 = 零 diff。"""
        cat = {c["type"]: c for c in S.parse_catalog()}
        lines = ["    /* ===== 分区A ===== */"] + B.make_arm_snippet([self.ITEM]) + [
            '    SCRIPT_STEP        (STEP_LASER_ON,                        "开激光"),  // vRelayOn()']
        self.assertEqual(S.serialize_map(S._parse_body(lines), cat), lines)

    def test_defaults_and_padding(self):
        out = B.make_arm_snippet([{"name": "只有三个角", "angles": [1, 2, 3]}])
        self.assertEqual(len(out), 1)                       # 没有夹爪行
        self.assertIn("1.0f, 2.0f, 3.0f, 0.0f, 0.0f", out[0])

    def test_malicious_tag_is_neutralised(self):
        out = B.make_arm_snippet([{"name": 'x" // (', "angles": [0] * 5}])
        body = out[0]
        self.assertNotIn("//", body.split('"')[1])          # 标签里没有 //
        self.assertEqual(body.count('"'), 2)                # 只有一对引号

    def test_empty(self):
        self.assertEqual(B.make_arm_snippet([]), [])
        self.assertEqual(B.make_arm_snippet(None), [])


class TestErrText(unittest.TestCase):

    def test_known_codes(self):
        self.assertIn("握手", B._err_text("$B,ERR,NOHANDSHAKE"))
        self.assertIn("编辑模式", B._err_text("$B,ERR,NOTEDIT"))
        self.assertIn("只读", B._err_text("$B,ERR,READONLY"))
        self.assertIn("动作预设", B._err_text("$B,ERR,NOACTION"))

    def test_range_carries_bounds(self):
        self.assertIn("0.00", B._err_text("$B,ERR,RANGE,0.00,5.00"))
        self.assertIn("5.00", B._err_text("$B,ERR,RANGE,0.00,5.00"))

    def test_unknown_code_is_echoed(self):
        self.assertIn("WEIRD", B._err_text("$B,ERR,WEIRD"))

    def test_empty_does_not_crash(self):
        self.assertTrue(B._err_text(""))
        self.assertTrue(B._err_text(None))


# =====================================================================
# 2. 偏好文件
# =====================================================================

class TestPrefs(unittest.TestCase):

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gks_bt_"))
        self.json = self.tmp / "bt.json"
        p = mock.patch.object(B, "BT_JSON", self.json)
        p.start()
        self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def test_missing_returns_empty(self):
        self.assertEqual(B.load_prefs(), {})

    def test_corrupt_returns_empty(self):
        self.json.write_text("{ not json", encoding="utf-8")
        self.assertEqual(B.load_prefs(), {})

    def test_non_dict_returns_empty(self):
        self.json.write_text("[1,2,3]", encoding="utf-8")
        self.assertEqual(B.load_prefs(), {})

    def test_roundtrip(self):
        self.assertTrue(B.save_prefs("COM7", 115200))
        self.assertEqual(B.load_prefs(), {"port": "COM7", "baud": 115200})

    def test_atomic_no_tmp_residue(self):
        B.save_prefs("COM3", 9600)
        leftovers = [f.name for f in self.tmp.iterdir() if f.name.endswith(".tmp")]
        self.assertEqual(leftovers, [])


# =====================================================================
# 3. 会话（FakePort 顶替真实串口）
# =====================================================================

class FakePort:
    """假串口。write() 把预设应答塞进读队列；read() 每次最多吐 chunk 字节,
    用来模拟 BT 链路上的半包/粘包。"""

    responses = {}
    read_threads = []

    def __init__(self, port, baud=115200, read_timeout_ms=50):
        self.port = port
        self.baud = baud
        self._open = False
        self._out = bytearray()
        self._lock = threading.Lock()
        self.chunk = 7
        self.fail_on_open = None
        self.written = []
        self.closed = False

    def open(self):
        if self.fail_on_open:
            raise BS.SerialError(self.fail_on_open)
        self._open = True

    def close(self):
        self._open = False
        self.closed = True

    @property
    def is_open(self):
        return self._open

    def write(self, data):
        cmd = data.decode("utf-8").strip()
        self.written.append(cmd)
        with self._lock:
            for key, lines in FakePort.responses.items():
                if key in cmd:
                    self._out += "".join(l + "\r\n" for l in lines).encode("utf-8")
                    return len(data)
            self._out += b"$B,ERR,UNKNOWN\r\n"
        return len(data)

    def read(self, size=4096):
        FakePort.read_threads.append(threading.current_thread().name)
        with self._lock:
            if not self._out:
                time.sleep(0.005)
                return b""                      # 超时无数据：不是错误
            n = min(size, self.chunk, len(self._out))
            chunk, self._out = bytes(self._out[:n]), self._out[n:]
            return chunk


class LinkCase(unittest.TestCase):
    """成功 connect 会调 save_prefs —— 所以 BT_JSON 也必须沙箱化,
    否则测试会往仓库里写一个真实的 bt.json。"""

    def setUp(self):
        FakePort.responses = {}
        FakePort.read_threads = []
        self._ports = []

        tmp = Path(tempfile.mkdtemp(prefix="gks_bt_link_"))
        pj = mock.patch.object(B, "BT_JSON", tmp / "bt.json")
        pj.start()
        self.addCleanup(pj.stop)
        self.addCleanup(shutil.rmtree, tmp, ignore_errors=True)

        def factory(port, baud=115200, read_timeout_ms=50):
            fp = FakePort(port, baud)
            self._ports.append(fp)
            return fp

        p = mock.patch.object(BS, "SerialPort", side_effect=factory)
        p.start()
        self.addCleanup(p.stop)
        # connect 的 PING 重试间隔是真机节奏（0.3s）, 测试里别真等 ——
        # 各用例要动次数/超时再自行 patch（常量在方法里运行时才取, patch 得动）
        pd = mock.patch.object(B, "PING_RETRY_DELAY", 0.01)
        pd.start()
        self.addCleanup(pd.stop)
        self.link = B.BtLink()
        self.addCleanup(self.link.disconnect)

    def last_port(self):
        return self._ports[-1]


class TestLinkFraming(LinkCase):

    def test_connect_requires_pong(self):
        FakePort.responses = {"$B,PING": ["$B,PONG,1", "$B,OK"]}
        self.assertTrue(self.link.connect("COM9", 115200))
        self.assertTrue(self.link.is_connected)
        self.assertEqual(self.last_port().written[0], "$B,PING")

    def test_connect_without_pong_raises_and_cleans_up(self):
        FakePort.responses = {}                 # 一律回 ERR,UNKNOWN
        with self.assertRaises(B.BtError):
            self.link.connect("COM9", 115200)
        self.assertFalse(self.link.is_connected)
        self.assertTrue(self.last_port().closed)

    def test_connect_retries_ping_when_first_one_is_dropped(self):
        """「连上了但设备没应答」最常见的原因：建链瞬间头几个字节被丢（模块
        SPP 通路/流控未就绪、刚上电刚醒来），第一次 PING 石沉大海 ——
        必须重试到应答为止，而不是报错让用户「再点一次」。"""
        pings = {"n": 0}
        real_write = FakePort.write

        def drop_first_ping(self, data):
            if b"$B,PING" in data:
                pings["n"] += 1
                if pings["n"] == 1:
                    self.written.append(data.decode("utf-8").strip())
                    return len(data)            # 这次石沉大海, 不排队任何应答
            return real_write(self, data)

        FakePort.responses = {"$B,PING": ["$B,PONG,1", "$B,OK"]}
        with mock.patch.object(FakePort, "write", drop_first_ping):
            self.assertTrue(self.link.connect("COM9", 115200))
        self.assertEqual(pings["n"], 2)         # 第二次就通了
        self.assertTrue(self.link.is_connected)

    def test_connect_err_answer_is_not_reported_as_silence(self):
        """设备回 $B,ERR,SYNC = 它活着且在说话（建链瞬间的字节错位, 固件的语义
        本来就是「请你重发」）—— 报错必须带上这条原文, 不能混成「没有应答」
        把人往「固件没跑/波特率不对」上带。"""
        FakePort.responses = {"$B,PING": ["$B,ERR,SYNC"]}
        with self.assertRaises(B.BtError) as cm:
            self.link.connect("COM9", 115200)
        msg = str(cm.exception)
        self.assertIn("$B,ERR,SYNC", msg)
        self.assertIn("重连", msg)
        self.assertNotIn("没有应答", msg)
        self.assertFalse(self.link.is_connected)        # 失败了仍要清理干净
        self.assertTrue(self.last_port().closed)

    def test_connect_true_silence_reports_after_retries_exhausted(self):
        """真静默（字节没到固件）也要把重试次数发满才报错, 且文案要能引导排查
        （固件/模块上电/模块↔主板串口）, 不再是误导人去查 PC 端波特率的旧话术。"""
        FakePort.responses = {"$B,PING": []}    # 命中键但不产出任何字节 = 真静默
        with mock.patch.object(B, "PING_FIRST_TIMEOUT", 0.05), \
             mock.patch.object(B, "PING_RETRY_TIMEOUT", 0.05):
            with self.assertRaises(B.BtError) as cm:
                self.link.connect("COM9", 115200)
        msg = str(cm.exception)
        self.assertIn("没有应答", msg)
        self.assertIn("115200", msg)
        self.assertEqual(len(self.last_port().written), B.PING_ATTEMPTS)

    def test_connect_open_failure_propagates(self):
        fake = mock.MagicMock()
        fake.open.side_effect = BS.SerialError("COM9 被占用")
        with mock.patch.object(BS, "SerialPort", return_value=fake):
            with self.assertRaises(BS.SerialError):
                self.link.connect("COM9", 115200)
        self.assertFalse(self.link.is_connected)

    def test_split_frames_reassemble(self):
        """chunk=7 强制把 ' $B,OK' 切成好几块 —— 行累积必须能拼回来。"""
        FakePort.responses = {"$B,PING": ["$B,PONG,1", "$B,OK"]}
        self.link.connect("COM9")
        self.assertEqual(self.last_port().chunk, 7)
        self.assertTrue(self.link.is_connected)

    def test_multiple_lines_one_chunk(self):
        FakePort.responses = {"$B,PING": ["$B,PONG,1", "$B,OK"]}
        self.link.connect("COM9")
        self.last_port().chunk = 4096
        FakePort.responses = {"$B,GET,x": ["$B,GET,x,1.00", "$B,LIST,0,0", "$B,OK"]}
        lines, status, _term = self.link.send_command("$B,GET,x", timeout=1.0)
        self.assertEqual(status, "ok")
        self.assertEqual(lines, ["$B,GET,x,1.00", "$B,LIST,0,0"])

    def test_crlf_and_bare_lf_both_work(self):
        FakePort.responses = {"$B,PING": ["$B,PONG,1", "$B,OK"]}
        self.link.connect("COM9")
        with self.link._log_lock:
            seq = self.link._rx_seq
        with self.link._log_lock:
            self.link._log.append({"dir": "rx", "text": "$B,OK", "seq": seq + 1, "t": time.time()})
        self.assertEqual(self.link._rx_since(seq), ["$B,OK"])


class TestLinkCommands(LinkCase):

    def setUp(self):
        LinkCase.setUp(self)
        FakePort.responses = {"$B,PING": ["$B,PONG,1", "$B,OK"]}
        self.link.connect("COM9")

    def test_timeout(self):
        """设备收到指令但一声不吭（掉线/卡住）→ timeout，而不是 err。"""
        FakePort.responses = {"$B,NOSUCH": []}      # 命中但不产出任何字节
        lines, status, term = self.link.send_command("$B,NOSUCH", timeout=0.25)
        self.assertEqual(status, "timeout")
        self.assertEqual(lines, [])
        self.assertEqual(term, "")

    def test_timeout_raises_in_command(self):
        FakePort.responses = {"$B,NOSUCH": []}
        with self.assertRaises(B.BtError):
            self.link._command("$B,NOSUCH", timeout=0.25)

    def test_error_terminator_is_surfaced_not_lost(self):
        """错误响应**没有正文行** —— 错误码只在终止符里, 不能去读 lines[-1]。"""
        FakePort.responses = {"$B,SET": ["$B,ERR,READONLY"]}
        with self.assertRaises(B.BtError) as cm:
            self.link.set_param("x", 1)
        self.assertIn("只读", str(cm.exception))

    def test_range_error(self):
        FakePort.responses = {"$B,SET": ["$B,ERR,RANGE,0.00,5.00"]}
        with self.assertRaises(B.BtError) as cm:
            self.link.set_param("x", 9)
        self.assertIn("5.00", str(cm.exception))

    def test_list_params(self):
        FakePort.responses = {"$B,LIST": [
            "$B,LIST,2,2",
            "$B,P,chassis_accel,底盘加速度,底盘,I32,1,1,0.00,2000.00,rpm/s,230",
            "$B,P,line_err,巡线偏差,巡线,F32,0,0,0.00,0.00,,0.12",
            "$B,J,0,s1,0.00,180.00,90.00",
            "$B,J,1,s2,0.00,180.00,45.00",
            "$B,OK"]}
        data = self.link.list_params()
        self.assertEqual(len(data["params"]), 2)
        self.assertEqual(len(data["joints"]), 2)
        self.assertEqual(data["params"][0]["name"], "chassis_accel")
        self.assertFalse(data["params"][1]["rw"])
        self.assertEqual(data["joints"][0]["index"], 0)

    def test_list_params_rejects_empty(self):
        FakePort.responses = {"$B,LIST": ["$B,LIST,0,0", "$B,OK"]}
        with self.assertRaises(B.BtError):
            self.link.list_params()

    def test_joints_sorted_by_index(self):
        FakePort.responses = {"$B,LIST": [
            "$B,LIST,1,2",
            "$B,P,x,名,组,F32,1,0,0.00,1.00,,0.0",
            "$B,J,1,s2,0.00,180.00,45.00",
            "$B,J,0,s1,0.00,180.00,90.00",
            "$B,OK"]}
        self.assertEqual([j["index"] for j in self.link.list_params()["joints"]], [0, 1])

    def test_get_pos(self):
        FakePort.responses = {"$B,POS": ["$B,POS,1.00,2.00,3.00,4.00,5.00,6.00,1", "$B,OK"]}
        pos = self.link.get_pos()
        self.assertEqual(pos["angles"], [1.0, 2.0, 3.0, 4.0, 5.0])
        self.assertEqual(pos["gripper"], 6.0)
        self.assertTrue(pos["done"])

    def test_jog_mode_grip_wire_format(self):
        FakePort.responses = {"$B,JOG": ["$B,JOG,2,135.0", "$B,OK"],
                              "$B,MODE": ["$B,MODE,1", "$B,OK"],
                              "$B,GRIP": ["$B,GRIP,1,100", "$B,OK"]}
        self.link.jog(2, 135.0, 250)
        self.link.set_mode(True)
        self.link.set_mode(False)
        self.link.grip(1, 100)
        self.assertEqual(self.last_port().written[-4:],
                         ["$B,JOG,2,135.0,250", "$B,MODE,1", "$B,MODE,0", "$B,GRIP,1,100"])

    def test_only_reader_thread_reads_the_port(self):
        """ReadFile 必须只有一个属主：send_command 只写口, 读一律归读线程。"""
        FakePort.responses = {"$B,GET,x": ["$B,GET,x,1.00", "$B,OK"]}
        FakePort.read_threads = []
        for _ in range(3):
            self.link.send_command("$B,GET,x", timeout=1.0)
        self.assertTrue(FakePort.read_threads)
        self.assertEqual(set(FakePort.read_threads), {"bt-reader"})

    def test_commands_are_serialised(self):
        """并发发令不能互相插队：每条指令都要拿到属于自己的应答。"""
        FakePort.responses = {"$B,GET": ["$B,GET,k,1.00", "$B,OK"]}
        results = []

        def worker(i):
            try:
                self.link.send_command("$B,GET,k", timeout=1.5)
                results.append(True)
            except Exception:                                # noqa: BLE001
                results.append(False)

        ts = [threading.Thread(target=worker, args=(i,)) for i in range(6)]
        for t in ts:
            t.start()
        for t in ts:
            t.join(timeout=5)
        self.assertEqual(len(results), 6)

    def test_command_without_connection_raises(self):
        link = B.BtLink()
        with self.assertRaises(B.BtError):
            link.send_command("$B,PING")


# =====================================================================
# 4. 串口枚举（不打真实 PowerShell / 注册表）
# =====================================================================

SPP_UUID = r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&0000\7&179D&0&0"


def _ports(pnp_ports, registry_ports=()):
    """跑一次 refresh_ports，PnP 与注册表两个来源都用假的顶替。"""
    with mock.patch.object(BS, "_friendly_cache", {}), \
         mock.patch.object(BS, "_friendly_cache_at", 0.0), \
         mock.patch.object(BS, "_query_pnp_ports", return_value=pnp_ports), \
         mock.patch.object(BS, "_enum_registry", return_value=list(registry_ports)):
        return BS.refresh_ports(force=True)


class TestPortEnum(unittest.TestCase):

    def test_bluetooth_spp_port_is_detected_from_pnp(self):
        """核心回归：蓝牙 SPP 口在链路未连接时**不在 SERIALCOMM 里**，
        只看注册表会一个都看不到（曾经 4 个蓝牙口全漏）。"""
        ports = _ports({"COM3": {"name": "Standard Serial over Bluetooth link",
                                 "pnp": SPP_UUID}})
        self.assertEqual(len(ports), 1)
        self.assertTrue(ports[0]["is_bt"])
        self.assertEqual(ports[0]["name"], "Standard Serial over Bluetooth link")
        self.assertFalse(ports[0]["ready"])          # 链路没连上，但照样列出来

    def test_ready_flag_tracks_registry(self):
        ports = _ports({"COM3": {"name": "Standard Serial over Bluetooth link", "pnp": SPP_UUID}},
                       [("COM3", r"\Device\BthModem0")])
        self.assertTrue(ports[0]["ready"])
        self.assertTrue(ports[0]["is_bt"])

    def test_plain_usb_port_is_not_bluetooth(self):
        ports = _ports({"COM8": {"name": "USB 串行设备",
                                 "pnp": r"USB\VID_FAED&PID_4870&MI_02\6&164FB5C&0&0002"}},
                       [("COM8", r"\Device\USBSER000")])
        self.assertFalse(ports[0]["is_bt"])
        self.assertTrue(ports[0]["ready"])

    def test_bluetooth_by_name_when_pnp_id_is_unhelpful(self):
        ports = _ports({"COM7": {"name": "Bluetooth 链接上的标准串行", "pnp": ""}})
        self.assertTrue(ports[0]["is_bt"])

    def test_bluetooth_by_device_path_fallback(self):
        """PnP 查不到时（拿不到注册表设备路径 BTPORT/BthModem）也不能漏。"""
        ports = _ports({}, [("COM7", r"\Device\BTPORT0")])
        self.assertTrue(ports[0]["is_bt"])

    def test_registry_only_port_is_still_listed(self):
        ports = _ports({}, [("COM9", r"\Device\USBSER001")])
        self.assertEqual([p["port"] for p in ports], ["COM9"])

    def test_sorted_numerically_not_lexically(self):
        ports = _ports({}, [("COM10", "d"), ("COM2", "d"), ("COM1", "d")])
        self.assertEqual([p["port"] for p in ports], ["COM1", "COM2", "COM10"])

    def test_empty_pnp_still_lists_registry_ports(self):
        ports = _ports({}, [("COM3", r"\Device\X")])
        self.assertEqual(len(ports), 1)
        self.assertEqual(ports[0]["name"], "")

    def test_pnp_name_drops_com_suffix(self):
        ports = _ports({"COM7": {"name": "Standard Serial over Bluetooth link", "pnp": SPP_UUID}})
        self.assertNotIn("COM7", ports[0]["name"])   # 端口号由界面单独拼


class TestPnpQueryParsing(unittest.TestCase):
    """PnP 查询输出两段：P<tab>端口 / D<tab>已配对蓝牙设备。"""

    OUTPUT = (
        "P\tStandard Serial over Bluetooth link (COM3)\t%s\n"
        "P\tStandard Serial over Bluetooth link (COM35)\t"
        "BTHENUM\\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&0002"
        "\\7&179D28B7&0&00260102C82C_C00000000\n"
        "P\tUSB 串行设备 (COM8)\tUSB\\VID_FAED&PID_4870\n"
        "P\t一些没有端口号的设备\tACPI\\X\n"
        "D\tQE_GKS\tBTHENUM\\DEV_00260102C82C\\7&2E39EDAD&0&BLUETOOTHDEVICE_00260102C82C\n"
        "D\tAda 的 Buds2\tBTHENUM\\DEV_405EF60D26BE\\7&2E39EDAD&0&BLUETOOTHDEVICE_405EF60D26BE\n"
    )

    def setUp(self):
        with mock.patch.object(BS, "_ps_stdout",
                               return_value=self.OUTPUT % SPP_UUID):
            self.out = BS._query_pnp_ports()

    def test_parses_ports(self):
        self.assertEqual(set(self.out), {"COM3", "COM8", "COM35"})
        self.assertEqual(self.out["COM3"]["name"], "Standard Serial over Bluetooth link")

    def test_maps_port_to_paired_device_name(self):
        """4 个蓝牙口里只有 1 个属于本车模块 —— 必须能说出是哪一个。"""
        self.assertEqual(self.out["COM35"]["device"], "QE_GKS")
        self.assertFalse(self.out["COM35"]["generic"])

    def test_generic_port_has_no_device(self):
        """设备地址全 0 = 不挂任何设备的通用传入口，永远连不上具体模块。"""
        self.assertTrue(self.out["COM3"]["generic"])
        self.assertEqual(self.out["COM3"]["device"], "")

    def test_non_bluetooth_port_is_not_generic(self):
        self.assertEqual(self.out["COM8"]["device"], "")
        self.assertFalse(self.out["COM8"]["generic"])

    def test_empty_on_failure(self):
        with mock.patch.object(BS, "_ps_stdout", return_value=""):
            self.assertEqual(BS._query_pnp_ports(), {})


class TestDirection(unittest.TestCase):
    """传入/传出判定 —— 判错会把用户带偏，所以依据必须是可验证的硬信号。

    ⚠️ 判据**改过一次**，别改回去：最初按 HardwareID 的 `_VID&…_PID&…`（当成传出）
    / `_LOCALMFG&…`（当成传入）猜，结果把本车模块的**传出口判成了传入口**。
    在 `蓝牙设置 → 更多蓝牙选项 → COM 端口` 的**方向**列上核对后确认：
      实例 ID 里**设备地址全 0** → 传入(通用监听口)
      实例 ID 里**带真实设备地址** → 传出(该设备的口)
    本机实测：COM35 = `LOCALMFG&0002` + `00260102C82C`(QE_GKS) → **传出**（正确）
              COM34 = `LOCALMFG&0000` + `000000000000`     → 传入
    """

    def _one(self, instance_id):
        with mock.patch.object(BS, "_ps_stdout", return_value=(
                "P\tStandard Serial over Bluetooth link (COM9)\t%s\n" % instance_id)), \
             mock.patch.object(BS, "_friendly_cache", {}), \
             mock.patch.object(BS, "_friendly_cache_at", 0.0):
            return BS._query_pnp_ports()["COM9"]

    def test_device_bound_port_is_outgoing(self):
        """带真实设备地址 = 该设备的口 = 传出（哪怕 HardwareID 是 _LOCALMFG&0002）。"""
        info = self._one(r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}"
                         r"_LOCALMFG&0002\7&179D28B7&0&00260102C82C_C00000000")
        self.assertEqual(info["direction"], "out")
        self.assertFalse(info["generic"])
        self.assertEqual(info["addr"], "00260102C82C")

    def test_remote_provided_port_is_outgoing(self):
        info = self._one(r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}"
                         r"_VID&00010075_PID&A013\7&179D28B7&0&405EF60D26BE_C00000000")
        self.assertEqual(info["direction"], "out")

    def test_all_zero_address_is_incoming_and_generic(self):
        """地址全 0 = 不挂任何设备的通用监听口，永远连不上具体模块。"""
        info = self._one(r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}"
                         r"_LOCALMFG&0000\7&179D28B7&0&000000000000_00000000")
        self.assertEqual(info["direction"], "in")
        self.assertTrue(info["generic"])

    def test_non_bluetooth_port_has_no_direction(self):
        info = self._one(r"USB\VID_FAED&PID_4870&MI_02\6&164FB5C&0&0002")
        self.assertEqual(info["direction"], "")

    def test_ports_are_not_misflagged_by_hardwareid(self):
        """回归：仅凭 _LOCALMFG&0002 判成传入是错的（老 bug），必须以地址为准。"""
        bound = self._one(r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}"
                          r"_LOCALMFG&0002\7&179D28B7&0&00260102C82C_C00000000")
        self.assertNotEqual(bound["direction"], "in")


class TestNotFoundHint(unittest.TestCase):
    """ERROR_FILE_NOT_FOUND 的提示必须能指出真正的原因，而不是误导成「没配对」。

    ⚠️ 这些用例要模拟一台**驱动正常**的机器：_any_bt_port_ready() 看到有蓝牙口进了
    SERIALCOMM，才会走「方向/服务」那几档提示；否则一律先说「驱动坏了」。"""

    CACHE = {
        # 通用传入口（地址全 0）—— 永远连不上具体模块
        "COM34": {"name": "Standard Serial over Bluetooth link", "pnp": SPP_UUID,
                  "device": "", "generic": True, "direction": "in"},
        # 本车模块的传出口
        "COM35": {"name": "Standard Serial over Bluetooth link", "pnp": SPP_UUID,
                  "device": "QE_GKS", "generic": False, "direction": "out"},
        # 手工「添加→传入」建出来的按设备传入口（方向以 Windows 对话框为准）
        "COM36": {"name": "Standard Serial over Bluetooth link", "pnp": SPP_UUID,
                  "device": "QE_GKS", "generic": False, "direction": "in"},
    }

    def setUp(self):
        p = mock.patch.object(BS, "_friendly_cache", self.CACHE)
        p.start()
        self.addCleanup(p.stop)
        # 模拟「蓝牙口已注册进 SERIALCOMM」= 驱动正常，走方向/服务那几档提示
        p = mock.patch.object(BS, "_enum_registry",
                              return_value=[("COM35", r"\Device\BthModem0")])
        p.start()
        self.addCleanup(p.stop)

    def test_systemic_driver_failure_is_diagnosed_first(self):
        """实测踩过：4 个蓝牙口 PnP 里「存在 + 状态 OK」，CreateFile 却一律
        ERROR_FILE_NOT_FOUND（连耳机的传出口都打不开），USB 串口正常 ——
        这是驱动绑错了 INF，不能再让用户去折腾方向/服务。"""
        with mock.patch.object(BS, "_enum_registry",
                               return_value=[("COM8", r"\Device\USBSER000")]):
            text = BS._not_found_hint("COM35")
        self.assertIn("驱动", text)
        self.assertIn("设备管理器", text)
        self.assertNotIn("传入", text)           # 别再往方向上带

    def test_healthy_machine_generic_port(self):
        text = BS._not_found_hint("COM34")
        self.assertIn("通用", text)
        self.assertIn("串行端口", text)          # 给出「怎么建传出端口」的做法

    def test_healthy_machine_incoming_port(self):
        text = BS._not_found_hint("COM36")
        self.assertIn("QE_GKS", text)            # 告诉用户这个口属于谁
        self.assertIn("传入", text)
        self.assertIn("传出", text)              # 并给出正确做法

    def test_healthy_machine_outgoing_port_blames_reachability(self):
        """传出口打不开就不是方向问题，别再让用户去勾服务 —— 是设备没开机。"""
        text = BS._not_found_hint("COM35")
        self.assertIn("QE_GKS", text)
        self.assertIn("没开机", text)
        self.assertNotIn("勾选", text)

    def test_non_bluetooth_port_stays_generic(self):
        with mock.patch.object(BS, "_friendly_cache",
                               {"COM8": {"name": "USB 串行设备",
                                         "pnp": r"USB\VID_FAED&PID_4870", "device": ""}}):
            text = BS._not_found_hint("COM8")
        self.assertIn("COM8", text)
        self.assertNotIn("蓝牙", text)           # 不是蓝牙口就别往蓝牙上扯

    def test_unknown_port_keeps_generic_wording(self):
        text = BS._not_found_hint("COM99")
        self.assertIn("COM99", text)
        self.assertNotIn("蓝牙", text)


class TestAnyBtPortReady(unittest.TestCase):

    def test_true_when_a_bt_modem_is_registered(self):
        with mock.patch.object(BS, "_enum_registry",
                               return_value=[("COM8", r"\Device\USBSER000"),
                                             ("COM35", r"\Device\BTPORT0")]):
            self.assertTrue(BS._any_bt_port_ready())

    def test_false_when_only_usb_serial(self):
        with mock.patch.object(BS, "_enum_registry",
                               return_value=[("COM8", r"\Device\USBSER000")]):
            self.assertFalse(BS._any_bt_port_ready())

    def test_registry_error_does_not_claim_driver_failure(self):
        """查不到就别乱下「驱动坏了」的结论。"""
        with mock.patch.object(BS, "_enum_registry", side_effect=OSError("nope")):
            self.assertTrue(BS._any_bt_port_ready())


class TestPsHelper(unittest.TestCase):

    def test_forces_utf8_pipe(self):
        """text=True 默认按系统码页解码（本机 GBK）会抛 UnicodeDecodeError 并被吞掉,
        表现成「设备名永远是空的」。必须显式指定 utf-8。"""
        with mock.patch.object(BS.subprocess, "run") as run:
            run.return_value = mock.Mock(stdout="COM7|名字\n")
            BS._ps_stdout("echo hi")
        kwargs = run.call_args.kwargs
        self.assertEqual(kwargs.get("encoding"), "utf-8")
        self.assertEqual(kwargs.get("errors"), "replace")
        cmd = run.call_args.args[0]
        self.assertIn("UTF8", " ".join(cmd))       # 让 PowerShell 也按 UTF-8 输出

    def test_failure_returns_empty(self):
        with mock.patch.object(BS.subprocess, "run", side_effect=OSError("nope")):
            self.assertEqual(BS._ps_stdout("x"), "")


# =====================================================================
# 5. 服务端动作分发
# =====================================================================

class TestServerActions(unittest.TestCase):

    def setUp(self):
        self.link = mock.Mock()
        self.link.list_params.return_value = {
            "params": [{"name": "a", "label": "A", "group": "G", "type": "F32",
                        "rw": True, "warn": False, "min": 0.0, "max": 1.0,
                        "unit": "", "value": "0.5"}],
            "joints": [{"index": 0, "name": "s1", "min": 0.0, "max": 180.0, "angle": 90.0}],
        }
        self.link.get_pos.return_value = {"angles": [1, 2, 3, 4, 5],
                                          "gripper": 45.0, "done": True}
        p = mock.patch.object(S.bt_link, "LINK", self.link)
        p.start()
        self.addCleanup(p.stop)

    def test_snippet_needs_no_connection(self):
        ok, log, data, err = S.run_bt_action("snippet", {"items": [
            {"name": "A", "angles": [1, 2, 3, 4, 5]}]})
        self.assertTrue(ok, err)
        self.assertIn("SCRIPT_STEP_F5", data["code"])

    def test_list_passes_device_data_through(self):
        ok, log, data, err = S.run_bt_action("list", {})
        self.assertTrue(ok, err)
        self.assertEqual(data["params"][0]["label"], "A")
        self.assertEqual(len(data["joints"]), 1)

    def test_connect_without_port_is_rejected(self):
        ok, log, data, err = S.run_bt_action("connect", {"port": ""})
        self.assertFalse(ok)
        self.assertIn("串口", err)
        self.link.connect.assert_not_called()

    def test_connect_passes_port_and_baud(self):
        ok, log, data, err = S.run_bt_action("connect", {"port": "COM7", "baud": 9600})
        self.assertTrue(ok, err)
        self.link.connect.assert_called_once_with("COM7", 9600)

    def test_bt_error_becomes_readable_failure(self):
        self.link.set_mode.side_effect = B.BtError("设备不在握手等待阶段")
        ok, log, data, err = S.run_bt_action("mode", {"on": True})
        self.assertFalse(ok)
        self.assertIn("握手", err)
        self.assertTrue(any(l["kind"] == "bad" for l in log))

    def test_serial_error_is_also_swallowed(self):
        self.link.jog.side_effect = BS.SerialError("COM7 被占用")
        ok, log, data, err = S.run_bt_action("jog", {"index": 0, "angle": 1})
        self.assertFalse(ok)
        self.assertIn("占用", err)

    def test_unexpected_exception_reported_not_raised(self):
        self.link.get_pos.side_effect = RuntimeError("boom")
        ok, log, data, err = S.run_bt_action("pos", {})
        self.assertFalse(ok)
        self.assertIn("boom", err)

    def test_unknown_action(self):
        ok, log, data, err = S.run_bt_action("nope", {})
        self.assertFalse(ok)
        self.assertIn("未知动作", err)

    def test_mode_has_no_home_flag_any_more(self):
        """「退出时回默认位姿」已去掉：请求里带 home 也不许传下去（固件那条路没了,
        传下去只会发出一个第三个字段被忽略的 $B,MODE,0,1）。"""
        self.link.set_mode.return_value = []
        S.run_bt_action("mode", {"on": False, "home": True})
        self.link.set_mode.assert_called_once_with(False)

    def test_refresh_returns_ports(self):
        with mock.patch.object(S.bt_serial, "refresh_ports",
                               return_value=[{"port": "COM7"}]) as rf:
            ok, log, data, err = S.run_bt_action("refresh", {})
        self.assertTrue(ok, err)
        self.assertEqual(data["ports"], [{"port": "COM7"}])
        rf.assert_called_once_with(force=True)

    def test_jog_ms_zero_reaches_the_device(self):
        """ms=0 是跟手模式的有意义取值, 绝不能被 `or 250` 之类的写法吞掉。

        吞掉的后果不是报错, 而是「拖滑杆还是不跟手」—— 现场根本查不出来。
        """
        self.link.jog.return_value = []
        S.run_bt_action("jog", {"index": 0, "angle": 90, "ms": 0})
        self.link.jog.assert_called_once_with(0, 90, 0)

    def test_jog_default_ms_when_absent(self):
        self.link.jog.return_value = []
        S.run_bt_action("jog", {"index": 0, "angle": 90})
        self.link.jog.assert_called_once_with(0, 90, 250)

    def test_arm_forwards_pose_and_gripper(self):
        self.link.arm_pose.return_value = []
        ok, log, data, err = S.run_bt_action("arm", {
            "angles": [90, 45, 90, 90, 180], "kai_he": 1, "degree": 100})
        self.assertTrue(ok, err)
        self.link.arm_pose.assert_called_once_with([90, 45, 90, 90, 180], 1, 100)

    def test_play_requires_an_id(self):
        ok, log, data, err = S.run_bt_action("play", {})
        self.assertFalse(ok)
        self.assertTrue(err)
        self.link.play_action.assert_not_called()

    def test_play_forwards_id(self):
        self.link.play_action.return_value = []
        ok, log, data, err = S.run_bt_action("play", {"id": 3})
        self.assertTrue(ok, err)
        self.link.play_action.assert_called_once_with(3)


class TestBtState(unittest.TestCase):

    def setUp(self):
        self.link = mock.Mock()
        self.link.state.return_value = {"connected": False, "port": "", "baud": 115200,
                                        "error": "", "log": []}
        p = mock.patch.object(S.bt_link, "LINK", self.link)
        p.start()
        self.addCleanup(p.stop)

    def _state(self, ports, prefs):
        with mock.patch.object(S.bt_serial, "refresh_ports", return_value=ports), \
             mock.patch.object(S.bt_link, "load_prefs", return_value=prefs):
            return S.bt_state()

    def test_last_port_auto_selected_when_present(self):
        st = self._state([{"port": "COM7"}, {"port": "COM8"}], {"port": "COM8", "baud": 9600})
        self.assertEqual(st["selected"], "COM8")
        self.assertEqual(st["baud"], 9600)

    def test_last_port_missing_is_not_selected(self):
        st = self._state([{"port": "COM7"}], {"port": "COM99"})
        self.assertEqual(st["selected"], "")

    def test_defaults(self):
        st = self._state([], {})
        self.assertEqual(st["baud"], 115200)
        self.assertEqual(st["selected"], "")
        self.assertIn(115200, st["baud_choices"])

    def test_port_enum_failure_does_not_break_state(self):
        with mock.patch.object(S.bt_serial, "refresh_ports",
                               side_effect=RuntimeError("boom")), \
             mock.patch.object(S.bt_link, "load_prefs", return_value={}):
            st = S.bt_state()
        self.assertEqual(st["ports"], [])


class TestFrontendContract(unittest.TestCase):
    """前端源码级约定。不是重实现，只是锁住踩过的坑。"""

    def setUp(self):
        self.js = (HERE / "static" / "app.js").read_text(encoding="utf-8")

    def test_h_coerces_non_node_children(self):
        """h() 必须把数字/布尔也转成文本节点。

        踩过：渲染波特率下拉时把 st.baud_choices 里的**数字**直接当子节点传给 h()，
        h() 只把 string 转成文本节点，于是 appendChild(9600) 抛
        "Failed to execute 'appendChild' on 'Node': parameter 1 is not of type 'Node'",
        整个蓝牙 Tab 一进去就报「读取蓝牙状态失败」。"""
        self.assertIn("instanceof Node", self.js,
                      "app.js::h() 丢了非 Node 子节点的转换守卫（数字子节点会抛 TypeError）")

    def test_baud_choices_are_numbers_from_the_server(self):
        """服务端确实下发数字数组 —— 就是上面那个坑的来源，别改成字符串后又不改前端。"""
        raw = (HERE / "bt_link.py").read_text(encoding="utf-8")
        self.assertIn("BAUD_CHOICES = (9600,", raw)
        for b in B.BAUD_CHOICES:
            self.assertIsInstance(b, int)


class TestFrontendWiring(unittest.TestCase):
    """前端源码级契约（只读 static/ 下的文件, 不启服务）。

    这里查的都是「服务端测试与 node --check 都看不见、只在浏览器里炸」的那类问题。
    """

    def test_every_getElementById_literal_exists_in_the_html(self):
        """app.js 里写死的元素 id 必须真在 index.html 里。

        这类拼写错**只会在浏览器里炸**（null.addEventListener），node --check 与所有
        服务端测试都看不见 —— 前端改动一大就容易漏。动态生成的 id（blk-*）不在检查范围。
        """
        js = srccheck.read_source("static/app.js")
        html = srccheck.read_source("static/index.html")
        html_ids = set(re.findall(r'id="([^"]+)"', html))
        used = {u for u in re.findall(r"getElementById\('([^']+)'\)", js)
                if not u.startswith("blk-")}
        self.assertTrue(used, "没从 app.js 里抠到任何 getElementById 字面量")
        self.assertEqual(sorted(u for u in used if u not in html_ids), [])

    def test_arm_tab_is_wired(self):
        """机械臂 Tab 的四件套: TABS 条目 / pane DOM / CSS 覆盖 / 懒加载。

        ⚠️ 少了 CSS 那条覆盖, 页面会继承「地图」的三栏网格 —— 静默错版,
            在宽屏上看着「也还行」, 所以极易漏。
        """
        js = srccheck.read_source("static/app.js")
        html = srccheck.read_source("static/index.html")
        css = srccheck.read_source("static/style.css")
        self.assertRegex(js, r"id:\s*'arm'[^}]*label:\s*'机械臂'")
        self.assertIn('id="pane-arm"', html)
        self.assertIn("#pane-arm.active", css)
        self.assertIn("if (id === 'arm' && !ARM.loaded) ARM.loading = loadArm();", js)

    def test_arm_palette_entries_land_on_the_real_step(self):
        """积木库里的「机械臂动作」大积木落地时必须用 synth_step, 不能用合成键。"""
        js = srccheck.read_source("static/app.js")
        self.assertIn("step: cat.synth_step || cat.type", js)
        self.assertIn("params: cat.synth_params || defaults", js)


class TestOriginGuard(unittest.TestCase):
    """蓝牙端会驱动真实硬件，必须和 /api/tools 一样拒绝跨站 POST。

    ⚠️ 判据不是一串写死的 localhost: 服务端绑 0.0.0.0, 队友用局域网地址打开时
       Origin/Host 都是那个地址 —— 写死 localhost 会让队友**一保存就 403**,
       而页面看着完全正常（最难查的那种）。真正的判据在 server.origin_ok。
    """

    def _origin_ok(self, origin, host="127.0.0.1:8410"):
        handler = S.Handler.__new__(S.Handler)

        class Hdrs(dict):
            def get(self, k, d=None):
                return dict.get(self, k, d)
        hdrs = {"Origin": origin} if origin else {}
        if host is not None:
            hdrs["Host"] = host
        handler.headers = Hdrs(hdrs)
        return handler._origin_ok()

    def test_same_origin_allowed(self):
        self.assertTrue(self._origin_ok("http://127.0.0.1:8410"))
        self.assertTrue(self._origin_ok("http://localhost:8410", "localhost:8410"))
        self.assertTrue(self._origin_ok("http://[::1]:8410", "[::1]:8410"))

    def test_lan_origin_allowed(self):
        """绑定 0.0.0.0 之后队友就是这么访问的 —— 必须放行, 否则保存全 403。

        允许集合随机器而变, 所以直接打桩本机地址, 不依赖跑测试这台机器的网卡。
        """
        with mock.patch.object(S, "local_host_names",
                               return_value={"127.0.0.1", "localhost",
                                             "gks-pc", "192.0.2.20"}):
            self.assertTrue(self._origin_ok("http://192.0.2.20:8410",
                                            "192.0.2.20:8410"))
            self.assertTrue(self._origin_ok("http://gks-pc:8410", "gks-pc:8410"))
            # 别人机器的 IP 不在本机地址集合里 → 拒
            self.assertFalse(self._origin_ok("http://192.0.2.99:8410",
                                             "192.0.2.99:8410"))

    def test_foreign_origin_rejected(self):
        self.assertFalse(self._origin_ok("http://evil.example.com",
                                         "evil.example.com"))
        # 带端口的同样拒 —— 别放宽成「端口对上就行」
        self.assertFalse(self._origin_ok("http://evil.example.com:8410",
                                         "evil.example.com:8410"))

    def test_rejects_dns_rebinding_and_bad_ports(self):
        """"本机地址 + Host 一致"两道都要过。

        DNS rebinding: 攻击者把自己的域名解析到本机 IP, 浏览器就会带着
        `Origin: http://evil.com:8410` 访问 —— 域名不在本机地址集合里, 拒。
        """
        self.assertFalse(self._origin_ok("http://192.0.2.20:8410",
                                         "evil.example.com:8410"))
        # 端口必须就是本控制台那个: 本机其它服务上的页面不能当跳板
        self.assertFalse(self._origin_ok("http://192.0.2.20:9999",
                                         "192.0.2.20:9999"))
        # 沙箱 iframe / file:// 发出来的 Origin 是字符串 "null"（不是没有 Origin）
        self.assertFalse(self._origin_ok("null", "127.0.0.1:8410"))
        # HTTP/1.1 起 Host 就是必带的; 缺了说明不是浏览器在同源访问 → 拒
        self.assertFalse(self._origin_ok("http://127.0.0.1:8410", None))

    def test_no_origin_allowed(self):
        self.assertTrue(self._origin_ok(None))

    def test_renderJoints_tolerates_a_missing_container(self):
        """关节滑杆只在「机械臂」Tab 里有容器, 但 renderJoints 的调用点是不分区格的
        （loadBt / linkDisconnect / onConnected / btReadParams 的回退）。

        少了空判就会在「蓝牙」Tab 上抛 TypeError, 而且**连带滑杆永远轮不到渲染** ——
        症状是「机械臂 Tab 里一个关节滑杆都没有」。真踩过。
        """
        js = srccheck.read_source("static/app.js")
        self.assertIn("function jointBox()", js)
        m = re.search(r"function renderJoints\(\)[\s\S]*?\n\}\n", js)
        self.assertIsNotNone(m, "没找到 renderJoints 的函数体")
        body = m.group(0)
        self.assertIn("if (!box) return;", body)
        # 空判必须在**用到 box 之前**（按函数体匹配, 不按「开头往后 N 个字符」——
        # 窗口是脆的, 往 renderJoints 开头加几行就会假报失败）
        self.assertLess(body.index("if (!box) return;"), body.index("box.textContent"),
                        "renderJoints 里用了 box 却没先判空")

    def test_mod_marker_updates_without_rebuilding_the_dom(self):
        """拖滑杆 / 改延时**不能**重建画布（会把正在拖的那个滑块弄丢, 拖动直接断掉）,
        所以「改过」的黄标与那块 ↺ 必须由 armRefreshBlock → armRefreshModified 就地刷。

        踩过: 只在 armRenderBlock 里挂类名 → 拖完滑杆块永远不变黄（用户报的
        「修改了但是没有变黄」）。"""
        js = srccheck.read_source("static/app.js")
        m = re.search(r"function armRefreshBlock\(id\)[\s\S]*?\n\}\n", js)
        self.assertIsNotNone(m, "没找到 armRefreshBlock 的函数体")
        body = m.group(0)
        self.assertIn("armRefreshModified(it);", body)
        # 而且要在**延时/只读块的早退之前**: 那两种块没有 .b-body 摘要要更新,
        # 但它们照样会变黄、照样该出现 ↺
        self.assertLess(body.index("armRefreshModified(it);"),
                        body.index("it.kind === 'delay'"),
                        "armRefreshModified 排在早退之后, 延时块就刷不到黄标")

    def test_arm_mod_marker_is_styled(self):
        """「改过（还没保存）」的积木靠 `.armmod` 把左边框染黄 —— 类名写在 app.js、
        样式写在 style.css, 两边拼错一个是**静默失效**（块还是那块块, 就是不黄）。"""
        js = srccheck.read_source("static/app.js")
        css = srccheck.read_source("static/style.css")
        self.assertIn("' armmod'", js, "armRenderBlock 没挂上 armmod 类")
        self.assertIn("block.armmod", css, "style.css 里没有 .armmod 的样式")

    def test_arm_tab_renders_joints_when_entered(self):
        """进「机械臂」Tab（首入 + 回来）都必须让滑杆有内容 ——
        否则从「蓝牙」Tab 连好再切过来会看到空面板。"""
        js = srccheck.read_source("static/app.js")
        # 首入: loadArm → armReload（占位表是它带回来的, 所以 renderJoints 必须在它里面,
        #       不能在 loadArm 里提前调 —— 那样会先画一句提示再被替换掉, 闪一下）。
        #       2026-10-31 起 loadArm 是同步函数 + 内部 async IIFE（共享 promise）,
        #       按函数体匹配（同下面 armReload 那条的理由）
        m = re.search(r"function loadArm\(\)[\s\S]*?\n\}\n", js)
        self.assertIsNotNone(m, "没找到 loadArm 的函数体")
        self.assertIn("await armReload();", m.group(0))
        # ⚠️ 按**函数体**匹配, 不按「开头往后 N 个字符」—— 窗口是脆的: 往 armReload 里
        #    加几行注释就会假报失败（真踩过）, 而它其实只关心「在不在这个函数里」。
        m = re.search(r"async function armReload\(\)[\s\S]*?\n\}\n", js)
        self.assertIsNotNone(m, "没找到 armReload 的函数体")
        self.assertIn("renderJoints();", m.group(0),
                      "armReload 里必须调 renderJoints —— 灰色占位滑杆是它带回来的")
        # 回来时
        self.assertIn("else if (id === 'arm') { armRenderAll(); renderJoints(); }", js)
        # 进编辑模式顺手拉一次参数表（滑杆的范围来自设备上报）。
        # ⚠️ 按**函数体**匹配（同上面 armReload 那条的理由）—— 2026-10 btEnterEdit
        #    加了「握手让出后的静默重试」, btReadParams 离函数头远了, 字符窗口会假报。
        m = re.search(r"async function btEnterEdit\([^)]*\)[\s\S]*?\n\}\n", js)
        self.assertIsNotNone(m, "没找到 btEnterEdit 的函数体")
        self.assertIn("btReadParams(true);", m.group(0),
                      "btEnterEdit 里必须拉一次参数表 —— 滑杆范围来自设备上报")

    def test_gripper_angle_round_trips_through_kai_he_degree(self):
        """「获取当前位姿」把夹爪**角度**反解成控件用的（开合, 程度%），必须能原样回代。

        固件那条映射是多对一的（app_arm.c: openPct = kai ? degree : 100 - degree;
        angle = CLOSE + (OPEN-CLOSE)·openPct/100），所以反解得按开度选分支。这里把**固件
        公式原样搬进 probe** 逐度验证：往返误差必须落在量化步长（全量程/100）的一半以内
        —— 选错分支、正负号写反、或忘了 clamp 都会立刻爆出来。
        """
        if not srccheck.node_available():
            self.skipTest("没装 node")
        rc, out = srccheck.run_js_top_level(
            srccheck.read_source("static/app.js"),
            "var span = {open: 0, close: 180};"
            "var worst = 0, bad = 0;"
            "for (var a = 0; a <= 180; a += 1) {"
            "  var g = armGripperFromAngle(a, span);"
            "  var openPct = g.kai_he ? g.degree : (100 - g.degree);"
            "  var back = span.close + (span.open - span.close) * openPct / 100;"
            "  if (g.degree < 0 || g.degree > 100) bad++;"
            "  worst = Math.max(worst, Math.abs(back - a));"
            "}"
            "console.log('WORST=' + worst.toFixed(3) + ' BAD=' + bad);"
            "console.log('MID=' + JSON.stringify(armGripperFromAngle(90, span)));"
            "console.log('OPENEND=' + JSON.stringify(armGripperFromAngle(0, span)));"
            "console.log('CLOSEEND=' + JSON.stringify(armGripperFromAngle(180, span)));")
        self.assertEqual(rc, 0, out)
        self.assertIn("BAD=0", out)
        m = re.search(r"WORST=([\d.]+)", out)
        self.assertIsNotNone(m, out)
        self.assertLessEqual(float(m.group(1)), 0.9 + 1e-6, out)   # 半个量化步长
        # 分支选择: 半程(90°)归「张开」区间, 两端各自落在正确的分支上
        self.assertIn('MID={"kai_he":1,"degree":50}', out)
        self.assertIn('OPENEND={"kai_he":1,"degree":100}', out)
        self.assertIn('CLOSEEND={"kai_he":0,"degree":100}', out)

    def test_get_pose_button_is_wired_and_gated_by_edit_mode(self):
        """「获取当前位姿」按钮: 必须真挂在 DOM 上、且跟着编辑模式启停。

        $B,POS 在固件里有编辑模式门禁（app_bt.c::bt_cmd_pos 开头 bt_edit_active 检查），
        不在编辑模式只回 NOTEDIT —— 所以按钮不跟着门禁走就会变成一个必然报错的按钮。
        """
        js = srccheck.read_source("static/app.js")
        html = srccheck.read_source("static/index.html")
        self.assertIn('id="arm-get-pose"', html)
        self.assertIn("document.getElementById('arm-get-pose')", js)
        self.assertRegex(js, r"const poseBtn = document\.getElementById\('arm-get-pose'\);\s*"
                             r"if \(poseBtn\) poseBtn\.disabled = !BT\.edit;")

    def test_joints_panel_follows_the_selection(self):
        """「关节」面板有两种模式，由「有没有选中一个位姿块」决定：

          选中位姿块 → 滑杆显示**那个块存的角度**，拖动即改积木（离线也能改）
          没选中     → 实时设备值（拖动 = 让机械臂动）/ 离线灰色占位

        这条规则让「调一条已有动作」与「录一条新动作」共用同一组滑杆。
        """
        js = srccheck.read_source("static/app.js")
        self.assertIn("function armSelectedItem()", js)
        self.assertIn("function armPoseTarget()", js)
        self.assertRegex(js, r"const tgt = armPoseTarget\(\);")
        self.assertIn("tgt.angles[j.index] = v;", js)
        self.assertIn("tgt.edited = true;", js)
        # 面板状态写在卡片标题右边，不藏
        self.assertIn("'编辑「' + (tgt.tag || '位姿') + '」的角度'", js)

    def test_editing_a_block_does_not_rebuild_the_canvas_mid_drag(self):
        """拖动滑杆改积木时只能就地更新那一行的摘要。

        ⚠️ 若在这里调 armRenderCanvas(), 正在拖的滑块会被重建掉（指针还按着、元素
        已不在文档里）→ 拖动直接断掉。这是个很容易「顺手优化」回去的坑。
        """
        js = srccheck.read_source("static/app.js")
        self.assertIn("function armRefreshBlock(id)", js)
        self.assertIn("body.textContent = (it.kind === 'pose') ? armPoseSummary(it) : armGripSummary(it);", js)
        self.assertIn("armRefreshBlock(tgt.id);", js)
        # 一次拖动只压一次撤回快照（每个 input 都压会把撤回栈灌满中间态）
        self.assertRegex(js, r"if \(tgt\) armPushUndo\(\);")

    def test_clicking_blank_canvas_deselects(self):
        js = srccheck.read_source("static/app.js")
        self.assertIn("function armDeselect()", js)
        self.assertRegex(js, r"getElementById\('arm-canvas'\)\.addEventListener\('click'")
        self.assertIn("if (e.target.closest('.block')) return;", js)
        # 选中项消失（切预设/重载/被删）要一并清掉选中态
        self.assertIn("if (ARM.selectedId && !ARM.items.some(i => i.id === ARM.selectedId)) ARM.selectedId = null;", js)

    def test_arm_commands_are_only_sent_in_edit_mode(self):
        """不在编辑模式就别发臂指令 —— 设备只会回 NOTEDIT，白白占着链路。"""
        js = srccheck.read_source("static/app.js")
        self.assertRegex(js, r"if \(!BT\.edit\) return;      /\* 设备只在编辑模式下收臂指令")

    def test_arm_canvas_normalises_item_ids(self):
        """子积木的唯一 id 是选中/拖拽/删除三件事的共同判据。

        服务端已保证（见 test_arm_action 的 id 测试），前端再兜一次并留痕 ——
        缺 id 的后果（点一块选中全部 / 拖动不让位 / 删除永远删第一块）看着毫不相干，
        极难反查到「少了个 id」。
        """
        js = srccheck.read_source("static/app.js")
        self.assertIn("if (!it.id || seen.has(it.id)) { it.id = armNewId(); patched++; }", js)
        self.assertIn("' 条子积木的 id 缺失或重复, 已临时补上（服务端 bug?）'", js)

    def test_offline_joint_sliders_are_drawn_but_disabled(self):
        """没连设备时滑杆要**画出来但灰掉**, 不是空面板。

        空面板会让人以为「这功能没做」（真反馈过）；占位表由服务端从源码派生
        （`joints_offline`），前端不写死任何量程。
        """
        js = srccheck.read_source("static/app.js")
        self.assertIn("ARM.jointsOffline", js)
        self.assertIn("const live = BT.joints.length > 0;", js)
        self.assertRegex(js, r"const ranges = live \? BT\.joints : \(ARM\.jointsOffline \|\| \[\]\);")
        # 可拖的条件是「选中了位姿块」**或**「设备在线」—— 只看 live 会把「未连设备但
        # 想改一条已有动作的角度」这条很常用的路径堵死。
        # ⚠️ 但**多选**（Ctrl+点击累加）时必须一律灰掉: 一组积木没有「它存的角度」,
        #    退化成实时设备值更糟 —— 拖一下会把整组里唯一能改的那块悄悄改掉。
        self.assertIn("const editable = !multi && (!!tgt || live);", js)
        self.assertIn("const multi = armMultiSel();", js)
        self.assertIn("disabled: editable ? null : ''", js)
        self.assertIn("'bt-joint' + (editable ? '' : ' off')", js)
        # 灰态下不能挂监听（挂了就是"看着能拖其实没反应"）
        self.assertRegex(js, r"if \(editable\) \{\s*rng\.addEventListener\('pointerdown'")
        css = srccheck.read_source("static/style.css")
        self.assertIn(".bt-joint.off", css)
        self.assertIn(".bt-joint input[type=range]:disabled", css)

    def test_arm_panel_has_a_joints_box(self):
        """容器得有 `.bt-joints-box` 这个 class —— jointBox() 就是按它找的。"""
        html = srccheck.read_source("static/index.html")
        self.assertRegex(html, r'id="arm-joints"[^>]*class="[^"]*bt-joints-box')

    def test_app_js_loads_without_throwing(self):
        """app.js 的**顶层**在最小 DOM 桩下必须不抛异常。

        抓的是「页面一加载就炸」的引用错误（写错的函数名、TDZ 的 const…）。这类错
        node --check 只看语法看不见, 所有服务端测试更看不见 —— 只有打开页面才发现。
        """
        if not srccheck.node_available():
            self.skipTest("没装 node")
        rc, out = srccheck.run_js_top_level(
            srccheck.read_source("static/app.js"),
            "console.log('TABS=' + TABS.map(t => t.id).join(','));"
            "console.log('DND=' + (typeof MDND) + '/' + (typeof ARMDND));")
        self.assertEqual(rc, 0, out)
        self.assertIn("TABS=", out)
        self.assertIn("arm", out)
        self.assertIn("DND=object/object", out)

    def test_protected_paths_are_really_listed_in_server(self):
        """受保护的路径必须**真在 server.py 的元组里** —— 别在这里再抄一份。

        ⚠️ 原来这里硬编码了元组文本, 加 /api/capture 时就忘了同步, 于是通过了却
        什么都没保证（假的安全感）。现在改成去读源码里真正的那个元组。
        """
        _, origin = srccheck.guard_tuples(srccheck.read_source("server.py"))
        self.assertTrue(origin, "没从 server.py 抠到 Origin 前缀元组")
        for path in ("/api/tools", "/api/tuning", "/api/bt", "/api/capture", "/api/arm"):
            self.assertIn(path, origin, "%s 没被 Origin 校验覆盖" % path)


class TestArmMultiSelect(unittest.TestCase):
    """机械臂 Tab: 多选积木 / 整组拖动 / 复制移动到别的动作预设。

    这里既有源码契约（「漏改就静默失效」的那几处）, 也有真在 node 里**跑一遍**的
    行为测试（搬运与落位规则是纯数据操作, 值得逐条验证, 不该只查子串）。
    """

    def setUp(self):
        self.js = srccheck.read_source("static/app.js")
        self.css = srccheck.read_source("static/style.css")

    # ---- 源码契约 ----

    def test_record_pose_lands_below_the_selected_block(self):
        """「＋ 记录当前位姿」要插在**选中块的下方**, 不是一律追加到末尾。

        原来一律 push: 想在中间补一步, 加完还得手动把它拖上去。
        """
        self.assertIn("function armInsertIdx()", self.js)
        self.assertIn("function armInsertItem(it)", self.js)
        # 落点 = 选中组的最后一块之后; 没选中才是末尾
        m = re.search(r"function armInsertIdx\(\)[\s\S]*?\n\}\n", self.js)
        self.assertIsNotNone(m, "没找到 armInsertIdx 的函数体")
        self.assertIn("return last < 0 ? ARM.items.length : last + 1;", m.group(0))
        # 记录位姿 / 加延时都走同一条落位路径（分成两套迟早漂开）
        for fn in ("armRecord", "armAddDelay"):
            m = re.search(r"function %s\(\)[\s\S]*?\n\}\n" % fn, self.js)
            self.assertIsNotNone(m, "没找到 %s 的函数体" % fn)
            self.assertIn("armInsertItem(", m.group(0),
                          "%s 没走 armInsertItem（落点会与「记录当前位姿」不一致）" % fn)

    def test_ctrl_click_accumulates_and_right_click_opens_the_menu(self):
        """Ctrl+点击累加多选; 右键一块不在选中组里的积木 → 先选中它再弹菜单。"""
        self.assertIn("function armToggleSelect(id)", self.js)
        m = re.search(r"onclick: \(e\) => \{\s*if \(e\.target\.closest\('input,select,button'\)\) "
                      r"return;\s*if \(ARMDND\.justDragged\(\)\) return;[\s\S]*?\n    \},", self.js)
        self.assertIsNotNone(m, "没找到机械臂积木的 onclick")
        self.assertIn("if (e.ctrlKey || e.metaKey) armToggleSelect(it.id);", m.group(0))
        self.assertIn("else armSelect(it.id);", m.group(0))
        self.assertRegex(self.js, r"oncontextmenu: \(e\) => \{\s*"
                                  r"if \(e\.target\.closest\('input,select,button'\)\) return;\s*"
                                  r"e\.preventDefault\(\);\s*"
                                  r"if \(!ARM\.selIds\.includes\(it\.id\)\) armSelect\(it\.id\);"
                                  r"\s*openArmCtx\(e\.clientX, e\.clientY\);")

    def test_ctx_menu_dom_is_built_in_js_not_in_the_html(self):
        """菜单是动态的, DOM 由 JS 现搭 —— 别为了拿它去 index.html 塞静态节点。

        （test_every_getElementById_literal_exists_in_the_html 那张契约网正是拦这个的:
        菜单不该有 id。）"""
        self.assertNotRegex(self.js, r"getElementById\('arm-ctx")
        self.assertIn("document.body.appendChild(el);", self.js)
        self.assertIn(".ctxmenu", self.css)
        # Esc / 点菜单外都要能关掉
        self.assertIn("if (armCtxEl) { closeArmCtx(); return; }", self.js)
        self.assertRegex(self.js, r"if \(armCtxEl && !armCtxEl\.contains\(e\.target\)\) closeArmCtx\(\);")

    def test_scrolling_inside_the_menu_does_not_close_it(self):
        """预设多到出滚动条时, 在菜单里滚轮不能把菜单关掉。

        scroll 事件**不冒泡**, 但**走捕获阶段** —— 所以 init 里那条
        `window.addEventListener('scroll', …, true)` 同样收得到 `.ctxmenu` 自己的滚动
        （它有 `max-height:70vh; overflow-y:auto`, 预设一多就出条）。老代码直接挂
        `closeArmCtx`, 于是「滚一下『复制/移动到…』的目标列表 → 菜单当场消失」,
        而那个列表正是要用滚轮翻的东西。"""
        # 页面滚动照样关（这条别一起修没了）
        self.assertIn("window.addEventListener('scroll', armCtxOnScroll, true);", self.js)
        # 滚到列表尽头之后的滚动链别再交给画布（否则页面一滚, 菜单还是没）
        self.assertIn("overscroll-behavior: contain;", self.css)

    def test_group_drag_is_wired_into_the_shared_dnd(self):
        """整组拖动走的是**共用的** makeDnd（别再抄一份机械臂专用拖拽）。

        必须有的四处: 落点跳过整组 / 幽灵与占位槽按整组高度 / 整组落座后保持整组选中 /
        整组拖进垃圾桶是**一次删光**（只删第一块会让人以为丢了数据）。"""
        self.assertIn("dragSet: armDragSet,", self.js)
        self.assertIn("selectMany: armSetSelection,", self.js)
        self.assertIn("deleteItems: armDeleteItems,", self.js)
        self.assertIn("function dragSkipIds(payload)", self.js)
        self.assertIn("if (skipIds && skipIds.has(id)) continue;", self.js)
        self.assertIn("function groupHeight(els)", self.js)
        self.assertIn("function createGroupGhost(els)", self.js)
        self.assertIn("if (ids.length > 1 && cfg.deleteItems) cfg.deleteItems(ids);", self.js)
        # 幽灵容器也得借 .block.drag-ghost 的定位/阴影（样式见 style.css）
        self.assertIn(".block.drag-ghost.drag-group", self.css)
        self.assertIn(".drag-group > .block:first-child { margin-top: 0; }", self.css)

    def test_multi_selection_greys_the_joint_sliders(self):
        """多选时滑杆谁都不属于 → 一律灰色（连"实时设备值"也不给）。"""
        self.assertIn("function armMultiSel() { return ARM.selIds.length > 1; }", self.js)
        # 单选/多选的分界就在 armSelectedItem: 只有**恰好一块**时才有"它存的角度"
        m = re.search(r"function armSelectedItem\(\)[\s\S]*?\n\}\n", self.js)
        self.assertIsNotNone(m, "没找到 armSelectedItem 的函数体")
        self.assertIn("if (ARM.selIds.length !== 1) return null;", m.group(0))
        self.assertIn("const gripEditable = !multi && (!!gt || live);", self.js)

    def test_selection_is_cleared_when_the_preset_changes(self):
        """换动作/重载/保存后选中必须显式归零。

        ⚠️ 两条预设的子积木 id **会重号**（都是从 map.c 解析出来的 i0/i1…）,
        只靠「id 不在 items 里就清掉」挡不住 —— 切过去之后新动作里会有一块
        "莫名其妙已经被选中", 这时拖滑杆改的就是它。"""
        self.assertIn("function armClearSelection()", self.js)
        n = self.js.count("armClearSelection();")
        self.assertGreaterEqual(n, 8, "选中归零的点位少了（切预设/新建/复制/删除/重载/保存/跳转/跨预设搬运）")
        # 关掉选中态的那句兜底也要还在（id 真的不在了的那种）
        self.assertIn("if (ARM.selectedId && !ARM.items.some(i => i.id === ARM.selectedId)) ARM.selectedId = null;", self.js)

    # ---- 行为（真在 node 里跑） ----

    def _run(self, probe):
        if not srccheck.node_available():
            self.skipTest("没装 node")
        rc, out = srccheck.run_js_top_level(srccheck.read_source("static/app.js"), probe)
        self.assertEqual(rc, 0, out)
        return out

    def test_scroll_close_lets_the_menu_scroll_itself(self):
        """行为: 滚菜单内部 → 菜单还在; 滚页面 → 关掉。"""
        out = self._run(
            "var inside = {id: 'inside'}, page = {id: 'page'};"
            "armCtxEl = { contains: (t) => t === inside };"
            "armCtxOnScroll({ target: inside });"
            "console.log('KEPT=' + (armCtxEl !== null));"
            "armCtxOnScroll({ target: page });"
            "console.log('CLOSED=' + (armCtxEl === null));")
        self.assertIn("KEPT=true", out)
        self.assertIn("CLOSED=true", out)

    def test_insert_index_follows_the_selection(self):
        """落点: 单选/多选/没选中 三种情况。"""
        out = self._run(
            "ARM.items = [{id:'x1'},{id:'x2'},{id:'x3'}];"
            "ARM.selIds = ['x2'];   console.log('ONE=' + armInsertIdx());"
            "ARM.selIds = ['x1','x3']; console.log('MANY=' + armInsertIdx());"
            "ARM.selIds = ['x3','x1']; console.log('ORDER=' + armInsertIdx());"
            "ARM.selIds = [];       console.log('NONE=' + armInsertIdx());")
        self.assertIn("ONE=2", out)
        self.assertIn("MANY=3", out)      # 组里最后一块（x3）的下面
        self.assertIn("ORDER=3", out)     # 与拾取顺序无关, 看的是在数组里的位置
        self.assertIn("NONE=3", out)

    def test_inserted_block_gets_its_id_before_it_is_selected(self):
        """插入的块必须**先拿到 id 再设选中**。

        ⚠️ 真踩过: `armPoseItem()` 和「＋ 加延时」造出来的块都**不带 id** —— id 一直是
           `armRenderCanvas` 那条兜底补的, 而兜底在 `armInsertItem` **之后**才跑,
           补出来的 id 落不进已经写死的 selIds, 于是留下 `selIds = [undefined]`:
           新积木根本没被选中（画布上不亮; 滑杆退化成「实时设备值」而不是
           「编辑「位姿N」的角度」）, 而且每次都打一条
           「id 缺失或重复…（服务端 bug?）」的**假警告** —— 明明是本地的块。
        """
        out = self._run(
            "toast = () => {}; btlog = () => {};"
            "armRenderCanvas = () => {}; armRenderBar = () => {};"
            "armMarkDirty = () => {}; armPushUndo = () => {};"
            "renderJoints = () => {};"
            "ARM.presets = [{id:1, title:'A', items:[]}]; ARM.activeIdx = 0;"
            "armSyncItems(); ARM.seq = 7;"
            "armAddDelay();"
            "console.log('ID=' + ARM.items[0].id);"
            "console.log('SEL=' + ARM.selIds.join(',') + '|' + ARM.selectedId);"
            "armRecord();"
            "console.log('SEL2=' + ARM.selIds.join(',') + '|' + ARM.selectedId);")
        self.assertIn("ID=a7", out)
        self.assertIn("SEL=a7|a7", out)
        # 「记录当前位姿」那条也必须自足地拿到 id, 不能依赖 armRenderCanvas 的兜底
        self.assertIn("SEL2=a8|a8", out)

    def test_transfer_to_another_preset_appends_with_fresh_ids(self):
        """「移动到别的动作」: 源里摘掉、目标追加, 而且**搬过去的条目一律换新 id**。

        ⚠️ 两条预设的 id 重号是常态（都是 i0/i1…）—— 原样搬过去就是「一个预设里两块
        同 id」, 症状是点一块选中全部 / 拖动不让位 / 删除永远删第一块。这条测试就是
        拿重号的场景建的。
        """
        out = self._run(
            "toast = () => {}; btlog = () => {};"
            "armRenderCanvas = () => {}; armRenderAll = () => {};"
            "armRenderBar = () => {}; renderJoints = () => {};"
            "ARM.presets = ["
            "  {id:1,title:'A',items:[{id:'i0',kind:'pose',tag:'p1',angles:[1,1,1,1,1]},"
            "                         {id:'i1',kind:'delay',tag:'d',ms:100},"
            "                         {id:'i2',kind:'pose',tag:'p3',angles:[3,3,3,3,3]}]},"
            "  {id:2,title:'B',items:[{id:'i0',kind:'pose',tag:'b1',angles:[5,5,5,5,5]}]}];"
            "ARM.activeIdx = 0; armSyncItems(); ARM.dirty = false;"
            "ARM.selIds = ['i1','i2']; ARM.selectedId = 'i2';"
            "armTransferSelection(1, 'move');"
            "console.log('A=' + ARM.items.map(x => x.tag).join(','));"
            "console.log('B=' + ARM.presets[1].items.map(x => x.tag).join(','));"
            "var ids = ARM.presets[1].items.map(x => x.id);"
            "console.log('UNIQ=' + (new Set(ids).size) + '/' + ids.length);"
            "console.log('SEL=' + ARM.selIds.length + '/' + ARM.selectedId);"
            "console.log('DIRTY=' + ARM.dirty + '/UNDO=' + ARM.undo.length);"
            "console.log('SRC=' + ARM.items.length);")
        self.assertIn("A=p1", out)
        self.assertIn("B=b1,d,p3", out)          # 追加在末尾, 组内相对顺序不变
        self.assertIn("UNIQ=3/3", out)           # 新 id, 与目标里原有的 i0 不撞
        self.assertIn("SEL=0/null", out)         # 搬走的块已不在本动作里 → 选中归零
        self.assertIn("DIRTY=true/UNDO=0", out)  # 跨预设的改动不进撤回栈（撤了会变成两份）
        self.assertIn("SRC=1", out)

    def test_transfer_refuses_to_empty_the_source_preset(self):
        """把整条动作的块全搬走 → 源预设变成空数组, C 编译不过 → 拦下来。"""
        out = self._run(
            "var msg = []; toast = (m) => msg.push(m); btlog = () => {};"
            "armRenderCanvas = () => {}; armRenderAll = () => {};"
            "armRenderBar = () => {}; renderJoints = () => {};"
            "ARM.presets = ["
            "  {id:1,title:'A',items:[{id:'i0',kind:'pose',tag:'p1',angles:[1,1,1,1,1]}]},"
            "  {id:2,title:'B',items:[{id:'i0',kind:'pose',tag:'b1',angles:[5,5,5,5,5]}]}];"
            "ARM.activeIdx = 0; armSyncItems();"
            "ARM.selIds = ['i0']; ARM.selectedId = 'i0';"
            "armTransferSelection(1, 'move');"
            "console.log('SRC=' + ARM.items.length + '/DST=' + ARM.presets[1].items.length);"
            "console.log('MSG=' + (msg[msg.length - 1] || '').indexOf('空数组'));")
        self.assertIn("SRC=1/DST=1", out)        # 一根毫毛都没动
        self.assertIn("MSG=", out)
        self.assertNotIn("MSG=-1", out, "应该提示过「会变成空数组」")

    def test_copy_within_the_same_preset_is_undoable(self):
        """在本动作里复制走正常撤回路径（源没被动, 撤回不会变成两份）。"""
        out = self._run(
            "var msg = []; toast = (m) => msg.push(m); btlog = () => {};"
            "armRenderCanvas = () => {}; armRenderAll = () => {};"
            "armRenderBar = () => {}; renderJoints = () => {};"
            "ARM.presets = [{id:1,title:'A',items:"
            "  [{id:'i0',kind:'pose',tag:'p1',angles:[1,1,1,1,1]},"
            "   {id:'i1',kind:'delay',tag:'d',ms:100}]}];"
            "ARM.activeIdx = 0; armSyncItems(); ARM.undo = [];"
            "ARM.selIds = ['i0']; ARM.selectedId = 'i0';"
            "armTransferSelection(0, 'copy');"
            "console.log('TAGS=' + ARM.items.map(x => x.tag).join(','));"
            "console.log('UNIQ=' + (new Set(ARM.items.map(x => x.id)).size));"
            "console.log('UNDO=' + ARM.undo.length);"
            "console.log('SEL=' + ARM.selIds.join(','));")
        self.assertIn("TAGS=p1,p1,d", out)       # 插在选中块**下面**（与记录位姿同一规则）
        self.assertIn("UNIQ=3", out)             # 克隆换了新 id
        self.assertIn("UNDO=1", out)
        self.assertIn("SEL=", out)
        self.assertNotIn("SEL=i0", out, "复制之后选中的应该是新块")


class TestArmJointNudge(unittest.TestCase):
    """关节滑杆两端的 0.1° 微调箭头。"""

    def setUp(self):
        self.js = srccheck.read_source("static/app.js")
        self.css = srccheck.read_source("static/style.css")

    def test_arrows_sit_at_both_ends_of_the_slider(self):
        """左箭头 / 滑杆 / 右箭头 / 读数，顺序不能乱（箭头就是"滑杆两端"）。"""
        m = re.search(r"row\.appendChild\(mkStep\(-1[\s\S]*?box\.appendChild\(row\);", self.js)
        self.assertIsNotNone(m, "没找到关节行的拼装代码")
        body = m.group(0)
        self.assertLess(body.index("mkStep(-1"), body.index("row.appendChild(rng);"))
        self.assertLess(body.index("row.appendChild(rng);"), body.index("mkStep(1"))
        self.assertLess(body.index("mkStep(1"), body.index("row.appendChild(out);"))

    def test_nudge_writes_through_the_same_path_as_dragging(self):
        """微调必须走 applyJoint（写积木 / 记 touched / 编辑模式下让真机跟过去）。

        ⚠️ 自己再写一遍落库, 迟早和拖滑杆/双击输入漂开 —— 那三处必须只有一条路。"""
        m = re.search(r"const mkStep = \(dir, glyph, tip\) => \{[\s\S]*?\n    \};\n", self.js)
        self.assertIsNotNone(m, "没找到 mkStep")
        body = m.group(0)
        self.assertIn("const v = armStepValue(base, dir, j.min, j.max);", body)
        self.assertIn("applyJoint(v);", body)
        # 基准取**显示出来的那个数**（用户看到的就是它, 实时回读也往这里写）
        self.assertIn("parseFloat(out.textContent)", body)

    def test_nudge_is_greyed_out_with_the_slider(self):
        """灰态（未选中/多选/未连设备）下按钮要 disabled 且**不挂监听** ——
        与滑杆同一条规矩: 挂了就是"看着能按, 按了没反应"。"""
        self.assertIn("disabled: editable ? null : '',", self.js)
        self.assertRegex(self.js, r"if \(editable\) \{\s*btn\.addEventListener\('pointerdown'")
        # 样式选择器必须压过 `.btn.tiny` —— 那条规则带 margin-left:auto, 盖不掉的话
        # 左箭头会被推到右边去（.b-btns 里踩过一模一样的坑）
        self.assertIn(".bt-joint .btn.bt-step {", self.css)
        self.assertIn("margin: 0;", self.css)
        self.assertIn(".bt-joint .btn.bt-step:disabled", self.css)

    def test_hold_to_repeat_captures_the_pointer(self):
        """按住连发必须 setPointerCapture —— 否则在按钮外松手收不到 pointerup,
        连发定时器会一直跑（松手后滑块自己一路爬到底）。"""
        m = re.search(r"const mkStep = \(dir, glyph, tip\) => \{[\s\S]*?\n    \};\n", self.js)
        body = m.group(0)
        self.assertIn("btn.setPointerCapture(e.pointerId)", body)
        for ev in ("pointerup", "pointercancel", "lostpointercapture"):
            self.assertIn("addEventListener('%s', stop)" % ev, body,
                          "%s 没收连发定时器" % ev)
        # 长按松手时会多来一次 click —— 必须跳过, 否则每松一次手就多走一步
        self.assertIn("if (repeated) { repeated = false; return; }", body)
        # 一次按住只压一次撤回快照（与拖滑杆同规则）
        self.assertIn("if (tgt) armPushUndo();", body)

    def test_step_math(self):
        """±0.1°、夹量程、一位小数（不取整会攒出 90.20000000000002 那种数写进 C 表）。"""
        if not srccheck.node_available():
            self.skipTest("没装 node")
        rc, out = srccheck.run_js_top_level(
            srccheck.read_source("static/app.js"),
            "console.log('UP=' + armStepValue(90, 1, 0, 180));"
            "console.log('DN=' + armStepValue(90, -1, 0, 180));"
            "console.log('HI=' + armStepValue(180, 1, 0, 180));"
            "console.log('LO=' + armStepValue(0, -1, 0, 180));"
            "var v = 0; for (var i = 0; i < 100; i++) v = armStepValue(v, 1, 0, 180);"
            "console.log('HUNDRED=' + v);"
            "var d = 180; for (var j = 0; j < 3; j++) d = armStepValue(d, -1, 0, 180);"
            "console.log('DOWN3=' + d);")
        self.assertEqual(rc, 0, out)
        for want in ("UP=90.1", "DN=89.9", "HI=180", "LO=0", "HUNDRED=10", "DOWN3=179.7"):
            self.assertIn(want, out)


class TestChassisPush(LinkCase):
    """$B,CPUSH/$B,CIT 行构造的钉子 —— AGENTS.md「加新段类型 5 处要同步」里
    bt_link 这一环原先零覆盖（漏一处就是 CITBAD/静默丢行）。
    字段序 = stChActItemTdf 声明序: kind,start,dur,wx,wy,speed,accel,delta,armId,hold,abs;
    kind: 0平移 1转弯 2臂 3等臂 4支点（与固件 CH_ACT_KIND_* 一一对应）。"""

    def setUp(self):
        super().setUp()
        FakePort.responses = {"$B,PING": ["$B,PONG,1", "$B,OK"],
                              "$B,CPUSH": ["$B,CPW,5", "$B,OK"], "$B,CIT": ["$B,OK"]}
        self.assertTrue(self.link.connect("COM9", 115200))

    def test_all_five_kinds_emit_fields_in_struct_order(self):
        items = [
            {"kind": "move", "start": 0, "dur": 0, "wx": 0.0, "wy": 300.0,
             "speed": 150, "accel": 120, "hold": True},
            {"kind": "turn", "start": 1600, "dur": 0, "delta": -90,
             "speed": 60, "accel": 80, "abs": True},
            {"kind": "arm", "start": 400, "arm_id": 11},
            {"kind": "arm_wait", "start": 2200, "dur": 3000},
            {"kind": "pivot", "start": 0, "delta": 90, "speed": 40, "abs": False},
        ]
        self.link.push_chassis_action(1, items)
        wr = [w for w in self.last_port().written if w.startswith("$B,C")]
        self.assertEqual(wr[0], "$B,CPUSH,1,5")
        self.assertEqual(wr[1], "$B,CIT,0,0,0,0.00,300.00,150,120,0,0,1,0")
        self.assertEqual(wr[2], "$B,CIT,1,1600,0,0.00,0.00,60,80,-90,0,0,1")
        self.assertEqual(wr[3], "$B,CIT,2,400,0,0.00,0.00,0,0,0,11,0,0")
        self.assertEqual(wr[4], "$B,CIT,3,2200,3000,0.00,0.00,0,0,0,0,0,0")
        self.assertEqual(wr[5], "$B,CIT,4,0,0,0.00,0.00,40,0,90,0,0,0")

    def test_raw_and_disabled_are_not_pushed(self):
        items = [
            {"kind": "raw", "lines": ["    CH_ACT_MOVE     (0u, 0.0f, 1.0f, 1, 1, 0u),"]},
            {"kind": "move", "start": 0, "dur": 0, "wx": 0.0, "wy": 1.0,
             "speed": 10, "accel": 10, "disabled": True},
            {"kind": "arm", "start": 5, "arm_id": 2},
        ]
        self.link.push_chassis_action(1, items)
        wr = [w for w in self.last_port().written if w.startswith("$B,C")]
        self.assertEqual(wr[0], "$B,CPUSH,1,1")
        self.assertEqual(wr[1], "$B,CIT,2,5,0,0.00,0.00,0,0,0,2,0,0")


class TestPresetIdOf(unittest.TestCase):
    """「可重入的任务」勾选框的 id 判据（2026-11 修）。

    新建预设前端只给 ident（g_astScriptPresetN）不给 id, 保存响应也不回填 id ——
    原来勾选框判 cur.id == null, 于是**新建/刚保存的预设勾不了、要重载才行**
    （现场报「执行反向看路」不能勾）。现在走 presetIdOf: id 优先, 否则解析 ident
    （服务端给合规 ident 分的 id 恰是其中的号）。"""

    def test_id_falls_back_to_ident_number(self):
        if not srccheck.node_available():
            self.skipTest("没装 node")
        rc, out = srccheck.run_js_top_level(
            srccheck.read_source("static/app.js"),
            "console.log('NEW=' + presetIdOf({ident: 'g_astScriptPreset48'}));"
            "console.log('DISK=' + presetIdOf({ident: 'g_astScriptPreset48', id: 48}));"
            "console.log('HAND=' + presetIdOf({ident: 'myTable'}));"
            "console.log('HANDREG=' + presetIdOf({ident: 'myTable', id: 7}));"
            "console.log('NUL=' + presetIdOf(null));"
            "console.log('TOK=' + presetToken({ident: 'g_astScriptPreset48'}));"
            "console.log('TOKHAND=[' + presetToken({ident: 'myTable'}) + ']');")
        self.assertEqual(rc, 0, out)
        for want in ("NEW=48", "DISK=48", "HAND=null", "HANDREG=7", "NUL=null",
                     "TOK=SCRIPT_PRESET_48", "TOKHAND=[]"):
            self.assertIn(want, out)

    def test_checkbox_uses_preset_id_of(self):
        """勾选框的显示与点击两处都必须走 presetIdOf, 别再直接判 cur.id == null。
        （断言前剥掉注释: 注释里会引用旧判据原文来解释为什么改, 那不算代码。）"""
        js = srccheck.read_source("static/app.js")

        def code_only(s):
            s = re.sub(r"/\*[\s\S]*?\*/", "", s)
            return re.sub(r"//[^\n]*", "", s)

        m = re.search(r"function renderPresetBar\(\)[\s\S]*?\n\}\n", js)
        self.assertIsNotNone(m, "没找到 renderPresetBar 的函数体")
        self.assertIn("presetIdOf(cur)", code_only(m.group(0)))
        self.assertNotIn("cur.id == null", code_only(m.group(0)))
        m = re.search(r"'preset-reentrant'\)\.addEventListener\('change'[\s\S]*?\n  \}\);", js)
        self.assertIsNotNone(m, "没找到可重入勾选框的 change 监听")
        self.assertIn("presetIdOf(p)", code_only(m.group(0)))
        self.assertNotIn("p.id == null", code_only(m.group(0)))


if __name__ == "__main__":
    unittest.main(verbosity=2)
