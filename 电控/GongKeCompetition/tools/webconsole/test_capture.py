#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""capture.py 的单元测试（不碰真摄像头、不往仓库里写文件）。

全程沙箱:
  · RAW_DIR / CAP_JSON 一律 patch 到 tempfile 建的临时目录（否则会往仓库里写真 cap.json,
    同 bt.json 踩过的坑）
  · cv2 用 FakeCv2 顶替 —— 所以**没装 opencv-python 的机器上本文件也必须全绿**。
    这正是 capture.py 把 cv2 做成可选依赖的意义, 别在这里引入真实依赖。
  · CAMERA 用 FakeCamera 顶替, 绝不真的开设备

跑法:  python tools/webconsole/test_capture.py
"""
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

import capture as C        # noqa: E402
import srccheck            # noqa: E402


# =====================================================================
# 替身
# =====================================================================

class _Buf:
    def __init__(self, data):
        self._data = data

    def tobytes(self):
        return self._data


class _Frame:
    """帧替身: capture.py 只用它的 shape, 以及「是不是 None」。"""
    def __init__(self, h=1080, w=1920):
        self.shape = (h, w, 3)


class FakeCapture:
    """cv2.VideoCapture 的替身。"""
    def __init__(self, opens=True, readable=True):
        self.opens = opens
        self.readable = readable
        self.released = False
        self.sets = []

    def isOpened(self):
        return self.opens

    def set(self, prop, value):
        self.sets.append((prop, value))
        return True

    def get(self, prop):
        return 1920 if prop == FakeCv2.CAP_PROP_FRAME_WIDTH else 1080

    def read(self):
        return (True, _Frame()) if self.readable else (False, None)

    def release(self):
        self.released = True


class FakeCv2:
    """够用的 cv2 替身 —— 只实现 capture.py 真正碰到的那些符号。"""
    INTER_AREA = 1
    IMWRITE_JPEG_QUALITY = 2
    CAP_PROP_FRAME_WIDTH = 3
    CAP_PROP_FRAME_HEIGHT = 4
    CAP_PROP_FOURCC = 5
    CAP_DSHOW = 700
    CAP_V4L2 = 200

    class VideoWriter:
        @staticmethod
        def fourcc(*code):
            return 0x47504A4D          # 'MJPG' 的 little-endian 形式

    def __init__(self, cameras=(0,), readable=True):
        self.cameras = set(cameras)
        self.readable = readable
        self.fail_write = False
        self.calls = []
        self.opened = []

    def VideoCapture(self, index, backend=None):
        self.opened.append(index)
        return FakeCapture(opens=index in self.cameras, readable=self.readable)

    def resize(self, frame, size, interpolation=None):
        self.calls.append(("resize", size))
        return _Frame(size[1], size[0])

    def imencode(self, ext, img, params=None):
        self.calls.append(("imencode", ext))
        return True, _Buf(b"\xff\xd8fake-jpeg")

    def imwrite(self, path, frame):
        self.calls.append(("imwrite", path))
        if self.fail_write:
            return False
        Path(path).write_bytes(b"\xff\xd8fake-body")
        return True


class LegacyCv2(FakeCv2):
    """老 OpenCV: 有 VideoWriter_fourcc, 没有 VideoWriter.fourcc。"""
    VideoWriter = type("VideoWriter", (), {})       # 故意不带 fourcc

    @staticmethod
    def VideoWriter_fourcc(*code):
        return 0x4C454741


class FakeCamera:
    """capture.CAMERA 的替身（模块级单例, 靠 patch 注入）。"""
    def __init__(self, state="ok", frame=_Frame(), age=0.0):
        self.state = state
        self.last_frame = frame
        self.last_frame_time = time.time() - age
        self.desired_index = 1
        self.resolution = (1920, 1080)
        self.lock = threading.Lock()
        self.paused = 0
        self.resumed = 0
        self.switched = []
        self.closed = 0

    @property
    def ready(self):
        return self.state == "ok" and self.last_frame is not None

    @property
    def resolution_text(self):
        w, h = self.resolution
        return "%dx%d" % (w, h) if w and h else ""

    def pause(self):
        self.paused += 1

    def resume(self):
        self.resumed += 1

    def request_switch(self, index):
        self.switched.append(index)
        self.desired_index = index

    def close(self):
        self.closed += 1
        self.state = "idle"
        self.last_frame = None


class Sandbox(unittest.TestCase):
    """把所有路径 / 设备 / 依赖都指向沙箱。"""
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gks_cap_"))
        self.raw = self.tmp / "raw"
        self.json = self.tmp / "cap.json"
        self.cv2 = FakeCv2()
        self.cam = FakeCamera()
        for name, value in (("RAW_DIR", self.raw), ("CAP_JSON", self.json),
                            ("cv2", self.cv2), ("CAMERA", self.cam),
                            ("_s_frame_cache", [None, 0.0, None])):
            p = mock.patch.object(C, name, value)
            p.start()
            self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def files(self, cls):
        d = self.raw / cls
        return sorted(d.glob("*.jpg")) if d.is_dir() else []


# =====================================================================
# 类别词表
# =====================================================================

class TestVocabulary(unittest.TestCase):
    def test_repo_classes_only(self):
        """类别必须与 vision/data.yaml 对齐; 旧工具的 pellet 不许再进来。"""
        self.assertEqual(["cylinder", "cone", "drum", "all"], C.CLASS_NAMES)
        self.assertNotIn("pellet", C.ALLOWED)


# =====================================================================
# 存帧
# =====================================================================

class TestCaptureImage(Sandbox):
    def test_not_ready(self):
        self.cam.state = "connecting"
        ok, msg = C.capture_image("cylinder")
        self.assertFalse(ok)
        self.assertIn("未就绪", msg)
        self.assertEqual([], self.files("cylinder"))

    def test_none_frame(self):
        self.cam.last_frame = None
        ok, msg = C.capture_image("cylinder")
        self.assertFalse(ok)
        self.assertEqual([], self.files("cylinder"))

    def test_stale_frame_rejected(self):
        self.cam.last_frame_time = time.time() - (C.FRAME_STALE_SEC + 1.0)
        ok, msg = C.capture_image("cylinder")
        self.assertFalse(ok)
        self.assertIn("帧过期", msg)
        self.assertEqual([], self.files("cylinder"))

    def test_class_whitelist(self):
        for bad in ("pellet", "", "CYLINDER", "../etc", "cylinder/../.."):
            ok, msg = C.capture_image(bad)
            self.assertFalse(ok, bad)
            self.assertIn("无效类别", msg)
        self.assertEqual([], list(self.raw.glob("**/*.jpg")))
        # 沙箱之外一个目录都不许建
        self.assertFalse((self.tmp / "etc").exists())

    def test_all_bucket_accepted(self):
        ok, _ = C.capture_image("all")
        self.assertTrue(ok)
        self.assertEqual(1, len(self.files("all")))

    def test_happy_path_saves_full_frame(self):
        ok, path = C.capture_image("drum")
        self.assertTrue(ok)
        self.assertEqual(Path(path), self.files("drum")[0])
        # 存的是相机给的原始帧对象, 不是预览缩放过的图
        self.assertEqual(("imwrite", path), self.cv2.calls[-1])
        self.assertEqual(1, C.counts()["drum"])

    def test_same_millisecond_does_not_overwrite(self):
        """同毫秒连拍 → 第二张缀 _1, 绝不覆盖。"""
        cls_dir = self.raw / "cone"
        cls_dir.mkdir(parents=True)
        now = 1700000000.0
        first = C.next_path(cls_dir, now=now)
        first.write_bytes(b"a")
        second = C.next_path(cls_dir, now=now)
        second.write_bytes(b"b")
        self.assertEqual("1700000000000.jpg", first.name)
        self.assertEqual("1700000000000_1.jpg", second.name)
        self.assertEqual(2, len(self.files("cone")))

    def test_existing_files_untouched(self):
        """跨进程重启: 老文件既不被计数也不被覆盖。"""
        cls_dir = self.raw / "cylinder"
        cls_dir.mkdir(parents=True)
        old = cls_dir / "1788726079580.jpg"
        old.write_bytes(b"old")
        ok, path = C.capture_image("cylinder")
        self.assertTrue(ok)
        self.assertNotEqual(str(old), path)
        self.assertEqual(b"old", old.read_bytes())
        self.assertEqual(2, C.counts()["cylinder"])

    def test_write_failure_raises_domain_error(self):
        self.cv2.fail_write = True
        with self.assertRaises(C.CaptureError):
            C.capture_image("cylinder")


class TestCounts(Sandbox):
    def test_missing_dirs_are_zero(self):
        self.assertEqual({name: 0 for name in C.CLASS_NAMES}, C.counts())

    def test_counts_by_listing(self):
        for name, n in (("cylinder", 3), ("drum", 1)):
            d = self.raw / name
            d.mkdir(parents=True)
            for i in range(n):
                (d / ("%d.jpg" % i)).write_bytes(b"x")
        (self.raw / "cone").mkdir(parents=True)
        (self.raw / "cone" / "notes.txt").write_text("不算")     # 非图片后缀不计
        got = C.counts()
        self.assertEqual(3, got["cylinder"])
        self.assertEqual(0, got["cone"])
        self.assertEqual(1, got["drum"])


class TestNextPath(Sandbox):
    def test_ms_name_format(self):
        d = self.raw / "cone"
        d.mkdir(parents=True)
        p = C.next_path(d, now=1788726079.58)
        self.assertEqual("1788726079580.jpg", p.name)
        self.assertEqual(d, p.parent)


# =====================================================================
# 偏好文件
# =====================================================================

class TestPrefs(Sandbox):
    def test_roundtrip_and_merge(self):
        self.assertEqual({}, C.load_prefs())
        C.save_prefs(index=2)
        C.save_prefs(interval=3.5)
        got = C.load_prefs()
        self.assertEqual(2, got["index"])
        self.assertEqual(3.5, got["interval"])

    def test_none_values_are_ignored(self):
        C.save_prefs(index=1)
        C.save_prefs(index=None, interval=2.0)
        self.assertEqual(1, C.load_prefs()["index"])

    def test_corrupt_file_falls_back_to_empty(self):
        self.json.write_text("{ 这不是 json", encoding="utf-8")
        self.assertEqual({}, C.load_prefs())

    def test_no_leftover_tmp(self):
        C.save_prefs(index=0)
        self.assertFalse((self.tmp / "cap.json.tmp").exists())


# =====================================================================
# 摄像头枚举
# =====================================================================

class TestEnumerate(Sandbox):
    def test_hole_at_index_0_does_not_stop_the_scan(self):
        """旧工具「第一个打不开就 break」的毛病: 笔记本只有 index 1 有相机时会扫出空列表。"""
        self.cv2.cameras = {1}
        cams = C.enumerate_cameras()
        self.assertEqual([1], [c["index"] for c in cams])
        self.assertEqual("1920x1080", cams[0]["resolution"])
        self.assertIsNotNone(cams[0]["thumb"])

    def test_stops_after_two_consecutive_misses(self):
        """没相机的机器不该把 4 个 index 全等一遍（每个 index 会试两种后端, 所以看 set）。"""
        self.cv2.cameras = set()
        self.assertEqual([], C.enumerate_cameras())
        self.assertEqual([0, 1], sorted(set(self.cv2.opened)))

    def test_contiguous_cameras(self):
        self.cv2.cameras = {0, 1}
        self.assertEqual([0, 1], [c["index"] for c in C.enumerate_cameras()])

    def test_unreadable_camera_has_no_thumb(self):
        self.cv2.cameras = {0}
        self.cv2.readable = False
        cams = C.enumerate_cameras()
        self.assertEqual(1, len(cams))
        self.assertIsNone(cams[0]["thumb"])


class TestFourcc(unittest.TestCase):
    def test_opencv5_uses_videowriter_fourcc(self):
        with mock.patch.object(C, "cv2", FakeCv2()):
            self.assertEqual(0x47504A4D, C._fourcc("MJPG"))

    def test_legacy_opencv_uses_videowriter_fourcc(self):
        with mock.patch.object(C, "cv2", LegacyCv2()):
            self.assertEqual(0x4C454741, C._fourcc("MJPG"))


class OneFrameThenDead(FakeCapture):
    """实测踩过的坑: 本机这颗相机在 MSMF 后端下先吐一帧, 之后每次 read 都失败。"""
    def __init__(self):
        super().__init__(opens=True)
        self.n = 0

    def read(self):
        self.n += 1
        return (True, _Frame()) if self.n == 1 else (False, None)


class ComboCv2(FakeCv2):
    """**第一个**组合（本机实测是 MSMF 或 MSMF+FOURCC）只吐一帧, 之后能稳定出图。"""
    def VideoCapture(self, index, backend=None):
        self.opened.append((index, backend))
        return OneFrameThenDead() if len(self.opened) == 1 else FakeCapture(opens=True)


class TestWarmup(Sandbox):
    def test_steady_stream_accepted(self):
        self.assertTrue(C._warmup(FakeCapture()))

    def test_one_frame_then_dead_is_rejected(self):
        """只读到 1 帧不能算「这个组合能用」——否则会选中一个根本出不了图的组合。"""
        self.assertFalse(C._warmup(OneFrameThenDead()))

    def test_recovers_from_a_bad_start(self):
        """开头几帧是空的设备, 只要之后连续 3 帧好就认。"""
        class SlowStart(FakeCapture):
            def __init__(self):
                super().__init__(opens=True)
                self.n = 0

            def read(self):
                self.n += 1
                return (False, None) if self.n <= 2 else (True, _Frame())

        self.assertTrue(C._warmup(SlowStart()))

    def test_read_exception_is_a_rejection(self):
        class Boom(FakeCapture):
            def read(self):
                raise RuntimeError("设备炸了")
        self.assertFalse(C._warmup(Boom()))


class TestOpenCap(Sandbox):
    def test_broken_first_combo_falls_through_to_working_one(self):
        """第一个组合只能吐一帧 → 必须被跳过, 落到能真正出图的那个上。"""
        self.cv2 = ComboCv2()
        with mock.patch.object(C, "cv2", self.cv2):
            cap, size = C.Camera()._open_cap(0)
        self.assertIsNotNone(cap, "坏组合被选中了 —— _warmup 的判据又退回单帧了")
        self.assertTrue(cap.read()[0])
        self.assertEqual((1920, 1080), size)
        self.assertGreaterEqual(len(self.cv2.opened), 2, "第一个组合坏掉后没有继续试")

    def test_reopen_after_close_gets_a_fresh_thread(self):
        """close() 之后马上 start(): 必须等旧线程退干净再起新的, 不能两只抢设备。"""
        cv2 = FakeCv2()
        cam = C.Camera()
        with mock.patch.object(C, "cv2", cv2):
            cam.start(0)
            for _ in range(100):
                if cam.state == "ok":
                    break
                time.sleep(0.02)
            self.assertEqual("ok", cam.state)
            cam.close()
            self.assertEqual("idle", cam.state)
            cam.start(0)
            for _ in range(100):
                if cam.state == "ok":
                    break
                time.sleep(0.02)
            self.assertEqual("ok", cam.state, "close 后重开卡住了（旧线程没收干净?）")
        cam.close()

    def test_stops_at_the_first_working_combo(self):
        """第一个就能用 → 不再往下试（避免反复开合同一个设备把它带坏）。"""
        cv2 = FakeCv2()
        with mock.patch.object(C, "cv2", cv2):
            cap, _ = C.Camera()._open_cap(0)
        self.assertIsNotNone(cap)
        self.assertEqual(1, len(cv2.opened))

    @unittest.skipUnless(C.IS_WINDOWS, "组合表按平台不同")
    def test_windows_tries_dshow_first_and_msmf_mjpg_last(self):
        with mock.patch.object(C, "cv2", FakeCv2()):
            table = C._combo_table()
        self.assertEqual((FakeCv2.CAP_DSHOW, "MJPG"), table[0])
        self.assertEqual((None, "MJPG"), table[-1])

    def test_no_index_returns_nothing(self):
        with mock.patch.object(C, "cv2", FakeCv2()):
            self.assertEqual((None, (0, 0)), C.Camera()._open_cap(None))


# =====================================================================
# 取帧
# =====================================================================

class TestFrameBytes(Sandbox):
    def test_none_without_frame(self):
        self.cam.last_frame = None
        self.assertIsNone(C.get_frame_bytes())

    def test_encodes_and_caches_by_frame(self):
        data = C.get_frame_bytes()
        self.assertEqual(b"\xff\xd8fake-jpeg", data)
        n = len([c for c in self.cv2.calls if c[0] == "imencode"])
        self.assertEqual(1, n)
        self.assertEqual(data, C.get_frame_bytes())              # 同一帧 → 命中缓存
        self.assertEqual(n, len([c for c in self.cv2.calls if c[0] == "imencode"]))
        self.cam.last_frame = _Frame()                           # 换帧 → 重编码
        C.get_frame_bytes()
        self.assertEqual(n + 1, len([c for c in self.cv2.calls if c[0] == "imencode"]))

    def test_downscales_1080p_to_540p(self):
        C.get_frame_bytes()
        size = [c[1] for c in self.cv2.calls if c[0] == "resize"][0]
        self.assertEqual((960, 540), size)


class TestReadOnce(Sandbox):
    def test_publishes_when_running(self):
        cam = C.Camera()
        cam.cap = FakeCapture(opens=True)
        cam.state = "ok"
        cam._read_once()
        self.assertIsNotNone(cam.last_frame)

    def test_stale_read_after_close_is_not_published(self):
        """close() 后线程退出前残留的那次成功读, 不能把帧又塞回来 ——
        否则预览接口会一直吐同一张旧图（关掉开关了却还有画面）。"""
        cam = C.Camera()
        cam.cap = FakeCapture(opens=True)
        cam.state = "ok"
        cam._stop.set()
        cam._read_once()
        self.assertIsNone(cam.last_frame)


class TestFrameResponse(Sandbox):
    def test_ok(self):
        status, body, ctype = C.frame_response()
        self.assertEqual(200, status)
        self.assertEqual("image/jpeg", ctype)
        self.assertTrue(body)

    def test_404_without_frame(self):
        self.cam.last_frame = None
        status, _, _ = C.frame_response()
        self.assertEqual(404, status)


# =====================================================================
# 动作分发
# =====================================================================

class TestActions(Sandbox):
    def test_unknown_action(self):
        ok, log, data, err = C.run_capture_action("nope", {})
        self.assertFalse(ok)
        self.assertIn("未知动作", err)
        self.assertIsNone(data)

    def test_scan_pauses_and_resumes(self):
        self.cv2.cameras = {0}
        ok, log, data, err = C.run_capture_action("scan", {})
        self.assertTrue(ok, err)
        self.assertEqual([0], [c["index"] for c in data["cameras"]])
        self.assertEqual(1, self.cam.paused)
        self.assertEqual(1, self.cam.resumed)

    def test_scan_resumes_even_if_enumeration_blows_up(self):
        with mock.patch.object(C, "enumerate_cameras", side_effect=RuntimeError("boom")):
            ok, log, data, err = C.run_capture_action("scan", {})
        self.assertFalse(ok)
        self.assertIn("boom", err)
        self.assertEqual(1, self.cam.resumed)                    # finally 生效

    def test_open_switches_and_remembers(self):
        ok, log, data, err = C.run_capture_action("open", {"index": 2})
        self.assertTrue(ok, err)
        self.assertEqual([2], self.cam.switched)
        self.assertEqual({"index": 2}, data)
        self.assertEqual(2, C.load_prefs()["index"])

    def test_open_rejects_bad_index(self):
        for bad in ("x", None, -1, 99):
            ok, _, _, err = C.run_capture_action("open", {"index": bad})
            self.assertFalse(ok, bad)
            self.assertEqual([], self.cam.switched)

    def test_capture_ok_returns_path_and_counts(self):
        ok, log, data, err = C.run_capture_action("capture", {"class": "cone"})
        self.assertTrue(ok, err)
        self.assertEqual("cone", data["class"])
        self.assertEqual(1, data["counts"]["cone"])
        self.assertTrue(Path(data["path"]).is_file())
        self.assertEqual("cone", C.load_prefs()["class"])
        self.assertEqual("ok", log[-1]["kind"])

    def test_capture_rejects_bad_class(self):
        ok, log, data, err = C.run_capture_action("capture", {"class": "pellet"})
        self.assertFalse(ok)
        self.assertIn("无效类别", err)
        self.assertEqual("bad", log[-1]["kind"])

    def test_capture_when_camera_not_ready_is_soft_failure(self):
        self.cam.state = "disconnected"
        ok, log, data, err = C.run_capture_action("capture", {"class": "cone"})
        self.assertFalse(ok)
        self.assertIn("未就绪", err)
        self.assertIsNone(data)

    def test_close_releases_device(self):
        ok, log, data, err = C.run_capture_action("close", {})
        self.assertTrue(ok, err)
        self.assertEqual(1, self.cam.closed)
        self.assertEqual("idle", data["state"])
        self.assertEqual("ok", log[-1]["kind"])

    def test_enabled_flag_persists(self):
        ok, _, data, err = C.run_capture_action("prefs", {"enabled": False})
        self.assertTrue(ok, err)
        self.assertFalse(data["prefs"]["enabled"])
        self.assertFalse(C.load_prefs()["enabled"])         # 写盘了, 重启控制台仍生效
        ok, _, data, err = C.run_capture_action("prefs", {"enabled": True})
        self.assertTrue(data["prefs"]["enabled"])

    def test_counts_action(self):
        ok, _, data, err = C.run_capture_action("counts", {})
        self.assertTrue(ok, err)
        self.assertEqual(set(C.CLASS_NAMES), set(data["counts"]))

    def test_prefs_action_validates(self):
        ok, _, _, err = C.run_capture_action("prefs", {"interval": 0.05})
        self.assertFalse(ok)
        self.assertIn("0.1", err)
        ok, _, _, err = C.run_capture_action("prefs", {"class": "pellet"})
        self.assertFalse(ok)
        ok, _, data, err = C.run_capture_action("prefs", {"class": "drum", "interval": 1.5})
        self.assertTrue(ok, err)
        self.assertEqual("drum", data["prefs"]["class"])
        self.assertEqual(1.5, data["prefs"]["interval"])

    def test_reveal_opens_raw_dir(self):
        with mock.patch.object(C.os, "startfile", create=True) as fake:
            ok, _, data, err = C.run_capture_action("reveal", {})
        self.assertTrue(ok, err)
        fake.assert_called_once_with(str(self.raw))
        self.assertTrue((self.raw / "cylinder").is_dir())

    def test_unexpected_exception_is_not_raised(self):
        """仓库惯例: 业务异常一律收成 (False, …, str), 绝不抛到 HTTP 层。"""
        with mock.patch.object(C, "counts", side_effect=RuntimeError("炸了")):
            ok, log, data, err = C.run_capture_action("counts", {})
        self.assertFalse(ok)
        self.assertEqual("炸了", err)


class TestState(Sandbox):
    def test_shape(self):
        st = C.capture_state()
        self.assertTrue(st["available"])
        self.assertEqual("", st["hint"])
        self.assertEqual("ok", st["state"])
        self.assertTrue(st["ready"])
        self.assertEqual("1920x1080", st["resolution"])
        self.assertEqual(["cylinder", "cone", "drum", "all"],
                         [c["name"] for c in st["classes"]])
        self.assertEqual(str(self.raw), st["raw_dir"])
        self.assertEqual(C.FRAME_POLL_MS, st["poll_ms"])

    def test_never_raises_when_counts_blows_up(self):
        with mock.patch.object(C, "counts", side_effect=RuntimeError("炸了")):
            st = C.capture_state()
        self.assertEqual({name: 0 for name in C.CLASS_NAMES}, st["counts"])


# =====================================================================
# 没装 opencv
# =====================================================================

class TestCv2Missing(unittest.TestCase):
    """opencv 缺失时: 模块可导入、状态如实上报、所有入口软失败, 绝不抛异常。"""

    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="gks_cap_nocv_"))
        for name, value in (("RAW_DIR", self.tmp / "raw"),
                            ("CAP_JSON", self.tmp / "cap.json"),
                            ("cv2", None),
                            ("CAMERA", FakeCamera())):
            p = mock.patch.object(C, name, value)
            p.start()
            self.addCleanup(p.stop)
        self.addCleanup(shutil.rmtree, self.tmp, ignore_errors=True)

    def test_available_false(self):
        self.assertFalse(C.available())

    def test_state_has_hint(self):
        st = C.capture_state()
        self.assertFalse(st["available"])
        self.assertIn("pip install opencv-python", st["hint"])
        self.assertFalse(st["ready"])
        self.assertEqual("", st["resolution"])
        self.assertEqual(4, len(st["classes"]))          # 类别表照样下发, 前端才有提示可画

    def test_every_action_soft_fails(self):
        for action, payload in (("scan", {}), ("open", {"index": 0}), ("close", {}),
                                ("capture", {"class": "cone"}), ("counts", {}),
                                ("prefs", {"interval": 2}), ("reveal", {})):
            ok, log, data, err = C.run_capture_action(action, payload)
            self.assertFalse(ok, action)
            self.assertIn("pip install opencv-python", err)
            self.assertEqual([], log)

    def test_frame_helpers(self):
        self.assertIsNone(C.get_frame_bytes())
        self.assertEqual(404, C.frame_response()[0])
        self.assertEqual((False, C.CV2_HINT), C.capture_image("cone"))

    def test_enumerate_raises_domain_error(self):
        with self.assertRaises(C.CaptureError):
            C.enumerate_cameras()


# =====================================================================
# 跨文件契约（源码级, 锁住容易静默失效的三处）
# =====================================================================

class TestFrontendContract(unittest.TestCase):
    def setUp(self):
        self.js = (HERE / "static" / "app.js").read_text(encoding="utf-8")
        self.html = (HERE / "static" / "index.html").read_text(encoding="utf-8")
        self.css = (HERE / "static" / "style.css").read_text(encoding="utf-8")
        self.py = (HERE / "server.py").read_text(encoding="utf-8")

    def test_tab_and_pane_exist(self):
        self.assertIn("{ id: 'capture', label: '采集' }", self.js)
        self.assertIn('id="pane-capture"', self.html)

    def test_pane_layout_override_is_not_forgotten(self):
        """少这条覆盖, 采集页会继承「地图」的三列网格, 静默错版。"""
        self.assertIn("#pane-capture.active", self.css)

    def test_tab_exit_stops_timers(self):
        self.assertIn("capStopTimers", self.js)
        self.assertIn("if (id !== 'capture') capStopTimers();", self.js)

    def test_master_switch_exists_and_releases_device(self):
        """总开关: 关掉要真的让服务端释放设备, 不能只停前端定时器。"""
        self.assertIn('id="cap-enable"', self.html)
        self.assertIn("capSetEnabled", self.js)
        self.assertIn("capRun('close'", self.js)
        self.assertIn('"/api/capture"', self.py)
        self.assertIn('if action == "close":', open(
            HERE / "capture.py", encoding="utf-8").read())

    def test_frame_endpoint_matches_server(self):
        self.assertIn("'/api/capture/frame.jpg?t='", self.js)
        self.assertIn('"/api/capture/frame.jpg"', self.py)

    def test_server_routes_and_guards(self):
        self.assertIn('import capture', self.py)
        self.assertIn('elif self.path == "/api/capture":', self.py)
        # 白名单与 Origin 前缀元组都要带上它（会开摄像头并往仓库写图）。
        # ⚠️ 别再用「某个两路径子串出现几次」那种数法: 加第三个端点时它会失配,
        #    却没告诉你漏了哪一个元组。
        whitelist, origin = srccheck.guard_tuples(self.py)
        self.assertIn("/api/capture", whitelist, "POST 白名单漏了 /api/capture")
        self.assertIn("/api/capture", origin, "Origin 校验元组漏了 /api/capture")

    def test_query_string_is_stripped(self):
        """前端靠 ?t= 破缓存, 而路由是精确匹配 —— 不剥查询串就 404。"""
        self.assertGreaterEqual(self.py.count('self.path.partition("?")[0]'), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
