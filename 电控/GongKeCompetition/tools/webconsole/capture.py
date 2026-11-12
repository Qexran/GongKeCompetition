#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""「采集」Tab 的服务端 —— 摄像头预览 + 分类采集到 vision/datasets/raw/<类别>/。

⚠️ 这是控制台里**唯一**带可选第三方依赖的模块:

    try:
        import cv2
    except ImportError:
        cv2 = None

没装 opencv-python 时本模块**仍必须能被 import**, 所有入口返回 available:false 加一句
安装提示 —— server.py 与其余四个 Tab 一位都不受影响（见 CLAUDE.md 的「采集」节）。
由此派生三条铁律, 改这个文件前先读懂:

  · 只有上面那一处 try/except。禁止在任何作用域 import numpy —— 要用图的时候直接把
    cv2 给的 ndarray 使, 不为它单独引包。
  · 禁止在 import 期调用任何 cv2.*（所以 _fourcc() / _combo_table() 是函数不是常量,
    否则 cv2 缺失时整个模块 import 就炸, 「没装 opencv 也能开控制台」这条就断了）。
  · 「没画面」不做占位图: 预览接口直接回 404, 前端保留上一帧并盖一层 CSS 遮罩。
    反正 cv2.putText 是 Hershey 字体, 本来也写不出中文。

摄像头由一个后台线程（camera-watchdog）**独占**访问: ThreadingHTTPServer 每请求一线程,
多线程并发碰同一个 VideoCapture 会崩。请求线程只读缓存帧的引用（整帧替换是原子的）。
命令锁挂在**模块级单例** CAMERA 上 —— 同 bt_link.LINK 的道理, 锁挂实例等于没锁。
"""

import base64
import json
import os
import platform
import threading
import time
from pathlib import Path

try:
    import cv2                      # 可选依赖: 只为「采集」Tab 存在, 见模块 docstring
except ImportError:                 # pragma: no cover - 取决于环境
    cv2 = None

APP_DIR = Path(__file__).resolve().parent
ROOT = APP_DIR.parents[1]                           # tools/webconsole → 仓库根

CAP_JSON = APP_DIR / "cap.json"                     # 本机私有偏好, 不入库（同 bt.json）
RAW_DIR = ROOT / "vision" / "datasets" / "raw"      # 采集落盘根目录（vision/datasets/ 已 gitignore）

# 类别词表 = vision/data.yaml 的口径。⚠️ 别再引回旧工具的 pellet —— 它就是这里的 drum
# （固件 Application/app_beam.h: SHAPE_DRUM = 3, /* 腰鼓 drum（Pi 模型类名 pellet） */）。
# key 直接当子目录名, 所以 ALLOWED 这个白名单同时兼任路径穿越防护。
CLASSES = [("cylinder", "圆柱体 cylinder"),
           ("cone",     "圆锥体 cone"),
           ("drum",     "腰鼓体 drum"),
           ("all",      "全部 all（三者同框暂存）")]
CLASS_NAMES = [name for name, _ in CLASSES]
ALLOWED = frozenset(CLASS_NAMES)
IMG_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp"}

CV2_HINT = ("未安装 opencv-python，「采集」不可用。"
            "在仓库根执行： pip install opencv-python")

FRAME_WIDTH, FRAME_HEIGHT = 1920, 1080
PREVIEW_MAX_W, PREVIEW_MAX_H = 960, 540             # 预览缩放上限（存的仍是原分辨率）
JPEG_QUALITY = 85
THUMB_H, THUMB_QUALITY = 96, 70
READ_INTERVAL = 0.033                               # 看门狗取帧节拍 ≈30fps
STREAM_CHECK_FRAMES = 5                             # 认一个后端可用所需的连续成功帧数（见 _warmup）
DISCONNECT_THRESHOLD = 20                           # 连读失败次数 → 判定断线（≈0.66s）
RECONNECT_INTERVAL = 2.0
FRAME_STALE_SEC = 2.0                               # 采集时允许的帧最大年龄
DEFAULT_INDEX = 1                                   # 笔记本内置一般 0, USB 一般 1
FALLBACK_INDICES = [0, 2, 3]                        # 只用于开机首次选择, 重连不回退
SCAN_MAX = 4
FRAME_POLL_MS = 33                                  # 前端单帧轮询节拍（随 state 下发, 保持一处定义）
IS_WINDOWS = platform.system() == "Windows"

# 预览编码记忆: **单元素元组**（帧对象, 时间戳, JPEG 字节）—— 整体换绑, 见 get_frame_bytes。
# ⚠️ 用元组而不是 list 逐槽写: 逐槽写时读者可能拿到「[0]/[1] 已是新帧、[2] 还是上一帧的
#    JPEG」的组合（两个请求线程交错就会预览闪一帧旧画面）。
_s_frame_cache = (None, 0.0, None)
_s_frame_error = None                               # 上次编码失败的报错（同一条只喊一次, 见下）


class CaptureError(Exception):
    """采集业务错误（设备不可用/参数非法）。由 run_capture_action 转成 (False, …, str)。"""


def available():
    """opencv 是否可用。**所有**入口的第一道分支都看它。"""
    return cv2 is not None


# =====================================================================
# 低层: 后端探测 / 取帧
# =====================================================================

def _fourcc(code):
    """'MJPG' → fourcc 整数。OpenCV 5 去掉了 cv2.VideoWriter_fourcc, 改用 VideoWriter.fourcc。"""
    fn = getattr(cv2, "VideoWriter_fourcc", None) or getattr(cv2.VideoWriter, "fourcc", None)
    return fn(*code) if fn else 0


def _combo_table():
    """「后端 × 像素格式」的尝试顺序 —— **先来的先用**, 不再逐个试完挑最高分辨率。

    ⚠️ 旧写法（试完所有组合、挑上报分辨率最高的那个）在本机实测是坏的, 两个原因:
      · 名叫 MSMF 的那个后端一旦设 FOURCC=MJPG 就**一帧都出不来**（实测 0/30,
        报 `cap_msmf ... can't grab frame`）。它排在前面会把设备带进坏状态。
      · 「挑最高分辨率」意味着同一个设备被接连开好几次（已选中的那个还一直握着不释放,
        见旧 _open_cap）, 而 UVC 摄像头被同一进程反复开合很容易变成
        「先吐一帧、之后每次 read 都失败」—— 表现就是预览永远 404、状态 ok↔connecting 刷屏。
    所以改成: 挨个试, **第一个能稳定出图的就用它并立刻收手**, 每次只握一个设备。
    DSHOW+MJPG 放最前, 依据是本项目 capture_gui.py 当年的实测记录
    「Windows 下优先 DSHOW + MJPG, 以拿到 1080p」; MSMF+MJPG 垫底。

    必须是函数 —— 模块不能在 import 期碰 cv2.*（见 docstring）。
    None 后端 = OpenCV 的平台默认。
    """
    if cv2 is None:
        return []
    if IS_WINDOWS:
        dshow = getattr(cv2, "CAP_DSHOW", 700)
        return [(dshow, "MJPG"), (dshow, None), (None, None), (None, "MJPG")]
    v4l2 = getattr(cv2, "CAP_V4L2", 200)
    return [(v4l2, "MJPG"), (v4l2, None), (None, None)]


def _warmup(cap, tries=12, need=None):
    """连读若干帧, 要**连续** STREAM_CHECK_FRAMES 帧都成功才认这个组合能用。

    ⚠️ 「读到 1 帧就算能用」是不够的, 这是实测踩出来的: 坏组合的典型症状就是
    **先吐一帧、然后每次 read 都失败**（本机 MSMF+FOURCC=MJPG 则是干脆 0/30）。
    拿 1 帧当"能用"会让 _open_cap 选中一个根本出不了图的组合 —— 表现为预览永远 404、
    状态 ok↔connecting 刷屏。要连续若干帧才算数, 顺带也保留了原来「前几帧是空的」
    那层预热语义（凑够连续几帧就行, 不要求第一帧就成功）。
    """
    need = STREAM_CHECK_FRAMES if need is None else need
    streak = 0
    for _ in range(tries):
        try:
            ok, frame = cap.read()
        except Exception:
            return False
        if ok and frame is not None:
            streak += 1
            if streak >= need:
                return True
        else:
            streak = 0
    return False


def _thumb_b64(frame):
    """缩到高 96 的 base64 JPEG（摄像头卡片上的缩略图）。"""
    h, w = frame.shape[:2]
    if h <= 0 or w <= 0:
        return None
    scale = THUMB_H / float(h)
    small = cv2.resize(frame, (max(1, int(w * scale)), THUMB_H),
                       interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), THUMB_QUALITY])
    return base64.b64encode(buf.tobytes()).decode("ascii") if ok else None


def _probe_camera(index):
    """试开一个 index → {index, resolution, thumb}；打不开回 None。"""
    cap = None
    try:
        backends = [None] + ([getattr(cv2, "CAP_DSHOW", 700)] if IS_WINDOWS
                             else [getattr(cv2, "CAP_V4L2", 200)])
        for backend in backends:
            try:
                cap = cv2.VideoCapture(index) if backend is None else cv2.VideoCapture(index, backend)
            except Exception:
                cap = None
            if cap is not None and cap.isOpened():
                break
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
                cap = None
        if cap is None:
            return None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
        thumb = None
        for _ in range(8):
            try:
                ok, frame = cap.read()
            except Exception:
                break
            if ok and frame is not None:
                thumb = _thumb_b64(frame)
                break
        return {"index": index,
                "resolution": ("%dx%d" % (w, h)) if w and h else "unknown",
                "thumb": thumb}
    except Exception:
        return None
    finally:
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass


def enumerate_cameras(max_test=SCAN_MAX):
    """扫出可用摄像头。

    ⚠️ 不能「遇到第一个打不开的 index 就 break」（旧工具的老毛病）: 笔记本上如果
    只有 index 1 有相机, index 0 打不开就 break 会让列表永远空着。改成「连续两个
    打不开才收手」—— 既容得下空洞, 又不会在没相机的机器上把 4 个 index 全等一遍。
    """
    if cv2 is None:
        raise CaptureError(CV2_HINT)
    found, misses = [], 0
    for index in range(max_test):
        got = _probe_camera(index)
        if got is None:
            misses += 1
            if misses >= 2:
                break
            continue
        misses = 0
        found.append(got)
    return found


# =====================================================================
# 摄像头 + 看门狗线程
# =====================================================================

class Camera:
    """单个摄像头 + 一个后台看门狗线程。

    状态机:
        idle ──start()──► starting ──► ok ⇄ connecting ──► disconnected ──┐
                           │          ▲                                  │
                           └─ 全打不开 ─┴──────────── 每 2s 重试 ────────┘
        paused 由 pause()/resume() 进出（扫描其它设备前先让位, 扫完还原原状态）。

    重连**只重开 desired_index**, 不回退到别的 index —— 采集中途偷偷换摄像头比断线更糟。
    """

    def __init__(self):
        self.lock = threading.Lock()
        self.cap = None
        self.last_frame = None
        self.last_frame_time = 0.0
        self.state = "idle"
        self.desired_index = None
        self.resolution = (0, 0)
        self._fail_count = 0
        self._resume_state = None
        self._stop = threading.Event()
        self._thread = None
        # 只序列化「起/换取流线程」这件事（不能复用 self.lock —— 见 _ensure_thread）
        self._ensure_lock = threading.Lock()

    @property
    def ready(self):
        return self.state == "ok" and self.last_frame is not None

    @property
    def resolution_text(self):
        w, h = self.resolution
        return "%dx%d" % (w, h) if w and h else ""

    # ---- 生命周期 ----

    def _ensure_thread(self, join_wait=2.0):
        # ⚠️ 整段串行化（用**独立**的小锁, 不能用 self.lock）: ThreadingHTTPServer 每请求
        #    一线程, 而 CAMERA 是模块级单例 —— 两个客户端同时 POST {action:"open"}
        #    会都看到 self._thread 为空/已死, 各起一只看门狗线程, 两只同时 cap.read()
        #    同一个 VideoCapture（就是注释里反复警告的「先吐一帧、之后每次 read 都失败」）;
        #    而且只有一只能被 close() 杀掉, 另一只变成孤儿线程永久占着设备。
        # ⚠️ 不能用 self.lock: 看门狗线程的 _loop 也要拿它, 持着它去 join 就是互等
        #    （结果必然是 join 超时 → 永远起不了新线程）。
        with self._ensure_lock:
            thread = self._thread
            if thread is not None and thread.is_alive():
                if not self._stop.is_set():
                    return                              # 已经有一只健康的在跑
                # 上一只正在收尾（close() 之后马上又 start()）: 必须等它退干净再起新的,
                # 否则两只线程会同时碰同一个设备 —— 正是「先吐一帧、之后 read 全失败」的成因。
                thread.join(join_wait)
                if thread.is_alive():
                    # 旧线程卡在 cap.read()（设备 wedge）里没退 → 本次**不**起第二只。
                    # ⚠️ 但必须把状态从 starting 挪开: 留着它, 前端会一直转圈、而下次点
                    #    「打开」走的还是这条路 —— 现场看就是「按了没反应」。
                    with self.lock:
                        if self.state == "starting":
                            self.state = "disconnected"
                            self._resume_state = None
                    print("[WARN] 上一只取流线程未退出（设备可能 wedge）, 本次不再新开一只; "
                          "状态已置 disconnected —— 可再点一次「打开」或断电重插相机")
                    return
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, name="camera-watchdog", daemon=True)
            self._thread.start()

    def start(self, index):
        """首次开始取流。服务器**不会**自动调它 —— 不主动抢用户的摄像头。"""
        self.desired_index = index
        with self.lock:
            if self.state in ("idle", "disconnected"):
                self.state = "starting"
        self._ensure_thread()

    def request_switch(self, index):
        """换摄像头（异步: 真正的开/关在看门狗线程里做）。"""
        self.desired_index = index
        with self.lock:
            self._release_locked()
            self.state = "starting"
        self._ensure_thread()

    def pause(self):
        """让出设备（扫描前用）。记住原状态供 resume() 还原。"""
        with self.lock:
            self._resume_state = self.state
            self.state = "paused"
            self._release_locked()

    def resume(self):
        with self.lock:
            prev, self._resume_state = self._resume_state or "idle", None
            self.state = "idle" if prev == "idle" else "connecting"

    def close(self):
        """停掉取流并**立刻释放设备** —— 关掉「采集」后摄像头就不再被这个进程占着,
        别的程序（或 vision/gui.py）才抢得到它。

        同步释放（不等看门狗线程自然退出）, 所以返回后 LED 就该灭了;
        线程本身最多再活一个 read 的时间, `_ensure_thread` 会等它退干净。
        """
        self._stop.set()
        with self.lock:
            self._release_locked()
            self.last_frame = None
            self.last_frame_time = 0.0
            self.state = "idle"
        thread = self._thread
        if thread is not None and thread.is_alive():
            thread.join(1.0)

    def _release_locked(self):
        """调用方必须已持 self.lock。"""
        cap, self.cap = self.cap, None
        if cap is not None:
            try:
                cap.release()
            except Exception:
                pass

    # ---- 看门狗线程 ----

    def _loop(self):
        while not self._stop.is_set():
            try:
                state = self.state
                if state == "starting":
                    self._startup_select()
                elif state == "connecting":
                    self._reconnect()
                elif state == "ok":
                    self._read_once()
                    time.sleep(READ_INTERVAL)
                elif state == "disconnected":
                    time.sleep(RECONNECT_INTERVAL)
                    if self.state == "disconnected":
                        self.state = "connecting"
                else:                                   # idle / paused
                    time.sleep(0.3)
            except Exception as exc:                    # 线程绝不能死
                print("[capture] 看门狗异常: %s" % exc, flush=True)
                time.sleep(1.0)

    def _startup_select(self):
        """开机首次选择: 先试想要的 index, 再依次回退。之后不再回退。"""
        want = self.desired_index if self.desired_index is not None else DEFAULT_INDEX
        order = [want] + [i for i in FALLBACK_INDICES if i != want]
        for index in order:
            cap, size = self._open_cap(index)
            if cap is None:
                continue
            with self.lock:
                if self.state != "starting":            # 期间被 pause/换机抢先 → 丢弃这次结果
                    try:
                        cap.release()
                    except Exception:
                        pass
                    return
                self.cap = cap
                self.resolution = size
                self.desired_index = index
                self._fail_count = 0
                self.state = "ok"
            print("[capture] 已连接摄像头 #%d（%s）" % (index, self.resolution_text), flush=True)
            return
        print("[capture] 未找到可用设备（尝试过 %s）, 每 %.0fs 重试一次"
              % (order, RECONNECT_INTERVAL), flush=True)
        self.state = "disconnected"

    def _reconnect(self):
        """只重开 desired_index —— 不静默换摄像头。"""
        cap, size = self._open_cap(self.desired_index)
        with self.lock:
            if self.state != "connecting":              # 期间被抢先
                if cap is not None:
                    try:
                        cap.release()
                    except Exception:
                        pass
                return
            if cap is None:
                self.state = "disconnected"
                return
            self.cap = cap
            self.resolution = size
            self._fail_count = 0
            self.state = "ok"
        print("[capture] 摄像头 #%s 已重新接入（%s）"
              % (self.desired_index, self.resolution_text), flush=True)

    def _open_cap(self, index):
        """按 _combo_table() 挨个试, 返回**第一个**能稳定出图的 (cap, size)。

        只握一个设备: 每个落选的候选当场释放, 不做「边握一个边试下一个」
        （那正是让 UVC 摄像头进坏状态的原因, 见 _combo_table 注释）。
        """
        if cv2 is None or index is None:
            return None, (0, 0)
        for backend, fourcc_code in _combo_table():
            cap, size = self._try_combo(index, backend, fourcc_code)
            if cap is not None:
                return cap, size
        return None, (0, 0)

    def _try_combo(self, index, backend, fourcc_code):
        cap = None
        try:
            cap = cv2.VideoCapture(index) if backend is None else cv2.VideoCapture(index, backend)
            if not cap.isOpened():
                cap.release()
                return None, (0, 0)
            if fourcc_code:
                cap.set(cv2.CAP_PROP_FOURCC, _fourcc(fourcc_code))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
            if not _warmup(cap):
                cap.release()
                return None, (0, 0)
            w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
            h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
            return cap, (w, h)
        except Exception:
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            return None, (0, 0)

    def _read_once(self):
        with self.lock:
            cap = self.cap
        if cap is None:
            return
        try:
            ok, frame = cap.read()
        except Exception:
            ok, frame = False, None
        if ok and frame is not None:
            if self._stop.is_set():
                # close() 已按下: 这次读是线程退出前残留的一次, 别把帧塞回去
                # —— 否则 last_frame 会停在最后一张旧图上, 预览接口就一直吐它（已踩过）。
                return
            self.last_frame = frame
            self.last_frame_time = time.time()
            self._fail_count = 0
            return
        self._fail_count += 1
        if self._fail_count >= DISCONNECT_THRESHOLD and self.state == "ok":
            print("[capture] 摄像头无响应, 等待重新接入…", flush=True)
            with self.lock:
                self._release_locked()
                self.state = "disconnected"
                self.last_frame = None          # 旧帧作废 → 预览回 404, 前端留画面盖遮罩
            self._fail_count = 0


CAMERA = Camera()               # 模块级单例（测试 mock.patch.object(capture, "CAMERA", fake)）


# =====================================================================
# 取帧 / 落盘 / 偏好
# =====================================================================

def get_frame_bytes():
    """当前帧 → 缩放后的 JPEG 字节；没帧回 None。

    带一层按帧缓存的编码记忆: 原来的实现在**每个客户端每帧**都重缩重编一次, 多开一个
    标签页就多一份 CPU。缓存键用帧对象的 is 比较 + 时间戳, 换帧即失效。
    """
    global _s_frame_cache                         # 整体换绑（见下）, 必须声明 global
    if cv2 is None:
        return None
    frame = CAMERA.last_frame
    if frame is None:
        return None
    stamp = CAMERA.last_frame_time
    cached = _s_frame_cache                       # 一次取引用 = 拿到一个**一致**的三元组
    if cached[0] is frame and cached[1] == stamp:
        return cached[2]
    try:
        h, w = frame.shape[:2]
        if h <= 0 or w <= 0:
            return None
        scale = min(PREVIEW_MAX_W / float(w), PREVIEW_MAX_H / float(h), 1.0)
        img = frame if scale >= 1.0 else cv2.resize(
            frame, (max(1, int(w * scale)), max(1, int(h * scale))),
            interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY])
        if not ok:
            return None
        data = buf.tobytes()
    except Exception as exc:                    # noqa: BLE001
        # 静默 None 会让"预览一直 404"变得无从下手（踩过）, 所以同一条报错喊一次。
        global _s_frame_error
        if _s_frame_error != str(exc):
            _s_frame_error = str(exc)
            print("[capture] 预览帧编码失败: %s" % exc, flush=True)
        return None
    _s_frame_cache = (frame, stamp, data)         # 整体换绑（读者不可能拿到半新半旧）
    return data


def ensure_raw_dirs():
    for name in CLASS_NAMES:
        (RAW_DIR / name).mkdir(parents=True, exist_ok=True)


def counts():
    """每个类别目录里的图片数。按目录实时数 —— 删过图也准（增量计数器做不到）。"""
    out = {}
    for name in CLASS_NAMES:
        n = 0
        d = RAW_DIR / name
        if d.is_dir():
            try:
                n = sum(1 for f in d.iterdir()
                        if f.is_file() and f.suffix.lower() in IMG_SUFFIXES)
            except OSError:
                n = 0
        out[name] = n
    return out


def next_path(cls_dir, now=None):
    """{毫秒}.jpg —— 与 vision/scripts/capture.py、gui.py 的导入器同一命名。

    不用「{类别}_{序号:05d}」: split_dataset.py 会用目录名再缀一次前缀, 序号方案会得到
    cylinder_cylinder_00001.jpg 这种叠字, 而毫秒方案得到的 cylinder_1788726079580.jpg
    正是 datasets/images/ 里现有的形态。顺带不需要任何计数器状态, 同毫秒撞了就缀 _N。
    """
    base = int((now if now is not None else time.time()) * 1000)
    path = cls_dir / ("%d.jpg" % base)
    n = 1
    while path.exists():
        path = cls_dir / ("%d_%d.jpg" % (base, n))
        n += 1
    return path


def capture_image(class_name):
    """存一帧原分辨率图 → (ok, 路径或错误文案)。"""
    if cv2 is None:
        return False, CV2_HINT
    if class_name not in ALLOWED:                       # 白名单同时挡住路径穿越
        return False, "无效类别: %s" % class_name
    if not CAMERA.ready:
        return False, "摄像头未就绪"
    if time.time() - CAMERA.last_frame_time > FRAME_STALE_SEC:
        return False, "摄像头无响应（帧过期）"
    frame = CAMERA.last_frame
    if frame is None:
        return False, "摄像头未就绪"
    cls_dir = RAW_DIR / class_name
    try:
        cls_dir.mkdir(parents=True, exist_ok=True)
        path = next_path(cls_dir)
        ok = cv2.imwrite(str(path), frame)              # 原分辨率, 预览的缩放不许泄漏到这
    except Exception as exc:
        raise CaptureError("写盘失败: %s" % exc)
    if not ok:
        raise CaptureError("写盘失败: %s" % path)
    return True, str(path)


def load_prefs():
    """cap.json → dict。坏文件/不存在一律回 {}（偏好丢了大不了重选一次）。"""
    try:
        data = json.loads(CAP_JSON.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}
    return data if isinstance(data, dict) else {}


def save_prefs(**kw):
    """合并写入 cap.json（先写 .tmp 再 os.replace, 中途失败不会毁掉原文件）。"""
    prefs = load_prefs()
    for key, value in kw.items():
        if value is not None:
            prefs[key] = value
    try:
        tmp = CAP_JSON.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(prefs, ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(tmp, CAP_JSON)
    except OSError:
        pass                                            # 写不进去不致命（同 bt_link.save_prefs）
    return prefs


# =====================================================================
# 对 server.py 的两个入口
# =====================================================================

def capture_state():
    """GET /api/capture 的载荷。**永不抛异常** —— 没装 opencv 也是一种正常状态。"""
    ok = available()
    try:
        cls_counts = counts()
    except Exception:
        cls_counts = {name: 0 for name in CLASS_NAMES}
    return {
        "available": ok,
        "hint": "" if ok else CV2_HINT,
        "state": CAMERA.state if ok else "idle",
        "ready": bool(CAMERA.ready) if ok else False,
        "index": CAMERA.desired_index,
        "resolution": CAMERA.resolution_text if ok else "",
        "counts": cls_counts,
        "classes": [{"name": name, "label": label} for name, label in CLASSES],
        "raw_dir": str(RAW_DIR),
        "poll_ms": FRAME_POLL_MS,
        "prefs": load_prefs(),
    }


def frame_response():
    """GET /api/capture/frame.jpg 的 (status, body, ctype)。

    故意做成模块级函数而不是 Handler 的方法: 仓库的测试惯例是不碰 Handler/_send
    （那需要真 socket）, 这样单帧接口也能被单测覆盖。
    无帧 = 404: 前端保留上一帧画面, 由 CSS 遮罩解释「断线/未启动」。
    """
    if not available():
        return 404, {"error": "opencv 不可用"}, "application/json; charset=utf-8"
    data = get_frame_bytes()
    if not data:
        return 404, {"error": "no frame"}, "application/json; charset=utf-8"
    return 200, data, "image/jpeg"


def _need_index(payload):
    try:
        index = int(payload.get("index"))
    except (TypeError, ValueError):
        raise CaptureError("无效的摄像头索引")
    if not 0 <= index < SCAN_MAX:
        raise CaptureError("摄像头索引超出范围: %d" % index)
    return index


def run_capture_action(action, payload):
    """POST /api/capture 的分发 → (ok, log, data, error)。

    与 run_bt_action 同形: 业务失败也是 HTTP 200, 客户端读 ok 而不是状态码。
    log 条目是 {"text", "kind"}（kind ∈ ok|bad）, 前端直接复用 capRun 的渲染。
    """
    log = []

    def note(text, kind="ok"):
        log.append({"text": text, "kind": kind})

    try:
        if not available():
            return False, log, None, CV2_HINT

        if action == "scan":
            CAMERA.pause()                              # 扫别的设备前先让位
            try:
                cams = enumerate_cameras()
            finally:
                CAMERA.resume()
            note("扫描到 %d 个摄像头" % len(cams))
            return True, log, {"cameras": cams, "current": CAMERA.desired_index}, ""

        if action == "open":
            index = _need_index(payload)
            CAMERA.request_switch(index)                # 异步: 真正的开关在看门狗线程里
            save_prefs(index=index)
            note("正在切换到摄像头 #%d …" % index)
            return True, log, {"index": index}, ""

        if action == "close":
            CAMERA.close()
            note("已关闭摄像头（设备已释放）")
            return True, log, {"state": CAMERA.state}, ""

        if action == "capture":
            class_name = str(payload.get("class") or "").strip()
            if class_name not in ALLOWED:
                raise CaptureError("无效类别: %s" % class_name)
            ok, msg = capture_image(class_name)
            if not ok:
                note("✗ %s" % msg, "bad")
                return False, log, None, msg
            save_prefs(**{"class": class_name})
            note("已保存 %s" % msg)
            return True, log, {"class": class_name, "path": msg, "counts": counts()}, ""

        if action == "counts":
            return True, log, {"counts": counts()}, ""

        if action == "prefs":
            patch = payload.get("prefs") if isinstance(payload.get("prefs"), dict) else payload
            kw = {}
            if patch.get("class") is not None:
                if patch["class"] not in ALLOWED:
                    raise CaptureError("无效类别: %s" % patch["class"])
                kw["class"] = patch["class"]
            if patch.get("interval") is not None:
                try:
                    interval = float(patch["interval"])
                except (TypeError, ValueError):
                    raise CaptureError("采集间隔无效")
                if interval < 0.1:
                    raise CaptureError("采集间隔至少 0.1 秒")
                kw["interval"] = interval
            if patch.get("index") is not None:
                kw["index"] = _need_index({"index": patch["index"]})
            if patch.get("enabled") is not None:
                kw["enabled"] = bool(patch["enabled"])     # 「采集」总开关, 关掉即释放设备
            return True, log, {"prefs": save_prefs(**kw)}, ""

        if action == "reveal":
            if not hasattr(os, "startfile"):            # 只有 Windows 有
                raise CaptureError("当前平台不支持打开目录")
            ensure_raw_dirs()
            os.startfile(str(RAW_DIR))                  # noqa: S606 - 本地开发者工具
            return True, log, {"raw_dir": str(RAW_DIR)}, ""

        return False, log, None, "未知动作: %s" % action

    except CaptureError as exc:
        note("✗ %s" % exc, "bad")
        return False, log, None, str(exc)
    except Exception as exc:                            # noqa: BLE001
        note("✗ %s" % exc, "bad")
        return False, log, None, str(exc)
