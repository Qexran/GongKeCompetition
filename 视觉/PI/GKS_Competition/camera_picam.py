#!/usr/bin/env python3
# camera_picam.py — 树莓派 CSI 摄像头取流（libcamera），目前**只服务 GKS_Web 的预览**
#
# 与 camera.py / camera_lease.py 管的那两个 USB 摄像头（/dev/camera_main、/dev/camera_Qr）
# **完全无关**：
#   * 硬件上是 CSI 排线上那颗（本机 = OV5647 / 官方 Camera Module v1）；
#   * 走 libcamera + ISP，**不能**用 cv2.VideoCapture 打开 —— 内核给的是
#     /dev/videoN(unicam) 原始 Bayer 节点（本机 video4, ID_V4L_PRODUCT=unicam），
#     没有 ISP 管线，直读拿不到能显示的图；
#   * 所以这里用 `rpicam-vid` 出 MJPEG（它自己选 sensor mode、跑 ISP，按
#     /usr/share/libcamera/ipa/rpi/vc4/ov5647.json 调优），我们在 Python 侧按
#     JPEG 的 SOI/EOI 切帧。
#
# ⚠ 为什么不按 /dev/videoN 找设备：那编号是内核按探测顺序动态分配的，拔插/换口 USB
#   设备就会变（本机实测：camera_main 从 video0 漂到 video2，与 camera_Qr 整个对调）。
#   libcamera 则是按「哪颗传感器」枚举，所以这里用 `rpicam-*  --list-cameras` 找
#   **那颗非 UVC 的相机** —— 比设备号稳，也不需要为它建 udev 符号链接。
# ⚠ Pi 4 的 CSI 口**不支持热插拔**：插拔必须断电/重启，重插后由 config.txt 的
#   camera_auto_detect=1 自动重新加载 overlay（本模块不参与这件事）。
#
# 目前**刻意没有**接进 camera_lease / 任何识别任务 —— 它独立于 USB 那两颗，
# 不占「跨进程相机租约」，也不会跟识别任务抢设备。
#
# 台架自测（不启 Web 也能验）：
#   python3 camera_picam.py            # 列相机 + 抓帧自检 + 预览 5s 存一帧
#   python3 camera_picam.py 10 /tmp/a.jpg

import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
import urllib.request
import weakref

# ---- 预览取流参数（想改清晰度/开销就改这里）----
PICAM_WIDTH = 1280
PICAM_HEIGHT = 960
PICAM_FRAMERATE = 15
PICAM_JPEG_QUALITY = 80

# 自检抓帧的参数（小图，只为回答「能不能出图」）
PROBE_WIDTH = 640
PROBE_HEIGHT = 480

_SOI = b"\xff\xd8\xff"          # JPEG 起始标记
_EOI = b"\xff\xd9"              # JPEG 结束标记
_MAX_BUF = 8 * 1024 * 1024      # 切帧缓冲上限：一直切不出帧时别把内存吃光

# 相机索引缓存（见 csi_camera）
_CSI_CACHE = None

# `rpicam-hello --list-cameras` 的等待上限。⚠ 别调大：**相机被占用时它会一直阻塞**
# （libcamera 要等设备），而它跑在「发 HTTP 响应头之前」—— 调大就是把预览请求挂死。
_LIST_TIMEOUT = 6


def _bin(name):
    """在 PATH 里找 rpicam-* 可执行文件；找不到返回 None。"""
    return shutil.which(name)


def list_cameras():
    """`rpicam-hello --list-cameras` → [{'index', 'model', 'path'}]。

    ⚠ USB(UVC) 摄像头**不会**出现在这个列表里（rpicam-apps 按角色过滤掉了），
    所以列表里剩下的就都是 CSI 那颗。返回空列表 = CSI 口上没有相机。
    """
    exe = _bin("rpicam-hello") or _bin("libcamera-hello")
    if not exe:
        return []
    try:
        out = subprocess.run([exe, "--list-cameras"], capture_output=True,
                             text=True, timeout=_LIST_TIMEOUT)
    except (OSError, subprocess.SubprocessError):
        return []
    cams = []
    for line in (out.stdout or "").splitlines():
        # 形如: `0 : ov5647 [2592x1944 10-bit GBRG] (/base/soc/i2c0mux/i2c@1/ov5647@36)`
        if " : " not in line or "(" not in line:
            continue
        head, _, tail = line.partition(" : ")
        if not head.strip().isdigit():
            continue                      # 排除 `    Modes: 'SGBRG10_CSI2P' : ...` 这类行
        model = tail.split(" [", 1)[0].strip()
        path = tail[tail.find("(") + 1:tail.rfind(")")].strip()
        cams.append({"index": int(head.strip()), "model": model, "path": path})
    return cams


def csi_camera():
    """CSI 那颗相机的描述 dict（{'index','model','path'}）；没有则 None。

    ⚠ 结果会缓存（`_CSI_CACHE`）：解析要起一个子进程，而**相机被占用时
    `--list-cameras` 会一直阻塞** —— 它跑在打开预览的关键路径上，每次都重解析会把
    请求挂死（实测：连开两次预览时第二次 5 秒收不到任何字节）。索引会变的情况只有
    「CSI 相机被拔掉/换位置」，那条路由 `forget_camera()` 在打开失败时兜住。
    """
    global _CSI_CACHE
    if _CSI_CACHE is not None:
        return _CSI_CACHE
    cams = list_cameras()
    if not cams:
        return None
    pick = None
    for c in cams:                        # 双保险：明确排除 uvcvideo
        if "uvcvideo" not in c["path"]:
            pick = c
            break
    _CSI_CACHE = pick or cams[0]
    return _CSI_CACHE


def forget_camera():
    """丢掉上面的缓存 —— 打开失败时调用，下次重新解析（相机换过位就靠它）。"""
    global _CSI_CACHE
    _CSI_CACHE = None


class PicamStream:
    """rpicam-vid 子进程 + 后台切帧线程；get_frame() 取最新一帧 JPEG。

    生命周期：构造 = 起子进程；close() = 收子进程。**必须 close()**，
    否则 rpicam-vid 会一直占着 ISP（而且会被当成僵尸进程）。
    """

    def __init__(self, width=PICAM_WIDTH, height=PICAM_HEIGHT,
                 framerate=PICAM_FRAMERATE, quality=PICAM_JPEG_QUALITY,
                 awb_mode=None, awbgains=None, tag=None):
        cam = csi_camera()
        if cam is None:
            raise RuntimeError(
                "libcamera 没看到 CSI 摄像头（排线没插好 / 插错口 / 需断电重插后重启）")
        exe = _bin("rpicam-vid")
        if not exe:
            raise RuntimeError("找不到 rpicam-vid（未装 rpicam-apps）")

        self.camera = cam
        self.stop = threading.Event()
        # 由调用方（app.py）填成 HTTP 连接 —— _stop_* 靠 shutdown 踢掉推流循环，
        # 与本文件里的 _CameraStream 同一套约定。
        self.conn = None

        self._cv = threading.Condition()   # 保护 _frame / _seq / _last_frame_t
        self._frame = None
        self._seq = 0
        self._last_frame_t = 0.0           # 流里最近一次出帧的时刻（读线程打, 判活用）
        self._err = tempfile.TemporaryFile()

        self.cmd = [exe, "-t", "0", "--codec", "mjpeg", "-n",
                    "--camera", str(cam["index"]),
                    "--width", str(width), "--height", str(height),
                    "--framerate", str(framerate), "--quality", str(quality),
                    "-o", "-"]
        # 白平衡（2026-10-16 加, 只为"识别"那条路）: 色识别是 HSV 阈值, 对白平衡很敏感 ——
        # 主摄当年正是被固定成 6500K 才稳（见 CLAUDE.md）。预览那条路两个参数都不传
        # ⇒ 命令行与加这俩参数**逐字节相同**（默认行为不变）。
        # ⚠️ 固定增益要配 `--awb custom`（只给 --awbgains 在部分版本上会被 --wb 默认值盖掉）。
        if awbgains:
            self.cmd += ["--awb", "custom", "--awbgains", str(awbgains)]
        elif awb_mode and str(awb_mode).strip().lower() not in ("", "auto"):
            self.cmd += ["--awb", str(awb_mode).strip()]
        # bufsize=0 → stdout 是裸 FileIO，read(n) 只返回「当前可得」的字节，
        # 不会为了凑够 n 字节阻塞（BufferedReader.read 会，那就卡住切帧了）
        # tag（2026-10-16 加）: 传了就把它当 **argv[0]**（`executable=` 保证真正执行的还是
        # rpicam-vid）—— 于是这个子进程在 `ps`/`pgrep -a rpicam` 里一眼可认, 也是
        # reap_stale() 认出"这是我们这类流"的第一条判据。识别那条路传 OWN_TAG, 预览不传。
        argv0 = tag or exe
        self.proc = subprocess.Popen([argv0] + self.cmd[1:], executable=exe,
                                     stdout=subprocess.PIPE,
                                     stderr=self._err, bufsize=0)
        # 登记"本进程在用"（弱引用, 见 _LIVE_STREAMS）→ reap_stale 别碰它
        _LIVE_STREAMS[self.proc.pid] = weakref.ref(self)
        self._thread = threading.Thread(target=self._reader, daemon=True,
                                        name="picam-reader")
        self._thread.start()

    # ---------------- 后台：MJPEG → 一帧帧 JPEG ----------------
    def _reader(self):
        buf = bytearray()
        try:
            while not self.stop.is_set():
                try:
                    chunk = self.proc.stdout.read(8192)
                except (OSError, ValueError):
                    break                  # 子进程已收（管道被关）
                if not chunk:
                    break
                buf += chunk
                while True:
                    s = buf.find(_SOI)
                    if s < 0:
                        if len(buf) > 2:   # 留尾巴：SOI 可能被切在两个 chunk 之间
                            del buf[:-2]
                        break
                    e = buf.find(_EOI, s + 3)
                    if e < 0:
                        del buf[:s]        # 丢掉 SOI 之前的垃圾
                        if len(buf) > _MAX_BUF:
                            del buf[:-_MAX_BUF]
                        break
                    jpg = bytes(buf[s:e + 2])
                    del buf[:e + 2]
                    with self._cv:
                        self._frame = jpg
                        self._seq += 1
                        # ⚠️ **流自己的**出帧时刻（判活的唯一真相, 见 PicamSource.alive()）:
                        #    由读线程在真出一帧时打, 与"谁取过帧"无关 —— 预热完没人取的那段
                        #    也算"流活着"（那正是预热的意义）。
                        self._last_frame_t = time.time()
                        self._cv.notify_all()
        finally:
            with self._cv:
                self._cv.notify_all()      # 叫醒可能在 get_frame 里等的人

    def last_frame_t(self):
        """**流里**最近一次出帧的时刻（0.0 = 还没出过帧）。判活用, 见 PicamSource.alive()。

        ⚠️ 与"调用方最近取帧的时刻"是两回事: 后者在"预热好、待会儿用"的空窗里必然过期,
        拿它判活会把一段完全健康的流报成"中断"（2026-10-16 现场就是这么栽的）。
        """
        return self._last_frame_t

    def get_frame(self, last_seq=-1, timeout=2.0):
        """等一帧比 last_seq 新的；超时就把手上最新那帧给出去。

        返回 (jpeg_bytes | None, seq)。⚠ 超时**不**返回 None 是故意的：
        推流侧宁可重复发上一帧（浏览器那边看着就是掉帧），也别黑屏。
        """
        with self._cv:
            if self._seq == last_seq and not self.stop.is_set():
                self._cv.wait(timeout)
            return self._frame, self._seq

    def wait_ready(self, timeout=2.5):
        """等到出第一帧。判「这次真起来了」必须看帧、不能只看 Popen 成功 ——
        rpicam-vid 在相机被别的进程占着时会起来、报错、**立刻退出**。"""
        t0 = time.time()
        while time.time() - t0 < timeout:
            jpg, _ = self.get_frame(timeout=0.3)
            if jpg is not None:
                return True
            if not self.alive():
                return False
        return False

    def alive(self):
        return self.proc.poll() is None

    def stderr_text(self):
        """rpicam-vid 的 stderr（诊断用，取流失败时看这里）。"""
        try:
            self._err.seek(0)
            return self._err.read().decode("utf-8", "replace")
        except Exception:
            return ""

    def close(self):
        self.stop.set()
        p = self.proc
        if p is not None and p.poll() is None:
            try:
                p.terminate()
                p.wait(timeout=3)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
        if p is not None:
            # 摘掉登记: 从此它算"遗留"（close 没收干净时能被 reap_stale 兜住）
            _LIVE_STREAMS.pop(p.pid, None)
        with self._cv:
            self._cv.notify_all()


def _rpicam_holders():
    """谁在跑 rpicam-*（诊断用）: "1234 rpicam-vid -t 0 ..." 这种一行行拼起来。

    ⚠️ libcamera **独占**相机 —— 只要上面还挂着一个 rpicam-vid, 谁都别想再开
       （后到的只拿到 `failed to acquire camera`）。失败信息里带上它, 排障就不用猜。
    """
    try:
        out = subprocess.run(["pgrep", "-a", "rpicam"], capture_output=True,
                             text=True, timeout=2.0)
        lines = [ln.strip() for ln in (out.stdout or "").splitlines() if ln.strip()]
        return " / ".join(lines)[:160]
    except Exception:                                  # noqa: BLE001
        return ""


# ==================== 收掉「本进程该负责的」取流子进程（2026-10-16 加）====================
# ⚠️⚠️ 为什么必须有这一段: `rpicam-vid` 是**子进程**, 而 main.py 的两条重启都是
#    `os.execl` **换进程镜像**（serial_comm._exec_self）—— 换镜像**不会**带走子进程
#    （PID 与父进程都不变）。于是任何一次"重启前没走到 release_now()"都会在场上留一个
#    `-t 0`（**永不超时**）的 rpicam-vid, 它一直攥着 libcamera:
#      ① 链路故障自愈那条重启（handler 的 restart_process）—— 原来没释放就 exec（已补）;
#      ② 直接 kill -9 / 掉电 / 崩溃 —— 谁都来不及释放, 子进程被 init 收养（PPID=1）。
#    现象固定是那一句 `failed to acquire camera`, 而 `pgrep -a rpicam` 里那个 PID
#    谁也不认识（父进程早换过一轮了）—— 现场只能手工 pkill 才能救回来。
#    治法: 起流前 / 启动时 / 自检时, 把**自己留下的**收掉（判据见 _own_stream_pids）。
# ⚠️ 只认自己的, 绝不误伤:
#    · 网页那条 pi 预览是 **GKS_Web 进程**起的（PPID 对不上本进程、也不是孤儿）,
#      且参数是另一套（1280x960@15/80 vs 识别的 2592x1944@10/85）;
#    · 别人的 `rpicam-still` / 手工 `rpicam-vid` 形状不是"识别流"（见 _is_ident_stream_cmdline）;
#    · 本进程**正在用**的流（对象还活着）登记在 _LIVE_STREAMS 里, 一律跳过。
OWN_TAG = "rpicam-vid-gks"      # 识别流的 argv[0]（伪装, 见 PicamStream 的 tag 参数）

# 本进程**还活着**的取流子进程: {pid: weakref(留下它的那个 PicamStream)}。
# ⚠️ 用弱引用而不是裸 set: 有人 new 了流却把对象丢掉（没 close()）时, 弱引用随对象一起失效
#    → 那个子进程就重新变成"该负责的遗留", 会被 reap_stale 收掉（裸 set 会把它永久豁免）。
_LIVE_STREAMS = {}
_PROC = "/proc"                 # 只在 Linux 上有意义; 抽成变量 = 单测能喂假树（见 test_picam_reap.py）


def _pid_alive(pid):
    """探活。POSIX: `kill(pid, 0)` 是**纯探测**; 其它平台没有这个语义（Windows 的 os.kill
    是 TerminateProcess, 拿 0 去"探测"会把进程真干掉）→ 直接返回 False（"当它已经走了"）。
    ⚠️ 本模块的战场是树莓派（posix）; 这个分支只是为了让逻辑能在开发机上跑单测。"""
    if os.name != "posix":
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _proc_cmdline(pid):
    """→ 某个 PID 的命令行（空格分隔）。读不到（进程没了/权限/僵尸）→ None。"""
    try:
        with open("%s/%d/cmdline" % (_PROC, pid), "rb") as f:
            raw = f.read()
    except OSError:
        return None
    if not raw:
        return None                            # 内核线程/僵尸: cmdline 是空的
    return raw.replace(b"\x00", b" ").decode("utf-8", "replace").strip()


def _proc_ppid(pid):
    """→ 某个 PID 的父 PID（读 /proc/<pid>/stat 第 4 个字段）。读不到 → -1。

    ⚠️ comm 字段（括号里那段）**可能带空格和括号** → 必须从**最后一个 ')'** 之后切。
    """
    try:
        with open("%s/%d/stat" % (_PROC, pid), "rb") as f:
            data = f.read().decode("utf-8", "replace")
    except OSError:
        return -1
    tail = data.rpartition(")")[2].split()
    if len(tail) < 2:
        return -1
    try:
        # ⚠️ ')' 之后是 ` <state> <ppid> ...` —— **状态字占第一位**, ppid 是第二位。
        #    （这是 /proc/<pid>/stat 的第 4 个字段; 写成 tail[0] 会拿状态字去 int()，
        #      永远返回 -1 —— 那会让"换镜像留下的"判定整条失效。）
        return int(tail[1])
    except ValueError:
        return -1


def _is_ident_stream_cmdline(cmd):
    """命令行是不是「识别流那条 rpicam-vid」的形状（数字不参与判断）:

        rpicam-vid -t 0 --codec mjpeg -n --camera N --width W --height H
                   --framerate F --quality Q -o -   [--awb ...] [--awbgains ...]

    ⚠️ 与网页预览**同一个形状**（预览只是数字不同）—— 所以"这条是不是我们该负责的"
       靠**父子关系**补足（见 _own_stream_pids）, 不靠数字猜。
    """
    if "rpicam-vid" not in cmd:
        return False
    for tok in ("-t 0", "--codec mjpeg", "-n", "--camera", "--width", "--height",
                "--framerate", "--quality", "-o -"):
        if tok not in cmd:
            return False
    return True


def _own_stream_pids(keep_pid=None):
    """本进程**该负责**的取流子进程 PID（陈旧/自留的都算, 不含 live 的与 keep_pid）。"""
    me = os.getpid()
    out = []
    try:
        names = os.listdir(_PROC)
    except OSError:
        return out
    for name in names:
        if not name.isdigit():
            continue
        pid = int(name)
        if pid == keep_pid:
            continue
        ref = _LIVE_STREAMS.get(pid)
        if (ref is not None) and (ref() is not None):
            continue                           # 本进程正在用它（对象还活着）→ 不是"遗留"
        cmd = _proc_cmdline(pid)
        if not cmd:
            continue
        # ⚠️⚠️ 判据 = 「形状/标签」**且**「父进程已经不在了」, 两条缺一不可（2026-10-16 修）。
        #    只按标签（`OWN_TAG`）认是**错的**: 标签说明"这是本模块起的那类流", 但**说明不了
        #    "它没人用了"** —— 网页进程（GKS_Web）自己起的那条识别流**也**带标签, 而它在
        #    **另一个活着的进程**手里 ⇒ 一律杀掉 = 把人家正在用的流打断。
        #    现场（2026-10-16 06:30/06:31 日志, 每 20s 一条）: main.py 的看门狗把网页进程
        #    的识别流当"遗留"收掉 ⇒ 网页「靶标识别(pi)」跑到一半「取流中断」→ 退回主摄,
        #    看着就是"这个按钮时好时坏/一直失败"。
        #    父进程关系才是"没人要了"的证据:
        #      ① PPID = 本进程: "换镜像"留下的（os.execl 不改 PID, 所以它的父还叫这个名字）;
        #      ② PPID = 1: 父进程被 kill -9 / 崩掉 / 掉电后被 init 收养的孤儿。
        #    别的活进程（GKS_Web、别人的 python）当父的流**一律不碰** ✓。
        ppid = _proc_ppid(pid)
        if (ppid != me) and (ppid != 1):
            continue
        if cmd.startswith(OWN_TAG + " "):
            out.append(pid)                    # 带标签 + 没人要了 → 就是我们自己的遗留
            continue
        if _is_ident_stream_cmdline(cmd):
            out.append(pid)                    # 识别流的形状 + 没人要了（老固件留下的没标签也认）
    return out


def _kill_pid(pid, term_wait=1.5):
    """先 SIGTERM, 等一会儿, 还在就 SIGKILL。返回 True = 已经不在了。"""
    import signal
    # ⚠️ SIGKILL 不是所有平台都有（Windows 的 signal 模块里就没有）→ 取不到就退回 SIGTERM。
    #    树莓派上两者都在; 这个兜底只是让逻辑能在开发机上被单测。
    kill_sig = getattr(signal, "SIGKILL", signal.SIGTERM)
    for sig, wait in ((signal.SIGTERM, term_wait), (kill_sig, 0.5)):
        try:
            os.kill(pid, sig)
        except ProcessLookupError:
            return True                        # 已经不在了
        except OSError:
            # 发不出去 —— 可能是权限（EPERM, 真收不掉）, 也可能进程刚没了（各平台报的错不一样:
            # Windows 上"死进程"报 WinError 87 而不是 ProcessLookupError）→ 探一下再下结论。
            return not _pid_alive(pid)
        t0 = time.time()
        while time.time() - t0 < wait:
            if not _pid_alive(pid):
                return True
            time.sleep(0.05)
    return not _pid_alive(pid)


def reap_stale(verbose=True):
    """收掉「上次运行遗留」的识别流子进程。返回收掉几个。

    ⚠️ 幂等且快（纯 /proc 扫描, 不开子进程）——启动时、每次起流前、自检前都可以调;
    平时**一个都不匹配**（本进程在用的流登记在 _LIVE_STREAMS 里）。
    """
    now = time.time()
    n = 0
    for pid in _own_stream_pids():
        if now < _FAILED_KILLS.get(pid, 0.0):
            continue                           # 收不掉的（见下）别每 10s 刷一条日志
        cmd = (_proc_cmdline(pid) or "")[:90]
        if _kill_pid(pid):
            n += 1
            if verbose:
                print(f"[INFO] pi摄像头: 收掉上次运行遗留的取流进程 pid={pid}（{cmd}）")
        else:
            # ⚠️ 收不掉就是**真收不掉**: 常见是①进程卡在不可中断状态(D, libcamera 里等设备),
            #    ②不是本用户的进程（手工 `sudo python3 main.py` 起的流是 root 的）。
            #    这时**必须把命令行和 pid 打出来** —— 排障只有这一条线索（2026-10-16 现场
            #    就是只报了 pid=4547 没报命令行, 结果查不出那是谁）。
            _FAILED_KILLS[pid] = now + 60.0
            if verbose:
                print(f"[WARN] pi摄像头: 收不掉遗留的取流进程 pid={pid}（{cmd}）—— "
                      f"可能是不可中断状态(D) 或别的用户的进程; 手工: sudo kill -9 {pid}"
                      f"（60s 内不再重复报）")
    return n


# 收不掉的 PID → 下次再尝试的时刻（避免看门狗每 10s 刷同一条 WARN）
_FAILED_KILLS = {}


# ---- 跨进程/同进程让出 CSI（GKS_Web 那个 pi 预览，2026-10-16 加）----
# 与 camera_lease 里那套**完全同源**（那边管 USB 两颗, 这边管 CSI 一颗）:
_GKS_WEB_ABORT_URL = "http://127.0.0.1:9000/abort"
_ABORT_HTTP_TIMEOUT_S = 0.5
_ABORT_SETTLE_S = 0.3
_ABORT_HOOK = None


def set_abort_hook(fn):
    """注册同进程的「让出 CSI」钩子: fn(cam: str) -> bool（True = 已请对方停推流）。

    只该由 GKS_Web 的 app.py 在启动时注册一次（它与 pi 预览在同一个进程里）。
    ⚠️ 理由同 camera_lease.set_abort_hook: 在 GKS_Web 自己进程里 HTTP 打自己, 会顺手
       把页面上正在跑的识别一起取消（`/abort` 会 set `_scan_abort`）—— 钩子只停推流。
    """
    global _ABORT_HOOK
    _ABORT_HOOK = fn


def _abort_gks_web_pi():
    """请 GKS_Web 停掉 pi 预览推流。True = 请求确实送到了（值得等一下再重试打开）。

    ⚠️ 它**够不着 main.py 进程**里的 PICAM 持有器（没有反向通道）—— 那种占用只能靠
       空闲窗口自己到期（`PICAM_IDLE_TIMEOUT_S` × 预热次数）。但那种情况下重试也是白等,
       所以这里返回 False, 调用方立刻收手、**退回主摄**。
    ⚠️ 任何异常都静默吞掉: 比赛时 GKS_Web 可能压根没起, 那是 normal case 而不是错误。
    """
    if _ABORT_HOOK is not None:
        try:
            if _ABORT_HOOK("pi"):
                print("[INFO] 已请本进程停掉 pi 预览推流（进程内钩子）")
                return True
            return False
        except Exception:                              # noqa: BLE001
            return False
    try:
        with urllib.request.urlopen(f"{_GKS_WEB_ABORT_URL}?cam=pi",
                                    timeout=_ABORT_HTTP_TIMEOUT_S) as resp:
            resp.read(64)                              # 读完响应体, 别让连接半开着
    except Exception:
        return False
    print("[INFO] 已请 GKS_Web 停掉 pi 预览推流（跨进程抢 CSI: 有人开着网页预览）")
    return True


def probe(timeout=15.0):
    """自检：列出 CSI 相机 + 真的抓一帧。返回 (ok: bool, 说明: str)。

    说明在 ok 时当「分辨率/型号」显示、失败时当「原因」显示（GKS_Web 的
    摄像头测试按钮直接把这两句话贴出来）。
    """
    # ⚠️ 先清掉**自己**上次运行遗留的取流进程（2026-10-16 加）: 重启/掉电留下的
    #    `rpicam-vid` 会一直占着 libcamera, 那时"自检"永远失败、而人只能手工 pkill。
    #    清完再抓帧, 这个按钮自己就能把这种局面救回来。
    try:
        reap_stale()
    except Exception as e:                             # noqa: BLE001
        print(f"[WARN] pi摄像头: 清理遗留取流进程失败（继续自检）: {e}")

    # ⚠️⚠️ 再把**本进程正热着的识别流**收掉（2026-10-16 加）。libcamera 独占: 刚点过
    #    「靶标识别(pi)」再点自检, 那颗 `rpicam-vid-gks` 还在（识别流是**故意留着**的:
    #    空闲看门狗要等 PICAM_IDLE_TIMEOUT_S×预热次数 才关), 自检的 `rpicam-still`
    #    只能拿到 `failed to acquire camera` —— 失败信息里那行 `rpicam-vid-gks …`
    #    就是**我们自己**。收掉是安全的: 下次识别按需重起（~3s, 那条路本来就要等）。
    #    ⚠️ 只收 `PICAM`（本进程的识别流）, **不动**网页那个 pi 预览流 ——
    #       后者由 `_check_picam` 的"预览中直接判 OK"分支管, 停它会把页面上的 <img> 打断。
    try:
        PICAM.release_now()
    except Exception as e:                             # noqa: BLE001
        print(f"[WARN] pi摄像头: 自检前释放识别流失败（继续自检）: {e}")

    cam = csi_camera()
    if cam is None:
        return False, "libcamera 没看到 CSI 摄像头（排线/口/供电，需断电重插后重启）"
    exe = _bin("rpicam-still")
    if not exe:
        return False, "找不到 rpicam-still（未装 rpicam-apps）"

    fd, path = tempfile.mkstemp(suffix=".jpg", prefix="picam_probe_")
    os.close(fd)
    try:
        r = subprocess.run([exe, "-n", "-t", "500", "--camera", str(cam["index"]),
                            "--width", str(PROBE_WIDTH), "--height", str(PROBE_HEIGHT),
                            "-o", path],
                           capture_output=True, text=True, timeout=timeout)
        size = os.path.getsize(path) if os.path.exists(path) else 0
        if size > 1024:
            with open(path, "rb") as f:
                head = f.read(3)
            if head == _SOI:
                return True, "%s %dx%d" % (cam["model"], PROBE_WIDTH, PROBE_HEIGHT)
            return False, "抓到的文件不是 JPEG（相机输出异常）"
        tail = [ln for ln in (r.stderr or "").strip().splitlines() if ln.strip()]
        reason = tail[-1] if tail else ""
        if "acquire" in reason:
            # libcamera 独占: 相机**没坏**, 是被人占着（网页预览没关 / 有个 rpicam 没退干净 /
            # main.py 刚用完还在空闲窗内）。⚠️ 上面已经先 reap_stale() 收过**自己**的遗留了,
            # 所以这里剩下的多半是别人（网页预览）—— 把"谁占着"直接写进给界面看的那句话里。
            who = _rpicam_holders()
            reason += (" ← 相机被占用（libcamera 独占）: 关掉网页的 pi 预览; 或等一下再试"
                       f"（刚被识别任务用过时最多几十秒; 预热次数填大时更久）"
                       + (f"; 现在挂着: {who}" if who else
                          "; `ps aux | grep rpicam` 看有没有退不掉的 rpicam-vid"))
        return False, ("抓帧失败" + ("：" + reason if reason else ""))
    except subprocess.TimeoutExpired:
        return False, "抓帧超时（相机没出图）"
    except Exception as e:
        return False, "抓帧异常: %s" % e
    finally:
        try:
            os.unlink(path)
        except OSError:
            pass


# ==================== 识别用的持有器（2026-10-16 加, 服务"平移射靶"看靶）====================
# 与 camera_lease.py 的租约是**同一套思路, 但两台设备毫无关系**:
#   · 租约管的是 /dev/camera_main 与 /dev/camera_Qr 两颗 USB 摄像头（单槽, 收一颗会放另一颗）;
#   · 这里管的是 CSI 那颗（libcamera 子进程）—— 与它们**不冲突** ⇒ CSI 可以和主摄同时热着。
#     所以"预热 pi"不会把主摄的租约顶掉（见 camera_lease.LEASE.warm 里那条 release_now）。
# 为什么需要它（而不是每次现场起一个 PicamStream）:
#   ① `rpicam-vid` 起流实测 ~2s（同 USB 开相机那条账）—— 单片机那条「预热摄像头」积木
#      就是为了把这 2s 花在车移动/摆臂的空档里, 所以必须有个"提前开好、留着"的地方;
#   ② 必须**持续读到帧**: 不读的话 MJPEG 管道会积压、"要用时抓一帧"拿到的是**几秒前**的画面
#      （同 V4L2 环形缓冲那条铁律）—— `PicamStream` 的读线程一直在排空管道, 这里只管取最新;
#   ③ 空闲必须自动关: 发了预热却没跟进识别（路线改了/脚本被中止）就**永久占着 ISP**,
#      Web 预览从此打不开且无人报错 —— 纪律同 LEASE_IDLE_TIMEOUT_S / BT_EDIT_IDLE_TIMEOUT_MS。
#
# ⚠ 与 GKS_Web 预览的关系: 预览是**另一个进程**里的 rpicam-vid, libcamera 独占 ⇒ 有人在看
#   `?cam=pi` 预览时这里会起流失败。那不是故障, 失败会被吞掉并**退回主摄**（本模块只打日志）;
#   真要在场地上用 CSI 看靶, 先把那个预览关掉。
_FAIL_BACKOFF_S = 10.0        # 起流失败后多久不再去撞（防"每步都白等 2s"; 台架调试也够快重试）


def _cfg(name, default):
    """从 config 现取一个旋钮（延迟 import: 本模块被 GKS_Web 在启动时 import, 别拉重依赖）。"""
    try:
        import config
        return getattr(config, name, default)
    except Exception:                                  # noqa: BLE001
        return default


def _clamp_uses(uses):
    """同 camera_lease._clamp_uses: ≤1 → 1, 上限 PICAM_USES_MAX（窗口 = uses × 空闲超时）。"""
    try:
        n = int(uses) if uses is not None else 1
    except (TypeError, ValueError):
        n = 1
    return max(1, min(n, int(_cfg("PICAM_USES_MAX", 10))))


def rotate_frame(img, deg):
    """按 `config.PICAM_ROTATE` 把交付给识别的画面转正（**只服务识别那条路**）。

    ⚠️ 那颗 CSI 是**侧装**的（现场: 不转画面就是躺的）—— 90 = 顺时针 90°。
    ⚠️ 转 90/270 会**把画面的宽高换个个儿**（2592×1944 → **1944×2592**），于是:
       · 检测/流帧回传给单片机的 cx/cy 落在 **1944×2592** 那个坐标系里（与主摄那套不同）;
       · 单片机的瞄准常数 tc_cx_aim_pi/tc_cy_aim_pi 就在这个坐标系里标定;
       · 改这个角度 = 换坐标系 ⇒ 那对常数要重标（见 config.py 那段注释）。
    ⚠️ 侧装还有个反直觉的后果（2026-10-16 踩了两轮）: **世界水平方向 = 画面高** ⇒ 三靶
       并排那排能不能全进画面**只看取流的高**; 而 rpicam 是按请求挑**传感器模式**的 ——
       请求 1920×1080 或 1440×1080 都会落到 1920×1080 那档**窄视场**模式（1440×1080 实测
       靶盘坐标逐像素不变 = 白改）⇒ 现在取 2592×1944 原生满分辨率（帧率随之降到 10）。
    ⚠️ 网页那个 pi 预览走的是 MJPEG 直通（`PicamStream.get_frame`），**不经过这里、不转**
       —— 刻度/像素别拿预览去量。
    """
    import cv2
    if img is None:
        return None
    try:
        d = int(deg) % 360
    except (TypeError, ValueError):
        d = 0
    if d == 90:
        return cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    if d == 180:
        return cv2.rotate(img, cv2.ROTATE_180)
    if d == 270:
        return cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE)
    return img


class _PicamHandle:
    """任务侧句柄（鸭子类型同 camera_lease.LeaseHandle）: read()/alive()/release()。

    ⚠️ `release()` **不关设备**（只续期）—— 语义与相机租约一致: 留着给下一个任务用,
    真正关它的是空闲看门狗 `picam_watchdog`。
    """

    def __init__(self, src):
        self._src = src

    cam_is_pi = True                                   # 任务侧据此写回帧的 status bit7

    def read(self, timeout=0.0):
        return self._src.read(timeout)

    def alive(self):
        return self._src.alive()

    def release(self):
        self._src.touch()


class PicamSource:
    """预热/持有一路 CSI 取流, 给识别任务按需取最新帧。模块级单例 = 下面的 PICAM。"""

    def __init__(self):
        self._lock = threading.Lock()
        self._stream = None
        self._warm_thread = None
        self._uses_total = 1
        self._uses_left = 1
        self._last_use = 0.0
        # ⚠️ 判活**不用**"我们取过帧没有" —— 那个时刻在"预热好、还没人用"的空窗里必然过期。
        #    真相在 `PicamStream.last_frame_t()`（读线程出帧时打）, 见 alive()。
        self._seq = -1
        self._bgr = None
        self._bgr_seq = -1
        self._fail_until = 0.0          # 失败退避截止（戳 _FAIL_BACKOFF_S）

    # ---- 能力/状态 ----
    def use_pi(self):
        """这台机器**有能力**用 CSI 吗（相机在 + rpicam-apps 在）。不看退避, 只看硬件/软件。"""
        return (csi_camera() is not None) and (_bin("rpicam-vid") is not None)

    def available(self):
        """现在**值不值得**去试（有能力 且 不在失败退避期）。"""
        return self.use_pi() and (time.time() >= self._fail_until)

    def ready(self):
        st = self._stream
        return (st is not None) and st.alive() and (st.get_frame(last_seq=-1, timeout=0.0)[0] is not None)

    def alive(self):
        """流还活着吗（子进程在 + **流里**最近还在出新帧）。掉了 = 调用方该退回主摄。

        ⚠️⚠️ 判"还有没有新帧"必须看**流自己的时刻**（`PicamStream.last_frame_t()`, 读线程在
        出帧时打的），**不能**看"我们消费过没有"（2026-10-16 修）。原来这里读的是
        `self._last_frame`, 而它只在 `read()` 里刷新 —— 于是**预热好了、还没人用**的那段
        （正是预热的意义所在！）判据恒为假:
          现场 05:28:07「pi摄像头(CSI) 预热完成」→ 05:28:10 `"6,12…,1"` 一进门
          `alive()` → False → 「pi(CSI) 取流中断 → 交给主摄重试」; 05:28:14 的 `"7,0,1"`
          同样「中断（不再出帧）」→ 又退回主摄。**而流一直是好的**（读线程每秒 20 帧）。
        """
        st = self._stream
        if st is None or not st.alive():
            return False
        return (time.time() - st.last_frame_t()) < float(_cfg("PICAM_STALE_S", 1.5))

    def idle_limit_s(self):
        """空闲多久算"没人用了"（预热次数预算用完了就回 1 倍窗口）。"""
        total = self._uses_total if self._uses_left > 0 else 1
        return float(_cfg("PICAM_IDLE_TIMEOUT_S", 15.0)) * max(1, total)

    def touch(self):
        """只续期（"这台还在用"）。⚠️ 别在这里扣 `_uses_left` —— 那是**每次 acquire** 扣一次
        的量（同 camera_lease.LEASE.acquire）; 放在这里会被"每帧 read"扣光, 窗口立刻退回 1 倍。"""
        self._last_use = time.time()

    def note_failure(self):
        self._fail_until = time.time() + _FAIL_BACKOFF_S

    # ---- 取帧 ----
    def _decode(self, jpg):
        import numpy as np
        import cv2

        img = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        return rotate_frame(img, _cfg("PICAM_ROTATE", 0))

    def read(self, timeout=0.0):
        """→ (ok, BGR 帧)。**只给最新帧、绝不给陈帧**: 摘不到新帧就 (False, None)。

        ⚠️ 陈帧判据是"**流里**多久没新帧"（`st.last_frame_t()`）而不是"手上的帧多久了"、
          也不是"我们多久没取过" —— 子进程半死（管道还在、不再出图）时, 不判就是把**同一张
          老图**一路喂给检测器（检测器会稳稳地报同一个坐标, 单片机那边看着像"环没起作用"）;
          反过来, 判"我们取过没有"会把"预热好还没人用"误判成陈帧（见 alive() 的注释）。
        """
        st = self._stream
        if st is None:
            return False, None
        jpg, seq = st.get_frame(last_seq=self._seq, timeout=timeout)
        if jpg is None:
            return False, None
        now = time.time()
        if seq != self._seq:
            self._seq = seq
            self._bgr_seq = -1                         # 有新帧 → 下面重解码
        if (now - st.last_frame_t()) > float(_cfg("PICAM_STALE_S", 1.5)):
            return False, None
        if (self._bgr is None) or (self._bgr_seq != self._seq):
            try:
                self._bgr = self._decode(jpg)
            except Exception as e:                     # noqa: BLE001
                print(f"[WARN] pi摄像头解码失败: {type(e).__name__}: {e}")
                self._bgr = None
            self._bgr_seq = self._seq
        if self._bgr is None:
            return False, None
        self.touch()
        return True, self._bgr

    # ---- 起流/预热 ----
    def warm(self, uses=1):
        """**非阻塞**: 起后台线程把 CSI 开好并持续缓存最新帧（幂等; 已在跑就只续期）。

        ⚠️ 必须立刻返回 —— 它由 handler 在**串口接收线程**里内联调（"C,...,1"）, 在那里等
           2s 会把心跳一起压住（单片机 8s 判 DISCONNECTED）。同 camera_lease.LEASE.warm。
        """
        uses_n = _clamp_uses(uses)
        with self._lock:
            self._last_use = time.time()
            self._uses_total = uses_n
            self._uses_left = uses_n
            thread = self._warm_thread
            # ⚠️ 已经热着（子进程在跑）就**只续期、直接回 True** —— 退避闸门只该拦"要不要
            #    去起一条新流", 不该拦"这台已经在跑了"（2026-10-16 加: 退避期内续一次预热
            #    会拿到 False, 调用方以为这台不行而退回主摄, 而它其实帧都在手上）。
            if (self._stream is not None) and self._stream.alive():
                return True
            if not self.available():
                return False                           # 退避期/没相机: 不发车, 调用方自己退回
            if (thread is None) or (not thread.is_alive()):
                self._warm_thread = threading.Thread(target=self._warm_worker,
                                                     name="picam-warm", daemon=True)
                self._warm_thread.start()
        return True

    def _warm_worker(self):
        try:
            self._open_now()
        except Exception as e:                         # noqa: BLE001
            msg = str(e)
            hint = ""
            if "acquire" in msg:
                who = _rpicam_holders()
                hint = ("（libcamera 独占: 相机被占用 —— 已先试过①收掉自己上次运行遗留的取流"
                        "进程 ②请 GKS_Web 让出 pi 预览; 还失败就看看是不是网页预览没关"
                        + (f"; 现在挂着: {who}" if who else
                           "; `ps aux | grep rpicam` 看谁占着") + "）")
            print(f"[WARN] pi摄像头(CSI) 预热失败: {type(e).__name__}: {msg}{hint}")
            self.note_failure()

    def _new_stream(self):
        return PicamStream(width=int(_cfg("PICAM_TARGET_WIDTH", 2592)),
                           height=int(_cfg("PICAM_TARGET_HEIGHT", 1944)),
                           framerate=int(_cfg("PICAM_TARGET_FPS", 10)),
                           quality=int(_cfg("PICAM_TARGET_QUALITY", 85)),
                           awb_mode=_cfg("PICAM_AWB_MODE", "auto"),
                           awbgains=_cfg("PICAM_AWB_GAINS", ""),
                           tag=OWN_TAG)            # 带标签 → ps/pgrep 里可认、reap_stale 认得出

    def _open_now(self):
        """起流 → 等出第一帧 → **丢掉收敛期的几帧**（AE/AWB 还没稳, 同 USB 那 15 帧的道理）。

        ⚠️ 失败时按顺序试两种自救, 每次都重试一次:
          ① **收掉自己上次运行遗留的取流进程**（换镜像/kill -9 留下的 `rpicam-vid`,
             见 reap_stale 的注释）—— 这是我们自己的锅, 收掉就好了, 不用等谁;
          ② **请 GKS_Web 让出它的 pi 预览**（另一个进程, 够不着就作罢）—— 照抄
             camera_lease 的 `_open_with_abort_retry`（libcamera 独占的解法就这一个）。
        够不着 GKS_Web 时（它没跑 / 占着的是别的进程）就不白等, 直接抛给调用方退回主摄。
        """
        err = None
        abort_asked = False
        reap_done = False
        for _attempt in range(3):                      # 最多三次: 原样 / 清遗留后 / 请预览让位后
            try:
                st = self._start_stream()
            except Exception as e:                     # noqa: BLE001
                err, st = e, None
            if st is not None:
                self._adopt(st)
                return
            forget_camera()                            # 索引也可能变了（相机拔插过）
            msg = str(err or "")
            if "acquire" not in msg:
                break                                  # 不是"被占用"（没帧/找不到相机）: 重试没意义
            if not reap_done:                          # ① 先清自己的遗留
                reap_done = True
                if reap_stale():
                    time.sleep(0.3)                    # 等 libcamera 松手（同 _ABORT_SETTLE_S 的量级）
                    continue
            if not abort_asked and _abort_gks_web_pi():  # ② 再请 GKS_Web 让位
                abort_asked = True
                time.sleep(_ABORT_SETTLE_S)
                continue
            break
        raise RuntimeError(str(err) if err is not None else "起流失败")

    def _start_stream(self):
        """真去起一个流并等它出帧、丢够收敛帧。失败抛异常（调用方负责重试/收尾）。"""
        st = self._new_stream()
        try:
            if not st.wait_ready(timeout=float(_cfg("PICAM_STREAM_UP_S", 3.0))):
                raise RuntimeError("起流失败（没有帧）: "
                                   + (st.stderr_text()[-160:].strip() or "无 stderr"))
            n = int(_cfg("PICAM_WARM_FRAMES", 15))
            last = -1
            got = 0
            t_end = time.time() + max(1.0, 0.4 * n)
            while (got < n) and (time.time() < t_end) and (not st.stop.is_set()):
                jpg, seq = st.get_frame(last_seq=last, timeout=0.5)
                if seq != last:
                    last = seq
                    got += 1
                if not st.alive():
                    raise RuntimeError("起流中途退出（相机被占用/掉线?）: "
                                       + (st.stderr_text()[-160:].strip() or "无 stderr"))
        except Exception:
            try:
                st.close()                             # 失败立刻收掉, 别把相机攥在手里
            except Exception:                          # noqa: BLE001
                pass
            raise
        return st

    def _adopt(self, st):
        """把新流换上、收掉旧流（旧的要真 terminate, 否则它一直占着 ISP）。"""
        with self._lock:
            old = self._stream
            self._stream = st
            self._seq = -1
            self._bgr = None
            self._bgr_seq = -1
            # ⚠️ 起流成功 = 这一路**现在**是好的 → 立刻清掉失败退避（2026-10-16 加）。
            #    不退避的话会出现"流刚起好、接下来 10s 的 acquire 全被退避挡掉"（现场:
            #    `"6"` 失败记了退避 → `_open_now` 自愈把流起好 → `"7"` 却被退避挡回主摄）。
            self._fail_until = 0.0
        if old is not None:
            try:
                old.close()
            except Exception:                          # noqa: BLE001
                pass
        print("[INFO] pi摄像头(CSI) 预热完成: %s %dx%d@%dfps（交付前转 %s°）"
              % (st.camera.get("model", "?"),
                 int(_cfg("PICAM_TARGET_WIDTH", 2592)),
                 int(_cfg("PICAM_TARGET_HEIGHT", 1944)),
                 int(_cfg("PICAM_TARGET_FPS", 10)),
                 int(_cfg("PICAM_ROTATE", 0))))

    def acquire(self, wait_s):
        """等到 ready（或超时）→ 句柄; 拿不到就 None（调用方据此**立刻退回主摄**）。

        ⚠️ 这里**不重试**: 试不试、试几次由调用方按单片机给的预算决定（一次射靶步里
           重试 CSI 只会把退回主摄那次的时间吃光）。冷启动（没预热过）会就地起流,
           所以 wait_s 至少要给得住 ~2s。
        """
        # ① **现成的 ready 流优先**（2026-10-16 加）—— 这一步必须排在"能力/退避"判定**之前**:
        #    下面那道 `available()` 治的是"去撞一个起不来的相机", 而这里帧已经在手上了,
        #    没有任何理由拒绝。不这么写就会出这种现场（2026-10-16 实测）:
        #      ① `"6"` 的 CSI 尝试失败（相机那时真被占着）→ `note_failure()` 记 10s 退避,
        #         同时 `_open_now` 的自愈（收遗留+重试）在 1s 后**真把流起好了**;
        #      ② 接着 `"7,0,1"` 进来 → `available()` 还在退避期 → 立刻 None →
        #         日志「pi(CSI) 起流失败」→ 退回主摄 —— **而相机明明是热的**
        #         （用户现象: "跑完这一轮, 模拟器里测 pi 又能看到画面"）。
        #    ⚠️ `ready()` 不碰 `use_pi()`（那条会去跑 `rpicam-hello --list-cameras`,
        #       相机被自己占着时它会阻塞到超时）, 所以这条快路径也顺带躲开了那个坑。
        if self.ready():
            self.touch()
            with self._lock:                           # 每次 acquire 扣一档（窗口 = uses × 超时）
                if self._uses_left > 0:
                    self._uses_left -= 1
            return _PicamHandle(self)

        t_end = time.time() + max(0.0, float(wait_s))
        if not self.available():
            # 没能力（没插/没 rpicam）或在失败退避期 → **立刻**回 None, 别白等满 wait_s。
            # 这条是"退回主摄要快"的关键: 射靶保持档每 5s 重发一次 "7", 每次白等 3s
            # 就是每秒都在丢帧（单片机那边看着像流在抽）。
            return None
        if self._stream is None:
            self.warm(1)                               # 冷启动: 后台起流, 下面边等边看
        while time.time() < t_end:
            if self.ready():
                self.touch()
                with self._lock:                       # 每次 acquire 扣一档（窗口 = uses × 超时）
                    if self._uses_left > 0:
                        self._uses_left -= 1
                return _PicamHandle(self)
            # ⚠️ 这里用"流还在不在"判早退, **不再**调 `use_pi()`（2026-10-16 改）: 那条会跑
            #    `rpicam-hello --list-cameras`, 而**我们自己正占着相机时它会阻塞到超时**
            #    （见 csi_camera 的注释）→ 冷启动那次 acquire 会被它吃掉一大截预算。
            #    要判"硬件/软件就没有"的话, 下面 warm() 都没起来、上面 ready() 一直是 False,
            #    到 t_end 自然返回 None, 语义一样。
            time.sleep(0.1)
        self.note_failure()                            # 等不到 = 这一路不行, 退避一会儿
        return None

    # ---- 关/收 ----
    def close(self):
        """真关（terminate rpicam-vid）。⚠️ 不关的话它一直占着 ISP + 留僵尸进程。"""
        with self._lock:
            st, self._stream = self._stream, None
            self._seq = -1
            self._bgr = None
            self._bgr_seq = -1
        if st is not None:
            try:
                st.close()
            except Exception as e:                     # noqa: BLE001
                print(f"[WARN] pi摄像头关闭异常: {type(e).__name__}: {e}")

    def release_now(self):
        """立刻交还（"end"/"r" 重启主程序前调, 同 camera_lease 的口径）。"""
        if self._stream is not None:
            self.close()

    def reap(self):
        """看门狗一拍: 子进程死了 / 空闲超时 → 收掉。"""
        st = self._stream
        if st is None:
            return
        if not st.alive():
            print("[INFO] pi摄像头(CSI) 释放: 子进程已退出")
            self.close()
            return
        idle = time.time() - self._last_use
        if idle > self.idle_limit_s():
            print("[INFO] pi摄像头(CSI) 释放: 空闲超过 %.0fs" % self.idle_limit_s())
            self.close()

PICAM = PicamSource()


def acquire_picam(wait_s):
    """给识别任务用的一行入口: 拿到句柄或 None（None = 调用方退回 USB 主摄）。"""
    return PICAM.acquire(wait_s)


def picam_watchdog():
    """1s 一拍的守护线程体（main.py 起它, 与 camera_lease.lease_watchdog 并列）。

    ⚠️ 每 10 拍顺带 `reap_stale()`（2026-10-16 加）: 链路上的自愈重启（exec 换镜像）与
    kill -9 都会留下取流子进程, 有个周期性兜底就不必等"下次起流才发现相机被自己占着"。
    平时它一个都不匹配（本进程在用的流登记在 _LIVE_STREAMS 里）, 代价是扫一遍 /proc。
    """
    n = 0
    while True:
        time.sleep(1.0)
        n += 1
        try:
            PICAM.reap()
            if (n % 10) == 0:
                reap_stale()
        except Exception as e:                         # noqa: BLE001
            print(f"[WARN] pi摄像头看门狗异常: {type(e).__name__}: {e}")


# ---------------- 台架自测入口 ----------------
def _main(argv):
    seconds = float(argv[1]) if len(argv) > 1 else 5.0
    out = argv[2] if len(argv) > 2 else "/tmp/picam_test.jpg"

    print("== rpicam 列表 ==")
    for c in list_cameras():
        print("   #%d %s  (%s)" % (c["index"], c["model"], c["path"]))
    print("\n== 抓帧自检 ==")
    ok, info = probe()
    print("   %s: %s" % ("OK" if ok else "FAIL", info))
    if not ok:
        return 1

    print("\n== 预览 %.1fs ==" % seconds)
    st = PicamStream(tag=OWN_TAG)   # 带标签: 这个台架自检流万一没收干净, 也能被 reap_stale 认出来
    try:
        t0 = time.time()
        n = last = -1
        while time.time() - t0 < seconds:
            jpg, last = st.get_frame(last_seq=last, timeout=2.0)
            if jpg is not None:
                n = last
        print("   收到 %d 帧（约 %.1f fps），相机=%s，子进程存活=%s"
              % (n + 1, n / max(seconds, 0.01), st.camera["model"], st.alive()))
        jpg, _ = st.get_frame(timeout=0.1)
        if jpg:
            with open(out, "wb") as f:
                f.write(jpg)
            print("   最后一帧已存: %s (%d 字节)" % (out, len(jpg)))
    finally:
        st.close()
    return 0


if __name__ == "__main__":
    sys.exit(_main(sys.argv))
