# -*- coding: utf-8 -*-
"""功能自测: camera_picam 的两块"错了也不报错"的逻辑 ——
  ① **收掉自己遗留的取流子进程**（reap_stale / _own_stream_pids）;
  ② **取流闸门**: 「已有热流」永远优先于「失败退避」（PicamSource.acquire / warm）。

为什么值得写: 这两块的错法都是**静默**的 ——
  · ①里的 /proc/<pid>/stat 字段位置取错、argv[0] 伪装写错 ⇒ 只会"永远清不掉那个占着相机的
    进程", 现场没人查得出来;
  · ②的先后顺序写反 ⇒ 相机明明热着、帧在手上, 却因为 10s 退避期被挡掉, 日志只报
    「pi(CSI) 起流失败」并退回主摄（2026-10-16 现场: 用户发现"跑完这一轮去模拟器里看 pi
    画面又是好的"）。
做法: ①喂**假 /proc 树**验选择逻辑 + 拿**真子进程**验真能收掉; ②拿桩流直接验闸门顺序。

用法:
    python3 test_picam_reap.py          # 树莓派
    python  test_picam_reap.py          # 开发机（Windows 也能跑: 假 /proc + os.kill 的
                                        #   SIGTERM = TerminateProcess; 只有"探活"那条分支不同）
"""
import importlib.util
import io
import os
import shutil
import subprocess
import sys
import tempfile
import time
import weakref

if hasattr(sys.stdout, "buffer"):        # 控制台是 GBK 时别在中文上炸
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("camera_picam",
                                              os.path.join(HERE, "camera_picam.py"))
cp = importlib.util.module_from_spec(spec)
spec.loader.exec_module(cp)          # 模块级只 import stdlib（cv2 在函数里）→ 哪儿都能加载

FAIL = []


def ok(cond, msg):
    print(("OK   " if cond else "FAIL ") + msg)
    if not cond:
        FAIL.append(msg)


# 现场那条（2026-10-16 用户日志）= 识别流: 1920x1080@20 q85
LEAKED = ("/usr/bin/rpicam-vid -t 0 --codec mjpeg -n --camera 0 --width 1920 "
          "--height 1080 --framerate 20 --quality 85 -o -")
# 网页 pi 预览那条（PicamStream 默认参数）= 1280x960@15 q80
PREVIEW = ("/usr/bin/rpicam-vid -t 0 --codec mjpeg -n --camera 0 --width 1280 "
           "--height 960 --framerate 15 --quality 80 -o -")
STILL = "/usr/bin/rpicam-still -n -t 500 --camera 0 --width 640 --height 480 -o /tmp/a.jpg"
HELLO = "/usr/bin/rpicam-hello --list-cameras"
TAGGED = ("rpicam-vid-gks -t 0 --codec mjpeg -n --camera 0 --width 1920 --height 1080 "
          "--framerate 20 --quality 85 -o -")

print("== ① 形状判据 _is_ident_stream_cmdline ==")
ok(cp._is_ident_stream_cmdline(LEAKED), "现场那条识别流应判为「识别流形状」")
ok(cp._is_ident_stream_cmdline(PREVIEW), "预览也是同一形状（数字不同, 形状判据不区分）")
ok(cp._is_ident_stream_cmdline(TAGGED), "带标签的也认")
ok(not cp._is_ident_stream_cmdline(STILL), "rpicam-still 不认")
ok(not cp._is_ident_stream_cmdline(HELLO), "rpicam-hello --list-cameras 不认")
ok(not cp._is_ident_stream_cmdline("/bin/bash -c echo rpicam-vid"), "沾点字面的 shell 行不认")

print("\n== ② 假 /proc 树: ppid 必须取对, 只认自己人 ==")
tmp = tempfile.mkdtemp(prefix="fakeproc_")
old_proc = cp._PROC
cp._PROC = tmp
ME = os.getpid()


def mk(pid, cmdline, ppid=None, comm="rpicam-vid"):
    d = os.path.join(tmp, str(pid))
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "cmdline"), "wb") as f:
        f.write(cmdline.replace(" ", "\x00").encode() + b"\x00")
    if ppid is not None:
        with open(os.path.join(d, "stat"), "w", encoding="utf-8") as f:
            f.write("%d (%s) S %d 1 1 0 -1 4194560 0 0 0 0 0 0 0 0 20 0 1 0 12345\n"
                    % (pid, comm, ppid))


try:
    mk(1101, LEAKED, ppid=ME)        # 换镜像留下的: 父还叫"本进程这个名字"
    mk(1102, LEAKED, ppid=1)         # kill -9 后被 init 收养的孤儿
    mk(1103, LEAKED, ppid=4242)      # 别人的同类进程
    mk(1104, PREVIEW, ppid=4242)     # 网页预览（GKS_Web 起的）→ 不动
    mk(1105, TAGGED, ppid=4242)      # ⚠️ 带标签但**父是另一个活进程** = 人家正在用 ⇒ 不收
    mk(1106, STILL, ppid=1)          # 自检工具: 形状不对, 不收
    mk(1107, LEAKED, ppid=1)         # 一会儿登记成"本进程在用" → 必须跳过
    mk(1108, PREVIEW, ppid=ME)       # 父是本进程 + 不在册 ⇒ 就是"我们留下的"（本进程起的
    #                                 流都登记在 _LIVE_STREAMS）→ 该收
    mk(1109, TAGGED, ppid=1)         # 带标签 + 父没了（孤儿）⇒ 我们的遗留 → 该收
    mk(1110, TAGGED, ppid=ME)        # 带标签 + 父是本进程（换镜像留下的）⇒ 该收

    ok(cp._proc_ppid(1101) == ME,
       "_proc_ppid 取到的是 ppid（不是状态字）: %s" % cp._proc_ppid(1101))
    ok(cp._proc_ppid(9999) == -1, "读不到 → -1")
    got = sorted(cp._own_stream_pids())
    ok(got == [1101, 1102, 1107, 1108, 1109, 1110],
       "该负责的 = 换镜像遗留/孤儿/父是本进程（含带标签的）: %s" % got)
    ok(1103 not in got, "别人的同类进程不动（PPID 既不是我也不是 1）")
    ok(1104 not in got, "网页预览（父是 GKS_Web）不动")
    ok(1105 not in got,
       "**带标签但父是另一个活进程（GKS_Web 正在用）→ 坚决不收**"
       "（2026-10-16 现场: 收了它 = 把网页正在跑的靶标识别(pi) 打断 → 「取流中断」）")
    ok(1106 not in got, "rpicam-still 不动")

    class _FakeStream:
        pass

    keeper_obj = _FakeStream()
    cp._LIVE_STREAMS[1107] = weakref.ref(keeper_obj)
    ok(1107 not in cp._own_stream_pids(), "本进程在用的流（在册且对象还活着）跳过")
    del keeper_obj
    ok(1107 in cp._own_stream_pids(),
       "对象被丢掉（没 close()）→ 弱引用失效 ⇒ 重新算'遗留'（裸 set 会永久豁免它）")
    cp._LIVE_STREAMS.pop(1107, None)
    ok(1107 not in cp._own_stream_pids(keep_pid=1107), "keep_pid 跳过")

    print("\n== ③ 真收一个: stale 被 kill, 在跑的自己人不被碰 ==")
    victim = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    keeper = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    cp._LIVE_STREAMS[keeper.pid] = weakref.ref(keeper)
    cp._PROC = os.path.join(tmp, "real")
    os.makedirs(cp._PROC, exist_ok=True)
    mk_v = os.path.join(cp._PROC, str(victim.pid))
    os.makedirs(mk_v, exist_ok=True)
    with open(os.path.join(mk_v, "cmdline"), "wb") as f:
        f.write(LEAKED.replace(" ", "\x00").encode() + b"\x00")
    with open(os.path.join(mk_v, "stat"), "w", encoding="utf-8") as f:
        f.write("%d (rpicam-vid) S 1 1 1 0 -1 0 0 0 0 0 0 0 0 0 20 0 1 0 1\n" % victim.pid)
    n = cp.reap_stale()
    ok(n == 1, "reap_stale 收掉 1 个（实际 %d）" % n)
    time.sleep(0.2)
    ok(victim.poll() is not None, "被收的子进程真的没了（returncode=%s）" % victim.poll())
    ok(keeper.poll() is None, "登记在 _LIVE_STREAMS 里的没被误杀")
    keeper.kill()
    keeper.wait(timeout=5)
    cp._LIVE_STREAMS.pop(keeper.pid, None)

    print("\n== ④ 幂等: 收完就没了（真 /proc 里那个条目随进程一起消失）==")
    shutil.rmtree(mk_v, ignore_errors=True)
    ok(cp.reap_stale() == 0, "没有遗留时 reap_stale 返回 0 且不打日志")
finally:
    cp._PROC = old_proc
    shutil.rmtree(tmp, ignore_errors=True)

print("\n== ⑤ 取流闸门: 「已有热流」必须优先于「失败退避」==")
# 桩流: 只要能回答 alive()/get_frame()/last_frame_t() 就够了（PicamSource 按鸭子类型用它）
class _StubStream:
    """`t_last` = **流里**最近出帧的时刻（真实实现里由读线程打）。造桩时把"我们的消费时刻"
    与它**故意错开**, 才能钉住 alive() 看的是哪一个。"""

    def __init__(self, t_last, seq=7):
        self.t_last = t_last
        self.seq = seq

    def alive(self):
        return True

    def get_frame(self, last_seq=-1, timeout=0.0):
        return b"\xff\xd8\xff\xd9", self.seq

    def last_frame_t(self):
        return self.t_last

    def close(self):
        pass


src = cp.PicamSource()
src._stream = _StubStream(time.time())
src._fail_until = time.time() + 10.0          # 模拟"刚失败过, 还在退避期"
ok(src.available() is False, "前提: 退避期内 available() 为假")
ok(src.ready() is True, "前提: 热流 ready() 为真")
h = src.acquire(0.0)
ok(h is not None, "**有热流时 acquire 不该被退避挡掉**（现场: 日志报'起流失败'而相机是好的）")
ok(src.warm(1) is True, "**有热流时 warm 只续期, 也该回 True**（否则调用方以为这台不行）")
src.close()
src._stream = None
h2 = src.acquire(0.0)
ok(h2 is None, "真的没有流 + 退避期 → 仍然立刻 None（'退回主摄要快'那条不能丢）")

print("\n== ⑥ 判活看的是【流自己的】出帧时刻, 不是【我们取过帧】==")
# 现场 2026-10-16 05:28: 预热完成(05:28:07) 与第一次使用(05:28:10/14) 之间没人取过帧 ——
# 旧实现拿"消费时刻"判活, 于是把一段完全健康的流报成"取流中断"并退回主摄。
src2 = cp.PicamSource()
src2._stream = _StubStream(time.time() + 0.05)     # 流刚刚还在出帧
# ⚠️ 解码要 cv2/numpy（开发机上不一定有）→ 换成桩: 这里验的是**陈帧判据**, 不是解码
src2._decode = lambda jpg: "FRAME"
ok(src2.alive() is True, "流在出帧（last_frame_t 是新的）→ alive() 必须为真")
ok(src2.read(0.0)[0] is True, "同一条流 read() 也该拿到帧（不是'陈帧'）")
stale = _StubStream(time.time() - 5.0)             # 流 5s 没出帧了（读线程死了/管道停了）
src2._stream = stale
ok(src2.alive() is False, "流 5s 没新帧 → alive() 必须为假（这条路真死了, 该退回主摄）")
ok(src2.read(0.0)[0] is False, "同一情况 read() 也要拒发陈帧（宁可真失败）")

print("\n=== %s ===" % ("ALL OK" if not FAIL else ("%d 失败" % len(FAIL))))
sys.exit(1 if FAIL else 0)
