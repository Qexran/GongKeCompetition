# camera_lease.py — 相机租约: 预热 / 复用 / 回收（2026-09 加）
#
# 为什么有这个东西
# ----------------
# 原来每个视觉任务都是「进来 open_camera() → 识别 → release()」，而
# camera.py::CameraManager.open() 每次都要跑完 **15 帧预热** + V4L2 协商 +
# 曝光应用 —— 实测**每台相机开一次 ≈ 1.3s**（Pi 上 /api/camera/check 连开
# 主摄+扫码 = 2.94s，三次一致）。路线里 5 个视觉步骤 → 一趟约 6.5s 白花在
# 开关相机上，而且这笔开销还被硬编码进了固件超时（HOSTAGE_ALIGN_TIMEOUT_MS
# 的注释里写着「= Pi 的 HOSTAGE_TIMEOUT(6.0s) + 摄像头打开(~1s)」）。
#
# 单片机侧现在会**在车移动/巡线的那几秒里**先发一条预热指令（"B" 扫码相机 /
# "C" 主摄），本模块收到就把相机开好并**持续取帧保持新鲜**，视觉任务进来直接
# 解码 —— 省掉那 1.3s。
#
# 2026-09 加：这两条指令可以带一个**次数**后缀（"C,2"，固件积木「预热摄像头」的参数 2），
# 意思是「这台相机接下来要被**几个任务**消费」。它把空闲窗口放宽成
# 次数 × LEASE_IDLE_TIMEOUT_S —— 因为任务用完相机本来就不关它（release_camera 只
# touch 一下），卡住复用的只是那个看门狗窗口。不带后缀 / 次数 1 = 老行为一字不差。
# 倍数保持到那 N 次用完为止。见 _uses_total / idle_limit_s / acquire()。
#
# ⚠️⚠️ 四个必须守住的点（改本文件前先读完）
# ------------------------------------------
# 1. **必须持续 read()，不能「开完放着」**。V4L2 是环形缓冲（全仓库没设过
#    CAP_PROP_BUFFERSIZE，默认 4 帧）：相机开着但没人 read 时，驱动填满队列后
#    就丢新帧，队列里那几帧永远停在**最后一次读的时刻**。于是「提前开好、等车停
#    了一抓」拿到的是**车还在转弯时拍的画面** —— 而且**不报错**，只表现为
#    「扫到一个莫名其妙的码」。所以读线程要一直跑，只保留最新一帧。
#    顺带好处：AE/AWB 一直跟着场景走，车停时曝光已经收敛。
# 2. **空闲看门狗不能删**。单片机发了预热却没跟进识别（路线被改 / 脚本被中止 /
#    OLED 那边操作员反悔），租约就会**永久占着相机** —— Web 控制台的预览从此
#    打不开，而且没有任何地方会报错。超时自动释放是这条链路上唯一的安全阀
#    （纪律同固件侧的 BT_EDIT_IDLE_TIMEOUT_MS）。
#    ⚠️ 2026-09: 那个超时值现在是 次数 × LEASE_IDLE_TIMEOUT_S（见 idle_limit_s）——
#       按次数放宽是**有意**的（让隔十几秒的下一个任务也能命中），但它同时就是
#       **安全阀被削弱的倍数**：脚本发了预热却没跟进识别时，相机被白占的时间也翻 N 倍。
#       所以次数在固件侧有上限、config 侧再有 LEASE_USES_MAX；别再往这里加别的放宽容忍。
# 3. **主摄有多个相机档**（camera.py::_CAMERA_PROFILES：人质 1/350、色球/靶标 3/166、
#    看路固定白平衡，同一台物理设备）。租约命中但档位不同时必须**就地重设曝光与
#    白平衡**并**丢掉收敛期那几帧**，否则人质会拿到色球那档曝光而识别失败、
#    看路会拿到"还没收敛的 AWB"。这就是 _discard_until 的用途。
# 4. **打开失败先怀疑「另一个进程占着」**。真正的相机独占者是 **GKS_Web 那个
#    独立进程**（网页预览，:9000），它 open 之后一直占着到浏览器断开；而本进程的
#    handler._stop_main_stream() 只能停自己进程内的推流，**对它是空操作**。
#    于是「谁先开到设备谁赢」，表现随机：预热失败 → 识别任务重试也抢不回来 →
#    `[ERROR] 无法打开摄像头索引 N`。唯一的跨进程办法是打它的 HTTP 端点
#    `/abort?cam=main|qr`（见 _abort_gks_web_stream，_open_with_abort_retry 用它）。
#    ⚠️ 反向的纪律：**预热会牺牲租约里的另一台相机**（warm 目标不同时先
#    release_now）—— 所以**不要在靶标流（"7"）运行期间发预热指令**，那会把正在
#    跑的主摄流掐断。排路线时把预热放在「离使用点最近、且另一台相机没人用」的位置。
# 5. **陈帧绝不发放（2026-10 加, 新鲜度闸）**。读线程停摆（USB 重置 / 设备 wedge /
#    线程死亡）时 _latest 会冻结在最后一帧 —— 照发的话任务会对**同一张旧图**反复
#    解码: 表现是「识别结果与上次一字不差」或「对着断电前的画面检出 0 个目标」,
#    且旧帧的 read() 会 touch 续命、看门狗永远不触发。三道闸: LeaseHandle.read 按
#    config.LEASE_STALE_S 判帧龄（陈帧返回 False 且不 touch）; warm/acquire 过
#    _healthy_locked 健康门（读线程死/帧过期 → 回收重开, 不命中僵尸）; 看门狗对
#    不健康租约立即回收。别删任何一道。
#
# 与 handler / tasks 的分工
# ------------------------
# tasks/*.py 只调 acquire_camera() / release_camera()（见文件末尾），**函数体
# 一行都不用改** —— LeaseHandle.read() 返回 (True, frame)，与 cv2.VideoCapture
# 的 read() 同形，所以 camera.py::CameraManager.read(cap, ...) 直接就能用。

import threading
import time
import urllib.request

from camera import CameraManager, open_camera

# 读线程两帧之间的最小间隔。相机本身就是 30fps，这里只是别把 CPU 吃满，
# 不是节流精度 —— 取最新帧靠的是「只存最新」，不是靠这个间隔。
_FRAME_GAP_S = 0.005

# 就地换档前等读线程回执「不在 cap.read() 里」的上限（秒）。
# 健康流下读线程 ~33ms 一圈, 这个上限只会在两种情况触顶: 读线程卡死在 V4L2 DQBUF
# 里（正是要防的 wedge, 2026-10-02 run_20261002_015404 两次复现）, 或它恰好排在
# 帧入队的锁上（无害——排队中同样不在 read() 里）。前者照写不误并留 WARN, 看门狗收尸。
_CTL_HANDSHAKE_S = 1.0

# 跨进程抢相机（见文件头第 4 条）。GKS_Web 是独立进程、独立端口，本进程停不了它的推流，
# 只能请它自己停。⚠️ 超时给得短: 它就住在同一块板子上, loopback 上不该有慢请求;
# 万一 GKS_Web 卡住, 也不能把识别链路一起拖住。
_GKS_WEB_ABORT_URL = "http://127.0.0.1:9000/abort"
_ABORT_HTTP_TIMEOUT_S = 0.5   # 打 /abort 最多等这么久
_ABORT_SETTLE_S = 0.3         # 等 GKS_Web 的推流线程退出循环并 release 设备


def _idle_timeout_s():
    """空闲超时, 现取 config —— 支持 Web「设置参数」改完即时生效（同 open_camera 的纪律）。"""
    from config import LEASE_IDLE_TIMEOUT_S
    return float(LEASE_IDLE_TIMEOUT_S)


def _warm_wait_s():
    """acquire 等待「正在进行的预热」落地的上限。"""
    from config import LEASE_WARM_WAIT_S
    return float(LEASE_WARM_WAIT_S)


def _stale_s():
    """租约帧新鲜度阈值（秒），见 config.LEASE_STALE_S 与 _Lease.healthy_locked。"""
    from config import LEASE_STALE_S
    return float(LEASE_STALE_S)


# ---- 同进程 abort 钩子（GKS_Web 进程用，2026-10 加）----
# 背景: GKS_Web 的「就地识别」也走 acquire_camera 冷开兜底 → _open_with_abort_retry
# 开失败会打 http://127.0.0.1:9000/abort —— 而在 GKS_Web **自己**的进程里这一枪打的是
# 自己: /abort 会 set 它的 _scan_abort, 把正在跑的就地识别当场取消（页面显示「已取消」,
# 现场看着像玄学）。注册本钩子后, 同进程调用改走钩子（只停推流、不动识别）,
# HTTP 路径保留给 main.py 那个进程（车优先语义不变: main.py 的 /abort 仍会取消页面识别）。
_ABORT_HOOK = None


def set_abort_hook(fn):
    """注册同进程的「让出相机」钩子: fn(cam: str) -> bool, True=已请对方停推流。

    只该由 GKS_Web 的 app.py 在启动时注册一次; fn 内部自行吞异常并返回布尔。
    """
    global _ABORT_HOOK
    _ABORT_HOOK = fn


def _clamp_uses(uses):
    """次数后缀（"C,2" 里那个数）→ 内部用的次数（≥1 的整数）；超上限夹住并打 WARN。

    没给 / 0 / 负数 / 解析不出来 → 1（= 老行为，窗口 1 倍）。
    ⚠️ 上限的唯一理由是**安全阀**（见文件头第 2 条）：窗口 = 次数 × LEASE_IDLE_TIMEOUT_S，
       一个手滑的 "C,99" 会把相机钉住 25 分钟。
    """
    import config
    try:
        n = int(uses)
    except (TypeError, ValueError):
        return 1
    if n <= 1:
        return 1
    mx = int(getattr(config, "LEASE_USES_MAX", 10))
    if n > mx:
        print(f"[WARN] 预热次数 {n} 超过上限 {mx}，夹到 {mx}"
              f"（空闲窗口最多放宽到 {mx}×{_idle_timeout_s():.0f}s）")
        return mx
    return n


def _profile_of(is_qr, profile):
    """与 camera.open_camera 同一套默认: None → 'qr'/'main'。"""
    return profile or ("qr" if is_qr else "main")


# ==================== 跨进程抢相机（GKS_Web 预览页，见文件头第 4 条）====================

def _abort_gks_web_stream(is_qr):
    """请 GKS_Web 进程停掉这台相机的推流。

    返回 True = 请求确实送到了（值得等一下再重试打开），False = 够不着 GKS_Web
    （没在跑 / 卡住）—— 那时它不是占设备的人，重试也是白等。

    ⚠️ **本进程就是 GKS_Web 时走 _ABORT_HOOK**（set_abort_hook 注册）：HTTP 打自己
       会让 /abort 顺手 set 页面识别的 _scan_abort, 把发起这次冷开的识别自己取消掉
       （症状=页面莫名「已取消」）。钩子只停推流, 不动识别 —— 让冷开重试自己分出胜负。
    ⚠️ 任何异常都静默吞掉: 比赛时 GKS_Web 可能压根没起, 那是 normal case 而不是错误。
    """
    cam = "qr" if is_qr else "main"
    if _ABORT_HOOK is not None:
        try:
            if _ABORT_HOOK(cam):
                print(f"[INFO] 已请本进程停掉{'扫码相机' if is_qr else '主摄'}推流（进程内钩子）")
                return True
            return False
        except Exception:
            return False
    try:
        with urllib.request.urlopen(f"{_GKS_WEB_ABORT_URL}?cam={cam}",
                                    timeout=_ABORT_HTTP_TIMEOUT_S) as resp:
            resp.read(64)          # 读完响应体, 别让连接半开着
    except Exception:
        return False
    print(f"[INFO] 已请 GKS_Web 停掉{'扫码相机' if is_qr else '主摄'}推流"
          f"（跨进程抢相机: 有人开着网页预览）")
    return True


def _open_with_abort_retry(is_qr, profile=None, attempts=3):
    """开相机；失败就请 GKS_Web 让出设备再试（带退避，最多 attempts 轮）。

    ⚠️ 只在**第一次真的失败**之后才会去打扰 GKS_Web —— 所以比赛时（没人开预览页）
       行为与加这条之前一模一样: 就是一次 open_camera()，没有任何额外开销。
    ⚠️ 2026-10: 重试从 1 次加到 attempts 次、退避 0.3→0.6→1.0s —— 对端推流线程
       退出循环 + release 设备在 Pi 忙时不止 0.3s, 只试一次会表现成「随机打开失败」。
    """
    name = "扫码相机" if is_qr else "主摄"
    settle = _ABORT_SETTLE_S
    for attempt in range(attempts):
        cap = open_camera(is_qr=is_qr, profile=profile)
        if cap is not None:
            return cap
        if attempt == attempts - 1:
            break
        if not _abort_gks_web_stream(is_qr):
            # 够不着 GKS_Web → 占设备的多半是别的（预热线程竞态 / 本进程推流）。
            # 仍退避重试（不请 /abort）: EBUSY 的另一头往往几百 ms 内就会放手。
            print(f"[WARN] {'扫码相机' if is_qr else '主摄'}打开失败 (第{attempt + 1}/{attempts}次), "
                  f"退避 {settle:.1f}s 重试")
            time.sleep(settle)
            settle *= 2
            continue
        time.sleep(settle)    # 等 GKS_Web 的推流线程退出循环并 release 设备
        settle *= 2
    if attempts > 1:
        print(f"[WARN] 抢回{name}失败: {attempts} 次重试后仍打不开 —— "
              f"还有别的进程/任务占着设备（见 camera_lease 文件头第 4 条）")
    return None


class LeaseHandle:
    """租约句柄 —— 鸭子类型的 cv2.VideoCapture（只用到 read()）。

    ⚠️ read() 返回**最新一帧的引用**（不是拷贝）。各任务的扫描循环只读不写，
    所以共享引用是安全的；**别**在任务里就地改这帧（要改先 .copy()）。

    ⚠️ read() 有**新鲜度闸**（2026-10 加）: 最新帧距今超过 config.LEASE_STALE_S
       就返回 (False, None) 且**不 touch** —— 读线程停摆时（USB 重置 / 设备 wedge /
       线程死亡）_latest 会冻结在最后一帧, 照发的话任务会对同一张旧图反复解码:
       表现正是「识别结果与上次一字不差」或「对着别的场景的旧帧检出 0 个目标」。
       返回 False 让调用方按「无帧」走自己的重试/超时; 不 touch 则让空闲看门狗
       能到期回收这个僵尸租约（否则每次读到旧帧都续命, 看门狗永远不触发）。

    ⚠️ release() 是**故意**的空操作: 有些任务的 finally 里还留着
    `CameraManager.release(cap)`（改漏了也不会出事）—— 真关设备会让租约失效。
    归还租约请走 release_camera()。
    """

    def __init__(self, is_qr):
        self._is_qr = is_qr

    def read(self):
        frame, age = LEASE.latest_with_age()
        if frame is None:
            return (False, None)          # 还没读到帧 / 正在丢收敛帧 → 让调用方重试
        if age > _stale_s():
            LEASE.note_stale_read()
            return (False, None)          # 陈帧不发放（见类注释的新鲜度闸）
        LEASE.touch()
        return (True, frame)

    def release(self):
        """故意不关设备（见类注释）。"""
        return None


class _Lease:
    def __init__(self):
        self._lock = threading.RLock()
        self._cap = None
        self._is_qr = None
        self._profile = None
        self._latest = None               # 最新一帧（只保一个）
        self._latest_ts = 0.0             # 最新一帧的写入时刻（新鲜度闸用, 见 LeaseHandle）
        self._read_fails = 0              # 读线程**连续**读帧失败次数（2026-10-06 加, 见 _healthy_locked）
        self._stale_warned = False        # 本次租约是否已打过「陈帧」WARN（限一条, 免刷屏）
        self._thread = None
        self._stop = threading.Event()
        self._ctl_pause = threading.Event()    # 置位=读线程暂停进 cap.read()（换档独占设备写控件）
        self._reader_idle = threading.Event()  # 读线程回执「当前不在 cap.read() 里」（换档握手用）
        self._pending = None              # (is_qr, profile) —— 预热进行中
        self._last_use = 0.0
        self._discard_until = 0.0         # 曝光重设后丢帧到这一刻（见文件头第 3 条）
        self._uses_total = 0              # 预热时声明「这台相机接下来要用几次」→ **窗口倍数的来源**
        self._uses_left = 0               # 还剩几次没用（acquire 命中时递减）
                                          # ⚠️ 唯一写这两个的是 warm()（含两条早退路径）、
                                          #    acquire() 递减 _uses_left、release_now() 两个都清零。
                                          # 为什么是两个而不是一个: 窗口要在**用满 N 次之前**
                                          # 一直保持 N 倍 —— 只留"剩余次数"的话窗口会随着使用
                                          # 越来越窄, 把最大的宽限给了第一个间隔（那通常是摆臂的
                                          # 一两秒）, 反而把真正长的第二个间隔压回 1 倍, 与
                                          # 「一次预热盖住 N 次」正好相反。

    # ---- 读线程 ----

    def _reader(self, cap, is_qr):
        name = "QR Camera" if is_qr else "Main Camera"
        try:
            while not self._stop.is_set():
                if self._ctl_pause.is_set():
                    # 换档握手（2026-10-02）: 我此刻不碰 cap → 回执; 暂停期间空转等解除。
                    # ⚠️ ack/清 ack 都**不碰 self._lock** —— 换档方持着锁等这个回执,
                    #    这里一旦去排队拿锁, 回执就永远出不来（握手白等一个超时）。
                    self._reader_idle.set()
                    time.sleep(0.01)
                    continue
                self._reader_idle.clear()
                frame = CameraManager.read(cap, name, max_retries=2)
                if frame is None:
                    self._read_fails += 1        # 连续失败 → _healthy_locked 快速判死（见那里）
                    time.sleep(0.02)
                    continue
                self._read_fails = 0
                # 换档握手(续): 刚出 read() 就发现要暂停 → 这帧不存也不抢锁,
                # 直接回循环顶回执 —— 否则会在下面的锁上被换档方挡住, 回执超时。
                if self._ctl_pause.is_set():
                    continue
                now = time.time()
                # 存帧用**限时抢锁**: 换档方持锁等回执最长 1s, 无限等会把回执路径
                # 堵死(上面那道检查关不掉「已过检查、还没抢到锁」的微秒窗口)。
                # 50ms 拿不到 = 锁被长持 → 这帧不要了, 回循环顶(可能正等着回执)。
                if not self._lock.acquire(timeout=0.05):
                    continue
                try:
                    if now < self._discard_until:
                        pass           # 曝光刚重设, 这几帧还在收敛, 丢掉
                    else:
                        self._latest = frame
                        self._latest_ts = now
                        self._stale_warned = False
                finally:
                    self._lock.release()
                time.sleep(_FRAME_GAP_S)
        finally:
            # ⚠️ **设备归读线程所有**: 它是唯一在读这个句柄的人, 所以也只有它能在
            #    安全的时候关。release_now() 的 join 超时后会把设备留给这里（见那里
            #    的注释）—— 少了这一行, 那条「超时就不放」的兜底就真的变成泄漏了。
            #    ⚠️ release_now() 正常路径下也会再 release 一次, 那是空操作
            #    （cv2 的 release() 对已释放的句柄幂等）, 不是双重释放 bug。
            # ⚠️ 2026-10: 读线程**异常退出**（cv2 抛错 / 设备消失时 cap.read 抛异常）
            #    还要清掉租约状态 —— 否则租约握着一个「设备已关但 _cap 还在」的僵尸:
            #    acquire/warm 照旧命中它, 任务拿到的全是冻结的 _latest（陈帧闸至少
            #    挡住发放, 但设备谁也不开, 直到重启）。判据 `self._cap is cap`:
            #    release_now() 先清了状态的情况这里自然跳过, 不会双清。
            try:
                CameraManager.release(cap)
            except Exception as e:
                print(f"[WARN] 租约读线程退出时释放相机失败: {e}")
            with self._lock:
                if self._cap is cap:
                    print("[WARN] 相机租约读线程退出, 租约状态已清空"
                          "（下次 warm/acquire 重新开设备）")
                    self._cap = None
                    self._is_qr = None
                    self._profile = None
                    self._latest = None
                    self._thread = None
                    self._pending = None
                    self._discard_until = 0.0
                    self._uses_total = 0
                    self._uses_left = 0

    # ---- 内部: 真正把相机开起来（在预热线程里跑, 所以不阻塞串口线程）----

    def _open_and_serve(self, is_qr, profile):
        name = "扫码相机" if is_qr else "主摄"
        cap = None          # 非 None = 本函数还持有设备、要负责交还
        try:
            # profile 直接传: "main"/"qr"/"hostage"/"road" 都是 _CAMERA_PROFILES 的键
            cap = _open_with_abort_retry(is_qr, profile)
            if cap is None:
                print(f"[WARN] 相机预热失败: {name} 打不开（档位={profile}）—— 已试过请 "
                      f"GKS_Web 让出设备（/abort）仍不行。要么是**有人开着网页预览页**，"
                      f"要么是别的进程/任务正占着；把预览关掉再试。"
                      f"⚠️ 这条不代表后续识别也会失败: 没有租约时各任务退回「现开现扫」, "
                      f"只是慢 ~2s（见 camera_lease 文件头第 4 条）")
                return
            # ⚠️ 「判 _pending 还在不在」「认领 _cap」「start 读线程」必须在**同一个**临界区。
            #    中间放出去的话 release_now() 会挤进来把 _pending 清掉, 而我们随后
            #    仍然把 _cap 装上 —— 相机被一个「已经释放过」的租约握着, 要等看门狗
            #    超时才还回来（不是泄漏, 但白占十几秒, 而且现象是「预热了却没生效」）。
            #    ⚠️ th.start() **也必须在锁内**: 原来它在锁外 start, 那一瞬间若
            #    release_now() 进来, 它会对一个**还没 start 的线程**调 join() →
            #    RuntimeError → 异常让它在 CameraManager.release(cap) 之前就中断 →
            #    读线程随后 start 成功、租约状态却已被清空 → 那台相机**再无任何路径
            #    会关它**（永久 EBUSY, 只能重启 main.py）。窗口极窄但命中一次就废一台相机。
            claimed = False
            with self._lock:
                if self._pending == (is_qr, profile):
                    self._cap = cap
                    self._is_qr = is_qr
                    self._profile = profile
                    self._latest = None
                    self._read_fails = 0    # 新一代租约, 失败计数清零（新一代读线程重新算）
                    # 新鲜度闸的「起跑时刻」: 读线程在开相机后的头一两秒还没有帧,
                    # 用认领时刻当 0 点, 给它 LEASE_STALE_S 的起帧宽限（开相机 ~2s +
                    # 预热 15 帧正好落在这个量级, 不够时宁可让它判不健康重开）。
                    self._latest_ts = time.time()
                    self._stale_warned = False
                    self._last_use = time.time()
                    self._stop = threading.Event()
                    self._ctl_pause = threading.Event()    # 握手事件随新一代租约换新,
                    self._reader_idle = threading.Event()  # 不带上一代的残留状态
                    th = threading.Thread(target=self._reader, args=(cap, is_qr),
                                          name="cam-lease-reader", daemon=True)
                    self._thread = th
                    th.start()                 # ⚠️ 锁内, 理由见上
                    cap = None                 # 所有权已交给读线程
                    claimed = True
            if claimed:
                print(f"[INFO] 相机预热完成: {name} 档位={profile}")
                return
        except Exception as e:
            print(f"[WARN] 相机预热异常({name}): {e}")
        finally:
            with self._lock:
                if self._pending == (is_qr, profile):
                    self._pending = None
                if cap is not None and self._cap is cap:
                    # 认领了却没跑起来（start 抛异常）→ 把租约状态一起清掉, 不留一个
                    # 「握着句柄但没人读」的租约（那种租约的 _latest 永远是 None,
                    # 任务会一直 read 失败, 而且看门狗也判不出它有问题）。
                    self._cap = None
                    self._is_qr = None
                    self._profile = None
                    self._thread = None
                    self._latest = None
                if self._cap is None:
                    # 这一轮到头来没留下活租约（打不开 / 异常 / 被抢）→ 次数一起清零,
                    # 免得留一个「有次数、没相机」的残值。
                    # ⚠️ 判据必须是 `_cap is None` 而**不是**在函数开头无条件清零:
                    #    **认领成功**那条路也会走本 finally（它顺手清 _pending），
                    #    在那里清零会把刚设好的次数当场抹掉（= 次数后缀静默失效）。
                    self._uses_total = 0
                    self._uses_left = 0
        if cap is not None:
            CameraManager.release(cap)         # 已被抢掉 / 异常 → 交还设备

    # ---- 对外 ----

    def warm(self, is_qr, profile=None, uses=1):
        """预热（非阻塞）: 起线程去开相机, 立即返回。

        ⚠️ **必须非阻塞**: 本函数由 handler 在**串口接收线程**里内联调用
        （handler.handle()），而阻塞式指令期间 Pi 是不发心跳的 —— 在这里做那
        1.3s 的打开会把心跳也一起压住（固件 8s 判 DISCONNECTED）。

        uses = 这台相机**接下来要被几个任务消费**（固件积木「预热摄像头」的参数 2，
        随 "C,2" 这种次数后缀过来）→ 空闲窗口放宽成 uses × LEASE_IDLE_TIMEOUT_S
        （见 idle_limit_s）。≤1 与不带后缀同义 = 老行为。
        """
        uses = _clamp_uses(uses)
        prof = _profile_of(is_qr, profile)
        with self._lock:
            if self._cap is not None and self._is_qr == is_qr:
                if self._healthy_locked():
                    if self._profile != prof:
                        self._reapply_profile(prof)
                        self._profile = prof   # ⚠️ 必须跟着改: 漏了的话 acquire() 后面
                                               # 判档位仍是旧的 → 同一档白重设两遍、
                                               # 白丢两个 0.5s 的收敛窗（acquire 是两行都写了的）
                    self._last_use = time.time()
                    # ⚠️⚠️ 这两条早退路径（含下面那条「同一目标已在预热」）**必须也重设次数**。
                    #    固件在「主摄租约还活着」时再发一条 "C,N" 是**常态** —— 只 touch 不刷新的话
                    #    次数后缀就静默地什么也没干，而本功能唯一的症状本来就是「慢一点」，最难查。
                    #    语义 = **后写者胜**（与这行的 _last_use 续期同一条纪律）: 后发的 "C,1"
                    #    必须能把先前 uses=5 收窄回 1，否则「填 1 == 老行为」就不成立了。
                    self._set_uses_locked(uses)
                    return True
                # 租约不健康（读线程停摆 / 帧过期, 见 _healthy_locked）→ 不给僵尸续命,
                # 掉到下面 release_now() 重开一条新的。
                print("[WARN] 相机租约不健康（读线程停摆或帧过期）, 预热改为回收重开")
            if self._pending == (is_qr, prof):
                self._last_use = time.time()
                self._set_uses_locked(uses)  # 同上: 已在预热也要重设
                return True                  # 同一个目标已在预热, 续期即可
        # 目标是另一台相机（或上面的租约不健康）→ 先把旧租约关掉（「与本次无关的相机要关掉」）
        self.release_now()
        with self._lock:
            # ⚠️⚠️ 次数**必须**在这里设（release_now 之后、与 _pending 同一临界区）:
            #    上面那句 release_now() 会清空全部状态 —— 把赋值放在函数开头会在
            #    「换成另一台相机」这条路上被每次抹掉（正是最需要它的那条路）。
            self._set_uses_locked(uses)
            self._pending = (is_qr, prof)
            self._last_use = time.time()
        threading.Thread(target=self._open_and_serve, args=(is_qr, prof),
                         name="cam-lease-warm", daemon=True).start()
        return True

    def _reapply_profile(self, profile):
        """就地换相机档（曝光 + 白平衡，不重开设备 —— 那要再付一次 15 帧预热）。

        ⚠️⚠️ 2026-10-02: 控件写入前必须先与读线程**握手**（_ctl_pause/_reader_idle）。
        OpenCV 的 VideoCapture 不保证线程安全 —— set() 与另一线程阻塞中的 read()
        （V4L2 DQBUF）并发是未定义行为。现场已两次复现（run_20261002_015404 的
        01:54:53 / 01:55:19）: 换档的 set() 失败后流从此只返回失败不返回帧（读线程
        活着但帧过期）→ 看门狗判「租约不健康」回收重开, 夹在中间的任务拿 0 帧失败。
        协议: 清回执 → 置 pause → 等读线程回执「不在 read() 里」（≤_CTL_HANDSHAKE_S）
        → 控件写入 → finally 解除 pause。等不到回执 = 读线程卡死在 DQBUF（正是要防的
        wedge）或仅是排在帧入队的锁上（同样不在 read() 里, 无害）→ 照写不误并留 WARN。
        ⚠️ 本函数由 warm/acquire 在**持 self._lock** 时调用, 等待回执期间不放手是
        安全的: 读线程的回执路径不碰锁; 而排在锁上的读线程恰恰不在 read() 里。
        ⚠️ pause 的解除必须放 finally —— 换档失败也绝不能把读线程永久饿死在暂停态。

        ⚠️ 重设后头几帧还在收敛, 必须丢掉: 否则任务拿到的第一帧是旧曝光/旧白平衡的。
        实测原地切换白平衡 2~3 帧即稳（AWB 关+固定色温、以及反方向切回自动都是），
        0.5s 的丢弃窗（~15 帧）覆盖得住。
        """
        from camera import profile_params
        # ---- 相位A: 握手 + 控件写入 ----
        self._reader_idle.clear()
        self._ctl_pause.set()
        acked = self._reader_idle.wait(_CTL_HANDSHAKE_S)
        if not acked:
            print(f"[WARN] 租约换档: 读线程 {_CTL_HANDSHAKE_S:.1f}s 内未回执空闲 —— "
                  f"疑似卡死在 V4L2 读里（正是要防的 wedge）。控件写入继续（可能与读"
                  f"并发）, 本租约若真已卡死会被看门狗回收")
        try:
            p = profile_params(profile)
            CameraManager._apply_white_balance(self._cap, *p["wb"])
            CameraManager._apply_exposure(self._cap, *p["exposure"])
        except Exception as e:
            print(f"[WARN] 租约换档失败({profile}): {e}")
            return
        finally:
            self._ctl_pause.clear()
        with self._lock:
            # ⚠️ 必须**同时清 _latest**: 只设 _discard_until 的话读线程不再存新帧,
            #    但 lease.latest_frame() 照旧把**换档前那一帧**发给任务反复解码 ——
            #    于是收敛窗里任务拿到的是旧曝光/旧白平衡的图, 正好是这条机制要防的事。
            self._latest = None
            self._latest_ts = time.time()   # 丢弃窗内不算「过期」（见新鲜度闸）
            self._stale_warned = False
            self._discard_until = time.time() + 0.5   # ~15 帧 @30fps
        print(f"[INFO] 租约就地把相机换成档位 {profile}")

    def adopt(self, cap, is_qr, profile):
        """把一条**已经打开**的相机接管成租约（2026-10-06 加）→ LeaseHandle | None。

        用途: acquire_camera 的冷开兜底。旧行为里冷开的 cap 是「裸句柄」, 任务用完
        release_camera **真关掉**, 于是下一个任务又得全量重开 —— 台架实测: 看路
        连续两条 "G" 各自重开（~1s/条）, 直到中间恰好出现一条 "C" 预热才有租约可命中。
        接管之后这条相机就是租约: 本任务照常用, 用完归还（保持打开）, **下一个任务
        直接命中**（档位不对时 acquire 自带就地换档）。

        ⚠️ 认领 + 起读线程与 _open_and_serve 是**同一段临界区纪律**（start 必须锁内,
           否则 release_now 挤进来 join 一个没 start 的线程 → 相机永久 EBUSY, 那边的
           注释就是为这条写的）。
        ⚠️ 拒绝接管的两种情形（返回 None, 调用方退回裸句柄旧行为）:
           ① 同台相机的预热正在跑 —— 接管 = 两个线程开/读同一设备;
           ② 已有租约占着（acquire 未命中本不该走到这, 防御）。
        返回 LeaseHandle —— 调用方**必须**改用它读帧（读线程从此拥有 cap）。
        """
        prof = _profile_of(is_qr, profile)
        with self._lock:
            if self._pending is not None and self._pending[0] == is_qr:
                return None
            if self._cap is not None:
                return None
            self._cap = cap
            self._is_qr = is_qr
            self._profile = prof
            self._latest = None
            self._read_fails = 0
            self._latest_ts = time.time()      # 起帧宽限（同 _open_and_serve 的口径）
            self._stale_warned = False
            self._last_use = time.time()
            self._stop = threading.Event()
            self._ctl_pause = threading.Event()
            self._reader_idle = threading.Event()
            th = threading.Thread(target=self._reader, args=(cap, is_qr),
                                  name="cam-lease-reader", daemon=True)
            self._thread = th
            th.start()                          # ⚠️ 锁内, 理由见 _open_and_serve
        print(f"[INFO] 冷开相机接管为租约（档位={prof}）—— 下一个任务可直接命中")
        return LeaseHandle(is_qr)

    def acquire(self, is_qr, profile=None):
        """取租约。命中返回 LeaseHandle，未命中返回 None（调用方走旧行为自己 open）。

        等待规则: 若**同一台相机正在预热**, 就等它落地再判 —— 不等的话两边会
        同时 open 同一台设备, 后到的那个吃 EBUSY（Pi 的 CLAUDE.md 里
        _cmd_aim_start 的 old.join 就是同一个坑）。

        ⚠️ 2026-10: 命中的租约还要过**健康门**（_healthy_locked）—— 读线程死了或
           最新帧过期（冻结租约）就不发放, 回收后返回 None 让调用方冷开。否则
           USB 重置后的僵尸租约会把断电前的旧帧当实时帧发给任务（症状=识别结果
           与上次一字不差 / 0 目标, 重启 main.py 才恢复）。
        """
        prof = _profile_of(is_qr, profile)
        deadline = time.time() + _warm_wait_s()
        while True:
            with self._lock:
                if self._cap is not None and self._is_qr == is_qr:
                    if self._healthy_locked():
                        if self._profile != prof:
                            self._reapply_profile(prof)
                            self._profile = prof
                        self._last_use = time.time()
                        # 这一次使用花掉一档「预备好的次数」: 递减只用来知道**N 次用满了没有**
                        # （用满之后窗口才从 N 倍缩回 1 倍，见 idle_limit_s）。
                        # ⚠️ 粒度是「任务」= 每命中一次算一次（_open_with_retry 只在 acquire
                        #    返回 None 时重试，所以一次任务不会烧掉好几档）。给少了只是退回老行为。
                        if self._uses_left > 0:
                            self._uses_left -= 1
                        return LeaseHandle(is_qr)
                    print("[WARN] 相机租约不健康（读线程停摆/帧过期）, 回收后走冷开")
                held_other = (self._cap is not None and self._is_qr != is_qr)
                pending_same = (self._pending is not None and self._pending[0] == is_qr)
                need_release = self._cap is not None     # 另一台相机 / 本台但不健康
            if need_release:
                # 租约持的是另一台相机, 或本台但已不健康 → 关掉它, 再看一次
                self.release_now()
                continue
            if pending_same and time.time() < deadline:
                time.sleep(0.02)
                continue
            return None                   # 没租约 / 预热带超时 → 旧行为

    def _healthy_locked(self):
        """租约是否健康（调用方必须已持锁）: 持有相机 + 读线程活着 + 最新帧未过期。

        「帧过期」按 config.LEASE_STALE_S 判, 判据是 _latest_ts（读线程每存一帧
        刷新）; 认领/换档时会把 _latest_ts 重置当起跑宽限（见 _open_and_serve /
        _reapply_profile）。
        """
        if self._cap is None:
            return False
        if self._thread is not None and not self._thread.is_alive():
            return False
        # 连续读帧失败（设备掉线/被抢/格式协商坏了 —— 2026-10-06 台架实测: 一次坏 open
        # 的 cap.read() 全 False, 却要等帧龄超 LEASE_STALE_S(2s) 才被判不健康, 预热的
        # 2s 全打了水漂）。一次失败读的循环 ~0.1~0.2s, 5 次 ≈ 1s —— 到 5 次直接判死,
        # 看门狗当场回收重开, 比"等帧过期"快约一秒。⚠️ 换档暂停期间读线程不进 read,
        # 不会累计失败, 不误伤。
        if self._read_fails >= 5:
            return False
        return (time.time() - self._latest_ts) <= _stale_s()

    def healthy(self):
        """租约是否健康（看门狗等外部用, 自带锁）。空租约 = False。"""
        with self._lock:
            return self._healthy_locked()

    def latest_with_age(self):
        """(最新帧, 帧龄秒)。无帧时 (None, None)。LeaseHandle.read 的新鲜度闸用。"""
        with self._lock:
            if self._latest is None:
                return None, None
            return self._latest, time.time() - self._latest_ts

    def note_stale_read(self):
        """LeaseHandle 读到陈帧时调用: 限一条 WARN（每次租约只提醒一次, 免刷屏）。"""
        with self._lock:
            if self._stale_warned:
                return
            self._stale_warned = True
        print(f"[WARN] 相机租约的帧已超过 {_stale_s():.0f}s 没更新（读线程停摆?）, "
              f"陈帧暂停发放 —— 任务按无帧处理, 看门狗稍后回收本租约")

    def latest_frame(self):
        with self._lock:
            return self._latest

    def idle_age(self):
        """(是否有相机被持有, 距上次使用的秒数) —— 给看门狗用。"""
        with self._lock:
            if self._cap is None or self._pending is not None:
                return (False, 0.0)
            return (True, time.time() - self._last_use)

    def touch(self):
        with self._lock:
            self._last_use = time.time()

    def uses_left(self):
        with self._lock:
            return self._uses_left

    def _set_uses_locked(self, uses):
        """登记「这次预热准备被用几次」（⚠️ 调用方必须**已持锁**）。

        三个调用点（warm 的两条早退 + 换相机那条）都要的是**同一对**赋值 ——
        收成一个函数是为了以后加字段时不会只改到其中一处（那种漏改的症状是
        「次数后缀静默失效」, 而本功能唯一的症状本来就是"慢一点", 最难查）。
        """
        self._uses_total = uses
        self._uses_left = uses

    def idle_limit_s(self):
        """空闲窗口的**上限**（秒）—— 看门狗拿它跟 idle_age() 的秒数比。

        = LEASE_IDLE_TIMEOUT_S × N（N = 这次预热声明的使用次数; 用满 N 次之后回到 1 倍）。

        N 来自固件积木「预热摄像头」的 uses 参数（线上 "C,2" 里那个数）= 「这台相机
        接下来要被**几个任务**消费」。于是:
          uses=1（或不带后缀）→ 窗口一个 LEASE_IDLE_TIMEOUT_S = 加这个参数之前**一字不差**;
          uses=2 → 从预热到「第二个任务用完」之间的**每一个**间隔都有 2 个窗口的余量。
        要解决的就是「隔了十几秒的第二、三个任务」——任务用完相机是不关它的
        （release_camera 只 touch 一下），会撞上的只有这个看门狗窗口, 撞上就白花 ~2s 重开。

        ⚠️ 为什么是**声明次数 N** 而不是「剩余次数」: 后者会让窗口越用越窄 —— 最大的宽限
           落在第一个间隔（通常只是摆臂那一两秒）, 真正长的第二个间隔反而只剩 1 倍,
           与「一次预热盖住 N 次」正相反。见 _uses_total 的注释。
        ⚠️ 倍数同时就是**安全阀被削弱的倍数**（见文件头第 2 条）: 脚本发了预热却没跟进识别时,
           相机被白占的时间也翻 N 倍（上限 N × LEASE_IDLE_TIMEOUT_S）。所以次数有上限
           （固件侧钳一次、handler 侧 _clamp_uses 再钳一次）。
        """
        with self._lock:
            n = self._uses_total if self._uses_left > 0 else 0
        return _idle_timeout_s() * float(n if n > 0 else 1)

    def release_now(self):
        """显式释放（看门狗 / "end" / "r" / 换成另一台相机时）。"""
        with self._lock:
            cap, th = self._cap, self._thread
            uses_left = self._uses_left
            self._cap = None
            self._is_qr = None
            self._profile = None
            self._latest = None
            self._latest_ts = 0.0
            self._stale_warned = False
            self._thread = None
            self._pending = None
            self._discard_until = 0.0
            self._uses_total = 0              # 释放了, 次数预算作废（下一次预热重新登记）
            self._uses_left = 0
            stop = self._stop
        stop.set()
        if th is not None:
            try:
                th.join(timeout=1.0)      # 读线程自己会退出, 超时也不阻塞调用方太久
            except RuntimeError as e:
                # 「线程还没 start」= 与 _open_and_serve 的认领临界区互斥后理论上
                # 不会发生（start 已挪进锁内）。兜住只是不让它把下面的 release 一起吃掉。
                print(f"[WARN] 租约释放: join 一个未启动的线程({e})")
        if cap is None:
            return
        if th is not None and th.is_alive():
            # ⚠️ 读线程还阻塞在 V4L2 DQBUF 里（相机卡住）→ 这时 release 就是对
            #    **使用中的句柄**动手, 行为未定义（OpenCV 的 V4L2 后端不保证线程安全）。
            #    把设备**留给读线程**: 它一旦解阻塞就会自己关（见 _reader 的 finally）。
            #    宁可漏放这一轮, 也不做一次未定义操作。
            print("[WARN] 相机租约释放: 读线程未在 1s 内退出, 本轮不释放设备"
                  "（留给读线程自己关）")
            return
        CameraManager.release(cap)
        if uses_left > 1:
            # 「安全阀被放宽过」的现场收据: 次数预算没花完就释放了（路线被改/被中止/
            # 压根没走到那几次使用）—— 相机被白占的时间就是窗口的倍数。看到这条就
            # 知道该把那个「预热摄像头」的 uses 调小。
            print(f"[INFO] 相机租约已释放（预热次数预算还剩 {uses_left} 档没用完）")
        else:
            print("[INFO] 相机租约已释放")


LEASE = _Lease()


def lease_watchdog():
    """空闲看门狗线程（main.py 启动时拉起一次）。

    ⚠️ 不能删 —— 见文件头第 2 条。没有它, 一条没有后续识别的预热指令就会让
    相机被永久占住, 且无任何报错。
    ⚠️ 比的是 LEASE.idle_limit_s()（= 超时 × 剩余次数），**不是**裸的超时常量 ——
       次数后缀（"C,2"）就是靠它生效的。日志里打的是**实际用到的秒数**: 一台
       uses=3 的相机全程没人用，这里应该报 ~45s 而不是 15s，那就是倍数落地的证据。
    ⚠️ 2026-10 加第二条腿: 租约**不健康**（读线程停摆 / 帧过期, 见 _healthy_locked）
       时立即回收, 不等空闲窗 —— 冻结租约即使一直被 touch 续命也要能在 1s 内出局
       （LeaseHandle 的新鲜度闸已经保证陈帧不续命, 这里是兜底回收）。
    """
    while True:
        time.sleep(1.0)
        try:
            held, age = LEASE.idle_age()
            if not held:
                continue
            if not LEASE.healthy():
                print("[WARN] 相机租约不健康（读线程停摆/帧过期）, 看门狗立即回收")
                LEASE.release_now()
                continue
            limit = LEASE.idle_limit_s()
            if age > limit:
                n = LEASE.uses_left()
                why = f"（预热次数预算 ×{n}，一直没人用）" if n > 1 else ""
                print(f"[INFO] 相机租约空闲超过 {limit:.0f}s, 自动释放{why}")
                LEASE.release_now()
        except Exception as e:
            print(f"[WARN] 租约看门狗异常: {e}")


# ==================== 任务侧入口（tasks/*.py 只用这两个） ====================

def acquire_camera(is_qr=False, profile=None):
    """取相机: 命中租约 → LeaseHandle; 否则 → 真的 open_camera()（旧行为）。

    返回值两种都能直接喂给 camera.py::CameraManager.read(cap, ...)。
    ⚠️ 调用方**必须**在 finally 里交给 release_camera(), 别自己 release ——
    租约命中时要「归还」而不是「关闭」（下次任务还要用）。

    ⚠️ 2026-10-06 加第三条腿: 冷开成功的 cap **接管成租约**（LEASE.adopt）——
       旧行为里它是裸句柄、用完真关, 于是「连续几个主摄任务、中间没有 C 预热」时
       每个任务都全量重开（台架实测: 看路连续两条 "G" 各自重开 ~1s）。接管后下一个
       任务直接命中。接管失败（同台预热恰好在跑等）→ 退回裸句柄旧行为, 不影响本次。
    """
    handle = LEASE.acquire(is_qr, profile)
    if handle is not None:
        return handle
    # 冷开兜底。⚠️ 走 _open_with_abort_retry 而不是裸 open_camera: 网页预览在
    # **另一个进程**（GKS_Web）里占着设备时, 只有它能被请走（见文件头第 4 条）。
    # 各任务的 _open_with_retry 会反复调本函数, 于是天然形成重试。
    cap = _open_with_abort_retry(is_qr, profile)
    if cap is None:
        return None
    handle = LEASE.adopt(cap, is_qr, profile)
    if handle is not None:
        return handle
    return cap                       # 接管没成 → 旧行为（裸句柄, 用完真关）


def release_camera(cap):
    """归还相机: LeaseHandle → 留在租约里（保持打开）; 普通 cap → 真释放。"""
    if isinstance(cap, LeaseHandle):
        LEASE.touch()
        return
    CameraManager.release(cap)


def release_lease_if(is_qr):
    """若当前租约正是这台相机, 立即释放。→ 是否真释放了一次（2026-10 加）。

    给 GKS_Web 的 /abort 用: /abort 的语义是「把这台相机的**一切持有形式**让出来」
    —— 只停推流不动租约, 相机仍被**网页进程**的租约攥到看门狗到期; 而抢相机的一方
    （main.py 的 _open_with_abort_retry）只留了 0.3s×3 的重试窗, 永远等不到那 15s。
    症状即 2026-10-04 的现场: main.py 一直「无法打开摄像头」, /abort 连打 8 次无效,
    直到拔插 USB（当时 GKS_Web 进程根本没跑看门狗, 见 app.py 的修复注释）。
    ⚠️ 租约是单槽的（一次只持一台）, is_qr 对不上就不动 —— 那台设备的占用另有其人。"""
    with LEASE._lock:
        if LEASE._cap is not None and LEASE._is_qr == is_qr:
            LEASE.release_now()               # RLock 可重入, 内部再拿锁安全
            return True
    return False
