# serial_comm.py — 串口通信（单片机 ↔ 树莓派）
#
# 同时等待两个信号，二者平级、无静态开关，谁先就绪就用谁：
#   - 真实串口 /dev/ttyUSB_GKS          → 正常模式（比赛）
#   - 模拟器虚拟串口 127.0.0.1:9997      → 模拟模式（无单片机单端调试）
#
# 打开失败会一直重试等待（上电自启时 USB 串口可能尚未枚举）；断开后自动重新等待。
import os
import time
import socket
import threading
from collections import deque

import serial

import protocol
from protocol import (FrameParser, pack_frame, TYPE_CMD, TYPE_COORD,
                      TYPE_HELLO, TYPE_ACK, TYPE_DEBUG)

# 调试工具模式：设 SERIAL_OPTIONAL=1 时各尝试一次、不阻塞（供工具脚本导入用）。
SERIAL_OPTIONAL = os.environ.get("SERIAL_OPTIONAL") == "1"

# 等待连接时状态日志间隔（秒）：等待期每这么久打一次「两侧各自为何没就绪」，
# 否则日志停在启动那几行、看不出是在等硬件还是卡住了。
CONNECT_LOG_INTERVAL = 10.0

# ---- 链路故障自愈：原地重启本进程（不退出）----
# 串口 I/O 错误（USB 串口适配器掉线 → 读时 `[Errno 5] Input/output error`）之后，
# 本进程再也读不到单片机，留着也没用。调试模式（GKS_Web 控制台「开始/重启 main.py」
# 用 Popen 拉起）下**没有任何东西会把它重新拉起来** —— 所以自己 os.execl 重启一份，
# 效果等同 systemd gks.service 的 Restart=always（execl 保持 PID，systemd 那边不察觉）。
#
# 重启前等待的两个用途：① 给 USB 串口适配器留出重新枚举的时间（重新插拔 / 驱动重载
# 都要几百 ms~几秒）；② 避免「连上就报错、报错就重启」打转。
# ⚠ 连续重启时**逐次加长**（3s、6s、…、上限 60s）：适配器真掉线或驱动假死时上面那个
#    循环是收敛不了的，而每次重启都会新建一个日志文件（main.setup_logging 按启动时刻
#    命名）—— 固定 3s 会在几分钟内往 SD 卡堆出上千个小文件。
# 计数靠环境变量跨 exec 传递（os.execl 继承当前环境）；握手成功一次即清零
# （note_link_ok）—— 所以偶发抖动之后不会一直背着那个大延时。
RESTART_DELAY_S = 3.0
RESTART_DELAY_MAX_S = 60.0
RESTART_COUNT_ENV = "GKS_LINK_RESTART_N"
# 等待重启时的分段长度（秒）：分段睡是为了**等待期间还能响应停止信号** ——
# 一次睡满 60s 会把操作员按下的 Ctrl+C 和 systemd 发来的 SIGTERM 一起吃掉
# （本进程装了 SIGTERM 处理器，默认的「收到即终止」动作被接管了，只能自己查）。
RESTART_SLEEP_SLICE_S = 0.2


class SerialDisconnected(Exception):
    """串口/虚拟串口断开异常 —— 任务代码捕获此异常以中止当前操作。"""
    pass


class SerialComm:
    def __init__(self):
        self.ser = None
        self.mode = None   # 'serial' | 'simulator' | None
        # 最近一次 OS 级 I/O 错误（如 `[Errno 5] Input/output error`）。
        # 只为一件事：ser 被置空之后再有人 send()，报出来的原因是「上次那次 I/O 错误」
        # 而不是干巴巴的「串口未打开」—— 后者会把人带去查权限/占用，方向全错（现场
        # USB 串口适配器掉线就长这样）。每次开始等连接时清掉（connect 开头）。
        self.last_io_error = None
        # 已解析、尚未被消费的业务帧队列（2026-10 加）。
        # 旧版 receive() 一批字节里解析出多帧时**只保留最后一帧**、其余静默丢弃 ——
        # 阻塞任务执行期间（主循环卡在 handle 里）单片机连发的多条指令会被吃掉只剩
        # 最后一条（例如 "3" 停巡线 + "1" 扫码连发, "3" 直接消失）。改为逐帧入队、
        # receive() 每次弹出最早一帧, 顺序保持、一条不丢。
        self._frame_queue = deque()
        # ---- 专职接收线程（2026-10 加, 生产者）----
        # 旧版读串口发生在主循环的 receive() 里 —— 主循环一旦卡在阻塞任务
        # （"1"/"4"/"5"/"6"/"A"/"G", 最长 ~6s+）就没人读串口: 取消指令("x")躺在内核
        # 缓冲里等任务跑完才被看见, 「取消当前任务」名不副实。现在由本线程持续读帧
        # 入队（并触发 _frame_hook）, 主循环变成纯消费者 —— "x" 一到就能置位取消
        # 事件, 把在跑的任务当场打断（它们都在扫描循环里查 abort_event）。
        self._rx_thread = None
        self._rx_stop = threading.Event()
        self._rx_lock = threading.Lock()    # 只护 parser（flush_input 会 reset 它）
        self._frame_hook = None             # f(ftype, payload) -> None; 在接收线程里执行

    def set_frame_hook(self, fn):
        """注册「每帧到达」钩子: fn(ftype, payload), 在**接收线程**里执行。

        用途是让取消类指令绕过主循环的排队延迟（主循环可能正卡在某个阻塞任务里）。
        ⚠️ 钩子必须快、必须自己吞异常 —— 它卡住会拖死整个接收线程。
        ⚠️ 钩子里**不要**发串口帧（与主循环的写虽然不冲突, 但容易想当然）。
        """
        self._frame_hook = fn

    def _start_rx(self):
        """起接收线程（connect()/reconnect() 成功后各调一次）。"""
        ser = self.ser
        if ser is None:
            return
        self._rx_stop.clear()
        t = threading.Thread(target=self._rx_worker, args=(ser,),
                             name="gks-rx", daemon=True)
        self._rx_thread = t
        t.start()

    def _rx_worker(self, ser):
        """接收线程体: 只读这一个 ser 对象; self.ser 换人（重连/关闭）即退出。"""
        if not hasattr(self, "_parser"):
            self._parser = FrameParser()
        while not self._rx_stop.is_set() and self.ser is ser and ser is not None:
            try:
                n = ser.in_waiting
                if n == 0:
                    time.sleep(0.02)
                    continue
                raw = ser.read(n)
            except (serial.SerialException, OSError) as e:
                if self.ser is not ser:
                    return   # 报错的口已经换人（重连/关闭）—— 不是链路故障, 别把新连接带走
                print(f"[ERROR] 接收失败: {e}")
                self.last_io_error = e
                self._mark_disconnected()
                return
            if not raw:
                continue
            with self._rx_lock:
                for b in raw:
                    r = self._parser.feed(b)
                    if r is None:
                        continue
                    ftype, payload = r
                    if ftype == TYPE_DEBUG:
                        try:
                            print(f"[STM-DBG] {payload.decode('ascii', 'replace')}")
                        except Exception:
                            pass
                        continue
                    self._dispatch_frame(ftype, payload)

    def _dispatch_frame(self, ftype, payload):
        """业务帧的统一入口（接收线程里执行）: 触发钩子 → 入队等主循环消费。"""
        if self._frame_hook is not None:
            try:
                self._frame_hook(ftype, payload)
            except Exception as e:
                print(f"[WARN] 帧钩子异常: {e}")
        self._frame_queue.append((ftype, payload))

    # ---- 连接：同时等两个信号 ----
    def connect(self, should_stop=None, prefer_serial=False):
        """等待真实串口 / 模拟器虚拟串口两个信号之一就绪。

        阻塞直到连接成功或 should_stop() 返回 True（供 SIGTERM/SIGINT 打断等待）。
        返回 True 表示已连接，False 表示被停止信号打断。

        prefer_serial=True（2026-10 加, 重连场景用）: 先给真串口 8s 的优先窗,
        期间不碰模拟器 —— 单片机断电重跑时设备节点要几秒才回来, 而网页模拟器
        只要开着就立刻能连上; 不加这个的话**重连后 main.py 会切到模拟模式**,
        真单片机从此对着空气握手（现象: 车上的 Pi 日志一切正常, 车却不动）。
        首次启动不带本参数, 两个信号平级（谁先就绪用谁）, 与原行为一字不差。
        """
        should_stop = should_stop or (lambda: False)
        from config import (SERIAL_PORT, BAUDRATE, SERIAL_TIMEOUT,
                            SIMULATOR_HOST, SIMULATOR_PORT)

        self.last_io_error = None   # 开始等连接 = 上一次的 I/O 错误不再代表当前链路

        if SERIAL_OPTIONAL:
            # 工具模式：各尝试一次，不阻塞
            s, err = self._open_serial_once(SERIAL_PORT, BAUDRATE, SERIAL_TIMEOUT)
            if s is not None:
                self.ser, self.mode = s, "serial"
                return True
            print(f"[WARN] 串口不可用: {err}")
            sim, err = self._connect_sim_once(SIMULATOR_HOST, SIMULATOR_PORT, SERIAL_TIMEOUT)
            if sim is None:
                print(f"[WARN] 模拟器不可用: {err}")
            self.ser, self.mode = sim, ("simulator" if sim is not None else None)
            return sim is not None

        print(f"[INFO] 等待连接（谁先就绪用谁）: 串口 {SERIAL_PORT} @{BAUDRATE}"
              f" 或 模拟器 {SIMULATOR_HOST}:{SIMULATOR_PORT}"
              + ("（真串口优先窗 8s）" if prefer_serial else ""))

        # 正常模式：两个线程分别等串口 / 模拟器，谁先成功用谁（余者关闭）
        result = {}
        ready = threading.Event()
        lock = threading.Lock()
        status = {"serial": "尝试中...", "sim": "尝试中..."}   # 两侧最近一次失败原因

        def try_serial():
            while not ready.is_set() and not should_stop():
                s, err = self._open_serial_once(SERIAL_PORT, BAUDRATE, SERIAL_TIMEOUT)
                if s is not None:
                    if not ready.is_set():
                        result["ser"], result["mode"] = s, "serial"
                        ready.set()
                    else:
                        s.close()
                    return
                with lock:
                    status["serial"] = err
                time.sleep(2)

        def try_sim():
            if prefer_serial:
                # 真串口优先窗（见 connect 的 docstring）: 等它过了才允许碰模拟器。
                end = time.time() + 8.0
                while not ready.is_set() and not should_stop() and time.time() < end:
                    time.sleep(0.2)
            while not ready.is_set() and not should_stop():
                sim, err = self._connect_sim_once(SIMULATOR_HOST, SIMULATOR_PORT, SERIAL_TIMEOUT)
                if sim is not None:
                    if not ready.is_set():
                        result["ser"], result["mode"] = sim, "simulator"
                        ready.set()
                    else:
                        sim.close()
                    return
                with lock:
                    status["sim"] = err
                time.sleep(2)

        threading.Thread(target=try_serial, daemon=True).start()
        threading.Thread(target=try_sim, daemon=True).start()

        # 可被停止信号打断的等待（Event.wait() 不带超时无法被信号打断）
        t0 = time.time()
        next_log = CONNECT_LOG_INTERVAL / 2.0   # 首次约 5s 就提示, 之后每 10s 一次
        while not ready.is_set() and not should_stop():
            ready.wait(timeout=0.5)
            waited = time.time() - t0
            if waited >= next_log:
                next_log = waited + CONNECT_LOG_INTERVAL
                with lock:
                    s_msg, sim_msg = status["serial"], status["sim"]
                print(f"[WARN] 仍在等待({waited:.0f}s) —— 串口: {s_msg} | 模拟器: {sim_msg}")

        if not ready.is_set():
            self.ser, self.mode = None, None
            return False

        self.ser = result.get("ser")
        self.mode = result.get("mode")
        elapsed = time.time() - t0
        if self.mode == "serial":
            print(f"[INFO] 正常模式：串口 {SERIAL_PORT} 已打开（等待 {elapsed:.1f}s）")
        elif self.mode == "simulator":
            print(f"[INFO] 模拟模式：已连接模拟器 "
                  f"{SIMULATOR_HOST}:{SIMULATOR_PORT}（等待 {elapsed:.1f}s）")
        self._start_rx()   # 专职接收线程（2026-10, 见 __init__ 注释）
        return True

    @staticmethod
    def _open_serial_once(port, baudrate, timeout):
        """尝试打开串口。返回 (串口对象 或 None, 失败原因字符串)。

        区分「设备不存在」（USB 未插 / udev 符号链接未生效）与「打开失败」
        （权限不足 / 被其他进程占用）——两者现场处置完全不同。
        """
        if not os.path.exists(port):
            return None, f"{port} 不存在（USB 未插 / udev 规则未生效）"
        try:
            return serial.Serial(
                port=port, baudrate=baudrate,
                bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE,
                parity=serial.PARITY_NONE, timeout=timeout), ""
        except Exception as e:
            return None, f"{port} 打开失败: {type(e).__name__}: {e}"

    @staticmethod
    def _connect_sim_once(host, port, timeout):
        """尝试连接模拟器。返回 (socket 对象 或 None, 失败原因字符串)。"""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            sock.settimeout(timeout)
            sock.connect((host, port))
            return _SimSocket(sock), ""
        except Exception as e:
            return None, f"{host}:{port} 未就绪: {type(e).__name__}: {e}"

    def reconnect(self, should_stop=None):
        """断开后重新等待两个信号之一（阻塞直到就绪或被停止打断）。

        上一次是**真串口**模式时带 prefer_serial 重连（见 connect 的 docstring）:
        单片机断电重跑的设备枚举窗口内, 别让开着的网页模拟器把链路抢走。
        """
        prefer = (self.mode == "serial")   # 必须在清 mode 之前取
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.mode = None
        if hasattr(self, "_parser"):
            self._parser.reset()
        return self.connect(should_stop=should_stop, prefer_serial=prefer)

    def _mark_disconnected(self):
        try:
            self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.mode = None

    # ---- 发送 ----
    # 2026-10: 加发送互斥。旧版 send_frame 无锁 —— 心跳(主循环)/拍照回执(Timer线程/
    # openmv读线程)/结果帧(主循环)与巡线·靶标流(独立线程, 30~50Hz)会并发写同一个
    # pyserial 对象; 单次 os.write 内核按 tty 锁原子, 但 pyserial 部分写回补时两帧
    # 可在补写间隙交错 → 对端 CRC 校验失败丢帧。帧都很小(≤30B), 锁持有微秒级。
    _tx_lock = threading.Lock()

    def send_frame(self, ftype: int, data: bytes = b""):
        if self.ser is None:
            # ser 是被 I/O 错误置空的 → 报那次错误本身（见 self.last_io_error 注释）
            raise SerialDisconnected(self.last_io_error or "串口未打开")
        packet = pack_frame(ftype, data)
        try:
            with self._tx_lock:
                if self.ser is None:          # 等锁期间被置空（TOCTOU 窗口收掉）
                    raise SerialDisconnected(self.last_io_error or "串口未打开")
                self.ser.write(packet)
        except (serial.SerialException, OSError) as e:
            print(f"[ERROR] 发送失败: {e}")
            self.last_io_error = e
            self._mark_disconnected()
            raise SerialDisconnected(f"发送失败: {e}") from e

    def send(self, data: str, ftype: int = TYPE_COORD):
        """发送数据帧：DATA 补齐到 9 字节 ASCII（与旧 g_aucSerialRxPacket 布局一致）。"""
        arr = []
        for ch in data:
            code = ord(ch)
            if code > 127:
                raise ValueError(f"send() 收到非ASCII字符 U+{code:04X} ('{ch}')")
            arr.append(code)
        while len(arr) < 9:
            arr.append(0)
        self.send_frame(ftype, bytes(arr[:9]))

    def send_ack(self, sig: int):
        """发送 ACK 帧。sig: 0x00=失败, 0x01=成功。"""
        self.send_frame(TYPE_ACK, bytes([sig & 0xFF]))

    # ---- 接收 ----
    def receive(self):
        """非阻塞取**一帧**，返回 (ftype, payload) 或 None。DEBUG 帧由接收线程打印后不入队。

        2026-10 起**不再直接读串口**（读串口移到专职接收线程 _rx_worker, 见 __init__ 注释）
        —— 本方法变成纯消费者: 从帧队列弹出最早一帧（先进先出, 一条不丢）。
        阻塞任务执行期间到达的指令会由接收线程持续入队, 任务返回后按序处理。
        """
        if self._frame_queue:
            return self._frame_queue.popleft()
        return None

    def wait_for_command(self, expected: bytes, timeout_s: float = 15.0):
        """阻塞等待指定 payload，收到返回 True，超时返回 False。

        ⚠️ 2026-10: 等待期间弹出的**不匹配帧不再丢弃** —— 命中/超时后原序塞回
           队列头，由主循环照常分发。旧版直接丢弃是「重连握手窗口吃掉单片机
           第一条指令」的半个根因（另一半是握手后的 flush_input, 见 handler）:
           单片机在 Pi 忙于握手时发来的 "1"/"B" 会凭空消失, 之后两边互相干等。
        """
        t0 = time.time()
        held = []
        try:
            while time.time() - t0 < timeout_s:
                if self.ser is None:
                    return False
                try:
                    r = self.receive()
                    if r is not None:
                        if r[1] == expected:
                            return True
                        held.append(r)
                except (serial.SerialException, OSError):
                    return False
                time.sleep(0.05)
            return False
        finally:
            # 不匹配的帧原序放回队列头（即使超时返回也不丢）
            for f in reversed(held):
                self._frame_queue.appendleft(f)

    def wait_for_ack(self, ack_byte=0xAA, timeout_s=15.0):
        return self.wait_for_command(bytes([ack_byte]), timeout_s)

    def flush_input(self):
        """清空内核接收缓冲 + 重置解析器 + 丢弃未消费的帧队列。

        ⚠️ 持 _rx_lock 与接收线程互斥（它正在喂同一个 parser）。
        """
        with self._rx_lock:
            if self.ser:
                self.ser.reset_input_buffer()
            if hasattr(self, "_parser"):
                self._parser.reset()
            self._frame_queue.clear()

    def close(self):
        self._rx_stop.set()   # 让接收线程退出（它随后对关闭的口 read 会抛错, 双保险）
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            print("[INFO] 串口已关闭")
        self.mode = None

    def restart_program(self):
        """重启程序（20 秒后）。关闭串口后用 os.execl 原地重启。

        这是**单片机下发的 `"r"` 指令**（等 20s 让机械臂/底盘停稳）；链路故障自愈
        走 restart_process()，那个只等 RESTART_DELAY_S。
        """
        print("[INFO] 收到重启指令，20秒后重启...")
        try:
            self.close()
        except Exception as e:
            print(f"[WARN] 关闭串口失败: {e}")
        time.sleep(20)
        self._exec_self()

    def note_link_ok(self):
        """握手成功 = 链路真的通了：清掉连续重启计数。

        调用点只有一个（handler._handshake 收到 ACK 那一刻）—— 别改成「串口一打开就清」：
        链路自愈要治的正是「口打得开、但一读就 Input/output error」那种适配器假死，
        那时 open() 是成功的，只有走完一次收发才算链路真好。
        """
        os.environ.pop(RESTART_COUNT_ENV, None)

    def restart_process(self, reason="", should_stop=None):
        """链路故障自愈：原地重启 main.py（**不是**退出进程）。

        与 restart_program()（单片机下发的重启指令）的区别只有等待时长；两者共用
        _exec_self()。重启后照常等串口（connect 无限等），所以适配器插回来就能自愈。

        `should_stop` 在等待期间逐段查：为真就**放弃重启**并返回 False（调用方按正常退出
        往下走）—— 那几秒里按的 Ctrl+C / 发来的 SIGTERM 必须算数。
        成功 exec 时不会返回（进程镜像已被换掉）。
        """
        n = int(os.environ.get(RESTART_COUNT_ENV, "0")) + 1
        os.environ[RESTART_COUNT_ENV] = str(n)
        delay = min(RESTART_DELAY_S * n, RESTART_DELAY_MAX_S)
        if reason:
            print(f"[ERROR] {reason}")
        print(f"[WARN] {delay:.0f}s 后原地重启 main.py（连续第 {n} 次）"
              f" —— 重启后照常等串口，链路一恢复就重新握手继续跑")
        try:
            self.close()
        except Exception as e:
            print(f"[WARN] 关闭串口失败: {e}")

        should_stop = should_stop or (lambda: False)
        deadline = time.time() + delay
        while time.time() < deadline:
            if should_stop():
                print("[WARN] 重启等待期间收到停止信号，放弃重启")
                return False
            time.sleep(RESTART_SLEEP_SLICE_S)
        self._exec_self()
        return True

    def _exec_self(self):
        """用 os.execl 原地重启本进程（PID 与命令行都不变）。

        ⚠ 调用前必须已经释放串口：os.execl 是「换进程镜像」不是「起新进程」，
        没关掉的 tty 会一直被这个 PID 占着，新镜像再打开就是「设备忙」。
        ⚠ 用 sys.argv 原样重启，所以必须在启动时的工作目录里（systemd 的
        WorkingDirectory / GKS_Web 的 cwd 都满足）；相对路径的 argv[0] 依赖这一点。
        """
        import sys
        python = sys.executable
        os.execl(python, python, *sys.argv)


class _SimSocket:
    """TCP socket 伪装成 pyserial Serial 对象，实现 SerialComm 所需的最小接口。"""
    def __init__(self, sock):
        self._sock = sock
        self.is_open = True
        self._buf = b""

    @property
    def in_waiting(self):
        try:
            data = self._sock.recv(4096)
            if data:
                self._buf += data
            else:
                self.is_open = False   # recv 返回 b'' = 对端已关闭（EOF）
        except (BlockingIOError, socket.timeout):
            pass
        except (ConnectionResetError, BrokenPipeError, OSError):
            self.is_open = False
        return len(self._buf)

    def read(self, n=1):
        try:
            data = self._sock.recv(4096)
            if data:
                self._buf += data
            else:
                self.is_open = False   # EOF：对端已关闭
        except (BlockingIOError, socket.timeout):
            pass
        except (ConnectionResetError, BrokenPipeError, OSError):
            self.is_open = False
        if n >= len(self._buf):
            result = self._buf
            self._buf = b""
            return result
        result = self._buf[:n]
        self._buf = self._buf[n:]
        return result

    def write(self, data):
        try:
            self._sock.sendall(data)
        except (ConnectionResetError, BrokenPipeError, OSError):
            self.is_open = False
            raise

    def reset_input_buffer(self):
        self._buf = b""

    def close(self):
        try:
            self._sock.close()
        except Exception:
            pass
        self.is_open = False


serial_comm = SerialComm()
