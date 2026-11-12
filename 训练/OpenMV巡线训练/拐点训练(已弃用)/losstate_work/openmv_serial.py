# openmv_serial.py — OpenMV 寻线传感器常驻读取线程
#
# OpenMV 是独立传感器，**按需发送**：默认不发，收到树莓派写来的 'G'(开始) 才发，
# 写 'S'(停止) 停（触发方是 handler._cmd_line_start / _cmd_line_stop）：
#   "L,<offset>,<angle>,<valid>[,<state>]\r\n"   @ OPENMV_BAUDRATE (115200)
#   offset: [-100,100] 归一化横向偏差; angle: 0.1° 单位航向角; valid: 0/1
#   state:  2026-09 加的丢线分类（旧固件只有 4 个字段, 解析时缺省为 0）:
#           0=正常 1=有效但交界是曲线(路口) 2=丢线·边界远出画面(该向边界横移)
#           3=丢线·压到边界区(该向路内横移) 4=丢线·方向不明
# 本模块只负责「读 + 缓存最新值」，不做任何控制逻辑——
# 归一化 / PID / 转发单片机在 tasks/_line.py（巡线命令触发后）。
#
# 设计要点（与 serial_comm.py 的区别）：
#   - 模块级单例 openmv_reader，start() 起后台 daemon 线程，立即返回不阻塞
#   - 无模拟器分支、无握手 —— 纯单向数据流，中途接入丢几帧无所谓
#   - 串口打开失败静默重试（30s 一条 WARN，不刷屏），绝不影响主程序启动
#   - ⚠ 「端口打开成功」≠「传感器真在发」：没发 'G' 时空闲零字节是正常的。
#     发了 'G' 后仍长时间无数据（OPENMV_SILENT_WARN_S）才告警，见 _check_silent()。
#     踩过：插错 USB 口时表现为 connected=true + 零字节，界面看着一切正常。
import threading
import time

import serial

import config

# 「已发 'G' 却没有数据」时给日志与 Web 页的同一句话（文案单一来源）
SILENT_HINT = "已发 'G' 但无数据：插错口 / OpenMV 未上电 / 固件没在跑"


def _parse_line(line):
    """解析一行文本帧 "L,<offset>,<angle>,<valid>[,<state>]"。

    第 5 字段可缺（旧 OpenMV 固件）, 缺省为 0。
    返回 (offset, angle, valid, state) 或 None（非法行）。staticmethod 风格便于单测。
    """
    try:
        text = line.strip().decode("ascii", errors="ignore") if isinstance(line, bytes) else line.strip()
        parts = text.split(",")
        if len(parts) not in (4, 5) or parts[0] != "L":
            return None
        offset = int(parts[1])
        angle = int(parts[2])
        valid = int(parts[3])
        state = int(parts[4]) if len(parts) == 5 else 0
        if valid not in (0, 1):
            return None
        if state not in (0, 1, 2, 3, 4):
            return None
        if not (-200 <= offset <= 200):    # 名义范围 ±100，留裕量防噪点
            return None
        if not (-2000 <= angle <= 2000):   # 名义范围 ±900 (0.1°)
            return None
        return offset, angle, valid, state
    except (ValueError, IndexError, AttributeError):
        return None


def _parse_photo_line(line):
    """解析拍照回执一行 "P,<ok>"（OpenMV 存盘后回写）。

    返回 True=成功 / False=失败 / None=**不是回执行**。
    ⚠️ 单独一个函数、不并进 _parse_line: 那条返回 3 元组, 混在一起调用方分不清
       "失败(False)" 与 "不是这行(None)" —— 而失败正是要如实上报的那一半。
    """
    try:
        text = (line.strip().decode("ascii", errors="ignore")
                if isinstance(line, bytes) else line.strip())
        parts = text.split(",")
        if len(parts) != 2 or parts[0] != "P" or parts[1] not in ("0", "1"):
            return None
        return parts[1] == "1"
    except (ValueError, IndexError, AttributeError):
        return None


class OpenMVReader:
    """后台线程持续读取 OpenMV 文本帧，缓存最新一组数据。

    线程安全：缓存读写共用一把锁（20~50Hz 频率下锁开销可忽略）。
    """

    _RETRY_INTERVAL = 2.0     # 打开失败重试间隔（秒）
    _WARN_INTERVAL = 30.0     # 未连接时 WARN 打印间隔（秒）

    def __init__(self):
        self._lock = threading.Lock()
        self._offset = 0
        self._angle = 0
        self._valid = 0
        self._state = 0
        self._timestamp = 0.0            # 最近一帧的 time.time()
        self._connected = False
        self._bad_lines = 0
        self._thread = None
        self._stop_flag = threading.Event()
        self._last_warn = 0.0
        self._ser = None                 # 当前串口对象（None=未打开），供 send_command 并发写
        self._photo_cb = None            # 拍照回执回调（set_photo_callback 注册；见 _run 里调用处）
        self._ser_lock = threading.Lock()
        self._expecting = False          # 已发 'G' 触发发送，理应收到数据
        self._expecting_since = 0.0      # 发 'G' 的时刻（time.monotonic）
        self._last_valid = 0.0           # 最近一帧有效数据的时刻（0 = 本次从未收到）
        self._silent_warned = False      # 本次等待期内是否已打过「无数据」告警

    # ---- 对外接口 ----

    def set_photo_callback(self, fn):
        """注册拍照回执回调 `fn(ok: bool)`。

        ⚠️ 回调在**本类的读线程**里执行, 所以回调方必须自己吞掉异常（发串口帧要 catch
        断线）, 否则一个异常就把读线程带走 —— 巡线数据跟着断。
        """
        self._photo_cb = fn

    def start(self):
        """启动后台读取线程（幂等）。"""
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_flag.clear()
        self._thread = threading.Thread(target=self._run, name="openmv-reader", daemon=True)
        self._thread.start()

    def get_latest(self):
        """返回最新数据快照。

        {"offset", "angle", "valid", "state", "timestamp", "connected", "bad_lines",
         "expecting", "silent_s", "hint"}
        数据是否过期由调用方按 config.LINE_STALE_TIMEOUT 判定（任务侧职责）。
        state: OpenMV 的丢线分类（0 正常 / 1 曲线 / 2 边界远出 / 3 压到边界区 / 4 不明）,
        旧固件恒为 0。expecting/silent_s/hint:「已发 'G' 却没数据」的显式状态 —— 让
        Web 页与日志能区分「端口开着」与「传感器真在发」（插错口时两者长得一模一样）。
        """
        with self._lock:
            silent = self._silent_seconds()
            return {
                "offset": self._offset,
                "angle": self._angle,
                "valid": self._valid,
                "state": self._state,
                "timestamp": self._timestamp,
                "connected": self._connected,
                "bad_lines": self._bad_lines,
                "expecting": self._expecting,
                "silent_s": round(silent, 1) if silent is not None else None,
                "hint": (SILENT_HINT
                         if silent is not None and silent >= config.OPENMV_SILENT_WARN_S
                         else None),
            }

    def close(self):
        """停止线程（进程退出时调用；daemon 线程下不调也无妨）。"""
        self._stop_flag.set()

    def send_command(self, cmd):
        """向 OpenMV 写控制字节（b'G'=开始发 / b'S'=停止发）。

        串口未打开或写失败（断线）返回 False。与读线程并发写同一 Serial 对象：
        pyserial 底层 read/write 是独立 fd 操作，Linux 下安全；ser 引用经锁保护。
        即时性：OpenMV 主循环一直在跑，收到字节后下一采样周期即生效（~33ms）。

        'G' 写成功即进入「等数据」状态（get_latest 的 expecting/silent_s 由此而来）：
        此后收不到数据才算异常；空闲（没发 'G'）时零字节是正常的。
        """
        with self._ser_lock:
            ser = self._ser
        if ser is None:
            return False
        try:
            ser.write(cmd)
        except (serial.SerialException, OSError):
            return False
        with self._lock:
            if cmd == b'G':
                self._expecting = True
                self._expecting_since = time.monotonic()
                self._silent_warned = False
            elif cmd == b'S':
                self._expecting = False
        return True

    # ---- 内部实现 ----

    def _run(self):
        while not self._stop_flag.is_set():
            try:
                ser = serial.Serial(config.OPENMV_PORT, config.OPENMV_BAUDRATE, timeout=0.5)
            except (serial.SerialException, OSError):
                self._set_connected(False)
                self._warn_unconnected()
                time.sleep(self._RETRY_INTERVAL)
                continue

            with self._ser_lock:
                self._ser = ser            # 供 send_command 并发写控制字节
            with self._lock:
                self._connected = True
            try:
                ser.reset_input_buffer()   # 丢弃接入前的陈旧数据
                while not self._stop_flag.is_set():
                    line = ser.readline()
                    if not line:
                        self._check_silent()   # timeout 空转：顺手查「发了 'G' 却没数据」
                        continue
                    # 拍照回执（"P,<ok>"）先试: 它不是 L 帧, 不先拦会被记进 bad_lines。
                    # ⚠️ 判据用 `is not None`（**不是** `if ph:`）—— ph=False 正是"拍照失败",
                    #    写成真值判断会把失败那一半静默吃掉（那就又回到"只报成功"了）。
                    ph = _parse_photo_line(line)
                    if ph is not None:
                        cb = self._photo_cb
                        if cb is not None:
                            try:
                                cb(ph)          # 在**读线程**里执行 → 回调方自己要吞异常
                            except Exception as e:
                                print(f"[WARN] 拍照回执回调出错: {e}")
                        continue
                    parsed = _parse_line(line)
                    if parsed is None:
                        with self._lock:
                            self._bad_lines += 1
                        continue
                    offset, angle, valid, state = parsed
                    with self._lock:
                        self._offset = offset
                        self._angle = angle
                        self._valid = valid
                        self._state = state
                        self._timestamp = time.time()
                        self._last_valid = time.monotonic()
                        self._silent_warned = False   # 有数据了；断流后超时会再告警一次
            except (serial.SerialException, OSError):
                pass   # 断线 → 关闭后回到外层重开
            finally:
                with self._ser_lock:
                    self._ser = None
                try:
                    ser.close()
                except Exception:
                    pass
                self._set_connected(False)
            time.sleep(self._RETRY_INTERVAL)

    def _set_connected(self, value):
        with self._lock:
            self._connected = value

    def _silent_seconds(self):
        """距最近一帧有效数据的秒数；未在等数据（没发 'G'）时返回 None。调用方持 self._lock。"""
        if not self._expecting:
            return None
        return time.monotonic() - max(self._expecting_since, self._last_valid)

    def _check_silent(self):
        """已发 'G' 却长时间无数据 → 限次告警（打开成功 ≠ 传感器在发）。"""
        with self._lock:
            if self._silent_warned:
                return
            silent = self._silent_seconds()
            if silent is None or silent < config.OPENMV_SILENT_WARN_S:
                return
            self._silent_warned = True
        print(f"[WARN] {config.OPENMV_PORT} 已触发发送('G')但 {silent:.0f}s 无数据 —— {SILENT_HINT}")

    def _warn_unconnected(self):
        """未连接时限频打印，避免刷爆日志。"""
        now = time.time()
        if now - self._last_warn > self._WARN_INTERVAL:
            self._last_warn = now
            print(f"[WARN] OpenMV 未连接（{config.OPENMV_PORT}），持续重试中...")


# 模块级单例（与 serial_comm.serial_comm 同风格）
openmv_reader = OpenMVReader()
