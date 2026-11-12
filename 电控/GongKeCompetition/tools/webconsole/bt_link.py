"""蓝牙链路层 —— $B 行协议会话 + 机械臂动作片段生成。

传输在 bt_serial.py（ctypes Win32），这里只管协议与业务：
  · BtLink     一个长连会话：后台读线程收行、send_command 发一条收一条、环形日志
  · list_params / set_param / jog / grip / mode / get_pos   对应固件 app_bt.c 的指令
  · make_arm_snippet   把编辑好的动作列表生成可粘贴进 map.c 的 C 代码
  · load_prefs / save_prefs  记住上次的串口（bt.json）

协议（固件侧 app_bt.c，双端契约）：
    $B,PING                       → $B,PONG,1
    $B,LIST                       → $B,LIST,<n>,<m> + n×$B,P,.. + m×$B,J,.. + $B,OK
    $B,GET,<name>                 → $B,GET,<name>,<value> + $B,OK
    $B,SET,<name>,<val>           → $B,SET,.. + $B,OK / $B,ERR,READONLY|RANGE|NOTFOUND
    $B,MODE,<0|1>                 → $B,MODE,<n> + $B,OK / $B,ERR,NOHANDSHAKE
    $B,JOG,<0..4>,<角度>[,<ms>]    → $B,JOG,.. + $B,OK / $B,ERR,NOTEDIT
    $B,GRIP,<kai_he>,<degree>     → $B,GRIP,.. + $B,OK / $B,ERR,NOTEDIT
    $B,ARM,<a0..a4>,<kai>,<deg>   → $B,ARM,.. + $B,OK / $B,ERR,NOTEDIT|RANGE
    $B,PLAY,<id>                  → $B,PLAY,.. + $B,OK / $B,ERR,NOTEDIT|NOACTION
    $B,POS                        → $B,POS,<a0>..<a4>,<grip>,<done> + $B,OK
每条响应的终止符是 $B,OK 或 $B,ERR,...；异常头（循环 DMA 回绕掉了帧头）回 $B,ERR,SYNC。

**JOG 的 ms 是两种语义**（固件 app_bt.c::bt_cmd_jog）：
  ms > 0 = 走完整 S 曲线（默认 250，比赛语义，台架上记录位姿更稳）；
  ms = 0 = **跟手模式**，把目标直接写进舵机（命中驱动里现成的零时长旁路）——
           拖滑杆时用这个。单次行程超过 8° 时固件自动降级成 60ms 短 S 曲线保护齿轮，
           PC 不需要知道这个阈值。不传 ms 走默认值。

⚠️ `$B,PLAY` 是**阻塞**的（整段走完才回 $B,OK，可能几秒，含表内 STEP_DELAY），
   超时给 `PLAY_TIMEOUT`，且期间不要轮询 $B,POS（那时它也不会应答，只会排在同一条
   命令锁后面空等）。
"""

import json
import os
import re
import threading
import time
from pathlib import Path

import bt_serial
import map_codec                # 清洗规则的唯一来源（sanitize_tag / sanitize_comment_text）

APP_DIR = Path(__file__).resolve().parent
# 模块级常量：测试要能用 mock.patch.object 沙箱化（照 test_host_tools.py 的做法）
BT_JSON = APP_DIR / "bt.json"

DEFAULT_BAUD = 115200
BAUD_CHOICES = (9600, 19200, 38400, 57600, 115200, 230400)

LOG_MAX = 400          # 界面日志条数上限
RX_WINDOW_MAX = 200    # send_command 回扫用的 rx 行保留量

# 固件的步骤参数定义（app_script.h），片段生成必须与之一致
ARM_STEP_WAIT = "STEP_ARM_GROUP_WAIT"
ARM_STEP_NOWAIT = "STEP_ARM_GROUP_NO_WAIT"
ARM_PRIM_WAIT = "vArmGroupMoveWait"
ARM_PRIM_NOWAIT = "vArmGroupMoveNoWait"
GRIP_STEP = "STEP_SET_ZHUASHOU_WAIT"
GRIP_PRIM = "vArmSetZhuaShou"

TAG_MAX_LEN = 24

# $B,PLAY 是阻塞的（整段含表内延时都走完才回 OK）—— 超时给足
PLAY_TIMEOUT = 15.0

# 建链后的 PING 握手重试（BtLink._handshake）：CreateFileW 成功只代表 RFCOMM 链路
# 建好，模块的 SPP 数据通路/流控未必立刻就绪，模块刚上电/刚从省电醒来时头几个字节
# 还可能被丢 —— PING 一次没应答就判死，会让「再点一次就好」变成常态。
# 首试给足 2s（保住慢启动的设备）；重试 1s 就够（真应答的往返只有几 ms，固件实测
# 链路 4~6ms）。最坏 ~4.6s，失败本就是例外路径，连接请求等得起。
PING_ATTEMPTS = 3
PING_FIRST_TIMEOUT = 2.0
PING_RETRY_TIMEOUT = 1.0
PING_RETRY_DELAY = 0.3


class BtError(Exception):
    """协议/链路错误，消息可直接展示给用户。"""


# ==================== 纯函数：解析 ====================

def _to_float(text, default=0.0):
    try:
        return float(text)
    except (TypeError, ValueError):
        return default


def _to_int(text, default=0):
    try:
        return int(float(text))
    except (TypeError, ValueError):
        return default


def parse_param_line(line):
    """'$B,P,name,label,group,type,rw,warn,lo,hi,unit,value[,options]' → dict；不匹配回 None。

    字段顺序与 app_bt.c::bt_cmd_list 一一对应，改一处必须改两处。
    ⚠️ 末尾 options 是**可选**字段（"0=主口那颗|1=副口那颗"）：只有固件那条 pcOptions
       非空才出现 ⇒ 合法长度是 12 **或** 13。别写成 `!= 13`（那会把绝大多数普通参数
       全判成无效行、参数面板直接空掉）。用 `|` 分隔是因为 $B 协议无转义 —— 字段里
       出现逗号会把整条记录串行。
    """
    parts = line.split(",")
    if len(parts) not in (12, 13) or parts[0] != "$B" or parts[1] != "P":
        return None
    return {
        "name": parts[2],
        "label": parts[3],
        "group": parts[4],
        "type": parts[5],
        "rw": parts[6] == "1",
        "warn": parts[7] == "1",
        "min": _to_float(parts[8]),
        "max": _to_float(parts[9]),
        "unit": parts[10],
        "value": parts[11],
        "options": parse_options(parts[12]) if len(parts) == 13 else None,
    }


def parse_options(text):
    """"0=主口那颗|1=副口那颗" → [{"value": "0", "label": "主口那颗"}, …]；解析不出回 None。

    前端见到非空 options 就把该参数渲染成下拉框（「蓝牙」Tab）。"""
    out = []
    for piece in text.split("|"):
        value, sep, label = piece.partition("=")
        value, label = value.strip(), label.strip()
        if sep and value and label:
            out.append({"value": value, "label": label})
    return out or None


def parse_joint_line(line):
    """'$B,J,idx,name,min,max,angle' → dict；不匹配回 None。"""
    parts = line.split(",")
    if len(parts) != 7 or parts[0] != "$B" or parts[1] != "J":
        return None
    return {
        "index": _to_int(parts[2]),
        "name": parts[3],
        "min": _to_float(parts[4]),
        "max": _to_float(parts[5]),
        "angle": _to_float(parts[6]),
    }


def parse_pos_line(line):
    """'$B,POS,a0..a4,grip,done' → {'angles':[5], 'gripper':float, 'done':bool}。"""
    parts = line.split(",")
    if len(parts) != 9 or parts[0] != "$B" or parts[1] != "POS":
        return None
    return {
        "angles": [_to_float(v) for v in parts[2:7]],
        "gripper": _to_float(parts[7]),
        "done": parts[8] == "1",
    }


def sanitize_tag(name):
    """把用户起的动作名洗成能安全放进 map.c 字符串字面量的标签。

    规则本体已收敛到 map_codec.sanitize_tag（**唯一来源**, 那边 docstring 有完整理由）:
    这里只补上机械臂这条路的两个约定 —— 长度上限 TAG_MAX_LEN（对外行为, 有测试钉着）
    与空名字的兜底。别再把规则抄一份回来。
    """
    return map_codec.sanitize_tag(name, TAG_MAX_LEN, "臂组动作")


def _fmt_f1(value):
    """动作参数里的 float：带 .0f（C 里是 float 字段）。"""
    return "%.1ff" % float(value)


def _fmt_gripper_step(tag, kai_he, degree):
    """生成夹爪那一步（SCRIPT_STEP_I2），版式与 server.py::_serialize_step 一致。

    整数参数带 `u` 后缀：这是「地图」Tab 的 _normalize_param 对无符号整数槽的写法，
    跟它保持一致，贴回去的片段才和 Tab 自己写出来的行一模一样。

    ⚠️ **自己洗一遍 tag**：本函数有两个调用方, 而 make_arm_snippet 会先洗好再传进来、
       arm_action 的 grip 分支（_render_item）却是拿用户输入**直接**调它 —— 不在这儿补
       一层, 手打的夹爪积木名就会带着 `"` / `//` / 不成对括号写进 app_arm_action.c。
       sanitize_tag 是幂等的, 所以两条路各洗一遍不会产生两个结果。
    """
    kai_he = 1 if _to_int(kai_he) else 0
    degree = max(0, min(100, _to_int(degree)))
    head = "    %s (%s %s" % (
        "SCRIPT_STEP_I2".ljust(18),
        (GRIP_STEP + ",").ljust(37),
        '"%s"' % sanitize_tag(tag),
    )
    return "%s, %du, %du),  // %s()" % (head, kai_he, degree, GRIP_PRIM)


def make_arm_snippet(items, wait=True):
    """动作列表 → 可粘贴进 map.c 的 C 代码行。

    每个 item: {'name': str, 'angles': [5 个角度], 'gripper': {'enabled','kai_he','degree'}}
    返回行列表（不含末尾换行）。版式逐字符对齐 server.py::_serialize_step，
    这样贴回 map.c 后与原生行同款，「地图」Tab 也能原样显示/往返。
    """
    step = ARM_STEP_WAIT if wait else ARM_STEP_NOWAIT
    prim = ARM_PRIM_WAIT if wait else ARM_PRIM_NOWAIT
    macro = "SCRIPT_STEP_F5"

    out = []
    for item in items or []:
        tag = sanitize_tag(item.get("name"))
        angles = list(item.get("angles") or [])[:5]
        while len(angles) < 5:
            angles.append(0.0)

        head = "    %s (%s %s" % (
            macro.ljust(18),
            (step + ",").ljust(37),
            '"%s"' % tag,
        )
        vals = ", ".join(_fmt_f1(a) for a in angles)
        out.append("%s, %s),  // %s()" % (head, vals, prim))

        grip = item.get("gripper") or {}
        if grip.get("enabled"):
            out.append(_fmt_gripper_step(
                sanitize_tag("%s-夹爪" % tag),
                grip.get("kai_he"), grip.get("degree")))
    return out


# ==================== 偏好（记住上次的串口） ====================

def load_prefs():
    """读 bt.json → dict（缺失/损坏回 {}）。"""
    try:
        data = json.loads(BT_JSON.read_text(encoding="utf-8"))
        return data if isinstance(data, dict) else {}
    except (OSError, ValueError):
        return {}


def save_prefs(port, baud):
    """原子写 bt.json（tmp + os.replace，不留残渣），同 host_tools.save_phrases。"""
    try:
        tmp = BT_JSON.with_suffix(".json.tmp")
        tmp.write_text(json.dumps({"port": port, "baud": int(baud)},
                                  ensure_ascii=False, indent=2), encoding="utf-8")
        os.replace(tmp, BT_JSON)
        return True
    except OSError:
        return False


# ==================== 会话 ====================

class BtLink:
    """一条蓝牙串口长连。

    并发模型：
      · **ReadFile 只有一个属主** —— 后台读线程。send_command 只写口 + 等事件，
        绝不自己读（否则与读线程抢字节：日志出空洞、响应终止符可能抓不到）。
      · 命令用一把锁串行化。锁挂在**实例**上，而 server.py 用的是模块级单例
        （ThreadingHTTPServer 每请求一线程，每请求一把锁等于没锁）。
    """

    def __init__(self):
        self._port = None
        self._name = ""
        self._baud = DEFAULT_BAUD
        self._log = []                 # [{'dir','text','seq','t'}]
        self._log_lock = threading.Lock()
        self._rx_seq = 0
        self._event = threading.Event()
        # RLock: connect/_connect_locked 内部要调 disconnect / send_command（都取这把锁）
        self._cmd_lock = threading.RLock()
        self._thread = None
        self._stop = threading.Event()
        self._error = ""

    # ---- 状态 ----

    @property
    def is_connected(self):
        return self._port is not None and self._port.is_open

    @property
    def port_name(self):
        return self._name

    @property
    def baud(self):
        return self._baud

    def state(self):
        return {
            "connected": self.is_connected,
            "port": self._name,
            "baud": self._baud,
            "error": self._error,
            "log": self.get_log(),
        }

    def get_log(self, since=0):
        with self._log_lock:
            return [dict(e) for e in self._log if e["t"] >= since][-120:]

    # ---- 生命周期 ----

    def connect(self, port, baud=DEFAULT_BAUD):
        """打开串口并握手（PING）。失败抛 BtError，且保证不留下半开的句柄。

        ⚠️ 整段持 `_cmd_lock`（与 send_command 同一把）: LINK 是模块级单例、被所有
        请求线程共用, 不持锁的话「A 客户端正在等 $B,LIST（持锁 ~850ms）」时 B 客户端
        一点「断开」就把 `self._port` 置空/关掉 → A 的 send_command 里 write 抛异常
        （或退化成超时、`_error` 被写上）。CLAUDE.md 写的「多人点动只是排队」
        只对走这把锁的那批指令成立 —— connect/disconnect 原来正是漏的两个。
        ⚠️ 用 RLock: connect 内部要调 disconnect 与 send_command（两者都会取同一把锁）。
        """
        with self._cmd_lock:
            return self._connect_locked(port, baud)

    def _connect_locked(self, port, baud=DEFAULT_BAUD):
        self._disconnect_locked()

        sp = bt_serial.SerialPort(port, baud)
        sp.open()                     # 失败抛 bt_serial.SerialError（消息已是中文）
        self._port = sp
        self._name = port
        self._baud = int(baud)
        self._error = ""
        self._stop.clear()
        self._rx_seq = 0
        with self._log_lock:
            self._log = []

        self._thread = threading.Thread(target=self._reader, name="bt-reader", daemon=True)
        self._thread.start()

        try:
            self._handshake(port)
        except Exception:
            self._disconnect_locked()
            raise

        save_prefs(port, baud)
        return True

    def _handshake(self, port):
        """建链后的 PING 握手：带重试，并区分「答了但回错」与「一声不吭」。

        send_command 的 status 有 ok/err/timeout 三种。老代码 `!= "ok"` 一刀切成
        「没有应答」，把 err（设备明明活着、回的是 $B,ERR,SYNC —— 固件对垃圾行的
        语义本来就是「请你重发」，见 app_bt.c::bt_handle_line）也当成死设备直接断开，
        等于把双方约定好的重试单方面放弃。另外建链瞬间的垃圾行/字节错位只发生在
        头几个字节上，重试一次基本就能干净应答 —— 这正是「经常连不上、重连就好」
        的主要来源。
        """
        status, term = "timeout", ""
        for attempt in range(PING_ATTEMPTS):
            timeout = PING_FIRST_TIMEOUT if attempt == 0 else PING_RETRY_TIMEOUT
            _lines, status, term = self.send_command("$B,PING", timeout=timeout)
            if status == "ok":
                return
            if attempt < PING_ATTEMPTS - 1:
                time.sleep(PING_RETRY_DELAY)

        if status == "err":
            # 设备有应答，只是回了错误行 —— 报错带上原文与翻译，别混进「没有应答」
            raise BtError("连上了 %s，设备活着但握手失败：%s（%s）。"
                          "已自动重试 %d 次，多为建链瞬间的字节错位，请断开重连一次"
                          % (port, term, _err_text(term), PING_ATTEMPTS))
        # 全程真静默：字节根本没到固件（或固件没在跑）
        raise BtError("连上了 %s 但设备没有应答（已自动重试 %d 次；模块上电了吗？"
                      "固件在跑吗？模块与主板间的串口是 115200 吗 —— "
                      "PC 端这个波特率下拉不影响蓝牙链路）" % (port, PING_ATTEMPTS))

    def disconnect(self):
        with self._cmd_lock:              # 理由见 connect 的注释
            self._disconnect_locked()

    def _disconnect_locked(self):
        self._stop.set()
        self._event.set()
        thread, self._thread = self._thread, None
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        port, self._port = self._port, None
        self._name = ""
        if port is not None:
            port.close()

    # ---- 读线程 ----

    def _reader(self):
        port = self._port
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = port.read(4096)
            except bt_serial.SerialError as exc:
                self._error = str(exc)
                self._append("sys", "✗ %s" % exc)
                break
            if not chunk:
                continue
            buf += chunk
            if len(buf) > 1000000:
                # 设备跑飞/接错线时可能狂吐不带换行的字节 —— 半包缓冲必须封顶
                buf = b""
            # 按字节累积、按 \n 切行后再解码：标签是 UTF-8 中文，
            # 若在解码后的字符串上切行，多字节字符被拆包时会出乱码。
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                text = raw.decode("utf-8", "replace").strip("\r").strip()
                if text:
                    self._append("rx", text)

    def _append(self, direction, text):
        with self._log_lock:
            seq = None
            if direction == "rx":
                self._rx_seq += 1
                seq = self._rx_seq
            self._log.append({"dir": direction, "text": text, "seq": seq,
                              "t": time.time()})
            if len(self._log) > LOG_MAX:
                del self._log[:len(self._log) - LOG_MAX]
        if direction == "rx":
            self._event.set()

    def _rx_since(self, seq):
        with self._log_lock:
            return [e["text"] for e in self._log
                    if e["dir"] == "rx" and e["seq"] is not None and e["seq"] > seq]

    # ---- 命令 ----

    def send_command(self, cmd, timeout=1.5):
        """发一条指令并收到终止符。返回 (lines, status, terminator)。

        status ∈ ok/err/timeout；lines 是应答的**正文行**（不含终止符）。
        terminator 是那条 $B,OK / $B,ERR,... 原文——出错时错误码只在它里面，
        正文行是空的，所以必须单独回传（别去猜 lines[-1]）。
        """
        with self._cmd_lock:
            if not self.is_connected:
                raise BtError("蓝牙未连接")
            self._append("tx", cmd)
            with self._log_lock:
                start = self._rx_seq
            try:
                self._port.write((cmd + "\r\n").encode("utf-8"))
            except bt_serial.SerialError as exc:
                self._error = str(exc)
                raise BtError(str(exc))

            deadline = time.time() + timeout
            while time.time() < deadline:
                self._event.wait(0.05)
                self._event.clear()
                lines = self._rx_since(start)
                for idx, line in enumerate(lines):
                    if line.startswith("$B,OK") or line.startswith("$B,ERR"):
                        ok = line.startswith("$B,OK")
                        return lines[:idx], ("ok" if ok else "err"), line
            return self._rx_since(start), "timeout", ""

    def _command(self, cmd, timeout=1.5):
        """send_command + 统一的错误抛出。"""
        lines, status, terminator = self.send_command(cmd, timeout=timeout)
        if status == "timeout":
            raise BtError("设备无响应（超时 %s；蓝牙掉线或固件卡住？）" % cmd)
        if status == "err":
            raise BtError(_err_text(terminator))
        return lines

    # ---- 业务 ----

    def list_params(self):
        """拉整张参数表（设备自述）→ {'params': [...], 'joints': [...]}。

        ⚠️ LIST 的应答是 ~17KB 的大突发（两百多行 $B,P 一次倾泻）, 蓝牙 SPP 上
        **偶发丢行/掰碎**是正常磨损 —— 所以零参数时**自动重试一次**; 再失败才报错,
        且报错里带上**当时实际收到了什么**（行数 + 前几行原文）: 下一次现场一眼
        就能分辨「丢行」（首行是 $B,LIST,n,m 但 P 行没跟上来）/「链路质量」
        （行是乱码）/「设备真的回了空表」（$B,LIST,0,m）, 不用再猜协议版本。"""
        diag = "没有收到任何正文行"
        for _attempt in (1, 2):
            lines = self._command("$B,LIST", timeout=6.0)
            params, joints = [], []
            for line in lines:
                p = parse_param_line(line)
                if p:
                    params.append(p)
                    continue
                j = parse_joint_line(line)
                if j:
                    joints.append(j)
            if params:
                joints.sort(key=lambda j: j["index"])
                return {"params": params, "joints": joints}
            head = [l for l in lines if l][:3]
            diag = ("收到 %d 行正文, 前 %s 行: %s"
                    % (len(lines), min(3, len(lines)),
                       " ⏎ ".join(l[:70] for l in head) if head else "（空）"))
        raise BtError("设备回了 $B,LIST 但没解析出任何参数 —— %s" % diag)

    def set_param(self, name, value):
        # 协议按行分帧: 名/值里的 CR/LF 会截断或伪造命令（蓝牙串口当输入框的人不多,
        # 但成本是一行 replace）—— 展平成空格
        name = str(name).replace("\r", " ").replace("\n", " ")
        value = str(value).replace("\r", " ").replace("\n", " ")
        return self._command("$B,SET,%s,%s" % (name, value), timeout=2.0)

    def jog(self, index, angle, ms=250):
        return self._command("$B,JOG,%d,%.1f,%d" % (_to_int(index), float(angle),
                                                   _to_int(ms, 250)), timeout=2.0)

    def grip(self, kai_he, degree):
        return self._command("$B,GRIP,%d,%d" % (1 if _to_int(kai_he) else 0,
                                               _to_int(degree)), timeout=2.0)

    def arm_pose(self, angles, kai_he, degree):
        """$B,ARM —— 播放一组位姿（5 轴同时启 S 曲线 + 夹爪），立即返回。

        机械臂 Tab 的「单条播放」与 PC 编排的整序列播放都走它：一条指令 5 轴同时
        启动，比 5 条 jog 少 4 次往返、也不会串行启动。到位与否用 get_pos() 轮询。
        """
        vals = [_to_float(a) for a in list(angles or [])[:5]]
        while len(vals) < 5:
            vals.append(0.0)
        return self._command("$B,ARM,%s,%d,%d" % (
            ",".join("%.1f" % v for v in vals),
            1 if _to_int(kai_he) else 0, _to_int(degree)), timeout=2.0)

    def get_param(self, name):
        """$B,GET,<name> —— 读一个参数的当前值（轻量; 'list' 是全表 ~10KB, 轮询用它太重）。

        返回 float（固件按 $B,GET 的 value 段回; 解析失败抛 BtError）。
        """
        lines = self._command("$B,GET,%s" % (name or "").strip(), timeout=1.5)
        for line in lines:
            parts = line.split(",")
            if len(parts) >= 4 and parts[0] == "$B" and parts[1] == "GET":
                return _to_float(parts[3])
        raise BtError("$B,GET 响应解析失败")

    def play_action(self, index):
        """$B,PLAY —— 让固件播放**已编译的**动作预设整段（阻塞, 含表内延时）。

        与路线里的 STEP_ARM_PLAY_WAIT 是同一个固件函数，所以这是「台架试跑 == 比赛
        跑路线」的验证路径。`index` 是动作 ID（= emArmActionIdTdf 的值）。
        """
        return self._command("$B,PLAY,%d" % _to_int(index), timeout=PLAY_TIMEOUT)

    def play_chassis_action(self, index):
        """$B,CPLAY —— 让固件播放**已编译的**底盘动作组整段（非阻塞, 2026-10 加）。

        与路线里的 STEP_CHASSIS_ACTION_PLAY_WAIT 是同一个固件播放器（预览在
        vChassisActionTask 里走同一条 c_vChActPlay），台架试跑 == 比赛跑路线。
        与 $B,PLAY 的关键差别: 固件**立刻应答**（播放丢给专用任务），所以这里
        timeout 用普通的 2s 就够；「播完没有」由前端轮询参数 ch_busy（1=在播）。
        ⚠️ 需先 set_mode(True) 进编辑模式（固件侧 NOTEDIT 门）。
        """
        return self._command("$B,CPLAY,%d" % _to_int(index), timeout=2.0)

    def push_chassis_action(self, gid, items):
        """$B,CPUSH + $B,CIT×N —— 把控制台**当前这一份**动作组推进固件 RAM 草稿区。

        预览前调它: 预览（$B,CPLAY）命中同 id 草稿就播草稿 —— 在线编辑立即能预览,
        不用每次 Keil 重编译。⚠️ 路线/比赛仍只读 FLASH 编译表（固件铁律）, 改完
        要上比赛必须 保存 + 重编译 + 烧录。
        ⚠️ 两类条目推不了, 调用方先滤掉并提示（返回值不含它们）:
          · raw（手写只读行）—— 固件草稿只有四类结构化条目;
          · disabled（被「启」眼睛禁用的）—— 编译版里它们是注释行, 同样不存在;
            **漏了这条, 预览就会把禁用的段当启用播出来**（台架实测踩过）。
        """
        kind_map = {"move": 0, "turn": 1, "arm": 2, "arm_wait": 3, "pivot": 4}
        rows = []
        for it in items or []:
            kind = kind_map.get(it.get("kind"))
            if kind is None or it.get("disabled"):
                continue
            rows.append("$B,CIT,%d,%d,%d,%.2f,%.2f,%d,%d,%d,%d,%d,%d" % (
                kind,
                _to_int(it.get("start")),
                int(_to_float(it.get("dur"))),
                _to_float(it.get("wx")),
                _to_float(it.get("wy")),
                int(_to_float(it.get("speed"))),
                int(_to_float(it.get("accel"))),
                int(_to_float(it.get("delta"))),
                int(_to_float(it.get("arm_id"))),
                1 if it.get("hold") else 0,
                1 if it.get("abs") else 0))
        out = self._command("$B,CPUSH,%d,%d" % (_to_int(gid), len(rows)), timeout=2.0)
        for row in rows:
            out += self._command(row, timeout=1.5)
        return out

    def stop_chassis_action(self):
        """$B,CSTOP —— 停底盘动作组预览（置停止位 + 掐臂序列, ≤20ms 生效）。

        刹车指令永远合法（不受编辑模式门槛），预览跑飞时的第一反应就是它。
        """
        return self._command("$B,CSTOP", timeout=2.0)

    def reset_chassis_heading(self):
        """$B,CHR0 —— 航向关系重置：把世界航向累加器对齐到当前 IMU 工作航向。

        台架常态「上电 → 手动把车搬回出发区 → 点播放」的关键一步: 物理朝向已经
        转过, 账面还是 0°, 相对转弯会转错、保持层会打架。摆好车按一下, 当前车头
        方向重新成为 0° 基准。不动车、不受编辑模式门槛; 应答里带新的基准角。
        """
        return self._command("$B,CHR0", timeout=2.0)

    def handshake_yield(self):
        """$B,HS —— 投一次「握手让出」（与 OLED 第 3 页那个按钮同一入口）。

        脚本任务取走请求后，正在跑的路线在步骤边界中止，ucRpiHandshake 落回
        WAITING 进阶段 1 → bt_hs_wait 拉高 → 这时 `$B,MODE,1` 才能进来。
        给「机械臂」Tab 的「连接 → 自动让位」用（2026-10 加）：Pi 在线时脚本多半
        在跑，不让位就 NOHANDSHAKE。

        ⚠️ 只投递不等待：让位真正生效要等路线当前这一步跑完（大多数 <1s），
        所以调用方 MODE,1 拿到 NOHANDSHAKE 时应稍等重试，而不是立刻报死。
        """
        return self._command("$B,HS", timeout=2.0)

    def set_mode(self, on):
        """进/出机械臂编辑模式。

        ⚠️ 退出**不带**任何「回默认位姿」参数了（那个功能已去掉）：脚本一离开让位状态
        就接着跑路线，这里再让舵机回位会盖掉路线那一步的臂动作（固件是后写者胜）。
        要回上电默认位姿就在路线里排一条组动作积木。
        2026-10 起固件在「退出且正处让位窗」时会**立刻**踢 Pi 重启客户端（原来自愈
        要等 6s 重试闸 + Pi 重启 20s+），所以退出后链路几十秒内自己回到 OK，无需手工。
        """
        return self._command("$B,MODE,%d" % (1 if on else 0), timeout=2.0)

    def get_pos(self):
        lines = self._command("$B,POS", timeout=1.5)
        for line in lines:
            pos = parse_pos_line(line)
            if pos:
                return pos
        raise BtError("$B,POS 响应解析失败")


def _err_text(err_line):
    """把 '$B,ERR,<CODE>[,args]' 翻成人话。"""
    parts = (err_line or "").split(",")
    code = parts[2] if len(parts) >= 3 else "?"
    extra = parts[3:]
    table = {
        "NOHS": "设备不在握手等待阶段——编辑模式只能在开机等树莓派握手时进入",
        "NOHANDSHAKE": "设备不在握手等待阶段——编辑模式只能在开机等树莓派握手时进入",
        "NOTEDIT": "设备不在机械臂编辑模式（可能树莓派已握手，脚本已恢复运行）",
        "READONLY": "该项是只读的",
        "NOTFOUND": "设备上没有这个参数（固件版本旧了？）",
        "BADVAL": "数值格式不对",
        "ARGS": "指令参数不足",
        "BUSY": "上一条脚本请求还没被设备取走，稍等一下再发",
        "NOJOINT": "没有这个关节",
        "NOACTION": "设备上没有这个动作预设（固件没重编译？先在「机械臂」Tab 保存一次）",
        "SYNC": "串口帧头丢失（循环 DMA 回绕），这条指令被丢弃了，请重试",
        "UNKNOWN": "设备不认识这条指令",
    }
    if code == "RANGE" and len(extra) >= 2:
        return "超出范围（允许 %s ~ %s）" % (extra[0], extra[1])
    return table.get(code, "设备返回错误: %s" % code)


# 模块级单例：server.py 的每个请求线程共用这一条连接
LINK = BtLink()
