# simulator.py — 无头单片机模拟器（GKS_Web 内置，按需启动）
#
# 模拟单片机客户端：在 127.0.0.1:9997 监听，main.py 进入模拟模式时会主动连上来，
# 之后网页可发任务指令帧给 main.py，用于无单片机单端调试 / 跳前置流程 / 快速定位问题。
#
# 与 GKS_Competition 的「真实串口」平级：本模拟器默认不监听，由网页「启动/停止模拟器」
# 开关控制。main.py 会同时等待真实串口与这里两个信号，谁先就绪就进哪个模式。
#
# 移植自 cv_python/simulator.py 的 SimulatorServer（去 PyQt，改线程 + 锁）。
# protocol 由 app.py 先把 GKS_Competition 目录加入 sys.path 后导入。
import struct
import threading
import time
import socket
from collections import deque

import protocol
from protocol import pack_frame, FrameParser, TYPE_CMD, TYPE_COORD, TYPE_ACK

# 指令表：与 GKS_Competition/handler.py::_COMMANDS 保持一致
# 靶标流帧（§6f TAG=0x05）日志摘录间隔[秒]：流是 ~20Hz，逐帧写会刷爆日志环。
STREAM_LOG_S = 1.0
# 流帧间隔超过此值 = 上一次起流已经停了，再来的一帧算新的一轮（计数/计时归零）。
STREAM_GAP_S = 2.0

COMMANDS = [
    ("1",   "1 二维码识别",   "触发二维码检测任务"),
    ("2",   "2 开始巡线",     "OpenMV 寻线转发（PID 结果实时回传）"),
    ("3",   "3 停止巡线",     "停止寻线转发线程"),
    ("4",   "4 人质识别",     "主摄 YOLO 识别圆柱/圆锥/腰鼓，按从左到右回传"),
    ("5",   "5 色球识别",     "主摄 HSV 识别红/绿/蓝顺序，按从左到右回传"),
    ("6",   "6 靶标识别",     "识别三色靶盘并定位靶心（跨色尺寸一致性），按从左到右回传。"
                              "可带 is_use_picam（写法 \"6,6000,1\"）= 优先用树莓派 CSI 那颗, "
                              "它打不开/掉线/看不到靶时自动退回主摄（回帧 status 的 bit7 报实际用了哪台）"),
    ("7",   "7 靶标流起流",   "小窗跟踪靶盘，高频回传靶心流帧（瞄准闭环的测量源, 见协议 §6f）。"
                              "同样可带 is_use_picam（\"7,0,1\"）= 优先 CSI, 起不来/中途掉线整条改用主摄"),
    ("8",   "8 靶标流停流",   "停止靶标流跟踪线程"),
    ("end", "end 停止循环",   "停止 main.py 主循环等待"),
    ("r",   "r 重启程序",     "20 秒后重启 main.py"),
    ("sd",  "sd 系统关机",    "树莓派整机关机（单片机 OLED 菜单用；只收不回）"),
    ("rb",  "rb 系统重启",    "树莓派整机重启（同上）"),
    ("9",   "9 拍照",         "让 OpenMV 存一张照片到它自己盘上的 photos/（只发不回, 见协议 §6a）"),
    ("A",   "A 矩形地板",     "主摄识别色球区黑胶带矩形，回传板心像素（回 0x07 帧, 见协议 §6h）"),
    ("G",   "G 看路",         "主摄找地面上的黄色区 → 回传「左边缘的 x」作 Δx（回 0x08 帧, 见协议 §6i）"),
    ("H",   "H 看路校准",     "主摄拟合黄区直边的图像方向角（陀螺漂移的视觉基准, 只测不改: 不动车不写航向, 回 0x09 帧, 见协议 §6j）。"
                              "标定: 车摆基准位读单片机 roadyaw_ang 填 roadyaw_ideal, 原地转已知角定 roadyaw_dir"),
    ("B",   "B 预热扫码相机", "提前开好扫码相机并持续取帧；随后的 \"1\" 直接解码（只发不回, 省 ~1.3s）。"
                              "可带**使用次数**后缀（写成 \"B,2\"）：这台相机接下来要被几个任务用，"
                              "Pi 把相机租约的空闲窗口放宽成 次数×LEASE_IDLE_TIMEOUT_S"),
    ("C",   "C 预热主摄",     "同上，给随后的人质/色球/靶标识别用（只发不回, 省 ~1.3s）。"
                              "同样可带次数后缀（\"C,2\"）。"
                              "⚠️ 再带一个 is_use_picam（\"C,2,1\"）= 预热的是**树莓派 CSI** 那颗"
                              "（平移射靶看靶用的那台）—— 要与随后带 pi 标志的识别配对才不会白预热"),
    ("D",   "D 主摄扫码",     "二维码识别但用主摄（与 \"1\" 同链同帧, 只换相机；回 §6a 的 QR 帧）"),
    ("E",   "E 副摄自研扫码", "二维码识别用**自研难度算法**（几何重建+候选集, 治斜视/小码；比 \"1\" 慢，超时见 QR_DIFF_TIMEOUT；回 §6a 的 QR 帧）"),
    ("F",   "F 主摄自研扫码", "同上但用主摄（斜视扫码那条路；先 \"C\" 预热主摄）"),
    ("x",   "x 取消当前任务", "取消 Pi 侧正在跑的任务/巡线流/靶标流（只收不回; 单片机中止脚本或重新握手时自动发）"),
    ("L",   "L 补光亮度",     "设 OpenMV 补光亮度（只发不回）。写法 \"L,70\"（0~100, 0=灯灭）; 裸 \"L\" = 重发 config 记住的值。"
                              "单片机 OLED「巡线→调参」与积木 STEP_OPENMV_LED 发的是同一条; 值会持久化并在巡线起流前自动重发"),
]


class SimulatorServer(threading.Thread):
    """TCP 服务线程：按需监听、解析协议帧、自动握手应答、可发指令。"""

    def __init__(self, host="127.0.0.1", port=9997, max_log=500):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self._stop = False          # 线程退出标志
        self._enabled = False       # 是否在监听（网页开关控制）
        self._client = None
        self._lock = threading.Lock()
        self._log = deque(maxlen=max_log)
        self._last_hostage = None   # 最近一帧人质结果（结构化，供网页显示顺序/坐标）
        self._last_color = None     # 最近一帧色球结果（同上）
        self._last_rect = None      # 最近一帧矩形地板结果（同上）
        self._last_road = None      # 最近一帧看路结果（同上）
        self._last_road_yaw = None  # 最近一帧看路校准结果（0x09, 结构化）
        self._last_target = None    # 最近一帧靶标结果（同上，含半径）
        self._last_stream = None    # 最近一帧靶标**流**帧（§6f TAG=0x05，同上）
        # 流帧的摘录状态：它是 ~20Hz 的连续流，逐帧写日志会瞬间刷爆 _log（maxlen=500），
        # 所以只按 STREAM_LOG_S 摘一条（帧率 + 靶数），见 _stream_frame。
        self._stream_n = 0
        self._stream_t0 = 0.0         # 本轮起流时刻（算平均帧率）
        self._stream_last_ts = 0.0    # 上一帧到达时刻（判"是不是新的一轮"）
        self._stream_last_log = 0.0
        self._stream_max = 3        # 单帧靶数上限（与 §6f 的 count ≤ 3 对齐）

    # ---- 日志（线程安全）----
    def _emit(self, direction, text):
        with self._lock:
            self._log.append((time.strftime("%H:%M:%S"), direction, text))

    def recent_log(self):
        with self._lock:
            return list(self._log)

    def clear_log(self):
        with self._lock:
            self._log.clear()

    # ---- 开关 / 状态 ----
    def enable(self):
        if not self._enabled:
            self._enabled = True
            self._emit("sys", f"模拟器已启动，监听 {self.host}:{self.port}，等待 main.py 连接...")

    def disable(self):
        if self._enabled:
            self._enabled = False
            self._emit("sys", "模拟器已停止")
        # 断开当前客户端，让 main.py 端感知断开并重新等待（可切回真实串口）
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None

    def is_enabled(self):
        return self._enabled

    def client_connected(self):
        return self._client is not None

    def shutdown(self):
        """终止线程（进程退出时调用）。"""
        self._stop = True
        self._enabled = False
        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None

    # ---- 线程主体 ----
    def run(self):
        while not self._stop:
            if not self._enabled:
                time.sleep(0.2)
                continue
            try:
                self._serve()
            except Exception as e:
                self._emit("sys", f"服务异常: {e}，2秒后重试...")
                time.sleep(2)

    def _serve(self):
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            srv.bind((self.host, self.port))
        except OSError as e:
            self._emit("sys", f"监听失败: {e}（端口被占用？）")
            srv.close()
            time.sleep(2)
            return
        srv.listen(1)
        srv.settimeout(0.5)

        while self._enabled and not self._stop:
            try:
                conn, addr = srv.accept()
                self._emit("sys", f"main.py 已连接: {addr}")
                self._client = conn
                conn.settimeout(0.1)
                self._handle_client(conn)
                # 2026-10: 正常断开路径补 close（旧版只置 None, fd 以 CLOSE_WAIT 挂到
                # 下一次 accept 覆盖才被 GC —— main.py 反复重启会累积悬挂 fd）。
                try:
                    conn.close()
                except Exception:
                    pass
                self._client = None
                self._emit("sys", "main.py 已断开，重新等待连接...")
            except socket.timeout:
                continue
            except Exception as e:
                if self._enabled:
                    self._emit("sys", f"accept 错误: {e}")
                break
        try:
            srv.close()
        except Exception:
            pass
        self._client = None

    def _handle_client(self, conn):
        parser = FrameParser()
        buf = b""
        while self._enabled and not self._stop:
            try:
                data = conn.recv(4096)
                if not data:
                    break
                buf += data
                for b in buf:
                    result = parser.feed(b)
                    if result is not None:
                        ftype, payload = result
                        self._on_frame(conn, ftype, payload)
                buf = b""
            except socket.timeout:
                continue
            except (ConnectionResetError, BrokenPipeError, OSError):
                break

    @staticmethod
    def _format_payload(ftype, payload):
        """把帧载荷转成可读文本。

        COORD 结果帧若首字节 < 0x20 则是二进制子命令帧（协议约定，见
        GKS_Competition/docs/串口协议.md）：按协议解成字段；其余按 ASCII 文本解码。
        若把二进制帧直接当文本解会全是乱码（巡线帧 30Hz 刷屏，尤其需要能看懂）。
        """
        if ftype == TYPE_COORD and payload and payload[0] < 0x20:
            tag = payload[0]
            # §6b 巡线帧: [TAG=0x01][err i16 LE /10000][steer i16 LE /1000]
            #              [angle i16 LE 0.1°][valid u8], 8B
            if tag == 0x01 and len(payload) == 8:
                err, steer, ang, valid = struct.unpack("<hhhB", payload[1:])
                return (f"巡线帧 len={len(payload)} | err={err / 10000.0:+.4f} "
                        f"steer={steer / 1000.0:+.3f} angle={ang / 10.0:+.1f}° "
                        f"valid={valid}")
            # §6c 人质帧: [TAG=0x02][status u8][count u8] + count×[shape][cx u16][cy u16]
            if tag == 0x02 and len(payload) >= 3:
                status, cnt = payload[1], payload[2]
                names = {1: "圆柱", 2: "圆锥", 3: "腰鼓"}
                items = []
                for i in range(cnt):
                    p = payload[3 + 5 * i:3 + 5 * i + 5]
                    if len(p) < 5:
                        break
                    shape = p[0]
                    cx = p[1] | (p[2] << 8)
                    cy = p[3] | (p[4] << 8)
                    items.append(f"{names.get(shape, shape)}({cx},{cy})")
                body = " ".join(items) if items else "无目标"
                return (f"人质帧 len={len(payload)} | status={status} "
                        f"count={cnt} | 从左到右: {body}")
            # §6d 色球帧: [TAG=0x03][status u8][count u8] + count×[color][cx u16][cy u16]
            #             + （2026-09 扩展, 仅成功帧）[yaw i16 LE 0.01°][fit_ok u8]
            if tag == 0x03 and len(payload) >= 3:
                status, cnt = payload[1], payload[2]
                names = {1: "红", 2: "绿", 3: "蓝"}
                items = []
                for i in range(cnt):
                    p = payload[3 + 5 * i:3 + 5 * i + 5]
                    if len(p) < 5:
                        break
                    color = p[0]
                    cx = p[1] | (p[2] << 8)
                    cy = p[3] | (p[4] << 8)
                    items.append(f"{names.get(color, color)}({cx},{cy})")
                body = " ".join(items) if items else "无目标"
                ext = ""
                if len(payload) >= 21:
                    raw = payload[18] | (payload[19] << 8)
                    if raw >= 0x8000:
                        raw -= 0x10000          # i16 补码
                    ext = f" | 连线角 {raw / 100.0:+.2f}° 共线{'OK' if payload[20] else 'NG'}"
                return (f"色球帧 len={len(payload)} | status={status} "
                        f"count={cnt} | 从左到右: {body}{ext}")
            # §6e 靶标帧: [TAG=0x04][status u8][count u8]
            #             + count×[color u8][cx u16 LE][cy u16 LE][r u16 LE]
            if tag == 0x04 and len(payload) >= 3:
                status, cnt = payload[1], payload[2]
                names = {1: "红", 2: "绿", 3: "蓝"}
                items = []
                for i in range(cnt):
                    p = payload[3 + 7 * i:3 + 7 * i + 7]
                    if len(p) < 7:
                        break
                    color = p[0]
                    cx = p[1] | (p[2] << 8)
                    cy = p[3] | (p[4] << 8)
                    r = p[5] | (p[6] << 8)
                    items.append(f"{names.get(color, color)}({cx},{cy})r{r}")
                body = " ".join(items) if items else "无目标"
                return (f"靶标帧 len={len(payload)} | status={status} "
                        f"count={cnt} | 从左到右: {body}")
            # §6f 靶标流帧: [TAG=0x05][status u8][count u8]
            #              + count×[color u8][cx u16 LE][cy u16 LE][r u16 LE]
            # 布局与 §6e 逐字节相同（color=0xFF 表示"无色模式"）。
            # ⚠ 调用方 _on_frame 对流帧**不走**本函数的通用路径（见那里的说明）——
            #   这里只为"手动看单帧"留一份可读文案。
            if tag == 0x05 and len(payload) >= 3:
                status, cnt = payload[1], payload[2]
                names = {1: "红", 2: "绿", 3: "蓝", 0xFF: "无色"}
                items = []
                for i in range(cnt):
                    p = payload[3 + 7 * i:3 + 7 * i + 7]
                    if len(p) < 7:
                        break
                    items.append(f"{names.get(p[0], p[0])}"
                                 f"({p[1] | (p[2] << 8)},{p[3] | (p[4] << 8)})"
                                 f"r{p[5] | (p[6] << 8)}")
                body = " ".join(items) if items else "空窗（流仍活着）"
                return (f"流帧 len={len(payload)} | status={status} "
                        f"count={cnt} | 从左到右: {body}")
            # §6h 矩形地板帧: [TAG=0x07][status u8][cx u16 LE][cy u16 LE][flags][spread][保留], 10B
            if tag == 0x07 and len(payload) >= 10:
                if payload[1] != 1:
                    return f"矩形地板帧 len={len(payload)} | status=0（板子没识别出来/超时）"
                cx = payload[2] | (payload[3] << 8)
                cy = payload[4] | (payload[5] << 8)
                fl, sp = payload[6], payload[7]
                return (f"矩形地板帧 len={len(payload)} | 板心=({cx},{cy}) "
                        f"{'三边估' if fl & 1 else '四边精确'} 边数={(fl >> 1) & 7} "
                        f"{'量内沿' if fl & 0x10 else '量中线'} 散布={sp}px")
            # §6i 看路帧: [TAG=0x08][status u8][dx i16 LE /10][ex i16 LE][ey i16 LE]
            #            [flags u8][spread u8][保留 u16], 12B
            # ⚠️ dx/ex/ey 是 **i16**（dx 要留负号、ex 可以到 0 附近）——
            #    本协议里唯一一条有符号坐标，别照上面的 u16 帧改回去。
            if tag == 0x08 and len(payload) >= 12:
                if payload[1] != 1:
                    return f"看路帧 len={len(payload)} | status=0（没找到黄区/超时）"
                def _i16(o):
                    v = payload[o] | (payload[o + 1] << 8)
                    return v - 65536 if v >= 32768 else v
                dx = _i16(2) / 10.0
                ex, ey = _i16(4), _i16(6)
                fl, sp = payload[8], payload[9]
                return (f"看路帧 len={len(payload)} | Δx={dx:.1f}px 左边缘=({ex},{ey}) "
                        f"{'边缘有效' if fl & 1 else '边缘缺失'} "
                        f"{'贴画面左边(饱和)' if fl & 2 else ''} "
                        f"{'贴上下边' if fl & 4 else ''} "
                        f"{'面积勉强' if fl & 8 else ''} 散布={sp}px")
            return f"二进制帧 TAG=0x{tag:02X} len={len(payload)} | {payload.hex(' ')}"
        try:
            return payload.decode("ascii", "replace").rstrip("\x00")
        except Exception:
            return payload.hex()

    def _remember_hostage(self, payload):
        """存下最近一帧人质结果（结构化），供网页显示「从左到右」顺序与坐标。"""
        names = {1: "圆柱", 2: "圆锥", 3: "腰鼓"}
        status, cnt = payload[1], payload[2]
        targets = []
        for i in range(cnt):
            p = payload[3 + 5 * i:3 + 5 * i + 5]
            if len(p) < 5:
                break
            shape = p[0]
            targets.append({"shape": shape, "name": names.get(shape, str(shape)),
                            "cx": p[1] | (p[2] << 8), "cy": p[3] | (p[4] << 8)})
        with self._lock:
            self._last_hostage = {"status": status, "count": cnt, "targets": targets,
                                  "ts": time.strftime("%H:%M:%S")}

    def last_hostage(self):
        """最近一帧人质结果（无则 None）。"""
        with self._lock:
            return self._last_hostage

    def _remember_color(self, payload):
        """存下最近一帧色球结果（结构化），供网页显示「从左到右」顺序与坐标。"""
        names = {1: "红", 2: "绿", 3: "蓝"}
        status, cnt = payload[1], payload[2]
        targets = []
        for i in range(cnt):
            p = payload[3 + 5 * i:3 + 5 * i + 5]
            if len(p) < 5:
                break
            color = p[0]
            targets.append({"color": color, "name": names.get(color, str(color)),
                            "cx": p[1] | (p[2] << 8), "cy": p[3] | (p[4] << 8)})
        with self._lock:
            self._last_color = {"status": status, "count": cnt, "targets": targets,
                                "ts": time.strftime("%H:%M:%S")}

    def last_color(self):
        """最近一帧色球结果（无则 None）。"""
        with self._lock:
            return self._last_color

    def _remember_rect(self, payload):
        """存下最近一帧矩形地板结果（结构化），供网页显示板心坐标。"""
        if payload[1] != 1:
            with self._lock:
                self._last_rect = {"status": 0, "ts": time.strftime("%H:%M:%S")}
            return
        with self._lock:
            self._last_rect = {"status": 1,
                               "cx": payload[2] | (payload[3] << 8),
                               "cy": payload[4] | (payload[5] << 8),
                               "flags": payload[6], "spread": payload[7],
                               "ts": time.strftime("%H:%M:%S")}

    def last_rect(self):
        """最近一帧矩形地板结果（无则 None）。"""
        with self._lock:
            return self._last_rect

    def _remember_road(self, payload):
        """存下最近一帧看路结果（结构化），供网页显示交点/右切点/Δ。

        ⚠️ dx/ex/ey 按 **i16** 解（协议 §6i 里唯一一条有符号坐标）。
        """
        if payload[1] != 1:
            with self._lock:
                self._last_road = {"status": 0, "ts": time.strftime("%H:%M:%S")}
            return

        def _i16(o):
            v = payload[o] | (payload[o + 1] << 8)
            return v - 65536 if v >= 32768 else v

        # ⚠️ Δx 是 Pi 量好的（0.1px 整数），这里只是 /10 还原 —— 不要再自己算距离
        dx = _i16(2) / 10.0
        ex, ey = _i16(4), _i16(6)
        with self._lock:
            self._last_road = {"status": 1, "dx": round(dx, 1), "ex": ex, "ey": ey,
                               "flags": payload[8], "spread": payload[9],
                               "ts": time.strftime("%H:%M:%S")}

    def last_road(self):
        """最近一帧看路结果（无则 None）。"""
        with self._lock:
            return self._last_road

    def _remember_road_yaw(self, payload):
        """存下最近一帧看路校准结果（结构化），供网页显示方向角。

        ⚠️ ang 按 **i16** 解（0.01°），cols/rms/spread/flags 都是 u8（协议 §6j）。
        """
        if payload[1] != 1:
            with self._lock:
                self._last_road_yaw = {"status": 0, "ts": time.strftime("%H:%M:%S")}
            return

        v = payload[2] | (payload[3] << 8)
        ang = (v - 65536 if v >= 32768 else v) / 100.0
        with self._lock:
            self._last_road_yaw = {"status": 1, "ang": round(ang, 2),
                                   "cols": payload[4], "rms": payload[5] / 10.0,
                                   "spread": payload[6] / 100.0, "flags": payload[7],
                                   "ts": time.strftime("%H:%M:%S")}

    def last_road_yaw(self):
        """最近一帧看路校准结果（无则 None）。"""
        with self._lock:
            return self._last_road_yaw

    def _remember_target(self, payload):
        """存下最近一帧靶标结果（结构化），供网页显示靶心坐标与半径。"""
        names = {1: "红", 2: "绿", 3: "蓝"}
        status, cnt = payload[1], payload[2]
        targets = []
        for i in range(cnt):
            p = payload[3 + 7 * i:3 + 7 * i + 7]
            if len(p) < 7:
                break
            color = p[0]
            targets.append({"color": color, "name": names.get(color, str(color)),
                            "cx": p[1] | (p[2] << 8), "cy": p[3] | (p[4] << 8),
                            "r": p[5] | (p[6] << 8)})
        with self._lock:
            self._last_target = {"status": status, "count": cnt, "targets": targets,
                                 "ts": time.strftime("%H:%M:%S")}

    def last_target(self):
        """最近一帧靶标结果（无则 None）。"""
        with self._lock:
            return self._last_target

    def _remember_stream(self, payload):
        """靶标流帧（§6f TAG=0x05）：存最新一帧 + **按 STREAM_LOG_S 摘一条日志**。

        ⚠ 为什么流帧要单独一条路、不能复用 _remember_target + 通用 _emit：
        流是 ~20Hz 的连续流，逐帧 emit 会在 25 秒内把 maxlen=500 的日志环刷干净，
        前面那些握手/指令/结果全看不见了 —— 而"看日志"恰恰是台架调参的主要手段。
        所以这里只留 1Hz 摘录（帧率 + 靶数 + 坐标），要看逐帧细节就停流。
        """
        cnt = min(payload[2], self._stream_max)
        names = {1: "红", 2: "绿", 3: "蓝", 0xFF: "无色"}
        targets = []
        for i in range(cnt):
            p = payload[3 + 7 * i:3 + 7 * i + 7]
            if len(p) < 7:
                break
            targets.append({"color": p[0], "name": names.get(p[0], str(p[0])),
                            "cx": p[1] | (p[2] << 8), "cy": p[3] | (p[4] << 8),
                            "r": p[5] | (p[6] << 8)})

        now = time.time()
        with self._lock:
            self._last_stream = {"status": payload[1], "count": cnt, "targets": targets,
                                 "ts": time.strftime("%H:%M:%S")}
            # ⚠ 隔了 STREAM_GAP_S 才来的帧 = **新的一次起流**（上一次已停/超时），
            #   计数与计时归零重来。不归零的话"累计"会跨轮一直涨（实测第二轮从 253 开始），
            #   台架上分不清"这一轮发了多少帧"。只能按间隔猜 —— 协议里没有"起流"标记帧。
            # ⚠ 判据必须是"距**上一帧**的间隔"，不能写成"距本轮起点"：后者在流跑满
            #   STREAM_GAP_S 之后**每帧**都成立，于是每帧都重置（实测每秒打一条"累计1"）。
            if self._stream_last_ts <= 0.0 or (now - self._stream_last_ts) > STREAM_GAP_S:
                self._stream_n, self._stream_t0, self._stream_last_log = 0, now, 0.0
            self._stream_last_ts = now
            self._stream_n += 1
            due = (now - self._stream_last_log) >= STREAM_LOG_S
            n, t0 = self._stream_n, self._stream_t0
            if due:
                self._stream_last_log = now
        if due:
            # ⚠ 首帧（或起流后不到 STREAM_LOG_S）时 now-t0 几乎为 0，直接除会打出
            #   "1000000.0fps" 这种把日志搞糊涂的数（真踩过）—— 那时报"—"。
            span = now - t0
            fps = "—" if span < STREAM_LOG_S else f"{n / span:.1f}"
            body = " ".join(f"{t['name']}({t['cx']},{t['cy']})r{t['r']}"
                            for t in targets) or "空窗（流仍活着）"
            self._emit("rx", f"流帧  | {fps}fps 累计{n} 靶数={cnt} | {body}")

    def last_stream(self):
        """最近一帧靶标流帧（无则 None）。"""
        with self._lock:
            return self._last_stream

    def _on_frame(self, conn, ftype, payload):
        type_names = {TYPE_CMD: "CMD", TYPE_COORD: "COORD", TYPE_ACK: "ACK"}
        tname = type_names.get(ftype, f"0x{ftype:02X}")

        # ⚠ 靶标流帧（TAG=0x05）**不进通用日志**：~20Hz 逐帧 emit 会刷爆日志环
        #    （见 _remember_stream 的说明）。这里提前返回，改走 1Hz 摘录那条路。
        if ftype == TYPE_COORD and len(payload) >= 3 and payload[0] == 0x05:
            self._remember_stream(payload)
            return

        self._emit("rx", f"{tname:5s} | {self._format_payload(ftype, payload)}")

        # 人质结果帧：额外存结构化副本（网页显示顺序/坐标用）
        if ftype == TYPE_COORD and len(payload) >= 3 and payload[0] == 0x02:
            self._remember_hostage(payload)

        # 色球结果帧：同上
        if ftype == TYPE_COORD and len(payload) >= 3 and payload[0] == 0x03:
            self._remember_color(payload)

        # 靶标结果帧：同上
        if ftype == TYPE_COORD and len(payload) >= 3 and payload[0] == 0x04:
            self._remember_target(payload)

        # 矩形地板结果帧：同上
        if ftype == TYPE_COORD and len(payload) >= 10 and payload[0] == 0x07:
            self._remember_rect(payload)

        # 看路结果帧：同上（⚠️ 12B, 别写成 10 —— 长度判错会静默少记一帧）
        if ftype == TYPE_COORD and len(payload) >= 12 and payload[0] == 0x08:
            self._remember_road(payload)

        # 看路校准结果帧：同上（⚠️ 8B, 见协议 §6j）
        if ftype == TYPE_COORD and len(payload) >= 8 and payload[0] == 0x09:
            self._remember_road_yaw(payload)

        # 自动握手应答：收到 "SYNC_HS_1" → 回 ACK 0xCC（与真实单片机一致）
        if ftype == TYPE_COORD and payload.rstrip(b"\x00") == b"SYNC_HS_1":
            time.sleep(0.02)
            try:
                conn.sendall(pack_frame(TYPE_ACK, bytes([0xCC])))
                self._emit("tx", "ACK   | 0xCC (握手应答)")
            except Exception:
                pass

    # ---- 发指令给已连接的 main.py ----
    def send_command(self, key):
        if not self._client:
            self._emit("sys", "错误: 无客户端连接")
            return False
        try:
            self._client.sendall(pack_frame(TYPE_CMD, key.encode("ascii")))
            self._emit("tx", f"CMD   | '{key}'")
            return True
        except Exception as e:
            self._emit("sys", f"发送失败: {e}")
            return False
