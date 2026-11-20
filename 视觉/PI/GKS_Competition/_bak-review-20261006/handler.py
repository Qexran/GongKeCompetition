# handler.py — 单片机指令分发（GKS 工科赛）
#
# 精简版移植自 cv_python/handler.py。主循环收帧 → 查注册表 → 调用任务函数。
# 后续接入物块/色环/码垛/扫雷/转盘等任务时，只需在 _COMMANDS 里追加一条
# `b"<命令码>": 任务函数`，无需改动主循环。
import subprocess
import threading
import time

from serial_comm import serial_comm, SerialDisconnected
import config
from protocol import TYPE_ACK, TYPE_CMD, TYPE_HELLO, TYPE_COORD
from config import HANDSHAKE_DATA, HANDSHAKE_ACK_BYTE, RPI_HEARTBEAT_INTERVAL
from config import PHOTO_ACK_TIMEOUT_S
from tasks import run_qr_wechat, run_qr_wechat_main, run_line_follow
from tasks import run_qr_diff, run_qr_diff_main
from tasks import run_hostage_detect, build_result_frame
from tasks import run_color_detect, build_color_frame
from tasks import run_target_detect, build_target_frame
from tasks import run_target_stream
from tasks import run_rect_detect, build_rect_frame
# 看路（命令 "G"）：主摄量地面黄色区的左边缘 x / 下边缘 y，结果回 0x08 帧（见 docs/串口协议.md §6i）
from tasks import run_road_detect, build_road_frame
from openmv_serial import openmv_reader
# 相机租约（预热复用）—— 命令 "B"/"C" 用；见 camera_lease.py 顶部的长注释。
# 模块级导入是安全的: camera_lease 只依赖 camera, 不反向依赖 handler（无循环）。
import camera_lease


# ---- 串口侧任务的「取消」事件（命令 "x", 2026-10 加）----
# 单片机中止脚本/重新握手时发 "x" 置位; handle() 在分发**每条新指令前**清零,
# 各阻塞任务经 abort_event=_task_abort 感知（与 Web 端 _scan_abort 同一套约定）。
# 治的是「迟到结果帧错位」: 旧任务在 Pi 上跑完回帧, 落进单片机**下一轮**同类型
# 步骤的等待窗口被当成新结果（app_rpi.c 头注释自己承认的结构性风险）—— 现在单片机
# 一中止就先取消 Pi 侧在跑的任务, 迟到帧从根上不再产生。
_task_abort = threading.Event()


def _on_rx_frame(ftype, payload):
    """串口接收线程里的逐帧回调（serial_comm.set_frame_hook 注册, 2026-10 加）。

    主循环可能正卡在某个阻塞任务里（"1"/"4"/"5"/"6"/"A"/"G", 最长 ~6s+）—— 那时
    队列里的 "x" 要等任务返回才被处理, 旧任务的迟到结果帧已经发出、错位已经发生。
    在这里就地置位取消事件, 在跑的任务会在自己的下一个 abort 检查点退出。
    ⚠️ 快进快出、不吞异常的调用方自己吞（_dispatch_frame 已兜）; "x" 照样入队,
    主循环随后处理它时还会停巡线流/靶标流（_cmd_cancel）, 两件事互补。
    """
    if ftype == TYPE_CMD and payload == b"x":
        _task_abort.set()
        return
    if ftype == TYPE_CMD and payload == b"r":
        _r_pair_gate()


# ---- "r" 双帧的第二道门（接收线程版, 2026-10 加）----
# 单片机「等链路超时自动重启 Pi 客户端」发的是成对的 "r"（vRpiRestartPiClient,
# 两帧间隔 500ms, 靠 Pi 侧 _require_double_send 的双发确认门）。那道门在 handle()
# 里 —— 主循环**卡死**（比如某个相机调用陷在内核里, abort 检查点永远到不了）时
# 谁也处理不了它, 车就再也救不回来。这里在接收线程里数同一对帧: 6s 内两帧也在
# 后台线程走一次 restart_program（20s 缓冲后原地重启, 换进程镜像即救活一切）。
# ⚠️ 门比 handle() 那道更紧（6s < 10s）→ 这里能触发的, 主循环那道也会认 ——
#    两路只会合成**一次**重启（都在 sleep(20), 先到的 os.execl 生效, 进程被换掉）。
# ⚠️ 只数 TYPE_CMD 的裸 "r": 网页/菜单单发的零星 "r" 凑不齐一对, 不会误伤。
_r_seen_at = []
_R_PAIR_WINDOW_S = 6.0


def _r_pair_gate():
    import time as _time
    now = _time.time()
    _r_seen_at.append(now)
    # 只保留窗口内的
    while _r_seen_at and now - _r_seen_at[0] > _R_PAIR_WINDOW_S:
        _r_seen_at.pop(0)
    if len(_r_seen_at) < 2:
        return
    _r_seen_at.clear()
    print("[WARN] 收到成对 \"r\"（接收线程侧识别）—— 20s 后原地重启 main.py"
          "（主循环若卡死也救得回来; 若没卡死, 这与主循环那道门合成同一次重启）")
    threading.Thread(target=_safe_restart_program, name="r-restart", daemon=True).start()


def _safe_restart_program():
    try:
        serial_comm.restart_program()
    except Exception as e:
        print(f"[WARN] 接收线程侧重启执行失败: {e}")


serial_comm.set_frame_hook(_on_rx_frame)


def _split_payload(data):
    """`<命令码>[,<数字>]` → (head, num)。

    **不带逗号时 head == data、num is None** —— 加这个扩展之前的所有指令一个字节都不用改。

    ⚠️ 那个数字的**含义按命令码而定**（线上格式是同一个、白名单也共用一张，所以
       下面那个局部名历来叫 ms，别被它带偏）：
         阻塞类视觉指令 "1"/"D"/"E"/"F"/"G" → **毫秒** = 「这条我最多等这么久」，
                                             见 _budget_from_ms；
         预热类指令 "B"/"C"                → **使用次数** = 「这台相机接下来要用几次」，
                                             放宽相机租约的空闲窗口，见
                                             camera_lease._clamp_uses / idle_limit_s。
         补光亮度指令 "L"                  → **亮度值** = OpenMV 补光 0~100（灯灭=0 合法），
                                             见 _cmd_openmv_led。
       加新命令族时先想清楚自己属于哪一类。

    拆不成合法形状时返回 `(整串, None)`：调用方拿它查表必然查不到 → 响亮地报未知指令。
    刻意**不**静默忽略那个数字 —— 少了它本端会按自己的默认值跑，可能跑得比对方愿意等的还久。
    """
    head, sep, tail = data.partition(b",")
    if not sep:
        return data, None
    if not tail.isdigit():
        print(f"[WARN] 指令后缀不是数字: {data!r}")
        return data, None
    if head not in _MS_COMMANDS:
        print(f"[WARN] 该指令不接受数值后缀: {data!r}"
              f"（能收的只有 {'/'.join(c.decode('ascii') for c in _MS_COMMANDS)}）")
        return data, None
    return head, int(tail)


def _budget_from_ms(ms):
    """把单片机给的超时（毫秒）换成**本端该用的预算**（秒）；None/非正 → None（用本端默认）。

    ⚠️ 必须减一个余量：单片机那边 `ucRpiWaitQrResult(ms)` 是从**它发指令的那一刻**开始
       计的，而这 ms 里本端还要做「开相机 + 扫描 + 回帧」。余量要盖住开相机（租约没命中
       时实测 ~1.3s）—— 否则冷启动那一次会**答得比单片机放弃还晚**，而迟到的结果帧会
       落进下一次重扫的等待窗口里被当成那一次的结果（见 tasks/_qr.py 的同名注释）。
       预热 "B"/"C" 能省掉那 1.3s，但余量照留：宁可少试一轮也不能迟到。
    """
    from config import QR_CMD_MARGIN_S, QR_CMD_MAX_TIMEOUT_S
    if not ms or ms <= 0:
        return None
    budget = ms / 1000.0 - QR_CMD_MARGIN_S
    if budget > QR_CMD_MAX_TIMEOUT_S:
        # 夹住：相机租约的空闲看门狗只被 read() 续命，单次扫描超过它会在**解码途中**
        # 把设备关掉，后续 read() 全失败、扫描循环空转到超时（现象是「一直超时」而不是报错）
        import config
        print(f"[WARN] 单片机给的超时 {ms}ms 超出本端上限，预算夹到 "
              f"{QR_CMD_MAX_TIMEOUT_S:.0f}s（租约空闲看门狗 {config.LEASE_IDLE_TIMEOUT_S:.0f}s）")
        budget = QR_CMD_MAX_TIMEOUT_S
    return max(1.0, budget)


def _save_frame(key, frame, legacy_path):
    """任务结果帧落盘（2026-10 起统一走 logstore）。

    一帧写三个地方:
      · logs/run_*/frames/<key>_NNNN.jpg —— **序号递增、永不覆盖**（比对第 N 次
        跑同一任务时相机各看到什么, 配合日志回看用）;
      · logs/run_*/last_<key>.jpg —— 本次运行内最新（run 目录内的便利副本）;
      · legacy_path（logs/<key>_last.jpg 固定路径）—— GKS_Web 的 /api/*/frame
        轮询端点读它, 保持兼容。
    任何失败只打 WARN —— 落帧是旁路, 不许影响任务本身。
    """
    if frame is None:
        return
    try:
        import logstore
        logstore.archive_frame(key, frame, legacy_path=legacy_path)
    except Exception as e:
        print(f"[WARN] 保存{key}最后一帧失败: {e}")


def _cmd_qr_common(run_fn, ms=None):
    """四条扫码命令（"1"/"D" 微信，"E"/"F" 自研）的**唯一实现**。

    ⚠️ 回传的是**同一种帧**（TYPE_COORD，9B ASCII 补零，见 docs/串口协议.md §6a）：
    成功 → "QR"+三位数字（如 "QR211"）；失败/超时 → "QRF"。
    四条的差别**只在 run_fn 用哪台相机、哪个算法** —— 所以单片机侧
    ucRpiWaitQrResult 与下游（IF 条件/排爆/反恐）一个字都不用改。
    别在这里加任何"按相机/算法区分"的分支或新帧型。

    ms: 单片机随指令发来的**它自己愿意等多久**（见 docs/串口协议.md §6 的
        "<命令码>,<毫秒>" 扩展）。给了就按它倒推本端预算，让本端**先**答；
        没给（老固件 / 不是从串口来的）就退回 config 里那两个各自的默认值。
    """
    result = run_fn(timeout=_budget_from_ms(ms), abort_event=_task_abort)
    if result.get("success"):
        print(f"[INFO] 二维码识别成功: {result['code']} ({result['method']})")
        _send_qr_result("QR" + result["code"])
    elif result.get("aborted"):
        print("[INFO] 二维码任务被打断")
    else:
        print(f"[WARN] 二维码识别失败: {result['reason']}")
        _send_qr_result("QRF")
    # 保存最后一帧（成功为标注帧，失败为最后一帧原图），供 Web 控制端预览显示
    _save_frame("qr", result.get("frame"), _cfg("QR_LAST_FRAME_PATH"))


def _cfg(name):
    """按名取 config 常量（避免模块顶 import 一串 *_LAST_FRAME_PATH）。"""
    import config
    return getattr(config, name, None)


def _cmd_qr(ms=None):
    """命令 '1'：二维码识别（微信引擎，比 OpenCV QRCodeDetector 更鲁棒）。**扫码副摄**。"""
    _cmd_qr_common(run_qr_wechat, ms)


def _cmd_qr_main(ms=None):
    """命令 'D'：二维码识别，但用**主摄**（/dev/camera_main）。

    与 '1' 同一条解码链、同一种结果帧，只换相机与配套预处理参数
    （config 的 QR_MAIN_SCAN_*，默认不中心裁剪）。用于二维码只出现在主摄
    视野内的场合；也能吃 "C"（预热主摄）省下的开相机时间。
    """
    _cmd_qr_common(run_qr_wechat_main, ms)


def _cmd_qr_diff(ms=None):
    """命令 'E'：二维码识别用**自研难度算法**（候选集模式）。**扫码副摄**。

    与 '1' 同一条链、同一种结果帧，只有解码器不同。治 wechat 治不了的图
    （斜视/小码/强透视），代价是慢得多（台架实测 4.5s）。
    """
    _cmd_qr_common(run_qr_diff, ms)


def _cmd_qr_diff_main(ms=None):
    """命令 'F'：自研难度算法 + **主摄**（/dev/camera_main）。

    与 'E' 同一个实现，只换相机。斜视扫码那条路走它（先 "C" 预热主摄，
    摆臂那几秒就是预热窗口）。
    """
    _cmd_qr_common(run_qr_diff_main, ms)


def _send_qr_result(text: str):
    """把二维码结果回传单片机（9B ASCII COORD 帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端 ucRpiWaitQrResult 会超时重试。
    """
    try:
        serial_comm.send(text)
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 二维码结果回传失败: {e}")


# ---- 人质形状识别（命令 '4'）----

def _cmd_hostage(ms=None):
    """命令 '4'：人质形状识别（主摄像头 YOLO，圆柱/圆锥/腰鼓）。

    ms: 单片机随指令发来的「它自己愿意等多久」（"<命令码>,<毫秒>"，2026-10 起固件
    会带）。给了就按它倒推本端预算（本端先答）—— 冷开相机 + 6s 检测最坏可逼近
    单片机 8s 断链阈值, 让对端决定超时就不怕这个; 没给退回 config.HOSTAGE_TIMEOUT。

    结果回传单片机（TYPE_COORD TAG=0x02 二进制帧，见 docs/串口协议.md §6c）：
      成功 → [0x02][01][count][shape/cx/cy]×count（目标按图像从左到右排序）；
      失败/超时 → [0x02][00][00]（count=0，让单片机立即解锁，不空烧等待超时）；
      被打断 → 不回传（单片机/web 主动放弃，调用方自己知道原因）。
    """
    _stop_main_stream()   # 释放 Web 端可能占用的主摄像头推流
    result = run_hostage_detect(timeout=_budget_from_ms(ms), abort_event=_task_abort)
    if result.get("success"):
        targets = result["targets"]
        print("[INFO] 人质识别成功（从左到右）: "
              + " ".join(f"{t['shape_name']}({t['cx']},{t['cy']})" for t in targets))
        _send_hostage_result(build_result_frame(True, targets))
    elif result.get("aborted"):
        print("[INFO] 人质识别任务被打断")
    else:
        print(f"[WARN] 人质识别失败: {result['reason']}")
        _send_hostage_result(build_result_frame(False, []))
    # 保存最后一帧（成功为标注帧，失败为最佳成绩帧），供 Web 控制端预览显示
    _save_frame("hostage", result.get("frame"), _cfg("HOSTAGE_LAST_FRAME_PATH"))


def _send_hostage_result(data: bytes):
    """把人质识别结果帧回传单片机（TYPE_COORD TAG=0x02 二进制帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端 ucRpiWaitHostageResult 会超时。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, data)
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 人质结果回传失败: {e}")


# ---- 色球顺序识别（命令 '5'）----

def _cmd_color():
    """命令 '5'：色球顺序识别（主摄像头 HSV，红/绿/蓝）。

    结果回传单片机（TYPE_COORD TAG=0x03 二进制帧，见 docs/串口协议.md §6d）：
      成功 → [0x03][01][count][color/cx/cy]×count + [yaw i16 0.01°][fit_ok u8]
              （LEN=21, 小球对位扩展 2026-09: yaw=三球连线**图像**倾角,
                左高右低为正; fit_ok=0 表示共线残差超限、有球识别可疑）；
      失败/超时 → [0x03][00][00]（count=0，让单片机立即解锁，不空烧等待超时）；
      被打断 → 不回传（单片机/web 主动放弃，调用方自己知道原因）。

    ⚠️ 要求小车静止（多帧投票需同一机位）。
    """
    _stop_main_stream()   # 释放 Web 端可能占用的主摄像头推流
    result = run_color_detect(abort_event=_task_abort)
    if result.get("success"):
        targets = result["targets"]
        print("[INFO] 色球识别成功（从左到右）: "
              + " ".join(f"{t['color_name']}({int(t['cx'])},{int(t['cy'])})"
                         for t in targets))
        _send_color_result(build_color_frame(True, targets,
                                             result.get("yaw"),
                                             bool(result.get("fit_ok"))))
    elif result.get("aborted"):
        print("[INFO] 色球识别任务被打断")
    else:
        print(f"[WARN] 色球识别失败: {result['reason']}")
        _send_color_result(build_color_frame(False, []))
    # 保存最后一帧（成功为标注帧，失败为最佳成绩帧），供 Web 控制端预览显示
    _save_frame("color", result.get("frame"), _cfg("COLOR_LAST_FRAME_PATH"))


def _send_color_result(data: bytes):
    """把色球结果帧回传单片机（TYPE_COORD TAG=0x03 二进制帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端 ucRpiWaitColorResult 会超时。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, data)
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 色球结果回传失败: {e}")


# ---- 色球区矩形地板识别（命令 'A'）----

def _cmd_rect():
    """命令 'A'：色球区**矩形地板**（黑胶带边框）识别，回板心像素坐标。

    结果回传单片机（TYPE_COORD TAG=0x07 二进制帧，见 docs/串口协议.md §6h）：
      成功 → [0x07][01][cx u16 LE][cy u16 LE][flags][spread][保留]（10B）
      失败/超时 → [0x07][00] + 全 0（**让单片机立刻解锁**，不空烧等待超时）
      被打断 → 不回传（单片机/web 主动放弃，调用方自己知道原因）

    单片机侧实测一次约 2~3s（9 帧 × ~180ms + 开摄像头）。⚠️ 要求小车静止。
    ⚠️ 用主摄 → 必须与巡线/靶标流互斥（见 handle() 里那个元组）。
    """
    _stop_main_stream()   # 释放 Web 端可能占用的主摄像头推流
    result = run_rect_detect(abort_event=_task_abort)
    if result.get("success"):
        cx, cy = result["center"]
        print(f"[INFO] 矩形地板识别成功: 中心=({cx:.0f},{cy:.0f}) "
              f"{'三边估' if result.get('estimated') else '四边精确'} "
              f"path={result.get('path')} 帧{result.get('votes')}/剔{result.get('dropped')} "
              f"散布{result.get('spread', 0):.1f}px")
        _send_rect_result(build_rect_frame(True, cx, cy,
                                           result.get("estimated"),
                                           result.get("n_sides"),
                                           result.get("path") == "enclosed",
                                           result.get("spread", 0.0)))
    elif result.get("aborted"):
        print("[INFO] 矩形地板识别任务被打断")
    else:
        print(f"[WARN] 矩形地板识别失败: {result['reason']}")
        _send_rect_result(build_rect_frame(False))
    # 保存最后一帧（成功为标注帧），供排障（Web 控制端另有 /rect 就地预览）
    _save_frame("rect", result.get("frame"), _cfg("RECT_LAST_FRAME_PATH"))


def _send_rect_result(data: bytes):
    """把矩形结果帧回传单片机（TYPE_COORD TAG=0x07 二进制帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端 ucRpiWaitRectResult 会超时。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, data)
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 矩形结果回传失败: {e}")


# ---- 看路（命令 'G'）----

def _cmd_road(ms=None):
    """命令 'G'：主摄「看路」—— 量地面黄色区（左边缘 x → 车体纵向；下边缘 y → 车体横向）。

    回传单片机（TYPE_COORD TAG=0x08 二进制帧，见 docs/串口协议.md §6i）：
      成功 → [0x08][01][dx i16 /10][ex i16][ey i16][flags][spread][dy u16 /10]（12B）
      失败/超时 → [0x08][00] + 全 0（**让单片机立刻解锁**，不空烧等待超时）
      被打断 → 不回传（单片机/web 主动放弃，调用方自己知道原因）

    ms: 单片机随指令发来的**它自己愿意等多久**（"<命令码>,<毫秒>"），
        给了就按它倒推本端预算（本端先答），没给就退回 config.ROAD_TIMEOUT。
        ⚠️ 这一步是**阻塞**的，所以 "G" 必须同时在 _MS_COMMANDS 与互斥元组里。

    单片机侧实测一次约 2~3s（9 帧 × ~200ms + 开摄像头）。⚠️ 要求小车静止。
    ⚠️ 用主摄 → 必须与巡线/靶标流互斥（见 handle() 里那个元组）。
    """
    _stop_main_stream()   # 释放 Web 端可能占用的主摄像头推流
    result = run_road_detect(timeout=_budget_from_ms(ms), abort_event=_task_abort)
    if result.get("success"):
        # ⚠️ 字段是 **dx / dy / edge / area**。改过 run_road_detect 的返回结构就要同步
        #    这里 —— 漏一次的表现是「Pi 日志报成功, 单片机却一直 ROAD fail: wait st=0」
        #    （回来一看是 KeyError, 结果帧根本没发出去, 真踩过）。
        e = result.get("edge") or (0.0, 0.0)
        print(f"[INFO] 看路成功: Δx={result['dx']:.1f}px 左边缘=({e[0]:.0f},{e[1]:.0f}) "
              f"Δy={result.get('dy', 0.0):.1f}px 面积={result.get('area', 0.0):.0f} "
              f"帧{result.get('votes')}/剔{result.get('dropped')} "
              f"散布{result.get('spread', 0):.1f}px "
              f"fl=0x{result.get('flags', 0):02X}")
        # ⚠️ dy 必须跟着 flags 一起发: 单片机靠 flags 的 bit4 判「这个量到底有没有」
        #    （帧尾那个字段原来是恒 0 的保留位, 光看值分不出"没量到"和"下边缘在 y=0"）。
        _send_road_result(build_road_frame(True, result["dx"], e[0], e[1],
                                           result.get("flags", 0),
                                           result.get("spread", 0.0),
                                           result.get("dy", 0.0)))
    elif result.get("aborted"):
        print("[INFO] 看路任务被打断")
    else:
        print(f"[WARN] 看路失败: {result['reason']}")
        _send_road_result(build_road_frame(False))
    # 保存最后一帧（成功为标注帧），供排障（Web 控制端另有 /road 就地预览）
    _save_frame("road", result.get("frame"), _cfg("ROAD_LAST_FRAME_PATH"))


def _send_road_result(data: bytes):
    """把看路结果帧回传单片机（TYPE_COORD TAG=0x08 二进制帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端 ucRpiWaitRoadResult 会超时。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, data)
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 看路结果回传失败: {e}")


def _stop_main_stream():
    """停掉 Web 端主摄像头推流（若在推），释放主摄像头给人质识别任务。

    延迟导入避免与 camera_web 循环依赖（camera_web 亦在函数内 import handler）。
    """
    try:
        from camera_web import _stop_stream
        _stop_stream("main")
    except Exception:
        pass   # Web 模块未就绪/未启用不影响串口链路


# ---- 相机预热（命令 'B' 扫码相机 / 'C' 主摄，2026-09 加）----

def _cmd_warm_qr(uses=None):
    """命令 'B'：预热扫码相机（**只收不回**，没有结果帧）。

    单片机在**车还在移动/巡线的那几秒里**先发这一条，Pi 就把 /dev/camera_Qr
    开好并起读线程持续取最新帧；随后那条 "1"（二维码识别）直接解码租约里的
    最新帧，省掉实测 ~1.3s 的开相机时间（15 帧预热 + V4L2 协商 + 曝光）。

    ⚠️ **本函数必须立即返回**：handle() 是在**串口接收线程**里内联调用的，而
    阻塞式指令期间 Pi 是不发心跳的 —— 在这里做那 1.3s 的打开会把心跳一起压住
    （单片机 8s 判 DISCONNECTED）。所以 warm 只是起个后台线程。

    "B,2" 这种带数字的形状 = 「这台相机接下来要被 **2 个任务**消费」→ 相机租约的
    空闲窗口放宽成 2 倍（camera_lease.idle_limit_s），于是**隔了十几秒的第二个任务**
    也能命中同一份租约、不再白花 ~2s 重开设备。不带数字 / 0 / 1 = 与加这个参数之前
    **一字不差**（窗口仍是 1 倍）。
    """
    camera_lease.LEASE.warm(is_qr=True, uses=uses)


def _cmd_warm_main(uses=None):
    """命令 'C'：预热主摄（**只收不回**，没有结果帧）。

    同上，只是换成 /dev/camera_main，给随后的人质/色球/靶标识别省掉开相机时间。
    ⚠️ 主摄有**多个相机档**（人质 1/350、色球/靶标 3/166、看路固定白平衡）：这里按默认的
    main 档预热，人质/看路那一步 acquire 时会就地换成自己的档并丢掉收敛期几帧
    （camera_lease._reapply_profile）—— 不会再付一次开相机的钱。
    ⚠️ 与 "4"/"5"/"6"/"A" 不同，本命令**不进** handle() 里那个「先停巡线/靶标流」
    的元组：它只开相机、不发结果帧，不会与 COORD 通道撞车；而且它恰恰排在
    巡线那一步**之前**，停了巡线线程反而是错的。
    ⚠️ 真正占着主摄的网页推流跑在**另一个进程**（gks-web.service），跨进程没有
    释放协调原语（handler._stop_main_stream 在服务进程里是空操作）。
    2026-09-26 起由 **camera_lease._open_with_abort_retry** 兜住：打开失败就 HTTP 打
    GKS_Web 的 `/abort?cam=main` 请它停推流再重试一次（见 camera_lease 文件头第 4 条）。
    所以「有人开着预览页」不再等于「这里必然失败」，最多是慢 ~0.3s。

    "C,2" 这种带数字的形状 = 使用次数（见 _cmd_warm_qr 的说明与 camera_lease.idle_limit_s）。
    ⚠️ 次数放宽的正是**这个租约被占住的时间**，而主摄同时还是网页预览要抢的那台 ——
       次数别随手填大（上限见 config.LEASE_USES_MAX）。
    """
    camera_lease.LEASE.warm(is_qr=False, uses=uses)


# ---- 取消当前任务（命令 'x'，2026-10 加）----

def _cmd_cancel():
    """命令 'x'：取消 Pi 侧正在跑的任务 / 流（只收不回，无结果帧）。

    单片机在**中止脚本**（OLED 重投任务 / 菜单请求 / 蓝牙让位）与**重新握手**
    （ucRpiHandshake 判「断开重连」那条路）时发它。没有它的话:
      · 被中止的旧任务会跑完并把**迟到结果帧**发给单片机, 落进下一轮同类型步骤的
        等待窗口被当成新结果（「旧数据」错位的日常来源, 全程无报错）;
      · 旧巡线流/靶标流继续占着主摄 + 以 20~30Hz 发 COORD 帧, 与新任务的结果帧交错。
    ⚠️ 幂等：没有任务在跑时也无害（set 一个没人看的事件而已）。
    ⚠️ 与 "end"/"r" 的分工: 那两个是「停整个主程序」, 本命令只取消当前工作、
       链路与租约都不动。
    """
    _task_abort.set()
    if is_line_running():
        _cmd_line_stop()
    if is_aim_running():
        _cmd_aim_stop()
    print("[INFO] 已取消当前任务（x）")


# ---- 靶标识别 + 靶心定位（命令 '6'）----

def _cmd_target(ms=None):
    """命令 '6'：三色靶盘识别 + 靶心定位（主摄像头）。

    ms: 单片机随指令发来的「它自己愿意等多久」（同 _cmd_hostage, 2026-10 起固件会带）;
    没给退回 config.TARGET_TIMEOUT。

    结果回传单片机（TYPE_COORD TAG=0x04 二进制帧，见 docs/串口协议.md §6e）：
      成功 → [0x04][01][count][color/cx/cy/r]×count（按图像从左到右排序）；
      失败/超时 → [0x04][00][00]（count=0，让单片机立即解锁，不空烧等待超时）；
      被打断 → 不回传（单片机/web 主动放弃，调用方自己知道原因）。

    ⚠️ 回传的是**三颗**，哪一颗是目标由**单片机**按二维码第二位挑 —— Pi 侧不掺和
    任务语义（与色球那边完全一致）。
    ⚠️ 要求小车/机械臂静止（多帧投票需同一机位）。
    """
    _stop_main_stream()   # 释放 Web 端可能占用的主摄像头推流
    result = run_target_detect(timeout=_budget_from_ms(ms), abort_event=_task_abort)
    if result.get("success"):
        targets = result["targets"]
        print("[INFO] 靶标识别成功（从左到右）: "
              + " ".join(f"{t['color_name']}({int(t['cx'])},{int(t['cy'])})"
                         f"r{int(t['r'])}" for t in targets))
        _send_target_result(build_target_frame(True, targets))
    elif result.get("aborted"):
        print("[INFO] 靶标识别任务被打断")
    else:
        print(f"[WARN] 靶标识别失败: {result['reason']}")
        _send_target_result(build_target_frame(False, []))
    # 保存最后一帧（成功为标注帧，失败为最佳成绩帧），供 Web 控制端预览显示
    _save_frame("target", result.get("frame"), _cfg("TARGET_LAST_FRAME_PATH"))


def _send_target_result(data: bytes):
    """把靶标结果帧回传单片机（TYPE_COORD TAG=0x04 二进制帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端等结果的函数会超时。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, data)
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 靶标结果回传失败: {e}")


# ---- 靶标流式跟踪（命令 '7' 起流 / '8' 停流）----
#
# 与上面的 _cmd_target（"6"，同步阻塞式识别）是**两套并存**的方案：
#   "6"   一次性全画面多帧投票 → 0x04 结果帧 → 单片机开环迭代（STEP_TARGET_CALIB_WAIT）
#   "7/8" 小窗高频跟踪         → 0x05 流帧   → 单片机闭环微调（STEP_TARGET_SERVO_WAIT）
#
# 为什么做成独立线程而不是像 "6" 那样同步阻塞：流是**长任务**（要跑到单片机说停为止），
# 占着主循环的话心跳/其它指令全进不来。线程化之后心跳照发，链路比 "6" 更健康
# （"6" 期间 Pi 是不发心跳的，靠"默认超时 6s < 单片机断链阈值 8s"这条余量撑着）。
_aim_thread = None
_aim_abort = threading.Event()
_aim_lock = threading.Lock()    # 与 _line_lock 同款：保证同时只有一个流线程
_AIM_JOIN_S = 3.0               # 起流时等上一个线程释放相机的上限（见 _cmd_aim_start）


def _aim_worker():
    """靶标流线程体：跑 run_target_stream，结束后清理线程标记。"""
    global _aim_thread
    try:
        run_target_stream(abort_event=_aim_abort)
    except Exception as e:
        import traceback
        print(f"[ERROR] 靶标流线程异常: {type(e).__name__}: {e}")
        traceback.print_exc()
    finally:
        _aim_abort.clear()
        _aim_thread = None


def _cmd_aim_start():
    """命令 '7'：开始靶标流式跟踪（独立线程，回传 0x05 流帧）。

    ⚠ 起流前必须先把**其它会占主摄的东西**让开，两件事：
      ① `_cmd_line_stop()` —— 巡线与靶标流都走 COORD 通道，两条流同时在跑会帧交错
         （串口发送无跨线程锁，见本文件顶部与 _line.py 的说明）；
      ② `_stop_main_stream()` —— Web 端主摄推流占着设备，不释放的话 open_camera 会失败。
    """
    global _aim_thread
    _cmd_line_stop()                # 两条流互斥：主摄 + COORD 通道

    if not _aim_lock.acquire(blocking=False):
        print("[WARN] 靶标流已在运行，忽略重复的启动指令")
        return
    try:
        # 上一个线程还活着时，**两种情形必须分开处理**（混在一起会各错一半）：
        #   ① 它正在**正常跑流**（abort 未置位）→ 忽略本次启动，打 WARN。
        #      与 _cmd_line_start 对重复 '2' 的做法一致。这里**不能**为了"起个新的"
        #      去 abort 它：那会把一条健康的流杀掉再重新锁定（~1s 什么都发不出来），
        #      而 MCU 那边看到的正好是"帧计数卡住" —— 一次重复的 '7' 就退化成一拍卡顿。
        #      反而"上一条 '8' 在传输中丢了"这种真实场景，靠这里忽略才救得回来
        #      （流本来就是无状态的：它不认识 MCU 的哪一步在消费它）。
        #   ② 它正在**收尾**（abort 已置位，还卡在一帧相机读取里）→ 等它释放相机。
        #      不等的话 `_open_with_retry` 会一直拿到"设备仍被占用"而失败 ——
        #      表现是闭环那边帧计数一帧都不涨、冻到超时长鸣，而日志里只有一条
        #      "主摄像头打开失败"，很容易被误判成硬件问题。
        old = _aim_thread
        if old is not None and old.is_alive():
            if not _aim_abort.is_set():
                print("[WARN] 靶标流已在运行，忽略重复的启动指令")
                return
            print(f"[INFO] 靶标流：上一个线程还在收尾，等它释放相机（≤{_AIM_JOIN_S}s）...")
            old.join(timeout=_AIM_JOIN_S)
            if old.is_alive():
                print(f"[WARN] 靶标流：上一个线程 {_AIM_JOIN_S}s 内没退出，忽略本次启动")
                return

        _stop_main_stream()         # 释放 Web 端可能占用的主摄像头推流
        _aim_abort.clear()
        _aim_thread = threading.Thread(target=_aim_worker, name="target-stream", daemon=True)
        _aim_thread.start()
        print("[INFO] 靶标流跟踪已启动")
    finally:
        _aim_lock.release()


def _cmd_aim_stop():
    """命令 '8'：停止靶标流式跟踪。未在跑时打 WARN 忽略（幂等）。"""
    if _aim_thread is None or not _aim_thread.is_alive():
        print("[WARN] 靶标流未在运行，忽略停止指令")
        return
    _aim_abort.set()
    print("[INFO] 已发送靶标流停止信号")


def _send_photo_ack(ok):
    """把拍照结果回传单片机（COORD TAG=0x06, 载荷 [ok u8]）。

    单片机侧见 app_rpi.h 的 RPI_PHOTO_TAG / RPI_PHOTO_LEN；它只拿这个闪一句
    「拍照 成功!/失败!」（1s），不做别的。
    ⚠️ 断线时只打日志: 结果帧丢不致命, 但本函数**必须尽量发出去**, 而且不能把调用方
       （openmv_serial 的读线程）带崩 —— 一个异常就断掉巡线数据。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, bytes([_PHOTO_ACK_TAG, 1 if ok else 0]))
        print("[INFO] 已回传拍照结果: %s" % ("成功" if ok else "失败"))
    except (SerialDisconnected, OSError) as e:
        print(f"[WARN] 拍照结果回传失败: {e}")


def _photo_timeout():
    """超时兜底：到点还没等到 OpenMV 回执 → 回一条"失败"。

    ⚠️ 持 _photo_lock 判 _photo_timer: 回执刚到、正要 cancel 我们的瞬间我们已触发 ——
       cancel 对已触发的 Timer 无效, 不判的话两边各发一条 ACK, 单片机闪显哪条全凭运气。
    """
    global _photo_timer
    with _photo_lock:
        if _photo_timer is None:
            return   # 回执已经处理过了（或已被新的拍照请求顶替）—— 什么都不做
        _photo_timer = None
        print("[WARN] 拍照回执超时(%.1fs)：相机没上电 / 串口没插好 / "
              "跑的是没实现 'P' 的旧固件？" % PHOTO_ACK_TIMEOUT_S)
        _send_photo_ack(False)


def _on_photo_result(ok):
    """OpenMV 回执到达（在 openmv_serial 的读线程里）→ 撤销超时定时器 + 回传单片机。

    迟到的回执（超时帧已发过）只打日志不再补发 —— 单片机那次已按"失败"收场。
    """
    global _photo_timer
    with _photo_lock:
        if _photo_timer is None:
            print(f"[WARN] 拍照回执迟到(结果={'成功' if ok else '失败'}), 超时帧已发过, 丢弃")
            return
        _photo_timer.cancel()
        _photo_timer = None
        _send_photo_ack(ok)


# ---- 拍照回应链路的状态（命令 '9'）----
_PHOTO_ACK_TAG = 0x06   # COORD 子命令 TAG（须与 app_rpi.h::RPI_PHOTO_TAG 一致）
_photo_timer = None     # 回执超时定时器（发出 'P' 时起, 收到回执或超时后清）
_photo_lock = threading.Lock()   # 2026-10: 回执(openmv读线程)与超时(Timer线程)互斥 ——
                                 # 旧版两边可同时发, 单片机会先收"失败"再收"成功"两条 0x06

# 注册回执回调（模块级一次; 与 openmv_reader 本身同为模块级单例的用法）
openmv_reader.set_photo_callback(_on_photo_result)


def _cmd_photo():
    """命令 '9'：让 **OpenMV** 拍一张照（存到它自己盘上的 photos/），并等它的回执。

    链路（**四跳**，含回程）: 单片机 --"9"--> 本进程 --'P'--> OpenMV 存盘
      --回写 "P,<ok>"--> openmv_serial 解析 --回调 _on_photo_result--> COORD TAG=0x06
      --> 单片机（OLED 闪一句「拍照 成功!/失败!」）。

    ⚠ **有回应**: 存盘成功 OpenMV 回 "P,1"（失败 "P,0"）—— 所以本命令**不是**只发不回,
      单片机那边能真的知道成没成。
    ⚠ **超时兜底**: 到 PHOTO_ACK_TIMEOUT_S 还没等到回执就回一条"失败"。没有它的话,
      相机没上电 / 旧固件时单片机会**一直沉默**, "按了没反应"根本查不出来。
    ⚠ **不影响巡线**: OpenMV 那边只写文件, 采样循环与 'G'/'S' 发送开关都归别人管,
      所以拍照与巡线可以同时进行 —— 不像 '1'/'4'/'5'/'6' 那样要先停巡线腾主摄。
      （也因此不用像那几个命令那样在 _cmd_* 里调 _cmd_line_stop。）
    """
    global _photo_timer
    if not openmv_reader.send_command(b'P'):
        # 连串口都没打开 → 直接回"失败", 别让单片机白等到超时
        print("[WARN] 拍照失败：OpenMV 串口未连接（见 openmv_serial 的未连接提示）")
        _send_photo_ack(False)
        return
    print("[INFO] 已通知 OpenMV 拍照（存到相机盘 photos/），等回执")
    if _photo_timer is not None:
        _photo_timer.cancel()
    _photo_timer = threading.Timer(PHOTO_ACK_TIMEOUT_S, _photo_timeout)
    _photo_timer.daemon = True      # 别拦住进程退出
    _photo_timer.start()


def is_aim_running():
    """靶标流线程是否在跑（Web 状态显示用）。"""
    return _aim_thread is not None and _aim_thread.is_alive()


# ---- OpenMV 补光亮度 / 临时开关（命令 'L' 与 'T'）----
# 单片机 OLED「巡线→调参→补光亮度」与积木 STEP_OPENMV_LED 都发 "L,<n>"（vRpiOpenmvLed）;
# 积木 STEP_OPENMV_LED_TOGGLE 发裸 "T"（只发不回）;
# Web 模拟器 / 手动测试可发裸 "L"（= 把 config 记住的值重发一遍）。

# 最近一次**真正下发成功**的亮度（None = 本进程启动后还没发过）。
# 唯一消费者 = _cmd_openmv_led_toggle（判断该灭还是该亮）; 由 _apply_openmv_led 刷新。
_led_last = None


def _apply_openmv_led(level):
    """把亮度真正写给 OpenMV（"L<n>\n" 运行时命令, 相机端 _param_apply 收）。

    返回 True=已发出。OpenMV 未连接时返回 False —— 调用方各自决定怎么提示;
    值已经记进 config, 下次巡线起流前会自动重发, 所以断线时设置**不丢**。
    """
    global _led_last
    ok = openmv_reader.send_command(b"L%d\n" % int(level))
    if ok:
        _led_last = int(level)
    return ok


def _cmd_openmv_led(num=None):
    """命令 'L'：设置 OpenMV 补光亮度（0~100）。

    "L,<n>" = 设为 n（单片机专用发送器保证 0 也带后缀）;
    裸 "L"   = 重发 config.OPENMV_LED_BRIGHTNESS 当前值（Web 测试/手动对齐用）。
    ≥1 才**持久化**（config.save_override）：Pi 重启、OpenMV 重启都不丢 ——
    巡线起流("2")前会按 config 记住的值自动重发一遍。

    ⚠️ **0 = 临时灭灯, 刻意不写 config**。config 里那个值是「点亮时用多少」的
    **唯一真相**: 被 0 覆盖掉之后, 临时开关（命令 'T'）、OLED「补光亮度」、
    起流对齐就全都只能开回 0 —— 现场现象正是「关一次之后就只剩 0, 回不到 20」。
    要长期不亮就别开灯; 值留着不碍事。
    """
    if num is None:
        level = int(config.OPENMV_LED_BRIGHTNESS)
    else:
        try:
            level = int(num)
        except (TypeError, ValueError):
            print(f"[WARN] 补光亮度参数不合法: {num!r}（要 0~100 的整数）")
            return
    level = max(0, min(100, level))
    ok = _apply_openmv_led(level)
    if level == 0:
        print(f"[INFO] OpenMV 补光临时熄灭 → 0（未持久化: 点亮值 "
              f"{int(config.OPENMV_LED_BRIGHTNESS)} 保留, 命令 \"T\" 或下次巡线起流即恢复）"
              + ("" if ok else "（OpenMV 未连接, 未生效）"))
        return
    print(f"[INFO] OpenMV 补光亮度 → {level}"
          + ("" if ok else "（OpenMV 未连接, 值已记住, 巡线起流前自动重发）"))
    if level != int(config.OPENMV_LED_BRIGHTNESS):
        saved, msg = config.save_override("OPENMV_LED_BRIGHTNESS", level)
        if not saved:
            print(f"[WARN] 补光亮度持久化失败: {msg}")


def _cmd_openmv_led_toggle():
    """命令 'T'：**临时**开关 OpenMV 补光（只发不回，无结果帧）。

    亮 → 灭（发 0）; 灭 → 亮（重发 config.OPENMV_LED_BRIGHTNESS）。
    **全程不写 config** —— 所以"灭"只是临时的: Pi 重启, 或下一次巡线起流("2")前
    那次自动对齐, 都会把灯点回原亮度。

    ⚠️ 判据是「**本进程最近一次真正下发的亮度**」而不是一个自己的开关位:
    命令 'L' 也刷新它, 于是「先用积木设成 0、再点开关」不会反向 —— 否则那种顺序下
    开关会亮→灭→亮地原地打转, 看着像没生效。
    ⚠️ 进程刚起来（_led_last is None）当作"亮": 相机上电就是它的编译默认亮度, 且
    起流前还会对齐一次 —— 所以开机后第一次按它 = 关灯, 符合直觉。
    """
    level = 0 if (_led_last is None or _led_last > 0) else int(config.OPENMV_LED_BRIGHTNESS)
    ok = _apply_openmv_led(level)
    print(f"[INFO] OpenMV 补光临时{'熄灭' if level == 0 else '点亮'} → {level}"
          + ("" if ok else "（OpenMV 未连接, 未生效）"))


# ---- 巡线转发（命令 '2' 开始 / '3' 停止）----
# 巡线是长任务（持续发帧），跑独立线程，不阻塞主循环 —— 期间仍可收 '3'/end/r。
_line_thread = None
_line_abort = threading.Event()

# 危险指令（关机/重启）的二次确认状态: payload → 第一次到达的时刻。
# 见 _require_double_send; 派发在主循环单线程里, 不需要锁。
_system_cmd_arm = {}

# 危险指令「已执行」的时刻: payload → 上次真正执行的时刻（执行后的冷却, 防连发打穿确认门）。
_system_cmd_last = {}
_line_lock = threading.Lock()   # 保证同时只有一个巡线线程


def _line_worker():
    """巡线线程体：跑 run_line_follow，结束后清理线程标记。"""
    global _line_thread
    try:
        run_line_follow(abort_event=_line_abort)
    except Exception as e:
        import traceback
        print(f"[ERROR] 巡线线程异常: {type(e).__name__}: {e}")
        traceback.print_exc()
    finally:
        _line_abort.clear()
        _line_thread = None
        openmv_reader.send_command(b'S')   # 巡线结束(含超时/断线/异常)兜底停止 OpenMV 发送


def _cmd_line_start():
    """命令 '2'：开始巡线转发（OpenMV → PID → TYPE_COORD 实时发单片机）。"""
    global _line_thread
    # 两条流互斥（主摄 + COORD 通道）：巡线起流前先停靶标流。
    # ⚠ 放在抢 _line_lock **之前**：反过来的话，若巡线已在跑（下面那个分支会 return），
    #    靶标流就不会被停掉 —— 于是两条流同时在发 COORD 帧。
    _cmd_aim_stop()

    if not _line_lock.acquire(blocking=False):
        print("[WARN] 巡线已在运行，忽略重复的启动指令")
        return
    try:
        if _line_thread is not None and _line_thread.is_alive():
            print("[WARN] 巡线已在运行，忽略重复的启动指令")
            return
        # 按需发送：先把补光亮度对齐到 config 记住的值（幂等, 一条 8 字节）——
        # OpenMV 断电重启会回编译默认(40), 不重发的话现场调好的亮度悄悄丢。
        # 发不出（未连接）不拦起流: 值在 config 里, 下次起流还会试。
        try:
            if _apply_openmv_led(int(config.OPENMV_LED_BRIGHTNESS)):
                print(f"[INFO] 补光亮度已对齐: {int(config.OPENMV_LED_BRIGHTNESS)}")
        except Exception as e:
            print(f"[WARN] 补光亮度对齐失败(不影响起流): {e}")
        # 再触发 OpenMV 开始发（未连接时仅告警，巡线线程仍启动但读到 valid=0）
        if not openmv_reader.send_command(b'G'):
            print("[WARN] OpenMV 未连接，无法触发发送（巡线将读到 valid=0 丢线帧）")
        _line_abort.clear()
        _line_thread = threading.Thread(target=_line_worker, name="line-follow", daemon=True)
        _line_thread.start()
        print("[INFO] 巡线转发已启动")
    finally:
        _line_lock.release()


def _cmd_line_stop():
    """命令 '3'：停止巡线转发。"""
    if _line_thread is None or not _line_thread.is_alive():
        print("[WARN] 巡线未在运行，忽略停止指令")
        return
    _line_abort.set()
    openmv_reader.send_command(b'S')   # 立即触发 OpenMV 停止发送（finally 兜底幂等）
    print("[INFO] 已发送巡线停止信号")


def is_line_running():
    """巡线线程是否在跑（Web 状态显示用）。"""
    return _line_thread is not None and _line_thread.is_alive()


# ---- 系统级操作（命令 'sd' 关机 / 'rb' 重启；单片机 OLED 菜单用）----
#
# ⚠️ 与 'r'（只重启 GKS 主程序）**不是一回事**: 这两个停/重启整台树莓派。
# ⚠️ 服务是 `User=xixi`（见 gks.service）、**不是 root**, 所以必须提权:
#     ① `sudo -n ...` 需要一条免密 sudoers 规则（推荐, 见下）;
#     ② 直接 `systemctl` 走 polkit —— 后台服务没有活动会话, 默认策略常常直接拒绝。
#    两条都失败时**只打日志不抛异常**（免得把主循环带崩）, 由操作员看日志处置。
#    免密规则（现场一次配好即可, 用 `sudo visudo -f /etc/sudoers.d/gks`）:
#        xixi ALL=(ALL) NOPASSWD: /usr/bin/systemctl poweroff, /usr/bin/systemctl reboot
def _require_double_send(data: bytes, what: str) -> bool:
    """危险指令的二次确认门: **窗口内第二次收到同一条指令**才返回 True。

    为什么放这里（Pi = 执行方）而不是单片机侧: 无论指令是 OLED、地图 Tab 还是别的东西发的,
    都绕不过这一关; 单片机那边只要「两次操作各发一帧」就行。

    ⚠️ 「两次」指**两次操作**（按两次键）, 不是连发两帧 —— 连发两帧等于没有确认。
    ⚠️ 窗口可在 Web「设置参数」里调（`SYSTEM_CMD_CONFIRM_S`, 默认 10s）。
       超窗口再发 = 重新登记（first 变成本次）, 所以隔很久误按一次什么也不会发生。
    ⚠️⚠️ **执行后必须冷却一个窗口**（`_system_cmd_last`）: 少了它，连着发的第 3 帧会立刻
       重新登记、第 4 帧就通过 —— 「卡住的连发/按键抖动」会把确认门直接打穿（实测抓到过）。
       有冷却则一串重复最多一个窗口执行一次。
    ⚠️ 状态是模块级的 dict: 派发在主循环单线程里, 不需要锁。
    """
    from config import SYSTEM_CMD_CONFIRM_S

    now = time.time()

    t_last = _system_cmd_last.get(data)
    if t_last is not None and (now - t_last) <= SYSTEM_CMD_CONFIRM_S:
        print(f"[WARN] {what}: {now - t_last:.1f}s 前刚执行过, "
              f"{SYSTEM_CMD_CONFIRM_S:.0f}s 冷却期内不再响应")
        return False

    t0 = _system_cmd_arm.get(data)
    _system_cmd_arm[data] = now          # 本次成为新的「第一次」

    if t0 is not None and (now - t0) <= SYSTEM_CMD_CONFIRM_S:
        _system_cmd_arm.pop(data, None)  # 用掉了
        _system_cmd_last[data] = now     # 开始冷却
        print(f"[INFO] {what}: 二次确认通过（{now - t0:.1f}s 内两次）")
        return True

    print(f"[WARN] {what}: 第 1 次收到, {SYSTEM_CMD_CONFIRM_S:.0f}s 内再发一次才执行（防误触）")
    return False


def _system_priv_cmd(candidates, what: str) -> bool:
    """按顺序试几条提权命令, 命中第一条能用的就返回 True（全部失败返回 False）。"""
    for cmd in candidates:
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        except (OSError, subprocess.SubprocessError) as e:
            print(f"[WARN] {what}: 执行 {cmd} 失败: {e}")
            continue
        if r.returncode == 0:
            print(f"[INFO] {what}: 已执行 {' '.join(cmd)}")
            return True
        print(f"[WARN] {what}: {' '.join(cmd)} 退出码 {r.returncode} "
              f"{(r.stderr or '').strip()[:160]}")
    print(f"[ERROR] {what}: 所有提权方式都失败了 —— 见 handler.py 顶部的免密 sudoers 说明")
    return False


def _deferred_system_cmd(candidates, what: str, delay: float = 1.0):
    """延迟 + 独立线程执行系统级操作。

    ⚠️ 必须延迟且另起线程: 关机/重启会立刻掐断本进程 —— 在调用线程里同步执行的话,
       本次日志（`[INFO] 收到系统关机指令`）可能还没落盘, 串口那边也来不及收尾。
       延迟 1s 让日志与串口缓冲先走完, 再动手。
    """
    def worker():
        time.sleep(delay)
        _system_priv_cmd(candidates, what)
    threading.Thread(target=worker, daemon=True).start()


def _cmd_shutdown():
    """命令 'sd'：树莓派**系统关机**（单片机 OLED 菜单用）—— **连发两次才执行**。

    ⚠️ 关机后本端不再发心跳 → 单片机 8s 后把链路自动判为 DISCONNECTED（预期现象）;
       要恢复得给树莓派重新上电。车在跑的时候误触发 = 视觉全丢, 故有二次确认门。
    """
    if not _require_double_send(b"sd", "系统关机"):
        return
    print("[INFO] 收到系统关机指令（sd），1s 后执行")
    _deferred_system_cmd([["sudo", "-n", "systemctl", "poweroff"],
                          ["sudo", "-n", "shutdown", "-h", "now"],
                          ["systemctl", "poweroff"]], "系统关机")


def _cmd_reboot():
    """命令 'rb'：树莓派**系统重启**（同上, 关机改重启; 同样**连发两次才执行**）。

    重启后 gks.service 会自启（`Restart=always` + `WantedBy`）, 起来会重新握手 →
    单片机侧链路自行恢复（无需人工干预）。
    """
    if not _require_double_send(b"rb", "系统重启"):
        return
    print("[INFO] 收到系统重启指令（rb），1s 后执行")
    _deferred_system_cmd([["sudo", "-n", "systemctl", "reboot"],
                          ["sudo", "-n", "reboot"],
                          ["systemctl", "reboot"]], "系统重启")


# 命令注册表：DATA(payload) → 任务函数。
# 单片机端发来的 TYPE_CMD 帧 payload 查此表分发。
# ⚠ 命令码须与 docs/串口协议.md 及 GKS_Web/simulator.py 的指令表保持一致。
# 接受「<命令码>,<数字>」这个形状的指令（见 docs/串口协议.md §6）。两组含义不同、
# 共用同一个形状与同一张白名单：
#   · **阻塞类视觉任务**（"1"/"D"/"E"/"F"/"G"）→ 数字 = **毫秒** = 单片机愿意等多久。
#     它们的耗时由单片机那一步的超时决定，随指令发下来就不用在两个仓库各记一个数字互相迁就。
#   · **预热类**（"B"/"C"）→ 数字 = **使用次数** = 这台相机接下来要被几个任务消费。
#     相机租约的空闲窗口按它放大（camera_lease.idle_limit_s），于是隔了十几秒的
#     第二个任务也能命中同一份租约。
# ⚠️ 这是**白名单**：没登记过的指令若带了逗号会被判成「形状不认识」而拒掉 ——
#    宁可响亮地拒，也不要静默忽略那个数字（少了它本端会按自己的默认值跑，
#    有可能跑得比单片机愿意等的还久）。
# ⚠️ 旧固件发的是裸 "B"/"C"，照样走得了（_split_payload 不带逗号时 num=None → 次数 1 =
#    老行为）。反过来新固件 + 旧 Pi 才会失效：那一条预热被判未知指令丢掉，随后的识别
#    退回「现开现扫」（慢 ~2s, 不坏）—— 所以升级顺序是先 Pi 后固件。
_MS_COMMANDS = (b"1", b"D", b"E", b"F", b"G", b"B", b"C", b"4", b"6", b"L")

_COMMANDS = {
    b"1": _cmd_qr,              # 二维码识别（同步阻塞）
    b"2": _cmd_line_start,      # 巡线转发开始（独立线程）
    b"3": _cmd_line_stop,       # 巡线转发停止
    b"4": _cmd_hostage,         # 人质形状识别（同步阻塞）
    b"5": _cmd_color,           # 色球顺序识别（同步阻塞）
    b"6": _cmd_target,          # 靶标识别 + 靶心定位（同步阻塞）
    b"7": _cmd_aim_start,       # 靶标流跟踪开始（独立线程, 0x05 流帧 ~20Hz）
    b"8": _cmd_aim_stop,        # 靶标流跟踪停止
    b"sd": _cmd_shutdown,       # 树莓派系统关机（延迟执行, 见 _cmd_shutdown）
    b"rb": _cmd_reboot,         # 树莓派系统重启（延迟执行）
    b"9": _cmd_photo,           # 拍照（转告 OpenMV 存一张到它盘上的 photos/；只发不回）
    b"A": _cmd_rect,            # 色球区矩形地板识别（主摄，阻塞；回 0x07 帧）
    b"B": _cmd_warm_qr,         # 预热扫码相机（只发不回, 见 camera_lease.py）
    b"C": _cmd_warm_main,       # 预热主摄（只发不回, 同上）
    b"D": _cmd_qr_main,         # 二维码识别（主摄；与 "1" 同链同帧, 只换相机）
    b"E": _cmd_qr_diff,         # 二维码识别（自研难度算法；与 "1" 同链同帧, 只换解码器）
    b"F": _cmd_qr_diff_main,    # 二维码识别（自研难度算法 + 主摄；斜视扫码那条路）
    b"G": _cmd_road,            # 看路（主摄量黄色区左边缘+下边缘，阻塞；回 0x08 帧）
    b"x": _cmd_cancel,          # 取消当前任务/流（单片机中止脚本/重握手时发; 只收不回）
    b"L": _cmd_openmv_led,      # OpenMV 补光亮度（"L,<0~100>"; 裸"L"=重发 config 值; 只发不回）
    b"T": _cmd_openmv_led_toggle,   # OpenMV 补光**临时**开关（裸"T", 无参; 亮↔灭; 只发不回）
    # TODO(工科赛): 物块检测/色环/码垛/扫雷/转盘等任务命令在此追加
}


class SerialInterruptHandler:
    def __init__(self, should_stop=None):
        self.running = True
        # 可选的外部停止谓词：返回 True 时主循环退出（用于 SIGTERM/SIGINT 优雅退出）
        self._should_stop = should_stop or (lambda: False)

    def handle(self, data):
        """分发一条命令 payload。

        payload 可以是 `<命令码>` 或 `<命令码>,<毫秒>`（后者 = 单片机告诉本端
        「我最多等你这么久」，见 docs/串口协议.md §6）。拆出来的头用来查表，
        **不带逗号时 head == data，行为与加这个特性之前一字不差**。
        """
        print(f"收到指令: {data!r}")

        # num = 命令码后面那个数值。**含义按命令码而定**（毫秒 / 使用次数），见 _split_payload。
        head, num = _split_payload(data)

        # 系统命令（不走任务注册表）
        if data == b"end":
            print("[INFO] 收到结束指令，主循环退出")
            _cmd_line_stop()          # 停巡线线程，避免主循环退出后仍在发帧
            _cmd_aim_stop()           # 同理：靶标流线程也要停，否则会占着相机继续发帧
            camera_lease.LEASE.release_now()   # 别把预热好的相机带进下次运行
            self.running = False
            return
        if data == b"r":
            # ⚠️ 与 'sd'/'rb' 同样要**连发两次**: 重启主程序也会中断当前任务/路线
            if not _require_double_send(b"r", "重启程序"):
                return
            _cmd_line_stop()
            _cmd_aim_stop()           # 同理
            camera_lease.LEASE.release_now()   # 同理: 重启前把相机交还, 否则新进程起不来
            serial_comm.restart_program()
            return

        if head in (b"1", b"4", b"5", b"6", b"A", b"D", b"E", b"F", b"G"):
            _cmd_line_stop()          # 结果帧与巡线帧同为 COORD，先停巡线防帧交错
            # ⚠️ 靶标流也要停：流帧与"6"的结果帧同为 COORD（都是 Pi→单片机），
            #    交错起来单片机那边会拿流帧去喂 wait_target_result。而且这些任务
            #    都要用主摄 —— 流线程占着设备的话 open_camera 会直接失败。
            #    ⚠️ "D"（主摄扫码）同样在主摄上，所以也在这个元组里。
            #    ⚠️ "B"/"C"（预热）**刻意不在**：它们只开相机、不发结果帧，
            #       而且 "B" 恰恰排在巡线**之前** —— 停巡线是错的。别顺手加进来。
            _cmd_aim_stop()

        try:
            func = _COMMANDS.get(head)
            if func is None:
                print(f"[WARN] 未知指令: {data!r}")
                return
            # 新指令开始前清零取消事件（"x" 自己的 func 会在 clear 之后 set, 顺序安全）。
            # 连发 "x"+"5" 时: 先取消旧任务, 再干净地起任务 —— 语义就是「顶掉重来」。
            _task_abort.clear()
            if num is not None:
                func(num)             # 带数值后缀的指令（_MS_COMMANDS 里那几条; 毫秒 or 次数）
            else:
                func()
        except SerialDisconnected as e:
            print(f"[WARN] 任务中止: {e}，返回主循环等待重连")
        except Exception as e:
            import traceback
            print(f"[ERROR] 未捕获异常: {type(e).__name__}: {e}")
            traceback.print_exc()

    # ---- 握手 ----
    def _handshake(self, should_stop=None):
        """与单片机握手，阻塞直到成功，或 should_stop() 为真时放弃。

        每 10s 打一次进度（已等时长 + 已发次数 + 当前连接模式），否则对端不在线时
        日志只有开始那一行，看不出是「没插好」还是「单片机没在跑」。

        ⚠ 必须尊重停止信号：否则主线程会永远卡在这个重试循环里，Ctrl+C/SIGTERM
        只能打印「正在退出」却退不掉（进程无法响应停止）。
        返回 True=握手成功，False=被停止信号打断。
        """
        should_stop = should_stop or (lambda: False)
        print(f"[INFO] 正在握手：发 {HANDSHAKE_DATA}，"
              f"等单片机回 ACK 0x{HANDSHAKE_ACK_BYTE:02X} ...")
        t0 = time.time()
        tries = 0
        next_log = 10.0
        while not should_stop():
            try:
                serial_comm.send(HANDSHAKE_DATA)
            except SerialDisconnected:
                raise
            tries += 1
            if serial_comm.wait_for_ack(HANDSHAKE_ACK_BYTE, timeout_s=2.0):
                print(f"[INFO] 握手成功（第 {tries} 次尝试，用时 {time.time() - t0:.1f}s），"
                      f"收到单片机响应 0x{HANDSHAKE_ACK_BYTE:02X}")
                # 走完一次真收发 = 链路真好：清掉连续重启计数（见 serial_comm 顶部常量）
                serial_comm.note_link_ok()
                return True
            if should_stop():
                break
            waited = time.time() - t0
            if waited >= next_log:
                next_log = waited + 10.0
                print(f"[WARN] 握手未成功（已 {waited:.0f}s，共发 {tries} 次，"
                      f"当前为 {serial_comm.mode or '未连接'} 模式）：未收到单片机 ACK"
                      f" —— 确认单片机已上电且在跑、主串口接对、波特率 115200")
            time.sleep(0.5)
        print("[INFO] 握手等待被停止信号打断，放弃")
        return False

    def _is_serial_ok(self):
        return serial_comm.ser is not None and serial_comm.ser.is_open

    # ---- 主循环 ----
    def run(self):
        """跑主循环；**因链路故障而退出的情形改为原地重启**，只有真被叫停才退出进程。

        ⚠⚠ 原来这里把一切都当「收到中断信号」并直接退出，于是**握手阶段**的一次串口
        I/O 错误就把整个进程带走了：USB 串口适配器掉线 → `receive()` 读 `in_waiting`
        抛 `OSError [Errno 5] Input/output error` → `_mark_disconnected()` 把 ser 置空
        → `_handshake()` 下一轮 `send()` 抛 SerialDisconnected（那里只有
        `except SerialDisconnected: raise`）→ 一路上抛到这里 → 打印「收到中断信号」
        然后退出。而调试模式（GKS_Web 控制台 start_main_py 的 Popen）下**没有任何东西
        会把它重新拉起来**，现场现象就是「Pi 掉一次串口，车从此不响应，只能手动重启」。
        （_run() 主循环里那个「串口断开则重连并重新握手」的分支覆盖不到握手阶段。）

        现在按「谁让它退出的」分两种：
          · should_stop()（Ctrl+C / SIGTERM / 菜单）为真 → 真退出，照旧打印清理信息；
          · 否则就是链路自己断了 → serial_comm.restart_process() 原地重启（不退出）。
        """
        restart_reason = ""
        try:
            self._run()
        except (OSError, SerialDisconnected) as e:
            if self._should_stop():
                print("\n[INFO] 收到中断信号，正在清理资源...")
            else:
                restart_reason = f"串口链路故障: {type(e).__name__}: {e}"
        except (KeyboardInterrupt, ValueError):
            print("\n[INFO] 收到中断信号，正在清理资源...")
        finally:
            try:
                serial_comm.close()
            except Exception as e:
                print(f"[WARN] 关闭串口失败: {e}")

        if restart_reason:
            # 正常路径下这里**不会返回**（os.execl 换掉进程镜像）；会返回只有两种情形：
            # 等待重启期间又收到停止信号（放弃重启），或 exec 本身失败 —— 两种都照常落下、
            # 按退出处理，不会静默卡住。
            serial_comm.restart_process(restart_reason, should_stop=self._should_stop)
        print("[INFO] 资源已释放，程序退出。")

    def _run(self):
        if not serial_comm.connect(should_stop=self._should_stop):
            return   # 启动阶段被停止信号（SIGTERM/SIGINT）打断
        serial_comm.flush_input()
        if not self._handshake(self._should_stop):
            return   # 握手等待期间收到停止信号
        # ⚠️ 2026-10 起握手成功后**不再 flush_input()** —— 旧版这一下会把单片机
        #    在「回 ACK → 收到我们下一条指令」之间发来的第一条指令（断电重跑后
        #    的 "1"/"B", 落点正是这个窗口）整段清掉, 之后两边互相干等。残留垃圾帧
        #    由 receive() 的队列 + 主循环的未知指令 WARN 兜底（ACK 帧本来就被跳过）。
        mode_name = "模拟" if serial_comm.mode == "simulator" else "正常"
        print(f"系统就绪（{mode_name}模式），等待单片机指令...")

        last_idle_print = time.time()
        last_heartbeat = time.time()

        while self.running and not self._should_stop():
            try:
                # 串口断开则重连并重新握手
                if not self._is_serial_ok():
                    print("[WARN] 检测到串口断开，尝试重连...")
                    if not serial_comm.reconnect(should_stop=self._should_stop):
                        break
                    serial_comm.flush_input()
                    if not self._handshake(self._should_stop):
                        break   # 重连握手期间收到停止信号，退出主循环
                    # ---- 旧会话收尾（2026-10 加）----
                    # 断线期间单片机可能整台断电重跑过: 它发不出 "3"/"8"/"end",
                    # 旧巡线流/靶标流线程若写串口没报错就一直活着（占主摄 + 以
                    # 20~30Hz 发 COORD 帧, 与新任务结果帧交错）; OpenMV 也可能还
                    # 停留在上一次 'G' 的发送状态。这里统一收掉, 新会话从干净状态开始。
                    # （阻塞任务不用收: 主循环是单线程, 能走到这里说明它已返回。）
                    if is_line_running():
                        print("[INFO] 串口重连: 停掉旧会话残留的巡线流")
                        _cmd_line_stop()
                    if is_aim_running():
                        print("[INFO] 串口重连: 停掉旧会话残留的靶标流")
                        _cmd_aim_stop()
                    openmv_reader.send_command(b'S')   # OpenMV 若还在按旧 'G' 发数据, 停掉
                    mode_name = "模拟" if serial_comm.mode == "simulator" else "正常"
                    print(f"[INFO] 串口重连并握手成功（{mode_name}模式），继续等待指令...")
                    last_idle_print = time.time()
                    last_heartbeat = time.time()
                    continue

                data = serial_comm.receive()
                if data:
                    _ftype, payload = data
                    if _ftype == TYPE_ACK:
                        continue  # 忽略应答/握手残留帧，避免被当成指令
                    self.handle(payload)
                else:
                    # 空闲心跳帧：TYPE_HELLO 空帧，让单片机端据此检测链路存活
                    # （超时未收到 → 判 Disconnected；重连重握手后恢复 OK）
                    now = time.time()
                    if now - last_heartbeat >= RPI_HEARTBEAT_INTERVAL:
                        try:
                            serial_comm.send_frame(TYPE_HELLO, b"")
                            last_heartbeat = now
                        except SerialDisconnected:
                            pass   # 断开由外层重连逻辑处理
                    if now - last_idle_print > 30:
                        print("[INFO] 心跳: 空闲等待中...")
                        last_idle_print = now
                    time.sleep(0.05)
            except (OSError, RuntimeError, SerialDisconnected) as e:
                print(f"[WARN] 主循环异常: {e}，尝试重连串口...")
                try:
                    serial_comm.ser.close()
                except Exception:
                    pass
                serial_comm.ser = None
                time.sleep(0.5)
