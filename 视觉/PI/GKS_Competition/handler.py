# handler.py — 单片机指令分发（GKS 工科赛）
#
# 精简版移植自 cv_python/handler.py。主循环收帧 → 查注册表 → 调用任务函数。
# 后续接入物块/色环/码垛/扫雷/转盘等任务时，只需在 _COMMANDS 里追加一条
# `b"<命令码>": 任务函数`，无需改动主循环。
import json
import os
import subprocess
import threading
import time

from serial_comm import serial_comm, SerialDisconnected
import config
from protocol import TYPE_ACK, TYPE_CMD, TYPE_HELLO, TYPE_COORD, TAG_HEALTH
from config import HANDSHAKE_DATA, HANDSHAKE_ACK_BYTE, RPI_HEARTBEAT_INTERVAL
from config import PHOTO_ACK_TIMEOUT_S
from tasks import run_qr_wechat, run_qr_wechat_main, run_line_follow
from tasks import run_qr_diff, run_qr_diff_main
from tasks import run_qr_auto, run_qr_auto_main
from tasks import run_hostage_detect, build_result_frame
from tasks import run_color_detect, build_color_frame
from tasks import run_target_detect, build_target_frame
from tasks import run_target_stream
from tasks import run_bucket_stream
from tasks import run_rect_detect, build_rect_frame
# 看路（命令 "G"）：主摄量地面黄色区的左边缘 x / 下边缘 y，结果回 0x08 帧（见 docs/串口协议.md §6i）
from tasks import run_road_detect, build_road_frame
# 看路校准（命令 "H"）：主摄拟合黄区直边的图像方向角（陀螺漂移的视觉基准, 只测不改），
# 结果回 0x09 帧（见 docs/串口协议.md §6j）
from tasks import run_road_yaw_detect, build_road_yaw_frame
from openmv_serial import openmv_reader
# 相机租约（预热复用）—— 命令 "B"/"C" 用；见 camera_lease.py 顶部的长注释。
# 模块级导入是安全的: camera_lease 只依赖 camera, 不反向依赖 handler（无循环）。
import camera_lease
import camera_picam


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
    # ⚠️ 重启前先按**主循环那条 "r" 分支**的同一套收尾: 停两条流 + 交还相机租约。
    #    这条是接收线程侧的救援路径, 原来少了这一步 —— 流线程会一直往一个**即将被
    #    关掉的串口**写帧, 而 20s 后 os.execl 换进程镜像, 那些线程没人收尾。
    #    （收尾失败不拦重启: "r" 的语义就是"无论如何救活它"。）
    try:
        _cmd_line_stop()
        _cmd_aim_stop()
        _cmd_bucket_stop()             # 爆爆桶流同收（2026-10-24 加）
        camera_lease.LEASE.release_now()
        # ⚠️ CSI 那颗**必须一起收**（2026-10-16 加）: os.execl 只换进程镜像, **子进程活着** ——
        #    漏了这一步, 那个 rpicam-vid 会一直攥着 libcamera（独占）, 之后谁都开不了 CSI
        #    （症状 = 「pi摄像头: 失败（抓帧失败: failed to acquire camera）」, 只有人工 kill 或
        #    重启才恢复）。这条路径是**链路故障自动重启**, 没有人在旁边看, 所以更要收干净。
        camera_picam.PICAM.release_now()
    except Exception as e:                             # noqa: BLE001
        print(f"[WARN] 重启前收尾失败（不拦重启）: {e}")
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
         阻塞类视觉指令 "1"/"D"/"E"/"F"（扫码）"4"/"5"/"6"（人质/色球/靶标）
             "G"/"H"/"A"（看路/看路校准/矩形地板）"Z"（四路相机健康检查）→ **毫秒** = 「这条我最多等这么久」，
                                             见 _budget_from_ms；
         预热类指令 "B"/"C"                → **使用次数** = 「这台相机接下来要用几次」，
                                             放宽相机租约的空闲窗口，见
                                             camera_lease._clamp_uses / idle_limit_s。
         补光亮度指令 "L"                  → **亮度值** = OpenMV 补光 0~100（灯灭=0 合法），
                                             见 _cmd_openmv_led。
       而**第二段**（`_split_payload_ext` 的 n2, 2026-10-16 加）只有一种含义:
         "6"/"7"/"C"                       → **is_use_picam** = 1 优先 pi(CSI)摄像头,
                                             见 _TWO_NUM_HEADS。
       加新命令族时先想清楚自己属于哪一类。

    拆不成合法形状时返回 `(整串, None)`：调用方拿它查表必然查不到 → 响亮地报未知指令。
    刻意**不**静默忽略那个数字 —— 少了它本端会按自己的默认值跑，可能跑得比对方愿意等的还久。

    ⚠️ 2026-10-16 起多了一种**两段数字**的形状（`"6,6000,1"` = 靶标识别 + 优先 pi 摄像头），
       见 _split_payload_ext。老入口（本函数）仍只回 2 元组 ⇒ 那 ~20 处调用方一字不改。
    """
    head, num, _num2 = _split_payload_ext(data)
    return head, num


# 允许"两段数字"（`<码>,<n1>,<n2>`）的指令 —— 2026-10-16 加, 只为「平移射靶优先 pi(CSI)摄像头」:
#   "6,<毫秒>,<pi>"   "7,0,<pi>"（或简写 "7,<pi>"）   "C,<次数>,<pi>"
#   pi: 1 = 优先 CSI（它打不开/掉线/看不到靶时 Pi 内部退回主摄）, 省略/0 = 只用主摄。
# 2026-10-23: 带档位的预热用**单独的 "W" 头**（"W,<cam>,<uses>,<prof>"），在 handle() 里
#   整条拦走（同 "P"/"QA" 族），所以**不**进本表 —— 本表只覆盖"最多两段"的形状。
# ⚠️ 只有这几条能收两段: 别的指令多带一个逗号仍按"形状不认识"**响亮拒收**
#    （别放宽成"任意指令都能带"—— 那会把"打错的形状"变成静默的语义漂移）。
_TWO_NUM_HEADS = (b"6", b"7", b"C")


def _split_payload_ext(data):
    """`<命令码>[,<数字>[,<数字>]]` → (head, n1, n2)。认不出来 → (整串, None, None)。

    ⚠️ n2 的含义只有一种: is_use_picam（见 _TWO_NUM_HEADS）。
    ⚠️ `"7,N"` 的 N（只有一段时）也当 is_use_picam —— "7" 历史上从不带数字后缀,
       没有别的语义可冲突（本端发的是 `"7,0,<pi>"` 那种统一三字段, 两种都认）。
    """
    parts = data.split(b",")
    head = parts[0]
    if len(parts) == 1:
        return head, None, None

    def _bad(msg):
        print(f"[WARN] {msg}: {data!r}")
        return data, None, None

    if len(parts) > 3 or not all(p.isdigit() for p in parts[1:]):
        return _bad("指令形状不认识")
    if head not in _MS_COMMANDS:
        return _bad("该指令不接受数值后缀"
                    f"（能收的只有 {'/'.join(c.decode('ascii') for c in _MS_COMMANDS)}）")
    if head == b"7" and len(parts) == 2:
        return head, 0, int(parts[1])          # "7,N" → N 就是 is_use_picam
    if len(parts) == 3 and head not in _TWO_NUM_HEADS:
        return _bad("该指令不接受两个数值"
                    f"（能收两段的只有 {'/'.join(c.decode('ascii') for c in _TWO_NUM_HEADS)}）")
    return head, int(parts[1]), (int(parts[2]) if len(parts) == 3 else None)


def _budget_from_ms(ms):
    """把单片机给的超时（毫秒）换成**本端该用的预算**（秒）；None/非正 → None（用本端默认）。

    ⚠️ 必须减一个余量：单片机那边 `ucRpiWaitQrResult(ms)` 是从**它发指令的那一刻**开始
       计的，而这 ms 里本端还要做「开相机 + 扫描 + 回帧」。余量要盖住开相机（租约没命中
       时实测 ~1.3s）—— 否则冷启动那一次会**答得比单片机放弃还晚**，而迟到的结果帧会
       落进下一次重扫的等待窗口里被当成那一次的结果（见 tasks/_qr.py 的同名注释）。
       预热 "B"/"C" 能省掉那 1.3s，但余量照留：宁可少试一轮也不能迟到。
    """
    from config import CMD_MARGIN_S, CMD_MAX_TIMEOUT_S
    if not ms or ms <= 0:
        return None
    budget = ms / 1000.0 - CMD_MARGIN_S
    if budget > CMD_MAX_TIMEOUT_S:
        # 夹住：相机租约的空闲看门狗只被 read() 续命，单次扫描超过它会在**解码途中**
        # 把设备关掉，后续 read() 全失败、扫描循环空转到超时（现象是「一直超时」而不是报错）
        import config
        print(f"[WARN] 单片机给的超时 {ms}ms 超出本端上限，预算夹到 "
              f"{CMD_MAX_TIMEOUT_S:.0f}s（租约空闲看门狗 {config.LEASE_IDLE_TIMEOUT_S:.0f}s）")
        budget = CMD_MAX_TIMEOUT_S
    # ⚠️⚠️ **地板不能把预算抬到超过单片机给的时限**（2026-10 修）。
    #    原来是 `max(1.0, budget)` —— 朝**上**钳: ms=2000 时本端承诺跑 1.0s, 而
    #    `timeout` 只覆盖**扫描**、**不覆盖开相机**（实测冷开 ~1.5~2s, 它是靠上面那个
    #    1.5s 余量盖的）→ 本端必然答得比单片机放弃还晚, 而**迟到的结果帧会落进下一轮
    #    同类型步骤的等待窗口被当成那一次的结果**（协议 §6 明确警告的错位）。
    #    「谁先放弃只由单片机一个数决定」这条不变量在小 ms 下必须优先于"扫描要跑得动"。
    #    现在: 地板降到 _MIN_SCAN_BUDGET_S(0.2s), 并**响亮告警**让人去改积木的 I(0)。
    if budget < _MIN_SCAN_BUDGET_S:
        print(f"[WARN] 单片机给的超时 {ms}ms 太短（减掉 {CMD_MARGIN_S}s 余量后只剩 "
              f"{budget:.2f}s）—— 本端无法保证先答, 已把扫描预算压到 "
              f"{_MIN_SCAN_BUDGET_S}s; 请把路线里那一步的 I(0) 调大")
        return _MIN_SCAN_BUDGET_S
    return budget


def _save_frame(key, result, legacy_path):
    """任务结果帧落盘（2026-10 起统一走 logstore; 2026-10-21 起收**整个 result dict**）。

    一帧写三个地方:
      · logs/run_*/frames/<key>_NNNN.jpg —— **序号递增、永不覆盖**（比对第 N 次
        跑同一任务时相机各看到什么, 配合日志回看用）;
      · logs/run_*/last_<key>.jpg —— 本次运行内最新（run 目录内的便利副本）;
      · legacy_path（logs/<key>_last.jpg 固定路径）—— GKS_Web 的 /api/*/frame
        轮询端点读它, 保持兼容。
    统一约定（成功与失败**走同一个入口**, 记录行为不再看任务脸色）:
      · result["success"] 为 False → logstore 在帧上盖红字 FAIL 印再落盘
        （回看网格里一眼分清; 老调用方没这个键 = 不盖, 行为不变）;
      · result["frame_raw"]（处理前原始帧, 各任务在成功且帧被标注/裁剪过时才给）
        + 训练模式开着（Web 参数页 TRAIN_MODE_EN, 两个进程即时生效）→
        同序号再归一张 <key>_before_<n>.jpg（离线复跑/再训练用）。
      · 失败帧与成功帧**同等对待**: 只要任务给了帧（哪怕是"最后一帧原图"）就归档
        —— 失败样本恰是排障/训练最想要的, 以前失败什么都不留是缺口不是设计。
    任何失败只打 WARN —— 落帧是旁路, 不许影响任务本身。
    """
    if not isinstance(result, dict):
        return
    try:
        import logstore
        logstore.archive_frame(key, result.get("frame"), legacy_path=legacy_path,
                               before_frame=result.get("frame_raw"),
                               ok=result.get("success"))
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
        没给（老固件 / 不是从串口来的）就退回 config 里那两个各自的默认超时。
    """
    auto_qr = _web_auto_qr()
    if auto_qr:
        # Web 控制端「自动下发二维码」（调试）：不开相机，直接把预设值回给单片机
        # （为什么必须有这条见 _web_auto_qr 的头注释 —— 握手时推的帧可能被固件
        #  等待窗的清残留逻辑丢掉，单片机来问时在这里补上才是可靠送达）。
        print(f"[INFO] 二维码自动下发: QR{auto_qr}（Web 调试设置，跳过扫码）")
        _send_qr_result("QR" + auto_qr)
        return
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
    _save_frame("qr", result, _cfg("QR_LAST_FRAME_PATH"))


def _cfg(name):
    """按名取 config 常量（避免模块顶 import 一串 *_LAST_FRAME_PATH）。"""
    import config
    return getattr(config, name, None)


def _cmd_qr(ms=None):
    """命令 '1'：二维码识别（**新引擎**: 全帧定位→按检测框紧裁→强放大）。**扫码副摄**。

    2026-10 起改用 run_qr_auto：旧「固定中心裁剪 0.30 + 微信引擎」在现场真帧上
    基线只有 2/10、平移 6% 就全灭；新引擎同批帧 10/10（见 tasks/_qr_auto.py 文件头）。
    ⚠️ 对外完全同构：仍是 QR<ddd>/QRF 结果帧、仍是 111~333 取值域 —— 单片机侧无感。
    """
    _cmd_qr_common(run_qr_auto, ms)


def _cmd_qr_main(ms=None):
    """命令 'D'：二维码识别，但用**主摄**（/dev/camera_main）。

    与 '1' 同一条解码链（run_qr_auto）、同一种结果帧，只换相机。
    新引擎对主摄同样适用 —— 它按检测框自适应位置/尺度，不再依赖「码在画面中心」。
    也能吃 "C"（预热主摄）省下的开相机时间。
    """
    _cmd_qr_common(run_qr_auto_main, ms)


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


# 「最近一次发出结果帧」的时刻 —— 只给 handle() 的兜底 except 用。
# 任务函数抛异常时，只有**还没回过帧**才补发失败帧：已经回过一帧再补一帧，
# 那多出来的一帧会落进单片机**下一次**等待窗口被当成那一次的结果 —— 正是
# "x"/重扫机制要治的那种错位（协议 §6 明确警告过）。
_last_result_frame_t = 0.0


def _note_result_frame():
    global _last_result_frame_t
    _last_result_frame_t = time.time()


def _send_qr_result(text: str):
    """把二维码结果回传单片机（9B ASCII COORD 帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端 ucRpiWaitQrResult 会超时重试。
    """
    try:
        serial_comm.send(text)
        _note_result_frame()
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 二维码结果回传失败: {e}")


# ---- Web 控制端「自动下发二维码」（跨进程走文件，2026-10-11 加）----
# GKS_Web（gks-web.service）是独立进程，与本进程只共享文件系统：网页上勾选/填的
# 值写在本目录 web_auto_qr.json（原子替换，{"enabled": bool, "value": "213"}）。
# 消费点两处（都**现读不缓存**，网页改完下一次握手/扫码即生效，不用重启 main.py）：
#   ① 握手成功后立刻推一帧 **"QP<ddd>"（伪装帧）**：新固件的 vRpiTask 收到后当场
#      vSetQrCode + vSetQrManual → OLED 显示 `QR:213*` 且后续扫码步骤直接跳过
#      （与 OLED 第 2 页手设同一语义）。⚠️ 刻意不用 "QR" 前缀：那是扫码**结果**帧，
#      只该在等待窗内消费，复用会把真实扫码结果误标成伪装。
#   ② 单片机真发扫码指令（"1"/"D"/"E"/"F"）时直接回预设值、跳过相机 —— 这是
#      **旧固件兜底**（旧固件没有 QP 分支，握手时推的帧落进 last-packet 被忽略）。
#   ③ **中途改值监听**（2026-10-11 加）：守护线程 (~0.6s) 轮询本文件，值一变就
#      立刻重推 QP 帧 —— 比赛途中在 Web 上换码即刻生效，不必等下次握手/扫码。
#      固件对 QP 帧**没有一次性语义**（vRpiTask 收到就写值+打标记），重复到达无害。
# ⚠ 调试功能：勾选期间真实扫码全被替换，比赛前取消勾选（新固件的伪装标记还要
#   断电/复位才清，见 app_rpi.c QP 分支注释）。
WEB_AUTO_QR_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "web_auto_qr.json")

# 最近一次**已观察到的原始设置** (enabled, value字符串) —— 监听线程与握手推送共用，
# 用于"只在变化时推"（1Hz 轮询若不做变化去重会每秒重发一帧、刷爆调试串）。
# 只在两条推送路径成功/观察到时更新；None = 监听线程还没起步（未观测）。
_last_auto_qr_state = None


def _auto_qr_read_state():
    """读文件的原始状态 (enabled: bool, value: str)。读不到 = (False, "")."""
    try:
        with open(WEB_AUTO_QR_PATH, encoding="utf-8") as f:
            d = json.load(f)
    except (OSError, ValueError):
        return (False, "")
    return (bool(d.get("enabled")), str(d.get("value", "")).strip())


def _web_auto_qr():
    """读 Web 的自动二维码设置；未启用/文件不存在/值非法 → None。

    值域：三位数字且**每位 1~3**（与固件二维码语义同域：百位=目标色 / 十位=靶色 /
    个位=序号-形状，各 1~3 共 27 种 —— 同 app_rpi.c vSetQrCodeRandom 的取值域）。
    """
    enabled, v = _auto_qr_read_state()
    if not enabled:
        return None
    if len(v) == 3 and all(c in "123" for c in v):
        return v
    print(f"[WARN] Web 自动二维码设置值非法（{v!r}，须每位 1~3），忽略")
    return None


def _auto_qr_push(value, source):
    """推一帧伪装帧 "QP<ddd>" 给单片机，成功返回 True 并记录已推送状态。

    链路没通（串口未打开）直接返回 False 不打印 —— 监听线程每次轮询都会重试，
    等链路回来自然推上（或下次握手补推当前值）；不让"没插线"刷出一屏 WARN。
    """
    global _last_auto_qr_state
    ser = getattr(serial_comm, "ser", None)
    if ser is None or not getattr(ser, "is_open", False):
        return False
    try:
        serial_comm.send("QP" + value)
        _note_result_frame()
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 自动二维码下发失败: {e}")
        return False
    _last_auto_qr_state = (True, value)
    print(f"[INFO] 自动二维码下发（{source}）: QP{value} "
          f"→ 单片机 OLED 显示 QR:{value}*")
    return True


def _auto_qr_watch_step():
    """监听一步：读文件，设置**有变化**且新值启用有效 → 推。返回本次推的值或 None。

    ⚠️ 变化判据是 (enabled, value) 整对：勾选框来回勾、同一值重新启用都会算变化
    → 重推一次（幂等；防"中途 MCU 复位丢了标记、重新启用同一值却不补推"）。
    值非法（如 205）只记录状态、不推 —— 修成合法值算下一次变化，照常推。
    ⚠️ 推送**成功**才记录状态（在 _auto_qr_push 里做）：链路断开时本轮不算数,
    下一轮自动重试；彻底没连上则由下次握手补推当前值（那条路是无条件推的）。
    """
    global _last_auto_qr_state
    st = _auto_qr_read_state()
    if st == _last_auto_qr_state:
        return None
    v = _web_auto_qr()          # 启用 + 合法才有值（非法会打 WARN）
    if not v:
        _last_auto_qr_state = st        # 没东西可推: 状态照记, 免得每轮重读重判
        return None
    if _auto_qr_push(v, "Web 改值"):
        return v
    return None                 # 链路没通: 状态未记, 下一轮再试


def _auto_qr_watch_loop(interval=0.6):
    """守护线程主体：轮询 web_auto_qr.json，值一变就立刻推 QP 帧。

    ⚠️ 起始值只记录、不推（初始下发是 **握手** 的职责 —— 那时链路刚验证过、
    推得进；本线程可能在 Pi 还没握上手时就起步，抢推是白发）。之后的每次
    变化才是本线程的活。
    ⚠️ 整步套 try：守护线程**绝不能死**（一次 json 竞态/IO 抖动就把中途改值
    这个功能永久废掉，且没有任何表面症状）。
    """
    global _last_auto_qr_state
    _last_auto_qr_state = _auto_qr_read_state()
    print("[INFO] 自动二维码中途改值监听已启动（Web 改数字后立即同步给单片机）")
    while True:
        time.sleep(interval)
        try:
            _auto_qr_watch_step()
        except Exception as e:                          # noqa: BLE001
            print(f"[WARN] 自动二维码监听异常（继续）: {type(e).__name__}: {e}")


# ---- OLED 第 2 页「二维码功能」指令族（"QA"/"QE"/"QU"/"QC"，2026-10 加）----
# 单片机 OLED 第 2 页的功能菜单直接读写 Web 的「自动下发二维码」设置（web_auto_qr.json）:
#   "QA"        查询       → 回 "QS,<e>,<值>"（TAG=0x0A; e: 1=开 0=关; 值=三位 或 "---"）
#   "QE,<0|1>"  开/关      → 回 QS（值保持不变）
#   "QU,<ddd>"  上传并启用 → 写 {"enabled":true,"value":ddd} → 回 QS
#                          （中途改值监听随后把 QP 伪装帧推回单片机 —— 同值, 幂等;
#                            这就是「台架调好的码 → 上传 → 比赛上电即是它」那条路）
#   "QC"        清空       → 写 {"enabled":false,"value":""} → 回 QS
# ⚠️ 与网页是**同一份文件、同一个真相**: 这里写完, 网页下次读文件就是新状态;
#    网页改了, 这里的查询也如实回。原子替换写（tmp + os.replace）, 两边并发写也
#    只是"后写者胜", 不会写坏文件。
# ⚠️ QU 的值必须三位且每位 1~3（与固件二维码语义同域）, 否则回 ERR 不写 ——
#    非法值若推下去会被固件当「人工预设」标上, 比不推更糟。

def _auto_qr_write(enabled, value):
    """原子写 web_auto_qr.json。value 只接受 "" 或三位 1~3（调用方已验）。"""
    tmp = WEB_AUTO_QR_PATH + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump({"enabled": bool(enabled), "value": value}, f, ensure_ascii=False)
    os.replace(tmp, WEB_AUTO_QR_PATH)


def _send_qr_state():
    """回一帧 "QS,<e>,<值>"（TAG=0x0A; 单片机 ucRpiQrAutoReply 只取开关位 e）。"""
    enabled, v = _auto_qr_read_state()
    val = v if (len(v) == 3 and all(c in "123" for c in v)) else "---"
    _send_cfg_reply("QS,%d,%s" % (1 if enabled else 0, val))


def _dispatch_qr_auto(data):
    """QA/QE/QU/QC 的形状拆分与分发（第二字段是数字/开关, _split_payload 管不了）。

    刻意**不复用** _split_payload: 那张表的第二字段语义是毫秒/次数/亮度,
    而 "QE,0" 的 0 是开关位。形状不认识响一声 WARN 并回 ERR —— 不静默忽略。
    """
    if data == b"QA":
        _send_qr_state()
        return
    if data == b"QC":
        _auto_qr_write(False, "")
        print("[INFO] 单片机清空二维码: 自动下发已关 + 值已清")
        _send_qr_state()
        return
    if data.startswith(b"QE,"):
        on = data[3:].decode("ascii", "replace").strip()
        if on not in ("0", "1"):
            _send_cfg_reply("ERR,QE,用法: QE,<0|1>")
            return
        _enabled, v = _auto_qr_read_state()
        _auto_qr_write(on == "1", v)
        print(f"[INFO] 单片机切换自动下发二维码: {'开' if on == '1' else '关'}（值不变: {v!r}）")
        _send_qr_state()
        return
    if data.startswith(b"QU,"):
        v = data[3:].decode("ascii", "replace").strip()
        if len(v) != 3 or any(c not in "123" for c in v):
            _send_cfg_reply("ERR,QU,值须为三位且每位1~3")
            print(f"[WARN] 单片机上传二维码被拒: {v!r}")
            return
        _auto_qr_write(True, v)
        print(f"[INFO] 单片机上传二维码: {v}（已启用, 中途改值监听会立刻推 QP）")
        _send_qr_state()
        return
    print(f"[WARN] 二维码指令形状不认识: {data!r}")
    _send_cfg_reply("ERR,形状不认识: QA / QE,<0|1> / QU,<ddd> / QC")


# ---- OLED「重启main / 重启服务 / 清屏Log」（命令 "RM"/"RS"/"CL"，2026-10-18 加）----
# 三个都是**只收不回**的单帧指令（发起方是 OLED 菜单按键 = 已经过一次刻意操作,
# 不再要第二道确认; 发完之后的效果各自看日志/链路状态）。

def _cmd_pi_main_restart():
    """命令 'RM'（OLED「重启main」）: 重启主程序。

    与 "r" 走**同一条** serial_comm.restart_program（关串口 → 等 20s → 原地
    os.execl, 起来第一件事是发 SYNC_HS_1 重新握手）, 但**不走 "r" 的双发确认门**:
    那道门防的是文本通道上一敲一个准的误触; OLED 菜单按到这一下要先翻页、选中、
    再按 MAIN, 本身就是刻意操作。
    ⚠️ 收尾与 "r" 完全一致: 停巡线/靶标流/爆爆桶流 + 交还相机租约 + 收 CSI, 别把相机带进下次运行。
    """
    _cmd_line_stop()
    _cmd_aim_stop()
    _cmd_bucket_stop()
    camera_lease.LEASE.release_now()
    camera_picam.PICAM.release_now()
    print("[INFO] 单片机请求重启主程序（RM）")
    serial_comm.restart_program()


def _cmd_pi_svc_restart():
    """命令 'RS'（OLED「重启服务」）: systemctl restart gks.service。

    本进程就是 gks.service 的一员 —— 这条指令本质是「约好时间自杀」: 起一个
    脱离会话的 systemctl 进程后**立刻返回**（不等它）, 让本帧发完、日志落盘,
    systemctl 随后 SIGTERM 过来走 main.py 的优雅退出。
    ⚠️ 重启期间串口断几分钟（服务停止 → 启动 → 重新握手）, 单片机第 1 页 RPI 行
    变 Waiting/Disconnected 属预期, 回来自动置 OK。
    ⚠️ systemctl restart 对**停止状态**的服务 = 直接启动, 所以这一下同时就是
    「开始服务」（服务被手动 stop 后串口没人听, 那种情况这条指令到不了 ——
    只能上 Pi 手工 systemctl start, 或靠 Restart=always 自愈）。
    ⚠️ sudo 依赖 xixi 用户的 NOPASSWD（与控制台/调参那些 sudo 同一条纪律）。
    """
    print("[INFO] 单片机请求重启 gks 服务（RS）")
    subprocess.Popen(["sudo", "systemctl", "restart", "gks.service"],
                     start_new_session=True)


def _cmd_pi_log_clear():
    """命令 'CL'（OLED「清屏Log」）: 清屏日志。

    走 **GKS_Web 清屏按钮同一条通道**（logstore.request_clear 写标志文件 →
    本进程的 _log_clear_watchdog 轮询到后 truncate 文件 + 清内存环）—— 刻意
    **不**直接摸日志文件: 句柄/内存环/归档序号三处一起动才是"清屏", 单独截断
    文件会让内存环里的旧日志下次又写回来。
    """
    import logstore
    if logstore.request_clear():
        print("[INFO] 单片机请求清屏日志（CL）")
    else:
        print("[WARN] 单片机清屏日志失败（标志文件写不进去?）")


# ---- 人质形状识别（命令 '4'）----

def _cmd_hostage(ms=None):
    """命令 '4'：人质形状识别（主摄像头 YOLO，圆柱/圆锥/腰鼓）。

    ms: 单片机随指令发来的「它自己愿意等多久」（"<命令码>,<毫秒>"，2026-10 起固件
    会带）。给了就按它倒推本端预算（本端先答）—— 冷开相机 + 6s 检测最坏可逼近
    单片机 8s 断链阈值, 让对端决定超时就不怕这个; 没给退回 config.HOSTAGE_TIMEOUT。

    结果回传单片机（TYPE_COORD TAG=0x02 二进制帧，见 docs/串口协议.md §6c）：
      成功 → [0x02][01][count][shape/cx/cy]×count（目标按图像从左到右排序）；
      失败/超时 → [0x02][00][count][shape/cx/cy]×count（**带上最好成绩**, 2026-10-26:
              剩 1 只时固件的"单目标拉回"要拿它当路标——把那只平移到理想列再重扫
              （见 app_rpi.c::ucRpiHostageGrabOne）；旧固件按 status=0 照样判失败,
              多余的 targets 无害——长度校验 3+5n 与 LEN 一致, 快照照填）；
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
        _send_hostage_result(build_result_frame(False, result.get("targets") or []))
    # 保存最后一帧（成功为标注帧，失败为最佳成绩帧），供 Web 控制端预览显示
    _save_frame("hostage", result, _cfg("HOSTAGE_LAST_FRAME_PATH"))


def _send_hostage_result(data: bytes):
    """把人质识别结果帧回传单片机（TYPE_COORD TAG=0x02 二进制帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端 ucRpiWaitHostageResult 会超时。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, data)
        _note_result_frame()
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 人质结果回传失败: {e}")


# ---- 色球顺序识别（命令 '5'）----

def _cmd_color(ms=None):
    """命令 '5'：色球顺序识别（主摄像头 HSV，红/绿/蓝）。

    结果回传单片机（TYPE_COORD TAG=0x03 二进制帧，见 docs/串口协议.md §6d）：
      成功 → [0x03][01][count][color/cx/cy]×count + [yaw i16 0.01°][fit_ok u8]
              （LEN=21, 小球对位扩展 2026-09: yaw=三球连线**图像**倾角,
                左高右低为正; fit_ok=0 表示共线残差超限、有球识别可疑）；
      失败/超时 → [0x03][00][00][pull_dir u8]（count=0，让单片机立即解锁，不空烧
              等待超时; pull_dir 0=无信息 1=缺的球左出界 2=右 —— 单片机
              ALIGN_PULLBACK 据此开环倒回, 旧固件无视该字节）；
      被打断 → 不回传（单片机/web 主动放弃，调用方自己知道原因）。

    ms: 单片机随指令发来的「它自己愿意等多久」（"<命令码>,<毫秒>"）。给了就按它
    倒推本端预算（本端先答）; 没给退回 config.COLOR_TIMEOUT。

    ⚠️ 要求小车静止（多帧投票需同一机位）。
    """
    _stop_main_stream()   # 释放 Web 端可能占用的主摄像头推流
    result = run_color_detect(timeout=_budget_from_ms(ms), abort_event=_task_abort)
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
        # 失败帧方向字节（2026-10-26）: 剩两球且中点偏出阈值 → 告诉单片机缺的球
        # 从画面哪侧出界（ALIGN_PULLBACK 开环倒回用）; 无信息传 0（旧固件行为不变）。
        pdir = int(result.get("pull_dir", 0) or 0)
        _send_color_result(build_color_frame(False, [], pull_dir=pdir))
    # 保存最后一帧（成功为标注帧，失败为最佳成绩帧），供 Web 控制端预览显示
    _save_frame("color", result, _cfg("COLOR_LAST_FRAME_PATH"))


def _send_color_result(data: bytes):
    """把色球结果帧回传单片机（TYPE_COORD TAG=0x03 二进制帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端 ucRpiWaitColorResult 会超时。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, data)
        _note_result_frame()
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 色球结果回传失败: {e}")


# ---- 色球区矩形地板识别（命令 'A'）----

def _cmd_rect(ms=None):
    """命令 'A'：色球区**矩形地板**（黑胶带边框）识别，回板心像素坐标。

    结果回传单片机（TYPE_COORD TAG=0x07 二进制帧，见 docs/串口协议.md §6h）：
      成功 → [0x07][01][cx u16 LE][cy u16 LE][flags][spread][保留]（10B）
      失败/超时 → [0x07][00] + 全 0（**让单片机立刻解锁**，不空烧等待超时）
      被打断 → 不回传（单片机/web 主动放弃，调用方自己知道原因）

    ms: 单片机随指令发来的「它自己愿意等多久」（"<命令码>,<毫秒>"）。给了就按它
    倒推本端预算（本端先答）; 没给退回 config.RECT_TIMEOUT。

    单片机侧实测一次约 2~3s（9 帧 × ~180ms + 开摄像头）。⚠️ 要求小车静止。
    ⚠️ 用主摄 → 必须与巡线/靶标流互斥（见 handle() 里那个元组）。
    """
    _stop_main_stream()   # 释放 Web 端可能占用的主摄像头推流
    result = run_rect_detect(timeout=_budget_from_ms(ms), abort_event=_task_abort)
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
    _save_frame("rect", result, _cfg("RECT_LAST_FRAME_PATH"))


def _send_rect_result(data: bytes):
    """把矩形结果帧回传单片机（TYPE_COORD TAG=0x07 二进制帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端 ucRpiWaitRectResult 会超时。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, data)
        _note_result_frame()
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 矩形结果回传失败: {e}")


# ---- 看路（命令 'G'）----

def _cmd_road(ms=None, rev=False):
    """命令 'G'：主摄「看路」—— 量地面黄色区（左边缘 x → 车体纵向；下边缘 y → 车体横向）。
    rev=True 时走**反向看路**（2026-10-31 加）：Pi 侧整帧镜像后量黄区**右边缘**，见
    tasks/_road.py 文件头「反向看路」节与固件 RPI_TASK_ROAD_REV（app_rpi.h, 命令 "M"）。

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
    result = run_road_detect(timeout=_budget_from_ms(ms), abort_event=_task_abort,
                             rev=rev)
    if result.get("success"):
        # ⚠️ 字段是 **dx / dy / edge / area**。改过 run_road_detect 的返回结构就要同步
        #    这里 —— 漏一次的表现是「Pi 日志报成功, 单片机却一直 ROAD fail: wait st=0」
        #    （回来一看是 KeyError, 结果帧根本没发出去, 真踩过）。
        e = result.get("edge") or (0.0, 0.0)
        print(f"[INFO] 看路{'(反向)' if rev else ''}成功: "
              f"Δx={result['dx']:.1f}px 左边缘=({e[0]:.0f},{e[1]:.0f}) "
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
    _save_frame("road", result, _cfg("ROAD_LAST_FRAME_PATH"))


def _send_road_result(data: bytes):
    """把看路结果帧回传单片机（TYPE_COORD TAG=0x08 二进制帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端 ucRpiWaitRoadResult 会超时。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, data)
        _note_result_frame()
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 看路结果回传失败: {e}")


# ---- 反向看路（命令 'M'，2026-10-31 加）----

def _cmd_road_rev(ms=None):
    """命令 'M'：**反向看路** —— 车从路的另一边接近黄区（返程段机位）时用。

    与 'G' 完全同一套链路与结果帧（0x08, 12B），唯一差别 = run_road_detect(rev=True)：
    Pi 侧整帧水平镜像后走同一套检测管线 —— Δx 变成量黄区**右边缘**（镜像系左边缘）、
    ROI 自动左右对调、灰世界参考块换到镜像系右下（真实左下地面）。详见 tasks/_road.py
    文件头「反向看路」节。
    ⚠️ 数字全是**镜像系坐标** —— 单片机的 road_dx_ideal / road_dy_ideal 要按反向机位
       另标（STEP_ROAD_SET_IDEAL 在路线里覆盖，或蓝牙现改）；比例/死区/方向符号
       （road_scale/road_sign 等）与正向同套。
    ⚠️ 阻塞，"M" 必须同时在 _MS_COMMANDS、互斥元组与 _FAIL_FRAME_SENDERS 里（与 "G" 同款）。
    ⚠️ 对齐固件 RPI_TASK_ROAD_REV = 30（Application/app_rpi.h 的 emRpiTaskTdf + app_rpi.c
       的 s_apcTaskCmd，两边同注释互指）；老 Pi 收到 "M" 会打「未知指令」拒收 —— 先推本
       文件再烧新固件。
    """
    _cmd_road(ms, rev=True)


# ---- 看路校准（命令 'H'）----

def _cmd_road_yaw(ms=None):
    """命令 'H'：主摄「看路校准」—— 拟合**黄区直边的图像方向角**（陀螺漂移的视觉基准）。

    物理: 黄区固定在场上、相机固定在车上 ⇒ 直边在图像里的方向只随**车航向**变
    （直线方向对平移不敏感）。车停在基准停车位标一次 ideal（单片机的 roadyaw_ideal），
    之后每次到点 err = (ang − ideal)×方向 就是「陀螺漂了多少」的现场读数。

    ⚠️⚠️ **只测不改**: 本命令的结果只进单片机遥测（roadyaw_*）与调试串，
    **不动车、不写航向** —— 接闭环前先并排验证（别硬置 yaw, 会踢到咬航向的闭环）。

    回传单片机（TYPE_COORD TAG=0x09 二进制帧，见 docs/串口协议.md §6j）：
      成功 → [0x09][01][ang i16 /100][cols][rms /10][spread /100][flags]（8B）
      失败/超时 → [0x09][00] + 全 0（**让单片机立刻解锁**，不空烧等待超时）
      被打断 → 不回传（单片机/web 主动放弃，调用方自己知道原因）

    ms: 单片机随指令发来的**它自己愿意等多久**（"<命令码>,<毫秒>"），
        给了就按它倒推本端预算（本端先答），没给就退回 config.ROADYAW_TIMEOUT。
        ⚠️ 这一步是**阻塞**的，所以 "H" 必须同时在 _MS_COMMANDS 与互斥元组里。

    单片机侧实测一次约 1~2s（≤9 帧 + 开摄像头）。⚠️ 要求小车静止。
    ⚠️ 用主摄 → 必须与巡线/靶标流互斥（见 handle() 里那个元组）。
    ⚠️ 字段是 **angle / cols / rms / spread / flags** —— 改 run_road_yaw_detect 的
       返回结构就要同步这里（漏一次 = KeyError, 结果帧根本没发出去, 同 _cmd_road 的坑）。
    """
    _stop_main_stream()   # 释放 Web 端可能占用的主摄像头推流
    result = run_road_yaw_detect(timeout=_budget_from_ms(ms), abort_event=_task_abort)
    if result.get("success"):
        print(f"[INFO] 看路校准成功: ang={result['angle']:.2f}° cols={result.get('cols', 0)} "
              f"rms={result.get('rms', 0.0):.1f}px 直段={result.get('span', 0.0):.0f}px "
              f"帧{result.get('votes')}/剔{result.get('dropped')} "
              f"散布{result.get('spread', 0.0):.2f}° "
              f"fl=0x{result.get('flags', 0):02X}")
        _send_road_yaw_result(build_road_yaw_frame(True, result["angle"],
                                                   result.get("cols", 0),
                                                   result.get("rms", 0.0),
                                                   result.get("spread", 0.0),
                                                   result.get("flags", 0)))
    elif result.get("aborted"):
        print("[INFO] 看路校准任务被打断")
    else:
        print(f"[WARN] 看路校准失败: {result['reason']}")
        _send_road_yaw_result(build_road_yaw_frame(False))
    # 保存最后一帧（成功为标注帧），供排障（Web 控制端另有 /road_yaw 就地预览）
    _save_frame("road_yaw", result, _cfg("ROADYAW_LAST_FRAME_PATH"))


def _send_road_yaw_result(data: bytes):
    """把看路校准结果帧回传单片机（TYPE_COORD TAG=0x09 二进制帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端 ucRpiWaitRoadYawResult 会超时。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, data)
        _note_result_frame()
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 看路校准结果回传失败: {e}")


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


def _cmd_warm_main(uses=None, use_pi=None):
    """命令 'C'：预热识别主相机（**只收不回**，没有结果帧）。

    同上，只是换成"识别用的那台主相机"，给随后的人质/色球/靶标识别省掉开相机时间。
    `"C,<uses>,1"`（2026-10-16 加）的第二段 = is_use_picam → 预热的是**树莓派 CSI**
    那颗（camera_picam.PICAM）而不是 /dev/camera_main —— 与随后那次带 pi 标志的识别
    配对使用（否则预热了 A、识别用 B, 那 2s 白省）。
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

    ⚠️ pi 那一路（use_pi）走的是**另一个持有器**（camera_picam.PICAM，libcamera 子进程）:
       它与 USB 相机租约互不干扰 ⇒ **两台可以同时热着**（换相机不再付 2s）。
    ⚠️⚠️ **靶标流运行期间预热主摄 = 跳过**（2026-10-28 加防呆）: LEASE.warm 发现要换档
       会先 release_now() 把正被流线程使用的设备抽走、再重开又撞半释放的设备 → 连串
       EBUSY（01:15 现场: 'W' 撞 '7' 流, 12 条 [ERROR] 无法打开）。跳过的代价只是
       下一个主摄任务现开现扫慢 ~2s —— 排表把预热挪出流的存活窗口才是正解
       （docs/vision-tasks.md 相机预热节那条警告的代码化）。pi 分支不受影响照常热。
    """
    if use_pi:
        camera_picam.PICAM.warm(uses)
    else:
        if is_aim_running():
            print("[WARN] 主摄预热跳过: 靶标流正在运行（warm 换档会掐流, "
                  "2026-10-28 现场踩过; 下一个主摄任务将现开现扫 ~2s）")
            return
        camera_lease.LEASE.warm(is_qr=False, uses=uses)


def _dispatch_warm_w(data):
    """命令 "W"：带**相机 + 次数 + 档位**三项的预热（2026-10-23 定稿, 只收不回）。

        形状: `"W,<cam>,<uses>,<prof>"`（cam/prof 见下面的表；三段都可省略，缺省=0）

    为什么单独一个命令头（而不是往 "B"/"C" 加字段）:
      · "C,<uses>,<n2>" 的 n2 已经是 **is_use_picam**，再塞档位码就是两个语义挤一格
        —— 旧 Pi 会把"看路档预热"当 CSI 预热，静默开错相机；
      · 本形状对旧 Pi 是**形状不认识 → 响亮拒收 + 随后退回现开现扫**（慢 ~2s，不坏）。
      · 固件只在**用户显式选了非默认档**时才发它: 默认档仍走老的 "B"/"C"，
        老路线/老参数在线上一个字节都没变。

    ⚠️ cam 与 prof 是**正交**两轴: cam 决定开哪台设备，prof 决定用哪套曝光/白平衡。
       所以"看路档"不是一种相机 —— 它是主摄 + road 档，别在 cam 里造组合项。
    ⚠️ prof 只对 **USB 那台**有意义（主摄 / 扫码副摄）；WARM_CAM_PI 忽略它并留一条
       WARN（CSI 的白平衡在 camera_picam 自己的 config 里: PICAM_AWB_MODE/GAINS）。
    ⚠️ 档位码越界 → 退回**默认档**（不是不预热）: 宁可退化成老行为，也别因为一个手滑
       的数字白多花 ~1.3s 开相机。
    """
    parts = data.split(b",")
    vals = []
    for p in parts[1:]:
        if not p.isdigit():
            print(f"[WARN] 预热指令形状不认识: {data!r}（要 \"W,<cam>,<uses>,<plan>\"）")
            return
        vals.append(int(p))
    if len(vals) > 3:
        print(f"[WARN] 预热指令参数过多: {data!r}")
        return
    vals += [0] * (3 - len(vals))
    cam, uses, plan = vals

    if cam not in _WARM_CAMS:
        print(f"[WARN] 预热相机码 {cam!r} 不认识 → 不预热（宁可不做也不猜一台去开）")
        return
    profiles = _unpack_profiles(plan, max(1, min(int(uses or 1), 10)))

    is_pi = cam in (2, 3)                          # 2 = 只 pi, 3 = 主摄+pi
    is_qr = (cam == 1)
    if is_pi:
        if any(profiles):
            print("[WARN] 档位只对 USB 相机有意义 → pi(CSI) 忽略本次档位")
        camera_picam.PICAM.warm(uses)
        if cam == 2:
            return                                 # 只要 pi
    if (not is_qr) and is_aim_running():
        # 同 _cmd_warm_main 的防呆（2026-10-28）: 主摄预热会掐正在跑的靶标流;
        # cam=3 时 pi 那颗已在上面热过, 这里只跳过主摄那条 —— 正是"主摄+pi"里
        # 主摄那条单独让路, 下一任务现开现扫 ~2s。
        print("[WARN] 主摄预热跳过: 靶标流正在运行（warm 换档会掐流, "
              "2026-10-28 现场踩过; 下一个主摄任务将现开现扫 ~2s）")
        return
    camera_lease.LEASE.warm(is_qr=is_qr, uses=uses, profiles=profiles)


def _unpack_profiles(plan, uses):
    """把"每次使用档位"的打包整数拆成名字列表（长度 uses, 元素 None = 默认档）。

    编码（固件 app_rpi.h::emWarmProfTdf 那一节写着同一套）:
        **每次使用占 2 bit**, 第 i 次在 bit 2i 上；值 0=默认 / 1=看路 / 2=人质。
        例: uses=3, [默认, 看路, 人质] → (0<<0)|(1<<2)|(2<<4) = 4 + 32 = 36。
    ⚠️ 单档老写法天然兼容: plan=1 就是"第 1 次看路、其余默认"。
    ⚠️ 超过 uses 的位忽略（固件按同一 uses 打包, 不会多出来）; 值 3 是保留位 → 默认。
    """
    out = []
    for i in range(uses):
        code = (int(plan) >> (2 * i)) & 0x3
        out.append(_WARM_PROFILES.get(code))       # 0/3 → None（默认档）
    return out


# W 指令的相机码 —— 必须与固件 emWarmCamTdf 的数值一致。
_WARM_CAMS = {0: "主摄", 1: "扫码副摄", 2: "pi(CSI)", 3: "主摄+pi"}

# 档位码 → camera.py::_CAMERA_PROFILES 的键。**只登记"非默认"的那两档**:
#   0（默认）= 主摄 main / 扫码副摄 qr —— 二维码、色球、靶标、主摄扫码共用，不需要单列;
#   越界/不认识 → 调用方退回默认档。
# ⚠️ 数字必须与固件 emWarmProfTdf 一致（1=road, 2=hostage）。
_WARM_PROFILES = {1: "road", 2: "hostage"}


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
    if is_bucket_running():
        _cmd_bucket_stop()          # 爆爆桶流同收（2026-10-24 加）
    print("[INFO] 已取消当前任务（x）")


# ---- 四路相机健康检查（命令 'Z'，2026-11 加）----
#
# 单片机握手积木勾了「健康检查」时发 "Z,<ms>"：本端依次确认
#   M  = 主摄（/dev/camera_main, USB）
#   Q  = 扫码副摄（/dev/camera_Qr, USB）
#   Pi = 树莓派 CSI（camera_picam）
#   MV = OpenMV（openmv_serial）
# 都**能出帧**才算通过，最后回一帧 TYPE_COORD TAG=0x0D：DATA=[0x0D][M][Q][Pi][MV]
# （1=通过 0=失败），见 docs/串口协议.md §6o / 固件 app_rpi.h::RPI_HEALTH_TAG。
#
# ⚠️ 这是**阻塞**命令（在 main.py 主循环里跑）：期间心跳不发，链路可能在 8s 后闪一次
#    Disconnected —— 结果帧一到就自愈，属预期（同人质/看路那几条阻塞命令）。
# ⚠️ **查完必须解除占用**：USB 租约的 release_camera() 只是"归还"（保持打开等复用），
#    所以每台查完都额外 LEASE.release_now()；CSI 的 probe() 自己会收 rpicam-vid；
#    OpenMV 发 'G' 后无论成败都补 'S'。不这么做 = 体检把四台相机全留热，
#    而接下来的路线可能要用完全不同的档位/相机。
# ⚠️ 每台相机独立 try/except：一台异常不能把另外三台的结果一起吞掉（用户要看到
#    哪一项是 ×）。整条命令异常时 handle() 的兜底会回四项全 0（_FAIL_FRAME_SENDERS）。
# ⚠️ 时间预算由单片机给（"Z,<ms>"）；本端减 CMD_MARGIN_S 当总预算，四台各自再限一段，
#    **谁先放弃由单片机说了算**（同其它阻塞类指令）。

_HEALTH_DEFAULT_BUDGET_S = 12.0   # 裸 "Z"（台架手动测试）时的总预算


def _health_keepalive():
    """体检期间补一帧心跳（TYPE_HELLO）。

    main 主循环被本命令占着、正常心跳发不出去；四台相机串行冷开可能超过
    单片机的 RPI_DISCONNECT_TIMEOUT_MS（8s），会让 OLED 闪一下 RPI:Disconnected。
    补一帧 HELLO 就能把「Pi 还活着」这件事维持住（固件对 HELLO 只刷新存活时刻）。
    断线就吞掉 —— 后面的结果帧发送会如实报错。
    """
    try:
        serial_comm.send_frame(TYPE_HELLO, b"")
    except (SerialDisconnected, OSError):
        pass


def _send_health_result(m, q, pi, mv):
    """回传四路相机健康检查结果（COORD TAG=0x0D, 5B）。断线只打日志。"""
    bits = (1 if m else 0, 1 if q else 0, 1 if pi else 0, 1 if mv else 0)
    try:
        serial_comm.send_frame(TYPE_COORD, bytes((TAG_HEALTH,) + bits))
        print("[INFO] 已回传健康检查: M=%s Q=%s Pi=%s MV=%s"
              % tuple("√" if b else "×" for b in bits))
    except (SerialDisconnected, OSError) as e:
        print(f"[WARN] 健康检查结果回传失败: {e}")


def _health_usb_camera(is_qr, name, deadline):
    """开一台 USB 相机并读到一帧。True=能出帧。查完立刻释放占用。"""
    if time.time() >= deadline:
        print(f"[WARN] 健康检查: {name} 预算已用完，跳过")
        return False

    cap = camera_lease.acquire_camera(
        is_qr=is_qr, profile=("qr" if is_qr else "main"))
    if cap is None:
        print(f"[WARN] 健康检查: {name} 打开失败")
        return False

    try:
        while time.time() < deadline:
            try:
                ok, frame = cap.read()
            except Exception as e:                     # noqa: BLE001
                print(f"[WARN] 健康检查: {name} 读帧异常: {e}")
                ok, frame = False, None
            if ok and frame is not None:
                print(f"[INFO] 健康检查: {name} 出帧正常")
                return True
            time.sleep(0.05)
        print(f"[WARN] 健康检查: {name} 超时未出帧")
        return False
    finally:
        try:
            camera_lease.release_camera(cap)
            # ⚠️ 必须是 release_now（不是 release_camera）：后者对租约句柄只是"归还,
            #    保持打开"，体检要的是**解除占用** —— 否则这台相机会一直热着。
            camera_lease.LEASE.release_now()
        except Exception as e:                         # noqa: BLE001
            print(f"[WARN] 健康检查: {name} 释放失败: {e}")


def _health_pi_camera(deadline):
    """CSI（libcamera）自检：真的抓一帧（camera_picam.probe 内部会收识别流/遗留进程）。"""
    remain = deadline - time.time()
    if remain <= 0:
        print("[WARN] 健康检查: pi摄像头 预算已用完，跳过")
        return False
    ok, info = camera_picam.probe(timeout=max(1.0, min(8.0, remain)))
    print("[INFO] 健康检查: pi摄像头 %s（%s）" % ("正常" if ok else "失败", info))
    return bool(ok)


def _health_openmv(deadline):
    """OpenMV：发 'G' 等一帧新鲜数据，随后**无论成败都发 'S'**（恢复空闲、不占串口流）。"""
    if time.time() >= deadline:
        print("[WARN] 健康检查: OpenMV 预算已用完，跳过")
        return False

    before = (openmv_reader.get_latest().get("timestamp") or 0.0)
    if not openmv_reader.send_command(b'G'):
        print("[WARN] 健康检查: OpenMV 串口未连接")
        return False

    try:
        while time.time() < deadline:
            st = openmv_reader.get_latest()
            if st.get("connected") and ((st.get("timestamp") or 0.0) > before):
                print("[INFO] 健康检查: OpenMV 出帧正常")
                return True
            time.sleep(0.05)
        print("[WARN] 健康检查: OpenMV 超时未出帧")
        return False
    finally:
        openmv_reader.send_command(b'S')     # 全停：体检不把相机留在"连续发流"状态


def _cmd_health(ms=None):
    """命令 'Z'：四路相机健康检查（阻塞；查完解除全部占用）→ 回 COORD TAG=0x0D。"""
    margin = getattr(config, "CMD_MARGIN_S", 1.5)
    max_s = getattr(config, "CMD_MAX_TIMEOUT_S", _HEALTH_DEFAULT_BUDGET_S)
    budget = (float(ms) / 1000.0 - margin) if ms else _HEALTH_DEFAULT_BUDGET_S
    budget = max(1.0, min(budget, max_s))
    t0 = time.time()
    deadline = t0 + budget
    print(f"[INFO] 健康检查开始（预算 {budget:.1f}s）")

    m = q = pi = mv = 0
    aborted = False
    try:
        # USB 两台共用单槽租约，必须**串行**（acquire 第二台会先收掉第一台）。
        # 每台最多 3.5s：冷开 ~1.5~2s + 首帧；到点还没出帧就判 ×，别吃光后面的预算。
        # 每台各自 try/except：一台异常不影响另外三台的结论（用户要看具体哪项 ×）。
        if not _task_abort.is_set():
            try:
                m = 1 if _health_usb_camera(False, "主摄",
                                            min(deadline, time.time() + 3.5)) else 0
            except Exception as e:                     # noqa: BLE001
                print(f"[WARN] 健康检查: 主摄异常: {e}")
        _health_keepalive()
        if not _task_abort.is_set():
            try:
                q = 1 if _health_usb_camera(True, "扫码副摄",
                                            min(deadline, time.time() + 3.5)) else 0
            except Exception as e:                     # noqa: BLE001
                print(f"[WARN] 健康检查: 扫码副摄异常: {e}")
        _health_keepalive()
        if not _task_abort.is_set():
            try:
                pi = 1 if _health_pi_camera(deadline) else 0
            except Exception as e:                     # noqa: BLE001
                print(f"[WARN] 健康检查: pi摄像头异常: {e}")
        _health_keepalive()
        if not _task_abort.is_set():
            try:
                mv = 1 if _health_openmv(min(deadline, time.time() + 2.5)) else 0
            except Exception as e:                     # noqa: BLE001
                print(f"[WARN] 健康检查: OpenMV异常: {e}")
        aborted = _task_abort.is_set()
    finally:
        # 兜底解除占用：即使某项抛异常/中途取消，也不把相机留在手里。
        try:
            camera_lease.LEASE.release_now()
        except Exception as e:                         # noqa: BLE001
            print(f"[WARN] 健康检查: 释放 USB 租约失败: {e}")
        try:
            camera_picam.PICAM.release_now()
        except Exception as e:                         # noqa: BLE001
            print(f"[WARN] 健康检查: 释放 CSI 失败: {e}")
        try:
            openmv_reader.send_command(b'S')
        except Exception:                              # noqa: BLE001
            pass

    if aborted:
        print("[WARN] 健康检查被取消（x），按已完成的项回传")
    _send_health_result(m, q, pi, mv)
    print("[INFO] 健康检查完成（用时 %.1fs）" % (time.time() - t0))


# ---- 靶标识别 + 靶心定位（命令 '6'）----

def _cmd_target(ms=None, use_pi=None):
    """命令 '6'：三色靶盘识别 + 靶心定位（**优先 pi(CSI)摄像头, 不成退回主摄**）。

    ms: 单片机随指令发来的「它自己愿意等多久」（同 _cmd_hostage, 2026-10 起固件会带）;
    没给退回 config.TARGET_TIMEOUT。
    use_pi: `"6,<ms>,1"` 的第二段（2026-10-16 加）= 优先用树莓派 CSI 那颗看靶
    （它固定在底盘上、几何稳; 主摄在机械臂上会漂）。0/省略 = 只用主摄。
    ⚠️ **实际**是哪台测的看返回的 `cam_is_pi`, 不是看这个请求 —— CSI 可能没成而退回主摄,
       而单片机是靠回帧 status 的 bit7 选瞄准列的（它两把尺各标各的）。

    结果回传单片机（TYPE_COORD TAG=0x04 二进制帧，见 docs/串口协议.md §6e）：
      成功 → [0x04][status][count][color/cx/cy/r]×count（按图像从左到右排序）;
             status: bit0=1 成功, **bit7=本次来自 pi/CSI**
      失败/超时 → [0x04][00][00]（count=0，让单片机立即解锁，不空烧等待超时）；
      被打断 → 不回传（单片机/web 主动放弃，调用方自己知道原因）。

    ⚠️ 回传的是**三颗**，哪一颗是目标由**单片机**按二维码第二位挑 —— Pi 侧不掺和
    任务语义（与色球那边完全一致）。
    ⚠️ 要求小车/机械臂静止（多帧投票需同一机位）。
    """
    _stop_main_stream()   # 释放 Web 端可能占用的主摄推流（退回主摄那一路要用它）
    result = run_target_detect(timeout=_budget_from_ms(ms), abort_event=_task_abort,
                               use_pi=bool(use_pi))
    if result.get("success"):
        targets = result["targets"]
        print(f"[INFO] 靶标识别成功[{('pi/CSI' if result.get('cam_is_pi') else '主摄')}]"
              f"（从左到右）: "
              + " ".join(f"{t['color_name']}({int(t['cx'])},{int(t['cy'])})"
                         f"r{int(t['r'])}" for t in targets))
        _send_target_result(build_target_frame(True, targets, result.get("cam_is_pi", False)))
    elif result.get("aborted"):
        print("[INFO] 靶标识别任务被打断")
    else:
        print(f"[WARN] 靶标识别失败: {result['reason']}")
        _send_target_result(build_target_frame(False, []))
    # 保存最后一帧（成功为标注帧，失败为最佳成绩帧），供 Web 控制端预览显示
    _save_frame("target", result, _cfg("TARGET_LAST_FRAME_PATH"))


def _send_target_result(data: bytes):
    """把靶标结果帧回传单片机（TYPE_COORD TAG=0x04 二进制帧）。

    串口断开时静默丢弃：结果帧丢失不致命，单片机端等结果的函数会超时。
    """
    try:
        serial_comm.send_frame(TYPE_COORD, data)
        _note_result_frame()
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


def _aim_worker(use_pi=False):
    """靶标流线程体：跑 run_target_stream，结束后清理线程标记。"""
    global _aim_thread
    try:
        run_target_stream(abort_event=_aim_abort, use_pi=bool(use_pi))
    except Exception as e:
        import traceback
        print(f"[ERROR] 靶标流线程异常: {type(e).__name__}: {e}")
        traceback.print_exc()
    finally:
        _aim_abort.clear()
        _aim_thread = None


def _cmd_aim_start(ms=None, use_pi=None):
    """命令 '7'：开始靶标流式跟踪（独立线程，回传 0x05 流帧）。

    use_pi: `"7,0,1"`（或简写 `"7,1"`）的 is_use_picam（2026-10-16 加）= 优先用 CSI 看靶,
    起不来/中途掉线由 run_target_stream **整条改用主摄重跑**（回帧的 cam 位跟着变）。
    ms 段不用（"7" 没有时限语义, 统一三字段形状里它是占位 0）。

    ⚠ 起流前必须先把**其它会占主摄的东西**让开，两件事：
      ① `_cmd_line_stop()` —— 巡线与靶标流都走 COORD 通道，两条流同时在跑会帧交错
         （串口发送无跨线程锁，见本文件顶部与 _line.py 的说明）；
      ② `_stop_main_stream()` —— Web 端主摄推流占着设备，不释放的话 open_camera 会失败。
    """
    global _aim_thread
    _cmd_line_stop()                # 两条流互斥：主摄 + COORD 通道
    _cmd_bucket_stop()              # 爆爆桶流同走 COORD 通道（2026-10-24 加）

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
        # ⚠️ use_pi 必须**过给线程**: 重复的 "7" 会被上面的 _aim_lock 忽略（保留老流的老值）,
        #    靠模块全局存它就会串味（"忽略重复启动"那条路径不执行任何赋值）。
        _aim_thread = threading.Thread(target=_aim_worker, args=(bool(use_pi),),
                                       name="target-stream", daemon=True)
        _aim_thread.start()
        print(f"[INFO] 靶标流跟踪已启动（{'优先pi/CSI' if use_pi else '主摄'}）")
    finally:
        _aim_lock.release()


def _cmd_aim_stop():
    """命令 '8'：停止靶标流式跟踪。未在跑时打 WARN 忽略（幂等）。"""
    if _aim_thread is None or not _aim_thread.is_alive():
        print("[WARN] 靶标流未在运行，忽略停止指令")
        return
    _aim_abort.set()
    print("[INFO] 已发送靶标流停止信号")


# ---- 爆爆桶坐标流（命令 'J' 起 / 'K' 停，2026-10-24 加）----
# 独立阶段任务：OpenMV v3.4 检测黑色爆爆桶 → "B,<valid>,<cx>,<cy>,<w>,<h>" 文本帧 →
# 本线程纯转发成 0x0B 二进制流帧（~20Hz，无桶也照发，单片机靠帧计数判活性、
# status bit0 判有效性 —— 同 0x05 靶标流语义，见 tasks/_bucket.py）。
# 与巡线流/靶标流**互斥**（同走 COORD 通道；对位时车不巡线，用户 2026-10-24 定案）。
# 它只消费 OpenMV 的串口缓存、不占主摄/CSI —— 所以互斥比靶标流少一条
# _stop_main_stream（Web 推流不用让）。
_bucket_thread = None
_bucket_abort = threading.Event()
_bucket_lock = threading.Lock()
_BUCKET_JOIN_S = 3.0            # 起流时等上一个线程收尾的上限（同 _AIM_JOIN_S 的理由）


def _bucket_worker():
    """爆爆桶流线程体：跑 run_bucket_stream，结束后清理线程标记并兜底停相机。"""
    global _bucket_thread
    try:
        run_bucket_stream(abort_event=_bucket_abort)
    except Exception as e:
        import traceback
        print(f"[ERROR] 爆爆桶流线程异常: {type(e).__name__}: {e}")
        traceback.print_exc()
    finally:
        _bucket_abort.clear()
        _bucket_thread = None
        openmv_reader.send_command(b'b')   # 流结束(含超时/断线/异常)兜底停相机 B 帧


def _cmd_bucket_start(ms=None):
    """命令 'J'：开始爆爆桶坐标转发（独立线程，0x0B 流帧 ~20Hz）。

    ms = 单片机愿意等多久（"J,15000"）→ _budget_from_ms 换算成预算（同阻塞类
    指令口径：谁先放弃只由单片机说了算）。裸 "J" → None → 用 config.BUCKET_MAX_S。

    ⚠ 起流前必须把**其它 COORD 流**让开（同 _cmd_aim_start 的互斥①）：
      `_cmd_line_stop()` + `_cmd_aim_stop()` —— 三条流同走 COORD 通道，
      两条同时在跑会帧交错（串口发送互斥在帧内，跨流没有）。
    """
    global _bucket_thread
    _cmd_line_stop()                # 三条流互斥：COORD 通道
    _cmd_aim_stop()

    if not _bucket_lock.acquire(blocking=False):
        print("[WARN] 爆爆桶流已在运行，忽略重复的启动指令")
        return
    try:
        # 上一个线程"还在收尾"与"正在正常跑流"分开处理（同 _cmd_aim_start 的注释）
        old = _bucket_thread
        if old is not None and old.is_alive():
            if not _bucket_abort.is_set():
                print("[WARN] 爆爆桶流已在运行，忽略重复的启动指令")
                return
            print(f"[INFO] 爆爆桶流：上一个线程还在收尾，等它退出（≤{_BUCKET_JOIN_S}s）...")
            old.join(timeout=_BUCKET_JOIN_S)
            if old.is_alive():
                print(f"[WARN] 爆爆桶流：上一个线程 {_BUCKET_JOIN_S}s 内没退出，忽略本次启动")
                return
        # 触发 OpenMV 开始发 B 帧（未连接时仅告警：线程仍启动，转发 valid=0 帧）
        if not openmv_reader.send_command(b'B'):
            print("[WARN] OpenMV 未连接，无法触发爆爆桶流（将转发 valid=0 帧）")
        _bucket_abort.clear()
        _bucket_thread = threading.Thread(target=_bucket_worker, name="bucket-stream",
                                          daemon=True)
        _bucket_thread.start()
        print(f"[INFO] 爆爆桶流已启动（预算 {_budget_from_ms(ms) or '默认'}s）")
    finally:
        _bucket_lock.release()


def _cmd_bucket_stop():
    """命令 'K'：停止爆爆桶坐标转发。未在跑时打 WARN 忽略（幂等）。"""
    if _bucket_thread is None or not _bucket_thread.is_alive():
        print("[WARN] 爆爆桶流未在运行，忽略停止指令")
        return
    _bucket_abort.set()
    openmv_reader.send_command(b'b')   # 立即触发相机停止发 B 帧（finally 兜底幂等）
    print("[INFO] 已发送爆爆桶流停止信号")


def is_bucket_running():
    """爆爆桶流线程是否在跑（互斥判断/Web 状态用）。"""
    return _bucket_thread is not None and _bucket_thread.is_alive()


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


def _photo_timeout(gen):
    """超时兜底：到点还没等到 OpenMV 回执 → 回一条"失败"。

    ⚠️ 持 _photo_lock 判 _photo_timer: 回执刚到、正要 cancel 我们的瞬间我们已触发 ——
       cancel 对已触发的 Timer 无效, 不判的话两边各发一条 ACK, 单片机闪显哪条全凭运气。
    ⚠️⚠️ **必须按代次号（gen）判「我还是当前那次请求吗」**（2026-10 加）。
    只有"timer is None"这一条判据挡不住「换了代」: 新版 _cmd_photo 会 cancel 旧定时器
    并装一个新的, 而**旧定时器可能已经触发**、正卡在锁上 —— 它一进来看到的是**新**的
    _photo_timer（非 None）, 于是把新的清掉 + 替**新**那次请求发一条"失败"
    （真回执随后到达只会打「迟到, 丢弃」）→ 第二次拍照永远显示失败。
    """
    global _photo_timer
    with _photo_lock:
        if gen != _photo_gen:
            return   # 已被新的拍照请求顶替: 那一次的时间由新定时器管, 我不许动
        if _photo_timer is None:
            return   # 回执已经处理过了 —— 什么都不做
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
_photo_gen = 0          # 每次 '9' 加一; 定时器凭它认自己还是不是"当前那次请求"（见 _photo_timeout）
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
    # ⚠️ **装定时器这段必须在 _photo_lock 里**（2026-10 修）。`_photo_timer` 有三个
    #    线程在动: 主循环(这里)、Timer 线程(_photo_timeout)、OpenMV 读线程
    #    (_on_photo_result)。后两个都持 _photo_lock（该锁的注释就是这么写的),
    #    只有这里原来在锁外 —— 连按两次 '9' 时: 第一个定时器正跑在 _photo_timeout 里
    #    （已读到非 None、还没写 None）, 主循环装上第二个, 随后 _photo_timeout 把全局
    #    置 None → **第二个定时器被抹掉**; 它到点后见 timer is None 直接 return,
    #    而那次请求的回执到了也只会打「迟到, 丢弃」→ 第二次拍照**没有 0x06 回执**,
    #    OLED 不闪提示, 操作员以为没按上（"9" 本身没有别的反馈）。
    global _photo_gen
    with _photo_lock:
        if _photo_timer is not None:
            _photo_timer.cancel()
        _photo_gen += 1                      # 新代次: 旧的（可能已触发的）定时器据此退场
        _photo_timer = threading.Timer(PHOTO_ACK_TIMEOUT_S, _photo_timeout,
                                       args=(_photo_gen,))
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


# ---- 参数直取直设（命令 "P" 族，2026-10 加，单片机现场调试用）----
# 形状（TYPE_CMD 的 DATA，ASCII）:
#   "P?,<NAME>"     取一个参数的当前值 → 回 "V,<NAME>,<type>,<value>[,<opts>]"
#   "P,<NAME>,<值>"  **临时**设一个参数   → 回 "OK,<NAME>,<新值>" 或 "ERR,<NAME>,<原因>"
#   裸 "P"          回一条用法说明（响亮的失败，不静默）
# 回传统一走 TYPE_COORD + TAG=0x0A（_CFG_TAG，须与 app_rpi.h::RPI_CFG_TAG 一致）。
#
# ⚠️ 与「设置参数」页（save_override）的**唯一区别是不落盘**：值只改到 **main.py 这个
#    进程**的 config 副本里（handler 与各识别任务同进程），下一拍/下一次识别就按新值跑，
#    **不需要重启服务**；Pi 重启后还原成 config.py + config_override.json 的值。
#    于是「试错」不会污染现场标定，而「标好了」再去 Web 页固化。
# ⚠️ 只对迟绑定读取的参数有效（每帧 config.X / 函数内 from config import X）——
#    巡线/识别各任务全是这种；模块级 from-import 的快照看不到，但那些名字
#    （SERIAL_PORT/LOG_DIR…）不在 EDITABLE_CONFIG 里，本就不会被设。
_CFG_TAG = 0x0A


def _cfg_fmt_value(name, value):
    """把参数值格式化成短线（帧只有 64B，见 _send_cfg_reply 的截断）。"""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, int):
        return "%d" % value
    if isinstance(value, float):
        return "%.6g" % value
    return str(value)


def _send_cfg_reply(text):
    """回一帧 TAG=0x0A 的 ASCII 帧。串口断开时静默丢（单片机那侧会超时）。"""
    payload = ("%s" % text).encode("utf-8", "replace")
    # 帧上限 64B：TAG 1B + 文本。超长从尾部截断并留一个 "…" 痕迹 —— 只可能是
    # ERR 的原因串（值本身很短），截断不丢关键信息。
    if len(payload) > 63:
        payload = payload[:62] + b"!"
    try:
        serial_comm.send_frame(TYPE_COORD, bytes([_CFG_TAG]) + payload)
        _note_result_frame()
    except (SerialDisconnected, ValueError, OSError) as e:
        print(f"[WARN] 参数回传失败: {e}")


def _cmd_cfg_get(name):
    """命令 "P?,<NAME>"：回一个参数的当前值（含选项表，供单片机渲染/步进）。"""
    import config
    editable = {n for n, _t, _d, _o in config.EDITABLE_CONFIG}
    if name not in editable:
        _send_cfg_reply("ERR,%s,未登记的参数名" % name)
        return
    value = getattr(config, name)
    typ = config._EDITABLE_TYPES.get(name, "str")
    opts = config._EDITABLE_OPTS.get(name, "")
    text = "V,%s,%s,%s" % (name, typ, _cfg_fmt_value(name, value))
    if opts:
        text += ",%s" % opts
    _send_cfg_reply(text)


def _cmd_cfg_set(name, raw):
    """命令 "P,<NAME>,<值>"：**临时**设一个参数（不落盘，见本节开头说明）。"""
    import config
    ok, msg = config.set_temporary(name, raw)
    if ok:
        _send_cfg_reply("OK,%s,%s" % (name, _cfg_fmt_value(name, getattr(config, name))))
        print(f"[INFO] 单片机临时设参: {msg}")
    else:
        _send_cfg_reply("ERR,%s,%s" % (name, msg))
        print(f"[WARN] 单片机临时设参被拒: {msg}")


def _dispatch_cfg(data):
    """"P" 族的形状拆分与分发（三个字段，_split_payload 那张表管不了）。

    刻意**不**复用 _split_payload：它的第二字段只能是数字（毫秒/次数/亮度），
    而这里第二字段是参数名。形状不认识的响一声 WARN 并回 ERR —— 不静默忽略。
    """
    if data == b"P":
        _send_cfg_reply("ERR,用法: P?,<参数名> 取值 / P,<参数名>,<值> 临时设置（不落盘）")
        return
    if data.startswith(b"P?,"):
        name = data[3:].decode("ascii", "replace").strip()
        if not name:
            _send_cfg_reply("ERR,参数名为空")
            return
        _cmd_cfg_get(name)
        return
    if data.startswith(b"P,"):
        parts = data[2:].split(b",", 1)
        if len(parts) != 2 or not parts[0] or not parts[1]:
            _send_cfg_reply("ERR,用法: P,<参数名>,<值>")
            return
        _cmd_cfg_set(parts[0].decode("ascii", "replace").strip(),
                     parts[1].decode("ascii", "replace").strip())
        return
    print(f"[WARN] P 族指令形状不认识: {data!r}")
    _send_cfg_reply("ERR,形状不认识: P?,<名> 或 P,<名>,<值>")


# ---- 巡线转发（命令 '2' 开始 / '3' 停止）----
# 巡线是长任务（持续发帧），跑独立线程，不阻塞主循环 —— 期间仍可收 '3'/end/r。
_line_thread = None
_line_abort = threading.Event()
# 起流时等上一个线程**收尾**的上限（与 _AIM_JOIN_S 同款、同一个理由:
# 固件的收尾帧与下一条起流帧只隔几毫秒, 不等它退出就会把新流丢掉 —— 见 _cmd_line_start）
_LINE_JOIN_S = 3.0

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
    _cmd_bucket_stop()              # 爆爆桶流同走 COORD 通道（2026-10-24 加）

    if not _line_lock.acquire(blocking=False):
        print("[WARN] 巡线已在运行，忽略重复的启动指令")
        return
    try:
        # ⚠️⚠️ **上一个线程"还在收尾"与"正在正常跑流"必须分开处理**（与 _cmd_aim_start 同款）:
        #   固件每条巡线类步骤收尾都**无条件**发 LINE_STOP（ucRpiPivotTurnLine ⑤ / ucRpiLineRun
        #   收尾），紧接的下一步若又是巡线步就发 LINE_START —— 两条指令在几毫秒内先后到达，
        #   而 run_line_follow 要等一拍（1/LINE_SEND_HZ）才在循环顶部看到 abort 退出。
        #   旧写法把这段窗口里的 `_line_thread.is_alive()` 一律当成"重复启动"丢掉：
        #   新的 '2' 被忽略、旧的流又正好退出 → **这一整步一帧都收不到**，
        #   现场是 ucRpiLineRun 起流段超时长鸣 + 车不动，日志里只有一行 WARN。
        old = _line_thread
        if old is not None and old.is_alive():
            if not _line_abort.is_set():
                print("[WARN] 巡线已在运行，忽略重复的启动指令")
                return
            print(f"[INFO] 巡线：上一个线程还在收尾，等它释放 OpenMV/相机（≤{_LINE_JOIN_S}s）...")
            old.join(timeout=_LINE_JOIN_S)
            if old.is_alive():
                print(f"[WARN] 巡线：上一个线程 {_LINE_JOIN_S}s 内没退出，忽略本次启动")
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
#   · **阻塞类视觉任务**（"1"/"D"/"E"/"F"/"G"/**"H"/"4"/"6"**）→ 数字 = **毫秒** = 单片机愿意等多久。
#     它们的耗时由单片机那一步的超时决定，随指令发下来就不用在两个仓库各记一个数字互相迁就。
#     ⚠️ 固件**确实**会给 "4"（人质, `"4,7500"`）与 "6"（靶标）也带上毫秒后缀 ——
#        所以这两条必须在白名单里（旧版这段注释只列了 "1"/"D"/"E"/"F"/"G"，别照它判断族别）。
#   · **预热类**（"B"/"C"）→ 数字 = **使用次数** = 这台相机接下来要被几个任务消费。
#     相机租约的空闲窗口按它放大（camera_lease.idle_limit_s），于是隔了十几秒的
#     第二个任务也能命中同一份租约。
#   · **补光亮度**（"L"）→ 数字 = **亮度值** 0~100（见 _split_payload 的 docstring）。
#   · **爆爆桶流**（"J", 2026-10-24 加）→ 数字 = **毫秒** = 单片机愿意等多久（同阻塞类，
#     _budget_from_ms 换算成转发预算；裸 "J" 用 config.BUCKET_MAX_S）。
# 加新命令族时**先想清楚自己的数属于哪一类**，别照着字面把毫秒抄过去。
# ⚠️ 这是**白名单**：没登记过的指令若带了逗号会被判成「形状不认识」而拒掉 ——
#    宁可响亮地拒，也不要静默忽略那个数字（少了它本端会按自己的默认值跑，
#    有可能跑得比单片机愿意等的还久）。
# ⚠️ 旧固件发的是裸 "B"/"C"，照样走得了（_split_payload 不带逗号时 num=None → 次数 1 =
#    老行为）。反过来新固件 + 旧 Pi 才会失效：那一条预热被判未知指令丢掉，随后的识别
#    退回「现开现扫」（慢 ~2s, 不坏）—— 所以升级顺序是先 Pi 后固件。
#   · **is_use_picam**（"6"/"7"/"C" 的**第二段**数, 2026-10-16 加）→ 1 = 优先 pi(CSI)摄像头
#     （见 _TWO_NUM_HEADS 与 tasks/_target*.py）。⚠️ "7" 因此也进了本表 —— 它**唯一**的
#     数字含义就是这一档（历史上 "7" 从不带后缀, 不存在被抢的旧语义）。
#   · **档位预热**（"W,<cam>,<uses>,<prof>", 2026-10-23 加）→ 三项全是数字, 但**不走本表**:
#     handle() 在拆分之前按 "W" 头整条拦给 _dispatch_warm_w（同 "P"/"QA" 族的做法）——
#     它要比通用拆分多认一个数（三段）, 而通用表只到两段。
_MS_COMMANDS = (b"1", b"D", b"E", b"F", b"G", b"M", b"H", b"A", b"B", b"C", b"4", b"5", b"6", b"7", b"L", b"J", b"Z")

# 扫描预算的地板（秒）—— 见 _budget_from_ms: 它**只是让循环跑得起来**的下限,
# 不许把预算抬到超过单片机给的时限（那会破坏「谁先放弃只由单片机说了算」）。
_MIN_SCAN_BUDGET_S = 0.2

# 每条"会回结果帧"的命令对应的「失败帧怎么发」——**只给 handle() 的兜底 except 用**
# （见那里的注释: 任务函数抛异常时要补一帧让单片机立刻解锁）。
# ⚠️ 加新的"结果帧命令"时记得同步这张表, 否则那条命令的异常路径还是会让单片机白烧超时。
_FAIL_FRAME_SENDERS = {
    b"1": lambda: _send_qr_result("QRF"),
    b"D": lambda: _send_qr_result("QRF"),
    b"E": lambda: _send_qr_result("QRF"),
    b"F": lambda: _send_qr_result("QRF"),
    b"4": lambda: _send_hostage_result(build_result_frame(False, [])),
    b"5": lambda: _send_color_result(build_color_frame(False, [])),
    b"6": lambda: _send_target_result(build_target_frame(False, [])),
    b"A": lambda: _send_rect_result(build_rect_frame(False)),
    b"G": lambda: _send_road_result(build_road_frame(False)),
    b"M": lambda: _send_road_result(build_road_frame(False)),   # 反向看路, 同 0x08 帧
    b"H": lambda: _send_road_yaw_result(build_road_yaw_frame(False)),
    b"Z": lambda: _send_health_result(0, 0, 0, 0),   # 四路相机健康检查, 异常时回四项全失败
}

_COMMANDS = {
    b"1": _cmd_qr,              # 二维码识别（同步阻塞）
    b"2": _cmd_line_start,      # 巡线转发开始（独立线程）
    b"3": _cmd_line_stop,       # 巡线转发停止
    b"4": _cmd_hostage,         # 人质形状识别（同步阻塞）
    b"5": _cmd_color,           # 色球顺序识别（同步阻塞）
    b"6": _cmd_target,          # 靶标识别 + 靶心定位（同步阻塞）
    b"7": _cmd_aim_start,       # 靶标流跟踪开始（独立线程, 0x05 流帧 ~20Hz）
    b"8": _cmd_aim_stop,        # 靶标流跟踪停止
    b"J": _cmd_bucket_start,    # 爆爆桶坐标转发开始（独立线程, 0x0B 流帧 ~20Hz; "J,<ms>"=预算）
    b"K": _cmd_bucket_stop,     # 爆爆桶坐标转发停止
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
    b"M": _cmd_road_rev,        # 反向看路（2026-10-31 加, 镜像系量黄区右边缘; 同 0x08 帧; "M,<ms>"=预算）
    b"H": _cmd_road_yaw,        # 看路校准（主摄拟合黄区直边方向角，阻塞；回 0x09 帧, 只测不改）
    b"Z": _cmd_health,          # 四路相机健康检查（阻塞; 回 0x0D 帧；查完解除全部占用, 2026-11 加）
    b"x": _cmd_cancel,          # 取消当前任务/流（单片机中止脚本/重握手时发; 只收不回）
    b"L": _cmd_openmv_led,      # OpenMV 补光亮度（"L,<0~100>"; 裸"L"=重发 config 值; 只发不回）
    b"T": _cmd_openmv_led_toggle,   # OpenMV 补光**临时**开关（裸"T", 无参; 亮↔灭; 只发不回）
    b"RM": _cmd_pi_main_restart,    # 重启主程序（OLED「重启main」; 单帧直达, 不走 "r" 双发门; 只收不回）
    b"RS": _cmd_pi_svc_restart,     # 重启 gks 服务（OLED「重启服务」; systemctl restart; 只收不回）
    b"CL": _cmd_pi_log_clear,       # 清屏日志（OLED「清屏Log」; GKS_Web 同一条 request_clear 通道）
    # TODO(工科赛): 物块检测/色环/码垛/扫雷/转盘等任务命令在此追加
}


class SerialInterruptHandler:
    def __init__(self, should_stop=None):
        self.running = True
        # 可选的外部停止谓词：返回 True 时主循环退出（用于 SIGTERM/SIGINT 优雅退出）
        self._should_stop = should_stop or (lambda: False)
        # Web「自动下发二维码」中途改值监听（守护线程；值一变就推 QP 帧，见
        # _auto_qr_watch_loop）。挂这里而不是 _run：握手失败/重连期间也要持续监听。
        threading.Thread(target=_auto_qr_watch_loop, name="auto-qr-watch",
                         daemon=True).start()

    def handle(self, data):
        """分发一条命令 payload。

        payload 可以是 `<命令码>` 或 `<命令码>,<毫秒>`（后者 = 单片机告诉本端
        「我最多等你这么久」，见 docs/串口协议.md §6）。拆出来的头用来查表，
        **不带逗号时 head == data，行为与加这个特性之前一字不差**。
        """
        print(f"收到指令: {data!r}")

        # "P" 族（取参/临时设参）在查表**之前**拦走：它的第二字段是参数名不是数字，
        # _split_payload 那张表管不了（会判成"后缀不是数字"再落到未知指令）。
        if data == b"P" or data.startswith(b"P,") or data.startswith(b"P?,"):
            _dispatch_cfg(data)
            return

        # "W" 族（带档位的预热, 2026-10-23 加）**同样在查表之前拦走**: 它是三段数字
        # （"W,<cam>,<uses>,<prof>"），通用拆分只认到两段。见 _dispatch_warm_w。
        if data == b"W" or data.startswith(b"W,"):
            _dispatch_warm_w(data)
            return

        # "QA"/"QE"/"QU"/"QC"（OLED 第 2 页「自动下发二维码」族, 2026-10 加）同样在
        # 查表**之前**拦走: 第二字段是开关位/三位数字, _split_payload 管不了。
        if (data == b"QA" or data == b"QC"
                or data.startswith(b"QE,") or data.startswith(b"QU,")):
            _dispatch_qr_auto(data)
            return

        # num = 命令码后面那个数值。**含义按命令码而定**（毫秒 / 使用次数），见 _split_payload。
        # num2 = 第二段（只有 "6"/"7"/"C" 能有）= is_use_picam, 见 _TWO_NUM_HEADS。
        head, num, num2 = _split_payload_ext(data)

        # 系统命令（不走任务注册表）
        if data == b"end":
            print("[INFO] 收到结束指令，主循环退出")
            _cmd_line_stop()          # 停巡线线程，避免主循环退出后仍在发帧
            _cmd_aim_stop()           # 同理：靶标流线程也要停，否则会占着相机继续发帧
            _cmd_bucket_stop()        # 同理：爆爆桶流线程也要停（2026-10-24 加）
            camera_lease.LEASE.release_now()   # 别把预热好的相机带进下次运行
            camera_picam.PICAM.release_now()   # 同理（CSI 那条路, 2026-10-16 加）
            self.running = False
            return
        if data == b"r":
            # ⚠️ 与 'sd'/'rb' 同样要**连发两次**: 重启主程序也会中断当前任务/路线
            if not _require_double_send(b"r", "重启程序"):
                return
            _cmd_line_stop()
            _cmd_aim_stop()           # 同理
            _cmd_bucket_stop()        # 同理：爆爆桶流（2026-10-24 加）
            camera_lease.LEASE.release_now()   # 同理: 重启前把相机交还, 否则新进程起不来
            camera_picam.PICAM.release_now()   # 同理（CSI 的 rpicam-vid 也要收掉）
            # ⚠️ 把停止谓词传进去: 那 20 秒等待期间按的 Ctrl+C / 发来的 SIGTERM 必须算数
            #    （否则进程会"自己回来" —— 见 serial_comm.restart_program 的注释）
            serial_comm.restart_program(self._should_stop)
            return

        if head in (b"1", b"4", b"5", b"6", b"A", b"D", b"E", b"F", b"G", b"M", b"H", b"Z"):
            _cmd_line_stop()          # 结果帧与巡线帧同为 COORD，先停巡线防帧交错
            # ⚠️ 靶标流也要停：流帧与"6"的结果帧同为 COORD（都是 Pi→单片机），
            #    交错起来单片机那边会拿流帧去喂 wait_target_result。而且这些任务
            #    都要用主摄 —— 流线程占着设备的话 open_camera 会直接失败。
            #    ⚠️ "D"（主摄扫码）同样在主摄上，所以也在这个元组里。
            #    ⚠️ "B"/"C"（预热）**刻意不在**：它们只开相机、不发结果帧，
            #       而且 "B" 恰恰排在巡线**之前** —— 停巡线是错的。别顺手加进来。
            _cmd_aim_stop()
            # ⚠️ 爆爆桶流（0x0B）同走 COORD 通道，也要停（2026-10-24 加）。
            #    它不占主摄 —— 停它是为了 COORD 帧，不是为释放相机。
            _cmd_bucket_stop()

        try:
            func = _COMMANDS.get(head)
            if func is None:
                print(f"[WARN] 未知指令: {data!r}")
                return
            # 新指令开始前清零取消事件（"x" 自己的 func 会在 clear 之后 set, 顺序安全）。
            # 连发 "x"+"5" 时: 先取消旧任务, 再干净地起任务 —— 语义就是「顶掉重来」。
            _task_abort.clear()
            if num2 is not None:
                func(num, num2)       # 两段形状（只有 "6"/"7"/"C"; 第二段 = is_use_picam）
            elif num is not None:
                func(num)             # 带数值后缀的指令（_MS_COMMANDS 里那几条; 毫秒 or 次数）
            else:
                func()
        except SerialDisconnected as e:
            print(f"[WARN] 任务中止: {e}，返回主循环等待重连")
        except Exception as e:
            import traceback
            print(f"[ERROR] 未捕获异常: {type(e).__name__}: {e}")
            traceback.print_exc()
            # ⚠️⚠️ **兜底也要把失败帧发出去**（2026-10 加）。结果打包/取字段这一路抛异常
            #    时（_cmd_road 的注释记过这个坑: 「Pi 日志报成功、单片机却一直
            #    ROAD fail: wait st=0 —— 回来一看是 KeyError, 结果帧根本没发出去」），
            #    失败帧是这几条命令「让单片机**立刻解锁**」的唯一手段: 不发它，单片机要
            #    白烧满整个超时（最长 7.5~8s，还会把链路顶到 DISCONNECTED 边缘），
            #    而现场只看到一行 traceback。
            #    ⚠️ 只在**这一轮还没回过任何结果帧**时补（_note_result_frame 记录）:
            #       已经回过一帧再补一帧，那多出来的一帧会落进单片机下一次等待窗口
            #       被当成那一次的结果 —— 正是 "x"/重扫机制要治的错位。
            #    ⚠️ 自己再包一层: 补发失败（链路断了）不该再抛一个异常出来。
            if (head in _FAIL_FRAME_SENDERS) and ((time.time() - _last_result_frame_t) > 2.0):
                try:
                    _FAIL_FRAME_SENDERS[head]()
                    print("[WARN] 已补发失败帧（兜底），让单片机立刻解锁")
                except Exception as e2:                    # noqa: BLE001
                    print(f"[WARN] 兜底失败帧也发不出去: {type(e2).__name__}: {e2}")

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
                # Web 控制端「自动下发二维码」：握手一成功就推一帧**伪装帧 "QP<ddd>"**
                # （新固件 vRpiTask 收到 → OLED 显示 QR:xxx* 并跳过后续扫码；旧固件
                # 忽略它，由 _cmd_qr_common 的扫码指令注入兜底 —— 详见 _web_auto_qr）。
                # 走 _auto_qr_push 统一入口：顺带刷新去重状态，防监听线程立刻又推一帧。
                auto_qr = _web_auto_qr()
                if auto_qr:
                    _auto_qr_push(auto_qr, "握手成功")
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
            # ⚠️⚠️ 重启**之前**必须把两台相机交还（2026-10-16 补）: 下面那条重启是
            #    `os.execl` **换进程镜像**（serial_comm._exec_self）—— 换镜像只换掉本进程的
            #    代码/数据, **不会带走子进程, 也不会执行任何 finally**: CSI 那颗的
            #    `rpicam-vid` 会原地活下来, 而新镜像的 PICAM 以为"没有流"。
            #    后果是 libcamera 被一个谁也认不出的进程永久占着（现象 = 之后每次起流都
            #    `failed to acquire camera`, 只能手工 pkill; 2026-10-16 现场就是这条）。
            #    main.py 退出前那段收尾罩不到这条路径 —— 它走的是 exec, 不是退出。
            #    handler 里另外两条（"end"/"r"）早就交了, 这条自愈路径当时漏了。
            for _mod, _rel in (("camera_lease", "LEASE"), ("camera_picam", "PICAM")):
                try:
                    _m = __import__(_mod)
                    getattr(_m, _rel).release_now()
                except Exception as e:                 # noqa: BLE001
                    print(f"[WARN] 重启前释放 {_mod} 失败（不拦重启）: {e}")
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
                    if is_bucket_running():
                        print("[INFO] 串口重连: 停掉旧会话残留的爆爆桶流")
                        _cmd_bucket_stop()
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
