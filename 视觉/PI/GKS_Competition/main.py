#!/usr/bin/env python3
# main.py — GKS 工科赛 主程序骨架
#
# 当前仅包含通用的生命周期管理：日志初始化、无头运行(SERVICE_MODE)、
# 优雅退出(SIGTERM/SIGINT)、空闲心跳主循环。
#
# 尚未从 GCS(cv_python) 移植任何任务/视觉/串口逻辑 —— 后续在此骨架上逐步接入。
#
# 启动方式：
#   - 手动调试:   python3 main.py
#   - systemd 服务: gks.service（SERVICE_MODE=1，无头运行，Restart=always）
import os
import sys
import time
import signal
import logging
import threading

# 树莓派上限制 OpenBLAS/OpenMP 线程数，避免后续视觉/矩阵运算抢占所有核
os.environ["OPENBLAS_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"

# 服务模式下无显示环境（无 X Server），后续任务应据此禁用 cv2.imshow 等 GUI 操作
SERVICE_MODE = os.environ.get("SERVICE_MODE") == "1"

# 主循环运行标志：SIGTERM/SIGINT 置 False，主循环退出后进入清理流程
_running = True

# 当前日志文件句柄（「真清屏」要 truncate 它, 见 _clear_log_now）
_log_file = None


def _sigterm_handler(signum, frame):
    """SIGTERM/SIGINT → 优雅退出。

    后续任务引入摄像头/串口等资源后，在此处统一释放（参考 cv_python/main.py
    的做法：释放摄像头、关闭串口、关闭预览窗口）。
    """
    global _running
    if not _running:
        return   # 已在退出流程中：反复按 Ctrl+C 时不再重复刷日志
    print("\n[INFO] 收到终止信号，正在清理资源...")
    _running = False
    print("[INFO] 清理完成，程序即将退出")


def _clear_log_now():
    """**真清屏**（2026-10 加, GKS_Web 日志页「清屏」按钮 → 标志文件 → 本线程消费）。

    旧版清屏只清浏览器 DOM, 文件还在 —— 重连/刷新后历史又被回显（"假清屏"）。
    现在把当前 run 的 log.txt 从**本进程内部**截断（外部 truncate 会让我们持有的
    fd 偏移悬空、写出稀疏空洞文件, 所以必须由写者自己来）, 同时清内存环形缓冲,
    再落一行标记。所有正在 tail 这个文件的连接（GKS_Web 的 SSE 流）靠
    「文件变小的检测」自动从头跟上。
    """
    global _log_file
    try:
        import log_buffer
        if _log_file is not None:
            _log_file.flush()
            _log_file.seek(0)
            _log_file.truncate(0)
        log_buffer.clear()
        print("[INFO] === 日志已清空（操作员在 Web 页点了「清屏」） ===")
        if _log_file is not None:
            _log_file.flush()
    except Exception as e:
        try:
            print(f"[WARN] 清屏处理出错: {e}")
        except Exception:
            pass


def _log_clear_watchdog():
    """轮询 logs/.clear_flag（GKS_Web 写）, 出现即消费并执行 _clear_log_now。

    ⚠️ 刻意用标志文件而不是 SIGUSR1: 混合部署期（新版 GKS_Web + 旧版 main.py）
       信号会终止旧进程; 文件轮询天然只对实现了它的进程生效。
    0.5s 的响应延迟对"清屏"毫无感知差别。
    """
    import logstore
    while True:
        time.sleep(0.5)
        try:
            if logstore.clear_requested() and logstore.consume_clear():
                _clear_log_now()
        except Exception:
            pass   # 清屏是旁路功能, 任何异常都不许带走本线程


_log_io_warned = False   # 「日志文件写不进去」只喊一次（见 _TeeWriter.write）


class _TeeWriter:
    """同时写入日志缓冲和日志文件，捕获所有 print() 输出（不再打印控制台）。

    行首写 [HH:MM:SS] 时间戳标记，同一秒内的连续日志共享一个时间标记，
    避免高频 print 伤害 SD 卡性能。
    """
    def __init__(self, buf, log_file):
        self.buf = buf
        self.log_file = log_file
        self._at_line_start = True
        self._last_ts = None

    def _prefixed(self, message):
        if self._at_line_start and message and message != '\n':
            ts = time.strftime("[%H:%M:%S] ")
            if ts != self._last_ts:
                self._last_ts = ts
                return ts + message
            return " " * len(ts) + message
        return message

    def write(self, message):
        # ⚠️⚠️ **写日志绝不允许抛异常**（2026-10 修）。`print()` 会走这里, 而
        #    SD 卡写满 / 变只读 / 被拔时 `log_file.write/flush` 会抛 OSError ——
        #    异常从 print() 里冒出去: 在**接收线程**里就是那个线程直接死掉, 而
        #    `self.ser` 还留着 → handler._is_serial_ok() 继续返回 True →
        #    主循环照常发心跳（单片机看着链路完好）却再也收不到任何指令 =
        #    **永久变聋、零日志、不重连、不重启**（本工程最坏的一类故障）。
        #    日志是旁路, 自己坏掉可以接受; 把主链路带崩不可以。
        out = self._prefixed(message)
        self._at_line_start = message.endswith('\n')
        try:
            self.buf.write(out)
        except Exception:
            pass
        if self.log_file:
            try:
                self.log_file.write(out)
                if '\n' in message:
                    self.log_file.flush()
            except Exception as e:
                # 只喊一次（同一条错误反复刷屏会把 Web 日志面板冲垮）
                global _log_io_warned
                if not _log_io_warned:
                    _log_io_warned = True
                    try:
                        self.buf.write(f"[WARN] 日志文件写入失败（磁盘满/只读?）: {e}"
                                       f" —— 后续只写内存缓冲, 不再重试\n")
                    except Exception:
                        pass

    def flush(self):
        try:
            self.buf.flush()
        except Exception:
            pass
        if self.log_file:
            try:
                self.log_file.flush()
            except Exception:
                pass


def setup_logging():
    """初始化日志：写日志缓冲（Web 端显示）+ 文件，不再打印控制台。

    logging 模块写一份（带级别前缀），stdout 重定向到 _TeeWriter 再写一份
    （捕获所有裸 print() 输出），两者都指向日志缓冲和日志文件。
    """
    from config import LOG_DIR, ENABLE_LOGGING
    import log_buffer

    root_logger = logging.getLogger()
    root_logger.setLevel(logging.DEBUG)

    # 日志缓冲（Web 端 /log 读取）
    buf_handler = logging.StreamHandler(log_buffer.Writer())
    buf_handler.setLevel(logging.INFO)
    buf_handler.setFormatter(logging.Formatter(
        '%(asctime)s [%(levelname)s] %(message)s', datefmt='%H:%M:%S'))
    root_logger.addHandler(buf_handler)

    log_f = None
    log_path = None
    if ENABLE_LOGGING:
        try:
            # 2026-10 起日志进**运行目录**（logstore.start_run）:
            #   logs/run_YYYYMMDD_HHMMSS/log.txt + frames/<task>_NNNN.jpg（帧归档,
            #   见 logstore.archive_frame）。run 目录同时是 Web「回看/清理」的单元。
            import logstore
            run_dir = logstore.start_run()
            log_path = os.path.join(run_dir, "log.txt")
            log_f = open(log_path, "a", encoding="utf-8")

            file_handler = logging.FileHandler(log_path, encoding="utf-8")
            file_handler.setLevel(logging.DEBUG)
            file_handler.setFormatter(logging.Formatter(
                '%(asctime)s [%(levelname)s] %(name)s: %(message)s',
                datefmt='%H:%M:%S'))
            root_logger.addHandler(file_handler)
        except Exception as e:
            print(f"[ERROR] 无法创建日志文件: {e}")
            log_f = None

    # 重定向 stdout，捕获所有 print() 输出（写缓冲 + 文件，不再写控制台）
    sys.stdout = _TeeWriter(log_buffer.Writer(), log_f)
    globals()["_log_file"] = log_f        # 供 SIGUSR1「真清屏」拿到句柄
    if log_path:
        print(f"[INFO] 日志文件: {log_path}")


def get_logger(name):
    """返回配置好的 logging.Logger，替代裸 print()。

    用法: from main import get_logger; _log = get_logger(__name__)
    """
    return logging.getLogger(name)


def main():
    print("[INFO] GKS 工科赛主程序已启动")
    print(f"[INFO] SERVICE_MODE = {SERVICE_MODE}")
    # 连接模式（正常/模拟）由 serial_comm 同时等待串口与模拟器两个信号后自动决定，
    # 详见 serial_comm._open()。此处仅提示等待中。

    # ── Web 控制页（:8000）已废弃，此处**故意不再 start_server()** ──
    # 唯一入口是 GKS_Web（:9000，gks-web.service），它已覆盖本页全部功能（摄像头预览 /
    # 扫码 / 人质 / 色球 / 日志 / 参数设置——两边的 /config 端点是同一份实现）。
    # camera_web.py 文件必须保留：handler.py 的 _stop_main_stream() 仍要
    # `from camera_web import _stop_stream`（人质/色球识别前释放主摄）。
    # 需要单独调试那个页面时手动起：python3 camera_web.py --port 8000
    # 别在这里恢复 start_server()：两个页面并存会争抢摄像头，参数也变成两个入口。

    # ── 启动 OpenMV 寻线传感器读取线程 ──
    # OpenMV 上电即持续发数据（独立传感器），此处只负责读 + 缓存最新值；
    # 串口打开失败会静默重试，绝不影响主程序启动。
    try:
        from openmv_serial import openmv_reader
        openmv_reader.start()
    except Exception as e:
        print(f"[WARN] OpenMV 读取线程启动失败: {e}")

    # ── 相机租约空闲看门狗 ──
    # ⚠️ 不能省：单片机发了 "B"/"C" 预热却没跟进识别时（路线被改 / 脚本被中止 /
    #    操作员反悔），没有它就**永久占着相机** —— Web 预览从此打不开，且无任何报错。
    #    这是这条链路上唯一的安全阀，纪律同固件侧的 BT_EDIT_IDLE_TIMEOUT_MS。
    try:
        import camera_lease
        threading.Thread(target=camera_lease.lease_watchdog,
                         name="cam-lease-watchdog", daemon=True).start()
    except Exception as e:
        print(f"[WARN] 相机租约看门狗启动失败: {e}")

    # ── pi(CSI)摄像头持有器的空闲看门狗（2026-10-16 加, 平移射靶看靶用）──
    # 同一套纪律、另一台设备：预热了 CSI 却没跟进识别时，不关它就一直占着 ISP
    # （Web 的 pi 预览从此打不开，且无人报错）。与上面那颗**不冲突**：CSI 是 libcamera
    # 那条路，不占 USB 相机租约。
    try:
        import camera_picam
        # ⚠️ 启动先清场（2026-10-16 加）: 上次运行若是**换镜像重启**（handler 自愈那条,
        #    `os.execl`）或被 kill -9 / 掉电收走的, 它留下的 `rpicam-vid` 子进程还活着 ——
        #    libcamera 独占, 不清掉本次**全程**都开不了 CSI（现象 = `failed to acquire
        #    camera`, 而那个 PID 谁也认不出来; 2026-10-16 现场就是这条）。判据只认"自己人"
        #    （带 OWN_TAG 的, 或父进程是本进程 / 已被 init 收养的同类形状）, 网页预览那条
        #    路不会被误伤 —— 见 camera_picam.reap_stale 的注释。
        camera_picam.reap_stale()
        threading.Thread(target=camera_picam.picam_watchdog,
                         name="picam-watchdog", daemon=True).start()
    except Exception as e:
        print(f"[WARN] pi摄像头看门狗启动失败: {e}")

    # ── 「真清屏」轮询（GKS_Web 日志页的清屏按钮走这里, 见 _log_clear_watchdog）──
    threading.Thread(target=_log_clear_watchdog,
                     name="log-clear-watchdog", daemon=True).start()

    # ── 串口接收 / 指令分发主循环 ──
    # 收单片机（或模拟器）帧 → handler 查注册表分发任务（参考 cv_python）。
    # 收到 SIGTERM/SIGINT 时 _running 置 False，主循环随之退出。
    from handler import SerialInterruptHandler
    handler = SerialInterruptHandler(should_stop=lambda: not _running)
    handler.run()

    # 退出前把相机都交还（同 handler 的 "end"/"r" 两条分支的收尾）。
    # ⚠️ CSI 那颗尤其不能漏（2026-10-16 加）: 它是 `rpicam-vid` **子进程**,
    #    父进程没了它还活着（要么被 systemd 的 cgroup 一起收, 要么就一直攥着 ——
    #    libcamera 独占, 下次启动谁都开不了那颗相机, 报 `failed to acquire camera`）。
    for _mod, _rel in (("camera_lease", "LEASE"), ("camera_picam", "PICAM")):
        try:
            _m = __import__(_mod)
            getattr(_m, _rel).release_now()
        except Exception as e:                             # noqa: BLE001
            print(f"[WARN] 退出前释放 {_mod} 失败（不拦退出）: {e}")

    print("[INFO] 主循环退出，程序结束")


# 在导入其他模块之前初始化日志（捕获后续所有 import 和 print 输出）
setup_logging()

if __name__ == "__main__":
    signal.signal(signal.SIGTERM, _sigterm_handler)
    signal.signal(signal.SIGINT, _sigterm_handler)   # Ctrl+C 也走优雅退出
    # 兜底：未捕获异常走 stderr（systemd journald 可查），这里额外用 print()
    # 写一份到日志文件/Web 日志面板 —— 否则启动期异常只在 stderr，日志里
    # 表现为「启动后无声退出」，很难定位（如导入错误）。
    try:
        main()
    except Exception:
        import traceback
        print("[ERROR] 主程序异常退出:")
        print(traceback.format_exc())
        raise
