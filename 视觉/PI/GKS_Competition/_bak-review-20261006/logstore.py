# logstore.py — 运行(run)目录 / 帧归档 / 日志清理与回看（2026-10 加）
#
# 为什么有这个东西
# ----------------
# 旧机制两个痛点:
# 1. 日志按「每次启动一个 logs/YYYYMMDD_HHMMSS_log.txt」平铺, 相机帧却永远只写
#    logs/xxx_last.jpg（覆盖式）—— 想比对「第 N 次跑色球识别时相机看到什么」没门,
#    图早被下一次覆盖了。
# 2. 清理/回看没有入口, 得进 VSCode 手删 logs/。
#
# 新机制: main.py 每次启动开一个**运行目录**
#     logs/run_YYYYMMDD_HHMMSS/
#         log.txt                 ← 本次运行的完整日志（main.py 的 TeeWriter 写这里）
#         frames/<key>_NNN.jpg    ← 每次任务执行归档一帧, 序号递增**永不覆盖**
#         last_<key>.jpg          ← 本次运行内该任务的最新一帧（便利副本）
# 同时保留 logs/<key>_last.jpg 固定路径（GKS_Web 的轮询端点 /api/*/frame 读它,
# 改动它要动一串端点, 不值得）——固定路径是"最新", run 目录里的是"全部历史"。
#
# 跨进程约定（main.py 与 GKS_Web 两个进程都用本模块）:
#   · logs/main.pid 内容 = "<pid>\n<run_dir绝对路径>"。pid 活着 = 那个 run 是
#     **活动运行**, 清理时跳过它; GKS_Web 归档自己就地识别的帧时也归进它。
#   · 本模块**无副作用**（import 不建目录不写文件）, 创建动作只在 start_run()。
#   · 所有对 GKS_Web 暴露的 run 名只允许 `run_XXX` / 旧式 `XXX_log.txt` 白名单形态,
#     路径拼接前过 _safe_name(), 杜绝目录穿越。
import os
import re
import shutil
import time

from config import LOG_DIR

RUN_PREFIX = "run_"
PID_FILE = os.path.join(LOG_DIR, "main.pid")
# 「真清屏」标志文件（GKS_Web 创建 → main.py 的轮询线程消费并截断自己的日志文件）。
# ⚠️ 刻意不用信号（SIGUSR1）: 混合部署期（新版 GKS_Web + 旧版 main.py）里, 旧进程
#    没注册过该信号的处理器, 默认动作是**终止进程** —— 一个清屏把 main.py 杀了。
CLEAR_FLAG = os.path.join(LOG_DIR, ".clear_flag")

# run 目录名: run_YYYYMMDD_HHMMSS（加 _2/_3 后缀防同秒冲突）
_RUN_RE = re.compile(r"^run_\d{8}_\d{6}(?:_\d+)?$")
# 旧式平铺日志文件名: YYYYMMDD_HHMMSS_log.txt
_LEGACY_RE = re.compile(r"^\d{8}_\d{6}_log\.txt$")

# frames/<key>_<seq>.jpg 的合法名（key 白名单在调用方, 这里只验形态）
_FRAME_RE = re.compile(r"^[a-z_]+_\d{3,5}\.jpg$")

# 当前进程持有的 run 目录（只有 main.py 调 start_run 后非 None;
# GKS_Web 进程里恒为 None → 走 pid/最新目录推断, 见 active_run_dir）
_current_run = None
_seq = {}          # key → 已归档帧数（进程内计数, run 目录换新时清）


# ==================== run 目录 ====================

def start_run():
    """开一个新的运行目录（main.py 启动时调一次）, 返回其绝对路径。

    幂等保护: 本进程已开过就返回现有的（main.py 只该调一次, 防手滑）。
    """
    global _current_run, _seq
    if _current_run is not None and os.path.isdir(_current_run):
        return _current_run
    os.makedirs(LOG_DIR, exist_ok=True)
    base = time.strftime("run_%Y%m%d_%H%M%S")
    run_dir = os.path.join(LOG_DIR, base)
    n = 2
    while os.path.exists(run_dir):          # 同秒启动撞名 → 加后缀
        run_dir = os.path.join(LOG_DIR, f"{base}_{n}")
        n += 1
    os.makedirs(os.path.join(run_dir, "frames"), exist_ok=True)
    _current_run = run_dir
    _seq = {}
    _write_pid(run_dir)
    return run_dir


def _write_pid(run_dir):
    """把本进程 pid + run 目录写到 logs/main.pid（GKS_Web 靠它找活动运行）。"""
    try:
        with open(PID_FILE, "w", encoding="ascii") as f:
            f.write(f"{os.getpid()}\n{os.path.abspath(run_dir)}\n")
    except Exception as e:
        print(f"[WARN] 写 pid 文件失败: {e}")


def read_pid_file():
    """返回 (pid, run_dir) 或 (None, None)。pid 不活着时 run_dir 仍返回（供回看）。"""
    try:
        with open(PID_FILE, "r", encoding="ascii") as f:
            lines = [l.strip() for l in f.readlines() if l.strip()]
        if len(lines) < 2:
            return None, None
        return int(lines[0]), lines[1]
    except Exception:
        return None, None


def _pid_alive(pid):
    if not pid:
        return False
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def request_clear():
    """GKS_Web 侧调用：请求活动 main.py 清空它的日志文件（写标志文件）。

    返回 True = 标志已落盘。是否真被消费由 main.py 的轮询线程决定
    （见 consume_clear），它没在跑/是旧版本时标志会一直留着 —— 调用方
    据此回退（直接截断或提示重启 gks 服务）。
    """
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(CLEAR_FLAG, "w") as f:
            f.write(str(time.time()))
        return True
    except OSError as e:
        print(f"[WARN] 写清屏标志失败: {e}")
        return False


def clear_requested():
    """标志文件是否还在（= 请求还没被 main.py 消费）。"""
    return os.path.exists(CLEAR_FLAG)


def consume_clear():
    """main.py 侧调用：取走清屏请求（删除标志文件）。返回 True = 本次有请求。"""
    try:
        os.remove(CLEAR_FLAG)
        return True
    except OSError:
        return False


def active_run_dir():
    """当前**活动**运行目录（优先）, 没有则最新的 run 目录, 再没有返回 None。

    活动的判据 = main.pid 里的 pid 还活着（main.py 可能是 service 或调试两种方式起的,
    只要它调过 start_run 就有 pid 文件）。
    """
    if _current_run is not None and os.path.isdir(_current_run):
        return _current_run
    pid, run_dir = read_pid_file()
    if run_dir and os.path.isdir(run_dir):
        if _pid_alive(pid):
            return run_dir
    runs = list_runs()
    return runs[0]["dir"] if runs else None


def list_runs():
    """全部运行（新→旧）, 供回看列表。含旧式平铺 *_log.txt（当做一个没有 frames 的运行）。

    返回 [{"name", "dir", "mtime", "size_bytes", "is_run_dir", "active",
           "log_path", "frames": N}]，按 mtime 降序。
    """
    out = []
    pid, active_dir = read_pid_file()
    active_alive = active_dir is not None and _pid_alive(pid)
    try:
        names = os.listdir(LOG_DIR)
    except OSError:
        return []
    for name in names:
        full = os.path.join(LOG_DIR, name)
        if os.path.isdir(full) and _RUN_RE.match(name):
            log_path = os.path.join(full, "log.txt")
            frames_dir = os.path.join(full, "frames")
            n_frames = 0
            try:
                n_frames = len([x for x in os.listdir(frames_dir)
                                if x.endswith(".jpg")])
            except OSError:
                pass
            out.append({
                "name": name, "dir": full,
                "mtime": _dir_mtime(full, log_path),
                "size_bytes": _tree_size(full),
                "is_run_dir": True,
                "active": bool(active_alive and os.path.abspath(full)
                               == os.path.abspath(active_dir)),
                "log_path": log_path if os.path.exists(log_path) else None,
                "frames": n_frames,
            })
        elif os.path.isfile(full) and _LEGACY_RE.match(name):
            out.append({
                "name": name, "dir": None,
                "mtime": os.path.getmtime(full),
                "size_bytes": os.path.getsize(full),
                "is_run_dir": False,
                "active": False,
                "log_path": full,
                "frames": 0,
            })
    out.sort(key=lambda r: r["mtime"], reverse=True)
    return out


def _dir_mtime(run_dir, log_path):
    try:
        return os.path.getmtime(log_path)
    except OSError:
        try:
            return os.path.getmtime(run_dir)
        except OSError:
            return 0.0


def _tree_size(path):
    total = 0
    for root, _dirs, files in os.walk(path):
        for f in files:
            try:
                total += os.path.getsize(os.path.join(root, f))
            except OSError:
                pass
    return total


def _safe_name(name):
    """run 名 / 帧文件名的白名单校验（GKS_Web 传来的参数一律先过这里）。"""
    return bool(name) and (".." not in name) and ("/" not in name) and ("\\" not in name)


def run_dir_of(name):
    """run 名 → 目录路径（带白名单校验）, 不存在返回 None。"""
    if not _safe_name(name) or not _RUN_RE.match(name):
        return None
    full = os.path.join(LOG_DIR, name)
    return full if os.path.isdir(full) else None


# ==================== 帧归档 ====================

def archive_frame(key, frame, legacy_path=None):
    """归档一帧任务结果图: run 目录里写 frames/<key>_NNN.jpg（序号递增, 永不覆盖）
    + last_<key>.jpg（run 内最新）, 并镜像到 legacy_path（logs/xxx_last.jpg, Web
    轮询端点读的固定路径, 保持兼容）。

    key: 任务短名（qr/hostage/color/rect/road/target/...）。
    frame: BGR ndarray（cv2.imwrite 直接吃）; None 直接返回。
    返回归档文件名（"key_0007.jpg", 相对 frames/, 2026-10 加 —— 靶标流的定格帧
    要把它打进行里, 供「标靶练习」页把帧与那一发 TGTS 记录精确配对）;
    frame 为 None 或失败时返回 None。老调用方不接返回值, 行为不变。
    归档目标 run 的选择: 本进程的活动 run → main.pid 的活动 run → 最新 run
    → 都没有就现开一个（GKS_Web 独立跑就地识别、main.py 没起时也能归档）。
    任何失败只打 WARN —— 归档是旁路, 绝不能影响任务本身。
    """
    import cv2      # 局部导入: 本模块被 GKS_Web import 时已带 cv2, 但保持低依赖
    if frame is None:
        return None
    try:
        run_dir = _current_run if (_current_run and os.path.isdir(_current_run)) \
            else active_run_dir()
        if run_dir is None:
            run_dir = start_run()
        frames_dir = os.path.join(run_dir, "frames")
        os.makedirs(frames_dir, exist_ok=True)
        n = _seq.get(key, 0) + 1
        archive_path = os.path.join(frames_dir, f"{key}_{n:04d}.jpg")
        if not cv2.imwrite(archive_path, frame):
            raise IOError(f"imwrite 失败: {archive_path}")
        cv2.imwrite(os.path.join(run_dir, f"last_{key}.jpg"), frame)
        _seq[key] = n
        if legacy_path:
            cv2.imwrite(legacy_path, frame)
        return f"{key}_{n:04d}.jpg"
    except Exception as e:
        print(f"[WARN] 帧归档失败({key}): {e}")
        return None


def frames_of(run_name):
    """run 名 → 其 frames/ 下的 jpg 列表（文件名升序 = 时间序）, 供回看。"""
    run_dir = run_dir_of(run_name)
    if run_dir is None:
        return None
    frames_dir = os.path.join(run_dir, "frames")
    try:
        names = sorted(x for x in os.listdir(frames_dir)
                       if x.endswith(".jpg") and _FRAME_RE.match(x))
    except OSError:
        return []
    return [{
        "file": n,
        "mtime": os.path.getmtime(os.path.join(frames_dir, n)),
        "size": os.path.getsize(os.path.join(frames_dir, n)),
    } for n in names]


def frame_path(run_name, file_name):
    """run 名 + 帧文件名 → 绝对路径（白名单校验）, 不存在返回 None。"""
    run_dir = run_dir_of(run_name)
    if run_dir is None or not _safe_name(file_name) or not _FRAME_RE.match(file_name):
        return None
    full = os.path.join(run_dir, "frames", file_name)
    return full if os.path.isfile(full) else None


def read_log(run_name, max_bytes=262144):
    """读一个 run 的 log.txt 尾部 max_bytes 字节（防浏览器被巨大日志撑爆）。

    返回 {"text", "truncated", "total_bytes"} 或 None（run/log 不存在）。
    旧式平铺日志: run_name 也接受 `XXX_log.txt` 形态, 直接读那个文件。
    """
    path = None
    if _LEGACY_RE.match(run_name or "") and _safe_name(run_name):
        cand = os.path.join(LOG_DIR, run_name)
        path = cand if os.path.isfile(cand) else None
    else:
        d = run_dir_of(run_name)
        if d:
            cand = os.path.join(d, "log.txt")
            path = cand if os.path.isfile(cand) else None
    if path is None:
        return None
    total = os.path.getsize(path)
    truncated = total > max_bytes
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        if truncated:
            f.seek(total - max_bytes)
            text = f.read()
            # 从第一个换行开始, 别输出半行
            nl = text.find("\n")
            text = text[nl + 1:] if nl >= 0 else text
        else:
            text = f.read()
    return {"text": text, "truncated": truncated, "total_bytes": total}


# ==================== 删除单次运行（控制台右键） ====================

def delete_run(name):
    """删除**一次**运行: run 目录整棵（含其下 frames/）/ 旧式平铺日志那一个文件。

    活动运行一律拒绝 —— 理由同 cleanup: main.py 的 fd 还攥着那个 log.txt, 删了之后
    日志会写进孤儿 inode 里丢掉, 页面上看起来「日志突然不动了」。

    返回 {"ok": True, "freed_bytes": N} 或 {"ok": False, "error": "..."}。
    """
    if not _safe_name(name):
        return {"ok": False, "error": f"名字不合法: {name}"}
    pid, active_dir = read_pid_file()
    if _RUN_RE.match(name):
        full = run_dir_of(name)
        if full is None:
            return {"ok": False, "error": f"没有这个运行: {name}"}
        if active_dir and _pid_alive(pid) and \
                os.path.abspath(full) == os.path.abspath(active_dir):
            return {"ok": False, "error": "这是正在运行的那一次, 不能删"}
        freed = _tree_size(full)
        try:
            shutil.rmtree(full)
        except OSError as e:
            return {"ok": False, "error": f"删除失败: {e}"}
        return {"ok": True, "freed_bytes": freed}
    if _LEGACY_RE.match(name):
        full = os.path.join(LOG_DIR, name)
        if not os.path.isfile(full):
            return {"ok": False, "error": f"没有这个日志: {name}"}
        freed = os.path.getsize(full)
        try:
            os.remove(full)
        except OSError as e:
            return {"ok": False, "error": f"删除失败: {e}"}
        return {"ok": True, "freed_bytes": freed}
    return {"ok": False, "error": f"名字不合法: {name}"}


# ==================== 清理（今日 / 最近7天 / 全部） ====================

def cleanup(scope):
    """删除历史日志。scope ∈ {"today", "week", "all"}。

    · today = 删除「今天开始之前」创建的（即只保留今天的）;
    · week  = 只保留最近 7 天;
    · all   = 全部历史（但**活动 run 一律跳过**——正在写的文件删了,
      main.py 的 fd 还攥着, 日志会写进孤儿 inode 里丢掉）。
    附带清理: 旧式平铺 *_log.txt、logs/log_pics/、logs/Difficult/（自研失败留档）、
    以及 main.pid 之外的散落 *_last.jpg **不删**（Web 端点还在读, 删了页面空图）。
    返回 {"deleted": N, "freed_bytes": N, "kept": [...]}。
    """
    now = time.time()
    if scope == "today":
        cutoff = now - _seconds_since_midnight()
    elif scope == "week":
        cutoff = now - 7 * 86400
    elif scope == "all":
        cutoff = None
    else:
        return {"deleted": 0, "freed_bytes": 0, "kept": [], "error": f"未知 scope: {scope}"}

    pid, active_dir = read_pid_file()
    active_alive = active_dir is not None and _pid_alive(pid)
    deleted, freed, kept = 0, 0, []

    def _rm(path):
        nonlocal deleted, freed
        try:
            if os.path.isdir(path):
                freed += _tree_size(path)
                shutil.rmtree(path)
            elif os.path.isfile(path):
                freed += os.path.getsize(path)
                os.remove(path)
            deleted += 1
        except OSError as e:
            print(f"[WARN] 日志清理: 删 {path} 失败: {e}")

    for r in list_runs():
        # all 也保留活动 run; 旧式平铺没有"活动"概念, 照删
        if r["active"]:
            kept.append(r["name"])
            continue
        if cutoff is not None and r["mtime"] >= cutoff:
            continue          # 在保留窗口内
        _rm(r["dir"] if r["is_run_dir"] else r["log_path"])

    # 附带的目录（没有"活动"问题, 按同一时间窗删）
    for sub in ("log_pics", "Difficult"):
        p = os.path.join(LOG_DIR, sub)
        if not os.path.exists(p):
            continue
        if cutoff is not None and os.path.getmtime(p) >= cutoff:
            continue
        _rm(p)

    return {"deleted": deleted, "freed_bytes": freed, "kept": kept,
            "active_kept": active_alive}


def _seconds_since_midnight():
    lt = time.localtime()
    return lt.tm_hour * 3600 + lt.tm_min * 60 + lt.tm_sec
