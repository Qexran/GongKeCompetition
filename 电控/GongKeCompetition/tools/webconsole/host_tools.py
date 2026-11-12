#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
宿主联机工具 — 「调试助手」Tab 的后端逻辑。

移植自 pro/调试助手/调试助手.py（PyQt5 单机版），去掉 GUI 只留动作函数：
  · 本机 IP 探测 + 树莓派 IP 推导（本机前三段 + .125）
  · 一键更新 ~/.ssh/config 与资源管理器「网络位置」
  · 远程桌面 / 远程文件夹(Samba) / Trae 与 VS Code 远程连接 / 树莓派 Web 控制台
  · 常用语读写与动态字段解析（最新日志路径）

与原版的两处有意差异：
  1. 生成 .lnk 原用 pywin32 的 Dispatch("WScript.Shell")，这里改调 PowerShell 的
     WScript.Shell COM（同为系统自带）—— 目的是让 webconsole 保持 stdlib-only，
     勿顺手换回 pywin32。
  2. 改 ~/.ssh/config 前先备份 config.bak（原版原地改、无备份），与仓库既有的
     map.c.bak / app_script.h.bak 习惯一致。
  3. extract_unc_from_lnk 补上 UTF-16 分支（原版只扫 ANSI, 在现代 Windows 上恒返回
     None, 详见该函数注释）。

另需留意（原版既有行为, 未改）：do_update 会把 ~/.ssh/config 里**每个** IPv4 的第三段
都换成当前本机网段, 末段保持不变 —— 不止树莓派那一条。config.bak 现在能兜住误改。

本模块只被 server.py 调用；所有函数都不得在导入时产生副作用（导入即读磁盘/联网会
拖慢服务启动，也让离线测试变难）。
"""

import json
import os
import re
import shutil
import socket
import subprocess
import webbrowser
from pathlib import Path

APP_DIR = Path(__file__).resolve().parent
SSH_CONFIG = Path.home() / ".ssh" / "config"
SSH_CONFIG_BAK = SSH_CONFIG.parent / (SSH_CONFIG.name + ".bak")
NET_SHORTCUTS = Path.home() / "AppData" / "Roaming" / "Microsoft" / "Windows" / "Network Shortcuts"
PHRASES_FILE = APP_DIR / "phrases.json"
DESKTOP_INI = "[.ShellClassInfo]\nCLSID2={0AFACED1-E828-11D1-9187-B532F1E9575D}\nFlags=2\n"

# 树莓派 IP = 本机 IP 前三段 + 该后缀
PI_HOST_SUFFIX = 125
PI_SSH_USER = "pi"
PI_SSH_HOME = "/home/pi"

# ⚠️ Pi 上放 logcat 的目录（相对 \\<pi>\shared）。cv_python 是 pro 项目的 Pi 工程名，
#    本仓库对应的 Pi 端工程是 GKS_Competition —— 换机器/换工程时只改这一行。
PI_SHARE_LOG_DIR = r"cv_python\logs"

TRAE_EXE = Path.home() / "AppData" / "Local" / "Programs" / "Trae CN" / "Trae CN.exe"

# VS Code 常见安装位置（本机实测在 %LOCALAPPDATA%），都没有就退回 PATH 里的 code
VSCODE_CANDIDATES = (
    Path.home() / "AppData" / "Local" / "Programs" / "Microsoft VS Code" / "Code.exe",
    Path("C:/Program Files/Microsoft VS Code/Code.exe"),
    Path("C:/Program Files (x86)/Microsoft VS Code/Code.exe"),
)

# 树莓派上的 Web 控制台端口
PI_CONSOLE_PORT = 9000

# **本**控制台的端口（端口常量的唯一来源: server.py 的 PORT 取自这里, 这里要拼地址）。
CONSOLE_PORT = 8410

# 不弹黑窗（非 Windows 上该常量不存在，取 0 以便本模块能被导入做离线测试）
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


# =====================================================================
# 本机 / 树莓派 地址
# =====================================================================

def get_local_ip():
    """探测本机对外 IP（UDP connect 不实际发包，只让内核选路由）。"""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except OSError:
        return socket.gethostbyname(socket.gethostname())


def ip_prefix(ip):
    return ".".join(ip.split(".")[:3])


def detect_pi_ip(prefix):
    return "%s.%d" % (prefix, PI_HOST_SUFFIX)


def pi_share_unc(pi_ip):
    """树莓派 Samba 共享的 UNC 路径。"""
    return "\\\\%s\\shared" % pi_ip


def pi_console_url(pi_ip):
    """树莓派 Web 控制台地址。"""
    return "http://%s:%d/" % (pi_ip, PI_CONSOLE_PORT)


def console_url():
    """**本机**这台控制台的局域网地址 —— 发给队友让他们照抄的那个串。

    服务端绑 0.0.0.0（见 server.py），所以本机 IP 就是队友要输的地址。探测失败回 None
    （调用方别拿它去拼字符串）。"""
    try:
        return "http://%s:%d/" % (get_local_ip(), CONSOLE_PORT)
    except OSError:
        return None


def host_info():
    """→ {local_ip, prefix, pi_ip, ssh, share, console, console_url}。
    每次调用实时探测, 换网后无需重启服务。"""
    ip = get_local_ip()
    prefix = ip_prefix(ip)
    pi_ip = detect_pi_ip(prefix)
    return {"local_ip": ip, "prefix": prefix, "pi_ip": pi_ip,
            "ssh": "%s@%s" % (PI_SSH_USER, pi_ip),
            "share": pi_share_unc(pi_ip), "console": pi_console_url(pi_ip),
            "console_url": console_url()}


def replace_ip_in_path(path, new_prefix):
    """把路径里的 IP 第三段换成 new_prefix, 末段保持不变。"""
    return re.sub(r"\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b",
                  lambda m: "%s.%s" % (new_prefix, m.group(0).split(".")[3]),
                  path)


# =====================================================================
# 进程 / 快捷方式 原语
# =====================================================================

def _spawn(argv):
    """后台拉起一个进程（不阻塞、不弹窗）。→ (ok, err)"""
    try:
        subprocess.Popen(argv, creationflags=NO_WINDOW)
        return True, ""
    except Exception as e:                                        # noqa: BLE001
        return False, str(e)


def _ps(command):
    """跑一段 PowerShell（不弹窗），等它结束。→ (returncode, stderr)

    ⚠️ 两处编码坑（bt_serial._ps_stdout 已踩过并修，这里跟着对齐）:
      1. 先让 PowerShell 自己按 UTF-8 输出 —— 否则中文（设备名、路径）在管道里按 ANSI 码页出去;
      2. subprocess 必须显式 encoding="utf-8" —— 默认 text=True 用系统码页（本机 GBK）解码,
         遇到非 GBK 字节会在**子进程读取线程里**抛 UnicodeDecodeError 并被吞掉,
         表现就是「报错文案一直是空的」, 很难查。errors="replace" 兜住最后一道。
    """
    ps = "[Console]::OutputEncoding=[System.Text.Encoding]::UTF8; " + command
    try:
        p = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
            capture_output=True, text=True, encoding="utf-8", errors="replace",
            timeout=30, creationflags=NO_WINDOW,
        )
        return p.returncode, (p.stderr or "").strip()
    except subprocess.TimeoutExpired:
        return -1, "PowerShell 超时(30s)"
    except Exception as e:                                        # noqa: BLE001
        return -1, str(e)


def _ps_quote(s):
    """转义进 PowerShell 单引号字符串（内部 ' 需写成 ''）。"""
    return str(s).replace("'", "''")


def _attrib(path, *flags):
    """attrib 改文件属性。失败无所谓（属性本就没设时 attrib 会返回非 0）。"""
    try:
        subprocess.run(["attrib", *flags, str(path)],
                       capture_output=True, creationflags=NO_WINDOW)
    except Exception:                                             # noqa: BLE001
        pass


def _remove_attrs(path):
    """递归去掉 系统/隐藏 属性 —— 否则 rmtree / 改名会被 Windows 拒绝。"""
    for root, dirs, files in os.walk(path):
        for name in files + dirs:
            _attrib(os.path.join(root, name), "-S", "-H")
    _attrib(path, "-S", "-H")


RE_UNC_ASCII = re.compile(rb"\\\\\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\\[^\x00]+")
RE_UTF16_RUN = re.compile(rb"(?:[\x20-\x7e]\x00){6,}")


def extract_unc_from_lnk(lnk_path):
    r"""从 .lnk 二进制里抠出 UNC 目标（无则 None）。

    ⚠️ 两种编码都要试。原工具只扫 ANSI: 目标主机**不可达**时 Windows 建的 .lnk 里压根
    没有 ANSI 副本（只有 UTF-16, 实测 pywin32 与 PowerShell 生成的字节完全相同）, 于是
    恒返回 None → do_update 的「已存在则跳过」与改名两个分支永远走不到, 每跑一次就在旁边
    多建一个目录、旧的留在原地。可达时才有 ANSI 副本, 所以原工具是「连得上树莓派时才
    碰巧正常」。

    返回的共享名大小写取决于 Windows 的规范化（可达时按服务端真实名, 如 \\h\shared →
    \\h\\SHARED）, do_update 只替换 IP 段, 不受影响。"""
    try:
        data = Path(lnk_path).read_bytes()
    except Exception:                                             # noqa: BLE001
        return None

    m = RE_UNC_ASCII.search(data)
    if m:
        return m.group(0).decode("ascii", errors="ignore")

    for run in RE_UTF16_RUN.finditer(data):                       # UTF-16-LE 串
        text = run.group(0).decode("utf-16-le", errors="ignore")
        # 只认「IP\共享名」这一段再把前缀 \\ 补回去: .lnk 里这个串前面可能跟一段相对路径
        # (..\..\..\192.168.x.x\shared), 单双反斜杠不固定, 写死 \\\\ 会匹配不到。
        um = re.search(r"\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\\\S+", text)
        if um:
            return "\\\\" + um.group(0)
    return None


def _create_net_shortcut(old_entry, new_name, unc_path, log):
    """新建/替换一个「网络位置」快捷方式目录, 并删掉旧目录。→ ok"""
    new_dir = NET_SHORTCUTS / new_name
    if new_dir.exists():
        _remove_attrs(new_dir)
        shutil.rmtree(new_dir, ignore_errors=True)
    new_dir.mkdir(parents=True, exist_ok=True)
    (new_dir / "desktop.ini").write_text(DESKTOP_INI, encoding="utf-8")

    lnk = new_dir / "target.lnk"
    rc, err = _ps("$s = (New-Object -ComObject WScript.Shell).CreateShortcut('%s');"
                  " $s.TargetPath = '%s'; $s.Save()"
                  % (_ps_quote(lnk), _ps_quote(unc_path)))
    if rc != 0:
        log("    ✗ 快捷方式创建失败: %s\n" % (err or ("PowerShell 退出码 %d" % rc)))
        return False

    if old_entry is not None:
        _remove_attrs(old_entry)
        shutil.rmtree(old_entry, ignore_errors=True)

    _attrib(new_dir, "+S")
    return True


# =====================================================================
# 一键更新 SSH config + 网络位置
# =====================================================================

def do_update(prefix, log, dry_run=False):
    """把 SSH config 与全部网络位置里的 IP 第三段换成 prefix。→ ok"""
    # ---- 1) ~/.ssh/config ----
    if SSH_CONFIG.exists():
        text = SSH_CONFIG.read_text(encoding="utf-8")
        hit = []

        def replacer(m):
            ip = m.group(0)
            new_ip = "%s.%s" % (prefix, ip.split(".")[3])
            if new_ip != ip:
                hit.append(True)
            return new_ip

        new_text = re.sub(r"\b\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3}\b", replacer, text)
        if hit:
            if dry_run:
                log("  [DRY-RUN] SSH config 将更新（备份 → %s）\n" % SSH_CONFIG_BAK.name)
            else:
                shutil.copy2(SSH_CONFIG, SSH_CONFIG_BAK)
                SSH_CONFIG.write_text(new_text, encoding="utf-8")
                log("  SSH config 已更新（备份 → %s）\n" % SSH_CONFIG_BAK)
        else:
            log("  SSH config 无需更新\n")
    else:
        log("  SSH config 不存在，跳过\n")

    # ---- 2) 资源管理器「网络位置」 ----
    pi_ip = detect_pi_ip(prefix)
    default_unc = "\\\\%s\\shared" % pi_ip
    found = False
    NET_SHORTCUTS.mkdir(parents=True, exist_ok=True)

    for entry in sorted(NET_SHORTCUTS.iterdir()):
        if not entry.is_dir():
            continue
        lnk = entry / "target.lnk"
        if not lnk.exists():
            continue
        old_unc = extract_unc_from_lnk(lnk)
        if old_unc is None:
            continue

        new_unc = replace_ip_in_path(old_unc, prefix)
        if new_unc == old_unc:
            log("  网络位置 '%s' 无需更新，已存在\n" % entry.name)
            found = True
            continue

        new_name = replace_ip_in_path(entry.name, prefix)
        if dry_run:
            log("  [DRY-RUN] 将更新: %s → %s\n" % (entry.name, new_name))
        elif _create_net_shortcut(entry, new_name, new_unc, log):
            log("  网络位置已更新: %s → %s\n" % (entry.name, new_name))
        log("    UNC: %s → %s\n" % (old_unc, new_unc))
        found = True

    if not found:
        folder_name = "shared (%s (Samba))" % pi_ip
        if dry_run:
            log("  [DRY-RUN] 将创建: %s\n" % folder_name)
            log("    UNC: %s\n" % default_unc)
        elif _create_net_shortcut(None, folder_name, default_unc, log):
            log("  网络位置已创建: %s\n" % folder_name)
            log("    UNC: %s\n" % default_unc)

    log("[完成] 更新完毕\n" if not dry_run else "[完成] 预览完毕（未改动任何文件）\n")
    return True


# =====================================================================
# 远程桌面 / 远程文件夹 / Trae
# =====================================================================

def do_remote_desktop(pi_ip, log):
    log("  目标: %s\n" % pi_ip)
    ok, err = _spawn(["mstsc", "/v:%s" % pi_ip])
    log("  远程桌面已启动\n" if ok else "  启动失败: %s\n" % err)
    return ok


def do_open_remote_folder(pi_ip, log):
    unc = pi_share_unc(pi_ip)
    log("  路径: %s\n" % unc)
    try:
        os.startfile(unc)                                       # noqa: S606 (Windows 专有)
        log("  资源管理器已打开\n")
        return True
    except Exception as e:                                        # noqa: BLE001
        log("  打开失败: %s\n" % e)
        return False


def do_open_trae(pi_ip, log):
    log("  远程: %s@%s\n" % (PI_SSH_USER, pi_ip))
    log("  目录: %s\n" % PI_SSH_HOME)
    if not TRAE_EXE.exists():
        log("  未找到 Trae: %s\n" % TRAE_EXE)
        return False
    ok, err = _spawn([str(TRAE_EXE), "--remote",
                      "ssh-remote+%s@%s" % (PI_SSH_USER, pi_ip), PI_SSH_HOME])
    log("  Trae 已启动\n" if ok else "  启动失败: %s\n" % err)
    return ok


def find_vscode():
    """VS Code 可执行文件路径（都没有则 None）。"""
    for p in VSCODE_CANDIDATES:
        if p.exists():
            return p
    return shutil.which("code") or shutil.which("code.cmd")


def do_open_vscode(pi_ip, log):
    """VS Code 以 Remote-SSH 连到树莓派（与 Trae 同一套 --remote 参数）。"""
    remote = "%s@%s" % (PI_SSH_USER, pi_ip)
    log("  远程: %s\n" % remote)
    log("  目录: %s\n" % PI_SSH_HOME)
    exe = find_vscode()
    if not exe:
        log("  未找到 VS Code（找过 Program Files / %%LOCALAPPDATA%%, PATH 里也没有 code）\n")
        return False
    log("  可执行: %s\n" % exe)
    ok, err = _spawn([str(exe), "--remote", "ssh-remote+%s" % remote, PI_SSH_HOME])
    log("  VS Code 已启动\n" if ok else "  启动失败: %s\n" % err)
    return ok


def do_open_pi_console(pi_ip, log):
    """在默认浏览器打开树莓派上的 Web 控制台。"""
    url = pi_console_url(pi_ip)
    log("  地址: %s\n" % url)
    try:
        webbrowser.open(url)
        log("  已在默认浏览器打开\n")
        return True
    except Exception as e:                                        # noqa: BLE001
        log("  打开失败: %s\n" % e)
        return False


# =====================================================================
# 常用语
# =====================================================================

def load_phrases():
    if PHRASES_FILE.exists():
        try:
            data = json.loads(PHRASES_FILE.read_text(encoding="utf-8"))
            return data.get("phrases", [])
        except Exception:                                         # noqa: BLE001
            pass
    return []


def save_phrases(phrases):
    """整表覆盖写。先写临时文件再 os.replace —— 直接覆盖若中途失败会毁掉用户的常用语。"""
    tmp = PHRASES_FILE.with_suffix(".json.tmp")
    tmp.write_text(json.dumps({"phrases": phrases}, ensure_ascii=False, indent=2),
                   encoding="utf-8")
    os.replace(tmp, PHRASES_FILE)


def find_latest_log(pi_ip):
    """在 Samba 共享里找最新的 logcat 日志（找不到 → None）。
    两条路都试: Python Path API 与 cmd dir —— 后者能触发 Windows 对 Guest 共享的认证。"""
    log_dir_unc = "\\\\%s\\shared\\%s" % (pi_ip, PI_SHARE_LOG_DIR)
    pattern = "logcat_*.log"

    try:
        log_dir = Path(log_dir_unc)
        if log_dir.exists():
            files = sorted(log_dir.glob(pattern),
                           key=lambda p: p.stat().st_mtime, reverse=True)
            if files:
                return str(files[0])
    except Exception:                                             # noqa: BLE001
        pass

    try:
        result = subprocess.run(
            ["cmd", "/c", 'dir "%s\\%s" /b /o-d 2>nul' % (log_dir_unc, pattern)],
            capture_output=True, text=True, timeout=10, creationflags=NO_WINDOW,
        )
        lines = [l.strip() for l in result.stdout.splitlines() if l.strip()]
        if lines:
            return "%s\\%s" % (log_dir_unc, lines[0])
    except Exception:                                             # noqa: BLE001
        pass

    return None


def resolve_phrase_text(phrase, pi_ip):
    """解析常用语文本: 展开 dynamic=latest_log 与 {pi_ip}。"""
    text = phrase.get("text", "")
    if phrase.get("dynamic") == "latest_log":
        latest = find_latest_log(pi_ip)
        if latest:
            text = "树莓派端最后一次运行的log是 %s" % latest
        else:
            pattern = "\\\\%s\\shared\\%s\\logcat_*.log" % (pi_ip, PI_SHARE_LOG_DIR)
            text = "树莓派端最后一次运行的log是 %s（无法自动定位最新，请手动选择）" % pattern
    return text.replace("{pi_ip}", pi_ip)


# =====================================================================
# 动作分发（server.py 唯一入口）
# =====================================================================

def run_action(action, payload, log):
    """执行一个「调试助手」动作 → (ok, text)。log 为收行长文本的 list.append。

    text 仅在 action == 'phrase' 时有值（解析后的常用语文本, 由前端写剪贴板）。"""
    info = host_info()

    if action == "update":
        return do_update(info["prefix"], log, bool(payload.get("dry_run"))), ""
    if action == "rdp":
        return do_remote_desktop(info["pi_ip"], log), ""
    if action == "folder":
        return do_open_remote_folder(info["pi_ip"], log), ""
    if action == "trae":
        return do_open_trae(info["pi_ip"], log), ""
    if action == "vscode":
        return do_open_vscode(info["pi_ip"], log), ""
    if action == "console":
        return do_open_pi_console(info["pi_ip"], log), ""
    if action == "phrase":
        text = resolve_phrase_text(payload.get("phrase") or {}, info["pi_ip"])
        # 真正写剪贴板的是浏览器, 这里只把解析过程报给日志区
        log("[常用语] %s\n" % (text[:120] + "..." if len(text) > 120 else text))
        return True, text

    log("  未知动作: %s\n" % action)
    return False, ""
