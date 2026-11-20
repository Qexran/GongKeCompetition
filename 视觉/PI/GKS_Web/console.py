# console.py — 控制台后端（网络 / 服务 / 电源 / 系统状态）
#
# 移植自 GKS_Competition/control_panel.py 的非 GUI 逻辑（去掉 PyQt），
# 供 app.py 的 HTTP 路由调用。所有 sudo 命令依赖用户 xixi 的 NOPASSWD 权限。
import os
import re
import socket
import subprocess
import threading
import time
import uuid

from webconfig import (GKS_SERVICE, WEB_SERVICE, GKS_COMPETITION_DIR,
                       FIX_DEVICE_SCRIPT, NMCLI, NM_KEYFILE_DIR, WIFI_IFACE,
                       WIFI_TOP_PRIORITY, WIFI_SWITCH_WAIT, WIFI_VERIFY_GRACE,
                       WIFI_SCAN_MIN_INTERVAL, WIFI_HINT_URL)


# ---- 基础 ----
def run_cmd(cmd, timeout=5):
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True,
                           text=True, timeout=timeout)
        return p.stdout.strip(), p.stderr.strip(), p.returncode
    except subprocess.TimeoutExpired:
        return "", "timeout", -1
    except Exception as e:
        return "", str(e), -1


# ---- 网络 ----
def get_ip_addresses():
    ips = []
    try:
        out = subprocess.check_output(["hostname", "-I"], text=True).strip()
        ips = [ip for ip in out.split()
               if ip.count(".") == 3 and not ip.startswith("127.")]
    except Exception:
        pass
    if not ips:
        try:
            out = subprocess.check_output(["ip", "-4", "-br", "addr"], text=True)
            for line in out.strip().split("\n"):
                parts = line.split()
                for p in parts:
                    if "/" in p and p.count(".") == 3:
                        ip = p.split("/")[0]
                        if not ip.startswith("127."):
                            ips.append(ip)
        except Exception:
            pass
    return ips if ips else ["no ip"]


def get_hostname():
    return socket.gethostname()


# ==== WiFi（NetworkManager / nmcli）====
#
# ⚠ 四条铁律, 改这里之前先读:
#  1) 一律用 _nm() 的 **argv 列表** 调 nmcli, **绝不用上面的 run_cmd()**（它是 shell=True）:
#     SSID 与密码都可能含 shell 元字符（`"; rm -rf ~` 是合法 SSID）, 拼进命令串就是注入;
#     而且 subprocess 超时的异常文本**带完整命令行**, 会顺着 journal 把密码写进日志。
#  2) LC_ALL 必须 C.UTF-8: 用 C 会把 `有线连接 1` 这类非 ASCII 名字变成 `????`;
#     用默认的 zh_CN 又只能拿到中文报错, 没法匹配。C.UTF-8 = 英文报错 + 名字完整。
#  3) 密码只经 keyfile 走 stdin（_write_root_file）, **不进 argv、不进日志、不回给浏览器**。
#  4) 写操作一律 sudo -n: polkit 里 network-control 对普通用户是"需鉴权", 而 systemd 服务
#     不是 loginctl 会话、判定可能更严; xixi 的 sudo 免密, 所以直接用最省事也最稳。
_NM_ENV = dict(os.environ, LC_ALL="C.UTF-8", LANG="C.UTF-8")

_wifi_lock = threading.Lock()          # 单飞: 一次只允许一个切换流程（worker 结束时才释放）
_wifi_state = {"phase": "idle", "target": "", "prev": "", "reason": "",
               "ip": "", "started": 0, "finished": 0}
_wifi_state_lock = threading.Lock()
_last_scan_ts = 0.0

_REDACT = re.compile(r"(?i)(psk|password|passwd|wifi-sec\.psk)\s*[:=]?\s*\S+")


def redact(text):
    """给任何要进日志/回给浏览器的串兜底。"""
    return _REDACT.sub(r"\1=<已隐藏>", text or "")


def _sudo(argv, timeout=10, stdin_text=None):
    """以 root 跑一条 argv（**不经 shell**）。返回 (stdout, stderr, rc)。"""
    try:
        p = subprocess.run(["sudo", "-n"] + list(argv), capture_output=True,
                           text=True, input=stdin_text, timeout=timeout, env=_NM_ENV)
        return p.stdout.strip(), p.stderr.strip(), p.returncode
    except subprocess.TimeoutExpired:
        return "", "timeout", 3
    except Exception as e:                                    # noqa: BLE001
        return "", type(e).__name__, -1


def _nm(argv, timeout=10, stdin_text=None):
    return _sudo([NMCLI] + list(argv), timeout=timeout, stdin_text=stdin_text)


def wifi_available():
    if not os.path.exists(NMCLI):
        return False, "nmcli 不存在（这台机器不是 NetworkManager 管理网络）"
    _, err, rc = _nm(["general", "status"], timeout=5)
    if rc != 0:
        return False, "NetworkManager 未运行: " + (err or "nmcli rc=%d" % rc)
    return True, ""


# ---- 扫描 ----
def wifi_hint_url():
    """换网后找回页面的地址（靠 mDNS, 与 IP 无关）。"""
    return WIFI_HINT_URL % get_hostname()


def _auth_kind(security):
    s = (security or "").upper()
    if not s:
        return "open"                       # 开放网络: 不需要密码
    if "802.1X" in s or "EAP" in s:
        return "enterprise"                 # 要用户名/证书, 本页不支持
    if "WEP" in s:
        return "wep"
    return "psk"                            # WPA/WPA2/WPA3 个人版


def _parse_scan(out):
    """解析 `nmcli -t` 的扫描输出。

    ⚠ 别按 man 说的"会被转义"去写: 实测 nmcli 1.52.1 的 -t **不转义**值里的冒号
    （`-g` 才转义）。SSID 是第一个字段且可能含 `:`, 后面三个字段（SIGNAL/SECURITY/IN-USE）
    都保证不含冒号 → 所以从右往左 rsplit 3 次是精确的。
    """
    rows = []
    for line in out.splitlines():
        if not line:
            continue
        parts = line.rsplit(":", 3)
        if len(parts) != 4:
            continue
        ssid, signal, security, in_use = parts
        ssid = ssid.replace("\\:", ":").replace("\\\\", "\\")   # 万一将来版本又转义了
        try:
            sig = int(signal or 0)
        except ValueError:
            sig = 0
        rows.append({"ssid": ssid, "signal": sig, "security": security.strip(),
                     "in_use": in_use.strip() == "*"})
    return rows


def wifi_scan(rescan=True):
    """扫描附近 WiFi → (ok, {networks:[...], rescan_limited:bool} | 错误文案)。

    按 SSID 去重取最强信号。NM 对重扫有限流, 间隔内直接回缓存并置 rescan_limited。
    """
    global _last_scan_ts
    ok, msg = wifi_available()
    if not ok:
        return False, msg
    limited = False
    mode = "no"
    if rescan:
        if (time.time() - _last_scan_ts) >= WIFI_SCAN_MIN_INTERVAL:
            _last_scan_ts = time.time()
            mode = "auto"
        else:
            limited = True
    # ⚠ 用 list 自带的 --rescan（**不要**先 `dev wifi rescan` 再 list）:
    #   `dev wifi rescan` 是**异步**的、立刻返回, 紧接着 list 只会拿到"扫了一半"的
    #   结果（实测只剩当前那一个 AP）; 而 `list --rescan auto|yes` 会等扫描完成再打印。
    out, err, rc = _nm(["-t", "-f", "SSID,SIGNAL,SECURITY,IN-USE",
                        "dev", "wifi", "list", "--rescan", mode], timeout=25)
    if rc != 0 and not out:
        return False, "扫描失败: " + (err or "nmcli rc=%d" % rc)
    best = {}
    for r in _parse_scan(out):
        if not r["ssid"]:                                       # 隐藏 SSID 的空行
            continue
        cur = best.get(r["ssid"])
        if cur is None or r["signal"] > cur["signal"]:
            if cur:
                r["in_use"] = r["in_use"] or cur["in_use"]
            best[r["ssid"]] = r
        else:
            cur["in_use"] = cur["in_use"] or r["in_use"]
    nets = sorted(best.values(), key=lambda x: -x["signal"])
    for r in nets:
        r["auth"] = _auth_kind(r["security"])
        r["secure"] = r["auth"] != "open"
    return True, {"networks": nets, "rescan_limited": limited}


# ---- 档案 / 当前连接 ----
def _active_wifi():
    """当前活动的 WiFi 连接名（没有则 None）。"""
    out, _, rc = _nm(["-t", "-f", "NAME,DEVICE,TYPE", "con", "show", "--active"], timeout=8)
    if rc != 0:
        return None
    for line in out.splitlines():
        parts = line.rsplit(":", 2)
        if len(parts) == 3 and parts[2] == "802-11-wireless":
            return parts[0]
    return None


def _wlan_ip():
    out, _, _ = _nm(["-t", "-f", "IP4.ADDRESS", "dev", "show", WIFI_IFACE], timeout=8)
    for line in out.splitlines():
        v = line.split(":", 1)[-1].strip()
        if v and v.count(".") == 3:
            return v.split("/")[0]
    return ""


def _wlan_gateway():
    out, _, _ = _nm(["-t", "-f", "IP4.GATEWAY", "dev", "show", WIFI_IFACE], timeout=8)
    for line in out.splitlines():
        v = line.split(":", 1)[-1].strip()
        if v.count(".") == 3:
            return v
    return ""


def _wlan_connected_to(name):
    """wlan0 是否已连到这个档案 —— 成功判据的一半。"""
    return bool(name) and _active_wifi() == name


def _profile_filename(name):
    """档案的 keyfile 路径。

    ⚠ `-f FILENAME` 只在**列表形式**（`con show`）有效, 单档案形式
    （`con show id <name>`）会直接报 `invalid field 'FILENAME'` 并 rc=2 —— 踩过。
    （`-g 802-11-wireless.ssid` 那种"设置字段"两种形式都认, 所以它没事。）
    """
    out, _, rc = _nm(["-t", "-f", "NAME,FILENAME", "con", "show"], timeout=10)
    if rc != 0 and not out:
        return ""
    for line in out.splitlines():
        parts = line.rsplit(":", 1)                 # NAME 可含冒号 → 从右切一次
        if len(parts) == 2 and parts[0] == name:
            return parts[1].strip()
    return ""


def _has_stored_psk(name, fname=None):
    """档案里到底有没有存着密码（`Mi 10 Ultra` 就是 autoconnect=yes 但没有 psk 的坑）。

    只看"有没有 psk= 这一行", **绝不取出它的值** —— 别用 nmcli -s/--show-secrets。
    """
    fname = fname or _profile_filename(name)
    if not fname:
        return False
    out, _, rc = _sudo(["grep", "-c", "^psk=", fname], timeout=8)
    try:
        return rc == 0 and int(out.strip() or 0) > 0
    except ValueError:
        return False


def wifi_profiles():
    """所有 WiFi 档案（含开机优先级）。另附每条的 SSID 与"是否存了密码"。"""
    out, err, rc = _nm(["-t", "-f", "NAME,TYPE,AUTOCONNECT,AUTOCONNECT-PRIORITY,"
                        "ACTIVE,DEVICE,FILENAME", "con", "show"], timeout=12)
    if rc != 0 and not out:
        return []
    items = []
    for line in out.splitlines():
        parts = line.rsplit(":", 6)         # NAME 可含冒号 → 从右往左切 6 次
        if len(parts) != 7:
            continue
        name, ctype, auto, prio, active, device, fname = parts
        if ctype != "802-11-wireless":
            continue
        ssid, _, _ = _nm(["-g", "802-11-wireless.ssid", "con", "show", "id", name], timeout=8)
        items.append({"name": name,
                      "ssid": ssid.strip() or name,
                      "autoconnect": auto == "yes",
                      "priority": int(prio) if prio.lstrip("-").isdigit() else 0,
                      "active": active == "yes",
                      "device": device,
                      "has_secret": _has_stored_psk(name, fname),
                      "persistent": fname.startswith("/etc/"),
                      "filename": fname})
    items.sort(key=lambda x: -x["priority"])
    return items


def wifi_status():
    """「网络」页签的一次性快照（当前连接 + 档案表 + 切换状态）。"""
    ok, msg = wifi_available()
    host = get_hostname()
    st = {"available": ok, "error": "" if ok else msg,
          "ssid": "", "active_name": "", "device": WIFI_IFACE,
          "ip": _wlan_ip(), "gateway": _wlan_gateway(),
          "hostname": host, "hint_url": WIFI_HINT_URL % host,
          "ips": get_ip_addresses(), "profiles": [], "switch": wifi_switch_state()}
    if not ok:
        return st
    active = _active_wifi()
    st["active_name"] = active or ""
    profiles = wifi_profiles()
    st["profiles"] = profiles
    for p in profiles:
        if p["name"] == active:
            st["ssid"] = p["ssid"]
            break
    return st


# ---- 档案的增删改（密码走 keyfile, 不经 argv）----
def _slug(text):
    """SSID → 安全文件名。**路径里绝不能出现用户输入原文**（这是唯一会被拼进路径的地方）。"""
    s = re.sub(r"[^A-Za-z0-9._-]", "_", text or "")
    return (s[:40] or "net")


def _write_root_file(path, content):
    """以 root 写文件: 内容走 **stdin**（`sudo tee`）, 不经过 argv/shell, 再收紧权限。"""
    out, err, rc = _sudo(["tee", path], timeout=10, stdin_text=content)
    if rc != 0:
        print(f"[WIFI] 写 {path} 失败 (rc={rc}, {redact(err)[:120]})")
        return False
    _sudo(["chmod", "600", path], timeout=8)
    _sudo(["chown", "root:root", path], timeout=8)
    return True


def _wifi_value_ok(v):
    """SSID/密码能否安全写进 keyfile（2026-10 加）。

    NetworkManager 的 keyfile 是 INI 形态: 值里带换行就能注入任意配置节
    （dns= / addresses= / [ipv4] method=manual ...）—— argv 面堵死了, 文件内容面
    之前没堵。入口只允许 printable 且无换行/回车。"""
    return bool(v) and "\r" not in v and "\n" not in v and v.isprintable()


def _create_profile(ssid, password, priority):
    """新建档案（keyfile + reload）。成功返回档案名（= SSID）, 失败 None。"""
    if not _wifi_value_ok(ssid):
        return None
    if password and not _wifi_value_ok(password):
        return None
    path = os.path.join(NM_KEYFILE_DIR, "gks-%s.nmconnection" % _slug(ssid))
    lines = ["[connection]",
             "id=%s" % ssid,
             "uuid=%s" % uuid.uuid4(),
             "type=wifi",
             "interface-name=%s" % WIFI_IFACE,
             "autoconnect=true",
             "autoconnect-priority=%d" % priority,
             "",
             "[wifi]",
             "mode=infrastructure",
             "ssid=%s" % ssid,
             ""]
    if password:
        lines += ["[wifi-security]", "key-mgmt=wpa-psk", "psk=%s" % password, ""]
    lines += ["[ipv4]", "method=auto", "", "[ipv6]", "addr-gen-mode=default",
              "method=auto", ""]
    if not _write_root_file(path, "\n".join(lines)):
        return None
    _nm(["con", "reload"], timeout=15)
    # ⚠ 存在性检查必须用**列表形式**（`con show`）:
    #   `con show id <name>` 是"看单个档案的设置", `-f` 只认设置字段(connection.id/…) ——
    #   给它 `-f NAME` 会直接 `invalid field 'NAME'` + rc=2, 于是明明建好了却被判失败。
    #   （同一个坑在 FILENAME 上已经踩过一次, 见 _profile_filename。）
    names = _nm(["-t", "-f", "NAME", "con", "show"], timeout=10)[0]
    if ssid not in names.splitlines():
        _sudo(["rm", "-f", path], timeout=8)        # 别留垃圾 keyfile
        _nm(["con", "reload"], timeout=15)
        print(f"[WIFI] 新建的档案没被 NetworkManager 接受, 已丢弃: {path}")
        return None
    return ssid


def _set_psk(name, password):
    """给已有档案补/换密码（改 keyfile 里的 psk= 行）。"""
    if not _wifi_value_ok(password):
        print("[WIFI] 密码含换行/不可打印字符, 拒绝写入 keyfile")
        return False
    path = _profile_filename(name)
    if not path:
        return False
    text, err, rc = _sudo(["cat", path], timeout=8)
    if rc != 0:
        print(f"[WIFI] 读 {name} 的 keyfile 失败 (rc={rc}, {redact(err)[:120]})")
        return False
    if "[wifi-security]" in text:
        if re.search(r"(?m)^psk=", text):
            text = re.sub(r"(?m)^psk=.*$", lambda m: "psk=" + password, text)
        else:
            text = text.replace("[wifi-security]",
                                "[wifi-security]\nkey-mgmt=wpa-psk\npsk=" + password, 1)
    else:
        text = text.rstrip("\n") + "\n\n[wifi-security]\nkey-mgmt=wpa-psk\npsk=%s\n" % password
    if not _write_root_file(path, text):
        return False
    _nm(["con", "reload"], timeout=15)
    return True


def _delete_profile(name):
    out, err, rc = _nm(["con", "delete", "id", name], timeout=15)
    if rc != 0:
        print(f"[WIFI] 删除档案 {name} 失败 (rc={rc}, {redact(err)[:120]})")
    return rc == 0


def _apply_prefer(name, others_zero=True):
    """把某档案设为开机首选, 其余 WiFi 归零（有线不动 —— 它是另一个设备, 不受影响）。"""
    for p in wifi_profiles():
        if p["name"] == name:
            _nm(["con", "mod", "id", name, "connection.autoconnect", "yes"], timeout=12)
            _nm(["con", "mod", "id", name,
                 "connection.autoconnect-priority", str(WIFI_TOP_PRIORITY)], timeout=12)
        elif others_zero and p["priority"] != 0:
            _nm(["con", "mod", "id", p["name"],
                 "connection.autoconnect-priority", "0"], timeout=12)


def prefer_wifi(name):
    """把一条档案设为开机首选（其余 WiFi 归零, 有线不动）。"""
    if not name:
        return False, "缺少档案名"
    _apply_prefer(name)
    return True, "已把「%s」设为开机首选（其余 WiFi 已归零）" % name


def set_wifi_priority(name, priority):
    """改一条档案的开机优先级（页面上的手动微调）。"""
    try:
        prio = int(priority)
    except (TypeError, ValueError):
        return False, "优先级必须是整数"
    if not -999 <= prio <= 999:
        return False, "优先级要在 -999 ~ 999 之间"
    out, err, rc = _nm(["con", "mod", "id", name,
                        "connection.autoconnect-priority", str(prio)], timeout=12)
    if rc != 0:
        return False, "改优先级失败: " + (redact(err) or "nmcli rc=%d" % rc)
    return True, "「%s」的开机优先级已设为 %d" % (name, prio)


def delete_wifi_profile(name):
    """删档案 —— 四道闸: 只能删 WiFi、不能删当前活动的、不能删最后一个、不能删 lo。"""
    if not name or name == "lo":
        return False, "不能删这个档案"
    profiles = wifi_profiles()
    target = next((p for p in profiles if p["name"] == name), None)
    if target is None:
        return False, "没找到档案「%s」" % name
    if target["active"]:
        return False, "「%s」正在使用中 —— 先切到别的网络再删（删了就把自己断开了）" % name
    if len(profiles) <= 1:
        return False, "这是最后一条 WiFi 档案，删了就再也连不上了"
    ok = _delete_profile(name)
    return (True, "已删除「%s」" % name) if ok else (False, "删除失败（看日志）")


# ---- 切换: 先应答, 再动手; 失败自动切回 ----
def wifi_switch_state():
    with _wifi_state_lock:
        return dict(_wifi_state)


def _set_state(**kw):
    with _wifi_state_lock:
        _wifi_state.update(kw)


def _map_nm_error(rc, err):
    """把 nmcli 的报错翻成人话。

    ⚠ 实测这里必须**同时按退出码和文本**判: rc=4 既可能是"密码不对"（secrets were
    required）也可能是"这个网络不在范围内"（the wi-fi network could not be found）,
    光看 rc 分不出来; 而文本措辞又和 man 里的示例不完全一样。别删任何一条匹配。
    """
    e = (err or "").lower()
    if "secrets were required" in e or "secret" in e or "密码" in e:
        return "密码错误（或该网络需要额外认证）"
    if ("could not be found" in e or "not found" in e
            or "no network with ssid" in e or "no such connection" in e or rc == 10):
        return "找不到该网络（可能不在范围内，或 SSID 敲错了）"
    if "timeout expired" in e or rc == 3:
        return "超时（信号弱或拿不到 IP）"
    if "networkmanager is not running" in e or rc == 8:
        return "NetworkManager 未运行"
    if "not authorized" in e or "权限" in e:
        return "没权限执行（polkit/sudo 配置被改了？）"
    return "切换失败（nmcli rc=%d）%s" % (rc, redact(e)[:120])


def begin_wifi_switch(ssid, password, secure):
    """发起切换。**立刻返回**, 真正的切换在后台线程里（见 _wifi_worker）。

    这里同步做完"不改网络"的那部分（建/改档案、设首选）, 交给 worker 的只有激活与验证。
    """
    ssid = (ssid or "").strip()
    if not ssid:
        return False, "SSID 不能为空"
    if secure and not password:
        return False, "这个网络需要密码"
    ok, msg = wifi_available()
    if not ok:
        return False, msg
    if not _wifi_lock.acquire(blocking=False):
        return False, "已有切换任务在进行中，等它结束再试"
    started = False
    try:
        prev = _active_wifi()
        profiles = wifi_profiles()
        # 先记下当前各自的优先级: 万一切失败要**原样还原**, 不能替用户重新决定
        # "谁当开机首选"（比如把手机热点设成 100 —— 正是他想避开的）
        prev_prios = {p["name"]: p["priority"] for p in profiles}
        target = None
        created = ""
        for p in profiles:                              # 已有同 SSID 的档案就直接用
            if p["ssid"] == ssid:
                target = p["name"]
                if secure and password:
                    _set_psk(target, password)
                break
        if target is None:
            target = _create_profile(ssid, password if secure else None,
                                     WIFI_TOP_PRIORITY)
            if target is None:
                return False, "建立网络配置失败（看日志；可能是 SSID 里有特殊字符）"
            created = target
        _apply_prefer(target)                           # 其余 WiFi 归零（用户要求）
        _set_state(phase="activating", target=ssid, target_name=target, prev=prev or "",
                   created=created, reason="", ip="", started=time.time(), finished=0)
        threading.Thread(target=_wifi_worker, args=(prev, target, created, prev_prios),
                         daemon=True).start()
        started = True
        return True, ("正在切换到「%s」… 本页面会断开，请连上新网络后用 %s 打开"
                      % (ssid, WIFI_HINT_URL % get_hostname()))
    finally:
        if not started:                                 # 没起成 worker 就把锁还回去
            _wifi_lock.release()


def _restore_priorities(prev_prios):
    """把优先级还原成切换前的样子（只动还存在的档案）。"""
    for name, prio in (prev_prios or {}).items():
        if any(p["name"] == name for p in wifi_profiles()):
            _nm(["con", "mod", "id", name,
                 "connection.autoconnect-priority", str(prio)], timeout=12)


def _wifi_worker(prev, target_name, created, prev_prios):
    """后台切换 + 验证 + 失败自动切回。**它负责释放 _wifi_lock**。"""
    try:
        time.sleep(1.0)             # 先让上面的 HTTP 响应发出去（同 restart_web_service 的道理）
        out, err, rc = _nm(["--wait", str(WIFI_SWITCH_WAIT), "con", "up", "id", target_name],
                           timeout=WIFI_SWITCH_WAIT + 15)
        reason = "" if rc == 0 else _map_nm_error(rc, err)
        print("[WIFI] 激活 %s → rc=%s" % (target_name, rc))
        # 验证: 光"关联上"不算成功 —— 必须真的拿到 IPv4 地址（密码错/卡在 DHCP 都会在这栽）
        # rc=4(密码/密钥被拒) 与 rc=10(没这个网络) 是**确定性**失败, 不必等宽限期;
        # rc=3(超时) 则可能只是慢, 照样给足宽限期。
        hard_fail = rc in (4, 10)
        deadline = time.time() + WIFI_VERIFY_GRACE
        while not hard_fail:
            if _wlan_connected_to(target_name) and _wlan_ip():
                _set_state(phase="ok", ip=_wlan_ip(), gateway=_wlan_gateway(),
                           reason="", finished=time.time())
                print("[WIFI] 切换成功: %s (%s)" % (target_name, _wlan_ip()))
                return
            if time.time() >= deadline:
                break
            time.sleep(1.5)
        # ---- 失败: 删掉本次新建的档案（用户要求）, 再切回原来那个 ----
        _set_state(phase="reverting", reason=reason or "切换后没连上（可能是密码错或信号太弱）")
        print("[WIFI] 切换失败: %s" % (reason or "没连上"))
        if created:
            _delete_profile(created)
        # 无论目标档案是新建的还是原有的, 都把优先级梯队还原成切换前的样子
        # （本次的 _apply_prefer 动过它们）—— 只在目标**没有**连上时才走到这
        _restore_priorities(prev_prios)
        if prev and prev != target_name:
            _nm(["--wait", "30", "con", "up", "id", prev], timeout=45)
            if _wlan_connected_to(prev):
                _set_state(phase="reverted", finished=time.time())
                print("[WIFI] 已自动切回: %s" % prev)
            else:
                _set_state(phase="failed", finished=time.time())
                print("[WIFI] 自动切回也失败了 —— 需要物理接触这台设备")
        else:
            _set_state(phase="failed", finished=time.time())
    except Exception as e:                                   # noqa: BLE001
        _set_state(phase="failed", reason="内部错误: %s" % redact(str(e)),
                   finished=time.time())
        print("[WIFI] worker 异常: %s" % redact(str(e)))
    finally:
        _wifi_lock.release()


def wifi_revert():
    """手动/兜底切回上一个网络。**幂等**: 目标已经活动时什么都不做。"""
    st = wifi_switch_state()
    target, prev = st.get("target_name", ""), st.get("prev", "")
    if target and _wlan_connected_to(target):
        return False, "已经连在「%s」上了, 不用切回" % st.get("target", target)
    if not prev:
        return False, "不知道要切回哪个网络（没有记录到切换前的连接）"
    if _wlan_connected_to(prev):
        return True, "已经回到「%s」了" % prev
    if not _wifi_lock.acquire(blocking=False):
        return False, "有切换任务在进行中，等它结束"
    try:
        _nm(["--wait", "30", "con", "up", "id", prev], timeout=45)
        ok = _wlan_connected_to(prev)
        _set_state(phase="reverted" if ok else "failed", finished=time.time())
        return ok, ("已切回「%s」" % prev) if ok else "切回「%s」失败" % prev
    finally:
        _wifi_lock.release()


# ---- 服务状态 / 控制（gks.service = GKS 主程序）----
def is_service_active():
    _, _, rc = run_cmd(f"systemctl is-active --quiet {GKS_SERVICE}")
    return rc == 0


def is_service_enabled():
    _, _, rc = run_cmd(f"systemctl is-enabled --quiet {GKS_SERVICE}")
    return rc == 0


def start_service():
    _, stderr, rc = run_cmd(f"sudo systemctl start --no-block {GKS_SERVICE}")
    return rc == 0, stderr


def stop_service():
    _, stderr, rc = run_cmd(f"sudo systemctl stop --no-block {GKS_SERVICE}")
    return rc == 0, stderr


def enable_service():
    _, stderr, rc = run_cmd(f"sudo systemctl enable {GKS_SERVICE}")
    return rc == 0, stderr


def disable_service():
    _, stderr, rc = run_cmd(f"sudo systemctl disable {GKS_SERVICE}")
    return rc == 0, stderr


def restart_service():
    """重启 gks.service（未运行则直接启动）。

    2026-10: 加 --no-block（与 stop 同口径）—— 阻塞式 restart 在 main.py 停止收尾
    超 5s（run_cmd 默认超时）时页面会报"timeout 失败"而 systemctl 其实仍在后台
    跑成, 提示与事实相反; no-block 立即返回, 结果交给页面状态轮询看真值。"""
    _, stderr, rc = run_cmd(f"sudo systemctl restart {GKS_SERVICE} --no-block")
    return rc == 0, stderr


# ---- 本 Web 控制端自身（gks-web.service）----
# ⚠ 与上面 gks.service 的区别：这几个操作的目标就是**本进程自己**。
#   改完 app.py / console.py 等源码后，正是靠这里重启才会生效。
def is_web_service_active():
    _, _, rc = run_cmd(f"systemctl is-active --quiet {WEB_SERVICE}")
    return rc == 0


def is_web_service_enabled():
    _, _, rc = run_cmd(f"systemctl is-enabled --quiet {WEB_SERVICE}")
    return rc == 0


def enable_web_service():
    _, stderr, rc = run_cmd(f"sudo systemctl enable {WEB_SERVICE}")
    return rc == 0, stderr


def disable_web_service():
    _, stderr, rc = run_cmd(f"sudo systemctl disable {WEB_SERVICE}")
    return rc == 0, stderr


def restart_web_service():
    """重启本 Web 控制端自身（gks-web.service）。

    必须「延迟 + 脱离会话」执行：systemctl restart 会立刻杀掉本进程，若同步等待，
    HTTP 响应还没发出去进程就没了，浏览器只会看到连接中断（且分不清是重启还是崩了）。
    故用 start_new_session 起一个独立后台 shell，先 sleep 1s 让本响应发完，再重启。

    返回 (ok, message)；ok=True 只表示「已安排重启」，不代表重启已成功。
    """
    try:
        subprocess.Popen(
            f"sleep 1; sudo systemctl restart {WEB_SERVICE}",
            shell=True, start_new_session=True,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return True, "Web 控制端将在 1 秒后重启，页面会自动恢复"
    except Exception as e:
        return False, f"安排重启失败: {e}"


# ---- 设备固定（udev 符号链接）----
def fix_devices(timeout=30):
    """运行设备固定脚本，重新生成并应用 udev 规则（摄像头/串口 → 固定符号链接）。

    非交互模式（--apply）：按当前检测到的设备自动生成规则 + reload + trigger。
    ⚠ 脚本按「检测到的先后顺序」命名摄像头，调用方必须先确认两个摄像头都已插入，
    否则只插一个时它会被命名成 camera_main，与 camera_Qr 角色对调。

    返回 (ok, 输出文本)；输出含检测详情与符号链接验证结果，可直接展示给用户。
    """
    if not os.path.exists(FIX_DEVICE_SCRIPT):
        return False, f"脚本不存在: {FIX_DEVICE_SCRIPT}"
    out, err, rc = run_cmd(f"sudo bash {FIX_DEVICE_SCRIPT} --apply", timeout=timeout)
    text = out + (("\n" + err) if err else "")
    return rc == 0, (text.strip() or "(无输出)")


# ---- 直接启停 main.py（调试模式，不走 systemd）----
def is_main_py_running():
    """是否有 GKS_Competition/main.py 进程在运行（不限启动方式）。

    匹配「python3 … GKS_Competition/main.py」的**命令行形态**（2026-10 修）:
    旧版 pgrep -f "GKS_Competition/main.py" 是子串匹配 —— SSH 里开着的
    vim/tail -f/grep/cp 该文件的进程全都会中招（被 pkill 误杀, 或让"启动"被误拒）。
    """
    try:
        result = subprocess.run(
            ["pgrep", "-f", r"python3(\.[0-9]+)?(-u)?\s.*GKS_Competition/main\.py"],
            capture_output=True, text=True, timeout=3)
        return result.returncode == 0
    except Exception:
        return False


def start_main_py():
    """直接后台启动 main.py（调试模式，无自动重启）。"""
    if is_main_py_running():
        return False, "main.py 已在运行中"
    try:
        # 2026-10: 启动期(import 阶段/装 TeeWriter 之前)的崩溃不再 DEVNULL 蒸发 ——
        # 落一份到 logs/mainpy_debug.log, 现场排"点了没反应"有迹可循。
        log_path = os.path.join(GKS_COMPETITION_DIR, "logs", "mainpy_debug.log")
        try:
            os.makedirs(os.path.dirname(log_path), exist_ok=True)
            dbg = open(log_path, "ab", buffering=0)
            dbg.write(("\n===== %s start =====\n" % time.strftime("%F %T")).encode())
        except OSError:
            dbg = subprocess.DEVNULL
        subprocess.Popen(
            ["python3", os.path.join(GKS_COMPETITION_DIR, "main.py")],
            cwd=GKS_COMPETITION_DIR,
            stdout=dbg, stderr=subprocess.STDOUT,
            start_new_session=True)
        return True, "main.py 已在后台启动（调试模式：无自动重启; 启动期输出见 logs/mainpy_debug.log）"
    except Exception as e:
        return False, f"启动失败: {e}"


def kill_main_py():
    """终止 main.py。先停 systemd 防止 Restart=always 自动复活。"""
    if not is_main_py_running():
        return False, "main.py 本来就没有在运行"
    if is_service_active():
        _, stderr, rc = run_cmd(f"sudo systemctl stop {GKS_SERVICE}", timeout=15)
        if rc != 0:
            print(f"[WARN] systemd 停止异常: {stderr}")
    try:
        subprocess.run(["pkill", "-f",
                        r"python3(\.[0-9]+)?(-u)?\s.*GKS_Competition/main\.py"], timeout=5)
    except subprocess.TimeoutExpired:
        pass
    if is_main_py_running():
        return False, "main.py 进程可能未能终止，请重试"
    return True, "main.py 已终止（systemd 已停止，手动进程已清理）"


def restart_main_py():
    """重启 main.py（调试模式）：先彻底终止现有进程，再启动新的。"""
    kill_main_py()
    # 等旧进程完全退出（最多 3 秒）
    for _ in range(30):
        if not is_main_py_running():
            break
        time.sleep(0.1)
    return start_main_py()


# ---- 系统状态 ----
def get_system_status():
    status = {}

    # CPU 温度
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            status["cpu_temp"] = round(int(f.read().strip()) / 1000.0, 1)
    except Exception:
        status["cpu_temp"] = None

    # 负载
    try:
        with open("/proc/loadavg") as f:
            a, b, c = f.read().split()[:3]
        status["load"] = {"1min": float(a), "5min": float(b), "15min": float(c)}
    except Exception:
        status["load"] = None

    # 内存
    try:
        mem = {}
        with open("/proc/meminfo") as f:
            for line in f:
                if line.startswith("MemTotal:"):
                    mem["total_kb"] = int(line.split()[1])
                elif line.startswith("MemAvailable:"):
                    mem["avail_kb"] = int(line.split()[1])
        if "total_kb" in mem and "avail_kb" in mem:
            mem["used_kb"] = mem["total_kb"] - mem["avail_kb"]
        status["mem"] = mem if "total_kb" in mem else None
    except Exception:
        status["mem"] = None

    # 磁盘
    try:
        out = subprocess.check_output(["df", "-h", "/"], text=True)
        cols = out.strip().split("\n")[-1].split()
        status["disk"] = {"fs": cols[0], "size": cols[1], "used": cols[2],
                          "avail": cols[3], "use%": cols[4]}
    except Exception:
        status["disk"] = None

    # 运行时长
    try:
        with open("/proc/uptime") as f:
            status["uptime_sec"] = int(float(f.read().split()[0]))
    except Exception:
        status["uptime_sec"] = None

    return status


# ---- 电源 ----
def reboot():
    out, err, rc = run_cmd("sudo shutdown -r now")
    return rc == 0, err


def shutdown():
    out, err, rc = run_cmd("sudo shutdown -h now")
    return rc == 0, err


# ---- 日志 ----
def get_log(unit, lines=200):
    out, _, _ = run_cmd(
        f"journalctl -u {unit} -n {lines} --no-pager --output=cat", timeout=10)
    return out
