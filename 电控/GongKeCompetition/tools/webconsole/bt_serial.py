"""蓝牙串口传输层 —— ctypes 直调 Win32，保住 tools/webconsole 的 stdlib-only 约定。

为什么不引 pyserial：CLAUDE.md 写明本目录 stdlib-only（见 host_tools.py 顶部那条
「故意走 PowerShell 而不引 pywin32」的先例）。Python 标准库没有串口模块，而 Windows
的 COM 口不能当普通文件读写，所以这里用 ctypes 调 kernel32。

本模块**只管传输**（枚举 / 打开 / 读写 / 关闭），协议与业务在 bt_link.py。
在非 Windows 上 import 不报错（AVAILABLE=False），这样单元测试在哪都能跑。

⚠️ 一条铁律：**ReadFile 只能有一个属主**。bt_link.BtLink 的后台读线程独占它，
   别在别处再调 read()（会跟读线程抢字节，日志空洞且抓不到响应终止符）。
"""

import ctypes
import ctypes.wintypes as wt
import re
import subprocess
import sys
import threading
import time

ON_WINDOWS = sys.platform == "win32"

# host_tools 已有同样的守卫写法：非 Windows 上 import 也要能过
_NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)


class SerialError(Exception):
    """串口打开/读写失败。message 已带中文提示，可直接显示给用户。"""


# ==================== Win32 常量 ====================

GENERIC_READ = 0x80000000
GENERIC_WRITE = 0x40000000
OPEN_EXISTING = 3
INVALID_HANDLE_VALUE = wt.HANDLE(-1).value if ON_WINDOWS else -1

ERROR_ACCESS_DENIED = 5
ERROR_FILE_NOT_FOUND = 2
ERROR_OPERATION_ABORTED = 995
ERROR_BAD_COMMAND = 22
# 1168 = "找不到元素"。蓝牙口**建链没跑完**时 CreateFileW 回这个，与 2
# （端口根本不存在）是两回事 —— 见 SerialPort.open() 的重试说明与 _elem_not_found_hint。
ERROR_NOT_FOUND = 1168

PURGE_TXABORT = 0x0001
PURGE_RXABORT = 0x0002
PURGE_TXCLEAR = 0x0004
PURGE_RXCLEAR = 0x0008

# DCB 里那 32 位标志字的位序（见 Win32 DCB 定义）
DCB_FBINARY = 1 << 0        # fBinary = 1（串口必须为二进制模式）
DCB_FDTR_MASK = 0x3 << 4    # fDtrControl 是 2 位字段（位 4-5）
DCB_FRTS_MASK = 0x3 << 12   # fRtsControl 是 2 位字段（位 12-13）
DCB_FDTR_ENABLE = 1 << 4    # fDtrControl = 1 (ENABLE)
DCB_FRTS_ENABLE = 1 << 12   # fRtsControl = 1 (ENABLE)


if ON_WINDOWS:
    class DCB(ctypes.Structure):
        """Win32 DCB —— 用平铺字段而非 ctypes 位域，布局好核对（sizeof 必须 28）。

        第 8 字节那个 32 位标志字（fBinary/fDtrControl/fRtsControl/…）整体当
        dwFlags 处理，用上面的 DCB_F* 常量按位操作。位域在 ctypes 里的对齐
        容易出微妙的错，平铺字段 + 断言更保险。
        """
        _fields_ = [
            ("DCBlength",  wt.DWORD),      # 0
            ("BaudRate",   wt.DWORD),      # 4
            ("dwFlags",    wt.DWORD),      # 8  fBinary/fParity/…/fRtsControl 打包
            ("wReserved",  wt.WORD),       # 12
            ("XonLim",     wt.WORD),       # 14
            ("XoffLim",    wt.WORD),       # 16
            ("ByteSize",   wt.BYTE),       # 18
            ("Parity",     wt.BYTE),       # 19
            ("StopBits",   wt.BYTE),       # 20
            ("XonChar",    ctypes.c_char),  # 21
            ("XoffChar",   ctypes.c_char),  # 22
            ("ErrorChar",  ctypes.c_char),  # 23
            ("EofChar",    ctypes.c_char),  # 24
            ("EvtChar",    ctypes.c_char),  # 25
            ("wReserved1", wt.WORD),       # 26
        ]

    class COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout",         wt.DWORD),
            ("ReadTotalTimeoutMultiplier",  wt.DWORD),
            ("ReadTotalTimeoutConstant",    wt.DWORD),
            ("WriteTotalTimeoutMultiplier", wt.DWORD),
            ("WriteTotalTimeoutConstant",   wt.DWORD),
        ]

    if ctypes.sizeof(DCB) != 28:                      # 布局错了后面全错，早炸早好
        raise RuntimeError("DCB 布局异常: sizeof=%d (应为 28)" % ctypes.sizeof(DCB))

    _k32 = ctypes.WinDLL("kernel32", use_last_error=True)

    _k32.CreateFileW.restype = wt.HANDLE
    _k32.CreateFileW.argtypes = [wt.LPCWSTR, wt.DWORD, wt.DWORD, ctypes.c_void_p,
                                 wt.DWORD, wt.DWORD, wt.HANDLE]
    _k32.CloseHandle.argtypes = [wt.HANDLE]
    _k32.CloseHandle.restype = wt.BOOL
    _k32.GetCommState.argtypes = [wt.HANDLE, ctypes.POINTER(DCB)]
    _k32.GetCommState.restype = wt.BOOL
    _k32.SetCommState.argtypes = [wt.HANDLE, ctypes.POINTER(DCB)]
    _k32.SetCommState.restype = wt.BOOL
    _k32.SetCommTimeouts.argtypes = [wt.HANDLE, ctypes.POINTER(COMMTIMEOUTS)]
    _k32.SetCommTimeouts.restype = wt.BOOL
    _k32.PurgeComm.argtypes = [wt.HANDLE, wt.DWORD]
    _k32.PurgeComm.restype = wt.BOOL
    # 注意 dwDesiredAccess 是 DWORD：默认 argtypes(c_int) 下 0xC0000000 会抛
    # OverflowError: int too long to convert，所以上面每个 DWORD 都显式声明了。
    _k32.ReadFile.argtypes = [wt.HANDLE, ctypes.c_void_p, wt.DWORD,
                              ctypes.POINTER(wt.DWORD), ctypes.c_void_p]
    _k32.ReadFile.restype = wt.BOOL
    _k32.WriteFile.argtypes = [wt.HANDLE, ctypes.c_void_p, wt.DWORD,
                               ctypes.POINTER(wt.DWORD), ctypes.c_void_p]
    _k32.WriteFile.restype = wt.BOOL

    AVAILABLE = True
else:
    _k32 = None
    AVAILABLE = False


def _last_error_text(code):
    try:
        return ctypes.FormatError(code).strip()
    except Exception:                                        # noqa: BLE001
        return "错误码 %d" % code


# ==================== 设备枚举 ====================

_friendly_cache = {}          # {port: 友好名}
_friendly_cache_at = 0.0
_FRIENDLY_TTL = 30.0          # PowerShell 冷启 0.5~1.5s，别每次 GET /api/bt 都拉
_ports_lock = threading.Lock()


def _enum_registry():
    """从注册表读「**驱动已加载**」的 COM 口 → [(port, device_path)]。

    纯 winreg，快。但注意这只是**就绪**的那一批，不能拿来当完整列表用 —— 见
    _query_pnp_ports 的说明（蓝牙 SPP 口在链路未连接时不在这个键里）。
    """
    out = []
    if not ON_WINDOWS:
        return out
    try:
        import winreg                                        # Windows-only
        with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                            r"HARDWARE\DEVICEMAP\SERIALCOMM") as key:
            idx = 0
            while True:
                try:
                    name, value, _ = winreg.EnumValue(key, idx)
                except OSError:
                    break
                out.append((str(value), str(name)))
                idx += 1
    except OSError:
        pass
    return out


def _ps_stdout(command, timeout=20):
    """跑 PowerShell 并**返回 stdout**。

    不能复用 host_tools._ps —— 它 `return p.returncode, (p.stderr or "").strip()`
    把 stdout 丢掉了（design note 里明说只回 stderr），拿不到任何查询结果。

    ⚠️ 两处编码坑（都踩过）：
      1. 先让 PowerShell 自己按 UTF-8 输出（否则中文设备名在管道里按 ANSI 码页出去）；
      2. subprocess 必须显式 encoding="utf-8" —— 默认 text=True 用系统码页（本机 GBK）
         解码，遇到非 GBK 字节会在**子进程读取线程里**抛 UnicodeDecodeError 并被吞掉，
         表现就是「设备名一直是空的」，很难查。errors="replace" 兜住最后一道。
    """
    ps = ("[Console]::OutputEncoding=[System.Text.Encoding]::UTF8; " + command)
    try:
        proc = subprocess.run(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", ps],
            capture_output=True, timeout=timeout,
            encoding="utf-8", errors="replace",
            creationflags=_NO_WINDOW,
        )
        return proc.stdout or ""
    except Exception:                                        # noqa: BLE001
        return ""


def _query_pnp_ports():
    """{COM名: {'name','pnp','device','generic'}} —— 从 **PnP 设备树**枚举串口。

    一条 PowerShell 同时拿两批数据（省一次冷启）：
      P<TAB>端口友好名<TAB>PNPDeviceID
      D<TAB>已配对蓝牙设备名<TAB>设备 InstanceId

    ⚠️ 为什么必须以 PnP 为准、不能只看 SERIALCOMM / Win32_SerialPort / .NET 的
    GetPortNames()：**蓝牙 SPP 的虚拟串口只有在 RFCOMM 链路已连接时才会被注册进
    HARDWARE\\DEVICEMAP\\SERIALCOMM**。模块没上电/不在范围内时，设备在 PnP 树里明明
    存在（Class=Ports、状态 OK，例如 COM3/COM4/COM34/COM35），那三个来源却一个都
    看不到它 —— 于是变成「得先连上才能在列表里选，可要连上又得先在列表里选」的死循环。

    PNPDeviceID 里含蓝牙串口服务的 UUID（00001101-…），是判定蓝牙口最可靠的依据。
    再从中抠出 12 位蓝牙地址，就能反查**这个口属于哪台已配对设备** ——
    这是「4 个蓝牙口到底该选哪个」唯一的可靠答案（实测：4 个口里只有 1 个属于本车的
    模块，另一个是耳机的）。地址为全 0 的是**不挂任何设备**的通用传入口，永远连不上
    具体设备，界面要标出来别让用户白试。
    """
    ps = ('Get-CimInstance Win32_PnPEntity -Filter "PNPClass=\'Ports\'" | '
          'ForEach-Object { "P`t$($_.Name)`t$($_.PNPDeviceID)" }; '
          'Get-PnpDevice -Class Bluetooth -ErrorAction SilentlyContinue | '
          'Where-Object { $_.InstanceId -like \'BTHENUM\\DEV_*\' } | '
          'ForEach-Object { "D`t$($_.FriendlyName)`t$($_.InstanceId)" }')
    ports_raw, dev_names = {}, {}
    for line in _ps_stdout(ps).splitlines():
        parts = line.rstrip("\r").split("\t")
        if len(parts) < 3:
            continue
        kind, a, b = parts[0].strip(), parts[1].strip(), parts[2].strip()
        if kind == "D":
            m = re.search(r"DEV_([0-9A-Fa-f]{12})", b)
            if m:
                dev_names[m.group(1).upper()] = a
        elif kind == "P":
            m = re.search(r"\((COM\d+)\)", a)
            if m:
                ports_raw[m.group(1).upper()] = {
                    # 去掉 " (COMx)" 后缀，端口号由界面单独显示
                    "name": re.sub(r"\s*\(COM\d+\)\s*$", "", a).strip(),
                    "pnp": b.upper(),
                }

    out = {}
    for port, info in ports_raw.items():
        pnp = info["pnp"]
        m = re.search(r"&([0-9A-F]{12})_", pnp)
        addr = m.group(1).upper() if m else ""
        is_bt = "BTHENUM" in pnp
        generic = (addr == "") or (addr == "000000000000")
        info["device"] = "" if generic else dev_names.get(addr, "")
        info["addr"] = addr
        # generic 只对**蓝牙口**有意义（「不挂任何设备的通用传入口」）。
        # 普通 USB 串口本来就没有这个地址段，标它只会误导界面。
        info["generic"] = bool(is_bt and generic)
        # 方向：**按实例 ID 里的设备地址判**，就是上面算出的 generic：
        #   地址全 0（不挂任何设备）→ 传入(Incoming)，Windows 的通用监听口
        #   带真实设备地址          → 传出(Outgoing)，该设备的口，能主动建链
        # ⚠️ 判据改过一次：最初按 HardwareID 的 `_VID&…_PID&…` / `_LOCALMFG&…` 猜，
        #    结果把本车模块的传出口判成了传入口（在「蓝牙设置→更多蓝牙选项→COM 端口」
        #    的**方向**列上核对过：`LOCALMFG&0002` + 真实地址那个是**传出**）。
        #    这里**以设备地址为准** —— 它同时解释了「哪些口永远连不上」。
        # 注：手工「添加→传入」建出来的按设备传入口也会被这里标成传出，方向以 Windows
        #    对话框为准；但那种口打不开时没有任何实用差别（都是打开失败）。
        if is_bt:
            info["direction"] = "in" if generic else "out"
            # 全 0 地址必然是不挂设备的通用口，一并当 generic（上面已置位）
            if generic:
                info["generic"] = True
        else:
            info["direction"] = ""
        out[port] = info
    return out


def lookup_port(port):
    """从缓存里查某个口的字典（不打 PowerShell）。查不到回 None。"""
    if not port:
        return None
    port = str(port).upper()
    with _ports_lock:
        cached = dict(_friendly_cache)
    return cached.get(port)


def refresh_ports(force=False):
    """枚举串口 → [{'port','name','is_bt','ready'}]，按 COM 号排序。

    两个来源取并集（缺一不可）：
      · PnP 设备树（PowerShell，慢，带 30s 缓存）—— **全集**，含链路未连接的蓝牙口
      · SERIALCOMM 注册表（winreg，快）—— 驱动已加载的那批，据此标 ready
    拿不到 PnP（PowerShell 挂了/非 Windows）时退回只用注册表，界面照样能用。
    """
    global _friendly_cache, _friendly_cache_at
    now = time.time()
    with _ports_lock:
        stale = force or (now - _friendly_cache_at) > _FRIENDLY_TTL or not _friendly_cache
    if stale:
        # PowerShell 枚举最长 ~20s —— 挪到锁外跑（并发 GET /api/bt 不再排队串行）,
        # 只在换缓存引用的那一下进锁; 两客户端同时触发重复查询无害（结果同形）
        pnp = _query_pnp_ports()
        with _ports_lock:
            if pnp:
                _friendly_cache = pnp
            _friendly_cache_at = now
    with _ports_lock:
        pnp_ports = dict(_friendly_cache)

    ready = {str(port).upper() for port, _ in _enum_registry()}

    merged = {}
    for port, info in pnp_ports.items():
        merged[port] = dict(info)
    # 注册表里有、PnP 没枚举到的（罕见）也补上，宁可多列不可漏
    for port, devpath in _enum_registry():
        merged.setdefault(str(port).upper(),
                          {"name": "", "pnp": str(devpath).upper(),
                           "device": "", "generic": False})

    ports = []
    for port, info in merged.items():
        name = info.get("name", "") or ("串口 " + port)
        blob = (name + " " + info.get("pnp", "")).upper()
        ports.append({
            "port": port,
            "name": info.get("name", ""),
            # 判定蓝牙口：PNPDeviceID 里 BTHENUM\ 前缀最可靠（含 SPP 服务 UUID
            # 00001101-…）；名称/设备路径里的字样作兜底
            # （见过 \Device\BthModem0 与 \Device\BTPORT0，所以既查 BTH 也查 BTPORT）。
            "is_bt": ("BTHENUM" in blob) or ("BLUETOOTH" in blob)
                     or ("蓝牙" in name) or ("BTH" in blob) or ("BTPORT" in blob),
            # ready=驱动已加载（现在就打得开）。蓝牙口未连接链路时 ready=False，
            # 但仍要列出来 —— 打开它本身就会触发 RFCOMM 建链。
            "ready": port in ready,
            # 这个蓝牙口属于哪台已配对设备（界面显示，解决「4 个口该选哪个」）
            "device": info.get("device", ""),
            # "out"=传出(能主动连) / "in"=传入(只能被连, 打不开) / ""=非蓝牙
            "direction": info.get("direction", ""),
            # 通用传入口：不挂任何设备，永远连不上具体设备，界面要提示
            "generic": bool(info.get("generic", False)),
        })

    def _key(d):
        m = re.search(r"\d+", d["port"])
        return (int(m.group()) if m else 0, d["port"])

    ports.sort(key=_key)
    return ports


# ==================== 串口对象 ====================

def _any_bt_port_ready():
    """有没有**任何一个**蓝牙口真的进了 SERIALCOMM。

    进了就说明蓝牙串口驱动能把 DOS 设备名建出来（`\\\\.\\COMx` 可打开）；
    一个都没有 = 这台机器的蓝牙串口驱动是坏的/绑错了 INF（实测踩过：4 个蓝牙口
    PnP 里全是「存在 + 状态 OK + PortName 已分配」，`CreateFileW` 却一律
    ERROR_FILE_NOT_FOUND，连耳机的传出端口都打不开，而 USB 串口 COM8 正常）。
    纯 winreg，不启动 PowerShell，可安全用于报错路径。
    """
    try:
        return any(("BTH" in str(dv).upper()) or ("BTPORT" in str(dv).upper())
                   for _, dv in _enum_registry())
    except Exception:                                        # noqa: BLE001
        return True                                          # 查不到就别乱下结论

REINSTALL_HINT = ("这台机器的**蓝牙串口驱动**像是坏的：所有蓝牙口的 `\\\\\\\\.\\\\COMx` 都打不开"
                  "（连其它蓝牙设备的端口也一样），而普通 USB 串口正常。修法："
                  "`设备管理器 → 查看 → 显示隐藏的设备 → 端口(COM 和 LPT)` → "
                  "把所有「Standard Serial over Bluetooth link」逐个右键卸载（顺带删除驱动程序软件）"
                  "→ 扫描硬件改动 → 重新配对蓝牙设备。"
                  "另外可在 `蓝牙设置 → 更多蓝牙选项 → COM 端口 → 添加 → 传出` 手工建一个传出端口试试。")


def _not_found_hint(port):
    """CreateFileW 报 ERROR_FILE_NOT_FOUND 时的中文提示。

    这个错**不代表「蓝牙没配对」**。蓝牙口的可能原因，按概率排：
      · **蓝牙串口驱动坏了/绑错 INF** —— 所有蓝牙口一律打不开（含其它设备的口），
        但设备在 PnP 里显示「存在 + 状态 OK」。这是系统级问题，只能重装驱动（见 REINSTALL_HINT）；
      · 选中的是**传入(Incoming)**口 —— 传入口只能被动等对端连进来，应用程序打不开它；
      · 选中的是**通用传入口**（设备地址全 0），不挂任何设备，永远连不上谁；
      · 没建出**传出(Outgoing)**口 —— 去 `控制面板→设备和打印机→右键设备→属性→服务`
        勾「串行端口」可生成一个；
      · 设备没开机 / 不在范围内。
    提示里带上该口关联的设备名，用户一眼就知道是不是选错口了。
    """
    info = lookup_port(port)
    if info is None or not (info.get("name") or info.get("pnp")):
        return "%s 不存在（设备没插好？）" % port

    is_bt = ("BTHENUM" in str(info.get("pnp", "")).upper()) or ("蓝牙" in str(info.get("name", "")))
    if not is_bt:
        return "%s 不存在（设备没插好？）" % port

    # 系统级：一个蓝牙口都建不出来 → 先别让用户去折腾方向/服务，那是白费功夫
    if not _any_bt_port_ready():
        return "%s 打不开。%s" % (port, REINSTALL_HINT)

    dev = info.get("device") or ""
    who = ("（属于「%s」）" % dev) if dev else ""
    mk_out = ("请到 `控制面板 → 设备和打印机 → 右键该蓝牙设备 → 属性 → 服务`，"
              "勾选「串行端口」→ 确定，会新出现一个**传出**端口，用那个。")

    if info.get("generic"):
        return ("%s 是通用蓝牙传入口，不挂任何设备，连不上具体模块。" % port) + mk_out

    if info.get("direction") == "in":
        return ("%s%s 是**传入(Incoming)**端口，只能等对方连进来，程序打不开它。" % (port, who)
                + mk_out + "（也可在 蓝牙设置→更多蓝牙选项→COM 端口 看方向列核对。）")

    if info.get("direction") == "out":
        return ("%s%s 是传出端口，打不开通常是**设备没开机/不在范围内**。" % (port, who))

    return ("%s%s 打不开：%s" % (port, who, mk_out))


# 蓝牙口首次 CreateFileW 会**同步触发 RFCOMM 建链**，建链没跑完内核就回 1168。
# 给几次机会，别让「第一次点连接必失败」在用户眼里变成「又找不到设备了」。
# 4 次、每次 0.7s：最坏 ~2.1s，控制台一个 HTTP 请求完全等得起。
_BT_OPEN_RETRIES = 4
_BT_OPEN_RETRY_DELAY = 0.7


def _elem_not_found_hint(port):
    """CreateFileW 报 ERROR_NOT_FOUND(1168) 时的中文提示。

    1168 与 2 **不是同一回事**：2 是「这个口根本不存在/打不开」（传入口、通用传入口、
    驱动坏，见 _not_found_hint）；1168 是「口在，但这一次没连上对端」——蓝牙 SPP 的
    CreateFileW 是同步等 RFCOMM 建链的，模块刚上电、刚从省电里醒来、链路被系统回收
    之后，第一次调用常常在链路建好之前就返回了。

    所以正确处理是**稍等几秒再点一次**，不是去折腾方向/服务/重装驱动。
    SerialPort.open() 已经自动重试过；走到这里说明连试几次都没成，多半是对端没开机。
    """
    info = lookup_port(port) or {}
    # 传入口/通用传入口本来就打不开 —— 别让用户以为「再等等就好」，那两种要给结构性提示
    if info.get("direction") == "in" or info.get("generic"):
        return _not_found_hint(port)
    # 非蓝牙口没有「现场建链」这回事，1168 另有所指，别套蓝牙的说法误导人
    if info.get("direction") != "out":
        return "%s 打不开（设备没插好？）" % port
    dev = info.get("device") or ""
    who = ("「%s」" % dev) if dev else "对端设备"
    return ("%s 打不开：%s 没有应答（没开机 / 不在范围内 / 电量低）。"
            "蓝牙口每次都是现场建链，若刚给设备上电，稍等几秒再点一次「连接」通常就能连上。"
            % (port, who))


class SerialPort:
    """一个已打开的 Win32 串口。open() 失败抛 SerialError（消息可直接展示）。"""

    def __init__(self, port, baud=115200, read_timeout_ms=50):
        self.port = port
        self.baud = int(baud)
        self.read_timeout_ms = int(read_timeout_ms)
        self._h = None
        self._io_lock = threading.Lock()

    # ---- 生命周期 ----

    def open(self):
        if not AVAILABLE:
            raise SerialError("当前平台不支持串口（仅 Windows）")
        if self._h is not None:
            return

        # \\\\.\\COM7 形式：COM10 以上必须这么写，低号也兼容
        path = "\\\\.\\%s" % self.port
        code = 0
        for attempt in range(_BT_OPEN_RETRIES):
            h = _k32.CreateFileW(path, GENERIC_READ | GENERIC_WRITE,
                                 0,                # 独占：别的串口助手占着就报 ACCESS_DENIED
                                 None, OPEN_EXISTING, 0, None)
            if h != INVALID_HANDLE_VALUE and h is not None:
                break
            code = ctypes.get_last_error()

            # 1168 只说明「这一次没连上对端」，等一下再来（蓝牙口是现场建链的）。
            # ⚠️ 特意**不**重试 ERROR_FILE_NOT_FOUND(2)：那是传入口 / 通用传入口 /
            #    驱动坏那种**结构性**打不开（见 _not_found_hint），重试纯属浪费用户时间。
            if code == ERROR_NOT_FOUND and attempt < _BT_OPEN_RETRIES - 1:
                time.sleep(_BT_OPEN_RETRY_DELAY)
                continue

            if code == ERROR_ACCESS_DENIED:
                raise SerialError("%s 被占用（另一个串口助手/控制台还开着）" % self.port)
            if code == ERROR_FILE_NOT_FOUND:
                raise SerialError(_not_found_hint(self.port))
            if code == ERROR_NOT_FOUND:
                raise SerialError(_elem_not_found_hint(self.port))
            raise SerialError("打开 %s 失败: %s" % (self.port, _last_error_text(code)))

        try:
            self._setup(h)
        except Exception:
            _k32.CloseHandle(h)
            raise

        self._h = h

    def _setup(self, h):
        dcb = DCB()
        if not _k32.GetCommState(h, ctypes.byref(dcb)):
            raise SerialError("GetCommState 失败: %s" % _last_error_text(ctypes.get_last_error()))

        dcb.DCBlength = ctypes.sizeof(DCB)   # 某些驱动只认正确的 DCBlength
        dcb.BaudRate = self.baud
        dcb.ByteSize = 8
        dcb.Parity = 0                       # NOPARITY
        dcb.StopBits = 0                     # ONESTOPBIT
        # fBinary 必须为 1；DTR/RTS 必须拉起来，否则不少 USB 蓝牙/HC-05 模块
        # 「口打开了但完全没反应」——这是最经典的一个坑。
        # fDtrControl/fRtsControl 是 2 位字段，要先整体清掉再置 ENABLE(1)。
        flags = dcb.dwFlags | DCB_FBINARY
        flags &= ~(DCB_FDTR_MASK | DCB_FRTS_MASK)
        flags |= (DCB_FDTR_ENABLE | DCB_FRTS_ENABLE)
        dcb.dwFlags = flags

        if not _k32.SetCommState(h, ctypes.byref(dcb)):
            raise SerialError("SetCommState 失败: %s" % _last_error_text(ctypes.get_last_error()))

        # ReadIntervalTimeout=MAXDWORD + TotalTimeoutConstant=50：
        # 有数据立刻返回，没数据最多等 50ms —— 读线程可以放心循环轮询。
        to = COMMTIMEOUTS()
        to.ReadIntervalTimeout = 0xFFFFFFFF
        to.ReadTotalTimeoutMultiplier = 0
        to.ReadTotalTimeoutConstant = self.read_timeout_ms
        to.WriteTotalTimeoutMultiplier = 0
        to.WriteTotalTimeoutConstant = 200
        if not _k32.SetCommTimeouts(h, ctypes.byref(to)):
            raise SerialError("SetCommTimeouts 失败: %s" % _last_error_text(ctypes.get_last_error()))

        _k32.PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_TXABORT)

    def close(self):
        """关句柄。**持 _io_lock**, 见下面 read/write 的说明。

        读线程可能正阻塞在同一个句柄的 ReadFile 里; 不持锁就关会撞上「I/O 在飞时
        CloseHandle」——句柄值被复用后那次 ReadFile 可能读到别的东西或报假错, 极难查。
        最多等一个读超时（read_timeout_ms, 默认 50ms）—— 可接受, 因为 _io_lock 只被
        那一次 ReadFile/WriteFile 持有, 不会跨更长的区间。
        """
        with self._io_lock:
            h, self._h = self._h, None
            if h is not None:
                try:
                    _k32.CloseHandle(h)
                except Exception:                            # noqa: BLE001
                    pass

    @property
    def is_open(self):
        return self._h is not None

    # ---- 读写 ----

    def read(self, size=4096):
        """读至多 size 字节。**超时且无数据时返回 b''（不是错误）**。

        只有 ReadFile 失败才算掉线。⚠️ 这个口只能有一个读者，见模块顶部说明。
        ⚠️ 句柄**在锁内取**（并判空）: 在锁外判 self._h、锁内又读一次的话, 中间的 close()
           会把 None 塞进去 —— ReadFile(None) 报的是「无效句柄」, 看着像掉线, 其实只是关闭竞态。
        """
        buf = ctypes.create_string_buffer(size)
        nread = wt.DWORD(0)
        with self._io_lock:
            h = self._h
            if h is None:
                raise SerialError("串口未打开")
            ok = _k32.ReadFile(h, buf, size, ctypes.byref(nread), None)
        if not ok:
            code = ctypes.get_last_error()
            if code in (ERROR_OPERATION_ABORTED, ERROR_BAD_COMMAND):
                raise SerialError("%s 已断开（蓝牙掉了？）" % self.port)
            raise SerialError("读 %s 失败: %s" % (self.port, _last_error_text(code)))
        if nread.value == 0:
            return b""                                       # 超时无数据：不是错误
        return buf.raw[:nread.value]

    def write(self, data):
        """写全部 data。返回实际写出的字节数（= len(data)）。

        ⚠️ **循环到写完**: WriteFile 允许只写一部分（串口流控/驱动缓冲满时会发生）,
           老写法只看第一次的返回值且调用方不看它 —— 命令被静默截断, 而 _cmd_lock 那套
           请求/响应配对还在等一个永远不会来的应答（现象是「某条指令偶尔不生效」）。
           写满或抛错, 不做部分成功。
        """
        data = bytes(data)
        with self._io_lock:
            h = self._h
            if h is None:
                raise SerialError("串口未打开")
            sent = 0
            while sent < len(data):
                nwritten = wt.DWORD(0)
                ok = _k32.WriteFile(h, data[sent:], len(data) - sent,
                                    ctypes.byref(nwritten), None)
                if not ok:
                    raise SerialError("写 %s 失败: %s"
                                      % (self.port, _last_error_text(ctypes.get_last_error())))
                if nwritten.value == 0:
                    # 报告成功却一个字节没写出去 —— 再循环就是死转, 直接报出来
                    raise SerialError("写 %s 卡住（WriteFile 报告写了 0 字节）" % self.port)
                sent += nwritten.value
        return sent
