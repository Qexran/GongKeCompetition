# webconfig.py — GKS Web 控制端 自身配置
#
# 注意：本模块刻意不叫 config.py，避免与 GKS_Competition/config.py 重名，
# 防止 app.py 通过 sys.path 导入 GKS_Competition 的 camera/protocol 时发生
# `from config import ...` 解析冲突。
import os

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# Web 控制台监听地址/端口（避开 web_preview 8080 与 camera_web 遗留的 8000——后者已废弃，
# main.py 不再启动它，仅手动独立调试时才占用）
WEB_HOST = "0.0.0.0"
WEB_PORT = 9000

# 访问令牌（2026-10 加, 治两类现实威胁: ① 树莓派进手机热点时, 热点上**任何设备**
# （含蹭网的）都能打关机/停服务/改参数这些无认证端点; ② 操作员浏览器开着的其它
# http 网页能跨站发简单 POST（CSRF）打到同样的端点 —— 浏览器对 text/plain 的
# POST 不做预检, 页面自己设的 Content-Type 并不是防线。
# 用法: 浏览器第一次打开 http://<ip>:9000/?key=<本值>（会种 cookie, 之后免带）。
# 空串 = 关闭认证（恢复旧行为, 仅建议在完全隔离的私有网络里）。**改完要换一个**
# 自己的随机串, 别用注释里的示例。
WEB_KEY = "2752844566"

# 无头单片机模拟器监听地址（main.py 以 SIMULATOR_MODE=1 时连接这里）
SIM_HOST = "127.0.0.1"
SIM_PORT = 9997

# GKS_Competition 项目目录：app.py 将其加入 sys.path，导入
# protocol / camera / tasks._qr / log_buffer 等无副作用库模块。
GKS_COMPETITION_DIR = "/home/xixi/Desktop/GKS_Competition"

# systemd 服务名（控制台启停 / 看日志用）
GKS_SERVICE = "gks.service"          # GKS 主程序 main.py
WEB_SERVICE = "gks-web.service"      # 本 Web 控制端自身

# 设备固定脚本（重新生成 udev 规则，把摄像头/串口绑成固定符号链接：
# camera_main / camera_Qr / ttyUSB_GKS）。控制台「固定设备」按钮 = sudo bash 本脚本 --apply。
# ⚠ 脚本按「检测到的先后顺序」给摄像头命名（第一个→camera_main，第二个→camera_Qr），
#   运行前必须两个摄像头都已插入，否则角色会对调。
FIX_DEVICE_SCRIPT = "/home/xixi/Desktop/fix_device.sh"

# 模拟器帧交换日志在内存中的保留行数
SIM_LOG_MAX_LINES = 500

# ---- WiFi（NetworkManager / nmcli）----
NMCLI = "/usr/bin/nmcli"
NM_KEYFILE_DIR = "/etc/NetworkManager/system-connections"
WIFI_IFACE = "wlan0"
WIFI_TOP_PRIORITY = 100      # 「设为开机首选」写入的 autoconnect-priority（数字越大越先）
WIFI_SWITCH_WAIT = 45        # nmcli --wait 秒（激活上界）
WIFI_VERIFY_GRACE = 20       # 激活后等 DHCP 拿地址的秒数——只关联上不算成功
WIFI_SCAN_MIN_INTERVAL = 10  # 重复扫描的最小间隔（NM 自身也限流）
WIFI_HINT_URL = "http://%s.local:9000/"   # %s = hostname → 换网后靠 mDNS 找回页面的线索
                                          # ⚠ 必须是 <hostname>.local: get_hostname() 给的是
                                          # `xixi`, 少了 .local 就只是个解析不了的名字
