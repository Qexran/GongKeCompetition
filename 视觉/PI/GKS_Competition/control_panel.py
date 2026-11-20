#!/usr/bin/env python3
"""GKS 工科赛 控制台 — 800x480 触摸屏适配

仅包含通用功能（无任何任务）：
  - 网络信息 + 复制 SSH 命令
  - main.py 服务控制（启停 / 开机自启 / 直接启动或杀死 / 查看日志）
  - 控制台自身开机自启开关
  - 系统重启 / 关机
"""

import sys, os, subprocess, socket, time

from PyQt5 import QtWidgets, QtCore, QtGui

from config import PROJECT_DIR, SERVICE_NAME

AUTOSTART_DIR = os.path.expanduser("~/.config/autostart")
AUTOSTART_FILE = os.path.join(AUTOSTART_DIR, "gks-control-panel.desktop")
CONTROL_PANEL_PATH = os.path.abspath(__file__)

# 中文无乱码字体（树莓派 Debian 自带的 DroidSansFallbackFull）
FONT_FAMILY = "Droid Sans Fallback"


# ── 样式表（深色模式，仅通用按钮配色）──────────────────────
STYLE = """
QWidget {
    font-family: "Droid Sans Fallback";
    font-size: 14px;
    color: #e0e0e0;
    background-color: #1e1e2e;
}
QMainWindow {
    background-color: #1e1e2e;
}
QGroupBox {
    font-weight: bold;
    font-size: 13px;
    color: #cdd6f4;
    background-color: #262636;
    border: 1px solid #3a3a4e;
    border-radius: 8px;
    margin-top: 10px;
    padding-top: 16px;
    padding-bottom: 4px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 12px;
    padding: 0 6px;
    color: #89b4fa;
}
QPushButton {
    min-height: 38px;
    padding: 8px 15px;
    border-radius: 6px;
    font-size: 13px;
    font-weight: bold;
    border: none;
}
QPushButton:hover {
    opacity: 0.9;
}
QPushButton:pressed {
    border: 2px solid #cdd6f4;
}
/* 彩色功能按钮 */
QPushButton#btnStart { background-color: #40a02b; color: #ffffff; }
QPushButton#btnStop  { background-color: #d20f39; color: #ffffff; }
QPushButton#btnStartPy { background-color: #40a02b; color: #ffffff; }
QPushButton#btnKillPy  { background-color: #d20f39; color: #ffffff; }
QPushButton#btnEnableAuto  { background-color: #1e66f5; color: #ffffff; }
QPushButton#btnDisableAuto { background-color: #df8e1d; color: #ffffff; }
QPushButton#btnLog       { background-color: #8839ef; color: #ffffff; }
QPushButton#btnCopySsh   { background-color: #179299; color: #ffffff; }
QPushButton#btnClear     { background-color: #5c5c7a; color: #ffffff; }
QPushButton#btnShutdown { background-color: #d20f39; color: #ffffff; }
QPushButton#btnReboot  { background-color: #fe640b; color: #ffffff; }
/* IP 标签 */
QLabel#ipLabel {
    font-size: 16px;
    font-weight: bold;
    color: #89b4fa;
    padding: 2px;
    background: transparent;
    border: none;
}
QLabel#hostLabel {
    font-size: 13px;
    color: #a6adc8;
    padding: 1px;
    background: transparent;
    border: none;
}
/* 状态标签 */
QLabel#statusLabel {
    font-size: 15px;
    font-weight: bold;
    padding: 4px;
    background: transparent;
    border: none;
}
QLabel#autoStatusLabel {
    font-size: 13px;
    color: #a6adc8;
    padding: 2px;
    background: transparent;
    border: none;
}
QLabel#netLabel {
    background: transparent;
    border: none;
}
QCheckBox {
    color: #cdd6f4;
    font-size: 13px;
    background: transparent;
}
QCheckBox::indicator {
    width: 20px;
    height: 20px;
}
/* 滚动条 */
QScrollBar:vertical {
    width: 28px;
    background: #2a2a3c;
    border: none;
    border-radius: 4px;
}
QScrollBar::handle:vertical {
    background: #585b70;
    min-height: 40px;
    border-radius: 4px;
}
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {
    height: 0px;
}
QScrollBar:horizontal {
    height: 0px;
}
QScrollArea {
    background: transparent;
    border: none;
}
/* TextEdit 日志窗口 */
QTextEdit#logView {
    font-family: monospace;
    font-size: 11px;
    color: #cdd6f4;
    background-color: #11111b;
    border: 1px solid #3a3a4e;
    border-radius: 4px;
}
QMessageBox {
    background-color: #262636;
}
QMessageBox QLabel {
    color: #cdd6f4;
    background: transparent;
}
QMessageBox QPushButton {
    background-color: #1e66f5;
    color: #fff;
    min-width: 80px;
}
"""


# ── 触摸滚动区域（鼠标拖拽 → 滚动）────────────────────────
class TouchScrollArea(QtWidgets.QScrollArea):
    """用鼠标拖拽模拟手指滑动，兼容所有电阻/电容触摸屏。
    只有当拖拽超过阈值(THRESHOLD px)才进入滚动模式，
    否则正常传递给子控件（保证按钮能点击）。"""

    THRESHOLD = 8

    def __init__(self, parent=None):
        super().__init__(parent)
        self._drag_active = False
        self._press_y = 0
        self._scroll_start = 0
        self._moved_total = 0
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAsNeeded)
        self.setFrameShape(QtWidgets.QFrame.NoFrame)

    def eventFilter(self, obj, event):
        if obj is not self.viewport():
            return super().eventFilter(obj, event)

        etype = event.type()

        if etype == QtCore.QEvent.MouseButtonPress and event.button() == QtCore.Qt.LeftButton:
            self._drag_active = False
            self._press_y = event.pos().y()
            self._scroll_start = self.verticalScrollBar().value()
            self._moved_total = 0
            event.accept()
            return True

        elif etype == QtCore.QEvent.MouseMove and event.buttons() & QtCore.Qt.LeftButton:
            dy = self._press_y - event.pos().y()
            self._moved_total = abs(dy)
            if not self._drag_active and self._moved_total > self.THRESHOLD:
                self._drag_active = True
            if self._drag_active:
                self.verticalScrollBar().setValue(self._scroll_start + dy)
                event.accept()
                return True
            return super().eventFilter(obj, event)

        elif etype == QtCore.QEvent.MouseButtonRelease and event.button() == QtCore.Qt.LeftButton:
            was_drag = self._drag_active
            self._drag_active = False
            if was_drag:
                event.accept()
                return True
            return super().eventFilter(obj, event)

        return super().eventFilter(obj, event)


# ── 工具函数 ────────────────────────────────────────────────
def run_cmd(cmd, timeout=5):
    try:
        p = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=timeout)
        return p.stdout.strip(), p.stderr.strip(), p.returncode
    except subprocess.TimeoutExpired:
        return "", "timeout", -1
    except Exception as e:
        return "", str(e), -1


def get_ip_addresses():
    ips = []
    try:
        out = subprocess.check_output(["hostname", "-I"], text=True).strip()
        ips = [ip for ip in out.split() if ip.count(".") == 3 and not ip.startswith("127.")]
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


def is_service_active():
    _, _, rc = run_cmd(f"systemctl is-active --quiet {SERVICE_NAME}")
    return rc == 0


def is_service_enabled():
    _, _, rc = run_cmd(f"systemctl is-enabled --quiet {SERVICE_NAME}")
    return rc == 0


def is_control_panel_autostart():
    return os.path.exists(AUTOSTART_FILE)


def is_main_py_running():
    """检查是否有 main.py 进程在运行（不限启动方式）。

    注：与 cv_python 共用同一台树莓派时，此检查会匹配到任意 python3.*main.py；
    同一时间只运行一个项目即可避免误判。
    """
    try:
        result = subprocess.run(
            ["pgrep", "-f", "python3.*main.py"],
            capture_output=True, text=True, timeout=3)
        return result.returncode == 0
    except Exception:
        return False


# ── 日志查看窗口 ───────────────────────────────────────────
class LogViewerWindow(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("GKS 服务日志 - 实时")
        self.resize(780, 440)
        self.setMinimumSize(640, 320)

        self.log_view = QtWidgets.QTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setObjectName("logView")
        font = self.log_view.font()
        font.setFamily("monospace")
        font.setPointSize(11)
        self.log_view.setFont(font)

        self.btn_clear = QtWidgets.QPushButton("清空")
        self.btn_clear.setObjectName("btnClear")
        self.btn_close = QtWidgets.QPushButton("关闭")
        self.btn_close.setObjectName("btnClose")

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.addStretch()
        btn_row.addWidget(self.btn_clear)
        btn_row.addWidget(self.btn_close)

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.log_view)
        layout.addLayout(btn_row)

        self.btn_clear.clicked.connect(self.log_view.clear)
        self.btn_close.clicked.connect(self.close)

        self.process = None
        self.timer = None
        self._start_journalctl()

    def _start_journalctl(self):
        self.process = subprocess.Popen(
            ["journalctl", "-u", SERVICE_NAME, "-f", "-n", "100", "--no-pager"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
        )
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._read_output)
        self.timer.start(300)

    def _read_output(self):
        if self.process is None or self.process.poll() is not None:
            return
        try:
            import select
            while select.select([self.process.stdout], [], [], 0)[0]:
                line = self.process.stdout.readline()
                if line:
                    self.log_view.append(line.rstrip())
        except Exception:
            pass
        sb = self.log_view.verticalScrollBar()
        sb.setValue(sb.maximum())

    def closeEvent(self, event):
        if self.timer:
            self.timer.stop()
        if self.process:
            self.process.terminate()
            try:
                self.process.wait(timeout=2)
            except Exception:
                self.process.kill()
        super().closeEvent(event)


# ── 主窗口 ───────────────────────────────────────────────────
class ControlPanel(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("GKS 控制台")
        self.resize(800, 480)

        self._set_app_font()

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        main_layout = QtWidgets.QVBoxLayout(central)
        main_layout.setContentsMargins(6, 6, 6, 6)
        main_layout.setSpacing(0)

        self.scroll = TouchScrollArea()
        self.scroll.setWidgetResizable(True)

        self.scroll_content = QtWidgets.QWidget()
        self.content_layout = QtWidgets.QVBoxLayout(self.scroll_content)
        self.content_layout.setContentsMargins(4, 2, 4, 2)
        self.content_layout.setSpacing(8)

        self.scroll.setWidget(self.scroll_content)
        self.scroll.viewport().installEventFilter(self.scroll)
        self.scroll.viewport().setMouseTracking(True)

        main_layout.addWidget(self.scroll)

        self._build_ip_section()
        self._build_service_section()
        self._build_settings_section()
        self._build_power_section()
        self.content_layout.addStretch()

        self.refresh_timer = QtCore.QTimer(self)
        self.refresh_timer.timeout.connect(self._refresh_status)
        self.refresh_timer.start(2000)
        self._refresh_status()

    def _set_app_font(self):
        font = QtWidgets.QApplication.font()
        font.setFamily(FONT_FAMILY)
        font.setPointSize(11)
        QtWidgets.QApplication.setFont(font)

    # ── 1. 网络信息 ──
    def _build_ip_section(self):
        grp = QtWidgets.QGroupBox("网络信息")
        grp.setMaximumHeight(110)
        layout = QtWidgets.QHBoxLayout(grp)
        layout.setContentsMargins(8, 4, 8, 4)
        layout.setSpacing(10)

        left = QtWidgets.QVBoxLayout()
        left.setSpacing(1)
        self.ip_label = QtWidgets.QLabel()
        self.ip_label.setObjectName("ipLabel")
        self.host_label = QtWidgets.QLabel()
        self.host_label.setObjectName("hostLabel")
        left.addWidget(self.ip_label)
        left.addWidget(self.host_label)

        btn_copy = QtWidgets.QPushButton("复制SSH命令")
        btn_copy.setObjectName("btnCopySsh")
        btn_copy.setMaximumWidth(140)
        btn_copy.clicked.connect(self._copy_ssh)

        layout.addLayout(left)
        layout.addWidget(btn_copy)

        self.content_layout.addWidget(grp)

    # ── 2. 服务控制 ──
    def _build_service_section(self):
        grp = QtWidgets.QGroupBox("服务控制 (main.py)")
        layout = QtWidgets.QVBoxLayout(grp)

        self.status_label = QtWidgets.QLabel("状态: 检测中...")
        self.status_label.setObjectName("statusLabel")
        layout.addWidget(self.status_label)

        self.auto_status_label = QtWidgets.QLabel("开机自启: 检测中...")
        self.auto_status_label.setObjectName("autoStatusLabel")
        layout.addWidget(self.auto_status_label)

        btn_row1 = QtWidgets.QHBoxLayout()
        self.btn_start_svc = QtWidgets.QPushButton("启动服务")
        self.btn_start_svc.setObjectName("btnStart")
        self.btn_stop_svc = QtWidgets.QPushButton("停止服务")
        self.btn_stop_svc.setObjectName("btnStop")
        btn_row1.addWidget(self.btn_start_svc)
        btn_row1.addWidget(self.btn_stop_svc)
        layout.addLayout(btn_row1)

        btn_row2 = QtWidgets.QHBoxLayout()
        self.btn_start_py = QtWidgets.QPushButton("启动 main.py")
        self.btn_start_py.setObjectName("btnStartPy")
        self.btn_kill_py = QtWidgets.QPushButton("杀死 main.py")
        self.btn_kill_py.setObjectName("btnKillPy")
        btn_row2.addWidget(self.btn_start_py)
        btn_row2.addWidget(self.btn_kill_py)
        layout.addLayout(btn_row2)

        btn_row3 = QtWidgets.QHBoxLayout()
        self.btn_enable = QtWidgets.QPushButton("开启开机自启")
        self.btn_enable.setObjectName("btnEnableAuto")
        self.btn_disable = QtWidgets.QPushButton("关闭开机自启")
        self.btn_disable.setObjectName("btnDisableAuto")
        btn_row3.addWidget(self.btn_enable)
        btn_row3.addWidget(self.btn_disable)
        layout.addLayout(btn_row3)

        self.btn_log = QtWidgets.QPushButton("查看服务日志 (新窗口)")
        self.btn_log.setObjectName("btnLog")
        self.btn_log.clicked.connect(self._open_log_viewer)
        layout.addWidget(self.btn_log)

        self.btn_start_svc.clicked.connect(self._start_service)
        self.btn_stop_svc.clicked.connect(self._stop_service)
        self.btn_start_py.clicked.connect(self._start_main_py)
        self.btn_kill_py.clicked.connect(self._kill_main_py)
        self.btn_enable.clicked.connect(self._enable_service)
        self.btn_disable.clicked.connect(self._disable_service)

        self.content_layout.addWidget(grp)

    # ── 3. 设置 ──
    def _build_settings_section(self):
        grp = QtWidgets.QGroupBox("设置")
        layout = QtWidgets.QVBoxLayout(grp)

        self.cb_autostart_panel = QtWidgets.QCheckBox("开机自动启动此控制台")
        self.cb_autostart_panel.stateChanged.connect(self._toggle_panel_autostart)
        layout.addWidget(self.cb_autostart_panel)

        self.content_layout.addWidget(grp)

    # ── 4. 系统电源 ──
    def _build_power_section(self):
        grp = QtWidgets.QGroupBox("系统电源")
        layout = QtWidgets.QHBoxLayout(grp)

        self.btn_reboot = QtWidgets.QPushButton("重启")
        self.btn_reboot.setObjectName("btnReboot")
        self.btn_reboot.clicked.connect(self._reboot)
        layout.addWidget(self.btn_reboot)

        self.btn_shutdown = QtWidgets.QPushButton("关机")
        self.btn_shutdown.setObjectName("btnShutdown")
        self.btn_shutdown.clicked.connect(self._shutdown)
        layout.addWidget(self.btn_shutdown)

        self.content_layout.addWidget(grp)

    # ── 刷新 ──
    def _refresh_status(self):
        ips = get_ip_addresses()
        self.ip_label.setText("IP: " + ", ".join(ips))
        self.host_label.setText("主机名: " + get_hostname())

        svc_active = is_service_active()
        py_running = is_main_py_running()

        if svc_active:
            self.status_label.setText("[RUNNING] 服务状态: 运行中")
            self.status_label.setStyleSheet(
                "font-size: 15px; font-weight: bold; padding: 4px; color: #40a02b; background: transparent;")
        elif py_running:
            self.status_label.setText("[RUNNING] main.py 运行中 (非服务模式)")
            self.status_label.setStyleSheet(
                "font-size: 15px; font-weight: bold; padding: 4px; color: #df8e1d; background: transparent;")
        else:
            self.status_label.setText("[STOP] 服务状态: 已停止")
            self.status_label.setStyleSheet(
                "font-size: 15px; font-weight: bold; padding: 4px; color: #d20f39; background: transparent;")

        if is_service_enabled():
            self.auto_status_label.setText("开机自启: 已启用")
        else:
            self.auto_status_label.setText("开机自启: 已禁用")

        self.cb_autostart_panel.blockSignals(True)
        self.cb_autostart_panel.setChecked(is_control_panel_autostart())
        self.cb_autostart_panel.blockSignals(False)

    # ── 服务操作 ──
    def _start_service(self):
        _, stderr, rc = run_cmd(f"sudo systemctl start --no-block {SERVICE_NAME}")
        self._notify("提示", "服务已发出启动指令" if rc == 0 else f"启动失败:\n{stderr}")
        self._refresh_status()

    def _stop_service(self):
        _, stderr, rc = run_cmd(f"sudo systemctl stop --no-block {SERVICE_NAME}")
        self._notify("提示", "服务已停止" if rc == 0 else f"停止失败:\n{stderr}")
        self._refresh_status()

    def _enable_service(self):
        _, stderr, rc = run_cmd(f"sudo systemctl enable {SERVICE_NAME}")
        self._notify("提示", "开机自启已启用" if rc == 0 else f"操作失败:\n{stderr}")
        self._refresh_status()

    def _disable_service(self):
        _, stderr, rc = run_cmd(f"sudo systemctl disable {SERVICE_NAME}")
        self._notify("提示", "开机自启已禁用" if rc == 0 else f"操作失败:\n{stderr}")
        self._refresh_status()

    # ── 直接启停 main.py ──
    def _start_main_py(self):
        """直接在后台启动 main.py（不走 systemd，调试模式，无自动重启）"""
        if is_main_py_running():
            if is_service_active():
                self._notify("提示",
                             "main.py 已通过 systemd 在运行中。\n\n"
                             "如需调试模式，请先点击「停止服务」，再点「启动 main.py」。")
            else:
                self._notify("提示", "main.py 已在运行中（手动模式），无需重复启动")
            self._refresh_status()
            return
        try:
            subprocess.Popen(
                ["python3", "main.py"], cwd=PROJECT_DIR,
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            self._notify("提示", "main.py 已在后台启动\n(调试模式：无自动重启)")
        except Exception as e:
            self._notify("错误", f"启动失败: {e}")
        self._refresh_status()

    def _kill_main_py(self):
        """终止所有 main.py 进程。先停 systemd 防止 Restart=always 自动复活"""
        if not is_main_py_running():
            self._notify("提示", "main.py 本来就没有在运行")
            self._refresh_status()
            return

        if is_service_active():
            _, stderr, rc = run_cmd(f"sudo systemctl stop {SERVICE_NAME}", timeout=15)
            if rc != 0:
                self._notify("警告", f"systemd 停止异常:\n{stderr}\n继续尝试终止进程...")

        try:
            subprocess.run(["pkill", "-f", "python3.*main.py"], timeout=5)
        except subprocess.TimeoutExpired:
            pass
        except Exception as e:
            self._notify("错误", f"终止进程失败: {e}")

        self._refresh_status()
        if is_main_py_running():
            self._notify("警告", "main.py 进程可能未能终止，请重试")
        else:
            self._notify("提示", "main.py 已终止\n(systemd 已停止，手动进程已清理)")

    # ── 日志窗口 ──
    def _open_log_viewer(self):
        self.log_window = LogViewerWindow(self)
        self.log_window.show()

    # ── 控制台开机自启 ──
    def _toggle_panel_autostart(self, state):
        if state:
            os.makedirs(AUTOSTART_DIR, exist_ok=True)
            desktop_entry = f"""[Desktop Entry]
Type=Application
Name=GKS Control Panel
Comment=GKS Control Panel
Exec=/usr/bin/python3 {CONTROL_PANEL_PATH}
StartupNotify=false
X-GNOME-Autostart-enabled=true
"""
            with open(AUTOSTART_FILE, "w") as f:
                f.write(desktop_entry)
            self._notify("提示", "控制台已设为开机自启")
        else:
            if os.path.exists(AUTOSTART_FILE):
                os.remove(AUTOSTART_FILE)
            self._notify("提示", "控制台开机自启已取消")

    # ── 复制 SSH ──
    def _copy_ssh(self):
        ips = get_ip_addresses()
        ip = ips[0] if ips else "unknown"
        ssh_cmd = f"ssh xixi@{ip}"
        clipboard = QtWidgets.QApplication.clipboard()
        clipboard.setText(ssh_cmd)
        self._notify("已复制", f"SSH 命令已复制到剪贴板:\n{ssh_cmd}")

    # ── 系统电源 ──
    def _shutdown(self):
        reply = QtWidgets.QMessageBox.question(
            self, "确认关机", "确定要关闭树莓派电源吗？",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.Yes:
            run_cmd("sudo shutdown -h now")
            self._notify("提示", "关机指令已发送")

    def _reboot(self):
        reply = QtWidgets.QMessageBox.question(
            self, "确认重启", "确定要重启树莓派吗？",
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if reply == QtWidgets.QMessageBox.Yes:
            run_cmd("sudo shutdown -r now")
            self._notify("提示", "重启指令已发送")

    # ── 提示 ──
    def _notify(self, title, msg):
        QtWidgets.QMessageBox.information(self, title, msg)


# ── 入口 ──
def main():
    os.environ.setdefault("QT_LOGGING_RULES", "qt.qpa.fonts.warning=false")
    app = QtWidgets.QApplication(sys.argv)
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    panel = ControlPanel()
    panel.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
