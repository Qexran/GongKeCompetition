# web_preview.py — Web 实时预览框架（MJPEG over HTTP）
#
# 统一的调试预览方案：放弃 cv2.imshow / Qt 窗口，改用浏览器实时推流。
# 任何需要调试的地方，导入 preview 单例并调用 publish() / add_camera() 即可，
# 服务在首次调用时自动后台启动，浏览器访问 http://<IP>:<port>/ 查看。
#
# 典型用法:
#   from web_preview import preview
#
#   # 1) 推一帧带标注的图（任务循环里最常用，自动启动服务）
#   preview.publish("detection", annotated_frame)
#
#   # 2) 绑定摄像头，后台线程自动读帧推流
#   preview.add_camera("main", cap, label="主摄像头")
#
#   # 3) 独立预览脚本里阻塞运行直到 Ctrl+C
#   preview.serve(port=8080)
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote

import cv2

from config import WEB_PREVIEW_PORT, WEB_PREVIEW_QUALITY


def _get_ips():
    import subprocess
    try:
        out = subprocess.check_output(["hostname", "-I"], text=True).strip()
        return [ip for ip in out.split() if ip.count(".") == 3]
    except Exception:
        return ["127.0.0.1"]


def _draw_overlay(frame, label, t_list):
    """在帧上叠加 名称 + FPS + 时间戳（原地修改，供摄像头流使用）。"""
    now = time.time()
    t_list.append(now)
    t_list[:] = [t for t in t_list if now - t <= 2.0]
    fps = len(t_list) / (t_list[-1] - t_list[0]) if len(t_list) >= 2 else 0.0
    cv2.putText(frame, f"{label}  FPS:{fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
    cv2.putText(frame, time.strftime("%H:%M:%S"), (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return frame


class StreamChannel:
    """一个命名推流通道：保存最新一帧的 JPEG 编码，供 HTTP 拉取。

    - name:  URL 路径标识（建议 ASCII，如 "main"/"detection"）
    - label: 网页显示名（默认同 name）
    """

    def __init__(self, name, label=None, quality=WEB_PREVIEW_QUALITY):
        self.name = name
        self.label = label or name
        self.quality = quality
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0

    def publish(self, frame):
        """编码并发布一帧（线程安全）。frame 为 None 时忽略。"""
        if frame is None:
            return
        ok, buf = cv2.imencode(".jpg", frame,
                               [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if not ok:
            return
        with self._cond:
            self._jpeg = buf.tobytes()
            self._seq += 1
            self._cond.notify_all()

    def wait_latest(self, last_seq, timeout=2.0):
        """阻塞等待 seq > last_seq 的新帧；超时返回最后一帧作 keepalive。

        返回 (jpeg, seq)。jpeg 为 None 表示尚无任何帧。
        """
        with self._cond:
            if self._seq <= last_seq:
                self._cond.wait(timeout)
            return self._jpeg, self._seq

    def latest(self):
        with self._cond:
            return self._jpeg


class CameraStream:
    """后台线程持续从摄像头读帧 → 发布到关联通道。"""

    def __init__(self, channel, cap, read_fn, overlay=True):
        self.channel = channel
        self.cap = cap
        self.read_fn = read_fn
        self.overlay = overlay
        self._t_list = []
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while self._running:
            frame = self.read_fn(self.cap)
            if frame is None:
                time.sleep(0.01)
                continue
            if self.overlay:
                # 画面叠加用 ASCII 的 name：cv2.putText 的 Hershey 字体不支持中文，
                # 中文标签（label）改由网页标题渲染。
                _draw_overlay(frame, self.channel.name, self._t_list)
            self.channel.publish(frame)

    def stop(self):
        self._running = False


def _index_html(channels):
    items = "".join(
        f'<div class="stream"><h2>{ch.label}</h2><img src="/stream/{ch.name}"></div>'
        for ch in sorted(channels.values(), key=lambda c: c.name)
    )
    if not items:
        items = '<p style="text-align:center;color:#888">暂无推流通道</p>'
    return f"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Web 预览</title>
<style>
body{{background:#111;color:#eee;font-family:sans-serif;margin:0;padding:20px;
     display:flex;gap:20px;flex-wrap:wrap;justify-content:center}}
.stream{{text-align:center}}
.stream h2{{margin:4px 0;font-size:16px}}
img{{max-width:90vw;max-height:80vh;border:1px solid #333;background:#000}}
</style></head><body>{items}</body></html>"""


def _make_handler(channels):
    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path = unquote(self.path)
            if path == "/":
                self._serve_index()
            elif path.startswith("/stream/"):
                ch = channels.get(path[len("/stream/"):])
                if ch is None:
                    self.send_error(404, "unknown stream")
                else:
                    self._serve_mjpeg(ch)
            elif path.startswith("/snapshot/"):
                ch = channels.get(path[len("/snapshot/"):])
                if ch is None:
                    self.send_error(404, "unknown stream")
                else:
                    self._serve_snapshot(ch)
            else:
                self.send_error(404)

        def _serve_index(self):
            body = _index_html(channels).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_snapshot(self, ch):
            jpeg = ch.latest()
            if jpeg is None:
                self.send_error(404, "no frame yet")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)

        def _serve_mjpeg(self, ch):
            self.send_response(200)
            self.send_header("Content-Type",
                             "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.end_headers()
            last_seq = 0
            try:
                while True:
                    jpeg, seq = ch.wait_latest(last_seq, timeout=2.0)
                    if jpeg is None:
                        time.sleep(0.05)
                        continue
                    last_seq = seq
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        f"Content-Length: {len(jpeg)}\r\n\r\n".encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass  # 浏览器关闭标签页，正常断开

        def log_message(self, fmt, *args):
            pass  # 静音访问日志，避免刷屏

    return Handler


class WebPreview:
    """Web 推流框架门面。模块级单例 `preview`。

    用法:
        from web_preview import preview
        preview.publish("detection", frame)               # 推一帧（自动启动服务）
        preview.add_camera("main", cap, label="主摄像头")  # 绑定摄像头自动推流
        preview.serve(port=8080)                          # 独立脚本阻塞运行
        preview.stop()                                    # 停止服务与所有摄像头流
    """

    def __init__(self):
        self._lock = threading.Lock()
        self._channels = {}          # name -> StreamChannel
        self._camera_streams = []    # CameraStream
        self._server = None
        self._port = WEB_PREVIEW_PORT

    # ---- 内部 ----
    def _register_channel(self, name, label=None):
        with self._lock:
            ch = self._channels.get(name)
            if ch is None:
                ch = StreamChannel(name, label)
                self._channels[name] = ch
            return ch

    def _ensure_started(self):
        self.start()

    def _stop_server(self):
        server = self._server
        self._server = None
        if server is not None:
            server.shutdown()
            server.server_close()

    def _start_server(self, port):
        self._port = port
        self._server = ThreadingHTTPServer(
            ("0.0.0.0", port), _make_handler(self._channels))
        threading.Thread(target=self._server.serve_forever, daemon=True).start()
        print("\n[web_preview] 推流服务已启动，浏览器访问:")
        for ip in _get_ips():
            print(f"  http://{ip}:{port}/")

    # ---- 公开 API ----
    def publish(self, name, frame, label=None):
        """推送一帧到命名通道（线程安全）。首次调用自动后台启动服务。"""
        ch = self._register_channel(name, label)
        self._ensure_started()
        ch.publish(frame)

    def add_camera(self, name, cap, read_fn=None, label=None):
        """注册摄像头流：后台线程持续 read_fn(cap) → 推送到 name 通道。"""
        from camera import CameraManager
        if read_fn is None:
            read_fn = lambda c: CameraManager.read(c, name, max_retries=1)
        ch = self._register_channel(name, label)
        self._ensure_started()
        stream = CameraStream(ch, cap, read_fn)
        with self._lock:
            self._camera_streams.append(stream)
        return stream

    def start(self, port=None):
        """启动服务（后台线程，不阻塞）。

        指定 port 且与当前已启动端口不同时会重启到新端口；未启动时按指定端口
        （默认 config 端口）启动。publish/add_camera 内部已自动调用。
        """
        with self._lock:
            if port is not None and self._server is not None and port != self._port:
                self._stop_server()      # 端口变化 → 先停旧服务
            if self._server is None:
                if port is not None:
                    self._port = port
                self._start_server(self._port)

    def serve(self, port=None):
        """阻塞运行直到 Ctrl+C。供独立预览脚本使用。"""
        self.start(port=port)
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("\n[web_preview] 收到中断，停止服务")
        finally:
            self.stop()

    def stop(self):
        """停止服务与所有摄像头推流线程。"""
        with self._lock:
            for s in self._camera_streams:
                s.stop()
            self._camera_streams.clear()
            self._stop_server()

    @property
    def names(self):
        return list(self._channels.keys())


preview = WebPreview()
