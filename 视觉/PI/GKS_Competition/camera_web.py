#!/usr/bin/env python3
"""摄像头 Web 控制页 —— 预览 + 二维码识别（无需 X11 / 显示环境）

原理：MJPEG over HTTP。浏览器打开首页即可：
  - 预览主/扫码摄像头（点「测试摄像头」开始推流，加载时不自动推流）；
  - 点「扫码二维码(OpenCV)」/「(pyzbar)」触发二维码识别任务，识别帧回显到网页；
  - 点「退出任务」随时打断正在运行的识别任务。

⚠ **已废弃**：main.py **不再**调用 start_server()（见 main.py 里那段说明注释）——
日常控制/参数调整请用 GKS_Web 的 `:9000` 页面，它已覆盖本页全部功能。
保留本文件是因为 handler.py 的 `_stop_main_stream()` 要 import 本模块的 `_stop_stream`
（人质/色球识别前释放主摄），删除会让识别任务抢不到摄像头。

单独调试时仍可独立运行:

    python3 camera_web.py                     # 默认 0.0.0.0:8000
    python3 camera_web.py --port 8080         # 指定端口

接口:
    首页:         http://<IP>:8000/
    实时流:       /stream?cam=main 或 ?cam=qr
    单帧快照:     /snapshot.jpg?cam=main 或 ?cam=qr
    二维码识别:   /scan?method=opencv 或 ?method=pyzbar  (返回 JSON)
    人质识别:     /hostage  (返回 JSON，含从左到右目标顺序+中心坐标+标注帧)
    打断识别:     /abort
"""

import argparse
import base64
import json
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2

from camera import CameraManager, open_camera
import config
import log_buffer
from openmv_serial import openmv_reader
from config import WEB_CONTROL_PORT
from tasks import run_qr_opencv, run_qr_pyzbar, run_hostage_detect

JPEG_QUALITY = 80
BOUNDARY = "frame"

# 扫描任务状态：同一时间只允许一个扫描任务，_scan_abort 用于随时打断
_scan_lock = threading.Lock()
_scan_abort = threading.Event()

# 摄像头推流注册表：cam_key -> _CameraStream，供「退出任务/扫码」主动停止并释放摄像头
_streams = {}
_streams_lock = threading.Lock()


class _CameraStream:
    def __init__(self, cam_key, cap, conn):
        self.cam_key = cam_key
        self.cap = cap
        self.conn = conn          # 底层 socket，用于 shutdown 打断阻塞中的 write
        self.stop = threading.Event()


def _stop_stream(cam_key):
    """主动停止某路推流：置停止标志 + shutdown socket 打断可能阻塞的 write。"""
    with _streams_lock:
        s = _streams.get(cam_key)
    if s is not None:
        s.stop.set()
        try:
            s.conn.shutdown(socket.SHUT_RDWR)
        except Exception:
            pass


def _stop_all_streams():
    """停止所有推流（「退出任务」用），让各推流线程释放摄像头。"""
    with _streams_lock:
        keys = list(_streams.keys())
    for k in keys:
        _stop_stream(k)

HTML_PAGE = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>GKS 摄像头控制</title>
<style>
  body { font-family: sans-serif; background:#1e1e2e; color:#e0e0e0; margin:0; padding:16px; }
  h1 { font-size:18px; color:#89b4fa; }
  .grid { display:flex; flex-wrap:wrap; gap:16px; }
  .cam { flex:1 1 400px; background:#262636; border:1px solid #3a3a4e; border-radius:8px; padding:10px; }
  .cam h2 { font-size:14px; margin:0 0 8px; color:#cdd6f4; }
  .box { width:100%; aspect-ratio:4/3; background:#111; border-radius:4px; overflow:hidden; }
  .box img { width:100%; height:100%; object-fit:contain; display:block; }
  .placeholder { width:100%; height:100%; display:flex; align-items:center; justify-content:center;
                 color:#6c7086; font-size:14px; background:#181825; }
  .controls { margin-top:16px; display:flex; flex-wrap:wrap; gap:10px; }
  .controls button { background:#1e66f5; color:#fff; border:none; border-radius:6px; padding:10px 16px;
                     font-size:14px; font-weight:bold; cursor:pointer; }
  .controls button#btnTest { background:#40a02b; }
  .controls button#btnAbort { background:#d20f39; }
  .status { margin-top:12px; font-size:14px; color:#a6adc8; min-height:20px; }
  .settings { display:none; margin-top:16px; background:#262636; border:1px solid #3a3a4e;
              border-radius:8px; padding:12px; }
  .settings h3 { font-size:14px; color:#cdd6f4; margin:0 0 8px; }
  .cfg-row { display:flex; align-items:center; gap:10px; margin-bottom:8px; }
  .cfg-row label { flex:0 0 220px; font-size:13px; color:#a6adc8; }
  .cfg-row input { flex:1; background:#11111b; color:#e0e0e0; border:1px solid #3a3a4e;
                   border-radius:4px; padding:6px 8px; font-size:13px; }
  .cfg-actions { margin-top:10px; display:flex; gap:10px; }
  .cfg-actions button { background:#1e66f5; color:#fff; border:none; border-radius:6px;
                        padding:8px 14px; font-size:13px; font-weight:bold; cursor:pointer; }
  .logpanel { margin-top:16px; background:#11111b; border:1px solid #3a3a4e;
              border-radius:8px; padding:10px; }
  .logpanel h3 { font-size:14px; color:#cdd6f4; margin:0 0 6px; }
  .logpanel pre { margin:0; height:220px; overflow-y:auto; font-family:monospace;
                  font-size:12px; color:#cdd6f4; white-space:pre-wrap; word-break:break-all; }
</style>
</head>
<body>
<h1>GKS 摄像头控制</h1>
<div class="grid">
  <div class="cam"><h2>主摄像头 /dev/camera_main</h2>
    <div class="box" id="box-main"><div class="placeholder">等待开始...</div></div></div>
  <div class="cam"><h2>扫码摄像头 /dev/camera_Qr</h2>
    <div class="box" id="box-qr"><div class="placeholder">等待开始...</div></div></div>
</div>
<div class="controls">
  <button id="btnTest" onclick="startTest()">测试摄像头</button>
  <button onclick="scanQR('opencv')">扫码二维码(OpenCV)</button>
  <button onclick="scanQR('pyzbar')">扫码二维码(pyzbar)</button>
  <button onclick="scanHostage()">人质识别(YOLO)</button>
  <button id="btnAbort" onclick="abortTask()">退出任务</button>
  <button id="btnCfg" onclick="openSettings()">设置参数</button>
</div>
<div class="status" id="status">就绪</div>
<div class="logpanel">
  <h3>OpenMV 寻线</h3>
  <div class="cfg-row"><label id="lineInfo">读取中...</label></div>
  <div class="cfg-actions">
    <button onclick="lineStart()">开始巡线转发</button>
    <button onclick="lineStop()">停止巡线转发</button>
  </div>
</div>
<div id="settingsPanel" class="settings">
  <h3>设置参数</h3>
  <div id="settingsForm"></div>
  <div class="cfg-actions">
    <button onclick="saveSettings()">保存</button>
    <button onclick="el('settingsPanel').style.display='none'">关闭</button>
  </div>
</div>
<div class="logpanel">
  <h3>运行日志</h3>
  <pre id="logArea">加载中...</pre>
</div>
<script>
let scanCtrl = null;   // 当前扫码请求的 AbortController，退出/重开时用于取消

function el(id){ return document.getElementById(id); }
function placeholder(txt){ return '<div class="placeholder">' + (txt || '等待开始...') + '</div>'; }
function setStatus(t){ el('status').textContent = t; }

// 停止服务端正在运行的扫码任务（若有）
function stopScan(){
  if (scanCtrl) { scanCtrl.abort(); scanCtrl = null; }
  fetch('/abort');
}

function startTest(){
  stopScan();   // 开始推流前先取消可能残留的扫码，释放二维码摄像头
  el('box-main').innerHTML = '<img src="/stream?cam=main">';
  el('box-qr').innerHTML = '<img src="/stream?cam=qr">';
  setStatus('测试摄像头：两路推流已开启');
}

async function scanQR(method){
  if (scanCtrl) scanCtrl.abort();   // 新任务打断旧任务：取消上一次扫码
  const ctrl = new AbortController();
  scanCtrl = ctrl;
  // 暂时屏蔽二维码推流：还原 QR 框为占位（服务端 /scan 也会主动停掉二维码推流）
  el('box-qr').innerHTML = placeholder('扫码中...');
  setStatus('正在扫码 (' + method + ')...');
  try {
    const resp = await fetch('/scan?method=' + method, { signal: ctrl.signal });
    const d = await resp.json();
    if (ctrl.signal.aborted) return;   // 已被更新任务/退出打断，不再刷新界面
    if (d.image) {
      el('box-qr').innerHTML = '<img src="data:image/jpeg;base64,' + d.image + '">';
    } else {
      el('box-qr').innerHTML = placeholder('识别完成');
    }
    setStatus(d.success ? ('识别成功: ' + d.code + ' (' + d.method + ')')
                        : ('识别失败: ' + d.reason));
  } catch (e) {
    if (!ctrl.signal.aborted) setStatus('请求失败: ' + e);
  }
}

async function scanHostage(){
  if (scanCtrl) scanCtrl.abort();   // 新任务打断旧任务：取消上一次扫码/识别
  const ctrl = new AbortController();
  scanCtrl = ctrl;
  // 暂时屏蔽主摄推流：还原主摄框为占位（服务端 /hostage 也会主动停掉主摄推流）
  el('box-main').innerHTML = placeholder('人质识别中...');
  setStatus('人质识别中（凑满 3 个目标，限时 ~6s）...');
  try {
    const resp = await fetch('/hostage', { signal: ctrl.signal });
    const d = await resp.json();
    if (ctrl.signal.aborted) return;   // 已被更新任务/退出打断，不再刷新界面
    if (d.image) {
      el('box-main').innerHTML = '<img src="data:image/jpeg;base64,' + d.image + '">';
    } else {
      el('box-main').innerHTML = placeholder('识别完成');
    }
    if (d.success) {
      // 目标已按图像从左到右排序：1.圆锥(640,512) 2.圆柱(960,508) ...
      const order = d.targets.map((t, i) => (i + 1) + '.' + t.shape_name
                                             + '(' + t.cx + ',' + t.cy + ')').join(' ');
      setStatus('识别成功（从左到右）: ' + order);
    } else {
      setStatus('识别失败: ' + d.reason);
    }
  } catch (e) {
    if (!ctrl.signal.aborted) setStatus('请求失败: ' + e);
  }
}

function abortTask(){
  stopScan();   // 打断服务端扫码 + 取消前端请求
  el('box-main').innerHTML = placeholder();   // 停止主摄像头推流
  el('box-qr').innerHTML = placeholder();     // 停止二维码推流
  setStatus('已退出：识别任务与推流均已停止');
}

// ---- 日志轮询 ----
async function refreshLog(){
  try {
    const r = await fetch('/log');
    const t = await r.text();
    const pre = el('logArea');
    pre.textContent = t;
    pre.scrollTop = pre.scrollHeight;
  } catch (e) {}
}
setInterval(refreshLog, 1000);
refreshLog();

// ---- OpenMV 寻线状态轮询 ----
async function refreshLine(){
  try {
    const r = await fetch('/line_status');
    const d = await r.json();
    let txt = (d.connected ? '已连接' : '未连接') +
      ' | offset=' + d.offset + ' angle=' + (d.angle / 10).toFixed(1) + '°' +
      ' valid=' + d.valid +
      (d.age !== null ? ' 延迟=' + d.age + 's' : '') +
      ' | 转发: ' + (d.running ? '运行中' : '停止');
    if (d.bad_lines > 0) txt += ' | 坏行=' + d.bad_lines;
    if (d.hint) txt += ' | ⚠ ' + d.hint;
    el('lineInfo').textContent = txt;
  } catch (e) {}
}
setInterval(refreshLine, 1000);
refreshLine();

async function lineStart(){ fetch('/line_start').then(refreshLine); setStatus('巡线转发已启动'); }
async function lineStop(){ fetch('/line_stop').then(refreshLine); setStatus('巡线转发已停止'); }

// ---- 设置参数 ----
let cfgItems = [];
async function openSettings(){
  const panel = el('settingsPanel');
  if (panel.style.display === 'block') { panel.style.display = 'none'; return; }
  try {
    const r = await fetch('/config');
    cfgItems = await r.json();
    let html = '';
    for (const it of cfgItems) {
      html += '<div class="cfg-row"><label>' + it.desc + ' (' + it.name + ')</label>' +
              '<input id="cfg-' + it.name + '" value="' + it.value + '"></div>';
    }
    el('settingsForm').innerHTML = html;
    panel.style.display = 'block';
  } catch (e) { setStatus('读取配置失败: ' + e); }
}

async function saveSettings(){
  const form = new URLSearchParams();
  for (const it of cfgItems) {
    form.append(it.name, el('cfg-' + it.name).value);
  }
  try {
    const resp = await fetch('/config', { method: 'POST', body: form });
    const d = await resp.json();
    setStatus(d.ok ? '设置已保存' : '保存失败: ' + (d.message || ''));
    if (d.ok) { el('settingsPanel').style.display = 'none'; }
  } catch (e) { setStatus('保存失败: ' + e); }
}
</script>
</body>
</html>
"""


def _local_ips():
    import subprocess
    try:
        out = subprocess.check_output(["hostname", "-I"], text=True).strip()
        return [ip for ip in out.split()
                if ip.count(".") == 3 and not ip.startswith("127.")]
    except Exception:
        return []


class CameraHandler(BaseHTTPRequestHandler):
    def _cam_key(self):
        parsed = urlparse(self.path)
        qs = parse_qs(parsed.query)
        return "qr" if qs.get("cam", ["main"])[0] == "qr" else "main"

    def _send_html(self):
        body = HTML_PAGE.encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _send_json(self, code, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_scan(self, method):
        """运行二维码识别任务（限时，可被打断），返回 JSON + 识别帧 base64。

        新任务打断正在执行的任务：若已有扫码在跑，先置中断信号让旧任务退出，
        再串行接手执行新扫码。
        """
        if method not in ("opencv", "pyzbar"):
            self._send_json(400, {"success": False, "reason": f"未知方法: {method}"})
            return
        # 打断正在执行的扫码任务（若有），并阻塞等待其释放锁后接手
        _scan_abort.set()
        _scan_lock.acquire()
        try:
            _scan_abort.clear()
            _stop_stream("qr")  # 扫码前主动停掉二维码推流，释放设备
            fn = run_qr_opencv if method == "opencv" else run_qr_pyzbar
            result = fn(abort_event=_scan_abort)

            # 被打断（单片机/web 主动放弃）：不返回失败信息，日志已记录
            if result.get("aborted"):
                self._send_json(200, {"aborted": True, "method": result["method"]})
                return

            image_b64 = None
            frame = result.get("frame")
            if frame is not None:
                ok, buf = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    image_b64 = base64.b64encode(buf.tobytes()).decode("ascii")

            self._send_json(200, {
                "success": result["success"],
                "code": result["code"],
                "method": result["method"],
                "reason": result["reason"],
                "image": image_b64,
            })
        finally:
            _scan_lock.release()

    def _handle_hostage(self):
        """运行人质形状识别任务（主摄像头 YOLO，限时，可被打断），返回 JSON + 标注帧。

        与 _handle_scan 同构：仅页面显示结果，不回传单片机（串口链路由单片机
        发指令 '4' 触发 handler._cmd_hostage 完成）。共用 _scan_lock/_scan_abort，
        与扫码任务互斥、可互相打断。
        """
        _scan_abort.set()
        _scan_lock.acquire()
        try:
            _scan_abort.clear()
            _stop_stream("main")  # 识别前主动停掉主摄推流，释放设备
            result = run_hostage_detect(abort_event=_scan_abort)

            # 被打断（单片机/web 主动放弃）：不返回失败信息，日志已记录
            if result.get("aborted"):
                self._send_json(200, {"aborted": True})
                return

            image_b64 = None
            frame = result.get("frame")
            if frame is not None:
                ok, buf = cv2.imencode(".jpg", frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    image_b64 = base64.b64encode(buf.tobytes()).decode("ascii")

            self._send_json(200, {
                "success": result["success"],
                "reason": result["reason"],
                "targets": [{k: t[k] for k in ("shape", "shape_name", "cx", "cy", "conf")}
                            for t in result.get("targets", [])],
                "image": image_b64,
            })
        finally:
            _scan_lock.release()

    def _handle_abort(self):
        """退出任务：打断扫码 + 停止所有推流并释放摄像头。"""
        _scan_abort.set()
        _stop_all_streams()
        self._send_json(200, {"ok": True})

    # ---- 巡线调试（OpenMV 中转）----

    def _handle_line_status(self):
        """返回 OpenMV 传感器最新数据 + 巡线线程状态（JSON）。"""
        import handler
        data = openmv_reader.get_latest()
        data["running"] = handler.is_line_running()
        data["age"] = round(time.time() - data["timestamp"], 3) if data["timestamp"] else None
        self._send_json(200, data)

    def _handle_line_start(self):
        """手动开始巡线转发（等价于单片机发命令 '2'）。"""
        import handler
        handler._cmd_line_start()
        self._send_json(200, {"ok": True, "running": handler.is_line_running()})

    def _handle_line_stop(self):
        """手动停止巡线转发（等价于单片机发命令 '3'）。"""
        import handler
        handler._cmd_line_stop()
        self._send_json(200, {"ok": True})

    def _handle_log(self):
        """返回运行日志（控制台已不再打印，日志通过此端点展示在 Web 端）。"""
        body = log_buffer.recent().encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "text/plain; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _handle_config_get(self):
        """返回「设置参数」可编辑项及其当前值。"""
        # EDITABLE_CONFIG 是 4 元组 (name, type, desc, opts)：第 4 字段是选项表
        # （"0=关|1=开"），前端据此把布尔项画成勾选框、枚举项画成下拉框。
        # ⚠ 本页已废弃（:8000 的旧调参页，现行的是 GKS_Web :9000），只跟着改解包形状。
        items = [{"name": name, "type": typ, "desc": desc, "opts": opt,
                  "value": getattr(config, name)}
                 for name, typ, desc, opt in config.EDITABLE_CONFIG]
        self._send_json(200, items)

    def _handle_config_post(self):
        """保存「设置参数」修改：运行时即时生效 + 持久化到 config_override.json。"""
        length = int(self.headers.get("Content-Length", 0) or 0)
        body = self.rfile.read(length).decode("utf-8") if length else ""
        params = parse_qs(body) if body else {}
        editable = {name for name, _t, _d, _o in config.EDITABLE_CONFIG}
        msgs = []
        ok = True
        for name, vals in params.items():
            value = vals[0] if vals else ""
            if name not in editable:
                ok = False
                msgs.append(f"未知参数 {name}")
                continue
            saved, msg = config.save_override(name, value)
            msgs.append(msg)
            if not saved:
                ok = False
        self._send_json(200, {"ok": ok, "message": "; ".join(msgs) if msgs else "无修改"})

    def _send_snapshot(self, cam_key):
        cap = open_camera(is_qr=(cam_key == "qr"))
        if cap is None:
            self.send_error(503, f"{cam_key} camera open failed")
            return
        try:
            frame = None
            for _ in range(10):
                frame = CameraManager.read(cap, cam_key, max_retries=3)
                if frame is not None:
                    break
                time.sleep(0.1)
            if frame is None:
                self.send_error(500, "no frame")
                return
            ok, buf = cv2.imencode('.jpg', frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
            if not ok:
                self.send_error(500, "encode failed")
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(buf)))
            self.end_headers()
            self.wfile.write(buf.tobytes())
        finally:
            CameraManager.release(cap)

    def _send_stream(self, cam_key):
        # 同一摄像头只保留一路推流：先停掉旧的，避免多连接争抢同一设备
        _stop_stream(cam_key)

        cap = open_camera(is_qr=(cam_key == "qr"))
        if cap is None:
            self.send_error(503, f"{cam_key} camera open failed")
            return

        stream = _CameraStream(cam_key, cap, self.connection)
        with _streams_lock:
            _streams[cam_key] = stream

        self.send_response(200)
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Pragma", "no-cache")
        self.end_headers()
        try:
            while not stream.stop.is_set():
                frame = CameraManager.read(cap, cam_key, max_retries=3)
                if frame is None:
                    time.sleep(0.05)
                    continue
                ok, buf = cv2.imencode('.jpg', frame,
                                       [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if not ok:
                    continue
                self.wfile.write(f"--{BOUNDARY}\r\n".encode())
                self.wfile.write(b"Content-Type: image/jpeg\r\n\r\n")
                self.wfile.write(buf.tobytes())
                self.wfile.write(b"\r\n")
                time.sleep(0.03)  # 上限约 30fps
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass  # 浏览器关闭标签页 / 退出任务主动断开
        finally:
            with _streams_lock:
                if _streams.get(cam_key) is stream:
                    del _streams[cam_key]
            CameraManager.release(cap)

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        qs = parse_qs(parsed.query)
        if path in ("/", "/index.html"):
            self._send_html()
        elif path == "/stream":
            self._send_stream(self._cam_key())
        elif path == "/snapshot.jpg":
            self._send_snapshot(self._cam_key())
        elif path == "/scan":
            self._handle_scan(qs.get("method", ["opencv"])[0])
        elif path == "/hostage":
            self._handle_hostage()
        elif path == "/abort":
            self._handle_abort()
        elif path == "/line_status":
            self._handle_line_status()
        elif path == "/line_start":
            self._handle_line_start()
        elif path == "/line_stop":
            self._handle_line_stop()
        elif path == "/log":
            self._handle_log()
        elif path == "/config":
            self._handle_config_get()
        else:
            self.send_error(404, "not found")

    def do_POST(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path == "/config":
            self._handle_config_post()
        else:
            self.send_error(404, "not found")

    def log_message(self, fmt, *args):
        pass  # 静音访问日志，避免刷屏


def start_server(host="0.0.0.0", port=WEB_CONTROL_PORT):
    """启动 Web 控制页（后台线程，非阻塞）。返回 HTTPServer 对象。

    由 main.py 调用 —— 运行 main.py 即开启 web 服务器。
    """
    server = ThreadingHTTPServer((host, port), CameraHandler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print(f"[INFO] GKS 摄像头控制页已启动: http://{host}:{port}/")
    for ip in _local_ips():
        print(f"        局域网: http://{ip}:{port}/")
    return server


def main():
    parser = argparse.ArgumentParser(description="GKS 摄像头 Web 控制页")
    parser.add_argument("--host", default="0.0.0.0", help="监听地址（默认 0.0.0.0）")
    parser.add_argument("--port", type=int, default=WEB_CONTROL_PORT,
                        help=f"监听端口（默认 {WEB_CONTROL_PORT}）")
    args = parser.parse_args()

    server = start_server(args.host, args.port)
    print("        按 Ctrl+C 停止")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INFO] 停止服务")
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
