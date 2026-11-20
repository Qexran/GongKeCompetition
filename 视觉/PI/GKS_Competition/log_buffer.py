# log_buffer.py — 内存日志缓冲
#
# 收集程序的日志输出（print + logging），供 Web 端 /log 端点实时显示。
# main.py 的 setup_logging() 把 stdout / logging 写到这里；
# camera_web.py 读取这里返回给浏览器。
import threading
from collections import deque

_MAX_LINES = 800

_lock = threading.Lock()
_lines = deque(maxlen=_MAX_LINES)


class Writer:
    """类文件对象：写入日志缓冲。

    同时供 sys.stdout 重定向（捕获 print）与 logging.StreamHandler 使用。
    """

    def write(self, s):
        if s:
            add(s)

    def flush(self):
        pass


def add(text):
    """追加一段文本到缓冲（线程安全）。"""
    with _lock:
        _lines.append(text)


def recent(n=None):
    """返回最近的日志文本。n 为 None 时返回全部。"""
    with _lock:
        if n is None:
            return "".join(_lines)
        return "".join(list(_lines)[-n:])


def clear():
    with _lock:
        _lines.clear()
