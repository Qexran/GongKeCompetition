# -*- coding: utf-8 -*-
"""攒样张：从 Pi 的摄像头抓一帧，存进 qr_corpus/ —— 给 bench_qr.py 做算法对比用。

⚠️ **在 PC 上跑**（它通过 HTTP 找 Pi 要图）。Pi 上跑的是 bench_qr.py。

⚠️ 为什么需要它：现成的 Web `/scan` 每点一次都会顺带跑**一整轮识别**
   （自研那条要好几秒），而 `logs/qr_last.jpg` 是每次**覆盖写**的 —— 两个都攒不出样本集。
   而 `/snapshot.jpg` 是**只采不解**的，正好。

⚠️ 采之前先停 Pi 页面的预览（页面上的「退出」按钮）：摄像头是**跨进程独占**设备，
   推流线程占着它时 `/snapshot.jpg` 会开相机失败（EBUSY）。

用法:
    python grab_qr.py 正视15度              # 主摄抓一张，标签写进文件名
    python grab_qr.py 斜视35度 --cam qr     # 抓副摄那张
    python grab_qr.py 亮 --pi 192.168.159.125 --dir D:\\qr_corpus
抓完一批后，把它们拷到 Pi 上再跑 bench_qr.py（或直接指目录）。
"""
import argparse
import os
import sys
import time
import urllib.request

DEFAULT_PI = '192.168.159.125'
DEFAULT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'qr_corpus')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('label', help='这张图代表什么（角度/光照…），会写进文件名')
    ap.add_argument('--cam', choices=('main', 'qr'), default='main',
                    help='哪台相机（默认 main=主摄）')
    ap.add_argument('--pi', default=DEFAULT_PI, help='Pi 的地址')
    ap.add_argument('--port', type=int, default=9000)
    ap.add_argument('--dir', default=DEFAULT_DIR, help='存到哪')
    ap.add_argument('--note', default='', help='附注，写进文件名后缀')
    args = ap.parse_args()

    safe = ''.join(c for c in args.label if c.isalnum() or c in '-_')
    url = 'http://%s:%d/snapshot.jpg?cam=%s' % (args.pi, args.port, args.cam)
    try:
        with urllib.request.urlopen(url, timeout=20) as r:
            data = r.read()
    except Exception as e:
        print('[ERROR] 取帧失败: %s: %s' % (type(e).__name__, e))
        print('        · Pi 的 Web 控制端开着吗？（%s:%d）' % (args.pi, args.port))
        print('        · 页面上的摄像头预览停了吗？设备是跨进程独占的，'
              '推流占着就开不了相机')
        sys.exit(1)
    if not data or data[:2] != b'\xff\xd8':
        print('[ERROR] 拿到的不是 JPEG（%d 字节）' % len(data))
        sys.exit(1)

    os.makedirs(args.dir, exist_ok=True)
    stamp = time.strftime('%H%M%S')
    name = '%s_%s%s%s.jpg' % (safe or 'shot', args.cam,
                              ('_' + args.note) if args.note else '', '_' + stamp)
    path = os.path.join(args.dir, name)
    with open(path, 'wb') as f:
        f.write(data)
    print('已保存 %s  (%d 字节)' % (path, len(data)))


if __name__ == '__main__':
    main()
