# -*- coding: utf-8 -*-
"""二维码算法跑分：同一批图片，wechat 与自研各跑一遍，出成功率 + 耗时表。

⚠️ **A/B 要跑在 Pi 上**，理由两条：
  1. Pi 的 Web `/scan` 端点 method 白名单里**没有副摄 wechat 那一档**
     （只有 opencv/pyzbar/wechat_main），拿它对比等于「主摄 wechat vs 副摄自研」，
     相机都不同，差异说明不了算法好坏；
  2. wechat 需要带 contrib 的 cv2（`cv2.wechat_qrcode`），开发机上通常没有，
     而 Pi 上有 —— 所以只有 Pi 能同时跑两边。
自研那一侧本身不依赖 Pi（只要 cv2+numpy），所以在开发机上也能单独跑，方便快速迭代。

用法（Pi 或 PC 都行）:
    python3 bench.py                       # 用内置的种子样张
    python3 bench.py <目录或图片> ...
    python3 bench.py --known 123 <目录>

输出：每张图的两边结果 + 耗时，末尾一行汇总（成功率 / 平均耗时）。
"""
import glob
import os
import sys
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
import _qr_diff as D

# 赛题二维码内容固定是三位数、每位 1~3 → 111..333 共 27 种
DEFAULT_KNOWN = '123'
# 单帧给自己留的上限（跑分时给足；在线链路由 config.QR_DIFF_TIMEOUT 限制）
BENCH_DEADLINE_S = 120.0

SEED_GLOBS = [
    '//192.168.159.125/shared/Desktop/Desktop_shared_seed_*.jpg',   # 占位，不存在也无妨
    r'C:\Users\Administrator\Desktop\qr_oblique_offline\live_main_*.jpg',
    r'C:\Users\Administrator\Desktop\pic\畸形二维码.jpg',
    r'C:\Users\Administrator\Desktop\pic\跟顺眼的二维码.jpg',
]

_WD = None
_WD_TRIED = False
_WD_ERR = ''


def _wechat_model_dirs():
    """按优先级找 wechat 模型目录（Pi 上就是 config.WECHAT_QR_MODEL_DIR）。"""
    out = []
    try:
        import config
        out.append(config.WECHAT_QR_MODEL_DIR)
    except Exception:
        pass
    out += [
        os.path.join(_HERE, 'models', 'wechat_qrcode'),
        os.path.join(os.path.dirname(_HERE), 'models', 'wechat_qrcode'),
        '//192.168.159.125/shared/Desktop/GKS_Competition/models/wechat_qrcode',
    ]
    return out


def wechat_ready():
    """返回 (解码函数, 错误说明)。两边都可能是 None / 空串。"""
    global _WD, _WD_TRIED, _WD_ERR
    if not _WD_TRIED:
        _WD_TRIED = True
        if not hasattr(cv2, 'wechat_qrcode_WeChatQRCode'):
            _WD_ERR = ('当前 cv2 %s 没有 cv2.wechat_qrcode（普通 opencv-python 不带，'
                       '要 opencv-contrib-python）' % cv2.__version__)
        else:
            d = next((p for p in _wechat_model_dirs() if os.path.isdir(p)), None)
            if d is None:
                _WD_ERR = '找不到 wechat 模型目录（models/wechat_qrcode）'
            else:
                try:
                    _WD = cv2.wechat_qrcode_WeChatQRCode(
                        os.path.join(d, 'detect.prototxt'),
                        os.path.join(d, 'detect.caffemodel'),
                        os.path.join(d, 'sr.prototxt'),
                        os.path.join(d, 'sr.caffemodel'))
                except Exception as e:
                    _WD_ERR = '加载模型失败(%s): %s' % (d, e)
    return _WD, _WD_ERR


def wechat_decode(frame):
    det, _err = wechat_ready()
    if det is None:
        return None
    try:
        res, _pts = det.detectAndDecode(frame)
    except Exception:
        return None
    for t in res or []:
        t = (t or '').strip()
        if t:
            return t
    return None


def imread_u(path):
    """np.fromfile + imdecode：非 ASCII 路径（中文文件名）也能读。"""
    img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise IOError('cannot read: ' + path)
    return img


def collect(args):
    files = []
    for a in args:
        if os.path.isdir(a):
            for ext in ('*.jpg', '*.jpeg', '*.png', '*.bmp'):
                files += sorted(glob.glob(os.path.join(a, ext)))
        elif any(c in a for c in '*?['):
            files += sorted(glob.glob(a))
        else:
            files.append(a)
    return files


def main(files, known):
    contents = D.parse_known_spec(known)
    det, werr = wechat_ready()
    print('候选集 %r -> %d 个 (%s ... %s)' % (known, len(contents), contents[0], contents[-1]))
    print('wechat: %s' % ('可用' if det is not None else '不可用 - ' + werr))
    print()
    hdr = '%-26s %10s %8s %10s %8s  %s' % (
        '样张', 'wechat', '耗时', '自研', '耗时', '自研备注')
    print(hdr)
    print('-' * len(hdr))
    w_ok = d_ok = 0
    w_t = d_t = 0.0
    for p in files:
        try:
            img = imread_u(p)
        except IOError as e:
            print('%-26s  %s' % (os.path.basename(p)[:26], e))
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        t0 = time.time()
        w = wechat_decode(img) if det is not None else None
        wt = time.time() - t0

        t0 = time.time()
        r = D.decode_gray(gray, contents=contents,
                          deadline=time.time() + BENCH_DEADLINE_S)
        dt = time.time() - t0

        w_ok += bool(w)
        d_ok += bool(r['ok'])
        w_t += wt
        d_t += dt
        note = ('score=%.3f hits=%s' % (r['score'], r['hits'])) if r['ok'] else r['reason']
        print('%-26s %10s %7.2fs %10s %7.2fs  %s' % (
            os.path.basename(p)[:26],
            w if w else '-', wt,
            r['text'] if r['ok'] else '-', dt,
            note))

    n = len(files)
    if not n:
        print('没有图片')
        return
    print()
    if det is not None:
        print('成功率  wechat %d/%d   自研 %d/%d' % (w_ok, n, d_ok, n))
        print('平均耗时 wechat %.2fs   自研 %.2fs' % (w_t / n, d_t / n))
    else:
        print('成功率  自研 %d/%d  （wechat 本机不可用，未对比）' % (d_ok, n))
        print('平均耗时 自研 %.2fs' % (d_t / n))


if __name__ == '__main__':
    argv = sys.argv[1:]
    known = DEFAULT_KNOWN
    if '--known' in argv:
        i = argv.index('--known')
        known = argv[i + 1]
        del argv[i:i + 2]
    files = collect(argv) if argv else collect(SEED_GLOBS)
    if not files:
        print('没有找到图片')
        sys.exit(1)
    main(files, known)
