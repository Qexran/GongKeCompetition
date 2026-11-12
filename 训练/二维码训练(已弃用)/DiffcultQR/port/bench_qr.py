# -*- coding: utf-8 -*-
"""二维码算法跑分：同一批图片，微信与自研各跑一遍，出成功率 + 耗时表。

⚠️ **跑在 Pi 上**（`python3 bench_qr.py <目录>`，放在 GKS_Competition 仓库根）。
   理由两条：
     1. 微信引擎需要带 contrib 的 cv2（`cv2.wechat_qrcode`），开发机上的
        `opencv-python` 通常没有，只有 Pi 有；
     2. 耗时必须在**部署目标**上测 —— 开发机 1.0s 的图，Pi 上实测 4.5s。
   自研那一侧只要 cv2+numpy，所以在开发机上也能单独跑，方便快速迭代。

⚠️ 它调的是**生产代码里的解码器**（`tasks._qr._decode_wechat` / `tasks._qr_diff`），
   不是这里另写一份 —— 否则跑分测的是影子实现，跟线上没关系。
   ⚠️ 也因此它**不测**「开相机」那段（那是任务层的 `_run_task` 干的）：这里只比算法。
      整步耗时（含开相机/回帧）看 Pi 日志里的「用时 x.xxs」。

用法:
    python3 bench_qr.py                      # 用 qr_corpus/ 里的全部样张
    python3 bench_qr.py <目录或图片> ...
    python3 bench_qr.py --known 111,222 <目录>
"""
import glob
import os
import re
import sys
import time

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

# 赛题二维码内容固定是三位数、每位 1~3 → 111..333 共 27 种
DEFAULT_KNOWN = '123'
DEFAULT_DIR = os.path.join(_HERE, 'qr_corpus')
# 跑分时给足（只测真实耗时，不是测超时）；在线链路由固件下发的时限决定
BENCH_DEADLINE_S = 120.0


def _load_module(name, relpath):
    """按**文件路径**直接加载一个模块，绕开包导入。

    ⚠️ 为什么不用 `from tasks import _qr_diff`：那会执行 `tasks/__init__.py` →
       `tasks/_qr.py` → `camera_lease` → `serial_comm` → **pyserial**，为了一个纯视觉
       模块把整条硬件栈拖进来。于是「没有 pyserial 的机器（比如开发机）」上连自研
       都跑不了，而它只依赖 cv2+numpy。按路径加载就没这回事。
    """
    import importlib.util
    path = os.path.join(_HERE, relpath)
    if not os.path.isfile(path):
        raise ImportError(f'找不到 {path}')
    spec = importlib.util.spec_from_file_location(name, path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def _wechat_fn():
    """微信解码器。优先用**生产代码**里的那份，退回自己直接建。返回 (fn, err)。"""
    import cv2 as _cv
    if not hasattr(_cv, 'wechat_qrcode_WeChatQRCode'):
        return None, f'cv2 {_cv.__version__} 没有 wechat_qrcode（要 opencv-contrib-python）'

    # ① 生产代码那条（跑分要测的就是它）
    try:
        from tasks._qr import _decode_wechat      # 需要 pyserial 等硬件依赖

        def fn(frame):
            for r in _decode_wechat(frame):
                if r.get('data'):
                    return r['data']
            return None
        return fn, ''
    except Exception as e:
        prod_err = f'{type(e).__name__}: {e}'

    # ② 退路：自己建检测器（模型目录就在仓库里，与生产用同一份）
    d = os.path.join(_HERE, 'models', 'wechat_qrcode')
    if not os.path.isdir(d):
        return None, f'生产那份 import 不了({prod_err})，且找不到模型目录 {d}'
    try:
        det = _cv.wechat_qrcode_WeChatQRCode(
            os.path.join(d, 'detect.prototxt'), os.path.join(d, 'detect.caffemodel'),
            os.path.join(d, 'sr.prototxt'), os.path.join(d, 'sr.caffemodel'))
    except Exception as e:
        return None, f'加载模型失败({d}): {e}'

    def fn(frame):
        try:
            res, _pts = det.detectAndDecode(frame)
        except Exception:
            return None
        for t in res or []:
            if (t or '').strip():
                return t.strip()
        return None
    return fn, ''


def _diff_fn():
    """自研解码器（生产代码那份，按路径加载）。返回 (fn, err)。"""
    try:
        qr_diff = _load_module('_qr_diff', os.path.join('tasks', '_qr_diff.py'))
    except Exception as e:
        return None, f'加载 tasks/_qr_diff.py 失败: {type(e).__name__}: {e}'
    state = {'contents': None, 'key': None}

    def fn(gray, known):
        if state['key'] != known:
            state['contents'] = qr_diff.parse_known_spec(known)
            state['key'] = known
        r = qr_diff.decode_gray(gray, contents=state['contents'],
                                deadline=time.time() + BENCH_DEADLINE_S)
        return (r['text'] if r['ok'] else None), r

    return fn, ''


def imread_u(path):
    """np.fromfile + imdecode：中文文件名也能读（与 Pi 侧 camera 无关，纯文件）。"""
    img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise IOError('读不出来: ' + path)
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
    return [f for f in files if not os.path.basename(f).startswith('qr_last')]


def main(files, known):
    w_fn, w_err = _wechat_fn()
    d_fn, d_err = _diff_fn()
    print('自研候选集: %r' % known)
    print('微信引擎: %s' % ('可用' if w_fn else '不可用 - ' + w_err))
    print('自研引擎: %s' % ('可用' if d_fn else '不可用 - ' + d_err))
    print()
    hdr = '%-30s %8s %9s %8s %9s  %s' % ('样张', '微信', '耗时', '自研', '耗时', '备注')
    print(hdr)
    print('-' * len(hdr))
    n = w_ok = d_ok = 0
    w_t = d_t = 0.0
    for p in files:
        try:
            img = imread_u(p)
        except IOError as e:
            print('%-30s %s' % (os.path.basename(p)[:30], e))
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        n += 1

        t0 = time.time()
        w = w_fn(img) if w_fn else None
        wt = time.time() - t0

        t0 = time.time()
        d, info = (None, None)
        if d_fn:
            d, info = d_fn(gray, known)
        dt = time.time() - t0

        w_ok += bool(w)
        d_ok += bool(d)
        w_t += wt
        d_t += dt
        note = ('hits=%s score=%.3f' % (info['hits'], info['score'] or 0)) if (info and info['ok']) \
            else (info['reason'] if info else d_err)
        print('%-30s %8s %8.2fs %8s %8.2fs  %s' % (
            os.path.basename(p)[:30], w or '-', wt, d or '-', dt, note))

    if not n:
        print('没有图片')
        return
    print()

    def pct(k):
        return '%.0f%% (%d/%d)' % (100.0 * k / n, k, n)

    if w_fn:
        print('成功率   微信 %-14s 自研 %s' % (pct(w_ok), pct(d_ok)))
        print('平均耗时 微信 %-14s 自研 %.2fs' % ('%.2fs' % (w_t / n), d_t / n))
    else:
        print('成功率   自研 %s（微信本机不可用，未对比）' % pct(d_ok))
        print('平均耗时 自研 %.2fs' % (d_t / n))


if __name__ == '__main__':
    argv = sys.argv[1:]
    known = DEFAULT_KNOWN
    if '--known' in argv:
        i = argv.index('--known')
        known = argv[i + 1]
        del argv[i:i + 2]
    if argv:
        files = collect(argv)
    elif os.path.isdir(DEFAULT_DIR):
        files = collect([DEFAULT_DIR])
    else:
        print('用法: python3 bench_qr.py <目录或图片> ...   （默认目录 %s 不存在）' % DEFAULT_DIR)
        sys.exit(1)
    if not files:
        print('没有找到图片')
        sys.exit(1)
    main(files, known)
