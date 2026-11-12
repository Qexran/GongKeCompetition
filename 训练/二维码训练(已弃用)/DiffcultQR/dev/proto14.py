# -*- coding: utf-8 -*-
"""proto14: 分块相位相关自一致精化 + 解码"""
import cv2
import numpy as np

N, PPM, QZ = 21, 40, 6
S = (N + 2 * QZ) * PPM
OFF = QZ * PPM

import proto12 as p12
from reedsolo import RSCodec, ReedSolomonError
import qrdecode as q


def pred_x(coef_x, u, v):
    return OFF + u * PPM + float((p12.cubic_design(np.array([u]), np.array([v])) @ coef_x)[0])


def pred_y(coef_y, u, v):
    return OFF + v * PPM + float((p12.cubic_design(np.array([u]), np.array([v])) @ coef_y)[0])


def sample_field(warp, coef_x, coef_y, dcf=None):
    """dcf: (r,c) -> (dx,dy) 修正(像素), 可选"""
    vals = np.zeros((N, N), np.float32)
    for r in range(N):
        for c in range(N):
            xs = [pred_x(coef_x, k, r + 1.0) for k in (c, c + 1)]
            ys = [pred_y(coef_y, c + 1.0, k) for k in (r, r + 1)]
            xc, yc = (xs[0] + xs[1]) / 2, (ys[0] + ys[1]) / 2
            if dcf is not None:
                dx, dy = dcf(r, c)
                xc, yc = xc + dx, yc + dy
            pts = []
            for ddy in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                for ddx in np.linspace(-0.15 * PPM, 0.15 * PPM, 5):
                    xi, yi = int(xc + ddx), int(yc + ddy)
                    if 0 <= xi < S and 0 <= yi < S:
                        pts.append(warp[yi, xi])
            vals[r, c] = np.median(pts)
    return vals


def render_synth(mat, coef_x, coef_y):
    """把采样矩阵渲染到 cubic 网格 -> 合成图"""
    img = np.full((S, S), 255.0, np.float64)
    for r in range(N):
        for c in range(N):
            xs = [pred_x(coef_x, k, r + 1.0) for k in (c, c + 1)]
            ys = [pred_y(coef_y, c + 1.0, k) for k in (r, r + 1)]
            x0, x1 = int(round(xs[0])), int(round(xs[1]))
            y0, y1 = int(round(ys[0])), int(round(ys[1]))
            if mat[r, c]:
                img[y0:y1, x0:x1] = 0.0
    return img


def block_phase_offsets(warp, synth, coef_x, coef_y, bs=5):
    """分块相位相关: 返回 [(uc, vc, dx, dy, peak)]"""
    warp_f = warp.astype(np.float64)
    out = []
    for r0 in range(0, N - bs + 1, bs):
        for c0 in range(0, N - bs + 1, bs):
            # 块中心 (module)
            uc = c0 + bs / 2
            vc = r0 + bs / 2
            # 取 warp 中该块的像素块 (按网格映射的包围盒+余量)
            xs = [pred_x(coef_x, k, vc) for k in (c0, c0 + bs)]
            ys = [pred_y(coef_y, uc, k) for k in (r0, r0 + bs)]
            x0, y0 = int(round(min(xs))), int(round(min(ys)))
            x1, y1 = int(round(max(xs))), int(round(max(ys)))
            if x0 < 0 or y0 < 0 or x1 > S or y1 > S:
                continue
            a = warp_f[y0:y1, x0:x1]
            b = synth[y0:y1, x0:x1]
            if a.shape != b.shape or min(a.shape) < 100:
                continue
            hann = cv2.createHanningWindow(a.shape[::-1], cv2.CV_64F)
            try:
                (dx, dy), resp = cv2.phaseCorrelate(a * hann, b * hann)
            except cv2.error:
                continue
            if resp > 0.05:
                out.append((uc, vc, dx, dy, resp))
    return out


def make_dcf(offsets, sigma=4.0):
    """从块偏移构造平滑修正场"""
    if not offsets:
        return None

    def dcf(r, c):
        u = c + 0.5
        v = r + 0.5
        num_x = num_y = den = 0.0
        for (uc, vc, dx, dy, resp) in offsets:
            d2 = (u - uc) ** 2 + (v - vc) ** 2
            w = resp * np.exp(-d2 / (2 * sigma * sigma))
            num_x += w * dx
            num_y += w * dy
            den += w
        if den < 1e-9:
            return 0.0, 0.0
        return num_x / den, num_y / den
    return dcf


def try_decode2(matrix):
    Nn = matrix.shape[0]
    fm = q.function_map(Nn)
    masks = q.mask_funcs()
    results = []
    for ecc, info in q.ECC_INFO.items():
        nsym = info['total'] - info['data']
        rsc = RSCodec(nsym)
        for mi, mf in enumerate(masks):
            un = matrix.copy()
            for r in range(Nn):
                for c in range(Nn):
                    if not fm[r, c] and mf(r, c):
                        un[r, c] ^= 1
            ucw = q.bits_to_codewords(q.extract_bits(un, Nn))
            if len(ucw) < info['total']:
                continue
            try:
                dec, full, errata = rsc.decode(bytes(ucw))
            except ReedSolomonError:
                continue
            if all(v == 0 for v in dec):
                continue
            try:
                text = q.parse_payload(list(dec))
            except Exception:
                continue
            if text and not text.startswith('<mode'):
                results.append((len(errata), ecc, mi, text))
    results.sort()
    return results


def main():
    gray, Hp2w = p12.load_warp()
    warp = cv2.warpPerspective(gray, Hp2w.astype(np.float32), (S, S), flags=cv2.INTER_CUBIC)
    coef_x, coef_y = p12.fit_cubic(warp) if hasattr(p12, 'fit_cubic') else (None, None)
    if coef_x is None:
        # proto12 没有 fit_cubic, 手动跑
        from proto12 import collect_anchors, cubic_design, robust_fit
        coef_x = np.zeros(10)
        coef_y = np.zeros(10)
        for rnd in range(3):
            anchors = collect_anchors(warp, coef_x, coef_y, cubic_design)
            au = np.array([a[0] for a in anchors], np.float64)
            av = np.array([a[1] for a in anchors], np.float64)
            ax = np.array([a[2] for a in anchors], np.float64)
            ay = np.array([a[3] for a in anchors], np.float64)
            aw = np.array([a[4] for a in anchors], np.float64)
            A = cubic_design(au, av)
            coef_x = robust_fit(A, ax - (OFF + au * PPM), aw)
            coef_y = robust_fit(A, ay - (OFF + av * PPM), aw)

    gt = np.load('gt.npy')
    def dih(x):
        return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]

    dcf = None
    for it in range(4):
        vals = sample_field(warp, coef_x, coef_y, dcf)
        _, bw = cv2.threshold(np.clip(vals, 0, 255).astype(np.uint8), 0, 255,
                              cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        mat = (bw < 127).astype(np.uint8)
        best = min((int(np.count_nonzero(mat != d)), i) for i, d in enumerate(dih(gt)))
        res = try_decode2(mat)
        print(f'iter{it}: vs133={best[0]} 解码={res[:3] if res else "fail"}')
        if res:
            np.save('matrix_win.npy', mat)
            print('*** 解码成功 ***')
            return mat, res
        synth = render_synth(mat, coef_x, coef_y)
        offs = block_phase_offsets(warp, synth, coef_x, coef_y, bs=5)
        print(f'  块偏移: {len(offs)} 块, 平均|d|={np.mean([abs(o[2])+abs(o[3]) for o in offs]) if offs else 0:.1f}px')
        dcf = make_dcf(offs, sigma=3.5)
    np.save('matrix_pc.npy', mat)
    return mat, None


if __name__ == '__main__':
    main()
