# -*- coding: utf-8 -*-
"""暴力补全: 枚举损坏的前导 codeword, 用完好的 ECC 字节验证"""
import numpy as np
import os
from reedsolo import RSCodec, ReedSolomonError
import qrdecode as q

D = os.path.dirname(os.path.abspath(__file__))
mat = np.load(os.path.join(D, 'matrix_cd.npy'))
N = 21

# bit -> (r,c) 映射
def bit_positions(N=21):
    fm = q.function_map(N)
    bits = []
    col = N - 1
    upward = True
    while col > 0:
        if col == 6:
            col -= 1
        c1, c0 = col, col - 1
        rows = range(N - 1, -1, -1) if upward else range(N)
        for r in rows:
            for c in (c1, c0):
                if not fm[r, c]:
                    bits.append((r, c))
        upward = not upward
        col -= 2
    return bits

BITS = bit_positions()
DAMAGED = [(r, c) for (r, c) in BITS if r >= 9 and c >= 13]  # BR疑似区

def cw_damage_mask():
    """每个 codeword 的损坏位占比"""
    nb = len(BITS) // 8
    mask = np.zeros(nb)
    for cw in range(nb):
        cnt = tot = 0
        for b in range(8):
            bi = cw * 8 + b
            if bi >= len(BITS):
                break
            tot += 1
            if (BITS[bi][0] >= 9 and BITS[bi][1] >= 13):
                cnt += 1
        mask[cw] = cnt / max(tot, 1)
    return mask

DMASK = cw_damage_mask()
print('codeword 损坏率:', np.round(DMASK, 2).tolist())

fm = q.function_map(N)
masks = q.mask_funcs()

for ecc_name, info in q.ECC_INFO.items():
    nsym = info['total'] - info['data']
    ndata = info['data']
    rsc = RSCodec(nsym)
    for mi, mf in enumerate(masks):
        un = mat.copy()
        for r in range(N):
            for c in range(N):
                if not fm[r, c] and mf(r, c):
                    un[r, c] ^= 1
        ub = q.extract_bits(un, N)
        ucw = q.bits_to_codewords(ub)
        # 完好的 ECC codeword 索引 (data之后)
        ecc_cw = ucw[ndata:]
        ecc_dmg = DMASK[ndata:]
        trusted = ecc_dmg < 0.25
        if trusted.sum() < 4:
            continue
        # 损坏的前导数据 codeword 数量
        lead_bad = 0
        for i in range(ndata):
            if DMASK[i] > 0.25:
                lead_bad = i + 1
        lead_bad = min(lead_bad, 3)
        if lead_bad == 0:
            continue
        print(f'--- ECC={ecc_name} mask={mi}: 完好ECC={trusted.sum()}/{nsym}, 损坏前导={lead_bad} ---')
        # 已知数据后缀
        known_suffix = ucw[lead_bad:ndata]
        # 暴力枚举前导
        best = []
        rng = range(65536) if lead_bad == 2 else (range(256) if lead_bad == 1 else [0])
        for cand in rng:
            if lead_bad == 2:
                cw0, cw1 = cand >> 8, cand & 0xFF
                msg = bytes([cw0, cw1]) + bytes(known_suffix)
            elif lead_bad == 1:
                msg = bytes([cand]) + bytes(known_suffix)
            else:
                msg = bytes(known_suffix)
            if len(msg) < ndata:
                msg = msg + bytes(ndata - len(msg))
            enc = rsc.encode(msg)
            pred_ecc = enc[ndata:]
            mism = sum(1 for j in range(nsym) if trusted[j] and pred_ecc[j] != ecc_cw[j])
            if mism == 0:
                best.append((cw0 if lead_bad == 2 else cand))
                if len(best) > 50:
                    break
        if best:
            print(f'  零失配候选数: {len(best)}')
            for b in best[:10]:
                if lead_bad == 2:
                    msg = bytes([b >> 8, b & 0xFF]) + bytes(known_suffix)
                else:
                    msg = bytes([b]) + bytes(known_suffix)
                msg = msg + bytes(ndata - len(msg))
                try:
                    print('   内容:', repr(q.parse_payload(list(msg))))
                except Exception as e:
                    pass
            if len(best) == 1:
                b = best[0]
                if lead_bad == 2:
                    msg = bytes([b >> 8, b & 0xFF]) + bytes(known_suffix)
                else:
                    msg = bytes([b]) + bytes(known_suffix)
                msg = msg + bytes(ndata - len(msg))
                print('*** 唯一解! 内容 =', repr(q.parse_payload(list(msg))), '***')
