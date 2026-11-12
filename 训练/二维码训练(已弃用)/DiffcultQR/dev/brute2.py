# -*- coding: utf-8 -*-
"""结构化暴力补全: 枚举合法(mode,len)前缀, RS验证"""
import numpy as np
import os, sys
from reedsolo import RSCodec, ReedSolomonError
import qrdecode as q

D = os.path.dirname(os.path.abspath(__file__))
mat = np.load(os.path.join(D, sys.argv[1] if len(sys.argv) > 1 else 'matrix_cd.npy'))
N = 21

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
nbits = len(BITS)

def damaged_cw(mask_pred):
    nb = nbits // 8
    out = np.zeros(nb, bool)
    for cw in range(nb):
        for b in range(8):
            bi = cw * 8 + b
            if bi >= nbits:
                break
            r, c = BITS[bi]
            if mask_pred(r, c):
                out[cw] = True
    return out

fm = q.function_map(N)
masks = q.mask_funcs()

# 候选位流生成器 (24 bits)
def candidates24():
    # mode=0: 空内容
    bits = [0]*4 + [0]*20
    yield bits
    # mode=1 numeric: 4 + 10(len) + 10(第一组数字)
    for ln in range(1, 35):
        for g in range(1024):
            b = [0,0,0,1]
            b += [(ln >> (9-i)) & 1 for i in range(10)]
            b += [(g >> (9-i)) & 1 for i in range(10)]
            yield b
    # mode=2 alnum: 4 + 9(len) + 11(一对字符)
    for ln in range(1, 26):
        for g in range(2048):
            b = [0,0,1,0]
            b += [(ln >> (8-i)) & 1 for i in range(9)]
            b += [(g >> (10-i)) & 1 for i in range(11)]
            yield b
    # mode=4 byte: 4 + 8(len) + 12(1.5字节)
    for ln in range(1, 18):
        for g in range(4096):
            b = [0,1,0,0]
            b += [(ln >> (7-i)) & 1 for i in range(8)]
            b += [(g >> (11-i)) & 1 for i in range(12)]
            yield b

def bits_to_bytes(bits):
    v = 0
    for b in bits:
        v = (v << 1) | b
    return v

found = {}
def dih(x):
    return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]

for ori, base_m in enumerate(dih(mat)):
  for ecc_name, info in q.ECC_INFO.items():
    nsym = info['total'] - info['data']
    ndata = info['data']
    rsc = RSCodec(nsym)
    for mi, mf in enumerate(masks):
        un = base_m.copy()
        for r in range(N):
            for c in range(N):
                if not fm[r, c] and mf(r, c):
                    un[r, c] ^= 1
        ub = q.extract_bits(un, N)
        ucw = q.bits_to_codewords(ub)
        # 损坏区: BR
        dmg = damaged_cw(lambda r, c: r >= 9 and c >= 13)
        lead_bad = 0
        for i in range(ndata):
            if dmg[i]:
                lead_bad = i + 1
            else:
                break
        if lead_bad < 1:
            try:
                dec, full, errata = rsc.decode(bytes(ucw))
                if any(v != 0 for v in dec):
                    t = q.parse_payload(list(dec))
                    if not t.startswith('<mode'):
                        found[(ecc_name, mi)] = (len(errata), t)
            except Exception:
                pass
            continue
        if 1 <= lead_bad <= 3:
            nbitsx = lead_bad * 8
            suffix_bits = ub[nbitsx : ndata * 8]
            suffix_bytes = []
            for i in range(0, len(suffix_bits) - 7, 8):
                v = 0
                for b in suffix_bits[i:i+8]:
                    v = (v << 1) | b
                suffix_bytes.append(v)
            suffix = bytes(suffix_bytes)
            ecc_obs = ucw[ndata:]
            ecc_trusted = ~dmg[ndata:]
            hits = []
            for cand in candidates24():
                cand = cand[:nbitsx]
                pre = [bits_to_bytes(cand[i*8:(i+1)*8]) for i in range(lead_bad)]
                msg = bytes(pre) + suffix
                msg = msg[:ndata]
                if len(msg) < ndata:
                    msg = msg + bytes(ndata - len(msg))
                enc = rsc.encode(msg)
                ok = True
                for j in range(nsym):
                    if ecc_trusted[j] and enc[ndata + j] != ecc_obs[j]:
                        ok = False
                        break
                if ok:
                    hits.append(msg)
            if hits:
                for msg in hits[:3]:
                    try:
                        t = q.parse_payload(list(msg[:ndata]))
                    except Exception:
                        t = '<parse err>'
                    print(f'姿态{ori} ECC={ecc_name} mask={mi} 命中! 内容={t!r} (候选数{len(hits)})')
                    found[(ecc_name, mi, ori)] = (0, t)
        print(f'姿态{ori} ECC={ecc_name} mask={mi} lead_bad={lead_bad} done', file=sys.stderr)

print()
print('==== 结果 ====')
for k, v in found.items():
    print(k, v)
if not found:
    print('无命中')
