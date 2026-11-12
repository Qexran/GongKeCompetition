# -*- coding: utf-8 -*-
"""终极暴力: 候选前缀 + 标准填充 -> 完整数据流 -> RS验证完好ECC"""
import numpy as np, os, sys
from reedsolo import RSCodec
import qrdecode as q

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

def build_data_stream(first_bits, ndata):
    """候选前缀 + 终止符 + 标准填充 -> ndata 字节"""
    bits = list(first_bits)
    bits += [0, 0, 0, 0]  # 终止符
    if len(bits) % 8:
        bits += [0] * (8 - len(bits) % 8)
    pad_alt = [[1,1,1,0,1,1,0,0], [0,0,0,1,0,0,0,1]]  # 0xEC 0x11
    i = 0
    while len(bits) < ndata * 8:
        bits += pad_alt[i % 2]
        i += 1
    out = []
    for k in range(ndata):
        v = 0
        for b in bits[k*8:(k+1)*8]:
            v = (v << 1) | b
        out.append(v)
    return bytes(out)

def candidates(nbits):
    """生成前 nbits 位的候选 (结构化)"""
    yield [0]*4  # 终止符开头(空内容)
    # numeric: 4+10 头 + 内容
    hdr_len = 14
    content_bits = nbits - hdr_len
    if content_bits >= 0:
        maxg = 1 << min(content_bits, 20)
        for ln in range(1, 35):
            for g in range(maxg):
                b = [0,0,0,1] + [(ln>>(9-i))&1 for i in range(10)]
                gb = []
                for i in range(content_bits):
                    gb.append((g >> (content_bits-1-i)) & 1)
                yield b + gb

def dih(x):
    return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]

def run(mat, damage_pred):
    N = mat.shape[0]
    fm = q.function_map(N)
    masks = q.mask_funcs()
    results = []
    for ori, base in enumerate(dih(mat)):
        for ecc_name, info in q.ECC_INFO.items():
            nsym = info['total'] - info['data']
            ndata = info['data']
            rsc = RSCodec(nsym)
            for mi, mf in enumerate(masks):
                un = base.copy()
                for r in range(N):
                    for c in range(N):
                        if not fm[r, c] and mf(r, c):
                            un[r, c] ^= 1
                ucw = np.array(q.bits_to_codewords(q.extract_bits(un, N)))
                # 完好 codeword: 不含损坏区位
                good = np.ones(len(ucw), bool)
                for cw in range(len(ucw)):
                    for b in range(8):
                        bi = cw*8+b
                        if bi >= len(BITS): break
                        r, c = BITS[bi]
                        if damage_pred(r, c):
                            good[cw] = False
                good_ecc = good[ndata:]
                if good_ecc.sum() < 5:
                    continue
                # 完好的数据codeword也用于过滤
                good_data = good[:ndata]
                if good_data.sum() < ndata * 0.2:
                    continue
                hits = []
                for cand in candidates(24):
                    data = build_data_stream(cand, ndata)
                    # 与完好数据codeword比对
                    md = sum(1 for j in range(ndata) if good_data[j] and data[j] != ucw[j])
                    if md > 1:
                        continue
                    enc = rsc.encode(data)
                    me = sum(1 for j in range(nsym) if good_ecc[j] and enc[ndata+j] != ucw[ndata+j])
                    if me == 0:
                        hits.append(data)
                if hits:
                    for data in hits[:3]:
                        t = q.parse_payload(list(data))
                        print(f'*** 姿态{ori} ECC={ecc_name} mask={mi}: 内容={t!r} ***')
                        results.append((ori, ecc_name, mi, t))
    return results

if __name__ == '__main__':
    mat = np.load(sys.argv[1] if len(sys.argv) > 1 else 'matrix_cd.npy')
    res = run(mat, lambda r, c: r >= 9 and c >= 13)
    print()
    print('结论:', set(x[3] for x in res) if res else '无命中')
