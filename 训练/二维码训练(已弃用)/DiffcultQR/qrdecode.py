# -*- coding: utf-8 -*-
"""V1 QR 解码器: 去掩码 + codeword 提取 + RS 纠错(重写版) + 载荷解析"""
import numpy as np

# ---------- GF(256) ----------
GF_EXP = [0] * 512
GF_LOG = [0] * 256
_x = 1
for i in range(255):
    GF_EXP[i] = _x
    GF_LOG[_x] = i
    _x <<= 1
    if _x & 0x100:
        _x ^= 0x11d
for i in range(255, 512):
    GF_EXP[i] = GF_EXP[i - 255]


def gf_mul(a, b):
    if a == 0 or b == 0:
        return 0
    return GF_EXP[GF_LOG[a] + GF_LOG[b]]


def gf_div(a, b):
    if a == 0:
        return 0
    return GF_EXP[(GF_LOG[a] - GF_LOG[b]) % 255]


def gf_pow(a, p):
    return GF_EXP[(GF_LOG[a] * p) % 255]


def gf_inv(a):
    return GF_EXP[255 - GF_LOG[a]]


def gf_sub(a, b):
    return a ^ b


# 多项式: 高次在前
def poly_scale(p, x):
    return [gf_mul(c, x) for c in p]


def poly_add(p, q):
    r = [0] * max(len(p), len(q))
    for i, c in enumerate(p):
        r[i + len(r) - len(p)] = c
    for i, c in enumerate(q):
        r[i + len(r) - len(q)] ^= c
    return r


def poly_mul(p, q):
    r = [0] * (len(p) + len(q) - 1)
    for j, qj in enumerate(q):
        for i, pi in enumerate(p):
            r[i + j] ^= gf_mul(pi, qj)
    return r


def poly_eval(p, x):
    y = p[0]
    for c in p[1:]:
        y = gf_mul(y, x) ^ c
    return y


def poly_div(dividend, divisor):
    msg_out = list(dividend)
    for i in range(len(dividend) - (len(divisor) - 1)):
        coef = msg_out[i]
        if coef != 0:
            for j in range(1, len(divisor)):
                if divisor[j] != 0:
                    msg_out[i + j] ^= gf_mul(divisor[j], coef)
    sep = -(len(divisor) - 1)
    return msg_out[:sep], msg_out[sep:]


def rs_calc_syndromes(msg, nsym):
    return [0] + [poly_eval(msg, gf_pow(2, i)) for i in range(nsym)]


def find_error_locator(synd, nsym):
    err_loc = [1]
    old_loc = [1]
    synd_shift = len(synd) - nsym if len(synd) > nsym else 0
    for i in range(nsym - synd_shift):
        K = i + synd_shift
        delta = synd[K]
        for j in range(1, len(err_loc)):
            delta ^= gf_mul(err_loc[-(j + 1)], synd[K - j])
        if delta != 0:
            if len(old_loc) > len(err_loc):
                new_loc = poly_scale(old_loc, delta)
                old_loc = poly_scale(err_loc, gf_inv(delta))
                err_loc = new_loc
            err_loc = poly_add(err_loc, poly_scale(old_loc, delta))
    while len(err_loc) and err_loc[0] == 0:
        del err_loc[0]
    return err_loc


def find_errors(err_loc, nmess):
    errs = len(err_loc) - 1
    err_pos = []
    for i in range(nmess):
        if poly_eval(err_loc, gf_pow(2, i)) == 0:
            err_pos.append(nmess - 1 - i)
    if len(err_pos) != errs:
        return None
    return err_pos


def find_errata_locator(e_pos):
    e_loc = [1]
    for i in e_pos:
        e_loc = poly_mul(e_loc, poly_add([1], [gf_pow(2, i), 0]))
    return e_loc


def find_error_evaluator(synd, err_loc, nsym):
    _, rem = poly_div(poly_mul(synd, err_loc), [1] + [0] * (nsym + 1))
    return rem


def correct_errata(msg_in, synd, err_pos):
    msg = list(msg_in)
    coef_pos = [len(msg) - 1 - p for p in err_pos]
    err_loc = find_errata_locator(coef_pos)
    err_eval = find_error_evaluator(synd[::-1], err_loc, len(err_loc) - 1)[::-1]
    X = [gf_pow(2, p) for p in coef_pos]
    E = [0] * len(msg)
    Xlen = len(X)
    for i, Xi in enumerate(X):
        Xi_inv = gf_inv(Xi)
        err_loc_prime = 1
        for j in range(Xlen):
            if j != i:
                err_loc_prime = gf_mul(err_loc_prime,
                                       gf_sub(1, gf_mul(Xi_inv, X[j])))
        y = poly_eval(err_eval[::-1], Xi_inv)
        y = gf_mul(gf_pow(Xi, 1), y)
        magnitude = gf_div(y, err_loc_prime)
        E[err_pos[i]] = magnitude
    return poly_add(msg, E)


def rs_correct(msg_in, nsym):
    msg = list(msg_in)
    synd = rs_calc_syndromes(msg, nsym)
    if max(synd) == 0:
        return msg, 0
    err_loc = find_error_locator(synd, nsym)
    errs = len(err_loc) - 1
    if errs * 2 > nsym:
        return None, -1
    err_pos = find_errors(err_loc, len(msg))
    if err_pos is None:
        return None, -1
    msg = correct_errata(msg, synd, err_pos)
    if max(rs_calc_syndromes(msg, nsym)) != 0:
        return None, -1
    return msg, len(err_pos)


# ---------- QR 结构 ----------
def function_map(N=21):
    fm = np.zeros((N, N), bool)
    fm[0:9, 0:9] = True
    fm[0:9, N - 8:] = True
    fm[N - 8:, 0:9] = True
    fm[6, :] = True
    fm[:, 6] = True
    fm[13, 8] = True
    return fm


def mask_funcs():
    return [
        lambda r, c: (r + c) % 2 == 0,
        lambda r, c: r % 2 == 0,
        lambda r, c: c % 3 == 0,
        lambda r, c: (r + c) % 3 == 0,
        lambda r, c: (r // 2 + c // 3) % 2 == 0,
        lambda r, c: (r * c) % 2 + (r * c) % 3 == 0,
        lambda r, c: ((r * c) % 2 + (r * c) % 3) % 2 == 0,
        lambda r, c: ((r + c) % 2 + (r * c) % 3) % 2 == 0,
    ]


def extract_bits(matrix, N=21):
    fm = function_map(N)
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
                    bits.append(int(matrix[r, c]))
        upward = not upward
        col -= 2
    return bits


def bits_to_codewords(bits):
    cw = []
    for i in range(0, len(bits) - 7, 8):
        v = 0
        for b in bits[i:i + 8]:
            v = (v << 1) | b
        cw.append(v)
    return cw


def parse_payload(cw):
    bits = []
    for v in cw:
        bits += [(v >> (7 - i)) & 1 for i in range(8)]
    pos = 0

    def take(n):
        nonlocal pos
        v = 0
        for i in range(n):
            v = (v << 1) | (bits[pos + i] if pos + i < len(bits) else 0)
        pos += n
        return v

    mode = take(4)
    if mode == 0b0001:
        n = take(10)
        s = ''
        while n >= 3:
            s += '%03d' % take(10)
            n -= 3
        if n == 2:
            s += '%02d' % take(7)
        elif n == 1:
            s += '%d' % take(4)
        return s
    elif mode == 0b0010:
        n = take(9)
        chars = '0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ $%*+-./:'
        s = ''
        while n >= 2:
            v = take(11)
            s += chars[v // 45] + chars[v % 45]
            n -= 2
        if n == 1:
            s += chars[take(6)]
        return s
    elif mode == 0b0100:
        n = take(8)
        bs = bytes(take(8) for _ in range(n))
        try:
            return bs.decode('utf-8')
        except UnicodeDecodeError:
            return bs.decode('latin1')
    return f'<mode {mode:04b}>'


ECC_INFO = {
    'L': dict(total=26, data=19),
    'M': dict(total=26, data=16),
    'Q': dict(total=26, data=13),
    'H': dict(total=26, data=9),
}


def try_decode(matrix):
    N = matrix.shape[0]
    fm = function_map(N)
    masks = mask_funcs()
    results = []
    for ecc, info in ECC_INFO.items():
        nsym = info['total'] - info['data']
        for mi, mf in enumerate(masks):
            un = matrix.copy()
            for r in range(N):
                for c in range(N):
                    if not fm[r, c] and mf(r, c):
                        un[r, c] ^= 1
            ub = extract_bits(un, N)
            ucw = bits_to_codewords(ub)
            if len(ucw) < info['total']:
                continue
            fixed, nerr = rs_correct(ucw, nsym)
            if fixed is None:
                continue
            data_cw = fixed[:info['data']]
            if all(v == 0 for v in data_cw):
                continue
            try:
                text = parse_payload(data_cw)
            except Exception:
                continue
            if text and not text.startswith('<mode'):
                results.append((nerr, ecc, mi, text))
    results.sort()
    return results


if __name__ == '__main__':
    import sys
    path = sys.argv[1] if len(sys.argv) > 1 else 'matrix10.npy'
    m = np.load(path)
    print('matrix:', path)

    def dih(x):
        return [np.rot90(x, k) for k in range(4)] + [np.fliplr(np.rot90(x, k)) for k in range(4)]

    allres = []
    for di, d in enumerate(dih(m)):
        for r in try_decode(d):
            allres.append((r[0], di, r[1], r[2], r[3]))
    allres.sort()
    for r in allres[:10]:
        print(f'RS纠{r[0]}个错 姿态{r[1]} ECC={r[2]} mask={r[3]} 内容={r[4]!r}')
    if not allres:
        print('全部失败')
