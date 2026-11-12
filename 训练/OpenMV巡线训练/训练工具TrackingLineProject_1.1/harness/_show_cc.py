# -*- coding: utf-8 -*-
"""临时: 打印当前 column_crossing 全文（带行号）。"""
import os
HERE = os.path.dirname(os.path.abspath(__file__))
src = open(os.path.join(HERE, '..', 'main.py'), encoding='utf-8').read().splitlines()
start = next(i for i, l in enumerate(src) if l.startswith('def column_crossing'))
end = next(i for i, l in enumerate(src) if i > start and '丢线分类' in l)
out = []
for i in range(start, end):
    out.append('%4d %s' % (i + 1, src[i]))
open(os.path.join(HERE, 'out', '_cc_source.txt'), 'w', encoding='utf-8').write('\n'.join(out))
print('written', end - start, 'lines')
