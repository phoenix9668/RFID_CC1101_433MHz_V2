#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
固件喘息检测器的位精确离线模型
Bit-exact model of the panting detector implemented in Core/Src/adxl362.c.

复刻的是固件里的定点链路本身（同样的 Q14 系数、S=11 加窗移位、拆分乘法、
uint64 功率累加、整数开方、无除法判决），用来在真实项圈数据上回放，
而不是一个"参考实现"。任何与固件的差异都算 bug。

用法:
  <python-with-numpy-pandas> Doc/Paper/validation/pant_firmware_model.py

注意：`Doc/Paper/validation/独立复核报告.md` 的复核结论是该论文判据在这批
数据上判别力接近随机。若本脚本给出很低的可分性，那是数据/算法的性质，
不是实现缺陷——不要为了好看去调参。
"""
import os
import numpy as np
import pandas as pd

BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")

# ---- 必须与 Core/Src/adxl362.c 保持一致 ----
FS, N, S_SHIFT = 25.0, 150, 11
WIN_SEG = 3                                  # _PANT_WIN_SEG
GATE_NUM, GATE_DEN = 2, 1                    # _PANT_GATE_NUM / _PANT_GATE_DEN
BAND_LO = np.array([0, 1, 2, 3, 4])          # k= 6..10  (k=11 保护带)
BAND_HI = np.array([7, 8, 9, 10, 11])        # k=13..17  (k=12 保护带)
NBAND = 5                                    # _PANT_BAND_BINS

COEF = np.array([int(round(2 * np.cos(2 * np.pi * k / N) * 16384)) for k in range(6, 18)], dtype=np.int64)
HANN = np.array([int(round(32767 * 0.5 * (1 - np.cos(2 * np.pi * n / N)))) for n in range(N)], dtype=np.int64)

FILES = [
    ("喘息20250819 1622.csv",             "panting_0819",       False, 0),
    ("喘息20250820 1644.csv",             "panting_0820a",      False, 0),
    ("喘息20250820 1755.csv",             "panting_0820b",      False, 15537),
    ("05项圈 5826牛 0302 1549  反刍.csv", "rumination_control", True,  0),
]


def load(fn, ctrl, drop):
    path = os.path.join(BASE, fn)
    if ctrl:
        r = pd.read_csv(path)
        r.columns = ['x', 'y', 'z', '状态', '休息', '采食', '活动', '爬跨', '反刍']
    else:
        r = pd.read_csv(path, encoding='gbk')
    return r.iloc[drop:].reset_index(drop=True)


def coef_mul(c, s):
    """PantCoefMul(): 拆分乘法，与 (c*s)>>14 位等价（>> 对负数向下取整）。"""
    return c * (s >> 14) + ((c * (s & 0x3FFF)) >> 14)


def isqrt(n):
    """PantIsqrt(): floor(sqrt(n))。"""
    n = np.asarray(n, dtype=np.int64)
    r = np.floor(np.sqrt(n.astype(np.float64))).astype(np.int64)
    r = np.where((r + 1) * (r + 1) <= n, r + 1, r)      # 修正浮点向下偏差
    return np.where(r * r > n, r - 1, r)


def seg_power(x, y, z):
    """PantWindowAdd(): 一个 150 样本段 -> 12 个 bin 的功率。"""
    mag = isqrt(x.astype(np.int64) ** 2 + y.astype(np.int64) ** 2 + z.astype(np.int64) ** 2)
    mean = (int(mag.sum()) + 75) // 150
    xw = ((mag - mean) * HANN) >> S_SHIFT
    s1 = np.zeros(12, dtype=np.int64)
    s2 = np.zeros(12, dtype=np.int64)
    for n in range(N):
        s0 = coef_mul(COEF, s1) - s2 + xw[n]
        s2, s1 = s1, s0
    t = coef_mul(COEF, s1)
    p = s1 * s1 + s2 * s2 - t * s2
    return np.maximum(p, 0)


def decide(acc):
    """PantWindowDecide(): 返回 (panting, F12, F23)。"""
    mx = int(acc.max())
    sh, hi = 0, mx >> 32
    while hi:
        hi >>= 1
        sh += 2
    amp = isqrt(acc >> sh)
    a12, a23 = amp[BAND_LO], amp[BAND_HI]
    s12, s23 = int(a12.sum()), int(a23.sum())
    m12, m23 = int(a12.max()), int(a23.max())
    if s12 == 0 or s23 == 0:
        return 0, 0.0, 0.0
    pant = int((m12 * s23 > m23 * s12) and (NBAND * GATE_DEN * m12 >= GATE_NUM * s12))
    return pant, NBAND * m12 / s12, NBAND * m23 / s23


def run_file(df, tag):
    x = df['x'].astype(np.int64).values
    y = df['y'].astype(np.int64).values
    z = df['z'].astype(np.int64).values
    st = df['状态'].values
    nseg = len(x) // N
    rows = []
    acc = np.zeros(12, dtype=np.int64)
    cnt = 0
    for s in range(nseg):
        a, b = s * N, (s + 1) * N
        acc += seg_power(x[a:b], y[a:b], z[a:b])
        cnt += 1
        if cnt >= WIN_SEG:
            pant, F12, F23 = decide(acc)
            w0, w1 = (s - WIN_SEG + 1) * N, (s + 1) * N
            seg_st = st[w0:w1]
            rows.append(dict(file=tag, win=len(rows), pant=pant, F12=F12, F23=F23,
                             frac300=float(np.mean(seg_st == 300)),
                             frac250=float(np.mean(seg_st == 250)),
                             dominant=int(pd.Series(seg_st).mode().values[0])))
            acc[:] = 0
            cnt = 0
    return pd.DataFrame(rows)


if __name__ == '__main__':
    allw = []
    for fn, tag, ctrl, drop in FILES:
        w = run_file(load(fn, ctrl, drop), tag)
        allw.append(w)
        print(f"{tag:20s} windows={len(w):4d}  pant_flagged={w.pant.mean():6.1%}  "
              f"F12 median={w.F12.median():5.2f}")
    W = pd.concat(allw, ignore_index=True)

    print("\n=== 与固件自身 state==300 标签的对照（18 s 窗）===")
    ctrl = W[W.file == 'rumination_control']
    pf = W[W.file != 'rumination_control']
    pos = pf[pf.frac300 >= 0.5]      # 窗内多数为喘息
    neg = pf[pf.frac300 == 0.0]      # 窗内完全没有喘息
    print(f"  喘息为主的窗 (frac300>=0.5): n={len(pos):4d}  检出率 = {pos.pant.mean():6.1%}")
    print(f"  完全无喘息的窗 (frac300=0) : n={len(neg):4d}  误报率 = {neg.pant.mean():6.1%}")
    print(f"  干净反刍对照文件            : n={len(ctrl):4d}  误报率 = {ctrl.pant.mean():6.1%}")

    if len(pos) and len(neg):
        a = np.concatenate([pos.F12.values, neg.F12.values])
        r = pd.Series(a).rank().values
        auc = (r[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg))
        print(f"  F12 的 AUC(喘息窗 vs 无喘息窗，同记录内) = {auc:.3f}")

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output_reverify')
    os.makedirs(out, exist_ok=True)
    W.to_csv(os.path.join(out, 'pant_firmware_windows.csv'), index=False)
    print(f"\n逐窗结果已写入 {os.path.join(out, 'pant_firmware_windows.csv')}")
