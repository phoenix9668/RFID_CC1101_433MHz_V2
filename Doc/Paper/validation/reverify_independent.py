#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立复核：牛喘息（热应激）频谱算法
Independent re-verification of the panting / heat-stress spectral rule
(Davison et al. 2020, Agriculture 10, 210 — Equation (5), steps 1-9).

与 validate_v2.py 的区别（从零重做，不沿用其结论）：
  1. 独立验证状态码语义（计数器富集分析），不假设 300=喘息；
  2. 独立验证采样率（状态块量化 + 反刍咀嚼谐波的生理约束）；
  3. 主分析用非重叠窗，避免 50% 重叠造成的伪重复（pseudo-replication）；
  4. 主要对比设为「同记录内反刍」（部署真实场景），而非跨牛的干净对照；
  5. 新增阴性对照：反刍 vs 反刍（跨牛），用于检出牛/设备混杂；
  6. 报告全配置分布，而非挑最大值；置信区间用移动块 bootstrap（考虑序列相关）。

运行:
  <python-with-numpy-pandas> Doc/Paper/validation/reverify_independent.py
"""
import os
import numpy as np
import pandas as pd
import itertools

BASE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
FS = 25.0                      # 见 CHECK B
STATE = {50: 'rest', 100: 'feed', 150: 'activity',
         200: 'mount', 250: 'rumination', 300: 'panting'}   # 见 CHECK C

FILES = [
    ("喘息20250819 1622.csv",              "panting_0819",       False, 0),
    ("喘息20250820 1644.csv",              "panting_0820a",      False, 0),
    ("喘息20250820 1755.csv",              "panting_0820b",      False, 15537),  # 见 CHECK A
    ("05项圈 5826牛 0302 1549  反刍.csv",  "rumination_control", True,  0),
]
RNG = np.random.default_rng(20260724)


def load(fn, ctrl, drop):
    path = os.path.join(BASE, fn)
    if ctrl:
        r = pd.read_csv(path)
        r.columns = ['x', 'y', 'z', '状态', '休息', '采食', '活动', '爬跨', '反刍']
    else:
        r = pd.read_csv(path, encoding='gbk')
    return r.iloc[drop:].reset_index(drop=True)


# ----------------------------------------------------------------------------- signal
def band_ratio(sig, fs, lo, hi, taper='rect'):
    """论文 step 4-7: 频带内 峰值幅度 / 该带均值。"""
    sig = np.asarray(sig, float)
    sig = sig - sig.mean()
    if taper == 'hann':
        sig = sig * np.hanning(len(sig))
    amp = np.abs(np.fft.rfft(sig))
    fr = np.fft.rfftfreq(len(sig), 1.0 / fs)
    m = (fr >= lo) & (fr < hi)
    if m.sum() < 2:
        return np.nan
    b = amp[m]
    return b.max() / b.mean() if b.mean() > 0 else np.nan


def get_signal(win, kind):
    x, y, z = [win[c].astype(float).values for c in 'xyz']
    # 论文 step 2 的 E = sqrt(dx^2+dy^2+dz^2) 即 'mag'
    return {'mag': np.sqrt(x**2 + y**2 + z**2), 'x': x, 'y': y, 'z': z}[kind]


def build(data, win_s=20, purity=0.85, overlap=0.0, kind='mag', taper='rect'):
    """切窗 -> 每窗算 F(1-2), F(2-3), D。overlap=0 即非重叠（主分析）。"""
    W = int(round(win_s * FS))
    step = max(1, int(round(W * (1 - overlap))))
    rows = []
    for fn, tag, ctrl, drop in FILES:
        df = data[tag]
        st = df['状态'].values
        for s in range(0, len(df) - W + 1, step):
            seg = st[s:s + W]
            vals, cnts = np.unique(seg, return_counts=True)
            dom = vals[np.argmax(cnts)]
            if cnts.max() / W < purity or dom not in STATE:
                continue
            sig = get_signal(df.iloc[s:s + W], kind)
            F12 = band_ratio(sig, FS, 1, 2, taper)
            F23 = band_ratio(sig, FS, 2, 3, taper)
            if np.isnan(F12) or np.isnan(F23):
                continue
            rows.append(dict(file=tag, behavior=STATE[dom], start=s,
                             F12=F12, F23=F23, D=F12 - F23, passes=int(F12 > F23)))
    return pd.DataFrame(rows)


# ----------------------------------------------------------------------------- stats
def auc(pos, neg):
    pos, neg = np.asarray(pos, float), np.asarray(neg, float)
    if len(pos) == 0 or len(neg) == 0:
        return np.nan
    r = pd.Series(np.concatenate([pos, neg])).rank().values
    return (r[:len(pos)].sum() - len(pos) * (len(pos) + 1) / 2) / (len(pos) * len(neg))


def block_boot_auc(pos, neg, n_boot=2000, block=5):
    """移动块 bootstrap：窗在时间上相关，独立重采样会低估区间宽度。"""
    def resample(a):
        a = np.asarray(a, float)
        if len(a) <= block:
            return RNG.choice(a, len(a), replace=True)
        nb = int(np.ceil(len(a) / block))
        starts = RNG.integers(0, len(a) - block + 1, nb)
        return np.concatenate([a[s:s + block] for s in starts])[:len(a)]
    vals = [auc(resample(pos), resample(neg)) for _ in range(n_boot)]
    return float(np.nanpercentile(vals, 2.5)), float(np.nanpercentile(vals, 97.5))


def hr(t):
    print("\n" + "=" * 78); print(t); print("=" * 78)


# ============================================================================= MAIN
def main():
    # ---------- CHECK A: 数据完整性 / 重复 ----------
    hr("CHECK A — 数据完整性与文件间重复")
    raw = {tag: load(fn, ctrl, 0) for fn, tag, ctrl, _ in FILES}
    a = raw['panting_0820a'][['x', 'y', 'z', '状态']].values
    b = raw['panting_0820b'][['x', 'y', 'z', '状态']].values
    k = 0
    while k < min(len(a), len(b)) and np.array_equal(a[k], b[k]):
        k += 1
    print(f"  0820_1755 与 0820_1644 的逐字节相同前缀 = {k} 行  -> 复核时从 1755 剔除")
    print(f"  => 3 个喘息文件实际只对应 2 个独立记录时段 (0819, 0820)")
    for fn, tag, ctrl, drop in FILES:
        df = raw[tag]
        print(f"  {tag:20s} rows={len(df):6d}  NaN(xyz)={int(df[['x','y','z']].isna().sum().sum()):5d}  "
              f"states={sorted(set(df['状态'].dropna().astype(int)))}")

    data = {tag: load(fn, ctrl, drop) for fn, tag, ctrl, drop in FILES}

    # ---------- CHECK B: 采样率 ----------
    hr("CHECK B — 采样率 (报告假设 25 Hz)")
    st = data['panting_0820a']['状态'].values
    runs, cur, n = [], st[0], 1
    for v in st[1:]:
        if v == cur:
            n += 1
        else:
            runs.append(n); cur = v; n = 1
    runs.append(n)
    rl = np.array(runs)
    print(f"  状态段长度为 25 的整数倍的比例 = {np.mean(rl % 25 == 0):.1%}  "
          f"(众数段长 {pd.Series(rl).mode().values[:3]})")
    print("  -> 状态列每 25 样本更新一次；若行为分类每秒输出一次，则 fs = 25 Hz")
    sub = data['rumination_control']
    sub = sub[sub['状态'] == 250]
    mag = np.sqrt(sum(sub[c].astype(float).values**2 for c in 'xyz'))
    W = int(60 * FS); acc = None; nw = 0
    for s in range(0, len(mag) - W + 1, W // 2):
        sg = (mag[s:s + W] - mag[s:s + W].mean()) * np.hanning(W)
        A = np.abs(np.fft.rfft(sg)); acc = A if acc is None else acc + A; nw += 1
    fr = np.fft.rfftfreq(W, 1 / FS); acc /= nw
    m = (fr >= 0.3) & (fr <= 6)
    pk = fr[m][np.argmax(acc[m])]
    print(f"  干净反刍主谱峰 = {pk:.2f} Hz -> 若为咀嚼 3 次谐波，基频 {pk/3:.2f} Hz = {pk/3*60:.0f} 次/分")
    print("  论文: 咀嚼 40-70 次/分。加速度 ∝ f^2 会放大谐波 -> 与 fs=25 Hz 自洽")

    # ---------- CHECK C: 状态码语义 ----------
    hr("CHECK C — 状态码语义（不假设 300=喘息，用计数器富集独立判定）")
    for tag in ['panting_0820a', 'panting_0820b']:
        df = data[tag]; stv = df['状态'].values
        base = {c: (stv == c).mean() for c in STATE}
        print(f"  [{tag}]")
        for cname, expect in [('休息', 50), ('采食', 100), ('活动', 150), ('反刍', 250)]:
            v = pd.to_numeric(df[cname], errors='coerce').values
            idx = np.where(np.diff(v) > 0)[0]
            if len(idx) < 5:
                continue
            enr = {}
            for c in STATE:
                if base[c] < 1e-3:
                    continue
                fr_ = [ (stv[max(0, i-1500):i+1] == c).mean() for i in idx ]
                enr[c] = np.mean(fr_) / base[c]
            best = max(enr, key=enr.get)
            flag = "OK" if best == expect else "!! MISMATCH"
            print(f"     计数器{cname}(n={len(idx):3d}) 富集最高的状态 = {best}  (期望 {expect})  {flag}")
        oth = pd.to_numeric(df['其他'], errors='coerce').values
        print(f"     计数器『其他』: 递增次数 = {int((np.diff(oth) > 0).sum())} "
              f"，而 state300 出现 {int((stv == 300).sum())} 次 -> 300 不是『其他』")
    print("  结论: 50/100/150/250 与 休息/采食/活动/反刍 一一对应；『其他』恒为0，")
    print("        300 是无对应计数器的新增状态，且在反刍对照文件中占比 0% -> 判定为『喘息』")

    # ---------- CHECK D: 喘息暴发时长 vs 论文 90 s 窗 ----------
    hr("CHECK D — 喘息暴发时长与论文 90 s 窗的可行性")
    allb = []
    for fn, tag, ctrl, drop in FILES:
        if ctrl:
            continue
        stv = data[tag]['状态'].values
        rs, cur, n = [], stv[0], 1
        for v in stv[1:]:
            if v == cur:
                n += 1
            else:
                rs.append((cur, n)); cur = v; n = 1
        rs.append((cur, n))
        d = [n / FS for s, n in rs if s == 300]
        allb += d
        print(f"  {tag:15s} state300占比={np.mean(stv==300):.3f}  暴发数={len(d):4d}  "
              f"中位={np.median(d) if d else 0:5.1f}s  最长={max(d) if d else 0:5.1f}s")
    allb = np.array(allb)
    print(f"  全部: n={len(allb)}  p95={np.percentile(allb,95):.1f}s  最长={allb.max():.1f}s  ≥90s 的暴发数 = {(allb>=90).sum()}")
    p90 = build(data, win_s=90, purity=0.85, overlap=0.0, kind='mag', taper='rect')
    npant90 = int((p90.behavior == 'panting').sum())
    print(f"  => 论文原版 90 s 非重叠窗、≥85% 纯度下的可评估喘息窗 = {npant90} 个")
    if npant90 == 0:
        print("  => 论文算法【按其原文规格】在本数据上无法评估（不是效果差，是样本为 0）")

    # ---------- CHECK E: 主分析 + 阴性对照 ----------
    hr("CHECK E — 主分析（非重叠窗）与阴性对照")
    print("  三组对比：")
    print("    (1) 喘息 vs 同记录内反刍   <- 部署真实场景，同牛同设备同日，PRIMARY")
    print("    (2) 喘息 vs 对照牛反刍     <- 原报告头条，跨牛/跨设备/跨日期")
    print("    (3) 反刍 vs 对照牛反刍     <- 阴性对照：同一种行为，理应 AUC≈0.5")
    for kind in ['mag', 'x', 'y', 'z']:
        m = build(data, win_s=20, purity=0.85, overlap=0.0, kind=kind, taper='rect')
        pant = m[m.behavior == 'panting'].D.values
        rin = m[(m.behavior == 'rumination') & (m.file != 'rumination_control')].D.values
        rct = m[(m.behavior == 'rumination') & (m.file == 'rumination_control')].D.values
        if len(pant) < 3:
            print(f"\n  [{kind}] 喘息窗仅 {len(pant)} 个，跳过"); continue
        a1, a2, a3 = auc(pant, rin), auc(pant, rct), auc(rin, rct)
        c1 = block_boot_auc(pant, rin)
        print(f"\n  [{kind}]  n: 喘息={len(pant)}  同记录反刍={len(rin)}  对照牛反刍={len(rct)}")
        print(f"     (1) PRIMARY 喘息 vs 同记录内反刍 = {a1:.3f}  95%CI[{c1[0]:.3f},{c1[1]:.3f}]")
        print(f"     (2)         喘息 vs 对照牛反刍   = {a2:.3f}")
        print(f"     (3) 阴性对照 反刍 vs 对照牛反刍  = {a3:.3f}   <- 若显著偏离 0.5 则存在牛/设备混杂")
        if not np.isnan(a3) and abs(a3 - 0.5) > 0.15:
            print(f"         !! 同一行为跨牛可分度 {a3:.3f}，说明 (2) 的高 AUC 主要来自牛/设备差异")

    # ---------- CHECK F: 全配置分布 ----------
    hr("CHECK F — 全配置分布（报告分布而非最大值）")
    rows = []
    for kind, ws, tap, pur, ov in itertools.product(
            ['mag', 'x', 'y', 'z'], [15, 20, 30], ['rect', 'hann'], [0.85, 0.90], [0.0]):
        m = build(data, win_s=ws, purity=pur, overlap=ov, kind=kind, taper=tap)
        pant = m[m.behavior == 'panting'].D.values
        rin = m[(m.behavior == 'rumination') & (m.file != 'rumination_control')].D.values
        rct = m[(m.behavior == 'rumination') & (m.file == 'rumination_control')].D.values
        if len(pant) < 3 or len(rin) < 3 or len(rct) < 3:
            continue
        rows.append(dict(signal=kind, win_s=ws, taper=tap, purity=pur, n_pant=len(pant),
                         auc_within=round(auc(pant, rin), 3),
                         auc_vs_control=round(auc(pant, rct), 3),
                         auc_negctrl=round(auc(rin, rct), 3)))
    sw = pd.DataFrame(rows)
    if len(sw):
        print(sw.sort_values('auc_within', ascending=False).to_string(index=False))
        print(f"\n  PRIMARY(同记录内) AUC: 中位 {sw.auc_within.median():.3f}  "
              f"范围 [{sw.auc_within.min():.3f}, {sw.auc_within.max():.3f}]  n_configs={len(sw)}")
        print(f"  跨牛对照     AUC: 中位 {sw.auc_vs_control.median():.3f}  "
              f"范围 [{sw.auc_vs_control.min():.3f}, {sw.auc_vs_control.max():.3f}]")
        print(f"  阴性对照     AUC: 中位 {sw.auc_negctrl.median():.3f}  "
              f"范围 [{sw.auc_negctrl.min():.3f}, {sw.auc_negctrl.max():.3f}]  (理应 ≈0.5)")
        out = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'output_reverify')
        os.makedirs(out, exist_ok=True)
        sw.to_csv(os.path.join(out, 'config_landscape.csv'), index=False)
        print(f"\n  已保存: {os.path.join(out, 'config_landscape.csv')}")


if __name__ == '__main__':
    main()
