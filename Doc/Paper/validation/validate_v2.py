#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pandas as pd, numpy as np, json, math

FS = 25.0
STATE = {50:'rest',100:'feed',150:'activity',200:'mount',250:'rumination',300:'panting'}
BASE="/sessions/lucid-festive-edison/mnt/Paper/"
FILES = [
    ("喘息20250819 1622.csv", "panting_0819_1622", False, 0),
    ("喘息20250820 1644.csv", "panting_0820_1644", False, 0),
    ("喘息20250820 1755.csv", "panting_0820_1755", False, 15537),
    ("05项圈 5826牛 0302 1549  反刍.csv", "rumination_control", True, 0),
]
def load(fn,ctrl,drop):
    if ctrl:
        r=pd.read_csv(BASE+fn); r.columns=['x','y','z','状态','休息','采食','活动','爬跨','反刍']
    else:
        r=pd.read_csv(BASE+fn,encoding='gbk')
    return r.iloc[drop:].reset_index(drop=True)
def band_ratio(sig,fs,lo,hi,taper='rect'):
    sig=sig-np.mean(sig); n=len(sig)
    if taper=='hann': sig=sig*np.hanning(n)
    amp=np.abs(np.fft.rfft(sig)); freq=np.fft.rfftfreq(n,d=1.0/fs)
    m=(freq>=lo)&(freq<hi)
    if m.sum()<2: return np.nan,np.nan
    b=amp[m]; return (b.max()/b.mean() if b.mean()>0 else np.nan), freq[m][np.argmax(b)]
def metrics(sig,fs,taper='rect'):
    F12,f12=band_ratio(sig,fs,1,2,taper); F23,f23=band_ratio(sig,fs,2,3,taper)
    return F12,F23,f12*60,f23*60
def get_signal(win,kind='mag'):
    x,y,z=win['x'].astype(float).values,win['y'].astype(float).values,win['z'].astype(float).values
    return {'mag':np.sqrt(x**2+y**2+z**2),'x':x,'y':y,'z':z}[kind]
def pure_windows(df,win_s,purity,overlap,fs=FS):
    W=int(round(win_s*fs)); step=max(1,int(W*(1-overlap))); st=df['状态'].values; out=[]
    for s in range(0,len(df)-W+1,step):
        seg=st[s:s+W]; vals,cnts=np.unique(seg,return_counts=True)
        dom=vals[np.argmax(cnts)]; frac=cnts.max()/W
        if frac>=purity and dom in STATE: out.append((STATE[dom],df.iloc[s:s+W]))
    return out
def wilson(k,n,z=1.96):
    if n==0: return (float('nan'),float('nan'))
    p=k/n; d=1+z*z/n; c=(p+z*z/(2*n))/d; h=z*math.sqrt(p*(1-p)/n+z*z/(4*n*n))/d
    return (max(0,c-h),min(1,c+h))

data={tag:load(fn,ctrl,drop) for fn,tag,ctrl,drop in FILES}

def run_pure(win_s=20,purity=0.85,overlap=0.5,signal_kind='mag',taper='rect'):
    rows=[]
    for fn,tag,ctrl,drop in FILES:
        for beh,win in pure_windows(data[tag],win_s,purity,overlap):
            sig=get_signal(win,signal_kind); F12,F23,f12,f23=metrics(sig,FS,taper)
            if np.isnan(F12) or np.isnan(F23): continue
            rows.append(dict(file=tag,behavior=beh,F12=F12,F23=F23,passes=int(F12>F23),peak12_bpm=f12,peak23_bpm=f23))
    return pd.DataFrame(rows)

main=run_pure()
print("=== ANALYSIS 1: behavior-pure 20s windows (85% purity), magnitude signal ===")
summ=main.groupby('behavior').agg(n=('passes','size'),rule_positive=('passes','sum'),
    pass_rate=('passes','mean'),median_F12=('F12','median'),median_F23=('F23','median'),
    median_peak12_bpm=('peak12_bpm','median')).round(3)
print(summ.to_string())
pant=main[main.behavior=='panting']['passes']; rumin=main[main.behavior=='rumination']['passes']
others=main[main.behavior.isin(['rest','feed','activity'])]['passes']
res=dict(panting_n=int(pant.size),panting_pass=int(pant.sum()),
  panting_sensitivity=float(pant.mean()) if pant.size else None,panting_ci=wilson(int(pant.sum()),int(pant.size)),
  rumination_n=int(rumin.size),rumination_reject=int((1-rumin).sum()),
  rumination_specificity=float((1-rumin).mean()) if rumin.size else None,rumination_ci=wilson(int((1-rumin).sum()),int(rumin.size)),
  other_n=int(others.size),other_specificity=float((1-others).mean()) if others.size else None)
print("\nPanting sensitivity: %s CI=%s"%(round(res['panting_sensitivity'],3),tuple(round(x,3) for x in res['panting_ci'])))
print("Rumination specificity: %s CI=%s"%(round(res['rumination_specificity'],3),tuple(round(x,3) for x in res['rumination_ci'])))
print("Rest/feed/activity specificity: %s (n=%d)"%(round(res['other_specificity'],3),res['other_n']))
if res['panting_sensitivity'] and res['rumination_specificity']:
    res['balanced_accuracy']=0.5*(res['panting_sensitivity']+res['rumination_specificity'])
    print("Balanced accuracy (panting vs rumination): %.3f"%res['balanced_accuracy'])

def run_90s():
    rows=[]; W=int(90*FS)
    for fn,tag,ctrl,drop in FILES:
        df=data[tag]; st=df['状态'].values
        for s in range(0,len(df)-W+1,W):
            seg=df.iloc[s:s+W]; segst=st[s:s+W]; vals,cnts=np.unique(segst,return_counts=True)
            dom=vals[np.argmax(cnts)]; frac=cnts.max()/W; sig=get_signal(seg,'mag'); F12,F23,f12,f23=metrics(sig,FS)
            rows.append(dict(file=tag,majority_state=STATE.get(dom,str(dom)),majority_frac=round(frac,2),
                panting_frac=round((segst==300).mean(),2),F12=F12,F23=F23,passes=int(F12>F23)))
    return pd.DataFrame(rows)
paper90=run_90s()
print("\n=== ANALYSIS 2: paper-faithful 90s windows (majority-vote 状态) ===")
print("Windows by majority behavior:",paper90.majority_state.value_counts().to_dict())
print("Mean panting-fraction within 90s windows: %.2f (max %.2f)"%(paper90.panting_frac.mean(),paper90.panting_frac.max()))
if paper90.panting_frac.std()>0:
    print("Corr(panting_fraction, rule_passes): %.3f"%paper90[['panting_frac','passes']].corr().iloc[0,1])

print("\n=== ANALYSIS 3: sensitivity (pant sens / rumin spec) ===")
sens=[]
for kind in ['mag','x','y','z']:
    for ws in [15,20,30]:
        for tap in ['rect','hann']:
            m=run_pure(win_s=ws,purity=0.85,overlap=0.5,signal_kind=kind,taper=tap)
            p=m[m.behavior=='panting']['passes']; r=m[m.behavior=='rumination']['passes']
            sens.append(dict(signal=kind,win_s=ws,taper=tap,pant_n=int(p.size),
                pant_sens=round(p.mean(),3) if p.size else None,rumin_n=int(r.size),
                rumin_spec=round((1-r).mean(),3) if r.size else None))
sens_df=pd.DataFrame(sens); print(sens_df.to_string(index=False))

O="/sessions/lucid-festive-edison/mnt/outputs/"
main.to_csv(O+'windows_pure.csv',index=False); paper90.to_csv(O+'windows_90s.csv',index=False)
sens_df.to_csv(O+'sensitivity_v2.csv',index=False); summ.to_csv(O+'behavior_summary.csv')
json.dump({'sample_rate_hz':FS,'state_map':STATE,'main':res,
    'behavior_summary':summ.reset_index().to_dict('records')},
    open(O+'results_v2.json','w'),ensure_ascii=False,indent=2,default=str)
print("\nSaved outputs.")
