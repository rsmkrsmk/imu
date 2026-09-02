#!/usr/bin/env python3
"""Simulate the firmware stroke pipeline on RAW files with different HP cutoffs,
to pick HP_CUTOFF_HZ and reference constants for the patch.

We reconstruct af (forward accel, g) and dt from the RAW record, then run the
firmware's one-pole HP + surge LP, intensity and instantaneous strength.
"""
import struct, sys, math, os
from statistics import median
sys.path.insert(0, os.path.dirname(__file__))
from parse_v3 import parse_raw, pctl, HDR

PI = math.pi

def hp_update(state, x, dt, fc):
    # OnePoleHighPass.update
    if not state['init']:
        state['prevX']=x; state['init']=True; state['y']=0.0; return 0.0
    rc = 1.0/(2*PI*fc); alpha = rc/(rc+dt)
    state['y'] = alpha*(state['y'] + x - state['prevX'])
    state['prevX']=x
    return state['y']

def lp_update(state, x, dt, fc):
    if not state['init']:
        state['y']=x; state['init']=True; return x
    rc = 1.0/(2*PI*fc); alpha = dt/(rc+dt)
    state['y'] += alpha*(x-state['y'])
    return state['y']

def run(path, HP, SURGE_LP=8.0, LIVE_INT=1.5,
        W_PEAK=0.40, W_IMP=0.30, W_SURGE=0.30,
        REF_PEAK=0.55, REF_IMP=0.05, REF_SURGE=45.0,
        ENTER_SURGE=6.0, ENTER_HP=0.20, CONF_SURGE=10.0, CONF_HP=0.35,
        REFRACT_US=280000, WIN_US=120000):
    b=open(path,'rb').read(); h,recs=parse_raw(b)
    hp={'init':False,'y':0,'prevX':0}
    surgef={'init':False,'y':0}
    prevHp=0.0
    prevT=None
    hp_vals=[]; surge_vals=[]; strengthI=[]
    # stroke detector
    inCand=False; candPeakHp=0; candPeakSurge=0; candImpulse=0; candStart=0; candPeak=0; candConfirm=False
    lastStroke=0
    stroke_strengths=[]; intervals=[]; stroke_peaks=[]; stroke_surges=[]; stroke_impulses=[]
    for r in recs:
        t=r['t']
        if prevT is None:
            prevT=t; continue
        rawdt=(t-prevT)*1e-6; prevT=t
        dt=min(max(rawdt,0.002),0.050)
        af=r['af']/1000.0
        hpAf=hp_update(hp, af, dt, HP)
        surgeRaw=(hpAf-prevHp)/dt; prevHp=hpAf
        surge=lp_update(surgef, surgeRaw, dt, SURGE_LP)
        hp_vals.append(hpAf); surge_vals.append(surge)
        si=100.0*min(1.0, W_PEAK*max(0,hpAf)/REF_PEAK + W_SURGE*max(0,surge)/REF_SURGE)
        strengthI.append(si)
        # detector
        if not inCand:
            if surge>=ENTER_SURGE or hpAf>=ENTER_HP:
                inCand=True; candPeakHp=hpAf; candPeakSurge=surge; candImpulse=abs(hpAf)*dt
                candStart=t; candPeak=t; candConfirm=(surge>=CONF_SURGE or hpAf>=CONF_HP)
        else:
            if hpAf>candPeakHp: candPeakHp=hpAf; candPeak=t
            if surge>candPeakSurge: candPeakSurge=surge
            candImpulse+=abs(hpAf)*dt
            if not candConfirm and (surge>=CONF_SURGE or hpAf>=CONF_HP): candConfirm=True
            if (t-candStart)>=WIN_US:
                if candConfirm:
                    if lastStroke==0 or (candPeak-lastStroke)>=REFRACT_US:
                        nP=min(candPeakHp/REF_PEAK,2.0); nI=min(candImpulse/REF_IMP,2.0); nS=min(candPeakSurge/REF_SURGE,2.0)
                        ss=100.0*(W_PEAK*nP+W_IMP*nI+W_SURGE*nS); ss=max(0,min(100,ss))
                        stroke_strengths.append(ss)
                        stroke_peaks.append(candPeakHp); stroke_surges.append(candPeakSurge); stroke_impulses.append(candImpulse)
                        if lastStroke: intervals.append((candPeak-lastStroke)/1e3)
                        lastStroke=candPeak
                inCand=False
    span=(recs[-1]['t']-recs[0]['t'])/1e6
    spm = len(stroke_strengths)*60.0/span if span>0 else 0
    cad_ms = median(intervals) if intervals else 0
    return dict(
        recs=len(recs), span=span,
        hp_p95=pctl([abs(x) for x in hp_vals],95), hp_p999=pctl([abs(x) for x in hp_vals],99.9), hp_max=max(abs(x) for x in hp_vals),
        surge_p95=pctl([abs(x) for x in surge_vals],95), surge_p999=pctl([abs(x) for x in surge_vals],99.9), surge_max=max(abs(x) for x in surge_vals),
        si_p95=pctl(strengthI,95), si_max=max(strengthI),
        strokes=len(stroke_strengths), spm=spm,
        ss_p50=median(stroke_strengths) if stroke_strengths else 0,
        ss_p95=pctl(stroke_strengths,95) if stroke_strengths else 0,
        ss_max=max(stroke_strengths) if stroke_strengths else 0,
        cad_ms=cad_ms,
        pk_p50=median(stroke_peaks) if stroke_peaks else 0, pk_p95=pctl(stroke_peaks,95) if stroke_peaks else 0,
        sg_p50=median(stroke_surges) if stroke_surges else 0, sg_p95=pctl(stroke_surges,95) if stroke_surges else 0,
        im_p50=median(stroke_impulses) if stroke_impulses else 0, im_p95=pctl(stroke_impulses,95) if stroke_impulses else 0,
    )

if __name__=='__main__':
    files=[a for a in sys.argv[1:] if a.endswith('.rwl')]
    for HP in (20.0, 5.0, 2.0, 1.0, 0.5, 0.3):
        print(f"\n########## HP_CUTOFF = {HP} Hz ##########")
        for p in files:
            r=run(p, HP)
            print(f"{os.path.basename(p):16} n={r['recs']:5} {r['span']:5.1f}s | "
                  f"hpAf p95={r['hp_p95']:.3f} p999={r['hp_p999']:.3f} max={r['hp_max']:.3f} | "
                  f"surge p95={r['surge_p95']:5.1f} p999={r['surge_p999']:6.1f} | "
                  f"strokes={r['strokes']:3} spm={r['spm']:5.1f} cad={r['cad_ms']:.0f}ms | "
                  f"SS p50={r['ss_p50']:5.1f} p95={r['ss_p95']:5.1f} max={r['ss_max']:5.1f} | "
                  f"instS p95={r['si_p95']:4.1f} max={r['si_max']:5.1f}")
