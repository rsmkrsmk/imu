#!/usr/bin/env python3
"""Final calibration sweep at HP=1.0Hz using refs derived from measured per-stroke
distributions (measure.py). Goal: SS spreads 0..100 (calm typical ~45-65, dynamic
strong ~85-100); cadence ~45-75 spm single-leg; instS not permanently pinned."""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import sim_hp

CALM='imu_data/ses00038.rwl'; DYN='imu_data/ses00040.rwl'

def show(tag, **kw):
    print(f"\n=== {tag} ===")
    for p in (CALM,DYN):
        r=sim_hp.run(p,1.0,**kw)
        print(f"{os.path.basename(p):16} strokes={r['strokes']:3} spm={r['spm']:5.1f} cad={r['cad_ms']:.0f}ms | "
              f"SS p50={r['ss_p50']:5.1f} p95={r['ss_p95']:5.1f} max={r['ss_max']:5.1f} | "
              f"instS p95={r['si_p95']:5.1f} max={r['si_max']:5.1f}")

# Detector thresholds fixed at the sane-cadence point; sweep REFS for SS spread.
DET=dict(ENTER_SURGE=20.0, ENTER_HP=0.8, CONF_SURGE=40.0, CONF_HP=1.3, REFRACT_US=450000)

for refs in (
    dict(REF_PEAK=4.0, REF_IMP=0.25, REF_SURGE=110.0),
    dict(REF_PEAK=5.0, REF_IMP=0.30, REF_SURGE=130.0),
    dict(REF_PEAK=6.0, REF_IMP=0.35, REF_SURGE=150.0),
):
    show(f"refs {refs}", **DET, **refs)
