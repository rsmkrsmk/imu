#!/usr/bin/env python3
"""Measure per-stroke peak/surge/impulse distributions at HP=1.0 with moderate
thresholds, to pick REF_* directly from the data."""
import sys, os
sys.path.insert(0, os.path.dirname(__file__))
import sim_hp

for p in ('imu_data/ses00038.rwl','imu_data/ses00040.rwl'):
    # moderate detector so we get clean per-stroke stats
    r = sim_hp.run(p, 1.0,
        REF_PEAK=2.0, REF_IMP=0.15, REF_SURGE=80.0,
        ENTER_SURGE=15.0, ENTER_HP=0.6, CONF_SURGE=30.0, CONF_HP=1.0,
        REFRACT_US=350000)
    print(f"{os.path.basename(p):16} strokes={r['strokes']:3} spm={r['spm']:5.1f} cad={r['cad_ms']:.0f}ms")
    print(f"    peak(hpAf,g) p50={r['pk_p50']:.3f} p95={r['pk_p95']:.3f}")
    print(f"    surge(g/s)   p50={r['sg_p50']:.1f} p95={r['sg_p95']:.1f}")
    print(f"    impulse(g*s) p50={r['im_p50']:.4f} p95={r['im_p95']:.4f}")
