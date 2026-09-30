#!/usr/bin/env python3
"""Symulacja DOKLADNIE logiki firmware kadencji rytmicznej (jak w imuTask):
- petla po rekordach (nowUs z pliku), obwiednia surge max, probka co 50ms do bufora 120,
- co 1s: cadencePeriodFromEnvelope() z korekta sub-harmoniczna,
- wygladzanie CADENCE_SMOOTH, kadencja KROK = leg*2.
Potwierdza, ze firmware policzy to samo co zweryfikowany algorytm."""
import struct

FS = 20; BUF = 120; LAG_MIN = 8; LAG_MAX = 44
SMOOTH = 0.35; SUBHARM = 0.80
ENV_PERIOD_US = 50000; CALC_US = 1000000

def load(path):
    rs = 48 if path.endswith('.rwl') else 32
    si = 18 if rs == 48 else 12
    b = open(path, 'rb').read(); hb = struct.unpack_from('<H', b, 10)[0]; body = b[hb:]
    n = len(body)//rs
    FMT = "<I hhh hhh hhh hhh H h H H H h h BBB B H" if rs == 48 else "<I hhh hhh H h H H H h B B B B"
    R = [struct.unpack_from(FMT, body, i*rs) for i in range(n)]
    return [abs(r[si]/10.0) for r in R], [r[0] for r in R]

def period_from_env(buf, idx, filled):
    if filled < BUF: return 0.0
    seq = [buf[(idx+k) % BUF] for k in range(BUF)]
    m = sum(seq)/BUF; x = [v-m for v in seq]
    ac0 = sum(v*v for v in x)
    if ac0 <= 1e-6: return 0.0
    ac = [0.0]*(LAG_MAX+1); best = -1e30; blag = 0
    for lag in range(LAG_MIN, LAG_MAX+1):
        s = sum(x[k]*x[k+lag] for k in range(BUF-lag)); ac[lag] = s
        if s > best: best = s; blag = lag
    if best <= 0.15*ac0: return 0.0
    d = blag*2
    if d <= LAG_MAX and ac[d] > SUBHARM*best: blag = d
    return blag/FS

def run(path):
    su, t = load(path); t0 = t[0]
    buf = [0.0]*BUF; idx = 0; filled = 0; peak = 0.0
    lastEnv = 0; lastCalc = 0; smoothed = 0.0
    med5 = []
    series = []
    for i in range(len(t)):
        now = t[i]
        if su[i] > peak: peak = su[i]
        if lastEnv == 0: lastEnv = now
        if now - lastEnv >= ENV_PERIOD_US:
            buf[idx] = peak; idx = (idx+1) % BUF; filled = min(filled+1, BUF); peak = 0.0; lastEnv = now
        if lastCalc == 0 or now - lastCalc >= CALC_US:
            lastCalc = now
            p = period_from_env(buf, idx, filled)
            if not hasattr(run, '_hist'): pass
            if p > 0:
                stride = (60.0/p)*2
                med5.append(stride)
                if len(med5) > 5: med5.pop(0)
                ss = sorted(med5); smoothed = ss[len(ss)//2]  # mediana z 5 ostatnich okien
            else:
                med5.clear(); smoothed = 0.0
            if smoothed > 0: series.append(((now-t0)/1e6, smoothed))
    return series

for f in ['ses00052.rwl', 'ses00053.rwl', 'ses00054.pnt']:
    s = run(f)
    vals = [v for _, v in s]
    vals.sort()
    med = vals[len(vals)//2] if vals else 0
    lo = vals[int(len(vals)*0.1)] if vals else 0
    hi = vals[int(len(vals)*0.9)] if vals else 0
    print(f"{f}: KROK mediana={med:.0f}/min  (p10={lo:.0f} p90={hi:.0f})  noga~{med/2:.0f}/min")
