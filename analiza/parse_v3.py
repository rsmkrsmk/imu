#!/usr/bin/env python3
"""Parser for previous-build files: RIMU03 (PNT, 23B rec) and RRAW01 (RAW, 40B rec).
Header is 46 bytes for both. Used for algorithm verification + calibration, not shipped.
"""
import struct, sys, math, os
from statistics import median

HDR = 46

def read_header(b):
    magic = b[:8].split(b'\x00')[0].decode('latin1')
    fv, hb = struct.unpack_from('<HH', b, 8)
    sid, sms = struct.unpack_from('<II', b, 12)
    rate = struct.unpack_from('<H', b, 20)[0]
    axes = struct.unpack_from('<8b', b, 22)
    bias = struct.unpack_from('<fff', b, 30)
    return dict(magic=magic, fv=fv, hb=hb, sid=sid, rate=rate, axes=axes, bias=bias)

# RAW v1 record (40 B) — derived empirically:
# u32 tUs
# i16 rawAx,rawAy,rawAz (mg)
# i16 rawGx,rawGy,rawGz (dps*10)
# i16 af,al,au (mg)
# i16 gf,gl,gu (dps*10)
# i16 accNorm (mg)
# i16 lean_cdeg
# u16 intensity_x10
# u16 strokeStrength_x10
# i16 surge_dps10
# then 4 tail bytes: hypothesis -> u16 cadenceMs? , u8 strokePhaseMs?, u8 flags/side
# We'll unpack tail flexibly.
RAW = struct.Struct('<I hhh hhh hhh hhh h h H H h')  # up to surge = 4 + 2*17 = 38 bytes; 2 tail bytes remain
# Actually 4 + 3*2*4 + 2 + 2 + 2 + 2 + 2 = 4 + 24 + 10 = 38 -> 2 bytes tail.

def parse_raw(b):
    h = read_header(b); body = b[HDR:]; REC=40; n=len(body)//REC
    recs=[]
    for i in range(n):
        r = body[i*REC:(i+1)*REC]
        vals = struct.unpack_from('<I hhh hhh hhh hhh h h H H h', r, 0)
        tUs = vals[0]
        rawA = vals[1:4]; rawG = vals[4:7]
        af,al,au = vals[7:10]
        gf,gl,gu = vals[10:13]
        accNorm = vals[13]; lean=vals[14]; inten=vals[15]; strk=vals[16]; surge=vals[17]
        tail = r[38:40]
        recs.append(dict(t=tUs, rawA=rawA, rawG=rawG, af=af,al=al,au=au,
                         gf=gf,gl=gl,gu=gu, accNorm=accNorm, lean=lean,
                         inten=inten, strk=strk, surge=surge, tail=tail))
    return h, recs

def pctl(xs, p):
    if not xs: return 0
    s=sorted(xs); k=(len(s)-1)*p/100.0
    lo=int(math.floor(k)); hi=int(math.ceil(k))
    if lo==hi: return s[lo]
    return s[lo]*(hi-k)+s[hi]*(k-lo)

def analyze_raw(path):
    b=open(path,'rb').read(); h,recs=parse_raw(b)
    print(f"\n===== {os.path.basename(path)}  magic={h['magic']} recs={len(recs)} =====")
    print(f"header bias(dps)={tuple(round(x,3) for x in h['bias'])} axes={h['axes']}")
    ts=[r['t'] for r in recs]
    span=(ts[-1]-ts[0])/1e6
    print(f"span={span:.1f}s  eff_rate={ (len(recs)-1)/span:.1f}Hz")
    # dt distribution
    dts=[(ts[i]-ts[i-1])/1e3 for i in range(1,len(ts))]
    print(f"dt ms: p50={median(dts):.2f} p90={pctl(dts,90):.2f} p99={pctl(dts,99):.2f} max={max(dts):.2f}")
    # raw accel scale sanity (g)
    accn=[r['accNorm']/1000.0 for r in recs]
    print(f"|acc| g: p5={pctl(accn,5):.3f} p50={median(accn):.3f} p95={pctl(accn,95):.3f} p99.9={pctl(accn,99.9):.3f}")
    # forward accel (mapped) in g
    af=[r['af']/1000.0 for r in recs]
    print(f"af(fwd) g: min={min(af):.3f} p50={median(af):.3f} p95={pctl(af,95):.3f} p99.9={pctl(af,99.9):.3f} max={max(af):.3f}")
    # gyro magnitude (dps) from mapped gf,gl,gu (/10)
    gmag=[math.sqrt((r['gf']/10.)**2+(r['gl']/10.)**2+(r['gu']/10.)**2) for r in recs]
    print(f"gyro |dps|: p50={median(gmag):.1f} p95={pctl(gmag,95):.1f} p99.9={pctl(gmag,99.9):.1f} max={max(gmag):.1f}")
    # firmware-recorded metrics
    inten=[r['inten']/10.0 for r in recs]
    strk=[r['strk']/10.0 for r in recs]
    surge=[r['surge']/10.0 for r in recs]
    print(f"intensity(fw) p50={median(inten):.1f} p95={pctl(inten,95):.1f} max={max(inten):.1f}")
    print(f"strokeStr(fw) p50={median(strk):.1f} p95={pctl(strk,95):.1f} max={max(strk):.1f}")
    print(f"surge(fw) g/s p50={median(surge):.2f} p95={pctl(surge,95):.2f} p99.9={pctl(surge,99.9):.2f} max={max(surge):.2f}")
    # idle detection like firmware: |accNorm-1|<0.12 and gyro<25
    idle=[i for i,r in enumerate(recs) if abs(r['accNorm']/1000.0-1.0)<0.12 and gmag[i]<25.0]
    print(f"idle-like samples: {len(idle)}/{len(recs)} = {100*len(idle)/len(recs):.1f}%")
    return h, recs, dict(af=af,gmag=gmag,surge=surge,inten=inten,strk=strk,accn=accn)

if __name__=='__main__':
    for p in sys.argv[1:]:
        analyze_raw(p)
