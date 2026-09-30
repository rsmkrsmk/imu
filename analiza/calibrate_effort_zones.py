#!/usr/bin/env python3
"""Kalibracja WSZYSTKICH docelowych stalych na 56/57/58. Symuluje pelny potok
firmware z NOWYMI wzorami: sila odepchniec (held, wariant 2), strefy z held,
wysilek chwilowy (wygladzony), TSS narastajacy. Wypisuje docelowe rozklady."""
import struct, math
PI=math.pi

# ---- KANDYDACI STALYCH (do zatwierdzenia) ----
# Sila odepchniecia (wariant 2: mocna 80% -> strefa 3, sprint -> 4)
RP, RI, RS = 2.0, 0.15, 55.0            # ref peak / impulse / surge
WP, WI, WSU = 0.40, 0.30, 0.30          # wagi (suma 1.0 -> skala do 100)
# progi detektora (obnizone do jazdy)
ES, EH, CS, CH = 12.0, 0.45, 20.0, 0.70
REFR=450000; WIN=120000
# wysilek: intensywnosc chwilowa (ref podniesione by nie zapychac) * skala
IA_REF, ISU_REF = 1.5, 70.0
IA_W, ISU_W = 0.55, 0.45
EFFORT_SCALE = 3.0                       # rozciagniecie 0-100 (bo surowe ~0-40)
EFFORT_LP_HZ = 0.3                       # wolne wygladzanie "jak ciezko dysze"
# TSS: (wysilek/100)^2 * minuty * 100
TSS_SCALE = 100.0

def load(path):
    b=open(path,'rb').read(); hb=struct.unpack_from('<H',b,10)[0]; body=b[hb:]
    rs=48; FMT="<I hhh hhh hhh hhh H h H H H h h BBB B H"; n=len(body)//rs
    R=[struct.unpack_from(FMT,body,i*rs) for i in range(n)]
    af=[r[7]/1000.0 for r in R]; su=[r[18]/10.0 for r in R]; t=[r[0] for r in R]
    hp=[];y=0;px=None
    for i,x in enumerate(af):
        dt=0.005 if i==0 else min(max((t[i]-t[i-1])/1e6,0.002),0.05)
        if px is None:px=x;hp.append(0.0);continue
        rc=1/(2*PI);a=rc/(rc+dt);y=a*(y+x-px);px=x;hp.append(y)
    return af,su,hp,t

def zones(vals,z=(15,30,50,70)):
    c=[0]*5
    for v in vals:
        c[0 if v<z[0] else 1 if v<z[1] else 2 if v<z[2] else 3 if v<z[3] else 4]+=1
    n=len(vals); return [round(100*x/n) for x in c]

def run(path,lab):
    af,su,hp,t=load(path); n=len(t); span=(t[-1]-t[0])/1e6
    # --- detektor + sila held (skala do 100 dzieki wagom sum=1) ---
    inC=False;cS=cP=0;cH=cU=cImp=0;cf=False;last=0;held=0.0
    held_series=[]; stroke_vals=[]
    for i in range(n):
        h=hp[i];s=su[i];ti=t[i]
        dt=0.005 if i==0 else min(max((t[i]-t[i-1])/1e6,0.002),0.05)
        if not inC:
            if s>=ES or h>=EH: inC=True;cS=cP=ti;cH=h;cU=s;cImp=abs(h)*dt;cf=(s>=CS or h>=CH)
        else:
            if h>cH:cH=h;cP=ti
            if s>cU:cU=s
            cImp+=abs(h)*dt
            if not cf and (s>=CS or h>=CH):cf=True
            if ti-cS>=WIN:
                if cf and (last==0 or cP-last>=REFR):
                    nP=min(cH/RP,2.0);nI=min(cImp/RI,2.0);nSu=min(cU/RS,2.0)
                    held=max(0,min(100,100*(WP*nP+WI*nI+WSU*nSu)))
                    stroke_vals.append(held); last=cP
                inC=False
        held_series.append(held)
    # --- wysilek chwilowy (wygladzony) + TSS ---
    eff=[]; y=0; tss=0
    for i in range(n):
        dt=0.005 if i==0 else min(max((t[i]-t[i-1])/1e6,0.002),0.05)
        raw=min(100, EFFORT_SCALE*100*min(1.0, IA_W*max(0,hp[i])/IA_REF + ISU_W*max(0,su[i])/ISU_REF))
        rc=1/(2*PI*EFFORT_LP_HZ); a=dt/(rc+dt); y+=a*(raw-y); eff.append(y)
        tss += (y/100.0)**2 * dt
    tss_val = tss/60.0*TSS_SCALE
    es=sorted(eff); hv=sorted(held_series)
    print(f"\n=== {path} ({lab}) {span:.0f}s ===")
    print(f"  SILA odepchniec: n={len(stroke_vals)} sr={sum(stroke_vals)/len(stroke_vals):.0f} min={min(stroke_vals):.0f} max={max(stroke_vals):.0f}" if stroke_vals else "  brak odepchniec")
    print(f"  STREFY (z held): {zones(held_series)}%  [Z0 Z1 Z2 Z3 Z4]")
    print(f"  WYSILEK chwilowy: p50={es[int(n*.5)]:.0f} p90={es[int(n*.9)]:.0f} max={max(eff):.0f}")
    print(f"  TSS za sesje={tss_val:.0f}  (na godzine ~{tss_val*3600/span:.0f})")

for f,l in [('ses00056.rwl','spokojne'),('ses00057.rwl','MOCNE 80%'),('ses00058.rwl','schody')]:
    run(f,l)
print("\n>>> Cel: spokojne strefy 2-3, MOCNE strefy 3-4; wysilek MOCNE>spokojne; TSS/h rozny")
