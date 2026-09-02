#!/usr/bin/env python3
# Probe RRAW01 (v1) 40-byte record layout empirically.
# Header 46 bytes. Body = N*40.
# v4 RawLabRecord (48B) layout as reference; v1 likely dropped some fields to 40B.
# Known: firmware writes rawAx/Ay/Az (mg,int16), rawGx/Gy/Gz (dps*10,int16),
# af/al/au (mg), gf/gl/gu (dps*10), accNorm(mg,u16), lean(cdeg,i16),
# intensity_x10(u16), strokeStrength_x10(u16), cadenceMs(u16), surge_dps10(i16),
# intensitySmooth?(i16), strokePhaseMs(u8), flags(u8), side(u8) ...
# Let's just dump a few records as int16 arrays to see structure.
import struct, sys

f = sys.argv[1]
b = open(f,'rb').read()
hdr = 46
body = b[hdr:]
REC = 40
n = len(body)//REC
print(f"{f}: recs={n}")
# tUs is u32 at start (elapsed micros). Print first few records:
for i in range(6):
    r = body[i*REC:(i+1)*REC]
    tus = struct.unpack_from('<I', r, 0)[0]
    # rest as int16
    i16 = struct.unpack_from('<' + 'h'*((REC-4-2)//2), r, 4)  # leave last 2 bytes for flags/side/pad
    tail = r[REC-4:]
    print(f"[{i}] t={tus:>9} dt={ (tus - (struct.unpack_from('<I', body[(i-1)*REC:(i-1)*REC+4],0)[0]) if i>0 else 0) } i16={i16} tail={tail.hex(' ')}")

# time base sanity
t0 = struct.unpack_from('<I', body, 0)[0]
tN = struct.unpack_from('<I', body, (n-1)*REC)[0]
print(f"t0={t0} tN={tN} span={ (tN-t0)/1e6:.2f}s  approx rate={ (n-1)/((tN-t0)/1e6):.1f} Hz")
