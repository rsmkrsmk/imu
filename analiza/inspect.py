#!/usr/bin/env python3
import struct, sys, os

# SessionHeader (64 B) per firmware:
# char magic[8]; u16 formatVersion; u16 headerBytes; u32 sessionId; u32 startMillis;
# u16 targetRateHz; i8 fAxis,fSign,lAxis,lSign,vAxis,vSign,rAxis,rSign;
# f32 gBiasX,gBiasY,gBiasZ; u32 reserved;
# u8 side, zoneVersion, reservedByte, reservedByte2; f32 refPeakG, refImpulseGs, refSurgeGps; u8 pad0,pad1
HDR = struct.Struct('<8s H H I I H bbbbbbbb fff I BBBB fff BB')

def parse_header(b):
    (magic, fv, hb, sid, sms, rate,
     fa,fsn,la,lsn,va,vsn,ra,rsn,
     gbx,gby,gbz, reserved,
     side, zver, rb, rb2, refPeak, refImp, refSurge, p0, p1) = HDR.unpack_from(b, 0)
    magic = magic.split(b'\x00')[0].decode('latin1')
    return dict(magic=magic, fv=fv, hb=hb, sid=sid, startMillis=sms, rate=rate,
                axes=(fa,fsn,la,lsn,va,vsn,ra,rsn),
                bias=(gbx,gby,gbz), reserved=reserved,
                side=side, zoneVersion=zver, refPeak=refPeak, refImp=refImp, refSurge=refSurge)

for f in sys.argv[1:]:
    b = open(f,'rb').read()
    print("===", os.path.basename(f), "size", len(b), "===")
    print("first 16 bytes:", b[:16].hex(' '))
    print("magic ascii   :", b[:8].split(b'\x00')[0].decode('latin1', 'replace'))
    try:
        h = parse_header(b)
        for k,v in h.items():
            print(f"  {k}: {v}")
        body = len(b) - h['hb']
        for rec in (23, 32, 40, 48):
            print(f"  body={body}  /{rec} = {body/rec:.3f}  (rem {body % rec})")
    except Exception as e:
        print("  header parse error:", e)
    print()
