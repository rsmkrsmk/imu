#!/usr/bin/env python3
"""Rozpakowywanie plikow sesji spakowanych STRUMIENIOWO w locie (.pzs / .rzs,
magic RIMUZ1 / RRAWZ1, formatVersion 4).

Format:
  [64 B SessionHeader]  (reservedByte=1 => kompresja LZSS blokowa,
                         reservedByte2 = liczba rekordow na blok,
                         recordBytes = rozmiar rekordu przed kompresja)
  potem ciag blokow:
    [u16 rawLen LE][u16 compLen LE][u8 flags][compLen B danych]
    flags bit0: 1 = LZSS (jak packFileLzss), 0 = surowy blok (fallback)

Wynik: plik surowy identyczny z niekompresowanym .rwl/.pnt (naglowek RRAW02/RIMU04
+ ciag rekordow), gotowy dla parse_v4.py.

Uzycie:
  python3 unpack_stream.py ses00050.rzs [ses00050.rwl]
"""
import struct, sys, os

MAGIC_MAP = {b'RIMUZ1': b'RIMU04', b'RRAWZ1': b'RRAW02'}

def lzss_decode_block(comp: bytes, raw_len: int) -> bytes:
    """Dekoduje jeden blok LZSS (slownik lokalny, jak verifyLzssFile firmware)."""
    out = bytearray()
    i = 0
    n = len(comp)
    while len(out) < raw_len and i < n:
        g = comp[i]; i += 1
        for k in range(8):
            if len(out) >= raw_len:
                break
            if (g >> (7 - k)) & 1:
                if i + 1 >= n:
                    raise ValueError("uciety token dopasowania")
                b0 = comp[i]; b1 = comp[i + 1]; i += 2
                off = b0 | (((b1 >> 5) & 0x07) << 8)
                ln = (b1 & 0x1F) + 3
                for _ in range(ln):
                    out.append(out[len(out) - off])
            else:
                if i >= n:
                    raise ValueError("uciety literal")
                out.append(comp[i]); i += 1
    return bytes(out)

def unpack(path):
    b = open(path, 'rb').read()
    if len(b) < 64:
        raise ValueError("plik krotszy niz naglowek")
    magic = b[:8].split(b'\x00')[0]
    hdrBytes = struct.unpack_from('<H', b, 10)[0]
    compression = b[46]      # reservedByte @ offset: magic8+fmt2+hdr2+sid4+ms4+rate2+8osi + 3*4 bias = 8+2+2+4+4+2+8+12 = 42; recordBytes u32 @42; side @46? 
    # Policzmy offsety dokladnie wg struktury SessionHeader:
    # 0 magic[8]; 8 formatVersion u16; 10 headerBytes u16; 12 sessionId u32;
    # 16 startMillis u32; 20 targetRateHz u16; 22..29 osie (8xi8); 30 gyroBiasX f;
    # 34 gyroBiasY f; 38 gyroBiasZ f; 42 recordBytes u32; 46 side u8; 47 zoneVersion u8;
    # 48 reservedByte u8; 49 reservedByte2 u8; 50 refPeakG f; ...
    recordBytes = struct.unpack_from('<I', b, 42)[0]
    comp_flag = b[48]
    recs_per_block = b[49]
    if magic not in MAGIC_MAP:
        raise ValueError(f"magic {magic!r} nie jest strumieniowo spakowany (.pzs/.rzs)")
    if not (comp_flag & 1):
        raise ValueError("naglowek nie ma flagi kompresji (reservedByte bit0)")

    # Zbuduj naglowek wyjsciowy: ten sam 64B, ale magic->nieskompresowany, flagi kompresji wyzerowane.
    out_hdr = bytearray(b[:hdrBytes])
    out_magic = MAGIC_MAP[magic]
    out_hdr[:8] = out_magic + b'\x00' * (8 - len(out_magic))
    out_hdr[48] = 0  # reservedByte
    out_hdr[49] = 0  # reservedByte2

    out = bytearray(out_hdr)
    pos = hdrBytes
    nblk = 0; raw_total = 0; comp_total = 0; last_partial = False
    while pos + 5 <= len(b):
        rawLen, compLen = struct.unpack_from('<HH', b, pos)
        flags = b[pos + 4]
        pos += 5
        if pos + compLen > len(b):
            # niekompletny ostatni blok (np. awaria zasilania) — pomijamy
            last_partial = True
            break
        data = b[pos:pos + compLen]; pos += compLen
        if flags & 1:
            dec = lzss_decode_block(data, rawLen)
            if len(dec) != rawLen:
                raise ValueError(f"blok {nblk}: zdekodowano {len(dec)} != rawLen {rawLen}")
        else:
            dec = data
            if len(dec) != rawLen:
                raise ValueError(f"blok {nblk}: surowy {len(dec)} != rawLen {rawLen}")
        out.extend(dec)
        nblk += 1; raw_total += rawLen; comp_total += compLen + 5
    return bytes(out), dict(magic=magic.decode(), recordBytes=recordBytes,
                            recs_per_block=recs_per_block, blocks=nblk,
                            raw=raw_total, comp=comp_total, partial=last_partial)

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print(__doc__); sys.exit(1)
    src = sys.argv[1]
    out_data, info = unpack(src)
    dst = sys.argv[2] if len(sys.argv) > 2 else os.path.splitext(src)[0] + \
        ('.rwl' if info['magic'] == 'RRAWZ1' else '.pnt')
    open(dst, 'wb').write(out_data)
    ratio = 100 * info['comp'] / info['raw'] if info['raw'] else 0
    print(f"OK {src} -> {dst}")
    print(f"  magic={info['magic']} recordBytes={info['recordBytes']} rekordow/blok={info['recs_per_block']}")
    print(f"  blokow={info['blocks']}  surowe={info['raw']}B  spakowane={info['comp']}B ({ratio:.1f}%)")
    if info['partial']:
        print("  UWAGA: ostatni blok byl niekompletny (pominiety) — plik zapisany do miejsca awarii.")
