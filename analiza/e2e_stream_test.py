#!/usr/bin/env python3
"""Test end-to-end pakowania w locie: symuluje zapis firmware (naglowek RRAWZ1
+ bloki po 64 rekordy LZSS), pakuje realny .rwl, rozpakowuje parserem i porownuje
bajt-w-bajt z oryginalem."""
import struct, sys, os
sys.path.insert(0, os.path.dirname(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'imu', 'analiza'))
from sim_stream_lzss import lzss_block  # zweryfikowany enkoder blokowy
from unpack_stream import unpack

REC_RAW = 48
BLOCK_RECORDS = 64

def firmware_pack(src_path, dst_path):
    """Odtwarza dokladnie to, co robi firmware przy activeCompression=1 dla RAW LAB."""
    b = open(src_path, 'rb').read()
    hdrBytes = struct.unpack_from('<H', b, 10)[0]
    hdr = bytearray(b[:hdrBytes])
    # magic RRAW02 -> RRAWZ1
    hdr[:8] = b'RRAWZ1' + b'\x00\x00'
    # reservedByte(48)=1 flaga kompresji, reservedByte2(49)=64 rekordow/blok
    hdr[48] = 1
    hdr[49] = BLOCK_RECORDS
    body = b[hdrBytes:]
    out = bytearray(hdr)
    block_bytes = BLOCK_RECORDS * REC_RAW
    for i in range(0, len(body), block_bytes):
        chunk = body[i:i+block_bytes]      # ostatni moze byc krotszy
        comp = lzss_block(chunk)
        rawLen = len(chunk)
        if len(comp) == 0 or len(comp) >= rawLen:
            flags = 0; data = chunk         # fallback surowy (jak firmware)
        else:
            flags = 1; data = comp
        compLen = len(data)
        out += struct.pack('<HH', rawLen, compLen) + bytes([flags]) + data
    open(dst_path, 'wb').write(out)
    return len(body), len(out) - hdrBytes

def main():
    src = sys.argv[1] if len(sys.argv) > 1 else '../ses00045.rwl'
    packed = '_stream_test.rzs'
    unpacked = '_stream_test.rwl'
    raw_body, comp_body = firmware_pack(src, packed)
    print(f"spakowano: cialo {raw_body}B -> {comp_body}B ({100*comp_body/raw_body:.1f}%)")
    # rozpakuj parserem
    out_data, info = unpack(packed)
    open(unpacked, 'wb').write(out_data)
    # porownaj z oryginalem
    orig = open(src, 'rb').read()
    rec = out_data
    # naglowki roznia sie magic/flagami — porownaj CIALA (po hdrBytes)
    hb = struct.unpack_from('<H', orig, 10)[0]
    same_body = orig[hb:] == rec[hb:]
    # naglowek: po rozpakowaniu magic ma wrocic do RRAW02, flagi do 0
    same_magic = rec[:6] == b'RRAW02'
    same_flags = rec[48] == 0 and rec[49] == 0
    print(f"blokow={info['blocks']} recordBytes={info['recordBytes']} recs/blk={info['recs_per_block']}")
    print(f"cialo identyczne z oryginalem: {'TAK' if same_body else 'NIE'}")
    print(f"magic po rozpakowaniu = RRAW02: {'TAK' if same_magic else 'NIE'}")
    print(f"flagi kompresji wyzerowane: {'TAK' if same_flags else 'NIE'}")
    ok = same_body and same_magic and same_flags
    print("\nEND-TO-END: " + ("OK" if ok else "FAIL"))
    return 0 if ok else 1

if __name__ == '__main__':
    sys.exit(main())
