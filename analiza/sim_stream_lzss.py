#!/usr/bin/env python3
"""Symulacja pakowania BLOKOWEGO w locie: LZSS z resetem slownika co blok,
dokladnie taki algorytm jak firmware (okno 2047, min match 3, max len 34,
grupa 8 decyzji). Cel: dobrac rozmiar bloku (kompresja vs pamiec vs odpornosc).

LZSS firmware:
  token dopasowania: off = b0 | ((b1>>5)&7)<<8  (11 bitow, 1..2047)
                     len = (b1 & 0x1F) + 3       (3..34)
  literal: 1 bajt; grupa: 1 bajt znacznikow (MSB pierwszy) + do 8 tokenow.
"""
import sys, struct

WINDOW = 2047
MINM = 3
MAXM = 34

def lzss_block(data: bytes) -> bytes:
    """Koduje jeden blok od zera (pusty slownik na starcie)."""
    out = bytearray()
    n = len(data)
    pos = 0
    group = 0; gcount = 0; pend = bytearray()
    def flush():
        nonlocal group, gcount, pend
        if gcount == 0: return
        out.append(group); out.extend(pend)
        group = 0; gcount = 0; pend = bytearray()
    while pos < n:
        # najdluzsze dopasowanie w oknie [max(0,pos-WINDOW), pos)
        best_len = 0; best_off = 0
        start = max(0, pos - WINDOW)
        # brute force jak firmware (po dystansie off=1..maxDist)
        maxdist = pos - start
        limit = min(MAXM, n - pos)
        if limit >= MINM:
            for off in range(1, maxdist + 1):
                if data[pos - off] != data[pos]:
                    continue
                l = 1
                while l < limit and (off - l) >= 0 and pos - off + l < pos and data[pos - off + l] == data[pos + l]:
                    # dopasowanie moze siegac poza (overlap) - firmware pozwala gdy off-len>=1
                    if (off - l) < 1:
                        break
                    l += 1
                if l > best_len:
                    best_len = l; best_off = off
                    if l == limit: break
        if best_len >= MINM:
            group |= (1 << (7 - gcount))
            b1 = (((best_off >> 8) & 0x07) << 5) | (best_len - 3)
            pend.append(best_off & 0xFF); pend.append(b1)
            pos += best_len
        else:
            pend.append(data[pos]); pos += 1
        gcount += 1
        if gcount == 8: flush()
    flush()
    return bytes(out)

def sim(path, block_bytes):
    b = open(path, 'rb').read()
    hb = struct.unpack_from('<H', b, 10)[0]
    body = b[hb:]
    comp_total = 0
    nblocks = 0
    for i in range(0, len(body), block_bytes):
        chunk = body[i:i+block_bytes]
        c = lzss_block(chunk)
        # naglowek bloku: 2B comp_len + 2B raw_len (jak w projekcie)
        comp_total += len(c) + 4
        nblocks += 1
    return len(body), comp_total, nblocks

if __name__ == '__main__':
    files = sys.argv[1:] or ['newdata/ses00043.rwl', 'newdata/ses00045.rwl']
    # rozmiary bloku w rekordach 48B: 64rec=3072B (jak WRITER_BLOCK), 128, 256
    for rec in (64, 128, 256):
        blk = rec * 48
        print(f"\n### blok = {rec} rekordow ({blk} B surowych) ###")
        for f in files:
            raw, comp, nb = sim(f, blk)
            print(f"{f.split('/')[-1]:16} raw={raw:7} comp={comp:7} ({100*comp/raw:5.1f}%)  blokow={nb}")
