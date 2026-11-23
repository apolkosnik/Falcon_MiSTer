#!/usr/bin/env python3
# REVERT.PRG's depacker (ICE 2.x, text+$B0..$15E), transcribed instruction by
# instruction, run on the program file as GEMDOS loads it at text $23DAC.
#   ice_ref.py PRG OUT.bin [RAMHEX]
# OUT.bin: guest memory $000000-$A00000 after the depack (the reference).
# RAMHEX: the same memory BEFORE the depack, in the DDR3 model's +ram= format
# (rom2hex64.py's), with every vector pointing at the test ROM's fail stub.
import struct, sys

TEXT = 0x23DAC
SIZE = 0xA00000
f = open(sys.argv[1], 'rb').read()
tlen, dlen = struct.unpack('>II', f[2:10])
mem = bytearray(SIZE)
mem[TEXT:TEXT + tlen + dlen] = f[0x1C:0x1C + tlen + dlen]

if len(sys.argv) > 3:
    init = bytearray(mem)
    for v in range(2, 256):                     # vectors -> $E00100 (fail stub)
        init[4 * v:4 * v + 4] = struct.pack('>I', 0xE00100)
    with open(sys.argv[3], 'w') as h:
        for i in range(0, 0x940000, 8):
            h.write(init[i:i + 8][::-1].hex() + '\n')

def rl(a): return struct.unpack('>I', mem[a:a + 4])[0]
def rw(a): return struct.unpack('>H', mem[a:a + 2])[0]
def sb(v): return v - 256 if v & 0x80 else v
def sw(v): v &= 0xFFFF; return v - 65536 if v & 0x8000 else v

a4 = TEXT + 0x216                    # lea $23fc2(pc),a4
a5 = (a4 + 0x559695) & 0xFFFFFFFF
a6 = (a4 + 0x910430) & 0xFFFFFFFF
d7 = 0; X = 0

def getbit():                        # $23e8a: add.b d7,d7; bne; move.b -(a5),d7; addx.b d7,d7
    global d7, a5, X
    c = (d7 >> 7) & 1
    d7 = (d7 << 1) & 0xFF
    X = c
    if d7 != 0:
        return c
    a5 -= 1
    d7 = mem[a5]
    c = (d7 >> 7) & 1
    d7 = ((d7 << 1) | X) & 0xFF
    X = c
    return c

def getbits(d0):                     # $23e94: d1 = 0; d0.w+1 bits, MSB first
    d1 = 0
    n = (d0 & 0xFFFF)
    while True:
        b = getbit()
        d1 = ((d1 << 1) | X) & 0xFFFF
        n = (n - 1) & 0xFFFF
        if n == 0xFFFF:
            return d1

a5 -= 1; d7 = mem[a5]                # $23e5c
steps = 0
while True:
    steps += 1
    if getbit():                     # $23e5e: literal run
        d1 = 0
        if getbit():
            a1 = 0x23F22; d3 = 4
            while True:
                a1 -= 4
                d0 = rl(a1)
                d1 = getbits(d0)
                hi = (d0 >> 16) & 0xFFFF
                if d1 != hi:
                    break
                d3 -= 1
                if d3 == -1:
                    break
            d1 = (d1 + rl(a1 + 0x14)) & 0xFFFFFFFF
        n = d1 & 0xFFFF              # $23e7e: move.b -(a5),-(a6); dbra d1
        while True:
            a5 -= 1; a6 -= 1; mem[a6] = mem[a5]
            n = (n - 1) & 0xFFFF
            if n == 0xFFFF:
                break
    # $23e84: cmpa.l a4,a6; bgt match; else rts
    if not (a6 - a4 > 0 and a6 > a4):
        break
    a1 = 0x23F36; d2 = 3             # $23ea6
    while True:
        if not getbit():
            break
        d2 -= 1
        if d2 == -1:
            break
    d2 &= 0xFFFF
    d4 = 0; d1 = 0
    d0 = sb(mem[a1 + 1 + sw(d2)])
    if d0 >= 0:
        d1 = getbits(d0)
    d4 = mem[a1 + 6 + sw(d2)]
    d4 = (d4 + d1) & 0xFFFF
    if d4 == 0:                      # $23eea
        d0 = 5; d2 = 0xFFFF
        if getbit():
            d0 = 8; d2 = 0x3F
        d1 = getbits(d0)
        d1 = (d1 + d2) & 0xFFFF
    else:
        a1 = 0x23F40; d2 = 1
        while True:
            if not getbit():
                break
            d2 -= 1
            if d2 == -1:
                break
        d2 &= 0xFFFF
        d0 = sb(mem[a1 + 1 + sw(d2)])
        d1 = getbits(d0)
        d2 = (d2 + d2) & 0xFFFF
        d1 = (d1 + rw(a1 + 6 + sw(d2))) & 0xFFFF
        if d1 & 0x8000:
            d1 = (d1 - d4) & 0xFFFF
    a1 = a6 + sw(d4) + 2 + sw(d1)    # $23efc
    a1 -= 1; a6 -= 1; mem[a6] = mem[a1]
    n = d4
    while True:
        a1 -= 1; a6 -= 1; mem[a6] = mem[a1]
        n = (n - 1) & 0xFFFF
        if n == 0xFFFF:
            break
    if a5 < 0x800 or a6 < 0x800:
        print('reference ran off the data: a5 %x a6 %x after %d tokens' % (a5, a6, steps)); break

print('reference depack: %d tokens, a5 ends %x (packed start %x), a6 ends %x (a4 %x)' % (steps, a5, a4, a6, a4))
open(sys.argv[2], 'wb').write(mem)
