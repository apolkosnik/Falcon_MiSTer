#!/usr/bin/env python3
"""tt030_pace_gen.py > rtl/falcon/tt030_pace.mem
Idle clocks per operation word for the TT030 timing mode: the 32 MHz clocks a real TT's 68030 takes
beyond the AP68030, from per-instruction timings of loops under Atari System V on a real TT and on the core
(cache hits, MMU on).  One hex byte per opcode, 65536 lines.  Where the core is the
slower one (memory writes, uncached memory, the FPU) the entry is 0: idle clocks cannot help there.
Measured on the board, an entry of n adds about 1.1 n clocks; the values allow for that.
The multiply and divide loops time a MOVE and the operation together, so those entries are
twice the row's difference."""
import sys

def ea_extra(mode, reg, long_):
    """extra for a source operand read through this EA (0 for registers: the caller adds the base)"""
    if mode in (2, 3): return 0
    if mode == 4: return 1
    if mode == 5: return 1
    if mode == 6: return 3
    if mode == 7:
        if reg in (0, 1): return 2
        if reg == 2: return 8
        if reg == 3: return 10
        if reg == 4: return 4 if long_ else 2
    return 0

def is_reg(mode): return mode in (0, 1)
def is_imm(mode, reg): return mode == 7 and reg == 4

def alu_src(mode, reg, long_):
    """<ea>,Dn forms: register source +1, immediate +5/+2, memory by its EA"""
    if is_reg(mode): return 1
    return ea_extra(mode, reg, long_)

def extra(op):
    line = op >> 12; mode = (op >> 3) & 7; reg = op & 7; size = (op >> 6) & 3
    dmode = (op >> 6) & 7; b8 = (op >> 8) & 1
    if line == 0:
        if b8 == 0 and (op >> 9) & 7 in (0, 1, 2, 3, 5, 6) and size != 3:      # ORI ANDI SUBI ADDI EORI CMPI
            return (4 if size == 2 else 2) if mode == 0 else 0
        if (op & 0x0F00) == 0x0800:                                           # static bit operations
            if mode == 0: return 2 if size == 0 else 4
            return 1
        if b8 == 1 and mode != 1:                                             # dynamic bit operations
            if mode == 0: return 3 if size == 0 else 5
            return 1
        return 0
    if line in (1, 2, 3):                                                     # MOVE, MOVEA
        long_ = line == 2
        if is_reg(dmode):
            return alu_src(mode, reg, long_)
        e = ea_extra(mode, reg, long_) if not is_imm(mode, reg) else 0
        if dmode == 4: e += 3                                                 # push
        return e
    if line == 4:
        if (op & 0xFFF8) == 0x49C0: return 3                                  # EXTB
        if (op & 0x01C0) == 0x01C0:                                           # LEA
            return {2: 1, 5: 2, 6: 4}.get(mode, 2 if mode == 7 and reg in (0, 1, 2) else 4 if mode == 7 else 0)
        if (op & 0x0F00) in (0x0000, 0x0200, 0x0400, 0x0600) and size != 3:   # NEGX CLR NEG NOT
            return 1 if mode == 0 else 0
        if (op & 0xFFF8) == 0x4840: return 3                                  # SWAP
        if (op & 0xFFF8) in (0x4880, 0x48C0, 0x49C0): return 3                # EXT, EXTB
        if (op & 0xFB80) == 0x4880 and mode >= 2: return 4                    # MOVEM
        if (op & 0xFF00) == 0x4A00:
            if size == 3: return 3 if mode == 0 else 0                        # TAS
            return 1 if is_reg(mode) else ea_extra(mode, reg, size == 2)      # TST
        if (op & 0xFFC0) == 0x4C00: return 41                                 # MULU.L MULS.L
        if (op & 0xFFC0) == 0x4C40: return 78                                 # DIVU.L DIVS.L (82 and 97 wanted: one entry for both)
        if (op & 0xFFF8) == 0x4E50: return 2                                  # LINK
        if op == 0x4E74: return 2                                             # RTD
        if (op & 0xFFC0) == 0x4E80:                                           # JSR
            return 3 if mode == 7 and reg in (0, 1) else 2 if mode in (6, 7) else 0
        if (op & 0xFFC0) == 0x4EC0: return 1 if mode == 7 and reg in (0, 1) else 0   # JMP
        return 0
    if line == 5:
        if size == 3:
            if mode == 1: return 2                                            # DBcc
            return 2 if mode == 0 else 0                                      # Scc
        return 1 if is_reg(mode) else 0                                       # ADDQ SUBQ
    if line == 6:
        cond = (op >> 8) & 15; d8 = op & 0xFF
        if cond == 0: return 2                                                # BRA
        if cond == 1: return 2                                                # BSR
        return 4                                                              # Bcc
    if line == 7: return 1 if b8 == 0 else 0                                  # MOVEQ
    if line in (8, 9, 0xB, 0xC, 0xD):
        opm = (op >> 6) & 7
        if line == 8 and opm == 3: return 34 + ea_extra(mode, reg, False)     # DIVU.W
        if line == 8 and opm == 7: return 45 + ea_extra(mode, reg, False)     # DIVS.W
        if line == 0xC and opm in (3, 7): return 22 + ea_extra(mode, reg, False)   # MULU.W MULS.W
        if line == 0xC and (op & 0xF1F8) in (0xC140, 0xC148, 0xC188): return 1     # EXG
        if line in (9, 0xB, 0xD) and opm in (3, 7):                           # SUBA CMPA ADDA
            if is_reg(mode): return 1 if opm == 7 else 3
            return ea_extra(mode, reg, opm == 7) + (2 if opm == 3 else 0)
        if opm < 3: return alu_src(mode, reg, opm == 2)                       # <ea>,Dn
        if mode == 0: return 1 if line == 0xB else 1                          # EOR Dn,Dn; ADDX/SUBX/ABCD/SBCD
        return 0                                                              # Dn,<ea> in memory; CMPM
    if line == 0xE:
        if size == 3:
            if (op >> 11) & 1:                                                # bit fields
                return 6 if (op >> 8) & 7 in (0, 1, 3, 5) else 8
            return 0                                                          # memory shifts
        typ = (op >> 3) & 3; left = b8; byreg = (op >> 5) & 1
        if typ == 2: return 11                                                # ROXL ROXR
        if byreg or typ == 3: return 5                                        # by a register; ROL ROR
        if typ == 0 and left: return 5                                        # ASL
        return 3                                                              # LSL LSR ASR by a constant
    return 0

for op in range(65536):
    print('%02x' % min(extra(op), 127))
