#!/usr/bin/env python3
# Per-test clock counts from marker lines "MARK <value> <clock>" (core) or
# "<value> <clock>" (Hatari golden): the cost of test k is clock(2k) - clock(2k-1).
# table.py core.log [hatari.txt]
import re, sys
NAMES = ["nop","moveq","add.l Dn","move.l Dn","lea d(An)","dbra loop","bra.s","bcc not taken",
         "move.w (An)","move.l (An)","move.w ->(An)","move.l ->(An)","move.l (An)+,(An)+","move.w 1(An)",
         "mulu.w","muls.w","divu.w","divs.w","mulu.l","divu.l","lsl.l #8","asr.w Dn","bsr/rts","jsr/rts",
         "movem.l x2","MFP read","ROM move.l","add.l Dn,(An)","clr.l (An)","trap/rte","ext/swap",
         "nop, no I-cache","move.w (An), no I-cache","move.l (An), D-cache","copy, D-cache","(harness only)"]
def load(path):
    m = {}
    for line in open(path):
        x = re.match(r'(?:MARK\s+)?(\d+)\s+(\d+)\s*$', line.strip())
        if x: m.setdefault(int(x.group(1)), int(x.group(2)))
    return m
core = load(sys.argv[1])
gold = load(sys.argv[2]) if len(sys.argv) > 2 else {}
print("%-26s %8s %8s %6s %6s" % ("test", "core", "hatari", "diff", "-base"))
k0 = len(NAMES)
base = None
if all(x in core for x in (2*k0-1, 2*k0)) and all(x in gold for x in (2*k0-1, 2*k0)):
    base = (core[2*k0] - core[2*k0-1]) - (gold[2*k0] - gold[2*k0-1])
for k, name in enumerate(NAMES, 1):
    a, b = 2*k - 1, 2*k
    c = core[b] - core[a] if a in core and b in core else None
    g = gold[b] - gold[a] if a in gold and b in gold else None
    d = (c - g) if c is not None and g is not None else None
    r = (d - base) if d is not None and base is not None else None
    print("%-26s %8s %8s %6s %6s" % (name, c, g if g is not None else "-", d if d is not None else "-",
                                     r if r is not None else "-"))
