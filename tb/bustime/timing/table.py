#!/usr/bin/env python3
# Per-test clock counts from marker lines "MARKE <value> <clock>" (core: the
# instruction boundary after the marker, as Hatari's counter) or
# "<value> <clock>" (Hatari golden): the cost of test k is clock(2k) - clock(2k-1).
# "span" is clock(2k+1) - clock(2k-1): the test and the next test's
# cache-filling run.  Where a marker falls relative to the bus cycles around
# it (a fetch just before or just after it) moves clocks between the two
# parts, not their sum, so the span difference is the one that measures the
# core; the totals at the end compare whole sections.
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
        x = re.match(r'(?:MARKE\s+)?(\d+)\s+(-?\d+)\s*$', line.strip())
        if x: m.setdefault(int(x.group(1)), int(x.group(2)))
    return m
def span(m, a, b):
    return m[b] - m[a] if a in m and b in m else None
def sub(x, y):
    return x - y if x is not None and y is not None else None
def s(x):
    return "-" if x is None else str(x)
core = load(sys.argv[1])
gold = load(sys.argv[2]) if len(sys.argv) > 2 else {}
print("%-26s %6s %6s %5s  %6s %6s %5s" % ("test", "core", "hatari", "diff", "span", "hatari", "diff"))
for k, name in enumerate(NAMES, 1):
    a, b, n = 2*k - 1, 2*k, 2*k + 1
    c, g = span(core, a, b), span(gold, a, b)
    cs, gs = span(core, a, n), span(gold, a, n)
    print("%-26s %6s %6s %5s  %6s %6s %5s" % (name, s(c), s(g), s(sub(c, g)), s(cs), s(gs), s(sub(cs, gs))))
if gold:
    last = 2 * len(NAMES)
    for a, b, what in ((1, last - 10, "I-cache on, D-cache off"), (last - 9, last, "the rest"), (1, last, "all")):
        c, g = span(core, a, b), span(gold, a, b)
        if c is not None and g:
            print("markers %d..%d (%s): core %d, Hatari %d, %+d (%+.2f%%)" % (a, b, what, c, g, c - g, 100.0 * (c - g) / g))
