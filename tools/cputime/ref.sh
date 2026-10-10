#!/bin/bash
# Regenerate ref.i: Falcon time at each of CPUTIME's markers in the
# full-system simulation (sim.sh: tb/system, the DDR3 model, the counters
# on), from CPUTIME.TOS built with REFZERO (the same layout as the final
# program, without the numbers; it then prints them).  The board's must be
# equal to them.
#   VASM=/path/to/vasmm68k_mot ref.sh
# Needs tb/system/build.sh to have been run.
set -e
cd "$(dirname "$0")"
VASM=${VASM:-vasmm68k_mot}
W=$(mktemp -d)
trap 'rm -rf "$W"' EXIT
$VASM -Ftos -m68030 -no-opt -quiet -DREFZERO -I../../tb/bustime/timing -o $W/CPUTIME.TOS cputime.s
OUT=$W/sim PRG=$W/CPUTIME.TOS ./sim.sh > $W/sim.txt
python3 -I - $W/sim.txt > ref.i <<'PY'
import re, sys
mark = {}
for line in open(sys.argv[1], errors="replace"):
    x = re.match(r'^M (\d+) (-?\d+)\s*$', line.strip())
    if x: mark.setdefault(int(x.group(1)), int(x.group(2)))
if sorted(mark) != list(range(1, 74)): sys.exit("ref.sh: markers missing in the simulation: %s" % sorted(mark))
print("; ref.i - CPUTIME's reference: Falcon time at markers 1-73 (from marker 1)")
print("; in the full-system simulation (written by ref.sh, do not edit)")
print("ref_valid:\tdc.w\t2\t\t; bit 1: the simulation")
for i in range(1, 74, 8):
    print("%s\tdc.l\t%s" % ("ref_mark:" if i == 1 else "", ",".join(str(mark[k]) for k in range(i, min(i + 8, 74)))))
PY
cat ref.i
./build.sh
