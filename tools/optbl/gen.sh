#!/bin/bash
# Generate rtl/falcon/falcon_optbl.mem (falcon_pipescan's opcode table) from
# Hatari's own 68030 data: op_smalltbl_23 in the generated cpustbl.c and the
# table68k handler mapping (readcpu.c, generated cpudefs.c), as newcpu.c
# build_cpufunctbl builds cpudatatbl.
#   HATARI_SRC=<hatari>/src HATARI_BUILD=<cmake build dir> gen.sh
# One 9-bit entry per opcode, in binary ($readmemb: 9 digits, so Quartus
# reads it without a width warning per entry):
#   [2:0] length in words (0: none, 7: variable, the F-line opcodes)
#   [3]   prefetch stops (branch > 0: RTS, RTE, RTD, RTR, JSR, JMP, BSR)
#   [6:4] disp020[0] / 2 (0: none; Hatari's -3 never matches: 0)
#   [8:7] disp020[1] / 2
set -e
cd "$(dirname "$0")"
: "${HATARI_SRC:?}" "${HATARI_BUILD:?}"
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT
python3 -I - "$HATARI_BUILD/src/cpu/cpustbl.c" > "$T/smalltbl23.h" <<'PY'
import re, sys
s = open(sys.argv[1]).read()
i = s.index("const struct cputbl op_smalltbl_23[] = {")
j = s.index("};", i)
print("static const struct { unsigned opcode, length, d0, d1; int branch; } st23[] = {")
for m in re.finditer(r"\{\s*NULL,\s*op_\w+,\s*0x([0-9a-f]+),\s*(-?\d+),\s*\{\s*(-?\d+),\s*(-?\d+)\s*\},\s*(-?\d+)\s*\}", s[i:j]):
    print("{0x%s,%s,%s,%s,%s}," % m.groups())
print("};")
PY
gcc -O1 -w -I"$T" -I"$HATARI_SRC/cpu" -I"$HATARI_SRC/includes" -I"$HATARI_BUILD" -I"$HATARI_BUILD/src/cpu" \
	-o "$T/dump" dump.c "$HATARI_SRC/cpu/readcpu.c" "$HATARI_BUILD/src/cpu/cpudefs.c"
"$T/dump" | python3 -I -c '
import sys
for line in sys.stdin:
    ln, d0, d1, br = map(int, line.split())
    lw = 7 if ln < 0 else ln // 2
    d0 = d0 // 2 if d0 > 0 else 0
    d1 = d1 // 2 if d1 > 0 else 0
    print(format(lw | ((1 if br > 0 else 0) << 3) | (d0 << 4) | (d1 << 7), "09b"))
' > ../../rtl/falcon/falcon_optbl.mem
wc -l ../../rtl/falcon/falcon_optbl.mem
