#!/bin/bash
# Build and run the falcon_blitter Verilator testbench.
# The golden model is Hatari's src/blitter.c, copied unmodified into the
# build directory and compiled against the stub headers in golden/stubs.
# Usage: ./run.sh [number_of_random_blits [seed]]
set -e
cd "$(dirname "$0")"
TB=$(pwd)
RTL=${BLITTER_RTL_DIR:-$TB/../../rtl/falcon}
HATARI=${HATARI:-/home/adam/hatari}
OBJ=${BLITTER_OBJ:-$TB/obj}
NRAND=${1:-20000}
SEED=${2:-0x1234567887654321}

rm -rf "$OBJ"
mkdir -p "$OBJ/golden"
cp "$HATARI/src/blitter.c" "$OBJ/golden/blitter.c"
GCFLAGS="-O2 -Wall -I$TB/golden/stubs -I$HATARI/src/includes -I$TB/golden"
gcc $GCFLAGS -c "$OBJ/golden/blitter.c" -o "$OBJ/golden/blitter.o"
gcc $GCFLAGS -c "$TB/golden/golden_glue.c" -o "$OBJ/golden/golden_glue.o"
ar rcs "$OBJ/golden/libgolden.a" "$OBJ/golden/blitter.o" "$OBJ/golden/golden_glue.o"

verilator --cc --exe --build -j 4 -O2 -Wall -Wno-fatal \
    -Mdir "$OBJ/vl" --top-module falcon_blitter \
    -CFLAGS "-O2 -I$TB/golden" \
    -LDFLAGS "$OBJ/golden/libgolden.a -lm" \
    "$RTL/falcon_blitter.sv" "$TB/tb_blitter.cpp" > "$OBJ/verilator.log" 2>&1 \
    || { cat "$OBJ/verilator.log"; echo "SUMMARY: build FAILED"; exit 1; }
grep -E "%Warning" "$OBJ/verilator.log" || true

set +e
"$OBJ/vl/Vfalcon_blitter" "$NRAND" "$SEED"
rc=$?
if [ $rc -eq 0 ]; then echo "RESULT: PASS"; else echo "RESULT: FAIL"; fi
exit $rc
