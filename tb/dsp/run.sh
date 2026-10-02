#!/bin/bash
# Falcon DSP (falcon_dsp) testbench: builds Hatari's DSP core as the golden
# model, verilates the RTL with the C++ testbench, runs every test, then the
# integration test with the real falcon_crossbar, and prints a PASS/FAIL
# summary.
# Usage: ./run.sh [--seeds N] [--insns M] [--seed0 S] [--only NAME] [--quiet]
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
RTL="$ROOT/rtl/falcon/dsp"
HATARI="${HATARI:-/home/adam/hatari}"
BUILD="$HERE/build"
mkdir -p "$BUILD"

# regenerate the decode/ROM tables from Hatari (keeps RTL and tb in sync)
python3 "$HERE/gen/gen_tables.py" "$HATARI/src/falcon" "$RTL" "$HERE/gen/opcodes8h.inc" > /dev/null

# golden model: Hatari src/falcon/dsp_cpu.c + dsp_core.c, unmodified
GINC="-I$HATARI/build -I$HATARI/src/includes -I$HATARI/src/debug -I$HATARI/src/falcon -I$HATARI/src -I$HATARI/src/cpu"
for f in dsp_cpu dsp_core; do
    gcc -O2 -w -c $GINC "$HATARI/src/falcon/$f.c" -o "$BUILD/golden_$f.o"
done
gcc -O2 -c "$HERE/golden/golden_stubs.c" -o "$BUILD/golden_stubs.o"
ar rcs "$BUILD/libgolden.a" "$BUILD/golden_dsp_cpu.o" "$BUILD/golden_dsp_core.o" "$BUILD/golden_stubs.o"

DSP_RTL="$RTL/falcon_dsp.sv $RTL/dsp56k_core.sv $RTL/dsp56k_dec.sv $RTL/dsp56k_alu.sv $RTL/dsp_periph.sv $RTL/dsp_ram.sv $RTL/dsp56k_rom.sv"

verilator --cc --exe --build -j 8 -O3 --x-assign fast --x-initial fast \
    -Wall -Wno-fatal -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-BLKSEQ \
    --Mdir "$BUILD/obj" -I"$RTL" \
    -CFLAGS "-O2 -I$HERE $GINC" \
    -LDFLAGS "$BUILD/libgolden.a" \
    --top-module falcon_dsp \
    $DSP_RTL \
    "$HERE/tb_dsp.cpp" "$HERE/tb_common.cpp" "$HERE/tb_gen.cpp" "$HERE/tb_lockstep.cpp" \
    "$HERE/tb_tests.cpp" "$HERE/tb_directed.cpp" "$HERE/tb_host.cpp" -o tb_dsp \
    > "$BUILD/verilator.log" 2>&1 || { grep -v "^%Warning\|^ " "$BUILD/verilator.log" | tail -40; exit 1; }

# integration with the real crossbar (rtl/falcon/falcon_crossbar.sv)
verilator --cc --exe --build -j 8 -O3 --x-assign fast --x-initial fast \
    -Wno-fatal -Wno-lint -Wno-style \
    --Mdir "$BUILD/obj_xbar" -I"$RTL" \
    -CFLAGS "-O2" \
    --top-module tb_xbar_top \
    "$HERE/tb_xbar_top.sv" "$ROOT/rtl/falcon/falcon_crossbar.sv" $DSP_RTL \
    "$HERE/tb_xbar.cpp" -o tb_xbar \
    > "$BUILD/verilator_xbar.log" 2>&1 || { grep -v "^%Warning\|^ " "$BUILD/verilator_xbar.log" | tail -40; exit 1; }

set +e
"$BUILD/obj/tb_dsp" "$@"
rc1=$?
rc2=0
case " $* " in
    *" --only "*) ;;
    *) "$BUILD/obj_xbar/tb_xbar"; rc2=$? ;;
esac
echo
if [ $rc1 -eq 0 ] && [ $rc2 -eq 0 ]; then
    echo "OVERALL: PASS (falcon_dsp testbench and crossbar integration)"
    exit 0
else
    echo "OVERALL: FAIL (tb_dsp rc=$rc1, tb_xbar rc=$rc2)"
    exit 1
fi
