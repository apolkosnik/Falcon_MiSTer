#!/bin/bash
# Falcon bus timing bench (tb_bustime.sv, docs/CPU_TIMING.md milestone 2):
# the AP68030, falcon_cpuclk, falcon_cpubus (Falcon mode) and falcon_memarb
# on the DDR3 model run t_bustime.s; every bus cycle must take Hatari's
# length in processor clocks.  The program runs in several configurations
# (DDR3 and device latencies, video load, 8 MHz) and must take the same
# processor clocks in each: late answers hold the processor, they never
# lengthen a cycle.
#   VASM=/path/to/vasmm68k_mot   NOBUILD=1: reuse obj/vl
set -e
cd "$(dirname "$0")"
ROOT=../..
RTL=$ROOT/rtl
CPU=$RTL/AP68030/rtl
VASM=${VASM:-vasmm68k_mot}
mkdir -p obj

command -v "$VASM" >/dev/null 2>&1 || { echo "vasmm68k_mot not found (set VASM=...)"; echo "RESULT: FAIL"; exit 1; }
"$VASM" -Fbin -m68030 -no-opt -quiet -o obj/t_bustime.bin t_bustime.s
python3 -I $ROOT/tb/system/rom2hex64.py obj/t_bustime.bin obj/t_bustime.hex

if [ "${NOBUILD:-0}" != 1 ] || [ ! -x obj/vl/tb_bustime ]; then
	verilator --binary --timing -j "${JOBS:-8}" -O2 -Wno-fatal -Wno-lint -Wno-style -Wno-WIDTH \
		-Wno-TIMESCALEMOD -Wno-CASEINCOMPLETE -Wno-MULTIDRIVEN -Wno-UNOPTFLAT \
		--output-split 20000 --output-split-cfuncs 500 \
		-I$CPU -I$CPU/core --top-module tb_bustime --Mdir obj/vl -o tb_bustime \
		tb_bustime.sv $ROOT/tb/system/ddr3_model.sv \
		$CPU/ap030_top.v $CPU/ap030_core.v $CPU/ap030_memsys.v $CPU/ap030_mmu.v $CPU/ap030_cache.v \
		$CPU/ap030_bus.v $CPU/ap030_alu.v $CPU/ap030_muldiv.v $CPU/ap030_regfile.v \
		$RTL/falcon/falcon_cpuclk.sv $RTL/falcon/falcon_pipescan.sv $RTL/falcon/falcon_cpubus.sv $RTL/falcon/falcon_l2.sv $RTL/falcon/falcon_memarb.sv \
		> obj/build.log 2>&1 || { tail -30 obj/build.log; echo "RESULT: FAIL (build)"; exit 1; }
fi

fail=0
clocks=""
run() {
	name=$1; shift
	./obj/vl/tb_bustime +rom=obj/t_bustime.hex "$@" > obj/$name.log 2>&1 || true
	r=$(grep -E "^(PASS|FAIL)" obj/$name.log | head -1)
	c=$(sed -n 's/^PROCESSOR CLOCKS \([0-9]*\).*/\1/p' obj/$name.log)
	printf "  %-10s %s  (%s processor clocks; %s)\n" "$name" "${r:-no result}" "$c" \
		"$(grep -h '^debt peak' obj/$name.log | cut -d';' -f1)"
	[ "$r" = PASS ] || { fail=1; grep -h "TIMING:\|FAIL" obj/$name.log | head -5 | sed 's/^/      /'; }
	clocks="$clocks $c"
}
run base
run vidload +vidload
run mhz8 +mhz8
run seed2 +seed=2 +vidload
run seed3 +seed=3 +vidload +mhz8
grep -h "^cycle" obj/base.log | sed 's/^/  /'
if [ "$(echo $clocks | tr ' ' '\n' | sort -u | wc -l)" != 1 ]; then
	echo "  processor clocks differ between the runs:$clocks"
	fail=1
fi
if [ $fail -eq 0 ]; then echo "RESULT: PASS"; else echo "RESULT: FAIL"; exit 1; fi
