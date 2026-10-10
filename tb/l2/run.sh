#!/bin/bash
# The CPU's line cache (falcon_l2) with falcon_memarb on the DDR3 model:
# random CPU, DMA, loader and video traffic, the cache switched off and on;
# every read checked against the memory (tb_l2.sv).  Several seeds and DDR3
# timings.  JOBS=n for the build.
set -e
cd "$(dirname "$0")"
ROOT=../..
mkdir -p obj
verilator --binary --timing -j "${JOBS:-8}" -O2 -Wno-fatal -Wno-lint -Wno-style -Wno-WIDTH -Wno-TIMESCALEMOD \
	--top-module tb_l2 --Mdir obj/vl -o tb_l2 \
	tb_l2.sv $ROOT/rtl/falcon/falcon_l2.sv $ROOT/rtl/falcon/falcon_memarb.sv $ROOT/tb/system/ddr3_model.sv \
	> obj/build.log 2>&1 || { tail -30 obj/build.log; echo "RESULT: FAIL (build)"; exit 1; }
fail=0
run() {
	name=$1; shift
	./obj/vl/tb_l2 "$@" > obj/$name.log 2>&1 || true
	r=$(grep -E "^RESULT" obj/$name.log | head -1)
	printf "  %-8s %s  %s\n" "$name" "${r:-no result}" "$(grep -h ' reads (' obj/$name.log)"
	[ "$r" = "RESULT: PASS" ] || { fail=1; grep -h "FAIL" obj/$name.log | head -5 | sed 's/^/      /'; }
}
run base   +seed=1
run lat    +seed=2 +ddrlat=20
run busy   +seed=3 +ddrbusy=30 +ddrspike=200
run fast   +seed=4 +ddrlat=0 +ddrseed=777
run long   +seed=5 +clocks=1500000 +ddrbusy=8
run hot    +seed=6 +hot=80
run hotlat +seed=7 +hot=90 +ddrlat=25 +ddrbusy=20
run hits   +seed=8 +hot=100 +dma=0 +toggle=0
run mix    +seed=9 +hot=90 +dma=300 +toggle=0 +ddrbusy=10
if [ $fail -eq 0 ]; then echo "RESULT: PASS"; else echo "RESULT: FAIL"; exit 1; fi
