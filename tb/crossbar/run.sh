#!/bin/bash
# Build and run the falcon_crossbar Verilator testbench.
set -e
cd "$(dirname "$0")"
ROOT=../..
rm -rf obj_dir
verilator --cc --exe --build -j 4 -O2 -Wall -Wno-BLKSEQ -Wno-UNUSEDSIGNAL \
	--top-module falcon_crossbar \
	$ROOT/rtl/falcon/falcon_crossbar.sv tb_crossbar.cpp \
	-CFLAGS "-O2 -include cstdarg" -o tb_crossbar > build.log 2>&1 || { cat build.log; echo "RESULT: FAIL (build)"; exit 1; }
./obj_dir/tb_crossbar | tee run.log
grep -q "RESULT: PASS" run.log
