#!/bin/sh
# falcon_combel bench (Jaguar pads, paddles, DIP switches): Verilator
set -e
cd "$(dirname "$0")"
verilator --binary --timing -Wno-fatal -Wno-WIDTH --top-module tb_combel -Mdir obj_dir -o tb \
	tb_combel.sv ../../rtl/falcon/falcon_combel.sv > build.log 2>&1 || { cat build.log; exit 1; }
./obj_dir/tb | tee run.log
grep -q "PASS" run.log
