#!/bin/sh
# Build the full-system simulation of the real RTL.
set -e
cd "$(dirname "$0")"
R=../../rtl
FIREBEE_V=$(ls build/vhdl/*.v 2>/dev/null || true)
verilator --cc --exe --build -j 16 -O3 --x-assign fast --x-initial fast \
	-Wno-fatal -Wno-WIDTH -Wno-CASEINCOMPLETE -Wno-PINMISSING -Wno-TIMESCALEMOD -Wno-MULTIDRIVEN \
	--top-module tb_top -Mdir obj_dir \
	-I$R/AP68030/rtl -I$R/AP68030/rtl/core -I$R/falcon -I$R/falcon/dsp \
	tb_top.sv ddr3_model.sv \
	$R/AP68030/rtl/*.v \
	$(ls $R/falcon/*.sv $R/falcon/*.v 2>/dev/null) \
	$(ls $R/falcon/dsp/*.sv $R/falcon/dsp/*.v 2>/dev/null) \
	$FIREBEE_V \
	sim_main.cpp
