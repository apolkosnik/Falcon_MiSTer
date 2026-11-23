#!/bin/sh
# falcon_cpubus read buffer vs a DMA snoop in its refill clock (Verilator)
set -e
cd "$(dirname "$0")"
R=../../rtl
verilator --binary --timing --build -j 8 -O2 -Wno-fatal -Wno-WIDTH -Wno-PINMISSING -Wno-TIMESCALEMOD \
	-Wno-MULTIDRIVEN -Wno-UNOPTFLAT --top-module tb_rb_race --Mdir obj_dir \
	-I$R/AP68030/rtl -I$R/AP68030/rtl/core -I$R/falcon -I$R/falcon/dsp \
	tb_rb_race.sv ../system/tb_top.sv ../system/ddr3_model.sv $R/AP68030/rtl/*.v \
	$(ls $R/falcon/*.sv | grep -v cpubus060) $R/falcon/dsp/*.sv > build.log 2>&1 || { tail -20 build.log; exit 1; }
./obj_dir/Vtb_rb_race
