#!/bin/sh
# falcon_cpubus060 + ap060_fastram + falcon_ddr_arb + falcon_memarb + DDR3 model (Verilator)
set -e
cd "$(dirname "$0")"
R=../../rtl
F=$R/AP68060/rtl/fastram
verilator --binary --timing --build -j 8 -Wno-fatal -Wno-WIDTH -Wno-PINMISSING -Wno-TIMESCALEMOD \
	--top-module tb_bridge060 --Mdir obj_dir -I$F tb_bridge060.sv \
	$F/ap030_async_fifo.v $F/ap030_fastram_fe.v $F/ap030_fastram_be.v $F/ap030_l2ram.sv $F/ap060_fastram_bus.sv $F/ap060_fastram.sv \
	$R/falcon/falcon_cpubus060.sv $R/falcon/falcon_ddr_arb.sv $R/falcon/falcon_memarb.sv ../system/ddr3_model.sv > build.log 2>&1 || { tail -20 build.log; exit 1; }
./obj_dir/Vtb_bridge060
