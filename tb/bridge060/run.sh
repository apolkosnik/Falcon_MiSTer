#!/bin/sh
# falcon_cpubus060 + falcon_memarb + DDR3 model (Verilator)
set -e
cd "$(dirname "$0")"
R=../../rtl
verilator --binary --timing --build -j 8 -Wno-fatal -Wno-WIDTH -Wno-PINMISSING -Wno-TIMESCALEMOD \
	--top-module tb_bridge060 --Mdir obj_dir tb_bridge060.sv \
	$R/falcon/falcon_cpubus060.sv $R/falcon/falcon_memarb.sv ../system/ddr3_model.sv > build.log 2>&1 || { tail -20 build.log; exit 1; }
./obj_dir/Vtb_bridge060
