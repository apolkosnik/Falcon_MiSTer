#!/bin/bash
# System bus integration test (tb_bus.sv): builds the real falcon_system with
# the CPU pins driven by the bench and exits nonzero on any failed check.
set -euo pipefail
# from the repository root: the RTL loads its .mem tables by that path
cd "$(dirname "$0")/../.."
R=${RTL:-rtl}
T=tb/integration
O=${OBJ:-$T/obj}
verilator --binary --timing --build -j 8 -O2 -Wno-fatal -Wno-WIDTH \
 -Wno-PINMISSING -Wno-TIMESCALEMOD -Wno-MULTIDRIVEN -Wno-UNOPTFLAT \
 --top-module tb_bus --Mdir $O \
 -I$R/AP68030/rtl -I$R/AP68030/rtl/core -I$R/falcon -I$R/falcon/dsp \
 $T/tb_bus.sv tb/system/tb_top.sv tb/system/ddr3_model.sv \
 $R/AP68030/rtl/*.v $R/falcon/*.sv $R/falcon/dsp/*.sv \
 > $O.build.log 2>&1 || { tail -30 $O.build.log; exit 1; }
./$O/Vtb_bus | tee $O.run.log
./$O/Vtb_bus +falcon | tee $O.falcon.log

# DMA memory types at the arbiter's ports
verilator --binary --timing --build -j 8 -Wno-fatal -Wno-WIDTH -Wno-PINMISSING -Wno-TIMESCALEMOD \
 --top-module tb_memarb --Mdir ${O}_memarb \
 $T/tb_memarb.sv tb/system/ddr3_model.sv $R/falcon/falcon_memarb.sv \
 > ${O}_memarb.build.log 2>&1 || { tail -30 ${O}_memarb.build.log; exit 1; }
./${O}_memarb/Vtb_memarb | tee ${O}_memarb.run.log
