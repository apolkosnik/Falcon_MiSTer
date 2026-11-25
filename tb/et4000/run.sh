#!/bin/bash
# The Nova ET4000 card against a model of the chip: ./run.sh [tb_et4k arguments]
cd "$(dirname "$0")" || exit 1
RTL=../../rtl
OBJ=${OBJ:-obj}
verilator --cc --exe --build -j 8 -O2 -Wno-fatal $DEFS \
    -Mdir "$OBJ" --top-module tb_et4k_top -CFLAGS -O2 \
    tb_et4k_top.sv $RTL/vme/vme_et4000.sv $RTL/falcon/falcon_memarb.sv ../system/ddr3_model.sv tb_et4k.cpp > "$OBJ.log" 2>&1 \
    || { tail -30 "$OBJ.log"; echo "build FAILED"; exit 1; }
"$OBJ/Vtb_et4k_top" "$@"
