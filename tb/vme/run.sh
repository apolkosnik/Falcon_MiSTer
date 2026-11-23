#!/bin/bash
# The 2D engine against a byte-by-byte model: ./run.sh [tb_vme arguments]
#   ENGINE=file.sv   another vme_atw800 (OLD16=1 if its memory port is the 16-bit one)
#   DEFS="-DX"       passed to Verilator
cd "$(dirname "$0")" || exit 1
RTL=../../rtl
OBJ=${OBJ:-obj}
ENGINE=${ENGINE:-$RTL/vme/vme_atw800.sv}
verilator --cc --exe --build -j 8 -O2 -Wno-fatal ${OLD16:+-DOLD16} $DEFS \
    -Mdir "$OBJ" --top-module tb_vme_top -CFLAGS -O2 \
    tb_vme_top.sv "$ENGINE" $RTL/falcon/falcon_memarb.sv ../system/ddr3_model.sv tb_vme.cpp > "$OBJ.log" 2>&1 \
    || { tail -30 "$OBJ.log"; echo "build FAILED"; exit 1; }
"$OBJ/Vtb_vme_top" ${OLD16:+--old} "$@"
