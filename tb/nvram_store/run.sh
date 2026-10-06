#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
# Build the production Main persistence implementation into the RTL test.
MAIN=${MAIN:-/home/adam/MiSTer_Main/Main_MiSTer_falcon}
MAIN=$(realpath "$MAIN")
verilator --cc --exe --build -j 4 -O2 -Wall -Wno-fatal -Wno-PINMISSING -Wno-UNUSEDPARAM -Wno-DECLFILENAME \
    -Wno-PINCONNECTEMPTY -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-BLKSEQ \
    -Wno-WIDTHTRUNC -Wno-PROCASSINIT --top-module tb_nvram_store \
    --Mdir obj_dir hps_io.vlt ../../sys/hps_io.sv ../../rtl/falcon/falcon_nvram.sv \
    ../../rtl/falcon/falcon_nvram_store.sv tb_nvram_store.sv tb_nvram_store.cpp \
    "$MAIN/support/falcon/falcon_nvram.cpp" \
    -CFLAGS "-I$MAIN/support/falcon" > build.log 2>&1 || \
    { tail -60 build.log; echo 'FINAL: FAIL (build)'; exit 1; }
if ./obj_dir/Vtb_nvram_store | tee sim.log; then
    echo 'FINAL: PASS'
else
    echo 'FINAL: FAIL'
    exit 1
fi
