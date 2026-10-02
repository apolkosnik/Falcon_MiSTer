#!/bin/bash
# Build and run the falcon_nvram Verilator testbench.
# Golden model: Hatari's src/falcon/nvram.c compiled into the test.
set -e
cd "$(dirname "$0")"
HATARI=${HATARI:-/home/adam/hatari}
RTL=../../rtl/falcon
OBJ=obj_dir
HCFLAGS="-O1 -w -I$HATARI/build -I$HATARI/src/includes -I$HATARI/src -I$HATARI/src/cpu -I$HATARI/src/debug -I$HATARI/src/falcon $(sdl2-config --cflags)"

mkdir -p $OBJ
gcc $HCFLAGS -c hatari_nvram_golden.c -o $OBJ/hatari_nvram_golden.o
rm -f $OBJ/Vtb_nvram_top
verilator --cc --exe --build -j 8 -O2 -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
    --top-module tb_nvram_top -Mdir $OBJ \
    $RTL/falcon_nvram.sv tb_nvram_top.sv tb_nvram.cpp \
    $PWD/$OBJ/hatari_nvram_golden.o \
    > $OBJ/build.log 2>&1 || { tail -40 $OBJ/build.log; echo "FAIL: build"; exit 1; }

./$OBJ/Vtb_nvram_top
