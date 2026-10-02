#!/bin/bash
# Build and run the falcon_psg Verilator testbench.
# Golden model: Hatari's src/sound.c and src/psg.c compiled into the test.
set -e
cd "$(dirname "$0")"
HATARI=${HATARI:-/home/adam/hatari}
RTL=../../rtl/falcon
OBJ=obj_dir
HCFLAGS="-O1 -w -I$HATARI/build -I$HATARI/src/includes -I$HATARI/src -I$HATARI/src/cpu -I$HATARI/src/debug -I$HATARI/src/falcon $(sdl2-config --cflags)"

mkdir -p $OBJ
# Golden objects from Hatari sources
gcc $HCFLAGS -c hatari_ym_golden.c  -o $OBJ/hatari_ym_golden.o
gcc $HCFLAGS -c hatari_psg_golden.c -o $OBJ/hatari_psg_golden.o

# Regenerate the volume table from Hatari's code and make sure the RTL copy
# is identical
gcc -w gen_vol_hex.c $OBJ/hatari_ym_golden.o $OBJ/hatari_psg_golden.o -lm -o $OBJ/gen_vol_hex
$OBJ/gen_vol_hex $OBJ/falcon_psg_vol.mem
if ! cmp -s $OBJ/falcon_psg_vol.mem $RTL/falcon_psg_vol.mem; then
    echo "FAIL: rtl/falcon/falcon_psg_vol.mem differs from the table built by Hatari's sound.c"
    exit 1
fi

rm -f $OBJ/Vtb_psg_top
verilator --cc --exe --build -j 8 -O2 --public-flat-rw -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
    --top-module tb_psg_top -Mdir $OBJ \
    $RTL/falcon_psg.sv tb_psg_top.sv tb_psg.cpp \
    $PWD/$OBJ/hatari_ym_golden.o $PWD/$OBJ/hatari_psg_golden.o \
    -LDFLAGS -lm > $OBJ/build.log 2>&1 || { tail -40 $OBJ/build.log; echo "FAIL: build"; exit 1; }

./$OBJ/Vtb_psg_top
