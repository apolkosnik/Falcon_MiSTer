#!/bin/bash
# Build and run the falcon_acia / falcon_ikbd Verilator testbenches:
#   tb_acia.cpp  ACIA registers, serial timing, IKBD commands, keys, mouse, clock
#   tb_joy.cpp   IKBD joystick behaviour (audit against Hatari), buttons 2/3
set -e
cd "$(dirname "$0")"
RTL=../../rtl/falcon
SRC="$RTL/falcon_acia.sv $RTL/falcon_ikbd.sv $RTL/falcon_ikbd_keymap.sv"
WARN="-Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-BLKSEQ -Wno-PROCASSINIT"
rm -rf obj_dir obj_joy
for t in acia joy; do
    md=obj_dir; [ $t = joy ] && md=obj_joy
    verilator --cc --exe --build -O2 -j 4 $WARN --top-module falcon_acia --Mdir $md \
        $SRC tb_$t.cpp -o Vtb_$t > build_$t.log 2>&1 || { cat build_$t.log; echo "FINAL: FAIL (build $t)"; exit 1; }
done
set +e
./obj_dir/Vtb_acia | tee sim.log
rc1=${PIPESTATUS[0]}
./obj_joy/Vtb_joy | tee sim_joy.log
rc2=${PIPESTATUS[0]}
echo
echo "tb_acia: $(grep -c '  PASS:' sim.log) passed, $(grep -c '  FAIL:' sim.log) failed"
echo "tb_joy:  $(grep -c '  PASS:' sim_joy.log) passed, $(grep -c '  FAIL:' sim_joy.log) failed"
if [ $rc1 -eq 0 ] && [ $rc2 -eq 0 ]; then echo "FINAL: PASS"; exit 0; else echo "FINAL: FAIL"; exit 1; fi
