#!/bin/bash
# Build and run the falcon_acia / falcon_ikbd Verilator testbench.
set -e
cd "$(dirname "$0")"
RTL=../../rtl/falcon
rm -rf obj_dir
verilator --cc --exe --build -O2 -j 4 \
    -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND -Wno-BLKSEQ -Wno-PROCASSINIT \
    --top-module falcon_acia \
    $RTL/falcon_acia.sv $RTL/falcon_ikbd.sv $RTL/falcon_ikbd_keymap.sv \
    tb_acia.cpp -o Vtb_acia > build.log 2>&1 || { cat build.log; echo "RESULT: FAIL (build)"; exit 1; }
set +e
./obj_dir/Vtb_acia | tee sim.log
rc=${PIPESTATUS[0]}
echo
grep -c "  PASS:" sim.log | sed 's/^/checks passed: /'
grep -c "  FAIL:" sim.log | sed 's/^/checks failed: /'
if [ $rc -eq 0 ]; then echo "FINAL: PASS"; else echo "FINAL: FAIL"; fi
exit $rc
