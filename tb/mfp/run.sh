#!/bin/bash
# Build and run the falcon_mfp Verilator testbench.
set -u
cd "$(dirname "$0")"
RTL=../../rtl/falcon
rm -rf obj_dir
verilator --cc --exe --build -j 8 -O2 -Wall -Wno-fatal \
    --public-flat-rw --top-module falcon_mfp \
    $RTL/falcon_mfp.sv $RTL/falcon_mfp_timer.sv $RTL/falcon_mfp_usart.sv \
    tb_mfp.cpp -o tb_mfp > build.log 2>&1
if [ $? -ne 0 ]; then
    cat build.log
    echo "FAIL: build"
    exit 1
fi
./obj_dir/tb_mfp
rc=$?
if [ $rc -eq 0 ]; then echo "RESULT: PASS"; else echo "RESULT: FAIL"; fi
exit $rc
