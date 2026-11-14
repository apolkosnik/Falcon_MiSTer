#!/bin/bash
# Build and run the SCC Verilator testbench.
set -u
cd "$(dirname "$0")"
RTL=../../rtl/scc
rm -rf obj_dir
verilator --cc --exe --build -j 8 -O2 -Wall -Wno-fatal \
    --top-module scc $RTL/scc.v $RTL/scc_chan.v \
    tb_scc.cpp -o tb_scc > build.log 2>&1
if [ $? -ne 0 ]; then
    cat build.log
    echo "FAIL: build"
    exit 1
fi
./obj_dir/tb_scc
rc=$?
if [ $rc -eq 0 ]; then echo "RESULT: PASS"; else echo "RESULT: FAIL"; fi
exit $rc
