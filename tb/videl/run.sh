#!/bin/bash
# Build and run the falcon_videl Verilator testbench.
set -e
cd "$(dirname "$0")"
RTL=../../rtl/falcon/falcon_videl.sv
rm -rf obj_dir
verilator --cc --exe --build -O2 -Wall -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL \
	--top-module falcon_videl -CFLAGS "-O2" \
	$RTL tb_videl.cpp -o tb_videl > build.log 2>&1 || { cat build.log; echo "FAIL (build)"; exit 1; }
set +e
./obj_dir/tb_videl | tee run.log
rc=${PIPESTATUS[0]}
echo
if [ $rc -eq 0 ] && grep -q "^PASS$" run.log; then
	echo "SUMMARY: PASS ($(grep -c '\[PASS\]' run.log) checks)"
else
	echo "SUMMARY: FAIL ($(grep -c '\[FAIL\]' run.log) failing checks)"
	exit 1
fi
