#!/bin/bash
# Build and run the falcon_ide Verilator testbench.
set -u
cd "$(dirname "$0")"
RTL=../../rtl/falcon
rm -rf obj_dir
verilator --cc --exe --build -j 4 -O2 -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC \
	--top-module falcon_ide $RTL/falcon_ide.sv tb_ide.cpp -o tb_ide > build.log 2>&1
if [ $? -ne 0 ]; then
	cat build.log
	echo "IDE SUMMARY: FAIL (build)"
	exit 1
fi
./obj_dir/tb_ide | tee run.log
rc=${PIPESTATUS[0]}
if [ $rc -eq 0 ] && grep -q "IDE TEST PASS" run.log; then
	echo "IDE SUMMARY: PASS"
	exit 0
fi
echo "IDE SUMMARY: FAIL"
exit 1
