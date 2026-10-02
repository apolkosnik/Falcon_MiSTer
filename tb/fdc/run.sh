#!/bin/bash
# Build and run the falcon_fdc Verilator testbench.
# Three builds of the real module:
#   obj_dir       CLK_HZ = 32 MHz, DRV_PRESENT = 11 : full suite
#   obj_absent_b  CLK_HZ = 32 MHz, DRV_PRESENT = 01 : drive B not connected
#   obj_frac      CLK_HZ = 28.636363 MHz            : fractional FDC clock enable
set -u
cd "$(dirname "$0")"
RTL=../../rtl/falcon
SRC="$RTL/falcon_fdc.sv $RTL/falcon_fdc_ncr5380.sv tb_fdc.cpp"
OPTS="--cc --exe --build -j 4 -O3 -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-BLKSEQ --top-module falcon_fdc"
rm -rf obj_dir obj_absent_b obj_frac
verilator $OPTS $SRC -CFLAGS "-O2 -DTB_CLK_HZ=32000000" -o tb_fdc > build.log 2>&1 &&
verilator $OPTS -GDRV_PRESENT=1 -Mdir obj_absent_b $SRC -CFLAGS "-O2 -DTB_CLK_HZ=32000000" -o tb_fdc >> build.log 2>&1 &&
verilator $OPTS -GCLK_HZ=28636363 -Mdir obj_frac $SRC -CFLAGS "-O2 -DTB_CLK_HZ=28636363" -o tb_fdc >> build.log 2>&1
if [ $? -ne 0 ]; then
	cat build.log
	echo "FDC SUMMARY: FAIL (build)"
	exit 1
fi
./obj_dir/tb_fdc | tee run.log
rc1=${PIPESTATUS[0]}
./obj_absent_b/tb_fdc absent_b | tee run_absent_b.log
rc2=${PIPESTATUS[0]}
./obj_frac/tb_fdc frac | tee run_frac.log
rc3=${PIPESTATUS[0]}
if [ $rc1 -eq 0 ] && [ $rc2 -eq 0 ] && [ $rc3 -eq 0 ] && grep -q "FDC TEST PASS" run.log &&
   grep -q "FDC TEST PASS" run_absent_b.log && grep -q "FDC TEST PASS" run_frac.log; then
	echo "FDC SUMMARY: PASS"
	exit 0
fi
echo "FDC SUMMARY: FAIL"
exit 1
