#!/bin/bash
# Build and run the Falcon SCSI Verilator testbench.
# DUT: rtl/falcon/falcon_fdc.sv (EXT_SCSI = 1) + rtl/falcon/falcon_scsi.sv
# (tb_scsi_top.sv wires them as falcon_system does).  The HPS side links the
# Main_MiSTer sources that ship (support/falcon/falcon_scsi.cpp/.h, copied
# from the Main tree at build time) with the shims in shim/.
set -u
cd "$(dirname "$0")"
RTL=../../rtl/falcon
MAIN=${MAIN_MISTER:-/home/adam/MiSTer_Main/Main_MiSTer_falcon}
rm -rf obj_dir main_src work
mkdir -p main_src/support/falcon work
cp "$MAIN/support/falcon/falcon_scsi.cpp" "$MAIN/support/falcon/falcon_scsi.h" main_src/support/falcon/ || {
	echo "SCSI SUMMARY: FAIL (Main sources not found in $MAIN)"; exit 1; }
cp shim/file_io.h shim/spi.h shim/user_io.h shim/main_shim.h shim/main_shim.cpp main_src/
SRC="tb_scsi_top.sv $RTL/falcon_fdc.sv $RTL/falcon_scsi.sv tb_scsi.cpp main_src/main_shim.cpp main_src/support/falcon/falcon_scsi.cpp"
verilator --cc --exe --build -j 4 -O3 -Wno-fatal -Wno-WIDTHEXPAND -Wno-WIDTHTRUNC -Wno-BLKSEQ \
	--top-module tb_scsi_top $SRC -CFLAGS "-O2 -I$PWD/main_src -Wall" -o tb_scsi > build.log 2>&1
if [ $? -ne 0 ]; then
	cat build.log
	echo "SCSI SUMMARY: FAIL (build)"
	exit 1
fi
./obj_dir/tb_scsi | tee run.log
rc=${PIPESTATUS[0]}
if [ $rc -eq 0 ] && grep -q "SCSI TEST PASS" run.log; then
	echo "SCSI SUMMARY: PASS"
	exit 0
fi
echo "SCSI SUMMARY: FAIL"
exit 1
