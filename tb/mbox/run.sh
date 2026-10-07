#!/bin/bash
# Build and run the FPGA<->ARM mailbox latency probe co-simulation:
# falcon_mbox_test + falcon_memarb (FALCON_MBOX_TEST) against the real host tool
# tools/mbox_ping/mbox_ping.c (built here with cc into obj_dir_a/).
# Two builds: CLK_HZ=1000000 (all tests) and CLK_HZ=32000000 (CLKMHZ = 32 check).
set -e
cd "$(dirname "$0")"
ROOT=../..
rm -rf obj_dir_a obj_dir_b
mkdir -p obj_dir_a obj_dir_b

cc -O2 -Wall -Wextra -o obj_dir_a/mbox_ping_host $ROOT/tools/mbox_ping/mbox_ping.c \
	|| { echo "RESULT: FAIL (tool build)"; exit 1; }

build() {   # dir clk_hz exe
	verilator --cc --exe --build -j 4 -O2 -Wall -Wno-fatal -Wno-UNUSEDSIGNAL -Wno-DECLFILENAME \
		+define+FALCON_MBOX_TEST --top-module tb_mbox_top -GCLK_HZ=$2 --Mdir $1 \
		$ROOT/rtl/falcon/falcon_memarb.sv $ROOT/rtl/falcon/falcon_mbox_test.sv tb_mbox_top.sv tb_mbox.cpp \
		-CFLAGS "-O2 -Wall -Wno-unused-function -DTB_CLK_HZ=$2" -o $3 > $1/build.log 2>&1 \
		|| { cat $1/build.log; echo "RESULT: FAIL (build)"; exit 1; }
	# warnings in this bench's own files must be fixed
	if grep -E "%Warning.*tb_mbox_top|tb_mbox\.cpp:[0-9]+:[0-9]+: warning" $1/build.log; then
		echo "warnings in tb/mbox files (see above)"
	fi
}
build obj_dir_a 1000000 tb_mbox_a
build obj_dir_b 32000000 tb_mbox_b

rc=0
./obj_dir_a/tb_mbox_a $PWD/obj_dir_a/mbox_ping_host $PWD/obj_dir_a | tee run_a.log || rc=1
./obj_dir_b/tb_mbox_b $PWD/obj_dir_a/mbox_ping_host $PWD/obj_dir_b quick | tee run_b.log || rc=1

if [ $rc -eq 0 ] && grep -q "RESULT: PASS" run_a.log && grep -q "RESULT: PASS" run_b.log; then
	echo "RESULT: PASS"
	exit 0
fi
echo "RESULT: FAIL"
exit 1
