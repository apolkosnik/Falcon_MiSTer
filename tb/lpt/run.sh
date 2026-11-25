#!/bin/bash
# Build and run the printer port co-simulation: falcon_lpt against the real HPS
# program tools/falcon_lpt/falcon_lpt.c (built here with cc), sharing a file as
# the ring's memory.  The bench runs the module's clock at about real time, so
# the program's heartbeat means what it does on the board.
set -e
cd "$(dirname "$0")"
ROOT=../..
rm -rf obj_dir; mkdir -p obj_dir
cc -O2 -Wall -Wextra -o obj_dir/falcon_lpt_host $ROOT/tools/falcon_lpt/falcon_lpt.c \
	|| { echo "RESULT: FAIL (tool build)"; exit 1; }
verilator --cc --exe --build -j 4 -O2 -Wall -Wno-fatal -Wno-DECLFILENAME -Wno-PROCASSINIT -Wno-UNUSEDSIGNAL -Wno-WIDTHEXPAND \
	--top-module falcon_lpt -GCLK_HZ=100000 --Mdir obj_dir \
	$ROOT/rtl/falcon/falcon_lpt.sv tb_lpt.cpp \
	-CFLAGS "-O2 -Wall -DTB_CLK_HZ=100000" -o tb_lpt > obj_dir/build.log 2>&1 \
	|| { cat obj_dir/build.log; echo "RESULT: FAIL (build)"; exit 1; }
grep -E "%Warning" obj_dir/build.log && echo "warnings (see above)"
./obj_dir/tb_lpt $PWD/obj_dir/falcon_lpt_host $PWD/obj_dir
