#!/bin/bash
# Run FPUTEST (or FPUBENCH) in the full-system simulation (tb/system: the
# whole core's RTL, the FPU served by libfpe in the simulator) on the stand-in
# ROM simrom.s instead of TOS, and print what the program wrote.
#   sim.sh [test|bench]
#   STRIDE=n  FPUTEST: only every nth check (default 25; 1 = the whole sequence)
#   PASSES=n  FPUTEST passes (default 1)
#   MS=n      simulated milliseconds at most (default 4000)
#   VASM=/path/to/vasmm68k_mot
# Needs tb/system/build.sh to have been run.
set -e
cd "$(dirname "$0")"
ROOT=$PWD/../..
VASM=${VASM:-vasmm68k_mot}
ENG=../falcon_fpu/engine
OUT=$PWD/build/sim
mkdir -p $OUT
[ -f $ENG/build/host/libfpe.a ] || HOST_ONLY=1 $ENG/build.sh
cc -O2 -Wall -Wextra -I$ENG -o build/gen_fputest gen_fputest.c $ENG/build/host/libfpe.a -lm
if [ "${1:-test}" = bench ]; then
	$VASM -Ftos -m68030 -m68882 -no-opt -quiet -o $OUT/prg.tos fpubench.s
else
	build/gen_fputest -s ${STRIDE:-25} > $OUT/gen.s
	$VASM -Ftos -m68030 -m68882 -no-opt -quiet -I$OUT -DPASSES=${PASSES:-1} -o $OUT/prg.tos fputest.s
fi
$VASM -Fbin -m68030 -m68882 -no-opt -quiet -I$OUT -o $OUT/simrom.img simrom.s
python3 -I $ROOT/tb/system/rom2hex64.py $OUT/simrom.img $OUT/simrom.hex
cd $ROOT/tb/system
./obj_dir/Vtb_top +rom=$OUT/simrom.hex --ms ${MS:-4000} --fpu --text 100000 --done FFFF0 | grep -v "frame [0-9]*:"
