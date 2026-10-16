#!/bin/bash
# Blitter speed in the full-system simulation (tb/system: the whole core's
# RTL on the DDR3 model): runs blitbench.s, a stand-in ROM, and prints for
# each blit its name, the system clocks (real time) and the processor clocks
# of Falcon time it took.
#   sim.sh [simulator arguments, e.g. +ddrlat=20 (tb/system/ddr3_model.sv)]
#   TURBO=1   the CPU at 32 MHz (OSD turbo) instead of the Falcon's 16 MHz
#   MS=n      simulated milliseconds at most (default 300)
#   VASM=/path/to/vasmm68k_mot
# Needs tb/system/build.sh to have been run.
set -e
cd "$(dirname "$0")"
ROOT=$PWD/../..
VASM=${VASM:-vasmm68k_mot}
OUT=${OUT:-$PWD/build}
mkdir -p $OUT
$VASM -Fbin -m68030 -no-opt -quiet -o $OUT/blitbench.img blitbench.s
python3 -I $ROOT/tb/system/rom2hex64.py $OUT/blitbench.img $OUT/blitbench.hex
cd $ROOT/tb/system
./obj_dir/Vtb_top +rom=$OUT/blitbench.hex --ms ${MS:-300} --tstat ${TURBO:+--turbo} --text 100000 --done FFFF0 "$@" | grep -v "frame [0-9]*:"
