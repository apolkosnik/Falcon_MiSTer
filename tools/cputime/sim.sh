#!/bin/bash
# Run CPUTIME in the full-system simulation (tb/system: the whole core's RTL
# on the DDR3 model, the timing counters on) on the stand-in ROM
# tools/fputest/simrom.s instead of TOS, and print what it wrote.
#   sim.sh [sim_main options, e.g. --rdlat 0.0005 --monitor 0]
#   MS=n     simulated milliseconds at most (default 300)
#   OUT=dir  work directory (default build/sim; one per run for parallel runs)
#   PRG=file the program (default: assemble cputime.s with ref.i)
#   VASM=/path/to/vasmm68k_mot
# Needs tb/system/build.sh to have been run.
set -e
cd "$(dirname "$0")"
ROOT=$PWD/../..
VASM=${VASM:-vasmm68k_mot}
OUT=$(realpath -m "${OUT:-$PWD/build/sim}")
mkdir -p $OUT
if [ -n "${PRG:-}" ]; then cp "$PRG" $OUT/prg.tos
else $VASM -Ftos -m68030 -no-opt -quiet -I../../tb/bustime/timing -o $OUT/prg.tos cputime.s; fi
$VASM -Fbin -m68030 -m68882 -no-opt -quiet -I$OUT -o $OUT/simrom.img ../fputest/simrom.s
python3 -I $ROOT/tb/system/rom2hex64.py $OUT/simrom.img $OUT/simrom.hex
cd $ROOT/tb/system
./obj_dir/Vtb_top +rom=$OUT/simrom.hex --ms ${MS:-300} --fpu --tstat --text 100000 --done FFFF0 "$@" | grep -v "frame [0-9]*:"
