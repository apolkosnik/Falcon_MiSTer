#!/bin/sh
# Full Quartus build of the Falcon core.
#
# Quartus 17.0's quartus_fit sometimes crashes while exiting on newer Linux
# distributions ("ended unexpectedly") after it has written a complete fit.
# This script runs the stages one by one, accepts the fit when the fit
# report says it succeeded, and then runs the assembler and the timing
# analyser itself.  It prints the timing verdict at the end.
#
# Only processes started by this script are ever waited on; nothing is killed.
set -u
cd "$(dirname "$0")/.."
Q=/opt/intelFPGA_lite/17.0/quartus/bin
P=Falcon
LOG=output_files/build.log
mkdir -p output_files
: > $LOG

run() {
	echo "== $*" | tee -a $LOG
	"$@" >> $LOG 2>&1
}

$Q/quartus_sh -t sys/build_id.tcl >> $LOG 2>&1 || true
run $Q/quartus_map $P || { echo "synthesis failed (see $LOG)"; exit 1; }
run $Q/quartus_fit $P
if ! grep -q "Fitter Status : Successful" output_files/$P.fit.summary 2>/dev/null; then
	echo "fit failed (see $LOG)"; exit 1
fi
run $Q/quartus_asm $P || { echo "assembler failed (see $LOG)"; exit 1; }
run $Q/quartus_sta $P || { echo "timing analysis failed (see $LOG)"; exit 1; }

grep -E "Logic utilization|RAM Blocks|DSP Blocks" output_files/$P.fit.summary
NEG=$(grep -c "Slack : -" output_files/$P.sta.summary)
grep -E "Type|Slack" output_files/$P.sta.summary | paste - - | grep "emu|pll" 
if [ "$NEG" -eq 0 ]; then echo "TIMING MET: output_files/$P.rbf"; else echo "TIMING FAILED: $NEG negative slacks"; exit 2; fi
