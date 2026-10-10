#!/bin/sh
# Full Quartus build of the Falcon core.
#
# Quartus 17.0's quartus_fit sometimes crashes while exiting on newer Linux
# distributions ("ended unexpectedly") after it has written a complete fit.
# This script runs the stages one by one, accepts the fit when the fit
# report says it succeeded, and then runs the assembler and the timing
# analyser itself.  It prints the timing verdict at the end.
#
# The project requires Quartus 17.0; /opt/intelFPGA_lite/17.0 is the default.
# QUARTUS selects another install, as its quartus directory or its bin:
#   QUARTUS=$HOME/intelFPGA_lite/17.0/quartus ./scripts/build.sh
#
# Only processes started by this script are ever waited on; nothing is killed.
set -u
cd "$(dirname "$0")/.." || exit 1
Q=${QUARTUS:-/opt/intelFPGA_lite/17.0/quartus}
[ -x "$Q/bin/quartus_sh" ] && Q=$Q/bin
[ -x "$Q/quartus_sh" ] || { echo "no quartus_sh in $Q or $Q/bin (set QUARTUS)"; exit 1; }
P=Falcon
LOG=output_files/build.log
mkdir -p output_files
: > $LOG

# The build leaves Falcon.qsf as it was.  Quartus saves the project when
# build_id.tcl closes it, and then writes every assignment it evaluated into
# the .qsf -- including the FALCON_BRINGUP macros files.qip adds, which would
# turn every later build into a bring-up or measurement build.
KEEP=output_files/.$P.qsf.keep
cp $P.qsf $KEEP
trap 'cp -f $KEEP $P.qsf; rm -f $KEEP' EXIT
trap 'exit 130' INT TERM

# A project database written by another Quartus version or edition is
# refused ("not compatible with the installed version"): start that one from
# scratch.  The whole version line counts, e.g. "17.0.0 Build 595 04/25/2017
# SJ Lite Edition" (a Standard Edition database does not open in Lite).
qver() { sed -n 's/.*Version \(.*[^[:space:]]\)[[:space:]]*$/\1/p' | head -n 1; }
WANT=$("$Q/quartus_sh" --version | qver)
HAVE=$(qver < db/$P.db_info 2>/dev/null)
if [ -n "$HAVE" ] && [ "$HAVE" != "$WANT" ]; then
	echo "== database from Quartus $HAVE, building with $WANT: removing db/ incremental_db/" | tee -a $LOG
	rm -rf db incremental_db
fi

run() {
	echo "== $*" | tee -a $LOG
	"$@" >> $LOG 2>&1
}

# build_id.tcl is the project's pre-flow hook (build_id.v, jtag.cdf); a flow
# passes it "<flow> <project> <revision>", the stand-alone stages do not run it
run "$Q"/quartus_sh -t sys/build_id.tcl compile $P $P || { echo "build_id.tcl failed (see $LOG)"; exit 1; }
run "$Q"/quartus_map $P || { echo "synthesis failed (see $LOG)"; exit 1; }
run "$Q"/quartus_fit $P
if ! grep -q "Fitter Status : Successful" output_files/$P.fit.summary 2>/dev/null; then
	echo "fit failed (see $LOG)"; exit 1
fi
run "$Q"/quartus_asm $P || { echo "assembler failed (see $LOG)"; exit 1; }
run "$Q"/quartus_sta $P || { echo "timing analysis failed (see $LOG)"; exit 1; }

grep -E "Logic utilization|RAM Blocks|DSP Blocks" output_files/$P.fit.summary
NEG=$(grep -c "Slack : -" output_files/$P.sta.summary)
grep -E "Type|Slack" output_files/$P.sta.summary | paste - - | grep "emu|pll" 
if [ "$NEG" -eq 0 ]; then echo "TIMING MET: output_files/$P.rbf"; else echo "TIMING FAILED: $NEG negative slacks"; exit 2; fi
