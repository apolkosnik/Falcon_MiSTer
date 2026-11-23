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
# The project to build is the argument (default Falcon):
#   ./scripts/build.sh              Falcon.qpf     the AP68030
#   ./scripts/build.sh FalconFPU    FalconFPU.qpf  the AP68030, the ARM FPU bridge
#   ./scripts/build.sh Falcon040    Falcon040.qpf  the AP68040
#   ./scripts/build.sh Falcon060    Falcon060.qpf  the AP68060
# (FALCON_BRINGUP with CPU040 or CPU060 picks Falcon040 or Falcon060, as
# before the projects were split.)  The result is output_files/<project>.rbf,
# the log output_files/<project>.build.log.  The projects share db/ and
# output_files/: build one at a time in one checkout.
#
# Only processes started by this script are ever waited on; nothing is killed.
set -u
cd "$(dirname "$0")/.." || exit 1
Q=${QUARTUS:-/opt/intelFPGA_lite/17.0/quartus}
[ -x "$Q/bin/quartus_sh" ] && Q=$Q/bin
[ -x "$Q/quartus_sh" ] || { echo "no quartus_sh in $Q or $Q/bin (set QUARTUS)"; exit 1; }
P=${1:-}
if [ -z "$P" ]; then
	case " ${FALCON_BRINGUP:-} " in
	*" CPU040 "*) P=Falcon040 ;;
	*" CPU060 "*) P=Falcon060 ;;
	*) P=Falcon ;;
	esac
fi
case $P in Falcon|FalconFPU|Falcon040|Falcon060) ;; *) echo "unknown project $P (Falcon, FalconFPU, Falcon040, Falcon060)"; exit 1 ;; esac
LOG=output_files/$P.build.log
mkdir -p output_files
: > $LOG

# The build leaves the project's .qsf as it was.  Quartus saves the project when
# build_id.tcl closes it, and then writes every assignment it evaluated into
# the .qsf -- including the FALCON_BRINGUP macros files.qip adds, which would
# turn every later build into a bring-up or measurement build.
KEEP=output_files/.$P.qsf.keep
cp $P.qsf $KEEP
trap 'cp -f $KEEP $P.qsf; rm -f $KEEP' EXIT
trap 'exit 130' INT TERM

# Falcon040 and Falcon060 synthesize in BALANCED mode for area (their .qsf);
# the fitter runs in HIGH PERFORMANCE EFFORT (the HDMI clock needs its timing
# effort): the mode is switched between the two stages (the .qsf is restored
# on exit like the rest).
AREA060=0
case $P in Falcon040|Falcon060) AREA060=1
	echo "== $P: synthesis in BALANCED for area, the fitter in HIGH PERFORMANCE EFFORT" | tee -a $LOG ;;
esac

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
[ $AREA060 = 1 ] && sed -i 's/^set_global_assignment -name OPTIMIZATION_MODE .*/set_global_assignment -name OPTIMIZATION_MODE "HIGH PERFORMANCE EFFORT"/' $P.qsf
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
