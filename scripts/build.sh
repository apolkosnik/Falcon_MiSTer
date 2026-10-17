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

# The ap060 variant (FALCON_BRINGUP with CPU060) is at the edge of the
# device: synthesis for area (the core is ~1,300 ALUTs smaller), registers
# packed harder.  The HIGH PERFORMANCE EFFORT mode forces speed in
# synthesis, so synthesis runs in BALANCED mode with technique AREA; the
# fitter then runs in HIGH PERFORMANCE EFFORT again (the HDMI clock needs
# its timing effort).  Project-file assignments win over files.qip's, so
# these go into Falcon.qsf (restored on exit like the rest).
# With everything in (the DSP, the 50 MHz processor, Fast RAM) the device is
# 99 % full, and four more settings make it fit and meet timing:
#  - no physical synthesis in the AP68060 (its register duplication and
#    retiming for the 50 MHz clock cost ~1,400 ALMs; it meets 50 MHz without)
#  - the scaler optimized for speed, with physical synthesis (its HDMI
#    counter paths need it)
#  - small RAMs stay in block RAM (no MLAB conversion: MLABs take logic LABs)
#  - routability optimization always, and no redundant-cell removal (with it
#    the scaler's HDMI paths miss timing at this density)
AREA060=0
case " ${FALCON_BRINGUP:-} " in *" CPU060 "*)
	AREA060=1
	sed -i -e 's/^set_global_assignment -name OPTIMIZATION_MODE .*/set_global_assignment -name OPTIMIZATION_MODE BALANCED/' \
	       -e 's/^set_global_assignment -name OPTIMIZATION_TECHNIQUE .*/set_global_assignment -name OPTIMIZATION_TECHNIQUE AREA/' \
	       -e 's/^set_global_assignment -name ALM_REGISTER_PACKING_EFFORT .*/set_global_assignment -name ALM_REGISTER_PACKING_EFFORT HIGH/' \
	       -e '/^set_global_assignment -name REMOVE_REDUNDANT_LOGIC_CELLS /d' $P.qsf
	[ -z "$(tail -c1 $P.qsf)" ] || echo >> $P.qsf
	cat >> $P.qsf <<-'EOF'
	set_global_assignment -name BLOCK_RAM_TO_MLAB_CELL_CONVERSION OFF
	set_global_assignment -name FITTER_AGGRESSIVE_ROUTABILITY_OPTIMIZATION ALWAYS
	set_instance_assignment -name ADV_NETLIST_OPT_ALLOWED "NEVER ALLOW" -to "emu:emu|falcon_system:system|ap68060:cpu"
	set_instance_assignment -name ADV_NETLIST_OPT_ALLOWED "ALWAYS ALLOW" -to "ascal:ascal"
	set_instance_assignment -name OPTIMIZATION_TECHNIQUE SPEED -to "ascal:ascal"
	EOF
	echo "== CPU060: synthesis for area (BALANCED, AREA), register packing HIGH; the fitter in HIGH PERFORMANCE EFFORT;" | tee -a $LOG
	echo "== CPU060: no physical synthesis in the CPU, the scaler for speed, no MLAB conversion, routability always" | tee -a $LOG ;;
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
