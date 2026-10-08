#!/bin/bash
# Build and run the MC68882 FPU bridge co-simulation (milestone 1):
#   AP68030 + falcon_cpubus + falcon_memarb + falcon_fpu_bridge (real RTL),
#   68k test programs asm/*.s (vasm, 68030 + 68882), C++ DDR3 model with a
#   shared-file mailbox, and the real tools/falcon_fpu service built for the
#   host (cc).  Prints every check with expected vs got and a PASS/FAIL summary.
#
#   VASM=/path/to/vasmm68k_mot  (default vasmm68k_mot, as rtl/AP68030/tb/run_tests.sh)
#   NOBUILD=1                   reuse obj/ (programs are always re-assembled)
#   CLK_HZ                      bridge clock parameter (default 200000 = poll every 1000 clocks)
#   BRIDGE_SV / CPUBUS_SV / MEMARB_SV  alternative RTL files (mutation experiments)
#   ONLY=name                   run one scenario (detect_alive, detect_nosvc, detect_badmagic,
#                               detect_hb0, detect_real, watch_freeze, watch_magic, watch_real,
#                               frames, cond, gen, cpid_alive, cpid_absent, absent, reset, raw, rawna, straddle)
set -e
cd "$(dirname "$0")"
ROOT=../..
RTL=$ROOT/rtl
CPU=$RTL/AP68030/rtl
VASM=${VASM:-vasmm68k_mot}
CLK_HZ=${CLK_HZ:-200000}
mkdir -p obj

command -v "$VASM" >/dev/null 2>&1 || { echo "vasmm68k_mot not found (set VASM=...)"; echo "RESULT: FAIL (no assembler)"; exit 1; }

echo "== assembling test programs ($($VASM -v 2>&1 | head -1)) =="
for t in t_detect t_watch t_frames t_cond t_gen t_cpid t_absent t_reset t_straddle t_raw t_rawna; do
	( cd asm && $VASM -quiet -Fbin -m68030 -m68882 -no-opt -o ../obj/$t.bin $t.s ) \
		|| { echo "RESULT: FAIL (assembling $t)"; exit 1; }
done

echo "== building falcon_fpu_host (tools/falcon_fpu/falcon_fpu.c, host cc) =="
cc -O2 -Wall -Wextra -o obj/falcon_fpu_host $ROOT/tools/falcon_fpu/falcon_fpu.c \
	|| { echo "RESULT: FAIL (falcon_fpu host build)"; exit 1; }

if [ "${NOBUILD:-0}" != 1 ] || [ ! -x obj/vl/tb_fpu ] || [ -n "$BRIDGE_SV$CPUBUS_SV$MEMARB_SV" ]; then
	echo "== verilating (CLK_HZ=$CLK_HZ) =="
	rm -rf obj/vl
	verilator --cc --exe --build -j "${JOBS:-8}" -O2 -Wall -Wno-fatal \
		-Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY -Wno-TIMESCALEMOD \
		-Wno-CASEINCOMPLETE -Wno-WIDTH -Wno-MULTIDRIVEN -Wno-UNOPTFLAT -Wno-LATCH -Wno-VARHIDDEN \
		--output-split 20000 --output-split-cfuncs 500 \
		-I$CPU -I$CPU/core --top-module tb_fpu_top -GCLK_HZ=$CLK_HZ --Mdir obj/vl \
		$CPU/ap030_top.v $CPU/ap030_core.v $CPU/ap030_memsys.v $CPU/ap030_mmu.v $CPU/ap030_cache.v \
		$CPU/ap030_bus.v $CPU/ap030_alu.v $CPU/ap030_muldiv.v $CPU/ap030_regfile.v \
		${CPUBUS_SV:-$RTL/falcon/falcon_cpubus.sv} ${MEMARB_SV:-$RTL/falcon/falcon_memarb.sv} ${BRIDGE_SV:-$RTL/falcon/falcon_fpu_bridge.sv} \
		tb_fpu_top.sv tb_fpu.cpp \
		-CFLAGS "-O2 -Wall -Wextra -DTB_CLK_HZ=$CLK_HZ" -o tb_fpu > obj/build.log 2>&1 \
		|| { tail -40 obj/build.log; echo "RESULT: FAIL (build)"; exit 1; }
	if grep -E "tb_fpu_top\.sv|tb_fpu\.cpp:[0-9]+:[0-9]+: warning" obj/build.log; then
		echo "warnings in tb/fpu files (see above)"
	fi
fi

echo "== running =="
rc=0
./obj/vl/tb_fpu --host=obj/falcon_fpu_host ${ONLY:+--only=$ONLY} 2>&1 | tee obj/run.log || rc=1
grep -q "^RESULT: PASS" obj/run.log || rc=1
if [ $rc -eq 0 ]; then
	echo "RESULT: PASS"
	exit 0
fi
echo "RESULT: FAIL"
exit 1
