#!/bin/bash
# Build and run the MC68882 FPU bridge co-simulation (milestones 1 and 2):
#   AP68030 + falcon_cpubus + falcon_memarb + falcon_fpu_bridge (real RTL),
#   68k test programs asm/*.s (vasm, 68030 + 68882), a C++ DDR3 model with a
#   shared-file mailbox, and the real tools/falcon_fpu service (Hatari fpp.c
#   engine) built for the host as a child process.  The milestone 2 expected
#   values come from an independent golden: Hatari's fpp.c driven the normal
#   way with real EAs over a fake 68k RAM (golden.c, m2.cpp).
#   Prints every check with expected vs got and a PASS/FAIL summary.
#
#   VASM=/path/to/vasmm68k_mot  (default vasmm68k_mot, as rtl/AP68030/tb/run_tests.sh)
#   HATARI=...                  Hatari source tree (default ~/Devel/Atari/hatari)
#   NOBUILD=1                   reuse obj/vl (programs are always re-assembled)
#   CPU_DIV                     the CPU on every CPU_DIV-th clock: 1 (default, turbo), 2 (16 MHz), 4 (8 MHz)
#   CLK_HZ                      bridge clock parameter (default 1000000 = poll every 5000 clocks,
#                               watchdog 100000 clocks); runs with the real service are paced to
#                               wall time (1 clock = 1/CLK_HZ s) when the simulator is faster
#   BRIDGE_SV / CPUBUS_SV / MEMARB_SV  alternative RTL files (mutation experiments)
#   ONLY=name                   run one scenario: detect_real detect_nosvc detect_badmagic detect_hb0
#                               detect_badversion watch_freeze watch_magic watch_real frames cond gen
#                               cpid_alive cpid_absent absent reset raw rawna straddle
#                               m2cases m3sweeps m4irq m4irqk m4fsdie m2bg m2busy m2wd_stop m2wd_nosvc m2wd_presence
set -e
cd "$(dirname "$0")"
ROOT=../..
RTL=$ROOT/rtl
CPU=${CPU_DIR:-$RTL/AP68030/rtl}      # (CPU_DIR: alternative AP68030 sources, mutation experiments)
ENG=$ROOT/tools/falcon_fpu/engine
VASM=${VASM:-vasmm68k_mot}
HATARI=${HATARI:-$HOME/Devel/Atari/hatari}
CLK_HZ=${CLK_HZ:-1000000}
mkdir -p obj

command -v "$VASM" >/dev/null 2>&1 || { echo "vasmm68k_mot not found (set VASM=...)"; echo "RESULT: FAIL (no assembler)"; exit 1; }

echo "== Hatari engine library (tools/falcon_fpu/engine, host) =="
if [ ! -f obj/fpe/host/libfpe.a ] || [ -n "$(find $ENG -maxdepth 1 \( -name '*.c' -o -name '*.h' -o -name build.sh \) -newer obj/fpe/host/libfpe.a)" ] \
   || [ "$HATARI/src/cpu/fpp.c" -nt obj/fpe/host/libfpe.a ]; then
	HOST_ONLY=1 HATARI=$HATARI OUT=$PWD/obj/fpe $ENG/build.sh > obj/fpe.log 2>&1 \
		|| { tail -20 obj/fpe.log; echo "RESULT: FAIL (engine build)"; exit 1; }
fi
LIBFPE=$PWD/obj/fpe/host/libfpe.a

echo "== golden.c, falcon_fpu_host =="
H=$HATARI/src
cc -std=gnu99 -O2 -w -fwrapv -I$ENG -I$ENG/shim -I$H/cpu -I$H/cpu/softfloat -I$H/includes -I$H/debug -c golden.c -o obj/golden.o \
	|| { echo "RESULT: FAIL (golden build)"; exit 1; }
cc -O2 -Wall -Wextra -I$ROOT/tools/falcon_fpu -o obj/falcon_fpu_host $ROOT/tools/falcon_fpu/falcon_fpu.c $LIBFPE -lm \
	|| { echo "RESULT: FAIL (falcon_fpu host build)"; exit 1; }

if [ "${NOBUILD:-0}" != 1 ] || [ ! -x obj/vl/tb_fpu ] || [ -n "$BRIDGE_SV$CPUBUS_SV$MEMARB_SV$CPU_DIR" ]; then
	echo "== verilating (CLK_HZ=$CLK_HZ CPU_DIV=${CPU_DIV:-1}) =="
	rm -rf obj/vl
	verilator --cc --exe --build -j "${JOBS:-8}" -O2 -Wall -Wno-fatal \
		-Wno-UNUSEDSIGNAL -Wno-UNUSEDPARAM -Wno-DECLFILENAME -Wno-PINCONNECTEMPTY -Wno-TIMESCALEMOD \
		-Wno-CASEINCOMPLETE -Wno-WIDTH -Wno-MULTIDRIVEN -Wno-UNOPTFLAT -Wno-LATCH -Wno-VARHIDDEN \
		--output-split 20000 --output-split-cfuncs 500 \
		-I$CPU -I$CPU/core --top-module tb_fpu_top -GCLK_HZ=$CLK_HZ -GCPU_DIV=${CPU_DIV:-1} --Mdir obj/vl \
		$CPU/ap030_top.v $CPU/ap030_core.v $CPU/ap030_memsys.v $CPU/ap030_mmu.v $CPU/ap030_cache.v \
		$CPU/ap030_bus.v $CPU/ap030_alu.v $CPU/ap030_muldiv.v $CPU/ap030_regfile.v \
		${CPUBUS_SV:-$RTL/falcon/falcon_cpubus.sv} ${MEMARB_SV:-$RTL/falcon/falcon_memarb.sv} ${BRIDGE_SV:-$RTL/falcon/falcon_fpu_bridge.sv} \
		tb_fpu_top.sv tb_fpu.cpp m2.cpp m3.cpp \
		-CFLAGS "-O2 -Wall -Wextra -DTB_CLK_HZ=$CLK_HZ" -LDFLAGS "$PWD/obj/golden.o $LIBFPE -lm" -o tb_fpu > obj/build.log 2>&1 \
		|| { tail -40 obj/build.log; echo "RESULT: FAIL (build)"; exit 1; }
	if grep -E "tb_fpu_top\.sv|(tb_fpu|m2|m3)\.cpp:[0-9]+:[0-9]+: warning" obj/build.log; then
		echo "warnings in tb/fpu files (see above)"
	fi
fi

echo "== generating the milestone 2 cases =="
rm -f obj/gen_cases_*.s obj/gen_m3_*.s obj/t_cases[0-9]*.s obj/t_m3[0-9]*.s
./obj/vl/tb_fpu --gen=obj/gen_cases || { echo "RESULT: FAIL (case generation)"; exit 1; }
read NM2 NM3 < obj/gen_chunks.txt
for k in $(seq 0 $((NM2-1))); do sed "s/gen_cases.s/gen_cases_$k.s/" asm/t_cases.s > obj/t_cases$k.s; done
for k in $(seq 0 $((NM3-1))); do sed "s/gen_cases.s/gen_m3_$k.s/" asm/t_cases.s > obj/t_m3$k.s; done

echo "== assembling test programs ($($VASM -v 2>&1 </dev/null | head -1)) =="
ASMS="t_detect t_watch t_frames t_cond t_gen t_cpid t_absent t_reset t_straddle t_raw t_rawna t_bg t_busy t_wd t_irq t_fsdie"
for t in $ASMS; do
	( cd asm && $VASM -quiet -Fbin -m68030 -m68882 -no-opt -I../obj -L ../obj/$t.lst -o ../obj/$t.bin $t.s ) \
		|| { echo "RESULT: FAIL (assembling $t)"; exit 1; }
done
# the generated chunks: assembled in parallel (large)
pids=""
for t in $(seq 0 $((NM2-1))); do n=t_cases$t; ( cd asm && $VASM -quiet -Fbin -m68030 -m68882 -no-opt -I../obj -L ../obj/$n.lst -o ../obj/$n.bin ../obj/$n.s ) & pids="$pids $!"; done
for t in $(seq 0 $((NM3-1))); do n=t_m3$t; ( cd asm && $VASM -quiet -Fbin -m68030 -m68882 -no-opt -I../obj -L ../obj/$n.lst -o ../obj/$n.bin ../obj/$n.s ) & pids="$pids $!"; done
for p in $pids; do wait $p || { echo "RESULT: FAIL (assembling generated chunks)"; exit 1; }; done

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
