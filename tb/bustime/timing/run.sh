#!/bin/bash
# Instruction timing sequences (timing_body.i) on the core (tb_bustime, the
# Falcon bus model and governor) and, with HATARI and TOS set, in Hatari
# (hatari_golden.sh); prints the clocks of every test in both and the
# difference (also less the harness's own difference).
#   VASM=... [HATARI=<binary> TOS=<tos404.img>] run.sh
# Needs tb/bustime/run.sh to have built obj/vl/tb_bustime.
set -e
cd "$(dirname "$0")"
VASM=${VASM:-vasmm68k_mot}
O=../obj
"$VASM" -Fbin -m68030 -no-opt -quiet -o $O/t_timing_rom.bin t_timing_rom.s
"$VASM" -Ftos -m68030 -no-opt -quiet -o $O/TIMING.PRG t_timing_tos.s
python3 -I ../../system/rom2hex64.py $O/t_timing_rom.bin $O/t_timing_rom.hex
$O/vl/tb_bustime +rom=$O/t_timing_rom.hex > $O/timing_core.log 2>&1
grep -E "^(PASS|FAIL|governor)" $O/timing_core.log
if [ -n "${HATARI:-}" ] && [ -n "${TOS:-}" ]; then
	./hatari_golden.sh $O/TIMING.PRG > $O/timing_hatari.txt
	python3 -I table.py $O/timing_core.log $O/timing_hatari.txt
else
	python3 -I table.py $O/timing_core.log
fi
