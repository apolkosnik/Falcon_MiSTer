#!/bin/bash
# Build CPUTIME.TOS (cputime.s: the CPU timing program for the board, docs/
# CPU_TIMING.md milestone 4) and CPUTIME.ST, a 720K floppy image with it
# (with mtools).  The reference spans come from ref.i (ref.sh).
#   VASM=/path/to/vasmm68k_mot (default vasmm68k_mot)
set -e
cd "$(dirname "$0")"
VASM=${VASM:-vasmm68k_mot}
mkdir -p build
$VASM -Ftos -m68030 -no-opt -quiet -I../../tb/bustime/timing -o build/CPUTIME.TOS cputime.s
if command -v mformat >/dev/null && command -v mcopy >/dev/null; then
	rm -f build/CPUTIME.ST
	mformat -i build/CPUTIME.ST -f 720 -C -v CPUTIME ::
	mcopy -i build/CPUTIME.ST build/CPUTIME.TOS ::
fi
ls -l build/CPUTIME.TOS build/CPUTIME.ST 2>/dev/null
