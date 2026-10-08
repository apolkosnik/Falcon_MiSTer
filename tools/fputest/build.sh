#!/bin/bash
# Build the hardware test programs of the FPU served by the ARM:
#   FPUTEST.TOS   self-checking: the generated sequence (gen_fputest.c, expected
#                 results from the same 68882 engine as falcon_fpu) x 20 passes,
#                 exception round trips
#   FPUBENCH.TOS  FPU instruction throughput / latency
#   FPUTEST.ST    both on a 720K floppy image (with mtools)
# Needs the engine's host library (tools/falcon_fpu/engine/build.sh) and vasm.
#   VASM=/path/to/vasmm68k_mot (default vasmm68k_mot)
set -e
cd "$(dirname "$0")"
ENG=../falcon_fpu/engine
VASM=${VASM:-vasmm68k_mot}
[ -f $ENG/build/host/libfpe.a ] || HOST_ONLY=1 $ENG/build.sh
mkdir -p build
cc -O2 -Wall -Wextra -I$ENG -o build/gen_fputest gen_fputest.c $ENG/build/host/libfpe.a -lm
build/gen_fputest > build/gen.s
$VASM -Ftos -m68030 -m68882 -no-opt -quiet -Ibuild -o build/FPUTEST.TOS fputest.s
$VASM -Ftos -m68030 -m68882 -no-opt -quiet -o build/FPUBENCH.TOS fpubench.s
# both on a 720K floppy image for the core's drive A (needs mtools)
if command -v mformat >/dev/null && command -v mcopy >/dev/null; then
	rm -f build/FPUTEST.ST
	mformat -i build/FPUTEST.ST -f 720 -C -v FPUTEST ::
	mcopy -i build/FPUTEST.ST build/FPUTEST.TOS build/FPUBENCH.TOS ::
fi
ls -l build/FPUTEST.TOS build/FPUBENCH.TOS build/FPUTEST.ST 2>/dev/null
