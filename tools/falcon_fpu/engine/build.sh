#!/bin/bash
# Build libfpe.a (MC68882 engine = Hatari fpp.c + softfloat + shim) for the host and for ARM, the host
# golden self-test and an ARM smoke binary.
#   HATARI   Hatari source tree (default ~/Devel/Atari/hatari)
#   CC       ARM compiler (default arm-none-linux-gnueabihf-gcc, the Main_MiSTer toolchain)
#   HOSTCC   host compiler (default cc)
#   OUT      output directory (default ./build next to this script)
#   HOST_ONLY=1  only the host library (no ARM toolchain needed; the simulations)
# Hatari's files are compiled in place and unmodified (fpp.c through fpe_fpp.c, which includes it and adds
# the register-to-register fast path).  shim/config.h is an empty stand-in for
# Hatari's CMake config.h: no SDL, no config, so the same sources build for the ARM target.
set -e
cd "$(dirname "$0")"
HATARI="${HATARI:-$HOME/Devel/Atari/hatari}"
H="$HATARI/src"
OUT="${OUT:-$PWD/build}"
CC="${CC:-arm-none-linux-gnueabihf-gcc}"
HOSTCC="${HOSTCC:-cc}"
AR_ARM="${AR_ARM:-${CC%gcc}ar}"
INC="-I. -Ishim -I$H/cpu -I$H/cpu/softfloat -I$H/includes -I$H/debug"
SRC="fpe_fpp.c $H/cpu/fpp_softfloat.c $H/cpu/softfloat/softfloat.c $H/cpu/softfloat/softfloat_fpsp.c $H/cpu/softfloat/softfloat_decimal.c"
# -fwrapv: Hatari builds with it.  -w: Hatari's code is warning-noisy; the shim itself is built with -Wall.
HF="-std=gnu99 -O2 -w -fwrapv"
mkdir -p "$OUT/host" "$OUT/arm"

build_lib() {  # $1 compiler  $2 ar  $3 dir  $4 extra flags
    local cc="$1" ar="$2" d="$3"; shift 3
    local objs=""
    for f in $SRC; do
        o="$d/$(basename "$f" .c).o"; $cc $HF "$@" $INC -c "$f" -o "$o"; objs="$objs $o"
    done
    $cc -std=gnu99 -O2 -Wall -Wextra -Wno-unused-parameter -fwrapv "$@" $INC -c fpe_shim.c -o "$d/fpe_shim.o"
    rm -f "$d/libfpe.a"; $ar rcs "$d/libfpe.a" $objs "$d/fpe_shim.o"
}

build_lib "$HOSTCC" ar "$OUT/host"
if [ -n "${HOST_ONLY:-}" ]; then echo "built: $OUT/host/libfpe.a"; exit 0; fi
build_lib "$CC" "$AR_ARM" "$OUT/arm" -static
$HOSTCC -std=gnu99 -O2 -w -fwrapv $INC fpe_selftest.c "$OUT/host/libfpe.a" -lm -o "$OUT/fpe_selftest"
$HOSTCC -std=gnu99 -O2 -Wall -I. fpe_smoke.c "$OUT/host/libfpe.a" -lm -o "$OUT/fpe_smoke_host"
$CC -std=gnu99 -O2 -Wall -static -I. fpe_smoke.c "$OUT/arm/libfpe.a" -lm -o "$OUT/fpe_smoke_arm"
ls -l "$OUT/host/libfpe.a" "$OUT/arm/libfpe.a" "$OUT/fpe_selftest" "$OUT/fpe_smoke_arm"
echo "built: $OUT/{host,arm}/libfpe.a  fpe_selftest (host golden)  fpe_smoke_arm"
