#!/bin/sh
# Build falcon_fpu for the MiSTer (static) and for this host (simulation, -m FILE),
# with the 68882 engine (engine/: Hatari's fpp.c + softfloat, built in place).
#   HATARI  Hatari source tree (default ~/Devel/Atari/hatari)
#   CC      ARM compiler (default arm-none-linux-gnueabihf-gcc, the Main_MiSTer
#           toolchain gcc-arm-10.2-2020.11)
set -e
cd "$(dirname "$0")"
engine/build.sh > /dev/null
${CC:-arm-none-linux-gnueabihf-gcc} -O2 -Wall -Wextra -static -o falcon_fpu falcon_fpu.c engine/build/arm/libfpe.a -lm
${HOSTCC:-cc} -O2 -Wall -Wextra -o falcon_fpu_host falcon_fpu.c engine/build/host/libfpe.a -lm
echo "built falcon_fpu (ARM) and falcon_fpu_host"
