#!/bin/sh
# Build falcon_fpu for the MiSTer (static) and for this host (simulation, -m FILE).
#   CC   ARM compiler (default arm-none-linux-gnueabihf-gcc, the Main_MiSTer
#        toolchain gcc-arm-10.2-2020.11)
set -e
cd "$(dirname "$0")"
${CC:-arm-none-linux-gnueabihf-gcc} -O2 -Wall -Wextra -static -o falcon_fpu falcon_fpu.c
${HOSTCC:-cc} -O2 -Wall -Wextra -o falcon_fpu_host falcon_fpu.c
echo "built falcon_fpu (ARM) and falcon_fpu_host"
