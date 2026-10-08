#!/bin/sh
# Build mbox_ping for the MiSTer (static, so it does not depend on the HPS
# Linux C library) and for this host (simulation, -m FILE).
#   CC   ARM compiler (default arm-none-linux-gnueabihf-gcc, the Main_MiSTer
#        toolchain gcc-arm-10.2-2020.11)
set -e
cd "$(dirname "$0")"
${CC:-arm-none-linux-gnueabihf-gcc} -O2 -Wall -Wextra -static -o mbox_ping mbox_ping.c
${HOSTCC:-cc} -O2 -Wall -Wextra -o mbox_ping_host mbox_ping.c
echo "built mbox_ping (ARM) and mbox_ping_host"
