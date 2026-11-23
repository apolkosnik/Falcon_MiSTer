#!/bin/sh
# Build falcon_net for the MiSTer (static ARM) in a Debian container with its cross
# compiler, and for this host (-m FILE testing).
set -e
cd "$(dirname "$0")"
podman run --rm --userns=keep-id -v "$PWD":/src:Z -u 0 docker.io/library/debian:bullseye sh -c \
	'apt-get -qq update >/dev/null && apt-get -qq install -y gcc-arm-linux-gnueabihf libc6-dev-armhf-cross >/dev/null &&
	 arm-linux-gnueabihf-gcc -O2 -Wall -Wextra -static -o /src/falcon_net /src/falcon_net.c'
${HOSTCC:-cc} -O2 -Wall -Wextra -o falcon_net_host falcon_net.c
echo "built falcon_net (ARM) and falcon_net_host"
