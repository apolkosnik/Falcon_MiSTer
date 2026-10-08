#!/bin/sh
# Floppy boot, pre-audit RTL snapshot (A) against the current RTL (B)
cd "$(dirname "$0")"
RTL=/tmp/falcon-hatari-audit-20261003/rtl OBJ=obj_preaudit ./build.sh > build_preaudit.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
rm -rf frames_fdA frames_fdB
./obj_preaudit/Vtb_top +rom=etos512us.hex --ms 20000 --fda media/fd720.st --frames frames_fdA --frame-every 60 > fdA.log 2>&1 &
./obj_full14/Vtb_top +rom=etos512us.hex --ms 20000 --fda media/fd720.st --frames frames_fdB --frame-every 60 > fdB.log 2>&1 &
wait
for f in $(ls frames_fdA); do cmp -s frames_fdA/$f frames_fdB/$f && echo "$f same" || echo "$f DIFF"; done
