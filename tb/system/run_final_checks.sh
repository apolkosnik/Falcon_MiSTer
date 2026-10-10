#!/bin/sh
cd "$(dirname "$0")"
OBJ=obj_full11 ./build.sh > full_build11.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
rm -rf frames_beers3 frames_r1 frames_r2
./obj_full11/Vtb_top +rom=tos404.hex --turbo --ms 75000 --ide0 media/beers32.img --ram 14 --monitor 2 --frames frames_beers3 --frame-every 30 > beers3_run.log 2>&1 &
./obj_full11/Vtb_top +rom=etos512us.hex --turbo --ms 12000 --scsi0 media/scsi32.img --frames frames_r1 --frame-every 60 > r1.log 2>&1 &
./obj_full11/Vtb_top +rom=tos492.hex --turbo --ramtos --ms 32000 --frames frames_r2 --frame-every 60 > r2.log 2>&1 &
wait
for f in 0300 0480 0600 0660; do cmp -s frames_scsi/frame_$f.ppm frames_r1/frame_$f.ppm && echo "EmuTOS frame $f identical" || echo "EmuTOS frame $f DIFFERS"; done
cmp -s frames_492c/frame_1860.ppm frames_r2/frame_1860.ppm && echo "TOS 4.92 desktop frame identical" || echo "TOS 4.92 desktop frame DIFFERS"
echo "demo frames: $(ls frames_beers3 | wc -l)"
