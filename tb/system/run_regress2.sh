#!/bin/sh
cd "$(dirname "$0")"
OBJ=obj_full12 ./build.sh > full_build12.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
rm -rf frames_r3 frames_r4
./obj_full12/Vtb_top +rom=etos512us.hex --ms 12000 --scsi0 media/scsi32.img --frames frames_r3 --frame-every 60 > r3.log 2>&1 &
./obj_full12/Vtb_top +rom=tos492.hex --ramtos --ms 32000 --frames frames_r4 --frame-every 60 > r4.log 2>&1 &
./obj_full12/Vtb_top +rom=rom/tc640_test.hex --ms 400 --frames frames_tc2 --frame-every 5 > tc2.log 2>&1 &
wait
for f in 0300 0480 0600 0660; do cmp -s frames_scsi/frame_$f.ppm frames_r3/frame_$f.ppm && echo "EmuTOS frame $f identical" || echo "EmuTOS frame $f DIFFERS"; done
cmp -s frames_492c/frame_1860.ppm frames_r4/frame_1860.ppm && echo "TOS 4.92 desktop frame identical" || echo "TOS 4.92 desktop frame DIFFERS"
cmp -s frames_tc/frame_0020.ppm frames_tc2/frame_0020.ppm && echo "TC640 ROM test frame identical" || echo "TC640 ROM test frame DIFFERS"
