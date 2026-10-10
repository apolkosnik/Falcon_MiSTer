#!/bin/sh
# Boot regressions on the current RTL against the stored reference frames.
cd "$(dirname "$0")"
OBJ=obj_full15 ./build.sh > full_build15.log 2>&1 || { echo "BUILD FAILED"; exit 1; }
rm -rf frames_g1 frames_g2 frames_g3 frames_g4 frames_g5
./obj_full15/Vtb_top +rom=etos512us.hex --turbo --ms 12000 --scsi0 media/scsi32.img --frames frames_g1 --frame-every 60 > g1.log 2>&1 &
./obj_full15/Vtb_top +rom=tos492.hex --turbo --ramtos --ms 32000 --frames frames_g2 --frame-every 60 > g2.log 2>&1 &
./obj_full15/Vtb_top +rom=rom/tc640_test.hex --turbo --ms 400 --frames frames_g3 --frame-every 5 > g3.log 2>&1 &
./obj_full15/Vtb_top +rom=tos404.hex --turbo --ms 18500 --frames frames_g4 --frame-every 60 > g4.log 2>&1 &
./obj_full15/Vtb_top +rom=etos512us.hex --turbo --ms 12000 --fda media/fd720.st --frames frames_g5 --frame-every 60 > g5.log 2>&1 &
wait
for f in 0300 0480 0600 0660; do cmp -s frames_scsi/frame_$f.ppm frames_g1/frame_$f.ppm && echo "EmuTOS frame $f identical" || echo "EmuTOS frame $f DIFFERS"; done
cmp -s frames_492c/frame_1860.ppm frames_g2/frame_1860.ppm && echo "TOS 4.92 desktop frame identical" || echo "TOS 4.92 desktop frame DIFFERS"
cmp -s frames_tc/frame_0020.ppm frames_g3/frame_0020.ppm && echo "TC640 ROM test frame identical" || echo "TC640 ROM test frame DIFFERS"
cmp -s frames_404/frame_0960.ppm frames_g4/frame_0960.ppm && echo "TOS 4.04 frame 0960 identical" || echo "TOS 4.04 frame 0960 DIFFERS"
ls frames_g5 | tail -1
