#!/bin/bash
# Serial ADC -> signed stereo -> real Falcon crossbar/DMA/DSP/output.
set -euo pipefail
cd "$(dirname "$0")/../.."
O=${AUDIO_IN_OBJ:-tb/audio_in/obj_dir}
mkdir -p "$O"
verilator --binary --timing --build -j 4 -O2 -Wno-fatal -Wno-TIMESCALEMOD \
    --top-module tb_audio_in --Mdir "$O" \
    sys/ltc2308.sv rtl/falcon/falcon_audio_adc.sv rtl/falcon/falcon_crossbar.sv \
    tb/audio_in/tb_audio_in.sv > "$O/build.log" 2>&1 || {
    cat "$O/build.log"; echo "RESULT: FAIL (build)"; exit 1;
}
"$O/Vtb_audio_in" | tee "$O/run.log"
grep -q 'RESULT: PASS' "$O/run.log"
