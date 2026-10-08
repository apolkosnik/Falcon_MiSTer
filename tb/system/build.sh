#!/bin/sh
# Build the full-system simulation of the real RTL.
#   BRINGUP="NO_IDE NO_FDC NO_DSP NO_FPU" ./build.sh   leaves those devices out
#   BRINGUP="MBOX_TEST" ./build.sh                     mailbox latency probe on d3
set -e
cd "$(dirname "$0")"
R=../../rtl
DEFS=""
SRCS="$R/falcon/falcon_system.sv $R/falcon/falcon_cpubus.sv $R/falcon/falcon_memarb.sv $R/falcon/falcon_combel.sv
 $R/falcon/falcon_videl.sv $R/falcon/falcon_psg.sv $R/falcon/falcon_mfp.sv $R/falcon/falcon_mfp_timer.sv $R/falcon/falcon_mfp_usart.sv
 $R/falcon/falcon_acia.sv $R/falcon/falcon_ikbd.sv $R/falcon/falcon_ikbd_keymap.sv $R/falcon/falcon_nvram.sv $R/falcon/falcon_mbox_test.sv $R/falcon/falcon_fpu_bridge.sv
 $R/falcon/falcon_blitter.sv $R/falcon/falcon_crossbar.sv"
case " $BRINGUP " in *" NO_IDE "*) DEFS="$DEFS +define+FALCON_NO_IDE" ;; *) SRCS="$SRCS $(ls $R/falcon/falcon_ide*.sv)" ;; esac
case " $BRINGUP " in *" NO_FDC "*) DEFS="$DEFS +define+FALCON_NO_FDC" ;; *) SRCS="$SRCS $(ls $R/falcon/falcon_fdc*.sv)" ;; esac
case " $BRINGUP " in *" MBOX_TEST "*) DEFS="$DEFS +define+FALCON_MBOX_TEST" ;; esac
case " $BRINGUP " in *" NO_FPU "*) DEFS="$DEFS +define+FALCON_NO_FPU" ;; esac
case " $BRINGUP " in *" NO_DSP "*) DEFS="$DEFS +define+FALCON_NO_DSP" ;; *) SRCS="$SRCS $(ls $R/falcon/dsp/*.sv)" ;; esac
# the 68882 engine for --fpu (Hatari's fpp.c, tools/falcon_fpu/engine)
FPE=../../tools/falcon_fpu/engine
HOST_ONLY=1 $FPE/build.sh > /dev/null
# the PSG volume table is found relative to the project root, as in Quartus
mkdir -p rtl/falcon && ln -sf ../../$R/falcon/falcon_psg_vol.mem rtl/falcon/falcon_psg_vol.mem
verilator --cc --exe --build -j 16 -O3 --x-assign fast --x-initial fast \
	-Wno-fatal -Wno-WIDTH -Wno-CASEINCOMPLETE -Wno-PINMISSING -Wno-TIMESCALEMOD -Wno-MULTIDRIVEN -Wno-UNOPTFLAT \
	--top-module tb_top -Mdir ${OBJ:-obj_dir} $DEFS \
	-I$R/AP68030/rtl -I$R/AP68030/rtl/core -I$R/falcon -I$R/falcon/dsp \
	-CFLAGS "-I$PWD/$FPE -I$PWD/$FPE/.." -LDFLAGS "$PWD/$FPE/build/host/libfpe.a -lm" \
	tb_top.sv ddr3_model.sv $R/AP68030/rtl/*.v $SRCS sim_main.cpp
