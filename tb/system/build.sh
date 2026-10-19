#!/bin/sh
# Build the full-system simulation of the real RTL.
#   BRINGUP="NO_IDE NO_FDC NO_DSP NO_FPU" ./build.sh   leaves those devices out
#   BRINGUP="MBOX_TEST" ./build.sh                     mailbox latency probe on d3
#   BRINGUP="CPU060 NO_DSP NO_FPU" OBJ=obj_060 ./build.sh   the ap060 variant (AP68060)
#   BRINGUP="CPU040 NO_DSP NO_FPU" OBJ=obj_040 ./build.sh   the ap040 variant (AP68040)
#   RTL=/abs/path/rtl OBJ=obj_x ./build.sh             builds another copy of rtl/
set -e
cd "$(dirname "$0")"
R=${RTL:-../../rtl}
DEFS=""
SRCS="$R/falcon/falcon_system.sv $R/falcon/falcon_memarb.sv $R/falcon/falcon_combel.sv
 $R/falcon/falcon_videl.sv $R/falcon/falcon_psg.sv $R/falcon/falcon_mfp.sv $R/falcon/falcon_mfp_timer.sv $R/falcon/falcon_mfp_usart.sv
 $R/falcon/falcon_acia.sv $R/falcon/falcon_ikbd.sv $R/falcon/falcon_ikbd_keymap.sv $R/falcon/falcon_nvram.sv $R/falcon/falcon_mbox_test.sv $R/falcon/falcon_fpu_bridge.sv
 $R/falcon/falcon_blitter.sv $R/falcon/falcon_crossbar.sv"
# the processor: the AP68030 with its bus bridge and Falcon timing, or the AP68060 (CPU060)
A60=$R/AP68060/rtl
A40=$R/AP68040/rtl
case " $BRINGUP " in
*" CPU040 "*) DEFS="$DEFS +define+FALCON_CPU040 +define+FALCON_CPU060"; CDEFS="-DFALCON_CPU060"; CPUINC="-I$A40 -I$A40/fpu -I$A40/gen -I$A60/fastram"
	CPUSRC="$A40/ap68040_pkg.sv $A40/gen/ap68040_upkg.sv $A40/ap68040_alu.sv $A40/ap68040_alu_slow.sv $A40/ap68040_muldiv.sv $A40/ap68040_biu.sv
	 $A40/ap68040_ram.sv $A40/ap68040_atc.sv $A40/ap68040_dmu.sv $A40/ap68040_predec.sv $A40/ap68040_fetch.sv $A40/ap68040_decode.sv
	 $A40/ap68040_useq.sv $A40/fpu/ap040_fpu.v $A40/ap68040_fpif.sv $A40/ap68040_backend.sv $A40/ap68040_snoop.sv $A40/ap68040.sv
	 $A60/fastram/ap030_async_fifo.v $A60/fastram/ap030_fastram_fe.v $A60/fastram/ap030_fastram_be.v
	 $A60/fastram/ap030_l2ram.sv $A60/fastram/ap060_fastram_bus.sv $A60/fastram/ap060_fastram.sv
	 $R/falcon/falcon_cpubus040.sv $R/falcon/falcon_ddr_arb.sv" ;;
*" CPU060 "*) DEFS="$DEFS +define+FALCON_CPU060"; CDEFS="-DFALCON_CPU060"; CPUINC="-I$A60 -I$A60/fpu -I$A60/gen -I$A60/fastram"
	CPUSRC="$A60/ap68060_pkg.sv $A60/gen/ap68060_upkg.sv $A60/ap68060_alu.sv $A60/ap68060_alu_slow.sv $A60/ap68060_muldiv.sv $A60/ap68060_biu.sv
	 $A60/ap68060_ram.sv $A60/ap68060_atc.sv $A60/ap68060_dmu.sv $A60/ap68060_predec.sv $A60/ap68060_fetch.sv $A60/ap68060_decode.sv
	 $A60/ap68060_useq.sv $A60/fpu/ap040_fpu.v $A60/ap68060_fpif.sv $A60/ap68060_backend.sv $A60/ap68060_snoop.sv $A60/ap68060.sv
	 $A60/fastram/ap030_async_fifo.v $A60/fastram/ap030_fastram_fe.v $A60/fastram/ap030_fastram_be.v
	 $A60/fastram/ap030_l2ram.sv $A60/fastram/ap060_fastram_bus.sv $A60/fastram/ap060_fastram.sv
	 $R/falcon/falcon_cpubus060.sv $R/falcon/falcon_ddr_arb.sv" ;;
*) CPUINC="-I$R/AP68030/rtl -I$R/AP68030/rtl/core"
	CPUSRC="$R/AP68030/rtl/*.v $R/falcon/falcon_cpubus.sv $R/falcon/falcon_cpuclk.sv $R/falcon/falcon_pipescan.sv $R/falcon/falcon_tstat.sv $R/falcon/falcon_l2.sv" ;;
esac
case " $BRINGUP " in *" NO_IDE "*) DEFS="$DEFS +define+FALCON_NO_IDE" ;; *) SRCS="$SRCS $(ls $R/falcon/falcon_ide*.sv)" ;; esac
case " $BRINGUP " in *" NO_FDC "*) DEFS="$DEFS +define+FALCON_NO_FDC" ;; *) SRCS="$SRCS $(ls $R/falcon/falcon_fdc*.sv)" ;; esac
case " $BRINGUP " in *" MBOX_TEST "*) DEFS="$DEFS +define+FALCON_MBOX_TEST" ;; esac
case " $BRINGUP " in *" NO_FPU "*) DEFS="$DEFS +define+FALCON_NO_FPU" ;; esac
case " $BRINGUP " in *" NO_DSP "*) DEFS="$DEFS +define+FALCON_NO_DSP" ;; *) SRCS="$SRCS $(ls $R/falcon/dsp/*.sv)" ;; esac
# the 68882 engine for --fpu (Hatari's fpp.c, tools/falcon_fpu/engine)
FPE=../../tools/falcon_fpu/engine
HOST_ONLY=1 $FPE/build.sh > /dev/null
# the .mem tables (PSG volumes, opcode table) are found relative to the project root, as in Quartus
[ -n "$RTL" ] || { mkdir -p rtl/falcon && ln -sf ../../$R/falcon/falcon_psg_vol.mem rtl/falcon/falcon_psg_vol.mem; ln -sf ../../$R/falcon/falcon_optbl.mem rtl/falcon/falcon_optbl.mem; }
verilator --cc --exe --build -j 16 -O3 --x-assign fast --x-initial fast \
	-Wno-fatal -Wno-WIDTH -Wno-CASEINCOMPLETE -Wno-PINMISSING -Wno-TIMESCALEMOD -Wno-MULTIDRIVEN -Wno-UNOPTFLAT \
	--top-module tb_top -Mdir ${OBJ:-obj_dir} $DEFS \
	$CPUINC -I$R/falcon -I$R/falcon/dsp \
	-CFLAGS "-I$PWD/$FPE -I$PWD/$FPE/.. $CDEFS" -LDFLAGS "$PWD/$FPE/build/host/libfpe.a -lm" \
	tb_top.sv ddr3_model.sv $CPUSRC $SRCS sim_main.cpp
