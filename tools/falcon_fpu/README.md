# falcon_fpu: the HPS side of the Falcon core's MC68882

The FPGA (`rtl/falcon/falcon_fpu_bridge.sv`) answers the 68030's coprocessor
interface; the FPU's registers and arithmetic run here, on the MiSTer's ARM,
through a DDR3 mailbox at 0x30E90000.  Design and milestones:
`docs/FPU_ARM.md`.

The service writes MAGIC, VERSION 2 and a heartbeat every 10 ms (while the
heartbeat moves the core reports a 68882) and executes every request posted
by the bridge with the engine in `engine/` (Hatari's `fpp.c` + softfloat,
built from the Hatari tree: `HATARI`, default `~/Devel/Atari/hatari`).  It
spins for 1 ms after a request, then polls every 50 us.

```
tools/falcon_fpu/build.sh                 # needs arm-none-linux-gnueabihf-gcc and Hatari's source
/media/fat/falcon_fpu -c 0 &              # on the MiSTer, before or after loading the core
```

`-c 0` keeps it off CPU 1, where Main_MiSTer runs.  `-m FILE` maps a file
instead of `/dev/mem` (simulation).  Main_MiSTer will start and stop it with
the core in a later milestone.
