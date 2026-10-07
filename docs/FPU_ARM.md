# MC68882 FPU served by the ARM - design

Status: proposal (branch `feature/fpu-arm`).  The core has no FPU today
(README: "the 68881 does not fit"; `falcon_cpubus` ends every coprocessor
cycle in a bus error, so the 68030 takes the F-line exception).  ~3,100 ALMs
are free, far too few for an FPU, but enough for a protocol bridge: the
68882's arithmetic runs on the HPS (ARM Cortex-A9, Linux) and the FPGA only
speaks the coprocessor interface.

## Facts this rests on

| Fact | Source |
|------|--------|
| Mailbox round trip FPGA -> DDR3 -> ARM -> DDR3 -> FPGA: median 0.59 us; pinned SCHED_FIFO max 7.5 us (CPU 1) | `tools/mbox_ping` README, measured 2026-10-07 |
| One FPGA DDR3 read through `falcon_memarb`: 0.32 us | same |
| AP68030 implements the whole MC68030 coprocessor protocol (all response primitives, cpSAVE/cpRESTORE, busy, take-exception) and has a scripted-coprocessor test (`t_cp`) | `rtl/AP68030/doc/ARCHITECTURE.md`, `core/ap030_exec_c.vh`, `tb/tb_cp_model.svh` |
| CIR cycle: FC=7, A19-16=0010, A15-13=CpID (1), A4-0=CIR; CPU space is never cached or posted | `ap030_exec_c.vh:8-34`, `ap030_memsys.v:420,577` |
| BERR is "no coprocessor" only on the first CIR access of an instruction; after the command write a BERR is a real bus error | `ap030_exec_a.vh:106`, `ap030_exec_c.vh:33` |
| Hook: `falcon_cpubus.sv:150` sends all non-IACK CPU space to `finish_berr`; the CPU and the bus bridge have no timeout for such a cycle | `falcon_cpubus.sv` |
| 68882 dialog: response word CA/PC/DR + primitive; reg-to-reg ops release the CPU at once (null, CA=0) and finish in the background; a later instruction gets "null, come again" ($8900) until the FPU is free; conditionals return null CA=0 with TF | MC68881/MC68882 UM ch. 7 (Tables 7-3..7-7) |
| FSAVE frames: null $00xx; 68882 idle $1F38 + 14 longs; busy $xxD4 | UM 6.4.2 |
| EmuTOS detects the FPU with FRESTORE(null), FNOP, FSAVE and reads the frame size ($38 = 68882) | EmuTOS `bios/processor.S` `_detect_fpu` |
| Hatari/WinUAE softfloat FPU back-end (`fpp_softfloat.c`, `softfloat/*`, FPSP-derived transcendentals) is plain portable C; `fpp.c` front-end is tied to the WinUAE CPU core | `hatari/src/cpu` |
| Main_MiSTer pins itself to CPU 1 (CPU 0 takes the IRQs); it is restarted on every core switch | `main.cpp:42-48`, `fpga_io.cpp:620` |

## Architecture

```
68030 (AP68030) --CIR cycles--> falcon_cpubus --cp_*--> falcon_fpu_bridge --d3 DMA--> falcon_memarb --> DDR3 mailbox
                                                         (68882 dialog,                                     ^
                                                          operand buffers,                                  | /dev/mem
                                                          FSAVE/FRESTORE)                       falcon_fpu (ARM, Linux):
                                                                                                softfloat 68882 engine
```

**Split: protocol in the FPGA, state and arithmetic on the ARM.**  The
dialog of an instruction (which primitive, how many operand longs, DR,
register-select mask) depends only on the command word and a few bits of FPU
state, so the bridge answers every CIR access locally, from registers, at bus
speed.  The ARM sees one request per FPU instruction (command word + operand
data + instruction address) and returns one reply (result data, FPSR
condition codes, exception request).  A round trip is paid only where the
68030 needs FPU data.

Per instruction class:

| Class | Bridge | ARM round trip |
|-------|--------|----------------|
| reg-to-reg, mem-to-reg arithmetic, FMOVE in, FMOVECR | collects the operand, posts the request, releases the CPU (null CA=0, as a real 68882 does) | in the background; the next FPU instruction gets come-again ($8900) until it is done |
| FMOVE out, FMOVEM out, FMOVE from FPCR/FPSR/FPIAR | come-again until the reply, then the data primitives | yes, CPU waits |
| FBcc/FScc/FDBcc/FTRAPcc | first version: come-again until the ARM answers TF; later: evaluate the predicate in the bridge from FPSR condition codes mirrored from each reply | first version yes, later no |
| FSAVE / FRESTORE, FNOP | format words in the bridge; idle frame body (14 longs) is opaque ARM state | FSAVE/FRESTORE of an idle frame: yes |
| enabled FPU exceptions | reply flags it; the next FPU instruction gets the pre-instruction exception primitive | no extra |

Waiting is always "null, come again, IA=1": the 68030 re-reads the response
CIR in short bus cycles, so interrupts are serviced and blitter/DMA keep the
bus between reads (no microsecond-long held cycles).

**Presence and failure.**  The ARM service writes a magic/heartbeat word into
the mailbox.  Without it (old Main, service not started) the bridge answers
the first CIR access with BERR: no FPU, as today, so TOS and programs behave
exactly as now.  If the service stops answering mid-instruction, a watchdog
(~100 ms) ends the instruction with a protocol-violation exception instead of
hanging the machine (the CPU has no timeout of its own).

**ARM side: `falcon_fpu`.**  A separate executable that Main_MiSTer starts
for the Falcon core (Main patch, next to `falcon_nvram_init`) and kills on a
core switch; a crash cannot take Main down.  It maps the mailbox through
`/dev/mem` like `mbox_ping`.  Engine: Hatari's softfloat back-end, unmodified
where possible, under a new front-end that takes a command word plus operand
bytes (replacing `fpp.c`'s EA/operand layer, which assumes it owns the CPU).
Basic operations are exact 80-bit; transcendentals follow Motorola's FPSP
(as in Hatari/WinUAE/Previous), not bit-exact to 68882 microcode.

**CPU placement.**  Main owns CPU 1, so the service runs on CPU 0 (with the
IRQ handlers).  Policy: SCHED_FIFO, spin while requests keep arriving, fall
back to sleeping polls after ~1 ms idle so an idle FPU does not take a core
(Linux's RT throttling caps it at 95% anyway).  To be confirmed by
`mbox_ping -c 0 [-f]` measurements.

**Mailbox.**  Guest $E80000 region (DDR3 0x30E80000), as the probe: request
and reply slots with sequence numbers, one outstanding request at first.  The
bridge takes `falcon_memarb`'s d3 port (between the blitter and the CPU),
which the probe already proved.

## Verification plan

- Bridge unit: the AP68030 scripted-coprocessor style (`tb_cp_model.svh`),
  re-pointed at the real bridge, plus a C++ ARM model.
- Co-simulation like `tb/mbox`: real AP68030 RTL running 68k test programs
  (every class above, all operand formats, FSAVE/FRESTORE, exceptions,
  interrupts during come-again) + real bridge + the real `falcon_fpu` built
  for the host on a shared-file mailbox.
- Golden results: Hatari's own `fpp.c` (stubbed CPU accessors) executing the
  same instructions; every FP register, FPSR and memory result must match.
- System: EmuTOS/TOS 4.04 must report a 68882 (_FPU cookie $00060000);
  then FPU software on hardware.

## Milestones

1. Presence and frames: bridge skeleton in `falcon_cpubus`, heartbeat,
   FNOP, FSAVE/FRESTORE (null/idle), detection by EmuTOS/TOS; ARM stub.
2. Data: FMOVE in/out (all formats), FMOVEM data/control, FMOVECR.
3. Arithmetic: all 6888x opmodes through softfloat; FPSR/FPCR; conditionals.
4. Exceptions, background execution, condition-code mirroring in the bridge.
5. Main integration (start/stop the service), CPU-0 policy, hardware tests.

## Open decisions

1. Service process: separate executable started by Main (proposed) or a thread inside Main.
2. CPU 0 spin-then-sleep policy (pending the CPU 0 measurements).
3. Emulate a 68882 (proposed; the Falcon's FPU option) rather than a 68881.
4. Accept FPSP-accurate (not microcode-exact) transcendentals.
