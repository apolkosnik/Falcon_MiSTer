# MC68882 FPU served by the ARM - design

Status (branch `feature/fpu-arm`): milestones 1-5 done; on the DE10-Nano
(2026-10-08) FPUTEST.TOS passes and FPUBENCH.TOS runs with the service started
by Main.  Presence, frames and detection; full instruction dialogs in
`rtl/falcon/falcon_fpu_bridge.sv`, requests executed by `tools/falcon_fpu` with
Hatari's `fpp.c` (`tools/falcon_fpu/engine`), verified by `tb/fpu` against Hatari
driven with real EAs.  Milestone 4: enabled exceptions (pending, pre- and
mid-instruction), FPIAR through the PC primitive, FSAVE/FRESTORE frames
owned by the ARM.  Milestone 2 needed two
AP68030 coprocessor fixes (immediate operands in the "memory" EA category, cpScc
byte size), committed in the submodule on branch `fix/coprocessor-imm-cpscc`;
milestone 5 a third: an interrupt taken while an FPU instruction waits on
come-again (null, CA, IA) must stack a coprocessor mid-instruction frame
(format $9) whose RTE resumes the dialogue (UM 10.4.8); the core stacked a
normal frame past the instruction, so its remaining transfers were lost
(found by FPUTEST in the system simulation with VBL interrupts).  The core has no FPU today
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
| Hatari/WinUAE (tested on real chips) write the 68882 null frame as $00380000 and keep an exception pending when the CPU takes it (68882 only) until FSAVE | `fpp.c` `fpuop_save`, `fp_exception_pending` |
| FMOVEM and the control register moves do not report a pending exception (Hatari does: hidden by the shim); FSAVE with one pending clears BIU bit 27 (Hatari loses it: fixed in the shim) | UM 6.4.2.2, `engine/fpe_shim.c` |
| The 68030 stores an FSAVE body to descending addresses and sends an FRESTORE body ascending | MC68030 UM 10.2.3.3/4, AP68030 `S_CPSV_BODY`/`S_CPRS_BODY` |
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
| FSAVE / FRESTORE | the frame is the ARM's (Hatari `fpuop_save`/`fpuop_restore`): FSAVE reads come-again until the save reply, then the format word and the body; FRESTORE formats are checked in the bridge, the body goes to the ARM | yes (FRESTORE: in the background) |
| FNOP and other conditionals | evaluated in the bridge; the first one after a reset or a null frame goes to the ARM (null -> idle) | only that one |
| enabled FPU exceptions | the reply flags EXC PEND and the vector; the next opclass 000/010/011 instruction or conditional gets take pre-instruction exception ($1Cvv); FMOVE out raising one ends with $1Dvv; while one is enabled the first primitive asks for the PC (FPIAR) | no extra |

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

**Mailbox.**  Guest $E90000 (DDR3 0x30E90000), next to the probe's $E80000
so the two never mix; 16-bit words, Falcon byte order.  +$000 MAGIC $4650
("FP", written by the service, cleared when it stops), +$002 HEARTBEAT
(incremented every 10 ms), +$004 VERSION (4).  One request at a time:
+$100 RSEQ (written last), KIND (1 execute, 2 reset, 3 condition, 4 save,
5 restore), CMD, AUX (Dn of a dynamic list or k-factor; the restore format
word), NBYTES, IADDR (+$10A, when the PC was asked for), operand bytes at
+$110 (the restore body).  The bridge writes a header field only when the
kind uses it and it differs from what the mailbox holds, so a run of like
instructions posts RSEQ alone.  The reply at +$200: STATUS (written last;
the only word the bridge reads: RSEQ[3:0], the FPSR condition codes, the
exception vector - 48 and the flags not implemented, exception pending,
condition true, exception enabled, raised by this request), then FLAGS,
FPSR, FPCR[15:0], NBYTES (informational), result bytes at +$210 (the save
frame).  The service reads and writes it in 32-bit words (the mapping is
uncached).  `tools/falcon_fpu/fpu_request.h` executes a request for the
service and for the system simulation alike.  Operand and result bytes are copied in
memory order between the operand CIR and the mailbox, so the bridge needs no
buffer.

**Engine.**  `tools/falcon_fpu/engine` builds Hatari's `fpp.c`, `fpp_softfloat.c`
and softfloat unmodified with a shim (`libfpe`, host and ARM).  `fpp.c` is
given a synthetic `(A0)` opcode (`-(A0)` for predecrement FMOVEM lists) and
its memory accessors map onto the operand/result buffer, so the buffer is
the memory image the 68030 transfers and every 68882 detail of `fpp.c`
applies; `engine/fpe_selftest` compares this against `fpp.c` driven with
real EAs (16,930 checks, host and qemu-arm).
The bridge polls MAGIC/HEARTBEAT every 5 ms and reports an FPU while the
heartbeat has moved within 100 ms.  It owns `falcon_memarb`'s d3 port
(between the blitter and the CPU), which the probe proved; a measurement
build (`FALCON_BRINGUP="MBOX_TEST"`) gives d3 to the probe and has no FPU,
and `FALCON_BRINGUP="NO_FPU"` leaves the bridge out (coprocessor cycles end
in BERR as before).

## Running it (milestone 5)

- **Start/stop** (`main_patch/0003`, Main_MiSTer `support/falcon/falcon_fpu.cpp`):
  at core start, before the reset is released, Main runs `falcon_fpu -c 0 -f -d`
  from the directory of its own binary (output to `/tmp/falcon_fpu.log`);
  `fpga_load_rbf` stops it (SIGTERM, wait, SIGKILL after 200 ms) before the
  FPGA is loaded with another core, since its heartbeat writes would land in
  the next core's DDR3.  `-d` (PR_SET_PDEATHSIG) ends it when Main exits for
  any other reason (Main restarts itself on every core load).  Only one
  instance serves the mailbox (`flock` on `/tmp/falcon_fpu.lock`).
- **CPU policy**: CPU 0 (Main owns CPU 1), SCHED_FIFO; spin while requests
  keep coming, poll every 50 us after 1 ms without one.  Under SCHED_FIFO a
  busy spin is broken every 1.8 ms by a 200 us sleep, so ~10% of CPU 0 stays
  with Linux, under the RT throttling limit (95%) that would otherwise stop
  the service for 50 ms at a time under a long FPU-bound run.
- **Hardware tests** (`tools/fputest`, `build.sh`): FPUTEST.TOS runs a
  generated sequence (every opmode, operand format, rounding mode, FMOVECR
  constant, predicate, FMOVEM; 3,850 results) 20 times and compares it with
  the results of the same engine on the host, so a difference is a fault of
  the FPGA/DDR3/ARM path; then an enabled DZ (pre-instruction, FPIAR,
  FSAVE/FRESTORE with BIU bit 27) and OPERR (FMOVE out, mid-instruction)
  exception round trip.  FPUBENCH.TOS times reg-reg, transcendental,
  move in/out, conditional and memory-operand instructions.  `sim.sh` runs
  either in the system simulation on a stand-in ROM (`simrom.s`: the few
  GEMDOS/BIOS/XBIOS calls they make, VBL interrupts) instead of TOS.

### Measured speed

Mailbox VERSION 3 (milestone 5 as committed): DE10-Nano, 2026-10-08,
`releases/Falcon_20261008.rbf` with the service started by Main (CPU 0,
SCHED_FIFO), FPUBENCH.TOS, 100,000 instructions each:

| Instruction | us each | per second |
|-------------|---------|------------|
| FNOP | 0.95 | 1,052,631 |
| FSGT D0 | 0.95 | 1,052,631 |
| FADD FP1,FP0 | 9.65 | 103,626 |
| FMUL FP1,FP0 | 9.50 | 105,263 |
| FDIV FP1,FP0 | 9.45 | 105,820 |
| FSQRT FP1,FP0 | 12.25 | 81,632 |
| FSIN FP1,FP0 | 14.40 | 69,444 |
| FETOX FP1,FP0 | 14.25 | 70,175 |
| FMOVE.L D7,FP0 | 11.65 | 85,836 |
| FMOVE.L FP1,D0 | 12.90 | 77,519 |
| FMOVE.X FP0,(A4) | 18.45 | 54,200 |
| FMOVE.D (A4),FP0 + FMUL.D (A4),FP0 | 13.45 | 74,349 |

FNOP and FSGT are answered by the bridge alone (the predicate from the
mirrored condition codes): ~1 us is the coprocessor dialogue itself.
Every other row was one mailbox round trip per instruction with a fixed
cost of about 9 us, against 0.59 us for the bare mailbox round trip
(`tools/mbox_ping`): the request/reply framing.

Mailbox VERSION 4 (a header field written only when it changes, one
STATUS word as the reply, the service polling without system calls and
accessing the mailbox in 32-bit words), same board, same day:

| Instruction | VERSION 3 us | VERSION 4 us | per second |
|-------------|--------------|--------------|------------|
| FNOP | 0.95 | 1.00 | 1,000,000 |
| FSGT D0 | 0.95 | 0.95 | 1,052,631 |
| FADD FP1,FP0 | 9.65 | 5.00 | 200,000 |
| FMUL FP1,FP0 | 9.50 | 4.90 | 204,081 |
| FDIV FP1,FP0 | 9.45 | 4.90 | 204,081 |
| FSQRT FP1,FP0 | 12.25 | 7.75 | 129,032 |
| FSIN FP1,FP0 | 14.40 | 10.40 | 96,153 |
| FETOX FP1,FP0 | 14.25 | 10.15 | 98,522 |
| FMOVE.L D7,FP0 | 11.65 | 6.80 | 147,058 |
| FMOVE.L FP1,D0 | 12.90 | 8.15 | 122,699 |
| FMOVE.X FP0,(A4) | 18.45 | 12.95 | 77,220 |
| FMOVE.D (A4),FP0 + FMUL.D (A4),FP0 | 13.45 | 8.70 | 114,942 |

`falcon_fpu -s` during the run (A9 global timer): a mailbox read costs
128 ns, a write 71 ns; while the reg-reg rows ran, a request spent 3.05 us
in the service (2.16 us in the engine, 0.89 us mailbox and the rest) and
2.23 us between its reply and the next request (the bridge noticing the
STATUS, the 68030's next instruction, the bridge posting it, the service
noticing RSEQ).  What is left, largest first:
- the engine: FADD, FMUL and FDIV cost the same, so the time is
  fpp.c's per-instruction work (decode, operand fetch through the shim,
  status, exception checks), not the arithmetic;
- the FPGA side: one STATUS read and one RSEQ write per request at about
  0.32 us each through `falcon_memarb`, plus the 68030's dialogue;
- the service's mailbox work: the IADDR read is needed only with an
  enabled exception, FPSR/FPCR/NBYTES are informational, and the barrier
  before STATUS is needed only after result bytes;
- the FIFO duty cap: ~10% under a continuous FPU-bound run.

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
4. Exceptions (pending, pre/mid-instruction, BSUN with PC), FPIAR, FSAVE/FRESTORE
   frames from the ARM.  (Background execution and condition-code mirroring
   came with milestones 2 and 3.)
5. Main integration (start/stop the service), CPU-0 policy, hardware tests
   (see "Running it").

## Open decisions

1. Service process: separate executable started by Main (proposed) or a thread inside Main.
2. CPU 0 spin-then-sleep policy (pending the CPU 0 measurements).
3. Emulate a 68882 (proposed; the Falcon's FPU option) rather than a 68881.
4. Accept FPSP-accurate (not microcode-exact) transcendentals.
