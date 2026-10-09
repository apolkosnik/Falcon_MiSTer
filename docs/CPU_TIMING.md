# A 16 MHz, cycle-accurate 68030 - design

Status (branch `feature/cpu-16mhz`): decisions taken (below); milestones 1
(the CPU clock: 16/8 MHz or 32 MHz turbo) and 2 (the Falcon bus: Hatari's
cycle lengths on DDR3) done, see the sections at the end; milestones 3-4
open.  Until milestone 1 the core ran the AP68030 on every
clock of the 32 MHz system clock and ended every bus cycle as soon as memory
or the device answered; a Falcon030 runs it at 16 MHz (8 MHz selectable) on
a 16-bit bus.  This document says what "cycle-accurate" can mean for this
core, how to get there, and what was decided.

## Facts this rests on

| Fact | Source |
|------|--------|
| Falcon CPU clock 16.042 MHz (32.084988 MHz / 2); $FF8007 bit 0 switches 8 / 16 MHz (68030 only), a read returns the current setting; bit 5 (bus mode) has no timing effect | Hatari clocks_timings.c:157,325-342; ioMemTabFalcon.c:103-155 |
| The CPU's data bus to ST-RAM is 16 bits; Videl is the DRAM controller with its own 32-bit burst path; video modes cost the CPU only a few percent | Atari Falcon references (Wikipedia, DFB wiki, Robbins 1993); forum reports (unverified) |
| Hatari's Falcon CPU model ("cycle exact", the default): ST-RAM is CHIP16 - an access at clock position 2 or 3 (mod 4) first waits 2 cycles, then takes 3; a long is two 16-bit accesses (8 cycles back to back); ROM, cartridge, I/O are FAST16, 3 cycles per byte/word, 6 per long; device waits on top (MFP, YM, FDC +4, ACIA +6 and E-clock) | Hatari custom.c:329-426; newcpu.c:9664-9900; memory.c:1640,1778-1808; mfp.c, psg.c, acia.c |
| Hatari charges 2 internal cycles per instruction word consumed (overlapping the previous bus access), no bus time for I/D-cache hits, MULU/MULS.W 20, DIVU.W 34, DIVS.W 48, interrupt acknowledge 12 (MFP) / 10 (video) + 4; the MC68030 UM head/tail/cache-case timings are only comments in its generated code | Hatari newcpu.c:10627, cpu_prefetch.h:54-80, gencpu.c:605-670,8286-8399, newcpu.c:2905-3027 |
| Hatari models no Videl, DMA sound, disk DMA or DSP contention; the blitter stops the CPU (4 cycles per word, 64/256 sharing outside hog mode) | Hatari videl.c, blitter.c:254-451 |
| AP68030: one `clk`, no clock enable, both edges used (one bus S-state per half clock); bus cycles follow the UM state by state (asynchronous 3 clocks + waits, synchronous 2, burst 2-1-1-1, dynamic bus sizing); internal timing is as fast as its sequencer allows (MOVE.L Dn,Dn 1 clock, MULU.W 7, DIVU.L 22, about 5.5 clocks per instruction); real cache organisation (256-byte I/D, 16-byte lines, write-through D-cache) | rtl/AP68030 ap030_top.v:27, ap030_bus.v:14-25,104-699, README; measurements with its tb_prog bench |
| Adding a clock enable to the AP68030 is mechanical: the posedge blocks are gated by `ce`, the negedge blocks by `ce_f` (the `ce` of the preceding rising edge); at 16 MHz in the 32 MHz domain each clk_sys is exactly one S-state | AP68030 code review; done in milestone 1 |
| The core today: RAM/ROM answered as a 32-bit port, no deliberate waits; an uncached ST-RAM read is ~13 clk_sys (0.41 us: 3 + DDR3 latency ~7, plus arbitration) and not deterministic - the DDR3 is shared with the ARM, the scaler and refresh, and a Videl line fetch can hold the CPU off for hundreds of clocks; `cpu_16mhz` ($FF8007 bit 0) is stored in falcon_combel but unconnected | falcon_cpubus.sv:165-266, falcon_memarb.sv:157-270, tools/mbox_ping README, falcon_combel.sv:56,67, falcon_system.sv:460 |
| Things that already assume 16 MHz: the blitter's non-hog CPU timeout (512 clk_sys = "256 CPU cycles at 16 MHz"); the DSP runs at its real rate (1 DSP cycle per clk_sys), so the CPU:DSP ratio becomes the Falcon's 1:2 once the CPU runs at 16 MHz | falcon_blitter.sv:30-43, dsp56k_core.sv:24-26 |

## What "cycle-accurate" can mean here

1. **Real 68030, cycle for cycle.**  The 68030's timing comes from microcode that
   is not public, a 3-stage pipe and the overlap of sequencer and bus controller;
   the UM Section 11 tables are an approximation of that overlap.  Matching it
   would mean redesigning the AP68030's sequencer, and it can only be checked
   against measurements on a real Falcon.
2. **Hatari's Falcon timing, cycle for cycle** ("Hatari-exact").  Hatari is the
   project's behavioural reference, its model is precisely defined (the table
   above) and it can be the golden: the same 68k test programs, timed with the
   MFP timers or a cycle-counting loop, run in Hatari and on the core.  Where the
   real 68030 differs from Hatari (an odd word is two bus cycles on the chip, one
   in Hatari), the AP68030's pin-level bus already does what the chip does, and
   the deviation is documented ("hardware wins").
3. **Bus-accurate only.**  16 MHz bus cycles of the right length on a 16-bit
   port, instruction timing left to the AP68030 (2-4x faster per instruction
   from the caches).

Recommendation: target 2, built so that 3 is an intermediate milestone and
the internal-timing rules can be refined towards 1 later where a real Falcon is
available to measure.

## The memory problem

A Falcon ST-RAM word access is 3 CPU clocks in a 4-clock slot: 6-8 clk_sys at
16 MHz.  A DDR3 read through the arbiter is ~13 clk_sys with unbounded tails.
Padding a cycle to its target length cannot shorten a late one.  Options:

- **A. DDR3 with time accounting.**  The bus model keeps a virtual 16 MHz time:
  a cycle ends at its Hatari-exact time if the data is there, later otherwise,
  and the lateness ("debt") is paid back until the CPU is back on schedule.
  (As built in milestone 2: a late cycle does not get longer; the processor's
  clock is held until the answer is there, and it runs at 32 MHz while it is
  behind.)  A read-ahead buffer serves the other three words of a 64-bit DDR3
  word without a new DDR3 access, so sequential code (every instruction fetch
  that misses the I-cache) needs one DDR3 read per 4 words: ~13 clk_sys against
  32 clk_sys of real bus time.  Exact whenever DDR3 keeps up (the common case:
  cached code, sequential fetch); a bounded, self-correcting drift under heavy
  random access or video/ARM load.  No extra hardware.  Deterministic in
  simulation, not exactly so on hardware.
- **B. SDRAM for the cycle-exact mode.**  ST-RAM in the MiSTer SDRAM board
  (deterministic latency, ~60 ns for a word at 100+ MHz, refresh in the bus idle
  slots), Videl and the DMA masters fetching from it in fixed slots like the real
  DRAM controller.  Exact always, but a new memory subsystem (controller, loader,
  DMA, cache snoop, video fetch), more logic (the core is at 96% of the ALMs),
  and the SDRAM board becomes a requirement for the mode.
- **C. A whole-machine stall** when DDR3 is late.  Rejected: DDR3 is late on
  nearly every uncached access, so the machine would run far below real speed,
  and video/audio output would glitch.

Recommendation: A, with the debt measured and reported (a counter readable in
simulation, and a statistic on hardware), and B kept open if A's drift shows up
in real software.

## Proposed architecture

```
falcon_cpuclk: cpu_ce (16 or 8 MHz, 32 MHz while behind), debt, clock position
   |  hold (from the bridge)
AP68030 (USE_CE)
   |  pin-level bus, DSACK1 for ST-RAM/ROM (16-bit port), as on the Falcon
falcon_cpubus, Falcon mode: the Falcon/Hatari bus timing model
   |   - 4-clock ST-RAM slots, 3-clock accesses, FAST16 ROM/I/O,
   |     per-device waits and acknowledge lengths (Hatari's numbers)
   |   - holds the processor while an answer is late; posted write;
   |     read-ahead buffer (one 64-bit DDR3 word)
falcon_memarb (unchanged priorities)  ->  DDR3
```
- **Clock enable** in the AP68030 (`USE_CE`, `ce`; always 1 in the 32 MHz
  mode, which stays bit-identical) - a change to the submodule, upstreamed
  as an AP68030 PR like the coprocessor fixes.
- **Speed selection**: OSD "CPU: Falcon 16 MHz / 32 MHz turbo"; in the Falcon
  setting $FF8007 bit 0 switches 8/16 MHz as Hatari does (and reads back the
  current speed).  32 MHz turbo keeps today's behaviour exactly.
- **Internal timing governor** (milestone 3): Hatari's rules - 2 cycles per
  instruction word consumed, absorbed by the last bus access's window, MUL/DIV
  constants, interrupt-acknowledge costs (section "Hatari's Falcon CPU timing") -
  as a stall on the AP68030's dispatch, from a small table.  The AP68030 is
  faster than Hatari in every measured cached case, so padding is enough; cases
  where it is slower are found by the timing benches.
- **Blitter**: its 64/256 sharing already counts 16 MHz cycles; its bus cycles
  get Hatari's 4 cycles per word.

## Verification plan

- AP68030 regression with the clock enable active (16 and 8 MHz) and inactive.
- A timing bench: 68k programs that time instruction sequences (cache on/off,
  ST-RAM/ROM/I/O, MUL/DIV, interrupts) with a cycle counter, run in Hatari
  (golden, its own CPU core) and on the core (tb/system with the DDR3 model,
  plus a fixed-latency memory model for the exact case); the cycle counts must
  match exactly where DDR3 keeps up, and the debt is reported where it does not.
- Existing benches: tb/fpu, tb/integration, tb/system frames (32 MHz turbo stays
  bit-identical; 16 MHz gets new references).
- Hardware: a TOS timing program (like FPUBENCH) reporting cycles per sequence,
  compared with Hatari; and with a real Falcon if one is available.

## Milestones

1. Clock enable in the AP68030 (submodule), 16/8/32 MHz selection (OSD, $FF8007
   bit 0), everything else unchanged.  The machine runs at 16 MHz with today's
   bus.
2. The Falcon bus: 16-bit port for ST-RAM/ROM, Hatari's slot and wait-state
   timing, virtual time with debt, read-ahead buffer; the timing bench (the
   bus-cycle checker; the whole-program comparison with Hatari needs the
   internal timing of milestone 3 and moves there).
3. Internal timing governor (Hatari's rules); exceptions and interrupt
   acknowledge (the 4 idle clocks after it); blitter cycle costs; programs
   timed in Hatari and on the core.
4. Hardware: timing program, debt statistics, Quartus fit (the core is at 96%).

## Hatari's Falcon CPU timing in detail (the reference for milestones 2 and 3)

What Hatari charges for the 68030 in Falcon mode with its default "cycle
exact" CPU (Falcon, 68030, cycle exact, data cache on, no MMU, no TT-RAM;
configuration.c:726,841-844), read from its source (src/, paths relative to
it).  The generated CPU file (cpuemu_23.c) is not in the source tree, so the
per-instruction parts follow the generator, cpu/gencpu.c.  This is Hatari's
"~cycle-exact" 68030, not the UM's head/tail model.

**Time base.**  One unit is one CPU clock at the current frequency (8 or 16
MHz): CYCLE_UNIT 512, cpucycleunit 256 (cpu/sysdeps.h:551, newcpu.c:2274).
The position used by the slot rule is `CyclesGlobalClockCounter +
currcycle*2/CYCLE_UNIT` (cycles.c:312-320), a free-running clock count with no
relation to CPU reset or video.  `do_cycles_ce020` (cpu/custom.c:512-526)
advances the visible time in 2-clock steps and keeps an odd clock owed for
the next call, so the visible time is always even: in true time T the
visible time is T & ~1.

**Data accesses** (mem_access_delay_*_ce020, newcpu.c:9664-9900; bank types
memory.c:1639-1640,1783-1808):

| bank | regions | byte | word | long |
|------|---------|------|------|------|
| CHIP16 | ST-RAM, IDE, void and bus-error regions | 1 access | 1 access; 2 byte accesses if (addr & 3) == 3 | always 2 word accesses (addr, addr+2) |
| FAST16 | ROM, cartridge, I/O ($FF0000-) | 3 | 3; 6 if (addr & 3) == 3 | 6 |

- A CHIP16 access (cpu/custom.c:329-359 read, 407-426 write): `bus_pos =
  position & 3; if (bus_pos & 2) wait (4 - bus_pos)`, then the access, then
  3 clocks.  With the even visible time bus_pos is 0 or 2 (the 1-clock wait
  for 3 is never reached), so in true time: **wait 2 if T & 2, then 3**.  A
  byte/word costs 3 (T mod 4 = 0, 1) or 5 (2, 3); a long 8/6/10/8 for T mod 4
  = 0/1/2/3; back-to-back words 3 then 5.  Writes time like reads.
- FAST16: no slot rule (`do_cycles_ce020_mem`, 3 clocks per CPU020_MEM_CYCLE).
  An I/O register handler runs inside the access and adds its device wait
  before the 3/6 clocks; a word or long that spans several handlers pays each
  handler's wait (ioMem.c:513-589; an MFP long = 2 handlers = +8).
- TAS/CAS/CAS2: an ordinary read then an ordinary write, no lock
  (gencpu.c:5451-5454); with the data cache on, the read can hit the cache.
- Bus errors are taken at the end of the instruction (newcpu.c:5265-5272);
  the cost is the format $B frame's timed writes (not counted yet).

**Instruction fetch** (get_word_ce030_prefetch_2, newcpu.c:10597-10629): a
3-word prefetch queue refilled by aligned longword fetches, one long per two
words consumed, 6 bytes ahead.  After a jump, branch, RTE or exception
`fill_prefetch_030_ntx` (newcpu.c:11428-11463) fetches the target's long and
the next one, then consumes the first word (a third long for a target with
pc & 2).  The next opcode is fetched at the end of the current instruction,
so its cost goes to that instruction.
- I-cache hit: 0 clocks.  Miss: one long through the bank rule (ROM 6, ST-RAM
  two slot-ruled words); no burst on the Falcon (burst needs a FAST32 bank,
  TT-RAM only), so a miss fills one longword, never a line
  (fill_icache030, newcpu.c:9984-10105).
- Quirk: a fetched long enters the I-cache only if the global
  `mmu030_cache_state` has bit 7 (newcpu.c:10028), and with the data cache
  on every data write overwrites it with the written address's cachability
  (newcpu.c:859-871): after a write to I/O, instruction fills are not cached
  until the next write to RAM or ROM.
- **Each consumed instruction word costs 2 clocks** (`do_cycles_ce020_internal(2)`,
  newcpu.c:10627), taken first from the window of the last bus access
  (cpu_prefetch.h:54-80: the window is that access's visible duration; a new
  access replaces it; cache hits leave it alone); only the excess is real
  time.  I-cache-hit code with no recent bus access runs at 2 clocks per word;
  after a 6-clock ROM fetch the next 3 words are free.

**Data cache** (read_dcache030 / write_dcache030, newcpu.c:10208-10471): hit
0 clocks; a miss on a cacheable address reads the whole aligned long (two
slot-ruled words from ST-RAM) and validates that longword; writes are
write-through at the full bus cost, then update the cache (write-allocate
for long-aligned longs with CACR bit 13).  Stack accesses go the same way.

**Internal cycles.**  The UM head/tail/cycles table is emitted only as C
comments (gencpu.c:800-818; the emission is `#if 0`, gencpu.c:2653-2658, and
`c = 0; // HACK`, 2578-2580, 2697).  The only run-time internal charges are
DIVU.W 34, DIVS.W 48, MULU.W 20, MULS.W 20 (gencpu.c:8286, 8346, 8371,
8399), charged in full, not absorbed by the window.  Shifts, MULL/DIVL, bit
fields, MOVEM, the FPU and EA calculation cost only their instruction words
and bus accesses.

**Exceptions and interrupts** (Exception_normal, newcpu.c:3487-3790; no fixed
start cycles for the 68030, newcpu.c:3409,3478): IACK (iack_cycle,
newcpu.c:2901-3029) - MFP level 6: 12 clocks (CPU_IACK_CYCLES_MFP_CE,
includes/m68000.h:192); HBL/VBL autovector: wait to the next multiple of 10
clocks (M68000_WaitEClock, m68000.c:808-824: 0, 8, 6, 4 or 2) then 10;
DSP and SCC vectors: none - then 4 idle clocks in every case.  Then the frame
writes (word, long, word for format 0), the vector long read (ST-RAM page 0)
and the prefetch refill.  RTE: 3 reads (plus the rest of a longer frame),
then the refill.  An interrupt pending at an instruction boundary is taken
after the next instruction (spcflags snapshot, newcpu.c:5133,5303-5333;
inferred, not observed).

**Device waits** (M68000_WaitState, m68000.c:791-798; clocks at the current
frequency, never scaled):

| device | wait | source |
|--------|------|--------|
| MFP registers | +4 per handler (USART registers none) | mfp.c |
| YM2149 | +4 on the first access of an instruction (MOVEM: +4 on every 4th further access) | psg.c:475-503 |
| FDC/DMA | $FF8604 write +4, read +4 (0 for the sector-count branch); $FF8606 write +4, read 0; $FF860E +4 | fdc.c:4744-5600 |
| ACIA (IKBD, MIDI) | 6, plus an E-clock sync (to a multiple of 10) on the first ACIA access of an instruction | acia.c:546-561, midi.c |
| DSP host port | +4 per byte after the first (word +4, long +12) | falcon/dsp.c:856-900 |
| Videl, SCC, NVRAM, crossbar, blitter registers, joypad, IDE, SCSI | 0 (IDE pays the CHIP16 slot rule) | - |

**8 vs 16 MHz** ($FF8007 bit 0, ioMemTabFalcon.c:113-155,
configuration.c:1297-1325): only the real length of a clock changes.  The
slot rule, the 3-clock access, 2 clocks per word, the device waits, IACK
and the blitter costs are the same clock counts at both speeds.

**Blitter** (blitter.c): not concurrent with the 68030 (BLITTER_RUN_CE is for
the 68000 only).  It starts at an instruction boundary after
`CurrentInstrCycles` (a generator constant, value unknown here) and the CPU
is stopped while it runs: 4 clocks arbitration in, 4 per word read and 4 per
word write (any bank, any alignment), 4 out.  Hog mode runs to the end;
otherwise 64 bus accesses, then the CPU runs 256 clocks.

**Not modelled by Hatari:** Videl, DMA sound, disk DMA, DRAM refresh, DSP or
SCC bus contention, and FPU latency (an FPU instruction costs its words and
EA accesses only).  The CHIP16 slot rule is the only stand-in for the
memory controller.

**Reference algorithm for the bus model** (true time T, window credit W):
```
CHIP16 access:  if (T & 2) T += 2;  access;  T += 3
FAST16 access:  access (device waits added to T here);  T += 3 per word (6 for a long or an (addr&3)==3 word)
after a bus access:   W = its visible duration (even), replacing the old W
consume an instruction word:  fetch a long if due (I-cache miss only);  if (W >= 2) W -= 2; else T += 2
MULU.W/MULS.W +20, DIVU.W +34, DIVS.W +48  (no window credit)
interrupt: IACK (MFP 12 | autovector E-sync + 10) + 4; push word, long, word; read vector long; refill
```
Checks against a trace: a CHIP16 long from T mod 4 = 0/1/2/3 lasts 8/6/10/8
clocks; two back-to-back words from an aligned start last 8.

**Open points** (to settle against Hatari runs in milestone 2's timing bench):
the generated per-instruction code (refill placement, MOVEM, CAS2); the
interrupt recognition delay; the format $B frame cost; the blitter's start
delay; STOP; CACR freeze/flush details; whether the I-cache fill quirk
matters for TOS code.

## Decisions (2026-10-08)

1. Timing target: **Hatari-exact**; where the real 68030 differs, the chip's
   behaviour wins and is documented.
2. Memory: **DDR3 with time accounting** (virtual time, debt, read-ahead buffer).
3. Speed: OSD **Falcon 16 MHz (default) / 32 MHz turbo**; in the Falcon setting
   $FF8007 bit 0 switches 8/16 MHz.  Turbo keeps today's behaviour bit for bit.
4. Contention: none beyond Hatari's for now.
5. No real Falcon is available: Hatari is the golden; hardware runs on the
   MiSTer check the debt and the timing program against Hatari.

## Milestone 1: the CPU clock (done)

- **AP68030** (`USE_CE=1`, input `ce`): the core advances on the rising edges
  of clk with `ce` and on the falling edge after each of them (`ce_f`, made
  in ap030_top).  Every pattern works and the rate may change at any time.
  The data cache's snoop input stays ungated (a DMA write is a one-clock
  pulse in the 32 MHz domain); a snoop between the two enabled edges of a
  fill is applied again after the fill.  `USE_CE=0` leaves the logic as it
  was.
- **AP68030 regression** (rtl/AP68030/tb/run_tests.sh): every program also
  runs with `+ce=2`, `+ce=4 +waits=2` and `+ce_rand`.  The bench's memory and
  test registers run on the processor clock, and each run must take exactly
  the processor clocks of the run without the enable; all pass, in the pin
  and the native-port (`FAST_PORT=1`) builds.  t_exceptions' instruction
  trace is identical, clock for clock, at enable 1, 2, 4 and random.
- **Falcon** (falcon_system): `cpu_ce` every clock (OSD "32 MHz turbo"),
  every second clock (16 MHz) or every fourth (8 MHz) when $FF8007 bit 0 is
  clear (falcon_combel; cold reset $25 = 16 MHz, kept over warm resets, as
  Hatari).  OSD `status[9]`: "CPU: Falcon 16 MHz / 32 MHz turbo", Falcon 16
  MHz the default.  The bus is unchanged: falcon_cpubus holds DSACK/BERR/AVEC
  until AS negates, so it serves a slower CPU as it is.
- **Benches**: tb/system runs at 16 MHz by default, `--turbo` for 32 MHz (the
  frame-reference scripts run/*.sh pass `--turbo`, their references being
  32 MHz frames); tools/fputest/sim.sh `TURBO=1`; tb/fpu `CPU_DIV=1|2|4`.
  Results: tb/fpu PASS at CPU_DIV 1 and 2 (7167 checks each); tb/integration
  39/39; FPUTEST ALL PASS at 16 MHz; FPUBENCH in simulation FNOP 2.70 us /
  FADD 3.00 us at 16 MHz against 1.80 / 2.25 us in turbo (the bridge's
  latency is in system clocks, so the ratio is below 2).

## Milestone 2: the Falcon bus (done)

In the Falcon setting `falcon_cpubus` runs a second state machine (turbo
keeps the old one, untouched) that gives every bus cycle Hatari's length in
processor clocks:

- **Lengths**: CHIP16 (ST-RAM, IDE, unmapped) 3 clocks, plus 2 when S0 is at
  clock position 2 or 3 mod 4; FAST16 (ROM, cartridge, $FFxxxx) 3 plus the
  device waits of the table above (MFP, YM first access and every 4th
  further one in an instruction, FDC/DMA with the sector-count case, ACIA
  6 + E clock, DSP host port); interrupt acknowledge MFP 12, autovector E
  clock + 10, DSP and spurious 3; coprocessor and other CPU space 3; bus
  errors as their space.  Instruction boundaries (for the YM and ACIA rules)
  come from the AP68030's dispatch pulse; the clock position (slot, E clock)
  from `falcon_cpuclk` (restarted by reset).
- **The 16-bit port**: ST-RAM, ROM and cartridge answer with DSACK1 only, so
  a long is two word cycles as in Hatari.  A word at an odd address is two
  byte cycles on the 68030's 16-bit port and one access in Hatari; the chip's
  behaviour is kept (hardware wins).
- **Timing**: the cycle's length is set by when DSACK (BERR, AVEC) becomes
  visible: at the falling edge after the processor's (w+1)th rising edge of
  the cycle.  An answer that is not there by then never lengthens the cycle:
  `hold` stops the processor's clock until it is, and `falcon_cpuclk` counts
  the Falcon time that passed as debt and gives the processor a clock on every
  system clock (32 MHz) until it has caught up.  The processor is also held on
  the clock a new cycle is first seen (the edge after S0 can be on the next
  clock while it catches up, before the bridge has decided anything).  So the
  processor's execution, counted in its own clocks, does not depend on DDR3
  latency, video load or device delays at all; only its real-time position
  lags by the debt.  The debt is capped at 4095 system clocks (128 us); above
  that it is forgiven and counted.
- **Memory**: RAM writes are posted (one entry; a later RAM read, a device
  access and the blitter's bus grant wait for it); RAM/ROM reads go through a
  read-ahead buffer holding the last 64-bit DDR3 word (`falcon_memarb` now
  also returns the whole word), kept coherent with the posted write and
  invalidated by other masters' writes (the arbiter's snoop) and while the
  loader runs, so sequential fetches need one DDR3 read per four words.  The
  data of a write is taken at S0 (the AP68030 drives its write lanes from
  there), so a write cycle never waits for DS.
- **Deviations from Hatari** (hardware wins or Hatari has no bus cycle): odd
  words (above); a DSP-vectored acknowledge takes a 3-clock cycle (Hatari
  charges nothing); the 4 idle clocks after an acknowledge are internal time
  (milestone 3).  FPU latency (the ARM) and DDR3/video contention are debt:
  Hatari charges nothing for the FPU and models no contention.

Verification:

- **tb/bustime** (new): the AP68030, `falcon_cpuclk`, `falcon_cpubus` and
  `falcon_memarb` on the DDR3 model (random latency and BUSY), with device,
  interrupt-controller and DMA models, run `t_bustime.s` (RAM in every size
  and alignment, buffer and posted-write coherence, a DMA write into the
  buffered word, ROM, every device class, bus errors in CHIP16/FAST16 space
  and from user mode, the four acknowledge kinds, TAS; caches off and on).
  A checker written from the rules above, independently of the bridge,
  measures every cycle (about 7,200 per run) in processor clocks: all match
  Hatari, none needed a wait state.  The program runs in five configurations
  (default; video fetches; 8 MHz; two other device/acknowledge delay seeds
  with video, one at 8 MHz) and takes exactly the same processor clocks in
  each (37,551).  With heavy video load the debt reaches the cap.
- tb/fpu with `FMODE=1` (the bridge's Falcon mode, coprocessor cycles held
  for the ARM): 7167/7167; tb/integration 57 + 39 checks (its pin-driven
  bus cycles now go through the Falcon mode); FPUTEST passes on the whole
  system at 16 MHz (the debt reaches the cap: FPU latency).
- Turbo is bit-identical to milestone 1: FPUTEST's whole run in tb/system
  with `--turbo` gives the same PC trace (651 samples) as the milestone 1
  build.
- Quartus 17.0, seed 3: timing met (+2.70 ns setup on the system clock),
  41,175 ALMs (98%, +892 over milestone 1): milestones 3 and 4 have about
  700 ALMs left.

