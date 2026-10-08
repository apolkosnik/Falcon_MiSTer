# A 16 MHz, cycle-accurate 68030 - design

Status (branch `feature/cpu-16mhz`): decisions taken (below); milestone 1
done (the CPU clock: 16/8 MHz or 32 MHz turbo, see "Milestone 1" at the end);
milestones 2-4 open.  Until milestone 1 the core ran the AP68030 on every
clock of the 32 MHz system clock and ended every bus cycle as soon as memory
or the device answered; a Falcon030 runs it at 16 MHz (8 MHz selectable) on
a 16-bit bus.  This document says what "cycle-accurate" can mean for this
core, how to get there, and what was decided.

## Facts this rests on

| Fact | Source |
|------|--------|
| Falcon CPU clock 16.042 MHz (32.084988 MHz / 2); $FF8007 bit 0 switches 8 / 16 MHz (68030 only), a read returns the current setting; bit 5 (bus mode) has no timing effect | Hatari clocks_timings.c:157,325-342; ioMemTabFalcon.c:103-155 |
| The CPU's data bus to ST-RAM is 16 bits; Videl is the DRAM controller with its own 32-bit burst path; video modes cost the CPU only a few percent | Atari Falcon references (Wikipedia, DFB wiki, Robbins 1993); forum reports (unverified) |
| Hatari's Falcon CPU model ("cycle exact", the default): ST-RAM is CHIP16 - an access starts on a 4-cycle slot boundary (cycle position mod 4: 2 or 3 waits to the next slot) and takes 3 cycles; a long is two 16-bit accesses (8 cycles back to back); ROM, cartridge, I/O are FAST16, 3 cycles per byte/word, 6 per long; device waits on top (MFP, YM, FDC +4, ACIA +6 and E-clock) | Hatari custom.c:329-426; newcpu.c:9664-9900; memory.c:1640,1778-1808; mfp.c, psg.c, acia.c |
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
  and the lateness ("debt") is paid back by ending later cycles early (cache hits
  and on-chip time never wait while there is debt) until the CPU is back on
  schedule.  A read-ahead buffer serves the other three words of a 64-bit DDR3
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
AP68030 (ce_rise/ce_fall: 16 or 8 MHz)
   |  pin-level bus, DSACK1 for ST-RAM/ROM (16-bit port), as on the Falcon
falcon_cpubus  +  falcon_bustime (new): the Falcon/Hatari bus timing model
   |   - 16 MHz cycle counter, 4-cycle ST-RAM slots, 3-cycle accesses,
   |     FAST16 ROM/I/O, per-device waits (Hatari's numbers)
   |   - virtual time and debt; read-ahead buffer (one 64-bit DDR3 word)
falcon_memarb (unchanged priorities)  ->  DDR3
```
- **Clock enable** in the AP68030 (`USE_CE`, `ce`; always 1 in the 32 MHz
  mode, which stays bit-identical) - a change to the submodule, upstreamed
  as an AP68030 PR like the coprocessor fixes.
- **Speed selection**: OSD "CPU: Falcon 16 MHz / 32 MHz turbo"; in the Falcon
  setting $FF8007 bit 0 switches 8/16 MHz as Hatari does (and reads back the
  current speed).  32 MHz turbo keeps today's behaviour exactly.
- **Internal timing governor** (milestone 3): Hatari's rules - at least 2 cycles
  per instruction word consumed, MUL/DIV constants, interrupt-acknowledge costs -
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
   timing, virtual time with debt, read-ahead buffer; the timing bench and the
   Hatari golden.
3. Internal timing governor (Hatari's rules); exceptions and interrupt
   acknowledge; blitter cycle costs.
4. Hardware: timing program, debt statistics, Quartus fit (the core is at 96%).

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

