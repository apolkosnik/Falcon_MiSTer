# AP68030

A synthesisable MC68030 in Verilog: the complete 68020/030 integer
instruction set, the on-chip instruction and data caches, the paged memory
management unit, the coprocessor interface and the MC68030 bus with its
timing, all validated against the *MC68030 User's Manual* (MC68030UM) and
the *M68000 Family Programmer's Reference Manual* with Verilator.

The design targets 50 MHz in a Cyclone V and behaves like the real part at
the pins: half-clock bus states S0-S5, asynchronous (DSACKx, 8/16/32-bit
dynamic bus sizing) and synchronous (STERM) cycles, burst fills with
CBREQ/CBACK, ECS/OCS, DBEN, RMC, CIOUT/CIIN, BERR/HALT retry and rerun,
bus arbitration, interrupt, breakpoint and coprocessor acknowledge cycles
in CPU space, and the RESET instruction driving the RESET pin for 512
clocks.

## Layout

```
rtl/ap030_top.v        pin-level top: 68030 signal set, reset synchroniser
rtl/ap030_core.v       instruction unit: prefetch pipe, decoder, sequencer, exceptions
rtl/core/*.vh          sequencer states (exec_a/b/c), decoder, frame layout, tasks
rtl/ap030_memsys.v     data/instruction units: cache lookup, translation, write buffer,
                       bus arbitration between walker, writes, data and prefetch
rtl/ap030_cache.v      256-byte direct mapped cache (16 lines x 4 entries), logical tags
rtl/ap030_mmu.v        ATC (22 entries), TT0/TT1, table search engine, PTEST/PLOAD/PFLUSH
rtl/ap030_bus.v        bus controller: S0-S5 states, sizing, bursts, retry, arbitration
rtl/ap030_alu.v        ALU, barrel shifter, BCD
rtl/ap030_muldiv.v     32x32 multiply, 64/32 divide
rtl/ap030_regfile.v    D0-D7, A0-A6 (in MLABs, one copy per read port), USP/ISP/MSP
rtl/ap030_defs.svh     constants
tb/tb_ap030_bus.sv     pin-level bench of the bus controller with slave models
tb/tb_ap030_program.sv program bench: 1 MB RAM behind several port types, test registers
tb/tb_cp_model.svh     scripted coprocessor at CpID 1
tb/asm/*.s             self-test programs (vasm syntax)
tb/run_tests.sh        the regression
syn/                   Quartus project for the timing check
doc/ARCHITECTURE.md    how the core is built
```

## Running the tests

Requirements: Verilator 5.x, `vasmm68k_mot` (vasm with the Motorola syntax
module), Python 3.

```
cd tb
sh run_tests.sh            # everything: bus bench, every program, with and without wait states and the clock enable
FAST_PORT=1 sh run_tests.sh /tmp/ap030-native-suite  # native RAM plus pin-bus fallbacks
./run_prog.sh t_mmu +trace # one program, with the instruction trace (+bustrace, +ctrace, +rftrace, +itrace, +strace)
```

Every passing program reports its clock count, instruction count and the
resulting clocks per instruction (processor clocks: with `+ce=N` or
`+ce_rand` the bench and its memory run on the processor clock, and the
suite checks that each such run takes exactly the clocks of the run without
the enable); `+strace` prints the sequencer state and
the memory unit's handshakes every clock for profiling.

The programs report through memory-mapped test registers (documented at
the top of `tb/tb_ap030_program.sv`) and print the source of a failing
check.

| program        | covers                                                                       |
|----------------|------------------------------------------------------------------------------|
| `t_integer`    | the integer instruction set, addressing modes, condition codes, bit fields, CAS, MOVEM, MOVEP, BCD |
| `t_exceptions` | traps and frames ($0/$1/$2/$9/$A/$B), illegal/A-line/F-line, CHK/CHK2, privilege, MOVEC/MOVES, trace T1/T0, interrupts (levels, autovector, vectored, spurious, level 7 edge), STOP, master stack and throwaway frames, RTE format errors, BKPT, bus error rerun and software completion on reads, posted writes (two failing in a row: two bus errors in program order, no halt) and instruction fetches, address errors, RESET (512 clocks, no bus cycle during the pulse) |
| `t_stack`      | the active stack pointer right after a write to it: an exception at the next instruction's dispatch, MOVEC ISP/MSP, RTS/RTD/RTR/RTE to an odd address (frame position and A7 afterwards) |
| `t_held`       | a failed posted write suspends the instruction at its next data access: one bus error ($B, PC of that instruction, the first failed write), RTE reruns the write and issues the held access (reads, writes, RMW, MOVEM/MOVEP, (An)+/-(An), coprocessor and breakpoint cycles, user mode); without another access the fault is taken at the boundary ($A) |
| `t_extword`    | an illegal instruction, or a privileged one in user mode, whose next word's prefetch bus-errors takes the illegal instruction or privilege violation exception; an instruction that uses that word takes the bus error |
| `t_bus`        | dynamic bus sizing through 32/16/8-bit ports, misaligned operands, line and page crossings, cache hits, MOVEP, TAS/CAS RMW cycles (RMC, no prefetch inside, release after a CAS/CAS2 mismatch, bus grant after), CIIN (the rest of a misaligned read without CBREQ), code from narrow ports |
| `t_cache`      | CACR (enable, freeze, clear all/entry, write allocate, burst enable), write-through, the instruction cache and self-modifying code, hit/miss timing, a bus error on a burst's first cycle invalidating the line |
| `t_snoop`      | the snoop port: another master writes a longword 0-15 clocks after the processor read it, while the fill of that read is under way (32-bit synchronous or native port, 16-bit and 8-bit ports, burst) |
| `t_mmu`        | MMU registers and configuration exceptions, two- and three-level trees, short and long descriptors, early termination, indirect descriptors, limits (checked after the U update, not by a PTEST that ends above them), U/M history updates, WP/supervisor faults with RTE rerun, supervisor violations in PTEST (S on table and page descriptors), page-crossing operands, instruction fetch faults, harmless prefetches into unmapped pages, PTEST (all levels, An result), PLOAD, PFLUSH variants, PMOVEFD, FCL, SRE, TT0 (FC, R/W, CI), MMUDIS, ATC replacement, cache inhibit |
| `t_pmmuenc`    | MMU instruction encodings: the supported forms run; reserved bits of the UM 3.3.3 formats, function code 11xxx, a PC relative or immediate EA field and the other CpID 0 types take the F-line exception in supervisor mode and a privilege violation in user mode (UM 9.8) |
| `t_cp`         | the coprocessor protocol: every response primitive, cpGEN with all EA forms, cpBcc/cpDBcc/cpScc/cpTRAPcc, cpSAVE/cpRESTORE, busy, exceptions requested by the coprocessor |
| `t_cpx_*`      | programs on the coprocessor model's response script (`tb/asm/t_cpx.i`, found by `run_tests.sh`; each header says what it checks): the script itself, tracing through coprocessor instructions and instruction traps, the take address and top of stack primitives, the encodings the processor rejects |
| `t_dblfault`   | a bus error while stacking a bus error frame halts the processor |
| `t_lazy`       | `fetch_lazy` with the pipeline-model inputs set by the program (run with `+lazy` only): wrong stops and a stalled scan must not hang the processor, a correct stop on an RTS must hold |

## Timing and performance

The standalone FPGA figures below are historical measurements, before the
current throughput and optional native-port changes. Current full Minimig
fits are reported under the native-port results below. The updated RTL has
not been validated on a board.

`syn/run_syn.sh` runs Quartus Prime (Cyclone V 5CSEBA6U23I7, 50 MHz
constraint, ports as virtual pins with a 2 ns budget) and prints the Fmax
summary. Quartus Prime 17.0 reports Fmax 53.0 MHz (slow 1100 mV 100 C
corner, slack +0.57 ns at 50 MHz), 17.0k ALMs and 9.5k registers.

Run Quartus outside the execution sandbox. Here, sandboxed invocations
report expired-evaluation error 292037, while the same installation runs
successfully outside the sandbox. Use an isolated source snapshot for a
build so concurrent RTL edits cannot change its inputs.

Dhrystone 2.1 (`tb/c`, compiled with vbcc `-O2 -speed -cpu=68030`, caches
on, 2000 runs, checked against its final values) now runs in 2423 clocks
per Dhrystone on the synchronous burst port: **11.74 DMIPS at 50 MHz**.
With two wait states it takes 2802 clocks, **10.16 DMIPS**. These are
simulation results normalized to 50 MHz. `run_tests.sh` builds and runs it
when vbcc is installed (`VBCC=/opt/amiga-cc/vbcc` by default).

| FPGA resource (Cyclone V) | used |
|---|---|
| ALMs | 17,041 (41 %) |
| registers | 9,548 |
| block RAM | 5 M10K, 4,096 bits |
| DSP blocks | 4 |

On the self-test programs (cache hits, synchronous memory, no wait states)
the core runs at 5.5 clocks per instruction on the integer suite and 6.1
to 6.6 on the exception and MMU suites, which are dominated by long
instructions (absolute long operands, MOVEM, exception frames, table
searches); instruction cache hits stream at one longword per clock, a data
cache hit takes two clocks and consecutive stores run four clocks apart.

## Optional native Fast RAM port

`ap030_top` defaults to `FAST_PORT=0`, retaining the pin-bus interface.
With `FAST_PORT=1`, an integration can decode the translated `fast_addr`
and `fast_fc`, then assert `fast_match` for reliable internal RAM.
The memory system keeps locked RMW transfers, MMU table searches and
interrupt acknowledgements on the pin bus. Translation and protection
checks precede either route. Instruction fetches and ordinary data accesses
can use the native port.

The request is accepted on a rising edge with `fast_req && fast_ready`;
fields remain stable until acceptance. `fast_rw=1` reads. `fast_be[3]`
selects `fast_wdata[31:24]`, the lowest-address byte of the aligned word.
Responses start at least one clock after acceptance, with `fast_valid`
and an aligned 32-bit `fast_rdata`. The first response completes the operand;
`fast_word` identifies its word within the 16-byte line. A burst returns
four words, requested word first, wrapping modulo four; `fast_last` releases
the slot. Writes require one completion response. There is one transaction
at a time and no response backpressure or bus-error response. Targets that
can fault must use the pin route. Reset must cancel or drain the target's
outstanding transaction before accepting a new request.

The prepared Minimig integration enables this port and shares the existing
8 KiB cache, line buffers, posted-write queue and DDR clock crossings between
native and pin traffic. CPU-space accesses and the special NMI-vector path
retain the pin route. Cache-inhibited accesses, clears, byte writes and
cross-port coherence follow the existing cache rules. The implementation
also prevents an interrupted fill from surviving a concurrent cache clear.

The same 500-run Fast RAM Dhrystone image produced these simulated results:

| Integration variant | CPU cycles | DMIPS normalized to 50 MHz |
|---|---:|---:|
| Committed 8 KiB cache (`04af1491`) | 1,470,205 | 9.68 |
| EA/MOVE/branch throughput changes | 1,360,257 | 10.46 |
| Throughput changes plus native port | 1,170,812 | 12.15 |
| Native port, slower DDR and stalls | 1,172,264 | 12.14 |
| Native port on original core, without throughput changes | 1,307,759 | 10.88 |

The native port adds **16.2%** over the throughput changes; together they
add **25.6%** over the committed cached integration. The integrated bench
uses a 20.6 ns CPU period and reports normalized 50 MHz results. These are
not board measurements.

Quartus Prime 17.0.2 full-project fits, Cyclone V 5CSEBA6U23I7, original
timing constraints:

| Integration variant | ALMs | RAM blocks | CPU Fmax, slow 100 C | CPU setup slack at 50 MHz |
|---|---:|---:|---:|---:|
| Previous board report (2026-10-01 21:29) | 37,072 | 287 | 50.27 MHz | +0.053 ns |
| Selected combined version, seed 8 | 38,203 | 287 | 50.55 MHz | +0.108 ns |
| Corrected combined version, seed 7 | 38,221 | 287 | 50.48 MHz | +0.191 ns |
| Native port on original core, seed 7 | 37,343 | 287 | 50.41 MHz | +0.081 ns |

The previous board report is a reference measurement, not a fresh baseline
rebuild. The initial combined fit failed CPU timing (42.27 MHz). Selecting
the MMU port before resolving late bus stalls and separating the pin-path
cache-hit comparison shortened those critical paths without changing the
Dhrystone cycle count. The selected seed-8 build passes project-wide setup,
hold, recovery, removal and pulse-width checks at all four available corners.
It costs 1,131 more ALMs than the prior board report (3.1%); this change
improves speed rather than reducing area. Its tightest project-wide setup
margin is +0.012 ns in the HDMI scaler at the slow -40 C corner. CPU timing
passes at 50 MHz; this is not an overclocking result.

Neither seed-7 fit meets timing for the full project: the corrected combined
build has a -0.099 ns setup path from the shared DDR arbiter into the HPS
SDRAM interface. The original-core variant has -0.277 ns DDR and -0.015 ns
HDMI setup failures at the default corner; its CPU also misses timing at the
slow -40 C corner (-0.011 ns, 49.95 MHz Fmax). Seed 8 uses identical RTL and
SDC to the corrected combined seed-7 version; no timing exceptions were
added. The selected build is ready for board testing, but has not been
programmed onto a board.

Validation includes 58 integration runs; 18 standalone runs with the native
port disabled and 18 enabled; frontend tests with mixed native/pin traffic,
partial writes, FIFO pressure, wrapped fills and clear/reset races; and
28,987 selected WinUAE rounds across 51 slices with zero mismatches. The
native corpus mode routes ordinary data through the direct port and leaves
instruction fetches on pins for the replay harness's observation points.
Use `tb/cputest/run_cputest.py --native` with a separate work directory to
repeat that mode. This is a targeted corpus subset, not a complete corpus run.

The [board patch](doc/minimig-native.patch) includes placement seed 8 and
targets integration commit `04af149119925f1a33dbe9807cf131a120f07b68`.
[Validation records](doc/native-port-results.json) include the selected
build, comparison fits and artifact hashes. The fitted image is
`/tmp/ap030-native/quartus-timing-seed8/output_files/Minimig.rbf`; exact source
hashes and timing reports are in that build directory. Applying the patch
to the board-build worktree is pending confirmation that no Quartus compile
is using that tree. All builds here used isolated source snapshots.

## Clock enable

With `USE_CE=1` the processor clock is `clk` gated by `ce`, a clock enable
in the `clk` domain: the core advances on the rising edges of `clk` with
`ce` set and on the falling edge that follows each of them.  `ce` on every
second clock runs it at half the `clk` rate, on every fourth at a quarter;
any pattern works, and the rate may change at any time.  A system that runs
the processor slower than its own clock (the Falcon030 core: 16 or 8 MHz
from 32 MHz, with a 32 MHz turbo setting) needs no second clock domain.

The surroundings see an MC68030 whose clock is the enabled edges.  The
pins change only on those edges, and the inputs are sampled only on them,
so an asynchronous slave (DSACKx, BERR, AVEC held until AS negates) works
unchanged.  A synchronous one (STERM, CBACK, the native port) must also
advance on the enabled edges, as a board's memory controller would run on
the processor clock.  `snoop_we` is the exception: it is taken on every
clock, since another bus master's write is a single-clock pulse in the
system's clock, and a snoop from the clock a read is accepted until its
cache fill is written is applied again after that fill.

`USE_CE=0` (the default) ignores `ce`; the enables are then constant and
the logic is the same as without them.

## Timing hooks

Two outputs let an external timing model follow the instruction stream
(the Falcon030 core maps the processor's internal time onto Hatari's with
them): `tm_pop` is the number of instruction words taken from the prefetch
queue in a processor clock, and `tm_md` reports a word-size multiply or
divide starting (1 MULU.W/MULS.W, 2 DIVU.W, 3 DIVS.W).  Both are registered
and hold for one processor clock, like `dbg_inst` (an instruction
dispatched).  They change nothing inside the processor.

`fetch_lazy` gives the instruction prefetch the policy of Hatari's
cycle-exact 68030 (`get_word_ce030_prefetch`, `fill_prefetch_030_ntx`): a
longword is fetched only when two words or fewer are left, queued or on the
way, after the words taken in a clock (instead of four), and a branch
refills the queue with the target's longword and the next one.  Hatari also
stops prefetching "one word early" ahead of an unconditional flow change
(RTS, RTE, RTD, RTR, JSR, JMP, BSR), which it finds by decoding instruction
lengths ahead of execution (`pipeline_020`); the processor leaves that
decoding to an external model and exports the queue for it: `tm_q`/`tm_qn`
(the queued words, the next to take in `tm_q[95:80]`), `tm_scan` (its
address) and `tm_flush` (the queue was flushed or reloaded).  The model
answers with `fetch_stop_v`/`fetch_stop` (no longword is fetched once the
next word to take is at `fetch_stop - 2` or beyond) and
`fetch_scan_v`/`fetch_scan_to` (fetch decisions wait until the model has
scanned two words past the consumption that made the fetch due).  Should the
processor wait 63 clocks for an instruction word that is neither queued nor
on the way, it ignores the stop and the scan wait, so wrong model inputs
cannot hang it.  With `fetch_lazy` at 0 the model inputs are ignored and the
processor fetches as before; `tb/run_tests.sh` runs every program once more
with `+lazy`, and `t_lazy` with model inputs set by the program (wrong ones,
and a correct stop that must hold).  The Falcon030 core's model is
`falcon_pipescan` (Hatari's opcode table in block RAM).

## Licence
Copyright (C) 2026 Adam Polkosnik

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; **either version 2 of the License, or
(at your option) any later version**.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License along
with this program; if not, write to the Free Software Foundation, Inc.,
51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

GPL, see `LICENSE` for details.
