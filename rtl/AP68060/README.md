# AP68060

A pipelined Motorola **MC68060** in SystemVerilog: integer unit, FPU, MMU,
caches, store buffer, branch cache and the real 68060 bus. It targets the
MiSTer's Cyclone V (5CSEBA6U23I7) at **60 MHz** in **under 20,000 ALMs**.
It is validated in Verilator against self-checking programs, the WinUAE
68060 `cputest` corpus and the *MC68060 User's Manual* (MC68060UM).

The core is derived from the AP68040-60 core and follows the 68060's
architecture rather than the 68040's. Examples:

- one supervisor stack;
- the 060's exception frames;
- traps to the 68060 software package for the instructions the 060 leaves to software;
- three-long-word FPU state frames;
- the 060's MMU and cache control registers;
- the 060 bus protocol, including its reset modes.

Behaviour is checked against the manual first. Where WinUAE and the manual
disagree, the core follows the manual, and the differences are listed in
[doc/STATUS.md](doc/STATUS.md).

## Status at a glance

Measured at commit `b4cf202`, default build (all area options off).

| | |
|---|---|
| Target | Cyclone V 5CSEBA6U23I7 (MiSTer), out-of-context fit |
| Area | 19,972 ALMs at 60 MHz, 28 under the 20,000 target. The 112 ALMs of virtual I/O from the out-of-context fit are included. Results vary by about ±300 ALMs from fit to fit, so the target is not yet met with margin. |
| Other resources | 12,094 registers, 352,256 block memory bits, 15 DSP blocks |
| Timing | +0.380 ns setup slack at 60 MHz (16.667 ns); Fmax 61.4 MHz |
| Regression | 34 programs × 11 bus configurations (374 runs), plus the fetch unit's TCI bench: all pass |
| Corpus | WinUAE `cputest` 68060 corpus: every slice passes except the documented oracle deviations |
| Dhrystone 2.1 | 971 cycles per run: 35.17 DMIPS at 60 MHz (0.586 DMIPS/MHz) |
| Whetstone | 628,390 cycles per loop: 9.55 MWIPS at 60 MHz (0.159 MWIPS/MHz) |

**Not implemented**:

- the second integer pipeline (sOEP, superscalar dispatch): `PCR.ESS` reads and writes but has no effect;
- branch-prediction-error access errors (`FSLW.BPE`): a stale branch cache entry is corrected in the decoder instead;
- the multiplexed bus mode that CDIS selects at reset.

## Architecture

```
            +-----------+   +----------+   +----------+
 68060 bus  |  fetch    |-->| D1       |-->| D2       |--+
 <--------->|  I-ATC    |   | decode   |   | micro-   |  |  uops
   BIU      |  I-cache  |   | (predec) |   | sequencer|  |
            |  BTB, IQ  |   +----------+   +----------+  |
            +-----------+                                v
                 ^           +-----+   +-----+   +-----+   +----+   +----+
                 |           | AG  |-->| DC1 |-->| DC2 |-->| EX |-->| WB |
                 |           +-----+   +-----+   +-----+   +----+   +----+
                 |                        \_______DMU______/   |
                 |                   D-ATC, data cache, table   |
                 +---------------    walker, store buffer,      +-- ALU, shifter,
                     snooper         push buffer                    mul/div, FPU
```

- **Fetch** ([ap68060_fetch.sv](rtl/ap68060_fetch.sv)): the I-ATC, an 8 KB four-way instruction cache, the branch cache and the instruction queue, with two predecoders ([ap68060_predec.sv](rtl/ap68060_predec.sv)).
- **D1** ([ap68060_decode.sv](rtl/ap68060_decode.sv)): parses one instruction per cycle on the fast path. Full-format extensions and FPU immediates take a multi-cycle path.
- **D2** ([ap68060_useq.sv](rtl/ap68060_useq.sv)): walks the instruction's microcode routine, one uop per cycle. It maps symbolic operands to registers and expands memory-indirect effective addresses into pointer loads.
- **Back end** ([ap68060_backend.sv](rtl/ap68060_backend.sv)):
  - AG: register read and effective address;
  - DC1 and DC2: the data ATC and cache lookup;
  - EX: [single-cycle ALU](rtl/ap68060_alu.sv), [two-cycle shifter/BCD unit](rtl/ap68060_alu_slow.sv), [iterative multiply/divide](rtl/ap68060_muldiv.sv), branch resolution and the FPU;
  - WB: write-back.
- **DMU** ([ap68060_dmu.sv](rtl/ap68060_dmu.sv)):
  - the D-ATC;
  - an 8 KB four-way data cache: copyback, write-through or noncachable;
  - the table walker;
  - the four-entry store buffer;
  - the push buffer.
- **ATC** ([ap68060_atc.sv](rtl/ap68060_atc.sv)): 64 entries, four-way set associative, 4K or 8K pages.
- **FPU** ([fpu/ap040_fpu.v](rtl/fpu/ap040_fpu.v), wrapped by [ap68060_fpif.sv](rtl/ap68060_fpif.sv)):
  - FP0-FP7 in extended precision;
  - the 060's hardware instruction subset;
  - an optional arithmetic pipeline (`FPU_PIPE`).

  Every FPU side effect takes place only when its uop is the oldest in flight.
- **BIU** ([ap68060_biu.sv](rtl/ap68060_biu.sv)): one transaction at a time on the 68060 bus:
  - byte enables BS3-BS0;
  - bursts, with TBI falling back to long-word transfers;
  - retries, including native TRA retries;
  - locked sequences;
  - acknowledge cycles;
  - bus arbitration with BB or BTT.
- **Snooper** ([ap68060_snoop.sv](rtl/ap68060_snoop.sv)): invalidates lines in both caches for snooped transfers by another master. It keeps a copy of the data cache tags, so a snoop miss costs no processor cycles.
- **Microcode** ([tools/ucode.py](tools/ucode.py)): generates the decoder PLA, the microcode ROM and the attribute tables under [rtl/gen/](rtl/gen/). The generated files are committed. `tb/build_sim.sh` regenerates them on every build.

## What is implemented

The full list, with MC68060UM section references, is in
[doc/STATUS.md](doc/STATUS.md). In summary:

- **Programming model**:
  - SR, the single supervisor stack, VBR, SFC/DFC, CACR, TCR, URP/SRP, ITT/DTT, BUSCR and PCR;
  - exception frames $0, $2, $3 and $4, with the 060's trace and interrupt priorities;
  - double bus fault halt (PST $1C).
- **Integer**: the 060 instruction set. The instructions the 060 leaves to software trap to vector 61, as on the real chip:
  - MOVEP, CAS2 and CHK2/CMP2;
  - 64-bit MUL/DIV;
  - misaligned CAS.

  PLPAR/PLPAW, LPSTOP, HALT and PULSE are implemented.
- **FPU**: the 060 hardware subset, with FINT/FINTRZ in hardware. The rest traps as on the real chip:
  - transcendentals, FMOVECR and the conditional FPU instructions go to vector 11;
  - packed, denormal and unnormal operands go to vector 55;
  - FMOVEM forms the 060 does not handle in hardware go to vector 60.

  FSAVE and FRESTORE use NULL, IDLE and EXCP frames.
- **MMU**:
  - 4K and 8K pages, with 64-entry ATCs;
  - table searches that bypass the data cache;
  - U/M updates per Table 4-1;
  - transparent translation and default cache modes;
  - PLPA, PFLUSH and an FSLW per section 8.4.
- **Caches**:
  - 8 KB data and 8 KB instruction caches;
  - copyback and write-through modes, precise and imprecise noncachable accesses;
  - no-allocate modes and half-cache modes;
  - the store buffer with SBE access errors;
  - the branch cache.
- **Bus**:
  - BS3-BS0, SAS, TRA, CLA, BGR and TIP;
  - BB or BTT arbitration;
  - the reset modes read from IPL2-IPL0 and D15-D0 (ignore states, native retries, extra write hold);
  - RSTO;
  - LPSTOP;
  - BUSCR locked sequences;
  - IPL sampled twice, so a pin-skew glitch is ignored;
  - snoop invalidation.

## Software: the 68060 software package

Like a real 68060, the core leaves parts of the architecture to software.
The operating system must install the 68060 software package (Motorola's
M68060SP). On the Amiga that is the `68060.library`; on an Atari it is
FPSP060, as on a CT60. The core traps on:

| Vector | Exception | Raised by |
|---|---|---|
| 61 | Unimplemented integer instruction | MOVEP, CAS2, CHK2/CMP2, 64-bit MUL/DIV, misaligned CAS |
| 11 | Unimplemented FPU instruction (F-line) | Transcendentals, FMOVECR, FScc, FDBcc, FTRAPcc |
| 55 | Unsupported data type | Packed decimal, denormal and unnormal operands |
| 60 | Unimplemented effective address | `#imm` in extended or packed format, dynamic FMOVEM lists, FMOVEM.L `#imm` to several registers |
| 51, 53 | Underflow, overflow | A tiny or overflowing result, *even when FPCR disables the exception*: the package forms the denormalized or default result |

Without the package, these traps are fatal. For example, EmuTOS without
FPSP060 panics with exception 55 the first time a program loads a
denormalized number. A 68881 or 68882 handles denormalized numbers in
hardware, so software written for those may never have hit this.

## Repository layout

| Path | Contents |
|---|---|
| [rtl/](rtl/) | The core. [ap68060.sv](rtl/ap68060.sv) is the top level. |
| [rtl/fpu/](rtl/fpu/) | The FPU |
| [rtl/gen/](rtl/gen/) | Generated decoder and microcode tables (from `tools/ucode.py`) |
| [rtl/fastram/](rtl/fastram/) | DDR3-backed Fast RAM as a 68060-bus slave, for integrations |
| [tb/](tb/) | Program bench, bus slave and alternate bus master models, DDR3 model, build and run scripts |
| [tb/asm/](tb/asm/) | Self-checking test programs (`t_*.s`) and micro-benchmarks (`bench_*.s`) |
| [tb/c/](tb/c/) | Dhrystone 2.1, Whetstone and a small kernel benchmark, with their start-up code |
| [tools/](tools/) | Microcode generator, ISA tables, `cputest` replay generator, reference dumper |
| [syn/](syn/) | Quartus out-of-context fit scripts and timing reports |
| [doc/](doc/) | [STATUS.md](doc/STATUS.md) (features, deviations, performance, area history) and the audit reports |

## Requirements

- **Verilator 5.x**: development uses 5.052.
- **Python 3**.
- **vasm** (`vasmm68k_mot`) for the test programs. The scripts default to `/opt/amiga-cc/vbcc/bin/`; override with `VASM=` or `VBCC=`.
- **vbcc** for the C benchmarks.
- **Quartus Prime** for fits. The scripts have been used with 17.0 Standard Edition.
- Optional: the WinUAE `cputest` 68060 corpus, for the corpus replay.

Builds put their objects under `$WORK`, which defaults to `obj/` in the repository.

## Quick start

```sh
cd tb

# build the program bench (Verilator) and assemble the test programs
./build_sim.sh
./build_asm.sh smoke t_integer t_fpu

# run one program; it reports PASS or FAIL with the failing test number
../obj/obj_prog/tb_ap68060 +prog=build/t_integer.hex

# the same, with wait states and BCLK at half CLK
../obj/obj_prog/tb_ap68060 +prog=build/t_integer.hex +bclk2 +waits
```

## Verification

### Regression

```sh
cd tb && ./run_tests.sh            # every program under every bus configuration
cd tb && ./run_tests.sh t_fpu      # selected programs
```

`run_tests.sh` assembles the programs, builds the simulator and runs each
program under 11 bus configurations:

- zero wait states;
- random wait states;
- burst inhibit;
- retries;
- BCLK at half CLK;
- ignore states with junk terminations;
- native TRA retries;
- extra write hold;
- CLA;
- BGR;
- IPL pin skew.

It also runs the fetch unit's TCI bench (`run_fetch_tci.sh`). The run
fails on any FAIL or simulator error.

Beyond each program's own checks, the bench checks the following on every transfer:

- **Bus protocol**: BS3-BS0, SAS timing, the A3:A2/CLA rule and the write hold.
- **Bus ownership**: the processor must not drive the bus while another master owns it, while RSTO is asserted, or after the LPSTOP broadcast.
- **Locked sequences**: no other master's transfer may fall inside a locked sequence.
- **Halt**: a halted processor must be quiet.
- **Snooping**: no snoop invalidation may be dropped, and the snooper's tag copy must agree with the data cache.

The programs cover:

- **Integer and exceptions**: `t_integer`, `t_exceptions`, `t_early`, `t_loops`, `t_halt`, `t_xnest`.
- **MMU**: `t_mmu`, `t_mmu060`, `t_atcprobe`, `t_bitfield_mmu`, `t_movem_restart`, `t_moves_fc`.
- **Caches and store buffer**: `t_cache`, `t_cache060`, `t_cmodes`, `t_cbsplit`, `t_bitfield_cache`, `t_stbuf`, `t_stld`.
- **FPU**: `t_fpu`, `t_fpu_frames`.
- **Bus, snooping and interrupts**:
  - `t_pins`, `t_berr`, `t_eredir`, `t_bsrfault`, `t_buslock`;
  - `t_snoop`, `t_snstress`, `t_snirq`;
  - `t_irq7`, `t_iplglitch`;
  - `t_btb`.
- **Fast RAM**: `t_fastram`.

### Bench interface

Programs are loaded at address 0 from `+prog=<hex>`. They report through
memory-mapped registers:

- `$F100`: the failing test number;
- `$F102`: `$600D` for a pass.

Other registers inject faults and stimulus:

- one-shot bus errors on chosen accesses;
- interrupt levels and timed IPL changes;
- alternate-master and snoop traffic;
- CDIS and MDIS;
- timed resets.

They are documented at the top of [tb/tb_ap68060.sv](tb/tb_ap68060.sv).
Useful plusargs:

- `+waits`: random wait states;
- `+bclk2`: BCLK at half CLK;
- `+tbi=<n>`: burst inhibit;
- `+retry=<n>`: retries;
- `+ign=<counts>` and `+ignjunk`: ignore states, with junk terminations;
- `+native`: native TRA retries;
- `+whold`: extra write hold;
- `+cla`, `+bgr`: CLA and BGR;
- `+iplskew=<n>`: IPL pin skew;
- `+trace`: a retirement trace;
- `+timeout=<cycles>`.

### FPU pipeline differential

```sh
cd tb && ./build_fpu_pipe.sh && ../obj/obj_fpu_pipe/tb_fpu_pipe +seed=13 +cmds=100000
```

This bench runs the pipelined FPU (`FPU_PIPE=1`) and the sequential one
side by side on random command streams, and compares them.

### WinUAE cputest corpus

```sh
cd tb && ./build_cputest.sh
python3 run_cputest.py --full --jobs 16 <corpus>                        # every slice
python3 run_cputest.py --full --group BasicFPU --jobs 16 <corpus>       # one group
python3 run_cputest.py --full --group AE --instruction 'Bcc.*' <corpus> # one instruction
```

The replay runs each corpus slice on the core through a small monitor
program, [tb/asm/cputest_mon.s](tb/asm/cputest_mon.s), and compares the
results round by round. It needs the WinUAE `cputest` 68060 corpus, either
the zip file or its extracted directory.

The known oracle deviations are listed in [doc/STATUS.md](doc/STATUS.md).
In each one the core follows MC68060UM:

- the frame format for denormal and unnormal FPU source operands;
- the frame address for an unimplemented FPU instruction with an extended `-(An)` operand;
- the effects of trace exceptions inside traced multi-instruction rounds.

### Benchmarks

```sh
cd tb && ./run_bench.sh            # Dhrystone 2.1 and Whetstone, self-checking
```

Both benchmarks are built with vbcc for the 68060. The start-up code enables
user mode, copyback data caching, all caches, the store buffer and the
branch cache. Each benchmark runs at two iteration counts, and the
difference gives the steady-state cost.

## Synthesis

```sh
cd syn
PERIOD=16.667 ./fitcore.sh mybuild     # 60 MHz; the default PERIOD is 15.385 (65 MHz)
```

`fitcore.sh` makes every core port a virtual pin and runs a full Quartus
compile. It writes the following into `syn/<name>/`:

- `ap68060.fit.summary`: area;
- `ap68060.sta.summary`: timing for all corners;
- `paths.txt`: the worst setup paths.

Set `OPTMODE` to change the Quartus optimization mode, and `R=<dir>` to fit
another RTL tree.

Close timing structurally, by changing the logic. Do not sweep fitter seeds.

### Area options

These are parameters of `ap68060`, all 0 by default. Their costs are in
[doc/STATUS.md](doc/STATUS.md).

| Parameter | Effect when 1 |
|---|---|
| `FPU_PIPE` | The FPU's arithmetic pipeline (about 990 ALMs; FPU-heavy code about 7% faster) |
| `LOOP_PRED` | D1's loop exit predictor |
| `EARLY_RES` | Early AG results for younger uops' address registers |
| `DC1_ATC` | DC1's own ATC port and tag compare (shorter paths, for 65 MHz) |
| `SKID` | A skid register in front of AG (shorter paths, for 65 MHz) |
| `PCR_REV` | The revision number read from PCR (default `8'h06`) |

The bench passes these through defines such as `AP_FPU_PIPE`, `AP_SKID` and
`AP_DC1_ATC`, for example:

```sh
DEFS=+define+AP_FPU_PIPE=1 ./build_sim.sh
```

## Integrating the core

`ap68060` exposes the 68060's pins with the bidirectional ones split into
`_i`, `_o` and `_oe` signals:

- `clk` is the processor clock (CLK).
- `bclk_en` is CLKEN, and marks the CLK edges that are also bus clock (BCLK) edges.
- `a_i`, `ts_n_i` and `tt_i` carry another master's transfer, for snooping with `snoop_n`.
- `d_i` is the data bus in both directions.
- `a_oe` covers the address, the transfer attributes, TS and BS3-BS0.

RSTI is synchronized to bus clock edges. After it negates, the processor
stays in reset for another 27 CLK cycles (MC68060UM 7.13). IPL2-IPL0 and
D15-D0 select the special bus modes at reset (7.14). `dbg_pc`,
`dbg_retire` and `dbg_halted` are for debugging.

[rtl/fastram/ap060_fastram.sv](rtl/fastram/ap060_fastram.sv) puts
DDR3-backed Fast RAM on the 68060 bus. It joins three parts through
dual-clock FIFOs:

- a bus slave;
- the AP030 Fast RAM front end: line buffers, an 8 KiB cache and posted writes;
- an Avalon-MM DDR3 master.

The integration supplies the address window decode and the DDR3 word address.

The front end's cache is cleared only at reset. No other master may write
the DDR3 behind it.

## Documentation

- [doc/STATUS.md](doc/STATUS.md): implemented behaviour with manual section references, known deviations and simplifications, performance and area history.
- [doc/](doc/): the audit reports, each a review of one area against MC68060UM and WinUAE, with its findings and fixes.

## License

GNU General Public License, version 2 or later: see [LICENSE](LICENSE). The FPU and
the Fast RAM front end come from the author's earlier AP68040 and AP68030
cores.
