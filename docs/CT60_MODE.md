# CT60 mode for the ap060 (and ap040) builds

Status: milestones 0, 1, 3 and 4 implemented (2026-10-13; milestone 3 redone
2026-10-16: the processor at 50 MHz, Fast RAM on its own bus; milestone 4,
the 060 + DSP with CT60 mode, 2026-10-16), milestone 2 (the board) open, 5
not started.  Applies to the ap060 variant (`FALCON_BRINGUP="CPU060 ..."`,
the AP68060 on `falcon_cpubus060`) and to the ap040 variant (branch
`ap040`, `FALCON_BRINGUP="CPU040 ..."`, the AP68040 on `falcon_cpubus040`
with the same Fast RAM).

Results so far:

- DDR3 `0x31000000-0x3FFFFFFF` is free: the Falcon Main code maps no DDR3,
  the FPU service uses `0x30E90000`, the scaler `0x20000000`, the Linux
  framebuffer `0x22000000`; Main's save-state area `0x3E000000` is for GBA
  and cores that declare save states only.  All three sizes are offered.
- `tb/bridge060`: every range of the address map for each size (long and
  line, read and write), Fast RAM at DDR3 `0x31000000` with ST-RAM
  untouched, bus errors after the end, in the hole and at `$80000000`, the
  `$FF000000` mirror; Fast RAM off keeps the 24-bit mirror.
- EmuTOS 1.3 in the full-system simulation with 64 MB Fast RAM: welcome
  screen "Alt-RAM: 64 MB"; `ramtop`/`ramvalid` (`$5A4`) = `$05000000`,
  `$1357BD13`, the same as Hatari (`--machine falcon --cpulevel 6 --addr24
  off --ttram 64`) on the same image.  Again with the processor at 50 MHz
  and Fast RAM through `ap060_fastram` (2026-10-16): the same welcome
  screen and `ramtop`; with Fast RAM off "ST-RAM: 14 MB", no Alt-RAM,
  `ramtop` 0; the bus errors are EmuTOS's probes, as before.
- Bus speed (milestone 3, `tb/bridge060`, 50 MHz clocks from TS to the last
  TA, 16 transfers each, DDR3 model latencies): ST-RAM long 35.4, line
  66.7 (0.71 and 1.33 us: the request crosses to the 32 MHz side and back);
  Fast RAM cache hit long 5.0, line 8.0 (0.10 and 0.16 us), miss long 35.8,
  line 36.7.  The bench checks that hits take under a quarter of an ST-RAM
  read and that a missed line is faster than an ST-RAM line.
- `releases/Falcon-060ct60_20261013.rbf` (060, no DSP, AP68060 `3a8b0e5`):
  34,933 ALMs (83 %), timing met (HDMI +0.149 ns, core +9.07 ns).
- `releases/Falcon-060all-50_20261016.rbf` (060 + FPU + DSP + CT60 Fast
  RAM, the 50 MHz processor): 41,474 ALMs (99 %), timing met (50 MHz
  +0.441 ns, 32 MHz +2.309 ns, HDMI +0.029 ns).  The repository's
  `scripts/build.sh` reproduces it with HDMI at -0.181 ns (placement).
- ap040 (2026-10-17): `tb/bridge040` (83 checks: `tb/bridge060`'s, plus
  the 68040's snoop handshake, every DMA line snooped once as an SC = 10
  write and acknowledged only with MI negated); EmuTOS in the full-system
  simulation reports "CPU type: M68040" and, with Fast RAM on, "Alt-RAM:
  64 MB", `ramtop` `$05000000`.  `releases/Falcon-040_20261016.rbf` (no
  DSP): 35,847 ALMs (86 %), timing met; `releases/Falcon-040all_20261017.rbf`
  (everything, AP68040 `ab4d83d`): 40,713 ALMs (97 %), 50 MHz +1.423 ns,
  32 MHz +2.017 ns, HDMI -0.228 ns.

The CT60 is a 68060 accelerator for the Falcon.  To software it is a Falcon
whose CPU is a 68060 on a 32-bit bus, with Fast RAM (TT-RAM) on the 060's
own bus.  CT60 mode makes the ap060 build look like that, so EmuTOS and
FastRAM-aware software (MiNT, `Mxalloc` users, large programs) get up to
240 MB of Fast RAM.

## Goal and non-goals

Goal: EmuTOS on the ap060 build detects a 32-bit bus and Fast RAM at
`$01000000`, reports it as Alt-RAM, and uses it; everything that works on
the ap060 build today keeps working with Fast RAM off.

Not in this plan:

- CT60 TOS (TOS 4.04 with the CT60 patches).  It boots from the CT60's own
  1 MB flash and programs the CT60 CPLD's registers (SDRAM controller,
  clock module, temperature sensor).  No register map was found (see
  "Open items").  EmuTOS uses none of it.
- The `_CT60` cookie and the CT60 XBIOS calls (installed by CT60 TOS).
- CTPCI, SuperVidel, EtherNAT, the CTCM clock module.
- The 060/030 switch: on MiSTer that is the two builds (030 and 060); both
  CPUs do not fit in one core.

## Facts this rests on

| Fact | Source |
|------|--------|
| EmuTOS decides the address bus is 32-bit by writing `$0000000C` and reading `$0100000C`: the same value there means a 24-bit bus (a mirror); a different value or a bus error means 32-bit | EmuTOS `bios/memory.S` `_detect_32bit_address_bus` |
| EmuTOS looks for TT-RAM only on a 32-bit bus, from `$01000000` in 1 MB steps: it reads the last byte of each MB (stops at a bus error), writes two patterns and reads them back (stops on a mismatch or a mirror of the first MB), only at positive addresses | EmuTOS `bios/memory2.c` `detect_ttram_size`, `include/tosvars.h` `TTRAM_START` |
| A warm reset keeps the detected size (`ramvalid`) and only rechecks the first and last byte; a size change needs a cold reset | EmuTOS `bios/memory2.c` `ttram_detect` |
| With TT-RAM present EmuTOS does not use the blitter (24-bit DMA), "e.g. a CT60 in 68060 mode" | EmuTOS `bios/machine.c` `detect_blitter` |
| EmuTOS's welcome screen shows found TT-RAM as "Alt-RAM" | EmuTOS `bios/initinfo.c` |
| CT60: one PC133 SDRAM module, 64-512 MB, "accessed as FAST RAM (TT RAM) at full system speed"; the 060 bus runs at 80-100 MHz; 1 MB flash with boot, 060 init and patched TOS 4; a switch selects 060 or "100% Falcon 030 mode" | [AtariForumWiki: CT60](https://temlib.org/AtariForumWiki/index.php/CT60) |
| Hatari runs a Falcon with a 68060 and TT-RAM (`--machine falcon --cpulevel 6 --addr24 off --ttram N`, N up to 1024 MB in steps of 4) | Hatari `src/options.c` |
| The ap060 bridge decodes A23-A0 only: `$01000000+` mirrors `$000000+`, so EmuTOS sees a 24-bit bus | `rtl/falcon/falcon_cpubus060.sv` (`la <= a[23:0]`) |
| Before milestone 3 every 060 transfer ran at BCLK = CLK/2 (16 MHz); now the processor runs at 50 MHz with BCLK = CLK | `rtl/falcon/falcon_system.sv`, `rtl/pll/pll_0002.v` (`outclk_1`) |
| The arbiter's CPU port takes `cpu_addr[23:2]`; guest memory is DDR3 byte `0x30000000 + address` (`DDR_BASE` = word `0x6000000`) | `rtl/falcon/falcon_memarb.sv` |
| DDR3 use: the scaler at `0x20000000`; ST-RAM, ROM and cartridge at `0x30000000-0x30FFFFFF`; the FPU mailbox at `0x30E90000` (030 builds only); nothing in `tools/` uses DDR3 above `0x30FFFFFF` | `sys/sys_top.v` (`RAMBASE`), `falcon_memarb.sv`, `tools/falcon_fpu/falcon_fpu.c` |
| OSD status bits 0-10 are in use; 11 and up are free | `Falcon.sv` `CONF_STR` |
| Area before milestone 4: the 060 build without the DSP used 39,127 of 41,910 ALMs; the 060 + DSP build 4,191 of 4,191 LABs (no room) | `releases/Falcon-060_20261012d.rbf`, `releases/Falcon-060dsp_20261013.rbf` fit reports |
| The simulator's DDR3 model holds 16 MB (`mem[0:(1<<21)-1]`, 64-bit words) | `tb/system/ddr3_model.sv` |

## Design

### Address map (the bridge, A31-A0)

| Guest range | Fast RAM off (today) | Fast RAM on (CT60 mode) |
|-------------|----------------------|-------------------------|
| `$00000000-$00FFFFFF` | Falcon | Falcon |
| `$01000000` to `$01000000 + size - 1` | mirror of the Falcon | Fast RAM |
| `$01000000 + size` to `$FEFFFFFF` | mirror of the Falcon | bus error (TEA) |
| `$FF000000-$FFFFFFFF` | mirror of the Falcon | mirror of the Falcon |

- With Fast RAM off nothing changes: A31-A24 stay ignored, exactly as now.
  The 32-bit decode is switched on with Fast RAM, so 24-bit software that
  keeps flags in the top address byte only breaks when the user asks for
  Fast RAM (it breaks on a real CT60 too).
- `$FF000000-$FFFFFFFF` must keep reaching the Falcon: TOS and programs use
  absolute short addresses (`$FFFF8240.w` is `$FFFF8240`), and EmuTOS
  defines its I/O registers there (`0xffff8201` `VIDEOBASE_ADDR_HI`,
  `0xffff8260` `ST_SHIFTER`, ...).
- The bus error above the Fast RAM is how EmuTOS finds its size.
- Supervisor protection (`$000000-$0007FF`, I/O) and the reset-vector
  mapping stay as they are for the `$00xxxxxx` and `$FFxxxxxx` windows.

### Where Fast RAM lives

Guest `$01000000-$0FFFFFFF` maps to DDR3 `0x31000000-0x3FFFFFFF` with the
same formula as ST-RAM (`0x30000000 + address`).  Sizes: 64, 128 or 240 MB
(240 MB reaches the end of DDR3 at `0x3FFFFFFF`).

- The arbiter's CPU port widens to `cpu_addr[27:2]`; video, the loader and
  the DMA ports (sound, disk, blitter) stay 24-bit.  The DMA rules are
  unchanged: a DMA master cannot reach Fast RAM, as on a real CT60, so Fast
  RAM is never written behind the CPU and needs no snooping.
- The CPU's line reads use the existing two-word path.
- Fast RAM is not cleared at reset (EmuTOS tests what it uses).

### OSD

`O[12:11],Fast RAM,Off,64 MB,128 MB,240 MB;` - present only in 060 builds
(`ifdef FALCON_CPU060` around the line in `CONF_STR`).  The size is taken
at reset; changing it needs a cold reset (EmuTOS keeps its old detection
across a warm one).

### Bus speed (milestone 3)

On a CT60 the 060 bus runs at full speed, Fast RAM is answered at that
speed and Falcon accesses go through the CPLD with wait states.  In the
ap060 build the processor runs on its own 50 MHz clock (`clk_cpu`, PLL
output 1, asynchronous to the 32 MHz system clock) with BCLK = CLK:

- Fast RAM is the processor's own: `ap060_fastram` (the AP68060
  repository's `rtl/fastram`, as on the Minimig) answers the Fast RAM
  window on the 060 bus at 50 MHz, from two line buffers and an 8 KiB
  cache, with posted writes; its DDR3 side runs on the system clock and
  shares the one DDRAM port with the Falcon arbiter through
  `falcon_ddr_arb` (Fast RAM first, a read's beats or a write burst never
  split).  Its 64-bit words hold 16-bit big-endian words, the lowest
  address in bits 15:0 (the Minimig layout); only the processor reads
  them.
- Everything else crosses to the system clock in `falcon_cpubus060`: one
  request toggle and one answer toggle through three-flop synchronisers,
  with the address, size, lanes and data held while they cross; the
  Falcon side runs the transfer as before.  DMA snoops go the other way
  through a 32-entry dual-clock FIFO; `snoop_hold` stops DMA writes at 28.
- The Fast RAM front end's cache is written through by every processor
  write and swept by a processor reset.  Nothing else writes Fast RAM
  (the Falcon's DMA reaches ST-RAM only).
- `Falcon.sdc` puts `clk_cpu` in its own asynchronous clock group.

## Area today

The 060 + DSP build with CT60 mode and the 50 MHz processor
(`releases/Falcon-060all-50_20261016.rbf`, AP68060 `3a8b0e5`) uses 41,474
of 41,910 ALMs (99 %).  From its fit report (ALMs needed, sub-blocks
included):

| Block | ALMs | % of device |
|-------|-----:|------------:|
| **Falcon system** | **33,888** | **80.9** |
| AP68060 | 18,948 | 45.2 |
| DSP56001 | 5,815 | 13.9 |
| FDC (WD1772 + DMA) | 1,277 | 3.0 |
| Videl | 1,204 | 2.9 |
| IDE | 1,083 | 2.6 |
| Crossbar (DMA sound, codec) | 1,074 | 2.6 |
| 060 bus bridge with `ap060_fastram` (775) | 1,024 | 2.4 |
| SCSI | 865 | 2.1 |
| ACIA + IKBD | 715 | 1.7 |
| Blitter | 414 | 1.0 |
| MFP | 378 | 0.9 |
| PSG | 291 | 0.7 |
| NVRAM | 247 | 0.6 |
| Arbiter | 206 | 0.5 |
| Microphone ADC | 132 | 0.3 |
| **hps_io** (OSD, disks, ioctl) | 735 | 1.8 |
| **MiSTer framework** | | |
| Scaler (`ascal`) | 2,201 | 5.3 |
| Audio output | 961 | 2.3 |
| HDMI PLL adjust | 489 | 1.2 |
| HDMI OSD, VGA OSD | 510, 475 | 2.4 |

Inside the AP68060:

| Unit | ALMs |
|------|-----:|
| Back end (`be`) | 9,237 |
| - FPU interface and FPU (`fpif`) | 4,093 |
| Data cache / DMU | 3,622 |
| Fetch unit | 2,578 |
| Decoder | 1,785 |
| Microsequencer | 1,262 |
| Snoop, bus interface | 225, 229 |

What made it fit (milestone 4; each change checked against Hatari or the
previous RTL by the existing benches):

- DSP core: one accumulator write for the ALU, DIV, NORM and Tcc; two
  data register write ports for the parallel moves; the first move's
  data from the shared register read; the XY move's AGU for its four
  modes; one adder for the second fetch address; the ALU's E/U/N/Z once
  (-1,064 ALUTs; 5,000 random seeds against Hatari give the original
  RTL's result).  The DSP peripherals' backing store tags its entries
  with a reset generation instead of 128 valid bits, and holds PCDDR,
  PCD, SCR, SSR, SCCR and BCR.  The DSP went from 6,698 to 5,815 ALMs.
- Crossbar: the DMA record write queue in block RAM; the bridge's line
  buffers in dual-clock block RAM; the clock enable accumulators of the
  MFP, PSG, Videl and ACIA reduced by their gcd, the crossbar's to 26
  bits; the arbiter's DMA word and address selected once.
- `scripts/build.sh` for CPU060/CPU040: no physical synthesis in the
  processor (its register duplication and retiming for 50 MHz cost about
  1,400 ALMs, and 50 MHz is met without), the scaler optimized for speed
  with physical synthesis, small RAMs kept out of MLABs, routability
  optimization always, no redundant-cell removal (with it the scaler's
  HDMI counter paths miss timing at this density).

## Milestones

0. **DDR3 check.**  Confirm that nothing on the ARM side (Linux, our
   Main_MiSTer, the scaler at its largest mode) uses DDR3
   `0x31000000-0x3FFFFFFF`; pick the largest safe size.
1. **32-bit decode and Fast RAM.**  `falcon_cpubus060.sv` (decode, size
   input), `falcon_memarb.sv` (28-bit CPU port), `falcon_system.sv` and
   `Falcon.sv` (OSD option, `ifdef FALCON_CPU060`).
   Tests (Verilator, real RTL):
   - `tb/bridge060`: every range of the table above for byte/word/long/line
     reads and writes, bus errors above the size, the `$FFxxxxxx` mirror,
     Fast RAM off = today's mirroring; all sizes.
   - `tb/integration` (`tb_memarb`): CPU accesses above 16 MB land at DDR3
     `0x31000000 + offset`; DMA writes above the RAM top still bus-error.
   - `tb/system`: the DDR3 model grows (sparse, or 256 MB) and EmuTOS
     boots with Fast RAM 64 MB; its welcome screen shows Alt-RAM 64 MB.
     Reference: Hatari with `--machine falcon --cpulevel 6 --addr24 off
     --ttram 64` on the same EmuTOS image.
   - The existing 060 regressions (EmuTOS boot from SCSI, 124 Beers) with
     Fast RAM off: unchanged frames.
2. **Board build and test.**  The 060 build with or without the DSP, and
   the ap040's.  On the board: EmuTOS reports the Fast RAM; a
   memory test over all of it; MiNT boots and uses it.
3. **Full-speed bus.**  The processor at 50 MHz, BCLK = CLK, Fast RAM on
   its bus (above); timing measured from ST-RAM and Fast RAM; the
   regressions again.
4. **060 + DSP with CT60 mode.**  Done (2026-10-16): the area work and
   build settings above; `releases/Falcon-060all-50_20261016.rbf`.
5. **CT60 TOS (optional).**  Only with the CT60 CPLD register map and the
   CT60 TOS sources: the 1 MB flash image at its address, the registers
   the boot code programs, the cookie and XBIOS.

## Open items

- The CT60 CPLD register map and flash address (for milestone 5): the
  [CT60 TOS archive](https://atariuptodate.de/en/36/ct60-tos-archive) and
  the [CT60lib](https://atariuptodate.de/en/6152/ct60lib) are the places to
  look; the wiki pages found have no addresses.
- The 060 + DSP build has 436 ALMs to spare; anything added to it needs
  area taken out first.  The ap040's has about 1,200 (the AP68040 `faster`
  area round, `effec8b`, and `DC1_ATC` off).
