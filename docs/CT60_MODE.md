# CT60 mode for the ap060 build - plan

Status: milestones 0, 1 and 3 implemented (2026-10-13), milestone 2 (the
board) open, 4 and 5 not started.  Applies to the ap060 variant
(`FALCON_BRINGUP="CPU060 ..."`, the AP68060 on `falcon_cpubus060`).

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
  off --ttram 64`) on the same image.
- Bus speed (milestone 3, `tb/bridge060`, clocks from TS to the last TA,
  16 transfers each, DDR3 model latencies): half-speed bus ST-RAM long
  20.1, line 45.0; CT60 mode ST-RAM long 21.1, line 47.1 (`FALCON_WAIT` 4,
  `FALCON_BEAT_WAIT` 2: never faster than today), Fast RAM long 16.4, line
  35.1.  The bench checks both relations.
- `releases/Falcon-060ct60_20261013.rbf` (060, no DSP, AP68060 `3a8b0e5`):
  34,933 ALMs (83 %), timing met (HDMI +0.149 ns, core +9.07 ns).

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
| Today every 060 transfer runs at BCLK = CLK/2 (16 MHz) | `rtl/falcon/falcon_system.sv` (`cpu060_bclk`) |
| The arbiter's CPU port takes `cpu_addr[23:2]`; guest memory is DDR3 byte `0x30000000 + address` (`DDR_BASE` = word `0x6000000`) | `rtl/falcon/falcon_memarb.sv` |
| DDR3 use: the scaler at `0x20000000`; ST-RAM, ROM and cartridge at `0x30000000-0x30FFFFFF`; the FPU mailbox at `0x30E90000` (030 builds only); nothing in `tools/` uses DDR3 above `0x30FFFFFF` | `sys/sys_top.v` (`RAMBASE`), `falcon_memarb.sv`, `tools/falcon_fpu/falcon_fpu.c` |
| OSD status bits 0-10 are in use; 11 and up are free | `Falcon.sv` `CONF_STR` |
| Area: the 060 build without the DSP uses 39,127 of 41,910 ALMs; the 060 + DSP build uses 4,191 of 4,191 LABs (no room) | `releases/Falcon-060_20261012d.rbf`, `releases/Falcon-060dsp_20261013.rbf` fit reports |
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

### Bus speed (a later milestone)

On a CT60 the 060 bus runs at full speed, Fast RAM is answered at that
speed and Falcon accesses go through the CPLD with wait states.  Today the
ap060 bus runs at 16 MHz for everything, so Fast RAM would be more memory
but no faster than ST-RAM (both are in the same DDR3 behind the same
arbiter).  Milestone 3 runs the bus at 32 MHz (BCLK = CLK) and keeps
Falcon-space transfers at their current length with a minimum-length
counter in the bridge; Fast RAM transfers end as soon as the arbiter
answers.  The bridge already latches every answer for the bus clock, so
this is a wait-state count, not a redesign.  Measured before and after with
the same program run from ST-RAM and from Fast RAM.

## Area today

The 060 + DSP build (`releases/Falcon-060dsp_20261013.rbf`, AP68060
`b3bc1ea`, synthesized for area) uses 41,474 of 41,910 ALMs (99 %) and all
4,191 LABs.  From its fit report (ALMs needed, sub-blocks included):

| Block | ALMs | % of device |
|-------|-----:|------------:|
| **Falcon system** | **34,170** | **81.5** |
| AP68060 | 18,927 | 45.2 |
| DSP56001 | 6,698 | 16.0 |
| FDC (WD1772 + DMA) | 1,250 | 3.0 |
| Crossbar (DMA sound, codec) | 1,238 | 3.0 |
| Videl | 1,216 | 2.9 |
| IDE | 1,010 | 2.4 |
| SCSI | 863 | 2.1 |
| ACIA + IKBD | 740 | 1.8 |
| Blitter | 426 | 1.0 |
| MFP | 392 | 0.9 |
| PSG | 317 | 0.8 |
| 060 bus bridge (`falcon_cpubus060`) | 299 | 0.7 |
| NVRAM | 262 | 0.6 |
| Arbiter, combel, glue | 234 | 0.6 |
| **hps_io** (OSD, disks, ioctl) | 732 | 1.7 |
| Other `emu` logic | 183 | 0.4 |
| **MiSTer framework** | **6,389** | **15.2** |
| Scaler (`ascal`) | 2,147 | 5.1 |
| Audio output (IIR filter 443) | 925 | 2.2 |
| HDMI PLL adjust | 494 | 1.2 |
| HDMI OSD, VGA OSD | 480, 478 | 2.3 |
| Rest of `sys_top` | 1,865 | 4.5 |

Inside the AP68060:

| Unit | ALMs |
|------|-----:|
| Back end (`be`) | 9,218 |
| - FPU interface and FPU (`fpif`) | 4,028 |
| - the back end's own logic | 3,494 |
| - ALUs, multiply/divide, register files | about 1,660 |
| Data cache / DMU | 3,725 |
| Fetch unit (two predecoders, about 615) | 2,516 |
| Decoder | 1,744 |
| Microsequencer | 1,252 |
| Snoop and bus interface | 463 |

The DSP is one block: `u_core` has 5,329 ALMs of its own, its ALU 680,
its two decoders 120.  CT60 mode adds the 32-bit decode and a wider CPU
port to the arbiter (expected well under 200 ALMs), so it fits the 060
build without the DSP (39,127 ALMs) but not the 060 + DSP build without
logic out first (milestone 4).

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
2. **Board build and test.**  The 060 build without the DSP (the 060 + DSP
   build has no free LABs).  On the board: EmuTOS reports the Fast RAM; a
   memory test over all of it; MiNT boots and uses it.
3. **Full-speed bus.**  BCLK = CLK, Falcon-space wait states as above;
   timing measured from ST-RAM and Fast RAM; the regressions again.
4. **060 + DSP with CT60 mode.**  Needs logic out first: the DSP core
   decodes the same instruction-class field in four `case` blocks
   (`dsp56k_core.sv`), and the AP68060 fetch unit has two predecoders
   (about 615 ALMs, in the AP68060 repository).
5. **CT60 TOS (optional).**  Only with the CT60 CPLD register map and the
   CT60 TOS sources: the 1 MB flash image at its address, the registers
   the boot code programs, the cookie and XBIOS.

## Open items

- The CT60 CPLD register map and flash address (for milestone 5): the
  [CT60 TOS archive](https://atariuptodate.de/en/36/ct60-tos-archive) and
  the [CT60lib](https://atariuptodate.de/en/6152/ct60lib) are the places to
  look; the wiki pages found have no addresses.
- Whether the arbiter should give the CPU a separate, faster path to Fast
  RAM (the AP68060 repository's `rtl/fastram`, written for the Minimig,
  streams a line at one beat per clock behind its own DDR3 port).  It needs
  a second DDR3 arbiter and more area; decide after milestone 3's
  measurements.
