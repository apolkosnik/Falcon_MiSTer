# Falcon_MiSTer

Atari Falcon030 core for MiSTer (DE10-Nano).

- CPU: [AP68030](https://github.com/apolkosnik/AP68030), a pin-level MC68030
  (caches, PMMU), submodule at `rtl/AP68030`.
- Chipset: written for this core, with the
  [Hatari](https://hatari.tuxfamily.org/) emulator as the behavioural
  reference for every device (each module header names the Hatari functions it
  follows and lists its deviations).
- Platform: the MiSTer template (`sys/`).

Status: untested on hardware. In full-system simulation (Verilator, the real
RTL, EmuTOS 1.3) the machine boots to the GEM desktop from floppy, IDE and
SCSI disks, with keyboard and mouse input, matching Hatari's boot frame by
frame.  The complete core fits (92% of the logic) and meets timing at 32 MHz.

## What is in the FPGA

| Part | Module | Notes |
|------|--------|-------|
| 68030 | `rtl/AP68030` | 32 MHz core clock (twice a stock Falcon) |
| 68882 FPU | `falcon_fpu_bridge`, `tools/falcon_fpu` | the coprocessor interface in the FPGA, the arithmetic on the ARM (Hatari's 68882 emulation): `docs/FPU_ARM.md` |
| Bus, decode, interrupts | `falcon_system`, `falcon_cpubus` | Falcon 24-bit map, Hatari's bus-error rules incl. STE-compatible bus mode |
| ST-RAM, ROM | `falcon_memarb` | 4 or 14 MB in the HPS DDR3 (no SDRAM board needed) |
| COMBEL | `falcon_combel` | $FF8001/6/7, DIP switches, Jaguar pads |
| Videl | `falcon_videl` | all ST/Falcon modes, VGA and RGB/TV timing from the registers, both palettes |
| DSP56001 | `rtl/falcon/dsp` | 16 MIPS, 32K words, host port, SSI; matches Hatari instruction by instruction and cycle by cycle |
| DMA sound, crossbar, codec | `falcon_crossbar` | play/record, multi-track, DSP links incl. handshake mode |
| YM2149 | `falcon_psg` | Hatari's volume table |
| MFP 68901 | `falcon_mfp*` | timers, interrupts, USART (MiSTer UART) |
| ACIAs + IKBD | `falcon_acia`, `falcon_ikbd*` | PS/2 keyboard and mouse, joysticks, MIDI on the UART |
| Blitter | `falcon_blitter` | matches Hatari's blitter.c |
| NVRAM/RTC | `falcon_nvram` | clock from the MiSTer |
| Floppy | `falcon_fdc` | ST DMA chip + WD1772, `.ST` images, two drives |
| IDE | `falcon_ide` | master and slave |
| SCSI | `falcon_scsi` | NCR 5380 behind the DMA chip, IDs 0/1 disks, ID 2 CD-ROM |

## Using it

1. Copy `releases/Falcon_<date>.rbf` to `_Computer/` on the SD card.
2. TOS: put a 512 KB TOS 4.0x or EmuTOS image in `games/Falcon/boot.rom`
   (loaded at every start), or load one from the OSD ("Load TOS").  EmuTOS
   (`etos512*.img`) is free and boots from IDE and SCSI without a driver.
3. Disks from the OSD: floppy A/B (`.ST`), IDE master/slave and SCSI 0/1
   (`.vhd`, `.img`, `.hdf`), SCSI 2 CD-ROM (`.iso`, `.cue`/`.bin`).
4. Monitor: OSD "Monitor" (VGA, RGB, TV, mono) is what TOS sees in $FF8006.

### Main_MiSTer

SCSI target responses (INQUIRY, READ CAPACITY, MODE SENSE, CD-ROM TOC) are
built on the ARM by a Falcon module in Main_MiSTer (`support/falcon`, branch
`falcon`), using the block "window" protocol of the NeXT-Color core.  With a
stock Main, SCSI hard disks still work from built-in default responses; the
CD-ROM needs the Falcon Main (`releases/MiSTer_falcon`).  Floppy and IDE work
with any Main.

### 68882 FPU

A 68882 does not fit in the FPGA; the core answers the 68030's coprocessor
interface and the program `falcon_fpu` executes the instructions on the ARM.
Main starts it with the core (patch `main_patch/0003`):

1. Copy `releases/MiSTer_falcon` and `releases/falcon_fpu` to `/media/fat/`
   (falcon_fpu must sit next to the Main binary that runs).
2. In `MiSTer.ini`, section `[Falcon]`: `main=MiSTer_falcon`.

TOS then reports a 68881/68882.  Without `falcon_fpu` (or with another Main)
the core has no FPU, as before.  `/tmp/falcon_fpu.log` holds its messages.
Test programs for the machine: `tools/fputest` (FPUTEST.TOS checks results,
FPUBENCH.TOS measures speed).

## Building

```
git clone --recurse-submodules <this repo>
./scripts/build.sh          # Quartus 17.0.x; prints TIMING MET / FAILED
```

`scripts/build.sh` runs the Quartus stages separately because Quartus 17.0's
fitter can crash while exiting on newer Linux after a complete fit.

## Simulation

- Unit benches: `tb/<module>/run.sh` (Verilator; most compare against Hatari's
  own C code compiled into the bench).
- Full system: `tb/system/build.sh`, then
  `tb/system/obj_dir/Vtb_top +rom=<emutos.hex> --ms 25000 --ide0 disk.img --frames out`
  (see the header of `tb/system/sim_main.cpp`; `rom2hex64.py` converts a ROM).

## Not yet done

- Hardware testing.
- 16 MHz CPU option (the CPU runs from the 32 MHz system clock).
- SCC serial ports (registers only), Centronics printer,
  microphone input, NVRAM saving, `.MSA`/`.STX` floppy images.
