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
| NVRAM/RTC | `falcon_nvram`, `falcon_nvram_store` | MiSTer clock and automatic NVRAM persistence with Falcon Main |
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
interface and the program `falcon_fpu` executes the instructions on the ARM
(Hatari's 68882 emulation; design and measurements in `docs/FPU_ARM.md`).
Main starts it with the core and stops it before another core is loaded:

1. Use a core with the bridge: `releases/Falcon-fpu_20261008.rbf` (it also has
   the NVRAM store below).
2. Copy `releases/falcon_fpu` to `/media/fat/` and make it executable.
3. Use `releases/MiSTer_falcon_fpu.bin` as Main: the configuration-menu Main
   below plus the FPU service (`extra/falcon_fpu_main.patch`, on top of
   `extra/falcon_config_main.patch`).  Copy it as `/media/fat/MiSTer`, or as
   `/media/fat/MiSTer_falcon` with `main=MiSTer_falcon` in the `[Falcon]`
   section of `MiSTer.ini`; `falcon_fpu` must sit in the same directory.

TOS then reports a 68881/68882.  Without `falcon_fpu` (or with another Main)
the core has no FPU, as before.  `/tmp/falcon_fpu.log` holds its messages.
Test programs for the machine: `tools/fputest` (FPUTEST.TOS checks results,
FPUBENCH.TOS measures speed).

### Configuration menu

`releases/MiSTer_falcon_config.bin` adds an Atari ST style main menu for Falcon:

- **Modify config** opens the Falcon settings editor, including TOS, cartridge,
  disks, monitor, ST-RAM, aspect ratio and UART. **Back** returns to the main menu.
- **Save config** stores the current setup in startup slot **0** or slots **1-8**.
- **Load config** restores a saved setup and cold boots with its ROMs and disks.

Profiles are stored in `config/FALCON0.CFG` through `config/FALCON8.CFG`. They
remember the core options, joystick swap, TOS/cartridge paths and all seven disk
slots, including empty drives. Images remain separate files. Slot 0 loads when
the core starts; without a usable slot 0, the usual `boot.rom`/`boot.vhd` startup
continues. Missing or unreadable listed images are rejected before changing the
machine. Later transfer or mount failures are reported; another profile can be
loaded to recover. Saving a config reports success only after the file is saved.

NVRAM remains separate and saves automatically. Loading a profile preserves the
guest's current NVRAM; it does not reset it or reload an older NVRAM file. Use
**Reset NVRAM** to initialize defaults for a different monitor when needed.

The menu and profiles require only the updated Main. Copy the binary to the SD
card as `MiSTer` (keeping a backup of the previous Main), then restart MiSTer.
`extra/falcon_config_main.patch` contains the cumulative Main source changes,
including NVRAM persistence, against Falcon Main commit
`3138b365183cc1de6728ecddc7ec61104267d341`. Apply it to that base instead of also
applying `falcon_nvram_main.patch`. The host regression in the patched Main is
`tests/falcon_config/run.sh`; it tests the production profile module and Falcon
menu cases with file/HPS/OSD adapters and address/undefined-behavior sanitizers.
Hardware testing is still pending.

### NVRAM

NVRAM persistence requires a core built with `falcon_nvram_store` and the matching
Falcon Main (`releases/MiSTer_falcon_config.bin`, or the earlier
`releases/MiSTer_falcon_nvram.bin`). For persistence without the new configuration
menu, use `extra/falcon_nvram_main.patch`. Earlier core binaries do not contain
the persistence controller.

The 50-byte image is restored from `saves/Falcon/falcon.nvram` before TOS loads.
Guest changes are saved automatically after one second without NVRAM writes.
Allow that interval before switching cores or removing power. The file has the
same byte layout as Hatari's `hatari.nvram`; it excludes the running clock.
Failed saves preserve the previous file and retry after five seconds. A save
acknowledgement does not discard changes made while the save was in progress.

Normal resets, cold resets and OSD monitor changes preserve NVRAM. To discard
settings, select **Reset NVRAM** in the OSD; this restores defaults for the
currently selected monitor and saves them. If changing monitor type leaves a
saved video mode unusable, this action restores the appropriate default mode.
A missing file starts with defaults, which are then saved automatically.

Keyboard mappings are unchanged by the persistence update. See
[the Atari ST comparison](docs/keymap-comparison.txt) for the special-key
mapping differences and MiSTer interception behavior.

## Building

```
git clone --recurse-submodules <this repo>
./scripts/build.sh          # Quartus 17.0.x; prints TIMING MET / FAILED
QUARTUS=/path/to/intelFPGA_lite/17.0/quartus ./scripts/build.sh   # another install
```

`scripts/build.sh` runs the Quartus stages separately because Quartus 17.0's
fitter can crash while exiting on newer Linux after a complete fit.  The
project requires Quartus 17.0 (`/opt/intelFPGA_lite/17.0` by default,
`QUARTUS` for another install).  The script deletes `db/` and
`incremental_db/` when they were written by another Quartus version or
edition (which Quartus refuses to open), and leaves `Falcon.qsf` as it was:
Quartus rewrites the project file when it closes it, including the
`FALCON_BRINGUP` macros `files.qip` adds.

## Simulation

- Unit benches: `tb/<module>/run.sh` (Verilator; most compare against Hatari's
  own C code compiled into the bench).
- Persistence: `MAIN=/path/to/Main_MiSTer_falcon tb/nvram_store/run.sh` links
  the production Main save/load implementation to the real `hps_io`, NVRAM
  and persistence-controller RTL, using temporary files for storage.
- Full system: `tb/system/build.sh`, then
  `tb/system/obj_dir/Vtb_top +rom=<emutos.hex> --ms 25000 --ide0 disk.img --frames out`
  (see the header of `tb/system/sim_main.cpp`; `rom2hex64.py` converts a ROM).

## Not yet done

- Hardware testing.
- 16 MHz CPU option (the CPU runs from the 32 MHz system clock).
- SCC serial ports (registers only), Centronics printer,
  microphone input, `.MSA`/`.STX` floppy images.
