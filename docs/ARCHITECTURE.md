# Falcon_MiSTer architecture and module contracts

Atari Falcon030 for MiSTer (DE10-Nano, Cyclone V 5CSEBA6U23I7).

Sources:
- CPU: AP68030 (`rtl/AP68030`, submodule), a pin-level MC68030.
- Behavioural reference for every chipset device: Hatari (`/home/adam/hatari/src`,
  Falcon specifics in `src/falcon/`).  Where Hatari and the hardware
  documentation disagree, the hardware documentation wins and the difference
  is written down in the module header.
- Platform scaffold: the MiSTer template `sys/` (copied from the NeXT core).

Target machine: Falcon030, 68030 at 16 MHz (the core clock is 32 MHz), 14 MB
ST-RAM, TOS 4.0x (or EmuTOS) 512 KB ROM, DSP56001 at 32 MHz, VGA or RGB/TV
monitor.

## Clocks and reset

- `clk_sys` = 32 MHz.  Everything (CPU, chipset, DSP, DDR3 port) runs on it.
  The CPU runs on a clock enable (`cpu_ce`, `falcon_cpuclk`): every clock in
  the OSD's 32 MHz turbo, else 16 or 8 MHz ($FF8007 bit 0) on Hatari's clock
  count, with `falcon_cpubus` giving every bus cycle Hatari's length
  (docs/CPU_TIMING.md).  Devices see the CPU bus in `clk_sys` and must hold
  their answers (`bus_ack`/`bus_berr` pulses are taken by the bridge, which
  holds the CPU while they are late).
  Devices that need slower timebases derive clock enables from it; every
  device takes `parameter CLK_HZ = 32000000` and computes its enables with a
  fractional accumulator when the ratio is not an integer (MFP 2.4576 MHz,
  Videl 25.175 MHz, ...).
- `reset` is synchronous and active high.  It is the MiSTer reset OR the
  CPU's RESET instruction output (`reset_n_oe`), so a RESET instruction resets
  the peripherals and not the CPU, as on the real machine.

## Byte order and lanes

The 68030 is big endian.  All internal 16-bit buses carry the even byte
(lower address, UDS) on bits [15:8] and the odd byte (LDS) on [7:0].  64-bit
video data carries the word at the lowest address on [63:48].  The DDR3 is
little endian; the swap happens only in `falcon_ddram`.

## Memory map (24-bit, the Falcon decodes A23..A0; A31..A24 are ignored)

| Range                | Device                                   |
|----------------------|------------------------------------------|
| 000000-000007        | ROM (first 8 bytes, reset vectors) for reads while `boot_rom_overlay`, RAM afterwards. Writes always go to RAM. |
| 000008-DFFFFF        | ST-RAM (size from OSD: 4 or 14 MB; above size: bus error) |
| E00000-E7FFFF        | TOS ROM 512 KB (read only, writes bus error)  |
| E80000-EFFFFF        | bus error                                 |
| F00000-F0003F        | IDE (Falcon internal IDE)                 |
| FA0000-FBFFFF        | cartridge ROM (read; bus error if no cartridge? reads 0xFFFF) |
| FF8000-FFFFFF        | I/O (see below); unlisted addresses bus error in Falcon bus mode |

Supervisor-only: FF8000-FFFFFF and 000000-0007FF (user accesses bus error).

I/O decode (word address bits [23:1] are given to each device):

| Range          | Module            | IRQ / DMA                     |
|----------------|-------------------|-------------------------------|
| FF8000-FF800F  | `falcon_combel` (memory config $FF8001, monitor $FF8006, bus control $FF8007) | |
| FF8200-FF82C3  | `falcon_videl`    | VBL, HBL, DE (to MFP TBI)     |
| FF8604-FF860F  | `falcon_fdc` (ST DMA + WD1772) | GPIP5 (FDC/HDC)  |
| FF8604 (DMA mode bit 3) | `falcon_scsi` NCR 5380, through the DMA chip in `falcon_fdc` (target responses from Main_MiSTer support/falcon) | GPIP5 |
| FF8800-FF88FF  | `falcon_psg` (YM2149, mirrors every 4 bytes) |   |
| FF8900-FF8943  | `falcon_crossbar` (DMA sound, crossbar, codec) | GPIP7, MFP TAI |
| FF8960-FF8963  | `falcon_nvram` (MC146818)                      |   |
| FF8A00-FF8A3D  | `falcon_blitter`                               | GPIP3 (busy) |
| FF8C80-FF8C87  | SCC 85C30 - stub (reads 0, no bus error)       | IPL5 |
| FF9200-FF9223  | `falcon_combel` (joypads, DIP switches $FF9200) |   |
| FF9800-FF9BFF  | `falcon_videl` (Falcon palette, 256 x 32 bit)  |   |
| FFA200-FFA207  | `falcon_dsp` host interface                    | IPL6 (HREQ) |
| FFFA00-FFFA2F  | `falcon_mfp` (MC68901)                         | IPL6 |
| FFFC00-FFFC07  | `falcon_acia` (IKBD ACIA $FFFC00/2, MIDI ACIA $FFFC04/6) | GPIP4 |

## Interrupts

| IPL | Source | Acknowledge |
|-----|--------|-------------|
| 7   | none (NMI unused) | autovector |
| 6   | MFP IRQ OR DSP HREQ | vectored: MFP vector if the MFP requests, else the DSP's IVR ($FFA203) |
| 5   | SCC (stub: never) | vectored |
| 4   | VBL  | autovector; the request is latched at VBL start and cleared by the acknowledge |
| 2   | HBL  | autovector; latched at line start, cleared by the acknowledge |

MFP GPIP inputs (Falcon):
- I0 printer BUSY (1 = no printer)
- I1 Centronics ACK on the Falcon (idle: 1)
- I2 RS232 CTS (inverted: 1)
- I3 blitter busy (1 while the blitter runs, 0 when it finishes, so the
  default falling-edge interrupt fires at completion; blitter.c).  Hatari
  wires the blitter here for the Falcon too and delivers the DSP's HREQ
  directly on IPL6 (dsp.c notes the real board may use GPIP3); we follow Hatari.
- I4 ACIA IRQ (0 when either ACIA requests)
- I5 FDC/HDC IRQ (0 = request; WD1772 INTRQ via the DMA chip, OR IDE INTRQ)
- I6 RS232 RI (1)
- I7 DMA sound SNDINT (Falcon: the DMA sound "play" signal)
- TAI (timer A input): DMA sound SOUNDINT (frame end event)
- TBI (timer B input): Videl `de_tb`, the display enable once per displayed
  SOURCE line (line-doubled modes count the repeat copy only).  This matches
  Hatari for 200-line modes and mono (400); in 240/480-line modes it counts
  every source line, where Hatari always counts 200 per frame - per-line
  counting is the closer model of the hardware.

## Common device register bus

Every memory mapped device has this port (names exact, so the decoder can
instantiate them uniformly):

```
input             clk,         // clk_sys
input             reset,       // synchronous, active high
input             bus_cs,      // access to this device; held until bus_ack
input             bus_stb,     // one clock pulse on the first clock of bus_cs
input             bus_we,      // 1 = write
input      [N:1]  bus_addr,    // word address within the device (N as needed)
input             bus_uds,     // even byte (bits 15:8) selected
input             bus_lds,     // odd byte (bits 7:0) selected
input      [15:0] bus_din,     // write data (valid while bus_cs)
output     [15:0] bus_dout,    // read data, must be valid on the bus_ack clock
output            bus_ack      // exactly one clock; at the bus_stb clock or later
```

Rules:
- Side effects (register writes, read-to-clear, FIFO pops) happen exactly once
  per access, on `bus_stb` or on the `bus_ack` clock, never on every clock of
  `bus_cs`.
- A device may hold off `bus_ack` for as long as it needs (wait states), but
  must never deadlock: an access it cannot complete is acknowledged anyway.
- A byte access asserts only one of `bus_uds`/`bus_lds`.  Byte-wide 8-bit
  peripherals on the odd byte (MFP, ACIA, PSG data...) return their byte on
  [7:0] and the other byte as 8'hFF (open bus pull-ups), unless Hatari says
  otherwise for that register.
- The decoder, not the device, generates bus errors for unmapped addresses.
  A device only needs `output bus_berr` if some of its own register addresses
  bus error (document it in the header).

## Common DMA (RAM master) port

Devices that read or write ST-RAM by DMA (blitter, FDC DMA, DMA sound, DSP
nothing) have a 16-bit master port served by the memory arbiter:

```
output            dma_req,     // level; address/data stable until dma_ack
output            dma_we,
output     [23:1] dma_addr,    // word address in the 16 MB space
output      [1:0] dma_be,      // [1] = even byte (15:8), [0] = odd byte (7:0)
output     [15:0] dma_wdata,
input      [15:0] dma_rdata,   // valid on the dma_ack clock
input             dma_ack      // one clock; the next request may follow at once
```

Latency is variable (DDR3, typically 6..20 clocks).  A device that needs a
guaranteed rate keeps its own FIFO.

The arbiter applies the memory types to every DMA access, as Hatari's
STMemory_DMA_ReadWord/WriteWord and Hatari's memory banks do: ST-RAM is read
and written directly, except that DMA sound and blitter writes to
$000000-$000007 are dropped (SysMem_wput; the disk DMA copies straight into
ST-RAM and may write there), ROM and cartridge are read only (writes
dropped), and a bus-error region reads $0000 and drops writes.  The
system routes blitter accesses to $F00000-$F0FFFF and $FF8000-$FFFFFF to the
device bus instead (put_word/get_word reach the I/O handlers); a device
address that bus errors reads $0000 and drops the write there too.

## Video fetch port (Videl only)

```
output            vid_req,     // level; held until vid_ack
output     [23:3] vid_addr,    // 64-bit word address of the first word
input             vid_ack,     // one clock: the burst is accepted, the port may
                               //   issue the next request immediately
input      [63:0] vid_data,    // [63:48] = word at the lowest address
input             vid_valid    // one clock per 64-bit word; exactly 4 per burst,
                               //   in order, bursts returned in request order
```

A burst is always four 64-bit words (32 bytes) aligned to 32 bytes
(`vid_addr[4:3]` = 0).  The arbiter gives the video port the highest
priority.

## Disk image port (hps_io block interface)

Identical to MiSTer `hps_io` (512-byte blocks):

```
input             img_mounted,  // pulse
input             img_readonly,
input      [63:0] img_size,
output reg [31:0] sd_lba,
output reg        sd_rd,
output reg        sd_wr,
input             sd_ack,
input       [8:0] sd_buff_addr,
input       [7:0] sd_buff_dout,
output      [7:0] sd_buff_din,
input             sd_buff_wr
```

## Module list and ownership

| Module | File | Notes |
|--------|------|-------|
| `emu` | `Falcon.sv` | MiSTer top: hps_io, PLL, OSD, DDR3, video/audio out |
| `falcon_system` | `rtl/falcon/falcon_system.sv` | CPU, bus bridge, decode, interrupt logic, device instances |
| `falcon_cpubus` | `rtl/falcon/falcon_cpubus.sv` | 68030 pin bus to RAM/device bus bridge, CPU space cycles; Falcon mode: Hatari's cycle lengths, 16-bit RAM/ROM port, posted write, read-ahead buffer |
| `falcon_cpuclk` | `rtl/falcon/falcon_cpuclk.sv` | CPU clock enable (turbo, 16/8 MHz) and the time accounting (debt) |
| `falcon_memarb` | `rtl/falcon/falcon_memarb.sv` | ST-RAM arbiter: video, DMA masters, CPU, ROM loader |
| `falcon_ddram` | `rtl/falcon/falcon_ddram.sv` | DDR3 adapter with bursts |
| `falcon_combel` | `rtl/falcon/falcon_combel.sv` | $FF8001/6/7, joypads, DIP switches |
| `falcon_videl` | `rtl/falcon/falcon_videl.sv` | Videl |
| `falcon_mfp` | `rtl/falcon/falcon_mfp.sv` | MC68901 |
| `falcon_acia` | `rtl/falcon/falcon_acia.sv`, `falcon_ikbd.sv` | two MC6850 + IKBD (HD6301) behaviour |
| `falcon_psg` | `rtl/falcon/falcon_psg.sv` | YM2149 |
| `falcon_nvram` | `rtl/falcon/falcon_nvram.sv` | MC146818 RTC + NVRAM |
| `falcon_ide` | `rtl/falcon/falcon_ide.sv` | IDE (ATA PIO, master and slave) |
| `falcon_fdc` | `rtl/falcon/falcon_fdc.sv` | ST DMA chip + WD1772 |
| `falcon_blitter` | `rtl/falcon/falcon_blitter.sv` | Blitter |
| `falcon_crossbar` | `rtl/falcon/falcon_crossbar.sv` | DMA sound, crossbar, codec |
| `falcon_dsp` | `rtl/falcon/dsp/*.sv` | DSP56001, host port, SSI, memories |

## Coding rules

- Synthesizable SystemVerilog that Quartus Prime 17.0 accepts: `module`,
  `always @(posedge clk)`, `reg`/`wire`/`logic`, `localparam`, functions,
  generate.  No interfaces, no classes, no unpacked-array ports, no
  `always_ff` with multiple drivers.
- One clock (`clk`); no gated or derived clocks, only clock enables.
- Block RAMs are inferred with the standard Intel templates (single or simple
  dual port, registered read) so they map to M10K.
- Every module has a header that names the Hatari functions it follows and
  lists deviations.
- Testbenches: Verilator, plain ASCII, no emojis.  Tests drive the real
  module (never a model of it) and check against values derived from the
  reference (Hatari source or the datasheet), with each check printing what it
  expected.
