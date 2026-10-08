# mbox_ping: FPGA <-> ARM mailbox round-trip latency

Measures how long one request/answer exchange between the core and a process
on the MiSTer's ARM takes through a DDR3 mailbox: the cost per exchange of
any coprocessor running on the ARM (e.g. an FPU).  The FPGA side is
`rtl/falcon/falcon_mbox_test.sv`, present only in a measurement build.

## Build

```
FALCON_BRINGUP="MBOX_TEST" ./scripts/build.sh     # output_files/Falcon.rbf, measurement core
tools/mbox_ping/build.sh                          # needs arm-none-linux-gnueabihf-gcc (Main_MiSTer's toolchain)
```

## Run

1. Copy the measurement `Falcon.rbf` to `/media/fat/_Computer/` under a name
   of its own (e.g. `Falcon_mboxtest.rbf`) and `tools/mbox_ping/mbox_ping` to
   `/media/fat/` (or anywhere on the MiSTer).
2. Load that core (it behaves as the normal Falcon core; TOS may run or not).
3. Over SSH, as root:

```
/media/fat/mbox_ping                    # 10000 round trips, any CPU
/media/fat/mbox_ping -c 1               # pinned to CPU 1
/media/fat/mbox_ping -c 1 -f            # pinned, SCHED_FIFO
/media/fat/mbox_ping -n 32768 -c 1 -f -o /tmp/rt.csv
```

Output: round-trip time in microseconds (min, median, p90, p99, p99.9, max,
mean), measured by the FPGA at its 32 MHz clock from its PING write to the
read that sees the echo, and the time of one FPGA read of the mailbox.

`-m FILE` maps a file instead of `/dev/mem`; `tb/mbox` uses it to run this
program against the simulated RTL.

The probe shares falcon_memarb's d3 port with the FPU bridge: the
measurement core has no FPU (its bridge never sees the ARM service).

## Results (DE10-Nano, 2026-10-07, measurement core built with Quartus 17.0 Lite)

10000 round trips each, microseconds:

| mbox_ping      | min  | median | p90  | p99  | p99.9 | max     |
|----------------|------|--------|------|------|-------|---------|
| (default)      | 0.59 | 0.59   | 0.72 | 4.03 | 13.97 | 171.34  |
| -c 1           | 0.59 | 0.59   | 0.69 | 2.03 | 9.06  | 3022.66 |
| -c 1 -f        | 0.44 | 0.59   | 0.72 | 2.44 | 4.69  | 7.50    |

One FPGA read of the mailbox through falcon_memarb takes 0.32 us; the FPGA
read PONG 2.0-2.3 times per round trip, so the answer arrives within about
two of its reads (the measurement's resolution).  The tail is Linux
scheduling: a service pinned to CPU 1 with SCHED_FIFO stays below 8 us.

Note: until scripts/build.sh restores Falcon.qsf after a build (it does on
feature/nvram-saving), check `git diff Falcon.qsf` after a MBOX_TEST build:
Quartus can write the FALCON_MBOX_TEST macro into the project file.
