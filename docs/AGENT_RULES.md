# Rules for module builders

You are building one module of a new MiSTer FPGA core implementing the Atari
Falcon030, in /home/adam/Falcon_MiSTer.

1. Read docs/ARCHITECTURE.md first.  It is the contract: register bus port
   names and protocol, DMA port, byte lanes, interrupt wiring, coding rules.
2. Behavioural reference: Hatari, /home/adam/hatari/src (Falcon parts in
   src/falcon/).  Read the relevant source fully before writing RTL; do not
   guess behaviour.  A built Hatari binary is /home/adam/hatari/build/src/hatari
   and you may compile pieces of Hatari's C as golden models in your tests
   (Verilator C++ testbenches can link C code).
3. Additional reference: FireBee FPGA sources (LGPL VHDL IP by Wolfgang
   Foerster) under /home/adam/Falcon_MiSTer/ref/firebee/ (FalconIO_SDCard_IDE_CF/
   WF_MFP68901_IP, WF_UART6850_IP, WF_SND2149_IP, WF_FDC1772_IP, WF5380,
   Video/BLITTER, DSP/).  You may study them or adopt them.  Adopted VHDL
   must compile in Quartus 17.0 and be converted for Verilator with
   `ghdl synth --std=08 --out=verilog` (GHDL 6.0 installed).  Prefer native
   SystemVerilog unless adoption clearly saves effort; keep the original
   license headers.  The DSP/ folder has no license header: reference only,
   do not copy code from it.
4. Ownership: edit only the files your task lists.  Do not edit
   docs/ARCHITECTURE.md, sys/, rtl/AP68030 or other modules; if the contract
   needs a change, say so in your final report.  Do not git commit.
5. Tools: Verilator 5.052, iverilog, GHDL 6.0, Quartus 17.0
   (/opt/intelFPGA_lite/17.0/quartus/bin).  Check synthesizability with
   quartus_map (and quartus_fit if you want timing) on your module alone in a
   throwaway project under tb/<name>/syn/ (device 5CSEBA6U23I7, 32 MHz clock);
   report ALMs, registers, M10K blocks.  Other agents run Quartus and
   simulations on this machine at the same time: never pkill/killall; kill
   only processes whose PIDs you started.
6. The user's rules (mandatory):
   - no emojis or non-ASCII characters in any .v/.sv/.vhd file or testbench;
   - tests exercise the actual RTL module, never a mock or simplified model;
   - never simplify or weaken a failing test to make it pass - fix the RTL;
   - do not assume how things work - read the reference first;
   - fix issues, do not hide them.
7. Deliverables: the RTL; a Verilator testbench in tb/<name>/ with a run.sh
   that builds and runs everything and ends with a PASS/FAIL summary; all
   tests passing.
8. Final report (your last message): files created, port list, what is
   implemented, test list with results, resource estimate, known gaps and
   deviations from Hatari/hardware, and any contract issues.
