//============================================================================
//  Atari Falcon030 system: CPU, bus, decode, interrupts, devices
//
//  See docs/ARCHITECTURE.md for the memory map, the interrupt wiring and the
//  device bus contract.  Hatari references: ioMemTabFalcon.c (I/O map and
//  the addresses that do not bus error), ioMem.c (STE-compatible bus mode
//  void regions), m68000.c (M68000_Update_intlev: IPL6 = MFP OR DSP, IPL5
//  SCC, IPL4 VBL, IPL2 HBL), mfp.c (GPIP wiring).
//============================================================================

module falcon_system #(parameter CLK_HZ = 32000000)
(
	input             clk,
	input             reset,        // machine reset
	input             cold_reset,   // power on / cold reset / TOS load
	input             por,          // power on only (memory arbiter)

	input       [3:0] ram_mb,
	input             ram_tos,      // loaded TOS is a RAM TOS behind its loader
	input       [1:0] monitor,
	input             cpu_turbo,    // 1: CPU on every clock (32 MHz); 0: Falcon, 16/8 MHz ($FF8007 bit 0)

	// ROM/cartridge loader
	input             ld_wr,
	input      [23:0] ld_addr,
	input       [7:0] ld_data,
	output            ld_busy,

	// MiSTer inputs
	input      [10:0] ps2_key,
	input      [24:0] ps2_mouse,
	input      [31:0] joy0,
	input      [31:0] joy1,
	input      [15:0] ana0,         // MiSTer left analog sticks (paddles)
	input      [15:0] ana1,
	input      [64:0] rtc,

	// Persistent 50-byte NVRAM image, served by the MiSTer top level.
	input             nv_init,
	input       [5:0] nv_addr,
	input       [7:0] nv_din,
	input             nv_wr,
	output      [7:0] nv_dout,
	output            nv_changed,
	output            nv_ready,

	// disk slots: 0 floppy A, 1 floppy B, 2 IDE master, 3 IDE slave,
	//             4..6 SCSI ID 0..2 (2 = CD-ROM); Main's FALCON_SCSI_SLOT0 = 4
	input       [6:0] img_mounted,
	input             img_readonly,
	input      [63:0] img_size,
	output     [31:0] sd_lba[7],
	output      [6:0] sd_rd,
	output      [6:0] sd_wr,
	output      [5:0] sd_blk_cnt[7],
	input       [6:0] sd_ack,
	input      [13:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din[7],
	input             sd_buff_wr,

	// video
	output      [7:0] r,
	output      [7:0] g,
	output      [7:0] b,
	output            hsync,
	output            vsync,
	output            hblank,
	output            vblank,
	output            ce_pix,

	// audio
	output signed [15:0] audio_l,
	output signed [15:0] audio_r,

	// serial
	input             midi_rx,
	output            midi_tx,
	input             ser_rx,
	output            ser_tx,

	output            fdd_led,
	output            hdd_led,

	// DDR3
	input             DDRAM_BUSY,
	output      [7:0] DDRAM_BURSTCNT,
	output     [28:0] DDRAM_ADDR,
	input      [63:0] DDRAM_DOUT,
	input             DDRAM_DOUT_READY,
	output            DDRAM_RD,
	output     [63:0] DDRAM_DIN,
	output      [7:0] DDRAM_BE,
	output            DDRAM_WE
);

//////////////////////////////////////////////////////////////////
//  CPU
//////////////////////////////////////////////////////////////////

wire [31:0] cpu_a, cpu_do;
wire  [2:0] cpu_fc;
wire  [1:0] cpu_siz;
wire        cpu_rw, cpu_rmc_n, cpu_as_n, cpu_ds_n, cpu_dben_n, cpu_ecs_n, cpu_ocs_n;
wire        cpu_ciout_n, cpu_cbreq_n, cpu_bus_oe, cpu_d_oe, cpu_bg_n, cpu_ipend_n;
wire        cpu_reset_oe, cpu_refill_n, cpu_status_n, cpu_halted;
wire [31:0] cpu_di;
wire        dsack0_n, dsack1_n, berr_n, avec_n, ciin_n;
wire  [2:0] ipl_n;
reg         cpu_br_n = 1, cpu_bgack_n = 1;
wire        snoop_we;
wire [23:0] snoop_addr;
wire [31:0] dbg_pc;

// The RESET instruction resets the peripherals only (cpu_reset_oe); the
// CPU itself takes the machine reset.
wire dev_reset = reset | cpu_reset_oe;

wire ld_busy_arb;
assign ld_busy = ld_busy_arb;

// CPU clock (docs/CPU_TIMING.md): the 68030 advances on the clocks with
// cpu_ce.  Turbo: every clock (32 MHz).  Falcon: 16 MHz, or 8 MHz when
// $FF8007 bit 0 is clear, on Hatari's clock count: falcon_cpubus gives
// every bus cycle Hatari's length and holds the processor while an answer
// is late; falcon_cpuclk lets it catch up afterwards.
wire        cpu_16mhz;
wire        cpu_ce, cpu_hold, cpu_fmode, cpu_wbuf_busy, cpu_inst, cpu_idle, cpu_idle_tick;
wire  [1:0] cpu_tm_pop, cpu_tm_md;
wire  [7:0] cpu_back;
wire [95:0] cpu_tm_q;
wire  [2:0] cpu_tm_qn;
wire [31:0] cpu_tm_scan, cpu_fetch_stop;
wire        cpu_tm_flush, cpu_fetch_stop_v, cpu_scan_v;
wire [31:0] cpu_scan_to;
// Hatari's prefetch pipeline (stops before unconditional branches)
falcon_pipescan pipescan
(
	.clk(clk), .ce(cpu_ce), .enable(cpu_fmode), .q(cpu_tm_q), .qn(cpu_tm_qn), .scan(cpu_tm_scan),
	.flush(cpu_tm_flush), .stop_v(cpu_fetch_stop_v), .stop_at(cpu_fetch_stop),
	.scan_v(cpu_scan_v), .scan_to(cpu_scan_to)
);
wire  [4:0] cpu_tpos;
wire  [3:0] cpu_credit;
falcon_cpuclk cpuclk
(
	.clk(clk), .reset(reset),
	.turbo(!cpu_fmode), .cpu_16mhz(cpu_16mhz),
	.hold(cpu_hold), .credit(cpu_credit), .idle(cpu_idle), .back(cpu_back),
	.cpu_ce(cpu_ce), .idle_tick(cpu_idle_tick), .tpos(cpu_tpos),
	.debt(), .debt_peak(), .forgiven(), .held(), .idled()
);

ap030_top #(.USE_CE(1)) cpu
(
	.clk(clk), .ce(cpu_ce),
	.a(cpu_a), .fc(cpu_fc), .siz(cpu_siz), .rw(cpu_rw), .rmc_n(cpu_rmc_n),
	.as_n(cpu_as_n), .ds_n(cpu_ds_n), .dben_n(cpu_dben_n), .ecs_n(cpu_ecs_n), .ocs_n(cpu_ocs_n),
	.ciout_n(cpu_ciout_n), .cbreq_n(cpu_cbreq_n), .bus_oe(cpu_bus_oe),
	.d_o(cpu_do), .d_oe(cpu_d_oe), .d_i(cpu_di),
	.dsack0_n(dsack0_n), .dsack1_n(dsack1_n), .sterm_n(1'b1), .berr_n(berr_n), .halt_n(1'b1),
	.avec_n(avec_n), .ciin_n(ciin_n), .cback_n(1'b1),
	.br_n(cpu_br_n), .bg_n(cpu_bg_n), .bgack_n(cpu_bgack_n),
	.ipl_n(ipl_n), .ipend_n(cpu_ipend_n),
	.reset_n_i(~(reset | sv_busy)), .reset_n_oe(cpu_reset_oe),
	.cdis_n(1'b1), .mmudis_n(1'b1), .refill_n(cpu_refill_n), .status_n(cpu_status_n),
	.dbg_pc(dbg_pc), .dbg_sr(), .dbg_state(), .dbg_inst(cpu_inst), .tm_pop(cpu_tm_pop), .tm_md(cpu_tm_md), .dbg_halted(cpu_halted),
	.tm_q(cpu_tm_q), .tm_qn(cpu_tm_qn), .tm_scan(cpu_tm_scan), .tm_flush(cpu_tm_flush),
	.fetch_stop_v(cpu_fetch_stop_v), .fetch_stop(cpu_fetch_stop),
	.fetch_scan_v(cpu_scan_v), .fetch_scan_to(cpu_scan_to),
	.dbg_vbr(), .dbg_cacr(), .dbg_cache_clear(),
	.snoop_we(snoop_we), .snoop_addr({8'd0, snoop_addr}), .nmi_vec_nocache(1'b0),
	.fetch_lazy(cpu_fmode)   // Falcon mode: Hatari's instruction prefetch
);

//////////////////////////////////////////////////////////////////
//  Bus bridge
//////////////////////////////////////////////////////////////////

wire        cram_req, cram_we, cram_ack;
wire [23:2] cram_addr;
wire  [3:0] cram_be;
wire [31:0] cram_wdata, cram_rdata;
wire [63:0] cram_rdata64;

// device bus: the CPU bridge's, or the blitter's while it reaches I/O
wire        c_dev_cs, c_dev_stb, c_dev_we, c_dev_uds, c_dev_lds, dev_super;
wire [23:1] c_dev_addr;
wire [15:0] c_dev_din;
wire        dev_cs, dev_stb, dev_we, dev_uds, dev_lds;
wire [23:1] dev_addr;
wire [15:0] dev_din;
reg  [15:0] dev_dout;
reg         dev_ack, dev_berr;

wire        iack_req;
wire  [2:0] iack_level;
reg         iack_done, iack_avec, iack_spur, iack_mfp;
reg   [7:0] iack_vector;
wire        cpu_cycle_done;

// coprocessor interface registers: the FPU bridge (docs/FPU_ARM.md)
wire        cp_req, cp_we, cp_ack, cp_berr;
wire  [2:0] cp_id;
wire  [4:0] cp_off;
wire  [1:0] cp_siz;
wire [31:0] cp_wdata, cp_rdata;

// the blitter's DMA port (its accesses also count in the CPU's Falcon time)
wire        blt_req, blt_we, blt_ack;
wire [23:1] blt_addr;
wire  [1:0] blt_be;
wire [15:0] blt_wdata, blt_rdata;

falcon_cpubus cpubus
(
	.clk(clk), .reset(reset | sv_busy),
	.ram_mb(ram_mb), .ram_tos(ram_tos),
	.a(cpu_a), .fc(cpu_fc), .siz(cpu_siz), .rw(cpu_rw), .as_n(cpu_as_n), .ds_n(cpu_ds_n),
	.bus_oe(cpu_bus_oe), .d_o(cpu_do), .d_i(cpu_di),
	.dsack0_n(dsack0_n), .dsack1_n(dsack1_n), .berr_n(berr_n), .avec_n(avec_n), .ciin_n(ciin_n),
	.ram_req(cram_req), .ram_we(cram_we), .ram_addr(cram_addr), .ram_be(cram_be),
	.ram_wdata(cram_wdata), .ram_rdata(cram_rdata), .ram_ack(cram_ack),
	.dev_cs(c_dev_cs), .dev_stb(c_dev_stb), .dev_we(c_dev_we), .dev_addr(c_dev_addr),
	.dev_uds(c_dev_uds), .dev_lds(c_dev_lds), .dev_din(c_dev_din), .dev_dout(dev_dout),
	.dev_ack(dev_ack), .dev_berr(dev_berr), .dev_super(dev_super),
	.iack_req(iack_req), .iack_level(iack_level), .iack_done(iack_done), .iack_avec(iack_avec),
	.iack_spur(iack_spur), .iack_vector(iack_vector),
	.cp_req(cp_req), .cp_we(cp_we), .cp_id(cp_id), .cp_off(cp_off), .cp_siz(cp_siz),
	.cp_wdata(cp_wdata), .cp_ack(cp_ack), .cp_berr(cp_berr), .cp_rdata(cp_rdata),
	.cycle_done(cpu_cycle_done),
	.fmode_in(!cpu_turbo), .fmode(cpu_fmode), .cpu_ce(cpu_ce), .tpos(cpu_tpos),
	.inst(cpu_inst & cpu_ce), .hold(cpu_hold), .credit(cpu_credit), .wbuf_busy(cpu_wbuf_busy),
	.ram_rdata64(cram_rdata64), .snoop_we(snoop_we), .snoop_addr(snoop_addr),
	.buf_flush(ld_busy_arb | sv_busy), .iack_mfp(iack_mfp),
	.dispatch(cpu_inst), .tm_pop(cpu_tm_pop), .tm_md(cpu_tm_md),
	.idle(cpu_idle), .idle_tick(cpu_idle_tick), .back(cpu_back), .bus_lost(!cpu_bgack_n), .blit_acc(blt_ack),
	.gov_idle(), .gov_back()
);


//////////////////////////////////////////////////////////////////
//  Memory arbiter
//////////////////////////////////////////////////////////////////

wire        vid_req, vid_ack, vid_valid;
wire [23:3] vid_addr;
wire [63:0] vid_data;

wire        snd_req, snd_we, snd_ack;
wire [23:1] snd_addr;
wire  [1:0] snd_be;
wire [15:0] snd_wdata, snd_rdata;

wire        fdc_dreq, fdc_dwe, fdc_dack;
wire [23:1] fdc_daddr;
wire  [1:0] fdc_dbe;
wire [15:0] fdc_dwdata, fdc_drdata;

// The blitter's accesses to the IDE and I/O areas go to the devices
// (Hatari's blitter uses get_word/put_word, which reach the I/O handlers):
// one device-bus cycle per word while the blitter owns the bus.  A bus
// error there reads $0000 and drops the write (STMemory_DMA_*Word).
wire        blt_io = (blt_addr[23:16] == 8'hF0) || (blt_addr[23:15] == 9'h1FF);
wire        mblt_ack;
wire [15:0] mblt_rdata;
reg         bd_cs = 0, bd_stb = 0, bd_ack = 0;
reg  [15:0] bd_rdata;
reg   [9:0] bd_tmo;
always @(posedge clk) begin
	bd_stb <= 0;
	bd_ack <= 0;
	if (reset) bd_cs <= 0;
	else if (!bd_cs) begin
		// the request is still up on the clock of the acknowledge
		if (blt_req && blt_io && !bd_ack) begin bd_cs <= 1; bd_stb <= 1; bd_tmo <= 0; end
	end
	else begin
		bd_tmo <= bd_tmo + 1'd1;
		if (dev_berr || bd_tmo == 10'h3FF) begin bd_cs <= 0; bd_ack <= 1; bd_rdata <= 16'h0000; end
		else if (dev_ack)                 begin bd_cs <= 0; bd_ack <= 1; bd_rdata <= dev_dout; end
	end
end
assign blt_ack   = mblt_ack | bd_ack;
assign blt_rdata = bd_ack ? bd_rdata : mblt_rdata;

assign dev_cs   = bd_cs ? 1'b1      : c_dev_cs;
assign dev_stb  = bd_cs ? bd_stb    : c_dev_stb;
assign dev_we   = bd_cs ? blt_we    : c_dev_we;
assign dev_addr = bd_cs ? blt_addr  : c_dev_addr;
assign dev_uds  = bd_cs ? blt_be[1] : c_dev_uds;
assign dev_lds  = bd_cs ? blt_be[0] : c_dev_lds;
assign dev_din  = bd_cs ? blt_wdata : c_dev_din;

//////////////////////////////////////////////////////////////////
//  RAM TOS system variables (Hatari STMemory_SetDefaultConfig)
//
//  A RAM TOS (TOS 4.92) skips memory sizing and trusts the system
//  variables the TOS that loaded it left behind.  When cold reset ends
//  with a RAM TOS loaded, write them as Hatari does for a Falcon: the
//  memvalid magics, no TT-RAM, phystop = end of ST-RAM and memtop 32 KB
//  below it.  The CPU stays in reset until the 28 bytes are written.
//////////////////////////////////////////////////////////////////

reg        sv_busy = 0;
reg        sv_wr;
reg  [4:0] sv_idx;
reg        sv_wait;
reg        cold_d;
wire [23:0] phys_top = {ram_mb, 20'd0};
wire [23:0] mem_top  = phys_top - 24'h8000;

reg [23:0] sv_addr;
reg  [7:0] sv_data;
always @* begin : sv_table
	reg [31:0] v; reg [11:0] a;
	case (sv_idx[4:2])
		3'd0: begin a = 12'h420; v = 32'h752019F3; end   // memvalid
		3'd1: begin a = 12'h43A; v = 32'h237698AA; end   // memval2
		3'd2: begin a = 12'h51A; v = 32'h5555AAAA; end   // memval3
		3'd3: begin a = 12'h5A4; v = 32'h00000000; end   // ramtop: no TT-RAM
		3'd4: begin a = 12'h5A8; v = 32'h1357BD13; end   // ramvalid
		3'd5: begin a = 12'h42E; v = {8'd0, phys_top}; end // phystop
		default: begin a = 12'h436; v = {8'd0, mem_top}; end // memtop
	endcase
	sv_addr = {12'd0, a} + sv_idx[1:0];
	sv_data = v[31 - 8 * sv_idx[1:0] -: 8];
end

always @(posedge clk) begin
	cold_d <= cold_reset;
	sv_wr  <= 0;
	if (cold_d && !cold_reset && ram_tos) begin
		sv_busy <= 1; sv_idx <= 0; sv_wait <= 0;
	end
	else if (sv_busy) begin
		if (sv_wait) sv_wait <= 0;                 // the arbiter shows busy one clock later
		else if (!ld_busy_arb && !sv_wr) begin
			sv_wr   <= 1;
			sv_wait <= 1;
			if (sv_idx == 5'd27) sv_busy <= 0;
			else sv_idx <= sv_idx + 1'd1;
		end
	end
end
reg [23:0] sv_addr_q;
reg  [7:0] sv_data_q;
always @(posedge clk) if (!sv_wr) begin sv_addr_q <= sv_addr; sv_data_q <= sv_data; end

// the arbiter's d3 port: the FPU bridge's HPS mailbox, or in a measurement
// build the mailbox latency probe (the bridge then never sees the ARM
// service, so there is no FPU)
wire        d3_req, d3_we, d3_ack;
wire [23:1] d3_addr;
wire  [1:0] d3_be;
wire [15:0] d3_wdata, d3_rdata;
wire        fpu_present;

`ifdef FALCON_MBOX_TEST
falcon_mbox_test #(.CLK_HZ(CLK_HZ)) mbox_test
(
	.clk(clk), .reset(por),
	.dma_req(d3_req), .dma_we(d3_we), .dma_addr(d3_addr), .dma_be(d3_be),
	.dma_wdata(d3_wdata), .dma_rdata(d3_rdata), .dma_ack(d3_ack)
);
`endif

`ifdef FALCON_NO_FPU
// no FPU: every coprocessor cycle ends in BERR (F-line), as before the bridge
reg cp_ack_r;
always @(posedge clk) cp_ack_r <= cp_req;
assign cp_ack = cp_ack_r, cp_berr = 1'b1, cp_rdata = 32'hFFFF_FFFF, fpu_present = 1'b0;
`ifndef FALCON_MBOX_TEST
assign d3_req = 1'b0, d3_we = 1'b0, d3_addr = 23'd0, d3_be = 2'b00, d3_wdata = 16'd0;
`endif
`else
falcon_fpu_bridge #(.CLK_HZ(CLK_HZ)) fpu
(
	.clk(clk), .reset(dev_reset), .por(por),
	.cp_req(cp_req), .cp_we(cp_we), .cp_id(cp_id), .cp_off(cp_off), .cp_siz(cp_siz),
	.cp_wdata(cp_wdata), .cp_ack(cp_ack), .cp_berr(cp_berr), .cp_rdata(cp_rdata),
`ifdef FALCON_MBOX_TEST
	.dma_req(), .dma_we(), .dma_addr(), .dma_be(), .dma_wdata(), .dma_rdata(16'd0), .dma_ack(1'b0),
`else
	.dma_req(d3_req), .dma_we(d3_we), .dma_addr(d3_addr), .dma_be(d3_be),
	.dma_wdata(d3_wdata), .dma_rdata(d3_rdata), .dma_ack(d3_ack),
`endif
	.present(fpu_present)
);
`endif

falcon_memarb memarb
(
	.clk(clk), .reset(por), .ram_mb(ram_mb),
	.vid_req(vid_req), .vid_addr(vid_addr), .vid_ack(vid_ack), .vid_data(vid_data), .vid_valid(vid_valid),
	.ld_wr(ld_wr | sv_wr), .ld_addr(sv_wr ? sv_addr_q : ld_addr), .ld_data(sv_wr ? sv_data_q : ld_data), .ld_busy(ld_busy_arb),
	.d0_req(snd_req), .d0_we(snd_we), .d0_addr(snd_addr), .d0_be(snd_be), .d0_wdata(snd_wdata),
	.d0_rdata(snd_rdata), .d0_ack(snd_ack),
	.d1_req(fdc_dreq), .d1_we(fdc_dwe), .d1_addr(fdc_daddr), .d1_be(fdc_dbe), .d1_wdata(fdc_dwdata),
	.d1_rdata(fdc_drdata), .d1_ack(fdc_dack),
	.d2_req(blt_req & ~blt_io), .d2_we(blt_we), .d2_addr(blt_addr), .d2_be(blt_be), .d2_wdata(blt_wdata),
	.d2_rdata(mblt_rdata), .d2_ack(mblt_ack),
	.d3_req(d3_req), .d3_we(d3_we), .d3_addr(d3_addr), .d3_be(d3_be), .d3_wdata(d3_wdata),
	.d3_rdata(d3_rdata), .d3_ack(d3_ack),
	.cpu_req(cram_req), .cpu_we(cram_we), .cpu_addr(cram_addr), .cpu_be(cram_be),
	.cpu_wdata(cram_wdata), .cpu_rdata(cram_rdata), .cpu_rdata64(cram_rdata64), .cpu_ack(cram_ack),
	.snoop_we(snoop_we), .snoop_addr(snoop_addr),
	.DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE)
);

//////////////////////////////////////////////////////////////////
//  Address decode (ioMemTabFalcon.c)
//////////////////////////////////////////////////////////////////

wire        falcon_bus;      // $FF8007 bit 5: 1 = Falcon bus (more bus errors)
wire [23:0] da = {dev_addr, 1'b0};

wire sel_ide     = (da[23:6]  == 18'h3C000);                   // F00000-F0003F
wire sel_combel  = ((da[23:4] == 20'hFF800) &&                 // FF8000/1, FF8006/7, FF800C/D; the
                    (da[3:0] == 4'h0 || da[3:0] == 4'h6 ||     // holes between them are void only in
                     da[3:0] == 4'hC)) ||                      // the STE-compatible bus mode
                   (da[23:2]  == 22'h3FE480) ||                // FF9200-FF9203
                   (da[23:3]  == 21'h1FF242) ||                // FF9210-FF9217
                   (da[23:2]  == 22'h3FE488);                  // FF9220-FF9223
wire sel_videl   = (da[23:8]  == 16'hFF82) && (da[7:0] < 8'hC4);  // FF8200-FF82C3
wire sel_pal     = (da[23:10] == 14'h3FE6);                    // FF9800-FF9BFF
wire sel_fdc     = (da[23:4]  == 20'hFF860) && (da[3:2] != 2'b00); // FF8604-FF860F
wire sel_psg     = (da[23:2]  == 22'h3FE200);                  // FF8800-FF8803
wire sel_xbar    = (da[23:8]  == 16'hFF89) && (da[7:0] < 8'h44);  // FF8900-FF8943
wire sel_nvram   = (da[23:2]  == 22'h3FE258);                  // FF8960-FF8963
wire sel_blit    = (da[23:6]  == 18'h3FE28);                   // FF8A00-FF8A3F
wire sel_scc     = (da[23:3]  == 21'h1FF190);                  // FF8C80-FF8C87
wire sel_dsp     = (da[23:3]  == 21'h1FF440);                  // FFA200-FFA207
wire sel_mfp     = (da[23:6]  == 18'h3FFE8) && (da[5:0] < 6'h30); // FFFA00-FFFA2F
wire sel_acia    = (da[23:3]  == 21'h1FFF80);                  // FFFC00-FFFC07

// addresses that read as $FF/write nothing instead of bus erroring
// (IoMem_FixVoidAccessForCompatibleFalcon in the STE-compatible bus mode,
// plus the "No bus error here" entries that exist in both modes)
function in_rng;
	input [23:0] x, lo, hi;
	in_rng = (x >= lo) && (x <= hi);
endfunction
wire void_always = in_rng(da, 24'hFFFF82, 24'hFFFF83);
wire void_compat = in_rng(da, 24'hFF8002, 24'hFF8005) || in_rng(da, 24'hFF8008, 24'hFF800B) ||
                   in_rng(da, 24'hFF800E, 24'hFF805F) || in_rng(da, 24'hFF8064, 24'hFF81FF) ||
                   in_rng(da, 24'hFF82C4, 24'hFF83FF) || in_rng(da, 24'hFF8804, 24'hFF88FF) ||
                   in_rng(da, 24'hFF8964, 24'hFF896F) || in_rng(da, 24'hFF8C00, 24'hFF8C7F) ||
                   in_rng(da, 24'hFF8C88, 24'hFF8CFF) || in_rng(da, 24'hFF9000, 24'hFF91FF) ||
                   in_rng(da, 24'hFF9204, 24'hFF920F) || in_rng(da, 24'hFF9218, 24'hFF921F) ||
                   in_rng(da, 24'hFF9224, 24'hFF97FF) || in_rng(da, 24'hFF9C00, 24'hFF9FFF);
// STE-compatible mode only (IoMemTabFalc_Compatible_*): byte accesses to
// $FF8560, $FF8564, $FFC020/1, $FFD020, $FFD420 and $FFD425 (uds = even
// byte, lds = odd byte), word accesses to $FFD074, $FFD520 and $FFD530
wire is_byte     = dev_uds ^ dev_lds;
wire void_cbyte  = is_byte && (((da == 24'hFF8560 || da == 24'hFF8564 || da == 24'hFFD020 ||
                                 da == 24'hFFD420) && dev_uds) ||
                               (da == 24'hFFD424 && dev_lds) || (da == 24'hFFC020));
wire void_cword  = !is_byte && (da == 24'hFFD074 || da == 24'hFFD520 || da == 24'hFFD530);
wire sel_void    = void_always || (!falcon_bus && (void_compat || void_cbyte || void_cword));

// per-device bus signals
wire [15:0] ide_dout, combel_dout, videl_dout, fdc_dout, psg_dout, xbar_dout, nvram_dout;
wire [15:0] blit_dout, dsp_dout, mfp_dout, acia_dout;
wire        ide_ack, combel_ack, videl_ack, fdc_ack, psg_ack, xbar_ack, nvram_ack;
wire        blit_ack, dsp_ack, mfp_ack, acia_ack;
wire        xbar_berr, fdc_berr;
reg   [7:0] scc_ptr;

always @* begin
	dev_dout = 16'hFFFF;
	dev_ack  = 0;
	dev_berr = 0;
	if (dev_cs) begin
		if      (sel_ide)    begin dev_dout = ide_dout;    dev_ack = ide_ack;    end
		else if (sel_combel) begin dev_dout = combel_dout; dev_ack = combel_ack; end
		else if (sel_videl || sel_pal) begin dev_dout = videl_dout; dev_ack = videl_ack; end
		else if (sel_fdc)    begin dev_dout = fdc_dout;    dev_ack = fdc_ack;    dev_berr = fdc_berr; end
		else if (sel_psg)    begin dev_dout = psg_dout;    dev_ack = psg_ack;    end
		else if (sel_xbar)   begin dev_dout = xbar_dout;   dev_ack = xbar_ack;   dev_berr = xbar_berr; end
		else if (sel_nvram)  begin dev_dout = nvram_dout;  dev_ack = nvram_ack;  end
		else if (sel_blit)   begin dev_dout = blit_dout;   dev_ack = blit_ack;   end
		else if (sel_scc)    begin dev_dout = 16'h2C2C;    dev_ack = dev_stb;    end  // RR0: Tx empty, DCD, CTS
		else if (sel_dsp)    begin dev_dout = dsp_dout;    dev_ack = dsp_ack;    end
		else if (sel_mfp)    begin dev_dout = mfp_dout;    dev_ack = mfp_ack;    end
		else if (sel_acia)   begin dev_dout = acia_dout;   dev_ack = acia_ack;   end
		else if (sel_void)   begin dev_dout = 16'hFFFF;    dev_ack = dev_stb;    end
		else dev_berr = 1;
	end
end

`define DEVBUS(sel) .bus_cs(dev_cs & (sel)), .bus_stb(dev_stb & (sel)), .bus_we(dev_we), .bus_uds(dev_uds), .bus_lds(dev_lds), .bus_din(dev_din)

//////////////////////////////////////////////////////////////////
//  Devices
//////////////////////////////////////////////////////////////////

falcon_combel combel
(
	.clk(clk), .reset(dev_reset), .cold_reset(cold_reset),
	.ram_mb(ram_mb), .monitor(monitor),
	`DEVBUS(sel_combel), .bus_addr(dev_addr), .bus_dout(combel_dout), .bus_ack(combel_ack),
	.joy0(joy0), .joy1(joy1), .ana0(ana0), .ana1(ana1),
	.falcon_bus(falcon_bus), .cpu_16mhz(cpu_16mhz)
);

// ---- Videl ----
wire videl_vbl, videl_hbl, videl_de, videl_de_tb;
falcon_videl #(.CLK_HZ(CLK_HZ)) videl
(
	.clk(clk), .reset(dev_reset),
	.bus_cs(dev_cs & sel_videl), .bus_stb(dev_stb & sel_videl),
	.pal_cs(dev_cs & sel_pal), .pal_stb(dev_stb & sel_pal),
	.bus_we(dev_we), .bus_uds(dev_uds), .bus_lds(dev_lds), .bus_din(dev_din),
	.bus_addr(dev_addr[10:1]), .bus_dout(videl_dout), .bus_ack(videl_ack),
	.monitor_type(monitor),
	.vid_req(vid_req), .vid_addr(vid_addr), .vid_ack(vid_ack), .vid_data(vid_data), .vid_valid(vid_valid),
	.r(r), .g(g), .b(b), .hsync(hsync), .vsync(vsync), .hblank(hblank), .vblank(vblank),
	.ce_pix(ce_pix), .de(videl_de), .de_tb(videl_de_tb), .vbl(videl_vbl), .hbl(videl_hbl)
);

// ---- PSG ----
wire  [7:0] porta, portb;
wire signed [15:0] psg_audio;
wire        [15:0] psg_raw;
wire               psg_stb;
falcon_psg #(.CLK_HZ(CLK_HZ), .MIRROR(0)) psg
(
	.clk(clk), .reset(dev_reset),
	`DEVBUS(sel_psg), .bus_addr(dev_addr[7:1]), .bus_dout(psg_dout), .bus_ack(psg_ack),
	.port_a_in(8'hFF), .port_b_in(8'hFF),
	.port_a_out(porta), .port_b_out(portb), .port_a_oe(), .port_b_oe(),
	.ch_a(), .ch_b(), .ch_c(),
	.snd_sample(psg_raw), .snd_stb(psg_stb)
);

// The PSG level is Hatari's 0..32767 table value; remove its DC offset
// with a one-pole high-pass (about 5 Hz at the 250 kHz sample rate), as
// Hatari's Falcon path does before the codec.
reg  signed [31:0] psg_dc;
wire signed [16:0] psg_ac = $signed({2'b00, psg_raw[14:0]}) - $signed(psg_dc[31:15]);
always @(posedge clk) begin
	if (reset) psg_dc <= 32'sd16384 <<< 15;
	else if (psg_stb) psg_dc <= psg_dc + ((($signed({2'b00, psg_raw[14:0]}) <<< 15) - psg_dc) >>> 13);
end
assign psg_audio = (psg_ac > 17'sd32767) ? 16'h7FFF : (psg_ac < -17'sd32768) ? 16'h8000 : psg_ac[15:0];

// ---- MFP ----
wire       mfp_irq, mfp_iack_ack, mfp_iack_spur;
reg        mfp_iack;
wire [7:0] mfp_vector;
wire       acia_irq, fdc_irq, ide_irq, blit_busy, sndint, soundint;
wire       mfp_tdo;
falcon_mfp #(.CLK_HZ(CLK_HZ)) mfp
(
	.clk(clk), .reset(dev_reset),
	`DEVBUS(sel_mfp), .bus_addr(dev_addr[5:1]), .bus_dout(mfp_dout), .bus_ack(mfp_ack),
	.irq(mfp_irq), .iack(mfp_iack), .iack_vector(mfp_vector), .iack_ack(mfp_iack_ack), .iack_spurious(mfp_iack_spur),
	.gpip_in({sndint, 1'b1, ~(fdc_irq | ide_irq), ~acia_irq, blit_busy, 1'b1, 1'b1, 1'b1}),   // GPIP3 high while the blitter runs (blitter.c)
	.gpip_out(), .gpip_oe(),
	.tai(soundint), .tbi(videl_de_tb),   // one event per source line (Hatari's ST line timing)
	.tao(), .tbo(), .tco(), .tdo(mfp_tdo),
	.si(ser_rx), .so(ser_tx)
);

// ---- ACIAs + IKBD ----
falcon_acia #(.CLK_HZ(CLK_HZ)) acia
(
	.clk(clk), .reset(dev_reset),
	`DEVBUS(sel_acia), .bus_addr(dev_addr[2:1]), .bus_dout(acia_dout), .bus_ack(acia_ack),
	.irq(acia_irq),
	.ps2_key(ps2_key), .ps2_mouse(ps2_mouse), .joystick_0(joy1), .joystick_1(joy0),   // MiSTer joystick 1 -> ST port 1 (games), 2 -> port 0 (mouse port)
	.midi_rx(midi_rx), .midi_tx(midi_tx)
);

// ---- NVRAM / RTC ----
// Monitor changes must not erase saved language, keyboard or boot settings.
// Defaults use the selected monitor when explicitly initialized by the HPS.

falcon_nvram #(.CLK_HZ(CLK_HZ)) nvram
(
	.clk(clk), .reset(dev_reset),
	`DEVBUS(sel_nvram), .bus_addr(dev_addr[1:1]), .bus_dout(nvram_dout), .bus_ack(nvram_ack),
	.rtc(rtc),
	.cfg_vga(monitor == 2'b10), .cfg_lang(8'd0), .cfg_kbd(8'd0), .nv_init(nv_init),
	.nv_addr(nv_addr), .nv_dout(nv_dout), .nv_din(nv_din), .nv_wr(nv_wr),
	.nv_changed(nv_changed), .nv_ready(nv_ready),
	.irq()
);

`ifndef FALCON_NO_IDE
// ---- IDE ----
wire ide_led;
assign hdd_led = ide_led | scsi_led;
falcon_ide ide
(
	.clk(clk), .reset(dev_reset),
	`DEVBUS(sel_ide), .bus_addr(dev_addr[5:1]), .bus_dout(ide_dout), .bus_ack(ide_ack),
	.irq(ide_irq),
	.img_mounted(img_mounted[3:2]), .img_readonly(img_readonly), .img_size(img_size),
	.sd_lba0(sd_lba[2]), .sd_lba1(sd_lba[3]), .sd_rd(sd_rd[3:2]), .sd_wr(sd_wr[3:2]), .sd_ack(sd_ack[3:2]),
	.sd_buff_addr(sd_buff_addr[8:0]), .sd_buff_dout(sd_buff_dout),
	.sd_buff_din0(sd_buff_din[2]), .sd_buff_din1(sd_buff_din[3]), .sd_buff_wr(sd_buff_wr),
	.led(ide_led)
);

`else
// bring-up build without the IDE controller
assign ide_dout = 16'hFFFF; assign ide_ack = dev_stb & sel_ide; assign ide_irq = 0; assign ide_led = 0;
assign sd_lba[2] = 0; assign sd_lba[3] = 0; assign sd_rd[3:2] = 0; assign sd_wr[3:2] = 0;
assign sd_buff_din[2] = 0; assign sd_buff_din[3] = 0;
`endif

`ifndef FALCON_NO_FDC
// ---- FDC + DMA ----
wire        hdc_acc, hdc_we, hdc_irq_set, hdc_irq_clr, scsi_led;
wire  [5:0] scsi_blk_cnt;
assign sd_blk_cnt[0] = 6'd0; assign sd_blk_cnt[1] = 6'd0; assign sd_blk_cnt[2] = 6'd0; assign sd_blk_cnt[3] = 6'd0;
assign sd_blk_cnt[4] = scsi_blk_cnt; assign sd_blk_cnt[5] = scsi_blk_cnt; assign sd_blk_cnt[6] = scsi_blk_cnt;
wire  [2:0] hdc_rs;
wire  [7:0] hdc_wdata, hdc_rdata, hdc_push_byte, hdc_pull_byte;
wire        hdc_push_req, hdc_push_ack, hdc_pull_req, hdc_pull_ack, hdc_flush;
falcon_fdc #(.CLK_HZ(CLK_HZ), .EXT_SCSI(1)) fdc
(
	.clk(clk), .reset(dev_reset),
	`DEVBUS(sel_fdc), .bus_addr(dev_addr[3:1]), .bus_dout(fdc_dout), .bus_ack(fdc_ack), .bus_berr(fdc_berr),
	.irq(fdc_irq),
	.drv_sel(porta[2:1]), .side_sel(porta[0]),
	.dma_req(fdc_dreq), .dma_we(fdc_dwe), .dma_addr(fdc_daddr), .dma_be(fdc_dbe),
	.dma_wdata(fdc_dwdata), .dma_rdata(fdc_drdata), .dma_ack(fdc_dack),
	.img_mounted(img_mounted[1:0]), .img_readonly(img_readonly), .img_size(img_size),
	.sd_lba0(sd_lba[0]), .sd_lba1(sd_lba[1]), .sd_rd(sd_rd[1:0]), .sd_wr(sd_wr[1:0]), .sd_ack(sd_ack[1:0]),
	.sd_buff_addr(sd_buff_addr[8:0]), .sd_buff_dout(sd_buff_dout),
	.sd_buff_din0(sd_buff_din[0]), .sd_buff_din1(sd_buff_din[1]), .sd_buff_wr(sd_buff_wr),
	.led(fdd_led),
	.hdc_acc(hdc_acc), .hdc_we(hdc_we), .hdc_rs(hdc_rs), .hdc_wdata(hdc_wdata),
	.hdc_rdata(hdc_rdata), .hdc_irq_set(hdc_irq_set), .hdc_irq_clr(hdc_irq_clr),
	.hdc_push_req(hdc_push_req), .hdc_push_byte(hdc_push_byte), .hdc_push_ack(hdc_push_ack),
	.hdc_pull_req(hdc_pull_req), .hdc_pull_byte(hdc_pull_byte), .hdc_pull_ack(hdc_pull_ack),
	.hdc_flush(hdc_flush)
);

// ---- SCSI: NCR 5380 behind the DMA chip ($FF8604 with DMA mode bit 3) ----
falcon_scsi #(.CLK_HZ(CLK_HZ)) scsi
(
	.clk(clk), .reset(dev_reset),
	.acc(hdc_acc), .we(hdc_we), .rs(hdc_rs), .wdata(hdc_wdata), .rdata(hdc_rdata),
	.irq_set(hdc_irq_set), .irq_clr(hdc_irq_clr),
	.dma_push_req(hdc_push_req), .dma_push_byte(hdc_push_byte), .dma_push_ack(hdc_push_ack),
	.dma_pull_req(hdc_pull_req), .dma_pull_byte(hdc_pull_byte), .dma_pull_ack(hdc_pull_ack),
	.dma_flush(hdc_flush),
	.img_mounted(img_mounted[6:4]), .img_readonly(img_readonly), .img_size(img_size),
	.sd_lba0(sd_lba[4]), .sd_lba1(sd_lba[5]), .sd_lba2(sd_lba[6]),
	.sd_rd(sd_rd[6:4]), .sd_wr(sd_wr[6:4]), .sd_ack(sd_ack[6:4]), .sd_blk_cnt(scsi_blk_cnt),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout),
	.sd_buff_din0(sd_buff_din[4]), .sd_buff_din1(sd_buff_din[5]), .sd_buff_din2(sd_buff_din[6]),
	.sd_buff_wr(sd_buff_wr),
	.led(scsi_led)
);

`else
// bring-up build without the floppy controller
assign fdc_dout = 16'hFFFF; assign fdc_ack = dev_stb & sel_fdc; assign fdc_berr = 0; assign fdc_irq = 0; assign fdd_led = 0;
assign fdc_dreq = 0; assign fdc_dwe = 0; assign fdc_daddr = 0; assign fdc_dbe = 0; assign fdc_dwdata = 0;
assign sd_lba[0] = 0; assign sd_lba[1] = 0; assign sd_rd[1:0] = 0; assign sd_wr[1:0] = 0;
assign sd_buff_din[0] = 0; assign sd_buff_din[1] = 0;
assign sd_blk_cnt[0] = 0; assign sd_blk_cnt[1] = 0; assign sd_blk_cnt[2] = 0; assign sd_blk_cnt[3] = 0;
assign sd_blk_cnt[4] = 0; assign sd_blk_cnt[5] = 0; assign sd_blk_cnt[6] = 0;
`endif

// ---- Blitter: owns the bus through BR/BG/BGACK ----
wire blit_br;
reg  blit_bg = 0;
falcon_blitter blitter
(
	.clk(clk), .reset(dev_reset),
	`DEVBUS(sel_blit), .bus_addr(dev_addr[5:1]), .bus_dout(blit_dout), .bus_ack(blit_ack),
	.br(blit_br), .bg(blit_bg), .cpu_bus_cycle(cpu_cycle_done),
	.fmode(cpu_fmode), .ftick(cpu_ce | cpu_idle_tick),
	.busy(blit_busy),
	.dma_req(blt_req), .dma_we(blt_we), .dma_addr(blt_addr), .dma_be(blt_be),
	.dma_wdata(blt_wdata), .dma_rdata(blt_rdata), .dma_ack(blt_ack)
);

// MC68030 arbitration (UM 7.7): BR -> BG -> the new master asserts BGACK,
// negates BR, and keeps BGACK while it owns the bus
always @(posedge clk) begin
	if (reset) begin
		cpu_br_n <= 1; cpu_bgack_n <= 1; blit_bg <= 0;
	end
	else if (!blit_bg) begin
		cpu_br_n <= ~blit_br;
		if (blit_br && !cpu_bg_n && cpu_as_n && !cpu_wbuf_busy) begin   // a posted CPU write first
			cpu_bgack_n <= 0;
			cpu_br_n    <= 1;
			blit_bg     <= 1;
		end
	end
	else if (!blit_br) begin
		cpu_bgack_n <= 1;
		blit_bg     <= 0;
	end
end

// ---- DMA sound / crossbar / codec ----
wire        ssi_slot_stb, ssi_frame, ssi_tx_valid, ssi_tx_en, ssi_rx_en, ssi_rx_frame, ssi_hs_play_req;
wire [15:0] ssi_rx_data, ssi_tx_data;
falcon_crossbar #(.CLK_HZ(CLK_HZ)) crossbar
(
	.clk(clk), .reset(dev_reset),
	`DEVBUS(sel_xbar), .bus_addr(dev_addr[6:1]), .bus_dout(xbar_dout), .bus_ack(xbar_ack), .bus_berr(xbar_berr),
	.dma_req(snd_req), .dma_we(snd_we), .dma_addr(snd_addr), .dma_be(snd_be),
	.dma_wdata(snd_wdata), .dma_rdata(snd_rdata), .dma_ack(snd_ack),
	.sndint(sndint), .soundint(soundint),
	.psg_audio(psg_audio), .mic_l(16'sd0), .mic_r(16'sd0),
	.audio_l(audio_l), .audio_r(audio_r), .audio_stb(),
	.ssi_slot_stb(ssi_slot_stb), .ssi_frame(ssi_frame), .ssi_rx_data(ssi_rx_data),
	.ssi_tx_data(ssi_tx_data), .ssi_tx_valid(ssi_tx_valid),
	.ssi_tx_en(ssi_tx_en), .ssi_rx_en(ssi_rx_en), .ssi_rx_frame(ssi_rx_frame), .ssi_hs_play_req(ssi_hs_play_req), .dbg_underrun()
);

`ifndef FALCON_NO_DSP
// ---- DSP56001 ----
wire       dsp_hreq;
wire [7:0] dsp_ivr;
falcon_dsp dsp
(
	.clk(clk), .reset(dev_reset), .dsp_reset(dev_reset | porta[4]),
	`DEVBUS(sel_dsp), .bus_addr(dev_addr[2:1]), .bus_dout(dsp_dout), .bus_ack(dsp_ack),
	.hreq(dsp_hreq), .ivr(dsp_ivr), .iack(1'b0),
	.ssi_slot_stb(ssi_slot_stb), .ssi_frame(ssi_frame), .ssi_rx_data(ssi_rx_data),
	.ssi_tx_data(ssi_tx_data), .ssi_tx_valid(ssi_tx_valid),
	.ssi_tx_en(ssi_tx_en), .ssi_rx_en(ssi_rx_en), .ssi_rx_frame(ssi_rx_frame),
	.ssi_hs_play_req(ssi_hs_play_req)
);

`else
// bring-up build without the DSP
wire       dsp_hreq = 0;
wire [7:0] dsp_ivr = 8'h0F;
assign dsp_dout = 16'hFFFF; assign dsp_ack = dev_stb & sel_dsp;
assign ssi_tx_data = 0; assign ssi_tx_valid = 0; assign ssi_hs_play_req = 0;
`endif

//////////////////////////////////////////////////////////////////
//  Interrupts (M68000_Update_intlev)
//////////////////////////////////////////////////////////////////

reg vbl_pend, hbl_pend;
reg [2:0] ipl;
always @* begin
	if (mfp_irq | dsp_hreq) ipl = 3'd6;
	else if (vbl_pend)      ipl = 3'd4;
	else if (hbl_pend)      ipl = 3'd2;
	else                    ipl = 3'd0;
end
assign ipl_n = ~ipl;

localparam I_IDLE = 2'd0, I_MFP = 2'd1;
reg [1:0] ist;

always @(posedge clk) begin
	iack_done <= 0; iack_avec <= 0; iack_spur <= 0; iack_mfp <= 0;
	mfp_iack  <= 0;

	if (videl_vbl) vbl_pend <= 1;
	if (videl_hbl) hbl_pend <= 1;

	if (reset) begin
		vbl_pend <= 0; hbl_pend <= 0; ist <= I_IDLE;
	end
	else case (ist)
	I_IDLE:
		if (iack_req) begin
			case (iack_level)
				3'd6:
					if (mfp_irq) begin mfp_iack <= 1; ist <= I_MFP; end
					else if (dsp_hreq) begin iack_vector <= dsp_ivr; iack_done <= 1; end
					else begin iack_spur <= 1; iack_done <= 1; end
				3'd4: begin vbl_pend <= videl_vbl; iack_avec <= 1; iack_done <= 1; end
				3'd2: begin hbl_pend <= videl_hbl; iack_avec <= 1; iack_done <= 1; end
				default: begin iack_spur <= 1; iack_done <= 1; end
			endcase
		end
	I_MFP:
		if (mfp_iack_ack) begin
			iack_vector <= mfp_vector;     // $18 (spurious) when the request vanished
			iack_done   <= 1;
			iack_mfp    <= 1;
			ist         <= I_IDLE;
		end
	default: ist <= I_IDLE;
	endcase
end

endmodule
