//============================================================================
//  Full-system testbench top: the real falcon_system on a DDR3 model.
//  The C++ driver (sim_main.cpp) provides the clock, the hps_io disk block
//  protocol, keyboard/mouse events and captures the video output.
//============================================================================

module tb_top
(
	input             clk,
	input             clk_cpu,      // ap060: the processor's 50 MHz, asynchronous to clk
	input             reset,
	input             cold_reset,
	input             por,
	input       [3:0] ram_mb,
	input             ram_tos,
	input       [1:0] monitor,
	input             cpu_turbo,
	input             tstat_en,
	input       [1:0] fast_ram,     // ap060 CT60 mode: 0 off, 1 64 MB, 2 128 MB, 3 240 MB

	input      [10:0] ps2_key,
	input      [24:0] ps2_mouse,
	input      [31:0] joy0,
	input      [64:0] rtc,

	input       [6:0] img_mounted,
	input             img_readonly,
	input      [63:0] img_size,
	output    [223:0] sd_lba_f,
	output      [6:0] sd_rd,
	output      [6:0] sd_wr,
	input       [6:0] sd_ack,
	input      [13:0] sd_buff_addr,
	output     [41:0] sd_blk_cnt_f,
	input       [7:0] sd_buff_dout,
	output     [55:0] sd_buff_din_f,
	input             sd_buff_wr,

	output      [7:0] r,
	output      [7:0] g,
	output      [7:0] b,
	output            hsync,
	output            vsync,
	output            hblank,
	output            vblank,
	output            ce_pix,
	output signed [15:0] audio_l,
	output signed [15:0] audio_r,
	output            ser_tx,
	output            midi_tx,

	output     [31:0] dbg_pc,
	output            dbg_retire,   // dbg_pc is a retired instruction's this clock
	output            dbg_busown,   // the bus bridge owns the bus (ap060: snooping DMA writes / the blitter)
	output            dbg_snoop,    // a snoop transfer this clock
	output            dbg_halted,
	output      [2:0] dbg_ipl,
	output            dbg_as,
	output     [31:0] dbg_a,
	output      [2:0] dbg_fc,
	output            dbg_rw,
	output            dbg_berr,
	output            dbg_dev_stb,
	output     [23:0] dbg_dev_addr,
	output            dbg_dev_we,
	output            dbg_dev_uds,
	output            dbg_dev_lds,
	output     [15:0] dbg_dev_din,
	output            dbg_dev_ack,
	output     [15:0] dbg_dev_dout,
	output            dbg_mfp_irq,
	output            dbg_cp_req,
	output            dbg_cp_we,
	output      [4:0] dbg_cp_off,
	output     [31:0] dbg_cp_wdata,
	output            dbg_cp_ack,
	output            dbg_cp_berr,
	output     [31:0] dbg_cp_rdata,
	output            dbg_fpu_present
);

assign dbg_pc     = system.dbg_pc;
assign dbg_retire = system.cpu_inst;
`ifdef FALCON_CPU060
assign dbg_busown = system.cpubus.owned;
assign dbg_snoop  = ~system.cpubus.ts_n_i;
`else
assign dbg_busown = system.blit_bg;
assign dbg_snoop  = 1'b0;
`endif
assign dbg_halted = system.cpu_halted;
assign dbg_ipl    = system.ipl;
assign dbg_as     = ~system.cpu_as_n & system.cpu_bus_oe;
assign dbg_a      = system.cpu_a;
assign dbg_fc     = system.cpu_fc;
assign dbg_rw     = system.cpu_rw;
assign dbg_berr   = ~system.berr_n;
assign dbg_dev_stb  = system.dev_stb;
assign dbg_dev_addr = {system.dev_addr, 1'b0};
assign dbg_dev_we   = system.dev_we;
assign dbg_dev_uds  = system.dev_uds;
assign dbg_dev_lds  = system.dev_lds;
assign dbg_dev_din  = system.dev_din;
assign dbg_dev_ack  = system.dev_ack & system.dev_cs;
assign dbg_dev_dout = system.dev_dout;
assign dbg_mfp_irq  = system.mfp_irq;
assign dbg_cp_req   = system.cp_req;
assign dbg_cp_we    = system.cp_we;
assign dbg_cp_off   = system.cp_off;
assign dbg_cp_wdata = system.cp_wdata;
assign dbg_cp_ack   = system.cp_ack;
assign dbg_cp_berr  = system.cp_berr;
assign dbg_cp_rdata = system.cp_rdata;
assign dbg_fpu_present = system.fpu_present;

wire        DDRAM_BUSY, DDRAM_DOUT_READY, DDRAM_RD, DDRAM_WE;
wire  [7:0] DDRAM_BURSTCNT, DDRAM_BE;
wire [28:0] DDRAM_ADDR;
wire [63:0] DDRAM_DOUT, DDRAM_DIN;

wire [31:0] sd_lba[7];
wire  [7:0] sd_buff_din[7];
wire  [5:0] sd_blk_cnt[7];
assign sd_blk_cnt_f = {sd_blk_cnt[6], sd_blk_cnt[5], sd_blk_cnt[4], sd_blk_cnt[3], sd_blk_cnt[2], sd_blk_cnt[1], sd_blk_cnt[0]};
assign sd_lba_f      = {sd_lba[6], sd_lba[5], sd_lba[4], sd_lba[3], sd_lba[2], sd_lba[1], sd_lba[0]};
assign sd_buff_din_f = {sd_buff_din[6], sd_buff_din[5], sd_buff_din[4], sd_buff_din[3], sd_buff_din[2], sd_buff_din[1], sd_buff_din[0]};

falcon_system #(.CLK_HZ(32000000)) system
(
	.clk(clk), .clk_cpu(clk_cpu), .reset(reset), .cold_reset(cold_reset), .por(por),
	.ram_mb(ram_mb), .ram_tos(ram_tos), .monitor(monitor), .cpu_turbo(cpu_turbo), .tstat_en(tstat_en), .fast_ram(fast_ram),
	.ld_wr(1'b0), .ld_addr(24'd0), .ld_data(8'd0), .ld_busy(),
	.ps2_key(ps2_key), .ps2_mouse(ps2_mouse), .joy0(joy0), .joy1(32'd0), .ana0(16'd0), .ana1(16'd0), .rtc(rtc),
	.nv_init(1'b0), .nv_addr(6'd0), .nv_din(8'd0), .nv_wr(1'b0),
	.nv_dout(), .nv_changed(), .nv_ready(),
	.img_mounted(img_mounted), .img_readonly(img_readonly), .img_size(img_size),
	.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack), .sd_blk_cnt(sd_blk_cnt),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
	.r(r), .g(g), .b(b), .hsync(hsync), .vsync(vsync), .hblank(hblank), .vblank(vblank), .ce_pix(ce_pix),
	.mic_l(16'sd0), .mic_r(16'sd0),
	.audio_l(audio_l), .audio_r(audio_r),
	.midi_rx(1'b1), .midi_tx(midi_tx), .ser_rx(1'b1), .ser_tx(ser_tx),
	.fdd_led(), .hdd_led(),
	.DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE)
);

ddr3_model ddr
(
	.clk(clk),
	.DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE)
);

`ifdef FALCON_CPU060
`ifdef FALCON_CPU040
final $display("CPU clock: turbo (AP68040)");
`else
final $display("CPU clock: turbo (AP68060)");
`endif
// the snooper's data cache queue lost invalidations (sticky)
reg ovf_seen = 0;
`ifndef FALCON_CPU040
always @(posedge clk_cpu) if (system.cpu.sn_ovf && !ovf_seen) begin
	ovf_seen <= 1;
	$display("[%9.3f ms] SNOOP QUEUE OVERFLOW: data cache invalidations lost", clk_n / 32000.0);
end
`endif
// the AP68060's fetch redirects into the vector area and the snoop
// refetches, with their cause (bring-up debugging)
integer clk_n = 0;              // system clocks (the time stamps)
always @(posedge clk) clk_n <= clk_n + 1;
reg        d_ref = 0, d_xgo = 0, d_redir = 0;
reg  [7:0] d_xvec = 0, d_wbexc = 0;
reg [31:0] d_wbpc = 0;
always @(posedge clk_cpu) begin
	d_ref   <= system.cpu.be.wb_refetch;
	d_xgo   <= system.cpu.be.x_go;
	d_xvec  <= system.cpu.be.x_vec;
	d_wbpc  <= system.cpu.be.wb_u.pc;
	d_wbexc <= system.cpu.be.wb_exc;
	d_redir <= system.cpu.be.wb_commit && system.cpu.be.wb_redir;
	if (system.cpu.be.redir_v && system.cpu.be.redir_pc < 32'h1000)
		$display("[%9.3f ms] redirect to %08x: refetch %b, exception %b (vector %0d), branch %b; WB pc %08x exc %02x; snp_pend %b",
		         clk_n / 32000.0, system.cpu.be.redir_pc, d_ref, d_xgo, d_xvec, d_redir, d_wbpc, d_wbexc, system.cpu.be.snp_pend);
	if (system.cpu.be.ag_snref)
		$display("[%9.3f ms] snoop refetch marks the uop at AG: pc %08x op %0d first %b last %b exc %02x ser %b",
		         clk_n / 32000.0, system.cpu.be.ag_u.pc, system.cpu.be.ag_u.op,
		         system.cpu.be.ag_u.first, system.cpu.be.ag_u.last, system.cpu.be.ag_u.exc, system.cpu.be.ag_u.ser);
	if (system.cpu.sn_ihit)
		$display("[%9.3f ms] snoop hit on the instruction cache (every line: %b) pc %08x", clk_n / 32000.0, system.cpu.fetch.inv_scope == 2'd3, dbg_pc);
end
`else
// the CPU clock's time accounting (falcon_cpuclk), in system clocks; and
// what would make Falcon time depend on real time: wait states the bridge
// could not avoid (credit), fetches let through by the AP68030's guard
// against a stop that never ends, scans given up (falcon_pipescan)
integer n_credit = 0, n_starve = 0, n_gone = 0, lag_max = 0;   // lag: words the scan is behind the queue head
always @(posedge clk) begin
	n_credit <= n_credit + system.cpu_credit;
	if (system.cpu_ce && system.cpu.core.stop_starve) n_starve <= n_starve + 1;
	if (system.cpu_ce && system.pipescan.act && system.pipescan.gone && !system.pipescan.flush) n_gone <= n_gone + 1;
	if (system.cpu_ce && system.pipescan.act && !system.pipescan.flush && $signed(system.pipescan.off) < 0 &&
	    -($signed(system.pipescan.off) / 2) > lag_max) lag_max <= -($signed(system.pipescan.off) / 2);
end
final $display("CPU clock: %s, debt peak %0d, forgiven %0d, held %0d; credit %0d, stop guard %0d, scans lost %0d, scan lag %0d words",
               system.cpu_fmode ? "Falcon" : "turbo", system.cpuclk.debt_peak,
               system.cpuclk.forgiven, system.cpuclk.held, n_credit, n_starve, n_gone, lag_max);

`endif

// activity per 100 ms (+actlog): instructions retired, CPU bus transfers,
// clocks the blitter is busy and (ap060) clocks the bridge holds the bus
integer a_clk = 0, a_inst = 0, a_cyc = 0, a_blt = 0, a_own = 0;
bit actlog = 0;
initial actlog = $test$plusargs("actlog");
always @(posedge clk) if (actlog) begin
	a_clk  <= a_clk + 1;
	a_inst <= a_inst + (system.cpu_inst ? 1 : 0);
	a_cyc  <= a_cyc + (system.cpu_cycle_done ? 1 : 0);
	a_blt  <= a_blt + (system.blit_busy ? 1 : 0);
`ifdef FALCON_CPU060
	a_own  <= a_own + (system.cpubus.owned ? 1 : 0);
`endif
	if (a_clk == 3200000 - 1) begin
		$display("ACT inst %0d  bus %0d  blitter-busy %0d  bus-held %0d (per 100 ms)", a_inst, a_cyc, a_blt, a_own);
		a_clk <= 0; a_inst <= 0; a_cyc <= 0; a_blt <= 0; a_own <= 0;
	end
end

// IKBD interrupt path (+ikbdmon): bytes the IKBD ACIA receives, overruns,
// RDR reads by the CPU, MFP channel 6 (GPIP4) events and acknowledges; and
// the stuck case: the ACIA requests for more than 20 ms without being read
integer k_rx = 0, k_ovr = 0, k_rd = 0, k_ev = 0, k_ack = 0, k_stuck = 0;
reg     k_rdrf_d = 0, k_ovr_d = 0, k_rd_d = 0, k_rep = 0;
bit     ikbdmon = 0;
initial ikbdmon = $test$plusargs("ikbdmon");
integer k_clk = 0;
always @(posedge clk) if (ikbdmon) begin
	k_clk    <= k_clk + 1;
	k_rdrf_d <= system.acia.u_acia_ikbd.rdrf;
	k_ovr_d  <= system.acia.u_acia_ikbd.ovr_pend;
	k_rd_d   <= system.acia.ik_rd_dr;
	if (system.acia.u_acia_ikbd.rdrf && !k_rdrf_d) k_rx <= k_rx + 1;
	if (system.acia.u_acia_ikbd.ovr_pend && !k_ovr_d) begin
		k_ovr <= k_ovr + 1;
		$display("[%9.3f ms] IKBD ACIA overrun", k_clk / 32000.0);
	end
	if (system.acia.ik_rd_dr && !k_rd_d) k_rd <= k_rd + 1;
	if (system.mfp.gp_ev[4]) k_ev <= k_ev + 1;
	if (system.mfp.ack_ch && system.mfp.pend_m[3:0] == 4'd6) k_ack <= k_ack + 1;
	if (system.acia.irq_ikbd) k_stuck <= k_stuck + 1; else k_stuck <= 0;
	if (k_stuck == 640000 && !k_rep) begin
		k_rep <= 1;
		$display("[%9.3f ms] IKBD STUCK: ACIA irq 20 ms; MFP ier %04x ipr %04x isr %04x imr %04x vr %02x aer %02x gpip4 %b mfp_irq %b ipl %0d pc %08x",
		         k_clk / 32000.0, system.mfp.ier, system.mfp.ipr, system.mfp.isr, system.mfp.imr, {system.mfp.vr, 3'b0},
		         system.mfp.aer, system.mfp.gpip_in[4], system.mfp_irq, system.ipl, system.dbg_pc);
	end
	if (k_rep && !system.acia.irq_ikbd) begin
		k_rep <= 0;
		$display("[%9.3f ms] IKBD unstuck", k_clk / 32000.0);
	end
	if (k_clk % 3200000 == 0)
		$display("[%9.3f ms] IKBD rx %0d ovr %0d rdr reads %0d gpip4 events %0d ch6 acks %0d", k_clk / 32000.0, k_rx, k_ovr, k_rd, k_ev, k_ack);
end
final if (ikbdmon) $display("IKBD rx %0d ovr %0d rdr reads %0d gpip4 events %0d ch6 acks %0d", k_rx, k_ovr, k_rd, k_ev, k_ack);

// IDE commands (+idemon): each command as it starts (code, sector count,
// LBA), each disk image request (unit, LBA, blocks), and per 100 ms totals
bit idemon = 0;
initial idemon = $test$plusargs("idemon");
integer i_clk = 0, i_cmd = 0, i_req = 0, i_blk = 0, i_sec = 0;
reg [1:0] i_rd_d = 0;
`ifndef FALCON_NO_IDE
always @(posedge clk) if (idemon) begin
	i_clk  <= i_clk + 1;
	i_rd_d <= system.ide.sd_rd;
	if (system.ide.cmd_req && ((system.ide.est == system.ide.E_IDLE && !system.ide.h_act) || system.ide.est == system.ide.E_RDRQ ||
	    system.ide.est == system.ide.E_WDRQ || system.ide.est == system.ide.E_IDDRQ)) begin
		i_cmd <= i_cmd + 1;
		i_sec <= i_sec + (system.ide.r_nsec == 0 ? 256 : system.ide.r_nsec);
		$display("[%9.3f ms] IDE cmd %02x unit %0d nsec %0d lba %0d pc %08x", i_clk / 32000.0, system.ide.cmd_val, system.ide.cmd_unit,
		         system.ide.r_nsec, {system.ide.r_select[3:0], system.ide.r_hcyl, system.ide.r_lcyl, system.ide.r_sect}, system.dbg_pc);
	end
	if ((system.ide.sd_rd & ~i_rd_d) != 0) begin
		i_req <= i_req + 1;
		i_blk <= i_blk + system.ide.sd_blk_cnt + 1;
	end
	if (i_clk % 3200000 == 0)
		$display("[%9.3f ms] IDE totals: %0d commands, %0d sectors asked, %0d image reads, %0d blocks", i_clk / 32000.0, i_cmd, i_sec, i_req, i_blk);
end
`endif

// CPU bus transfers by region (+busmon, from +busfrom=<ms>): per 100 ms,
// line and non-line, reads and writes, ST-RAM / ROM+I/O / Fast RAM, the
// cache-inhibited ones, and the retired instructions; plus the most recent
// non-line write and read addresses (where the uncached traffic goes)
`ifdef FALCON_CPU060
bit busmon = 0;
integer busfrom = 0;
initial begin busmon = $test$plusargs("busmon"); if (!$value$plusargs("busfrom=%d", busfrom)) busfrom = 0; end
integer b_clk = 0, b_inst = 0;
integer b_n[0:2][0:1][0:1];     // [region][line][write]
integer b_ci = 0;
reg [31:0] b_lastw = 0, b_lastr = 0;
initial for (int r = 0; r < 3; r++) for (int l = 0; l < 2; l++) for (int w = 0; w < 2; w++) b_n[r][l][w] = 0;
always @(posedge clk) if (busmon) b_clk <= b_clk + 1;
integer b_per = 0;               // the 100 ms period last reported
always @(posedge clk_cpu) if (busmon && b_clk >= busfrom * 32000) begin
	automatic int r, l, w;
	if (b_clk / 3200000 != b_per) begin
		b_per <= b_clk / 3200000;
		$display("[%9.3f ms] BUS inst %0d | ST line r %0d w %0d, single r %0d w %0d | ROM/IO single r %0d w %0d | FAST line r %0d w %0d, single r %0d w %0d | CI %0d | last single w %08x r %08x",
		         b_clk / 32000.0, b_inst, b_n[0][1][0], b_n[0][1][1], b_n[0][0][0], b_n[0][0][1], b_n[1][0][0], b_n[1][0][1],
		         b_n[2][1][0], b_n[2][1][1], b_n[2][0][0], b_n[2][0][1], b_ci, b_lastw, b_lastr);
		b_inst <= 0; b_ci <= 0;
		for (int i = 0; i < 3; i++) for (int j = 0; j < 2; j++) for (int k = 0; k < 2; k++) b_n[i][j][k] <= 0;
	end
	else begin
		if (system.cpu_inst) b_inst <= b_inst + 1;
		if (!system.cpu060_ts_n && system.cpu060_a_oe) begin
			r = system.cpu_a >= 32'h01000000 ? 2 : (system.cpu_a >= 32'h00E00000 ? 1 : 0);
			l = (system.cpu060_siz == 2'b11) ? 1 : 0;
			w = !system.cpu060_rw_n ? 1 : 0;
			b_n[r][l][w] <= b_n[r][l][w] + 1;
			if (!system.cpu060_ciout_n) b_ci <= b_ci + 1;
			if (l == 0 && w == 1) b_lastw <= system.cpu_a;
			if (l == 0 && w == 0) b_lastr <= system.cpu_a;
		end
	end
end
`endif

// the AP68040/AP68060 pipeline (+pipemon, from +busfrom=<ms>): per 100 ms,
// CPU clocks, instructions retired, clocks WB is held by a store the DMU
// has not accepted, clocks WB stalls for any reason, clocks the DMU engine
// is busy, instruction fetch bus requests (line fetches: I-cache misses)
// and the clocks they wait, DMU bus requests and the clocks they wait
`ifdef FALCON_CPU040
bit pipemon = 0;
initial pipemon = $test$plusargs("pipemon");
integer p_clk = 0, p_per = 0;
integer p_cyc = 0, p_inst = 0, p_sthold = 0, p_wbst = 0, p_eng = 0, p_freq = 0, p_fwait = 0, p_dreq = 0, p_dwait = 0;
reg     p_fb_d = 0, p_db_d = 0;
always @(posedge clk) if (pipemon) p_clk <= p_clk + 1;
always @(posedge clk_cpu) if (pipemon && p_clk >= busfrom * 32000) begin
	p_fb_d <= system.cpu.fetch.b_req;
	p_db_d <= system.cpu.dmu.e_breq;
	if (p_clk / 3200000 != p_per) begin
		p_per <= p_clk / 3200000;
		$display("[%9.3f ms] PIPE clk %0d inst %0d | WB held by store %0d, WB stalled %0d | DMU engine busy %0d | fetch bus req %0d wait %0d | DMU bus req %0d wait %0d",
		         p_clk / 32000.0, p_cyc, p_inst, p_sthold, p_wbst, p_eng, p_freq, p_fwait, p_dreq, p_dwait);
		p_cyc <= 0; p_inst <= 0; p_sthold <= 0; p_wbst <= 0; p_eng <= 0; p_freq <= 0; p_fwait <= 0; p_dreq <= 0; p_dwait <= 0;
	end
	else begin
		p_cyc <= p_cyc + 1;
		if (system.cpu_inst) p_inst <= p_inst + 1;
		if (system.cpu.be.stall_wb && system.cpu.be.wb_st && !system.cpu.be.dm_st_rdy) p_sthold <= p_sthold + 1;
		if (system.cpu.be.stall_wb) p_wbst <= p_wbst + 1;
		if (!system.cpu.dmu.sim_es_i) p_eng <= p_eng + 1;
		if (system.cpu.fetch.b_req && !p_fb_d) p_freq <= p_freq + 1;
		if (system.cpu.fetch.b_req) p_fwait <= p_fwait + 1;
		if (system.cpu.dmu.e_breq && !p_db_d) p_dreq <= p_dreq + 1;
		if (system.cpu.dmu.e_breq) p_dwait <= p_dwait + 1;
	end
end
`endif

// one access at a time (+lattrace, from +busfrom=<ms>, LATN CPU clocks): a
// row per CPU clock with the uncached access path's handshakes
`ifdef FALCON_CPU040
bit lattrace = 0;
initial lattrace = $test$plusargs("lattrace");
integer l_clk = 0, l_n = 0;
always @(posedge clk) if (lattrace) l_clk <= l_clk + 1;
always @(posedge clk_cpu) if (lattrace && l_clk >= busfrom * 32000 && l_n < 600) begin
	l_n <= l_n + 1;
	$display("LAT %4d ret=%b pc=%08x | dmu: breq=%b gnt=%b done=%b rv=%b idle=%b | be: wbstall=%b | bus: ts=%b a=%08x rw=%b ta=%b tci=%b | fr: sel=%b nreq=%b nrdy=%b nval=%b ta=%b",
	         l_n, system.cpu_inst, system.dbg_pc,
	         system.cpu.dmu.e_breq, system.cpu.dmu.b_gnt, system.cpu.dmu.b_done, system.cpu.dmu.b_rvalid, system.cpu.dmu.sim_es_i,
	         system.cpu.be.stall_wb,
	         !system.cpu060_ts_n && system.cpu060_a_oe, system.cpu_a, system.cpu060_rw_n, !system.cpu060_ta_n, !system.cpu060_tci_n,
	         system.cpubus.fr_sel, system.cpubus.fastram.n_req, system.cpubus.fastram.n_ready, system.cpubus.fastram.n_valid, !system.cpubus.fr_ta_n);
end
`endif

endmodule
