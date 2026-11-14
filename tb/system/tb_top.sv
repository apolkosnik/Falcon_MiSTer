//============================================================================
//  Full-system testbench top: the real falcon_system on a DDR3 model.
//  The C++ driver (sim_main.cpp) provides the clock, the hps_io disk block
//  protocol, keyboard/mouse events and captures the video output.
//============================================================================

module tb_top
(
	input             clk,
	input             reset,
	input             cold_reset,
	input             por,
	input       [3:0] ram_mb,
	input             ram_tos,
	input       [1:0] monitor,
	input             cpu_turbo,
	input             tt_mode,
	input       [1:0] ttram_sel,
	input             tstat_en,
	input             net_on,
	input       [1:0] vme_sel,

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
	output            dbg_halted,
	output      [2:0] dbg_ipl,
	output            dbg_as,
	output     [31:0] dbg_a,
	output      [2:0] dbg_fc,
	output            dbg_rw,
	output            dbg_berr,
	output     [15:0] dbg_sr,
	output      [7:0] dbg_state,
	output      [3:0] dbg_owner,     // {own_ifetch, i_fc!=d_fc, owner}
	output      [5:0] dbg_cfc,       // {core i_fc, core d_fc}
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
assign dbg_halted = system.cpu_halted;
assign dbg_ipl    = system.ipl;
assign dbg_as     = ~system.cpu_as_n & system.cpu_bus_oe;
assign dbg_a      = system.cpu_a;
assign dbg_fc     = system.cpu_fc;
assign dbg_rw     = system.cpu_rw;
assign dbg_berr   = ~system.berr_n;
assign dbg_sr     = system.cpu.dbg_sr;
assign dbg_state  = system.cpu.dbg_state;
assign dbg_owner  = {system.cpu.memsys.own_ifetch, 1'b0, system.cpu.memsys.owner};
assign dbg_cfc    = {system.cpu.core.i_fc, system.cpu.core.d_fc};
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

// bus fault frames stacked and rerun, for the ASV user-mode fault hunt
reg [63:0] tr_clk = 0;
reg  [7:0] tr_prev = 0;
integer    tr_n = 0;
always @(posedge clk) begin
	tr_clk <= tr_clk + 1;
	tr_prev <= system.cpu.core.state;
	if (system.cpu.core.state != tr_prev && tr_n < 4000) begin
		if (system.cpu.core.state == 8'd75 && (system.cpu.core.exc_vec == 8'd2 || system.cpu.core.exc_vec == 8'd3) && !system.cpu.core.exc_throw) begin
			tr_n = tr_n + 1;
			$display("[%0d.%03d ms] FRAME vec=%0d fmt=%h ssw=%04h fa=%08h sr=%04h pc=%08h pci=%08h rk=%0d sp=%08h",
				tr_clk / 32000, (tr_clk % 32000) / 32, system.cpu.core.exc_vec, system.cpu.core.exc_fmt, system.cpu.core.exc_ssw,
				system.cpu.core.exc_fa, system.cpu.core.exc_sr, system.cpu.core.exc_pc, system.cpu.core.pc_i,
				system.cpu.core.exc_rk, system.cpu.core.exc_sp);
		end
		if (tr_clk > 64'd2336000000 && tr_n < 20000) begin
			if (system.cpu.core.state == 8'd80) begin
				tr_n = tr_n + 1;
				$display("[%0d.%03d ms] RTE0 a7=%08x sr=%04h pci=%08h", tr_clk / 32000, (tr_clk % 32000) / 32,
					system.cpu.core.rf_c, system.cpu.core.sr, system.cpu.core.pc_i);
			end
			if (system.cpu.core.state == 8'd82) begin
				tr_n = tr_n + 1;
				$display("[%0d.%03d ms] RTE2 w0=%08x w1=%08x a7=%08x sr=%04h", tr_clk / 32000, (tr_clk % 32000) / 32,
					system.cpu.core.tmp, system.cpu.core.tmp2, system.cpu.core.rf_c, system.cpu.core.sr);
			end
			if (system.cpu.core.state == 8'd74) begin
				tr_n = tr_n + 1;
				$display("[%0d.%03d ms] EXC vec=%0d fmt=%h sr=%04h pc=%08h pci=%08h irq=%0d thr=%0d", tr_clk / 32000, (tr_clk % 32000) / 32,
					system.cpu.core.exc_vec, system.cpu.core.exc_fmt, system.cpu.core.exc_sr, system.cpu.core.exc_pc,
					system.cpu.core.pc_i, system.cpu.core.exc_is_irq, system.cpu.core.exc_throw);
			end
		end
		if (system.cpu.core.state == 8'd84 && (system.cpu.core.exc_fmt == 4'hA || system.cpu.core.exc_fmt == 4'hB)) begin
			tr_n = tr_n + 1;
			$display("[%0d.%03d ms] RTEBUS fmt=%h ssw=%04h fa=%08h sr=%04h pc=%08h rk=%0d",
				tr_clk / 32000, (tr_clk % 32000) / 32, system.cpu.core.exc_fmt, system.cpu.core.exc_ssw,
				system.cpu.core.exc_fa, system.cpu.core.exc_sr, system.cpu.core.exc_pc, system.cpu.core.exc_rk);
		end
	end
end

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

// the VME card's picture, as the scaler is told it: logged
wire        fb_en, fb_pal_wr;
wire  [4:0] fb_format;
wire [11:0] fb_width, fb_height;
wire [31:0] fb_base;
wire [13:0] fb_stride;
wire  [7:0] fb_pal_addr;
wire [23:0] fb_pal_rgb;
reg  [75:0] fb_was = 0;
always @(posedge clk) begin
	fb_was <= {fb_en, fb_format, fb_width, fb_height, fb_base, fb_stride};
	if (fb_was != {fb_en, fb_format, fb_width, fb_height, fb_base, fb_stride})
		$display("fb: en %0d format %02x %0dx%0d base %08x stride %0d", fb_en, fb_format, fb_width, fb_height, fb_base, fb_stride);
	if (fb_pal_wr) $display("fb: pal %02x = %06x", fb_pal_addr, fb_pal_rgb);
end

falcon_system #(.CLK_HZ(32000000)) system
(
	.clk(clk), .reset(reset), .cold_reset(cold_reset), .por(por),
	.ram_mb(ram_mb), .ram_tos(ram_tos), .monitor(monitor), .cpu_turbo(cpu_turbo), .tstat_en(tstat_en), .net_on(net_on), .vme_sel(vme_sel), .tt_mode(tt_mode), .ttram_sel(ttram_sel),
	.ld_wr(1'b0), .ld_addr(24'd0), .ld_data(8'd0), .ld_busy(),
	.ps2_key(ps2_key), .ps2_mouse(ps2_mouse), .joy0(joy0), .joy1(32'd0), .ana0(16'd0), .ana1(16'd0), .rtc(rtc),
	.nv_init(1'b0), .nv_addr(6'd0), .nv_din(8'd0), .nv_wr(1'b0),
	.nv_dout(), .nv_changed(), .nv_ready(),
	.img_mounted(img_mounted), .img_readonly(img_readonly), .img_size(img_size),
	.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack), .sd_blk_cnt(sd_blk_cnt),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
	.r(r), .g(g), .b(b), .hsync(hsync), .vsync(vsync), .hblank(hblank), .vblank(vblank), .ce_pix(ce_pix),
	.fb_en(fb_en), .fb_format(fb_format), .fb_width(fb_width), .fb_height(fb_height),
	.fb_base(fb_base), .fb_stride(fb_stride),
	.fb_pal_wr(fb_pal_wr), .fb_pal_addr(fb_pal_addr), .fb_pal_rgb(fb_pal_rgb),
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

endmodule
