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
	.clk(clk), .reset(reset), .cold_reset(cold_reset), .por(por),
	.ram_mb(ram_mb), .ram_tos(ram_tos), .monitor(monitor),
	.ld_wr(1'b0), .ld_addr(24'd0), .ld_data(8'd0), .ld_busy(),
	.ps2_key(ps2_key), .ps2_mouse(ps2_mouse), .joy0(joy0), .joy1(32'd0), .ana0(16'd0), .ana1(16'd0), .rtc(rtc),
	.nv_init(1'b0), .nv_addr(6'd0), .nv_din(8'd0), .nv_wr(1'b0),
	.nv_dout(), .nv_changed(), .nv_ready(),
	.img_mounted(img_mounted), .img_readonly(img_readonly), .img_size(img_size),
	.sd_lba(sd_lba), .sd_rd(sd_rd), .sd_wr(sd_wr), .sd_ack(sd_ack), .sd_blk_cnt(sd_blk_cnt),
	.sd_buff_addr(sd_buff_addr), .sd_buff_dout(sd_buff_dout), .sd_buff_din(sd_buff_din), .sd_buff_wr(sd_buff_wr),
	.r(r), .g(g), .b(b), .hsync(hsync), .vsync(vsync), .hblank(hblank), .vblank(vblank), .ce_pix(ce_pix),
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

endmodule
