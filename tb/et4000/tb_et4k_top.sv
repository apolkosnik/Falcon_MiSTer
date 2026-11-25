// The Nova ET4000 card (rtl/vme/vme_et4000.sv) with the memory arbiter and the DDR3 model:
// tb_et4k.cpp drives its bus and checks it against a model of the ET4000 in C.
module tb_et4k_top (
	input             clk,
	input             reset,
	input             sys_reset,
	input             bus_cs,
	input             bus_stb,
	input             bus_we,
	input             bus_mem,
	input      [19:1] bus_addr,
	input             bus_uds,
	input             bus_lds,
	input      [15:0] bus_din,
	output     [15:0] bus_dout,
	output            bus_ack,
	output            fb_en,
	output      [4:0] fb_format,
	output     [11:0] fb_width,
	output     [11:0] fb_height,
	output     [23:0] fb_addr,
	output     [13:0] fb_stride,
	output            pal_wr,
	output      [7:0] pal_addr,
	output     [23:0] pal_rgb,
	input             vid_req,       // other traffic on the DDR3: a video burst
	output            vid_ack,
	output            ddr_we_wait    // a write the DDR3 has not taken yet
);

wire        m_req, m_we, m_ack;
wire [23:3] m_addr;
wire  [7:0] m_be;
wire [63:0] m_wdata, m_rdata64;

vme_et4000 #(.CLK_HZ(32000000)) card (
	.clk(clk), .reset(reset), .sys_reset(sys_reset),
	.bus_cs(bus_cs), .bus_stb(bus_stb), .bus_we(bus_we), .bus_mem(bus_mem), .bus_addr(bus_addr),
	.bus_uds(bus_uds), .bus_lds(bus_lds), .bus_din(bus_din), .bus_dout(bus_dout), .bus_ack(bus_ack),
	.fb_en(fb_en), .fb_format(fb_format), .fb_width(fb_width), .fb_height(fb_height),
	.fb_addr(fb_addr), .fb_stride(fb_stride),
	.pal_wr(pal_wr), .pal_addr(pal_addr), .pal_rgb(pal_rgb),
	.mem_req(m_req), .mem_we(m_we), .mem_addr(m_addr), .mem_be(m_be), .mem_wdata(m_wdata),
	.mem_rdata(m_rdata64), .mem_ack(m_ack)
);

wire        d_busy, d_ready, d_rd, d_we;
wire  [7:0] d_cnt, d_be;
wire [28:0] d_addr;
wire [63:0] d_din, d_dout;
assign ddr_we_wait = d_we;

falcon_memarb arb (
	.clk(clk), .reset(reset), .ram_mb(4'd10), .tt_top(29'd0),
	.vid_req(vid_req), .vid_addr(21'h001000), .vid_ack(vid_ack), .vid_data(), .vid_valid(),
	.ld_wr(1'b0), .ld_addr(24'd0), .ld_data(8'd0), .ld_busy(),
	.d0_req(1'b0), .d0_we(1'b0), .d0_addr(23'd0), .d0_be(2'b00), .d0_wdata(16'd0), .d0_rdata(), .d0_ack(),
	.d1_req(1'b0), .d1_we(1'b0), .d1_addr(28'd0), .d1_be(2'b00), .d1_wdata(16'd0), .d1_rdata(), .d1_ack(), .d1_bad(),
	.d2_req(m_req), .d2_we(m_we), .d2_addr({m_addr, 2'b00}), .d2_rdata(), .d2_ack(m_ack),
	.d2_hold(1'b1), .d2_vme(1'b1), .d2_rdata64(m_rdata64),
	.d2_be(2'b00), .d2_wdata(16'd0), .d2_wide(1'b1), .d2_wdata64(m_wdata), .d2_be8(m_be),
	.d3_req(1'b0), .d3_we(1'b0), .d3_addr(23'd0), .d3_be(2'b00), .d3_wdata(16'd0), .d3_rdata(), .d3_ack(),
	.cpu_req(1'b0), .cpu_we(1'b0), .cpu_addr(27'd0), .cpu_be(4'd0), .cpu_wdata(32'd0),
	.cpu_rdata(), .cpu_rdata64(), .cpu_ack(), .cpu_burst(1'b0), .cpu_beat(),
	.owr_we(), .owr_addr(), .snoop_we(), .snoop_addr(), .snoop_hold(1'b0),
	.DDRAM_BUSY(d_busy), .DDRAM_BURSTCNT(d_cnt), .DDRAM_ADDR(d_addr), .DDRAM_DOUT(d_dout),
	.DDRAM_DOUT_READY(d_ready), .DDRAM_RD(d_rd), .DDRAM_DIN(d_din), .DDRAM_BE(d_be), .DDRAM_WE(d_we)
);

ddr3_model ddr (
	.clk(clk), .DDRAM_BUSY(d_busy), .DDRAM_BURSTCNT(d_cnt), .DDRAM_ADDR(d_addr), .DDRAM_DOUT(d_dout),
	.DDRAM_DOUT_READY(d_ready), .DDRAM_RD(d_rd), .DDRAM_DIN(d_din), .DDRAM_BE(d_be), .DDRAM_WE(d_we)
);

endmodule
