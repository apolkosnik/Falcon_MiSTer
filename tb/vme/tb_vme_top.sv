// The ATW800/2's 2D engine with the memory arbiter and the DDR3 model: tb_vme.cpp gives it
// commands and compares the video memory with the same done byte by byte.
// OLD16: an engine with the 16-bit port (the arbiter's narrow path), for the comparison.
module tb_vme_top (
	input             clk,
	input             reset,
	input             mem4,
	input             bus_stb,
	input             bus_we,
	input      [11:1] bus_addr,
	input             bus_uds,
	input             bus_lds,
	input      [15:0] bus_din,
	output     [15:0] bus_dout,
	output            bus_ack,
	output            busy,
	output            lay4,
	input             vid_req,       // other traffic on the DDR3: a video burst
	output            vid_ack,
	output            ddr_rd_cmd,
	output            ddr_we_cmd,
	output            ddr_we_wait    // a write the DDR3 has not taken yet
);

wire        m_req, m_we, m_ack;
wire [15:0] m_rdata16;
wire [63:0] m_rdata64;
`ifdef OLD16
wire [21:1] m_off;
wire  [1:0] m_be;
wire [15:0] m_wdata;
wire [23:1] m_addr = {3'b101 + {2'b00, m_off[21]}, m_off[20:1]};
`else
wire [21:3] m_off;
wire  [7:0] m_be;
wire [63:0] m_wdata;
wire [23:1] m_addr = {3'b101 + {2'b00, m_off[21]}, m_off[20:3], 2'b00};
`endif

vme_atw800 card (
	.clk(clk), .reset(reset), .sys_reset(reset), .mem4(mem4), .lay4(lay4),
	.bus_cs(1'b1), .bus_stb(bus_stb), .bus_we(bus_we), .bus_aux(1'b0), .bus_addr(bus_addr),
	.bus_uds(bus_uds), .bus_lds(bus_lds), .bus_din(bus_din), .bus_dout(bus_dout), .bus_ack(bus_ack),
	.vtg(), .fb_en(), .fb_format(), .fb_width(), .fb_height(), .fb_start(), .fb_stride(),
	.pal_wr(), .pal_addr(), .pal_rgb(),
	.mem_req(m_req), .mem_we(m_we), .mem_addr(m_off), .mem_be(m_be), .mem_wdata(m_wdata),
`ifdef OLD16
	.mem_rdata(m_rdata16),
`else
	.mem_rdata(m_rdata64),
`endif
	.mem_ack(m_ack), .blit_busy(busy)
);

wire        d_busy, d_ready, d_rd, d_we;
wire  [7:0] d_cnt, d_be;
wire [28:0] d_addr;
wire [63:0] d_din, d_dout;
assign ddr_rd_cmd = d_rd && !d_busy;
assign ddr_we_cmd = d_we && !d_busy;
assign ddr_we_wait = d_we;

falcon_memarb arb (
	.clk(clk), .reset(reset), .ram_mb(4'd10), .tt_top(29'd0),
	.vid_req(vid_req), .vid_addr(21'h001000), .vid_ack(vid_ack), .vid_data(), .vid_valid(),
	.ld_wr(1'b0), .ld_addr(24'd0), .ld_data(8'd0), .ld_busy(),
	.d0_req(1'b0), .d0_we(1'b0), .d0_addr(23'd0), .d0_be(2'b00), .d0_wdata(16'd0), .d0_rdata(), .d0_ack(),
	.d1_req(1'b0), .d1_we(1'b0), .d1_addr(28'd0), .d1_be(2'b00), .d1_wdata(16'd0), .d1_rdata(), .d1_ack(), .d1_bad(),
	.d2_req(m_req), .d2_we(m_we), .d2_addr(m_addr), .d2_rdata(m_rdata16), .d2_ack(m_ack),
	.d2_hold(busy), .d2_vme(1'b1), .d2_rdata64(m_rdata64),
`ifdef OLD16
	.d2_be(m_be), .d2_wdata(m_wdata), .d2_wide(1'b0), .d2_wdata64(64'd0), .d2_be8(8'd0),
`else
	.d2_be(2'b00), .d2_wdata(16'd0), .d2_wide(1'b1), .d2_wdata64(m_wdata), .d2_be8(m_be),
`endif
	.d3_req(1'b0), .d3_we(1'b0), .d3_addr(23'd0), .d3_be(2'b00), .d3_wdata(16'd0), .d3_rdata(), .d3_ack(),
	.cpu_req(1'b0), .cpu_we(1'b0), .cpu_addr(27'd0), .cpu_be(4'd0), .cpu_wdata(32'd0),
	.cpu_rdata(), .cpu_rdata64(), .cpu_ack(), .cpu_burst(1'b0), .cpu_beat(),
	.owr_we(), .owr_addr(), .snoop_we(), .snoop_addr(),
	.DDRAM_BUSY(d_busy), .DDRAM_BURSTCNT(d_cnt), .DDRAM_ADDR(d_addr), .DDRAM_DOUT(d_dout),
	.DDRAM_DOUT_READY(d_ready), .DDRAM_RD(d_rd), .DDRAM_DIN(d_din), .DDRAM_BE(d_be), .DDRAM_WE(d_we)
);

ddr3_model ddr (
	.clk(clk), .DDRAM_BUSY(d_busy), .DDRAM_BURSTCNT(d_cnt), .DDRAM_ADDR(d_addr), .DDRAM_DOUT(d_dout),
	.DDRAM_DOUT_READY(d_ready), .DDRAM_RD(d_rd), .DDRAM_DIN(d_din), .DDRAM_BE(d_be), .DDRAM_WE(d_we)
);

endmodule
