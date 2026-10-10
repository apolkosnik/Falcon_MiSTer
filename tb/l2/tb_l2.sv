`timescale 1ns/1ps
//============================================================================
//  falcon_l2 (the CPU's line cache) with the real falcon_memarb on the DDR3
//  model: random CPU reads and writes over three times the cache's size,
//  another master's writes (DMA port d1, the loader) and video bursts at
//  random moments, the cache switched off (turbo: passed through) and on
//  again now and then.  Every CPU read must return what the memory held at
//  some clock between the request and the answer; every write must reach
//  the memory.  Exits nonzero on a mismatch.
//    +seed=N  +clocks=N  +hot=P (P% of the CPU's accesses, P/4% of the DMA
//    writes, in the first 4 KB)  (and the DDR3 model's +ddrlat= +ddrbusy= +ddrspike=)
//============================================================================
module tb_l2;
reg clk = 1'b0;
always #5 clk = ~clk;
reg reset = 1'b1;

integer seed = 1, clocks = 400000, hot = 0, dma = 40, toggle = 20000;
initial begin
	if ($value$plusargs("dma=%d", dma)) ;          // a DMA write every N clocks on average (0: none)
	if ($value$plusargs("toggle=%d", toggle)) ;    // the cache off/on every N clocks on average (0: never)
	if ($value$plusargs("seed=%d", seed)) ;
	if ($value$plusargs("clocks=%d", clocks)) ;
	if ($value$plusargs("hot=%d", hot)) ;      // percent of CPU accesses in the first 4 KB
end
function integer rnd(input integer n);
	rnd = $unsigned($random(seed)) % n;
endfunction

localparam [23:0] WIN  = 24'h010000;        // the window the masters use
localparam integer WLEN = 24'h030000;       // 192 KB: three times the cache

// ---------------------------------------------------------------- the DUT
reg         en = 1'b1;
reg         c_req = 1'b0, c_we = 1'b0;
reg  [23:2] c_addr = 22'd0;
reg   [3:0] c_be = 4'd0;
reg  [31:0] c_wdata = 32'd0;
wire [31:0] c_rdata;
wire [63:0] c_rdata64;
wire        c_ack;
wire        m_req, m_we, m_burst, m_beat, m_ack;
wire [23:2] m_addr;
wire  [3:0] m_be;
wire [31:0] m_wdata, m_rdata;
wire [63:0] m_rdata64;
wire        owr_we;
wire [23:3] owr_addr;

falcon_l2 l2
(
	.clk(clk), .reset(reset), .en(en),
	.c_req(c_req), .c_we(c_we), .c_addr(c_addr), .c_be(c_be), .c_wdata(c_wdata),
	.c_rdata(c_rdata), .c_rdata64(c_rdata64), .c_ack(c_ack),
	.m_req(m_req), .m_we(m_we), .m_addr(m_addr), .m_be(m_be), .m_wdata(m_wdata),
	.m_burst(m_burst), .m_rdata(m_rdata), .m_rdata64(m_rdata64), .m_beat(m_beat), .m_ack(m_ack),
	.owr_we(owr_we), .owr_addr(owr_addr)
);

reg         vid_req = 1'b0;
reg  [23:3] vid_addr = 21'd0;
wire        vid_ack;
reg         ld_wr = 1'b0;
reg  [23:0] ld_addr = 24'd0;
reg   [7:0] ld_data = 8'd0;
wire        ld_busy;
reg         d1_req = 1'b0, d1_we = 1'b0;
reg  [23:1] d1_addr = 23'd0;
reg   [1:0] d1_be = 2'd0;
reg  [15:0] d1_wdata = 16'd0;
wire        d1_ack;

wire        DDRAM_BUSY, DDRAM_DOUT_READY, DDRAM_RD, DDRAM_WE;
wire  [7:0] DDRAM_BURSTCNT, DDRAM_BE;
wire [28:0] DDRAM_ADDR;
wire [63:0] DDRAM_DOUT, DDRAM_DIN;

falcon_memarb memarb
(
	.clk(clk), .reset(reset), .ram_mb(4'd4),
	.vid_req(vid_req), .vid_addr(vid_addr), .vid_ack(vid_ack), .vid_data(), .vid_valid(),
	.ld_wr(ld_wr), .ld_addr(ld_addr), .ld_data(ld_data), .ld_busy(ld_busy),
	.d0_req(1'b0), .d0_we(1'b0), .d0_addr(23'd0), .d0_be(2'b00), .d0_wdata(16'd0), .d0_rdata(), .d0_ack(),
	.d1_req(d1_req), .d1_we(d1_we), .d1_addr(d1_addr), .d1_be(d1_be), .d1_wdata(d1_wdata), .d1_rdata(), .d1_ack(d1_ack),
	.d2_req(1'b0), .d2_we(1'b0), .d2_addr(23'd0), .d2_be(2'b00), .d2_wdata(16'd0), .d2_rdata(), .d2_ack(),
	.d3_req(1'b0), .d3_we(1'b0), .d3_addr(23'd0), .d3_be(2'b00), .d3_wdata(16'd0), .d3_rdata(), .d3_ack(),
	.cpu_req(m_req), .cpu_we(m_we), .cpu_addr(m_addr), .cpu_be(m_be),
	.cpu_wdata(m_wdata), .cpu_rdata(m_rdata), .cpu_rdata64(m_rdata64), .cpu_ack(m_ack),
	.cpu_burst(m_burst), .cpu_beat(m_beat), .owr_we(owr_we), .owr_addr(owr_addr),
	.snoop_we(), .snoop_addr(),
	.DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE)
);

ddr3_model ddr
(
	.clk(clk), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE)
);

// the memory's 64-bit word at a guest address, in guest (big-endian) order
function [63:0] memw(input [23:3] a);
	reg [63:0] le;
	integer i;
	begin
		le = ddr.mem[a];
		for (i = 0; i < 8; i = i + 1) memw[8 * (7 - i) +: 8] = le[8 * i +: 8];
	end
endfunction

// ---------------------------------------------------------------- checking
integer checks = 0, errors = 0, n_rd = 0, n_wr = 0, n_dma = 0, n_ld = 0, n_hit = 0, n_fill = 0, n_toggle = 0;
// the values the read's word held while the read was out (at most 8)
reg  [63:0] seen [0:7];
integer     n_seen = 0;
reg         rd_out = 1'b0;
always @(negedge clk) if (rd_out) begin : sample
	integer k;
	reg     have;
	have = 1'b0;
	for (k = 0; k < n_seen; k = k + 1) if (seen[k] == memw(c_addr[23:3])) have = 1'b1;
	if (!have && n_seen < 8) begin seen[n_seen] = memw(c_addr[23:3]); n_seen = n_seen + 1; end
end

// ---------------------------------------------------------------- the CPU
integer gap = 0;
always @(posedge clk) if (!reset) begin
	if (c_req && c_ack) begin : done
		integer k;
		reg     ok;
		c_req <= 1'b0;
		gap = 1 + rnd(3);
		if (!c_we) begin
			ok = 1'b0;
			for (k = 0; k < n_seen; k = k + 1)
				if (c_rdata64 == seen[k] && c_rdata == (c_addr[2] ? seen[k][31:0] : seen[k][63:32])) ok = 1'b1;
			checks = checks + 1;
			if (!ok) begin
				errors = errors + 1;
				if (errors <= 10) $display("FAIL: read $%06x at %0t: %016x (%08x), memory held %016x%s", {c_addr, 2'b00}, $time,
				                           c_rdata64, c_rdata, seen[0], (n_seen > 1) ? " (and more)" : "");
			end
			rd_out <= 1'b0;
			n_rd = n_rd + 1;
		end else n_wr = n_wr + 1;
	end else if (!c_req) begin
		if (gap > 0) gap = gap - 1;
		else if (rnd(2) == 0) begin : issue
			reg w;
			w = (rnd(10) < 3);
			c_req   <= 1'b1;
			c_we    <= w;
			c_addr  <= (WIN + ((rnd(100) < hot) ? rnd(4096) : rnd(WLEN))) >> 2;
			c_be    <= 4'd1 + rnd(15);
			c_wdata <= $random(seed);
			// a read: collect what its word holds from its first clock on
			if (!w) begin rd_out <= 1'b1; n_seen = 0; end
		end
	end
end

// ---------------------------------------------------------------- the others
always @(posedge clk) if (!reset) begin
	// DMA writes (another master): every 40 clocks on average
	if (d1_req && d1_ack) d1_req <= 1'b0;
	else if (!d1_req && dma != 0 && rnd(dma) == 0) begin
		d1_req <= 1'b1; d1_we <= 1'b1;
		d1_addr <= (WIN + ((rnd(100) < hot / 4) ? rnd(4096) : rnd(WLEN))) >> 1;
		d1_be <= 2'd1 + rnd(3);
		d1_wdata <= $random(seed);
		n_dma = n_dma + 1;
	end
	// loader bytes: rarely
	ld_wr <= 1'b0;
	if (!ld_busy && !ld_wr && rnd(500) == 0) begin
		ld_wr <= 1'b1; ld_addr <= WIN + rnd(WLEN); ld_data <= $random(seed);
		n_ld = n_ld + 1;
	end
	// video bursts
	if (vid_req && vid_ack) vid_req <= 1'b0;
	else if (!vid_req && rnd(60) == 0) begin vid_req <= 1'b1; vid_addr <= rnd(1 << 20); end
	// the cache off (turbo) and on again
	if (toggle != 0 && rnd(toggle) == 0) begin en <= !en; n_toggle = n_toggle + 1; end
	// statistics
	if (l2.st == 3'd1 && l2.on && !l2.q_we) begin
		if (l2.hit) n_hit = n_hit + 1;
		else if (l2.fill_start) n_fill = n_fill + 1;
	end
end

integer t = 0;
initial begin
	#1;
	begin : fill
		integer i;
		for (i = WIN >> 3; i < (WIN + WLEN) >> 3; i = i + 1) ddr.mem[i] = {$random(seed), $random(seed)};
	end
	repeat (20) @(posedge clk);
	reset = 1'b0;
	while (t < clocks) begin @(posedge clk); t = t + 1; end
	repeat (2000) @(posedge clk);        // (the last access finishes)
	$display("%0d reads (%0d hits, %0d fills), %0d writes, %0d DMA writes, %0d loader bytes, %0d switches",
	         n_rd, n_hit, n_fill, n_wr, n_dma, n_ld, n_toggle);
	$display("SUMMARY: %0d checks, %0d errors", checks, errors);
	if (errors != 0 || checks < 1000) begin $display("RESULT: FAIL"); $fatal(1, "falcon_l2"); end
	$display("RESULT: PASS");
	$finish;
end

endmodule
