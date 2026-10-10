//============================================================================
//  Behavioural model of the MiSTer DDRAM port (Avalon-MM burst slave)
//
//  16 MB window at byte 0x30000000 (the core's guest space), 64-bit words,
//  little endian like the real HPS DDR3.  Read latency and BUSY stalls are
//  pseudo-random within the ranges seen on hardware so the arbiter is
//  exercised with realistic, varying timing.  +ddrlat=N adds N clocks to
//  every read's latency (N < 52) and +ddrseed=S (hex) seeds the pseudo-random timing;
//  +ddrbusy=N holds BUSY for stretches of up to N clocks (about one clock in
//  eight), as the HPS port does under load, and +ddrspike=N adds N clocks to
//  one read in sixteen (N < 450).  The Falcon timing must not depend on any
//  of them (tools/cputime/sim.sh).
//============================================================================

module ddr3_model
(
	input             clk,
	output reg        DDRAM_BUSY,
	input       [7:0] DDRAM_BURSTCNT,
	input      [28:0] DDRAM_ADDR,
	output reg [63:0] DDRAM_DOUT,
	output reg        DDRAM_DOUT_READY,
	input             DDRAM_RD,
	input      [63:0] DDRAM_DIN,
	input       [7:0] DDRAM_BE,
	input             DDRAM_WE
);

reg [63:0] mem [0:(1<<21)-1];

// the C++ driver reads and writes guest memory directly (e.g. to play the
// HPS side of a mailbox); qaddr is the 64-bit word index in the 16 MB window
export "DPI-C" function ddr_peek;
export "DPI-C" function ddr_poke;
function longint unsigned ddr_peek(input int unsigned qaddr);
	ddr_peek = mem[qaddr[20:0]];
endfunction
function void ddr_poke(input int unsigned qaddr, input longint unsigned data, input byte unsigned be);
	integer i;
	for (i = 0; i < 8; i = i + 1)
		if (be[i]) mem[qaddr[20:0]][8*i +: 8] = data[8*i +: 8];
endfunction

initial begin
	string f;
	integer i;
	for (i = 0; i < (1<<21); i = i + 1) mem[i] = 64'd0;
	if ($value$plusargs("rom=%s", f)) $readmemh(f, mem, 'hE00000 >> 3);
	if ($value$plusargs("cart=%s", f)) $readmemh(f, mem, 'hFA0000 >> 3);
	DDRAM_BUSY = 0;
	DDRAM_DOUT_READY = 0;
end

reg [31:0] lfsr = 32'h1234_5678;
reg  [5:0] xlat = 6'd0;
reg  [7:0] xbusy = 8'd0;       // longest BUSY stretch (0: single clocks only)
reg  [8:0] xspike = 9'd0;
initial begin
	if ($value$plusargs("ddrlat=%d", xlat)) ;
	if ($value$plusargs("ddrbusy=%d", xbusy)) ;
	if ($value$plusargs("ddrspike=%d", xspike)) ;
	if ($value$plusargs("ddrseed=%h", lfsr)) if (lfsr == 32'd0) lfsr = 32'h1234_5678;
end
always @(posedge clk) lfsr <= {lfsr[30:0], lfsr[31] ^ lfsr[21] ^ lfsr[1] ^ lfsr[0]};

reg        rd_pend = 0;
reg [20:0] rd_addr;
reg  [7:0] rd_left;
reg  [9:0] rd_wait;
reg  [7:0] busy_left = 8'd0;

wire hit = (DDRAM_ADDR[28:21] == 8'h30);   // 0x30000000 >> 3 = 0x6000000, bits 28:21 = 0x30

always @(posedge clk) begin
	DDRAM_DOUT_READY <= 0;
	// occasional one-clock BUSY when idle, as the controller shows; with
	// +ddrbusy longer stretches
	if (busy_left != 8'd0) begin
		DDRAM_BUSY <= 1'b1;
		busy_left  <= busy_left - 8'd1;
	end else if (xbusy != 8'd0 && lfsr[2:0] == 3'd0) begin
		DDRAM_BUSY <= 1'b1;
		busy_left  <= {1'b0, lfsr[20:14]} % xbusy;
	end else
		DDRAM_BUSY <= (lfsr[4:0] == 5'd0);

	if (!DDRAM_BUSY) begin
		if (DDRAM_WE) begin
			integer i;
			if (!hit) $display("DDR3 model: write outside the guest window %07x", DDRAM_ADDR);
			for (i = 0; i < 8; i = i + 1)
				if (DDRAM_BE[i]) mem[DDRAM_ADDR[20:0]][8*i +: 8] <= DDRAM_DIN[8*i +: 8];
		end
		if (DDRAM_RD) begin
			if (rd_pend) $display("DDR3 model: read issued while a read is outstanding");
			if (!hit) $display("DDR3 model: read outside the guest window %07x", DDRAM_ADDR);
			rd_pend <= 1;
			rd_addr <= DDRAM_ADDR[20:0];
			rd_left <= DDRAM_BURSTCNT;
			rd_wait <= 10'd5 + {7'd0, lfsr[9:7]} + {4'd0, xlat} + ((lfsr[27:24] == 4'd0) ? {1'b0, xspike} : 10'd0);
		end
	end

	if (rd_pend) begin
		if (rd_wait != 0) rd_wait <= rd_wait - 1'd1;
		else if (lfsr[12:11] != 2'd0) begin     // gaps between beats
			DDRAM_DOUT       <= mem[rd_addr];
			DDRAM_DOUT_READY <= 1;
			rd_addr          <= rd_addr + 1'd1;
			rd_left          <= rd_left - 1'd1;
			if (rd_left == 8'd1) rd_pend <= 0;
		end
	end
end

endmodule
