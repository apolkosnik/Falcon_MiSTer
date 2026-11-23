//============================================================================
//  Behavioural model of the MiSTer DDRAM port (Avalon-MM burst slave)
//
//  The core's window at byte 0x30000000, 64-bit words, little endian like
//  the real HPS DDR3: the guest's 24-bit space (0x30000000-0x30FFFFFF) in an
//  array, the ap060's Fast RAM (CT60 mode, 0x31000000-0x3FFFFFFF) sparse.  Read latency and BUSY stalls are
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
longint unsigned fmem [int unsigned];   // Fast RAM, by word offset in the window (sparse)

// a 64-bit word of the window (word offset from 0x30000000)
function automatic logic [63:0] wrd(input logic [24:0] w);
	if (w < 25'h200000) wrd = mem[w[20:0]];
	else wrd = fmem.exists(int'(w)) ? fmem[int'(w)] : 64'd0;
endfunction
function automatic void wwr(input logic [24:0] w, input logic [63:0] d, input logic [7:0] be);
	logic [63:0] q;
	integer i;
	q = wrd(w);
	for (i = 0; i < 8; i = i + 1) if (be[i]) q[8*i +: 8] = d[8*i +: 8];
	if (w < 25'h200000) mem[w[20:0]] = q;
	else fmem[int'(w)] = q;
endfunction

// the C++ driver reads and writes guest memory directly (e.g. to play the
// HPS side of a mailbox); qaddr is the 64-bit word index from 0x30000000
// (guest address / 8, Fast RAM included)
export "DPI-C" function ddr_peek;
export "DPI-C" function ddr_poke;
function longint unsigned ddr_peek(input int unsigned qaddr);
	ddr_peek = wrd(qaddr[24:0]);
endfunction
function void ddr_poke(input int unsigned qaddr, input longint unsigned data, input byte unsigned be);
	wwr(qaddr[24:0], data, be);
endfunction

initial begin
	string f;
	integer i;
	for (i = 0; i < (1<<21); i = i + 1) mem[i] = 64'd0;
	if ($value$plusargs("rom=%s", f)) $readmemh(f, mem, 'hE00000 >> 3);
	if ($value$plusargs("ram=%s", f)) $readmemh(f, mem, 0);    // RAM image from 0 (rom2hex64.py format)
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
reg [24:0] rd_addr;
reg  [7:0] rd_left;
reg  [9:0] rd_wait;
reg  [7:0] busy_left = 8'd0;

wire hit = (DDRAM_ADDR[28:25] == 4'h3);    // 0x30000000-0x3FFFFFFF: word 0x6000000-0x7FFFFFF
wire [24:0] woff = DDRAM_ADDR[24:0];          // word offset from 0x30000000

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
			if (!hit) $display("DDR3 model: write outside the guest window %07x", DDRAM_ADDR);
			wwr(woff, DDRAM_DIN, DDRAM_BE);
		end
		if (DDRAM_RD) begin
			if (rd_pend) $display("DDR3 model: read issued while a read is outstanding");
			if (!hit) $display("DDR3 model: read outside the guest window %07x", DDRAM_ADDR);
			rd_pend <= 1;
			rd_addr <= woff;
			rd_left <= DDRAM_BURSTCNT;
			rd_wait <= 10'd5 + {7'd0, lfsr[9:7]} + {4'd0, xlat} + ((lfsr[27:24] == 4'd0) ? {1'b0, xspike} : 10'd0);
		end
	end

	if (rd_pend) begin
		if (rd_wait != 0) rd_wait <= rd_wait - 1'd1;
		else if (lfsr[12:11] != 2'd0) begin     // gaps between beats
			DDRAM_DOUT       <= wrd(rd_addr);
			DDRAM_DOUT_READY <= 1;
			rd_addr          <= rd_addr + 1'd1;
			rd_left          <= rd_left - 1'd1;
			if (rd_left == 8'd1) rd_pend <= 0;
		end
	end
end

endmodule
