//============================================================================
//  The core's DDR3 port (MiSTer DDRAM, Avalon-MM, 64 bits) shared by two
//  masters: m0, the ap060's Fast RAM (ap060_fastram, CT60 mode), first;
//  m1, falcon_memarb (everything else).  One command is outstanding at a
//  time.  A command is granted when it is presented with the port idle and
//  keeps its master while the port stalls it (waitrequest); a write burst
//  keeps it until its last beat is accepted, a read until its last data
//  beat has come back (readdatavalid goes to the owner only).  The master
//  not granted sees waitrequest.
//============================================================================

module falcon_ddr_arb
(
	input             clk,
	input             reset,

	input      [28:0] m0_address,
	input       [7:0] m0_burstcount,
	input             m0_read,
	input             m0_write,
	input      [63:0] m0_writedata,
	input       [7:0] m0_byteenable,
	output            m0_waitrequest,
	output     [63:0] m0_readdata,
	output            m0_readdatavalid,

	input      [28:0] m1_address,
	input       [7:0] m1_burstcount,
	input             m1_read,
	input             m1_write,
	input      [63:0] m1_writedata,
	input       [7:0] m1_byteenable,
	output            m1_waitrequest,
	output     [63:0] m1_readdata,
	output            m1_readdatavalid,

	output     [28:0] s_address,
	output      [7:0] s_burstcount,
	output            s_read,
	output            s_write,
	output     [63:0] s_writedata,
	output      [7:0] s_byteenable,
	input             s_waitrequest,
	input      [63:0] s_readdata,
	input             s_readdatavalid
);

reg        busy = 1'b0;      // a burst is in flight: owner holds the port
reg        owner = 1'b0;     // 0: m0, 1: m1
reg        rd_burst;         // the burst in flight is a read
reg  [7:0] left;             // beats still to come (read data, or write beats)
reg        held = 1'b0;      // a command was presented and stalled: its master keeps the port
reg        held_m;

wire m0_req = m0_read | m0_write;
wire m1_req = m1_read | m1_write;
// the master presenting to the port this clock
wire sel    = busy ? owner : held ? held_m : (m0_req ? 1'b0 : 1'b1);
wire s_cmd  = sel ? m1_req : m0_req;

// a read burst's data is not a new command: only its owner's write beats
// (and, when idle, a new command) go to the port
wire pass   = !busy || !rd_burst;

assign s_address    = sel ? m1_address    : m0_address;
assign s_burstcount = sel ? m1_burstcount : m0_burstcount;
assign s_read       = pass && (sel ? m1_read  : m0_read);
assign s_write      = pass && (sel ? m1_write : m0_write);
assign s_writedata  = sel ? m1_writedata  : m0_writedata;
assign s_byteenable = sel ? m1_byteenable : m0_byteenable;

assign m0_waitrequest   = !(pass && sel == 1'b0) || s_waitrequest;
assign m1_waitrequest   = !(pass && sel == 1'b1) || s_waitrequest;
assign m0_readdata      = s_readdata;
assign m1_readdata      = s_readdata;
assign m0_readdatavalid = s_readdatavalid && busy && rd_burst && owner == 1'b0;
assign m1_readdatavalid = s_readdatavalid && busy && rd_burst && owner == 1'b1;

always @(posedge clk) begin
	if (reset) begin
		busy <= 1'b0; held <= 1'b0;
	end
	else if (!busy) begin
		if (s_cmd && s_waitrequest) begin held <= 1'b1; held_m <= sel; end
		else if (s_cmd) begin
			// accepted: a read waits for its data; a write burst for its
			// other beats (a single beat is done now)
			held <= 1'b0;
			owner <= sel;
			if (s_read) begin
				busy <= 1'b1; rd_burst <= 1'b1; left <= s_burstcount;
			end else if (s_burstcount > 8'd1) begin
				busy <= 1'b1; rd_burst <= 1'b0; left <= s_burstcount - 8'd1;
			end
		end
	end
	else if (rd_burst) begin
		if (s_readdatavalid) begin
			left <= left - 8'd1;
			if (left == 8'd1) busy <= 1'b0;
		end
	end
	else if (s_write && !s_waitrequest) begin
		left <= left - 8'd1;
		if (left == 8'd1) busy <= 1'b0;
	end
end

endmodule
