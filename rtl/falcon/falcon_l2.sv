//============================================================================
//  Falcon: the CPU's line cache in block RAM (docs/CPU_TIMING.md, milestone 4)
//
//  Between falcon_cpubus and falcon_memarb's CPU port: the guest's RAM and
//  ROM live in the HPS DDR3, whose latency costs the Falcon-mode processor
//  real time (the bridge holds it, falcon_cpuclk's debt grows and is
//  forgiven at the cap).  This cache answers repeated reads from block RAM
//  within a couple of clocks and fills a 32-byte line with one 4-beat DDR3
//  burst.  It changes when answers arrive, never what the processor sees or
//  when in Falcon time (the bridge's holds absorb any latency), so it has no
//  counterpart in Hatari; tools/cputime and tb/bustime check that Falcon time
//  is the same with it.
//
//  Direct mapped, 2^IW lines of 32 bytes (IW = 11: 64 KB), the data as eight
//  byte lanes (CPU write hits update the bytes written), a tag RAM of
//  {valid, address[23:5+IW]}.  Reads that miss fill the whole line; the
//  word asked for is answered as soon as its beat arrives.  Writes go
//  through to the DDR3 (and update a line that holds them), never allocate.
//  Coherence: every write by another master (DMA, the loader) drops the line
//  at its index when its DDR3 command goes out (falcon_memarb owr_*), and a
//  fill that such a write touched is not kept; a sweep invalidates every
//  line after reset and whenever the cache is switched on.
//
//  en = 0 (the 32 MHz turbo setting) passes the CPU port straight through,
//  combinationally: the turbo machine behaves exactly as without the cache.
//  en takes effect only while nothing is in flight.
//============================================================================

module falcon_l2 #(parameter IW = 11)
(
	input             clk,
	input             reset,
	input             en,           // the Falcon mode (cache in use)

	// CPU side (falcon_cpubus): falcon_memarb's CPU port protocol
	input             c_req,
	input             c_we,
	input      [23:2] c_addr,
	input       [3:0] c_be,         // [3] = lowest address
	input      [31:0] c_wdata,
	output     [31:0] c_rdata,
	output     [63:0] c_rdata64,
	output            c_ack,

	// memory side (falcon_memarb's CPU port)
	output            m_req,
	output            m_we,
	output     [23:2] m_addr,
	output      [3:0] m_be,
	output     [31:0] m_wdata,
	output            m_burst,
	input      [31:0] m_rdata,
	input      [63:0] m_rdata64,
	input             m_beat,
	input             m_ack,

	// writes by other masters (falcon_memarb owr_*)
	input             owr_we,
	input      [23:3] owr_addr
);

localparam TW = 19 - IW;                       // tag bits: address[23:5+IW]
localparam NL = 1 << IW;

//----------------------------------------------------------------- storage
// tag RAM: {valid, tag}
reg [TW:0]    tags [0:NL-1];
reg [IW-1:0]  t_ra;
reg [TW:0]    t_q;
reg           t_we;
reg [IW-1:0]  t_wa;
reg [TW:0]    t_wd;
always @(posedge clk) begin
	if (t_we) tags[t_wa] <= t_wd;
	t_q <= tags[t_ra];
end

// data RAM: eight byte lanes of 2^(IW+2) bytes, lane 7 = the lowest address
reg [IW+1:0]  d_ra, d_wa;
reg  [7:0]    d_we;
reg [63:0]    d_wd;
wire [63:0]   d_q;
genvar gl;
generate for (gl = 0; gl < 8; gl = gl + 1) begin : g_lane
	reg [7:0] lane [0:(4 << IW) - 1];
	reg [7:0] q;
	always @(posedge clk) begin
		if (d_we[gl]) lane[d_wa] <= d_wd[8 * gl +: 8];
		q <= lane[d_ra];
	end
	assign d_q[8 * gl +: 8] = q;
end endgenerate

//----------------------------------------------------------------- control
localparam S_IDLE = 3'd0, S_LOOK = 3'd1, S_WRITE = 3'd2, S_FILL = 3'd3, S_GAP = 3'd4;
reg  [2:0]  st;
reg         on;                 // en, as taken while idle
reg         sweep;              // invalidating every line
reg [IW-1:0] sw_cnt;

reg         q_we;
reg [23:2]  q_addr;
reg  [3:0]  q_be;
reg [31:0]  q_wdata;
wire [IW-1:0] q_idx = q_addr[IW+4:5];
wire [TW-1:0] q_tag = q_addr[23:5+IW];
reg         poison;             // the line being filled was written by another master
reg         stale;              // the tag was read while its line was being dropped
reg  [1:0]  beat;
reg         answered;

reg         r_req, r_we, r_burst;
reg [23:2]  r_addr;
reg  [3:0]  r_be;
reg [31:0]  r_wdata;
reg         r_ack;
reg [63:0]  r_rdata64;

wire        inv = owr_we;       // another master's write: drop that line
wire [IW-1:0] inv_idx = owr_addr[IW+4:5];
wire        hit = t_q[TW] && (t_q[TW-1:0] == q_tag) && !sweep && !stale && !(inv && inv_idx == q_idx);

// the read addresses for the lookup, from the request being taken
always @* begin
	t_ra = c_addr[IW+4:5];
	d_ra = c_addr[IW+4:3];
end

// tag writes: the sweep, then another master's write, then a fill's start
// (the line is dropped before its data changes) and end
reg fill_start, fill_end;
always @* begin
	t_we = 1'b0; t_wa = q_idx; t_wd = {1'b0, q_tag};
	if (sweep)             begin t_we = 1'b1; t_wa = sw_cnt; end
	else if (inv)          begin t_we = 1'b1; t_wa = inv_idx; end
	else if (fill_start)   begin t_we = 1'b1; end
	else if (fill_end)     begin t_we = !poison; t_wd = {1'b1, q_tag}; end
end
// a fill starts when the lookup misses and the tag port is free
// (during the sweep the line is filled but not kept: poison)
always @* fill_start = (st == S_LOOK) && on && !q_we && !hit && !inv;
always @* fill_end   = (st == S_FILL) && m_ack;

// data writes: a fill's beats, or the bytes of a write that hits
always @* begin
	d_we = 8'd0; d_wa = {q_idx, beat}; d_wd = m_rdata64;
	if (st == S_FILL && m_beat) d_we = 8'hFF;
	else if (st == S_LOOK && on && q_we && hit) begin
		d_wa = q_addr[IW+4:3];
		d_wd = {2{q_wdata}};
		d_we = q_addr[2] ? {4'd0, q_be} : {q_be, 4'd0};
	end
end

always @(posedge clk) begin
	r_ack <= 1'b0;
	if (reset) begin
		st <= S_IDLE; on <= 1'b0; sweep <= 1'b1; sw_cnt <= {IW{1'b0}};
		r_req <= 1'b0; r_burst <= 1'b0;
	end else begin
		if (sweep) begin
			sw_cnt <= sw_cnt + 1'd1;
			if (sw_cnt == {IW{1'b1}}) sweep <= 1'b0;
		end
		if (st == S_FILL && inv && inv_idx == q_idx) poison <= 1'b1;
		case (st)
		S_IDLE:
			if (en != on && !c_req) begin
				on <= en;
				if (en) begin sweep <= 1'b1; sw_cnt <= {IW{1'b0}}; end
			end else if (on && c_req) begin
				q_we <= c_we; q_addr <= c_addr; q_be <= c_be; q_wdata <= c_wdata;
				stale <= sweep || (inv && inv_idx == c_addr[IW+4:5]);
				if (c_we) begin
					// write through at once
					r_req <= 1'b1; r_we <= 1'b1; r_burst <= 1'b0;
					r_addr <= c_addr; r_be <= c_be; r_wdata <= c_wdata;
				end
				st <= S_LOOK;
			end
		S_LOOK:
			if (q_we) st <= S_WRITE;                  // (a hit's bytes are written now)
			else if (hit) begin
				r_rdata64 <= d_q; r_ack <= 1'b1;
				st <= S_GAP;
			end else if (fill_start) begin
				stale <= 1'b0;
				r_req <= 1'b1; r_we <= 1'b0; r_burst <= 1'b1; r_addr <= q_addr;
				beat <= 2'd0; answered <= 1'b0; poison <= sweep;
				st <= S_FILL;
			end
		S_WRITE:
			if (m_ack) begin
				r_req <= 1'b0; r_ack <= 1'b1;
				st <= S_GAP;
			end
		S_FILL: begin
			if (m_beat) begin
				beat <= beat + 1'd1;
				if (!answered && beat == q_addr[4:3]) begin
					r_rdata64 <= m_rdata64; r_ack <= 1'b1; answered <= 1'b1;
				end
			end
			if (m_ack) begin
				r_req <= 1'b0;
				st <= S_GAP;
			end
		end
		default: st <= S_IDLE;                     // S_GAP: the CPU drops its request
		endcase
		if (sweep && st == S_FILL) poison <= 1'b1;
	end
end

//----------------------------------------------------------------- the ports
assign m_req     = on ? r_req   : c_req;
assign m_we      = on ? r_we    : c_we;
assign m_addr    = on ? r_addr  : c_addr;
assign m_be      = on ? r_be    : c_be;
assign m_wdata   = on ? r_wdata : c_wdata;
assign m_burst   = on ? r_burst : 1'b0;
assign c_ack     = on ? r_ack   : m_ack;
assign c_rdata64 = on ? r_rdata64 : m_rdata64;
assign c_rdata   = on ? (q_addr[2] ? r_rdata64[31:0] : r_rdata64[63:32]) : m_rdata;

endmodule
