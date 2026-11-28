// falcon_lpt.sv - the Centronics port's output side, handed to the HPS.
//
// The PSG drives the printer port: port B is the eight data lines, port A
// bit 5 is /STROBE, and the printer's BUSY comes back on MFP GPIP0.  Each
// byte the machine strobes out is put in a ring in DDR3 (port m_*, the
// arbiter's d3 port, shared with the FPU mailbox and the DaynaPORT rings),
// from where tools/falcon_lpt on the HPS sends it to a printer or a file:
//
//   guest $EB8000 MAGIC $4C50 "LP" (HPS)   $EB8002 HEARTBEAT (HPS, ten a second)
//         $EB8004 HEAD (here): bytes put in the ring, a free-running count
//         $EB8006 TAIL (HPS): bytes taken out
//         $EB8008 STATUS (HPS): bit 0 = the printer cannot take data now
//         $EB8800 + 2*n ring entry n (0..2047), the byte in the low half
//
// 16-bit words in guest byte order.  BUSY is high while a byte waits to be
// stored, while the ring is full, while STATUS bit 0 is set, and whenever
// the HPS program is not there (no MAGIC, or a HEARTBEAT that stands still
// for a second): without it the machine sees a printer that is switched
// off, as it did before this module existed.  A byte strobed while BUSY is
// high for that last reason is dropped.
//
// A strobe is /STROBE going low after the machine has driven it low and
// high again since its reset.  That is Hatari's rule (psg.c: LastStrobe
// starts at 0), and it keeps TOS's own start out of the print: TOS makes
// port A an output and writes $07 to it, bit 5 low, which is not a byte.
//
// HEAD lives through a machine reset.  When the HPS program appears, HEAD is
// read back from memory, where the program has made TAIL equal to it.
module falcon_lpt #(
	parameter CLK_HZ = 32000000
) (
	input             clk,
	input             por,        // FPGA configuration only
	input             reset,      // the machine's reset
	input             strobe_n,   // PSG port A bit 5
	input             strobe_drv, // port A is an output
	input       [7:0] data,       // PSG port B
	output reg        busy,       // to MFP GPIP0

	output reg        m_req,
	output reg        m_we,
	output reg [23:1] m_addr,
	output reg [15:0] m_wdata,
	input      [15:0] m_rdata,
	input             m_ack
);

localparam [23:0] L_MB   = 24'hEB8000;
localparam [15:0] MAGIC  = 16'h4C50;
localparam [15:0] FULL   = 16'd2040;
localparam integer POLL  = CLK_HZ / 250;       // 4 ms
localparam [7:0]  STALE  = 8'd250;             // polls without a new HEARTBEAT: 1 s

localparam [3:0] S_IDLE = 4'd0, S_WAIT = 4'd1, S_WR2 = 4'd2, S_WR3 = 4'd3,
                 S_P1 = 4'd4, S_P2 = 4'd5, S_P3 = 4'd6, S_P4 = 4'd7, S_P5 = 4'd8;
reg  [3:0] st = S_IDLE, ret;
reg [15:0] q;
reg [15:0] head = 16'd0, tail = 16'd0, hb = 16'd0;
reg  [7:0] age = STALE;
reg        hb_seen = 1'b0;          // a first HEARTBEAT is only a reference
reg        present = 1'b0, magic_ok = 1'b0, st_busy = 1'b1;
reg        pend = 1'b0;
reg  [7:0] pend_d;
reg        stb_q = 1'b0;            // /STROBE is driven high, after a low
reg        stb_low = 1'b0;          // /STROBE has been driven low since the reset
reg [$clog2(POLL)-1:0] pcnt = '0;
reg        poll = 1'b0;

wire full = (head - tail) >= FULL;

task automatic acc(input we_, input [23:0] a, input [15:0] d, input [3:0] next);
	begin
		m_req <= 1'b1; m_we <= we_; m_addr <= a[23:1]; m_wdata <= d;
		ret <= next; st <= S_WAIT;
	end
endtask

always @(posedge clk) begin
	if (reset) begin stb_q <= 1'b0; stb_low <= 1'b0; end
	else if (strobe_drv) begin
		if (!strobe_n) stb_low <= 1'b1;
		stb_q <= strobe_n & stb_low;
	end
	if (stb_q & ~strobe_n & strobe_drv & present & ~pend) begin
		pend <= 1'b1;
		pend_d <= data;
	end

	if (pcnt == POLL - 1) begin pcnt <= '0; poll <= 1'b1; end
	else pcnt <= pcnt + 1'd1;

	busy <= ~present | pend | full | st_busy;

	if (por) begin
		m_req <= 1'b0; m_we <= 1'b0; m_addr <= 23'd0; m_wdata <= 16'd0;
		st <= S_IDLE; present <= 1'b0; pend <= 1'b0; age <= STALE; st_busy <= 1'b1;
		hb_seen <= 1'b0;
	end else case (st)
		S_IDLE:
			if (pend & present) acc(1'b1, L_MB + 24'h800 + {12'd0, head[10:0], 1'b0}, {8'd0, pend_d}, S_WR2);
			else if (poll) begin
				poll <= 1'b0;
				acc(1'b0, L_MB, 16'd0, S_P1);                       // MAGIC
			end
		S_WAIT: if (m_ack) begin
			m_req <= 1'b0;
			q <= m_rdata;
			st <= ret;
		end
		// a byte: the ring entry is written, then HEAD moves
		S_WR2: begin
			head <= head + 1'd1;
			acc(1'b1, L_MB + 24'h4, head + 1'd1, S_WR3);
		end
		S_WR3: begin pend <= 1'b0; st <= S_IDLE; end
		// the poll: MAGIC, HEARTBEAT, TAIL, STATUS, and HEAD when the HPS side appears
		S_P1: begin
			magic_ok <= (q == MAGIC);
			acc(1'b0, L_MB + 24'h2, 16'd0, S_P2);
		end
		S_P2: begin
			hb <= q;
			hb_seen <= 1'b1;
			if (hb_seen && q != hb) age <= 8'd0;
			else if (age != STALE) age <= age + 1'd1;
			acc(1'b0, L_MB + 24'h6, 16'd0, S_P3);
		end
		S_P3: begin
			tail <= q;
			acc(1'b0, L_MB + 24'h8, 16'd0, S_P4);
		end
		S_P4: begin
			st_busy <= q[0];
			if (!(magic_ok && age != STALE)) begin
				present <= 1'b0;
				pend <= 1'b0;
				st <= S_IDLE;
			end else if (!present) acc(1'b0, L_MB + 24'h4, 16'd0, S_P5);
			else st <= S_IDLE;
		end
		S_P5: begin
			head <= q;
			present <= 1'b1;
			st <= S_IDLE;
		end
		default: st <= S_IDLE;
	endcase
end

endmodule
