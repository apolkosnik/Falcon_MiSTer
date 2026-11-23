//--------------------------------------------------------------------------//
// AP68030 - MC68030 compatible CPU                                         //
//                                                                          //
// ap030_regfile.v - D0-D7, A0-A6 and the three stack pointers              //
//                                                                          //
// Index 0..7 = D0..D7, 8..14 = A0..A6, 15 = A7 (USP/ISP/MSP by SR.S/SR.M). //
// Five combinational read ports, one write port (full 32 bits; the core    //
// merges byte and word results), direct access to the stack pointers for   //
// MOVE USP / MOVEC and the exception logic.  D0-D7 and A0-A6 are kept in   //
// LUT RAM (Intel MLAB: synchronous write, asynchronous read), one copy per //
// read port; a flag per register makes it read 0 from reset until it is    //
// first written, as a reset to 0 would.                                    //
//--------------------------------------------------------------------------//

module ap030_regfile
(
	input             clk,
	input             ce,          // clock enable: the core advances on enabled rising edges
	input             rst,
	input             sr_s,
	input             sr_m,

	input             we,
	input       [3:0] waddr,
	input       [1:0] wact,        // for waddr 15: the stack pointer A7 was when the write was issued
	input      [31:0] wdata,

	input       [3:0] raddr_a,
	output     [31:0] rdata_a,
	input       [3:0] raddr_b,
	output     [31:0] rdata_b,
	input       [3:0] raddr_c,
	output     [31:0] rdata_c,
	// dispatch ports: the operands of the instruction being dispatched
	// (separate, so the decoder is not in the paths of the other ports)
	input       [3:0] raddr_d,
	output     [31:0] rdata_d,
	input       [3:0] raddr_e,
	output     [31:0] rdata_e,

	// stack pointers not selected by SR (MOVE USP, MOVEC, RTE stack switch)
	input             sp_we,
	input       [1:0] sp_sel,      // 0 USP 1 ISP 2 MSP
	input      [31:0] sp_wdata,
	output     [31:0] usp_q,
	output     [31:0] isp_q,
	output     [31:0] msp_q
);

// one copy per read port; index 15 (A7) is never written there
(* ramstyle = "MLAB, no_rw_check" *) reg [31:0] ma [0:15];
(* ramstyle = "MLAB, no_rw_check" *) reg [31:0] mb [0:15];
(* ramstyle = "MLAB, no_rw_check" *) reg [31:0] mc [0:15];
(* ramstyle = "MLAB, no_rw_check" *) reg [31:0] md [0:15];
(* ramstyle = "MLAB, no_rw_check" *) reg [31:0] me [0:15];
reg [15:0] zero;           // the register reads 0 (reset, not written since; bit 15 unused)
`ifdef VERILATOR
// the registers as they read (for benches: rf.r[n]; nothing uses it in synthesis)
wire [31:0] r [0:14];
genvar gr;
generate for (gr = 0; gr < 15; gr = gr + 1) begin : g_r
	assign r[gr] = zero[gr] ? 32'd0 : ma[gr];
end endgenerate
`endif
reg [31:0] usp, isp, msp;
wire       mwe = ce && we && (waddr != 4'd15) && !rst;
always @(posedge clk) if (mwe) ma[waddr] <= wdata;
always @(posedge clk) if (mwe) mb[waddr] <= wdata;
always @(posedge clk) if (mwe) mc[waddr] <= wdata;
always @(posedge clk) if (mwe) md[waddr] <= wdata;
always @(posedge clk) if (mwe) me[waddr] <= wdata;

wire [1:0] act = !sr_s ? 2'd0 : (sr_m ? 2'd2 : 2'd1);
`ifdef VERILATOR
wire [31:0] a7 = (act == 2'd0) ? usp : (act == 2'd1) ? isp : msp;   // committed A7 (the benches show it)
`endif

// a write is visible to reads in the same clock it is applied, so a value
// written by one state can be read by the next without a hazard.  A pending
// A7 write is forwarded only to reads of the same stack pointer.
// Plain continuous logic: written as a function (input "i", the name of the
// reset loop's integer, reading the module signals) Quartus 17 did not build
// the A7 forwarding, and the board read the old stack pointer.
// Both write paths are forwarded to every read of the stack pointer they
// write, A7 and the direct outputs alike, sp_we over we as in the block
// below.  The active ISP/MSP is A7 (UM Section 1, supervisor programming
// model): an exception frame (UM 8.1) or a MOVEC in the clock after an A7
// write (a stack adjustment overlapped with the dispatch of an illegal
// instruction, an RTS popping to an odd address) must see the new value,
// and so must an A7 read in the clock after a MOVEC to the active stack
// pointer.
wire [31:0] usp_f  = (sp_we && sp_sel == 2'd0) ? sp_wdata : (we && waddr == 4'd15 && wact == 2'd0) ? wdata : usp;
wire [31:0] isp_f  = (sp_we && sp_sel == 2'd1) ? sp_wdata : (we && waddr == 4'd15 && wact == 2'd1) ? wdata : isp;
wire [31:0] msp_f  = (sp_we && sp_sel[1])      ? sp_wdata : (we && waddr == 4'd15 && wact[1])      ? wdata : msp;
wire [31:0] rd_a7  = (act == 2'd0) ? usp_f : (act == 2'd1) ? isp_f : msp_f;
wire        fwd_a  = we && (waddr == raddr_a) && (raddr_a != 4'd15);
wire        fwd_b  = we && (waddr == raddr_b) && (raddr_b != 4'd15);
wire        fwd_c  = we && (waddr == raddr_c) && (raddr_c != 4'd15);
wire        fwd_d  = we && (waddr == raddr_d) && (raddr_d != 4'd15);
wire        fwd_e  = we && (waddr == raddr_e) && (raddr_e != 4'd15);

assign rdata_a = (raddr_a == 4'd15) ? rd_a7 : fwd_a ? wdata : zero[raddr_a] ? 32'd0 : ma[raddr_a];
assign rdata_b = (raddr_b == 4'd15) ? rd_a7 : fwd_b ? wdata : zero[raddr_b] ? 32'd0 : mb[raddr_b];
assign rdata_c = (raddr_c == 4'd15) ? rd_a7 : fwd_c ? wdata : zero[raddr_c] ? 32'd0 : mc[raddr_c];
assign rdata_d = (raddr_d == 4'd15) ? rd_a7 : fwd_d ? wdata : zero[raddr_d] ? 32'd0 : md[raddr_d];
assign rdata_e = (raddr_e == 4'd15) ? rd_a7 : fwd_e ? wdata : zero[raddr_e] ? 32'd0 : me[raddr_e];
assign usp_q = usp_f;
assign isp_q = isp_f;
assign msp_q = msp_f;

always @(posedge clk) if (ce) begin
	if (rst) begin
		zero <= 16'h7FFF;
		usp <= 32'd0; isp <= 32'd0; msp <= 32'd0;
	end else begin
		if (we) begin
			if (waddr == 4'd15) begin
				// the stack pointer selected when the write was issued: an RTE
				// pops its frame in the same clock it loads a new S/M
				case (wact)
					2'd0: usp <= wdata;
					2'd1: isp <= wdata;
					default: msp <= wdata;
				endcase
			end else zero[waddr] <= 1'b0;
		end
		if (sp_we) begin
			case (sp_sel)
				2'd0: usp <= sp_wdata;
				2'd1: isp <= sp_wdata;
				default: msp <= sp_wdata;
			endcase
		end
	end
end

endmodule
