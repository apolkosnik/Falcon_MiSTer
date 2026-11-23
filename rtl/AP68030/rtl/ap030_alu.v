//--------------------------------------------------------------------------//
// AP68030 - MC68030 compatible CPU                                         //
//                                                                          //
// ap030_alu.v - integer ALU: arithmetic, logic, BCD, shifts/rotates with a  //
// single-cycle barrel, bit operations; MC68020/MC68030 CCR semantics       //
//                                                                          //
// Conventions:                                                             //
//  - a is the source, b the destination: "OP src,dst" computes b OP a       //
//  - operands and results live in the low bits selected by size; the core   //
//    merges the result into the destination register                       //
//  - flags are {X,N,Z,V,C}                                                  //
//  - shifts/rotates take the count in shcnt (0..63)                         //
//                                                                          //
// The architecturally undefined flags follow the MC68020/MC68030 silicon   //
// as modelled by WinUAE's 68030 core (BCD N from the result and V cleared,  //
// gencpu "a real 68030 clears V"; DIVx overflow leaves N set, Z clear).    //
//--------------------------------------------------------------------------//

`include "ap030_defs.svh"

module ap030_alu
(
	input             clk,
	input             ce,          // clock enable: the core advances on enabled rising edges
	input       [5:0] op,
	input       [1:0] size,
	input       [5:0] shcnt,
	input      [31:0] a,
	input      [31:0] b,
	input       [4:0] flags_in,
	output reg [31:0] result,
	output reg  [4:0] flags_out
);

wire f_x = flags_in[4];
wire f_z = flags_in[2];

wire        addr_op  = (op == `ALU_ADDA) || (op == `ALU_SUBA) || (op == `ALU_CMPA);
wire [1:0]  eff_size = addr_op ? `SZ_L : size;
wire [5:0]  nbits    = (eff_size == `SZ_B) ? 6'd8 : (eff_size == `SZ_W) ? 6'd16 : 6'd32;
wire [31:0] szmask   = (eff_size == `SZ_B) ? 32'h0000_00FF : (eff_size == `SZ_W) ? 32'h0000_FFFF : 32'hFFFF_FFFF;
wire [31:0] am = a & szmask;
wire [31:0] bm = b & szmask;
wire        a_msb = (eff_size == `SZ_B) ? a[7] : (eff_size == `SZ_W) ? a[15] : a[31];
wire        b_msb = (eff_size == `SZ_B) ? b[7] : (eff_size == `SZ_W) ? b[15] : b[31];

function msb_of;
	input [31:0] r;
	begin
		msb_of = (eff_size == `SZ_B) ? r[7] : (eff_size == `SZ_W) ? r[15] : r[31];
	end
endfunction
function zero_of;
	input [31:0] r;
	begin
		zero_of = ((r & szmask) == 32'd0);
	end
endfunction

//---------------------------------------------------------------------------
// shared adder / subtractor (ADD/ADDX/ADDA share, SUB/SUBX/SUBA/CMP/CMPA/NEG/NEGX share)
//---------------------------------------------------------------------------
wire        is_neg   = (op == `ALU_NEG) || (op == `ALU_NEGX);
wire [31:0] sub_lhs  = is_neg ? 32'd0 : bm;
wire [31:0] sub_rhs  = is_neg ? bm    : am;
wire        add_ext  = (op == `ALU_ADDX) & f_x;
wire        sub_ext  = ((op == `ALU_SUBX) || (op == `ALU_NEGX)) & f_x;
wire [32:0] add_full = {1'b0, bm} + {1'b0, am} + {32'd0, add_ext};
wire [32:0] sub_full = {1'b0, sub_lhs} - {1'b0, sub_rhs} - {32'd0, sub_ext};
wire        add_c    = (eff_size == `SZ_B) ? add_full[8] : (eff_size == `SZ_W) ? add_full[16] : add_full[32];
wire        sub_c    = (eff_size == `SZ_B) ? sub_full[8] : (eff_size == `SZ_W) ? sub_full[16] : sub_full[32];
wire        add_msb  = msb_of(add_full[31:0]);
wire        sub_msb  = msb_of(sub_full[31:0]);
wire        add_v    = (a_msb == b_msb) && (add_msb != a_msb);
wire        sub_v    = is_neg ? (b_msb & sub_msb) : ((a_msb != b_msb) && (sub_msb == a_msb));

//---------------------------------------------------------------------------
// BCD (byte only; SBCD and NBCD share the decimal subtractor)
//---------------------------------------------------------------------------
// ABCD: b + a + X with decimal correction
wire [4:0] abcd_lo   = {1'b0, b[3:0]} + {1'b0, a[3:0]} + {4'd0, f_x};
wire [9:0] abcd_raw  = {2'd0, b[7:0]} + {2'd0, a[7:0]} + {9'd0, f_x} + ((abcd_lo > 5'd9) ? 10'd6 : 10'd0);
wire       abcd_c    = (abcd_raw[9:4] > 6'd9);
wire [9:0] abcd_res  = abcd_raw + (abcd_c ? 10'h60 : 10'd0);
// SBCD (b - a - X) / NBCD (0 - b - X)
wire [7:0] sbcd_a    = (op == `ALU_NBCD) ? b[7:0] : a[7:0];
wire [7:0] sbcd_b    = (op == `ALU_NBCD) ? 8'd0   : b[7:0];
wire       sbcd_lb   = ({1'b0, sbcd_b[3:0]} < ({1'b0, sbcd_a[3:0]} + {4'd0, f_x}));
wire [9:0] sbcd_raw  = {2'd0, sbcd_b} - {2'd0, sbcd_a} - {9'd0, f_x};
wire [9:0] sbcd_cor  = sbcd_raw - (sbcd_lb ? 10'd6 : 10'd0);
wire [9:0] sbcd_res  = sbcd_cor - (sbcd_raw[9] ? 10'h60 : 10'd0);
wire       sbcd_c    = sbcd_cor[9];

//---------------------------------------------------------------------------
// shifter: closed forms of shcnt single-bit steps.  Operands in S_GEN_EXEC
// are already registered in the core (src, dst, g_alu, g_size, sr).
wire  [5:0] sq_op     = op;
wire  [5:0] sq_cnt    = shcnt;
wire  [5:0] sq_nbits  = nbits;
wire [31:0] sq_szmask = szmask;
wire [31:0] sq_bm     = bm;
wire        sq_b_msb  = b_msb;
wire        sq_x      = f_x;
//---------------------------------------------------------------------------
function [31:0] rev_sz;
	input [31:0] v;
	input [1:0]  sz;
	integer i;
	begin
		rev_sz = 32'd0;
		case (sz)
			`SZ_B: for (i = 0; i < 8;  i = i + 1) rev_sz[i] = v[7-i];
			`SZ_W: for (i = 0; i < 16; i = i + 1) rev_sz[i] = v[15-i];
			default: for (i = 0; i < 32; i = i + 1) rev_sz[i] = v[31-i];
		endcase
	end
endfunction

wire shift_left  = (sq_op == `ALU_ASL) || (sq_op == `ALU_LSL);
wire shift_arith = (sq_op == `ALU_ASR);
wire signfill    = shift_arith && sq_b_msb;
wire [31:0] sh_in = shift_left ? rev_sz(sq_bm, size) : (signfill ? (sq_bm | ~sq_szmask) : sq_bm);
wire signed [33:0] sh_sin = {signfill, sh_in, 1'b0};
wire signed [33:0] sh_sr  = sh_sin >>> sq_cnt;
wire [31:0] sh_r = sh_sr[32:1];
wire [31:0] shift_res = (shift_left ? rev_sz(sh_r, size) : sh_r) & sq_szmask;

reg  [31:0] sh_result;
reg   [4:0] sh_flags;
always @* begin : shifter
	reg  [5:0] n, nm, nx, w_amt, w_left;
	reg [32:0] cont, rot, cmask, rin, rmask;
	reg  [5:0] rw;
	reg        rext, rright;
	reg [31:0] r, win, ovf_mask;
	reg        c, x2, vf;
	n = sq_cnt; r = 32'd0; c = 1'b0; x2 = sq_x; vf = 1'b0;
	win = 32'd0; ovf_mask = 32'd0;
	nm = n & (sq_nbits - 6'd1);
	// n mod (size+1) for the extend rotates
	case (size)
		`SZ_B: nx = (n >= 6'd63) ? n - 6'd63 : (n >= 6'd54) ? n - 6'd54 : (n >= 6'd45) ? n - 6'd45 :
		            (n >= 6'd36) ? n - 6'd36 : (n >= 6'd27) ? n - 6'd27 : (n >= 6'd18) ? n - 6'd18 :
		            (n >= 6'd9) ? n - 6'd9 : n;
		`SZ_W: nx = (n >= 6'd51) ? n - 6'd51 : (n >= 6'd34) ? n - 6'd34 : (n >= 6'd17) ? n - 6'd17 : n;
		default: nx = (n >= 6'd33) ? n - 6'd33 : n;
	endcase
	cmask  = (size == `SZ_B) ? 33'h0_0000_01FF : (size == `SZ_W) ? 33'h0_0001_FFFF : 33'h1_FFFF_FFFF;
	cont   = (size == `SZ_B) ? {24'd0, sq_x, sq_bm[7:0]} :
	         (size == `SZ_W) ? {16'd0, sq_x, sq_bm[15:0]} : {sq_x, sq_bm};
	rext   = (sq_op == `ALU_ROXL) || (sq_op == `ALU_ROXR);
	rright = (sq_op == `ALU_ROR) || (sq_op == `ALU_ROXR);
	rw     = sq_nbits + {5'd0, rext};
	w_amt  = rext ? nx : nm;
	w_left = rright ? (rw - w_amt) : w_amt;
	rin    = rext ? cont : {1'b0, sq_bm};
	rmask  = rext ? cmask : {1'b0, sq_szmask};
	rot    = ((rin << w_left) | (rin >> (rw - w_left))) & rmask;
	case (sq_op)
		`ALU_ASL, `ALU_LSL, `ALU_LSR, `ALU_ASR: begin
			r = shift_res;
			c = sh_sr[0];
			x2 = (n == 0) ? sq_x : c;
			if (sq_op == `ALU_ASL) begin
				if (n >= sq_nbits) vf = (sq_bm != 0);
				else if (n != 0) begin
					ovf_mask = (32'd2 << n[4:0]) - 32'd1;
					win = sh_in & ovf_mask;
					vf = (win != 32'd0) && (win != ovf_mask);
				end
			end
		end
		`ALU_ROL: begin
			r = rot[31:0] & sq_szmask;
			c = (n == 0) ? 1'b0 : r[0];
		end
		`ALU_ROR: begin
			r = rot[31:0] & sq_szmask;
			c = (n == 0) ? 1'b0 : msb_of(r);
		end
		default: begin  // ROXL / ROXR
			x2 = (size == `SZ_B) ? rot[8] : (size == `SZ_W) ? rot[16] : rot[32];
			r  = rot[31:0] & sq_szmask;
			c  = x2;                       // count 0: C = X
		end
	endcase
	sh_result = r;
	sh_flags  = {x2, msb_of(r), zero_of(r), vf, c};
end

//---------------------------------------------------------------------------
// bit operations: a holds the bit number, already reduced modulo 8 or 32
//---------------------------------------------------------------------------
wire [31:0] bit_mask = 32'd1 << a[4:0];
wire        bit_set  = |(b & bit_mask);

wire [31:0] ext_res  = (size == `SZ_W) ? {16'd0, {8{b[7]}}, b[7:0]} : {{16{b[15]}}, b[15:0]};

always @* begin
	result    = bm;
	flags_out = flags_in;
	case (op)
		`ALU_MOVE, `ALU_TST: begin
			result = am;
			flags_out = {f_x, msb_of(am), zero_of(am), 1'b0, 1'b0};
		end
		`ALU_MOVEA: begin
			result = a;
		end
		`ALU_ADD, `ALU_ADDA: begin
			result = add_full[31:0] & szmask;
			flags_out = {add_c, add_msb, zero_of(add_full[31:0]), add_v, add_c};
		end
		`ALU_ADDX: begin
			result = add_full[31:0] & szmask;
			flags_out = {add_c, add_msb, f_z & zero_of(add_full[31:0]), add_v, add_c};
		end
		`ALU_SUB, `ALU_SUBA, `ALU_NEG: begin
			result = sub_full[31:0] & szmask;
			flags_out = {sub_c, sub_msb, zero_of(sub_full[31:0]), sub_v, sub_c};
		end
		`ALU_SUBX, `ALU_NEGX: begin
			result = sub_full[31:0] & szmask;
			flags_out = {sub_c, sub_msb, f_z & zero_of(sub_full[31:0]), sub_v, sub_c};
		end
		`ALU_CMP, `ALU_CMPA: begin
			result = bm;
			flags_out = {f_x, sub_msb, zero_of(sub_full[31:0]), sub_v, sub_c};
		end
		`ALU_AND: begin
			result = bm & am;
			flags_out = {f_x, msb_of(bm & am), zero_of(bm & am), 1'b0, 1'b0};
		end
		`ALU_OR: begin
			result = bm | am;
			flags_out = {f_x, msb_of(bm | am), zero_of(bm | am), 1'b0, 1'b0};
		end
		`ALU_EOR: begin
			result = bm ^ am;
			flags_out = {f_x, msb_of(bm ^ am), zero_of(bm ^ am), 1'b0, 1'b0};
		end
		`ALU_NOT: begin
			result = (~bm) & szmask;
			flags_out = {f_x, msb_of(~bm), zero_of(~bm), 1'b0, 1'b0};
		end
		`ALU_CLR: begin
			result = 32'd0;
			flags_out = {f_x, 1'b0, 1'b1, 1'b0, 1'b0};
		end
		`ALU_EXT: begin
			result = ext_res;
			flags_out = {f_x, msb_of(ext_res), zero_of(ext_res), 1'b0, 1'b0};
		end
		`ALU_EXTB: begin
			result = {{24{b[7]}}, b[7:0]};
			flags_out = {f_x, b[7], (b[7:0] == 8'd0), 1'b0, 1'b0};
		end
		`ALU_SWAP: begin
			result = {b[15:0], b[31:16]};
			flags_out = {f_x, b[15], (b == 32'd0), 1'b0, 1'b0};
		end
		`ALU_TAS: begin
			result = {24'd0, 1'b1, b[6:0]};
			flags_out = {f_x, b[7], (b[7:0] == 8'd0), 1'b0, 1'b0};
		end
		`ALU_ABCD: begin
			result = {24'd0, abcd_res[7:0]};
			flags_out = {abcd_c, abcd_res[7], f_z & (abcd_res[7:0] == 8'd0), 1'b0, abcd_c};
		end
		`ALU_SBCD, `ALU_NBCD: begin
			result = {24'd0, sbcd_res[7:0]};
			flags_out = {sbcd_c, sbcd_res[7], f_z & (sbcd_res[7:0] == 8'd0), 1'b0, sbcd_c};
		end
		`ALU_ASL, `ALU_ASR, `ALU_LSL, `ALU_LSR, `ALU_ROL, `ALU_ROR, `ALU_ROXL, `ALU_ROXR: begin
			result = sh_result;
			flags_out = sh_flags;
		end
		`ALU_BTST: begin
			result = bm;
			flags_out = {f_x, flags_in[3], ~bit_set, flags_in[1], flags_in[0]};
		end
		`ALU_BCHG: begin
			result = (bm ^ bit_mask) & szmask;
			flags_out = {f_x, flags_in[3], ~bit_set, flags_in[1], flags_in[0]};
		end
		`ALU_BCLR: begin
			result = bm & ~bit_mask;
			flags_out = {f_x, flags_in[3], ~bit_set, flags_in[1], flags_in[0]};
		end
		`ALU_BSET: begin
			result = (bm | bit_mask) & szmask;
			flags_out = {f_x, flags_in[3], ~bit_set, flags_in[1], flags_in[0]};
		end
		default: ;
	endcase
end

endmodule
