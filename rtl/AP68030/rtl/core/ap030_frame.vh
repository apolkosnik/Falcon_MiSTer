// AP68030 core - exception stack frame words (UM 8.4, included by ap030_core.v)
//
// frame_word(i) is word i (offset 2*i) of the frame being built.  Formats:
//   $0/$1  4 words   SR, PC, format/vector
//   $2     6 words   + instruction address
//   $9    10 words   + instruction address, 4 internal words
//   $A    16 words   short bus fault
//   $B    46 words   long bus fault
// The internal words of the bus fault frames hold this core's resume state
// (resume kind/state, temporaries); handlers must leave them alone except
// for the SSW rerun bits and the data input buffer, as UM 8.2.2 says.  An
// instruction suspended at an access behind a failed posted write (resume
// kind "held") keeps that access in words 37-38 (address), 45 (size, R/W,
// FC, ...) and 22-23 (write data: the DIB of a write fault, which UM 8.2.2
// leaves to read faults).

function [31:0] frame_long_out;
	input [4:0] idx;
	begin
		case (idx)
			5'd0:  frame_long_out = {exc_throw ? (exc_sr | 16'h2000) : exc_sr, exc_pc[31:16]};   // throwaway: S set (UM 8.1.9)
			5'd1:  frame_long_out = {exc_pc[15:0], exc_fmt, 2'b00, exc_vec, 2'b00};
			default: begin
				if (exc_fmt == `FMT_SIXWORD || exc_fmt == `FMT_CPMID) begin
					case (idx)
						5'd2: frame_long_out = exc_fa;
						5'd3: frame_long_out = {tr_t1, tr_t0, flow, cp_cond, cp_dr, cp_ca, cp_pcbit, cp_trace_wait, cp_kind, cp_id, 1'b0, ir};
						default: frame_long_out = ea;
					endcase
				end else begin
					case (idx)
						5'd2:  frame_long_out = {exc_rk, 4'd0, exc_rs, exc_ssw};
						5'd3:  frame_long_out = {exc_stage_c, exc_stage_b};
						5'd4:  frame_long_out = exc_fa;
						5'd5:  frame_long_out = ea;
						5'd6:  frame_long_out = exc_dob;
						5'd7:  frame_long_out = src;
						5'd8:  frame_long_out = dst;
						5'd9:  frame_long_out = exc_baddr;
						5'd10: frame_long_out = 32'd0;
						// data input buffer; a suspended access keeps its write data there
						5'd11: frame_long_out = (exc_rk == RK_HELD) ? exc_dib : 32'd0;
						5'd12: frame_long_out = tmp;
						5'd13: frame_long_out = {ir, 4'h0, exc_dw_dst, exc_dw_ret};  // version 0
						5'd14: frame_long_out = tmp2;
						5'd15: frame_long_out = ea2;
						5'd16: frame_long_out = {exc_cnt, sub, mm_mask};
						5'd17: frame_long_out = {tmp3[15:0], ea_sel, imm_tgt, dw_reg, 1'b0, ea_pc, ea_ret};
						5'd18: frame_long_out = {exc_got, 5'd0, imm_ret, exc_partial[31:16]};
						5'd19: frame_long_out = {exc_partial[15:0], ext};
						5'd20: frame_long_out = {tr_t1, tr_t0, flow, cp_cond, cp_dr, cp_ca, cp_pcbit, cp_trace_wait, cp_kind, cp_id, 1'b0, cp_len, cp_pos};
						5'd21: frame_long_out = {2'd0, g_alu, g_size, g_srck, g_dstk, g_flags, g_wb, g_dreg, g_sext, g_bitop, g_move_mem, 1'b0, nx};
						5'd22: frame_long_out = {cp_resp, 5'd0, exc_held};
						default: frame_long_out = 32'd0;
					endcase
				end
			end
		endcase
	end
endfunction

// frame length in longwords per format
function [5:0] frame_lw;
	input [3:0] f;
	begin
		case (f)
			`FMT_SIXWORD: frame_lw = 6'd3;
			`FMT_CPMID:   frame_lw = 6'd5;
			`FMT_SHORTBUS: frame_lw = 6'd8;
			`FMT_LONGBUS: frame_lw = 6'd23;
			default: frame_lw = 6'd2;
		endcase
	end
endfunction
