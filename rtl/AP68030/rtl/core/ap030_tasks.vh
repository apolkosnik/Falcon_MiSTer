// AP68030 core - sequencer tasks (included by ap030_core.v)
//
// All tasks are called from the clocked sequencer block and use
// nonblocking assignments; the pipe commands (pop_n, flush_req) are
// blocking per-clock variables consumed at the end of the block.

// data access: the result goes to dw_dst, execution continues at dw_ret
task dreq;
	input [31:0] addr;
	input  [1:0] size;
	input        rw;
	input [31:0] wdata;
	input  [2:0] fc;
	input        rmc;
	input        rmc_last;
	input  [3:0] dst_sel;
	input  [7:0] ret;
	begin
		d_stb <= 1'b1;
		d_addr <= addr; d_size <= size; d_rw <= rw; d_wdata <= wdata; d_fc <= fc;
		d_rmc <= rmc; d_rmc_last <= rmc_last; d_iack <= 1'b0; d_nocache <= 1'b0;
		dw_dst <= dst_sel; dw_ret <= ret;
		state <= S_DWAIT;
	end
endtask

// Complete an effective address and launch a generic operand read in the
// same clock. The read uses this address and the EA's program/data space;
// ea and ea_pc are still being registered, so neither is read back here.
task ea_ready;
 input [31:0] addr;
 begin
  ea <= addr;
  if (ea_ret == S_GEN_RD)
   dreq(addr, g_size, 1'b1, 32'd0,
        (ea_pcrel && PCREL_PROGRAM_SPACE) ? fc_prog : fc_data,
        1'b0, 1'b0, (g_srck == SK_EA) ? DW_SRC : DW_DST,
        g_move_mem ? S_MOVE_DEA : nx);
  else if (ea_ret == S_MOVE_WR) begin
   // MOVE's source and ALU controls are already registered. Its store
   // and flags can be issued with the completed destination address.
   sr[4:0] <= alu_f;
   finish;
   wr(addr, g_size, src, S_FETCH);
  end else if (ea_ret == S_LEA) begin
   wreg({1'b1, ir[11:9]}, addr);
   finish;
  end else state <= ea_ret;
 end
endtask

// convenience wrappers
task rd;   input [31:0] addr; input [1:0] size; input [3:0] dst_sel; input [7:0] ret;
	begin dreq(addr, size, 1'b1, 32'd0, (ea_pc && PCREL_PROGRAM_SPACE) ? fc_prog : fc_data, 1'b0, 1'b0, dst_sel, ret); end endtask
task wr;   input [31:0] addr; input [1:0] size; input [31:0] data; input [7:0] ret;
	begin dreq(addr, size, 1'b0, data, fc_data, 1'b0, 1'b0, DW_NONE, ret); end endtask
task rd_sd; input [31:0] addr; input [1:0] size; input [3:0] dst_sel; input [7:0] ret;   // supervisor data
	begin dreq(addr, size, 1'b1, 32'd0, `FC_SUPER_DATA, 1'b0, 1'b0, dst_sel, ret); end endtask
task wr_sd; input [31:0] addr; input [1:0] size; input [31:0] data; input [7:0] ret;
	begin dreq(addr, size, 1'b0, data, `FC_SUPER_DATA, 1'b0, 1'b0, DW_NONE, ret); end endtask
// coprocessor interface register access (CPU space type 2)
task cir_rd; input [4:0] off; input [1:0] size; input [3:0] dst_sel; input [7:0] ret;
	begin dreq(cp_base | {27'd0, off}, size, 1'b1, 32'd0, `FC_CPU_SPACE, 1'b0, 1'b0, dst_sel, ret); end endtask
task cir_wr; input [4:0] off; input [1:0] size; input [31:0] data; input [7:0] ret;
	begin dreq(cp_base | {27'd0, off}, size, 1'b0, data, `FC_CPU_SPACE, 1'b0, 1'b0, DW_NONE, ret); end endtask

// register write (full 32 bits)
// A7 means the stack pointer selected by S and M now, when the write is
// issued; the register file commits a clock later, after a same-clock SR
// change (RTE) has already switched stacks
task wreg; input [3:0] idx; input [31:0] val;
	begin
		rf_we <= 1'b1; rf_waddr <= idx; rf_wdata <= val;
		rf_wact <= !sr_s ? 2'd0 : (sr_m ? 2'd2 : 2'd1);
		if (state == S_GEN_EXEC || state == S_MOVEM1 || state == S_DBCC) begin
			byp_we = 1'b1; byp_reg = idx; byp_data = val;
		end
	end endtask
// register write merged by size (old value supplied)
task wreg_sz; input [3:0] idx; input [31:0] old; input [31:0] val; input [1:0] sz;
	begin wreg(idx, merge(old, val, sz)); end endtask

// control register read/write (shared by MOVEC and cpCTRL)
function [32:0] ctrl_rd;
	input [11:0] sel;
	begin
		case (sel)
			12'h000: ctrl_rd = {1'b0, 29'd0, sfc};
			12'h001: ctrl_rd = {1'b0, 29'd0, dfc};
			12'h002: ctrl_rd = {1'b0, cacr};
			12'h800: ctrl_rd = {1'b0, usp_q};
			12'h801: ctrl_rd = {1'b0, vbr};
			12'h802: ctrl_rd = {1'b0, caar};
			12'h803: ctrl_rd = {1'b0, msp_q};
			12'h804: ctrl_rd = {1'b0, isp_q};
			default: ctrl_rd = {1'b1, 32'd0};
		endcase
	end
endfunction

task ctrl_wr;
	input [11:0] sel;
	input [31:0] wval;
	begin
		case (sel)
			12'h000: sfc <= wval[2:0];
			12'h001: dfc <= wval[2:0];
			12'h002: begin
				cacr <= wval & `CACR_RDMASK;
				cacr_ci <= wval[`CACR_CI]; cacr_cei <= wval[`CACR_CEI];
				cacr_cd <= wval[`CACR_CD]; cacr_ced <= wval[`CACR_CED];
			end
			12'h800: begin sp_we <= 1'b1; sp_sel <= 2'd0; sp_wdata <= wval; end
			12'h801: vbr <= wval;
			12'h802: caar <= wval;
			12'h803: begin sp_we <= 1'b1; sp_sel <= 2'd2; sp_wdata <= wval; end
			default: begin sp_we <= 1'b1; sp_sel <= 2'd1; sp_wdata <= wval; end
		endcase
	end
endtask

// instruction complete: trace disposition, back to the boundary
task finish;
	begin
		trace_pend <= tr_t1 | (tr_t0 & flow);
		state <= S_FETCH;
	end
endtask

// instruction complete with same-clock dispatch when no boundary event waits
task finish_dispatch;
	begin
		if (!tr_t1 && !(tr_t0 & flow) && !irq_pend && !late_fault_pend && !stopped &&
		    w0_v && !w0_f && !(dc_needs_ext && (!w1_v || w1_f)))
			disp_req = 1'b1;
		else
			finish;
	end
endtask

// change of flow to a new PC: flush the pipe (address error on odd)
task go_pc;
	input [31:0] npc;
	begin
		flow <= 1'b1;
		if (npc[0]) begin
			exc_addr_err(npc);
		end else begin
			flush_req = 1'b1;
			flush_pc = npc;
			trace_pend <= tr_t1 | tr_t0;
			state <= S_FETCH;
		end
	end
endtask

// generic exception entry (UM 8.1): the frame fields are captured here
task exc_go;
	input  [7:0] vec;
	input  [3:0] fmt;
	input [31:0] pcv;
	input [31:0] iav;
	begin
		exc_vec <= vec; exc_fmt <= fmt; exc_pc <= pcv;
		if (fmt == `FMT_SIXWORD || fmt == `FMT_CPMID) exc_fa <= iav;
		exc_sr <= sr;
		exc_is_irq <= 1'b0; exc_throw <= 1'b0; exc_late <= 1'b0;
		exc_busfault <= (vec == `VEC_BUSERR) || (vec == `VEC_ADDRERR);
		// A trace pending for an instruction whose own exception is part of
		// its execution is taken after that exception's processing (UM 8.1.4,
		// 8.1.12 Table 8-5): TRAP #n, TRAPcc/TRAPV/cpTRAPcc, CHK/CHK2 and divide
		// by zero (instruction traps, traced in both trace modes, UM 8.1.7),
		// and a coprocessor post-instruction exception (UM 10.5.2.5).  Decided
		// by the instruction, not the vector: an interrupt or a coprocessor
		// can supply any vector, and a pre-instruction exception (not
		// executed) or a mid-instruction one (resumed) has no trace now
		trace_after_exc <= (vec != `VEC_BUSERR) &&
		                   (((state == S_TRAP || state == S_TRAPCC || state == S_CHK || state == S_CHK2_2 ||
		                      state == S_DIVZ || state == S_CPTRAP2) && (tr_t1 | tr_t0)) ||
		                    (state == S_CP_EXC && sub[1:0] == 2'd2 && (tr_t1 | cp_trace_wait)));
		trace_pend <= 1'b0;
		if (vec == `VEC_BUSERR || vec == `VEC_ADDRERR || vec == `VEC_SPURIOUS || vec == `VEC_FLINE ||
		    (vec >= `VEC_AUTOVEC && vec < `VEC_TRAP)) status_cnt <= 2'd3;
		// a bus/address error stacks its frame after the posted writes
		// before it (see the posted write faults in ap030_core.v)
		state <= (((vec == `VEC_BUSERR) || (vec == `VEC_ADDRERR)) && d_wpend) ? S_EXCW : S_EXC0;
	end
endtask

// pre-instruction exceptions (illegal, privilege, A/F-line): PC = instruction
task exc_pre; input [7:0] vec;
	begin exc_go(vec, `FMT_NORMAL, pc_i, 32'd0); end endtask
// post-instruction six-word frame (TRAPcc, CHK, divide by zero, MMU config): PC = next instruction
task exc_post; input [7:0] vec;
	begin exc_go(vec, `FMT_SIXWORD, scan_pc, pc_i); end endtask
task go_illegal; begin exc_pre(`VEC_ILLEGAL); end endtask
task go_priv;    begin exc_pre(`VEC_PRIV); end endtask

// stage images for the bus fault frames
task capture_pipe;
	begin
		exc_stage_c <= w0;
		exc_stage_b <= w1;
		exc_baddr <= scan_pc + 32'd2;
	end
endtask

// data access fault (from the memory system's fault record)
task exc_data_fault;
	input        boundary;      // at an instruction boundary (posted write): short frame
	input  [3:0] rkind;
	input  [7:0] rstate;
	begin
		capture_pipe;
		exc_fa <= f_addr;
		exc_dob <= f_dob;
		exc_got <= f_got;
		exc_partial <= f_partial;
		// SSW: FC FB RC RB 0 0 0 DF RM RW SIZE 0 FC2-FC0 (UM Figure 8-9)
		exc_ssw <= {1'b0, 1'b0, ~w0_v | w0_f, ~w1_v | w1_f, 3'b000, 1'b1, f_rm, f_rw, f_size, 1'b0, f_fc};
		exc_rk <= rkind; exc_rs <= rstate;
		exc_go(`VEC_BUSERR, boundary ? `FMT_SHORTBUS : `FMT_LONGBUS, boundary ? scan_pc : pc_i, 32'd0);
	end
endtask

// a posted write that failed, reported at a boundary from its record
task exc_late_fault;
	begin
		late_fault_pend <= 1'b0;
		d_unhold <= 1'b1;               // no access waits at a boundary
		capture_pipe;
		exc_fa <= lf_fa; exc_dob <= lf_dob;
		exc_got <= 3'd0; exc_partial <= 32'd0;
		exc_ssw <= {1'b0, 1'b0, ~w0_v | w0_f, ~w1_v | w1_f, 3'b000, lf_ssw};
		exc_rk <= RK_BOUNDARY; exc_rs <= S_FETCH;
		exc_go(`VEC_BUSERR, `FMT_SHORTBUS, scan_pc, 32'd0);
		exc_late <= 1'b1;
	end
endtask

// A posted write failed while the instruction in execution waits with its
// next data access, which has not reached the bus (the memory system holds
// it).  The bus error is taken now, mid-instruction: a long frame with the
// instruction's address as the PC, DF and the record of the failed write
// (UM 8.1.2, Table 8-6: "may not be the instruction that generated the
// faulted bus cycle").  The held access is dropped and kept in the frame;
// RTE reruns the write (DF set) and then issues it (S_RTE_HELD).
task exc_held_fault;
	begin
		late_fault_pend <= 1'b0;
		d_cancel <= 1'b1;
		capture_pipe;
		exc_fa <= lf_fa; exc_dob <= lf_dob;
		exc_got <= 3'd0;
		exc_partial <= d_addr;
		exc_dib <= d_wdata;
		exc_held <= {d_size, d_rw, d_fc, d_rmc, d_rmc_last, d_nocache, cpu_flt_ill, cpu_flt_fline};
		cpu_flt_ill <= 1'b0; cpu_flt_fline <= 1'b0;
		exc_ssw <= {1'b0, 1'b0, ~w0_v | w0_f, ~w1_v | w1_f, 3'b000, lf_ssw};
		exc_rk <= RK_HELD; exc_rs <= S_DWAIT;
		exc_go(`VEC_BUSERR, `FMT_LONGBUS, pc_i, 32'd0);
	end
endtask

// instruction stream fault: the word(s) needed are marked faulted
task exc_stream_fault;
	input need_b;      // the instruction also needed stage B
	input boundary;
	begin
		sf_req = 1'b1;
		sf_need_b = need_b;
		sf_boundary = boundary;
	end
endtask

// address error: prefetch from an odd address (UM 8.1.3)
task exc_addr_err;
	input [31:0] target;
	begin
		exc_stage_c <= 16'd0; exc_stage_b <= 16'd0;
		exc_baddr <= target + 32'd2;
		exc_fa <= 32'd0; exc_dob <= 32'd0; exc_got <= 3'd0; exc_partial <= 32'd0;
		exc_ssw <= {1'b0, 1'b0, 1'b1, 1'b1, 3'b000, 1'b0, 1'b0, 1'b1, 2'b00, 1'b0, fc_prog};
		exc_rk <= RK_BOUNDARY; exc_rs <= S_FETCH;
		exc_go(`VEC_ADDRERR, `FMT_LONGBUS, pc_i, 32'd0);
	end
endtask

// pop k words of the pipe (the caller checked they are present)
task pop; input [1:0] k; begin pop_n = k; end endtask

// consume an immediate/extension longword or word into imm (sign extended
// when asked); returns 0 when the words are not there yet or faulted
function pipe_ready;
	input two;
	begin
		pipe_ready = two ? have2 : w0_v;
	end
endfunction
function pipe_faulted;
	input two;
	begin
		pipe_faulted = two ? ((w0_v & w0_f) | (w1_v & w1_f) | (w0_v & !w0_f & w1_v & w1_f)) : (w0_v & w0_f);
	end
endfunction

// dispatch the next instruction from pq[0]
task dispatch;
	begin
		dbg_inst <= 1'b1;
		ir <= w0;
		ext <= w1;
		pc_i <= scan_pc;
		pop(dc_needs_ext ? 2'd2 : 2'd1);
		tr_t1 <= sr[`SR_T1];
		tr_t0 <= sr[`SR_T0];
		flow <= 1'b0;
		ea_sel <= 1'b0;
		ea_pc <= 1'b0;
		imm_tgt <= 1'b0;
		status_cnt <= 2'd1;
		// generic-path controls, so the decoder can look at the next word
		// while this instruction executes
		g_alu <= dc_alu; g_size <= dc_size; g_srck <= dc_srck; g_dstk <= dc_dstk; g_dreg <= dc_dreg;
		g_flags <= dc_flags; g_wb <= dc_wb; g_sext <= dc_sext; g_bitop <= dc_bitop;
		g_move_mem <= dc_move_mem;
		nx <= dc_first;
		// register operands (with the bypass of a write applied this clock)
		if (dc_srck == SK_REG) src <= (byp_we && byp_reg == dc_sreg) ? byp_data : rf_d;
		else if (dc_srck == SK_QUICK) src <= dc_quick;
		if (dc_dstk == DK_REG) dst <= (byp_we && byp_reg == dc_dreg) ? byp_data : rf_e;
		// exceptions detected at dispatch stack this instruction's address,
		// which is scan_pc in this clock (pc_i is being updated)
		if (dc_illegal) begin
			if (w0[15:12] == 4'hA) exc_go(`VEC_ALINE, `FMT_NORMAL, scan_pc, 32'd0);
			else if (w0[15:12] == 4'hF) begin
				if (!sr_s && w0[11:9] == 3'd0) exc_go(`VEC_PRIV, `FMT_NORMAL, scan_pc, 32'd0);
				else exc_go(`VEC_FLINE, `FMT_NORMAL, scan_pc, 32'd0);
			end else exc_go(`VEC_ILLEGAL, `FMT_NORMAL, scan_pc, 32'd0);
		end else if (dc_priv && !sr_s) begin
			exc_go(`VEC_PRIV, `FMT_NORMAL, scan_pc, 32'd0);
		end else if (dc_srck == SK_IMM) begin
			state <= S_IMM;
			imm_ret <= dc_move_mem ? S_MOVE_DEA : (dc_dstk == DK_EA) ? S_EA : (dc_dstk == DK_IMM) ? S_IMM : dc_first;
			ea_ret <= dc_dstrd ? S_GEN_RD : dc_first;
		end else if (dc_srck == SK_EA) begin
			state <= S_EA;
			ea_ret <= S_GEN_RD;
		end else if (dc_dstk == DK_EA) begin
			if (dc_move_mem) begin ea_sel <= 1'b1; ea_ret <= S_MOVE_WR; state <= S_EA; end
			else begin state <= S_EA; ea_ret <= dc_dstrd ? S_GEN_RD : dc_first; end
		end else if (dc_dstk == DK_IMM) begin
			state <= S_IMM; imm_tgt <= 1'b1; imm_ret <= dc_first;
		end else if (dc_eaonly) begin
			state <= S_EA; ea_ret <= dc_first;
		end else begin
			state <= dc_first;
		end
	end
endtask
