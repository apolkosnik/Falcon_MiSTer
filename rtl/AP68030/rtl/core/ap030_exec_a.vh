// AP68030 core - states part A: boundary, data wait, immediates, effective
// addresses, the generic read-operate-write path, MOVE, program flow,
// linkage, and the simpler special instructions

//-------------------------------------------------------------- reset (UM 8.1.1)
S_RESET0: begin
	exc_active <= 1'b1; exc_busfault <= 1'b1;
	status_cnt <= 2'd3;
	dreq(32'd0, `SZ_L, 1'b1, 32'd0, `FC_SUPER_PROG, 1'b0, 1'b0, DW_TMP, S_RESET1);
end
S_RESET1: begin
	sp_we <= 1'b1; sp_sel <= 2'd1; sp_wdata <= tmp;
	dreq(32'd4, `SZ_L, 1'b1, 32'd0, `FC_SUPER_PROG, 1'b0, 1'b0, DW_TMP, S_RESET2);
end
S_RESET2: begin
	exc_busfault <= 1'b0; exc_active <= 1'b0; fetch_hold <= 1'b0;
	pc_i <= tmp;
	if (tmp[0]) begin halted_r <= 1'b1; state <= S_HALT; end
	else begin flush_req = 1'b1; flush_pc = tmp; state <= S_FETCH; end
end
S_HALT: begin
	halted_r <= 1'b1;
end

//-------------------------------------------------------------- instruction boundary
S_FETCH: begin
	if (late_fault_pend) begin
		exc_late_fault;
	end else if (trace_pend) begin
		trace_pend <= 1'b0;
		status_cnt <= 2'd2;
		exc_go(`VEC_TRACE, `FMT_SIXWORD, scan_pc, pc_i);
	end else if (irq_pend) begin
		// not while a posted write is outstanding: it may be the one that withdraws the
		// request (an interrupt mask register written just before the SR is lowered),
		// and the acknowledge it would leave behind finds no source: a spurious interrupt
		if (!d_wpend) begin
			stopped <= 1'b0;
			status_cnt <= 2'd2;
			exc_ilvl <= irq_lvl;
			state <= S_IACK;
		end
	end else if (stopped) begin
		// STOP: wait for an interrupt (UM 8.1.7 / PRM STOP)
	end else if (w0_v) begin
		if (w0_f) exc_stream_fault(1'b0, 1'b1);
		else if (dc_needs_ext && !w1_v) ;                    // second word not yet here
		else if (dc_needs_ext && w1_f) exc_stream_fault(1'b1, 1'b1);
		else disp_req = 1'b1;
	end
end

//-------------------------------------------------------------- interrupt acknowledge (UM 7.4.1)
S_IACK: begin
	d_stb <= 1'b1; d_iack <= 1'b1;
	d_addr <= {28'hFFFFFFF, 1'b1, exc_ilvl}; d_addr[0] <= 1'b1; d_addr[3:1] <= exc_ilvl;
	d_size <= `SZ_B; d_rw <= 1'b1; d_fc <= `FC_CPU_SPACE; d_rmc <= 1'b0; d_rmc_last <= 1'b0; d_wdata <= 32'd0;
	dw_dst <= DW_TMP; dw_ret <= S_EXC0;
	ipend_r <= 1'b0;
	state <= S_DWAIT;
end

//-------------------------------------------------------------- data access wait
S_DWAIT: begin
	if (d_iack && (d_ack || d_iack_berr)) begin
		// interrupt vector: supplied, autovector, or spurious (bus error)
		d_iack <= 1'b0;
		// a coprocessor busy/not-ready service stacks the instruction's own
		// address so that it restarts (UM 10.4.3); a come-again null
		// primitive with IA stacks a coprocessor mid-instruction frame, whose
		// RTE reads the response CIR again (UM 10.4.8); otherwise the next one
		exc_go(d_iack_berr ? `VEC_SPURIOUS : (d_avec ? (`VEC_AUTOVEC + {5'd0, exc_ilvl}) : d_rdata[7:0]),
		       iack_cpmid ? `FMT_CPMID : `FMT_NORMAL, iack_pc_i ? pc_i : scan_pc, iack_cpmid ? pc_i : 32'd0);
		iack_pc_i <= 1'b0;
		iack_cpmid <= 1'b0;
		exc_is_irq <= 1'b1;
		if (exc_ilvl == 3'd7) irq_taken7 <= 1'b1;
	end else if (rte_fake || d_ack) begin : deliver
		reg [31:0] data;
		reg  [1:0] dsz;
		data = rte_fake ? exc_dib : d_rdata;
		dsz  = d_size;
		if (rerun_merge) begin
			// UM 8.2.1: a rerun of the second portion completes the operand;
			// the rerun moves the SSW SIZE bytes, the operand is those plus
			// the bytes already read
			data = (exc_partial << (8 * bytes_of_sz(sz_of_siz(exc_ssw[5:4])))) | data;
			dsz  = sz_of_bytes(exc_got + bytes_of_sz(sz_of_siz(exc_ssw[5:4])));
			rerun_merge <= 1'b0;
		end
		rte_fake <= 1'b0;
		case (dw_dst)
			DW_SRC:  src <= data;
			DW_DST:  dst <= data;
			DW_TMP:  tmp <= data;
			DW_TMP2: tmp2 <= data;
			DW_EA:   ea <= data;
			DW_REG:  wreg(dw_reg, merge(dst, data, dsz));
			DW_REGL: wreg(dw_reg, sext_sz(data, dsz));
			DW_SR:   sr <= data[15:0] & `SR_MASK;
			DW_FRAME: begin
				cnt <= cnt + 8'd1;
				case (cnt[4:0])
					5'd2: if (exc_fmt == `FMT_CPMID) pc_i <= data;
					      else begin
					      	exc_rk <= (exc_fmt == `FMT_SHORTBUS) ? RK_BOUNDARY : data[31:28];
					      	exc_rs <= data[23:16];
					      	exc_ssw <= data[15:0];
					      end
					5'd3: if (exc_fmt == `FMT_CPMID) begin
					      	{cp_cond, cp_dr, cp_ca, cp_pcbit, cp_trace_wait, cp_kind, cp_id} <= data[28:17];
					      	ir <= data[15:0];
					      end else begin
					      	exc_stage_c <= data[31:16];
					      	exc_stage_b <= data[15:0];
					      end
					5'd4: if (exc_fmt == `FMT_CPMID) ea <= data;
					      else exc_fa <= data;
					5'd5: ea <= data;
					5'd6: exc_dob <= data;
					5'd7: src <= data;
					5'd8: dst <= data;
					5'd9: exc_baddr <= data;
					5'd11: exc_dib <= data;
					5'd12: tmp <= data;
					5'd13: begin
						ir <= data[31:16];
						rte_ver_err <= (data[15:12] != 4'h0);
						exc_dw_dst <= data[11:8]; exc_dw_ret <= data[7:0];
					end
					5'd14: tmp2 <= data;
					5'd15: ea2 <= data;
					5'd16: begin
						exc_cnt <= data[31:24]; sub <= data[23:16];
						mm_mask <= data[15:0];
					end
					5'd17: begin
						tmp3 <= {1'b0, data[31:16]};

						{ea_sel, imm_tgt, dw_reg} <= data[15:10];
						ea_pc <= data[8]; ea_ret <= data[7:0];
					end
					5'd18: begin
						exc_got <= data[31:29]; imm_ret <= data[23:16];
						exc_partial[31:16] <= data[15:0];
					end
					5'd19: begin
						exc_partial[15:0] <= data[31:16];
						ext <= data[15:0];
					end
					5'd20: begin
						{tr_t1, tr_t0, flow, cp_cond, cp_dr, cp_ca, cp_pcbit, cp_trace_wait, cp_kind, cp_id} <= data[31:17];
						{cp_len, cp_pos} <= data[15:0];
					end
					5'd21: begin
						{g_alu, g_size, g_srck, g_dstk, g_flags, g_wb} <= data[29:16];
						{g_dreg, g_sext, g_bitop, g_move_mem} <= data[15:9];
						nx <= data[7:0];
					end
					5'd22: begin
						cp_resp <= data[31:16];
						exc_held <= data[10:0];
					end
					default: ;
				endcase
			end
			default: ;
		endcase
		if (dw_ret == S_MOVE_DEA) begin
			ea_sel <= 1'b1; ea_ret <= S_MOVE_WR; state <= S_EA;
		end else if (dw_ret == S_JMP && !ea[0]) begin
			go_pc(ea);
		end else state <= dw_ret;
	end else if (d_fault) begin
		if (exc_busfault) begin
			// UM 7.5.4: double bus fault
			halted_r <= 1'b1; state <= S_HALT;
		end else if (cpu_flt_ill) begin
			cpu_flt_ill <= 1'b0; exc_pre(`VEC_ILLEGAL);           // BKPT acknowledge terminated by BERR
		end else if (cpu_flt_fline) begin
			cpu_flt_fline <= 1'b0; exc_pre(`VEC_FLINE);           // no coprocessor answered (UM 10.5.2.8)
		end else if (exc_active) begin
			exc_data_fault(1'b1, RK_BOUNDARY, S_FETCH);
		end else begin
			// an RMW fault is rerun as a whole instruction, or the instruction
			// counts as emulated when DF is cleared (UM 8.2.2)
			exc_data_fault(1'b0, d_rmc ? RK_RMW : d_rw ? RK_READ : RK_WRITE, S_DWAIT);
		end
	end else if (d_held && late_fault_pend) begin
		// a posted write failed and this access waits behind it: suspend the
		// instruction here (UM 8.1.2); exception processing, the RTE's frame
		// loads and IACK cannot be suspended, they go on and the fault is
		// taken at the next boundary
		if (exc_active || exc_busfault || d_iack) d_unhold <= 1'b1;
		else exc_held_fault;
	end
end

//-------------------------------------------------------------- immediate operand
S_IMM: begin : imm_state
	reg two;
	two = !imm_tgt && (g_size == `SZ_L) && !g_bitop;
	if (!pipe_ready(two)) ;
	else if (pipe_faulted(two)) exc_stream_fault(two, 1'b0);
	else begin
		if (imm_tgt) dst <= {24'd0, w0[7:0]};
		else if (g_bitop) src <= {24'd0, w0[7:0]};      // bit number: one word, low byte
		else if (two) src <= {w0, w1};
		else if (g_size == `SZ_B) src <= {24'd0, w0[7:0]};
		else src <= {16'd0, w0};
		pop(two ? 2'd2 : 2'd1);
		imm_tgt <= 1'b1;
		if (imm_ret == S_MOVE_DEA) begin
			ea_sel <= 1'b1; ea_ret <= S_MOVE_WR; state <= S_EA;
		end else state <= imm_ret;
	end
end

//-------------------------------------------------------------- effective address (PRM 2.2)
S_EA: begin : ea_state
	reg [31:0] bytes;
	bytes = size_bytes(g_size, ea_regn == 3'd7);
	ea_pc <= ea_pcrel;          // the reads that follow use program space for PC-relative modes
	case (ea_mode)
		3'b010: begin ea_ready(rf_a); end
		3'b011: begin
			ea_ready(rf_a); wreg({1'b1, ea_regn}, rf_a + bytes);
			// a register destination that is this base register sees the update
			if (g_dstk == DK_REG && g_dreg == {1'b1, ea_regn}) dst <= rf_a + bytes;
		end
		3'b100: begin
			ea_ready(rf_a - bytes); wreg({1'b1, ea_regn}, rf_a - bytes);
			if (g_dstk == DK_REG && g_dreg == {1'b1, ea_regn}) dst <= rf_a - bytes;
			// MOVE An,-(An) stores the initial value of An: the source is read
			// before the destination address is formed (WinUAE 68030 cputest)
		end
		3'b101: begin
			if (!w0_v) ; else if (w0_f) exc_stream_fault(1'b0, 1'b0);
			else begin ea_ready(rf_a + sext16(w0)); pop(2'd1); end
		end
		3'b110: begin
			if (!w0_v) ; else if (w0_f) exc_stream_fault(1'b0, 1'b0);
			else if (!w0[8]) begin
				ea_ready(rf_a + sext8(w0[7:0]) + index_val(rf_c, w0));
				pop(2'd1);
			end else begin
				tmp <= w0[7] ? 32'd0 : rf_a;
				tmp2 <= w0[6] ? 32'd0 : index_val(rf_c, w0);
				sub <= w0[7:0];
				pop(2'd1); state <= S_EA_FULL;
			end
		end
		default: begin
			case (ea_regn)
				3'b000: begin
					if (!w0_v) ; else if (w0_f) exc_stream_fault(1'b0, 1'b0);
					else begin ea_ready(sext16(w0)); pop(2'd1); end
				end
				3'b001: begin
					if (!have2) ; else if (pipe_faulted(1'b1)) exc_stream_fault(1'b1, 1'b0);
					else begin ea_ready({w0, w1}); pop(2'd2); end
				end
				3'b010: begin
					if (!w0_v) ; else if (w0_f) exc_stream_fault(1'b0, 1'b0);
					else begin ea_ready(scan_pc + sext16(w0)); pop(2'd1); end
				end
				3'b011: begin
					if (!w0_v) ; else if (w0_f) exc_stream_fault(1'b0, 1'b0);
					else if (!w0[8]) begin
						ea_ready(scan_pc + sext8(w0[7:0]) + index_val(rf_c, w0));
						pop(2'd1);
					end else begin
						tmp <= w0[7] ? 32'd0 : scan_pc;
						tmp2 <= w0[6] ? 32'd0 : index_val(rf_c, w0);
						sub <= w0[7:0];
						pop(2'd1); state <= S_EA_FULL;
					end
				end
				default: begin
					// immediate as a destination address is meaningless; illegal
					go_illegal;
				end
			endcase
		end
	endcase
end

S_EA_FULL: begin : ea_full
	// base displacement per BD SIZE, then the indirection of PRM Table 2-2
	reg [31:0] bd;
	reg ok, flt, two;
	case (sub[5:4])
		2'b10: begin two = 1'b0; ok = w0_v; flt = w0_v & w0_f; bd = sext16(w0); end
		2'b11: begin two = 1'b1; ok = have2; flt = pipe_faulted(1'b1); bd = {w0, w1}; end
		default: begin two = 1'b0; ok = 1'b1; flt = 1'b0; bd = 32'd0; end
	endcase
	if (!ok) ;
	else if (flt) exc_stream_fault(two, 1'b0);
	else begin
		if (sub[5:4] == 2'b10) pop(2'd1);
		else if (sub[5:4] == 2'b11) pop(2'd2);
		if (sub[2:0] == 3'b000) begin
			ea_ready(tmp + bd + tmp2);
		end else begin
			// memory indirect: preindexed unless IS=1 or I/IS[2]=1 (postindexed)
			tmp <= tmp + bd + ((!sub[6] && !sub[2]) ? tmp2 : 32'd0);
			rd(tmp + bd + ((!sub[6] && !sub[2]) ? tmp2 : 32'd0), `SZ_L, DW_EA, S_EA_IND);
		end
	end
end

S_EA_IND: begin : ea_ind_blk
	// outer displacement per I/IS[1:0]; postindexing adds the index now
	reg [31:0] od;
	reg ok, flt, two;
	case (sub[1:0])
		2'b10: begin two = 1'b0; ok = w0_v; flt = w0_v & w0_f; od = sext16(w0); end
		2'b11: begin two = 1'b1; ok = have2; flt = pipe_faulted(1'b1); od = {w0, w1}; end
		default: begin two = 1'b0; ok = 1'b1; flt = 1'b0; od = 32'd0; end
	endcase
	if (!ok) ;
	else if (flt) exc_stream_fault(two, 1'b0);
	else begin
		if (sub[1:0] == 2'b10) pop(2'd1);
		else if (sub[1:0] == 2'b11) pop(2'd2);
		ea_ready(ea + od + ((!sub[6] && sub[2]) ? tmp2 : 32'd0));
	end
end

//-------------------------------------------------------------- generic path
S_GEN_EXEC: begin : gen_exec
	if (g_flags) sr[4:0] <= alu_f;
	if (g_dstk == DK_REG && g_wb) wreg(g_dreg, g_dreg[3] ? alu_r : merge(dst, alu_r, g_size));
	if (g_dstk == DK_EA && g_wb) begin
		finish;
		wr(ea, g_size, alu_r, S_FETCH);
	end else begin
		finish_dispatch;
	end
end
// (S_MOVE_DEA handled directly by dispatch / S_IMM / S_DWAIT)


//-------------------------------------------------------------- PEA
S_PEA: begin wreg(4'd15, rf_c - 32'd4); finish; wr(rf_c - 32'd4, `SZ_L, ea, S_FETCH); end

//-------------------------------------------------------------- Bcc / BRA / BSR
S_BCC: begin : bcc_state
	reg [31:0] target, ret;
	reg ok, flt, two;
	reg [1:0] npop;
	if (ir[7:0] == 8'h00) begin two = 1'b0; ok = w0_v; flt = w0_v & w0_f; target = scan_pc + sext16(w0); npop = 2'd1; end
	else if (ir[7:0] == 8'hFF) begin two = 1'b1; ok = have2; flt = pipe_faulted(1'b1); target = scan_pc + {w0, w1}; npop = 2'd2; end
	else begin two = 1'b0; ok = 1'b1; flt = 1'b0; target = scan_pc + sext8(ir[7:0]); npop = 2'd0; end
	ret = scan_pc + {29'd0, npop, 1'b0};
	if (!ok) ;
	else if (flt) exc_stream_fault(two, 1'b0);
	else if (ir[11:8] == 4'h1) begin
		// BSR: push the return address, then branch
		pop(npop);
		ea <= target;
		wreg(4'd15, rf_c - 32'd4);
		wr(rf_c - 32'd4, `SZ_L, ret, S_JMP);
	end else if (ir[11:8] == 4'h0 || cc_true(ir[11:8], sr[4:0])) begin
		pop(npop);
		go_pc(target);
	end else if (npop == 2'd0) begin
		finish_dispatch;
	end else begin
		pop(npop);
		finish;
	end
end

//-------------------------------------------------------------- JMP / JSR / RTS / RTR / RTD
// an odd target: the address error frame's PC is the instruction + 2 for
// JMP and the target for JSR (WinUAE gencpu, 68030 AE corpus v24)
S_JMP: begin
	go_pc(ea);
	// JMP: instruction + 2; the 68020+ index modes have already consumed
	// their extension words, so the end of the instruction + 2 for them
	if (ea[0] && ir[15:6] == 10'b0100_1110_11)
		exc_pc <= (ir[5:3] == 3'b110 || ir[5:0] == 6'b111011) ? scan_pc + 32'd2 : pc_i + 32'd2;
	if (ea[0] && ir[15:6] == 10'b0100_1110_10) exc_pc <= ea;             // JSR
end
S_JSR: begin
	wreg(4'd15, rf_c - 32'd4);
	wr(rf_c - 32'd4, `SZ_L, scan_pc, S_JMP);
end
S_RTS: rd(rf_c, `SZ_L, DW_TMP, S_RTS2);
S_RTS2: begin
	wreg(4'd15, rf_c + 32'd4);
	go_pc(tmp);
	if (tmp[0] && ir[2:0] == 3'd7) exc_pc <= pc_i + 32'd2;   // RTR: instruction + 2
end
S_RTR: rd(rf_c, `SZ_W, DW_TMP2, S_RTR2);
S_RTR2: begin
	sr[4:0] <= tmp2[4:0];
	wreg(4'd15, rf_c + 32'd2);
	rd(rf_c + 32'd2, `SZ_L, DW_TMP, S_RTS2);
end
S_RTD: begin
	// the displacement was popped with the opcode (ext)
	tmp2 <= sext16(ext);
	rd(rf_c, `SZ_L, DW_TMP, S_RTD2);
end
S_RTD2: begin wreg(4'd15, rf_c + 32'd4 + tmp2); go_pc(tmp); end

//-------------------------------------------------------------- DBcc / Scc / TRAPcc / TRAPV
S_DBCC: begin : dbcc_state
	// the displacement (ext) is relative to its own address, pc_i + 2
	reg [15:0] nw;
	nw = rf_a[15:0] - 16'd1;
	if (cc_true(ir[11:8], sr[4:0])) finish_dispatch;
	else begin
		wreg({1'b0, ir[2:0]}, {rf_a[31:16], nw});
		// the MC68020/030 prefetch from the branch target whenever the
		// condition is false, so an odd displacement is an address error
		// even when the count expires (WinUAE gencpu DBcc, 68030 AE corpus)
		if (nw != 16'hFFFF || ext[0]) begin
			go_pc(pc_i + 32'd2 + sext16(ext));
			if (ext[0]) exc_pc <= pc_i + 32'd2 + sext16(ext);   // the frame's PC is the target
		end
		else finish_dispatch;
	end
end
S_SCC: begin : scc_state
	reg [7:0] v;
	v = cc_true(ir[11:8], sr[4:0]) ? 8'hFF : 8'h00;
	if (ir[5:3] == 3'b000) begin wreg({1'b0, ir[2:0]}, {dst[31:8], v}); finish; end
	else begin finish; wr(ea, `SZ_B, {24'd0, v}, S_FETCH); end
end
S_TRAPCC: begin : trapcc_state
	reg [1:0] npop;
	reg ok, flt, two;
	if (ir[15:12] == 4'h4) begin npop = 2'd0; ok = 1'b1; flt = 1'b0; two = 1'b0; end       // TRAPV
	else if (ir[2:0] == 3'b010) begin npop = 2'd1; ok = w0_v; flt = w0_v & w0_f; two = 1'b0; end
	else if (ir[2:0] == 3'b011) begin npop = 2'd2; ok = have2; flt = pipe_faulted(1'b1); two = 1'b1; end
	else begin npop = 2'd0; ok = 1'b1; flt = 1'b0; two = 1'b0; end
	if (!ok) ;
	else if (flt) exc_stream_fault(two, 1'b0);
	else begin
		pop(npop);
		if (cc_true((ir[15:12] == 4'h4) ? 4'h9 : ir[11:8], sr[4:0]))
			exc_go(`VEC_TRAPCC, `FMT_SIXWORD, scan_pc + {29'd0, npop, 1'b0}, pc_i);
		else finish;
	end
end

//-------------------------------------------------------------- LINK / UNLK
S_LINK: begin : link_state
	// LINK.W: the displacement is ext; LINK.L: ext is its high word, the low word follows
	reg two;
	reg [31:0] disp;
	two = (g_size == `SZ_L);
	disp = two ? {ext, w0} : sext16(ext);
	if (two && !w0_v) ; else if (two && w0_f) exc_stream_fault(1'b0, 1'b0);
	else begin
		if (two) pop(2'd1);
		ea <= disp;
		tmp <= rf_c - 32'd4;
		// LINK A7 pushes the initial A7 on the 68020/030 (only the 68040
		// pushes the decremented value; WinUAE gencpu and 68030 cputest)
		wr(rf_c - 32'd4, `SZ_L, rf_a, S_LINK2);
	end
end
S_LINK2: begin
	if (ir[2:0] != 3'd7) begin
		wreg({1'b1, ir[2:0]}, tmp);
		sp_we <= 1'b1; sp_sel <= !sr_s ? 2'd0 : (sr_m ? 2'd2 : 2'd1); sp_wdata <= tmp + ea;
	end else begin
		wreg(4'd15, tmp + ea);
	end
	finish;
end
S_UNLK: begin tmp <= rf_a; dw_reg <= {1'b1, ir[2:0]}; rd(rf_a, `SZ_L, DW_REGL, S_UNLK2); end
S_UNLK2: begin if (ir[2:0] != 3'd7) wreg(4'd15, tmp + 32'd4); finish; end

//-------------------------------------------------------------- MOVE USP / from SR,CCR / to SR,CCR
S_MOVE_USP: begin
	if (ir[3]) wreg({1'b1, ir[2:0]}, usp_q);
	else begin sp_we <= 1'b1; sp_sel <= 2'd0; sp_wdata <= rf_a; end
	finish;
end
S_MOVE_FSR: begin : movefsr
	reg [15:0] v;
	v = (ir[11:9] == 3'b000) ? sr : {8'd0, sr[7:0]};
	if (ir[5:3] == 3'b000) begin wreg({1'b0, ir[2:0]}, {dst[31:16], v}); finish; end
	else begin finish; wr(ea, `SZ_W, {16'd0, v}, S_FETCH); end
end
S_MOVE_SR: begin : movesr
	reg [15:0] nsr;
	reg is_sr;
	is_sr = (ir[15:12] == 4'h4) ? ir[9] : ir[6];        // MOVE to SR ($46C0) / ORI etc. with word size
	case (g_alu)
		`ALU_OR:  nsr = sr | src[15:0];
		`ALU_AND: nsr = sr & src[15:0];
		`ALU_EOR: nsr = sr ^ src[15:0];
		default:  nsr = src[15:0];
	endcase
	// T0 traces ORI/ANDI/EORI to CCR and SR and MOVE to SR, whatever
	// they change; MOVE to CCR is not traced by T0 (WinUAE gencpu
	// check_trace, 68020/68030; 68030 corpus v24)
	if (!is_sr) begin
		sr[4:0] <= nsr[4:0];
		finish;
		if (ir[15:12] != 4'h4) trace_pend <= tr_t1 | tr_t0;
	end else begin
		sr <= nsr & `SR_MASK;
		// the program space may have changed: refill the pipe (UM 12.7.1)
		flush_req = 1'b1; flush_pc = scan_pc;
		trace_pend <= tr_t1 | tr_t0;
		state <= S_FETCH;
	end
end

//-------------------------------------------------------------- MOVEC (PRM)
S_MOVEC: begin : movec_state
	reg [32:0] cr;
	cr = ctrl_rd(ext[11:0]);
	if (cr[32]) go_illegal;
	else if (!ir[0]) begin
		wreg({ext[15], ext[14:12]}, cr[31:0]);
		finish;
	end else begin
		ctrl_wr(ext[11:0], rf_a);
		finish;
	end
end

//-------------------------------------------------------------- MOVES (PRM)
S_MOVES0: begin
	if (ext[11]) begin
		// register to <ea> in DFC space
		finish;
		dreq(ea, g_size, 1'b0, rf_a, dfc, 1'b0, 1'b0, DW_NONE, S_FETCH);
	end else begin
		dst <= rf_a;
		dw_reg <= {ext[15], ext[14:12]};
		dreq(ea, g_size, 1'b1, 32'd0, sfc, 1'b0, 1'b0, ext[15] ? DW_REGL : DW_REG, S_MOVES1);
	end
end
S_MOVES1: finish_dispatch;


//-------------------------------------------------------------- NOP / STOP / RESET / TRAP / BKPT
S_NOP: begin
	if (bus_quiet) finish;      // UM 7.6: NOP synchronizes with pending bus cycles
end
S_STOP: begin
	// the immediate SR value was popped with the opcode (ext)
	sr <= ext & `SR_MASK;
	// a traced STOP does not stop (UM 8.1.7)
	if (tr_t1 || (tr_t0 && ({ext[15:12], ext[10:8]} != {sr[15:12], sr[10:8]}))) trace_pend <= 1'b1;
	else stopped <= 1'b1;
	state <= S_FETCH;
end
S_RESETI: begin
	// UM 7.8: RESET drives the pin for 512 clocks; the CPU's own state is
	// untouched.  The bus is idle for the whole pulse (Figure 7-65: the
	// external devices are being reset): prefetching stops at once, and the
	// pin is driven once the fetches already requested have returned and
	// the posted writes have completed (bus_quiet); fetching resumes after.
	fetch_hold <= 1'b1;
	if (!reset_drive) begin
		if (fetch_hold && fetch_out == 2'd0 && bus_quiet) begin reset_drive <= 1'b1; rst_cnt <= 10'd511; end
	end else if (rst_cnt != 10'd0) rst_cnt <= rst_cnt - 10'd1;
	else begin reset_drive <= 1'b0; fetch_hold <= 1'b0; finish; end
end
S_TRAP: exc_go(`VEC_TRAP + {4'd0, ir[3:0]}, `FMT_NORMAL, scan_pc, 32'd0);
S_BKPT: begin
	// breakpoint acknowledge cycle (UM 7.4.2): CPU space type 0, number on A2-A4
	cpu_flt_ill <= 1'b1;
	dreq({27'd0, ir[2:0], 2'b00}, `SZ_W, 1'b1, 32'd0, `FC_CPU_SPACE, 1'b0, 1'b0, DW_TMP, S_BKPT2);
end
S_BKPT2: begin
	// the returned word replaces the BKPT in the pipe and executes
	cpu_flt_ill <= 1'b0;
	pipe_load = 1'b1; flush_pc = pc_i;
	pipe_c = tmp[15:0]; pipe_c_v = 1'b1;
	pipe_b = w0; pipe_b_v = w0_v & !w0_f;
	state <= S_FETCH;
end

//-------------------------------------------------------------- EXG
S_EXG: begin tmp <= rf_a; wreg(ra_a, rf_b); state <= S_EXG2; end
S_EXG2: begin wreg(ra_b, tmp); finish; end

//-------------------------------------------------------------- -(Ay),-(Ax) / (Ay)+,(Ax)+ forms and PACK/UNPK
// ABCD/SBCD/ADDX/SUBX -(Ay),-(Ax); CMPM (Ay)+,(Ax)+; PACK/UNPK -(Ay),-(Ax) or Dx,Dy
S_PACK: begin : pack_state
	reg [31:0] bs;       // source operand size in bytes
	reg is_pack, is_unpk, is_cmpm;
	is_pack = (ir[15:12] == 4'h8) && (ir[8:6] == 3'b101);
	is_unpk = (ir[15:12] == 4'h8) && (ir[8:6] == 3'b110);
	is_cmpm = (ir[15:12] == 4'hB);
	bs = is_pack ? 32'd2 : (is_unpk ? ((ir[2:0] == 3'd7) ? 32'd2 : 32'd1) : size_bytes(g_size, ir[2:0] == 3'd7));
	if (!ir[3]) begin
		// register form (PACK/UNPK Dx,Dy)
		src <= rf_a; dst <= rf_b; state <= S_PACK3;
	end else if (is_cmpm) begin
		wreg({1'b1, ir[2:0]}, rf_a + bs);
		rd(rf_a, g_size, DW_SRC, S_PACK2);
	end else begin
		wreg({1'b1, ir[2:0]}, rf_a - bs);
		rd(rf_a - bs, is_pack ? `SZ_W : (is_unpk ? `SZ_B : g_size), DW_SRC, S_PACK2);
	end
end
S_PACK2: begin : pack2_state
	reg [31:0] bd;
	reg is_pack, is_unpk, is_cmpm;
	is_pack = (ir[15:12] == 4'h8) && (ir[8:6] == 3'b101);
	is_unpk = (ir[15:12] == 4'h8) && (ir[8:6] == 3'b110);
	is_cmpm = (ir[15:12] == 4'hB);
	bd = is_pack ? ((ir[11:9] == 3'd7) ? 32'd2 : 32'd1) : (is_unpk ? 32'd2 : size_bytes(g_size, ir[11:9] == 3'd7));
	if (is_cmpm) begin
		wreg({1'b1, ir[11:9]}, rf_b + bd);
		ea <= rf_b;
		rd(rf_b, g_size, DW_DST, S_PACK3);
	end else if (is_pack || is_unpk) begin
		// PACK/UNPK do not read the destination
		wreg({1'b1, ir[11:9]}, rf_b - bd);
		ea <= rf_b - bd;
		state <= S_PACK3;
	end else begin
		wreg({1'b1, ir[11:9]}, rf_b - bd);
		ea <= rf_b - bd;
		rd(rf_b - bd, g_size, DW_DST, S_PACK3);
	end
end
S_PACK3: begin : pack3_state
	reg is_pack, is_unpk, is_cmpm;
	reg [15:0] adj;
	reg [31:0] r;
	is_pack = (ir[15:12] == 4'h8) && (ir[8:6] == 3'b101);
	is_unpk = (ir[15:12] == 4'h8) && (ir[8:6] == 3'b110);
	is_cmpm = (ir[15:12] == 4'hB);
	adj = src[15:0] + ext;
	if (is_pack) begin
		r = {24'd0, adj[11:8], adj[3:0]};
		if (!ir[3]) begin wreg({1'b0, ir[11:9]}, {dst[31:8], r[7:0]}); finish; end
		else begin finish; wr(ea, `SZ_B, r, S_FETCH); end
	end else if (is_unpk) begin
		r = {16'd0, ({4'd0, src[7:4], 4'd0, src[3:0]} + ext)};
		if (!ir[3]) begin wreg({1'b0, ir[11:9]}, {dst[31:16], r[15:0]}); finish; end
		else begin finish; wr(ea, `SZ_W, r, S_FETCH); end
	end else begin
		sr[4:0] <= alu_f;
		if (is_cmpm) finish;
		else begin finish; wr(ea, g_size, alu_r, S_FETCH); end
	end
end

//-------------------------------------------------------------- TAS (read-modify-write, UM 7.3.3)
S_TAS: dreq(ea, `SZ_B, 1'b1, 32'd0, fc_data, 1'b1, 1'b0, DW_DST, S_TAS2);
S_TAS2: begin
	sr[4:0] <= alu_f;
	finish;
	dreq(ea, `SZ_B, 1'b0, alu_r, fc_data, 1'b1, 1'b1, DW_NONE, S_FETCH);
end

//-------------------------------------------------------------- MOVEP
S_MOVEP0: begin
	ea <= rf_a + sext16(ext);
	cnt <= 8'd0; tmp <= 32'd0; dst <= rf_b;
	state <= S_MOVEP1;
end
S_MOVEP1: begin : movep1
	reg [2:0] n;
	reg [7:0] wb;
	n = ir[6] ? 3'd4 : 3'd2;
	case ({ir[6] & ~cnt[1], ~cnt[0]})
		2'd3: wb = dst[31:24];
		2'd2: wb = dst[23:16];
		2'd1: wb = dst[15:8];
		default: wb = dst[7:0];
	endcase
	if (cnt[2:0] == n) begin
		if (!ir[7]) wreg({1'b0, ir[11:9]}, ir[6] ? tmp : {dst[31:16], tmp[15:0]});
		finish;
	end else if (ir[7]) begin
		cnt <= cnt + 8'd1;
		wr(ea + {cnt[6:0], 1'b0}, `SZ_B, {24'd0, wb}, S_MOVEP1);
	end else begin
		cnt <= cnt + 8'd1;
		rd(ea + {cnt[6:0], 1'b0}, `SZ_B, DW_TMP2, S_MOVEP2);
	end
end
S_MOVEP2: begin tmp <= {tmp[23:0], tmp2[7:0]}; state <= S_MOVEP1; end

//-------------------------------------------------------------- CHK (PRM)
S_CHK: begin : chk_state
	reg signed [31:0] v, b;
	reg [31:0] d;
	reg [3:0] nf;
	reg out_of;
	v = sext_sz(dst, g_size);
	b = sext_sz(src, g_size);
	d = b - v;
	out_of = (v < 0) || (v > b);
	// N, Z, V and C are undefined; the MC68020/030 values on both paths
	// (WinUAE setchkundefinedflags): V is the overflow of bound - Dn, C
	// follows the signs, both are clear when Dn is in bounds
	nf[3] = (v < 0);
	nf[2] = (v == 0);
	nf[1] = out_of && ((v[31] ^ b[31]) & ((g_size == `SZ_W ? d[15] : d[31]) ^ b[31]));
	nf[0] = out_of && ((v < 0) ? ((v > b) || (b >= 0)) : (b >= 0));
	sr[3:0] <= nf;
	if (out_of) begin
		exc_go(`VEC_CHK, `FMT_SIXWORD, scan_pc, pc_i);
		exc_sr <= {sr[15:4], nf};       // the frame holds the updated flags
	end else finish;
end
S_DIVZ: exc_go(`VEC_DIVZERO, `FMT_SIXWORD, scan_pc, pc_i);
