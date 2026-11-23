// AP68030 core - states part C: coprocessor interface (UM Section 10) and
// MMU instructions (UM 9.8)
//
// cp_kind: 0 cpGEN, 1 cpBcc, 2 cpDBcc, 3 cpScc, 4 cpTRAPcc
// cp_base: CIR set base (CPU space type 2, CpID on A13-A15)

//-------------------------------------------------------------- instruction start
S_CP0: begin
	// cpGEN: the command word to the command CIR ($0A)
	cp_id <= ir[11:9];
	cp_kind <= 4'd0; cp_cond <= 1'b0; cp_pcbit <= 1'b0; cp_trace_wait <= 1'b0;
	cpu_flt_fline <= 1'b1;
	dreq({14'd0, 1'b1, 1'b0, ir[11:9], 13'd0} | 32'h0A, `SZ_W, 1'b0, {16'd0, ext}, `FC_CPU_SPACE, 1'b0, 1'b0, DW_NONE, S_CP1);
end
S_CPBCC: begin
	// cpBcc: the operation word to the condition CIR ($0E)
	cp_id <= ir[11:9];
	cp_kind <= 4'd1; cp_cond <= 1'b1; cp_pcbit <= 1'b0; cp_trace_wait <= 1'b0;
	cpu_flt_fline <= 1'b1;
	dreq({14'd0, 1'b1, 1'b0, ir[11:9], 13'd0} | 32'h0E, `SZ_W, 1'b0, {16'd0, ir}, `FC_CPU_SPACE, 1'b0, 1'b0, DW_NONE, S_CP1);
end
S_CPDBCC, S_CPSCC, S_CPTRAP: begin
	// the condition selector word (second word) to the condition CIR
	cp_id <= ir[11:9];
	cp_kind <= (state == S_CPDBCC) ? 4'd2 : (state == S_CPSCC) ? 4'd3 : 4'd4;
	cp_cond <= 1'b1; cp_pcbit <= 1'b0; cp_trace_wait <= 1'b0;
	cpu_flt_fline <= 1'b1;
	dreq({14'd0, 1'b1, 1'b0, ir[11:9], 13'd0} | 32'h0E, `SZ_W, 1'b0, {16'd0, ext}, `FC_CPU_SPACE, 1'b0, 1'b0, DW_NONE, S_CP1);
end

//-------------------------------------------------------------- response dialogue
S_CP1: begin
	cpu_flt_fline <= 1'b0;
	cir_rd(5'h00, `SZ_W, DW_TMP, S_CP2);
end
S_CP2: begin : cp2
	reg [15:0] r;
	r = tmp[15:0];
	cp_resp <= r;
	cp_ca <= r[15]; cp_dr <= r[13];
	if (r[14] && !cp_pcbit) begin
		// PC bit: the instruction address to the instruction address CIR first
		cp_pcbit <= 1'b1;
		cir_wr(5'h18, `SZ_L, pc_i, S_CP2);
	end else begin
		cp_pcbit <= 1'b0;
		casez (r[12:8])
			5'b00000: begin
				// write to previously evaluated EA (DR=1; bits 13:8 = $00 are
				// undefined, UM 10.6)
				if (cp_cond || !r[13]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else begin cp_len <= r[7:0]; cp_pos <= 8'd0; cp_dr <= 1'b1; state <= S_CPXFER; end
			end
			5'b00001: begin
				// transfer multiple coprocessor registers
				if (cp_cond || r[0]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else if (!(r[13] ? (ir_ea_ctrlalt || ir_ea_pd) : (ir_ea_ctrl || ir_ea_pi))) begin
					tmp <= {24'd0, `VEC_FLINE}; cir_wr(5'h02, `SZ_W, 32'h0001, S_CP_ABORT);
				end else begin cp_len <= r[7:0]; state <= S_CPMULT; end
			end
			5'b00100: begin
				if (r[13]) begin
					// busy: service interrupts with a pre-instruction frame, restart
					if (irq_pend) begin iack_pc_i <= 1'b1; exc_ilvl <= irq_lvl; state <= S_IACK; end
					else begin flush_req = 1'b1; flush_pc = pc_i; state <= S_FETCH; end
				end else begin
					// supervisor check
					if (cp_cond && !r[15]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
					else if (!sr_s) begin tmp <= {24'd0, `VEC_PRIV}; cir_wr(5'h02, `SZ_W, 32'h0001, S_CP_ABORT); end
					else state <= S_CP1;
				end
			end
			5'b00101: begin
				// take address and transfer data: the address goes to tmp2, ea
				// keeps the evaluated EA for a write to previously evaluated EA
				// and the mid-instruction frame (UM 10.4.10)
				if (cp_cond && !r[15]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else begin cp_len <= r[7:0]; cp_pos <= 8'd0; cir_rd(5'h1C, `SZ_L, DW_TMP2, S_CPXFER); end
			end
			5'b00110: begin
				// transfer multiple main processor registers
				if (cp_cond && !r[15]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else cir_rd(5'h14, `SZ_W, DW_TMP, S_CPREGS);
			end
			5'b00111: begin
				// transfer operation word
				if (cp_cond && !r[15]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else cir_wr(5'h08, `SZ_W, {16'd0, ir}, S_CPWAIT);
			end
			5'b0100?: begin
				// null (DR=0; $28/$29 are undefined, UM 10.6)
				if (r[13]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else state <= S_CPNULL;
			end
			5'b01010: begin
				// evaluate and transfer effective address (DR=0; $2A is undefined, UM 10.6)
				if (cp_cond || r[13]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else if (!ir_ea_ctrlalt) begin tmp <= {24'd0, `VEC_FLINE}; cir_wr(5'h02, `SZ_W, 32'h0001, S_CP_ABORT); end
				else begin ea_ret <= S_CPEAX; state <= S_EA; end
			end
			5'b01100: begin
				// transfer single main processor register (D/A in bit 3, number in 2:0)
				if (cp_cond && !r[15]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else state <= S_CPREG;
			end
			5'b01101: begin
				// transfer main processor control register
				if (cp_cond && !r[15]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else cir_rd(5'h14, `SZ_W, DW_TMP, S_CPCTRL);
			end
			5'b01110: begin
				// transfer to/from top of stack
				if ((cp_cond && !r[15]) || !(r[7:0] == 8'd1 || r[7:0] == 8'd2 || r[7:0] == 8'd4))
					exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else begin cp_len <= r[7:0]; state <= S_CPTOS; end
			end
			5'b01111: begin
				// transfer from instruction stream
				if ((cp_cond && !r[15]) || r[0]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else begin cp_len <= r[7:0]; cp_pos <= 8'd0; state <= S_CP_ISTREAM; end
			end
			5'b0001?: begin
				// transfer status register and scanPC
				if (cp_cond) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else state <= S_CPSR;
			end
			5'b10???: begin
				// evaluate effective address and transfer data: an EA outside
				// the primitive's category is an F-line (abort); a write to a
				// non-alterable EA, a register length other than 1, 2 or 4 and
				// an odd immediate length above 1 are protocol violations even
				// when the category matches (UM 10.4.9, Table 10-6)
				if (cp_cond) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else if (!cp_ea_ok(r[10:8])) begin
					tmp <= {24'd0, `VEC_FLINE}; cir_wr(5'h02, `SZ_W, 32'h0001, S_CP_ABORT);
				end else if ((r[13] && !ir_ea_alt) ||
				             ((ir_ea_dn || ir_ea_an) && !(r[7:0] == 8'd1 || r[7:0] == 8'd2 || r[7:0] == 8'd4)) ||
				             (ir_ea_imm && r[0] && r[7:0] != 8'd1))
					exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
				else begin cp_len <= r[7:0]; cp_pos <= 8'd0; state <= S_CPXEA; end
			end
			5'b11100: begin tmp <= {24'd0, r[7:0]}; cp_kind[3] <= 1'b0; cir_wr(5'h02, `SZ_W, 32'h0002, S_CP_EXC); sub <= 8'd0; end
			5'b11101: begin tmp <= {24'd0, r[7:0]}; cir_wr(5'h02, `SZ_W, 32'h0002, S_CP_EXC); sub <= 8'd1; end
			5'b11110: begin tmp <= {24'd0, r[7:0]}; cir_wr(5'h02, `SZ_W, 32'h0002, S_CP_EXC); sub <= 8'd2; end
			default: exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
		endcase
	end
end
// after a service: come again, or the instruction is done
S_CPWAIT: begin
	if (cp_ca) state <= S_CP1;
	else if (cp_cond) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);   // conditional needs a null to finish
	// a pending trace: the dialog goes on until null CA=0 PF=1 or a take
	// post-instruction exception (UM 10.5.2.5)
	else if (tr_t1 || cp_trace_wait) state <= S_CP1;
	else finish;
end
S_CP_ABORT: begin
	// the instruction was aborted with $0001 in the control CIR; the caller
	// left the vector (F-line, privilege violation or format error) in tmp
	exc_pre(tmp[7:0]);
end
S_CP_EXC: begin
	// take pre/mid/post-instruction exception (acknowledged with $0002)
	case (sub[1:0])
		2'd0: exc_go(tmp[7:0], `FMT_NORMAL, pc_i, 32'd0);
		2'd1: exc_go(tmp[7:0], `FMT_CPMID, scan_pc, pc_i);
		default: exc_go(tmp[7:0], `FMT_SIXWORD, scan_pc, pc_i);
	endcase
end
S_CPNULL: begin
	// null primitive (UM Table 10-3)
	if (cp_resp[15]) begin
		// come again; IA allows interrupts, serviced with a mid-instruction frame
		if (cp_resp[8] && irq_pend) begin iack_cpmid <= 1'b1; exc_ilvl <= irq_lvl; state <= S_IACK; end
		else state <= S_CP1;
	end else if (cp_cond) begin
		// TF completes the conditional instruction
		case (cp_kind)
			4'd1: state <= S_CPBCC2;
			4'd2: state <= S_CPDBCC2;
			4'd3: state <= S_CPSCC2;
			default: state <= S_CPTRAP2;
		endcase
	end else if ((tr_t1 || cp_trace_wait) && !cp_resp[1]) begin
		// a pending trace waits for processing finished (UM 10.5.2.5)
		if (cp_resp[8] && irq_pend) begin iack_cpmid <= 1'b1; exc_ilvl <= irq_lvl; state <= S_IACK; end
		else state <= S_CP1;
	end else begin
		finish;
		// trace on change of flow: a transfer of SR and scanPC into the
		// processor made the trace pending (UM 10.4.17, 10.5.2.5)
		if (cp_trace_wait) trace_pend <= 1'b1;
	end
end

//-------------------------------------------------------------- conditional completions
S_CPBCC2: begin : cpbcc2
	reg two, ok, flt;
	reg [31:0] target;
	two = ir[6];
	ok = pipe_ready(two); flt = pipe_faulted(two);
	target = two ? (scan_pc + {w0, w1}) : (scan_pc + sext16(w0));
	if (!ok) ; else if (flt) exc_stream_fault(two, 1'b0);
	else begin
		pop(two ? 2'd2 : 2'd1);
		if (cp_resp[0]) go_pc(target); else finish;
	end
end
S_CPDBCC2: begin : cpdbcc2
	reg [15:0] nw;
	nw = rf_a[15:0] - 16'd1;
	if (!w0_v) ; else if (w0_f) exc_stream_fault(1'b0, 1'b0);
	else if (cp_resp[0]) begin pop(2'd1); finish; end
	else begin
		wreg({1'b0, ir[2:0]}, {rf_a[31:16], nw});
		pop(2'd1);
		if (nw != 16'hFFFF) go_pc(scan_pc + sext16(w0));
		else finish;
	end
end
S_CPSCC2: begin
	// Dn: only its low byte changes (UM 10.2.2.2); the upper bytes are read
	// now, not taken from dst: the dialog may have loaded Dn (transfer main
	// processor register), and after a mid-instruction frame's RTE dst
	// holds whatever the handler left there
	if (ir[5:3] == 3'b000) begin wreg({1'b0, ir[2:0]}, {rf_a[31:8], {8{cp_resp[0]}}}); finish; end
	else begin ea_ret <= S_CPSCC_WR; state <= S_EA; end
end
S_CPSCC_WR: begin finish; wr(ea, `SZ_B, {24'd0, {8{cp_resp[0]}}}, S_FETCH); end   // cpScc memory destination
S_CPTRAP2: begin : cptrapcc
	// cpTRAPcc completion: skip the operand words, trap when true
	reg [1:0] npop;
	reg ok, flt, two;
	if (ir[2:0] == 3'b010) begin npop = 2'd1; ok = w0_v; flt = w0_v & w0_f; two = 1'b0; end
	else if (ir[2:0] == 3'b011) begin npop = 2'd2; ok = have2; flt = pipe_faulted(1'b1); two = 1'b1; end
	else begin npop = 2'd0; ok = 1'b1; flt = 1'b0; two = 1'b0; end
	if (!ok) ; else if (flt) exc_stream_fault(two, 1'b0);
	else begin
		pop(npop);
		if (cp_resp[0]) exc_go(`VEC_TRAPCC, `FMT_SIXWORD, scan_pc + {29'd0, npop, 1'b0}, pc_i);
		else finish;
	end
end

//-------------------------------------------------------------- primitives with transfers
S_CPEAX: cir_wr(5'h1C, `SZ_L, ea, S_CPWAIT);                    // evaluate and transfer EA
S_CP_ISTREAM: begin : cpistream
	// operand words from the instruction stream to the operand CIR
	reg [7:0] rem;
	rem = cp_len - cp_pos;
	if (rem == 8'd0) state <= S_CPWAIT;
	else if (rem >= 8'd4) begin
		if (!have2) ; else if (pipe_faulted(1'b1)) exc_stream_fault(1'b1, 1'b0);
		else begin pop(2'd2); cp_pos <= cp_pos + 8'd4; cir_wr(5'h10, `SZ_L, {w0, w1}, S_CP_ISTREAM); end
	end else begin
		if (!w0_v) ; else if (w0_f) exc_stream_fault(1'b0, 1'b0);
		else begin pop(2'd1); cp_pos <= cp_pos + 8'd2; cir_wr(5'h10, `SZ_W, {16'd0, w0}, S_CP_ISTREAM); end
	end
end
S_CPXEA: begin : cpxea
	// evaluate EA and transfer data: register direct, immediate, or memory
	reg [31:0] bytes;
	bytes = {24'd0, cp_len};
	if (ir_ea_dn || ir_ea_an) begin
		dw_reg <= {ir_ea_an, ir[2:0]};
		if (!cp_dr) cir_wr(5'h10, sz_of_bytes(cp_len[2:0]), ir_ea_an ? rf_b : rf_c, S_CPWAIT);
		else begin dst <= ir_ea_an ? rf_b : rf_c; cir_rd(5'h10, sz_of_bytes(cp_len[2:0]), ir_ea_an ? DW_REGL : DW_REG, S_CPWAIT); end
	end else if (ir_ea_imm) begin
		// immediate: from the instruction stream (byte operands take a word)
		cp_pos <= 8'd0;
		if (cp_len == 8'd1) begin
			if (!w0_v) ; else if (w0_f) exc_stream_fault(1'b0, 1'b0);
			else begin pop(2'd1); cir_wr(5'h10, `SZ_B, {24'd0, w0[7:0]}, S_CPWAIT); end
		end else state <= S_CP_ISTREAM;
	end else if (ir_ea_pi) begin
		ea <= rf_b;
		wreg({1'b1, ir[2:0]}, rf_b + ((cp_len == 8'd1 && ir[2:0] == 3'd7) ? 32'd2 : bytes));
		state <= S_CPXFER;
	end else if (ir_ea_pd) begin
		ea <= rf_b - ((cp_len == 8'd1 && ir[2:0] == 3'd7) ? 32'd2 : bytes);
		wreg({1'b1, ir[2:0]}, rf_b - ((cp_len == 8'd1 && ir[2:0] == 3'd7) ? 32'd2 : bytes));
		state <= S_CPXFER;
	end else begin
		ea_ret <= S_CPXFER; state <= S_EA;
	end
end
S_CPXFER: begin : cpxfer
	// memory <-> operand CIR in longword parts, ascending (UM 10.3.8).  The
	// take address and transfer data primitive's address is in tmp2 and its
	// operand in data space (UM 10.4.11); the evaluated EA's reads are
	// program references when the EA is PC-relative
	reg [7:0] rem;
	reg [2:0] chunk;
	rem = cp_len - cp_pos;
	chunk = (rem >= 8'd4) ? 3'd4 : rem[2:0];
	if (rem == 8'd0) state <= S_CPWAIT;
	else if (cp_dr) cir_rd(5'h10, sz_of_bytes(chunk), DW_TMP, S_CPXFER3);
	else if (cp_resp[12:8] == 5'b00101)
		dreq(tmp2 + {24'd0, cp_pos}, sz_of_bytes(chunk), 1'b1, 32'd0, fc_data, 1'b0, 1'b0, DW_TMP, S_CPXFER2);
	else rd(ea + {24'd0, cp_pos}, sz_of_bytes(chunk), DW_TMP, S_CPXFER2);
end
S_CPXFER2: begin : cpxfer2
	reg [7:0] rem; reg [2:0] chunk;
	rem = cp_len - cp_pos; chunk = (rem >= 8'd4) ? 3'd4 : rem[2:0];
	cp_pos <= cp_pos + {5'd0, chunk};
	cir_wr(5'h10, sz_of_bytes(chunk), tmp, S_CPXFER);
end
S_CPXFER3: begin : cpxfer3
	reg [7:0] rem; reg [2:0] chunk;
	rem = cp_len - cp_pos; chunk = (rem >= 8'd4) ? 3'd4 : rem[2:0];
	cp_pos <= cp_pos + {5'd0, chunk};
	wr(((cp_resp[12:8] == 5'b00101) ? tmp2 : ea) + {24'd0, cp_pos}, sz_of_bytes(chunk), tmp, S_CPXFER);
end
S_CPTOS: begin
	// transfer to/from top of stack: (A7)+ to the coprocessor, or -(A7) from
	// it (UM 10.4.12), in data space; the -(A7) address goes to tmp2, ea
	// keeps the evaluated EA (UM 10.4.10)
	if (!cp_dr) begin
		wreg(4'd15, rf_c + ((cp_len == 8'd1) ? 32'd2 : {24'd0, cp_len}));
		dreq(rf_c, sz_of_bytes(cp_len[2:0]), 1'b1, 32'd0, fc_data, 1'b0, 1'b0, DW_TMP, S_CPTOS2);
	end else begin
		tmp2 <= rf_c - ((cp_len == 8'd1) ? 32'd2 : {24'd0, cp_len});
		wreg(4'd15, rf_c - ((cp_len == 8'd1) ? 32'd2 : {24'd0, cp_len}));
		cir_rd(5'h10, sz_of_bytes(cp_len[2:0]), DW_TMP, S_CPSR3);
	end
end
S_CPTOS2: cir_wr(5'h10, sz_of_bytes(cp_len[2:0]), tmp, S_CPWAIT);
S_CPSR3: begin wr(tmp2, sz_of_bytes(cp_len[2:0]), tmp, S_CPWAIT); end
S_CPREG: begin
	// transfer single main processor register
	dw_reg <= {cp_resp[3], cp_resp[2:0]};
	if (!cp_dr) cir_wr(5'h10, `SZ_L, rf_a, S_CPWAIT);
	else cir_rd(5'h10, `SZ_L, DW_REGL, S_CPWAIT);
end
S_CPCTRL: begin : cpctrl
	// control register select code from the register select CIR (UM Table 10-5)
	reg [32:0] cr;
	cr = ctrl_rd(tmp[11:0]);
	if (cr[32]) exc_go(`VEC_CPPROTO, `FMT_CPMID, scan_pc, pc_i);
	else if (!cp_dr) cir_wr(5'h10, `SZ_L, cr[31:0], S_CPWAIT);
	else cir_rd(5'h10, `SZ_L, DW_TMP2, S_CPCTRL2);
end
S_CPCTRL2: begin
	ctrl_wr(tmp[11:0], tmp2);
	state <= S_CPWAIT;
end
S_CPREGS: begin
	// transfer multiple main processor registers: mask D0..D7,A0..A7 (bit 0 = D0)
	mm_mask <= tmp[15:0];
	state <= S_CPREGS2;
end
S_CPREGS2: begin : cpregs2
	reg [3:0] r;
	r = mm_next(mm_mask, 1'b0);
	if (mm_mask == 16'd0) state <= S_CPWAIT;
	else begin
		cnt <= {4'd0, r};
		mm_mask[r] <= 1'b0;
		dw_reg <= r;
		// the register value arrives through port A next clock (cnt): use a
		// two-step: first select, then transfer
		state <= S_CP9;
	end
end
S_CP9: begin
	if (!cp_dr) cir_wr(5'h10, `SZ_L, rf_a, S_CPREGS2);
	else cir_rd(5'h10, `SZ_L, DW_REGL, S_CPREGS2);
end
S_CPMULT: begin
	// transfer multiple coprocessor registers: EA, then the register mask
	if (ir_ea_pi) begin ea <= rf_b; state <= S_CPMULT2; end
	else if (ir_ea_pd) begin ea <= rf_b; state <= S_CPMULT2; end
	else begin ea_ret <= S_CPMULT2; state <= S_EA; end
end
S_CPMULT2: cir_rd(5'h14, `SZ_W, DW_TMP, S_CPMULT3);
S_CPMULT3: begin : cpmult3
	// cnt = operands left; each is cp_len bytes moved in longword parts
	reg [4:0] n;
	reg [11:0] tb;
	n = popcount16(tmp[15:0]);
	tb = (n[0] ? {4'd0, cp_len} : 12'd0) +
	     (n[1] ? {3'd0, cp_len, 1'b0} : 12'd0) +
	     (n[2] ? {2'd0, cp_len, 2'b00} : 12'd0) +
	     (n[3] ? {1'd0, cp_len, 3'b000} : 12'd0) +
	     (n[4] ? {cp_len, 4'b0000} : 12'd0);
	cnt <= {3'd0, n};
	cp_pos <= 8'd0;
	if (ir_ea_pi) wreg({1'b1, ir[2:0]}, rf_b + {20'd0, tb});
	if (ir_ea_pd) begin
		wreg({1'b1, ir[2:0]}, rf_b - {20'd0, tb});
		ea <= rf_b - {24'd0, cp_len};        // first operand at An - length, descending
	end

	state <= S_CPMULT4;
end
S_CPMULT4: begin : cpmult4
	reg [7:0] rem; reg [2:0] chunk;
	rem = cp_len - cp_pos;
	chunk = (rem >= 8'd4) ? 3'd4 : rem[2:0];
	if (cnt == 8'd0) state <= S_CPWAIT;
	else if (rem == 8'd0) begin
		// next operand
		cnt <= cnt - 8'd1;
		cp_pos <= 8'd0;
		if (cnt != 8'd1) ea <= ir_ea_pd ? (ea - {24'd0, cp_len}) : (ea + {24'd0, cp_len});
	end else if (!cp_dr) begin
		cp_pos <= cp_pos + {5'd0, chunk};
		tmp3 <= {14'd0, chunk};
		rd(ea + {24'd0, cp_pos}, sz_of_bytes(chunk), DW_TMP, S_CPSAVE2);
	end else begin
		cp_pos <= cp_pos + {5'd0, chunk};
		tmp3 <= {14'd0, chunk};
		cir_rd(5'h10, sz_of_bytes(chunk), DW_TMP, S_CPREST5);
	end

end
S_CPSAVE2: cir_wr(5'h10, sz_of_bytes(tmp3[2:0]), tmp, S_CPMULT4);
S_CPREST5: wr(ea + {24'd0, cp_pos} - {29'd0, tmp3[2:0]}, sz_of_bytes(tmp3[2:0]), tmp, S_CPMULT4);
S_CPSR: begin
	// transfer status register and scanPC
	if (!cp_dr) begin
		if (cp_resp[8]) cir_wr(5'h18, `SZ_L, scan_pc, S_CPSR2);
		else cir_wr(5'h10, `SZ_W, {16'd0, sr}, S_CPWAIT);
	end else cir_rd(5'h10, `SZ_W, DW_TMP, S_CPSR2);
end
S_CPSR2: begin
	if (!cp_dr) cir_wr(5'h10, `SZ_W, {16'd0, sr}, S_CPWAIT);
	else begin
		sr <= tmp[15:0] & `SR_MASK;
		if (tr_t0) cp_trace_wait <= 1'b1;
		if (cp_resp[8]) cir_rd(5'h18, `SZ_L, DW_TMP, S_CP10);
		else begin flush_req = 1'b1; flush_pc = scan_pc; state <= S_CPWAIT; end
	end
end
S_CP10: begin
	// new scanPC: the pipe refills from it (address error when odd)
	if (tmp[0]) exc_addr_err(tmp);
	else begin flush_req = 1'b1; flush_pc = tmp; state <= S_CPWAIT; end
end

//-------------------------------------------------------------- cpSAVE (UM 10.2.3.3)
S_CPSAVE0: begin
	cp_id <= ir[11:9];
	cpu_flt_fline <= 1'b1;
	dreq({14'd0, 1'b1, 1'b0, ir[11:9], 13'd0} | 32'h04, `SZ_W, 1'b1, 32'd0, `FC_CPU_SPACE, 1'b0, 1'b0, DW_TMP, S_CPSAVE1);
end
S_CPSAVE1: begin : cpsave1
	reg [31:0] base;
	cpu_flt_fline <= 1'b0;
	case (tmp[15:8])
		8'h00: begin
			// empty/reset: only the format word and its reserved word
			if (ir_ea_pd) begin base = rf_b - 32'd4; wreg({1'b1, ir[2:0]}, base); end else base = ea;
			finish;
			wr(base, `SZ_L, {tmp[15:0], 16'd0}, S_FETCH);
		end
		8'h01: begin
			// not ready: service interrupts (pre-instruction frame), read again
			if (irq_pend) begin iack_pc_i <= 1'b1; exc_ilvl <= irq_lvl; state <= S_IACK; end
			else cir_rd(5'h04, `SZ_W, DW_TMP, S_CPSAVE1);
		end
		default: begin
			if (tmp[15:12] == 4'h0 || tmp[1:0] != 2'b00) begin
				// invalid or reserved format, or a length not a multiple of four
				tmp <= {24'd0, `VEC_FMTERR};
				cir_wr(5'h02, `SZ_W, 32'h0001, S_CP_ABORT);
			end else begin
				if (ir_ea_pd) begin base = rf_b - 32'd4 - {24'd0, tmp[7:0]}; wreg({1'b1, ir[2:0]}, base); end else base = ea;
				ea <= base;
				cp_len <= tmp[7:0]; cp_pos <= tmp[7:0];
				wr(base, `SZ_L, {tmp[15:0], 16'd0}, S_CPSV_BODY);
			end
		end
	endcase
end
S_CPSV_BODY: begin
	// state frame entries from the operand CIR to descending addresses (UM Figure 10-14)
	if (cp_pos == 8'd0) finish;
	else cir_rd(5'h10, `SZ_L, DW_TMP, S_CPSV_WR);
end
S_CPSV_WR: begin cp_pos <= cp_pos - 8'd4; wr(ea + {24'd0, cp_pos}, `SZ_L, tmp, S_CPSV_BODY); end

//-------------------------------------------------------------- cpRESTORE (UM 10.2.3.4)
S_CPREST0: begin
	cp_id <= ir[11:9];
	if (ir_ea_pi) ea <= rf_b;
	rd(ir_ea_pi ? rf_b : ea, `SZ_W, DW_TMP, S_CPREST1);
end
S_CPREST1: begin
	cpu_flt_fline <= 1'b1;
	cir_wr(5'h06, `SZ_W, {16'd0, tmp[15:0]}, S_CPREST2);
end
S_CPREST2: begin cpu_flt_fline <= 1'b0; cir_rd(5'h06, `SZ_W, DW_TMP2, S_CPREST3); end
S_CPREST3: begin
	case (tmp2[15:8])
		8'h00: begin
			if (ir_ea_pi) wreg({1'b1, ir[2:0]}, ea + 32'd4);
			finish;
		end
		8'h01: cir_rd(5'h06, `SZ_W, DW_TMP2, S_CPREST3);          // not ready: read again, no interrupts
		default: begin
			if (tmp2[15:12] == 4'h0 || tmp[1:0] != 2'b00) begin
				tmp <= {24'd0, `VEC_FMTERR};
				cir_wr(5'h02, `SZ_W, 32'h0001, S_CP_ABORT);
			end else begin
				cp_len <= tmp[7:0]; cp_pos <= 8'd0;
				if (ir_ea_pi) wreg({1'b1, ir[2:0]}, ea + 32'd4 + {24'd0, tmp[7:0]});
				state <= S_CPRS_BODY;
			end
		end
	endcase
end
S_CPRS_BODY: begin
	// state frame entries from ascending addresses to the operand CIR
	if (cp_pos == cp_len) finish;
	else rd(ea + 32'd4 + {24'd0, cp_pos}, `SZ_L, DW_TMP, S_CPRS_WR);
end
S_CPRS_WR: begin cp_pos <= cp_pos + 8'd4; cir_wr(5'h10, `SZ_L, tmp, S_CPRS_BODY); end

//-------------------------------------------------------------- MMU instructions (UM 9.8, PRM)
S_PMMU0: begin
	// F-line with CpID 0: the second word selects the operation.  An
	// encoding the MC68030 does not support takes the F-line exception here,
	// in supervisor mode (UM 9.8; in user mode the dispatch took the
	// privilege violation).  The supported patterns are the formats of the UM
	// 3.3.3 descriptions (PFLUSH, PLOAD, PMOVE, PTEST: their zero bits, the
	// register and mode fields, control alterable EAs); which reserved bits
	// make an encoding unsupported where the UM does not say follows WinUAE
	// (cpummu030.c mmu_op30_pmove/_pload/_pflush/_ptest: PMOVE R/W=1 with FD;
	// table68k MMUOP030: no PC relative or immediate EA field even for the
	// forms without an operand)
	if (ir[8:6] != 3'b000) exc_pre(`VEC_FLINE);
	else case (ext[15:13])
		3'b000: begin
			// PMOVE TT0/TT1
			if (ext[12:11] != 2'b01 || ext[7:0] != 8'd0 || (ext[9] && ext[8]) || !ir_ea_ctrlalt) exc_pre(`VEC_FLINE);
			else begin ea_ret <= ext[9] ? S_PMOVE_WR : S_PMOVE_RD; state <= S_EA; end
		end
		3'b001: begin
			case (ext[12:10])
				3'b000: begin   // PLOAD
					if (ext[8:5] != 4'd0 || !mmu_fc_ok(ext[4:0]) || !ir_ea_ctrlalt) exc_pre(`VEC_FLINE);
					else begin ea_ret <= S_PLOAD; state <= S_EA; end
				end
				3'b001: begin   // PFLUSHA
					if (ext[9:0] != 10'd0 || ir_ea_pcd || ir_ea_pci || ir_ea_imm || ir_ea_bad) exc_pre(`VEC_FLINE);
					else begin op_req <= 1'b1; op_kind <= 3'd4; state <= S_PFLUSH; end
				end
				3'b100: begin   // PFLUSH by function code
					if (ext[9:8] != 2'd0 || !mmu_fc_ok(ext[4:0]) || ir_ea_pcd || ir_ea_pci || ir_ea_imm || ir_ea_bad) exc_pre(`VEC_FLINE);
					else begin op_req <= 1'b1; op_kind <= 3'd5; op_fc <= mmu_fc(ext[4:0], rf_a[2:0]); op_fcmask <= ext[7:5]; state <= S_PFLUSH; end
				end
				3'b110: begin   // PFLUSH by function code and EA
					if (ext[9:8] != 2'd0 || !mmu_fc_ok(ext[4:0]) || !ir_ea_ctrlalt) exc_pre(`VEC_FLINE);
					else begin ea_ret <= S_PFLUSH2; state <= S_EA; end
				end
				default: exc_pre(`VEC_FLINE);
			endcase
		end
		3'b010: begin
			// PMOVE TC / SRP / CRP
			if (!(ext[12:10] == 3'b000 || ext[12:10] == 3'b010 || ext[12:10] == 3'b011) || ext[7:0] != 8'd0 || (ext[9] && ext[8]) || !ir_ea_ctrlalt)
				exc_pre(`VEC_FLINE);
			else begin ea_ret <= ext[9] ? S_PMOVE_WR : S_PMOVE_RD; state <= S_EA; end
		end
		3'b011: begin
			// PMOVE MMUSR (no FD bit)
			if (ext[12:10] != 3'b000 || ext[8:0] != 9'd0 || !ir_ea_ctrlalt) exc_pre(`VEC_FLINE);
			else begin ea_ret <= ext[9] ? S_PMOVE_WR : S_PMOVE_RD; state <= S_EA; end
		end
		3'b100: begin
			// PTEST
			if (!mmu_fc_ok(ext[4:0]) || !ir_ea_ctrlalt || (ext[8] && ext[12:10] == 3'd0)) exc_pre(`VEC_FLINE);
			else begin ea_ret <= S_PTEST; state <= S_EA; end
		end
		default: exc_pre(`VEC_FLINE);
	endcase
end
S_PLOAD: begin
	op_req <= 1'b1; op_kind <= {2'b00, ~ext[9]}; op_la <= ea; op_fc <= mmu_fc(ext[4:0], rf_a[2:0]);
	state <= S_PFLUSH;
end
S_PFLUSH: begin
	if (op_done) begin
		// translations changed: refill (UM 12.7.1)
		flush_req = 1'b1; flush_pc = scan_pc;
		trace_pend <= tr_t1;
		state <= S_FETCH;
	end
end
S_PFLUSH2: begin
	// PFLUSH fc,#mask,<ea>: the EA is ready
	op_req <= 1'b1; op_kind <= 3'd6; op_la <= ea;
	op_fc <= mmu_fc(ext[4:0], rf_a[2:0]); op_fcmask <= ext[7:5];
	state <= S_PFLUSH;
end
S_PTEST: begin
	op_req <= 1'b1; op_kind <= {2'b01, ~ext[9]}; op_level <= ext[12:10]; op_la <= ea; op_fc <= mmu_fc(ext[4:0], rf_a[2:0]);
	state <= S_PTEST2;
end
S_PTEST2: begin
	if (op_done) begin
		if (ext[8]) wreg({1'b1, ext[7:5]}, op_desc_addr);
		finish;
	end
end
S_PMOVE_RD: begin
	// memory to MMU register
	case (ext[15:13])
		3'b011: rd(ea, `SZ_W, DW_TMP, S_PMOVE_RD3);
		3'b010: begin
			if (ext[12:10] == 3'b000) rd(ea, `SZ_L, DW_TMP, S_PMOVE_RD3);
			else rd(ea, `SZ_L, DW_TMP, S_PMOVE_RD2);
		end
		default: rd(ea, `SZ_L, DW_TMP, S_PMOVE_RD3);
	endcase
end
S_PMOVE_RD2: rd(ea + 32'd4, `SZ_L, DW_TMP2, S_PMOVE_RD3);
S_PMOVE_RD3: begin
	reg_we <= 1'b1;
	reg_fd <= ext[8];
	case (ext[15:13])
		3'b000: begin reg_sel <= ext[10] ? 3'd4 : 3'd3; reg_wdata_lo <= tmp; end
		3'b010: begin
			reg_sel <= (ext[12:10] == 3'b000) ? 3'd0 : (ext[12:10] == 3'b010) ? 3'd1 : 3'd2;
			reg_wdata_hi <= tmp; reg_wdata_lo <= (ext[12:10] == 3'b000) ? tmp : tmp2;
		end
		default: begin reg_sel <= 3'd5; reg_wdata_lo <= tmp; end
	endcase
	sub <= 8'd1;
	state <= S_PMOVE_FIN;
end
S_PMOVE_FIN: begin
	// the MMU reports a configuration error in the second clock after the write
	if (sub[0]) sub <= 8'd0;
	else if (cfg_err) exc_go(`VEC_MMUCONF, `FMT_SIXWORD, scan_pc, pc_i);
	else if (ext[15:13] == 3'b011) finish;
	else begin
		flush_req = 1'b1; flush_pc = scan_pc;       // translation may have changed: refill
		trace_pend <= tr_t1;
		state <= S_FETCH;
	end
end
S_PMOVE_WR: begin
	// MMU register to memory
	case (ext[15:13])
		3'b000: begin finish; wr(ea, `SZ_L, ext[10] ? tt1 : tt0, S_FETCH); end
		3'b011: begin finish; wr(ea, `SZ_W, {16'd0, mmusr}, S_FETCH); end
		default: begin
			if (ext[12:10] == 3'b000) begin finish; wr(ea, `SZ_L, tc, S_FETCH); end
			else begin
				wr(ea, `SZ_L, (ext[12:10] == 3'b010) ? srp_hi : crp_hi, S_PMOVE_WR2);
			end
		end
	endcase
end
S_PMOVE_WR2: begin finish; wr(ea + 32'd4, `SZ_L, (ext[12:10] == 3'b010) ? srp_lo : crp_lo, S_FETCH); end
