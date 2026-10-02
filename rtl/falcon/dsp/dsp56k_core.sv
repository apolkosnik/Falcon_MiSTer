// DSP56001 core for the Falcon DSP (falcon_dsp): instruction decode and
// execution, AGU, program controller (hardware stack, DO/REP loops),
// interrupt controller, and the Falcon memory map with its memories.
//
// Behavioural reference: Hatari src/falcon/dsp_cpu.c, followed function by
// function (dsp56k_execute_instruction, dsp_postexecute_update_pc,
// dsp_postexecute_interrupts, read_memory/write_memory_raw, dsp_write_reg,
// dsp_stack_push/pop, dsp_update_rn*, dsp_calc_ea, dsp_calc_cc, every
// opcodes8h[] / parallel move handler; the data ALU is dsp56k_alu.sv and the
// decoder dsp56k_dec.sv, whose dispatch table is generated from Hatari's
// opcodes8h[] by tb/dsp/gen/gen_tables.py).
//
// Memory map (Hatari dsp_cpu.c, Falcon):
//   P:$0000-$01FF internal program RAM (512 words), P:$0200-$FFFF external
//   RAM (32K words, address bits 14:0).
//   X/Y:$0000-$00FF internal data RAM (256 words each);
//   X/Y:$0100-$01FF ROM tables when OMR.DE=1 (writes ignored), external RAM
//   when DE=0; X/Y:$FFC0-$FFFF peripherals (dsp_periph);
//   other X/Y addresses: external RAM, Y at ext[13:0], X at ext[$4000|13:0].
//   All external RAM is zero wait state; Hatari's cycle rule applies: each
//   external space (P, X, Y) touched by an instruction beyond the first
//   costs 2 cycles.
//
// Timing: one DSP clock cycle = one clk.  Each instruction takes exactly
// Hatari's dsp_core.instr_cycle clocks (2 = one instruction cycle, i.e.
// 16 MIPS at 32 MHz):
//   clock 0 ("E1"): decode the fetched words (opcode + the following word,
//                   both fetched in the previous instruction's last clock),
//                   AGU, data accesses (one clock later for the -(Rn) and
//                   (Rn+Nn) modes, which have 2 extra cycles), ALU operand
//                   preparation and product.
//   access + 1:     data arrive; read-modify-write / memory-to-memory writes.
//   last clock ("EL"): ALU result, register writes, Hatari's
//                   dsp_postexecute_update_pc and dsp_postexecute_interrupts,
//                   fetch of the next opcode and the word after it.
//   The EL logic only uses registered information (instruction word, AGU
//   results, source operands, read data), never the fetch RAM outputs.
//
// Deviations from Hatari (hardware documentation wins, see falcon_dsp.sv):
//   - SWI raises the SWI interrupt (vector P:$0006, level 3); Hatari only
//     spends the cycles.
//   - WAIT stops instruction execution until an interrupt that would be
//     accepted is pending; STOP stops it until the DSP is reset.  Hatari
//     executes both as NOPs.  Leaving WAIT costs one extra fetch cycle.
//   - Bit-reverse (M=0) address update with a modifier of 0: the register is
//     left unchanged (reverse-carry add of 0).  Hatari sets bit 16 of the
//     register (an impossible 17-bit value).
// Everything else, including Hatari's emulation-specific behaviour (the
// interrupt pipeline model, AGU delay slot, register read side effects,
// flags), is reproduced exactly and checked by the differential testbench.

module dsp56k_core (
    input             clk,
    input             rst,            // synchronous DSP reset (dsp_core_reset)

    // execution start (bootstrap finished: Hatari dsp_core.running = 1)
    input             run,            // level: DSP running
    input             run_start,      // pulse: set R0 = boot_pos, OMR = 2
    input       [9:0] boot_pos,

    // bootstrap upload into internal P RAM (only while !run)
    input             boot_we,
    input       [8:0] boot_addr,
    input      [23:0] boot_data,

    // X peripheral port (X:$FFC0-$FFFF) and Y peripheral port (Y:$FFC0-$FFFF)
    // read data valid on the clock after the strobe (like the RAMs)
    output reg        perx_rd,
    output reg        perx_wr,
    output reg  [5:0] perx_addr,
    output reg [23:0] perx_wdata,
    input      [23:0] perx_rdata,
    output reg        pery_rd,
    output reg        pery_wr,
    output reg  [5:0] pery_addr,
    output reg [23:0] pery_wdata,
    input      [23:0] pery_rdata,
    output reg        per_soft_reset, // RESET instruction (dsp_reset())

    // interrupt sources from the peripherals
    input             int_host_rcv,   // status bit 16 (HRDF)
    input             int_host_trx,   // status bit 17 (HTDE)
    input             int_host_cmd,   // status bit 18 (HCP)
    input             int_ssi_rcv,    // status bit 6
    input             int_ssi_trx,    // status bit 8
    input             msk_host_rcv,   // HCR.HRIE
    input             msk_host_trx,   // HCR.HTIE
    input             msk_host_cmd,   // HCR.HCIE
    input             msk_ssi_rcv,    // CRB.RIE
    input             msk_ssi_trx,    // CRB.TIE
    input      [23:0] ipr,            // X:$FFFF
    input       [4:0] hc_vector,      // CVR & $1F
    output            hc_ack,         // host command accepted: clear HCP, HC

    // instruction boundary (testbench / debug)
    output reg        retire /*verilator public_flat_rw*/ // last clock of an instruction
);

`include "dsp56k_tables.svh"

    // ------------------------------------------------------------------
    // constants
    // ------------------------------------------------------------------
    localparam [5:0] RG_X0 = 6'h04, RG_X1 = 6'h05, RG_Y0 = 6'h06, RG_Y1 = 6'h07;
    localparam [5:0] RG_A0 = 6'h08, RG_B0 = 6'h09, RG_A2 = 6'h0a, RG_B2 = 6'h0b;
    localparam [5:0] RG_A1 = 6'h0c, RG_B1 = 6'h0d, RG_A  = 6'h0e, RG_B  = 6'h0f;
    localparam [5:0] RG_LCSAVE = 6'h30;
    localparam [5:0] RG_SR = 6'h39, RG_OMR = 6'h3a, RG_SP = 6'h3b, RG_SSH = 6'h3c;
    localparam [5:0] RG_SSL = 6'h3d, RG_LA = 6'h3e, RG_LC = 6'h3f;

    localparam [1:0] IS_NONE = 2'd0, IS_DISABLED = 2'd1, IS_LONG = 2'd2;

    localparam [3:0] PK_NONE = 4'd0, PK_RUPD = 4'd1, PK_RR = 4'd2, PK_IMM = 4'd3,
                     PK_L = 4'd4, PK_XY = 4'd5, PK_XYXY = 4'd6, PK_0 = 4'd7, PK_1 = 4'd8;

    localparam [2:0] RG_INT = 3'd0, RG_ROM = 3'd1, RG_EXT = 3'd2, RG_PER = 3'd3,
                     RG_IGN = 3'd4, RG_PINT = 3'd5;

    localparam [2:0] ST_IDLE = 3'd0, ST_FETCH = 3'd1, ST_EXEC = 3'd2, ST_WAIT = 3'd3,
                     ST_STOP = 3'd4;

    // ------------------------------------------------------------------
    // architectural state
    // ------------------------------------------------------------------
    reg [23:0] x0 /*verilator public_flat_rw*/, x1 /*verilator public_flat_rw*/;
    reg [23:0] y0 /*verilator public_flat_rw*/, y1 /*verilator public_flat_rw*/;
    reg [23:0] a0 /*verilator public_flat_rw*/, a1 /*verilator public_flat_rw*/;
    reg [23:0] b0 /*verilator public_flat_rw*/, b1 /*verilator public_flat_rw*/;
    reg  [7:0] a2 /*verilator public_flat_rw*/, b2 /*verilator public_flat_rw*/;
    reg [15:0] rr [0:7] /*verilator public_flat_rw*/;
    reg [15:0] nn [0:7] /*verilator public_flat_rw*/;
    reg [15:0] mm [0:7] /*verilator public_flat_rw*/;
    reg [15:0] sr /*verilator public_flat_rw*/;
    reg  [7:0] omr /*verilator public_flat_rw*/;
    reg  [5:0] sp /*verilator public_flat_rw*/;
    reg [15:0] la /*verilator public_flat_rw*/;
    reg [23:0] lc /*verilator public_flat_rw*/;
    reg [23:0] lcsave /*verilator public_flat_rw*/;
    reg [15:0] stk_h [0:15] /*verilator public_flat_rw*/;
    reg [15:0] stk_l [0:15] /*verilator public_flat_rw*/;
    reg [15:0] pc /*verilator public_flat_rw*/;
    reg        loop_rep /*verilator public_flat_rw*/, pc_on_rep /*verilator public_flat_rw*/;
    reg  [1:0] int_state /*verilator public_flat_rw*/;
    reg  [2:0] int_cnt /*verilator public_flat_rw*/;
    reg [15:0] int_fetch /*verilator public_flat_rw*/, int_save_pc /*verilator public_flat_rw*/;
    reg  [1:0] int_ipl /*verilator public_flat_rw*/;
    reg  [5:0] agu_reg /*verilator public_flat_rw*/;
    reg [15:0] agu_val /*verilator public_flat_rw*/;
    reg        st_trace /*verilator public_flat_rw*/, st_swi /*verilator public_flat_rw*/;
    reg        st_illegal /*verilator public_flat_rw*/, st_stkerr /*verilator public_flat_rw*/;

    // sequencer
    reg  [2:0] state /*verilator public_flat_rw*/;
    reg  [6:0] cyc /*verilator public_flat_rw*/;
    reg  [6:0] ncyc /*verilator public_flat_rw*/;
    reg [23:0] ir_q, ew_q;
    reg        fa_ext, fb_ext;       // fetch source of opcode / next word

    wire [15:0] ssh_cur = stk_h[sp[3:0]];
    wire [15:0] ssl_cur = stk_l[sp[3:0]];
    wire [55:0] acc_a = {a2, a1, a0};
    wire [55:0] acc_b = {b2, b1, b0};
    wire        de = omr[2];

    // ------------------------------------------------------------------
    // memories
    // ------------------------------------------------------------------
    reg         pint_we_a, pint_we_b;
    reg  [8:0]  pint_addr_a, pint_addr_b;
    reg  [23:0] pint_d_a, pint_d_b;
    wire [23:0] pint_q_a, pint_q_b;
    dsp_ram_tdp #(.AW(9), .DW(24)) u_pint (
        .clk(clk),
        .we_a(pint_we_a), .addr_a(pint_addr_a), .d_a(pint_d_a), .q_a(pint_q_a),
        .we_b(pint_we_b), .addr_b(pint_addr_b), .d_b(pint_d_b), .q_b(pint_q_b));

    reg         ext_we_a, ext_we_b;
    reg  [14:0] ext_addr_a, ext_addr_b;
    reg  [23:0] ext_d_a, ext_d_b;
    wire [23:0] ext_q_a, ext_q_b;
    dsp_ram_tdp #(.AW(15), .DW(24)) u_ext (
        .clk(clk),
        .we_a(ext_we_a), .addr_a(ext_addr_a), .d_a(ext_d_a), .q_a(ext_q_a),
        .we_b(ext_we_b), .addr_b(ext_addr_b), .d_b(ext_d_b), .q_b(ext_q_b));

    reg         xint_we, yint_we;
    reg  [7:0]  xint_addr, yint_addr;
    reg  [23:0] xint_d, yint_d;
    wire [23:0] xint_q, yint_q;
    dsp_ram_sp #(.AW(8), .DW(24)) u_xint (
        .clk(clk), .we(xint_we), .addr(xint_addr), .d(xint_d), .q(xint_q));
    dsp_ram_sp #(.AW(8), .DW(24)) u_yint (
        .clk(clk), .we(yint_we), .addr(yint_addr), .d(yint_d), .q(yint_q));

    reg         xrom_en, yrom_en;
    reg  [7:0]  xrom_addr, yrom_addr;
    wire [23:0] xrom_q, yrom_q;
    dsp56k_rom u_rom (
        .clk(clk), .x_en(xrom_en), .x_addr(xrom_addr), .x_q(xrom_q),
        .y_en(yrom_en), .y_addr(yrom_addr), .y_q(yrom_q));

    // ------------------------------------------------------------------
    // instruction words and decoders
    // ------------------------------------------------------------------
    wire        e1 = (state == ST_EXEC) && (cyc == 7'd0);
    wire        el = (state == ST_EXEC) && (cyc == ncyc - 7'd1);
    wire [23:0] ir1 = fa_ext ? ext_q_a : pint_q_a;     // fetched opcode (E1 only)
    wire [23:0] ew1 = fb_ext ? ext_q_b : pint_q_b;     // fetched next word (E1 only)

    // E1 decoder (fetched opcode)
    wire [6:0] d1_cls;  wire [3:0] d1_pk;  wire d1_ispm, d1_alu_und;
    wire d1_ea1_used, d1_ea2_used, d1_ea6, d1_ea_m6, d1_ea_imm, d1_ea_xtra, d1_acc_delay;
    wire [2:0] d1_ea1_mode, d1_ea1_rn, d1_ea2_mode, d1_ea2_rn;
    wire d1_bitaa, d1_bitea, d1_bitpp, d1_bitrg, d1_jbaa, d1_jbea, d1_jbpp, d1_jbrg;
    wire d1_bit, d1_jb, d1_bwr, d1_do, d1_rep, d1_movem, d1_jb_set, d1_jb_sub;
    wire [1:0] d1_bitop;
    dsp56k_dec u_dec1 (
        .ir(ir1), .cls(d1_cls), .ispm(d1_ispm), .pk(d1_pk), .alu_und(d1_alu_und),
        .ea1_used(d1_ea1_used), .ea1_mode(d1_ea1_mode), .ea1_rn(d1_ea1_rn),
        .ea2_used(d1_ea2_used), .ea2_mode(d1_ea2_mode), .ea2_rn(d1_ea2_rn),
        .ea6(d1_ea6), .ea_m6(d1_ea_m6), .ea_imm(d1_ea_imm), .ea_xtra(d1_ea_xtra),
        .acc_delay(d1_acc_delay),
        .c_bitaa(d1_bitaa), .c_bitea(d1_bitea), .c_bitpp(d1_bitpp), .c_bitrg(d1_bitrg),
        .c_jbaa(d1_jbaa), .c_jbea(d1_jbea), .c_jbpp(d1_jbpp), .c_jbrg(d1_jbrg),
        .c_bit(d1_bit), .c_jb(d1_jb), .c_bwr(d1_bwr), .c_do(d1_do), .c_rep(d1_rep),
        .c_movem(d1_movem), .bitop(d1_bitop), .jb_set(d1_jb_set), .jb_sub(d1_jb_sub));

    // registered-opcode decoder (later clocks, EL)
    wire [6:0] dq_cls;  wire [3:0] dq_pk;  wire dq_ispm, dq_alu_und;
    wire dq_ea1_used, dq_ea2_used, dq_ea6, dq_ea_m6, dq_ea_imm, dq_ea_xtra, dq_acc_delay;
    wire [2:0] dq_ea1_mode, dq_ea1_rn, dq_ea2_mode, dq_ea2_rn;
    wire dq_bitaa, dq_bitea, dq_bitpp, dq_bitrg, dq_jbaa, dq_jbea, dq_jbpp, dq_jbrg;
    wire dq_bit, dq_jb, dq_bwr, dq_do, dq_rep, dq_movem, dq_jb_set, dq_jb_sub;
    wire [1:0] dq_bitop;
    dsp56k_dec u_decq (
        .ir(ir_q), .cls(dq_cls), .ispm(dq_ispm), .pk(dq_pk), .alu_und(dq_alu_und),
        .ea1_used(dq_ea1_used), .ea1_mode(dq_ea1_mode), .ea1_rn(dq_ea1_rn),
        .ea2_used(dq_ea2_used), .ea2_mode(dq_ea2_mode), .ea2_rn(dq_ea2_rn),
        .ea6(dq_ea6), .ea_m6(dq_ea_m6), .ea_imm(dq_ea_imm), .ea_xtra(dq_ea_xtra),
        .acc_delay(dq_acc_delay),
        .c_bitaa(dq_bitaa), .c_bitea(dq_bitea), .c_bitpp(dq_bitpp), .c_bitrg(dq_bitrg),
        .c_jbaa(dq_jbaa), .c_jbea(dq_jbea), .c_jbpp(dq_jbpp), .c_jbrg(dq_jbrg),
        .c_bit(dq_bit), .c_jb(dq_jb), .c_bwr(dq_bwr), .c_do(dq_do), .c_rep(dq_rep),
        .c_movem(dq_movem), .bitop(dq_bitop), .jb_set(dq_jb_set), .jb_sub(dq_jb_sub));

    // ------------------------------------------------------------------
    // AGU (dsp_calc_ea / dsp_update_rn*), E1
    // ------------------------------------------------------------------
    function automatic [15:0] rev16(input [15:0] v);
        integer k;
        begin
            for (k = 0; k < 16; k = k + 1) rev16[k] = v[15-k];
        end
    endfunction

    function automatic [15:0] smear(input [15:0] v);
        reg [15:0] t;
        begin
            t = v;
            t = t | (t >> 1); t = t | (t >> 2); t = t | (t >> 4); t = t | (t >> 8);
            smear = t;
        end
    endfunction

    // returns {valid, new value}; valid = 0 for M in $8000-$FFFE (no update)
    function automatic [16:0] agu_upd(input [15:0] r, input [15:0] md, input [15:0] m);
        reg [16:0] rr17, lob, hib, modulo;
        reg [15:0] bufmask, absm, lsb;
        reg [17:0] t;
        begin
            if (m == 16'hffff) begin
                agu_upd = {1'b1, r + md};
            end else if (m == 16'h0000) begin
                lsb = md & (16'd0 - md);
                agu_upd = {1'b1, rev16(rev16(r) + rev16(lsb))};
            end else if (m[15] == 1'b0) begin
                rr17    = {1'b1, r};
                modulo  = {1'b0, m} + 17'd1;
                bufmask = smear(m);
                lob     = rr17 & ~{1'b0, bufmask};
                hib     = lob + {1'b0, m};
                absm    = md[15] ? (16'd0 - md) : md;
                t       = {1'b0, rr17} + {{2{md[15]}}, md};
                if ((absm & bufmask) != 16'd0) begin
                    if ({1'b0, absm} > modulo) begin
                        agu_upd = {1'b1, r};
                    end else begin
                        if (t > {1'b0, hib}) t = t - {1'b0, modulo};
                        else if (t < {1'b0, lob}) t = t + {1'b0, modulo};
                        agu_upd = {1'b1, t[15:0]};
                    end
                end else begin
                    agu_upd = {1'b1, t[15:0]};
                end
            end else begin
                agu_upd = {1'b0, r};
            end
        end
    endfunction

    // one effective address: returns {commit, newr, addr}
    function automatic [32:0] calc_ea(input [2:0] mode, input [15:0] r_act, input [15:0] r_s,
                                      input [15:0] n_s, input [15:0] m_s, input [15:0] ewv);
        reg [16:0] u;
        reg [15:0] addr, newr, md;
        reg        commit;
        begin
            addr = r_s; newr = r_act; commit = 1'b0;
            // one update unit: select the modifier first
            case (mode)
                3'd0: md = 16'd0 - n_s;       // (Rn)-Nn
                3'd1, 3'd5: md = n_s;         // (Rn)+Nn, (Rn+Nn)
                3'd3: md = 16'h0001;          // (Rn)+
                default: md = 16'hffff;       // (Rn)-, -(Rn)
            endcase
            u = agu_upd(r_s, md, m_s);
            case (mode)
                3'd0, 3'd1, 3'd2, 3'd3: if (u[16]) begin newr = u[15:0]; commit = 1'b1; end
                3'd5: addr = u[16] ? u[15:0] : r_act;
                3'd6: addr = ewv;
                3'd7: begin
                    addr = u[16] ? u[15:0] : r_act;
                    if (u[16]) begin newr = u[15:0]; commit = 1'b1; end
                end
                default: ;
            endcase
            calc_ea = {commit, newr, addr};
        end
    endfunction

    // AGU pipeline (Hatari agu_pipeline_reg[0] / val[0]) substitution
    wire [15:0] r1_s = (agu_reg == {3'b010, d1_ea1_rn}) ? agu_val : rr[d1_ea1_rn];
    wire [15:0] n1_s = (agu_reg == {3'b011, d1_ea1_rn}) ? agu_val : nn[d1_ea1_rn];
    wire [15:0] m1_s = (agu_reg == {3'b100, d1_ea1_rn}) ? agu_val : mm[d1_ea1_rn];
    wire [15:0] r2_s = (agu_reg == {3'b010, d1_ea2_rn}) ? agu_val : rr[d1_ea2_rn];
    wire [15:0] n2_s = (agu_reg == {3'b011, d1_ea2_rn}) ? agu_val : nn[d1_ea2_rn];
    wire [15:0] m2_s = (agu_reg == {3'b100, d1_ea2_rn}) ? agu_val : mm[d1_ea2_rn];
    wire [32:0] agu1 = calc_ea(d1_ea1_mode, rr[d1_ea1_rn], r1_s, n1_s, m1_s, ew1[15:0]);
    wire [32:0] agu2 = calc_ea(d1_ea2_mode, rr[d1_ea2_rn], r2_s, n2_s, m2_s, ew1[15:0]);
    wire [15:0] agu1_addr = agu1[15:0];
    wire [15:0] agu1_newr = agu1[31:16];
    wire        agu1_commit = agu1[32] && d1_ea1_used && (d1_cls != C_LUA);
    // address for an access issued in E1 (modes 0-4 use Rn, mode 6 the next
    // word); modes 5 and 7 are issued one clock later from agu1_addr
    wire [15:0] agu1_fast = (d1_ea1_mode == 3'd6) ? ew1[15:0] : r1_s;
    wire [15:0] agu2_addr = agu2[15:0];
    wire [15:0] agu2_newr = agu2[31:16];
    wire        agu2_commit = agu2[32] && d1_ea2_used;

    // ------------------------------------------------------------------
    // limiter (dsp_pm_read_accu24)
    // ------------------------------------------------------------------
    function automatic [24:0] lim24(input [7:0] e, input [23:0] h, input [23:0] l,
                                    input [1:0] sc);
        reg [31:0] v;
        begin
            v = {e, h};
            case (sc)
                2'd1: v = v >> 1;
                2'd2: v = {v[30:0], l[23]};
                default: ;
            endcase
            v = {8'd0, v[23:0]};
            if (e == 8'h00 && v[23] == 1'b0) lim24 = {1'b0, v[23:0]};
            else if (e == 8'hff && v[23] == 1'b1) lim24 = {1'b0, v[23:0]};
            else if (e[7]) lim24 = {1'b1, 24'h800000};
            else lim24 = {1'b1, 24'h7fffff};
        end
    endfunction
    wire [24:0] lim_a = lim24(a2, a1, a0, sr[11:10]);
    wire [24:0] lim_b = lim24(b2, b1, b0, sr[11:10]);

    // ------------------------------------------------------------------
    // generic register read (dsp_core.registers[n]) in E1; A/B are read
    // limited when the instruction uses dsp_pm_read_accu24, else 0
    // (registers[A]).  R registers see this instruction's AGU update
    // (Hatari reads them after dsp_calc_ea).
    // ------------------------------------------------------------------
    reg  [5:0] sreg;
    reg        sreg_lim;
    reg [23:0] sval;
    reg        sval_l;
    always @* begin
        sreg = ir1[13:8]; sreg_lim = 1'b1;
        if (d1_ispm) begin
            if (d1_pk == PK_RR) sreg = {1'b0, ir1[17:13]};
            else sreg = {1'b0, ir1[21:20], ir1[18:16]};          // pm_5 numreg
        end else case (d1_cls)
            C_MOVEC_REG: begin sreg = ir1[15] ? ir1[13:8] : ir1[5:0]; sreg_lim = ir1[15]; end
            C_MOVEC_AA, C_MOVEC_EA: begin sreg = ir1[5:0]; sreg_lim = 1'b0; end
            C_MOVEM_AA, C_MOVEM_EA: sreg = ir1[5:0];
            default: sreg = ir1[13:8];   // movep_0, bit/jump reg, do/rep reg
        endcase
    end
    always @* begin
        sval = 24'd0; sval_l = 1'b0;
        case (sreg)
            RG_X0: sval = x0;  RG_X1: sval = x1;  RG_Y0: sval = y0;  RG_Y1: sval = y1;
            RG_A0: sval = a0;  RG_B0: sval = b0;
            RG_A2: sval = {16'd0, a2};  RG_B2: sval = {16'd0, b2};
            RG_A1: sval = a1;  RG_B1: sval = b1;
            RG_A: if (sreg_lim) begin sval = lim_a[23:0]; sval_l = lim_a[24]; end
            RG_B: if (sreg_lim) begin sval = lim_b[23:0]; sval_l = lim_b[24]; end
            6'h10, 6'h11, 6'h12, 6'h13, 6'h14, 6'h15, 6'h16, 6'h17:
                sval = (agu1_commit && sreg[2:0] == d1_ea1_rn) ? {8'd0, agu1_newr}
                                                                 : {8'd0, rr[sreg[2:0]]};
            6'h18, 6'h19, 6'h1a, 6'h1b, 6'h1c, 6'h1d, 6'h1e, 6'h1f: sval = {8'd0, nn[sreg[2:0]]};
            6'h20, 6'h21, 6'h22, 6'h23, 6'h24, 6'h25, 6'h26, 6'h27: sval = {8'd0, mm[sreg[2:0]]};
            RG_LCSAVE: sval = lcsave;
            RG_SR:  sval = {8'd0, sr};
            RG_OMR: sval = {16'd0, omr};
            RG_SP:  sval = {18'd0, sp};
            RG_SSH: sval = {8'd0, ssh_cur};
            RG_SSL: sval = {8'd0, ssl_cur};
            RG_LA:  sval = {8'd0, la};
            RG_LC:  sval = lc;
            default: sval = 24'd0;
        endcase
    end

    function automatic [24:0] src_xab(input [1:0] s, input [23:0] vx0, input [23:0] vx1,
                                      input [24:0] la_, input [24:0] lb_);
        case (s)
            2'd0: src_xab = {1'b0, vx0};
            2'd1: src_xab = {1'b0, vx1};
            2'd2: src_xab = la_;
            default: src_xab = lb_;
        endcase
    endfunction

    // ------------------------------------------------------------------
    // memory access plan (E1): acc1 = primary access (X, Y or P space),
    // acc2 = Y access of L: and XY moves, accw = second-phase write whose
    // data come from the read.
    // ------------------------------------------------------------------
    reg        a1_en, a1_we, a2_en, a2_we, aw_en;
    reg  [1:0] a1_sp, aw_sp;      // 0 X, 1 Y, 2 P
    reg [15:0] a1_ao, a2_ao, aw_ao;   // address parts not from the AGU
    reg [23:0] a1_wd, a2_wd;
    reg        a1_agu;            // a1_ad is the effective address (agu1)
    reg        aw_agu;            // aw_ad is the effective address (agu1)
    reg        lhit;              // dsp_pm_read_accu24 limited (sets SR.L)
    reg        ew_rd;             // instruction reads P:pc+1
    always @* begin
        reg [24:0] t1, t2;
        a1_en = 1'b0; a1_we = 1'b0; a1_sp = 2'd0; a1_ao = 16'd0; a1_wd = 24'd0;
        a2_en = 1'b0; a2_we = 1'b0; a2_ao = 16'd0; a2_wd = 24'd0;
        aw_en = 1'b0; aw_sp = 2'd0; aw_ao = 16'd0;
        a1_agu = 1'b0; aw_agu = 1'b0;
        lhit = 1'b0; ew_rd = d1_ea_m6 | d1_jb | d1_do;
        t1 = 25'd0; t2 = 25'd0;
        if (d1_ispm) begin
            case (d1_pk)
                PK_RR: lhit = sval_l;
                PK_XY: begin
                    a1_en = !(ir1[15] && d1_ea_imm);
                    a1_we = !ir1[15];
                    a1_sp = {1'b0, ir1[19]};
                    a1_ao = {10'd0, ir1[13:8]}; a1_agu = ir1[14];
                    a1_wd = sval;
                    if (!ir1[15]) lhit = sval_l;
                end
                PK_0: begin
                    t1 = ir1[16] ? lim_b : lim_a;
                    a1_en = 1'b1; a1_we = 1'b1; a1_sp = {1'b0, ir1[15]};
                    a1_ao = 16'd0; a1_agu = 1'b1; a1_wd = t1[23:0];
                    lhit = t1[24];
                end
                PK_1: begin
                    a1_sp = {1'b0, ir1[14]};
                    a1_ao = 16'd0; a1_agu = 1'b1;
                    a1_we = !ir1[15];
                    a1_en = !(ir1[15] && d1_ea_imm);
                    t1 = ir1[14] ? src_xab(ir1[17:16], y0, y1, lim_a, lim_b)
                                 : src_xab(ir1[19:18], x0, x1, lim_a, lim_b);
                    a1_wd = t1[23:0];
                    t2 = (ir1[14] ? ir1[19] : ir1[17]) ? lim_b : lim_a;
                    lhit = (!ir1[15] && t1[24]) || t2[24];
                end
                PK_L: begin
                    a1_en = 1'b1; a2_en = 1'b1;
                    a1_we = !ir1[15]; a2_we = !ir1[15];
                    a1_sp = 2'd0;
                    a1_ao = {10'd0, ir1[13:8]}; a1_agu = ir1[14];
                    a2_ao = a1_ao;
                    case ({ir1[19], ir1[17:16]})
                        3'd0: begin a1_wd = a1; a2_wd = a0; end
                        3'd1: begin a1_wd = b1; a2_wd = b0; end
                        3'd2: begin a1_wd = x1; a2_wd = x0; end
                        3'd3: begin a1_wd = y1; a2_wd = y0; end
                        3'd4: begin a1_wd = lim_a[23:0];
                                    a2_wd = lim_a[24] ? (lim_a[23] ? 24'd0 : 24'hffffff) : a0;
                                    if (!ir1[15]) lhit = lim_a[24]; end
                        3'd5: begin a1_wd = lim_b[23:0];
                                    a2_wd = lim_b[24] ? (lim_b[23] ? 24'd0 : 24'hffffff) : b0;
                                    if (!ir1[15]) lhit = lim_b[24]; end
                        3'd6: begin a1_wd = lim_a[23:0]; a2_wd = lim_b[23:0];
                                    if (!ir1[15]) lhit = lim_a[24] | lim_b[24]; end
                        default: begin a1_wd = lim_b[23:0]; a2_wd = lim_a[23:0];
                                    if (!ir1[15]) lhit = lim_a[24] | lim_b[24]; end
                    endcase
                end
                PK_XYXY: begin
                    t1 = src_xab(ir1[19:18], x0, x1, lim_a, lim_b);
                    t2 = src_xab(ir1[17:16], y0, y1, lim_a, lim_b);
                    a1_en = 1'b1; a1_we = !ir1[15]; a1_sp = 2'd0; a1_ao = 16'd0; a1_agu = 1'b1; a1_wd = t1[23:0];
                    a2_en = 1'b1; a2_we = !ir1[22]; a2_ao = 16'd0; a2_wd = t2[23:0];
                    lhit = (!ir1[15] && t1[24]) || (!ir1[22] && t2[24]);
                end
                default: ;
            endcase
        end else begin
            if (d1_bitaa | d1_bitea | d1_bitpp | d1_jbaa | d1_jbea | d1_jbpp) begin
                a1_en = 1'b1; a1_sp = {1'b0, ir1[6]};
                a1_ao = (d1_bitaa | d1_jbaa) ? {10'd0, ir1[13:8]} : {10'h3ff, ir1[13:8]};
                a1_agu = d1_bitea | d1_jbea;
                aw_agu = d1_bitea;
                aw_en = d1_bwr; aw_sp = a1_sp; aw_ao = a1_ao;
            end
            if (d1_bitrg | d1_jbrg) lhit = sval_l;
            case (d1_cls)
                C_MOVEC_AA, C_MOVEC_EA: begin
                    a1_sp = {1'b0, ir1[6]};
                    a1_ao = {10'd0, ir1[13:8]};
                    a1_agu = (d1_cls == C_MOVEC_EA);
                    a1_we = !ir1[15];
                    a1_en = !(ir1[15] && d1_cls == C_MOVEC_EA && d1_ea_imm);
                    a1_wd = sval;
                end
                C_MOVEC_REG: lhit = ir1[15] ? sval_l : 1'b0;
                C_MOVEM_AA, C_MOVEM_EA: begin
                    a1_en = 1'b1; a1_sp = 2'd2; a1_we = !ir1[15];
                    a1_ao = {10'd0, ir1[13:8]};
                    a1_agu = (d1_cls == C_MOVEM_EA);
                    a1_wd = sval;
                    if (!ir1[15]) lhit = sval_l;
                end
                C_MOVEP_0: begin
                    a1_en = 1'b1; a1_sp = {1'b0, ir1[16]}; a1_we = ir1[15];
                    a1_ao = {10'h3ff, ir1[5:0]};
                    a1_wd = sval;
                    if (ir1[15]) lhit = sval_l;
                end
                C_MOVEP_1: begin
                    a1_en = 1'b1;
                    aw_en = 1'b1;
                    if (ir1[15]) begin   // P:ea -> X/Y:pp
                        a1_sp = 2'd2; a1_ao = 16'd0; a1_agu = 1'b1;
                        aw_sp = {1'b0, ir1[16]}; aw_ao = {10'h3ff, ir1[5:0]};
                    end else begin       // X/Y:pp -> P:ea
                        a1_sp = {1'b0, ir1[16]}; a1_ao = {10'h3ff, ir1[5:0]};
                        aw_sp = 2'd2; aw_ao = 16'd0; aw_agu = 1'b1;
                    end
                end
                C_MOVEP_23: begin
                    if (ir1[15]) begin
                        if (d1_ea_imm) begin  // #xxxxxx -> X/Y:pp
                            a1_en = 1'b1; a1_we = 1'b1; a1_sp = {1'b0, ir1[16]};
                            a1_ao = {10'h3ff, ir1[5:0]}; a1_wd = ew1;
                        end else begin        // X/Y:ea -> X/Y:pp
                            a1_en = 1'b1; a1_sp = {1'b0, ir1[6]}; a1_ao = 16'd0; a1_agu = 1'b1;
                            aw_en = 1'b1; aw_sp = {1'b0, ir1[16]}; aw_ao = {10'h3ff, ir1[5:0]};
                        end
                    end else begin            // X/Y:pp -> X/Y:ea
                        a1_en = 1'b1; a1_sp = {1'b0, ir1[16]}; a1_ao = {10'h3ff, ir1[5:0]};
                        aw_en = 1'b1; aw_sp = {1'b0, ir1[6]}; aw_ao = 16'd0; aw_agu = 1'b1;
                    end
                end
                C_DO_AA, C_REP_AA: begin
                    a1_en = 1'b1; a1_sp = {1'b0, ir1[6]}; a1_ao = {10'd0, ir1[13:8]};
                end
                C_DO_EA, C_REP_EA: begin
                    a1_en = 1'b1; a1_sp = {1'b0, ir1[6]}; a1_ao = 16'd0; a1_agu = 1'b1;
                end
                C_DO_REG, C_REP_REG: lhit = sval_l;
                default: ;
            endcase
        end
    end

    // region of an X/Y access (read_memory / write_memory_raw)
    function automatic [2:0] xy_region(input [15:0] ad, input we, input dev);
        if (ad[15:6] == 10'h3ff) xy_region = RG_PER;
        else if (ad[15:8] == 8'h00) xy_region = RG_INT;
        else if (ad[15:9] == 7'h00 && dev) xy_region = we ? RG_IGN : RG_ROM;
        else xy_region = RG_EXT;
    endfunction
    function automatic [2:0] region(input [1:0] spc, input [15:0] ad, input we, input dev);
        if (spc == 2'd2) region = (ad[15:9] == 7'h00) ? RG_PINT : RG_EXT;
        else region = xy_region(ad, we, dev);
    endfunction
    function automatic [14:0] ext_addr(input [1:0] spc, input [15:0] ad);
        case (spc)
            2'd0: ext_addr = {1'b1, ad[13:0]};
            2'd1: ext_addr = {1'b0, ad[13:0]};
            default: ext_addr = ad[14:0];
        endcase
    endfunction

    // full addresses (registered for a delayed issue, cycle count) and the
    // E1 issue addresses (no AGU arithmetic on that path)
    reg         a2_l;   // L: move: Y address = X address
    always @* a2_l = d1_ispm && d1_pk == PK_L;
    wire [15:0] a1_ad = a1_agu ? agu1_addr : a1_ao;
    wire [15:0] a2_ad = a2_l ? a1_ad : (d1_ispm && d1_pk == PK_XYXY) ? agu2_addr : a2_ao;
    wire [15:0] aw_ad = aw_agu ? agu1_addr : (d1_bitea ? agu1_addr : aw_ao);
    wire [2:0]  r1_c = region(a1_sp, a1_ad, a1_we, de);
    wire [15:0] a1_ad_now = a1_agu ? agu1_fast : a1_ao;      // E1 issue (modes 0-4, 6)
    wire [2:0]  r1_now = region(a1_sp, a1_ad_now, a1_we, de);
    wire [15:0] a2_ad_now = (d1_pk == PK_L) ? a1_ad_now : r2_s;  // L: same address; XY: modes 0-4
    wire [2:0]  r2_now = region(2'd1, a2_ad_now, a2_we, de);
    wire [2:0] r2_c = region(2'd1, a2_ad, a2_we, de);
    wire [2:0] rw_c1 = region(aw_sp, aw_ad, 1'b1, de);
    wire [15:0] aw_ad_now = (aw_agu | d1_bitea) ? agu1_fast : aw_ao;
    wire [2:0] rw_now = region(aw_sp, aw_ad_now, 1'b1, de);

    // external access penalty: 2 cycles per external space beyond the first
    function automatic [6:0] ext_pen(input ep, input ex, input ey);
        reg [1:0] nb;
        begin
            nb = {1'b0, ex} + {1'b0, ey} + {1'b0, ep};
            ext_pen = (nb > 2'd1) ? {4'd0, nb - 2'd1, 1'b0} : 7'd0;
        end
    endfunction

    // ------------------------------------------------------------------
    // cycle count (Hatari instr_cycle), E1
    // ------------------------------------------------------------------
    // For the -(Rn) / (Rn+Nn) modes the accesses are issued one clock later;
    // their external penalty is added then (from the registered addresses),
    // these instructions always take 4 or more cycles.
    reg [6:0] cyc_total, cyc_base;
    reg       ext_p0;
    always @* begin
        reg [6:0] c;
        reg       ext_x, ext_y, ext_p;
        c = 7'd2;
        if (d1_bit) c = c + 7'd2;
        if (d1_do) c = c + 7'd4;
        if (d1_jb) c = c + 7'd4;
        case (d1_cls)
            C_JCC_IMM, C_JCC_EA, C_JMP_IMM, C_JMP_EA, C_JSCC_IMM, C_JSCC_EA,
            C_JSR_IMM, C_JSR_EA, C_LUA, C_MOVEP_0, C_MOVEP_23: c = c + 7'd2;
            C_MOVEM_AA, C_MOVEM_EA, C_MOVEP_1: c = c + 7'd4;
            C_REP_AA, C_REP_EA, C_REP_IMM, C_REP_REG: c = c + 7'd2;
            C_UNDEFINED: c = c + 7'd100;
            C_OPCODE8H_0: begin
                case (ir1)
                    24'h000004, 24'h00000c, 24'h000084: c = c + 7'd2;   // rti rts reset
                    24'h000006: c = c + 7'd6;                            // swi
                    24'h000000, 24'h000005, 24'h000086, 24'h000087, 24'h00008c: ;
                    default: c = c + 7'd100;                             // undefined
                endcase
            end
            default: ;
        endcase
        if (d1_alu_und) c = c + 7'd100;
        if (d1_ea_xtra) c = c + 7'd2;
        cyc_base = c;
        // external memory accesses (fetch of the opcode / next word, data)
        ext_p0 = (pc[15:9] != 7'd0) || (ew_rd && ((pc + 16'd1) >> 9) != 16'd0);
        ext_p = ext_p0; ext_x = 1'b0; ext_y = 1'b0;
        if (a1_en && r1_now == RG_EXT)
            case (a1_sp) 2'd0: ext_x = 1'b1; 2'd1: ext_y = 1'b1; default: ext_p = 1'b1; endcase
        if (a2_en && r2_now == RG_EXT) ext_y = 1'b1;
        if (aw_en && rw_now == RG_EXT)
            case (aw_sp) 2'd0: ext_x = 1'b1; 2'd1: ext_y = 1'b1; default: ext_p = 1'b1; endcase
        cyc_total = d1_acc_delay ? c : (c + ext_pen(ext_p, ext_x, ext_y));
    end

    // ------------------------------------------------------------------
    // access issue (E1, or the next clock for modes 5 / 7) and data return
    // ------------------------------------------------------------------
    reg        delay_q;
    reg        q_a1_en, q_a1_we, q_a2_en, q_a2_we, q_aw_en;
    reg  [1:0] q_a1_sp, q_aw_sp;
    reg [15:0] q_a1_ad, q_a2_ad, q_aw_ad;
    reg [23:0] q_a1_wd, q_a2_wd;
    reg  [2:0] q_r1, q_r2, q_rw;

    // final count for the delayed modes (clock 1, registered plan)
    reg  [6:0] cyc_base_q;
    reg        ext_p0_q;
    wire [6:0] cyc_total_dly;
    wire       dx_x = (q_a1_en && q_r1 == RG_EXT && q_a1_sp == 2'd0) || (q_aw_en && q_rw == RG_EXT && q_aw_sp == 2'd0);
    wire       dx_y = (q_a1_en && q_r1 == RG_EXT && q_a1_sp == 2'd1) || (q_aw_en && q_rw == RG_EXT && q_aw_sp == 2'd1) ||
                      (q_a2_en && q_r2 == RG_EXT);
    wire       dx_p = ext_p0_q || (q_a1_en && q_r1 == RG_EXT && q_a1_sp == 2'd2) || (q_aw_en && q_rw == RG_EXT && q_aw_sp == 2'd2);
    assign     cyc_total_dly = cyc_base_q + ext_pen(dx_p, dx_x, dx_y);

    wire       iss_now = e1 && !d1_acc_delay;
    wire       iss_dly = (state == ST_EXEC) && !e1 && delay_q && cyc == 7'd1;
    wire       t_iss   = iss_now || iss_dly;
    // the plan in effect at the issue clock
    wire        p_a1_en = iss_now ? a1_en : q_a1_en;
    wire        p_a1_we = iss_now ? a1_we : q_a1_we;
    wire  [1:0] p_a1_sp = iss_now ? a1_sp : q_a1_sp;
    wire [15:0] p_a1_ad = iss_now ? a1_ad_now : q_a1_ad;
    wire [23:0] p_a1_wd = iss_now ? a1_wd : q_a1_wd;
    wire  [2:0] p_r1    = iss_now ? r1_now : q_r1;
    wire        p_a2_en = iss_now ? a2_en : q_a2_en;
    wire        p_a2_we = iss_now ? a2_we : q_a2_we;
    wire [15:0] p_a2_ad = iss_now ? a2_ad_now : q_a2_ad;
    wire [23:0] p_a2_wd = iss_now ? a2_wd : q_a2_wd;
    wire  [2:0] p_r2    = iss_now ? r2_now : q_r2;
    wire        p_aw_en = iss_now ? aw_en : q_aw_en;
    wire  [1:0] p_aw_sp = iss_now ? aw_sp : q_aw_sp;
    wire [15:0] p_aw_ad = iss_now ? aw_ad : q_aw_ad;
    wire  [2:0] p_rw    = iss_now ? rw_c1 : q_rw;

    reg        t_wr2;                   // clock after the issue (data valid)
    reg  [2:0] r1_q, r2_q;              // region of the issued reads
    reg  [1:0] a1_sp_q;
    reg [23:0] rd1_q, rd2_q;
    reg        aw_en_q;
    reg  [1:0] aw_sp_q;
    reg [15:0] aw_ad_q;
    reg  [2:0] rw_q;

    reg [23:0] rd1_c, rd2_c;
    always @* begin
        case (r1_q)
            RG_INT:  rd1_c = (a1_sp_q == 2'd1) ? yint_q : xint_q;
            RG_ROM:  rd1_c = (a1_sp_q == 2'd1) ? yrom_q : xrom_q;
            RG_PER:  rd1_c = (a1_sp_q == 2'd1) ? pery_rdata : perx_rdata;
            RG_PINT: rd1_c = pint_q_a;
            RG_EXT:  rd1_c = (a1_sp_q == 2'd1) ? ext_q_b : ext_q_a;
            default: rd1_c = 24'd0;
        endcase
        case (r2_q)
            RG_INT:  rd2_c = yint_q;
            RG_ROM:  rd2_c = yrom_q;
            RG_PER:  rd2_c = pery_rdata;
            RG_EXT:  rd2_c = ext_q_b;
            default: rd2_c = 24'd0;
        endcase
    end
    wire [23:0] rd1 = t_wr2 ? rd1_c : rd1_q;
    wire [23:0] rd2 = t_wr2 ? rd2_c : rd2_q;

    // second-phase write data (read-modify-write, memory to memory)
    reg [23:0] aw_wd;
    always @* begin
        reg [31:0] v, bm;
        v  = {8'd0, rd1_c};
        bm = 32'd1 << ir_q[4:0];
        aw_wd = rd1_c;
        if (dq_bwr) begin
            case (dq_bitop)
                2'd0: v = v & ~bm;
                2'd1: v = v | bm;
                default: v = v ^ bm;
            endcase
            aw_wd = v[23:0];
        end
    end

    // interrupt vector word read (long interrupt detection) one clock
    // before the last clock of the instruction, on P RAM port B
    wire        vec_rd = (state == ST_EXEC) && (e1 || cyc == ncyc - 7'd2) &&
                         int_state == IS_DISABLED && (int_cnt == 3'd4 || int_cnt == 3'd3);
    wire [15:0] vec_ad = (int_cnt == 3'd4) ? int_fetch : (int_fetch + 16'd1);
    // (P-space data writes belong to instructions of 6 or more cycles and never
    // coincide with this read, so no write forwarding is needed)
    wire [23:0] vec_word = pint_q_b;

    // ------------------------------------------------------------------
    // data ALU
    // ------------------------------------------------------------------
    wire        alu_wr, alu_dst_b;
    wire [55:0] alu_res;
    wire [15:0] alu_sr;
    reg         lhit_q;
    wire [15:0] sr_l = sr | (lhit_q ? 16'h0040 : 16'h0000);

    dsp56k_alu u_alu (
        .clk(clk), .pre_en(e1), .op(ir1[7:0]),
        .a(acc_a), .b(acc_b), .x0(x0), .x1(x1), .y0(y0), .y1(y1),
        .sr(sr_l), .wr_d(alu_wr), .dst_b(alu_dst_b), .res(alu_res), .sr_out(alu_sr));

    reg [23:0] sval_q;
    reg [15:0] ea1_addr_q, lua_q;

    // ------------------------------------------------------------------
    // helpers for the last clock
    // ------------------------------------------------------------------
    // dsp_stack_push / dsp_stack_pop on the stack pointer: {error, new sp}
    function automatic [6:0] sp_push(input [5:0] s);
        reg [4:0] st;
        begin
            st = {1'b0, s[3:0]} + 5'd1;
            sp_push = {!s[4] && st[4], s[5], s[4] | st[4], st[3:0]};
        end
    endfunction
    function automatic [6:0] sp_pop(input [5:0] s);
        if (s[3:0] == 4'd0) sp_pop = {!s[4], 6'h3f};
        else sp_pop = {1'b0, s[5:4], s[3:0] - 4'd1};
    endfunction

    // dsp_calc_cc
    function automatic cc_true(input [3:0] cc, input [15:0] s);
        reg c, v, z, n, u, e, l;
        begin
            c = s[0]; v = s[1]; z = s[2]; n = s[3]; u = s[4]; e = s[5]; l = s[6];
            case (cc)
                4'd0:  cc_true = !c;
                4'd1:  cc_true = !(n ^ v);
                4'd2:  cc_true = !z;
                4'd3:  cc_true = !n;
                4'd4:  cc_true = !(z | (!u & !e));
                4'd5:  cc_true = !e;
                4'd6:  cc_true = !l;
                4'd7:  cc_true = !(z | (n ^ v));
                4'd8:  cc_true = c;
                4'd9:  cc_true = n ^ v;
                4'd10: cc_true = z;
                4'd11: cc_true = n;
                4'd12: cc_true = z | (!u & !e);
                4'd13: cc_true = e;
                4'd14: cc_true = l;
                default: cc_true = z | (n ^ v);
            endcase
        end
    endfunction

    function automatic is_long_insn(input [23:0] w);
        is_long_insn = (w[23:12] == 12'h0d0) || ((w & 24'hffc0ff) == 24'h0bc080) ||
                       (w[23:12] == 12'h0c0) || ((w & 24'hffc0ff) == 24'h0ac080);
    endfunction

    // dsp_ccr_update_e_u_n_z (for NORM; the ALU has its own copy)
    function automatic [15:0] euzn_c(input [15:0] s, input [55:0] r);
        reg [15:0] o;
        begin
            o = s & ~16'h003c;
            case (s[11:10])
                2'd0: begin
                    if (!(r[55:47] == 9'h000 || r[55:47] == 9'h1ff)) o[5] = 1'b1;
                    if (r[47] == r[46]) o[4] = 1'b1;
                end
                2'd1: begin
                    if (!(r[55:48] == 8'h00 || r[55:48] == 8'hff)) o[5] = 1'b1;
                    if (r[48] == r[47]) o[4] = 1'b1;
                end
                2'd2: begin
                    if (!(r[55:46] == 10'h000 || r[55:46] == 10'h3ff)) o[5] = 1'b1;
                    if (r[46] == r[45]) o[4] = 1'b1;
                end
                default: ;
            endcase
            if (s[11:10] != 2'd3) begin
                if (r == 56'd0) o[2] = 1'b1;
                o[3] = r[55];
            end
            euzn_c = o;
        end
    endfunction

    // stack entry read after the phase-1 writes (write buffer, then array)
    function automatic [31:0] stk_rd(input [3:0] ix,
                                     input wah, input wal, input [3:0] wai, input [15:0] wahv, input [15:0] walv,
                                     input wbh, input [3:0] wbi, input [15:0] wbhv, input [15:0] wblv,
                                     input [15:0] bh, input [15:0] bl);
        reg [15:0] h, l;
        begin
            h = bh; l = bl;
            if (wah && wai == ix) h = wahv;
            if (wal && wai == ix) l = walv;
            if (wbh && wbi == ix) begin h = wbhv; l = wblv; end
            stk_rd = {h, l};
        end
    endfunction

    // ------------------------------------------------------------------
    // last clock (EL): instruction effects, update_pc, interrupts.
    // Inputs are registered only (ir_q, dq_*, ew_q, sval_q, rd1/rd2 from the
    // read registers or from the RAMs read in the previous clock, ALU).
    // ------------------------------------------------------------------
    reg [23:0] n_x0, n_x1, n_y0, n_y1, n_a0, n_a1, n_b0, n_b1;
    reg  [7:0] n_a2, n_b2;
    reg [15:0] n_rr [0:7];
    reg [15:0] n_nn [0:7];
    reg [15:0] n_mm [0:7];
    reg [15:0] n_sr;
    reg  [7:0] n_omr;
    reg  [5:0] n_sp;
    reg [15:0] n_la;
    reg [23:0] n_lc, n_lcsave;
    reg [15:0] n_pc, n_pc1;          // next pc and next pc + 1 (fetch words)
    reg        n_loop_rep, n_pc_on_rep;
    reg  [1:0] n_int_state;
    reg  [2:0] n_int_cnt;
    reg [15:0] n_int_fetch, n_int_save_pc;
    reg  [1:0] n_int_ipl;
    reg  [5:0] n_agu_reg;
    reg [15:0] n_agu_val;
    reg        n_st_trace, n_st_swi, n_st_illegal, n_st_stkerr;
    reg        n_hc_ack, n_wait, n_stop;
    // stack entry writes of this instruction, applied in the order a, b, c
    reg        wa_h, wa_l, wb_h, wc_h;
    reg  [3:0] wa_i, wb_i, wc_i;
    reg [15:0] wa_hv, wa_lv, wb_hv, wb_lv, wc_hv, wc_lv;

    assign hc_ack = el && n_hc_ack;

    wire [15:0] pc_p1 = pc + 16'd1;
    wire [15:0] pc_p2 = pc + 16'd2;
    wire [15:0] pc_p3 = pc + 16'd3;
    wire [16:0] if_p1 = {1'b0, int_fetch} + 17'd1;
    wire [16:0] if_p2 = {1'b0, int_fetch} + 17'd2;
    wire [15:0] isp_p1 = int_save_pc + 16'd1;
    wire  [6:0] sp_push_sp = sp_push(sp);
    wire [31:0] sval_q32 = {8'd0, sval_q};
    wire        sval_q_bit = sval_q32[ir_q[4:0]];
    wire  [3:0] sp_m1 = sp[3:0] - 4'd1;

    integer i;
    always @* begin
        reg        w_en, w_agu;          // the dsp_write_reg port
        reg  [5:0] w_num;
        reg [23:0] w_val;
        reg  [1:0] npop;                 // pops before the register write
        reg        push_j, push_do;      // pushes after the register write
        reg        jt;                   // jump taken (cur_inst_len = 0, pc = jtgt)
        reg [15:0] jtgt, len, hd, seq_pc, seq_pc1;
        reg [23:0] lc_i;
        reg        rep_dec, lc_eq1, do_hit, do_back, eq_f1, eq_f2;
        reg  [6:0] t7;
        reg        bitv, skip, found, stay;
        reg [31:0] bv, bm, rdv;
        reg [23:0] v24, xy0;
        reg [55:0] d56, s56, r56;
        reg  [5:0] nr;
        reg  [4:0] vec;
        reg  [1:0] ipl, lvl;
        reg        hst_on, ssi_on, p_hc, p_hr, p_ht, p_sr, p_st;
        reg  [3:0] ki;

        n_x0 = x0; n_x1 = x1; n_y0 = y0; n_y1 = y1;
        n_a0 = a0; n_a1 = a1; n_a2 = a2; n_b0 = b0; n_b1 = b1; n_b2 = b2;
        for (i = 0; i < 8; i = i + 1) begin n_rr[i] = rr[i]; n_nn[i] = nn[i]; n_mm[i] = mm[i]; end
        n_sr = sr_l; n_omr = omr; n_sp = sp; n_la = la; n_lc = lc; n_lcsave = lcsave;
        n_loop_rep = loop_rep; n_pc_on_rep = pc_on_rep;
        n_int_state = int_state; n_int_cnt = int_cnt; n_int_fetch = int_fetch;
        n_int_save_pc = int_save_pc; n_int_ipl = int_ipl;
        n_st_trace = st_trace; n_st_swi = st_swi; n_st_illegal = st_illegal; n_st_stkerr = st_stkerr;
        n_hc_ack = 1'b0; n_wait = 1'b0; n_stop = 1'b0;
        n_agu_reg = 6'd0; n_agu_val = agu_val;
        wa_h = 1'b0; wa_l = 1'b0; wb_h = 1'b0; wc_h = 1'b0;
        wa_i = 4'd0; wb_i = 4'd0; wc_i = 4'd0;
        wa_hv = 16'd0; wa_lv = 16'd0; wb_hv = 16'd0; wb_lv = 16'd0; wc_hv = 16'd0; wc_lv = 16'd0;
        w_en = 1'b0; w_agu = 1'b0; w_num = 6'd0; w_val = 24'd0;
        npop = 2'd0; push_j = 1'b0; push_do = 1'b0;
        jt = 1'b0; jtgt = 16'd0; hd = 16'd0;
        t7 = 7'd0; bitv = 1'b0; skip = 1'b0; found = 1'b0; stay = 1'b0;
        bv = 32'd0; bm = 32'd0; rdv = 32'd0; v24 = 24'd0; xy0 = 24'd0;
        d56 = 56'd0; s56 = 56'd0; r56 = 56'd0;
        nr = 6'd0; vec = 5'd0; ipl = 2'd0; lvl = 2'd0;
        hst_on = 1'b0; ssi_on = 1'b0; p_hc = 1'b0; p_hr = 1'b0; p_ht = 1'b0; p_sr = 1'b0; p_st = 1'b0;
        ki = 4'd0;
        len = (dq_ea_m6 || dq_do) ? 16'd2 : 16'd1;
        n_pc = pc; n_pc1 = pc_p1;
        seq_pc = pc; seq_pc1 = pc_p1; lc_i = lc; rep_dec = 1'b0; lc_eq1 = 1'b0;
        do_hit = 1'b0; do_back = 1'b0; eq_f1 = 1'b0; eq_f2 = 1'b0;

        // ================= instruction (Hatari handler) =================
        if (dq_ispm) begin
            // data ALU first, then the parallel move writes
            n_sr = alu_sr;
            if (alu_wr) begin
                if (alu_dst_b) begin n_b2 = alu_res[55:48]; n_b1 = alu_res[47:24]; n_b0 = alu_res[23:0]; end
                else           begin n_a2 = alu_res[55:48]; n_a1 = alu_res[47:24]; n_a0 = alu_res[23:0]; end
            end
            if (dq_alu_und) len = 16'd0;
            case (dq_pk)
                PK_RR: begin w_en = 1'b1; w_num = {1'b0, ir_q[12:8]}; w_val = sval_q; w_agu = 1'b1; end
                PK_IMM: begin
                    w_en = 1'b1; w_agu = 1'b1;
                    w_num = {1'b0, ir_q[20:16]};
                    w_val = {16'd0, ir_q[15:8]};
                    if (w_num == RG_X0 || w_num == RG_X1 || w_num == RG_Y0 || w_num == RG_Y1 ||
                        w_num == RG_A || w_num == RG_B)
                        w_val = {ir_q[15:8], 16'd0};
                end
                PK_0: begin
                    xy0 = ir_q[15] ? y0 : x0;
                    if (ir_q[16]) begin n_b0 = 24'd0; n_b1 = xy0; n_b2 = {8{xy0[23]}}; end
                    else          begin n_a0 = 24'd0; n_a1 = xy0; n_a2 = {8{xy0[23]}}; end
                end
                PK_1: begin
                    // S2 (A/B limited) -> D2 (Y0/Y1 for X: moves, X0/X1 for Y:)
                    v24 = (ir_q[14] ? ir_q[19] : ir_q[17]) ? lim_b[23:0] : lim_a[23:0];
                    if (ir_q[14]) begin if (ir_q[18]) n_x1 = v24; else n_x0 = v24; end
                    else          begin if (ir_q[16]) n_y1 = v24; else n_y0 = v24; end
                  if (ir_q[15]) begin
                    w_en = 1'b1;
                    w_val = dq_ea_imm ? ew_q : rd1;
                    if (ir_q[14]) w_num = (ir_q[17:16] == 2'd0) ? RG_Y0 : (ir_q[17:16] == 2'd1) ? RG_Y1 :
                                          (ir_q[17:16] == 2'd2) ? RG_A : RG_B;
                    else          w_num = (ir_q[19:18] == 2'd0) ? RG_X0 : (ir_q[19:18] == 2'd1) ? RG_X1 :
                                          (ir_q[19:18] == 2'd2) ? RG_A : RG_B;
                  end
                end
                PK_L: if (ir_q[15]) begin
                    case ({ir_q[19], ir_q[17:16]})
                        3'd0: begin n_a1 = rd1; n_a0 = rd2; end
                        3'd1: begin n_b1 = rd1; n_b0 = rd2; end
                        3'd2: begin n_x1 = rd1; n_x0 = rd2; end
                        3'd3: begin n_y1 = rd1; n_y0 = rd2; end
                        3'd4: begin n_a0 = rd2; n_a1 = rd1; n_a2 = {8{rd1[23]}}; end
                        3'd5: begin n_b0 = rd2; n_b1 = rd1; n_b2 = {8{rd1[23]}}; end
                        3'd6: begin n_a0 = 24'd0; n_a1 = rd1; n_a2 = {8{rd1[23]}};
                                    n_b0 = 24'd0; n_b1 = rd2; n_b2 = {8{rd2[23]}}; end
                        default: begin n_b0 = 24'd0; n_b1 = rd1; n_b2 = {8{rd1[23]}};
                                       n_a0 = 24'd0; n_a1 = rd2; n_a2 = {8{rd2[23]}}; end
                    endcase
                end
                PK_XY: if (ir_q[15]) begin
                    w_en = 1'b1; w_agu = 1'b1;
                    w_num = {1'b0, ir_q[21:20], ir_q[18:16]};
                    w_val = dq_ea_imm ? ew_q : rd1;
                end
                PK_XYXY: begin
                    if (ir_q[15]) case (ir_q[19:18])
                        2'd0: n_x0 = rd1;
                        2'd1: n_x1 = rd1;
                        2'd2: begin n_a0 = 24'd0; n_a1 = rd1; n_a2 = {8{rd1[23]}}; end
                        default: begin n_b0 = 24'd0; n_b1 = rd1; n_b2 = {8{rd1[23]}}; end
                    endcase
                    if (ir_q[22]) case (ir_q[17:16])
                        2'd0: n_y0 = rd2;
                        2'd1: n_y1 = rd2;
                        2'd2: begin n_a0 = 24'd0; n_a1 = rd2; n_a2 = {8{rd2[23]}}; end
                        default: begin n_b0 = 24'd0; n_b1 = rd2; n_b2 = {8{rd2[23]}}; end
                    endcase
                end
                default: ;
            endcase
        end else begin
            case (dq_cls)
                C_OPCODE8H_0: begin
                    case (ir_q)
                        24'h000000: ;                                   // nop
                        24'h000004: begin npop = 2'd1; jt = 1'b1; end   // rti
                        24'h000005: n_st_illegal = 1'b1;                // illegal
                        24'h000006: n_st_swi = 1'b1;                    // swi (hardware)
                        24'h00000c: begin npop = 2'd1; jt = 1'b1; end   // rts
                        24'h000084: ;                                   // reset (peripherals)
                        24'h000086: n_wait = 1'b1;                      // wait
                        24'h000087: n_stop = 1'b1;                      // stop
                        24'h00008c: npop = 2'd2;                        // enddo
                        default: len = 16'd0;                           // undefined
                    endcase
                end
                C_UNDEFINED: len = 16'd0;
                C_ANDI: case (ir_q[1:0])
                    2'd0: n_sr = n_sr & {ir_q[15:8], 8'hff};
                    2'd1: n_sr = n_sr & {8'hff, ir_q[15:8]};
                    2'd2: n_omr = n_omr & ir_q[15:8];
                    default: ;
                endcase
                C_ORI: case (ir_q[1:0])
                    2'd0: n_sr = n_sr | {ir_q[15:8], 8'h00};
                    2'd1: n_sr = n_sr | {8'h00, ir_q[15:8]};
                    2'd2: n_omr = n_omr | ir_q[15:8];
                    default: ;
                endcase
                C_DIV: begin
                    case (ir_q[5:4])
                        2'd0: v24 = x0; 2'd1: v24 = y0; 2'd2: v24 = x1; default: v24 = y1;
                    endcase
                    s56 = {{8{v24[23]}}, v24, 24'd0};
                    d56 = ir_q[3] ? acc_b : acc_a;
                    r56 = {d56[54:0], 1'b0};
                    if (d56[55] ^ v24[23]) r56 = r56 + s56;
                    else                   r56 = r56 - s56;
                    r56[0] = r56[0] | n_sr[0];
                    if (ir_q[3]) begin n_b2 = r56[55:48]; n_b1 = r56[47:24]; n_b0 = r56[23:0]; end
                    else         begin n_a2 = r56[55:48]; n_a1 = r56[47:24]; n_a0 = r56[23:0]; end
                    n_sr[0] = ~r56[55];
                    n_sr[1] = d56[55] ^ d56[54];
                    if (d56[55] ^ d56[54]) n_sr[6] = 1'b1;
                end
                C_NORM: begin
                    d56 = ir_q[3] ? acc_b : acc_a;
                    r56 = d56;
                    hd = 16'd0;   // newsr
                    if (!n_sr[5] && n_sr[4] && !n_sr[2]) begin
                        r56 = {d56[54:0], 1'b0};
                        hd[0] = d56[55];
                        hd[1] = d56[55] ^ d56[54];
                        hd[6] = d56[55] ^ d56[54];
                        n_rr[ir_q[10:8]] = rr[ir_q[10:8]] - 16'd1;
                    end else if (n_sr[5]) begin
                        r56 = {d56[55], d56[55:1]};
                        hd[0] = d56[0];
                        n_rr[ir_q[10:8]] = rr[ir_q[10:8]] + 16'd1;
                    end
                    if (ir_q[3]) begin n_b2 = r56[55:48]; n_b1 = r56[47:24]; n_b0 = r56[23:0]; end
                    else         begin n_a2 = r56[55:48]; n_a1 = r56[47:24]; n_a0 = r56[23:0]; end
                    n_sr = euzn_c(n_sr, r56);
                    n_sr = (n_sr & ~16'h0003) | hd;
                end
                C_TCC: begin
                    if (cc_true(ir_q[15:12], n_sr)) begin
                        case (ir_q[6:3])
                            4'd0:  begin d56 = acc_b; nr = RG_A; end
                            4'd1:  begin d56 = acc_a; nr = RG_B; end
                            4'd8:  begin d56 = {{8{x0[23]}}, x0, 24'd0}; nr = RG_A; end
                            4'd9:  begin d56 = {{8{x0[23]}}, x0, 24'd0}; nr = RG_B; end
                            4'd10: begin d56 = {{8{y0[23]}}, y0, 24'd0}; nr = RG_A; end
                            4'd11: begin d56 = {{8{y0[23]}}, y0, 24'd0}; nr = RG_B; end
                            4'd12: begin d56 = {{8{x1[23]}}, x1, 24'd0}; nr = RG_A; end
                            4'd13: begin d56 = {{8{x1[23]}}, x1, 24'd0}; nr = RG_B; end
                            4'd14: begin d56 = {{8{y1[23]}}, y1, 24'd0}; nr = RG_A; end
                            4'd15: begin d56 = {{8{y1[23]}}, y1, 24'd0}; nr = RG_B; end
                            default: begin d56 = 56'd0; nr = RG_B; end  // NULL source/dest
                        endcase
                        if (nr == RG_A) begin n_a2 = d56[55:48]; n_a1 = d56[47:24]; n_a0 = d56[23:0]; end
                        else            begin n_b2 = d56[55:48]; n_b1 = d56[47:24]; n_b0 = d56[23:0]; end
                        if (ir_q[16]) begin
                            w_en = 1'b1; w_agu = 1'b1;
                            w_num = {3'b010, ir_q[2:0]}; w_val = {8'd0, rr[ir_q[10:8]]};
                        end
                    end
                end
                C_LUA: begin
                    w_en = 1'b1; w_agu = 1'b1;
                    w_num = ir_q[3] ? {3'b011, ir_q[2:0]} : {3'b010, ir_q[2:0]};
                    w_val = {8'd0, lua_q};
                end
                C_MOVEC_REG: begin
                    w_en = 1'b1; w_agu = 1'b1;
                    if (ir_q[15]) begin w_num = ir_q[5:0]; w_val = sval_q; end
                    else begin
                        w_num = ir_q[13:8];
                        if (ir_q[5:0] == RG_SSH) begin npop = 2'd1; w_val = {8'd0, ssh_cur}; end
                        else w_val = sval_q;
                    end
                end
                C_MOVEC_AA, C_MOVEC_EA: begin
                    if (ir_q[15]) begin
                        w_en = 1'b1; w_agu = 1'b1; w_num = ir_q[5:0];
                        w_val = (dq_cls == C_MOVEC_EA && dq_ea_imm) ? ew_q : rd1;
                    end else if (ir_q[5:0] == RG_SSH) npop = 2'd1;
                end
                C_MOVEC_IMM: begin
                    w_en = 1'b1; w_agu = 1'b1; w_num = ir_q[5:0]; w_val = {16'd0, ir_q[15:8]};
                end
                C_MOVEM_AA, C_MOVEM_EA: begin
                    if (ir_q[15]) begin w_en = 1'b1; w_num = ir_q[5:0]; w_val = rd1; end
                    else if (ir_q[5:0] == RG_SSH) npop = 2'd1;
                end
                C_MOVEP_0: begin
                    if (ir_q[15]) begin
                        if (ir_q[13:8] == RG_SSH) npop = 2'd1;
                    end else begin
                        w_en = 1'b1; w_agu = 1'b1; w_num = ir_q[13:8]; w_val = rd1;
                    end
                end
                C_BCHG_AA, C_BCHG_EA, C_BCHG_PP, C_BCLR_AA, C_BCLR_EA, C_BCLR_PP,
                C_BSET_AA, C_BSET_EA, C_BSET_PP, C_BTST_AA, C_BTST_EA, C_BTST_PP: begin
                    bv = {8'd0, rd1};
                    n_sr[0] = bv[ir_q[4:0]];
                end
                C_BCHG_REG, C_BCLR_REG, C_BSET_REG, C_BTST_REG: begin
                    bv = sval_q32;
                    bm = 32'd1 << ir_q[4:0];
                    case (dq_bitop)
                        2'd0: bv = bv & ~bm;
                        2'd1: bv = bv | bm;
                        default: bv = bv ^ bm;
                    endcase
                    if (dq_bitop != 2'd3) begin w_en = 1'b1; w_num = ir_q[13:8]; w_val = bv[23:0]; end
                end
                C_JCLR_AA, C_JCLR_EA, C_JCLR_PP, C_JCLR_REG, C_JSET_AA, C_JSET_EA, C_JSET_PP, C_JSET_REG,
                C_JSCLR_AA, C_JSCLR_EA, C_JSCLR_PP, C_JSCLR_REG, C_JSSET_AA, C_JSSET_EA, C_JSSET_PP, C_JSSET_REG: begin
                    bv = {8'd0, dq_jbrg ? sval_q : rd1};
                    bitv = bv[ir_q[4:0]];
                    if (dq_jb_set ? bitv : !bitv) begin
                        push_j = dq_jb_sub;
                        jt = 1'b1; jtgt = ew_q[15:0];
                    end else
                        len = 16'd2;
                end
                C_JMP_IMM: begin jt = 1'b1; jtgt = {4'd0, ir_q[11:0]}; end
                C_JMP_EA:  begin jt = 1'b1; jtgt = ea1_addr_q; end
                C_JCC_IMM: if (cc_true(ir_q[15:12], n_sr)) begin jt = 1'b1; jtgt = {4'd0, ir_q[11:0]}; end
                C_JCC_EA:  if (cc_true(ir_q[3:0], n_sr)) begin jt = 1'b1; jtgt = ea1_addr_q; end
                C_JSCC_IMM: if (cc_true(ir_q[15:12], n_sr)) begin
                    push_j = 1'b1; jt = 1'b1; jtgt = {4'd0, ir_q[11:0]};
                end
                C_JSCC_EA: if (cc_true(ir_q[3:0], n_sr)) begin
                    push_j = 1'b1; jt = 1'b1; jtgt = ea1_addr_q;
                end
                C_JSR_IMM, C_JSR_EA: begin
                    if (int_state != IS_LONG) push_j = 1'b1;
                    else n_int_state = IS_DISABLED;
                    jt = 1'b1;
                    jtgt = (dq_cls == C_JSR_IMM) ? {4'd0, ir_q[11:0]} : ea1_addr_q;
                end
                C_DO_AA, C_DO_EA, C_DO_IMM, C_DO_REG: push_do = 1'b1;
                C_REP_AA, C_REP_EA, C_REP_IMM, C_REP_REG: begin
                    n_lcsave = lc;
                    n_pc_on_rep = 1'b1;
                    n_loop_rep = 1'b1;
                    case (dq_cls)
                        C_REP_AA, C_REP_EA: n_lc = rd1;
                        C_REP_IMM: n_lc = {12'd0, ir_q[3:0], ir_q[15:8]};
                        default: n_lc = {8'd0, sval_q[15:0]};
                    endcase
                end
                default: ;   // movep_1, movep_23: memory only
            endcase
        end

        // ================= stack, register write port =================
        // pops: RTI, RTS, ENDDO, register reads of SSH (dsp_stack_pop)
        if (npop != 2'd0) begin
            t7 = sp_pop(n_sp); n_sp = t7[5:0]; if (t7[6]) n_st_stkerr = 1'b1;
        end
        if (npop == 2'd2) begin
            t7 = sp_pop(n_sp); n_sp = t7[5:0]; if (t7[6]) n_st_stkerr = 1'b1;
        end
        if (!dq_ispm && dq_cls == C_OPCODE8H_0) begin
            if (ir_q == 24'h000004) begin jtgt = ssh_cur; n_sr = ssl_cur; end      // rti
            if (ir_q == 24'h00000c) jtgt = ssh_cur;                                // rts
            if (ir_q == 24'h00008c) begin                                          // enddo
                n_sr = (n_sr & 16'h007f) | (ssl_cur & 16'h8000);
                n_la = stk_h[sp_m1]; n_lc = {8'd0, stk_l[sp_m1]};
            end
        end
        // dsp_write_reg
        if (w_en) begin
            case (w_num)
                RG_A: begin n_a0 = 24'd0; n_a1 = w_val; n_a2 = {8{w_val[23]}}; end
                RG_B: begin n_b0 = 24'd0; n_b1 = w_val; n_b2 = {8{w_val[23]}}; end
                RG_X0: n_x0 = w_val;  RG_X1: n_x1 = w_val;  RG_Y0: n_y0 = w_val;  RG_Y1: n_y1 = w_val;
                RG_A0: n_a0 = w_val;  RG_B0: n_b0 = w_val;  RG_A1: n_a1 = w_val;  RG_B1: n_b1 = w_val;
                RG_A2: n_a2 = w_val[7:0];  RG_B2: n_b2 = w_val[7:0];
                6'h10, 6'h11, 6'h12, 6'h13, 6'h14, 6'h15, 6'h16, 6'h17: begin
                    if (w_agu) begin n_agu_reg = w_num; n_agu_val = n_rr[w_num[2:0]]; end
                    n_rr[w_num[2:0]] = w_val[15:0];
                end
                6'h18, 6'h19, 6'h1a, 6'h1b, 6'h1c, 6'h1d, 6'h1e, 6'h1f: begin
                    if (w_agu) begin n_agu_reg = w_num; n_agu_val = n_nn[w_num[2:0]]; end
                    n_nn[w_num[2:0]] = w_val[15:0];
                end
                6'h20, 6'h21, 6'h22, 6'h23, 6'h24, 6'h25, 6'h26, 6'h27: begin
                    if (w_agu) begin n_agu_reg = w_num; n_agu_val = n_mm[w_num[2:0]]; end
                    n_mm[w_num[2:0]] = w_val[15:0];
                end
                RG_LCSAVE: n_lcsave = 24'd0;
                RG_OMR: n_omr = w_val[7:0] & 8'hc7;
                RG_SR:  n_sr = w_val[15:0] & 16'haf7f;
                RG_SP: begin
                    if (n_sp[5:4] == 2'b00 && w_val[5:4] != 2'b00) begin
                        n_st_stkerr = 1'b1;
                        n_sp = {w_val[5:4], 4'd0};
                    end else
                        n_sp = w_val[5:0];
                end
                RG_SSH: begin   // push, SSH only
                    t7 = sp_push(n_sp); n_sp = t7[5:0]; if (t7[6]) n_st_stkerr = 1'b1;
                    wa_i = n_sp[3:0]; wa_h = (n_sp[3:0] != 4'd0); wa_hv = w_val[15:0];
                end
                RG_SSL: begin
                    wa_i = n_sp[3:0]; wa_l = (n_sp[3:0] != 4'd0); wa_lv = w_val[15:0];
                end
                RG_LA: n_la = w_val[15:0];
                RG_LC: n_lc = {8'd0, w_val[15:0]};
                default: ;
            endcase
        end
        // bit operations on registers set C after the register write
        if (!dq_ispm && dq_bitrg) n_sr[0] = sval_q_bit;
        // pushes: JSR, JScc, JSCLR, JSSET
        if (push_j) begin
            t7 = sp_push(n_sp); n_sp = t7[5:0]; if (t7[6]) n_st_stkerr = 1'b1;
            wa_i = n_sp[3:0]; wa_h = (n_sp[3:0] != 4'd0); wa_l = wa_h;
            wa_hv = dq_jb ? (pc + 16'd2) : (pc + len); wa_lv = n_sr;
        end
        // DO: push (LA, LC), LA = P:pc+1, [DO reg: LC], push (pc+2, SR), LF
        if (push_do) begin
            t7 = sp_push(n_sp); n_sp = t7[5:0]; if (t7[6]) n_st_stkerr = 1'b1;
            wa_i = n_sp[3:0]; wa_h = (n_sp[3:0] != 4'd0); wa_l = wa_h;
            wa_hv = la; wa_lv = lc[15:0];
            n_la = ew_q[15:0];
            case (ir_q[13:8])   // registers[] read after the first push (DO reg)
                RG_SP:  v24 = {18'd0, sp_push_sp[5:0]};   // DO has no earlier stack op
                RG_SSH: v24 = wa_h ? {8'd0, la} : 24'd0;
                RG_SSL: v24 = wa_h ? {8'd0, lc[15:0]} : 24'd0;
                RG_LA:  v24 = {8'd0, ew_q[15:0]};
                default: v24 = sval_q;
            endcase
            if (dq_cls == C_DO_REG) n_lc = {8'd0, v24[15:0]};
            t7 = sp_push(n_sp); n_sp = t7[5:0]; if (t7[6]) n_st_stkerr = 1'b1;
            wb_i = n_sp[3:0]; wb_h = (n_sp[3:0] != 4'd0);
            wb_hv = pc + 16'd2; wb_lv = n_sr;
            n_sr[15] = 1'b1;
            case (dq_cls)
                C_DO_AA, C_DO_EA: n_lc = {8'd0, rd1[15:0]};
                C_DO_IMM: n_lc = {12'd0, ir_q[3:0], ir_q[15:8]};
                default: ;
            endcase
        end

        // ================= dsp_postexecute_update_pc =================
        // (written for timing: the LC tests use the instruction's LC value,
        // both fetch addresses come from parallel candidates)
        lc_i = n_lc;
        rep_dec = n_loop_rep && !n_pc_on_rep;
        if (n_loop_rep) begin
            if (!n_pc_on_rep) begin
                stay = (lc_i[15:0] != 16'd1);           // (LC - 1) & $ffff != 0
                if (stay) n_lc = {8'd0, lc_i[15:0] - 16'd1};
                else begin n_loop_rep = 1'b0; n_lc = n_lcsave; end
            end else begin
                if (lc_i == 24'd0) n_lc = 24'h010000;
                n_pc_on_rep = 1'b0;
            end
        end
        lc_eq1 = rep_dec ? (stay ? (lc_i[15:0] == 16'd2) : (n_lcsave == 24'd1)) : (lc_i == 24'd1);
        if (jt)        begin seq_pc = jtgt;  seq_pc1 = jtgt + 16'd1; end
        else if (stay) begin seq_pc = pc;    seq_pc1 = pc_p1; end
        else if (len == 16'd2) begin seq_pc = pc_p2; seq_pc1 = pc_p3; end
        else if (len == 16'd1) begin seq_pc = pc_p1; seq_pc1 = pc_p2; end
        else           begin seq_pc = pc;    seq_pc1 = pc_p1; end
        n_pc = seq_pc; n_pc1 = seq_pc1;
        // DO loop end
        ki = n_sp[3:0];
        rdv = stk_rd(ki, wa_h, wa_l, wa_i, wa_hv, wa_lv, wb_h, wb_i, wb_hv, wb_lv,
                     stk_h[ki], stk_l[ki]);
        do_hit = n_sr[15] && ({1'b0, seq_pc} == {1'b0, n_la} + 17'd1);
        do_back = 1'b0;
        if (do_hit) begin
            if (lc_eq1) begin
                t7 = sp_pop(n_sp); n_sp = t7[5:0]; if (t7[6]) n_st_stkerr = 1'b1;
                n_sr = (n_sr & 16'h7fff) | (rdv[15:0] & 16'h8000);
                ki = n_sp[3:0];
                rdv = stk_rd(ki, wa_h, wa_l, wa_i, wa_hv, wa_lv, wb_h, wb_i, wb_hv, wb_lv,
                             stk_h[ki], stk_l[ki]);
                t7 = sp_pop(n_sp); n_sp = t7[5:0]; if (t7[6]) n_st_stkerr = 1'b1;
                n_la = rdv[31:16]; n_lc = {8'd0, rdv[15:0]};
            end else begin
                n_lc = {8'd0, n_lc[15:0] - 16'd1};
                n_pc = rdv[31:16]; n_pc1 = rdv[31:16] + 16'd1;
                do_back = 1'b1;
            end
        end
        eq_f1 = do_back ? ({1'b0, rdv[31:16]} == if_p1) : ({1'b0, seq_pc} == if_p1);
        eq_f2 = do_back ? ({1'b0, rdv[31:16]} == if_p2) : ({1'b0, seq_pc} == if_p2);

        // ================= dsp_postexecute_interrupts =================
        if (!n_loop_rep) begin
            if (n_int_state == IS_DISABLED) begin
                skip = 1'b1;
                case (n_int_cnt)
                    3'd5: n_int_cnt = 3'd4;
                    3'd4: begin
                        n_int_save_pc = n_pc;
                        n_pc = int_fetch; n_pc1 = if_p1[15:0];
                        if (is_long_insn(vec_word)) begin
                            n_int_state = IS_LONG;
                            t7 = sp_push(n_sp); n_sp = t7[5:0]; if (t7[6]) n_st_stkerr = 1'b1;
                            wc_i = n_sp[3:0]; wc_h = (n_sp[3:0] != 4'd0);
                            wc_hv = n_int_save_pc; wc_lv = n_sr;
                            n_sr = (n_sr & 16'h50ff) | {6'd0, n_int_ipl, 8'd0};
                        end
                        n_int_cnt = 3'd3;
                    end
                    3'd3: begin
                        if (eq_f1) begin
                            if (is_long_insn(vec_word)) begin
                                n_int_state = IS_LONG;
                                t7 = sp_push(n_sp); n_sp = t7[5:0]; if (t7[6]) n_st_stkerr = 1'b1;
                                wc_i = n_sp[3:0]; wc_h = (n_sp[3:0] != 4'd0);
                                wc_hv = n_int_save_pc; wc_lv = n_sr;
                                n_sr = (n_sr & 16'h50ff) | {6'd0, n_int_ipl, 8'd0};
                            end
                            n_int_cnt = 3'd2;
                        end else begin
                            if (eq_f2) begin n_pc = int_save_pc; n_pc1 = isp_p1; end
                            n_int_cnt = 3'd1;
                        end
                    end
                    3'd2: begin
                        if (eq_f2) begin n_pc = int_save_pc; n_pc1 = isp_p1; end
                        n_int_cnt = 3'd1;
                    end
                    3'd1: n_int_cnt = 3'd0;
                    default: begin
                        n_int_save_pc = 16'hffff;
                        n_int_fetch = 16'hffff;
                        n_int_state = IS_NONE;
                        skip = 1'b0;
                    end
                endcase
            end
            if (!skip) begin
                // NMI class: illegal, stack error, trace, swi (always taken)
                if (n_st_illegal | n_st_stkerr | n_st_trace | n_st_swi) begin
                    found = 1'b1; ipl = 2'd3;
                    if (n_st_illegal)      begin vec = 5'd31; n_st_illegal = 1'b0; end
                    else if (n_st_stkerr)  begin vec = 5'd1;  n_st_stkerr = 1'b0; end
                    else if (n_st_trace)   begin vec = 5'd2;  n_st_trace = 1'b0; end
                    else                   begin vec = 5'd3;  n_st_swi = 1'b0; end
                end else begin
                    // host and SSI interrupts (enabled by IPR, masked by HCR/CRB)
                    p_hc = int_host_cmd & msk_host_cmd;
                    p_hr = int_host_rcv & msk_host_rcv;
                    p_ht = int_host_trx & msk_host_trx;
                    p_sr = int_ssi_rcv & msk_ssi_rcv;
                    p_st = int_ssi_trx & msk_ssi_trx;
                    hst_on = (ipr[11:10] != 2'd0) && (p_hc | p_hr | p_ht);
                    ssi_on = (ipr[13:12] != 2'd0) && (p_sr | p_st);
                    // highest level first, down to the SR interrupt mask
                    if (n_sr[9:8] <= 2'd2) begin
                        if (hst_on && ipr[11:10] == 2'd3) begin found = 1'b1; lvl = 2'd2; end
                        else if (ssi_on && ipr[13:12] == 2'd3) begin found = 1'b1; lvl = 2'd2; end
                    end
                    if (!found && n_sr[9:8] <= 2'd1) begin
                        if (hst_on && ipr[11:10] == 2'd2) begin found = 1'b1; lvl = 2'd1; end
                        else if (ssi_on && ipr[13:12] == 2'd2) begin found = 1'b1; lvl = 2'd1; end
                    end
                    if (!found && n_sr[9:8] == 2'd0) begin
                        if (hst_on && ipr[11:10] == 2'd1) begin found = 1'b1; lvl = 2'd0; end
                        else if (ssi_on && ipr[13:12] == 2'd1) begin found = 1'b1; lvl = 2'd0; end
                    end
                    if (found) begin
                        ipl = lvl + 2'd1;
                        if (hst_on && ipr[11:10] == lvl + 2'd1) begin
                            if (p_hc) begin vec = hc_vector; n_hc_ack = 1'b1; end
                            else if (p_hr) vec = 5'd16;
                            else vec = 5'd17;
                        end else begin
                            if (p_sr) vec = 5'd6;
                            else vec = 5'd8;
                        end
                    end
                end
                if (found) begin
                    n_int_ipl = ipl;
                    n_int_cnt = 3'd5;
                    n_int_state = IS_DISABLED;
                    n_int_fetch = {10'd0, vec, 1'b0};
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // memory port control
    // ------------------------------------------------------------------
    wire        fetch_now = el || (state == ST_FETCH);
    wire [15:0] fetch_pc  = (state == ST_FETCH) ? pc : n_pc;
    wire [15:0] fetch_pc1 = (state == ST_FETCH) ? pc_p1 : n_pc1;
    wire        do1 = t_iss && p_a1_en;
    wire        do2 = t_iss && p_a2_en;
    wire        dow = t_wr2 && aw_en_q;

    always @* begin
        pint_we_a = 1'b0; pint_addr_a = 9'd0; pint_d_a = 24'd0;
        pint_we_b = 1'b0; pint_addr_b = 9'd0; pint_d_b = 24'd0;
        ext_we_a = 1'b0; ext_addr_a = 15'd0; ext_d_a = 24'd0;
        ext_we_b = 1'b0; ext_addr_b = 15'd0; ext_d_b = 24'd0;
        xint_we = 1'b0; xint_addr = 8'd0; xint_d = 24'd0;
        yint_we = 1'b0; yint_addr = 8'd0; yint_d = 24'd0;
        xrom_en = 1'b0; xrom_addr = 8'd0; yrom_en = 1'b0; yrom_addr = 8'd0;
        perx_rd = 1'b0; perx_wr = 1'b0; perx_addr = 6'd0; perx_wdata = 24'd0;
        pery_rd = 1'b0; pery_wr = 1'b0; pery_addr = 6'd0; pery_wdata = 24'd0;

        if (fetch_now) begin
            pint_addr_a = fetch_pc[8:0];
            ext_addr_a  = fetch_pc[14:0];
            pint_addr_b = fetch_pc1[8:0];
            ext_addr_b  = fetch_pc1[14:0];
        end else begin
            if (vec_rd) pint_addr_b = vec_ad[8:0];
            if (do1) begin
                case (p_r1)
                    RG_INT: if (p_a1_sp == 2'd1) begin yint_addr = p_a1_ad[7:0]; yint_we = p_a1_we; yint_d = p_a1_wd; end
                            else begin xint_addr = p_a1_ad[7:0]; xint_we = p_a1_we; xint_d = p_a1_wd; end
                    RG_ROM: if (p_a1_sp == 2'd1) begin yrom_en = 1'b1; yrom_addr = p_a1_ad[7:0]; end
                            else begin xrom_en = 1'b1; xrom_addr = p_a1_ad[7:0]; end
                    RG_PER: if (p_a1_sp == 2'd1) begin pery_addr = p_a1_ad[5:0]; pery_rd = !p_a1_we; pery_wr = p_a1_we; pery_wdata = p_a1_wd; end
                            else begin perx_addr = p_a1_ad[5:0]; perx_rd = !p_a1_we; perx_wr = p_a1_we; perx_wdata = p_a1_wd; end
                    RG_PINT: begin pint_addr_a = p_a1_ad[8:0]; pint_we_a = p_a1_we; pint_d_a = p_a1_wd; end
                    RG_EXT: if (p_a1_sp == 2'd1) begin ext_addr_b = ext_addr(2'd1, p_a1_ad); ext_we_b = p_a1_we; ext_d_b = p_a1_wd; end
                            else begin ext_addr_a = ext_addr(p_a1_sp, p_a1_ad); ext_we_a = p_a1_we; ext_d_a = p_a1_wd; end
                    default: ;
                endcase
            end
            if (do2) begin
                case (p_r2)
                    RG_INT: begin yint_addr = p_a2_ad[7:0]; yint_we = p_a2_we; yint_d = p_a2_wd; end
                    RG_ROM: begin yrom_en = 1'b1; yrom_addr = p_a2_ad[7:0]; end
                    RG_PER: begin pery_addr = p_a2_ad[5:0]; pery_rd = !p_a2_we; pery_wr = p_a2_we; pery_wdata = p_a2_wd; end
                    RG_EXT: begin ext_addr_b = ext_addr(2'd1, p_a2_ad); ext_we_b = p_a2_we; ext_d_b = p_a2_wd; end
                    default: ;
                endcase
            end
            if (dow) begin
                case (rw_q)
                    RG_INT: if (aw_sp_q == 2'd1) begin yint_addr = aw_ad_q[7:0]; yint_we = 1'b1; yint_d = aw_wd; end
                            else begin xint_addr = aw_ad_q[7:0]; xint_we = 1'b1; xint_d = aw_wd; end
                    RG_PER: if (aw_sp_q == 2'd1) begin pery_addr = aw_ad_q[5:0]; pery_wr = 1'b1; pery_wdata = aw_wd; end
                            else begin perx_addr = aw_ad_q[5:0]; perx_wr = 1'b1; perx_wdata = aw_wd; end
                    RG_PINT: begin pint_addr_a = aw_ad_q[8:0]; pint_we_a = 1'b1; pint_d_a = aw_wd; end
                    RG_EXT: if (aw_sp_q == 2'd1) begin ext_addr_b = ext_addr(2'd1, aw_ad_q); ext_we_b = 1'b1; ext_d_b = aw_wd; end
                            else begin ext_addr_a = ext_addr(aw_sp_q, aw_ad_q); ext_we_a = 1'b1; ext_d_a = aw_wd; end
                    default: ;
                endcase
            end
        end
        if (boot_we) begin
            pint_we_b = 1'b1; pint_addr_b = boot_addr; pint_d_b = boot_data;
        end
    end

    // ------------------------------------------------------------------
    // sequential part
    // ------------------------------------------------------------------
    wire h_pend = (int_host_cmd & msk_host_cmd) | (int_host_rcv & msk_host_rcv) | (int_host_trx & msk_host_trx);
    wire s_pend = (int_ssi_rcv & msk_ssi_rcv) | (int_ssi_trx & msk_ssi_trx);
    wire wait_release = st_illegal | st_stkerr | st_trace | st_swi |
        ((sr[9:8] <= 2'd2) && (((ipr[11:10] == 2'd3) && h_pend) || ((ipr[13:12] == 2'd3) && s_pend))) ||
        ((sr[9:8] <= 2'd1) && (((ipr[11:10] == 2'd2) && h_pend) || ((ipr[13:12] == 2'd2) && s_pend))) ||
        ((sr[9:8] == 2'd0) && (((ipr[11:10] == 2'd1) && h_pend) || ((ipr[13:12] == 2'd1) && s_pend)));

    always @(posedge clk) begin
        per_soft_reset <= 1'b0;
        retire <= 1'b0;
        t_wr2 <= t_iss;

        if (rst) begin
            x0 <= 24'd0; x1 <= 24'd0; y0 <= 24'd0; y1 <= 24'd0;
            a0 <= 24'd0; a1 <= 24'd0; a2 <= 8'd0; b0 <= 24'd0; b1 <= 24'd0; b2 <= 8'd0;
            for (i = 0; i < 8; i = i + 1) begin rr[i] <= 16'd0; nn[i] <= 16'd0; mm[i] <= 16'hffff; end
            for (i = 0; i < 16; i = i + 1) begin stk_h[i] <= 16'd0; stk_l[i] <= 16'd0; end
            sr <= 16'd0; omr <= 8'h02; sp <= 6'd0; la <= 16'd0; lc <= 24'd0; lcsave <= 24'd0;
            pc <= 16'd0;
            loop_rep <= 1'b0; pc_on_rep <= 1'b0;
            int_state <= IS_NONE; int_cnt <= 3'd0; int_fetch <= 16'hffff; int_save_pc <= 16'hffff;
            int_ipl <= 2'd0;
            agu_reg <= 6'd0; agu_val <= 16'd0;
            st_trace <= 1'b0; st_swi <= 1'b0; st_illegal <= 1'b0; st_stkerr <= 1'b0;
            state <= ST_IDLE; cyc <= 7'd0; ncyc <= 7'd2;
            delay_q <= 1'b0; t_wr2 <= 1'b0; aw_en_q <= 1'b0;
            fa_ext <= 1'b0; fb_ext <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (run_start) begin
                        rr[0] <= {6'd0, boot_pos};
                        omr <= 8'h02;
                    end
                    if (run) state <= ST_FETCH;
                end
                ST_FETCH: begin
                    fa_ext <= pc[15:9] != 7'd0;
                    fb_ext <= fetch_pc1[15:9] != 7'd0;
                    state <= ST_EXEC;
                    cyc <= 7'd0;
                end
                ST_WAIT: begin
                    if (wait_release) state <= ST_FETCH;
                end
                ST_STOP: ;
                default: begin   // ST_EXEC
                    cyc <= cyc + 7'd1;
                    if (e1) begin
                        // Hatari: trace interrupt raised before executing
                        if (sr[13]) st_trace <= 1'b1;
                        ir_q <= ir1;
                        ew_q <= ew1;
                        ncyc <= cyc_total;
                        cyc_base_q <= cyc_base;
                        ext_p0_q <= ext_p0;
                        delay_q <= d1_acc_delay;
                        ea1_addr_q <= agu1_addr;
                        lua_q <= agu1_newr;      // Hatari LUA srcnew
                        lhit_q <= lhit;
                        sval_q <= sval;
                        if (agu1_commit) rr[d1_ea1_rn] <= agu1_newr;
                        if (agu2_commit) rr[d1_ea2_rn] <= agu2_newr;
                        if (!d1_ispm && d1_cls == C_OPCODE8H_0 && ir1 == 24'h000084) per_soft_reset <= 1'b1;
                        // access plan for a delayed issue
                        q_a1_en <= a1_en; q_a1_we <= a1_we; q_a1_sp <= a1_sp; q_a1_ad <= a1_ad; q_a1_wd <= a1_wd;
                        q_a2_en <= a2_en; q_a2_we <= a2_we; q_a2_ad <= a2_ad; q_a2_wd <= a2_wd;
                        q_aw_en <= aw_en; q_aw_sp <= aw_sp; q_aw_ad <= aw_ad;
                        q_r1 <= r1_c; q_r2 <= r2_c; q_rw <= rw_c1;
                    end
                    if (iss_dly) ncyc <= cyc_total_dly;
                    if (t_iss) begin
                        r1_q <= (p_a1_en && !p_a1_we) ? p_r1 : RG_IGN;
                        r2_q <= (p_a2_en && !p_a2_we) ? p_r2 : RG_IGN;
                        a1_sp_q <= p_a1_sp;
                        aw_en_q <= p_aw_en;
                        aw_sp_q <= p_aw_sp;
                        aw_ad_q <= p_aw_ad;
                        rw_q <= p_rw;
                    end
                    if (t_wr2) begin
                        rd1_q <= rd1_c;
                        rd2_q <= rd2_c;
                        aw_en_q <= 1'b0;
                    end
                    if (el) begin
                        retire <= 1'b1;
                        x0 <= n_x0; x1 <= n_x1; y0 <= n_y0; y1 <= n_y1;
                        a0 <= n_a0; a1 <= n_a1; a2 <= n_a2; b0 <= n_b0; b1 <= n_b1; b2 <= n_b2;
                        for (i = 0; i < 8; i = i + 1) begin rr[i] <= n_rr[i]; nn[i] <= n_nn[i]; mm[i] <= n_mm[i]; end
                        if (wa_h) stk_h[wa_i] <= wa_hv;
                        if (wa_l) stk_l[wa_i] <= wa_lv;
                        if (wb_h) begin stk_h[wb_i] <= wb_hv; stk_l[wb_i] <= wb_lv; end
                        if (wc_h) begin stk_h[wc_i] <= wc_hv; stk_l[wc_i] <= wc_lv; end
                        sr <= n_sr; omr <= n_omr; sp <= n_sp; la <= n_la; lc <= n_lc; lcsave <= n_lcsave;
                        pc <= n_pc;
                        loop_rep <= n_loop_rep; pc_on_rep <= n_pc_on_rep;
                        int_state <= n_int_state; int_cnt <= n_int_cnt; int_fetch <= n_int_fetch;
                        int_save_pc <= n_int_save_pc; int_ipl <= n_int_ipl;
                        agu_reg <= n_agu_reg; agu_val <= n_agu_val;
                        st_trace <= n_st_trace; st_swi <= n_st_swi; st_illegal <= n_st_illegal;
                        st_stkerr <= n_st_stkerr;
                        fa_ext <= n_pc[15:9] != 7'd0;
                        fb_ext <= fetch_pc1[15:9] != 7'd0;
                        cyc <= 7'd0;
                        delay_q <= 1'b0;
                        if (n_stop) state <= ST_STOP;
                        else if (n_wait && n_int_state != IS_DISABLED) state <= ST_WAIT;
                    end
                end
            endcase
        end
    end
endmodule
