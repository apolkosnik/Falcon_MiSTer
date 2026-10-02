// DSP56001 instruction decoder for the Falcon DSP (falcon_dsp).
// Combinational; dsp56k_core instantiates it twice: on the freshly fetched
// opcode (first instruction clock) and on the registered opcode (later
// clocks), so the two never form one long timing path.
//
// Follows Hatari src/falcon/dsp_cpu.c dsp56k_execute_instruction():
// opcodes < $100000 dispatch on opcodes8h[{bits 19:14, bits 7:5}] (table
// generated from Hatari into dsp56k_tables.svh), others are parallel moves
// opcodes_parmove[bits 23:20] with dsp_pm_2 / dsp_pm_4 sub-decoding.

module dsp56k_dec (
    input      [23:0] ir,
    output     [6:0]  cls,       // class (C_PM_0 for every parallel move)
    output            ispm,      // parallel move instruction (with data ALU op)
    output reg [3:0]  pk,        // parallel move kind
    output            alu_und,   // undefined data ALU opcode
    output reg        ea1_used,
    output reg [2:0]  ea1_mode,
    output     [2:0]  ea1_rn,
    output reg        ea2_used,
    output reg [2:0]  ea2_mode,
    output     [2:0]  ea2_rn,
    output            ea6,       // dsp_calc_ea with the 6-bit mode field
    output            ea_m6,     // ... absolute / immediate (reads P:pc+1)
    output            ea_imm,    // ... immediate (calc_ea returns 1)
    output            ea_xtra,   // +2 cycles (modes 5, 6, 7)
    output            acc_delay, // accesses issued one clock later (modes 5, 7)
    output            c_bitaa, c_bitea, c_bitpp, c_bitrg,
    output            c_jbaa, c_jbea, c_jbpp, c_jbrg,
    output            c_bit, c_jb, c_bwr, c_do, c_rep, c_movem,
    output     [1:0]  bitop,     // 0 clr 1 set 2 chg 3 tst
    output            jb_set,    // jset/jsset (else jclr/jsclr)
    output            jb_sub     // jsclr/jsset
);
`include "dsp56k_tables.svh"

    localparam [3:0] PK_NONE = 4'd0, PK_RUPD = 4'd1, PK_RR = 4'd2, PK_IMM = 4'd3,
                     PK_L = 4'd4, PK_XY = 4'd5, PK_XYXY = 4'd6, PK_0 = 4'd7, PK_1 = 4'd8;

    wire       par_hi = (ir[23:20] != 4'd0);
    wire [6:0] cls8   = dsp_class8h({ir[19:14], ir[7:5]});
    assign cls  = par_hi ? C_PM_0 : cls8;
    assign ispm = par_hi || (cls8 == C_PM_0);

    always @* begin
        pk = PK_NONE;
        if (!par_hi) pk = PK_0;
        else case (ir[23:20])
            4'd1: pk = PK_1;
            4'd2: begin
                if (ir[19:8] == 12'h000) pk = PK_NONE;
                else if (ir[19:13] == 7'b0000010) pk = PK_RUPD;
                else if (ir[19:18] == 2'b00) pk = PK_RR;
                else pk = PK_IMM;
            end
            4'd3: pk = PK_IMM;
            4'd4: pk = (ir[18] == 1'b0) ? PK_L : PK_XY;   // (inst & 0xf40000) == 0x400000
            4'd5, 4'd6, 4'd7: pk = PK_XY;
            default: pk = PK_XYXY;
        endcase
    end
    assign alu_und = ispm && alu_undefined(ir[7:0]);

    assign c_bitaa = (cls == C_BCHG_AA) || (cls == C_BCLR_AA) || (cls == C_BSET_AA) || (cls == C_BTST_AA);
    assign c_bitea = (cls == C_BCHG_EA) || (cls == C_BCLR_EA) || (cls == C_BSET_EA) || (cls == C_BTST_EA);
    assign c_bitpp = (cls == C_BCHG_PP) || (cls == C_BCLR_PP) || (cls == C_BSET_PP) || (cls == C_BTST_PP);
    assign c_bitrg = (cls == C_BCHG_REG) || (cls == C_BCLR_REG) || (cls == C_BSET_REG) || (cls == C_BTST_REG);
    assign c_jbaa  = (cls == C_JCLR_AA) || (cls == C_JSET_AA) || (cls == C_JSCLR_AA) || (cls == C_JSSET_AA);
    assign c_jbea  = (cls == C_JCLR_EA) || (cls == C_JSET_EA) || (cls == C_JSCLR_EA) || (cls == C_JSSET_EA);
    assign c_jbpp  = (cls == C_JCLR_PP) || (cls == C_JSET_PP) || (cls == C_JSCLR_PP) || (cls == C_JSSET_PP);
    assign c_jbrg  = (cls == C_JCLR_REG) || (cls == C_JSET_REG) || (cls == C_JSCLR_REG) || (cls == C_JSSET_REG);
    assign c_bit   = c_bitaa | c_bitea | c_bitpp | c_bitrg;
    assign c_jb    = c_jbaa | c_jbea | c_jbpp | c_jbrg;
    assign c_bwr   = (c_bitaa | c_bitea | c_bitpp) && bitop != 2'd3;
    assign c_do    = (cls == C_DO_AA) || (cls == C_DO_EA) || (cls == C_DO_IMM) || (cls == C_DO_REG);
    assign c_rep   = (cls == C_REP_AA) || (cls == C_REP_EA) || (cls == C_REP_IMM) || (cls == C_REP_REG);
    assign c_movem = (cls == C_MOVEM_AA) || (cls == C_MOVEM_EA);
    // bit / jump-on-bit variants from the opcode bits (Hatari table layout)
    assign bitop   = {ir[16], ir[5]} == 2'b00 ? 2'd0 :     // bclr
                     {ir[16], ir[5]} == 2'b01 ? 2'd1 :     // bset
                     {ir[16], ir[5]} == 2'b10 ? 2'd2 : 2'd3; // bchg / btst
    assign jb_set  = ir[5];
    assign jb_sub  = ir[16];

    wire e6  = c_bitea | c_jbea | (cls == C_JCC_EA) | (cls == C_JMP_EA) | (cls == C_JSCC_EA) |
               (cls == C_JSR_EA) | (cls == C_MOVEC_EA) | (cls == C_MOVEM_EA) | (cls == C_MOVEP_1) |
               (cls == C_MOVEP_23) | (cls == C_DO_EA) | (cls == C_REP_EA) |
               (ispm && (pk == PK_0 || pk == PK_1 || ((pk == PK_L || pk == PK_XY) && ir[14])));
    wire e5  = (cls == C_LUA) || (ispm && pk == PK_RUPD);
    wire exy = ispm && pk == PK_XYXY;
    assign ea6 = e6;

    assign ea1_rn = ir[10:8];
    assign ea2_rn = {~ir[10], ir[14:13]};
    always @* begin
        ea1_used = 1'b0; ea1_mode = 3'd4;
        ea2_used = 1'b0; ea2_mode = 3'd4;
        if (e6) begin
            ea1_used = 1'b1; ea1_mode = ir[13:11];
        end else if (e5) begin
            ea1_used = 1'b1; ea1_mode = {1'b0, ir[12:11]};
        end else if (exy) begin
            ea1_used = 1'b1; ea1_mode = (ir[12:11] == 2'd0) ? 3'd4 : {1'b0, ir[12:11]};
            ea2_used = 1'b1; ea2_mode = (ir[21:20] == 2'd0) ? 3'd4 : {1'b0, ir[21:20]};
        end
    end
    assign ea_m6     = e6 && ir[13:11] == 3'd6;
    assign ea_imm    = ea_m6 && ir[10:8] != 3'd0;
    assign ea_xtra   = e6 && (ir[13:11] == 3'd5 || ir[13:11] == 3'd6 || ir[13:11] == 3'd7);
    assign acc_delay = e6 && (ir[13:11] == 3'd5 || ir[13:11] == 3'd7);
endmodule
