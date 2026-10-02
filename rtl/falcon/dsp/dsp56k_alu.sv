// DSP56001 data ALU for the Falcon DSP (falcon_dsp).
//
// Follows Hatari src/falcon/dsp_cpu.c: opcodes_alu[] (dsp_abs_a ... dsp_tst_b),
// dsp_abs56, dsp_asl56, dsp_asr56, dsp_add56, dsp_sub56, dsp_mul56,
// dsp_rnd56 and dsp_ccr_update_e_u_n_z, bit for bit, including Hatari's
// flag quirks (they are what the differential test compares against):
//   - ADC/SBC are two chained 56-bit operations, C and V are ORed;
//   - ADDL/SUBL OR the C/V of the ASL step into the result flags, ADDR/SUBR
//     OR the C of the ASR step;
//   - MAC/MACR/MPY/MPYR never touch C; RND touches only E,U,N,Z;
//   - ROR sets N from the bit shifted out; LSR clears N;
//   - with scaling mode S1:S0 = 11 the E,U,N,Z bits are cleared and not set;
//   - rounding is Hatari's dsp_rnd56 (convergent rounding variants per
//     scaling mode, S0 checked before S1).
//
// Timing: the operand preparation (selection, ASL/ASR/ABS pre-shifts, the
// 24x24 product, one DSP block) is registered on the first clock of an
// instruction (pre_en); the main add/subtract (the SIGN_MINUS multiplies
// subtract the product), the second step (ADC/SBC carry or rounding
// constant, one shared adder) and the flags are combinational in the last
// clock of the instruction.  Registers read by the ALU never change between
// those two clocks, so this is equivalent to Hatari's single-step code.

module dsp56k_alu (
    input             clk,
    input             pre_en,      // first clock of the instruction
    input       [7:0] op,          // opcode bits 7:0 (valid when pre_en)
    input      [55:0] a,
    input      [55:0] b,
    input      [23:0] x0,
    input      [23:0] x1,
    input      [23:0] y0,
    input      [23:0] y1,
    input      [15:0] sr,          // SR at the last clock (C, S1:S0 used)
    output reg        wr_d,        // write the destination accumulator
    output            dst_b,       // destination is B
    output reg [55:0] res,         // new destination accumulator value
    output reg [15:0] sr_out       // SR with the ALU flag updates applied
);

    // ------------------------------------------------------------------
    // helpers
    // ------------------------------------------------------------------
    function automatic [55:0] abs56(input [55:0] d);
        abs56 = d[55] ? (56'd0 - d) : d;
    endfunction

    function automatic [55:0] sext24(input [23:0] v);
        sext24 = {{8{v[23]}}, v, 24'd0};
    endfunction

    // dsp_ccr_update_e_u_n_z
    function automatic [15:0] euzn(input [15:0] s, input [55:0] r);
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
            euzn = o;
        end
    endfunction

    // dsp_rnd56: rounding constant per scaling mode (S0 checked first) and
    // the masking applied to the sum t = d + constant
    function automatic [55:0] rnd_const(input [15:0] s);
        if (s[10])      rnd_const = 56'h00000001000000;
        else if (s[11]) rnd_const = 56'h00000000400000;
        else            rnd_const = 56'h00000000800000;
    endfunction
    function automatic [55:0] rnd_mask(input [55:0] t_in, input [15:0] s);
        reg [55:0] t;
        begin
            t = t_in;
            if (s[10]) begin
                if (t[23:0] == 24'd0 && t[24] == 1'b0) t[25] = 1'b0;
                t[24] = 1'b0;
                t[23:0] = 24'd0;
            end else if (s[11]) begin
                if (t[22:0] == 23'd0) t[23:0] = 24'd0;
                else t[23:0] = {t[23], 23'd0};
            end else begin
                if (t[23:0] == 24'd0) t[24] = 1'b0;
                t[23:0] = 24'd0;
            end
            rnd_mask = t;
        end
    endfunction

    // ------------------------------------------------------------------
    // pre-stage (first clock)
    // ------------------------------------------------------------------
    wire [55:0] d_in = op[3] ? b : a;
    wire [55:0] o_in = op[3] ? a : b;
    reg  [23:0] r24;
    always @* begin
        case (op[5:4])
            2'd0: r24 = x0;
            2'd1: r24 = y0;
            2'd2: r24 = x1;
            default: r24 = y1;
        endcase
    end

    // product sources: Hatari opcodes_alu 0x80..0xff rows
    reg [23:0] m1, m2;
    always @* begin
        case (op[6:4])
            3'd0: begin m1 = x0; m2 = x0; end
            3'd1: begin m1 = y0; m2 = y0; end
            3'd2: begin m1 = x1; m2 = x0; end
            3'd3: begin m1 = y1; m2 = y0; end
            3'd4: begin m1 = x0; m2 = y1; end
            3'd5: begin m1 = y0; m2 = x0; end
            3'd6: begin m1 = x1; m2 = y0; end
            default: begin m1 = y1; m2 = x1; end
        endcase
    end
    // dsp_mul56: exact signed product * 2; the SIGN_MINUS forms subtract it
    // (D - P, 0 - P) instead of adding its negation (same result and V flag)
    wire signed [47:0] prod_s = $signed(m1) * $signed(m2);
    wire        [55:0] prod   = {{7{prod_s[47]}}, prod_s, 1'b0};

    wire [55:0] x_full = {{8{x1[23]}}, x1, x0};
    wire [55:0] y_full = {{8{y1[23]}}, y1, y0};

    reg  [55:0] dpre, spre;
    reg         c_pre, v_pre;
    reg         sub_pre;
    always @* begin
        dpre = d_in;
        spre = 56'd0;
        c_pre = 1'b0;
        v_pre = 1'b0;
        sub_pre = 1'b0;
        if (op[7]) begin
            dpre = op[1] ? d_in : 56'd0;     // MAC adds to D, MPY starts at 0
            spre = prod;
            sub_pre = op[2];
        end else if (op[6]) begin
            spre = sext24(r24);
            case (op[2:0])
                3'd4, 3'd5: sub_pre = 1'b1;              // sub, cmp
                3'd7: begin                               // cmpm
                    dpre = abs56(d_in);
                    spre = abs56(sext24(r24));
                    sub_pre = 1'b1;
                end
                default: ;
            endcase
        end else begin
            case (op[5:4])
                2'd0: spre = o_in;
                2'd1: spre = o_in;
                2'd2: spre = x_full;
                default: spre = y_full;
            endcase
            case ({op[5:4], op[2:0]})
                5'b00_010: begin                          // addr
                    dpre = {d_in[55], d_in[55:1]}; c_pre = d_in[0];
                end
                5'b00_110: begin                          // subr
                    dpre = {d_in[55], d_in[55:1]}; c_pre = d_in[0]; sub_pre = 1'b1;
                end
                5'b00_101: sub_pre = 1'b1;                // cmp
                5'b00_111: begin                          // cmpm
                    dpre = abs56(d_in); spre = abs56(o_in); sub_pre = 1'b1;
                end
                5'b01_010: begin                          // addl
                    dpre = {d_in[54:0], 1'b0}; c_pre = d_in[55]; v_pre = d_in[55] ^ d_in[54];
                end
                5'b01_100: sub_pre = 1'b1;                // sub
                5'b01_110: begin                          // subl
                    dpre = {d_in[54:0], 1'b0}; c_pre = d_in[55]; v_pre = d_in[55] ^ d_in[54];
                    sub_pre = 1'b1;
                end
                5'b01_001: spre = 56'd0;                  // rnd: D + 0
                5'b10_100, 5'b11_100: sub_pre = 1'b1;     // sub x / sub y
                5'b10_101, 5'b11_101: sub_pre = 1'b1;     // sbc
                5'b10_010: begin                          // asr
                    dpre = {d_in[55], d_in[55:1]}; c_pre = d_in[0]; spre = 56'd0;
                end
                5'b11_010: begin                          // asl
                    dpre = {d_in[54:0], 1'b0}; c_pre = d_in[55]; v_pre = d_in[55] ^ d_in[54];
                    spre = 56'd0;
                end
                5'b10_110: begin                          // abs
                    dpre = abs56(d_in); spre = 56'd0;
                end
                5'b11_110: begin                          // neg: 0 - D
                    dpre = 56'd0; spre = d_in; sub_pre = 1'b1;
                end
                default: ;
            endcase
        end
    end

    reg  [7:0]  op_q;
    reg  [55:0] d_q, dpre_q, spre_q;
    reg  [23:0] r24_q;
    reg         c_pre_q, v_pre_q, sub_q;
    always @(posedge clk) begin
        if (pre_en) begin
            op_q    <= op;
            d_q     <= d_in;
            dpre_q  <= dpre;
            spre_q  <= spre;
            r24_q   <= r24;
            c_pre_q <= c_pre;
            v_pre_q <= v_pre;
            sub_q   <= sub_pre;
        end
    end

    assign dst_b = op_q[3];

    // ------------------------------------------------------------------
    // post-stage (last clock)
    // ------------------------------------------------------------------
    wire [56:0] main57 = sub_q ? ({1'b0, dpre_q} - {1'b0, spre_q})
                               : ({1'b0, dpre_q} + {1'b0, spre_q});
    wire [55:0] mr  = main57[55:0];
    wire        c_m = main57[56];
    wire        v_m = sub_q ? ((spre_q[55] ^ dpre_q[55]) & (mr[55] ^ dpre_q[55]))
                            : ((spre_q[55] ^ mr[55]) & (dpre_q[55] ^ mr[55]));

    // second step, one adder: ADC (+1), SBC (-1) or rounding constant
    wire        is_sbc = !op_q[7] && !op_q[6] && op_q[5] && op_q[2:0] == 3'd5;
    wire        is_adc = !op_q[7] && !op_q[6] && op_q[5] && op_q[2:0] == 3'd1;
    wire [55:0] addend = is_sbc ? {56{1'b1}} : is_adc ? 56'd1 : rnd_const(sr);
    wire [56:0] sec57  = {1'b0, mr} + {1'b0, addend};
    wire [55:0] inc_r  = sec57[55:0];
    wire [55:0] dec_r  = sec57[55:0];
    wire        c_inc  = sec57[56];
    wire        v_inc  = inc_r[55] & ~mr[55];
    wire        c_dec  = ~sec57[56];
    wire        v_dec  = mr[55] & ~dec_r[55];
    wire [55:0] rnd_r  = rnd_mask(sec57[55:0], sr);

    wire        d_ovf = (d_q == 56'h80000000000000);
    wire [23:0] d1 = d_q[47:24];

    reg  [23:0] l24;
    always @* begin
        wr_d   = 1'b0;
        res    = d_q;
        sr_out = sr;
        l24    = d1;
        if (op_q[7]) begin
            // mpy / mpyr / mac / macr
            wr_d = 1'b1;
            res  = op_q[0] ? rnd_r : mr;
            sr_out = euzn(sr, res);
            if (op_q[1]) begin
                sr_out[1] = v_m;
                if (v_m) sr_out[6] = 1'b1;
            end else begin
                sr_out[1] = 1'b0;
            end
        end else if (op_q[6]) begin
            case (op_q[2:0])
                3'd0, 3'd4: begin                         // add / sub
                    wr_d = 1'b1; res = mr;
                    sr_out = euzn(sr, mr);
                    sr_out[0] = c_m; sr_out[1] = v_m; if (v_m) sr_out[6] = 1'b1;
                end
                3'd1: begin                               // tfr
                    wr_d = 1'b1; res = spre_q;
                end
                3'd5, 3'd7: begin                         // cmp / cmpm
                    sr_out = euzn(sr, mr);
                    sr_out[0] = c_m; sr_out[1] = v_m; if (v_m) sr_out[6] = 1'b1;
                end
                default: begin                            // or / eor / and
                    wr_d = 1'b1;
                    case (op_q[2:0])
                        3'd2: l24 = d1 | r24_q;
                        3'd3: l24 = d1 ^ r24_q;
                        default: l24 = d1 & r24_q;
                    endcase
                    res = {d_q[55:48], l24, d_q[23:0]};
                    sr_out[3] = l24[23];
                    sr_out[2] = (l24 == 24'd0);
                    sr_out[1] = 1'b0;
                end
            endcase
        end else begin
            case ({op_q[5:4], op_q[2:0]})
                5'b00_000: ;                              // move / undefined
                5'b00_001: begin wr_d = 1'b1; res = spre_q; end   // tfr acc
                5'b00_010, 5'b00_110: begin               // addr / subr
                    wr_d = 1'b1; res = mr;
                    sr_out = euzn(sr, mr);
                    sr_out[0] = c_pre_q | c_m; sr_out[1] = v_m; if (v_m) sr_out[6] = 1'b1;
                end
                5'b00_011: begin                          // tst
                    sr_out = euzn(sr, d_q);
                    sr_out[1] = 1'b0;
                end
                5'b00_100: ;                              // undefined
                5'b00_101, 5'b00_111: begin               // cmp / cmpm
                    sr_out = euzn(sr, mr);
                    sr_out[0] = c_m; sr_out[1] = v_m; if (v_m) sr_out[6] = 1'b1;
                end
                5'b01_000, 5'b01_100,                     // add / sub acc
                5'b10_000, 5'b10_100,                     // add / sub x
                5'b11_000, 5'b11_100: begin               // add / sub y
                    wr_d = 1'b1; res = mr;
                    sr_out = euzn(sr, mr);
                    sr_out[0] = c_m; sr_out[1] = v_m; if (v_m) sr_out[6] = 1'b1;
                end
                5'b01_001: begin                          // rnd
                    wr_d = 1'b1; res = rnd_r;
                    sr_out = euzn(sr, rnd_r);
                end
                5'b01_010, 5'b01_110: begin               // addl / subl
                    wr_d = 1'b1; res = mr;
                    sr_out = euzn(sr, mr);
                    sr_out[0] = c_pre_q | c_m;
                    sr_out[1] = v_pre_q | v_m;
                    if (v_pre_q | v_m) sr_out[6] = 1'b1;
                end
                5'b01_011: begin                          // clr
                    wr_d = 1'b1; res = 56'd0;
                    sr_out[5] = 1'b0; sr_out[3] = 1'b0; sr_out[1] = 1'b0;
                    sr_out[4] = 1'b1; sr_out[2] = 1'b1;
                end
                5'b01_101: ;                              // undefined
                5'b01_111: begin                          // not
                    wr_d = 1'b1; l24 = ~d1;
                    res = {d_q[55:48], l24, d_q[23:0]};
                    sr_out[3] = l24[23]; sr_out[2] = (l24 == 24'd0); sr_out[1] = 1'b0;
                end
                5'b10_001, 5'b11_001: begin               // adc
                    wr_d = 1'b1;
                    if (sr[0]) begin
                        res = inc_r;
                        sr_out = euzn(sr, inc_r);
                        sr_out[0] = c_m | c_inc; sr_out[1] = v_m | v_inc;
                        if (v_m | v_inc) sr_out[6] = 1'b1;
                    end else begin
                        res = mr;
                        sr_out = euzn(sr, mr);
                        sr_out[0] = c_m; sr_out[1] = v_m; if (v_m) sr_out[6] = 1'b1;
                    end
                end
                5'b10_101, 5'b11_101: begin               // sbc
                    wr_d = 1'b1;
                    if (sr[0]) begin
                        res = dec_r;
                        sr_out = euzn(sr, dec_r);
                        sr_out[0] = c_m | c_dec; sr_out[1] = v_m | v_dec;
                        if (v_m | v_dec) sr_out[6] = 1'b1;
                    end else begin
                        res = mr;
                        sr_out = euzn(sr, mr);
                        sr_out[0] = c_m; sr_out[1] = v_m; if (v_m) sr_out[6] = 1'b1;
                    end
                end
                5'b10_010: begin                          // asr
                    wr_d = 1'b1; res = dpre_q;
                    sr_out = euzn(sr, dpre_q);
                    sr_out[0] = c_pre_q; sr_out[1] = 1'b0;
                end
                5'b11_010: begin                          // asl
                    wr_d = 1'b1; res = dpre_q;
                    sr_out = euzn(sr, dpre_q);
                    sr_out[0] = c_pre_q; sr_out[1] = v_pre_q; if (v_pre_q) sr_out[6] = 1'b1;
                end
                5'b10_011: begin                          // lsr
                    wr_d = 1'b1; l24 = {1'b0, d1[23:1]};
                    res = {d_q[55:48], l24, d_q[23:0]};
                    sr_out[0] = d1[0]; sr_out[3] = 1'b0; sr_out[2] = (l24 == 24'd0); sr_out[1] = 1'b0;
                end
                5'b11_011: begin                          // lsl
                    wr_d = 1'b1; l24 = {d1[22:0], 1'b0};
                    res = {d_q[55:48], l24, d_q[23:0]};
                    sr_out[0] = d1[23]; sr_out[3] = l24[23]; sr_out[2] = (l24 == 24'd0); sr_out[1] = 1'b0;
                end
                5'b10_111: begin                          // ror
                    wr_d = 1'b1; l24 = {sr[0], d1[23:1]};
                    res = {d_q[55:48], l24, d_q[23:0]};
                    sr_out[0] = d1[0]; sr_out[3] = d1[0]; sr_out[2] = (l24 == 24'd0); sr_out[1] = 1'b0;
                end
                5'b11_111: begin                          // rol
                    wr_d = 1'b1; l24 = {d1[22:0], sr[0]};
                    res = {d_q[55:48], l24, d_q[23:0]};
                    sr_out[0] = d1[23]; sr_out[3] = l24[23]; sr_out[2] = (l24 == 24'd0); sr_out[1] = 1'b0;
                end
                5'b10_110: begin                          // abs
                    wr_d = 1'b1; res = dpre_q;
                    sr_out[1] = d_ovf; if (d_ovf) sr_out[6] = 1'b1;
                    sr_out = euzn(sr_out, dpre_q);
                end
                5'b11_110: begin                          // neg
                    wr_d = 1'b1; res = mr;
                    sr_out[1] = d_ovf; if (d_ovf) sr_out[6] = 1'b1;
                    sr_out = euzn(sr_out, mr);
                end
                default: ;
            endcase
        end
    end
endmodule
