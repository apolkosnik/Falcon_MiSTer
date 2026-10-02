// falcon_psg.sv - YM2149 sound generator of the Atari Falcon030
//
// Behavioural reference: Hatari
//   src/psg.c   : PSG_Reset, PSG_Set_SelectRegister, PSG_Get_DataRegister,
//                 PSG_Set_DataRegister, PSG_ff8800_ReadByte,
//                 PSG_ff880x_ReadByte, PSG_ff8801_WriteByte,
//                 PSG_ff8802_WriteByte, PSG_ff8803_WriteByte
//   src/sound.c : Sound_WriteReg, Ym2149_Reset, YM2149_DoSamples_250,
//                 YM2149_RndCompute, YM2149_EnvBuild (YmEnvDef),
//                 YmVolume4to5, interpolate_volumetable +
//                 YM2149_Normalise_5bit_Table (YM_TABLE_MIXING, the default),
//                 YM2149_BuildLinearVolumeTable (YM_LINEAR_MIXING),
//                 PWMaliasFilter (YM2149_LPF_FILTER_PWM, the default)
//
// Register bus (docs/ARCHITECTURE.md), bus_addr[7:1] = address bits 7..1
// inside $FF8800-$FF88FF.  Only address bit 1 reaches the YM (BC1/BDIR):
//   $FF8800 read  : contents of the selected register (see below)
//   $FF8800 write : register select (all 8 bits kept; >= 16 selects nothing)
//   $FF8802 write : data write into the selected register
//   $FF8801/2/3 read : $FF
//   $FF8801/$FF8803 write : shadow of $FF8800/$FF8802, only for byte accesses
//                  (bus_lds without bus_uds, as with move.b or movep); in a
//                  word access only the even byte acts (psg.c 2008/12/21).
// Register read data (psg.c PSGRegisterReadData): after a select, the
// register value with its unused bits cleared; after a data write without a
// new select, the full 8-bit value written.  Write masks: regs 1,3,5,13
// keep bits 3..0; regs 6,8,9,10 keep bits 4..0; others keep 8 bits.
// bus_ack is given on the bus_stb clock (no wait states).
//
// MIRROR = 1 (default, as in docs/ARCHITECTURE.md): $FF8804-$FF88FF mirror
//   $FF8800-$FF8803 every 4 bytes, as on the ST/STE/TT.
// MIRROR = 0: Hatari's Falcon behaviour (ioMem.c): $FF8804-$FF88FF are not
//   mirrored; in STE bus compatible mode they read $FF and ignore writes
//   (this module does that), in Falcon bus mode they bus error (the
//   decoder would have to do that, it is not done here).
//
// YM clock: 2 MHz derived from CLK_HZ by a fractional accumulator.  As in
// Hatari, all counters advance at 2 MHz / 8 = 250 kHz ("tick"), the noise
// counter at 125 kHz.  Each tick:
//   tone:  count++ ; if count >= period { count = 0 ; out ^= 1 }   (per 0 == 1)
//   noise: every 2nd tick count++ ; on every tick if count >= period
//          { count = 0 ; out = LFSR step }  (17-bit LFSR, taps 17/14, seed 1)
//   env:   count++ ; if count >= period { count = 0 ; pos++ ; 96 -> 32 }
//   voice level (5 bit) = ((tone|mix_t) & (noise|mix_n)) ? vol5 : 0, where
//   vol5 = envelope volume (YmEnvDef shapes, 32 steps) or YmVolume4to5[vol].
// Audio outputs:
//   ch_a/ch_b/ch_c [4:0] : the 5-bit voice levels, updated on every tick.
//   snd_sample signed [15:0] : the mixed sample = Hatari ymout5[C][B][A]
//        (range 0..32767, not centred, exactly as Hatari's YM_Buffer_250),
//        followed by Hatari's PWMaliasFilter when PWM_FILTER = 1.
//   snd_stb : one clk pulse when snd_sample is new: once per 250 kHz tick,
//        a few clocks after the tick (fixed latency: 3 clocks for
//        VOL_TABLE = 1, 4 clocks for VOL_TABLE = 0).
//   Hatari's later steps (resampling to the host rate and the
//   Subsonic_IIR_HPF DC filter applied for the Falcon in
//   Sound_GenerateSamples) are not done here; the crossbar / audio output
//   stage owns them.
// VOL_TABLE = 1: Hatari's default YM_TABLE_MIXING table (32768 x 15 bit ROM
//   from falcon_psg_vol.hex, generated from Hatari's own code by
//   tb/psg/gen_vol_hex.c), 48 M10K blocks.
// VOL_TABLE = 0: Hatari's YM_LINEAR_MIXING ((v[A]+v[B]+v[C])/3, scaled to
//   0..32767), no block RAM, one multiplier.
//
// I/O ports (register 7 bit 6 = port A direction, bit 7 = port B
// direction, 1 = output):
//   port_a_out / port_b_out: register 14 / 15 when the port is an output,
//   $FF (pull-ups) when it is an input.  port_a_oe / port_b_oe = direction.
//   Reading register 14/15 while the port is an input returns port_a_in /
//   port_b_in (datasheet); Hatari always returns the register latch.
// Falcon port A use (Hatari psg.c, PSG_Set_DataRegister and the pin list):
//   bit 0 floppy side select (0 = side 1, i.e. inverted)
//   bit 1 floppy drive 0 (A) select, 0 = selected
//   bit 2 floppy drive 1 (B) select, 0 = selected
//   bit 3 ST: RS232 RTS; Falcon: Centronics SELIN (pin 17) per Hatari
//   bit 4 ST: RS232 DTR; Falcon: DSP reset (Hatari resets the DSP when set)
//   bit 5 Centronics STROBE
//   bit 6 Falcon internal speaker control (Hatari: not emulated)
//   bit 7 Falcon IDE reset (Hatari: not emulated); not the floppy density
//         (that is $FF860E on the Falcon)
// Port B: Centronics data D0..D7.
//
// Reset (synchronous): all registers 0 except register 14 = $FF
// (PSG_Reset), select = 0, counters 0, LFSR seed 1.
//
// Deviations from Hatari:
//  - Hatari's Sound_Reset (after PSG_Reset) loads 0xFF into the sound
//    core's mixer while the readable register 7 stays 0.  This module has
//    one register 7 and follows the YM2149 datasheet: reset value 0 (all
//    tones and noises enabled; the volumes are 0, so the output is silent).
//  - Port reads in input direction return the input pins (see above).
//  - No CPU wait states (Hatari's PSG_WaitState); the bus has its own.
//  - The PWM filter state is cleared by reset (Hatari never clears it).
module falcon_psg #(
    parameter int    CLK_HZ     = 32000000,
    parameter int    YM_HZ      = 2000000,
    parameter bit    MIRROR     = 1'b1,
    parameter bit    VOL_TABLE  = 1'b1,
    parameter bit    PWM_FILTER = 1'b1,
    parameter        VOL_HEX    = "rtl/falcon/falcon_psg_vol.hex"
) (
    input             clk,
    input             reset,

    input             bus_cs,
    input             bus_stb,
    input             bus_we,
    input      [7:1]  bus_addr,
    input             bus_uds,
    input             bus_lds,
    input      [15:0] bus_din,
    output     [15:0] bus_dout,
    output            bus_ack,

    input       [7:0] port_a_in,
    input       [7:0] port_b_in,
    output      [7:0] port_a_out,
    output      [7:0] port_b_out,
    output            port_a_oe,
    output            port_b_oe,

    output reg  [4:0] ch_a,
    output reg  [4:0] ch_b,
    output reg  [4:0] ch_c,
    output reg signed [15:0] snd_sample,
    output reg        snd_stb
);

// ---------------------------------------------------------------------------
// Register interface
// ---------------------------------------------------------------------------
reg  [7:0] regs [0:15];
reg  [7:0] sel;
reg  [7:0] read_data;

wire void_area  = !MIRROR && (bus_addr[7:2] != 6'd0);
wire is_data    = bus_addr[1];

// Byte that acts on a write: even byte if UDS (word or even byte access),
// odd byte only for an odd byte access (shadow registers $FF8801/$FF8803).
wire       wr_act  = bus_stb && bus_we && !void_area && (bus_uds || bus_lds);
wire [7:0] wr_byte = bus_uds ? bus_din[15:8] : bus_din[7:0];
wire       wr_sel  = wr_act && !is_data;
wire       wr_dat  = wr_act &&  is_data && (sel[7:4] == 4'd0);
wire [3:0] wr_reg  = sel[3:0];

function automatic [7:0] reg_mask(input [3:0] r);
    case (r)
        4'd1, 4'd3, 4'd5, 4'd13:   reg_mask = 8'h0f;
        4'd6, 4'd8, 4'd9, 4'd10:   reg_mask = 8'h1f;
        default:                   reg_mask = 8'hff;
    endcase
endfunction

wire [7:0] r7 = regs[7];

reg [7:0] rd_val;
always @* begin
    if (sel[7:4] != 4'd0)
        rd_val = 8'hff;
    else if (sel[3:0] == 4'd14 && !r7[6])
        rd_val = port_a_in;
    else if (sel[3:0] == 4'd15 && !r7[7])
        rd_val = port_b_in;
    else
        rd_val = read_data;
end

assign bus_ack  = bus_stb;
assign bus_dout = (void_area || is_data) ? 16'hffff : {rd_val, 8'hff};

assign port_a_oe  = r7[6];
assign port_b_oe  = r7[7];
assign port_a_out = r7[6] ? regs[14] : 8'hff;
assign port_b_out = r7[7] ? regs[15] : 8'hff;

// ---------------------------------------------------------------------------
// 2 MHz clock enable and 250 kHz tick
// ---------------------------------------------------------------------------
localparam int ACC_W = $clog2(CLK_HZ + YM_HZ) + 1;
localparam [ACC_W-1:0] ACC_INC = ACC_W'(YM_HZ);
localparam [ACC_W-1:0] ACC_MOD = ACC_W'(CLK_HZ);
reg [ACC_W-1:0] acc;
reg [2:0]       div8;
reg             tick;
always @(posedge clk) begin
    tick <= 1'b0;
    if (reset) begin
        acc  <= '0;
        div8 <= 3'd0;
    end else if (acc + ACC_INC >= ACC_MOD) begin
        acc  <= acc + ACC_INC - ACC_MOD;
        div8 <= div8 + 3'd1;
        if (div8 == 3'd7)
            tick <= 1'b1;
    end else begin
        acc <= acc + ACC_INC;
    end
end

// ---------------------------------------------------------------------------
// Tone / noise / envelope generators (sound.c YM2149_DoSamples_250)
// ---------------------------------------------------------------------------
reg [11:0] tcnt_a, tcnt_b, tcnt_c;
reg        tval_a, tval_b, tval_c;
reg  [4:0] ncnt;
reg        nval;
reg        ndiv2;
reg [16:0] lfsr;
reg [15:0] ecnt;
reg  [6:0] epos;

wire [11:0] tper_a = {regs[1][3:0], regs[0]};
wire [11:0] tper_b = {regs[3][3:0], regs[2]};
wire [11:0] tper_c = {regs[5][3:0], regs[4]};
wire  [4:0] nper   = regs[6][4:0];
wire [15:0] eper   = {regs[12], regs[11]};
wire  [3:0] eshape = regs[13][3:0];

// next state values at a tick
wire [12:0] tinc_a = {1'b0, tcnt_a} + 13'd1;
wire [12:0] tinc_b = {1'b0, tcnt_b} + 13'd1;
wire [12:0] tinc_c = {1'b0, tcnt_c} + 13'd1;
wire        tflip_a = tinc_a >= {1'b0, tper_a};
wire        tflip_b = tinc_b >= {1'b0, tper_b};
wire        tflip_c = tinc_c >= {1'b0, tper_c};
wire        tval_a_n = tval_a ^ tflip_a;
wire        tval_b_n = tval_b ^ tflip_b;
wire        tval_c_n = tval_c ^ tflip_c;

wire        ndiv2_n = ~ndiv2;
wire  [5:0] ninc    = {1'b0, ncnt} + (ndiv2_n ? 6'd0 : 6'd1);
wire        nstep   = ninc >= {1'b0, nper};
wire        nval_n  = nstep ? lfsr[0] : nval;
wire [16:0] lfsr_n  = nstep ? ((lfsr >> 1) ^ (lfsr[0] ? 17'h12000 : 17'h0)) : lfsr;

wire [16:0] einc    = {1'b0, ecnt} + 17'd1;
wire        estep   = einc >= {1'b0, eper};
wire  [6:0] epos_i  = epos + 7'd1;
wire  [6:0] epos_n  = !estep ? epos : (epos_i >= 7'd96 ? epos_i - 7'd64 : epos_i);

// Envelope volume: YmEnvDef[shape][block], block = pos / 32
localparam [1:0] E_GODOWN = 2'd0, E_GOUP = 2'd1, E_DOWN = 2'd2, E_UP = 2'd3;
function automatic [1:0] env_def(input [3:0] shape, input [1:0] blk);
    reg [1:0] b0, b1;
    begin
        case (shape)
            4'd8:    begin b0 = E_GODOWN; b1 = E_GODOWN; end
            4'd10:   begin b0 = E_GODOWN; b1 = E_GOUP;   end
            4'd11:   begin b0 = E_GODOWN; b1 = E_UP;     end
            4'd12:   begin b0 = E_GOUP;   b1 = E_GOUP;   end
            4'd13:   begin b0 = E_GOUP;   b1 = E_UP;     end
            4'd14:   begin b0 = E_GOUP;   b1 = E_GODOWN; end
            4'd4, 4'd5, 4'd6, 4'd7, 4'd15:
                     begin b0 = E_GOUP;   b1 = E_DOWN;   end
            default: begin b0 = E_GODOWN; b1 = E_DOWN;   end   // 0-3, 9
        endcase
        case (blk)
            2'd0:    env_def = b0;
            2'd1:    env_def = b1;
            default: env_def = (shape == 4'd10 || shape == 4'd14) ? b0 : b1;
        endcase
    end
endfunction

function automatic [4:0] env_vol(input [3:0] shape, input [6:0] pos);
    reg [1:0] blk;
    begin
        blk = (pos < 7'd32) ? 2'd0 : (pos < 7'd64) ? 2'd1 : 2'd2;
        case (env_def(shape, blk))
            E_GODOWN: env_vol = 5'd31 - pos[4:0];
            E_GOUP:   env_vol = pos[4:0];
            E_DOWN:   env_vol = 5'd0;
            default:  env_vol = 5'd31;
        endcase
    end
endfunction

// YmVolume4to5
function automatic [4:0] vol4to5(input [3:0] v);
    vol4to5 = (v <= 4'd1) ? {1'b0, v} : {v, 1'b1};
endfunction

function automatic [4:0] voice_level(input t, input n, input mix_t, input mix_n,
                                     input [4:0] vreg, input [4:0] ev);
    begin
        if ((t | mix_t) & (n | mix_n))
            voice_level = vreg[4] ? ev : vol4to5(vreg[3:0]);
        else
            voice_level = 5'd0;
    end
endfunction

// The tick's sample uses the state after the tick (as DoSamples_250 does)
wire  [4:0] ev_n  = env_vol(eshape, epos_n);
wire  [4:0] lev_a = voice_level(tval_a_n, nval_n, r7[0], r7[3], regs[8][4:0],  ev_n);
wire  [4:0] lev_b = voice_level(tval_b_n, nval_n, r7[1], r7[4], regs[9][4:0],  ev_n);
wire  [4:0] lev_c = voice_level(tval_c_n, nval_n, r7[2], r7[5], regs[10][4:0], ev_n);

integer i;
always @(posedge clk) begin
    if (reset) begin
        for (i = 0; i < 16; i = i + 1)
            regs[i] <= 8'h00;
        regs[14]  <= 8'hff;
        sel       <= 8'h00;
        read_data <= 8'h00;
        tcnt_a <= 12'd0; tcnt_b <= 12'd0; tcnt_c <= 12'd0;
        tval_a <= 1'b0;  tval_b <= 1'b0;  tval_c <= 1'b0;
        ncnt   <= 5'd0;  nval   <= 1'b0;  ndiv2  <= 1'b0;
        lfsr   <= 17'd1;
        ecnt   <= 16'd0; epos   <= 7'd0;
        ch_a   <= 5'd0;  ch_b   <= 5'd0;  ch_c   <= 5'd0;
    end else begin
        // generators: the tick uses the register values before a write that
        // happens on the same clock (Hatari: samples first, then the write)
        if (tick) begin
            tcnt_a <= tflip_a ? 12'd0 : tinc_a[11:0];
            tcnt_b <= tflip_b ? 12'd0 : tinc_b[11:0];
            tcnt_c <= tflip_c ? 12'd0 : tinc_c[11:0];
            tval_a <= tval_a_n;
            tval_b <= tval_b_n;
            tval_c <= tval_c_n;
            ndiv2  <= ndiv2_n;
            ncnt   <= nstep ? 5'd0 : ninc[4:0];
            nval   <= nval_n;
            lfsr   <= lfsr_n;
            ecnt   <= estep ? 16'd0 : einc[15:0];
            epos   <= epos_n;
            ch_a   <= lev_a;
            ch_b   <= lev_b;
            ch_c   <= lev_c;
        end

        if (wr_sel) begin
            sel       <= wr_byte;
            read_data <= (wr_byte[7:4] == 4'd0) ? regs[wr_byte[3:0]] : 8'hff;
        end
        if (wr_dat) begin
            regs[wr_reg] <= wr_byte & reg_mask(wr_reg);
            read_data    <= wr_byte;
            if (wr_reg == 4'd13) begin
                // Sound_WriteReg(13): restart the envelope
                ecnt <= 16'd0;
                epos <= 7'd0;
            end
        end
    end
end

// ---------------------------------------------------------------------------
// Mixing: Hatari ymout5[C][B][A] then PWMaliasFilter
// ---------------------------------------------------------------------------
reg        mix_go;        // tick happened, ch_* hold the new levels
reg [14:0] mix_val;
reg        mix_valid;

generate
if (VOL_TABLE) begin : g_table
    (* ramstyle = "M10K" *) reg [14:0] vol_rom [0:32767];
    initial $readmemh(VOL_HEX, vol_rom);
    always @(posedge clk) begin
        mix_val <= vol_rom[{ch_c, ch_b, ch_a}];
    end
    always @(posedge clk) begin
        mix_valid <= mix_go && !reset;
    end
end else begin : g_linear
    // ymout1c5bit (sound.c)
    function automatic [15:0] out1c(input [4:0] v);
        case (v)
            5'd0:  out1c = 16'd0;     5'd1:  out1c = 16'd369;
            5'd2:  out1c = 16'd438;   5'd3:  out1c = 16'd521;
            5'd4:  out1c = 16'd619;   5'd5:  out1c = 16'd735;
            5'd6:  out1c = 16'd874;   5'd7:  out1c = 16'd1039;
            5'd8:  out1c = 16'd1234;  5'd9:  out1c = 16'd1467;
            5'd10: out1c = 16'd1744;  5'd11: out1c = 16'd2072;
            5'd12: out1c = 16'd2463;  5'd13: out1c = 16'd2927;
            5'd14: out1c = 16'd3479;  5'd15: out1c = 16'd4135;
            5'd16: out1c = 16'd4914;  5'd17: out1c = 16'd5841;
            5'd18: out1c = 16'd6942;  5'd19: out1c = 16'd8250;
            5'd20: out1c = 16'd9806;  5'd21: out1c = 16'd11654;
            5'd22: out1c = 16'd13851; 5'd23: out1c = 16'd16462;
            5'd24: out1c = 16'd19565; 5'd25: out1c = 16'd23253;
            5'd26: out1c = 16'd27636; 5'd27: out1c = 16'd32845;
            5'd28: out1c = 16'd39037; 5'd29: out1c = 16'd46395;
            5'd30: out1c = 16'd55141; default: out1c = 16'd65535;
        endcase
    endfunction
    reg [17:0] lsum;
    reg        lsum_v;
    // q = sum / 3 (exact for sum <= 196605); then q*32767/65535, which
    // equals (q-1)>>1 for q >= 1 and 0 for q = 0 (exact over 0..65535)
    wire [36:0] lprod = lsum * 19'd174763;
    wire [15:0] lq    = lprod[34:19];
    always @(posedge clk) begin
        lsum      <= {2'b0, out1c(ch_a)} + {2'b0, out1c(ch_b)} + {2'b0, out1c(ch_c)};
        lsum_v    <= mix_go && !reset;
        mix_val   <= (lq == 16'd0) ? 15'd0 : lq[15:1] - {14'd0, ~lq[0]};
        mix_valid <= lsum_v && !reset;
    end
end
endgenerate

// PWMaliasFilter: if (x0 >= y0) y0 = x0 else y0 = (3*(x0+x1) + (y0<<1)) >> 3
reg  [14:0] flt_y0, flt_x1;
wire [18:0] flt_sum = 19'd3 * ({4'd0, mix_val} + {4'd0, flt_x1}) + {3'd0, flt_y0, 1'b0};
wire [14:0] flt_dn  = flt_sum[17:3];

always @(posedge clk) begin
    snd_stb <= 1'b0;
    mix_go  <= tick && !reset;
    if (reset) begin
        flt_y0     <= 15'd0;
        flt_x1     <= 15'd0;
        snd_sample <= 16'sd0;
    end else if (mix_valid) begin
        snd_stb <= 1'b1;
        if (PWM_FILTER) begin
            if (mix_val >= flt_y0) begin
                flt_y0     <= mix_val;
                snd_sample <= {1'b0, mix_val};
            end else begin
                flt_y0     <= flt_dn;
                snd_sample <= {1'b0, flt_dn};
            end
            flt_x1 <= mix_val;
        end else begin
            snd_sample <= {1'b0, mix_val};
        end
    end
end

endmodule
