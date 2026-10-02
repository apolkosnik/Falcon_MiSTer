// falcon_nvram.sv - MC146818A RTC + NVRAM of the Atari Falcon030
//
// Behavioural reference: Hatari src/falcon/nvram.c
//   NvRam_Init (default image, VGA/RGB video mode bits, language, keyboard),
//   NvRam_SetChecksum, NvRam_Reset, clear_reg_c, NvRam_Select_ReadByte,
//   NvRam_Select_WriteByte, NvRam_Data_ReadByte, NvRam_Data_WriteByte,
//   bin2BCD, NvRam_Load/NvRam_Save (50 byte image = bytes 14..63).
// Chip behaviour where Hatari does not emulate it: MC146818A datasheet
//   (update cycle, UIP timing, SET, 12/24h, periodic/alarm/update flags).
//
// Register bus (docs/ARCHITECTURE.md), bus_addr[1:1] inside $FF8960-$FF8963,
// registers on the odd byte (bus_lds), even byte reads $FF:
//   $FF8961 : index (read/write).  Values >= 64 are ignored (Hatari).
//   $FF8963 : data of the indexed byte.
//   $FF8960/$FF8962 : read $FF, writes ignored (Hatari: void, no bus error).
// bus_ack is given on the bus_stb clock.
//
// Byte map:
//   0 sec  1 alarm sec  2 min  3 alarm min  4 hour  5 alarm hour
//   6 day of week (1..7, Sunday = 1)  7 day of month  8 month
//   9 year (years since 1968, as TOS and Hatari use: 2026 = 58)
//   10 reg A: UIP (bit 7, read only), DV2..0, RS3..0
//   11 reg B: SET, PIE, AIE, UIE, SQWE, DM (1 = binary), 24/12, DSE
//   12 reg C: IRQF, PF, AF, UF (read only, cleared by reading)
//   13 reg D: VRT = 1 (read only)
//   14..63 : NVRAM (TOS: 14/15 boot OS, 20 language, 21 keyboard, 22 date
//            format, 23 date separator, 24 boot delay, 28/29 video mode,
//            30 SCSI id/arbitration, 62/63 checksum)
//
// Time keeping: the time is kept in binary internally and converted on
// reads/writes according to the current DM bit (as Hatari does with bin2BCD).
// Implementation (area): bytes 0..63 (time fields in binary at 0,2,4,6..9,
// NVRAM at 14..63) are in three MLAB copies with asynchronous reads (CPU
// port, nv port, sequencer) and one shared write port; alarm bytes and
// registers A..D are flip-flops.  The once-per-second calendar update, the
// alarm compare and the loading of the MiSTer time run through one small
// sequential datapath, one field per step (a read step takes 2 clocks, a
// full calendar rollover with the alarm compare about 25 clocks; a step
// waits while the CPU or the nv port writes); UF/AF are set and UIP drops
// when it is done.  One register->binary converter serves CPU writes, the
// alarm compare and the MiSTer time.  The MiSTer time is written in 7
// clocks after rtc[64] toggles (day of week first); its two digit year is
// mapped in BCD ((yy + 32) mod 100 = years since 1968).
// A 32.768 kHz time base is derived from CLK_HZ.  Once per second (unless
// SET is 1) an update cycle runs: UIP rises 244 us (8 ticks) before the
// second boundary and falls 1984 us (65 ticks) after it, which is when the
// time advances, UF is set and the alarm is compared.  SET = 1 aborts the
// update cycle and clears UIP.  Periodic flag PF at the rate selected by
// RS3..0 (32.768 kHz table: RS 1/2 = 256/128 Hz, RS n = 2^(16-n) Hz).
// The divider select DV2..0 is stored but the time base always runs.
//
// MiSTer time (rtc[64:0] from hps_io, Main_MiSTer user_io.cpp): BCD
// [7:0] sec, [15:8] min, [23:16] hour (24 h), [31:24] day of month,
// [39:32] month 1..12, [47:40] year % 100, [55:48] day of week in binary
// 0..6 with Sunday = 0 (tm_wday), [63:56] = 0x40, [64] toggles on each new
// time.  The time is loaded when rtc[64] toggles and when the default image
// is built if rtc[39:32] != 0.  Two digit years 00..67 are 2000..2067,
// 68..99 are 1968..1999.  Without any MiSTer time the clock starts at
// 2026-01-01 00:00:00 (Thursday).
//
// Default image (Hatari's static nvram[] + NvRam_Init), built on the first
// reset after configuration and on every nv_init pulse, with the checksum of
// NvRam_SetChecksum (sum of bytes 14..61, byte 62 = ~sum, byte 63 = sum):
//   22 = 17, 23 = 46, 24 = 32, 25 = 1, 26 = 255, 28/29 = video mode,
//   30 = 135, others 0; 20 = cfg_lang, 21 = cfg_kbd (TOS_LANG_*, US = 0);
//   cfg_vga = 1: 28 = $00, 29 = $1A (VGA, 60 Hz, 640x480 16 colours)
//   cfg_vga = 0: 28 = $01, 29 = $2A (RGB/TV, 50 Hz, interlace)
//   reg A = 42 ($2A), reg B = DM|24h ($06), alarm bytes = 255.
//
// NVRAM save/restore port for the MiSTer (same 50 byte layout as Hatari's
// hatari.nvram file): nv_addr 0..49 addresses bytes 14..63.  nv_dout is the
// current byte (combinational, 0 for nv_addr >= 50), nv_wr writes nv_din
// (the saved image carries its own checksum, nothing is recomputed).
// nv_changed pulses for one clock whenever the CPU writes a byte 14..63.
// An nv_wr on the same clock as a CPU data write is held in a one entry
// buffer and written on the next clock (nv_dout shows it one clock later).
// nv_wr while the default image is being built may be overwritten by it.
//
// Reset (synchronous, the system reset): clears PIE, AIE, UIE, SQWE in reg B
// and the flags in reg C, index = 0 (NvRam_Reset).  The time and the NVRAM
// contents survive a reset.
//
// irq: MC146818 IRQ output (IRQF); not connected on the Falcon.
//
// Deviations from Hatari:
//  - Hatari reads the host clock on every access; here the clock runs from
//    the MiSTer time and software writes to the time bytes are kept.
//  - UIP follows the datasheet timing; Hatari toggles it on every read.
//  - Reg C flags are real (UF each second, AF on alarm, PF periodic);
//    Hatari reports UF|PF after every clear.
//  - 12 hour mode: hours 12..23 are PM, 0 reads as 12 AM (datasheet);
//    Hatari flags hour 0 as PM and hour 12 as AM.
//  - Alarm bytes read back as written; Hatari applies bin2BCD to the stored
//    value.  Alarm values $C0-$FF are "don't care".
//  - DSE (daylight saving) and the SQW output are not implemented.
module falcon_nvram #(
    parameter int CLK_HZ = 32000000
) (
    input             clk,
    input             reset,

    input             bus_cs,
    input             bus_stb,
    input             bus_we,
    input      [1:1]  bus_addr,
    input             bus_uds,
    input             bus_lds,
    input      [15:0] bus_din,
    output     [15:0] bus_dout,
    output            bus_ack,

    input      [64:0] rtc,

    input             cfg_vga,
    input       [7:0] cfg_lang,
    input       [7:0] cfg_kbd,
    input             nv_init,

    input       [5:0] nv_addr,
    output      [7:0] nv_dout,
    input       [7:0] nv_din,
    input             nv_wr,
    output reg        nv_changed,

    output            irq
);

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
// binary -> BCD for 0..127 (100..127 give tens digits A..C), as a table
function automatic [7:0] bin2bcd(input [6:0] v);
    case (v)
        7'd0: bin2bcd = 8'h00;
        7'd1: bin2bcd = 8'h01;
        7'd2: bin2bcd = 8'h02;
        7'd3: bin2bcd = 8'h03;
        7'd4: bin2bcd = 8'h04;
        7'd5: bin2bcd = 8'h05;
        7'd6: bin2bcd = 8'h06;
        7'd7: bin2bcd = 8'h07;
        7'd8: bin2bcd = 8'h08;
        7'd9: bin2bcd = 8'h09;
        7'd10: bin2bcd = 8'h10;
        7'd11: bin2bcd = 8'h11;
        7'd12: bin2bcd = 8'h12;
        7'd13: bin2bcd = 8'h13;
        7'd14: bin2bcd = 8'h14;
        7'd15: bin2bcd = 8'h15;
        7'd16: bin2bcd = 8'h16;
        7'd17: bin2bcd = 8'h17;
        7'd18: bin2bcd = 8'h18;
        7'd19: bin2bcd = 8'h19;
        7'd20: bin2bcd = 8'h20;
        7'd21: bin2bcd = 8'h21;
        7'd22: bin2bcd = 8'h22;
        7'd23: bin2bcd = 8'h23;
        7'd24: bin2bcd = 8'h24;
        7'd25: bin2bcd = 8'h25;
        7'd26: bin2bcd = 8'h26;
        7'd27: bin2bcd = 8'h27;
        7'd28: bin2bcd = 8'h28;
        7'd29: bin2bcd = 8'h29;
        7'd30: bin2bcd = 8'h30;
        7'd31: bin2bcd = 8'h31;
        7'd32: bin2bcd = 8'h32;
        7'd33: bin2bcd = 8'h33;
        7'd34: bin2bcd = 8'h34;
        7'd35: bin2bcd = 8'h35;
        7'd36: bin2bcd = 8'h36;
        7'd37: bin2bcd = 8'h37;
        7'd38: bin2bcd = 8'h38;
        7'd39: bin2bcd = 8'h39;
        7'd40: bin2bcd = 8'h40;
        7'd41: bin2bcd = 8'h41;
        7'd42: bin2bcd = 8'h42;
        7'd43: bin2bcd = 8'h43;
        7'd44: bin2bcd = 8'h44;
        7'd45: bin2bcd = 8'h45;
        7'd46: bin2bcd = 8'h46;
        7'd47: bin2bcd = 8'h47;
        7'd48: bin2bcd = 8'h48;
        7'd49: bin2bcd = 8'h49;
        7'd50: bin2bcd = 8'h50;
        7'd51: bin2bcd = 8'h51;
        7'd52: bin2bcd = 8'h52;
        7'd53: bin2bcd = 8'h53;
        7'd54: bin2bcd = 8'h54;
        7'd55: bin2bcd = 8'h55;
        7'd56: bin2bcd = 8'h56;
        7'd57: bin2bcd = 8'h57;
        7'd58: bin2bcd = 8'h58;
        7'd59: bin2bcd = 8'h59;
        7'd60: bin2bcd = 8'h60;
        7'd61: bin2bcd = 8'h61;
        7'd62: bin2bcd = 8'h62;
        7'd63: bin2bcd = 8'h63;
        7'd64: bin2bcd = 8'h64;
        7'd65: bin2bcd = 8'h65;
        7'd66: bin2bcd = 8'h66;
        7'd67: bin2bcd = 8'h67;
        7'd68: bin2bcd = 8'h68;
        7'd69: bin2bcd = 8'h69;
        7'd70: bin2bcd = 8'h70;
        7'd71: bin2bcd = 8'h71;
        7'd72: bin2bcd = 8'h72;
        7'd73: bin2bcd = 8'h73;
        7'd74: bin2bcd = 8'h74;
        7'd75: bin2bcd = 8'h75;
        7'd76: bin2bcd = 8'h76;
        7'd77: bin2bcd = 8'h77;
        7'd78: bin2bcd = 8'h78;
        7'd79: bin2bcd = 8'h79;
        7'd80: bin2bcd = 8'h80;
        7'd81: bin2bcd = 8'h81;
        7'd82: bin2bcd = 8'h82;
        7'd83: bin2bcd = 8'h83;
        7'd84: bin2bcd = 8'h84;
        7'd85: bin2bcd = 8'h85;
        7'd86: bin2bcd = 8'h86;
        7'd87: bin2bcd = 8'h87;
        7'd88: bin2bcd = 8'h88;
        7'd89: bin2bcd = 8'h89;
        7'd90: bin2bcd = 8'h90;
        7'd91: bin2bcd = 8'h91;
        7'd92: bin2bcd = 8'h92;
        7'd93: bin2bcd = 8'h93;
        7'd94: bin2bcd = 8'h94;
        7'd95: bin2bcd = 8'h95;
        7'd96: bin2bcd = 8'h96;
        7'd97: bin2bcd = 8'h97;
        7'd98: bin2bcd = 8'h98;
        7'd99: bin2bcd = 8'h99;
        7'd100: bin2bcd = 8'ha0;
        7'd101: bin2bcd = 8'ha1;
        7'd102: bin2bcd = 8'ha2;
        7'd103: bin2bcd = 8'ha3;
        7'd104: bin2bcd = 8'ha4;
        7'd105: bin2bcd = 8'ha5;
        7'd106: bin2bcd = 8'ha6;
        7'd107: bin2bcd = 8'ha7;
        7'd108: bin2bcd = 8'ha8;
        7'd109: bin2bcd = 8'ha9;
        7'd110: bin2bcd = 8'hb0;
        7'd111: bin2bcd = 8'hb1;
        7'd112: bin2bcd = 8'hb2;
        7'd113: bin2bcd = 8'hb3;
        7'd114: bin2bcd = 8'hb4;
        7'd115: bin2bcd = 8'hb5;
        7'd116: bin2bcd = 8'hb6;
        7'd117: bin2bcd = 8'hb7;
        7'd118: bin2bcd = 8'hb8;
        7'd119: bin2bcd = 8'hb9;
        7'd120: bin2bcd = 8'hc0;
        7'd121: bin2bcd = 8'hc1;
        7'd122: bin2bcd = 8'hc2;
        7'd123: bin2bcd = 8'hc3;
        7'd124: bin2bcd = 8'hc4;
        7'd125: bin2bcd = 8'hc5;
        7'd126: bin2bcd = 8'hc6;
        7'd127: bin2bcd = 8'hc7;
        default: bin2bcd = 8'h00;
    endcase
endfunction

function automatic [6:0] bcd2bin(input [7:0] v);
    bcd2bin = 7'(v[7:4] * 4'd10) + {3'd0, v[3:0]};
endfunction

function automatic [4:0] days_in_month(input [3:0] m, input [6:0] y);
    case (m)
        4'd2:                     days_in_month = (y[1:0] == 2'd0) ? 5'd29 : 5'd28;
        4'd4, 4'd6, 4'd9, 4'd11:  days_in_month = 5'd30;
        default:                  days_in_month = 5'd31;
    endcase
endfunction

// Default image bytes 14..63 except the checksum (NvRam_Init)
function automatic [7:0] default_byte(input [5:0] a, input vga, input [7:0] lang, input [7:0] kbd);
    case (a)
        6'd20: default_byte = lang;
        6'd21: default_byte = kbd;
        6'd22: default_byte = 8'd17;
        6'd23: default_byte = 8'd46;
        6'd24: default_byte = 8'd32;
        6'd25: default_byte = 8'd1;
        6'd26: default_byte = 8'd255;
        6'd28: default_byte = vga ? 8'h00 : 8'h01;
        6'd29: default_byte = vga ? 8'h1a : 8'h2a;
        6'd30: default_byte = 8'd135;
        default: default_byte = 8'd0;
    endcase
endfunction

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
reg  [5:0] index;
reg  [7:0] al_sec, al_min, al_hour;   // as written
reg  [6:0] reg_a;           // DV2..0, RS3..0
reg  [7:0] reg_b;
reg        uip, f_uf, f_af, f_pf;

wire dm   = reg_b[2];
wire h24  = reg_b[1];
wire set  = reg_b[7];

assign irq = (f_pf & reg_b[6]) | (f_af & reg_b[5]) | (f_uf & reg_b[4]);

function automatic [7:0] to_mode(input [6:0] v, input bin);
    to_mode = bin ? {1'b0, v} : bin2bcd(v);
endfunction
function automatic [6:0] from_mode(input [7:0] v, input bin);
    from_mode = bin ? v[6:0] : bcd2bin(v);
endfunction

// time field k (0 sec, 1 min, 2 hour, 3 day of week, 4 day of month,
// 5 month, 6 year) -> byte address
function automatic [5:0] f_addr(input [2:0] k);
    case (k)
        3'd0:    f_addr = 6'd0;
        3'd1:    f_addr = 6'd2;
        3'd2:    f_addr = 6'd4;
        3'd3:    f_addr = 6'd6;
        3'd4:    f_addr = 6'd7;
        3'd5:    f_addr = 6'd8;
        default: f_addr = 6'd9;
    endcase
endfunction

// ---------------------------------------------------------------------------
// Byte storage: three copies with one shared write port.
//   ram_c (M10K): CPU read port.  Its address is the next value of index,
//     so during every clock the registered output is the byte at index as
//     of the previous clock edge; a write at that edge is bypassed.
//   ram_s (M10K): sequencer read port, same scheme with address s_ra; the
//     data is valid once s_ra has been stable for one clock edge.
//   ram_n (MLAB, asynchronous read): nv port (nv_dout is combinational).
// ---------------------------------------------------------------------------
reg        m_we;
reg  [5:0] m_wa;
reg  [7:0] m_wd;
wire [5:0] s_ra;
wire [5:0] index_nx;
(* ramstyle = "M10K, no_rw_check" *) reg [7:0] ram_c [0:63];
(* ramstyle = "M10K, no_rw_check" *) reg [7:0] ram_s [0:63];
(* ramstyle = "MLAB, no_rw_check" *) reg [7:0] ram_n [0:63];
reg  [7:0] ram_c_qr, ram_s_qr, byp_c_d, byp_s_d;
reg        byp_c_v, byp_s_v;
reg  [5:0] s_ra_q;
always @(posedge clk) begin
    if (m_we) ram_c[m_wa] <= m_wd;
    ram_c_qr <= ram_c[index_nx];
end
always @(posedge clk) begin
    if (m_we) ram_s[m_wa] <= m_wd;
    ram_s_qr <= ram_s[s_ra];
end
always @(posedge clk) begin
    if (m_we) ram_n[m_wa] <= m_wd;
end
always @(posedge clk) begin
    byp_c_v <= m_we && (m_wa == index_nx);
    byp_s_v <= m_we && (m_wa == s_ra);
    byp_c_d <= m_wd;
    byp_s_d <= m_wd;
    s_ra_q  <= s_ra;
end

wire       nv_ok  = nv_addr < 6'd50;
wire [5:0] nv_idx = nv_addr + 6'd14;
assign nv_dout = nv_ok ? ram_n[nv_idx] : 8'h00;
wire [7:0] ram_c_q = byp_c_v ? byp_c_d : ram_c_qr;
wire [6:0] sq_f    = byp_s_v ? byp_s_d[6:0] : ram_s_qr[6:0];
wire       s_valid = s_ra_q == s_ra;

// ---------------------------------------------------------------------------
// Bus interface
// ---------------------------------------------------------------------------
reg rd_time;
always @* begin
    case (index)
        6'd0, 6'd2, 6'd4, 6'd6, 6'd7, 6'd8, 6'd9: rd_time = 1'b1;
        default: rd_time = 1'b0;
    endcase
end
wire [6:0] rd_f   = ram_c_q[6:0];
wire       rd_h12 = (index == 6'd4) && !h24;
wire [6:0] rd_bin = !rd_h12 ? rd_f : (rd_f == 7'd0) ? 7'd12 : (rd_f > 7'd12) ? rd_f - 7'd12 : rd_f;
wire       rd_pm  = rd_h12 && (rd_f >= 7'd12);

reg [7:0] direct_val;
always @* begin
    case (index)
        6'd1:    direct_val = al_sec;
        6'd3:    direct_val = al_min;
        6'd5:    direct_val = al_hour;
        6'd10:   direct_val = {uip, reg_a};
        6'd11:   direct_val = reg_b;
        6'd12:   direct_val = {irq, f_pf, f_af, f_uf, 4'd0};
        6'd13:   direct_val = 8'h80;
        default: direct_val = ram_c_q;
    endcase
end
wire [7:0] data_val = rd_time ? (to_mode(rd_bin, dm) | {rd_pm, 7'd0}) : direct_val;

wire [7:0] lo_val = bus_addr[1] ? data_val : {2'b00, index};
assign bus_dout = {8'hff, lo_val};
assign bus_ack  = bus_stb;

wire acc_lo   = bus_stb && bus_lds;
wire wr_index = acc_lo &&  bus_we && !bus_addr[1];
wire wr_data  = acc_lo &&  bus_we &&  bus_addr[1];
wire rd_data  = acc_lo && !bus_we &&  bus_addr[1];
wire [7:0] wv = bus_din[7:0];
wire       cpu_t_wr   = wr_data && rd_time;
assign     index_nx   = reset ? 6'd0 : (wr_index && wv < 8'd64) ? wv[5:0] : index;
wire       cpu_ram_wr = wr_data && (index >= 6'd14);

// ---------------------------------------------------------------------------
// 32.768 kHz time base (fractional accumulator reduced by gcd), divider
// ---------------------------------------------------------------------------
function automatic int gcd_f(input int a, input int b);
    int x, y, t;
    begin
        x = a; y = b;
        for (int i = 0; i < 64; i++)
            if (y != 0) begin t = x % y; x = y; y = t; end
        gcd_f = x;
    end
endfunction
localparam int TB_HZ = 32768;
localparam int TB_G  = gcd_f(CLK_HZ, TB_HZ);
localparam int TB_I  = TB_HZ / TB_G;
localparam int TB_M  = CLK_HZ / TB_G;
localparam int ACC_W = $clog2(TB_M + TB_I) + 2;
// acc holds (phase - TB_M), always negative; a tick is due when
// acc + TB_I >= 0, i.e. acc >= -TB_I
localparam [ACC_W-1:0] ACC_INC  = ACC_W'(TB_I);
localparam [ACC_W-1:0] ACC_WRAP = ACC_W'(TB_I - TB_M);
localparam [ACC_W-1:0] ACC_THR  = ACC_W'(-TB_I);
localparam [ACC_W-1:0] ACC_INIT = ACC_W'(-TB_M);
reg [ACC_W-1:0] acc;
wire            acc_due = $signed(acc) >= $signed(ACC_THR);
reg             tb_tick;
reg      [14:0] dc;          // 32768 ticks per second
reg             upd_run;     // between the second boundary and the update

always @(posedge clk) begin
    tb_tick <= 1'b0;
    if (reset)
        acc <= ACC_INIT;
    else begin
        acc     <= acc + (acc_due ? ACC_WRAP : ACC_INC);
        tb_tick <= acc_due;
    end
end

wire [14:0] dc_n = dc + 15'd1;

// periodic rate
// PF when the low (n-1) bits of the divider are 0, n = RS (RS 1/2 act as 8/9)
reg [14:0] pf_mask;
always @* begin
    for (int i = 0; i < 15; i++) begin
        case (reg_a[3:0])
            4'd1:    pf_mask[i] = i < 7;
            4'd2:    pf_mask[i] = i < 8;
            default: pf_mask[i] = i < (int'(reg_a[3:0]) - 1);
        endcase
    end
end
wire        pf_hit  = (reg_a[3:0] != 4'd0) && ((dc_n & pf_mask) == 15'd0);
wire        upd_go  = tb_tick && !set && dc_n == 15'd65 && upd_run;

// ---------------------------------------------------------------------------
// MiSTer time
// ---------------------------------------------------------------------------
reg        rtc_tog;
wire       rtc_new   = rtc[64] != rtc_tog;
wire       rtc_valid = rtc[39:32] != 8'd0;

// ---------------------------------------------------------------------------
// Default image builder
// ---------------------------------------------------------------------------
reg        img_done = 1'b0;   // FPGA configuration value: build on first reset
reg        img_busy;
reg  [5:0] img_ptr;
reg  [7:0] img_sum;
wire [7:0] img_byte = default_byte(img_ptr, cfg_vga, cfg_lang, cfg_kbd);
wire       img_go   = (reset && !img_done && !img_busy) || nv_init;

// ---------------------------------------------------------------------------
// Sequential datapath: calendar increment, alarm compare, time load
// ---------------------------------------------------------------------------
localparam [2:0] SQ_IDLE = 3'd0, SQ_INC = 3'd1, SQ_ALM = 3'd2, SQ_LOAD = 3'd3, SQ_RD = 3'd4;
reg  [2:0] sq_op_r;
reg  [2:0] sq_k_r;
reg        sq_def_r;        // load the fallback time instead of rtc
reg        sq_alok;
reg  [3:0] mon_l;
reg  [6:0] year_l;
// a time load (MiSTer time or image build) writes one field per clock in
// the order day of week, sec, min, hour, day, month, year
wire       ld_now = rtc_new || img_go;
wire [2:0] sq_op  = sq_op_r;
wire [2:0] sq_k   = sq_k_r;
wire       sq_def = sq_def_r;
assign     s_ra   = f_addr(sq_k);

// nv_wr buffer
reg        nvp;
reg  [5:0] nvp_a;
reg  [7:0] nvp_d;
wire       nv_req = nv_wr && nv_ok;

// write port users, in priority order: CPU, nv buffer, nv_wr, sequencer,
// image builder.  A user that does not get the port waits.
wire       sq_wants = (sq_op == SQ_INC) || (sq_op == SQ_LOAD);
wire       port_cpu = cpu_t_wr || cpu_ram_wr;
wire       port_nv  = nvp || nv_req;
wire       sq_stall = wr_data || (sq_wants && port_nv);
wire       sq_run   = !sq_stall && (sq_op == SQ_LOAD || s_valid);
wire       img_go_w = img_busy && !port_cpu && !port_nv && !(sq_wants && sq_run);

// increment with wrap
reg  [6:0] inc_lim, inc_wrap;
always @* begin
    case (sq_k)
        3'd0, 3'd1: begin inc_lim = 7'd59; inc_wrap = 7'd0; end
        3'd2:       begin inc_lim = 7'd23; inc_wrap = 7'd0; end
        3'd3:       begin inc_lim = 7'd7;  inc_wrap = 7'd1; end
        3'd4:       begin inc_lim = {2'd0, days_in_month(mon_l, year_l)}; inc_wrap = 7'd1; end
        3'd5:       begin inc_lim = 7'd12; inc_wrap = 7'd1; end
        default:    begin inc_lim = 7'd99; inc_wrap = 7'd0; end
    endcase
end
wire       inc_carry = sq_f >= inc_lim;
wire [6:0] inc_val   = inc_carry ? inc_wrap : sq_f + 7'd1;

// One register->binary converter shared by CPU time writes, the alarm
// compare and the MiSTer time load.  A CPU data write has priority; the
// sequencer waits for that clock.
wire       use_alm = (sq_op == SQ_ALM);
wire [7:0] al_raw  = (sq_k == 3'd0) ? al_sec : (sq_k == 3'd1) ? al_min : al_hour;
// fallback time (2026-01-01 00:00:00 Thursday) in MiSTer format
reg  [7:0] rtc_byte;
always @* begin
    if (sq_def) begin
        case (sq_k)
            3'd3:    rtc_byte = 8'h05;
            3'd4:    rtc_byte = 8'h01;
            3'd5:    rtc_byte = 8'h01;
            3'd6:    rtc_byte = 8'h26;
            default: rtc_byte = 8'h00;
        endcase
    end else case (sq_k)
        3'd0:    rtc_byte = rtc[7:0];
        3'd1:    rtc_byte = rtc[15:8];
        3'd2:    rtc_byte = rtc[23:16];
        3'd3:    rtc_byte = {5'd0, rtc[50:48] + 3'd1};   // Sunday = 0 -> 1
        3'd4:    rtc_byte = rtc[31:24];
        3'd5:    rtc_byte = rtc[39:32];
        default: rtc_byte = rtc[47:40];
    endcase
end
// year: two BCD digits yy -> years since 1968 = (yy + 32) mod 100, in BCD
wire [4:0] yr_u2   = {1'b0, rtc_byte[3:0]} + 5'd2;
wire       yr_c    = yr_u2 >= 5'd10;
wire [4:0] yr_t2   = {1'b0, rtc_byte[7:4]} + 5'd3 + {4'd0, yr_c};
wire [7:0] yr_map  = {4'(yr_t2 >= 5'd10 ? yr_t2 - 5'd10 : yr_t2), 4'(yr_c ? yr_u2 - 5'd10 : yr_u2)};
wire [7:0] ld_raw  = (sq_k == 3'd6) ? yr_map : rtc_byte;
wire [7:0] cv_raw  = wr_data ? wv : use_alm ? al_raw : ld_raw;
wire       cv_bin  = (wr_data || use_alm) ? dm : (sq_k == 3'd3);
wire       cv_hour = wr_data ? (index == 6'd4) : (use_alm && sq_k == 3'd2);
wire       cv_h12  = cv_hour && !h24;
wire       cv_m7   = (wr_data && index == 6'd0) || cv_h12;
wire [6:0] cv_base = from_mode(cv_m7 ? {1'b0, cv_raw[6:0]} : cv_raw, cv_bin);
wire [6:0] cv_h0   = (cv_base == 7'd12) ? 7'd0 : cv_base;
wire [6:0] cv_out  = !cv_h12 ? cv_base : cv_raw[7] ? cv_h0 + 7'd12 : cv_h0;

// alarm compare (values $C0-$FF match always)
wire       al_ok  = (al_raw[7:6] == 2'b11) || (cv_out == sq_f);


wire       sq_fin = (sq_op == SQ_ALM) && (sq_k == 3'd2) && sq_run;

// RAM write port
always @* begin
    m_we = 1'b0;
    m_wa = img_ptr;
    m_wd = img_byte;
    if (cpu_t_wr) begin
        m_we = 1'b1; m_wa = index;  m_wd = {1'b0, cv_out};
    end else if (cpu_ram_wr) begin
        m_we = 1'b1; m_wa = index;  m_wd = wv;
    end else if (nvp) begin
        m_we = 1'b1; m_wa = nvp_a;  m_wd = nvp_d;
    end else if (nv_req) begin
        m_we = 1'b1; m_wa = nv_idx; m_wd = nv_din;
    end else if (sq_wants && sq_run) begin
        m_we = 1'b1; m_wa = s_ra;
        m_wd = {1'b0, (sq_op == SQ_INC) ? inc_val : cv_out};
    end else if (img_busy) begin
        m_we = 1'b1;
        if (img_ptr == 6'd62)      m_wd = ~img_sum;
        else if (img_ptr == 6'd63) m_wd = img_sum;
    end
end

always @(posedge clk) begin
    nv_changed <= 1'b0;
    rtc_tog    <= rtc[64];

    // ---- nv_wr buffer (only a CPU data write blocks the nv port) ----
    if (!wr_data)
        nvp <= 1'b0;
    if (nv_req && (wr_data || nvp)) begin
        nvp   <= 1'b1;
        nvp_a <= nv_idx;
        nvp_d <= nv_din;
    end

    // ---- divider / update cycle ----
    if (tb_tick) begin
        dc <= dc_n;
        if (pf_hit) f_pf <= 1'b1;
        if (set) begin
            uip     <= 1'b0;
            upd_run <= 1'b0;
        end else begin
            if (dc_n == 15'd32760) uip <= 1'b1;
            if (dc_n == 15'd0 && uip) upd_run <= 1'b1;
            if (upd_go) upd_run <= 1'b0;
        end
    end

    // ---- sequencer ----
    if (!sq_run) begin
        sq_op_r  <= sq_op;
        sq_k_r   <= sq_k;
        sq_def_r <= sq_def;
    end else begin
        case (sq_op)
            SQ_INC: begin
                if (sq_k == 3'd3) begin
                    sq_op_r <= SQ_RD;          // fetch month and year for the day
                    sq_k_r  <= 3'd5;
                end else if (inc_carry && sq_k != 3'd6) begin
                    sq_op_r <= SQ_INC;
                    sq_k_r  <= sq_k + 3'd1;
                end else begin
                    sq_op_r <= SQ_ALM;
                    sq_k_r  <= 3'd0;
                    sq_alok <= 1'b1;
                end
            end
            SQ_RD: begin
                if (sq_k == 3'd5) begin
                    mon_l  <= sq_f[3:0];
                    sq_k_r <= 3'd6;
                end else begin
                    year_l  <= sq_f;
                    sq_op_r <= SQ_INC;
                    sq_k_r  <= 3'd4;
                end
            end
            SQ_ALM: begin
                sq_alok <= sq_alok & al_ok;
                sq_k_r  <= sq_k + 3'd1;
                if (sq_fin) begin
                    sq_op_r <= SQ_IDLE;
                    uip     <= 1'b0;
                    f_uf    <= 1'b1;
                    if (sq_alok && al_ok) f_af <= 1'b1;
                end
            end
            SQ_LOAD: begin
                sq_k_r   <= (sq_k == 3'd3) ? 3'd0 : (sq_k == 3'd2) ? 3'd4 : sq_k + 3'd1;
                if (sq_k == 3'd6) sq_op_r <= SQ_IDLE;
            end
            default: ;
        endcase
    end
    if (ld_now) begin
        sq_op_r  <= SQ_LOAD;
        sq_k_r   <= 3'd3;
        sq_def_r <= !rtc_new && !rtc_valid;
    end else if (upd_go) begin
        sq_op_r <= SQ_INC;
        sq_k_r  <= 3'd0;
    end

    // ---- CPU accesses ----
    if (wr_index && wv < 8'd64)
        index <= wv[5:0];
    if (rd_data && index == 6'd12) begin
        // reading reg C clears the flags; a flag set on this clock wins
        f_uf <= 1'b0;
        f_af <= 1'b0;
        f_pf <= 1'b0;
        if (tb_tick && pf_hit) f_pf <= 1'b1;
        if (sq_fin) begin
            f_uf <= 1'b1;
            if (sq_alok && al_ok) f_af <= 1'b1;
        end
    end
    if (wr_data) begin
        case (index)
            6'd1:  al_sec  <= wv;
            6'd3:  al_min  <= wv;
            6'd5:  al_hour <= wv;
            6'd10: reg_a   <= wv[6:0];
            6'd11: begin
                reg_b <= wv;
                if (wv[7]) begin
                    uip     <= 1'b0;
                    upd_run <= 1'b0;
                end
            end
            default: if (index >= 6'd14) nv_changed <= 1'b1;
        endcase
    end

    // ---- MiSTer time ----
    if (rtc_new) begin
        dc      <= 15'd0;
        uip     <= 1'b0;
        upd_run <= 1'b0;
    end

    // ---- default image ----
    if (img_go_w) begin
        img_sum <= img_sum + img_byte;
        img_ptr <= img_ptr + 6'd1;
        if (img_ptr == 6'd63) begin
            img_busy <= 1'b0;
            img_done <= 1'b1;
        end
    end
    if (img_go) begin
        img_busy <= 1'b1;
        img_ptr  <= 6'd14;
        img_sum  <= 8'd0;
        reg_a    <= 7'h2a;
        reg_b    <= 8'h06;
        al_sec   <= 8'hff;
        al_min   <= 8'hff;
        al_hour  <= 8'hff;
        dc       <= 15'd0;
        uip      <= 1'b0;
        upd_run  <= 1'b0;
    end

    // ---- system reset (NvRam_Reset) ----
    if (reset) begin
        reg_b[6:3] <= 4'd0;          // PIE, AIE, UIE, SQWE
        f_uf  <= 1'b0;
        f_af  <= 1'b0;
        f_pf  <= 1'b0;
        index <= 6'd0;
    end
    if (!img_done && !img_busy && !img_go) begin
        sq_op_r <= SQ_IDLE;          // nothing runs before the first image
        nvp     <= 1'b0;
    end
end

endmodule
