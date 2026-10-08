// falcon_ikbd.sv - Atari IKBD (HD6301 keyboard processor) at the protocol
// level.  No 6301 ROM is used: the documented IKBD command set is implemented
// the way Hatari src/ikbd.c does it.
//
// Serial link: rxd/txd at 7812.5 baud, 8N1, bit level (16x oversampling
// receiver, the transmitter sends start + 8 data + stop = 10 bit times per
// byte, back to back as Hatari's IKBD_SCI_Set_Line_TX does).  The baud rate
// is derived from CLK_HZ (independent of the ACIA's 500 kHz, as the real
// keyboard has its own crystal).
//
// MiSTer inputs:
//   ps2_key[10:0]   bit 10 toggles per event, bit 9 pressed, bit 8 E0, 7:0 code
//   ps2_mouse[24:0] bit 24 toggles per packet, 23:16 dy, 15:8 dx,
//                   7:0 status (0 left, 1 right, 4 X sign, 5 Y sign)
//   joystick_0/1    bit 0 right, 1 left, 2 down, 3 up, 4 fire (button 1),
//                   5 button 2, 6 button 3
// joystick_0 is ST port 0 (shared with the mouse), joystick_1 is port 1.
//
// Joystick buttons 2 and 3 follow Hatari joy.c Joy_GetStickData with its
// default configuration (bEnableJumpOnFire2 = false, bEnableAutoFire = off):
//  - button 2 presses the Space key: Hatari's JoystickSpaceBar state machine
//    (NULL -> DOWN on press, DOWN -> DOWNED when the make code $39 is sent,
//    DOWNED -> UP on release, UP -> NULL when the break code $B9 is sent).
//    The button is sampled whenever the port is read (autosend pass, $14
//    report, $16 interrogation; port 0 only while the IKBD reads it, i.e.
//    mouse off or the reset-time mouse+joystick mode), and the key code is
//    sent at the end of an autosend pass, after the mouse/joystick packets,
//    except in joystick monitoring modes (IKBD_SendAutoKeyboardCommands).
//    Hatari keeps a single state machine for both ports and updates it once
//    per port read, so a press on one port and none on the other would make
//    it alternate; here the two buttons are ORed (one Space key).
//  - button 3 is autofire for that port's fire button: while it is held the
//    fire bit is forced on for 4 VBLs and off for 4 VBLs ((nVBLs & 7) < 4
//    -> off), whatever button 1 does, wherever the stick is read.  The VBL
//    count is a free running counter at VBL_HZ (default 50 Hz: 80 ms on,
//    80 ms off, 6.25 Hz).
//
// Hatari functions followed: IKBD_Boot_ROM, IKBD_InterruptHandler_ResetTimer,
// IKBD_RunKeyboardCommand and all IKBD_Cmd_* handlers, IKBD_SCI_Get_Line_RX,
// IKBD_SCI_Set_Line_TX, IKBD_Send_Byte_Delay, IKBD_OutputBuffer_CheckFreeCount,
// IKBD_SendAutoKeyboardCommands (IKBD_GetJoystickData,
// IKBD_DuplicateMouseFireButtons, IKBD_SendOnMouseAction,
// IKBD_UpdateInternalMousePosition, IKBD_SendAutoJoysticks,
// IKBD_SendAutoJoysticksMonitoring, IKBD_SendRelMousePacket,
// IKBD_SendCursorMousePacket), IKBD_PressSTKey, IKBD_UpdateClockOnVBL,
// IKBD_CheckResetDisableBug, IKBD_BCD_Check, IKBD_BCD_Adjust.
//
// Timing (Hatari values converted from 8 MHz CPU cycles):
//   reset: $F1 is queued 502000 cycles (62.6 ms, counted as 7823 ticks of
//          8 us) after a reset (hardware reset or command $80 $01) plus 1 bit
//          time of SCI delay;
//          during that time all output is dropped (bDuringResetCriticalTime).
//   $0D  first byte delayed 10 bit times, $16 8 bit times, $1C/$21/$87-$9A
//        7 bit times (Hatari uses random delays in these ranges; fixed
//        mid values here).
//
// Deviations from Hatari (documented):
//  - Mouse/joystick sampling ("autosend") runs every 1/AUTOSEND_HZ s
//    (default 1 ms) instead of once per VBL.  Mouse motion accumulates
//    (also below the threshold) and a relative mouse packet or a cursor key
//    group is only queued when the output buffer is empty, so motion is
//    never lost while the 7812.5 baud link is busy (the real IKBD also
//    accumulates).  Hatari discards residual motion each VBL.
//  - Joystick event packets update the "previous state" only when the
//    packet was queued, so a dropped report is retried.
//  - A PS/2 mouse packet (motion and buttons) is taken by the processor
//    between two of its tasks (at most a few hundred clocks after it
//    arrives), not asynchronously.
//  - Mouse scale ($0C) divides (mouse ticks per internal unit, as the IKBD
//    documentation says); Hatari multiplies.
//  - The absolute report button bits ($0D) are "since last interrogation"
//    event flags set on every button edge (IKBD documentation), cleared by
//    the report, by $07 and by entering absolute mode ($09); Hatari derives
//    them from the current state versus the state at the last read.
//  - Typematic repeats from the PS/2 keyboard are filtered: a make code is
//    only sent if the ST key is not already down (the ST keyboard has no
//    hardware repeat).
//  - Fire button monitoring ($18) is implemented as documented (one byte
//    of 8 joystick 1 fire samples, MSB first, one byte per byte time);
//    Hatari ignores the command.  Joystick keycode mode ($19) is accepted
//    and ignored, as in Hatari.
//  - Output buffer: 32 bytes (real IKBD 20, Hatari 1024).
//  - Memory load ($20) consumes its data bytes; execute ($22) is ignored;
//    memory read ($21) returns $F6 $20 and 6 zero bytes, as Hatari.
//  - The time of day clock is cleared at power up (FPGA configuration) and
//    kept over resets, like Hatari's warm reset (cold reset is not
//    distinguishable from the 'reset' input).  It advances once per second
//    of CLK_HZ.
//  - Double click emulation and the custom 6301 program handlers of Hatari
//    are emulator specific and not implemented.
//
// Implementation notes (area; about 700 ALMs on Cyclone V):
//  - the state machine only assigns constants to its state register (ST_SEND
//    returns through a 3 bit code), so synthesis one-hot encodes it;
//  - the six mouse words (relative X/Y, absolute fractions X/Y, absolute
//    X/Y) live in an 8 x 18 register file RAM; all mouse arithmetic
//    (accumulation, scaling, clamping, threshold residuals, cursor key
//    deltas) goes through one shared 18 bit adder/subtractor whose first
//    operand is the register file read port (one operation per two clocks)
//    and whose second operand is chosen by a 3 bit source code;
//  - the configuration bytes (threshold, scale, keycode deltas, absolute
//    maxima, button action), the time of day clock and the command bytes
//    1..6 live in a 32 x 8 parameter RAM; reports read their bytes from it
//    while they are pushed, parameter commands copy RAM to RAM;
//  - commands are decoded by a 256 entry ROM (length + one-hot command), the
//    PS/2 keymap is a ROM, the key state table (128 x 1) is a RAM cleared in
//    128 clocks at boot;
//  - packets are generated byte by byte while they are pushed (2 bit type +
//    3 data bytes);
//  - all timers are chained from the 125 kHz (16 x baud) tick.

module falcon_ikbd #(
    parameter CLK_HZ      = 32000000,
    parameter AUTOSEND_HZ = 1000,
    parameter VBL_HZ      = 50       // autofire time base (Hatari counts VBLs)
)(
    input             clk,
    input             reset,
    input             rxd,
    output reg        txd,
    input      [10:0] ps2_key,
    input      [24:0] ps2_mouse,
    input      [31:0] joystick_0,
    input      [31:0] joystick_1
);

// ---------------------------------------------------------------------------
// constants (timers count 125 kHz ticks = 8 us)
// ---------------------------------------------------------------------------
localparam integer X16_HZ    = 125000;                       // 16 x 7812.5
localparam integer AUTO_X16  = X16_HZ / AUTOSEND_HZ;
localparam integer AUTO_W    = $clog2(AUTO_X16 + 1);
// IKBD_RESET_CYCLES = 502000 cycles at 8021247 Hz, in 8 us ticks (7823)
localparam [63:0]  RST64     = (64'd502000 * X16_HZ + 64'd4010623) / 64'd8021247;
localparam [12:0]  RESET_X16 = RST64[12:0];

localparam [1:0] M_OFF = 2'd0, M_REL = 2'd1, M_ABS = 2'd2, M_CUR = 2'd3;
localparam [1:0] J_OFF = 2'd0, J_AUTO = 2'd1, J_MON = 2'd2, J_FIRE = 2'd3;

localparam integer FIFO_DEPTH = 32;

// packet types (bytes generated at push time)
localparam [1:0] PT_V = 2'd0, PT_ABS = 2'd1, PT_CLK = 2'd2, PT_F6 = 2'd3;

// parameter RAM map
localparam [4:0] PA_THR = 5'd0, PA_SCL = 5'd2, PA_KCD = 5'd4, PA_MX = 5'd6,  // max Y at 8
                 PA_ACT = 5'd10, PA_CLK = 5'd16, PA_IB = 5'd24;   // command byte k at 24+k

// command ids (command ROM)
localparam [5:0]
    C_NONE = 6'd0,  C_RESET = 6'd1,  C_ACTION = 6'd2,  C_REL = 6'd3,
    C_ABS = 6'd4,   C_KCODE = 6'd5,  C_THR = 6'd6,     C_SCALE = 6'd7,
    C_ABSRD = 6'd8, C_ABSLD = 6'd9,  C_YDOWN = 6'd10,  C_YUP = 6'd11,
    C_RESUME = 6'd12, C_MOFF = 6'd13, C_PAUSE = 6'd14, C_JEVT = 6'd15,
    C_JINT = 6'd16, C_JRD = 6'd17,   C_JMON = 6'd18,   C_JFIRE = 6'd19,
    C_JKEY = 6'd20, C_JOFF = 6'd21,  C_SETCLK = 6'd22, C_RDCLK = 6'd23,
    C_MEMLD = 6'd24, C_MEMRD = 6'd25, C_EXEC = 6'd26,  C_RPACT = 6'd27,
    C_RPMODE = 6'd28, C_RPTHR = 6'd29, C_RPSCL = 6'd30, C_RPVERT = 6'd31,
    C_RPMAV = 6'd32, C_RPJMODE = 6'd33, C_RPJAV = 6'd34;

// return codes of ST_SEND (all assignments to st are constants, so that
// synthesis extracts and one-hot encodes the state machine)
localparam [2:0] R_IDLE = 3'd0, R_AX_PRE = 3'd1, R_JOY1 = 3'd2, R_A_MOUSE = 3'd3,
                 R_A_END = 3'd4, R_C_R = 3'd5, R_C_Z = 3'd6, R_C_BR = 3'd7;

localparam [5:0]
    ST_IDLE    = 6'd0,
    ST_EXEC    = 6'd1,
    ST_SEND    = 6'd2,
    ST_KEY     = 6'd3,
    ST_KEY2    = 6'd4,
    ST_CLR     = 6'd5,
    ST_CK0     = 6'd6,
    ST_MACC    = 6'd7,
    ST_ABSREP  = 6'd8,
    ST_A_START = 6'd9,
    ST_A_ACT   = 6'd10,
    ST_AX_SUB  = 6'd11,
    ST_AX_ADD  = 6'd12,
    ST_AX_N    = 6'd13,
    ST_AX_ALL  = 6'd14,
    ST_APP_B   = 6'd15,
    ST_A_JOY0  = 6'd17,
    ST_A_JOY1  = 6'd19,
    ST_A_MOUSE = 6'd21,
    ST_R_SUBX  = 6'd22,
    ST_R_SUBY  = 6'd23,
    ST_C_L     = 6'd24,
    ST_C_R     = 6'd25,
    ST_C_Z     = 6'd26,
    ST_C_BL    = 6'd27,
    ST_C_BR    = 6'd28,
    ST_A_END   = 6'd29,
    ST_RX      = 6'd30,
    ST_SEND_B  = 6'd31,
    ST_CP      = 6'd32,
    ST_CK1     = 6'd33,
    ST_CK2     = 6'd34,
    ST_CK3     = 6'd35,
    ST_CK4     = 6'd36,
    ST_AX_PRE  = 6'd37,
    ST_AX_LD   = 6'd38,
    ST_APP_H   = 6'd39,
    ST_R_T1    = 6'd40,
    ST_R_T2    = 6'd41,
    ST_C_PRE   = 6'd42,
    ST_C_LD    = 6'd43,
    ST_APP_L   = 6'd44,
    ST_LD0     = 6'd45,
    ST_LD1     = 6'd46,
    ST_END1    = 6'd47,
    ST_CP0     = 6'd48,
    ST_ML0     = 6'd49,
    ST_ML1     = 6'd50,
    ST_LD2     = 6'd51,
    ST_LD3     = 6'd52,
    ST_LD4     = 6'd53,
    ST_SPC     = 6'd54;

// ---------------------------------------------------------------------------
// helpers
// ---------------------------------------------------------------------------
// MiSTer -> ST joystick byte; button 3 (bit 6) is autofire (af = on phase)
function [7:0] jmap(input [31:0] j, input af);
    jmap = {j[6] ? af : j[4], 3'b000, j[0], j[1], j[2], j[3]};
endfunction

// command ROM contents: {length, id}
function [8:0] cmd_rom(input [7:0] c);
    case (c)
    8'h80: cmd_rom = {3'd2, C_RESET};
    8'h07: cmd_rom = {3'd2, C_ACTION};
    8'h08: cmd_rom = {3'd1, C_REL};
    8'h09: cmd_rom = {3'd5, C_ABS};
    8'h0A: cmd_rom = {3'd3, C_KCODE};
    8'h0B: cmd_rom = {3'd3, C_THR};
    8'h0C: cmd_rom = {3'd3, C_SCALE};
    8'h0D: cmd_rom = {3'd1, C_ABSRD};
    8'h0E: cmd_rom = {3'd6, C_ABSLD};
    8'h0F: cmd_rom = {3'd1, C_YDOWN};
    8'h10: cmd_rom = {3'd1, C_YUP};
    8'h11: cmd_rom = {3'd1, C_RESUME};
    8'h12: cmd_rom = {3'd1, C_MOFF};
    8'h13: cmd_rom = {3'd1, C_PAUSE};
    8'h14: cmd_rom = {3'd1, C_JEVT};
    8'h15: cmd_rom = {3'd1, C_JINT};
    8'h16: cmd_rom = {3'd1, C_JRD};
    8'h17: cmd_rom = {3'd2, C_JMON};
    8'h18: cmd_rom = {3'd1, C_JFIRE};
    8'h19: cmd_rom = {3'd7, C_JKEY};
    8'h1A: cmd_rom = {3'd1, C_JOFF};
    8'h1B: cmd_rom = {3'd7, C_SETCLK};
    8'h1C: cmd_rom = {3'd1, C_RDCLK};
    8'h20: cmd_rom = {3'd4, C_MEMLD};
    8'h21: cmd_rom = {3'd3, C_MEMRD};
    8'h22: cmd_rom = {3'd3, C_EXEC};
    8'h87: cmd_rom = {3'd1, C_RPACT};
    8'h88, 8'h89, 8'h8A: cmd_rom = {3'd1, C_RPMODE};
    8'h8B: cmd_rom = {3'd1, C_RPTHR};
    8'h8C: cmd_rom = {3'd1, C_RPSCL};
    8'h8F, 8'h90: cmd_rom = {3'd1, C_RPVERT};
    8'h92: cmd_rom = {3'd1, C_RPMAV};
    8'h94, 8'h95, 8'h99: cmd_rom = {3'd1, C_RPJMODE};
    8'h9A: cmd_rom = {3'd1, C_RPJAV};
    default: cmd_rom = {3'd0, C_NONE};
    endcase
endfunction

// parameter RAM values after IKBD_Boot_ROM
function [7:0] pram_default(input [3:0] a);
    case (a)
    4'd0, 4'd1, 4'd4, 4'd5: pram_default = 8'd1;      // threshold, keycode delta
    4'd6:  pram_default = 8'h01;                        // max X = 320
    4'd7:  pram_default = 8'h40;
    4'd9:  pram_default = 8'hC8;                        // max Y = 200
    default: pram_default = 8'h00;                      // scale, action
    endcase
endfunction

function bcd_ok(input [7:0] v);
    bcd_ok = (v[3:0] <= 4'd9) && (v[7:4] <= 4'd9);
endfunction

function [7:0] bcd_adj(input [7:0] v);
    reg [7:0] t;
    begin
        t = v;
        if (t[3:0] > 4'd9) t = t + 8'h06;
        if (t[7:4] > 4'd9) t = t + 8'h60;
        bcd_adj = t;
    end
endfunction

function [7:0] day_max(input [7:0] m);   // index = BCD month - 1 (0..17)
    case (m)
    8'd0, 8'd2, 8'd4, 8'd6, 8'd7, 8'd15, 8'd17: day_max = 8'h32;
    8'd1:                                       day_max = 8'h29;
    8'd3, 8'd5, 8'd8, 8'd16:                    day_max = 8'h31;
    default:                                    day_max = 8'h00;
    endcase
endfunction

// saturate a 16 bit value to -128..127
function [7:0] sat8(input [15:0] v);
    if (v[15:7] == 9'h000 || v[15:7] == 9'h1FF) sat8 = v[7:0];
    else if (v[15])                             sat8 = 8'h80;
    else                                        sat8 = 8'h7F;
endfunction

// ---------------------------------------------------------------------------
// 125 kHz tick (16 x baud)
// ---------------------------------------------------------------------------
reg x16 = 1'b0;
generate
if ((CLK_HZ % X16_HZ) == 0) begin : g_x16_int
    localparam integer XDIV = CLK_HZ / X16_HZ;
    localparam integer XW   = $clog2(XDIV);
    reg [XW-1:0] xc = {XW{1'b0}};
    always @(posedge clk) begin
        x16 <= (xc == XDIV - 1);
        xc  <= (xc == XDIV - 1) ? {XW{1'b0}} : xc + 1'b1;
    end
end else begin : g_x16_frac
    localparam integer FW = $clog2(CLK_HZ) + 1;
    reg [FW-1:0] bacc = {FW{1'b0}};
    always @(posedge clk) begin
        if (bacc + X16_HZ >= CLK_HZ) begin
            bacc <= bacc + X16_HZ - CLK_HZ;
            x16  <= 1'b1;
        end else begin
            bacc <= bacc + X16_HZ;
            x16  <= 1'b0;
        end
    end
end
endgenerate

// ---------------------------------------------------------------------------
// state
// ---------------------------------------------------------------------------
// SCI receiver
reg  [1:0] rxs = 2'b11;
reg  [1:0] r_st;
reg  [3:0] r_cnt;
reg  [2:0] r_bit;
reg  [7:0] r_sh;
reg        r_armed;
reg        rx_pend;
reg  [7:0] rx_byte;

// SCI transmitter
reg  [3:0] t_sub;
reg  [1:0] t_st;
reg  [7:0] t_sh;
reg  [2:0] t_bit;
reg  [3:0] tx_delay;
reg        pause;

// output buffer (RAM)
reg  [7:0] fifo [0:FIFO_DEPTH-1];
reg  [4:0] f_wr, f_rd;
reg  [5:0] f_cnt;

// command input
reg  [7:0] ib0, ib1;             // bytes 1.. are also in the parameter RAM
reg  [2:0] ib_n;
reg  [7:0] ld_left;

// FSM
reg  [5:0] st;
reg  [2:0] ret, abs_ret;
reg  [1:0] ptype;
reg  [2:0] plen, pidx;
reg  [3:0] pdly;
reg  [7:0] v0, v1, v2;
reg  [7:0] pq;                  // parameter byte latched from the RAM
reg        joy_only;
reg        ax;                  // axis: 0 = X, 1 = Y
reg  [1:0] msel;
reg  [6:0] clr_cnt;
reg  [2:0] cp_k, cp_n;
reg  [4:0] cp_base;
reg        cp_bcd;
reg        relc;

// IKBD processor state
reg  [1:0] mouse_mode, joy_mode;
reg        ydown;
reg  [2:0] act;                 // low bits of the mouse button action
reg  [3:0] absflags;
reg  [7:0] prev_j0, prev_j1;
reg        resetting, mouse_dis, joy_dis, both, men_reset;
reg [12:0] rst_cnt;
reg  [7:0] mon_rate, mon_div;

// time of day clock time base: free running from power up
reg  [6:0] ms_cnt  = 7'd0;      // 125 ticks = 1 ms
reg  [9:0] sec_cnt = 10'd0;     // 1000 ms
reg        sec_pend = 1'b0;
reg  [2:0] ci;
reg  [7:0] ck_mon;
reg        ck_leap;             // leap year test of the IKBD ROM, done when the year is read

// timers / events
reg [AUTO_W-1:0] auto_cnt;
reg [10:0] mon_x;               // 10 ms = 1250 ticks, restarted by $17
// autofire time base: VBL counter (free running from power up)
localparam integer VBL_MS = 1000 / VBL_HZ;
reg  [4:0] vbl_ms  = 5'd0;
reg  [2:0] vbl_cnt = 3'd0;
wire       af_on   = vbl_cnt[2];   // Hatari: fire removed while (nVBLs & 7) < 4
// joystick button 2 = Space key (Hatari JoystickSpaceBar), not reset by
// the IKBD reset, as in Hatari
localparam [1:0] SPC_NULL = 2'd0, SPC_DOWN = 2'd1, SPC_DOWNED = 2'd2, SPC_UP = 2'd3;
reg  [1:0] spc = SPC_NULL;
reg  [4:0] fire_cnt;            // 20 ticks = 160 us = 1/8 byte time
reg        auto_pend, mon_pend, fire_pend, boot_pend;
reg  [7:0] fire_sh;
reg  [2:0] fire_n;

// keyboard event latch
reg        key_tog;
reg        key_pend, key_press, key_ext;
reg  [7:0] key_code;
wire [7:0] key_st;
falcon_ikbd_keymap u_keymap (.clk(clk), .ext(key_ext), .code(key_code), .st(key_st));

// mouse
reg        m_tog;
reg        ml, mr;
reg        n_neg;               // direction of a scaled unit step
reg        ph;                  // ALU states: 0 = read, 1 = execute

// sampled values of an autosend pass
// joystick bytes of an autosend pass: IKBD_GetJoystickData and (not for
// the immediate report of command $14) IKBD_DuplicateMouseFireButtons
wire       m_off = (mouse_mode == M_OFF);
wire [7:0] jm0   = jmap(joystick_0, af_on);
wire [7:0] jm1   = jmap(joystick_1, af_on);
wire [7:0] cj0   = (m_off || (both && mouse_mode == M_REL))
                   ? (jm0 | {!joy_only && m_off && ml, 7'd0}) : 8'd0;
wire [7:0] cj1   = joy_only ? jm1 : {m_off ? (jm1[7] | mr) : 1'b0, jm1[6:0]};
reg        cl, crb;
reg        oa_l, oa_r;
reg        or_l, or_r;

wire       m_new = ps2_mouse[24] != m_tog;
wire       pkt2_fits = !resetting && (f_cnt <= FIFO_DEPTH - 2);   // same test as ST_SEND

initial txd = 1'b1;

// ---------------------------------------------------------------------------
// command ROM (256 x 9), address = first byte of the command
// ---------------------------------------------------------------------------
wire [7:0] c_addr = (ib_n == 3'd0) ? rx_byte : ib0;
// ROM word: {one-hot command vector (bit = command id), length}.  In ST_RX
// c_q is the entry of the first command byte; in ST_EXEC it still is (the
// address was taken in ST_RX, before ib_n is cleared).
function [37:0] crom_word(input [7:0] c);
    reg [8:0] e;
    begin
        e = cmd_rom(c);
        crom_word = {((35'd1 << e[5:0]) & ~35'd1), e[8:6]};
    end
endfunction
reg [37:0] c_q;
reg [37:0] crom [0:255];
integer ci_i;
initial for (ci_i = 0; ci_i < 256; ci_i = ci_i + 1) crom[ci_i] = crom_word(ci_i[7:0]);
always @(posedge clk) c_q <= crom[c_addr];
wire [34:0] oh  = c_q[37:3];
wire [2:0]  clen = c_q[2:0];

// ---------------------------------------------------------------------------
// key state RAM (128 x 1)
// ---------------------------------------------------------------------------
reg        kram [0:127];
reg  [6:0] k_addr;
reg        k_we, k_d, k_q;
always @(*) begin
    k_addr = key_st[6:0];
    k_we   = 1'b0;
    k_d    = 1'b0;
    if (st == ST_CLR) begin
        k_addr = clr_cnt;
        k_we   = 1'b1;
    end else if (st == ST_KEY2) begin
        k_d    = key_press;
        k_we   = key_press ? !k_q : k_q;
    end
end
always @(posedge clk) begin
    if (k_we) kram[k_addr] <= k_d;
    k_q <= kram[k_addr];
end

// ---------------------------------------------------------------------------
// parameter RAM (32 x 8): 0/1 threshold X/Y, 2/3 scale X/Y, 4/5 keycode
// delta X/Y, 6..9 absolute max X/Y (MSB first), 10 button action,
// 16..21 time of day (YY MM DD hh mm ss, BCD; cleared at power up only)
// ---------------------------------------------------------------------------
(* ramstyle = "M10K" *) reg [7:0] pram [0:31];
integer pi;
initial for (pi = 0; pi < 32; pi = pi + 1) pram[pi] = 8'h00;
reg  [4:0] p_ra, p_wa;
reg  [7:0] p_wd;
reg        p_we;
reg  [7:0] p_q;
always @(posedge clk) begin
    if (p_we) pram[p_wa] <= p_wd;
    p_q <= pram[p_ra];
end

// mouse register file (8 x 18, signed): 0/1 relative X/Y accumulators
// (ST orientation, Y down), 2/3 absolute position fractions X/Y, 4/5
// absolute position X/Y.  Cleared in ST_CLR at boot.
localparam [2:0] RF_RX = 3'd0, RF_AF = 3'd2, RF_AB = 3'd4;
localparam [2:0] WS_ZERO = 3'd0, WS_SAT = 3'd1, WS_RAW = 3'd2, WS_MAX = 3'd5;
reg [17:0] rf [0:7];
reg  [2:0] rf_ra;
reg [17:0] rf_q;
always @(posedge clk) rf_q <= rf[rf_ra];
always @(*) begin
    case (st)
    ST_MACC:                         rf_ra = {1'b0, msel};
    ST_AX_SUB, ST_AX_ADD:            rf_ra = RF_AF + {2'b00, ax};
    ST_AX_N, ST_AX_ALL,
    ST_APP_L, ST_APP_B:              rf_ra = RF_AB + {2'b00, ax};
    ST_A_MOUSE, ST_R_SUBX:           rf_ra = RF_RX;
    ST_R_T1, ST_R_SUBY:              rf_ra = RF_RX + 3'd1;
    ST_C_L, ST_C_R:                  rf_ra = RF_RX + {2'b00, ax};
    ST_SEND:                         rf_ra = (pidx >= 3'd4) ? RF_AB + 3'd1 : RF_AB;
    default:                         rf_ra = RF_RX;
    endcase
end

// bytes of the $F6 status reports that come from the parameter RAM
reg  [4:0] f6_base;
reg  [2:0] f6_cnt;
always @(*) begin
    case (v0)
    8'h07:   begin f6_base = PA_ACT; f6_cnt = 3'd1; end
    8'h09:   begin f6_base = PA_MX;  f6_cnt = 3'd4; end
    8'h0A:   begin f6_base = PA_KCD; f6_cnt = 3'd2; end
    8'h0B:   begin f6_base = PA_THR; f6_cnt = 3'd2; end
    8'h0C:   begin f6_base = PA_SCL; f6_cnt = 3'd2; end
    default: begin f6_base = 5'd0;   f6_cnt = 3'd0; end
    endcase
end
wire [2:0] f6_i = pidx - 3'd2;


// clock update (IKBD_UpdateClockOnVBL) for byte ci, value in p_q
reg  [7:0] ck_v, ck_max;
reg        ck_wrap;
always @(*) begin
    ck_v = bcd_adj(p_q + 8'd1);
    case (ci)
    3'd0: ck_max = 8'hFF;
    3'd1: ck_max = 8'h13;
    3'd3: ck_max = 8'h24;
    3'd4: ck_max = 8'h60;
    3'd5: ck_max = 8'h60;
    default: begin
        ck_max = day_max(((ck_mon > 8'h12) ? 8'h12 : ck_mon) - 8'd1);
        if (ck_mon == 8'h02) begin
            if (ck_leap) ck_max = 8'h30;
        end
    end
    endcase
    ck_wrap = (ck_v == ck_max);
    if (ck_wrap) ck_v = (ci == 3'd1 || ci == 3'd2) ? 8'h01 : 8'h00;
end

// parameter RAM ports
always @(*) begin
    p_ra = 5'd0;
    p_we = 1'b0;
    p_wa = 5'd0;
    p_wd = 8'd0;
    case (st)
    ST_SEND:   p_ra = (ptype == PT_CLK) ? PA_CLK + {2'b00, pidx} - 5'd1
                                        : f6_base + {2'b00, f6_i};
    ST_A_MOUSE: p_ra = PA_THR;
    ST_R_T1:   p_ra = PA_THR + 5'd1;
    ST_AX_PRE: p_ra = PA_SCL + {4'd0, ax};
    ST_C_PRE:  p_ra = PA_KCD + {4'd0, ax};
    ST_APP_H:  p_ra = PA_MX + {3'd0, ax, 1'b0};
    ST_APP_L:  p_ra = PA_MX + {3'd0, ax, 1'b1};
    ST_CK0:    p_ra = PA_CLK + 5'd1;
    ST_CK1:    p_ra = PA_CLK;
    ST_CK3:    p_ra = PA_CLK + {2'b00, ci};
    ST_CP0:    p_ra = PA_IB + 5'd1;
    ST_CP:     p_ra = PA_IB + {2'b00, cp_k} + 5'd1;
    ST_ML0:    p_ra = PA_IB + 5'd3;
    ST_LD0:    p_ra = PA_IB + 5'd2;
    ST_LD1:    p_ra = PA_IB + 5'd3;
    ST_LD2:    p_ra = PA_IB + 5'd4;
    ST_LD3:    p_ra = PA_IB + 5'd5;
    default:   p_ra = 5'd0;
    endcase
    if (st == ST_CLR && clr_cnt[6:4] == 3'd0) begin
        p_we = 1'b1;
        p_wa = {1'b0, clr_cnt[3:0]};
        p_wd = pram_default(clr_cnt[3:0]);
    end else if (st == ST_RX) begin             // command bytes 1..
        p_we = (ld_left == 8'd0) && (ib_n != 3'd0);
        p_wa = PA_IB + {2'b00, ib_n};
        p_wd = rx_byte;
    end else if (st == ST_CP) begin             // p_q = command byte cp_k
        p_we = !cp_bcd || bcd_ok(p_q);
        p_wa = cp_base + {2'b00, cp_k} - 5'd1;
        p_wd = p_q;
    end else if (st == ST_CK4) begin
        p_we = 1'b1;
        p_wa = PA_CLK + {2'b00, ci};
        p_wd = ck_v;
    end
end

// relative mouse packet decision (IKBD_SendRelMousePacket): the
// accumulator (register file) and its threshold (parameter RAM) are read
// together, X in ST_R_T1, Y in ST_R_T2
wire [7:0] s8      = sat8(rf_q[15:0]);
wire [7:0] m8      = s8[7] ? (~s8 + 8'd1) : s8;
wire       thr_hit = (s8 != 8'd0) && (m8 >= p_q);

// ---------------------------------------------------------------------------
// shared ALU: alu_r = alu_a +/- alu_b (18 bit signed), alu_s = saturated to 16
// ---------------------------------------------------------------------------
reg  [17:0] alu_a, alu_b;
reg         alu_sub;
wire [17:0] alu_r = alu_a + (alu_sub ? ~alu_b : alu_b) + {17'd0, alu_sub};
wire        alu_neg  = alu_r[17];
wire        alu_zero = (alu_r == 18'd0);
reg  [15:0] alu_s;
always @(*) begin
    if (!alu_r[17] && alu_r[16:14] != 3'b000)      alu_s = 16'h3FFF;   // > 16383
    else if (alu_r[17] && alu_r[16:14] != 3'b111)  alu_s = 16'hC000;   // < -16384
    else                                           alu_s = alu_r[15:0];
end

// operand B: a 3 bit source code decoded from the state (kept as its own
// net so that synthesis does not fold the state decode into every bit)
localparam [2:0] B_ZERO = 3'd0, B_MD = 3'd1, B_PQ = 3'd2, B_ONE = 3'd3,
                 B_T18 = 3'd4, B_MAX = 3'd5, B_S8 = 3'd6;
reg  [2:0] bsel_c;
reg        bsub_c;
always @(*) begin
    bsel_c = B_ZERO;
    bsub_c = 1'b0;
    case (st)
    ST_MACC:              begin bsel_c = B_MD;  bsub_c = msel[0]; end   // ST Y is down: subtract PS/2 dy
    ST_AX_SUB, ST_C_R:    begin bsel_c = B_PQ;  bsub_c = 1'b1; end
    ST_AX_ADD, ST_C_L:          bsel_c = B_PQ;
    ST_AX_N:              begin bsel_c = B_ONE; bsub_c = n_neg ^ (ax & ydown); end
    ST_AX_ALL:            begin bsel_c = B_T18; bsub_c = ax & ydown; end
    ST_APP_B:             begin bsel_c = B_MAX; bsub_c = 1'b1; end
    ST_R_SUBX, ST_R_SUBY: begin bsel_c = B_S8;  bsub_c = 1'b1; end   // acc - sat8(acc)
    default: ;
    endcase
end
(* keep *) wire [2:0] bsel = bsel_c;
(* keep *) wire       bsub = bsub_c;
// PS/2 deltas are read from ps2_mouse during the 8 clocks of ST_MACC (the
// next packet is at least a PS/2 byte time away)
wire [8:0] md = msel[0] ? {ps2_mouse[5], ps2_mouse[23:16]} : {ps2_mouse[4], ps2_mouse[15:8]};
always @(*) begin
    alu_a   = rf_q;
    alu_sub = bsub;
    case (bsel)
    B_MD:    alu_b = {{9{md[8]}}, md};
    B_PQ:    alu_b = {10'd0, pq};
    B_ONE:   alu_b = 18'd1;
    B_T18:   alu_b = {{2{v1[7]}}, v1, v2};
    B_MAX:   alu_b = {2'b00, pq, p_q};
    B_S8:    alu_b = {{10{s8[7]}}, s8};
    default: alu_b = 18'd0;
    endcase
end

// ---------------------------------------------------------------------------
// packet byte generator (ST_SEND_B; parameter RAM bytes arrive in p_q)
// ---------------------------------------------------------------------------
reg [7:0] pbyte;
always @(*) begin
    pbyte = 8'h00;
    case (ptype)
    PT_V: pbyte = (pidx == 3'd0) ? v0 : (pidx == 3'd1) ? v1 : v2;
    PT_ABS: case (pidx)         // position bytes from the register file
        3'd0: pbyte = 8'hF7;
        3'd1: pbyte = v0;
        default: pbyte = pidx[0] ? rf_q[7:0] : rf_q[15:8];
        endcase
    PT_CLK: pbyte = (pidx == 3'd0) ? 8'hFC : p_q;
    default: begin                // PT_F6 status reports, v0 = report code
        if (pidx == 3'd0)        pbyte = 8'hF6;
        else if (pidx == 3'd1)   pbyte = v0;
        else if (f6_i < f6_cnt)  pbyte = p_q;
        else                     pbyte = 8'h00;
    end
    endcase
end


// ---------------------------------------------------------------------------
// main process
// ---------------------------------------------------------------------------
reg        push, pop;
reg        do_boot;
reg  [2:0] n_new;
reg        press_l, rel_l, press_r, rel_r;
reg  [7:0] j0t, j1t;
reg        rf_we;               // register file write port
reg  [2:0] rf_wa;
reg  [2:0] rf_ws;               // write data source
reg [17:0] rf_wd;

task automatic snd(input [1:0] t, input [2:0] len, input [3:0] dly, input [2:0] r);
    begin
        ptype <= t;
        plen  <= len;
        pdly  <= dly;
        pidx  <= 3'd0;
        ret   <= r;
        st    <= ST_SEND;
    end
endtask

task automatic go_ret(input [2:0] r);
    case (r)
    R_AX_PRE: st <= ST_AX_PRE;
    R_JOY1:    st <= ST_A_JOY1;
    R_A_MOUSE: st <= ST_A_MOUSE;
    R_A_END:  st <= ST_A_END;
    R_C_R:    st <= ST_C_R;
    R_C_Z:    st <= ST_C_Z;
    R_C_BR:   st <= ST_C_BR;
    default:  st <= ST_IDLE;
    endcase
endtask

task automatic f6(input [7:0] code);
    begin
        v0 <= code;
        snd(PT_F6, 3'd0, 4'd7, R_IDLE);
    end
endtask

// Joy_ButtonSpaceJump: button 2 sampled on a port read
task automatic spc_read(input press);
    begin
        if (press && spc == SPC_NULL)        spc <= SPC_DOWN;
        else if (!press && spc == SPC_DOWNED) spc <= SPC_UP;
    end
endtask

task automatic copy(input [4:0] base, input [2:0] n, input bcd);
    begin
        cp_base <= base;
        cp_n    <= n;
        cp_k    <= 3'd1;
        cp_bcd  <= bcd;
        st      <= ST_CP0;
    end
endtask

always @(posedge clk) begin
    push    = 1'b0;
    pop     = 1'b0;
    do_boot = 1'b0;
    rf_we   = 1'b0;
    rf_wa   = 3'd0;
    rf_ws   = WS_ZERO;

    rxs <= {rxs[0], rxd};

    // ------------------------------------------------------------------
    // timers (all on the 125 kHz tick)
    // ------------------------------------------------------------------
    if (x16) begin
        if (ms_cnt == 7'd124) begin
            ms_cnt <= 7'd0;
            if (sec_cnt == 10'd999) begin
                sec_cnt  <= 10'd0;
                sec_pend <= 1'b1;
            end else begin
                sec_cnt <= sec_cnt + 10'd1;
            end
            if (vbl_ms == VBL_MS - 1) begin
                vbl_ms  <= 5'd0;
                vbl_cnt <= vbl_cnt + 3'd1;
            end else begin
                vbl_ms <= vbl_ms + 5'd1;
            end
        end else begin
            ms_cnt <= ms_cnt + 7'd1;
        end

        // joystick monitoring period (rate x 10 ms from the $17 command)
        if (mon_x == 11'd1249) begin
            mon_x <= 11'd0;
            if (joy_mode == J_MON) begin
                if (mon_div + 8'd1 >= mon_rate) begin
                    mon_div  <= 8'd0;
                    mon_pend <= 1'b1;
                end else begin
                    mon_div <= mon_div + 8'd1;
                end
            end
        end else begin
            mon_x <= mon_x + 11'd1;
        end

        if (AUTO_X16 == 125) begin
            if (ms_cnt == 7'd124) auto_pend <= 1'b1;   // 1 ms: share the ms divider
        end else if (auto_cnt == AUTO_X16 - 1) begin
            auto_cnt  <= {AUTO_W{1'b0}};
            auto_pend <= 1'b1;
        end else begin
            auto_cnt <= auto_cnt + 1'b1;
        end

        if (fire_cnt == 5'd19) begin
            fire_cnt <= 5'd0;
            if (joy_mode == J_FIRE) begin
                fire_sh <= {fire_sh[6:0], jm1[7] | mr};
                fire_n  <= fire_n + 3'd1;
                if (fire_n == 3'd7)
                    fire_pend <= 1'b1;      // fire_sh complete for 160 us
            end else begin
                fire_n <= 3'd0;
            end
        end else begin
            fire_cnt <= fire_cnt + 5'd1;
        end

        if (resetting) begin
            if (rst_cnt == 13'd0) boot_pend <= 1'b1;
            else                  rst_cnt <= rst_cnt - 13'd1;
        end
    end

    // ------------------------------------------------------------------
    // keyboard event latch
    // ------------------------------------------------------------------
    if (ps2_key[10] != key_tog) begin
        key_tog   <= ps2_key[10];
        key_pend  <= 1'b1;
        key_press <= ps2_key[9];
        key_ext   <= ps2_key[8];
        key_code  <= ps2_key[7:0];
    end

    // ------------------------------------------------------------------
    // SCI receiver (IKBD_SCI_Get_Line_RX)
    // ------------------------------------------------------------------
    if (x16) begin
        case (r_st)
        2'd0: begin
            if (rxs[1]) r_armed <= 1'b1;
            if (r_armed && !rxs[1]) begin
                r_st  <= 2'd1;
                r_cnt <= 4'd0;
            end
        end
        2'd1: begin
            r_cnt <= r_cnt + 4'd1;
            if (r_cnt == 4'd7) begin
                r_cnt <= 4'd0;
                r_bit <= 3'd0;
                r_st  <= rxs[1] ? 2'd0 : 2'd2;
            end
        end
        2'd2: begin
            r_cnt <= r_cnt + 4'd1;
            if (r_cnt == 4'd15) begin
                r_sh  <= {rxs[1], r_sh[7:1]};
                r_bit <= r_bit + 3'd1;
                if (r_bit == 3'd7) r_st <= 2'd3;
            end
        end
        default: begin
            r_cnt <= r_cnt + 4'd1;
            if (r_cnt == 4'd15) begin
                if (rxs[1]) begin
                    rx_byte <= r_sh;
                    rx_pend <= 1'b1;
                end else begin
                    r_armed <= 1'b0;   // framing error: byte ignored
                end
                r_st <= 2'd0;
            end
        end
        endcase
    end

    // ------------------------------------------------------------------
    // SCI transmitter (IKBD_SCI_Set_Line_TX)
    // ------------------------------------------------------------------
    if (x16) begin
        t_sub <= t_sub + 4'd1;
        if (t_sub == 4'd15) begin
            case (t_st)
            2'd0: begin
                txd <= 1'b1;
                if (tx_delay != 4'd0) begin
                    tx_delay <= tx_delay - 4'd1;
                end else if (f_cnt != 6'd0 && !pause) begin
                    t_sh  <= fifo[f_rd];
                    f_rd  <= f_rd + 5'd1;
                    pop   = 1'b1;
                    t_bit <= 3'd0;
                    txd   <= 1'b0;         // start bit
                    t_st  <= 2'd1;
                end
            end
            2'd1: begin
                txd   <= t_sh[0];
                t_sh  <= {1'b0, t_sh[7:1]};
                t_bit <= t_bit + 3'd1;
                if (t_bit == 3'd7) t_st <= 2'd2;
            end
            default: begin
                txd  <= 1'b1;             // stop bit
                t_st <= 2'd0;
            end
            endcase
        end
    end

    // ------------------------------------------------------------------
    // command / event processor
    // ------------------------------------------------------------------
    case (st)
    ST_IDLE: begin
        if (rx_pend) begin
            rx_pend <= 1'b0;
            st      <= ST_RX;               // command ROM lookup
        end else if (key_pend) begin
            key_pend <= 1'b0;
            st       <= ST_KEY;              // keymap ROM lookup
        end else if (m_new) begin
            m_tog <= ps2_mouse[24];
            ml    <= ps2_mouse[0];
            mr    <= ps2_mouse[1];
            msel  <= 2'd0;
            st    <= ST_MACC;
        end else if (boot_pend) begin
            // IKBD_InterruptHandler_ResetTimer
            boot_pend <= 1'b0;
            resetting <= 1'b0;
            men_reset <= 1'b0;
            v0 <= 8'hF1;
            snd(PT_V, 3'd1, 4'd1, R_IDLE);
        end else if (sec_pend) begin
            sec_pend <= 1'b0;
            st       <= ST_CK0;
        end else if (mon_pend) begin
            // IKBD_SendAutoJoysticksMonitoring
            mon_pend <= 1'b0;
            if (joy_mode == J_MON && !resetting) begin
                j0t = jm0 | {ml, 7'd0};
                j1t = jm1 | {mr, 7'd0};
                v0 <= {6'd0, j0t[7], j1t[7]};
                v1 <= {j0t[3:0], j1t[3:0]};
                snd(PT_V, 3'd2, 4'd0, R_IDLE);
            end
        end else if (fire_pend) begin
            fire_pend <= 1'b0;
            if (joy_mode == J_FIRE) begin
                v0 <= fire_sh;
                snd(PT_V, 3'd1, 4'd0, R_IDLE);
            end
        end else if (auto_pend) begin
            auto_pend <= 1'b0;
            if (!resetting) begin
                joy_only <= 1'b0;
                st       <= ST_A_START;
            end
        end
    end

    // IKBD_RunKeyboardCommand (no command is longer than 7 bytes, so the
    // 8th byte of Hatari's input buffer is never used); c_q = ROM entry
    ST_RX: begin
        st <= ST_IDLE;
        if (ld_left != 8'd0) begin
            ld_left <= ld_left - 8'd1;        // IKBD_LoadMemoryByte
        end else begin
            n_new = ib_n + 3'd1;
            case (ib_n)
            3'd0: ib0 <= rx_byte;
            3'd1: ib1 <= rx_byte;
            default: ;
            endcase
            if (clen == 3'd0) begin
                ib_n <= 3'd0;
            end else if (clen == n_new) begin
                pause <= 1'b0;
                ib_n  <= 3'd0;
                st    <= ST_EXEC;
            end else begin
                ib_n <= n_new;
            end
        end
    end

    // mouse packet accumulation: rel X, rel Y, abs fraction X, Y
    ST_MACC: begin
        ph <= ~ph;
        if (ph) begin
            rf_we = 1'b1;
            rf_wa = {1'b0, msel};
            rf_ws = resetting ? WS_ZERO : WS_SAT;
            msel  <= msel + 2'd1;
            if (msel == 2'd3) st <= ST_IDLE;
        end
    end

    // ---------------------------------------------------------------
    ST_EXEC: begin
        st <= ST_IDLE;
        if (oh[C_RESET]) if (ib1 == 8'h01) do_boot = 1'b1;
        if (oh[C_ACTION]) begin
            act      <= ib1[2:0];
            absflags <= 4'd0;
            copy(PA_ACT, 3'd1, 1'b0);
        end
        if (oh[C_REL]) begin
            mouse_mode <= M_REL;
            if (resetting) men_reset <= 1'b1;
        end
        if (oh[C_ABS]) begin
            mouse_mode <= M_ABS;
            absflags   <= 4'd0;    // no stale edges from relative mode (Hatari reports 0)
            copy(PA_MX, 3'd4, 1'b0);
        end
        if (oh[C_KCODE]) begin
            mouse_mode <= M_CUR;
            copy(PA_KCD, 3'd2, 1'b0);
        end
        if (oh[C_THR]) copy(PA_THR, 3'd2, 1'b0);
        if (oh[C_SCALE]) copy(PA_SCL, 3'd2, 1'b0);
        if (oh[C_ABSRD]) begin abs_ret <= R_IDLE; st <= ST_ABSREP; end
        if (oh[C_ABSLD]) st <= ST_LD0;
        if (oh[C_YDOWN]) ydown <= 1'b1;
        if (oh[C_YUP]) ydown <= 1'b0;
        if (oh[C_RESUME]) pause <= 1'b0;
        if (oh[C_MOFF]) begin
            mouse_mode <= M_OFF;
            mouse_dis  <= 1'b1;
            if (joy_dis && resetting) begin                 // IKBD_CheckResetDisableBug
                mouse_mode <= M_REL;
                joy_mode   <= J_AUTO;
                both       <= 1'b1;
            end
        end
        if (oh[C_PAUSE]) if (!resetting) pause <= 1'b1;
        if (oh[C_JEVT]) begin
            joy_mode   <= J_AUTO;
            mouse_mode <= M_OFF;
            if (resetting && (men_reset || mouse_dis)) begin
                mouse_mode <= M_REL;
                both       <= 1'b1;
            end
            prev_j0  <= 8'd0;
            prev_j1  <= 8'd0;
            joy_only <= 1'b1;
            st       <= ST_A_START;
        end
        if (oh[C_JINT]) joy_mode <= J_OFF;
        if (oh[C_JRD]) begin
            v0 <= 8'hFD;
            v1 <= jm0;
            v2 <= jm1;
            spc_read(joystick_0[5] | joystick_1[5]);      // both ports are read
            snd(PT_V, 3'd3, 4'd8, R_IDLE);
        end
        if (oh[C_JMON]) begin
            joy_mode   <= J_MON;
            mouse_mode <= M_OFF;
            mon_rate   <= (ib1 == 8'd0) ? 8'd1 : ib1;
            mon_div    <= 8'd0;
            mon_pend   <= 1'b0;
            mon_x      <= 11'd0;   // first report rate x 10 ms after the command (Hatari)
        end
        if (oh[C_JFIRE]) begin
            joy_mode   <= J_FIRE;
            mouse_mode <= M_OFF;
            fire_n     <= 3'd0;
            fire_pend  <= 1'b0;
        end
        if (oh[C_JOFF]) begin
            joy_mode <= J_OFF;
            joy_dis  <= 1'b1;
            if (mouse_dis && resetting) begin
                mouse_mode <= M_REL;
                joy_mode   <= J_AUTO;
                both       <= 1'b1;
            end
        end
        if (oh[C_SETCLK]) copy(PA_CLK, 3'd6, 1'b1);
        if (oh[C_RDCLK]) snd(PT_CLK, 3'd7, 4'd7, R_IDLE);
        if (oh[C_MEMLD]) st <= ST_ML0;
        if (oh[C_MEMRD]) f6(8'h20);
        if (oh[C_RPACT]) f6(8'h07);
        if (oh[C_RPMODE]) begin
            case (mouse_mode)
            M_REL: f6(8'h08);
            M_ABS: f6(8'h09);
            M_CUR: f6(8'h0A);
            // Hatari sends only the $F6 header when the mouse is off
            default: snd(PT_F6, 3'd1, 4'd7, R_IDLE);
            endcase
        end
        if (oh[C_RPTHR]) f6(8'h0B);
        if (oh[C_RPSCL]) f6(8'h0C);
        if (oh[C_RPVERT]) f6(ydown ? 8'h0F : 8'h10);
        if (oh[C_RPMAV]) f6((mouse_mode == M_OFF) ? 8'h12 : 8'h00);
        if (oh[C_RPJMODE]) f6((joy_mode == J_AUTO) ? 8'h14 : 8'h15);
        if (oh[C_RPJAV]) f6((joy_mode == J_OFF) ? 8'h1A : 8'h00);

    end

    // MEMORY LOAD: byte count = command byte 3
    ST_ML0: st <= ST_ML1;
    ST_ML1: begin
        ld_left <= p_q;
        st      <= ST_IDLE;
    end

    // LOAD MOUSE POSITION: command bytes 2..5 -> absolute X and Y
    ST_LD0: st <= ST_LD1;
    ST_LD1: begin
        pq <= p_q;
        st <= ST_LD2;
    end
    ST_LD2: begin
        rf_we = 1'b1;
        rf_wa = RF_AB;
        rf_ws = WS_MAX;
        st    <= ST_LD3;
    end
    ST_LD3: begin
        pq <= p_q;
        st <= ST_LD4;
    end
    ST_LD4: begin
        rf_we = 1'b1;
        rf_wa = RF_AB + 3'd1;
        rf_ws = WS_MAX;
        st    <= ST_IDLE;
    end

    // copy command bytes to the parameter RAM (one per clock, the read of
    // byte k+1 overlaps the write of byte k)
    ST_CP0: st <= ST_CP;
    ST_CP: begin
        cp_k <= cp_k + 3'd1;
        if (cp_k == cp_n) st <= ST_IDLE;
    end

    // ---------------------------------------------------------------
    // push a packet if it fits completely (IKBD_OutputBuffer_CheckFreeCount)
    // plen 0 means 8 bytes.  ST_SEND addresses the parameter RAM,
    // ST_SEND_B pushes the byte.
    ST_SEND: begin
        if (pidx == 3'd0 &&
            (resetting || (FIFO_DEPTH - f_cnt) < {plen == 3'd0, plen})) begin
            go_ret(ret);
        end else begin
            st <= ST_SEND_B;
        end
    end
    ST_SEND_B: begin
        push = 1'b1;
        if (pidx == 3'd0 && pdly != 4'd0)
            tx_delay <= pdly;
        if (pidx == plen - 3'd1) begin
            go_ret(ret);
        end else begin
            st <= ST_SEND;
        end
        pidx <= pidx + 3'd1;
    end

    // ---------------------------------------------------------------
    // IKBD_PressSTKey (key_st from the keymap ROM; the key state RAM is read
    // in ST_KEY and updated in ST_KEY2)
    ST_KEY: begin
        if (key_st != 8'hFF && joy_mode != J_MON && joy_mode != J_FIRE)
            st <= ST_KEY2;
        else
            st <= ST_IDLE;
    end
    ST_KEY2: begin
        st <= ST_IDLE;
        if (key_press != k_q) begin
            v0 <= {!key_press, key_st[6:0]};
            snd(PT_V, 3'd1, 4'd0, R_IDLE);
        end
    end

    ST_CLR: begin
        clr_cnt <= clr_cnt + 7'd1;
        if (clr_cnt == 7'd127) st <= ST_IDLE;
    end

    // ---------------------------------------------------------------
    // IKBD_UpdateClockOnVBL: read month and year, then update second,
    // minute, ... while a byte wraps (read in ST_CK3, write in ST_CK4)
    ST_CK0: st <= ST_CK1;                     // reading month
    ST_CK1: begin                             // month in p_q, reading year
        ck_mon <= p_q;
        st     <= (p_q == 8'h00) ? ST_IDLE : ST_CK2;
    end
    ST_CK2: begin
        ck_leap <= (p_q[4] ? p_q[1:0] + 2'd2 : p_q[1:0]) == 2'd0;   // (year + $0A if bit 4) & 3 == 0
        ci      <= 3'd5;
        st      <= ST_CK3;
    end
    ST_CK3: st <= ST_CK4;
    ST_CK4: begin
        if (!ck_wrap || ci == 3'd0) begin
            st <= ST_IDLE;
        end else begin
            ci <= ci - 3'd1;
            st <= ST_CK3;
        end
    end

    // ---------------------------------------------------------------
    // IKBD_Cmd_ReadAbsMousePos
    ST_ABSREP: begin
        v0 <= {4'd0, absflags};
        absflags <= 4'd0;
        snd(PT_ABS, 3'd6, 4'd10, abs_ret);
    end

    // ---------------------------------------------------------------
    // autosend pass (IKBD_SendAutoKeyboardCommands)
    // (the joystick bytes of the pass, cj0/cj1, are combinational; ml/mr and
    // the modes cannot change during a pass)
    ST_A_START: begin
        ax <= 1'b0;
        // ports read by IKBD_GetJoystickData: 1 always, 0 when not the mouse
        spc_read(joystick_1[5] | ((m_off || (both && mouse_mode == M_REL)) && joystick_0[5]));
        if (joy_only) begin
            st  <= ST_A_JOY0;
        end else begin
            cl  <= ml;
            // IKBD_DuplicateMouseFireButtons: joystick 1 fire is the right
            // button while the mouse is on
            crb <= mr | (mouse_mode != M_OFF && jm1[7]);
            st  <= ST_A_ACT;
        end
    end

    // IKBD_SendOnMouseAction
    ST_A_ACT: begin
        press_l = cl && !oa_l;
        rel_l   = !cl && oa_l;
        press_r = crb && !oa_r;
        rel_r   = !crb && oa_r;
        absflags <= absflags | {rel_l, press_l, rel_r, press_r};
        oa_l <= cl;
        oa_r <= crb;
        st   <= ST_AX_PRE;
        if (act[2]) begin
            if (press_l || rel_l) begin
                v0 <= press_l ? 8'h74 : 8'hF4;
                v1 <= press_r ? 8'h75 : 8'hF5;
                snd(PT_V, (press_r || rel_r) ? 3'd2 : 3'd1, 4'd0, R_AX_PRE);
            end else if (press_r || rel_r) begin
                v0 <= press_r ? 8'h75 : 8'hF5;
                snd(PT_V, 3'd1, 4'd0, R_AX_PRE);
            end
        end else if (act[1:0] != 2'b00) begin
            if (((act[0] && (press_l || press_r)) || (act[1] && (rel_l || rel_r))) &&
                mouse_mode == M_ABS) begin
                abs_ret <= R_AX_PRE;
                st      <= ST_ABSREP;
            end
        end
    end

    // IKBD_UpdateInternalMousePosition (scaled: scale = ticks per unit),
    // X then Y, accumulated directly into the absolute position and
    // clamped afterwards; pq = scale of the axis
    ST_AX_PRE: st <= ST_AX_LD;
    ST_AX_LD: begin
        pq <= p_q;
        st <= ST_AX_SUB;
    end
    ST_AX_SUB: begin                          // rf_q = fraction
        ph <= ~ph;
        if (ph) begin
            {v1, v2} <= rf_q[15:0];           // fraction (16 bit) for ST_AX_ALL
            if (pq <= 8'd1) begin             // unscaled: position += fraction
                rf_we = 1'b1;
                rf_wa = RF_AF + {2'b00, ax};
                rf_ws = WS_ZERO;
                st    <= ST_AX_ALL;
            end else if (!alu_neg) begin      // fraction - scale >= 0
                rf_we = 1'b1;
                rf_wa = RF_AF + {2'b00, ax};
                rf_ws = WS_RAW;
                n_neg <= 1'b0;
                st    <= ST_AX_N;
            end else begin
                st <= ST_AX_ADD;
            end
        end
    end
    ST_AX_ADD: begin
        ph <= ~ph;
        if (ph) begin
            if (alu_neg || alu_zero) begin    // fraction + scale <= 0
                rf_we = 1'b1;
                rf_wa = RF_AF + {2'b00, ax};
                rf_ws = WS_RAW;
                n_neg <= 1'b1;
                st    <= ST_AX_N;
            end else begin
                ax <= ~ax;
                st <= ax ? ST_APP_H : ST_AX_PRE;
            end
        end
    end
    ST_AX_N: begin                            // position +/- 1 (Y inverted if Y=0 at bottom)
        ph <= ~ph;
        if (ph) begin
            rf_we = 1'b1;
            rf_wa = RF_AB + {2'b00, ax};
            rf_ws = WS_RAW;
            st    <= ST_AX_SUB;
        end
    end
    ST_AX_ALL: begin                          // position +/- fraction
        ph <= ~ph;
        if (ph) begin
            rf_we = 1'b1;
            rf_wa = RF_AB + {2'b00, ax};
            rf_ws = WS_RAW;
            ax    <= ~ax;
            st    <= ax ? ST_APP_H : ST_AX_PRE;
        end
    end

    // clamp to 0..max (max read from the parameter RAM, MSB then LSB)
    ST_APP_H: st <= ST_APP_L;
    ST_APP_L: begin
        pq <= p_q;
        st <= ST_APP_B;
    end
    ST_APP_B: begin                           // rf_q = position, {pq, p_q} = max
        if (rf_q[17]) begin
            rf_we = 1'b1;
            rf_ws = WS_ZERO;
        end else if (!alu_neg && !alu_zero) begin
            rf_we = 1'b1;
            rf_ws = WS_MAX;
        end
        rf_wa = RF_AB + {2'b00, ax};
        ax <= ~ax;
        if (!ax)                                          st <= ST_APP_H;
        else if (joy_mode == J_MON || joy_mode == J_FIRE) st <= ST_A_END;
        else                                              st <= ST_A_JOY0;
    end

    // IKBD_SendAutoJoysticks (the previous state is updated only when the
    // packet fits, so that a dropped report is sent again)
    ST_A_JOY0: begin
        st <= ST_A_JOY1;
        if (joy_mode == J_AUTO && cj0 != prev_j0) begin
            if (pkt2_fits) prev_j0 <= cj0;
            v0 <= 8'hFE;
            v1 <= cj0;
            snd(PT_V, 3'd2, 4'd0, R_JOY1);
        end
    end
    ST_A_JOY1: begin
        st <= joy_only ? ST_IDLE : ST_A_MOUSE;
        if (joy_mode == J_AUTO && cj1 != prev_j1) begin
            if (pkt2_fits) prev_j1 <= cj1;
            v0 <= 8'hFF;
            v1 <= cj1;
            snd(PT_V, 3'd2, 4'd0, joy_only ? R_IDLE : R_A_MOUSE);
        end
    end

    // IKBD_SendRelMousePacket / IKBD_SendCursorMousePacket
    // (ST_A_MOUSE addresses relative X and the X threshold)
    ST_A_MOUSE: begin
        st <= ST_A_END;
        ax <= 1'b0;
        if (f_cnt == 6'd0) begin
            if (mouse_mode == M_REL)      st <= ST_R_T1;
            else if (mouse_mode == M_CUR) st <= ST_C_PRE;
        end
    end
    ST_R_T1: begin                            // relative X and X threshold
        relc <= thr_hit || (cl != or_l) || (crb != or_r);
        v1   <= s8;
        st   <= ST_R_T2;
    end
    ST_R_T2: begin                            // relative Y and Y threshold
        st <= ST_A_END;
        if (relc || thr_hit) begin
            v0    <= {6'b111110, cl, crb};
            v2    <= ydown ? (~s8 + 8'd1) : s8;
            or_l  <= cl;
            or_r  <= crb;
            st    <= ST_R_SUBX;
        end
    end
    ST_R_SUBX: begin
        ph <= ~ph;
        if (ph) begin
            rf_we = 1'b1;
            rf_wa = RF_RX;
            rf_ws = WS_SAT;
            st    <= ST_R_SUBY;
        end
    end
    ST_R_SUBY: begin
        ph <= ~ph;
        if (ph) begin
            rf_we = 1'b1;
            rf_wa = RF_RX + 3'd1;
            rf_ws = WS_SAT;
            snd(PT_V, 3'd3, 4'd0, R_A_END);
        end
    end

    // cursor keys: per axis left/up then right/down, then the buttons;
    // pq = keycode delta of the axis
    ST_C_PRE: st <= ST_C_LD;
    ST_C_LD: begin
        pq <= p_q;
        st <= ST_C_L;
    end
    ST_C_L: begin
        ph <= ~ph;
        if (ph) begin
            if (rf_q == 18'd0) begin
                ax <= ~ax;
                st <= ax ? ST_C_BL : ST_C_PRE;
            end else if (alu_neg || alu_zero) begin   // acc + delta <= 0
                rf_we = 1'b1;
                rf_wa = RF_RX + {2'b00, ax};
                rf_ws = WS_SAT;
                v0    <= ax ? 8'd72 : 8'd75;
                v1    <= ax ? 8'd200 : 8'd203;        // | $80
                snd(PT_V, 3'd2, 4'd0, R_C_R);
            end else begin
                st <= ST_C_R;
            end
        end
    end
    ST_C_R: begin
        ph <= ~ph;
        if (ph) begin
            if (!alu_neg) begin                       // acc - delta >= 0
                rf_we = 1'b1;
                rf_wa = RF_RX + {2'b00, ax};
                rf_ws = WS_SAT;
                v0    <= ax ? 8'd80 : 8'd77;
                v1    <= ax ? 8'd208 : 8'd205;
                snd(PT_V, 3'd2, 4'd0, R_C_Z);
            end else begin
                st <= ST_C_Z;
            end
        end
    end
    ST_C_Z: begin
        if (pq == 8'd0) begin
            rf_we = 1'b1;
            rf_wa = RF_RX + {2'b00, ax};
            rf_ws = WS_ZERO;
        end
        ax <= ~ax;
        st <= ax ? ST_C_BL : ST_C_PRE;
    end
    ST_C_BL: begin
        or_l <= cl;
        st   <= ST_C_BR;
        if (cl != or_l) begin
            v0 <= cl ? 8'h74 : 8'hF4;
            snd(PT_V, 3'd1, 4'd0, R_C_BR);
        end
    end
    ST_C_BR: begin
        or_r <= crb;
        st   <= ST_A_END;
        if (crb != or_r) begin
            v0 <= crb ? 8'h75 : 8'hF5;
            snd(PT_V, 3'd1, 4'd0, R_A_END);
        end
    end

    // other modes: relative accumulators are not kept
    ST_A_END: begin
        st <= ST_SPC;
        if (mouse_mode != M_REL && mouse_mode != M_CUR) begin
            or_l  <= cl;
            or_r  <= crb;
            rf_we = 1'b1;
            rf_wa = RF_RX;
            st    <= ST_END1;
        end
    end
    ST_END1: begin
        rf_we = 1'b1;
        rf_wa = RF_RX + 3'd1;
        st    <= ST_SPC;
    end

    // joystick button 2 as Space, after the mouse/joystick packets
    // (not in the monitoring modes)
    ST_SPC: begin
        st <= ST_IDLE;
        if (joy_mode == J_MON || joy_mode == J_FIRE) begin
            // IKBD_SendAutoJoysticksMonitoring path returns before the key
        end else if (spc == SPC_DOWN) begin
            spc <= SPC_DOWNED;
            v0  <= 8'h39;
            snd(PT_V, 3'd1, 4'd0, R_IDLE);
        end else if (spc == SPC_UP) begin
            spc <= SPC_NULL;
            v0  <= 8'hB9;
            snd(PT_V, 3'd1, 4'd0, R_IDLE);
        end
    end

    default: st <= ST_IDLE;
    endcase

    // register file write port (cleared in ST_CLR)
    if (st == ST_CLR && clr_cnt[6:3] == 4'd0) begin
        rf_we = 1'b1;
        rf_wa = clr_cnt[2:0];
        rf_ws = WS_ZERO;
    end
    case (rf_ws)
    WS_SAT:  rf_wd = {{2{alu_s[15]}}, alu_s};
    WS_RAW:  rf_wd = alu_r;
    WS_MAX:  rf_wd = {2'b00, pq, p_q};
    default: rf_wd = 18'd0;
    endcase
    if (rf_we) rf[rf_wa] <= rf_wd;

    // ------------------------------------------------------------------
    // output buffer
    // ------------------------------------------------------------------
    if (push) begin
        fifo[f_wr] <= pbyte;
        f_wr <= f_wr + 5'd1;
    end
    f_cnt <= f_cnt + (push ? 6'd1 : 6'd0) - (pop ? 6'd1 : 6'd0);

    // ------------------------------------------------------------------
    // IKBD_Boot_ROM (reset command $80 $01, or hardware reset).  The
    // parameter defaults are written to the parameter RAM in ST_CLR.
    // ------------------------------------------------------------------
    if (do_boot || reset) begin
        mouse_mode <= M_REL;
        joy_mode   <= J_AUTO;
        absflags   <= 4'd0;
        ydown      <= 1'b0;
        act        <= 3'd0;
        prev_j0    <= 8'd0;
        prev_j1    <= 8'd0;
        f_wr       <= 5'd0;
        f_rd       <= 5'd0;
        f_cnt      <= 6'd0;
        ib_n       <= 3'd0;
        pause      <= 1'b0;
        mouse_dis  <= 1'b0;
        joy_dis    <= 1'b0;
        resetting  <= 1'b1;
        both       <= 1'b0;
        men_reset  <= 1'b0;
        ld_left    <= 8'd0;
        rst_cnt    <= RESET_X16;
        boot_pend  <= 1'b0;
        oa_l <= 1'b0; oa_r <= 1'b0; or_l <= 1'b0; or_r <= 1'b0;
        cl   <= 1'b0; crb  <= 1'b0;
        mon_pend   <= 1'b0;
        fire_pend  <= 1'b0;
        mon_rate   <= 8'd1;
        mon_div    <= 8'd0;
        clr_cnt    <= 7'd0;
        st         <= ST_CLR;      // clear key states, load parameter defaults
    end

    if (reset) begin
        // IKBD_Reset: the SCI is reset too
        r_st      <= 2'd0;
        r_cnt     <= 4'd0;
        r_bit     <= 3'd0;
        r_armed   <= 1'b0;
        rx_pend   <= 1'b0;
        t_sub     <= 4'd0;
        t_st      <= 2'd0;
        t_bit     <= 3'd0;
        tx_delay  <= 4'd0;
        txd       <= 1'b1;
        key_tog   <= ps2_key[10];
        key_pend  <= 1'b0;
        m_tog     <= ps2_mouse[24];
        ml        <= 1'b0;
        mr        <= 1'b0;
        auto_cnt  <= {AUTO_W{1'b0}};
        auto_pend <= 1'b0;
        mon_x     <= 11'd0;
        fire_cnt  <= 5'd0;
        fire_n    <= 3'd0;
        joy_only  <= 1'b0;
        pidx      <= 3'd0;
        ph        <= 1'b0;
    end
end

endmodule
