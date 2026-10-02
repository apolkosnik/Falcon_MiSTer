// falcon_acia.sv - the two MC6850 ACIAs of the Atari Falcon030 ($FFFC00-$FFFC07)
// plus the IKBD (HD6301 keyboard processor, protocol level model, falcon_ikbd).
//
// Registers (even bytes, the odd bytes read $FF):
//   $FFFC00  IKBD ACIA control (write) / status (read)
//   $FFFC02  IKBD ACIA data
//   $FFFC04  MIDI ACIA control (write) / status (read)
//   $FFFC06  MIDI ACIA data
// irq: active high when either ACIA asserts its IRQ (the system inverts it
// onto MFP GPIP4).
//
// Both ACIAs get a 500 kHz RX/TX clock (fractional accumulator from CLK_HZ).
// The IKBD ACIA is wired to the IKBD model through a real bit level serial
// link (ACIA TXD -> IKBD RXD, IKBD TXD -> ACIA RXD).  The MIDI ACIA is wired
// to midi_tx (idle high) / midi_rx (synchronised here).
//
// Behaviour follows Hatari src/acia.c (ACIA_Write_CR, ACIA_MasterReset,
// ACIA_Read_SR, ACIA_Read_RDR, ACIA_Write_TDR, ACIA_UpdateIRQ, ACIA_Clock_TX,
// ACIA_Clock_RX) with these differences, where the MC6850 datasheet wins:
//  - The receiver in /16 and /64 modes detects the start bit edge, checks
//    it again at mid bit and then samples every bit at mid bit (Hatari
//    samples once per bit period on its own timer).  /1 mode samples on
//    every RX clock as Hatari does.
//  - 7 bit words are received into RDR bits 6:0 with bit 7 = 0 (Hatari's
//    RSR shifting leaves 7 bit data shifted left by one).
//  - Only the first stop bit is checked by the receiver (datasheet: framing
//    error = absence of the first stop bit); Hatari waits for all stop bits.
//  - A character with a framing error is transferred to RDR with RDRF set
//    and FE set (datasheet); Hatari copies it to RDR without setting RDRF.
//    After a framing error the receiver waits for the line to go high
//    before looking for the next start bit.
//  - While CR1:CR0 = 11 (master reset) the transmitter and receiver are
//    held idle until a control word with another divide ratio is written
//    (power-up state is "held in reset", SR = $00, as in Hatari's
//    ACIA_Init before the first master reset).
//  - The MC6850 has no reset pin: the system 'reset' input does not change
//    the ACIA registers (power-up values come from register initialisers),
//    exactly as Hatari's ACIA_Reset() leaves the ACIA state alone.  The
//    IKBD, whose reset pin is wired to the system reset, is reset.
//  - Bus timing: an access is acknowledged after a fixed 6 cycle delay
//    (at 8 MHz) plus synchronisation to the end of the next E clock cycle
//    (E = 800 kHz), modelled on Hatari's ACIA_AddWaitCycles; ACCESS_WAIT=0
//    acknowledges at once.
//  - CTS and DCD are tied low (Falcon), RTS is not connected.

module falcon_acia #(
    parameter CLK_HZ      = 32000000,
    parameter ACCESS_WAIT = 1,
    parameter IKBD_AUTOSEND_HZ = 1000
)(
    input             clk,
    input             reset,

    input             bus_cs,
    input             bus_stb,
    input             bus_we,
    input      [2:1]  bus_addr,
    input             bus_uds,
    input             bus_lds,
    input      [15:0] bus_din,
    output reg [15:0] bus_dout,
    output reg        bus_ack,

    output            irq,

    input      [10:0] ps2_key,
    input      [24:0] ps2_mouse,
    input      [31:0] joystick_0,
    input      [31:0] joystick_1,

    output            midi_tx,
    input             midi_rx
);

// ---------------------------------------------------------------------------
// 500 kHz ACIA clock enable
// ---------------------------------------------------------------------------
localparam [31:0] ACIA_CLK_HZ = 32'd500000;
reg  [31:0] cacc = 32'd0;
reg         clk500 = 1'b0;
always @(posedge clk) begin
    if (cacc + ACIA_CLK_HZ >= CLK_HZ) begin
        cacc   <= cacc + ACIA_CLK_HZ - CLK_HZ;
        clk500 <= 1'b1;
    end else begin
        cacc   <= cacc + ACIA_CLK_HZ;
        clk500 <= 1'b0;
    end
end

// ---------------------------------------------------------------------------
// Bus access with E clock synchronisation
// ---------------------------------------------------------------------------
localparam integer E_DIV    = (CLK_HZ / 800000) < 2 ? 2 : (CLK_HZ / 800000);
localparam integer MIN_WAIT = (CLK_HZ / 8000000) * 6;
reg  [7:0] e_cnt = 8'd0;
always @(posedge clk)
    e_cnt <= (e_cnt == E_DIV - 1) ? 8'd0 : e_cnt + 8'd1;

reg        busy = 1'b0;
reg  [7:0] wcnt = 8'd0;
wire       go = busy && (ACCESS_WAIT == 0 || (wcnt >= MIN_WAIT && e_cnt == E_DIV - 1));
reg        acc_we;
reg  [2:1] acc_addr;
reg        acc_uds;
reg  [7:0] acc_data;

wire [7:0] sr_ikbd, rdr_ikbd, sr_midi, rdr_midi;
wire       irq_ikbd, irq_midi;

// one clock strobes to the ACIA cores (on the clock that sets bus_ack)
wire stb_ok   = go && acc_uds;
wire ik_rd_sr = stb_ok && !acc_we && acc_addr == 2'd0;
wire ik_rd_dr = stb_ok && !acc_we && acc_addr == 2'd1;
wire ik_wr_cr = stb_ok &&  acc_we && acc_addr == 2'd0;
wire ik_wr_dr = stb_ok &&  acc_we && acc_addr == 2'd1;
wire mi_rd_sr = stb_ok && !acc_we && acc_addr == 2'd2;
wire mi_rd_dr = stb_ok && !acc_we && acc_addr == 2'd3;
wire mi_wr_cr = stb_ok &&  acc_we && acc_addr == 2'd2;
wire mi_wr_dr = stb_ok &&  acc_we && acc_addr == 2'd3;

reg [7:0] rd_byte;
always @(*) begin
    case (acc_addr)
        2'd0: rd_byte = sr_ikbd;
        2'd1: rd_byte = rdr_ikbd;
        2'd2: rd_byte = sr_midi;
        default: rd_byte = rdr_midi;
    endcase
end

always @(posedge clk) begin
    bus_ack <= 1'b0;
    if (bus_stb && !busy) begin
        busy     <= 1'b1;
        wcnt     <= 8'd0;
        acc_we   <= bus_we;
        acc_addr <= bus_addr;
        acc_uds  <= bus_uds;
        acc_data <= bus_din[15:8];
    end else if (busy) begin
        if (wcnt != 8'hFF) wcnt <= wcnt + 8'd1;
        if (go) begin
            busy     <= 1'b0;
            bus_ack  <= 1'b1;
            bus_dout <= {(acc_uds && !acc_we) ? rd_byte : 8'hFF, 8'hFF};
        end
    end
end

// ---------------------------------------------------------------------------
// The two ACIAs
// ---------------------------------------------------------------------------
wire ikbd_acia_txd, ikbd_txd;

falcon_acia_6850 u_acia_ikbd (
    .clk     (clk),
    .clk_en  (clk500),
    .rd_sr   (ik_rd_sr),
    .rd_rdr  (ik_rd_dr),
    .wr_cr   (ik_wr_cr),
    .wr_tdr  (ik_wr_dr),
    .din     (acc_data),
    .sr      (sr_ikbd),
    .rdr     (rdr_ikbd),
    .irq     (irq_ikbd),
    .txd     (ikbd_acia_txd),
    .rxd     (ikbd_txd)
);

reg [1:0] midi_rx_s = 2'b11;
always @(posedge clk) midi_rx_s <= {midi_rx_s[0], midi_rx};

falcon_acia_6850 u_acia_midi (
    .clk     (clk),
    .clk_en  (clk500),
    .rd_sr   (mi_rd_sr),
    .rd_rdr  (mi_rd_dr),
    .wr_cr   (mi_wr_cr),
    .wr_tdr  (mi_wr_dr),
    .din     (acc_data),
    .sr      (sr_midi),
    .rdr     (rdr_midi),
    .irq     (irq_midi),
    .txd     (midi_tx),
    .rxd     (midi_rx_s[1])
);

assign irq = irq_ikbd | irq_midi;

// ---------------------------------------------------------------------------
// IKBD
// ---------------------------------------------------------------------------
falcon_ikbd #(
    .CLK_HZ      (CLK_HZ),
    .AUTOSEND_HZ (IKBD_AUTOSEND_HZ)
) u_ikbd (
    .clk        (clk),
    .reset      (reset),
    .rxd        (ikbd_acia_txd),
    .txd        (ikbd_txd),
    .ps2_key    (ps2_key),
    .ps2_mouse  (ps2_mouse),
    .joystick_0 (joystick_0),
    .joystick_1 (joystick_1)
);

endmodule

// ===========================================================================
// MC6850 core.  Register strobes are one clock wide; sr/rdr are the values a
// read returns on that clock.
// ===========================================================================
module falcon_acia_6850 (
    input            clk,
    input            clk_en,     // 500 kHz RX/TX clock enable
    input            rd_sr,
    input            rd_rdr,
    input            wr_cr,
    input            wr_tdr,
    input      [7:0] din,
    output     [7:0] sr,
    output     [7:0] rdr,
    output           irq,
    output reg       txd,
    input            rxd
);

// Word select (CR4:CR2): data bits, parity (0 none, 1 even, 2 odd), stop bits
function [3:0] ws_bits(input [2:0] ws);
    ws_bits = ws[2] ? 4'd8 : 4'd7;
endfunction
function [1:0] ws_par(input [2:0] ws);
    case (ws)
        3'd0, 3'd2, 3'd6: ws_par = 2'd1;
        3'd1, 3'd3, 3'd7: ws_par = 2'd2;
        default:          ws_par = 2'd0;
    endcase
endfunction
function ws_two_stop(input [2:0] ws);
    ws_two_stop = (ws == 3'd0) || (ws == 3'd1) || (ws == 3'd4);
endfunction

reg  [7:0] cr       = 8'h03;     // power-up: held in reset
reg  [7:0] tdr      = 8'h00;
reg  [7:0] rdr_r    = 8'h00;
reg        rdrf     = 1'b0;
reg        tdre     = 1'b0;
reg        fe       = 1'b0;
reg        ovrn     = 1'b0;
reg        pe       = 1'b0;
reg        ovr_pend = 1'b0;      // Hatari RX_Overrun
reg        sr_read  = 1'b0;      // Hatari SR_Read

wire       in_reset = (cr[1:0] == 2'b11);
wire       tx_ie    = (cr[6:5] == 2'b01);
wire       tx_brk   = (cr[6:5] == 2'b11);
wire [2:0] ws       = cr[4:2];

assign irq = (cr[7] & (rdrf | ovr_pend)) | (tx_ie & tdre);
assign sr  = {irq, pe, ovrn, fe, 1'b0 /*CTS*/, 1'b0 /*DCD*/, tdre, rdrf};
assign rdr = rdr_r;

// divided bit clock for the transmitter (free running, restarted on a
// divide ratio change, as Hatari restarts its timer)
reg  [5:0] tx_div = 6'd0;
wire       tx_tick = clk_en && !in_reset &&
                     (cr[1:0] == 2'b00 ||
                      (cr[1:0] == 2'b01 && tx_div[3:0] == 4'd15) ||
                      (cr[1:0] == 2'b10 && tx_div == 6'd63));

// transmitter
localparam S_IDLE = 2'd0, S_DATA = 2'd1, S_PAR = 2'd2, S_STOP = 2'd3;
reg  [1:0] tx_st   = S_IDLE;
reg  [7:0] tsr     = 8'd0;
reg  [3:0] tx_n    = 4'd0;
reg        tx_p    = 1'b0;
reg        tx_stop2 = 1'b0;

// receiver
reg  [1:0] rx_st   = S_IDLE;
reg        rx_start = 1'b0;      // waiting for mid start bit
reg        rx_armed = 1'b0;
reg  [5:0] rx_div  = 6'd0;
reg  [7:0] rsr     = 8'd0;
reg  [3:0] rx_n    = 4'd0;
reg        rx_p    = 1'b0;
reg        rx_perr = 1'b0;

initial txd = 1'b1;

wire [5:0] rx_full = (cr[1:0] == 2'b10) ? 6'd63 : 6'd15;
wire [5:0] rx_half = (cr[1:0] == 2'b10) ? 6'd31 : 6'd7;
wire       div1    = (cr[1:0] == 2'b00);
wire       rx_samp = clk_en && !in_reset && (div1 || (rx_div == rx_full));

reg        rdrf_n, tdre_n, ovr_n, fe_n, pe_n, ovrn_n;

always @(posedge clk) begin
    // values after this clock's register accesses, used by the serial logic
    rdrf_n = rdrf;
    tdre_n = tdre;
    ovr_n  = ovr_pend;
    fe_n   = fe;
    pe_n   = pe;
    ovrn_n = ovrn;

    // ---------------- transmitter ----------------
    if (clk_en && !in_reset)
        tx_div <= tx_div + 6'd1;

    if (tx_tick) begin
        case (tx_st)
        S_IDLE: begin
            if (tx_brk) begin
                txd <= 1'b0;
            end else if (!tdre) begin
                tsr      <= tdr;
                tdre_n   = 1'b1;
                tx_n     <= ws_bits(ws);
                tx_p     <= 1'b0;
                tx_stop2 <= ws_two_stop(ws);
                txd      <= 1'b0;                 // start bit
                tx_st    <= S_DATA;
            end else begin
                txd <= 1'b1;
            end
        end
        S_DATA: begin
            txd  <= tsr[0];
            tx_p <= tx_p ^ tsr[0];
            tsr  <= {1'b0, tsr[7:1]};
            tx_n <= tx_n - 4'd1;
            if (tx_n == 4'd1)
                tx_st <= (ws_par(ws) != 2'd0) ? S_PAR : S_STOP;
        end
        S_PAR: begin
            txd   <= (ws_par(ws) == 2'd1) ? tx_p : ~tx_p;
            tx_st <= S_STOP;
        end
        default: begin
            txd <= 1'b1;
            if (tx_stop2) tx_stop2 <= 1'b0;
            else          tx_st <= S_IDLE;
        end
        endcase
    end

    // ---------------- receiver ----------------
    if (clk_en && !in_reset) begin
        if (rx_st == S_IDLE && !rx_start)
            rx_div <= 6'd0;
        else if (rx_start && rx_div == rx_half)
            rx_div <= 6'd0;
        else if (rx_div == rx_full)
            rx_div <= 6'd0;
        else
            rx_div <= rx_div + 6'd1;
    end

    if (clk_en && !in_reset) begin
        if (rx_st == S_IDLE && !rx_start) begin
            if (rxd) rx_armed <= 1'b1;
            if (rx_armed && !rxd) begin
                rsr     <= 8'd0;
                rx_n    <= ws_bits(ws);
                rx_p    <= 1'b0;
                rx_perr <= 1'b0;
                if (div1) rx_st <= S_DATA;        // Hatari: start bit seen
                else      rx_start <= 1'b1;
            end
        end else if (rx_start) begin
            if (rx_div == rx_half) begin
                rx_start <= 1'b0;
                if (!rxd) rx_st <= S_DATA;        // valid start bit
            end
        end
    end

    if (rx_samp && !rx_start) begin
        case (rx_st)
        S_DATA: begin
            if (ws_bits(ws) == 4'd8) rsr <= {rxd, rsr[7:1]};
            else                     rsr <= {1'b0, rxd, rsr[6:1]};
            rx_p <= rx_p ^ rxd;
            rx_n <= rx_n - 4'd1;
            if (rx_n == 4'd1)
                rx_st <= (ws_par(ws) != 2'd0) ? S_PAR : S_STOP;
        end
        S_PAR: begin
            rx_perr <= (ws_par(ws) == 2'd1) ? (rx_p != rxd) : (rx_p == rxd);
            rx_st   <= S_STOP;
        end
        S_STOP: begin
            // a read of RDR on this same clock frees the register first
            if (rd_rdr) rdrf_n = 1'b0;
            if (!rdrf_n) begin
                rdr_r  <= rsr;
                rdrf_n = 1'b1;
                fe_n   = !rxd;
                pe_n   = rx_perr;
            end else begin
                ovr_n  = 1'b1;
            end
            if (!rxd) rx_armed <= 1'b0;
            rx_st <= S_IDLE;
        end
        default: ;
        endcase
    end

    // ---------------- register accesses ----------------
    if (rd_sr)
        sr_read <= 1'b1;

    if (rd_rdr) begin
        if (!(rx_samp && !rx_start && rx_st == S_STOP))
            rdrf_n = 1'b0;
        pe_n = (rx_samp && !rx_start && rx_st == S_STOP) ? pe_n : 1'b0;
        if (sr_read) begin
            sr_read <= 1'b0;
            ovrn_n  = 1'b0;
        end
        if (ovr_n) begin
            ovrn_n = 1'b1;
            ovr_n  = 1'b0;
        end
    end

    if (wr_tdr) begin
        tdr    <= din;
        tdre_n = 1'b0;
    end

    if (wr_cr) begin
        cr <= din;
        if (din[1:0] == 2'b11) begin
            // master reset (ACIA_MasterReset)
            tdre_n   = 1'b1;
            rdrf_n   = 1'b0;
            fe_n     = 1'b0;
            ovrn_n   = 1'b0;
            pe_n     = 1'b0;
            ovr_n    = 1'b0;
            tx_st    <= S_IDLE;
            tsr      <= 8'd0;
            rx_st    <= S_IDLE;
            rx_start <= 1'b0;
            rx_armed <= 1'b0;
            rsr      <= 8'd0;
            tx_div   <= 6'd0;
            rx_div   <= 6'd0;
            txd      <= 1'b1;
        end else if (din[1:0] != cr[1:0]) begin
            tx_div <= 6'd0;
        end
    end

    rdrf     <= rdrf_n;
    tdre     <= tdre_n;
    ovr_pend <= ovr_n;
    fe       <= fe_n;
    pe       <= pe_n;
    ovrn     <= ovrn_n;
end

endmodule
