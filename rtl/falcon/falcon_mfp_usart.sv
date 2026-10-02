// falcon_mfp_usart.sv - MC68901 USART (asynchronous modes) for falcon_mfp.
//
// Registers (MFP register index): SCR 19, UCR 20, RSR 21, TSR 22, UDR 23.
//
// Behavioural reference: MC68901 data sheet (USART section); Hatari
// src/rs232.c only provides the register level view (RS232_TSR_ReadByte
// returns BE = 1 for an idle transmitter, RS232_RSR_ReadByte returns BF = 1
// only when a byte was received, RS232_UDR_ReadByte clears BF), which this
// implementation matches for an idle line:
//   after reset        TSR = $80 (BE), RSR = $00
//   after RSR=1 TSR=1  TSR = $81,      RSR = $01
//
// UCR: bit 7 CLK (1 = divide RC/TC by 16), bits 6:5 word length (00 = 8,
//      01 = 7, 10 = 6, 11 = 5), bits 4:3 start/stop (00 synchronous, 01 one
//      stop bit, 10 one and a half, 11 two), bit 2 parity enable, bit 1
//      even parity (1) / odd (0), bit 0 unused (reads 0).
// RSR: BF OE PE FE B CIP SS RE (B = break detect, CIP = character in
//      progress, asynchronous meaning).  Only SS and RE are writable.
// TSR: BE UE AT END B H L TE.  AT, B, H, L, TE are writable.
//
// Clocks: the receiver clock RC and transmitter clock TC are the timer D
// output TDO (Falcon wiring).  The receiver samples on rising RC edges, the
// transmitter changes SO on falling TC edges.  With UCR bit 7 = 1 a bit lasts
// 16 clocks and the receiver validates the start bit and samples every bit in
// its middle (8 clocks after the falling edge of the start bit).
//
// Interrupt events (one clock pulses, to the MFP channels):
//   ev_tx_err  (channel 9)  END set (transmitter disabled and idle)
//   ev_tx_empty(channel 10) BE set (UDR moved to the shift register)
//   ev_rx_err  (channel 11) character with PE/FE, overrun or break, when the
//                           receive error channel is enabled (rx_err_ena)
//   ev_rx_full (channel 12) character received; also for characters with
//                           errors when the error channel is disabled (data
//                           sheet: errors are then reported through the
//                           buffer full channel)
//
// Deviations / not implemented:
// - Synchronous mode (UCR bits 4:3 = 00): SCR is stored and read back but the
//   transmitter and receiver stay idle.  Neither TOS nor EmuTOS use it.
// - UE (underrun) only has a meaning in synchronous mode and stays 0.
// - Overrun: the new character is discarded, OE is set and an error event is
//   raised; OE is cleared by reading RSR.
// - Clearing RE aborts a character in progress and clears BF/OE/PE/FE/B/CIP.
// - At reset END is 0 (Hatari reads TSR = $80 after reset); END is set when
//   TE is written from 1 to 0 and the transmitter has finished.
//
// Copyright (C) 2026 Falcon_MiSTer project.  GPL v2 or later (as Hatari).

module falcon_mfp_usart (
    input            clk,
    input            reset,
    input            tc_rise,      // rising edge of TDO (RC/TC)
    input            tc_fall,      // falling edge of TDO
    // register writes (one clock each)
    input            scr_wr,
    input            ucr_wr,
    input            rsr_wr,
    input            tsr_wr,
    input            udr_wr,
    input      [7:0] din,
    // read side effects (one clock each)
    input            rsr_rd,
    input            tsr_rd,
    input            udr_rd,
    output reg [7:0] scr,
    output     [7:0] ucr_q,
    output     [7:0] rsr_q,
    output     [7:0] tsr_q,
    output reg [7:0] udr_rx,
    // serial pins
    input            si,
    output           so,
    // interrupt routing
    input            rx_err_ena,   // IERA bit 3 (receive error channel enabled)
    output reg       ev_tx_err,
    output reg       ev_tx_empty,
    output reg       ev_rx_err,
    output reg       ev_rx_full
);

    // ------------------------------------------------------------------
    // registers
    reg [7:1] ucr;
    assign ucr_q = {ucr, 1'b0};

    wire       div16   = ucr[7];
    wire [1:0] wl      = ucr[6:5];
    wire [1:0] st      = ucr[4:3];
    wire       par_en  = ucr[2];
    wire       par_ev  = ucr[1];
    wire       async   = (st != 2'b00);
    wire [3:0] nbits   = 4'd8 - {2'b00, wl};

    reg rx_bf, rx_oe, rx_pe, rx_fe, rx_b, rx_cip, rx_ss, rx_re;
    assign rsr_q = {rx_bf, rx_oe, rx_pe, rx_fe, rx_b, rx_cip, rx_ss, rx_re};

    reg tx_be, tx_at, tx_end, tx_brk, tx_h, tx_l, tx_te;
    assign tsr_q = {tx_be, 1'b0, tx_at, tx_end, tx_brk, tx_h, tx_l, tx_te};

    wire loopback = tx_h & tx_l;

    // ------------------------------------------------------------------
    // transmitter
    localparam TX_IDLE = 3'd0, TX_START = 3'd1, TX_DATA = 3'd2,
               TX_PAR  = 3'd3, TX_STOP  = 3'd4, TX_BREAK = 3'd5;

    reg  [2:0] tx_state;
    reg  [7:0] tx_buf;
    reg  [7:0] tx_sh;
    reg  [4:0] tx_cnt;     // clocks within the current bit
    reg  [3:0] tx_bit;     // data bits sent
    reg        tx_par;
    reg        tx_line;
    reg        end_armed;  // TE was cleared: set END once idle

    wire [4:0] bit_last  = div16 ? 5'd15 : 5'd0;
    wire [4:0] stop_last = !div16 ? ((st == 2'b01) ? 5'd0 : 5'd1)
                                  : ((st == 2'b01) ? 5'd15 : (st == 2'b10) ? 5'd23 : 5'd31);

    wire tx_busy   = (tx_state != TX_IDLE) && (tx_state != TX_BREAK);
    wire tx_start_ok = tx_te && !tx_be && !tx_brk && async;

    function [7:0] mask_wl;
        input [7:0] v;
        input [1:0] w;
        case (w)
            2'b00: mask_wl = v;
            2'b01: mask_wl = {1'b0, v[6:0]};
            2'b10: mask_wl = {2'b00, v[5:0]};
            default: mask_wl = {3'b000, v[4:0]};
        endcase
    endfunction

    // so: transmitter line while enabled or busy, else per H/L
    reg so_r;
    always @(*) begin
        if (tx_te || tx_busy)
            so_r = tx_line;
        else
            case ({tx_h, tx_l})
                2'b01:   so_r = 1'b0;   // low
                default: so_r = 1'b1;   // high impedance (pulled high), high, loopback
            endcase
    end
    assign so = so_r;

    // ------------------------------------------------------------------
    // receiver
    localparam RX_IDLE = 2'd0, RX_START = 2'd1, RX_BITS = 2'd2, RX_STOP = 2'd3;

    reg  [1:0] rx_state;
    reg  [4:0] rx_cnt;
    reg  [3:0] rx_bit;     // data + parity bits received
    reg  [7:0] rx_sh;
    reg        rx_parbit;
    reg        si_s1, si_s2;

    wire rx_line = loopback ? ((tx_te || tx_busy) ? tx_line : 1'b1) : si_s2;
    wire [4:0] rx_half = div16 ? 5'd7 : 5'd0;
    wire [3:0] rx_nbits = nbits + {3'b000, par_en};

    // received data, right aligned for the word length
    function [7:0] align_rx;
        input [7:0] v;
        input [1:0] w;
        case (w)
            2'b00: align_rx = v;
            2'b01: align_rx = {1'b0, v[7:1]};
            2'b10: align_rx = {2'b00, v[7:2]};
            default: align_rx = {3'b000, v[7:3]};
        endcase
    endfunction

    wire [7:0] rx_data_al = align_rx(rx_sh, wl);
    wire       rx_par_calc = ^rx_data_al ^ ~par_ev;   // expected parity bit
    // even parity: bit = xor(data); odd parity: bit = ~xor(data)

    always @(posedge clk) begin
        ev_tx_err   <= 1'b0;
        ev_tx_empty <= 1'b0;
        ev_rx_err   <= 1'b0;
        ev_rx_full  <= 1'b0;
        si_s1 <= si;
        si_s2 <= si_s1;

        if (reset) begin
            scr       <= 8'd0;
            ucr       <= 7'd0;
            rx_bf <= 1'b0; rx_oe <= 1'b0; rx_pe <= 1'b0; rx_fe <= 1'b0;
            rx_b  <= 1'b0; rx_cip <= 1'b0; rx_ss <= 1'b0; rx_re <= 1'b0;
            tx_be <= 1'b1; tx_at <= 1'b0; tx_end <= 1'b0; tx_brk <= 1'b0;
            tx_h  <= 1'b0; tx_l <= 1'b0; tx_te <= 1'b0;
            tx_state  <= TX_IDLE;
            tx_line   <= 1'b1;
            tx_cnt    <= 5'd0;
            tx_bit    <= 4'd0;
            tx_sh     <= 8'd0;
            tx_buf    <= 8'd0;
            tx_par    <= 1'b0;
            end_armed <= 1'b0;
            rx_state  <= RX_IDLE;
            rx_cnt    <= 5'd0;
            rx_bit    <= 4'd0;
            rx_sh     <= 8'd0;
            rx_parbit <= 1'b0;
            udr_rx    <= 8'd0;
            si_s1     <= 1'b1;
            si_s2     <= 1'b1;
        end else begin
            // ---------------- register writes
            if (scr_wr) scr <= din;
            if (ucr_wr) ucr <= din[7:1];
            if (rsr_wr) begin
                rx_ss <= din[1];
                rx_re <= din[0];
            end
            if (tsr_wr) begin
                tx_at  <= din[5];
                tx_brk <= din[3];
                tx_h   <= din[2];
                tx_l   <= din[1];
                tx_te  <= din[0];
                if (din[0])
                    tx_end <= 1'b0;
                if (tx_te && !din[0])
                    end_armed <= 1'b1;
                if (din[0])
                    end_armed <= 1'b0;
            end
            if (udr_wr) begin
                tx_buf <= din;
                tx_be  <= 1'b0;
            end

            // ---------------- transmitter (changes on falling TC edges)
            if (tc_fall) begin
                case (tx_state)
                    TX_IDLE, TX_BREAK: begin
                        if (tx_te && tx_brk && async) begin
                            tx_state <= TX_BREAK;
                            tx_line  <= 1'b0;
                        end else if (tx_start_ok) begin
                            tx_sh       <= mask_wl(tx_buf, wl);
                            tx_par      <= ^mask_wl(tx_buf, wl) ^ ~par_ev;
                            tx_be       <= 1'b1;
                            ev_tx_empty <= 1'b1;
                            tx_state    <= TX_START;
                            tx_line     <= 1'b0;
                            tx_cnt      <= 5'd0;
                        end else begin
                            tx_state <= TX_IDLE;
                            tx_line  <= 1'b1;
                        end
                    end
                    TX_START: begin
                        if (tx_cnt == bit_last) begin
                            tx_cnt   <= 5'd0;
                            tx_bit   <= 4'd1;
                            tx_line  <= tx_sh[0];
                            tx_sh    <= {1'b0, tx_sh[7:1]};
                            tx_state <= TX_DATA;
                        end else
                            tx_cnt <= tx_cnt + 5'd1;
                    end
                    TX_DATA: begin
                        if (tx_cnt == bit_last) begin
                            tx_cnt <= 5'd0;
                            if (tx_bit == nbits) begin
                                if (par_en) begin
                                    tx_line  <= tx_par;
                                    tx_state <= TX_PAR;
                                end else begin
                                    tx_line  <= 1'b1;
                                    tx_state <= TX_STOP;
                                end
                            end else begin
                                tx_bit  <= tx_bit + 4'd1;
                                tx_line <= tx_sh[0];
                                tx_sh   <= {1'b0, tx_sh[7:1]};
                            end
                        end else
                            tx_cnt <= tx_cnt + 5'd1;
                    end
                    TX_PAR: begin
                        if (tx_cnt == bit_last) begin
                            tx_cnt   <= 5'd0;
                            tx_line  <= 1'b1;
                            tx_state <= TX_STOP;
                        end else
                            tx_cnt <= tx_cnt + 5'd1;
                    end
                    TX_STOP: begin
                        if (tx_cnt == stop_last) begin
                            tx_cnt <= 5'd0;
                            // back to back characters: next start bit now
                            if (tx_start_ok) begin
                                tx_sh       <= mask_wl(tx_buf, wl);
                                tx_par      <= ^mask_wl(tx_buf, wl) ^ ~par_ev;
                                tx_be       <= 1'b1;
                                ev_tx_empty <= 1'b1;
                                tx_state    <= TX_START;
                                tx_line     <= 1'b0;
                            end else if (tx_te && tx_brk && async) begin
                                tx_state <= TX_BREAK;
                                tx_line  <= 1'b0;
                            end else begin
                                tx_state <= TX_IDLE;
                                tx_line  <= 1'b1;
                            end
                        end else
                            tx_cnt <= tx_cnt + 5'd1;
                    end
                    default: tx_state <= TX_IDLE;
                endcase
            end
            // break ends when B or TE is cleared
            if (tx_state == TX_BREAK && (!tx_brk || !tx_te)) begin
                tx_state <= TX_IDLE;
                tx_line  <= 1'b1;
            end

            // END: transmitter disabled and idle
            if (end_armed && !tx_te && !tx_busy && !(tsr_wr && din[0])) begin
                end_armed <= 1'b0;
                tx_end    <= 1'b1;
                ev_tx_err <= 1'b1;
                if (tx_at)
                    rx_re <= 1'b1;   // auto turnaround
            end

            // ---------------- receiver (samples on rising RC edges)
            if (tc_rise && rx_re && async) begin
                case (rx_state)
                    RX_IDLE: begin
                        if (!rx_line) begin
                            if (div16) begin
                                rx_state <= RX_START;
                                rx_cnt   <= 5'd1;
                            end else begin
                                rx_state <= RX_BITS;
                                rx_cnt   <= 5'd0;
                                rx_bit   <= 4'd0;
                                rx_cip   <= 1'b1;
                            end
                        end
                    end
                    RX_START: begin
                        if (rx_cnt == rx_half) begin
                            if (!rx_line) begin
                                rx_state <= RX_BITS;
                                rx_cnt   <= 5'd0;
                                rx_bit   <= 4'd0;
                                rx_cip   <= 1'b1;
                            end else
                                rx_state <= RX_IDLE;   // false start bit
                        end else
                            rx_cnt <= rx_cnt + 5'd1;
                    end
                    RX_BITS: begin
                        if (rx_cnt == bit_last) begin
                            rx_cnt <= 5'd0;
                            if (rx_bit < nbits)
                                rx_sh <= {rx_line, rx_sh[7:1]};
                            else
                                rx_parbit <= rx_line;
                            if (rx_bit + 4'd1 == rx_nbits)
                                rx_state <= RX_STOP;
                            rx_bit <= rx_bit + 4'd1;
                        end else
                            rx_cnt <= rx_cnt + 5'd1;
                    end
                    RX_STOP: begin
                        if (rx_cnt == bit_last) begin
                            rx_cnt   <= 5'd0;
                            rx_state <= RX_IDLE;
                            rx_cip   <= 1'b0;
                            if (!rx_line && rx_data_al == 8'd0 &&
                                (!par_en || !rx_parbit)) begin
                                // break: all zero character without stop bit
                                rx_b <= 1'b1;
                                if (rx_err_ena) ev_rx_err  <= 1'b1;
                                else            ev_rx_full <= 1'b1;
                            end else if (rx_bf) begin
                                rx_oe <= 1'b1;
                                if (rx_err_ena) ev_rx_err  <= 1'b1;
                                else            ev_rx_full <= 1'b1;
                            end else begin
                                udr_rx <= rx_data_al;
                                rx_bf  <= 1'b1;
                                rx_pe  <= par_en && (rx_parbit != rx_par_calc);
                                rx_fe  <= !rx_line;
                                if (((par_en && (rx_parbit != rx_par_calc)) || !rx_line) && rx_err_ena)
                                    ev_rx_err  <= 1'b1;
                                else
                                    ev_rx_full <= 1'b1;
                            end
                        end else
                            rx_cnt <= rx_cnt + 5'd1;
                    end
                endcase
            end

            // ---------------- read side effects
            if (udr_rd)
                rx_bf <= 1'b0;
            if (rsr_rd) begin
                rx_oe <= 1'b0;
                if (rx_line)
                    rx_b <= 1'b0;
            end

            // receiver disabled: abort and clear the flags
            if (!rx_re) begin
                rx_state <= RX_IDLE;
                rx_cip   <= 1'b0;
                rx_bf    <= 1'b0;
                rx_oe    <= 1'b0;
                rx_pe    <= 1'b0;
                rx_fe    <= 1'b0;
                rx_b     <= 1'b0;
            end
        end
    end

endmodule
