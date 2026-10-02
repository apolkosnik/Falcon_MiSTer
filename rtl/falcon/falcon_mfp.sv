// falcon_mfp.sv - MC68901 MFP of the Atari Falcon030 ($FFFA01-$FFFA2F).
//
// Registers on the odd bytes, bus_addr[5:1] = register index:
//   0 GPIP  1 AER   2 DDR   3 IERA  4 IERB  5 IPRA  6 IPRB  7 ISRA
//   8 ISRB  9 IMRA 10 IMRB 11 VR   12 TACR 13 TBCR 14 TCDCR 15 TADR
//  16 TBDR 17 TCDR 18 TDDR 19 SCR  20 UCR  21 RSR  22 TSR  23 UDR
// The even byte reads $FF (Hatari IoMem_VoidRead), indexes 24..31 read $FF.
// bus_ack comes one clock after bus_stb with the data latched at bus_stb;
// register side effects happen once, on the bus_stb clock.
//
// Behavioural reference (Hatari src/mfp.c, src/rs232.c):
//   MFP_Reset                  reset values (all registers 0)
//   MFP_GPIP_ReadByte_Main     GPIP read: DDR=1 bits return the output latch,
//                              DDR=0 bits the external line
//   MFP_GPIP_Update_Interrupt  edge detection on (line XOR AER); an AER write
//                              that makes a line "match" its AER bit raises
//                              the interrupt (the 'M'/'Realtime' quirk);
//                              only lines with DDR=0 raise interrupts; a DDR
//                              or GPIP write never raises one by itself
//   MFP_InputOnChannel         pending bit set only when the channel is
//                              enabled in IER
//   MFP_EnableX_WriteByte      IPR &= IER on an IER write
//   MFP_PendingX/InServiceX_WriteByte  writing 0 bits clears, 1 bits keep
//   MFP_VectorReg_WriteByte    S bit 1 -> 0 clears both ISR registers
//   MFP_CheckPendingInterrupts / MFP_InterruptRequest  priority 15 (GPIP7)
//                              .. 0 (GPIP0); a pending unmasked channel
//                              requests only if no ISR bit of the same or
//                              higher priority is set
//   MFP_ProcessIACK            vector = VR[7:4] | channel; clears the IPR bit,
//                              sets the ISR bit when S=1 (clears it when S=0)
//   MFP_TimerxCtrl/Data_*      see falcon_mfp_timer.sv
//   MFP_TimerA_Set_Line_Input  TAI counts with AER bit 4
//   MFP_TimerB_EventCount      TBI (Videl DE) counts with AER bit 3 (AER3=0:
//                              end of line, DE 1->0; AER3=1: start of line)
//   RS232_TSR/RSR/UDR_ReadByte idle status: TSR bit 7 = 1, RSR bit 7 = 0
//
// Interface beyond the common register bus:
//   irq            active high request to IPL6 (registered, follows the
//                  IPR/ISR/IMR registers on the same clock)
//   iack           one clock pulse: the CPU acknowledges level 6 and the
//                  system chose the MFP (it should do so when irq is 1)
//   iack_ack       one clock, the clock after iack, with iack_vector valid;
//                  iack_spurious = 1 if no channel was requesting any more
//                  (iack_vector is then $18, the spurious interrupt vector)
//   gpip_in        external line levels (Falcon wiring, see ARCHITECTURE.md)
//   gpip_out/oe    output latch / DDR
//   tai, tbi       timer A/B inputs (DMA sound SOUNDINT, Videl DE)
//   tao..tdo       timer outputs (toggle at each timeout)
//   si, so         USART serial input (idle high) / output
//
// Deviations from Hatari (hardware behaviour chosen, documented here):
// - Hatari's 4 CPU cycle IRQ-to-CPU delay and its "oldest event first"
//   ordering inside one CPU instruction are emulation artefacts and are not
//   modelled; the request is evaluated by priority at the acknowledge.
// - GPIP output latch keeps all 8 written bits (Hatari keeps only DDR=1
//   bits); reads are identical while DDR does not change.
// - The GPIP edge detector always follows the external line, so switching a
//   line from output to input never raises an interrupt (as in Hatari) and
//   later edges of the line are detected normally.
// - VR bits 2..0, TCDCR bits 7 and 3, TACR/TBCR bits 7..4 and UCR bit 0 read
//   as 0 (data sheet).  Hatari returns some of them as written.
// - Timer A/B pulse width mode is real (Hatari runs it as delay mode); in
//   that mode the GPIP4/GPIP3 interrupt channel is triggered by the end of
//   the TAI/TBI pulse (input leaving the AER level) instead of the pin.
// - TACR/TBCR bit 4 (output reset) forces TAO/TBO low (Hatari ignores it).
// - Event count mode counts transitions of TAI/TBI only; an AER write does
//   not produce a count (Hatari).
// - The USART is a real asynchronous transmitter/receiver (Hatari only
//   models the register view); see falcon_mfp_usart.sv.
//
// Copyright (C) 2026 Falcon_MiSTer project.  GPL v2 or later (as Hatari).

module falcon_mfp #(
    parameter integer CLK_HZ = 32000000,
    parameter integer MFP_HZ = 2457600
) (
    input             clk,
    input             reset,
    // common register bus
    input             bus_cs,
    input             bus_stb,
    input             bus_we,
    input       [5:1] bus_addr,
    input             bus_uds,
    input             bus_lds,
    input      [15:0] bus_din,
    output reg [15:0] bus_dout,
    output reg        bus_ack,
    // interrupt
    output reg        irq,
    input             iack,
    output reg  [7:0] iack_vector,
    output reg        iack_ack,
    output reg        iack_spurious,
    // GPIP
    input       [7:0] gpip_in,
    output      [7:0] gpip_out,
    output      [7:0] gpip_oe,
    // timers
    input             tai,
    input             tbi,
    output            tao,
    output            tbo,
    output            tco,
    output            tdo,
    // USART
    input             si,
    output            so
);

    // ------------------------------------------------------------------
    // 2.4576 MHz clock enable (fractional accumulator)
    reg [31:0] acc;
    reg        mfp_tick;
    always @(posedge clk) begin
        if (reset) begin
            acc      <= 32'd0;
            mfp_tick <= 1'b0;
        end else if (acc + MFP_HZ >= CLK_HZ) begin
            acc      <= acc + MFP_HZ - CLK_HZ;
            mfp_tick <= 1'b1;
        end else begin
            acc      <= acc + MFP_HZ;
            mfp_tick <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // bus decode
    wire [4:0] idx = bus_addr[5:1];
    wire [7:0] d   = bus_din[7:0];
    wire       wr  = bus_cs && bus_stb && bus_we && bus_lds;
    wire       rd  = bus_cs && bus_stb && !bus_we && bus_lds;

    wire wr_gpip  = wr && idx == 5'd0;
    wire wr_aer   = wr && idx == 5'd1;
    wire wr_ddr   = wr && idx == 5'd2;
    wire wr_iera  = wr && idx == 5'd3;
    wire wr_ierb  = wr && idx == 5'd4;
    wire wr_ipra  = wr && idx == 5'd5;
    wire wr_iprb  = wr && idx == 5'd6;
    wire wr_isra  = wr && idx == 5'd7;
    wire wr_isrb  = wr && idx == 5'd8;
    wire wr_imra  = wr && idx == 5'd9;
    wire wr_imrb  = wr && idx == 5'd10;
    wire wr_vr    = wr && idx == 5'd11;
    wire wr_tacr  = wr && idx == 5'd12;
    wire wr_tbcr  = wr && idx == 5'd13;
    wire wr_tcdcr = wr && idx == 5'd14;
    wire wr_tadr  = wr && idx == 5'd15;
    wire wr_tbdr  = wr && idx == 5'd16;
    wire wr_tcdr  = wr && idx == 5'd17;
    wire wr_tddr  = wr && idx == 5'd18;
    wire wr_scr   = wr && idx == 5'd19;
    wire wr_ucr   = wr && idx == 5'd20;
    wire wr_rsr   = wr && idx == 5'd21;
    wire wr_tsr   = wr && idx == 5'd22;
    wire wr_udr   = wr && idx == 5'd23;

    // ------------------------------------------------------------------
    // GPIP / AER / DDR
    reg [7:0] gpdr, aer, ddr;
    assign gpip_out = gpdr;
    assign gpip_oe  = ddr;
    wire [7:0] gpip_rd = (gpdr & ddr) | (gpip_in & ~ddr);

    // ------------------------------------------------------------------
    // timers
    /* verilator lint_off UNUSEDSIGNAL */
    wire [3:0] ta_ctrl, tb_ctrl, tc_ctrl, td_ctrl;   // tc/td_ctrl[3] is always 0
    /* verilator lint_on UNUSEDSIGNAL */
    wire [7:0] ta_data, tb_data, tc_data, td_data;
    wire [7:0] ta_cnt, tb_cnt, tc_cnt, td_cnt;
    wire       ta_to, tb_to, tc_to, td_to;

    falcon_mfp_timer timer_a (
        .clk(clk), .reset(reset), .mfp_tick(mfp_tick),
        .ctrl_wr(wr_tacr), .ctrl_din(d[3:0]),
        .data_wr(wr_tadr), .data_din(d),
        .out_reset(wr_tacr && d[4]),
        .tin(tai), .aer_bit(aer[4]),
        .ctrl(ta_ctrl), .data(ta_data), .counter(ta_cnt),
        .timeout(ta_to), .tout(tao));

    falcon_mfp_timer timer_b (
        .clk(clk), .reset(reset), .mfp_tick(mfp_tick),
        .ctrl_wr(wr_tbcr), .ctrl_din(d[3:0]),
        .data_wr(wr_tbdr), .data_din(d),
        .out_reset(wr_tbcr && d[4]),
        .tin(tbi), .aer_bit(aer[3]),
        .ctrl(tb_ctrl), .data(tb_data), .counter(tb_cnt),
        .timeout(tb_to), .tout(tbo));

    falcon_mfp_timer timer_c (
        .clk(clk), .reset(reset), .mfp_tick(mfp_tick),
        .ctrl_wr(wr_tcdcr), .ctrl_din({1'b0, d[6:4]}),
        .data_wr(wr_tcdr), .data_din(d),
        .out_reset(1'b0),
        .tin(1'b0), .aer_bit(1'b0),
        .ctrl(tc_ctrl), .data(tc_data), .counter(tc_cnt),
        .timeout(tc_to), .tout(tco));

    falcon_mfp_timer timer_d (
        .clk(clk), .reset(reset), .mfp_tick(mfp_tick),
        .ctrl_wr(wr_tcdcr), .ctrl_din({1'b0, d[2:0]}),
        .data_wr(wr_tddr), .data_din(d),
        .out_reset(1'b0),
        .tin(1'b0), .aer_bit(1'b0),
        .ctrl(td_ctrl), .data(td_data), .counter(td_cnt),
        .timeout(td_to), .tout(tdo));

    wire ta_pwm = ta_ctrl[3] && (ta_ctrl[2:0] != 3'd0);
    wire tb_pwm = tb_ctrl[3] && (tb_ctrl[2:0] != 3'd0);

    // USART clock = TDO
    reg  tdo_prev;
    always @(posedge clk) tdo_prev <= tdo;
    wire tc_rise = tdo && !tdo_prev;
    wire tc_fall = !tdo && tdo_prev;

    // ------------------------------------------------------------------
    // interrupt registers
    reg [15:0] ier, ipr, isr, imr;
    reg  [7:3] vr;

    // ------------------------------------------------------------------
    // USART
    wire [7:0] scr_q, ucr_q, rsr_q, tsr_q, udr_q;
    wire       ev_tx_err, ev_tx_empty, ev_rx_err, ev_rx_full;

    falcon_mfp_usart usart (
        .clk(clk), .reset(reset),
        .tc_rise(tc_rise), .tc_fall(tc_fall),
        .scr_wr(wr_scr), .ucr_wr(wr_ucr), .rsr_wr(wr_rsr), .tsr_wr(wr_tsr),
        .udr_wr(wr_udr), .din(d),
        .rsr_rd(rd && idx == 5'd21), .tsr_rd(rd && idx == 5'd22),
        .udr_rd(rd && idx == 5'd23),
        .scr(scr_q), .ucr_q(ucr_q), .rsr_q(rsr_q), .tsr_q(tsr_q), .udr_rx(udr_q),
        .si(si), .so(so),
        .rx_err_ena(ier[11]),
        .ev_tx_err(ev_tx_err), .ev_tx_empty(ev_tx_empty),
        .ev_rx_err(ev_rx_err), .ev_rx_full(ev_rx_full));

    // ------------------------------------------------------------------
    // GPIP edge detection (Hatari MFP_GPIP_Update_Interrupt):
    // interrupt when (line XNOR AER) goes 0 -> 1 on a DDR=0 line.
    wire [7:0] gp_state = ~(gpip_in ^ aer);
    reg  [7:0] gp_state_prev;
    wire [7:0] gp_ev = gp_state & ~gp_state_prev & ~ddr;

    // pulse width mode: end of the pulse = input leaves the AER level
    wire tai_off = tai ^ aer[4];
    wire tbi_off = tbi ^ aer[3];
    reg  tai_off_prev, tbi_off_prev;
    wire tai_end = tai_off && !tai_off_prev;
    wire tbi_end = tbi_off && !tbi_off_prev;

    always @(posedge clk) begin
        gp_state_prev <= gp_state;
        tai_off_prev  <= tai_off;
        tbi_off_prev  <= tbi_off;
    end

    wire [15:0] ev = {
        gp_ev[7], gp_ev[6], ta_to, ev_rx_full,
        ev_rx_err, ev_tx_empty, ev_tx_err, tb_to,
        gp_ev[5], ta_pwm ? tai_end : gp_ev[4], tc_to, td_to,
        tb_pwm ? tbi_end : gp_ev[3], gp_ev[2], gp_ev[1], gp_ev[0] };

    // ------------------------------------------------------------------
    // priority logic
    function [4:0] msb16;    // {valid, index of the highest set bit}
        input [15:0] v;
        integer i;
        begin
            msb16 = 5'd0;
            for (i = 0; i < 16; i = i + 1)
                if (v[i]) msb16 = {1'b1, i[3:0]};
        end
    endfunction

    wire [15:0] pend   = ipr & imr;
    wire [4:0]  pend_m = msb16(pend);
    wire [4:0]  isr_m  = msb16(isr);
    wire        req    = pend_m[4] && (!isr_m[4] || pend_m[3:0] > isr_m[3:0]);
    wire [15:0] req_bit = 16'd1 << pend_m[3:0];
    wire        ack_ch  = iack && req;

    // next state
    reg [15:0] ier_n, ipr_n, isr_n, imr_n;
    always @(*) begin
        imr_n = imr;
        if (wr_imra) imr_n[15:8] = d;
        if (wr_imrb) imr_n[7:0]  = d;

        ier_n = ier;
        if (wr_iera) ier_n[15:8] = d;
        if (wr_ierb) ier_n[7:0]  = d;

        ipr_n = ipr;
        if (wr_ipra) ipr_n[15:8] = ipr_n[15:8] & d;
        if (wr_iprb) ipr_n[7:0]  = ipr_n[7:0]  & d;
        if (ack_ch)  ipr_n = ipr_n & ~req_bit;
        ipr_n = (ipr_n | ev) & ier_n;

        isr_n = isr;
        if (wr_isra) isr_n[15:8] = isr_n[15:8] & d;
        if (wr_isrb) isr_n[7:0]  = isr_n[7:0]  & d;
        if (ack_ch) begin
            if (vr[3]) isr_n = isr_n | req_bit;
            else       isr_n = isr_n & ~req_bit;
        end
        if (wr_vr && vr[3] && !d[3])
            isr_n = 16'd0;
    end

    // request with the next state: irq follows IPR/ISR/IMR changes (IACK,
    // EOI, mask writes) on the same clock as the registers
    wire [4:0] pend_n_m = msb16(ipr_n & imr_n);
    wire [4:0] isr_n_m  = msb16(isr_n);
    wire       req_n    = pend_n_m[4] && (!isr_n_m[4] || pend_n_m[3:0] > isr_n_m[3:0]);

    always @(posedge clk) begin
        iack_ack      <= 1'b0;
        iack_spurious <= 1'b0;
        if (reset) begin
            gpdr <= 8'd0;
            aer  <= 8'd0;
            ddr  <= 8'd0;
            ier  <= 16'd0;
            ipr  <= 16'd0;
            isr  <= 16'd0;
            imr  <= 16'd0;
            vr   <= 5'd0;
            irq  <= 1'b0;
            iack_vector <= 8'd0;
        end else begin
            if (wr_gpip) gpdr <= d;
            if (wr_aer)  aer  <= d;
            if (wr_ddr)  ddr  <= d;
            imr  <= imr_n;
            if (wr_vr)   vr <= d[7:3];
            ier <= ier_n;
            ipr <= ipr_n;
            isr <= isr_n;
            irq <= req_n;
            if (iack) begin
                iack_ack <= 1'b1;
                if (req)
                    iack_vector <= {vr[7:4], pend_m[3:0]};
                else begin
                    iack_vector   <= 8'h18;
                    iack_spurious <= 1'b1;
                end
            end
        end
    end

    // ------------------------------------------------------------------
    // register read
    reg [7:0] rdata;
    always @(*) begin
        case (idx)
            5'd0:  rdata = gpip_rd;
            5'd1:  rdata = aer;
            5'd2:  rdata = ddr;
            5'd3:  rdata = ier[15:8];
            5'd4:  rdata = ier[7:0];
            5'd5:  rdata = ipr[15:8];
            5'd6:  rdata = ipr[7:0];
            5'd7:  rdata = isr[15:8];
            5'd8:  rdata = isr[7:0];
            5'd9:  rdata = imr[15:8];
            5'd10: rdata = imr[7:0];
            5'd11: rdata = {vr, 3'b000};
            5'd12: rdata = {4'h0, ta_ctrl};
            5'd13: rdata = {4'h0, tb_ctrl};
            5'd14: rdata = {1'b0, tc_ctrl[2:0], 1'b0, td_ctrl[2:0]};
            5'd15: rdata = ta_cnt;
            5'd16: rdata = tb_cnt;
            5'd17: rdata = tc_cnt;
            5'd18: rdata = td_cnt;
            5'd19: rdata = scr_q;
            5'd20: rdata = ucr_q;
            5'd21: rdata = rsr_q;
            5'd22: rdata = tsr_q;
            5'd23: rdata = udr_q;
            default: rdata = 8'hFF;
        endcase
    end

    always @(posedge clk) begin
        bus_ack <= 1'b0;
        if (reset) begin
            bus_dout <= 16'hFFFF;
        end else if (bus_cs && bus_stb) begin
            bus_ack  <= 1'b1;
            bus_dout <= {8'hFF, rdata};
        end
    end

    // unused inputs of the common bus
    /* verilator lint_off UNUSEDSIGNAL */
    wire unused_ok = bus_uds | (|bus_din[15:8]) | (|ta_data) | (|tb_data) | (|tc_data) | (|td_data);
    /* verilator lint_on UNUSEDSIGNAL */

endmodule
