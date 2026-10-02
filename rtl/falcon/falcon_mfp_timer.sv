// falcon_mfp_timer.sv - one MC68901 timer (A, B, C or D) for falcon_mfp.
//
// Behavioural reference: Hatari src/mfp.c
//   MFP_StartTimer_AB / MFP_StartTimer_CD   (start: prescaler restarts at the
//                                            control write, first timeout after
//                                            counter * prescale MFP clocks)
//   MFP_ReadTimer_AB / MFP_ReadTimer_CD      (data register read returns the
//                                            main counter; Hatari computes it
//                                            as ceil(remaining / prescale),
//                                            which is exactly the value of the
//                                            down counter kept here)
//   MFP_TimerxCtrl_WriteByte                 (stop keeps the counter; stopping
//                                            while the counter is between 1
//                                            and 0 reloads it from the data
//                                            register, Hatari 2025/05/07 fix)
//   MFP_TimerxData_WriteByte                 (data write also loads the counter
//                                            only while the timer is stopped)
//   MFP_TimerA_Set_Line_Input / MFP_TimerB_EventCount (event count mode:
//                                            count when the input changes to
//                                            the level of the AER bit; counter
//                                            1 -> reload + interrupt)
//
// Control value ctrl[3:0] as in TACR/TBCR: 0 stop, 1..7 delay with prescale
// 4,10,16,50,64,100,200, 8 event count, 9..15 pulse width (delay mode gated
// by the timer input).  Timers C and D use ctrl[3] = 0.
//
// Deviations:
// - Pulse width mode is implemented as on the MC68901 (the prescaler only
//   runs while the input TAI/TBI is at the level selected by the AER bit);
//   Hatari treats pulse width mode as delay mode.
// - Changing the prescale of a running timer keeps the current counter value
//   (hardware); Hatari restarts from its last stored MAINCOUNTER value.
//
// Copyright (C) 2026 Falcon_MiSTer project.  GPL v2 or later (as Hatari).

module falcon_mfp_timer (
    input            clk,
    input            reset,
    input            mfp_tick,     // 2.4576 MHz clock enable
    input            ctrl_wr,      // write to the control field (one clock)
    input      [3:0] ctrl_din,
    input            data_wr,      // write to the data register (one clock)
    input      [7:0] data_din,
    input            out_reset,    // TACR/TBCR bit 4 written as 1: force output low
    input            tin,          // timer input (TAI/TBI), 0 for C/D
    input            aer_bit,      // AER bit associated with tin
    output reg [3:0] ctrl,
    output reg [7:0] data,         // timer data register (reload value)
    output reg [7:0] counter,      // main counter (value read at TxDR)
    output reg       timeout,      // one clock pulse per timeout
    output reg       tout          // timer output, toggles at each timeout
);

    reg [7:0] presc;
    reg       tin_prev;

    function [7:0] div_last;
        input [2:0] p;
        case (p)
            3'd1: div_last = 8'd3;    // /4
            3'd2: div_last = 8'd9;    // /10
            3'd3: div_last = 8'd15;   // /16
            3'd4: div_last = 8'd49;   // /50
            3'd5: div_last = 8'd63;   // /64
            3'd6: div_last = 8'd99;   // /100
            3'd7: div_last = 8'd199;  // /200
            default: div_last = 8'd0;
        endcase
    endfunction

    wire       mode_delay = (ctrl[3] == 1'b0) && (ctrl[2:0] != 3'd0);
    wire       mode_event = (ctrl == 4'd8);
    wire       mode_pwm   = ctrl[3] && (ctrl[2:0] != 3'd0);
    wire       gate       = mode_delay || (mode_pwm && (tin == aer_bit));
    wire       presc_end  = (presc == div_last(ctrl[2:0]));
    // event count: the input changes to the level of the AER bit
    wire       ev_edge    = (tin != tin_prev) && (tin == aer_bit);
    wire       dec        = mode_event ? ev_edge : (gate && mfp_tick && presc_end);

    always @(posedge clk) begin
        timeout  <= 1'b0;
        tin_prev <= tin;
        if (reset) begin
            ctrl     <= 4'd0;
            data     <= 8'd0;
            counter  <= 8'd0;
            presc    <= 8'd0;
            tout     <= 1'b0;
            tin_prev <= tin;
        end else begin
            // prescaler
            if (gate && mfp_tick)
                presc <= presc_end ? 8'd0 : presc + 8'd1;

            // main counter
            if (dec) begin
                if (counter == 8'd1) begin
                    counter <= data;
                    timeout <= 1'b1;
                    tout    <= ~tout;
                end else begin
                    counter <= counter - 8'd1;
                end
            end

            if (ctrl_wr && (ctrl_din != ctrl)) begin
                ctrl  <= ctrl_din;
                presc <= 8'd0;
                // stopping a delay mode timer while its counter is between 1
                // and 0 (counter 1 and the prescaler already advanced): the
                // next start uses the data register (Hatari MFP_ReadTimer_xx
                // with TimerIsStopping).
                if (ctrl_din == 4'd0 && mode_delay && counter == 8'd1 && presc != 8'd0)
                    counter <= data;
            end

            if (data_wr) begin
                data <= data_din;
                if (ctrl == 4'd0 && !(ctrl_wr && ctrl_din != 4'd0))
                    counter <= data_din;
            end

            if (out_reset)
                tout <= 1'b0;
        end
    end

endmodule
