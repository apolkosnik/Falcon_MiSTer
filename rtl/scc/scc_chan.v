// One channel of the Z85C30 SCC, asynchronous modes only: registers, baud rate
// generator, transmitter, receiver with its 3-byte FIFO, external/status latching,
// and the channel's three interrupt-pending sources. scc.v holds the shared parts
// (WR2, WR9, IUS, RR3) and the bus side.
//
// Copyright (C) 2026 the contributors
// This program is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation; either version 2 of the License, or (at your option) any later
// version.
//
// Reference: Hatari's scc.c (Nicolas Pomarede, Thomas Huth; from ARAnyM), and the
// Z85C30 manual. Not modelled: synchronous and SDLC modes, the DPLL (selected as a
// clock it runs as the BRG), auto enables, TRxC as an output, the crystal option,
// the special-condition FIFO lock.

module scc_chan (
	input            clk,
	input            reset_hw,      // hardware reset, or WR9 "force hardware reset"
	input            reset_ch,      // WR9 channel reset

	input            pclk_ce,       // PCLK edges (8 MHz on the Mega STe/TT)
	input            rtxc_ce,       // RTxC rising edges
	input            trxc_ce,       // TRxC rising edges

	// register writes from scc.v (WR1, WR3-WR8, WR10-WR15; data = 8)
	input            wr,
	input      [3:0] wreg,
	input      [7:0] wdata,

	// WR0 commands for this channel
	input            cmd,
	input      [2:0] cmd_code,
	input      [1:0] crc_code,

	input            rd_data,       // a data read: pops the receive FIFO
	input      [3:0] rreg,
	output reg [7:0] rdata,         // RR0, RR1, RR8, RR10, RR12, RR13, RR15 (and aliases)

	output           ip_rx,
	output           ip_tx,
	output           ip_ext,
	output           special,       // the receive IP is a special condition (vector status)

	input            rxd,
	output reg       txd,
	input            dcd,           // asserted = high (the /DCD pin low)
	input            cts,           // asserted = high
	input            sync,          // asserted = high (async: the /SYNC pin)
	output           rts,
	output           dtr
);

// ---------------------------------------------------------------- registers

// WR6/WR7 (sync characters) and parts of WR10/WR11 are kept but not modelled
/* verilator lint_off UNUSEDSIGNAL */
reg [7:0] wr1, wr3, wr4, wr5, wr6, wr7, wr10, wr11, wr12, wr13, wr14, wr15;
/* verilator lint_on UNUSEDSIGNAL */

assign rts = wr5[1];
assign dtr = wr5[7];

wire [1:0] rx_mode  = wr1[4:3];        // 0 off, 1 first char/special, 2 all/special, 3 special only
wire       rx_en    = wr3[0];
wire       tx_en    = wr5[3];
wire       brk_tx   = wr5[4];
wire [1:0] tx_bits  = wr5[6:5];        // 0: 5 bits, 1: 7, 2: 6, 3: 8
wire [1:0] rx_bits  = wr3[7:6];        // 0: 5 bits, 1: 7, 2: 6, 3: 8
wire       par_en   = wr4[0];
wire       par_even = wr4[1];
wire [1:0] stop     = wr4[3:2];        // 0 sync, 1: 1 bit, 2: 1.5, 3: 2
wire [1:0] cmode    = wr4[7:6];        // x1 x16 x32 x64

function [3:0] nbits(input [1:0] code);
	case (code)
		2'd0: nbits = 4'd5;
		2'd1: nbits = 4'd7;
		2'd2: nbits = 4'd6;
		2'd3: nbits = 4'd8;
	endcase
endfunction

// clock edges per bit, minus one
wire [5:0] bit_len = cmode == 2'd0 ? 6'd0 : cmode == 2'd1 ? 6'd15 : cmode == 2'd2 ? 6'd31 : 6'd63;

// ---------------------------------------------------------------- baud rate generator

reg  [16:0] brg_cnt;                   // TC+2 source edges a half period: rate = src / (2 (TC+2))
reg         brg_out, brg_rise, zc_pulse;
wire        brg_src = wr14[1] ? pclk_ce : rtxc_ce;

always @(posedge clk) begin
	brg_rise <= 0;
	zc_pulse <= 0;
	if (reset_hw || reset_ch) begin
		brg_cnt <= 0;
		brg_out <= 0;
	end else if (!wr14[0]) begin
		brg_cnt <= {1'b0, wr13, wr12} + 1'd1;   // stopped: loads the time constant
	end else if (brg_src) begin
		if (brg_cnt == 0) begin
			brg_cnt  <= {1'b0, wr13, wr12} + 1'd1;
			brg_out  <= ~brg_out;
			brg_rise <= ~brg_out;
			zc_pulse <= 1;
		end else
			brg_cnt <= brg_cnt - 1'd1;
	end
end

function clk_sel(input [1:0] s, input rt, input tr, input br);
	case (s)
		2'd0: clk_sel = rt;
		2'd1: clk_sel = tr;
		default: clk_sel = br;              // BRG, or the DPLL run as the BRG
	endcase
endfunction

wire tx_clk = clk_sel(wr11[4:3], rtxc_ce, trxc_ce, brg_rise);
wire rx_clk = clk_sel(wr11[6:5], rtxc_ce, trxc_ce, brg_rise);

// ---------------------------------------------------------------- transmitter

reg  [7:0] tdr, tsr;
reg        tdr_full, tsr_full, tx_written, all_sent;
reg  [3:0] tx_state;                    // 0 idle, 1 start, 2 data, 3 parity, 4 stop
reg  [2:0] tx_bitn;
reg  [6:0] tx_cnt;
reg        tx_par;

wire [6:0] tx_len_bit  = {1'b0, bit_len};
wire [6:0] tx_len_stop = stop == 2'd3 ? {bit_len, 1'b1} :                          // 2 bits
                         stop == 2'd2 ? {1'b0, bit_len} + {2'b0, bit_len[5:1]} + 1'd1 : // 1.5
                         {1'b0, bit_len};

always @(posedge clk) begin
	if (reset_hw || reset_ch) begin
		tdr_full   <= 0;
		tsr_full   <= 0;
		tx_state   <= 0;
		txd        <= 1;
		all_sent   <= 1;
		tx_written <= 0;
	end else begin
		if (wr && wreg == 4'd8) begin
			tdr        <= wdata;
			tdr_full   <= 1;
			tx_written <= 1;
		end
		if (cmd && cmd_code == 3'd5) tx_written <= 0;    // reset Tx interrupt pending

		// the buffer moves to the shift register as soon as that is free
		if (tdr_full && !tsr_full && !(wr && wreg == 4'd8)) begin
			tsr      <= tdr;
			tsr_full <= 1;
			tdr_full <= 0;
			all_sent <= 0;
		end

		if (!tx_en) begin
			tx_state <= 0;
			txd      <= !brk_tx;
		end else if (tx_clk) begin
			if (tx_cnt != 0)
				tx_cnt <= tx_cnt - 1'd1;
			else case (tx_state)
				4'd0: if (tsr_full) begin
					txd      <= 0;                // start bit
					tx_state <= 1;
					tx_cnt   <= tx_len_bit;
					tx_bitn  <= 0;
					tx_par   <= !par_even;
				end else
					txd <= 1;
				4'd1, 4'd2: begin
					txd      <= tsr[tx_bitn];
					tx_par   <= tx_par ^ tsr[tx_bitn];
					tx_cnt   <= tx_len_bit;
					tx_state <= 2;
					if ({1'b0, tx_bitn} == nbits(tx_bits) - 1'd1)
						tx_state <= par_en ? 4'd3 : 4'd4;
					else
						tx_bitn <= tx_bitn + 1'd1;
				end
				4'd3: begin
					txd      <= tx_par;
					tx_cnt   <= tx_len_bit;
					tx_state <= 4;
				end
				4'd4: begin
					txd      <= 1;                // stop bit(s)
					tx_cnt   <= tx_len_stop;
					tx_state <= 5;
				end
				default: begin                    // stop done
					tsr_full <= 0;
					all_sent <= !tdr_full;
					tx_state <= 0;
				end
			endcase
			if (brk_tx) txd <= 0;
		end else if (brk_tx)
			txd <= 0;
	end
end

wire tx_empty = !tdr_full;

// ---------------------------------------------------------------- receiver

reg  [7:0] fifo_d [0:2];
reg  [2:0] fifo_fe, fifo_pe;
reg  [1:0] fifo_n;
reg        overrun, pe_latch;
reg  [3:0] rx_state;                    // 0 hunt, 1 start, 2 data, 3 parity, 4 stop
reg  [2:0] rx_bitn;
reg  [6:0] rx_cnt;
reg  [7:0] rx_sr;
reg        rx_par, rx_zero;
reg        rxd_s, rxd_m;
reg        brk_det;
reg        first_armed;
reg        rx_push;
reg        push_fe, push_pe;
reg  [7:0] push_d;

always @(posedge clk) begin
	rxd_m <= rxd;
	rxd_s <= rxd_m;
end

integer k;
always @(posedge clk) begin
	rx_push <= 0;
	if (reset_hw || reset_ch) begin
		rx_state <= 0;
		brk_det  <= 0;
	end else if (!rx_en) begin
		rx_state <= 0;
	end else if (rx_clk) begin
		if (rx_state == 0) begin
			if (rxd_s)
				brk_det <= 0;                     // after a break, hunt only once the line is high
			else if (!brk_det) begin          // a start edge: go to its middle
				rx_state <= 1;
				rx_cnt   <= {2'b0, bit_len[5:1]};
			end
		end else if (rx_cnt != 0)
			rx_cnt <= rx_cnt - 1'd1;
		else case (rx_state)
			4'd1: if (rxd_s) rx_state <= 0;   // a glitch, not a start bit
			      else begin
				rx_state <= 2;
				rx_cnt   <= {1'b0, bit_len};
				rx_bitn  <= 0;
				rx_par   <= !par_even;
				rx_zero  <= 1;
				rx_sr    <= 0;
			      end
			4'd2: begin
				rx_sr[rx_bitn] <= rxd_s;
				rx_par  <= rx_par ^ rxd_s;
				rx_zero <= rx_zero & !rxd_s;
				rx_cnt  <= {1'b0, bit_len};
				if ({1'b0, rx_bitn} == nbits(rx_bits) - 1'd1)
					rx_state <= par_en ? 4'd3 : 4'd4;
				else
					rx_bitn <= rx_bitn + 1'd1;
			end
			4'd3: begin
				push_pe  <= rx_par != rxd_s;
				rx_zero  <= rx_zero & !rxd_s;
				rx_cnt   <= {1'b0, bit_len};
				rx_state <= 4;
			end
			4'd4: begin
				if (!par_en) push_pe <= 0;
				if (!rxd_s && rx_zero) begin      // all zeros, no stop bit: a break
					brk_det  <= 1;
					rx_state <= 0;
					push_d   <= 8'h00;           // the break's single null character
					push_fe  <= 1;
					rx_push  <= !brk_det;
				end else begin
					push_d   <= rx_sr;
					push_fe  <= !rxd_s;
					rx_push  <= 1;
					rx_state <= 0;
				end
			end
			default: rx_state <= 0;
		endcase
	end
end

wire rx_avail = fifo_n != 0;

always @(posedge clk) begin
	if (reset_hw || reset_ch) begin
		fifo_n      <= 0;
		overrun     <= 0;
		pe_latch    <= 0;
		first_armed <= 1;
	end else begin
		if (cmd && cmd_code == 3'd6) begin   // error reset
			overrun  <= 0;
			pe_latch <= 0;
		end
		if (cmd && cmd_code == 3'd4) first_armed <= 1;   // interrupt on next Rx character
		if (rd_data && rx_avail && rx_mode == 2'd1) first_armed <= 0;

		case ({rx_push, rd_data && rx_avail})
			2'b10: if (fifo_n == 2'd3) begin
				fifo_d[2] <= push_d; fifo_fe[2] <= push_fe; fifo_pe[2] <= push_pe;
				overrun   <= 1;
			end else begin
				fifo_d[fifo_n] <= push_d; fifo_fe[fifo_n] <= push_fe; fifo_pe[fifo_n] <= push_pe;
				fifo_n <= fifo_n + 1'd1;
				if (push_pe) pe_latch <= 1;
			end
			2'b01, 2'b11: begin
				for (k = 0; k < 2; k = k + 1) begin
					fifo_d[k] <= fifo_d[k + 1]; fifo_fe[k] <= fifo_fe[k + 1]; fifo_pe[k] <= fifo_pe[k + 1];
				end
				if (rx_push) begin
					fifo_d[fifo_n - 1'd1] <= push_d; fifo_fe[fifo_n - 1'd1] <= push_fe;
					fifo_pe[fifo_n - 1'd1] <= push_pe;
					if (push_pe) pe_latch <= 1;
				end else
					fifo_n <= fifo_n - 1'd1;
			end
			default: ;
		endcase
	end
end

// ---------------------------------------------------------------- external/status

reg  [7:0] rr0_latch;
reg        ext_latched, ext_ip;
reg        zc;
reg        underrun;

// live values of the external/status bits (RR0 bits 1, 3-7)
wire [7:0] ext_live = {brk_det, underrun, cts, sync, dcd, 2'b00, zc};
wire [7:0] ext_ie   = {wr15[7:3], 2'b00, wr15[1]};   // break, underrun, CTS, sync, DCD, -, -, ZC
/* verilator lint_off UNUSEDSIGNAL */
wire [7:0] ext_now  = ext_latched ? (ext_live & ~ext_ie | rr0_latch & ext_ie) : ext_live;
/* verilator lint_on UNUSEDSIGNAL */
reg  [7:0] ext_prev;

// a change that sets the IP: ZC and underrun on 0->1, the others on any change
wire [7:0] ext_chg  = (ext_live ^ ext_prev) & ext_ie & ~(8'h42 & ext_prev);

always @(posedge clk) begin
	if (reset_hw || reset_ch) begin
		ext_latched <= 0;
		ext_ip      <= 0;
		zc          <= 0;
		underrun    <= 1;
		ext_prev    <= 0;
	end else begin
		if (zc_pulse && wr15[1]) zc <= 1;
		else if (!wr15[1]) zc <= 0;
		else if (zc && !zc_pulse) zc <= 0;   // ZC is a pulse while the count is 0

		if (cmd && crc_code == 2'd3) underrun <= 0;            // reset Tx underrun/EOM latch
		if (wr && wreg == 4'd4 && wdata[3:2] != 2'd0) underrun <= 1;   // async mode

		ext_prev <= ext_live;
		if (cmd && cmd_code == 3'd2) begin   // reset ext/status interrupts
			// the latches open; a change made while they were closed interrupts again
			if (wr1[0] && ((ext_live ^ rr0_latch) & ext_ie & ~(8'h42 & rr0_latch)) != 0) begin
				rr0_latch <= ext_live;
				ext_ip    <= 1;
			end else begin
				ext_latched <= 0;
				ext_ip      <= 0;
			end
		end else if (wr1[0] && !ext_latched && ext_chg != 0) begin
			ext_latched <= 1;
			rr0_latch   <= ext_live;
			ext_ip      <= 1;
		end
	end
end

// ---------------------------------------------------------------- interrupt pendings

wire spec_cond = overrun || (rx_avail && fifo_fe[0]) || (pe_latch && wr1[2]);

assign special = rx_mode != 2'd0 && spec_cond;
assign ip_rx   = special ||
                 (rx_avail && (rx_mode == 2'd2 || (rx_mode == 2'd1 && first_armed)));
assign ip_tx   = wr1[1] && tx_empty && tx_written;
assign ip_ext  = ext_ip;

// ---------------------------------------------------------------- register writes

always @(posedge clk) begin
	if (reset_hw) begin
		wr1  <= wr1 & 8'h24;  wr3 <= wr3 & 8'hfe;  wr4 <= wr4 | 8'h04;  wr5 <= wr5 & 8'h61;
		wr10 <= 8'h00;        wr11 <= 8'h08;       wr14 <= (wr14 & 8'hc0) | 8'h30;
		wr15 <= 8'hf8;
	end else if (reset_ch) begin
		wr1  <= wr1 & 8'h24;  wr3 <= wr3 & 8'hfe;  wr4 <= wr4 | 8'h04;  wr5 <= wr5 & 8'h61;
		wr10 <= wr10 & 8'h60; wr14 <= (wr14 & 8'hc3) | 8'h20;
		wr15 <= 8'hf8;
	end else if (wr) case (wreg)
		4'd1:  wr1  <= wdata;
		4'd3:  wr3  <= wdata;
		4'd4:  wr4  <= wdata;
		4'd5:  wr5  <= wdata;
		4'd6:  wr6  <= wdata;
		4'd7:  wr7  <= wdata;
		4'd10: wr10 <= wdata;
		4'd11: wr11 <= wdata;
		4'd12: wr12 <= wdata;
		4'd13: wr13 <= wdata;
		4'd14: wr14 <= wdata;
		4'd15: wr15 <= wdata;
		default: ;
	endcase
end

// ---------------------------------------------------------------- register reads

wire [7:0] rr0 = {ext_now[7:3], tx_empty, ext_now[1], rx_avail};
wire [7:0] rr1 = {1'b0, rx_avail && fifo_fe[0], overrun, pe_latch, 3'b011, all_sent};

always @(*) begin
	case (rreg)
		4'd0, 4'd4:   rdata = rr0;
		4'd1, 4'd5:   rdata = rr1;
		4'd8:         rdata = rx_avail ? fifo_d[0] : 8'h00;
		4'd10, 4'd14: rdata = 8'h00;     // DPLL/SDLC status: not modelled
		4'd12:        rdata = wr12;
		4'd13, 4'd9:  rdata = wr13;
		4'd15, 4'd11: rdata = wr15 & 8'hfa;
		default:      rdata = 8'h00;     // RR2, RR3, RR6, RR7 come from scc.v
	endcase
end

endmodule
