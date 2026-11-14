// Z85C30 SCC, asynchronous modes, for the Mega STe, TT030 and Falcon: the bus side,
// the register pointer, the shared WR2/WR9, the interrupt daisy chain (RR3, IUS) and
// the interrupt acknowledge. Two scc_chan instances do the rest.
//
// Copyright (C) 2026 the contributors
// This program is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation; either version 2 of the License, or (at your option) any later
// version.
//
// Bus: four byte registers, A control, A data, B control, B data -- on the Mega STe
// and TT $FF8C81, $FF8C83, $FF8C85, $FF8C87, so addr[2] picks the channel and
// addr[1] data. sel is a one-clock strobe per bus access; dout is valid in that
// clock. Read side effects (the receive FIFO, a software IACK through RR2) happen
// once per strobe.
//
// Interrupts: irq is the /INT pin, active high here. On an acknowledge of its level
// (5 on the Mega STe and TT) the host strobes iack once; vector then holds what the
// SCC drives on the bus, and vector_en is low when WR9 says No Vector.
//
// Reference: Hatari's scc.c (Nicolas Pomarede, Thomas Huth; from ARAnyM), and the
// Z85C30 manual. Differences from Hatari: WR0 sets the pointer and runs a command
// in one write, as the chip does; data-port accesses leave the pointer alone; the
// receive FIFO is 3 bytes deep; bytes go out and come in bit by bit.

module scc (
	input            clk,
	input            reset,

	input            sel,
	input      [2:1] addr,
	input            we,
	input      [7:0] din,
	output reg [7:0] dout,

	output           irq,
	input            iack,
	output     [7:0] vector,
	output           vector_en,

	input            pclk_ce,       // PCLK edges, 8 MHz
	input      [1:0] rtxc_ce,       // per channel, [0] = A
	input      [1:0] trxc_ce,

	input      [1:0] rxd,
	output     [1:0] txd,
	input      [1:0] dcd,           // asserted = high
	input      [1:0] cts,
	input      [1:0] sync,
	output     [1:0] rts,
	output     [1:0] dtr
);

wire       chn  = addr[2];          // 0 = A, 1 = B
wire       data = addr[1];

reg  [3:0] ptr;
reg  [7:0] wr2;
/* verilator lint_off UNUSEDSIGNAL */
reg  [7:0] wr9;                     // D7-D6 are commands, D2 (DLC) has no lower chain here
/* verilator lint_on UNUSEDSIGNAL */
reg  [5:0] ius;                     // as RR3: 5 A Rx, 4 A Tx, 3 A Ext, 2 B Rx, 1 B Tx, 0 B Ext

// WR0 decode
wire [2:0] w0_cmd = din[5:3];
wire [1:0] w0_crc = din[7:6];

wire ctl_wr  = sel && we && !data;
wire ctl_rd  = sel && !we && !data;
wire wr0     = ctl_wr && ptr == 4'd0;
wire wr_reg  = (ctl_wr && ptr != 4'd0) || (sel && we && data);
wire [3:0] wreg = data ? 4'd8 : ptr;

// WR9 resets: 11 force hardware reset, 10 channel A, 01 channel B
wire wr9_hit   = wr_reg && wreg == 4'd9;
wire hw_reset  = reset || (wr9_hit && din[7:6] == 2'b11);
wire [1:0] ch_reset = {wr9_hit && din[7:6] == 2'b01, wr9_hit && din[7:6] == 2'b10};

wire [7:0] rd_a, rd_b;
wire       ipr_a, ipt_a, ipe_a, sp_a, ipr_b, ipt_b, ipe_b, sp_b;

wire [5:0] ip = {ipr_a, ipt_a, ipe_a, ipr_b, ipt_b, ipe_b};
wire [3:0] rreg = data ? 4'd8 : ptr;
wire       wr_chan = wr_reg && wreg != 4'd2 && wreg != 4'd9;   // WR2, WR9 are shared, here
wire       rd_fifo = sel && !we && rreg == 4'd8;

scc_chan cha (
	.clk(clk), .reset_hw(hw_reset), .reset_ch(ch_reset[0]),
	.pclk_ce(pclk_ce), .rtxc_ce(rtxc_ce[0]), .trxc_ce(trxc_ce[0]),
	.wr(wr_chan && !chn), .wreg(wreg), .wdata(din),
	.cmd(wr0 && !chn), .cmd_code(w0_cmd), .crc_code(w0_crc),
	.rd_data(rd_fifo && !chn), .rreg(rreg), .rdata(rd_a),
	.ip_rx(ipr_a), .ip_tx(ipt_a), .ip_ext(ipe_a), .special(sp_a),
	.rxd(rxd[0]), .txd(txd[0]), .dcd(dcd[0]), .cts(cts[0]), .sync(sync[0]),
	.rts(rts[0]), .dtr(dtr[0])
);

scc_chan chb (
	.clk(clk), .reset_hw(hw_reset), .reset_ch(ch_reset[1]),
	.pclk_ce(pclk_ce), .rtxc_ce(rtxc_ce[1]), .trxc_ce(trxc_ce[1]),
	.wr(wr_chan && chn), .wreg(wreg), .wdata(din),
	.cmd(wr0 && chn), .cmd_code(w0_cmd), .crc_code(w0_crc),
	.rd_data(rd_fifo && chn), .rreg(rreg), .rdata(rd_b),
	.ip_rx(ipr_b), .ip_tx(ipt_b), .ip_ext(ipe_b), .special(sp_b),
	.rxd(rxd[1]), .txd(txd[1]), .dcd(dcd[1]), .cts(cts[1]), .sync(sync[1]),
	.rts(rts[1]), .dtr(dtr[1])
);

// ---------------------------------------------------------------- interrupts

// the highest pending source, and whether an IUS at or above it blocks it
reg  [2:0] top;                     // index into ip/ius, 7 = none
reg        blocked;
integer i;
always @(*) begin
	top = 3'd7;
	blocked = 0;
	for (i = 0; i < 6; i = i + 1) begin
		if (ius[i]) begin top = 3'd7; blocked = 1; end
		else if (ip[i]) begin top = i[2:0]; blocked = 0; end
	end
end
// the loop runs low to high, so the last hit is the highest: an IUS above a pending
// source blocks it, a pending source above an IUS wins
assign irq = wr9[3] && !blocked && top != 3'd7;

// vector status, as Hatari's SCC_Get_Vector_Status
reg [2:0] vstat;
always @(*) begin
	if      (ip[5]) vstat = sp_a ? 3'd7 : 3'd6;
	else if (ip[4]) vstat = 3'd4;
	else if (ip[3]) vstat = 3'd5;
	else if (ip[2]) vstat = sp_b ? 3'd3 : 3'd2;
	else if (ip[1]) vstat = 3'd0;
	else if (ip[0]) vstat = 3'd1;
	else            vstat = 3'd3;
end

wire [7:0] rr2b = wr9[4] ? {wr2[7], vstat[0], vstat[1], vstat[2], wr2[3:0]}
                         : {wr2[7:4], vstat, wr2[0]};

assign vector    = wr9[0] ? rr2b : wr2;
assign vector_en = !wr9[1];

// highest pending, for setting its IUS on an acknowledge
reg [5:0] hi_ip;
always @(*) begin
	hi_ip = 6'd0;
	if      (ip[5]) hi_ip = 6'b100000;
	else if (ip[4]) hi_ip = 6'b010000;
	else if (ip[3]) hi_ip = 6'b001000;
	else if (ip[2]) hi_ip = 6'b000100;
	else if (ip[1]) hi_ip = 6'b000010;
	else if (ip[0]) hi_ip = 6'b000001;
end

reg [5:0] hi_ius;
always @(*) begin
	hi_ius = 6'd0;
	if      (ius[5]) hi_ius = 6'b100000;
	else if (ius[4]) hi_ius = 6'b010000;
	else if (ius[3]) hi_ius = 6'b001000;
	else if (ius[2]) hi_ius = 6'b000100;
	else if (ius[1]) hi_ius = 6'b000010;
	else if (ius[0]) hi_ius = 6'b000001;
end

wire soft_iack = ctl_rd && ptr == 4'd2 && wr9[5];

// ---------------------------------------------------------------- state

always @(posedge clk) begin
	if (reset) begin
		ptr <= 0;
		wr2 <= 0;
		wr9 <= 8'hc0;
		ius <= 0;
	end else begin
		if (sel && !data) begin
			// the pointer returns to 0 after any control access but a WR0 that sets it
			if (wr0) ptr <= {w0_cmd == 3'd1, din[2:0]};
			else     ptr <= 0;
		end

		if (wr0 && w0_cmd == 3'd7) ius <= ius & ~hi_ius;   // reset highest IUS

		if (wr9_hit) begin
			// a force reset clears software IACK and keeps NV/VIS; status high/low, MIE
			// and DLC take the values written with it (Z85C30 manual); a channel
			// reset clears software IACK
			case (din[7:6])
				2'b11:   wr9 <= {2'b11, 1'b0, din[4:2], wr9[1:0]};
				2'b01,
				2'b10:   wr9 <= din & 8'hdf;
				default: wr9 <= din;
			endcase
			if (!din[3] || din[7:6] == 2'b11) ius <= 0;       // MIE off clears IUS
		end
		if (wr_reg && wreg == 4'd2) wr2 <= din;

		if ((iack || soft_iack) && irq) ius <= ius | hi_ip;
	end
end

// ---------------------------------------------------------------- reads

always @(*) begin
	if (data)
		dout = chn ? rd_b : rd_a;
	else case (ptr)
		4'd2, 4'd6: dout = chn ? rr2b : wr2;              // RR2B carries the status always
		4'd3, 4'd7: dout = chn ? 8'h00 : {2'b00, ip};     // RR3 on channel A only
		default:    dout = chn ? rd_b : rd_a;
	endcase
end

endmodule
