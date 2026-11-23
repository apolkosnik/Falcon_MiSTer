// SCU, the System Control Unit of the Mega STe and the TT030 ($FF8E01-$FF8E0F).
//
// Copyright (C) 2026 the contributors
// This program is free software; you can redistribute it and/or modify it under
// the terms of the GNU General Public License as published by the Free Software
// Foundation; either version 2 of the License, or (at your option) any later
// version.
//
// Every interrupt request of the machine passes through the SCU on its way to the
// CPU's IPL lines. Two masks gate them: the system mask for the motherboard sources
// (soft interrupt 1, HSYNC 2, VSYNC 4, SYSFAIL 7) and the VME mask for the VMEbus
// levels. The MFP (6) and the SCC (5) are wired to the VMEbus side even though they
// sit on the motherboard, so the VME mask gates them; TOS sets the system mask to
// $14 (HSYNC, VSYNC) and the VME mask to $60 (MFP, SCC).
//
// Reference: Hatari's scu_vme.c (Nicolas Pomarede, 2024), the TT030 Hardware
// Reference Manual (June 1990). Differences from Hatari:
// - the soft interrupt ($FF8E05 bit 0) raises level 1 (Hatari sets bit 0 of the
//   state, which no mask bit covers);
// - $FF8E07 reads back the VME interrupter (Hatari returns the system one);
// - the soft interrupt follows its register bit; a mask access does not drop it.
// HSYNC and VSYNC are latched here, as the GLUE latches them on an ST; a mask
// access drops the latched ones, as Hatari does. The MFP, the SCC and the VMEbus
// lines are levels and are seen live. The VMEbus IRQ3 that $FF8E07 drives is not
// modelled (no VMEbus); its bit only reads back.

module scu #(
	// 1: HSYNC/VSYNC are edges, latched here (a TT build). 0: they are the pending
	// latches of a GLUE that keeps them, as levels, and hv_clr asks it to drop them
	// on a mask access (the MiSTer ST core, whose GLUE also autovectors them)
	parameter LATCH_HV = 1
) (
	input            clk,
	input            reset,         // any reset: the registers clear
	input            cold,          // with reset: a power-on, which also clears GPR1/2

	// register port, byte wide on the odd addresses $FF8E01-$FF8E0F
	input            sel,           // the decoder's $FF8E0x select, one bus cycle
	input      [3:1] addr,
	input            we,
	input      [7:0] din,
	output reg [7:0] dout,

	// requests
	input            hsync,         // level-2 source: an edge, or the GLUE's pending latch
	input            vsync,         // level-4 source: an edge, or the GLUE's pending latch
	input            mfp_irq,       // level 6, active high, level
	input            scc_irq,       // level 5, active high, level
	input      [7:1] vme_irq,       // VMEbus IRQ1-7, active high (tie 0 without a bus)
	input            sysfail,       // VMEbus SYSFAIL: system level 7

	// acknowledge of an autovectored level (2 or 4) clears its latch
	input            iack,
	input      [2:0] iack_level,

	output           hv_clr,        // LATCH_HV = 0: drop the GLUE's HSYNC/VSYNC latches
	output     [7:1] irq,           // the requests that pass the masks, per level
	output     [2:0] ipl            // the highest of them, to the CPU (active high)
);

reg [7:0] sys_mask, vme_mask;
reg       soft_int, vme_int3;
reg [7:0] gpr1, gpr2;
reg       hs_lat, vs_lat;
reg       hsync_d, vsync_d;

wire sel_mask_sys = sel && addr == 3'd0;   // $FF8E01
wire sel_mask_vme = sel && addr == 3'd6;   // $FF8E0D

assign hv_clr = !LATCH_HV && (sel_mask_sys || sel_mask_vme);
wire   hs_pend = LATCH_HV ? hs_lat : hsync;
wire   vs_pend = LATCH_HV ? vs_lat : vsync;

// state as the registers show it: system levels 1-7, VMEbus levels 1-7
wire [7:0] sys_state = {sysfail, 2'b00, vs_pend, 1'b0, hs_pend, soft_int, 1'b0};
wire [7:0] vme_state = {vme_irq[7], mfp_irq | vme_irq[6], scc_irq | vme_irq[5], vme_irq[4:1], 1'b0};

// levels 5 and 6 only through the VME mask, the others through either
assign irq = (sys_state[7:1] & sys_mask[7:1] & 7'b1001111) | (vme_state[7:1] & vme_mask[7:1]);

assign ipl = irq[7] ? 3'd7 : irq[6] ? 3'd6 : irq[5] ? 3'd5 : irq[4] ? 3'd4 :
             irq[3] ? 3'd3 : irq[2] ? 3'd2 : irq[1] ? 3'd1 : 3'd0;

always @(posedge clk) begin
	hsync_d <= hsync;
	vsync_d <= vsync;

	if (reset) begin
		sys_mask <= 0;
		vme_mask <= 0;
		soft_int <= 0;
		vme_int3 <= 0;
		hs_lat   <= 0;
		vs_lat   <= 0;
		// TOS 2.0x/3.0x need GPR1 bit 0 set after a reset (Hatari: "TOS v2 / v3 crash
		// on MegaSTE / TT unless gen reg 1 has this value"); GPR2 survives a warm reset
		gpr1     <= 8'h01;
		if (cold) gpr2 <= 0;
	end else begin
		if (hsync && !hsync_d) hs_lat <= 1;
		if (vsync && !vsync_d) vs_lat <= 1;
		if (iack && iack_level == 3'd2) hs_lat <= 0;
		if (iack && iack_level == 3'd4) vs_lat <= 0;

		// any access to a mask, read or write, drops what is latched
		if (sel_mask_sys || sel_mask_vme) begin
			hs_lat <= 0;
			vs_lat <= 0;
		end

		if (sel && we) case (addr)
			3'd0: sys_mask <= din;
			3'd2: soft_int <= din[0];
			3'd3: vme_int3 <= din[0];
			3'd4: gpr1     <= din;
			3'd5: gpr2     <= din;
			3'd6: vme_mask <= din;
			default: ;   // $FF8E03, $FF8E0F: read only
		endcase
	end
end

always @(*) begin
	case (addr)
		3'd0: dout = sys_mask;
		3'd1: dout = sys_state;
		3'd2: dout = {7'd0, soft_int};
		3'd3: dout = {7'd0, vme_int3};
		3'd4: dout = gpr1;
		3'd5: dout = gpr2;
		3'd6: dout = vme_mask;
		3'd7: dout = vme_state;
	endcase
end

endmodule
