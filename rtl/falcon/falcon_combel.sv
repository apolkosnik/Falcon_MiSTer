//============================================================================
//  Falcon COMBEL glue registers
//
//  $FF8001 memory configuration (read/write byte, STMemory_MMU_Config_*)
//  $FF8006 monitor type and memory configuration (read only, the values of
//          Hatari's STMemory_SetDefaultConfig: 14 MB = $26, 4 MB = $16,
//          monitor in bits 7..6: 00 mono, 01 RGB, 10 VGA, 11 TV)
//  $FF8007 bus control (IoMemTabFalcon_BusCtrl_*): bit 6 warm start,
//          bit 5 Falcon bus mode (1 after reset), bit 3/2 blitter,
//          bit 0 CPU 16 MHz.  Bit 6 survives every reset but the cold one.
//  $FF9200 joypad fire buttons, DIP switches in the upper byte ($BF,
//          IoMemTabFalcon_DIPSwitches_Read)
//  $FF9202 joypad directions (read) / row select (write)
//  $FF9210-$FF9217 paddle positions, $FF9220/$FF9222 light pen
//
//  Joypad A is MiSTer joystick 0, pad B joystick 1 (Joy_StePad* in
//  joy.c), MiSTer joystick bits as declared by the OSD "J" line:
//    0 right 1 left 2 down 3 up  4 Fire/A  5 B  6 C  7 Pause  8 Option
//    9..20 keypad 1 2 3 4 5 6 7 8 9 * 0 #
//  $FF9202 reads the selected row in its EVEN byte (joy.c: nData << 8),
//  pad A in bits 11..8, pad B in bits 15..12, active low:
//    row 0  up, down, left, right       (Joy_GetStickData)
//    row 1  *, 7, 4, 1                  (fire buttons >> 13)
//    row 2  0, 8, 5, 2                  (fire buttons >> 9)
//    row 3  #, 9, 6, 3                  (fire buttons >> 5)
//  Paddles $FF9211/13 (pad A X/Y) and $FF9215/17 (pad B) follow the MiSTer
//  left analog sticks with Hatari's scale: $04 + (axis + 128) / 4, $24 at
//  rest (Joy_GetStickAnalogData).
//============================================================================

module falcon_combel
(
	input             clk,
	input             reset,
	input             cold_reset,

	input       [3:0] ram_mb,        // 4 or 14
	input       [1:0] monitor,       // 00 mono, 01 RGB, 10 VGA, 11 TV

	input             bus_cs,
	input             bus_stb,
	input             bus_we,
	input      [23:1] bus_addr,
	input             bus_uds,
	input             bus_lds,
	input      [15:0] bus_din,
	output reg [15:0] bus_dout,
	output            bus_ack,

	input      [31:0] joy0,          // MiSTer: 0 right 1 left 2 down 3 up 4 A 5 B 6 C 7 Pause 8 Option
	input      [31:0] joy1,
	input      [15:0] ana0,          // MiSTer joystick_l_analog_0: [7:0] X, [15:8] Y, signed
	input      [15:0] ana1,

	output            falcon_bus,    // $FF8007 bit 5
	output            cpu_16mhz      // $FF8007 bit 0
);

assign bus_ack = bus_stb;

reg [7:0] memcfg;
reg [7:0] busctl;
reg [7:0] padsel = 8'hFF;
reg [7:0] lpx_h, lpx_l, lpy_h, lpy_l;

assign falcon_bus = busctl[5];
assign cpu_16mhz  = busctl[0];

wire [7:0] f8006 = {monitor, (ram_mb == 4'd14) ? 6'h26 : 6'h16};

// Hatari stick bits: 0 up, 1 down, 2 left, 3 right
wire [3:0] sticka = {joy0[0], joy0[1], joy0[2], joy0[3]};
wire [3:0] stickb = {joy1[0], joy1[1], joy1[2], joy1[3]};

reg [7:0] btn;
reg [7:0] dir;
always @* begin
	btn = 8'hFF;
	if (!padsel[0])      begin if (joy0[4]) btn[1] = 0; if (joy0[7]) btn[0] = 0; end
	else if (!padsel[1]) begin if (joy0[5]) btn[1] = 0; end
	else if (!padsel[2]) begin if (joy0[6]) btn[1] = 0; end
	else if (!padsel[3]) begin if (joy0[8]) btn[1] = 0; end
	if (!padsel[4])      begin if (joy1[4]) btn[3] = 0; if (joy1[7]) btn[2] = 0; end
	else if (!padsel[5]) begin if (joy1[5]) btn[3] = 0; end
	else if (!padsel[6]) begin if (joy1[6]) btn[3] = 0; end
	else if (!padsel[7]) begin if (joy1[8]) btn[3] = 0; end

	dir = 8'hFF;
	if (padsel[3:0] != 4'hF) begin
		if      (!padsel[0]) dir[3:0] = ~sticka;
		else if (!padsel[1]) dir[3:0] = ~{joy0[9],  joy0[12], joy0[15], joy0[18]};   // 1 4 7 *
		else if (!padsel[2]) dir[3:0] = ~{joy0[10], joy0[13], joy0[16], joy0[19]};   // 2 5 8 0
		else if (!padsel[3]) dir[3:0] = ~{joy0[11], joy0[14], joy0[17], joy0[20]};   // 3 6 9 #
	end
	if (padsel[7:4] != 4'hF) begin
		if      (!padsel[4]) dir[7:4] = ~stickb;
		else if (!padsel[5]) dir[7:4] = ~{joy1[9],  joy1[12], joy1[15], joy1[18]};
		else if (!padsel[6]) dir[7:4] = ~{joy1[10], joy1[13], joy1[16], joy1[19]};
		else if (!padsel[7]) dir[7:4] = ~{joy1[11], joy1[14], joy1[17], joy1[20]};
	end
end

// Hatari STE_JOY_ANALOG_MIN_VALUE + (axis + 128) / 4
function [7:0] paddle;
	input [7:0] axis;
	paddle = 8'h04 + ({~axis[7], axis[6:0]} >> 2);    // (signed axis + 128) / 4
endfunction

wire hi = (bus_addr[23:16] == 8'hFF) && (bus_addr[15:12] == 4'h9);   // $FF9xxx
wire [7:0] off = {bus_addr[7:1], 1'b0};

always @* begin
	bus_dout = 16'hFFFF;
	if (!hi) begin
		case (off)
			8'h00: bus_dout = {8'hFF, memcfg};
			8'h06: bus_dout = {f8006, busctl};
			default: bus_dout = 16'hFFFF;
		endcase
	end
	else begin
		case (off)
			8'h00: bus_dout = {8'hBF, btn};
			8'h02: bus_dout = {dir, 8'hFF};                    // even byte, joy.c: nData << 8
			8'h10: bus_dout = {8'hFF, paddle(ana0[7:0])};
			8'h12: bus_dout = {8'hFF, paddle(ana0[15:8])};
			8'h14: bus_dout = {8'hFF, paddle(ana1[7:0])};
			8'h16: bus_dout = {8'hFF, paddle(ana1[15:8])};
			default: bus_dout = 16'hFFFF;
		endcase
	end
end

always @(posedge clk) begin
	if (cold_reset) begin
		busctl <= 8'h25;          // cold start, Falcon bus, 16 MHz CPU, 16 MHz blitter
		memcfg <= 8'h0A;
	end
	if (reset) begin
		padsel <= 8'hFF;
		busctl[5] <= 1'b1;        // IoMem_Reset: back to the Falcon-only bus
	end
	else if (bus_stb && bus_we) begin
		if (!hi) begin
			if (off == 8'h00 && bus_lds) memcfg <= bus_din[7:0];
			if (off == 8'h06 && bus_lds) busctl <= bus_din[7:0];
		end
		else begin
			if (off == 8'h02 && bus_lds) padsel <= bus_din[7:0];
			if (off == 8'h20) begin if (bus_uds) lpx_h <= bus_din[15:8]; if (bus_lds) lpx_l <= bus_din[7:0]; end
			if (off == 8'h22) begin if (bus_uds) lpy_h <= bus_din[15:8]; if (bus_lds) lpy_l <= bus_din[7:0]; end
		end
	end
end

endmodule
