//============================================================================
//  falcon_combel bench: Jaguar pad ports, paddles, DIP switches, $FF8006/7
//
//  Expected values follow Hatari joy.c (Joy_StePadButtons_DIPSwitches_ReadWord,
//  Joy_StePadMulti_ReadWord/WriteWord, Joy_GetStickAnalogData,
//  Joy_KeyToButton) and stMemory.c ($FF8006).  The $FF9202 values for
//  "up+A", "key 1", "key 3", "key 0" were also observed in a Hatari Falcon
//  run (FEFF, F7FF, F7FF, FEFF).  MiSTer joystick bits as the OSD declares
//  them: 0 R 1 L 2 D 3 U, 4 Fire/A 5 B 6 C 7 Pause 8 Option,
//  9..20 keypad 1 2 3 4 5 6 7 8 9 * 0 #.
//============================================================================
`timescale 1ns/1ps
module tb_combel;

reg clk = 0;
always #15.625 clk = ~clk;
reg reset = 1, cold_reset = 1;
reg bus_cs = 0, bus_stb = 0, bus_we = 0, bus_uds = 0, bus_lds = 0;
reg [23:1] bus_addr = 0;
reg [15:0] bus_din = 0;
wire [15:0] bus_dout;
wire bus_ack;
reg [31:0] joy0 = 0, joy1 = 0;
reg [15:0] ana0 = 0, ana1 = 0;
reg [3:0] ram_mb = 4'd14;
reg [1:0] monitor = 2'b10;

falcon_combel dut (
	.clk(clk), .reset(reset), .cold_reset(cold_reset), .ram_mb(ram_mb), .monitor(monitor),
	.bus_cs(bus_cs), .bus_stb(bus_stb), .bus_we(bus_we), .bus_addr(bus_addr), .bus_uds(bus_uds),
	.bus_lds(bus_lds), .bus_din(bus_din), .bus_dout(bus_dout), .bus_ack(bus_ack),
	.joy0(joy0), .joy1(joy1), .ana0(ana0), .ana1(ana1),
	.falcon_bus(), .cpu_16mhz()
);

integer errors = 0, checks = 0;
reg [15:0] rd;

task access(input [23:0] a, input we, input u, input l, input [15:0] d);
	begin
		@(posedge clk);
		bus_addr <= a[23:1]; bus_we <= we; bus_uds <= u; bus_lds <= l; bus_din <= d;
		bus_cs <= 1; bus_stb <= 1;
		@(posedge clk);
		bus_stb <= 0;
		while (!bus_ack) @(posedge clk);
		rd = bus_dout;
		bus_cs <= 0;
		@(posedge clk);
	end
endtask

task check(input [23:0] a, input u, input l, input [15:0] mask, input [15:0] exp, input [8*48-1:0] what);
	begin
		access(a, 0, u, l, 16'h0000);
		checks = checks + 1;
		if ((rd & mask) !== (exp & mask)) begin
			errors = errors + 1;
			$display("FAIL %0s: read %06x = %04x, expected %04x (mask %04x)", what, a, rd, exp, mask);
		end
	end
endtask

task sel(input [7:0] s);      // word write to $FF9202: the select byte is the low byte
	access(24'hFF9202, 1, 1, 1, {8'h00, s});
endtask

initial begin
	repeat (4) @(posedge clk);
	cold_reset = 0; reset = 0;

	// $FF8006: 14 MB, VGA -> $A6 (Hatari nFalcSysCntrl $26 | FALCON_MONITOR_VGA)
	check(24'hFF8006, 1, 0, 16'hFF00, 16'hA600, "FF8006 14MB VGA");

	// no row selected after reset
	check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFFFF, "9202 no row selected");
	check(24'hFF9200, 1, 1, 16'hFFFF, 16'hBFFF, "9200 DIP $BF, no buttons");

	// row 0: directions in the EVEN byte (joy.c nData << 8)
	sel(8'hEE);
	joy0 = 32'h0000_0018;                                   // up + A
	check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFEFF, "row0 pad A up (Hatari FEFF)");
	check(24'hFF9202, 1, 0, 16'hFF00, 16'hFE00, "row0 byte read $FF9202");
	check(24'hFF9200, 1, 1, 16'hFFFF, 16'hBFFD, "row0 fire A (Hatari BFFD)");
	joy0 = 32'h0000_0080;                                   // Pause
	check(24'hFF9200, 1, 1, 16'hFFFF, 16'hBFFE, "row0 Pause bit 0");
	joy0 = 0; joy1 = 32'h0000_0001;                         // pad B right
	check(24'hFF9202, 1, 1, 16'hFFFF, 16'h7FFF, "row0 pad B right");
	joy1 = 32'h0000_0006;                                   // pad B left + down
	check(24'hFF9202, 1, 1, 16'hFFFF, 16'hBFFF & 16'hDFFF, "row0 pad B left+down");
	joy1 = 32'h0000_0010;                                   // pad B fire A
	check(24'hFF9200, 1, 1, 16'hFFFF, 16'hBFF7, "row0 pad B fire A (bit 3)");
	joy1 = 0;

	// row 1: *, 7, 4, 1 in bits 0..3
	sel(8'hDD);
	joy0 = 32'h1 << 9;  check(24'hFF9202, 1, 1, 16'hFFFF, 16'hF7FF, "row1 key 1 (Hatari F7FF)");
	joy0 = 32'h1 << 12; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFBFF, "row1 key 4");
	joy0 = 32'h1 << 15; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFDFF, "row1 key 7");
	joy0 = 32'h1 << 18; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFEFF, "row1 key *");
	joy0 = 32'h0000_0020; check(24'hFF9200, 1, 1, 16'hFFFF, 16'hBFFD, "row1 fire B");
	joy0 = 32'h0000_0008; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFFFF, "row1 ignores directions");

	// row 2: 0, 8, 5, 2
	sel(8'hBB);
	joy0 = 32'h1 << 19; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFEFF, "row2 key 0 (Hatari FEFF)");
	joy0 = 32'h1 << 16; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFDFF, "row2 key 8");
	joy0 = 32'h1 << 13; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFBFF, "row2 key 5");
	joy0 = 32'h1 << 10; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hF7FF, "row2 key 2");
	joy0 = 32'h0000_0040; check(24'hFF9200, 1, 1, 16'hFFFF, 16'hBFFD, "row2 fire C");

	// row 3: #, 9, 6, 3
	sel(8'h77);
	joy0 = 32'h1 << 11; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hF7FF, "row3 key 3 (Hatari F7FF)");
	joy0 = 32'h1 << 14; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFBFF, "row3 key 6");
	joy0 = 32'h1 << 17; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFDFF, "row3 key 9");
	joy0 = 32'h1 << 20; check(24'hFF9202, 1, 1, 16'hFFFF, 16'hFEFF, "row3 key #");
	joy0 = 32'h0000_0100; check(24'hFF9200, 1, 1, 16'hFFFF, 16'hBFFD, "row3 fire Option");
	joy0 = 0;

	// pad B keypad (upper nibble), row 1 key 1
	sel(8'hDD);
	joy1 = 32'h1 << 9;  check(24'hFF9202, 1, 1, 16'hFFFF, 16'h7FFF, "pad B row1 key 1");
	joy1 = 0;

	// a byte write to $FF9202 alone does not change the select (low byte)
	access(24'hFF9202, 1, 1, 0, 16'hEE00);
	joy0 = 32'h1 << 9;  check(24'hFF9202, 1, 1, 16'hFFFF, 16'hF7FF, "byte write to 9202 keeps row 1");
	joy0 = 0;

	// paddles: $04 + (axis + 128) / 4 (Joy_GetStickAnalogData)
	ana0 = 16'h0000; check(24'hFF9210, 0, 1, 16'h00FF, 16'h0024, "paddle A X centre $24");
	ana0 = 16'h0080; check(24'hFF9210, 0, 1, 16'h00FF, 16'h0004, "paddle A X min $04");
	ana0 = 16'h007F; check(24'hFF9210, 0, 1, 16'h00FF, 16'h0043, "paddle A X max $43");
	ana0 = 16'h8000; check(24'hFF9212, 0, 1, 16'h00FF, 16'h0004, "paddle A Y min $04");
	ana0 = 16'h7F00; check(24'hFF9212, 0, 1, 16'h00FF, 16'h0043, "paddle A Y max $43");
	ana1 = 16'h0040; check(24'hFF9214, 0, 1, 16'h00FF, 16'h0034, "paddle B X +64 -> $34");
	ana1 = 16'hC000; check(24'hFF9216, 0, 1, 16'h00FF, 16'h0014, "paddle B Y -64 -> $14");
	check(24'hFF9210, 1, 1, 16'hFF00, 16'hFF00, "paddle even byte $FF");

	if (errors == 0) $display("COMBEL: %0d checks, 0 failures -- PASS", checks);
	else             $display("COMBEL: %0d checks, %0d FAILURES -- FAIL", checks, errors);
	$finish;
end
endmodule
