//============================================================================
//  Falcon: CPU timing counters (docs/CPU_TIMING.md, milestone 4)
//
//  A diagnostic register block of this core, with no counterpart on a Falcon
//  or in Hatari: there $FFF000-$FFF01F bus errors (ioMem.c IoMem_Init sets
//  every I/O address to IoMem_BusErrorEvenReadAccess /
//  IoMem_BusErrorOddReadAccess, and ioMemTabFalcon.c maps nothing here), and
//  it still does unless the OSD option "Timing counters" is on (falcon_system
//  decodes the block only then).  A program reads it to compare the core's
//  Falcon time with the simulation's and with real time (tools/cputime).
//
//  Word registers at $FFF000 + offset, read only (writes are taken and
//  ignored):
//    $00       ID: $5453 ("TS")
//    $02       [15:8] version (1), [1] 16 MHz ($FF8007 bit 0),
//              [0] Falcon mode (0: 32 MHz turbo)
//    $04/$06   FTIME     processor clocks of Falcon time since reset: every
//                        processor clock and idle clock, less the clocks
//                        the governor gave back and the wait states the
//                        bridge could not avoid (falcon_cpuclk); Hatari's
//                        cycle count in Falcon mode
//    $08/$0A   RTIME     system clocks (32 MHz) since reset
//    $0C       DEBT      system clocks the processor is behind
//    $0E-$1E   0
//  The clocks forgiven at the debt cap (the processor behind for good)
//  follow: over any stretch in one mode, RTIME = period * FTIME + DEBT +
//  forgiven (differences; period 2 at 16 MHz, 4 at 8 MHz, 1 in turbo, where
//  DEBT stays 0).
//  The 32-bit counters are high word first.  Reading a high word latches
//  the low word of the same counter, and the next read of that low word
//  returns the latch (once), so a long read is consistent.  The counters
//  start at 0 at reset and wrap.
//============================================================================

module falcon_tstat
(
	input             clk,
	input             reset,

	// register bus (docs/ARCHITECTURE.md)
	input             bus_cs,
	input             bus_stb,
	input             bus_we,
	input       [4:1] bus_addr,
	output reg [15:0] bus_dout,
	output            bus_ack,

	// the CPU clock (falcon_cpuclk, falcon_cpubus)
	input             fmode,       // Falcon mode (not turbo)
	input             cpu_16mhz,
	input             cpu_ce,
	input             idle_tick,
	input       [7:0] back,
	input       [3:0] credit,
	input      [15:0] debt
);

reg [31:0] ftime, rtime;
always @(posedge clk) begin
	if (reset) begin
		ftime <= 32'd0;
		rtime <= 32'd0;
	end else begin
		ftime <= ftime + {31'd0, cpu_ce | idle_tick} - {24'd0, back} - {28'd0, credit};
		rtime <= rtime + 32'd1;
	end
end

// the 32-bit counter at a register pair ($04: FTIME, $08: RTIME)
wire [31:0] cnt    = bus_addr[3] ? rtime : ftime;
wire        is_cnt = (bus_addr[4:3] == 2'b00 && bus_addr[2]) || (bus_addr[4:2] == 3'b010);

reg [15:0] lat;                // the low word latched by a high word read
reg        lat_v;              // ... valid, of the counter lat_f ...
reg        lat_f;              // ... (bus_addr[3])
always @(posedge clk) begin
	if (reset) lat_v <= 1'b0;
	else if (bus_stb && !bus_we && is_cnt) begin
		if (!bus_addr[1]) begin
			lat <= cnt[15:0];
			lat_v <= 1'b1;
			lat_f <= bus_addr[3];
		end else if (lat_f == bus_addr[3])
			lat_v <= 1'b0;   // the latch is used once
	end
end

always @* begin
	case (bus_addr)
		4'h0:       bus_dout = 16'h5453;
		4'h1:       bus_dout = {8'd1, 6'd0, cpu_16mhz, fmode};
		4'h2, 4'h4: bus_dout = cnt[31:16];
		4'h3, 4'h5: bus_dout = (lat_v && lat_f == bus_addr[3]) ? lat : cnt[15:0];
		4'h6:       bus_dout = debt;
		default:    bus_dout = 16'h0000;
	endcase
end

assign bus_ack = bus_stb;

/* verilator lint_off UNUSEDSIGNAL */
wire unused_tstat = bus_cs;
/* verilator lint_on UNUSEDSIGNAL */

endmodule
