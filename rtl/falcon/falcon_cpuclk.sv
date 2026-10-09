//============================================================================
//  Falcon CPU clock and time accounting (docs/CPU_TIMING.md, milestone 2)
//
//  cpu_ce is the AP68030's clock enable (USE_CE): the processor advances on
//  the system clocks with cpu_ce.
//
//    turbo   every clock (32 MHz), the bus bridge in its 32-bit mode.
//    Falcon  one processor clock per `period` system clocks on average:
//            2 (16 MHz), or 4 (8 MHz) when $FF8007 bit 0 is clear, as Hatari
//            switches its 68030 (IoMemTabFalcon_BusCtrl_WriteByte; only the
//            length of a clock changes, every clock count stays the same).
//
//  In the Falcon setting the processor runs on Hatari's clock count: every
//  bus cycle takes exactly the processor clocks of Hatari's model
//  (falcon_cpubus).  When its data is not there in time (DDR3 latency, a
//  slow device, the FPU on the ARM) the bridge holds the processor (`hold`)
//  rather than adding wait states, so the processor sees the exact cycle;
//  Falcon time goes on meanwhile.  `debt` counts the system clocks of Falcon
//  time the processor is behind: +1 every system clock, -period for every
//  processor clock.  The processor gets a clock whenever it is at least one
//  processor clock behind, so it runs at the Falcon rate while the debt is
//  small and on every system clock (32 MHz) while it catches up.  Wait states
//  the bridge could not avoid come back as `credit` (processor clocks that
//  did not count).  The debt is capped at DEBT_MAX system clocks; the rest is
//  forgiven (the processor falls behind for good, counted in `forgiven`).
//
//  tpos counts the processor clocks, mod 20: Hatari's 4-clock ST-RAM slot
//  and 10-clock E clock are positions in this count (custom.c:329,
//  m68000.c:808-824).
//============================================================================

module falcon_cpuclk #(parameter DEBT_MAX = 4095)
(
	input             clk,
	input             reset,
	input             turbo,       // the bridge's mode: 1 = every clock
	input             cpu_16mhz,   // $FF8007 bit 0
	input             hold,        // falcon_cpubus: no processor clock now
	input       [3:0] credit,      // processor clocks to give back (one clock pulse)

	output            cpu_ce,
	output reg  [4:0] tpos,        // processor clocks before this clock, mod 20

	// statistics
	output reg [15:0] debt,        // system clocks behind
	output reg [15:0] debt_peak,
	output reg [31:0] forgiven,    // system clocks given up at the cap
	output reg [31:0] held         // system clocks the bridge held the processor
);

wire [2:0] period = cpu_16mhz ? 3'd2 : 3'd4;

assign cpu_ce = turbo || ((debt >= {13'd0, period}) && !hold);

// the debt after this clock, before the cap
wire  [6:0] credit_sys = credit * period;
wire [17:0] debt_nxt = {2'b00, debt} + 18'd1 + {11'd0, credit_sys} - (cpu_ce ? {15'd0, period} : 18'd0);
wire        over     = debt_nxt > DEBT_MAX;
wire [15:0] debt_cap = over ? DEBT_MAX[15:0] : debt_nxt[15:0];
wire [17:0] over_by  = debt_nxt - DEBT_MAX[17:0];

// (the pacing runs through reset: the processor takes its reset on clocks
// with cpu_ce; the clock position restarts, so Hatari's slot and E clock
// phases are the same after every reset, and the statistics are cleared)
initial begin tpos = 5'd0; debt = 16'd0; debt_peak = 16'd0; forgiven = 32'd0; held = 32'd0; end
always @(posedge clk) begin
	if (reset)       tpos <= 5'd0;
	else if (cpu_ce) tpos <= (tpos == 5'd19) ? 5'd0 : tpos + 5'd1;

	if (turbo) debt <= 16'd0;
	else       debt <= debt_cap;

	if (reset) begin
		debt_peak <= 16'd0; forgiven <= 32'd0; held <= 32'd0;
	end else if (!turbo) begin
		if (over) forgiven <= forgiven + {14'd0, over_by};
		if (debt_cap > debt_peak) debt_peak <= debt_cap;
		if (hold && debt >= {13'd0, period}) held <= held + 32'd1;
	end
end

endmodule
