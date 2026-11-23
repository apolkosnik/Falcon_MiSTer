//============================================================================
//  TT030 timing: idle clocks per instruction.
//
//  The AP68030 runs most instructions in fewer clocks than a real 68030 (a
//  register move in 1 where the chip takes 2, a shift in 1 against 4 to 6,
//  MULU.W in 4 against 17).  With `enable`, every dispatched instruction adds
//  the difference for its operation word (tt030_pace.mem, made by
//  tools/tt030_pace_gen.py from measurements on a real TT and on this core) to
//  a count of idle clocks owed, and the processor's clock enable is withheld
//  for one system clock per idle clock, only on clocks with no bus cycle in
//  progress, so that the bus bridge never sees a stretched cycle.
//
//  Only what the core does too fast can be corrected this way: memory the
//  caches do not hold, and the FPU, are slower than a TT's and stay so.
//============================================================================

module tt030_pace
(
	input             clk,
	input             reset,
	input             enable,
	input             cpu_ce,      // the processor had a clock this system clock
	input             dispatch,    // it dispatched an instruction (with cpu_ce)
	input      [15:0] ir,          // whose operation word this is on the next processor clock
	input             bus_act,     // a bus cycle is in progress
	output            pause        // no processor clock now
);

(* ramstyle = "M10K" *) reg [6:0] extra [0:65535];
initial $readmemh("rtl/falcon/tt030_pace.mem", extra);

reg        disp_d;                 // `q` is the entry of the instruction dispatched
reg  [6:0] q;
reg  [9:0] owed;

assign pause = enable && (owed != 10'd0) && !bus_act;

wire [10:0] owed_n = {1'b0, owed} + (disp_d ? {4'd0, q} : 11'd0) - (pause ? 11'd1 : 11'd0);

always @(posedge clk) begin
	q <= extra[ir];
	if (reset || !enable) begin
		disp_d <= 1'b0; owed <= 10'd0;
	end else begin
		// `ir` takes the operation word on the processor clock of the dispatch and
		// `dispatch` shows from then to the next processor clock: on that clock the
		// table is read with `ir` (the next dispatch may replace it right after)
		disp_d <= cpu_ce && dispatch;
		owed   <= owed_n[10] ? 10'h3FF : owed_n[9:0];
	end
end

endmodule
