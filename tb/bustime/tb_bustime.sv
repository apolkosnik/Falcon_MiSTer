//============================================================================
//  Falcon bus timing bench (docs/CPU_TIMING.md, milestone 2)
//
//  The real AP68030 (USE_CE), falcon_cpuclk, falcon_cpubus in its Falcon
//  mode and falcon_memarb on the DDR3 model (tb/system/ddr3_model.sv,
//  pseudo-random latency and BUSY), with a device bus model, an interrupt
//  controller model, a DMA master on the arbiter's d2 port and optional
//  video fetches.  The CPU runs t_bustime.s from "ROM" at $E00000.
//
//  The checker measures every bus cycle in processor clocks (falcon_cpuclk's
//  cpu_ce edges from S0 to the end of S5) and compares it with Hatari's
//  model, written down here independently of falcon_cpubus from the rules
//  in docs/CPU_TIMING.md ("Hatari's Falcon CPU timing in detail"):
//    CHIP16 (ST-RAM, IDE, unmapped): 3, plus 2 if S0 is at position 2 or 3
//    mod 4; FAST16 (ROM, cartridge, $FFxxxx): 3 plus the device's waits;
//    interrupt acknowledge: MFP 12, autovector the E clock + 10, others 3;
//    other CPU space 3.
//  A cycle may be longer only by the wait states the bridge reports as
//  unavoidable (credit).  The run also reports the processor clocks from
//  the first bus cycle to the end, less those wait states: the bridge holds
//  the processor instead of adding wait states, so this count must not
//  depend on memory latency, video load or the clock rate (run.sh compares
//  runs).
//
//  Plusargs: +rom=<hex> (the program), +mhz8 (8 MHz), +vidload (video
//  fetches), +seed=<n> (device and IACK delays), +maxclk=<n>, +trace.
//
//  Test device at $FFFF00: see t_bustime.s.  A word written to $3F0 is a
//  timing marker (timing/): "MARK <value> <processor clock of its S0>".
//============================================================================

module tb_bustime;

reg clk = 0;
always #10 clk = ~clk;
reg reset = 1;

integer sysclk = 0;
always @(posedge clk) sysclk <= sysclk + 1;

reg mhz8, vidload, trace;
integer seed, maxclk;
initial begin
	mhz8    = $test$plusargs("mhz8");
	vidload = $test$plusargs("vidload");
	trace   = $test$plusargs("trace");
	if (!$value$plusargs("seed=%d", seed)) seed = 1;
	if (!$value$plusargs("maxclk=%d", maxclk)) maxclk = 2000000;
	void'($urandom(seed));
end

//----------------------------------------------------------------- CPU
wire [31:0] cpu_a, cpu_do;
wire  [2:0] cpu_fc;
wire  [1:0] cpu_siz;
wire        cpu_rw, cpu_rmc_n, cpu_as_n, cpu_ds_n, cpu_dben_n, cpu_ecs_n, cpu_ocs_n;
wire        cpu_ciout_n, cpu_cbreq_n, cpu_bus_oe, cpu_d_oe, cpu_bg_n, cpu_ipend_n;
wire        cpu_reset_oe, cpu_refill_n, cpu_status_n, cpu_halted, cpu_inst;
wire [31:0] cpu_di, dbg_pc;
wire        dsack0_n, dsack1_n, berr_n, avec_n, ciin_n;
reg   [2:0] irq_level = 3'd0;
wire        snoop_we;
wire [23:0] snoop_addr;
wire        cpu_ce, cpu_hold, cpu_fmode, cpu_idle, cpu_idle_tick;
wire  [1:0] cpu_tm_pop, cpu_tm_md;
wire [31:0] gov_idle, gov_back, idled;
wire  [7:0] cpu_back;
wire [95:0] cpu_tm_q;
wire  [2:0] cpu_tm_qn;
wire [31:0] cpu_tm_scan, cpu_fetch_stop;
wire        cpu_tm_flush, cpu_fetch_stop_v, cpu_scan_v;
wire [31:0] cpu_scan_to;
falcon_pipescan #(.OPTBL_MEM("../../rtl/falcon/falcon_optbl.mem")) pipescan
(
	.clk(clk), .ce(cpu_ce), .enable(cpu_fmode), .q(cpu_tm_q), .qn(cpu_tm_qn), .scan(cpu_tm_scan),
	.flush(cpu_tm_flush), .stop_v(cpu_fetch_stop_v), .stop_at(cpu_fetch_stop),
	.scan_v(cpu_scan_v), .scan_to(cpu_scan_to)
);
wire  [4:0] cpu_tpos;
wire  [3:0] cpu_credit;
wire [15:0] debt, debt_peak;
wire [31:0] forgiven, held;

falcon_cpuclk cpuclk
(
	.clk(clk), .reset(reset), .turbo(!cpu_fmode), .cpu_16mhz(!mhz8),
	.hold(cpu_hold), .credit(cpu_credit), .idle(cpu_idle), .back(cpu_back), .cpu_ce(cpu_ce), .idle_tick(cpu_idle_tick),
	.tpos(cpu_tpos), .debt(debt), .debt_peak(debt_peak), .forgiven(forgiven), .held(held), .idled(idled)
);

ap030_top #(.USE_CE(1)) cpu
(
	.clk(clk), .ce(cpu_ce),
	.fast_req(), .fast_addr(), .fast_fc(), .fast_rw(), .fast_ci(), .fast_burst(), .fast_be(), .fast_wdata(),
	.fast_match(1'b0), .fast_ready(1'b0), .fast_valid(1'b0), .fast_last(1'b0),
	.fast_word(2'b00), .fast_rdata(32'd0),
	.a(cpu_a), .fc(cpu_fc), .siz(cpu_siz), .rw(cpu_rw), .rmc_n(cpu_rmc_n),
	.as_n(cpu_as_n), .ds_n(cpu_ds_n), .dben_n(cpu_dben_n), .ecs_n(cpu_ecs_n), .ocs_n(cpu_ocs_n),
	.ciout_n(cpu_ciout_n), .cbreq_n(cpu_cbreq_n), .bus_oe(cpu_bus_oe),
	.d_o(cpu_do), .d_oe(cpu_d_oe), .d_i(cpu_di),
	.dsack0_n(dsack0_n), .dsack1_n(dsack1_n), .sterm_n(1'b1), .berr_n(berr_n), .halt_n(1'b1),
	.avec_n(avec_n), .ciin_n(ciin_n), .cback_n(1'b1),
	.br_n(1'b1), .bg_n(cpu_bg_n), .bgack_n(1'b1),
	.ipl_n(~irq_level), .ipend_n(cpu_ipend_n),
	.reset_n_i(~reset), .reset_n_oe(cpu_reset_oe),
	.cdis_n(1'b1), .mmudis_n(1'b1), .refill_n(cpu_refill_n), .status_n(cpu_status_n),
	.dbg_pc(dbg_pc), .dbg_sr(), .dbg_state(), .dbg_inst(cpu_inst), .tm_pop(cpu_tm_pop), .tm_md(cpu_tm_md), .dbg_halted(cpu_halted),
	.tm_q(cpu_tm_q), .tm_qn(cpu_tm_qn), .tm_scan(cpu_tm_scan), .tm_flush(cpu_tm_flush),
	.fetch_stop_v(cpu_fetch_stop_v), .fetch_stop(cpu_fetch_stop),
	.fetch_scan_v(cpu_scan_v), .fetch_scan_to(cpu_scan_to),
	.dbg_vbr(), .dbg_cacr(), .dbg_cache_clear(),
	.snoop_we(snoop_we), .snoop_addr({8'd0, snoop_addr}), .nmi_vec_nocache(1'b0), .fetch_lazy(cpu_fmode)
);

//----------------------------------------------------------------- bridge
wire        cram_req, cram_we, cram_ack;
wire [23:2] cram_addr;
wire  [3:0] cram_be;
wire [31:0] cram_wdata, cram_rdata;
wire [63:0] cram_rdata64;
wire        dev_cs, dev_stb, dev_we, dev_uds, dev_lds, dev_super;
wire [23:1] dev_addr;
wire [15:0] dev_din;
reg  [15:0] dev_dout = 16'd0;
reg         dev_ack = 1'b0, dev_berr = 1'b0;
wire        iack_req;
wire  [2:0] iack_level;
reg         iack_done = 1'b0, iack_avec = 1'b0, iack_spur = 1'b0, iack_mfp = 1'b0;
reg   [7:0] iack_vector = 8'd0;
wire        cpu_cycle_done;

falcon_cpubus cpubus
(
	.clk(clk), .reset(reset),
	.ram_mb(4'd4), .ram_tos(1'b0),
	.a(cpu_a), .fc(cpu_fc), .siz(cpu_siz), .rw(cpu_rw), .as_n(cpu_as_n), .ds_n(cpu_ds_n),
	.bus_oe(cpu_bus_oe), .d_o(cpu_do), .d_i(cpu_di),
	.dsack0_n(dsack0_n), .dsack1_n(dsack1_n), .berr_n(berr_n), .avec_n(avec_n), .ciin_n(ciin_n),
	.ram_req(cram_req), .ram_we(cram_we), .ram_addr(cram_addr), .ram_be(cram_be),
	.ram_wdata(cram_wdata), .ram_rdata(cram_rdata), .ram_ack(cram_ack),
	.dev_cs(dev_cs), .dev_stb(dev_stb), .dev_we(dev_we), .dev_addr(dev_addr),
	.dev_uds(dev_uds), .dev_lds(dev_lds), .dev_din(dev_din), .dev_dout(dev_dout),
	.dev_ack(dev_ack), .dev_berr(dev_berr), .dev_super(dev_super),
	.iack_req(iack_req), .iack_level(iack_level), .iack_done(iack_done), .iack_avec(iack_avec),
	.iack_spur(iack_spur), .iack_vector(iack_vector),
	.cp_req(), .cp_we(), .cp_id(), .cp_off(), .cp_siz(), .cp_wdata(),
	.cp_ack(1'b0), .cp_berr(1'b0), .cp_rdata(32'd0),
	.cycle_done(cpu_cycle_done),
	.fmode_in(1'b1), .fmode(cpu_fmode), .cpu_ce(cpu_ce), .tpos(cpu_tpos),
	.inst(cpu_inst & cpu_ce), .hold(cpu_hold), .credit(cpu_credit), .wbuf_busy(),
	.ram_rdata64(cram_rdata64), .snoop_we(snoop_we), .snoop_addr(snoop_addr),
	.buf_flush(1'b0), .iack_mfp(iack_mfp),
	.dispatch(cpu_inst), .tm_pop(cpu_tm_pop), .tm_md(cpu_tm_md),
	.idle(cpu_idle), .idle_tick(cpu_idle_tick), .back(cpu_back), .bus_lost(1'b0), .blit_acc(1'b0),
	.gov_idle(gov_idle), .gov_back(gov_back)
);

//----------------------------------------------------------------- memory
reg         d2_req = 1'b0;
reg  [23:1] d2_addr;
reg  [15:0] d2_wdata;
wire        d2_ack;
reg         vid_req = 1'b0;
reg  [23:3] vid_addr = 21'h100000 >> 3;
wire        vid_ack;
wire        DDRAM_BUSY, DDRAM_DOUT_READY, DDRAM_RD, DDRAM_WE;
wire  [7:0] DDRAM_BURSTCNT, DDRAM_BE;
wire [28:0] DDRAM_ADDR;
wire [63:0] DDRAM_DOUT, DDRAM_DIN;

falcon_memarb memarb
(
	.clk(clk), .reset(reset), .ram_mb(4'd4),
	.vid_req(vid_req), .vid_addr(vid_addr), .vid_ack(vid_ack), .vid_data(), .vid_valid(),
	.ld_wr(1'b0), .ld_addr(24'd0), .ld_data(8'd0), .ld_busy(),
	.d0_req(1'b0), .d0_we(1'b0), .d0_addr(23'd0), .d0_be(2'b00), .d0_wdata(16'd0), .d0_rdata(), .d0_ack(),
	.d1_req(1'b0), .d1_we(1'b0), .d1_addr(23'd0), .d1_be(2'b00), .d1_wdata(16'd0), .d1_rdata(), .d1_ack(),
	.d2_req(d2_req), .d2_we(1'b1), .d2_addr(d2_addr), .d2_be(2'b11), .d2_wdata(d2_wdata), .d2_rdata(), .d2_ack(d2_ack),
	.d3_req(1'b0), .d3_we(1'b0), .d3_addr(23'd0), .d3_be(2'b00), .d3_wdata(16'd0), .d3_rdata(), .d3_ack(),
	.cpu_req(cram_req), .cpu_we(cram_we), .cpu_addr(cram_addr), .cpu_be(cram_be),
	.cpu_wdata(cram_wdata), .cpu_rdata(cram_rdata), .cpu_rdata64(cram_rdata64), .cpu_ack(cram_ack),
	.snoop_we(snoop_we), .snoop_addr(snoop_addr),
	.DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE)
);

ddr3_model ddr
(
	.clk(clk), .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
	.DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
	.DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE)
);

// video fetches (+vidload): a 4-word burst every 40-100 clocks
integer vid_gap = 50;
always @(posedge clk) begin
	if (vid_ack) vid_req <= 1'b0;
	if (vidload && !vid_req) begin
		if (vid_gap == 0) begin
			vid_req  <= 1'b1;
			vid_addr <= vid_addr + 21'd4;
			vid_gap  <= 40 + ($urandom % 60);
		end else vid_gap <= vid_gap - 1;
	end
end

//----------------------------------------------------------------- devices
// every I/O address stores what is written and reads it back, except
// $FF8E00-$FF8EFF (no device: bus error) and the test device at $FFFF00;
// answers come after 0-5 clocks (+seed)
reg [15:0] regs [bit [22:0]];
reg        dv_pend = 1'b0;
reg  [2:0] dv_dly;
reg [23:1] dv_addr;
reg        dv_we, dv_uds, dv_lds;
reg [15:0] dv_din;
reg        done = 1'b0, failed = 1'b0;
reg [15:0] fail_num = 16'd0;
reg [31:0] dma_addr = 32'd0;
reg        dma_wait = 1'b0;     // a DMA write in progress: the device write ends with it
reg  [1:0] irq_kind = 2'd0;
reg  [7:0] irq_vec = 8'd0;
wire [23:0] dva = {dv_addr, 1'b0};

always @(posedge clk) begin
	dev_ack <= 1'b0; dev_berr <= 1'b0;
	if (d2_ack) begin d2_req <= 1'b0; dma_wait <= 1'b0; dev_ack <= 1'b1; end
	if (dev_stb) begin
		dv_pend <= 1'b1; dv_dly <= $urandom % 6;
		dv_addr <= dev_addr; dv_we <= dev_we; dv_uds <= dev_uds; dv_lds <= dev_lds; dv_din <= dev_din;
	end else if (dv_pend) begin
		if (dv_dly != 0) dv_dly <= dv_dly - 3'd1;
		else begin
			dv_pend <= 1'b0;
			if (dva[23:8] == 16'hFF8E) dev_berr <= 1'b1;
			else if (dva[23:5] == 19'h7FFF8) begin
				// the test device
				if (dv_we) case (dva[4:0])
					5'h00: begin
						if (dv_din == 16'h600D) done <= 1'b1;
						else begin failed <= 1'b1; done <= 1'b1; end
					end
					5'h02: $write("%c", dv_din[7:0]);
					5'h04: begin irq_level <= dv_din[2:0]; irq_kind <= dv_din[5:4]; irq_vec <= dv_din[15:8]; end
					5'h06: irq_level <= 3'd0;
					5'h08: dma_addr[31:16] <= dv_din;
					5'h0A: dma_addr[15:0] <= dv_din;
					5'h0C: begin d2_req <= 1'b1; d2_addr <= dma_addr[23:1]; d2_wdata <= dv_din; dma_wait <= 1'b1; end
					5'h10: fail_num <= dv_din;
					default: ;
				endcase
				else dev_dout <= 16'h0000;             // +14: DMA busy (never: the write waits)
				if (!(dv_we && dva[4:0] == 5'h0C)) dev_ack <= 1'b1;
			end
			else begin
				if (dv_we) begin
					reg [15:0] v;
					v = regs.exists(dv_addr) ? regs[dv_addr] : 16'h0000;
					if (dv_uds) v[15:8] = dv_din[15:8];
					if (dv_lds) v[7:0] = dv_din[7:0];
					regs[dv_addr] = v;
				end
				else dev_dout <= regs.exists(dv_addr) ? regs[dv_addr] : {dva[7:0], ~dva[7:0]};
				dev_ack <= 1'b1;
			end
		end
	end
end

// interrupt controller: the acknowledge answers after 1-8 clocks
reg        ia_pend = 1'b0;
reg  [3:0] ia_dly;
reg  [2:0] ia_lvl;
reg  [1:0] ia_kind_used = 2'd3;  // what the last acknowledge was: 0 MFP, 1 DSP, 2 autovector, 3 spurious
always @(posedge clk) begin
	iack_done <= 1'b0; iack_avec <= 1'b0; iack_spur <= 1'b0; iack_mfp <= 1'b0;
	if (iack_req) begin ia_pend <= 1'b1; ia_dly <= 4'd1 + ($urandom % 8); ia_lvl <= iack_level; end
	else if (ia_pend) begin
		if (ia_dly != 0) ia_dly <= ia_dly - 4'd1;
		else begin
			ia_pend   <= 1'b0;
			iack_done <= 1'b1;
			if (ia_lvl == irq_level && irq_level != 3'd0 && irq_kind != 2'd3) begin
				ia_kind_used <= irq_kind;
				case (irq_kind)
					2'd0: begin iack_vector <= irq_vec; iack_mfp <= 1'b1; end
					2'd1: iack_vector <= irq_vec;
					default: iack_avec <= 1'b1;
				endcase
			end
			else begin ia_kind_used <= 2'd3; iack_spur <= 1'b1; end
		end
	end
end

//----------------------------------------------------------------- checker
wire as_act = ~cpu_as_n & cpu_bus_oe;
integer e = 0;                 // processor rising edges before this clock
integer started = -1;          // S0 of the first bus cycle
integer finished = -1;         // S0 of the cycle that writes the result
integer cur_mark = -1;         // the last timing marker
integer itrace_mark = -1;      // +itrace=<n>: dispatches while marker n is current
reg     pe_tb = 1'b0;
reg     pipescan_stop_d = 1'b0;
// Hatari's marker point: the instruction boundary after the marker write,
// i.e. the dispatch of the next instruction once the governor has applied
// its adjustment for it (Falcon time + the pending adjustment, gA)
reg     mark_arm = 1'b0, mark_sync = 1'b0;
reg     itrace_pend = 1'b0;
reg [23:0] itrace_pc;
initial if (!$value$plusargs("itrace=%d", itrace_mark)) itrace_mark = -1;
integer credits = 0;
reg     in_cyc = 1'b0, c_iack, c_pend = 1'b0;
integer c_t0, c_exp, c_credit;
reg [23:0] c_a;
reg  [2:0] c_fc;
reg  [1:0] c_siz;
reg        c_rw;
reg        k_ym_seen = 1'b0, k_acia_seen = 1'b0, k_fdc4 = 1'b0;
integer    k_ym_cnt = 0;
integer    n_cyc = 0, n_bad = 0, n_cred_cyc = 0, n_held_cyc = 0;
integer    hist [0:31];
initial for (int i = 0; i < 32; i++) hist[i] = 0;

function automatic int esync_f(input int pos);
	int m;
	m = pos % 10;
	return (m == 0) ? 0 : 10 - m;
endfunction

// Hatari's device waits (M68000_WaitState) for a cycle of the 16-bit port
function automatic int dev_wait(input [23:0] x, input [1:0] sz, input wr, input int t0);
	int nb;
	nb = (sz == 2'b01) ? 1 : (sz == 2'b10) ? 2 : (sz == 2'b11) ? 3 : 4;
	if (x[23:6] == 18'h3FFE8 && (x[0] || nb >= 2) && ({x[5:1], 1'b1} <= 6'h25)) return 4;   // MFP
	if (x[23:2] == 22'h3FE200) begin                                                         // YM2149
		int w;
		if (!k_ym_seen) begin w = 4; k_ym_cnt = 0; end
		else begin k_ym_cnt = k_ym_cnt + 1; w = (k_ym_cnt % 4 == 0) ? 4 : 0; end
		k_ym_seen = 1'b1;
		return w;
	end
	if (x[23:4] == 20'hFF860) begin                                                          // FDC/DMA
		if (x[3:1] == 3'd2) return (wr || !k_fdc4) ? 4 : 0;
		if (x[3:1] == 3'd3) return wr ? 4 : 0;
		if (x[3:1] == 3'd7) return 4;
		return 0;
	end
	if (x[23:3] == 21'h1FFF80 && !x[0]) begin                                                // ACIA
		int w;
		w = 6 + (k_acia_seen ? 0 : esync_f(t0));
		k_acia_seen = 1'b1;
		return w;
	end
	if (x[23:3] == 21'h1FF440) return (sz == 2'b00) ? 8 : (nb >= 2) ? 4 : 0;                 // DSP host
	return 0;
endfunction

always @(posedge clk) begin
	if (in_cyc) c_credit = c_credit + cpu_credit;
	// end of a cycle: the first clock after AS negated
	if (in_cyc && !as_act) begin
		int len;
		in_cyc = 1'b0;
		len = e - c_t0;
		if (c_iack) begin
			case (ia_kind_used)
				2'd0: c_exp = 12;
				2'd2: c_exp = 10 + esync_f(c_t0);
				default: c_exp = 3;
			endcase
		end
		n_cyc = n_cyc + 1;
		if (c_credit != 0) n_cred_cyc = n_cred_cyc + 1;
		credits = credits + c_credit;
		if (len >= 0 && len < 32) hist[len] = hist[len] + 1;
		if (len != c_exp + c_credit) begin
			n_bad = n_bad + 1;
			if (n_bad <= 10)
				$display("TIMING: %0s a=%06x fc=%0d siz=%0d at %0d: %0d clocks, Hatari %0d (+%0d unavoidable)",
				         c_rw ? "RD" : "WR", c_a, c_fc, c_siz, c_t0, len, c_exp, c_credit);
		end
		if (trace)
			$display("%8d %0s a=%06x fc=%0d siz=%0d S0 at %0d: %0d clocks (Hatari %0d, +%0d)",
			         sysclk, c_rw ? "RD" : "WR", c_a, c_fc, c_siz, c_t0, len, c_exp, c_credit);
	end
	// start of a cycle: the first clock with AS.  Its S0 position counts the
	// idle clocks the governor inserts before the cycle goes on: it is taken
	// at the processor's next rising edge (S0 was the edge before it)
	if (as_act && !in_cyc && !reset) begin
		in_cyc   = 1'b1;
		c_pend   = 1'b1;
		c_a      = cpu_a[23:0];
		c_fc     = cpu_fc;
		c_siz    = cpu_siz;
		c_rw     = cpu_rw;
		c_credit = 0;
		c_iack   = 1'b0;
	end
	if (c_pend && cpu_ce) begin
		reg fast16;
		c_pend = 1'b0;
		c_t0   = e - 1;
		if (started < 0) started = c_t0;
		if (!c_rw && c_a == 24'hFFFF00) finished = c_t0;
		// timing markers (timing/timing_body.i): the processor clock at S0
		if (!c_rw && c_a == 24'h0003F0) begin
			$display("MARK %0d %0d", cpu_do[31:16], c_t0);
			cur_mark = cpu_do[31:16];
			mark_arm = 1'b1;
		end
		if (c_fc == 3'd7) begin
			if (c_a[19:16] == 4'hF) c_iack = 1'b1;
			else c_exp = 3;
		end else begin
			fast16 = (c_a[23:20] == 4'hE) || (c_a[23:17] == 7'b1111101) || (c_a[23:16] == 8'hFF);
			c_exp = 3 + ((!fast16 && (c_t0 % 4) >= 2) ? 2 : 0);
			if (c_a[23:15] == 9'h1FF && c_fc[2]) c_exp = 3 + dev_wait(c_a, c_siz, !c_rw, c_t0);
			if (!c_rw && c_a[23:1] == 23'h7FC303 && (c_a[0] || c_siz != 2'b01)) k_fdc4 = cpu_do[20];
		end
	end
	if (cpu_inst && cpu_ce) begin k_ym_seen = 1'b0; k_acia_seen = 1'b0; end
	// +itrace: the Falcon time of each dispatch (the clock after it), and the
	// bus cycles, while the marker is current
	if (itrace_pend) begin
		itrace_pend = 1'b0;
		$display("I %06x %0d", itrace_pc, e + $signed(cpubus.gA));
	end
	if (itrace_mark >= 0 && cur_mark == itrace_mark && pe_tb && cpu_inst) begin
		itrace_pend = 1'b1; itrace_pc = dbg_pc[23:0];
	end
	if (mark_sync) begin
		mark_sync = 1'b0;
		$display("MARKE %0d %0d", cur_mark, e + $signed(cpubus.gA));
	end
	if (mark_arm && pe_tb && cpu_inst) begin mark_arm = 1'b0; mark_sync = 1'b1; end
	pe_tb = cpu_ce;
	// +itrace also shows the pipeline model's flushes and stop points
	if (itrace_mark >= 0 && cur_mark == itrace_mark) begin
		if (cpu_tm_flush) $display("F scan=%06x", cpu_tm_scan[23:0]);
		if (cpu_ce && $test$plusargs("ftrace")) $display("  c scan=%06x qn=%0d fpc=%06x out=%0d due=%0d/%06x sto=%0d/%06x stop=%0d/%06x istb=%0d", cpu.core.scan_pc[23:0], cpu.core.pq_n,
		                     cpu.core.fetch_pc[23:0], cpu.core.fetch_out, cpu.core.due_v, cpu.core.due_scan[23:0],
		                     cpu_scan_v, cpu_scan_to[23:0], cpu_fetch_stop_v, cpu_fetch_stop[23:0], cpu.core.i_stb);
		if (cpu_fetch_stop_v && !pipescan_stop_d) $display("S at=%06x", cpu_fetch_stop[23:0]);
	end
	pipescan_stop_d = cpu_fetch_stop_v;
	if (cpu_hold && in_cyc) n_held_cyc = n_held_cyc + 1;
	// Falcon time from reset, as falcon_cpuclk's position: processor and idle
	// clocks, less the clocks the governor gives back
	e = reset ? 0 : e + ((cpu_ce || cpu_idle_tick) ? 1 : 0) - cpu_back;
end

//----------------------------------------------------------------- run
initial begin
	#1 if (pipescan.optbl[16'h4E75] == 9'd0) $fatal(1, "FAIL: the opcode table (falcon_optbl.mem) was not loaded: run from tb/bustime");
	repeat (20) @(posedge clk);
	reset = 1'b0;
	while (!done && !cpu_halted && sysclk < maxclk) @(posedge clk);
	repeat (5) @(posedge clk);
	$display("cycles %0d, off Hatari's length %0d, with unavoidable wait states %0d (%0d clocks)",
	         n_cyc, n_bad, n_cred_cyc, credits);
	$write("cycle lengths:");
	for (int i = 0; i < 32; i++) if (hist[i] != 0) $write(" %0d:%0d", i, hist[i]);
	$display("");
	$display("debt peak %0d, forgiven %0d, held %0d system clocks; %0d system clocks",
	         debt_peak, forgiven, held, sysclk);
	$display("governor: %0d idle clocks inserted, %0d clocks given back", gov_idle, gov_back);
	$display("PROCESSOR CLOCKS %0d (from the first bus cycle to the result, less unavoidable wait states)",
	         finished - started - credits);
	if (cpu_halted) $display("FAIL: processor halted at pc %08x", dbg_pc);
	else if (!done) $display("FAIL: timeout at pc %08x", dbg_pc);
	else if (failed) $display("FAIL: program reports test %0d", fail_num);
	else if (n_bad != 0) $display("FAIL: %0d cycles off Hatari's length", n_bad);
	else $display("PASS");
	$finish;
end

endmodule
