// tb_fpu_top.sv - AP68030 + falcon_cpubus + falcon_memarb + falcon_fpu_bridge
//
// The wiring is the one in rtl/falcon/falcon_system.sv (cpu, cpubus, fpu,
// memarb instances): CPU pins <-> cpubus, cpubus RAM port <-> memarb cpu port,
// cpubus cp_* <-> bridge, bridge dma_* <-> memarb d3.  The other arbiter
// masters are idle; the DDRAM_* port and observation points are ports for the
// C++ bench (tb_fpu.cpp).
//
// CLK_HZ is the bridge's clock-rate parameter: it sets the mailbox poll period
// (CLK_HZ/200 clocks = 5 ms) and so the 100 ms alive window (20 polls).  The
// bench uses CLK_HZ = 200000 (poll every 1000 clocks, window about 20000
// clocks), i.e. one simulated clock = 5 us of Falcon time; this keeps the
// 100 ms window simulable while every protocol cycle is the real RTL.

module tb_fpu_top #(
    parameter int CLK_HZ = 200000,
    parameter int CPU_DIV = 1,          // the CPU on every CPU_DIV-th clock (falcon_system: 1 turbo, 2 = 16 MHz, 4 = 8 MHz)
    parameter int FMODE = 0             // 1: the bus bridge's Falcon mode at 16 MHz, paced by falcon_cpuclk (CPU_DIV unused)
) (
    input             clk,
    input             por,          // power-on reset (arbiter, bridge presence)
    input       [2:0] ipl_n,        // interrupt priority level (active low), autovectored
    input             reset,        // machine reset (CPU, bus bridge); the bench holds it with por

    // MiSTer DDRAM port
    input             DDRAM_BUSY,
    output      [7:0] DDRAM_BURSTCNT,
    output     [28:0] DDRAM_ADDR,
    input      [63:0] DDRAM_DOUT,
    input             DDRAM_DOUT_READY,
    output            DDRAM_RD,
    output     [63:0] DDRAM_DIN,
    output      [7:0] DDRAM_BE,
    output            DDRAM_WE,

    // observation: 68030 pins
    output     [31:0] o_a,
    output      [2:0] o_fc,
    output      [1:0] o_siz,
    output            o_rw,
    output            o_as_n,
    output            o_ds_n,
    output            o_bus_oe,
    output     [31:0] o_d_o,
    output     [31:0] o_d_i,
    output            o_dsack0_n,
    output            o_dsack1_n,
    output            o_berr_n,
    output            o_dbg_inst,
    output     [31:0] o_dbg_pc,
    output            o_halted,
    output            o_reset_oe,

    // observation: bridge
    output            o_cp_req,
    output            o_cp_we,
    output      [2:0] o_cp_id,
    output      [4:0] o_cp_off,
    output            o_cp_ack,
    output            o_cp_berr,
    output            o_present,
    output            o_d3_req,
    output            o_d3_we,
    output     [23:1] o_d3_addr,
    output     [15:0] o_d3_rdata,
    output     [15:0] o_d3_wdata,
    output            o_d3_ack
);

// ---------------------------------------------------------------- CPU
wire [31:0] cpu_a, cpu_do;
wire  [2:0] cpu_fc;
wire  [1:0] cpu_siz;
wire        cpu_rw, cpu_rmc_n, cpu_as_n, cpu_ds_n, cpu_dben_n, cpu_ecs_n, cpu_ocs_n;
wire        cpu_ciout_n, cpu_cbreq_n, cpu_bus_oe, cpu_d_oe, cpu_bg_n, cpu_ipend_n;
wire        cpu_reset_oe, cpu_refill_n, cpu_status_n, cpu_halted, cpu_dbg_inst;
wire [31:0] cpu_di;
wire        dsack0_n, dsack1_n, berr_n, avec_n, ciin_n;
wire        snoop_we;
wire [23:0] snoop_addr;
wire [31:0] dbg_pc;

wire dev_reset = reset | cpu_reset_oe;      // as in falcon_system: RESET resets the peripherals only

reg [1:0] cpu_div = 2'd0;
reg       div_ce = 1'b1;
always @(posedge clk) begin
    cpu_div <= (cpu_div == CPU_DIV - 1) ? 2'd0 : cpu_div + 2'd1;
    div_ce  <= (CPU_DIV == 1) || (cpu_div == CPU_DIV - 1);
end
wire       fc_ce, cpu_hold, cpu_fmode, cpu_inst, cpu_idle, cpu_idle_tick;
wire [1:0] cpu_tm_pop, cpu_tm_md;
wire [7:0] cpu_back;
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
wire [4:0] cpu_tpos;
wire [3:0] cpu_credit;
falcon_cpuclk cpuclk
(
    .clk(clk), .reset(reset), .turbo(!cpu_fmode), .cpu_16mhz(1'b1),
    .hold(cpu_hold), .credit(cpu_credit), .idle(cpu_idle), .back(cpu_back), .cpu_ce(fc_ce), .idle_tick(cpu_idle_tick), .tpos(cpu_tpos),
    .debt(), .debt_peak(), .forgiven(), .held(), .idled()
);
wire cpu_ce = (FMODE != 0) ? fc_ce : div_ce;

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
    .ipl_n(ipl_n), .ipend_n(cpu_ipend_n),
    .reset_n_i(~reset), .reset_n_oe(cpu_reset_oe),
    .cdis_n(1'b1), .mmudis_n(1'b1), .refill_n(cpu_refill_n), .status_n(cpu_status_n),
    .dbg_pc(dbg_pc), .dbg_sr(), .dbg_state(), .dbg_inst(cpu_dbg_inst), .tm_pop(cpu_tm_pop), .tm_md(cpu_tm_md), .dbg_halted(cpu_halted),
    .tm_q(cpu_tm_q), .tm_qn(cpu_tm_qn), .tm_scan(cpu_tm_scan), .tm_flush(cpu_tm_flush),
    .fetch_stop_v(cpu_fetch_stop_v), .fetch_stop(cpu_fetch_stop),
    .fetch_scan_v(cpu_scan_v), .fetch_scan_to(cpu_scan_to),
    .dbg_vbr(), .dbg_cacr(), .dbg_cache_clear(),
    .snoop_we(snoop_we), .snoop_addr({8'd0, snoop_addr}), .nmi_vec_nocache(1'b0), .fetch_lazy(cpu_fmode)
);

// ---------------------------------------------------------------- bus bridge
wire        cram_req, cram_we, cram_ack;
wire [23:2] cram_addr;
wire  [3:0] cram_be;
wire [31:0] cram_wdata, cram_rdata;
wire [63:0] cram_rdata64;

wire        cp_req, cp_we, cp_ack, cp_berr;
wire  [2:0] cp_id;
wire  [4:0] cp_off;
wire  [1:0] cp_siz;
wire [31:0] cp_wdata, cp_rdata;
wire        cpu_cycle_done;
wire        iack_req;
reg  [2:0]  iack_cnt = 3'd0;
reg         iack_done = 1'b0;
// interrupt acknowledge: autovector, answered three clocks after the request (the interrupt controller is not under test)
always @(posedge clk) begin
    iack_done <= 1'b0;
    if (iack_req) iack_cnt <= 3'd3;
    else if (iack_cnt != 3'd0) begin
        iack_cnt <= iack_cnt - 3'd1;
        if (iack_cnt == 3'd1) iack_done <= 1'b1;
    end
end

falcon_cpubus cpubus
(
    .clk(clk), .reset(reset),
    .ram_mb(4'd4), .ram_tos(1'b0),
    .a(cpu_a), .fc(cpu_fc), .siz(cpu_siz), .rw(cpu_rw), .as_n(cpu_as_n), .ds_n(cpu_ds_n),
    .bus_oe(cpu_bus_oe), .d_o(cpu_do), .d_i(cpu_di),
    .dsack0_n(dsack0_n), .dsack1_n(dsack1_n), .berr_n(berr_n), .avec_n(avec_n), .ciin_n(ciin_n),
    .ram_req(cram_req), .ram_we(cram_we), .ram_addr(cram_addr), .ram_be(cram_be),
    .ram_wdata(cram_wdata), .ram_rdata(cram_rdata), .ram_ack(cram_ack),
    .dev_cs(), .dev_stb(), .dev_we(), .dev_addr(),
    .dev_uds(), .dev_lds(), .dev_din(), .dev_dout(16'd0),
    .dev_ack(1'b0), .dev_berr(1'b0), .dev_super(),
    .iack_req(iack_req), .iack_level(), .iack_done(iack_done), .iack_avec(iack_done),
    .iack_spur(1'b0), .iack_vector(8'd0),
    .cp_req(cp_req), .cp_we(cp_we), .cp_id(cp_id), .cp_off(cp_off), .cp_siz(cp_siz),
    .cp_wdata(cp_wdata), .cp_ack(cp_ack), .cp_berr(cp_berr), .cp_rdata(cp_rdata),
    .cycle_done(cpu_cycle_done),
    .fmode_in(FMODE != 0), .fmode(cpu_fmode), .cpu_ce(cpu_ce), .tpos(cpu_tpos),
    .inst(cpu_dbg_inst & cpu_ce), .hold(cpu_hold), .credit(cpu_credit), .wbuf_busy(),
    .ram_rdata64(cram_rdata64), .snoop_we(snoop_we), .snoop_addr(snoop_addr),
    .buf_flush(1'b0), .iack_mfp(1'b0),
    .dispatch(cpu_dbg_inst), .tm_pop(cpu_tm_pop), .tm_md(cpu_tm_md),
    .idle(cpu_idle), .idle_tick(cpu_idle_tick), .back(cpu_back), .bus_lost(1'b0), .blit_acc(1'b0),
    .gov_idle(), .gov_back()
);

// ---------------------------------------------------------------- FPU bridge
wire        d3_req, d3_we, d3_ack;
wire [23:1] d3_addr;
wire  [1:0] d3_be;
wire [15:0] d3_wdata, d3_rdata;
wire        fpu_present;

falcon_fpu_bridge #(.CLK_HZ(CLK_HZ)) fpu
(
    .clk(clk), .reset(dev_reset), .por(por),
    .cp_req(cp_req), .cp_we(cp_we), .cp_id(cp_id), .cp_off(cp_off), .cp_siz(cp_siz),
    .cp_wdata(cp_wdata), .cp_ack(cp_ack), .cp_berr(cp_berr), .cp_rdata(cp_rdata),
    .dma_req(d3_req), .dma_we(d3_we), .dma_addr(d3_addr), .dma_be(d3_be),
    .dma_wdata(d3_wdata), .dma_rdata(d3_rdata), .dma_ack(d3_ack),
    .present(fpu_present)
);

// ---------------------------------------------------------------- memory arbiter
falcon_memarb memarb
(
    .clk(clk), .reset(por), .ram_mb(4'd4),
    .vid_req(1'b0), .vid_addr(21'd0), .vid_ack(), .vid_data(), .vid_valid(),
    .ld_wr(1'b0), .ld_addr(24'd0), .ld_data(8'd0), .ld_busy(),
    .d0_req(1'b0), .d0_we(1'b0), .d0_addr(23'd0), .d0_be(2'b00), .d0_wdata(16'd0),
    .d0_rdata(), .d0_ack(),
    .d1_req(1'b0), .d1_we(1'b0), .d1_addr(23'd0), .d1_be(2'b00), .d1_wdata(16'd0),
    .d1_rdata(), .d1_ack(),
    .d2_req(1'b0), .d2_we(1'b0), .d2_addr(23'd0), .d2_be(2'b00), .d2_wdata(16'd0),
    .d2_rdata(), .d2_ack(),
    .d3_req(d3_req), .d3_we(d3_we), .d3_addr(d3_addr), .d3_be(d3_be), .d3_wdata(d3_wdata),
    .d3_rdata(d3_rdata), .d3_ack(d3_ack),
    .cpu_req(cram_req), .cpu_we(cram_we), .cpu_addr(cram_addr), .cpu_be(cram_be),
    .cpu_wdata(cram_wdata), .cpu_rdata(cram_rdata), .cpu_rdata64(cram_rdata64), .cpu_ack(cram_ack),
    .snoop_we(snoop_we), .snoop_addr(snoop_addr),
    .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
    .DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
    .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE)
);

// ---------------------------------------------------------------- observation
assign o_a = cpu_a, o_fc = cpu_fc, o_siz = cpu_siz, o_rw = cpu_rw, o_as_n = cpu_as_n, o_ds_n = cpu_ds_n;
assign o_bus_oe = cpu_bus_oe, o_d_o = cpu_do, o_d_i = cpu_di;
assign o_dsack0_n = dsack0_n, o_dsack1_n = dsack1_n, o_berr_n = berr_n;
assign o_dbg_inst = cpu_dbg_inst, o_dbg_pc = dbg_pc, o_halted = cpu_halted, o_reset_oe = cpu_reset_oe;
assign o_cp_req = cp_req, o_cp_we = cp_we, o_cp_id = cp_id, o_cp_off = cp_off;
assign o_cp_ack = cp_ack, o_cp_berr = cp_berr, o_present = fpu_present;
assign o_d3_req = d3_req, o_d3_we = d3_we, o_d3_addr = d3_addr, o_d3_rdata = d3_rdata, o_d3_wdata = d3_wdata, o_d3_ack = d3_ack;

endmodule
