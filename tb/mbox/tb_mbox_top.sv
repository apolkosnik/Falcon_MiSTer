// tb_mbox_top.sv - falcon_memarb (FALCON_MBOX_TEST) + falcon_mbox_test.
//
// The mailbox probe is wired to the arbiter's d3 port exactly as
// rtl/falcon/falcon_system.sv does it (mbx_* wires).  All other arbiter
// masters and the MiSTer DDRAM interface are ports for the C++ bench; the
// loader port is tied off.  Compile with +define+FALCON_MBOX_TEST.

module tb_mbox_top #(
    parameter int CLK_HZ = 1000000
) (
    input             clk,
    input             reset,

    input             vid_req,
    input      [23:3] vid_addr,
    output            vid_ack,
    output     [63:0] vid_data,
    output            vid_valid,

    input             d0_req,
    input             d0_we,
    input      [23:1] d0_addr,
    input       [1:0] d0_be,
    input      [15:0] d0_wdata,
    output     [15:0] d0_rdata,
    output            d0_ack,

    input             d1_req,
    input             d1_we,
    input      [23:1] d1_addr,
    input       [1:0] d1_be,
    input      [15:0] d1_wdata,
    output     [15:0] d1_rdata,
    output            d1_ack,

    input             d2_req,
    input             d2_we,
    input      [23:1] d2_addr,
    input       [1:0] d2_be,
    input      [15:0] d2_wdata,
    output     [15:0] d2_rdata,
    output            d2_ack,

    input             cpu_req,
    input             cpu_we,
    input      [23:2] cpu_addr,
    input       [3:0] cpu_be,
    input      [31:0] cpu_wdata,
    output     [31:0] cpu_rdata,
    output            cpu_ack,

    output            snoop_we,
    output     [23:0] snoop_addr,

    // d3 (the mailbox probe) observed by the bench
    output            mbx_req,
    output            mbx_we,
    output     [23:1] mbx_addr,
    output      [1:0] mbx_be,
    output     [15:0] mbx_wdata,
    output     [15:0] mbx_rdata,
    output            mbx_ack,

    input             DDRAM_BUSY,
    output      [7:0] DDRAM_BURSTCNT,
    output     [28:0] DDRAM_ADDR,
    input      [63:0] DDRAM_DOUT,
    input             DDRAM_DOUT_READY,
    output            DDRAM_RD,
    output     [63:0] DDRAM_DIN,
    output      [7:0] DDRAM_BE,
    output            DDRAM_WE
);

wire ld_busy_unused;

falcon_mbox_test #(.CLK_HZ(CLK_HZ)) mbox_test
(
    .clk(clk), .reset(reset),
    .dma_req(mbx_req), .dma_we(mbx_we), .dma_addr(mbx_addr), .dma_be(mbx_be),
    .dma_wdata(mbx_wdata), .dma_rdata(mbx_rdata), .dma_ack(mbx_ack)
);

falcon_memarb memarb
(
    .clk(clk), .reset(reset),
    .vid_req(vid_req), .vid_addr(vid_addr), .vid_ack(vid_ack), .vid_data(vid_data), .vid_valid(vid_valid),
    .ld_wr(1'b0), .ld_addr(24'd0), .ld_data(8'd0), .ld_busy(ld_busy_unused),
    .d0_req(d0_req), .d0_we(d0_we), .d0_addr(d0_addr), .d0_be(d0_be), .d0_wdata(d0_wdata),
    .d0_rdata(d0_rdata), .d0_ack(d0_ack),
    .d1_req(d1_req), .d1_we(d1_we), .d1_addr(d1_addr), .d1_be(d1_be), .d1_wdata(d1_wdata),
    .d1_rdata(d1_rdata), .d1_ack(d1_ack),
    .d2_req(d2_req), .d2_we(d2_we), .d2_addr(d2_addr), .d2_be(d2_be), .d2_wdata(d2_wdata),
    .d2_rdata(d2_rdata), .d2_ack(d2_ack),
    .d3_req(mbx_req), .d3_we(mbx_we), .d3_addr(mbx_addr), .d3_be(mbx_be), .d3_wdata(mbx_wdata),
    .d3_rdata(mbx_rdata), .d3_ack(mbx_ack),
    .cpu_req(cpu_req), .cpu_we(cpu_we), .cpu_addr(cpu_addr), .cpu_be(cpu_be),
    .cpu_wdata(cpu_wdata), .cpu_rdata(cpu_rdata), .cpu_ack(cpu_ack),
    .snoop_we(snoop_we), .snoop_addr(snoop_addr),
    .DDRAM_BUSY(DDRAM_BUSY), .DDRAM_BURSTCNT(DDRAM_BURSTCNT), .DDRAM_ADDR(DDRAM_ADDR),
    .DDRAM_DOUT(DDRAM_DOUT), .DDRAM_DOUT_READY(DDRAM_DOUT_READY), .DDRAM_RD(DDRAM_RD),
    .DDRAM_DIN(DDRAM_DIN), .DDRAM_BE(DDRAM_BE), .DDRAM_WE(DDRAM_WE)
);

endmodule
