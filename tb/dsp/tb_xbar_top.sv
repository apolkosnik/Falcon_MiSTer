// Testbench top: the real falcon_crossbar connected to the real falcon_dsp
// through the SSI slot interface, as falcon_system will wire them.
// Plain ASCII.
module tb_xbar_top (
    input             clk,
    input             reset,
    // crossbar register bus ($FF8900-$FF8943)
    input             xb_cs,
    input             xb_stb,
    input             xb_we,
    input       [6:1] xb_addr,
    input             xb_uds,
    input             xb_lds,
    input      [15:0] xb_din,
    output     [15:0] xb_dout,
    output            xb_ack,
    // DSP host port ($FFA200-$FFA207)
    input             hp_cs,
    input             hp_stb,
    input             hp_we,
    input       [2:1] hp_addr,
    input             hp_uds,
    input             hp_lds,
    input      [15:0] hp_din,
    output     [15:0] hp_dout,
    output            hp_ack,
    // observation of the SSI link
    output            mon_stb,
    output            mon_tx_en,
    output            mon_rx_en,
    output            mon_frame,
    output     [15:0] mon_tx_data,
    output     [15:0] mon_rx_data
);
    wire        ssi_slot_stb, ssi_frame, ssi_tx_en, ssi_rx_en, ssi_rx_frame;
    wire [15:0] ssi_rx_data, ssi_tx_data;
    wire        ssi_tx_valid, ssi_hs_play_req;
    wire        hreq;
    wire  [7:0] ivr;

    falcon_crossbar u_xbar (
        .clk(clk), .reset(reset),
        .bus_cs(xb_cs), .bus_stb(xb_stb), .bus_we(xb_we), .bus_addr(xb_addr),
        .bus_uds(xb_uds), .bus_lds(xb_lds), .bus_din(xb_din), .bus_dout(xb_dout),
        .bus_ack(xb_ack), .bus_berr(),
        .dma_req(), .dma_we(), .dma_addr(), .dma_be(), .dma_wdata(),
        .dma_rdata(16'd0), .dma_ack(1'b0),
        .sndint(), .soundint(),
        .psg_audio(16'sd0), .mic_l(16'sd0), .mic_r(16'sd0),
        .audio_l(), .audio_r(), .audio_stb(),
        .ssi_slot_stb(ssi_slot_stb), .ssi_frame(ssi_frame), .ssi_rx_data(ssi_rx_data),
        .ssi_tx_data(ssi_tx_data), .ssi_tx_valid(ssi_tx_valid), .ssi_hs_play_req(ssi_hs_play_req),
        .ssi_tx_en(ssi_tx_en), .ssi_rx_en(ssi_rx_en), .ssi_rx_frame(ssi_rx_frame),
        .dbg_underrun());

    falcon_dsp u_dsp (
        .clk(clk), .reset(reset),
        .bus_cs(hp_cs), .bus_stb(hp_stb), .bus_we(hp_we), .bus_addr(hp_addr),
        .bus_uds(hp_uds), .bus_lds(hp_lds), .bus_din(hp_din), .bus_dout(hp_dout),
        .bus_ack(hp_ack), .hreq(hreq), .ivr(ivr), .iack(1'b0),
        .dsp_reset(1'b0),
        .ssi_slot_stb(ssi_slot_stb), .ssi_frame(ssi_frame), .ssi_rx_data(ssi_rx_data),
        .ssi_tx_data(ssi_tx_data), .ssi_tx_valid(ssi_tx_valid),
        .ssi_tx_en(ssi_tx_en), .ssi_rx_en(ssi_rx_en), .ssi_rx_frame(ssi_rx_frame),
        .ssi_hs_play_req(ssi_hs_play_req));

    assign mon_stb     = ssi_slot_stb;
    assign mon_tx_en   = ssi_tx_en;
    assign mon_rx_en   = ssi_rx_en;
    assign mon_frame   = ssi_frame;
    assign mon_tx_data = ssi_tx_data;
    assign mon_rx_data = ssi_rx_data;
endmodule
