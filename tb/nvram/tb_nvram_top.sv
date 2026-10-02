// Testbench wrapper: two instances of the real falcon_nvram on one bus.
//   u_real : CLK_HZ = 32 MHz (the core clock)
//   u_fast : CLK_HZ = 131072 (4 clocks per 32.768 kHz tick) for long
//            calendar runs
module tb_nvram_top (
    input             clk,
    input             reset,
    input             bus_cs,
    input             bus_stb,
    input             bus_we,
    input      [1:1]  bus_addr,
    input             bus_uds,
    input             bus_lds,
    input      [15:0] bus_din,
    input      [64:0] rtc,
    input             cfg_vga,
    input       [7:0] cfg_lang,
    input       [7:0] cfg_kbd,
    input             nv_init,
    input       [5:0] nv_addr,
    input       [7:0] nv_din,
    input             nv_wr,
    output     [15:0] r_dout,
    output            r_ack,
    output      [7:0] r_nv_dout,
    output            r_nv_changed,
    output            r_irq,
    output     [15:0] f_dout,
    output            f_ack,
    output      [7:0] f_nv_dout,
    output            f_nv_changed,
    output            f_irq
);
    falcon_nvram u_real (
        .clk(clk), .reset(reset), .bus_cs(bus_cs), .bus_stb(bus_stb), .bus_we(bus_we),
        .bus_addr(bus_addr), .bus_uds(bus_uds), .bus_lds(bus_lds), .bus_din(bus_din),
        .bus_dout(r_dout), .bus_ack(r_ack), .rtc(rtc), .cfg_vga(cfg_vga),
        .cfg_lang(cfg_lang), .cfg_kbd(cfg_kbd), .nv_init(nv_init), .nv_addr(nv_addr),
        .nv_dout(r_nv_dout), .nv_din(nv_din), .nv_wr(nv_wr), .nv_changed(r_nv_changed),
        .irq(r_irq));
    falcon_nvram #(.CLK_HZ(131072)) u_fast (
        .clk(clk), .reset(reset), .bus_cs(bus_cs), .bus_stb(bus_stb), .bus_we(bus_we),
        .bus_addr(bus_addr), .bus_uds(bus_uds), .bus_lds(bus_lds), .bus_din(bus_din),
        .bus_dout(f_dout), .bus_ack(f_ack), .rtc(rtc), .cfg_vga(cfg_vga),
        .cfg_lang(cfg_lang), .cfg_kbd(cfg_kbd), .nv_init(nv_init), .nv_addr(nv_addr),
        .nv_dout(f_nv_dout), .nv_din(nv_din), .nv_wr(nv_wr), .nv_changed(f_nv_changed),
        .irq(f_irq));
endmodule
