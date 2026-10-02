// Testbench wrapper: three instances of the real falcon_psg sharing one bus.
//   u_tab : default parameters (Hatari table mixing, PWM filter, MIRROR=1)
//   u_lin : VOL_TABLE=0 (Hatari linear mixing)
//   u_nom : MIRROR=0 (Hatari Falcon decode without the 4-byte mirrors)
module tb_psg_top (
    input             clk,
    input             reset,
    input             bus_cs,
    input             bus_stb,
    input             bus_we,
    input      [7:1]  bus_addr,
    input             bus_uds,
    input             bus_lds,
    input      [15:0] bus_din,
    input       [7:0] port_a_in,
    input       [7:0] port_b_in,

    output     [15:0] t_dout,
    output            t_ack,
    output      [7:0] t_pa, t_pb,
    output            t_pa_oe, t_pb_oe,
    output      [4:0] t_cha, t_chb, t_chc,
    output     [15:0] t_smp,
    output            t_stb,

    output     [15:0] l_dout,
    output      [4:0] l_cha, l_chb, l_chc,
    output     [15:0] l_smp,
    output            l_stb,

    output     [15:0] n_dout,
    output            n_ack,
    output      [4:0] n_cha
);
    falcon_psg #(.VOL_MEM("../../rtl/falcon/falcon_psg_vol.mem")) u_tab (
        .clk(clk), .reset(reset), .bus_cs(bus_cs), .bus_stb(bus_stb), .bus_we(bus_we),
        .bus_addr(bus_addr), .bus_uds(bus_uds), .bus_lds(bus_lds), .bus_din(bus_din),
        .bus_dout(t_dout), .bus_ack(t_ack), .port_a_in(port_a_in), .port_b_in(port_b_in),
        .port_a_out(t_pa), .port_b_out(t_pb), .port_a_oe(t_pa_oe), .port_b_oe(t_pb_oe),
        .ch_a(t_cha), .ch_b(t_chb), .ch_c(t_chc), .snd_sample(t_smp), .snd_stb(t_stb));

    falcon_psg #(.VOL_TABLE(1'b0)) u_lin (
        .clk(clk), .reset(reset), .bus_cs(bus_cs), .bus_stb(bus_stb), .bus_we(bus_we),
        .bus_addr(bus_addr), .bus_uds(bus_uds), .bus_lds(bus_lds), .bus_din(bus_din),
        .bus_dout(l_dout), .bus_ack(), .port_a_in(port_a_in), .port_b_in(port_b_in),
        .port_a_out(), .port_b_out(), .port_a_oe(), .port_b_oe(),
        .ch_a(l_cha), .ch_b(l_chb), .ch_c(l_chc), .snd_sample(l_smp), .snd_stb(l_stb));

    falcon_psg #(.MIRROR(1'b0), .VOL_TABLE(1'b0)) u_nom (
        .clk(clk), .reset(reset), .bus_cs(bus_cs), .bus_stb(bus_stb), .bus_we(bus_we),
        .bus_addr(bus_addr), .bus_uds(bus_uds), .bus_lds(bus_lds), .bus_din(bus_din),
        .bus_dout(n_dout), .bus_ack(n_ack), .port_a_in(port_a_in), .port_b_in(port_b_in),
        .port_a_out(), .port_b_out(), .port_a_oe(), .port_b_oe(),
        .ch_a(n_cha), .ch_b(), .ch_c(), .snd_sample(), .snd_stb());
endmodule
