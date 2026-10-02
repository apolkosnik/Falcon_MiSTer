// DSP56001 memory primitives for the Falcon DSP (falcon_dsp).
//
// Standard Intel inference templates so Quartus maps them to M10K:
//   dsp_ram_tdp : true dual-port RAM, registered read on both ports
//                 (read-during-write on the same port returns the new data,
//                 mixed-port read-during-write is "don't care"; the DSP core
//                 never relies on it, it forwards write data itself).
//   dsp_ram_sp  : single-port RAM, registered read.
// Contents power up as zero (Hatari memsets its RAM arrays at init).

module dsp_ram_tdp #(
    parameter AW = 9,
    parameter DW = 24
) (
    input               clk,
    input               we_a,
    input      [AW-1:0] addr_a,
    input      [DW-1:0] d_a,
    output reg [DW-1:0] q_a,
    input               we_b,
    input      [AW-1:0] addr_b,
    input      [DW-1:0] d_b,
    output reg [DW-1:0] q_b
);
    reg [DW-1:0] mem [0:(1<<AW)-1] /*verilator public_flat_rw*/;

`ifdef VERILATOR
    // simulation: power-up contents zero (M10K power up as zero on the FPGA)
    integer i;
    initial begin
        for (i = 0; i < (1<<AW); i = i + 1) mem[i] = {DW{1'b0}};
    end
`endif

    /* verilator lint_off MULTIDRIVEN */
    always @(posedge clk) begin
        if (we_a) begin
            mem[addr_a] <= d_a;
            q_a <= d_a;
        end else
            q_a <= mem[addr_a];
    end

    always @(posedge clk) begin
        if (we_b) begin
            mem[addr_b] <= d_b;
            q_b <= d_b;
        end else
            q_b <= mem[addr_b];
    end
    /* verilator lint_on MULTIDRIVEN */
endmodule

module dsp_ram_sp #(
    parameter AW = 8,
    parameter DW = 24
) (
    input               clk,
    input               we,
    input      [AW-1:0] addr,
    input      [DW-1:0] d,
    output reg [DW-1:0] q
);
    reg [DW-1:0] mem [0:(1<<AW)-1] /*verilator public_flat_rw*/;

`ifdef VERILATOR
    // simulation: power-up contents zero (M10K power up as zero on the FPGA)
    integer i;
    initial begin
        for (i = 0; i < (1<<AW); i = i + 1) mem[i] = {DW{1'b0}};
    end
`endif

    always @(posedge clk) begin
        if (we) mem[addr] <= d;
        q <= mem[addr];
    end
endmodule
