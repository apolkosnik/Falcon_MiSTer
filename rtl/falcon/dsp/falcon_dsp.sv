// falcon_dsp - Atari Falcon030 DSP56001 (32 MHz, 32K words of zero wait
// state SRAM), host interface and SSI.
//
// Behavioural reference: Hatari src/falcon/dsp_cpu.c (core, see
// dsp56k_core.sv / dsp56k_alu.sv), dsp_core.c (memory map, ROM tables, host
// port, bootstrap, SSI, port C; see dsp_periph.sv) and dsp.c
// (DSP_HandleReadAccess / DSP_HandleWriteAccess: the 68030 accesses the host
// port byte by byte at $FFA200-$FFA207).
//
// 68030 host port, byte offsets ($FFA200 + n), even bytes on bus_uds
// (bits 15:8), odd bytes on bus_lds (bits 7:0):
//   0 ICR   1 CVR   2 ISR (read only)   3 IVR
//   4 (reads 0)   5 RXH/TXH   6 RXM/TXM   7 RXL/TXL
// Reading RXL acknowledges a word; writing TXL sends a word (the 68030 side
// byte order is high, middle, low).  A word access is two byte accesses in
// address order.  bus_ack is given on the bus_stb clock.
//
// hreq   = ISR.HREQ (bit 7): (ICR & ISR & 3) != 0.  Level, as on the
//          hardware; the system ORs it into IPL6 and uses ivr (IVR, $FFA203)
//          as the vector when the MFP does not request.
// iack   accepted but unused: HREQ is level sensitive and is removed by the
//        68030 servicing the host port, not by the acknowledge.
// dsp_reset  DSP reset, level sensitive.  On the Falcon the DSP reset line is
//          PSG port A bit 4 (Hatari psg.c: a port A write with bit 4 set calls
//          DSP_Reset); the system drives dsp_reset = PSG port A bit 4.  After
//          reset the DSP waits for its bootstrap: the next 512 words written
//          through TXH/TXM/TXL go to P:$0000-$01FF and the DSP starts at
//          P:$0000 (R0 = 512, OMR = 2) after the 512th word, or earlier when
//          the 68030 sets ICR.HF0 (Hatari dsp_core_write_host).
//
// SSI slot link to the crossbar (see falcon_crossbar.sv header for timing):
//   ssi_slot_stb   one clock per slot event
//   ssi_tx_en      the strobe clocks the transmitter: SC2(ssi_frame) + SCK
//                  (Hatari DSP_SsiReceive_SC2 + DSP_SsiReceive_SCK);
//                  ssi_tx_data is the transmitted value (Hatari
//                  dsp_core.ssi.transmit_value: TX >> (24 - word length),
//                  0 when TE is off or the transmitter waits for a frame).
//                  It depends combinationally on DSP registers and ssi_frame
//                  only.  16-bit words are TX bits 23:8.
//   ssi_rx_en      the strobe clocks the receiver: SC1(ssi_rx_frame) + SC0
//                  with ssi_rx_data (sign extended to 24 bits, then shifted
//                  left by 24 - word length: 16-bit words land in RX 23:8).
//   ssi_tx_valid   Hatari dmaRecord.handshakeMode_Frame: set when the DSP
//                  writes TX while PC5 handshake framing is on, cleared when
//                  the DSP writes PC5 = 0 (PCDDR bit 5 set) or a transmit
//                  slot takes the word.
//   ssi_hs_play_req one clock pulse when the DSP writes PCD with PC4 = 1 while
//                  PCDDR bit 4 is set (Hatari dsp_core_setPortCDataRegister ->
//                  DSP_SsiTransmit_SC1 -> Crossbar_DmaPlayInHandShakeMode).
//
// Memories (M10K): 32K x 24 external RAM, 512 x 24 internal P RAM (also the
// bootstrap target), 256 x 24 internal X and Y RAM, 256 x 24 X and Y ROM
// (Hatari's mu-law/A-law and sine tables), 128 x 24 peripheral storage.
//
// Deviations from Hatari (hardware documentation wins):
//   - SWI raises its interrupt (P:$0006), WAIT waits for an interrupt, STOP
//     stops until reset (Hatari: NOPs).
//   - Bit-reverse addressing with modifier 0 leaves Rn unchanged.
//   - HREQ level / reset level / handshake TFS: see dsp_periph.sv.
//   - The crossbar link carries 16-bit words: with a 24-bit SSI word length
//     only the low 16 bits of Hatari's transmit_value reach the crossbar
//     (as Hatari's DAC path, which takes int16), and received words are the
//     sign-extended 16-bit slot data (Hatari's DMA play convention).
//   - SSI RX keeps 24 bits; Hatari's SHFD bit swap can leave a 25th bit in
//     ssi.RX, visible there only through a bit test of bit 24 of X:$FFEF.
// Everything else follows Hatari's emulation exactly, including its cycle
// counts (instruction clocks = Hatari instr_cycle at 32 MHz, 2 clocks per
// instruction cycle: 16 MIPS), and is checked instruction by instruction
// against Hatari's dsp_cpu.c / dsp_core.c by tb/dsp (run.sh).
//
// Resources (Quartus 17.0, 5CSEBA6U23I7, this module alone, tb/dsp/syn):
// about 6.7k ALMs, 2.4k registers, 104 M10K, 1 DSP block; meets 32 MHz at
// all corners (slow 100C Fmax about 34.5 MHz).

module falcon_dsp #(
    parameter CLK_HZ = 32000000
) (
    input             clk,
    input             reset,

    input             bus_cs,
    input             bus_stb,
    input             bus_we,
    input       [2:1] bus_addr,
    input             bus_uds,
    input             bus_lds,
    input      [15:0] bus_din,
    output     [15:0] bus_dout,
    output            bus_ack,

    output            hreq,
    output      [7:0] ivr,
    input             iack,

    input             dsp_reset,

    input             ssi_slot_stb,
    input             ssi_frame,
    input      [15:0] ssi_rx_data,
    output     [15:0] ssi_tx_data,
    output            ssi_tx_valid,
    input             ssi_tx_en,
    input             ssi_rx_en,
    input             ssi_rx_frame,
    output            ssi_hs_play_req
);

    wire rst = reset | dsp_reset;

    wire        running, run_start, boot_we;
    wire  [9:0] boot_pos;
    wire  [8:0] boot_addr;
    wire [23:0] boot_data;
    wire        perx_rd, perx_wr, pery_rd, pery_wr;
    wire  [5:0] perx_addr, pery_addr;
    wire [23:0] perx_wdata, perx_rdata, pery_wdata, pery_rdata;
    wire        soft_reset, hc_ack;
    wire        int_host_rcv, int_host_trx, int_host_cmd, int_ssi_rcv, int_ssi_trx;
    wire        msk_host_rcv, msk_host_trx, msk_host_cmd, msk_ssi_rcv, msk_ssi_trx;
    wire [23:0] ipr;
    wire  [4:0] hc_vector;
    wire        retire;

    dsp56k_core u_core (
        .clk(clk), .rst(rst),
        .run(running), .run_start(run_start), .boot_pos(boot_pos),
        .boot_we(boot_we), .boot_addr(boot_addr), .boot_data(boot_data),
        .perx_rd(perx_rd), .perx_wr(perx_wr), .perx_addr(perx_addr),
        .perx_wdata(perx_wdata), .perx_rdata(perx_rdata),
        .pery_rd(pery_rd), .pery_wr(pery_wr), .pery_addr(pery_addr),
        .pery_wdata(pery_wdata), .pery_rdata(pery_rdata),
        .per_soft_reset(soft_reset),
        .int_host_rcv(int_host_rcv), .int_host_trx(int_host_trx), .int_host_cmd(int_host_cmd),
        .int_ssi_rcv(int_ssi_rcv), .int_ssi_trx(int_ssi_trx),
        .msk_host_rcv(msk_host_rcv), .msk_host_trx(msk_host_trx), .msk_host_cmd(msk_host_cmd),
        .msk_ssi_rcv(msk_ssi_rcv), .msk_ssi_trx(msk_ssi_trx),
        .ipr(ipr), .hc_vector(hc_vector), .hc_ack(hc_ack),
        .retire(retire));

    dsp_periph u_periph (
        .clk(clk), .rst(rst),
        .bus_cs(bus_cs), .bus_stb(bus_stb), .bus_we(bus_we), .bus_addr(bus_addr),
        .bus_uds(bus_uds), .bus_lds(bus_lds), .bus_din(bus_din), .bus_dout(bus_dout),
        .bus_ack(bus_ack), .hreq(hreq), .ivr(ivr),
        .running(running), .run_start(run_start), .boot_pos(boot_pos),
        .boot_we(boot_we), .boot_addr(boot_addr), .boot_data(boot_data),
        .perx_rd(perx_rd), .perx_wr(perx_wr), .perx_addr(perx_addr),
        .perx_wdata(perx_wdata), .perx_rdata(perx_rdata),
        .pery_rd(pery_rd), .pery_wr(pery_wr), .pery_addr(pery_addr),
        .pery_wdata(pery_wdata), .pery_rdata(pery_rdata),
        .soft_reset(soft_reset), .hc_ack(hc_ack),
        .int_host_rcv(int_host_rcv), .int_host_trx(int_host_trx), .int_host_cmd(int_host_cmd),
        .int_ssi_rcv(int_ssi_rcv), .int_ssi_trx(int_ssi_trx),
        .msk_host_rcv(msk_host_rcv), .msk_host_trx(msk_host_trx), .msk_host_cmd(msk_host_cmd),
        .msk_ssi_rcv(msk_ssi_rcv), .msk_ssi_trx(msk_ssi_trx),
        .ipr_out(ipr), .hc_vector(hc_vector),
        .ssi_slot_stb(ssi_slot_stb), .ssi_tx_en(ssi_tx_en), .ssi_frame(ssi_frame),
        .ssi_rx_en(ssi_rx_en), .ssi_rx_frame(ssi_rx_frame), .ssi_rx_data(ssi_rx_data),
        .ssi_tx_data(ssi_tx_data), .ssi_tx_valid(ssi_tx_valid),
        .ssi_hs_play_req(ssi_hs_play_req));

    wire unused_ok = &{1'b0, iack, retire};
endmodule
