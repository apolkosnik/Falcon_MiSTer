// falcon_mbox_test.sv - FPGA <-> HPS mailbox round-trip latency probe
//
// Measurement build only (FALCON_BRINGUP="MBOX_TEST" defines FALCON_MBOX_TEST;
// falcon_system then instantiates this on falcon_memarb's d3 port).  The
// question it answers: how long does it take for the FPGA to put a word into
// the DDR3, for a process on the ARM to see it and write an answer, and for
// the FPGA to see that answer -- the round trip a coprocessor on the ARM
// (e.g. an FPU) would pay per exchange.  The ARM side is tools/mbox_ping.
//
// Mailbox: guest $E80000..$EFFFFF (DDR3 byte 0x30E80000..), which the 68030
// never reaches (bus error) and no DMA uses.  16-bit words, guest (big
// endian) byte order: DDR3 byte k holds guest byte k.
//   +$000 CMD    ARM -> FPGA  run id (non-zero; a new value starts a run)
//   +$002 COUNT  ARM -> FPGA  samples to take (1..32768; 0 means 1)
//   +$004 DONE   FPGA -> ARM  run id, written when the run has ended
//   +$006 STATUS FPGA -> ARM  0 = complete, 1 = no answer within ~1 s
//   +$008 PING   FPGA -> ARM  sequence number
//   +$00A PONG   ARM -> FPGA  the sequence number echoed
//   +$00C MAGIC  FPGA -> ARM  $4D42 ("MB"), written at power on (then CMD
//                             is cleared: the DDR3 keeps an old run id)
//   +$00E CLKMHZ FPGA -> ARM  the clock in MHz (cycle counts below)
//   +$100 + 8*i  sample i: cycles[31:16], cycles[15:0], PONG reads (the
//                fourth word is not written)
//
// A run: read PONG once and number the samples from PONG + 1.  The ARM side
// sets PONG = PING before it writes CMD, so the first PING differs from the
// one it last saw whatever an earlier (e.g. timed-out) run left.  Then, for
// each sample write PING = seq and count clocks from the acknowledge of that
// write (the arbiter has passed it to the DDR3 controller) to the
// acknowledge of the PONG read that returns seq; store the count and the
// number of PONG reads.  PONG is
// read back to back.  When idle, CMD is read about every 1 ms.
//
// The DMA port follows docs/ARCHITECTURE.md (16 bit, level request, one
// clock acknowledge); this is a debug client, not part of the Falcon.
module falcon_mbox_test #(
    parameter int CLK_HZ = 32000000
) (
    input             clk,
    input             reset,        // power on

    output reg        dma_req = 1'b0,
    output reg        dma_we,
    output reg [23:1] dma_addr,
    output     [1:0]  dma_be,
    output reg [15:0] dma_wdata,
    input      [15:0] dma_rdata,
    input             dma_ack
);

localparam [23:0] MB      = 24'hE80000;
localparam [23:0] A_CMD   = MB + 24'h000;
localparam [23:0] A_COUNT = MB + 24'h002;
localparam [23:0] A_DONE  = MB + 24'h004;
localparam [23:0] A_STAT  = MB + 24'h006;
localparam [23:0] A_PING  = MB + 24'h008;
localparam [23:0] A_PONG  = MB + 24'h00A;
localparam [23:0] A_MAGIC = MB + 24'h00C;
localparam [23:0] A_CLK   = MB + 24'h00E;
localparam [23:0] A_SMP   = MB + 24'h100;

localparam int IDLE_CLKS = CLK_HZ / 1000;            // CMD poll period
localparam int TMO_CLKS  = CLK_HZ;                    // ~1 s without an answer
localparam int IW        = $clog2(IDLE_CLKS + 1);

localparam [3:0] S_MAGIC = 4'd0,  S_CLK  = 4'd1,  S_IDLE = 4'd2,  S_CMD   = 4'd3,
                 S_COUNT = 4'd4,  S_PONG0 = 4'd5, S_PING = 4'd6,  S_POLL  = 4'd7,
                 S_SMP0  = 4'd8,  S_SMP1 = 4'd9,  S_SMP2 = 4'd10, S_STAT  = 4'd11,
                 S_DONE  = 4'd12, S_CLRC = 4'd13;

reg  [3:0]    st = S_MAGIC;
reg  [IW-1:0] idle_t;
reg  [15:0]   last_run = 16'd0, run;
reg  [15:0]   count, idx, seq, polls;
reg  [31:0]   cyc;
reg  [15:0]   smp_lo;           // low half of the count, written after the high half

assign dma_be = 2'b11;

// issue one access; the request is held until the acknowledge
task automatic access(input we, input [23:0] a, input [15:0] d);
    dma_req   <= 1'b1;
    dma_we    <= we;
    dma_addr  <= a[23:1];
    dma_wdata <= d;
endtask

wire [23:0] smp_a = A_SMP + {5'd0, idx[14:0], 3'd0};

always @(posedge clk) begin
    if (dma_ack) dma_req <= 1'b0;
    if (st == S_POLL) cyc <= cyc + 1'd1;

    case (st)
        S_MAGIC: if (!dma_req) access(1'b1, A_MAGIC, 16'h4D42);
                 else if (dma_ack) st <= S_CLK;
        S_CLK:   if (!dma_req) access(1'b1, A_CLK, 16'(CLK_HZ / 1000000));
                 else if (dma_ack) st <= S_CLRC;
        // the DDR3 keeps its contents over a core reload: drop an old CMD
        S_CLRC:  if (!dma_req) access(1'b1, A_CMD, 16'd0);
                 else if (dma_ack) begin st <= S_IDLE; idle_t <= '0; end
        S_IDLE:  if (idle_t == IW'(IDLE_CLKS)) begin
                     access(1'b0, A_CMD, 16'd0);
                     st <= S_CMD;
                 end else
                     idle_t <= idle_t + 1'd1;
        S_CMD:   if (dma_ack) begin
                     if (dma_rdata != 16'd0 && dma_rdata != last_run) begin
                         run <= dma_rdata;
                         access(1'b0, A_COUNT, 16'd0);
                         st  <= S_COUNT;
                     end else begin
                         idle_t <= '0;
                         st     <= S_IDLE;
                     end
                 end
        S_COUNT: if (dma_ack) begin
                     count <= (dma_rdata == 16'd0) ? 16'd1 :
                              (dma_rdata > 16'd32768) ? 16'd32768 : dma_rdata;
                     idx   <= 16'd0;
                     access(1'b0, A_PONG, 16'd0);
                     st    <= S_PONG0;
                 end
        S_PONG0: if (dma_ack) begin
                     seq <= dma_rdata + 1'd1;
                     access(1'b1, A_PING, dma_rdata + 1'd1);
                     st  <= S_PING;
                 end
        S_PING:  if (dma_ack) begin
                     cyc   <= 32'd0;
                     polls <= 16'd0;
                     access(1'b0, A_PONG, 16'd0);
                     st    <= S_POLL;
                 end
        S_POLL:  if (dma_ack) begin
                     polls <= polls + 1'd1;
                     if (dma_rdata == seq) begin
                         smp_lo <= cyc[15:0];
                         access(1'b1, smp_a, cyc[31:16]);
                         st  <= S_SMP0;
                     end else if (cyc >= 32'(TMO_CLKS)) begin
                         access(1'b1, A_STAT, 16'd1);
                         st  <= S_STAT;
                     end else
                         access(1'b0, A_PONG, 16'd0);
                 end
        S_SMP0:  if (dma_ack) begin access(1'b1, smp_a + 24'd2, smp_lo); st <= S_SMP1; end
        S_SMP1:  if (dma_ack) begin access(1'b1, smp_a + 24'd4, polls);     st <= S_SMP2; end
        S_SMP2:  if (dma_ack) begin
                     if (idx + 1'd1 == count) begin
                         access(1'b1, A_STAT, 16'd0);
                         st <= S_STAT;
                     end else begin
                         idx <= idx + 1'd1;
                         seq <= seq + 1'd1;
                         access(1'b1, A_PING, seq + 1'd1);
                         st  <= S_PING;
                     end
                 end
        S_STAT:  if (dma_ack) begin access(1'b1, A_DONE, run); st <= S_DONE; end
        S_DONE:  if (dma_ack) begin
                     last_run <= run;
                     idle_t   <= '0;
                     st       <= S_IDLE;
                 end
        default: st <= S_IDLE;
    endcase

    if (reset) begin
        st       <= S_MAGIC;
        dma_req  <= 1'b0;
        last_run <= 16'd0;
    end
end

endmodule
