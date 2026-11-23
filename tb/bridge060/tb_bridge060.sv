//============================================================================
//  falcon_cpubus060 (the ap060 bus bridge, two clocks) with the real
//  falcon_memarb, ap060_fastram and falcon_ddr_arb on the DDR3 model: the
//  processor's bus on clk_cpu (50 MHz, BCLK = CLK), the Falcon on clk
//  (32 MHz), asynchronous.  From the bus audit of 2026-10-12, CT60 mode
//  (docs/CT60_MODE.md) and the 50 MHz processor:
//   - reset-vector mapping of single and line reads (ROM, RAM TOS), lines
//     at $0 and wrapping from $8
//   - CT60 mode, for each Fast RAM size: Fast RAM off keeps the 24-bit
//     mirror; Fast RAM reads and writes (long, word, byte, line) at both ends of the
//     size, through ap060_fastram into DDR3 0x31000000+, not the Falcon's
//     memory; the address after the end, a hole and $80000000 end in TEA;
//     the $FF000000 mirror reaches the Falcon RAM and ROM; a Fast RAM
//     address with A23-A3 zero is no reset vector
//   - Fast RAM timing: a cache hit in a fraction of an ST-RAM read, a
//     miss's line faster than an ST-RAM line
//   - the snoop FIFO under DMA writes while the processor keeps the bus: the
//     arbiter holds DMA writes before it can overflow (snoop_hold), and once
//     the processor's tenure ends every write is snooped once
//============================================================================
`timescale 1ns/1ps
module tb_bridge060;
reg clk=0;      always #15.625 clk=~clk;       // 32 MHz
reg clk_cpu=0;  always #10 clk_cpu=~clk_cpu;   // 50 MHz
reg reset=1;
reg [31:0] a=0, d_o=0;
reg a_oe=1, bb_oe=1, rw_n=1, ts_n=1, ram_tos=0;
reg [1:0] fast_ram=0;
reg [1:0] siz=0;
reg [3:0] bsn=0;            // bs_n[0] = BS0 = D31..D24
reg [1:0] psiz=0; reg [3:0] pbs=0;   // the next single transfer's size and strobes
wire [31:0] d_i;
wire ta_n, tea_n, ram_req, ram_we, ram_ack, cpu_rst;
wire [27:2] ram_addr;
wire [3:0] ram_be;
wire [31:0] ram_wdata, ram_rdata;
wire [63:0] ram_rdata64;
wire snoop_we, snoop_hold; wire [23:0] snoop_addr;
wire ts_n_i; wire [31:0] a_i;
reg d1_req=0;
reg [23:1] d1_addr=0;
wire d1_ack;
// DDR3: falcon_memarb and ap060_fastram through falcon_ddr_arb
wire        ma_busy, ma_dout_ready, ma_rd, ma_we;
wire  [7:0] ma_burstcnt, ma_be;
wire [28:0] ma_addr;
wire [63:0] ma_dout, ma_din;
wire [28:0] fr_address;
wire  [7:0] fr_burstcount, fr_byteenable;
wire        fr_read, fr_write, fr_waitrequest, fr_readdatavalid;
wire [63:0] fr_writedata, fr_readdata;
wire busy, dout_ready, ddr_rd, ddr_we;
wire [7:0] burstcnt, ddr_be;
wire [28:0] ddr_addr;
wire [63:0] din, dout;
falcon_cpubus060 dut(.clk(clk),.clk_cpu(clk_cpu),.reset(reset),.cpu_rst(cpu_rst),.rsto(1'b0),.rsto_sys(),
 .ram_mb(4'd4),.ram_tos(ram_tos),.fast_ram(fast_ram),
 .a(a),.a_oe(a_oe),.d_o(d_o),.d_oe(!rw_n),.rw_n(rw_n),.siz(siz),.bs_n(bsn),
 .tt(2'b00),.tm(3'd5),.ts_n(ts_n),.ciout_n(1'b1),.bb_oe(bb_oe),.d_i(d_i),.ta_n(ta_n),.tea_n(tea_n),
 .a_i(a_i),.ts_n_i(ts_n_i),
 .avm_address(fr_address),.avm_burstcount(fr_burstcount),.avm_read(fr_read),.avm_write(fr_write),
 .avm_writedata(fr_writedata),.avm_byteenable(fr_byteenable),.avm_waitrequest(fr_waitrequest),
 .avm_readdata(fr_readdata),.avm_readdatavalid(fr_readdatavalid),
 .ram_req(ram_req),.ram_we(ram_we),.ram_addr(ram_addr),
 .ram_be(ram_be),.ram_wdata(ram_wdata),.ram_rdata(ram_rdata),.ram_rdata64(ram_rdata64),.ram_ack(ram_ack),
 .dev_dout(16'd0),.dev_ack(1'b0),.dev_berr(1'b0),.iack_done(1'b0),.iack_avec(1'b0),
 .iack_spur(1'b0),.iack_vector(8'd0),.snoop_we(snoop_we),.snoop_addr(snoop_addr),.snoop_hold(snoop_hold),.blit_br(1'b0));
falcon_memarb arb(.clk(clk),.reset(reset),.ram_mb(4'd4),.vid_req(1'b0),.vid_addr(21'd0),
 .ld_wr(1'b0),.ld_addr(24'd0),.ld_data(8'd0),.d0_req(1'b0),.d2_req(1'b0),.d3_req(1'b0),
 .d1_req(d1_req),.d1_we(1'b1),.d1_addr(d1_addr),.d1_be(2'b11),.d1_wdata(16'hcafe),.d1_ack(d1_ack),
 .cpu_req(ram_req),.cpu_we(ram_we),.cpu_addr(ram_addr),.cpu_be(ram_be),.cpu_wdata(ram_wdata),
 .cpu_rdata(ram_rdata),.cpu_rdata64(ram_rdata64),.cpu_ack(ram_ack),.cpu_burst(1'b0),
 .snoop_we(snoop_we),.snoop_addr(snoop_addr),.snoop_hold(snoop_hold),
 .DDRAM_BUSY(ma_busy),.DDRAM_BURSTCNT(ma_burstcnt),.DDRAM_ADDR(ma_addr),.DDRAM_DOUT(ma_dout),
 .DDRAM_DOUT_READY(ma_dout_ready),.DDRAM_RD(ma_rd),.DDRAM_DIN(ma_din),.DDRAM_BE(ma_be),.DDRAM_WE(ma_we));
falcon_ddr_arb ddr_arb(.clk(clk),.reset(1'b0),
 .m0_address(fr_address),.m0_burstcount(fr_burstcount),.m0_read(fr_read),.m0_write(fr_write),
 .m0_writedata(fr_writedata),.m0_byteenable(fr_byteenable),.m0_waitrequest(fr_waitrequest),
 .m0_readdata(fr_readdata),.m0_readdatavalid(fr_readdatavalid),
 .m1_address(ma_addr),.m1_burstcount(ma_burstcnt),.m1_read(ma_rd),.m1_write(ma_we),
 .m1_writedata(ma_din),.m1_byteenable(ma_be),.m1_waitrequest(ma_busy),
 .m1_readdata(ma_dout),.m1_readdatavalid(ma_dout_ready),
 .s_address(ddr_addr),.s_burstcount(burstcnt),.s_read(ddr_rd),.s_write(ddr_we),
 .s_writedata(din),.s_byteenable(ddr_be),.s_waitrequest(busy),
 .s_readdata(dout),.s_readdatavalid(dout_ready));
ddr3_model mem(.clk(clk),.DDRAM_BUSY(busy),.DDRAM_BURSTCNT(burstcnt),.DDRAM_ADDR(ddr_addr),
 .DDRAM_DOUT(dout),.DDRAM_DOUT_READY(dout_ready),.DDRAM_RD(ddr_rd),.DDRAM_DIN(din),.DDRAM_BE(ddr_be),.DDRAM_WE(ddr_we));
integer errors=0, observed=0;
reg [31:0] result[4];
reg [31:0] wbuf[4];
bit got_tea;
integer xclk;    // clk_cpu clocks from TS to the transfer's last TA
task check(input bit ok,input string msg);
 if (!ok) begin errors++; $display("FAIL: %s",msg); end else $display("PASS: %s",msg);
endtask
// a transfer as the 68060 runs it on clk_cpu (BCLK = CLK): TS for one clock,
// write data from the clock after TS, a line's next beat the clock after
// the processor sampled TA; it may end in TEA
task cpu_xfer(input [31:0] addr, input bit line, input bit wr);
 integer n,k;
 begin
  @(negedge clk_cpu); #1;
  a=addr; siz=line ? 2'd3 : psiz; bsn=line ? 4'd0 : pbs; rw_n=!wr; d_o=wbuf[0]; ts_n=0; got_tea=0;
  @(posedge clk_cpu); @(negedge clk_cpu); #1; ts_n=1;
  n=0; k=0; xclk=0;
  while(k<(line ? 4:1) && !got_tea && n<40000) begin
   @(posedge clk_cpu);
   if (!tea_n) got_tea=1;
   else if (!ta_n) begin
    result[k]=d_i; k++; xclk=n+2;
    @(negedge clk_cpu); #1; if (k<4) d_o=wbuf[k];
   end
   n++;
  end
  if(n==40000) $fatal(1,"CPU timeout at %08x",addr);
  rw_n=1; psiz=0; pbs=0; bsn=0;
  repeat(4) @(negedge clk_cpu);
 end
endtask
task rst_with(input [1:0] fr);
 begin
  @(negedge clk); reset=1; fast_ram=fr; a_oe=1; bb_oe=1;
  repeat(5) @(negedge clk); reset=0;
  while (cpu_rst) @(negedge clk_cpu);
  repeat(4) @(negedge clk_cpu);
 end
endtask
task long_wr(input [31:0] addr, input [31:0] v); begin wbuf[0]=v; cpu_xfer(addr,0,1); end endtask
task long_rd(input [31:0] addr); begin cpu_xfer(addr,0,0); end endtask
// byte (SIZ 01) and word (SIZ 10) writes, the data on its own lanes
task byte_wr(input [31:0] addr, input [7:0] v);
 begin psiz=2'b01; pbs=~(4'b0001 << addr[1:0]); wbuf[0]=32'(v) << (24-8*addr[1:0]); cpu_xfer(addr,0,1); end
endtask
task word_wr(input [31:0] addr, input [15:0] v);
 begin psiz=2'b10; pbs=~(4'b0011 << {addr[1],1'b0}); wbuf[0]=32'(v) << (16-16*addr[1]); cpu_xfer(addr,0,1); end
endtask
task dma_write(input [23:0] addr);
 begin
  @(negedge clk); d1_addr=addr[23:1]; d1_req=1;
  do begin @(posedge clk); #1; end while(!d1_ack);
  @(negedge clk); d1_req=0;
  repeat(2) @(negedge clk);
 end
endtask
always @(posedge clk_cpu) if (!ts_n_i) observed++;
integer tl0, tn0;
initial begin
 rst_with(2'd0);
 mem.mem[0]=64'h4444333322221111;
 mem.mem[1]=64'h8888777766665555;
 mem.mem[2]=64'h0123456789abcdef;   // guest $000010: ef cd ab 89 (the model is little endian, as the HPS DDR3)
 mem.mem['he00000>>3]=64'hbbbbaaaa00008000;
 long_rd(0);
 check(result[0]==32'h00800000,"single reset-vector read maps ROM");
 cpu_xfer(0,1,0);
 check(result[0]==32'h00800000 && result[1]==32'haaaabbbb,
       $sformatf("line at zero maps ROM vectors: got %08x %08x",result[0],result[1]));
 check(result[2]==32'h55556666 && result[3]==32'h77778888,"line at zero keeps RAM bytes 8..15");
 cpu_xfer(8,1,0);
 check(result[2]==32'h00800000 && result[3]==32'haaaabbbb,
       $sformatf("line starting at 8 maps wrapped vector beats: got %08x %08x",result[2],result[3]));
 ram_tos=1;
 long_rd(0); check(result[0]==32'h00008000,"RAM TOS single vector is synthesized");
 cpu_xfer(0,1,0);
 check(result[0]==32'h00008000 && result[1]==32'h00e00000,
       $sformatf("RAM TOS line vectors are synthesized: got %08x %08x",result[0],result[1]));
 ram_tos=0;
 // ST-RAM writes and reads back, long and line (through the crossing)
 long_wr(32'h00003000, 32'h5a5aa5a5); long_rd(32'h00003000);
 check(!got_tea && result[0]==32'h5a5aa5a5, $sformatf("ST-RAM long written and read back: %08x", result[0]));
 wbuf[0]=32'h10000001; wbuf[1]=32'h10000002; wbuf[2]=32'h10000003; wbuf[3]=32'h10000000;
 cpu_xfer(32'h00003104,1,1); cpu_xfer(32'h00003100,1,0);
 check(!got_tea && result[0]==32'h10000000 && result[1]==32'h10000001 && result[2]==32'h10000002 && result[3]==32'h10000003,
       $sformatf("ST-RAM line written (wrapping from A3:A2 = 1) and read back: %08x %08x %08x %08x", result[0], result[1], result[2], result[3]));
 long_wr(32'h00003200, 32'h00000000); byte_wr(32'h00003201, 8'h5b); word_wr(32'h00003202, 16'hc3d4);
 long_rd(32'h00003200);
 check(!got_tea && result[0]==32'h005bc3d4, $sformatf("ST-RAM byte and word writes land on their lanes: %08x", result[0]));
 wbuf[0]=32'h11111111; cpu_xfer(32'h00e00000,0,1);
 check(got_tea, "a write to the ROM ends in TEA");
 long_rd(32'h00500000);
 check(got_tea, "a read above the ST-RAM top ends in TEA");

 // CT60 mode, for each Fast RAM size
 rst_with(2'd0);
 long_rd(32'h01000010);
 check(!got_tea && result[0]==32'hefcdab89, $sformatf("Fast RAM off: $01000010 mirrors $000010 (24-bit bus): %08x", result[0]));
 long_rd(32'h80000010);
 check(!got_tea && result[0]==32'hefcdab89, "Fast RAM off: $80000010 mirrors $000010");
 for(integer fr=1;fr<=3;fr++) begin
  logic [31:0] top;
  top = (fr==1) ? 32'h05000000 : (fr==2) ? 32'h09000000 : 32'h10000000;
  rst_with(2'(fr));
  long_wr(32'h01000000, 32'hfa570000 + 32'(fr));
  long_wr(top-4, 32'hfa57ffff - 32'(fr));
  long_rd(32'h01000000);
  check(!got_tea && result[0]==32'hfa570000 + 32'(fr), $sformatf("size %0d: Fast RAM first long word: %08x", fr, result[0]));
  long_rd(top-4);
  check(!got_tea && result[0]==32'hfa57ffff - 32'(fr), $sformatf("size %0d: Fast RAM last long word: %08x", fr, result[0]));
  long_wr(32'h01000204, 32'h11223344); byte_wr(32'h01000204, 8'ha0+8'(fr)); byte_wr(32'h01000207, 8'h0b); word_wr(32'h01000206, 16'h7e0b);
  word_wr(32'h01000208, 16'h1234); long_rd(32'h01000204);
  check(!got_tea && result[0]=={8'ha0+8'(fr), 24'h227e0b}, $sformatf("size %0d: Fast RAM byte and word writes land on their lanes: %08x", fr, result[0]));
  long_rd(32'h00000000);
  check(!got_tea && result[0]==32'h00800000, $sformatf("size %0d: Fast RAM $01000000 is not the vector at $000000 (ROM SSP): %08x", fr, result[0]));
  long_rd(top);
  check(got_tea, $sformatf("size %0d: TEA at the address after the Fast RAM (%08x)", fr, top));
  wbuf[0]=32'h11111111; cpu_xfer(top,0,1);
  check(got_tea, $sformatf("size %0d: TEA for a write after the Fast RAM", fr));
  long_rd(32'h20000000);
  check(got_tea, $sformatf("size %0d: TEA in the hole at $20000000", fr));
  long_rd(32'h80000010);
  check(got_tea, $sformatf("size %0d: TEA at $80000010 (no 24-bit mirror)", fr));
  long_rd(32'hff000010);
  check(!got_tea && result[0]==32'hefcdab89, $sformatf("size %0d: $FF000010 reaches ST-RAM $000010: %08x", fr, result[0]));
  long_rd(32'hffe00004);
  check(!got_tea && result[0]==32'haaaabbbb, $sformatf("size %0d: $FFE00004 reaches the ROM: %08x", fr, result[0]));
  // a Fast RAM line written and read back, wrapping (A3:A2 = 2 first)
  wbuf[0]=32'hc0000002+32'(fr<<8); wbuf[1]=32'hc0000003+32'(fr<<8); wbuf[2]=32'hc0000000+32'(fr<<8); wbuf[3]=32'hc0000001+32'(fr<<8);
  cpu_xfer(top-16+8,1,1);
  check(!got_tea, $sformatf("size %0d: Fast RAM line write in the last line", fr));
  cpu_xfer(top-16,1,0);
  check(!got_tea && result[0]==32'hc0000000+32'(fr<<8) && result[1]==32'hc0000001+32'(fr<<8) &&
        result[2]==32'hc0000002+32'(fr<<8) && result[3]==32'hc0000003+32'(fr<<8),
        $sformatf("size %0d: Fast RAM line read back: %08x %08x %08x %08x", fr, result[0], result[1], result[2], result[3]));
  // the writes reached DDR3 0x31000000 (word offset 0x200000 from 0x30000000)
  repeat(400) @(negedge clk);
  // (ap060_fastram's layout: 16-bit words, big-endian, the lowest address's
  // word in bits 15:0; only the processor reads this window)
  check(mem.wrd(25'h200000) == {32'd0, 16'h0000 + 16'(fr), 16'hfa57},
        $sformatf("size %0d: Fast RAM $01000000 is DDR3 0x31000000: %016x", fr, mem.wrd(25'h200000)));
  check(mem.wrd(25'h200040) == {16'h7e0b, 8'ha0+8'(fr), 8'h22, 32'd0} && mem.wrd(25'h200041) == 64'h1234,
        $sformatf("size %0d: Fast RAM $01000204-$01000209 partial writes are in DDR3: %016x %016x", fr, mem.wrd(25'h200040), mem.wrd(25'h200041)));
  check(mem.mem[0]==64'h4444333322221111 && mem.mem[1]==64'h8888777766665555,
        $sformatf("size %0d: ST-RAM $000000-$00000F unchanged by Fast RAM writes: %016x %016x", fr, mem.mem[0], mem.mem[1]));
  long_rd(32'h00000010);
  check(!got_tea && result[0]==32'hefcdab89, $sformatf("size %0d: ST-RAM $000010 unchanged by Fast RAM writes", fr));
 end

 // transfer lengths (clk_cpu clocks from TS to the last TA, 16 of each):
 // ST-RAM crosses to clk and the arbiter, Fast RAM is the processor's own
 begin
  integer tl, tn, fl, fn;
  rst_with(2'd1);
  // (the reset sweeps the front end's cache invalid: 512 clocks in which
  // reads go to DDR3 and do not allocate)
  repeat(600) @(negedge clk_cpu);
  tl=0; tn=0; fl=0; fn=0;
  for(integer r=0;r<16;r++) begin
   long_rd(32'h00001000 + 32'(r*64)); tl+=xclk;
   cpu_xfer(32'h00002000 + 32'(r*64),1,0); tn+=xclk;
   long_rd(32'h01001000 + 32'(r*64)); fl+=xclk;
   cpu_xfer(32'h01002000 + 32'(r*64),1,0); fn+=xclk;
  end
  // (a miss is one DDR3 round trip, as an ST-RAM read: a long word costs
  // about the same, a line half, its four beats at one a clock)
  check(fn < tn && fl <= tl + 32, $sformatf("Fast RAM misses: line faster than ST-RAM (%0d < %0d), long within 2 clocks (%0d, %0d; clocks per 16)", fn, tn, fl, tl));
  $display("TIMING (clk_cpu 50 MHz clocks): ST-RAM long %0d.%02d, line %0d.%02d; Fast RAM long %0d.%02d, line %0d.%02d",
           tl/16, (tl%16)*100/16, tn/16, (tn%16)*100/16, fl/16, (fl%16)*100/16, fn/16, (fn%16)*100/16);
  // the same Fast RAM lines again: the front end's cache answers
  fl=0; fn=0;
  for(integer r=0;r<16;r++) begin
   long_rd(32'h01001000 + 32'(r*64)); fl+=xclk;
   cpu_xfer(32'h01002000 + 32'(r*64),1,0); fn+=xclk;
  end
  check(fl*4 < tl && fn*4 < tn, $sformatf("Fast RAM cache hits under a quarter of ST-RAM (long %0d, line %0d, clocks per 16)", fl, fn));
  $display("TIMING (clk_cpu 50 MHz clocks): Fast RAM cache hit long %0d.%02d, line %0d.%02d",
           fl/16, (fl%16)*100/16, fn/16, (fn%16)*100/16);
 end

 // DMA writes to distinct lines while the processor keeps the bus
 for(integer ci=0;ci<4;ci++) begin
  integer count, done, stall, peak;
  count = (ci==0) ? 27 : (ci==1) ? 32 : (ci==2) ? 33 : 200;
  rst_with(2'd0);
  observed=0; done=0; stall=0; peak=0;
  fork
   begin
    for(integer j=0;j<count;j++) dma_write(24'h1000+24'(j*16));
    done=1;
   end
  join_none
  // the processor keeps the bus until the writer is done or stalled
  while (!done && stall<2000) begin
   @(posedge clk);
   if (d1_req && !d1_ack) stall++; else stall=0;
   if (dut.sq_wlevel > peak) peak = dut.sq_wlevel;
  end
  $display("QUEUED with the bus kept: writer %s, FIFO peak %0d", done ? "done" : "held", peak);
  check(count < 28 || !done, $sformatf("%0d DMA lines: the arbiter holds DMA writes before the FIFO fills", count));
  check(peak <= 32, $sformatf("%0d DMA lines: the snoop FIFO never overflows (peak %0d of 32)", count, peak));
  @(negedge clk_cpu); a_oe=0; bb_oe=0;
  for(integer t=0;t<40000 && !done;t++) @(negedge clk);
  check(done==1, $sformatf("%0d DMA lines: every write completes once the processor releases the bus", count));
  repeat(800) @(negedge clk);
  check(observed==count,$sformatf("snoop delivery after CPU releases ownership: %0d of %0d",observed,count));
 end
 $display("SUMMARY: %0d failed checks",errors);
 if(errors) $fatal(1,"68060 bridge checks failed");
 $finish;
end
initial begin #20000000; $fatal(1,"global timeout"); end
endmodule
