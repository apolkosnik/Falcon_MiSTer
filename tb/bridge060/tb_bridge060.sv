//============================================================================
//  falcon_cpubus060 (the ap060 bus bridge) with the real falcon_memarb on the
//  DDR3 model, BCLK at half CLK.  From the bus audit of 2026-10-12:
//   - reset-vector mapping of single and line reads (ROM, RAM TOS), lines
//     at $0 and wrapping from $8
//   - the snoop queue under DMA writes while the processor keeps the bus:
//     the arbiter holds DMA writes before the queue can wrap (snoop_hold),
//     and once the processor's tenure ends every write is snooped once
//  CT60 mode (docs/CT60_MODE.md), for each Fast RAM size: Fast RAM off keeps
//  the 24-bit mirror; Fast RAM reads and writes (long and line) at both ends
//  of the size reach their own memory, not the Falcon's; the address after
//  the end, a hole above the Falcon space and $80000000 end in TEA; the
//  $FF000000 mirror reaches the Falcon RAM and ROM; a Fast RAM address with
//  A23-A3 zero is no reset vector.
//============================================================================
`timescale 1ns/1ps
module tb_bridge060;
reg clk=0; always #5 clk=~clk;
reg reset=1, bclk_en=0;
// BCLK = CLK/2, or CLK in CT60 mode (as falcon_system: taken at reset)
always @(negedge clk) bclk_en <= (dut.fsel != 2'd0) ? 1'b1 : ~bclk_en;
reg [31:0] a=0, d_o=0;
reg a_oe=1, bb_oe=1, rw_n=1, ts_n=1, ram_tos=0;
reg [1:0] fast_ram=0;
reg [1:0] siz=0;
wire [31:0] d_i;
wire ta_n, tea_n, ram_req, ram_we, ram_ack;
wire [27:2] ram_addr;
wire [3:0] ram_be;
wire [31:0] ram_wdata, ram_rdata;
wire [63:0] ram_rdata64;
wire snoop_we, snoop_hold; wire [23:0] snoop_addr;
wire ts_n_i; wire [31:0] a_i;
reg d1_req=0;
reg [23:1] d1_addr=0;
wire d1_ack;
wire busy, dout_ready, ddr_rd, ddr_we;
wire [7:0] burstcnt, ddr_be;
wire [28:0] ddr_addr;
wire [63:0] din, dout;
falcon_cpubus060 dut(.clk(clk),.reset(reset),.bclk_en(bclk_en),.ram_mb(4'd4),.ram_tos(ram_tos),.fast_ram(fast_ram),
 .a(a),.a_oe(a_oe),.d_o(d_o),.d_oe(!rw_n),.rw_n(rw_n),.siz(siz),.bs_n(4'b0000),
 .tt(2'b00),.tm(3'd5),.ts_n(ts_n),.bb_oe(bb_oe),.d_i(d_i),.ta_n(ta_n),.tea_n(tea_n),
 .a_i(a_i),.ts_n_i(ts_n_i),.ram_req(ram_req),.ram_we(ram_we),.ram_addr(ram_addr),
 .ram_be(ram_be),.ram_wdata(ram_wdata),.ram_rdata(ram_rdata),.ram_rdata64(ram_rdata64),.ram_ack(ram_ack),
 .dev_dout(16'd0),.dev_ack(1'b0),.dev_berr(1'b0),.iack_done(1'b0),.iack_avec(1'b0),
 .iack_spur(1'b0),.iack_vector(8'd0),.snoop_we(snoop_we),.snoop_addr(snoop_addr),.snoop_hold(snoop_hold),.blit_br(1'b0));
falcon_memarb arb(.clk(clk),.reset(reset),.ram_mb(4'd4),.vid_req(1'b0),.vid_addr(21'd0),
 .ld_wr(1'b0),.ld_addr(24'd0),.ld_data(8'd0),.d0_req(1'b0),.d2_req(1'b0),.d3_req(1'b0),
 .d1_req(d1_req),.d1_we(1'b1),.d1_addr(d1_addr),.d1_be(2'b11),.d1_wdata(16'hcafe),.d1_ack(d1_ack),
 .cpu_req(ram_req),.cpu_we(ram_we),.cpu_addr(ram_addr),.cpu_be(ram_be),.cpu_wdata(ram_wdata),
 .cpu_rdata(ram_rdata),.cpu_rdata64(ram_rdata64),.cpu_ack(ram_ack),.cpu_burst(1'b0),
 .snoop_we(snoop_we),.snoop_addr(snoop_addr),.snoop_hold(snoop_hold),.DDRAM_BUSY(busy),.DDRAM_BURSTCNT(burstcnt),
 .DDRAM_ADDR(ddr_addr),.DDRAM_DOUT(dout),.DDRAM_DOUT_READY(dout_ready),.DDRAM_RD(ddr_rd),
 .DDRAM_DIN(din),.DDRAM_BE(ddr_be),.DDRAM_WE(ddr_we));
ddr3_model mem(.clk(clk),.DDRAM_BUSY(busy),.DDRAM_BURSTCNT(burstcnt),.DDRAM_ADDR(ddr_addr),
 .DDRAM_DOUT(dout),.DDRAM_DOUT_READY(dout_ready),.DDRAM_RD(ddr_rd),.DDRAM_DIN(din),.DDRAM_BE(ddr_be),.DDRAM_WE(ddr_we));
integer errors=0, observed=0;
reg [31:0] result[4];
task check(input bit ok,input string msg);
 if (!ok) begin errors++; $display("FAIL: %s",msg); end else $display("PASS: %s",msg);
endtask
task cpu_read(input [31:0] addr,input bit line_read);
 integer n,k;
 begin
  @(negedge clk); #1;
  while (!bclk_en) begin @(negedge clk); #1; end
  a=addr; siz=line_read ? 3:0; ts_n=0;
  @(posedge clk); @(negedge clk); #1; ts_n=1;
  n=0; k=0;
  while(k<(line_read ? 4:1) && n<1000) begin
   @(posedge clk);
   if (bclk_en && !ta_n) begin result[k]=d_i; k++; end
   if (bclk_en && !tea_n) $fatal(1,"Unexpected TEA");
   n++;
  end
  if(n==1000) $fatal(1,"CPU timeout");
  repeat(6) @(negedge clk);
 end
endtask
// a transfer that may end in TEA: long or line, read or write (wbuf: the
// write data, one long word per beat in transfer order); got_tea reports it
reg [31:0] wbuf[4];
bit got_tea;
integer xclk;    // clocks from TS to the transfer's last TA
integer tl0, tn0;  // ST-RAM long and line lengths on the half-speed bus (x16)

task cpu_xfer(input [31:0] addr, input bit line, input bit wr);
 integer n,k;
 begin
  @(negedge clk); #1;
  while (!bclk_en) begin @(negedge clk); #1; end
  a=addr; siz=line ? 3:0; rw_n=!wr; d_o=wbuf[0]; ts_n=0; got_tea=0;
  @(posedge clk); @(negedge clk); #1; ts_n=1;
  n=0; k=0; xclk=0;
  while(k<(line ? 4:1) && !got_tea && n<4000) begin
   @(posedge clk);
   if (bclk_en && !tea_n) got_tea=1;
   else if (bclk_en && !ta_n) begin
    result[k]=d_i; k++; xclk=n+2;
    @(negedge clk); #1; if (k<4) d_o=wbuf[k];
   end
   n++;
  end
  if(n==4000) $fatal(1,"CPU timeout at %08x",addr);
  rw_n=1;
  repeat(6) @(negedge clk);
 end
endtask
task rst_with(input [1:0] fr);
 begin
  @(negedge clk); reset=1; fast_ram=fr; a_oe=1; bb_oe=1;
  repeat(5) @(negedge clk); reset=0;
  repeat(2) @(negedge clk);
 end
endtask
task long_wr(input [31:0] addr, input [31:0] v); begin wbuf[0]=v; cpu_xfer(addr,0,1); end endtask
task long_rd(input [31:0] addr); begin cpu_xfer(addr,0,0); end endtask
task dma_write(input [23:0] addr);
 begin
  @(negedge clk); d1_addr=addr[23:1]; d1_req=1;
  do begin @(posedge clk); #1; end while(!d1_ack);
  @(negedge clk); d1_req=0;
  repeat(2) @(negedge clk);
 end
endtask
always @(posedge clk) if (bclk_en && !ts_n_i) observed++;
initial begin
 repeat(5) @(negedge clk); reset=0;
 mem.mem[0]=64'h4444333322221111;
 mem.mem[1]=64'h8888777766665555;
 mem.mem['he00000>>3]=64'hbbbbaaaa00008000;
 cpu_read(0,0);
 check(result[0]==32'h00800000,"single reset-vector read maps ROM");
 cpu_read(0,1);
 check(result[0]==32'h00800000 && result[1]==32'haaaabbbb,
       $sformatf("line at zero maps ROM vectors: got %08x %08x",result[0],result[1]));
 check(result[2]==32'h55556666 && result[3]==32'h77778888,"line at zero keeps RAM bytes 8..15");
 cpu_read(8,1);
 check(result[2]==32'h00800000 && result[3]==32'haaaabbbb,
       $sformatf("line starting at 8 maps wrapped vector beats: got %08x %08x",result[2],result[3]));
 ram_tos=1;
 cpu_read(0,0); check(result[0]==32'h00008000,"RAM TOS single vector is synthesized");
 cpu_read(0,1);
 check(result[0]==32'h00008000 && result[1]==32'h00e00000,
       $sformatf("RAM TOS line vectors are synthesized: got %08x %08x",result[0],result[1]));
 // CT60 mode, for each Fast RAM size
 ram_tos=0;
 rst_with(2'd0);
 mem.mem[2]=64'h0123456789abcdef;   // guest $000010: ef cd ab 89 (the model is little endian, as the HPS DDR3)
 long_rd(32'h01000010);
 check(!got_tea && result[0]==32'hefcdab89, $sformatf("Fast RAM off: $01000010 mirrors $000010 (24-bit bus): %08x", result[0]));
 long_rd(32'h80000010);
 check(!got_tea && result[0]==32'hefcdab89, "Fast RAM off: $80000010 mirrors $000010");
 for(integer fr=1;fr<=3;fr++) begin
  logic [31:0] top;
  top = (fr==1) ? 32'h05000000 : (fr==2) ? 32'h09000000 : 32'h10000000;
  rst_with(2'(fr));
  // first and last long word of the Fast RAM, then the address after it
  long_wr(32'h01000000, 32'hfa570000 + 32'(fr));
  long_wr(top-4, 32'hfa57ffff - 32'(fr));
  long_rd(32'h01000000);
  check(!got_tea && result[0]==32'hfa570000 + 32'(fr), $sformatf("size %0d: Fast RAM first long word: %08x", fr, result[0]));
  // in the DDR3 at 0x31000000 (word offset 0x200000 from 0x30000000), guest bytes little endian
  check(mem.wrd(25'h200000) == {32'd0, 8'(32'hfa570000 + 32'(fr)), 8'((32'hfa570000 + 32'(fr)) >> 8), 8'h57, 8'hfa},
        $sformatf("size %0d: Fast RAM $01000000 is DDR3 0x31000000: %016x", fr, mem.wrd(25'h200000)));
  long_rd(top-4);
  check(!got_tea && result[0]==32'hfa57ffff - 32'(fr), $sformatf("size %0d: Fast RAM last long word: %08x", fr, result[0]));
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
  // the Falcon mirror at $FFxxxxxx: RAM and ROM
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
  // the Fast RAM did not land in the Falcon space (ST-RAM $000000-$00000F)
  check(mem.mem[0]==64'h4444333322221111 && mem.mem[1]==64'h8888777766665555,
        $sformatf("size %0d: ST-RAM $000000-$00000F unchanged by Fast RAM writes: %016x %016x", fr, mem.mem[0], mem.mem[1]));
  long_rd(32'h00000010);
  check(!got_tea && result[0]==32'hefcdab89, $sformatf("size %0d: ST-RAM $000010 unchanged by Fast RAM writes", fr));
 end
 // transfer lengths (clocks from TS to the last TA, 16 of each): the half-
 // speed bus (Fast RAM off) against CT60 mode's full-speed bus
 for(integer m=0;m<2;m++) begin
  integer tl, tn, fl, fn;
  rst_with(m ? 2'd1 : 2'd0);
  tl=0; tn=0; fl=0; fn=0;
  for(integer r=0;r<16;r++) begin
   long_rd(32'h00001000 + 32'(r*64)); tl+=xclk;
   cpu_xfer(32'h00002000 + 32'(r*64),1,0); tn+=xclk;
   if (m) begin long_rd(32'h01001000 + 32'(r*64)); fl+=xclk; cpu_xfer(32'h01002000 + 32'(r*64),1,0); fn+=xclk; end
  end
  if (m) check(tl >= tl0 && tn >= tn0, $sformatf("CT60 mode: Falcon-space transfers no faster than on the half-speed bus (long %0d >= %0d, line %0d >= %0d, clocks per 16)", tl, tl0, tn, tn0));
  if (m) check(fl < tl0 && fn < tn0, $sformatf("CT60 mode: Fast RAM faster than ST-RAM on the half-speed bus (long %0d < %0d, line %0d < %0d)", fl, tl0, fn, tn0));
  tl0 = tl; tn0 = tn;
  $display("TIMING %s bus: ST-RAM long %0d.%02d, line %0d.%02d clocks%s", m ? "full-speed (CT60)" : "half-speed (Fast RAM off)",
           tl/16, (tl%16)*100/16, tn/16, (tn%16)*100/16, m ? $sformatf("; Fast RAM long %0d.%02d, line %0d.%02d", fl/16, (fl%16)*100/16, fn/16, (fn%16)*100/16) : "");
 end
 rst_with(2'd0);
 // DMA writes to distinct lines while the processor keeps the bus
 for(integer ci=0;ci<4;ci++) begin
  integer count, done, stall, peak;
  count = (ci==0) ? 63 : (ci==1) ? 64 : (ci==2) ? 65 : 200;
  @(negedge clk); reset=1; a_oe=1; bb_oe=1;
  repeat(5) @(negedge clk); reset=0; observed=0; done=0; stall=0; peak=0;
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
   if (dut.sq_used > peak) peak = dut.sq_used;
  end
  $display("QUEUED with the bus kept: writer %s, queue peak %0d, wr=%0d rd=%0d",
           done ? "done" : "held", peak, dut.sq_wr, dut.sq_rd);
  check(count < 60 || !done, $sformatf("%0d DMA lines: the arbiter holds DMA writes before the queue wraps", count));
  @(negedge clk); a_oe=0; bb_oe=0;
  for(integer t=0;t<20000 && !done;t++) @(negedge clk);
  check(done==1, $sformatf("%0d DMA lines: every write completes once the processor releases the bus", count));
  repeat(800) @(negedge clk);
  check(observed==count,$sformatf("snoop delivery after CPU releases ownership: %0d of %0d",observed,count));
 end
 $display("SUMMARY: %0d failed checks",errors);
 if(errors) $fatal(1,"68060 bridge audit checks failed");
 $finish;
end
initial begin #5000000; $fatal(1,"global timeout"); end
endmodule
