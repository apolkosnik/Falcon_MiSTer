`timescale 1ns/1ps
//============================================================================
//  DMA memory types in the real falcon_memarb on the DDR3 model, driven at
//  its DMA ports: d0 DMA sound, d1 disk DMA (FDC/SCSI), d2 blitter.
//
//  Expected results: Hatari 028dbf7f.  ST-RAM is read/written directly; ROM
//  is never stored (ROMmem_wput); bus-error regions read $0000 and drop
//  writes (STMemory_DMA_ReadWord/WriteWord).  Below $000008 reads see the
//  RAM backing store (SysMem_wget); sound and blitter writes there are
//  dropped (SysMem_wput), the disk DMA writes them (fdc.c FDC_DMA_FIFO_Push
//  STMemory_SafeCopy, FDC_DMA_FIFO_Pull memcpy from STRam).
//  The first 27 checks are the 2026-10-03 fix review's reproductions.
//============================================================================
module tb_memarb;
reg clk = 0;
always #5 clk = ~clk;
reg reset = 1;
reg [3:0] ram_mb = 4;
reg [2:0] req = 0;
reg we = 0;
reg [23:1] addr = 0;
reg [15:0] data = 0;
wire [2:0] ack;
wire [15:0] rd[3];
wire busy, dout_ready, ddr_rd, ddr_we;
wire [7:0] burstcnt, be;
wire [28:0] ddr_addr;
wire [63:0] din, dout;
wire snoop_we;
falcon_memarb dut(
 .clk(clk), .reset(reset), .ram_mb(ram_mb),
 .vid_req(1'b0), .vid_addr(21'd0), .ld_wr(1'b0), .ld_addr(24'd0), .ld_data(8'd0),
 .d0_req(req[0]), .d0_we(we), .d0_addr(addr), .d0_be(2'b11), .d0_wdata(data), .d0_rdata(rd[0]), .d0_ack(ack[0]),
 .d1_req(req[1]), .d1_we(we), .d1_addr(addr), .d1_be(2'b11), .d1_wdata(data), .d1_rdata(rd[1]), .d1_ack(ack[1]),
 .d2_req(req[2]), .d2_we(we), .d2_addr(addr), .d2_be(2'b11), .d2_wdata(data), .d2_rdata(rd[2]), .d2_ack(ack[2]),
 .cpu_req(1'b0), .cpu_we(1'b0), .cpu_addr(22'd0), .cpu_be(4'd0), .cpu_wdata(32'd0),
 .snoop_we(snoop_we), .DDRAM_BUSY(busy), .DDRAM_BURSTCNT(burstcnt), .DDRAM_ADDR(ddr_addr),
 .DDRAM_DOUT(dout), .DDRAM_DOUT_READY(dout_ready), .DDRAM_RD(ddr_rd),
 .DDRAM_DIN(din), .DDRAM_BE(be), .DDRAM_WE(ddr_we));
ddr3_model mem(.clk(clk), .DDRAM_BUSY(busy), .DDRAM_BURSTCNT(burstcnt), .DDRAM_ADDR(ddr_addr),
 .DDRAM_DOUT(dout), .DDRAM_DOUT_READY(dout_ready), .DDRAM_RD(ddr_rd),
 .DDRAM_DIN(din), .DDRAM_BE(be), .DDRAM_WE(ddr_we));
integer checks = 0, errors = 0;
reg [15:0] result;
task check(input bit ok, input string msg);
 begin checks++; if (!ok) begin errors++; $display("FAIL: %s",msg); end else $display("PASS: %s",msg); end
endtask
function [15:0] peek(input [23:0] a);
 reg [63:0] q;
 begin q=mem.mem[a[23:3]]; peek={q[8*a[2:0]+:8],q[8*(a[2:0]+1)+:8]}; end
endfunction
task poke(input [23:0] a, input [15:0] v);
 begin mem.mem[a[23:3]][8*a[2:0]+:8]=v[15:8]; mem.mem[a[23:3]][8*(a[2:0]+1)+:8]=v[7:0]; end
endtask
task access(input integer master, input bit write_en, input [23:0] a, input [15:0] d);
 integer n;
 begin
  @(negedge clk); req=1<<master; we=write_en; addr=a[23:1]; data=d; n=0;
  do begin @(posedge clk); #1; n++; end while (!ack[master] && n<1000);
  if (n==1000) $fatal(1,"DMA timed out");
  result=rd[master]; @(negedge clk); req=0; repeat(3) @(negedge clk);
 end
endtask
integer m, cfg;
initial begin
 repeat(5) @(negedge clk); reset=0;
 // Regression controls: allowed RAM, blocked ROM, blocked unmapped space,
 // correct ram_mb boundary and read/write transition behavior, all masters.
 for (cfg=0; cfg<2; cfg++) begin
  ram_mb=cfg ? 14:4;
  for(m=0;m<3;m++) begin
   access(m,1,24'h001000,16'h1234+m);
   access(m,0,24'h001000,0);
   check(result==16'h1234+m,$sformatf("%0dMB DMA%0d RAM read/write",ram_mb,m));
   poke('he00100,'hcafe); access(m,1,'he00100,'hffff);
   check(peek('he00100)=='hcafe,$sformatf("%0dMB DMA%0d ROM write blocked",ram_mb,m));
   poke('hf10000,'hcafe); access(m,1,'hf10000,'hffff);
   access(m,0,'hf10000,0);
   check(peek('hf10000)=='hcafe && result==0,$sformatf("%0dMB DMA%0d unmapped access",ram_mb,m));
   poke('h400000,'hcafe); access(m,1,'h400000,'hbeef); access(m,0,'h400000,0);
   check(cfg ? result=='hbeef : (result==0 && peek('h400000)=='hcafe),$sformatf("%0dMB DMA%0d RAM boundary",ram_mb,m));
  end
 end
 // Disk DMA can access RAM at 0; CPU and blitter write protection differs.
 poke(0,'h5555); poke('he00000,'h1234);
 access(1,0,0,0);
 check(result=='h5555,$sformatf("FDC DMA read at zero: %04x expected RAM 5555",result));
 access(1,1,0,'hbeef);
 check(peek(0)=='hbeef,$sformatf("FDC DMA write at zero: backing=%04x expected beef",peek(0)));
 access(1,1,8,'hcafe);
 check(peek(8)=='hcafe,"FDC DMA next word at eight still writes RAM");
 // reads below 8 see the RAM for every master; only the disk DMA writes there
 for(m=0;m<3;m++) begin
  poke(2,'h6666); poke('he00002,'h4321);
  access(m,0,2,0);
  check(result=='h6666,$sformatf("DMA%0d read at two: %04x expected RAM 6666",m,result));
  access(m,1,2,'h7777);
  check(peek(2)==(m==1 ? 'h7777 : 'h6666),$sformatf("DMA%0d write at two: backing=%04x expected %04x",m,peek(2),m==1 ? 'h7777 : 'h6666));
  check(peek('he00002)=='h4321,$sformatf("DMA%0d access at two leaves the ROM alone",m));
 end
 // a dropped access never shows up on the snoop port
 poke('h1000,0);
 fork
  access(2,1,'he00100,'h1111);
  begin repeat(40) begin @(posedge clk); #1; if (snoop_we) errors++; end end
 join
 checks++;
 $display("SUMMARY: %0d checks, %0d failures",checks,errors);
 if(errors) $fatal(1,"DMA memory type test failed");
 $finish;
end
endmodule
