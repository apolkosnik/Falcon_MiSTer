`timescale 1ns/1ps
//============================================================================
//  The 68030 bridge's read buffer (falcon_cpubus) against a DMA write that
//  lands in the clock the buffer is refilled (bus audit of 2026-10-12).
//
//  The real tb_top / falcon_system / falcon_cpubus / falcon_l2 /
//  falcon_memarb and the DDR3 model; the CPU is replaced by pin stimulus on
//  its bus (the AP68030 is held in reset and its outputs are forced).
//  The buffer holds line A; a CPU read of line B (an L2 hit) refills it in
//  the clock a disk DMA write to line B is snooped.  The snoop must
//  invalidate the word being loaded: the next CPU read of B returns the
//  DMA's data, not the buffered copy.
//============================================================================
module tb_rb_race;
reg clk = 0;
always #5 clk = ~clk;
reg reset = 1;
reg [1:0] monitor = 2'b10;
reg [31:0] a = 0, d = 0;
reg [2:0] fc = 5;
reg [1:0] siz = 2;
reg rw = 1, as_n = 1, ds_n = 1;
// +falcon: the bus bridge's Falcon mode (Hatari's cycle lengths and Falcon
// time: a cycle started while the blitter owns the bus waits for the blit's
// Hatari time); default: turbo, cycles end when answered
reg turbo;
initial turbo = 0;
reg tstat_en = 0;               // OSD "Timing counters" (falcon_tstat at $FFF000)
tb_top t(.clk(clk), .reset(reset), .cold_reset(reset), .por(reset),
 .ram_mb(4'd14), .ram_tos(1'b0), .monitor(monitor), .cpu_turbo(turbo), .tstat_en(tstat_en),
 .ps2_key(11'd0), .ps2_mouse(25'd0), .joy0(32'd0), .rtc(65'd0),
 .img_mounted(7'd0), .img_readonly(1'b0), .img_size(64'd0),
 .sd_ack(7'd0), .sd_buff_addr(14'd0), .sd_buff_dout(8'd0), .sd_buff_wr(1'b0));
integer checks = 0, failures = 0;
reg berr;
reg [31:0] rd;
task cycle(input [23:0] addr, input wr, input [1:0] size, input [31:0] data);
 integer n;
 begin
  @(negedge clk); a = {8'd0,addr}; rw = !wr; siz = size; d = data; as_n = 0; ds_n = 0;
  n = 0;
  do begin @(posedge clk); #1; n = n + 1; end
  while (t.system.berr_n && t.system.dsack0_n && t.system.dsack1_n && n < (turbo ? 2000 : 200000));
  if (n == (turbo ? 2000 : 200000)) $fatal(1, "Bus timeout at %06x",addr);
  berr = !t.system.berr_n; rd = t.system.cpu_di;
  @(negedge clk); as_n = 1; ds_n = 1;
  repeat (3) @(negedge clk);
 end
endtask
task check(input bit ok, input string msg);
 begin
  checks = checks + 1;
  if (!ok) begin failures = failures + 1; $display("FAIL: %s",msg); end
  else $display("PASS: %s",msg);
 end
endtask
task mapcheck(input [23:0] addr, input [1:0] size, input bit expect_berr, input bit wr);
 begin
  cycle(addr,wr,size,0);
  check(berr == expect_berr,$sformatf("map %s $%06x size=%0d BERR=%0d expected=%0d",wr ? "write":"read",addr,size,berr,expect_berr));
 end
endtask
task wb(input [23:0] addr, input [7:0] data);
 cycle(addr,1,1,{4{data}});
endtask
task ww(input [23:0] addr, input [15:0] data);
 cycle(addr,1,2,{2{data}});
endtask
// guest word at a byte address in the DDR3 model (little endian 64-bit words)
function [15:0] peekw(input [23:0] addr);
 reg [63:0] q;
 begin
  q = t.ddr.mem[addr[23:3]];
  peekw = {q[8*addr[2:0] +: 8], q[8*(addr[2:0]+1) +: 8]};
 end
endfunction
task pokew(input [23:0] addr, input [15:0] v);
 begin
  t.ddr.mem[addr[23:3]][8*addr[2:0] +: 8] = v[15:8];
  t.ddr.mem[addr[23:3]][8*(addr[2:0]+1) +: 8] = v[7:0];
 end
endtask
// one-line blit, x words, HOP source, given LOP, hog mode
task blit(input [23:0] src, input [23:0] dst, input [15:0] words, input [3:0] lop);
 integer n;
 begin
  ww('hff8a20,2); ww('hff8a22,2);
  ww('hff8a24,src[23:16]); ww('hff8a26,src[15:0]);
  ww('hff8a28,'hffff); ww('hff8a2a,'hffff); ww('hff8a2c,'hffff);
  ww('hff8a2e,2); ww('hff8a30,2);
  ww('hff8a32,dst[23:16]); ww('hff8a34,dst[15:0]);
  ww('hff8a36,words); ww('hff8a38,1);
  wb('hff8a3a,2); wb('hff8a3b,{4'd0,lop}); wb('hff8a3d,0);
  wb('hff8a3c,'hc0);
  n = 0;
  repeat (4) @(negedge clk);
  while (t.system.blit_busy && n < 100000) begin @(negedge clk); n = n + 1; end
  if (n == 100000) $fatal(1,"blitter timeout");
  repeat (4) @(negedge clk);
 end
endtask
reg [15:0] w, hi, lo;
integer i;
reg [31:0] c0, c1, r0, r1, f0, f1, d0, d1, g0, g1;
// the counter a high word read returns, on that read's bus_stb clock
reg [31:0] at_hi;
reg [31:0] sysclk = 0;          // system clocks
always @(posedge clk) sysclk <= sysclk + 1;
always @(posedge clk)
 if (t.system.tstat.bus_stb && !t.system.tstat.bus_we && t.system.tstat.bus_addr[1] == 1'b0)
  at_hi <= t.system.tstat.cnt;

reg audit_req=0;
reg [23:1] audit_addr=23'h1000;
initial begin
 force t.system.cpu.reset_n_i = 0;
 force t.system.cpu_a = a;
 force t.system.cpu_do = d;
 force t.system.cpu_fc = fc;
 force t.system.cpu_siz = siz;
 force t.system.cpu_rw = rw;
 force t.system.cpu_as_n = as_n;
 force t.system.cpu_ds_n = ds_n;
 force t.system.cpu_bus_oe = 1;
 force t.system.cpu_reset_oe = 0;
 force t.system.cpu_bg_n = 0;
 force t.system.fdc_dreq = audit_req;
 force t.system.fdc_dwe = 1;
 force t.system.fdc_daddr = audit_addr;
 force t.system.fdc_dbe = 3;
 force t.system.fdc_dwdata = 16'h3333;
 repeat(10) @(negedge clk); reset=0;
 repeat(2200) @(negedge clk);
 pokew('h1000,'h1111); pokew('h2000,'h2222);
 cycle('h2000,0,2,0);
 check(rd[31:16]=='h2222,"prime L2 line B");
 repeat(100) @(negedge clk);
 cycle('h1000,0,2,0);
 check(rd[31:16]=='h1111,"read buffer now holds line A");
 repeat(100) @(negedge clk);
 fork
  cycle('h2000,0,2,0);
  begin
   wait(t.system.l2.st == 1 && t.system.l2.q_addr == ('h2000>>2));
   @(negedge clk);
   $display("INJECT: L2 hit=%0d arbiter idle=%0d old rb_tag=%06x",t.system.l2.hit,t.system.memarb.st==0,{t.system.cpubus.rb_tag,3'b0});
   audit_req=1;
   do begin @(posedge clk); #1; end while(!t.system.fdc_dack);
   @(negedge clk); audit_req=0;
  end
 join
 repeat(100) @(negedge clk);
 check(peekw('h2000)=='h3333,"DMA write has reached DDR3");
 $display("AFTER DMA: rb_valid=%0d rb_tag=%06x rb_data=%016x",t.system.cpubus.rb_valid,{t.system.cpubus.rb_tag,3'b0},t.system.cpubus.rb_data);
 cycle('h2000,0,2,0);
 check(rd[31:16]=='h3333,$sformatf("subsequent CPU read sees completed DMA write: got %04x expected 3333",rd[31:16]));
 cycle('h1000,0,2,0);
 cycle('h2000,0,2,0);
 check(rd[31:16]=='h3333,"evicting read buffer restores correct value");
 $display("SUMMARY: %0d checks, %0d failures",checks,failures);
 if(failures) $fatal(1,"68030 read-buffer audit check failed");
 $finish;
end
always @(posedge clk) if (t.system.cram_ack || t.system.snoop_we)
 $display("TRACE ack=%0d snoop=%0d snoop_addr=%06x rb_tag=%06x la=%06x",t.system.cram_ack,t.system.snoop_we,t.system.snoop_addr,{t.system.cpubus.rb_tag,3'b0},a);
initial begin #10000000; $fatal(1,"global timeout"); end
endmodule
