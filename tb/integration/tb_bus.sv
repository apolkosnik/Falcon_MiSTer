`timescale 1ns/1ps
//============================================================================
//  System bus integration test: address map, CPU bridge, DMA memory types,
//  blitter I/O accesses and the blitter interrupt line.
//
//  The real tb_top / falcon_system / falcon_cpubus / falcon_memarb / chipset
//  and the DDR3 model; only the CPU is replaced, by pin stimulus on its bus
//  (the AP68030 is held in reset and its outputs are forced).
//
//  Expected results: Hatari 028dbf7f (framagit.org/hatari/hatari):
//  ioMemTabFalcon.c, ioMem.c, cpu/memory.c SysMem_*put and ROMmem_*put,
//  stMemory.c STMemory_DMA_ReadWord/WriteWord, blitter.c Blitter_Start.
//  The first 36 checks are the 2026-10-03 audit's reproductions.
//============================================================================
module tb_bus;
reg clk = 0;
always #5 clk = ~clk;
reg reset = 1;
reg [1:0] monitor = 2'b10;
reg [31:0] a = 0, d = 0;
reg [2:0] fc = 5;
reg [1:0] siz = 2;
reg rw = 1, as_n = 1, ds_n = 1;
tb_top t(.clk(clk), .reset(reset), .cold_reset(reset), .por(reset),
 .ram_mb(4'd14), .ram_tos(1'b0), .monitor(monitor),
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
  while (t.system.berr_n && t.system.dsack0_n && t.system.dsack1_n && n < 2000);
  if (n == 2000) $fatal(1, "Bus timeout at %06x",addr);
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
reg [15:0] w;
integer i;
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
 repeat (10) @(negedge clk);
 reset = 0;
 repeat (10) @(negedge clk);
 $display("Native Falcon map");
 mapcheck('hff8000,1,0,0);
 mapcheck('hff8001,1,0,0);
 mapcheck('hff8002,1,1,0);
 mapcheck('hff8004,2,1,0);
 mapcheck('hff8008,2,1,1);
 mapcheck('hff800a,2,1,0);
 mapcheck('hff800c,2,0,0);
 mapcheck('hff800e,2,1,0);
 $display("STE-compatible map");
 wb('hff8007,'h05);
 mapcheck('hff8002,1,0,0);
 mapcheck('hff8560,1,0,0);
 mapcheck('hff8561,1,1,0);
 mapcheck('hff8564,1,0,0);
 mapcheck('hff8565,1,1,0);
 mapcheck('hffc021,1,0,0);
 mapcheck('hffd020,1,0,0);
 mapcheck('hffd021,1,1,0);
 mapcheck('hffd420,1,0,0);
 mapcheck('hffd421,1,1,0);
 mapcheck('hffd424,1,1,0);
 mapcheck('hffd425,1,0,0);
 mapcheck('hffd074,2,0,0);
 mapcheck('hffd520,2,0,1);
 mapcheck('hffd530,2,0,0);
 mapcheck('hffd074,1,1,0);
 wb('hff8007,'h25);
 $display("Protected reset vector writes");
 mapcheck(0,1,1,1);
 mapcheck(4,2,1,1);
 mapcheck(7,1,1,1);
 mapcheck(8,2,0,1);
 $display("Blitter/MFP integration");
 wb('hfffa03,0);   // Falling edge on GPIP3
 wb('hfffa09,8);   // IERB: GPIP3
 wb('hfffa15,8);   // IMRB: GPIP3
 wb('hfffa0d,0);   // Clear pending
 ww('hff8a20,2); ww('hff8a22,2);
 ww('hff8a24,0); ww('hff8a26,'h8000);
 ww('hff8a28,'hffff); ww('hff8a2a,'hffff); ww('hff8a2c,'hffff);
 ww('hff8a2e,2); ww('hff8a30,2);
 ww('hff8a32,1); ww('hff8a34,0);
 ww('hff8a36,256); ww('hff8a38,1);
 wb('hff8a3a,2); wb('hff8a3b,3);
 wb('hff8a3c,'hc0);
 repeat (20) @(negedge clk);
 check(t.system.blit_busy == 1,"blitter transfer is active");
 check(t.system.mfp.gpip_in[3] == 1,"GPIP3 high while blitter busy (Hatari)");
 check(t.system.mfp_irq == 0,"falling-edge GPIP3 must not interrupt at blit START");
 // Clear the premature pending bit while the long blit is still active.
 wb('hfffa0d,0);
 i = 0;
 while (t.system.blit_busy && i < 100000) begin @(negedge clk); i = i+1; end
 if (i == 100000) $fatal(1,"blitter timeout");
 repeat (20) @(negedge clk);
 check(t.system.mfp.gpip_in[3] == 0,"GPIP3 low on blitter completion (Hatari)");
 check(t.system.mfp_irq == 1,"falling-edge GPIP3 must interrupt at blit END");
 $display("DMA destination protection");
 // All-ones one-word blit, no source or destination read needed.
 // Hatari ROMmem_wput never modifies the ROM backing storage.
 cycle('he00100,0,2,0);
 check(rd[31:16] == 0,"ROM test location starts at zero");
 ww('hff8a32,'h00e0); ww('hff8a34,'h0100);
 ww('hff8a36,1); ww('hff8a38,1);
 wb('hff8a3a,0); wb('hff8a3b,15);
 wb('hff8a3c,'hc0);
 repeat (100) @(negedge clk);
 check(t.system.blit_busy == 0,"one-word ROM blit completed");
 cycle('he00100,0,2,0);
 check(rd[31:16] == 0,$sformatf("blitter must preserve ROM: read=%04x expected=0000",rd[31:16]));
 check(peekw('he00100) == 0,"ROM backing store unchanged after the blit");

 $display("DMA memory types");
 // reset vectors: blitter reads see the RAM (SysMem_wget), writes are dropped
 pokew('he00000,'h1234); pokew(0,'h5555); pokew(2,'h6666); pokew('h8,'h0000);
 blit(0,'h20020,1,3);
 check(peekw('h20020) == 'h5555,$sformatf("blitter read of $000000 sees the RAM: %04x expected 5555",peekw('h20020)));
 blit(0,2,1,15);
 check(peekw(2) == 'h6666,$sformatf("DMA write to $000002 dropped: %04x expected 6666",peekw(2)));
 blit(0,'h8,1,15);
 check(peekw('h8) == 'hffff,$sformatf("DMA write to $000008 reaches RAM: %04x expected ffff",peekw('h8)));
 // cartridge is read only
 pokew('hfa0010,'hc0de);
 blit(0,'hfa0010,1,15);
 check(peekw('hfa0010) == 'hc0de,$sformatf("DMA write to the cartridge dropped: %04x expected c0de",peekw('hfa0010)));
 blit('hfa0010,'h20022,1,3);
 check(peekw('h20022) == 'hc0de,$sformatf("DMA read of the cartridge: %04x expected c0de",peekw('h20022)));
 // bus-error region: reads $0000 (DMA_READ_WORD_BUS_ERR), writes dropped
 pokew('hf10000,'h7777); pokew('h20010,'h5a5a);
 blit('hf10000,'h20010,1,3);
 check(peekw('h20010) == 'h0000,$sformatf("DMA read of bus-error region $F10000: %04x expected 0000",peekw('h20010)));
 blit(0,'hf10000,1,15);
 check(peekw('hf10000) == 'h7777,$sformatf("DMA write to bus-error region dropped: %04x expected 7777",peekw('hf10000)));
 // RAM to RAM still copies, through the CPU bridge view as well
 pokew('h30000,'hbeef); pokew('h30002,'hcafe); pokew('h30004,'h0102);
 blit('h30000,'h31000,3,3);
 check(peekw('h31000) == 'hbeef && peekw('h31002) == 'hcafe && peekw('h31004) == 'h0102,
       $sformatf("RAM copy: %04x %04x %04x",peekw('h31000),peekw('h31002),peekw('h31004)));
 cycle('h31000,0,2,0);
 check(rd[31:16] == 'hbeef,$sformatf("CPU reads the blitted RAM: %04x expected beef",rd[31:16]));

 $display("Blitter I/O accesses");
 // RAM -> ST palette ($FF8240.., 12 bits read back)
 pokew('h10000,'h0123); pokew('h10002,'h0456); pokew('h10004,'h0789); pokew('h10006,'h0abc);
 ww('hff8240,0); ww('hff8242,0); ww('hff8244,0); ww('hff8246,0);
 blit('h10000,'hff8240,4,3);
 for (i = 0; i < 4; i = i + 1) begin
  cycle('hff8240 + 2*i,0,2,0);
  w = (i == 0) ? 'h0123 : (i == 1) ? 'h0456 : (i == 2) ? 'h0789 : 'h0abc;
  check(rd[31:16] == w,$sformatf("blit to palette $%06x: %04x expected %04x",'hff8240 + 2*i,rd[31:16],w));
 end
 // ST palette -> RAM
 pokew('h20000,0); pokew('h20002,0); pokew('h20004,0); pokew('h20006,0);
 blit('hff8240,'h20000,4,3);
 check(peekw('h20000) == 'h0123 && peekw('h20002) == 'h0456 && peekw('h20004) == 'h0789 && peekw('h20006) == 'h0abc,
       $sformatf("blit from palette: %04x %04x %04x %04x",peekw('h20000),peekw('h20002),peekw('h20004),peekw('h20006)));
 // I/O that bus errors: reads $0000, the write goes nowhere
 pokew('h20030,'h5a5a);
 blit('hff8004,'h20030,1,3);
 check(peekw('h20030) == 'h0000,$sformatf("blit from bus-error I/O $FF8004: %04x expected 0000",peekw('h20030)));
 blit('h10000,'hff8004,1,3);
 check(t.system.blit_busy == 0,"blit to bus-error I/O completes");
 // the I/O space is not DDR3 storage
 check(peekw('hff8240) == 0 && peekw('hff8004) == 0,"blitter I/O accesses left the DDR3 copy of the I/O area alone");

 $display("NVRAM preservation through system wiring");
 wb('hff8961,20); wb('hff8963,3);
 wb('hff8961,21); wb('hff8963,4);
 wb('hff8961,24); wb('hff8963,5);
 monitor = 2'b01;
 repeat (300) @(negedge clk);
 wb('hff8961,20); cycle('hff8962,0,2,0);
 check(rd[23:16] == 3,"monitor change preserves NVRAM language through falcon_system");
 wb('hff8961,21); cycle('hff8962,0,2,0);
 check(rd[23:16] == 4,"monitor change preserves NVRAM keyboard layout");
 reset = 1;
 repeat (20) @(negedge clk);
 reset = 0;
 repeat (100) @(negedge clk);
 wb('hff8961,24); cycle('hff8962,0,2,0);
 check(rd[23:16] == 5,"cold reset preserves NVRAM boot delay");

 $display("SUMMARY: %0d checks, %0d failures",checks,failures);
 if (failures) $fatal(1,"bus integration test failed");
 $finish;
end
endmodule
