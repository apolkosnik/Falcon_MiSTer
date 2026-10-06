//============================================================================
//  Falcon ST-RAM / ROM arbiter on the MiSTer DDR3 port
//
//  Guest physical address space (24 bit) is mapped 1:1 onto the DDR3 at
//  byte base 0x30000000: ST-RAM at 0x000000.., TOS ROM at 0xE00000 and the
//  cartridge at 0xFA0000 are all plain DDR3 storage, loaded by the HPS
//  through the loader port while the machine is held in reset.
//
//  Masters, in fixed priority order:
//    vid  video fetch, bursts of four 64-bit words (Videl)
//    ld   ROM/cartridge loader, byte writes (ioctl)
//    d0   DMA port 0 (crossbar: DMA sound play/record)
//    d1   DMA port 1 (FDC DMA)
//    d2   DMA port 2 (blitter)
//    cpu  68030 data/instruction accesses, 32 bit with byte enables
//
//  One command is outstanding at a time.  DMA writes are reported on the
//  snoop port so the CPU data cache never holds stale lines.
//
//  DMA accesses (d0..d2) follow Hatari's memory banks: ST-RAM is read and
//  written directly (SysMem_wget reads the RAM, not the ROM vectors), ROM
//  and cartridge are read only (ROMmem_wput never stores), and anything else
//  is a bus-error region that reads $0000 and ignores writes
//  (STMemory_DMA_ReadWord/WriteWord).  Writes to $000000-$000007 are dropped
//  for the DMA sound and the blitter, which go through SysMem_wput; the disk
//  DMA copies straight into ST-RAM (fdc.c STMemory_SafeCopy, ncr5380.c) and
//  may write there.  A dropped access is acknowledged without a DDR3 cycle.
//  (The system sends the blitter's I/O accesses to the device bus, not here.)
//
//  Byte order: guest data is big endian (lowest address in the most
//  significant byte); the DDR3 is little endian, so 64-bit words are byte
//  reversed at this boundary and nowhere else.
//============================================================================

module falcon_memarb
(
	input             clk,
	input             reset,       // power-on only: the loader runs while the machine is in reset
	input       [3:0] ram_mb,      // ST-RAM size in MB: DMA writes stop at ram_mb * 1 MB

	// video: 4 x 64-bit read bursts
	input             vid_req,
	input      [23:3] vid_addr,
	output reg        vid_ack,
	output reg [63:0] vid_data,
	output reg        vid_valid,

	// loader: byte writes (accepted when ld_busy is low)
	input             ld_wr,
	input      [23:0] ld_addr,
	input       [7:0] ld_data,
	output            ld_busy,

	// 16-bit DMA masters
	input             d0_req,
	input             d0_we,
	input      [23:1] d0_addr,
	input       [1:0] d0_be,
	input      [15:0] d0_wdata,
	output reg [15:0] d0_rdata,
	output reg        d0_ack,

	input             d1_req,
	input             d1_we,
	input      [23:1] d1_addr,
	input       [1:0] d1_be,
	input      [15:0] d1_wdata,
	output reg [15:0] d1_rdata,
	output reg        d1_ack,

	input             d2_req,
	input             d2_we,
	input      [23:1] d2_addr,
	input       [1:0] d2_be,
	input      [15:0] d2_wdata,
	output reg [15:0] d2_rdata,
	output reg        d2_ack,

	// CPU: 32 bit, level request, one-clock acknowledge
	input             cpu_req,
	input             cpu_we,
	input      [23:2] cpu_addr,
	input       [3:0] cpu_be,      // [3] = lowest address (D31..D24)
	input      [31:0] cpu_wdata,
	output reg [31:0] cpu_rdata,
	output reg        cpu_ack,

	// snoop: a non-CPU master wrote RAM
	output reg        snoop_we,
	output reg [23:0] snoop_addr,

	// MiSTer DDRAM interface
	input             DDRAM_BUSY,
	output reg  [7:0] DDRAM_BURSTCNT,
	output reg [28:0] DDRAM_ADDR,
	input      [63:0] DDRAM_DOUT,
	input             DDRAM_DOUT_READY,
	output reg        DDRAM_RD,
	output reg [63:0] DDRAM_DIN,
	output reg  [7:0] DDRAM_BE,
	output reg        DDRAM_WE
);

function [63:0] swap64;
	input [63:0] x;
	swap64 = {x[7:0], x[15:8], x[23:16], x[31:24], x[39:32], x[47:40], x[55:48], x[63:56]};
endfunction

function [7:0] rev8;
	input [7:0] x;
	rev8 = {x[0], x[1], x[2], x[3], x[4], x[5], x[6], x[7]};
endfunction

localparam [28:0] DDR_BASE = 29'h6000000;    // byte 0x30000000 / 8

// loader: one byte write is captured and held until the arbiter takes it
reg        ld_pend = 0;
reg        ld_take;
reg [23:0] ld_a;
reg  [7:0] ld_d;
assign ld_busy = ld_pend;

localparam M_VID = 3'd0, M_LD = 3'd1, M_D0 = 3'd2, M_D1 = 3'd3, M_D2 = 3'd4, M_CPU = 3'd5;

localparam S_IDLE = 2'd0, S_CMD = 2'd1, S_READ = 2'd2;
reg  [1:0] st;
reg  [2:0] owner;
reg  [1:0] sub;         // 16-bit word index or 32-bit half within the 64-bit word
reg  [1:0] beats;
reg        cmd_we;

// the selected master's request, in guest big-endian form
reg        g_we;
reg [23:3] g_addr;
reg [63:0] g_wdata;
reg  [7:0] g_be;        // [7] = lowest address
reg  [2:0] g_owner;
reg  [1:0] g_sub;
reg        g_any;
reg        g_ok;        // 0: a DMA access outside writable/readable memory, no DDR3 cycle

wire [23:0] ram_top = {ram_mb, 20'd0};

always @* begin : sel
	reg [23:0] wa;
	g_any = 1; g_we = 0; g_addr = 0; g_wdata = 0; g_be = 0; g_owner = M_CPU; g_sub = 0; g_ok = 1;
	if (vid_req) begin
		g_owner = M_VID; g_addr = vid_addr; g_be = 8'hFF;
	end else if (ld_pend) begin
		g_owner = M_LD; g_we = 1; g_addr = ld_a[23:3];
		g_wdata = {8{ld_d}}; g_be = 8'h80 >> ld_a[2:0];
	end else if (d0_req) begin
		g_owner = M_D0; g_we = d0_we; g_addr = d0_addr[23:3]; g_sub = d0_addr[2:1];
		g_wdata = {4{d0_wdata}}; g_be = {6'd0, d0_be} << (6 - 2 * d0_addr[2:1]);
	end else if (d1_req) begin
		g_owner = M_D1; g_we = d1_we; g_addr = d1_addr[23:3]; g_sub = d1_addr[2:1];
		g_wdata = {4{d1_wdata}}; g_be = {6'd0, d1_be} << (6 - 2 * d1_addr[2:1]);
	end else if (d2_req) begin
		g_owner = M_D2; g_we = d2_we; g_addr = d2_addr[23:3]; g_sub = d2_addr[2:1];
		g_wdata = {4{d2_wdata}}; g_be = {6'd0, d2_be} << (6 - 2 * d2_addr[2:1]);
	end else if (cpu_req) begin
		g_owner = M_CPU; g_we = cpu_we; g_addr = cpu_addr[23:3]; g_sub = {cpu_addr[2], 1'b0};
		g_wdata = {2{cpu_wdata}}; g_be = cpu_addr[2] ? {4'd0, cpu_be} : {cpu_be, 4'd0};
	end else g_any = 0;

	// memory types for the DMA masters
	wa = {g_addr, g_sub, 1'b0};
	if (g_owner == M_D0 || g_owner == M_D1 || g_owner == M_D2) begin
		if (g_we)
			g_ok = (wa < ram_top) && (wa[23:3] != 21'd0 || g_owner == M_D1);
		else
			g_ok = (wa < ram_top) || (wa[23:19] == 5'b11100) || (wa[23:17] == 7'b1111101);
	end
end

wire [63:0] rd_be = swap64(DDRAM_DOUT);      // guest order

always @(posedge clk) begin
	vid_ack <= 0; vid_valid <= 0;
	d0_ack <= 0; d1_ack <= 0; d2_ack <= 0; cpu_ack <= 0;
	snoop_we <= 0;

	ld_take = 0;

	if (reset) begin
		st <= S_IDLE;
		DDRAM_RD <= 0;
		DDRAM_WE <= 0;
	end
	else case (st)
	S_IDLE:
		// a master that was acknowledged last clock still shows its old
		// request this clock, so never start right after an acknowledge
		if (g_any && !g_ok && !(d0_ack | d1_ack | d2_ack | cpu_ack)) begin
			// bus-error region, ROM write or reset-vector write: no DDR3 cycle
			case (g_owner)
				M_D0:    begin d0_rdata <= 16'h0000; d0_ack <= 1; end
				M_D1:    begin d1_rdata <= 16'h0000; d1_ack <= 1; end
				default: begin d2_rdata <= 16'h0000; d2_ack <= 1; end
			endcase
		end
		else if (g_any && !(d0_ack | d1_ack | d2_ack | cpu_ack)) begin
			owner      <= g_owner;
			sub        <= g_sub;
			cmd_we     <= g_we;
			DDRAM_ADDR <= DDR_BASE | {8'd0, g_addr};
			DDRAM_BURSTCNT <= (g_owner == M_VID) ? 8'd4 : 8'd1;
			DDRAM_DIN  <= swap64(g_wdata);
			DDRAM_BE   <= g_we ? rev8(g_be) : 8'hFF;
			DDRAM_WE   <= g_we;
			DDRAM_RD   <= !g_we;
			beats      <= 0;
			st         <= S_CMD;
			if (g_owner == M_VID) vid_ack <= 1;
			if (g_owner == M_LD)  ld_take = 1;
			if (g_we && g_owner != M_CPU && g_owner != M_LD) begin
				snoop_we   <= 1;
				snoop_addr <= {g_addr, g_sub, 1'b0};
			end
		end

	S_CMD:
		if (!DDRAM_BUSY) begin
			DDRAM_RD <= 0;
			DDRAM_WE <= 0;
			if (cmd_we) begin
				st <= S_IDLE;
				case (owner)
					M_D0:  d0_ack  <= 1;
					M_D1:  d1_ack  <= 1;
					M_D2:  d2_ack  <= 1;
					M_CPU: cpu_ack <= 1;
					default: ;
				endcase
			end
			else st <= S_READ;
		end

	S_READ:
		if (DDRAM_DOUT_READY) begin
			case (owner)
				M_VID: begin
					vid_data  <= rd_be;
					vid_valid <= 1;
					beats     <= beats + 1'd1;
					if (beats == 2'd3) st <= S_IDLE;
				end
				M_D0:  begin d0_rdata <= rd_be[63 - 16 * sub -: 16]; d0_ack <= 1; st <= S_IDLE; end
				M_D1:  begin d1_rdata <= rd_be[63 - 16 * sub -: 16]; d1_ack <= 1; st <= S_IDLE; end
				M_D2:  begin d2_rdata <= rd_be[63 - 16 * sub -: 16]; d2_ack <= 1; st <= S_IDLE; end
				default: begin cpu_rdata <= sub[1] ? rd_be[31:0] : rd_be[63:32]; cpu_ack <= 1; st <= S_IDLE; end
			endcase
		end

	default: st <= S_IDLE;
	endcase

	// a new loader byte wins over the clear of the one just taken
	if (ld_wr) begin
		ld_pend <= 1; ld_a <= ld_addr; ld_d <= ld_data;
	end
	else if (ld_take) ld_pend <= 0;
end

endmodule
