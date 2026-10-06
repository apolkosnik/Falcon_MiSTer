//============================================================================
//  Falcon 68030 bus bridge
//
//  Terminates the AP68030's pin-level bus cycles (MC68030UM section 7):
//
//    RAM / ROM / cartridge   32-bit port (DSACK1+DSACK0), served by the
//                            DDR3 arbiter.  The Falcon's real ST-RAM bus is
//                            16 bits wide; the wider port only saves cycles.
//    I/O and IDE             16-bit port (DSACK1), on the common device bus
//                            of docs/ARCHITECTURE.md.  D31..D24 is the even
//                            byte, D23..D16 the odd byte.
//    interrupt acknowledge   vectored (vector on every byte lane) or AVEC
//    other CPU space         bus error (no FPU: coprocessor cycles end in
//                            BERR and the CPU takes the F-line exception)
//
//  Decoding follows the Falcon's 24-bit address bus (A31..A24 ignored):
//    000000-000007  reads: ROM (reset vectors), writes: bus error.  A RAM TOS
//                   (TOS 4.92: a 34-byte loader, then an image linked for
//                   ST-RAM, Hatari tos.c) has loader code where the vectors
//                   would be; for it the vectors read SSP $8000 and PC
//                   $E00000, so the real loader copies the image to its RAM
//                   address and jumps to it, as when booted from disk.
//    000008-ram_top ST-RAM, above: bus error
//    E00000-E7FFFF  ROM, read only
//    FA0000-FBFFFF  cartridge, read only
//    F00000-F0FFFF, FF8000-FFFFFF  device bus (the system decides which
//                   addresses exist; it answers dev_berr for the others)
//    everything else: bus error
//  Supervisor only: 000000-0007FF and FF8000-FFFFFF.
//
//  Timing: every decision is taken on the rising edge.  The CPU samples
//  DSACKx/BERR/AVEC on falling edges and latches read data with them, so
//  the outputs here are registered and held until AS negates.
//============================================================================

module falcon_cpubus
(
	input             clk,
	input             reset,

	input       [3:0] ram_mb,      // ST-RAM size in MB (4 or 14)
	input             ram_tos,     // the TOS image is a RAM TOS behind its loader (TOS 4.92)

	// 68030 pins
	input      [31:0] a,
	input       [2:0] fc,
	input       [1:0] siz,
	input             rw,
	input             as_n,
	input             ds_n,
	input             bus_oe,
	input      [31:0] d_o,
	output reg [31:0] d_i,
	output reg        dsack0_n,
	output reg        dsack1_n,
	output reg        berr_n,
	output reg        avec_n,
	output reg        ciin_n,

	// memory arbiter, CPU port
	output reg        ram_req,
	output reg        ram_we,
	output reg [23:2] ram_addr,
	output reg  [3:0] ram_be,
	output reg [31:0] ram_wdata,
	input      [31:0] ram_rdata,
	input             ram_ack,

	// device bus (decoded further by the system)
	output reg        dev_cs,
	output reg        dev_stb,
	output reg        dev_we,
	output reg [23:1] dev_addr,
	output reg        dev_uds,
	output reg        dev_lds,
	output reg [15:0] dev_din,
	input      [15:0] dev_dout,
	input             dev_ack,
	input             dev_berr,
	output            dev_super,   // the access is in supervisor mode

	// interrupt acknowledge
	output reg        iack_req,    // one clock
	output reg  [2:0] iack_level,
	input             iack_done,   // one clock, with one of:
	input             iack_avec,
	input             iack_spur,   //   no source: bus error (spurious)
	input       [7:0] iack_vector,

	output reg        cycle_done   // one clock per completed CPU bus cycle
);

localparam S_IDLE = 3'd0, S_RAM = 3'd1, S_DEV = 3'd2, S_IACK = 3'd3, S_HOLD = 3'd4;
reg  [2:0] st;
reg  [9:0] tmo;

wire as_act = ~as_n & bus_oe;
wire [23:0] la = a[23:0];
wire is_super = fc[2];
assign dev_super = is_super;

// bytes of the operand in this cycle on a 32-bit port (UM table 7-4)
reg  [3:0] be32;
reg  [2:0] nbytes;
always @* begin
	nbytes = (siz == 2'b01) ? 3'd1 : (siz == 2'b10) ? 3'd2 : (siz == 2'b11) ? 3'd3 : 3'd4;
	case (la[1:0])
		2'd0: be32 = (nbytes >= 3'd4) ? 4'b1111 : (nbytes == 3'd3) ? 4'b1110 : (nbytes == 3'd2) ? 4'b1100 : 4'b1000;
		2'd1: be32 = (nbytes >= 3'd3) ? 4'b0111 : (nbytes == 3'd2) ? 4'b0110 : 4'b0100;
		2'd2: be32 = (nbytes >= 3'd2) ? 4'b0011 : 4'b0010;
		default: be32 = 4'b0001;
	endcase
end

wire [23:0] ram_top   = {ram_mb, 20'd0};
wire is_vec    = (la[23:3] == 21'd0);
wire is_ram    = (la < ram_top);
wire is_rom    = (la[23:19] == 5'b11100);           // E00000-E7FFFF
wire is_cart   = (la[23:17] == 7'b1111101);         // FA0000-FBFFFF
wire is_ide    = (la[23:16] == 8'hF0);
wire is_io     = (la[23:15] == 9'b111111111);       // FF8000-FFFFFF
wire is_prot   = (la[23:11] == 13'd0) || is_io;
wire cpu_space = (fc == 3'd7);

// RAM/ROM address actually used: the reset vectors come from ROM
function [23:2] mem_addr;
	input [23:0] x; input rd;
	mem_addr = (rd && x[23:3] == 21'd0) ? (22'h380000 | {21'd0, x[2]}) : x[23:2];
endfunction

task finish_berr;
	begin berr_n <= 0; st <= S_HOLD; end
endtask

always @(posedge clk) begin
	dev_stb    <= 0;
	iack_req   <= 0;
	cycle_done <= 0;

	if (reset) begin
		st <= S_IDLE;
		dsack0_n <= 1; dsack1_n <= 1; berr_n <= 1; avec_n <= 1; ciin_n <= 1;
		ram_req <= 0; dev_cs <= 0;
	end
	else case (st)
	S_IDLE:
		// writes start once DS shows the data is on the bus
		if (as_act && (rw || !ds_n)) begin
			tmo <= 0;
			if (cpu_space) begin
				if (la[19:16] == 4'hF) begin
					iack_req   <= 1;
					iack_level <= la[3:1];
					st         <= S_IACK;
				end
				else finish_berr;          // breakpoint, coprocessor, MMU access level
			end
			else if (is_prot && !is_super) finish_berr;
			else if (is_vec && !rw) finish_berr;   // SysMem_*put: the reset vectors are ROM
			else if (is_vec && rw && ram_tos) begin
				d_i      <= la[2] ? 32'h00E00000 : 32'h00008000;
				dsack0_n <= 0;
				dsack1_n <= 0;
				st       <= S_HOLD;
			end
			else if ((is_vec && rw) || is_ram || ((is_rom || is_cart) && rw)) begin
				ram_req   <= 1;
				ram_we    <= !rw;
				ram_addr  <= mem_addr(la, rw);
				ram_be    <= be32;
				ram_wdata <= d_o;
				st        <= S_RAM;
			end
			else if (is_ide || is_io) begin
				dev_cs   <= 1;
				dev_stb  <= 1;
				dev_we   <= !rw;
				dev_addr <= la[23:1];
				dev_uds  <= !la[0];
				dev_lds  <= la[0] || (nbytes != 3'd1);
				dev_din  <= d_o[31:16];
				ciin_n   <= 0;                    // I/O is never cached
				st       <= S_DEV;
			end
			else finish_berr;
		end

	S_RAM:
		if (ram_ack) begin
			ram_req  <= 0;
			d_i      <= ram_rdata;
			dsack0_n <= 0;
			dsack1_n <= 0;
			st       <= S_HOLD;
		end

	S_DEV: begin
		tmo <= tmo + 1'd1;
		if (dev_berr || tmo == 10'h3FF) begin
			dev_cs <= 0;
			finish_berr;
		end
		else if (dev_ack) begin
			dev_cs   <= 0;
			d_i      <= {dev_dout, dev_dout};
			dsack1_n <= 0;
			st       <= S_HOLD;
		end
	end

	S_IACK:
		if (iack_done) begin
			if (iack_spur) berr_n <= 0;
			else if (iack_avec) avec_n <= 0;
			else begin
				d_i <= {4{iack_vector}};
				dsack0_n <= 0;
				dsack1_n <= 0;
			end
			st <= S_HOLD;
		end

	S_HOLD:
		if (!as_act) begin
			dsack0_n <= 1; dsack1_n <= 1; berr_n <= 1; avec_n <= 1; ciin_n <= 1;
			cycle_done <= 1;
			st <= S_IDLE;
		end

	default: st <= S_IDLE;
	endcase
end

endmodule
