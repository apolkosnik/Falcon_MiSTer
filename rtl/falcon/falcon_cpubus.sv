//============================================================================
//  Falcon 68030 bus bridge
//
//  Terminates the AP68030's pin-level bus cycles (MC68030UM section 7):
//
//    RAM / ROM / cartridge   served by the DDR3 arbiter: in turbo a 32-bit
//                            port (DSACK1+DSACK0), in the Falcon mode a
//                            16-bit port (DSACK1) like the Falcon's ST-RAM
//                            and ROM bus.
//    I/O and IDE             16-bit port (DSACK1), on the common device bus
//                            of docs/ARCHITECTURE.md.  D31..D24 is the even
//                            byte, D23..D16 the odd byte.
//    interrupt acknowledge   vectored (vector on every byte lane) or AVEC
//    coprocessor (CPU space  passed to the FPU bridge (cp_* port): the
//      type 2, A19-16=0010)  word CIRs answer as a 16-bit port (DSACK1,
//                            data on D31..D16) like the MC68881/882, the
//                            operand, instruction address and operand
//                            address CIRs ($10/$18/$1C) as a 32-bit port;
//                            the bridge may ask for BERR instead (no FPU)
//    other CPU space         bus error (breakpoint, MMU access level)
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
//
//  Two modes (fmode, taken from fmode_in between cycles):
//
//  turbo (fmode 0): a cycle ends as soon as memory or the device answers;
//  writes start once DS shows the data.
//
//  Falcon (fmode 1, docs/CPU_TIMING.md): every cycle takes exactly the
//  processor clocks of Hatari's Falcon model (its "cycle exact" 68030,
//  cpu/custom.c, cpu/newcpu.c mem_access_delay_*_ce020):
//    ST-RAM, IDE, unmapped (CHIP16)   3 clocks, plus 2 when the cycle starts
//                                     at clock position 2 or 3 mod 4 (the
//                                     4-clock slot, custom.c:329-359)
//    ROM, cartridge, I/O (FAST16)     3 clocks, plus the device's wait states
//                                     (M68000_WaitState): MFP registers +4
//                                     (mfp.c), YM2149 +4 on the first access
//                                     of an instruction and on every 4th
//                                     further one (psg.c:477-503), FDC/DMA
//                                     $FF8604 +4 (reads: not with the sector
//                                     count selected), $FF8606 writes +4,
//                                     $FF860E +4 (fdc.c), ACIA 6 plus the
//                                     E clock (to a multiple of 10) on the
//                                     first ACIA access of an instruction
//                                     (acia.c:546-561), DSP host port +4 per
//                                     byte after the first (falcon/dsp.c)
//    interrupt acknowledge            MFP 12 clocks, autovector (VBL/HBL) the
//                                     E clock then 10, others 3 (iack_cycle,
//                                     newcpu.c:2901-3029)
//    coprocessor, bus errors          3 (bus errors: plus the slot rule in
//                                     CHIP16 space)
//  A long is two 16-bit cycles as in Hatari; a word at an odd address is
//  two byte cycles on the 68030's 16-bit port, one access in Hatari, and
//  the chip's behaviour is kept.  The cycle length is set by when DSACK
//  (BERR, AVEC) becomes visible: for w wait states at the falling edge after
//  the (w+1)th processor rising edge of the cycle.  When the answer is not
//  there by then, `hold` stops the processor's clock (falcon_cpuclk) instead
//  of letting it add wait states; Falcon time keeps running as debt.  Wait
//  states that could not be avoided are reported as `credit` (none are
//  expected: the edge after S0 is held on the clock the cycle is first seen).
//  RAM writes are posted (one entry, written to memory in order before any
//  later RAM read or device access); RAM/ROM reads go through a read-ahead
//  buffer that keeps the last 64-bit DDR3 word (kept coherent with the
//  posted writes, invalidated by other masters' writes and by the loader),
//  so sequential fetches need one DDR3 read per four words.  The data of a
//  write is taken at the start of the cycle: the AP68030 drives its write
//  lanes from S0 (ap030_bus wlanes).
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

	// coprocessor interface registers (falcon_fpu_bridge)
	output reg        cp_req,      // one clock, with the fields below
	output reg        cp_we,
	output reg  [2:0] cp_id,       // A15..A13
	output reg  [4:0] cp_off,      // CIR select, A4..A0
	output reg  [1:0] cp_siz,
	output reg [31:0] cp_wdata,
	input             cp_ack,      // one clock, with:
	input             cp_berr,     //   end the cycle with BERR
	input      [31:0] cp_rdata,    //   word CIRs on [31:16]

	output reg        cycle_done,  // one clock per completed CPU bus cycle

	// Falcon bus timing (see above)
	input             fmode_in,    // 1: Falcon timing, 0: turbo
	output reg        fmode,       // the mode in use (changes between cycles)
	input             cpu_ce,      // the processor's clock enable (falcon_cpuclk)
	input       [4:0] tpos,        // processor clocks before this clock, mod 20
	input             inst,        // an instruction was dispatched (with cpu_ce)
	output            hold,        // no processor clock now
	output reg  [3:0] credit,      // processor clocks of wait states that could not be avoided
	output            wbuf_busy,   // a posted RAM write is not in memory yet
	input      [63:0] ram_rdata64, // the whole DDR3 word of a read, with ram_ack
	input             snoop_we,    // another master wrote RAM
	input      [23:0] snoop_addr,
	input             buf_flush,   // the loader writes memory
	input             iack_mfp     // with iack_done: the vector came from the MFP
);

localparam S_IDLE = 3'd0, S_RAM = 3'd1, S_DEV = 3'd2, S_IACK = 3'd3, S_HOLD = 3'd4, S_CP = 3'd5;
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

//----------------------------------------------------------------------------
// Falcon mode
//----------------------------------------------------------------------------
localparam F_IDLE = 2'd0, F_RUN = 2'd1, F_HOLD = 2'd2;
localparam J_NONE = 3'd0, J_RD = 3'd1, J_WR = 3'd2, J_DEV = 3'd3, J_DEVW = 3'd4, J_IACK = 3'd5, J_CP = 3'd6;
localparam K_D16 = 2'd0, K_D32 = 2'd1, K_BERR = 2'd2, K_AVEC = 2'd3;
reg  [1:0] fst;
reg  [2:0] fjob;
reg  [1:0] fkind;          // how the cycle ends
reg        fready;         // the answer (data, vector, error) is there
reg        fplan;          // fw is known (an acknowledge's length depends on its source)
reg  [4:0] fk;             // processor rising edges of this cycle seen so far
reg  [4:0] fw;             // Hatari's wait states for this cycle
reg  [4:0] ft0;            // processor clock position of S0, mod 20
reg [23:2] faddr;          // RAM/ROM longword (reads: after the vector mapping)
reg        fa1;            // A1: the word within it
reg  [3:0] fbe;            // a write waiting for the write buffer
reg [31:0] fwd;
// posted RAM write
reg        wb_valid, wb_inflight;
reg [23:2] wb_addr;
reg  [3:0] wb_be;
reg [31:0] wb_data;
reg        rd_inflight;
// read-ahead buffer: the last 64-bit DDR3 word read (guest order)
reg        rb_valid;
reg [23:3] rb_tag;
reg [63:0] rb_data;
// device wait state memory
reg        ym_seen, acia_seen;
reg  [1:0] ym_cnt;
reg        fdc_mode4;   // $FF8606 bit 4: $FF8604 is the DMA sector count

assign wbuf_busy = wb_valid;
// hold the processor's next rising edge while it would be the cycle's
// DSACK edge (or later) and the answer is not there; and on the clock a
// cycle is first seen, whose next edge could be the DSACK edge before this
// bridge has decided anything (while the processor catches up, S0 and the
// following edge are on consecutive clocks).  A hold costs Falcon time
// (debt), never a processor clock.
assign hold = fmode && (((fst == F_RUN) && !fready && (fk >= (fplan ? fw : 5'd0))) ||
                        ((fst == F_IDLE) && as_act));

// position of S0: the rising edge before the one this AS was first seen at
wire [4:0] t0 = (tpos == 5'd0) ? 5'd19 : tpos - 5'd1;
function [3:0] esync;          // M68000_WaitEClock: clocks to the next multiple of 10
	input [4:0] pos;
	reg [4:0] m;
	begin
		m = (pos >= 5'd10) ? pos - 5'd10 : pos;
		esync = (m == 5'd0) ? 4'd0 : 4'd10 - m[3:0];
	end
endfunction
wire       fast16  = (la[23:20] == 4'hE) || is_cart || (la[23:16] == 8'hFF);
wire [4:0] bank_w  = (!fast16 && t0[1]) ? 5'd2 : 5'd0;    // the CHIP16 slot

// the device wait states of this cycle (16-bit port: the bytes it moves)
wire       cov_even = !la[0];
wire       cov_odd  = la[0] || (nbytes != 3'd1);
wire       is_mfpr  = (la[23:6] == 18'h3FFE8) && cov_odd && ({la[5:1], 1'b1} <= 6'h25);
wire       is_ym    = (la[23:2] == 22'h3FE200);
wire       is_fdcr  = (la[23:4] == 20'hFF860);
wire       is_aciar = (la[23:3] == 21'h1FFF80) && cov_even;
wire       is_dsph  = (la[23:3] == 21'h1FF440);
reg  [4:0] dev_w;
always @* begin
	dev_w = 5'd0;
	if (is_mfpr) dev_w = 5'd4;
	else if (is_ym) dev_w = (!ym_seen || ym_cnt == 2'd3) ? 5'd4 : 5'd0;
	else if (is_fdcr) begin
		case (la[3:1])
			3'd2:    dev_w = (!rw || !fdc_mode4) ? 5'd4 : 5'd0;   // $FF8604
			3'd3:    dev_w = !rw ? 5'd4 : 5'd0;                    // $FF8606
			3'd7:    dev_w = 5'd4;                                 // $FF860E
			default: dev_w = 5'd0;
		endcase
	end
	else if (is_aciar) dev_w = 5'd6 + (acia_seen ? 5'd0 : {1'b0, esync(t0)});
	else if (is_dsph) dev_w = (siz == 2'b00) ? 5'd8 : (nbytes != 3'd1) ? 5'd4 : 5'd0;
end

// 16-bit port writes: the cycle's bytes are on D31..D16
wire        p16_ub = !la[0];
wire        p16_lb = la[0] || (nbytes != 3'd1);
wire  [3:0] p16_be = la[1] ? {2'b00, p16_ub, p16_lb} : {p16_ub, p16_lb, 2'b00};
wire [31:0] p16_wd = la[1] ? {16'h0000, d_o[31:16]} : {d_o[31:16], 16'h0000};
function [15:0] word64;        // the 16-bit word i of a 64-bit word (guest order)
	input [63:0] q; input [1:0] i;
	word64 = q[63 - 16 * i -: 16];
endfunction
function [63:0] merge64;       // a 32-bit write (be[3] = lowest address) into a 64-bit word
	input [63:0] q; input hi; input [3:0] be; input [31:0] wd;
	integer j;
	begin
		merge64 = q;
		for (j = 0; j < 4; j = j + 1)
			if (be[3 - j]) merge64[63 - 8 * (4 * hi + j) -: 8] = wd[31 - 8 * j -: 8];
	end
endfunction
wire [23:2] maddr_rd = mem_addr(la, 1'b1);
wire        rb_hit   = rb_valid && (rb_tag == maddr_rd[23:3]);

task f_assert;                 // end the cycle (visible at the next falling edge)
	input [1:0] k;
	begin
		case (k)
			K_D16:   dsack1_n <= 0;
			K_D32:   begin dsack1_n <= 0; dsack0_n <= 0; end
			K_BERR:  berr_n <= 0;
			default: avec_n <= 0;
		endcase
		fst <= F_HOLD;
	end
endtask


task finish_berr;
	begin berr_n <= 0; st <= S_HOLD; end
endtask

always @(posedge clk) begin
	dev_stb    <= 0;
	iack_req   <= 0;
	cp_req     <= 0;
	cycle_done <= 0;
	credit     <= 4'd0;

	if (reset) begin
		st <= S_IDLE;
		dsack0_n <= 1; dsack1_n <= 1; berr_n <= 1; avec_n <= 1; ciin_n <= 1;
		ram_req <= 0; dev_cs <= 0;
		fmode <= fmode_in;
		fst <= F_IDLE; wb_valid <= 0; wb_inflight <= 0; rd_inflight <= 0; rb_valid <= 0;
		ym_seen <= 0; acia_seen <= 0;
	end
	else begin
	// the mode changes only between cycles, with no write in flight
	if (st == S_IDLE && fst == F_IDLE && !as_act && !wb_valid && !ram_req && fmode != fmode_in) begin
		fmode    <= fmode_in;
		rb_valid <= 0;
	end

	if (!fmode) case (st)
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
				else if (la[19:16] == 4'h2) begin
					cp_req   <= 1;
					cp_we    <= !rw;
					cp_id    <= la[15:13];
					cp_off   <= la[4:0];
					cp_siz   <= siz;
					cp_wdata <= d_o;
					st       <= S_CP;
				end
				else finish_berr;          // breakpoint, MMU access level
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

	S_CP:
		if (cp_ack) begin
			if (cp_berr) finish_berr;
			else begin
				d_i      <= cp_rdata;
				dsack1_n <= 0;
				// operand, instruction address, operand address: 32-bit port
				if (la[4:0] == 5'h10 || la[4:0] == 5'h18 || la[4:0] == 5'h1C) dsack0_n <= 0;
				st <= S_HOLD;
			end
		end

	S_HOLD:
		if (!as_act) begin
			dsack0_n <= 1; dsack1_n <= 1; berr_n <= 1; avec_n <= 1; ciin_n <= 1;
			cycle_done <= 1;
			st <= S_IDLE;
		end

	default: st <= S_IDLE;
	endcase

	else begin
		//------------------------------------------------ Falcon mode
		if (fst == F_RUN && fk != 5'd31) fk <= fk + {4'd0, cpu_ce};

		// the posted write goes to memory as soon as the port is free
		if (wb_valid && !wb_inflight && !rd_inflight && !ram_req) begin
			ram_req   <= 1;
			ram_we    <= 1;
			ram_addr  <= wb_addr;
			ram_be    <= wb_be;
			ram_wdata <= wb_data;
			wb_inflight <= 1;
		end
		if (ram_ack && wb_inflight) begin
			ram_req <= 0; wb_inflight <= 0; wb_valid <= 0;
		end

		case (fst)
		F_IDLE:
			if (as_act) begin
				// seen at the first rising edge after AS: S0 was the
				// processor's previous rising edge (position t0)
				fk     <= {4'd0, cpu_ce};
				ft0    <= t0;
				fplan  <= 1;
				fready <= 0;
				fjob   <= J_NONE;
				fkind  <= K_D16;
				tmo    <= 0;
				if (cpu_space) begin
					if (la[19:16] == 4'hF) begin
						iack_req   <= 1;
						iack_level <= la[3:1];
						fjob  <= J_IACK;
						fplan <= 0;            // the length depends on the source
						fw    <= 5'd0;
						fst   <= F_RUN;
					end
					else if (la[19:16] == 4'h2) begin
						cp_req   <= 1;
						cp_we    <= !rw;
						cp_id    <= la[15:13];
						cp_off   <= la[4:0];
						cp_siz   <= siz;
						cp_wdata <= d_o;
						fjob <= J_CP;
						fw   <= 5'd0;
						fst  <= F_RUN;
					end
					else begin                 // breakpoint, MMU access level
						fkind <= K_BERR; fw <= 5'd0;
						f_assert(K_BERR);
					end
				end
				else if ((is_prot && !is_super) || (is_vec && !rw)) begin
					// supervisor space from user mode; the reset vectors are ROM
					fkind <= K_BERR; fready <= 1; fw <= bank_w;
					if (bank_w == 5'd0) f_assert(K_BERR); else fst <= F_RUN;
				end
				else if (is_vec && rw && ram_tos) begin
					d_i    <= {2{la[2] ? (la[1] ? 16'h0000 : 16'h00E0) : (la[1] ? 16'h8000 : 16'h0000)}};
					fready <= 1; fw <= bank_w;
					if (bank_w == 5'd0) f_assert(K_D16); else fst <= F_RUN;
				end
				else if ((is_vec && rw) || (is_ram && rw) || ((is_rom || is_cart) && rw)) begin
					faddr <= maddr_rd;
					fa1   <= la[1];
					fw    <= bank_w;
					if (rb_hit) begin
						d_i    <= {2{word64(rb_data, {maddr_rd[2], la[1]})}};
						fready <= 1;
						if (bank_w == 5'd0) f_assert(K_D16); else fst <= F_RUN;
					end
					else begin
						fjob <= J_RD;
						fst  <= F_RUN;
					end
				end
				else if (is_ram && !rw) begin
					fw <= bank_w;
					if (!wb_valid) begin
						wb_valid <= 1; wb_addr <= la[23:2]; wb_be <= p16_be; wb_data <= p16_wd;
						if (rb_valid && rb_tag == la[23:3]) rb_data <= merge64(rb_data, la[2], p16_be, p16_wd);
						fready <= 1;
						if (bank_w == 5'd0) f_assert(K_D16); else fst <= F_RUN;
					end
					else begin
						faddr <= la[23:2]; fbe <= p16_be; fwd <= p16_wd;
						fjob <= J_WR;
						fst  <= F_RUN;
					end
				end
				else if (is_ide || is_io) begin
					dev_we   <= !rw;
					dev_addr <= la[23:1];
					dev_uds  <= !la[0];
					dev_lds  <= la[0] || (nbytes != 3'd1);
					dev_din  <= d_o[31:16];
					ciin_n   <= 0;                 // I/O is never cached
					fw       <= is_ide ? bank_w : dev_w;
					if (is_ym)    begin ym_seen <= 1; ym_cnt <= ym_seen ? ym_cnt + 2'd1 : 2'd0; end
					if (is_aciar) acia_seen <= 1;
					if (!wb_valid) begin           // RAM writes reach memory before any I/O
						dev_cs  <= 1;
						dev_stb <= 1;
						fjob    <= J_DEV;
					end
					else fjob <= J_DEVW;
					fst <= F_RUN;
				end
				else begin
					fkind <= K_BERR; fready <= 1; fw <= bank_w;
					if (bank_w == 5'd0) f_assert(K_BERR); else fst <= F_RUN;
				end
			end

		F_RUN: begin
			case (fjob)
				J_RD:
					if (!rd_inflight && !ram_req && !wb_valid) begin
						ram_req   <= 1;
						ram_we    <= 0;
						ram_addr  <= faddr;
						ram_be    <= 4'b1111;
						rd_inflight <= 1;
					end
					else if (ram_ack && rd_inflight) begin
						ram_req     <= 0;
						rd_inflight <= 0;
						rb_valid    <= 1;
						rb_tag      <= faddr[23:3];
						rb_data     <= ram_rdata64;
						d_i         <= {2{word64(ram_rdata64, {faddr[2], fa1})}};
						fready      <= 1;
						fjob        <= J_NONE;
					end
				J_WR:
					if (!wb_valid) begin
						wb_valid <= 1; wb_addr <= faddr; wb_be <= fbe; wb_data <= fwd;
						if (rb_valid && rb_tag == faddr[23:3]) rb_data <= merge64(rb_data, faddr[2], fbe, fwd);
						fready <= 1;
						fjob   <= J_NONE;
					end
				J_DEVW:
					if (!wb_valid) begin
						dev_cs  <= 1;
						dev_stb <= 1;
						tmo     <= 0;
						fjob    <= J_DEV;
					end
				J_DEV: begin
					tmo <= tmo + 1'd1;
					if (dev_berr || tmo == 10'h3FF) begin
						dev_cs <= 0;
						fkind  <= K_BERR;
						fready <= 1;
						fjob   <= J_NONE;
					end
					else if (dev_ack) begin
						dev_cs <= 0;
						d_i    <= {dev_dout, dev_dout};
						fready <= 1;
						fjob   <= J_NONE;
					end
				end
				J_IACK:
					if (iack_done) begin
						if (iack_spur) begin fkind <= K_BERR; fw <= 5'd0; end
						else if (iack_avec) begin fkind <= K_AVEC; fw <= 5'd7 + {1'b0, esync(ft0)}; end
						else begin
							d_i   <= {4{iack_vector}};
							fkind <= K_D32;
							fw    <= iack_mfp ? 5'd9 : 5'd0;
						end
						fplan  <= 1;
						fready <= 1;
						fjob   <= J_NONE;
					end
				J_CP:
					if (cp_ack) begin
						if (cp_berr) fkind <= K_BERR;
						else begin
							d_i <= cp_rdata;
							// operand, instruction address, operand address: 32-bit port
							fkind <= (la[4:0] == 5'h10 || la[4:0] == 5'h18 || la[4:0] == 5'h1C) ? K_D32 : K_D16;
						end
						fready <= 1;
						fjob   <= J_NONE;
					end
				default: ;
			endcase
			// DSACK (BERR, AVEC) for the falling edge after rising edge fw+1;
			// fk > fw: wait states that could not be avoided
			if (fready && fplan && fk >= fw) begin
				f_assert(fkind);
				credit <= (fk - fw > 5'd15) ? 4'd15 : fk[3:0] - fw[3:0];
			end
		end

		F_HOLD:
			if (!as_act) begin
				dsack0_n <= 1; dsack1_n <= 1; berr_n <= 1; avec_n <= 1; ciin_n <= 1;
				cycle_done <= 1;
				fst <= F_IDLE;
			end

		default: fst <= F_IDLE;
		endcase

		if (buf_flush || (snoop_we && snoop_addr[23:3] == rb_tag)) rb_valid <= 0;
		// a new instruction: its first YM/ACIA access waits again (an access
		// seen on the same clock belongs to the previous instruction)
		if (inst) begin ym_seen <= 0; acia_seen <= 0; end
	end
	end
end

// $FF8606 bit 4 selects the DMA sector count at $FF8604 (FDC_DMA.Mode,
// fdc.c): the CPU is the only writer of the mode register
always @(posedge clk)
	if (reset) fdc_mode4 <= 0;
	else if (as_act && !rw && la[23:1] == 23'h7FC303 && cov_odd) fdc_mode4 <= d_o[20];

endmodule
