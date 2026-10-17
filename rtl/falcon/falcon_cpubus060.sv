//============================================================================
//  Falcon bus bridge for the AP68060 (the ap060 variant), two clocks
//
//  The processor runs on clk_cpu (50 MHz, BCLK = CLK, docs/CT60_MODE.md);
//  the Falcon (arbiter, devices, interrupts, DMA) on clk (32 MHz).  The
//  68060 bus (MC68060UM section 7) is answered in two halves:
//
//    clk_cpu  the 68060 bus: a transfer is latched with TS.  A transfer to
//             Fast RAM (CT60 mode) goes to ap060_fastram, the processor's
//             own port to DDR3 (its line buffers and cache answer at the
//             processor's clock).  Any other transfer is a request to the
//             clk half: its write data is taken the clock after TS (a line
//             write's four beats one after the other, each acknowledged at
//             once but the last), then the request toggle flips; the
//             answer comes back with the answer toggle, and TA (TEA for a
//             bus error, TCI for a device, AVEC for an autovectored
//             interrupt) ends the transfer; a line read's four long words
//             go back at one beat a clock in the 68060's wrapping order.
//    clk      the Falcon side: the request is decoded and run as before -
//             RAM/ROM/cartridge on the arbiter's port, devices as 16-bit
//             cycles, interrupt acknowledges - and answered.
//
//  The request and the answer cross with toggles through three-stage
//  synchronisers; each side holds its fields (address, size, lanes, write
//  data; read data, flags) steady from its toggle until the other side has
//  answered, so only the toggles need synchronising (Falcon.sdc: the clocks
//  are asynchronous).  Writes are not posted: a write's bus error reaches
//  the processor.
//
//  Address map, Fast RAM off (24 bits, A31..A24 ignored), as falcon_cpubus:
//    000000-000007  reads: ROM (reset vectors; a RAM TOS: SSP $8000, PC
//                   $E00000), writes: bus error
//    000008-ram_top ST-RAM, above: bus error
//    E00000-E7FFFF  ROM, read only
//    FA0000-FBFFFF  cartridge, read only
//    F00000-F0FFFF, FF8000-FFFFFF  device bus (16 bits; the system answers
//                   dev_berr for the addresses that do not exist)
//    everything else: bus error
//  Supervisor only (TM2 = 1): 000000-0007FF and FF8000-FFFFFF.
//  CT60 mode (fast_ram != 0, taken at reset): 32-bit decode as on a CT60.
//  $00000000-$00FFFFFF and the mirror at $FF000000-$FFFFFFFF are the Falcon
//  map above (TOS reaches I/O as $FFFF8xxx); Fast RAM (TT-RAM) from
//  $01000000, 64, 128 or 240 MB, at DDR3 0x31000000+ through ap060_fastram;
//  every other address is a bus error, which is how EmuTOS finds the Fast
//  RAM's size and that the bus is 32 bits wide.  The Fast RAM is the
//  processor's alone: no DMA reaches it, so nothing writes the DDR3 that
//  ap060_fastram caches.
//
//  Transfers on the Falcon side: byte, word and long to RAM/ROM/cartridge
//  go to the arbiter's 32-bit port with the byte lanes of BS3-BS0; a line
//  (16 bytes) is read as two 64-bit words and written as four long words.
//  The reset vectors keep their mapping in a line (a RAM TOS's are
//  synthesized, a vector beat of a line write is dropped).  Device accesses
//  are 16-bit cycles (a long is two), answered with TCI; a line to a device
//  is a bus error.  Interrupt acknowledge (TT = 11): the MFP vector in every
//  byte lane, or AVEC, or TEA for a spurious interrupt; TM = 0 (breakpoint
//  acknowledge) is TEA.
//
//  Snooping (7.12): the 68060 only invalidates, and only while another
//  master owns the bus.  Writes by the DMA masters (snoop_we/snoop_addr from
//  falcon_memarb, clk) go through a dual-clock FIFO; a non-empty FIFO, or
//  the blitter's bus request, takes the bus from the processor (BG negated,
//  its tenure ends with BTT); the clk_cpu half then holds BB, drives one
//  snoop transfer per SNOOP_GAP clocks (TS on the bused TS, SNOOP asserted,
//  no termination) and ends its tenure with BTT when the FIFO is empty and
//  the blitter has finished.  The FIFO never overflows: when it is nearly
//  full (snoop_hold) the arbiter grants no DMA writes; the processor's own
//  accesses still go through, so its tenure can end and the FIFO drains.
//  The blitter has bus_grant while the bridge owns the bus for it.  A dirty
//  copyback line that a DMA write hits loses its data, as on the chip: the
//  OS must keep DMA buffers write-through.
//============================================================================

module falcon_cpubus060 #(
	parameter SNOOP_GAP = 2       // processor clocks from one snoop's TS to the next
)
(
	input             clk,         // the Falcon (32 MHz)
	input             clk_cpu,     // the processor (BCLK = CLK)
	input             reset,       // clk
	output            cpu_rst,     // clk_cpu: the processor's reset, released synchronised
	input             rsto,        // clk_cpu: the processor's RSTO (RESET instruction)
	output            rsto_sys,    // clk: RSTO, synchronised

	input       [3:0] ram_mb,      // clk, static
	input             ram_tos,
	input       [1:0] fast_ram,    // CT60 mode: 0 off, 1 64 MB, 2 128 MB, 3 240 MB Fast RAM (taken at reset)

	// the processor's bus (clk_cpu)
	input      [31:0] a,
	input             a_oe,
	input      [31:0] d_o,
	input             d_oe,
	input             rw_n,
	input       [1:0] siz,
	input       [3:0] bs_n,        // bs_n[0] = BS0 = D31..D24
	input       [1:0] tt,
	input       [2:0] tm,
	input             ts_n,
	input             ciout_n,
	input             bb_oe,       // the processor drives BB (explicit ownership)
	output     [31:0] d_i,
	output            ta_n,
	output reg        tea_n,
	output reg        tci_n,
	output reg        tbi_n,
	output reg        avec_n,
	output reg        bg_n,
	output reg        bb_n_i,      // the bus as another master drives it
	output reg        btt_n_i,
	output reg [31:0] a_i,
	output reg        ts_n_i,
	output reg  [1:0] tt_i,
	output reg        snoop_n,

	// Fast RAM's DDR3 port (clk; Avalon-MM, to the DDR3 arbiter)
	output     [28:0] avm_address,
	output      [7:0] avm_burstcount,
	output            avm_read,
	output            avm_write,
	output     [63:0] avm_writedata,
	output      [7:0] avm_byteenable,
	input             avm_waitrequest,
	input      [63:0] avm_readdata,
	input             avm_readdatavalid,

	// RAM/ROM/cartridge: the arbiter's CPU port (clk)
	output reg        ram_req,
	output reg        ram_we,
	output reg [27:2] ram_addr,    // the Falcon's 24-bit space ([27:24] zero)
	output reg  [3:0] ram_be,      // [3] = lowest address (D31..D24)
	output     [31:0] ram_wdata,
	input      [31:0] ram_rdata,
	input      [63:0] ram_rdata64, // the whole 64-bit word, guest order
	input             ram_ack,

	// device bus (16 bits, clk)
	output reg        dev_cs,
	output reg        dev_stb,
	output reg        dev_we,
	output reg [23:1] dev_addr,
	output reg        dev_uds,
	output reg        dev_lds,
	output     [15:0] dev_din,
	input      [15:0] dev_dout,
	input             dev_ack,
	input             dev_berr,
	output            dev_super,

	// interrupt acknowledge (clk)
	output reg        iack_req,
	output reg  [2:0] iack_level,
	input             iack_done,
	input             iack_avec,
	input             iack_spur,
	input       [7:0] iack_vector,

	// DMA writes to snoop, bus ownership for the blitter (clk)
	input             snoop_we,
	input      [23:0] snoop_addr,
	input             blit_br,
	output            blit_bg,
	output            snoop_hold,  // the snoop FIFO is nearly full: the arbiter holds DMA writes
	output reg        cycle_done   // one clock per completed transfer on the Falcon side
);

//============================================================================
// resets and static settings across
//============================================================================
reg  [2:0] rst_c = 3'b111;
always @(posedge clk_cpu or posedge reset)
	if (reset) rst_c <= 3'b111;
	else       rst_c <= {rst_c[1:0], 1'b0};
assign cpu_rst = rst_c[2];

reg  [2:0] rsto_s = 3'b000;
always @(posedge clk) rsto_s <= {rsto_s[1:0], rsto};
assign rsto_sys = rsto_s[2];

reg  [1:0] fsel = 2'd0;            // Fast RAM size, taken at reset (clk)
always @(posedge clk) if (reset) fsel <= fast_ram;
reg  [1:0] fsel_c1 = 2'd0, fsel_c = 2'd0;
reg  [3:0] mb_c1 = 4'd0, mb_c = 4'd0;
always @(posedge clk_cpu) begin
	fsel_c1 <= fsel; fsel_c <= fsel_c1;
	mb_c1   <= ram_mb; mb_c <= mb_c1;
end

// CT60 mode's map, from an address (either side, with its own copy of fsel)
function automatic is_fast_f(input [31:0] x, input [1:0] fs);
	is_fast_f = (fs != 2'd0) && x[31:24] != 8'h00 &&
	            x[31:24] < ((fs == 2'd1) ? 8'h05 : (fs == 2'd2) ? 8'h09 : 8'h10);
endfunction
function automatic fspace_f(input [31:0] x, input [1:0] fs);
	fspace_f = (fs == 2'd0) || x[31:24] == 8'h00 || x[31:24] == 8'hFF;
endfunction

//============================================================================
// clk_cpu: the 68060 bus
//============================================================================
localparam C_IDLE = 3'd0, C_FAST = 3'd1, C_DEC = 3'd2, C_WAIT = 3'd3,
           C_RBEAT = 3'd4, C_LWACK = 3'd5, C_LWDATA = 3'd6, C_ACK = 3'd7;
reg   [2:0] cst;
reg  [31:0] xa;                    // the transfer (held for the clk half)
reg         xrd;
reg   [1:0] xsiz;
reg   [3:0] xbe;
reg   [1:0] xtt;
reg   [2:0] xtm;
reg   [1:0] beat;
reg   [2:0] fleft;                 // Fast RAM: TAs still to come
reg         p_tgl = 1'b0;          // request toggle
reg  [31:0] b_d;
reg         b_ta_n;
reg         last_beat, err_beat;
reg         owned;                 // this bridge holds BB

// the answer (clk half), held from its toggle until the next request
reg         k_tgl = 1'b0;
reg  [31:0] k_single;               // a single transfer's read data
reg         k_err, k_ci, k_avec;
reg   [2:0] k_s = 3'b000;
always @(posedge clk_cpu) k_s <= {k_s[1:0], k_tgl};
wire        answered = (k_s[2] == p_tgl);

wire  [3:0] lanes = {~bs_n[0], ~bs_n[1], ~bs_n[2], ~bs_n[3]};
wire [23:0] ram_top_c = {mb_c, 20'd0};

// a line write goes to ST-RAM (or the vectors, dropped there) or is a bus error
wire        lw_ok = fspace_f(xa, fsel_c) && (xa[23:0] < ram_top_c) &&
                    !((xa[23:11] == 13'd0) && !xtm[2]);

// Fast RAM: decoded with TS from the bus
wire        fr_sel = !ts_n && a_oe && !owned && (tt != 2'd3) && is_fast_f(a, fsel_c);
wire        fr_ta_n, fr_doe, fr_busy;
wire [31:0] fr_d;

assign ta_n = b_ta_n & fr_ta_n;
assign d_i  = fr_doe ? fr_d : b_d;

// the write data by long word (A3:A2): block RAM, written on clk_cpu, read
// on clk (xq: the long word at x_ra a clock before)
reg  [31:0] xmem [0:3];
wire        x_we = !cpu_rst && (cst == C_DEC || cst == C_LWDATA);
wire  [1:0] x_wa = (cst == C_DEC) ? xa[3:2] : xa[3:2] + beat;
always @(posedge clk_cpu) if (x_we) xmem[x_wa] <= d_o;

// a line read's data: even and odd long words of each half, block RAM
// written on clk (S_LRD), read on clk_cpu one beat ahead (k_ra: the row of
// the long word the next clock takes, k_sel: its half)
reg  [31:0] kmem_e [0:1], kmem_o [0:1];
reg  [31:0] kq_e, kq_o;
reg         k_sel;
wire  [1:0] k_ni = (cst == C_RBEAT) ? xa[3:2] + beat + 2'd1 :
                   (cst == C_WAIT && answered) ? xa[3:2] + 2'd1 : xa[3:2];
always @(posedge clk_cpu) begin
	kq_e  <= kmem_e[k_ni[1]];
	kq_o  <= kmem_o[k_ni[1]];
	k_sel <= k_ni[0];
end
wire [31:0] k_word = k_sel ? kq_o : kq_e;

task c_start;
	begin
		xa <= a; xrd <= rw_n; xsiz <= siz; xbe <= lanes; xtt <= tt; xtm <= tm;
		beat <= 2'd0; last_beat <= 1'b0;
		if (fr_sel) begin fleft <= (siz == 2'b11) ? 3'd4 : 3'd1; cst <= C_FAST; end
		else cst <= C_DEC;
	end
endtask

always @(posedge clk_cpu) begin
	b_ta_n   <= 1'b1;
	tea_n    <= 1'b1;
	tci_n    <= 1'b1;
	tbi_n    <= 1'b1;
	avec_n   <= 1'b1;
	err_beat <= 1'b0;
	if (cpu_rst) begin
		cst <= C_IDLE;
		b_d <= 32'd0;
		last_beat <= 1'b0;
	end
	else case (cst)
	C_IDLE:
		if (!ts_n && a_oe && !owned) c_start;

	C_FAST:
		// ap060_fastram answers; its TAs are counted to see the end
		if (!fr_ta_n) begin
			fleft <= fleft - 3'd1;
			if (fleft == 3'd1) cst <= C_IDLE;
		end

	C_DEC: begin
		// the clock after TS: a write's (first) data is on the bus
		if (xsiz == 2'b11 && !xrd) begin
			if (!lw_ok) begin tea_n <= 1'b0; last_beat <= 1'b1; err_beat <= 1'b1; cst <= C_ACK; end
			else begin b_ta_n <= 1'b0; beat <= 2'd1; cst <= C_LWACK; end
		end else begin
			p_tgl <= ~p_tgl;
			cst   <= C_WAIT;
		end
	end

	C_LWACK:
		// the processor samples the beat's TA; its next data follows
		cst <= C_LWDATA;

	C_LWDATA: begin
		if (beat == 2'd3) begin
			p_tgl <= ~p_tgl;           // the last beat's TA waits for the write
			cst   <= C_WAIT;
		end else begin
			b_ta_n <= 1'b0;
			beat   <= beat + 2'd1;
			cst    <= C_LWACK;
		end
	end

	C_WAIT:
		if (answered) begin
			if (k_err) begin tea_n <= 1'b0; last_beat <= 1'b1; err_beat <= 1'b1; cst <= C_ACK; end
			else if (xsiz == 2'b11 && xrd) begin
				// a line read: four beats, one a clock
				b_d    <= k_word;
				b_ta_n <= 1'b0;
				beat   <= 2'd1;
				cst    <= C_RBEAT;
			end else begin
				b_d    <= k_single;
				b_ta_n <= 1'b0;
				if (k_ci)   tci_n  <= 1'b0;
				if (k_avec) avec_n <= 1'b0;
				last_beat <= 1'b1;
				cst <= C_ACK;
			end
		end

	C_RBEAT: begin
		b_d    <= k_word;
		b_ta_n <= 1'b0;
		beat   <= beat + 2'd1;
		if (beat == 2'd3) begin last_beat <= 1'b1; cst <= C_ACK; end
	end

	C_ACK:
		// the clock the processor samples the last TA/TEA
		begin
			cst <= C_IDLE;
			if (!ts_n && a_oe && !owned && !err_beat) c_start;
		end

	default: cst <= C_IDLE;
	endcase
end

//----------------------------------------------------------------------------
// clk_cpu: snoops and bus ownership
//----------------------------------------------------------------------------
wire        sq_empty;
wire [19:0] sq_line;
wire  [5:0] sq_wlevel;
reg   [1:0] br_c = 2'b00;
always @(posedge clk_cpu) br_c <= {br_c[0], blit_br};
reg         ending;
reg   [2:0] sgap;
wire        want_bus = br_c[1] || !sq_empty || owned;
wire        sq_issue = owned && !sq_empty && !ending && sgap == 3'd0;

always @(posedge clk_cpu) begin
	ts_n_i  <= 1'b1;
	snoop_n <= 1'b1;
	tt_i    <= 2'd0;
	btt_n_i <= 1'b1;
	ending  <= 1'b0;
	if (cpu_rst) begin
		bg_n   <= 1'b0;
		bb_n_i <= 1'b1;
		owned  <= 1'b0;
		a_i    <= 32'd0;
		sgap   <= 3'd0;
	end else begin
		if (sgap != 3'd0) sgap <= sgap - 3'd1;
		// take the bus: BG negated; the processor's tenure is over once it
		// has stopped driving BB and the address bus
		bg_n <= want_bus;
		if (!owned) begin
			if (want_bus && bg_n && !bb_oe && !a_oe && cst == C_IDLE) begin
				owned  <= 1'b1;
				bb_n_i <= 1'b0;
			end
		end else if (sq_issue) begin
			a_i     <= {8'd0, sq_line, 4'd0};
			ts_n_i  <= 1'b0;
			snoop_n <= 1'b0;
			sgap    <= 3'(SNOOP_GAP - 1);
		end else if (!br_c[1] && sq_empty && !ending && sgap == 3'd0) begin
			// end of tenure: BTT for one clock, BB negated
			btt_n_i <= 1'b0;
			bb_n_i  <= 1'b1;
			ending  <= 1'b1;
			owned   <= 1'b0;
		end
	end
end

ap030_async_fifo #(.W(20), .AW(5)) snoopq (
	.wclk(clk), .we(snoop_we), .wdata(snoop_addr[23:4]), .wfull(), .wlevel(sq_wlevel),
	.rclk(clk_cpu), .re(sq_issue), .rdata(sq_line), .rempty(sq_empty), .rlevel()
);
// four entries of margin: a write granted before the hold shows still arrives
assign snoop_hold = (sq_wlevel >= 6'd28);

reg   [1:0] bg_s = 2'b00;
always @(posedge clk) bg_s <= {bg_s[0], owned && br_c[1]};
assign blit_bg = bg_s[1];

//----------------------------------------------------------------------------
// clk_cpu: Fast RAM (CT60 mode), the processor's own port to DDR3
//----------------------------------------------------------------------------
ap060_fastram fastram (
	.clk(clk_cpu), .clk_mem(clk), .rst(cpu_rst), .bclk_en(1'b1),
	.m_ign(1'b0), .m_cnt(16'd0), .m_hold(1'b0),
	.a(a), .rw_n(rw_n), .siz(siz), .tt(tt), .tm(tm), .bs_n(bs_n), .ts_n(ts_n), .ciout_n(ciout_n),
	.d_cpu(d_o), .sel(fr_sel),
	// guest $01000000+ is DDR3 0x31000000+ (64-bit word 0x6000000 + A27..A3)
	.ddr_addr(29'h6000000 | {4'd0, a[27:3]}),
	.ta_n(fr_ta_n), .d_o(fr_d), .d_oe(fr_doe), .busy(fr_busy),
	.avm_address(avm_address), .avm_burstcount(avm_burstcount), .avm_read(avm_read), .avm_write(avm_write),
	.avm_writedata(avm_writedata), .avm_byteenable(avm_byteenable), .avm_waitrequest(avm_waitrequest),
	.avm_readdata(avm_readdata), .avm_readdatavalid(avm_readdatavalid)
);

//============================================================================
// clk: the Falcon side
//============================================================================
localparam S_IDLE = 4'd0, S_DEC = 4'd1, S_RAMW = 4'd2, S_DEV = 4'd3, S_DEV2 = 4'd4,
           S_IACK = 4'd5, S_LRD = 4'd6, S_LWR = 4'd7, S_DONE = 4'd8;
reg   [3:0] st;
reg   [2:0] p_s = 3'b000;
always @(posedge clk) p_s <= {p_s[1:0], p_tgl};
wire        requested = (p_s[2] != k_tgl);

// the request: the clk_cpu half's fields, steady from its toggle until the
// answer has been taken (read here only from S_DEC to S_DONE)
wire [31:0] la   = xa;
wire        rd   = xrd;
wire  [1:0] tsiz = xsiz;
wire  [3:0] be   = xbe;
wire  [1:0] ltt  = xtt;
wire  [2:0] ltm  = xtm;
wire        sup  = xtm[2];
reg   [1:0] lb;                    // line: the long word being read or written
reg         dev_lo;                // the device word is the long word's D15..D0

// the write data (xq: the long word at x_ra, registered; steady from the
// request's toggle on): A3:A2, or lb in a line write
reg  [31:0] xq;
wire  [1:0] x_ra = (st == S_LWR) ? lb : xa[3:2];
always @(posedge clk) xq <= xmem[x_ra];
assign ram_wdata = xq;
assign dev_din   = dev_lo ? xq[15:0] : xq[31:16];

// a line read's 64-bit halves into the read RAM (S_LRD below): the RAM TOS
// vectors at A3 = 0, or the arbiter's answer for half lb[1]
wire        k_tos = (st == S_LRD) && !lb[0] && !ram_req && vec_line && !lb[1] && ram_tos;
wire        k_we  = k_tos || ((st == S_LRD) && !(!lb[0] && !ram_req) && ram_ack);
wire        k_row = k_tos ? 1'b0 : lb[1];
wire [63:0] k_wd  = k_tos ? {32'h00008000, 32'h00E00000} : ram_rdata64;
always @(posedge clk) if (k_we) begin kmem_e[k_row] <= k_wd[63:32]; kmem_o[k_row] <= k_wd[31:0]; end
reg  [15:0] dev_hi;
reg   [9:0] tmo;

wire [23:0] ram_top = {ram_mb, 20'd0};
wire        fspace  = fspace_f(la, fsel);
wire        is_vec  = fspace && (la[23:3] == 21'd0);
wire        is_ram  = fspace && (la[23:0] < ram_top);
wire        is_rom  = fspace && (la[23:19] == 5'b11100);
wire        is_cart = fspace && (la[23:17] == 7'b1111101);
wire        is_ide  = fspace && (la[23:16] == 8'hF0);
wire        is_io   = fspace && (la[23:15] == 9'b111111111);
wire        is_prot = fspace && ((la[23:11] == 13'd0) || is_io);
wire        is_mem  = is_ram || ((is_rom || is_cart) && rd);
wire        vec_line = fspace && (la[23:4] == 20'd0);   // a line over the reset vectors

assign dev_super = sup;

// RAM/ROM address actually used: the reset vectors come from ROM
function automatic [23:2] mem_addr(input [23:0] x, input r);
	mem_addr = (r && x[23:3] == 21'd0) ? (22'h380000 | {21'd0, x[2]}) : x[23:2];
endfunction

task answer(input err, input ci, input av);
	begin k_err <= err; k_ci <= ci; k_avec <= av; st <= S_DONE; end
endtask

always @(posedge clk) begin
	dev_stb    <= 1'b0;
	iack_req   <= 1'b0;
	cycle_done <= 1'b0;
	if (reset) begin
		st <= S_IDLE;
		ram_req <= 1'b0; dev_cs <= 1'b0;
		k_tgl <= p_s[2];   // no answer owed for a request of before the reset
	end
	else case (st)
	S_IDLE:
		if (requested) begin
			lb <= 2'd0; tmo <= 10'd0;
			st <= S_DEC;
		end

	S_DEC:
		if (ltt == 2'd3) begin
			// acknowledge: interrupt (TM = level) or breakpoint (TM = 0, TEA)
			if (ltm == 3'd0) answer(1'b1, 1'b0, 1'b0);
			else begin iack_req <= 1'b1; iack_level <= ltm; st <= S_IACK; end
		end
		else if (is_prot && !sup) answer(1'b1, 1'b0, 1'b0);
		else if (tsiz == 2'b11) begin
			// a line: ST-RAM, ROM or cartridge (a read), never a device
			if (!(is_ram || ((is_rom || is_cart) && rd))) answer(1'b1, 1'b0, 1'b0);
			else if (rd) st <= S_LRD;
			else st <= S_LWR;
		end
		else if (is_vec && rd && ram_tos) begin
			k_single <= la[2] ? 32'h00E00000 : 32'h00008000;
			answer(1'b0, 1'b0, 1'b0);
		end
		else if (is_vec && !rd) answer(1'b1, 1'b0, 1'b0);
		else if (is_mem) begin
			ram_req   <= 1'b1;
			ram_we    <= !rd;
			ram_addr  <= {4'd0, mem_addr(la[23:0], rd)};
			ram_be    <= be;
			st <= S_RAMW;
		end
		else if (is_ide || is_io) begin
			dev_cs   <= 1'b1;
			dev_stb  <= 1'b1;
			dev_we   <= !rd;
			dev_addr <= la[23:1];
			// the 16-bit device word is on D31..D16 for A1 = 0, D15..D0 for A1 = 1
			dev_uds  <= la[1] ? be[1] : be[3];
			dev_lds  <= la[1] ? be[0] : be[2];
			dev_lo   <= la[1];
			st <= S_DEV;
		end
		else answer(1'b1, 1'b0, 1'b0);

	S_RAMW:
		if (ram_ack) begin
			ram_req <= 1'b0;
			k_single <= ram_rdata;
			answer(1'b0, 1'b0, 1'b0);
		end

	S_LRD:
		// a line read: the two 64-bit words, A3 = 0 then 1 (the vectors from
		// ROM, or a RAM TOS's synthesized)
		if (lb[0] == 1'b0 && !ram_req) begin
			if (vec_line && lb[1] == 1'b0 && ram_tos) begin
				lb <= 2'd2;
				if (lb[1]) answer(1'b0, 1'b0, 1'b0);
			end else begin
				ram_req  <= 1'b1; ram_we <= 1'b0; ram_be <= 4'hF;
				ram_addr <= {4'd0, mem_addr({la[23:4], lb[1], 3'b000}, 1'b1)};
				lb[0]    <= 1'b1;
			end
		end
		else if (ram_ack) begin
			ram_req <= 1'b0;
			if (lb[1]) answer(1'b0, 1'b0, 1'b0);
			else lb <= 2'd2;
		end

	S_LWR:
		// a line write: four long words (a vector beat is dropped)
		if (!ram_req) begin
			if (vec_line && !lb[1]) begin
				if (lb == 2'd1) lb <= 2'd2; else lb <= lb + 2'd1;
			end else begin
				ram_req   <= 1'b1; ram_we <= 1'b1; ram_be <= 4'hF;
				ram_addr  <= {4'd0, la[23:4], lb};
			end
		end
		else if (ram_ack) begin
			ram_req <= 1'b0;
			if (lb == 2'd3) answer(1'b0, 1'b0, 1'b0);
			lb <= lb + 2'd1;
		end

	S_DEV: begin
		tmo <= tmo + 10'd1;
		if (dev_berr || tmo == 10'h3FF) begin dev_cs <= 1'b0; answer(1'b1, 1'b0, 1'b0); end
		else if (dev_ack) begin
			if (tsiz == 2'b00) begin
				// a long: the second 16-bit cycle at A + 2
				dev_hi   <= dev_dout;
				dev_stb  <= 1'b1;
				dev_addr <= {la[23:2], 1'b1};
				dev_uds  <= be[1];
				dev_lds  <= be[0];
				dev_lo   <= 1'b1;
				tmo      <= 10'd0;
				st <= S_DEV2;
			end else begin
				dev_cs <= 1'b0;
				k_single <= la[1] ? {16'd0, dev_dout} : {dev_dout, 16'd0};
				answer(1'b0, 1'b1, 1'b0);
			end
		end
	end

	S_DEV2: begin
		tmo <= tmo + 10'd1;
		if (dev_berr || tmo == 10'h3FF) begin dev_cs <= 1'b0; answer(1'b1, 1'b0, 1'b0); end
		else if (dev_ack) begin
			dev_cs <= 1'b0;
			k_single <= {dev_hi, dev_dout};
			answer(1'b0, 1'b1, 1'b0);
		end
	end

	S_IACK:
		if (iack_done) begin
			k_single <= {4{iack_vector}};
			answer(iack_spur, 1'b0, iack_avec && !iack_spur);
		end

	S_DONE: begin
		// the answer's fields are in place: flip the toggle
		k_tgl      <= ~k_tgl;
		cycle_done <= 1'b1;
		st <= S_IDLE;
	end

	default: st <= S_IDLE;
	endcase
end

endmodule
