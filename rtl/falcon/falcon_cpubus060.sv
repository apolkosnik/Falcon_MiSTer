//============================================================================
//  Falcon bus bridge for the AP68060 (the ap060 variant)
//
//  The 68060 bus (MC68060UM section 7) as a synchronous slave: TS is
//  sampled on a bus clock edge, TA/TEA/TCI/TBI/AVEC are driven from
//  flip-flops for one bus clock per beat, read data with TA.  The bus
//  clock is the processor clock's edges with bclk_en (CLKEN): the
//  processor's half-speed bus mode when bclk_en toggles (BCLK = CLK/2).
//  Everything facing the processor moves on those edges only; the arbiter
//  and device handshakes run on every clock and are latched for the next
//  bus clock edge.
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
//
//  CT60 mode (fast_ram != 0, taken at reset; docs/CT60_MODE.md): 32-bit
//  decode as on a CT60.  $00000000-$00FFFFFF and the mirror at
//  $FF000000-$FFFFFFFF are the Falcon map above (TOS reaches I/O as
//  $FFFF8xxx); Fast RAM (TT-RAM) from $01000000, 64, 128 or 240 MB, on the
//  arbiter's port at guest $01000000+ (DDR3 0x31000000+); every other
//  address is a bus error, which is how EmuTOS finds the Fast RAM's size
//  and that the bus is 32 bits wide.  The system runs the bus at full speed
//  in CT60 mode (BCLK = CLK, docs/CT60_MODE.md milestone 3); a Falcon-space
//  transfer (ST-RAM, ROM, I/O) then waits FALCON_WAIT bus clocks before its
//  decode and FALCON_BEAT_WAIT before each further line beat, as a CT60's
//  Falcon accesses go through its CPLD: it stays at least as long as on the
//  half-speed bus; Fast RAM is answered as soon as the arbiter has it.
//
//  Transfers: byte, word and long to RAM/ROM/cartridge go to the arbiter's
//  32-bit port with the byte lanes of BS3-BS0; a line (SIZ = 11, 16 bytes)
//  is four beats in the 68060's wrapping order (A3:A2 given with TS first),
//  reads from two 64-bit words, writes as four 32-bit words.  Device
//  accesses are 16-bit cycles on the device bus (a long is two), with TCI
//  asserted (cache inhibit); a line to a device is a bus error.  Interrupt
//  acknowledge (TT = 11): the MFP vector in every byte lane, or AVEC, or
//  TEA for a spurious interrupt; TM = 0 (breakpoint acknowledge) is TEA.
//
//  Snooping (7.12): the 68060 only invalidates, and only while another
//  master owns the bus.  Writes by the DMA masters (snoop_we/snoop_addr
//  from falcon_memarb) are queued; the queue, or the blitter's bus request,
//  takes the bus from the processor (BG negated, its tenure ends with BTT),
//  then this bridge holds BB, drives the snoop transfers (TS on the bused
//  TS, SNOOP asserted, no termination) SNOOP_GAP bus clocks apart, and
//  ends its tenure with BTT when the queue is empty and the blitter has
//  finished.  The processor cannot stall a snooped transfer and its data
//  cache takes about six processor clocks per lookup through an eight-entry
//  queue that drops what does not fit: snoops are spaced like the
//  transfers of a real alternate master, and a write to the line queued
//  last (the DMA's 16-bit words) is not queued again.  The queue never
//  overflows: when it is nearly full (snoop_hold) the arbiter grants no
//  DMA writes; the processor's own accesses still go through, so its
//  tenure can end and the queue drains.  The blitter
//  has bus_grant while the bridge owns the bus for it; its own writes are
//  snooped on the fly.  A dirty copyback line that a DMA write hits loses
//  its data, as on the chip: the OS must keep DMA buffers write-through.
//============================================================================

module falcon_cpubus060 #(
	parameter SNOOP_GAP   = 4,    // bus clocks from one snoop's TS to the next
	parameter FALCON_WAIT = 4,    // CT60 mode: bus clocks a Falcon-space transfer waits before its decode,
	parameter FALCON_BEAT_WAIT = 2 // ... and before each further beat of a line (tb/bridge060 measures both)
)
(
	input             clk,
	input             bclk_en,     // this clock edge is a bus clock edge (CLKEN)
	input             reset,

	input       [3:0] ram_mb,
	input             ram_tos,
	input       [1:0] fast_ram,    // CT60 mode: 0 off, 1 64 MB, 2 128 MB, 3 240 MB Fast RAM (taken at reset)

	// the processor's bus
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
	input             bb_oe,       // the processor drives BB (explicit ownership)
	output reg [31:0] d_i,
	output reg        ta_n,
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

	// RAM/ROM/cartridge: the arbiter's CPU port
	output reg        ram_req,
	output reg        ram_we,
	output reg [27:2] ram_addr,    // guest address: [27:24] non-zero only for Fast RAM
	output reg  [3:0] ram_be,      // [3] = lowest address (D31..D24)
	output reg [31:0] ram_wdata,
	input      [31:0] ram_rdata,
	input      [63:0] ram_rdata64, // the whole 64-bit word, guest order
	input             ram_ack,

	// device bus (16 bits)
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
	output            dev_super,

	// interrupt acknowledge
	output reg        iack_req,
	output reg  [2:0] iack_level,
	input             iack_done,
	input             iack_avec,
	input             iack_spur,
	input       [7:0] iack_vector,

	// DMA writes to snoop, bus ownership for the blitter
	input             snoop_we,
	input      [23:0] snoop_addr,
	input             blit_br,
	output            blit_bg,
	output            snoop_hold,  // the snoop queue is nearly full: the arbiter holds DMA writes
	output reg        cycle_done   // one clock per completed transfer
);

localparam S_IDLE = 4'd0, S_RAM = 4'd1, S_RAMW = 4'd2, S_DEV = 4'd3, S_DEV2 = 4'd4,
           S_IACK = 4'd5, S_ACK = 4'd6, S_LINE = 4'd7, S_LINEW = 4'd8, S_WDATA = 4'd9;

reg  [3:0] st;
reg [31:0] la;           // the transfer's address
reg  [1:0] fsel;         // Fast RAM size, taken at reset
always @(posedge clk) if (reset) fsel <= fast_ram;
reg        rd;
reg  [1:0] tsiz;
reg  [3:0] be;           // byte lanes, [3] = D31..D24
reg        sup;
reg  [1:0] beat;         // line beats done
reg  [1:0] bsel;         // A3:A2 of the current beat
reg        have64;       // ram_rd64 holds the 64-bit word of the current pair of beats
reg [15:0] dev_hi;       // first half of a long device read
reg  [9:0] tmo;
reg  [3:0] fw;           // CT60 mode: Falcon-space wait clocks so far
reg  [3:0] fb;           // ... and the wait left before the next line beat
reg        last_beat;    // the TA being driven ends the transfer
reg        err_beat;

wire [23:0] ram_top = {ram_mb, 20'd0};
// CT60 mode: the Falcon is at $00xxxxxx and $FFxxxxxx, Fast RAM from
// $01000000 up to f_end (exclusive, in A31..A24), the rest a bus error
wire        ct60    = (fsel != 2'd0);
wire  [7:0] f_end   = (fsel == 2'd1) ? 8'h05 : (fsel == 2'd2) ? 8'h09 : 8'h10;
wire        fspace  = !ct60 || la[31:24] == 8'h00 || la[31:24] == 8'hFF;
wire        is_fast = ct60 && la[31:24] != 8'h00 && la[31:24] < f_end;
wire        is_vec  = fspace && (la[23:3] == 21'd0);
wire        is_ram  = fspace && (la[23:0] < ram_top);
wire        is_rom  = fspace && (la[23:19] == 5'b11100);
wire        is_cart = fspace && (la[23:17] == 7'b1111101);
wire        is_ide  = fspace && (la[23:16] == 8'hF0);
wire        is_io   = fspace && (la[23:15] == 9'b111111111);
wire        is_prot = fspace && ((la[23:11] == 13'd0) || is_io);
wire        is_mem  = is_ram || ((is_rom || is_cart) && rd) || is_fast;
// the address on the arbiter's port: the Falcon's 24 bits, or Fast RAM's 28
wire [27:0] pa      = is_fast ? la[27:0] : {4'd0, la[23:0]};

assign dev_super = sup;

// the 64-bit word address of a line beat; the beat's long word is the upper
// half of the word when A2 = 0
wire [27:2] beat_addr = {pa[27:4], bsel};
// the beat reads or writes the reset vectors ($000000-$000007): a line
// beat gets the same mapping as a single transfer
wire        vec_beat  = fspace && (la[23:4] == 20'd0) && !bsel[1];

// RAM/ROM address actually used: the reset vectors come from ROM
function [27:2] mem_addr;
	input [27:0] x; input r;
	mem_addr = (r && x[27:3] == 25'd0) ? (26'h0380000 | {25'd0, x[2]}) : x[27:2];
endfunction

// the byte lanes a byte/word/long transfer uses (table 2-6), from BS3-BS0
wire [3:0] lanes = {~bs_n[0], ~bs_n[1], ~bs_n[2], ~bs_n[3]};

//----------------------------------------------------------------------------
// snoop queue and bus ownership
//----------------------------------------------------------------------------
reg [19:0] sq [0:63];
reg  [5:0] sq_wr, sq_rd;
wire       sq_empty = (sq_wr == sq_rd);
wire [5:0] sq_used  = sq_wr - sq_rd;
// four entries of margin: a write granted before the hold shows still arrives
assign     snoop_hold = (sq_used >= 6'd60);
reg        owned;        // this bridge holds BB
reg        ending;       // BTT driven this bus clock
reg  [2:0] sgap;         // bus clocks until the next snoop may start
reg [19:0] sq_last;      // the line queued last
reg        sq_lv;
wire       want_bus = blit_br || !sq_empty || owned;
assign     blit_bg  = owned && blit_br;
wire       sq_drive = owned && !sq_empty && !ending && sgap == 0;   // a snoop transfer this bus clock

always @(posedge clk) begin
	if (reset) begin
		sq_wr <= 0; sq_rd <= 0; sq_lv <= 0;
	end else begin
		// (only while that line still waits: once its snoop has gone out,
		// the processor may have cached the line again)
		if (bclk_en && sq_drive) begin
			sq_rd <= sq_rd + 1'd1;
			if (sq_rd + 1'd1 == sq_wr) sq_lv <= 1'b0;
		end
		if (snoop_we && !(sq_lv && sq_last == snoop_addr[23:4] && !(bclk_en && sq_drive && sq_rd + 1'd1 == sq_wr))) begin
			sq[sq_wr] <= snoop_addr[23:4];
			sq_wr   <= sq_wr + 1'd1;
			sq_last <= snoop_addr[23:4];
			sq_lv   <= 1'b1;
		end
	end
end

always @(posedge clk) begin
	if (reset) begin
		bg_n   <= 1'b0;
		bb_n_i <= 1'b1;
		owned  <= 1'b0;
		a_i    <= 32'd0;
		ts_n_i  <= 1'b1;
		snoop_n <= 1'b1;
		tt_i    <= 2'd0;
		btt_n_i <= 1'b1;
		ending  <= 1'b0;
		sgap    <= 3'd0;
	end else if (bclk_en) begin
		ts_n_i  <= 1'b1;
		snoop_n <= 1'b1;
		tt_i    <= 2'd0;
		btt_n_i <= 1'b1;
		ending  <= 1'b0;
		if (sgap != 0) sgap <= sgap - 1'd1;
		// take the bus: BG negated; the processor's tenure is over once it
		// has stopped driving BB and the address bus
		bg_n <= want_bus;
		if (!owned) begin
			if (want_bus && bg_n && !bb_oe && !a_oe && st == S_IDLE) begin
				owned  <= 1'b1;
				bb_n_i <= 1'b0;
			end
		end else begin
			if (sq_drive) begin
				a_i     <= {8'd0, sq[sq_rd], 4'd0};
				ts_n_i  <= 1'b0;
				snoop_n <= 1'b0;
				sgap    <= 3'(SNOOP_GAP - 1);
			end else if (!blit_br && sq_empty && !ending && sgap == 0) begin
				// end of tenure: BTT for one bus clock, BB negated
				btt_n_i <= 1'b0;
				bb_n_i  <= 1'b1;
				ending  <= 1'b1;
				owned   <= 1'b0;
			end
		end
	end
end

//----------------------------------------------------------------------------
// the arbiter's and the devices' answers, latched for the bus clock
//----------------------------------------------------------------------------
reg         ram_got, dev_got, dev_err, iack_got, iack_sp, iack_av;
reg  [63:0] ram_rd64;     // the last 64-bit word the arbiter answered (also a line's held word)
reg  [15:0] dev_rd;
reg   [7:0] iack_vec;
wire        ram_done   = ram_ack || ram_got;
wire [63:0] ram_data64 = ram_got ? ram_rd64 : ram_rdata64;
// the arbiter's 32-bit answer is the half of the 64-bit word at A2
wire [31:0] ram_data   = ram_addr[2] ? ram_data64[31:0] : ram_data64[63:32];
wire        dev_done   = dev_ack || dev_got;
wire        dev_fail   = dev_berr || dev_err;
wire [15:0] dev_data   = dev_got ? dev_rd : dev_dout;
wire        iack_fin   = iack_done || iack_got;
wire        iack_spv   = iack_got ? iack_sp : iack_spur;
wire        iack_avv   = iack_got ? iack_av : iack_avec;
wire  [7:0] iack_vecv  = iack_got ? iack_vec : iack_vector;

//----------------------------------------------------------------------------
// transfers
//----------------------------------------------------------------------------
task finish_tea;
	begin tea_n <= 1'b0; last_beat <= 1'b1; err_beat <= 1'b1; st <= S_ACK; end
endtask

task start_transfer;
	begin
		la    <= a;
		rd    <= rw_n;
		tsiz  <= siz;
		be    <= lanes;
		sup   <= tm[2];
		beat  <= 2'd0;
		bsel  <= a[3:2];
		have64 <= 1'b0;
		tmo   <= 10'd0;
		fw    <= 4'd0;
		fb    <= 4'd0;
		last_beat <= 1'b0;
		if (tt == 2'd3) begin
			// acknowledge: interrupt (TM = level) or breakpoint (TM = 0)
			if (tm == 3'd0) begin tea_n <= 1'b0; last_beat <= 1'b1; err_beat <= 1'b1; st <= S_ACK; end
			else begin iack_req <= 1'b1; iack_level <= tm; st <= S_IACK; end
		end
		else st <= S_RAM;   // decoded next bus clock from the latched address
	end
endtask

always @(posedge clk) begin
	// every clock: the one-clock strobes, the answers latched
	dev_stb    <= 1'b0;
	iack_req   <= 1'b0;
	cycle_done <= 1'b0;
	if (ram_ack)   begin ram_got <= 1'b1; ram_rd64 <= ram_rdata64; ram_req <= 1'b0; end
	if (dev_ack)   begin dev_got <= 1'b1; dev_rd <= dev_dout; end
	if (dev_berr)  dev_err <= 1'b1;
	if (iack_done) begin iack_got <= 1'b1; iack_sp <= iack_spur; iack_av <= iack_avec; iack_vec <= iack_vector; end

	if (reset) begin
		st <= S_IDLE;
		ram_req <= 1'b0; dev_cs <= 1'b0;
		d_i <= 32'd0;
		ta_n <= 1'b1; tea_n <= 1'b1; avec_n <= 1'b1; tci_n <= 1'b1; tbi_n <= 1'b1;
		last_beat <= 1'b0; err_beat <= 1'b0;
		ram_got <= 1'b0; dev_got <= 1'b0; dev_err <= 1'b0; iack_got <= 1'b0;
	end
	else if (bclk_en) begin
		ta_n     <= 1'b1;
		tea_n    <= 1'b1;
		avec_n   <= 1'b1;
		tci_n    <= 1'b1;
		tbi_n    <= 1'b1;
		err_beat <= 1'b0;

		case (st)
		S_IDLE:
			if (!ts_n && a_oe && !owned) start_transfer;

		S_RAM: if (ct60 && !is_fast && fw != 4'(FALCON_WAIT)) fw <= fw + 4'd1; else begin
			// decode (the bus clock after TS, on the latched address)
			if (is_prot && !sup) finish_tea;
			else if (is_vec && rd && ram_tos && tsiz != 2'b11) begin
				d_i <= la[2] ? 32'h00E00000 : 32'h00008000;
				ta_n <= 1'b0; last_beat <= 1'b1; st <= S_ACK;
			end
			else if (is_vec && !rd && tsiz != 2'b11) finish_tea;
			else if (is_mem) begin
				ram_got <= 1'b0;
				if (tsiz == 2'b11) begin
					if (rd) begin
						// (a RAM TOS's vector beat is answered in S_LINE, no fetch)
						ram_req  <= !(ram_tos && vec_beat); ram_we <= 1'b0;
						ram_addr <= mem_addr({beat_addr, 2'b00}, 1'b1); ram_be <= 4'hF;
						st <= S_LINE;
					end else begin
						// line write: the first beat's data is on the bus now;
						// TA takes it, then the next beat's data follows.  A
						// vector beat is not written (as a single write would
						// bus error): it is acknowledged alone
						ram_req   <= !vec_beat; ram_we <= 1'b1;
						ram_got   <= vec_beat;
						ram_addr  <= beat_addr; ram_be <= 4'hF;
						ram_wdata <= d_o;
						st <= S_LINEW;
					end
				end else begin
					ram_req   <= 1'b1;
					ram_we    <= !rd;
					ram_addr  <= mem_addr(pa, rd);
					ram_be    <= be;
					ram_wdata <= d_o;
					st <= S_RAMW;
				end
			end
			else if (is_ide || is_io) begin
				if (tsiz == 2'b11) finish_tea;
				else begin
					dev_got  <= 1'b0; dev_err <= 1'b0;
					dev_cs   <= 1'b1;
					dev_stb  <= 1'b1;
					dev_we   <= !rd;
					dev_addr <= la[23:1];
					// the 16-bit device word is on D31..D16 for A1 = 0, D15..D0 for A1 = 1
					dev_uds  <= la[1] ? be[1] : be[3];
					dev_lds  <= la[1] ? be[0] : be[2];
					dev_din  <= la[1] ? d_o[15:0] : d_o[31:16];
					st <= S_DEV;
				end
			end
			else finish_tea;
		end

		S_RAMW:
			if (ram_done) begin
				ram_got <= 1'b0;
				d_i     <= ram_data;
				ta_n    <= 1'b0; last_beat <= 1'b1; st <= S_ACK;
			end

		S_LINE:
			// a beat of a line read: fetch the 64-bit word when the beat is not
			// in the one held, then answer the beat from it
			if (fb != 4'd0) fb <= fb - 4'd1;
			else if (ram_req || ram_got) begin
				if (ram_done) begin
					ram_got <= 1'b0;
					have64  <= 1'b1;   // (ram_rd64 took the word with the acknowledge)
					d_i     <= bsel[0] ? ram_data64[31:0] : ram_data64[63:32];
					ta_n    <= 1'b0;
					last_beat <= (beat == 2'd3);
					st <= S_ACK;
				end
			end else begin
				// the next beat: a RAM TOS's vector, in the held word (same
				// A3), or a new fetch (the vectors from ROM)
				if (ram_tos && vec_beat) begin
					d_i  <= bsel[0] ? 32'h00E00000 : 32'h00008000;
					ta_n <= 1'b0;
					last_beat <= (beat == 2'd3);
					st <= S_ACK;
				end else if (have64 && bsel[1] == ram_addr[3]) begin
					d_i  <= bsel[0] ? ram_rd64[31:0] : ram_rd64[63:32];
					ta_n <= 1'b0;
					last_beat <= (beat == 2'd3);
					st <= S_ACK;
				end else begin
					ram_req  <= 1'b1; ram_we <= 1'b0;
					ram_addr <= mem_addr({beat_addr, 2'b00}, 1'b1); ram_be <= 4'hF;
				end
			end

		S_LINEW:
			if (ram_done) begin
				ram_got <= 1'b0;
				ta_n    <= 1'b0;
				last_beat <= (beat == 2'd3);
				st <= S_ACK;
			end

		S_DEV: begin
			tmo <= tmo + 1'd1;
			if (dev_fail || tmo == 10'h3FF) begin
				dev_cs <= 1'b0;
				finish_tea;
			end
			else if (dev_done) begin
				dev_got <= 1'b0;
				tci_n   <= 1'b0;
				if (tsiz == 2'b00) begin
					// a long: the second 16-bit cycle at A + 2
					dev_hi   <= dev_data;
					dev_stb  <= 1'b1;
					dev_addr <= {la[23:2], 1'b1};
					dev_uds  <= be[1];
					dev_lds  <= be[0];
					dev_din  <= d_o[15:0];
					tmo      <= 10'd0;
					st <= S_DEV2;
				end else begin
					dev_cs <= 1'b0;
					d_i    <= la[1] ? {16'd0, dev_data} : {dev_data, 16'd0};
					ta_n   <= 1'b0; last_beat <= 1'b1; st <= S_ACK;
				end
			end
		end

		S_DEV2: begin
			tmo <= tmo + 1'd1;
			if (dev_fail || tmo == 10'h3FF) begin
				dev_cs <= 1'b0;
				finish_tea;
			end
			else if (dev_done) begin
				dev_got <= 1'b0;
				dev_cs  <= 1'b0;
				tci_n   <= 1'b0;
				d_i     <= {dev_hi, dev_data};
				ta_n    <= 1'b0; last_beat <= 1'b1; st <= S_ACK;
			end
		end

		S_IACK:
			if (iack_fin) begin
				iack_got <= 1'b0;
				if (iack_spv) begin tea_n <= 1'b0; err_beat <= 1'b1; end
				else begin
					ta_n <= 1'b0;
					if (iack_avv) avec_n <= 1'b0;
					d_i <= {4{iack_vecv}};
				end
				last_beat <= 1'b1;
				st <= S_ACK;
			end

		S_ACK: begin
			// the bus clock the processor samples the termination in (TA/TEA
			// were driven from the previous edge); a line write beat's data
			// is on the bus now
			if (last_beat) begin
				cycle_done <= 1'b1;
				st <= S_IDLE;
				// back-to-back: a new TS with our TA
				if (!ts_n && a_oe && !owned && !err_beat) start_transfer;
			end else begin
				beat <= beat + 1'd1;
				bsel <= bsel + 1'd1;
				fb   <= (ct60 && !is_fast) ? 4'(FALCON_BEAT_WAIT) : 4'd0;
				// a line write: the next beat's data is driven the bus clock
				// after the processor sampled this TA (S_WDATA takes it)
				st <= rd ? S_LINE : S_WDATA;
			end
		end

		S_WDATA: if (fb != 4'd0) fb <= fb - 4'd1; else begin
			ram_got   <= vec_beat;
			ram_req   <= !vec_beat; ram_we <= 1'b1;
			ram_addr  <= beat_addr; ram_be <= 4'hF;
			ram_wdata <= d_o;
			st <= S_LINEW;
		end

		default: st <= S_IDLE;
		endcase
	end
end

endmodule
