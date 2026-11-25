// vme_et4000.sv - a Nova ET4000 graphics card on the TT's VME bus
//
// Behavioural references: the ET4000AX emulation of the Hatari fork (src/video_et4000.c
// and src/vme_nova.c, branch et4000 of slaapliedje/hatari, itself following PCem's
// vid_svga.c / vid_et4000.c); EmuTOS bios/nova.c for the card's addresses and the way a
// driver sets it up; and ao486's vga.v as extended in z486_MiSTer for the ET4000's
// attribute register $16 (BSD, Aleksander Osman).  No code is copied from them.
//
// The card, as EmuTOS finds it ("Nova/ET4000 in Atari MegaSTe or TT"):
//   $FEC00000-$FECFFFFF  1 MB, the video memory window (bus_mem)
//   $FEDC0000-$FEDCFFFF  the ISA I/O ports: port p at +p (the VGA's are $3B0-$3DF)
// A word access is two byte accesses, the even address (the high byte) first.  Ports that
// the ET4000 does not have read $FF; anything else on the bus stays a bus error, which is
// what the drivers' probes look for (falcon_cpubus decodes the two windows).
//
// The video memory is 1 MB of 32-bit words, one byte a plane ("interleaved": plane p of
// word W is the byte at 4W+p), in DDR3 at $C00000 of the 24-bit map; that is also how a
// packed (chain-4) mode sees it, so 256 colours and HiColor are a plain frame buffer there.
// Every access of the CPU goes through the VGA's data path here (map mask, the four write
// modes, rotation, set/reset, the logical operations, the bit mask, the latches, read
// modes 0 and 1, odd/even and chain-4), with the Tseng segment register $3CD when the
// linear window is off (CRTC $36 bit 4).
//
// The picture is the MiSTer scaler's, in its frame buffer mode, straight from DDR3 (fb_*,
// pal_*), as for the ATW800/2: on HDMI only, at the HDMI mode's timing.  The scaler cannot
// show planes, so a planar write also writes the eight pixels of its word, four bits each,
// into a shadow of 8 bits a pixel at $A00000 (2 MB: word W's pixels at 8W), which is what
// a planar mode shows; its 16 entries of the scaler's palette are the attribute
// controller's palette (with the colour plane enable and colour select) through the DAC.
// A packed mode shows the video memory itself: 8 bits through the DAC, or the Sierra
// HiColor DAC's 15 or 16 bits.  Text and CGA modes are not shown (fb_en low: the TT's own
// picture), nor 24 bits.
//
// Input status 1 ($3DA) follows a free-running 640x480 60 Hz timing of the card's own,
// which is what the drivers' retrace waits and EmuTOS's count_vbls() need.
//
// A reset of the machine (sys_reset, the VME SYSRESET) resets the chip as an ISA reset
// does: the registers, not the video memory.

module vme_et4000 #(parameter CLK_HZ = 32000000) (
	input             clk,
	input             reset,          // cold reset
	input             sys_reset,      // the machine's reset

	input             bus_cs,
	input             bus_stb,
	input             bus_we,
	input             bus_mem,        // the video memory window, else the I/O ports
	input      [19:1] bus_addr,       // the offset in the window
	input             bus_uds,
	input             bus_lds,
	input      [15:0] bus_din,
	output reg [15:0] bus_dout,
	output reg        bus_ack,

	// the picture, for the scaler's frame buffer mode
	output reg        fb_en,
	output reg  [4:0] fb_format,
	output reg [11:0] fb_width,
	output reg [11:0] fb_height,
	output reg [23:0] fb_addr,        // in the 24-bit map
	output reg [13:0] fb_stride,
	output reg        pal_wr,
	output reg  [7:0] pal_addr,
	output reg [23:0] pal_rgb,

	// the video memory and the shadow, in 64-bit words of the 24-bit map: mem_be[7] and
	// bits 63:56 the byte at the lowest address; mem_ack for one clock
	output reg        mem_req,
	output reg        mem_we,
	output reg [23:3] mem_addr,
	output reg  [7:0] mem_be,
	output reg [63:0] mem_wdata,
	input      [63:0] mem_rdata,
	input             mem_ack
);

wire chip_reset = reset | sys_reset;

//----------------------------------------------------------------------------
// Registers
//----------------------------------------------------------------------------

reg  [7:0] miscout, vidsub, featcon, mode_3x8, banking;
reg  [2:0] seqaddr;
reg  [7:0] seq [0:7];
reg  [3:0] gdcaddr;
reg  [7:0] gdc [0:8];
reg  [4:0] attraddr;
reg        attr_pal_en, attrff;
reg  [7:0] attr [0:22];
reg  [5:0] crtcaddr;
reg  [7:0] crtc01, crtc07, crtc08, crtc09, crtc0c, crtc0d, crtc11, crtc12, crtc13;
reg  [7:0] crtc33, crtc35, crtc36;
reg  [7:0] dac_mask, dac_read, dac_write, dac_status, dac_r, dac_g;
reg  [1:0] dac_pos;
reg  [2:0] ramdac_state;
reg  [7:0] ramdac_ctrl;
reg  [1:0] hc_bpp;                // the Sierra DAC: 0 8 bits, 1 15, 2 16, 3 24/32
reg  [7:0] la, lb, lc, ld;        // the latches, planes 0..3
reg        pal_dirty;
wire       pal_take = pal_dirty && !chip_reset;   // the palette below starts again

// CRTC: all 64 read back; the ones the picture needs are copied above
reg  [7:0] crtc_ram [0:63];
reg  [7:0] crtc_q;
reg        crtc_we;
reg  [5:0] crtc_wa;
reg  [7:0] crtc_wd;
always @(posedge clk) begin
	if (crtc_we) crtc_ram[crtc_wa] <= crtc_wd;
	crtc_q <= crtc_ram[crtcaddr];
end

// the DAC: two copies, one read by the CPU and one by the palette for the scaler
reg [17:0] dac_a [0:255];
reg [17:0] dac_b [0:255];
reg [17:0] dac_qa, dac_qb;
reg  [7:0] dac_rb;
reg        dac_we;
reg  [7:0] dac_wa;
reg [17:0] dac_wd;
always @(posedge clk) begin
	if (dac_we) begin dac_a[dac_wa] <= dac_wd; dac_b[dac_wa] <= dac_wd; end
	dac_qa <= dac_a[dac_read];
	dac_qb <= dac_b[dac_rb];
end

// the data path
wire       chain4      = seq[4][3];
wire       chain2_wr   = !seq[4][2];
wire [3:0] writemask   = seq[2][3:0];
wire [1:0] writemode   = gdc[5][1:0];
wire       readmode    = gdc[5][3];
wire       chain2_rd   = gdc[5][4];
wire [1:0] readplane   = gdc[4][1:0];
wire       linear      = crtc36[4];

//----------------------------------------------------------------------------
// Input status 1: a 640x480 60 Hz timing (800 x 525 at 25.175 MHz)
//----------------------------------------------------------------------------

localparam integer LINE_CLKS = CLK_HZ / 31469;
reg [11:0] hcnt = 0;
reg  [9:0] vcnt = 0;
always @(posedge clk) begin
	if (hcnt == LINE_CLKS - 1) begin
		hcnt <= 0;
		vcnt <= (vcnt == 10'd524) ? 10'd0 : vcnt + 1'd1;
	end
	else hcnt <= hcnt + 1'd1;
end
wire vretrace = vcnt >= 10'd490 && vcnt < 10'd492;
wire dispoff  = vcnt >= 10'd480 || hcnt >= (LINE_CLKS * 640) / 800;
wire [7:0] status1 = {4'b0000, vretrace, 2'b00, dispoff};

//----------------------------------------------------------------------------
// The memory data path for one byte (as svga_write_linear / svga_read_linear)
//----------------------------------------------------------------------------

// the byte's offset in the window, then in the video memory
function [19:0] map_lin(input [19:0] off, input wr);
	map_lin = linear ? off : {wr ? banking[3:0] : banking[7:4], off[15:0]};
endfunction

function [7:0] rotr(input [7:0] v, input [2:0] n);
	reg [15:0] d;
	begin
		d = {v, v} >> n;
		rotr = d[7:0];
	end
endfunction

function [7:0] logic_op(input [7:0] d, input [7:0] l, input [7:0] mask);
	case (gdc[3][4:3])
	2'd0: logic_op = (d & mask) | (l & ~mask);
	2'd1: logic_op = (d | ~mask) & l;
	2'd2: logic_op = (d & mask) | l;
	2'd3: logic_op = (d & mask) ^ l;
	endcase
endfunction

reg        s_we;                  // the byte being done: a write
reg [19:0] s_off;                 //   its offset in the window
reg  [7:0] s_val;                 //   the CPU's byte
reg [63:0] img;                   // the video memory's 64-bit word the cycle is in

reg [19:0] p_lin, p_w;
reg  [3:0] p_wm;
reg  [1:0] p_rp;
reg        p_in;                  // in the 1 MB
reg  [7:0] p_old [0:3];
reg  [7:0] p_new [0:3];
reg  [7:0] p_rd;
reg  [7:0] p_d, p_mask, p_rot;
reg  [7:0] t0, t1, t2, t3;
integer    pi;
always @* begin
	p_lin = map_lin(s_off, s_we);
	p_rp  = readplane;
	if (chain4) begin
		p_w  = {2'b00, p_lin[19:2]};
		p_wm = 4'b0001 << p_lin[1:0];
	end
	else if (s_we ? chain2_wr : chain2_rd) begin
		p_w  = {p_lin[19:1], 1'b0};
		p_wm = (writemask & 4'b0101) << p_lin[0];
		p_rp = {readplane[1], p_lin[0]};
	end
	else begin
		p_w  = p_lin;
		p_wm = writemask;
	end
	p_in = p_w[19:18] == 2'b00;
	for (pi = 0; pi < 4; pi = pi + 1) p_old[pi] = img[63 - 8 * ({p_w[0], 2'b00} + pi) -: 8];

	// write: the planes' new bytes
	p_rot  = rotr(s_val, gdc[3][2:0]);
	p_mask = gdc[8];
	if (writemode == 2'd3) p_mask = gdc[8] & p_rot;
	for (pi = 0; pi < 4; pi = pi + 1) begin
		case (writemode)
		2'd0: p_d = gdc[1][pi] ? {8{gdc[0][pi]}} : p_rot;
		2'd2: p_d = {8{s_val[pi]}};
		default: p_d = {8{gdc[0][pi]}};
		endcase
		case (pi)
		0: p_new[pi] = (writemode == 2'd1) ? la : logic_op(p_d, la, p_mask);
		1: p_new[pi] = (writemode == 2'd1) ? lb : logic_op(p_d, lb, p_mask);
		2: p_new[pi] = (writemode == 2'd1) ? lc : logic_op(p_d, lc, p_mask);
		default: p_new[pi] = (writemode == 2'd1) ? ld : logic_op(p_d, ld, p_mask);
		endcase
	end

	// read: mode 0 a plane, mode 1 the colour compare
	t0 = (p_old[0] ^ {8{gdc[2][0]}}) & {8{gdc[7][0]}};
	t1 = (p_old[1] ^ {8{gdc[2][1]}}) & {8{gdc[7][1]}};
	t2 = (p_old[2] ^ {8{gdc[2][2]}}) & {8{gdc[7][2]}};
	t3 = (p_old[3] ^ {8{gdc[2][3]}}) & {8{gdc[7][3]}};
	if (!p_in)        p_rd = 8'hFF;
	else if (chain4)  p_rd = p_old[p_lin[1:0]];
	else if (readmode) p_rd = ~(t0 | t1 | t2 | t3);
	else              p_rd = p_old[p_rp];
end

// a word's eight pixels, 4 bits each in a byte, leftmost at the lowest address
function [63:0] expand(input [31:0] w);
	integer k;
	begin
		for (k = 0; k < 8; k = k + 1)
			expand[63 - 8 * k -: 8] = {4'h0, w[7 - k + 24], w[7 - k + 16], w[7 - k + 8], w[7 - k]};
	end
endfunction
function [31:0] planes_of(input [63:0] im, input h);
	planes_of = h ? {im[7:0], im[15:8], im[23:16], im[31:24]} : {im[39:32], im[47:40], im[55:48], im[63:56]};
endfunction

//----------------------------------------------------------------------------
// The bus
//----------------------------------------------------------------------------

localparam B_IDLE = 4'd0, B_IOA = 4'd1, B_IOB = 4'd2, B_MRD = 4'd3, B_MSTEP = 4'd4,
           B_MWR = 4'd5, B_MSH0 = 4'd6, B_MSH1 = 4'd7, B_ACK = 4'd8;
reg  [3:0] bst;
reg        c_we, c_mem, c_uds, c_lds;
reg [19:1] c_addr;
reg [15:0] c_din;
reg        step;                  // 0 the even byte (uds), 1 the odd one
reg  [7:0] be8;
reg        sh0, sh1;              // shadow words to write: the even and the odd one
reg [17:0] sh_w;                  // their 64-bit word in the video memory

wire [15:0] port  = {c_addr[15:1], step};
wire  [7:0] wval  = step ? c_din[7:0] : c_din[15:8];
wire [19:0] c_off = {c_addr, step};
// the first byte's word, which the cycle's other byte shares (an even offset and the next)
wire [19:0] w_off = {c_addr, !c_uds};

// whether a cycle needs the old contents: reads, and planar writes (for the shadow) but
// for ones of all four planes
wire need_read = !c_we || (!chain4 && (chain2_wr || writemask != 4'hF));

// the word of the first byte (combinational, from the cycle's offset)
reg [19:0] f_lin, f_w;
always @* begin
	f_lin = map_lin(w_off, c_we);
	if (chain4) f_w = {2'b00, f_lin[19:2]};
	else if (c_we ? chain2_wr : chain2_rd) f_w = {f_lin[19:1], 1'b0};
	else f_w = f_lin;
end

always @* begin
	s_we  = c_we;
	s_off = c_off;
	s_val = wval;
end

integer ii;
always @(posedge clk) begin
	bus_ack <= 0;
	dac_we  <= 0;
	crtc_we <= 0;

	case (bst)
	B_IDLE:
		if (bus_stb && bus_cs) begin
			c_we <= bus_we; c_mem <= bus_mem; c_uds <= bus_uds; c_lds <= bus_lds;
			c_addr <= bus_addr; c_din <= bus_din;
			step <= !bus_uds;
			bus_dout <= 16'hFFFF;
			be8 <= 8'h00; sh0 <= 0; sh1 <= 0;
			bst <= bus_mem ? B_MRD : B_IOA;
		end

	// I/O: a clock for the DAC's and the CRTC's read ports, then the access
	B_IOA: bst <= B_IOB;
	B_IOB: begin
		if (c_we) io_write(port, wval);
		else begin
			if (step) bus_dout[7:0] <= io_read(port);
			else      bus_dout[15:8] <= io_read(port);
			io_read_fx(port);
		end
		if (!step && c_lds) begin step <= 1; bst <= B_IOA; end
		else bst <= B_ACK;
	end

	// memory: the 64-bit word, if needed
	B_MRD:
		if (!need_read || f_w[19:18] != 2'b00) begin
			img <= 64'hFFFFFFFFFFFFFFFF;
			bst <= B_MSTEP;
		end
		else if (!mem_req) begin
			mem_req <= 1; mem_we <= 0; mem_addr <= {4'hC, f_w[17:1]};
		end
		else if (mem_ack) begin
			mem_req <= 0;
			img <= mem_rdata;
			bst <= B_MSTEP;
		end

	// a byte: the cycle's even one first
	B_MSTEP: begin
		if (c_we) begin
			if (p_in) begin
				for (ii = 0; ii < 4; ii = ii + 1)
					if (p_wm[ii]) begin
						img[63 - 8 * ({p_w[0], 2'b00} + ii) -: 8] <= p_new[ii];
						be8[7 - ({p_w[0], 2'b00} + ii)] <= 1'b1;
					end
				if (!chain4 && p_wm != 4'h0) begin
					if (p_w[0]) sh1 <= 1; else sh0 <= 1;
				end
			end
		end
		else begin
			if (step) bus_dout[7:0] <= p_rd; else bus_dout[15:8] <= p_rd;
			if (!chain4) begin
				if (p_in) begin la <= p_old[0]; lb <= p_old[1]; lc <= p_old[2]; ld <= p_old[3]; end
				else begin la <= 8'hFF; lb <= 8'hFF; lc <= 8'hFF; ld <= 8'hFF; end
			end
		end
		sh_w <= f_w[17:0];
		if (!step && c_lds) step <= 1;
		else bst <= (c_we && f_w[19:18] == 2'b00) ? B_MWR : B_ACK;
	end

	B_MWR:
		if (be8 == 8'h00) bst <= B_MSH0;
		else if (!mem_req) begin
			mem_req <= 1; mem_we <= 1; mem_addr <= {4'hC, sh_w[17:1]};
			mem_be <= be8; mem_wdata <= img;
		end
		else if (mem_ack) begin
			mem_req <= 0;
			bst <= B_MSH0;
		end

	// the shadow's words: eight pixels for each 32-bit word written
	B_MSH0:
		if (!sh0) bst <= B_MSH1;
		else if (!mem_req) begin
			mem_req <= 1; mem_we <= 1; mem_addr <= {3'b101, sh_w[17:1], 1'b0};
			mem_be <= 8'hFF; mem_wdata <= expand(planes_of(img, 1'b0));
		end
		else if (mem_ack) begin
			mem_req <= 0;
			bst <= B_MSH1;
		end
	B_MSH1:
		if (!sh1) bst <= B_ACK;
		else if (!mem_req) begin
			mem_req <= 1; mem_we <= 1; mem_addr <= {3'b101, sh_w[17:1], 1'b1};
			mem_be <= 8'hFF; mem_wdata <= expand(planes_of(img, 1'b1));
		end
		else if (mem_ack) begin
			mem_req <= 0;
			bst <= B_ACK;
		end

	B_ACK: begin
		bus_ack <= 1;
		bst <= B_IDLE;
	end

	default: bst <= B_IDLE;
	endcase

	if (chip_reset) begin
		miscout <= 8'h00; vidsub <= 8'h00; featcon <= 8'h00; mode_3x8 <= 8'h00; banking <= 8'h00;
		seqaddr <= 0; gdcaddr <= 0; attraddr <= 0; attr_pal_en <= 0; attrff <= 0; crtcaddr <= 0;
		for (ii = 0; ii < 8; ii = ii + 1) seq[ii] <= 8'h00;
		for (ii = 0; ii < 9; ii = ii + 1) gdc[ii] <= 8'h00;
		for (ii = 0; ii < 23; ii = ii + 1) attr[ii] <= 8'h00;
		crtc01 <= 0; crtc07 <= 0; crtc08 <= 0; crtc09 <= 0; crtc0c <= 0; crtc0d <= 0;
		crtc11 <= 0; crtc12 <= 0; crtc13 <= 0; crtc33 <= 0; crtc35 <= 0; crtc36 <= 0;
		dac_mask <= 8'hFF; dac_read <= 0; dac_write <= 0; dac_status <= 0; dac_pos <= 0;
		ramdac_state <= 0; ramdac_ctrl <= 0; hc_bpp <= 0;
		la <= 0; lb <= 0; lc <= 0; ld <= 0;
		pal_dirty <= 1;
	end
	if (reset) begin
		bst <= B_IDLE; mem_req <= 0;
	end
	if (pal_take) pal_dirty <= 0;
end

//----------------------------------------------------------------------------
// The I/O ports (as ET4000_IO_WriteByte / ET4000_IO_ReadByte); $3Bx and $3Dx both
//----------------------------------------------------------------------------

function [7:0] io_read(input [15:0] p);
	case (p)
	16'h3C0: io_read = {2'b00, attr_pal_en, attraddr};
	16'h3C1: io_read = (attraddr < 5'd23) ? attr[attraddr] : 8'h00;
	16'h3C2: io_read = 8'h10;                 // monitor sense: a colour display
	16'h3C3: io_read = vidsub;
	16'h3C4: io_read = {5'd0, seqaddr};
	16'h3C5: io_read = seq[seqaddr] | (seqaddr == 3'd7 ? 8'h04 : 8'h00);
	16'h3C6: io_read = (ramdac_state == 3'd4) ? ramdac_ctrl : dac_mask;
	16'h3C7: io_read = dac_status;
	16'h3C8: io_read = dac_write;
	16'h3C9: io_read = {2'b00, dac_pos == 2'd0 ? dac_qa[17:12] : dac_pos == 2'd1 ? dac_qa[11:6] : dac_qa[5:0]};
	16'h3CA: io_read = featcon;
	16'h3CC: io_read = miscout;
	16'h3CD: io_read = banking;
	16'h3CE: io_read = {4'd0, gdcaddr};
	16'h3CF: io_read = (gdcaddr < 4'd9) ? gdc[gdcaddr] : 8'h00;
	16'h3B4, 16'h3D4: io_read = {2'b00, crtcaddr};
	16'h3B5, 16'h3D5: io_read = crtc_q;
	16'h3BA, 16'h3DA: io_read = status1;
	default: io_read = 8'hFF;
	endcase
endfunction

task io_write(input [15:0] p, input [7:0] v);
	begin
		case (p)
		16'h3C0: begin
			if (!attrff) begin attraddr <= v[4:0]; attr_pal_en <= v[5]; end
			else begin
				if (attraddr < 5'd23) attr[attraddr] <= v;
				pal_dirty <= 1;
			end
			attrff <= !attrff;
		end
		16'h3C2: miscout <= v;
		16'h3C3: vidsub <= v;
		16'h3C4: seqaddr <= v[2:0];
		16'h3C5: seq[seqaddr] <= v;
		16'h3C6: begin
			if (ramdac_state == 3'd4) begin
				if (v != 8'hFF) begin
					ramdac_ctrl <= v;
					case ({v[7], v[6], v[0]})
					3'd0: hc_bpp <= 2'd0;
					3'd2, 3'd3: hc_bpp <= 2'd3;
					3'd4, 3'd5: hc_bpp <= 2'd1;
					3'd6: hc_bpp <= 2'd2;
					3'd7: hc_bpp <= v[2] ? 2'd3 : 2'd2;
					default: ;
					endcase
					pal_dirty <= 1;
				end
			end
			else begin
				dac_mask <= v;
				pal_dirty <= 1;
			end
			ramdac_state <= 0;
		end
		16'h3C7: begin ramdac_state <= 0; dac_read <= v; dac_pos <= 0; end
		16'h3C8: begin ramdac_state <= 0; dac_write <= v; dac_read <= v - 1'd1; dac_pos <= 0; end
		16'h3C9: begin
			ramdac_state <= 0;
			dac_status <= 0;
			case (dac_pos)
			2'd0: begin dac_r <= v; dac_pos <= 2'd1; end
			2'd1: begin dac_g <= v; dac_pos <= 2'd2; end
			default: begin
				dac_we <= 1; dac_wa <= dac_write; dac_wd <= {dac_r[5:0], dac_g[5:0], v[5:0]};
				dac_write <= dac_write + 1'd1;
				dac_pos <= 0;
				pal_dirty <= 1;
			end
			endcase
		end
		16'h3CD: banking <= v;
		16'h3CE: gdcaddr <= v[3:0];
		16'h3CF: begin
			if (gdcaddr < 4'd9) gdc[gdcaddr] <= v;
			if (gdcaddr == 4'd5) pal_dirty <= 1;
		end
		16'h3B4, 16'h3D4: crtcaddr <= v[5:0];
		16'h3B5, 16'h3D5:
			if (!(crtcaddr < 6'd7 && crtc11[7])) begin
				crtc_we <= 1; crtc_wa <= crtcaddr;
				crtc_wd <= (crtcaddr == 6'd7 && crtc11[7]) ? ((crtc07 & ~8'h10) | (v & 8'h10)) : v;
				case (crtcaddr)
				6'h01: crtc01 <= v;
				6'h07: crtc07 <= crtc11[7] ? ((crtc07 & ~8'h10) | (v & 8'h10)) : v;
				6'h08: crtc08 <= v;
				6'h09: crtc09 <= v;
				6'h0C: crtc0c <= v;
				6'h0D: crtc0d <= v;
				6'h11: crtc11 <= v;
				6'h12: crtc12 <= v;
				6'h13: crtc13 <= v;
				6'h33: crtc33 <= v;
				6'h35: crtc35 <= v;
				6'h36: crtc36 <= v;
				default: ;
				endcase
			end
		16'h3BA, 16'h3DA: featcon <= v;
		16'h3B8, 16'h3D8: mode_3x8 <= v;
		default: ;
		endcase
	end
endtask

// the side effects of reads
task io_read_fx(input [15:0] p);
	case (p)
	16'h3C6: if (ramdac_state != 3'd4) ramdac_state <= ramdac_state + 1'd1;
	16'h3C7, 16'h3C8: ramdac_state <= 0;
	16'h3C9: begin
		ramdac_state <= 0;
		dac_status <= 8'h03;
		if (dac_pos == 2'd2) begin dac_pos <= 0; dac_read <= dac_read + 1'd1; end
		else dac_pos <= dac_pos + 1'd1;
	end
	16'h3BA, 16'h3DA: attrff <= 0;
	default: ;
	endcase
endtask

//----------------------------------------------------------------------------
// The picture
//----------------------------------------------------------------------------

wire        planar  = gdc[5][6:5] == 2'b00;
wire        pmode  = gdc[5][6];
wire        card_on = vidsub[0] && !seq[1][5] && attr_pal_en && gdc[6][0];
wire  [8:0] hdisp   = {1'b0, crtc01} + 1'd1;
wire [11:0] wchars  = {hdisp, 3'b000};
wire [11:0] wpack   = attr[22][5] ? {wchars[10:0], 1'b0} : wchars;
wire [10:0] dispend = {1'b0, crtc07[6], crtc07[1], crtc12} + (crtc35[2] ? 11'h400 : 11'h000) + 1'd1;
wire [11:0] rowbytes = (crtc13 == 8'd0) ? 12'd2048 : {1'b0, crtc13, 3'b000};
wire [19:0] ma = {({crtc33[1:0], crtc0c, crtc0d} + {16'd0, crtc08[6:5]}), 2'b00};
wire [10:0] lines = dispend >> (crtc09[7] + (crtc09[4:0] == 5'd1));

always @(posedge clk) begin
	fb_height <= {1'b0, lines};
	if (planar) begin
		fb_en     <= card_on;
		fb_format <= 5'b00011;
		fb_width  <= wchars;
		fb_addr   <= 24'hA00000 + {3'b000, ma, 1'b0};
		fb_stride <= {1'b0, rowbytes, 1'b0};
	end
	else if (pmode && hc_bpp == 2'd0) begin
		fb_en     <= card_on;
		fb_format <= 5'b00011;
		fb_width  <= wpack;
		fb_addr   <= {4'hC, ma};
		fb_stride <= {2'b00, rowbytes};
	end
	else if (pmode && hc_bpp != 2'd3) begin
		fb_en     <= card_on;
		fb_format <= (hc_bpp == 2'd1) ? 5'b11100 : 5'b10100;
		fb_width  <= {1'b0, wpack[11:1]};
		fb_addr   <= {4'hC, ma};
		fb_stride <= {2'b00, rowbytes};
	end
	else begin
		fb_en     <= 0;
		fb_format <= 5'b00011;
		fb_width  <= wchars;
		fb_addr   <= {4'hC, ma};
		fb_stride <= {2'b00, rowbytes};
	end
end

// the scaler's palette: all 256 entries again whenever the DAC, its mask, the attribute
// controller or the mode changes.  A planar pixel's 4 bits go through the attribute
// controller's palette (the colour plane enable, then the colour select); a packed one's 8
// bits straight to the DAC; either through the pixel mask.
reg        p_run = 0;
reg  [7:0] p_idx;
reg        p_v1, p_v2;
reg  [7:0] p_i1, p_i2;
wire [3:0] p_pix = p_idx[3:0] & attr[18][3:0];
wire [7:0] p_atc = attr[p_pix];
wire [7:0] p_col = attr[16][7] ? {attr[20][3:0], p_atc[3:0]} : {attr[20][3:2], p_atc[5:0]};
always @(posedge clk) begin
	if (pal_take) begin p_run <= 1; p_idx <= 0; end
	else if (p_run) begin
		p_idx <= p_idx + 1'd1;
		if (p_idx == 8'hFF) p_run <= 0;
	end
	dac_rb <= (planar ? p_col : p_idx) & dac_mask;
	p_i1 <= p_idx; p_v1 <= p_run && !pal_take;
	p_i2 <= p_i1;  p_v2 <= p_v1;
	pal_wr   <= p_v2;
	pal_addr <= p_i2;
	pal_rgb  <= {dac_qb[17:12], dac_qb[17:16], dac_qb[11:6], dac_qb[11:10], dac_qb[5:0], dac_qb[5:4]};
	if (reset) begin p_run <= 0; p_v1 <= 0; p_v2 <= 0; end
end

endmodule
