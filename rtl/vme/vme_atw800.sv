// vme_atw800.sv - an ATW800/2 graphics card ("Seurat") on the TT's VME bus: its registers
//
// Behavioural references: the ATW800/2 emulation of the Hatari fork (src/vme_atw800.c,
// branch et4000 of slaapliedje/hatari), which follows the card's Programmer's Manual
// V1.0a; and what atari-sysv-sp1's Xatw and tools/atw measured on a real V0205 card.
//
// The card is 2 MB of video memory at $FEC00000 or $FEA00000, or 4 MB at $FEA00000, by
// its jumpers; falcon_cpubus decodes it and keeps the memory in DDR3.  A 4 MB card
// (mem4) starts in a 2 MB layout, its upper half a mirror of the lower, until the VTG's
// word 15 is 3 (lay4).  At the top of the layout the FPGA has its registers, which are
// here (bus_addr is the offset in that last 4 KB; the offsets below are a 2 MB layout's):
//   +$1FF000  LUT: 256 x RGB565 words
//   +$1FF200  32 bytes of id, read only: " cpm" at offset 24 is what xVDI looks for
//   +$1FF800  the video timing generator: 16 words; word 15 is the memory register
//   +$1FF900  the 2D engine: 64 bytes, write only: every offset reads the status word,
//             bit 0 busy.  As atari-sysv-sp1 found it on a V0205 card (xserver-r6
//             hw/atw/atwAccel.c; tools/atw atwblit, atwfill, atwtorture, byte-exact
//             there):
//               +0 source, +4 destination (longs, offsets in the card)
//               +8, +10 their strides (signed words), +12 bytes a row, +14 rows
//               +16 command: 1 copy, addresses ascending; 3 copy descending (source
//                   and destination name the LAST byte, the strides are negative);
//                   5 fill: a row is the first 64 source bytes, then the second 32 of
//                   them again and again (bit 0 go, bit 1 descending, bit 2 fill)
//             Here it moves up to eight bytes at a time through a 64-bit port to the
//             video memory (mem_*), with the result of moving byte by byte in the order
//             the command names: what a copy gives where that order is the wrong one for
//             an overlap is not the card's, nor is a fill over its own source.  A fill
//             ignores bit 1.  No rows or no bytes: nothing is done.  A command while busy
//             is dropped.
//   +$1FFAC0  the FPGA's Transputer link: reads 0, nothing connected
// and, with bus_aux, the bus CPLD's block at $FEFFFA80-$FEFFFAFF: its version (4) in the
// low byte of $FEFFFAD8, the C011 link beside it reads 0.
//
// The FPGA keeps its mode over the Atari's reset: only a cold reset clears the VTG.  The
// LUT is block RAM and is not cleared.
//
// The picture is the MiSTer scaler's to make, in its frame buffer mode, straight from
// the video memory (fb_*, pal_*): so it is on HDMI only, at the HDMI mode's own timing,
// and the VTG's timing words and PLL are kept and not used.  From the VTG it takes
//   word 0   control: bit 0 on, bits 5:4 the depth: 1 = 8 bits through the LUT,
//            2 = 16 bits (RGB565, low byte first), 3 = 32 bits (bytes R, G, B, x) -- as
//            tools/atw atwtest showed them on the real card.  Depth 0, 1 bit, is not
//            shown.
//   word 4, word 8   pixels a line, lines
//   words 12, 13     the display's start in the video memory, in bytes: taken as the
//            VTG goes from off to on, as measured
// fb_en is the VTG on, since it was last set on: a reset of the machine (sys_reset)
// gives the screen back to the TT's own picture until a program sets the card up again.

module vme_atw800 (
	input             clk,
	input             reset,          // cold reset
	input             sys_reset,      // the machine's reset
	input             mem4,           // a 4 MB card
	output            lay4,           //   in its 4 MB layout

	input             bus_cs,
	input             bus_stb,
	input             bus_we,
	input             bus_aux,        // the CPLD's block, else the top page of the video memory
	input      [11:1] bus_addr,
	input             bus_uds,
	input             bus_lds,
	input      [15:0] bus_din,
	output reg [15:0] bus_dout,
	output reg        bus_ack,

	output    [255:0] vtg,            // the timing generator's words, word 0 in [15:0]

	// the picture, for the scaler: its frame buffer mode's format, and the LUT's writes
	output            fb_en,
	output      [4:0] fb_format,
	output     [11:0] fb_width,
	output     [11:0] fb_height,
	output reg [21:0] fb_start,       // offset in the card
	output     [13:0] fb_stride,      // bytes a line
	output reg        pal_wr,
	output reg  [7:0] pal_addr,
	output reg [23:0] pal_rgb,

	// the 2D engine's port to the video memory, in 64-bit words: mem_addr is the word's
	// offset in the card, mem_be[7] and bits 63:56 the byte at the lowest address; mem_ack
	// for one clock, and the next request may be there on the clock after it
	output reg        mem_req,
	output reg        mem_we,
	output reg [21:3] mem_addr,
	output reg  [7:0] mem_be,
	output reg [63:0] mem_wdata,
	input      [63:0] mem_rdata,
	input             mem_ack,
	output            blit_busy
);

localparam CPLD_VERSION = 8'd4;

wire [11:0] off = {bus_addr, 1'b0};
wire is_lut  = !bus_aux && off[11:9] == 3'd0;           // 000-1FF
wire is_info = !bus_aux && off[11:5] == 7'h10;          // 200-21F
wire is_vtg  = !bus_aux && off[11:5] == 7'h40;          // 800-81F
wire is_blit = !bus_aux && off[11:6] == 6'h24;          // 900-93F

// "Seurat v0106 MiSTer.", four zeroes, " cpm", three zeroes, by words
function [15:0] info(input [3:0] w);
	case (w)
		4'd0: info = "Se"; 4'd1: info = "ur"; 4'd2: info = "at"; 4'd3: info = " v";
		4'd4: info = "01"; 4'd5: info = "06"; 4'd6: info = " M"; 4'd7: info = "iS";
		4'd8: info = "Te"; 4'd9: info = "r.";
		4'd12: info = " c"; 4'd13: info = "pm";
		default: info = 16'h0000;
	endcase
endfunction

reg [15:0] vtg_r [0:15];
reg [15:0] blit_r [0:7];          // source, destination, strides, bytes, rows
assign lay4 = mem4 && vtg_r[15][1];
genvar g;
generate for (g = 0; g < 16; g = g + 1) begin : vtgw
	assign vtg[g*16 +: 16] = vtg_r[g];
end endgenerate

(* ramstyle = "M10K, no_rw_check" *) reg [15:0] lut [0:255];
reg  [15:0] lut_q;
reg         lut_we;
reg   [7:0] lut_wa;
reg  [15:0] lut_wd;
always @(posedge clk) lut_q <= lut[bus_addr[8:1]];
always @(posedge clk) if (lut_we) lut[lut_wa] <= lut_wd;
always @(posedge clk) begin
	pal_wr   <= lut_we;
	pal_addr <= lut_wa;
	pal_rgb  <= {lut_wd[15:11], lut_wd[15:13], lut_wd[10:5], lut_wd[10:9], lut_wd[4:0], lut_wd[4:2]};
end

reg        fb_on;                 // the VTG was set on since the machine's reset
wire [1:0] fb_depth = vtg_r[0][5:4];
assign fb_en     = fb_on && vtg_r[0][0] && fb_depth != 2'd0;
assign fb_format = fb_depth == 2'd1 ? 5'b00011 : fb_depth == 2'd2 ? 5'b10100 : 5'b00110;
assign fb_width  = vtg_r[4][11:0];
assign fb_height = vtg_r[8][11:0];
assign fb_stride = fb_depth == 2'd1 ? {2'b00, vtg_r[4][11:0]} : fb_depth == 2'd2 ? {1'b0, vtg_r[4][11:0], 1'b0} : {vtg_r[4][11:0], 2'b00};

reg        ph;                    // the second clock of a LUT access: its word is out
reg        a_we, a_u, a_l;
reg [15:0] a_din;

function [15:0] merge(input [15:0] old, input [15:0] d, input u, input l);
	merge = {u ? d[15:8] : old[15:8], l ? d[7:0] : old[7:0]};
endfunction

// ---- the 2D engine ----
// A row goes in pieces of up to eight bytes, each inside one 64-bit word of the
// destination: the one or two words of the source with its bytes are read, then the piece
// is written.  A piece is no longer than the distance from the source to a destination
// that follows it closely in the command's order, so the result is that of moving byte by
// byte.  A fill reads its first 64 bytes as a copy does, and keeps the four words of the
// destination that repeat from there (the second 32 bytes): the rest of the row is written
// without a read.
localparam B_IDLE = 3'd0, B_CHK = 3'd1, B_RA = 3'd2, B_RB = 3'd3, B_WRW = 3'd4;
reg  [2:0] bst = B_IDLE;
reg [21:0] b_src, b_dst;          // the row's first byte
reg [15:0] b_x;                   // the byte of the row
reg [15:0] b_rows;                // rows to go
reg        b_dir, b_fill;
reg [63:0] b_a;                   // the first source word of a piece that has two
reg [63:0] b_pat [0:3];           // a fill's repeating words
reg        blit_go, go_dir, go_fill;
assign blit_busy = bst != B_IDLE || blit_go;

function [3:0] min4(input [3:0] a, input [3:0] b);
	min4 = a < b ? a : b;
endfunction
function [63:0] merge8(input [63:0] old, input [63:0] d, input [7:0] be);
	integer k;
	for (k = 0; k < 8; k = k + 1) merge8[8*k +: 8] = be[k] ? d[8*k +: 8] : old[8*k +: 8];
endfunction

wire [21:0] b_mask = lay4 ? 22'h3FFFFF : 22'h1FFFFF;
wire [21:0] b_sa   = (b_dir ? b_src - {6'd0, b_x} : b_src + {6'd0, b_x}) & b_mask;
wire [21:0] b_da   = (b_dir ? b_dst - {6'd0, b_x} : b_dst + {6'd0, b_x}) & b_mask;
wire        b_rep  = b_fill && b_x[15:6] != 10'd0;        // a fill past its first 64 bytes
wire [15:0] b_rem  = blit_r[6] - b_x;
// the piece ends with the destination's word, the row, a fill's first 64 bytes, or where a
// byte of it would be read after it was written
wire  [3:0] b_room = b_dir ? {1'b0, b_da[2:0]} + 4'd1 : 4'd8 - {1'b0, b_da[2:0]};
wire  [3:0] b_lim1 = (b_rem[15:3] != 13'd0) ? 4'd8 : {1'b0, b_rem[2:0]};
wire [21:0] b_gap  = (b_dir ? b_src - b_dst : b_dst - b_src) & b_mask;
wire  [3:0] b_lim2 = (!b_rep && b_gap[21:3] == 19'd0 && b_gap[2:0] != 3'd0) ? {1'b0, b_gap[2:0]} : 4'd8;
wire  [3:0] b_lim3 = (b_fill && !b_rep && b_x[5:3] == 3'd7) ? 4'd8 - {1'b0, b_x[2:0]} : 4'd8;
wire  [3:0] b_n    = min4(min4(b_room, b_lim1), min4(b_lim2, b_lim3));
wire  [2:0] b_n1   = b_n[2:0] - 3'd1;
wire        b_last = b_rem == {12'd0, b_n};
// its lowest addresses, the words of the source, the destination's word
wire [21:0] b_dlo  = b_dir ? b_da - {19'd0, b_n1} : b_da;
wire [21:0] b_slo  = b_dir ? (b_sa - {19'd0, b_n1}) & b_mask : b_sa;
wire        b_two  = {1'b0, b_slo[2:0]} + {1'b0, b_n1} > 4'd7;
wire [21:3] b_lb   = (b_slo[21:3] + 19'd1) & b_mask[21:3];
wire  [7:0] b_be   = (8'hFF << (4'd8 - b_n)) >> b_dlo[2:0];
wire [127:0] b_s16 = b_two ? {b_a, mem_rdata} : {mem_rdata, mem_rdata};
wire [127:0] b_sh  = b_s16 << {b_slo[2:0], 3'b000};
wire [63:0] b_copy = b_sh[127:64] >> {b_dlo[2:0], 3'b000};
// a fill: the destination's words of the row counted from its first, 0 up; from word 9 each
// is the one four before it, so words 5 to 8 are kept, and 8 is 4 but for its first bytes
wire [15:0] b_lx   = {13'd0, b_dst[2:0]} + b_x;
wire  [1:0] b_pi   = b_lx[4:3];

always @(posedge clk) begin
	if (reset) begin
		bst <= B_IDLE; mem_req <= 0; mem_we <= 0;
	end
	else case (bst)
	B_IDLE: if (blit_go && blit_r[6] != 16'd0 && blit_r[7] != 16'd0) begin
		b_src  <= {blit_r[0][5:0], blit_r[1]};
		b_dst  <= {blit_r[2][5:0], blit_r[3]};
		b_rows <= blit_r[7];
		b_dir  <= go_dir && !go_fill; b_fill <= go_fill;
		b_x    <= 16'd0;
		bst    <= B_CHK;
	end
	B_CHK: begin
		mem_req <= 1;
		if (b_rep) begin
			mem_we <= 1; mem_addr <= b_da[21:3]; mem_be <= b_be; mem_wdata <= b_pat[b_pi];
			bst <= B_WRW;
		end
		else begin
			mem_we <= 0; mem_addr <= b_slo[21:3];
			bst <= B_RA;
		end
	end
	B_RA, B_RB: if (mem_ack) begin
		if (bst == B_RA && b_two) begin
			b_a <= mem_rdata; mem_addr <= b_lb;
			bst <= B_RB;
		end
		else begin
			mem_we <= 1; mem_addr <= b_dlo[21:3]; mem_be <= b_be; mem_wdata <= b_copy;
			// words 4 to 8 of a fill's row
			if (b_fill && b_lx[6:5] != 2'd0) b_pat[b_pi] <= merge8(b_pat[b_pi], b_copy, b_be);
			bst <= B_WRW;
		end
	end
	B_WRW: if (mem_ack) begin
		mem_req <= 0; mem_we <= 0;
		if (b_last) begin
			b_x    <= 16'd0;
			b_src  <= b_src + {{6{blit_r[4][15]}}, blit_r[4]};
			b_dst  <= b_dst + {{6{blit_r[5][15]}}, blit_r[5]};
			b_rows <= b_rows - 16'd1;
			bst    <= (b_rows == 16'd1) ? B_IDLE : B_CHK;
		end
		else begin
			b_x <= b_x + {12'd0, b_n};
			bst <= B_CHK;
		end
	end
	default: bst <= B_IDLE;
	endcase
end

integer i;
always @(posedge clk) begin
	bus_ack <= 0;
	lut_we  <= 0;
	blit_go <= 0;
	if (sys_reset) fb_on <= 0;
	if (reset) begin
		ph <= 0; fb_on <= 0;
		for (i = 0; i < 16; i = i + 1) vtg_r[i] <= 16'h0000;
		for (i = 0; i < 8; i = i + 1) blit_r[i] <= 16'h0000;
	end
	else if (ph) begin
		ph       <= 0;
		bus_ack  <= 1;
		bus_dout <= lut_q;
		if (a_we) begin
			lut_we <= 1; lut_wa <= bus_addr[8:1]; lut_wd <= merge(lut_q, a_din, a_u, a_l);
		end
	end
	else if (bus_stb && bus_cs) begin
		if (is_lut) begin
			ph <= 1; a_we <= bus_we; a_u <= bus_uds; a_l <= bus_lds; a_din <= bus_din;
		end
		else begin
			bus_ack <= 1;
			if (bus_aux)      bus_dout <= (bus_addr[7:1] == 7'h6C) ? {8'h00, CPLD_VERSION} : 16'h0000;
			else if (is_info) bus_dout <= info(bus_addr[4:1]);
			else if (is_vtg)  bus_dout <= vtg_r[bus_addr[4:1]];
			else if (is_blit) bus_dout <= {15'd0, blit_busy};      // the engine's status
			else              bus_dout <= 16'h0000;                // the link
			if (bus_we && is_vtg) begin
				vtg_r[bus_addr[4:1]] <= merge(vtg_r[bus_addr[4:1]], bus_din, bus_uds, bus_lds);
				if (bus_addr[4:1] == 4'd0 && bus_lds) begin
					if (bus_din[0] && !vtg_r[0][0]) fb_start <= {vtg_r[13][5:0], vtg_r[12]};
					if (!sys_reset) fb_on <= bus_din[0];
				end
			end
			if (bus_we && is_blit && !blit_busy) begin
				if (bus_addr[5:4] == 2'b00)
					blit_r[bus_addr[3:1]] <= merge(blit_r[bus_addr[3:1]], bus_din, bus_uds, bus_lds);
				if (bus_addr[5:1] == 5'd8 && bus_lds && bus_din[0]) begin
					blit_go <= 1; go_dir <= bus_din[1]; go_fill <= bus_din[2];
				end
			end
		end
	end
end

endmodule
