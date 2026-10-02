//============================================================================
// falcon_videl - Atari Falcon030 Videl video controller
//
// Registers $FF8200-$FF82C3 (bus_cs) and the Falcon palette $FF9800-$FF9BFF
// (pal_cs).  Runs on clk (CLK_HZ, 32 MHz); the Videl base clock (VCO bit 2:
// 25.175 MHz, else 32 MHz) is a clock enable (ce_base) produced by a
// fractional accumulator.  Video data comes through the 64-bit burst port
// into a double line buffer (two lines of up to 1024 words in M10K).
//
// Behavioural reference: Hatari src/falcon/videl.c
//   VIDEL_ScreenBase_WriteByte   ($8201/$8203 writes clear $820D)
//   VIDEL_ScreenCounter_Read/WriteByte, VIDEL_SyncMode_WriteByte (& 3)
//   VIDEL_LineOffset_ReadWord    ($820E high byte reads bit 0 only)
//   VIDEL_ST_ShiftModeWriteByte  ($8260 write: ST palette, sets $8210/$82C2
//                                 from the monitor type)
//   VIDEL_Falcon_ShiftMode_WriteWord ($8266 write: Falcon palette)
//   VIDEL_getScreenBpp           (bpp from $8266 bits 10/8/4, then $8260)
//   VIDEL_getScreenWidth         (hdb/hde offsets, HDB bit 9 = 2nd half line)
//   VIDEL_getScreenHeight        (VC registers count half lines, VMD bit 0
//                                 line doubling, bit 1 interlace)
//   VIDEL_Get_VFreq              (line = 2*(HHT+2) units)
//   Videl_GetPixelCyclesAndDivider (VMD bits 3:2 cycles/pixel, divider)
//   VIDEL_UpdateColors, Videl_ColorReg_WriteWord (ST palette byte writes are
//                                 copied to both bytes), VIDEL_FalconColorRegsWrite
//                                 (mask $FCFC00FC)
//   ConvGen (conv_gen.c): planar decode, hscroll from $8265 bits 3:0, an
//                                 hscroll adds bpp words to the line stride;
//                                 true colour RRRRRGGGGGGBBBBB, border =
//                                 palette entry 0.
//   Video_GetScreenBaseAddr      (base low byte: & ~3 bitplane, & ~1 TC)
//
// Timing generator (derived; Hatari renders whole frames):
//   * Horizontal unit = D base clocks: D = 16 in ST-shifter mode, else on a
//     VGA monitor 4 (VMD[3:2]=00) or 2, else cycles/pixel.  A half line is
//     HHT+2 units, a line two half lines (VGA: 2*200*2 = 800 clocks, RGB:
//     2*256*4 = 2048 clocks = 64 us, VGA ST-low 2*25*16 = 800).
//   * HBE (first half) ends the blank, HBB (second half) starts it, HSS
//     (second half) starts hsync which lasts to the end of the line.
//   * HDB starts a display line in the first (bit 9 = 0) or second half
//     line (bit 9 = 1, the line is the next one).  Pixels appear off_b base
//     clocks after HDB and end off_e base clocks after HDE (second half):
//       off_b = base + body_b + D, off_e = body_e
//       Falcon bitplanes: body_b = (128/bpp+18)*cyc, body_e = (128/bpp+2)*cyc
//       true colour:      body_b = 16*cyc,           body_e = 0
//       ST shifter mode:  body_b = body_e = (128/bpp+2)*cyc
//       base = VCO bit 8 ? 64 : 128, plus 64 in ST shifter mode.
//     These are Hatari's hdb/hde offsets taken in base clocks, with the VCO
//     bit 8 term of the Falcon documentation ("Authoritative guide to the
//     Falcon video hardware").  With the TOS 4.04 register tables this gives
//     exactly the line width ($8210) in pixels for every TOS mode.
//   * VFC counts half lines 0..VFT (a field is VFT+1 half lines; TOS uses an
//     odd VFT+1 for interlace).  Display lines start at HDB when VDB <= VFC
//     < VDE; vertical blank is VFC < VBE or VFC >= VBB, sampled at the start
//     of each line; vsync while VFC >= VSS.
//   * Monochrome monitor (SM124): no borders (blank = not display) and one
//     base clock per pixel whatever VMD says (TOS leaves VMD = 0 there).
//   * All horizontal registers and the mode are latched at the start of each
//     line, so CPU writes never glitch sync/blank/DE inside a line.  The
//     screen base is latched at the start of the frame (VFC wrap).
//
// Deviations / notes:
//   * VFC ($82A0) and HHC ($8280) read the live counters (Hatari returns a
//     fake incrementing VFC and the written HHC).
//   * VCO bits 1:0 read the monitor type input (Hatari: mirror of $8006).
//   * The video counter $8205/7/9 reads the current display line start plus
//     the words shown so far; writes change the next line fetched.
//   * Colour bank ($8266 bits 3:0) is applied to 4-bitplane Falcon modes.
//   * True colour overlay mode (bit 9) and external sync are not modelled.
//   * HFS/HEE (equalising pulses) are stored and readable but unused.
//   * Fetch is limited to 64 bursts (1024 words) per line.
//   * Sync polarity: VCO bit 6 (hsync) / bit 5 (vsync): 0 = active low
//     (TOS uses 0 for all modes; VGA 640x480 is negative/negative).
//============================================================================

module falcon_videl #(
	parameter integer CLK_HZ = 32000000
) (
	input             clk,
	input             reset,

	// register bus ($FF8200-$FF82C3) and palette ($FF9800-$FF9BFF)
	input             bus_cs,
	input             bus_stb,
	input             pal_cs,
	input             pal_stb,
	input             bus_we,
	input      [10:1] bus_addr,
	input             bus_uds,
	input             bus_lds,
	input      [15:0] bus_din,
	output reg [15:0] bus_dout,
	output reg        bus_ack,

	input       [1:0] monitor_type,    // 00 mono, 01 RGB, 10 VGA, 11 TV

	// video fetch port
	output reg        vid_req,
	output reg [23:3] vid_addr,
	input             vid_ack,
	input      [63:0] vid_data,
	input             vid_valid,

	// video out
	output reg  [7:0] r,
	output reg  [7:0] g,
	output reg  [7:0] b,
	output reg        hsync,
	output reg        vsync,
	output reg        hblank,
	output reg        vblank,
	output reg        ce_pix,
	output reg        de,
	output reg        vbl,
	output reg        hbl,
	output reg        field,
	output reg        underrun
);

localparam [1:0] MON_MONO = 2'b00, MON_VGA = 2'b10;   // 01 RGB, 11 TV

//----------------------------------------------------------------------------
// Registers
//----------------------------------------------------------------------------
reg  [7:0] base_hi, base_mid, base_lo;
reg  [1:0] sync_mode;
reg  [8:0] lof;
reg [15:0] lwd;
reg  [1:0] st_shift;
reg  [7:0] hsc64, hsc65;
reg [15:0] spshift;
reg        st_mode;                 // Hatari videl.bUseSTShifter
reg [15:0] hht, hbb, hbe, hdb, hde, hss, hfs, hee;
reg [15:0] vft, vbb, vbe, vdb, vde, vss;
reg [15:0] vco, vmd;

wire is_vga  = (monitor_type == MON_VGA);
wire is_mono = (monitor_type == MON_MONO);

function [15:0] merge16(input [15:0] o, input [15:0] d, input u, input l);
	merge16 = {u ? d[15:8] : o[15:8], l ? d[7:0] : o[7:0]};
endfunction

//----------------------------------------------------------------------------
// Bus access sequencing: side effects on the strobe, ack two clocks later
//----------------------------------------------------------------------------
wire       any_stb  = (bus_stb & bus_cs) | (pal_stb & pal_cs);
reg  [1:0] acc_ph;
reg        acc_pal, acc_we, acc_u, acc_l;
reg  [7:1] acc_ra;      // register word offset
reg  [9:1] acc_pa;      // palette word offset
reg [15:0] acc_din;

wire reg_wr = bus_stb & bus_cs & bus_we;
wire [7:0] ra = {bus_addr[7:1], 1'b0};

// counter write request (to the fetch sequencer)
reg        vc_wr;
reg  [1:0] vc_wr_byte;
reg  [7:0] vc_wr_data;

//----------------------------------------------------------------------------
// Palettes (M10K).  Each palette has a video copy (simple dual port) and a
// CPU copy (single port, read-modify-write) so CPU reads never steal a video
// read cycle.
//----------------------------------------------------------------------------
// Falcon palette: 18 bits per entry {R6,G6,B6}
(* ramstyle = "M10K, no_rw_check" *) reg [17:0] fpal_vid [0:255];
(* ramstyle = "M10K, no_rw_check" *) reg [17:0] fpal_cpu [0:255];
reg [17:0] fpal_vq, fpal_cq;
reg  [7:0] fpal_vaddr;
reg        fpal_we;
reg  [7:0] fpal_wa;
reg [17:0] fpal_wd;
wire [7:0] fpal_ca = fpal_we ? fpal_wa : bus_addr[9:2];

always @(posedge clk) begin
	if (fpal_we) fpal_vid[fpal_wa] <= fpal_wd;
	fpal_vq <= fpal_vid[fpal_vaddr];
end
always @(posedge clk) begin
	if (fpal_we) fpal_cpu[fpal_ca] <= fpal_wd;
	fpal_cq <= fpal_cpu[fpal_ca];
end

// ST palette: 12 bits per entry (STe 4-4-4 with the STe bit order)
(* ramstyle = "M10K, no_rw_check" *) reg [11:0] spal_vid [0:15];
(* ramstyle = "M10K, no_rw_check" *) reg [11:0] spal_cpu [0:15];
reg [11:0] spal_vq, spal_cq;
reg  [3:0] spal_vaddr;
reg        spal_we;
reg  [3:0] spal_wa;
reg [11:0] spal_wd;
wire [3:0] spal_ca = spal_we ? spal_wa : bus_addr[4:1];

always @(posedge clk) begin
	if (spal_we) spal_vid[spal_wa] <= spal_wd;
	spal_vq <= spal_vid[spal_vaddr];
end
always @(posedge clk) begin
	if (spal_we) spal_cpu[spal_ca] <= spal_wd;
	spal_cq <= spal_cpu[spal_ca];
end

//----------------------------------------------------------------------------
// Live counters (declared here for register reads)
//----------------------------------------------------------------------------
reg [14:0] hpos;            // base clock position in the line
reg [10:0] vfc;             // half line counter
wire [9:0] hhc_rd;
wire [23:0] vc_rd;

//----------------------------------------------------------------------------
// Register read mux
//----------------------------------------------------------------------------
reg [15:0] rd_reg;
always @(*) begin
	case ({acc_ra, 1'b0})
	8'h00: rd_reg = {8'hFF, base_hi};
	8'h02: rd_reg = {8'hFF, base_mid};
	8'h04: rd_reg = {8'hFF, vc_rd[23:16]};
	8'h06: rd_reg = {8'hFF, vc_rd[15:8]};
	8'h08: rd_reg = {8'hFF, vc_rd[7:0]};
	8'h0A: rd_reg = {6'd0, sync_mode, 8'h00};
	8'h0C: rd_reg = {8'h00, base_lo};
	8'h0E: rd_reg = {7'd0, lof};
	8'h10: rd_reg = lwd;
	8'h60: rd_reg = {6'd0, st_shift, 8'h00};
	8'h62: rd_reg = 16'h0000;
	8'h64: rd_reg = {hsc64, hsc65};
	8'h66: rd_reg = spshift;
	8'h80: rd_reg = {6'd0, hhc_rd};
	8'h82: rd_reg = hht;
	8'h84: rd_reg = hbb;
	8'h86: rd_reg = hbe;
	8'h88: rd_reg = hdb;
	8'h8A: rd_reg = hde;
	8'h8C: rd_reg = hss;
	8'h8E: rd_reg = hfs;
	8'h90: rd_reg = hee;
	8'hA0: rd_reg = {5'd0, vfc};
	8'hA2: rd_reg = vft;
	8'hA4: rd_reg = vbb;
	8'hA6: rd_reg = vbe;
	8'hA8: rd_reg = vdb;
	8'hAA: rd_reg = vde;
	8'hAC: rd_reg = vss;
	8'hC0: rd_reg = {vco[15:2], monitor_type};
	8'hC2: rd_reg = vmd;
	default:
		if ({acc_ra, 1'b0} >= 8'h40 && {acc_ra, 1'b0} <= 8'h5E) rd_reg = {4'd0, spal_cq};
		else if ({acc_ra, 1'b0} >= 8'h68 && {acc_ra, 1'b0} <= 8'h7F) rd_reg = 16'h0000;
		else rd_reg = 16'hFFFF;
	endcase
end

//----------------------------------------------------------------------------
// Register writes
//----------------------------------------------------------------------------
always @(posedge clk) begin
	vc_wr   <= 1'b0;
	fpal_we <= 1'b0;
	spal_we <= 1'b0;
	bus_ack <= 1'b0;

	if (reset) begin
		base_hi <= 8'h00; base_mid <= 8'h00; base_lo <= 8'h00;
		sync_mode <= 2'd0;
		lof <= 9'd0;
		st_shift <= 2'd0;
		hsc64 <= 8'd0; hsc65 <= 8'd0;
		spshift <= 16'd0;
		hfs <= 16'd0; hee <= 16'd0;
		acc_ph <= 2'd0;
		// Power-up timing: the TOS 4.04 mode for the attached monitor, so
		// the output is a sane signal before TOS programs the Videl.
		case (monitor_type)
		MON_VGA: begin      // VGA 640x480 16 colours
			st_mode <= 1'b0; lwd <= 16'h00A0;
			hht <= 16'h00C6; hbb <= 16'h008D; hbe <= 16'h0015;
			hdb <= 16'h02A3; hde <= 16'h007C; hss <= 16'h0096;
			vft <= 16'h0419; vbb <= 16'h03FF; vbe <= 16'h003F;
			vdb <= 16'h003F; vde <= 16'h03FF; vss <= 16'h0415;
			vco <= 16'h0186; vmd <= 16'h0008;
		end
		MON_MONO: begin     // SM124 640x400
			st_mode <= 1'b1; st_shift <= 2'd2; lwd <= 16'h0028;
			hht <= 16'h001A; hbb <= 16'h0000; hbe <= 16'h0000;
			hdb <= 16'h020F; hde <= 16'h000C; hss <= 16'h0014;
			vft <= 16'h03E9; vbb <= 16'h0000; vbe <= 16'h0000;
			vdb <= 16'h0043; vde <= 16'h0363; vss <= 16'h03E7;
			vco <= 16'h0080; vmd <= 16'h0000;
		end
		default: begin      // RGB/TV 640x200 16 colours, 50 Hz
			st_mode <= 1'b0; lwd <= 16'h00A0;
			hht <= 16'h01FE; hbb <= 16'h0199; hbe <= 16'h0050;
			hdb <= 16'h004D; hde <= 16'h00FE; hss <= 16'h01B2;
			vft <= 16'h0271; vbb <= 16'h0265; vbe <= 16'h002F;
			vdb <= 16'h007F; vde <= 16'h020F; vss <= 16'h026B;
			vco <= 16'h0181; vmd <= 16'h0004;
		end
		endcase
	end else begin
		// ---- access sequencing ----
		if (any_stb && acc_ph == 2'd0) begin
			acc_ph  <= 2'd1;
			acc_pal <= pal_cs;
			acc_we  <= bus_we;
			acc_u   <= bus_uds;
			acc_l   <= bus_lds;
			acc_ra  <= bus_addr[7:1];
			acc_pa  <= bus_addr[9:1];
			acc_din <= bus_din;
		end else if (acc_ph == 2'd1) begin
			// palette RAM outputs now hold the addressed entry
			acc_ph <= 2'd0;
			bus_ack <= 1'b1;
			if (acc_pal) begin
				bus_dout <= acc_pa[1] ? {8'h00, fpal_cq[5:0], 2'b00}
				                      : {fpal_cq[17:12], 2'b00, fpal_cq[11:6], 2'b00};
				if (acc_we) begin
					fpal_we <= 1'b1;
					fpal_wa <= acc_pa[9:2];
					if (acc_pa[1])
						fpal_wd <= {fpal_cq[17:6], acc_l ? acc_din[7:2] : fpal_cq[5:0]};
					else
						fpal_wd <= {acc_u ? acc_din[15:10] : fpal_cq[17:12],
						            acc_l ? acc_din[7:2]   : fpal_cq[11:6],
						            fpal_cq[5:0]};
				end
			end else begin
				bus_dout <= rd_reg;
			end
		end

		// ---- register side effects on the strobe ----
		if (reg_wr) begin
			case (ra)
			8'h00: if (bus_lds) begin base_hi <= bus_din[7:0]; base_lo <= 8'h00; end
			8'h02: if (bus_lds) begin base_mid <= bus_din[7:0]; base_lo <= 8'h00; end
			8'h04: if (bus_lds) begin vc_wr <= 1'b1; vc_wr_byte <= 2'd2; vc_wr_data <= bus_din[7:0]; end
			8'h06: if (bus_lds) begin vc_wr <= 1'b1; vc_wr_byte <= 2'd1; vc_wr_data <= bus_din[7:0]; end
			8'h08: if (bus_lds) begin vc_wr <= 1'b1; vc_wr_byte <= 2'd0; vc_wr_data <= bus_din[7:0]; end
			8'h0A: if (bus_uds) sync_mode <= bus_din[9:8];
			8'h0C: if (bus_lds) base_lo <= bus_din[7:0];
			8'h0E: begin
				if (bus_uds) lof[8]   <= bus_din[8];
				if (bus_lds) lof[7:0] <= bus_din[7:0];
			end
			8'h10: lwd <= merge16(lwd, bus_din, bus_uds, bus_lds);
			8'h60: if (bus_uds) begin
				// VIDEL_ST_ShiftModeWriteByte
				st_shift <= bus_din[9:8];
				st_mode  <= 1'b1;
				case (bus_din[9:8])
				2'd0: begin lwd <= 16'h0050; vmd <= is_vga ? 16'h0005 : 16'h0000; end
				2'd1: begin lwd <= 16'h0050; vmd <= is_vga ? 16'h0009 : 16'h0004; end
				2'd2: begin lwd <= 16'h0028; vmd <= is_mono ? 16'h0000 : is_vga ? 16'h0008 : 16'h0006; end
				default: begin lwd <= 16'h0050; vmd <= 16'h0000; end
				endcase
			end
			8'h64: begin
				if (bus_uds) hsc64 <= bus_din[15:8];
				if (bus_lds) hsc65 <= bus_din[7:0];
			end
			8'h66: begin spshift <= merge16(spshift, bus_din, bus_uds, bus_lds); st_mode <= 1'b0; end
			8'h82: hht <= merge16(hht, bus_din, bus_uds, bus_lds);
			8'h84: hbb <= merge16(hbb, bus_din, bus_uds, bus_lds);
			8'h86: hbe <= merge16(hbe, bus_din, bus_uds, bus_lds);
			8'h88: hdb <= merge16(hdb, bus_din, bus_uds, bus_lds);
			8'h8A: hde <= merge16(hde, bus_din, bus_uds, bus_lds);
			8'h8C: hss <= merge16(hss, bus_din, bus_uds, bus_lds);
			8'h8E: hfs <= merge16(hfs, bus_din, bus_uds, bus_lds);
			8'h90: hee <= merge16(hee, bus_din, bus_uds, bus_lds);
			8'hA2: vft <= merge16(vft, bus_din, bus_uds, bus_lds);
			8'hA4: vbb <= merge16(vbb, bus_din, bus_uds, bus_lds);
			8'hA6: vbe <= merge16(vbe, bus_din, bus_uds, bus_lds);
			8'hA8: vdb <= merge16(vdb, bus_din, bus_uds, bus_lds);
			8'hAA: vde <= merge16(vde, bus_din, bus_uds, bus_lds);
			8'hAC: vss <= merge16(vss, bus_din, bus_uds, bus_lds);
			8'hC0: vco <= merge16(vco, bus_din, bus_uds, bus_lds);
			8'hC2: vmd <= merge16(vmd, bus_din, bus_uds, bus_lds);
			default: ;
			endcase
			// ST palette: a byte write is copied to both bytes (Hatari
			// Videl_ColorReg_WriteWord), then masked to 12 bits.
			if (ra >= 8'h40 && ra <= 8'h5E) begin
				spal_we <= 1'b1;
				spal_wa <= bus_addr[4:1];
				if (bus_uds && bus_lds) spal_wd <= bus_din[11:0];
				else if (bus_uds)      spal_wd <= {bus_din[11:8], bus_din[15:8]};
				else                   spal_wd <= {bus_din[3:0], bus_din[7:0]};
			end
		end
	end
end

//----------------------------------------------------------------------------
// Base clock enable: 25.175 MHz or 32 MHz from CLK_HZ
//----------------------------------------------------------------------------
localparam [31:0] F25 = 32'd25175000;
localparam [31:0] F32 = 32'd32000000;
localparam [31:0] FCLK = CLK_HZ;

reg        lt_clk25;                // line-latched clock select
reg [31:0] facc;
reg        ce_base;
wire [32:0] facc_sum = {1'b0, facc} + {1'b0, (lt_clk25 ? F25 : F32)};

always @(posedge clk) begin
	if (reset) begin
		facc    <= 32'd0;
		ce_base <= 1'b0;
	end else if (facc_sum >= {1'b0, FCLK}) begin
		facc    <= facc_sum[31:0] - FCLK;
		ce_base <= 1'b1;
	end else begin
		facc    <= facc_sum[31:0];
		ce_base <= 1'b0;
	end
end

//----------------------------------------------------------------------------
// Mode decode (live) and per-line latch
//----------------------------------------------------------------------------
// bpl: log2 bits per pixel (0..3 = 1/2/4/8 bitplanes, 4 = true colour)
reg [2:0] m_bpl;
always @(*) begin
	if (spshift[10])      m_bpl = 3'd0;
	else if (spshift[8])  m_bpl = 3'd4;
	else if (spshift[4])  m_bpl = 3'd3;
	else if (!st_mode)    m_bpl = 3'd2;
	else if (st_shift == 2'd0) m_bpl = 3'd2;
	else if (st_shift == 2'd1) m_bpl = 3'd1;
	else                  m_bpl = 3'd0;
end

// cycles per pixel (log2) from VMD bits 3:2; one on a mono monitor
wire [1:0] m_cyc = is_mono ? 2'd0 :
                   (vmd[3:2] == 2'b00) ? 2'd2 :
                   (vmd[3:2] == 2'b01) ? 2'd1 : 2'd0;
// horizontal unit (log2 base clocks)
wire [2:0] m_sd = st_mode ? 3'd4 :
                  is_vga  ? ((vmd[3:2] == 2'b00) ? 3'd2 : 3'd1) :
                            {1'b0, m_cyc};

reg  [2:0] lt_bpl;
reg  [1:0] lt_cyc;
reg  [2:0] lt_sd;
reg        lt_st, lt_mono, lt_vco8;
reg  [8:0] lt_hht, lt_hbb, lt_hbe, lt_hss, lt_hde;
reg  [9:0] lt_hdb;
reg  [3:0] lt_bank;

// positions in base clocks within the line
wire [9:0]  lt_h   = {1'b0, lt_hht} + 10'd2;
wire [14:0] pos_hh = {5'd0, lt_h} << lt_sd;          // half line length
wire [14:0] pos_l  = {pos_hh[13:0], 1'b0};          // line length
wire [14:0] pos_hbe = {6'd0, lt_hbe} << lt_sd;
wire [14:0] pos_hbb = pos_hh + ({6'd0, lt_hbb} << lt_sd);
wire [14:0] pos_hss = pos_hh + ({6'd0, lt_hss} << lt_sd);
wire [14:0] pos_hdb = (lt_hdb[9] ? pos_hh : 15'd0) + ({6'd0, lt_hdb[8:0]} << lt_sd);
wire [14:0] pos_hde = pos_hh + ({6'd0, lt_hde} << lt_sd);

// display offsets (base clocks)
wire [8:0]  q128   = 9'd128 >> lt_bpl;               // 128/bpp for bitplanes
wire [9:0]  body_b0 = lt_st ? {1'b0, q128} + 10'd2 :
                      (lt_bpl == 3'd4) ? 10'd16 : {1'b0, q128} + 10'd18;
wire [9:0]  body_e0 = (lt_bpl == 3'd4 && !lt_st) ? 10'd0 : {1'b0, q128} + 10'd2;
wire [11:0] body_b = {2'd0, body_b0} << lt_cyc;
wire [11:0] body_e = {2'd0, body_e0} << lt_cyc;
wire [11:0] off_base = lt_st ? (lt_vco8 ? 12'd128 : 12'd192) : (lt_vco8 ? 12'd64 : 12'd128);
wire [11:0] off_b = off_base + body_b + (12'd1 << lt_sd);
wire [11:0] off_e = body_e;
wire [15:0] deon_raw  = {1'b0, pos_hdb} + {4'd0, off_b};
wire [15:0] deoff_raw = {1'b0, pos_hde} + {4'd0, off_e};
wire [14:0] pos_deon  = (deon_raw  >= {1'b0, pos_l}) ? deon_raw[14:0]  - pos_l : deon_raw[14:0];
wire [14:0] pos_deoff = (deoff_raw >= {1'b0, pos_l}) ? deoff_raw[14:0] - pos_l : deoff_raw[14:0];

wire [14:0] hhc_full = ((hpos >= pos_hh) ? (hpos - pos_hh) : hpos) >> lt_sd;
assign hhc_rd = hhc_full[9:0];

//----------------------------------------------------------------------------
// Timing generator
//----------------------------------------------------------------------------
reg  [1:0] pix_ph;
reg        hsync_s, hblank_s, vblank_s, vsync_s, de_s, hwin_s;
reg        line_pend;
reg        frame_start;             // one clock pulse at the VFC wrap
reg        hdb_go;                  // HDB of a display line (pulse)
reg        go_pending;
reg        line_go;                 // shifter starts the line (previous DE over)
reg  [3:0] lt_hscroll;
reg        lt_vsync_pol, lt_hsync_pol;

wire [10:0] vft_m = vft[10:0], vbb_m = vbb[10:0], vbe_m = vbe[10:0];
wire [10:0] vdb_m = vdb[10:0], vde_m = vde[10:0], vss_m = vss[10:0];

wire at_end  = (hpos >= pos_l - 15'd1);
wire at_half = (hpos == pos_hh - 15'd1);
wire [10:0] vfc_inc = (vfc >= vft_m) ? 11'd0 : vfc + 11'd1;
wire vcond = (vfc >= vdb_m) && (vfc < vde_m);
wire vcond_next = (vfc_inc >= vdb_m) && (vfc_inc < vde_m);

wire ce_pix_t = ce_base && (pix_ph == 2'd0);
wire [1:0] cyc_mask = (2'd1 << lt_cyc) - 2'd1;

always @(posedge clk) begin
	frame_start <= 1'b0;
	hdb_go      <= 1'b0;
	vbl         <= 1'b0;
	hbl         <= 1'b0;
	if (reset) begin
		hpos <= 15'd0; vfc <= 11'd0; pix_ph <= 2'd0;
		hsync_s <= 1'b0; hblank_s <= 1'b1; vblank_s <= 1'b1; vsync_s <= 1'b0;
		de_s <= 1'b0; hwin_s <= 1'b0; line_pend <= 1'b0;
		field <= 1'b0;
		lt_clk25 <= 1'b0;
		lt_bpl <= 3'd2; lt_cyc <= 2'd0; lt_sd <= 3'd1; lt_st <= 1'b0; lt_mono <= 1'b0; lt_vco8 <= 1'b1;
		lt_hht <= 9'd198; lt_hbb <= 9'd0; lt_hbe <= 9'd0; lt_hss <= 9'd0; lt_hde <= 9'd0; lt_hdb <= 10'd0;
		lt_bank <= 4'd0; lt_hscroll <= 4'd0; lt_hsync_pol <= 1'b0; lt_vsync_pol <= 1'b0;
	end else if (ce_base) begin
		// pixel phase (reset at the line start)
		pix_ph <= at_end ? 2'd0 : ((pix_ph & cyc_mask) == cyc_mask) ? 2'd0 : pix_ph + 2'd1;

		if (at_end) begin
			hpos <= 15'd0;
			// latch the line parameters
			lt_clk25 <= vco[2];
			lt_bpl   <= m_bpl;
			lt_cyc   <= m_cyc;
			lt_sd    <= m_sd;
			lt_st    <= st_mode;
			lt_mono  <= is_mono;
			lt_vco8  <= vco[8];
			lt_hht   <= hht[8:0];
			lt_hbb   <= hbb[8:0];
			lt_hbe   <= hbe[8:0];
			lt_hss   <= hss[8:0];
			lt_hde   <= hde[8:0];
			lt_hdb   <= hdb[9:0];
			lt_bank  <= spshift[3:0];
			lt_hscroll <= hsc65[3:0];
			lt_hsync_pol <= vco[6];
			lt_vsync_pol <= vco[5];
		end else begin
			hpos <= hpos + 15'd1;
		end

		// half line boundaries: vertical counter
		if (at_end || at_half) begin
			vfc <= vfc_inc;
			vsync_s <= (vfc_inc >= vss_m);
			if (vfc_inc == 11'd0) begin
				frame_start <= 1'b1;
				field <= ~field;
			end
		end
		if (at_end) begin
			// vertical blank sampled at the line start
			if (is_mono) begin
				vblank_s <= !vcond_next;
				if (vblank_s == 1'b0 && !vcond_next) vbl <= 1'b1;
			end else begin
				vblank_s <= (vfc_inc < vbe_m) || (vfc_inc >= vbb_m);
				if (vblank_s == 1'b0 && ((vfc_inc < vbe_m) || (vfc_inc >= vbb_m))) vbl <= 1'b1;
			end
		end

		// horizontal events (positions are those of the current tick)
		if (hpos == 15'd0) hsync_s <= 1'b0;        // hsync ends with the line
		if (hpos == pos_hss) begin hsync_s <= 1'b1; hbl <= 1'b1; end
		if (hpos == pos_hbe) hblank_s <= 1'b0;
		if (hpos == pos_hbb) hblank_s <= 1'b1;
		if (hpos == pos_hdb) begin
			line_pend <= vcond;
			if (vcond) hdb_go <= 1'b1;
		end
		if (hpos == pos_deon) begin
			hwin_s <= 1'b1;
			if (line_pend || (hpos == pos_hdb && vcond)) de_s <= 1'b1;
			line_pend <= 1'b0;
		end
		if (hpos == pos_deoff) begin
			hwin_s <= 1'b0;
			de_s <= 1'b0;
		end
	end
end

// The HDB of the next line can come while the previous line is still being
// displayed (HDB in the second half line, e.g. the VGA ST modes): the
// shifter (and the fetch of the line after, which reuses the buffer) start
// the new line only once the previous DE is over.
always @(posedge clk) begin
	line_go <= 1'b0;
	if (reset) go_pending <= 1'b0;
	else if (hdb_go) go_pending <= 1'b1;
	else if (go_pending && !de_s && !line_go) begin
		line_go <= 1'b1;
		go_pending <= 1'b0;
	end
end

wire hblank_eff = lt_mono ? ~hwin_s : hblank_s;
wire de_vis     = de_s & ~hblank_eff & ~vblank_s;

//----------------------------------------------------------------------------
// Fetch: double line buffer (512 x 64), lines fetched one ahead
//----------------------------------------------------------------------------
(* ramstyle = "M10K, no_rw_check" *) reg [63:0] lbuf [0:511];
reg  [8:0] lb_raddr;
reg [63:0] lb_q;
reg        lb_we;
reg  [8:0] lb_waddr;
reg [63:0] lb_wdata;
always @(posedge clk) begin
	if (lb_we) lbuf[lb_waddr] <= lb_wdata;
	lb_q <= lbuf[lb_raddr];
end

reg  [23:1] f_addr;        // next line word address
reg         f_rep;         // line doubling: second copy pending
reg  [10:0] f_idx;         // next line index to fetch in this frame
reg  [10:0] go_cnt;        // display lines started in this frame
reg         f_restart;
reg         f_busy;
reg  [23:5] fb_addr;
reg   [6:0] fb_left;
reg  [23:1] lb_addr0, lb_addr1;   // line start address held in each buffer
reg   [1:0] buf_ready;
reg  [23:0] base_l;

// rx queue (lines in flight): two entries {buf, qword count}
reg         rxq_buf0, rxq_buf1;
reg   [8:0] rxq_n0, rxq_n1;
reg   [1:0] rxq_cnt;
reg   [7:0] rx_q;

wire        hs_on    = (hsc65[3:0] != 4'd0);
wire  [4:0] bplw     = (m_bpl == 3'd4) ? 5'd16 : (5'd1 << m_bpl);   // words per 16 pixels
wire [10:0] nwords   = {1'b0, lwd[9:0]} + (hs_on ? {6'd0, bplw} : 11'd0);
wire [11:0] stride   = {2'd0, lwd[9:0]} + {3'd0, lof} + (hs_on ? {7'd0, bplw} : 12'd0);
wire        dbl      = vmd[0];
wire        ilace    = vmd[1];
wire [12:0] nb_raw   = ({9'd0, f_addr[4:1]} + {2'd0, nwords} + 13'd15) >> 4;
wire  [6:0] nb       = (nb_raw > 13'd64) ? 7'd64 : nb_raw[6:0];
wire        f_allow  = (f_idx <= 11'd1) || (go_cnt >= f_idx);
wire [23:1] f_next   = f_addr + (ilace ? {10'd0, stride, 1'b0} : {11'd0, stride});

always @(posedge clk) begin
	lb_we <= 1'b0;
	if (reset) begin
		vid_req <= 1'b0;
		f_busy <= 1'b0;
		f_restart <= 1'b1;
		f_idx <= 11'd0;
		f_addr <= 23'd0;
		f_rep <= 1'b0;
		rxq_cnt <= 2'd0;
		rx_q <= 8'd0;
		buf_ready <= 2'b00;
		base_l <= 24'd0;
		lb_addr0 <= 23'd0; lb_addr1 <= 23'd0;
	end else begin
		if (frame_start) begin
			f_restart <= 1'b1;
			base_l <= {base_hi, base_mid, base_lo & (spshift[8] ? 8'hFE : 8'hFC)};
		end

		// ---- receive side ----
		if (vid_valid && rxq_cnt != 2'd0) begin
			lb_we    <= 1'b1;
			lb_waddr <= {rxq_buf0, rx_q};
			lb_wdata <= vid_data;
			if ({1'b0, rx_q} == rxq_n0 - 9'd1) begin
				rx_q <= 8'd0;
				buf_ready[rxq_buf0] <= 1'b1;
			end else begin
				rx_q <= rx_q + 8'd1;
			end
		end
		begin : rxq_update
			reg pop, push;
			pop  = vid_valid && rxq_cnt != 2'd0 && ({1'b0, rx_q} == rxq_n0 - 9'd1);
			push = !f_busy && !f_restart && !frame_start && f_allow && (rxq_cnt != 2'd2 || pop) && nb != 7'd0;
			// queue shift / push
			if (pop) begin
				rxq_buf0 <= rxq_buf1; rxq_n0 <= rxq_n1;
			end
			if (push) begin
				if ((pop ? rxq_cnt - 2'd1 : rxq_cnt) == 2'd0) begin
					rxq_buf0 <= f_idx[0]; rxq_n0 <= {nb, 2'b00};
				end else begin
					rxq_buf1 <= f_idx[0]; rxq_n1 <= {nb, 2'b00};
				end
			end
			rxq_cnt <= rxq_cnt + (push ? 2'd1 : 2'd0) - (pop ? 2'd1 : 2'd0);

			// ---- request side ----
			if (!f_busy) begin
				if (f_restart && !frame_start) begin
					f_restart <= 1'b0;
					f_idx <= 11'd0;
					f_rep <= 1'b0;
					f_addr <= base_l[23:1] + ((ilace && field) ? {11'd0, stride} : 23'd0);
				end else if (!f_restart && !frame_start && f_allow && (rxq_cnt != 2'd2 || pop)) begin
					if (f_idx[0]) lb_addr1 <= f_addr; else lb_addr0 <= f_addr;
					buf_ready[f_idx[0]] <= (nb == 7'd0);
					if (nb != 7'd0) begin
						f_busy  <= 1'b1;
						vid_req <= 1'b1;
						vid_addr <= {f_addr[23:5], 2'b00};
						fb_addr <= f_addr[23:5] + 19'd1;
						fb_left <= nb;
					end else begin
						f_idx <= f_idx + 11'd1;
						if (dbl && !f_rep) f_rep <= 1'b1;
						else begin f_rep <= 1'b0; f_addr <= f_next; end
					end
				end
			end else if (vid_ack) begin
				if (fb_left == 7'd1) begin
					vid_req <= 1'b0;
					f_busy <= 1'b0;
					f_idx <= f_idx + 11'd1;
					if (dbl && !f_rep) f_rep <= 1'b1;
					else begin f_rep <= 1'b0; f_addr <= f_next; end
				end else begin
					vid_addr <= {fb_addr, 2'b00};
					fb_addr <= fb_addr + 19'd1;
				end
				fb_left <= fb_left - 7'd1;
			end
		end

		// video counter writes change the next line fetched
		if (vc_wr) begin
			case (vc_wr_byte)
			2'd2: f_addr[23:16] <= vc_wr_data;
			2'd1: f_addr[15:8]  <= vc_wr_data;
			default: f_addr[7:1] <= vc_wr_data[7:1];
			endcase
		end
	end
end

//----------------------------------------------------------------------------
// Shifter: word reader, plane loader, pixel decode
//----------------------------------------------------------------------------
reg        ln_buf;
reg  [2:0] ln_bpl;
reg        ln_st;
reg  [9:0] rp;                 // reader word pointer within the line buffer
reg        rd_pend, rd_pend2;  // read issued (address registered) / data in lb_q
reg  [1:0] rd_sel, rd_sel2;
reg [15:0] wf [0:3];
reg  [2:0] wf_cnt;
reg [15:0] cur [0:7];
reg [15:0] nxt [0:7];
reg  [3:0] nxt_cnt;
reg        cur_valid;
reg  [3:0] bp;
reg [10:0] consumed;
reg        active_rd;

wire [3:0] nplanes = (ln_bpl == 3'd4) ? 4'd1 : (4'd1 << ln_bpl);
wire       ln_tc   = (ln_bpl == 3'd4);
wire [15:0] wf_head = wf[0];
wire [15:0] rd_word = (rd_sel2 == 2'd0) ? lb_q[63:48] : (rd_sel2 == 2'd1) ? lb_q[47:32] :
                      (rd_sel2 == 2'd2) ? lb_q[31:16] : lb_q[15:0];
wire       px_take = ce_pix_t && de_s;
wire [10:0] go_idx = frame_start ? 11'd0 : go_cnt;

// pixel index from the current group
wire [3:0] bit_i = 4'd15 - bp;
reg  [7:0] pix_idx;
always @(*) begin : pidx
	integer k;
	for (k = 0; k < 8; k = k + 1)
		pix_idx[k] = (k < nplanes) ? cur[k][bit_i] : 1'b0;
end

wire       ld_pop  = !ln_tc && (nxt_cnt < nplanes) && (wf_cnt != 3'd0);
wire       tc_pop  = ln_tc && px_take && (wf_cnt != 3'd0);
wire       wf_pop  = ld_pop || tc_pop;
wire       wf_push = rd_pend2;
wire [2:0] wf_cnt_after_pop = wf_cnt - (wf_pop ? 3'd1 : 3'd0);
wire       rd_issue = active_rd && ((wf_cnt + (rd_pend ? 3'd1 : 3'd0) + (rd_pend2 ? 3'd1 : 3'd0)) < 3'd4);

always @(posedge clk) begin
	if (reset) begin
		rd_pend <= 1'b0; rd_pend2 <= 1'b0; rd_sel2 <= 2'd0; wf_cnt <= 3'd0; nxt_cnt <= 4'd0; cur_valid <= 1'b0;
		bp <= 4'd0; rp <= 10'd0; ln_buf <= 1'b0; ln_bpl <= 3'd2; ln_st <= 1'b0;
		consumed <= 11'd0; active_rd <= 1'b0; go_cnt <= 11'd0; underrun <= 1'b0;
		lb_raddr <= 9'd0; rd_sel <= 2'd0;
	end else begin
		underrun <= 1'b0;
		if (frame_start) go_cnt <= 11'd0;
		if (line_go) begin
			// start of a display line: reset the reader to the line start
			ln_buf  <= go_idx[0];
			ln_bpl  <= lt_bpl;
			ln_st   <= lt_st;
			rp      <= {6'd0, (go_idx[0] ? lb_addr1[4:1] : lb_addr0[4:1])};
			rd_pend <= 1'b0;
			rd_pend2 <= 1'b0;
			wf_cnt  <= 3'd0;
			nxt_cnt <= 4'd0;
			cur_valid <= 1'b0;
			bp      <= (lt_bpl == 3'd4) ? 4'd0 : lt_hscroll;
			consumed <= 11'd0;
			active_rd <= 1'b1;
			go_cnt  <= go_idx + 11'd1;
			if (!buf_ready[go_idx[0]] || go_idx >= f_idx) underrun <= 1'b1;
		end else begin
			// RAM read
			rd_pend <= rd_issue;
			rd_pend2 <= rd_pend;
			rd_sel2 <= rd_sel;
			if (rd_issue) begin
				lb_raddr <= {ln_buf, rp[9:2]};
				rd_sel   <= rp[1:0];
				rp       <= rp + 10'd1;
			end
			// word fifo
			begin : wfifo
				integer k;
				reg [15:0] nw [0:3];
				for (k = 0; k < 4; k = k + 1) nw[k] = wf[k];
				if (wf_pop) begin
					nw[0] = wf[1]; nw[1] = wf[2]; nw[2] = wf[3];
				end
				if (wf_push) nw[wf_cnt_after_pop[1:0]] = rd_word;
				for (k = 0; k < 4; k = k + 1) wf[k] <= nw[k];
				wf_cnt <= wf_cnt_after_pop + (wf_push ? 3'd1 : 3'd0);
			end
			// plane loader
			if (ld_pop) begin
				nxt[nxt_cnt[2:0]] <= wf_head;
				nxt_cnt <= nxt_cnt + 4'd1;
			end
			if (!ln_tc && !cur_valid && nxt_cnt == nplanes) begin
				begin : xfer0
					integer k;
					for (k = 0; k < 8; k = k + 1) cur[k] <= nxt[k];
				end
				cur_valid <= 1'b1;
				nxt_cnt <= 4'd0;
			end
			// pixel consume
			if (px_take) begin
				if (ln_tc) begin
					cur[0] <= wf_head;
					consumed <= consumed + 11'd1;
				end else if (bp == 4'd15) begin
					begin : xfer1
						integer k;
						for (k = 0; k < 8; k = k + 1) cur[k] <= nxt[k];
					end
					if (nxt_cnt != nplanes) underrun <= 1'b1;
					nxt_cnt <= 4'd0;
					bp <= 4'd0;
					consumed <= consumed + {7'd0, nplanes};
				end else begin
					bp <= bp + 4'd1;
				end
			end
		end
	end
end

// video counter readback
wire [23:1] cur_line_addr = ln_buf ? lb_addr1 : lb_addr0;
wire [23:1] nxt_line_addr = (f_idx == go_cnt) ? f_addr : (go_cnt[0] ? lb_addr1 : lb_addr0);
assign vc_rd = de_s ? {cur_line_addr + {12'd0, consumed}, 1'b0} : {nxt_line_addr, 1'b0};

//----------------------------------------------------------------------------
// Output pipeline: palette lookup and colour expansion
//----------------------------------------------------------------------------
// true colour pixel: in TC mode the word is shown as it is popped
wire [15:0] tc_word = wf_head;
wire        use_spal = lt_st && (lt_bpl != 3'd4);
wire  [7:0] pal_idx  = !de_s ? 8'd0 :
                       (ln_bpl == 3'd2 && !ln_st) ? {lt_bank, pix_idx[3:0]} : pix_idx;

reg        s1_tc, s1_de, s1_vis, s1_hs, s1_vs, s1_ce, s1_spal, s1_hb, s1_vb;
reg [15:0] s1_tcw;

always @(posedge clk) begin
	// RAM address registered on the pixel clock enable, data next clock;
	// the value is held for the whole pixel period.
	if (ce_pix_t) begin
		fpal_vaddr <= pal_idx;
		spal_vaddr <= pal_idx[3:0];
	end
end

// stage 1 (aligned with the palette RAM address register)
reg        s0_tc, s0_de, s0_vis, s0_hs, s0_vs, s0_ce, s0_spal, s0_hb, s0_vb;
reg [15:0] s0_tcw;
always @(posedge clk) begin
	s0_ce   <= ce_pix_t;
	s1_ce   <= s0_ce;
	if (ce_pix_t) begin
	s0_tc   <= ln_tc && de_s;
	s0_tcw  <= tc_word;
	s0_de   <= de_vis;
	s0_vis  <= ~(hblank_eff | vblank_s);
	s0_hb   <= hblank_eff;
	s0_vb   <= vblank_s;
	s0_hs   <= hsync_s ^ ~lt_hsync_pol;
	s0_vs   <= vsync_s ^ ~lt_vsync_pol;
	s0_spal <= use_spal;
	end

	s1_tc <= s0_tc; s1_tcw <= s0_tcw; s1_de <= s0_de; s1_vis <= s0_vis;
	s1_hb <= s0_hb; s1_vb <= s0_vb; s1_hs <= s0_hs; s1_vs <= s0_vs;
	s1_spal <= s0_spal;
end

function [7:0] ste8(input [3:0] c);
	ste8 = {c[2:0], c[3], c[2:0], c[3]};
endfunction

always @(posedge clk) begin
	if (reset) begin
		r <= 8'd0; g <= 8'd0; b <= 8'd0;
		hsync <= 1'b1; vsync <= 1'b1; hblank <= 1'b1; vblank <= 1'b1; ce_pix <= 1'b0; de <= 1'b0;
	end else begin
		hsync  <= s1_hs;
		vsync  <= s1_vs;
		hblank <= s1_hb;
		vblank <= s1_vb;
		ce_pix <= s1_ce;
		de     <= s1_de;
		if (!s1_vis) begin
			r <= 8'd0; g <= 8'd0; b <= 8'd0;
		end else if (s1_tc && s1_de) begin
			r <= {s1_tcw[15:11], s1_tcw[15:13]};
			g <= {s1_tcw[10:5],  s1_tcw[10:9]};
			b <= {s1_tcw[4:0],   s1_tcw[4:2]};
		end else if (s1_spal) begin
			r <= ste8(spal_vq[11:8]);
			g <= ste8(spal_vq[7:4]);
			b <= ste8(spal_vq[3:0]);
		end else begin
			r <= {fpal_vq[17:12], fpal_vq[17:16]};
			g <= {fpal_vq[11:6],  fpal_vq[11:10]};
			b <= {fpal_vq[5:0],   fpal_vq[5:4]};
		end
	end
end

endmodule
