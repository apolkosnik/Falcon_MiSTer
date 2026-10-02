// falcon_ide.sv - Atari Falcon030 internal IDE interface ($F00000-$F0003F)
//
// One master and one slave ATA disk, each backed by a MiSTer hps_io disk
// image slot.  PIO only (the Falcon has no IDE DMA).
//
// Behavioural reference: Hatari src/ide.c (QEMU derived):
//   fcha2io()            register layout on the Falcon bus
//   Ide_Mem_bget/wget/bput/wput   access sizes (registers are byte wide on the
//                        odd byte, the data register is word wide at $F00000
//                        and $F00002, every other access reads $FF/$FFFF and
//                        writes are ignored; nothing in $F00000-$F0003F bus
//                        errors, so this module has no bus_berr output)
//   ide_ioport_write/read, ide_status_read, ide_ctrl_write, ide_data_readw/
//   writew, ide_identify, ide_sector_read/write, ide_get_sector,
//   ide_set_signature, ide_reset, ide_init_one (default geometry)
//   Ide_Init + HDC_PartitionCount (automatic byte swap detection)
//
// Byte order of the data register (Hatari "byteswap", automatic mode): a
// disk image whose sector 0 holds $AA,$55 at $1FE (or one of the byte-swapped
// AHDI partition ids at $1C6/$1C8/$1C9) is a raw dump of a Falcon disk and is
// presented unswapped: the image byte 2n appears on the CPU's odd byte
// (D7..D0) of the n-th word.  Any other image is byte swapped so that image
// byte 2n appears on the even byte (D15..D8).  The detection runs when the
// image is mounted.  IDENTIFY data words are always presented as their
// numeric value (word 0 = $0040 reads as $0040).
//
// Disk image port: one engine serves both units.  sd_rd[u], sd_wr[u],
// sd_ack[u] and img_mounted[u] are per unit (bit 0 = master, bit 1 = slave);
// sd_lba0/sd_lba1 and sd_buff_din0/sd_buff_din1 (the per slot hps_io
// inputs) carry the same value.  The shared sd_buff_* inputs are only
// accepted while one of our sd_ack bits is set.  At most one sd_rd/sd_wr bit
// is set at a time.  led: an image transfer is in progress.
//
// irq: ATA INTRQ, active high (the system ORs it onto MFP GPIP5 low).
// It follows the selected device's interrupt pending flag and is disabled by
// nIEN; reading the status register (not the alternate status) or writing a
// command clears the pending flag.
//
// Deviations from Hatari (hardware documentation - ATA-3/ATA-4 - wins, or
// resource limits):
// - BSY is really reported while the image is accessed (Hatari is instant);
//   while BSY is set the command block registers read as the status
//   register and writes to them are ignored.
// - Writing a command clears INTRQ; nIEN masks the INTRQ output instead of
//   suppressing the request.
// - After a multi sector command the address registers hold the last sector
//   transferred (ATA), Hatari leaves the following sector.  On an error they
//   hold the failing sector.
// - Sectors beyond the end of the image, or invalid CHS values, give IDNF
//   (error $10); Hatari aborts (ABRT).  Writing a read only image aborts as
//   in Hatari.
// - EXECUTE DEVICE DIAGNOSTIC leaves DRDY|DSC set and runs on both devices
//   (Hatari: status 0 and only the selected device).
// - A soft reset selects device 0 and loads device/head = $A0 (ATA); Hatari
//   keeps the DEV bit.
// - The task file (sector count, sector number, cylinder, device/head) is
//   one register set shared by both devices; Hatari keeps a copy per device
//   that only differs after a command updated the selected device's copy.
//   Status, error and the interrupt pending flag are per device.
// - INITIALIZE DEVICE PARAMETERS really changes the CHS translation and
//   IDENTIFY words 54-58 (Hatari acknowledges it and ignores it).  The
//   default geometry is Hatari's "no hint" geometry (16 heads, 63 sectors,
//   cylinders = sectors / 1008 clamped to 2..16383); Hatari's guess from an
//   MS-DOS partition table is not done.
// - Images are limited to 2^28 - 1 sectors (LBA28, 128 GB).
// - Commands to an absent device are ignored (Hatari runs them on a missing
//   master).  LBA48 commands, DMA commands, DEVICE RESET and PACKET commands
//   are aborted (LBA48 is not advertised in IDENTIFY).  SET FEATURES 03h
//   updates IDENTIFY words 63/88 (Hatari's code writes them at a byte offset).
// - The data register reads $FFFF when no transfer is in progress.
// - IDENTIFY model string is "MiSTer  IDE disk <n>M" (Hatari: "Hatari  IDE
//   disk <n>M"), serial "QM0000<unit+1>", firmware "1.0".
//
// Sector buffer: 16 sectors (MAX_MULT_SECTORS, the READ/WRITE MULTIPLE
// block limit) = 8 KB in two 4096 x 8 byte lanes (M10K).  The constant
// IDENTIFY words are a 256 x 16 ROM (M10K).

module falcon_ide #(
	parameter CLK_HZ = 32000000
) (
	input             clk,
	input             reset,

	// register bus ($F00000-$F0003F, bus_addr = A5..A1)
	input             bus_cs,
	input             bus_stb,
	input             bus_we,
	input       [5:1] bus_addr,
	input             bus_uds,
	input             bus_lds,
	input      [15:0] bus_din,
	output reg [15:0] bus_dout,
	output reg        bus_ack,

	output            irq,

	// hps_io disk image interface (two slots, one engine)
	input       [1:0] img_mounted,
	input             img_readonly,
	input      [63:0] img_size,
	output     [31:0] sd_lba0,
	output     [31:0] sd_lba1,
	output reg  [1:0] sd_rd,
	output reg  [1:0] sd_wr,
	input       [1:0] sd_ack,
	input       [8:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din0,
	output      [7:0] sd_buff_din1,
	input             sd_buff_wr,

	output            led
);

// ---------------------------------------------------------------------------
// ATA constants
localparam [7:0] ST_ERR  = 8'h01;
localparam [7:0] ST_DRQ  = 8'h08;
localparam [7:0] ST_DSC  = 8'h10;
localparam [7:0] ST_DRDY = 8'h40;
localparam [7:0] ST_BSY  = 8'h80;

localparam [7:0] ER_ABRT = 8'h04;
localparam [7:0] ER_IDNF = 8'h10;

localparam [4:0] MAX_MULT = 5'd16;

// ---------------------------------------------------------------------------
// Per unit state
reg  [1:0] present;
reg  [1:0] ro;
reg  [1:0] swap;            // 1 = image byte 2n on D15..D8
reg [27:0] nbs   [0:1];     // number of 512 byte sectors (LBA28)
reg [15:0] ccyl  [0:1];     // current cylinders
reg [13:0] dcyl  [0:1];     // power-on default cylinders (computed at mount)
reg  [4:0] chead [0:1];     // current heads (1..16)
reg  [7:0] cspt  [0:1];     // current sectors per track (1..255)
reg  [4:0] mult  [0:1];     // multiple sector setting (0 = disabled)
reg  [4:0] xmode [0:1];     // SET FEATURES 03h: {kind, mode}, kind 0 = power on,
                            // 1 = PIO, 2 = multiword DMA, 3 = Ultra DMA
reg  [7:0] r_status [0:1];
reg  [7:0] r_error  [0:1];
reg  [1:0] pend;            // interrupt pending per device

// task file (shared, written to both devices)
reg  [7:0] r_nsec, r_sect, r_lcyl, r_hcyl, r_select, r_feature;
reg        cur;             // selected device (DEV bit)
reg  [7:0] devctl;          // device control register
wire       nien = devctl[1];
wire       srst = devctl[2];

reg        h_act;           // image transfer in progress
assign irq = pend[cur] & ~nien;
assign led = h_act;

reg [31:0] sd_lba;
assign sd_lba0 = sd_lba;
assign sd_lba1 = sd_lba;

// ---------------------------------------------------------------------------
// Sector buffer: two byte lanes, 4096 x 8 each, simple dual port, registered
// read.  Byte address b (0..8191) = sector*512 + offset; lane = b[0].
reg  [7:0] lane_e [0:4095];
reg  [7:0] lane_o [0:4095];
reg        le_we, lo_we;
reg [11:0] l_waddr;
reg  [7:0] le_wdata, lo_wdata;
wire [11:0] l_raddr;
reg  [7:0] le_q, lo_q;

always @(posedge clk) begin
	if (le_we) lane_e[l_waddr] <= le_wdata;
	if (lo_we) lane_o[l_waddr] <= lo_wdata;
	le_q <= lane_e[l_raddr];
	lo_q <= lane_o[l_raddr];
end

// ---------------------------------------------------------------------------
// Identify data (Hatari ide_identify).  A 256 x 20 ROM holds the constant
// words and, in bits 19:16, the source of the words that depend on the unit
// (0 = the constant).
localparam [3:0] IS_CONST = 4'd0, IS_DCYL = 4'd1, IS_UNIT = 4'd2, IS_M35 = 4'd3, IS_M36 = 4'd4,
                 IS_M37 = 4'd5, IS_M38 = 4'd6, IS_CCYL = 4'd7, IS_CHEAD = 4'd8, IS_CSPT = 4'd9,
                 IS_CAPLO = 4'd10, IS_CAPHI = 4'd11, IS_MULT = 4'd12, IS_LBALO = 4'd13,
                 IS_LBAHI = 4'd14, IS_XMODE = 4'd15;

function [3:0] ident_src(input [7:0] w);
	begin
		case (w)
			8'd1:  ident_src = IS_DCYL;
			8'd13: ident_src = IS_UNIT;
			8'd35: ident_src = IS_M35;
			8'd36: ident_src = IS_M36;
			8'd37: ident_src = IS_M37;
			8'd38: ident_src = IS_M38;
			8'd54: ident_src = IS_CCYL;
			8'd55: ident_src = IS_CHEAD;
			8'd56: ident_src = IS_CSPT;
			8'd57: ident_src = IS_CAPLO;
			8'd58: ident_src = IS_CAPHI;
			8'd59: ident_src = IS_MULT;
			8'd60: ident_src = IS_LBALO;
			8'd61: ident_src = IS_LBAHI;
			8'd63, 8'd88: ident_src = IS_XMODE;
			default: ident_src = IS_CONST;
		endcase
	end
endfunction

function [15:0] ident_rom(input [7:0] w);
	begin
		case (w)
			8'd0:   ident_rom = 16'h0040;
			8'd3:   ident_rom = 16'd16;
			8'd4:   ident_rom = 16'd512 * 16'd63;
			8'd5:   ident_rom = 16'd512;
			8'd6:   ident_rom = 16'd63;
			8'd10:  ident_rom = "QM";            // serial "QM0000<unit+1>"
			8'd11:  ident_rom = "00";
			8'd12:  ident_rom = "00";
			8'd14, 8'd15, 8'd16, 8'd17, 8'd18, 8'd19: ident_rom = "  ";
			8'd20:  ident_rom = 16'd3;
			8'd21:  ident_rom = 16'd512;
			8'd22:  ident_rom = 16'd4;
			8'd23:  ident_rom = "1.";            // firmware "1.0"
			8'd24:  ident_rom = "0 ";
			8'd25, 8'd26: ident_rom = "  ";
			8'd27:  ident_rom = "Mi";            // model "MiSTer  IDE disk <n>M"
			8'd28:  ident_rom = "ST";
			8'd29:  ident_rom = "er";
			8'd30:  ident_rom = "  ";
			8'd31:  ident_rom = "ID";
			8'd32:  ident_rom = "E ";
			8'd33:  ident_rom = "di";
			8'd34:  ident_rom = "sk";
			8'd39, 8'd40, 8'd41, 8'd42, 8'd43, 8'd44, 8'd45, 8'd46: ident_rom = "  ";
			8'd47:  ident_rom = 16'h8000 | {11'd0, MAX_MULT};
			8'd48:  ident_rom = 16'd1;
			8'd49:  ident_rom = 16'h0B00;
			8'd51:  ident_rom = 16'h0200;
			8'd52:  ident_rom = 16'h0200;
			8'd53:  ident_rom = 16'h0007;
			8'd65, 8'd66, 8'd67, 8'd68: ident_rom = 16'd120;
			8'd80:  ident_rom = 16'h00F0;
			8'd81:  ident_rom = 16'h0016;
			8'd82:  ident_rom = 16'h4000;
			8'd83:  ident_rom = 16'h5000;
			8'd84:  ident_rom = 16'h4000;
			8'd85:  ident_rom = 16'h4000;
			8'd86:  ident_rom = 16'h5000;
			8'd87:  ident_rom = 16'h4000;
			8'd93:  ident_rom = 16'h6001;
			8'd106: ident_rom = 16'h1000;
			8'd117: ident_rom = 16'd256;
			default: ident_rom = 16'h0000;
		endcase
	end
endfunction

reg [19:0] idrom [0:255];
integer k;
initial begin
	for (k = 0; k < 256; k = k + 1) idrom[k] = {ident_src(k[7:0]), ident_rom(k[7:0])};
end

reg  [12:0] ptr;            // word pointer in the DRQ block
reg  [19:0] id_rom_q;
always @(posedge clk) id_rom_q <= idrom[ptr[7:0]];

// IDENTIFY scratch values, computed when the command runs
reg  [13:0] id_dcyl;        // default cylinders
reg  [23:0] id_bcd;         // capacity in MB, digits left aligned
reg   [2:0] id_nd;          // number of digits - 1

// model characters 17..23: digits, "M", spaces
function [7:0] mchar(input [2:0] k, input [23:0] bcd, input [2:0] nd);
	reg [3:0] dg;
	begin
		case (k)
			3'd0: dg = bcd[23:20];
			3'd1: dg = bcd[19:16];
			3'd2: dg = bcd[15:12];
			3'd3: dg = bcd[11:8];
			3'd4: dg = bcd[7:4];
			default: dg = bcd[3:0];
		endcase
		if (k <= nd) mchar = 8'h30 + {4'd0, dg};
		else if ({1'b0, k} == {1'b0, nd} + 4'd1) mchar = "M";
		else mchar = " ";
	end
endfunction

reg         eu;             // unit being served by the engine
wire [27:0] i_cap  = nbs[eu];
wire [31:0] i_curcap = ccyl[eu] * chead[eu] * cspt[eu];
wire  [4:0] i_xm   = xmode[eu];
wire [15:0] i_w63  = (i_xm[4:3] == 2'd2) ? (16'h0007 | (16'h0100 << i_xm[2:0])) : 16'h0007;
wire [15:0] i_w88  = (i_xm[4:3] == 2'd0) ? 16'h203F :
                     (i_xm[4:3] == 2'd3) ? (16'h003F | (16'h0100 << i_xm[2:0])) : 16'h003F;

reg [15:0] ident_word;
always @* begin
	case (id_rom_q[19:16])
		IS_DCYL:  ident_word = {2'b00, id_dcyl};
		IS_UNIT:  ident_word = {eu ? "2" : "1", " "};
		IS_M35:   ident_word = {" ", mchar(3'd0, id_bcd, id_nd)};
		IS_M36:   ident_word = {mchar(3'd1, id_bcd, id_nd), mchar(3'd2, id_bcd, id_nd)};
		IS_M37:   ident_word = {mchar(3'd3, id_bcd, id_nd), mchar(3'd4, id_bcd, id_nd)};
		IS_M38:   ident_word = {mchar(3'd5, id_bcd, id_nd), mchar(3'd6, id_bcd, id_nd)};
		IS_CCYL:  ident_word = ccyl[eu];
		IS_CHEAD: ident_word = {11'd0, chead[eu]};
		IS_CSPT:  ident_word = {8'd0, cspt[eu]};
		IS_CAPLO: ident_word = i_curcap[15:0];
		IS_CAPHI: ident_word = i_curcap[31:16];
		IS_MULT:  ident_word = (mult[eu] != 5'd0) ? (16'h0100 | {11'd0, mult[eu]}) : 16'h0000;
		IS_LBALO: ident_word = i_cap[15:0];
		IS_LBAHI: ident_word = {4'd0, i_cap[27:16]};
		IS_XMODE: ident_word = ptr[0] ? i_w63 : i_w88;     // word 63 / word 88
		default:  ident_word = id_rom_q[15:0];
	endcase
end

// ---------------------------------------------------------------------------
// Engine
localparam [5:0]
	E_IDLE    = 6'd0,
	E_MNT     = 6'd1,
	E_MNT_W   = 6'd2,
	E_MNT_DIV = 6'd3,
	E_MNT_DW  = 6'd4,
	E_CMD     = 6'd6,
	E_RBLK    = 6'd7,
	E_RSEC    = 6'd8,
	E_RA1     = 6'd9,
	E_RA2     = 6'd10,
	E_RA3     = 6'd11,
	E_RWAIT   = 6'd12,
	E_RDRQ    = 6'd13,
	E_WBLK    = 6'd14,
	E_WDRQ    = 6'd15,
	E_WSEC    = 6'd16,
	E_WA1     = 6'd17,
	E_WA2     = 6'd18,
	E_WA3     = 6'd19,
	E_WWAIT   = 6'd20,
	E_VSEC    = 6'd21,
	E_VA1     = 6'd22,
	E_VA2     = 6'd23,
	E_VA3     = 6'd24,
	E_SKA1    = 6'd25,
	E_SKA2    = 6'd26,
	E_SKA3    = 6'd27,
	E_INIT_D  = 6'd28,
	E_INIT_W  = 6'd29,
	E_IDDRQ   = 6'd30,
	E_ID_DIV  = 6'd31,
	E_NMAX    = 6'd32,
	E_ID_DW   = 6'd33,
	E_ID_BCD  = 6'd34;

reg  [5:0] est;
reg  [7:0] ecmd;
reg  [8:0] remain;          // sectors left in the command
reg  [4:0] blk;             // sectors per DRQ block
reg  [4:0] bcnt;            // sector index in the current block
reg  [4:0] bsz;             // sectors in the current block
reg        first;
reg [12:0] pend_words;      // words in the DRQ block
reg        drq_ident;       // DRQ block is IDENTIFY data
reg        drq_dir_wr;      // DRQ block is written by the host
reg        drq_act;
reg        cmd_req;         // command written, not yet taken by the engine
reg  [7:0] cmd_val;
reg        cmd_unit;

// LBA computation
reg [20:0] chs_t1;
reg [27:0] lba;
reg        addr_bad;

// mount
reg  [1:0] mnt_pend;
reg  [1:0] mnt_ro;
reg  [7:0] mb_1fe, mb_1ff, mb_1c6, mb_1c8, mb_1c9;
reg        mnt_scan;        // capture sector 0 bytes

// srst
reg        srst_req;        // SRST asserted, engine must stop
reg        srst_rel;        // SRST released, apply signature when idle

// hps transfer
reg        h_ack_seen;
reg        h_unit;
reg        h_wr;
reg  [3:0] h_sec;           // buffer sector
reg        h_done;          // one clock pulse

wire       mnt_busy = (est == E_MNT_W || est == E_MNT_DIV || est == E_MNT_DW);
wire [1:0] remount  = mnt_pend | img_mounted | (mnt_busy ? (eu ? 2'b10 : 2'b01) : 2'b00);

initial begin
	present = 2'b00; ro = 2'b00; swap = 2'b11;
	nbs[0] = 28'd0; nbs[1] = 28'd0; dcyl[0] = 14'd2; dcyl[1] = 14'd2;
	mnt_pend = 2'b00; mnt_ro = 2'b00;
	h_act = 1'b0; h_ack_seen = 1'b0; h_unit = 1'b0; h_wr = 1'b0; h_sec = 4'd0;
	sd_rd = 2'b00; sd_wr = 2'b00; sd_lba = 32'd0;
end

// divider (sequential, restoring): 28 bit / 12 bit
reg        dv_busy;
reg [27:0] dv_num;
reg [11:0] dv_den;
reg [27:0] dv_q;
reg [12:0] dv_r;
reg  [4:0] dv_cnt;
reg        dv_start;

always @(posedge clk) begin
	if (reset) begin
		dv_busy <= 1'b0;
		dv_cnt <= 5'd0;
	end else if (dv_start) begin
		dv_busy <= 1'b1;
		dv_cnt <= 5'd28;
		dv_q <= dv_num;
		dv_r <= 13'd0;
	end else if (dv_busy) begin
		if (dv_cnt == 5'd0) dv_busy <= 1'b0;
		else begin
			if ({dv_r[11:0], dv_q[27]} >= {1'b0, dv_den}) begin
				dv_r <= {dv_r[11:0], dv_q[27]} - {1'b0, dv_den};
				dv_q <= {dv_q[26:0], 1'b1};
			end else begin
				dv_r <= {dv_r[11:0], dv_q[27]};
				dv_q <= {dv_q[26:0], 1'b0};
			end
			dv_cnt <= dv_cnt - 5'd1;
		end
	end
end

// BCD conversion (double dabble, 17 bit input, 6 digits)
reg        bcd_busy;
reg [16:0] bcd_bin;
reg  [4:0] bcd_cnt;

function [23:0] dabble(input [23:0] v);
	integer j;
	begin
		dabble = v;
		for (j = 0; j < 6; j = j + 1)
			if (dabble[j*4 +: 4] >= 4'd5) dabble[j*4 +: 4] = dabble[j*4 +: 4] + 4'd3;
	end
endfunction

// ---------------------------------------------------------------------------
// Address helpers (task file)
wire        e_lba_mode = r_select[6];
wire [27:0] e_lba28 = {r_select[3:0], r_hcyl, r_lcyl, r_sect};
wire [15:0] e_cyl = {r_hcyl, r_lcyl};
wire  [3:0] e_head = r_select[3:0];

// ---------------------------------------------------------------------------
// Bus decode
wire        a_data = (bus_addr[5:2] == 4'd0) & bus_uds & bus_lds;
wire        a_breg = bus_lds & ~bus_uds;
wire  [2:0] a_reg  = bus_addr[4:2];          // 1..7 for F00005..F0001D
wire        a_io   = a_breg & ~bus_addr[5] & ~bus_addr[1] & (a_reg != 3'd0);
wire        a_alt  = a_breg & (bus_addr[5:1] == 5'd28);  // F00039

wire        both_absent = (present == 2'b00);
wire        cur_absent  = ~present[cur];
wire  [7:0] cur_status  = (both_absent | cur_absent) ? 8'h00 : r_status[cur];
wire        cur_bsy     = cur_status[7];

reg  [7:0] reg_rd;
always @* begin
	case (a_reg)
		3'd1: reg_rd = r_error[cur];
		3'd2: reg_rd = r_nsec;
		3'd3: reg_rd = r_sect;
		3'd4: reg_rd = r_lcyl;
		3'd5: reg_rd = r_hcyl;
		3'd6: reg_rd = r_select;
		default: reg_rd = cur_status;
	endcase
	if (both_absent) reg_rd = 8'h00;
	else if (cur_bsy) reg_rd = cur_status;
end

wire        data_valid = drq_act & (eu == cur);
wire [15:0] buf_word = swap[eu] ? {le_q, lo_q} : {lo_q, le_q};

// bus access pipeline: stb -> (1 clock for RAM/ROM) -> ack
reg        acc_pend;
reg        acc_we;
reg        acc_data;
reg [15:0] acc_din;

// read port address: hps byte address while sending a sector, else the
// data register word pointer (the RAM output is valid one clock later)
assign l_raddr = (h_act & h_wr) ? {h_sec, sd_buff_addr[8:1]} : ptr[11:0];

// sd_buff_din: lane select by the current hps byte address
assign sd_buff_din0 = sd_buff_addr[0] ? lo_q : le_q;
assign sd_buff_din1 = sd_buff_din0;

// ---------------------------------------------------------------------------
// Main sequential logic
integer i;

// command classification
wire c_read   = (ecmd == 8'h20) | (ecmd == 8'h21) | (ecmd == 8'hC4);
wire c_write  = (ecmd == 8'h30) | (ecmd == 8'h31) | (ecmd == 8'h38) | (ecmd == 8'h3C) |
                (ecmd == 8'hC5) | (ecmd == 8'hCD);
wire c_multi  = (ecmd == 8'hC4) | (ecmd == 8'hC5) | (ecmd == 8'hCD);

// helper: increment the address registers
task automatic incr_addr;
	begin
		if (e_lba_mode) begin
			{r_select[3:0], r_hcyl, r_lcyl, r_sect} <= e_lba28 + 28'd1;
		end else begin
			if (r_sect >= cspt[eu]) begin
				r_sect <= 8'd1;
				if ({1'b0, e_head} + 5'd1 >= chead[eu]) begin
					r_select[3:0] <= 4'd0;
					{r_hcyl, r_lcyl} <= e_cyl + 16'd1;
				end else
					r_select[3:0] <= e_head + 4'd1;
			end else
				r_sect <= r_sect + 8'd1;
		end
	end
endtask

// ide_set_signature
task automatic signature;
	begin
		r_nsec <= 8'd1;
		r_sect <= 8'd1;
		r_lcyl <= 8'h00;
		r_hcyl <= 8'h00;
	end
endtask

task automatic fin(input [7:0] st, input [7:0] er, input doirq);
	begin
		r_status[eu] <= st;
		r_error[eu] <= er;
		if (doirq) pend[eu] <= 1'b1;
		drq_act <= 1'b0;
		est <= E_IDLE;
	end
endtask

always @(posedge clk) begin
	bus_ack <= 1'b0;
	le_we <= 1'b0;
	lo_we <= 1'b0;
	dv_start <= 1'b0;
	h_done <= 1'b0;

	// The media state (present, ro, swap, nbs, dcyl), mount requests and an
	// image transfer in flight survive `reset` (machine reset or the 68030
	// RESET instruction): a drive keeps its disk over a bus reset and hps_io
	// finishes the block it is serving.

	// ---------------- mount latch ----------------
	for (i = 0; i < 2; i = i + 1)
		if (img_mounted[i]) begin
			mnt_pend[i] <= 1'b1;
			mnt_ro[i] <= img_readonly;
			nbs[i] <= (img_size[63:37] != 27'd0) ? 28'h0FFFFFFF : img_size[36:9];
		end

	// ---------------- hps transfer ----------------
	if (h_act) begin
		if (sd_ack[h_unit]) begin
			h_ack_seen <= 1'b1;
			sd_rd <= 2'b00;
			sd_wr <= 2'b00;
		end else if (h_ack_seen) begin
			h_act <= 1'b0;
			h_ack_seen <= 1'b0;
			h_done <= 1'b1;
		end
	end
	// incoming sector data
	if (h_act & ~h_wr & sd_buff_wr & sd_ack[h_unit]) begin
		l_waddr <= {h_sec, sd_buff_addr[8:1]};
		if (sd_buff_addr[0]) begin lo_we <= 1'b1; lo_wdata <= sd_buff_dout; end
		else begin le_we <= 1'b1; le_wdata <= sd_buff_dout; end
		if (mnt_scan) begin
			if (sd_buff_addr == 9'h1FE) mb_1fe <= sd_buff_dout;
			if (sd_buff_addr == 9'h1FF) mb_1ff <= sd_buff_dout;
			if (sd_buff_addr == 9'h1C6) mb_1c6 <= sd_buff_dout;
			if (sd_buff_addr == 9'h1C8) mb_1c8 <= sd_buff_dout;
			if (sd_buff_addr == 9'h1C9) mb_1c9 <= sd_buff_dout;
		end
	end


	if (reset) begin
		// ATA hardware reset: power-on defaults, diagnostics passed
		for (i = 0; i < 2; i = i + 1) begin
			ccyl[i] <= {2'b00, dcyl[i]};
			chead[i] <= 5'd16;
			cspt[i] <= 8'd63;
			mult[i] <= MAX_MULT;
			xmode[i] <= 5'd0;
			// a unit whose mount scan is pending stays busy until it is done
			r_status[i] <= remount[i] ? (present[i] ? ST_BSY : 8'h00) :
			               present[i] ? (ST_DRDY | ST_DSC) : 8'h00;
			r_error[i] <= 8'h01;
		end
		// an interrupted mount scan is done again
		if (mnt_busy) mnt_pend[eu] <= 1'b1;
		r_nsec <= 8'd1;
		r_sect <= 8'd1;
		r_lcyl <= (present != 2'b00) ? 8'h00 : 8'hFF;
		r_hcyl <= (present != 2'b00) ? 8'h00 : 8'hFF;
		r_select <= 8'hA0;
		r_feature <= 8'h00;
		pend <= 2'b00;
		cur <= 1'b0;
		devctl <= 8'h00;
		est <= E_IDLE;
		eu <= 1'b0;
		ecmd <= 8'h00;
		drq_act <= 1'b0;
		drq_ident <= 1'b0;
		drq_dir_wr <= 1'b0;
		ptr <= 13'd0;
		pend_words <= 13'd0;
		mnt_scan <= 1'b0;
		cmd_req <= 1'b0;
		cmd_val <= 8'h00;
		cmd_unit <= 1'b0;
		srst_req <= 1'b0;
		srst_rel <= 1'b0;
		acc_pend <= 1'b0;
		bus_dout <= 16'hFFFF;
		bcd_busy <= 1'b0;
		id_dcyl <= 14'd2;
		id_bcd <= 24'd0;
		id_nd <= 3'd0;
	end else begin

		// ---------------- BCD ----------------
		if (bcd_busy) begin
			if (bcd_cnt == 5'd0) bcd_busy <= 1'b0;
			else begin
				{id_bcd, bcd_bin} <= {dabble(id_bcd), bcd_bin} << 1;
				bcd_cnt <= bcd_cnt - 5'd1;
			end
		end

		// ---------------- bus access ----------------
		if (bus_stb) begin
			acc_pend <= 1'b1;
			acc_we <= bus_we;
			acc_din <= bus_din;
			acc_data <= a_data;
			if (bus_we) begin
				// register writes take effect at stb
				if (a_alt) begin
					// device control (common to both devices)
					if (~devctl[2] & bus_din[2]) begin
						srst_req <= 1'b1;
						srst_rel <= 1'b0;
						for (i = 0; i < 2; i = i + 1) begin
							r_status[i] <= present[i] ? (ST_BSY | ST_DSC) : 8'h00;
							r_error[i] <= 8'h01;
						end
						pend <= 2'b00;
					end else if (devctl[2] & ~bus_din[2]) begin
						srst_rel <= 1'b1;
					end
					devctl <= bus_din[7:0];
				end else if (a_io & ~cur_bsy & ~srst) begin
					case (a_reg)
						3'd1: r_feature <= bus_din[7:0];
						3'd2: r_nsec <= bus_din[7:0];
						3'd3: r_sect <= bus_din[7:0];
						3'd4: r_lcyl <= bus_din[7:0];
						3'd5: r_hcyl <= bus_din[7:0];
						3'd6: begin
							r_select <= bus_din[7:0] | 8'hA0;
							cur <= bus_din[4];
						end
						3'd7: begin
							// command: ignored by an absent device
							if (present[cur] && !cmd_req) begin
								cmd_req <= 1'b1;
								cmd_val <= bus_din[7:0];
								cmd_unit <= cur;
								pend[cur] <= 1'b0;
								r_status[cur] <= ST_BSY | (r_status[cur] & ST_DSC);
							end
						end
						default: ;
					endcase
				end
			end else begin
				// register reads with side effects at stb
				if (a_io && a_reg == 3'd7) pend[cur] <= 1'b0;
			end
		end

		if (acc_pend) begin
			acc_pend <= 1'b0;
			bus_ack <= 1'b1;
			if (!acc_we) begin
				if (acc_data) begin
					if (data_valid && !drq_dir_wr)
						bus_dout <= drq_ident ? ident_word : buf_word;
					else
						bus_dout <= 16'hFFFF;
				end else if (a_io) bus_dout <= {8'hFF, reg_rd};
				else if (a_alt) bus_dout <= {8'hFF, cur_status};
				else bus_dout <= 16'hFFFF;
			end
			// data register transfer
			if (acc_data && data_valid && (acc_we == drq_dir_wr)) begin
				if (acc_we) begin
					l_waddr <= ptr[11:0];
					le_we <= 1'b1;
					lo_we <= 1'b1;
					le_wdata <= swap[eu] ? acc_din[15:8] : acc_din[7:0];
					lo_wdata <= swap[eu] ? acc_din[7:0] : acc_din[15:8];
				end
				ptr <= ptr + 13'd1;
			end
		end

		// ---------------- engine ----------------
		if (srst_req & ~h_act) begin
			// abort everything while SRST is asserted
			if (est == E_MNT_W || est == E_MNT_DIV || est == E_MNT_DW)
				mnt_pend[eu] <= 1'b1;   // redo the interrupted mount scan
			mnt_scan <= 1'b0;
			est <= E_IDLE;
			drq_act <= 1'b0;
			cmd_req <= 1'b0;
			if (srst_rel) begin
				srst_req <= 1'b0;
				srst_rel <= 1'b0;
				for (i = 0; i < 2; i = i + 1)
					r_status[i] <= present[i] ? (ST_DRDY | ST_DSC) : 8'h00;
				signature();
				r_select <= 8'hA0;
				cur <= 1'b0;
			end
		end else if (!srst_req) begin
		if (cmd_req && ((est == E_IDLE && !h_act) || est == E_RDRQ || est == E_WDRQ || est == E_IDDRQ)) begin
			// a new command ends a data transfer in progress
			cmd_req <= 1'b0;
			eu <= cmd_unit;
			ecmd <= cmd_val;
			drq_act <= 1'b0;
			drq_ident <= 1'b0;
			r_status[cmd_unit] <= ST_BSY | (r_status[cmd_unit] & ST_DSC);
			pend[cmd_unit] <= 1'b0;
			est <= E_CMD;
		end else
		case (est)
		E_IDLE: begin
			if ((mnt_pend[0] | mnt_pend[1]) && !h_act) begin
				eu <= mnt_pend[0] ? 1'b0 : 1'b1;
				est <= E_MNT;
			end
		end

		// ---------------- mount ----------------
		E_MNT: begin
			mnt_pend[eu] <= 1'b0;
			ro[eu] <= mnt_ro[eu];
			mult[eu] <= MAX_MULT;
			xmode[eu] <= 5'd0;
			pend[eu] <= 1'b0;
			if (nbs[eu] == 28'd0) begin
				present[eu] <= 1'b0;
				r_status[eu] <= 8'h00;
				est <= E_IDLE;
			end else begin
				present[eu] <= 1'b1;
				r_status[eu] <= ST_BSY;
				mb_1fe <= 8'h00; mb_1ff <= 8'h00; mb_1c6 <= 8'h00; mb_1c8 <= 8'h00; mb_1c9 <= 8'h00;
				mnt_scan <= 1'b1;
				h_act <= 1'b1; h_unit <= eu; h_wr <= 1'b0; h_sec <= 4'd0;
				sd_lba <= 32'd0;
				sd_rd[eu] <= 1'b1;
				est <= E_MNT_W;
			end
		end
		E_MNT_W: if (h_done) begin
			mnt_scan <= 1'b0;
			// Hatari HDC_PartitionCount: image is already byte swapped?
			swap[eu] <= !((mb_1fe == 8'hAA && mb_1ff == 8'h55) ||
			              (mb_1c6 == "G" && mb_1c8 == "M" && mb_1c9 == "E") ||
			              (mb_1c6 == "B" && mb_1c8 == "M" && mb_1c9 == "G") ||
			              (mb_1c6 == "X" && mb_1c8 == "M" && mb_1c9 == "G") ||
			              (mb_1c6 == "L" && mb_1c8 == "X" && mb_1c9 == "N") ||
			              (mb_1c6 == "R" && mb_1c8 == "W" && mb_1c9 == "A") ||
			              (mb_1c6 == "F" && mb_1c8 == "2" && mb_1c9 == "3") ||
			              (mb_1c6 == "U" && mb_1c8 == "X" && mb_1c9 == "N") ||
			              (mb_1c6 == "M" && mb_1c8 == "X" && mb_1c9 == "I") ||
			              (mb_1c6 == "S" && mb_1c8 == "P" && mb_1c9 == "W"));
			dv_num <= nbs[eu];
			dv_den <= 12'd1008;
			dv_start <= 1'b1;
			est <= E_MNT_DIV;
		end
		E_MNT_DIV: est <= E_MNT_DW;
		E_MNT_DW: if (!dv_busy) begin
			// Hatari ide_init_one default geometry (current = default)
			if (dv_q > 28'd16383) begin ccyl[eu] <= 16'd16383; dcyl[eu] <= 14'd16383; end
			else if (dv_q < 28'd2) begin ccyl[eu] <= 16'd2; dcyl[eu] <= 14'd2; end
			else begin ccyl[eu] <= dv_q[15:0]; dcyl[eu] <= dv_q[13:0]; end
			chead[eu] <= 5'd16;
			cspt[eu] <= 8'd63;
			r_status[eu] <= ST_DRDY | ST_DSC;
			r_error[eu] <= 8'h01;
			signature();
			r_select <= r_select & 8'hF0;
			est <= E_IDLE;
		end

		// ---------------- command decode ----------------
		E_CMD: begin
			remain <= (r_nsec == 8'd0) ? 9'd256 : {1'b0, r_nsec};
			first <= 1'b1;
			drq_ident <= 1'b0;
			if (c_read) begin
				if (c_multi && mult[eu] == 5'd0) fin(ST_DRDY | ST_ERR, ER_ABRT, 1'b1);
				else begin
					blk <= c_multi ? mult[eu] : 5'd1;
					r_error[eu] <= 8'h00;
					est <= E_RBLK;
				end
			end else if (c_write) begin
				if (c_multi && mult[eu] == 5'd0) fin(ST_DRDY | ST_ERR, ER_ABRT, 1'b1);
				else begin
					blk <= c_multi ? mult[eu] : 5'd1;
					r_error[eu] <= 8'h00;
					est <= E_WBLK;
				end
			end else begin
				case (ecmd)
					8'hEC: begin   // IDENTIFY DEVICE: default cylinders, then the size digits
						dv_num <= nbs[eu];
						dv_den <= 12'd1008;
						dv_start <= 1'b1;
						est <= E_ID_DIV;
					end
					8'h91: begin   // INITIALIZE DEVICE PARAMETERS
						if (r_nsec == 8'd0) fin(ST_DRDY | ST_ERR, ER_ABRT, 1'b1);
						else begin
							chead[eu] <= {1'b0, r_select[3:0]} + 5'd1;
							cspt[eu] <= r_nsec;
							dv_num <= (nbs[eu] > 28'd16514064) ? 28'd16514064 : nbs[eu];
							dv_den <= ({8'd0, r_select[3:0]} + 12'd1) * {4'd0, r_nsec};
							dv_start <= 1'b1;
							est <= E_INIT_D;
						end
					end
					8'h10, 8'h11, 8'h12, 8'h13, 8'h14, 8'h15, 8'h16, 8'h17,
					8'h18, 8'h19, 8'h1A, 8'h1B, 8'h1C, 8'h1D, 8'h1E, 8'h1F:
						fin(ST_DRDY | ST_DSC, 8'h00, 1'b1);           // RECALIBRATE
					8'h70, 8'h71, 8'h72, 8'h73, 8'h74, 8'h75, 8'h76, 8'h77,
					8'h78, 8'h79, 8'h7A, 8'h7B, 8'h7C, 8'h7D, 8'h7E, 8'h7F:
						est <= E_SKA1;                                // SEEK
					8'hC6: begin   // SET MULTIPLE MODE
						if (r_nsec != 8'd0 &&
						    (r_nsec > {3'd0, MAX_MULT} || (r_nsec & (r_nsec - 8'd1)) != 8'd0))
							fin(ST_DRDY | ST_ERR, ER_ABRT, 1'b1);
						else begin
							mult[eu] <= r_nsec[4:0];
							fin(ST_DRDY, r_error[eu], 1'b1);
						end
					end
					8'h40, 8'h41: begin   // READ VERIFY SECTORS
						r_error[eu] <= 8'h00;
						est <= E_VSEC;
					end
					8'hE5, 8'h98: begin   // CHECK POWER MODE
						r_nsec <= 8'hFF;
						fin(ST_DRDY, r_error[eu], 1'b1);
					end
					8'hE7: fin(ST_DRDY, r_error[eu], 1'b1);       // FLUSH CACHE
					8'hE0, 8'hE1, 8'hE2, 8'hE3, 8'hE6, 8'h94, 8'h95, 8'h96, 8'h97, 8'h99:
						fin(ST_DRDY, r_error[eu], 1'b1);           // STANDBY / IDLE / SLEEP
					8'hEF: begin   // SET FEATURES
						case (r_feature)
							8'hCC, 8'h66, 8'h02, 8'h82, 8'hAA, 8'h55, 8'h05, 8'h85,
							8'h69, 8'h67, 8'h96, 8'h9A, 8'h42, 8'hC2:
								fin(ST_DRDY | ST_DSC, r_error[eu], 1'b1);
							8'h03: begin
								case (r_nsec[7:3])
									5'h00, 5'h01: begin
										xmode[eu] <= {2'd1, 3'd0};
										fin(ST_DRDY | ST_DSC, r_error[eu], 1'b1);
									end
									5'h04: begin
										xmode[eu] <= {2'd2, r_nsec[2:0]};
										fin(ST_DRDY | ST_DSC, r_error[eu], 1'b1);
									end
									5'h08: begin
										xmode[eu] <= {2'd3, r_nsec[2:0]};
										fin(ST_DRDY | ST_DSC, r_error[eu], 1'b1);
									end
									default: fin(ST_DRDY | ST_ERR, ER_ABRT, 1'b1);
								endcase
							end
							default: fin(ST_DRDY | ST_ERR, ER_ABRT, 1'b1);
						endcase
					end
					8'h90: begin   // EXECUTE DEVICE DIAGNOSTIC (both devices)
						signature();
						r_select <= r_select & 8'hF0;
						for (i = 0; i < 2; i = i + 1) begin
							r_error[i] <= 8'h01;
							if (present[i]) r_status[i] <= ST_DRDY | ST_DSC;
						end
						pend[eu] <= 1'b1;
						est <= E_IDLE;
					end
					8'hF8: begin   // READ NATIVE MAX ADDRESS (LBA only)
						if (!e_lba_mode) fin(ST_DRDY | ST_ERR, ER_ABRT, 1'b1);
						else est <= E_NMAX;
					end
					default: fin(ST_DRDY | ST_ERR, ER_ABRT, 1'b1);
				endcase
			end
		end

		E_NMAX: begin
			{r_select[3:0], r_hcyl, r_lcyl, r_sect} <= nbs[eu] - 28'd1;
			fin(ST_DRDY, r_error[eu], 1'b1);
		end

		E_INIT_D: est <= E_INIT_W;
		E_INIT_W: if (!dv_busy) begin
			ccyl[eu] <= (dv_q > 28'd65535) ? 16'd65535 : dv_q[15:0];
			fin(ST_DRDY | ST_DSC, 8'h00, 1'b1);
		end

		// IDENTIFY: Hatari default cylinders, then the capacity in MB
		E_ID_DIV: est <= E_ID_DW;
		E_ID_DW: if (!dv_busy) begin
			if (dv_q > 28'd16383) id_dcyl <= 14'd16383;
			else if (dv_q < 28'd2) id_dcyl <= 14'd2;
			else id_dcyl <= dv_q[13:0];
			bcd_bin <= nbs[eu][27:11];
			id_bcd <= 24'd0;
			bcd_cnt <= 5'd17;
			bcd_busy <= 1'b1;
			est <= E_ID_BCD;
		end
		E_ID_BCD: if (!bcd_busy) begin
			// drop leading zero digits (keep at least one)
			if (id_bcd[23:20] == 4'd0 && bcd_cnt != 5'd5) begin
				id_bcd <= {id_bcd[19:0], 4'd0};
				bcd_cnt <= bcd_cnt + 5'd1;
			end else begin
				id_nd <= 3'd5 - bcd_cnt[2:0];
				r_error[eu] <= 8'h00;
				r_status[eu] <= ST_DRDY | ST_DSC | ST_DRQ;
				drq_ident <= 1'b1;
				drq_dir_wr <= 1'b0;
				drq_act <= 1'b1;
				ptr <= 13'd0;
				pend_words <= 13'd256;
				pend[eu] <= 1'b1;
				est <= E_IDDRQ;
			end
		end
		E_IDDRQ: begin
			if (ptr >= pend_words) begin
				drq_act <= 1'b0;
				drq_ident <= 1'b0;
				r_status[eu] <= ST_DRDY | ST_DSC;
				est <= E_IDLE;
			end
		end

		// ---------------- READ SECTORS / MULTIPLE ----------------
		E_RBLK: begin
			bcnt <= 5'd0;
			est <= E_RSEC;
		end
		E_RSEC: begin
			if (!first) incr_addr();
			first <= 1'b0;
			est <= E_RA1;
		end
		E_RA1, E_WA1, E_VA1, E_SKA1: begin
			chs_t1 <= e_cyl * chead[eu] + {17'd0, e_head};
			addr_bad <= !e_lba_mode && (r_sect == 8'd0 || r_sect > cspt[eu] ||
			                            {1'b0, e_head} >= chead[eu] || e_cyl >= ccyl[eu]);
			case (est)
				E_RA1: est <= E_RA2;
				E_WA1: est <= E_WA2;
				E_VA1: est <= E_VA2;
				default: est <= E_SKA2;
			endcase
		end
		E_RA2, E_WA2, E_VA2, E_SKA2: begin
			if (e_lba_mode) lba <= e_lba28;
			else lba <= chs_t1 * cspt[eu] + {20'd0, r_sect} - 28'd1;
			case (est)
				E_RA2: est <= E_RA3;
				E_WA2: est <= E_WA3;
				E_VA2: est <= E_VA3;
				default: est <= E_SKA3;
			endcase
		end
		E_RA3: begin
			if (addr_bad || lba >= nbs[eu]) fin(ST_DRDY | ST_ERR, ER_IDNF, 1'b1);
			else begin
				h_act <= 1'b1; h_unit <= eu; h_wr <= 1'b0; h_sec <= bcnt[3:0];
				sd_lba <= {4'd0, lba};
				sd_rd[eu] <= 1'b1;
				est <= E_RWAIT;
			end
		end
		E_RWAIT: if (h_done) begin
			remain <= remain - 9'd1;
			r_nsec <= r_nsec - 8'd1;
			bcnt <= bcnt + 5'd1;
			if (remain == 9'd1 || bcnt + 5'd1 == blk) begin
				r_status[eu] <= ST_DRDY | ST_DSC | ST_DRQ;
				drq_dir_wr <= 1'b0;
				drq_act <= 1'b1;
				ptr <= 13'd0;
				pend_words <= {bcnt + 5'd1, 8'd0};
				pend[eu] <= 1'b1;
				est <= E_RDRQ;
			end else
				est <= E_RSEC;
		end
		E_RDRQ: begin
			if (ptr >= pend_words) begin
				drq_act <= 1'b0;
				if (remain == 9'd0) begin
					r_status[eu] <= ST_DRDY | ST_DSC;
					est <= E_IDLE;
				end else begin
					r_status[eu] <= ST_BSY | ST_DSC;
					est <= E_RBLK;
				end
			end
		end

		// ---------------- WRITE SECTORS / MULTIPLE ----------------
		E_WBLK: begin
			bsz <= (remain < {4'd0, blk}) ? remain[4:0] : blk;
			pend_words <= {((remain < {4'd0, blk}) ? remain[4:0] : blk), 8'd0};
			ptr <= 13'd0;
			bcnt <= 5'd0;
			drq_dir_wr <= 1'b1;
			drq_act <= 1'b1;
			r_status[eu] <= ST_DRDY | ST_DSC | ST_DRQ;
			est <= E_WDRQ;
		end
		E_WDRQ: begin
			if (ptr >= pend_words) begin
				drq_act <= 1'b0;
				r_status[eu] <= ST_BSY | ST_DSC;
				est <= E_WSEC;
			end
		end
		E_WSEC: begin
			if (!first) incr_addr();
			first <= 1'b0;
			est <= E_WA1;
		end
		E_WA3: begin
			if (addr_bad || lba >= nbs[eu]) fin(ST_DRDY | ST_ERR, ER_IDNF, 1'b1);
			else if (ro[eu]) fin(ST_DRDY | ST_ERR, ER_ABRT, 1'b1);
			else begin
				h_act <= 1'b1; h_unit <= eu; h_wr <= 1'b1; h_sec <= bcnt[3:0];
				sd_lba <= {4'd0, lba};
				sd_wr[eu] <= 1'b1;
				est <= E_WWAIT;
			end
		end
		E_WWAIT: if (h_done) begin
			remain <= remain - 9'd1;
			r_nsec <= r_nsec - 8'd1;
			bcnt <= bcnt + 5'd1;
			if (bcnt + 5'd1 == bsz) begin
				pend[eu] <= 1'b1;
				if (remain == 9'd1) fin(ST_DRDY | ST_DSC, 8'h00, 1'b1);
				else est <= E_WBLK;
			end else
				est <= E_WSEC;
		end

		// ---------------- READ VERIFY ----------------
		E_VSEC: begin
			if (!first) incr_addr();
			first <= 1'b0;
			est <= E_VA1;
		end
		E_VA3: begin
			if (addr_bad || lba >= nbs[eu]) fin(ST_DRDY | ST_ERR, ER_IDNF, 1'b1);
			else begin
				remain <= remain - 9'd1;
				r_nsec <= r_nsec - 8'd1;
				if (remain == 9'd1) fin(ST_DRDY | ST_DSC, 8'h00, 1'b1);
				else est <= E_VSEC;
			end
		end

		// ---------------- SEEK ----------------
		E_SKA3: begin
			if (addr_bad || lba >= nbs[eu]) fin(ST_DRDY | ST_ERR, ER_IDNF, 1'b1);
			else fin(ST_DRDY | ST_DSC, 8'h00, 1'b1);
		end

		default: est <= E_IDLE;
		endcase
		end
	end
end

endmodule
