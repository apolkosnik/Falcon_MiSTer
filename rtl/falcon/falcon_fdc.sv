// falcon_fdc.sv - Atari Falcon030 floppy subsystem: ST DMA chip registers
// $FF8604-$FF860F and a sector level WD1772 floppy disk controller, two
// drives (A, B) backed by .ST images through the hps_io block interface.
//
// Behavioural reference: Hatari src/fdc.c (sector level WD1772 with
// realistic delays) and src/floppy.c (.ST geometry), src/hdc.c / ncr5380.c
// for the "no HDC" behaviour.  Functions followed:
//   FDC_DiskController_WriteWord / FDC_DiskControllerStatus_ReadWord
//   FDC_DmaModeControl_WriteWord / FDC_DmaStatus_ReadWord / FDC_ResetDMA
//   FDC_DmaAddress_WriteByte / FDC_WriteDMAAddress (Falcon: 24 bit, bit 0 = 0)
//   FDC_DensityMode_WriteWord / ReadWord ($FF860E)
//   FDC_DMA_FIFO_Push / FDC_DMA_FIFO_Pull (16 byte blocks, sector count)
//   FDC_WriteCommandRegister (busy / replace rules), FDC_ExecuteCommand,
//   FDC_Type*_*, FDC_Update*Cmd state machines, FDC_Set_MotorON,
//   FDC_CmdCompleteCommon, FDC_UpdateMotorStop, FDC_VerifyTrack,
//   FDC_SetIRQ / FDC_ClearIRQ (forced interrupt rules),
//   FDC_IndexPulse_* (index pulse length 3.71 ms), FDC_NextSectorID_FdcCycles_ST
//   (standard track layout GAP1 60, GAP2 12, GAP3a 22, GAP3b 12, GAP4 40,
//   614 bytes per 512 byte sector), FDC_ReadAddress_ST, FDC_ReadTrack_ST,
//   FDC_CanMachineHandleDensity, FDC_GetBytesPerTrack, FDC_SetDriveSide,
//   Floppy_FindDiskDetails / Floppy_DoubleCheckFormat / Floppy_ReadSectors /
//   Floppy_WriteSectors, Floppy_DriveTransitionUpdateState (WPRT forced on
//   eject for 18 VBL).
//
// Timing: all WD1772 delays are counted in 8 MHz FDC cycles (Hatari uses an
// 8 MHz reference for the Falcon too).  Step rates 6/12/2/3 ms, head settle
// 15 ms, type I prepare 90 us, spin-up 6 index pulses, motor off after 9
// index pulses, 32 us per DD byte (16 us HD, 8 us ED), 300 rpm.
//
// Register bus: bus_addr = A3..A1 of $FF8600+x (2 = $FF8604, 3 = $FF8606,
// 4/5/6 = $FF8608/A/C, 7 = $FF860E).  bus_berr is raised for byte writes to
// $FF8604-$FF8607 (Hatari: "does not like to be accessed in byte mode");
// byte reads are allowed on the Falcon.
//
// Drive selection: drv_sel[0] = PSG port A bit 1 (drive A, active low),
// drv_sel[1] = PSG port A bit 2 (drive B); side_sel = PSG port A bit 0
// as written (1 = side 0, 0 = side 1).  When both drives are selected,
// drive A wins (Hatari FDC_SetDriveSide).  Parameter DRV_PRESENT[n] = drive
// connected (Hatari FDC_DRIVES[].Enabled).  Both drives are double sided.
//
// Disk images: hps_io slot per drive (img_mounted[0] / sd_rd[0] / sd_lba0 /
// sd_buff_din0 = A, [1] = B).  One engine serves both drives: sd_lba0 and
// sd_lba1 carry the same value, so do sd_buff_din0/1; the shared sd_buff_*
// inputs are only accepted while one of our sd_ack bits is set.
// led: WD1772 busy or an image transfer in progress.  The geometry is
// computed from the boot sector and the image size when the image is mounted
// (and again whenever sector 0 is written), as Floppy_FindDiskDetails().
//
// irq: WD1772 INTRQ / HDC interrupt (Hatari FDC.IRQ_Signal), active high;
// the system drives MFP GPIP5 low while it is high.
//
// Deviations from Hatari (hardware wins / sector level limits):
// - Write Track on .ST images is parsed: every ID field (after $F5 syncs,
//   $FE mark, track, side, sector, size) followed by a data field ($FB/$F8)
//   of 512 bytes is written to the image at the physical head position and
//   side, for the sector number of the ID field.  Hatari returns LOST_DATA
//   and writes nothing.  LOST_DATA is set if a data field can not be stored
//   (sector size not 512, sector/track/side outside the image).  Bytes
//   $F5-$F7 inside data fields are stored as written.
// - The DMA has two 16 byte FIFOs: in write (RAM -> disk) mode it prefetches
//   32 bytes as soon as the sector count is non zero (Hatari loads 16 bytes on
//   demand), so the address counter runs up to 32 bytes ahead of the FDC and
//   the sector count reaches 0 when the last 16 bytes are fetched.  The
//   address counter advances by 2 for every word transferred.
// - Toggling the DMA direction (mode bit 8) also sets the status "no error"
//   bit (DMA documentation); Hatari leaves it unchanged.
// - A track is 6250 bytes (DD, 300 rpm at exactly 8 MHz); Hatari uses 6268.
//   The index pulse position is not randomised when the motor starts.
// - FDC pushes in DMA write mode and pulls in DMA read mode transfer nothing
//   (pull returns 0).
// - One disk rotation counter is shared by both drives (only the selected
//   drive's index pulses reach the WD1772).
// - The eject/insert write protect transition is 360 ms (18 VBL at 50 Hz).
// - HDC accesses go to a device-less NCR 5380 model (falcon_fdc_ncr5380)
//   when EXT_SCSI = 0, else out of the hdc_* port to falcon_scsi (the 5380
//   and its targets).
// As in Hatari: the DMA sector count is 14 bits on the Falcon and reads back
// as the latest $FF8604 value, unused DMA status bits come from that value,
// the WD1772 waits forever (busy) for index pulses when no disk is present,
// CRC errors never occur with .ST images.
//
// DMA port: dma_req is gated with dma_ack, so it is low in the acknowledge
// clock; a following word is requested from the next clock on.
//
// HDC port (EXT_SCSI = 1): hdc_acc is the one clock 5380 register access
// (hdc_we, hdc_rs, hdc_wdata), hdc_rdata the 5380 read value for hdc_rs
// (= DMA mode bits 2..0 outside an access); hdc_irq_set / hdc_irq_clr set and
// clear the HDC interrupt source (Hatari FDC_SetIRQ(HDC) / FDC_ClearIRQ).
// The 5380 DMA handshake uses the same FIFO as the WD1772: hdc_push_req /
// hdc_push_byte / hdc_push_ack (SCSI -> RAM) and hdc_pull_req / hdc_pull_byte /
// hdc_pull_ack (RAM -> SCSI) are served only while $FF8606 bits 7:6 = 00 (HDC
// DMA enabled, Hatari dma_check) in the matching direction (bit 8), and a
// push only while the sector count has room for the byte (the transfer stops
// when the count runs out instead of losing data as the WD1772 does).
// hdc_flush (end of the SCSI data in phase) queues a partly filled FIFO half
// for its RAM write (16 bytes, the address advances by 16).  A WD1772 byte
// request in the same clock has priority.

module falcon_fdc #(
	parameter       CLK_HZ      = 32000000,
	parameter [1:0] DRV_PRESENT = 2'b11,     // drives connected (bit 0 = A, bit 1 = B)
	parameter       EXT_SCSI    = 0          // 1 = HDC accesses go to the hdc_* port (falcon_scsi)
) (
	input             clk,
	input             reset,

	// register bus ($FF8604-$FF860F)
	input             bus_cs,
	input             bus_stb,
	input             bus_we,
	input       [3:1] bus_addr,
	input             bus_uds,
	input             bus_lds,
	input      [15:0] bus_din,
	output reg [15:0] bus_dout,
	output reg        bus_ack,
	output reg        bus_berr,

	output            irq,

	// DMA master port (ST-RAM)
	output            dma_req,
	output reg        dma_we,
	output reg [23:1] dma_addr,
	output      [1:0] dma_be,
	output reg [15:0] dma_wdata,
	input      [15:0] dma_rdata,
	input             dma_ack,

	// drive control from the PSG port A
	input       [1:0] drv_sel,      // port A bits 2:1, active low (bit 0 = drive A)
	input             side_sel,     // port A bit 0 (1 = side 0)

	// hps_io disk images (slot 0 = drive A, slot 1 = drive B)
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

	output            led,          // floppy activity (WD1772 busy or image access)

	// HDC side: external NCR 5380 (falcon_scsi), used when EXT_SCSI = 1
	output            hdc_acc,
	output            hdc_we,
	output      [2:0] hdc_rs,
	output      [7:0] hdc_wdata,
	input       [7:0] hdc_rdata,
	input             hdc_irq_set,
	input             hdc_irq_clr,
	input             hdc_push_req,
	input       [7:0] hdc_push_byte,
	output reg        hdc_push_ack,
	input             hdc_pull_req,
	output reg  [7:0] hdc_pull_byte,
	output reg        hdc_pull_ack,
	input             hdc_flush
);

reg  [15:0] sd_lba;
assign sd_lba0 = {16'd0, sd_lba};
assign sd_lba1 = {16'd0, sd_lba};
wire  [1:0] drv_present = DRV_PRESENT;

assign dma_be = 2'b11;

// dma_req is low in the dma_ack clock, so a request that is still pending
// in that clock is never mistaken for the next one (the next word, if any,
// is requested from the following clock on).
reg dma_req_r;
assign dma_req = dma_req_r & ~dma_ack;

// ===========================================================================
// Constants (FDC cycles at 8 MHz)
localparam [23:0] D_TYPE_I_PREP   = 24'd720;
localparam [23:0] D_TYPE_II_PREP  = 24'd8;
localparam [23:0] D_TYPE_IV_PREP  = 24'd800;
localparam [23:0] D_COMPLETE      = 24'd8;
localparam [23:0] D_NO_DRIVE      = 24'd50000;
localparam [23:0] D_REFRESH_IP    = 24'd500;
localparam [23:0] D_HEAD_LOAD     = 24'd120000;     // 15 ms
localparam [20:0] REV_CYCLES      = 21'd1600000;    // 200 ms at 8 MHz
localparam [20:0] IP_LEN          = 21'd29680;      // 3.71 ms
localparam  [8:0] WP_TRANS_MS     = 9'd360;         // 18 VBL (360 ms)

localparam [7:0] STR_BUSY = 8'h01, STR_INDEX = 8'h02, STR_DRQ = 8'h02, STR_TR00 = 8'h04,
                 STR_LOST = 8'h04, STR_CRC = 8'h08, STR_RNF = 8'h10, STR_SPINUP = 8'h20,
                 STR_RT = 8'h20, STR_WPRT = 8'h40, STR_MOTOR = 8'h80;

// ===========================================================================
// 8 MHz FDC cycle enable
reg        cen8;
generate
if (CLK_HZ % 8000000 == 0) begin : g_cen_int
	// integer ratio: plain divider
	localparam integer DIV = CLK_HZ / 8000000;
	reg [7:0] cen_cnt;
	always @(posedge clk) begin
		if (reset) begin
			cen_cnt <= 8'd0;
			cen8 <= 1'b0;
		end else if (cen_cnt == DIV - 1) begin
			cen_cnt <= 8'd0;
			cen8 <= 1'b1;
		end else begin
			cen_cnt <= cen_cnt + 8'd1;
			cen8 <= 1'b0;
		end
	end
end else begin : g_cen_frac
	// fractional accumulator
	reg [31:0] cen_acc;
	always @(posedge clk) begin
		if (reset) begin
			cen_acc <= 32'd0;
			cen8 <= 1'b0;
		end else if (cen_acc + 32'd8000000 >= CLK_HZ) begin
			cen_acc <= cen_acc + 32'd8000000 - CLK_HZ;
			cen8 <= 1'b1;
		end else begin
			cen_acc <= cen_acc + 32'd8000000;
			cen8 <= 1'b0;
		end
	end
end
endgenerate

// ===========================================================================
// CRC16 (CCITT, init $FFFF) - Hatari crc16_add_byte
function [15:0] crc_byte(input [15:0] c, input [7:0] b);
	integer k;
	reg [15:0] x;
	begin
		x = c ^ {b, 8'h00};
		for (k = 0; k < 8; k = k + 1)
			x = x[15] ? ((x << 1) ^ 16'h1021) : (x << 1);
		crc_byte = x;
	end
endfunction

localparam [15:0] CRC_A1A1A1 = 16'hCDB4;   // crc after A1 A1 A1 (init FFFF)
localparam [15:0] CRC_IDAM   = 16'hB230;   // ... and the ID address mark FE
localparam [15:0] CRC_DAM    = 16'hE295;   // ... and the data address mark FB

// ===========================================================================
// Drive state
wire       sel_a = ~drv_sel[0];
wire       sel_b = ~drv_sel[1];
wire       sel_v = sel_a | sel_b;               // a drive is selected
wire       sel   = sel_a ? 1'b0 : 1'b1;         // drive A wins
wire       side  = ~side_sel;

reg  [1:0] inserted;
reg  [1:0] ro;
reg  [1:0] geo_valid;
reg  [6:0] head [0:1];                          // physical head position 0..90
reg  [5:0] g_spt [0:1];                         // sectors per track
reg  [1:0] g_sides [0:1];
reg  [7:0] g_tracks [0:1];
reg  [8:0] wp_force [0:1];                      // WPRT forced to 1 (eject transition), ms
reg [20:0] rot;                                 // disk rotation position in FDC cycles
reg [12:0] ms_div;                              // 1 ms prescaler (8000 FDC cycles)

reg [15:0] density;                             // $FF860E

wire       drv_ok   = sel_v & drv_present[sel];             // selected and enabled
wire       disk_ok  = drv_ok & inserted[sel] & geo_valid[sel];
wire [5:0] c_spt    = g_spt[sel];
wire [1:0] c_sides  = g_sides[sel];
wire [7:0] c_tracks = g_tracks[sel];
wire [6:0] c_head   = head[sel];
// FDC_GetBytesPerTrack / FDC_ComputeFloppyDensity
wire [1:0] dsh      = (c_spt >= 6'd36) ? 2'd2 : (c_spt >= 6'd18) ? 2'd1 : 2'd0;
wire [15:0] bpt     = 16'd6250 << dsh;
wire [8:0] bt       = 9'd256 >> dsh;                         // FDC cycles per byte
// FDC_CanMachineHandleDensity (Falcon)
wire       dens_ok  = (dsh == 2'd0) ? (density[1:0] == 2'b00) : (density[1:0] == 2'b11);

// ===========================================================================
// WD1772 registers
reg  [7:0] STR, TR, SR, DR, CR;
reg        status_type1;
reg        replace_ok;
reg  [2:0] cmd_type;
reg        step_dir;             // 1 = in (+1), 0 = out (-1)
reg  [3:0] int_cond;
reg        irq_forced;
reg        irq_other;            // complete / index / HDC sources
reg  [3:0] ip_cnt;
assign irq = irq_forced | irq_other;
assign led = STR[0] | h_act;

wire motor_on = STR[7];
wire ip_drive = motor_on & drv_ok & inserted[sel] & geo_valid[sel];
wire index_now = ip_drive & (rot < IP_LEN);

// status as read (type I live bits)
reg  [7:0] str_rd;
always @* begin
	str_rd = STR;
	if (status_type1) begin
		if (!drv_ok)
			str_rd = str_rd & ~(STR_TR00 | STR_INDEX | STR_WPRT);
		else begin
			str_rd[2] = (c_head == 7'd0);
			str_rd[1] = index_now;
			str_rd[3] = 1'b0;
			str_rd[6] = ~inserted[sel] | ro[sel] | (wp_force[sel] != 9'd0);
		end
	end
end

// ===========================================================================
// Sector buffer (512 bytes, M10K)
reg  [7:0] sbuf [0:511];
reg  [8:0] sb_waddr, sb_raddr_f;
reg  [7:0] sb_wdata;
reg        sb_we;
reg  [7:0] sb_q;
wire [8:0] sb_raddr;
always @(posedge clk) begin
	if (sb_we) sbuf[sb_waddr] <= sb_wdata;
	sb_q <= sbuf[sb_raddr];
end
assign sd_buff_din0 = sb_q;
assign sd_buff_din1 = sb_q;

// ===========================================================================
// hps / mount engine
reg        h_req;                // FDC request (level until h_done)
reg        h_req_wr;
reg        h_req_unit;
reg [15:0] h_req_lba;
reg        h_done;               // pulse
reg        h_act, h_ack_seen, h_unit, h_wr;
reg        h_scan;               // current transfer is a mount scan
reg  [1:0] m_pend;
reg  [1:0] m_ro;
reg [16:0] m_total [0:1];       // image size in 512 byte sectors (saturated)
reg  [1:0] m_small;             // image smaller than 500 KB (one side)
reg  [1:0] m_empty;             // no image
reg        buf_req;              // FDC owns the buffer (level)
reg        buf_gnt;
reg        g_busy;               // geometry computation running
reg        g_unit;
reg [16:0] g_total;
reg  [7:0] sh19, sh20, sh24, sh25, sh26, sh27;   // boot sector shadow
reg  [3:0] g_st;
reg  [7:0] gd_den;
reg [16:0] gd_q;
reg [16:0] gd_r;
reg  [4:0] gd_cnt;
reg  [5:0] gn_spt;
reg  [1:0] gn_sides;
reg  [3:0] gt_spt;
reg  [2:0] gt_t;
reg  [5:0] gt_step;
reg [16:0] gt_prod;
wire [16:0] gt_next = {11'd0, gt_step} + ((gn_sides == 2'd2) ? 17'd2 : 17'd1);   // (spt + 1) * sides

assign sb_raddr = (h_act & h_wr) ? sd_buff_addr : sb_raddr_f;

// ===========================================================================
// DMA / FIFO
reg [15:0] dmode;
reg [13:0] scount;
reg  [9:0] bis;                  // bytes left in the current 512 byte DMA sector
reg  [2:0] dstat;                // bit0 = no error
reg [15:0] recent;               // ff8604_recent_val
reg  [1:0] half_full;
reg        wh;                   // push half (read direction) / fill half (write)
reg  [3:0] wb;                   // push byte index
reg        rh;                   // pull half
reg  [3:0] rb;                   // pull byte index
reg        xf_act;               // RAM transfer of a half in progress
reg        xf_half;
reg  [2:0] xf_word;
reg  [1:0] xf_wait;              // flush: RAM read latency
reg        xf_abort;

// FIFO storage: 2 x 16 bytes as two byte lanes of 16 entries ({half, word}),
// even bytes (D15..D8) and odd bytes (D7..D0); registered read.
reg  [7:0] ff_e [0:15];
reg  [7:0] ff_o [0:15];
reg        ffe_we, ffo_we;
reg  [3:0] ff_waddr;
reg  [7:0] ffe_wd, ffo_wd;
reg  [7:0] ffe_q, ffo_q;
wire [3:0] ff_raddr;
always @(posedge clk) begin
	if (ffe_we) ff_e[ff_waddr] <= ffe_wd;
	if (ffo_we) ff_o[ff_waddr] <= ffo_wd;
	ffe_q <= ff_e[ff_raddr];
	ffo_q <= ff_o[ff_raddr];
end

// FDC <-> DMA byte handshake
reg        push_req, pull_req;
reg  [7:0] push_byte;
reg        push_ack, pull_ack;
reg  [7:0] pull_byte;

// bus -> FDC request pulses
reg        f_wr;                 // FDC register write
reg  [1:0] f_wr_reg;
reg  [7:0] f_wr_val;
reg        f_rd_st;              // status register read

// NCR 5380: device-less model, or the external falcon_scsi
wire [7:0] ncr_rdata;
wire       ncr_irq_set, ncr_irq_clr;
reg        ncr_acc, ncr_we;
reg  [2:0] ncr_rs;
reg  [7:0] ncr_wdata;
assign hdc_acc   = ncr_acc;
assign hdc_we    = ncr_we;
assign hdc_rs    = ncr_acc ? ncr_rs : dmode[2:0];
assign hdc_wdata = ncr_wdata;
generate
if (EXT_SCSI != 0) begin : g_ext_scsi
	assign ncr_rdata   = hdc_rdata;
	assign ncr_irq_set = hdc_irq_set;
	assign ncr_irq_clr = hdc_irq_clr;
end else begin : g_ncr_stub
	falcon_fdc_ncr5380 ncr (
		.clk(clk), .reset(reset), .acc(ncr_acc), .we(ncr_we), .rs(hdc_rs), .wdata(ncr_wdata),
		.rdata(ncr_rdata), .irq_set(ncr_irq_set), .irq_clr(ncr_irq_clr)
	);
end
endgenerate

// ===========================================================================
// Bus interface + DMA chip
wire a_8604 = (bus_addr == 3'd2);
wire a_8606 = (bus_addr == 3'd3);
wire a_ahi  = (bus_addr == 3'd4);
wire a_amid = (bus_addr == 3'd5);
wire a_alo  = (bus_addr == 3'd6);
wire a_860e = (bus_addr == 3'd7);
wire bytew  = ~(bus_uds & bus_lds);

reg  [7:0] fdc_reg_rd;
always @* begin
	case (dmode[2:1])
		2'd0: fdc_reg_rd = str_rd;
		2'd1: fdc_reg_rd = TR;
		2'd2: fdc_reg_rd = SR;
		default: fdc_reg_rd = DR;
	endcase
end

wire [15:0] reg8604_rd = dmode[4] ? recent : dmode[3] ? {8'h00, ncr_rdata} : {8'h00, fdc_reg_rd};

integer i;
reg [15:0] new_recent;

// FIFO accesses: pushes (disk -> RAM) and fills (RAM -> disk) never happen
// in the same DMA direction, so one write port per lane is enough.
wire push_take = push_req & ~push_ack & ~dmode[8] & (scount != 14'd0) & ~half_full[wh];
wire fill_take = xf_act & dma_ack & ~dma_we & ~xf_abort;
// HDC (5380) DMA: bytes not yet counted by the sector counter (FIFO halves
// waiting for RAM plus the half being filled) must leave room in the count
wire       hdc_dma   = ~dmode[7] & ~dmode[6] & (EXT_SCSI != 0);
wire [5:0] hdc_infl  = {(half_full[0] & half_full[1]), (half_full[0] ^ half_full[1]), 4'd0} + {2'b00, wb};
wire       hdc_room  = (scount > 14'd1) || ((scount == 14'd1) && (bis > {4'd0, hdc_infl}));
wire hdc_push_take = hdc_push_req & ~hdc_push_ack & hdc_dma & ~dmode[8] & hdc_room & ~half_full[wh] &
                     ~(push_req & ~push_ack);
wire hdc_pull_take = hdc_pull_req & ~hdc_pull_ack & hdc_dma & dmode[8] & half_full[rh] &
                     ~(pull_req & ~pull_ack);
assign ff_raddr = dmode[8] ? {rh, rb[3:1]} : {xf_half, xf_word};
always @* begin
	ffe_we = 1'b0; ffo_we = 1'b0;
	ff_waddr = {wh, wb[3:1]};
	ffe_wd = push_byte; ffo_wd = push_byte;
	if (push_take) begin
		ffe_we = ~wb[0];
		ffo_we = wb[0];
	end else if (hdc_push_take) begin
		ffe_we = ~wb[0];
		ffo_we = wb[0];
		ffe_wd = hdc_push_byte;
		ffo_wd = hdc_push_byte;
	end else if (fill_take) begin
		ff_waddr = {xf_half, xf_word};
		ffe_we = 1'b1; ffo_we = 1'b1;
		ffe_wd = dma_rdata[15:8];
		ffo_wd = dma_rdata[7:0];
	end
end

always @(posedge clk) begin
	bus_ack <= 1'b0;
	bus_berr <= 1'b0;
	f_wr <= 1'b0;
	f_rd_st <= 1'b0;
	ncr_acc <= 1'b0;
	push_ack <= 1'b0;
	pull_ack <= 1'b0;
	hdc_push_ack <= 1'b0;
	hdc_pull_ack <= 1'b0;

	if (reset) begin
		dmode <= 16'h0000;
		scount <= 14'd0;
		bis <= 10'd512;
		dstat <= 3'b001;
		recent <= 16'h0000;
		half_full <= 2'b00;
		wh <= 1'b0; wb <= 4'd0; rh <= 1'b0; rb <= 4'd0;
		xf_act <= 1'b0;
		xf_abort <= 1'b0;
		xf_wait <= 2'd0;
		dma_req_r <= 1'b0;
		dma_we <= 1'b0;
		dma_addr <= 23'd0;
		dma_wdata <= 16'h0000;
		density <= 16'h0000;
		bus_dout <= 16'hFFFF;
	end else begin
		new_recent = recent;

		// ----------------------------------------------------------------
		// register accesses (side effects on bus_stb, ack on the next clock)
		if (bus_stb) begin
			bus_ack <= 1'b1;
			if (bus_we) begin
				if ((a_8604 | a_8606) & bytew) begin
					bus_berr <= 1'b1;
				end else if (a_8604) begin
					if (dmode[4]) begin
						scount <= bus_din[13:0];
					end else begin
						new_recent = {new_recent[15:8], bus_din[7:0]};
						if (dmode[3]) begin
							ncr_acc <= 1'b1; ncr_we <= 1'b1; ncr_rs <= dmode[2:0]; ncr_wdata <= bus_din[7:0];
						end else begin
							f_wr <= 1'b1; f_wr_reg <= dmode[2:1]; f_wr_val <= bus_din[7:0];
						end
					end
				end else if (a_8606) begin
					dmode <= bus_din;
					if (dmode[8] ^ bus_din[8]) begin
						// FDC_ResetDMA: empty FIFO, sector count 0
						half_full <= 2'b00;
						wh <= 1'b0; wb <= 4'd0; rh <= 1'b0; rb <= 4'd0;
						scount <= 14'd0;
						bis <= 10'd512;
						dstat[0] <= 1'b1;
						if (xf_act) xf_abort <= 1'b1;
					end
				end else if ((a_ahi | a_amid | a_alo) & bus_lds) begin
					if (a_ahi)  dma_addr[23:16] <= bus_din[7:0];
					if (a_amid) dma_addr[15:8]  <= bus_din[7:0];
					if (a_alo)  dma_addr[7:1]   <= bus_din[7:1];
				end else if (a_860e) begin
					if (bus_uds) density[15:8] <= bus_din[15:8];
					if (bus_lds) density[7:0]  <= bus_din[7:0];
				end
			end else begin
				if (a_8604) begin
					bus_dout <= reg8604_rd;
					if (!dmode[4]) begin
						new_recent = {new_recent[15:8], reg8604_rd[7:0]};
						if (dmode[3]) begin
							ncr_acc <= 1'b1; ncr_we <= 1'b0; ncr_rs <= dmode[2:0];
						end else if (dmode[2:1] == 2'd0)
							f_rd_st <= 1'b1;
					end
				end else if (a_8606) begin
					bus_dout <= {recent[15:3], 1'b0, (scount != 14'd0), dstat[0]};
				end else if (a_ahi)  bus_dout <= {8'hFF, dma_addr[23:16]};
				else if (a_amid) bus_dout <= {8'hFF, dma_addr[15:8]};
				else if (a_alo)  bus_dout <= {8'hFF, dma_addr[7:1], 1'b0};
				else if (a_860e) bus_dout <= density;
				else bus_dout <= 16'hFFFF;
			end
		end

		// ----------------------------------------------------------------
		// FDC byte push (disk -> RAM direction)
		if (push_req && !push_ack) begin
			if (dmode[8]) begin
				push_ack <= 1'b1;                     // wrong direction: lost
			end else if (scount == 14'd0) begin
				new_recent = {new_recent[15:8], push_byte};
				dstat[0] <= 1'b0;
				push_ack <= 1'b1;
			end else if (!half_full[wh]) begin
				// push_take: the byte is written to the FIFO RAM
				new_recent = {new_recent[15:8], push_byte};
				dstat[0] <= 1'b1;
				if (wb == 4'd15) begin
					half_full[wh] <= 1'b1;
					wh <= ~wh;
				end
				wb <= wb + 4'd1;
				push_ack <= 1'b1;
			end
		end

		// FDC byte pull (RAM -> disk direction)
		if (pull_req && !pull_ack) begin
			if (!dmode[8]) begin
				pull_byte <= 8'h00;
				pull_ack <= 1'b1;
			end else if (half_full[rh]) begin
				pull_byte <= rb[0] ? ffo_q : ffe_q;
				new_recent = {new_recent[15:8], rb[0] ? ffo_q : ffe_q};
				dstat[0] <= 1'b1;
				if (rb == 4'd15) begin
					half_full[rh] <= 1'b0;
					rh <= ~rh;
				end
				rb <= rb + 4'd1;
				pull_ack <= 1'b1;
			end else if (scount == 14'd0 && !xf_act) begin
				pull_byte <= 8'h00;
				dstat[0] <= 1'b0;
				pull_ack <= 1'b1;
			end
		end

		// 5380 DMA byte push (SCSI -> RAM) / pull (RAM -> SCSI)
		if (hdc_push_take) begin
			new_recent = {new_recent[15:8], hdc_push_byte};
			dstat[0] <= 1'b1;
			if (wb == 4'd15) begin
				half_full[wh] <= 1'b1;
				wh <= ~wh;
			end
			wb <= wb + 4'd1;
			hdc_push_ack <= 1'b1;
		end else if (hdc_flush && hdc_dma && !dmode[8] && wb != 4'd0 && !half_full[wh] &&
		             !(push_req && !push_ack)) begin
			// end of the data in phase: the partly filled half goes to RAM
			half_full[wh] <= 1'b1;
			wh <= ~wh;
			wb <= 4'd0;
		end
		if (hdc_pull_take) begin
			hdc_pull_byte <= rb[0] ? ffo_q : ffe_q;
			new_recent = {new_recent[15:8], rb[0] ? ffo_q : ffe_q};
			dstat[0] <= 1'b1;
			if (rb == 4'd15) begin
				half_full[rh] <= 1'b0;
				rh <= ~rh;
			end
			rb <= rb + 4'd1;
			hdc_pull_ack <= 1'b1;
		end

		// ----------------------------------------------------------------
		// RAM side of the FIFO
		if (!xf_act) begin
			if (!dmode[8] && (half_full != 2'b00)) begin
				// flush the oldest full half (read direction)
				xf_act <= 1'b1;
				xf_half <= half_full[~wh] ? ~wh : wh;
				xf_word <= 3'd0;
				xf_wait <= 2'd2;
				dma_we <= 1'b1;
			end else if (dmode[8] && scount != 14'd0 && !half_full[wh] && !(rh == wh && rb != 4'd0)) begin
				// prefetch into the next empty half (write direction)
				xf_act <= 1'b1;
				xf_half <= wh;
				xf_word <= 3'd0;
				xf_wait <= 2'd0;
				dma_req_r <= 1'b1;
				dma_we <= 1'b0;
			end
		end else if (xf_wait != 2'd0) begin
			// flush: the RAM output is valid two clocks after the address
			xf_wait <= xf_wait - 2'd1;
			if (xf_wait == 2'd1) begin
				if (xf_abort) begin
					xf_act <= 1'b0;
					xf_abort <= 1'b0;
				end else begin
					dma_wdata <= {ffe_q, ffo_q};
					dma_req_r <= 1'b1;
				end
			end
		end else if (dma_ack) begin
			// a CPU write of the address counter in the same clock wins
			if (!(bus_stb && bus_we && (a_ahi | a_amid | a_alo) && bus_lds))
				dma_addr <= dma_addr + 23'd1;
			// fill_take: the word is written to the FIFO RAM
			new_recent = dma_we ? dma_wdata : dma_rdata;
			if (xf_abort) begin
				dma_req_r <= 1'b0;
				xf_act <= 1'b0;
				xf_abort <= 1'b0;
			end else if (xf_word == 3'd7) begin
				dma_req_r <= 1'b0;
				xf_act <= 1'b0;
				if (dma_we) half_full[xf_half] <= 1'b0;
				else begin
					half_full[xf_half] <= 1'b1;
					wh <= ~wh;
				end
				if (bis == 10'd16) begin
					bis <= 10'd512;
					scount <= scount - 14'd1;
				end else
					bis <= bis - 10'd16;
			end else begin
				xf_word <= xf_word + 3'd1;
				if (dma_we) begin
					dma_req_r <= 1'b0;
					xf_wait <= 2'd2;
				end
			end
		end

		recent <= new_recent;
	end
end

// ===========================================================================
// FDC state machine
localparam [5:0]
	S_IDLE      = 6'd0,
	S_MSTOP     = 6'd1,
	S_MSTOP_W   = 6'd2,
	S_PREP      = 6'd3,
	S_SPIN      = 6'd4,
	S_T1_ON     = 6'd5,
	S_RST_LOOP  = 6'd6,
	S_SEEK_LOOP = 6'd7,
	S_STEP      = 6'd8,
	S_VERIFY    = 6'd9,
	S_V_HEADOK  = 6'd10,
	S_V_NEXT    = 6'd11,
	S_V_CHECK   = 6'd12,
	S_COMPLETE  = 6'd13,
	S_RNF       = 6'd14,
	S_HLOAD     = 6'd15,
	S_R_MOTORON = 6'd16,
	S_R_NEXT    = 6'd17,
	S_R_CHECK   = 6'd18,
	S_R_START   = 6'd19,
	S_R_LOOP    = 6'd20,
	S_R_CRC     = 6'd21,
	S_R_MULTI   = 6'd22,
	S_W_MOTORON = 6'd23,
	S_W_NEXT    = 6'd24,
	S_W_CHECK   = 6'd25,
	S_W_START   = 6'd26,
	S_W_LOOP    = 6'd27,
	S_W_CRC     = 6'd28,
	S_W_WAITH   = 6'd29,
	S_W_MULTI   = 6'd30,
	S_A_MOTORON = 6'd31,
	S_A_NEXT    = 6'd32,
	S_A_START   = 6'd33,
	S_A_LOOP    = 6'd34,
	S_T_MOTORON = 6'd35,
	S_T_INDEX   = 6'd36,
	S_T_LOOP    = 6'd37,
	S_WT_MOTORON= 6'd38,
	S_WT_INDEX  = 6'd39,
	S_WT_LOOP   = 6'd40,
	S_WT_END    = 6'd41,
	S_IDCALC    = 6'd42;

localparam [2:0] C_RESTORE = 3'd0, C_SEEK = 3'd1, C_STEP = 3'd2, C_READ = 3'd3, C_WRITE = 3'd4,
                 C_RADDR = 3'd5, C_RTRACK = 3'd6, C_WTRACK = 3'd7;

reg  [5:0] fst;
reg  [5:0] id_ret;               // state after the ID search
reg  [2:0] fcmd;
reg [23:0] dly;
reg        cmd_pend;             // command register written
reg  [7:0] cmd_val;

// next sector ID (FDC_NextSectorID_FdcCycles_ST)
reg [15:0] id_pos;
reg [15:0] id_tp;
reg  [5:0] id_i;
reg  [7:0] id_tr;
reg  [5:0] id_sr;
reg        id_ra;                // read address: 4 bytes after the ID search, else 10

// transfer
reg  [9:0] bpos;
reg        x_valid;              // image location valid for the transfer
reg [15:0] tcnt;                 // read/write track byte counter
reg [15:0] ttotal;
reg        trandom;
reg [15:0] lfsr;
reg        x_unit;
reg  [7:0] x_track;
reg        x_side;
reg [15:0] crc;
wire [15:0] crc_next;            // crc updated with the byte being pushed

// read track generator
localparam [4:0] G_GAP1 = 5'd0, G_GAP2 = 5'd1, G_IDSYNC = 5'd2, G_IDAM = 5'd3, G_ID = 5'd4,
                 G_IDCRC = 5'd5, G_GAP3A = 5'd6, G_GAP3B = 5'd7, G_DSYNC = 5'd8, G_DAM = 5'd9,
                 G_DATA = 5'd10, G_DCRC = 5'd11, G_GAP4 = 5'd12, G_GAP5 = 5'd13;
reg  [4:0] gseg;
reg  [9:0] gcnt;
reg  [5:0] gsec;
reg        gdata_rdy;            // sector data of gsec is in the buffer
reg        gfetch;               // fetch for gsec issued

// write track parser
localparam [1:0] P_IDLE = 2'd0, P_ID = 2'd1, P_DATA = 2'd2;
reg  [1:0] pst;
reg        psync;
reg  [2:0] pk;
reg [10:0] pdk;
reg        pid_valid;
reg  [7:0] pid_r;
reg  [1:0] pid_n;
reg        plost;
reg        pwr_pend;             // a data field is waiting for its hps write

// state helpers
reg        go;                   // delay elapsed
assign crc_next = crc_byte(crc, push_byte);

// LBA of a sector (Floppy_ReadSectors): (track * sides + side) * spt + sector - 1
// One shared multiplier; the operands follow the state that issues the request.
wire        l_cur   = (fst == S_R_CHECK);              // current head / side / drive
wire  [7:0] l_trk   = l_cur ? {1'b0, head[sel]} : x_track;
wire        l_side  = l_cur ? ~side_sel : x_side;
wire  [1:0] l_sides = l_cur ? g_sides[sel] : g_sides[x_unit];
wire  [5:0] l_spt   = l_cur ? g_spt[sel] : g_spt[x_unit];
wire  [5:0] l_sec   = (fst == S_T_LOOP) ? gsec : (fst == S_WT_LOOP || fst == S_WT_END) ? pid_r[5:0] : SR[5:0];
wire  [8:0] l_ts    = ((l_sides == 2'd2) ? {l_trk, 1'b0} : {1'b0, l_trk}) + {8'd0, l_side};
wire [14:0] l_prod  = l_ts * l_spt;
wire [15:0] l_lba   = {1'b0, l_prod} + {10'd0, l_sec} - 16'd1;

// FDC_StepRate_ms = { 6, 12, 2, 3 }
wire [23:0] step_cycles = (CR[1:0] == 2'd0) ? 24'd48000 : (CR[1:0] == 2'd1) ? 24'd96000 :
                          (CR[1:0] == 2'd2) ? 24'd16000 : 24'd24000;

reg ip_event;                    // index pulse of the selected drive (one clock)

// buffer write port users: hps (incoming), FDC
reg        f_sb_we;
reg  [8:0] f_sb_waddr;
reg  [7:0] f_sb_wdata;

always @* begin
	if (h_act & ~h_wr & sd_buff_wr & sd_ack[h_unit]) begin
		sb_we = 1'b1; sb_waddr = sd_buff_addr; sb_wdata = sd_buff_dout;
	end else begin
		sb_we = f_sb_we; sb_waddr = f_sb_waddr; sb_wdata = f_sb_wdata;
	end
end

// -------- rotation / index pulses / WP transitions --------
// One rotation counter: only the selected drive's index pulses are seen by
// the WD1772 (Hatari keeps one index reference per drive and forgets it when
// the drive is deselected).
always @(posedge clk) begin
	ip_event <= 1'b0;
	if (reset) begin
		rot <= 21'd0;
		ms_div <= 13'd0;
	end else if (cen8) begin
		if (motor_on) begin
			if (rot == REV_CYCLES - 21'd1) begin
				rot <= 21'd0;
				if (ip_drive) ip_event <= 1'b1;
			end else
				rot <= rot + 21'd1;
		end
		if (ms_div == 13'd7999) begin
			ms_div <= 13'd0;
			for (i = 0; i < 2; i = i + 1)
				if (wp_force[i] != 9'd0) wp_force[i] <= wp_force[i] - 9'd1;
		end else
			ms_div <= ms_div + 13'd1;
	end
	for (i = 0; i < 2; i = i + 1)
		if (img_mounted[i] && inserted[i]) wp_force[i] <= WP_TRANS_MS;
end

// -------- main FDC process --------
task automatic complete(input doirq);
	begin
		STR[0] <= 1'b0;
		if (doirq) irq_other <= 1'b1;
		buf_req <= 1'b0;
		fst <= S_MSTOP;
		dly <= 24'd0;
	end
endtask

// type II/III "wait for a valid drive" helper used by several states
// FDC_NextSectorID_FdcCycles_ST returns FDCEMU_RETURN_NO_DRIVE_FLOPPY otherwise
wire [20:0] rot_bytes = rot >> (8 - dsh);        // track position in bytes
wire [15:0] raw_len   = 16'd60 + ({10'd0, c_spt} << 9) + ({10'd0, c_spt} << 6) + ({10'd0, c_spt} << 5) +
                        ({10'd0, c_spt} << 2) + ({10'd0, c_spt} << 1);   // GAP1 + spt x 614
wire [15:0] id_nb = (id_i == c_spt) ? bpt - id_pos + 16'd72 : id_tp - id_pos;
wire id_ok = disk_ok & motor_on & ({1'b0, c_head} < c_tracks) & dens_ok;

always @(posedge clk) begin
	f_sb_we <= 1'b0;

	if (reset) begin
		// Hatari FDC_Reset (warm reset): TR and DR are kept, the heads stay
		STR <= 8'h00;
		SR <= 8'h01;
		CR <= 8'h00;
		status_type1 <= 1'b0;
		replace_ok <= 1'b0;
		cmd_type <= 3'd0;
		step_dir <= 1'b1;
		int_cond <= 4'd0;
		irq_forced <= 1'b0;
		irq_other <= 1'b0;
		ip_cnt <= 4'd0;
		fst <= S_IDLE;
		fcmd <= C_RESTORE;
		dly <= 24'd0;
		cmd_pend <= 1'b0;
		push_req <= 1'b0;
		pull_req <= 1'b0;
		buf_req <= 1'b0;
		h_req <= 1'b0;
		lfsr <= 16'hACE1;
		sb_raddr_f <= 9'd0;
	end else begin
		// ---------------- delay counter ----------------
		go = 1'b0;
		if (dly != 24'd0) begin
			if (cen8) dly <= dly - 24'd1;
		end else
			go = 1'b1;

		// ---------------- index pulse counter ----------------
		if (ip_event) begin
			if (ip_cnt != 4'd15) ip_cnt <= ip_cnt + 4'd1;
			if (int_cond[2]) irq_other <= 1'b1;   // force interrupt on index pulse
		end

		lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};

		// ---------------- interrupts from the NCR model ----------------
		if (ncr_irq_set) irq_other <= 1'b1;

		// ---------------- state machine ----------------
		if (cmd_pend) begin
			// FDC_ExecuteCommand
			cmd_pend <= 1'b0;
			CR <= cmd_val;
			push_req <= 1'b0;
			pull_req <= 1'b0;
			buf_req <= 1'b0;
			// a forced interrupt whose condition was removed is stopped;
			// FDC_ClearIRQ for type I-III (keeps only a forced interrupt)
			irq_forced <= irq_forced & int_cond[3];
			if (cmd_val[7:4] != 4'hD) irq_other <= 1'b0;
			int_cond <= 4'd0;
			replace_ok <= 1'b1;
			if (!cmd_val[7]) begin
				// type I
				cmd_type <= 3'd1;
				status_type1 <= 1'b1;
				STR <= (STR & ~(STR_INDEX | STR_CRC | STR_RNF)) | STR_BUSY;
				casez (cmd_val[6:4])
					3'b000: fcmd <= C_RESTORE;
					3'b001: fcmd <= C_SEEK;
					3'b01?: fcmd <= C_STEP;
					3'b10?: begin fcmd <= C_STEP; step_dir <= 1'b1; end
					default: begin fcmd <= C_STEP; step_dir <= 1'b0; end
				endcase
				fst <= S_PREP;
				dly <= D_TYPE_I_PREP;
			end else if (!cmd_val[6]) begin
				// type II
				cmd_type <= 3'd2;
				status_type1 <= 1'b0;
				if (!cmd_val[5]) begin
					fcmd <= C_READ;
					STR <= (STR & ~(STR_DRQ | STR_LOST | STR_CRC | STR_RNF | STR_RT | STR_WPRT)) | STR_BUSY;
				end else begin
					fcmd <= C_WRITE;
					STR <= (STR & ~(STR_DRQ | STR_LOST | STR_CRC | STR_RNF | STR_RT)) | STR_BUSY;
				end
				fst <= S_PREP;
				dly <= D_TYPE_II_PREP;
			end else if (cmd_val[7:4] != 4'hD) begin
				// type III
				cmd_type <= 3'd3;
				status_type1 <= 1'b0;
				fcmd <= (cmd_val[5:4] == 2'b00) ? C_RADDR : (cmd_val[5:4] == 2'b10) ? C_RTRACK : C_WTRACK;
				STR <= (STR & ~(STR_DRQ | STR_LOST | STR_CRC | STR_RNF | STR_RT | STR_WPRT)) | STR_BUSY;
				fst <= S_PREP;
				dly <= D_TYPE_II_PREP;
			end else begin
				// type IV: force interrupt
				cmd_type <= 3'd4;
				if (!STR[0]) begin
					status_type1 <= 1'b1;
					STR <= (STR & ~STR_SPINUP) | STR_MOTOR;
				end else
					STR <= STR & ~STR_BUSY;
				int_cond <= cmd_val[3:0];
				if (cmd_val[3]) irq_forced <= 1'b1;   // FDC_SetIRQ(FDC_IRQ_SOURCE_FORCED)
				else irq_other <= 1'b0;               // FDC_ClearIRQ
				fst <= S_MSTOP;
				dly <= D_TYPE_IV_PREP;
			end
		end else if (go) begin
			// write sector: write protect is checked at every step
			if (fcmd == C_WRITE && fst >= S_PREP && fst != S_MSTOP && fst != S_MSTOP_W &&
			    drv_ok && inserted[sel] && ro[sel] && !push_req && !pull_req && !h_req) begin
				STR <= STR | STR_WPRT;
				complete(1'b1);
			end else begin
			if (fcmd == C_WRITE && fst >= S_PREP) STR[6] <= 1'b0;
			case (fst)
			S_IDLE: ;

			// FDC_UpdateMotorStop
			S_MSTOP: begin
				ip_cnt <= 4'd0;
				fst <= S_MSTOP_W;
			end
			S_MSTOP_W: begin
				if (ip_cnt < 4'd9) dly <= D_REFRESH_IP;
				else begin
					STR[7] <= 1'b0;
					ip_cnt <= 4'd0;
					fst <= S_IDLE;
				end
			end

			// FDC_Set_MotorON
			S_PREP: begin
				STR[7] <= 1'b1;
				if (!CR[3] && !motor_on) begin
					STR[5] <= 1'b0;
					ip_cnt <= 4'd0;
					fst <= S_SPIN;
					dly <= D_REFRESH_IP;
				end else
					fst <= (fcmd <= C_STEP) ? S_T1_ON : S_HLOAD;
			end
			S_SPIN: begin
				if (ip_cnt < 4'd6) dly <= D_REFRESH_IP;
				else fst <= (fcmd <= C_STEP) ? S_T1_ON : S_HLOAD;
			end

			// ---------------- type I ----------------
			S_T1_ON: begin
				STR[5] <= 1'b1;
				replace_ok <= 1'b0;
				case (fcmd)
					C_RESTORE: begin TR <= 8'hFF; fst <= S_RST_LOOP; end
					C_SEEK: fst <= S_SEEK_LOOP;
					default: fst <= S_STEP;
				endcase
			end
			S_RST_LOOP: begin
				if (TR == 8'd0) begin
					STR <= (STR | STR_RNF) & ~STR_TR00;
					complete(1'b1);
				end else if (!drv_ok || c_head != 7'd0) begin
					STR[2] <= 1'b0;
					TR <= TR - 8'd1;
					if (drv_ok) head[sel] <= c_head - 7'd1;
					dly <= step_cycles;
				end else begin
					STR[2] <= 1'b1;
					TR <= 8'd0;
					fst <= S_VERIFY;
				end
			end
			S_SEEK_LOOP: begin
				if (TR == DR) fst <= S_VERIFY;
				else begin
					step_dir <= (DR > TR);
					TR <= (DR > TR) ? TR + 8'd1 : TR - 8'd1;
					dly <= step_cycles;
					STR[2] <= 1'b0;
					if (drv_ok) begin
						if (c_head == 7'd90 && DR > TR) begin
							fst <= S_VERIFY;
							dly <= 24'd0;
						end else if (c_head == 7'd0 && DR < TR) begin
							TR <= 8'd0;
							fst <= S_VERIFY;
							dly <= 24'd0;
							STR[2] <= 1'b1;
						end else begin
							head[sel] <= (DR > TR) ? c_head + 7'd1 : c_head - 7'd1;
							if (DR < TR && c_head == 7'd1) STR[2] <= 1'b1;
						end
					end
				end
			end
			S_STEP: begin
				if (CR[4]) TR <= step_dir ? TR + 8'd1 : TR - 8'd1;
				dly <= step_cycles;
				STR[2] <= 1'b0;
				if (drv_ok) begin
					if (c_head == 7'd90 && step_dir) dly <= 24'd0;
					else if (c_head == 7'd0 && !step_dir) begin
						dly <= 24'd0;
						STR[2] <= 1'b1;
					end else begin
						head[sel] <= step_dir ? c_head + 7'd1 : c_head - 7'd1;
						if (!step_dir && c_head == 7'd1) STR[2] <= 1'b1;
					end
				end
				fst <= S_VERIFY;
			end
			S_VERIFY: begin
				if (CR[2]) begin
					fst <= S_V_HEADOK;
					dly <= D_HEAD_LOAD;
				end else begin
					fst <= S_COMPLETE;
					dly <= D_COMPLETE;
				end
			end
			S_V_HEADOK: begin
				ip_cnt <= 4'd0;
				fst <= S_V_NEXT;
			end
			S_V_NEXT, S_R_NEXT, S_W_NEXT, S_A_NEXT: begin
				if (ip_cnt >= 4'd5) begin
					if (fst == S_V_NEXT) begin
						STR <= STR | STR_RNF;
						fst <= S_COMPLETE;
						dly <= D_COMPLETE;
					end else
						fst <= S_RNF;
				end else if (!id_ok) begin
					dly <= D_NO_DRIVE;
				end else begin
					// start the ID search from the current position
					id_pos <= rot_bytes[15:0];
					id_tp <= 16'd72;
					id_i <= 6'd0;
					id_tr <= {1'b0, c_head};
					id_ra <= (fst == S_A_NEXT);
					id_ret <= (fst == S_V_NEXT) ? S_V_CHECK : (fst == S_R_NEXT) ? S_R_CHECK :
					          (fst == S_W_NEXT) ? S_W_CHECK : S_A_START;
					fst <= S_IDCALC;
				end
			end
			S_IDCALC: begin
				if (id_i == c_spt || id_pos < id_tp) begin
					// next ID field found; past the last one: next index then sector 1
					id_sr <= (id_i == c_spt) ? 6'd1 : id_i + 6'd1;
					// + 3 x A1, FE (read address) or the whole ID field (10 bytes)
					dly <= ({8'd0, id_nb + (id_ra ? 16'd4 : 16'd10)} << (8 - dsh));
					fst <= id_ret;
				end else begin
					id_tp <= id_tp + 16'd614;
					id_i <= id_i + 6'd1;
				end
			end
			S_V_CHECK: begin
				// FDC_VerifyTrack
				if (drv_ok && inserted[sel] && id_tr == TR && !(side && c_sides != 2'd2)) begin
					STR <= STR & ~STR_RNF;
					fst <= S_COMPLETE;
					dly <= D_COMPLETE;
				end else
					fst <= S_V_NEXT;
			end
			S_COMPLETE: complete(1'b1);
			S_RNF: begin
				STR <= STR | STR_RNF;
				complete(1'b1);
			end

			// ---------------- type II / III common ----------------
			S_HLOAD: begin
				if (fcmd >= C_RADDR) replace_ok <= 1'b0;
				fst <= (fcmd == C_READ) ? S_R_MOTORON : (fcmd == C_WRITE) ? S_W_MOTORON :
				       (fcmd == C_RADDR) ? S_A_MOTORON : (fcmd == C_RTRACK) ? S_T_MOTORON : S_WT_MOTORON;
				if (CR[2]) dly <= D_HEAD_LOAD;
			end

			// ---------------- read sector ----------------
			S_R_MOTORON, S_W_MOTORON, S_A_MOTORON: begin
				replace_ok <= 1'b0;
				ip_cnt <= 4'd0;
				if (!sel_v) dly <= D_NO_DRIVE;
				else fst <= (fst == S_R_MOTORON) ? S_R_NEXT : (fst == S_W_MOTORON) ? S_W_NEXT : S_A_NEXT;
			end
			S_R_CHECK, S_W_CHECK: begin
				if (!sel_v) dly <= D_NO_DRIVE;
				else if (h_req) ;   // an aborted command's transfer is still running
				else if (id_tr == TR && {2'b00, id_sr} == SR) begin
					x_unit <= sel;
					x_track <= {1'b0, c_head};
					x_side <= side;
					x_valid <= inserted[sel] && geo_valid[sel] && ({1'b0, side} < c_sides) &&
					           ({1'b0, c_head} < c_tracks) && SR != 8'd0 && SR <= {2'b00, c_spt};
					dly <= 24'd38 << (8 - dsh);    // GAP3a + GAP3b + 3 x A1 + FB
					if (fst == S_R_CHECK) begin
						if (inserted[sel] && geo_valid[sel] && ({1'b0, side} < c_sides) &&
						    ({1'b0, c_head} < c_tracks) && SR != 8'd0 && SR <= {2'b00, c_spt}) begin
							buf_req <= 1'b1;
							h_req <= 1'b1;
							h_req_wr <= 1'b0;
							h_req_unit <= sel;
							h_req_lba <= l_lba;
						end
						fst <= S_R_START;
					end else
						fst <= S_W_START;
				end else
					fst <= (fst == S_R_CHECK) ? S_R_NEXT : S_W_NEXT;
			end
			S_R_START: begin
				if (!sel_v) dly <= D_NO_DRIVE;
				else if (!x_valid) fst <= S_RNF;
				else if (!h_req) begin
					STR[5] <= 1'b0;
					bpos <= 10'd0;
					sb_raddr_f <= 9'd0;
					dly <= {15'd0, bt};
					fst <= S_R_LOOP;
				end
			end
			S_R_LOOP: begin
				if (!push_req) begin
					push_req <= 1'b1;
					push_byte <= sb_q;
				end else if (push_ack) begin
					push_req <= 1'b0;
					bpos <= bpos + 10'd1;
					sb_raddr_f <= bpos[8:0] + 9'd1;
					if (bpos != 10'd511) dly <= {15'd0, bt};
					else begin
						buf_req <= 1'b0;
						dly <= {15'd0, bt} << 1;
						fst <= S_R_CRC;
					end
				end
			end
			S_R_CRC: fst <= S_R_MULTI;
			S_R_MULTI: begin
				if (CR[4]) begin
					SR <= SR + 8'd1;
					ip_cnt <= 4'd0;
					fst <= S_R_NEXT;
				end else begin
					fst <= S_COMPLETE;
					dly <= D_COMPLETE;
				end
			end

			// ---------------- write sector ----------------
			S_W_START: begin
				if (!sel_v) dly <= D_NO_DRIVE;
				else begin
					buf_req <= 1'b1;
					if (buf_gnt) begin
						bpos <= 10'd0;
						fst <= S_W_LOOP;
					end
				end
			end
			S_W_LOOP: begin
				if (bpos == 10'd512) begin
					dly <= {15'd0, bt} << 1;
					fst <= S_W_CRC;
				end else if (!pull_req) begin
					pull_req <= 1'b1;
				end else if (pull_ack) begin
					pull_req <= 1'b0;
					f_sb_we <= 1'b1;
					f_sb_waddr <= bpos[8:0];
					f_sb_wdata <= pull_byte;
					bpos <= bpos + 10'd1;
					dly <= {15'd0, bt};
				end
			end
			S_W_CRC: begin
				// Floppy_WriteSectors
				if (!x_valid || !inserted[x_unit] || ro[x_unit]) begin
					buf_req <= 1'b0;
					fst <= S_RNF;
				end else if (!h_req) begin
					h_req <= 1'b1;
					h_req_wr <= 1'b1;
					h_req_unit <= x_unit;
					h_req_lba <= l_lba;
					fst <= S_W_WAITH;
				end
			end
			S_W_WAITH: begin
				if (!h_req) begin
					buf_req <= 1'b0;
					fst <= S_W_MULTI;
				end
			end
			S_W_MULTI: begin
				if (CR[4]) begin
					SR <= SR + 8'd1;
					fst <= S_W_MOTORON;
				end else begin
					fst <= S_COMPLETE;
					dly <= D_COMPLETE;
				end
			end

			// ---------------- read address ----------------
			S_A_START: begin
				if (!sel_v) dly <= D_NO_DRIVE;
				else begin
					// FDC_ReadAddress_ST: TR SIDE SR LEN CRC1 CRC2
					x_side <= side;
					crc <= CRC_IDAM;
					SR <= id_tr;
					bpos <= 10'd0;
					dly <= {15'd0, bt};
					fst <= S_A_LOOP;
				end
			end
			S_A_LOOP: begin
				if (!push_req) begin
					push_req <= 1'b1;
					case (bpos[2:0])
						3'd0: push_byte <= id_tr;
						3'd1: push_byte <= {7'd0, x_side};
						3'd2: push_byte <= {2'b00, id_sr};
						3'd3: push_byte <= 8'd2;
						3'd4: push_byte <= crc[15:8];
						default: push_byte <= crc[7:0];
					endcase
				end else if (push_ack) begin
					push_req <= 1'b0;
					if (bpos < 10'd4) crc <= crc_next;
					bpos <= bpos + 10'd1;
					if (bpos != 10'd5) dly <= {15'd0, bt};
					else begin
						fst <= S_COMPLETE;
						dly <= D_COMPLETE;
					end
				end
			end

			// ---------------- read / write track: wait for the index ----------------
			S_T_MOTORON, S_WT_MOTORON: begin
				if (!(ip_drive)) dly <= D_NO_DRIVE;
				else begin
					dly <= (REV_CYCLES - rot <= 21'd1) ? {3'd0, REV_CYCLES} : {3'd0, REV_CYCLES - rot};
					fst <= (fst == S_T_MOTORON) ? S_T_INDEX : S_WT_INDEX;
				end
			end

			// ---------------- read track ----------------
			S_T_INDEX: begin
				if (!sel_v) dly <= D_NO_DRIVE;
				else begin
					x_unit <= sel;
					x_track <= {1'b0, c_head};
					x_side <= side;
					trandom <= (side && c_sides != 2'd2) || !dens_ok || ({1'b0, c_head} >= c_tracks);
					if ((side && c_sides != 2'd2) || !dens_ok || ({1'b0, c_head} >= c_tracks))
						ttotal <= bpt;
					else
						ttotal <= (raw_len > bpt) ? raw_len : bpt;
					tcnt <= 16'd0;
					gseg <= G_GAP1;
					gcnt <= 10'd0;
					gsec <= 6'd1;
					gfetch <= 1'b0;
					gdata_rdy <= 1'b0;
					buf_req <= 1'b1;
					dly <= {15'd0, bt};
					fst <= S_T_LOOP;
				end
			end
			S_T_LOOP: begin
				// fetch the data of the current sector at the start of its GAP2
				if (!trandom && gseg == G_GAP2 && !gfetch && buf_gnt && !h_req) begin
					gfetch <= 1'b1;
					gdata_rdy <= 1'b0;
					h_req <= 1'b1;
					h_req_wr <= 1'b0;
					h_req_unit <= x_unit;
					h_req_lba <= l_lba;
				end
				if (gfetch && !h_req && !gdata_rdy) gdata_rdy <= 1'b1;
				if (!push_req) begin
					if (trandom) begin
						push_req <= 1'b1;
						push_byte <= lfsr[7:0];
					end else if (gseg != G_DATA || gdata_rdy) begin
						push_req <= 1'b1;
						case (gseg)
							G_GAP1, G_GAP3A, G_GAP4, G_GAP5: push_byte <= 8'h4E;
							G_GAP2, G_GAP3B: push_byte <= 8'h00;
							G_IDSYNC, G_DSYNC: push_byte <= 8'hA1;
							G_IDAM: push_byte <= 8'hFE;
							G_ID: push_byte <= (gcnt == 10'd0) ? x_track : (gcnt == 10'd1) ? {7'd0, x_side} :
							                   (gcnt == 10'd2) ? {2'b00, gsec} : 8'd2;
							G_IDCRC, G_DCRC: push_byte <= (gcnt == 10'd0) ? crc[15:8] : crc[7:0];
							G_DAM: push_byte <= 8'hFB;
							default: push_byte <= sb_q;   // G_DATA
						endcase
					end
				end else if (push_ack) begin
					push_req <= 1'b0;
					tcnt <= tcnt + 16'd1;
					// advance the generator
					gcnt <= gcnt + 10'd1;
					case (gseg)
						G_GAP1:   if (gcnt == 10'd59) begin gseg <= (c_spt == 6'd0) ? G_GAP5 : G_GAP2; gcnt <= 10'd0; end
						G_GAP2:   if (gcnt == 10'd11) begin gseg <= G_IDSYNC; gcnt <= 10'd0; end
						G_IDSYNC: if (gcnt == 10'd2) begin gseg <= G_IDAM; gcnt <= 10'd0; end
						G_IDAM:   begin
							gseg <= G_ID; gcnt <= 10'd0;
							crc <= CRC_IDAM;
						end
						G_ID: begin
							crc <= crc_next;
							if (gcnt == 10'd3) begin gseg <= G_IDCRC; gcnt <= 10'd0; end
						end
						G_IDCRC:  if (gcnt == 10'd1) begin gseg <= G_GAP3A; gcnt <= 10'd0; end
						G_GAP3A:  if (gcnt == 10'd21) begin gseg <= G_GAP3B; gcnt <= 10'd0; end
						G_GAP3B:  if (gcnt == 10'd11) begin gseg <= G_DSYNC; gcnt <= 10'd0; end
						G_DSYNC:  if (gcnt == 10'd2) begin gseg <= G_DAM; gcnt <= 10'd0; end
						G_DAM: begin
							gseg <= G_DATA; gcnt <= 10'd0;
							crc <= CRC_DAM;
							sb_raddr_f <= 9'd0;
						end
						G_DATA: begin
							crc <= crc_next;
							sb_raddr_f <= gcnt[8:0] + 9'd1;
							if (gcnt == 10'd511) begin gseg <= G_DCRC; gcnt <= 10'd0; end
						end
						G_DCRC:   if (gcnt == 10'd1) begin gseg <= G_GAP4; gcnt <= 10'd0; end
						G_GAP4:   if (gcnt == 10'd39) begin
							gcnt <= 10'd0;
							gfetch <= 1'b0;
							gdata_rdy <= 1'b0;
							if (gsec == c_spt) gseg <= G_GAP5;
							else begin gseg <= G_GAP2; gsec <= gsec + 6'd1; end
						end
						default: ;
					endcase
					if (tcnt + 16'd1 == ttotal) begin
						buf_req <= 1'b0;
						fst <= S_COMPLETE;
						dly <= D_COMPLETE;
					end else
						dly <= {15'd0, bt};
				end
			end

			// ---------------- write track ----------------
			S_WT_INDEX: begin
				if (!sel_v) dly <= D_NO_DRIVE;
				else if (!dens_ok) begin
					STR <= STR | STR_LOST;
					complete(1'b1);
				end else if (ro[sel]) begin
					STR <= STR | STR_WPRT;
					complete(1'b1);
				end else begin
					STR[6] <= 1'b0;
					x_unit <= sel;
					x_track <= {1'b0, c_head};
					x_side <= side;
					tcnt <= bpt;
					pst <= P_IDLE;
					psync <= 1'b0;
					pid_valid <= 1'b0;
					plost <= 1'b0;
					pwr_pend <= 1'b0;
					buf_req <= 1'b1;
					fst <= S_WT_LOOP;
				end
			end
			S_WT_LOOP: begin
				if (pwr_pend && !h_req) begin
					// data field complete: write it to the image
					pwr_pend <= 1'b0;
					h_req <= 1'b1;
					h_req_wr <= 1'b1;
					h_req_unit <= x_unit;
					h_req_lba <= l_lba;
				end
				if (tcnt == 16'd0) begin
					fst <= S_WT_END;
				end else if (!pull_req) begin
					// a data field needs the buffer: wait for the previous write
					if (!(pst == P_DATA && (h_req || pwr_pend)) && buf_gnt) pull_req <= 1'b1;
				end else if (pull_ack) begin
					pull_req <= 1'b0;
					tcnt <= tcnt - 16'd1;
					dly <= {15'd0, bt};
					case (pst)
						P_IDLE: begin
							if (pull_byte == 8'hF5) psync <= 1'b1;
							else if (psync && pull_byte == 8'hFE) begin
								pst <= P_ID; pk <= 3'd0; psync <= 1'b0;
							end else if (psync && (pull_byte == 8'hFB || pull_byte == 8'hF8) && pid_valid) begin
								pst <= P_DATA; pdk <= 11'd0; psync <= 1'b0;
							end else psync <= 1'b0;
						end
						P_ID: begin
							pk <= pk + 3'd1;
							if (pk == 3'd2) pid_r <= pull_byte;
							if (pk == 3'd3) begin
								pid_n <= pull_byte[1:0];
								pid_valid <= 1'b1;
								pst <= P_IDLE;
							end
						end
						default: begin   // P_DATA
							if (pdk < 11'd512) begin
								f_sb_we <= 1'b1;
								f_sb_waddr <= pdk[8:0];
								f_sb_wdata <= pull_byte;
							end
							pdk <= pdk + 11'd1;
							if (pdk + 11'd1 == (11'd128 << pid_n)) begin
								pst <= P_IDLE;
								pid_valid <= 1'b0;
								if (pid_n == 2'd2 && pid_r != 8'd0 && pid_r <= {2'b00, g_spt[x_unit]} &&
								    x_track < g_tracks[x_unit] && {1'b0, x_side} < g_sides[x_unit] &&
								    inserted[x_unit] && geo_valid[x_unit])
									pwr_pend <= 1'b1;
								else
									plost <= 1'b1;
							end
						end
					endcase
				end
			end
			S_WT_END: begin
				if (pwr_pend && !h_req) begin
					pwr_pend <= 1'b0;
					h_req <= 1'b1;
					h_req_wr <= 1'b1;
					h_req_unit <= x_unit;
					h_req_lba <= l_lba;
				end else if (!pwr_pend && !h_req) begin
					if (plost) STR <= STR | STR_LOST;
					complete(1'b1);
				end
			end

			default: fst <= S_IDLE;
			endcase
			end
		end

		// hps request completion
		if (h_done) h_req <= 1'b0;

		// ---------------- register writes from the CPU ----------------
		if (f_wr) begin
			case (f_wr_reg)
				2'd0: begin
					// FDC_WriteCommandRegister: busy rules
					if (!STR[0] || f_wr_val[7:4] == 4'hD ||
					    (replace_ok && ((!f_wr_val[7] && cmd_type == 3'd1) ||
					                    (f_wr_val[7:6] == 2'b10 && cmd_type == 3'd2)))) begin
						cmd_pend <= 1'b1;
						cmd_val <= f_wr_val;
					end
				end
				2'd1: TR <= f_wr_val;
				2'd2: SR <= f_wr_val;
				default: DR <= f_wr_val;
			endcase
		end
		if (f_rd_st) begin
			// status read: latch the live type I bits, clear the interrupt
			STR <= str_rd;
			if (irq_forced && !int_cond[3]) begin
				irq_forced <= 1'b0;
				irq_other <= 1'b0;
			end else if (!irq_forced)
				irq_other <= 1'b0;
		end
		if (ncr_irq_clr) begin
			// Hatari ncr5380_bget(7) -> FDC_ClearIRQ
			if (!irq_forced) irq_other <= 1'b0;
		end
	end
end

// ===========================================================================
// hps engine: FDC sector transfers and mount scans

// The inserted disks, their geometry, mount requests, a mount scan and an
// image transfer in flight survive `reset` (machine reset or the 68030 RESET
// instruction): the drives keep their disks and hps_io finishes the block it
// is serving.  Only the buffer grant is reset (the FDC side drops buf_req).
initial begin
	h_act = 1'b0; h_ack_seen = 1'b0; h_scan = 1'b0; h_unit = 1'b0; h_wr = 1'b0;
	sd_rd = 2'b00; sd_wr = 2'b00; sd_lba = 16'd0;
	m_pend = 2'b00; m_ro = 2'b00; m_small = 2'b00; m_empty = 2'b11;
	buf_gnt = 1'b0; inserted = 2'b00; geo_valid = 2'b00; ro = 2'b00;
	g_busy = 1'b0; g_st = 4'd0; g_unit = 1'b0;
	g_spt[0] = 6'd9; g_spt[1] = 6'd9; g_sides[0] = 2'd2; g_sides[1] = 2'd2;
	g_tracks[0] = 8'd0; g_tracks[1] = 8'd0; m_total[0] = 17'd0; m_total[1] = 17'd0;
	TR = 8'h00; DR = 8'h00; head[0] = 7'd0; head[1] = 7'd0;
	wp_force[0] = 9'd0; wp_force[1] = 9'd0;
end

always @(posedge clk) begin
	h_done <= 1'b0;
	begin
		for (i = 0; i < 2; i = i + 1)
			if (img_mounted[i]) begin
				m_pend[i] <= 1'b1;
				m_ro[i] <= img_readonly;
				m_total[i] <= (img_size[63:26] != 38'd0) ? 17'h1FFFF : img_size[25:9];
				m_small[i] <= (img_size < 64'd512000);
				m_empty[i] <= (img_size[63:9] == 55'd0);
				inserted[i] <= 1'b0;        // ejected until the scan is done
				geo_valid[i] <= 1'b0;
			end

		// buffer grant (the mount scan uses the buffer too)
		if (!buf_req || reset) buf_gnt <= 1'b0;
		else if (!h_act && !g_busy) buf_gnt <= 1'b1;

		// boot sector shadow
		if (sb_we) begin
			case (sb_waddr)
				9'd19: sh19 <= sb_wdata;
				9'd20: sh20 <= sb_wdata;
				9'd24: sh24 <= sb_wdata;
				9'd25: sh25 <= sb_wdata;
				9'd26: sh26 <= sb_wdata;
				9'd27: sh27 <= sb_wdata;
				default: ;
			endcase
		end

		if (h_act) begin
			if (sd_ack[h_unit]) begin
				h_ack_seen <= 1'b1;
				sd_rd <= 2'b00;
				sd_wr <= 2'b00;
			end else if (h_ack_seen) begin
				h_act <= 1'b0;
				h_ack_seen <= 1'b0;
				if (h_scan) begin
					h_scan <= 1'b0;
					g_busy <= 1'b1;
					g_st <= 4'd0;
					g_unit <= h_unit;
				end else begin
					h_done <= 1'b1;
					// sector 0 written: recompute the geometry (Floppy_FindDiskDetails)
					if (h_wr && sd_lba == 16'd0) begin
						g_busy <= 1'b1;
						g_st <= 4'd0;
						g_unit <= h_unit;
					end
				end
			end
		end else if (!g_busy) begin
			if ((m_pend[0] || m_pend[1]) && !buf_req && !buf_gnt) begin
				// mount: latch, then read the boot sector
				g_unit <= m_pend[0] ? 1'b0 : 1'b1;
				m_pend[m_pend[0] ? 0 : 1] <= 1'b0;
				ro[m_pend[0] ? 0 : 1] <= m_ro[m_pend[0] ? 0 : 1];
				if (!m_empty[m_pend[0] ? 0 : 1]) begin
					h_act <= 1'b1;
					h_scan <= 1'b1;
					h_unit <= m_pend[0] ? 1'b0 : 1'b1;
					h_wr <= 1'b0;
					sd_lba <= 16'd0;
					sd_rd[m_pend[0] ? 0 : 1] <= 1'b1;
				end
			end else if (h_req && !h_done && buf_gnt) begin
				h_act <= 1'b1;
				h_scan <= 1'b0;
				h_unit <= h_req_unit;
				h_wr <= h_req_wr;
				sd_lba <= h_req_lba;
				if (h_req_wr) sd_wr[h_req_unit] <= 1'b1;
				else sd_rd[h_req_unit] <= 1'b1;
			end
		end

		// geometry (Floppy_FindDiskDetails / Floppy_DoubleCheckFormat)
		if (g_busy) begin
			case (g_st)
				4'd0: begin
					g_total <= m_total[g_unit];
					g_st <= 4'd1;
				end
				4'd1: begin
					gn_spt <= sh24[5:0];
					gn_sides <= sh26[1:0];
					if ({sh20, sh19} != g_total[15:0] || g_total[16] ||
					    {sh27, sh26} == 16'd0 || {sh27, sh26} > 16'd2 ||
					    {sh25, sh24} == 16'd0 || {sh25, sh24} > 16'd48) begin
						// double check: sides from the size, spt from the table
						gn_sides <= m_small[g_unit] ? 2'd1 : 2'd2;
						g_st <= 4'd2;
					end else
						g_st <= 4'd5;
				end
				4'd2: begin
					// TotalSectors == T * spt * sides for T = 80..84, spt = 9..12:
					// walk the 20 products with one adder and one comparator
					gn_spt <= 6'd0;
					gt_spt <= 4'd9;
					gt_t <= 3'd0;
					gt_step <= (gn_sides == 2'd2) ? 6'd18 : 6'd9;
					gt_prod <= (gn_sides == 2'd2) ? 17'd1440 : 17'd720;
					g_st <= 4'd7;
				end
				4'd7: begin
					if (g_total == gt_prod) gn_spt <= {2'b00, gt_spt};
					if (gt_t == 3'd4) begin
						if (gt_spt == 4'd12) g_st <= 4'd3;
						else begin
							gt_spt <= gt_spt + 4'd1;
							gt_t <= 3'd0;
							gt_step <= gt_step + ((gn_sides == 2'd2) ? 6'd2 : 6'd1);
							// 80 * (spt + 1) * sides
							gt_prod <= (gt_next << 6) + (gt_next << 4);
						end
					end else begin
						gt_t <= gt_t + 3'd1;
						gt_prod <= gt_prod + {11'd0, gt_step};
					end
				end
				4'd3: begin
					if (gn_spt != 6'd0) g_st <= 4'd5;
					else if ({sh25, sh24} >= 16'd5 && {sh25, sh24} <= 16'd48) begin
						gn_spt <= sh24[5:0];
						g_st <= 4'd5;
					end else begin
						// spt = TotalSectors / 80 / sides
						gd_den <= (gn_sides == 2'd2) ? 8'd160 : 8'd80;
						gd_q <= g_total;
						gd_r <= 17'd0;
						gd_cnt <= 5'd17;
						g_st <= 4'd4;
					end
				end
				4'd4: begin
					if (gd_cnt == 5'd0) begin
						gn_spt <= (gd_q > 17'd63) ? 6'd63 : gd_q[5:0];
						g_st <= 4'd5;
					end else begin
						if ({gd_r[15:0], gd_q[16]} >= {9'd0, gd_den}) begin
							gd_r <= {gd_r[15:0], gd_q[16]} - {9'd0, gd_den};
							gd_q <= {gd_q[15:0], 1'b1};
						end else begin
							gd_r <= {gd_r[15:0], gd_q[16]};
							gd_q <= {gd_q[15:0], 1'b0};
						end
						gd_cnt <= gd_cnt - 5'd1;
					end
				end
				4'd5: begin
					// tracks = TotalSectors / spt / sides
					if (gn_spt == 6'd0 || gn_sides == 2'd0) begin
						geo_valid[g_unit] <= 1'b0;
						inserted[g_unit] <= 1'b0;
						g_busy <= 1'b0;
					end else begin
						gd_den <= (gn_sides == 2'd2) ? {1'b0, gn_spt, 1'b0} : {2'b00, gn_spt};
						gd_q <= g_total;
						gd_r <= 17'd0;
						gd_cnt <= 5'd17;
						g_st <= 4'd6;
					end
				end
				4'd6: begin
					if (gd_cnt == 5'd0) begin
						g_spt[g_unit] <= gn_spt;
						g_sides[g_unit] <= gn_sides;
						g_tracks[g_unit] <= (gd_q > 17'd255) ? 8'd255 : gd_q[7:0];
						geo_valid[g_unit] <= (gd_q != 17'd0);
						inserted[g_unit] <= (gd_q != 17'd0);
						g_busy <= 1'b0;
					end else begin
						if ({gd_r[15:0], gd_q[16]} >= {9'd0, gd_den}) begin
							gd_r <= {gd_r[15:0], gd_q[16]} - {9'd0, gd_den};
							gd_q <= {gd_q[15:0], 1'b1};
						end else begin
							gd_r <= {gd_r[15:0], gd_q[16]};
							gd_q <= {gd_q[15:0], 1'b0};
						end
						gd_cnt <= gd_cnt - 5'd1;
					end
				end
				default: g_busy <= 1'b0;
			endcase
		end
	end
end

endmodule
