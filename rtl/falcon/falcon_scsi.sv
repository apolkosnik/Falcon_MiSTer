// falcon_scsi.sv - Atari Falcon030 SCSI: the NCR 5380 behind the ST DMA chip
// ($FF8604 with DMA mode bit 3 set, register = mode bits 2..0) and up to
// three SCSI targets on MiSTer hps_io disk image slots:
//   ID 0, ID 1 : hard disks (512 byte blocks)
//   ID 2       : CD-ROM (2048 byte blocks, read only)
//
// Behavioural reference: Hatari src/ncr5380.c (WinUAE derived):
//   ncr5380_bget / ncr5380_bput (all eight registers), ncr5380_reset (RST),
//   ncr5380_check_phase (phase mismatch interrupt), ncr5380_databusoutput,
//   raw_scsi_set_signal_phase (arbitration, selection with / without ATN),
//   raw_scsi_get_signal_phase, raw_scsi_set_ack / raw_scsi_write_data /
//   raw_scsi_get_data_2 (information transfer phases: message out, command,
//   data in / out, status, message in, bus free), getmsglen, scsicmdsizes,
//   dma_check / Ncr5380_DmaTransfer_Falcon (the DMA chip moves the data);
// src/hdc.c: HDC_WriteCommandPacket (LUN rules), HDC_Cmd_RequestSense
//   (sense formats), HDC_Cmd_Inquiry, HDC_Cmd_ReadCapacity, HDC_Cmd_Seek,
//   HDC_Cmd_ReadSector / WriteSector (range checks), sense codes of hdc.h.
// The command split follows the NeXT-Color core (ref/nextcolor rtl/tc_scsi.sv,
// docs/DECISIONS.md "SCSI and DMA" / "SCSI CD-ROM").
//
// 5380 register port (from falcon_fdc): acc = one clock register access
// (side effects), we, rs, wdata; rdata is combinational for rs (falcon_fdc
// samples it on bus_stb and issues acc on the next clock).  irq_set pulses
// when the 5380 interrupt goes active (Hatari ncr5380_set_irq ->
// FDC_SetIRQ(HDC)), irq_clr when register 7 is read (FDC_ClearIRQ).
//
// DMA: the 5380 DRQ/DACK handshake with the DMA chip's FIFO.  In DMA mode
// (MR bit 1) after a start-DMA write (5 = send, 6 = target receive treated as
// send as in Hatari, 7 = initiator receive) the 5380 requests a transfer
// whenever the target asserts REQ in the phase written to the TCR:
//   dma_push_req / dma_push_byte / dma_push_ack : target -> RAM (data in)
//   dma_pull_req / dma_pull_byte / dma_pull_ack : RAM -> target (data out)
// The req is a level, the ack a one clock pulse from falcon_fdc; the DMA chip
// services it only while it is set to HDC DMA ($FF8606 bits 7:6 = 00, as
// Hatari dma_check).  dma_flush pulses when the target leaves the data in
// phase so that the DMA chip writes a partly filled 16 byte FIFO half.
//
// Targets.  A target answers selection when its image is mounted; the CD-ROM
// (ID 2) answers with no medium too when CD_ALWAYS = 1 (NeXT-Color rule) and
// then reports NOT READY / medium not present to medium commands.
// Handled here: TEST UNIT READY, REZERO, REQUEST SENSE, FORMAT UNIT, READ(6),
// READ(10), WRITE(6), WRITE(10), SEEK(6), SEEK(10), RESERVE, RELEASE,
// START STOP UNIT, PREVENT ALLOW MEDIUM REMOVAL, VERIFY(10) (BYTCHK data is
// accepted and discarded), SYNCHRONIZE CACHE.
// Built by Main_MiSTer (support/falcon/falcon_scsi.cpp) and fetched as one
// 512 byte block READ of a window LBA on the target's own slot:
//   0x7E000000 | unit<<20 | flags<<16 | op<<8 | arg, bytes 510..511 =
//   response length (big endian), response at byte 0:
//   INQUIRY        op 12 flags[0] = EVPD, flags[3] = LUN != 0, arg = page
//   READ CAPACITY  op 25
//   MODE SENSE(6)  op 1A flags[0] = DBD, arg = PC | page code (CDB byte 2)
//   MODE SENSE(10) op 5A flags[0] = DBD, arg = CDB byte 2
//   READ TOC       op 43 flags[0] = MSF, flags[2:1] = format (MMC CDB byte 2,
//                  else byte 9 bits 7:6), arg = starting track (CD only)
//   READ SUB-CHAN. op 42 flags[0] = MSF, flags[1] = SubQ, arg = format
//                  (CD only)
// A response length of 0 or above 512 means "no answer" (a stock Main returns
// zeros for an LBA beyond the image): INQUIRY and READ CAPACITY then use the
// built-in responses below (capacity from img_size), the others report CHECK
// CONDITION, ILLEGAL REQUEST / invalid field in CDB.
// Forwarded to Main as one block WRITE of 0x7D000000 | unit<<20 | op<<8, the
// CDB at bytes 496..505 and the parameter list at byte 0: MODE SELECT(6/10)
// (any unit, parameter list up to 496 bytes) and, on the CD-ROM, PLAY AUDIO
// (10/12/MSF/TRACK INDEX), PAUSE/RESUME, STOP PLAY/SCAN.  The status is GOOD.
// Built-in INQUIRY: type 0 (disk) or 5 (CD-ROM, removable), version 2,
// response format 2, 36 bytes, vendor "MiSTer  ", product "Falcon HD       "
// or "Falcon CD-ROM   ", revision "1.0 ".
//
// Sector data paths: a 1024 byte buffer (one M10K) used as two 512 byte
// halves: the HPS fills one half while the initiator drains the other (and
// the other way round for writes).  A CD-ROM block is four 512 byte image
// blocks (LBA x 4 .. LBA x 4 + 3).
//
// hps_io ports in the style of falcon_ide: slot n = SCSI ID n; sd_rd[n],
// sd_wr[n], sd_ack[n], img_mounted[n]; sd_lba0..2 and sd_buff_din0..2 carry
// the same value; the shared sd_buff_* inputs are only used while one of our
// sd_ack bits is set; at most one request is pending.  The mounted media
// (img_mounted / img_size / img_readonly) are kept through `reset`: the
// 68030 RESET instruction resets the peripherals and the HPS does not
// announce the images again.  Everything else (5380, bus, targets' sense) is
// reset.  A transfer cut by reset is completed by the HPS and ignored.
//
// Deviations from Hatari (hardware / datasheet wins, or limits):
// - Hatari answers every phase at once; here the target drops REQ after each
//   ACK and raises it again only after ACK is released, and keeps REQ low
//   while it waits for the HPS (a real target's handshake).  REQ never comes
//   with ACK asserted, so Hatari's "REQ reads 0 while ACK" holds.
// - Bus status (reg 4): BSY is the wired OR of the target, ICR BSY and the
//   5380's own arbitration BSY; SEL is ICR SEL; MSG/C/D/I/O are 0 outside the
//   information phases.  Hatari shows only the target's BSY and never SEL.
//   Bus and status (reg 5): ATN and ACK show the ICR bits, phase match
//   compares the bus lines (all 0 when no target drives them) with the TCR,
//   busy error is latched on loss of BSY with MR bit 2 set and cleared by a
//   register 7 read (Hatari: live "bus free and MR bit 2"), DMA request is
//   the live request, end of DMA (bit 7) is never set (the Falcon DMA chip
//   gives no EOP; Hatari's Falcon path never sets it either).
// - Interrupts: phase mismatch is raised when REQ rises in a phase other than
//   the TCR phase while MR bit 1 (DMA mode) is set and MR bit 6 clear
//   (datasheet; Hatari tests the same condition on register accesses and
//   after every DMA block); loss of BSY with MR bit 2 set raises one too.
//   Hatari's unconditional interrupt after each DMA block is not done: when
//   the DMA sector count runs out before the target's data the transfer
//   simply stops, as on the hardware.
// - The target selection is evaluated continuously while SEL is asserted
//   and BSY released (Hatari only on ICR writes).
// - In PIO output phases a target accepts the bus as driven: ODR if ICR bit
//   0 is set, else 0 (Hatari ignores the ACK when the data bus is not
//   driven).  An extended message is 2 + its length byte long (Hatari uses
//   the length byte alone).  After a complete message the target stays in
//   message out while ATN is still asserted (SCSI-2; Hatari goes to the
//   command phase); message contents other than IDENTIFY are ignored (no
//   MESSAGE REJECT, no synchronous transfer).
// - Chip reset (reset input) clears every register without an interrupt
//   (datasheet); ICR RST behaves as Hatari's ncr5380_reset (registers
//   cleared, ICR = $80, interrupt) and also frees the bus.
// - REQUEST SENSE: no information field / valid bit (Hatari reports the last
//   LBA); sense is cleared after it is reported (SCSI).  The target mode of
//   the 5380, parity, reselection and unit attention are not implemented.
// - DMA "end address 16 bytes too high" (Hatari dma_check comment) is not
//   reproduced; the DMA chip writes the last partial 16 byte FIFO half as a
//   whole (address rounded up to 16).

module falcon_scsi #(
	parameter CLK_HZ    = 32000000,
	parameter CD_ALWAYS = 1           // ID 2 answers selection with no medium
) (
	input             clk,
	input             reset,

	// NCR 5380 register port (falcon_fdc)
	input             acc,
	input             we,
	input       [2:0] rs,
	input       [7:0] wdata,
	output reg  [7:0] rdata,
	output reg        irq_set,
	output reg        irq_clr,

	// DMA chip handshake
	output            dma_push_req,
	output      [7:0] dma_push_byte,
	input             dma_push_ack,
	output            dma_pull_req,
	input       [7:0] dma_pull_byte,
	input             dma_pull_ack,
	output reg        dma_flush,

	// hps_io disk images (slot n = SCSI ID n)
	input       [2:0] img_mounted,
	input             img_readonly,
	input      [63:0] img_size,
	output     [31:0] sd_lba0,
	output     [31:0] sd_lba1,
	output     [31:0] sd_lba2,
	output reg  [2:0] sd_rd,
	output reg  [2:0] sd_wr,
	output      [5:0] sd_blk_cnt,     // blocks - 1 of the pending HPS request (all slots)
	input       [2:0] sd_ack,
	input      [13:0] sd_buff_addr,
	input       [7:0] sd_buff_dout,
	output      [7:0] sd_buff_din0,
	output      [7:0] sd_buff_din1,
	output      [7:0] sd_buff_din2,
	input             sd_buff_wr,

	output            led
);

// ===========================================================================
// SCSI constants
localparam [2:0] PH_DO = 3'd0, PH_DI = 3'd1, PH_CMD = 3'd2, PH_ST = 3'd3,
                 PH_MO = 3'd6, PH_MI = 3'd7;
localparam [2:0] B_FREE = 3'd0, B_ARB = 3'd1, B_SEL1 = 3'd2, B_SEL2 = 3'd3, B_INFO = 3'd4;

localparam [3:0] T_IDLE = 4'd0, T_MO = 4'd1, T_CMD = 4'd2, T_EXEC = 4'd3, T_CHK = 4'd4,
                 T_DIN = 4'd5, T_DOUT = 4'd6, T_WAITW = 4'd7, T_WIN = 4'd8, T_FILL = 4'd9,
                 T_ST = 4'd10, T_MI = 4'd11;

localparam [1:0] K_RD = 2'd0, K_WR = 2'd1, K_VFY = 2'd2, K_NONE = 2'd3;
localparam [1:0] G_SENSE = 2'd0, G_INQ = 2'd1, G_CAP = 2'd2, G_NONE = 2'd3;

localparam [7:0] WIN_RESP = 8'h7E, WIN_CMD = 8'h7D;

// ===========================================================================
// 5380 registers
reg  [7:0] odr;              // output data register (Hatari data_write)
reg  [7:0] icr;              // bits 7, 4..0 as written (6/5 = AIP / LA below)
reg        aip;
reg  [7:0] mr, tcr, r7;      // (reg 4 write = select enable: no reselection, not stored)
reg        irq;
reg        berr;             // busy error (latched)
reg        dma_act, dma_send;

// SCSI bus / target
reg  [2:0] bst;              // bus state
reg  [2:0] ph;               // information transfer phase (bst == B_INFO)
reg        init_v;           // initiator ID known from arbitration
reg  [2:0] init_id;
reg        atn_l;
reg  [1:0] tid;              // selected target
reg        t_req;
reg  [7:0] t_stat;
reg  [3:0] st;
reg        prep;

// per target state; the mounted media are kept through `reset` (the
// 68030 RESET instruction resets the peripherals, the HPS does not announce
// the images again) and only change on img_mounted
reg  [2:0] mnt = 3'b000;     // image mounted
reg  [2:0] rov = 3'b000;     // read only
reg [31:0] cap0 = 32'd0, cap1 = 32'd0, cap2 = 32'd0;   // image size in 512 byte blocks
always @(posedge clk) begin
	if (img_mounted[0]) begin mnt[0] <= |img_size; rov[0] <= img_readonly; cap0 <= img_size[40:9]; end
	if (img_mounted[1]) begin mnt[1] <= |img_size; rov[1] <= img_readonly; cap1 <= img_size[40:9]; end
	if (img_mounted[2]) begin mnt[2] <= |img_size; rov[2] <= img_readonly; cap2 <= img_size[40:9]; end
end
reg  [3:0] skey [0:2];
reg  [7:0] sasc [0:2];

// command
reg  [7:0] c0, c1, c2, c3, c4, c5, c6, c7, c8, c9;
reg  [4:0] ccnt;
reg  [4:0] clen;
reg  [8:0] mcnt;
reg  [8:0] mlen;
reg        m_ext;
reg  [2:0] msglun;
reg        msglun_v;
reg        lun_bad_r;
reg  [1:0] kind;
reg [31:0] lba;
reg [15:0] cnt;
reg [31:0] capl;             // capacity of the selected unit in its own blocks
reg  [9:0] alloc_r;
reg  [1:0] gsel;
reg  [3:0] rs_key;
reg  [7:0] rs_asc;
reg        rs_short;

// data path
reg [13:0] pos;
reg [14:0] lim;
reg        bp, hp;           // bus side / HPS side buffer half
reg  [1:0] hv;               // half holds data (read) / is full (write)
reg [17:0] b_cnt, h_cnt;     // 512 byte chunks left (bus side / HPS side)
reg        dk_sect, dk_gen;
reg        h_wr, h_disc;
reg        h_act, h_seen, h_drop;
reg  [1:0] h_unit;
reg [31:0] h_lba;
reg  [7:0] wl_hi, wl_lo;
reg  [3:0] fcnt;
reg  [5:0] h_n;              // blocks in the pending / running HPS request

wire       is_cd = (tid == 2'd2);
wire [2:0] present = mnt | {CD_ALWAYS != 0, 2'b00};

assign sd_lba0 = h_lba;
assign sd_lba1 = h_lba;
assign sd_lba2 = h_lba;
assign led = h_act | (bst != B_FREE);
assign sd_blk_cnt = h_n - 6'd1;

// Sector data moves in buffer halves of up to 32 blocks (16 KB, the hps_io /
// Main limit), one HPS request per half: Main does one image write per 16 KB
// instead of one synchronous (O_SYNC) write per 512 bytes.  CD-ROM data stays
// one block per request (Main translates cue/bin images per 512 byte block).
function [5:0] chunk(input [17:0] n, input cd);
	chunk = (cd || n <= 18'd1) ? 6'd1 : (n >= 18'd32) ? 6'd32 : n[5:0];
endfunction

// ===========================================================================
// Buffer: 32768 x 8 (two 16 KB halves), simple dual port, registered read
reg  [7:0] buf_m [0:32767];
reg        bw_e;
reg [14:0] bw_a;
reg  [7:0] bw_d;
reg  [7:0] m_q;
wire       hw_e = h_act & ~h_wr & ~h_drop & sd_ack[h_unit] & sd_buff_wr;
wire       m_we = hw_e | bw_e;
wire [14:0] m_wa = hw_e ? {hp, sd_buff_addr} : bw_a;
wire [7:0] m_wd = hw_e ? sd_buff_dout : bw_d;
wire [14:0] m_ra = (h_act & h_wr) ? {hp, sd_buff_addr} : {bp, pos};
always @(posedge clk) begin
	if (m_we) buf_m[m_wa] <= m_wd;
	m_q <= buf_m[m_ra];
end
assign sd_buff_din0 = m_q;
assign sd_buff_din1 = m_q;
assign sd_buff_din2 = m_q;

// ===========================================================================
// Built-in responses (REQUEST SENSE, INQUIRY and READ CAPACITY fallbacks)
wire [31:0] capm1 = capl - 32'd1;
reg  [7:0] gen_b, gen_q;
always @* begin
	gen_b = 8'h00;
	case (gsel)
		G_SENSE: begin
			if (rs_short) gen_b = (pos[5:0] == 6'd0) ? rs_asc : 8'h00;    // old format: error code
			else case (pos[5:0])
				6'd0:  gen_b = 8'h70;
				6'd2:  gen_b = {4'h0, rs_key};
				6'd7:  gen_b = 8'd14;
				6'd12: gen_b = rs_asc;
				default: gen_b = 8'h00;
			endcase
		end
		G_INQ: case (pos[5:0])
			6'd0:  gen_b = lun_bad_r ? 8'h7F : (is_cd ? 8'h05 : 8'h00);
			6'd1:  gen_b = is_cd ? 8'h80 : 8'h00;
			6'd2:  gen_b = 8'h02;
			6'd3:  gen_b = 8'h02;
			6'd4:  gen_b = 8'd31;
			6'd8:  gen_b = "M";
			6'd9:  gen_b = "i";
			6'd10: gen_b = "S";
			6'd11: gen_b = "T";
			6'd12: gen_b = "e";
			6'd13: gen_b = "r";
			6'd14, 6'd15: gen_b = " ";
			6'd16: gen_b = "F";
			6'd17: gen_b = "a";
			6'd18: gen_b = "l";
			6'd19: gen_b = "c";
			6'd20: gen_b = "o";
			6'd21: gen_b = "n";
			6'd22: gen_b = " ";
			6'd23: gen_b = is_cd ? "C" : "H";
			6'd24: gen_b = "D";
			6'd25: gen_b = is_cd ? "-" : " ";
			6'd26: gen_b = is_cd ? "R" : " ";
			6'd27: gen_b = is_cd ? "O" : " ";
			6'd28: gen_b = is_cd ? "M" : " ";
			6'd29, 6'd30, 6'd31, 6'd35: gen_b = " ";
			6'd32: gen_b = "1";
			6'd33: gen_b = ".";
			6'd34: gen_b = "0";
			default: gen_b = 8'h00;
		endcase
		G_CAP: case (pos[2:0])
			3'd0: gen_b = capm1[31:24];
			3'd1: gen_b = capm1[23:16];
			3'd2: gen_b = capm1[15:8];
			3'd3: gen_b = capm1[7:0];
			3'd6: gen_b = is_cd ? 8'h08 : 8'h02;
			default: gen_b = 8'h00;
		endcase
		default: gen_b = 8'h00;
	endcase
end
always @(posedge clk) gen_q <= gen_b;

// ===========================================================================
// Bus signals
wire       info     = (bst == B_INFO);
wire [2:0] lines    = info ? ph : 3'b000;
wire       t_bsy    = (bst == B_SEL2) | info;
wire       bsy_line = t_bsy | icr[3] | (mr[0] & aip);
wire       pmatch   = (lines == tcr[2:0]);
wire [7:0] t_byte   = (ph == PH_ST) ? t_stat : (ph == PH_MI) ? 8'h00 : dk_gen ? gen_q : m_q;
wire       pio_ack  = icr[4] & ~mr[1];

assign dma_push_req  = t_req & ph[0] & mr[1] & dma_act & ~dma_send & pmatch;
assign dma_pull_req  = t_req & ~ph[0] & mr[1] & dma_act & dma_send & pmatch;
assign dma_push_byte = t_byte;

// pseudo DMA by the CPU (Hatari: reg 6 read / reg 0 write while DMA active)
wire pd_rd6 = acc & ~we & (rs == 3'd6) & mr[1] & dma_act;
wire pd_wr0 = acc & we & (rs == 3'd0) & mr[1] & dma_act & icr[0];
wire x_in   = t_req & ph[0] & (pio_ack | dma_push_ack | pd_rd6);
wire x_out  = t_req & ~ph[0] & (pio_ack | dma_pull_ack | pd_wr0);
wire [7:0] x_byte = pd_wr0 ? wdata : dma_pull_ack ? dma_pull_byte : (icr[0] ? odr : 8'h00);

// register read values (ncr5380_bget)
always @* begin
	case (rs)
		3'd0: rdata = (info & ph[0]) ? t_byte : ((icr[0] | bst == B_ARB) ? odr : 8'h00);
		3'd1: rdata = {icr[7], aip, 1'b0, icr[4:0]};
		3'd2: rdata = mr;
		3'd3: rdata = tcr;
		3'd4: rdata = {icr[7], bsy_line, t_req, lines, icr[2], 1'b0};
		3'd5: rdata = {1'b0, dma_push_req | dma_pull_req, 1'b0, irq, pmatch, berr, icr[1], pio_ack};
		3'd6: rdata = (info & ph[0]) ? t_byte : 8'h00;
		default: rdata = r7;
	endcase
end

// ===========================================================================
// Helpers
function integer popcnt(input [7:0] v);
	integer k;
	begin
		popcnt = 0;
		for (k = 0; k < 8; k = k + 1) popcnt = popcnt + v[k];
	end
endfunction

function [2:0] msb(input [7:0] v);
	integer k;
	begin
		msb = 3'd0;
		for (k = 0; k < 8; k = k + 1) if (v[k]) msb = k[2:0];
	end
endfunction

// scsicmdsizes[op >> 5]
function [4:0] cmdlen(input [2:0] g);
	case (g)
		3'd0, 3'd7: cmdlen = 5'd6;
		3'd1, 3'd2, 3'd6: cmdlen = 5'd10;
		3'd3, 3'd5: cmdlen = 5'd12;
		default: cmdlen = 5'd16;
	endcase
endfunction

// target selection (raw_scsi_set_signal_phase, SELECT_1): highest present ID
// on the bus other than the arbitrating initiator
wire [2:0] cand = odr[2:0] & present &
                  ~((init_v && init_id == 3'd0) ? 3'b001 : 3'b000) &
                  ~((init_v && init_id == 3'd1) ? 3'b010 : 3'b000) &
                  ~((init_v && init_id == 3'd2) ? 3'b100 : 3'b000);
wire [1:0] cand_id = cand[2] ? 2'd2 : cand[1] ? 2'd1 : 2'd0;

wire [31:0] cap_sel = (tid == 2'd0) ? cap0 : (tid == 2'd1) ? cap1 : cap2;
wire [2:0]  lun     = msglun_v ? msglun : c1[7:5];
wire [15:0] c78     = {c7, c8};
wire [9:0]  c78_c   = (c78 > 16'd512) ? 10'd512 : c78[9:0];
wire [15:0] wlen    = {wl_hi, wl_lo};
wire        wvalid  = (wlen != 16'd0) && (wlen <= 16'd512);
wire [32:0] lsum    = {1'b0, lba} + {17'd0, cnt};

// medium commands (NOT READY without a medium)
reg need_medium;
always @* begin
	case (c0)
		8'h00, 8'h08, 8'h28, 8'h0A, 8'h2A, 8'h2F, 8'h0B, 8'h2B, 8'h25, 8'h43, 8'h42,
		8'h45, 8'h47, 8'h48, 8'h4B, 8'h4E, 8'hA5: need_medium = 1'b1;
		default: need_medium = 1'b0;
	endcase
end

// ===========================================================================
// Main process
integer i;
reg  [1:0] hv_set, hv_clr;
reg        hv_rst;
reg        pm_d, bsy_d, di_d;
reg  [8:0] ml;
reg  [4:0] cl;
reg  [9:0] l10;
reg  [2:0] bst_n;
reg        rst_now;

task automatic status(input [7:0] s);
	begin
		t_stat <= s;
		ph <= PH_ST;
		st <= T_ST;
		t_req <= 1'b0;
		prep <= 1'b0;
	end
endtask

task automatic check(input [3:0] key, input [7:0] asc);
	begin
		skey[tid] <= key;
		sasc[tid] <= asc;
		status(8'h02);
	end
endtask

task automatic window(input [3:0] flags, input [7:0] op, input [7:0] arg, input [9:0] alloc, input [1:0] g);
	begin
		h_lba <= {WIN_RESP, 2'b00, tid, flags, op, arg};
		h_cnt <= 18'd1;
		h_wr <= 1'b0;
		alloc_r <= alloc;
		gsel <= g;
		wl_hi <= 8'h00;
		wl_lo <= 8'h00;
		st <= T_WIN;
	end
endtask

task automatic forward(input [9:0] plen);
	begin
		h_lba <= {WIN_CMD, 2'b00, tid, 4'h0, c0, 8'h00};
		h_cnt <= 18'd1;
		h_wr <= 1'b1;
		fcnt <= 4'd0;
		pos <= 10'd0;
		dk_sect <= 1'b0;
		if (plen == 10'd0)
			st <= T_FILL;
		else begin
			lim <= plen;
			ph <= PH_DO;
			st <= T_DOUT;
		end
	end
endtask

task automatic din_gen(input [1:0] g, input [9:0] l);
	begin
		gsel <= g;
		dk_gen <= 1'b1;
		dk_sect <= 1'b0;
		pos <= 10'd0;
		if (l == 10'd0)
			status(8'h00);
		else begin
			lim <= l;
			ph <= PH_DI;
			st <= T_DIN;
		end
	end
endtask

always @(posedge clk) begin
	irq_set <= 1'b0;
	irq_clr <= 1'b0;
	bw_e <= 1'b0;
	dma_flush <= 1'b0;

	if (reset) begin
		odr <= 8'h00; icr <= 8'h00; aip <= 1'b0; mr <= 8'h00; tcr <= 8'h00; r7 <= 8'h00;
		irq <= 1'b0; berr <= 1'b0; dma_act <= 1'b0; dma_send <= 1'b0;
		bst <= B_FREE; ph <= PH_DO; init_v <= 1'b0; init_id <= 3'd0; atn_l <= 1'b0; tid <= 2'd0;
		t_req <= 1'b0; t_stat <= 8'h00; st <= T_IDLE; prep <= 1'b0;
		for (i = 0; i < 3; i = i + 1) begin skey[i] <= 4'd0; sasc[i] <= 8'd0; end
		msglun_v <= 1'b0; msglun <= 3'd0;
		hv <= 2'b00; bp <= 1'b0; hp <= 1'b0; pos <= 10'd0; lim <= 10'd0;
		b_cnt <= 18'd0; h_cnt <= 18'd0; dk_sect <= 1'b0; dk_gen <= 1'b0; gsel <= G_NONE;
		h_wr <= 1'b0; h_disc <= 1'b0; h_act <= 1'b0; h_seen <= 1'b0; h_drop <= 1'b0; h_unit <= 2'd0;
		h_lba <= 32'd0; h_n <= 6'd1; sd_rd <= 3'b000; sd_wr <= 3'b000; wl_hi <= 8'h00; wl_lo <= 8'h00;
		pm_d <= 1'b0; bsy_d <= 1'b0; di_d <= 1'b0;
	end else begin
		hv_set = 2'b00;
		hv_clr = 2'b00;
		hv_rst = 1'b0;
		rst_now = 1'b0;

		// ------------------------------------------------------------------
		// HPS engine: one 512 byte block per request
		if (hw_e && sd_buff_addr == 14'd510) wl_hi <= sd_buff_dout;
		if (hw_e && sd_buff_addr == 14'd511) wl_lo <= sd_buff_dout;
		if (h_act) begin
			if (sd_ack[h_unit]) begin
				h_seen <= 1'b1;
				sd_rd <= 3'b000;
				sd_wr <= 3'b000;
			end else if (h_seen) begin
				h_act <= 1'b0;
				h_seen <= 1'b0;
				if (h_drop)
					h_drop <= 1'b0;
				else begin
					if (h_wr) hv_clr[hp] = 1'b1;
					else hv_set[hp] = 1'b1;
					hp <= ~hp;
					h_lba <= h_lba + {26'd0, h_n};
					h_cnt <= h_cnt - {12'd0, h_n};
				end
			end
		end else if (h_cnt != 18'd0 && !h_drop && sd_ack == 3'b000) begin
			// (no acknowledge of a transfer cut short by reset is still up)
			if (h_disc) begin
				// VERIFY with BYTCHK: the data is dropped
				if (hv[hp]) begin
					hv_clr[hp] = 1'b1;
					hp <= ~hp;
					h_cnt <= h_cnt - {12'd0, chunk(h_cnt, tid == 2'd2)};
				end
			end else if (h_wr ? hv[hp] : ~hv[hp]) begin
				h_act <= 1'b1;
				h_unit <= tid;
				h_n <= chunk(h_cnt, tid == 2'd2);
				if (h_wr) sd_wr[tid] <= 1'b1;
				else sd_rd[tid] <= 1'b1;
			end
		end

		// ------------------------------------------------------------------
		// bus: selection phases (evaluated continuously)
		case (bst)
			B_ARB: if (icr[3] && icr[2]) bst <= B_SEL1;
			B_SEL1: begin
				atn_l <= icr[1];
				if (!icr[3]) begin
					if (cand != 3'b000) begin
						tid <= cand_id;
						bst <= B_SEL2;
					end else if (!icr[2])
						bst <= B_FREE;
				end
			end
			B_SEL2: if (!icr[2]) begin
				bst <= B_INFO;
				msglun_v <= 1'b0;
				t_req <= 1'b0;
				prep <= 1'b0;
				mcnt <= 9'd0;
				ccnt <= 5'd0;
				if (atn_l) begin ph <= PH_MO; st <= T_MO; end
				else begin ph <= PH_CMD; st <= T_CMD; end
			end
			default: ;
		endcase

		// ------------------------------------------------------------------
		// target: information transfer phases
		if (bst == B_INFO) begin
			case (st)
				T_MO: begin
					if (x_out) begin
						t_req <= 1'b0;
						if (mcnt == 9'd0) begin
							// identify (Hatari: (msg & $A0) == $80)
							if (x_byte[7] && !x_byte[5]) begin
								msglun <= x_byte[2:0];
								msglun_v <= 1'b1;
							end
							// getmsglen: one byte, two byte ($20-$2F) or extended
							if (x_byte == 8'h00 || x_byte[7] || (x_byte[7:5] == 3'b000 && x_byte != 8'h01)) ml = 9'd1;
							else if (x_byte[7:4] == 4'h2) ml = 9'd2;
							else ml = 9'd3;
							m_ext <= (ml == 9'd3);
						end else if (mcnt == 9'd1 && m_ext)
							ml = {1'b0, x_byte} + 9'd2;
						else
							ml = mlen;
						mlen <= ml;
						mcnt <= mcnt + 9'd1;
						if (mcnt + 9'd1 >= ml) begin
							if (icr[1])
								mcnt <= 9'd0;           // ATN still asserted: another message
							else begin
								ph <= PH_CMD;
								st <= T_CMD;
								ccnt <= 5'd0;
							end
						end
					end else if (!t_req && !pio_ack)
						t_req <= 1'b1;
				end

				T_CMD: begin
					if (x_out) begin
						t_req <= 1'b0;
						case (ccnt)
							5'd0: c0 <= x_byte;
							5'd1: c1 <= x_byte;
							5'd2: c2 <= x_byte;
							5'd3: c3 <= x_byte;
							5'd4: c4 <= x_byte;
							5'd5: c5 <= x_byte;
							5'd6: c6 <= x_byte;
							5'd7: c7 <= x_byte;
							5'd8: c8 <= x_byte;
							5'd9: c9 <= x_byte;
							default: ;
						endcase
						cl = (ccnt == 5'd0) ? cmdlen(x_byte[7:5]) : clen;
						clen <= cl;
						ccnt <= ccnt + 5'd1;
						if (ccnt + 5'd1 == cl) st <= T_EXEC;
					end else if (!t_req && !pio_ack)
						t_req <= 1'b1;
				end

				// SCSI command decode (HDC_WriteCommandPacket / HDC_EmulateCommandPacket)
				T_EXEC: begin
					hv_rst = 1'b1;
					hp <= 1'b0;
					bp <= 1'b0;
					pos <= 10'd0;
					h_wr <= 1'b0;
					h_disc <= 1'b0;
					dk_gen <= 1'b0;
					dk_sect <= 1'b0;
					prep <= 1'b0;
					lun_bad_r <= (lun != 3'd0);
					capl <= is_cd ? {2'b00, cap_sel[31:2]} : cap_sel;
					kind <= K_NONE;
					if (c0 != 8'h03) begin
						skey[tid] <= 4'd0;
						sasc[tid] <= 8'h00;
					end
					if (c0 == 8'h03) begin
						// REQUEST SENSE (also for an invalid LUN, with that error)
						rs_key <= (lun != 3'd0) ? 4'h5 : skey[tid];
						rs_asc <= (lun != 3'd0) ? 8'h25 : sasc[tid];
						rs_short <= (c4 <= 8'd4);
						skey[tid] <= 4'd0;
						sasc[tid] <= 8'h00;
						din_gen(G_SENSE, (c4 == 8'd0) ? 10'd4 : (c4 > 8'd22) ? 10'd22 : {2'b00, c4});
					end else if (c0 == 8'h12) begin
						// INQUIRY (any LUN)
						window({lun != 3'd0, 2'b00, c1[0]}, 8'h12, c2, {2'b00, c4}, G_INQ);
					end else if (lun != 3'd0)
						check(4'h5, 8'h25);
					else if (need_medium && !mnt[tid])
						check(4'h2, 8'h3A);
					else case (c0)
						8'h00, 8'h01, 8'h04, 8'h16, 8'h17, 8'h1B, 8'h1E, 8'h35:
							status(8'h00);
						8'h08, 8'h0A: begin
							lba <= {11'd0, c1[4:0], c2, c3};
							cnt <= (c4 == 8'd0) ? 16'd256 : {8'd0, c4};
							if (c0 == 8'h0A && (rov[tid] || is_cd)) check(4'h7, 8'h27);
							else begin
								kind <= (c0 == 8'h08) ? K_RD : K_WR;
								st <= T_CHK;
							end
						end
						8'h28, 8'h2A, 8'h2F: begin
							lba <= {c2, c3, c4, c5};
							cnt <= c78;
							if (c0 == 8'h2A && (rov[tid] || is_cd)) check(4'h7, 8'h27);
							else begin
								kind <= (c0 == 8'h28) ? K_RD : (c0 == 8'h2A) ? K_WR : (c1[1] ? K_VFY : K_NONE);
								st <= T_CHK;
							end
						end
						8'h0B, 8'h2B: begin
							lba <= (c0 == 8'h0B) ? {11'd0, c1[4:0], c2, c3} : {c2, c3, c4, c5};
							cnt <= 16'd1;
							st <= T_CHK;
						end
						8'h25: window(4'h0, 8'h25, 8'h00, 10'd8, G_CAP);
						8'h1A: window({3'b000, c1[3]}, 8'h1A, c2, {2'b00, c4}, G_NONE);
						8'h5A: window({3'b000, c1[3]}, 8'h5A, c2, c78_c, G_NONE);
						8'h43: begin
							if (!is_cd) check(4'h5, 8'h20);
							else window({1'b0, (c2[2:0] != 3'd0) ? c2[1:0] : c9[7:6], c1[1]}, 8'h43, c6, c78_c, G_NONE);
						end
						8'h42: begin
							if (!is_cd) check(4'h5, 8'h20);
							else window({2'b00, c2[6], c1[1]}, 8'h42, c3, c78_c, G_NONE);
						end
						8'h15: forward({2'b00, c4});
						8'h55: forward((c78 > 16'd496) ? 10'd496 : c78[9:0]);
						8'h45, 8'h47, 8'h48, 8'h4B, 8'h4E, 8'hA5: begin
							if (!is_cd) check(4'h5, 8'h20);
							else forward(10'd0);
						end
						default: check(4'h5, 8'h20);
					endcase
				end

				// range check (HDC_Cmd_ReadSector / WriteSector / Seek)
				T_CHK: begin
					if (lsum > {1'b0, capl})
						check(4'h5, 8'h21);
					else if (cnt == 16'd0 || kind == K_NONE)
						status(8'h00);
					else begin
						h_cnt <= is_cd ? {cnt, 2'b00} : {2'b00, cnt};
						b_cnt <= is_cd ? {cnt, 2'b00} : {2'b00, cnt};
						h_lba <= is_cd ? {lba[29:0], 2'b00} : lba;
						lim <= {chunk(is_cd ? {cnt, 2'b00} : {2'b00, cnt}, is_cd), 9'd0};
						dk_sect <= 1'b1;
						if (kind == K_RD) begin
							h_wr <= 1'b0;
							ph <= PH_DI;
							st <= T_DIN;
						end else begin
							h_wr <= 1'b1;
							h_disc <= (kind == K_VFY);
							ph <= PH_DO;
							st <= T_DOUT;
						end
					end
				end

				// response window fetched: use it or fall back
				T_WIN: begin
					if (h_cnt == 18'd0 && !h_act) begin
						dk_gen <= 1'b0;
						dk_sect <= 1'b0;
						pos <= 10'd0;
						if (wvalid) begin
							l10 = (alloc_r < wlen[9:0]) ? alloc_r : wlen[9:0];
							if (l10 == 10'd0) status(8'h00);
							else begin
								lim <= l10;
								ph <= PH_DI;
								st <= T_DIN;
							end
						end else if (gsel == G_INQ)
							din_gen(G_INQ, (alloc_r < 10'd36) ? alloc_r : 10'd36);
						else if (gsel == G_CAP)
							din_gen(G_CAP, 10'd8);
						else
							check(4'h5, 8'h24);
					end
				end

				T_DIN: begin
					if (x_in) begin
						t_req <= 1'b0;
						if (pos == lim - 10'd1) begin
							pos <= 10'd0;
							if (dk_sect) begin
								hv_clr[bp] = 1'b1;
								bp <= ~bp;
								b_cnt <= b_cnt - {12'd0, lim[14:9]};
								lim <= {chunk(b_cnt - {12'd0, lim[14:9]}, is_cd), 9'd0};
								if (b_cnt == {12'd0, lim[14:9]}) status(8'h00);
							end else
								status(8'h00);
						end else
							pos <= pos + 10'd1;
					end else if (!t_req) begin
						// one clock for the buffer / generator output
						if (prep) begin
							prep <= 1'b0;
							t_req <= 1'b1;
						end else if ((!dk_sect || hv[bp]) && !pio_ack)
							prep <= 1'b1;
					end
				end

				T_DOUT: begin
					if (x_out) begin
						t_req <= 1'b0;
						bw_e <= 1'b1;
						bw_a <= {bp, pos};
						bw_d <= x_byte;
						if (pos == lim - 10'd1) begin
							pos <= 10'd0;
							if (dk_sect) begin
								hv_set[bp] = 1'b1;
								bp <= ~bp;
								b_cnt <= b_cnt - {12'd0, lim[14:9]};
								lim <= {chunk(b_cnt - {12'd0, lim[14:9]}, is_cd), 9'd0};
								if (b_cnt == {12'd0, lim[14:9]}) st <= T_WAITW;
							end else
								st <= T_FILL;
						end else
							pos <= pos + 10'd1;
					end else if (!t_req && (!dk_sect || !hv[bp]) && !pio_ack)
						t_req <= 1'b1;
				end

				// forwarded command: CDB into bytes 496..505, then the block write
				T_FILL: begin
					bw_e <= 1'b1;
					bw_a <= {bp, 5'b00000, 5'b11111, fcnt};   // 496 + fcnt
					case (fcnt)
						4'd0: bw_d <= c0;
						4'd1: bw_d <= c1;
						4'd2: bw_d <= c2;
						4'd3: bw_d <= c3;
						4'd4: bw_d <= c4;
						4'd5: bw_d <= c5;
						4'd6: bw_d <= c6;
						4'd7: bw_d <= c7;
						4'd8: bw_d <= c8;
						default: bw_d <= c9;
					endcase
					fcnt <= fcnt + 4'd1;
					if (fcnt == 4'd9) begin
						hv_set[bp] = 1'b1;
						st <= T_WAITW;
					end
				end

				T_WAITW: if (h_cnt == 18'd0 && !h_act) status(8'h00);

				T_ST: begin
					if (x_in) begin
						t_req <= 1'b0;
						ph <= PH_MI;
						st <= T_MI;
					end else if (!t_req && !pio_ack)
						t_req <= 1'b1;
				end

				T_MI: begin
					if (x_in) begin
						t_req <= 1'b0;
						bst <= B_FREE;
						st <= T_IDLE;
					end else if (!t_req && !pio_ack)
						t_req <= 1'b1;
				end

				default: ;
			endcase
		end

		// ------------------------------------------------------------------
		// register accesses (ncr5380_bget / ncr5380_bput)
		if (acc && !we) begin
			if (rs == 3'd7) begin
				irq <= 1'b0;
				berr <= 1'b0;
				irq_clr <= 1'b1;
			end
		end
		if (acc && we) begin
			case (rs)
				3'd0: odr <= wdata;
				3'd1: begin
					if (wdata[7]) begin
						// RST: ncr5380_reset
						rst_now = 1'b1;
					end else begin
						icr <= {1'b0, 2'b00, wdata[4:0]};
						bst_n = bst;
						if (bst != B_INFO && !icr[0] && wdata[0] && mr[0]) bst_n = B_SEL1;
						case (bst_n)
							B_FREE: begin
								if (wdata[3] && !wdata[2] && !wdata[0]) begin
									if (popcnt(odr) == 1) begin
										bst_n = B_ARB;
										init_v <= 1'b1;
										init_id <= msb(odr);
									end
								end else if (!wdata[3] && wdata[2]) begin
									if (!(popcnt(odr) > 2 || odr == 8'h00)) begin
										bst_n = B_SEL1;
										init_v <= 1'b0;
									end
								end
							end
							B_ARB: if (wdata[3] && wdata[2]) bst_n = B_SEL1;
							default: ;
						endcase
						if (bst_n != bst) bst <= bst_n;
					end
				end
				3'd2: begin
					mr <= wdata;
					if (wdata[0] && !mr[0]) begin
						// arbitrate: AIP set, LA cleared
						aip <= 1'b1;
						if (bst == B_FREE && popcnt(odr) == 1) begin
							bst <= B_ARB;
							init_v <= 1'b1;
							init_id <= msb(odr);
						end
					end else if (!wdata[0] && mr[0])
						aip <= 1'b0;
					if (!wdata[1]) begin
						dma_act <= 1'b0;
						dma_send <= 1'b0;
					end
				end
				3'd3: tcr <= wdata;
				3'd4: ;                       // select enable register
				3'd5, 3'd6: if (mr[1]) begin dma_act <= 1'b1; dma_send <= 1'b1; end
				default: begin
					r7 <= wdata;
					if (mr[1]) begin dma_act <= 1'b1; dma_send <= 1'b0; end
				end
			endcase
		end

		// ------------------------------------------------------------------
		// interrupts: phase mismatch in DMA mode, loss of BSY
		pm_d <= mr[1] & ~mr[6] & t_req & (ph != tcr[2:0]);
		bsy_d <= t_bsy;
		di_d <= info & (ph == PH_DI);
		if (di_d && !(info && ph == PH_DI)) dma_flush <= 1'b1;
		if ((mr[1] & ~mr[6] & t_req & (ph != tcr[2:0]) & ~pm_d) ||
		    (bsy_d & ~t_bsy & mr[2]) || rst_now) begin
			irq <= 1'b1;
			if (!irq) irq_set <= 1'b1;
		end
		if (bsy_d & ~t_bsy & mr[2]) berr <= 1'b1;

		// ------------------------------------------------------------------
		// ICR RST: registers cleared, ICR = $80, bus free, target aborted
		if (rst_now) begin
			odr <= 8'h00; icr <= 8'h80; aip <= 1'b0; mr <= 8'h00; tcr <= 8'h00; r7 <= 8'h00;
			berr <= 1'b0; dma_act <= 1'b0; dma_send <= 1'b0;
			bst <= B_FREE;
			init_v <= 1'b0;
			t_req <= 1'b0;
			prep <= 1'b0;
			st <= T_IDLE;
			h_cnt <= 18'd0;
			if (h_act) h_drop <= 1'b1;
			hv <= 2'b00;
		end else if (hv_rst)
			hv <= 2'b00;
		else
			hv <= (hv | hv_set) & ~hv_clr;
	end
end

endmodule
