// falcon_fpu_bridge.sv - MC68882 coprocessor interface for an FPU on the HPS
//
// The 68882's protocol is answered here; its registers and arithmetic live
// in a process on the ARM (tools/falcon_fpu, Hatari's 68882 emulation),
// reached through a DDR3 mailbox on falcon_memarb's d3 port.  Design:
// docs/FPU_ARM.md.  References: MC68881/MC68882 User's Manual (UM) ch. 6-7,
// MC68030 UM ch. 10 as implemented by the AP68030 (core/ap030_exec_c.vh).
//
// Presence: the service writes MAGIC, VERSION (2) and increments HEARTBEAT
// about every 10 ms.  Polled every 5 ms; the FPU exists while HEARTBEAT has
// changed within the last 100 ms.  Without it the first CIR access of an
// instruction (command, condition or restore write, save read) ends in BERR
// and the 68030 takes the F-line exception: no FPU.  BERR is never given
// later in an instruction (after the first access a BERR is a bus error).
// Only CpID 1 is the FPU; other CpIDs: BERR.
//
// Dialogs (response word: CA 15, PC 14, DR 13, primitive 12..8, parameter;
// UM table 7-7).  The CA=1 forms are used throughout (the 68881 dialog,
// legal for the 68882, UM 7.5.1); PC is never requested (FPIAR is not
// loaded from the instruction address yet: milestone 4).
//   cpGEN opclass 000 reg-to-reg, FMOVECR: the request is posted, the CPU is
//     released at once ($0900) and the ARM executes in the background.
//   opclass 010 <ea>,FPn: evaluate EA and transfer data, DR=0: B $9501,
//     W $9502, L/S $9504, D $9608, X/P $960C; the operand goes to the
//     request; final read: post, $0900.
//   opclass 011 FPn,<ea>: packed with a dynamic k-factor first asks for Dn
//     ($8C0n); the request is posted and the response is come-again ($8900)
//     until the reply, then DR=1: B $B101, W $B102, L/S $B104, D $B208,
//     X/P $B20C; the CPU reads the result; final read $0802.
//   opclass 100 <ea>,FPcr: FPCR or FPSR alone $9504, FPIAR alone $9704 (An
//     allowed), two $9608, three $960C; final read: post, $0802.
//   opclass 101 FPcr,<ea>: come-again until the reply, then $B104, $B304
//     (FPIAR alone), $B208, $B20C; final read $0802.
//   opclass 110/111 FMOVEM: a dynamic list first asks for Dn ($8C0n); 110
//     (to the FPU) transfer multiple coprocessor registers $810C, register
//     select read (mask in [15:8]), 12 bytes per register, final read: post,
//     $0802; 111 posts first, come-again until the reply, then $A10C,
//     register select, the registers, final read $0802.  In predecrement
//     mode the AP68030 stores the first register at the highest address
//     (core/ap030_exec_c.vh S_CPMULT3): the reply holds the memory image in
//     ascending order, so its 12-byte blocks are sent last block first.
//   opclass 001 and an empty control register list: F-line ($1C0B).
//   Opmodes (cmd[6:0]) of opclasses 000/010: as Hatari's fpp.c models the
//     6888x (fault_if_nonexisting_opmode, after WinUAE's tests of the real
//     chips) most undocumented opmodes execute as aliases; $42, $43,
//     $46-$57, $59, $5B, $5D, $5F, $61, $65, $69-$6B, $6D-$77 are refused
//     with F-line ($1C0B) and $78-$7F with illegal instruction (vector 4,
//     $1C04), at the instruction, before any operand (OP_FLINE).
//   A reply flagged "not implemented" for an instruction the CPU waits on:
//     F-line.
// Busy: a command or condition word written while a request is outstanding
// is answered with come-again ($8900) until the reply (UM 7.2.6/7.2.7);
// FSAVE while busy reads the come-again format $0118 (UM 6.4.3).  If the
// service stops answering (no reply within ~100 ms, or presence lost) the
// waiting instruction ends with the mid-instruction exception primitive,
// coprocessor protocol violation ($1D0D), instead of hanging the 68030.
//
// Null/idle state (UM 6.4.2): null after reset and after FRESTORE of a null
// frame (both also reset the FPU on the ARM: a reset request); idle after
// any command or condition word.  FSAVE: null -> $0000; idle -> $1F38 and
// 14 operand longs (no state in the body yet).  FRESTORE: $00xx resets,
// $1F38 restores idle (14 longs taken), any other format reads back $02xx
// (format error).
//
// Conditionals: the predicate is evaluated from the FPSR condition codes of
// the last reply (N, Z, I, NAN) with Hatari's 6888x table (COND_TAB) once no
// request is outstanding.  An
// IEEE-nonaware predicate ($10-$1F) with NAN set also sets BSUN and the
// accrued IOP in the FPSR, and raises the BSUN exception if it is enabled
// (Hatari fpp_cond/fpsr_set_bsun): that case goes to the ARM as a condition
// request (KIND 3, the predicate in CMD); the reply's FLAGS bit 2 is the
// result, bit 1 the BSUN exception (pre-instruction exception primitive,
// vector 48, $1C30).
//
// Mailbox (guest $E90000, DDR3 0x30E90000; 16-bit words big endian, bytes
// in memory order: DDR3 byte k = guest byte k):
//   +$000 MAGIC $4650  +$002 HEARTBEAT  +$004 VERSION (2)   ARM -> FPGA
//   request, RSEQ written last (FPGA -> ARM):
//   +$100 RSEQ  +$102 KIND (1 execute, 2 reset, 3 condition)  +$104 CMD  +$106 AUX (Dn
//   for a dynamic list or k-factor)  +$108 NBYTES  +$110.. operand bytes
//   reply, ASEQ written last (ARM -> FPGA):
//   +$200 ASEQ  +$202 FLAGS (bit 0 not implemented, 1 exception, 2 condition
//   true)  +$204 FPSR[31:16]
//   +$206 FPSR[15:0]  +$208 FPCR[15:0]  +$20A NBYTES  +$210.. result bytes
// Operand and result words move between the operand CIR and the mailbox
// directly (a CIR access waits for its one or two 16-bit DDR3 accesses).
module falcon_fpu_bridge #(
    parameter int CLK_HZ = 32000000
) (
    input             clk,
    input             reset,        // FPU reset: machine reset or RESET instruction
    input             por,          // power on

    // coprocessor interface registers (falcon_cpubus)
    input             cp_req,
    input             cp_we,
    input       [2:0] cp_id,
    input       [4:0] cp_off,
    input       [1:0] cp_siz,
    input      [31:0] cp_wdata,
    output reg        cp_ack,
    output reg        cp_berr,
    output reg [31:0] cp_rdata,

    // HPS mailbox (falcon_memarb DMA port)
    output reg        dma_req = 1'b0,
    output reg        dma_we,
    output reg [23:1] dma_addr,
    output reg  [1:0] dma_be,
    output reg [15:0] dma_wdata,
    input      [15:0] dma_rdata,
    input             dma_ack,

    output            present
);

localparam [4:0] CIR_RESP = 5'h00, CIR_CTRL = 5'h02, CIR_SAVE = 5'h04, CIR_REST = 5'h06,
                 CIR_CMD  = 5'h0A, CIR_COND = 5'h0E, CIR_OPND = 5'h10, CIR_RSEL = 5'h14;

localparam [15:0] R_IDLE  = 16'h0802;   // null, PF: idle
localparam [15:0] R_REL   = 16'h0900;   // null, IA: released while executing
localparam [15:0] R_CA    = 16'h8900;   // null, CA, IA: come again
localparam [15:0] R_FLINE = 16'h1C0B;   // take pre-instruction exception, vector 11
localparam [15:0] R_BSUN  = 16'h1C30;   // take pre-instruction exception, vector 48
localparam [15:0] R_PROTO = 16'h1D0D;   // take mid-instruction exception, vector 13
localparam [15:0] R_ILL   = 16'h1C04;   // take pre-instruction exception, vector 4
localparam [15:0] R_MIN   = 16'h810C;   // transfer multiple coprocessor registers, to the FPU
localparam [15:0] R_MOUT  = 16'hA10C;   // ... from the FPU

localparam [15:0] F_IDLE  = 16'h1F38;   // 68882 idle frame: version $1F, 56 bytes
localparam [15:0] F_CA    = 16'h0118;   // FSAVE while busy: come again
localparam [3:0]  FRAME_LONGS = 4'd14;

localparam [23:0] MB      = 24'hE90000;
localparam [23:0] A_MAGIC = MB + 24'h000;
localparam [23:0] A_HB    = MB + 24'h002;
localparam [23:0] A_VER   = MB + 24'h004;
localparam [23:0] A_RSEQ  = MB + 24'h100;
localparam [23:0] A_KIND  = MB + 24'h102;
localparam [23:0] A_CMD   = MB + 24'h104;
localparam [23:0] A_AUX   = MB + 24'h106;
localparam [23:0] A_RNB   = MB + 24'h108;
localparam [23:0] A_RDATA = MB + 24'h110;
localparam [23:0] A_ASEQ  = MB + 24'h200;
localparam [23:0] A_FLAGS = MB + 24'h202;
localparam [23:0] A_FPSRH = MB + 24'h204;
localparam [23:0] A_ADATA = MB + 24'h210;
localparam [15:0] MAGIC   = 16'h4650;
localparam [15:0] VERSION = 16'd2;

localparam int POLL_CLKS   = CLK_HZ / 200;          // presence poll, 5 ms
localparam int ALIVE_POLLS = 20;                    // 100 ms without a heartbeat
localparam int WD_CLKS     = CLK_HZ / 10;           // reply watchdog, 100 ms
localparam int PW          = $clog2(POLL_CLKS + 1);
localparam int WW          = $clog2(WD_CLKS + 1);
localparam [5:0] GAP_WAIT  = 6'd4;                  // ASEQ poll gap, CPU waiting
localparam [5:0] GAP_BG    = 6'd32;                 // ... CPU running (leave it the bus)

// dialog states
localparam [3:0] D_IDLE = 4'd0, D_CMD = 4'd1, D_COND = 4'd2, D_DN = 4'd3, D_IN = 4'd4,
                 D_RSEL = 4'd5, D_WAIT = 4'd6, D_OUT = 4'd7, D_PRIM = 4'd8;
// controller states
localparam [4:0] T_IDLE  = 5'd0,  T_CIR   = 5'd1,  T_DMA   = 5'd2,
                 T_OST1  = 5'd3,  T_OSTD  = 5'd4,  T_OLD1  = 5'd5,  T_OLDD  = 5'd6,
                 T_PKIND = 5'd7,  T_PCMD  = 5'd8,  T_PAUX  = 5'd9,  T_PNB   = 5'd10,
                 T_PSEQ  = 5'd11, T_PDONE = 5'd12,
                 T_QSEQ  = 5'd13, T_QFLG  = 5'd14, T_QFPSR = 5'd15,
                 T_QDONE = 5'd18,
                 T_SMAG  = 5'd19, T_SVER  = 5'd20, T_SHB   = 5'd21;

// ---------------------------------------------------------------------------
// Conditional predicates: Hatari's condition_table_6888x (fpp.c), indexed by
// the FPSR condition codes {N, Z, I, NAN} and the predicate's low four bits
// ($10-$1F repeat $00-$0F).  It is the UM's conditional test table for every
// code combination arithmetic produces, plus the 6888x results for the
// others (e.g. Z and NAN together, only from a move to the FPSR).
// Generated from fpp.c; bit (cc * 16 + predicate).
// ---------------------------------------------------------------------------
localparam [255:0] COND_TAB = 256'hffaaaaaaffaaaaaaff00f0f0ff00f0f0ffaaaaaaffaaaaaaff00ccccff00cccc;

// Opmodes a 6888x refuses (Hatari fpp.c fault_if_nonexisting_opmode, with
// fpu_no_unimplemented false): F-line for these, illegal instruction for
// $78-$7F; every other opmode executes (documented, or as an alias).
localparam [127:0] OP_FLINE = 128'h00ffee22aaffffcc0000000000000000;

// operand size of a data format (UM table 2-?): L S X P W D B (P with a
// dynamic k-factor, out only, as 7)
function automatic [7:0] fmt_size(input [2:0] f);
    case (f)
        3'd0, 3'd1: fmt_size = 8'd4;
        3'd2, 3'd3, 3'd7: fmt_size = 8'd12;
        3'd4: fmt_size = 8'd2;
        3'd5: fmt_size = 8'd8;
        default: fmt_size = 8'd1;
    endcase
endfunction

function automatic [3:0] popcount8(input [7:0] m);
    integer i;
    popcount8 = 4'd0;
    for (i = 0; i < 8; i = i + 1) popcount8 = popcount8 + {3'd0, m[i]};
endfunction

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------
reg  [4:0]    st = T_IDLE, nxt;
reg  [15:0]   rd_q;                 // data of the last DMA read

// presence
reg  [PW-1:0] ptmr = '0;
reg  [4:0]    alive = 5'd0;
reg  [15:0]   last_hb = 16'd0;
assign present = alive != 5'd0;

// latched CIR access
reg           cr_pend = 1'b0;
reg           cr_we;
reg  [2:0]    cr_id;
reg  [4:0]    cr_off;
reg  [1:0]    cr_siz;
reg  [31:0]   cr_wdata;

// FPU and dialog state
reg           st_null;
reg  [3:0]    dst;
reg  [15:0]   resp;
reg  [15:0]   cmd;
reg  [15:0]   cond;
reg  [15:0]   aux;
reg           phase2;               // the Dn of a dynamic list/k-factor is in
reg  [3:0]    fcc;                  // FPSR condition codes {N, Z, I, NAN}
reg  [3:0]    frm_cnt;
reg           frm_save;
reg  [15:0]   rest_fmt;

// transfers
reg  [7:0]    in_len, in_pos;
reg  [7:0]    blk_size, out_off;
reg  [3:0]    out_blk, nblk;
reg           out_pd;               // FMOVEM predecrement: blocks last first
reg  [15:0]   ld_hi;                // first word of a long result
reg  [2:0]    xn;                   // bytes in this operand access

// requests and replies
reg           busy = 1'b0;          // a request is outstanding
reg           post_req = 1'b0;      // post the request built in the mailbox
reg  [1:0]    post_kind;            // 1 execute, 2 reset, 3 condition
reg           rst_req = 1'b0;       // the ARM FPU has to be reset
reg  [15:0]   rseq = 16'd0;
reg           r_unimpl;
reg           r_exc;                // reply: exception (BSUN of a condition request)
reg           r_tf;                 // reply: condition result
reg           cond_arm;             // the condition went to the ARM
reg           dead;                 // the watchdog ended the last request
reg  [WW-1:0] wd;
reg  [5:0]    gap;

wire        cr_fpu   = cr_id == 3'd1;
wire [15:0] cr_word  = cr_off[1] ? cr_wdata[15:0] : cr_wdata[31:16];
wire        cr_first = (cr_we && (cr_off == CIR_CMD || cr_off == CIR_COND || cr_off == CIR_REST)) ||
                       (!cr_we && cr_off == CIR_SAVE);

wire [2:0]  opclass  = cmd[15:13];
wire [2:0]  fmt      = cmd[12:10];
wire        dyn_list = cmd[11];
wire        mm_pd    = !cmd[12];
wire [2:0]  dn_reg   = cmd[6:4];
wire [7:0]  mask     = dyn_list ? aux[7:0] : cmd[7:0];
wire [3:0]  ncr      = {3'd0, cmd[12]} + {3'd0, cmd[11]} + {3'd0, cmd[10]};
wire [7:0]  fsz      = fmt_size(fmt);
wire [3:0]  nmask    = popcount8(mask);
wire [7:0]  mm_len   = {nmask, 3'd0} + {1'b0, nmask, 2'd0};    // 12 per register

wire        tf       = COND_TAB[{fcc, cond[3:0]}];
wire        cond_nan = cond[4] && fcc[0];       // IEEE-nonaware with NAN: BSUN
wire [6:0]  opmode   = cmd[6:0];
wire        op_fline = OP_FLINE[opmode];
wire        op_ill   = opmode >= 7'h78;

// offset in the reply data of the current output position
wire [7:0]  oblk     = {4'd0, out_pd ? (nblk - 4'd1 - out_blk) : out_blk};
wire [7:0]  obase    = blk_size == 8'd12 ? ({oblk[4:0], 3'd0} + {oblk[5:0], 2'd0}) : 8'd0;
wire [7:0]  oofs     = obase + out_off;

// bytes of an operand CIR access (SIZ: 01 byte, 10 word, 11 three, 00 long)
wire [2:0]  acc_n    = (cr_siz == 2'b01) ? 3'd1 : (cr_siz == 2'b10) ? 3'd2 :
                       (cr_siz == 2'b11) ? 3'd3 : 3'd4;

// start one DMA word access; the controller continues in state n afterwards
task automatic dma(input we, input [23:0] a, input [15:0] d, input [1:0] be, input [4:0] n);
    dma_req   <= 1'b1;
    dma_we    <= we;
    dma_addr  <= a[23:1];
    dma_wdata <= d;
    dma_be    <= be;
    nxt       <= n;
    st        <= T_DMA;
endtask

task automatic ack(input [31:0] d);
    cp_ack   <= 1'b1;
    cp_rdata <= d;
    cr_pend  <= 1'b0;
    st       <= T_IDLE;
endtask

// the start of a command, once no request is outstanding: the first
// response of its dialog
task automatic start_cmd;
    case (opclass)
        3'b000:                                          // reg-to-reg
            if (op_fline || op_ill) begin
                resp <= op_ill ? R_ILL : R_FLINE; dst <= D_IDLE;
            end else begin
                in_len <= 8'd0; post_req <= 1'b1; post_kind <= 2'd1;
                resp <= R_REL; dst <= D_IDLE;
            end
        3'b010:
            if (fmt == 3'b111) begin                     // FMOVECR
                in_len <= 8'd0; post_req <= 1'b1; post_kind <= 2'd1;
                resp <= R_REL; dst <= D_IDLE;
            end else if (op_fline || op_ill) begin
                resp <= op_ill ? R_ILL : R_FLINE; dst <= D_IDLE;
            end else begin
                in_len <= fsz; in_pos <= 8'd0;
                resp <= {fsz > 8'd4 ? 8'h96 : 8'h95, fsz}; dst <= D_IN;
            end
        3'b011:
            if (fmt == 3'b111 && !phase2) begin          // packed, dynamic k
                resp <= {13'h1180, dn_reg}; dst <= D_DN;
            end else begin
                in_len <= 8'd0; post_req <= 1'b1; post_kind <= 2'd1;
                resp <= R_CA; dst <= D_WAIT;
            end
        3'b100:
            if (ncr == 4'd0) begin resp <= R_FLINE; dst <= D_IDLE; end
            else begin
                in_len <= {2'd0, ncr, 2'd0}; in_pos <= 8'd0;
                resp <= (ncr == 4'd1) ? (cmd[10] ? 16'h9704 : 16'h9504) :
                        (ncr == 4'd2) ? 16'h9608 : 16'h960C;
                dst <= D_IN;
            end
        3'b101:
            if (ncr == 4'd0) begin resp <= R_FLINE; dst <= D_IDLE; end
            else begin
                in_len <= 8'd0; post_req <= 1'b1; post_kind <= 2'd1;
                resp <= R_CA; dst <= D_WAIT;
            end
        3'b110:
            if (dyn_list && !phase2) begin resp <= {13'h1180, dn_reg}; dst <= D_DN; end
            else begin resp <= R_MIN; dst <= D_RSEL; end
        3'b111:
            if (dyn_list && !phase2) begin resp <= {13'h1180, dn_reg}; dst <= D_DN; end
            else begin
                in_len <= 8'd0; post_req <= 1'b1; post_kind <= 2'd1;
                resp <= R_CA; dst <= D_WAIT;
            end
        default: begin resp <= R_FLINE; dst <= D_IDLE; end   // opclass 001
    endcase
endtask

// the data primitive of an output dialog, once its reply is in
task automatic out_start;
    out_off <= 8'd0;
    out_blk <= 4'd0;
    case (opclass)
        3'b011: begin
            blk_size <= fsz; nblk <= 4'd1; out_pd <= 1'b0;
            resp <= {fsz > 8'd4 ? 8'hB2 : 8'hB1, fsz}; dst <= D_PRIM;
        end
        3'b101: begin
            blk_size <= {2'd0, ncr, 2'd0}; nblk <= 4'd1; out_pd <= 1'b0;
            resp <= (ncr == 4'd1) ? (cmd[10] ? 16'hB304 : 16'hB104) :
                    (ncr == 4'd2) ? 16'hB208 : 16'hB20C;
            dst <= D_PRIM;
        end
        default: begin                                  // 111 FMOVEM out
            blk_size <= 8'd12; nblk <= nmask; out_pd <= mm_pd;
            resp <= R_MOUT; dst <= D_RSEL;
        end
    endcase
endtask

// ---------------------------------------------------------------------------
// Controller
// ---------------------------------------------------------------------------
always @(posedge clk) begin
    cp_ack  <= 1'b0;
    cp_berr <= 1'b0;
    if (dma_ack) dma_req <= 1'b0;

    if (cp_req) begin
        cr_pend  <= 1'b1;
        cr_we    <= cp_we;
        cr_id    <= cp_id;
        cr_off   <= cp_off;
        cr_siz   <= cp_siz;
        cr_wdata <= cp_wdata;
    end

    if (ptmr != PW'(POLL_CLKS)) ptmr <= ptmr + 1'd1;
    if (gap != 6'd0) gap <= gap - 1'd1;

    // reply watchdog
    if (busy) begin
        if (wd != WW'(WD_CLKS)) wd <= wd + 1'd1;
        if (wd == WW'(WD_CLKS) || !present) begin
            busy <= 1'b0;
            dead <= 1'b1;
        end
    end else
        wd <= '0;

    case (st)
        T_IDLE:
            if (cr_pend && !cp_req)                     st <= T_CIR;
            else if (post_req && present)               st <= T_PKIND;
            else if (rst_req && !busy && present && !reset) begin
                post_req  <= 1'b1;
                post_kind <= 2'd2;
                in_len    <= 8'd0;
                rst_req   <= 1'b0;
                st        <= T_PKIND;
            end
            else if (busy && gap == 6'd0)               dma(1'b0, A_ASEQ, 16'd0, 2'b11, T_QSEQ);
            else if (ptmr == PW'(POLL_CLKS)) begin
                ptmr <= '0;
                dma(1'b0, A_MAGIC, 16'd0, 2'b11, T_SMAG);
            end

        T_DMA:
            if (dma_ack) begin
                rd_q <= dma_rdata;
                st   <= nxt;
            end

        // ---- one CIR access ----
        T_CIR:
            if (!cr_fpu || (cr_first && !present)) begin
                cp_ack  <= 1'b1;
                cp_berr <= 1'b1;
                cr_pend <= 1'b0;
                st      <= T_IDLE;
            end else if (cr_we) begin
                case (cr_off)
                    CIR_CMD: begin
                        st_null <= 1'b0;
                        cmd     <= cr_word;
                        phase2  <= 1'b0;
                        dead    <= 1'b0;
                        dst     <= D_CMD;
                        ack(32'hFFFF_FFFF);
                    end
                    CIR_COND: begin
                        st_null  <= 1'b0;
                        cond     <= cr_word;
                        cond_arm <= 1'b0;
                        dead     <= 1'b0;
                        dst     <= D_COND;
                        ack(32'hFFFF_FFFF);
                    end
                    CIR_CTRL: begin                     // abort / exception acknowledge
                        resp    <= R_IDLE;
                        dst     <= D_IDLE;
                        frm_cnt <= 4'd0;
                        ack(32'hFFFF_FFFF);
                    end
                    CIR_REST: begin
                        frm_cnt <= 4'd0;
                        dst     <= D_IDLE;
                        resp    <= R_IDLE;
                        if (cr_word[15:8] == 8'h00) begin   // null frame: reset
                            st_null  <= 1'b1;
                            fcc      <= 4'd0;
                            rst_req  <= 1'b1;
                            post_req <= 1'b0;
                            rest_fmt <= cr_word;
                        end else if (cr_word == F_IDLE) begin
                            st_null  <= 1'b0;
                            rest_fmt <= cr_word;
                            frm_cnt  <= FRAME_LONGS;
                            frm_save <= 1'b0;
                        end else
                            rest_fmt <= {8'h02, cr_word[7:0]};
                        ack(32'hFFFF_FFFF);
                    end
                    CIR_OPND:
                        if (dst == D_DN) begin              // Dn of a dynamic list/k
                            aux    <= cr_wdata[15:0];
                            phase2 <= 1'b1;
                            dst    <= D_CMD;
                            resp   <= R_CA;
                            ack(32'hFFFF_FFFF);
                        end else if (dst == D_IN && in_pos < in_len) begin
                            xn <= acc_n;
                            if (acc_n == 3'd1)
                                dma(1'b1, A_RDATA + {16'd0, in_pos}, {2{cr_wdata[31:24]}},
                                    in_pos[0] ? 2'b01 : 2'b10, T_OSTD);
                            else
                                dma(1'b1, A_RDATA + {16'd0, in_pos}, cr_wdata[31:16], 2'b11,
                                    acc_n == 3'd2 ? T_OSTD : T_OST1);
                        end else begin
                            if (frm_cnt != 4'd0 && !frm_save) frm_cnt <= frm_cnt - 1'd1;
                            ack(32'hFFFF_FFFF);
                        end
                    default: ack(32'hFFFF_FFFF);        // instruction address etc.
                endcase
            end else begin
                case (cr_off)
                    CIR_RESP:
                        case (dst)
                            D_CMD:
                                if (busy || post_req) ack({R_CA, R_CA});
                                else if (dead) begin
                                    dead <= 1'b0; dst <= D_IDLE; resp <= R_IDLE;
                                    ack({R_PROTO, R_PROTO});
                                end else begin
                                    start_cmd;
                                    st <= T_CIR;            // answer with the new response
                                end
                            D_COND:
                                if (busy || post_req) ack({R_CA, R_CA});
                                else if (dead) begin
                                    dead <= 1'b0; dst <= D_IDLE; resp <= R_IDLE;
                                    ack({R_PROTO, R_PROTO});
                                end else if (cond_arm) begin  // the ARM's answer
                                    cond_arm <= 1'b0;
                                    dst  <= D_IDLE;
                                    resp <= R_IDLE;
                                    ack(r_exc ? {R_BSUN, R_BSUN} : {2{15'h0400, r_tf}});
                                end else if (cond_nan) begin  // BSUN: ask the ARM
                                    cond_arm  <= 1'b1;
                                    in_len    <= 8'd0;
                                    post_req  <= 1'b1;
                                    post_kind <= 2'd3;
                                    ack({R_CA, R_CA});
                                end else begin
                                    dst  <= D_IDLE;
                                    resp <= R_IDLE;
                                    ack({2{15'h0400, tf}});
                                end
                            D_IN:
                                if (in_pos >= in_len) begin // operand complete: post
                                    post_req  <= 1'b1;
                                    post_kind <= 2'd1;
                                    dst       <= D_IDLE;
                                    resp      <= R_IDLE;
                                    ack(opclass == 3'b010 ? {R_REL, R_REL} : {R_IDLE, R_IDLE});
                                end else
                                    ack({resp, resp});
                            D_WAIT:
                                if (busy || post_req) ack({R_CA, R_CA});
                                else if (dead) begin
                                    dead <= 1'b0; dst <= D_IDLE; resp <= R_IDLE;
                                    ack({R_PROTO, R_PROTO});
                                end else if (r_unimpl) begin
                                    dst <= D_IDLE; resp <= R_IDLE;
                                    ack({R_FLINE, R_FLINE});
                                end else begin
                                    out_start;
                                    st <= T_CIR;
                                end
                            D_PRIM: begin                   // the data primitive
                                dst <= D_OUT;
                                ack({resp, resp});
                            end
                            D_OUT: begin                    // result taken
                                dst  <= D_IDLE;
                                resp <= R_IDLE;
                                ack({R_IDLE, R_IDLE});
                            end
                            default: begin                  // a primitive reads once
                                ack({resp, resp});
                                if (resp[12:8] != 5'b01000 && resp[12:8] != 5'b01001) resp <= R_IDLE;
                            end
                        endcase
                    CIR_RSEL: begin
                        if (dst == D_RSEL) begin
                            if (opclass == 3'b110) begin
                                in_len <= mm_len; in_pos <= 8'd0; dst <= D_IN;
                            end else
                                dst <= D_OUT;
                            resp <= R_CA;
                        end
                        ack({mask, 8'h00, mask, 8'h00});
                    end
                    CIR_SAVE:
                        if (busy || post_req) ack({F_CA, F_CA});
                        else if (st_null) ack(32'h0000_0000);
                        else begin
                            frm_cnt  <= FRAME_LONGS;
                            frm_save <= 1'b1;
                            ack({F_IDLE, F_IDLE});
                        end
                    CIR_REST: ack({rest_fmt, rest_fmt});
                    CIR_OPND:
                        if (dst == D_OUT) begin
                            xn <= acc_n;
                            dma(1'b0, A_ADATA + {16'd0, oofs[7:1], 1'b0}, 16'd0, 2'b11,
                                acc_n == 3'd4 ? T_OLD1 : T_OLDD);
                        end else if (frm_cnt != 4'd0 && frm_save) begin
                            frm_cnt <= frm_cnt - 1'd1;
                            ack(32'h0000_0000);
                        end else
                            ack(32'hFFFF_FFFF);
                    default: ack(32'hFFFF_FFFF);        // unimplemented CIRs
                endcase
            end

        // ---- operand to the request: second word of a long, then done ----
        T_OST1: dma(1'b1, A_RDATA + {16'd0, in_pos} + 24'd2, cr_wdata[15:0], 2'b11, T_OSTD);
        T_OSTD: begin
            in_pos <= in_pos + {5'd0, xn};
            ack(32'hFFFF_FFFF);
        end

        // ---- result from the reply ----
        T_OLD1: begin
            ld_hi <= rd_q;
            dma(1'b0, A_ADATA + {16'd0, oofs} + 24'd2, 16'd0, 2'b11, T_OLDD);
        end
        T_OLDD: begin
            if (xn == 3'd4)      ack({ld_hi, rd_q});
            else if (xn == 3'd2) ack({rd_q, rd_q});
            else                 ack({4{oofs[0] ? rd_q[7:0] : rd_q[15:8]}});
            if (out_off + {5'd0, xn} >= blk_size) begin
                out_off <= 8'd0;
                out_blk <= out_blk + 1'd1;
            end else
                out_off <= out_off + {5'd0, xn};
        end

        // ---- post a request: header, RSEQ last ----
        T_PKIND: dma(1'b1, A_KIND, {14'd0, post_kind}, 2'b11, T_PCMD);
        T_PCMD:  dma(1'b1, A_CMD, post_kind == 2'd3 ? {10'd0, cond[5:0]} : cmd, 2'b11, T_PAUX);
        T_PAUX:  dma(1'b1, A_AUX, aux, 2'b11, T_PNB);
        T_PNB:   dma(1'b1, A_RNB, {8'd0, in_len}, 2'b11, T_PSEQ);
        T_PSEQ:  dma(1'b1, A_RSEQ, rseq + 1'd1, 2'b11, T_PDONE);
        T_PDONE: begin
            rseq     <= rseq + 1'd1;
            busy     <= 1'b1;
            post_req <= 1'b0;
            gap      <= GAP_WAIT;
            st       <= T_IDLE;
        end

        // ---- poll for the reply ----
        T_QSEQ:
            if (busy && rd_q == rseq) dma(1'b0, A_FLAGS, 16'd0, 2'b11, T_QFLG);
            else begin
                gap <= (dst == D_WAIT || dst == D_CMD || dst == D_COND) ? GAP_WAIT : GAP_BG;
                st  <= T_IDLE;
            end
        T_QFLG: begin
            r_unimpl <= rd_q[0];
            r_exc    <= rd_q[1];
            r_tf     <= rd_q[2];
            dma(1'b0, A_FPSRH, 16'd0, 2'b11, T_QFPSR);
        end
        T_QFPSR: begin                      // (FPCR and the result length are not needed)
            fcc <= rd_q[11:8];
            st  <= T_QDONE;
        end
        T_QDONE: begin
            busy <= 1'b0;
            if (resp == R_REL) resp <= R_IDLE;  // the background instruction is done
            st   <= T_IDLE;
        end

        // ---- presence ----
        T_SMAG: begin
            if (rd_q != MAGIC) begin alive <= 5'd0; st <= T_IDLE; end
            else dma(1'b0, A_VER, 16'd0, 2'b11, T_SVER);
        end
        T_SVER:
            if (rd_q != VERSION) begin alive <= 5'd0; st <= T_IDLE; end
            else dma(1'b0, A_HB, 16'd0, 2'b11, T_SHB);
        T_SHB: begin
            if (rd_q != last_hb) alive <= 5'(ALIVE_POLLS);
            else if (alive != 5'd0) alive <= alive - 1'd1;
            last_hb <= rd_q;
            st      <= T_IDLE;
        end

        default: st <= T_IDLE;
    endcase

    if (reset) begin
        st_null  <= 1'b1;
        dst      <= D_IDLE;
        resp     <= R_IDLE;
        fcc      <= 4'd0;
        frm_cnt  <= 4'd0;
        rest_fmt <= 16'h0000;
        dead     <= 1'b0;
        cond_arm <= 1'b0;
        rst_req  <= 1'b1;               // reset the FPU on the ARM too
        if (st != T_PKIND && st != T_PCMD && st != T_PAUX && st != T_PNB && st != T_PSEQ)
            post_req <= 1'b0;
    end
    if (por) begin
        st       <= T_IDLE;
        dma_req  <= 1'b0;
        cr_pend  <= 1'b0;
        busy     <= 1'b0;
        post_req <= 1'b0;
        alive    <= 5'd0;
        ptmr     <= '0;
        gap      <= 6'd0;
        rseq     <= 16'd0;
    end
end

endmodule
