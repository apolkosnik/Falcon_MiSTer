// falcon_fpu_bridge.sv - MC68882 coprocessor interface for an FPU on the HPS
//
// The 68882's protocol is answered here; its registers and arithmetic live
// in a process on the ARM (tools/falcon_fpu), reached through a DDR3
// mailbox on falcon_memarb's d3 port.  Design: docs/FPU_ARM.md.
//
// Milestone 1 (this version): presence, state and frames.
//   - Presence: the ARM service writes MAGIC and increments HEARTBEAT (about
//     every 10 ms).  Polled every 5 ms; the FPU exists while HEARTBEAT has
//     changed within the last 100 ms.  Without it the first CIR access of an
//     instruction (command, condition or restore write, save read) ends in
//     BERR, so the 68030 takes the F-line exception: no FPU, as before.  BERR
//     is never given later in an instruction (MC68030 UM 10: after the first
//     access a BERR is a bus error, not "no coprocessor").
//   - Only CpID 1 (F-line $F2xx-$F3xx) is the FPU; other CpIDs: BERR.
//   - Null/idle state (MC68881/MC68882 UM 6.4.2): null after reset and after
//     FRESTORE of a null frame; idle after any command or condition word.
//     FSAVE: null -> format $0000; idle -> $1F38 (68882 idle frame) and 14
//     operand longs.  FRESTORE: $00xx resets to null; $1F38 restores idle
//     (14 longs taken); any other format is echoed as $02xx (invalid: the
//     68030 takes the format error exception).  The idle frame body carries
//     no state in this milestone (all visible FPU state is in FP0-7 and the
//     control registers, which an OS saves with FMOVEM).
//   - Conditionals (FBcc/FScc/FDBcc/FTRAPcc, FNOP = FBF): the predicate of
//     the condition word is evaluated from the FPSR condition codes held
//     here (N, Z, I, NAN; mirrored from the ARM in later milestones, all 0
//     for now) and answered as null with TF (UM 4.x conditional tests).
//     BSUN (IEEE-nonaware predicate with NAN set and BSUN enabled) answers
//     the pre-instruction exception primitive with vector 48.
//   - Any cpGEN command word (arithmetic, moves, FMOVEM) is not implemented
//     yet and answers the pre-instruction exception primitive with vector 11
//     (F-line), so a software FPU emulator could still take it.
//   - Response primitives (UM table 7-7): $0802 null/PF (idle), $0800|TF
//     null with the condition result, $1Cvv take pre-instruction exception.
//     A non-null primitive reads once; then the response is $0802 again.
//   - CIR reads: word CIRs on [31:16] and [15:0]; unimplemented CIRs read
//     all ones (operation word $08, operand address $1C).
//
// Mailbox (guest $E90000, DDR3 0x30E90000; 16-bit words, big endian; DDR3
// byte k = guest byte k):
//   +$000 MAGIC      ARM -> FPGA  $4650 ("FP") while the service runs
//   +$002 HEARTBEAT  ARM -> FPGA  incremented about every 10 ms
//   +$004 VERSION    ARM -> FPGA  mailbox protocol version (1)
//   (requests and replies follow in milestone 2)
module falcon_fpu_bridge #(
    parameter int CLK_HZ = 32000000
) (
    input             clk,
    input             reset,        // FPU reset: machine reset or RESET instruction
    input             por,          // power on: presence tracking

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
    output     [1:0]  dma_be,
    output reg [15:0] dma_wdata = 16'd0,
    input      [15:0] dma_rdata,
    input             dma_ack,

    output            present
);

localparam [4:0] CIR_RESP = 5'h00, CIR_CTRL = 5'h02, CIR_SAVE = 5'h04, CIR_REST = 5'h06,
                 CIR_OPWD = 5'h08, CIR_CMD  = 5'h0A, CIR_COND = 5'h0E, CIR_OPND = 5'h10,
                 CIR_RSEL = 5'h14, CIR_IADR = 5'h18, CIR_OADR = 5'h1C;

localparam [15:0] R_IDLE  = 16'h0802;   // null, PF
localparam [15:0] R_FLINE = 16'h1C0B;   // take pre-instruction exception, vector 11
localparam [15:0] R_BSUN  = 16'h1C30;   // take pre-instruction exception, vector 48

localparam [15:0] F_IDLE  = 16'h1F38;   // 68882 idle frame: version $1F, 56 bytes
localparam [3:0]  FRAME_LONGS = 4'd14;

// ---------------------------------------------------------------------------
// Presence: poll MAGIC and HEARTBEAT
// ---------------------------------------------------------------------------
localparam [23:0] MB      = 24'hE90000;
localparam [23:0] A_MAGIC = MB + 24'h000;
localparam [23:0] A_HB    = MB + 24'h002;
localparam [15:0] MAGIC   = 16'h4650;
localparam int    POLL_CLKS   = CLK_HZ / 200;     // 5 ms
localparam int    ALIVE_POLLS = 20;               // 100 ms without a new heartbeat
localparam int    PW          = $clog2(POLL_CLKS + 1);

localparam [1:0] P_WAIT = 2'd0, P_MAGIC = 2'd1, P_HB = 2'd2;
reg  [1:0]    pst = P_WAIT;
reg  [PW-1:0] ptmr = '0;
reg  [4:0]    alive = 5'd0;
reg  [15:0]   last_hb = 16'd0;

assign dma_be  = 2'b11;
assign present = alive != 5'd0;

always @(posedge clk) begin
    if (dma_ack) dma_req <= 1'b0;
    case (pst)
        P_WAIT:  if (ptmr == PW'(POLL_CLKS)) begin
                     ptmr     <= '0;
                     dma_req  <= 1'b1;
                     dma_we   <= 1'b0;
                     dma_addr <= A_MAGIC[23:1];
                     pst      <= P_MAGIC;
                 end else
                     ptmr <= ptmr + 1'd1;
        P_MAGIC: if (dma_ack) begin
                     if (dma_rdata == MAGIC) begin
                         dma_req  <= 1'b1;
                         dma_addr <= A_HB[23:1];
                         pst      <= P_HB;
                     end else begin
                         alive <= 5'd0;
                         pst   <= P_WAIT;
                     end
                 end
        P_HB:    if (dma_ack) begin
                     if (dma_rdata != last_hb) alive <= 5'(ALIVE_POLLS);
                     else if (alive != 5'd0)   alive <= alive - 1'd1;
                     last_hb <= dma_rdata;
                     pst     <= P_WAIT;
                 end
        default: pst <= P_WAIT;
    endcase
    if (por) begin
        pst     <= P_WAIT;
        ptmr    <= '0;
        alive   <= 5'd0;
        dma_req <= 1'b0;
    end
end

// ---------------------------------------------------------------------------
// Conditional predicates (MC68881/MC68882 UM, conditional tests): the low
// four bits select the test; predicates $10-$1F are the IEEE-nonaware forms
// of $00-$0F (same result, BSUN when NAN is set).
// ---------------------------------------------------------------------------
function automatic pred_true(input [3:0] p, input n, input z, input nan);
    case (p)
        4'h0: pred_true = 1'b0;                       // F
        4'h1: pred_true = z;                          // EQ
        4'h2: pred_true = !(nan || z || n);           // OGT
        4'h3: pred_true = z || !(nan || n);           // OGE
        4'h4: pred_true = n && !(nan || z);           // OLT
        4'h5: pred_true = z || (n && !nan);           // OLE
        4'h6: pred_true = !(nan || z);                // OGL
        4'h7: pred_true = !nan;                       // OR
        4'h8: pred_true = nan;                        // UN
        4'h9: pred_true = nan || z;                   // UEQ
        4'hA: pred_true = nan || !(n || z);           // UGT
        4'hB: pred_true = nan || z || !n;             // UGE
        4'hC: pred_true = nan || (n && !z);           // ULT
        4'hD: pred_true = nan || z || n;              // ULE
        4'hE: pred_true = !z;                         // NE
        default: pred_true = 1'b1;                    // T
    endcase
endfunction

// ---------------------------------------------------------------------------
// FPU state and CIR responses
// ---------------------------------------------------------------------------
reg         st_null;
reg  [15:0] resp;
reg   [3:0] fcc;            // FPSR condition codes {N, Z, I, NAN}
reg         bsun_en;        // FPCR BSUN exception enable
reg   [3:0] frm_cnt;        // frame longs still to move
reg         frm_save;       // 1: FSAVE (operand reads), 0: FRESTORE (writes)
reg  [15:0] rest_fmt;       // what the restore CIR reads back

wire        is_fpu = cp_id == 3'd1;
wire [15:0] wword  = cp_off[1] ? cp_wdata[15:0] : cp_wdata[31:16];
wire        first  = (cp_we && (cp_off == CIR_CMD || cp_off == CIR_COND || cp_off == CIR_REST)) ||
                     (!cp_we && cp_off == CIR_SAVE);
wire        tf     = pred_true(wword[3:0], fcc[3], fcc[2], fcc[0]);
wire        bsun   = wword[4] && fcc[0] && bsun_en;

always @(posedge clk) begin
    cp_ack  <= 1'b0;
    cp_berr <= 1'b0;

    if (cp_req) begin
        cp_ack   <= 1'b1;
        cp_rdata <= 32'hFFFF_FFFF;
        if (!is_fpu || (first && !present))
            cp_berr <= 1'b1;
        else if (cp_we) begin
            case (cp_off)
                CIR_CMD: begin
                    st_null <= 1'b0;
                    resp    <= R_FLINE;              // cpGEN: milestone 2
                end
                CIR_COND: begin
                    st_null <= 1'b0;
                    resp    <= bsun ? R_BSUN : {15'h0400, tf};
                end
                CIR_CTRL: begin                      // abort / exception acknowledge
                    resp    <= R_IDLE;
                    frm_cnt <= 4'd0;
                end
                CIR_REST: begin
                    frm_cnt <= 4'd0;
                    if (wword[15:8] == 8'h00) begin  // null frame: reset
                        st_null  <= 1'b1;
                        fcc      <= 4'd0;
                        bsun_en  <= 1'b0;
                        resp     <= R_IDLE;
                        rest_fmt <= wword;
                    end else if (wword == F_IDLE) begin
                        st_null  <= 1'b0;
                        resp     <= R_IDLE;
                        rest_fmt <= wword;
                        frm_cnt  <= FRAME_LONGS;
                        frm_save <= 1'b0;
                    end else
                        rest_fmt <= {8'h02, wword[7:0]};
                end
                CIR_OPND:
                    if (frm_cnt != 4'd0 && !frm_save) frm_cnt <= frm_cnt - 1'd1;
                default: ;                           // instruction address etc.
            endcase
        end else begin
            case (cp_off)
                CIR_RESP: begin
                    cp_rdata <= {resp, resp};
                    if (resp[12:8] != 5'b01000 && resp[12:8] != 5'b01001) resp <= R_IDLE;
                end
                CIR_SAVE:
                    if (st_null) cp_rdata <= 32'h0000_0000;
                    else begin
                        cp_rdata <= {F_IDLE, F_IDLE};
                        frm_cnt  <= FRAME_LONGS;
                        frm_save <= 1'b1;
                    end
                CIR_REST: cp_rdata <= {rest_fmt, rest_fmt};
                CIR_OPND:
                    if (frm_cnt != 4'd0 && frm_save) begin
                        cp_rdata <= 32'h0000_0000;   // idle frame body
                        frm_cnt  <= frm_cnt - 1'd1;
                    end
                CIR_RSEL: cp_rdata <= 32'h0000_0000;
                default: ;                           // all ones
            endcase
        end
    end

    if (reset) begin
        st_null  <= 1'b1;
        resp     <= R_IDLE;
        fcc      <= 4'd0;
        bsun_en  <= 1'b0;
        frm_cnt  <= 4'd0;
        rest_fmt <= 16'h0000;
    end
end

endmodule
