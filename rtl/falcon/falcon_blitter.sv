// falcon_blitter.sv - Atari BLiTTER (Mega ST / STE / Falcon030) for Falcon_MiSTer
//
// Behavioural reference: Hatari src/blitter.c
//   Blitter_Step, Blitter_ProcessWord, Blitter_SourceShift/SourceFetch/
//   SourceRead, Blitter_GetHalftoneWord, Blitter_HOP_*, Blitter_LOP_*,
//   Blitter_LOP_Table (need_src/need_dst), Blitter_FlushWordState,
//   Blitter_Start (pass loop, 64 bus accesses in non-hog mode),
//   Blitter_Control_WriteByte (start / restart / pause / busy clear),
//   Blitter_*_ReadWord/WriteWord (register masks and readback),
//   Blitter_Reset, Blitter_HOG_CPU_mem_access_after (64 CPU bus accesses).
// Secondary reference: FireBee Video/BLITTER (blitter.tdf).
//
// Registers ($FF8A00-$FF8A3D, bus_addr[5:1] = word index):
//   00-0F halftone RAM (16 words)       10 src X inc   11 src Y inc
//   12/13 src address (hi/lo word)      14/15/16 endmask 1/2/3
//   17 dst X inc   18 dst Y inc         19/1A dst address (hi/lo word)
//   1B X count     1C Y count           1D HOP (even byte) / LOP (odd byte)
//   1E control (even byte) / skew (odd byte)
//   Word and long registers ignore byte writes (Blitter_CheckAccess_Byte).
//   Increments and addresses have bit 0 forced to 0.  Addresses hold 24
//   significant bits; the high word reads $00xx.  X/Y count written as 0
//   means 65536 (reads back 0).  HOP reads back 2 bits, LOP 4 bits, the
//   control register is stored masked with $EF (bit 4 reads 0) and the skew
//   register keeps all 8 bits as written (as Hatari).
//
// Control register: bit 7 busy (write 1 = start/restart, write 0 = pause),
// bit 6 hog, bit 5 smudge, bits 3..0 halftone line number (live counter).
//
// Bus ownership:
//   br  = bus request to the CPU; the blitter only issues DMA accesses while
//         bg = 1 (bus granted).  An access in flight always completes.
//   Hog mode: br stays asserted until the blit is done.
//   Non-hog mode: br is released after NONHOG_BLIT_ACCESSES (64) blitter bus
//   accesses (reads and writes, counted exactly at the same points as Hatari:
//   possibly in the middle of a word, the word state is kept).  The blitter
//   then counts cpu_bus_cycle pulses (one per completed CPU bus cycle, given
//   by the system) and requests the bus again after NONHOG_CPU_ACCESSES (64)
//   of them.  Because a 68030 running from its caches or halted by STOP may
//   make no bus cycles at all, the blitter also requests the bus again after
//   NONHOG_CPU_TIMEOUT clocks without having seen 64 pulses (default 512
//   clocks = 256 CPU cycles at 16 MHz, which is the 64*4 CPU cycle
//   approximation Hatari uses when not in cycle exact mode).  Set
//   NONHOG_CPU_TIMEOUT = 0 to disable the timeout.
//   With the CPU's Falcon bus timing (fmode, docs/CPU_TIMING.md) the CPU's
//   share is NONHOG_CPU_CLOCKS (256) of its clocks in Falcon time (ftick),
//   as Hatari runs the 68030 between two non-hog bursts (blitter.c:931-937,
//   BLITTER_NONHOG_BUS_CPU * 4); the time of the blit itself is accounted
//   by falcon_cpubus (Hatari's 4 clocks per access and arbitration).
//   Writing control with bit 7 = 1 while the CPU owns the bus restarts the
//   blitter at once (keeping its word state); writing bit 7 = 0 pauses it
//   (busy stays 1, no bus request) until bit 7 is written to 1 again.
//   Writing bit 7 = 1 when Y count = 0 clears busy and hog and starts nothing.
//
// busy output: 1 from the start until Y count reaches 0 (also while paused or
//   while the CPU has the bus in non-hog mode).  The system inverts it for
//   MFP GPIP3.  No other interrupt output.
//
// Deviations from Hatari:
//   - Hatari is cycle exact only for a 68000; there is no start delay or
//     "63 accesses" bus count error (Blitter_HOG_CPU_BusCountError) here.
//   - Address registers: the two words of an address register are separate
//     registers here (as on the hardware); Hatari rebuilds the long from its
//     IoMem cache, which can be stale after the blitter crossed a 64 KB
//     boundary if only the low word is written afterwards.
//   - Address counters are 24 bits and wrap; Hatari's 32-bit counters can
//     carry above bit 23 (and read back so) after an overflow.
//   - Byte reads of word registers return the register (Hatari leaves the
//     stale IoMem content).
//   - The control register line number (bits 3..0) is the live counter; in
//     Hatari it is copied at the end of each pass (same value whenever the
//     CPU can read it).
//   - Writing bit 7 = 0 between the bus request and the grant also pauses
//     (busy stays 1); in Hatari non-CE that cancels the pending start.
//   - The halftone RAM, the source buffer and x_count_reset are not cleared
//     by reset (as Blitter_Reset); they power up as 0.

/* verilator lint_off UNUSEDPARAM */
module falcon_blitter #(
    parameter CLK_HZ               = 32000000,
    parameter NONHOG_BLIT_ACCESSES = 64,
    parameter NONHOG_CPU_ACCESSES  = 64,
    parameter NONHOG_CPU_TIMEOUT   = 512,
    parameter NONHOG_CPU_CLOCKS    = 256
) (
/* verilator lint_on UNUSEDPARAM */
    input             clk,
    input             reset,

    // register bus
    input             bus_cs,
    input             bus_stb,
    input             bus_we,
    input      [5:1]  bus_addr,
    input             bus_uds,
    input             bus_lds,
    input      [15:0] bus_din,
    output reg [15:0] bus_dout,
    output reg        bus_ack,

    // DMA (ST-RAM master) port
    output reg        dma_req,
    output reg        dma_we,
    output reg [23:1] dma_addr,
    output     [1:0]  dma_be,
    output reg [15:0] dma_wdata,
    input      [15:0] dma_rdata,
    input             dma_ack,

    // bus arbitration
    output            br,
    input             bg,
    input             cpu_bus_cycle,
    input             fmode,        // Falcon bus timing: the CPU's share in its clocks
    input             ftick,        // a CPU clock of Falcon time

    output            busy
);

    // ------------------------------------------------------------------
    // registers
    // ------------------------------------------------------------------
    // the halftone RAM in LUT RAM (MLAB, asynchronous read): one copy for
    // the blit, one for the CPU's reads
    (* ramstyle = "MLAB, no_rw_check" *) reg [15:0] ht  [0:15];
    (* ramstyle = "MLAB, no_rw_check" *) reg [15:0] htc [0:15];
    reg [15:1] src_xinc, src_yinc, dst_xinc, dst_yinc;
    reg [23:1] src_addr, dst_addr;
    reg [15:0] endmask1, endmask2, endmask3;
    reg [16:0] x_count, x_count_reset, y_count;
    reg [1:0]  hop;
    reg [3:0]  lop;
    reg        ctl_busy, ctl_hog, ctl_smudge;
    reg [3:0]  line_nr;
    reg [7:0]  skew_reg;

    wire       v_fxsr = skew_reg[7];
    wire       v_nfsr = skew_reg[6];
    wire [3:0] v_skew = skew_reg[3:0];

    integer i;
    initial begin
        for (i = 0; i < 16; i = i + 1) begin ht[i] = 16'h0000; htc[i] = 16'h0000; end
    end

    // ------------------------------------------------------------------
    // internal state (BlitterState / BlitterVars)
    // ------------------------------------------------------------------
    reg [31:0] buffer;
    reg [15:0] bus_word;
    reg [15:0] dst_word;
    reg [15:0] end_mask;
    reg [15:0] wdata;
    reg        st_fxsr, st_nfsr, have_fxsr, have_src, fetch_src, have_dst;
    reg        need_src;
    reg        need_dst;
    reg        weird;      // nfsr && x_count == 1 for the current word


    localparam P_STOP  = 2'd0;     // idle
    localparam P_RUN   = 2'd1;     // wants / owns the bus
    localparam P_CPU   = 2'd2;     // non-hog: CPU owns the bus for 64 accesses
    localparam P_PAUSE = 2'd3;     // stopped by writing busy = 0
    reg [1:0] phase;

    localparam F_STEP   = 3'd0;    // word setup (Blitter_Step head)
    localparam F_DECIDE = 3'd1;    // choose next read (Blitter_ProcessWord)
    localparam F_RD     = 3'd2;    // read in progress
    localparam F_PRE    = 3'd3;    // xcount=1 & NFSR pre-shift
    localparam F_CALC   = 3'd4;    // compute destination word
    localparam F_WR     = 3'd5;    // write in progress
    reg [2:0] fsm;

    localparam RK_FXSR = 2'd0;
    localparam RK_SRC  = 2'd1;
    localparam RK_DST  = 2'd2;
    reg [1:0] rd_kind;

    reg [7:0]  blit_cnt;
    reg [7:0]  cpu_cnt;
    reg [15:0] tmo_cnt;

    assign br     = (phase == P_RUN);
    assign busy   = (phase != P_STOP);
    assign dma_be = 2'b11;

    // power-up values of the state that reset does not clear
    initial begin
        buffer        = 32'h0;
        dst_word      = 16'h0;
        end_mask      = 16'h0;
        wdata         = 16'h0;
        need_src      = 1'b0;
        need_dst      = 1'b0;
        weird         = 1'b0;
        x_count_reset = 17'h0;
        rd_kind       = 2'd0;
        bus_dout      = 16'h0;
        dma_addr      = 23'h0;
        dma_wdata     = 16'h0;
    end

    // ------------------------------------------------------------------
    // datapath helpers
    // ------------------------------------------------------------------
    wire        src_neg = src_xinc[15];
    wire [22:0] src_xinc_ext = {{8{src_xinc[15]}}, src_xinc[15:1]};
    wire [22:0] src_yinc_ext = {{8{src_yinc[15]}}, src_yinc[15:1]};
    wire [22:0] dst_xinc_ext = {{8{dst_xinc[15]}}, dst_xinc[15:1]};
    wire [22:0] dst_yinc_ext = {{8{dst_yinc[15]}}, dst_yinc[15:1]};

    // Blitter_SourceShift + Blitter_SourceFetch
    function [31:0] shift_fetch(input [31:0] b, input [15:0] w, input neg);
        shift_fetch = neg ? {w, b[31:16]} : {b[15:0], w};
    endfunction

    // Blitter_SourceRead
    /* verilator lint_off UNUSEDSIGNAL */
    wire [31:0] buf_sh   = buffer >> v_skew;
    /* verilator lint_on UNUSEDSIGNAL */
    wire [15:0] src_word = buf_sh[15:0];
    // Blitter_GetHalftoneWord
    wire [15:0] ht_word  = ht[ctl_smudge ? src_word[3:0] : line_nr];

    reg [15:0] hop_out;
    always @(*) begin
        case (hop)
            2'd0: hop_out = 16'hFFFF;
            2'd1: hop_out = ht_word;
            2'd2: hop_out = src_word;
            default: hop_out = src_word & ht_word;
        endcase
    end

    reg [15:0] lop_out;
    always @(*) begin
        case (lop)
            4'h0: lop_out = 16'h0000;
            4'h1: lop_out =  hop_out &  dst_word;
            4'h2: lop_out =  hop_out & ~dst_word;
            4'h3: lop_out =  hop_out;
            4'h4: lop_out = ~hop_out &  dst_word;
            4'h5: lop_out =  dst_word;
            4'h6: lop_out =  hop_out ^  dst_word;
            4'h7: lop_out =  hop_out |  dst_word;
            4'h8: lop_out = ~hop_out & ~dst_word;
            4'h9: lop_out = ~hop_out ^  dst_word;
            4'hA: lop_out = ~dst_word;
            4'hB: lop_out =  hop_out | ~dst_word;
            4'hC: lop_out = ~hop_out;
            4'hD: lop_out = ~hop_out |  dst_word;
            4'hE: lop_out = ~hop_out | ~dst_word;
            default: lop_out = 16'hFFFF;
        endcase
    end

    wire [15:0] dst_data = (end_mask != 16'hFFFF) ?
                           ((lop_out & end_mask) | (dst_word & ~end_mask)) : lop_out;

    // Blitter_LOP_Table
    wire lop_need_src = !(lop == 4'h0 || lop == 4'h5 || lop == 4'hA || lop == 4'hF);
    wire lop_need_dst = !(lop == 4'h0 || lop == 4'h3 || lop == 4'hC || lop == 4'hF);

    // Blitter_Step: word setup
    wire        first_word = (x_count == x_count_reset);
    wire [15:0] mask_sel   = (first_word || x_count_reset == 17'd1) ? endmask1 :
                             (x_count == 17'd1) ? endmask3 : endmask2;

    // ------------------------------------------------------------------
    // register read mux
    // ------------------------------------------------------------------
    reg [15:0] rd_mux;
    always @(*) begin
        case (bus_addr)
            5'h10: rd_mux = {src_xinc, 1'b0};
            5'h11: rd_mux = {src_yinc, 1'b0};
            5'h12: rd_mux = {8'h00, src_addr[23:16]};
            5'h13: rd_mux = {src_addr[15:1], 1'b0};
            5'h14: rd_mux = endmask1;
            5'h15: rd_mux = endmask2;
            5'h16: rd_mux = endmask3;
            5'h17: rd_mux = {dst_xinc, 1'b0};
            5'h18: rd_mux = {dst_yinc, 1'b0};
            5'h19: rd_mux = {8'h00, dst_addr[23:16]};
            5'h1A: rd_mux = {dst_addr[15:1], 1'b0};
            5'h1B: rd_mux = x_count[15:0];
            5'h1C: rd_mux = y_count[15:0];
            5'h1D: rd_mux = {6'b0, hop, 4'b0, lop};
            5'h1E: rd_mux = {ctl_busy, ctl_hog, ctl_smudge, 1'b0, line_nr, skew_reg};
            5'h1F: rd_mux = 16'hFFFF;
            default: rd_mux = htc[bus_addr[4:1]];
        endcase
    end

    wire wr_stb  = bus_cs & bus_stb & bus_we;
    wire wr_word = wr_stb & bus_uds & bus_lds;
    wire wr_ctl  = wr_stb & bus_uds & (bus_addr == 5'h1E);
    wire ht_we   = !reset && wr_word && !bus_addr[5];
    always @(posedge clk) if (ht_we) ht[bus_addr[4:1]]  <= bus_din;
    always @(posedge clk) if (ht_we) htc[bus_addr[4:1]] <= bus_din;

    // non-hog bus count after the current access
    wire [7:0] blit_cnt_inc = blit_cnt + 8'd1;
    wire       nonhog_yield = !ctl_hog && (blit_cnt_inc >= NONHOG_BLIT_ACCESSES);

    // post-write values (Blitter_Step tail)
    wire        nfsr_next  = ((x_count == 17'd2) && v_nfsr) ? 1'b1 : st_nfsr;
    wire        line_end   = (x_count == 17'd1);
    wire [16:0] y_count_nx = line_end ? (y_count - 17'd1) : y_count;

    // ------------------------------------------------------------------
    // main sequential block
    // ------------------------------------------------------------------
    always @(posedge clk) begin
        bus_ack <= 1'b0;
        if (bus_cs && bus_stb) begin
            bus_ack  <= 1'b1;
            bus_dout <= rd_mux;
        end

        if (reset) begin
            src_xinc <= 15'd0; src_yinc <= 15'd0;
            dst_xinc <= 15'd0; dst_yinc <= 15'd0;
            src_addr <= 23'd0; dst_addr <= 23'd0;
            endmask1 <= 16'd0; endmask2 <= 16'd0; endmask3 <= 16'd0;
            x_count  <= 17'd0; y_count  <= 17'd0;
            hop      <= 2'd0;  lop      <= 4'd0;
            ctl_busy <= 1'b0;  ctl_hog  <= 1'b0; ctl_smudge <= 1'b0;
            line_nr  <= 4'd0;  skew_reg <= 8'd0;
            st_fxsr  <= 1'b0;  st_nfsr  <= 1'b0; have_fxsr <= 1'b0;
            have_src <= 1'b0;  fetch_src <= 1'b0; have_dst <= 1'b0;
            bus_word <= 16'd0;
            phase    <= P_STOP;
            fsm      <= F_STEP;
            dma_req  <= 1'b0;
            dma_we   <= 1'b0;
            blit_cnt <= 8'd0;
            cpu_cnt  <= 8'd0;
            tmo_cnt  <= 16'd0;
        end else begin
            // ---------------- non-hog CPU share ----------------
            if (phase == P_CPU && fmode) begin
                if (ftick) begin
                    tmo_cnt <= tmo_cnt + 16'd1;
                    if (tmo_cnt + 16'd1 >= NONHOG_CPU_CLOCKS) begin
                        phase    <= P_RUN;
                        blit_cnt <= 8'd0;
                    end
                end
            end
            else if (phase == P_CPU) begin
                if (cpu_bus_cycle)
                    cpu_cnt <= cpu_cnt + 8'd1;
                tmo_cnt <= tmo_cnt + 16'd1;
                if ((cpu_bus_cycle && (cpu_cnt + 8'd1 >= NONHOG_CPU_ACCESSES)) ||
                    (NONHOG_CPU_TIMEOUT != 0 && (tmo_cnt + 16'd1 >= NONHOG_CPU_TIMEOUT))) begin
                    phase    <= P_RUN;
                    blit_cnt <= 8'd0;
                end
            end

            // ---------------- blit sequencer ----------------
            case (fsm)
            F_STEP: begin
                if (phase == P_RUN && bg) begin
                    end_mask <= mask_sel;
                    if (first_word) begin
                        st_nfsr <= 1'b0;
                        st_fxsr <= v_fxsr;
                    end
                    need_src <= lop_need_src && (hop[1] || (hop == 2'd1 && ctl_smudge));
                    need_dst <= lop_need_dst || (mask_sel != 16'hFFFF);
                    fsm <= F_DECIDE;
                end
            end

            F_DECIDE: begin
                if (st_fxsr && !have_fxsr && need_src) begin
                    rd_kind  <= RK_FXSR;
                    dma_addr <= src_addr;
                    fsm      <= F_RD;
                end else if (need_src && !have_src && !st_nfsr) begin
                    rd_kind  <= RK_SRC;
                    dma_addr <= src_addr;
                    fsm      <= F_RD;
                end else if (need_dst && !have_dst) begin
                    rd_kind  <= RK_DST;
                    dma_addr <= dst_addr;
                    fsm      <= F_RD;
                end else begin
                    fsm <= F_PRE;
                end
                dma_we <= 1'b0;
                weird  <= v_nfsr && (x_count == 17'd1);
            end

            F_RD: begin
                if (!dma_req) begin
                    if (phase == P_RUN && bg)
                        dma_req <= 1'b1;
                end else if (dma_ack) begin
                    dma_req  <= 1'b0;
                    bus_word <= dma_rdata;
                    case (rd_kind)
                    RK_FXSR: begin
                        buffer    <= shift_fetch(buffer, dma_rdata, src_neg);
                        src_addr  <= src_addr + src_xinc_ext;
                        have_fxsr <= 1'b1;
                    end
                    RK_SRC: begin
                        buffer    <= shift_fetch(buffer, dma_rdata, src_neg);
                        have_src  <= 1'b1;
                        fetch_src <= 1'b1;
                    end
                    default: begin
                        dst_word  <= dma_rdata;
                        have_dst  <= 1'b1;
                    end
                    endcase
                    blit_cnt <= blit_cnt_inc;
                    if (nonhog_yield) begin
                        // BLITTER_CONTINUE_LATER_IF_MAX_BUS_REACHED
                        fsm <= F_STEP;
                        if (phase == P_RUN) begin
                            phase   <= P_CPU;
                            cpu_cnt <= 8'd0;
                            tmo_cnt <= 16'd0;
                        end
                    end else begin
                        fsm <= F_DECIDE;
                    end
                end
            end

            F_PRE: begin
                // special case x_count=1 and NFSR=1, before the LOP
                if (weird)
                    buffer <= shift_fetch(buffer, bus_word, src_neg);
                fsm <= F_CALC;
            end

            F_CALC: begin
                wdata <= dst_data;
                fsm   <= F_WR;
            end

            F_WR: begin
                if (!dma_req) begin
                    if (phase == P_RUN && bg) begin
                        dma_req   <= 1'b1;
                        dma_we    <= 1'b1;
                        dma_addr  <= dst_addr;
                        dma_wdata <= wdata;
                    end
                end else if (dma_ack) begin
                    dma_req  <= 1'b0;
                    dma_we   <= 1'b0;
                    bus_word <= wdata;
                    // special case x_count=1 and NFSR=1, after the write
                    if (weird)
                        buffer <= shift_fetch(buffer, wdata, src_neg);

                    st_nfsr <= nfsr_next;
                    if (fetch_src)
                        src_addr <= src_addr + ((line_end || nfsr_next) ? src_yinc_ext : src_xinc_ext);

                    if (line_end) begin
                        have_fxsr <= 1'b0;
                        y_count   <= y_count_nx;
                        x_count   <= x_count_reset;
                        dst_addr  <= dst_addr + dst_yinc_ext;
                        line_nr   <= dst_yinc[15] ? (line_nr - 4'd1) : (line_nr + 4'd1);
                    end else begin
                        x_count   <= x_count - 17'd1;
                        dst_addr  <= dst_addr + dst_xinc_ext;
                    end
                    have_src  <= 1'b0;
                    fetch_src <= 1'b0;
                    have_dst  <= 1'b0;

                    blit_cnt <= blit_cnt_inc;
                    fsm      <= F_STEP;
                    if (y_count_nx == 17'd0) begin
                        // blit complete: clear busy and hog
                        phase    <= P_STOP;
                        ctl_busy <= 1'b0;
                        ctl_hog  <= 1'b0;
                    end else if (nonhog_yield && phase == P_RUN) begin
                        phase   <= P_CPU;
                        cpu_cnt <= 8'd0;
                        tmo_cnt <= 16'd0;
                    end
                end
            end

            default: fsm <= F_STEP;
            endcase

            // ---------------- register writes ----------------
            if (wr_word) begin
                case (bus_addr)
                5'h10: src_xinc <= bus_din[15:1];
                5'h11: src_yinc <= bus_din[15:1];
                5'h12: src_addr[23:16] <= bus_din[7:0];
                5'h13: src_addr[15:1]  <= bus_din[15:1];
                5'h14: endmask1 <= bus_din;
                5'h15: endmask2 <= bus_din;
                5'h16: endmask3 <= bus_din;
                5'h17: dst_xinc <= bus_din[15:1];
                5'h18: dst_yinc <= bus_din[15:1];
                5'h19: dst_addr[23:16] <= bus_din[7:0];
                5'h1A: dst_addr[15:1]  <= bus_din[15:1];
                5'h1B: begin
                    x_count       <= (bus_din == 16'd0) ? 17'h10000 : {1'b0, bus_din};
                    x_count_reset <= (bus_din == 16'd0) ? 17'h10000 : {1'b0, bus_din};
                end
                5'h1C: y_count <= (bus_din == 16'd0) ? 17'h10000 : {1'b0, bus_din};
                default: ;
                endcase
                // (the halftone RAM: ht_we)
            end
            if (wr_stb && bus_addr == 5'h1D) begin
                if (bus_uds) hop <= bus_din[9:8];
                if (bus_lds) lop <= bus_din[3:0];
            end
            if (wr_stb && bus_addr == 5'h1E && bus_lds)
                skew_reg <= bus_din[7:0];
            if (wr_ctl) begin
                // Blitter_Control_WriteByte
                ctl_busy   <= bus_din[15];
                ctl_hog    <= bus_din[14];
                ctl_smudge <= bus_din[13];
                line_nr    <= bus_din[11:8];
                if (bus_din[15]) begin
                    if (y_count == 17'd0) begin
                        ctl_busy <= 1'b0;
                        ctl_hog  <= 1'b0;
                    end else begin
                        if (phase == P_STOP) begin
                            // Blitter_FlushWordState(true)
                            have_fxsr <= 1'b0;
                            have_src  <= 1'b0;
                            fetch_src <= 1'b0;
                            have_dst  <= 1'b0;
                            fsm       <= F_STEP;
                        end
                        if (phase != P_RUN)
                            blit_cnt <= 8'd0;
                        phase <= P_RUN;
                    end
                end else begin
                    if (phase == P_CPU || phase == P_RUN)
                        phase <= P_PAUSE;
                end
            end
        end
    end

endmodule
