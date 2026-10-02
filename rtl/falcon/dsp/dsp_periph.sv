// DSP56001 on-chip peripherals for the Falcon DSP (falcon_dsp):
// host interface (both the 68030 side at $FFA200-$FFA207 and the DSP side at
// X:$FFE8-$FFEB), SSI (X:$FFEC-$FFEF) with the Falcon crossbar slot link,
// port C data/direction (handshake lines), IPR, BCR, SCI registers and the
// plain storage of every other X:/Y:$FFC0-$FFFF location.
//
// Behavioural reference: Hatari src/falcon/dsp_core.c
//   dsp_core_reset, dsp_core_read_host, dsp_core_write_host,
//   dsp_core_hostport_update_trdy/_update_hreq, dsp_core_dsp2host,
//   dsp_core_host2dsp, dsp_core_hostport_dspread/_dspwrite,
//   dsp_core_ssi_configure, dsp_core_ssi_writeTX/_writeTSR/_readRX,
//   dsp_core_ssi_Receive_SC0/SC1/SC2/SCK, dsp_core_setPortCDataRegister;
// and src/falcon/dsp_cpu.c write_memory_raw / read_memory (peripheral part)
// and dsp_reset() (the RESET instruction, per_soft_reset).
//
// Order of events inside one clock: SSI slot (crossbar) first, then the DSP
// core's peripheral access / RESET instruction / host command acknowledge,
// then the 68030 bus access (even byte before odd byte, as Hatari's
// DSP_HandleRead/WriteAccess loop over the bytes of a word access).  A CVR
// read in the clock of the host command acknowledge already sees HC clear
// (Hatari runs dsp_postexecute_interrupts before the next host access).
//
// Deviations from Hatari:
//   - HREQ is a level (ISR bit 7) as on the hardware; Hatari additionally
//     drops its pending flag when the 68030 takes the interrupt.
//   - In DSP -> DMA record handshake mode the crossbar sends ssi_frame = 0
//     with the transmit slot; Hatari skips SC2 there, so TFS may differ.
//   - DSP reset is level sensitive (DSP held in reset while dsp_reset = 1);
//     Hatari resets once per PSG port A write with bit 4 set.
//   - A CRB write that sets TE (RE) makes the transmitter (receiver) wait
//     for the next frame sync again, as the hardware does.  Hatari intends
//     the same in dsp_core_ssi_configure, but its write_memory_raw stores the
//     new CRB before the old TE/RE bits are compared, so it never re-arms.
//   - With SHFD set Hatari's bit swap can produce a 25-bit RX value; RX keeps
//     the low 24 bits.

module dsp_periph (
    input             clk,
    input             rst,             // DSP reset (dsp_core_reset)

    // 68030 side (contract register bus, $FFA200-$FFA207)
    input             bus_cs,
    input             bus_stb,
    input             bus_we,
    input       [2:1] bus_addr,
    input             bus_uds,
    input             bus_lds,
    input      [15:0] bus_din,
    output     [15:0] bus_dout,
    output            bus_ack,
    output            hreq,
    output      [7:0] ivr,

    // bootstrap / run control
    output reg        running /*verilator public_flat_rw*/,
    output reg        run_start,
    output reg  [9:0] boot_pos /*verilator public_flat_rw*/,
    output reg        boot_we,
    output reg  [8:0] boot_addr,
    output reg [23:0] boot_data,

    // DSP core peripheral ports
    input             perx_rd,
    input             perx_wr,
    input       [5:0] perx_addr,
    input      [23:0] perx_wdata,
    output     [23:0] perx_rdata,
    input             pery_rd,
    input             pery_wr,
    input       [5:0] pery_addr,
    input      [23:0] pery_wdata,
    output     [23:0] pery_rdata,
    input             soft_reset,
    input             hc_ack,

    // interrupt lines to the core
    output            int_host_rcv,
    output            int_host_trx,
    output            int_host_cmd,
    output            int_ssi_rcv,
    output            int_ssi_trx,
    output            msk_host_rcv,
    output            msk_host_trx,
    output            msk_host_cmd,
    output            msk_ssi_rcv,
    output            msk_ssi_trx,
    output     [23:0] ipr_out,
    output      [4:0] hc_vector,

    // SSI slot link (crossbar)
    input             ssi_slot_stb,
    input             ssi_tx_en,
    input             ssi_frame,       // transmit frame sync (SC2)
    input             ssi_rx_en,
    input             ssi_rx_frame,    // receive frame sync (SC1)
    input      [15:0] ssi_rx_data,
    output     [15:0] ssi_tx_data,
    output reg        ssi_tx_valid,
    output reg        ssi_hs_play_req
);

    // ------------------------------------------------------------------
    // state
    // ------------------------------------------------------------------
    // 68030 side (hostport[])
    reg  [7:0] icr /*verilator public_flat_rw*/, cvr /*verilator public_flat_rw*/;
    reg  [7:0] isr /*verilator public_flat_rw*/, ivr_r /*verilator public_flat_rw*/;
    reg  [7:0] rxh /*verilator public_flat_rw*/, rxm /*verilator public_flat_rw*/, rxl /*verilator public_flat_rw*/;
    reg  [7:0] txh /*verilator public_flat_rw*/, txm /*verilator public_flat_rw*/, txl /*verilator public_flat_rw*/;
    // DSP side
    reg  [7:0] hsr /*verilator public_flat_rw*/;
    reg  [4:0] hcr /*verilator public_flat_rw*/;
    reg [23:0] htx /*verilator public_flat_rw*/, rtx /*verilator public_flat_rw*/;
    // SSI
    reg [23:0] cra /*verilator public_flat_rw*/, crb /*verilator public_flat_rw*/;
    reg  [7:0] ssisr /*verilator public_flat_rw*/;
    reg [23:0] ssi_tx /*verilator public_flat_rw*/, ssi_rx /*verilator public_flat_rw*/;
    reg [24:0] ssi_tval /*verilator public_flat_rw*/;   // transmit_value
    reg [23:0] ssi_rval /*verilator public_flat_rw*/;   // received_value
    reg        wait_tx /*verilator public_flat_rw*/, wait_rx /*verilator public_flat_rw*/;
    reg        hs_frame /*verilator public_flat_rw*/;
    reg        st_srcv /*verilator public_flat_rw*/, st_strx /*verilator public_flat_rw*/;
    // other registers with live behaviour
    reg [23:0] ipr /*verilator public_flat_rw*/, bcr /*verilator public_flat_rw*/;
    reg [23:0] pcddr /*verilator public_flat_rw*/, pcd /*verilator public_flat_rw*/;
    reg [23:0] scr /*verilator public_flat_rw*/, ssr /*verilator public_flat_rw*/, sccr /*verilator public_flat_rw*/;
    reg [127:0] pvalid /*verilator public_flat_rw*/;  // backing store entries written since reset

    assign hreq   = isr[7];
    assign ivr    = ivr_r;
    assign int_host_rcv = hsr[0];
    assign int_host_trx = hsr[1];
    assign int_host_cmd = hsr[2];
    assign int_ssi_rcv  = st_srcv;
    assign int_ssi_trx  = st_strx;
    assign msk_host_rcv = hcr[0];
    assign msk_host_trx = hcr[1];
    assign msk_host_cmd = hcr[2];
    assign msk_ssi_rcv  = crb[15];
    assign msk_ssi_trx  = crb[14];
    assign ipr_out      = ipr;
    assign hc_vector    = cvr[4:0];

    // SSI configuration (dsp_core_ssi_configure)
    wire [1:0] wl_sel  = cra[14:13];
    wire       crb_shfd = crb[6];
    wire       crb_mode = crb[11];
    wire       crb_te   = crb[12];
    wire       crb_re   = crb[13];

    // Hatari's bit swap loop: low wl bits reversed, then shifted left by 1
    function automatic [24:0] swap_wl(input [23:0] v, input [1:0] w);
        integer k;
        reg [24:0] t;
        begin
            t = 25'd0;
            case (w)
                2'd0: for (k = 0; k < 8;  k = k + 1) t[8 - k]  = v[k];
                2'd1: for (k = 0; k < 12; k = k + 1) t[12 - k] = v[k];
                2'd2: for (k = 0; k < 16; k = k + 1) t[16 - k] = v[k];
                default: for (k = 0; k < 24; k = k + 1) t[24 - k] = v[k];
            endcase
            swap_wl = t;
        end
    endfunction

    // transmit value for the current TX register (dsp_core_ssi_Receive_SCK)
    reg [24:0] tx_shift;
    always @* begin
        case (wl_sel)
            2'd0: tx_shift = {17'd0, ssi_tx[23:16]};
            2'd1: tx_shift = {13'd0, ssi_tx[23:12]};
            2'd2: tx_shift = {9'd0, ssi_tx[23:8]};
            default: tx_shift = {1'b0, ssi_tx};
        endcase
        if (crb_shfd) tx_shift = swap_wl(tx_shift[23:0], wl_sel);
    end
    // SC2 for this slot clears the frame wait in network mode
    wire       tx_wait_now = wait_tx && !(crb_mode && ssi_frame);
    wire       tx_go       = crb_te && !tx_wait_now;
    assign ssi_tx_data = tx_go ? tx_shift[15:0] : 16'd0;

    // received value conversion (dsp_core_ssi_Receive_SC0)
    function automatic [24:0] rx_conv(input [23:0] v, input [1:0] w, input sh);
        reg [23:0] t;
        begin
            case (w)
                2'd0: t = {v[7:0], 16'd0};
                2'd1: t = {v[11:0], 12'd0};
                2'd2: t = {v[15:0], 8'd0};
                default: t = v;
            endcase
            rx_conv = sh ? swap_wl(t, w) : {1'b0, t};
        end
    endfunction

    // ------------------------------------------------------------------
    // backing store for plain peripheral locations (X and Y, 64 each)
    // ------------------------------------------------------------------
    function automatic x_special(input [5:0] a);
        case (a)
            6'h23, 6'h25, 6'h28, 6'h29, 6'h2b, 6'h2c, 6'h2d, 6'h2e, 6'h2f,
            6'h30, 6'h31, 6'h32, 6'h3e, 6'h3f: x_special = 1'b1;
            default: x_special = 1'b0;
        endcase
    endfunction

    wire        pst_we_a = perx_wr && !x_special(perx_addr);
    wire        pst_we_b = pery_wr;
    wire [23:0] pst_q_a, pst_q_b;
    dsp_ram_tdp #(.AW(7), .DW(24)) u_pstore (
        .clk(clk),
        .we_a(pst_we_a), .addr_a({1'b0, perx_addr}), .d_a(perx_wdata), .q_a(pst_q_a),
        .we_b(pst_we_b), .addr_b({1'b1, pery_addr}), .d_b(pery_wdata), .q_b(pst_q_b));

    reg        rx_spec_q, rx_valid_q, ry_valid_q;
    reg [23:0] rx_val_q;
    assign perx_rdata = rx_spec_q ? rx_val_q : (rx_valid_q ? pst_q_a : 24'd0);
    assign pery_rdata = ry_valid_q ? pst_q_b : 24'd0;

    // ------------------------------------------------------------------
    // 68030 bus
    // ------------------------------------------------------------------
    function automatic [7:0] host_byte(input [2:0] a);
        case (a)
            3'd0: host_byte = icr;
            3'd1: host_byte = hc_ack ? (cvr & 8'h7f) : cvr;   // acknowledge first (Hatari order)
            3'd2: host_byte = isr;
            3'd3: host_byte = ivr_r;
            3'd4: host_byte = 8'h00;
            3'd5: host_byte = rxh;
            3'd6: host_byte = rxm;
            default: host_byte = rxl;
        endcase
    endfunction
    assign bus_dout = {host_byte({bus_addr, 1'b0}), host_byte({bus_addr, 1'b1})};
    assign bus_ack  = bus_cs && bus_stb;

    // ------------------------------------------------------------------
    // sequential behaviour, written as Hatari's functions on working copies
    // ------------------------------------------------------------------
    reg  [7:0] v_icr, v_cvr, v_isr, v_ivr, v_rxh, v_rxm, v_rxl, v_txh, v_txm, v_txl;
    reg  [7:0] v_hsr;
    reg  [4:0] v_hcr;
    reg [23:0] v_htx, v_rtx;
    reg        v_running, v_run_start;
    reg  [9:0] v_boot_pos;
    reg        v_boot_we;
    reg  [8:0] v_boot_addr;
    reg [23:0] v_boot_data;
    reg [23:0] v_cra, v_crb, v_tx, v_rx, v_ipr, v_bcr, v_pcddr, v_pcd, v_scr, v_ssr, v_sccr;
    reg  [7:0] v_ssisr;
    reg [24:0] v_tval;
    reg [23:0] v_rval;
    reg        v_wait_tx, v_wait_rx, v_hs_frame, v_srcv, v_strx, v_txvalid, v_hsplay;

    task automatic upd_trdy;
        v_isr[2] = v_isr[1] & ~v_hsr[0];
    endtask

    task automatic upd_hreq;
        v_isr[7] = ((v_icr & v_isr & 8'h03) != 8'h00);
    endtask

    task automatic dsp2host;
        if (!v_isr[0] && !v_hsr[1]) begin
            v_rxl = v_htx[7:0]; v_rxm = v_htx[15:8]; v_rxh = v_htx[23:16];
            v_hsr[1] = 1'b1;
            v_isr[0] = 1'b1;
            upd_hreq;
        end
    endtask

    task automatic host2dsp;
        if (!v_isr[1] && !v_hsr[0]) begin
            v_rtx = {v_txh, v_txm, v_txl};
            v_hsr[0] = 1'b1;
            v_isr[1] = 1'b1;
            upd_hreq;
            upd_trdy;
        end
    endtask

    task automatic write_host(input [2:0] a, input [7:0] d);
        case (a)
            3'd0: begin
                v_icr = d & 8'hfb;
                v_hsr[4:3] = v_icr[4:3];
                if (v_icr[7]) begin
                    if (v_icr[0]) begin v_isr[0] = 1'b0; v_hsr[1] = 1'b1; end
                    if (v_icr[1]) begin v_isr[1] = 1'b1; v_hsr[0] = 1'b0; end
                    v_icr[7] = 1'b0;
                end
                if (!v_running && v_icr[3]) begin
                    v_running = 1'b1; v_run_start = 1'b1;
                end
                upd_hreq;
            end
            3'd1: begin
                v_cvr = d & 8'h9f;
                v_hsr[2] = d[7];
            end
            3'd3: v_ivr = d;
            3'd5: v_txh = d;
            3'd6: v_txm = d;
            3'd7: begin
                v_txl = d;
                if (!v_running) begin
                    v_boot_we = 1'b1;
                    v_boot_addr = v_boot_pos[8:0];
                    v_boot_data = {v_txh, v_txm, v_txl};
                    v_boot_pos = v_boot_pos + 10'd1;
                    if (v_boot_pos == 10'h200) begin
                        v_running = 1'b1; v_run_start = 1'b1;
                    end
                end else begin
                    if (v_isr[2]) begin
                        v_rtx = {v_txh, v_txm, v_txl};
                        v_hsr[0] = 1'b1;
                    end else begin
                        v_isr[1] = 1'b0;
                        upd_hreq;
                    end
                    upd_trdy;
                    host2dsp;
                end
            end
            default: ;   // ISR, RX0: read only
        endcase
    endtask

    task automatic read_host(input [2:0] a);
        if (a == 3'd7) begin
            v_isr[0] = 1'b0;
            dsp2host;
            upd_hreq;
        end
    endtask

    reg [24:0] t25;
    reg [23:0] rdv;
    reg        rspec;
    always @* begin
        v_icr = icr; v_cvr = cvr; v_isr = isr; v_ivr = ivr_r;
        v_rxh = rxh; v_rxm = rxm; v_rxl = rxl; v_txh = txh; v_txm = txm; v_txl = txl;
        v_hsr = hsr; v_hcr = hcr; v_htx = htx; v_rtx = rtx;
        v_running = running; v_run_start = 1'b0; v_boot_pos = boot_pos;
        v_boot_we = 1'b0; v_boot_addr = boot_addr; v_boot_data = boot_data;
        v_cra = cra; v_crb = crb; v_tx = ssi_tx; v_rx = ssi_rx; v_ssisr = ssisr;
        v_tval = ssi_tval; v_rval = ssi_rval; v_wait_tx = wait_tx; v_wait_rx = wait_rx;
        v_hs_frame = hs_frame; v_srcv = st_srcv; v_strx = st_strx;
        v_txvalid = ssi_tx_valid; v_hsplay = 1'b0;
        v_ipr = ipr; v_bcr = bcr; v_pcddr = pcddr; v_pcd = pcd; v_scr = scr; v_ssr = ssr; v_sccr = sccr;
        rdv = 24'd0; rspec = 1'b0; t25 = 25'd0;

        // ---------------- SSI slot (crossbar) ----------------
        if (ssi_slot_stb && ssi_tx_en) begin
            // SC2 (transmit frame sync)
            if (crb_mode) begin
                if (ssi_frame) begin v_ssisr[2] = 1'b1; v_wait_tx = 1'b0; end
                else v_ssisr[2] = 1'b0;
            end else
                v_ssisr[2] = 1'b1;
            // SCK
            if (crb_te && !v_wait_tx) begin
                v_tval = tx_shift;
                v_strx = 1'b1;
            end else
                v_tval = 25'd0;
            v_ssisr[6] = 1'b1;
            // crossbar handshake record consumed the word
            v_txvalid = 1'b0;
        end
        if (ssi_slot_stb && ssi_rx_en) begin
            v_rval = {{8{ssi_rx_data[15]}}, ssi_rx_data};
            // SC1 (receive frame sync)
            if (crb_mode) begin
                if (ssi_rx_frame) begin v_ssisr[3] = 1'b1; v_wait_rx = 1'b0; end
                else v_ssisr[3] = 1'b0;
            end else
                v_ssisr[3] = 1'b1;
            // SC0
            t25 = rx_conv(v_rval, wl_sel, crb_shfd);
            if (crb_re && !v_wait_rx) begin
                v_rx = t25[23:0];
                v_srcv = 1'b1;
            end else
                v_rx = 24'd0;
            v_ssisr[7] = 1'b1;
        end

        // ---------------- host command accepted by the core ----------------
        if (hc_ack) begin
            v_hsr[2] = 1'b0;
            v_cvr[7] = 1'b0;
        end

        // ---------------- DSP side X peripheral read ----------------
        if (perx_rd) begin
            rspec = x_special(perx_addr);
            case (perx_addr)
                6'h23: rdv = v_pcddr;
                6'h25: rdv = v_pcd;
                6'h28: rdv = {19'd0, v_hcr};
                6'h29: rdv = {16'd0, v_hsr};
                6'h2b: begin                           // HRX: dsp_core_hostport_dspread
                    rdv = v_rtx;
                    v_hsr[0] = 1'b0;
                    upd_trdy;
                    host2dsp;
                end
                6'h2c: rdv = v_cra;
                6'h2d: rdv = v_crb;
                6'h2e: rdv = {16'd0, v_ssisr};
                6'h2f: begin                           // RX: dsp_core_ssi_readRX
                    rdv = v_rx;
                    v_ssisr[7] = 1'b0;
                    v_srcv = 1'b0;
                end
                6'h30: rdv = v_scr;
                6'h31: rdv = v_ssr;
                6'h32: rdv = v_sccr;
                6'h3e: rdv = v_bcr;
                6'h3f: rdv = v_ipr;
                default: ;
            endcase
        end

        // ---------------- DSP side X peripheral write ----------------
        if (perx_wr) begin
            case (perx_addr)
                6'h2b: begin                           // HTX: dsp_core_hostport_dspwrite
                    v_htx = perx_wdata;
                    v_hsr[1] = 1'b0;
                    dsp2host;
                end
                6'h28: begin                           // HCR
                    v_hcr = perx_wdata[4:0];
                    v_isr[4:3] = perx_wdata[4:3];
                end
                6'h29: ;                               // HSR: read only
                6'h2c: v_cra = perx_wdata;
                6'h2d: begin
                    if (!v_crb[12] && perx_wdata[12]) v_wait_tx = 1'b1;
                    if (!v_crb[13] && perx_wdata[13]) v_wait_rx = 1'b1;
                    v_crb = perx_wdata;
                end
                6'h2e: begin                           // TSR: dsp_core_ssi_writeTSR
                    v_ssisr[6] = 1'b0;
                    v_strx = 1'b0;
                end
                6'h2f: begin                           // TX: dsp_core_ssi_writeTX
                    v_ssisr[6] = 1'b0;
                    v_strx = 1'b0;
                    v_tx = perx_wdata;
                    if (v_hs_frame) v_txvalid = 1'b1;  // SC2(1) to the crossbar
                end
                6'h3f: v_ipr = perx_wdata;
                6'h3e: v_bcr = perx_wdata;
                6'h23: v_pcddr = perx_wdata;
                6'h25: begin                           // dsp_core_setPortCDataRegister
                    v_pcd = perx_wdata;
                    if (v_pcddr[4] && perx_wdata[4]) begin
                        v_wait_rx = 1'b0;
                        v_hsplay = 1'b1;               // SC1 -> Crossbar_DmaPlayInHandShakeMode
                    end
                    if (v_pcddr[5]) begin
                        if (perx_wdata[5]) begin
                            v_hs_frame = 1'b1;
                            v_wait_tx = 1'b0;
                        end else begin
                            v_hs_frame = 1'b0;
                            v_txvalid = 1'b0;          // SC2(0) to the crossbar
                        end
                    end
                end
                6'h30: v_scr = perx_wdata;
                6'h31: v_ssr = perx_wdata;
                6'h32: v_sccr = perx_wdata;
                default: ;                             // backing store
            endcase
        end

        // ---------------- RESET instruction (dsp_reset) ----------------
        if (soft_reset) begin
            v_ipr = 24'd0;
            v_hcr = 5'd0; v_isr[4:3] = 2'b00;
            write_host(3'd0, 8'h00);
            write_host(3'd1, 8'h12);
            v_isr = 8'h06;
            v_ivr = 8'h0f;
            v_cra = 24'd0;
            v_crb = 24'd0;
            v_ssisr[6] = 1'b0; v_strx = 1'b0;          // write to TSR
            v_scr = 24'd0; v_ssr = 24'd3; v_sccr = 24'd0;
        end

        // ---------------- 68030 bus access ----------------
        if (bus_cs && bus_stb) begin
            if (bus_we) begin
                if (bus_uds) write_host({bus_addr, 1'b0}, bus_din[15:8]);
                if (bus_lds) write_host({bus_addr, 1'b1}, bus_din[7:0]);
            end else begin
                if (bus_uds) read_host({bus_addr, 1'b0});
                if (bus_lds) read_host({bus_addr, 1'b1});
            end
        end

    end

    always @(posedge clk) begin
        icr <= v_icr; cvr <= v_cvr; isr <= v_isr; ivr_r <= v_ivr;
        rxh <= v_rxh; rxm <= v_rxm; rxl <= v_rxl; txh <= v_txh; txm <= v_txm; txl <= v_txl;
        hsr <= v_hsr; hcr <= v_hcr; htx <= v_htx; rtx <= v_rtx;
        running <= v_running; run_start <= v_run_start; boot_pos <= v_boot_pos;
        boot_we <= v_boot_we; boot_addr <= v_boot_addr; boot_data <= v_boot_data;
        cra <= v_cra; crb <= v_crb; ssi_tx <= v_tx; ssi_rx <= v_rx; ssisr <= v_ssisr;
        ssi_tval <= v_tval; ssi_rval <= v_rval; wait_tx <= v_wait_tx; wait_rx <= v_wait_rx;
        hs_frame <= v_hs_frame; st_srcv <= v_srcv; st_strx <= v_strx;
        ssi_tx_valid <= v_txvalid; ssi_hs_play_req <= v_hsplay;
        ipr <= v_ipr; bcr <= v_bcr; pcddr <= v_pcddr; pcd <= v_pcd;
        scr <= v_scr; ssr <= v_ssr; sccr <= v_sccr;

        rx_spec_q <= rspec;
        rx_val_q  <= rdv;
        rx_valid_q <= pvalid[{1'b0, perx_addr}];
        ry_valid_q <= pvalid[{1'b1, pery_addr}];
        if (pst_we_a) pvalid[{1'b0, perx_addr}] <= 1'b1;
        if (pst_we_b) pvalid[{1'b1, pery_addr}] <= 1'b1;

        if (rst) begin
            icr <= 8'h00; cvr <= 8'h12; isr <= 8'h06; ivr_r <= 8'h0f;
            hsr <= 8'h02; hcr <= 5'd0; htx <= 24'd0; rtx <= 24'd0;
            running <= 1'b0; run_start <= 1'b0; boot_pos <= 10'd0; boot_we <= 1'b0;
            cra <= 24'd0; crb <= 24'd0; ssisr <= 8'h40;
            ssi_tx <= 24'd0; ssi_rx <= 24'd0;
            wait_tx <= 1'b1; wait_rx <= 1'b1; hs_frame <= 1'b0;
            st_srcv <= 1'b0; st_strx <= 1'b0;
            ssi_tx_valid <= 1'b0; ssi_hs_play_req <= 1'b0;
            ipr <= 24'd0; bcr <= 24'h00ffff; pcddr <= 24'd0; pcd <= 24'd0;
            scr <= 24'd0; ssr <= 24'd0; sccr <= 24'd0;
            pvalid <= 128'd0;
        end
    end
endmodule
