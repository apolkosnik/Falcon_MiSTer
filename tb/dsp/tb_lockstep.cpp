// Falcon DSP testbench - lockstep differential engine. Plain ASCII.
#include "tb_lockstep.h"


void Lockstep::start(const std::vector<uint32_t> &p, const uint32_t *xint, const uint32_t *yint,
                     const std::vector<uint32_t> &ext, const DspState *regs) {
    Vfalcon_dsp *top = rtl.top;
    rtl.reset();
    golden_reset();
    // state that neither Hatari's dsp_core_reset nor the DSP reset clears
    // (power-up leftovers): give the golden model the RTL's values
    dsp_core.ssi.transmit_value = RP(ssi_tval);
    dsp_core.ssi.received_value = RP(ssi_rval);
    dsp_core.hostport[CPU_HOST_RXH] = RP(rxh); dsp_core.hostport[CPU_HOST_RXM] = RP(rxm);
    dsp_core.hostport[CPU_HOST_RXL] = RP(rxl);
    dsp_core.hostport[CPU_HOST_TXH] = RP(txh); dsp_core.hostport[CPU_HOST_TXM] = RP(txm);
    dsp_core.hostport[CPU_HOST_TXL] = RP(txl);
    dsp_core.interrupt_IplToRaise = RC(int_ipl);
    dsp_core.pc_on_rep = RC(pc_on_rep);
    golden_hs_record_frame = 0;
    hs_play_pulses = 0;
    // memories (identical backdoor load into both models)
    for (int i = 0; i < 32768; i++) {
        uint32_t v = ext[i] & 0xffffff;
        rtl.ext_write(i, v);
        dsp_core.ramext[i] = v;
    }
    for (int i = 0; i < 0x200; i++) {
        uint32_t v = p[i] & 0xffffff;
        RC(u_pint__DOT__mem)[i] = v;
        dsp_core.ramint[DSP_SPACE_P][i] = v;
    }
    for (int i = 0; i < 256; i++) {
        rtl.xyint_write(0, i, xint[i]);
        rtl.xyint_write(1, i, yint[i]);
        dsp_core.ramint[DSP_SPACE_X][i] = xint[i] & 0xffffff;
        dsp_core.ramint[DSP_SPACE_Y][i] = yint[i] & 0xffffff;
    }
    rtl.eval0();
    // start execution like the 68030 does after (part of) a bootstrap: ICR.HF0
    host_write_both(rtl, 0, 0x08);
    host_write_both(rtl, 0, 0x00);
    // wait for the fetch state, then set the registers in both models
    for (int i = 0; i < 10 && rtl.state() != 1; i++) { rtl.eval0(); rtl.clock(); }
    if (rtl.state() != 1) printf("FAIL: DSP did not start after ICR.HF0 (expected fetch state)\n");
    if (regs) {
        rtl.set_regs(*regs);
        golden_set_regs(*regs);
    }
    first = true;
}

int Lockstep::step(const HostEv *h, const SsiEv *s, uint16_t *host_rd, uint16_t *ssi_tx) {
    Vfalcon_dsp *top = rtl.top;
    uint32_t w = golden_pread(dsp_core.pc);
    if (insn_is_hw_deviation(w)) { st.stop_hw_insn++; return -1; }
    {
        std::string c = hatari_class(w);
        st.classes[c]++;
        if (w >= 0x100000 || c == "dsp_pm_0") st.alu_ops[hatari_alu(w)]++;
    }

    // ---------------- golden ----------------
    uint32_t prev_state = dsp_core.interrupt_state;
    uint32_t prev_crb = dsp_core.periph[DSP_SPACE_X][DSP_SSI_CRB];
    dsp56k_execute_instruction();
    // documented deviation: enabling TE / RE re-arms the wait for the next
    // frame sync (hardware); Hatari's CRB write overwrites the old value
    // before dsp_core_ssi_configure() compares it, so it never re-arms
    {
        uint32_t crb = dsp_core.periph[DSP_SPACE_X][DSP_SSI_CRB];
        bool te_up = !(prev_crb & (1 << DSP_SSI_CRB_TE)) && (crb & (1 << DSP_SSI_CRB_TE));
        bool re_up = !(prev_crb & (1 << DSP_SSI_CRB_RE)) && (crb & (1 << DSP_SSI_CRB_RE));
        if (te_up) dsp_core.ssi.waitFrameTX = 1;
        if (re_up) dsp_core.ssi.waitFrameRX = 1;
        if (te_up || re_up) st.dev_crb_rearm++;
    }
    uint32_t gcyc = dsp_core.instr_cycle;
    if (dsp_core.interrupt_state == DSP_INTERRUPT_DISABLED && dsp_core.interrupt_pipeline_count == 5)
        st.interrupts++;
    if (dsp_core.interrupt_state == DSP_INTERRUPT_LONG && prev_state != DSP_INTERRUPT_LONG)
        st.long_interrupts++;
    // documented deviation: Hatari's bit-reverse update with modifier 0 sets
    // bit 16 of Rn; the RTL leaves Rn unchanged (low 16 bits identical)
    bool dev_bitrev = false;
    for (int i = 0; i < 8; i++)
        if (dsp_core.registers[DSP_REG_R0 + i] > 0xffff) {
            dev_bitrev = true;
            dsp_core.registers[DSP_REG_R0 + i] &= 0xffff;
        }
    if (dev_bitrev) st.dev_bitrev0++;
    // events after the instruction
    uint16_t g_tx = 0, g_rd = 0;
    if (s && s->valid) {
        st.ssi_events++;
        if (s->tx_en) {
            dsp_core_ssi_Receive_SC2(s->frame ? 1 : 0);
            dsp_core_ssi_Receive_SCK();
            g_tx = dsp_core.ssi.transmit_value & 0xffff;
            golden_hs_record_frame = 0;
        }
        if (s->rx_en) {
            uint16_t d = s->loopback ? g_tx : s->rx_data;
            dsp_core.ssi.received_value = (uint32_t)(int32_t)(int16_t)d & 0xffffff;
            dsp_core_ssi_Receive_SC1(s->rx_frame ? 1 : 0);
            dsp_core_ssi_Receive_SC0();
            // documented deviation: Hatari's SHFD bit swap can leave a 25th
            // bit in ssi.RX (a value no 24-bit register can hold); the RTL
            // keeps the low 24 bits
            if (dsp_core.ssi.RX > 0xffffff) { dsp_core.ssi.RX &= 0xffffff; st.dev_rx25++; }
        }
    }
    if (h && h->valid) {
        st.host_events++;
        int o = h->off;
        if (h->we) {
            if (h->word) { dsp_core_write_host(o, h->data >> 8); dsp_core_write_host(o + 1, h->data & 0xff); }
            else dsp_core_write_host(o, (o & 1) ? (h->data & 0xff) : (h->data >> 8));
        } else {
            if (h->word) { uint8_t hi = dsp_core_read_host(o); uint8_t lo = dsp_core_read_host(o + 1); g_rd = (hi << 8) | lo; }
            else { uint8_t b = dsp_core_read_host(o); g_rd = (o & 1) ? b : (uint16_t)(b << 8); }
        }
    }

    // ---------------- RTL ----------------
    int clocks = 0;
    uint32_t rcyc = 0;
    uint16_t r_tx = 0, r_rd = 0;
    for (;;) {
        rtl.eval0();
        bool el = rtl.is_el();
        if (el) {
            rcyc = rtl.ncyc();
            if (s && s->valid) {
                top->ssi_slot_stb = 1; top->ssi_tx_en = s->tx_en; top->ssi_rx_en = s->rx_en;
                top->ssi_frame = s->frame; top->ssi_rx_frame = s->rx_frame; top->ssi_rx_data = s->rx_data;
                rtl.eval0();
                if (s->loopback) { top->ssi_rx_data = top->ssi_tx_data; rtl.eval0(); }
                r_tx = top->ssi_tx_data;
            }
            if (h && h->valid) {
                int o = h->off;
                top->bus_cs = 1; top->bus_stb = 1; top->bus_we = h->we; top->bus_addr = (o >> 1) & 3;
                top->bus_uds = h->word || !(o & 1); top->bus_lds = h->word || (o & 1);
                top->bus_din = h->data;
                rtl.eval0();
                if (!top->bus_ack) printf("  FAIL: bus_ack missing on the bus_stb clock\n");
                r_rd = top->bus_dout;
                if (!h->word) r_rd = (o & 1) ? (r_rd & 0xff) : (r_rd & 0xff00);
            }
        }
        rtl.clock();
        clocks++;
        if (top->ssi_hs_play_req) hs_play_pulses++;
        if (el) {
            rtl.idle_inputs();
            rtl.eval0();
            break;
        }
        if (clocks > 400) {
            printf("  FAIL: RTL did not complete the instruction at P:%04x (%06x) within 400 clocks\n",
                   dsp_core.pc, w);
            return -2;
        }
    }

    // ---------------- compare ----------------
    st.insns++;
    char ctx[96];
    snprintf(ctx, sizeof ctx, "insn #%llu %06x (%s)", (unsigned long long)st.insns, w, hatari_class(w));
    bool v = verbose && reported < max_report;
    int n = 0;
    if (rcyc != gcyc) {
        st.cyc_mismatch++;
        if (v) printf("  MISMATCH %s: cycles expected (Hatari) %u, RTL %u\n", ctx, gcyc, rcyc);
        n++;
    } else st.cyc_match++;
    if (!first && (uint32_t)clocks != rcyc) {
        if (v) printf("  MISMATCH %s: instruction took %d clocks, ncyc says %u\n", ctx, clocks, rcyc);
        n++;
    }
    first = false;
    DspState gs, rs;
    golden_read_state(gs);
    rtl.read_state(rs);
    n += compare_state(gs, rs, ctx, v);
    n += compare_memories(rtl, ctx, v);
    if (s && s->valid && s->tx_en && g_tx != r_tx) {
        if (v) printf("  MISMATCH %s: SSI transmitted word expected (Hatari) %04x, RTL %04x\n", ctx, g_tx, r_tx);
        n++;
    }
    if (h && h->valid && !h->we && g_rd != r_rd) {
        if (v) printf("  MISMATCH %s: host read at $FFA20%d expected (Hatari) %04x, RTL %04x\n", ctx, h->off, g_rd, r_rd);
        n++;
    }
    if ((int)top->ssi_tx_valid != golden_hs_record_frame) {
        if (v) printf("  MISMATCH %s: ssi_tx_valid expected (Hatari handshake frame) %d, RTL %d\n",
                      ctx, golden_hs_record_frame, top->ssi_tx_valid);
        n++;
    }
    if (hs_play_pulses != golden_sc1_calls) {
        if (v) printf("  MISMATCH %s: ssi_hs_play_req pulses expected (Hatari SC1 calls) %d, RTL %d\n",
                      ctx, golden_sc1_calls, hs_play_pulses);
        n++;
    }
    if (host_rd) *host_rd = r_rd;
    if (ssi_tx) *ssi_tx = r_tx;
    if (n) {
        st.mismatching_insns++;
        if (v) {
            reported++;
            printf("  (golden PC after: %04x, RTL PC %04x, SR %04x/%04x)\n", gs.pc, rs.pc, gs.sr, rs.sr);
        }
    }
    return n;
}
