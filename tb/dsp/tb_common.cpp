// Falcon DSP testbench - harness implementation. Plain ASCII.
#include "tb_common.h"

// ---------------------------------------------------------------------------
// golden model glue
// ---------------------------------------------------------------------------
int g_golden_hreq = 0;
static void golden_host_irq(int set) { g_golden_hreq = set ? 1 : 0; }

void golden_init_once() {
    static bool done = false;
    if (!done) {
        dsp_core_init(golden_host_irq);
        done = true;
    }
}

void golden_reset() {
    golden_init_once();
    dsp_core_reset();
    golden_sc1_calls = 0;
    golden_sc2_calls = 0;
    golden_sc2_last = 0;
}

uint32_t golden_pread(uint16_t a) {
    if (a < 0x200) return dsp_core.ramint[DSP_SPACE_P][a];
    return dsp_core.ramext[a & 0x7fff];
}

void golden_pwrite(uint16_t a, uint32_t v) {
    if (a < 0x200) dsp_core.ramint[DSP_SPACE_P][a] = v & 0xffffff;
    else dsp_core.ramext[a & 0x7fff] = v & 0xffffff;
}

void golden_read_state(DspState &s) {
    memset(&s, 0, sizeof(s));
    uint32_t *R = dsp_core.registers;
    s.x0 = R[DSP_REG_X0]; s.x1 = R[DSP_REG_X1]; s.y0 = R[DSP_REG_Y0]; s.y1 = R[DSP_REG_Y1];
    s.a0 = R[DSP_REG_A0]; s.a1 = R[DSP_REG_A1]; s.a2 = R[DSP_REG_A2];
    s.b0 = R[DSP_REG_B0]; s.b1 = R[DSP_REG_B1]; s.b2 = R[DSP_REG_B2];
    for (int i = 0; i < 8; i++) {
        s.r[i] = R[DSP_REG_R0 + i]; s.n[i] = R[DSP_REG_N0 + i]; s.m[i] = R[DSP_REG_M0 + i];
    }
    s.sr = R[DSP_REG_SR]; s.omr = R[DSP_REG_OMR]; s.sp = R[DSP_REG_SP];
    s.la = R[DSP_REG_LA]; s.lc = R[DSP_REG_LC]; s.lcsave = R[DSP_REG_LCSAVE];
    s.pc = dsp_core.pc;
    for (int i = 0; i < 16; i++) { s.ssh[i] = dsp_core.stack[0][i]; s.ssl[i] = dsp_core.stack[1][i]; }
    s.loop_rep = dsp_core.loop_rep; s.pc_on_rep = dsp_core.pc_on_rep;
    s.int_state = dsp_core.interrupt_state; s.int_cnt = dsp_core.interrupt_pipeline_count;
    s.int_fetch = (uint16_t)dsp_core.interrupt_instr_fetch;
    s.int_save_pc = (uint16_t)dsp_core.interrupt_save_pc;
    s.int_ipl = dsp_core.interrupt_IplToRaise;
    s.agu_reg = dsp_core.agu_pipeline_reg[0]; s.agu_val = dsp_core.agu_pipeline_val[0];
    s.int_status = dsp_core.interrupt_status;
    s.icr = dsp_core.hostport[CPU_HOST_ICR]; s.cvr = dsp_core.hostport[CPU_HOST_CVR];
    s.isr = dsp_core.hostport[CPU_HOST_ISR]; s.ivr = dsp_core.hostport[CPU_HOST_IVR];
    s.rxh = dsp_core.hostport[CPU_HOST_RXH]; s.rxm = dsp_core.hostport[CPU_HOST_RXM];
    s.rxl = dsp_core.hostport[CPU_HOST_RXL];
    s.txh = dsp_core.hostport[CPU_HOST_TXH]; s.txm = dsp_core.hostport[CPU_HOST_TXM];
    s.txl = dsp_core.hostport[CPU_HOST_TXL];
    s.htx = dsp_core.dsp_host_htx; s.rtx = dsp_core.dsp_host_rtx;
    s.running = dsp_core.running ? 1 : 0; s.boot_pos = dsp_core.bootstrap_pos;
    s.hreq = g_golden_hreq;
    s.ssi_tx = dsp_core.ssi.TX; s.ssi_rx = dsp_core.ssi.RX;
    s.ssi_tval = dsp_core.ssi.transmit_value; s.ssi_rval = dsp_core.ssi.received_value;
    s.wait_tx = dsp_core.ssi.waitFrameTX; s.wait_rx = dsp_core.ssi.waitFrameRX;
    s.hs_frame = dsp_core.ssi.dspPlay_handshakeMode_frame;
    for (int i = 0; i < 64; i++) {
        s.perx[i] = dsp_core.periph[DSP_SPACE_X][i];
        s.pery[i] = dsp_core.periph[DSP_SPACE_Y][i];
    }
    // never stored by Hatari (intercepted registers)
    s.perx[0x2b] = 0; s.perx[0x2f] = 0;
}

void golden_set_regs(const DspState &s) {
    uint32_t *R = dsp_core.registers;
    R[DSP_REG_X0] = s.x0; R[DSP_REG_X1] = s.x1; R[DSP_REG_Y0] = s.y0; R[DSP_REG_Y1] = s.y1;
    R[DSP_REG_A0] = s.a0; R[DSP_REG_A1] = s.a1; R[DSP_REG_A2] = s.a2;
    R[DSP_REG_B0] = s.b0; R[DSP_REG_B1] = s.b1; R[DSP_REG_B2] = s.b2;
    for (int i = 0; i < 8; i++) {
        R[DSP_REG_R0 + i] = s.r[i]; R[DSP_REG_N0 + i] = s.n[i]; R[DSP_REG_M0 + i] = s.m[i];
    }
    R[DSP_REG_SR] = s.sr; R[DSP_REG_OMR] = s.omr; R[DSP_REG_SP] = s.sp;
    R[DSP_REG_LA] = s.la; R[DSP_REG_LC] = s.lc; R[DSP_REG_LCSAVE] = s.lcsave;
    for (int i = 0; i < 16; i++) { dsp_core.stack[0][i] = s.ssh[i]; dsp_core.stack[1][i] = s.ssl[i]; }
    R[DSP_REG_SSH] = s.ssh[s.sp & 15];
    R[DSP_REG_SSL] = s.ssl[s.sp & 15];
    dsp_core.pc = s.pc;
}

// ---------------------------------------------------------------------------
// RTL harness
// ---------------------------------------------------------------------------
Rtl::Rtl() : clocks(0) {
    top = new Vfalcon_dsp;
    idle_inputs();
    eval0();
}

Rtl::~Rtl() { top->final(); delete top; }

void Rtl::idle_inputs() {
    top->reset = 0; top->dsp_reset = 0;
    top->bus_cs = 0; top->bus_stb = 0; top->bus_we = 0; top->bus_addr = 0;
    top->bus_uds = 0; top->bus_lds = 0; top->bus_din = 0; top->iack = 0;
    top->ssi_slot_stb = 0; top->ssi_frame = 0; top->ssi_rx_data = 0;
    top->ssi_tx_en = 0; top->ssi_rx_en = 0; top->ssi_rx_frame = 0;
}

void Rtl::reset(int cycles) {
    idle_inputs();
    top->dsp_reset = 1;
    for (int i = 0; i < cycles; i++) { eval0(); clock(); }
    top->dsp_reset = 0;
    eval0();
}

bool Rtl::is_el() const {
    auto *rp = top->rootp;
    return rp->falcon_dsp__DOT__u_core__DOT__state == 2 &&
           rp->falcon_dsp__DOT__u_core__DOT__cyc != 0 &&
           rp->falcon_dsp__DOT__u_core__DOT__cyc == (uint32_t)(rp->falcon_dsp__DOT__u_core__DOT__ncyc - 1);
}

static inline uint32_t bit128(const VlWide<4> &w, int b) { return (w[b >> 5] >> (b & 31)) & 1; }

void Rtl::read_state(DspState &s) {
    memset(&s, 0, sizeof(s));
    s.x0 = RC(x0); s.x1 = RC(x1); s.y0 = RC(y0); s.y1 = RC(y1);
    s.a0 = RC(a0); s.a1 = RC(a1); s.a2 = RC(a2); s.b0 = RC(b0); s.b1 = RC(b1); s.b2 = RC(b2);
    for (int i = 0; i < 8; i++) { s.r[i] = RC(rr)[i]; s.n[i] = RC(nn)[i]; s.m[i] = RC(mm)[i]; }
    s.sr = RC(sr); s.omr = RC(omr); s.sp = RC(sp); s.la = RC(la); s.lc = RC(lc); s.lcsave = RC(lcsave);
    s.pc = RC(pc);
    for (int i = 0; i < 16; i++) { s.ssh[i] = RC(stk_h)[i]; s.ssl[i] = RC(stk_l)[i]; }
    s.loop_rep = RC(loop_rep); s.pc_on_rep = RC(pc_on_rep);
    s.int_state = RC(int_state); s.int_cnt = RC(int_cnt); s.int_fetch = RC(int_fetch);
    s.int_save_pc = RC(int_save_pc); s.int_ipl = RC(int_ipl);
    s.agu_reg = RC(agu_reg); s.agu_val = RC(agu_val);
    uint32_t hsr = RP(hsr);
    s.int_status = (RC(st_stkerr) << 1) | (RC(st_trace) << 2) | (RC(st_swi) << 3) |
                   ((uint32_t)RC(st_illegal) << 31) | (RP(st_srcv) << 6) | (RP(st_strx) << 8) |
                   ((hsr & 1) << 16) | (((hsr >> 1) & 1) << 17) | (((hsr >> 2) & 1) << 18);
    s.icr = RP(icr); s.cvr = RP(cvr); s.isr = RP(isr); s.ivr = RP(ivr_r);
    s.rxh = RP(rxh); s.rxm = RP(rxm); s.rxl = RP(rxl);
    s.txh = RP(txh); s.txm = RP(txm); s.txl = RP(txl);
    s.htx = RP(htx); s.rtx = RP(rtx); s.running = RP(running); s.boot_pos = RP(boot_pos);
    s.hreq = top->hreq;
    s.ssi_tx = RP(ssi_tx); s.ssi_rx = RP(ssi_rx); s.ssi_tval = RP(ssi_tval); s.ssi_rval = RP(ssi_rval);
    s.wait_tx = RP(wait_tx); s.wait_rx = RP(wait_rx); s.hs_frame = RP(hs_frame);
    const auto &pv = RP(pvalid);
    const auto &ps = RP(u_pstore__DOT__mem);
    for (int i = 0; i < 64; i++) {
        s.perx[i] = bit128(pv, i) ? ps[i] : 0;
        s.pery[i] = bit128(pv, 64 + i) ? ps[64 + i] : 0;
    }
    s.perx[0x23] = RP(pcddr); s.perx[0x25] = RP(pcd); s.perx[0x28] = RP(hcr); s.perx[0x29] = hsr;
    s.perx[0x2b] = 0; s.perx[0x2c] = RP(cra); s.perx[0x2d] = RP(crb); s.perx[0x2e] = RP(ssisr);
    s.perx[0x2f] = 0; s.perx[0x30] = RP(scr); s.perx[0x31] = RP(ssr); s.perx[0x32] = RP(sccr);
    s.perx[0x3e] = RP(bcr); s.perx[0x3f] = RP(ipr);
}

void Rtl::set_regs(const DspState &s) {
    RC(x0) = s.x0; RC(x1) = s.x1; RC(y0) = s.y0; RC(y1) = s.y1;
    RC(a0) = s.a0; RC(a1) = s.a1; RC(a2) = s.a2; RC(b0) = s.b0; RC(b1) = s.b1; RC(b2) = s.b2;
    for (int i = 0; i < 8; i++) { RC(rr)[i] = s.r[i]; RC(nn)[i] = s.n[i]; RC(mm)[i] = s.m[i]; }
    RC(sr) = s.sr; RC(omr) = s.omr; RC(sp) = s.sp; RC(la) = s.la; RC(lc) = s.lc; RC(lcsave) = s.lcsave;
    for (int i = 0; i < 16; i++) { RC(stk_h)[i] = s.ssh[i]; RC(stk_l)[i] = s.ssl[i]; }
    RC(pc) = s.pc;
    eval0();
}

void Rtl::pwrite(uint16_t a, uint32_t v) {
    if (a < 0x200) RC(u_pint__DOT__mem)[a] = v & 0xffffff;
    else RC(u_ext__DOT__mem)[a & 0x7fff] = v & 0xffffff;
}

uint32_t Rtl::pread(uint16_t a) {
    if (a < 0x200) return RC(u_pint__DOT__mem)[a];
    return RC(u_ext__DOT__mem)[a & 0x7fff];
}

void Rtl::xyint_write(int space, uint8_t a, uint32_t v) {
    if (space == 0) RC(u_xint__DOT__mem)[a] = v & 0xffffff;
    else RC(u_yint__DOT__mem)[a] = v & 0xffffff;
}

void Rtl::ext_write(uint16_t a15, uint32_t v) { RC(u_ext__DOT__mem)[a15 & 0x7fff] = v & 0xffffff; }

uint16_t Rtl::bus_access(bool we, int word_addr, bool uds, bool lds, uint16_t din) {
    top->bus_cs = 1; top->bus_stb = 1; top->bus_we = we; top->bus_addr = word_addr & 3;
    top->bus_uds = uds; top->bus_lds = lds; top->bus_din = din;
    eval0();
    uint16_t d = top->bus_dout;
    if (!top->bus_ack) {
        printf("FAIL: bus_ack not given on the bus_stb clock (expected bus_ack=1)\n");
    }
    clock();
    top->bus_cs = 0; top->bus_stb = 0; top->bus_we = 0; top->bus_uds = 0; top->bus_lds = 0;
    eval0();
    return d;
}

// ---------------------------------------------------------------------------
// comparison
// ---------------------------------------------------------------------------
static int diff1(const char *ctx, const char *name, uint32_t g, uint32_t r, bool verbose) {
    if (g == r) return 0;
    if (verbose) printf("  MISMATCH %s: %s expected (Hatari) %06x, RTL %06x\n", ctx, name, g, r);
    return 1;
}

int compare_state(const DspState &g, const DspState &r, const char *ctx, bool v) {
    int n = 0;
    char nm[32];
#define C1(f) n += diff1(ctx, #f, g.f, r.f, v)
    C1(x0); C1(x1); C1(y0); C1(y1); C1(a0); C1(a1); C1(a2); C1(b0); C1(b1); C1(b2);
    for (int i = 0; i < 8; i++) {
        snprintf(nm, sizeof nm, "r%d", i); n += diff1(ctx, nm, g.r[i], r.r[i], v);
        snprintf(nm, sizeof nm, "n%d", i); n += diff1(ctx, nm, g.n[i], r.n[i], v);
        snprintf(nm, sizeof nm, "m%d", i); n += diff1(ctx, nm, g.m[i], r.m[i], v);
    }
    C1(sr); C1(omr); C1(sp); C1(la); C1(lc); C1(lcsave); C1(pc);
    for (int i = 0; i < 16; i++) {
        snprintf(nm, sizeof nm, "stack_ssh[%d]", i); n += diff1(ctx, nm, g.ssh[i], r.ssh[i], v);
        snprintf(nm, sizeof nm, "stack_ssl[%d]", i); n += diff1(ctx, nm, g.ssl[i], r.ssl[i], v);
    }
    C1(loop_rep); C1(pc_on_rep); C1(int_state); C1(int_cnt); C1(int_fetch); C1(int_save_pc);
    if (g.int_state != 0) C1(int_ipl);
    C1(agu_reg);
    if (g.agu_reg != 0) C1(agu_val);
    C1(int_status);
    C1(icr); C1(cvr); C1(isr); C1(ivr); C1(rxh); C1(rxm); C1(rxl); C1(txh); C1(txm); C1(txl);
    C1(htx); C1(rtx); C1(running); C1(boot_pos); C1(hreq);
    C1(ssi_tx); C1(ssi_rx); C1(ssi_tval); C1(ssi_rval); C1(wait_tx); C1(wait_rx); C1(hs_frame);
    for (int i = 0; i < 64; i++) {
        snprintf(nm, sizeof nm, "X:$ff%02x", 0xc0 + i); n += diff1(ctx, nm, g.perx[i], r.perx[i], v);
        snprintf(nm, sizeof nm, "Y:$ff%02x", 0xc0 + i); n += diff1(ctx, nm, g.pery[i], r.pery[i], v);
    }
#undef C1
    return n;
}

int compare_memories(Rtl &rtl, const char *ctx, bool v) {
    Vfalcon_dsp *top = rtl.top;
    int n = 0;
    const auto &ext = RC(u_ext__DOT__mem);
    const auto &pint = RC(u_pint__DOT__mem);
    const auto &xint = RC(u_xint__DOT__mem);
    const auto &yint = RC(u_yint__DOT__mem);
    if (memcmp(&ext[0], dsp_core.ramext, sizeof(dsp_core.ramext)) != 0) {
        for (int i = 0; i < 32768; i++)
            if (ext[i] != dsp_core.ramext[i]) {
                if (v && n < 8) printf("  MISMATCH %s: ext RAM[%04x] expected (Hatari) %06x, RTL %06x\n",
                                       ctx, i, dsp_core.ramext[i], ext[i]);
                n++;
            }
    }
    for (int i = 0; i < 512; i++)
        if (pint[i] != dsp_core.ramint[DSP_SPACE_P][i]) {
            if (v && n < 8) printf("  MISMATCH %s: P:%03x expected (Hatari) %06x, RTL %06x\n",
                                   ctx, i, dsp_core.ramint[DSP_SPACE_P][i], pint[i]);
            n++;
        }
    for (int i = 0; i < 256; i++) {
        if (xint[i] != dsp_core.ramint[DSP_SPACE_X][i]) {
            if (v && n < 8) printf("  MISMATCH %s: X:%02x expected (Hatari) %06x, RTL %06x\n",
                                   ctx, i, dsp_core.ramint[DSP_SPACE_X][i], xint[i]);
            n++;
        }
        if (yint[i] != dsp_core.ramint[DSP_SPACE_Y][i]) {
            if (v && n < 8) printf("  MISMATCH %s: Y:%02x expected (Hatari) %06x, RTL %06x\n",
                                   ctx, i, dsp_core.ramint[DSP_SPACE_Y][i], yint[i]);
            n++;
        }
    }
    return n;
}

uint8_t host_read_both(Rtl &rtl, int off, int &mismatch) {
    uint16_t d = rtl.bus_access(false, off >> 1, !(off & 1), (off & 1), 0);
    uint8_t rb = (off & 1) ? (d & 0xff) : (d >> 8);
    uint8_t gb = dsp_core_read_host(off);
    if (rb != gb) {
        printf("  MISMATCH host read $FFA20%d: expected (Hatari) %02x, RTL %02x\n", off, gb, rb);
        mismatch++;
    }
    return rb;
}

void host_write_both(Rtl &rtl, int off, uint8_t v) {
    rtl.bus_access(true, off >> 1, !(off & 1), (off & 1), (off & 1) ? v : (uint16_t)(v << 8));
    dsp_core_write_host(off, v);
}
