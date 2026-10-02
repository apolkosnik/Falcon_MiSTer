// Falcon DSP testbench - 68030 host port tests (through the register bus,
// in lockstep with Hatari's dsp_core_read_host / dsp_core_write_host) and a
// free-running SSI test with crossbar slot timing. Plain ASCII.
#include "tb_prog.h"
#include "tb_tests.h"
#include <cstdarg>

using namespace enc;

static std::string fmt(const char *f, ...) {
    char b[512];
    va_list ap;
    va_start(ap, f);
    vsnprintf(b, sizeof b, f, ap);
    va_end(ap);
    return b;
}

static int check(const char *what, uint32_t got, uint32_t exp) {
    if (got != exp) {
        printf("  CHECK FAILED %s: expected %06x, got %06x\n", what, exp, got);
        return 1;
    }
    return 0;
}

// host access helpers inside a lockstep run: one 68030 byte access per DSP
// instruction boundary
struct HostScript {
    Lockstep &ls;
    uint64_t mis = 0, insns = 0;
    explicit HostScript(Lockstep &l) : ls(l) {}
    void idle(int n) { for (int i = 0; i < n; i++) { int r = ls.step(); insns++; if (r) mis++; } }
    uint8_t rd(int off) {
        HostEv h; h.valid = true; h.we = false; h.off = off;
        uint16_t v = 0;
        int r = ls.step(&h, nullptr, &v);
        insns++; if (r) mis++;
        return (off & 1) ? (v & 0xff) : (v >> 8);
    }
    uint16_t rdw(int off) {
        HostEv h; h.valid = true; h.we = false; h.off = off; h.word = true;
        uint16_t v = 0;
        int r = ls.step(&h, nullptr, &v);
        insns++; if (r) mis++;
        return v;
    }
    void wr(int off, uint8_t d) {
        HostEv h; h.valid = true; h.we = true; h.off = off;
        h.data = (off & 1) ? d : (uint16_t)(d << 8);
        int r = ls.step(&h);
        insns++; if (r) mis++;
    }
    void wrw(int off, uint16_t d) {
        HostEv h; h.valid = true; h.we = true; h.off = off; h.word = true; h.data = d;
        int r = ls.step(&h);
        insns++; if (r) mis++;
    }
    bool poll(int off, uint8_t mask, uint8_t val, int max) {
        for (int i = 0; i < max; i++) { if ((rd(off) & mask) == val) return true; idle(1); }
        return false;
    }
};

// echo program: X0 = 1; loop { wait HRDF; A = HRX; A += X0; wait HTDE; HTX = A }
static void echo_program(std::vector<uint32_t> &w, uint16_t base) {
    w.clear();
    auto put = [&](uint32_t v) { w.push_back(v & 0xffffff); };
    put(movei(RX0)); put(1);
    uint16_t l1 = base + 2;
    put(jbit_pp(0, 0, 0x29, 0)); put(l1);           // jclr #0,x:$ffe9,l1
    put(movep_reg(false, 0, RA, 0x2b));             // movep x:$ffeb,a
    put(alu(0x40));                                 // add x0,a
    uint16_t l2 = base + (uint16_t)w.size();
    put(jbit_pp(0, 0, 0x29, 1)); put(l2);           // jclr #1,x:$ffe9,l2
    put(movep_reg(true, 0, RA, 0x2b));              // movep a,x:$ffeb
    put(jmp(l1));
}

// ===========================================================================
// bootstrap upload of 512 words, then data exchange both directions
// ===========================================================================
static void test_bootstrap_echo(Rtl &rtl, Stats &st) {
    int e = 0;
    Vfalcon_dsp *top = rtl.top;
    rtl.reset();
    golden_reset();
    std::vector<uint32_t> prog;
    echo_program(prog, 0);
    Rng rng(5);
    std::vector<uint32_t> words(512);
    for (int i = 0; i < 512; i++) words[i] = i < (int)prog.size() ? prog[i] : (rng.u32() & 0x00ff00);
    // upload: TXH, TXM, TXL per word through the bus (Hatari bootstrap)
    for (int i = 0; i < 512; i++) {
        host_write_both(rtl, 5, words[i] >> 16);
        host_write_both(rtl, 6, (words[i] >> 8) & 0xff);
        host_write_both(rtl, 7, words[i] & 0xff);
        if (i == 510) {
            e += check("not running before the 512th word", RP(running), 0);
        }
    }
    rtl.eval0(); rtl.clock(); rtl.eval0();
    e += check("running after the 512th word", RP(running), 1);
    e += check("Hatari running after the 512th word", dsp_core.running ? 1 : 0, 1);
    for (int i = 0; i < 512; i++) e += check("bootstrap word in P RAM", RC(u_pint__DOT__mem)[i], words[i]);
    for (int i = 0; i < 3 && rtl.state() != 1; i++) { rtl.eval0(); rtl.clock(); }
    rtl.eval0();
    e += check("R0 = 512 after the bootstrap", RC(rr)[0], 0x200);
    e += check("OMR = 2 after the bootstrap", RC(omr), 2);
    DspState g, r;
    golden_read_state(g); rtl.read_state(r);
    int d = compare_state(g, r, "after bootstrap");
    e += d;
    d = compare_memories(rtl, "after bootstrap");
    e += d;

    // data exchange: CPU writes a word, the DSP adds 1 and sends it back
    Lockstep ls(rtl, st);
    HostScript hs(ls);
    for (int k = 0; k < 20; k++) {
        uint32_t v = rng.u32() & 0x3fffff;
        if (!hs.poll(2, 0x02, 0x02, 40)) { e += check("TXDE set before writing", 0, 1); break; }
        hs.wr(5, v >> 16); hs.wr(6, v >> 8); hs.wr(7, v);
        if (!hs.poll(2, 0x01, 0x01, 60)) { e += check("RXDF set by the DSP answer", 0, 1); break; }
        uint32_t back = hs.rd(5) << 16;
        back |= hs.rd(6) << 8;
        back |= hs.rd(7);
        e += check("echoed word = written + 1", back, (v + 1) & 0xffffff);
        e += check("RXDF cleared after reading RXL", dsp_core.hostport[CPU_HOST_ISR] & 1, 0);
    }
    // word accesses: move.w to $FFA206 (TXM, TXL) and from $FFA206
    {
        uint32_t v = 0x112233;
        hs.poll(2, 0x02, 0x02, 40);
        hs.wr(5, v >> 16);
        hs.wrw(6, v & 0xffff);
        hs.poll(2, 0x01, 0x01, 60);
        uint32_t back = hs.rd(5) << 16;
        back |= hs.rdw(6);
        e += check("word access echo", back, v + 1);
    }
    report("host port: 512-word bootstrap, data exchange TXDE/RXDF vs Hatari", hs.mis == 0 && e == 0,
           fmt("(%llu insns, %llu mismatching, %d check failures)", (unsigned long long)hs.insns,
               (unsigned long long)hs.mis, e));
}

// ===========================================================================
// bootstrap stopped early by ICR.HF0, host command handshake, HREQ/IVR
// ===========================================================================
static void test_host_cmd_hreq(Rtl &rtl, Stats &st) {
    int e = 0;
    Vfalcon_dsp *top = rtl.top;
    rtl.reset();
    golden_reset();
    // short bootstrap: main loop + host command handler at vector $18 (P:$30)
    std::vector<uint32_t> w(0x40, 0);
    uint16_t pc = 0;
    w[pc++] = movep_xy(true, 0, EA_IMM, 0, 0x3f); w[pc++] = 0x0800;   // IPR host level 2
    w[pc++] = movep_xy(true, 0, EA_IMM, 0, 0x28); w[pc++] = 0x04;     // HCR: HCIE
    w[pc++] = jmp(pc);                                                // loop
    w[0x30] = movep_xy(true, 0, EA_IMM, 0, 0x2b); w[0x31] = 0xabcdef; // fast: movep #,x:htx
    for (int i = 0; i < 0x32; i++) {
        host_write_both(rtl, 5, w[i] >> 16);
        host_write_both(rtl, 6, (w[i] >> 8) & 0xff);
        host_write_both(rtl, 7, w[i] & 0xff);
    }
    e += check("not running during a short bootstrap", RP(running), 0);
    host_write_both(rtl, 0, 0x08);                  // ICR.HF0: start
    rtl.eval0(); rtl.clock(); rtl.eval0();
    e += check("running after ICR.HF0", RP(running), 1);
    e += check("Hatari running after ICR.HF0", dsp_core.running ? 1 : 0, 1);
    for (int i = 0; i < 3 && rtl.state() != 1; i++) { rtl.eval0(); rtl.clock(); }
    rtl.eval0();
    e += check("R0 = words uploaded", RC(rr)[0], 0x32);
    e += check("HSR.HF0 follows ICR.HF0", RP(hsr) & 0x08, 0x08);

    Lockstep ls(rtl, st);
    HostScript hs(ls);
    hs.wr(0, 0x00);                                  // ICR = 0
    hs.idle(4);
    // IVR
    hs.wr(3, 0x45);
    e += check("ivr output after writing IVR", top->ivr, 0x45);
    e += check("IVR read back", hs.rd(3), 0x45);
    // host command $18 -> handler sends $abcdef
    hs.wr(1, 0x98);
    e += check("CVR.HC set by the write", RP(cvr) & 0x80, 0x80);
    bool hc_done = hs.poll(1, 0x80, 0x00, 40);
    e += check("CVR.HC cleared when the DSP takes the host command", hc_done, 1);
    // HREQ on RXDF with ICR.RREQ
    e += check("hreq low with ICR = 0", top->hreq, 0);
    hs.wr(0, 0x01);                                  // RREQ
    bool rx = hs.poll(2, 0x01, 0x01, 40);
    e += check("RXDF set by the host command handler", rx, 1);
    e += check("hreq asserted (RREQ and RXDF)", top->hreq, 1);
    e += check("ISR.HREQ set", RP(isr) >> 7, 1);
    uint32_t v = hs.rd(5) << 16; v |= hs.rd(6) << 8; v |= hs.rd(7);
    e += check("word sent by the host command handler", v, 0xabcdef);
    e += check("hreq released after reading RXL", top->hreq, 0);
    // TREQ: TXDE is set -> HREQ
    hs.wr(0, 0x02);
    e += check("hreq asserted (TREQ and TXDE)", top->hreq, 1);
    // HF2/HF3 from the DSP's HCR into ISR; INIT
    hs.wr(0, 0x00);
    hs.wr(5, 0x01); hs.wr(6, 0x02); hs.wr(7, 0x03);  // DSP does not read: HRDF stays set
    hs.idle(4);
    e += check("HSR.HRDF set by the host write", RP(hsr) & 1, 1);
    hs.wr(0, 0x80 | 0x02);                           // INIT with TREQ: TXDE=1, HRDF=0
    e += check("INIT: HRDF cleared", RP(hsr) & 1, 0);
    e += check("INIT: ICR.INIT self-clears", RP(icr) & 0x80, 0);
    report("host port: HF0 start, host command, HREQ/IVR, INIT vs Hatari", hs.mis == 0 && e == 0,
           fmt("(%llu insns, %llu mismatching, %d check failures)", (unsigned long long)hs.insns,
               (unsigned long long)hs.mis, e));
}

// ===========================================================================
// SSI with crossbar slot timing (free running, asynchronous to the DSP
// instruction stream): the DSP negates every received 16-bit sample and
// transmits it; the crossbar side checks the transmitted stream
// ===========================================================================
static void test_ssi_crossbar(Rtl &rtl) {
    Vfalcon_dsp *top = rtl.top;
    int e = 0;
    Prog pg;
    pg.org(0x40);
    pg.movep_imm(0x2c, 0x4100);                  // CRA: 16-bit words, 2 per frame
    pg.movep_imm(0x2d, 0x3800);                  // CRB: RE TE, network mode
    pg.label("loop");
    pg.w2l(jbit_pp(0, 0, 0x2e, 7), "loop");      // jclr #7,x:$ffee,loop (RDF)
    pg.w(movep_reg(false, 0, RA, 0x2f));         // movep x:$ffef,a
    pg.w(alu(0x36));                             // neg a
    pg.label("wt");
    pg.w2l(jbit_pp(0, 0, 0x2e, 6), "wt");        // jclr #6,x:$ffee,wt (TDE)
    pg.w(movep_reg(true, 0, RA, 0x2f));          // movep a,x:$ffef
    pg.wl12(jmp(0), "loop");
    pg.finish();
    Stats st;
    Lockstep ls(rtl, st);
    DspState r;
    memset(&r, 0, sizeof r);
    for (int i = 0; i < 8; i++) r.m[i] = 0xffff;
    r.omr = 2; r.pc = 0x40;
    ls.start(pg.p, pg.xint, pg.yint, pg.ext, &r);
    // crossbar: one slot every 64 clocks (2 tracks), frame on the left slot
    const int N = 200;
    std::vector<uint16_t> sent(N), got(N);
    Rng rng(9);
    for (int k = 0; k < N; k++) {
        sent[k] = (uint16_t)rng.u32();
        if (sent[k] == 0x8000) sent[k] = 0x8001;
        int gap = 64 + (int)rng.below(5);        // jitter: phase is arbitrary
        for (int c = 0; c < gap; c++) { rtl.eval0(); rtl.clock(); }
        top->ssi_slot_stb = 1; top->ssi_tx_en = 1; top->ssi_rx_en = 1;
        top->ssi_frame = (k % 2) == 0; top->ssi_rx_frame = (k % 2) == 0;
        top->ssi_rx_data = sent[k];
        rtl.eval0();
        got[k] = top->ssi_tx_data;
        rtl.clock();
        rtl.idle_inputs();
        rtl.eval0();
    }
    // the first frame sync releases the transmitter and receiver; from then
    // on slot k transmits the negation of the word received in slot k-1
    int bad = 0;
    for (int k = 4; k < N; k++) {
        uint16_t exp = (uint16_t)(0 - sent[k - 1]);
        if (got[k] != exp) {
            if (bad < 5) printf("  CHECK FAILED SSI slot %d: transmitted %04x, expected -%04x = %04x\n",
                                k, got[k], sent[k - 1], exp);
            bad++;
        }
    }
    e += bad;
    e += check("SSI receive overrun flag never set by Hatari model", RP(ssisr) & 0x20, 0);
    report("SSI with crossbar slot timing (free running DSP program)", e == 0,
           fmt("(%d slots, %d wrong words)", N, bad));
}

void run_host_tests(Rtl &rtl, const TestConfig &cfg, Stats &st) {
    auto want = [&](const char *n) { return cfg.only.empty() || std::string(n).find(cfg.only) != std::string::npos; };
    if (want("host")) test_bootstrap_echo(rtl, st);
    if (want("host")) test_host_cmd_hreq(rtl, st);
    if (want("ssi")) test_ssi_crossbar(rtl);
}
