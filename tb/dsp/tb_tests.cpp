// Falcon DSP testbench - tests. Plain ASCII.
#include "tb_tests.h"

static bool want(const TestConfig &c, const char *name) {
    return c.only.empty() || std::string(name).find(c.only) != std::string::npos;
}

static std::string fmt(const char *f, ...) {
    char b[512];
    va_list ap;
    va_start(ap, f);
    vsnprintf(b, sizeof b, f, ap);
    va_end(ap);
    return b;
}

// ---------------------------------------------------------------------------
// encoders vs Hatari dispatch
// ---------------------------------------------------------------------------
static void test_encoders() {
    int e = enc_selfcheck();
    report("encoder self-check vs Hatari opcodes8h[]", e == 0, fmt("(%d errors)", e));
}

// ---------------------------------------------------------------------------
// state after DSP reset
// ---------------------------------------------------------------------------
static void test_reset_state(Rtl &rtl) {
    rtl.reset();
    golden_reset();
    DspState g, r;
    golden_read_state(g);
    rtl.read_state(r);
    int n = compare_state(g, r, "after reset");
    report("reset state vs Hatari dsp_core_reset", n == 0, fmt("(%d differences)", n));
}

// ---------------------------------------------------------------------------
// random differential test
// ---------------------------------------------------------------------------
static void random_memories(Rng &rng, Gen &gen, std::vector<uint32_t> &p, std::vector<uint32_t> &ext,
                            uint32_t *xint, uint32_t *yint) {
    p.assign(0x10000, 0);
    ext.assign(32768, 0);
    // program space: P:$0000-$01FF internal, external RAM filled with
    // instructions everywhere (data reads see instruction words, executing
    // data never runs into undefined opcodes)
    std::vector<uint32_t> prog(0x10000, 0);
    gen.fill_program(prog, 0x0000, 0x8000);
    for (int i = 0; i < 0x200; i++) p[i] = prog[i];
    for (int i = 0; i < 32768; i++) ext[i] = prog[i];
    for (int i = 0; i < 0x200; i++) ext[i] = gen.data24();             // Y: mirror area data
    for (int i = 0; i < 256; i++) { xint[i] = gen.data24(); yint[i] = gen.data24(); }
    // interrupt vectors: some JSR / JMP (long interrupts), the rest random
    for (int v = 0; v < 0x40; v += 2) {
        uint32_t k = rng.below(100);
        if (k < 25) { p[v] = enc::jsr(0x40 + rng.below(0x1c0)); }
        else if (k < 32) { p[v] = enc::jmp(0x40 + rng.below(0x1c0)); }
        else if (k < 40) { p[v] = enc::nop(); p[v + 1] = enc::jsr(0x40 + rng.below(0x1c0)); }
        else if (k < 45) { p[v] = enc::jsr_ea(enc::EA_ABS); p[v + 1] = 0x40 + rng.below(0x1c0); }
    }
}

// peripheral setup preamble (executed by both models): enables host / SSI
// interrupts at random priority levels so the random streams take them
static uint16_t preamble(Rng &rng, std::vector<uint32_t> &p, uint16_t pc) {
    auto put = [&](uint32_t w0, uint32_t w1) { p[pc++] = w0; p[pc++] = w1 & 0xffffff; };
    if (rng.chance(80)) put(enc::movep_xy(true, 0, enc::EA_IMM, 0, 0x3f), rng.below(0x10000) & 0x3c00);
    if (rng.chance(80)) put(enc::movep_xy(true, 0, enc::EA_IMM, 0, 0x28), rng.below(32));
    if (rng.chance(50)) put(enc::movep_xy(true, 0, enc::EA_IMM, 0, 0x2c), rng.u32() & 0x7fff);
    if (rng.chance(50)) put(enc::movep_xy(true, 0, enc::EA_IMM, 0, 0x2d), rng.u32() & 0xfa7f);
    if (rng.chance(20)) put(enc::movep_xy(true, 0, enc::EA_IMM, 0, 0x23), rng.u32() & 0x3f);
    return pc;
}

static void test_random(Rtl &rtl, const TestConfig &cfg, Stats &st) {
    uint64_t total_mis = 0;
    int runs = 0, early_hw = 0, early_stuck = 0;
    for (int sd = cfg.seed0; sd < cfg.seed0 + cfg.seeds; sd++) {
        Rng rng(sd);
        Gen gen(rng);
        // a third of the seeds also fuzz the 68030 host port and the SSI
        bool fuzz_host = (sd % 3) == 1;
        bool fuzz_ssi = (sd % 3) == 2;
        gen.opt.pct_periph = (sd % 4 == 0) ? 20 : 6;
        std::vector<uint32_t> p, ext;
        uint32_t xint[256], yint[256];
        random_memories(rng, gen, p, ext, xint, yint);
        DspState regs;
        gen.random_state(regs);
        if (p.size() && rng.chance(85)) {
            uint16_t e = preamble(rng, p, regs.pc);
            for (uint16_t i = regs.pc; i < e; i++) if (i >= 0x200) ext[i & 0x7fff] = p[i];
        }
        Lockstep ls(rtl, st);
        ls.verbose = cfg.verbose;
        ls.start(p, xint, yint, ext, &regs);
        // tight-loop detection: few distinct PCs over the last 256 steps
        uint16_t hist[256];
        int hn = 0;
        uint64_t mis = 0;
        runs++;
        for (int i = 0; i < cfg.insns_per_seed; i++) {
            HostEv h; SsiEv s;
            if (fuzz_host && rng.chance(6)) {
                h.valid = true;
                h.we = rng.chance(55);
                static const int offs[] = {0, 1, 3, 5, 6, 7, 7, 7, 2, 4};
                h.off = offs[rng.below(10)];
                h.word = !(h.off & 1) && rng.chance(25);
                uint8_t v = rng.u32();
                if (h.we && h.off == 0) v &= rng.chance(80) ? 0x7b : 0xff;   // INIT rarely
                if (h.we && h.off == 1 && rng.chance(70)) v &= 0x7f;         // host command sometimes
                h.data = h.word ? (uint16_t)rng.u32() : ((h.off & 1) ? v : (uint16_t)(v << 8));
            }
            if (fuzz_ssi && rng.chance(8)) {
                s.valid = true;
                s.tx_en = rng.chance(70);
                s.rx_en = rng.chance(70);
                s.frame = rng.chance(30);
                s.rx_frame = rng.chance(30);
                s.rx_data = rng.u32();
                s.loopback = s.tx_en && s.rx_en && rng.chance(30);
            }
            int r = ls.step(&h, &s);
            if (r == -1) { early_hw++; break; }
            if (r == -2) { mis++; break; }
            if (r > 0) mis++;
            if (mis > 20) break;
            hist[hn & 255] = dsp_core.pc;
            hn++;
            if (!dsp_core.loop_rep && hn >= 256 && (hn & 63) == 0) {
                int distinct = 0;
                uint16_t seen[16];
                for (int k = 0; k < 256 && distinct <= 12; k++) {
                    bool f = false;
                    for (int j = 0; j < distinct; j++) if (seen[j] == hist[k]) { f = true; break; }
                    if (!f) { if (distinct < 16) seen[distinct] = hist[k]; distinct++; }
                }
                if (distinct <= 12) { early_stuck++; break; }
            }
        }
        if (mis) printf("  seed %d: %llu mismatching instructions\n", sd, (unsigned long long)mis);
        total_mis += mis;
    }
    printf("  coverage: %d instruction handlers, %d ALU operations exercised\n",
           (int)st.classes.size(), (int)st.alu_ops.size());
    for (auto &c : st.classes) printf("    %-16s %llu\n", c.first.c_str(), (unsigned long long)c.second);
    int alu_missing = 0;
    for (int i = 0; i < 256; i++) {
        extern bool tb_alu_defined(int);
        if (tb_alu_defined(i) && !st.alu_ops.count(hatari_alu(i))) alu_missing++;
    }
    printf("    ALU handlers never executed: %d\n", alu_missing);
    report("random differential vs Hatari", total_mis == 0 && st.insns > 0,
           fmt("(%d seeds, %llu insns compared, %llu mismatching, cycles %llu/%llu match, "
               "host events %llu, ssi events %llu, interrupts %llu (long %llu), "
               "runs ended at SWI/WAIT/STOP %d, stuck %d; documented-deviation events: bit-reverse-0 %llu, "
               "SSI RX bit 24 %llu, CRB TE/RE re-arm %llu)",
               runs, (unsigned long long)st.insns, (unsigned long long)st.mismatching_insns,
               (unsigned long long)st.cyc_match, (unsigned long long)(st.cyc_match + st.cyc_mismatch),
               (unsigned long long)st.host_events, (unsigned long long)st.ssi_events,
               (unsigned long long)st.interrupts, (unsigned long long)st.long_interrupts,
               early_hw, early_stuck, (unsigned long long)st.dev_bitrev0,
               (unsigned long long)st.dev_rx25, (unsigned long long)st.dev_crb_rearm));
}

void run_directed_tests(Rtl &rtl, const TestConfig &cfg, Stats &st);
void run_host_tests(Rtl &rtl, const TestConfig &cfg, Stats &st);

void run_all_tests(Rtl &rtl, const TestConfig &cfg) {
    if (want(cfg, "enc")) test_encoders();
    if (want(cfg, "reset")) test_reset_state(rtl);
    Stats st;
    Stats dst;
    run_directed_tests(rtl, cfg, dst);
    run_host_tests(rtl, cfg, dst);
    if (want(cfg, "random")) test_random(rtl, cfg, st);
}
