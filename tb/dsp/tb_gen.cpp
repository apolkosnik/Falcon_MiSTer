// Falcon DSP testbench - encoders self-check and random generator.
// Plain ASCII.
#include "tb_gen.h"
#include "gen/opcodes8h.inc"

const char *hatari_class(uint32_t w) {
    if (w >= 0x100000) {
        switch ((w >> 20) & 15) {
        case 1: return "dsp_pm_1";
        case 2: case 3: return "dsp_pm_2";
        case 4: return ((w & 0xf40000) == 0x400000) ? "dsp_pm_4x" : "dsp_pm_5";
        case 5: case 6: case 7: return "dsp_pm_5";
        default: return "dsp_pm_8";
        }
    }
    uint32_t v = ((w >> 11) & (63 << 3)) + ((w >> 5) & 7);
    return hatari_opcodes8h[v];
}

const char *hatari_alu(uint32_t w) { return hatari_opcodes_alu[w & 0xff]; }

bool insn_is_hw_deviation(uint32_t w) {
    return w == 0x000006 || w == 0x000086 || w == 0x000087;
}

static int check1(uint32_t w, const char *want) {
    const char *got = hatari_class(w);
    if (strcmp(got, want) != 0) {
        printf("FAIL: encoder self-check: %06x dispatches to %s, expected %s\n", w, got, want);
        return 1;
    }
    return 0;
}

int enc_selfcheck() {
    using namespace enc;
    int e = 0;
    e += check1(andi(1, 0xfe), "dsp_andi");
    e += check1(ori(0, 0x03), "dsp_ori");
    e += check1(div_(3, 1), "dsp_div");
    e += check1(norm(5, 1), "dsp_norm");
    e += check1(tcc(7, 1, true, 3, 4), "dsp_tcc");
    e += check1(tcc(7, 15, false), "dsp_tcc");
    e += check1(lua(0x1f, true, 7), "dsp_lua");
    e += check1(movec_imm(0xff, 0x3f), "dsp_movec_imm");
    e += check1(movec_reg(true, 0x3f, 0x39), "dsp_movec_reg");
    e += check1(movec_reg(false, 0x04, 0x20), "dsp_movec_reg");
    e += check1(movec_aa(true, 1, 0x3f, 0x3e), "dsp_movec_aa");
    e += check1(movec_ea(false, 1, 0x3f, 0x27), "dsp_movec_ea");
    e += check1(do_imm(0xfff), "dsp_do_imm");
    e += check1(do_aa(1, 0x3f), "dsp_do_aa");
    e += check1(do_ea(1, 0x2f), "dsp_do_ea");
    e += check1(do_reg(0x3f), "dsp_do_reg");
    e += check1(rep_imm(0xfff), "dsp_rep_imm");
    e += check1(rep_aa(1, 0x3f), "dsp_rep_aa");
    e += check1(rep_ea(1, 0x2f), "dsp_rep_ea");
    e += check1(rep_reg(0x3f), "dsp_rep_reg");
    e += check1(movem_aa(true, 0x3f, 0x3f), "dsp_movem_aa");
    e += check1(movem_aa(false, 0x3f, 0x04), "dsp_movem_aa");
    e += check1(movem_ea(true, 0x3f, 0x3f), "dsp_movem_ea");
    e += check1(movem_ea(false, 0x30, 0x04), "dsp_movem_ea");
    e += check1(movep_reg(true, 1, 0x3f, 0x3f), "dsp_movep_0");
    e += check1(movep_reg(false, 0, 0x04, 0x00), "dsp_movep_0");
    e += check1(movep_p(true, 1, 0x30, 0x3f), "dsp_movep_1");
    e += check1(movep_xy(false, 1, 0x3f, 1, 0x3f), "dsp_movep_23");
    e += check1(movep_xy(true, 0, 0x34, 0, 0x00), "dsp_movep_23");
    const char *bn[4][4] = {{"dsp_bclr_aa", "dsp_bclr_ea", "dsp_bclr_pp", "dsp_bclr_reg"},
                            {"dsp_bset_aa", "dsp_bset_ea", "dsp_bset_pp", "dsp_bset_reg"},
                            {"dsp_bchg_aa", "dsp_bchg_ea", "dsp_bchg_pp", "dsp_bchg_reg"},
                            {"dsp_btst_aa", "dsp_btst_ea", "dsp_btst_pp", "dsp_btst_reg"}};
    const char *jn[4][4] = {{"dsp_jclr_aa", "dsp_jclr_ea", "dsp_jclr_pp", "dsp_jclr_reg"},
                            {"dsp_jset_aa", "dsp_jset_ea", "dsp_jset_pp", "dsp_jset_reg"},
                            {"dsp_jsclr_aa", "dsp_jsclr_ea", "dsp_jsclr_pp", "dsp_jsclr_reg"},
                            {"dsp_jsset_aa", "dsp_jsset_ea", "dsp_jsset_pp", "dsp_jsset_reg"}};
    for (int op = 0; op < 4; op++)
        for (int sp = 0; sp < 2; sp++) {
            e += check1(bitop_aa(op, sp, 0x3f, 23), bn[op][0]);
            e += check1(bitop_ea(op, sp, 0x3f, 31), bn[op][1]);
            e += check1(bitop_pp(op, sp, 0x3f, 0), bn[op][2]);
            e += check1(bitop_reg(op, 0x3f, 17), bn[op][3]);
            e += check1(jbit_aa(op, sp, 0x3f, 23), jn[op][0]);
            e += check1(jbit_ea(op, sp, 0x3f, 31), jn[op][1]);
            e += check1(jbit_pp(op, sp, 0x3f, 0), jn[op][2]);
            e += check1(jbit_reg(op, 0x3f, 17), jn[op][3]);
        }
    e += check1(jmp(0xfff), "dsp_jmp_imm");
    e += check1(jsr(0xfff), "dsp_jsr_imm");
    e += check1(jcc(15, 0xfff), "dsp_jcc_imm");
    e += check1(jscc(15, 0xfff), "dsp_jscc_imm");
    e += check1(jmp_ea(0x3f), "dsp_jmp_ea");
    e += check1(jcc_ea(15, 0x3f), "dsp_jcc_ea");
    e += check1(jsr_ea(0x3f), "dsp_jsr_ea");
    e += check1(jscc_ea(15, 0x3f), "dsp_jscc_ea");
    for (int a = 0; a < 2; a++)
        for (int s = 0; s < 2; s++) e += check1(pm0(a, s, 0x3f, 0xff), "dsp_pm_0");
    e += check1(rupd(0x1f, 0x10), "dsp_pm_2");
    e += check1(rr(31, 4, 0x10), "dsp_pm_2");
    e += check1(imm8(4, 0xff, 0x10), "dsp_pm_2");
    e += check1(imm8(31, 0xff, 0x10), "dsp_pm_2");
    for (int d = 4; d < 32; d++) {
        e += check1(xy(1, true, d, true, 0x3f, 0), "dsp_pm_5");
        e += check1(xy(0, false, d, false, 0x3f, 0), "dsp_pm_5");
    }
    for (int l = 0; l < 8; l++) e += check1(lmove(l, true, true, 0x3f, 0), "dsp_pm_4x");
    e += check1(pm1(0xfff, 0x10), "dsp_pm_1");
    e += check1(xymove(true, 3, 3, 7, true, 3, 3, 3, 0xff), "dsp_pm_8");
    e += check1(xymove(false, 0, 0, 0, false, 0, 0, 0, 0x00), "dsp_pm_8");
    if (strcmp(hatari_alu(0x04), "dsp_undefined") != 0) { printf("FAIL: alu table\n"); e++; }
    return e;
}

// ---------------------------------------------------------------------------
// generator
// ---------------------------------------------------------------------------
uint16_t Gen::addr_xy() {
    uint32_t p = rng.below(100);
    if (p < (uint32_t)opt.pct_periph) return 0xffc0 + rng.below(64);
    if (p < 55) return rng.below(0x100);
    if (p < 70) return 0x100 + rng.below(0x100);
    if (p < 95) return 0x200 + rng.below(0x3e00);
    return rng.below(0xffc0);
}

uint16_t Gen::addr_p() {
    uint32_t p = rng.below(100);
    if (p < 50) return 0x40 + rng.below(0x1c0);
    if (p < 90) return 0x200 + rng.below(0x0e00);
    return rng.below(0x10000);
}

int Gen::pp() {
    static const int useful[] = {0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x28, 0x29, 0x2b, 0x2c,
                                 0x2d, 0x2e, 0x2f, 0x30, 0x31, 0x32, 0x3e, 0x3f};
    if (rng.chance(70)) return useful[rng.below(sizeof(useful) / sizeof(useful[0]))];
    return rng.below(64);
}

int Gen::reg_any() {
    if (rng.chance(opt.pct_control_regs)) {
        static const int c[] = {RSR, ROMR, RSP, RSSH, RSSL, RLA, RLC};
        return c[rng.below(7)];
    }
    if (rng.chance(55)) return 4 + rng.below(12);
    return 0x10 + rng.below(24);
}

int Gen::reg5() { return 4 + rng.below(28); }

int Gen::reg_movec() {
    if (rng.chance(50)) return 0x20 + rng.below(8);
    static const int c[] = {RSR, ROMR, RSP, RSSH, RSSL, RLA, RLC};
    return c[rng.below(7)];
}

uint32_t Gen::alu_op() {
    for (;;) {
        uint32_t a = rng.below(256);
        if (strcmp(hatari_opcodes_alu[a], "dsp_undefined") != 0) return a;
    }
}

int Gen::ea6(bool allow_imm, bool allow_abs, bool &two, uint32_t &w2, bool is_p) {
    int mode;
    uint32_t p = rng.below(100);
    if (p < 15) mode = 0;
    else if (p < 30) mode = 1;
    else if (p < 42) mode = 2;
    else if (p < 60) mode = 3;
    else if (p < 75) mode = 4;
    else if (p < 82) mode = 5;
    else if (p < 90 && (allow_abs || allow_imm)) mode = 6;
    else mode = 7;
    int r = rng.below(8);
    if (mode == 6) {
        two = true;
        if (allow_imm && (!allow_abs || rng.chance(50))) {
            r = 4;
            w2 = data24();
        } else {
            r = 0;
            w2 = is_p ? addr_p() : addr_xy();
        }
    }
    return (mode << 3) | r;
}

int Gen::ea5() { return rng.below(32); }

int Gen::insn(uint16_t pc, uint32_t w[2]) {
    using namespace enc;
    bool two = false;
    uint32_t w2 = 0;
    uint32_t a = alu_op();
    (void)pc;
    if (rng.chance(opt.pct_parallel)) {
        uint32_t k = rng.below(100);
        if (k < 8) w[0] = alu(a);
        else if (k < 13) w[0] = rupd(ea5(), a);
        else if (k < 22) w[0] = rr(reg5(), reg5(), a);
        else if (k < 30) w[0] = imm8(reg5(), rng.u32(), a);
        else if (k < 40) {
            bool to_reg = rng.chance(50);
            bool use_ea = rng.chance(70);
            int e = use_ea ? ea6(false, true, two, w2) : (int)rng.below(64);
            w[0] = lmove(rng.below(8), to_reg, use_ea, e, a);
        } else if (k < 62) {
            bool to_reg = rng.chance(50);
            bool use_ea = rng.chance(75);
            int e = use_ea ? ea6(to_reg, true, two, w2) : (int)rng.below(64);
            w[0] = xy(rng.below(2), to_reg, reg5(), use_ea, e, a);
        } else if (k < 82) {
            w[0] = xymove(rng.chance(50), rng.below(4), rng.below(4), rng.below(8),
                          rng.chance(50), rng.below(4), rng.below(4), rng.below(4), a);
        } else if (k < 90) {
            int e = ea6(false, true, two, w2);
            w[0] = pm0(rng.below(2), rng.below(2), e, a);
        } else {
            bool to_reg = rng.chance(50);
            int e = ea6(to_reg, true, two, w2);
            uint32_t bits = (rng.below(16) << 8) | (to_reg ? 0x80 : 0) | (rng.below(2) << 6) | e;
            w[0] = pm1(bits, a);
        }
    } else {
        uint32_t k = rng.below(64);
        int sp = rng.below(2);
        int bit = rng.chance(90) ? (int)rng.below(24) : (int)(24 + rng.below(8));
        switch (k) {
        case 0: w[0] = nop(); break;
        case 1: w[0] = andi(rng.below(3), rng.u32() | (rng.chance(80) ? 0xa0 : 0)); break;
        case 2: {
            uint32_t v = rng.u32() & 0xff;
            int d = rng.below(3);
            if (d == 0 && rng.chance(85)) v &= ~0xa0u;     // T and LF rarely
            w[0] = ori(d, v);
            break;
        }
        case 3: case 4: w[0] = div_(rng.below(4), rng.below(2)); break;
        case 5: w[0] = norm(rng.below(8), rng.below(2)); break;
        case 6: {
            static const int idx[] = {0, 1, 8, 9, 10, 11, 12, 13, 14, 15};
            w[0] = tcc(rng.below(16), idx[rng.below(10)], rng.chance(50), rng.below(8), rng.below(8));
            break;
        }
        case 7: w[0] = lua(ea5(), rng.chance(50), rng.below(8)); break;
        case 8: w[0] = movec_imm(rng.u32(), reg_movec()); break;
        case 9: w[0] = movec_reg(rng.chance(50), reg_any(), reg_movec()); break;
        case 10: w[0] = movec_aa(rng.chance(50), sp, rng.below(64), reg_movec()); break;
        case 11: {
            bool to_c = rng.chance(50);
            w[0] = movec_ea(to_c, sp, ea6(to_c, true, two, w2), reg_movec());
            break;
        }
        case 12: case 13: {   // DO with a short loop body following
            int body = 1 + rng.below(6);
            uint32_t sel = rng.below(10);
            if (sel < 6) w[0] = do_imm(1 + rng.below(rng.chance(80) ? 4 : 40));
            else if (sel < 7) w[0] = do_aa(sp, rng.below(64));
            else if (sel < 8) { int e; do { e = ea6(false, false, two, w2); } while ((e >> 3) == 6); w[0] = do_ea(sp, e); }
            else w[0] = do_reg(reg_any());
            two = true;
            w2 = (uint16_t)(pc + 1 + body);
            break;
        }
        case 14: case 15: {
            uint32_t sel = rng.below(10);
            if (sel < 6) w[0] = rep_imm(rng.below(rng.chance(80) ? 5 : 60));
            else if (sel < 7) w[0] = rep_aa(sp, rng.below(64));
            else if (sel < 8) { int e; do { e = ea6(false, false, two, w2); } while ((e >> 3) == 6); w[0] = rep_ea(sp, e); }
            else w[0] = rep_reg(reg_any());
            break;
        }
        case 16: w[0] = enddo(); break;
        case 17: w[0] = movem_aa(rng.chance(50), rng.below(64), reg_any()); break;
        case 18: w[0] = movem_ea(rng.chance(50), ea6(false, true, two, w2, true), reg_any()); break;
        case 19: case 20: w[0] = movep_reg(rng.chance(50), sp, reg_any(), pp()); break;
        case 21: w[0] = movep_p(rng.chance(50), sp, ea6(false, true, two, w2, true), pp()); break;
        case 22: case 23: {
            bool to_pp = rng.chance(50);
            w[0] = movep_xy(to_pp, sp, ea6(to_pp, true, two, w2), rng.below(2), pp());
            break;
        }
        case 24: case 25: w[0] = bitop_aa(rng.below(4), sp, rng.below(64), bit); break;
        case 26: case 27: w[0] = bitop_ea(rng.below(4), sp, ea6(false, true, two, w2), bit); break;
        case 28: case 29: w[0] = bitop_pp(rng.below(4), sp, pp(), bit); break;
        case 30: case 31: w[0] = bitop_reg(rng.below(4), reg_any(), bit); break;
        case 32: case 33: case 34: case 35: {
            int op = rng.below(4);
            uint32_t sel = rng.below(4);
            if (sel == 0) w[0] = jbit_aa(op, sp, rng.below(64), bit);
            else if (sel == 1) { int e; do { e = ea6(false, true, two, w2); } while ((e >> 3) == 6); w[0] = jbit_ea(op, sp, e, bit); }
            else if (sel == 2) w[0] = jbit_pp(op, sp, pp(), bit);
            else w[0] = jbit_reg(op, reg_any(), bit);
            two = true;
            w2 = addr_p();
            break;
        }
        case 36: w[0] = jmp(addr_p()); break;
        case 37: w[0] = jsr(addr_p()); break;
        case 38: case 39: w[0] = jcc(rng.below(16), addr_p()); break;
        case 40: w[0] = jscc(rng.below(16), addr_p()); break;
        case 41: w[0] = jmp_ea(ea6(false, true, two, w2, true)); if (two) w2 = addr_p(); break;
        case 42: w[0] = jcc_ea(rng.below(16), ea6(false, true, two, w2, true)); if (two) w2 = addr_p(); break;
        case 43: w[0] = jsr_ea(ea6(false, true, two, w2, true)); if (two) w2 = addr_p(); break;
        case 44: w[0] = jscc_ea(rng.below(16), ea6(false, true, two, w2, true)); if (two) w2 = addr_p(); break;
        case 45: w[0] = rts(); break;
        case 46: w[0] = rti(); break;
        case 47: w[0] = rng.chance(30) ? illegal() : nop(); break;
        case 48: w[0] = (opt.allow_reset && rng.chance(20)) ? reset() : nop(); break;
        default: {
            // more parallel move variety
            bool to_reg = rng.chance(50);
            w[0] = xy(rng.below(2), to_reg, reg5(), true, ea6(to_reg, true, two, w2), a);
            break;
        }
        }
    }
    if (two) { w[1] = w2 & 0xffffff; return 2; }
    return 1;
}

void Gen::fill_program(std::vector<uint32_t> &p, uint16_t from, uint16_t to) {
    uint32_t a = from;
    while (a < to) {
        uint32_t w[2];
        int n = insn((uint16_t)a, w);
        p[a & 0xffff] = w[0];
        if (n == 2 && a + 1 < to) p[(a + 1) & 0xffff] = w[1];
        else if (n == 2) p[a & 0xffff] = 0;   // do not leave a cut 2-word instruction
        a += n;
    }
}

uint32_t Gen::data24() {
    static const uint32_t edge[] = {0x000000, 0x000001, 0x7fffff, 0x800000, 0x800001, 0xffffff,
                                    0x400000, 0xc00000, 0x3fffff, 0xbfffff, 0x000002, 0xfffffe};
    if (rng.chance(25)) return edge[rng.below(sizeof(edge) / sizeof(edge[0]))];
    return rng.u32() & 0xffffff;
}

void Gen::random_state(DspState &s) {
    memset(&s, 0, sizeof(s));
    s.x0 = data24(); s.x1 = data24(); s.y0 = data24(); s.y1 = data24();
    s.a0 = data24(); s.a1 = data24(); s.b0 = data24(); s.b1 = data24();
    static const uint32_t e8[] = {0x00, 0xff, 0x7f, 0x80, 0x01, 0xfe};
    s.a2 = rng.chance(60) ? ((s.a1 & 0x800000) ? 0xff : 0) : (rng.chance(50) ? e8[rng.below(6)] : (rng.u32() & 0xff));
    s.b2 = rng.chance(60) ? ((s.b1 & 0x800000) ? 0xff : 0) : (rng.chance(50) ? e8[rng.below(6)] : (rng.u32() & 0xff));
    for (int i = 0; i < 8; i++) {
        s.r[i] = addr_xy();
        uint32_t p = rng.below(100);
        if (p < 70) s.n[i] = rng.below(9) - (rng.chance(30) ? 9 : 0);
        else s.n[i] = rng.u32();
        s.n[i] &= 0xffff;
        p = rng.below(100);
        if (p < 65) s.m[i] = 0xffff;
        else if (p < 80) s.m[i] = 1 + rng.below(rng.chance(80) ? 40 : 0x7fff);
        else if (p < 90) { s.m[i] = 0; if (s.n[i] == 0) s.n[i] = 1 << rng.below(8); }
        else if (p < 95) s.m[i] = 0x8000 + rng.below(0x7fff);
        else s.m[i] = rng.u32() & 0xffff;
    }
    s.sr = rng.u32() & 0x0f7f;
    if (rng.chance(4)) s.sr |= 0x2000;
    if (rng.chance(80)) s.sr &= ~0x0300u;   // interrupt mask 0 most of the time
    uint32_t p = rng.below(100);
    s.omr = p < 80 ? 0x02 : (p < 90 ? 0x00 : (rng.u32() & 0xc7));
    s.sp = rng.below(4);
    for (int i = 1; i < 16; i++) { s.ssh[i] = rng.u32() & 0xffff; s.ssl[i] = rng.u32() & 0xffff; }
    if (rng.chance(30)) for (int i = 1; i < 16; i++) s.ssh[i] = 0x40 + rng.below(0x1c0);
    s.la = rng.u32() & 0xffff;
    s.lc = 1 + rng.below(100);
    s.lcsave = 0;
    s.pc = 0x40 + rng.below(0x100);
}

bool tb_alu_defined(int i) { return strcmp(hatari_opcodes_alu[i & 255], "dsp_undefined") != 0; }
