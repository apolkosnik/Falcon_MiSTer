// Falcon DSP testbench - directed tests. Every test runs the real RTL in
// lockstep with Hatari's DSP core (all state compared after every
// instruction) and additionally checks the intended result of the program
// against values derived from the DSP56000 family manual / Hatari.
// The SWI/WAIT/STOP test runs the RTL alone (hardware semantics, Hatari
// treats them as no-ops).  Plain ASCII.
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

RunResult run_lockstep(Rtl &rtl, Stats &st, Prog &pg, const DspState *regs, uint16_t stop_pc,
                       uint64_t max_insns, std::function<void(uint64_t, HostEv &, SsiEv &)> ev,
                       bool verbose) {
    pg.finish();
    Lockstep ls(rtl, st);
    ls.verbose = verbose;
    ls.start(pg.p, pg.xint, pg.yint, pg.ext, regs);
    RunResult rr;
    for (uint64_t i = 0; i < max_insns; i++) {
        if (dsp_core.pc == stop_pc) { rr.reached = true; break; }
        HostEv h; SsiEv s;
        if (ev) ev(i, h, s);
        int r = ls.step(&h, &s);
        rr.insns++;
        if (r < 0) { rr.mismatches++; break; }
        if (r > 0) rr.mismatches++;
    }
    if (dsp_core.pc == stop_pc) rr.reached = true;
    return rr;
}

static DspState base_regs(uint16_t pc) {
    DspState s;
    memset(&s, 0, sizeof s);
    for (int i = 0; i < 8; i++) s.m[i] = 0xffff;
    s.omr = 2;
    s.pc = pc;
    return s;
}

static int check(const char *what, uint32_t got, uint32_t exp) {
    if (got != exp) {
        printf("  CHECK FAILED %s: expected %06x, got %06x\n", what, exp, got);
        return 1;
    }
    return 0;
}

// ===========================================================================
// DO / REP / ENDDO
// ===========================================================================
static void test_loops(Rtl &rtl, Stats &st) {
    Prog pg;
    pg.org(0x40);
    pg.move_imm(RX0, 1);                 // add x0,a adds 1 to A1
    pg.w(alu(0x13));                     // clr a
    pg.w(alu(0x1b));                     // clr b
    // 3 x 4 x rep 5
    pg.w2l(do_imm(3), "o_end");
    pg.w2l(do_imm(4), "i_end");
    pg.w(rep_imm(5));
    pg.w(alu(0x40));                     // add x0,a
    pg.label("i_end"); pg.w(alu(0x48));  // add x0,b
    pg.label("o_end"); pg.w(nop());
    pg.move_imm(RY1, 0);                 // marker
    // ENDDO: leave a 100-iteration loop when B1 reaches 20
    pg.w(alu(0x1b));                     // clr b
    pg.move_imm(RY0, 20);
    pg.w2l(do_imm(100), "e_end");
    pg.w(alu(0x48));                     // add x0,b
    pg.w(alu(0x5d));                     // cmp y0,b
    pg.wl12(jcc(2, 0), "e_skip");        // jne e_skip
    pg.w(enddo());
    pg.wl12(jmp(0), "e_out");
    pg.label("e_skip"); pg.w(nop());
    pg.label("e_end"); pg.w(nop());
    pg.label("e_out");
    pg.w(movec_reg(false, RX1, RLC));                // movec lc,x1 (LC restored by ENDDO)
    // single-instruction DO body, DO with count from a register,
    // REP of a 2-word instruction, REP #0 (65536 times)
    pg.move_imm(RY1, 7);
    pg.w2l(do_reg(RY1), "s_end");
    pg.label("s_end"); pg.w(alu(0x40));               // add x0,a
    pg.w(rep_imm(3));
    pg.w2(xy(0, true, RY0, true, EA_ABS, 0x40), 0x10); // add x0,a  x:$10,y0
    pg.w(alu(0x1b));                                    // clr b
    pg.w(rep_imm(0));
    pg.w(alu(0x48));                                    // add x0,b  (65536 times)
    // 7 nested DO loops (stack depth 14)
    pg.move_imm(RY0, 0);
    for (int k = 0; k < 7; k++) { pg.w2l(do_imm(2), "n" + std::to_string(k)); }
    pg.w(alu(0x50));                                    // add y0,a ... placeholder op
    pg.move_imm(RX1, 0);
    for (int k = 6; k >= 0; k--) { pg.label("n" + std::to_string(k)); pg.w(alu(0x40)); }
    pg.label("done"); pg.w(jmp(0x40 + 0));              // never reached
    pg.xint[0x10] = 0x123456;

    DspState r = base_regs(0x40);
    RunResult rr = run_lockstep(rtl, st, pg, &r, pg.at("done"), 300000);
    int e = 0;
    e += check("loops reached the end", rr.reached, 1);
    // A1: 60 (3*4*5) + 7 (DO reg body) + 3 (REP of 2-word insn) + nested:
    // innermost body executes 2^7 = 128 times, level k body 2^k... sum of
    // 2^1..2^7 = 254 adds of x0
    uint32_t exp_a1 = 60 + 7 + 3 + 254;
    e += check("A1 = loop iteration count", dsp_core.registers[DSP_REG_A1], exp_a1);
    e += check("B1 = 65536 (REP #0) low 24 bits", dsp_core.registers[DSP_REG_B1], 0x010000);
    e += check("SP back to 0", dsp_core.registers[DSP_REG_SP], 0);
    e += check("LF clear", (dsp_core.registers[DSP_REG_SR] >> 15) & 1, 0);
    e += check("X1 = LC after ENDDO (restored 0)", dsp_core.registers[DSP_REG_X1] & 0xffff, 0);
    report("DO/REP nesting, ENDDO, REP #0, DO reg vs Hatari", rr.mismatches == 0 && e == 0,
           fmt("(%llu insns, %llu mismatching, %d check failures)", (unsigned long long)rr.insns,
               (unsigned long long)rr.mismatches, e));
}

// ===========================================================================
// AGU: modulo, bit reverse, all addressing modes, AGU delay slot
// ===========================================================================
static uint16_t mod_next(uint16_t r, int32_t n, uint16_t m) {
    // DSP56000 family manual: buffer of size M+1 at the 2^k boundary
    uint16_t size = m + 1, k = 1;
    while (k < size) k <<= 1;
    uint16_t lo = r & ~(k - 1), hi = lo + m;
    int32_t v = r + n;
    if (v > hi) v -= size;
    else if (v < lo) v += size;
    return v & 0xffff;
}

static uint16_t rev_next(uint16_t r, uint16_t n) {
    // reverse-carry add of n (power of two)
    uint16_t rr = 0, nr = 0;
    for (int i = 0; i < 16; i++) { if (r & (1 << i)) rr |= 1 << (15 - i); if (n & (1 << i)) nr |= 1 << (15 - i); }
    rr += nr;
    uint16_t o = 0;
    for (int i = 0; i < 16; i++) if (rr & (1 << i)) o |= 1 << (15 - i);
    return o;
}

static void test_agu(Rtl &rtl, Stats &st) {
    Prog pg;
    pg.org(0x40);
    pg.movec_imm24(RM0 + 0, 9);          // m0 = 9: modulo 10
    pg.move_imm(RR0 + 0, 0x23);
    pg.move_imm(RN0 + 0, 3);
    pg.move_imm(RR0 + 1, 0x80);          // r1: store pointer X:$80..
    pg.w2l(do_imm(25), "m_end");
    pg.w(xy(0, false, RR0 + 0, true, ea(3, 1), 0));    // move r0,x:(r1)+
    pg.label("m_end"); pg.w(rupd(ea(1, 0), 0));          // (r0)+n0
    // modulo with negative step and -(Rn)
    pg.move_imm(RN0 + 0, 4);
    pg.w2l(do_imm(12), "m2_end");
    pg.w(xy(0, false, RR0 + 0, true, ea(3, 1), 0));    // move r0,x:(r1)+
    pg.label("m2_end"); pg.w(rupd(ea(0, 0), 0));         // (r0)-n0
    pg.w2l(do_imm(12), "m3_end");
    pg.label("m3_end"); pg.w(xy(0, false, RR0 + 0, true, ea(7, 0), 0));   // move r0,x:-(r0)
    // modulo with step equal to the buffer size (16): linear
    pg.move_imm(RN0 + 0, 16);
    pg.w(nop());                                        // AGU delay slot
    pg.w(rupd(ea(1, 0), 0));
    pg.w(xy(0, false, RR0 + 0, true, ea(3, 1), 0));
    // bit reverse: m2 = 0, n2 = 4, r2 = 0x40: 8-point FFT order
    pg.movec_imm24(RM0 + 2, 0);
    pg.move_imm(RN0 + 2, 4);
    pg.move_imm(RR0 + 2, 0x40);
    pg.w2l(do_imm(8), "b_end");
    pg.w(xy(0, false, RR0 + 2, true, ea(3, 1), 0));    // move r2,x:(r1)+
    pg.label("b_end"); pg.w(rupd(ea(1, 2), 0));          // (r2)+n2
    // (Rn+Nn) and -(Rn) reads with a linear buffer, (Rn)-Nn
    pg.move_imm(RR0 + 3, 0x10);
    pg.move_imm(RN0 + 3, 5);
    pg.w(nop());                                        // AGU delay slot
    pg.w(xy(0, true, RX0, true, ea(5, 3), 0));          // move x:(r3+n3),x0
    pg.w(xy(0, true, RX1, true, ea(7, 3), 0));          // move x:-(r3),x1
    pg.w(xy(0, true, RY0, true, ea(0, 3), 0));          // move x:(r3)-n3,y0
    // AGU delay slot: an address register written by a move is not yet
    // used by the next instruction's address calculation (Hatari)
    pg.move_imm(RR0 + 4, 0x30);
    pg.w(imm8(RR0 + 4, 0x38, 0));                       // move #$38,r4
    pg.w(xy(0, true, RY1, true, ea(4, 4), 0));          // move x:(r4),y1 (uses $30)
    pg.w(xy(0, true, RA, true, ea(4, 4), 0));           // move x:(r4),a  (uses $38)
    pg.label("done"); pg.w(nop());
    for (int i = 0; i < 0x40; i++) pg.xint[i] = 0x100000 + i;

    DspState r = base_regs(0x40);
    RunResult rr = run_lockstep(rtl, st, pg, &r, pg.at("done"), 10000);
    int e = 0;
    e += check("AGU program reached the end", rr.reached, 1);
    // expected sequences
    uint16_t a = 0x80, v = 0x23;
    for (int i = 0; i < 25; i++) { e += check("modulo (r0)+n0 sequence", dsp_core.ramint[0][a++], v); v = mod_next(v, 3, 9); }
    for (int i = 0; i < 12; i++) { e += check("modulo (r0)-n0 sequence", dsp_core.ramint[0][a++], v); v = mod_next(v, -4, 9); }
    for (int i = 0; i < 12; i++) v = mod_next(v, -1, 9);
    v = (v + 16) & 0xffff;
    e += check("modulo step = buffer size", dsp_core.ramint[0][a++], v);
    uint16_t b = 0x40;
    for (int i = 0; i < 8; i++) { e += check("bit reverse sequence", dsp_core.ramint[0][a++], b); b = rev_next(b, 4); }
    e += check("x:(r3+n3)", dsp_core.registers[DSP_REG_X0], 0x100000 + 0x15);
    e += check("x:-(r3)", dsp_core.registers[DSP_REG_X1], 0x100000 + 0x0f);
    e += check("x:(r3)-n3", dsp_core.registers[DSP_REG_Y0], 0x100000 + 0x0f);
    e += check("AGU delay slot (old r4)", dsp_core.registers[DSP_REG_Y1], 0x100000 + 0x30);
    e += check("AGU delay slot over (new r4)", dsp_core.registers[DSP_REG_A1], 0x100000 + 0x38);
    report("AGU modulo / bit-reverse / modes / delay slot vs Hatari", rr.mismatches == 0 && e == 0,
           fmt("(%llu insns, %llu mismatching, %d check failures)", (unsigned long long)rr.insns,
               (unsigned long long)rr.mismatches, e));
}

// ===========================================================================
// limiting, scaling, rounding, multiply-accumulate
// ===========================================================================
static void test_limit_scale(Rtl &rtl, Stats &st) {
    Prog pg;
    pg.org(0x40);
    // A = $01:400000:000000 -> move a,x0 limits to $7fffff, sets L
    pg.move_imm(RA, 0x400000);
    pg.move_imm(RA2, 0x01);
    pg.w(rr(RA, RX0, 0));                       // move a,x0
    pg.w(xy(0, false, RA, true, EA_ABS, 0)); pg.w(0x20);   // move a,x:$20
    // B = $fe:000000:... -> limits to $800000
    pg.move_imm(RB, 0x123456);
    pg.move_imm(RB2, 0xfe);
    pg.w(rr(RB, RX1, 0));                       // move b,x1
    pg.w(lmove(4, false, true, EA_ABS, 0)); pg.w(0x21);    // move a,l:$21 (limited L move)
    // clear L, scaling down: move a (no extension) is shifted right
    pg.w(andi(1, 0xbf));                        // andi #$bf,ccr (clear L)
    pg.move_imm(RA, 0x345678);
    pg.move_imm(RA0, 0x800001);
    pg.w(ori(0, 0x04));                         // ori #4,mr: scale down
    pg.w(rr(RA, RY0, 0));                       // move a,y0
    pg.w(alu(0x11));                            // rnd a (scale down rounding)
    pg.w(rr(RA0, RY1, 0));                      // move a0,y1
    pg.w(andi(0, 0xf3));                        // scaling off
    pg.w(ori(0, 0x08));                         // ori #8,mr: scale up
    pg.move_imm(RB, 0x234567);
    pg.move_imm(RB0, 0x800000);
    pg.w(rr(RB, RX0, 0));                       // move b,x0 (shifted left, B0 bit 23 in)
    pg.w(alu(0x19));                            // rnd b (scale up)
    pg.w(lmove(1, false, true, EA_ABS, 0)); pg.w(0x22);    // move b10,l:$22
    pg.w(andi(0, 0xf3));
    // convergent rounding: A0 = $800000 exactly
    pg.move_imm(RA, 0x000005);
    pg.move_imm(RA0, 0x800000);
    pg.w(alu(0x11));                            // rnd a: 5 stays 5 (even rounding of .5 -> even? 5 is odd -> 6)
    pg.w(lmove(0, false, true, EA_ABS, 0)); pg.w(0x23);
    // MPY / MPYR / MAC / MACR with -1.0 * -1.0 (limiting on move)
    pg.move_imm(RX0, 0x800000);
    pg.move_imm(RY0, 0x800000);
    pg.w(alu(0xd0));                            // mpy y0,x0,a  (+1.0 = $00:800000:000000)
    pg.w(rr(RA, RX1, 0));                       // move a,x1 (limited to $7fffff)
    pg.w(alu(0xd6));                            // mac -y0,x0,a
    pg.w(alu(0xd3));                            // macr y0,x0,a
    pg.w(lmove(4, false, true, EA_ABS, 0)); pg.w(0x24);
    pg.w(alu(0xd9));                            // mpyr y0,x0,b
    pg.w(lmove(1, false, true, EA_ABS, 0)); pg.w(0x25);
    pg.label("done"); pg.w(nop());

    DspState r = base_regs(0x40);
    RunResult rr2 = run_lockstep(rtl, st, pg, &r, pg.at("done"), 1000);
    int e = 0;
    e += check("limit program reached the end", rr2.reached, 1);
    e += check("move a,x:$20 limited to $7fffff", dsp_core.ramint[0][0x20], 0x7fffff);
    e += check("l:$21 X part limited A = $7fffff", dsp_core.ramint[0][0x21], 0x7fffff);
    e += check("l:$21 Y part when limited positive = $ffffff", dsp_core.ramint[1][0x21], 0xffffff);
    e += check("L bit set by limiting at the end", (dsp_core.registers[DSP_REG_SR] >> 6) & 1, 1);
    report("limiting / scaling / rounding / MAC vs Hatari", rr2.mismatches == 0 && e == 0,
           fmt("(%llu insns, %llu mismatching, %d check failures)", (unsigned long long)rr2.insns,
               (unsigned long long)rr2.mismatches, e));
}

// ===========================================================================
// DIV (24 step non-restoring division) and NORM
// ===========================================================================
static void test_div_norm(Rtl &rtl, Stats &st) {
    Rng rng(77);
    int e = 0;
    uint64_t insns = 0, mis = 0;
    for (int t = 0; t < 40; t++) {
        uint32_t d = 1 + rng.below(0x3fffff);           // positive divisor
        uint32_t n = rng.below(d);                       // positive dividend < divisor
        if (t == 0) { n = 0x200000; d = 0x400000; }
        Prog pg;
        pg.org(0x40);
        pg.move_imm(RA, n);                              // A = n (A0 = 0)
        pg.move_imm(RX0, d);
        pg.w(andi(1, 0xfe));                             // clear carry
        pg.w(rep_imm(24));
        pg.w(div_(0, 0));                                // div x0,a
        pg.label("done"); pg.w(nop());
        DspState r = base_regs(0x40);
        RunResult rr = run_lockstep(rtl, st, pg, &r, pg.at("done"), 100, nullptr, true);
        insns += rr.insns; mis += rr.mismatches;
        // 24 DIV steps leave 23 quotient bits in A0 (the last one in C)
        uint32_t q = (uint32_t)((((uint64_t)n) << 23) / d) & 0xffffff;
        e += check("DIV quotient in A0 = n * 2^23 / d", dsp_core.registers[DSP_REG_A0], q);
        e += check("DIV last quotient bit in C", dsp_core.registers[DSP_REG_SR] & 1,
                   (uint32_t)((((uint64_t)n) << 24) / d) & 1);
    }
    // NORM: tst a, rep #46, norm r0,a
    for (int t = 0; t < 20; t++) {
        uint32_t v = (rng.u32() & 0xffffff) >> rng.below(23);
        if (v == 0) v = 1;
        Prog pg;
        pg.org(0x40);
        pg.move_imm(RA, v);
        pg.move_imm(RR0, 0);
        pg.w(alu(0x03));                                 // tst a
        pg.w(rep_imm(46));
        pg.w(norm(0, 0));                                // norm r0,a
        pg.label("done"); pg.w(nop());
        DspState r = base_regs(0x40);
        RunResult rr = run_lockstep(rtl, st, pg, &r, pg.at("done"), 100, nullptr, true);
        insns += rr.insns; mis += rr.mismatches;
        // normalized: A1 bits 23 and 22 differ; R0 = -(left shifts)
        int sh = 0;
        uint32_t x = v;
        if (x & 0x800000) { /* negative */ while (((x >> 22) & 3) == 3 && sh < 46) { x = (x << 1) & 0xffffff; sh++; } }
        else { while (((x >> 22) & 3) == 0 && sh < 46) { x = (x << 1) & 0xffffff; sh++; } }
        e += check("NORM exponent in R0", dsp_core.registers[DSP_REG_R0], (uint16_t)(-sh));
        e += check("NORM mantissa in A1", dsp_core.registers[DSP_REG_A1], x);
    }
    report("DIV sequences and NORM vs Hatari", mis == 0 && e == 0,
           fmt("(%llu insns, %llu mismatching, %d check failures)", (unsigned long long)insns,
               (unsigned long long)mis, e));
}

// ===========================================================================
// interrupts: host command (fast and long), host receive / transmit, SSI
// receive / transmit, illegal, stack error, trace, priorities
// ===========================================================================
static void test_interrupts(Rtl &rtl, Stats &st) {
    int e = 0;
    uint64_t insns = 0, mis = 0;
    // ---- host command, fast (2-word instruction) and long (JSR) ----
    {
        Prog pg;
        pg.org(0x24); pg.move_imm(RX1, 0x111111);          // vector $12: fast, one 2-word insn
        pg.org(0x26); pg.wl12(jsr(0), "h_long");            // vector $13: long
        pg.org(0x40);
        pg.movep_imm(0x3f, 0x0400);                         // IPR: host level 1 (IPL 0)
        pg.movep_imm(0x28, 0x04);                           // HCR: HCIE
        pg.move_imm(RX0, 1);
        pg.label("loop");
        pg.w(alu(0x40));                                    // add x0,a
        pg.wl12(jmp(0), "loop");
        pg.label("h_long");
        pg.move_imm(RY1, 0x222222);
        pg.w(rti());
        pg.label("done"); pg.w(nop());
        DspState r = base_regs(0x40);
        auto ev = [&](uint64_t i, HostEv &h, SsiEv &) {
            if (i == 20) { h.valid = true; h.we = true; h.off = 1; h.data = 0x92; }   // CVR: HC | $12
            if (i == 60) { h.valid = true; h.we = true; h.off = 1; h.data = 0x93; }   // CVR: HC | $13
            if (i == 61) { h.valid = true; h.we = false; h.off = 1; }                 // read CVR
        };
        RunResult rr = run_lockstep(rtl, st, pg, &r, 0xffff, 120, ev);
        insns += rr.insns; mis += rr.mismatches;
        e += check("fast host command handler ran (X1)", dsp_core.registers[DSP_REG_X1], 0x111111);
        e += check("long host command handler ran (Y1)", dsp_core.registers[DSP_REG_Y1], 0x222222);
        e += check("CVR.HC cleared after acceptance", dsp_core.hostport[CPU_HOST_CVR] & 0x80, 0);
        e += check("HSR.HCP cleared", dsp_core.periph[0][DSP_HOST_HSR] & 4, 0);
        e += check("SP back to 0 after RTI", dsp_core.registers[DSP_REG_SP], 0);
    }
    // ---- host receive (level 2) / transmit interrupts ----
    {
        Prog pg;
        pg.org(0x20); pg.w(movep_reg(false, 0, RY0, 0x2b)); pg.w(alu(0x58));  // movep x:hrx,y0 ; add y0,b? (0x58 add y0,b)
        pg.org(0x22); pg.w(movep_reg(true, 0, RX1, 0x2b)); pg.w(movep_xy(true, 0, EA_IMM, 0, 0x28)); // movep x1,x:htx ; movep #,x:hcr (next word)
        pg.p[0x24] = 0x000001;   // HCR value: HRIE only (disables HTIE again)
        pg.org(0x40);
        pg.movep_imm(0x3f, 0x0800);                         // IPR: host level 2
        pg.movep_imm(0x28, 0x01);                           // HCR: HRIE
        pg.move_imm(RX1, 0x5a5a5a);
        pg.label("loop"); pg.w(nop()); pg.wl12(jmp(0), "loop");
        DspState r = base_regs(0x40);
        auto ev = [&](uint64_t i, HostEv &h, SsiEv &) {
            if (i == 10) { h.valid = true; h.we = true; h.off = 5; h.data = 0x12 << 8 | 0x12; }  // TXH (odd: low byte)
            if (i == 11) { h.valid = true; h.we = true; h.off = 6; h.data = 0x34 << 8; }          // TXM (even: high byte)
            if (i == 12) { h.valid = true; h.we = true; h.off = 7; h.data = 0x56; }               // TXL -> HRDF
        };
        RunResult rr = run_lockstep(rtl, st, pg, &r, 0xffff, 60, ev);
        insns += rr.insns; mis += rr.mismatches;
        e += check("host receive interrupt read HRX into Y0", dsp_core.registers[DSP_REG_Y0], 0x123456);
        e += check("HRDF cleared by the read", dsp_core.periph[0][DSP_HOST_HSR] & 1, 0);
    }
    // ---- SSI receive / transmit interrupts in network mode ----
    {
        Prog pg;
        pg.org(0x0c); pg.w(movep_reg(false, 0, RY1, 0x2f)); pg.w(alu(0x00));  // movep x:rx,y1
        pg.org(0x10); pg.w(movep_reg(true, 0, RX1, 0x2f)); pg.w(alu(0x00));   // movep x1,x:tx
        pg.org(0x40);
        pg.movep_imm(0x3f, 0x3000);                         // IPR: SSI level 3
        pg.movep_imm(0x2c, 0x4100);                         // CRA: 16 bit words, 2 per frame
        pg.movep_imm(0x2d, 0xfa00);                         // CRB: RIE TIE RE TE MOD (network)
        pg.move_imm(RX1, 0xabcd00);
        pg.label("loop"); pg.w(nop()); pg.wl12(jmp(0), "loop");
        DspState r = base_regs(0x40);
        uint16_t rx_seq[6] = {0x1111, 0x2222, 0x8333, 0x4444, 0x5555, 0x6666};
        int ri = 0;
        auto ev = [&](uint64_t i, HostEv &, SsiEv &s) {
            if (i >= 12 && (i % 8) == 4 && ri < 6) {
                s.valid = true; s.tx_en = true; s.rx_en = true;
                s.frame = s.rx_frame = (ri % 2) == 0;
                s.rx_data = rx_seq[ri++];
            }
        };
        RunResult rr = run_lockstep(rtl, st, pg, &r, 0xffff, 80, ev);
        insns += rr.insns; mis += rr.mismatches;
        e += check("SSI receive interrupt read RX (16-bit word in RX[23:8])",
                   dsp_core.registers[DSP_REG_Y1], 0x666600);
        e += check("SSI transmit word (TX[23:8])", dsp_core.ssi.transmit_value & 0xffff, 0xabcd);
    }
    // ---- illegal instruction, stack overflow, trace ----
    {
        Prog pg;
        pg.org(0x02); pg.move_imm(RY0, 0x0000aa);          // stack error vector
        pg.org(0x04); pg.w(alu(0x48)); pg.w(nop());         // trace vector: add x0,b
        pg.org(0x3e); pg.move_imm(RX1, 0x333333);          // illegal vector
        pg.org(0x40);
        pg.move_imm(RX0, 1);
        pg.w(illegal());
        pg.w(nop()); pg.w(nop()); pg.w(nop()); pg.w(nop());
        // 16 nested JSR: the 16th push overflows the stack
        for (int k = 0; k < 16; k++) { pg.w(jsr(pg.pc + 1)); }
        pg.w(nop()); pg.w(nop()); pg.w(nop()); pg.w(nop());
        pg.w(movec_imm(0, RSP));                            // reset SP (clears SE/UF)
        pg.w(alu(0x1b));                                    // clr b
        pg.w(ori(0, 0x20));                                 // trace on
        pg.w(nop()); pg.w(nop()); pg.w(nop());
        pg.w(andi(0, 0xdf));                                // trace off
        pg.label("done"); pg.w(nop());
        DspState r = base_regs(0x40);
        RunResult rr = run_lockstep(rtl, st, pg, &r, pg.at("done"), 200);
        insns += rr.insns; mis += rr.mismatches;
        e += check("illegal instruction vector ran", dsp_core.registers[DSP_REG_X1], 0x333333);
        e += check("stack error vector ran", dsp_core.registers[DSP_REG_Y0], 0xaa);
        e += check("trace vector ran", dsp_core.registers[DSP_REG_B1] != 0 ? 1 : 0, 1);
    }
    report("interrupts (host cmd fast/long, host rx, SSI, illegal, stack, trace) vs Hatari",
           mis == 0 && e == 0,
           fmt("(%llu insns, %llu mismatching, %d check failures)", (unsigned long long)insns,
               (unsigned long long)mis, e));
}

// ===========================================================================
// SWI / WAIT / STOP: hardware behaviour (RTL only; Hatari runs them as NOPs)
// ===========================================================================
static int run_until_retire_count(Rtl &rtl, int n, int max_clocks) {
    int r = 0;
    for (int c = 0; c < max_clocks && r < n; c++) {
        rtl.eval0(); rtl.clock();
        if (rtl.top->rootp->falcon_dsp__DOT__u_core__DOT__retire) r++;
    }
    return r;
}

static void test_swi_wait_stop(Rtl &rtl) {
    Vfalcon_dsp *top = rtl.top;
    int e = 0;
    // SWI: level 3 interrupt to P:$0006
    {
        Prog pg;
        pg.org(0x06); pg.move_imm(RX1, 0x0000ee);          // SWI vector (fast)
        pg.org(0x40);
        pg.w(swi());
        for (int i = 0; i < 8; i++) pg.w(nop());
        pg.finish();
        Stats st;
        Lockstep ls(rtl, st);
        DspState r = base_regs(0x40);
        ls.start(pg.p, pg.xint, pg.yint, pg.ext, &r);
        int n = run_until_retire_count(rtl, 8, 400);
        e += check("SWI: instructions retired", n, 8);
        e += check("SWI vector P:$0006 executed (X1)", RC(x1), 0xee);
        e += check("SWI: SP unchanged (fast interrupt)", RC(sp), 0);
    }
    // WAIT: stop until an enabled interrupt is pending
    {
        Prog pg;
        pg.org(0x20); pg.w(movep_reg(false, 0, RY0, 0x2b)); pg.w(nop());   // host receive vector
        pg.org(0x40);
        pg.movep_imm(0x3f, 0x0400);                         // IPR host level 1
        pg.movep_imm(0x28, 0x01);                           // HRIE
        pg.w(wait_());
        pg.move_imm(RX1, 0x777777);
        for (int i = 0; i < 8; i++) pg.w(nop());
        pg.finish();
        Stats st;
        Lockstep ls(rtl, st);
        DspState r = base_regs(0x40);
        ls.start(pg.p, pg.xint, pg.yint, pg.ext, &r);
        int n = run_until_retire_count(rtl, 3, 200);        // two movep + wait
        e += check("WAIT: retired up to the WAIT", n, 3);
        n = run_until_retire_count(rtl, 1, 2000);
        e += check("WAIT: no instruction while waiting (2000 clocks)", n, 0);
        e += check("WAIT: core in wait state", rtl.state(), 3);
        rtl.bus_access(true, 2, false, true, 0x00ab);       // TXH
        rtl.bus_access(true, 3, true, false, 0xcd00);       // TXM
        rtl.bus_access(true, 3, false, true, 0x00ef);       // TXL -> HRDF
        n = run_until_retire_count(rtl, 8, 400);
        e += check("WAIT: resumed after the host receive interrupt", n, 8);
        e += check("WAIT: interrupt handler read HRX", RC(y0), 0xabcdef);
        e += check("WAIT: code after WAIT executed", RC(x1), 0x777777);
    }
    // STOP: stop until reset
    {
        Prog pg;
        pg.org(0x40);
        pg.w(stop());
        pg.move_imm(RX1, 0x555555);
        pg.finish();
        Stats st;
        Lockstep ls(rtl, st);
        DspState r = base_regs(0x40);
        ls.start(pg.p, pg.xint, pg.yint, pg.ext, &r);
        int n = run_until_retire_count(rtl, 1, 100);
        e += check("STOP: retired", n, 1);
        n = run_until_retire_count(rtl, 1, 5000);
        e += check("STOP: no instruction after STOP", n, 0);
        e += check("STOP: core stopped", rtl.state(), 4);
        e += check("STOP: following instruction not executed", RC(x1) == 0x555555 ? 1 : 0, 0);
        rtl.reset();
        e += check("STOP: left by DSP reset", rtl.state(), 0);
    }
    report("SWI / WAIT / STOP hardware behaviour (RTL)", e == 0, fmt("(%d check failures)", e));
}

void run_directed_tests(Rtl &rtl, const TestConfig &cfg, Stats &st) {
    auto want = [&](const char *n) { return cfg.only.empty() || std::string(n).find(cfg.only) != std::string::npos; };
    if (want("loops")) test_loops(rtl, st);
    if (want("agu")) test_agu(rtl, st);
    if (want("limit")) test_limit_scale(rtl, st);
    if (want("div")) test_div_norm(rtl, st);
    if (want("interrupts")) test_interrupts(rtl, st);
    if (want("swi")) test_swi_wait_stop(rtl);
}
