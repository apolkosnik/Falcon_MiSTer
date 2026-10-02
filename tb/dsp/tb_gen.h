// Falcon DSP testbench - DSP56001 instruction encoders and random
// instruction-stream generator. Plain ASCII.
#ifndef TB_GEN_H
#define TB_GEN_H

#include "tb_common.h"

// register numbers (Hatari DSP_REG_*)
enum {
    RX0 = 4, RX1, RY0, RY1, RA0, RB0, RA2, RB2, RA1, RB1, RA, RB,
    RR0 = 0x10, RN0 = 0x18, RM0 = 0x20,
    RSR = 0x39, ROMR, RSP, RSSH, RSSL, RLA, RLC
};

// name of the Hatari handler an instruction word dispatches to
const char *hatari_class(uint32_t w);
const char *hatari_alu(uint32_t w);
bool insn_is_hw_deviation(uint32_t w);   // SWI / WAIT / STOP

// ---------------------------------------------------------------------------
// encoders (DSP56000 family manual encodings, checked against Hatari's
// dispatch tables by enc_selfcheck())
// ---------------------------------------------------------------------------
namespace enc {
    // effective address field mmmrrr
    inline uint32_t ea(int mode, int r) { return ((mode & 7) << 3) | (r & 7); }
    const uint32_t EA_ABS = 0x30, EA_IMM = 0x34;

    inline uint32_t nop() { return 0x000000; }
    inline uint32_t rti() { return 0x000004; }
    inline uint32_t illegal() { return 0x000005; }
    inline uint32_t swi() { return 0x000006; }
    inline uint32_t rts() { return 0x00000c; }
    inline uint32_t reset() { return 0x000084; }
    inline uint32_t wait_() { return 0x000086; }
    inline uint32_t stop() { return 0x000087; }
    inline uint32_t enddo() { return 0x00008c; }
    inline uint32_t andi(int dst, uint32_t imm) { return 0x0000b8 | ((imm & 0xff) << 8) | (dst & 3); }
    inline uint32_t ori(int dst, uint32_t imm) { return 0x0000f8 | ((imm & 0xff) << 8) | (dst & 3); }
    inline uint32_t div_(int src, int d) { return 0x018040 | ((src & 3) << 4) | ((d & 1) << 3); }
    inline uint32_t norm(int rn, int d) { return 0x01d815 | ((rn & 7) << 8) | ((d & 1) << 3); }
    inline uint32_t tcc(int cc, int idx, bool s2d2 = false, int r_src = 0, int r_dst = 0) {
        return 0x020000 | (s2d2 ? 0x10000 : 0) | ((cc & 15) << 12) | ((r_src & 7) << 8) |
               ((idx & 15) << 3) | (r_dst & 7);
    }
    inline uint32_t lua(int ea5, bool to_n, int dst) {
        return 0x044000 | ((ea5 & 31) << 8) | (to_n ? 8 : 0) | (dst & 7);
    }
    // movec: c = movec register (0x20-0x27, 0x39-0x3f)
    inline uint32_t movec_imm(uint32_t imm, int c) { return 0x050080 | ((imm & 0xff) << 8) | (c & 0x3f); }
    inline uint32_t movec_reg(bool to_c, int reg, int c) {
        return 0x044080 | (to_c ? 0x8000 : 0) | ((reg & 0x3f) << 8) | (c & 0x3f);
    }
    inline uint32_t movec_aa(bool to_c, int sp, int aa, int c) {
        return 0x050000 | (to_c ? 0x8000 : 0) | ((aa & 0x3f) << 8) | ((sp & 1) << 6) | (c & 0x3f);
    }
    inline uint32_t movec_ea(bool to_c, int sp, int e, int c) {
        return 0x054000 | (to_c ? 0x8000 : 0) | ((e & 0x3f) << 8) | ((sp & 1) << 6) | (c & 0x3f);
    }
    // do / rep (second word of DO = loop address LA)
    inline uint32_t do_imm(int cnt) { return 0x060080 | ((cnt & 0xff) << 8) | ((cnt >> 8) & 15); }
    inline uint32_t do_aa(int sp, int aa) { return 0x060000 | ((aa & 0x3f) << 8) | ((sp & 1) << 6); }
    inline uint32_t do_ea(int sp, int e) { return 0x064000 | ((e & 0x3f) << 8) | ((sp & 1) << 6); }
    inline uint32_t do_reg(int reg) { return 0x06c000 | ((reg & 0x3f) << 8); }
    inline uint32_t rep_imm(int cnt) { return 0x0600a0 | ((cnt & 0xff) << 8) | ((cnt >> 8) & 15); }
    inline uint32_t rep_aa(int sp, int aa) { return 0x060020 | ((aa & 0x3f) << 8) | ((sp & 1) << 6); }
    inline uint32_t rep_ea(int sp, int e) { return 0x064020 | ((e & 0x3f) << 8) | ((sp & 1) << 6); }
    inline uint32_t rep_reg(int reg) { return 0x06c020 | ((reg & 0x3f) << 8); }
    // movem P memory <-> register
    inline uint32_t movem_aa(bool to_reg, int aa, int reg) {
        return 0x070000 | (to_reg ? 0x8000 : 0) | ((aa & 0x3f) << 8) | (reg & 0x3f);
    }
    inline uint32_t movem_ea(bool to_reg, int e, int reg) {
        return 0x074080 | (to_reg ? 0x8000 : 0) | ((e & 0x3f) << 8) | (reg & 0x3f);
    }
    // movep: pp = peripheral offset 0..63 ($FFC0+pp)
    inline uint32_t movep_reg(bool to_pp, int sp, int reg, int pp) {
        return 0x084000 | ((sp & 1) << 16) | (to_pp ? 0x8000 : 0) | ((reg & 0x3f) << 8) | (pp & 0x3f);
    }
    inline uint32_t movep_p(bool to_pp, int sp, int e, int pp) {
        return 0x084040 | ((sp & 1) << 16) | (to_pp ? 0x8000 : 0) | ((e & 0x3f) << 8) | (pp & 0x3f);
    }
    inline uint32_t movep_xy(bool to_pp, int pp_sp, int e, int ea_sp, int pp) {
        return 0x084080 | ((pp_sp & 1) << 16) | (to_pp ? 0x8000 : 0) | ((e & 0x3f) << 8) |
               ((ea_sp & 1) << 6) | (pp & 0x3f);
    }
    // bit operations: op 0 bclr, 1 bset, 2 bchg, 3 btst
    inline uint32_t bitop_aa(int op, int sp, int aa, int bit) {
        static const uint32_t b[4] = {0x0a0000, 0x0a0020, 0x0b0000, 0x0b0020};
        return b[op & 3] | ((aa & 0x3f) << 8) | ((sp & 1) << 6) | (bit & 31);
    }
    inline uint32_t bitop_ea(int op, int sp, int e, int bit) { return bitop_aa(op, sp, 0, bit) | 0x4000 | ((e & 0x3f) << 8); }
    inline uint32_t bitop_pp(int op, int sp, int pp, int bit) { return bitop_aa(op, sp, 0, bit) | 0x8000 | ((pp & 0x3f) << 8); }
    inline uint32_t bitop_reg(int op, int reg, int bit) {
        static const uint32_t b[4] = {0x0ac040, 0x0ac060, 0x0bc040, 0x0bc060};
        return b[op & 3] | ((reg & 0x3f) << 8) | (bit & 31);
    }
    // bit test and jump: op 0 jclr, 1 jset, 2 jsclr, 3 jsset (second word = target)
    inline uint32_t jbit_aa(int op, int sp, int aa, int bit) {
        static const uint32_t b[4] = {0x0a0080, 0x0a00a0, 0x0b0080, 0x0b00a0};
        return b[op & 3] | ((aa & 0x3f) << 8) | ((sp & 1) << 6) | (bit & 31);
    }
    inline uint32_t jbit_ea(int op, int sp, int e, int bit) { return jbit_aa(op, sp, 0, bit) | 0x4000 | ((e & 0x3f) << 8); }
    inline uint32_t jbit_pp(int op, int sp, int pp, int bit) { return jbit_aa(op, sp, 0, bit) | 0x8000 | ((pp & 0x3f) << 8); }
    inline uint32_t jbit_reg(int op, int reg, int bit) {
        static const uint32_t b[4] = {0x0ac000, 0x0ac020, 0x0bc000, 0x0bc020};
        return b[op & 3] | ((reg & 0x3f) << 8) | (bit & 31);
    }
    inline uint32_t jmp(int a) { return 0x0c0000 | (a & 0xfff); }
    inline uint32_t jsr(int a) { return 0x0d0000 | (a & 0xfff); }
    inline uint32_t jcc(int cc, int a) { return 0x0e0000 | ((cc & 15) << 12) | (a & 0xfff); }
    inline uint32_t jscc(int cc, int a) { return 0x0f0000 | ((cc & 15) << 12) | (a & 0xfff); }
    inline uint32_t jmp_ea(int e) { return 0x0ac080 | ((e & 0x3f) << 8); }
    inline uint32_t jcc_ea(int cc, int e) { return 0x0ac0a0 | ((e & 0x3f) << 8) | (cc & 15); }
    inline uint32_t jsr_ea(int e) { return 0x0bc080 | ((e & 0x3f) << 8); }
    inline uint32_t jscc_ea(int cc, int e) { return 0x0bc0a0 | ((e & 0x3f) << 8) | (cc & 15); }

    // parallel moves (alu = data ALU opcode byte, 0x00 = move only)
    // pm_0: A/B -> X:ea or Y:ea, X0/Y0 -> A/B
    inline uint32_t pm0(int acc, int sp, int e, uint32_t alu) {
        return 0x080000 | ((acc & 1) << 16) | ((sp & 1) << 15) | ((e & 0x3f) << 8) | (alu & 0xff);
    }
    // I:R update (Rn)+ etc., ea5 = mmrrr
    inline uint32_t rupd(int ea5, uint32_t alu) { return 0x204000 | ((ea5 & 31) << 8) | (alu & 0xff); }
    inline uint32_t alu(uint32_t a) { return 0x200000 | (a & 0xff); }
    // register to register (S,D 5-bit register numbers 4..31)
    inline uint32_t rr(int s, int d, uint32_t alu) { return 0x200000 | ((s & 31) << 13) | ((d & 31) << 8) | (alu & 0xff); }
    // #xx,D short immediate (D 4..31)
    inline uint32_t imm8(int d, uint32_t v, uint32_t alu) { return 0x200000 | ((d & 31) << 16) | ((v & 0xff) << 8) | (alu & 0xff); }
    // X:/Y: memory <-> register (D 4..31), ea or aa (aa when !use_ea)
    inline uint32_t xy(int sp, bool to_reg, int d, bool use_ea, int e_or_aa, uint32_t alu) {
        return 0x400000 | (((d >> 3) & 3) << 20) | ((sp & 1) << 19) | ((d & 7) << 16) |
               (to_reg ? 0x8000 : 0) | (use_ea ? 0x4000 : 0) | ((e_or_aa & 0x3f) << 8) | (alu & 0xff);
    }
    // move #xxxxxx,D (second word = value)
    inline uint32_t movei(int d, uint32_t alu = 0) { return xy(0, true, d, true, EA_IMM, alu); }
    // L: moves, l = 0 A10, 1 B10, 2 X, 3 Y, 4 A, 5 B, 6 AB, 7 BA
    inline uint32_t lmove(int l, bool to_reg, bool use_ea, int e_or_aa, uint32_t alu) {
        return 0x400000 | (((l >> 2) & 1) << 19) | ((l & 3) << 16) | (to_reg ? 0x8000 : 0) |
               (use_ea ? 0x4000 : 0) | ((e_or_aa & 0x3f) << 8) | (alu & 0xff);
    }
    // pm_1: X: move + register move (S2 = A/B -> D2 = Y0/Y1) or Y: version
    inline uint32_t pm1(uint32_t bits_19_8, uint32_t alu) { return 0x100000 | ((bits_19_8 & 0xfff) << 8) | (alu & 0xff); }
    // pm_8: X: and Y: moves.  xr: 0 X0 1 X1 2 A 3 B; yr: 0 Y0 1 Y1 2 A 3 B;
    // xmode/ymode: 0 (Rn), 1 (Rn)+Nn, 2 (Rn)-, 3 (Rn)+; xr_n 0..7, yr_n low 2 bits
    inline uint32_t xymove(bool xw_reg, int xr, int xmode, int xrn, bool yw_reg, int yr, int ymode, int yrn,
                           uint32_t alu) {
        return 0x800000 | (yw_reg ? 0x400000 : 0) | ((ymode & 3) << 20) | ((xr & 3) << 18) |
               ((yr & 3) << 16) | (xw_reg ? 0x8000 : 0) | ((yrn & 3) << 13) | ((xmode & 3) << 11) |
               ((xrn & 7) << 8) | (alu & 0xff);
    }
}

// check the encoders against Hatari's dispatch tables
int enc_selfcheck();

// ---------------------------------------------------------------------------
// random program / state generation
// ---------------------------------------------------------------------------
struct GenOptions {
    int pct_parallel = 60;
    int pct_control_regs = 12;   // movec/register selections of SR/OMR/SP/SSH/LA/LC
    int pct_periph = 8;          // data addresses in X/Y:$FFC0-$FFFF
    bool allow_reset = true;
};

struct Gen {
    Rng &rng;
    GenOptions opt;
    explicit Gen(Rng &r) : rng(r) {}
    uint16_t addr_xy();
    uint32_t data24();
    uint16_t addr_p();
    int pp();
    int reg_any();
    int reg5();
    int reg_movec();
    int reg_xyab() { return (int)rng.below(4); }
    uint32_t alu_op();
    int ea6(bool allow_imm, bool allow_abs, bool &two, uint32_t &w2, bool is_p = false);
    int ea5();
    int insn(uint16_t pc, uint32_t w[2]);     // returns number of words
    void random_state(DspState &s);
    void fill_program(std::vector<uint32_t> &p, uint16_t from, uint16_t to);
};

#endif
