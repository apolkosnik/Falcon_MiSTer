// Falcon DSP testbench - tiny program builder for hand-encoded DSP56001
// test programs (labels resolved by patching). Plain ASCII.
#ifndef TB_PROG_H
#define TB_PROG_H

#include "tb_lockstep.h"
#include <map>
#include <functional>

struct Prog {
    std::vector<uint32_t> p;          // P space image, 64K words
    std::vector<uint32_t> ext;        // external RAM image (32K)
    uint32_t xint[256], yint[256];
    uint16_t pc = 0x40;
    std::map<std::string, uint16_t> labels;
    std::vector<std::pair<uint16_t, std::string>> fix_ew;   // word address <- label
    std::vector<std::pair<uint16_t, std::string>> fix_12;   // 12-bit field <- label

    Prog() : p(0x10000, 0), ext(32768, 0) {
        memset(xint, 0, sizeof xint);
        memset(yint, 0, sizeof yint);
    }
    void org(uint16_t a) { pc = a; }
    void w(uint32_t v) { p[pc++] = v & 0xffffff; }
    void w2(uint32_t v, uint32_t e) { w(v); w(e); }
    void label(const std::string &n) { labels[n] = pc; }
    // second word = address of a label (DO loop end, JCLR target, jmp ea)
    void w2l(uint32_t v, const std::string &l) { w(v); fix_ew.push_back({pc, l}); w(0); }
    // 12-bit absolute jump field
    void wl12(uint32_t v, const std::string &l) { fix_12.push_back({pc, l}); w(v); }
    // label at the last instruction of a loop body: label("x") placed
    // before that instruction
    void finish() {
        for (auto &f : fix_ew) p[f.first] = labels.at(f.second);
        for (auto &f : fix_12) p[f.first] = (p[f.first] & ~0xfffu) | (labels.at(f.second) & 0xfff);
        for (int i = 0x200; i < 0x10000; i++) if (p[i]) ext[i & 0x7fff] = p[i];
    }
    uint16_t at(const std::string &n) const { return labels.at(n); }

    // common helpers
    void move_imm(int reg, uint32_t v) { w2(enc::movei(reg), v); }                 // move #v,reg
    void movec_imm24(int creg, uint32_t v) { w2(enc::movec_ea(true, 0, enc::EA_IMM, creg), v); }
    void movep_imm(int pp, uint32_t v) { w2(enc::movep_xy(true, 0, enc::EA_IMM, 0, pp), v); } // X:$ffc0+pp
};

// lockstep runner: start both models on the program and step until P:pc
// reaches 'stop_label' (or max instructions); returns total mismatching
// instructions; ev() may supply events at each boundary
struct RunResult { uint64_t insns = 0, mismatches = 0; bool reached = false; };
RunResult run_lockstep(Rtl &rtl, Stats &st, Prog &pg, const DspState *regs, uint16_t stop_pc,
                       uint64_t max_insns,
                       std::function<void(uint64_t, HostEv &, SsiEv &)> ev = nullptr,
                       bool verbose = true);

#endif
