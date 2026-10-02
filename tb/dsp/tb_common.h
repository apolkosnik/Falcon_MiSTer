// Falcon DSP testbench - common harness: the RTL (Verilator model of
// falcon_dsp) and the golden model (Hatari src/falcon/dsp_cpu.c +
// dsp_core.c, linked unmodified), state readout and comparison.
// Plain ASCII.
#ifndef TB_COMMON_H
#define TB_COMMON_H

#include <cstdio>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <string>
#include <vector>

#include "Vfalcon_dsp.h"
#include "Vfalcon_dsp___024root.h"
#include "verilated.h"

extern "C" {
#include "dsp_core.h"
#include "dsp_cpu.h"
extern int golden_sc1_calls;
extern int golden_sc2_calls;
extern uint32_t golden_sc2_last;
}

#define RC(x) (top->rootp->falcon_dsp__DOT__u_core__DOT__##x)
#define RP(x) (top->rootp->falcon_dsp__DOT__u_periph__DOT__##x)

// ---------------------------------------------------------------------------
// random numbers (xorshift64*)
// ---------------------------------------------------------------------------
struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed = 1) : s(seed * 0x9E3779B97F4A7C15ull + 0x1234567ull) { if (!s) s = 1; }
    uint32_t u32() {
        s ^= s >> 12; s ^= s << 25; s ^= s >> 27;
        return (uint32_t)((s * 0x2545F4914F6CDD1Dull) >> 32);
    }
    uint32_t below(uint32_t n) { return n ? u32() % n : 0; }
    bool chance(uint32_t pct) { return below(100) < pct; }
};

// ---------------------------------------------------------------------------
// architectural + hidden state, read from either model
// ---------------------------------------------------------------------------
struct DspState {
    uint32_t x0, x1, y0, y1, a0, a1, a2, b0, b1, b2;
    uint32_t r[8], n[8], m[8];
    uint32_t sr, omr, sp, la, lc, lcsave, pc;
    uint32_t ssh[16], ssl[16];
    uint32_t loop_rep, pc_on_rep, int_state, int_cnt, int_fetch, int_save_pc, int_ipl;
    uint32_t agu_reg, agu_val;
    uint32_t int_status;
    // host port
    uint32_t icr, cvr, isr, ivr, rxh, rxm, rxl, txh, txm, txl;
    uint32_t htx, rtx, running, boot_pos, hreq;
    // ssi
    uint32_t ssi_tx, ssi_rx, ssi_tval, ssi_rval, wait_tx, wait_rx, hs_frame;
    // peripheral space
    uint32_t perx[64], pery[64];
};

// ---------------------------------------------------------------------------
// golden model
// ---------------------------------------------------------------------------
extern int g_golden_hreq;
void golden_init_once();
void golden_reset();
void golden_read_state(DspState &s);
void golden_set_regs(const DspState &s);   // core registers only
uint32_t golden_pread(uint16_t a);
void golden_pwrite(uint16_t a, uint32_t v);

// ---------------------------------------------------------------------------
// RTL harness
// ---------------------------------------------------------------------------
struct Rtl {
    Vfalcon_dsp *top;
    uint64_t clocks;
    Rtl();
    ~Rtl();
    void eval0() { top->clk = 0; top->eval(); }
    void clock() { top->clk = 1; top->eval(); top->clk = 0; top->eval(); clocks++; }
    void idle_inputs();
    void reset(int cycles = 4);
    bool is_el() const;
    int  state() const { return top->rootp->falcon_dsp__DOT__u_core__DOT__state; }
    uint32_t ncyc() const { return top->rootp->falcon_dsp__DOT__u_core__DOT__ncyc; }
    void read_state(DspState &s);
    void set_regs(const DspState &s);
    // backdoor memory (P space mapping: P<0x200 internal, else ext)
    void pwrite(uint16_t a, uint32_t v);
    uint32_t pread(uint16_t a);
    void xyint_write(int space, uint8_t a, uint32_t v);
    void ext_write(uint16_t a15, uint32_t v);
    // 68030 bus byte/word access performed in the current clock (call with
    // clk low, inputs are applied for exactly one rising edge)
    uint16_t bus_access(bool we, int word_addr, bool uds, bool lds, uint16_t din);
};

// compare RTL and golden: returns number of differences, prints them
int compare_state(const DspState &g, const DspState &r, const char *ctx, bool verbose = true);
int compare_memories(Rtl &rtl, const char *ctx, bool verbose = true);

// 68030-side byte helpers applied to both models at the same moment
uint8_t host_read_both(Rtl &rtl, int off, int &mismatch);
void host_write_both(Rtl &rtl, int off, uint8_t v);

#endif
