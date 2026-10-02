// Falcon DSP testbench - lockstep differential engine: executes one
// instruction in Hatari's DSP core and in the RTL, applies the same 68030 /
// crossbar events at the instruction boundary to both, and compares every
// register, the hidden emulation state, the peripherals, all memories and the
// cycle count. Plain ASCII.
#ifndef TB_LOCKSTEP_H
#define TB_LOCKSTEP_H

#include "tb_gen.h"
#include <map>

extern "C" {
extern int golden_hs_record_frame;   // Hatari dmaRecord.handshakeMode_Frame model
}

struct HostEv {
    bool valid = false;
    bool we = false;
    int off = 0;           // byte offset 0..7 (even offset + word = word access)
    bool word = false;
    uint16_t data = 0;
};

struct SsiEv {
    bool valid = false;
    bool tx_en = false, rx_en = false;
    bool frame = false, rx_frame = false;
    uint16_t rx_data = 0;
    bool loopback = false;   // receive the word transmitted in this slot
};

struct Stats {
    uint64_t insns = 0;
    uint64_t mismatching_insns = 0;
    uint64_t cyc_match = 0, cyc_mismatch = 0;
    uint64_t host_events = 0, ssi_events = 0;
    uint64_t dev_bitrev0 = 0;        // documented deviation events (Hatari bit 16 in Rn)
    uint64_t dev_rx25 = 0;           // Hatari SHFD swap left bit 24 in ssi.RX
    uint64_t dev_crb_rearm = 0;      // TE/RE enabled by a CRB write: hardware waits for frame sync
    uint64_t stop_hw_insn = 0;       // runs ended at SWI/WAIT/STOP (hardware semantics)
    uint64_t interrupts = 0, long_interrupts = 0;
    std::map<std::string, uint64_t> classes, alu_ops, ea_modes;
};

struct Lockstep {
    Rtl &rtl;
    Stats &st;
    bool first = true;          // first instruction after start has the fetch clock
    int hs_play_pulses = 0;
    bool verbose = true;
    int max_report = 10;
    int reported = 0;
    Lockstep(Rtl &r, Stats &s) : rtl(r), st(s) {}

    // reset both models, load memories, start (HF0) and set registers
    void start(const std::vector<uint32_t> &p, const uint32_t *xint, const uint32_t *yint,
               const std::vector<uint32_t> &ext, const DspState *regs);
    // execute one instruction (returns: 0 ok, >0 number of mismatches,
    // -1 stopped before a hardware-deviation instruction, -2 RTL hang)
    int step(const HostEv *h = nullptr, const SsiEv *s = nullptr, uint16_t *host_rd = nullptr,
             uint16_t *ssi_tx = nullptr);
};

#endif
