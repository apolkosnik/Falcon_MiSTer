// Falcon DSP testbench - replay of a recorded 68030 host port session.
// The log comes from Hatari built with the HATARI_DSPLOG hook (every host
// port read and write except ISR polls, with the number of DSP instructions
// executed since the last DSP reset, "X"):
//     <insns> W|R <offset 0..7> <hex byte>
// From the last reset on, the bootstrap writes go to both models while the
// DSP is stopped; once it runs, every event is applied at the instruction
// boundary where Hatari saw it, one per instruction (accesses Hatari made
// between the same two instructions go to the following boundaries), and the
// RTL is compared with Hatari's DSP core instruction by instruction
// (registers every instruction, memories every --mem-every instructions).
// Host reads are also compared with the log: if Hatari's own core reads a
// different value, the replay has left the recorded path (timing only, not a
// fault of the RTL).  Plain ASCII.
#include "tb_lockstep.h"
#include <cstdio>
#include <cstring>
#include <vector>
#include <string>

struct ReplayEv { uint64_t cnt; bool we; int off; uint8_t val; bool reset; };

int run_replay(Rtl &rtl, Stats &st, const std::string &path, uint64_t max_insns, uint64_t mem_every) {
    FILE *f = fopen(path.c_str(), "r");
    if (!f) { printf("FAIL: cannot open %s\n", path.c_str()); return 1; }
    std::vector<ReplayEv> ev;
    char line[128];
    while (fgets(line, sizeof line, f)) {
        if (line[0] == 'X') { ev.push_back({0, false, 0, 0, true}); continue; }
        unsigned long long c; char k; int o; unsigned v;
        if (sscanf(line, "%llu %c %d %x", &c, &k, &o, &v) == 4) ev.push_back({c, k == 'W', o, (uint8_t)v, false});
    }
    fclose(f);
    printf("replay: %zu host port events and DSP resets\n", ev.size());

    Vfalcon_dsp *top = rtl.top;
    (void)top;
    int rd_mis = 0;
    uint64_t log_mis = 0;
    size_t i = 0;
    uint64_t total = 0, applied = 0, first_mis = 0, first_log_dev = 0, blind = 0;
    const uint64_t wait_max = 4000000;
    int ret = 0, session = 0;
    Lockstep ls(rtl, st);
    ls.verbose = true;
    ls.max_report = 12;
    ls.mem_every = mem_every;
    if (i < ev.size() && ev[i].reset) i++;
    while (i <= ev.size() && total < max_insns) {
        // DSP reset (memories are kept by both models, as on the Falcon)
        session++;
        rtl.reset();
        golden_reset();
        dsp_core.ssi.transmit_value = RP(ssi_tval);
        dsp_core.ssi.received_value = RP(ssi_rval);
        dsp_core.hostport[CPU_HOST_RXH] = RP(rxh); dsp_core.hostport[CPU_HOST_RXM] = RP(rxm);
        dsp_core.hostport[CPU_HOST_RXL] = RP(rxl);
        dsp_core.hostport[CPU_HOST_TXH] = RP(txh); dsp_core.hostport[CPU_HOST_TXM] = RP(txm);
        dsp_core.hostport[CPU_HOST_TXL] = RP(txl);
        rtl.eval0();
        // bootstrap: the DSP is stopped, every access goes to both at once
        size_t b0 = i;
        while (i < ev.size() && !ev[i].reset && !dsp_core.running) {
            const ReplayEv &e = ev[i++];
            if (e.we) host_write_both(rtl, e.off, e.val);
            else {
                uint8_t bb = host_read_both(rtl, e.off, rd_mis);
                if (bb != e.val) log_mis++;
            }
        }
        if (!dsp_core.running) {
            printf("replay: session %d: %zu events, the DSP never started\n", session, i - b0);
            if (i < ev.size() && ev[i].reset) { i++; continue; }
            break;
        }
        for (int j = 0; j < 10 && rtl.state() != 1; j++) { rtl.eval0(); rtl.clock(); }
        rtl.eval0();
        printf("replay: session %d: DSP running after %zu bootstrap events\n", session, i - b0);
        ls.first = true;
        uint64_t k = 0, waited = 0;
        while (total < max_insns) {
            if (i < ev.size() && ev[i].reset) break;
            HostEv h;
            bool have = false;
            // as the 68030 polls ISR: data reads wait for RXDF, data writes
            // for TXDE (bounded: a blind access goes ahead after wait_max)
            bool ready = true;
            if (i < ev.size() && ev[i].off >= 5) {
                uint8_t isr = dsp_core.hostport[CPU_HOST_ISR];
                ready = ev[i].we ? (isr & 2) : (isr & 1);
                if (!ready && ev[i].cnt <= k + 1 && ++waited > wait_max) {
                    ready = true; blind++;
                    if (blind <= 4)
                        printf("replay: blind %s at offset %d (event %zu, logged at insn %llu, now %llu): ISR %02x, DSP PC %04x, HSR %02x HCR %02x\n",
                               ev[i].we ? "write" : "read", ev[i].off, i, (unsigned long long)ev[i].cnt, (unsigned long long)k,
                               isr, dsp_core.pc, dsp_core.periph[DSP_SPACE_X][DSP_HOST_HSR], dsp_core.periph[DSP_SPACE_X][DSP_HOST_HCR]);
                }
            }
            if (i < ev.size() && ready && ev[i].cnt <= k + 1) {
                waited = 0;
                h.valid = true; h.we = ev[i].we; h.off = ev[i].off; h.word = false;
                h.data = (ev[i].off & 1) ? ev[i].val : (uint16_t)(ev[i].val << 8);
                have = true;
            }
            uint16_t rd = 0;
            int r = ls.step(have ? &h : nullptr, nullptr, &rd);
            k++; total++;
            if (have) {
                if (!h.we) {
                    uint8_t bb = (h.off & 1) ? (rd & 0xff) : (rd >> 8);
                    if (bb != ev[i].val) { log_mis++; if (!first_log_dev) first_log_dev = total; }
                }
                i++; applied++;
            }
            if (r == -1) { printf("replay: stopped at a hardware-deviation instruction (SWI/WAIT/STOP) after %llu instructions, PC %04x\n",
                                  (unsigned long long)total, dsp_core.pc); total = max_insns; break; }
            if (r == -2) { printf("replay: RTL hang after %llu instructions\n", (unsigned long long)total); ret = 1; total = max_insns; break; }
            if (r > 0) {
                if (!first_mis) first_mis = total;
                if (st.mismatching_insns >= 3) { total = max_insns; break; }
            }
            if ((total % 20000000) == 0)
                printf("replay: %llu instructions, %llu events applied, %llu reads off the log (first at %llu), %llu blind\n",
                       (unsigned long long)total, (unsigned long long)applied, (unsigned long long)log_mis,
                       (unsigned long long)first_log_dev, (unsigned long long)blind);
            if (i >= ev.size()) { total = max_insns; break; }
        }
        if (i < ev.size() && ev[i].reset) { i++; continue; }
        break;
    }
    uint64_t k = total;
    int m = compare_memories(rtl, "end of replay", true);
    printf("replay: %llu instructions, %llu of %zu events applied, %llu mismatching instructions (first at %llu), "
           "%d memory differences at the end, %llu host reads off the log (first at %llu), %llu blind accesses\n",
           (unsigned long long)k, (unsigned long long)applied, ev.size(),
           (unsigned long long)st.mismatching_insns, (unsigned long long)first_mis, m,
           (unsigned long long)log_mis, (unsigned long long)first_log_dev, (unsigned long long)blind);
    return (st.mismatching_insns || m || ret) ? 1 : 0;
}
