// Verilator testbench for rtl/falcon/falcon_blitter.sv
//
// The real RTL module is driven through its register bus, with a RAM model
// (random latency) behind its DMA port, a CPU bus-grant model (bg follows br
// with random latency) and CPU bus cycle pulses.  Every operation is also
// executed by Hatari's src/blitter.c (compiled unmodified, see golden/) with
// the same register writes and the same initial memory.  Compared:
//   - the complete sequence of bus accesses (read/write, address, data),
//   - the number of bus accesses of every pass (non-hog: 64),
//   - all register readbacks after every pass and at the end,
//   - the busy line (Hatari MFP GPIP "GPU done" line),
//   - final memory contents.
// Plain ASCII only.

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <map>
#include <string>
#include "Vfalcon_blitter.h"
#include "verilated.h"
#include "golden.h"

static Vfalcon_blitter *top;
static uint64_t cyc = 0;
static int errors = 0;
static int checks = 0;
static const int TIMEOUT_CLKS = 512;   // NONHOG_CPU_TIMEOUT of the DUT

#define CHECK(cond, ...) do { checks++; if (!(cond)) { errors++; if (errors < 40) { printf("ERROR @%llu: ", (unsigned long long)cyc); printf(__VA_ARGS__); printf("\n"); } } } while (0)

// ---------------------------------------------------------------------------
// random
// ---------------------------------------------------------------------------
static uint64_t rng_state = 0x1234567887654321ULL;
static uint32_t rnd() {
    rng_state ^= rng_state << 13; rng_state ^= rng_state >> 7; rng_state ^= rng_state << 17;
    return (uint32_t)(rng_state >> 11);
}
static int rnd_range(int lo, int hi) { return lo + (int)(rnd() % (uint32_t)(hi - lo + 1)); }

// ---------------------------------------------------------------------------
// memory: sparse, initial content is a hash of (seed, address)
// ---------------------------------------------------------------------------
struct Mem {
    std::unordered_map<uint32_t, uint16_t> w;
    uint32_t seed = 1;
    uint16_t init(uint32_t a) const {
        uint32_t x = (a >> 1) * 0x9E3779B1u ^ seed * 0x85EBCA77u;
        x ^= x >> 15; x *= 0x2C1B3C6Du; x ^= x >> 12; x *= 0x297A2D39u; x ^= x >> 15;
        return (uint16_t)x;
    }
    uint16_t rd(uint32_t a) const { auto it = w.find(a); return it == w.end() ? init(a) : it->second; }
    void wr(uint32_t a, uint16_t v) { w[a] = v; }
};
static Mem rtl_mem, gm_mem;

struct Acc { bool we; uint32_t addr; uint16_t data; };
static std::vector<Acc> gm_trace, rtl_trace;

extern "C" uint16_t gm_mem_read(uint32_t a) {
    uint16_t v = gm_mem.rd(a);
    gm_trace.push_back({false, a, v});
    return v;
}
extern "C" void gm_mem_write(uint32_t a, uint16_t v) {
    gm_mem.wr(a, v);
    gm_trace.push_back({true, a, v});
}

// ---------------------------------------------------------------------------
// environment: RAM, bus grant, CPU bus cycles
// ---------------------------------------------------------------------------
static int ram_busy = 0, ram_wait = 0;
static uint32_t ram_addr; static bool ram_we; static uint16_t ram_wdata;
static int ram_lat_max = 8;
static int bg_lat_on = 2, bg_lat_off = 2, bg_cnt = 0;
static bool cpu_pulses_on = false;
static int pulse_wait = 0;
static int pulse_min = 1, pulse_max = 4;
static int pending_bus_pulse = 0;
static bool prev_req = false, prev_br = false;
static int pass_acc = 0;                 // accesses since br rose
static std::vector<int> rtl_passes;      // completed passes (accesses)
static int pulses_since_fall = 0;
static int pulses_at_rise = -1;
static bool pulse_at_rise_edge = false;
static uint64_t br_fall_cyc = 0, br_rise_cyc = 0;
static int br_rises = 0;
static bool bus_access_active = false;

static void env_before_edge() {
    // ---- RAM ----
    top->dma_ack = 0;
    if (ram_busy) {
        CHECK(top->dma_req, "dma_req dropped before dma_ack");
        CHECK(((uint32_t)top->dma_addr << 1) == ram_addr && (bool)top->dma_we == ram_we &&
              (!ram_we || top->dma_wdata == ram_wdata), "DMA address/data changed while dma_req held");
        if (--ram_wait <= 0) {
            top->dma_ack = 1;
            if (ram_we) {
                rtl_mem.wr(ram_addr, ram_wdata);
                rtl_trace.push_back({true, ram_addr, ram_wdata});
            } else {
                uint16_t v = rtl_mem.rd(ram_addr);
                top->dma_rdata = v;
                rtl_trace.push_back({false, ram_addr, v});
            }
            pass_acc++;
            ram_busy = 0;
        }
    } else if (top->dma_req) {
        CHECK(top->bg, "dma_req asserted while bg = 0");
        CHECK(top->dma_be == 3, "dma_be = %d, expected 3", top->dma_be);
        ram_busy = 1;
        ram_addr = (uint32_t)top->dma_addr << 1;
        ram_we = top->dma_we;
        ram_wdata = top->dma_wdata;
        ram_wait = rnd_range(1, ram_lat_max);
        if (--ram_wait <= 0) ram_wait = 0;
        if (ram_wait == 0) {
            // zero extra wait: acknowledge on this edge
            top->dma_ack = 1;
            if (ram_we) { rtl_mem.wr(ram_addr, ram_wdata); rtl_trace.push_back({true, ram_addr, ram_wdata}); }
            else { uint16_t v = rtl_mem.rd(ram_addr); top->dma_rdata = v; rtl_trace.push_back({false, ram_addr, v}); }
            pass_acc++;
            ram_busy = 0;
        }
    }
    prev_req = top->dma_req;

    // ---- bus grant: bg follows br with latency ----
    if (top->br && !top->bg) {
        if (++bg_cnt >= bg_lat_on) { top->bg = 1; bg_cnt = 0; }
    } else if (!top->br && top->bg) {
        if (ram_busy) bg_cnt = 0;   // never take the bus away during an access
        else if (++bg_cnt >= bg_lat_off) { top->bg = 0; bg_cnt = 0; }
    } else bg_cnt = 0;

    // ---- CPU bus cycles ----
    // a register access made by the CPU completes as a CPU bus cycle
    top->cpu_bus_cycle = 0;
    if (pending_bus_pulse) { top->cpu_bus_cycle = 1; pending_bus_pulse--; }
    else if (!top->bg) {
        if (cpu_pulses_on && !bus_access_active) {
            if (pulse_wait <= 0) { top->cpu_bus_cycle = 1; pulse_wait = rnd_range(pulse_min, pulse_max); }
            else pulse_wait--;
        }
    }
}

static void tick() {
    top->clk = 0; top->eval();
    env_before_edge();
    bool pulse = top->cpu_bus_cycle;
    top->clk = 1; top->eval();
    cyc++;
    // ---- after the edge ----
    if (pulse) pulses_since_fall++;
    bool br = top->br;
    if (br && !prev_br) {
        pulses_at_rise = pulses_since_fall;
        pulse_at_rise_edge = pulse;
        br_rise_cyc = cyc;
        br_rises++;
        pass_acc = 0;
    }
    if (!br && prev_br) {
        rtl_passes.push_back(pass_acc);
        pass_acc = 0;
        pulses_since_fall = 0;
        br_fall_cyc = cyc;
    }
    prev_br = br;
}

// ---------------------------------------------------------------------------
// register bus (RTL)
// ---------------------------------------------------------------------------
static void bus_cycle(int off, bool we, int size, uint16_t val, uint16_t *rd) {
    // the CPU can only access the bus when it is not granted to the blitter
    int guard = 0;
    while (top->bg) { tick(); if (++guard > 2000000) { CHECK(false, "bus access blocked by bg"); return; } }
    bus_access_active = true;
    top->bus_cs = 1; top->bus_stb = 1; top->bus_we = we;
    top->bus_addr = (off >> 1) & 0x1F;
    if (size == 2) { top->bus_uds = 1; top->bus_lds = 1; top->bus_din = val; }
    else if (off & 1) { top->bus_uds = 0; top->bus_lds = 1; top->bus_din = 0xFF00 | (val & 0xFF); }
    else { top->bus_uds = 1; top->bus_lds = 0; top->bus_din = (uint16_t)((val & 0xFF) << 8) | 0xFF; }
    int n = 0;
    for (;;) {
        tick();
        top->bus_stb = 0;
        if (top->bus_ack) break;
        if (++n > 16) { CHECK(false, "no bus_ack at offset %02x", off); break; }
    }
    if (rd) *rd = top->bus_dout;
    top->bus_cs = 0; top->bus_we = 0; top->bus_uds = 0; top->bus_lds = 0;
    tick();
    CHECK(!top->bus_ack, "bus_ack longer than one clock");
    pending_bus_pulse++;          // this access is a CPU bus cycle
    bus_access_active = false;
}
static void rtl_write(int off, int size, uint16_t v) { bus_cycle(off, true, size, v, nullptr); }
static uint16_t rtl_read(int off) { uint16_t r = 0; bus_cycle(off, false, 2, 0, &r); return r; }

// write the same value to the RTL and to Hatari
static void reg_write(int off, int size, uint16_t v) {
    rtl_write(off, size, v);
    gm_write(off, size, v);
}
static void reg_write_long(int off, uint32_t v) {
    // a 68030 long write to a 16-bit port: high word first, then low word
    reg_write(off, 2, (uint16_t)(v >> 16));
    reg_write(off + 2, 2, (uint16_t)v);
}

static const char *reg_name(int off) {
    static char b[16];
    switch (off) {
    case 0x20: return "SRC_XINC"; case 0x22: return "SRC_YINC"; case 0x24: return "SRC_ADDR_H"; case 0x26: return "SRC_ADDR_L";
    case 0x28: return "ENDMASK1"; case 0x2a: return "ENDMASK2"; case 0x2c: return "ENDMASK3"; case 0x2e: return "DST_XINC";
    case 0x30: return "DST_YINC"; case 0x32: return "DST_ADDR_H"; case 0x34: return "DST_ADDR_L"; case 0x36: return "XCOUNT";
    case 0x38: return "YCOUNT"; case 0x3a: return "HOP_LOP"; case 0x3c: return "CTRL_SKEW";
    default: snprintf(b, sizeof b, "HT%d", off / 2); return b;
    }
}

// compare all registers (word reads) RTL vs Hatari
static void compare_regs(const char *ctx) {
    for (int off = 0; off < 0x3e; off += 2) {
        uint16_t r = rtl_read(off);
        uint16_t g = gm_read(off);
        CHECK(r == g, "%s: register %s ($FF8A%02X) RTL=%04x expected(Hatari)=%04x", ctx, reg_name(off), off, r, g);
    }
    // byte reads of the byte registers
    for (int off = 0x3a; off <= 0x3d; off++) {
        uint16_t r = 0; bus_cycle(off, false, 1, 0, &r);
        uint8_t rb = (off & 1) ? (r & 0xFF) : (r >> 8);
        uint16_t gw = gm_read(off & ~1);
        uint8_t gb = (off & 1) ? (gw & 0xFF) : (gw >> 8);
        CHECK(rb == gb, "%s: byte register $FF8A%02X RTL=%02x expected(Hatari)=%02x", ctx, off, rb, gb);
    }
}

static size_t trace_cmp_from = 0;
static void compare_traces(const char *ctx) {
    size_t n = std::min(rtl_trace.size(), gm_trace.size());
    CHECK(rtl_trace.size() == gm_trace.size(), "%s: RTL made %zu bus accesses, expected(Hatari) %zu", ctx, rtl_trace.size(), gm_trace.size());
    for (size_t i = trace_cmp_from; i < n; i++) {
        const Acc &a = rtl_trace[i], &b = gm_trace[i];
        if (a.we != b.we || a.addr != b.addr || a.data != b.data) {
            CHECK(false, "%s: access %zu RTL %s %06x=%04x expected(Hatari) %s %06x=%04x", ctx, i,
                  a.we ? "W" : "R", a.addr, a.data, b.we ? "W" : "R", b.addr, b.data);
            break;
        }
        checks++;
    }
    trace_cmp_from = n;
}

static void compare_mem(const char *ctx) {
    int bad = 0;
    for (auto &kv : gm_mem.w) if (rtl_mem.rd(kv.first) != kv.second) bad++;
    for (auto &kv : rtl_mem.w) if (gm_mem.rd(kv.first) != kv.second) bad++;
    CHECK(bad == 0, "%s: %d memory words differ from Hatari", ctx, bad);
}

static bool wait_until(bool (*cond)(), int max) {
    for (int i = 0; i < max; i++) { if (cond()) return true; tick(); }
    return cond();
}
static bool c_br() { return top->br; }
static bool c_brlow_or_done() { return !top->br; }
static bool c_bg_low() { return !top->bg; }

// ---------------------------------------------------------------------------
// blit configuration
// ---------------------------------------------------------------------------
struct Cfg {
    uint16_t ht[16];
    int16_t sxi, syi, dxi, dyi;
    uint32_t saddr, daddr;
    uint16_t em1, em2, em3;
    uint16_t xc, yc;
    uint8_t hop, lop, ctrl, skew;
};

static void write_cfg(const Cfg &c) {
    for (int i = 0; i < 16; i++) reg_write(2 * i, 2, c.ht[i]);
    reg_write(0x20, 2, (uint16_t)c.sxi);
    reg_write(0x22, 2, (uint16_t)c.syi);
    reg_write_long(0x24, c.saddr);
    reg_write(0x28, 2, c.em1);
    reg_write(0x2a, 2, c.em2);
    reg_write(0x2c, 2, c.em3);
    reg_write(0x2e, 2, (uint16_t)c.dxi);
    reg_write(0x30, 2, (uint16_t)c.dyi);
    reg_write_long(0x32, c.daddr);
    reg_write(0x36, 2, c.xc);
    reg_write(0x38, 2, c.yc);
    if (rnd() & 1) reg_write(0x3a, 2, (uint16_t)((c.hop << 8) | c.lop));
    else { reg_write(0x3a, 1, c.hop); reg_write(0x3b, 1, c.lop); }
    reg_write(0x3d, 1, c.skew);
}

enum { ACT_NONE = 0, ACT_RESTART, ACT_PAUSE, ACT_MODIFY, ACT_TIMEOUT };

static int stat_passes = 0, stat_restart = 0, stat_pause = 0, stat_modify = 0, stat_timeout = 0, stat_words = 0;
static int stat_lop[16], stat_hop[4], stat_smudge_hop[4], stat_skew[16], stat_fn[4], stat_x1nfsr, stat_hog, stat_nonhog;

// run a configured blit (registers already written, ctrl byte to start in c.ctrl)
static void run_blit(const Cfg &c, int act_mode, const char *ctx) {
    gm_trace.clear(); rtl_trace.clear();
    rtl_passes.clear();
    trace_cmp_from = 0;
    char buf[96];

    // start (byte write to $FF8A3C, like move.b / bset)
    reg_write(0x3c, 1, c.ctrl);
    CHECK(top->busy == 1, "%s: busy output not set after start", ctx);

    int pass = 0;
    for (;;) {
        int ng = gm_run_pass();
        CHECK(ng >= 0, "%s: Hatari has no pass scheduled while the RTL is busy", ctx);
        if (ng < 0) break;
        stat_passes++;
        // RTL pass: br rises, accesses, br falls
        bool ok = wait_until(c_br, 5000);
        CHECK(ok, "%s: br did not rise for pass %d", ctx, pass);
        if (!ok) break;
        ok = wait_until(c_brlow_or_done, 10000000);
        CHECK(ok, "%s: pass %d did not end", ctx, pass);
        if (!ok) break;
        int nr = rtl_passes.empty() ? -1 : rtl_passes.back();
        CHECK(nr == ng, "%s: pass %d made %d bus accesses, expected(Hatari) %d", ctx, pass, nr, ng);
        if (!top->busy) {
            // blit complete
            CHECK(gm_int_pending == 0, "%s: RTL finished but Hatari has another pass scheduled", ctx);
            break;
        }
        CHECK(gm_int_pending == 1 && gm_gpu_line == 1, "%s: RTL continues but Hatari finished (pending=%d line=%d)", ctx, gm_int_pending, gm_gpu_line);
        if (!(gm_int_pending == 1)) break;
        CHECK(nr == 64, "%s: non-hog pass %d made %d accesses, expected 64", ctx, pass, nr);

        // CPU owns the bus now
        wait_until(c_bg_low, 100);
        snprintf(buf, sizeof buf, "%s pass %d", ctx, pass);
        compare_traces(buf);
        compare_regs(buf);
        CHECK(top->br == 0, "%s: br reasserted during the register readback (before 64 CPU cycles)", buf);

        int act = act_mode;
        if (act < 0) {
            int r = rnd_range(0, 99);
            act = r < 60 ? ACT_NONE : r < 75 ? ACT_RESTART : r < 88 ? ACT_PAUSE : ACT_MODIFY;
        }
        if (act == ACT_MODIFY) {
            // the CPU changes some registers between two passes
            stat_modify++;
            reg_write(2 * rnd_range(0, 15), 2, (uint16_t)rnd());
            reg_write(0x28 + 2 * rnd_range(0, 2), 2, (uint16_t)rnd());
            if (rnd() & 1) reg_write(0x3b, 1, (uint8_t)rnd_range(0, 15));
            act = ACT_NONE;
        }
        if (act == ACT_NONE) {
            cpu_pulses_on = true;
            ok = wait_until(c_br, 5000);
            cpu_pulses_on = false;
            CHECK(ok, "%s: blitter did not request the bus again", buf);
            CHECK(pulses_at_rise == 64 && pulse_at_rise_edge,
                  "%s: br reasserted after %d CPU bus cycles (on a pulse: %d), expected after exactly 64", buf, pulses_at_rise, (int)pulse_at_rise_edge);
        } else if (act == ACT_TIMEOUT) {
            stat_timeout++;
            uint64_t f = br_fall_cyc;
            int p0 = pulses_since_fall;
            ok = wait_until(c_br, 5000);
            CHECK(ok, "%s: blitter did not request the bus again after the timeout", buf);
            CHECK((int64_t)(br_rise_cyc - f) == TIMEOUT_CLKS, "%s: br low for %lld clocks with %d CPU cycles, expected %d (timeout)", buf,
                  (long long)(br_rise_cyc - f), p0, TIMEOUT_CLKS);
        } else if (act == ACT_RESTART) {
            // bset #7,$FF8A3C: read control, write it back with busy set -> immediate restart
            stat_restart++;
            uint16_t r = rtl_read(0x3c);
            uint16_t g = gm_read(0x3c);
            CHECK(r == g, "%s: control before restart RTL=%04x expected %04x", buf, r, g);
            uint8_t v = (uint8_t)(g >> 8) | 0x80;
            if (rnd_range(0, 3) == 0) v |= 0x40;      // and switch to hog mode
            int pb = pulses_since_fall;
            reg_write(0x3c, 1, v);
            ok = wait_until(c_br, 3);
            CHECK(ok, "%s: no immediate bus request after restart write (after %d CPU cycles)", buf, pb);
        } else if (act == ACT_PAUSE) {
            // write busy = 0: the blitter must stay off the bus
            stat_pause++;
            uint16_t g = gm_read(0x3c);
            uint8_t v = (uint8_t)(g >> 8) & 0x7F;
            reg_write(0x3c, 1, v);
            CHECK(gm_int_pending == 0, "%s: Hatari still has a pass scheduled after pause", buf);
            cpu_pulses_on = true;
            for (int i = 0; i < 3 * TIMEOUT_CLKS; i++) { tick(); if (top->br) break; }
            cpu_pulses_on = false;
            CHECK(top->br == 0, "%s: paused blitter requested the bus", buf);
            CHECK(top->busy == 1 && gm_gpu_line == 1, "%s: busy while paused RTL=%d expected(Hatari line)=%d", buf, top->busy, gm_gpu_line);
            compare_regs(buf);
            uint8_t v2 = (uint8_t)(v | 0x80);
            if (rnd_range(0, 3) == 0) v2 |= 0x40;
            reg_write(0x3c, 1, v2);
            ok = wait_until(c_br, 3);
            CHECK(ok, "%s: paused blitter did not restart", buf);
        }
        pass++;
    }
    CHECK(top->busy == 0 && gm_gpu_line == 0, "%s: busy at end RTL=%d expected(Hatari line)=%d", ctx, top->busy, gm_gpu_line);
    wait_until(c_bg_low, 100);
    compare_traces(ctx);
    compare_regs(ctx);
    compare_mem(ctx);
}

static Cfg random_cfg(int i) {
    Cfg c;
    for (int k = 0; k < 16; k++) c.ht[k] = (uint16_t)rnd();
    static const int16_t xi[] = {2, -2, 0, 4, -4, 8, -8, 2, -2, 6};
    auto rinc = [&](int lim) -> int16_t { int r = rnd_range(0, 9); if (r < 7) return xi[rnd_range(0, 9)]; return (int16_t)(rnd_range(-lim, lim) & ~1); };
    c.sxi = rinc(64); c.dxi = rinc(64);
    c.syi = (int16_t)(rnd_range(-600, 600) & ~1);
    c.dyi = (int16_t)(rnd_range(-600, 600) & ~1);
    c.saddr = 0x300000 + (rnd() & 0x1FFFE);
    c.daddr = 0x500000 + (rnd() & 0x1FFFE);
    if (rnd_range(0, 9) == 0) c.daddr = c.saddr + (rnd_range(-64, 64) & ~1);   // overlapping
    c.saddr |= (rnd() & 1) ? 0x01000000u : 0u;                                  // bits above 23 ignored
    c.em1 = rnd_range(0, 3) == 0 ? 0xFFFF : (uint16_t)rnd();
    c.em2 = rnd_range(0, 2) == 0 ? 0xFFFF : (uint16_t)rnd();
    c.em3 = rnd_range(0, 3) == 0 ? 0xFFFF : (uint16_t)rnd();
    int r = rnd_range(0, 99);
    c.xc = r < 25 ? 1 : r < 35 ? 2 : (uint16_t)rnd_range(3, 24);
    c.yc = (uint16_t)rnd_range(1, 10);
    c.hop = (i >> 4) & 3;
    c.lop = i & 15;
    if (rnd_range(0, 3) == 0) { c.hop = rnd() & 3; c.lop = rnd() & 15; }
    bool hog = rnd() & 1;
    bool smudge = rnd() & 1;
    c.ctrl = 0x80 | (hog ? 0x40 : 0) | (smudge ? 0x20 : 0) | (rnd() & 0x10) | (rnd() & 15);
    c.skew = (uint8_t)rnd();
    // statistics
    stat_lop[c.lop]++; stat_hop[c.hop]++; if (smudge) stat_smudge_hop[c.hop]++;
    stat_skew[c.skew & 15]++; stat_fn[c.skew >> 6]++;
    if (c.xc == 1 && (c.skew & 0x40)) stat_x1nfsr++;
    if (hog) stat_hog++; else stat_nonhog++;
    stat_words += c.xc * c.yc;
    return c;
}

static void new_memory() {
    rtl_mem.w.clear(); gm_mem.w.clear();
    rtl_mem.seed = gm_mem.seed = rnd();
}

static void do_reset() {
    top->reset = 1; tick(); tick(); top->reset = 0; tick();
    gm_reset();
}

// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    int nrand = 3000;
    if (argc > 1 && argv[1][0] != '+') nrand = atoi(argv[1]);
    if (argc > 2 && argv[2][0] != '+') rng_state = strtoull(argv[2], nullptr, 0) | 1;
    printf("random blits %d, seed 0x%llx\n", nrand, (unsigned long long)rng_state);
    top = new Vfalcon_blitter;
    gm_init();
    top->clk = 0; top->reset = 1; top->bg = 0; top->bus_cs = 0; top->bus_stb = 0;
    top->dma_ack = 0; top->cpu_bus_cycle = 0;
    for (int i = 0; i < 4; i++) tick();
    top->reset = 0; tick();
    gm_reset();

    struct Res { std::string name; int err; };
    std::vector<Res> results;
    auto section = [&](const char *name, int e0) { results.push_back({name, errors - e0}); printf("%-58s %s\n", name, errors == e0 ? "PASS" : "FAIL"); };

    // ---- 1. reset values ----
    { int e0 = errors; compare_regs("reset"); CHECK(top->busy == 0 && top->br == 0, "busy/br after reset");
      section("T1 register values after reset", e0); }

    // ---- 2. register write / readback, masks, byte writes ----
    { int e0 = errors;
      for (int it = 0; it < 50; it++) {
          for (int off = 0; off < 0x3a; off += 2) reg_write(off, 2, (uint16_t)rnd());
          reg_write(0x3a, 2, (uint16_t)rnd());
          reg_write(0x3d, 1, (uint8_t)rnd());
          reg_write(0x3c, 1, (uint8_t)(rnd() & 0x7F));     // no start
          if (rnd_range(0, 4) == 0) reg_write(0x36, 2, 0);   // 0 -> 65536
          if (rnd_range(0, 4) == 0) reg_write(0x38, 2, 0);
          compare_regs("readback");
          // byte writes to word / long registers are ignored
          for (int k = 0; k < 8; k++) { int off = rnd_range(0, 0x39); reg_write(off, 1, (uint8_t)rnd()); }
          compare_regs("byte write");
      }
      section("T2 register readback, masks, ignored byte writes", e0); }

    // ---- 3. random blits vs Hatari ----
    { int e0 = errors;
      for (int i = 0; i < nrand; i++) {
          new_memory();
          bg_lat_on = rnd_range(1, 4); bg_lat_off = rnd_range(1, 3); ram_lat_max = rnd_range(1, 10);
          pulse_min = 1; pulse_max = rnd_range(1, 4);
          Cfg c = random_cfg(i);
          write_cfg(c);
          char ctx[64]; snprintf(ctx, sizeof ctx, "blit %d (hop %d lop %x ctrl %02x skew %02x x %d y %d)", i, c.hop, c.lop, c.ctrl, c.skew, c.xc, c.yc);
          int e1 = errors;
          run_blit(c, -1, ctx);
          if (errors != e1 && errors < 40) printf("  config: sxi %d syi %d dxi %d dyi %d src %06x dst %06x em %04x %04x %04x\n",
                                                c.sxi, c.syi, c.dxi, c.dyi, c.saddr, c.daddr, c.em1, c.em2, c.em3);
      }
      char nm[96]; snprintf(nm, sizeof nm, "T3 %d random blits vs Hatari blitter.c", nrand);
      section(nm, e0); }

    // ---- 4. busy written with y count = 0 ----
    { int e0 = errors;
      // previous blit finished: y count is 0
      CHECK(gm_read(0x38) == 0, "y count not 0 after a blit");
      reg_write(0x3c, 1, 0xE5);
      CHECK(gm_int_pending == 0, "Hatari scheduled a pass");
      for (int i = 0; i < 50; i++) { tick(); CHECK(!top->br && !top->busy, "blitter started with y count = 0"); }
      compare_regs("start with y=0");
      CHECK((rtl_read(0x3c) >> 8) == 0x25, "control after start with y=0: %02x expected 25", rtl_read(0x3c) >> 8);
      section("T4 start with y count = 0 clears busy and hog", e0); }

    // ---- 5. timeout when the CPU makes no bus cycles ----
    { int e0 = errors;
      for (int i = 0; i < 20; i++) {
          new_memory();
          Cfg c = random_cfg(rnd());
          c.ctrl &= ~0x40; c.xc = 20; c.yc = 10;
          write_cfg(c);
          run_blit(c, ACT_TIMEOUT, "timeout");
      }
      section("T5 non-hog restart by timeout (no CPU bus cycles)", e0); }

    // ---- 6. every pass action, deterministic ----
    { int e0 = errors;
      const int acts[] = {ACT_NONE, ACT_RESTART, ACT_PAUSE, ACT_MODIFY};
      for (int a = 0; a < 4; a++) for (int i = 0; i < 20; i++) {
          new_memory();
          Cfg c = random_cfg(rnd());
          c.ctrl &= ~0x40; c.xc = rnd_range(1, 30); c.yc = rnd_range(3, 8);
          write_cfg(c);
          run_blit(c, acts[a], "action");
      }
      section("T6 non-hog: 64/64 sharing, restart (bset), pause/resume, modify", e0); }

    // ---- 7. x count / y count = 65536 ----
    { int e0 = errors;
      new_memory();
      Cfg c = random_cfg(3); c.xc = 0; c.yc = 1; c.ctrl = 0xC0; c.sxi = 2; c.dxi = 2; c.lop = 6; c.hop = 2;
      write_cfg(c); run_blit(c, ACT_NONE, "xcount=65536 hog");
      new_memory();
      c = random_cfg(5); c.xc = 1; c.yc = 0; c.ctrl = 0xC0; c.syi = 2; c.dyi = 2; c.lop = 3; c.hop = 3; c.skew = 0x47;
      write_cfg(c); run_blit(c, ACT_NONE, "ycount=65536 hog");
      new_memory();
      c = random_cfg(7); c.xc = 0; c.yc = 1; c.ctrl = 0x80; c.sxi = -2; c.dxi = -2; c.lop = 7;
      write_cfg(c); run_blit(c, ACT_NONE, "xcount=65536 non-hog");
      section("T7 x count / y count written as 0 (65536)", e0); }

    // ---- 8. reset while the blitter waits for the CPU share ----
    { int e0 = errors;
      new_memory();
      Cfg c = random_cfg(9); c.ctrl = 0x80; c.xc = 30; c.yc = 10;
      write_cfg(c);
      reg_write(0x3c, 1, c.ctrl);
      gm_trace.clear(); rtl_trace.clear();
      int ng = gm_run_pass();
      wait_until(c_br, 100); wait_until(c_brlow_or_done, 100000);
      CHECK(ng == 64 && top->busy, "first pass");
      do_reset();
      CHECK(!top->busy && !top->br, "busy/br after reset: %d %d", top->busy, top->br);
      cpu_pulses_on = true; for (int i = 0; i < 1000; i++) { tick(); CHECK(!top->br, "bus request after reset"); } cpu_pulses_on = false;
      compare_regs("after reset mid-blit");
      // and the blitter works normally afterwards
      new_memory(); c = random_cfg(11); write_cfg(c); run_blit(c, -1, "after reset");
      section("T8 reset during a non-hog blit", e0); }

    printf("\nstatistics: passes %d words %d restart %d pause %d modify %d timeout %d hog %d nonhog %d xcount1+nfsr %d\n",
           stat_passes, stat_words, stat_restart, stat_pause, stat_modify, stat_timeout, stat_hog, stat_nonhog, stat_x1nfsr);
    printf("lop counts:"); for (int i = 0; i < 16; i++) printf(" %d", stat_lop[i]); printf("\n");
    printf("hop counts:"); for (int i = 0; i < 4; i++) printf(" %d(smudge %d)", stat_hop[i], stat_smudge_hop[i]); printf("\n");
    printf("skew counts:"); for (int i = 0; i < 16; i++) printf(" %d", stat_skew[i]); printf("\n");
    printf("fxsr/nfsr counts: --:%d -N:%d F-:%d FN:%d\n", stat_fn[0], stat_fn[1], stat_fn[2], stat_fn[3]);
    printf("\nchecks: %d  errors: %d\n", checks, errors);
    int nf = 0; for (auto &r : results) if (r.err) nf++;
    printf("SUMMARY: %zu sections, %d failed -> %s\n", results.size(), nf, errors ? "FAIL" : "PASS");
    delete top;
    return errors ? 1 : 0;
}
