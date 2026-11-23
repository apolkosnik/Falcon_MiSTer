// The ATW800/2's 2D engine against the same commands done byte by byte in C.
//   tb_vme [--mem 2|4] [--n COUNT] [--seed S] [--old] [--vid N] [+ddrseed=..., +ddrbusy=..., +ddrlat=...]
// --old: what the 16-bit engine can be held to: no fill over its own source, no command 7,
//   no copy to one byte after its source in the command's order.
// --vid N: a video burst asks for the DDR3 one clock in N, as the TT's shifter does.
// Random commands, most with source and destination close together, some across the end of
// the card's memory; then two fixed ones for the speed.
#include "Vtb_vme_top.h"
#include "Vtb_vme_top__Dpi.h"
#include "verilated.h"
#include "svdpi.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>

static Vtb_vme_top *top;
static uint64_t clocks;
static uint64_t rng = 0x9E3779B97F4A7C15ull;
static unsigned vid_every;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (uint32_t)(rng >> 16); }
static int rr(int lo, int hi) { return lo + (int)(rnd() % (unsigned)(hi - lo + 1)); }

static void tick() {
    if (vid_every) {
        if (top->vid_ack) top->vid_req = 0;
        else if (rnd() % vid_every == 0) top->vid_req = 1;
    }
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
    clocks++;
}
static void wr(unsigned off, unsigned v) {            // a word of the top 4 KB
    top->bus_addr = off >> 1; top->bus_din = v; top->bus_we = 1; top->bus_uds = 1; top->bus_lds = 1;
    top->bus_stb = 1; tick(); top->bus_stb = 0; top->bus_we = 0; tick();
}

#define BASE 0xA00000u
static std::vector<uint8_t> sh;                        // the card's memory, as it should be
static uint32_t mask;
static uint8_t peek(uint32_t o) { uint32_t a = BASE + o; return (uint8_t)(ddr_peek(a >> 3) >> (8 * (a & 7))); }
static void poke(uint32_t o, uint8_t v) { uint32_t a = BASE + o; ddr_poke(a >> 3, (uint64_t)v << (8 * (a & 7)), (uint8_t)(1u << (a & 7))); }

struct Cmd { uint32_t src, dst; int16_t ss, ds; uint16_t bytes, rows; int cmd; };

static void model(const Cmd &c) {
    bool fill = c.cmd & 4, dir = (c.cmd & 2) && !fill;
    if (!c.bytes || !c.rows) return;
    uint32_t s = c.src, d = c.dst;
    for (unsigned r = 0; r < c.rows; r++) {
        uint8_t w[64];
        for (unsigned x = 0; x < c.bytes; x++) {
            uint8_t v;
            if (fill && x >= 64) v = w[32 + (x & 31)];
            else v = sh[(dir ? s - x : s + x) & mask];
            if (fill && x < 64) w[x] = v;
            sh[(dir ? d - x : d + x) & mask] = v;
        }
        s += (int32_t)c.ss; d += (int32_t)c.ds;
    }
}

static uint64_t run(const Cmd &c) {
    wr(0x900, c.src >> 16); wr(0x902, c.src & 0xFFFF);
    wr(0x904, c.dst >> 16); wr(0x906, c.dst & 0xFFFF);
    wr(0x908, (uint16_t)c.ss); wr(0x90A, (uint16_t)c.ds);
    wr(0x90C, c.bytes); wr(0x90E, c.rows);
    uint64_t t0 = clocks;
    wr(0x910, c.cmd);
    while (top->busy) {
        tick();
        if (clocks - t0 > 40000000) { printf("FAIL: the engine did not finish\n"); exit(1); }
    }
    uint64_t t = clocks - t0;
    tick();
    while (top->ddr_we_wait) tick();                  // the last write is acknowledged as it is issued
    tick();
    return t;
}

static int compare(uint32_t centre, uint32_t half, const Cmd &c, int n) {
    int bad = 0;
    for (uint32_t i = 0; i < 2 * half; i++) {
        uint32_t o = (centre - half + i) & mask;
        uint8_t got = peek(o);
        if (got != sh[o]) {
            if (!bad) printf("FAIL #%d: cmd %d src %06x dst %06x strides %d %d bytes %u rows %u: at %06x %02x, should be %02x\n",
                             n, c.cmd, c.src, c.dst, c.ss, c.ds, c.bytes, c.rows, o, got, sh[o]);
            bad++;
        }
    }
    if (bad) for (uint32_t i = 0; i < 2 * half; i++) { uint32_t o = (centre - half + i) & mask; poke(o, sh[o]); }
    return bad;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    int mem = 4, count = 3000; bool old = false;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--mem")) mem = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--n")) count = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seed")) rng ^= strtoull(argv[++i], 0, 0) * 0x2545F4914F6CDD1Dull;
        else if (!strcmp(argv[i], "--old")) old = true;
        else if (!strcmp(argv[i], "--vid")) vid_every = atoi(argv[++i]);
    }
    top = new Vtb_vme_top;
    svSetScope(svGetScopeFromName("TOP.tb_vme_top.ddr"));
    top->mem4 = mem == 4; top->reset = 1;
    for (int i = 0; i < 8; i++) tick();
    top->reset = 0; tick();
    if (mem == 4) wr(0x81E, 3);                        // the 4 MB layout
    uint32_t size = mem == 4 ? 0x400000 : 0x200000;
    mask = size - 1;
    if ((bool)top->lay4 != (mem == 4)) { printf("FAIL: layout\n"); return 1; }
    sh.assign(size, 0);

    const uint32_t HALF = 0x4000;
    const uint32_t centres[2] = { 0x048000, 0 };       // the second lies across the end of the memory
    for (int k = 0; k < 2; k++)
        for (uint32_t i = 0; i < 2 * HALF; i++) { uint32_t o = (centres[k] - HALF + i) & mask; sh[o] = rnd(); poke(o, sh[o]); }

    int fails = 0, ncmd[8] = {0};
    for (int n = 0; n < count; n++) {
        Cmd c;
        int k = rr(0, 3) == 0;
        static const int cmds[] = { 1, 1, 3, 3, 5, 5, 5, 7 };
        do c.cmd = cmds[rr(0, 7)]; while (old && c.cmd == 7);
        bool fill = c.cmd & 4, dir = (c.cmd & 2) && !fill;
        int t = rr(0, 9);
        c.bytes = t < 4 ? rr(1, 20) : t < 8 ? rr(21, 200) : rr(201, 600);
        if (rr(0, 49) == 0) c.bytes = 0;
        c.rows = rr(1, 4);
        if (rr(0, 49) == 0) c.rows = 0;
        static const int st[] = { 0, 1, -1, 8, -8, 256, -256, 1024, -1024 };
        t = rr(0, 3);
        c.ss = t == 0 ? st[rr(0, 8)] : t == 1 ? rr(-700, 700) : dir ? -1024 : 1024;
        c.ds = t == 0 ? st[rr(0, 8)] : t == 1 ? rr(-700, 700) : dir ? -1024 : 1024;
        if (fill && rr(0, 1)) c.ss = 0;
        int32_t s0 = rr(-2000, 2000), d0;
        t = rr(0, 3);
        d0 = t < 2 ? s0 + rr(-16, 16) : t == 2 ? s0 + rr(-300, 300) : s0 + (rr(0, 1) ? 1 : -1) * rr(700, 3000);
        if (old && fill) d0 = s0 + (rr(0, 1) ? 1 : -1) * (5200 + rr(0, 500));   // clear of every row's source
        // the 16-bit engine reads two bytes, then writes them: one byte apart is not byte by byte
        if (old && !fill) {
            for (int r = 1; r < c.rows; r++)
                if ((dir ? -1 : 1) * ((d0 + r * c.ds) - (s0 + r * c.ss)) == 1) c.ds = c.ss;
            if ((dir ? s0 - d0 : d0 - s0) == 1) d0 += dir ? -1 : 1;
        }
        c.src = (centres[k] + s0) & 0x3FFFFF; c.dst = (centres[k] + d0) & 0x3FFFFF;
        model(c);
        run(c);
        ncmd[c.cmd]++;
        if (compare(centres[k], HALF, c, n)) { fails++; if (fails >= 10) break; }
    }
    printf("%d commands (copy %d, descending %d, fill %d, fill with bit 1 %d): %d wrong\n",
           ncmd[1] + ncmd[3] + ncmd[5] + ncmd[7], ncmd[1], ncmd[3], ncmd[5], ncmd[7], fails);

    // the speed: 16 rows of 1024, source and destination not aligned alike
    Cmd c = { 0x048003, 0x04C008, 1024, 1024, 1024, 16, 1 };
    for (uint32_t i = 0; i < 0x10000; i++) { uint32_t o = 0x040000 + i; sh[o] = rnd(); poke(o, sh[o]); }
    model(c); uint64_t t = run(c);
    if (compare(0x048000, 0x8000, c, -1)) fails++;
    printf("copy 16 x 1024: %.2f clocks a byte, %.1f MB/s at 32 MHz\n", t / 16384.0, 32.0 * 16384 / t);
    c = { 0x048003, 0x04C008, 0, 1024, 1024, 16, 5 };
    model(c); t = run(c);
    if (compare(0x048000, 0x8000, c, -2)) fails++;
    printf("fill 16 x 1024: %.2f clocks a byte, %.1f MB/s at 32 MHz\n", t / 16384.0, 32.0 * 16384 / t);
    printf(fails ? "FAILED\n" : "PASSED\n");
    return fails != 0;
}
