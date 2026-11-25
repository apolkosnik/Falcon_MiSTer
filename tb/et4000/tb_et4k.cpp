// The Nova ET4000 card against a model of the chip in C (the VGA's register file and data
// path as the Hatari fork's video_et4000.c has them, from PCem), through the card's bus,
// the memory arbiter and the DDR3 model.
//   tb_et4k [--n SCENARIOS] [--ops OPS] [--seed S] [--vid N]
// 1. the registers: what reads back, the attribute controller's flip-flop, the DAC's read
//    and write sequences, the Sierra HiColor command register, the CRTC's protection,
//    ports the chip does not have
// 2. input status 1: the vertical retrace at 60 Hz
// 3. the memory: random setups of the data path (chain-4, odd/even, planar; the four write
//    modes, rotation, set/reset, the logical operations, the bit mask, the map mask; read
//    modes 0 and 1, the read plane; the linear window or the segment register) and random
//    byte and word reads and writes in them; every read's value, then the video memory and
//    the planar modes' shadow in DDR3
// 4. the picture: EmuTOS's 640x480 mono mode and a 640x480 256 colour mode, what the
//    scaler is told and its palette; HiColor; blanking; the machine's reset
// --vid N: a video burst asks for the DDR3 one clock in N, as the TT's shifter does.
#include "Vtb_et4k_top.h"
#include "Vtb_et4k_top__Dpi.h"
#include "verilated.h"
#include "svdpi.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <vector>
#include <set>

static Vtb_et4k_top *top;
static uint64_t clocks;
static uint64_t rng = 0x9E3779B97F4A7C15ull;
static unsigned vid_every;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return (uint32_t)(rng >> 16); }
static int rr(int lo, int hi) { return lo + (int)(rnd() % (unsigned)(hi - lo + 1)); }
static int fails;
static int max_cycle;             // the longest bus cycle, in clocks
#define CHECK(c, ...) do { if (!(c)) { printf("FAIL: " __VA_ARGS__); printf("\n"); fails++; } } while (0)

static uint8_t hpal[256][3];         // the scaler's palette, as written
static void tick() {
    if (vid_every) {
        if (top->vid_ack) top->vid_req = 0;
        else if (rnd() % vid_every == 0) top->vid_req = 1;
    }
    top->clk = 0; top->eval();
    if (top->pal_wr) {
        hpal[top->pal_addr][0] = top->pal_rgb >> 16;
        hpal[top->pal_addr][1] = top->pal_rgb >> 8;
        hpal[top->pal_addr][2] = top->pal_rgb;
    }
    top->clk = 1; top->eval();
    clocks++;
}

// one bus cycle; returns the word read
static uint16_t cycle(bool we, bool mem, uint32_t off, bool uds, bool lds, uint16_t din) {
    top->bus_we = we; top->bus_mem = mem; top->bus_addr = (off >> 1) & 0x7FFFF;
    top->bus_uds = uds; top->bus_lds = lds; top->bus_din = din;
    top->bus_cs = 1; top->bus_stb = 1; tick(); top->bus_stb = 0;
    int n = 0;
    while (!top->bus_ack) {
        tick();
        if (++n > 100000) { printf("FAIL: no acknowledge (off %05x)\n", off); exit(1); }
    }
    if (n > max_cycle) max_cycle = n;
    uint16_t d = top->bus_dout;
    tick(); top->bus_cs = 0; tick();
    return d;
}
static void outb(uint16_t p, uint8_t v) { cycle(true, false, p & ~1, !(p & 1), p & 1, (p & 1) ? v : v << 8); }
static uint8_t inb(uint16_t p) { uint16_t d = cycle(false, false, p & ~1, !(p & 1), p & 1, 0); return (p & 1) ? d : d >> 8; }

static uint8_t peek(uint32_t a) { return (uint8_t)(ddr_peek(a >> 3) >> (8 * (a & 7))); }

//----------------------------------------------------------------------------
// The model
//----------------------------------------------------------------------------
struct Et {
    uint8_t misc, vidsub, featcon, banking, seqaddr, seq[8], gdcaddr, gdc[9];
    uint8_t attraddr, palen, ff, attr[23], crtcaddr, crtc[64];
    uint8_t dacmask, dacread, dacwrite, dacstatus, dacr, dacg, dacpos, rdstate, rdctrl;
    int bpp;
    uint8_t pal[256][3];
    uint8_t la, lb, lc, ld;
    std::vector<uint8_t> vram, shadow;
    void reset() {
        misc = vidsub = featcon = banking = seqaddr = gdcaddr = attraddr = palen = ff = crtcaddr = 0;
        memset(seq, 0, sizeof seq); memset(gdc, 0, sizeof gdc); memset(attr, 0, sizeof attr);
        memset(crtc, 0, sizeof crtc);
        dacmask = 0xFF; dacread = dacwrite = dacstatus = dacpos = rdstate = rdctrl = 0; bpp = 8;
        la = lb = lc = ld = 0;
    }
    void out(uint16_t p, uint8_t v) {
        switch (p) {
        case 0x3C0:
            if (!ff) { attraddr = v & 31; palen = v & 0x20; }
            else if (attraddr < 23) attr[attraddr] = v;
            ff ^= 1; break;
        case 0x3C2: misc = v; break;
        case 0x3C3: vidsub = v; break;
        case 0x3C4: seqaddr = v & 7; break;
        case 0x3C5: seq[seqaddr] = v; break;
        case 0x3C6:
            if (rdstate == 4) {
                if (v != 0xFF) {
                    rdctrl = v;
                    switch ((v & 1) | ((v & 0xC0) >> 5)) {
                    case 0: bpp = 8; break;
                    case 2: case 3: bpp = 24; break;
                    case 4: case 5: bpp = 15; break;
                    case 6: bpp = 16; break;
                    case 7: bpp = (v & 4) ? 24 : 16; break;
                    default: break;
                    }
                }
            } else dacmask = v;
            rdstate = 0; break;
        case 0x3C7: rdstate = 0; dacread = v; dacpos = 0; break;
        case 0x3C8: rdstate = 0; dacwrite = v; dacread = v - 1; dacpos = 0; break;
        case 0x3C9:
            rdstate = 0; dacstatus = 0;
            if (dacpos == 0) { dacr = v; dacpos = 1; }
            else if (dacpos == 1) { dacg = v; dacpos = 2; }
            else { pal[dacwrite][0] = dacr & 63; pal[dacwrite][1] = dacg & 63; pal[dacwrite][2] = v & 63; dacwrite++; dacpos = 0; }
            break;
        case 0x3CD: banking = v; break;
        case 0x3CE: gdcaddr = v & 15; break;
        case 0x3CF: if (gdcaddr < 9) gdc[gdcaddr] = v; break;
        case 0x3B4: case 0x3D4: crtcaddr = v & 63; break;
        case 0x3B5: case 0x3D5:
            if (crtcaddr < 7 && (crtc[0x11] & 0x80)) break;
            if (crtcaddr == 7 && (crtc[0x11] & 0x80)) v = (crtc[7] & ~0x10) | (v & 0x10);
            crtc[crtcaddr] = v; break;
        case 0x3BA: case 0x3DA: featcon = v; break;
        default: break;
        }
    }
    // returns -1 for input status 1 (the timing is the card's own)
    int in(uint16_t p) {
        int v = 0xFF;
        switch (p) {
        case 0x3C0: v = attraddr | palen; break;
        case 0x3C1: v = attraddr < 23 ? attr[attraddr] : 0; break;
        case 0x3C2: v = 0x10; break;
        case 0x3C3: v = vidsub; break;
        case 0x3C4: v = seqaddr; break;
        case 0x3C5: v = seq[seqaddr] | (seqaddr == 7 ? 4 : 0); break;
        case 0x3C6: if (rdstate == 4) v = rdctrl; else { rdstate++; v = dacmask; } break;
        case 0x3C7: rdstate = 0; v = dacstatus; break;
        case 0x3C8: rdstate = 0; v = dacwrite; break;
        case 0x3C9:
            rdstate = 0; dacstatus = 3;
            v = pal[dacread][dacpos];
            if (dacpos == 2) { dacpos = 0; dacread++; } else dacpos++;
            break;
        case 0x3CA: v = featcon; break;
        case 0x3CC: v = misc; break;
        case 0x3CD: v = banking; break;
        case 0x3CE: v = gdcaddr; break;
        case 0x3CF: v = gdcaddr < 9 ? gdc[gdcaddr] : 0; break;
        case 0x3B4: case 0x3D4: v = crtcaddr; break;
        case 0x3B5: case 0x3D5: v = crtc[crtcaddr]; break;
        case 0x3BA: case 0x3DA: ff = 0; v = -1; break;
        default: break;
        }
        return v;
    }
    uint32_t lin(uint32_t off, bool wr) {
        if (crtc[0x36] & 0x10) return off;
        return (off & 0xFFFF) | ((wr ? banking & 15 : banking >> 4) << 16);
    }
    static uint8_t rot(uint8_t v, int n) { return n ? (uint8_t)((v >> n) | (v << (8 - n))) : v; }
    uint8_t op(uint8_t d, uint8_t l, uint8_t m) {
        switch (gdc[3] & 0x18) {
        case 0x00: return (d & m) | (l & ~m);
        case 0x08: return (d | ~m) & l;
        case 0x10: return (d & m) | l;
        default:   return (d & m) ^ l;
        }
    }
    // returns the 32-bit word written (planar), for the shadow
    void wrb(uint32_t off, uint8_t val) {
        uint32_t a = lin(off, true);
        int wm = seq[2] & 15;
        bool planar = false;
        if (seq[4] & 8) { wm = 1 << (a & 3); a &= ~3u; }
        else if (!(seq[4] & 4)) { wm &= ~0xA; if (a & 1) wm <<= 1; a &= ~1u; a <<= 2; planar = true; }
        else { a <<= 2; planar = true; }
        if (a >= 0x100000) return;
        uint8_t l[4] = { la, lb, lc, ld }, d[4], m = gdc[8];
        int mode = gdc[5] & 3;
        uint8_t r = rot(val, gdc[3] & 7);
        if (mode == 3) m = gdc[8] & r;
        for (int p = 0; p < 4; p++) {
            if (mode == 0) d[p] = (gdc[1] >> p & 1) ? ((gdc[0] >> p & 1) ? 0xFF : 0) : r;
            else if (mode == 2) d[p] = (val >> p & 1) ? 0xFF : 0;
            else d[p] = (gdc[0] >> p & 1) ? 0xFF : 0;
            if (wm >> p & 1) vram[a | p] = mode == 1 ? l[p] : op(d[p], l[p], m);
        }
        if (planar && wm) {
            uint32_t w = a >> 2;
            for (int k = 0; k < 8; k++) {
                int b = 7 - k;
                shadow[w * 8 + k] = (vram[a] >> b & 1) | (vram[a | 1] >> b & 1) << 1 | (vram[a | 2] >> b & 1) << 2 | (vram[a | 3] >> b & 1) << 3;
            }
        }
    }
    uint8_t rdb(uint32_t off) {
        uint32_t a = lin(off, false);
        int rp = gdc[4] & 3;
        if (seq[4] & 8) return a < 0x100000 ? vram[a] : 0xFF;
        if (gdc[5] & 0x10) { rp = (rp & 2) | (a & 1); a &= ~1u; a <<= 2; }
        else a <<= 2;
        if (a >= 0x100000) { la = lb = lc = ld = 0xFF; return 0xFF; }
        la = vram[a]; lb = vram[a | 1]; lc = vram[a | 2]; ld = vram[a | 3];
        if (gdc[5] & 8) {
            uint8_t t = 0, l[4] = { la, lb, lc, ld };
            for (int p = 0; p < 4; p++) t |= (l[p] ^ ((gdc[2] >> p & 1) ? 0xFF : 0)) & ((gdc[7] >> p & 1) ? 0xFF : 0);
            return ~t;
        }
        return vram[a | rp];
    }
};
static Et et;

static void out(uint16_t p, uint8_t v) { et.out(p, v); outb(p, v); }
static void idx(uint16_t p, uint8_t i, uint8_t v) { out(p, i); out(p + 1, v); }
static void in_check(uint16_t p, const char *what) {
    int want = et.in(p);
    uint8_t got = inb(p);
    if (want >= 0) CHECK(got == want, "%s: port %03x reads %02x, should be %02x", what, p, got, want);
    else CHECK((got & 0xF6) == 0, "%s: input status 1 reads %02x", what, got);
}
static void atc(uint8_t i, uint8_t v) { et.in(0x3DA); inb(0x3DA); out(0x3C0, i); out(0x3C0, v); }

//----------------------------------------------------------------------------
// 1. Registers
//----------------------------------------------------------------------------
static void test_registers() {
    int f0 = fails;
    for (int i = 0; i < 8; i++) { idx(0x3C4, i, rnd()); }
    for (int i = 0; i < 8; i++) { out(0x3C4, i); in_check(0x3C5, "sequencer"); }
    for (int i = 0; i < 16; i++) { idx(0x3CE, i, rnd()); }
    for (int i = 0; i < 16; i++) { out(0x3CE, i); in_check(0x3CF, "graphics controller"); }
    idx(0x3D4, 0x11, 0x00);
    for (int i = 0; i < 64; i++) idx(i & 1 ? 0x3D4 : 0x3B4, i, rnd());
    idx(0x3D4, 0x11, 0x80);                           // protection on: 0-6 kept, 7 bit 4 only
    for (int i = 0; i < 8; i++) idx(0x3D4, i, rnd());
    for (int i = 0; i < 64; i++) { out(0x3D4, i); in_check(i & 1 ? 0x3B5 : 0x3D5, "CRTC"); }
    idx(0x3D4, 0x11, 0x00);
    // the attribute controller: index, then data, by the flip-flop; 3DA resets it
    for (int i = 0; i < 23; i++) atc(i, rnd());
    atc(0x0A, 0x55);
    in_check(0x3C0, "attribute index");
    for (int i = 0; i < 23; i++) { et.in(0x3DA); inb(0x3DA); out(0x3C0, i); in_check(0x3C1, "attribute data"); out(0x3C0, rnd()); }
    out(0x3C0, 0x20);                                 // flip-flop at index: the screen on
    in_check(0x3C0, "attribute index with the palette enable");
    // the DAC: write 256 entries, read them back, from another start too
    out(0x3C8, 0);
    for (int i = 0; i < 768; i++) out(0x3C9, rnd());
    out(0x3C7, 0);
    for (int i = 0; i < 768; i++) in_check(0x3C9, "DAC read");
    out(0x3C7, 200);
    for (int i = 0; i < 20; i++) in_check(0x3C9, "DAC read from 200");
    in_check(0x3C7, "DAC status"); in_check(0x3C8, "DAC write index");
    out(0x3C8, 17); out(0x3C9, 1); out(0x3C9, 2);      // a write cut short by a new index
    out(0x3C8, 18); out(0x3C9, 3); out(0x3C9, 4); out(0x3C9, 5);
    out(0x3C7, 17); for (int i = 0; i < 6; i++) in_check(0x3C9, "DAC after a short write");
    // the Sierra command register: four reads of 3C6, then a write; 3C6 then reads it
    out(0x3C6, 0xF0); in_check(0x3C6, "pixel mask");
    for (int i = 0; i < 4; i++) in_check(0x3C6, "pixel mask, arming");
    out(0x3C6, 0xA0);
    for (int i = 0; i < 4; i++) in_check(0x3C6, "pixel mask, arming");
    in_check(0x3C6, "HiColor command");
    in_check(0x3C8, "disarm");
    in_check(0x3C6, "pixel mask after");
    out(0x3C6, 0xFF);
    // the rest
    out(0x3C2, 0x63); in_check(0x3CC, "misc output"); in_check(0x3C2, "input status 0");
    out(0x3C3, 0x01); in_check(0x3C3, "video subsystem");
    out(0x3CD, 0x5A); in_check(0x3CD, "segment select");
    out(0x3DA, 0x0F); in_check(0x3CA, "feature control");
    static const uint16_t none[] = { 0x3C3 ^ 0x100, 0x3D0, 0x46E8, 0x6EEC, 0x0000, 0x3DF, 0x3B0 };
    for (uint16_t p : none) in_check(p, "a port the chip does not have");
    // word cycles: two bytes, the even port first
    out(0x3C4, 2); et.out(0x3C4, 2);
    { uint16_t d = cycle(false, false, 0x3C4, true, true, 0);
      CHECK(d == ((2 << 8) | et.seq[2]), "word read of 3C4/3C5: %04x", d); }
    cycle(true, false, 0x3CE, true, true, (8 << 8) | 0x3C); et.out(0x3CE, 8); et.out(0x3CF, 0x3C);
    out(0x3CE, 8); in_check(0x3CF, "word write of 3CE/3CF");
    printf("registers: %s\n", fails == f0 ? "pass" : "FAIL");
}

//----------------------------------------------------------------------------
// 2. Input status 1
//----------------------------------------------------------------------------
static void test_status() {
    int edges = 0, last = 0, polls = 0;
    uint64_t t0 = clocks;
    while (clocks - t0 < 32000000ull / 10) {          // 100 ms
        int v = inb(0x3DA) & 8;
        if (v && !last) edges++;
        last = v; polls++;
    }
    CHECK(edges >= 5 && edges <= 7, "vertical retrace: %d in 100 ms", edges);
    printf("input status 1: %d vertical retraces in 100 ms (%d polls)\n", edges, polls);
}

//----------------------------------------------------------------------------
// 3. Memory
//----------------------------------------------------------------------------
static std::set<uint32_t> touched, shtouched;
static void compare_mem(const char *what) {
    // drain the posted writes
    for (int i = 0; i < 8; i++) tick();
    while (top->ddr_we_wait) tick();
    int bad = 0;
    for (uint32_t a : touched) {
        uint8_t got = peek(0xC00000 + a);
        if (got != et.vram[a]) { if (bad < 5) printf("FAIL: %s: video memory %05x is %02x, should be %02x\n", what, a, got, et.vram[a]); bad++; }
    }
    for (uint32_t a : shtouched) {
        uint8_t got = peek(0xA00000 + a);
        if (got != et.shadow[a]) { if (bad < 10) printf("FAIL: %s: shadow %06x is %02x, should be %02x\n", what, a, got, et.shadow[a]); bad++; }
    }
    if (bad) fails++;
}

static void track(uint32_t off) {
    uint32_t a = et.lin(off, true);
    if (et.seq[4] & 8) { touched.insert(a); return; }
    if (!(et.seq[4] & 4)) a &= ~1u;
    a <<= 2;
    if (a >= 0x100000) return;
    for (int p = 0; p < 4; p++) touched.insert(a | p);
    for (int k = 0; k < 8; k++) shtouched.insert((a >> 2) * 8 + k);
}

static int nsc[4][2];
static void scenario(int n, int ops) {
    // the data path
    int kind = rr(0, 9);           // 0-3 chain-4, 4-5 odd/even, 6-9 planar
    uint8_t s4 = kind < 4 ? 0x0E : kind < 6 ? 0x02 : 0x06;
    if (rr(0, 7) == 0) s4 ^= 0x08;
    idx(0x3C4, 4, s4);
    idx(0x3C4, 2, rr(0, 3) ? 0x0F : rnd() & 15);
    idx(0x3CE, 0, rnd() & 15);
    idx(0x3CE, 1, rr(0, 1) ? 0 : rnd() & 15);
    idx(0x3CE, 2, rnd() & 15);
    idx(0x3CE, 3, rr(0, 2) ? 0 : rnd() & 0x1F);
    idx(0x3CE, 4, rnd() & 3);
    int wmode = rr(0, 3), rmode = rr(0, 1), oe = rr(0, 1);
    if (kind >= 4 && kind < 6) oe = 1;
    idx(0x3CE, 5, wmode | rmode << 3 | oe << 4 | (rnd() & 0x60));
    idx(0x3CE, 7, rnd() & 15);
    idx(0x3CE, 8, rr(0, 2) ? 0xFF : rnd());
    bool linear = rr(0, 4) != 0;
    idx(0x3D4, 0x36, linear ? 0xD3 : 0xC3);
    out(0x3CD, rnd());
    nsc[wmode][rmode]++;

    // where: near the start, the planar end at 256 KB, the segments, the window's end
    static const uint32_t bases[] = { 0x00000, 0x3FFC0, 0x0FFC0, 0x40000, 0xFFFC0, 0x12340, 0x7FF80 };
    uint32_t base = bases[rr(0, 6)];
    for (int i = 0; i < ops; i++) {
        uint32_t off = (base + rr(0, 127)) & 0xFFFFF;
        int t = rr(0, 9);
        bool we = t < 5;
        bool word = rr(0, 1);
        if (word) off &= ~1u;
        bool uds = word || !(off & 1), lds = word || (off & 1);
        if (we) {
            uint16_t d = rnd();
            if (uds) { track(off & ~1u); et.wrb(off & ~1u, d >> 8); }
            if (lds) { track(off | 1); et.wrb(off | 1, d); }
            cycle(true, true, off & ~1u, uds, lds, d);
        } else {
            int w0 = -1, w1 = -1;
            if (uds) w0 = et.rdb(off & ~1u);
            if (lds) w1 = et.rdb(off | 1);
            uint16_t d = cycle(false, true, off & ~1u, uds, lds, 0);
            if (uds) CHECK((d >> 8) == w0, "scenario %d op %d: read of %05x gave %02x, should be %02x (seq4 %02x gdc5 %02x)", n, i, off & ~1u, d >> 8, w0, s4, et.gdc[5]);
            if (lds) CHECK((d & 0xFF) == w1, "scenario %d op %d: read of %05x gave %02x, should be %02x (seq4 %02x gdc5 %02x)", n, i, off | 1, d & 0xFF, w1, s4, et.gdc[5]);
            if (fails > 20) { printf("too many failures\n"); exit(1); }
        }
        // the latches now and then from somewhere else
        if (rr(0, 15) == 0) {
            uint32_t o = rr(0, 0x3FFFF);
            et.rdb(o); cycle(false, true, o & ~1u, !(o & 1), o & 1, 0);
        }
    }
    compare_mem("memory");
}

//----------------------------------------------------------------------------
// 4. The picture
//----------------------------------------------------------------------------
static void settle() { for (int i = 0; i < 2000; i++) tick(); }
static uint8_t c6to8(uint8_t c) { return (c << 2) | (c >> 4); }
static void check_pal(int n, const uint8_t *map, const char *what) {
    int bad = 0;
    for (int i = 0; i < n; i++) {
        int j = map[i] & et.dacmask;
        for (int c = 0; c < 3; c++)
            if (hpal[i][c] != c6to8(et.pal[j][c])) { if (!bad) printf("FAIL: %s: palette %02x is %02x%02x%02x, should be DAC %02x\n", what, i, hpal[i][0], hpal[i][1], hpal[i][2], j); bad++; break; }
    }
    if (bad) fails++;
}

// EmuTOS's mode (bios/nova.c): 640x480, one plane, colour 0 white, 1 black
static void emutos_mode() {
    out(0x3C3, 0x01); out(0x3C2, 0xE3);
    // ramdac_hicolor_off(): HiColor off
    in_check(0x3C8, "arming"); for (int i = 0; i < 4; i++) in_check(0x3C6, "arming");
    out(0x3C6, 0x00); in_check(0x3C8, "disarm");
    idx(0x3C4, 0, 1); idx(0x3C4, 0, 3);
    static const uint8_t ts[] = { 0x01, 0x01, 0x00, 0x06 };
    for (int i = 0; i < 4; i++) idx(0x3C4, i + 1, ts[i]);
    idx(0x3D4, 0x11, 0);
    static const uint8_t crtc[] = { 0x5F,0x4F,0x50,0x82,0x54,0x80,0x0B,0x3E,0x00,0x40,0x00,0x00,0x00,0x00,0x00,
                                    0x00,0xEA,0x0C,0xDF,0x28,0x00,0xE7,0x04,0xC3,0xFF };
    for (int i = 0; i < 25; i++) idx(0x3D4, i, crtc[i]);
    idx(0x3D4, 0x33, 0); idx(0x3D4, 0x34, 0); idx(0x3D4, 0x35, 0);
    static const uint8_t atcv[] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,0x01,0xFF,0x01,0x10,0x00,0x05,0x00 };
    et.in(0x3DA); inb(0x3DA);
    for (int i = 0; i < 23; i++) { out(0x3C0, i); out(0x3C0, atcv[i]); }
    out(0x3C0, 0x20);
    static const uint8_t gdcv[] = { 0,0,0,0,0,0,1,0x0F,0xFF };
    for (int i = 0; i < 9; i++) idx(0x3CE, i, gdcv[i]);
    idx(0x3D4, 0x36, 0xD3);
    out(0x3C6, 0xFF);
    out(0x3C8, 0); for (int i = 0; i < 3; i++) out(0x3C9, 0xFF); for (int i = 0; i < 3; i++) out(0x3C9, 0);
    out(0x3C8, 255); for (int i = 0; i < 3; i++) out(0x3C9, 0);
}

static void test_picture() {
    int f0 = fails;
    emutos_mode();
    settle();
    CHECK(top->fb_en, "mono: the picture is not on");
    CHECK(top->fb_format == 0x03 && top->fb_width == 640 && top->fb_height == 480 && top->fb_addr == 0xA00000 && top->fb_stride == 640,
          "mono: format %02x %dx%d at %06x stride %d", top->fb_format, top->fb_width, top->fb_height, top->fb_addr, top->fb_stride);
    uint8_t map[256];
    for (int i = 0; i < 256; i++) { int a = et.attr[i & 15 & et.attr[0x12]]; map[i] = (et.attr[0x10] & 0x80) ? ((et.attr[0x14] & 15) << 4) | (a & 15) : ((et.attr[0x14] & 0x0C) << 4) | (a & 0x3F); }
    check_pal(16, map, "mono");
    // a few pixels as EmuTOS draws them: a byte to the first line, one to the second
    et.wrb(0, 0xA5); cycle(true, true, 0, true, false, 0xA500);
    et.wrb(81, 0x3C); cycle(true, true, 80, false, true, 0x003C);
    for (int i = 0; i < 8; i++) tick();
    while (top->ddr_we_wait) tick();
    for (int k = 0; k < 8; k++) {
        CHECK((peek(0xA00000 + k) & 1) == ((0xA5 >> (7 - k)) & 1), "mono: pixel %d of line 0", k);
        CHECK((peek(0xA00000 + 81 * 8 + k) & 1) == ((0x3C >> (7 - k)) & 1), "mono: pixel %d of byte 81", k);
    }
    // the colour plane enable and the colour select change the palette, not the shadow
    atc(0x12, 0x0F); atc(0x14, 0x02); atc(0x10, 0x81); out(0x3C0, 0x20);
    out(0x3C8, 0); for (int i = 0; i < 768; i++) out(0x3C9, rnd());
    settle();
    for (int i = 0; i < 256; i++) { int a = et.attr[i & 15 & et.attr[0x12]]; map[i] = (et.attr[0x10] & 0x80) ? ((et.attr[0x14] & 15) << 4) | (a & 15) : ((et.attr[0x14] & 0x0C) << 4) | (a & 0x3F); }
    check_pal(16, map, "16 colours, colour select");
    out(0x3C6, 0x3F); settle();
    check_pal(16, map, "16 colours, pixel mask");
    out(0x3C6, 0xFF);
    // blanked: the TT's own picture
    idx(0x3C4, 1, 0x21); settle();
    CHECK(!top->fb_en, "blanked: the picture is still on");
    idx(0x3C4, 1, 0x01); settle();
    CHECK(top->fb_en, "unblanked: the picture is not on");

    // 640x480 256 colours, packed, a start address of its own
    idx(0x3C4, 4, 0x0E); idx(0x3CE, 5, 0x40); idx(0x3D4, 0x13, 0x50);
    atc(0x10, 0x41); out(0x3C0, 0x20);
    idx(0x3D4, 0x0C, 0x01); idx(0x3D4, 0x0D, 0x40); idx(0x3D4, 0x33, 0x01);
    settle();
    CHECK(top->fb_en && top->fb_format == 0x03 && top->fb_width == 640 && top->fb_height == 480 &&
          top->fb_addr == 0xC00000 + (0x10140u << 2) && top->fb_stride == 640,
          "256: format %02x %dx%d at %06x stride %d", top->fb_format, top->fb_width, top->fb_height, top->fb_addr, top->fb_stride);
    for (int i = 0; i < 256; i++) map[i] = i;
    check_pal(256, map, "256 colours");
    // HiColor: the Sierra DAC at 15 and 16 bits; the ET4000's doubled clock (ATC 16 bit 5)
    idx(0x3D4, 0x01, 0x9F); idx(0x3D4, 0x13, 0xA0); atc(0x16, 0x20); out(0x3C0, 0x20);
    for (int i = 0; i < 4; i++) { et.in(0x3C6); inb(0x3C6); }
    out(0x3C6, 0xA0); settle();
    CHECK(top->fb_en && top->fb_format == 0x1C && top->fb_width == 1280 && top->fb_stride == 1280,
          "15 bits: format %02x %dx%d stride %d", top->fb_format, top->fb_width, top->fb_height, top->fb_stride);
    for (int i = 0; i < 4; i++) { et.in(0x3C6); inb(0x3C6); }
    out(0x3C6, 0xC0); settle();
    CHECK(top->fb_en && top->fb_format == 0x14 && top->fb_width == 1280,
          "16 bits: format %02x %dx%d", top->fb_format, top->fb_width, top->fb_height);
    for (int i = 0; i < 4; i++) { et.in(0x3C6); inb(0x3C6); }
    out(0x3C6, 0x41); settle();
    CHECK(!top->fb_en, "24 bits is not shown");
    // the machine's reset: the chip's registers, not its memory
    top->sys_reset = 1; tick(); tick(); top->sys_reset = 0; et.reset(); settle();
    CHECK(!top->fb_en, "after a reset the picture is still on");
    in_check(0x3C3, "video subsystem after a reset");
    out(0x3C4, 2); in_check(0x3C5, "sequencer after a reset");
    CHECK(peek(0xA00000) == et.shadow[0], "the shadow kept over a reset");
    printf("picture: %s\n", fails == f0 ? "pass" : "FAIL");
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    int count = 60, ops = 300;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--n")) count = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--ops")) ops = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seed")) rng ^= strtoull(argv[++i], 0, 0) * 0x2545F4914F6CDD1Dull;
        else if (!strcmp(argv[i], "--vid")) vid_every = atoi(argv[++i]);
    }
    top = new Vtb_et4k_top;
    svSetScope(svGetScopeFromName("TOP.tb_et4k_top.ddr"));
    top->reset = 1;
    for (int i = 0; i < 8; i++) tick();
    top->reset = 0; tick();
    et.reset();
    et.vram.assign(0x100000, 0);
    et.shadow.assign(0x200000, 0);

    test_registers();
    test_status();
    int f0 = fails;
    for (int n = 0; n < count; n++) scenario(n, ops);
    printf("memory: %d setups of %d accesses (write modes 0-3 by read mode: %d/%d %d/%d %d/%d %d/%d), %zu bytes and %zu shadow bytes compared: %s\n",
           count, ops, nsc[0][0], nsc[0][1], nsc[1][0], nsc[1][1], nsc[2][0], nsc[2][1], nsc[3][0], nsc[3][1],
           touched.size(), shtouched.size(), fails == f0 ? "pass" : "FAIL");
    test_picture();
    printf("longest bus cycle: %d clocks (the CPU gives up at 1023)\n", max_cycle);
    CHECK(max_cycle < 1023, "a bus cycle longer than the CPU waits");
    printf("SUMMARY: %d failed checks\n", fails);
    return fails != 0;
}
