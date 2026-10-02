// Verilator testbench for rtl/falcon/falcon_psg.sv
//
// The DUTs are the real falcon_psg (three parameter sets, see tb_psg_top.sv).
// Expected values come from Hatari's own code compiled into the testbench:
//   hatari_ym_golden.c  includes src/sound.c (YM2149 core)
//   hatari_psg_golden.c includes src/psg.c   (register interface)
// Plain ASCII output, one line per failed check, summary at the end.

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <string>
#include "Vtb_psg_top.h"
#include "Vtb_psg_top___024root.h"
#include "verilated.h"

extern "C" {
void golden_init(void);
void golden_reset(void);
void golden_set_mixing(int linear);
void golden_write(int reg, int val);
int golden_tick(void);
int golden_levels(void);
int golden_ymout5(int idx);
int golden_envwave(int shape, int pos);
int golden_vol4to5(int v);
int golden_ymout1c5bit(int v);
int golden_rnd_step(void);
void golden_set_rndrack(unsigned v);
void golden_psg_reset(void);
void golden_psg_select(int v);
int golden_psg_read(void);
void golden_psg_data(int v);
}

static Vtb_psg_top *top;
static uint64_t cyc = 0;
static long n_checks = 0, n_fail = 0;
static std::string cur_test;
static long test_fail_start = 0;

static void check(bool ok, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));
static void check(bool ok, const char *fmt, ...)
{
    n_checks++;
    if (ok) return;
    n_fail++;
    if (n_fail - test_fail_start <= 20) {
        char buf[512];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof buf, fmt, ap);
        va_end(ap);
        printf("  FAIL [%s] cyc=%llu: %s\n", cur_test.c_str(), (unsigned long long)cyc, buf);
    }
}

static int tests_run = 0, tests_failed = 0;
static void begin_test(const char *name)
{
    cur_test = name;
    test_fail_start = n_fail;
    printf("TEST %s\n", name);
}
static void end_test()
{
    tests_run++;
    long f = n_fail - test_fail_start;
    if (f) tests_failed++;
    printf("  %s: %s (%ld failed checks)\n", cur_test.c_str(), f ? "FAIL" : "PASS", f);
}

// ---------------------------------------------------------------------------
// Lockstep tracking of the golden model
// ---------------------------------------------------------------------------
static bool track = false;          // apply golden ticks/writes in step()
static int  track_mode = 0;         // 0: compare u_tab samples, 1: u_lin samples
static std::deque<int> exp_samples;
static long lock_samples = 0;

static inline bool dut_tick() { return top->rootp->tb_psg_top__DOT__u_tab__DOT__tick; }
static inline bool lin_tick() { return top->rootp->tb_psg_top__DOT__u_lin__DOT__tick; }

// Golden side effect of a bus write (MIRROR=1 decode: only address bit 1)
static void golden_bus_write(int addr, bool uds, bool lds, uint16_t din)
{
    if (!uds && !lds) return;
    int byte = uds ? (din >> 8) : (din & 0xff);
    if (addr & 2) golden_psg_data(byte);
    else golden_psg_select(byte);
}

static void step()
{
    top->clk = 0;
    top->eval();
    bool tk = dut_tick();
    check(tk == lin_tick(), "u_tab and u_lin tick strobes differ");
    bool wr = top->bus_stb && top->bus_we;
    bool levels_due = false;
    int exp_lv = 0;
    if (track && tk) {
        exp_samples.push_back(golden_tick());
        exp_lv = golden_levels();
        levels_due = true;
    }
    if (track && wr)
        golden_bus_write(top->bus_addr << 1, top->bus_uds, top->bus_lds, top->bus_din);
    top->clk = 1;
    top->eval();
    cyc++;
    if (levels_due) {
        int a = exp_lv & 31, b = (exp_lv >> 5) & 31, c = (exp_lv >> 10) & 31;
        check(top->t_cha == a && top->t_chb == b && top->t_chc == c,
              "levels A/B/C expected %d/%d/%d (Hatari Tone3Voices) got %d/%d/%d",
              a, b, c, top->t_cha, top->t_chb, top->t_chc);
        check(top->l_cha == a && top->l_chb == b && top->l_chc == c,
              "u_lin levels expected %d/%d/%d got %d/%d/%d",
              a, b, c, top->l_cha, top->l_chb, top->l_chc);
    }
    if (track) {
        bool stb = track_mode ? top->l_stb : top->t_stb;
        int smp = (int16_t)(track_mode ? top->l_smp : top->t_smp);
        if (stb) {
            if (exp_samples.empty()) {
                check(false, "sample strobe without a golden tick");
            } else {
                int e = exp_samples.front();
                exp_samples.pop_front();
                check(smp == e, "sample expected %d (Hatari YM_Buffer_250) got %d", e, smp);
                lock_samples++;
            }
        }
    }
}

static void clocks(int n) { for (int i = 0; i < n; i++) step(); }

// One bus access, returns the read data of instance u_tab (and stores others)
static uint16_t last_l_dout, last_n_dout;
static bool last_n_ack;
static uint16_t bus(int addr, bool we, bool uds, bool lds, uint16_t din = 0)
{
    top->bus_cs = 1;
    top->bus_stb = 1;
    top->bus_we = we;
    top->bus_addr = (addr & 0xff) >> 1;
    top->bus_uds = uds;
    top->bus_lds = lds;
    top->bus_din = din;
    top->clk = 0;
    top->eval();
    check(top->t_ack == 1, "bus_ack expected on the bus_stb clock");
    uint16_t d = top->t_dout;
    last_l_dout = top->l_dout;
    last_n_dout = top->n_dout;
    last_n_ack = top->n_ack;
    step();
    top->bus_cs = 0;
    top->bus_stb = 0;
    top->bus_we = 0;
    top->bus_uds = 0;
    top->bus_lds = 0;
    top->clk = 0;
    top->eval();
    check(top->t_ack == 0, "bus_ack must be exactly one clock");
    return d;
}

static void wsel(int r) { bus(0x00, true, true, false, (uint16_t)(r << 8)); }
static void wdat(int v) { bus(0x02, true, true, false, (uint16_t)(v << 8)); }
static void wreg(int r, int v) { wsel(r); wdat(v); }
static int rreg(int r)
{
    wsel(r);
    return bus(0x00, false, true, false) >> 8;
}

static void do_reset()
{
    top->reset = 1;
    clocks(4);
    top->reset = 0;
    golden_psg_reset();     // psg.c PSG_Reset (all volumes 0)
    // The RTL clears the PWM filter state on reset; Hatari keeps it in
    // function statics.  Run the golden core silently until the filter
    // state is 0 (y0 /= 4 per step), then reset the YM state.
    for (int k = 0; k < 64; k++) golden_tick();
    golden_reset();         // sound.c Ym2149_Reset (mixer 0xff) ...
    golden_write(7, 0);     // ... the YM2149 datasheet: register 7 = 0 at reset
    exp_samples.clear();
    top->clk = 0;
    top->eval();
}

// Wait until the next tick has been processed; returns after that clock
static void wait_tick()
{
    for (int i = 0; i < 1000; i++) {
        top->clk = 0;
        top->eval();
        bool t = dut_tick();
        step();
        if (t) return;
    }
    check(false, "no tick within 1000 clocks");
}

// ---------------------------------------------------------------------------
static uint32_t rng = 12345;
static uint32_t rnd() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; }

// ---------------------------------------------------------------------------
static void test_reset_values()
{
    begin_test("reset_values");
    do_reset();
    top->port_a_in = 0x5a;
    top->port_b_in = 0xa5;
    for (int r = 0; r < 14; r++) {
        golden_psg_select(r);
        int e = golden_psg_read();
        int g = rreg(r);
        check(g == e, "reg %d after reset expected 0x%02x (psg.c) got 0x%02x", r, e, g);
    }
    // ports are inputs after reset (reg 7 = 0): pins are pull-ups
    check(top->t_pa == 0xff && top->t_pb == 0xff && !top->t_pa_oe && !top->t_pb_oe,
          "ports after reset expected out=FF/FF oe=0/0 got %02x/%02x oe=%d/%d",
          top->t_pa, top->t_pb, top->t_pa_oe, top->t_pb_oe);
    // reg 14 latch is 0xff after reset (PSG_Reset); visible once port A is output
    wreg(7, 0x40);
    check(top->t_pa == 0xff, "port A latch after reset expected 0xff (PSG_Reset) got 0x%02x", top->t_pa);
    check(rreg(14) == 0xff, "reg 14 read in output mode expected 0xff");
    end_test();
}

static int g_sel = 0;
static void test_register_interface2()
{
    begin_test("register_interface_vs_psg_c");
    do_reset();
    track = true;
    track_mode = 0;
    int nreads = 0, nwrites = 0;
    for (int i = 0; i < 30000; i++) {
        int op = rnd() % 10;
        int mirror = (rnd() % 4 == 0) ? (int)(rnd() % 64) * 4 : 0;
        int kind = rnd() % 3;   // 0 even byte, 1 odd byte (shadow), 2 word
        bool uds = kind != 1, lds = kind != 0;
        if (op < 3) {
            int r = (rnd() % 8 == 0) ? (int)(rnd() & 0xff) : (int)(rnd() % 16);
            uint16_t din = kind == 1 ? (uint16_t)(((rnd() & 0xff) << 8) | r)
                                     : (uint16_t)((r << 8) | (rnd() & 0xff));
            bus(mirror + 0x00, true, uds, lds, din);
            g_sel = r;
        } else if (op < 6) {
            int v = rnd() & 0xff;
            if (g_sel == 7) v |= 0xc0;   // keep both ports as outputs
            uint16_t din = kind == 1 ? (uint16_t)(((rnd() & 0xff) << 8) | v)
                                     : (uint16_t)((v << 8) | (rnd() & 0xff));
            bus(mirror + 0x02, true, uds, lds, din);
            nwrites++;
        } else {
            if (g_sel == 14 || g_sel == 15) {
                // make sure the port is an output first (as in Hatari)
                int keep = g_sel;
                bus(0x00, true, true, false, 0x0700);
                bus(0x02, true, true, false, (uint16_t)((0xc0 | (rnd() & 0x3f)) << 8));
                bus(0x00, true, true, false, (uint16_t)(keep << 8));
            }
            int e = golden_psg_read();
            uint16_t d = bus(mirror + 0x00, false, uds, lds);
            uint16_t ed = (uint16_t)((e << 8) | 0xff);
            if (uds)
                check(d == ed, "read @%02x uds=%d lds=%d sel=%d expected 0x%04x (psg.c) got 0x%04x",
                      mirror, uds, lds, g_sel, ed, d);
            else   // byte read of $FF8801: 0xff (PSG_ff880x_ReadByte)
                check((d & 0xff) == 0xff, "read @%02x (odd byte) expected 0xff got 0x%02x",
                      mirror + 1, d & 0xff);
            if (uds) {
                uint16_t el = (uint16_t)((e << 8) | 0xff);
                check(last_l_dout == el, "u_lin read expected 0x%04x got 0x%04x", el, last_l_dout);
            }
            uint16_t d2 = bus(mirror + 0x02, false, true, true);
            check(d2 == 0xffff, "read @%02x expected 0xffff got 0x%04x", mirror + 2, d2);
            nreads++;
        }
    }
    track = false;
    printf("  %d writes, %d reads compared with Hatari psg.c\n", nwrites, nreads);
    end_test();
}

static void test_shadow_rules()
{
    begin_test("shadow_and_word_access_rules");
    do_reset();
    // byte write to $FF8801 selects (odd byte access = movep / move.b)
    bus(0x01 & ~1, true, false, true, 0x0003);       // lds only at 8800 word -> $FF8801
    bus(0x02, true, true, false, 0x0700);              // data 0x07 -> reg 3 (masked 0x07)
    check(rreg(3) == 0x07, "select through $FF8801 byte write failed");
    // byte write to $FF8803 writes data
    wsel(2);
    bus(0x02, true, false, true, 0x00ab);
    check(rreg(2) == 0xab, "data write through $FF8803 byte write failed");
    // word write to $FF8800: even byte selects, odd byte ($FF8801) ignored
    bus(0x00, true, true, true, 0x0405);
    bus(0x02, true, true, true, 0x99ee);               // word to $FF8802: 0x99 -> reg 4
    check(rreg(4) == 0x99, "word write: reg 4 expected 0x99 got 0x%02x", rreg(4));
    check(rreg(5) == 0x00, "word write: odd byte must not select/write reg 5");
    // read data latch: after a data write without a new select the unmasked
    // value is returned (psg.c 2011/10/30, Murders In Venice)
    wsel(3);
    wdat(0x10);
    uint16_t d = bus(0x00, false, true, false);
    check((d >> 8) == 0x10, "unmasked read-back of reg 3 expected 0x10 got 0x%02x", d >> 8);
    wsel(3);
    d = bus(0x00, false, true, false);
    check((d >> 8) == 0x00, "masked read of reg 3 after reselect expected 0x00 got 0x%02x", d >> 8);
    // invalid selection: reads 0xff, writes ignored
    wsel(0x10);
    wdat(0x55);
    check((bus(0x00, false, true, false) >> 8) == 0xff, "select 0x10 must read 0xff");
    for (int r = 0; r < 16; r++) {
        if (r == 14 || r == 15) continue;
        int v = rreg(r);
        check(v != 0x55 || r == 2, "write with select 0x10 reached reg %d", r);
    }
    // mirrors (MIRROR=1): $FF88F8/$FF88FA act as $FF8800/$FF8802
    bus(0xf8, true, true, false, 0x0100);
    bus(0xfa, true, true, false, 0x0c00);
    check((bus(0x40, false, true, false) >> 8) == 0x0c, "mirror access failed");
    // MIRROR=0 instance: $FF8804-$FF88FF void
    do_reset();
    bus(0x00, true, true, false, 0x0800);
    bus(0x02, true, true, false, 0x0500);   // reg 8 = 5 in all instances
    bus(0x04, true, true, false, 0x0900);   // u_nom: ignored
    bus(0x06, true, true, false, 0x0700);   // u_nom: ignored
    bus(0x04, false, true, true);
    check(last_n_dout == 0xffff && last_n_ack, "MIRROR=0: $FF8804 read expected 0xffff with ack, got 0x%04x ack=%d",
          last_n_dout, last_n_ack);
    bus(0x00, false, true, false);
    check((last_n_dout >> 8) == 0x05, "MIRROR=0: selection must still be reg 8 (0x05), got 0x%02x", last_n_dout >> 8);
    end_test();
}

static void test_ports()
{
    begin_test("io_ports");
    do_reset();
    top->port_a_in = 0x3c;
    top->port_b_in = 0xc3;
    check(rreg(14) == 0x3c, "reg 14 read in input mode expected port_a_in 0x3c");
    check(rreg(15) == 0xc3, "reg 15 read in input mode expected port_b_in 0xc3");
    wreg(7, 0x40);       // port A output, port B input
    for (int v = 0; v < 256; v += 17) {
        wreg(14, v);
        check(top->t_pa == v && top->t_pa_oe, "port A out expected 0x%02x got 0x%02x", v, top->t_pa);
        check(rreg(14) == v, "reg 14 read in output mode expected 0x%02x", v);
    }
    // Falcon use: drive A select (bit 1 low), side 0 (bit 0 high), strobe high
    wreg(14, 0xfd);
    check((top->t_pa & 7) == 5, "port A: drive A selected side 0 -> bits 2..0 = 101");
    check(top->t_pb == 0xff && !top->t_pb_oe, "port B input: out 0xff oe 0");
    wreg(15, 0x81);
    check(top->t_pb == 0xff, "port B latch must not drive while input");
    wreg(7, 0xc0);
    check(top->t_pb == 0x81 && top->t_pb_oe, "port B output expected 0x81 got 0x%02x", top->t_pb);
    wreg(7, 0x80);
    check(top->t_pa == 0xff && !top->t_pa_oe, "port A back to input: out 0xff");
    check(top->t_pb == 0x81, "port B still output");
    end_test();
}

static void test_tone_frequency()
{
    begin_test("tone_frequency");
    const int periods[] = {0, 1, 2, 3, 10, 100, 284, 1000, 4095};
    for (int P : periods) {
        do_reset();
        wreg(7, 0x3e);       // tone A only
        wreg(8, 15);
        wreg(0, P & 0xff);
        wreg(1, P >> 8);
        // measure the distance in clocks between rising edges of ch_a
        int Pe = P ? P : 1;
        long expect = 2L * Pe * 128;   // 2 phases of Pe ticks at 32 MHz / 128
        long last = -1, n = 0, bad = 0;
        int prev = top->t_cha;
        long limit = expect * 6 + 2000;
        for (long c = 0; c < limit; c++) {
            step();
            int cur = top->t_cha;
            if (prev == 0 && cur == 31) {
                if (last >= 0) {
                    n++;
                    if ((long)cyc - last != expect) bad++;
                }
                last = (long)cyc;
            }
            prev = cur;
        }
        check(n >= 4 && bad == 0, "period %d: expected rising edges every %ld clocks (f = 2MHz/(16*%d) = %.2f Hz), %ld edges, %ld wrong",
              P, expect, Pe, 2000000.0 / (16.0 * Pe), n, bad);
        printf("  period %4d: %ld periods of %ld clocks = %.2f Hz\n", P, n, expect, 32e6 / expect);
    }
    // frequency by counting edges in a 0.1 s window for A4 (period 284)
    do_reset();
    wreg(7, 0x3e);
    wreg(8, 15);
    wreg(0, 284 & 0xff);
    wreg(1, 284 >> 8);
    long edges = 0;
    int prev = top->t_cha;
    for (long c = 0; c < 3200000; c++) {
        step();
        if (prev == 0 && top->t_cha == 31) edges++;
        prev = top->t_cha;
    }
    // 440.14 Hz -> 44 rising edges in 0.1 s
    check(edges == 44, "A4 (period 284): expected 44 rising edges in 0.1 s, got %ld", edges);
    printf("  period 284: %ld rising edges in 0.1 s (440.14 Hz)\n", edges);
    end_test();
}

static void test_noise_lfsr()
{
    begin_test("noise_lfsr_vs_hatari");
    for (int per : {0, 1, 3}) {
        do_reset();
        wreg(7, 0x37);       // noise A only
        wreg(8, 15);
        wreg(6, per);
        golden_set_rndrack(1);
        // Hatari: noise counter +1 every 2nd tick, step when count >= period
        int ncount = 0, div2 = 0, nval = 0, bad = 0, steps = 0, ticks = 0;
        while (steps < 600) {
            wait_tick();
            ticks++;
            div2 ^= 1;
            if (div2 == 0) ncount++;
            if (ncount >= per) {
                ncount = 0;
                nval = golden_rnd_step() ? 1 : 0;
                steps++;
            }
            int exp = nval ? 31 : 0;
            if (top->t_cha != exp) bad++;
        }
        check(bad == 0, "noise period %d: %d of %d ticks differ from Hatari YM2149_RndCompute", per, bad, ticks);
        printf("  noise period %d: %d LFSR steps over %d ticks compared\n", per, steps, ticks);
    }
    end_test();
}

static void test_envelopes()
{
    begin_test("envelope_shapes_vs_hatari");
    for (int per : {1, 3}) {
        for (int shape = 0; shape < 16; shape++) {
            do_reset();
            wreg(7, 0x3f);       // tone/noise off: output = volume
            wreg(8, 0x10);       // voice A uses the envelope
            wreg(11, per);
            wreg(12, 0);
            wreg(13, shape);     // restarts the envelope at pos 0
            int count = 0, pos = 0, bad = 0;
            for (int k = 0; k < 200 * per; k++) {
                wait_tick();
                count++;
                if (count >= per) {
                    count = 0;
                    pos++;
                    if (pos >= 96) pos -= 64;
                }
                int e = golden_envwave(shape, pos);
                if (top->t_cha != e) {
                    if (bad < 3)
                        printf("    shape %d tick %d pos %d expected %d got %d\n", shape, k, pos, e, top->t_cha);
                    bad++;
                }
            }
            check(bad == 0, "env shape %d period %d: %d mismatches vs Hatari YmEnvWaves", shape, per, bad);
        }
    }
    printf("  16 shapes x 2 periods, 200 envelope steps each (block 0 + 2 loops of blocks 1/2)\n");
    end_test();
}

static void test_volume_tables()
{
    begin_test("volume_tables");
    // ROM contents of u_tab against Hatari's ymout5 (YM_TABLE_MIXING)
    golden_set_mixing(0);
    int bad = 0;
    for (int i = 0; i < 32768; i++) {
        int g = top->rootp->tb_psg_top__DOT__u_tab__DOT__g_table__DOT__vol_rom[i];
        int e = golden_ymout5(i);
        if (g != e) {
            if (bad < 5) printf("    ymout5[%d] expected %d got %d\n", i, e, g);
            bad++;
        }
    }
    check(bad == 0, "%d of 32768 table entries differ from Hatari ymout5", bad);
    printf("  32768 table entries compared, e.g. [A=31]=%d [A=B=C=15]=%d [all 31]=%d\n",
           golden_ymout5(31), golden_ymout5(15 << 10 | 15 << 5 | 15), golden_ymout5(0x7fff));
    // fixed 4 bit volumes -> 5 bit (YmVolume4to5)
    do_reset();
    wreg(7, 0x3f);
    for (int v = 0; v < 16; v++) {
        wreg(8, v);
        wreg(9, 15 - v);
        wreg(10, v);
        wait_tick();
        int e = golden_vol4to5(v), e2 = golden_vol4to5(15 - v);
        check(top->t_cha == e && top->t_chb == e2 && top->t_chc == e,
              "fixed volume %d expected level %d (YmVolume4to5) got %d", v, e, top->t_cha);
    }
    // datapath: steady fixed volumes give exactly ymout5[] (filter settles to x0)
    for (int t = 0; t < 200; t++) {
        int a = rnd() & 15, b = rnd() & 15, c = rnd() & 15;
        wreg(8, a); wreg(9, b); wreg(10, c);
        for (int k = 0; k < 40; k++) wait_tick();
        clocks(8);
        int idx = golden_vol4to5(c) << 10 | golden_vol4to5(b) << 5 | golden_vol4to5(a);
        check((int16_t)top->t_smp == golden_ymout5(idx), "steady volumes %d/%d/%d expected sample %d got %d",
              a, b, c, golden_ymout5(idx), (int16_t)top->t_smp);
    }
    // linear table (u_lin) through the datapath for every 5-bit level triple
    // reachable with fixed volumes
    golden_set_mixing(1);
    for (int a = 0; a < 16; a++)
        for (int b = 0; b < 16; b += 3)
            for (int c = 0; c < 16; c += 5) {
                wreg(8, a); wreg(9, b); wreg(10, c);
                for (int k = 0; k < 40; k++) wait_tick();
                clocks(8);
                int idx = golden_vol4to5(c) << 10 | golden_vol4to5(b) << 5 | golden_vol4to5(a);
                check((int16_t)top->l_smp == golden_ymout5(idx), "linear %d/%d/%d expected %d got %d",
                      a, b, c, golden_ymout5(idx), (int16_t)top->l_smp);
            }
    golden_set_mixing(0);
    end_test();
}

static void random_reg_value(int &r, int &v)
{
    r = rnd() % 14;
    switch (r) {
    case 0: case 2: case 4: v = (rnd() % 4 == 0) ? (rnd() & 0xff) : (rnd() % 40); break;
    case 1: case 3: case 5: v = (rnd() % 6 == 0) ? (rnd() & 0xff) : 0; break;
    case 6: v = rnd() & 0xff; break;
    case 7: v = rnd() & 0xff; break;
    case 8: case 9: case 10: v = rnd() & 0xff; break;
    case 11: v = (rnd() % 3 == 0) ? (rnd() & 0xff) : (rnd() % 8); break;
    case 12: v = (rnd() % 8 == 0) ? (rnd() & 0x3) : 0; break;
    default: v = rnd() & 0xff; break;
    }
}

static void lockstep(int mode, long nticks)
{
    track_mode = mode;
    do_reset();
    track = true;
    lock_samples = 0;
    long ticks = 0;
    int forced = 0;
    while (ticks < nticks) {
        int r, v;
        random_reg_value(r, v);
        wsel(r);
        int gap = rnd() % 3000;
        for (int i = 0; i < gap; i++) {
            top->clk = 0;
            top->eval();
            if (dut_tick()) ticks++;
            // sometimes issue the data write exactly on a tick clock
            if (dut_tick() && rnd() % 4 == 0) { forced++; break; }
            step();
        }
        wdat(v);
        // a select of an invalid register now and then
        if (rnd() % 50 == 0) { wsel(16 + (rnd() % 240)); wdat(rnd() & 0xff); }
    }
    // silence so the PWM filter state of the golden model decays to 0
    wreg(8, 0); wreg(9, 0); wreg(10, 0);
    for (int k = 0; k < 64; k++) wait_tick();
    clocks(16);
    track = false;
    check(exp_samples.empty(), "%zu golden samples without RTL strobe", exp_samples.size());
    printf("  %ld samples compared (%d writes on a tick clock)\n", lock_samples, forced);
}

static void test_lockstep_table()
{
    begin_test("lockstep_vs_hatari_table_mixing");
    golden_set_mixing(0);
    lockstep(0, 400000);
    end_test();
}

static void test_lockstep_linear()
{
    begin_test("lockstep_vs_hatari_linear_mixing");
    golden_set_mixing(1);
    lockstep(1, 200000);
    golden_set_mixing(0);
    end_test();
}

static void test_sample_strobe()
{
    begin_test("sample_strobe_rate");
    do_reset();
    // count strobes in a 10 ms window that starts at the first strobe
    long n = 0, first = -1, gap_bad = 0, last = -1;
    for (long c = 0; c < 330000; c++) {
        step();
        if (top->t_stb) {
            if (first < 0) first = (long)cyc;
            if ((long)cyc < first + 320000) n++;
            if (last >= 0 && (long)cyc - last != 128) gap_bad++;
            last = (long)cyc;
        }
    }
    check(n == 2500, "snd_stb expected 2500 times in 10 ms (250 kHz) got %ld", n);
    check(gap_bad == 0, "snd_stb expected every 128 clocks at 32 MHz, %ld gaps differ", gap_bad);
    end_test();
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    top = new Vtb_psg_top;
    golden_init();
    top->clk = 0;
    top->reset = 1;
    top->bus_cs = top->bus_stb = top->bus_we = 0;
    top->port_a_in = 0xff;
    top->port_b_in = 0xff;
    top->eval();

    test_reset_values();
    test_register_interface2();
    test_shadow_rules();
    test_ports();
    test_sample_strobe();
    test_tone_frequency();
    test_noise_lfsr();
    test_envelopes();
    test_volume_tables();
    test_lockstep_table();
    test_lockstep_linear();

    printf("\nSUMMARY falcon_psg: %d tests, %d failed, %ld checks, %ld failed checks\n",
           tests_run, tests_failed, n_checks, n_fail);
    printf("%s\n", tests_failed ? "FAIL" : "PASS");
    delete top;
    return tests_failed ? 1 : 0;
}
