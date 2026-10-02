// tb_mfp.cpp - Verilator testbench for rtl/falcon/falcon_mfp.sv (MC68901).
//
// Every access goes through the real register bus of the real module.
// Expected values come from Hatari src/mfp.c (formulas quoted next to each
// check: MFP_REG_TO_CYCLES = data * div, MFP_CYCLE_TO_REG = ceil(cyc / div),
// interrupt priority / IACK rules) and from the MC68901 data sheet.
// The 2.4576 MHz timer clock reference is the module's own clock enable,
// counted here (its rate is checked against 2457600 / 32000000 first).
//
// Plain ASCII only.

#include <verilated.h>
#include "Vfalcon_mfp.h"
#include "Vfalcon_mfp___024root.h"
#include <cstdio>
#include <cstdint>
#include <cstdarg>
#include <cstdlib>
#include <vector>
#include <string>

static Vfalcon_mfp *m;
static uint64_t cyc = 0;          // posedges done
static uint64_t mfp_ticks = 0;    // MFP clock enables consumed by posedges
static int fails = 0, checks = 0;
static int test_fails = 0;

static const double CLK_HZ = 32000000.0;
static const double MFP_HZ = 2457600.0;
static const double CLK_PER_TICK = CLK_HZ / MFP_HZ;   // 13.0208333

// stimulus / monitors
static uint8_t gpin = 0x7F;       // Falcon idle GPIP lines
static int tai_v = 0, tbi_v = 0;
static int si_v = 1;
static bool loop_ext = false;     // si = so
static uint64_t gated_ticks = 0;  // ticks seen while tai == 1 (PWM test)

struct Ev { uint64_t cyc, ticks; int level; };
static std::vector<Ev> tog[4];    // tao, tbo, tco, tdo toggles
static int prev_t[4] = {0, 0, 0, 0};
static std::vector<Ev> so_ev;
static int prev_so = 1;
static std::vector<std::pair<uint64_t, int>> si_sched; // (cycle, level)
static size_t si_pos = 0;

static void check(bool ok, const char *fmt, ...)
{
    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    checks++;
    if (!ok) {
        fails++;
        test_fails++;
        printf("    FAIL: %s\n", buf);
    } else if (getenv("TB_VERBOSE")) {
        printf("    ok:   %s\n", buf);
    }
}

static void tick()
{
    while (si_pos < si_sched.size() && si_sched[si_pos].first <= cyc) {
        si_v = si_sched[si_pos].second;
        si_pos++;
    }
    m->gpip_in = gpin;
    m->tai = tai_v;
    m->tbi = tbi_v;
    m->si = loop_ext ? m->so : si_v;
    m->eval();
    bool t = m->rootp->falcon_mfp__DOT__mfp_tick;
    m->clk = 1;
    m->eval();
    cyc++;
    if (t) {
        mfp_ticks++;
        if (tai_v) gated_ticks++;
    }
    m->clk = 0;
    m->eval();
    int outs[4] = {m->tao, m->tbo, m->tco, m->tdo};
    for (int i = 0; i < 4; i++)
        if (outs[i] != prev_t[i]) {
            tog[i].push_back({cyc, mfp_ticks, outs[i]});
            prev_t[i] = outs[i];
        }
    if (m->so != prev_so) {
        so_ev.push_back({cyc, mfp_ticks, m->so});
        prev_so = m->so;
    }
}

static void si_idle()
{
    // drop any pending line changes and return the line to mark (high)
    si_sched.clear();
    si_pos = 0;
    si_v = 1;
}

static void run(uint64_t n) { for (uint64_t i = 0; i < n; i++) tick(); }

// ------------------------------------------------------------------
// register bus
enum {
    GPIP = 0, AER, DDR, IERA, IERB, IPRA, IPRB, ISRA, ISRB, IMRA, IMRB, VR,
    TACR, TBCR, TCDCR, TADR, TBDR, TCDR, TDDR, SCR, UCR, RSR, TSR, UDR
};
static const char *rname[32] = {
    "GPIP", "AER", "DDR", "IERA", "IERB", "IPRA", "IPRB", "ISRA", "ISRB",
    "IMRA", "IMRB", "VR", "TACR", "TBCR", "TCDCR", "TADR", "TBDR", "TCDR",
    "TDDR", "SCR", "UCR", "RSR", "TSR", "UDR", "R24", "R25", "R26", "R27",
    "R28", "R29", "R30", "R31"};

static uint64_t stb_cyc, stb_ticks_before, stb_ticks_after;

static uint16_t bus(bool we, int reg, uint8_t val, bool lds = true, bool uds = false)
{
    m->bus_cs = 1;
    m->bus_stb = 1;
    m->bus_we = we;
    m->bus_addr = reg;
    m->bus_lds = lds;
    m->bus_uds = uds;
    m->bus_din = 0x5A00 | val;
    stb_ticks_before = mfp_ticks;
    tick();
    stb_cyc = cyc;
    stb_ticks_after = mfp_ticks;
    m->bus_stb = 0;
    int n = 0;
    while (!m->bus_ack) {
        tick();
        if (++n > 16) { check(false, "bus_ack missing for %s", rname[reg & 31]); break; }
    }
    uint16_t r = m->bus_dout;
    m->bus_cs = 0;
    m->bus_we = 0;
    m->bus_lds = 0;
    m->bus_uds = 0;
    tick();
    if (m->bus_ack) check(false, "bus_ack longer than one clock (%s)", rname[reg & 31]);
    return r;
}

static uint8_t rd(int reg)
{
    uint16_t v = bus(false, reg, 0);
    if ((v >> 8) != 0xFF)
        check(false, "even byte of %s read %02X, expected FF", rname[reg], v >> 8);
    return v & 0xFF;
}
static void wr(int reg, uint8_t v) { bus(true, reg, v); }

static void expect_reg(int reg, uint8_t exp, const char *what)
{
    uint8_t v = rd(reg);
    check(v == exp, "%s: %s = %02X, expected %02X", what, rname[reg], v, exp);
}

// "CPU" side of an interrupt acknowledge
static int last_spurious = 0;
static int irq_at_ack = 0;
static int do_iack()
{
    m->iack = 1;
    tick();
    m->iack = 0;
    int ok = m->iack_ack;
    int vec = m->iack_vector;
    last_spurious = m->iack_spurious;
    irq_at_ack = m->irq;
    check(ok == 1, "iack_ack asserted on the clock after iack");
    tick();
    check(m->iack_ack == 0, "iack_ack lasts exactly one clock");
    return vec;
}

static void do_reset()
{
    m->reset = 1;
    m->bus_cs = 0; m->bus_stb = 0; m->bus_we = 0; m->bus_lds = 0; m->bus_uds = 0;
    m->iack = 0;
    gpin = 0x7F; tai_v = 0; tbi_v = 0; si_v = 1; loop_ext = false;
    si_idle();
    run(4);
    m->reset = 0;
    tick();
    for (int i = 0; i < 4; i++) tog[i].clear();
    so_ev.clear();
}

static void begin_test(const char *name)
{
    test_fails = 0;
    printf("TEST %s\n", name);
}
static std::vector<std::pair<std::string, bool>> results;
static void end_test(const char *name)
{
    printf("  %s: %s\n", name, test_fails ? "FAIL" : "PASS");
    results.push_back({name, test_fails == 0});
}

static const int DIV[8] = {0, 4, 10, 16, 50, 64, 100, 200};

// Hatari MFP_CYCLE_TO_REG
static int hatari_cycle_to_reg(int64_t cyc_remain, int div) { return (int)((cyc_remain + div - 1) / div); }

// ------------------------------------------------------------------
static void t_clock()
{
    begin_test("mfp_clock_2_4576MHz");
    uint64_t t0 = mfp_ticks;
    run(3200000);      // 0.1 s
    uint64_t n = mfp_ticks - t0;
    check(n == 245760, "MFP clock enables in 3200000 clocks = %llu, expected 245760 (2457600 Hz * 0.1 s)",
          (unsigned long long)n);
    end_test("mfp_clock_2_4576MHz");
}

static void t_reset_values()
{
    begin_test("reset_values");
    do_reset();
    const uint8_t exp[24] = {0x7F, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x80, 0};
    for (int r = 0; r < 24; r++)
        expect_reg(r, exp[r], "after reset (Hatari MFP_Reset, TSR BE=1 as RS232_TSR_ReadByte)");
    for (int r = 24; r < 32; r++) {
        uint16_t v = bus(false, r, 0);
        check(v == 0xFFFF, "unused register index %d reads %04X, expected FFFF", r, v);
    }
    check(m->irq == 0, "irq = 0 after reset");
    check(m->so == 1, "so idle high after reset");
    end_test("reset_values");
}

static void t_register_rw()
{
    begin_test("register_read_write");
    do_reset();
    const uint8_t pats[] = {0xA5, 0x5A, 0xFF, 0x00, 0x3C};
    struct { int r; uint8_t mask; } rw[] = {
        {AER, 0xFF}, {DDR, 0xFF}, {IERA, 0xFF}, {IERB, 0xFF}, {IMRA, 0xFF},
        {IMRB, 0xFF}, {VR, 0xF8}, {TACR, 0x0F}, {TBCR, 0x0F}, {TCDCR, 0x77},
        {SCR, 0xFF}, {UCR, 0xFE}};
    for (auto &e : rw)
        for (uint8_t p : pats) {
            wr(e.r, p);
            uint8_t v = rd(e.r);
            check(v == (p & e.mask), "%s write %02X read %02X, expected %02X", rname[e.r], p, v, p & e.mask);
        }
    // timers are stopped now (last pattern for ctrl... make sure)
    wr(TACR, 0); wr(TBCR, 0); wr(TCDCR, 0);
    for (int r = TADR; r <= TDDR; r++)
        for (uint8_t p : pats) {
            wr(r, p);
            uint8_t v = rd(r);
            check(v == p, "%s (timer stopped) write %02X read %02X, expected %02X (Hatari: data also loads MAINCOUNTER)",
                  rname[r], p, v, p);
        }
    // IPR / ISR: bits can only be cleared by software
    wr(IERA, 0); wr(IERB, 0);
    for (int r : {IPRA, IPRB, ISRA, ISRB}) {
        wr(r, 0xFF);
        expect_reg(r, 0x00, "write FF cannot set bits");
    }
    // RSR: only SS and RE writable; TSR: AT B H L TE writable, BE read only
    wr(RSR, 0xFE);
    expect_reg(RSR, 0x02, "RSR write FE (only bits 1..0 writable)");
    wr(RSR, 0x00);
    wr(TSR, 0xEE);
    expect_reg(TSR, 0xAE, "TSR write EE (AT,B,H,L writable, BE=1)");
    wr(TSR, 0x21);
    expect_reg(TSR, 0xA1, "TSR write 21 (AT, TE)");
    wr(TSR, 0x20);         // TE 1 -> 0, idle: END set, auto turnaround sets RE
    run(4);
    expect_reg(TSR, 0xB0, "TSR after TE 1->0 (END set)");
    expect_reg(RSR, 0x01, "RSR after END with AT=1 (auto turnaround sets RE)");
    wr(TSR, 0x00); wr(RSR, 0x00);
    // GPIP with DDR = FF: output latch read back
    wr(DDR, 0xFF);
    for (uint8_t p : pats) {
        wr(GPIP, p);
        uint8_t v = rd(GPIP);
        check(v == p, "GPIP (DDR=FF) write %02X read %02X", p, v);
        check(m->gpip_out == p && m->gpip_oe == 0xFF, "gpip_out=%02X gpip_oe=%02X, expected %02X FF",
              m->gpip_out, m->gpip_oe, p);
    }
    // byte access on the even byte: no write, reads FF
    wr(AER, 0x12);
    bus(true, AER, 0x99, false, true);
    expect_reg(AER, 0x12, "even byte write is ignored");
    uint16_t v = bus(false, AER, 0, false, true);
    check((v >> 8) == 0xFF, "even byte read returns FF (got %02X)", v >> 8);
    // word access: odd byte carries the register
    v = bus(false, AER, 0, true, true);
    check(v == 0xFF12, "word read of AER = %04X, expected FF12", v);
    end_test("register_read_write");
}

static void t_gpip_read()
{
    begin_test("gpip_read_rules");
    do_reset();
    wr(GPIP, 0xA5);
    wr(DDR, 0xF0);
    gpin = 0x3C;
    tick();
    uint8_t exp = (0xA5 & 0xF0) | (0x3C & 0x0F);
    expect_reg(GPIP, exp, "DDR=F0 GPDR=A5 lines=3C (output bits from latch, input bits from lines)");
    check(m->gpip_out == 0xA5 && m->gpip_oe == 0xF0, "gpip_out %02X gpip_oe %02X, expected A5 F0", m->gpip_out, m->gpip_oe);
    gpin = 0xC3;
    tick();
    expect_reg(GPIP, (0xA5 & 0xF0) | (0xC3 & 0x0F), "lines changed to C3");
    wr(DDR, 0x00);
    expect_reg(GPIP, 0xC3, "DDR=00: all input bits");
    wr(DDR, 0xFF);
    expect_reg(GPIP, 0xA5, "DDR=FF: whole output latch (written while DDR was F0)");
    end_test("gpip_read_rules");
}

// timers: index 0..3 = A..D
static const int ctrl_reg[4] = {TACR, TBCR, TCDCR, TCDCR};
static const int data_reg[4] = {TADR, TBDR, TCDR, TDDR};
static const int chan[4] = {13, 8, 5, 4};
static uint8_t ctrl_val(int t, int c) { return t == 2 ? (uint8_t)(c << 4) : (uint8_t)c; }

static void enable_chan(int ch, bool en)
{
    int r = ch >= 8 ? IERA : IERB;
    uint8_t v = rd(r);
    uint8_t b = 1 << (ch & 7);
    wr(r, en ? (v | b) : (v & ~b));
}
static bool pending(int ch)
{
    uint8_t v = rd(ch >= 8 ? IPRA : IPRB);
    return (v >> (ch & 7)) & 1;
}
static void clear_pending(int ch) { wr(ch >= 8 ? IPRA : IPRB, (uint8_t)~(1 << (ch & 7))); }

static void t_timer_delay()
{
    begin_test("timer_delay_mode_all_prescalers");
    do_reset();
    const int nper = 4;
    for (int t = 0; t < 4; t++)
        for (int p = 1; p <= 7; p++) {
            int data = (p >= 4) ? 3 : 7;
            wr(ctrl_reg[t], 0);
            wr(data_reg[t], data);
            enable_chan(chan[t], true);
            clear_pending(chan[t]);
            tog[t].clear();
            wr(ctrl_reg[t], ctrl_val(t, p));
            uint64_t c0 = stb_cyc, k0 = stb_ticks_after;
            int64_t P = (int64_t)data * DIV[p];         // Hatari MFP_REG_TO_CYCLES
            run((uint64_t)((nper * P + 2) * CLK_PER_TICK) + 40);
            check(tog[t].size() == (size_t)nper, "timer %c presc %d data %d: %zu timeouts in %d periods, expected %d",
                  'A' + t, DIV[p], data, tog[t].size(), nper, nper);
            for (size_t k = 0; k < tog[t].size() && k < (size_t)nper; k++) {
                int64_t dt = (int64_t)(tog[t][k].ticks - k0);
                check(dt == (int64_t)(k + 1) * P,
                      "timer %c presc %d data %d: timeout %zu after %lld MFP clocks, expected %lld",
                      'A' + t, DIV[p], data, k + 1, (long long)dt, (long long)((k + 1) * P));
                double dc = (double)(tog[t][k].cyc - c0);
                double ec = (double)((k + 1) * P) * CLK_PER_TICK;
                // the first MFP clock after the write comes 0..1 MFP period later
                // (phase of the 2.4576 MHz clock), so the timeout is up to one
                // MFP period earlier than data*div*13.02 clocks after the write
                check(dc >= ec - CLK_PER_TICK - 1.0 && dc <= ec + 1.0,
                      "timer %c presc %d: timeout %zu after %.0f clocks, expected %.1f (-1 MFP clock)",
                      'A' + t, DIV[p], k + 1, dc, ec);
            }
            check(pending(chan[t]), "timer %c presc %d: channel %d pending after timeout", 'A' + t, DIV[p], chan[t]);
            wr(ctrl_reg[t], 0);
            clear_pending(chan[t]);
            enable_chan(chan[t], false);
        }
    end_test("timer_delay_mode_all_prescalers");
}

static void t_timer_read()
{
    begin_test("timer_data_register_read");
    do_reset();
    for (int t = 0; t < 4; t++) {
        int p = (t & 1) ? 5 : 2;     // /64 or /10
        int data = 10;
        int div = DIV[p];
        int64_t P = (int64_t)data * div;
        wr(ctrl_reg[t], 0);
        wr(data_reg[t], data);
        wr(ctrl_reg[t], ctrl_val(t, p));
        uint64_t k0 = stb_ticks_after;
        int bad = 0;
        for (int i = 0; i < 60; i++) {
            run(37 + (i * 53) % 211);
            uint8_t v = rd(data_reg[t]);
            int64_t e = (int64_t)(stb_ticks_before - k0);
            int64_t remain = P - (e % P);                       // MFP clocks to next timeout
            int exp = hatari_cycle_to_reg(remain, div);         // Hatari MFP_ReadTimer_xx
            if (v != exp) bad++;
            check(v == exp, "timer %c /%d data %d: read %d at %lld MFP clocks, expected %d (ceil(%lld/%d))",
                  'A' + t, div, data, v, (long long)e, exp, (long long)remain, div);
        }
        wr(ctrl_reg[t], 0);
    }
    end_test("timer_data_register_read");
}

static void t_timer_stop_restart()
{
    begin_test("timer_stop_restart_and_data_write");
    do_reset();
    // timer C /4, data 10: stop in the middle, counter is held, restart continues
    wr(TCDCR, 0);
    wr(TCDR, 10);
    wr(TCDCR, 0x10);
    uint64_t k0 = stb_ticks_after;
    run(300);
    wr(TCDCR, 0x00);
    int64_t e = (int64_t)(stb_ticks_before - k0);
    int64_t remain = 40 - (e % 40);
    int exp = remain < 4 ? 10 : hatari_cycle_to_reg(remain, 4);
    uint8_t v = rd(TCDR);
    check(v == exp, "timer C stopped after %lld MFP clocks: counter %d, expected %d", (long long)e, v, exp);
    run(2000);
    v = rd(TCDR);
    check(v == exp, "timer C stays stopped: counter %d, expected %d", v, exp);
    tog[2].clear();
    wr(TCDCR, 0x10);
    uint64_t k1 = stb_ticks_after;
    run((uint64_t)((exp * 4 + 40 + 2) * CLK_PER_TICK) + 30);
    check(tog[2].size() == 2, "timer C restarted: %zu timeouts, expected 2", tog[2].size());
    if (tog[2].size() >= 2) {
        check((int64_t)(tog[2][0].ticks - k1) == exp * 4,
              "restart: first timeout after %lld MFP clocks, expected counter*4 = %d",
              (long long)(tog[2][0].ticks - k1), exp * 4);
        check((int64_t)(tog[2][1].ticks - tog[2][0].ticks) == 40,
              "restart: next period %lld, expected data*4 = 40", (long long)(tog[2][1].ticks - tog[2][0].ticks));
    }
    // stop while the counter is between 1 and 0: data register is used (Hatari 2025/05/07)
    int hit = 0;
    for (int attempt = 0; attempt < 50 && !hit; attempt++) {
        while (rd(TCDR) != 1) {}
        run(16);               // at least one more MFP clock inside the last count
        uint8_t c = rd(TCDR);
        if (c != 1) continue;
        wr(TCDCR, 0x00);
        int64_t ee = (int64_t)(stb_ticks_before - k1);
        int64_t rem = 40 - (ee % 40);
        if (rem < 4 && rem > 0) {
            hit = 1;
            v = rd(TCDR);
            check(v == 10, "stopped with %lld MFP clocks left (< prescale 4): counter %d, expected TCDR 10", (long long)rem, v);
        } else {
            wr(TCDCR, 0x10);
            k1 = stb_ticks_after;
        }
    }
    check(hit, "reached a stop between counter 1 and 0");
    // data write while running: counter unaffected, new data used at reload
    wr(TBCR, 0);
    wr(TBDR, 20);
    tog[1].clear();
    wr(TBCR, 1);
    uint64_t kb = stb_ticks_after;
    run(200);
    wr(TBDR, 5);
    v = rd(TBDR);
    int64_t eb = (int64_t)(stb_ticks_before - kb);
    check(v == hatari_cycle_to_reg(80 - eb, 4), "TBDR write while running keeps the counter: read %d, expected %d",
          v, hatari_cycle_to_reg(80 - eb, 4));
    run((uint64_t)((80 + 3 * 20) * CLK_PER_TICK));
    check(tog[1].size() >= 4, "timer B timeouts %zu", tog[1].size());
    if (tog[1].size() >= 4) {
        check((int64_t)(tog[1][0].ticks - kb) == 80, "first period %lld, expected old data 20*4 = 80",
              (long long)(tog[1][0].ticks - kb));
        for (int k = 1; k < 4; k++)
            check((int64_t)(tog[1][k].ticks - tog[1][k - 1].ticks) == 20,
                  "period %d = %lld, expected new data 5*4 = 20", k + 1,
                  (long long)(tog[1][k].ticks - tog[1][k - 1].ticks));
    }
    wr(TBCR, 0);
    // data 0 means 256
    wr(TACR, 0);
    wr(TADR, 0);
    tog[0].clear();
    wr(TACR, 1);
    uint64_t ka = stb_ticks_after;
    run((uint64_t)(1024 * CLK_PER_TICK) + 40);
    check(tog[0].size() == 1 && (int64_t)(tog[0][0].ticks - ka) == 1024,
          "TADR=0 is 256: timeout after %lld MFP clocks, expected 1024",
          tog[0].size() ? (long long)(tog[0][0].ticks - ka) : -1LL);
    // TACR bit 4 forces TAO low
    if (m->tao == 0) { run((uint64_t)(1024 * CLK_PER_TICK) + 40); }
    check(m->tao == 1, "TAO high before the output reset");
    wr(TACR, 0x11);
    check(m->tao == 0, "TACR bit 4 forces TAO low");
    expect_reg(TACR, 0x01, "TACR reads back without bit 4");
    wr(TACR, 0);
    end_test("timer_stop_restart_and_data_write");
}

static void pulse(int &line, int clocks_hi, int clocks_lo)
{
    line = 1; run(clocks_hi);
    line = 0; run(clocks_lo);
}

static void t_event_count()
{
    begin_test("event_count_TBI_TAI_with_AER");
    do_reset();
    // ---- timer B, TBI = Videl DE, AER bit 3 = 0: count at the end of the line (DE 1 -> 0)
    wr(VR, 0x48);
    enable_chan(8, true);
    wr(TBCR, 0);
    wr(TBDR, 3);
    wr(AER, 0x00);
    wr(TBCR, 8);
    tog[1].clear();
    tbi_v = 1; run(40);
    expect_reg(TBDR, 3, "AER3=0: DE 0->1 does not count");
    tbi_v = 0; run(40);
    expect_reg(TBDR, 2, "AER3=0: DE 1->0 counts (1)");
    pulse(tbi_v, 40, 40);
    expect_reg(TBDR, 1, "AER3=0: second line end");
    check(!pending(8), "no timer B interrupt before the counter passes 1");
    pulse(tbi_v, 40, 40);
    expect_reg(TBDR, 3, "counter 1 -> reload from TBDR (Hatari MFP_TimerB_EventCount)");
    check(pending(8), "timer B channel pending after 3 events");
    check(tog[1].size() == 1, "TBO toggled once (%zu)", tog[1].size());
    clear_pending(8);
    // AER changes do not count (Hatari counts only line changes)
    wr(AER, 0x08); wr(AER, 0x00); wr(AER, 0x08);
    expect_reg(TBDR, 3, "AER3 writes do not count");
    // AER bit 3 = 1: count at the start of the line (DE 0 -> 1)
    tbi_v = 1; run(40);
    expect_reg(TBDR, 2, "AER3=1: DE 0->1 counts");
    tbi_v = 0; run(40);
    expect_reg(TBDR, 2, "AER3=1: DE 1->0 does not count");
    pulse(tbi_v, 40, 40);
    pulse(tbi_v, 40, 40);
    expect_reg(TBDR, 3, "AER3=1: reload after 3 line starts");
    check(pending(8), "timer B pending (AER3=1)");
    // many lines: 200 lines, TBDR=3 -> 66 interrupts worth of timeouts
    tog[1].clear();
    wr(TBDR, 7);
    for (int i = 0; i < 70; i++) pulse(tbi_v, 20, 20);
    check(tog[1].size() == 10, "70 lines with counter 3 then 7: %zu timeouts, expected 10 (3 + 9*7 = 66 <= 70 < 73)", tog[1].size());
    wr(TBCR, 0);
    // ---- timer A, TAI = DMA sound SOUNDINT, AER bit 4
    enable_chan(13, true);
    wr(TACR, 0);
    wr(TADR, 2);
    wr(AER, 0x10);
    wr(TACR, 8);
    tai_v = 0; run(20);
    tai_v = 1; run(20);
    expect_reg(TADR, 1, "timer A AER4=1: TAI 0->1 counts");
    tai_v = 0; run(20);
    expect_reg(TADR, 1, "timer A AER4=1: TAI 1->0 does not count");
    tai_v = 1; run(20);
    expect_reg(TADR, 2, "timer A reload");
    check(pending(13), "timer A channel pending");
    wr(AER, 0x00);
    tai_v = 0; run(20);
    expect_reg(TADR, 1, "timer A AER4=0: TAI 1->0 counts");
    wr(TACR, 0);
    end_test("event_count_TBI_TAI_with_AER");
}

static void t_pulse_width()
{
    begin_test("pulse_width_mode");
    do_reset();
    wr(IERA, 0x20);            // timer A
    wr(IERB, 0x40);            // GPIP4 channel = end of TAI pulse in PWM mode
    wr(AER, 0x10);             // TAI active high
    wr(TACR, 0);
    wr(TADR, 100);
    wr(TACR, 0x09);            // pulse width, /4
    tai_v = 0;
    run(2000);
    expect_reg(TADR, 100, "PWM: TAI inactive, timer does not count");
    gated_ticks = 0;
    tai_v = 1;
    run(1500);
    tai_v = 0;
    tick();
    uint64_t g = gated_ticks;
    uint8_t v = rd(TADR);
    int exp = 100 - (int)(g / 4);
    check(v == exp, "PWM: %llu MFP clocks with TAI active: counter %d, expected 100 - %llu/4 = %d",
          (unsigned long long)g, v, (unsigned long long)g, exp);
    check(pending(6), "PWM: end of the TAI pulse (1 -> 0 with AER4=1) sets the GPIP4 channel");
    check(!pending(13), "PWM: no timer A timeout yet");
    clear_pending(6);
    // a GPIP4 pin edge is not an interrupt source in PWM mode
    gpin = 0x6F; run(10); gpin = 0x7F; run(10); gpin = 0x6F; run(10);
    check(!pending(6), "PWM: GPIP4 pin edge ignored while timer A is in pulse width mode");
    gpin = 0x7F;
    // count to timeout: 100 more ticks*4
    tai_v = 1;
    run((uint64_t)(400 * CLK_PER_TICK) + 30);
    check(pending(13), "PWM: timer A timeout while TAI active");
    tai_v = 0;
    run(5);
    // AER4 = 0: active low, pulse ends on 0 -> 1
    wr(TACR, 0);
    wr(TADR, 50);
    clear_pending(6); clear_pending(13);
    wr(AER, 0x00);
    tai_v = 1; run(10);
    wr(TACR, 0x09);
    run(1000);
    expect_reg(TADR, 50, "PWM AER4=0: TAI high is inactive");
    tai_v = 0; run(500);
    tai_v = 1; run(5);
    check(rd(TADR) < 50, "PWM AER4=0: counted while TAI low");
    check(pending(6), "PWM AER4=0: pulse end (0 -> 1) sets the GPIP4 channel");
    wr(TACR, 0);
    end_test("pulse_width_mode");
}

static void t_interrupts()
{
    begin_test("interrupt_priority_mask_isr_iack");
    do_reset();
    gpin = 0xFF;
    run(2);
    wr(VR, 0x48);                    // vectors $40.., software EOI
    wr(IERA, 0xFF); wr(IERB, 0xFF);
    wr(IMRA, 0xFF); wr(IMRB, 0xFF);
    check(m->irq == 0, "no request with nothing pending");
    // GPIP0, GPIP3 and GPIP7 fall together (AER = 0: 1 -> 0 edges)
    gpin = 0xFF & ~0x89;
    run(3);
    expect_reg(IPRA, 0x80, "GPIP7 pending");
    expect_reg(IPRB, 0x09, "GPIP3 and GPIP0 pending");
    check(m->irq == 1, "irq with pending enabled unmasked channels");
    int v = do_iack();
    check(v == 0x4F, "IACK vector %02X, expected 4F (VR $40 + channel 15 GPIP7)", v);
    check(irq_at_ack == 0, "irq already low on the iack_ack clock (GPIP7 now in service)");
    run(2);
    expect_reg(IPRA, 0x00, "IACK cleared IPRA bit 7");
    expect_reg(ISRA, 0x80, "S=1: IACK set ISRA bit 7");
    check(m->irq == 0, "GPIP7 in service blocks lower channels");
    wr(ISRA, 0x7F);                  // software EOI
    run(2);
    check(m->irq == 1, "after EOI the lower pending channels request again");
    v = do_iack();
    check(v == 0x43, "IACK vector %02X, expected 43 (GPIP3)", v);
    expect_reg(ISRB, 0x08, "ISRB bit 3 in service");
    check(m->irq == 0, "GPIP0 blocked by GPIP3 in service");
    // higher priority channel nests
    gpin &= ~0x20;
    run(3);
    check(m->irq == 1, "GPIP5 (channel 7) is above the in service channel 3");
    v = do_iack();
    check(v == 0x47, "IACK vector %02X, expected 47 (GPIP5)", v);
    expect_reg(ISRB, 0x88, "channels 7 and 3 in service");
    wr(ISRB, 0x7F);
    run(2);
    check(m->irq == 0, "channel 3 still in service blocks channel 0");
    wr(ISRB, 0xF7);
    run(2);
    check(m->irq == 1, "channel 0 requests after the last EOI");
    // mask: pending stays, request goes away
    wr(IMRB, 0xFE);
    run(2);
    check(m->irq == 0, "masked channel does not request");
    expect_reg(IPRB, 0x01, "masked channel still pending");
    wr(IMRB, 0xFF);
    run(2);
    check(m->irq == 1, "unmasked again");
    v = do_iack();
    check(v == 0x40, "IACK vector %02X, expected 40 (GPIP0)", v);
    wr(ISRB, 0x00);
    // writing 0 to IPR clears, writing 1 keeps
    gpin = 0xFF; run(3);
    gpin = 0xF9; run(3);             // GPIP1 and GPIP2 fall
    expect_reg(IPRB, 0x06, "GPIP1/2 pending");
    wr(IPRB, 0xFB);
    expect_reg(IPRB, 0x02, "IPRB write FB clears bit 2 only");
    // IER disable clears the pending bit and blocks new events
    wr(IERB, 0xFD);
    expect_reg(IPRB, 0x00, "IER disable clears IPR (Hatari MFP_EnableB_WriteByte)");
    gpin = 0xFF; run(3); gpin = 0xFD; run(3);
    expect_reg(IPRB, 0x00, "event on a disabled channel is not latched");
    wr(IERB, 0xFF);
    check(m->irq == 0, "no request");
    // spurious IACK
    v = do_iack();
    check(v == 0x18 && last_spurious == 1, "IACK with no request: vector %02X spurious %d, expected 18 1", v, last_spurious);
    // automatic EOI (S = 0)
    wr(VR, 0x50);
    gpin = 0xFF; run(3);
    gpin = 0x7E; run(3);             // GPIP7 and GPIP0
    v = do_iack();
    check(v == 0x5F, "S=0 VR=$50: vector %02X, expected 5F", v);
    expect_reg(ISRA, 0x00, "S=0: no in service bit");
    check(m->irq == 1, "S=0: next channel requests immediately");
    check(irq_at_ack == 1, "S=0: irq stays high on the iack_ack clock (GPIP0 still pending)");
    v = do_iack();
    check(v == 0x50, "S=0: vector %02X, expected 50", v);
    check(irq_at_ack == 0, "S=0: irq low on the iack_ack clock of the last channel");
    check(m->irq == 0, "nothing left");
    // S 1 -> 0 clears ISR
    wr(VR, 0xF8);
    gpin = 0xFF; run(3); gpin = 0xDF; run(3);
    v = do_iack();
    check(v == 0xF7, "VR=$F8: vector %02X, expected F7", v);
    expect_reg(ISRB, 0x80, "in service");
    wr(VR, 0xF0);
    expect_reg(ISRB, 0x00, "VR S 1->0 clears ISRB (Hatari MFP_VectorReg_WriteByte)");
    // all 16 vectors: timers give channels 13,8,5,4; GPIP the others
    wr(VR, 0x40);
    int seen = 0;
    for (int ch = 15; ch >= 0; ch--) {
        if (ch == 13 || ch == 8 || ch == 5 || ch == 4) {
            int t = ch == 13 ? 0 : ch == 8 ? 1 : ch == 5 ? 2 : 3;
            wr(data_reg[t], 1);
            wr(ctrl_reg[t], ctrl_val(t, 1));
            run(80);
            wr(ctrl_reg[t], 0);
        } else if (ch >= 9 && ch <= 12) {
            continue;                // USART channels: tested in the USART test
        } else {
            int line = ch == 15 ? 7 : ch == 14 ? 6 : ch == 7 ? 5 : ch == 6 ? 4 : ch;
            gpin = 0xFF; run(3);
            gpin = 0xFF & ~(1 << line); run(3);
            gpin = 0xFF; run(3);
        }
        v = do_iack();
        check(v == (0x40 | ch), "channel %d: vector %02X, expected %02X", ch, v, 0x40 | ch);
        seen++;
        wr(IPRA, 0); wr(IPRB, 0);
    }
    check(seen == 12, "12 non USART channels exercised");
    end_test("interrupt_priority_mask_isr_iack");
}

static void t_gpip_edges()
{
    begin_test("gpip_edges_aer_ddr");
    do_reset();
    gpin = 0xFF; run(2);
    wr(VR, 0x48);
    wr(IERA, 0xC0); wr(IERB, 0xEF);       // all GPIP channels
    wr(IMRA, 0xC0); wr(IMRB, 0xEF);
    // AER = 0: falling edge only
    gpin = 0xFB; run(3);
    expect_reg(IPRB, 0x04, "AER2=0: GPIP2 1->0 interrupts");
    wr(IPRB, 0);
    gpin = 0xFF; run(3);
    expect_reg(IPRB, 0x00, "AER2=0: GPIP2 0->1 does not");
    // AER = 1: rising edge only
    wr(AER, 0x04);              // line is 1, AER 0->1: line now matches AER -> interrupt (Hatari quirk)
    expect_reg(IPRB, 0x04, "AER2 0->1 with GPIP2=1 interrupts (MFP_GPIP_Update_Interrupt AER change)");
    wr(IPRB, 0);
    gpin = 0xFB; run(3);
    expect_reg(IPRB, 0x00, "AER2=1: 1->0 does not interrupt");
    gpin = 0xFF; run(3);
    expect_reg(IPRB, 0x04, "AER2=1: 0->1 interrupts");
    wr(IPRB, 0);
    // AER change with the line not matching the new AER: no interrupt
    wr(AER, 0x00);              // line 1, AER 1->0: state goes match -> no match
    expect_reg(IPRB, 0x00, "AER2 1->0 with GPIP2=1: no interrupt");
    gpin = 0xFB; run(3);        // falling edge with AER=0 -> interrupt
    wr(IPRB, 0);
    wr(AER, 0x04);              // line 0, AER 0->1: no match -> no interrupt
    expect_reg(IPRB, 0x00, "AER2 0->1 with GPIP2=0: no interrupt");
    wr(AER, 0x00);              // line 0, AER 1->0: now matches -> interrupt
    expect_reg(IPRB, 0x04, "AER2 1->0 with GPIP2=0 interrupts ('M'/'Realtime' bset/bclr AER quirk)");
    wr(IPRB, 0);
    gpin = 0xFF; run(3);
    // the bset #0 / bclr #0 sequence of 'M' and 'Realtime' on a line at 0
    gpin = 0xFE; run(3);
    wr(IPRB, 0);
    wr(AER, 0x01); wr(AER, 0x00);
    expect_reg(IPRB, 0x01, "bset/bclr AER0 with GPIP0=0 raises GPIP0");
    wr(IPRB, 0);
    gpin = 0xFF; run(3);
    // DDR = 1 (output) lines never interrupt
    wr(DDR, 0x04);
    gpin = 0xFB; run(3); gpin = 0xFF; run(3);
    wr(AER, 0x04); wr(AER, 0x00);
    wr(GPIP, 0x00); wr(GPIP, 0x04); wr(GPIP, 0x00);
    expect_reg(IPRB, 0x00, "output line: no interrupt from line, AER or GPIP writes");
    // switching to input with the line different from the latch: no interrupt
    wr(GPIP, 0x00);             // latch 0, line 1
    wr(DDR, 0x00);
    expect_reg(IPRB, 0x00, "DDR 1->0 does not raise an interrupt (Hatari MFP_DataDirection_WriteByte)");
    gpin = 0xFB; run(3);
    expect_reg(IPRB, 0x04, "after DDR 1->0 the next falling edge interrupts");
    wr(IPRB, 0);
    gpin = 0xFF; run(3);
    // every line, both polarities
    for (int l = 0; l < 8; l++) {
        int ch = l < 4 ? l : (l < 6 ? l + 2 : l + 8);
        int r = ch >= 8 ? IPRA : IPRB;
        uint8_t b = 1 << (ch & 7);
        wr(AER, 0x00);
        gpin = 0xFF & ~(1 << l); run(3); gpin = 0xFF; run(3);
        expect_reg(r, b, "AER=0 falling edge sets the line's channel");
        wr(r, 0);
        wr(AER, 1 << l);
        wr(r, 0);               // AER change raised it (line=1 matches): clear
        gpin = 0xFF & ~(1 << l); run(3);
        expect_reg(r, 0, "AER=1: falling edge ignored");
        gpin = 0xFF; run(3);
        expect_reg(r, b, "AER=1 rising edge sets the line's channel");
        wr(r, 0);
    }
    end_test("gpip_edges_aer_ddr");
}

// ------------------------------------------------------------------
// USART
static const double BIT_CLK = 32000000.0 / 9600.0;

static void sched_frame(uint64_t start, const std::vector<int> &bits)
{
    // bits: start bit, data..., stop...; afterwards line high
    for (size_t i = 0; i < bits.size(); i++)
        si_sched.push_back({start + (uint64_t)(i * BIT_CLK), bits[i]});
    si_sched.push_back({start + (uint64_t)(bits.size() * BIT_CLK), 1});
}

static void usart_9600(uint8_t ucr)
{
    wr(TCDCR, 0);
    wr(TDDR, 2);
    wr(TCDCR, 0x01);           // timer D /4, data 2: 2457600/4/2/2/16 = 9600 baud
    wr(UCR, ucr);
    wr(RSR, 0x01);
    wr(TSR, 0x01);
}

static int wait_reg_bit(int reg, uint8_t mask, uint8_t val, int max_reads)
{
    for (int i = 0; i < max_reads; i++) {
        if ((rd(reg) & mask) == val) return 1;
        run(50);
    }
    return 0;
}

static void t_usart_loopback()
{
    begin_test("usart_loopback_9600");
    do_reset();
    loop_ext = true;            // so wired to si
    wr(VR, 0x48);
    wr(IERA, 0x14); wr(IMRA, 0x14);   // rx full, tx empty (as EmuTOS)
    usart_9600(0x88);
    expect_reg(TSR, 0x81, "TSR idle after TSR=1 (Hatari: bit 7 = 1)");
    expect_reg(RSR, 0x01, "RSR idle after RSR=1 (Hatari: bit 7 = 0)");
    const uint8_t bytes[] = {0x55, 0xA3, 0x00, 0xFF, 0x0F};
    for (uint8_t b : bytes) {
        so_ev.clear();
        // the previous frame may still be in its stop bit (the receiver
        // completes in the middle of the stop bit)
        int idle = wait_reg_bit(TSR, 0x80, 0x80, 100);
        check(idle, "TSR BE=1 before writing UDR");
        run((uint64_t)BIT_CLK);
        wr(UDR, b);
        expect_reg(TSR, 0x01, "TSR BE=0 right after the UDR write");
        run(400);   // next falling TC edge (every 16 MFP clocks) moves UDR into the shift register
        expect_reg(TSR, 0x81, "TSR BE=1 after the transfer to the shift register");
        check(m->irq == 1, "tx buffer empty interrupt requested");
        int v = do_iack();
        check(v == 0x4A, "vector %02X, expected 4A (transmit buffer empty)", v);
        wr(ISRA, 0xFB);
        int ok = wait_reg_bit(RSR, 0x80, 0x80, 2000);
        check(ok, "byte %02X received (RSR BF)", b);
        expect_reg(RSR, 0x81, "RSR: BF, no error");
        check(m->irq == 1, "rx buffer full interrupt requested");
        v = do_iack();
        check(v == 0x4C, "vector %02X, expected 4C (receive buffer full)", v);
        expect_reg(UDR, b, "received byte");
        expect_reg(RSR, 0x01, "UDR read clears BF");
        wr(ISRA, 0xEF);
        // bit timing on so: start bit to stop bit = 9 bit times of 256 MFP clocks
        if (!so_ev.empty()) {
            check(so_ev[0].level == 0, "frame starts with a start bit (so 1->0)");
            uint64_t t0 = so_ev[0].ticks;
            // each so edge lies on a bit boundary of 256 MFP clocks (9600 baud)
            for (auto &e : so_ev)
                check((e.ticks - t0) % 256 == 0, "so edge at %llu MFP clocks after the start bit, multiple of 256",
                      (unsigned long long)(e.ticks - t0));
            check(so_ev.back().level == 1, "line returns high after the frame");
            // the stop bit begins 9 bit times after the start bit
            uint64_t last_rise = so_ev.back().ticks - t0;
            int ones_tail = 0;
            for (int i = 7; i >= 0 && ((b >> i) & 1); i--) ones_tail++;
            check(last_rise == 256ull * (9 - ones_tail), "last rising edge at %llu MFP clocks, expected %llu",
                  (unsigned long long)last_rise, 256ull * (9 - ones_tail));
            double us = (double)(256) / MFP_HZ * 1e6;
            check(us > 104.16 && us < 104.17, "bit time %.3f us = 1/9600 s", us);
        }
    }
    // back to back: two bytes written while the first one is being sent
    so_ev.clear();
    wr(IERA, 0); wr(IMRA, 0);
    int ok;
    run((uint64_t)BIT_CLK);       // transmitter idle
    wr(UDR, 0x31);
    ok = wait_reg_bit(TSR, 0x80, 0x80, 10);
    check(ok, "first byte moved to the shift register");
    wr(UDR, 0x32);
    expect_reg(TSR, 0x01, "second byte waits in UDR (BE=0) while the first is sent");
    ok = wait_reg_bit(RSR, 0x80, 0x80, 2000);
    check(ok, "first of two bytes");
    expect_reg(UDR, 0x31, "first byte");
    ok = wait_reg_bit(RSR, 0x80, 0x80, 2000);
    check(ok, "second of two bytes");
    expect_reg(UDR, 0x32, "second byte");
    if (so_ev.size() >= 3) {
        // second start bit begins exactly 10 bit times after the first
        uint64_t t0 = so_ev[0].ticks;
        bool found = false;
        for (auto &e : so_ev)
            if (e.level == 0 && e.ticks - t0 == 2560) found = true;
        check(found, "second start bit 10 bit times (2560 MFP clocks) after the first");
    }
    // internal loopback (TSR H=L=1), external si held high
    loop_ext = false;
    si_v = 1;
    wr(TSR, 0x07);
    wr(UDR, 0xC6);
    ok = wait_reg_bit(RSR, 0x80, 0x80, 2000);
    check(ok, "internal loopback receive");
    expect_reg(UDR, 0xC6, "internal loopback byte");
    wr(TSR, 0x06);              // TE off, loopback config kept
    run((uint64_t)BIT_CLK);     // the transmitter finishes its stop bit first
    check(m->so == 1, "loopback, transmitter disabled: so high");
    // END (transmitter disabled and idle) raises the transmit error channel 9
    wr(TSR, 0x01);
    wr(IPRA, 0);
    wr(IERA, 0x02); wr(IMRA, 0x02);
    wr(UDR, 0x5A);
    wait_reg_bit(TSR, 0x80, 0x80, 10);
    wr(TSR, 0x00);              // disable while the character is being sent
    expect_reg(TSR, 0x80, "END not set while the last character is still sent");
    check(m->irq == 0, "no transmit error interrupt before the character ends");
    run((uint64_t)(11 * BIT_CLK));
    expect_reg(TSR, 0x90, "END set when the character has been sent");
    check(m->irq == 1, "transmit error (END) interrupt requested");
    int vend = do_iack();
    check(vend == 0x49, "vector %02X, expected 49 (transmit error)", vend);
    wr(ISRA, 0);
    wr(IERA, 0); wr(IMRA, 0);
    // H/L output states with the transmitter disabled
    wr(TSR, 0x02); run(2);
    check(m->so == 0, "TSR L: so low");
    wr(TSR, 0x04); run(2);
    check(m->so == 1, "TSR H: so high");
    wr(TSR, 0x00);
    end_test("usart_loopback_9600");
}

static std::vector<int> frame_bits(uint8_t data, int nbits, int par /* -1 none */, int stop)
{
    std::vector<int> b;
    b.push_back(0);
    for (int i = 0; i < nbits; i++) b.push_back((data >> i) & 1);
    if (par >= 0) b.push_back(par);
    b.push_back(stop);
    return b;
}

static void t_usart_errors()
{
    begin_test("usart_receive_parity_framing_overrun_break");
    do_reset();
    wr(VR, 0x48);
    // 7 data bits, even parity, 1 stop: UCR = 1 01 01 1 1 0 = $AE
    usart_9600(0xAE);
    wr(IERA, 0x10); wr(IMRA, 0x10);   // rx full only (error channel disabled)
    loop_ext = true;
    wr(UDR, 0x41);
    int ok = wait_reg_bit(RSR, 0x80, 0x80, 2000);
    check(ok, "7E1 loopback");
    expect_reg(RSR, 0x81, "7E1: no parity error");
    expect_reg(UDR, 0x41, "7E1 byte");
    loop_ext = false;
    wr(IPRA, 0);
    // wrong parity bit from the line: $41 has two ones -> even parity bit 0, send 1
    uint64_t t = cyc + 2000;
    si_idle();
    sched_frame(t, frame_bits(0x41, 7, 1, 1));
    ok = wait_reg_bit(RSR, 0x80, 0x80, 3000);
    check(ok, "frame with parity error received");
    expect_reg(RSR, 0xA1, "RSR BF + PE");
    expect_reg(IPRA, 0x10, "error channel disabled: reported on buffer full (channel 12)");
    expect_reg(UDR, 0x41, "data with parity error");
    wr(IPRA, 0);
    // with the error channel enabled the error goes to channel 11
    wr(IERA, 0x18); wr(IMRA, 0x18);
    si_idle();
    sched_frame(cyc + 1000, frame_bits(0x41, 7, 1, 1));
    ok = wait_reg_bit(RSR, 0x80, 0x80, 3000);
    expect_reg(IPRA, 0x08, "error channel enabled: receive error (channel 11)");
    int v = do_iack();
    check(v == 0x4B, "vector %02X, expected 4B (receive error)", v);
    wr(ISRA, 0);
    rd(UDR);
    wr(IPRA, 0);
    // framing error: stop bit 0
    si_idle();
    sched_frame(cyc + 1000, frame_bits(0x12, 7, 0, 0));
    ok = wait_reg_bit(RSR, 0x80, 0x80, 3000);
    check(ok, "frame with stop bit 0 received");
    expect_reg(RSR, 0x91, "RSR BF + FE");
    expect_reg(UDR, 0x12, "framing error data");
    // overrun: two frames, no UDR read in between
    wr(IPRA, 0);
    si_idle();
    sched_frame(cyc + 1000, frame_bits(0x21, 7, 0, 1));
    sched_frame(cyc + 1000 + (uint64_t)(12 * BIT_CLK), frame_bits(0x22, 7, 0, 1));
    run((uint64_t)(26 * BIT_CLK));
    expect_reg(RSR, 0xC1, "RSR BF + OE after an overrun");
    expect_reg(RSR, 0x81, "reading RSR clears OE");
    expect_reg(UDR, 0x21, "the first byte is kept");
    // break: line low for two character times
    wr(IPRA, 0);
    si_idle();
    si_sched.push_back({cyc + 500, 0});
    si_sched.push_back({cyc + 500 + (uint64_t)(20 * BIT_CLK), 1});
    run((uint64_t)(12 * BIT_CLK));
    expect_reg(RSR, 0x09, "break detected (B), BF not set");
    run((uint64_t)(10 * BIT_CLK));
    expect_reg(IPRA, 0x08, "break raised a receive error");
    rd(RSR);
    expect_reg(RSR, 0x01, "B cleared by reading RSR after the line returned high");
    // receiver disable clears the status
    wr(RSR, 0x00);
    expect_reg(RSR, 0x00, "receiver disabled");
    end_test("usart_receive_parity_framing_overrun_break");
}

// ------------------------------------------------------------------
// TOS / EmuTOS initialisation, taken from the EmuTOS 512 KB ROM in
// tb/system/etos512us.hex (disassembled): mfp_init $E0E686, xbtimer
// $E0E768, jdisint $E0E69E, jenabint $E0E6F2, timer C setup $E0E872,
// delay calibration with timer D $E1467A, rsconf $E102CE and serial
// interrupt setup $E10DE4.
static int64_t tos_irqs = 0, tos_bad = 0;
static int cnt_c = 0, cnt_d = 0;

static void tos_service()
{
    // CPU at IPL 3: takes level 6 requests
    if (!m->irq) return;
    int v = do_iack();
    tos_irqs++;
    if (v == 0x45) { cnt_c++; wr(ISRB, 0xDF); }          // timer C handler $E00914
    else if (v == 0x44) { cnt_d++; wr(ISRB, 0xEF); }     // calibration handler $E146EC
    else {
        tos_bad++;
        check(false, "unexpected interrupt vector %02X during the TOS init", v);
    }
}
static void tos_run(uint64_t n)
{
    for (uint64_t i = 0; i < n; i++) {
        tick();
        // Videl DE: 640 clocks on, 400 off (lines keep running, timer B unused)
        tbi_v = (cyc % 1040) < 640;
        if (m->irq) tos_service();
    }
}
static void jdisint(int n)
{
    int lo = n < 8;
    uint8_t b = 1 << (n & 7);
    wr(lo ? IMRB : IMRA, rd(lo ? IMRB : IMRA) & ~b);
    wr(lo ? IERB : IERA, rd(lo ? IERB : IERA) & ~b);
    wr(lo ? IPRB : IPRA, ~b);
    wr(lo ? ISRB : ISRA, ~b);
}
static void jenabint(int n)
{
    int lo = n < 8;
    uint8_t b = 1 << (n & 7);
    wr(lo ? IERB : IERA, rd(lo ? IERB : IERA) | b);
    wr(lo ? IMRB : IMRA, rd(lo ? IMRB : IMRA) | b);
}

static void t_tos_init()
{
    begin_test("tos_emutos_init_sequence");
    do_reset();
    gpin = 0x7F;               // Falcon idle lines: I7 (sound) 0, others 1
    // mfp_init: clear $FFFA01..$FFFA2D, VR = $48
    for (int r = GPIP; r <= TSR; r++) wr(r, 0);
    wr(VR, 0x48);
    // xbtimer(2, $50, $C0, timer C handler)
    jdisint(5);
    wr(TCDCR, rd(TCDCR) & 0x0F);
    wr(TCDR, 0xC0);
    wr(TCDCR, (0x50 & 0xF0) | rd(TCDCR));
    jenabint(5);
    // mfpint(6, ACIA handler)
    jdisint(6);
    jenabint(6);
    // delay calibration with timer D
    wr(TCDCR, rd(TCDCR) & 0xF0);
    wr(TDDR, 0x00);
    wr(IERB, rd(IERB) | 0x10);
    wr(IMRB, rd(IMRB) | 0x10);
    wr(TCDCR, rd(TCDCR) | 0x02);
    tos_run(320000);           // 10 ms busy loop
    wr(IERB, rd(IERB) & ~0x10);
    wr(IMRB, rd(IMRB) & ~0x10);
    wr(TCDCR, rd(TCDCR) & 0xF0);
    // timer D /10 data 256: 2560 MFP clocks = 1.0417 ms -> 9 or 10 in 10 ms
    check(cnt_d >= 9 && cnt_d <= 10, "calibration: %d timer D interrupts in 10 ms, expected 9..10 (period 2560 MFP clocks)", cnt_d);
    // rsconf(9600, 0, $88, 1, 1, 0)
    rd(UCR); rd(RSR); rd(TSR);
    wr(TCDCR, rd(TCDCR) & 0xF0);
    wr(TDDR, 0x02);
    wr(TCDCR, (0x01 & 0x0F) | rd(TCDCR));
    wr(UCR, 0x88);
    wr(RSR, 0x01);
    wr(TSR, 0x01);
    wr(SCR, 0x00);
    // mfpint(12, rx), mfpint(10, tx)
    jdisint(12); jenabint(12);
    jdisint(10); jenabint(10);
    expect_reg(VR, 0x48, "VR");
    expect_reg(IERA, 0x14, "IERA (rx full, tx empty)");
    expect_reg(IMRA, 0x14, "IMRA");
    expect_reg(IERB, 0x60, "IERB (GPIP4 ACIA, timer C)");
    expect_reg(IMRB, 0x60, "IMRB");
    expect_reg(TCDCR, 0x51, "TCDCR (C /64, D /4)");
    expect_reg(UCR, 0x88, "UCR");
    expect_reg(RSR, 0x01, "RSR idle (BF = 0)");
    expect_reg(TSR, 0x81, "TSR idle (BE = 1)");
    expect_reg(GPIP, 0x7F, "GPIP idle Falcon lines");
    expect_reg(IPRA, 0x00, "no USART interrupt pending on an idle line");
    // 200 Hz system timer for 50 ms
    cnt_c = 0;
    uint64_t c0 = cyc;
    tos_run(1600000);
    double ms = (double)(cyc - c0) / 32000.0;
    check(cnt_c >= 9 && cnt_c <= 11, "timer C: %d interrupts in %.1f ms, expected 10 (200 Hz = 192 * 64 MFP clocks)", cnt_c, ms);
    check(tos_bad == 0, "no spurious interrupts (%lld unexpected)", (long long)tos_bad);
    uint8_t ipra = rd(IPRA), iprb = rd(IPRB);
    check(ipra == 0 && (iprb & ~0x20) == 0, "IPRA %02X IPRB %02X: only timer C may be pending", ipra, iprb);
    check(cnt_d >= 9 && cnt_d <= 10, "no timer D interrupt after the calibration (%d)", cnt_d);
    end_test("tos_emutos_init_sequence");
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    m = new Vfalcon_mfp;
    m->clk = 0;
    m->reset = 1;
    m->eval();
    do_reset();

    t_clock();
    t_reset_values();
    t_register_rw();
    t_gpip_read();
    t_timer_delay();
    t_timer_read();
    t_timer_stop_restart();
    t_event_count();
    t_pulse_width();
    t_interrupts();
    t_gpip_edges();
    t_usart_loopback();
    t_usart_errors();
    t_tos_init();

    printf("\nSUMMARY (%d checks)\n", checks);
    int nf = 0;
    for (auto &r : results) {
        printf("  %-48s %s\n", r.first.c_str(), r.second ? "PASS" : "FAIL");
        if (!r.second) nf++;
    }
    printf("%s: %d of %zu tests failed, %d failed checks\n", nf ? "FAIL" : "PASS", nf, results.size(), fails);
    delete m;
    return nf ? 1 : 0;
}
