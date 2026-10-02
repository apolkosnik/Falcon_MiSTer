// Verilator testbench for rtl/falcon/falcon_nvram.sv
//
// DUTs: the real falcon_nvram, two instances (tb_nvram_top.sv): u_real at
// CLK_HZ = 32 MHz and u_fast at CLK_HZ = 131072 for multi-second runs.
// Expected values: Hatari's src/falcon/nvram.c compiled in
// (hatari_nvram_golden.c) with the host clock replaced by a controlled
// time value, plus the MC146818A datasheet where Hatari does not emulate
// the chip (update cycle timing, flags), each such check says so.
// Plain ASCII output, summary at the end.

#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <ctime>
#include <string>
#include <vector>
#include "Vtb_nvram_top.h"
#include "verilated.h"

extern "C" {
void golden_nv_setup(void);
void golden_nv_init(int vga, int lang, int kbd);
void golden_nv_reset(void);
void golden_nv_set_time(long long t);
int golden_nv_byte(int i);
void golden_nv_select(int v);
int golden_nv_index(void);
int golden_nv_read(void);
void golden_nv_write(int v);
}

static Vtb_nvram_top *top;
static uint64_t cyc = 0;
static long n_checks = 0, n_fail = 0, test_fail_start = 0;
static int tests_run = 0, tests_failed = 0;
static std::string cur_test;

static void check(bool ok, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void check(bool ok, const char *fmt, ...)
{
    n_checks++;
    if (ok) return;
    n_fail++;
    if (n_fail - test_fail_start <= 25) {
        char buf[512];
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(buf, sizeof buf, fmt, ap);
        va_end(ap);
        printf("  FAIL [%s] cyc=%llu: %s\n", cur_test.c_str(), (unsigned long long)cyc, buf);
    }
}
static void begin_test(const char *n) { cur_test = n; test_fail_start = n_fail; printf("TEST %s\n", n); }
static void end_test()
{
    tests_run++;
    long f = n_fail - test_fail_start;
    if (f) tests_failed++;
    printf("  %s: %s (%ld failed checks)\n", cur_test.c_str(), f ? "FAIL" : "PASS", f);
}

static bool nvch_r = false, nvch_f = false;
static void step()
{
    top->clk = 0;
    top->eval();
    top->clk = 1;
    top->eval();
    cyc++;
    if (top->r_nv_changed) nvch_r = true;
    if (top->f_nv_changed) nvch_f = true;
}
static void clocks(long n) { for (long i = 0; i < n; i++) step(); }

static uint8_t rd_r, rd_f;   // last read data (odd byte) of both instances
static void bus(int hi, bool we, bool uds, bool lds, uint16_t din, uint16_t *dr = nullptr, uint16_t *df = nullptr)
{
    top->bus_cs = 1; top->bus_stb = 1; top->bus_we = we;
    top->bus_addr = hi; top->bus_uds = uds; top->bus_lds = lds; top->bus_din = din;
    top->clk = 0;
    top->eval();
    check(top->r_ack && top->f_ack, "bus_ack expected on the bus_stb clock");
    if (dr) *dr = top->r_dout;
    if (df) *df = top->f_dout;
    rd_r = top->r_dout & 0xff;
    rd_f = top->f_dout & 0xff;
    step();
    top->bus_cs = 0; top->bus_stb = 0; top->bus_we = 0; top->bus_uds = 0; top->bus_lds = 0;
    top->clk = 0;
    top->eval();
    check(!top->r_ack && !top->f_ack, "bus_ack must be one clock");
}
// $FF8961 / $FF8963 byte accesses (odd byte)
static void nsel(int i) { bus(0, true, false, true, (uint16_t)(0xab00 | (i & 0xff))); }
static void nwr(int v) { bus(1, true, false, true, (uint16_t)(0x5a00 | (v & 0xff))); }
static int nrd() { bus(1, false, false, true, 0); return rd_r; }   // u_real, u_fast in rd_f
static void wr_both(int idx, int v) { nsel(idx); nwr(v); golden_nv_select(idx); golden_nv_write(v); }

static int bcd(int v) { return ((v / 10) << 4) | (v % 10); }

static void set_rtc(long long t)
{
    time_t tt = (time_t)t;
    struct tm tmv;
    gmtime_r(&tt, &tmv);
    uint64_t lo = (uint64_t)bcd(tmv.tm_sec) | (uint64_t)bcd(tmv.tm_min) << 8 |
                  (uint64_t)bcd(tmv.tm_hour) << 16 | (uint64_t)bcd(tmv.tm_mday) << 24 |
                  (uint64_t)bcd(tmv.tm_mon + 1) << 32 | (uint64_t)bcd(tmv.tm_year % 100) << 40 |
                  (uint64_t)tmv.tm_wday << 48 | (uint64_t)0x40 << 56;
    uint32_t tog = top->rtc[2] & 1;
    top->rtc[0] = (uint32_t)lo;
    top->rtc[1] = (uint32_t)(lo >> 32);
    top->rtc[2] = tog ^ 1;
    clocks(3);
}

static void do_reset()
{
    top->reset = 1;
    clocks(3);
    top->reset = 0;
    clocks(1);
    golden_nv_reset();
}

static const char *fmt_time(long long t)
{
    static char b[64];
    time_t tt = (time_t)t;
    struct tm tmv;
    gmtime_r(&tt, &tmv);
    strftime(b, sizeof b, "%Y-%m-%d %H:%M:%S %a", &tmv);
    return b;
}

// Compare the clock bytes of both instances with Hatari for time t.
// is_fast_only: compare only u_fast (u_real has a different time then)
static void compare_time(long long t, bool fast_only, const char *what)
{
    golden_nv_set_time(t);
    static const int idx[] = {0, 2, 4, 6, 7, 8, 9};
    golden_nv_select(11);
    int b = golden_nv_read();
    for (int i : idx) {
        nsel(i);
        nrd();
        golden_nv_select(i);
        int e = golden_nv_read();
        // 12 hour mode, hour 0 and 12: Hatari sets the PM flag for hour 0 and
        // not for hour 12; the MC146818A datasheet: 12 AM = 0h, 12 PM = 12h
        if (i == 4 && !(b & 2)) {
            time_t tt = (time_t)t;
            struct tm tmv;
            gmtime_r(&tt, &tmv);
            if (tmv.tm_hour == 0) e = (b & 4) ? 12 : 0x12;
            if (tmv.tm_hour == 12) e = ((b & 4) ? 12 : 0x12) | 0x80;
        }
        if (!fast_only)
            check(rd_r == e, "%s: u_real byte %d expected 0x%02x (Hatari, %s, B=0x%02x) got 0x%02x",
                  what, i, e, fmt_time(t), b, rd_r);
        check(rd_f == e, "%s: u_fast byte %d expected 0x%02x (Hatari, %s, B=0x%02x) got 0x%02x",
              what, i, e, fmt_time(t), b, rd_f);
    }
}

// ---------------------------------------------------------------------------
static void build_image(int vga, int lang, int kbd, bool by_pulse)
{
    top->cfg_vga = vga;
    top->cfg_lang = lang;
    top->cfg_kbd = kbd;
    if (by_pulse) {
        top->nv_init = 1;
        step();
        top->nv_init = 0;
    }
    clocks(60);
    golden_nv_init(vga, lang, kbd);
}

static void compare_image(const char *what)
{
    int sum = 0;
    for (int i = 14; i < 64; i++) {
        nsel(i);
        nrd();
        int e = golden_nv_byte(i);
        check(rd_r == e && rd_f == e, "%s: NVRAM byte %d expected %d (Hatari NvRam_Init) got %d/%d",
              what, i, e, rd_r, rd_f);
        top->nv_addr = i - 14;
        top->eval();
        check(top->r_nv_dout == e && top->f_nv_dout == e, "%s: nv_dout[%d] expected %d got %d",
              what, i - 14, e, top->r_nv_dout);
        if (i < 62) sum += rd_r;
    }
    // NvRam_SetChecksum: byte 62 = ~sum, 63 = sum over bytes 14..61
    nsel(62); nrd(); int c1 = rd_r;
    nsel(63); nrd(); int c2 = rd_r;
    check(c1 == ((~sum) & 0xff) && c2 == (sum & 0xff), "%s: checksum expected %02x/%02x got %02x/%02x",
          what, (~sum) & 0xff, sum & 0xff, c1, c2);
    // registers B, D and the alarm bytes against Hatari
    for (int i : {1, 3, 5, 11, 13}) {
        nsel(i); nrd();
        golden_nv_select(i);
        int e = golden_nv_read();
        check(rd_r == e && rd_f == e, "%s: register %d expected 0x%02x (Hatari) got 0x%02x/0x%02x",
              what, i, e, rd_r, rd_f);
    }
    // reg A: dividers/rate 42 as in Hatari (UIP bit compared in uip tests)
    nsel(10); nrd();
    check((rd_r & 0x7f) == (golden_nv_byte(10) & 0x7f), "%s: reg A expected 0x%02x got 0x%02x",
          what, golden_nv_byte(10) & 0x7f, rd_r & 0x7f);
    printf("  %s: bytes 20/21/28/29 = %d/%d/0x%02x/0x%02x checksum 0x%02x 0x%02x\n", what,
           golden_nv_byte(20), golden_nv_byte(21), golden_nv_byte(28), golden_nv_byte(29), c1, c2);
}

static void test_default_image()
{
    begin_test("default_image_vs_hatari");
    // first reset after configuration builds the image (cfg = VGA, US)
    top->cfg_vga = 1; top->cfg_lang = 0; top->cfg_kbd = 0;
    top->reset = 1;
    clocks(3);
    top->reset = 0;
    clocks(60);
    golden_nv_init(1, 0, 0);
    golden_nv_reset();
    compare_image("power-up VGA/US");
    build_image(0, 0, 0, true);
    compare_image("RGB/US");
    build_image(1, 1, 1, true);
    compare_image("VGA/DE");
    build_image(0, 2, 3, true);
    compare_image("RGB/FR kbd UK");
    build_image(1, 17, 8, true);
    compare_image("VGA/PL kbd CH_DE");
    // a later reset does not rebuild the image
    build_image(1, 0, 0, true);
    golden_nv_init(1, 0, 0);
    wr_both(40, 0x77);
    top->cfg_vga = 0;
    do_reset();
    clocks(60);
    nsel(40); nrd();
    check(rd_r == 0x77 && rd_f == 0x77, "reset must keep NVRAM contents, byte 40 expected 0x77 got 0x%02x", rd_r);
    nsel(29); nrd();
    check(rd_r == 0x1a, "reset must not rebuild the image with the new cfg_vga, byte 29 expected 0x1a got 0x%02x", rd_r);
    end_test();
}

static void test_time_read()
{
    begin_test("time_read_bcd_binary_12_24_vs_hatari");
    std::vector<long long> times = {
        1790812800LL + 13 * 3600 + 7 * 60 + 9,   // 2026-10-01 13:07:09 Thu
        1790812800LL,                             // 2026-10-01 00:00:00
        1790812800LL + 12 * 3600 + 59 * 60 + 59,  // 12:59:59
        1790812800LL + 23 * 3600 + 30 * 60,       // 23:30
        1790812800LL + 11 * 3600 + 5,             // 11:00:05
        946684800LL + 86399,                      // 2000-01-01 23:59:59 Sat
        1709164800LL + 3600,                      // 2024-02-29 01:00 Thu
        3029529600LL - 1,                         // 2065-12-31 23:59:59
        86400LL * 3 + 7 * 3600,                   // 1970-01-04 07:00 Sun
        1234567890LL,                             // 2009-02-13 23:31:30 Fri
    };
    const int modes[] = {0x06, 0x02, 0x04, 0x00};
    int n = 0;
    for (long long t : times) {
        set_rtc(t);
        for (int b : modes) {
            wr_both(11, b);
            compare_time(t, false, "loaded");
            n++;
        }
    }
    wr_both(11, 0x06);
    printf("  %d time/mode combinations compared\n", n);
    end_test();
}

static void test_time_writes()
{
    begin_test("time_writes_bcd_binary_12h");
    set_rtc(1790812800LL);
    // write in BCD mode with SET, read back in binary and BCD
    wr_both(11, 0x82);            // SET, BCD, 24h
    nsel(0); nwr(0x45);
    nsel(2); nwr(0x37);
    nsel(4); nwr(0x21);
    nsel(6); nwr(0x03);
    nsel(7); nwr(0x29);
    nsel(8); nwr(0x02);
    nsel(9); nwr(0x56);          // 2024
    struct { int idx, bcdv, binv; } exp[] = {{0, 0x45, 45}, {2, 0x37, 37}, {4, 0x21, 21},
        {6, 0x03, 3}, {7, 0x29, 29}, {8, 0x02, 2}, {9, 0x56, 56}};
    for (auto &e : exp) {
        nsel(e.idx); nrd();
        check(rd_r == e.bcdv && rd_f == e.bcdv, "BCD write/read byte %d expected 0x%02x got 0x%02x", e.idx, e.bcdv, rd_r);
    }
    nsel(11); nwr(0x86);          // SET, binary
    for (auto &e : exp) {
        nsel(e.idx); nrd();
        check(rd_r == e.binv && rd_f == e.binv, "binary read byte %d expected %d got %d", e.idx, e.binv, rd_r);
    }
    // binary writes, BCD reads
    nsel(0); nwr(59); nsel(2); nwr(8); nsel(7); nwr(31); nsel(8); nwr(12);
    nsel(11); nwr(0x82);
    int bexp[][2] = {{0, 0x59}, {2, 0x08}, {7, 0x31}, {8, 0x12}};
    for (auto &e : bexp) {
        nsel(e[0]); nrd();
        check(rd_r == e[1], "binary write / BCD read byte %d expected 0x%02x got 0x%02x", e[0], e[1], rd_r);
    }
    // 12 hour writes (BCD): 0x12 = 12 AM = 0h, 0x92 = 12 PM, 0x87 = 7 PM
    int h12[][2] = {{0x12, 0}, {0x92, 12}, {0x87, 19}, {0x07, 7}, {0x91, 23}};
    for (auto &h : h12) {
        nsel(11); nwr(0x80);      // SET, BCD, 12h
        nsel(4); nwr(h[0]);
        nsel(4); nrd();
        check(rd_r == h[0], "12h BCD hour 0x%02x read back 0x%02x", h[0], rd_r);
        nsel(11); nwr(0x86);      // binary 24h
        nsel(4); nrd();
        check(rd_r == h[1], "12h hour 0x%02x expected %d in 24h binary got %d", h[0], h[1], rd_r);
    }
    // seconds bit 7 is ignored on writes
    nsel(11); nwr(0x86);
    nsel(0); nwr(0x80 | 30);
    nsel(0); nrd();
    check(rd_r == 30, "seconds write 0x9e expected to read 30 got %d", rd_r);
    // writes to reg C and reg D are ignored (Hatari: read-only status registers)
    nsel(13); nwr(0x00); nsel(13); nrd();
    check(rd_r == 0x80, "reg D after write expected 0x80 got 0x%02x", rd_r);
    nsel(11); nwr(0x06);
    nsel(12); nrd();
    nsel(12); nwr(0xff); nsel(12); nrd();
    check((rd_r & 0x70) == 0x00, "reg C write must be ignored, got 0x%02x", rd_r);
    golden_nv_select(11); golden_nv_write(0x06);
    end_test();
}

static void test_running_clock()
{
    begin_test("calendar_rollover_vs_hatari");
    std::vector<long long> starts = {
        1677628800LL - 2,   // 2023-02-28 23:59:58 -> 2023-03-01 (non leap)
        1709164800LL - 86400 - 2, // 2024-02-28 23:59:58 -> 02-29 (leap)
        1709251200LL - 2,   // 2024-02-29 23:59:58 -> 03-01
        1735689600LL - 2,   // 2024-12-31 23:59:58 -> 2025-01-01
        1777593600LL - 2,   // 2026-04-30 23:59:58 -> 05-01
        1790380800LL - 2,   // 2026-09-26 Sat 23:59:58 -> Sunday
        946684800LL - 2,    // 1999-12-31 23:59:58 -> 2000-01-01
        1790812800LL + 3600 - 2,   // 00:59:58 -> 01:00
    };
    for (long long s : starts) {
        set_rtc(s);
        for (int b : {0x06, 0x02}) {
            wr_both(11, b);
            compare_time(s, false, "start");
        }
        // u_fast: one second = 131072 clocks; the update happens 65 ticks
        // (260 clocks) after the second boundary
        long long t = s;
        for (int k = 1; k <= 4; k++) {
            clocks(k == 1 ? 131072 + 400 : 131072);
            t++;
            wr_both(11, (k & 1) ? 0x06 : 0x00);
            compare_time(t, true, "running");
        }
    }
    wr_both(11, 0x06);
    end_test();
}

// u_real: second length and UIP timing at the real 32 MHz clock
static void test_uip_and_second()
{
    begin_test("uip_timing_and_1hz_at_32mhz");
    set_rtc(1790812800LL + 10);
    wr_both(11, 0x06);
    nsel(10);
    // poll reg A and the seconds every 64 clocks for 2.2 s
    std::vector<long> rises, falls, sec_changes;
    int prev_uip = 0, prev_sec = -1;
    const long poll = 64;
    for (long c = 0; c < 70400000L / poll; c++) {
        nsel(10); nrd();
        int uip = rd_r >> 7;
        nsel(0); nrd();
        int sec = rd_r;
        clocks(poll - 4);
        if (uip && !prev_uip) rises.push_back((long)cyc);
        if (!uip && prev_uip) falls.push_back((long)cyc);
        if (prev_sec >= 0 && sec != prev_sec) {
            sec_changes.push_back((long)cyc);
            check(sec == prev_sec + 1, "seconds expected %d after %d, got %d", prev_sec + 1, prev_sec, sec);
            check(prev_uip, "seconds must change at the end of the update cycle (UIP was 1 at the previous poll)");
        }
        prev_uip = uip;
        prev_sec = sec;
    }
    check(sec_changes.size() == 2, "expected 2 second changes in 2.2 s, got %zu", sec_changes.size());
    for (size_t i = 1; i < sec_changes.size(); i++) {
        long d = sec_changes[i] - sec_changes[i - 1];
        check(labs(d - 32000000L) <= 2 * poll, "second length expected 32000000 clocks +- %ld, got %ld", 2 * poll, d);
        printf("  second length %ld clocks\n", d);
    }
    // datasheet: UIP high from 244 us before the update to its end (1984 us)
    // = 73 ticks of 30.52 us = 2227.9 us = 71289 clocks
    size_t np = std::min(rises.size(), falls.size());
    check(np >= 2, "expected at least 2 UIP pulses, got %zu", np);
    for (size_t i = 0; i < np; i++) {
        long w = falls[i] - rises[i];
        check(labs(w - 71289) <= 2 * poll + 1000, "UIP width expected 71289 clocks (2228 us) got %ld", w);
        printf("  UIP pulse width %ld clocks (%.1f us)\n", w, w / 32.0);
    }
    // SET: UIP stays 0 and the clock stops
    wr_both(11, 0x86);
    nsel(0); nrd();
    int frozen = rd_r;
    bool uip_seen = false;
    for (long c = 0; c < 36000000L / 256; c++) {
        nsel(10); nrd();
        if (rd_r & 0x80) uip_seen = true;
        clocks(254);
    }
    nsel(0); nrd();
    check(!uip_seen, "UIP must stay 0 while SET = 1");
    check(rd_r == frozen, "time must not advance while SET = 1 (expected %d got %d)", frozen, rd_r);
    // set a new time with SET, release, time runs from it
    nsel(0); nwr(50); nsel(2); nwr(59);
    wr_both(11, 0x06);
    clocks(33000000L);
    nsel(0); nrd();
    int s = rd_r;
    nsel(2); nrd();
    check(s == 51 && rd_r == 59, "after SET release expected 59:51 after ~1 s, got %d:%d", rd_r, s);
    end_test();
}

static void test_flags()
{
    begin_test("flags_uf_af_pf_irq");
    // on u_fast (1 s = 131072 clocks)
    set_rtc(1790812800LL + 100);   // 00:01:40
    wr_both(11, 0x06);
    nsel(12); nrd();               // clear
    // UF: once per second, cleared by reading C
    clocks(131072 + 400);
    nsel(12); nrd();
    check((rd_f & 0x10) && !(rd_f & 0x80), "UF expected set (IRQF 0 without UIE) got 0x%02x", rd_f);
    nsel(12); nrd();
    check((rd_f & 0x10) == 0, "UF must be cleared by the read of reg C, got 0x%02x", rd_f);
    // UIE -> IRQF and irq output
    nsel(11); nwr(0x16);
    clocks(131072);
    check(top->f_irq, "irq expected with UF and UIE");
    nsel(12); nrd();
    check((rd_f & 0x90) == 0x90, "reg C expected IRQF|UF got 0x%02x", rd_f);
    check(!top->f_irq, "irq must drop after reading reg C");
    // AF: alarm at 00:01:45 (time now 00:01:42)
    nsel(11); nwr(0x06);
    nsel(0); nrd();
    int now = rd_f;
    nsel(1); nwr(now + 3); nsel(3); nwr(1); nsel(5); nwr(0);
    nsel(11); nwr(0x26);           // AIE
    nsel(12); nrd();
    int af_at = -1;
    for (int k = 1; k <= 5; k++) {
        clocks(131072);
        nsel(12); nrd();
        int c = rd_f;
        nsel(0); nrd();
        if (c & 0x20) {
            check(af_at < 0, "AF must be set only once");
            af_at = rd_f;
            check((c & 0x80) != 0, "IRQF expected with AF and AIE, reg C 0x%02x", c);
        }
    }
    check(af_at == now + 3, "AF expected at second %d got %d", now + 3, af_at);
    // don't care alarm bytes: AF every second
    nsel(1); nwr(0xff); nsel(3); nwr(0xc0); nsel(5); nwr(0xff);
    nsel(12); nrd();
    int afs = 0;
    for (int k = 0; k < 3; k++) { clocks(131072); nsel(12); nrd(); if (rd_f & 0x20) afs++; }
    check(afs == 3, "don't care alarm: AF expected each second (3) got %d", afs);
    nsel(11); nwr(0x06);
    // PF rates (32.768 kHz time base), counted over 1 s
    struct { int rs; int hz; } rates[] = {{3, 8192}, {6, 1024}, {15, 2}, {1, 256}, {2, 128}, {10, 64}};
    for (auto &r : rates) {
        nsel(10); nwr(0x20 | r.rs);
        nsel(12);
        nrd();
        int cnt = 0;
        long t0 = (long)cyc;
        while ((long)cyc - t0 < 131072) {
            nrd();
            if (rd_f & 0x40) cnt++;
        }
        check(abs(cnt - r.hz) <= 1, "RS=%d: PF expected %d times per second got %d", r.rs, r.hz, cnt);
        printf("  RS=%2d: PF %d times in 1 s\n", r.rs, cnt);
    }
    nsel(10); nwr(0x20);
    nsel(12); nrd();
    clocks(131072);
    nsel(12); nrd();
    check((rd_f & 0x40) == 0, "RS=0: no PF expected, reg C 0x%02x", rd_f);
    nsel(10); nwr(0x2a);
    // reset clears interrupt enables and flags (NvRam_Reset)
    nsel(11); nwr(0x7e);
    clocks(131072);
    golden_nv_select(11); golden_nv_write(0x7e);
    do_reset();
    nsel(11); nrd();
    golden_nv_select(11);
    int eb = golden_nv_read();
    check(rd_r == eb && rd_f == eb, "reg B after reset expected 0x%02x (Hatari NvRam_Reset) got 0x%02x", eb, rd_f);
    nsel(12); nrd();
    check(rd_r == 0 && rd_f == 0, "reg C after reset expected 0 (datasheet) got 0x%02x", rd_f);
    end_test();
}

static void test_storage_and_index()
{
    begin_test("nvram_storage_index_vs_hatari");
    golden_nv_reset();
    do_reset();
    // index after reset = 0 (NvRam_Reset)
    bus(0, false, false, true, 0);
    check(rd_r == golden_nv_index() && rd_r == 0, "index after reset expected 0 got %d", rd_r);
    uint32_t rng = 777;
    auto rnd = [&]() { rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5; return rng; };
    for (int i = 14; i < 64; i++) wr_both(i, rnd() & 0xff);
    for (int k = 0; k < 2000; k++) {
        int op = rnd() % 4;
        if (op == 0) {
            int v = rnd() & 0xff;   // includes out of range values (ignored)
            nsel(v);
            golden_nv_select(v);
            bus(0, false, false, true, 0);
            int e = golden_nv_index();
            check(rd_r == e && rd_f == e, "index after write 0x%02x expected %d got %d", v, e, rd_r);
        } else if (op == 1) {
            int i = 14 + rnd() % 50;
            nvch_r = nvch_f = false;
            wr_both(i, rnd() & 0xff);
            check(nvch_r && nvch_f, "nv_changed expected after CPU write to byte %d", i);
        } else {
            int i = 14 + rnd() % 50;
            nsel(i);
            golden_nv_select(i);
            nrd();
            int e = golden_nv_read();
            check(rd_r == e && rd_f == e, "byte %d expected 0x%02x (Hatari) got 0x%02x/0x%02x", i, e, rd_r, rd_f);
        }
    }
    // even bytes and $FF8960/$FF8962 read 0xff
    uint16_t dr;
    bus(0, false, true, true, 0, &dr);
    check((dr >> 8) == 0xff, "$FF8960 expected 0xff got 0x%02x", dr >> 8);
    bus(1, false, true, true, 0, &dr);
    check((dr >> 8) == 0xff, "$FF8962 expected 0xff got 0x%02x", dr >> 8);
    // even byte writes do nothing
    nsel(20);
    bus(1, true, true, false, 0x1200);
    nrd();
    golden_nv_select(20);
    check(rd_r == golden_nv_read(), "even byte write must be ignored");
    // writes to clock registers do not pulse nv_changed
    nvch_r = false;
    nsel(11); nwr(0x06);
    check(!nvch_r, "nv_changed must not pulse for register writes");
    // nv port: read and write
    for (int a = 0; a < 50; a++) {
        top->nv_addr = a;
        top->eval();
        golden_nv_select(14 + a);
        int e = golden_nv_read();
        check(top->r_nv_dout == e, "nv_dout[%d] expected 0x%02x got 0x%02x", a, e, top->r_nv_dout);
    }
    for (int a = 0; a < 50; a++) {
        top->nv_addr = a;
        top->nv_din = (a * 7 + 3) & 0xff;
        top->nv_wr = 1;
        step();
    }
    top->nv_wr = 0;
    top->nv_addr = 50;     // out of range: no write
    top->nv_din = 0xee;
    top->nv_wr = 1;
    step();
    top->nv_wr = 0;
    for (int a = 0; a < 50; a++) {
        nsel(14 + a); nrd();
        check(rd_r == ((a * 7 + 3) & 0xff), "byte %d after nv_wr expected 0x%02x got 0x%02x",
              14 + a, (a * 7 + 3) & 0xff, rd_r);
    }
    for (int i = 0; i < 14; i++) { nsel(i); nrd(); check(rd_r != 0xee || i == 1 || i == 3 || i == 5, "nv_addr 50 wrote byte %d", i); }
    end_test();
}

static void test_rtc_reload()
{
    begin_test("mister_rtc_reload");
    set_rtc(1790812800LL + 3 * 3600);
    wr_both(11, 0x02);
    compare_time(1790812800LL + 3 * 3600, false, "first");
    // same data, no toggle: software time is kept
    nsel(11); nwr(0x82); nsel(2); nwr(0x44); nsel(11); nwr(0x02);
    clocks(10);
    nsel(2); nrd();
    check(rd_r == 0x44, "software time must stay without an rtc toggle, got 0x%02x", rd_r);
    // toggle: reloaded
    set_rtc(1234567890LL);
    golden_nv_select(11); golden_nv_write(0x02);
    compare_time(1234567890LL, false, "reloaded");
    // day of week mapping: MiSTer Sunday = 0 -> NVRAM 1
    set_rtc(86400LL * 3);          // 1970-01-04 Sunday
    nsel(6); nrd();
    check(rd_r == 1, "Sunday expected day of week 1 got %d", rd_r);
    wr_both(11, 0x06);
    end_test();
}

int main(int argc, char **argv)
{
    Verilated::commandArgs(argc, argv);
    golden_nv_setup();
    top = new Vtb_nvram_top;
    top->clk = 0;
    top->reset = 0;
    top->bus_cs = top->bus_stb = top->bus_we = 0;
    top->nv_init = 0;
    top->nv_wr = 0;
    top->rtc[0] = top->rtc[1] = top->rtc[2] = 0;
    top->eval();

    test_default_image();
    test_storage_and_index();
    test_time_read();
    test_time_writes();
    test_running_clock();
    test_rtc_reload();
    test_flags();
    test_uip_and_second();

    printf("\nSUMMARY falcon_nvram: %d tests, %d failed, %ld checks, %ld failed checks\n",
           tests_run, tests_failed, n_checks, n_fail);
    printf("%s\n", tests_failed ? "FAIL" : "PASS");
    delete top;
    return tests_failed ? 1 : 0;
}
