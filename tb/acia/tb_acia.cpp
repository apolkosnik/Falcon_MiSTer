// tb_acia.cpp - Verilator testbench for falcon_acia (two MC6850 + IKBD).
// Everything is driven through the real register bus, as the CPU would.
// Expected values are derived from Hatari src/acia.c, src/ikbd.c and the
// MC6850 datasheet (see the comments at each check).

#include "Vfalcon_acia.h"
#include "verilated.h"
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <vector>
#include <string>

static Vfalcon_acia *top;
static uint64_t cyc = 0;
static bool midi_loop = false;
static int  midi_manual = 1;
static int  n_pass = 0, n_fail = 0;
static uint32_t key_tog = 0, mouse_tog = 0;

static const double CLK_HZ = 32000000.0;
static const uint64_t BIT_IKBD = 4096;           // 32 MHz / 7812.5
static const uint64_t BIT_MIDI = 1024;           // 32 MHz / 31250
static const uint64_t MS = 32000;

#define CHECK(cond, ...) do { \
    if (cond) { n_pass++; printf("  PASS: "); printf(__VA_ARGS__); printf("\n"); } \
    else { n_fail++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } \
} while (0)

static void tick() {
    top->midi_rx = midi_loop ? top->midi_tx : midi_manual;
    top->clk = 0; top->eval();
    top->clk = 1; top->eval();
    cyc++;
}
static void run(uint64_t n) { for (uint64_t i = 0; i < n; i++) tick(); }

// one CPU bus access, returns bus_dout sampled on the bus_ack clock
static uint16_t bus(bool we, int a, bool uds, bool lds, uint16_t din, int *wait = nullptr) {
    top->bus_cs = 1; top->bus_stb = 1; top->bus_we = we;
    top->bus_addr = a; top->bus_uds = uds; top->bus_lds = lds; top->bus_din = din;
    tick();
    top->bus_stb = 0;
    int n = 1;
    while (!top->bus_ack) {
        tick();
        if (++n > 100000) { printf("  FAIL: bus access timeout\n"); n_fail++; break; }
    }
    uint16_t d = top->bus_dout;
    // bus_ack must be exactly one clock
    top->bus_cs = 0;
    tick();
    if (top->bus_ack) { printf("  FAIL: bus_ack longer than one clock\n"); n_fail++; }
    if (wait) *wait = n;
    return d;
}
static uint8_t rd8(int a) { return bus(false, a, true, false, 0) >> 8; }
static void wr8(int a, uint8_t v) { bus(true, a, true, false, (uint16_t)v << 8); }

// IKBD ACIA = regs 0/1, MIDI ACIA = regs 2/3
static int recv(int base, uint64_t timeout, uint64_t *t = nullptr) {
    uint64_t end = cyc + timeout;
    while (cyc < end) {
        uint8_t s = rd8(base);
        if (s & 1) {
            if (t) *t = cyc;
            return rd8(base + 1);
        }
    }
    return -1;
}
static void send(int base, uint8_t b) {
    uint64_t end = cyc + 100 * MS;
    while (!(rd8(base) & 2) && cyc < end) ;
    wr8(base + 1, b);
}
// send a command and wait until the IKBD has received its last byte
// (TDR->TSR transfer, then start + 8 data + stop = 10 bit times on the wire)
static void ikbd_cmd(std::vector<uint8_t> v, bool wait_rx = true) {
    for (auto b : v) send(0, b);
    if (wait_rx) {
        uint64_t end = cyc + 100 * MS;
        while (!(rd8(0) & 2) && cyc < end) ;
        run(11 * BIT_IKBD);
    }
}

static std::string hex(const std::vector<int> &v) {
    std::string s; char buf[8];
    for (int x : v) { if (x < 0) s += "-- "; else { snprintf(buf, sizeof buf, "%02X ", x); s += buf; } }
    return s;
}

// receive bytes and compare with the expected sequence
static bool expect(const char *name, std::vector<uint8_t> exp, uint64_t timeout = 30 * MS) {
    std::vector<int> got;
    std::vector<int> e(exp.begin(), exp.end());
    bool ok = true;
    for (size_t i = 0; i < exp.size(); i++) {
        int b = recv(0, timeout);
        got.push_back(b);
        if (b != exp[i]) { ok = false; if (b < 0) break; }
    }
    CHECK(ok, "%s: expected [%s] got [%s]", name, hex(e).c_str(), hex(got).c_str());
    return ok;
}
static bool silence(const char *name, uint64_t clocks) {
    int b = recv(0, clocks);
    CHECK(b < 0, "%s: expected no IKBD byte for %.1f ms, got %s", name, clocks / 32000.0,
          b < 0 ? "none" : hex({b}).c_str());
    return b < 0;
}
static void drain(uint64_t quiet = 5 * MS) { while (recv(0, quiet) >= 0) ; }

static void key(uint8_t code, bool ext, bool press) {
    key_tog ^= 1;
    top->ps2_key = (key_tog << 10) | (press << 9) | (ext << 8) | code;
}
static void mouse(int dx, int dy_ps2, int buttons) {
    mouse_tog ^= 1;
    uint32_t status = 0x08 | (buttons & 7) | (dx < 0 ? 0x10 : 0) | (dy_ps2 < 0 ? 0x20 : 0);
    top->ps2_mouse = (mouse_tog << 24) | ((uint32_t)(dy_ps2 & 0xFF) << 16) |
                     ((uint32_t)(dx & 0xFF) << 8) | status;
}

// sample the MIDI TX pin: wait for the start bit edge, then sample mid-bit
static std::vector<int> capture_frame(uint64_t bit, int nbits, uint64_t *start_t) {
    std::vector<int> v;
    uint64_t end = cyc + 200 * MS;
    while (top->midi_tx && cyc < end) tick();
    *start_t = cyc;
    run(bit / 2);
    for (int i = 0; i < nbits; i++) { v.push_back(top->midi_tx); run(bit); }
    return v;
}

// drive a serial frame on midi_rx (bit list, LSB first incl. start/stop)
static void drive_frame(const std::vector<int> &bits, uint64_t bit) {
    for (int b : bits) { midi_manual = b; run(bit); }
    midi_manual = 1;
}
static std::vector<int> frame8(uint8_t v, int par /*0 none 1 even 2 odd*/, int stop, int nbits = 8) {
    std::vector<int> f; f.push_back(0);
    int p = 0;
    for (int i = 0; i < nbits; i++) { int b = (v >> i) & 1; f.push_back(b); p ^= b; }
    if (par == 1) f.push_back(p);
    if (par == 2) f.push_back(p ^ 1);
    f.push_back(stop);
    return f;
}

int main(int argc, char **argv) {
    Verilated::commandArgs(argc, argv);
    top = new Vfalcon_acia;
    top->clk = 0; top->reset = 1;
    top->bus_cs = 0; top->bus_stb = 0;
    top->ps2_key = 0; top->ps2_mouse = 0; top->joystick_0 = 0; top->joystick_1 = 0;
    top->midi_rx = 1;
    run(10);
    top->reset = 0;
    uint64_t t_reset = cyc;

    // ------------------------------------------------------------------
    printf("TEST 1: power-up state, odd bytes, bus protocol\n");
    {
        int w;
        uint16_t d = bus(false, 0, true, false, 0, &w);
        // Hatari ACIA_Init: SR = 0 before the first master reset
        CHECK((d >> 8) == 0x00, "IKBD SR at power-up: expected 00 got %02X", d >> 8);
        CHECK((d & 0xFF) == 0xFF, "odd byte of a word read at $FFFC00: expected FF got %02X", d & 0xFF);
        CHECK(w >= 24 && w <= 24 + 41, "access wait (6 cycles @8MHz + E sync): expected 24..65 clocks got %d", w);
        d = bus(false, 0, false, true, 0);
        CHECK((d & 0xFF) == 0xFF, "odd byte read ($FFFC01): expected FF got %02X", d & 0xFF);
        d = bus(false, 3, false, true, 0);
        CHECK((d & 0xFF) == 0xFF, "odd byte read ($FFFC07): expected FF got %02X", d & 0xFF);
        CHECK(top->irq == 0, "irq at power-up: expected 0 got %d", top->irq);
    }

    // ------------------------------------------------------------------
    printf("TEST 2: master reset and control register\n");
    {
        wr8(0, 0x03);
        uint8_t s = rd8(0);
        // ACIA_MasterReset: SR = TDRE
        CHECK(s == 0x02, "SR after master reset $03: expected 02 got %02X", s);
        wr8(0, 0x96);  // /64, 8N1, RIE (TOS value)
        s = rd8(0);
        CHECK(s == 0x02, "SR after CR=$96: expected 02 got %02X", s);
        CHECK(top->irq == 0, "irq with RIE and nothing received: expected 0 got %d", top->irq);
    }

    // ------------------------------------------------------------------
    printf("TEST 3: IKBD power-up reset sends $F1 after 62.6 ms\n");
    {
        uint64_t t;
        int b = recv(0, 80 * MS, &t);
        double ms = (t - t_reset) / 32000.0;
        // IKBD_RESET_CYCLES = 502000 @ 8021247 Hz = 62.58 ms, + delay 0..2 bits + 10 bits
        CHECK(b == 0xF1, "power-up byte: expected F1 got %02X", b & 0xFF);
        CHECK(ms > 62.58 && ms < 62.58 + 13 * 0.128 + 0.1,
              "power-up $F1 time: expected 62.58..64.3 ms got %.3f ms", ms);
        uint8_t s = rd8(0);
        CHECK(s == 0x02, "SR after reading RDR: expected 02 got %02X", s);
    }

    // ------------------------------------------------------------------
    printf("TEST 4: TDRE timing at 7812.5 baud and TX interrupt\n");
    {
        // byte $00 is not an IKBD command (ignored)
        run(BIT_IKBD * 3);
        uint64_t t0 = cyc;
        wr8(1, 0x00);
        uint8_t s = rd8(0);
        CHECK((s & 2) == 0, "TDRE right after TDR write: expected 0 got %d", (s >> 1) & 1);
        while (!(rd8(0) & 2) && cyc < t0 + 10 * BIT_IKBD) ;
        uint64_t t1 = cyc;
        // ACIA_Clock_TX: TDR goes to TSR on the next bit clock when idle
        CHECK(t1 - t0 <= BIT_IKBD + 200, "TDRE set again (TDR->TSR) within one bit: expected <= %llu clocks got %llu",
              (unsigned long long)(BIT_IKBD + 200), (unsigned long long)(t1 - t0));
        wr8(1, 0x00);
        s = rd8(0);
        CHECK((s & 2) == 0, "second byte written while first is shifting: TDRE expected 0 got %d", (s >> 1) & 1);
        while (!(rd8(0) & 2) && cyc < t1 + 20 * BIT_IKBD) ;
        uint64_t t2 = cyc;
        int64_t dt = (int64_t)(t2 - t1);
        // start + 8 data + 1 stop = 10 bit times before the next load
        CHECK(llabs(dt - (int64_t)(10 * BIT_IKBD)) <= 300,
              "TDRE for the second byte after one 10 bit frame: expected %llu +-300 clocks got %lld",
              (unsigned long long)(10 * BIT_IKBD), (long long)dt);
        // TX interrupt enable (CR6:5 = 01)
        wr8(0, 0xB6);
        s = rd8(0);
        CHECK(s == 0x82 && top->irq == 1, "CR=$B6 with TDRE: expected SR 82 irq 1 got SR %02X irq %d", s, top->irq);
        wr8(1, 0x00);
        CHECK(top->irq == 0, "irq after TDR write: expected 0 got %d", top->irq);
        // the previous byte is still shifting: TDR->TSR happens when it ends
        uint64_t te = cyc + 12 * BIT_IKBD;
        while (!top->irq && cyc < te) tick();
        CHECK(top->irq == 1, "irq after TDR->TSR transfer: expected 1 got %d", top->irq);
        wr8(0, 0x96);
        CHECK(top->irq == 0, "irq after CR=$96 (TX irq off): expected 0 got %d", top->irq);
        run(12 * BIT_IKBD);
        drain(2 * MS);
    }

    // ------------------------------------------------------------------
    printf("TEST 5: MIDI ACIA divide select, word select, bit timing on midi_tx\n");
    {
        CHECK(top->midi_tx == 1, "midi_tx idle level: expected 1 got %d", top->midi_tx);
        struct { uint8_t cr; uint64_t bit; const char *name; } dv[] = {
            {0x15, 1024, "/16 (31250 baud)"}, {0x16, 4096, "/64 (7812.5 baud)"}, {0x14, 64, "/1 (500 kbaud)"} };
        for (auto &d : dv) {
            wr8(2, 0x03);
            wr8(2, d.cr);
            wr8(3, 0x55);
            // measure all edges of start + 01010101 + stop: every bit is an edge
            uint64_t end = cyc + 30 * d.bit + 1000;
            while (top->midi_tx && cyc < end) tick();
            uint64_t last = cyc;
            std::vector<uint64_t> w;
            int lvl = 0;
            while (w.size() < 9 && cyc < end) {
                tick();
                if (top->midi_tx != lvl) { w.push_back(cyc - last); last = cyc; lvl = top->midi_tx; }
            }
            bool ok = w.size() == 9;
            for (auto x : w) if (x != d.bit) ok = false;
            CHECK(ok, "%s: 9 bit cells of %llu clocks each, got %zu cells, first %llu", d.name,
                  (unsigned long long)d.bit, w.size(), w.empty() ? 0ULL : (unsigned long long)w[0]);
            run(4 * d.bit);
        }
        // word select 000 = 7 data, even parity, 2 stop bits; /16
        wr8(2, 0x03);
        wr8(2, 0x01);
        wr8(3, 0x41);
        uint64_t ts;
        auto f = capture_frame(1024, 12, &ts);
        // start, 1000001 (LSB first), even parity 0, stop 1, stop 1, idle 1
        std::vector<int> e = {0, 1, 0, 0, 0, 0, 0, 1, 0, 1, 1, 1};
        CHECK(f == e, "7E2 frame of $41: expected 0 1000001 0 1 1 got %s", [&] {
            std::string s; for (int b : f) s += char('0' + b); return s; }().c_str());
        run(4 * 1024);
        // word select 111 = 8 data, odd parity, 1 stop
        wr8(2, 0x03);
        wr8(2, 0x1D);
        wr8(3, 0xA5);
        f = capture_frame(1024, 11, &ts);
        e = {0, 1, 0, 1, 0, 0, 1, 0, 1, 1, 1};   // a5 = 10100101, 4 ones -> odd parity 1
        CHECK(f == e, "8O1 frame of $A5: expected 0 10100101 1 1 got %s", [&] {
            std::string s; for (int b : f) s += char('0' + b); return s; }().c_str());
        run(4 * 1024);
    }

    // ------------------------------------------------------------------
    printf("TEST 6: MIDI loopback midi_tx -> midi_rx\n");
    {
        midi_loop = true;
        wr8(2, 0x03);
        wr8(2, 0x95);    // RIE, 8N1, /16
        uint8_t bytes[] = {0x90, 0x3C, 0x7F, 0x00, 0xFF};
        for (uint8_t v : bytes) {
            uint64_t t0 = cyc;
            wr8(3, v);
            while (!top->irq && cyc < t0 + 20 * BIT_MIDI) tick();
            uint64_t dt = cyc - t0;
            uint8_t s = rd8(2);
            CHECK(s == 0x83, "MIDI SR with byte received: expected 83 got %02X", s);
            CHECK(dt >= 10 * BIT_MIDI - BIT_MIDI && dt <= 11 * BIT_MIDI + 200,
                  "MIDI byte round trip: expected about 10 bit times (9..11 x 1024 clocks) got %llu",
                  (unsigned long long)dt);
            uint8_t r = rd8(3);
            CHECK(r == v, "MIDI loopback byte: expected %02X got %02X", v, r);
            CHECK(top->irq == 0, "irq after RDR read: expected 0 got %d", top->irq);
        }
        // 7 bit odd parity 1 stop (ws 011)
        wr8(2, 0x03);
        wr8(2, 0x8D);
        wr8(3, 0x5A);
        uint64_t t0 = cyc;
        while (!top->irq && cyc < t0 + 20 * BIT_MIDI) tick();
        uint8_t s = rd8(2);
        uint8_t r = rd8(3);
        CHECK(s == 0x83 && r == 0x5A, "7O1 loopback: expected SR 83 data 5A got SR %02X data %02X", s, r);
        // overrun (Hatari: OVRN appears when RDR is read, cleared by SR read + RDR read)
        wr8(2, 0x03);
        wr8(2, 0x95);
        wr8(3, 0x11);
        while (!(rd8(2) & 2)) ;
        wr8(3, 0x22);
        while (!(rd8(2) & 2)) ;
        wr8(3, 0x33);
        run(40 * BIT_MIDI);
        s = rd8(2);
        CHECK(s == 0x83, "SR after 3 unread bytes: expected 83 (RDRF, OVRN not yet visible) got %02X", s);
        r = rd8(3);
        CHECK(r == 0x11, "RDR after overrun: expected first byte 11 got %02X", r);
        s = rd8(2);
        CHECK(s == 0x22, "SR after reading RDR: expected 22 (OVRN, TDRE) got %02X", s);
        CHECK(top->irq == 0, "irq after overrun read: expected 0 got %d", top->irq);
        r = rd8(3);
        s = rd8(2);
        CHECK(s == 0x02, "SR after SR read + RDR read: expected 02 (OVRN cleared) got %02X", s);
        midi_loop = false;
    }

    // ------------------------------------------------------------------
    printf("TEST 7: MIDI receive errors driven on midi_rx\n");
    {
        midi_manual = 1;
        wr8(2, 0x03);
        wr8(2, 0x19);   // 8 data even parity 1 stop, /16, no irq
        run(4 * BIT_MIDI);
        auto f = frame8(0x81, 2 /*odd: wrong for even*/, 1);
        drive_frame(f, BIT_MIDI);
        run(2 * BIT_MIDI);
        uint8_t s = rd8(2);
        CHECK(s == 0x43, "parity error: expected SR 43 (PE, TDRE, RDRF) got %02X", s);
        uint8_t r = rd8(3);
        s = rd8(2);
        CHECK(r == 0x81 && s == 0x02, "after RDR read: expected data 81 SR 02 got %02X SR %02X", r, s);
        f = frame8(0x81, 1, 0);         // stop bit 0 = framing error
        drive_frame(f, BIT_MIDI);
        run(2 * BIT_MIDI);
        s = rd8(2);
        CHECK(s == 0x13, "framing error: expected SR 13 (FE, TDRE, RDRF) got %02X", s);
        r = rd8(3);
        f = frame8(0x7E, 1, 1);
        drive_frame(f, BIT_MIDI);
        run(2 * BIT_MIDI);
        s = rd8(2);
        r = rd8(3);
        CHECK(s == 0x03 && r == 0x7E, "good frame after FE: expected SR 03 data 7E got SR %02X data %02X", s, r);
        // false start bit (glitch shorter than half a bit) is ignored
        midi_manual = 0; run(BIT_MIDI / 4); midi_manual = 1;
        run(12 * BIT_MIDI);
        s = rd8(2);
        CHECK(s == 0x02, "glitch shorter than half a bit: expected SR 02 got %02X", s);
    }

    // ------------------------------------------------------------------
    printf("TEST 8: IKBD reset command $80 $01 -> $F1, output dropped during reset\n");
    {
        drain(2 * MS);
        send(0, 0x80);
        send(0, 0x01);
        uint64_t t0 = cyc;
        run(15 * BIT_IKBD);
        ikbd_cmd({0x16});      // answer is dropped while the IKBD resets
        uint64_t t;
        int b = recv(0, 80 * MS, &t);
        double ms = (t - t0) / 32000.0;
        CHECK(b == 0xF1, "first byte after reset command: expected F1 got %s", hex({b}).c_str());
        // $01 waits for the $80 frame (<= 10 bits) + 9.5 bits to the IKBD, then
        // 62.58 ms (502000 cycles @ 8021247 Hz) + 1..2 bits SCI delay + 9.5 bits back
        // = 62.58 ms + 29..31.5 bit times (3.71..4.03 ms)
        CHECK(ms > 62.58 + 3.6 && ms < 62.58 + 4.2, "reset command -> $F1: expected 66.18..66.78 ms got %.3f ms", ms);
        silence("no answer to $16 sent during reset", 10 * MS);
        ikbd_cmd({0x92});
        expect("$92 mouse availability after reset (mouse on)", {0xF6, 0x00, 0, 0, 0, 0, 0, 0});
        ikbd_cmd({0x94});
        expect("$94 joystick mode after reset (event reporting)", {0xF6, 0x14, 0, 0, 0, 0, 0, 0});
        ikbd_cmd({0x8F});
        expect("$8F vertical after reset (Y=0 at top)", {0xF6, 0x10, 0, 0, 0, 0, 0, 0});
        ikbd_cmd({0x8B});
        expect("$8B threshold after reset (1,1)", {0xF6, 0x0B, 1, 1, 0, 0, 0, 0});
    }

    // ------------------------------------------------------------------
    printf("TEST 9: IKBD ACIA receive interrupt and overrun\n");
    {
        drain(2 * MS);
        ikbd_cmd({0x1C});
        run(20 * MS);                    // 7 bytes = 9 ms + 7 bit delay
        CHECK(top->irq == 1, "irq with unread IKBD byte (RIE): expected 1 got %d", top->irq);
        uint8_t s = rd8(0);
        CHECK(s == 0x83, "SR: expected 83 (IRQ, TDRE, RDRF) got %02X", s);
        uint8_t r = rd8(1);
        CHECK(r == 0xFC, "first byte of the time packet kept in RDR: expected FC got %02X", r);
        s = rd8(0);
        CHECK(s == 0x22, "SR after RDR read: expected 22 (OVRN, TDRE) got %02X", s);
        rd8(1);
        s = rd8(0);
        CHECK(s == 0x02, "SR after SR+RDR read: expected 02 got %02X", s);
        CHECK(top->irq == 0, "irq: expected 0 got %d", top->irq);
    }

    // ------------------------------------------------------------------
    printf("TEST 10: keyboard make/break codes\n");
    {
        drain(2 * MS);
        struct { uint8_t code; bool ext; uint8_t st; const char *name; } k[] = {
            {0x1C, 0, 0x1E, "A"}, {0x76, 0, 0x01, "Esc"}, {0x05, 0, 0x3B, "F1"},
            {0x83, 0, 0x41, "F7"}, {0x09, 0, 0x44, "F10"}, {0x78, 0, 0x62, "F11=Help"},
            {0x07, 0, 0x61, "F12=Undo"}, {0x75, 1, 0x48, "Up"}, {0x6B, 1, 0x4B, "Left"},
            {0x72, 1, 0x50, "Down"}, {0x74, 1, 0x4D, "Right"},
            {0x70, 1, 0x52, "Insert"}, {0x6C, 1, 0x47, "Home=ClrHome"}, {0x71, 1, 0x53, "Delete"},
            {0x69, 1, 0x62, "End=Help"}, {0x5A, 1, 0x72, "KP Enter"}, {0x4A, 1, 0x65, "KP /"},
            {0x7C, 0, 0x66, "KP *"}, {0x6C, 0, 0x67, "KP 7"}, {0x70, 0, 0x70, "KP 0"},
            {0x14, 1, 0x1D, "Right Ctrl"}, {0x11, 1, 0x38, "Right Alt"}, {0x12, 0, 0x2A, "Left Shift"},
            {0x59, 0, 0x36, "Right Shift"}, {0x61, 0, 0x60, "ISO <>"}, {0x5A, 0, 0x1C, "Return"},
            {0x29, 0, 0x39, "Space"}, {0x0E, 0, 0x29, "`"}, {0x5D, 0, 0x2B, "\\"} };
        for (auto &x : k) {
            key(x.code, x.ext, true);
            run(MS);
            key(x.code, x.ext, false);
            std::string n = std::string(x.name) + " make/break";
            expect(n.c_str(), {x.st, (uint8_t)(x.st | 0x80)});
        }
        key(0x1C, 0, true); run(2 * MS);
        key(0x1C, 0, true); run(2 * MS);   // typematic repeat
        key(0x1C, 0, false);
        expect("A with a PS/2 typematic repeat: one make, one break", {0x1E, 0x9E});
        silence("no extra byte after the repeat", 5 * MS);
        key(0x1F, 1, true); run(MS); key(0x1F, 1, false);   // left GUI, no ST key
        silence("unmapped key (left GUI)", 5 * MS);
    }

    // ------------------------------------------------------------------
    printf("TEST 11: relative mouse\n");
    {
        drain(2 * MS);
        mouse(5, 3, 0);
        expect("dx=+5 dy=+3 (PS/2 up) -> F8 05 FD", {0xF8, 0x05, 0xFD});
        mouse(0, 0, 1);
        expect("left button down -> FA 00 00", {0xFA, 0x00, 0x00});
        mouse(0, 0, 0);
        expect("left button up -> F8 00 00", {0xF8, 0x00, 0x00});
        mouse(0, 0, 2);
        expect("right button down -> F9 00 00", {0xF9, 0x00, 0x00});
        mouse(0, 0, 0);
        expect("right button up -> F8 00 00", {0xF8, 0x00, 0x00});
        mouse(-10, -7, 0);
        expect("dx=-10 dy=-7 (PS/2 down) -> F8 F6 07", {0xF8, 0xF6, 0x07});
        mouse(200, 0, 0);
        expect("dx=+200 -> F8 7F 00, F8 49 00", {0xF8, 0x7F, 0x00, 0xF8, 0x49, 0x00});
        ikbd_cmd({0x0B, 0x05, 0x05});
        ikbd_cmd({0x8B});
        expect("$8B threshold report", {0xF6, 0x0B, 5, 5, 0, 0, 0, 0});
        mouse(3, 0, 0);
        silence("dx=3 below threshold 5", 10 * MS);
        mouse(3, 0, 0);
        expect("accumulated dx=6 >= threshold -> F8 06 00", {0xF8, 0x06, 0x00});
        ikbd_cmd({0x0B, 0x01, 0x01});
        ikbd_cmd({0x0F});
        mouse(0, 4, 0);
        expect("Y=0 at bottom ($0F): PS/2 up 4 -> F8 00 04", {0xF8, 0x00, 0x04});
        ikbd_cmd({0x8F});
        expect("$8F report after $0F", {0xF6, 0x0F, 0, 0, 0, 0, 0, 0});
        ikbd_cmd({0x10});
        mouse(0, 4, 0);
        expect("Y=0 at top ($10): PS/2 up 4 -> F8 00 FC", {0xF8, 0x00, 0xFC});
        ikbd_cmd({0x88});
        expect("$88 mouse mode report (relative)", {0xF6, 0x08, 0, 0, 0, 0, 0, 0});
    }

    // ------------------------------------------------------------------
    printf("TEST 12: absolute mouse\n");
    {
        drain(2 * MS);
        ikbd_cmd({0x09, 0x01, 0x40, 0x00, 0xC8});     // max 320 x 200
        ikbd_cmd({0x0E, 0x00, 0x00, 0x64, 0x00, 0x32});  // X=100 Y=50
        mouse(10, -5, 0);
        silence("no automatic packets in absolute mode", 10 * MS);
        ikbd_cmd({0x0D});
        expect("$0D after dx=+10 dy=+5 (down) -> F7 00 006E 0037", {0xF7, 0x00, 0x00, 0x6E, 0x00, 0x37});
        mouse(0, 0, 1);
        run(5 * MS);
        ikbd_cmd({0x0D});
        expect("$0D after left press -> buttons 04", {0xF7, 0x04, 0x00, 0x6E, 0x00, 0x37});
        mouse(0, 0, 0);
        run(5 * MS);
        ikbd_cmd({0x0D});
        expect("$0D after left release -> buttons 08", {0xF7, 0x08, 0x00, 0x6E, 0x00, 0x37});
        mouse(0, 0, 2);
        run(5 * MS);
        mouse(0, 0, 0);
        run(5 * MS);
        ikbd_cmd({0x0D});
        expect("$0D after right click -> buttons 03", {0xF7, 0x03, 0x00, 0x6E, 0x00, 0x37});
        ikbd_cmd({0x0D});
        expect("$0D with no change -> buttons 00", {0xF7, 0x00, 0x00, 0x6E, 0x00, 0x37});
        mouse(-200, 0, 0);
        run(5 * MS);
        mouse(0, -127, 0);
        run(5 * MS);
        mouse(0, -127, 0);
        run(5 * MS);
        ikbd_cmd({0x0D});
        expect("clamped to X=0, Y=max 200", {0xF7, 0x00, 0x00, 0x00, 0x00, 0xC8});
        for (int i = 0; i < 3; i++) { mouse(127, 0, 0); run(5 * MS); }
        ikbd_cmd({0x0D});
        expect("clamped to X=max 320", {0xF7, 0x00, 0x01, 0x40, 0x00, 0xC8});
        ikbd_cmd({0x89});
        expect("$89 mouse mode report (absolute, max 320x200)", {0xF6, 0x09, 0x01, 0x40, 0x00, 0xC8, 0, 0});
        // scale: X 2 ticks per unit, Y 3 ticks per unit
        ikbd_cmd({0x0C, 0x02, 0x03});
        ikbd_cmd({0x0E, 0x00, 0x00, 0x64, 0x00, 0x32});
        mouse(10, -9, 0);
        run(5 * MS);
        ikbd_cmd({0x0D});
        expect("scale 2/3: dx=10 dy=9 -> X=105 Y=53", {0xF7, 0x00, 0x00, 0x69, 0x00, 0x35});
        ikbd_cmd({0x8C});
        expect("$8C scale report", {0xF6, 0x0C, 2, 3, 0, 0, 0, 0});
        ikbd_cmd({0x0C, 0x00, 0x00});
        // mouse button action: press reports absolute position
        ikbd_cmd({0x07, 0x01});
        ikbd_cmd({0x87});
        expect("$87 mouse action report", {0xF6, 0x07, 0x01, 0, 0, 0, 0, 0});
        mouse(0, 0, 1);
        expect("action $01: left press -> automatic F7 04 position", {0xF7, 0x04, 0x00, 0x69, 0x00, 0x35});
        mouse(0, 0, 0);
        silence("action $01: release does not report", 10 * MS);
        ikbd_cmd({0x07, 0x04});   // buttons act like keys
        mouse(0, 0, 1);
        expect("action $04: left press -> key 74", {0x74});
        mouse(0, 0, 0);
        expect("action $04: left release -> key F4", {0xF4});
        ikbd_cmd({0x07, 0x00});
        drain(5 * MS);
    }

    // ------------------------------------------------------------------
    printf("TEST 13: mouse keycode mode\n");
    {
        ikbd_cmd({0x0A, 0x02, 0x02});
        mouse(4, 0, 0);
        expect("dx=+4 with delta 2 -> Right Right", {0x4D, 0xCD, 0x4D, 0xCD});
        mouse(0, -2, 0);
        expect("dy=+2 (down) -> Down", {0x50, 0xD0});
        mouse(-2, 2, 0);
        expect("dx=-2 dy=-2 -> Left + Up", {0x4B, 0xCB, 0x48, 0xC8});
        ikbd_cmd({0x8A});
        expect("$8A mouse mode report (keycode 2,2)", {0xF6, 0x0A, 2, 2, 0, 0, 0, 0});
        ikbd_cmd({0x08});
        drain(5 * MS);
    }

    // ------------------------------------------------------------------
    printf("TEST 14: joysticks, event mode $14 and interrogation $16\n");
    {
        ikbd_cmd({0x14});
        silence("$14 with both joysticks idle", 10 * MS);
        ikbd_cmd({0x92});
        expect("$92 after $14: mouse disabled", {0xF6, 0x12, 0, 0, 0, 0, 0, 0});
        top->joystick_1 = 0x08 | 0x10;     // up + fire
        expect("joystick 1 up+fire -> FF 81", {0xFF, 0x81});
        top->joystick_0 = 0x01;            // right
        expect("joystick 0 right -> FE 08", {0xFE, 0x08});
        top->joystick_1 = 0;
        expect("joystick 1 released -> FF 00", {0xFF, 0x00});
        top->joystick_0 = 0x04 | 0x02;     // down + left
        expect("joystick 0 down+left -> FE 06", {0xFE, 0x06});
        top->joystick_0 = 0;
        expect("joystick 0 released -> FE 00", {0xFE, 0x00});
        // mouse buttons become fire buttons when the mouse is off
        mouse(0, 0, 2);
        expect("right mouse button with mouse off -> joystick 1 fire FF 80", {0xFF, 0x80});
        mouse(0, 0, 0);
        expect("released -> FF 00", {0xFF, 0x00});
        ikbd_cmd({0x15});
        top->joystick_0 = 0x02;            // left
        top->joystick_1 = 0x04;            // down
        silence("$15 interrogation mode: no event packets", 10 * MS);
        ikbd_cmd({0x16}, false);
        uint64_t t0 = cyc, t;
        int b = recv(0, 30 * MS, &t);
        CHECK(b == 0xFD, "$16 header: expected FD got %s", hex({b}).c_str());
        double ms = (t - t0) / 32000.0;
        // <=1 bit TDR->TSR + 9.5 bits to the IKBD + 8 bit delay (Hatari 7500..10000
        // cycles / 1024) + <=1 bit alignment + 9.5 bits back = 27..29 bit times
        CHECK(ms > 3.3 && ms < 4.0, "$16 response time: expected 3.3..4.0 ms got %.3f ms", ms);
        expect("$16 data: joy0 left, joy1 down", {0x04, 0x02});
        ikbd_cmd({0x94});
        expect("$94 after $15", {0xF6, 0x15, 0, 0, 0, 0, 0, 0});
        ikbd_cmd({0x1A});
        ikbd_cmd({0x9A});
        expect("$9A after $1A (joysticks disabled)", {0xF6, 0x1A, 0, 0, 0, 0, 0, 0});
        top->joystick_0 = 0; top->joystick_1 = 0;
    }

    // ------------------------------------------------------------------
    printf("TEST 15: joystick monitoring $17 and fire button monitoring $18\n");
    {
        top->joystick_0 = 0x02;            // left
        top->joystick_1 = 0x08 | 0x10;     // up + fire
        ikbd_cmd({0x17, 0x02});            // every 20 ms
        uint64_t t1, t2;
        int a = recv(0, 40 * MS, &t1), b2 = recv(0, 5 * MS);
        int c = recv(0, 40 * MS, &t2), d = recv(0, 5 * MS);
        CHECK(a == 0x01 && b2 == 0x41 && c == 0x01 && d == 0x41,
              "monitoring pairs: expected 01 41 01 41 got %s", hex({a, b2, c, d}).c_str());
        double ms = (t2 - t1) / 32000.0;
        CHECK(ms > 19.0 && ms < 21.0, "monitoring rate 2: expected 20 ms between pairs got %.2f ms", ms);
        key(0x1C, 0, true); run(MS); key(0x1C, 0, false);
        std::vector<int> got;
        for (int i = 0; i < 4; i++) got.push_back(recv(0, 25 * MS));
        bool nokey = true;
        for (int x : got) if (x == 0x1E || x == 0x9E) nokey = false;
        CHECK(nokey, "keys are not reported in monitoring mode: got %s", hex(got).c_str());
        ikbd_cmd({0x18});
        // drain monitoring bytes still in flight
        run(10 * MS);
        for (int i = 0; i < 3; i++) recv(0, 5 * MS);   // bytes queued before / while switching
        std::vector<int> fb;
        for (int i = 0; i < 3; i++) fb.push_back(recv(0, 5 * MS));
        CHECK(fb[0] == 0xFF && fb[1] == 0xFF && fb[2] == 0xFF,
              "fire monitoring, joystick 1 fire held: expected FF FF FF got %s", hex(fb).c_str());
        top->joystick_1 = 0;
        run(3 * MS);
        for (int i = 0; i < 3; i++) recv(0, 5 * MS);   // bytes sampled before the release
        fb.clear();
        for (int i = 0; i < 3; i++) fb.push_back(recv(0, 5 * MS));
        CHECK(fb[0] == 0x00 && fb[1] == 0x00 && fb[2] == 0x00,
              "fire monitoring, released: expected 00 00 00 got %s", hex(fb).c_str());
        ikbd_cmd({0x15});
        run(5 * MS);
        drain(5 * MS);
        top->joystick_0 = 0;
        ikbd_cmd({0x08});
        drain(5 * MS);
    }

    // ------------------------------------------------------------------
    printf("TEST 16: pause output $13 / resume $11\n");
    {
        ikbd_cmd({0x13});
        key(0x1C, 0, true);
        silence("key while output paused", 10 * MS);
        ikbd_cmd({0x11});
        expect("$11 resumes: queued make code", {0x1E});
        key(0x1C, 0, false);
        expect("break code", {0x9E});
        ikbd_cmd({0x13});
        key(0x32, 0, true);
        run(3 * MS);
        ikbd_cmd({0x8F});                 // any command resumes
        expect("any command resumes output", {0x30, 0xF6, 0x10, 0, 0, 0, 0, 0, 0});
        key(0x32, 0, false);
        expect("break", {0xB0});
    }

    // ------------------------------------------------------------------
    printf("TEST 17: memory load / read commands\n");
    {
        ikbd_cmd({0x20, 0x00, 0x80, 0x03, 0x16, 0x16, 0x16});   // 3 data bytes that look like $16
        silence("memory load data bytes are not executed", 15 * MS);
        ikbd_cmd({0x21, 0x00, 0x80});
        expect("$21 memory read", {0xF6, 0x20, 0, 0, 0, 0, 0, 0});
        ikbd_cmd({0x99, 0x00});            // unknown second byte after a 1 byte command
        expect("$99 joystick mode report (interrogation mode after $15)", {0xF6, 0x15, 0, 0, 0, 0, 0, 0});
        silence("byte $00 is not a command", 10 * MS);
    }

    // ------------------------------------------------------------------
    printf("TEST 18: time of day clock\n");
    {
        auto readclk = [&](std::vector<int> &v) {
            ikbd_cmd({0x1C});
            v.clear();
            for (int i = 0; i < 7; i++) v.push_back(recv(0, 30 * MS));
        };
        std::vector<int> v;
        ikbd_cmd({0x1B, 0x99, 0x12, 0x31, 0x23, 0x59, 0x58});
        readclk(v);
        bool ok = v[0] == 0xFC && v[1] == 0x99 && v[2] == 0x12 && v[3] == 0x31 && v[4] == 0x23 &&
                  v[5] == 0x59 && (v[6] == 0x58 || v[6] == 0x59);
        CHECK(ok, "$1C after setting 99-12-31 23:59:58: expected FC 99 12 31 23 59 58/59 got %s", hex(v).c_str());
        uint64_t end = cyc + 3200 * MS;
        do { run(100 * MS); readclk(v); } while (v[1] != 0x00 && cyc < end);
        std::vector<int> e = {0xFC, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00};
        CHECK(v == e, "rollover 99-12-31 23:59:59 -> expected FC 00 01 01 00 00 00 got %s", hex(v).c_str());
        // leap year: 2024-02-28 23:59:59 -> 02-29
        ikbd_cmd({0x1B, 0x24, 0x02, 0x28, 0x23, 0x59, 0x59});
        end = cyc + 1200 * MS;
        do { run(100 * MS); readclk(v); } while (v[4] == 0x23 && cyc < end);
        e = {0xFC, 0x24, 0x02, 0x29, 0x00, 0x00, 0x00};
        CHECK(v == e, "leap year 24-02-28 23:59:59 -> expected FC 24 02 29 00 00 00 got %s", hex(v).c_str());
        // invalid BCD year is ignored, the rest is set
        ikbd_cmd({0x1B, 0xFA, 0x03, 0x31, 0x23, 0x59, 0x59});
        end = cyc + 1200 * MS;
        do { run(100 * MS); readclk(v); } while (v[4] == 0x23 && cyc < end);
        e = {0xFC, 0x24, 0x04, 0x01, 0x00, 0x00, 0x00};
        CHECK(v == e, "invalid year $FA ignored, 03-31 rolls to 04-01: expected FC 24 04 01 00 00 00 got %s",
              hex(v).c_str());
        // month 0 stops the clock (Hatari 2026/08/22)
        ikbd_cmd({0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
        run(1100 * MS);
        readclk(v);
        e = {0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
        CHECK(v == e, "month 0: clock does not advance: expected FC 00 00 00 00 00 00 got %s", hex(v).c_str());
    }

    // ------------------------------------------------------------------
    printf("TEST 19: reset-time quirk ($12 $14 during reset -> mouse and joystick)\n");
    {
        ikbd_cmd({0x80, 0x01});
        ikbd_cmd({0x12, 0x14});
        int b = recv(0, 80 * MS);
        CHECK(b == 0xF1, "F1 after reset: expected F1 got %s", hex({b}).c_str());
        ikbd_cmd({0x92});
        expect("$92: mouse re-enabled by the reset quirk", {0xF6, 0x00, 0, 0, 0, 0, 0, 0});
        ikbd_cmd({0x94});
        expect("$94: joystick event mode", {0xF6, 0x14, 0, 0, 0, 0, 0, 0});
        top->joystick_0 = 0x01;            // right on port 0 (read because bBothMouseAndJoy)
        expect("joystick 0 reported together with the mouse", {0xFE, 0x08});
        mouse(1, 0, 0);
        expect("and relative mouse packets", {0xF8, 0x01, 0x00});
        top->joystick_0 = 0;
        drain(5 * MS);
    }

    // ------------------------------------------------------------------
    printf("TEST 20: system reset resets the IKBD, not the ACIA registers\n");
    {
        wr8(2, 0x03);
        wr8(2, 0x95);
        top->reset = 1; run(4); top->reset = 0;
        uint64_t t0 = cyc;
        uint8_t s = rd8(2);
        CHECK(s == 0x02, "MIDI ACIA SR after system reset (no reset pin): expected 02 got %02X", s);
        uint64_t t;
        int b = recv(0, 80 * MS, &t);
        double ms = (t - t0) / 32000.0;
        CHECK(b == 0xF1 && ms > 62.58 && ms < 64.3, "IKBD $F1 after system reset: expected F1 at 62.6..64.3 ms got %s at %.3f ms",
              hex({b}).c_str(), ms);
    }

    printf("\nSUMMARY: %d passed, %d failed\n", n_pass, n_fail);
    printf("%s\n", n_fail == 0 ? "RESULT: PASS" : "RESULT: FAIL");
    delete top;
    return n_fail == 0 ? 0 : 1;
}
