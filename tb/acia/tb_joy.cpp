// tb_joy.cpp - IKBD joystick tests for falcon_acia (joystick audit against
// Hatari ikbd.c / joy.c, plus joystick buttons 2/3 and the $17 timing).
// Helpers are the same as in tb_acia.cpp.
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
    top->clk = 0; top->reset = 1; top->bus_cs = 0; top->bus_stb = 0;
    top->ps2_key = 0; top->ps2_mouse = 0; top->joystick_0 = 0; top->joystick_1 = 0; top->midi_rx = 1;
    run(10); top->reset = 0;
    wr8(0, 0x03); wr8(0, 0x96);
    expect("F1 after power-up", {0xF1}, 80 * MS);

    printf("J1: default mode (mouse rel + joystick event): port 1 fire is the right mouse button\n");
    top->joystick_1 = 0x10;
    expect("joy1 fire -> F9 00 00 (no FF packet)", {0xF9, 0x00, 0x00});
    top->joystick_1 = 0;
    expect("joy1 fire released -> F8 00 00", {0xF8, 0x00, 0x00});
    top->joystick_1 = 0x08;
    expect("joy1 up -> FF 01", {0xFF, 0x01});
    top->joystick_1 = 0; expect("joy1 released -> FF 00", {0xFF, 0x00});
    top->joystick_0 = 0x01;
    silence("joy0 right with mouse on: not reported (IKBD_GetJoystickData)", 10 * MS);
    top->joystick_0 = 0;
    top->joystick_1 = 0x20;   // MiSTer button 2
    expect("joy1 button 2 (bit 5): Space make $39 (Hatari Joy_ButtonSpaceJump, bEnableJumpOnFire2=false)", {0x39});
    top->joystick_1 = 0;
    expect("joy1 button 2 released: Space break $B9", {0xB9});
    drain();

    printf("J2: $12 mouse off -> port 0 joystick reported, left mouse = port 0 fire\n");
    ikbd_cmd({0x12});
    top->joystick_0 = 0x01;
    expect("joy0 right -> FE 08", {0xFE, 0x08});
    mouse(0, 0, 1);
    expect("left mouse button -> FE 88", {0xFE, 0x88});
    mouse(0, 0, 0);
    expect("released -> FE 08", {0xFE, 0x08});
    top->joystick_0 = 0; expect("joy0 released -> FE 00", {0xFE, 0x00});

    printf("J3: $14 with sticks held: immediate report\n");
    top->joystick_0 = 0x02; top->joystick_1 = 0x08 | 0x10;
    drain(3 * MS);
    ikbd_cmd({0x08}); drain(3*MS);   // mouse back on
    ikbd_cmd({0x14});
    expect("$14 immediate: FE 04 FF 81", {0xFE, 0x04, 0xFF, 0x81});
    silence("no repeat", 10 * MS);

    printf("J4: $16 ignores mouse buttons\n");
    ikbd_cmd({0x15});
    mouse(0, 0, 3); run(2 * MS);
    top->joystick_1 = 0x08;
    ikbd_cmd({0x16});
    expect("$16 -> FD 04 01 (no mouse buttons)", {0xFD, 0x04, 0x01});
    printf("J5: $17 includes mouse buttons as fire (mouse off in monitoring)\n");
    ikbd_cmd({0x17, 0x01});
    std::vector<int> v; for (int i = 0; i < 2; i++) v.push_back(recv(0, 30 * MS));
    CHECK(v[0] == 0x03 && v[1] == 0x41, "$17 with both mouse buttons: expected 03 41 got %s", hex(v).c_str());
    mouse(0, 0, 0); ikbd_cmd({0x1A}); run(5*MS); drain(5*MS);
    top->joystick_0 = 0x01; top->joystick_1 = 0x02;
    silence("$1A: nothing reported", 10 * MS);
    ikbd_cmd({0x9A}); expect("$9A -> F6 1A", {0xF6, 0x1A, 0, 0, 0, 0, 0, 0});
    ikbd_cmd({0x16}); expect("$16 still works after $1A -> FD 08 04", {0xFD, 0x08, 0x04});

    printf("J6: after $80 $01 with port 1 held\n");
    top->joystick_0 = 0; top->joystick_1 = 0x08;
    ikbd_cmd({0x80, 0x01});
    expect("F1 then FF 01", {0xF1, 0xFF, 0x01}, 80 * MS);
    top->joystick_1 = 0; drain();


    printf("J7: joystick button 2 = Space key (Hatari JoystickSpaceBar)\n");
    drain();
    top->joystick_1 = 0x20;
    expect("port 1 button 2 pressed -> 39", {0x39});
    silence("held: no repeat", 30 * MS);
    top->joystick_1 = 0x20 | 0x08;
    expect("held with up: only the joystick packet FF 01", {0xFF, 0x01});
    top->joystick_1 = 0;
    expect("released -> FF 00 then B9 (key after the joystick packet)", {0xFF, 0x00, 0xB9});
    silence("no repeat of the break", 20 * MS);
    top->joystick_0 = 0x20;
    silence("port 0 button 2 while the mouse is on: port 0 is not read", 20 * MS);
    top->joystick_0 = 0;
    ikbd_cmd({0x12});                                 // mouse off: port 0 read
    top->joystick_0 = 0x20;
    expect("port 0 button 2 with mouse off -> 39", {0x39});
    top->joystick_0 = 0;
    expect("released -> B9", {0xB9});
    ikbd_cmd({0x17, 0x02});                           // monitoring
    top->joystick_1 = 0x20;
    {
        std::vector<int> got;
        for (int i = 0; i < 6; i++) got.push_back(recv(0, 25 * MS));
        bool nospace = true;
        for (int x : got) if (x == 0x39 || x == 0xB9) nospace = false;
        CHECK(nospace, "monitoring mode: no Space key (IKBD_SendAutoKeyboardCommands returns early): got %s",
              hex(got).c_str());
    }
    top->joystick_1 = 0;
    ikbd_cmd({0x08});
    run(10 * MS);
    drain();

    printf("J8: joystick button 3 = autofire (Hatari: fire off while (nVBLs & 7) < 4)\n");
    ikbd_cmd({0x14});                                 // joystick events, mouse off
    drain();
    top->joystick_1 = 0x40;
    {
        std::vector<uint64_t> t; std::vector<int> d;
        for (int i = 0; i < 5; i++) {
            uint64_t tt; int a = recv(0, 120 * MS, &tt), b = recv(0, 5 * MS);
            if (a != 0xFF) break;
            t.push_back(tt); d.push_back(b);
        }
        bool alt = d.size() == 5;
        for (size_t i = 1; i < d.size(); i++) if (d[i] == d[i - 1] || (d[i] != 0x80 && d[i] != 0x00)) alt = false;
        CHECK(alt, "port 1 button 3 held: FF 80 / FF 00 alternate, got %s", hex(d).c_str());
        bool per = t.size() == 5;
        for (size_t i = 2; i < t.size(); i++) {
            double ms = (t[i] - t[i - 1]) / 32000.0;
            if (ms < 79.0 || ms > 81.0) per = false;
            printf("    interval %zu: %.2f ms\n", i, ms);
        }
        CHECK(per, "autofire half period: expected 4 VBLs at 50 Hz = 80 ms (+-1 ms) between reports");
    }
    top->joystick_1 = 0x40 | 0x10;                    // button 1 + button 3
    {
        std::vector<int> d;
        for (int i = 0; i < 3; i++) { int a = recv(0, 120 * MS); int b = recv(0, 5 * MS); d.push_back(a); d.push_back(b); }
        bool ok = true;
        for (int i = 0; i < 6; i += 2) if (d[i] != 0xFF) ok = false;
        ok = ok && d[1] != d[3] && d[3] != d[5];
        CHECK(ok, "button 1 + button 3: fire still toggles (Hatari clears fire in the off phase): got %s", hex(d).c_str());
    }
    top->joystick_1 = 0;
    run(5 * MS); drain();
    top->joystick_0 = 0x40;                           // port 0 (mouse off since $14)
    {
        std::vector<int> d;
        for (int i = 0; i < 3; i++) { int a = recv(0, 120 * MS); int b = recv(0, 5 * MS); d.push_back(a); d.push_back(b); }
        bool ok = d[0] == 0xFE && d[2] == 0xFE && d[4] == 0xFE && d[1] != d[3] && d[3] != d[5];
        CHECK(ok, "port 0 button 3: FE 80 / FE 00 alternate, got %s", hex(d).c_str());
    }
    top->joystick_0 = 0;
    run(5 * MS); drain();
    ikbd_cmd({0x15});
    top->joystick_1 = 0x40;
    {
        int s80 = 0, s00 = 0;
        for (int i = 0; i < 12; i++) {
            ikbd_cmd({0x16});
            int a = recv(0, 10 * MS), b = recv(0, 5 * MS), c = recv(0, 5 * MS);
            if (a == 0xFD && b == 0x00) { if (c == 0x80) s80++; else if (c == 0x00) s00++; }
            run(15 * MS);
        }
        CHECK(s80 > 0 && s00 > 0 && s80 + s00 == 12, "$16 with button 3 held sees both fire states: %d x 80, %d x 00 of 12", s80, s00);
    }
    top->joystick_1 = 0;

    printf("J9: $17 first report rate x 10 ms after the command (audit F9)\n");
    for (int k = 0; k < 3; k++) {
        ikbd_cmd({0x15});
        run((3 + 4 * k) * MS + 1234 * k);             // different phases of the free running timers
        drain(2 * MS);
        ikbd_cmd({0x17, 0x05});
        uint64_t t0 = cyc, t1, t2;
        int a = recv(0, 80 * MS, &t1); recv(0, 5 * MS);
        int b = recv(0, 80 * MS, &t2); recv(0, 5 * MS);
        double first = (t1 - t0) / 32000.0, period = (t2 - t1) / 32000.0;
        // t0 is ~1.5 bits after the IKBD got the command; the report needs <=1 bit
        // to start and 9.5 bits on the wire: 50 ms + 8..9.5 bits (1.02..1.22 ms)
        CHECK(a >= 0 && first > 50.9 && first < 51.4,
              "phase %d: first $17 report after 50 ms + 8..9.5 bits (50.9..51.4 ms), got %.3f ms", k, first);
        CHECK(b >= 0 && period > 49.9 && period < 50.1, "phase %d: then every 50 ms, got %.3f ms", k, period);
    }
    ikbd_cmd({0x08});
    run(5 * MS); drain();

    printf("\nSUMMARY: %d passed, %d failed\n", n_pass, n_fail);
    printf("%s\n", n_fail == 0 ? "RESULT: PASS" : "RESULT: FAIL");
    delete top;
    return n_fail == 0 ? 0 : 1;
}
