// Verilator test of scc.v (with scc_chan.v). clk runs at 32 MHz and PCLK at 8 MHz
// (one edge every 4 clocks). Exit status 0 when every check passes.
#include "Vscc.h"
#include <cstdio>
#include <cstdint>
#include <vector>

static Vscc* t;
static int fails = 0;
static long now = 0;
static bool loop_a = true;          // channel A's TxD wired to its RxD
static int ext_rxd_a = 1;           // RxD A when not looped back

static void tick() {
	t->pclk_ce = (now % 4) == 0;
	t->rxd = (loop_a ? (t->txd & 1) : ext_rxd_a) | 2;   // B idles high
	t->clk = 0; t->eval(); t->clk = 1; t->eval();
	now++;
}
static void run(long n) { while (n--) tick(); }

enum { A_CTL = 0, A_DATA = 1, B_CTL = 2, B_DATA = 3 };   // addr[2:1]

static void wr(int port, int v) {
	t->sel = 1; t->we = 1; t->addr = port; t->din = v; tick();
	t->sel = 0; t->we = 0; tick();
}
static int rd(int port) {
	t->sel = 1; t->we = 0; t->addr = port; t->eval();
	int v = t->dout; tick(); t->sel = 0; tick();
	return v;
}
static void wreg(int ch, int r, int v) {     // through the pointer
	int ctl = ch ? B_CTL : A_CTL;
	if (r) wr(ctl, r < 8 ? r : 0x08 | (r & 7));
	wr(ctl, v);
}
static int rreg(int ch, int r) {
	int ctl = ch ? B_CTL : A_CTL;
	if (r) wr(ctl, r < 8 ? r : 0x08 | (r & 7));
	return rd(ctl);
}

static void check(const char* what, long got, long want) {
	if (got != want) { printf("FAIL %s: %lx, want %lx\n", what, got, want); fails++; }
	else printf("ok   %s\n", what);
}

static const int TC = 24;                     // 9615 baud at 8 MHz, x16
static const long BIT = 2L * (TC + 2) * 16 * 4;   // clocks per bit
static const long CHAR = BIT * 10;

static void setup_async(int ch, int wr4, int wr3, int wr5) {
	wreg(ch, 4, wr4);
	wreg(ch, 11, 0x50);                       // Rx and Tx clocks from the BRG
	wreg(ch, 12, TC); wreg(ch, 13, 0);
	wreg(ch, 14, 0x03);                       // BRG from PCLK, enabled
	wreg(ch, 3, wr3);
	wreg(ch, 5, wr5);
}

// drive one async frame onto RxD A by hand
static void send_frame(int byte, int nbits, int parity /* -1 none, 0/1 bit */, int stopbit) {
	ext_rxd_a = 0; run(BIT);
	for (int i = 0; i < nbits; i++) { ext_rxd_a = (byte >> i) & 1; run(BIT); }
	if (parity >= 0) { ext_rxd_a = parity; run(BIT); }
	ext_rxd_a = stopbit; run(BIT);
	ext_rxd_a = 1; run(BIT);
}

static int flush_a(const char* where) {
	int n = 0;
	while ((rreg(0, 0) & 1) && n < 8) { printf("     (%s: left in FIFO %02x, RR1 %02x)\n", where, rd(A_DATA), rreg(0, 1)); n++; }
	wr(A_CTL, 0x30);
	return n;
}

int main() {
	t = new Vscc;
	t->dcd = 0; t->cts = 0; t->sync = 0; t->rtxc_ce = 0; t->trxc_ce = 0;
	t->reset = 1; run(4); t->reset = 0; run(4);

	check("reset: RR0 A = Tx empty + underrun", rreg(0, 0), 0x44);
	check("reset: RR1 A = all sent, residue 011", rreg(0, 1), 0x07);
	check("reset: no IRQ", t->irq, 0);

	// the pointer: WR0 = $0C points at WR12 (point high + 4); it falls back to 0
	wr(A_CTL, 0x0C); wr(A_CTL, 0x5A);
	check("point high: WR12 written, RR12 reads it", rreg(0, 12), 0x5A);
	check("pointer back at 0: next read is RR0", rd(A_CTL) & 0x04, 0x04);

	// TOS-like async setup: x16, 1 stop, no parity, 8 bits, Rx and Tx on, DTR+RTS
	wr(A_CTL, 0x09); wr(A_CTL, 0xC0);         // WR9: force hardware reset
	setup_async(0, 0x44, 0xC1, 0xEA);
	check("RTS and DTR follow WR5", t->rts & 1 | (t->dtr & 1) << 1, 3);

	// bit timing on TxD: falling edge of the start bit to the first data edge
	wr(A_DATA, 0x55);
	long t0 = -1, t1 = -1;
	for (long n = 0; n < CHAR * 2 && t1 < 0; n++) {
		int before = t->txd & 1; tick(); int after = t->txd & 1;
		if (before && !after && t0 < 0) t0 = now;
		else if (t0 >= 0 && before != after) t1 = now;
	}
	check("one bit = 2 (TC+2) x16 PCLKs", t1 - t0, BIT);
	run(CHAR * 2);
	check("loopback: byte received", rreg(0, 0) & 1, 1);
	check("loopback: $55", rd(A_DATA), 0x55);
	check("FIFO empty after the read", rreg(0, 0) & 1, 0);
	check("all sent", rreg(0, 1) & 1, 1);

	// three bytes queue in the FIFO; a fourth overruns
	for (int b : {0x11, 0x22, 0x33, 0x44}) { wr(A_DATA, b); run(CHAR + BIT * 2); }
	check("overrun after 4 bytes unread", rreg(0, 1) & 0x20, 0x20);
	int b0 = rd(A_DATA), b1 = rd(A_DATA), b2 = rd(A_DATA);
	check("FIFO order 11 22", b0 << 8 | b1, 0x1122);
	check("the overrunning byte replaced the last", b2, 0x44);
	check("FIFO empty", rreg(0, 0) & 1, 0);
	check("overrun stays until error reset", rreg(0, 1) & 0x20, 0x20);
	wr(A_CTL, 0x30);                          // error reset
	check("error reset clears overrun", rreg(0, 1) & 0x20, 0);

	// 7 bits, even parity, 2 stop bits, by hand on RxD
	loop_a = false;
	wreg(0, 4, 0x4F);                          // x16, 2 stop, even parity, enabled
	wreg(0, 3, 0x41);                          // 7 bits, Rx on
	send_frame(0x41, 7, 0, 1);                 // 'A' = 1000001: two ones, even parity 0
	check("7E2: 'A' received", rd(A_DATA) & 0x7F, 0x41);
	check("no parity error", rreg(0, 1) & 0x10, 0);
	send_frame(0x41, 7, 1, 1);                 // wrong parity
	rd(A_DATA);
	check("parity error latched", rreg(0, 1) & 0x10, 0x10);
	wr(A_CTL, 0x30);
	send_frame(0x42, 7, 0, 0);                 // stop bit low: framing error
	check("framing error on that character", rreg(0, 1) & 0x40, 0x40);
	rd(A_DATA);
	check("framing error goes with the character", rreg(0, 1) & 0x40, 0);

	// break: RxD low for longer than a frame
	wreg(0, 15, 0x80); wreg(0, 1, 0x01);       // break interrupt enable, ext interrupts on
	wreg(0, 9, 0x08);                          // MIE
	ext_rxd_a = 0; run(CHAR * 2);
	check("break seen in RR0", rreg(0, 0) & 0x80, 0x80);
	check("break raises the ext/status interrupt", t->irq, 1);
	ext_rxd_a = 1; run(CHAR);
	check("RR0 keeps the break latched", rreg(0, 0) & 0x80, 0x80);
	wr(A_CTL, 0x10);                           // reset ext/status
	run(CHAR);                                 // the end of the break: another change
	wr(A_CTL, 0x10);
	check("after reset ext/status the latch follows the line", rreg(0, 0) & 0x80, 0);
	rd(A_DATA);                                // the break's null character
	wr(A_CTL, 0x30);
	check("IRQ gone", t->irq, 0);

	// interrupts with vectors: Rx on all characters, Tx; vector base $60 with status
	loop_a = true;
	wreg(0, 4, 0x44); wreg(0, 3, 0xC1);
	run(CHAR * 2);
	flush_a("after the break test");
	wreg(0, 15, 0x00); wreg(0, 1, 0x12);       // Rx int on all chars, Tx int
	wreg(0, 2, 0x60);
	wreg(0, 9, 0x09);                          // MIE + VIS
	wr(A_DATA, 0x77);
	run(10);
	check("Tx buffer empty: IRQ", t->irq, 1);
	check("vector: A Tx empty = status 4", t->vector, 0x60 | (4 << 1));
	check("RR3 A Tx IP", rreg(0, 3), 0x10);
	t->iack = 1; tick(); t->iack = 0; tick();
	check("IACK sets IUS: IRQ drops", t->irq, 0);
	run(CHAR + BIT * 2);
	check("A Rx above A Tx in service: IRQ again", t->irq, 1);
	check("vector: A Rx = status 6", t->vector, 0x60 | (6 << 1));
	check("RR2B carries the status", rreg(1, 2), 0x60 | (6 << 1));
	check("RR2A is WR2", rreg(0, 2), 0x60);
	t->iack = 1; tick(); t->iack = 0; tick();
	check("IACK of A Rx", t->irq, 0);
	check("$77 received", rd(A_DATA), 0x77);
	wr(A_CTL, 0x38);                           // reset highest IUS (A Rx)
	check("A Tx still in service: no IRQ", t->irq, 0);
	wr(A_CTL, 0x28);                           // reset Tx IP
	wr(A_CTL, 0x38);                           // reset highest IUS (A Tx)
	check("all clear", t->irq, 0);

	// status high: WR9 bit 4 puts the status, reversed, in bits 6-4
	wreg(0, 9, 0x19);
	wr(A_DATA, 0x01); run(CHAR + BIT * 2);
	check("status high, A Rx: 110 reversed -> 011 in bits 6-4", t->vector, (0x60 & 0x8f) | (3 << 4));
	rd(A_DATA); wr(A_CTL, 0x28);

	// No Vector, and MIE off clears IUS
	wreg(0, 9, 0x0B);
	check("NV: vector disabled", t->vector_en, 0);
	wreg(0, 9, 0x00);
	check("MIE off: no IRQ", t->irq, 0);

	// ext/status: DCD on channel B, latched until reset ext/status
	wreg(1, 15, 0x08); wreg(1, 1, 0x01); wreg(0, 9, 0x09);
	t->dcd = 2; run(4);
	check("DCD B change: IRQ", t->irq, 1);
	check("vector: B ext = status 1", t->vector, 0x60 | (1 << 1));
	check("RR0 B shows DCD", rreg(1, 0) & 0x08, 0x08);
	t->dcd = 0; run(4);
	check("latched: RR0 B still shows DCD", rreg(1, 0) & 0x08, 0x08);
	wr(B_CTL, 0x10);
	check("reset ext/status: the change since is a new interrupt", t->irq, 1);
	check("and RR0 B now shows DCD off", rreg(1, 0) & 0x08, 0);
	wr(B_CTL, 0x10);
	check("quiet line: IRQ gone", t->irq, 0);

	// software IACK through RR2
	wreg(0, 9, 0x29);                          // MIE, VIS, software IACK
	t->dcd = 2; run(4);
	check("soft IACK: IRQ pending", t->irq, 1);
	rreg(1, 2);
	check("reading RR2 acknowledged it", t->irq, 0);
	wr(B_CTL, 0x10); wr(B_CTL, 0x38);
	t->dcd = 0; run(4); wr(B_CTL, 0x10);

	// channel reset leaves the other channel alone; force reset clears all
	wreg(1, 12, 0x33);
	wr(A_CTL, 0x09); wr(A_CTL, 0x80);          // reset channel A
	check("channel A reset keeps channel B's WR12", rreg(1, 12), 0x33);
	wr(A_CTL, 0x09); wr(A_CTL, 0xC0);
	check("force reset: RR0 A", rreg(0, 0), 0x44);
	check("force reset: no IRQ", t->irq, 0);

	printf(fails ? "%d FAILED\n" : "all passed\n", fails);
	delete t;
	return fails != 0;
}
