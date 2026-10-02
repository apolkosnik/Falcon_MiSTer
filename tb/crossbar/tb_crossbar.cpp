// tb_crossbar.cpp - Verilator testbench for rtl/falcon/falcon_crossbar.sv
//
// Drives the real falcon_crossbar module.  Expected values come from Hatari
// src/falcon/crossbar.c (register layout, frequency / gain / attenuation
// tables, DMA play / record data paths, SNDINT/SOUNDINT behaviour) and the
// deviations documented in the RTL header.
//
// Environment: a 16 MB RAM model behind the DMA port with a random latency
// of 6..20 clocks per access, a register bus master, and a DSP SSI model.

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <vector>
#include <string>
#include <functional>
#include "Vfalcon_crossbar.h"
#include "verilated.h"

static const double CLK_HZ = 32000000.0;
static const int LINE_PULSE = 32;

static Vfalcon_crossbar *top;
static uint64_t cyc = 0;
static std::vector<uint8_t> ram(16 * 1024 * 1024, 0);
static int fails = 0, checks = 0;
static int test_fails = 0;

// ---------------------------------------------------------------- tables
static const int ste_rates[4] = { 6258, 12517, 25033, 50066 };
static const int f25[15] = { 49170, 32780, 24585, 19668, 16390, 14049, 12292, 10927,
                             9834, 8940, 8195, 7565, 7024, 6556, 6146 };
static const int f32[15] = { 62500, 41666, 31250, 25000, 20833, 17857, 15624, 13889,
                             12500, 11363, 10416, 9615, 8928, 8333, 7812 };
static const int adc_tab[16] = { 3276, 3894, 4628, 5500, 6537, 7769, 9234, 10975,
                                 13043, 15502, 18424, 21897, 26025, 30931, 36761, 43691 };
static const int dac_tab[16] = { 65535, 55142, 46396, 39037, 32846, 27636, 23253, 19565,
                                 16462, 13851, 11654, 9806, 8250, 6942, 5841, 4915 };

static void check(bool ok, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void check(bool ok, const char *fmt, ...)
{
	va_list ap;
	checks++;
	if (!ok) {
		fails++; test_fails++;
		printf("  FAIL: ");
		va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
		printf("\n");
	}
}
static void info(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
static void info(const char *fmt, ...)
{
	va_list ap;
	printf("  ");
	va_start(ap, fmt); vprintf(fmt, ap); va_end(ap);
	printf("\n");
}

// ---------------------------------------------------------------- monitors
struct Pair { int16_t l, r; };
static std::vector<Pair> audio;          // every audio_stb sample
static uint64_t n_audio_stb = 0;
static int snd_fall = 0, snd_rise = 0, scnt_fall = 0, scnt_rise = 0;
static int last_snd = 1, last_scnt = 1;
static uint64_t snd_low_at = 0, snd_high_at = 0;
static std::vector<uint64_t> snd_rise_t, snd_fall_t;
static uint64_t n_slot = 0, n_tx = 0, n_rx = 0, n_rx_only = 0;
static bool capture_audio = false;

// DSP model
enum DspMode { DSP_NONE, DSP_FRAMEBUF, DSP_COUNTER };
static DspMode dsp_mode = DSP_NONE;
static std::vector<uint16_t> dsp_rx_log;
static std::vector<int> dsp_rxframe_log;
static uint16_t rxbuf[8], txbuf[8], txcur[8];
static int rxi = 0, txi = 0;
static int dsp_slots = 2;
static uint16_t tx_counter = 0x1000;
static bool tx_valid_toggle = false;
static uint64_t n_tx_valid_slots = 0;

// DMA model
static int dma_wait = -1;
static bool dma_ack_next = false;
static int dma_lat_min = 6, dma_lat_max = 20;
static uint64_t dma_reads = 0, dma_writes = 0;

static void tick()
{
	// inputs for this edge are set; rising edge
	top->clk = 0; top->eval();
	top->clk = 1; top->eval();
	cyc++;
	// ---- after the edge: look at outputs
	top->dma_ack = 0;
	if (dma_ack_next) {
		dma_ack_next = false;
	} else if (top->dma_req) {
		if (dma_wait < 0) dma_wait = dma_lat_min + rand() % (dma_lat_max - dma_lat_min + 1);
		if (dma_wait == 0) {
			uint32_t a = (uint32_t)top->dma_addr << 1;
			if (top->dma_we) {
				if (top->dma_be & 2) ram[a]     = top->dma_wdata >> 8;
				if (top->dma_be & 1) ram[a + 1] = top->dma_wdata & 0xff;
				dma_writes++;
			} else {
				top->dma_rdata = (ram[a] << 8) | ram[a + 1];
				dma_reads++;
			}
			top->dma_ack = 1;
			dma_ack_next = true;
			dma_wait = -1;
		} else dma_wait--;
	}
	if (top->audio_stb) {
		n_audio_stb++;
		if (capture_audio) audio.push_back({(int16_t)top->audio_l, (int16_t)top->audio_r});
	}
	int s = top->sndint, t = top->soundint;
	if (s != last_snd) {
		if (s) { snd_rise++; snd_rise_t.push_back(cyc); snd_high_at = cyc; }
		else   { snd_fall++; snd_fall_t.push_back(cyc); snd_low_at = cyc; }
		last_snd = s;
	}
	if (t != last_scnt) { if (t) scnt_rise++; else scnt_fall++; last_scnt = t; }

	// SSI.  DSP model DSP_FRAMEBUF: a pass-through program with one frame of
	// latency; the transmit side of a strobe uses the DSP's TX state before
	// the word received in the same strobe (TX first, then RX).
	if (top->ssi_slot_stb) {
		n_slot++;
		if (top->ssi_tx_en) {
			n_tx++;
			// what the DSP drives during the strobe clock (sampled at its end)
			if (dsp_mode == DSP_FRAMEBUF) {
				if (top->ssi_frame) { txi = 0; for (int i = 0; i < 8; i++) txcur[i] = txbuf[i]; }
				else txi++;
				top->ssi_tx_data = txcur[txi & 7];
			} else if (dsp_mode == DSP_COUNTER) {
				top->ssi_tx_data = tx_counter++;
			}
		}
		if (top->ssi_rx_en) {
			n_rx++;
			if (!top->ssi_tx_en) n_rx_only++;
			dsp_rx_log.push_back(top->ssi_rx_data);
			dsp_rxframe_log.push_back(top->ssi_rx_frame);
			if (dsp_mode == DSP_FRAMEBUF) {
				if (top->ssi_rx_frame) rxi = 0;
				if (rxi < 8) rxbuf[rxi] = top->ssi_rx_data;
				rxi++;
				if (rxi == dsp_slots) for (int i = 0; i < 8; i++) txbuf[i] = rxbuf[i];
			}
		}
	}
	if (dsp_mode == DSP_COUNTER && tx_valid_toggle) {
		// DSP has data every other slot of opportunity
		static uint64_t k = 0;
		k++;
		top->ssi_tx_valid = ((k / 97) & 1);
	}
}

static void run(uint64_t n) { for (uint64_t i = 0; i < n; i++) tick(); }

// ---------------------------------------------------------------- bus
static uint16_t bus_access(uint32_t addr, bool we, bool uds, bool lds, uint16_t din, bool *berr = nullptr)
{
	top->bus_cs = 1; top->bus_stb = 1; top->bus_we = we;
	top->bus_addr = (addr & 0x7f) >> 1;
	top->bus_uds = uds; top->bus_lds = lds; top->bus_din = din;
	int guard = 0;
	uint16_t d = 0;
	bool be = false;
	for (;;) {
		tick();
		top->bus_stb = 0;
		if (top->bus_ack) { d = top->bus_dout; be = top->bus_berr; break; }
		if (++guard > 100) { check(false, "bus access 0x%06x: no bus_ack", addr); break; }
	}
	top->bus_cs = 0; top->bus_we = 0; top->bus_uds = 0; top->bus_lds = 0;
	tick();
	if (berr) *berr = be;
	return d;
}
static void wb(uint32_t a, uint8_t v)
{
	if (a & 1) bus_access(a, true, false, true, v);
	else       bus_access(a, true, true, false, (uint16_t)v << 8);
}
static void ww(uint32_t a, uint16_t v) { bus_access(a, true, true, true, v); }
static uint8_t rb(uint32_t a)
{
	uint16_t d = (a & 1) ? bus_access(a, false, false, true, 0) : bus_access(a, false, true, false, 0);
	return (a & 1) ? (d & 0xff) : (d >> 8);
}
static uint16_t rw(uint32_t a, bool *berr = nullptr) { return bus_access(a, false, true, true, 0, berr); }

static void set_frame(uint32_t base_reg, uint32_t v)   // base_reg = 0x03 start / 0x0F end
{
	wb(0xff8900 + base_reg,     (v >> 16) & 0xff);
	wb(0xff8900 + base_reg + 2, (v >> 8) & 0xff);
	wb(0xff8900 + base_reg + 4, v & 0xff);
}
static uint32_t read_frame(uint32_t base_reg)
{
	uint32_t h = rb(0xff8900 + base_reg), m = rb(0xff8900 + base_reg + 2), l = rb(0xff8900 + base_reg + 4);
	return (h << 16) | (m << 8) | l;
}

static void do_reset()
{
	top->reset = 1;
	run(4);
	top->reset = 0;
	run(2);
	last_snd = top->sndint; last_scnt = top->soundint;
}

static void reset_monitors()
{
	audio.clear(); n_audio_stb = 0;
	snd_fall = snd_rise = scnt_fall = scnt_rise = 0;
	snd_rise_t.clear(); snd_fall_t.clear();
	n_slot = n_tx = n_rx = n_rx_only = 0;
	dsp_rx_log.clear(); dsp_rxframe_log.clear();
}

static bool wait_play_end(uint64_t max)
{
	for (uint64_t i = 0; i < max; i += 200) {
		run(200);
		if (!(rb(0xff8901) & 0x01)) return true;
	}
	return false;
}

static int16_t att(int v, int a) { return (int16_t)(((int32_t)v * dac_tab[a]) >> 16); }
static int16_t sat(int v) { return v > 32767 ? 32767 : (v < -32768 ? -32768 : v); }

// Find the expected pair sequence in the captured audio: each expected pair
// must appear exactly once, in order, on consecutive audio strobes.
static bool match_exact(const std::vector<Pair> &exp, const char *what)
{
	size_t s = 0;
	while (s < audio.size() && !(audio[s].l == exp[0].l && audio[s].r == exp[0].r)) s++;
	if (s == audio.size()) {
		check(false, "%s: first expected pair (%d,%d) never seen on audio_l/r (%zu samples)",
		      what, exp[0].l, exp[0].r, audio.size());
		return false;
	}
	for (size_t i = 0; i < exp.size(); i++) {
		if (s + i >= audio.size()) {
			check(false, "%s: output ended after %zu of %zu pairs", what, i, exp.size());
			return false;
		}
		const Pair &g = audio[s + i];
		if (g.l != exp[i].l || g.r != exp[i].r) {
			check(false, "%s: pair %zu expected (%d,%d) got (%d,%d)", what, i,
			      exp[i].l, exp[i].r, g.l, g.r);
			return false;
		}
	}
	check(true, "%s", what);
	info("%s: %zu pairs matched exactly (expected sequence, one per audio_stb)", what, exp.size());
	return true;
}

// Same as match_exact, but repeated samples (gaps) are allowed between pairs.
static bool match_dedup(const std::vector<Pair> &exp, const char *what)
{
	std::vector<Pair> d;
	for (auto &p : audio) {
		if (!d.empty() && d.back().l == p.l && d.back().r == p.r) continue;
		if (p.l == 0 && p.r == 0) continue;
		d.push_back(p);
	}
	size_t s = 0;
	while (s < d.size() && !(d[s].l == exp[0].l && d[s].r == exp[0].r)) s++;
	for (size_t i = 0; i < exp.size(); i++) {
		if (s + i >= d.size() || d[s + i].l != exp[i].l || d[s + i].r != exp[i].r) {
			check(false, "%s: dedup pair %zu expected (%d,%d) got (%d,%d)", what, i, exp[i].l, exp[i].r,
			      s + i < d.size() ? d[s + i].l : 0, s + i < d.size() ? d[s + i].r : 0);
			return false;
		}
	}
	check(true, "%s", what);
	info("%s: %zu pairs matched in order", what, exp.size());
	return true;
}

// common setup: DMA play at the 25 MHz clock -> DAC, adder = crossbar only, att 0
static void setup_dma_to_dac(uint8_t mode8921, uint8_t div)
{
	wb(0xff8935, div);
	wb(0xff8921, mode8921);
	ww(0xff8930, 0x0001);     // DMA play: 25 MHz, handshake off; DSP xmit tristated
	ww(0xff8932, 0x000D);     // DAC <- DMA play; DSP rec tristated; DMA rec <- ADC(10->ext) h/s off
	wb(0xff8937, 0x02);       // adder: crossbar only
	ww(0xff893a, 0x0000);     // no attenuation
}

static void begin_test(const char *name)
{
	printf("\n[%s]\n", name);
	test_fails = 0;
}
static std::vector<std::pair<std::string, bool>> results;
static void end_test(const char *name)
{
	check(!top->dbg_underrun, "%s: dbg_underrun must stay 0 (prefetch FIFO never ran dry)", name);
	results.push_back({name, test_fails == 0});
}

// ================================================================ tests
static void test_registers()
{
	begin_test("registers");
	do_reset();
	bool be;
	uint16_t v;
	// reset values (Crossbar_Reset)
	v = rw(0xff8900); check(v == 0x0500, "$FF8900.w after reset expected 0500 got %04x", v);
	v = rw(0xff8902); check(v == 0xFFFF, "$FF8902.w (void|start H) expected FFFF got %04x", v);
	v = rw(0xff8906); check(v == 0xFFFE, "$FF8906.w (void|start L) expected FFFE got %04x", v);
	uint32_t f = read_frame(0x03); check(f == 0xFFFFFE, "frame start after reset expected FFFFFE got %06x", f);
	f = read_frame(0x09);          check(f == 0xFFFFFE, "frame count after reset expected FFFFFE got %06x", f);
	f = read_frame(0x0F);          check(f == 0xFFFFFE, "frame end after reset expected FFFFFE got %06x", f);
	v = rw(0xff893c); check(v == 0x2401, "$FF893C after reset expected 2401 got %04x", v);
	v = rw(0xff8920); check(v == 0x0003, "$FF8920.w after reset expected 0003 got %04x", v);
	v = rw(0xff8936); check(v == 0x0003, "$FF8936.w after reset expected 0003 got %04x", v);
	v = rw(0xff8938); check(v == 0x0300, "$FF8938.w after reset expected 0300 got %04x", v);
	check(top->sndint == 1 && top->soundint == 1, "SNDINT/SOUNDINT high after reset (got %d/%d)", top->sndint, top->soundint);

	// read/write registers with full readback
	struct { uint32_t a; uint16_t w; } regs[] = {
		{0xff8930, 0x1234}, {0xff8932, 0xA5C3}, {0xff8934, 0x5A0F}, {0xff8936, 0x0302},
		{0xff8938, 0x02F1}, {0xff893a, 0x0AB0}, {0xff893c, 0x0003}, {0xff893e, 0xBEEF},
		{0xff8940, 0x0007}, {0xff8942, 0x0005}, {0xff8920, 0x2341} };
	for (auto &r : regs) {
		ww(r.a, r.w);
		v = rw(r.a, &be);
		check(v == r.w && !be, "$%06X write %04x read back %04x (berr %d)", r.a, r.w, v, be);
	}
	// byte lanes merge into word registers
	wb(0xff8931, 0x77); v = rw(0xff8930); check(v == 0x1277, "$FF8931.b merge expected 1277 got %04x", v);
	wb(0xff8932, 0x11); v = rw(0xff8932); check(v == 0x11C3, "$FF8932.b merge expected 11C3 got %04x", v);
	wb(0xff8935, 0x03); v = rw(0xff8934); check(v == 0x5A03, "$FF8935.b expected 5A03 got %04x", v);
	wb(0xff8900, 0x0F); v = rw(0xff8900); check(v == 0x0F00, "$FF8900.b expected 0F00 got %04x", v);
	ww(0xff8930, 0); ww(0xff8932, 0); wb(0xff8935, 0); ww(0xff8920, 0x0003);

	// frame start/end registers: play and record banks selected by $FF8901 bit 7
	set_frame(0x03, 0x123457);   // bit 0 is cleared
	set_frame(0x0F, 0x12ABCD);
	wb(0xff8901, 0x80);
	set_frame(0x03, 0x0A0B0C);
	set_frame(0x0F, 0x0D0E0F);
	f = read_frame(0x03); check(f == 0x0A0B0C, "record frame start expected 0A0B0C got %06x", f);
	f = read_frame(0x0F); check(f == 0x0D0E0E, "record frame end expected 0D0E0E got %06x", f);
	wb(0xff8901, 0x00);
	f = read_frame(0x03); check(f == 0x123456, "play frame start expected 123456 got %06x", f);
	f = read_frame(0x0F); check(f == 0x12ABCC, "play frame end expected 12ABCC got %06x", f);
	v = rw(0xff8900); check(v == 0x0F00, "$FF8901 select bit written back expected 0F00 got %04x", v);
	// IoMem shadow semantics: writing only the low byte composes with the shadow bytes
	wb(0xff8907, 0x80);
	f = read_frame(0x03); check(f == 0x123480, "start low byte write expected 123480 got %06x", f);
	// count writes do not change the count readback
	set_frame(0x09, 0x111111);
	f = read_frame(0x09); check(f == 0xFFFFFE, "count write ignored, readback expected FFFFFE got %06x", f);

	// microwire: $FF8922 reads 0, $FF8924 holds ~value for 8 CPU cycles
	v = rw(0xff8922); check(v == 0x0000, "$FF8922 expected 0000 got %04x", v);
	ww(0xff8924, 0x1234);
	v = rw(0xff8924); check(v == 0xEDCB, "$FF8924 right after write expected EDCB (NOT value) got %04x", v);
	run(20);
	v = rw(0xff8924); check(v == 0x1234, "$FF8924 after 8 CPU cycles expected 1234 got %04x", v);

	// bus errors in the holes, none on the void even bytes
	uint32_t berr_a[] = { 0xff8914, 0xff8918, 0xff891e, 0xff8926, 0xff892a, 0xff892e };
	for (uint32_t a : berr_a) { rw(a, &be); check(be, "$%06X expected bus error", a); }
	uint32_t ok_a[] = { 0xff8912, 0xff8922, 0xff8942, 0xff893e };
	for (uint32_t a : ok_a) { rw(a, &be); check(!be, "$%06X expected no bus error", a); }
	end_test("registers");
}

static void fill_words(uint32_t a, const std::vector<int16_t> &w)
{
	for (size_t i = 0; i < w.size(); i++) { ram[a + 2 * i] = (uint16_t)w[i] >> 8; ram[a + 2 * i + 1] = w[i] & 0xff; }
}

static void test_play16()
{
	begin_test("play 16-bit stereo, no loop");
	do_reset(); reset_monitors();
	const uint32_t A = 0x010000; const int N = 64;
	std::vector<int16_t> w; std::vector<Pair> exp;
	for (int i = 0; i < N; i++) {
		int16_t l = (int16_t)(1000 + i * 397), r = (int16_t)(-2000 - i * 311);
		w.push_back(l); w.push_back(r);
		exp.push_back({att(l, 0), att(r, 0)});
	}
	fill_words(A, w);
	setup_dma_to_dac(0x40, 1);          // 16 bit stereo, 49170 Hz
	wb(0xff8900, 0x05);
	set_frame(0x03, A); set_frame(0x0F, A + 4 * N);
	capture_audio = true;
	uint64_t t0 = cyc;
	wb(0xff8901, 0x01);
	check(snd_fall == 1 && scnt_fall == 1, "start: SNDINT and SOUNDINT fall once (got %d/%d)", snd_fall, scnt_fall);
	// frame count readback is monotonic within the frame while playing
	uint32_t prev = A; bool mono = true; int nread = 0;
	while (rb(0xff8901) & 1) {
		uint32_t c = read_frame(0x09);
		if (c < prev || c > A + 4 * N) mono = false;
		prev = c; nread++;
		run(300);
		if (cyc - t0 > 3000000) break;
	}
	uint64_t t1 = cyc;
	check(!(rb(0xff8901) & 1), "$FF8901 bit 0 cleared at the end of the frame");
	check(mono, "frame count readback monotonic within [start,end] (%d reads)", nread);
	uint32_t c = read_frame(0x09);
	check(c == A + 4 * N, "frame count after the end expected %06x got %06x", A + 4 * N, c);
	check(snd_rise == 1 && scnt_rise == 1, "end: SNDINT and SOUNDINT rise once (got %d/%d)", snd_rise, scnt_rise);
	double dur = (snd_rise_t.back() - snd_fall_t.back()) / CLK_HZ;
	double expdur = N / 49170.0;
	check(dur > expdur * 0.97 && dur < expdur * 1.03, "frame duration %.6f s expected %.6f s (N/49170)", dur, expdur);
	(void)t1;
	run(4000);
	match_exact(exp, "16-bit stereo DAC output");
	check(audio.back().l == 0 && audio.back().r == 0, "DAC output returns to 0 after the DMA stopped (got %d,%d)",
	      audio.back().l, audio.back().r);
	capture_audio = false;
	end_test("play 16-bit stereo, no loop");
}

static void test_play8(bool mono)
{
	const char *name = mono ? "play 8-bit mono" : "play 8-bit stereo";
	begin_test(name);
	do_reset(); reset_monitors();
	const uint32_t A = 0x020000; const int N = 80;   // samples (pairs)
	std::vector<Pair> exp;
	int nbytes = mono ? N : 2 * N;
	for (int i = 0; i < nbytes; i++) ram[A + i] = (uint8_t)(0x05 + i * 37 + (i & 1) * 0x40);
	for (int i = 0; i < N; i++) {
		int8_t l = mono ? (int8_t)ram[A + i] : (int8_t)ram[A + 2 * i];
		int8_t r = mono ? (int8_t)ram[A + i] : (int8_t)ram[A + 2 * i + 1];
		exp.push_back({att(l * 64, 0), att(r * 64, 0)});
	}
	// distinct consecutive pairs are needed for the exact match
	setup_dma_to_dac(mono ? 0x80 : 0x00, 2);         // 32780 Hz
	wb(0xff8900, 0x01);
	set_frame(0x03, A); set_frame(0x0F, A + nbytes);
	capture_audio = true;
	wb(0xff8901, 0x01);
	check(wait_play_end(4000000), "%s: play ends", name);
	run(3000);
	match_exact(exp, name);
	uint32_t c = read_frame(0x09);
	check(c == A + (uint32_t)nbytes, "frame count after the end expected %06x got %06x", A + nbytes, c);
	check(snd_fall == 1 && snd_rise == 1 && scnt_fall == 0 && scnt_rise == 0,
	      "$FF8900=01: SNDINT fall/rise 1/1, SOUNDINT none (got %d/%d %d/%d)", snd_fall, snd_rise, scnt_fall, scnt_rise);
	capture_audio = false;
	end_test(name);
}

static void test_loop(bool late)
{
	const char *name = late ? "play loop, next frame set late (FIFO refill)" : "play loop, double buffer";
	begin_test(name);
	do_reset(); reset_monitors(); dma_reads = 0;
	const uint32_t A = 0x030000, B = 0x038000; const int N = 48;
	std::vector<int16_t> wa, wbuf;
	std::vector<Pair> ea, eb;
	for (int i = 0; i < N; i++) {
		int16_t l = (int16_t)(100 + i * 211), r = (int16_t)(-100 - i * 173);
		wa.push_back(l); wa.push_back(r); ea.push_back({att(l, 0), att(r, 0)});
		int16_t l2 = (int16_t)(20000 - i * 101), r2 = (int16_t)(-20000 + i * 89);
		wbuf.push_back(l2); wbuf.push_back(r2); eb.push_back({att(l2, 0), att(r2, 0)});
	}
	fill_words(A, wa); fill_words(B, wbuf);
	setup_dma_to_dac(0x40, 1);
	wb(0xff8900, 0x01);
	set_frame(0x03, A); set_frame(0x0F, A + 4 * N);
	capture_audio = true;
	// frame sequence A, B, A, B: at each frame start (SNDINT falling), program the next buffer
	int frames = 0;
	uint32_t next = B;
	int last_fall = snd_fall;
	wb(0xff8901, 0x03);           // play + loop
	uint64_t tstart = cyc;
	while (frames < 3 && cyc - tstart < 6000000) {
		if (snd_fall != last_fall) {
			last_fall = snd_fall;
			frames++;
			if (late) {
				// wait until nearly the end of the frame (prefetch already ran into the next frame)
				run((uint64_t)(N * 0.9 * CLK_HZ / 49170.0));
			}
			set_frame(0x03, next); set_frame(0x0F, next + 4 * N);
			next = (next == A) ? B : A;
		}
		run(50);
	}
	// let frame 4 (B) play completely, stop in frame 5
	for (int k = 0; k < 2; k++) {
		last_fall = snd_fall;
		while (snd_fall == last_fall && cyc - tstart < 8000000) run(50);
	}
	run((uint64_t)(N * 0.5 * CLK_HZ / 49170.0));
	wb(0xff8901, 0x00);
	run(3000);
	std::vector<Pair> exp;
	for (int k = 0; k < 4; k++) for (auto &p : (k & 1) ? eb : ea) exp.push_back(p);
	{
		// words consumed: 4 full frames + the part of frame 5 before the stop
		info("DMA read accesses: %lu", (unsigned long)dma_reads);
		if (late) check(dma_reads > (uint64_t)(4 * 2 * N + 3 * 15), "late writes: prefetched words of the stale next frame were refetched (%lu reads)", (unsigned long)dma_reads);
	}
	if (late) match_dedup(exp, "loop A,B,A,B output (late register writes)");
	else      match_exact(exp, "loop A,B,A,B output (seamless)");
	// SNDINT: 1 fall at start + 1 per looped frame start; rises at every frame end + stop
	check(snd_rise == snd_fall && snd_rise >= 4, "SNDINT rises %d falls %d (expected equal, >= 4)", snd_rise, snd_fall);
	bool pulse_ok = true;
	for (size_t i = 0; i + 1 < snd_rise_t.size(); i++) {
		// each frame-end rise is followed by the re-arm fall LINE_PULSE clocks later
		uint64_t r = snd_rise_t[i];
		uint64_t f = 0;
		for (auto t : snd_fall_t) if (t > r) { f = t; break; }
		if (f == 0 || f - r != (uint64_t)LINE_PULSE) { pulse_ok = false;
			info("rise at %lu fall at %lu", (unsigned long)r, (unsigned long)f); }
	}
	check(pulse_ok, "looped frame end: SNDINT high pulse of exactly %d clocks", LINE_PULSE);
	check(scnt_fall == 0 && scnt_rise == 0, "SOUNDINT stays high with $FF8900 bit 2 clear (got %d/%d)", scnt_fall, scnt_rise);
	check(top->sndint == 1, "SNDINT high after stop");
	capture_audio = false;
	end_test(name);
}

static void test_rates()
{
	begin_test("sample rates");
	do_reset(); reset_monitors();
	const uint64_t W = 640000;   // 20 ms
	// STE compatible (prescale 0)
	for (int s = 0; s < 4; s++) {
		wb(0xff8935, 0); wb(0xff8921, s);
		run(1000);
		uint64_t n0 = n_audio_stb; run(W);
		double e = ste_rates[s] * (W / CLK_HZ);
		double g = (double)(n_audio_stb - n0);
		check(g >= e - 1.01 && g <= e + 1.01, "STE freq %d: audio_stb count %.0f expected %.2f (%d Hz)", s, g, e, ste_rates[s]);
	}
	info("STE rates 6258/12517/25033/50066 Hz checked");
	// 25.175 MHz dividers
	for (int d = 1; d < 16; d++) {
		wb(0xff8935, d);
		run(1000);
		uint64_t n0 = n_audio_stb; run(W);
		double e = f25[d - 1] * (W / CLK_HZ);
		double g = (double)(n_audio_stb - n0);
		check(g >= e - 1.01 && g <= e + 1.01, "div %d (25 MHz): audio_stb count %.0f expected %.2f (%d Hz)", d, g, e, f25[d - 1]);
	}
	info("25.175 MHz dividers 1..15 checked");
	// 32 MHz clock: DSP transmit at 32 MHz to the DAC, count slots
	dsp_mode = DSP_COUNTER;
	ww(0xff8930, 0x00D1);   // DSP xmit connected, 32 MHz, handshake off; DMA play 25 MHz
	ww(0xff8932, 0x2009);   // DAC <- DSP
	for (int tr = 0; tr < 4; tr += 3) {
		wb(0xff8920, tr);
		for (int d = 1; d < 16; d++) {
			wb(0xff8935, d);
			run(1000);
			uint64_t n0 = n_tx; run(W / 4);
			double e = f32[d - 1] * 2.0 * (tr + 1) * (W / 4 / CLK_HZ);
			double g = (double)(n_tx - n0);
			check(g >= e - 1.01 && g <= e + 1.01, "div %d (32 MHz) tracks %d: DSP slots %.0f expected %.2f", d, tr + 1, g, e);
		}
	}
	info("32 MHz dividers 1..15 checked with 1 and 4 tracks (slots = rate * 2 * tracks)");
	// 25 MHz slots with 4 tracks: audio rate unchanged, slot rate x8
	wb(0xff8920, 3);
	ww(0xff8930, 0x0091);   // DSP xmit at 25 MHz
	wb(0xff8935, 3);
	run(1000);
	{
		uint64_t a0 = n_audio_stb, s0 = n_tx; run(W / 2);
		double ea = f25[2] * (W / 2 / CLK_HZ), es = ea * 8;
		double ga = (double)(n_audio_stb - a0), gs = (double)(n_tx - s0);
		check(ga >= ea - 1.01 && ga <= ea + 1.01, "4 tracks: audio_stb %.0f expected %.2f", ga, ea);
		check(gs >= es - 1.01 && gs <= es + 1.01, "4 tracks: 25 MHz DSP slots %.0f expected %.2f", gs, es);
	}
	// external clock: never ticks
	ww(0xff8930, 0x00B1);
	run(1000);
	{ uint64_t s0 = n_tx; run(100000); check(n_tx == s0, "external clock: no DSP slots (got %lu)", (unsigned long)(n_tx - s0)); }
	dsp_mode = DSP_NONE;
	end_test("sample rates");
}

static void test_record()
{
	begin_test("record");
	// a) DMA play -> DMA record, 16 bit stereo, play and record started together
	do_reset(); reset_monitors();
	const uint32_t P = 0x040000, R = 0x050000; const int NW = 128;
	std::vector<int16_t> w;
	for (int i = 0; i < NW; i++) w.push_back((int16_t)(0x1357 * (i + 1)));
	fill_words(P, w);
	for (int i = 0; i < 2 * NW + 16; i++) ram[R + i] = 0xEE;
	wb(0xff8935, 1); wb(0xff8921, 0x40);
	ww(0xff8930, 0x0001);
	ww(0xff8932, 0x0001);   // DMA record <- DMA play (00), handshake off
	wb(0xff8900, 0x02);     // record -> GPIP7
	wb(0xff8901, 0x80); set_frame(0x03, R); set_frame(0x0F, R + 2 * NW);
	wb(0xff8901, 0x00); set_frame(0x03, P); set_frame(0x0F, P + 2 * NW);
	wb(0xff8901, 0x11);
	// play line update (bit 0 clear -> high) then record (bit 1 set -> low)
	check(top->sndint == 0, "play+record start: SNDINT low from the record channel");
	for (int i = 0; i < 400 && (rb(0xff8901) & 0x11); i++) run(500);
	run(500);
	uint8_t c1 = rb(0xff8901);
	check((c1 & 0x11) == 0, "$FF8901 play and record bits cleared at the end (got %02x)", c1);
	int bad = -1;
	for (int i = 0; i < NW; i++) {
		uint16_t g = (ram[R + 2 * i] << 8) | ram[R + 2 * i + 1];
		if (g != (uint16_t)w[i]) { bad = i; break; }
	}
	check(bad < 0, "DMA play -> record: %d words copied (first bad word %d)", NW, bad);
	check(ram[R + 2 * NW] == 0xEE, "record stops at frame end (byte after end untouched: %02x)", ram[R + 2 * NW]);
	wb(0xff8901, 0x80);
	uint32_t c = read_frame(0x09);
	check(c == R + 2 * NW, "record count readback (select bit 7) expected %06x got %06x", R + 2 * NW, c);
	wb(0xff8901, 0x00);
	check(snd_fall == 1 && snd_rise == 1, "record SNDINT fall/rise 1/1 (got %d/%d)", snd_fall, snd_rise);
	info("DMA play -> DMA record 16-bit: %d words", NW);

	// b) ADC -> DMA record, 8 bit mono (byte writes): left = PSG, right = microphone
	reset_monitors();
	const uint32_t R2 = 0x060001;   // odd start is rounded to even (bit 0 cleared)
	const int NB = 100;
	for (int i = 0; i < NB + 16; i++) ram[0x060000 + i] = 0xEE;
	top->psg_audio = 0x1234; top->mic_l = 0x7777; top->mic_r = 0x5678;
	wb(0xff8938, 0x02);           // left PSG, right microphone
	wb(0xff8921, 0x80);           // 8 bit mono
	ww(0xff8932, 0x0007);         // DMA record <- ADC
	wb(0xff8901, 0x80); set_frame(0x03, R2); set_frame(0x0F, 0x060000 + NB);
	wb(0xff8901, 0x90);
	for (int i = 0; i < 400 && (rb(0xff8901) & 0x10); i++) run(500);
	run(300);
	bool alt = true; int n34 = 0, n78 = 0;
	for (int i = 0; i < NB; i++) {
		uint8_t b = ram[0x060000 + i];
		if (b == 0x34) n34++; else if (b == 0x78) n78++; else alt = false;
		if (i > 0 && b == ram[0x060000 + i - 1]) alt = false;
	}
	check(alt && n34 == NB / 2 && n78 == NB / 2, "ADC mono record alternates PSG low byte 34 / mic_r low byte 78 (34:%d 78:%d)", n34, n78);
	check(ram[0x060000 + NB] == 0xEE, "mono record stops at frame end");
	top->psg_audio = 0; top->mic_l = 0; top->mic_r = 0;
	info("ADC -> DMA record 8-bit mono: %d bytes", NB);

	// c) DSP transmit -> DMA record, 16 bit, no handshake: DSP counter words
	do_reset(); reset_monitors();
	const uint32_t R3 = 0x070000; const int N3 = 64;
	dsp_mode = DSP_COUNTER; tx_counter = 0x4000;
	wb(0xff8935, 1); wb(0xff8921, 0x40);
	ww(0xff8930, 0x0091);         // DSP xmit 25 MHz, handshake off
	ww(0xff8932, 0x0003);         // DMA record <- DSP xmit, handshake off
	wb(0xff8901, 0x80); set_frame(0x03, R3); set_frame(0x0F, R3 + 2 * N3);
	wb(0xff8901, 0x90);
	for (int i = 0; i < 400 && (rb(0xff8901) & 0x10); i++) run(500);
	run(300);
	uint16_t first = (ram[R3] << 8) | ram[R3 + 1];
	bool seq = true;
	for (int i = 1; i < N3; i++) {
		uint16_t g = (ram[R3 + 2 * i] << 8) | ram[R3 + 2 * i + 1];
		if (g != (uint16_t)(first + i)) seq = false;
	}
	check(seq && first >= 0x4000, "DSP -> record: %d consecutive DSP words from %04x", N3, first);

	// d) DSP transmit -> DMA record handshake mode: only slots where the DSP has data
	reset_monitors();
	const uint32_t R4 = 0x078000; const int N4 = 48;
	tx_counter = 0x6000; tx_valid_toggle = true;
	ww(0xff8932, 0x0002);         // DMA record <- DSP xmit, handshake ON (bits 3:0 = 0010)
	wb(0xff8901, 0x80); set_frame(0x03, R4); set_frame(0x0F, R4 + 2 * N4);
	uint64_t tx0 = n_tx;
	wb(0xff8901, 0x90);
	for (int i = 0; i < 800 && (rb(0xff8901) & 0x10); i++) run(500);
	run(300);
	first = (ram[R4] << 8) | ram[R4 + 1];
	seq = true;
	for (int i = 1; i < N4; i++) {
		uint16_t g = (ram[R4 + 2 * i] << 8) | ram[R4 + 2 * i + 1];
		if (g != (uint16_t)(first + i)) seq = false;
	}
	check(seq && first >= 0x6000, "DSP handshake record: %d consecutive words from %04x", N4, first);
	check(n_tx - tx0 == (uint64_t)N4, "handshake: one DSP transmit slot per recorded word (%lu slots, %d words)",
	      (unsigned long)(n_tx - tx0), N4);
	tx_valid_toggle = false; top->ssi_tx_valid = 0;
	dsp_mode = DSP_NONE;
	end_test("record");
}

static void test_dsp_loop()
{
	begin_test("DMA -> DSP -> DAC");
	do_reset(); reset_monitors();
	const uint32_t A = 0x080000; const int N = 64;
	std::vector<int16_t> w; std::vector<Pair> exp;
	for (int i = 0; i < N; i++) {
		int16_t l = (int16_t)(3000 + 251 * i), r = (int16_t)(-3000 - 199 * i);
		w.push_back(l); w.push_back(r); exp.push_back({att(l, 3), att(r, 3)});
	}
	fill_words(A, w);
	dsp_mode = DSP_FRAMEBUF; dsp_slots = 2;
	wb(0xff8935, 1); wb(0xff8921, 0x40);
	ww(0xff8930, 0x0091);   // DSP xmit connected, 25 MHz, no handshake; DMA play 25 MHz
	ww(0xff8932, 0x2095);   // DAC <- DSP xmit; DSP rec <- DMA play (connected, no h/s); DMA rec <- ext
	wb(0xff8937, 0x02);
	ww(0xff893a, 0x0330);   // attenuation 3 on both channels
	set_frame(0x03, A); set_frame(0x0F, A + 4 * N);
	capture_audio = true;
	wb(0xff8901, 0x01);
	check(wait_play_end(4000000), "play ends");
	run(3000);
	// what the DSP received: the DMA words in order, frame flag on the left words
	size_t s = 0;
	while (s < dsp_rx_log.size() && dsp_rx_log[s] != (uint16_t)w[0]) s++;
	bool rxok = s + 2 * N <= dsp_rx_log.size();
	for (int i = 0; rxok && i < 2 * N; i++)
		if (dsp_rx_log[s + i] != (uint16_t)w[i] || dsp_rxframe_log[s + i] != ((i & 1) == 0)) rxok = false;
	check(rxok, "DSP receive: %d DMA words in order with ssi_rx_frame on each left word", 2 * N);
	check(n_rx_only == 0 && n_rx > 0, "every DSP receive slot is also a DSP transmit slot (rx %lu, rx without tx %lu)",
	      (unsigned long)n_rx, (unsigned long)n_rx_only);
	if (!test_fails) {} else for (size_t i = 0; i < 12 && i < audio.size(); i++) info("audio[%zu] = %d,%d", i, audio[i].l, audio[i].r);
	if (!match_exact(exp, "DAC output through the DSP (attenuation 3)"))
		for (size_t i = 0; i < 12 && i < audio.size(); i++)
			info("audio[%zu] = %d,%d (expected[%zu] %d,%d)", i, audio[i].l, audio[i].r, i, exp[i].l, exp[i].r);
	capture_audio = false;

	// 4 tracks, the DAC monitors track 2 (DMA play straight to the DAC) and the DSP sees 8-slot frames
	reset_monitors();
	const uint32_t B = 0x090000; const int F = 40;
	std::vector<int16_t> w4; std::vector<Pair> e4;
	for (int i = 0; i < F; i++) for (int k = 0; k < 8; k++) w4.push_back((int16_t)(i * 512 + k * 37 + 1 - (k & 1) * 20000));
	for (int i = 0; i < F; i++) e4.push_back({att(w4[i * 8 + 4], 0), att(w4[i * 8 + 5], 0)});
	fill_words(B, w4);
	dsp_mode = DSP_NONE;
	wb(0xff8920, 0x23);     // 4 play tracks, monitor track 2
	ww(0xff8930, 0x0001);
	ww(0xff8932, 0x0095);   // DAC <- DMA; DSP rec <- DMA
	ww(0xff893a, 0x0000);
	set_frame(0x03, B); set_frame(0x0F, B + 16 * F);
	capture_audio = true;
	wb(0xff8901, 0x01);
	check(wait_play_end(4000000), "4-track play ends");
	run(3000);
	match_exact(e4, "4 tracks, monitored track 2 on the DAC");
	bool fr = dsp_rx_log.size() >= (size_t)(8 * F);
	for (size_t i = 0; fr && i < (size_t)(8 * F); i++)
		if (dsp_rxframe_log[i] != ((i % 8) == 0) || dsp_rx_log[i] != (uint16_t)w4[i]) fr = false;
	check(fr, "4 tracks: DSP receives %d words, ssi_rx_frame every 8th slot", 8 * F);
	capture_audio = false;
	end_test("DMA -> DSP -> DAC");
}

static void test_codec()
{
	begin_test("codec gain / adder / attenuation");
	do_reset(); reset_monitors();
	// constant 16-bit sample in a looped buffer
	const uint32_t A = 0x0A0000;
	std::vector<int16_t> w;
	for (int i = 0; i < 32; i++) { w.push_back(20000); w.push_back(-12345); }
	fill_words(A, w);
	setup_dma_to_dac(0x40, 1);
	set_frame(0x03, A); set_frame(0x0F, A + 128);
	wb(0xff8901, 0x03);
	auto settle = [](){ run(3000); };
	for (int a = 0; a < 16; a++) {
		ww(0xff893a, (uint16_t)((a << 8) | ((15 - a) << 4)));
		settle();
		int16_t el = att(20000, a), er = att(-12345, 15 - a);
		check((int16_t)top->audio_l == el && (int16_t)top->audio_r == er,
		      "attenuation L=%d R=%d: expected (%d,%d) got (%d,%d)", a, 15 - a, el, er,
		      (int16_t)top->audio_l, (int16_t)top->audio_r);
	}
	info("attenuation table checked (16 steps per channel)");
	ww(0xff893a, 0);
	// ADC direct path: PSG on both channels with gain
	top->psg_audio = 8000;
	wb(0xff8937, 0x01); wb(0xff8938, 0x03);
	for (int g = 0; g < 16; g++) {
		wb(0xff8939, (uint8_t)((g << 4) | (15 - g)));
		settle();
		int16_t el = att(sat((8000 * adc_tab[g]) >> 14), 0), er = att(sat((8000 * adc_tab[15 - g]) >> 14), 0);
		check((int16_t)top->audio_l == el && (int16_t)top->audio_r == er,
		      "ADC gain L=%d R=%d (PSG 8000): expected (%d,%d) got (%d,%d)", g, 15 - g, el, er,
		      (int16_t)top->audio_l, (int16_t)top->audio_r);
	}
	info("ADC gain table checked (PSG through the adder)");
	// microphone inputs
	top->mic_l = -5000; top->mic_r = 3000;
	wb(0xff8938, 0x00); wb(0xff8939, 0x55);
	settle();
	{
		int16_t el = att(sat((-5000 * adc_tab[5]) >> 14), 0), er = att(sat((3000 * adc_tab[5]) >> 14), 0);
		check((int16_t)top->audio_l == el && (int16_t)top->audio_r == er,
		      "microphone direct: expected (%d,%d) got (%d,%d)", el, er, (int16_t)top->audio_l, (int16_t)top->audio_r);
	}
	// mixed: ADC (left PSG, right mic) + crossbar
	wb(0xff8938, 0x02); wb(0xff8937, 0x03); wb(0xff8939, 0x30);
	settle();
	{
		int16_t el = att(sat(((8000 * adc_tab[3]) >> 14) + 20000), 0);
		int16_t er = att(sat(((3000 * adc_tab[0]) >> 14) - 12345), 0);
		check((int16_t)top->audio_l == el && (int16_t)top->audio_r == er,
		      "adder ADC+DAC: expected (%d,%d) got (%d,%d)", el, er, (int16_t)top->audio_l, (int16_t)top->audio_r);
	}
	// saturation
	top->psg_audio = 32000; wb(0xff8938, 0x03); wb(0xff8939, 0xFF);
	settle();
	{
		int16_t el = att(32767, 0);
		int16_t er = att(sat(sat((32000 * 43691) >> 14) - 12345), 0);
		check((int16_t)top->audio_l == el && (int16_t)top->audio_r == er,
		      "adder saturation: expected (%d,%d) got (%d,%d)", el, er, (int16_t)top->audio_l, (int16_t)top->audio_r);
	}
	// adder source 0: silence
	wb(0xff8937, 0x00); settle();
	check(top->audio_l == 0 && top->audio_r == 0, "adder input 0: silence");
	// muted dividers (Crossbar_Recalculate_Clocks_Cycles)
	wb(0xff8937, 0x01); wb(0xff8939, 0x88); top->psg_audio = 8000;
	for (int d = 0; d < 16; d++) {
		wb(0xff8935, d);
		run((uint64_t)(4 * CLK_HZ / 6000));
		// $FF8921 = $40 here: STE frequency bits 0, so prescale 0 is muted too
		bool muted = (d == 0 || d == 6 || d == 8 || d == 10 || d >= 12);
		bool zero = top->audio_l == 0 && top->audio_r == 0;
		check(zero == muted, "div %d: DAC %s (out %d)", d, muted ? "muted" : "running", (int16_t)top->audio_l);
	}
	wb(0xff8935, 0); wb(0xff8921, 0x00); run(30000);
	check(top->audio_l == 0, "STE 6258 Hz: DAC muted");
	wb(0xff8921, 0x01); run(30000);
	check(top->audio_l != 0, "STE 12517 Hz: DAC running");
	wb(0xff8901, 0x00);
	top->psg_audio = 0; top->mic_l = 0; top->mic_r = 0;
	end_test("codec gain / adder / attenuation");
}

static void test_lines()
{
	begin_test("SNDINT / SOUNDINT");
	const uint32_t A = 0x0B0000; const int N = 16;
	std::vector<int16_t> w;
	for (int i = 0; i < 2 * N; i++) w.push_back((int16_t)(i + 1));
	fill_words(A, w);
	uint8_t cfg[] = { 0x00, 0x01, 0x04, 0x05, 0x0A };
	for (uint8_t c : cfg) {
		do_reset(); reset_monitors();
		setup_dma_to_dac(0x40, 1);
		wb(0xff8900, c);
		set_frame(0x03, A); set_frame(0x0F, A + 4 * N);
		wb(0xff8901, 0x01);
		int low_snd = top->sndint == 0, low_scnt = top->soundint == 0;
		check(wait_play_end(2000000), "$FF8900=%02x: play ends", c);
		int eg = (c & 1) ? 1 : 0, et = (c & 4) ? 1 : 0;
		check(snd_fall == eg && snd_rise == eg && scnt_fall == et && scnt_rise == et,
		      "$FF8900=%02x: SNDINT fall/rise expected %d/%d got %d/%d, SOUNDINT expected %d/%d got %d/%d",
		      c, eg, eg, snd_fall, snd_rise, et, et, scnt_fall, scnt_rise);
		check(low_snd == eg && low_scnt == et, "$FF8900=%02x: lines low while playing (SNDINT %d SOUNDINT %d)", c, low_snd, low_scnt);
		check(top->sndint && top->soundint, "$FF8900=%02x: lines high when idle", c);
	}
	// stop by write while playing: rising edge on both
	do_reset(); reset_monitors();
	setup_dma_to_dac(0x40, 1);
	wb(0xff8900, 0x05);
	set_frame(0x03, A); set_frame(0x0F, A + 4 * N);
	wb(0xff8901, 0x03);
	run(20000);
	wb(0xff8901, 0x00);
	check(top->sndint && top->soundint && snd_rise >= 1 && scnt_rise >= 1, "stop by write: both lines high");
	end_test("SNDINT / SOUNDINT");
}

static void test_stress()
{
	begin_test("FIFO stress at the maximum slot rate");
	// 16-bit, 4 tracks, 32 MHz clock divider 1: 500000 words/s (one word per 64 clocks)
	const uint32_t P = 0x0C0000, R = 0x0D0000; const int NW = 2048;
	std::vector<int16_t> w;
	for (int i = 0; i < NW; i++) w.push_back((int16_t)(i * 7919 + 13));
	fill_words(P, w);
	// a) play only, DMA latency 40..60 clocks
	do_reset(); reset_monitors();
	dma_lat_min = 40; dma_lat_max = 60;
	wb(0xff8935, 1); wb(0xff8921, 0x40); wb(0xff8920, 0x03);
	ww(0xff8930, 0x0005);   // DMA play 32 MHz, handshake off
	ww(0xff8932, 0x000D);
	set_frame(0x03, P); set_frame(0x0F, P + 2 * NW);
	uint64_t t0 = cyc;
	wb(0xff8901, 0x01);
	check(wait_play_end(4000000), "play ends");
	double dur = (cyc - t0) / CLK_HZ, e = NW / 500000.0;
	check(dur > e * 0.98 && dur < e * 1.05, "%d words played in %.6f s, expected %.6f s at 500000 words/s", NW, dur, e);
	check(!top->dbg_underrun, "latency 40..60: no underrun at 500000 words/s");
	// b) play -> record at the same time, latency 6..28 (two accesses per slot)
	do_reset(); reset_monitors();
	dma_lat_min = 6; dma_lat_max = 28;
	for (int i = 0; i < 2 * NW; i++) ram[R + i] = 0;
	wb(0xff8935, 1); wb(0xff8921, 0x40); wb(0xff8920, 0x03);
	ww(0xff8930, 0x0005);
	ww(0xff8932, 0x0001);   // DMA record <- DMA play
	wb(0xff8901, 0x80); set_frame(0x03, R); set_frame(0x0F, R + 2 * NW);
	wb(0xff8901, 0x00); set_frame(0x03, P); set_frame(0x0F, P + 2 * NW);
	wb(0xff8901, 0x11);
	for (int i = 0; i < 400 && (rb(0xff8901) & 0x11); i++) run(500);
	run(500);
	int bad = -1;
	for (int i = 0; i < NW; i++)
		if (((ram[R + 2 * i] << 8) | ram[R + 2 * i + 1]) != (uint16_t)w[i]) { bad = i; break; }
	check(bad < 0, "play -> record at 500000 words/s: %d words copied (first bad %d)", NW, bad);
	dma_lat_min = 6; dma_lat_max = 20;
	end_test("FIFO stress at the maximum slot rate");
}

static void hs_req()
{
	top->ssi_hs_play_req = 1;
	tick();
	top->ssi_hs_play_req = 0;
}

static void test_handshake()
{
	begin_test("DMA play -> DSP handshake, DSP master clock");
	const uint32_t P = 0x0E0000, R = 0x0F0000; const int NW = 64;
	std::vector<int16_t> w;
	for (int i = 0; i < NW; i++) w.push_back((int16_t)(0x1234 + i * 0x2E5F));
	fill_words(P, w);

	// a) 25 MHz: one word per DSP request, then free running after $FF8932 is rewritten
	do_reset(); reset_monitors();
	wb(0xff8935, 1); wb(0xff8921, 0x40);
	ww(0xff8930, 0x0000);   // DMA play 25 MHz, handshake on
	ww(0xff8932, 0x0081);   // DSP rec <- DMA play, connected, handshake on (bits 6:4 = 000)
	wb(0xff8937, 0x02);
	set_frame(0x03, P); set_frame(0x0F, P + 2 * NW);
	hs_req();
	wb(0xff8901, 0x01);
	const int K = 20;
	bool one_each = true;
	for (int k = 0; k < K; k++) {
		run(3000);
		if (n_rx != (uint64_t)(k + 1)) { one_each = false; info("after request %d: %lu words", k, (unsigned long)n_rx); }
		hs_req();
	}
	run(3000);
	check(one_each && n_rx == (uint64_t)(K + 1), "one DSP receive word per request (%lu words for %d requests)", (unsigned long)n_rx, K + 1);
	uint32_t c0 = read_frame(0x09);
	run(20000);
	uint32_t c1 = read_frame(0x09);
	check(n_rx == (uint64_t)(K + 1) && c0 == c1 && c0 == P + 2 * (K + 1),
	      "no request: no transfer (words %lu, count %06x -> %06x, expected %06x)", (unsigned long)n_rx, c0, c1, P + 2 * (K + 1));
	bool ok = dsp_rx_log.size() >= (size_t)(K + 1);
	for (int i = 0; ok && i < K + 1; i++)
		if (dsp_rx_log[i] != (uint16_t)w[i] || dsp_rxframe_log[i] != 0) ok = false;
	check(ok, "25 MHz handshake: words unshifted, in order, ssi_rx_frame 0 (SC1 not sent)");
	check(rb(0xff8901) & 1, "play still running while waiting for the DSP");
	// $FF8932 write clears the DSP master clock: the play runs freely to the end
	ww(0xff8932, 0x0081);
	check(wait_play_end(2000000), "free running play ends after $FF8932 rewrite");
	ok = dsp_rx_log.size() == (size_t)NW;
	for (int i = K + 1; ok && i < NW; i++) {
		int ef = (i == K + 1) ? 0 : ((i % 2) == 0);
		if (dsp_rx_log[i] != (uint16_t)w[i] || dsp_rxframe_log[i] != ef) {
			ok = false; info("word %d: %04x frame %d, expected %04x frame %d", i, dsp_rx_log[i], dsp_rxframe_log[i], (uint16_t)w[i], ef);
		}
	}
	check(ok, "after rewrite: remaining %d words, first without SC1, then frame on left words (got %zu words total)",
	      NW - K - 1, dsp_rx_log.size());

	// b) 32 MHz: nocrew 2-bit shift, to the DSP and to DMA record
	do_reset(); reset_monitors();
	const int M = 24;
	for (int i = 0; i < 2 * M + 8; i++) ram[R + i] = 0xEE;
	wb(0xff8935, 1); wb(0xff8921, 0x40);
	ww(0xff8930, 0x0004);   // DMA play 32 MHz, handshake on
	ww(0xff8932, 0x0081);   // DSP rec <- DMA play h/s; DMA rec <- DMA play
	wb(0xff8901, 0x80); set_frame(0x03, R); set_frame(0x0F, R + 2 * M);
	wb(0xff8901, 0x00); set_frame(0x03, P); set_frame(0x0F, P + 2 * NW);
	hs_req();
	wb(0xff8901, 0x11);
	for (int k = 1; k < M; k++) { run(2000); hs_req(); }
	run(3000);
	std::vector<uint16_t> e;
	uint16_t save = 0;
	for (int i = 0; i < M; i++) {
		e.push_back((uint16_t)((save << 2) | ((uint16_t)w[i] >> 14)));
		save = (uint16_t)w[i];
	}
	ok = dsp_rx_log.size() == (size_t)M;
	for (int i = 0; ok && i < M; i++)
		if (dsp_rx_log[i] != e[i]) { ok = false; info("word %d: got %04x expected %04x", i, dsp_rx_log[i], e[i]); }
	check(ok, "32 MHz handshake: %d words = ((prev << 2) | (word >> 14)) & FFFF (got %zu words)", M, dsp_rx_log.size());
	ok = true;
	for (int i = 0; i < M; i++)
		if (((ram[R + 2 * i] << 8) | ram[R + 2 * i + 1]) != e[i]) { ok = false; break; }
	check(ok && !(rb(0xff8901) & 0x10), "32 MHz handshake: DMA record got the shifted words and stopped at its frame end");
	check(ram[R + 2 * M] == 0xEE, "record stops at frame end");
	wb(0xff8901, 0x00);

	// c) 32 MHz, handshake bits set but no DSP request: free running, no shift
	do_reset(); reset_monitors();
	wb(0xff8935, 1); wb(0xff8921, 0x40);
	ww(0xff8930, 0x0004);
	ww(0xff8932, 0x0081);
	set_frame(0x03, P); set_frame(0x0F, P + 2 * NW);
	wb(0xff8901, 0x01);
	check(wait_play_end(2000000), "free running 32 MHz play ends");
	ok = dsp_rx_log.size() == (size_t)NW;
	for (int i = 0; ok && i < NW; i++) {
		int ef = (i == 0) ? 0 : ((i % 2) == 0);
		if (dsp_rx_log[i] != (uint16_t)w[i] || dsp_rxframe_log[i] != ef) ok = false;
	}
	check(ok, "no DSP request: %d raw words, first without SC1 (handshakeMode_Frame from $FF8932), then frames (got %zu)",
	      NW, dsp_rx_log.size());
	end_test("DMA play -> DSP handshake, DSP master clock");
}

int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	srand(12345);
	top = new Vfalcon_crossbar;
	top->clk = 0; top->reset = 1;
	top->bus_cs = 0; top->bus_stb = 0; top->bus_we = 0;
	top->dma_ack = 0; top->dma_rdata = 0;
	top->psg_audio = 0; top->mic_l = 0; top->mic_r = 0;
	top->ssi_tx_data = 0; top->ssi_tx_valid = 0; top->ssi_hs_play_req = 0;
	top->eval();

	test_registers();
	test_play16();
	test_play8(false);
	test_play8(true);
	test_loop(false);
	test_loop(true);
	test_rates();
	test_record();
	test_dsp_loop();
	test_codec();
	test_lines();
	test_stress();
	test_handshake();

	printf("\n==== crossbar test summary ====\n");
	for (auto &r : results) printf("  %-48s %s\n", r.first.c_str(), r.second ? "PASS" : "FAIL");
	printf("checks: %d, failed: %d\n", checks, fails);
	printf("%s\n", fails ? "RESULT: FAIL" : "RESULT: PASS");
	top->final();
	delete top;
	return fails ? 1 : 0;
}
