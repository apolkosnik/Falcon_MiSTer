// Verilator testbench for falcon_videl (the real RTL module).
//
// Register values for the TOS modes:
//   Timing (HHT..VSS) - the Videl mode tables of TOS 4.04 as carried by
//   EmuTOS 1.3 (bios/videl.c tables, read out of ref/emutos-512k-1.3/
//   etos512us.img at offset 0x120d6: 13 words per mode {mode, HHT, HBB, HBE,
//   HDB, HDE, HSS, VFT, VBB, VBE, VDB, VDE, VSS}, VGA table then the 50 Hz
//   RGB table).
//   VCO ($82C0), VMD ($82C2), $8210, $8266, $8260 - register dumps taken
//   with Hatari (Falcon, EmuTOS 1.3) after XBIOS Vsetmode(mode) for each mode
//   on a VGA, RGB and mono monitor (a small 68k program reading
//   $FF8200-$FF82C3).
//
// Expected timings come from the video standards the modes implement
// (VGA 640x480: 800 pixel clocks per line at 25.175 MHz, 525 lines;
// RGB: 64 us lines = 2048 clocks at 32 MHz, PAL 625 half-lines per field
// interlaced / 626 non-interlaced), and from the register semantics
// documented in Hatari videl.c (VC registers count half lines, VMD bit 0
// doubles lines, bit 1 interlaces).
//
// Pixel colours are checked against a framebuffer pattern in a RAM model
// behind the fetch port, decoded here as Hatari's ConvGen does (planar
// interleaved words, RRRRRGGGGGGBBBBB true colour, hscroll from $8265,
// stride + bpp words when hscroll is used) with the Falcon / STe palettes
// expanded as in VIDEL_UpdateColors.

#include <cstdio>
#include <cstdarg>
#include <algorithm>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <vector>
#include <deque>
#include <string>
#include "Vfalcon_videl.h"
#include "verilated.h"

static Vfalcon_videl *top;
static uint64_t cycles = 0;
static int n_pass = 0, n_fail = 0;

static void check(bool ok, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void check(bool ok, const char *fmt, ...)
{
	va_list ap;
	va_start(ap, fmt);
	printf("  [%s] ", ok ? "PASS" : "FAIL");
	vprintf(fmt, ap);
	printf("\n");
	va_end(ap);
	if (ok) n_pass++; else n_fail++;
}

//---------------------------------------------------------------------------
// RAM model behind the video fetch port
//---------------------------------------------------------------------------
static const uint32_t RAM_SIZE = 4u << 20;
static std::vector<uint8_t> ram(RAM_SIZE);

struct MemModel {
	int ack_delay_max = 3;      // clocks before a request is acknowledged
	int lat_min = 6, lat_max = 36; // clocks before the first beat of a burst
	bool gaps = true;           // random idle clocks between beats
	bool fixed_worst = false;   // every burst takes exactly 40 clocks
	int ack_wait = -1;
	std::deque<uint32_t> q;     // queued burst byte addresses
	int serve_wait = -1;
	int beat = 0;
	uint32_t cur = 0;
	bool busy = false;
	uint64_t bursts = 0;
	void reset() { ack_wait = -1; q.clear(); serve_wait = -1; beat = 0; busy = false; bursts = 0; }
	static uint64_t qword(uint32_t a) {
		uint64_t v = 0;
		for (int i = 0; i < 8; i++) v = (v << 8) | ram[(a + i) % RAM_SIZE];
		return v;
	}
	// called after the rising edge with the module outputs; sets the inputs
	// for the next rising edge
	void step() {
		top->vid_ack = 0;
		top->vid_valid = 0;
		if (top->vid_req) {
			if (ack_wait < 0) ack_wait = fixed_worst ? 0 : (rand() % (ack_delay_max + 1));
			if (ack_wait == 0) {
				top->vid_ack = 1;
				q.push_back((uint32_t)top->vid_addr << 3);
				bursts++;
				ack_wait = -1;
			} else ack_wait--;
		}
		if (!busy && !q.empty()) {
			busy = true;
			cur = q.front(); q.pop_front();
			serve_wait = fixed_worst ? 36 : lat_min + rand() % (lat_max - lat_min + 1);
			beat = 0;
		}
		if (busy) {
			if (serve_wait > 0) serve_wait--;
			else if (gaps && !fixed_worst && (rand() % 4) == 0) { /* idle beat */ }
			else {
				top->vid_valid = 1;
				top->vid_data = qword(cur + beat * 8);
				if (++beat == 4) busy = false;
			}
		}
	}
} mem;

//---------------------------------------------------------------------------
// Output monitor
//---------------------------------------------------------------------------
struct LineRec {
	uint64_t start, clocks;
	int ce, de_px, vis_px, hbl;
	bool vblank_at_start;
	std::vector<uint32_t> px;   // rgb of de pixels
};
struct FrameRec {
	uint64_t start, clocks;
	int hsyncs, de_lines, vis_lines, vbl, hbl, field, underruns;
	int tb_falls, detb_diff;
	uint64_t first_de_rise, last_de_fall;
	std::vector<uint64_t> hbl_t, tbf_t;   // hbl pulses, de_tb falling edges
	std::vector<int> run_pos;            // pixel clocks from hsync start to each DE run
	int max_de, min_de, max_vis, min_vis;
	int vruns, vrun_min, vrun_max;      // visible (non blank) runs
	int border_err, blank_err, border_n;
	uint64_t hs_clk; int hs_ce;         // last hsync pulse width
	uint64_t vs_clk;                    // vsync pulse width
	std::vector<std::vector<uint32_t>> img;
	std::vector<std::vector<uint8_t>> msk;   // 1 = not blanked, per DE pixel
	int vis_de_lines;                       // DE runs with at least one visible pixel
};

struct Monitor {
	bool hs_pol = false, vs_pol = false;   // active level
	bool prev_hs = false, prev_vs = false, prev_de = false, prev_detb = false;
	bool in_line = false, in_frame = false;
	LineRec ln;
	FrameRec fr;
	std::vector<FrameRec> frames;
	bool capture = false;
	int de_rises = 0;
	bool in_run = false;
	std::vector<uint32_t> run;
	std::vector<uint8_t> runv;
	uint32_t border_rgb = 0;
	bool in_vrun = false; int vrun = 0;
	uint64_t hs_t0 = 0, vs_t0 = 0; int hs_ce0 = 0; bool hs_on = false, vs_on = false;
	void end_vrun() {
		if (!in_vrun) return;
		in_vrun = false;
		if (!in_frame) return;
		fr.vruns++;
		fr.vrun_min = std::min(fr.vrun_min, vrun);
		fr.vrun_max = std::max(fr.vrun_max, vrun);
	}
	void end_run() {
		if (!in_run) return;
		in_run = false;
		if (!in_frame) return;
		fr.de_lines++;
		int n = run.size();
		fr.max_de = std::max(fr.max_de, n);
		fr.min_de = std::min(fr.min_de, n);
		bool any = false;
		for (uint8_t v : runv) if (v) any = true;
		if (any) fr.vis_de_lines++;
		if (capture) { fr.img.push_back(run); fr.msk.push_back(runv); }
	}
	void reset() {
		prev_hs = prev_vs = prev_de = false; in_line = in_frame = false; in_run = false; in_vrun = false;
		hs_on = vs_on = false;
		frames.clear(); capture = false; de_rises = 0;
	}
	void end_line() {
		if (!in_line) return;
		ln.clocks = cycles - ln.start;
		if (in_frame) {
			fr.hsyncs++;
			if (ln.vis_px) {
				fr.vis_lines++;
				fr.max_vis = std::max(fr.max_vis, ln.vis_px);
				fr.min_vis = std::min(fr.min_vis, ln.vis_px);
			}
		}
	}
	void sample() {
		bool hs = (top->hsync != 0) == hs_pol;
		bool vs = (top->vsync != 0) == vs_pol;
		if (vs && !prev_vs) {
			if (in_frame) {
				end_line();
				in_line = false;
				fr.clocks = cycles - fr.start;
				frames.push_back(fr);
			}
			in_frame = true;
			fr = FrameRec();
			fr.start = cycles;
			fr.min_de = fr.min_vis = fr.vrun_min = 1 << 30;
			fr.max_de = fr.max_vis = 0;
			fr.field = -1;
		}
		if (hs && !prev_hs) {
			end_line();
			in_line = true;
			ln = LineRec();
			ln.start = cycles;
			ln.vblank_at_start = top->vblank;
		}
		// sync pulse widths
		if (hs && !prev_hs) { hs_t0 = cycles; hs_ce0 = 0; hs_on = true; }
		if (hs && top->ce_pix) hs_ce0++;
		if (!hs && prev_hs && hs_on && in_frame) { fr.hs_clk = cycles - hs_t0; fr.hs_ce = hs_ce0; }
		if (vs && !prev_vs) { vs_t0 = cycles; vs_on = true; }
		if (!vs && prev_vs && vs_on && in_frame) fr.vs_clk = cycles - vs_t0;
		// visible runs, border colour and blanking level
		if (top->ce_pix) {
			uint32_t rgb = ((uint32_t)top->r << 16) | ((uint32_t)top->g << 8) | top->b;
			bool vis = !top->hblank && !top->vblank;
			if (vis) {
				if (!in_vrun) { in_vrun = true; vrun = 0; }
				vrun++;
				if (!top->de && in_frame) {
					fr.border_n++;
					if (rgb != border_rgb) fr.border_err++;
				}
			} else {
				end_vrun();
				if (rgb != 0 && in_frame) fr.blank_err++;
			}
		}
		// DE runs (one per display line), sampled on the pixel clock
		if (top->ce_pix) {
			if (top->de) {
				if (!in_run) { in_run = true; run.clear(); runv.clear(); if (in_frame) fr.run_pos.push_back(in_line ? ln.ce : -1); }
				run.push_back(((uint32_t)top->r << 16) | ((uint32_t)top->g << 8) | top->b);
				runv.push_back(!top->hblank && !top->vblank);
			} else end_run();
		}
		if (top->ce_pix && in_line) {
			ln.ce++;
			if (!top->hblank && !top->vblank) ln.vis_px++;
		}
		if (top->de && !prev_de) {
			de_rises++;
			if (in_frame && fr.field < 0) fr.field = top->field;
		}
		if (in_frame) {
			if (top->vbl) fr.vbl++;
			if (top->hbl) { fr.hbl++; fr.hbl_t.push_back(cycles); }
			if (!top->de_tb && prev_detb) { fr.tb_falls++; fr.tbf_t.push_back(cycles); }
			if (top->de != top->de_tb) fr.detb_diff++;
			if (top->de && !prev_de && fr.first_de_rise == 0) fr.first_de_rise = cycles;
			if (!top->de && prev_de) fr.last_de_fall = cycles;
			if (top->underrun) fr.underruns++;
		}
		prev_hs = hs; prev_vs = vs; prev_de = top->de; prev_detb = top->de_tb;
	}
} mon;

//---------------------------------------------------------------------------
// Clock and bus
//---------------------------------------------------------------------------
static void tick()
{
	top->clk = 1;
	top->eval();
	cycles++;
	mon.sample();
	mem.step();
	top->clk = 0;
	top->eval();
}

static void ticks(int n) { while (n-- > 0) tick(); }

// one bus access; addr is the 68k address ($FF82xx or $FF98xx)
static uint16_t bus(uint32_t addr, bool we, uint16_t data, bool uds, bool lds)
{
	bool pal = (addr & 0xFFFC00) == 0xFF9800;
	top->bus_addr = (addr >> 1) & 0x3FF;
	top->bus_we = we;
	top->bus_din = data;
	top->bus_uds = uds;
	top->bus_lds = lds;
	top->bus_cs = !pal;
	top->pal_cs = pal;
	top->bus_stb = !pal;
	top->pal_stb = pal;
	int n = 0;
	uint16_t r = 0;
	for (;;) {
		tick();
		top->bus_stb = 0;
		top->pal_stb = 0;
		n++;
		if (top->bus_ack) { r = top->bus_dout; break; }
		if (n > 20) { printf("  bus timeout at %06x\n", addr); n_fail++; break; }
	}
	top->bus_cs = 0;
	top->pal_cs = 0;
	tick();
	return r;
}
static void ww(uint32_t a, uint16_t d) { bus(a, true, d, true, true); }
static void wb(uint32_t a, uint8_t d)
{
	if (a & 1) bus(a & ~1u, true, (d << 8) | d, false, true);
	else       bus(a, true, (d << 8) | d, true, false);
}
static uint16_t rw(uint32_t a) { return bus(a, false, 0, true, true); }
static uint8_t rb(uint32_t a)
{
	uint16_t v = bus(a & ~1u, false, 0, !(a & 1), (a & 1));
	return (a & 1) ? (v & 0xFF) : (v >> 8);
}

static void do_reset(int monitor)
{
	top->monitor_type = monitor;
	top->reset = 1;
	top->bus_cs = top->pal_cs = top->bus_stb = top->pal_stb = 0;
	ticks(8);
	top->reset = 0;
	mem.reset();
	mon.reset();
	ticks(2);
}

//---------------------------------------------------------------------------
// Modes
//---------------------------------------------------------------------------
enum { MON_MONO = 0, MON_RGB = 1, MON_VGA = 2 };

struct Mode {
	const char *name;
	int monitor;
	uint16_t vsetmode;
	// timing: HHT HBB HBE HDB HDE HSS VFT VBB VBE VDB VDE VSS
	uint16_t t[12];
	uint16_t vco, vmd, lwd, spshift;
	int st_shift;               // -1: Falcon mode, else value written to $8260
	// expectations
	double line_base_clocks;    // base clocks per line
	double base_hz;
	int lines_per_frame2;       // half lines per field (VFT+1)
	int width, height;          // de pixels per line, de lines per field
	int bpp;                    // 1,2,4,8,16
	bool stpal;
	int dbl, ilace;
};

// EmuTOS/TOS 4.04 Videl tables (see header); VCO/VMD/$8210/$8266 from the
// Hatari register dumps.
static const Mode modes[] = {
	{ "VGA 640x480x16 (Vsetmode $001A)", MON_VGA, 0x001A,
	  {0x00c6,0x008d,0x0015,0x02a3,0x007c,0x0096, 0x0419,0x03ff,0x003f,0x003f,0x03ff,0x0415},
	  0x0186, 0x0008, 0x00A0, 0x0000, -1,  800, 25175000, 1050, 640, 480, 4, false, 0, 0 },
	{ "VGA 320x240 true colour (Vsetmode $0114)", MON_VGA, 0x0114,
	  {0x00c6,0x008d,0x0015,0x02ac,0x0091,0x0096, 0x0419,0x03ff,0x003f,0x003f,0x03ff,0x0415},
	  0x0186, 0x0005, 0x0140, 0x0100, -1,  800, 25175000, 1050, 320, 480, 16, false, 1, 0 },
	{ "VGA 640x480x256 (Vsetmode $001B)", MON_VGA, 0x001B,
	  {0x00c6,0x008d,0x0015,0x02ab,0x0084,0x0096, 0x0419,0x03ff,0x003f,0x003f,0x03ff,0x0415},
	  0x0186, 0x0008, 0x0140, 0x0010, -1,  800, 25175000, 1050, 640, 480, 8, false, 0, 0 },
	{ "VGA ST low 320x200 line doubled (Vsetmode $0092)", MON_VGA, 0x0092,
	  {0x0017,0x0012,0x0001,0x020e,0x000d,0x0011, 0x0419,0x03af,0x008f,0x008f,0x03af,0x0415},
	  0x0186, 0x0005, 0x0050, 0x0000, 0,   800, 25175000, 1050, 320, 400, 4, true, 1, 0 },
	{ "RGB 640x400x16 interlaced (Vsetmode $012A)", MON_RGB, 0x012A,
	  {0x01fe,0x0199,0x0050,0x004d,0x00fe,0x01b2, 0x0270,0x0265,0x002f,0x007e,0x020e,0x026b},
	  0x0181, 0x0006, 0x00A0, 0x0000, -1, 2048, 32000000, 625, 640, 200, 4, false, 0, 1 },
	{ "RGB 320x200 true colour (Vsetmode $0024)", MON_RGB, 0x0024,
	  {0x00fe,0x00cb,0x0027,0x002e,0x008f,0x00d8, 0x0271,0x0265,0x002f,0x007f,0x020f,0x026b},
	  0x0181, 0x0000, 0x0140, 0x0100, -1, 2048, 32000000, 626, 320, 200, 16, false, 0, 0 },
	{ "VGA ST high 640x400 (Vsetmode $0098)", MON_VGA, 0x0098,
	  {0x00c6,0x008d,0x0015,0x0273,0x0050,0x0096, 0x0419,0x03af,0x008f,0x008f,0x03af,0x0415},
	  0x0186, 0x0008, 0x0028, 0x0400, -1,  800, 25175000, 1050, 640, 400, 1, false, 0, 0 },
	{ "RGB ST low 320x200 (Vsetmode $00A2)", MON_RGB, 0x00A2,
	  {0x003e,0x0032,0x0009,0x023f,0x001c,0x0034, 0x0271,0x0265,0x002f,0x006f,0x01ff,0x026b},
	  0x0081, 0x0000, 0x0050, 0x0000, 0,  2048, 32000000, 626, 320, 200, 4, true, 0, 0 },
	{ "RGB 640x200 true colour (Vsetmode $002C)", MON_RGB, 0x002C,
	  {0x01fe,0x0199,0x0050,0x0071,0x0122,0x01b2, 0x0271,0x0265,0x002f,0x007f,0x020f,0x026b},
	  0x0181, 0x0004, 0x0280, 0x0100, -1, 2048, 32000000, 626, 640, 200, 16, false, 0, 0 },
	{ "SM124 640x400 mono (Vsetmode $0088)", MON_MONO, 0x0088,
	  {0x001a,0x0000,0x0000,0x020f,0x000c,0x0014, 0x03e9,0x0000,0x0000,0x0043,0x0363,0x03e7},
	  0x0080, 0x0000, 0x0028, 0x0000, 2,   896, 32000000, 1002, 640, 400, 1, true, 0, 0 },
};

static int cyc_of(int monitor, uint16_t vco, uint16_t vmd, int bpp);
static uint32_t g_base = 0x100000;
static uint32_t fpal[256];      // as written (long)
static uint16_t spal[16];

static void program_mode(const Mode &m, uint32_t base, int hscroll = 0)
{
	static const uint32_t ta[12] = {0xFF8282,0xFF8284,0xFF8286,0xFF8288,0xFF828A,0xFF828C,
	                                0xFF82A2,0xFF82A4,0xFF82A6,0xFF82A8,0xFF82AA,0xFF82AC};
	for (int i = 0; i < 12; i++) ww(ta[i], m.t[i]);
	ww(0xFF820E, 0);
	ww(0xFF8266, m.spshift);
	if (m.st_shift >= 0) {
		wb(0xFF8260, m.st_shift);   // ST shifter mode: sets $8210 and $82C2
	} else {
		ww(0xFF8210, m.lwd);
		ww(0xFF82C2, m.vmd);
	}
	ww(0xFF82C0, m.vco);
	wb(0xFF8201, (base >> 16) & 0xFF);
	wb(0xFF8203, (base >> 8) & 0xFF);
	wb(0xFF820D, base & 0xFF);
	wb(0xFF8265, hscroll);
}

static uint32_t rand32() { return ((uint32_t)rand() << 16) ^ (uint32_t)rand(); }

static void fill_ram(uint32_t base, uint32_t len, uint32_t seed)
{
	srand(seed);
	for (uint32_t i = 0; i < len; i++) ram[(base + i) % RAM_SIZE] = rand() & 0xFF;
}

static void set_palettes(uint32_t seed)
{
	srand(seed);
	for (int i = 0; i < 256; i++) {
		fpal[i] = rand32();
		ww(0xFF9800 + i * 4, fpal[i] >> 16);
		ww(0xFF9802 + i * 4, fpal[i] & 0xFFFF);
	}
	for (int i = 0; i < 16; i++) {
		spal[i] = rand() & 0xFFFF;
		ww(0xFF8240 + i * 2, spal[i]);
	}
}

static uint32_t falcon_rgb(uint32_t c)
{
	uint32_t r = (c >> 24) & 0xFC, g = (c >> 16) & 0xFC, b = c & 0xFC;
	r |= r >> 6; g |= g >> 6; b |= b >> 6;
	return (r << 16) | (g << 8) | b;
}
static uint32_t ste_rgb(uint16_t c)
{
	auto e = [](int v) { v = ((v & 7) << 1) | (v >> 3); return v | (v << 4); };
	return (e((c >> 8) & 15) << 16) | (e((c >> 4) & 15) << 8) | e(c & 15);
}
static uint16_t rdw(uint32_t a) { return (ram[a % RAM_SIZE] << 8) | ram[(a + 1) % RAM_SIZE]; }

// expected colour of pixel x of source line starting at byte address la
static uint32_t expect_px(const Mode &m, uint32_t la, int x, int hs, int bank)
{
	if (m.bpp == 16) {
		uint16_t w = rdw(la + 2 * x);
		uint32_t r = ((w >> 8) & 0xf8) | (w >> 13);
		uint32_t g = ((w >> 3) & 0xfc) | ((w >> 9) & 0x3);
		uint32_t b = ((w << 3) & 0xf8) | ((w >> 2) & 0x07);
		return (r << 16) | (g << 8) | b;
	}
	int p = x + hs;
	int grp = p >> 4, bit = 15 - (p & 15);
	int idx = 0;
	for (int k = 0; k < m.bpp; k++)
		if (rdw(la + 2 * (grp * m.bpp + k)) & (1 << bit)) idx |= 1 << k;
	if (m.stpal) return ste_rgb(spal[idx & 15]);
	if (m.bpp == 4) idx |= bank << 4;
	return falcon_rgb(fpal[idx]);
}

// compare a captured field image with the framebuffer
static int g_blanked;   // DE pixels found blanked by the last compare_image
static int compare_image(const Mode &m, const FrameRec &f, uint32_t base, int hs, int bank,
                         int *checked)
{
	int words = m.lwd + (hs ? (m.bpp == 16 ? 16 : m.bpp) : 0);
	uint32_t stride = 2 * words;
	int errs = 0;
	*checked = 0;
	g_blanked = 0;
	for (size_t j = 0; j < f.img.size(); j++) {
		int src = (int)j;
		if (m.dbl) src = j / 2;
		if (m.ilace) src = 2 * j + (f.field ? 1 : 0);
		uint32_t la = base + src * stride;
		const std::vector<uint32_t> &px = f.img[j];
		for (size_t x = 0; x < px.size(); x++) {
			(*checked)++;
			if (j < f.msk.size() && x < f.msk[j].size() && !f.msk[j][x]) {
				// blanked by HBE/HBB/VBE/VBB: black
				g_blanked++;
				if (px[x] != 0) {
					if (errs < 5) printf("    blanked pixel line %zu x %zu not black: %06x\n", j, x, px[x]);
					errs++;
				}
				continue;
			}
			uint32_t e = expect_px(m, la, x, (m.bpp == 16) ? 0 : hs, bank);
			if (px[x] != e) {
				if (errs < 5)
					printf("    pixel mismatch line %zu x %zu: got %06x expected %06x\n", j, x, px[x], e);
				errs++;
			}
		}
	}
	return errs;
}

static bool run_frames(int n, uint64_t limit_clocks)
{
	uint64_t start = cycles;
	size_t want = mon.frames.size() + n;
	while (mon.frames.size() < want) {
		tick();
		if (cycles - start > limit_clocks) return false;
	}
	return true;
}

//---------------------------------------------------------------------------
// Tests
//---------------------------------------------------------------------------
static void test_mode(const Mode &m, bool worst_latency = false)
{
	printf("\n== Mode: %s%s ==\n", m.name, worst_latency ? " [40-clock bursts]" : "");
	do_reset(m.monitor);
	mem.fixed_worst = worst_latency;
	mon.hs_pol = false; mon.vs_pol = false;      // VCO bits 6/5 = 0: active low
	uint32_t base = g_base;
	fill_ram(base, 700000, 1234 + m.vsetmode);
	set_palettes(99 + m.vsetmode);
	program_mode(m, base);
	mon.border_rgb = m.stpal ? ste_rgb(spal[0]) : falcon_rgb(fpal[0]);
	if (m.st_shift >= 0) {
		uint16_t lw = rw(0xFF8210), vm = rw(0xFF82C2);
		check(lw == m.lwd && vm == m.vmd,
		      "$8260 write sets $8210=%04x $82C2=%04x (expected %04x %04x, Hatari VIDEL_ST_ShiftModeWriteByte)",
		      lw, vm, m.lwd, m.vmd);
	}
	mon.capture = true;
	// settle one field, then measure
	bool ok = run_frames(1, 4000000);
	int nf = m.ilace ? 4 : 2;
	ok = ok && run_frames(nf, 4000000ull * nf);
	check(ok, "frames produced");
	if (!ok) return;

	double clk_per_base = 32000000.0 / m.base_hz;
	double exp_line_clk = m.line_base_clocks * clk_per_base;
	int cyc = cyc_of(m.monitor, m.vco, m.vmd, m.bpp);
	int exp_ce = (int)m.line_base_clocks / cyc;

	size_t f0 = mon.frames.size() - nf;
	for (size_t k = f0; k < mon.frames.size(); k++) {
		const FrameRec &f = mon.frames[k];
		double exp_frame = m.lines_per_frame2 * m.line_base_clocks / 2.0 * clk_per_base;
		printf("  field %zu: %llu clocks, %d hsyncs, %d DE lines (%d..%d px), %d visible lines (%d..%d px), vbl %d hbl %d field %d\n",
		       k - f0, (unsigned long long)f.clocks, f.hsyncs, f.de_lines, f.min_de, f.max_de,
		       f.vis_lines, f.min_vis, f.max_vis, f.vbl, f.hbl, f.field);
		check(std::fabs((double)f.clocks - exp_frame) <= 1.0,
		      "frame period %llu clocks, expected %.2f (%d half lines x %.0f base clocks / 2 at %.3f MHz)",
		      (unsigned long long)f.clocks, exp_frame, m.lines_per_frame2, m.line_base_clocks, m.base_hz / 1e6);
		int exp_lines_lo = m.lines_per_frame2 / 2, exp_lines_hi = (m.lines_per_frame2 + 1) / 2;
		check(f.hsyncs == exp_lines_lo || f.hsyncs == exp_lines_hi,
		      "lines per field %d, expected %d..%d", f.hsyncs, exp_lines_lo, exp_lines_hi);
		check(f.de_lines == m.height, "active lines %d, expected %d", f.de_lines, m.height);
		check(f.min_de == m.width && f.max_de == m.width, "active pixels per line %d..%d, expected %d",
		      f.min_de, f.max_de, m.width);
		check(f.vbl == 1, "one VBL pulse per field (%d)", f.vbl);
		if (m.dbl) {
			check(f.hbl == f.hsyncs / 2 || f.hbl == (f.hsyncs + 1) / 2,
			      "doubled mode: one HBL pulse per two output lines (%d hbl, %d lines)", f.hbl, f.hsyncs);
			check(f.tb_falls == m.height / 2, "doubled mode: %d de_tb pulses, expected %d (one per source line)",
			      f.tb_falls, m.height / 2);
		} else {
			check(f.hbl == f.hsyncs, "one HBL pulse per line (%d hbl, %d lines)", f.hbl, f.hsyncs);
			check(f.detb_diff == 0 && f.tb_falls == m.height, "de_tb == de (%d differing clocks, %d de_tb pulses)",
			      f.detb_diff, f.tb_falls);
		}
		check(f.underruns == 0, "no line buffer underrun (%d)", f.underruns);
		int checked = 0;
		int errs = compare_image(m, f, base, 0, 0, &checked);
		check(errs == 0 && checked == m.width * m.height,
		      "pixel colours match the framebuffer (%d pixels checked, %d errors)", checked, errs);
		// visible area from the blank registers: HBE (first half) .. HBB
		// (second half) in units of D base clocks; lines whose start has
		// VBE <= VFC < VBB (VFC of a line start is even, or odd in the
		// other interlace field); mono: the display area
		{
			int D = m.st_shift >= 0 ? 16 : (m.monitor == MON_VGA ? (cyc == 4 ? 4 : 2) : cyc);
			int H = m.t[0] + 2;
			int vis_w, vis_l0 = 0, vis_l1 = 0;
			int vbe = m.t[8], vbb = m.t[7], vft = m.t[6];
			if (m.monitor == MON_MONO) { vis_w = m.width; vis_l0 = vis_l1 = m.height; }
			else {
				vis_w = ((H + m.t[1]) - m.t[2]) * D / cyc;
				for (int v = vbe; v < vbb && v <= vft; v++) { if (v & 1) vis_l1++; else vis_l0++; }
			}
			bool lines_ok = m.ilace ? (f.vruns == vis_l0 || f.vruns == vis_l1) : (f.vruns == vis_l0);
			check(f.vrun_min == vis_w && f.vrun_max == vis_w && lines_ok,
			      "visible area %d..%d px x %d lines, expected %d x %d (HBE/HBB, VBE/VBB)",
			      f.vrun_min, f.vrun_max, f.vruns, vis_w, m.ilace ? vis_l0 + vis_l1 : vis_l0);
			check(f.border_err == 0, "border pixels show palette entry 0 = %06x (%d of %d wrong)",
			      mon.border_rgb, f.border_err, f.border_n);
			check(f.blank_err == 0, "rgb is black during blanking (%d non-black samples)", f.blank_err);
			// hsync from HSS (second half) to the end of the line,
			// vsync from VSS to VFT+1 half lines
			int hs_base = 2 * H * D - (H + m.t[5]) * D;
			check(f.hs_ce == hs_base / cyc, "hsync width %d pixel clocks, expected %d (L - (H+HSS)*D = %d base clocks)",
			      f.hs_ce, hs_base / cyc, hs_base);
			double vs_exp = (vft + 1 - m.t[11]) * (double)H * D * clk_per_base;
			check(std::fabs((double)f.vs_clk - vs_exp) <= 1.0, "vsync width %llu clocks, expected %.1f (%d half lines)",
			      (unsigned long long)f.vs_clk, vs_exp, vft + 1 - m.t[11]);
		}
		if (m.ilace && k > f0)
			check(f.field != mon.frames[k - 1].field, "interlace field alternates (%d after %d)",
			      f.field, mon.frames[k - 1].field);
	}
	// line period: average over a field and per line in base clocks
	{
		const FrameRec &f = mon.frames.back();
		double avg = (double)f.clocks / (m.lines_per_frame2 / 2.0);
		check(std::fabs(avg - exp_line_clk) < 0.01,
		      "line period %.3f clocks, expected %.3f (%.0f base clocks)", avg, exp_line_clk, m.line_base_clocks);
	}
	// ce_pix count per line (pixel clock)
	{
		// measure directly: pixel clocks between two hsync starts
		int ce = 0; bool started = false; bool prev = true;
		uint64_t t0 = cycles;
		while (cycles - t0 < 100000) {
			tick();
			bool hs = !top->hsync;
			if (hs && !prev) { if (started) break; started = true; ce = 0; }
			if (started && top->ce_pix) ce++;
			prev = hs;
		}
		check(ce == exp_ce, "pixel clocks per line %d, expected %d (%d base clocks per pixel)", ce, exp_ce, cyc);
	}
	mem.fixed_worst = false;
}

static void test_registers()
{
	printf("\n== Register access ==\n");
	do_reset(MON_VGA);
	ww(0xFF820C, 0x00FE);
	wb(0xFF8201, 0x12);
	check(rb(0xFF820D) == 0, "write to $8201 clears $820D (Hatari VIDEL_ScreenBase_WriteByte): %02x", rb(0xFF820D));
	wb(0xFF820D, 0x56); wb(0xFF8203, 0x34);
	check(rb(0xFF820D) == 0 && rb(0xFF8203) == 0x34 && rb(0xFF8201) == 0x12,
	      "write to $8203 clears $820D, base reads %02x %02x %02x", rb(0xFF8201), rb(0xFF8203), rb(0xFF820D));
	wb(0xFF820D, 0x56);
	check(rb(0xFF820D) == 0x56, "$820D reads back 56: %02x", rb(0xFF820D));
	check(rw(0xFF8200) == 0xFF12, "$8200 word reads FF12 (even byte void): %04x", rw(0xFF8200));
	wb(0xFF820A, 0xFF);
	check(rb(0xFF820A) == 0x03, "$820A keeps bits 1:0 only (Hatari VIDEL_SyncMode_WriteByte): %02x", rb(0xFF820A));
	check(rb(0xFF820B) == 0x00, "$820B reads 00: %02x", rb(0xFF820B));
	ww(0xFF820E, 0xFFFF);
	check(rw(0xFF820E) == 0x01FF, "$820E reads 01FF after FFFF (Hatari VIDEL_LineOffset_ReadWord): %04x", rw(0xFF820E));
	ww(0xFF8210, 0x1234);
	check(rw(0xFF8210) == 0x1234, "$8210 reads back: %04x", rw(0xFF8210));
	check(rw(0xFF8212) == 0xFFFF && rw(0xFF8238) == 0xFFFF, "$8212-$823F void reads FFFF: %04x", rw(0xFF8212));
	check(rw(0xFF8262) == 0x0000 && rw(0xFF8268) == 0x0000 && rw(0xFF827E) == 0x0000,
	      "$8261-$8263, $8268-$827F read 0000");
	check(rw(0xFF8292) == 0xFFFF && rw(0xFF82B0) == 0xFFFF, "$8292-$829F, $82AE-$82BF void read FFFF");
	ww(0xFF8264, 0x0A05);
	check(rw(0xFF8264) == 0x0A05, "$8264/$8265 read back: %04x", rw(0xFF8264));
	ww(0xFF82C0, 0x0184);
	check(rw(0xFF82C0) == 0x0184, "VCO reads back the written value incl. bits 1:0 (Hatari): %04x", rw(0xFF82C0));
	static const uint32_t regs[] = {0xFF8282,0xFF8284,0xFF8286,0xFF8288,0xFF828A,0xFF828C,0xFF828E,0xFF8290,
	                                0xFF82A2,0xFF82A4,0xFF82A6,0xFF82A8,0xFF82AA,0xFF82AC,0xFF82C2,0xFF8266};
	bool ok = true;
	for (unsigned i = 0; i < sizeof(regs) / 4; i++) {
		ww(regs[i], 0x0100 + i * 3);
		if (rw(regs[i]) != 0x0100 + i * 3) ok = false;
	}
	check(ok, "timing/shift registers read back what was written");
	// byte write to a word register
	ww(0xFF8282, 0x0000); wb(0xFF8282, 0x01); wb(0xFF8283, 0x23);
	check(rw(0xFF8282) == 0x0123, "byte writes to HHT high/low: %04x", rw(0xFF8282));

	// $8260 effects per monitor (Hatari VIDEL_ST_ShiftModeWriteByte)
	struct { int mon; int st; uint16_t lw, vm; } se[] = {
		{MON_VGA, 0, 0x50, 5}, {MON_VGA, 1, 0x50, 9}, {MON_VGA, 2, 0x28, 8}, {MON_VGA, 3, 0x50, 0},
		{MON_RGB, 0, 0x50, 0}, {MON_RGB, 1, 0x50, 4}, {MON_RGB, 2, 0x28, 6},
		{MON_MONO, 2, 0x28, 0}, {3, 1, 0x50, 4},
	};
	for (auto &s : se) {
		do_reset(s.mon);
		wb(0xFF8260, 0xFC | s.st);
		uint16_t lw = rw(0xFF8210), vm = rw(0xFF82C2);
		uint8_t sh = rb(0xFF8260);
		check(lw == s.lw && vm == s.vm && sh == s.st,
		      "monitor %d: $8260=%d -> $8260 %d $8210 %04x $82C2 %04x (expected %04x %04x)",
		      s.mon, s.st, sh, lw, vm, s.lw, s.vm);
	}
}

static void test_palettes()
{
	printf("\n== Palettes ==\n");
	do_reset(MON_VGA);
	srand(7);
	bool ok = true;
	uint32_t v[256];
	for (int i = 0; i < 256; i++) {
		v[i] = rand32();
		ww(0xFF9800 + 4 * i, v[i] >> 16);
		ww(0xFF9802 + 4 * i, v[i] & 0xFFFF);
	}
	for (int i = 0; i < 256; i++) {
		uint32_t got = ((uint32_t)rw(0xFF9800 + 4 * i) << 16) | rw(0xFF9802 + 4 * i);
		if (got != (v[i] & 0xFCFC00FC)) {
			if (ok) printf("    entry %d: got %08x expected %08x\n", i, got, v[i] & 0xFCFC00FC);
			ok = false;
		}
	}
	check(ok, "256 Falcon palette entries read back masked with FCFC00FC (Hatari VIDEL_FalconColorRegsWrite)");
	ww(0xFF9804, 0x0000); ww(0xFF9806, 0x0000);
	wb(0xFF9804, 0xFF);
	check(rw(0xFF9804) == 0xFC00 && rw(0xFF9806) == 0x0000, "byte write to red only: %04x %04x",
	      rw(0xFF9804), rw(0xFF9806));
	wb(0xFF9805, 0xA5);
	wb(0xFF9807, 0x5B);
	wb(0xFF9806, 0xFF);
	check(rw(0xFF9804) == 0xFCA4 && rw(0xFF9806) == 0x0058, "byte writes to green/blue: %04x %04x",
	      rw(0xFF9804), rw(0xFF9806));

	// ST palette, examples from Hatari Videl_ColorReg_WriteWord
	ww(0xFF8240, 0x0000);
	wb(0xFF8240, 0x07);
	check(rw(0xFF8240) == 0x0707, "move.b #7,$ff8240 -> $707: %03x", rw(0xFF8240));
	wb(0xFF8241, 0x55);
	check(rw(0xFF8240) == 0x0555, "move.b #$55,$ff8241 -> $555: %03x", rw(0xFF8240));
	wb(0xFF8240, 0x71);
	check(rw(0xFF8240) == 0x0171, "move.b #$71,$ff8240 -> $171: %03x", rw(0xFF8240));
	ok = true;
	for (int i = 0; i < 16; i++) ww(0xFF8240 + 2 * i, 0xF000 | (i * 0x111 + 0x123));
	for (int i = 0; i < 16; i++) if (rw(0xFF8240 + 2 * i) != ((i * 0x111 + 0x123) & 0xFFF)) ok = false;
	check(ok, "16 ST palette entries read back masked with 0FFF");
}

// screen counter and VFC readback in VGA 640x480x16
static void test_counters()
{
	printf("\n== Counters ==\n");
	const Mode &m = modes[0];
	do_reset(m.monitor);
	uint32_t base = 0x123400;
	fill_ram(base, 400000, 5);
	program_mode(m, base);
	run_frames(2, 3000000);
	// wait for the vsync start, read VFC
	bool prev = true;
	while (true) { tick(); bool vs = !top->vsync; if (vs && !prev) break; prev = vs; }
	uint16_t vfc = rw(0xFF82A0);
	check(vfc >= m.t[11] && vfc <= m.t[11] + 1, "VFC at vsync start = %d, expected VSS=%d (+1)", vfc, m.t[11]);
	// after the frame start (VFC wrap), before the first display line:
	// counter = screen base (latched at the frame start)
	while (rw(0xFF82A0) > 8) ticks(50);
	uint32_t vc = (rb(0xFF8205) << 16) | (rb(0xFF8207) << 8) | rb(0xFF8209);
	check(vc == base, "video counter after the frame start %06x, expected base %06x", vc, base);
	// VFC advances by two per line: sample at two hsync starts 10 lines apart
	{
		int lines = 0; uint16_t v0 = 0, v1 = 0;
		prev = true;
		while (lines <= 10) {
			tick();
			bool hs = !top->hsync;
			if (hs && !prev) {
				if (lines == 0) v0 = rw(0xFF82A0);
				if (lines == 10) v1 = rw(0xFF82A0);
				lines++;
			}
			prev = hs;
		}
		int d = ((int)v1 - (int)v0 + (m.t[6] + 1)) % (m.t[6] + 1);
		check(d == 20, "VFC advances 2 per line: %d -> %d over 10 lines (expected +20)", v0, v1);
	}
	// max VFC seen = VFT
	int vmax = 0;
	for (int i = 0; i < 3000; i++) { int x = rw(0xFF82A0); vmax = std::max(vmax, x); ticks(rand() % 400); }
	check(vmax <= m.t[6] && vmax >= m.t[6] - 2, "VFC counts up to VFT=%d (max seen %d)", m.t[6], vmax);
	// during display of line 10
	mon.de_rises = 0;
	prev = true;
	while (true) { tick(); bool vs = !top->vsync; if (vs && !prev) break; prev = vs; }
	mon.de_rises = 0;
	while (mon.de_rises < 11) tick();
	ticks(200);
	vc = (rb(0xFF8205) << 16) | (rb(0xFF8207) << 8) | rb(0xFF8209);
	uint32_t l10 = base + 10 * 2 * m.lwd;
	check(vc > l10 && vc < l10 + 2 * m.lwd, "video counter during line 10: %06x within [%06x, %06x)",
	      vc, l10, l10 + 2 * m.lwd);
	// HHC readback stays within the half line
	int hmax = 0;
	for (int i = 0; i < 500; i++) { hmax = std::max(hmax, (int)rw(0xFF8280)); ticks(rand() % 50); }
	check(hmax <= m.t[0] + 1 && hmax >= m.t[0] - 2, "HHC counts 0..HHT+1 (max seen %d, HHT %d)", hmax, m.t[0]);
}

static void test_hscroll(int mi, int hs, int bank)
{
	const Mode &m = modes[mi];
	printf("\n== Horizontal fine scroll %d in %s%s ==\n", hs, m.name, bank ? " with colour bank" : "");
	do_reset(m.monitor);
	uint32_t base = 0x200000;
	fill_ram(base, 700000, 77 + hs);
	set_palettes(33 + hs);
	program_mode(m, base, hs);
	if (bank) ww(0xFF8266, m.spshift | bank);
	mon.capture = true;
	run_frames(1, 4000000);
	run_frames(1, 4000000);
	const FrameRec &f = mon.frames.back();
	check(f.de_lines == m.height && f.min_de == m.width && f.max_de == m.width,
	      "%d lines of %d pixels (expected %d x %d)", f.de_lines, f.min_de, m.height, m.width);
	int checked = 0;
	int errs = compare_image(m, f, base, hs, bank, &checked);
	check(errs == 0 && checked == m.width * m.height,
	      "pixels shifted left by %d, stride + %d words (%d checked, %d errors)", hs, m.bpp == 16 ? 16 : m.bpp, checked, errs);
	// cross check: the same image without scroll differs
	int errs0 = compare_image(m, f, base, 0, bank, &checked);
	check(errs0 > 0, "image differs from the unscrolled decode (%d differences)", errs0);
}

// "1600 x 600" (Archangel, 1993; the user's hardware report): XBIOS
// Vsetmode($00A2) (RGB ST low), then the program's own writes, in its order
// (disassembly at text+$2E2): VMD |= $0A (interlace, one dot per pixel),
// $8210 = 400 words, VFT $270, VSS $26A, VBB $26C, VBE $F, VDB $F,
// VDE $26A, HDB $23D, HDE $2E.  Screen base $027B54 (Hatari trace): four
// bytes into a 64-bit word.  Hatari shows 1602x605 at 4 bitplanes with the
// ST palette; every DE pixel must decode from its own line.
static void test_1600x600(bool worst_latency)
{
	printf("\n== 1600 x 600 demo mode (RGB, ST low + VMD $0A, $8210 = 400)%s ==\n",
	       worst_latency ? " [40-clock bursts]" : "");
	const Mode &st = modes[7];          // RGB ST low 320x200 (Vsetmode $00A2)
	do_reset(st.monitor);
	mem.fixed_worst = worst_latency;
	mon.hs_pol = false; mon.vs_pol = false;
	uint32_t base = 0x027B54;
	fill_ram(base & ~0xFFFu, 520000, 1600);
	set_palettes(600);
	program_mode(st, base);
	ww(0xFF82C2, rw(0xFF82C2) | 0x000A);
	ww(0xFF8210, 400);
	ww(0xFF82A2, 0x0270); ww(0xFF82AC, 0x026A); ww(0xFF82A4, 0x026C); ww(0xFF82A6, 0x000F);
	ww(0xFF82A8, 0x000F); ww(0xFF82AA, 0x026A); ww(0xFF8288, 0x023D); ww(0xFF828A, 0x002E);
	mon.border_rgb = ste_rgb(spal[0]);
	mon.capture = true;
	bool ok = run_frames(1, 8000000) && run_frames(4, 32000000);
	check(ok, "frames produced");
	if (!ok) return;
	Mode m = st;
	m.lwd = 400; m.vmd = 0x000A; m.ilace = 1; m.width = 1600;
	for (size_t k = mon.frames.size() - 4; k < mon.frames.size(); k++) {
		const FrameRec &f = mon.frames[k];
		check(f.min_de == 1600 && f.max_de == 1600, "field %zu: active pixels per line %d..%d, expected 1600",
		      k, f.min_de, f.max_de);
		check(f.de_lines >= 300 && f.de_lines <= 303, "field %zu: %d active lines (Hatari: 605 per frame)",
		      k, f.de_lines);
		check(f.underruns == 0, "field %zu: no line buffer underrun (%d)", k, f.underruns);
		if (getenv("DIAG1600") && f.img.size() > 2) {
			for (int j = 0; j < 2; j++) {
				int best = -1, bsrc = 0, boff = 0;
				for (int src = 0; src < 6; src++)
					for (int off = -16; off <= 16; off++) {
						int match = 0;
						uint32_t la = base + src * 800 + 2 * off;
						for (int x = 0; x < 1600; x++) if (f.img[j][x] == expect_px(m, la, x, 0, 0)) match++;
						if (match > best) { best = match; bsrc = src; boff = off; }
					}
				printf("    DIAG field %zu line %d: best source line %d, word offset %d, %d of 1600 match\n", k, j, bsrc, boff, best);
				// where along the line does the best fit hold
				uint32_t la = base + bsrc * 800 + 2 * boff;
				int first_ok = -1, last_bad = -1;
				for (int x = 0; x < 1600; x++) { bool ok = f.img[j][x] == expect_px(m, la, x, 0, 0); if (ok && first_ok < 0) first_ok = x; if (!ok) last_bad = x; }
				printf("    DIAG   first match x %d, last mismatch x %d\n", first_ok, last_bad);
			}
			int par = -1;
			for (int pp = 0; pp < 2; pp++) {
				uint32_t la = base + pp * 800;
				int mt = 0; for (int x = 0; x < 1600; x++) if (f.img[1][x] == expect_px(m, la, x, 0, 0)) mt++;
				if (mt == 1600) par = pp;
			}
			long bad = 0; int badlines = 0, minx = 1600, maxx = -1;
			for (size_t j = 1; j < f.img.size(); j++) {
				uint32_t la = base + (2 * (j - 1) + par) * 800;
				int lb = 0;
				for (int x = 0; x < 1600; x++) if (f.img[j][x] != expect_px(m, la, x, 0, 0)) { lb++; minx = std::min(minx, x); maxx = std::max(maxx, x); }
				bad += lb; if (lb) badlines++;
			}
			printf("    DIAG field %zu parity %d: lines 1..%zu shifted by one: %ld bad pixels in %d lines (x %d..%d)\n",
			       k, par, f.img.size() - 1, bad, badlines, minx, maxx);
		}
		int checked = 0;
		int errs = compare_image(m, f, base, 0, 0, &checked);
		check(errs == 0 && checked == 1600 * f.de_lines,
		      "field %zu (%s): pixel colours match the framebuffer (%d pixels checked, %d errors)",
		      k, f.field ? "odd" : "even", checked, errs);
	}
}

// register writes in the middle of a frame / line are latched
static void test_latching()
{
	printf("\n== Mid-frame register writes ==\n");
	const Mode &m = modes[0];
	do_reset(m.monitor);
	uint32_t base = 0x100000, base2 = 0x180000;
	fill_ram(base, 400000, 11);
	fill_ram(base2, 400000, 12);
	set_palettes(3);
	program_mode(m, base);
	mon.capture = true;
	run_frames(2, 3000000);
	// in the middle of the 100th display line: change HDE and the base
	bool prev = true;
	while (true) { tick(); bool vs = !top->vsync; if (vs && !prev) break; prev = vs; }
	mon.de_rises = 0;
	while (mon.de_rises < 100) tick();
	ticks(300);    // inside DE
	size_t fidx = mon.frames.size();
	ww(0xFF8288, m.t[3] - 16);          // HDB - 16 units = DE 32 pixels earlier
	wb(0xFF8201, (base2 >> 16) & 0xFF); wb(0xFF8203, (base2 >> 8) & 0xFF); wb(0xFF820D, base2 & 0xFF);
	run_frames(1, 3000000);
	const FrameRec &f = mon.frames[fidx];
	// The horizontal registers are latched at the start of each output line;
	// the DE start position comes from the parameters of the output line it
	// lands in, so the write made inside the line of display line 100 moves
	// display line 101 on by 32 pixel clocks.  The width stays $8210 (640).
	int p0 = f.run_pos.size() ? f.run_pos[0] : -1, same = 0, moved = 0, other = 0, w640 = 0;
	for (size_t j = 0; j < f.run_pos.size(); j++) {
		if (f.run_pos[j] == p0) same++; else if (f.run_pos[j] == p0 - 32) moved++; else other++;
		if (j < f.img.size() && f.img[j].size() == 640) w640++;
	}
	check(same == 100 && moved == 380 && other == 0 && w640 == 480,
	      "HDB write inside line 100 moves DE from line 101 on: %d lines at %d, %d at %d, %d other, %d of width 640",
	      same, p0, moved, p0 - 32, other, w640);
	// the rest of this frame still shows the old base
	int checked = 0;
	Mode m2 = m;
	FrameRec part = f;
	part.img.resize(100);
	int errs = compare_image(m2, part, base, 0, 0, &checked);
	check(errs == 0, "lines before the change from the old base (%d errors)", errs);
	FrameRec rest = f;
	rest.img.erase(rest.img.begin(), rest.img.begin() + 100);
	// the moved DE starts 32 dots before HBE: those pixels are blanked (black)
	int errs_old = 0, n = 0, nblank = 0;
	for (size_t j = 0; j < rest.img.size(); j++) {
		uint32_t la = base + (100 + j) * 2 * m.lwd;
		for (size_t x = 0; x < rest.img[j].size(); x++) {
			n++;
			if (!rest.msk[100 + j][x]) { nblank++; if (rest.img[j][x] != 0) errs_old++; continue; }
			if (rest.img[j][x] != expect_px(m, la, x, 0, 0)) errs_old++;
		}
	}
	check(errs_old == 0 && n > 0 && nblank == 32 * 380,
	      "screen base change does not affect the current frame (%d errors in %d px, %d blanked before HBE)", errs_old, n, nblank);
	run_frames(1, 3000000);
	const FrameRec &g = mon.frames.back();
	int errs2 = 0; n = 0; nblank = 0;
	for (size_t j = 0; j < g.img.size(); j++) {
		uint32_t la = base2 + j * 2 * m.lwd;
		for (size_t x = 0; x < g.img[j].size(); x++) {
			n++;
			if (!g.msk[j][x]) { nblank++; if (g.img[j][x] != 0) errs2++; continue; }
			if (g.img[j][x] != expect_px(m, la, x, 0, 0)) errs2++;
		}
	}
	check(errs2 == 0 && n == 640 * 480 && nblank == 32 * 480,
	      "next frame uses the new base (%d errors in %d px, %d blanked before HBE)", errs2, n, nblank);
}

// mid-line timing writes must not glitch hsync inside the line
static void test_sync_stability()
{
	printf("\n== Sync stability under register writes ==\n");
	const Mode &m = modes[0];
	do_reset(m.monitor);
	program_mode(m, 0x100000);
	run_frames(1, 3000000);
	// hammer HSS/HBB/HHT with the same values plus a different HSS for
	// several lines: hsync pulses must keep a whole-line period
	bool prev = true;
	std::vector<uint64_t> starts;
	uint64_t t0 = cycles;
	int phase = 0;
	while (starts.size() < 40) {
		if ((cycles - t0) % 97 == 0) {
			phase++;
			ww(0xFF828C, (phase & 1) ? m.t[5] : m.t[5] + 4);
		} else tick();
		bool hs = !top->hsync;
		if (hs && !prev) starts.push_back(cycles);
		prev = hs;
	}
	bool ok = true;
	for (size_t i = 2; i < starts.size(); i++) {
		uint64_t d = starts[i] - starts[i - 1];
		// HSS moves by 0 or 4 units (8 base clocks): period 1017 +- 10 clocks
		if (d < 1000 || d > 1035) { ok = false; printf("    hsync period %llu\n", (unsigned long long)d); }
	}
	check(ok, "hsync period stays one line (+- the HSS change) while HSS is rewritten every 97 clocks");
}

// VCO bit 6 / bit 5 select the hsync / vsync polarity
static void test_polarity()
{
	printf("\n== Sync polarity ==\n");
	const Mode &m = modes[0];
	do_reset(m.monitor);
	program_mode(m, 0x100000);
	ticks(20000);
	int hs_hi = 0, vs_hi = 0, n = 0;
	for (int i = 0; i < 600000; i++) { tick(); hs_hi += top->hsync; vs_hi += top->vsync; n++; }
	check(hs_hi > n * 8 / 10 && vs_hi > n * 9 / 10, "VCO=%04x: hsync/vsync idle high, pulses low (%d%% / %d%% high)",
	      m.vco, hs_hi * 100 / n, vs_hi * 100 / n);
	ww(0xFF82C0, m.vco | 0x0060);
	ticks(20000);
	hs_hi = vs_hi = n = 0;
	for (int i = 0; i < 600000; i++) { tick(); hs_hi += top->hsync; vs_hi += top->vsync; n++; }
	check(hs_hi < n * 2 / 10 && vs_hi < n * 1 / 10, "VCO=%04x: hsync/vsync idle low, pulses high (%d%% / %d%% high)",
	      m.vco | 0x60, hs_hi * 100 / n, vs_hi * 100 / n);
}

// Timer B / HBL in a line doubled mode (VGA ST low): one event per source
// line, on the repeat copy; a COLOR00 change made at the k-th de_tb falling
// edge (a Timer B event count interrupt with AER bit 3 = 0) first shows on
// output display line 2k.
static void test_timerb()
{
	const Mode &m = modes[3];
	printf("\n== Timer B / HBL per source line in %s ==\n", m.name);
	do_reset(m.monitor);
	uint32_t base = 0x100000;
	fill_ram(base, 200000, 21);
	set_palettes(22);
	program_mode(m, base);
	mon.capture = true;
	run_frames(1, 4000000);
	run_frames(1, 4000000);
	const FrameRec &f = mon.frames.back();
	check(f.tb_falls == 200, "%d de_tb falling edges per frame, expected 200", f.tb_falls);
	int hbl_disp = 0;
	for (uint64_t t : f.hbl_t) if (t >= f.first_de_rise && t <= f.last_de_fall) hbl_disp++;
	check(hbl_disp == 200, "%d HBL pulses during the 400 displayed output lines, expected 200 (%d per frame, %d lines)",
	      hbl_disp, f.hbl, f.hsyncs);
	// HBL phase: in this mode HSS (672) falls inside the DE of its output
	// line (36..676), so each HBL of a displayed pair must come while de_tb
	// is high, i.e. on the repeat copy
	{
		int on_rep = 0, n = 0; bool pv = true;
		while (true) { tick(); bool vs = !top->vsync; if (vs && !pv) break; pv = vs; }
		pv = true;
		while (true) {
			tick();
			if (top->hbl) { if (top->de) { n++; if (top->de_tb) on_rep++; } }
			bool vs = !top->vsync; if (vs && !pv) break; pv = vs;
		}
		check(n == 200 && on_rep == 200, "HBL pulses inside display lines: %d, on the repeat copy: %d (expected 200/200)", n, on_rep);
	}
	// first de_tb fall comes after the second copy of source line 0:
	// the 2nd DE run ends there (de and de_tb fall together)
	// - count de falling edges before the first de_tb fall
	{
		// rerun a frame and watch both
		bool pd = false, pt = false; int de_falls = 0, first = -1;
		bool pv = true; while (true) { tick(); bool vs = !top->vsync; if (vs && !pv) break; pv = vs; }
		while (first < 0) {
			tick();
			if (!top->de && pd) de_falls++;
			if (!top->de_tb && pt) first = de_falls;
			pd = top->de; pt = top->de_tb;
		}
		check(first == 2, "first de_tb fall at the end of DE run %d (expected 2: after both copies of line 0)", first);
	}
	// COLOR00 change at each de_tb falling edge
	uint16_t col[202];
	for (int k = 0; k < 202; k++) col[k] = (k * 0x123 + 0x456) & 0xFFF;
	{
		bool pv = true; while (true) { tick(); bool vs = !top->vsync; if (vs && !pv) break; pv = vs; }
	}
	ww(0xFF8240, col[0]);
	// wait for the next frame start (vsync) so the frame begins with col[0]
	{
		bool pv = true; while (true) { tick(); bool vs = !top->vsync; if (vs && !pv) break; pv = vs; }
	}
	std::vector<uint32_t> left;   // border colour just before each DE run
	uint32_t last_border = 0; bool pt = false, pd = false;
	int k = 0;
	while ((int)left.size() < 400) {
		tick();
		bool detb_fall = !top->de_tb && pt;
		if (top->ce_pix && !top->hblank && !top->vblank && !top->de)
			last_border = ((uint32_t)top->r << 16) | ((uint32_t)top->g << 8) | top->b;
		if (top->de && !pd) left.push_back(last_border);
		pt = top->de_tb; pd = top->de;
		if (detb_fall) { k++; ww(0xFF8240, col[k]); }    // Timer B interrupt handler
	}
	int bad = 0;
	for (int j = 0; j < 400; j++) {
		uint32_t e = ste_rgb(col[j / 2]);
		if (left[j] != e) { if (bad < 5) printf("    line %d border %06x expected %06x\n", j, left[j], e); bad++; }
	}
	check(bad == 0 && k >= 199, "COLOR00 written at the k-th de_tb fall first shows on output line 2k (%d wrong of 400, %d events)", bad, k);
}

//---------------------------------------------------------------------------
// Register sets replayed from Hatari (TOS 4.04 Vsetmode modes)
//---------------------------------------------------------------------------
struct HSet {
	const char *name; int monitor, stpal, s8260; uint16_t spshift, lwd, vco, vmd; uint16_t t[12];
	int hw, hh, hbpp;
};
static const HSet hsets[] = {
#include "hatari_modes.inc"
};

// dots per pixel (the RTL rule): VMD bits 3:2 4/2/1; one on a mono
// monitor; VGA true colour 2 unless VMD bit 3 (124 Beers Later, VMD=1)
static int cyc_of(int monitor, uint16_t vco, uint16_t vmd, int bpp)
{
	if (monitor == MON_MONO) return 1;
	bool vga = monitor == MON_VGA || (vco & 3) == 2;
	if (bpp == 16 && vga) return (vmd & 8) ? 1 : 2;
	int v = (vmd >> 2) & 3;
	return v == 0 ? 4 : v == 1 ? 2 : 1;
}
// Hatari VIDEL_getScreenBpp
static int bpp_of(uint16_t spshift, int stpal, int s8260)
{
	if (spshift & 0x400) return 1;
	if (spshift & 0x100) return 16;
	if (spshift & 0x010) return 8;
	if (!stpal) return 4;
	return (s8260 & 3) == 0 ? 4 : (s8260 & 3) == 1 ? 2 : 1;
}

// horizontal unit in base clocks (dots), the RTL rule: 16 in ST mode with
// ST timing; true colour counts pixel periods (prescaler 1); VGA bitplanes
// 4 (VMD[3:2]=00) or 2; otherwise the cycles per pixel
static int unit_of(int monitor, uint16_t vco, uint16_t vmd, int bpp, int stpal, uint16_t hht)
{
	if (stpal && (hht & 0x1ff) < 0x50) return 16;
	int cyc = cyc_of(monitor, vco, vmd, bpp);
	bool vga = monitor == MON_VGA || (vco & 3) == 2;
	if (bpp == 16) return cyc;
	return vga ? (((vmd >> 2) & 3) == 0 ? 4 : 2) : cyc;
}

// program a register set in the order TOS (EmuTOS Vsetmode, traced in
// Hatari) writes it: timing, $820E, $8210, $82C2, $82C0, $8266, $8260
static void program_set(const uint16_t *t, uint16_t lwd, uint16_t vmd, uint16_t vco, uint16_t spshift,
                        int s8260, uint32_t base)
{
	static const uint32_t ta[12] = {0xFF8282,0xFF8284,0xFF8286,0xFF8288,0xFF828A,0xFF828C,
	                                0xFF82A2,0xFF82A4,0xFF82A6,0xFF82A8,0xFF82AA,0xFF82AC};
	for (int i = 0; i < 12; i++) ww(ta[i], t[i]);
	ww(0xFF820E, 0);
	ww(0xFF8210, lwd);
	ww(0xFF82C2, vmd);
	ww(0xFF82C0, vco);
	ww(0xFF8266, spshift);
	if (s8260 >= 0) {
		wb(0xFF8260, s8260);
		ww(0xFF8210, lwd);      // final values as Hatari holds them
		ww(0xFF82C2, vmd);
	}
	wb(0xFF8201, (base >> 16) & 0xFF);
	wb(0xFF8203, (base >> 8) & 0xFF);
	wb(0xFF820D, base & 0xFF);
}

// measure two fields and check DE size, lines, Timer B, VBL and pixels
// against Hatari's XSize/YSize; returns the number of failed checks
// DE start of a display line in dots from the line start, as the header
// documents it (HDB position + base + body + D), before the modulo
static int de_start_raw(const uint16_t *t, int bpp, int stpal, int cyc, int D, uint16_t vco, uint16_t vmd, int *L)
{
	int H = (t[0] & 0x1ff) + 2;
	*L = 2 * H * D;
	int hdb = t[3];
	int pos_hdb = ((hdb & 0x200) ? H * D : 0) + (hdb & 0x1ff) * D;
	bool tc1 = bpp == 16 && cyc == 1 && !stpal;
	int base = stpal ? ((vco & 0x100) ? 128 : 192) : tc1 ? 0 : ((vco & 0x100) ? 64 : 128);
	int body = stpal ? (128 / bpp + 2) * cyc : (bpp == 16 ? 16 * cyc : (128 / bpp + 18) * cyc);
	(void)vmd;
	return pos_hdb + base + body + D;
}

// display and visible lines of a field from the register semantics: output
// line j starts at VFC v = 2j + ph; it holds a display line when the HDB
// that starts it (second half line if HDB bit 9, the previous line if the DE
// start wraps) saw VDB <= VFC < VDE; it is blanked when v >= VBB or v < VBE
// (VBE 11-bit signed, sampled at the line start)
static void expect_lines(const uint16_t *t, int wrap, int ph, int *de_lines, int *vis_lines)
{
	int vft = t[6] & 0x7ff, vbb = t[7] & 0x7ff, vbe = t[8] & 0x7ff, vdb = t[9] & 0x7ff, vde = t[10] & 0x7ff;
	bool vbe_neg = vbe & 0x400;
	int b9 = (t[3] & 0x200) ? 1 : 0;
	*de_lines = 0; *vis_lines = 0;
	for (int v = ph; v <= vft; v += 2) {
		int h = v + b9 - 2 * wrap;
		if (h < vdb || h >= vde) continue;
		(*de_lines)++;
		bool blank = (v >= vbb) || (!vbe_neg && v < vbe);
		if (!blank) (*vis_lines)++;
	}
}

static int check_fields(const char *name, int monitor, const uint16_t *t,
                        uint16_t lwd, uint16_t vmd, uint16_t spshift, int stpal, int s8260,
                        uint32_t base, int D, double *frame_clk, uint16_t vco)
{
	int fails0 = n_fail;
	uint16_t hht = t[0], vdb = t[9], vde = t[10], vft = t[6];
	// field period: (VFT+1) half lines of (HHT+2)*D dots at the VCO clock
	double exp_frame = vft ? (vft + 1) * (double)((hht & 0x1ff) + 2) * D * (32000000.0 / ((vco & 4) ? 25175000.0 : 32000000.0)) : 0;
	int bpp = bpp_of(spshift, stpal, s8260);
	int cyc = cyc_of(monitor, vco, vmd, bpp);
	int xs = (lwd & 0x3ff) * 16 / bpp;                  // Hatari XSize
	int Lx, raw = de_start_raw(t, bpp, stpal, cyc, D, vco, vmd, &Lx);
	int wrap = raw >= Lx ? 1 : 0;
	int del0, vis0, del1, vis1;
	expect_lines(t, wrap, 0, &del0, &vis0);
	expect_lines(t, wrap, 1, &del1, &vis1);
	bool two_ph = ((vft & 0x7ff) + 1) & 1;             // odd number of half lines: fields alternate
	if (monitor == MON_MONO) { vis0 = del0; vis1 = del1; }   // SM124: no blank, the display window is the visible line
	int ysh = (vde & 0x7ff) - (vdb & 0x7ff);
	int ys = ysh; if (!(vmd & 2)) ys >>= 1; if (vmd & 1) ys >>= 1;   // Hatari YSize
	bool il = vmd & 2, dbl = vmd & 1;
	int exp_lines = il ? ys / 2 : ys * (dbl ? 2 : 1);   // DE lines per field
	int exp_tb = il ? ys / 2 : ys;                      // Timer B events per field
	// width clipped to the line length (RTL rule): L = 2*(HHT+2)*D base clocks
	int L = 2 * ((hht & 0x1ff) + 2) * D;
	int px_max = (L - cyc) / cyc;
	int exp_w = std::min(xs, px_max);
	mon.capture = true;
	size_t f0 = mon.frames.size();
	bool ok = run_frames(1, 6000000) && run_frames(2, 12000000);
	check(ok, "%s: fields produced", name);
	if (!ok) return n_fail - fails0;
	Mode m = {};
	m.name = name; m.lwd = lwd & 0x3ff; m.bpp = bpp; m.stpal = stpal && bpp != 16; m.dbl = dbl; m.ilace = il;
	for (size_t k = f0 + 1; k < mon.frames.size(); k++) {
		const FrameRec &f = mon.frames[k];
		int checked = 0;
		int errs = compare_image(m, f, base, 0, (bpp == 4 && !stpal) ? (spshift & 15) : 0, &checked);
		bool okw = f.min_de == exp_w && f.max_de == exp_w;
		bool vis_ok = two_ph ? (f.vis_de_lines == vis0 || f.vis_de_lines == vis1) : (f.vis_de_lines == vis0);
		bool del_ok = two_ph ? (del0 == exp_lines || del1 == exp_lines) : (del0 == exp_lines);
		check(okw && f.de_lines == exp_lines && f.tb_falls == exp_tb && f.vbl == 1 && errs == 0 &&
		      checked == exp_w * exp_lines && f.underruns == 0 && vis_ok && del_ok,
		      "%s field %zu: DE %d..%d px x %d lines (Hatari %dx%d%s -> expected %d x %d), %d unblanked lines (exp %d%s), Timer B %d (exp %d), VBL %d, %d px checked (%d blanked), %d errors",
		      name, k - f0 - 1, f.min_de, f.max_de, f.de_lines, xs, ys, exp_w < xs ? ", clipped to the line" : "",
		      exp_w, exp_lines, f.vis_de_lines, vis0, two_ph ? "/alt field" : "", f.tb_falls, exp_tb, f.vbl, checked, g_blanked, errs);
		if (frame_clk) *frame_clk = (double)f.clocks;
		if (exp_frame > 0)
			check(std::fabs((double)f.clocks - exp_frame) <= 1.0, "%s field %zu: period %llu clocks, expected %.2f",
			      name, k - f0 - 1, (unsigned long long)f.clocks, exp_frame);
	}
	return n_fail - fails0;
}

static void test_hatari_sets(bool worst)
{
	printf("\n== Hatari register sets of all TOS 4.04 Vsetmode modes + 124 Beers Later sets%s ==\n",
	       worst ? " [every burst 40 clocks]" : " [bursts 6..36 clocks random]");
	int nfail = 0;
	for (const HSet &h : hsets) {
		do_reset(h.monitor);
		mem.fixed_worst = worst;
		mon.hs_pol = (h.vco >> 6) & 1; mon.vs_pol = (h.vco >> 5) & 1;
		uint32_t base = 0x100000;
		fill_ram(base, 1000000, 5000 + h.vmd + h.lwd);
		set_palettes(6000 + h.lwd);
		program_set(h.t, h.lwd, h.vmd, h.vco, h.spshift, h.stpal ? h.s8260 : -1, base);
		uint16_t lw = rw(0xFF8210), vm = rw(0xFF82C2), vc = rw(0xFF82C0);
		bool regs_ok = lw == h.lwd && vm == h.vmd && vc == h.vco;
		if (!regs_ok) {
			check(false, "%s: registers after the TOS write order $8210 %04x $82C2 %04x $82C0 %04x (Hatari %04x %04x %04x)",
			      h.name, lw, vm, vc, h.lwd, h.vmd, h.vco);
			nfail++;
		}
		// horizontal unit (RTL rule): 16 for ST mode with ST timing, else
		// VGA divider (VGA monitor or VCO=VGA) 4/2, else cycles per pixel
		int D = unit_of(h.monitor, h.vco, h.vmd, bpp_of(h.spshift, h.stpal, h.s8260), h.stpal, h.t[0]);
		nfail += check_fields(h.name, h.monitor, h.t, h.lwd, h.vmd, h.spshift, h.stpal,
		                      h.s8260, base, D, nullptr, h.vco) ? 1 : 0;
	}
	check(nfail == 0, "all %zu register sets: DE size = Hatari XSize x YSize, period, pixels, no underrun (%d sets failing)",
	      sizeof(hsets) / sizeof(hsets[0]), nfail);
	mem.fixed_worst = false;
}

// the demo's 640x240 true colour set: DE position [HBE, HBB) and the
// visible line
static void test_beers_tc640()
{
	printf("\n== 124 Beers Later 640x240 true colour: DE position ==\n");
	const HSet *h = nullptr;
	for (const HSet &x : hsets) if (!strcmp(x.name, "beers_tc640")) h = &x;
	do_reset(h->monitor);
	mon.hs_pol = mon.vs_pol = false;
	program_set(h->t, h->lwd, h->vmd, h->vco, h->spshift, -1, 0x100000);
	mon.capture = true;
	run_frames(1, 6000000);
	run_frames(1, 6000000);
	const FrameRec &f = mon.frames.back();
	// DE begins HBE dots after the hsync end = line start: (HBE+1)... the
	// line record counts pixel clocks (1 dot) from the hsync start (HSS,
	// second half): HSS -> line end = 800 - (400 + 0x12E) = 98 dots
	int exp_pos = 98 + 0x2D;
	int bad = 0;
	for (int p : f.run_pos) if (p != exp_pos) bad++;
	check(f.run_pos.size() == 480 && bad == 0, "DE starts %d dots after hsync start on all %zu lines (expected %d = 98 + HBE)",
	      f.run_pos.size() ? f.run_pos[0] : -1, f.run_pos.size(), exp_pos);
	check(f.vrun_min == 640 && f.vrun_max == 640 && f.vruns == 480,
	      "visible line %d..%d px x %d lines = HBB - HBE = 640 (DE inside the blank window)", f.vrun_min, f.vrun_max, f.vruns);
	check(f.hs_ce == 98, "hsync width %d dots, expected 98", f.hs_ce);
}

// $8260 written while a Falcon mode is active (Hatari: only $8210, $82C2
// and the palette change): the line/frame timing stays, the picture is the
// ST resolution with the ST palette
static void test_8260_from_falcon()
{
	printf("\n== $8260 written in a Falcon mode ==\n");
	// TOS 4.04 register sets (EmuTOS tables, see the header)
	static const Mode rgb320x16 = { "RGB 320x200x16 (Vsetmode $0022)", MON_RGB, 0x0022,
	  {0x00fe,0x00cb,0x0027,0x000c,0x006d,0x00d8, 0x0271,0x0265,0x002f,0x007f,0x020f,0x026b},
	  0x0181, 0x0000, 0x0050, 0x0000, -1, 2048, 32000000, 626, 320, 200, 4, false, 0, 0 };
	static const Mode rgb640x16 = { "RGB 640x200x16 (Vsetmode $002A)", MON_RGB, 0x002A,
	  {0x01fe,0x0199,0x0050,0x004d,0x00fe,0x01b2, 0x0271,0x0265,0x002f,0x007f,0x020f,0x026b},
	  0x0181, 0x0004, 0x00A0, 0x0000, -1, 2048, 32000000, 626, 640, 200, 4, false, 0, 0 };
	// true colour stays true colour: Hatari keeps $8266 and bpp from it
	// takes priority (VIDEL_getScreenBpp), so the width is $8210 = 80 px
	const Mode *list[5] = {&rgb320x16, &modes[0], &rgb640x16, &modes[5], &modes[8]};
	for (const Mode *mp : list) {
		const Mode &m = *mp;
		for (int st = 0; st <= 2; st++) {
			do_reset(m.monitor);
			mon.hs_pol = mon.vs_pol = false;
			uint32_t base = 0x100000;
			fill_ram(base, 700000, 700 + st);
			set_palettes(800 + st);
			program_mode(m, base);
			run_frames(2, 6000000);
			double before = (double)mon.frames.back().clocks;
			wb(0xFF8260, st);
			uint16_t lwd = rw(0xFF8210), vmd = rw(0xFF82C2);
			char name[96];
			snprintf(name, sizeof name, "%s + $8260=%d ($8210=%04x $82C2=%04x)", m.name, st, lwd, vmd);
			double after = 0;
			// the unit stays the one of the Falcon mode
			int D = unit_of(m.monitor, m.vco, m.vmd, m.bpp, 0, m.t[0]);
			check_fields(name, m.monitor, m.t, lwd, vmd, m.spshift, 1, st, base, D, &after, m.vco);
			check(std::fabs(after - before) <= 1.0, "%s: field period %.0f clocks unchanged (was %.0f)", name, after, before);
		}
	}
}

// VBL exactly once per field whatever VBB/VBE/VDE say
static void test_vbl_edge()
{
	printf("\n== VBL without a vertical blank edge ==\n");
	struct { const char *name; int mon; uint16_t t[12]; uint16_t lwd, vco, vmd; int st; } sets[] = {
		{ "RGB 320x200x16, VBE=0 VBB=$300 (> VFT)", MON_RGB,
		  {0x00fe,0x00cb,0x0027,0x000c,0x006d,0x00d8, 0x0271,0x0300,0x0000,0x007f,0x020f,0x026b}, 0x50, 0x181, 0, -1 },
		{ "RGB 320x200x16, VBB=VBE=0", MON_RGB,
		  {0x00fe,0x00cb,0x0027,0x000c,0x006d,0x00d8, 0x0271,0x0000,0x0000,0x007f,0x020f,0x026b}, 0x50, 0x181, 0, -1 },
		{ "RGB 320x200x16, VBE=$10 VBB=$300", MON_RGB,
		  {0x00fe,0x00cb,0x0027,0x000c,0x006d,0x00d8, 0x0271,0x0300,0x0010,0x007f,0x020f,0x026b}, 0x50, 0x181, 0, -1 },
		{ "SM124, VDE=$3FF (> VFT)", MON_MONO,
		  {0x001a,0x0000,0x0000,0x020f,0x000c,0x0014, 0x03e9,0x0000,0x0000,0x0043,0x03ff,0x03e7}, 0x28, 0x080, 0, 2 },
	};
	for (auto &s : sets) {
		do_reset(s.mon);
		mon.hs_pol = mon.vs_pol = false;
		program_set(s.t, s.lwd, s.vmd, s.vco, 0, s.st, 0x100000);
		run_frames(1, 6000000);
		run_frames(4, 24000000);
		int bad = 0;
		for (size_t k = mon.frames.size() - 4; k < mon.frames.size(); k++) if (mon.frames[k].vbl != 1) bad++;
		check(bad == 0 && mon.frames.size() >= 5, "%s: one VBL per field over 4 fields (%d fields wrong)", s.name, bad);
	}
}

// 124 Beers Later curtain scene (320x240 true colour, VDB $3F, VDE $3FF,
// VFT $419): per frame VBB/VBE pairs cut the picture vertically, VBE
// 'negative' (bit 10 set) leaves the top fully open; HBE/HBB cut it
// horizontally.  Blanked pixels are black, DE and Timer B run the full
// window.
static void test_curtain()
{
	printf("\n== 124 Beers Later curtain: VBB/VBE and HBE/HBB cut the picture ==\n");
	const HSet *h = nullptr;
	for (const HSet &x : hsets) if (!strcmp(x.name, "beers_tc320v1")) h = &x;
	struct { uint16_t vbb, vbe; const char *what; } vs[] = {
		{ 0x243, 0x4D, "closing" }, { 0x1D9, 0xB7, "nearly closed" }, { 0x239, 0x57, "opening" },
		{ 0x3CD, 0xFEC3, "open past the top, bottom cut" }, { 0x3FF, 0xFE91, "fully open" },
	};
	int bpp = 16, cyc = cyc_of(h->monitor, h->vco, h->vmd, bpp), D = unit_of(h->monitor, h->vco, h->vmd, bpp, 0, h->t[0]);
	for (auto &c : vs) {
		do_reset(h->monitor);
		mon.hs_pol = mon.vs_pol = false;
		uint32_t base = 0x100000;
		fill_ram(base, 400000, 4242);
		set_palettes(4343);
		uint16_t t[12]; memcpy(t, h->t, sizeof t);
		t[7] = c.vbb; t[8] = c.vbe;
		program_set(t, h->lwd, h->vmd, h->vco, h->spshift, -1, base);
		mon.capture = true;
		run_frames(1, 6000000);
		run_frames(1, 6000000);
		const FrameRec &f = mon.frames.back();
		// expected: display line k is in the output line starting at VFC
		// VDB+1+2k (HDB in the second half line, DE in the next line)
		int Lx, raw = de_start_raw(t, bpp, 0, cyc, D, h->vco, h->vmd, &Lx);
		int wrap = raw >= Lx;
		int del, vis; expect_lines(t, wrap, 0, &del, &vis);
		int vbe_s = (c.vbe & 0x400) ? (int)(c.vbe & 0x7ff) - 0x800 : (c.vbe & 0x7ff);
		int first = -1, last = -1, partial = 0;
		for (size_t j = 0; j < f.msk.size(); j++) {
			int nv = 0; for (uint8_t v : f.msk[j]) nv += v;
			if (nv && first < 0) first = j;
			if (nv) last = j;
		}
		int v0 = (t[9] & 0x7ff) + 1;     // VFC at the start of display line 0's output line
		int exp_first = 0; while (exp_first < del && v0 + 2 * exp_first < vbe_s) exp_first++;
		int exp_last = del - 1; while (exp_last >= 0 && v0 + 2 * exp_last >= (int)c.vbb) exp_last--;
		// horizontally the set's HDB ($2BA, 14 past TOS's $2AC) puts the DE
		// 28 dots right of HBE, so the last 14 pixels of every line are
		// past HBB: a visible line shows hvis pixels, a blanked line none
		int deon = raw % Lx, H = (t[0] & 0x1ff) + 2;
		int pos_hbe = t[2] * D, pos_hbb = H * D + t[1] * D;
		int hf = 0; while (hf < 320 && deon + cyc * hf < pos_hbe) hf++;
		int hl = 319; while (hl >= 0 && deon + cyc * hl >= pos_hbb) hl--;
		int hvis = hl - hf + 1;
		for (size_t j = 0; j < f.msk.size(); j++) {
			int nv = 0; for (uint8_t v : f.msk[j]) nv += v;
			int exp_nv = ((int)j >= exp_first && (int)j <= exp_last) ? hvis : 0;
			if (nv != exp_nv) partial++;
		}
		int checked = 0;
		Mode m = {}; m.lwd = h->lwd; m.bpp = 16; m.dbl = 1;
		int errs = compare_image(m, f, base, 0, 0, &checked);
		check(f.de_lines == del && f.tb_falls == 240 && first == exp_first && last == exp_last && partial == 0 &&
		      f.vis_de_lines == vis && errs == 0,
		      "VBB=$%03X VBE=$%04X (%s): %d DE lines, Timer B %d, display lines %d..%d visible (expected %d..%d = VFC %d..%d, %d lines, %d px each), %d lines with another count, %d px (%d blanked, %d errors)",
		      c.vbb, c.vbe, c.what, f.de_lines, f.tb_falls, first, last, exp_first, exp_last,
		      v0 + 2 * exp_first, v0 + 2 * exp_last, vis, hvis, partial, checked, g_blanked, errs);
	}
	// horizontal curtain: HBE/HBB sweep midpoint $53/$53 vs TOS $15/$8D
	struct { uint16_t hbe, hbb; } hs2[] = { {0x15, 0x8D}, {0x53, 0x53}, {0x18, 0x96} };
	for (auto &c : hs2) {
		do_reset(h->monitor);
		mon.hs_pol = mon.vs_pol = false;
		uint32_t base = 0x100000;
		fill_ram(base, 400000, 4545);
		set_palettes(4646);
		uint16_t t[12]; memcpy(t, h->t, sizeof t);
		t[2] = c.hbe; t[1] = c.hbb;
		program_set(t, h->lwd, h->vmd, h->vco, h->spshift, -1, base);
		mon.capture = true;
		run_frames(1, 6000000);
		run_frames(1, 6000000);
		const FrameRec &f = mon.frames.back();
		int Lx, raw = de_start_raw(t, bpp, 0, cyc, D, h->vco, h->vmd, &Lx);
		int deon = raw % Lx;                       // DE start dot (register positions)
		int H = (t[0] & 0x1ff) + 2;
		int pos_hbe = c.hbe * D, pos_hbb = H * D + c.hbb * D;
		// pixel i is at dot deon + cyc*i; visible when pos_hbe <= dot < pos_hbb
		int exp_first = 0; while (exp_first < 320 && deon + cyc * exp_first < pos_hbe) exp_first++;
		int exp_last = 319; while (exp_last >= 0 && deon + cyc * exp_last >= pos_hbb) exp_last--;
		int bad = 0, first = -1, last = -1;
		for (size_t j = 0; j < f.msk.size(); j++) {
			int fi = -1, la = -1;
			for (size_t x = 0; x < f.msk[j].size(); x++) if (f.msk[j][x]) { if (fi < 0) fi = x; la = x; }
			if (j == 0) { first = fi; last = la; }
			if (fi != exp_first || la != exp_last) bad++;
		}
		int checked = 0;
		Mode m = {}; m.lwd = h->lwd; m.bpp = 16; m.dbl = 1;
		int errs = compare_image(m, f, base, 0, 0, &checked);
		check(f.de_lines == 480 && f.min_de == 320 && f.max_de == 320 && bad == 0 && errs == 0,
		      "HBE=$%02X HBB=$%02X: DE %d px x %d lines, visible pixels %d..%d on all lines (expected %d..%d from dots %d..%d, DE from %d), %d lines off, %d px (%d blanked, %d errors)",
		      c.hbe, c.hbb, f.min_de, f.de_lines, first, last, exp_first, exp_last, pos_hbe, pos_hbb - 1, deon, bad, checked, g_blanked, errs);
	}
}

int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	top = new Vfalcon_videl;
	top->clk = 0;
	top->eval();

	test_registers();
	test_palettes();
	for (unsigned i = 0; i < sizeof(modes) / sizeof(modes[0]); i++)
		test_mode(modes[i]);
	test_mode(modes[8], true);      // RGB 640x200 TC, every burst 40 clocks
	test_mode(modes[2], true);      // VGA 640x480x256, every burst 40 clocks
	test_counters();
	test_1600x600(false);           // \"1600 x 600\" demo mode, random latency
	test_1600x600(true);            // the same, every burst 40 clocks
	test_hscroll(0, 3, 0);          // VGA 640x480x16
	test_hscroll(0, 11, 2);         // VGA 640x480x16, colour bank 2
	test_hscroll(2, 5, 0);          // VGA 640x480x256
	test_hscroll(3, 7, 0);          // VGA ST low
	test_hscroll(5, 3, 0);          // RGB 320x200 TC (no shift, stride + 16 words)
	test_latching();
	test_sync_stability();
	test_polarity();
	test_timerb();
	test_vbl_edge();
	test_8260_from_falcon();
	test_hatari_sets(false);
	test_hatari_sets(true);
	test_beers_tc640();
	test_curtain();

	printf("\n%d checks passed, %d failed\n", n_pass, n_fail);
	printf("%s\n", n_fail ? "FAIL" : "PASS");
	delete top;
	return n_fail ? 1 : 0;
}
