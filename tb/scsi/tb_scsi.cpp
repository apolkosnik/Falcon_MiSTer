// tb_scsi.cpp - Verilator testbench for the Falcon SCSI port:
// rtl/falcon/falcon_fdc.sv (the ST DMA chip, EXT_SCSI = 1) +
// rtl/falcon/falcon_scsi.sv (NCR 5380 and the SCSI targets), driven through
// the register bus at $FF8604/$FF8606/$FF8609.. as TOS / HDDRIVER do; the
// DMA port is served by a RAM model with variable latency; the hps_io block
// interface by an HPS protocol model whose blocks come from the real
// Main_MiSTer support/falcon/falcon_scsi.cpp (Falcon Main) through minimal
// shims of file_io / spi / user_io, or from the user_io generic path alone
// (stock Main: is_falcon() false, the hooks do nothing).
//
// Expected values: NCR 5380 register layout and phase rules of Hatari
// src/ncr5380.c (ncr5380_bget/bput, raw_scsi_set_signal_phase,
// scsicmdsizes), the 5380 datasheet for the bus lines and interrupts,
// Hatari src/hdc.c / hdc.h for the target (sense codes $20 opcode, $21
// address, $24 argument, $25 LUN; REQUEST SENSE formats; INQUIRY of an
// unsupported LUN = $7F; READ CAPACITY = blocks - 1 and the block size),
// SCSI-2 for NOT READY ($02/$3A) and DATA PROTECT ($07/$27), and the
// window protocol of falcon_scsi.sv / support/falcon/falcon_scsi.h.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include <sys/stat.h>
#include "Vtb_scsi_top.h"
#include "verilated.h"
#include "main_shim.h"
#include "hps_main_model.h"
#include "support/falcon/falcon_scsi.h"

static Vtb_scsi_top *dut;
static MainSim main_sim;
static HpsMain *hps;
static uint64_t cycle = 0;
static int fails = 0, checks = 0;

#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define CHECKEQ(got, exp, what) do { checks++; if ((uint64_t)(got) != (uint64_t)(exp)) { fails++; \
	printf("FAIL: %s: got 0x%llx expected 0x%llx\n", what, (unsigned long long)(got), (unsigned long long)(exp)); } \
	else printf("  ok: %s = 0x%llx\n", what, (unsigned long long)(exp)); } while (0)

static void section(const char *s) { printf("--- %s  (t=%.2f ms)\n", s, cycle * 1000.0 / 32e6); }

// ---------------------------------------------------------------------------
// RAM model on the DMA port
static std::vector<uint8_t> ram(16 << 20);
static int dma_lat = 0;
static bool dma_busy = false;
static uint32_t rng = 777;
static int rnd(int lo, int hi) { rng = rng * 1103515245u + 12345u; return lo + (int)((rng >> 8) % (uint32_t)(hi - lo + 1)); }

static uint32_t mnt_pulse = 0;
static uint64_t mnt_size = 0;
static bool mnt_ro = false;

static void tick() {
	if (dut->sd_lba0 != dut->sd_lba1 || dut->sd_lba0 != dut->sd_lba2 ||
	    dut->sd_buff_din0 != dut->sd_buff_din1 || dut->sd_buff_din0 != dut->sd_buff_din2) {
		printf("FAIL: sd_lba0..2 or sd_buff_din0..2 differ\n"); fails++;
	}
	hps->step(dut->sd_rd, dut->sd_wr, dut->sd_lba0, dut->sd_buff_din0, dut->sd_blk_cnt);
	bool ack = false;
	uint16_t rdata = 0;
	if (dut->dma_req && !dma_busy) { dma_busy = true; dma_lat = rnd(6, 20); }
	if (dma_busy && --dma_lat <= 0) {
		uint32_t a = (dut->dma_addr << 1) & 0xFFFFFE;
		if (dut->dma_we) {
			if (dut->dma_be & 2) ram[a] = dut->dma_wdata >> 8;
			if (dut->dma_be & 1) ram[a + 1] = dut->dma_wdata & 0xFF;
		} else rdata = (ram[a] << 8) | ram[a + 1];
		ack = true;
		dma_busy = false;
	}
	dut->clk = 1;
	dut->eval();
	cycle++;
	dut->dma_ack = ack;
	dut->dma_rdata = rdata;
	dut->sd_ack = hps->sd_ack;
	dut->sd_buff_addr = hps->sd_buff_addr;
	dut->sd_buff_dout = hps->sd_buff_dout;
	dut->sd_buff_wr = hps->sd_buff_wr;
	dut->img_mounted = mnt_pulse;
	dut->img_size = mnt_size;
	dut->img_readonly = mnt_ro;
	mnt_pulse = 0;
	dut->clk = 0;
	dut->eval();
}
static void ticks(uint64_t n) { while (n--) tick(); }

static uint16_t bus(bool we, uint32_t byteaddr, bool uds, bool lds, uint16_t din) {
	dut->bus_cs = 1;
	dut->bus_stb = 1;
	dut->bus_we = we;
	dut->bus_addr = (byteaddr >> 1) & 7;
	dut->bus_uds = uds;
	dut->bus_lds = lds;
	dut->bus_din = din;
	int n = 0;
	uint16_t r = 0;
	for (;;) {
		tick();
		dut->bus_stb = 0;
		if (dut->bus_ack) { r = dut->bus_dout; break; }
		if (++n > 1000) { printf("FAIL: bus timeout at %06x\n", byteaddr); fails++; break; }
	}
	dut->bus_cs = 0;
	dut->bus_we = 0;
	tick();
	return r;
}
static uint16_t rw(uint32_t a) { return bus(false, a, true, true, 0); }
static void ww(uint32_t a, uint16_t v) { bus(true, a, true, true, v); }
static uint8_t rb(uint32_t a) { uint16_t v = bus(false, a, !(a & 1), a & 1, 0); return (a & 1) ? v & 0xFF : v >> 8; }
static void wb(uint32_t a, uint8_t v) { bus(true, a, !(a & 1), a & 1, (a & 1) ? v : (uint16_t)(v << 8)); }

enum { DMA_DATA = 0xFF8604, DMA_MODE = 0xFF8606, DMA_HI = 0xFF8609, DMA_MID = 0xFF860B, DMA_LO = 0xFF860D };

static void set_dma_addr(uint32_t a) { wb(DMA_LO, a & 0xFF); wb(DMA_MID, (a >> 8) & 0xFF); wb(DMA_HI, (a >> 16) & 0xFF); }
static uint32_t get_dma_addr() { return (rb(DMA_HI) << 16) | (rb(DMA_MID) << 8) | rb(DMA_LO); }

// NCR 5380 registers through the DMA chip: mode bit 3 = HDC, bits 2..0 = register
enum { R_DATA = 0, R_ICR = 1, R_MR = 2, R_TCR = 3, R_CSBSR = 4, R_BSR = 5, R_IDR = 6, R_RPI = 7 };
// the DMA direction bit (8) is kept while selecting registers: toggling it
// resets the FIFO and the sector count (FDC_ResetDMA)
static uint16_t dma_dir = 0;
static void ncr_wr(int reg, uint8_t v) { ww(DMA_MODE, dma_dir | 0x88 | reg); ww(DMA_DATA, v); }
static uint8_t ncr_rd(int reg) { ww(DMA_MODE, dma_dir | 0x88 | reg); return rw(DMA_DATA) & 0xFF; }

// ICR bits
enum { ICR_DBUS = 0x01, ICR_ATN = 0x02, ICR_SEL = 0x04, ICR_BSY = 0x08, ICR_ACK = 0x10, ICR_LA = 0x20, ICR_AIP = 0x40, ICR_RST = 0x80 };
// CSBSR bits
enum { CS_RST = 0x80, CS_BSY = 0x40, CS_REQ = 0x20, CS_SEL = 0x02 };
// BSR bits
enum { BS_EOD = 0x80, BS_DRQ = 0x40, BS_IRQ = 0x10, BS_PM = 0x08, BS_BERR = 0x04, BS_ATN = 0x02, BS_ACK = 0x01 };
// phases
enum { P_DO = 0, P_DI = 1, P_CMD = 2, P_ST = 3, P_MO = 6, P_MI = 7 };

// ---------------------------------------------------------------------------
// images
static const char *WORK = "work";
static std::string wpath(const char *n) { return std::string(WORK) + "/" + n; }
static uint8_t pat(uint32_t seed, uint64_t off) {
	uint64_t x = (uint64_t)seed * 0x9E3779B97F4A7C15ull + off * 0x2545F4914F6CDD1Dull;
	x ^= x >> 29; x *= 0xBF58476D1CE4E5B9ull; x ^= x >> 32;
	return (uint8_t)x;
}
static void make_file(const std::string &p, uint64_t size, uint32_t seed) {
	chmod(p.c_str(), 0644);
	FILE *f = fopen(p.c_str(), "wb");
	std::vector<uint8_t> b(size);
	for (uint64_t i = 0; i < size; i++) b[i] = pat(seed, i);
	fwrite(b.data(), 1, size, f);
	fclose(f);
}
static std::vector<uint8_t> read_file(const std::string &p, uint64_t off, size_t n) {
	std::vector<uint8_t> b(n, 0);
	FILE *f = fopen(p.c_str(), "rb");
	if (f) { fseeko(f, off, SEEK_SET); size_t r = fread(b.data(), 1, n, f); (void)r; fclose(f); }
	return b;
}

// raw CD frame 2352: sync, header (MSF + mode 1), user data, EDC/ECC filler
static void cd_frame(uint8_t *fr, uint32_t lba, uint32_t seed) {
	static const uint8_t sync[12] = { 0, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0 };
	memcpy(fr, sync, 12);
	uint32_t a = lba + 150;
	fr[12] = (uint8_t)(((a / 4500) / 10) << 4 | ((a / 4500) % 10));
	fr[13] = (uint8_t)((((a / 75) % 60) / 10) << 4 | (((a / 75) % 60) % 10));
	fr[14] = (uint8_t)(((a % 75) / 10) << 4 | ((a % 75) % 10));
	fr[15] = 1;
	for (int i = 0; i < 2048; i++) fr[16 + i] = pat(seed, (uint64_t)lba * 2048 + i);
	for (int i = 2064; i < 2352; i++) fr[i] = 0xEC;
}

// mount through the simulated Main (user_io_file_mount + hooks), then the
// img_mounted pulse to the core
static uint64_t mount(int unit, const std::string &path) {
	bool ro = false;
	uint64_t sz = main_sim.mount(4 + unit, path.c_str(), &ro);
	mnt_size = sz;
	mnt_ro = ro;
	mnt_pulse = 1u << unit;
	ticks(4);
	return sz;
}

// ---------------------------------------------------------------------------
// initiator (TOS-style NCR 5380 programming)
static bool wait_csbsr(uint8_t mask, uint8_t val, uint64_t max_clk) {
	uint64_t t0 = cycle;
	ww(DMA_MODE, dma_dir | 0x88 | R_CSBSR);
	while (cycle - t0 < max_clk) {
		if ((rw(DMA_DATA) & mask) == val) return true;
	}
	return false;
}

// arbitration + selection; returns true when the target answers with BSY
static bool select_target(int id, bool atn, bool arbitrate = true, uint64_t timeout = 20000) {
	ncr_wr(R_TCR, 0);
	ncr_wr(R_ICR, 0);
	ncr_wr(R_MR, 0);
	if (arbitrate) {
		ncr_wr(R_DATA, 0x80);                     // our ID 7
		ncr_wr(R_MR, 0x01);                       // arbitrate
		uint64_t t0 = cycle;
		while (!(ncr_rd(R_ICR) & ICR_AIP) && cycle - t0 < 2000) ;
		if (ncr_rd(R_ICR) & ICR_LA) { ncr_wr(R_MR, 0); return false; }
		ncr_wr(R_ICR, ICR_SEL | ICR_BSY | (atn ? ICR_ATN : 0));
		ncr_wr(R_DATA, 0x80 | (1 << id));
		ncr_wr(R_ICR, ICR_SEL | ICR_BSY | ICR_DBUS | (atn ? ICR_ATN : 0));
		ncr_wr(R_MR, 0);
		ncr_wr(R_ICR, ICR_SEL | ICR_DBUS | (atn ? ICR_ATN : 0));     // release BSY
	} else {
		ncr_wr(R_DATA, 0x80 | (1 << id));
		ncr_wr(R_ICR, ICR_SEL | ICR_DBUS | (atn ? ICR_ATN : 0));
	}
	bool ok = wait_csbsr(CS_BSY, CS_BSY, timeout);
	ncr_wr(R_ICR, atn ? ICR_ATN : 0);                // release SEL and the bus
	return ok;
}

static bool wait_req(uint64_t max_clk = 4000000) { return wait_csbsr(CS_REQ, CS_REQ, max_clk); }
static int cur_phase() { return (ncr_rd(R_CSBSR) >> 2) & 7; }

static bool pio_out(uint8_t b, uint8_t icr_extra = 0) {
	ncr_wr(R_DATA, b);
	ncr_wr(R_ICR, ICR_DBUS | icr_extra);
	ncr_wr(R_ICR, ICR_DBUS | ICR_ACK | icr_extra);
	bool ok = wait_csbsr(CS_REQ, 0, 100000);
	ncr_wr(R_ICR, icr_extra);
	return ok;
}
static int pio_in() {
	uint8_t b = ncr_rd(R_DATA);
	ncr_wr(R_ICR, ICR_ACK);
	bool ok = wait_csbsr(CS_REQ, 0, 100000);
	ncr_wr(R_ICR, 0);
	return ok ? b : -1;
}

struct Res {
	bool sel = false;
	int status = -1;
	int msg = -1;
	bool bus_free = false;
	std::vector<uint8_t> din;
	int phases_err = 0;
	uint32_t dma_end = 0;
	bool dma_irq = false;
	uint8_t bsr_after_dma = 0;
};

enum { NO_DMA = 0, DMA_IN = 1, DMA_OUT = 2 };

struct Cmd {
	int id = 0;
	std::vector<uint8_t> cdb;
	std::vector<uint8_t> dout;            // PIO data out
	std::vector<uint8_t> msgs;            // message out bytes (ATN selection) - identify etc.
	bool atn = false;
	bool arbitrate = true;
	int dma = NO_DMA;
	uint32_t dma_addr = 0;
	int dma_count = 0;
	size_t din_max = 1 << 20;
	bool arm_early = false;               // arm the DMA receive right after the last CDB byte
	uint64_t dma_timeout = 40000000;
};

// full command through the phase loop (PIO, or DMA for the data phase)
static Res run(const Cmd &c) {
	Res r;
	r.sel = select_target(c.id, c.atn, c.arbitrate);
	if (!r.sel) return r;
	size_t ci = 0, di = 0, mi = 0;
	bool dma_done = false;
	for (int guard = 0; guard < 400000; guard++) {
		uint64_t t0 = cycle;
		bool req = false;
		ww(DMA_MODE, dma_dir | 0x88 | R_CSBSR);
		while (cycle - t0 < 8000000) {
			uint8_t cs = rw(DMA_DATA) & 0xFF;
			if (!(cs & CS_BSY)) { r.bus_free = true; return r; }
			if (cs & CS_REQ) { req = true; break; }
		}
		if (!req) { printf("  (REQ timeout)\n"); r.phases_err++; return r; }
		int ph = cur_phase();
		ncr_wr(R_TCR, ph);
		switch (ph) {
		case P_MO: {
			bool last = (mi + 1 >= c.msgs.size());
			uint8_t b = mi < c.msgs.size() ? c.msgs[mi] : 0x08;   // NOP
			mi++;
			pio_out(b, last ? 0 : ICR_ATN);
			break;
		}
		case P_CMD:
			if (ci >= c.cdb.size()) { r.phases_err++; pio_out(0); break; }
			pio_out(c.cdb[ci++]);
			if (ci == c.cdb.size() && c.arm_early && c.dma == DMA_IN && !dma_done) {
				// driver style: DMA armed for data in before the target moved on
				set_dma_addr(c.dma_addr);
				ww(DMA_MODE, 0x190); ww(DMA_MODE, 0x090);
				ww(DMA_DATA, c.dma_count);
				ncr_wr(R_TCR, P_DI);
				ncr_wr(R_MR, 0x02);
				ncr_wr(R_RPI, 0);
				ww(DMA_MODE, 0x000);
				uint64_t t1 = cycle;
				while (!dut->irq && cycle - t1 < c.dma_timeout) tick();
				r.dma_irq = dut->irq;
				ticks(320);                        // CPU interrupt latency (10 us)
				r.bsr_after_dma = ncr_rd(R_BSR);
				r.dma_end = get_dma_addr();
				ncr_wr(R_MR, 0);
				ncr_rd(R_RPI);
				dma_done = true;
				if (!r.dma_irq) return r;
			}
			break;
		case P_DO:
			if (c.dma == DMA_OUT && !dma_done) {
				set_dma_addr(c.dma_addr);
				ww(DMA_MODE, 0x090); ww(DMA_MODE, 0x190);
				ww(DMA_DATA, c.dma_count);
				dma_dir = 0x100;
				ncr_wr(R_ICR, ICR_DBUS);
				ncr_wr(R_MR, 0x02);                // DMA mode
				ncr_wr(R_BSR, 0);                  // start DMA send
				ww(DMA_MODE, 0x100);               // HDC, DMA on, RAM -> SCSI
				uint64_t t1 = cycle;
				while (!dut->irq && cycle - t1 < c.dma_timeout) tick();
				r.dma_irq = dut->irq;
				ticks(320);                        // CPU interrupt latency (10 us)
				r.bsr_after_dma = ncr_rd(R_BSR);
				r.dma_end = get_dma_addr();
				ncr_wr(R_MR, 0);
				ncr_wr(R_ICR, 0);
				ncr_rd(R_RPI);
				dma_done = true;
				dma_dir = 0;
				break;
			}
			pio_out(di < c.dout.size() ? c.dout[di] : 0);
			di++;
			break;
		case P_DI:
			if (c.dma == DMA_IN && !dma_done) {
				set_dma_addr(c.dma_addr);
				ww(DMA_MODE, 0x190); ww(DMA_MODE, 0x090);
				ww(DMA_DATA, c.dma_count);
				ncr_wr(R_MR, 0x02);                // DMA mode
				ncr_wr(R_RPI, 0);                  // start DMA initiator receive
				ww(DMA_MODE, 0x000);               // HDC, DMA on, SCSI -> RAM
				uint64_t t1 = cycle;
				while (!dut->irq && cycle - t1 < c.dma_timeout) tick();
				r.dma_irq = dut->irq;
				ticks(320);                        // CPU interrupt latency (10 us)
				r.bsr_after_dma = ncr_rd(R_BSR);
				r.dma_end = get_dma_addr();
				ncr_wr(R_MR, 0);
				ncr_rd(R_RPI);
				dma_done = true;
				if (!r.dma_irq) return r;          // caller handles a stopped transfer
				break;
			}
			{
				int b = pio_in();
				if (b < 0) { r.phases_err++; return r; }
				if (r.din.size() < c.din_max) r.din.push_back((uint8_t)b);
			}
			break;
		case P_ST: r.status = pio_in(); break;
		case P_MI: r.msg = pio_in(); break;
		default: r.phases_err++; return r;
		}
	}
	return r;
}

static Cmd cmd6(int id, uint8_t op, uint8_t b1, uint8_t b2, uint8_t b3, uint8_t b4, uint8_t b5 = 0) {
	Cmd c; c.id = id; c.cdb = { op, b1, b2, b3, b4, b5 }; return c;
}
static Cmd cmd10(int id, uint8_t op, uint8_t b1, uint32_t lba, uint16_t len, uint8_t b6 = 0, uint8_t b9 = 0) {
	Cmd c; c.id = id;
	c.cdb = { op, b1, (uint8_t)(lba >> 24), (uint8_t)(lba >> 16), (uint8_t)(lba >> 8), (uint8_t)lba, b6,
	          (uint8_t)(len >> 8), (uint8_t)len, b9 };
	return c;
}

static void expect_good(const Res &r, const char *what) {
	char s[256];
	CHECK(r.sel, "%s: target answers selection", what);
	snprintf(s, sizeof s, "%s: status GOOD", what); CHECKEQ(r.status, 0x00, s);
	snprintf(s, sizeof s, "%s: message COMMAND COMPLETE", what); CHECKEQ(r.msg, 0x00, s);
	CHECK(r.bus_free && r.phases_err == 0, "%s: bus free at the end, no phase error", what);
}
static void expect_check(const Res &r, const char *what) {
	char s[256];
	CHECK(r.sel, "%s: target answers selection", what);
	snprintf(s, sizeof s, "%s: status CHECK CONDITION", what); CHECKEQ(r.status, 0x02, s);
	CHECK(r.bus_free && r.phases_err == 0, "%s: bus free at the end, no phase error", what);
}
// REQUEST SENSE (extended, 18 bytes): returns key << 8 | asc
static int sense(int id, uint8_t lun = 0) {
	Res r = run(cmd6(id, 0x03, lun << 5, 0, 0, 18));
	if (r.status != 0 || r.din.size() != 18) return -1;
	return ((r.din[2] & 0x0F) << 8) | r.din[12];
}
static std::string str(const std::vector<uint8_t> &d, int off, int n) {
	std::string s;
	for (int i = off; i < off + n && i < (int)d.size(); i++) s += (char)d[i];
	return s;
}
static uint32_t be32(const std::vector<uint8_t> &d, int o) { return (d[o] << 24) | (d[o + 1] << 16) | (d[o + 2] << 8) | d[o + 3]; }

// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
	Verilated::commandArgs(argc, argv);
	dut = new Vtb_scsi_top;
	hps = new HpsMain(&main_sim, 4, 3);
	mkdir(WORK, 0755);

	// test images
	const uint64_t HD0_SECT = 4096, HD1_SECT = 2048, ISO_SECT = 64;
	make_file(wpath("hd0.img"), HD0_SECT * 512, 11);
	make_file(wpath("hd1.img"), HD1_SECT * 512, 22);
	chmod(wpath("hd1.img").c_str(), 0444);                      // read only
	make_file(wpath("cd.iso"), ISO_SECT * 2048, 33);
	// cue/bin: track 1 MODE1/2352 (30 frames) in cd2_data.bin, track 2 AUDIO
	// in cd2_audio.bin with a 2 s pregap (INDEX 00 at 0, INDEX 01 at 00:02:00)
	{
		FILE *f = fopen(wpath("cd2_data.bin").c_str(), "wb");
		uint8_t fr[2352];
		for (uint32_t l = 0; l < 30; l++) { cd_frame(fr, l, 44); fwrite(fr, 1, 2352, f); }
		fclose(f);
		f = fopen(wpath("cd2_audio.bin").c_str(), "wb");
		for (uint32_t l = 0; l < 200; l++) { for (int i = 0; i < 2352; i++) fr[i] = (uint8_t)(l + i); fwrite(fr, 1, 2352, f); }
		fclose(f);
		f = fopen(wpath("cd2.cue").c_str(), "w");
		fprintf(f, "FILE \"cd2_data.bin\" BINARY\n  TRACK 01 MODE1/2352\n    INDEX 01 00:00:00\n"
		           "FILE \"cd2_audio.bin\" BINARY\n  TRACK 02 AUDIO\n    INDEX 00 00:00:00\n    INDEX 01 00:02:00\n");
		fclose(f);
		// raw MODE1/2352 .bin without a cue (12 frames)
		f = fopen(wpath("cd3.bin").c_str(), "wb");
		for (uint32_t l = 0; l < 12; l++) { cd_frame(fr, l, 55); fwrite(fr, 1, 2352, f); }
		fclose(f);
	}

	dut->reset = 1;
	ticks(10);
	dut->reset = 0;
	ticks(10);

	// ------------------------------------------------------------------
	section("5380 registers after reset (ncr5380_bget)");
	{
		CHECKEQ(ncr_rd(R_ICR), 0x00, "ICR");
		CHECKEQ(ncr_rd(R_MR), 0x00, "MR");
		CHECKEQ(ncr_rd(R_TCR), 0x00, "TCR");
		CHECKEQ(ncr_rd(R_CSBSR), 0x00, "current SCSI bus status (bus free)");
		CHECKEQ(ncr_rd(R_BSR) & ~BS_PM, 0x00, "bus and status: no DRQ, no IRQ, no busy error");
		ncr_wr(R_TCR, 0x03);
		CHECKEQ(ncr_rd(R_TCR), 0x03, "TCR reads back");
		CHECKEQ(ncr_rd(R_BSR) & BS_PM, 0, "phase match off: bus lines 000 vs TCR 011");
		ncr_wr(R_TCR, 0x00);
		CHECKEQ(ncr_rd(R_BSR) & BS_PM, BS_PM, "phase match: bus lines 000 = TCR 000 (datasheet)");
		ncr_wr(R_ICR, ICR_ATN | ICR_DBUS);
		CHECKEQ(ncr_rd(R_ICR), ICR_ATN | ICR_DBUS, "ICR bits 4..0 read back");
		CHECKEQ(ncr_rd(R_BSR) & BS_ATN, BS_ATN, "BSR ATN follows ICR ATN");
		ncr_wr(R_DATA, 0x5A);
		CHECKEQ(ncr_rd(R_DATA), 0x5A, "data bus driven (ICR bit 0): current data = ODR");
		ncr_wr(R_ICR, 0);
		CHECKEQ(ncr_rd(R_DATA), 0x00, "data bus released: current data 0");
		CHECKEQ(dut->irq, 0, "no interrupt");
	}

	// ------------------------------------------------------------------
	section("arbitration (MR arbitrate: AIP, data bus, BSY)");
	{
		ncr_wr(R_DATA, 0x80);
		ncr_wr(R_MR, 0x01);
		uint8_t icr = ncr_rd(R_ICR);
		CHECKEQ(icr & (ICR_AIP | ICR_LA), ICR_AIP, "AIP set, LA clear");
		CHECKEQ(ncr_rd(R_DATA), 0x80, "data bus during arbitration = our ID");
		CHECKEQ(ncr_rd(R_CSBSR) & CS_BSY, CS_BSY, "BSY asserted by the arbitrating 5380");
		ncr_wr(R_MR, 0x00);
		CHECKEQ(ncr_rd(R_ICR) & ICR_AIP, 0, "AIP cleared with the arbitrate bit");
		ncr_wr(R_ICR, ICR_RST);                        // back to bus free
		ncr_wr(R_ICR, 0);
		ncr_rd(R_RPI);
	}

	// ------------------------------------------------------------------
	section("selection timeouts: no image mounted, IDs 0..7");
	{
		for (int id = 0; id < 7; id++) {
			if (id == 2) continue;                     // the CD-ROM answers with no medium
			bool ok = select_target(id, false, true, 20000);
			CHECK(!ok, "ID %d: BSY never asserted, selection times out", id);
			CHECKEQ(ncr_rd(R_CSBSR) & (CS_BSY | CS_REQ | CS_SEL), 0, "bus free after SEL released");
		}
		bool ok = select_target(1, false, false, 20000);
		CHECK(!ok, "ID 1 without arbitration: selection times out");
		CHECKEQ(dut->irq, 0, "no interrupt from a selection timeout");
	}

	// ------------------------------------------------------------------
	section("CD-ROM with no medium (always on the bus)");
	{
		Res r = run(cmd6(2, 0x00, 0, 0, 0, 0));
		expect_check(r, "TEST UNIT READY, no medium");
		CHECKEQ(sense(2), 0x23A, "sense NOT READY / medium not present");
		r = run(cmd6(2, 0x12, 0, 0, 0, 36));
		expect_good(r, "INQUIRY, no medium");
		CHECKEQ(r.din.size(), 36, "INQUIRY length");
		if (r.din.size() == 36) {
			CHECKEQ(r.din[0], 0x05, "INQUIRY peripheral type CD-ROM");
			CHECKEQ(r.din[1], 0x80, "INQUIRY removable");
			CHECK(str(r.din, 16, 16) == "Falcon SCSI CD  ", "INQUIRY product from Main: '%s'", str(r.din, 16, 16).c_str());
		}
		r = run(cmd10(2, 0x25, 0, 0, 0));
		expect_check(r, "READ CAPACITY, no medium");
	}

	// ------------------------------------------------------------------
	section("hard disk 0 mounted (Falcon Main): selection, messages, TEST UNIT READY");
	{
		uint64_t sz = mount(0, wpath("hd0.img"));
		CHECKEQ(sz, HD0_SECT * 512, "image size given to the core");
		Res r = run(cmd6(0, 0x00, 0, 0, 0, 0));
		expect_good(r, "TEST UNIT READY (arbitration, no ATN)");
		Cmd c = cmd6(0, 0x00, 0, 0, 0, 0);
		c.arbitrate = false;
		r = run(c);
		expect_good(r, "TEST UNIT READY (selection without arbitration)");
		c = cmd6(0, 0x00, 0, 0, 0, 0);
		c.atn = true; c.msgs = { 0xC0 };               // identify, disconnect allowed, LUN 0
		r = run(c);
		expect_good(r, "TEST UNIT READY after ATN + IDENTIFY");
		c.msgs = { 0xC0, 0x01, 0x03, 0x01, 0x19, 0x08 };  // identify + extended SDTR (5 bytes)
		r = run(c);
		expect_good(r, "TEST UNIT READY after IDENTIFY + extended message");
		c.msgs = { 0x81 };                              // identify LUN 1
		r = run(c);
		expect_check(r, "TEST UNIT READY on LUN 1 (identify)");
		CHECKEQ(sense(0), 0x525, "sense: invalid LUN ($25, hdc.h HD_REQSENS_INVLUN)");
		r = run(cmd6(0, 0x00, 0x20, 0, 0, 0));          // LUN 1 in the CDB
		expect_check(r, "TEST UNIT READY on LUN 1 (CDB)");
		r = run(cmd6(0, 0x03, 0x20, 0, 0, 18));         // REQUEST SENSE on LUN 1 is still handled
		expect_good(r, "REQUEST SENSE on LUN 1");
		if (r.din.size() == 18) { CHECKEQ(r.din[12], 0x25, "REQUEST SENSE LUN 1 reports the invalid LUN"); }
		CHECKEQ(dut->irq, 0, "no interrupt in PIO mode with MR = 0");
	}

	// ------------------------------------------------------------------
	section("INQUIRY / READ CAPACITY / MODE SENSE from the Main window (hard disk)");
	{
		hps->clear_log();
		Res r = run(cmd6(0, 0x12, 0, 0, 0, 36));
		expect_good(r, "INQUIRY");
		CHECK(hps->lba_log[0].size() == 1 && hps->lba_log[0][0] == 0x7E001200u,
		      "INQUIRY window LBA 0x7E001200 on slot 0 (got %08x)", hps->lba_log[0].empty() ? 0 : hps->lba_log[0][0]);
		if (r.din.size() == 36) {
			CHECKEQ(r.din[0], 0x00, "INQUIRY direct access device");
			CHECKEQ(r.din[4], 31, "INQUIRY additional length");
			CHECK(str(r.din, 8, 8) == "MiSTer  ", "INQUIRY vendor '%s'", str(r.din, 8, 8).c_str());
			CHECK(str(r.din, 16, 16) == "Falcon SCSI HD  ", "INQUIRY product '%s'", str(r.din, 16, 16).c_str());
			CHECK(str(r.din, 32, 4) == "M1.0", "INQUIRY revision from Main '%s'", str(r.din, 32, 4).c_str());
		}
		r = run(cmd6(0, 0x12, 0, 0, 0, 5));
		expect_good(r, "INQUIRY allocation length 5");
		CHECKEQ(r.din.size(), 5, "INQUIRY truncated to the allocation length");
		r = run(cmd6(0, 0x12, 0x20, 0, 0, 36));
		expect_good(r, "INQUIRY LUN 1");
		if (!r.din.empty()) { CHECKEQ(r.din[0], 0x7F, "INQUIRY LUN 1: qualifier/type $7F (HDC_Cmd_Inquiry)"); }
		r = run(cmd6(0, 0x12, 0x01, 0x80, 0, 255));
		expect_good(r, "INQUIRY EVPD page $80");
		CHECK(r.din.size() == 12 && str(r.din, 4, 8) == "FALCON00", "unit serial number page");
		r = run(cmd10(0, 0x25, 0, 0, 0));
		expect_good(r, "READ CAPACITY");
		CHECK(r.din.size() == 8, "READ CAPACITY 8 bytes");
		if (r.din.size() == 8) {
			CHECKEQ(be32(r.din, 0), HD0_SECT - 1, "READ CAPACITY last LBA = blocks - 1");
			CHECKEQ(be32(r.din, 4), 512, "READ CAPACITY block length");
		}
		r = run(cmd6(0, 0x1A, 0, 0x00, 0, 255));
		expect_good(r, "MODE SENSE(6) page 0 (Hatari vendor page)");
		if (r.din.size() == 16) {
			CHECKEQ(r.din[1], 14, "page 0 byte 1");
			CHECKEQ(r.din[3], 8, "page 0 byte 3");
			CHECKEQ((r.din[5] << 16) | (r.din[6] << 8) | r.din[7], HD0_SECT, "page 0 number of blocks");
			CHECKEQ(r.din[10], 2, "page 0 block size $200");
		} else CHECK(false, "MODE SENSE page 0 length %zu", r.din.size());
		r = run(cmd6(0, 0x1A, 0, 0x04, 0, 255));
		expect_good(r, "MODE SENSE(6) page 4");
		CHECK(r.din.size() == 4 + 8 + 24, "page 4 length %zu", r.din.size());
		if (r.din.size() == 36) {
			CHECKEQ(r.din[0], 35, "mode data length");
			CHECKEQ(r.din[3], 8, "block descriptor length");
			CHECKEQ(r.din[12], 0x04, "page code 4");
			CHECKEQ(r.din[17], 128, "heads (HDC_CmdModeSense0x04)");
		}
		r = run(cmd6(0, 0x1A, 0x08, 0x3F, 0, 255));
		expect_good(r, "MODE SENSE(6) all pages, DBD");
		CHECKEQ(r.din.size(), 4 + 12 + 24 + 24 + 20, "all pages length");
		Cmd c = cmd10(0, 0x5A, 0x08, 0, 255);
		c.cdb[2] = 0x08;                                // page 8 (caching)
		c.cdb[3] = c.cdb[4] = c.cdb[5] = 0;
		r = run(c);
		expect_good(r, "MODE SENSE(10) page 8");
		CHECKEQ(r.din.size(), 8 + 20, "MODE SENSE(10) length");
		if (r.din.size() == 28) CHECKEQ((r.din[0] << 8) | r.din[1], 26, "MODE SENSE(10) mode data length");
		r = run(cmd6(0, 0x1A, 0, 0xC4, 0, 255));
		expect_check(r, "MODE SENSE saved values (not supported)");
		CHECKEQ(sense(0), 0x524, "sense: invalid field in CDB");
		r = run(cmd6(0, 0x1A, 0, 0x37, 0, 255));
		expect_check(r, "MODE SENSE unknown page");
	}

	// ------------------------------------------------------------------
	section("errors and REQUEST SENSE formats (hdc.c)");
	{
		Res r = run(cmd6(0, 0x0D, 0, 0, 0, 0));
		expect_check(r, "unsupported opcode $0D");
		CHECKEQ(sense(0), 0x520, "sense: opcode not supported ($20)");
		CHECKEQ(sense(0), 0x000, "sense cleared after it was reported");
		Cmd c = cmd10(0, 0xC1, 0, 0, 0);               // group 6: 10 byte vendor command
		r = run(c);
		expect_check(r, "group 6 vendor opcode (10 byte CDB consumed)");
		r = run(cmd10(0, 0x28, 0, HD0_SECT - 1, 2));
		expect_check(r, "READ(10) past the end");
		CHECKEQ(sense(0), 0x521, "sense: invalid block address ($21)");
		r = run(cmd6(0, 0x0B, 0, 0x10, 0x00, 0));       // SEEK 4096 = capacity
		expect_check(r, "SEEK(6) to the capacity");
		r = run(cmd6(0, 0x03, 0, 0, 0, 4));
		expect_good(r, "REQUEST SENSE allocation 4 (old format)");
		CHECK(r.din.size() == 4 && r.din[0] == 0x21, "old format byte 0 = error code $21");
		r = run(cmd6(0, 0x0B, 0, 0x0F, 0xFF, 0));       // SEEK 4095
		expect_good(r, "SEEK(6) to the last block");
		r = run(cmd6(0, 0x03, 0, 0, 0, 0));
		expect_good(r, "REQUEST SENSE allocation 0");
		CHECKEQ(r.din.size(), 4, "allocation 0 = 4 bytes (SCSI-1, HDC_Cmd_RequestSense)");
		r = run(cmd6(0, 0x03, 0, 0, 0, 200));
		CHECKEQ(r.din.size(), 22, "REQUEST SENSE limited to 22 bytes");
		if (r.din.size() == 22) { CHECKEQ(r.din[0], 0x70, "extended sense"); CHECKEQ(r.din[7], 14, "additional length"); }
		r = run(cmd6(0, 0x1B, 0, 0, 0, 1));
		expect_good(r, "START STOP UNIT");
		r = run(cmd6(0, 0x1E, 0, 0, 0, 1));
		expect_good(r, "PREVENT ALLOW MEDIUM REMOVAL");
		r = run(cmd10(0, 0x2F, 0, 10, 5));
		expect_good(r, "VERIFY(10) without BYTCHK");
		r = run(cmd10(0, 0x2F, 0, HD0_SECT - 2, 5));
		expect_check(r, "VERIFY(10) past the end");
	}

	// ------------------------------------------------------------------
	section("READ by PIO and by DMA (sector data into RAM)");
	{
		Res r = run(cmd6(0, 0x08, 0, 0x00, 0x07, 1));
		expect_good(r, "READ(6) LBA 7, PIO");
		std::vector<uint8_t> g = read_file(wpath("hd0.img"), 7 * 512, 512);
		CHECK(r.din == g, "READ(6) PIO data = image sector 7");

		for (int pass = 0; pass < 2; pass++) {
			uint32_t lba = pass ? 1000 : 3, n = pass ? 40 : 3;
			uint32_t a = 0x100000 + pass * 0x40000;
			memset(&ram[a - 16], 0xEE, n * 512 + 64);
			Cmd c = cmd10(0, 0x28, 0, lba, n);
			c.dma = DMA_IN; c.dma_addr = a; c.dma_count = n;
			uint64_t t0 = cycle;
			r = run(c);
			char what[64]; snprintf(what, sizeof what, "READ(10) %u sectors by DMA", n);
			expect_good(r, what);
			CHECK(r.dma_irq, "interrupt at the end of the DMA (phase mismatch: status phase)");
			CHECKEQ(r.bsr_after_dma & (BS_IRQ | BS_PM | BS_DRQ), BS_IRQ, "BSR after DMA: IRQ, no phase match, no DRQ");
			CHECKEQ(r.dma_end, a + n * 512, "DMA address after the transfer");
			g = read_file(wpath("hd0.img"), (uint64_t)lba * 512, n * 512);
			int bad = 0;
			for (uint32_t i = 0; i < n * 512; i++) if (ram[a + i] != g[i]) bad++;
			CHECKEQ(bad, 0, "DMA data = image sectors");
			CHECK(ram[a - 1] == 0xEE && ram[a + n * 512] == 0xEE, "nothing written outside the buffer");
			printf("    %u sectors in %.2f ms simulated\n", n, (cycle - t0) * 1000.0 / 32e6);
			CHECKEQ(dut->irq, 0, "interrupt cleared by the register 7 read");
		}
		CHECKEQ(rw(DMA_MODE) & 3, 1, "DMA status: no error, sector count 0");

		// READ(6) count 0 = 256 blocks
		uint32_t a = 0x200000;
		Cmd c = cmd6(0, 0x08, 0, 0x01, 0x00, 0);
		c.dma = DMA_IN; c.dma_addr = a; c.dma_count = 256;
		r = run(c);
		expect_good(r, "READ(6) count 0 = 256 blocks by DMA");
		g = read_file(wpath("hd0.img"), 256 * 512, 256 * 512);
		int bad = 0;
		for (uint32_t i = 0; i < 256 * 512; i++) if (ram[a + i] != g[i]) bad++;
		CHECKEQ(bad, 0, "256 blocks of data");

		r = run(cmd10(0, 0x28, 0, 5, 0));
		expect_good(r, "READ(10) of 0 blocks: no data phase");
		CHECKEQ(r.din.size(), 0, "no data");
	}

	// ------------------------------------------------------------------
	section("short responses by DMA: the last partial FIFO half is written");
	{
		uint32_t a = 0x300000;
		memset(&ram[a], 0xEE, 128);
		Cmd c = cmd6(0, 0x12, 0, 0, 0, 36);
		c.dma = DMA_IN; c.dma_addr = a; c.dma_count = 1;
		Res r = run(c);
		expect_good(r, "INQUIRY by DMA");
		CHECK(r.dma_irq, "interrupt at the status phase");
		CHECK(str(std::vector<uint8_t>(ram.begin() + a, ram.begin() + a + 36), 16, 16) == "Falcon SCSI HD  ", "INQUIRY bytes in RAM");
		CHECKEQ(r.dma_end, a + 48, "DMA address rounded up to the 16 byte FIFO half");
		CHECKEQ(ram[a + 48], 0xEE, "nothing written beyond the flushed half");
	}

	// ------------------------------------------------------------------
	section("WRITE by DMA and PIO, VERIFY with BYTCHK");
	{
		uint32_t a = 0x400000, n = 5, lba = 200;
		for (uint32_t i = 0; i < n * 512; i++) ram[a + i] = (uint8_t)(i * 13 + 7);
		Cmd c = cmd10(0, 0x2A, 0, lba, n);
		c.dma = DMA_OUT; c.dma_addr = a; c.dma_count = n;
		Res r = run(c);
		expect_good(r, "WRITE(10) 5 sectors by DMA");
		CHECK(r.dma_irq, "interrupt at the end of the DMA write");
		CHECKEQ(r.bsr_after_dma & BS_IRQ, BS_IRQ, "BSR IRQ after the DMA write");
		std::vector<uint8_t> g = read_file(wpath("hd0.img"), (uint64_t)lba * 512, n * 512);
		int bad = 0;
		for (uint32_t i = 0; i < n * 512; i++) if (g[i] != (uint8_t)(i * 13 + 7)) bad++;
		CHECKEQ(bad, 0, "image sectors = RAM data");
		g = read_file(wpath("hd0.img"), (uint64_t)(lba - 1) * 512, 512);
		std::vector<uint8_t> orig(512);
		for (int i = 0; i < 512; i++) orig[i] = pat(11, (uint64_t)(lba - 1) * 512 + i);
		CHECK(g == orig, "sector before the range untouched");

		c = cmd6(0, 0x0A, 0, 0, 50, 1);
		for (int i = 0; i < 512; i++) c.dout.push_back((uint8_t)(255 - i));
		r = run(c);
		expect_good(r, "WRITE(6) 1 sector by PIO");
		g = read_file(wpath("hd0.img"), 50 * 512, 512);
		bad = 0;
		for (int i = 0; i < 512; i++) if (g[i] != (uint8_t)(255 - i)) bad++;
		CHECKEQ(bad, 0, "image sector 50 = PIO data");
		r = run(cmd6(0, 0x08, 0, 0, 50, 1));
		CHECK(r.din.size() == 512 && r.din[3] == 252, "read back sector 50");

		g = read_file(wpath("hd0.img"), 60 * 512, 1024);
		c = cmd10(0, 0x2F, 0x02, 60, 2);               // VERIFY with BYTCHK: 2 blocks out
		c.dma = DMA_OUT; c.dma_addr = a; c.dma_count = 2;
		r = run(c);
		expect_good(r, "VERIFY(10) BYTCHK, data out by DMA");
		CHECK(read_file(wpath("hd0.img"), 60 * 512, 1024) == g, "VERIFY data does not touch the image");

		r = run(cmd10(0, 0x2A, 0, 7, 0));
		expect_good(r, "WRITE(10) of 0 blocks");
	}

	// ------------------------------------------------------------------
	section("read only image: hard disk 1");
	{
		uint64_t sz = mount(1, wpath("hd1.img"));
		CHECKEQ(sz, HD1_SECT * 512, "hd1 mounted");
		Res r = run(cmd6(1, 0x00, 0, 0, 0, 0));
		expect_good(r, "ID 1 TEST UNIT READY");
		Cmd c = cmd10(1, 0x2A, 0, 0, 1);
		c.dout.assign(512, 0x55);
		r = run(c);
		expect_check(r, "WRITE(10) to a read only image");
		CHECKEQ(sense(1), 0x727, "sense: DATA PROTECT / write protected");
		r = run(cmd6(1, 0x1A, 0, 0x08, 0, 255));
		CHECK(r.din.size() > 3 && r.din[2] == 0x80, "MODE SENSE device specific: write protected");
		r = run(cmd10(1, 0x28, 0, HD1_SECT - 1, 1));
		expect_good(r, "READ(10) of the last block");
		CHECK(r.din == read_file(wpath("hd1.img"), (HD1_SECT - 1) * 512, 512), "last block data");
		CHECK(sense(0) == 0, "unit 0 sense unaffected by unit 1");
	}

	// ------------------------------------------------------------------
	section("DMA count shorter than the data: the transfer stops; RST recovers");
	{
		uint32_t a = 0x500000;
		memset(&ram[a], 0xEE, 4096);
		Cmd c = cmd10(0, 0x28, 0, 300, 4);
		c.dma = DMA_IN; c.dma_addr = a; c.dma_count = 2;
		c.dma_timeout = 400000;                        // 12.5 ms: far more than 4 sectors take
		Res r = run(c);
		CHECK(!r.dma_irq, "no interrupt while the target still has data (phase matches)");
		CHECKEQ(r.dma_end, a + 1024, "DMA address stops after 2 sectors");
		std::vector<uint8_t> g = read_file(wpath("hd0.img"), 300 * 512, 1024);
		CHECK(memcmp(&ram[a], g.data(), 1024) == 0, "two sectors transferred");
		CHECKEQ(ram[a + 1024], 0xEE, "nothing beyond the sector count");
		CHECKEQ(ncr_rd(R_CSBSR) & (CS_BSY | CS_REQ), CS_BSY | CS_REQ, "target still in data in with REQ");
		ncr_wr(R_ICR, ICR_RST);
		CHECKEQ(ncr_rd(R_ICR), 0x80, "ICR after RST = $80 (ncr5380_reset)");
		CHECKEQ(ncr_rd(R_CSBSR), CS_RST, "bus status: RST, bus free");
		CHECK(dut->irq, "RST interrupt (ncr5380_reset -> set_irq)");
		CHECKEQ(ncr_rd(R_BSR) & BS_IRQ, BS_IRQ, "BSR IRQ");
		ncr_wr(R_ICR, 0);
		ncr_rd(R_RPI);
		ticks(6);                                      // next CPU bus cycle (the clear takes 2 clocks)
		CHECKEQ(dut->irq, 0, "interrupt cleared by register 7");
		ticks(20000);                                  // let a dropped HPS transfer finish
		r = run(cmd10(0, 0x28, 0, 9, 1));
		expect_good(r, "READ after the RST");
		CHECK(r.din == read_file(wpath("hd0.img"), 9 * 512, 512), "data after the RST");
	}

	// ------------------------------------------------------------------
	section("DMA mode with the target in another phase: phase mismatch interrupt");
	{
		Cmd c = cmd10(0, 0x28, 0, HD0_SECT, 1);         // fails: status phase at once
		c.dma = DMA_IN; c.dma_addr = 0x600000; c.dma_count = 1; c.arm_early = true;
		memset(&ram[0x600000], 0xEE, 64);
		Res r = run(c);
		CHECK(r.dma_irq, "phase mismatch interrupt when DMA mode meets the status phase");
		CHECKEQ(r.bsr_after_dma & (BS_IRQ | BS_PM | BS_DRQ), BS_IRQ, "BSR: IRQ, no phase match, no DRQ");
		CHECKEQ(r.status, 0x02, "status CHECK CONDITION by PIO after the DMA attempt");
		CHECKEQ(ram[0x600000], 0xEE, "nothing transferred");
		CHECKEQ(sense(0), 0x521, "sense invalid address");

		// armed while the target still fetches its first sector: no early interrupt
		uint32_t a = 0x610000;
		c = cmd10(0, 0x28, 0, 77, 9);
		c.dma = DMA_IN; c.dma_addr = a; c.dma_count = 9; c.arm_early = true;
		r = run(c);
		expect_good(r, "READ(10) 9 sectors, DMA armed right after the command");
		CHECK(r.dma_irq, "interrupt only at the status phase");
		CHECKEQ(r.dma_end, a + 9 * 512, "all 9 sectors moved before the interrupt");
		CHECK(memcmp(&ram[a], read_file(wpath("hd0.img"), 77 * 512, 9 * 512).data(), 9 * 512) == 0, "data");

		// busy loss with MR monitor busy
		CHECK(select_target(0, false), "select ID 0");
		ncr_wr(R_MR, 0x04);                             // monitor busy
		ncr_wr(R_ICR, ICR_RST);
		ncr_wr(R_ICR, 0);
		ncr_rd(R_RPI);
		ticks(6);
		CHECKEQ(dut->irq, 0, "clean state");
		c = cmd6(0, 0x00, 0, 0, 0, 0);
		CHECK(select_target(0, false), "select ID 0 again");
		ncr_wr(R_MR, 0x04);
		for (int i = 0; i < 6; i++) { wait_req(); ncr_wr(R_TCR, P_CMD); pio_out(0); }
		wait_req(); ncr_wr(R_TCR, P_ST); CHECKEQ(pio_in(), 0, "status byte");
		wait_req(); ncr_wr(R_TCR, P_MI); CHECKEQ(pio_in(), 0, "message byte");
		CHECK(dut->irq, "loss of BSY with MR monitor busy: interrupt");
		CHECKEQ(ncr_rd(R_BSR) & BS_BERR, BS_BERR, "busy error bit");
		ncr_wr(R_MR, 0);
		ncr_rd(R_RPI);
		CHECKEQ(ncr_rd(R_BSR) & BS_BERR, 0, "busy error cleared by register 7");
	}

	// ------------------------------------------------------------------
	section("CD-ROM .iso (Falcon Main)");
	{
		uint64_t sz = mount(2, wpath("cd.iso"));
		CHECKEQ(sz, ISO_SECT * 2048, "iso mounted");
		Res r = run(cmd6(2, 0x00, 0, 0, 0, 0));
		expect_good(r, "TEST UNIT READY");
		r = run(cmd10(2, 0x25, 0, 0, 0));
		expect_good(r, "READ CAPACITY");
		if (r.din.size() == 8) {
			CHECKEQ(be32(r.din, 0), ISO_SECT - 1, "last LBA");
			CHECKEQ(be32(r.din, 4), 2048, "block length 2048");
		}
		uint32_t a = 0x700000;
		Cmd c = cmd10(2, 0x28, 0, 5, 3);
		c.dma = DMA_IN; c.dma_addr = a; c.dma_count = 12;   // 3 x 2048 bytes = 12 DMA sectors
		hps->clear_log();
		r = run(c);
		expect_good(r, "READ(10) 3 CD blocks by DMA");
		std::vector<uint8_t> g = read_file(wpath("cd.iso"), 5 * 2048, 3 * 2048);
		CHECK(memcmp(&ram[a], g.data(), 6144) == 0, "2048 byte sectors 5..7 of the iso");
		CHECK(hps->lba_log[2].size() == 12 && hps->lba_log[2][0] == 20 && hps->lba_log[2][11] == 31,
		      "image blocks 20..31 requested (LBA x 4)");
		r = run(cmd10(2, 0x28, 0, ISO_SECT - 1, 2));
		expect_check(r, "READ past the end of the disc");
		c = cmd10(2, 0x43, 0, 0, 100);                  // READ TOC, LBA, format 0, from track 0
		r = run(c);
		expect_good(r, "READ TOC");
		CHECKEQ(r.din.size(), 4 + 8 + 8, "TOC length: one track + lead-out");
		if (r.din.size() == 20) {
			CHECKEQ((r.din[0] << 8) | r.din[1], 18, "TOC data length");
			CHECKEQ(r.din[2], 1, "first track");
			CHECKEQ(r.din[3], 1, "last track");
			CHECKEQ(r.din[5], 0x14, "track 1 data");
			CHECKEQ(be32(r.din, 8), 0, "track 1 at LBA 0");
			CHECKEQ(r.din[14], 0xAA, "lead-out");
			CHECKEQ(be32(r.din, 16), ISO_SECT, "lead-out LBA");
		}
		c = cmd10(2, 0x43, 0x02, 0, 100);               // MSF
		hps->clear_log();
		r = run(c);
		CHECK(hps->lba_log[2].size() == 1 && hps->lba_log[2][0] == 0x7E214300u, "READ TOC window LBA with MSF flag (%08x)",
		      hps->lba_log[2].empty() ? 0 : hps->lba_log[2][0]);
		if (r.din.size() == 20) {
			CHECKEQ(be32(r.din, 8), 0x00000200, "track 1 at 00:02:00");
			CHECKEQ(be32(r.din, 16), (0 << 16) | (2 << 8) | 64, "lead-out at 00:02:64 (64 + 150 frames)");
		}
		c = cmd10(2, 0x43, 0, 0, 12, 0, 0x40);          // format 1 (session) in byte 9 bits 7:6
		r = run(c);
		expect_good(r, "READ TOC format 1 (sessions)");
		CHECKEQ(r.din.size(), 12, "session info length");
		c = cmd10(2, 0x43, 0, 0, 100, 5);               // start track 5 > last
		r = run(c);
		expect_check(r, "READ TOC from a track after the last");
		CHECKEQ(sense(2), 0x524, "sense invalid field");
		c = cmd10(2, 0x2A, 0, 0, 1);
		r = run(c);
		expect_check(r, "WRITE(10) to the CD-ROM");
		CHECKEQ(sense(2), 0x727, "sense write protected");
		r = run(cmd6(2, 0x1A, 0, 0x2A, 0, 255));
		expect_good(r, "MODE SENSE CD capabilities page");
		CHECK(r.din.size() == 4 + 8 + 20 && r.din[12] == 0x2A, "page $2A");
		if (r.din.size() == 32) CHECKEQ((r.din[4 + 5] << 16) | (r.din[4 + 6] << 8) | r.din[4 + 7], 2048, "block descriptor length 2048");
		r = run(cmd10(0, 0x43, 0, 0, 100));
		expect_check(r, "READ TOC on a hard disk");
		CHECKEQ(sense(0), 0x520, "sense invalid opcode");
	}

	// ------------------------------------------------------------------
	section("CD-ROM .cue/.bin: MODE1/2352 data track + audio track (Falcon Main)");
	{
		uint64_t sz = mount(2, wpath("cd2.cue"));
		CHECKEQ(sz, 230 * 2048, "virtual disc: 30 + 200 frames of 2048 bytes");
		Res r = run(cmd10(2, 0x43, 0, 0, 100));
		expect_good(r, "READ TOC");
		CHECKEQ(r.din.size(), 4 + 3 * 8, "two tracks + lead-out");
		if (r.din.size() == 28) {
			CHECKEQ(r.din[3], 2, "last track 2");
			CHECKEQ(r.din[5], 0x14, "track 1 data");
			CHECKEQ(be32(r.din, 8), 0, "track 1 at 0");
			CHECKEQ(r.din[13], 0x10, "track 2 audio");
			CHECKEQ(r.din[14], 2, "track 2 number");
			CHECKEQ(be32(r.din, 16), 180, "track 2 at 30 + 150 (INDEX 01 after the 2 s pregap)");
			CHECKEQ(be32(r.din, 24), 230, "lead-out 230");
		}
		r = run(cmd10(2, 0x43, 0, 0, 100, 2));          // from track 2
		CHECKEQ(r.din.size(), 4 + 2 * 8, "TOC from track 2: track 2 + lead-out");
		r = run(cmd10(2, 0x25, 0, 0, 0));
		CHECK(r.din.size() == 8 && be32(r.din, 0) == 229 && be32(r.din, 4) == 2048, "READ CAPACITY of the cue disc");
		uint32_t a = 0x780000;
		Cmd c = cmd10(2, 0x28, 0, 2, 3);
		c.dma = DMA_IN; c.dma_addr = a; c.dma_count = 12;
		r = run(c);
		expect_good(r, "READ(10) data blocks 2..4");
		int bad = 0;
		for (uint32_t l = 2; l < 5; l++)
			for (int i = 0; i < 2048; i++) if (ram[a + (l - 2) * 2048 + i] != pat(44, (uint64_t)l * 2048 + i)) bad++;
		CHECKEQ(bad, 0, "user data of the MODE1/2352 frames (offset 16)");
		r = run(cmd10(2, 0x28, 0, 190, 1));
		expect_good(r, "READ(10) of an audio block");
		bad = 0;
		for (uint8_t b : r.din) if (b) bad++;
		CHECK(r.din.size() == 2048 && bad == 0, "audio block reads as zeros");

		// audio commands forwarded to Main
		c = cmd10(2, 0x47, 0, 0, 0);
		c.cdb = { 0x47, 0, 0, 0, 4, 40, 0, 5, 0, 0 };   // PLAY MSF 00:04:40 .. 00:05:00 (LBA 190 .. 225)
		hps->clear_log();
		r = run(c);
		expect_good(r, "PLAY AUDIO MSF");
		CHECK(hps->lba_log[2].size() == 1 && hps->wr_log[2][0] && hps->lba_log[2][0] == 0x7D204700u,
		      "forwarded as a block write to 0x7D204700");
		const falcon_cd_audio *au = falcon_scsi_audio(2);
		CHECK(au->status == 0x11 && au->pos == 190 && au->end == 225, "Main play state: playing 190..225");
		c = cmd10(2, 0x42, 0x02, 0, 16);
		c.cdb = { 0x42, 0x02, 0x40, 0x01, 0, 0, 0, 0, 16, 0 };   // MSF, SubQ, format 1
		r = run(c);
		expect_good(r, "READ SUB-CHANNEL current position");
		if (r.din.size() == 16) {
			CHECKEQ(r.din[1], 0x11, "audio status: playing");
			CHECKEQ(r.din[6], 2, "track 2");
			CHECKEQ(be32(r.din, 8), (0 << 16) | (4 << 8) | 40, "absolute 00:04:40");
			CHECKEQ(be32(r.din, 12), (0 << 16) | (0 << 8) | 10, "relative 00:00:10");
		} else CHECK(false, "SUB-CHANNEL length %zu", r.din.size());
		c = cmd10(2, 0x4B, 0, 0, 0);                    // PAUSE
		r = run(c);
		expect_good(r, "PAUSE");
		CHECK(au->status == 0x12, "Main play state: paused");
		c = cmd6(2, 0x15, 0x10, 0, 0, 12);              // MODE SELECT(6) 12 bytes
		for (int i = 0; i < 12; i++) c.dout.push_back((uint8_t)(0xA0 + i));
		r = run(c);
		expect_good(r, "MODE SELECT(6) with a parameter list");
		CHECK(au->mode_sel_len == 12 && au->mode_sel[0] == 0xA0 && au->mode_sel[11] == 0xAB &&
		      au->last_cmd[0] == 0x15 && au->last_cmd[4] == 12, "parameter list and CDB reached Main");
		r = run(cmd10(0, 0x45, 0, 0, 10));
		expect_check(r, "PLAY AUDIO on a hard disk");
	}

	// ------------------------------------------------------------------
	section("CD-ROM raw MODE1/2352 .bin without a cue (Falcon Main)");
	{
		uint64_t sz = mount(2, wpath("cd3.bin"));
		CHECKEQ(sz, 12 * 2048, "raw bin: 12 frames");
		Res r = run(cmd10(2, 0x28, 0, 11, 1));
		expect_good(r, "READ(10) last block");
		int bad = 0;
		for (int i = 0; i < 2048 && i < (int)r.din.size(); i++) if (r.din[i] != pat(55, 11 * 2048 + i)) bad++;
		CHECK(r.din.size() == 2048 && bad == 0, "user data of frame 11");
	}

	// ------------------------------------------------------------------
	section("stock Main (no Falcon support): built-in responses, generic sector path");
	{
		g_is_falcon = false;
		mount(2, "");
		mount(0, wpath("hd0.img"));
		uint64_t ops_hook0 = main_sim.ops_hook;
		Res r = run(cmd6(0, 0x12, 0, 0, 0, 36));
		expect_good(r, "INQUIRY (window unanswered: built-in)");
		if (r.din.size() == 36) {
			CHECKEQ(r.din[0], 0x00, "type disk");
			CHECKEQ(r.din[2], 0x02, "version");
			CHECKEQ(r.din[4], 31, "additional length");
			CHECK(str(r.din, 8, 8) == "MiSTer  " && str(r.din, 16, 16) == "Falcon HD       " && str(r.din, 32, 4) == "1.0 ",
			      "built-in INQUIRY strings '%s' '%s' '%s'", str(r.din, 8, 8).c_str(), str(r.din, 16, 16).c_str(), str(r.din, 32, 4).c_str());
		}
		r = run(cmd6(0, 0x12, 0x20, 0, 0, 36));
		CHECK(!r.din.empty() && r.din[0] == 0x7F, "built-in INQUIRY LUN 1 = $7F");
		r = run(cmd10(0, 0x25, 0, 0, 0));
		expect_good(r, "READ CAPACITY (built-in from img_size)");
		CHECK(r.din.size() == 8 && be32(r.din, 0) == HD0_SECT - 1 && be32(r.din, 4) == 512, "built-in capacity");
		r = run(cmd6(0, 0x1A, 0, 0x3F, 0, 255));
		expect_check(r, "MODE SENSE without Main support");
		CHECKEQ(sense(0), 0x524, "sense invalid field");
		uint32_t a = 0x800000;
		Cmd c = cmd10(0, 0x28, 0, 1234, 6);
		c.dma = DMA_IN; c.dma_addr = a; c.dma_count = 6;
		r = run(c);
		expect_good(r, "READ(10) 6 sectors by DMA, generic path");
		CHECK(memcmp(&ram[a], read_file(wpath("hd0.img"), 1234 * 512, 6 * 512).data(), 6 * 512) == 0, "data");
		for (int i = 0; i < 1024; i++) ram[a + i] = (uint8_t)(i * 3);
		c = cmd10(0, 0x2A, 0, 2000, 2);
		c.dma = DMA_OUT; c.dma_addr = a; c.dma_count = 2;
		r = run(c);
		expect_good(r, "WRITE(10) 2 sectors, generic path");
		std::vector<uint8_t> g = read_file(wpath("hd0.img"), 2000 * 512, 1024);
		int bad = 0;
		for (int i = 0; i < 1024; i++) if (g[i] != (uint8_t)(i * 3)) bad++;
		CHECKEQ(bad, 0, "written data in the image");
		CHECKEQ(main_sim.ops_hook, ops_hook0, "no block went through the Falcon hooks");

		mount(2, wpath("cd.iso"));
		r = run(cmd6(2, 0x12, 0, 0, 0, 36));
		CHECK(r.din.size() == 36 && r.din[0] == 0x05 && r.din[1] == 0x80 && str(r.din, 16, 16) == "Falcon CD-ROM   ",
		      "built-in INQUIRY of the CD-ROM");
		r = run(cmd10(2, 0x25, 0, 0, 0));
		CHECK(r.din.size() == 8 && be32(r.din, 0) == ISO_SECT - 1 && be32(r.din, 4) == 2048, "built-in CD capacity");
		r = run(cmd10(2, 0x28, 0, 60, 2));
		expect_good(r, "CD READ(10) from the iso, generic path");
		CHECK(r.din == read_file(wpath("cd.iso"), 60 * 2048, 4096), "iso data");
		r = run(cmd10(2, 0x43, 0, 0, 100));
		expect_check(r, "READ TOC needs the Falcon Main");
		c = cmd6(2, 0x15, 0x10, 0, 0, 4);
		c.dout = { 1, 2, 3, 4 };
		r = run(c);
		expect_good(r, "MODE SELECT forwarded to a stock Main (ignored write)");
		g_is_falcon = true;
	}

	// ------------------------------------------------------------------
	section("peripheral reset (68030 RESET): media stay mounted");
	{
		// reset in the middle of a sector transfer from the HPS
		CHECK(select_target(0, false), "select ID 0");
		uint8_t cdb[6] = { 0x08, 0, 0, 40, 8, 0 };
		for (int i = 0; i < 6; i++) { wait_req(); ncr_wr(R_TCR, P_CMD); pio_out(cdb[i]); }
		uint64_t t0 = cycle;
		while (!(dut->sd_rd & 1) && cycle - t0 < 100000) tick();
		CHECK(dut->sd_rd & 1, "image read requested");
		while (!hps->sd_ack && cycle - t0 < 100000) tick();
		ticks(300);                                    // in the middle of the block
		dut->reset = 1;
		ticks(8);
		dut->reset = 0;
		ticks(4);
		CHECKEQ(ncr_rd(R_CSBSR), 0x00, "bus free after reset");
		CHECKEQ(ncr_rd(R_ICR) | ncr_rd(R_MR) | ncr_rd(R_TCR), 0x00, "5380 registers cleared");
		CHECKEQ(dut->irq, 0, "no interrupt from the chip reset");
		ticks(20000);                                  // the HPS finishes the cut transfer
		Res r = run(cmd6(0, 0x00, 0, 0, 0, 0));
		expect_good(r, "ID 0 TEST UNIT READY after reset (no new img_mounted)");
		r = run(cmd10(0, 0x28, 0, 41, 2));
		expect_good(r, "ID 0 READ after reset");
		CHECK(r.din == read_file(wpath("hd0.img"), 41 * 512, 1024), "data after reset");
		Cmd c = cmd10(1, 0x2A, 0, 0, 1);
		c.dout.assign(512, 0x55);
		r = run(c);
		expect_check(r, "ID 1 still read only after reset");
		r = run(cmd6(2, 0x00, 0, 0, 0, 0));
		expect_good(r, "ID 2 still has its medium after reset");
		g_is_falcon = false;                           // built-in capacity: the size survived too
		r = run(cmd10(0, 0x25, 0, 0, 0));
		CHECK(r.din.size() == 8 && be32(r.din, 0) == HD0_SECT - 1, "built-in READ CAPACITY after reset");
		r = run(cmd10(2, 0x25, 0, 0, 0));
		CHECK(r.din.size() == 8 && be32(r.din, 0) == ISO_SECT - 1 && be32(r.din, 4) == 2048, "CD capacity after reset");
		g_is_falcon = true;
	}

	// ------------------------------------------------------------------
	section("unmount: the hard disk leaves the bus");
	{
		mount(1, "");
		CHECK(!select_target(1, false), "ID 1 no longer answers");
	}

	CHECKEQ(hps->errors, 0, "hps protocol errors");
	CHECKEQ(g_spi_errors, 0, "Main SPI shim protocol errors");
	printf("HPS multi-block requests: %llu\n", (unsigned long long)hps->multi);
	printf("\nSCSI: %d checks, %d failures, %llu cycles (%.2f s simulated)\n", checks, fails,
	       (unsigned long long)cycle, cycle / 32e6);
	printf(fails ? "SCSI TEST FAIL\n" : "SCSI TEST PASS\n");
	delete dut;
	return fails ? 1 : 0;
}
