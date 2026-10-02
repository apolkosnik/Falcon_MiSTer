// tb_ide.cpp - Verilator testbench for rtl/falcon/falcon_ide.sv
//
// Drives the real falcon_ide through the common register bus and serves its
// two disk images with the hps_io model (hps_disk_model.h).  Expected values
// come from Hatari src/ide.c (ide_identify, ide_ioport_read/write,
// ide_ctrl_write, ide_set_signature, Ide_Mem_*) and the ATA spec where the
// module header documents a deviation.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>
#include "Vfalcon_ide.h"
#include "verilated.h"
#include "hps_disk_model.h"

static Vfalcon_ide *dut;
static HpsModel *hps;
static uint64_t cycle = 0;
static int fails = 0, checks = 0;

#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define CHECKEQ(got, exp, what) do { checks++; if ((uint64_t)(got) != (uint64_t)(exp)) { fails++; \
	printf("FAIL: %s: got 0x%llx expected 0x%llx\n", what, (unsigned long long)(got), (unsigned long long)(exp)); } } while (0)

static uint32_t mnt_pulse = 0;

static void tick() {
	if (dut->sd_lba0 != dut->sd_lba1 || dut->sd_buff_din0 != dut->sd_buff_din1) { printf("FAIL: sd_lba0/1 or sd_buff_din0/1 differ\n"); fails++; }
	hps->step(dut->sd_rd, dut->sd_wr, dut->sd_lba0, dut->sd_buff_din0);
	dut->clk = 1;
	dut->eval();
	cycle++;
	dut->sd_ack = hps->sd_ack;
	dut->sd_buff_addr = hps->sd_buff_addr;
	dut->sd_buff_dout = hps->sd_buff_dout;
	dut->sd_buff_wr = hps->sd_buff_wr;
	dut->img_mounted = mnt_pulse;
	mnt_pulse = 0;
	dut->clk = 0;
	dut->eval();
}

static void ticks(int n) { while (n--) tick(); }

// one bus access; returns the 16 bit read data
static uint16_t bus(bool we, uint32_t byteaddr, bool uds, bool lds, uint16_t din) {
	dut->bus_cs = 1;
	dut->bus_stb = 1;
	dut->bus_we = we;
	dut->bus_addr = (byteaddr >> 1) & 0x1F;
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

// Falcon register addresses (Hatari fcha2io)
enum { R_DATA = 0xF00000, R_ERR = 0xF00005, R_NSEC = 0xF00009, R_SECT = 0xF0000D,
       R_LCYL = 0xF00011, R_HCYL = 0xF00015, R_SEL = 0xF00019, R_CMD = 0xF0001D,
       R_ALT = 0xF00039 };

// byte access (move.b): odd address -> LDS / bits 7:0, even -> UDS / bits 15:8
static uint8_t rb(uint32_t a) {
	uint16_t v = bus(false, a, (a & 1) == 0, (a & 1) == 1, 0);
	return (a & 1) ? (v & 0xFF) : (v >> 8);
}
static void wb(uint32_t a, uint8_t v) {
	bus(true, a, (a & 1) == 0, (a & 1) == 1, (a & 1) ? v : (uint16_t)(v << 8));
}
static uint16_t rw(uint32_t a) { return bus(false, a, true, true, 0); }
static void ww(uint32_t a, uint16_t v) { bus(true, a, true, true, v); }

static void mount(int u, uint64_t size, bool ro) {
	hps->img[u].mounted = size != 0;
	hps->img[u].size = size;
	hps->img[u].readonly = ro;
	dut->img_size = size;
	dut->img_readonly = ro;
	mnt_pulse = 1u << u;
	ticks(10);     // the mount scan starts (BSY) a few clocks after the pulse
}

// wait until BSY clears (alternate status: no interrupt acknowledge)
static uint8_t wait_notbusy(int maxc = 200000) {
	uint8_t s = 0;
	for (int i = 0; i < maxc; i += 8) {
		s = rb(R_ALT);
		if (!(s & 0x80)) return s;
		ticks(4);
	}
	printf("FAIL: BSY timeout\n");
	fails++;
	return s;
}

// --------------------------------------------------------------------------
// Expected IDENTIFY data: Hatari ide_identify() with this core's documented
// deviations (model string vendor, no LBA48, current geometry in 54-58).
static void padstr(uint16_t *w, int first, const char *src, int len) {
	char tmp[64];
	for (int i = 0; i < len; i++) tmp[i] = *src ? *src++ : ' ';
	// Hatari padstr: str[i ^ 1] = v, then the words are little endian, so
	// the word value has character 2n in bits 15:8.
	for (int i = 0; i < len / 2; i++) w[first + i] = ((uint8_t)tmp[2 * i] << 8) | (uint8_t)tmp[2 * i + 1];
}

struct Geo { unsigned cyl, heads, spt; };

static void expected_identify(uint16_t *w, int unit, uint64_t nb, Geo def, Geo cur, unsigned mult,
                              uint16_t w63 = 0x07, uint16_t w88 = 0x3f | (1 << 13)) {
	memset(w, 0, 512);
	char buf[64];
	w[0] = 0x0040;
	w[1] = def.cyl;
	w[3] = def.heads;
	w[4] = 512 * def.spt;
	w[5] = 512;
	w[6] = def.spt;
	snprintf(buf, sizeof(buf), "QM%05d", unit + 1);
	padstr(w, 10, buf, 20);
	w[20] = 3; w[21] = 512; w[22] = 4;
	padstr(w, 23, "1.0", 8);
	snprintf(buf, sizeof(buf), "MiSTer  IDE disk %luM", (unsigned long)(nb / 2048));
	padstr(w, 27, buf, 40);
	w[47] = 0x8000 | 16;
	w[48] = 1;
	w[49] = (1 << 11) | (1 << 9) | (1 << 8);
	w[51] = 0x200; w[52] = 0x200;
	w[53] = 7;
	w[54] = cur.cyl; w[55] = cur.heads; w[56] = cur.spt;
	uint32_t old = cur.cyl * cur.heads * cur.spt;
	w[57] = old & 0xFFFF; w[58] = old >> 16;
	if (mult) w[59] = 0x100 | mult;
	uint64_t l28 = nb >= (1u << 28) ? (1u << 28) - 1 : nb;
	w[60] = l28 & 0xFFFF; w[61] = (l28 >> 16) & 0xFFFF;
	w[63] = w63;
	w[65] = w[66] = w[67] = w[68] = 120;
	w[80] = 0xf0; w[81] = 0x16; w[82] = 1 << 14;
	w[83] = (1 << 14) | (1 << 12);        // no LBA48 / flush ext
	w[84] = 1 << 14; w[85] = 1 << 14;
	w[86] = (1 << 14) | (1 << 12);
	w[87] = 1 << 14;
	w[88] = w88;
	w[93] = 1 | (1 << 14) | 0x2000;
	w[106] = 1 << 12;
	w[117] = 256; w[118] = 0;
}

// Hatari ide_init_one default geometry
static Geo default_geo(uint64_t nb) {
	uint64_t c = nb / (16 * 63);
	if (c > 16383) c = 16383; else if (c < 2) c = 2;
	return Geo{ (unsigned)c, 16, 63 };
}

// Hatari byteswap rule: word = swap ? {b0,b1} : {b1,b0}
static uint16_t img_word(int u, uint64_t lba, int wi, bool swap) {
	uint8_t b0 = hps->img[u].get(lba, wi * 2), b1 = hps->img[u].get(lba, wi * 2 + 1);
	return swap ? (b0 << 8) | b1 : (b1 << 8) | b0;
}

static void set_lba(uint32_t lba, int dev, unsigned cnt) {
	wb(R_SEL, 0xE0 | (dev << 4) | ((lba >> 24) & 0x0F));
	wb(R_NSEC, cnt & 0xFF);
	wb(R_SECT, lba & 0xFF);
	wb(R_LCYL, (lba >> 8) & 0xFF);
	wb(R_HCYL, (lba >> 16) & 0xFF);
}
static void set_chs(unsigned c, unsigned h, unsigned s, int dev, unsigned cnt) {
	wb(R_SEL, 0xA0 | (dev << 4) | (h & 0x0F));
	wb(R_NSEC, cnt & 0xFF);
	wb(R_SECT, s);
	wb(R_LCYL, c & 0xFF);
	wb(R_HCYL, (c >> 8) & 0xFF);
}

// wait for INTRQ (with timeout)
static bool wait_irq(int maxc = 400000) {
	for (int i = 0; i < maxc; i++) {
		if (dut->irq) return true;
		tick();
	}
	return false;
}

static int test_no = 0;
static void section(const char *s) { printf("--- %d: %s\n", ++test_no, s); }

int main(int argc, char **argv) {
	Verilated::commandArgs(argc, argv);
	dut = new Vfalcon_ide;
	hps = new HpsModel(2);
	hps->img[0].seed = 0x1111;
	hps->img[1].seed = 0x2222;

	dut->reset = 1;
	ticks(5);
	dut->reset = 0;
	ticks(5);

	// ------------------------------------------------------------------
	section("no drive: Hatari ide_ioport_read returns 0 when both are absent");
	CHECKEQ(rb(R_CMD), 0x00, "status, no drives");
	CHECKEQ(rb(R_ALT), 0x00, "alt status, no drives");
	CHECKEQ(rb(R_NSEC), 0x00, "sector count, no drives");
	CHECKEQ(rb(R_SEL), 0x00, "drive/head, no drives");
	CHECKEQ(rw(R_DATA), 0xFFFF, "data, no transfer");
	wb(R_CMD, 0xEC);
	ticks(20);
	CHECKEQ(dut->irq, 0, "no irq for a command to an absent device");
	CHECKEQ(rb(R_CMD), 0x00, "status still 0");

	// ------------------------------------------------------------------
	section("mount master (normal image, 55AA at $1FE) and slave (byte swapped)");
	// master: 20160 sectors (20 x 1008): default geometry 20/16/63, 9 MB
	const uint64_t nb0 = 20160;
	{
		uint8_t s0[512];
		for (int i = 0; i < 512; i++) s0[i] = HpsImage::pat(0x1111, 0, i);
		s0[0x1fe] = 0x55; s0[0x1ff] = 0xAA;      // PC order MBR signature
		hps->img[0].preset(0, s0);
	}
	mount(0, nb0 * 512, false);
	CHECK(wait_notbusy() == 0x50, "master DRDY|DSC after mount");
	CHECKEQ(rb(R_CMD), 0x50, "master status after mount");
	CHECKEQ(rb(R_ERR), 0x01, "master error after mount (diagnostic code)");
	CHECKEQ(rb(R_NSEC), 0x01, "signature sector count");
	CHECKEQ(rb(R_SECT), 0x01, "signature sector number");
	CHECKEQ(rb(R_LCYL), 0x00, "signature cyl low");
	CHECKEQ(rb(R_HCYL), 0x00, "signature cyl high");

	// ------------------------------------------------------------------
	section("absent slave probing (TOS / EmuTOS)");
	wb(R_SEL, 0xB0);
	CHECKEQ(rb(R_CMD), 0x00, "absent slave status reads 0");
	CHECKEQ(rb(R_ALT), 0x00, "absent slave alt status reads 0");
	wb(R_NSEC, 0x55);
	wb(R_SECT, 0xAA);
	CHECKEQ(rb(R_NSEC), 0x55, "registers are written to both devices (slave copy)");
	CHECKEQ(rb(R_SECT), 0xAA, "sector number slave copy");
	CHECKEQ(rb(R_SEL), 0xB0, "drive/head reads back with DEV=1");
	wb(R_CMD, 0xEC);
	ticks(50);
	CHECKEQ(dut->irq, 0, "command to the absent slave is ignored (no irq)");
	CHECKEQ(rb(R_ALT), 0x00, "absent slave status still 0");
	CHECKEQ(rw(R_DATA), 0xFFFF, "no data from the absent slave");
	wb(R_SEL, 0xA0);
	CHECKEQ(rb(R_NSEC), 0x55, "master copy of sector count");
	CHECKEQ(rb(R_CMD), 0x50, "master status");

	// ------------------------------------------------------------------
	section("IDENTIFY DEVICE on the master");
	{
		Geo d = default_geo(nb0);
		uint16_t exp[256], got[256];
		expected_identify(exp, 0, nb0, d, d, 16);
		wb(R_CMD, 0xEC);
		CHECK(wait_irq(), "IDENTIFY raises INTRQ");
		CHECKEQ(rb(R_ALT), 0x58, "alt status DRDY|DSC|DRQ");
		CHECKEQ(dut->irq, 1, "alt status read does not clear INTRQ");
		CHECKEQ(rb(R_CMD), 0x58, "status DRDY|DSC|DRQ");
		CHECKEQ(dut->irq, 0, "status read clears INTRQ");
		for (int i = 0; i < 256; i++) got[i] = rw(R_DATA);
		int bad = 0;
		for (int i = 0; i < 256; i++) if (got[i] != exp[i]) {
			if (bad < 10) printf("FAIL: identify word %d: got 0x%04x expected 0x%04x\n", i, got[i], exp[i]);
			bad++;
		}
		checks++; if (bad) fails++;
		char model[41];
		for (int i = 0; i < 20; i++) { model[2*i] = got[27+i] >> 8; model[2*i+1] = got[27+i] & 0xFF; }
		model[40] = 0;
		printf("    model \"%s\", cylinders %u heads %u sectors %u, LBA %u\n", model, got[1], got[3], got[6],
		       got[60] | (got[61] << 16));
		CHECKEQ(rb(R_CMD), 0x50, "status after IDENTIFY data read");
		CHECKEQ(rw(R_DATA), 0xFFFF, "data register after the transfer");
	}

	// ------------------------------------------------------------------
	section("mount slave (byte swapped image: AA55 at $1FE, 210000 sectors)");
	const uint64_t nb1 = 210000;
	{
		uint8_t s0[512];
		for (int i = 0; i < 512; i++) s0[i] = HpsImage::pat(0x2222, 0, i);
		s0[0x1fe] = 0xAA; s0[0x1ff] = 0x55;
		hps->img[1].preset(0, s0);
	}
	mount(1, nb1 * 512, false);
	ticks(20000);
	wb(R_SEL, 0xB0);
	CHECK(wait_notbusy() == 0x50, "slave DRDY|DSC after mount");
	{
		Geo d = default_geo(nb1);
		uint16_t exp[256], got[256];
		expected_identify(exp, 1, nb1, d, d, 16);
		wb(R_CMD, 0xEC);
		CHECK(wait_irq(), "slave IDENTIFY raises INTRQ");
		CHECKEQ(rb(R_CMD), 0x58, "slave status DRQ");
		for (int i = 0; i < 256; i++) got[i] = rw(R_DATA);
		int bad = 0;
		for (int i = 0; i < 256; i++) if (got[i] != exp[i]) {
			if (bad < 10) printf("FAIL: slave identify word %d: got 0x%04x expected 0x%04x\n", i, got[i], exp[i]);
			bad++;
		}
		checks++; if (bad) fails++;
		char model[41];
		for (int i = 0; i < 20; i++) { model[2*i] = got[27+i] >> 8; model[2*i+1] = got[27+i] & 0xFF; }
		model[40] = 0;
		printf("    slave model \"%s\" serial word10 %04x\n", model, got[10]);
	}

	// ------------------------------------------------------------------
	section("byte order: sector 0 of both images (Hatari byteswap auto)");
	for (int u = 0; u < 2; u++) {
		bool swap = (u == 0);     // master: normal image -> swapped; slave: already swapped
		set_lba(0, u, 1);
		wb(R_CMD, 0x20);
		CHECK(wait_irq(), "READ irq unit %d", u);
		CHECKEQ(rb(R_CMD), 0x58, "READ DRQ status");
		int bad = 0;
		for (int i = 0; i < 256; i++) {
			uint16_t g = rw(R_DATA), e = img_word(u, 0, i, swap);
			if (g != e) { if (bad < 4) printf("FAIL: unit %d word %d got %04x exp %04x\n", u, i, g, e); bad++; }
		}
		checks++; if (bad) fails++;
		CHECKEQ(rb(R_CMD), 0x50, "status after sector");
	}
	{
		// the boot signature word as the CPU sees it
		set_lba(0, 0, 1);
		wb(R_CMD, 0x20);
		wait_irq(); rb(R_CMD);
		uint16_t w255 = 0;
		for (int i = 0; i < 256; i++) w255 = rw(R_DATA);
		CHECKEQ(w255, 0x55AA, "master: file bytes 55 AA appear as word $55AA");
		set_lba(0, 1, 1);
		wb(R_CMD, 0x20);
		wait_irq(); rb(R_CMD);
		for (int i = 0; i < 256; i++) w255 = rw(R_DATA);
		CHECKEQ(w255, 0x55AA, "slave: swapped file bytes AA 55 appear as word $55AA");
	}

	// ------------------------------------------------------------------
	section("READ SECTORS, CHS, 5 sectors across a head boundary");
	{
		// default geometry 20/16/63: start C=3 H=15 S=61 -> crosses to C=4 H=0
		unsigned c = 3, h = 15, s = 61;
		set_chs(c, h, s, 0, 5);
		wb(R_CMD, 0x20);
		uint64_t lba0 = (c * 16 + h) * 63 + s - 1;
		for (int k = 0; k < 5; k++) {
			CHECK(wait_irq(), "irq for sector %d", k);
			CHECKEQ(rb(R_CMD), 0x58, "DRQ for each sector");
			int bad = 0;
			for (int i = 0; i < 256; i++) {
				uint16_t g = rw(R_DATA), e = img_word(0, lba0 + k, i, true);
				if (g != e) bad++;
			}
			CHECK(bad == 0, "CHS sector %d data (%d bad words)", k, bad);
			CHECKEQ(hps->last_lba[0], lba0 + k, "hps lba of the sector");
		}
		CHECKEQ(rb(R_CMD), 0x50, "status after the last sector");
		CHECKEQ(rb(R_NSEC), 0, "sector count 0 at the end");
		// last sector = lba0+4 = C4 H0 S2
		CHECKEQ(rb(R_SECT), 2, "sector number of the last sector (ATA)");
		CHECKEQ(rb(R_LCYL), 4, "cylinder of the last sector");
		CHECKEQ(rb(R_SEL) & 0x0F, 0, "head of the last sector");
	}

	// ------------------------------------------------------------------
	section("READ SECTORS, LBA, 3 sectors across the $xxFF boundary");
	{
		uint32_t l = 0x12FE;
		set_lba(l, 0, 3);
		wb(R_CMD, 0x21);
		for (int k = 0; k < 3; k++) {
			CHECK(wait_irq(), "irq LBA sector %d", k);
			rb(R_CMD);
			int bad = 0;
			for (int i = 0; i < 256; i++) if (rw(R_DATA) != img_word(0, l + k, i, true)) bad++;
			CHECK(bad == 0, "LBA sector %d data (%d bad)", k, bad);
		}
		CHECKEQ(rb(R_SECT), 0x00, "LBA 7:0 of the last sector 0x1300");
		CHECKEQ(rb(R_LCYL), 0x13, "LBA 15:8");
		CHECKEQ(rb(R_SEL), 0xE0, "LBA 27:24 / LBA mode bit");
	}

	// ------------------------------------------------------------------
	section("WRITE SECTORS, LBA, 3 sectors of a known pattern, read back");
	{
		uint32_t l = 777;
		set_lba(l, 0, 3);
		wb(R_CMD, 0x30);
		for (int k = 0; k < 3; k++) {
			if (k == 0) {
				uint8_t st = wait_notbusy();
				CHECKEQ(st, 0x58, "DRQ for the first block without irq");
				CHECKEQ(dut->irq, 0, "no irq before the first write block (Hatari)");
			} else {
				CHECK(wait_irq(), "irq after block %d", k - 1);
				CHECKEQ(rb(R_CMD), 0x58, "DRQ for the next block");
			}
			for (int i = 0; i < 256; i++) ww(R_DATA, (uint16_t)(0xA000 + k * 0x100 + i) ^ (i << 8));
		}
		CHECK(wait_irq(), "irq after the last block");
		CHECKEQ(rb(R_CMD), 0x50, "status after WRITE");
		CHECKEQ(rb(R_NSEC), 0, "count 0");
		// image check (byteswap: word bits 15:8 = file byte 2n)
		int bad = 0;
		for (int k = 0; k < 3; k++) for (int i = 0; i < 256; i++) {
			uint16_t v = (uint16_t)(0xA000 + k * 0x100 + i) ^ (i << 8);
			if (hps->img[0].get(l + k, 2 * i) != (v >> 8) || hps->img[0].get(l + k, 2 * i + 1) != (v & 0xFF)) bad++;
		}
		CHECK(bad == 0, "image bytes after WRITE (%d bad)", bad);
		set_lba(l, 0, 3);
		wb(R_CMD, 0x20);
		bad = 0;
		for (int k = 0; k < 3; k++) {
			wait_irq(); rb(R_CMD);
			for (int i = 0; i < 256; i++) if (rw(R_DATA) != (uint16_t)((0xA000 + k * 0x100 + i) ^ (i << 8))) bad++;
		}
		CHECK(bad == 0, "read back of written sectors (%d bad)", bad);
	}

	// ------------------------------------------------------------------
	section("WRITE SECTORS, CHS, slave (unswapped image)");
	{
		Geo d = default_geo(nb1);
		unsigned c = 100, h = 3, s = 63;
		set_chs(c, h, s, 1, 2);
		wb(R_CMD, 0x30);
		for (int k = 0; k < 2; k++) {
			if (k) { CHECK(wait_irq(), "slave write irq"); rb(R_CMD); } else wait_notbusy();
			for (int i = 0; i < 256; i++) ww(R_DATA, (uint16_t)(0x1234 + 77 * i + k));
		}
		CHECK(wait_irq(), "slave write last irq");
		CHECKEQ(rb(R_CMD), 0x50, "slave status");
		uint64_t l = (c * d.heads + h) * d.spt + s - 1;
		int bad = 0;
		for (int k = 0; k < 2; k++) for (int i = 0; i < 256; i++) {
			uint16_t v = (uint16_t)(0x1234 + 77 * i + k);
			// unswapped: file byte 2n = word bits 7:0
			if (hps->img[1].get(l + k, 2 * i) != (v & 0xFF) || hps->img[1].get(l + k, 2 * i + 1) != (v >> 8)) bad++;
		}
		CHECK(bad == 0, "slave image bytes (%d bad)", bad);
		CHECKEQ(rb(R_SECT), 1, "CHS sector of the last sector (wrapped)");
		CHECKEQ(rb(R_SEL) & 0x0F, 4, "CHS head of the last sector");
		wb(R_SEL, 0xA0);
	}

	// ------------------------------------------------------------------
	section("SET MULTIPLE MODE / READ MULTIPLE / WRITE MULTIPLE");
	{
		wb(R_NSEC, 3);
		wb(R_CMD, 0xC6);
		CHECK(wait_irq(), "SET MULTIPLE 3 irq");
		CHECKEQ(rb(R_CMD), 0x41, "SET MULTIPLE 3 (not a power of 2) aborts");
		CHECKEQ(rb(R_ERR), 0x04, "ABRT");
		wb(R_NSEC, 32);
		wb(R_CMD, 0xC6);
		wait_irq();
		CHECKEQ(rb(R_CMD), 0x41, "SET MULTIPLE 32 (> MAX_MULT_SECTORS) aborts");
		wb(R_NSEC, 4);
		wb(R_CMD, 0xC6);
		wait_irq();
		CHECKEQ(rb(R_CMD), 0x40, "SET MULTIPLE 4 ok (Hatari: READY_STAT)");
		// identify word 59
		wb(R_CMD, 0xEC); wait_irq(); rb(R_CMD);
		uint16_t w59 = 0;
		for (int i = 0; i < 256; i++) { uint16_t v = rw(R_DATA); if (i == 59) w59 = v; }
		CHECKEQ(w59, 0x104, "IDENTIFY word 59 after SET MULTIPLE 4");

		uint32_t l = 5000;
		set_lba(l, 0, 10);
		wb(R_CMD, 0xC4);
		int k = 0, blocks = 0, bad = 0;
		while (k < 10) {
			if (!wait_irq()) { CHECK(false, "READ MULTIPLE irq block %d", blocks); break; }
			CHECKEQ(rb(R_CMD), 0x58, "READ MULTIPLE DRQ");
			int n = (10 - k) < 4 ? (10 - k) : 4;
			for (int j = 0; j < n; j++, k++)
				for (int i = 0; i < 256; i++) if (rw(R_DATA) != img_word(0, l + k, i, true)) bad++;
			blocks++;
			if (k < 10) CHECKEQ(rb(R_ALT) & 0x08, 0, "DRQ low between blocks");
		}
		CHECK(bad == 0, "READ MULTIPLE data (%d bad)", bad);
		CHECKEQ(blocks, 3, "READ MULTIPLE 10 sectors in blocks of 4,4,2");
		CHECKEQ(rb(R_CMD), 0x50, "status after READ MULTIPLE");

		l = 9000;
		set_lba(l, 0, 6);
		wb(R_CMD, 0xC5);
		k = 0; blocks = 0;
		while (k < 6) {
			if (k == 0) wait_notbusy(); else { CHECK(wait_irq(), "WRITE MULTIPLE irq"); rb(R_CMD); }
			CHECKEQ(rb(R_ALT), 0x58, "WRITE MULTIPLE DRQ");
			int n = (6 - k) < 4 ? (6 - k) : 4;
			for (int j = 0; j < n; j++, k++)
				for (int i = 0; i < 256; i++) ww(R_DATA, (uint16_t)(k * 1000 + i * 3));
			blocks++;
		}
		CHECK(wait_irq(), "WRITE MULTIPLE final irq");
		CHECKEQ(rb(R_CMD), 0x50, "status after WRITE MULTIPLE");
		CHECKEQ(blocks, 2, "WRITE MULTIPLE 6 sectors in blocks of 4,2");
		bad = 0;
		for (int kk = 0; kk < 6; kk++) for (int i = 0; i < 256; i++) {
			uint16_t v = (uint16_t)(kk * 1000 + i * 3);
			if (hps->img[0].get(l + kk, 2 * i) != (v >> 8) || hps->img[0].get(l + kk, 2 * i + 1) != (v & 0xFF)) bad++;
		}
		CHECK(bad == 0, "WRITE MULTIPLE image (%d bad)", bad);
		// disable multiple mode: READ MULTIPLE aborts
		wb(R_NSEC, 0);
		wb(R_CMD, 0xC6); wait_irq(); rb(R_CMD);
		set_lba(1, 0, 1);
		wb(R_CMD, 0xC4);
		CHECK(wait_irq(), "READ MULTIPLE with multiple disabled: irq");
		CHECKEQ(rb(R_CMD), 0x41, "READ MULTIPLE with mult=0 aborts");
		CHECKEQ(rb(R_ERR), 0x04, "ABRT");
		wb(R_NSEC, 16);
		wb(R_CMD, 0xC6); wait_irq(); rb(R_CMD);
	}

	// ------------------------------------------------------------------
	section("READ SECTORS with count 0 (256 sectors)");
	{
		uint32_t l = 3000;
		set_lba(l, 0, 0);
		wb(R_CMD, 0x20);
		int bad = 0, irqs = 0;
		for (int k = 0; k < 256; k++) {
			if (!wait_irq()) break;
			irqs++;
			rb(R_CMD);
			for (int i = 0; i < 256; i++) { uint16_t g = rw(R_DATA); if (i < 4 && g != img_word(0, l + k, i, true)) bad++; }
		}
		CHECKEQ(irqs, 256, "256 sectors transferred");
		CHECK(bad == 0, "count 0 data (%d bad)", bad);
		CHECKEQ(rb(R_CMD), 0x50, "status");
		CHECKEQ(hps->last_lba[0], l + 255, "last lba");
	}

	// ------------------------------------------------------------------
	section("error cases: unknown command, IDNF in LBA and CHS, read only image");
	{
		wb(R_CMD, 0x55);
		CHECK(wait_irq(), "unknown command irq");
		CHECKEQ(rb(R_CMD), 0x41, "unknown command: DRDY|ERR (Hatari ide_abort_command)");
		CHECKEQ(rb(R_ERR), 0x04, "unknown command: ABRT");
		wb(R_CMD, 0x24);   // READ SECTORS EXT: no LBA48 here
		wait_irq();
		CHECKEQ(rb(R_CMD), 0x41, "LBA48 command aborts");

		set_lba(nb0 - 1, 0, 2);
		wb(R_CMD, 0x20);
		CHECK(wait_irq(), "read of the last sector");
		rb(R_CMD);
		for (int i = 0; i < 256; i++) rw(R_DATA);
		CHECK(wait_irq(), "IDNF irq for the sector past the end");
		CHECKEQ(rb(R_CMD), 0x41, "IDNF status");
		CHECKEQ(rb(R_ERR), 0x10, "IDNF error");
		CHECKEQ(rb(R_SECT), (nb0) & 0xFF, "address of the failing sector");
		CHECKEQ(rb(R_NSEC), 1, "one sector not transferred");

		set_chs(0, 0, 0, 0, 1);       // sector 0 is invalid in CHS mode
		wb(R_CMD, 0x20);
		CHECK(wait_irq(), "CHS sector 0 irq");
		CHECKEQ(rb(R_CMD), 0x41, "CHS sector 0: ERR");
		CHECKEQ(rb(R_ERR), 0x10, "CHS sector 0: IDNF");
		set_chs(0, 0, 64, 0, 1);      // sector > 63
		wb(R_CMD, 0x20);
		wait_irq();
		CHECKEQ(rb(R_ERR), 0x10, "CHS sector 64: IDNF");
		set_chs(20, 0, 1, 0, 1);      // cylinder 20 >= 20 cylinders
		wb(R_CMD, 0x20);
		wait_irq();
		CHECKEQ(rb(R_ERR), 0x10, "CHS cylinder out of range: IDNF");

		// read only image (remount the slave read only)
		mount(1, nb1 * 512, true);
		ticks(20000);
		set_lba(10, 1, 1);
		wait_notbusy();
		wb(R_CMD, 0x30);
		wait_notbusy();
		CHECKEQ(rb(R_ALT), 0x58, "write to read only image: DRQ first (Hatari)");
		for (int i = 0; i < 256; i++) ww(R_DATA, 0xDEAD);
		CHECK(wait_irq(), "read only write irq");
		CHECKEQ(rb(R_CMD), 0x41, "read only write: ERR");
		CHECKEQ(rb(R_ERR), 0x04, "read only write: ABRT (Hatari bdrv_write -EACCES)");
		CHECK(hps->img[1].get(10, 0) == HpsImage::pat(0x2222, 10, 0), "read only image unchanged");
		CHECKEQ(hps->errors, 0, "hps model saw no write to the read only image");
		wb(R_SEL, 0xA0);
	}

	// ------------------------------------------------------------------
	section("READ VERIFY, SEEK, RECALIBRATE, CHECK POWER MODE, IDLE/STANDBY, FLUSH");
	{
		set_lba(100, 0, 8);
		wb(R_CMD, 0x40);
		CHECK(wait_irq(), "READ VERIFY irq");
		CHECKEQ(rb(R_CMD), 0x50, "READ VERIFY ok");
		CHECKEQ(rb(R_SECT), 107, "READ VERIFY: address of the last sector");
		CHECKEQ(rb(R_NSEC), 0, "READ VERIFY: count 0");
		set_lba(nb0 - 2, 0, 8);
		wb(R_CMD, 0x41);
		wait_irq();
		CHECKEQ(rb(R_CMD), 0x41, "READ VERIFY past the end: ERR");
		CHECKEQ(rb(R_ERR), 0x10, "READ VERIFY past the end: IDNF");

		set_chs(19, 15, 63, 0, 1);
		wb(R_CMD, 0x70);
		CHECK(wait_irq(), "SEEK irq");
		CHECKEQ(rb(R_CMD), 0x50, "SEEK to the last sector ok");
		set_chs(25, 0, 1, 0, 1);
		wb(R_CMD, 0x70);
		wait_irq();
		CHECKEQ(rb(R_ERR), 0x10, "SEEK out of range: IDNF");

		wb(R_CMD, 0x10);
		CHECK(wait_irq(), "RECALIBRATE irq");
		CHECKEQ(rb(R_CMD), 0x50, "RECALIBRATE status");
		CHECKEQ(rb(R_ERR), 0x00, "RECALIBRATE error 0");

		wb(R_NSEC, 0x00);
		wb(R_CMD, 0xE5);
		CHECK(wait_irq(), "CHECK POWER MODE irq");
		CHECKEQ(rb(R_CMD), 0x40, "CHECK POWER MODE status (Hatari READY_STAT)");
		CHECKEQ(rb(R_NSEC), 0xFF, "CHECK POWER MODE: active/idle");
		const uint8_t pw[] = { 0xE0, 0xE1, 0xE2, 0xE3, 0x94, 0x95, 0x96, 0x97, 0xE7 };
		for (uint8_t c : pw) {
			wb(R_CMD, c);
			CHECK(wait_irq(), "command %02x irq", c);
			CHECKEQ(rb(R_CMD), 0x40, "IDLE/STANDBY/FLUSH status");
		}
	}

	// ------------------------------------------------------------------
	section("SET FEATURES");
	{
		wb(R_ERR, 0x02);   // write cache enable
		wb(R_CMD, 0xEF);
		CHECK(wait_irq(), "SET FEATURES 02 irq");
		CHECKEQ(rb(R_CMD), 0x50, "SET FEATURES 02 ok");
		wb(R_ERR, 0x03);   // transfer mode: multiword DMA 2
		wb(R_NSEC, 0x22);
		wb(R_CMD, 0xEF);
		wait_irq();
		CHECKEQ(rb(R_CMD), 0x50, "SET FEATURES 03 mdma2 ok");
		wb(R_CMD, 0xEC); wait_irq(); rb(R_CMD);
		uint16_t w63 = 0, w88 = 0;
		for (int i = 0; i < 256; i++) { uint16_t v = rw(R_DATA); if (i == 63) w63 = v; if (i == 88) w88 = v; }
		CHECKEQ(w63, 0x0407, "IDENTIFY word 63 after mdma2");
		CHECKEQ(w88, 0x003F, "IDENTIFY word 88 after mdma2");
		wb(R_ERR, 0x03);
		wb(R_NSEC, 0x08);  // pio mode 0
		wb(R_CMD, 0xEF); wait_irq(); rb(R_CMD);
		wb(R_ERR, 0x03);
		wb(R_NSEC, 0x30);  // invalid transfer mode class
		wb(R_CMD, 0xEF); wait_irq();
		CHECKEQ(rb(R_CMD), 0x41, "SET FEATURES 03 with bad mode aborts");
		wb(R_ERR, 0x77);
		wb(R_CMD, 0xEF); wait_irq();
		CHECKEQ(rb(R_CMD), 0x41, "unsupported feature aborts");
		CHECKEQ(rb(R_ERR), 0x04, "ABRT");
	}

	// ------------------------------------------------------------------
	section("INITIALIZE DEVICE PARAMETERS: 4 heads, 17 sectors");
	{
		wb(R_SEL, 0xA0 | 3);
		wb(R_NSEC, 17);
		wb(R_CMD, 0x91);
		CHECK(wait_irq(), "INITIALIZE irq");
		CHECKEQ(rb(R_CMD), 0x50, "INITIALIZE ok");
		Geo d = default_geo(nb0);
		Geo c = { (unsigned)(nb0 / (4 * 17)), 4, 17 };
		uint16_t exp[256], got[256];
		expected_identify(exp, 0, nb0, d, c, 16);
		wb(R_CMD, 0xEC); wait_irq(); rb(R_CMD);
		for (int i = 0; i < 256; i++) got[i] = rw(R_DATA);
		for (int i = 54; i <= 58; i++) CHECKEQ(got[i], exp[i], "IDENTIFY current geometry word");
		// CHS read with the new translation
		set_chs(10, 2, 5, 0, 1);
		wb(R_CMD, 0x20);
		wait_irq(); rb(R_CMD);
		uint64_t l = (10 * 4 + 2) * 17 + 5 - 1;
		int bad = 0;
		for (int i = 0; i < 256; i++) if (rw(R_DATA) != img_word(0, l, i, true)) bad++;
		CHECK(bad == 0, "CHS read with translated geometry (%d bad)", bad);
		CHECKEQ(hps->last_lba[0], l, "translated lba");
		wb(R_SEL, 0xA0 | 4);   // head 4 >= 4 heads
		wb(R_NSEC, 1);
		wb(R_CMD, 0x20); wait_irq();
		CHECKEQ(rb(R_ERR), 0x10, "head beyond the translated geometry: IDNF");
		wb(R_NSEC, 0);
		wb(R_CMD, 0x91); wait_irq();
		CHECKEQ(rb(R_CMD), 0x41, "INITIALIZE with 0 sectors aborts");
		// restore Hatari's default geometry
		wb(R_SEL, 0xA0 | 15);
		wb(R_NSEC, 63);
		wb(R_CMD, 0x91); wait_irq(); rb(R_CMD);
	}

	// ------------------------------------------------------------------
	section("nIEN masks INTRQ");
	{
		wb(R_ALT, 0x02);
		wb(R_CMD, 0x10);
		ticks(200);
		CHECKEQ(dut->irq, 0, "INTRQ masked by nIEN");
		wb(R_ALT, 0x00);
		tick();
		CHECKEQ(dut->irq, 1, "pending INTRQ appears when nIEN is cleared (ATA)");
		rb(R_CMD);
		CHECKEQ(dut->irq, 0, "status read clears it");
	}

	// ------------------------------------------------------------------
	section("EXECUTE DEVICE DIAGNOSTIC");
	{
		wb(R_SEL, 0xA5);
		wb(R_NSEC, 0x33);
		wb(R_LCYL, 0x44);
		wb(R_CMD, 0x90);
		CHECK(wait_irq(), "DIAGNOSTIC irq");
		CHECKEQ(rb(R_CMD), 0x50, "DIAGNOSTIC status DRDY|DSC (ATA)");
		CHECKEQ(rb(R_ERR), 0x01, "DIAGNOSTIC error code 01 (Hatari)");
		CHECKEQ(rb(R_NSEC), 0x01, "signature count");
		CHECKEQ(rb(R_SECT), 0x01, "signature sector");
		CHECKEQ(rb(R_LCYL), 0x00, "signature cyl low");
		CHECKEQ(rb(R_SEL), 0xA0, "head cleared (Hatari select &= 0xf0)");
	}

	// ------------------------------------------------------------------
	section("soft reset (SRST in device control)");
	{
		wb(R_SEL, 0xB0);       // select the slave first
		wb(R_NSEC, 0x77);
		wb(R_ALT, 0x04);
		CHECKEQ(rb(R_ALT) & 0x80, 0x80, "BSY while SRST is set (Hatari ide_ctrl_write)");
		CHECKEQ(rb(R_ERR), 0x90, "command block reads return status while BSY (ATA)");
		wb(R_ALT, 0x00);
		ticks(10);
		CHECKEQ(rb(R_ALT), 0x50, "DRDY|DSC after SRST (device 0 selected again)");
		CHECKEQ(rb(R_SEL), 0xA0, "device/head after SRST");
		CHECKEQ(rb(R_ERR), 0x01, "error 01 after SRST");
		CHECKEQ(rb(R_NSEC), 0x01, "signature count after SRST");
		CHECKEQ(rb(R_SECT), 0x01, "signature sector after SRST");
		CHECKEQ(rb(R_LCYL), 0x00, "signature cyl low after SRST");
		CHECKEQ(rb(R_HCYL), 0x00, "signature cyl high after SRST");
		wb(R_SEL, 0xB0);
		CHECKEQ(rb(R_ALT), 0x50, "slave DRDY|DSC after SRST");
		wb(R_SEL, 0xA0);
		// SRST during a data transfer ends it
		set_lba(50, 0, 2);
		wb(R_CMD, 0x20);
		wait_irq();
		rw(R_DATA);
		wb(R_ALT, 0x04);
		wb(R_ALT, 0x00);
		ticks(10);
		CHECKEQ(rb(R_ALT), 0x50, "SRST ended the transfer");
		CHECKEQ(rw(R_DATA), 0xFFFF, "no data after SRST");
		CHECKEQ(dut->irq, 0, "SRST cleared INTRQ");
	}

	// ------------------------------------------------------------------
	section("access sizes (Hatari Ide_Mem_*): word access to byte registers, byte access to data");
	{
		CHECKEQ(rw(0xF0001C), 0xFFFF, "word read at $F0001C reads $FFFF");
		CHECKEQ(rb(R_DATA), 0xFF, "byte read at $F00000 reads $FF");
		CHECKEQ(rb(0xF0001C), 0xFF, "byte read at the even address $F0001C reads $FF");
		ww(0xF00008, 0x1234);
		CHECKEQ(rb(R_NSEC), 0x01, "word write to $F00008 ignored");
		CHECKEQ(rb(0xF0003D), 0xFF, "unmapped register $F0003D reads $FF");
		// $F00002 is also the data register (Hatari Ide_Mem_wget)
		set_lba(60, 0, 1);
		wb(R_CMD, 0x20);
		wait_irq(); rb(R_CMD);
		int bad = 0;
		for (int i = 0; i < 256; i++) if (rw(i & 1 ? 0xF00002 : 0xF00000) != img_word(0, 60, i, true)) bad++;
		CHECK(bad == 0, "data at $F00000/$F00002 alternating (%d bad)", bad);
	}

	// ------------------------------------------------------------------
	section("peripheral reset (68030 RESET) keeps the mounted media");
	{
		// non default state that a reset must undo
		wb(R_SEL, 0xA0 | 3); wb(R_NSEC, 17); wb(R_CMD, 0x91); wait_irq(); rb(R_CMD);
		wb(R_NSEC, 4); wb(R_CMD, 0xC6); wait_irq(); rb(R_CMD);
		dut->reset = 1; ticks(5); dut->reset = 0; ticks(5);
		CHECKEQ(rb(R_ALT), 0x50, "master DRDY|DSC after reset, no new mount pulse");
		CHECKEQ(rb(R_ERR), 0x01, "diagnostic code after reset");
		CHECKEQ(rb(R_NSEC), 0x01, "signature after reset");
		CHECKEQ(rb(R_LCYL), 0x00, "signature cylinder after reset");
		wb(R_SEL, 0xB0);
		CHECKEQ(rb(R_ALT), 0x50, "slave still present after reset");
		wb(R_SEL, 0xA0);
		Geo d = default_geo(nb0);
		uint16_t exp[256], got[256];
		expected_identify(exp, 0, nb0, d, d, 16);
		wb(R_CMD, 0xEC);
		CHECK(wait_irq(), "IDENTIFY after reset");
		CHECKEQ(rb(R_CMD), 0x58, "IDENTIFY DRQ after reset");
		for (int i = 0; i < 256; i++) got[i] = rw(R_DATA);
		int bad = 0;
		for (int i = 0; i < 256; i++) if (got[i] != exp[i]) { if (bad < 5) printf("    word %d got %04x exp %04x\n", i, got[i], exp[i]); bad++; }
		CHECK(bad == 0, "IDENTIFY after reset: default geometry and multiple mode again (%d bad words)", bad);
		set_lba(123, 0, 1);
		wb(R_CMD, 0x20);
		CHECK(wait_irq(), "read after reset");
		rb(R_CMD);
		bad = 0;
		for (int i = 0; i < 256; i++) if (rw(R_DATA) != img_word(0, 123, i, true)) bad++;
		CHECK(bad == 0, "sector data after reset (%d bad)", bad);

		// reset while hps_io is transferring a sector to the core
		set_lba(200, 0, 2);
		wb(R_CMD, 0x20);
		int n = 0;
		while (!dut->sd_ack && n++ < 100000) tick();
		ticks(300);
		CHECK(dut->sd_ack, "a block transfer is running");
		dut->reset = 1; ticks(3); dut->reset = 0;
		n = 0;
		while ((dut->sd_ack || hps->st != HpsModel::IDLE) && n++ < 100000) tick();
		ticks(20);
		CHECKEQ(dut->sd_rd | dut->sd_wr, 0, "no request left after the absorbed transfer");
		CHECKEQ(dut->irq, 0, "no interrupt from the aborted command");
		CHECKEQ(rb(R_ALT), 0x50, "ready after reset in a transfer");
		set_lba(300, 0, 2);
		wb(R_CMD, 0x20);
		bad = 0;
		for (int k = 0; k < 2; k++) {
			CHECK(wait_irq(), "read after the interrupted transfer, sector %d", k);
			rb(R_CMD);
			for (int i = 0; i < 256; i++) if (rw(R_DATA) != img_word(0, 300 + k, i, true)) bad++;
		}
		CHECK(bad == 0, "data after the interrupted transfer (%d bad)", bad);

		// reset while the request is pending (sd_rd high, not yet acknowledged)
		set_lba(400, 0, 1);
		wb(R_CMD, 0x20);
		n = 0;
		while (!dut->sd_rd && n++ < 1000) tick();
		CHECK(dut->sd_rd && !dut->sd_ack, "request pending");
		dut->reset = 1; ticks(3); dut->reset = 0;
		n = 0;
		while ((dut->sd_rd || dut->sd_ack || hps->st != HpsModel::IDLE) && n++ < 100000) tick();
		ticks(20);
		set_lba(401, 0, 1);
		wb(R_CMD, 0x20);
		CHECK(wait_irq(), "read after reset with a pending request");
		rb(R_CMD);
		bad = 0;
		for (int i = 0; i < 256; i++) if (rw(R_DATA) != img_word(0, 401, i, true)) bad++;
		CHECK(bad == 0, "data (%d bad)", bad);

		// reset during a write transfer from the core to hps_io
		set_lba(500, 0, 1);
		wb(R_CMD, 0x30);
		wait_notbusy();
		for (int i = 0; i < 256; i++) ww(R_DATA, (uint16_t)(0x7700 + i));
		n = 0;
		while (!dut->sd_ack && n++ < 100000) tick();
		ticks(200);
		dut->reset = 1; ticks(3); dut->reset = 0;
		n = 0;
		while ((dut->sd_ack || hps->st != HpsModel::IDLE) && n++ < 100000) tick();
		ticks(20);
		CHECKEQ(hps->img[0].get(500, 0), 0x77, "the block hps_io was serving is written completely");
		CHECKEQ(hps->img[0].get(500, 511), 0xFF, "last byte of that block");
		set_lba(501, 0, 1);
		wb(R_CMD, 0x30);
		wait_notbusy();
		for (int i = 0; i < 256; i++) ww(R_DATA, (uint16_t)(0x1100 + i));
		CHECK(wait_irq(), "write after reset");
		CHECKEQ(rb(R_CMD), 0x50, "write status");
		CHECKEQ(hps->img[0].get(501, 2), 0x11, "write after reset reached the image");

		// a mount pulse while reset is held is latched
		const uint64_t nb2 = 30240;
		hps->img[1].readonly = false;
		dut->reset = 1;
		ticks(3);
		dut->img_size = nb2 * 512;
		dut->img_readonly = 0;
		mnt_pulse = 2;
		ticks(10);
		dut->reset = 0;
		ticks(10);
		wb(R_SEL, 0xB0);
		CHECK(wait_notbusy() == 0x50, "slave remounted from a pulse during reset");
		wb(R_CMD, 0xEC);
		wait_irq(); rb(R_CMD);
		uint16_t w60 = 0, w61 = 0;
		for (int i = 0; i < 256; i++) { uint16_t v = rw(R_DATA); if (i == 60) w60 = v; if (i == 61) w61 = v; }
		CHECKEQ(w60 | (w61 << 16), nb2, "new slave capacity");
		wb(R_SEL, 0xA0);
		CHECKEQ(hps->errors, 0, "hps protocol errors after the resets");
	}

	// ------------------------------------------------------------------
	section("new command during a data transfer, unmount");
	{
		set_lba(70, 0, 2);
		wb(R_CMD, 0x20);
		wait_irq(); rb(R_CMD);
		for (int i = 0; i < 100; i++) rw(R_DATA);
		set_lba(80, 0, 1);
		wb(R_CMD, 0x20);
		wait_irq(); rb(R_CMD);
		int bad = 0;
		for (int i = 0; i < 256; i++) if (rw(R_DATA) != img_word(0, 80, i, true)) bad++;
		CHECK(bad == 0, "new command replaces the old transfer (%d bad)", bad);
		mount(1, 0, false);
		ticks(10);
		wb(R_SEL, 0xB0);
		CHECKEQ(rb(R_CMD), 0x00, "unmounted slave reads as absent");
		wb(R_SEL, 0xA0);
		CHECKEQ(rb(R_CMD), 0x50, "master unaffected");
	}

	CHECKEQ(hps->errors, 0, "hps model protocol errors");
	printf("\nIDE: %d checks, %d failures, %llu cycles\n", checks, fails, (unsigned long long)cycle);
	printf(fails ? "IDE TEST FAIL\n" : "IDE TEST PASS\n");
	delete dut;
	return fails ? 1 : 0;
}
