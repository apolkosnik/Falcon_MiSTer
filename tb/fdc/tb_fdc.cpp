// tb_fdc.cpp - Verilator testbench for rtl/falcon/falcon_fdc.sv
//
// The real falcon_fdc is driven through the common register bus; its DMA
// port is served by a RAM model with variable latency and its two .ST images
// by the hps_io model (../ide/hps_disk_model.h).  Expected values come from
// Hatari src/fdc.c and src/floppy.c: status bits per command type, the
// standard track layout and CRC of FDC_ReadTrack_ST / FDC_ReadAddress_ST,
// the ID search of FDC_NextSectorID_FdcCycles_ST, the DMA FIFO rules of
// FDC_DMA_FIFO_Push/Pull and the delays (prepare 90 us, step rates, head
// settle 15 ms, spin up 6 index pulses, motor off 9 index pulses).
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>
#include "Vfalcon_fdc.h"
#include "verilated.h"
#include "../ide/hps_disk_model.h"

#ifndef TB_CLK_HZ
#define TB_CLK_HZ 32000000
#endif

static Vfalcon_fdc *dut;
static HpsModel *hps;
static uint64_t cycle = 0;
static int fails = 0, checks = 0;

#define CHECK(cond, ...) do { checks++; if (!(cond)) { fails++; printf("FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)
#define CHECKEQ(got, exp, what) do { checks++; if ((uint64_t)(got) != (uint64_t)(exp)) { fails++; \
	printf("FAIL: %s: got 0x%llx expected 0x%llx\n", what, (unsigned long long)(got), (unsigned long long)(exp)); } } while (0)

// FDC cycles (8 MHz) -> clocks
static uint64_t fdc2clk(uint64_t c) { return c * TB_CLK_HZ / 8000000ull; }
static uint64_t ms2clk(double ms) { return (uint64_t)(ms * TB_CLK_HZ / 1000.0); }

// ---------------------------------------------------------------------------
// RAM model on the DMA port (16 MB, big endian words)
static std::vector<uint8_t> ram(16 << 20);
static int dma_lat = 0;
static bool dma_busy = false;
static uint32_t rng = 777;
static int rnd(int lo, int hi) { rng = rng * 1103515245u + 12345u; return lo + (int)((rng >> 8) % (uint32_t)(hi - lo + 1)); }
static uint64_t dma_words = 0;

static uint32_t mnt_pulse = 0;

static void tick() {
	if (dut->sd_lba0 != dut->sd_lba1 || dut->sd_buff_din0 != dut->sd_buff_din1) { printf("FAIL: sd_lba0/1 or sd_buff_din0/1 differ\n"); fails++; }
	hps->step(dut->sd_rd, dut->sd_wr, dut->sd_lba0, dut->sd_buff_din0);
	// DMA: latency 6..20 clocks, ack one clock
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
		dma_words++;
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
	mnt_pulse = 0;
	dut->clk = 0;
	dut->eval();
}
static void ticks(uint64_t n) { while (n--) tick(); }

static bool last_berr = false;
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
		if (dut->bus_ack) { r = dut->bus_dout; last_berr = dut->bus_berr; break; }
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

enum { DMA_DATA = 0xFF8604, DMA_MODE = 0xFF8606, DMA_HI = 0xFF8609, DMA_MID = 0xFF860B, DMA_LO = 0xFF860D,
       DENS = 0xFF860E };

// FDC register helpers (TOS style: select with $ff8606 then access $ff8604)
static uint16_t dmode_cur = 0x80;
static void fdc_wr(int reg, uint8_t v) { dmode_cur = (dmode_cur & 0x100) | 0x80 | (reg << 1); ww(DMA_MODE, dmode_cur); ww(DMA_DATA, v); }
static uint8_t fdc_rd(int reg) { dmode_cur = (dmode_cur & 0x100) | 0x80 | (reg << 1); ww(DMA_MODE, dmode_cur); return rw(DMA_DATA) & 0xFF; }
static uint8_t fdc_status() { return fdc_rd(0); }

static void set_dma_addr(uint32_t a) { wb(DMA_LO, a & 0xFF); wb(DMA_MID, (a >> 8) & 0xFF); wb(DMA_HI, (a >> 16) & 0xFF); }
static uint32_t get_dma_addr() { return (rb(DMA_HI) << 16) | (rb(DMA_MID) << 8) | rb(DMA_LO); }

// DMA read (disk -> RAM): toggle the direction to clear the FIFO, set the count
static void dma_read_setup(uint32_t addr, unsigned count) {
	set_dma_addr(addr);
	ww(DMA_MODE, 0x190); ww(DMA_MODE, 0x090);
	ww(DMA_DATA, count);
	dmode_cur = 0x080;
	ww(DMA_MODE, dmode_cur);
}
static void dma_write_setup(uint32_t addr, unsigned count) {
	set_dma_addr(addr);
	ww(DMA_MODE, 0x090); ww(DMA_MODE, 0x190);
	ww(DMA_DATA, count);
	dmode_cur = 0x180;
	ww(DMA_MODE, dmode_cur);
}

static bool wait_irq(double max_ms) {
	uint64_t n = ms2clk(max_ms);
	for (uint64_t i = 0; i < n; i++) { if (dut->irq) return true; tick(); }
	return false;
}

static void mount(int u, uint64_t size, bool ro) {
	hps->img[u].mounted = size != 0;
	hps->img[u].size = size;
	hps->img[u].readonly = ro;
	dut->img_size = size;
	dut->img_readonly = ro;
	mnt_pulse = 1u << u;
	ticks(20000);
}

// PSG port A: bit 0 = side (1 = side 0), bit 1 = drive A (0 = selected), bit 2 = drive B
static void select_drive(int d, int side) {
	dut->drv_sel = (d == 0) ? 2 : (d == 1) ? 1 : 3;
	dut->side_sel = side ? 0 : 1;
}

// ---------------------------------------------------------------------------
// Golden models (Hatari)
static void crc16_add_byte(uint16_t *crc, uint8_t c) {      // utils.c
	*crc ^= (c << 8);
	for (int bit = 0; bit < 8; bit++)
		*crc = (*crc & 0x8000) ? (*crc << 1) ^ 0x1021 : (*crc << 1);
}
// FDC_ReadAddress_ST: 6 bytes
static void golden_id(uint8_t *out, int track, int side, int sector) {
	uint8_t b[8] = { 0xa1, 0xa1, 0xa1, 0xfe, (uint8_t)track, (uint8_t)side, (uint8_t)sector, 2 };
	uint16_t crc = 0xffff;
	for (int i = 0; i < 8; i++) crc16_add_byte(&crc, b[i]);
	out[0] = track; out[1] = side; out[2] = sector; out[3] = 2; out[4] = crc >> 8; out[5] = crc & 0xff;
}
struct Geo { int spt, sides, tracks; };
static uint64_t st_lba(Geo g, int track, int side, int sector) { return ((uint64_t)track * g.sides + side) * g.spt + sector - 1; }
// FDC_ReadTrack_ST (track size 6250 bytes per DD track here, see the module header)
static std::vector<uint8_t> golden_track(int u, Geo g, int track, int side, int bpt) {
	std::vector<uint8_t> t;
	for (int i = 0; i < 60; i++) t.push_back(0x4e);
	for (int s = 1; s <= g.spt; s++) {
		for (int i = 0; i < 12; i++) t.push_back(0x00);
		uint8_t id[10] = { 0xa1, 0xa1, 0xa1, 0xfe, (uint8_t)track, (uint8_t)side, (uint8_t)s, 2 };
		uint16_t crc = 0xffff;
		for (int i = 0; i < 8; i++) crc16_add_byte(&crc, id[i]);
		id[8] = crc >> 8; id[9] = crc & 0xff;
		for (int i = 0; i < 10; i++) t.push_back(id[i]);
		for (int i = 0; i < 22; i++) t.push_back(0x4e);
		for (int i = 0; i < 12; i++) t.push_back(0x00);
		crc = 0xffff;
		for (int i = 0; i < 3; i++) { t.push_back(0xa1); crc16_add_byte(&crc, 0xa1); }
		t.push_back(0xfb); crc16_add_byte(&crc, 0xfb);
		for (int i = 0; i < 512; i++) { uint8_t b = hps->img[u].get(st_lba(g, track, side, s), i); t.push_back(b); crc16_add_byte(&crc, b); }
		t.push_back(crc >> 8); t.push_back(crc & 0xff);
		for (int i = 0; i < 40; i++) t.push_back(0x4e);
	}
	while ((int)t.size() < bpt) t.push_back(0x4e);
	return t;
}

static void make_st(int u, uint32_t seed, int tracks, int spt, int sides) {
	hps->img[u].seed = seed;
	uint8_t s0[512];
	for (int i = 0; i < 512; i++) s0[i] = HpsImage::pat(seed, 0, i);
	unsigned total = tracks * spt * sides;
	s0[19] = total & 0xff; s0[20] = total >> 8;
	s0[24] = spt; s0[25] = 0;
	s0[26] = sides; s0[27] = 0;
	hps->img[u].preset(0, s0);
}

static int test_no = 0;
static void section(const char *s) { printf("--- %d: %s  (t=%.1f ms)\n", ++test_no, s, cycle * 1000.0 / TB_CLK_HZ); }

static bool ram_matches_image(uint32_t addr, int u, uint64_t lba, int nsec, int *bad_out = nullptr) {
	int bad = 0;
	for (int s = 0; s < nsec; s++)
		for (int i = 0; i < 512; i++)
			if (ram[addr + s * 512 + i] != hps->img[u].get(lba + s, i)) bad++;
	if (bad_out) *bad_out = bad;
	return bad == 0;
}

static uint8_t wait_notbusy(double max_ms) {
	uint64_t end = cycle + ms2clk(max_ms);
	uint8_t st = 0;
	while (cycle < end) {
		st = fdc_status();
		if (!(st & 1)) return st;
		ticks(ms2clk(0.05));
	}
	printf("FAIL: FDC busy timeout\n");
	fails++;
	return st;
}

int main(int argc, char **argv) {
	Verilated::commandArgs(argc, argv);
	dut = new Vfalcon_fdc;
	hps = new HpsModel(2);
	hps->min_lat = 50; hps->max_lat = 2000;

	const Geo gA = { 9, 2, 80 };      // 720 KB DD
	const Geo gB = { 18, 2, 80 };     // 1.44 MB HD
	make_st(0, 0xA0A0, 80, 9, 2);
	make_st(1, 0xB0B0, 80, 18, 2);

	select_drive(-1, 0);
	dut->reset = 1;
	ticks(5);
	dut->reset = 0;
	ticks(5);

	if (argc > 1 && !strcmp(argv[1], "frac")) {
		// built with CLK_HZ = 28636363: fractional 8 MHz FDC clock enable
		section("fractional FDC clock enable (CLK_HZ = 28636363)");
		mount(0, 80 * 9 * 2 * 512, false);
		select_drive(0, 0);
		fdc_wr(0, 0x07);
		CHECK(wait_irq(2500), "restore with verify irq");
		CHECKEQ(fdc_status() & 0x3D, 0x24, "restore status");
		fdc_wr(3, 4);
		uint64_t t0 = cycle;
		fdc_wr(0, 0x1B);
		CHECK(wait_irq(100), "seek irq");
		double tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		printf("    seek 4 tracks at 3 ms: %.3f ms\n", tms);
		CHECK(tms > 12.08 && tms < 12.12, "step timing with the fractional enable");
		fdc_status();
		uint32_t a = 0x2000;
		dma_read_setup(a, 1);
		fdc_wr(2, 9);
		fdc_wr(0, 0x80);
		CHECK(wait_irq(400), "read sector irq");
		fdc_status();
		ticks(ms2clk(0.05));
		int bad;
		CHECK(ram_matches_image(a, 0, st_lba(gA, 4, 0, 9), 1, &bad), "sector data (%d bad)", bad);
		fdc_wr(0, 0xD4);
		CHECK(wait_irq(300), "index irq");
		uint64_t t1 = cycle;
		fdc_status();
		CHECK(wait_irq(300), "next index irq");
		double per = (cycle - t1) * 1000.0 / TB_CLK_HZ;
		printf("    index period %.3f ms\n", per);
		CHECK(per > 199.9 && per < 200.1, "300 rpm");
		fdc_wr(0, 0xD0); fdc_status();
		printf("\nFDC(frac): %d checks, %d failures\n", checks, fails);
		printf(fails ? "FDC TEST FAIL\n" : "FDC TEST PASS\n");
		delete dut;
		return fails ? 1 : 0;
	}

	if (argc > 1 && !strcmp(argv[1], "absent_b")) {
		// built with DRV_PRESENT = 2'b01: drive B is not connected
		section("drive B not connected (DRV_PRESENT = 01), image mounted in slot B");
		mount(1, 80 * 18 * 2 * 512, false);
		select_drive(1, 0);
		uint8_t st = fdc_status();
		fdc_wr(0, 0xD0);
		ticks(ms2clk(0.2));
		st = fdc_status();
		CHECKEQ(st & 0x46, 0, "drive B not connected: TR00, INDEX, WPRT off");
		fdc_wr(0, 0x0B);
		CHECK(wait_irq(900), "restore irq");
		CHECKEQ(fdc_status() & 0x10, 0x10, "restore on a missing drive: RNF");
		dma_read_setup(0x1000, 1);
		fdc_wr(2, 1);
		fdc_wr(0, 0x80);
		CHECK(!wait_irq(1500), "read sector on a missing drive waits (no index pulses)");
		fdc_wr(0, 0xD0);
		ticks(ms2clk(0.2));
		CHECKEQ(fdc_status() & 1, 0, "D0 ends it");
		select_drive(0, 0);
		mount(0, 80 * 9 * 2 * 512, false);
		fdc_wr(0, 0x07);
		CHECK(wait_irq(2500), "drive A still works");
		CHECKEQ(fdc_status() & 0x14, 0x04, "drive A restore ok");
		printf("\nFDC(absent B): %d checks, %d failures\n", checks, fails);
		printf(fails ? "FDC TEST FAIL\n" : "FDC TEST PASS\n");
		delete dut;
		return fails ? 1 : 0;
	}

	// ------------------------------------------------------------------
	section("DMA chip registers (FDC_WriteDMAAddress, FDC_DmaStatus_ReadWord, $FF860E)");
	{
		wb(DMA_HI, 0xFF); wb(DMA_MID, 0xFF); wb(DMA_LO, 0xFF);
		CHECKEQ(rb(DMA_HI), 0xFF, "Falcon: DMA address high byte not masked (24 bit)");
		CHECKEQ(rb(DMA_LO), 0xFE, "DMA address bit 0 forced to 0");
		CHECKEQ(rw(0xFF8608), 0xFFFF, "word read $FF8608: void even byte + high address byte");
		set_dma_addr(0x123456);
		CHECKEQ(get_dma_addr(), 0x123456, "DMA address read back");
		CHECKEQ(rw(0xFF860C), 0xFF56, "word read $FF860C");
		ww(DENS, 0x0003);
		CHECKEQ(rw(DENS), 0x0003, "density register");
		ww(DENS, 0x0000);
		CHECKEQ(rw(DMA_MODE) & 7, 1, "DMA status after reset: no error, count 0, no DRQ");
		// sector count is write only: reads return the latest $ff8604 value
		fdc_wr(1, 0x5A);                     // track register, recent = $xx5A
		ww(DMA_MODE, 0x90);
		ww(DMA_DATA, 0x0003);
		CHECKEQ(rw(DMA_DATA), 0x005A, "sector count read returns ff8604_recent_val");
		CHECKEQ(rw(DMA_MODE), 0x005A | 0x3, "status: unused bits from recent value, bit 1 = count != 0");
		ww(DMA_MODE, 0x190);                 // toggle: count 0, FIFO empty
		CHECKEQ(rw(DMA_MODE) & 2, 0, "direction toggle clears the sector count");
		ww(DMA_MODE, 0x80);
		CHECKEQ(fdc_rd(1), 0x5A, "track register");
		bus(true, DMA_DATA + 1, false, true, 0x12);
		CHECK(last_berr, "byte write to $FF8605 bus errors (Hatari FDC_DiskController_WriteWord)");
		bus(true, DMA_MODE, true, false, 0x1200);
		CHECK(last_berr, "byte write to $FF8606 bus errors");
		CHECKEQ(rb(DMA_DATA + 1), 0x5A, "byte read of $FF8605 allowed on the Falcon");
		bus(true, DMA_DATA, true, true, 0x80);
		CHECK(!last_berr, "word write no bus error");
		fdc_wr(1, 0);
	}

	// ------------------------------------------------------------------
	section("no disk: restore on drive A (no image) - type I status");
	{
		select_drive(0, 0);
		// with spin-up the WD1772 waits for 6 index pulses: none without a disk
		fdc_wr(0, 0x03);                     // restore, spin-up, no verify, 3 ms
		CHECK(fdc_status() & 1, "busy after the command");
		CHECK(!wait_irq(1500), "restore with spin-up never completes without index pulses");
		uint8_t s0 = fdc_status();
		CHECKEQ(s0 & 0xA1, 0x81, "busy, motor on, spin-up not reached");
		fdc_wr(0, 0xD0);
		ticks(ms2clk(0.2));
		CHECKEQ(fdc_status() & 1, 0, "D0 terminates it");
		uint64_t t0 = cycle;
		fdc_wr(0, 0x0B);                     // restore, no spin-up, no verify, 3 ms
		CHECK(wait_irq(2000), "restore irq with no disk");
		double tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		uint8_t st = fdc_status();
		printf("    restore no disk: %.2f ms, status %02x\n", tms, st);
		CHECKEQ(st & 0x01, 0, "not busy");
		CHECKEQ(st & 0x04, 0x04, "TR00 (head at track 0)");
		CHECKEQ(st & 0x40, 0x40, "WPRT: no disk reads as write protected");
		CHECKEQ(st & 0x80, 0x80, "motor on");
		CHECKEQ(st & 0x20, 0x20, "spin-up bit set when the type I command proceeds (Hatari)");
		CHECK(tms > 0.09 && tms < 0.2, "restore at track 0: prepare 90 us + complete");
		CHECKEQ(dut->irq, 0, "status read clears the irq");
	}

	// ------------------------------------------------------------------
	section("mount A (720K DD) and B (1.44M HD); force interrupt to stop the motor sequence");
	mount(0, 80 * 9 * 2 * 512, false);
	mount(1, 80 * 18 * 2 * 512, false);
	{
		fdc_wr(0, 0xD0);
		ticks(ms2clk(0.2));
		uint8_t st = fdc_status();
		CHECKEQ(st & 0x01, 0, "D0 while idle: not busy");
		CHECK((st & 0x80), "D0 while idle sets the motor bit (Hatari)");
	}

	// ------------------------------------------------------------------
	section("restore with spin-up and verify on drive A");
	{
		select_drive(0, 0);
		// wait for the motor to stop first: 9 index pulses
		uint64_t t0 = cycle;
		while (fdc_status() & 0x80) { ticks(ms2clk(5)); if (cycle - t0 > ms2clk(3000)) break; }
		double off_ms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		printf("    motor stopped after %.1f ms (9 revolutions = 1800 ms)\n", off_ms);
		// 9 index pulses counted from an arbitrary rotation phase: 8..9 revolutions
		CHECK(off_ms > 1600 && off_ms < 1810, "motor off after 9 index pulses");
		CHECKEQ(fdc_status() & 0x80, 0, "motor bit off");
		t0 = cycle;
		fdc_wr(0, 0x07);                     // restore, spin-up, verify, 3 ms
		uint8_t st = fdc_status();
		CHECKEQ(st & 0x81, 0x01, "busy, motor still off during the 90 us prepare delay");
		ticks(ms2clk(0.2));
		st = fdc_status();
		CHECKEQ(st & 0x21, 0x01, "busy, spin-up bit cleared during spin-up");
		CHECKEQ(st & 0x80, 0x80, "motor on during spin-up");
		bool seen_index = false;
		while (!dut->irq && cycle - t0 < ms2clk(2000)) {
			uint8_t s2 = fdc_status();
			if (s2 & 0x02) seen_index = true;
			ticks(ms2clk(0.5));
		}
		CHECK(dut->irq, "restore with verify irq");
		double tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		printf("    restore+verify: %.1f ms\n", tms);
		CHECK(tms > 1015 && tms < 1250, "6 index pulses spin-up + 15 ms head settle + ID");
		CHECK(seen_index, "index pulse seen in the type I status");
		st = fdc_status();
		CHECKEQ(st & 0x3D, 0x24, "spin-up done, TR00, no RNF/CRC, not busy");
		CHECKEQ(st & 0x40, 0, "WPRT off for a writable disk");
		CHECKEQ(fdc_rd(1), 0, "track register 0");
	}

	// ------------------------------------------------------------------
	section("seek with verify: timing of 10 steps at 3 ms and status");
	{
		fdc_wr(3, 10);
		uint64_t t0 = cycle;
		fdc_wr(0, 0x17);                     // seek, verify, 3 ms (motor already on)
		CHECK(wait_irq(500), "seek irq");
		double tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		// 90 us prepare + 10 x 3 ms + 15 ms head settle + up to one revolution to an ID field
		printf("    seek 10 tracks + verify: %.2f ms\n", tms);
		CHECK(tms > 45.0 && tms < 246.0, "seek time");
		uint8_t st = fdc_status();
		CHECKEQ(st & 0x1D, 0x00, "no RNF, not TR00, not busy");
		CHECKEQ(fdc_rd(1), 10, "track register 10");
		// no verify: exact step timing
		fdc_wr(3, 14);
		t0 = cycle;
		fdc_wr(0, 0x1B);                     // seek, no verify, 3 ms... rate 3 = 3 ms
		CHECK(wait_irq(100), "seek irq");
		tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		printf("    seek 4 tracks without verify: %.3f ms (expected 0.09 + 4 x 3 + 0.001)\n", tms);
		CHECK(tms > 12.08 && tms < 12.12, "type I prepare + 4 steps of 3 ms");
		fdc_status();
		fdc_wr(3, 10);
		fdc_wr(0, 0x18);                     // back to 10, 6 ms steps
		t0 = cycle;
		CHECK(wait_irq(100), "seek irq");
		tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		CHECK(tms > 24.08 && tms < 24.12, "4 steps of 6 ms (%.3f ms)", tms);
		fdc_status();
	}

	// ------------------------------------------------------------------
	section("step in / step out with and without track update");
	{
		fdc_wr(0, 0x5B);                     // step in, update, no verify
		wait_irq(50); fdc_status();
		CHECKEQ(fdc_rd(1), 11, "step in with update: TR 11");
		fdc_wr(0, 0x4B);                     // step in, no update
		wait_irq(50); fdc_status();
		CHECKEQ(fdc_rd(1), 11, "step in without update: TR unchanged");
		fdc_wr(0, 0x7B);                     // step out, update
		wait_irq(50); fdc_status();
		CHECKEQ(fdc_rd(1), 10, "step out with update: TR 10");
		fdc_wr(0, 0x3B);                     // step (same direction: out), update
		wait_irq(50); fdc_status();
		CHECKEQ(fdc_rd(1), 9, "step uses the last direction");
		// head is at 11 now (in, in, out, out => 10+1+1-1-1 = 10); TR = 9
		fdc_wr(1, 10);
		fdc_wr(0, 0x1F);                     // seek 10 with verify -> no movement
		wait_irq(300);
		CHECKEQ(fdc_status() & 0x10, 0, "verify on track 10 ok");
	}

	// ------------------------------------------------------------------
	section("read address x8: ID fields through the DMA FIFO (FDC_ReadAddress_ST)");
	{
		uint32_t a = 0x20000;
		memset(&ram[a], 0xEE, 64);
		dma_read_setup(a, 1);
		int prev = -1;
		bool order_ok = true, crc_ok = true;
		for (int k = 0; k < 8; k++) {
			fdc_wr(0, 0xC0);
			CHECK(wait_irq(300), "read address irq %d", k);
			ticks(ms2clk(0.05));             // the DMA writes the FIFO to RAM
			uint8_t st = fdc_status();
			CHECKEQ(st & 0x1D, 0, "read address status");
		}
		CHECKEQ(get_dma_addr(), a + 48, "48 bytes -> 3 FIFO flushes of 16 bytes");
		for (int k = 0; k < 8; k++) {
			uint8_t exp[6];
			uint8_t *g = &ram[a + 6 * k];
			if (k < 8 && 6 * k + 6 <= 48) {
				golden_id(exp, 10, 0, g[2]);
				if (memcmp(exp, g, 6)) crc_ok = false;
				if (prev >= 0 && g[2] != (prev % 9) + 1) order_ok = false;
				prev = g[2];
			}
		}
		CHECK(crc_ok, "ID fields: track 10, side 0, size 2, CRC as Hatari FDC_CRC16");
		CHECK(order_ok, "consecutive read address commands return consecutive sectors");
		CHECKEQ(fdc_rd(2), 10, "sector register = track number of the ID (WD1772)");
		CHECKEQ(ram[a + 48], 0xEE, "bytes beyond the last full FIFO block not written");
	}

	// ------------------------------------------------------------------
	section("read sector: single sector into RAM, DMA address and count");
	{
		uint32_t a = 0x30000;
		dma_read_setup(a, 1);
		fdc_wr(2, 3);
		uint64_t t0 = cycle;
		fdc_wr(0, 0x80);
		CHECK(wait_irq(400), "read sector irq");
		double tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		ticks(ms2clk(0.05));
		uint8_t st = fdc_status();
		CHECKEQ(st & 0x7F, 0x00, "read sector status (no RNF/CRC/LOST/RT)");
		CHECKEQ(st & 0x80, 0x80, "motor on");
		int bad;
		CHECK(ram_matches_image(a, 0, st_lba(gA, 10, 0, 3), 1, &bad), "sector data (%d bad bytes)", bad);
		CHECKEQ(get_dma_addr(), a + 512, "DMA address advanced by 512");
		CHECKEQ(rw(DMA_MODE) & 3, 1, "DMA status: no error, sector count 0");
		CHECKEQ(hps->last_lba[0], st_lba(gA, 10, 0, 3), "image lba");
		printf("    read sector: %.2f ms (512 bytes at 32 us = 16.4 ms + rotation)\n", tms);
		CHECK(tms > 16.5 && tms < 220, "read sector duration");
	}

	// ------------------------------------------------------------------
	section("read sector on side 1");
	{
		select_drive(0, 1);
		uint32_t a = 0x31000;
		dma_read_setup(a, 1);
		fdc_wr(2, 7);
		fdc_wr(0, 0x80);
		CHECK(wait_irq(400), "read sector side 1 irq");
		CHECKEQ(fdc_status() & 0x1D, 0, "status");
		int bad;
		CHECK(ram_matches_image(a, 0, st_lba(gA, 10, 1, 7), 1, &bad), "side 1 data (%d bad)", bad);
		select_drive(0, 0);
	}

	// ------------------------------------------------------------------
	section("read multiple sectors 8, 9 then RNF on sector 10 (5 revolutions)");
	{
		uint32_t a = 0x40000;
		dma_read_setup(a, 2);
		fdc_wr(2, 8);
		uint64_t t0 = cycle;
		fdc_wr(0, 0x90);
		CHECK(wait_irq(1500), "multiple read irq");
		double tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		uint8_t st = fdc_status();
		printf("    read multiple with RNF: %.1f ms, status %02x\n", tms, st);
		CHECKEQ(st & 0x11, 0x10, "RNF set when sector 10 is not found");
		// sector 9 ends shortly before the index: 4 more revolutions + the search for sector 8
		CHECK(tms > 800 && tms < 1050, "RNF after 5 index pulses");
		int bad;
		CHECK(ram_matches_image(a, 0, st_lba(gA, 10, 0, 8), 2, &bad), "sectors 8,9 (%d bad)", bad);
		CHECKEQ(get_dma_addr(), a + 1024, "DMA address after 2 sectors");
		CHECKEQ(fdc_rd(2), 10, "sector register incremented to 10");
	}

	// ------------------------------------------------------------------
	section("read multiple stopped by force interrupt; DMA count exhausted -> DMA error");
	{
		uint32_t a = 0x50000;
		memset(&ram[a], 0x11, 4096);
		dma_read_setup(a, 3);
		fdc_wr(2, 1);
		fdc_wr(0, 0x90);
		uint64_t t0 = cycle;
		// poll the DMA address counter as it advances
		uint32_t last = a, seen = 0;
		bool mono = true;
		while (cycle - t0 < ms2clk(400)) {
			uint32_t d = get_dma_addr();
			if (d < last) mono = false;
			if (d != last) seen++;
			last = d;
			if (d >= a + 4 * 512) break;
			ticks(ms2clk(0.3));
		}
		CHECK(mono && seen > 20, "DMA address counter readable while it advances (%u changes)", seen);
		ticks(ms2clk(40));                   // 4th sector is read but the count is 0
		fdc_wr(0, 0xD0);
		ticks(ms2clk(0.2));
		uint8_t st = fdc_status();
		CHECKEQ(st & 0x01, 0, "force interrupt ends the command");
		CHECKEQ(dut->irq, 0, "D0 gives no interrupt");
		int bad;
		CHECK(ram_matches_image(a, 0, st_lba(gA, 10, 0, 1), 3, &bad), "3 sectors in RAM (%d bad)", bad);
		CHECKEQ(get_dma_addr(), a + 1536, "DMA stops after the sector count");
		CHECKEQ(ram[a + 1536], 0x11, "nothing written past the count");
		CHECKEQ(rw(DMA_MODE) & 3, 0, "DMA status: error (bytes with count 0), count 0");
	}

	// ------------------------------------------------------------------
	section("DMA direction toggle clears a partly filled FIFO and the error status");
	{
		CHECKEQ(rw(DMA_MODE) & 1, 0, "DMA error from the previous test");
		uint32_t a = 0x58000;
		dma_read_setup(0x57000, 1);
		CHECKEQ(rw(DMA_MODE) & 1, 1, "toggle sets the no-error bit (DMA documentation)");
		fdc_wr(0, 0xC0);                     // 6 ID bytes stay in the FIFO
		wait_irq(300); fdc_status();
		ticks(ms2clk(0.05));
		CHECKEQ(get_dma_addr(), 0x57000, "6 bytes are not written to RAM");
		dma_read_setup(a, 1);                // toggle: FIFO emptied
		fdc_wr(2, 4);
		fdc_wr(0, 0x80);
		CHECK(wait_irq(400), "read sector irq");
		fdc_status();
		ticks(ms2clk(0.05));
		int bad;
		CHECK(ram_matches_image(a, 0, st_lba(gA, 10, 0, 4), 1, &bad), "sector aligned after the toggle (%d bad)", bad);
		CHECKEQ(get_dma_addr(), a + 512, "DMA address");
	}

	// ------------------------------------------------------------------
	section("index pulse length in the type I status (3.71 ms) and head load delay (15 ms)");
	{
		fdc_wr(0, 0xD4);
		CHECK(wait_irq(300), "index interrupt");
		uint64_t t0 = cycle;
		fdc_wr(0, 0xD0);
		fdc_status();
		// the INDEX bit is high from the index pulse for 3.71 ms
		uint64_t hi_end = 0;
		while (cycle - t0 < ms2clk(6)) {
			if (!(fdc_status() & 2)) { hi_end = cycle; break; }
		}
		double len = (hi_end - t0) * 1000.0 / TB_CLK_HZ;
		printf("    INDEX bit high for %.2f ms after the index interrupt\n", len);
		CHECK(hi_end && len > 3.6 && len < 3.75, "index pulse length");
		// read track started 190 ms after the index: with the 15 ms head load it
		// misses the next index and waits one more revolution
		while (cycle - t0 < ms2clk(190)) tick();
		dma_read_setup(0xE0000, 13);
		fdc_wr(0, 0xE4);                     // read track, head load
		CHECK(wait_irq(700), "read track with head load irq");
		double tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		fdc_status();
		printf("    read track with head load: done %.1f ms after the index\n", tms);
		CHECK(tms > 599 && tms < 602, "15 ms head load: track read from the index at 400 ms");
		fdc_wr(0, 0xD4);
		wait_irq(300);
		t0 = cycle;
		fdc_wr(0, 0xD0);
		fdc_status();
		while (cycle - t0 < ms2clk(190)) tick();
		dma_read_setup(0xE0000, 13);
		fdc_wr(0, 0xE0);                     // read track, no head load
		CHECK(wait_irq(700), "read track irq");
		tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		fdc_status();
		printf("    read track without head load: done %.1f ms after the index\n", tms);
		CHECK(tms > 399 && tms < 402, "no head load: track read from the index at 200 ms");
	}

	// ------------------------------------------------------------------
	section("write sector with DMA prefetch (2 x 16 byte FIFO)");
	{
		uint32_t a = 0x60000;
		for (int i = 0; i < 512; i++) ram[a + i] = (uint8_t)(i * 7 + 3);
		dma_write_setup(a, 1);
		ticks(2000);
		CHECKEQ(get_dma_addr(), a + 32, "write mode prefetches two FIFOs (32 bytes)");
		fdc_wr(2, 5);
		fdc_wr(0, 0xA0);
		CHECK(wait_irq(400), "write sector irq");
		uint8_t st = fdc_status();
		CHECKEQ(st & 0x5D, 0, "write sector status");
		uint64_t l = st_lba(gA, 10, 0, 5);
		int bad = 0;
		for (int i = 0; i < 512; i++) if (hps->img[0].get(l, i) != (uint8_t)(i * 7 + 3)) bad++;
		CHECK(bad == 0, "image sector written (%d bad)", bad);
		CHECKEQ(get_dma_addr(), a + 512, "DMA address after the write");
		CHECKEQ(rw(DMA_MODE) & 3, 1, "no DMA error, count 0");
		// read back
		dma_read_setup(0x61000, 1);
		fdc_wr(2, 5);
		fdc_wr(0, 0x80);
		wait_irq(400);
		fdc_status();
		bad = 0;
		for (int i = 0; i < 512; i++) if (ram[0x61000 + i] != (uint8_t)(i * 7 + 3)) bad++;
		CHECK(bad == 0, "read back (%d bad)", bad);
	}

	// ------------------------------------------------------------------
	section("write multiple sectors 2..4 on side 1");
	{
		select_drive(0, 1);
		uint32_t a = 0x62000;
		for (int i = 0; i < 3 * 512; i++) ram[a + i] = (uint8_t)(i ^ 0x5A);
		dma_write_setup(a, 3);
		fdc_wr(2, 2);
		fdc_wr(0, 0xB0);
		uint64_t t0 = cycle;
		while (get_dma_addr() < a + 3 * 512 && cycle - t0 < ms2clk(800)) ticks(ms2clk(1));
		// the third sector is fetched; let the FDC finish it, then stop the multi command
		while (hps->ops_wr < 1000 && cycle - t0 < ms2clk(900)) {
			bool done = true;
			for (int k = 0; k < 3; k++) if (hps->img[0].get(st_lba(gA, 10, 1, 2 + k), 511) != (uint8_t)((k * 512 + 511) ^ 0x5A)) done = false;
			if (done) break;
			ticks(ms2clk(1));
		}
		fdc_wr(0, 0xD0);
		ticks(ms2clk(0.2));
		fdc_status();
		int bad = 0;
		for (int k = 0; k < 3; k++) for (int i = 0; i < 512; i++)
			if (hps->img[0].get(st_lba(gA, 10, 1, 2 + k), i) != (uint8_t)((k * 512 + i) ^ 0x5A)) bad++;
		CHECK(bad == 0, "3 sectors written (%d bad)", bad);
		select_drive(0, 0);
	}

	// ------------------------------------------------------------------
	section("record not found: sector 12 on a 9 sector track");
	{
		dma_read_setup(0x70000, 1);
		fdc_wr(2, 12);
		uint64_t t0 = cycle;
		fdc_wr(0, 0x80);
		CHECK(wait_irq(1500), "RNF irq");
		double tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		uint8_t st = fdc_status();
		CHECKEQ(st & 0x19, 0x10, "RNF");
		CHECK(tms > 800 && tms < 1010, "RNF after 5 index pulses (%.1f ms)", tms);
		// seek beyond the image (track 85 of 80) with verify -> RNF
		fdc_wr(3, 85);
		fdc_wr(0, 0x17);
		CHECK(wait_irq(1500), "seek 85 irq");
		CHECKEQ(fdc_status() & 0x10, 0x10, "verify on a track outside the image: RNF");
		fdc_wr(3, 10);
		fdc_wr(0, 0x13);
		wait_irq(500); fdc_status();
	}

	// ------------------------------------------------------------------
	section("read track (FDC_ReadTrack_ST layout and CRCs)");
	{
		uint32_t a = 0x80000;
		memset(&ram[a], 0, 8192);
		dma_read_setup(a, 13);
		uint64_t t0 = cycle;
		fdc_wr(0, 0xE0);
		CHECK(wait_irq(600), "read track irq");
		double tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		ticks(ms2clk(0.05));
		fdc_status();
		std::vector<uint8_t> g = golden_track(0, gA, 10, 0, 6250);
		int bad = 0, n = (6250 / 16) * 16;
		for (int i = 0; i < n; i++) if (ram[a + i] != g[i]) { if (bad < 5) printf("    track byte %d: %02x exp %02x\n", i, ram[a + i], g[i]); bad++; }
		CHECK(bad == 0, "track bytes (%d bad of %d)", bad, n);
		CHECKEQ(get_dma_addr(), a + n, "DMA address after read track");
		printf("    read track: %.1f ms (index wait + 6250 x 32 us = 200 ms)\n", tms);
		CHECK(tms > 200 && tms < 410, "read track duration");
	}

	// ------------------------------------------------------------------
	section("write track (format track 20 side 0, interleave 2) and read it back");
	{
		fdc_wr(3, 20);
		fdc_wr(0, 0x13);
		wait_irq(200); fdc_status();
		uint32_t a = 0x90000;
		std::vector<uint8_t> t;
		const int order[9] = { 1, 6, 2, 7, 3, 8, 4, 9, 5 };
		for (int i = 0; i < 60; i++) t.push_back(0x4e);
		for (int k = 0; k < 9; k++) {
			for (int i = 0; i < 12; i++) t.push_back(0x00);
			for (int i = 0; i < 3; i++) t.push_back(0xf5);
			t.push_back(0xfe); t.push_back(20); t.push_back(0); t.push_back(order[k]); t.push_back(2); t.push_back(0xf7);
			for (int i = 0; i < 22; i++) t.push_back(0x4e);
			for (int i = 0; i < 12; i++) t.push_back(0x00);
			for (int i = 0; i < 3; i++) t.push_back(0xf5);
			t.push_back(0xfb);
			for (int i = 0; i < 512; i++) t.push_back((uint8_t)(order[k] * 16 + (i & 15)));
			t.push_back(0xf7);
			for (int i = 0; i < 40; i++) t.push_back(0x4e);
		}
		while (t.size() < 6250) t.push_back(0x4e);
		for (size_t i = 0; i < t.size(); i++) ram[a + i] = t[i];
		dma_write_setup(a, 13);
		fdc_wr(0, 0xF0);
		CHECK(wait_irq(700), "write track irq");
		uint8_t st = fdc_status();
		CHECKEQ(st & 0x5D, 0, "write track status (no LOST_DATA / WPRT)");
		int bad = 0;
		for (int s = 1; s <= 9; s++) for (int i = 0; i < 512; i++)
			if (hps->img[0].get(st_lba(gA, 20, 0, s), i) != (uint8_t)(s * 16 + (i & 15))) bad++;
		CHECK(bad == 0, "formatted sectors in the image (%d bad)", bad);
		CHECK(hps->img[0].get(st_lba(gA, 20, 1, 1), 0) == HpsImage::pat(0xA0A0, st_lba(gA, 20, 1, 1), 0), "side 1 untouched");
		// read sectors back through the FDC
		dma_read_setup(0xA0000, 9);
		fdc_wr(2, 1);
		fdc_wr(0, 0x90);
		uint64_t t0 = cycle;
		while (get_dma_addr() < 0xA0000 + 9 * 512 && cycle - t0 < ms2clk(1500)) ticks(ms2clk(1));
		fdc_wr(0, 0xD0); ticks(ms2clk(0.2)); fdc_status();
		bad = 0;
		for (int s = 1; s <= 9; s++) for (int i = 0; i < 512; i++)
			if (ram[0xA0000 + (s - 1) * 512 + i] != (uint8_t)(s * 16 + (i & 15))) bad++;
		CHECK(bad == 0, "formatted sectors read back (%d bad)", bad);
		// read track shows the formatted data
		dma_read_setup(0xB0000, 13);
		fdc_wr(0, 0xE0);
		wait_irq(600); fdc_status();
		std::vector<uint8_t> g = golden_track(0, gA, 20, 0, 6250);
		bad = 0;
		for (int i = 0; i < 6240; i++) if (ram[0xB0000 + i] != g[i]) bad++;
		CHECK(bad == 0, "read track after format (%d bad)", bad);
		// a bad format (256 byte sectors) gives LOST_DATA
		std::vector<uint8_t> t2;
		for (int i = 0; i < 60; i++) t2.push_back(0x4e);
		for (int i = 0; i < 3; i++) t2.push_back(0xf5);
		t2.push_back(0xfe); t2.push_back(20); t2.push_back(0); t2.push_back(1); t2.push_back(1); t2.push_back(0xf7);
		for (int i = 0; i < 22; i++) t2.push_back(0x4e);
		for (int i = 0; i < 3; i++) t2.push_back(0xf5);
		t2.push_back(0xfb);
		for (int i = 0; i < 256; i++) t2.push_back(0x99);
		t2.push_back(0xf7);
		while (t2.size() < 6250) t2.push_back(0x4e);
		for (size_t i = 0; i < t2.size(); i++) ram[0xC0000 + i] = t2[i];
		dma_write_setup(0xC0000, 13);
		fdc_wr(0, 0xF0);
		wait_irq(700);
		CHECKEQ(fdc_status() & 0x04, 0x04, "256 byte sector in a .ST image: LOST_DATA");
		CHECK(hps->img[0].get(st_lba(gA, 20, 0, 1), 0) == 16, "sector 1 not overwritten");
	}

	// ------------------------------------------------------------------
	section("force interrupt: immediate (D8), index (D4), during a seek (D0)");
	{
		fdc_wr(0, 0xD8);
		ticks(100);
		CHECKEQ(dut->irq, 1, "D8: immediate interrupt");
		fdc_status();
		CHECKEQ(dut->irq, 1, "status read does not clear the forced interrupt while D8 is active");
		fdc_wr(0, 0xD0);
		ticks(100);
		CHECKEQ(dut->irq, 1, "D0 after D8: interrupt still set (Hatari)");
		fdc_status();
		CHECKEQ(dut->irq, 0, "status read after D0 clears it");
		fdc_wr(0, 0xD4);
		uint64_t t0 = cycle;
		CHECK(wait_irq(300), "D4: interrupt at the index pulse");
		uint64_t t1 = cycle;
		fdc_status();
		CHECK(wait_irq(300), "D4: interrupt at the next index pulse");
		double per = (cycle - t1) * 1000.0 / TB_CLK_HZ;
		printf("    index interrupt period %.2f ms (300 rpm = 200 ms), first after %.1f ms\n", per, (t1 - t0) * 1000.0 / TB_CLK_HZ);
		CHECK(per > 199.5 && per < 200.5, "index period");
		fdc_wr(0, 0xD0);
		fdc_status();
		ticks(100);
		CHECKEQ(dut->irq, 0, "D0 stops the index interrupt");
		ticks(ms2clk(250));
		CHECKEQ(dut->irq, 0, "no index interrupt after D0");
		// D0 during a seek
		fdc_wr(3, 70);
		fdc_wr(0, 0x10);                     // 6 ms steps
		ticks(ms2clk(20));
		CHECK(fdc_status() & 1, "seek busy");
		fdc_wr(0, 0xD0);
		ticks(ms2clk(0.2));
		uint8_t st = fdc_status();
		CHECKEQ(st & 1, 0, "D0 stops the seek");
		uint8_t tr = fdc_rd(1);
		CHECK(tr > 20 && tr < 70, "track register mid way (%d)", tr);
		CHECKEQ(dut->irq, 0, "no interrupt");
		// restore to get to a known place again
		fdc_wr(0, 0x03); wait_irq(800); fdc_status();
		fdc_wr(3, 10); fdc_wr(0, 0x13); wait_irq(200); fdc_status();
	}

	// ------------------------------------------------------------------
	section("write protect: drive A read only");
	{
		mount(0, 80 * 9 * 2 * 512, true);
		uint8_t st = fdc_status();     // type I status after the last seek
		CHECKEQ(st & 0x40, 0x40, "WPRT (forced during the eject/insert transition or read only)");
		ticks(ms2clk(400));
		CHECKEQ(fdc_status() & 0x40, 0x40, "WPRT read only");
		uint32_t a = 0x60000;
		dma_write_setup(a, 1);
		fdc_wr(2, 6);
		uint64_t before = hps->ops_wr;
		fdc_wr(0, 0xA0);
		CHECK(wait_irq(50), "write protected: immediate irq");
		st = fdc_status();
		CHECKEQ(st & 0x41, 0x40, "WPRT set, not busy");
		CHECKEQ(hps->ops_wr, before, "nothing written");
		fdc_wr(0, 0xF0);
		CHECK(wait_irq(500), "write track on a protected disk irq");
		CHECKEQ(fdc_status() & 0x40, 0x40, "write track: WPRT");
		CHECKEQ(hps->errors, 0, "no hps write to the read only image");
		mount(0, 80 * 9 * 2 * 512, false);
		fdc_wr(0, 0xD0);                     // idle force interrupt: type I status again
		ticks(ms2clk(0.2));
		// disk change transition: WPRT forced for 18 VBL after the eject
		CHECKEQ(fdc_status() & 0x40, 0x40, "WPRT forced right after a disk change");
		ticks(ms2clk(380));
		CHECKEQ(fdc_status() & 0x40, 0x00, "WPRT follows the disk again after 360 ms");
	}

	// ------------------------------------------------------------------
	section("drive B: HD disk needs $FF860E = 3 (FDC_CanMachineHandleDensity)");
	{
		select_drive(1, 0);
		fdc_wr(0, 0x07);                     // restore with verify: DD mode, HD disk
		CHECK(wait_irq(2500), "restore irq on B");
		uint8_t st = fdc_status();
		CHECKEQ(st & 0x10, 0x10, "verify fails (RNF) with the DD density setting");
		ww(DENS, 3);
		fdc_wr(0, 0x07);
		CHECK(wait_irq(2500), "restore irq on B, HD mode");
		CHECKEQ(fdc_status() & 0x14, 0x04, "HD mode: verify ok, TR00");
		fdc_wr(3, 5);
		fdc_wr(0, 0x13);
		wait_irq(200); fdc_status();
		uint32_t a = 0xD0000;
		dma_read_setup(a, 1);
		fdc_wr(2, 17);
		uint64_t t0 = cycle;
		fdc_wr(0, 0x80);
		CHECK(wait_irq(400), "HD read irq");
		double tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		CHECKEQ(fdc_status() & 0x1D, 0, "HD read status");
		int bad;
		CHECK(ram_matches_image(a, 1, st_lba(gB, 5, 0, 17), 1, &bad), "HD sector data (%d bad)", bad);
		CHECKEQ(hps->last_lba[1], st_lba(gB, 5, 0, 17), "drive B lba");
		printf("    HD read sector %.2f ms (512 x 16 us = 8.2 ms + rotation)\n", tms);
		// read address on HD: 18 sectors per track
		dma_read_setup(0xD1000, 1);
		for (int k = 0; k < 8; k++) { fdc_wr(0, 0xC0); wait_irq(300); fdc_status(); }
		ticks(ms2clk(0.05));
		bool ok = true;
		for (int k = 0; k < 8; k++) {
			uint8_t exp[6]; uint8_t *g = &ram[0xD1000 + 6 * k];
			golden_id(exp, 5, 0, g[2]);
			if (memcmp(exp, g, 6) || g[2] < 1 || g[2] > 18) ok = false;
		}
		CHECK(ok, "HD ID fields");
		ww(DENS, 0);
		select_drive(0, 0);
	}

	// ------------------------------------------------------------------
	section("peripheral reset (68030 RESET) keeps the inserted disks (Hatari FDC_Reset)");
	{
		select_drive(0, 0);
		fdc_wr(0, 0x07);                     // TR is shared by the drives: restore A first
		CHECK(wait_irq(2500), "restore A");
		fdc_status();
		fdc_wr(3, 12);
		fdc_wr(0, 0x17);
		CHECK(wait_irq(2500), "seek to 12 with verify");
		CHECKEQ(fdc_status() & 0x10, 0, "verify ok");
		dut->reset = 1; ticks(5); dut->reset = 0; ticks(5);
		CHECKEQ(dut->irq, 0, "reset clears the interrupt");
		CHECKEQ(fdc_rd(1), 12, "track register kept over a warm reset (Hatari)");
		CHECKEQ(fdc_rd(2), 1, "sector register 1 after reset");
		CHECKEQ(fdc_status(), 0x00, "status register 0 after reset");
		fdc_wr(0, 0xD0);
		ticks(ms2clk(0.2));
		uint8_t st = fdc_status();
		CHECKEQ(st & 0x44, 0x00, "type I status: head still at 12 (no TR00), disk inserted and writable");
		CHECKEQ(rw(DMA_MODE) & 3, 1, "DMA status after reset");
		uint32_t a = 0xF0000;
		dma_read_setup(a, 1);
		fdc_wr(2, 2);
		fdc_wr(0, 0x80);
		CHECK(wait_irq(2500), "read sector after reset, no new mount pulse");
		CHECKEQ(fdc_status() & 0x1D, 0, "read status");
		ticks(ms2clk(0.05));
		int bad;
		CHECK(ram_matches_image(a, 0, st_lba(gA, 12, 0, 2), 1, &bad), "data after reset (%d bad)", bad);

		// reset while hps_io is transferring a sector to the core
		dma_read_setup(a, 1);
		fdc_wr(2, 5);
		fdc_wr(0, 0x80);
		uint64_t n = 0;
		while (!(dut->sd_ack & 1) && n++ < ms2clk(400)) tick();
		ticks(500);
		CHECK(dut->sd_ack & 1, "a block transfer is running");
		dut->reset = 1; ticks(3); dut->reset = 0;
		n = 0;
		while ((dut->sd_ack || hps->st != HpsModel::IDLE) && n++ < 1000000) tick();
		ticks(20);
		CHECKEQ(dut->sd_rd | dut->sd_wr, 0, "no request left after the absorbed transfer");
		CHECKEQ(dut->irq, 0, "no interrupt from the aborted command");
		dma_read_setup(a + 0x1000, 1);
		fdc_wr(2, 6);
		fdc_wr(0, 0x80);
		CHECK(wait_irq(2500), "read sector after the interrupted transfer");
		CHECKEQ(fdc_status() & 0x1D, 0, "read status");
		ticks(ms2clk(0.05));
		CHECK(ram_matches_image(a + 0x1000, 0, st_lba(gA, 12, 0, 6), 1, &bad), "data (%d bad)", bad);

		// reset during a sector write to the image
		for (int i = 0; i < 512; i++) ram[a + 0x2000 + i] = (uint8_t)(i + 0x40);
		dma_write_setup(a + 0x2000, 1);
		fdc_wr(2, 7);
		fdc_wr(0, 0xA0);
		n = 0;
		while (!(dut->sd_ack & 1) && n++ < ms2clk(400)) tick();
		ticks(500);
		CHECK(dut->sd_ack & 1, "write block transfer running");
		dut->reset = 1; ticks(3); dut->reset = 0;
		n = 0;
		while ((dut->sd_ack || hps->st != HpsModel::IDLE) && n++ < 1000000) tick();
		CHECKEQ(hps->img[0].get(st_lba(gA, 12, 0, 7), 511), (uint8_t)(511 + 0x40), "the block hps_io was serving is complete");

		// a mount pulse while reset is held is latched: drive B gets a 720 KB disk
		make_st(1, 0xC0C0, 80, 9, 2);
		hps->img[1].wr.clear();
		dut->reset = 1;
		ticks(3);
		hps->img[1].mounted = true; hps->img[1].size = 80 * 9 * 2 * 512; hps->img[1].readonly = false;
		dut->img_size = 80 * 9 * 2 * 512;
		dut->img_readonly = 0;
		mnt_pulse = 2;
		ticks(10);
		dut->reset = 0;
		ticks(20000);
		select_drive(1, 0);
		fdc_wr(0, 0x07);
		CHECK(wait_irq(2500), "restore on B after the mount during reset");
		CHECKEQ(fdc_status() & 0x14, 0x04, "B: DD disk now, verify ok in DD mode");
		dma_read_setup(a + 0x3000, 1);
		fdc_wr(2, 9);
		fdc_wr(0, 0x80);
		CHECK(wait_irq(2500), "read sector 9 on B");
		CHECKEQ(fdc_status() & 0x1D, 0, "read status");
		ticks(ms2clk(0.05));
		CHECK(ram_matches_image(a + 0x3000, 1, st_lba(gA, 0, 0, 9), 1, &bad), "new disk data (%d bad)", bad);
		select_drive(0, 0);
		CHECKEQ(hps->errors, 0, "hps protocol errors after the resets");
	}

	// ------------------------------------------------------------------
	section("no drive selected / drive not present");
	{
		select_drive(-1, 0);
		uint8_t st = fdc_status();
		CHECKEQ(st & 0x46, 0, "no drive: TR00, INDEX, WPRT off in the type I status");
		uint64_t t0 = cycle;
		fdc_wr(0, 0x0B);                     // restore without a drive: 255 steps then RNF
		CHECK(wait_irq(900), "restore without drive irq");
		double tms = (cycle - t0) * 1000.0 / TB_CLK_HZ;
		CHECKEQ(fdc_status() & 0x10, 0x10, "restore without a drive: RNF");
		CHECKEQ(fdc_rd(1), 0, "TR 0 after 255 step attempts");
		CHECK(tms > 765 && tms < 766, "255 steps of 3 ms (%.2f ms)", tms);
		select_drive(0, 0);
	}

	// ------------------------------------------------------------------
	section("HDC / SCSI select (mode bit 3): device-less NCR 5380, selection times out");
	{
		ww(DMA_MODE, 0x88);                  // register 0 (output data)
		ww(DMA_DATA, 0x81);                  // initiator id 7 + target 0
		ww(DMA_MODE, 0x8A);                  // register 1 (ICR)
		ww(DMA_DATA, 0x05);                  // assert data bus + SEL
		ww(DMA_MODE, 0x88 | 0x08);           // register 4 (bus status) = mode 0x8C
		bool bsy = false;
		for (int i = 0; i < 50; i++) { if (rw(DMA_DATA) & 0x40) bsy = true; ticks(100); }
		CHECK(!bsy, "BSY never asserted by a target: selection times out");
		ww(DMA_MODE, 0x8A);
		CHECKEQ(rw(DMA_DATA), 0x0005, "ICR reads back");
		ww(DMA_DATA, 0x00);
		ww(DMA_MODE, 0x80);
		CHECKEQ(dut->irq, 0, "no interrupt from the 5380");
	}

	CHECKEQ(hps->errors, 0, "hps model protocol errors");
	printf("\nFDC: %d checks, %d failures, %llu cycles (%.2f s simulated)\n", checks, fails,
	       (unsigned long long)cycle, cycle / (double)TB_CLK_HZ);
	printf(fails ? "FDC TEST FAIL\n" : "FDC TEST PASS\n");
	delete dut;
	return fails ? 1 : 0;
}
