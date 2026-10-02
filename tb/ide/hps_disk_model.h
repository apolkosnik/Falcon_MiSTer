// hps_disk_model.h - C++ model of the MiSTer hps_io SD block interface as
// seen by the core (sys/hps_io.sv), used by the Falcon disk controller
// testbenches.
//
// Timing follows hps_io.sv:
//  - the HPS polls command 0x16 and sees sd_rd/sd_wr some time after they
//    rise; sd_lba is sampled during that poll;
//  - transfer commands 0x17 (HPS -> FPGA) / 0x18 (FPGA -> HPS): at the first
//    strobe (byte_cnt == 0) sd_ack goes high and sd_buff_addr is cleared;
//  - 0x17: each further strobe loads sd_buff_dout, sd_buff_wr is high for one
//    clock one clock later (b_wr[0]) and sd_buff_addr increments two clocks
//    after that (b_wr[2]);
//  - 0x18: each further strobe samples sd_buff_din and increments
//    sd_buff_addr (saturating at 511);
//  - sd_ack drops when the command ends (io_enable low).
// Strobes are spaced by a random number of clocks (min 3).
//
// The images are sparse: unwritten sectors return a deterministic pattern.
#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <vector>
#include <array>

struct HpsImage {
	bool     mounted = false;
	bool     readonly = false;
	uint64_t size = 0;
	uint32_t seed = 0;
	std::map<uint64_t, std::array<uint8_t, 512>> wr;
	std::map<uint64_t, std::array<uint8_t, 512>> fixed;   // preset sectors

	static uint8_t pat(uint32_t seed, uint64_t lba, unsigned i) {
		uint64_t x = (uint64_t)seed * 0x9E3779B97F4A7C15ull + lba * 0x100000001B3ull + i * 0x2545F4914F6CDD1Dull;
		x ^= x >> 29; x *= 0xBF58476D1CE4E5B9ull; x ^= x >> 32;
		return (uint8_t)x;
	}
	uint8_t get(uint64_t lba, unsigned i) const {
		auto it = wr.find(lba);
		if (it != wr.end()) return it->second[i];
		auto f = fixed.find(lba);
		if (f != fixed.end()) return f->second[i];
		return pat(seed, lba, i);
	}
	void put(uint64_t lba, const uint8_t *d) {
		std::array<uint8_t, 512> a;
		for (int i = 0; i < 512; i++) a[i] = d[i];
		wr[lba] = a;
	}
	void preset(uint64_t lba, const uint8_t *d) {
		std::array<uint8_t, 512> a;
		for (int i = 0; i < 512; i++) a[i] = d[i];
		fixed[lba] = a;
	}
};

struct HpsModel {
	// outputs to the core (registered, applied after the clock edge)
	uint32_t sd_ack = 0;
	uint32_t sd_buff_addr = 0;
	uint32_t sd_buff_dout = 0;
	uint32_t sd_buff_wr = 0;

	int nunits;
	std::vector<HpsImage> img;
	uint32_t rng = 12345;

	// state
	enum { IDLE, POLL_WAIT, XFER_START, XFER, XFER_END } st = IDLE;
	int      unit = -1;
	bool     is_wr = false;
	uint64_t lba = 0;
	int      wait = 0;
	int      cnt = 0;
	unsigned b_wr = 0;
	uint8_t  buf[512];
	uint64_t ops_rd = 0, ops_wr = 0;
	uint32_t last_lba[8];
	int      min_lat = 20, max_lat = 400;
	int      errors = 0;

	HpsModel(int n) : nunits(n), img(n) { for (auto &l : last_lba) l = 0; }

	int rnd(int lo, int hi) {
		rng = rng * 1103515245u + 12345u;
		return lo + (int)((rng >> 8) % (uint32_t)(hi - lo + 1));
	}

	// Called once per clock before the rising edge with the core's current
	// outputs.  Updates the model's registered outputs.
	void step(uint32_t sd_rd, uint32_t sd_wr, uint32_t sd_lba, uint32_t sd_buff_din) {
		// b_wr pipeline (hps_io: sd_buff_wr <= b_wr[0]; addr++ on b_wr[2])
		uint32_t n_wr = (b_wr & 1) ? 1 : 0;
		if ((b_wr & 4) && sd_buff_addr != 511) sd_buff_addr++;
		b_wr = (b_wr << 1) & 7;
		sd_buff_wr = n_wr;

		switch (st) {
		case IDLE:
			for (int u = 0; u < nunits; u++) {
				if (((sd_rd >> u) & 1) || ((sd_wr >> u) & 1)) {
					unit = u;
					is_wr = (sd_wr >> u) & 1;
					wait = rnd(min_lat, max_lat);
					st = POLL_WAIT;
					break;
				}
			}
			break;
		case POLL_WAIT:
			if (--wait <= 0) {
				// command 0x16 poll: sample request and lba
				bool r = (sd_rd >> unit) & 1, w = (sd_wr >> unit) & 1;
				if (!r && !w) { fprintf(stderr, "HPS: request dropped before ack (unit %d)\n", unit); errors++; st = IDLE; break; }
				is_wr = w;
				lba = sd_lba;
				last_lba[unit] = sd_lba;
				if (!is_wr) {
					for (int i = 0; i < 512; i++) buf[i] = img[unit].get(lba, i);
				}
				wait = rnd(10, 100);
				st = XFER_START;
			}
			break;
		case XFER_START:
			if (--wait <= 0) {
				// first strobe of 0x17/0x18
				sd_ack = 1u << unit;
				sd_buff_addr = 0;
				cnt = 0;
				wait = rnd(3, 8);
				st = XFER;
			}
			break;
		case XFER:
			if (--wait <= 0) {
				if (!is_wr) {
					sd_buff_dout = buf[cnt];
					b_wr = 1;
				} else {
					buf[cnt] = (uint8_t)sd_buff_din;
					if (sd_buff_addr != 511) sd_buff_addr++;
				}
				cnt++;
				wait = rnd(3, 8);
				if (cnt == 512) { wait = rnd(4, 12); st = XFER_END; }
			}
			break;
		case XFER_END:
			if (--wait <= 0) {
				sd_ack = 0;
				if (is_wr) {
					if (img[unit].readonly) { fprintf(stderr, "HPS: write to read only image unit %d\n", unit); errors++; }
					img[unit].put(lba, buf);
					ops_wr++;
				} else ops_rd++;
				st = IDLE;
			}
			break;
		}
	}
};
