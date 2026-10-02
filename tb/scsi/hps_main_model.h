// hps_main_model.h - the HPS side of the hps_io block interface for the SCSI
// bench.  Protocol timing as sys/hps_io.sv (same model as ../ide/
// hps_disk_model.h): the request is seen some time after sd_rd/sd_wr rise,
// sd_lba is sampled at that poll, the first strobe raises sd_ack and clears
// sd_buff_addr, reads deliver sd_buff_dout with sd_buff_wr one clock later and
// the address increment two clocks after that, writes sample sd_buff_din and
// increment the address, sd_ack drops at the end.  The block content comes
// from / goes to MainSim, i.e. the real Main_MiSTer support/falcon code and
// the user_io generic path.
#pragma once
#include <cstdint>
#include <cstdio>
#include <vector>
#include "main_shim.h"

struct HpsMain {
	uint32_t sd_ack = 0, sd_buff_addr = 0, sd_buff_dout = 0, sd_buff_wr = 0;

	MainSim *main;
	int      slot0;               // hps slot of unit 0
	int      nunits;
	uint32_t rng = 4242;
	enum { IDLE, POLL_WAIT, XFER_START, XFER, XFER_END } st = IDLE;
	int      unit = -1;
	bool     is_wr = false;
	uint32_t lba = 0;
	int      wait = 0, cnt = 0;
	unsigned b_wr = 0;
	uint8_t  buf[512];
	int      errors = 0;
	uint64_t ops_rd = 0, ops_wr = 0;
	std::vector<uint32_t> lba_log[4];
	std::vector<bool>     wr_log[4];

	HpsMain(MainSim *m, int s0, int n) : main(m), slot0(s0), nunits(n) {}

	int rnd(int lo, int hi) { rng = rng * 1103515245u + 12345u; return lo + (int)((rng >> 8) % (uint32_t)(hi - lo + 1)); }
	bool busy() const { return st != IDLE; }
	void clear_log() { for (int u = 0; u < 4; u++) { lba_log[u].clear(); wr_log[u].clear(); } }

	void step(uint32_t sd_rd, uint32_t sd_wr, uint32_t sd_lba, uint32_t sd_buff_din) {
		uint32_t n_wr = (b_wr & 1) ? 1 : 0;
		if ((b_wr & 4) && sd_buff_addr != 511) sd_buff_addr++;
		b_wr = (b_wr << 1) & 7;
		sd_buff_wr = n_wr;

		switch (st) {
		case IDLE:
			for (int u = 0; u < nunits; u++) {
				if (((sd_rd >> u) & 1) || ((sd_wr >> u) & 1)) {
					unit = u;
					wait = rnd(20, 300);
					st = POLL_WAIT;
					break;
				}
			}
			break;
		case POLL_WAIT:
			if (--wait <= 0) {
				bool r = (sd_rd >> unit) & 1, w = (sd_wr >> unit) & 1;
				if (!r && !w) { printf("HPS: request dropped before ack (unit %d)\n", unit); errors++; st = IDLE; break; }
				if (r && w) { printf("HPS: read and write requested together (unit %d)\n", unit); errors++; }
				if ((sd_rd | sd_wr) & ~(1u << unit) & 7) { printf("HPS: more than one slot requested\n"); errors++; }
				is_wr = w;
				lba = sd_lba;
				lba_log[unit].push_back(lba);
				wr_log[unit].push_back(is_wr);
				if (!is_wr) main->read_block(slot0 + unit, lba, buf);
				wait = rnd(10, 100);
				st = XFER_START;
			}
			break;
		case XFER_START:
			if (--wait <= 0) {
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
				if (is_wr) { main->write_block(slot0 + unit, lba, buf); ops_wr++; }
				else ops_rd++;
				st = IDLE;
			}
			break;
		}
	}
};
