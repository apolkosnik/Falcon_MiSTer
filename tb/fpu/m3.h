// m3.h - milestone 3 arithmetic sweeps: every documented opmode over operand classes, formats and
// rounding/precision modes, the undocumented opmodes, FSINCOS.  Executed by the real CPU + RTL + ARM
// service and by the golden (Hatari fpp.c with real EAs); the result stream is compared item by item.
#ifndef TB_M3_H
#define TB_M3_H
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

#include "m2.h"

struct M3Group {
	std::string name;
	unsigned items = 0, bad = 0;
	std::string detail;        // first mismatches (expected / got)
};

void m3_build();
int m3_nchunks();
void m3_emit_asm(FILE *f, int chunk);
void m3_load_ram(uint8_t *ram, int chunk);
void m3_run_golden(const std::map<std::string, uint32_t> &syms, int chunk);
// compare the result stream of a chunk; exceptions = every exception the program logged
std::vector<M3Group> m3_compare(const uint8_t *ram, int chunk, const std::vector<M2Exc> &exc, std::string &exc_detail, bool &exc_ok, unsigned &n_soft);
bool m3_done(const uint8_t *ram, int chunk);
uint32_t m3_done_addr();

#endif
