// m2.h - milestone 2 generated test cases: a table of FPU instruction
// sequences that is (a) emitted as 68k assembly for the real CPU+RTL+ARM
// service, and (b) executed by the golden (Hatari fpp.c with real EAs over a
// fake RAM, golden.c).  The bench then compares every FP register, FPCR, FPSR,
// the integer registers and the memory images.
#ifndef TB_M2_H
#define TB_M2_H
#include <cstdint>
#include <cstdio>
#include <map>
#include <string>
#include <vector>

struct M2Exc {
	uint32_t vec, fmt, pc;
};
struct M2Mismatch {
	std::string what;
	std::string exp, got;
};
struct M2Result {
	bool done = false;
	bool ok = false;
	std::string summary;                 // golden summary (expected)
	std::vector<M2Mismatch> diffs;       // first few mismatches
	uint32_t fpiar_exp = 0, fpiar_got = 0;
	bool fpiar_diff = false;
	int nexc = 0;
};

void m2_build();                                             // fill the case table
int m2_ncases();
const char *m2_group(int i);
const char *m2_name(int i);
int m2_nchunks();
int m2_chunk_first(int c);
int m2_chunk_end(int c);
void m2_emit_asm(FILE *f, int chunk);                        // body of obj/gen_cases_<chunk>.s
void m2_load_ram(uint8_t *ram, int chunk);                   // initial windows of one chunk
void m2_run_golden(const std::map<std::string, uint32_t> &syms, int chunk);   // needs the assembler's labels
unsigned m2_golden_cond_requests();                          // nonaware predicates evaluated with NAN set, last golden run
M2Result m2_compare(const uint8_t *ram, int i, const std::vector<M2Exc> &got);
uint32_t m2_dump_addr(int i);
uint32_t m2_marker(int i);                                   // value stored at dump+0xFC when the case ran to its end
bool m2_expects_imm(int i);                                  // the case uses #imm for D/X/P/multi-register (cp_ea_ok issue)

#endif
