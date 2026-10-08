// m2.cpp - milestone 2 test case table, assembly emitter and golden runner.
//
// Every case is a self-contained sequence executed by the real 68030 RTL:
//   FRESTORE null (reset), FMOVEM.X init image -> FP0-FP7, FMOVE.L #fpcr,FPCR,
//   integer registers loaded, the instruction(s) under test,
//   [FScc (a5)+ for predicates, FBcc/FDBcc/FTRAPcc flags],
//   dump: FMOVEM.X FP0-FP7, FMOVEM.L FPCR/FPSR/FPIAR, MOVEM.L d0-d7/a0-a6.
// The identical sequence (same opcode words, same register values, same
// initial RAM) is executed by Hatari's fpp.c through golden.c; the dump area
// and the data window must match byte for byte.  The expected values are the
// golden's, never computed from the RTL or the service.
#include "m2.h"

#include <algorithm>
#include <cstdlib>
#include <sys/wait.h>
#include <unistd.h>
#include <cstring>
#include <random>

#include "golden.h"

namespace {

enum { M_DN, M_AN, M_IND, M_POST, M_PRE, M_D16, M_IMM };
enum { F_L, F_S, F_X, F_P, F_W, F_D, F_B };               // cmd size field

const uint32_t WBASE = 0x100000, DBASE = 0x300000;
const uint32_t SRC = 0x10, DST = 0x80, CND = 0x800, BPA = 0x840, FPI = 0xF80, NULLF = 0xFE0;
const uint32_t CASE_STRIDE = 0x1000, DUMP_STRIDE = 0x100;
const int CHUNK = 400;                                     // cases per program image (ROM is 512 KB)

struct Insn {
	uint16_t op = 0, cmd = 0;
	std::vector<uint16_t> ext;
	bool reset = false;                                    // FRESTORE (a6) of the null frame
	bool scc = false;                                      // FScc <ea>: cmd is the predicate
	bool soft = false;                                     // an exception is expected: the handler steps over it
	int ckind = 0;                                         // conditional helpers: 1 FScc (a5)+ style, 2 FBcc, 3 FDBcc, 4 FTRAPcc
	int pred = 0;
	uint32_t slot = 0;                                     // window offset of the result byte(s)
	std::vector<std::pair<int, uint32_t> > pre;            // registers loaded just before the instruction (0-7 D, 8-14 A)
	int frame = 0;                                         // 1 FSAVE <ea>, 2 FRESTORE <ea>: op + extension words only (no command word)
	int frame_reg = 0, frame_mode = 0;                     // for the golden's image position
	uint32_t frame_disp = 0;
	bool bset27 = false;                                   // 68k BSET #3,<abs>: slot is the absolute address (BIU flags bit 27 of a frame)
	uint32_t bset_addr = 0;                                // window offset of the byte
	std::vector<uint8_t> pcdata;                           // data pool for (d16,PC): the extension word is generated
	bool pcrel = false;
};

struct Case {
	std::string group, name;
	uint32_t d[8], a[6];
	std::vector<uint8_t> mem;                              // CASE_STRIDE bytes
	bool soft_cond = false;                                // conditionals may raise BSUN: step over them
	bool skip = false;                                     // debugging (M2_ONLY): not emitted, not run
	bool softall = false;                                  // every instruction may be reported an exception: step over each
	int hmode = 0;                                         // exception handler: 0 absorbs the exception (FSAVE/FRESTORE epilogue), 1 only returns
	uint32_t fpcr = 0;
	std::vector<Insn> body;
	std::vector<int> spreds;                               // FScc (a5)+ predicates
	std::vector<int> bpreds;                               // FBcc / FDBcc / FTRAPcc predicates
	bool imm_issue = false;
};

std::vector<Case> cases;
std::vector<uint8_t> golden_img;                           // golden RAM after the run

std::vector<int> chunk_start;                              // first case of each program image
std::vector<int> li_of;                                    // index of a case within its image
bool g_isolate = false, g_force_new = false;               // cases that may hang the CPU get an image of their own
uint32_t W(int i) { return WBASE + (uint32_t)li_of[(size_t)i] * CASE_STRIDE; }
uint32_t DU(int i) { return DBASE + (uint32_t)li_of[(size_t)i] * DUMP_STRIDE; }
void pop_case()
{
	int i = (int)cases.size() - 1;
	cases.pop_back();
	li_of.pop_back();
	if (!chunk_start.empty() && chunk_start.back() == i) chunk_start.pop_back();
}

// -------------------------------------------------------------- operand data
typedef std::vector<uint8_t> Bytes;
Bytes be(uint64_t v, int n)
{
	Bytes b((size_t)n);
	for (int i = n - 1; i >= 0; i--) { b[(size_t)i] = (uint8_t)v; v >>= 8; }
	return b;
}
Bytes xval(uint16_t se, uint64_t m)
{
	Bytes b = be(se, 2);
	b.push_back(0); b.push_back(0);
	Bytes mm = be(m, 8);
	b.insert(b.end(), mm.begin(), mm.end());
	return b;
}
Bytes pval(uint32_t l0, uint32_t l1, uint32_t l2)
{
	Bytes b = be(l0, 4), c = be(l1, 4), d = be(l2, 4);
	b.insert(b.end(), c.begin(), c.end());
	b.insert(b.end(), d.begin(), d.end());
	return b;
}

const uint64_t VB[] = {0x00, 0x01, 0x7F, 0x80, 0xFF, 0x55};
const uint64_t VW[] = {0x0000, 0x0001, 0x7FFF, 0x8000, 0xFFFF, 0x1234};
const uint64_t VL[] = {0, 1, 0x7FFFFFFFull, 0x80000000ull, 0xFFFFFFFFull, 0x12345678ull, 0x00FFFFFFull};
const uint64_t VS[] = {0, 0x80000000ull, 0x3F800000ull, 0xC0200000ull, 0x7F800000ull, 0xFF800000ull, 0x7FC00000ull,
                       0x7FA00000ull, 0x00000001ull, 0x00800000ull, 0x7F7FFFFFull, 0x3DCCCCCDull, 0x4B000001ull};
const uint64_t VD[] = {0, 0x8000000000000000ull, 0x3FF0000000000000ull, 0xC004000000000000ull, 0x7FF0000000000000ull,
                       0x7FF8000000000000ull, 0x7FF4000000000000ull, 0x0000000000000001ull, 0x7E37E43C8800759Cull,
                       0x01A56E1FC2F8F359ull, 0x3FB999999999999Aull, 0xFFF0000000000000ull};
struct XV { uint16_t se; uint64_t m; };
const XV VX[] = {{0x0000, 0}, {0x8000, 0}, {0x3FFF, 0x8000000000000000ull}, {0x7FFF, 0}, {0xFFFF, 0},
                 {0x7FFF, 0xC000000000000000ull}, {0x7FFF, 0x8000000000000001ull}, {0x0000, 0x0000000000001000ull},
                 {0x0001, 0x8000000000000000ull}, {0x7FFE, 0xFFFFFFFFFFFFFFFFull}, {0x3FFF, 0x4000000000000000ull},
                 {0x4000, 0xC90FDAA22168C235ull}, {0xC000, 0xA000000000000000ull}, {0x4405, 0x8000000000000000ull},
                 {0x3BE0, 0x8000000000000000ull}};
const uint32_t VP[][3] = {{0, 0, 0}, {1, 0, 0}, {0x80100001u, 0x50000000u, 0}, {0x41000009u, 0x99999999u, 0x99999999u},
                          {0x7FFF0000u, 0, 0}, {0x7FFF0000u, 0, 1}, {0x80000000u, 0, 0},
                          {0x40050001u, 0x23456789u, 0x01234567u}, {0x01000001u, 0, 0}};

Bytes data_of(int fmt, int k)       // k-th edge value of a format
{
	switch (fmt) {
	case F_B: return be(VB[k % 6], 1);
	case F_W: return be(VW[k % 6], 2);
	case F_L: return be(VL[k % 7], 4);
	case F_S: return be(VS[k % 13], 4);
	case F_D: return be(VD[k % 12], 8);
	case F_X: return xval(VX[k % 15].se, VX[k % 15].m);
	default: return pval(VP[k % 9][0], VP[k % 9][1], VP[k % 9][2]);
	}
}
int nvals(int fmt)
{
	static const int n[7] = {7, 13, 15, 9, 6, 12, 6};
	return n[fmt];
}
int fsize(int fmt)
{
	static const int s[7] = {4, 4, 12, 12, 2, 8, 1};
	return s[fmt];
}
const char *fname(int fmt)
{
	static const char *n[7] = {"L", "S", "X", "P", "W", "D", "B"};
	return n[fmt];
}

// FP init images (12 bytes per register: sign/exponent, pad, mantissa)
Bytes image(int id)
{
	static const XV img[4][8] = {
	    {{0x3FFF, 0x8000000000000000ull}, {0xC000, 0xA000000000000000ull}, {0x4000, 0xC90FDAA22168C235ull}, {0x0000, 0},
	     {0x8000, 0}, {0x7FFF, 0}, {0x7FFF, 0xC000000000000000ull}, {0x3FFB, 0xCCCCCCCCCCCCCCCDull}},
	    {{0x4000, 0xA000000000000000ull}, {0xC000, 0xA000000000000000ull}, {0x4000, 0xE000000000000000ull},
	     {0xBFFE, 0x8000000000000000ull}, {0x4020, 0x9502F90000000000ull}, {0x4010, 0x88B8400000000000ull},
	     {0xBFEE, 0xA7C5AC471B478423ull}, {0x4020, 0x8000000080000000ull}},
	    {{0x7FFF, 0x8000000000000001ull}, {0x0000, 0x0000000000001000ull}, {0x0001, 0x8000000000000000ull},
	     {0x7FFE, 0xFFFFFFFFFFFFFFFFull}, {0x3FFF, 0x4000000000000000ull}, {0x4405, 0x8000000000000000ull},
	     {0x3BE0, 0x8000000000000000ull}, {0x3FFF, 0x8000000000000000ull}},
	    // image 3 (exception tests): sNaN, -1, huge, tiny, +0, 1.0, 3.0, 0.1
	    {{0x7FFF, 0x8000000000000001ull}, {0xBFFF, 0x8000000000000000ull}, {0x7FFE, 0xFFFFFFFFFFFFFFFFull},
	     {0x0001, 0x8000000000000000ull}, {0x0000, 0}, {0x3FFF, 0x8000000000000000ull}, {0x4000, 0xC000000000000000ull},
	     {0x3FFB, 0xCCCCCCCCCCCCCCCDull}}};
	Bytes b;
	for (int r = 0; r < 8; r++) {
		Bytes x = xval(img[id][r].se, img[id][r].m);
		b.insert(b.end(), x.begin(), x.end());
	}
	return b;
}

// -------------------------------------------------------------- builders
Case &new_case(const char *group, const std::string &name, int fpimg = 0, uint32_t fpcr = 0)
{
	int ni = (int)cases.size();
	if (chunk_start.empty() || g_force_new || g_isolate || ni - chunk_start.back() >= CHUNK) chunk_start.push_back(ni);
	g_force_new = g_isolate;
	li_of.push_back(ni - chunk_start.back());
	cases.emplace_back();
	Case &c = cases.back();
	int i = (int)cases.size() - 1;
	c.group = group;
	c.name = name;
	for (int k = 0; k < 8; k++) c.d[k] = 0x10203040u + 0x01010101u * (uint32_t)k;
	for (int k = 0; k < 5; k++) c.a[k] = W(i) + 0x1F0;     // harmless defaults inside the window
	c.a[5] = W(i) + CND;
	c.mem.assign(CASE_STRIDE, 0xA5);
	Bytes im = image(fpimg);
	std::copy(im.begin(), im.end(), c.mem.begin() + FPI);
	std::fill(c.mem.begin() + NULLF, c.mem.begin() + NULLF + 4, 0);
	c.fpcr = fpcr;
	return c;
}
int cidx() { return (int)cases.size() - 1; }

uint16_t ea_of(int mode, int reg)
{
	switch (mode) {
	case M_DN: return (uint16_t)reg;
	case M_AN: return (uint16_t)(0x08 | reg);
	case M_IND: return (uint16_t)(0x10 | reg);
	case M_POST: return (uint16_t)(0x18 | reg);
	case M_PRE: return (uint16_t)(0x20 | reg);
	case M_D16: return (uint16_t)(0x28 | reg);
	default: return 0x3C;
	}
}
const char *mname(int mode)
{
	static const char *n[] = {"Dn", "An", "(An)", "(An)+", "-(An)", "d16(An)", "#imm"};
	return n[mode];
}

// build the instruction word(s) for an EA and set up the register/memory so that it
// refers to window offset `off` (size bytes).  Data for a source goes to memory
// (or into Dn / the instruction stream).
Insn make(Case &c, uint16_t cmd, int mode, int reg, uint32_t off, int size, const Bytes *src)
{
	int i = cidx();
	Insn n;
	n.cmd = cmd;
	n.op = 0xF200 | ea_of(mode, reg);
	uint32_t addr = W(i) + off;
	switch (mode) {
	case M_DN:
		if (src) {
			uint32_t v = 0;
			for (uint8_t b : *src) v = (v << 8) | b;
			if (size == 1) v |= 0xDEADBE00u;
			if (size == 2) v |= 0xDEAD0000u;
			c.d[reg] = v;
			n.pre.push_back({reg, v});
		}
		break;
	case M_AN:
		if (src) { uint32_t v = 0; for (uint8_t b : *src) v = (v << 8) | b; c.a[reg] = v; n.pre.push_back({8 + reg, v}); }
		break;
	case M_IND: case M_POST: c.a[reg] = addr; n.pre.push_back({8 + reg, addr}); break;
	case M_PRE: c.a[reg] = addr + (uint32_t)size; n.pre.push_back({8 + reg, addr + (uint32_t)size}); break;
	case M_D16: c.a[reg] = addr - 0x20; n.pre.push_back({8 + reg, addr - 0x20}); n.ext.push_back(0x20); break;
	case M_IMM:
		if (src) {
			if (src->size() == 1) n.ext.push_back((*src)[0]);
			else for (size_t k = 0; k + 1 < src->size(); k += 2) n.ext.push_back((uint16_t)(((*src)[k] << 8) | (*src)[k + 1]));
		}
		break;
	}
	if (src && (mode == M_IND || mode == M_POST || mode == M_PRE || mode == M_D16))
		std::copy(src->begin(), src->end(), c.mem.begin() + off);
	return n;
}

uint16_t cmd_in(int fmt, int dst, int opmode) { return (uint16_t)(0x4000 | (fmt << 10) | (dst << 7) | opmode); }
uint16_t cmd_out(int fmt, int src, int k) { return (uint16_t)(0x6000 | (fmt << 10) | (src << 7) | (k & 0x7F)); }
uint16_t cmd_rr(int src, int dst, int opmode) { return (uint16_t)((src << 10) | (dst << 7) | opmode); }

Insn regreg(int src, int dst, int opmode)
{
	Insn n;
	n.op = 0xF200;
	n.cmd = cmd_rr(src, dst, opmode);
	return n;
}

// -------------------------------------------------------------- group 1: FMOVE <ea>,FPn
static std::string nm(const char *op, int fmt, int mode, int reg, int k)
{
	return std::string(op) + "." + fname(fmt) + " " + mname(mode) + ",FP" + std::to_string(reg) + " v" + std::to_string(k);
}

void g_fmove_in()
{
	static const int fmts[7] = {F_B, F_W, F_L, F_S, F_D, F_X, F_P};
	for (int fi = 0; fi < 7; fi++) {
		int fmt = fmts[fi];
		bool small = fmt == F_B || fmt == F_W || fmt == F_L || fmt == F_S;
		for (int k = 0; k < nvals(fmt); k++) {
			Bytes v = data_of(fmt, k);
			int dst = (k + fi) & 7;
			std::vector<int> ms = {M_IND};
			if (k < 3) { ms.push_back(M_POST); ms.push_back(M_PRE); ms.push_back(M_D16); }
			if (small) ms.push_back(M_DN);
			if (small || k < 2) ms.push_back(M_IMM);
			for (int m : ms) {
				uint32_t fpcr = (fmt == F_S || fmt == F_D) && (k & 1) ? 0x40 : 0;
				Case &c = new_case("fmove-in", nm("FMOVE", fmt, m, dst, k), k % 3, fpcr);
				c.body.push_back(make(c, cmd_in(fmt, dst, 0x00), m, 0, SRC, fsize(fmt), &v));
				if (m == M_IMM && !small) c.imm_issue = true;
			}
		}
	}
}

// -------------------------------------------------------------- group 2: FMOVE FPn,<ea>
void g_fmove_out()
{
	static const int fmts[7] = {F_B, F_W, F_L, F_S, F_D, F_X, F_P};
	static const uint32_t rm[4] = {0x00, 0x10, 0x20, 0x30};
	for (int fi = 0; fi < 7; fi++) {
		int fmt = fmts[fi];
		bool small = fmt == F_B || fmt == F_W || fmt == F_L || fmt == F_S;
		std::vector<int> ms = {M_IND, M_POST, M_PRE, M_D16};
		if (small) ms.insert(ms.begin(), M_DN);
		for (int pass = 0; pass < 2; pass++)
			for (int r = 0; r < 8; r++) {
				int m = ms[(size_t)(r + fi + pass) % ms.size()];
				uint32_t fpcr = rm[(r + pass) & 3] | ((fmt == F_S || fmt == F_D) && pass ? 0x40u : 0u);
				int kf = fmt == F_P ? (int[]){0, 5, 0x7D, 17, 0x3F, 1, 0x40, 2}[r] : 0;
				Case &c = new_case("fmove-out", std::string("FMOVE.") + fname(fmt) + " FP" + std::to_string(r) + "," + mname(m) +
				                   " img" + std::to_string(pass * 2) + " fpcr" + std::to_string(fpcr) + (fmt == F_P ? " k" + std::to_string(kf) : ""),
				                   pass * 2, fpcr);
				c.body.push_back(make(c, cmd_out(fmt, r, kf), m, 0, DST, fsize(fmt), nullptr));
			}
	}
	// rounding of integer conversions: image 1 (2.5, -2.5, 3.5, -0.5, 1e10, ...) in all four rounding modes
	for (int fmt : {F_B, F_W, F_L})
		for (int r = 0; r < 8; r++)
			for (int mi = 0; mi < 4; mi++) {
				Case &c = new_case("fmove-out", std::string("FMOVE.") + fname(fmt) + " FP" + std::to_string(r) + ",(An) img1 fpcr" + std::to_string(rm[mi]),
				                   1, rm[mi]);
				c.body.push_back(make(c, cmd_out(fmt, r, 0), M_IND, 0, DST, fsize(fmt), nullptr));
			}
	// float/double precision and rounding
	for (int fmt : {F_S, F_D})
		for (int r = 0; r < 8; r++)
			for (int mi = 0; mi < 4; mi++) {
				Case &c = new_case("fmove-out", std::string("FMOVE.") + fname(fmt) + " FP" + std::to_string(r) + ",(An) img1 fpcr" + std::to_string(rm[mi]),
				                   1, rm[mi]);
				c.body.push_back(make(c, cmd_out(fmt, r, 0), M_IND, 0, DST, fsize(fmt), nullptr));
			}
	// packed, dynamic k-factor in Dn (low 7 bits)
	for (int kv : {0, 3, 0x7D, 17, 0x41}) {
		for (int r : {0, 1, 2, 7}) {
			for (int m : {M_IND, M_D16}) {
				Case &c = new_case("fmove-out", "FMOVE.P FP" + std::to_string(r) + "," + mname(m) + " dynamic k=" + std::to_string(kv), r & 1 ? 1 : 0, 0);
				c.d[3] = 0xAB000000u | (uint32_t)kv;
				c.body.push_back(make(c, cmd_out(7, r, 3 << 4), m, 0, DST, 12, nullptr));
			}
		}
	}
}

// -------------------------------------------------------------- group 3: FMOVECR
void g_fmovecr()
{
	std::vector<int> offs = {0x00, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};
	for (int o = 0x30; o <= 0x3F; o++) offs.push_back(o);
	offs.push_back(0x01); offs.push_back(0x20); offs.push_back(0x40); offs.push_back(0x7F);
	static const uint32_t fp[4] = {0x00, 0x20, 0x40, 0x80};
	int n = 0;
	for (int o : offs)
		for (int v = 0; v < 2; v++) {
			uint32_t fpcr = v ? fp[(n++ % 3) + 1] | 0x10 : 0;
			Case &c = new_case("fmovecr", "FMOVECR #$" + [&] { char b[8]; snprintf(b, sizeof b, "%02X", o); return std::string(b); }() + ",FP" + std::to_string((o + v) & 7) +
			                   " fpcr" + std::to_string(fpcr), v, fpcr);
			Insn i;
			i.op = 0xF200;
			i.cmd = (uint16_t)(0x5C00 | (((o + v) & 7) << 7) | o);
			c.body.push_back(i);
		}
}

// -------------------------------------------------------------- group 4: control registers
uint16_t crlist(int l) { return (uint16_t)(((l & 1) ? 0x1000 : 0) | ((l & 2) ? 0x0800 : 0) | ((l & 4) ? 0x0400 : 0)); }   // l: bit0 FPCR, bit1 FPSR, bit2 FPIAR
void g_ctrl()
{
	static const uint32_t vcr[] = {0x00000000, 0x00000010, 0x00000020, 0x00000030, 0x00000040, 0x00000080, 0x000000B0};
	static const uint32_t vsr[] = {0x00000000, 0x0F000000, 0x08000000, 0x00000018, 0x00FF0000, 0x04000008, 0x0A0000F8};
	static const uint32_t via[] = {0x00000000, 0x12345678, 0xFFFFFFFF, 0x00E00400};
	auto vals = [&](int l, int k, Bytes &out) {
		out.clear();
		if (l & 1) { Bytes b = be(vcr[k % 7], 4); out.insert(out.end(), b.begin(), b.end()); }
		if (l & 2) { Bytes b = be(vsr[(k + 1) % 7], 4); out.insert(out.end(), b.begin(), b.end()); }
		if (l & 4) { Bytes b = be(via[(k + 2) % 4], 4); out.insert(out.end(), b.begin(), b.end()); }
	};
	auto lname = [&](int l) {
		std::string s;
		if (l & 1) s += "FPCR";
		if (l & 2) s += std::string(s.empty() ? "" : "/") + "FPSR";
		if (l & 4) s += std::string(s.empty() ? "" : "/") + "FPIAR";
		return s;
	};
	for (int l = 1; l <= 7; l++) {
		int nreg = ((l & 1) ? 1 : 0) + ((l & 2) ? 1 : 0) + ((l & 4) ? 1 : 0);
		bool one = nreg == 1;
		for (int k = 0; k < 3; k++) {
			Bytes v;
			vals(l, k, v);
			std::vector<int> ms = {M_IND, M_POST, M_D16, M_IMM};
			if (one) ms.push_back(M_DN);
			if (one && l == 4) ms.push_back(M_AN);
			for (int m : ms) {
				if (k > 0 && m != M_IND && m != M_IMM && !(one && (m == M_DN || m == M_AN))) continue;
				Case &c = new_case("ctrl-in", "FMOVE " + std::string(mname(m)) + "," + lname(l) + " v" + std::to_string(k), 0, 0);
				c.body.push_back(make(c, (uint16_t)(0x8000 | crlist(l)), m, 0, SRC, 4 * nreg, &v));
				if (m == M_IMM && !one) c.imm_issue = true;
			}
			// out: establish known FPSR and FPIAR first (single register moves with #imm)
			std::vector<int> mo = {M_IND, M_PRE, M_D16};
			if (one) mo.push_back(M_DN);
			if (one && l == 4) mo.push_back(M_AN);
			for (int m : mo) {
				if (k > 0 && m != M_IND && !(one && (m == M_DN || m == M_AN))) continue;
				Case &c = new_case("ctrl-out", "FMOVE " + lname(l) + "," + mname(m) + " v" + std::to_string(k), 0, vcr[k % 7]);
				Bytes s1 = be(vsr[(k + 3) % 7], 4), s2 = be(via[(k + 1) % 4], 4);
				c.body.push_back(make(c, 0x8800, M_IMM, 0, 0, 4, &s1));
				c.body.push_back(make(c, 0x8400, M_IMM, 0, 0, 4, &s2));
				c.body.push_back(make(c, (uint16_t)(0xA000 | crlist(l)), m, 0, DST, 4 * nreg, nullptr));
			}
		}
	}
}

// -------------------------------------------------------------- group 5: FMOVEM.X
void g_fmovem()
{
	static const int masks[] = {0x00, 0x01, 0x80, 0xFF, 0xA5, 0x5A, 0x0F, 0xF0, 0x24};
	auto cnt = [](int m) { int n = 0; for (int i = 0; i < 8; i++) n += (m >> i) & 1; return n; };
	int n = 0;
	for (int mask : masks) {
		int nr = cnt(mask);
		// to the FPU: (An) control, (An)+ , d16(An); static and dynamic
		for (int dyn = 0; dyn < 2; dyn++)
			for (int m : {M_IND, M_POST, M_D16}) {
				if (mask == 0x24 && m != M_IND) continue;
				Bytes img = image((n++) % 3);
				img.resize((size_t)(12 * nr == 0 ? 12 : 12 * nr));
				Case &c = new_case("fmovem-in", std::string("FMOVEM.X ") + mname(m) + ",mask $" + [&] { char b[8]; snprintf(b, sizeof b, "%02X", mask); return std::string(b); }() +
				                   (dyn ? " dynamic" : " static") + " (" + std::to_string(nr) + " regs)", n % 3, 0);
				c.d[2] = 0xCD000000u | (uint32_t)mask;
				uint16_t cmd = dyn ? (uint16_t)(0xD800 | (2 << 4)) : (uint16_t)(0xD000 | mask);
				c.body.push_back(make(c, cmd, m, 0, SRC, 12 * nr, &img));
			}
		// from the FPU: predecrement (reversed mask), control modes
		for (int dyn = 0; dyn < 2; dyn++)
			for (int m : {M_PRE, M_IND, M_D16}) {
				if (mask == 0x24 && m != M_PRE) continue;
				int rmask = 0;
				for (int b = 0; b < 8; b++) if (mask & (1 << b)) rmask |= 1 << (7 - b);   // predecrement order
				int use = m == M_PRE ? rmask : mask;
				Case &c = new_case("fmovem-out", std::string("FMOVEM.X ") + (m == M_PRE ? "" : "") + "FPn," + mname(m) + ",mask $" + [&] { char b[8]; snprintf(b, sizeof b, "%02X", use); return std::string(b); }() +
				                   (dyn ? " dynamic" : " static") + " (" + std::to_string(nr) + " regs)", n % 3, 0);
				c.d[2] = 0xCD000000u | (uint32_t)use;
				uint16_t base = (m == M_PRE) ? (dyn ? 0xE800 : 0xE000) : (dyn ? 0xF800 : 0xF000);
				uint16_t cmd = dyn ? (uint16_t)(base | (2 << 4)) : (uint16_t)(base | use);
				c.body.push_back(make(c, cmd, m, 0, DST, 12 * nr, nullptr));
				n++;
			}
	}
}

// -------------------------------------------------------------- group 6: arithmetic smoke
void g_arith()
{
	struct Op { const char *n; int code; };
	static const Op ops[] = {{"FADD", 0x22}, {"FSUB", 0x28}, {"FMUL", 0x23}, {"FDIV", 0x20}, {"FSQRT", 0x04}, {"FSIN", 0x0E},
	                         {"FABS", 0x18}, {"FNEG", 0x1A}, {"FCMP", 0x38}, {"FTST", 0x3A}, {"FINT", 0x01}, {"FMOVE", 0x00}};
	static const int pairs[][2] = {{0, 1}, {1, 2}, {2, 0}, {7, 3}};
	for (const Op &o : ops)
		for (int p = 0; p < 4; p++) {
			uint32_t fpcr = p == 3 ? 0x40 : (p == 2 ? 0x10 : 0);
			Case &c = new_case("arith", std::string(o.n) + ".X FP" + std::to_string(pairs[p][0]) + ",FP" + std::to_string(pairs[p][1]) + " fpcr" + std::to_string(fpcr),
			                   p & 1, fpcr);
			c.body.push_back(regreg(pairs[p][0], pairs[p][1], o.code));
		}
	// memory source: X, D, S, L
	static const int mf[4] = {F_X, F_D, F_S, F_L};
	for (int mi = 0; mi < 4; mi++)
		for (const Op &o : {ops[0], ops[2], ops[3], ops[4]})
			for (int k = 0; k < 2; k++) {
				Bytes v = data_of(mf[mi], k == 0 ? 2 : 6);
				if (mf[mi] == F_L) v = be(k ? 100 : 3, 4);
				Case &c = new_case("arith", std::string(o.n) + "." + fname(mf[mi]) + " (An),FP" + std::to_string(k + 1), k, 0);
				c.body.push_back(make(c, cmd_in(mf[mi], k + 1, o.code), M_IND, 0, SRC, fsize(mf[mi]), &v));
			}
}

// -------------------------------------------------------------- group 8: conditionals after real results
void g_cond()
{
	struct Prod { const char *n; int kind; int fmt; uint64_t v; int a, b; };
	static const Prod prods[] = {
	    {"L=0", 0, F_L, 0, 0, 0}, {"L=-5", 0, F_L, 0xFFFFFFFBull, 0, 0}, {"L=7", 0, F_L, 7, 0, 0},
	    {"S=qNaN", 0, F_S, 0x7FC00000ull, 0, 0}, {"S=-inf", 0, F_S, 0xFF800000ull, 0, 0}, {"S=+inf", 0, F_S, 0x7F800000ull, 0, 0},
	    {"S=-0", 0, F_S, 0x80000000ull, 0, 0}, {"D=sNaN", 0, F_D, 0x7FF4000000000000ull, 0, 0},
	    {"FCMP FP1,FP0 (greater)", 1, 0, 0, 1, 0}, {"FCMP FP0,FP1 (less)", 1, 0, 0, 0, 1}, {"FCMP FP0,FP0 (equal)", 1, 0, 0, 0, 0},
	    {"FCMP FP6,FP0 (NaN)", 1, 0, 0, 6, 0}, {"FTST FP5 (inf)", 2, 0, 0, 5, 0}, {"FTST FP4 (-0)", 2, 0, 0, 4, 0},
	    {"FTST FP6 (NaN)", 2, 0, 0, 6, 0}, {"FTST FP3 (+0)", 2, 0, 0, 3, 0}};
	std::vector<int> all;
	for (int p = 0; p < 32; p++) all.push_back(p);
	for (int bs = 0; bs < 2; bs++)
	for (const Prod &p : prods) {
		Case &c = new_case("cond", std::string("after ") + p.n + (bs ? " FPCR BSUN enabled" : ""), 0, bs ? 0x8000u : 0u);
		c.soft_cond = true;
		if (p.kind == 0) {
			Bytes v = be(p.v, fsize(p.fmt));
			c.body.push_back(make(c, cmd_in(p.fmt, 0, 0x00), M_IND, 0, SRC, fsize(p.fmt), &v));
		} else if (p.kind == 1)
			c.body.push_back(regreg(p.a, p.b, 0x38));
		else
			c.body.push_back(regreg(p.a, 0, 0x3A));
		c.spreds = all;
		c.bpreds = all;
	}
}

// -------------------------------------------------------------- all 16 condition code combinations x 32 predicates
// (FPSR written directly, so combinations no instruction produces are included)
void g_cctable()
{
	std::vector<int> all;
	for (int p = 0; p < 32; p++) all.push_back(p);
	for (int cc = 0; cc < 16; cc++) {
		Case &c = new_case("cctable", "FPSR cc=" + std::string(1, "0123456789ABCDEF"[cc]) + " {N Z I NAN}=" + std::to_string((cc >> 3) & 1) + std::to_string((cc >> 2) & 1) + std::to_string((cc >> 1) & 1) + std::to_string(cc & 1), 0, 0);
		c.soft_cond = true;
		Bytes v = be((uint32_t)cc << 24, 4);
		c.body.push_back(make(c, 0x8800, M_IMM, 0, 0, 4, &v));
		c.spreds = all;
		c.bpreds = {0x01, 0x0E, 0x0A, 0x1E};
	}
}

// -------------------------------------------------------------- FScc <ea> address register updates
void g_fscc_ea()
{
	for (int p : {0x0F, 0x00, 0x02}) {
		for (int m : {M_DN, M_IND, M_POST, M_PRE, M_D16}) {
			Case &c = new_case("fscc-ea", std::string("FScc ") + mname(m) + " pred $" + (p == 0x0F ? "0F" : p == 0 ? "00" : "02"), 0, 0);
			Insn n = make(c, (uint16_t)p, m, 0, DST, 1, nullptr);
			n.op = (uint16_t)(0xF240 | ea_of(m, 0));
			n.scc = true;
			c.body.push_back(n);
		}
	}
}

// -------------------------------------------------------------- group 9: FRESTORE null resets the FPU
void g_reset()
{
	for (int im = 0; im < 3; im++) {
		Case &c = new_case("reset", "FRESTORE null after a loaded state, img" + std::to_string(im), im, 0x20);
		Bytes s1 = be(0x0F000018u, 4), s2 = be(0x12345678u, 4);
		c.body.push_back(make(c, 0x8800, M_IMM, 0, 0, 4, &s1));
		c.body.push_back(make(c, 0x8400, M_IMM, 0, 0, 4, &s2));
		Insn r;
		r.reset = true;
		c.body.push_back(r);
	}
}

// -------------------------------------------------------------- group 11: milestone 4 (exceptions, FPIAR, frames)
const uint32_t EN_SNAN = 0x4000, EN_OPERR = 0x2000, EN_OVFL = 0x1000, EN_UNFL = 0x0800, EN_DZ = 0x0400, EN_INEX2 = 0x0200, EN_INEX1 = 0x0100;
const uint32_t FR_OFF = 0x600;                                 // frame area (FSAVE -(A1) with A1 = window + FR_OFF)

Insn fsave_i(Case &c, int mode, uint32_t off)           // FSAVE -(A1) / (A1) / d16(A1), the frame image ends at / starts at off
{
	Insn n;
	n.frame = 1;
	int i = cidx();
	n.op = (uint16_t)(0xF300 | ea_of(mode, 1));
	if (mode == M_PRE) n.pre.push_back({9, W(i) + off});
	else if (mode == M_IND) n.pre.push_back({9, W(i) + off});
	else { n.pre.push_back({9, W(i) + off - 0x20}); n.ext.push_back(0x20); }
	c.a[1] = n.pre.back().second;
	return n;
}
Insn frestore_i(Case &c, int mode, uint32_t off, bool setreg = true)       // FRESTORE (A1)+ / (A1) / d16(A1)
{
	Insn n;
	n.frame = 2;
	int i = cidx();
	n.op = (uint16_t)(0xF340 | ea_of(mode, 1));
	if (setreg) {
		if (mode == M_D16) { n.pre.push_back({9, W(i) + off - 0x20}); n.ext.push_back(0x20); }
		else n.pre.push_back({9, W(i) + off});
	} else if (mode == M_D16)
		n.ext.push_back(0x20);
	return n;
}
Insn bset27_i(uint32_t woff) { Insn n; n.bset27 = true; n.bset_addr = woff; return n; }

struct Raise { const char *n; uint32_t en; int kind; int a, b, op; };    // kind 0 reg-reg a->b, 1 FMOVE.P mem
const Raise RAISE[] = {
    {"SNAN (FADD sNaN)", EN_SNAN, 0, 0, 5, 0x22},     {"OPERR (FSQRT -1)", EN_OPERR, 0, 1, 5, 0x04},
    {"OVFL (FMUL huge*huge)", EN_OVFL, 0, 2, 2, 0x23}, {"UNFL (FMUL tiny*tiny)", EN_UNFL, 0, 3, 3, 0x23},
    {"DZ (FDIV 1/0)", EN_DZ, 0, 4, 5, 0x20},           {"INEX2 (FDIV 1/3)", EN_INEX2, 0, 6, 5, 0x20},
    {"INEX1 (FMOVE.P 0.1)", EN_INEX1, 1, 0, 5, 0}};

Insn raise_i(Case &c, const Raise &r)
{
	if (r.kind == 0) return regreg(r.a, r.b, r.op);
	Bytes v = pval(0x40010001u, 0, 0);                        // 1.0E-1: inexact decimal input
	return make(c, cmd_in(F_P, r.b, 0x00), M_IND, 0, SRC, 12, &v);
}

void g_m4()
{
	const Insn dummy;
	(void)dummy;
	// ---- enabled exceptions: raised by an instruction, reported to the next 000/010/011 instruction
	for (const Raise &r : RAISE) {
		for (int variant = 0; variant < 3; variant++) {
			const char *vn[3] = {"handler absorbs (FSAVE/FRESTORE epilogue)", "handler only returns: re-reported, FMOVEM/FPcr moves execute, FSAVE absorbs, FRESTORE re-arms",
			                     "FSAVE frame with bit 27 set is restored: not re-armed"};
			Case &c = new_case("m4-exc", std::string(r.n) + ": " + vn[variant], 3, r.en);
			c.softall = true;
			c.hmode = variant == 1 ? 1 : 0;
			int i = cidx();
			(void)i;
			c.body.push_back(raise_i(c, r));
			Insn x = regreg(6, 7, 0x22);                      // FADD FP6,FP7: the reporting instruction
			if (variant == 0) {
				c.body.push_back(x);                          // reported ($1Cvv), handler absorbs
				c.body.push_back(x);                          // executes
			} else if (variant == 1) {
				c.body.push_back(x);                          // reported
				c.body.push_back(x);                          // reported again (the 68882 keeps it pending)
				Bytes img = image(3);
				img.resize(24);
				c.body.push_back(make(c, 0xD0C0, M_IND, 0, SRC + 0x40, 24, &img));   // FMOVEM.X (A0),FP0-FP1 (mask $C0): not reported
				c.body.push_back(make(c, 0xA000 | 0x0800, M_DN, 0, 0, 4, nullptr));      // FMOVE.L FPSR,D0: not reported
				c.body.push_back(fsave_i(c, M_PRE, FR_OFF));  // absorbs (bit 27 = 0)
				c.body.push_back(x);                          // executes
				c.body.push_back(frestore_i(c, M_POST, FR_OFF - 60, false));   // re-arms
				c.body.push_back(x);                          // reported
				c.body.push_back(fsave_i(c, M_PRE, FR_OFF + 0x80));   // absorbs again
			} else {
				c.body.push_back(fsave_i(c, M_PRE, FR_OFF));
				c.body.push_back(bset27_i(FR_OFF - 60 + 56));
				c.body.push_back(frestore_i(c, M_POST, FR_OFF - 60, false));
				c.body.push_back(x);                          // executes: not re-armed
			}
		}
	}
	// ---- several exceptions enabled: the priority of the pending vector
	{
		struct Pri { const char *n; uint32_t en; int kind; int a, b, op; };
		static const Pri pri[] = {{"SNAN+OPERR (FADD sNaN)", EN_SNAN | EN_OPERR, 0, 0, 5, 0x22}, {"OVFL+INEX2 (FMUL huge*huge)", EN_OVFL | EN_INEX2, 0, 2, 2, 0x23},
		                          {"UNFL+INEX2 (FMUL tiny*tiny)", EN_UNFL | EN_INEX2, 0, 3, 3, 0x23}, {"all enabled (FDIV 0/0)", 0x7F00, 0, 4, 4, 0x20},
		                          {"all enabled (FDIV 1/3)", 0x7F00, 0, 6, 5, 0x20}, {"OPERR+DZ (FDIV 1/0)", EN_OPERR | EN_DZ, 0, 4, 5, 0x20}};
		for (const Pri &p : pri) {
			Case &c = new_case("m4-exc", std::string("priority ") + p.n, 3, p.en);
			c.softall = true;
			c.body.push_back(regreg(p.a, p.b, p.op));
			c.body.push_back(regreg(6, 7, 0x22));             // reported
			c.body.push_back(regreg(6, 7, 0x22));
		}
	}
	// ---- the reporting instruction is a conditional
	for (const Raise &r : RAISE) {
		for (int kind = 1; kind <= 4; kind++) {
			Case &c = new_case("m4-exc", std::string(r.n) + ": reported by " + (kind == 1 ? "FScc" : kind == 2 ? "FBcc" : kind == 3 ? "FDBcc" : "FTRAPcc"), 3, r.en);
			c.softall = true;
			c.body.push_back(raise_i(c, r));
			Insn n;
			n.ckind = kind;
			n.pred = 0x0F;
			n.slot = kind == 1 ? 0 : BPA;
			c.body.push_back(n);
			c.body.push_back(n);                               // after the handler absorbed it: evaluated normally
			if (kind != 1) c.body.back().slot = BPA + 4;
		}
	}
	// ---- FMOVE out raising an enabled exception itself: result stored, mid-instruction exception (format 9)
	{
		struct Out { const char *n; uint32_t en; int fmt, src; };
		static const Out outs[] = {{"OPERR FMOVE.B huge", EN_OPERR, F_B, 2}, {"OVFL FMOVE.S huge", EN_OVFL, F_S, 2}, {"UNFL FMOVE.S tiny", EN_UNFL, F_S, 3},
		                           {"INEX2 FMOVE.S 0.1", EN_INEX2, F_S, 7}, {"SNAN FMOVE.S sNaN", EN_SNAN, F_S, 0}, {"INEX2 FMOVE.D 0.1", EN_INEX2, F_D, 7},
		                           {"OPERR FMOVE.W -1/huge", EN_OPERR, F_W, 2}, {"OVFL FMOVE.D huge", EN_OVFL, F_D, 2}, {"INEX2 FMOVE.P 0.1", EN_INEX2, F_P, 7}};
		g_isolate = true;      // FMOVE out with an exception enabled: the bridge's first response lacks the PC bit and the CPU waits forever
		for (const Out &o : outs)
			for (int m : {M_IND, M_PRE, M_DN}) {
				if (m == M_DN && o.fmt == F_D) continue;
				if (m == M_DN && o.fmt == F_P) continue;
				Case &c = new_case("m4-excout", std::string(o.n) + " -> " + mname(m), 3, o.en);
				c.softall = true;
				c.body.push_back(make(c, cmd_out(o.fmt, o.src, o.fmt == F_P ? 5 : 0), m, 0, DST, fsize(o.fmt), nullptr));
				c.body.push_back(regreg(6, 7, 0x22));
			}
		g_isolate = false;
		// packed with a dynamic k-factor (the PC bit is set on the Dn request, so these do not hang)
		struct Dk { const char *n; uint32_t en; int src; };
		static const Dk dks[] = {{"INEX2 FMOVE.P 0.1", EN_INEX2, 7}, {"SNAN FMOVE.P sNaN", EN_SNAN, 0}, {"all enabled FMOVE.P huge", 0x7F00, 2},
		                         {"all enabled FMOVE.P tiny", 0x7F00, 3}};
		for (const Dk &d : dks)
			for (int m : {M_IND, M_D16}) {
				Case &c = new_case("m4-excout", std::string(d.n) + " dynamic k -> " + mname(m), 3, d.en);
				c.softall = true;
				Insn n = make(c, cmd_out(7, d.src, 3 << 4), m, 0, DST, 12, nullptr);
				n.pre.push_back({3, 0xAB000003u});
				c.d[3] = 0xAB000003u;
				c.body.push_back(n);
				c.body.push_back(regreg(6, 7, 0x22));
			}
	}
	// ---- FPIAR: loaded with the instruction address while an arithmetic exception is enabled
	{
		static const uint32_t pats[6] = {0, 0x8000, EN_SNAN, EN_OPERR | EN_OVFL, EN_INEX1, 0xFF00};
		const char *pn[6] = {"none", "BSUN only", "SNAN", "OPERR+OVFL", "INEX1", "all"};
		for (int p = 0; p < 6; p++) {
			for (int kind = 0; kind < 10; kind++) {
				static const char *kn[10] = {"FADD reg-reg", "FADD.X (A0)", "FADD.X (A0)+", "FADD.X d16(A0)", "FADD.X (d16,PC)", "FADD.X #imm", "FMOVECR",
				                             "FMOVE.X out", "FMOVE.P out static k", "FMOVE.P out dynamic k"};
				g_isolate = kind >= 7 && (pats[p] & 0x7F00);
				Case &c = new_case("m4-fpiar", std::string("FPCR enables ") + pn[p] + ": " + kn[kind], 3, pats[p]);
				g_isolate = false;
				c.softall = true;
				Bytes iv = be(0x13572468u, 4);
				c.body.push_back(make(c, 0x8400, M_IMM, 0, 0, 4, &iv));     // FMOVE.L #imm,FPIAR: a known FPIAR
				Bytes xv = xval(0x4000, 0xA000000000000000ull);
				switch (kind) {
				case 0: c.body.push_back(regreg(6, 7, 0x22)); break;
				case 1: c.body.push_back(make(c, cmd_in(F_X, 5, 0x22), M_IND, 0, SRC, 12, &xv)); break;
				case 2: c.body.push_back(make(c, cmd_in(F_X, 5, 0x22), M_POST, 0, SRC, 12, &xv)); break;
				case 3: c.body.push_back(make(c, cmd_in(F_X, 5, 0x22), M_D16, 0, SRC, 12, &xv)); break;
				case 4: {
					Insn n;
					n.op = 0xF200 | 0x3A;
					n.cmd = cmd_in(F_X, 5, 0x22);
					n.pcrel = true;
					n.pcdata = xv;
					c.body.push_back(n);
					break;
				}
				case 5: c.body.push_back(make(c, cmd_in(F_X, 5, 0x22), M_IMM, 0, SRC, 12, &xv)); break;
				case 6: { Insn n; n.op = 0xF200; n.cmd = 0x5C00 | (5 << 7) | 0x0B; c.body.push_back(n); break; }
				case 7: c.body.push_back(make(c, cmd_out(F_X, 6, 0), M_IND, 0, DST, 12, nullptr)); break;
				case 8: c.body.push_back(make(c, cmd_out(F_P, 6, 5), M_IND, 0, DST, 12, nullptr)); break;
				default: {
					Insn n = make(c, cmd_out(7, 6, 3 << 4), M_IND, 0, DST, 12, nullptr);
					n.pre.push_back({3, 0xAB000003u});
					c.d[3] = 0xAB000003u;
					c.body.push_back(n);
				}
				}
			}
		}
	}
	// ---- FSAVE / FRESTORE frames
	{
		struct Em { int smode, rmode; };
		static const Em ems[] = {{M_PRE, M_POST}, {M_IND, M_IND}, {M_D16, M_D16}, {M_PRE, M_IND}, {M_IND, M_POST}};
		for (const Em &e : ems) {
			for (int state = 0; state < 4; state++) {
				const char *sn[4] = {"null", "idle", "exception pending", "after a background FMUL"};
				Case &c = new_case("m4-frame", std::string("FSAVE ") + mname(e.smode) + " / FRESTORE " + mname(e.rmode) + ", FPU " + sn[state], 3, state == 2 ? EN_OVFL : 0);
				c.softall = true;
				if (state == 0) { Insn r; r.reset = true; c.body.push_back(r); }
				if (state == 2) c.body.push_back(regreg(2, 2, 0x23));
				if (state == 3) c.body.push_back(regreg(2, 6, 0x23));
				uint32_t fo = FR_OFF + (e.smode == M_PRE ? 0 : 0);
				Insn fs = fsave_i(c, e.smode, e.smode == M_PRE ? fo : fo - 60);
				c.body.push_back(fs);
				Insn fr = frestore_i(c, e.rmode, fo - 60);
				c.body.push_back(fr);
				c.body.push_back(fsave_i(c, M_PRE, FR_OFF + 0x100));   // the state after the restore
			}
		}
		// FRESTORE of frames built in memory
		struct Fm { const char *n; uint16_t fmt; int body_len; uint32_t biu; };
		static const Fm fms[] = {{"null $0038", 0x0038, 0, 0}, {"null $0000", 0x0000, 0, 0}, {"idle $1F38 BIU 5C0EFFFF", 0x1F38, 56, 0x5C0EFFFFu},
		                         {"idle $1F38 BIU 740EFFFF (exception armed)", 0x1F38, 56, 0x740EFFFFu}, {"68881 idle $1F18", 0x1F18, 24, 0x5C0EFFFFu},
		                         {"busy $1FD4", 0x1FD4, 0xD4, 0x5C0EFFFFu}, {"busy $1FB4", 0x1FB4, 0xB4, 0x5C0EFFFFu}, {"invalid $1234", 0x1234, 0, 0},
		                         {"invalid $1F00", 0x1F00, 0, 0}};
		for (const Fm &f : fms)
			for (int armed = 0; armed < 2; armed++) {
				if (armed && f.biu != 0x740EFFFFu) continue;
				Case &c = new_case("m4-frame", std::string("FRESTORE ") + f.n + (armed ? " with OVFL enabled and set in FPSR" : ""), 3, armed ? EN_OVFL : 0);
				c.softall = true;
				Bytes fr = be(((uint32_t)f.fmt << 16) | 0, 4);
				for (int k = 4; k < 4 + f.body_len; k += 4) {
					uint32_t v = 0x11110000u + (uint32_t)k * 0x01010101u;
					if (k == 4 + f.body_len - 4 && f.biu) v = f.biu;
					Bytes b = be(v, 4);
					fr.insert(fr.end(), b.begin(), b.end());
				}
				std::copy(fr.begin(), fr.end(), c.mem.begin() + 0x400);
				if (armed) {
					Bytes fp = be(0x00001000u, 4);                       // FPSR: OVFL exception status set
					c.body.push_back(make(c, 0x8800, M_IMM, 0, 0, 4, &fp));
				}
				c.body.push_back(frestore_i(c, M_POST, 0x400));
				c.body.push_back(regreg(6, 7, 0x22));                      // an armed exception is reported here
				c.body.push_back(fsave_i(c, M_PRE, FR_OFF + 0x100));
			}
	}
}

// -------------------------------------------------------------- group 10: random streams
static const int DOCOPS[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x06, 0x08, 0x09, 0x0A, 0x0C, 0x0D, 0x0E, 0x0F, 0x10, 0x11, 0x12, 0x14, 0x15, 0x16,
                             0x18, 0x19, 0x1A, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27, 0x28,
                             0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x3A};

// opmodes Hatari's fault_if_nonexisting_opmode refuses (fpp.c): F-line ($42, $43, $46-$57, $59, ... $6D) and from $6E-$77, vector 4 for $78-$7F
bool hatari_refused(int op)
{
	static const int l[] = {0x42, 0x43, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56,
	                        0x57, 0x59, 0x5b, 0x5d, 0x5f, 0x61, 0x65, 0x69, 0x6a, 0x6b, 0x6d};
	if (op >= 0x6E) return true;
	for (int x : l) if (x == op) return true;
	return false;
}

bool documented(int op)
{
	for (int d : DOCOPS) if (d == op) return true;
	return false;
}

void g_random(int nseq)
{
	std::mt19937 rg(0xF9B20003u);
	auto R = [&](uint32_t n) { return (uint32_t)(rg() % n); };
	auto rbytes = [&](int n) { Bytes b((size_t)n); for (auto &x : b) x = (uint8_t)rg(); return b; };
	auto value = [&](int fmt) -> Bytes {
		if (R(2)) return data_of(fmt, (int)R(32));
		Bytes b = rbytes(fsize(fmt));
		if (fmt == F_X) { b[2] = b[3] = 0; }
		return b;
	};
	for (int sq = 0; sq < nseq; sq++) {
		uint32_t fpcr = (R(4) << 4) | ((R(4) == 0 ? 1 : R(3) == 0 ? 2 : 0) << 6);
		if (R(8) == 0) fpcr |= 0x8000;                    // BSUN enabled
		bool keep = !getenv("M2_SEQ") || atoi(getenv("M2_SEQ")) == sq;          // debugging aid: one sequence only (same random stream)
		Case &c = new_case("random", "sequence " + std::to_string(sq), (int)R(3), fpcr);
		c.soft_cond = true;
		int n = 12 + (int)R(9);
		int ncond = 0;
		for (int k = 0; k < n; k++) {
			uint32_t soff = (uint32_t)k * 0x60, doff = soff + 0x30;
			int kind = (int)R(100);
			if (kind < 18) {                                // FMOVE <ea>,FPn
				int fmt = (int)R(7);
				bool small = fmt == F_B || fmt == F_W || fmt == F_L || fmt == F_S;
				int ms[] = {M_IND, M_POST, M_PRE, M_D16, M_IMM, M_DN};
				int m = ms[R(small ? 6 : 5)];
				Bytes v = value(fmt);
				c.body.push_back(make(c, cmd_in(fmt, (int)R(8), 0x00), m, 0, soff, fsize(fmt), &v));
			} else if (kind < 32) {                         // FMOVE FPn,<ea>
				int fmt = (int)R(7);
				bool small = fmt == F_B || fmt == F_W || fmt == F_L || fmt == F_S;
				int ms[] = {M_IND, M_POST, M_PRE, M_D16, M_DN};
				int m = ms[R(small ? 5 : 4)];
				int kf = 0;
				if (fmt == F_P) kf = (int)R(2) ? (int)R(18) : -(int)R(8);
				if (fmt == F_P && R(4) == 0) {              // dynamic k-factor in D3
					Insn n2 = make(c, cmd_out(7, (int)R(8), 3 << 4), R(2) ? M_IND : M_D16, 0, doff, 12, nullptr);
					n2.pre.push_back({3, 0xAB000000u | R(0x80)});
					c.d[3] = n2.pre.back().second;
					c.body.push_back(n2);
				} else
					c.body.push_back(make(c, cmd_out(fmt, (int)R(8), kf), m, 0, doff, fsize(fmt), nullptr));
			} else if (kind < 52) {                         // arithmetic, register to register
				int op;
				uint32_t t = R(100);
				bool soft = false;
				if (t < 88) op = DOCOPS[R(sizeof(DOCOPS) / sizeof(DOCOPS[0]))];
				else { do op = (int)R(0x6E); while (documented(op) || hatari_refused(op)); }
				Insn n2 = regreg((int)R(8), (int)R(8), op);
				n2.soft = soft;
				c.body.push_back(n2);
			} else if (kind < 64) {                         // arithmetic from memory
				int fmt = (int)R(7);
				bool small = fmt == F_B || fmt == F_W || fmt == F_L || fmt == F_S;
				int ms[] = {M_IND, M_POST, M_PRE, M_D16, M_IMM, M_DN};
				int m = ms[R(small ? 6 : 5)];
				int op;
				uint32_t t = R(100);
				bool soft = false;
				if (t < 85) op = DOCOPS[R(sizeof(DOCOPS) / sizeof(DOCOPS[0]))];
				else if (t < 95) { do op = (int)R(0x6E); while (documented(op) || hatari_refused(op)); }
				else { op = 0x6E + (int)R(18); soft = true; }
				Bytes v = value(fmt);
				Insn n2 = make(c, cmd_in(fmt, (int)R(8), op), m, 0, soff, fsize(fmt), &v);
				n2.soft = soft;
				if (soft) {                                  // no operand transfer: a post-increment/pre-decrement register is not touched
				}
				c.body.push_back(n2);
			} else if (kind < 68) {                         // FMOVECR
				Insn n2;
				n2.op = 0xF200;
				n2.cmd = (uint16_t)(0x5C00 | (R(8) << 7) | (R(3) ? (R(2) ? 0x30 + R(16) : 0x0B + R(5)) : R(0x80)));
				c.body.push_back(n2);
			} else if (kind < 76) {                         // control register moves
				int l = 1 + (int)R(7);
				int nreg = ((l & 1) ? 1 : 0) + ((l & 2) ? 1 : 0) + ((l & 4) ? 1 : 0);
				bool one = nreg == 1;
				bool in = R(2);
				Bytes v;
				if (l & 1) { Bytes b = be((R(4) << 4) | ((R(3)) << 6) | (R(10) == 0 ? 0x8000 : 0), 4); v.insert(v.end(), b.begin(), b.end()); }
				if (l & 2) { Bytes b = be(rg() & 0x0FFFFFF8u, 4); v.insert(v.end(), b.begin(), b.end()); }
				if (l & 4) { Bytes b = be(rg(), 4); v.insert(v.end(), b.begin(), b.end()); }
				if (in) {
					std::vector<int> ms = {M_IND, M_POST, M_D16, M_IMM};
					if (one) ms.push_back(M_DN);
					if (one && l == 4) ms.push_back(M_AN);
					c.body.push_back(make(c, (uint16_t)(0x8000 | crlist(l)), ms[R((uint32_t)ms.size())], 0, soff, 4 * nreg, &v));
				} else {
					std::vector<int> ms = {M_IND, M_PRE, M_D16};
					if (one) ms.push_back(M_DN);
					if (one && l == 4) ms.push_back(M_AN);
					c.body.push_back(make(c, (uint16_t)(0xA000 | crlist(l)), ms[R((uint32_t)ms.size())], 0, doff, 4 * nreg, nullptr));
				}
			} else if (kind < 86) {                         // FMOVEM.X
				int mask = (int)R(256);
				int nr = 0;
				for (int b = 0; b < 8; b++) nr += (mask >> b) & 1;
				if (nr > 4) { mask &= 0x0F << (int)(R(2) * 4); nr = 0; for (int b = 0; b < 8; b++) nr += (mask >> b) & 1; }
				bool dyn = R(3) == 0;
				if (R(2)) {                                  // into the FPU
					static const int mi3[3] = {M_IND, M_POST, M_D16};
					int m = mi3[R(3)];
					Bytes img = rbytes(12 * (nr ? nr : 1));
					for (size_t b = 0; b + 12 <= img.size(); b += 12) { img[b + 2] = img[b + 3] = 0; }
					uint16_t cmd = dyn ? (uint16_t)(0xD800 | (2 << 4)) : (uint16_t)(0xD000 | mask);
					Insn n2 = make(c, cmd, m, 0, soff, 12 * nr, &img);
					if (dyn) { n2.pre.push_back({2, 0xCD000000u | (uint32_t)mask}); c.d[2] = n2.pre.back().second; }
					c.body.push_back(n2);
				} else {
					static const int mo3[3] = {M_PRE, M_IND, M_D16};
					int m = mo3[R(3)];
					uint16_t base = (m == M_PRE) ? (dyn ? 0xE800 : 0xE000) : (dyn ? 0xF800 : 0xF000);
					uint16_t cmd = dyn ? (uint16_t)(base | (2 << 4)) : (uint16_t)(base | mask);
					Insn n2 = make(c, cmd, m, 0, doff, 12 * nr, nullptr);
					if (dyn) { n2.pre.push_back({2, 0xCD000000u | (uint32_t)mask}); c.d[2] = n2.pre.back().second; }
					c.body.push_back(n2);
				}
			} else {                                        // conditionals
				if (ncond >= 30) { k--; continue; }
				Insn n2;
				n2.ckind = 1 + (int)R(4);
				if (n2.ckind == 1 && R(2)) n2.ckind = 2;
				n2.pred = (int)R(32);
				n2.slot = BPA + 4 * (uint32_t)ncond;
				if (n2.ckind == 1) n2.slot = 0;
				ncond++;
				c.body.push_back(n2);
			}
		}
		if (!keep) pop_case();
	}
}

// -------------------------------------------------------------- walking a case
struct Sink {
	virtual ~Sink() {}
	virtual void begin(int i, const Case &c) = 0;
	virtual void setreg(int r, uint32_t v) = 0;
	virtual void fpu(const Insn &n, int k, const Case &c) = 0;
	virtual void reset() = 0;
	virtual void scc(int p, uint32_t a5, bool soft) = 0;
	virtual void mbcc(int p, uint32_t addr) = 0;
	virtual void mdbcc(int p, uint32_t addr) = 0;
	virtual void mtrap(int p, uint32_t addr) = 0;
	virtual void dumpregs(uint32_t addr) = 0;
	virtual void marker(uint32_t addr, uint32_t v) = 0;
	virtual void end(int i) = 0;
};

void walk(int i, Sink &s)
{
	const Case &c = cases[(size_t)i];
	int k = 0;
	s.begin(i, c);
	s.setreg(14, W(i) + NULLF);
	s.reset();
	s.setreg(14, W(i) + FPI);
	Insn fm;
	fm.op = 0xF216; fm.cmd = 0xD0FF;                       // FMOVEM.X (a6),FP0-FP7
	s.fpu(fm, k++, c);
	Insn fc;
	fc.op = 0xF23C; fc.cmd = 0x9000;                       // FMOVE.L #fpcr,FPCR
	fc.ext = {(uint16_t)(c.fpcr >> 16), (uint16_t)c.fpcr};
	s.fpu(fc, k++, c);
	for (int r = 0; r < 8; r++) s.setreg(r, c.d[r]);
	for (int r = 0; r < 6; r++) s.setreg(8 + r, c.a[r]);
	uint32_t a5 = W(i) + CND;
	for (const Insn &n : c.body) {
		for (auto &pr : n.pre) s.setreg(pr.first, pr.second);
		if (n.reset) { s.setreg(14, W(i) + NULLF); s.reset(); }
		else if (n.ckind == 1) { s.scc(n.pred, a5, true); a5++; }
		else if (n.ckind == 2) s.mbcc(n.pred, W(i) + n.slot);
		else if (n.ckind == 3) s.mdbcc(n.pred, W(i) + n.slot + 1);
		else if (n.ckind == 4) s.mtrap(n.pred, W(i) + n.slot + 3);
		else s.fpu(n, k++, c);
	}
	for (int p : c.spreds) { s.scc(p, a5, c.soft_cond); a5++; }
	for (size_t b = 0; b < c.bpreds.size(); b++) {
		uint32_t base = W(i) + BPA + 4 * (uint32_t)b;
		s.mbcc(c.bpreds[b], base);
		s.mdbcc(c.bpreds[b], base + 1);
		s.mtrap(c.bpreds[b], base + 3);
	}
	uint32_t d = DU(i);
	Insn dm;
	dm.op = 0xF239; dm.cmd = 0xF0FF; dm.ext = {(uint16_t)(d >> 16), (uint16_t)d};       // FMOVEM.X FP0-FP7,(abs).L
	s.fpu(dm, k++, c);
	Insn dc;
	dc.op = 0xF239; dc.cmd = 0xBC00; dc.ext = {(uint16_t)((d + 96) >> 16), (uint16_t)(d + 96)};   // FMOVEM.L FPCR/FPSR/FPIAR,(abs).L
	s.fpu(dc, k++, c);
	s.dumpregs(d + 108);
	s.marker(d + 0xFC, 0xC0DE0000u + (uint32_t)i);
	s.end(i);
}

int insn_words(const Insn &n) { return (n.frame ? 1 : 2) + (int)n.ext.size() + (n.pcrel ? 1 : 0); }

struct AsmSink : Sink {
	FILE *f;
	int ci = 0, nc = 0, cur_k = 0;
	explicit AsmSink(FILE *fp) : f(fp) {}
	void begin(int i, const Case &c) override
	{
		ci = i;
		nc = 0;
		fprintf(f, "; ---- case %d: %s / %s\n", i, c.group.c_str(), c.name.c_str());
		fprintf(f, "case%d:\n\tmove.l\t#%d,CASEIX\n\tlea\tend%d(pc),a0\n\tmove.l\ta0,RESUME\n\tmove.l\t#%d,HMODE\n", i, i, i, c.hmode);
	}
	void setreg(int r, uint32_t v) override
	{
		if (r < 8) {
			if (v == 0) fprintf(f, "\tmoveq\t#0,d%d\n", r);
			else fprintf(f, "\tmove.l\t#$%08X,d%d\n", v, r);
		} else
			fprintf(f, "\tmovea.l\t#$%08X,a%d\n", v, r - 8);
	}
	void fpu(const Insn &n, int k, const Case &c) override
	{
		bool soft = n.soft || c.softall;
		if (soft) fprintf(f, "\tmove.l\t#%d,SKIP\n\tmove.l\t#1,SOFTEXC\n", 2 * insn_words(n));
		if (n.bset27) {
			fprintf(f, "L%d_%d:\tbset\t#3,$%X\n", ci, k, W(ci) + n.bset_addr);
			if (soft) fprintf(f, "\tclr.l\tSOFTEXC\n");
			return;
		}
		fprintf(f, "L%d_%d:\tdc.w\t$%04X", ci, k, n.op);
		if (!n.frame) fprintf(f, ",$%04X", n.cmd);
		for (uint16_t e : n.ext) fprintf(f, ",$%04X", e);
		fprintf(f, "\n");
		if (n.pcrel) fprintf(f, "\tdc.w\tD%d_%d-*\n", ci, k);
		if (soft) fprintf(f, "\tclr.l\tSOFTEXC\n");
		if (n.pcrel) {
			fprintf(f, "\tbra.s\tP%d_%d\nD%d_%d:\tdc.b\t", ci, k, ci, k);
			for (size_t j = 0; j < n.pcdata.size(); j++) fprintf(f, "%s$%02X", j ? "," : "", n.pcdata[j]);
			fprintf(f, "\n\teven\nP%d_%d:\n", ci, k);
		}
	}
	void reset() override { fprintf(f, "\tdc.w\t$F356\t\t; frestore (a6)\n"); }
	void scc(int p, uint32_t, bool soft) override
	{
		if (soft) fprintf(f, "\tmove.l\t#4,SKIP\n\tmove.l\t#1,SOFTEXC\n");
		fprintf(f, "L%d_c%d:\tdc.w\t$F255,$%04X\t; fscc (a5)\n", ci, nc++, p);
		if (soft) fprintf(f, "\tclr.l\tSOFTEXC\n");
		fprintf(f, "\taddq.l\t#1,a5\n");
	}
	void mbcc(int p, uint32_t a) override { fprintf(f, "\tM2BCC\t%d,$%X,L%d_c%d\n", p, a, ci, nc++); }
	void mdbcc(int p, uint32_t a) override { fprintf(f, "\tM2DBCC\t%d,$%X,L%d_c%d\n", p, a, ci, nc++); }
	void mtrap(int p, uint32_t a) override { fprintf(f, "\tM2TRAP\t%d,$%X,L%d_c%d\n", p, a, ci, nc++); }
	void dumpregs(uint32_t a) override { fprintf(f, "\tmovem.l\td0-d7/a0-a6,$%X\n", a); }
	void marker(uint32_t a, uint32_t v) override { fprintf(f, "\tmove.l\t#$%08X,$%X\n", v, a); }
	void end(int i) override { fprintf(f, "end%d:\n", i); }
};

struct ExpExc { int vec; int fmt; uint32_t pc_off; std::string label; };   // pc: label address + pc_off
std::vector<std::string> gold_notes;                       // golden exceptions per case
std::vector<std::vector<ExpExc> > gold_exc;                // exceptions the case must take (refused opmodes, BSUN, FPU exceptions)
std::vector<uint32_t> mask_addrs;                          // frame images: byte whose low 6 bits (EA field of the CCR long) are not compared
std::vector<uint32_t> ia_must, ia_mustnot;                 // instruction addresses that must / must not be written to CIR $18
std::map<std::string, uint32_t> cur_syms;
unsigned gold_cond_reqs = 0;                               // conditionals the bridge sends to the ARM (KIND 3)

struct GoldSink : Sink {
	int ci = 0, nc = 0;
	void begin(int i, const Case &) override { ci = i; nc = 0; gold_reset(); }
	void setreg(int r, uint32_t v) override { gold_setreg(r, v); }
	uint32_t sym(const char *fmtstr, int a, int b)
	{
		char lb[48];
		snprintf(lb, sizeof lb, fmtstr, a, b);
		auto it = cur_syms.find(lb);
		return it == cur_syms.end() ? 0 : it->second;
	}
	void fpu(const Insn &n, int k, const Case &c) override
	{
		char lb[32];
		snprintf(lb, sizeof lb, "L%d_%d", ci, k);
		uint32_t addr = sym("L%d_%d", ci, k);
		if (n.bset27) { gold_ram()[W(ci) + n.bset_addr] |= 0x08; return; }
		std::vector<uint16_t> ext = n.ext;
		if (n.pcrel) ext.push_back((uint16_t)(sym("D%d_%d", ci, k) - (addr + 2 * (uint32_t)(insn_words(n) - 1))));   // PC = address of the extension word
		const uint16_t *ep = ext.empty() ? nullptr : ext.data();
		bool soft = n.soft || c.softall;
		uint32_t words = (uint32_t)insn_words(n);
		unsigned cls = n.cmd >> 13;
		bool fpclass = !n.frame && !n.scc && (cls == 0 || cls == 2 || cls == 3) && !((cls == 2) && (((n.cmd >> 10) & 7) == 7) && false);
		bool enabled = (gold_fpcr() & 0x7F00) != 0;
		if (n.frame) {
			int fl = n.frame == 1 ? gold_fsave(addr, n.op, ep, (int)ext.size()) : gold_frestore(addr, n.op, ep, (int)ext.size());
			if (n.frame == 1) {                              // image position -> the CCR long's EA field is not compared
				uint32_t base = gold_getreg(8 + (n.op & 7)), start = base;
				int mode = (n.op >> 3) & 7;
				if (mode == 5) start = base + (uint32_t)(int16_t)ext[0];
				mask_addrs.push_back(start + 5);
			}
			if (fl & GOLD_FMT) gold_exc[(size_t)ci].push_back({14, 0, 0, lb});
			else if (fl) gold_notes[(size_t)ci] += " golden flags " + std::to_string(fl) + " at " + lb;
			return;
		}
		if (n.scc) {
			count_cond(n.cmd & 0x3F);
			int fl = gold_scc(addr, n.op, n.cmd, ep, (int)ext.size());
			handle_cond_exc(fl, lb, c);
			return;
		}
		int fl = gold_exec(addr, n.op, n.cmd, ep, (int)ext.size());
		if (fl & GOLD_UNIMPL) {
			if (soft) gold_exc[(size_t)ci].push_back({gold_last_vector(), 0, 0, lb});
			else gold_notes[(size_t)ci] += " golden flags " + std::to_string(fl) + " at " + lb;
		} else if (fl & GOLD_EXC) {
			int st = gold_status();
			int vec = gold_last_vector();
			if (!soft) gold_notes[(size_t)ci] += " golden exception at " + std::string(lb) + " (not stepped over)";
			gold_exc[(size_t)ci].push_back({vec, (st & 2) ? 9 : 0, (st & 2) ? 2 * words : 0, lb});
			if (c.hmode == 0 && vec >= 48 && vec <= 54) gold_epilogue();
		} else if (fl) {
			gold_notes[(size_t)ci] += " golden flags " + std::to_string(fl) + " at " + lb;
		} else if (fpclass && cls != 7) {
			(enabled ? ia_must : ia_mustnot).push_back(addr);
		}
	}
	void reset() override { gold_reset(); }
	void count_cond(int p)
	{
		// the bridge asks the ARM for a conditional when the FPU may be null (first conditional after a reset),
		// or for an IEEE-nonaware predicate with NAN set; a pending exception is reported by the bridge itself
		bool sent = !gold_pending() && (gold_fpu_state() == 0 || ((p & 0x10) && (gold_fpsr() & 0x01000000u)));
		if (sent) gold_cond_reqs++;
	}
	void handle_cond_exc(int fl, const char *lb, const Case &c)
	{
		if (fl & GOLD_EXC) {
			int vec = gold_last_vector();
			gold_exc[(size_t)ci].push_back({vec, 0, 0, lb});
			if (vec == 48 && (gold_fpcr() & 0x8000)) ia_must.push_back(sym("L%d_c%d", ci, nc - 1));   // BSUN: $5C30 asks for the PC
			if (c.hmode == 0 && vec >= 48 && vec <= 54) gold_epilogue();
		}
	}
	// condition: -2 = an exception was reported instead (the instruction is stepped over)
	int cond(int p, const Case &c)
	{
		char lb[32];
		snprintf(lb, sizeof lb, "L%d_c%d", ci, nc++);
		count_cond(p);
		int r = gold_cond(p);
		if (getenv("M2_SEQ")) fprintf(stderr, "golden cond %s pred %02x -> %d\n", lb, p, r);
		if (r == -2) handle_cond_exc(GOLD_EXC, lb, c);
		return r;
	}
	const Case *cc = nullptr;
	void scc(int p, uint32_t a5, bool) override
	{
		int r = cond(p, *cc);
		if (r >= 0) gold_ram()[a5] = r ? 0xFF : 0x00;
		gold_setreg(13, a5 + 1);
	}
	void mbcc(int p, uint32_t a) override
	{
		int r = cond(p, *cc);
		int tf = r > 0 ? 1 : 0;
		gold_setreg(7, (uint32_t)tf);
		gold_ram()[a] = (uint8_t)tf;
	}
	void mdbcc(int p, uint32_t a) override
	{
		int r = cond(p, *cc);
		int tf = r != 0 ? 1 : 0;                          // exception: stepped over, same registers as "true"
		gold_setreg(6, tf ? 5u : 4u);
		gold_setreg(7, tf ? 0u : 1u);
		gold_ram()[a] = tf ? 0 : 1;
		gold_ram()[a + 1] = tf ? 5 : 4;
	}
	void mtrap(int p, uint32_t a) override
	{
		int r = cond(p, *cc);
		gold_ram()[a] = r > 0 ? 1 : 0;
	}
	void dumpregs(uint32_t a) override
	{
		for (int r = 0; r < 15; r++) {
			uint32_t v = gold_getreg(r);
			for (int b = 0; b < 4; b++) gold_ram()[a + 4 * (uint32_t)r + (uint32_t)b] = (uint8_t)(v >> (24 - 8 * b));
		}
	}
	void marker(uint32_t a, uint32_t v) override
	{
		for (int b = 0; b < 4; b++) gold_ram()[a + (uint32_t)b] = (uint8_t)(v >> (24 - 8 * b));
	}
	void end(int) override {}
};

}  // namespace

// ------------------------------------------------------------------ public API
void m2_build()
{
	cases.clear();
	chunk_start.clear();
	li_of.clear();
	g_isolate = g_force_new = false;
	if (!getenv("M2_SEQ")) {
		g_fmove_in();
		g_fmove_out();
		g_fmovecr();
		g_ctrl();
		g_fmovem();
		g_arith();
		g_cond();
		g_cctable();
		g_fscc_ea();
		g_reset();
		g_m4();
	}
	g_random(2200);
	if (getenv("M2_ONLY"))
		for (auto &c : cases) c.skip = (c.group + " / " + c.name).find(getenv("M2_ONLY")) == std::string::npos;
	gold_notes.assign(cases.size(), "");
	gold_exc.assign(cases.size(), {});
}
bool m2_skip(int i) { return cases[(size_t)i].skip; }
int m2_ncases() { return (int)cases.size(); }
int m2_nchunks() { return (int)chunk_start.size(); }
int m2_chunk_first(int c) { return chunk_start[(size_t)c]; }
int m2_chunk_end(int c) { return (size_t)c + 1 < chunk_start.size() ? chunk_start[(size_t)c + 1] : (int)cases.size(); }
const char *m2_group(int i) { return cases[(size_t)i].group.c_str(); }
const char *m2_name(int i) { return cases[(size_t)i].name.c_str(); }
bool m2_expects_imm(int i) { return cases[(size_t)i].imm_issue; }
uint32_t m2_dump_addr(int i) { return DU(i); }
uint32_t m2_marker(int i) { return 0xC0DE0000u + (uint32_t)i; }

void m2_emit_asm(FILE *f, int chunk)
{
	AsmSink s(f);
	for (int i = m2_chunk_first(chunk); i < m2_chunk_end(chunk); i++) if (!cases[(size_t)i].skip) walk(i, s);
}

void m2_load_ram(uint8_t *ram, int chunk)
{
	for (int i = m2_chunk_first(chunk); i < m2_chunk_end(chunk); i++)
		memcpy(ram + W(i), cases[(size_t)i].mem.data(), CASE_STRIDE);
}

// Hatari keeps FSAVE data (fsave_data) across instructions, so the golden must see exactly the history the device
// sees: the device (ARM service) starts fresh for every program image.  The golden of each image therefore runs in a
// fresh process (fork) and sends its results back through a pipe.
static void wr(FILE *f, const void *p, size_t n) { if (fwrite(p, 1, n, f) != n) _exit(3); }
static void wr32(FILE *f, uint32_t v) { wr(f, &v, 4); }
static void wrs(FILE *f, const std::string &s) { wr32(f, (uint32_t)s.size()); wr(f, s.data(), s.size()); }
static bool rd(FILE *f, void *p, size_t n) { return fread(p, 1, n, f) == n; }
static uint32_t rd32(FILE *f) { uint32_t v = 0; if (!rd(f, &v, 4)) v = 0; return v; }
static std::string rds(FILE *f) { uint32_t n = rd32(f); std::string s(n, ' '); if (n && !rd(f, &s[0], n)) s.clear(); return s; }

static void golden_child(const std::map<std::string, uint32_t> &syms, int chunk, const std::vector<uint8_t> *rom, FILE *out)
{
	gold_init();
	memset(gold_ram(), 0, GOLD_RAM_SIZE);
	m2_load_ram(gold_ram(), chunk);
	if (rom) memcpy(gold_ram() + 0xE00000, rom->data(), std::min<size_t>(rom->size(), 0x80000));   // (d16,PC) data pools live in the program
	cur_syms = syms;
	gold_cond_reqs = 0;
	mask_addrs.clear();
	ia_must.clear();
	ia_mustnot.clear();
	GoldSink s;
	int lo = m2_chunk_first(chunk), hi = m2_chunk_end(chunk);
	for (int i = lo; i < hi; i++) {
		gold_notes[(size_t)i].clear();
		gold_exc[(size_t)i].clear();
		if (cases[(size_t)i].skip) continue;
		s.cc = &cases[(size_t)i];
		walk(i, s);
	}
	wr32(out, gold_cond_reqs);
	wr32(out, (uint32_t)mask_addrs.size());
	for (uint32_t v : mask_addrs) wr32(out, v);
	wr32(out, (uint32_t)ia_must.size());
	for (uint32_t v : ia_must) wr32(out, v);
	wr32(out, (uint32_t)ia_mustnot.size());
	for (uint32_t v : ia_mustnot) wr32(out, v);
	for (int i = lo; i < hi; i++) {
		wrs(out, gold_notes[(size_t)i]);
		wr32(out, (uint32_t)gold_exc[(size_t)i].size());
		for (auto &x : gold_exc[(size_t)i]) { wr32(out, (uint32_t)x.vec); wr32(out, (uint32_t)x.fmt); wr32(out, x.pc_off); wrs(out, x.label); }
	}
	// the data windows and dump areas are all the compare needs
	wr(out, gold_ram() + WBASE, (size_t)CHUNK * CASE_STRIDE);
	wr(out, gold_ram() + DBASE, (size_t)CHUNK * DUMP_STRIDE);
}

void m2_run_golden(const std::map<std::string, uint32_t> &syms, int chunk, const std::vector<uint8_t> *rom)
{
	int fd[2];
	if (pipe(fd) != 0) { perror("pipe"); exit(2); }
	fflush(nullptr);
	pid_t pid = fork();
	if (pid < 0) { perror("fork"); exit(2); }
	if (pid == 0) {
		close(fd[0]);
		FILE *out = fdopen(fd[1], "wb");
		golden_child(syms, chunk, rom, out);
		fflush(out);
		_exit(0);
	}
	close(fd[1]);
	FILE *in = fdopen(fd[0], "rb");
	cur_syms = syms;
	int lo = m2_chunk_first(chunk), hi = m2_chunk_end(chunk);
	gold_cond_reqs = rd32(in);
	mask_addrs.assign(rd32(in), 0);
	for (auto &v : mask_addrs) v = rd32(in);
	ia_must.assign(rd32(in), 0);
	for (auto &v : ia_must) v = rd32(in);
	ia_mustnot.assign(rd32(in), 0);
	for (auto &v : ia_mustnot) v = rd32(in);
	for (int i = lo; i < hi; i++) {
		gold_notes[(size_t)i] = rds(in);
		uint32_t n = rd32(in);
		gold_exc[(size_t)i].clear();
		for (uint32_t k = 0; k < n; k++) {
			ExpExc x;
			x.vec = (int)rd32(in); x.fmt = (int)rd32(in); x.pc_off = rd32(in); x.label = rds(in);
			gold_exc[(size_t)i].push_back(x);
		}
	}
	golden_img.assign(GOLD_RAM_SIZE, 0);
	if (!rd(in, golden_img.data() + WBASE, (size_t)CHUNK * CASE_STRIDE)) fprintf(stderr, "golden window read failed\n");
	if (!rd(in, golden_img.data() + DBASE, (size_t)CHUNK * DUMP_STRIDE)) fprintf(stderr, "golden dump read failed\n");
	fclose(in);
	int st = 0;
	waitpid(pid, &st, 0);
	if (!WIFEXITED(st) || WEXITSTATUS(st) != 0) fprintf(stderr, "golden child failed (status %d)\n", st);
}
unsigned m2_golden_cond_requests() { return gold_cond_reqs; }
void m2_ia_sets(std::vector<uint32_t> &must, std::vector<uint32_t> &mustnot) { must = ia_must; mustnot = ia_mustnot; }

static std::string hexs(const uint8_t *p, int n)
{
	std::string s;
	char b[4];
	for (int i = 0; i < n; i++) { snprintf(b, sizeof b, "%02x", p[i]); s += b; }
	return s;
}

M2Result m2_compare(const uint8_t *ram, int i, const std::vector<M2Exc> &got)
{
	M2Result r;
	uint32_t d = DU(i), w = W(i);
	const uint8_t *g = golden_img.data();
	uint32_t mk = ((uint32_t)ram[d + 0xFC] << 24) | ((uint32_t)ram[d + 0xFD] << 16) | ((uint32_t)ram[d + 0xFE] << 8) | ram[d + 0xFF];
	r.done = mk == m2_marker(i);
	char b[160];
	uint32_t fpsr = ((uint32_t)g[d + 100] << 24) | ((uint32_t)g[d + 101] << 16) | ((uint32_t)g[d + 102] << 8) | g[d + 103];
	uint32_t fpcr = ((uint32_t)g[d + 96] << 24) | ((uint32_t)g[d + 97] << 16) | ((uint32_t)g[d + 98] << 8) | g[d + 99];
	snprintf(b, sizeof b, "golden: fpcr=%08x fpsr=%08x", fpcr, fpsr);
	r.summary = b + gold_notes[(size_t)i];
	auto add = [&](const std::string &what, const std::string &e, const std::string &gt) {
		if (r.diffs.size() < 6) r.diffs.push_back({what, e, gt});
	};
	bool bad = false;
	for (int off = 0; off < 96; off += 12)
		if (memcmp(g + d + off, ram + d + off, 12)) { bad = true; add("FP" + std::to_string(off / 12), hexs(g + d + off, 12), hexs(ram + d + off, 12)); }
	if (memcmp(g + d + 96, ram + d + 96, 8)) { bad = true; add("FPCR/FPSR", hexs(g + d + 96, 8), hexs(ram + d + 96, 8)); }
	static const char *rn[15] = {"D0", "D1", "D2", "D3", "D4", "D5", "D6", "D7", "A0", "A1", "A2", "A3", "A4", "A5", "A6"};
	for (int k = 0; k < 15; k++)
		if (memcmp(g + d + 108 + 4 * k, ram + d + 108 + 4 * k, 4)) { bad = true; add(rn[k], hexs(g + d + 108 + 4 * k, 4), hexs(ram + d + 108 + 4 * k, 4)); }
	// the data window; the EA field (low 6 bits) of the opword half of a frame's CCR long is not compared: the
	// engine's opword carries a synthetic EA, Hatari's the real one
	for (int off = 0; off < (int)CASE_STRIDE; off += 4) {
		bool diff = false;
		for (int k = 0; k < 4; k++) {
			uint8_t mskv = 0xFF;
			for (uint32_t m : mask_addrs) if (m == w + (uint32_t)off + (uint32_t)k) mskv = 0xC0;
			if ((g[w + (uint32_t)off + (uint32_t)k] & mskv) != (ram[w + (uint32_t)off + (uint32_t)k] & mskv)) diff = true;
		}
		if (diff) {
			bad = true;
			char nb[32];
			snprintf(nb, sizeof nb, "mem[win+0x%03X]", off);
			add(nb, hexs(g + w + off, 4), hexs(ram + w + off, 4));
		}
	}
	// exceptions: vector, frame format and stacked PC at exactly the golden's instructions
	const std::vector<ExpExc> &ex = gold_exc[(size_t)i];
	bool exok = ex.size() == got.size();
	for (size_t k = 0; exok && k < ex.size(); k++) {
		auto it = cur_syms.find(ex[k].label);
		if ((int)got[k].vec != ex[k].vec || (int)got[k].fmt != ex[k].fmt || it == cur_syms.end() || got[k].pc != it->second + ex[k].pc_off) exok = false;
	}
	if (!exok) {
		bad = true;
		std::string e, gt;
		for (auto &x : ex) { auto it = cur_syms.find(x.label); e += " vec" + std::to_string(x.vec) + "/fmt" + std::to_string(x.fmt) + "@" + (it == cur_syms.end() ? x.label : std::to_string(it->second + x.pc_off)); }
		for (auto &x : got) gt += " vec" + std::to_string(x.vec) + "/fmt" + std::to_string(x.fmt) + "@" + std::to_string(x.pc);
		add("exceptions", e.empty() ? "none" : e, gt.empty() ? "none" : gt);
	}
	r.nexc = (int)ex.size();
	r.fpiar_exp = ((uint32_t)g[d + 104] << 24) | ((uint32_t)g[d + 105] << 16) | ((uint32_t)g[d + 106] << 8) | g[d + 107];
	r.fpiar_got = ((uint32_t)ram[d + 104] << 24) | ((uint32_t)ram[d + 105] << 16) | ((uint32_t)ram[d + 106] << 8) | ram[d + 107];
	r.fpiar_diff = r.fpiar_exp != r.fpiar_got;
	r.ok = r.done && !bad;
	return r;
}
