// m3.cpp - milestone 3 arithmetic sweeps (see m3.h).
//
// Item = [load operands] [clear FPSR] [instruction under test] [store FP result(s)] [store FPSR].
// Results are stored through (a1)+; the whole stream is compared with the golden's.
// The expected values are the golden's (Hatari fpp.c), never the RTL's or the service's.
#include "m3.h"

#include <algorithm>
#include <cstring>

#include "golden.h"

namespace {

typedef std::vector<uint8_t> Bytes;
enum { F_L, F_S, F_X, F_P, F_W, F_D, F_B };

const uint32_t DATA3 = 0x100000, RES3 = 0x200000, DONE3 = 0x1FFFF0;
const uint32_t OFF_CLS = 0x0000, OFF_DST = 0x0100, OFF_MEM = 0x0200, OFF_INIT = 0x1000, OFF_NULL = 0x1F00;
const unsigned CHUNK_ITEMS = 5000;

struct SOp {
	int kind = 0;                       // 0 FPU instruction, 1 set register, 2 lea n(a1),a1
	uint16_t op = 0, cmd = 0;
	std::vector<uint16_t> ext;
	bool soft = false;                  // refused opmode: the exception is stepped over
	int reg = 0;
	uint32_t val = 0;
};

struct Item {
	std::vector<SOp> in;
	unsigned res_len = 16;
	std::string desc;
};

struct Grp {
	std::string name;
	uint32_t fpcr = 0;
	std::vector<Item> items;
};

std::vector<Grp> groups;
std::vector<std::vector<int> > chunk_groups;                // group indices per chunk
std::vector<std::vector<uint8_t> > ram_init;                // unused
std::vector<uint8_t> golden_img;
std::map<std::string, uint32_t> cur_syms;
struct Exp { int vec; std::string label; };
std::vector<Exp> gold_exps;

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

struct XV { const char *n; uint16_t se; uint64_t m; };
const XV CLS[14] = {{"+0", 0x0000, 0}, {"-0", 0x8000, 0}, {"+inf", 0x7FFF, 0}, {"-inf", 0xFFFF, 0},
                    {"qNaN", 0x7FFF, 0xC000000000000000ull}, {"sNaN", 0x7FFF, 0x8000000000000001ull},
                    {"denorm", 0x0000, 0x0000000000001000ull}, {"-denorm", 0x8000, 0x0000000100000000ull},
                    {"unnormal", 0x3FFF, 0x4000000000000000ull}, {"tiny", 0x0001, 0x8000000000000000ull},
                    {"huge", 0x7FFE, 0xFFFFFFFFFFFFFFFFull}, {"2.5", 0x4000, 0xA000000000000000ull},
                    {"-0.1", 0xBFFB, 0xCCCCCCCCCCCCCCCDull}, {"1.0", 0x3FFF, 0x8000000000000000ull}};
const XV DCLS[6] = {{"3.0", 0x4000, 0xC000000000000000ull}, {"-0.7", 0xBFFE, 0xB333333333333333ull}, {"+0", 0x0000, 0},
                    {"+inf", 0x7FFF, 0}, {"qNaN", 0x7FFF, 0xC000000000000000ull}, {"sNaN", 0x7FFF, 0x8000000000000001ull}};

const uint64_t VB[] = {0x00, 0x01, 0x7F, 0x80, 0xFF, 0x55};
const uint64_t VW[] = {0x0000, 0x0001, 0x7FFF, 0x8000, 0xFFFF, 0x1234};
const uint64_t VL[] = {0, 1, 0x7FFFFFFFull, 0x80000000ull, 0xFFFFFFFFull, 0x12345678ull, 0x00FFFFFFull};
const uint64_t VS[] = {0, 0x80000000ull, 0x3F800000ull, 0xC0200000ull, 0x7F800000ull, 0xFF800000ull, 0x7FC00000ull,
                       0x7FA00000ull, 0x00000001ull, 0x00800000ull, 0x7F7FFFFFull, 0x3DCCCCCDull, 0x4B000001ull};
const uint64_t VD[] = {0, 0x8000000000000000ull, 0x3FF0000000000000ull, 0xC004000000000000ull, 0x7FF0000000000000ull,
                       0x7FF8000000000000ull, 0x7FF4000000000000ull, 0x0000000000000001ull, 0x7E37E43C8800759Cull,
                       0x01A56E1FC2F8F359ull, 0x3FB999999999999Aull, 0xFFF0000000000000ull};
const uint32_t VP[][3] = {{0, 0, 0}, {1, 0, 0}, {0x80100001u, 0x50000000u, 0}, {0x41000009u, 0x99999999u, 0x99999999u},
                          {0x7FFF0000u, 0, 0}, {0x7FFF0000u, 0, 1}, {0x80000000u, 0, 0},
                          {0x40050001u, 0x23456789u, 0x01234567u}, {0x01000001u, 0, 0}};
int nvals(int fmt)
{
	static const int n[7] = {7, 13, 14, 9, 6, 12, 6};
	return n[fmt];
}
const char *fname(int fmt)
{
	static const char *n[7] = {"L", "S", "X", "P", "W", "D", "B"};
	return n[fmt];
}
Bytes data_of(int fmt, int k)
{
	switch (fmt) {
	case F_B: return be(VB[k % 6], 1);
	case F_W: return be(VW[k % 6], 2);
	case F_L: return be(VL[k % 7], 4);
	case F_S: return be(VS[k % 13], 4);
	case F_D: return be(VD[k % 12], 8);
	case F_X: return k < 14 ? xval(CLS[k].se, CLS[k].m) : xval(0x3FFF, 0x8000000000000000ull);
	default: return pval(VP[k % 9][0], VP[k % 9][1], VP[k % 9][2]);
	}
}

struct Op { const char *n; int code; };
const Op DOC[] = {{"FMOVE", 0x00}, {"FINT", 0x01}, {"FSINH", 0x02}, {"FINTRZ", 0x03}, {"FSQRT", 0x04}, {"FLOGNP1", 0x06},
                  {"FETOXM1", 0x08}, {"FTANH", 0x09}, {"FATAN", 0x0A}, {"FASIN", 0x0C}, {"FATANH", 0x0D}, {"FSIN", 0x0E},
                  {"FTAN", 0x0F}, {"FETOX", 0x10}, {"FTWOTOX", 0x11}, {"FTENTOX", 0x12}, {"FLOGN", 0x14}, {"FLOG10", 0x15},
                  {"FLOG2", 0x16}, {"FABS", 0x18}, {"FCOSH", 0x19}, {"FNEG", 0x1A}, {"FACOS", 0x1C}, {"FCOS", 0x1D},
                  {"FGETEXP", 0x1E}, {"FGETMAN", 0x1F}, {"FDIV", 0x20}, {"FMOD", 0x21}, {"FADD", 0x22}, {"FMUL", 0x23},
                  {"FSGLDIV", 0x24}, {"FREM", 0x25}, {"FSCALE", 0x26}, {"FSGLMUL", 0x27}, {"FSUB", 0x28}, {"FCMP", 0x38},
                  {"FTST", 0x3A}};
const int NDOC = (int)(sizeof(DOC) / sizeof(DOC[0]));
bool hatari_refused(int op)      // fpp.c fault_if_nonexisting_opmode (68881/68882, fpu_no_unimplemented = false)
{
	static const int l[] = {0x42, 0x43, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56,
	                        0x57, 0x59, 0x5b, 0x5d, 0x5f, 0x61, 0x65, 0x69, 0x6a, 0x6b, 0x6d};
	if (op >= 0x6E) return true;
	for (int x : l) if (x == op) return true;
	return false;
}
bool is_doc(int op)
{
	if (op >= 0x30 && op <= 0x37) return true;           // FSINCOS
	for (const Op &o : DOC) if (o.code == op) return true;
	return false;
}

// ---- instruction builders
SOp fpu(uint16_t op, uint16_t cmd, std::vector<uint16_t> ext = {}, bool soft = false)
{
	SOp s;
	s.op = op; s.cmd = cmd; s.ext = ext; s.soft = soft;
	return s;
}
SOp setreg(int r, uint32_t v) { SOp s; s.kind = 1; s.reg = r; s.val = v; return s; }
SOp storereg(int r) { SOp s; s.kind = 3; s.reg = r; return s; }
SOp lea_a1(int n) { SOp s; s.kind = 2; s.val = (uint32_t)n; return s; }

SOp load_x(int reg, uint32_t off) { return fpu(0xF22A, (uint16_t)(0x4000 | (F_X << 10) | (reg << 7)), {(uint16_t)off}); }     // fmove.x off(a2),FPreg
SOp store_x(int reg) { return fpu(0xF219, (uint16_t)(0x6000 | (F_X << 10) | (reg << 7))); }                                   // fmove.x FPreg,(a1)+
SOp store_fpsr() { return fpu(0xF219, 0xA800); }                                                                           // fmove.l FPSR,(a1)+
SOp clear_fpsr() { return fpu(0xF207, 0x8800); }                                                                           // fmove.l d7,FPSR
SOp arith_rr(int src, int dst, int op, bool soft = false) { return fpu(0xF200, (uint16_t)((src << 10) | (dst << 7) | op), {}, soft); }
SOp arith_mem(int fmt, int dst, int op, uint32_t off, bool soft = false)
{
	return fpu(0xF22A, (uint16_t)(0x4000 | (fmt << 10) | (dst << 7) | op), {(uint16_t)off}, soft);
}

std::string hx(uint32_t v) { char b[16]; snprintf(b, sizeof b, "%02X", v); return b; }

std::vector<uint8_t> data_img;                               // initial DATA3 area (0x2000 bytes)
std::map<std::string, uint32_t> mem_slot;                    // "fmt,k" -> offset
uint32_t next_slot = OFF_MEM;
uint32_t slot_for(int fmt, int k)
{
	std::string key = std::string(fname(fmt)) + "," + std::to_string(k);
	auto it = mem_slot.find(key);
	if (it != mem_slot.end()) return it->second;
	uint32_t off = next_slot;
	next_slot += 16;
	Bytes b = data_of(fmt, k);
	std::copy(b.begin(), b.end(), data_img.begin() + off);
	mem_slot[key] = off;
	return off;
}

uint32_t fpcr_of(int rm, int prec) { return (uint32_t)((rm << 4) | (prec << 6)); }
std::string fpcr_name(uint32_t f)
{
	static const char *rmn[4] = {"RN", "RZ", "RM", "RP"};
	static const char *pn[4] = {"ext", "sgl", "dbl", "?"};
	return std::string(rmn[(f >> 4) & 3]) + "/" + pn[(f >> 6) & 3];
}

Grp &new_group(const std::string &name, uint32_t fpcr)
{
	groups.emplace_back();
	groups.back().name = name;
	groups.back().fpcr = fpcr;
	return groups.back();
}

// reg-reg item: load src (once per src), dst, clear FPSR, op, store dst, store FPSR
Item rr_item(const char *opn, int op, int a, int b, bool first, bool soft)
{
	Item it;
	if (first) it.in.push_back(load_x(0, OFF_CLS + 16u * (uint32_t)a));
	it.in.push_back(load_x(1, OFF_DST + 16u * (uint32_t)b));
	it.in.push_back(clear_fpsr());
	it.in.push_back(arith_rr(0, 1, op, soft));
	it.in.push_back(store_x(1));
	it.in.push_back(store_fpsr());
	it.desc = std::string(opn) + " src=" + CLS[a].n + " dst=" + DCLS[b].n;
	return it;
}

void k1()
{
	for (int rm = 0; rm < 4; rm++)
		for (int prec = 0; prec < 3; prec++) {
			uint32_t fpcr = fpcr_of(rm, prec);
			for (const Op &o : DOC) {
				Grp &g = new_group(std::string("reg-reg ") + o.n + " " + fpcr_name(fpcr), fpcr);
				for (int a = 0; a < 14; a++) {
					int nb = (fpcr == 0) ? 6 : 2;
					for (int b = 0; b < nb; b++) g.items.push_back(rr_item(o.n, o.code, a, b, b == 0, false));
				}
			}
		}
}

void k2()
{
	static const int fmts[7] = {F_B, F_W, F_L, F_S, F_D, F_X, F_P};
	static const uint32_t fp3[3] = {0x00, 0x10 | 0x40, 0x30 | 0x80};
	for (uint32_t fpcr : fp3)
		for (const Op &o : DOC)
			for (int fmt : fmts) {
				Grp &g = new_group(std::string("mem ") + o.n + "." + fname(fmt) + " " + fpcr_name(fpcr), fpcr);
				for (int k = 0; k < nvals(fmt); k++) {
					Item it;
					uint32_t off = slot_for(fmt, k);
					it.in.push_back(load_x(1, OFF_DST + 16u * (uint32_t)(k & 1)));
					it.in.push_back(clear_fpsr());
					it.in.push_back(arith_mem(fmt, 1, o.code, off));
					it.in.push_back(store_x(1));
					it.in.push_back(store_fpsr());
					it.desc = std::string(o.n) + "." + fname(fmt) + " value#" + std::to_string(k) + " dst=" + DCLS[k & 1].n;
					g.items.push_back(it);
				}
			}
}

void k3()           // undocumented opmodes: reg-reg and memory
{
	for (int op = 0; op < 0x80; op++) {
		if (is_doc(op)) continue;
		bool refused = hatari_refused(op);
		Grp &g = new_group(std::string("undocumented opmode $") + hx((uint32_t)op) + (op >= 0x78 ? " (Hatari: vector 4)" : refused ? " (Hatari: F-line)" : " (alias)"), 0);
		static const int srcs[4] = {11, 12, 0, 4};
		for (int a : srcs) {
			Item it = rr_item(("op$" + hx((uint32_t)op)).c_str(), op, a, 0, true, refused);
			g.items.push_back(it);
		}
		for (int k : {11, 12}) {
			Item it;
			uint32_t off = slot_for(F_X, k);
			it.in.push_back(load_x(1, OFF_DST));
			it.in.push_back(clear_fpsr());
			it.in.push_back(arith_mem(F_X, 1, op, off, refused));
			it.in.push_back(store_x(1));
			it.in.push_back(store_fpsr());
			it.desc = "op$" + hx((uint32_t)op) + ".X " + CLS[k].n;
			g.items.push_back(it);
		}
		// a memory-source form with a post-increment register must not touch it when refused
		{
			Item it;
			uint32_t off = slot_for(F_X, 11);
			it.in.push_back(setreg(8 + 3, DATA3 + off));
			it.in.push_back(load_x(1, OFF_DST));
			it.in.push_back(clear_fpsr());
			it.in.push_back(fpu(0xF21B, (uint16_t)(0x4000 | (F_X << 10) | (1 << 7) | op), {}, refused));   // op.X (a3)+,fp1
			it.in.push_back(store_x(1));
			it.in.push_back(store_fpsr());
			it.in.push_back(storereg(8 + 3));                                                                 // a3 after the instruction
			it.res_len = 20;
			it.desc = "op$" + hx((uint32_t)op) + ".X (a3)+";
			g.items.push_back(it);
		}
	}
}

void k4()           // FSINCOS: all 8 FPc, FPc == FPs and FPc == FPd included
{
	static const int srcs[6] = {11, 12, 0, 2, 4, 9};
	for (uint32_t fpcr : {0x00u, 0x10u | 0x40u, 0x20u | 0x80u}) {
		Grp &g = new_group("FSINCOS FPs=FP0 FPd=FP1 " + fpcr_name(fpcr), fpcr);
		for (int c = 0; c < 8; c++)
			for (int a : srcs) {
				Item it;
				it.res_len = 100;
				it.in.push_back(setreg(8 + 6, DATA3 + OFF_INIT));
				it.in.push_back(fpu(0xF216, 0xD0FF));                                            // fmovem.x (a6),fp0-fp7
				it.in.push_back(load_x(0, OFF_CLS + 16u * (uint32_t)a));
				it.in.push_back(clear_fpsr());
				it.in.push_back(arith_rr(0, 1, 0x30 | c));
				it.in.push_back(fpu(0xF211, 0xF0FF));                                            // fmovem.x fp0-fp7,(a1)
				it.in.push_back(fpu(0xF229, 0xA800, {96}));                                      // fmove.l fpsr,96(a1)
				it.in.push_back(lea_a1(100));
				it.desc = std::string("FSINCOS src=") + CLS[a].n + " FPc=FP" + std::to_string(c);
				g.items.push_back(it);
			}
	}
	// FMOD / FREM quotient bytes with a wider set of operands
	static const int dsts[6] = {0, 1, 2, 3, 4, 5};
	for (const Op &o : DOC) {
		if (o.code != 0x21 && o.code != 0x25) continue;
		Grp &g = new_group(std::string("quotient ") + o.n + " all dst classes", 0);
		for (int a : {11, 12, 13, 9, 10, 6})
			for (int b : dsts) g.items.push_back(rr_item(o.n, o.code, a, b, b == 0, false));
	}
}

}  // namespace

void m3_build()
{
	groups.clear();
	data_img.assign(0x2000, 0);
	mem_slot.clear();
	next_slot = OFF_MEM;
	for (int a = 0; a < 14; a++) { Bytes b = xval(CLS[a].se, CLS[a].m); std::copy(b.begin(), b.end(), data_img.begin() + OFF_CLS + 16 * a); }
	for (int b = 0; b < 6; b++) { Bytes x = xval(DCLS[b].se, DCLS[b].m); std::copy(x.begin(), x.end(), data_img.begin() + OFF_DST + 16 * b); }
	{   // init image: eight ordinary values
		static const XV init[8] = {{"", 0x3FFF, 0x8000000000000000ull}, {"", 0xC000, 0xA000000000000000ull}, {"", 0x4000, 0xC90FDAA22168C235ull},
		                           {"", 0x0000, 0}, {"", 0x8000, 0}, {"", 0x7FFF, 0}, {"", 0x7FFF, 0xC000000000000000ull},
		                           {"", 0x3FFB, 0xCCCCCCCCCCCCCCCDull}};
		for (int r = 0; r < 8; r++) { Bytes x = xval(init[r].se, init[r].m); std::copy(x.begin(), x.end(), data_img.begin() + OFF_INIT + 12 * r); }
	}
	k1();
	k2();
	k3();
	k4();
	// chunks at group boundaries
	chunk_groups.clear();
	chunk_groups.emplace_back();
	unsigned n = 0;
	for (size_t g = 0; g < groups.size(); g++) {
		if (n >= CHUNK_ITEMS) { chunk_groups.emplace_back(); n = 0; }
		chunk_groups.back().push_back((int)g);
		n += (unsigned)groups[g].items.size();
	}
}
int m3_nchunks() { return (int)chunk_groups.size(); }
uint32_t m3_done_addr() { return DONE3; }

static void emit_op(FILE *f, const SOp &s, int &label)
{
	if (s.kind == 1) {
		if (s.reg < 8) fprintf(f, "\tmove.l\t#$%08X,d%d\n", s.val, s.reg);
		else fprintf(f, "\tmovea.l\t#$%08X,a%d\n", s.val, s.reg - 8);
	} else if (s.kind == 2)
		fprintf(f, "\tlea\t%d(a1),a1\n", (int)s.val);
	else if (s.kind == 3)
		fprintf(f, "\tmove.l\ta%d,(a1)+\n", s.reg - 8);
	else {
		if (s.soft) fprintf(f, "\tmove.l\t#%d,SKIP\n\tmove.l\t#1,SOFTEXC\n", 2 * (2 + (int)s.ext.size()));
		fprintf(f, "M3_%d:\tdc.w\t$%04X,$%04X", label++, s.op, s.cmd);
		for (uint16_t e : s.ext) fprintf(f, ",$%04X", e);
		fprintf(f, "\n");
		if (s.soft) fprintf(f, "\tclr.l\tSOFTEXC\n");
	}
}

void m3_emit_asm(FILE *f, int chunk)
{
	int label = 0;
	fprintf(f, "\tlea\tm3end(pc),a0\n\tmove.l\ta0,RESUME\n");
	fprintf(f, "\tmovea.l\t#$%08X,a6\n\tdc.w\t$F356\t\t; frestore (a6): reset\n", DATA3 + OFF_NULL);
	fprintf(f, "\tmovea.l\t#$%08X,a6\n\tdc.w\t$F216,$D0FF\t; fmovem.x (a6),fp0-fp7\n", DATA3 + OFF_INIT);
	fprintf(f, "\tmovea.l\t#$%08X,a1\n\tmovea.l\t#$%08X,a2\n\tmoveq\t#0,d7\n", RES3, DATA3);
	for (int g : chunk_groups[(size_t)chunk]) {
		const Grp &gr = groups[(size_t)g];
		fprintf(f, "; ---- %s\n\tdc.w\t$F23C,$9000,$%04X,$%04X\t; fmove.l #fpcr,fpcr\n", gr.name.c_str(), gr.fpcr >> 16, gr.fpcr & 0xFFFF);
		for (const Item &it : gr.items)
			for (const SOp &s : it.in) emit_op(f, s, label);
	}
	fprintf(f, "m3end:\tmove.l\t#$%08X,$%X\n", 0xC0DE3000u + (unsigned)chunk, DONE3);
}

void m3_load_ram(uint8_t *ram, int)
{
	memcpy(ram + DATA3, data_img.data(), data_img.size());
}

void m3_run_golden(const std::map<std::string, uint32_t> &syms, int chunk)
{
	gold_init();
	memset(gold_ram(), 0, GOLD_RAM_SIZE);
	m3_load_ram(gold_ram(), chunk);
	cur_syms = syms;
	gold_exps.clear();
	int label = 0;
	gold_reset();
	gold_setreg(14, DATA3 + OFF_NULL);
	gold_reset();
	gold_setreg(14, DATA3 + OFF_INIT);
	gold_exec(0, 0xF216, 0xD0FF, nullptr, 0);
	gold_setreg(9, RES3);
	gold_setreg(10, DATA3);
	gold_setreg(7, 0);
	for (int g : chunk_groups[(size_t)chunk]) {
		const Grp &gr = groups[(size_t)g];
		uint16_t e[2] = {(uint16_t)(gr.fpcr >> 16), (uint16_t)gr.fpcr};
		gold_exec(0, 0xF23C, 0x9000, e, 2);
		for (const Item &it : gr.items)
			for (const SOp &s : it.in) {
				if (s.kind == 1) { gold_setreg(s.reg, s.val); continue; }
				if (s.kind == 2) { gold_setreg(9, gold_getreg(9) + s.val); continue; }
				if (s.kind == 3) {
					uint32_t a = gold_getreg(9), v = gold_getreg(s.reg);
					for (int b = 0; b < 4; b++) gold_ram()[a + (uint32_t)b] = (uint8_t)(v >> (24 - 8 * b));
					gold_setreg(9, a + 4);
					continue;
				}
				char lb[32];
				snprintf(lb, sizeof lb, "M3_%d", label++);
				int fl = gold_exec(0, s.op, s.cmd, s.ext.empty() ? nullptr : s.ext.data(), (int)s.ext.size());
				if ((fl & GOLD_UNIMPL) && s.soft) gold_exps.push_back({gold_last_vector(), lb});
			}
	}
	uint32_t v = 0xC0DE3000u + (uint32_t)chunk;
	for (int b = 0; b < 4; b++) gold_ram()[DONE3 + (uint32_t)b] = (uint8_t)(v >> (24 - 8 * b));
	golden_img.assign(gold_ram(), gold_ram() + GOLD_RAM_SIZE);
}

bool m3_done(const uint8_t *ram, int chunk)
{
	uint32_t v = ((uint32_t)ram[DONE3] << 24) | ((uint32_t)ram[DONE3 + 1] << 16) | ((uint32_t)ram[DONE3 + 2] << 8) | ram[DONE3 + 3];
	return v == 0xC0DE3000u + (uint32_t)chunk;
}

static std::string hexs(const uint8_t *p, int n)
{
	std::string s;
	char b[4];
	for (int i = 0; i < n; i++) { snprintf(b, sizeof b, "%02x", p[i]); s += b; }
	return s;
}

std::vector<M3Group> m3_compare(const uint8_t *ram, int chunk, const std::vector<M2Exc> &exc, std::string &exc_detail, bool &exc_ok, unsigned &n_soft)
{
	std::vector<M3Group> out;
	uint32_t pos = RES3;
	for (int g : chunk_groups[(size_t)chunk]) {
		const Grp &gr = groups[(size_t)g];
		M3Group mg;
		mg.name = gr.name;
		for (const Item &it : gr.items) {
			mg.items++;
			if (memcmp(golden_img.data() + pos, ram + pos, it.res_len)) {
				mg.bad++;
				if (mg.bad <= 3)
					mg.detail += "; " + it.desc + " expected " + hexs(golden_img.data() + pos, (int)it.res_len) + " got " + hexs(ram + pos, (int)it.res_len);
			}
			pos += it.res_len;
		}
		out.push_back(mg);
	}
	// every logged exception must be one of the golden's refused instructions, in order
	exc_ok = exc.size() == gold_exps.size();
	n_soft = (unsigned)gold_exps.size();
	for (size_t k = 0; exc_ok && k < gold_exps.size(); k++) {
		auto it = cur_syms.find(gold_exps[k].label);
		if ((int)exc[k].vec != gold_exps[k].vec || exc[k].fmt != 0 || it == cur_syms.end() || exc[k].pc != it->second) {
			exc_ok = false;
			exc_detail = "exception #" + std::to_string(k) + " expected vec " + std::to_string(gold_exps[k].vec) + " at " + std::to_string(it == cur_syms.end() ? 0 : it->second) +
			             " got vec " + std::to_string(exc[k].vec) + " fmt " + std::to_string(exc[k].fmt) + " at " + std::to_string(exc[k].pc);
		}
	}
	if (exc.size() != gold_exps.size()) exc_detail = "expected " + std::to_string(gold_exps.size()) + " exceptions got " + std::to_string(exc.size());
	return out;
}
