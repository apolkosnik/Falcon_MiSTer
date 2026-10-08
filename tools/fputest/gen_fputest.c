/*
 * gen_fputest.c - generate the instruction sequence of FPUTEST.TOS and its
 * expected results.
 *
 * Every FPU instruction written to the 68k program is also executed, in the
 * same order and with the same operand bytes, by the 68882 engine the ARM
 * service runs (tools/falcon_fpu/engine, Hatari's fpp.c).  The expected
 * results are therefore exactly what a working FPGA <-> DDR3 <-> ARM path
 * delivers; FPUTEST.TOS checks the transport and the bridge on hardware.  (The
 * engine and the bridge are verified against an independent golden in
 * tb/fpu.)
 *
 * The program runs with A1 = result buffer, A2 = operand table.  Each check
 * writes a 16-byte slot: FPSR (4 bytes) and up to 12 result bytes; unused
 * bytes stay at the $A5 fill.  Output (stdout): vasm source with
 *   gen_code   the sequence (ends with RTS)
 *   gen_vals   the operand table
 *   gen_expect the expected result buffer, GEN_SLOTS slots
 *   gen_names  one name per slot (for the failure report)
 */
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "falcon_fpu_engine.h"

#define MAXSLOTS 8192
static uint8_t expect[MAXSLOTS * 16];
static char names[MAXSLOTS][48];
static int nslots;
static uint8_t vals[30000];
static int nvals;
static int pc = 0x1000;                 /* a nominal instruction address for the engine */
static int stride = 1, keep = 1, nchk;  /* -s N: only every Nth check (a small build for simulation) */
static uint8_t dummy[16];

/* 68k source of the sequence; a skipped check still runs on the engine (every check
 * loads the state it uses, so the engine stays in step with the shorter program) */
static void emit(const char *fmt, ...)
{
	va_list ap;
	if (!keep) return;
	va_start(ap, fmt);
	vprintf(fmt, ap);
	va_end(ap);
}

static void fail(const char *m)
{
	fprintf(stderr, "gen_fputest: %s\n", m);
	exit(1);
}

/* run one instruction on the engine; out bytes returned */
static int run(uint16_t cmd, const uint8_t *in, int n, uint16_t aux, uint8_t *out)
{
	int len = 0;
	int fl = fpe_exec(cmd, in, n, aux, (uint32_t)pc, out, &len);
	pc += 4;
	if (fl & (FPE_UNIMPL | FPE_ERROR | FPE_EXCEPTION)) {
		fprintf(stderr, "cmd %04x flags %x\n", cmd, fl);
		fail("unexpected engine flags");
	}
	return len;
}

static int add_val(const uint8_t *b, int n)
{
	/* reuse an identical entry */
	for (int o = 0; o + n <= nvals; o += 2)
		if (!memcmp(vals + o, b, (size_t)n)) return o;
	int o = (nvals + 1) & ~1;
	if (o + n > (int)sizeof(vals) || o + n > 32766) fail("operand table full");
	memcpy(vals + o, b, (size_t)n);
	nvals = o + n;
	return o;
}

static void be32(uint8_t *b, uint32_t v) { b[0] = v >> 24; b[1] = v >> 16; b[2] = v >> 8; b[3] = v; }

static void xval(uint8_t *b, uint16_t se, uint64_t m)
{
	b[0] = se >> 8; b[1] = (uint8_t)se; b[2] = b[3] = 0;
	for (int i = 0; i < 8; i++) b[4 + i] = (uint8_t)(m >> (56 - 8 * i));
}

/* ---- emitted instructions (engine mirrored) ---- */

static void e_fpcr(uint32_t v)                  /* FMOVE.L #v,FPCR */
{
	uint8_t b[4], o[16];
	be32(b, v);
	emit("\tdc.w\t$F23C,$9000,$%04X,$%04X\n", v >> 16, v & 0xffff);
	run(0x9000, b, 4, 0, o);
}

static void e_fpsr(uint32_t v)                  /* FMOVE.L #v,FPSR */
{
	uint8_t b[4], o[16];
	be32(b, v);
	emit("\tdc.w\t$F23C,$8800,$%04X,$%04X\n", v >> 16, v & 0xffff);
	run(0x8800, b, 4, 0, o);
}

/* FMOVE.fmt / op.fmt d16(A2),FPn */
static void e_mem(int fmt, int reg, int opmode, const uint8_t *b, int n)
{
	uint8_t o[16];
	int off = add_val(b, n);
	uint16_t cmd = 0x4000 | fmt << 10 | reg << 7 | opmode;
	emit("\tdc.w\t$F22A,$%04X,$%04X\n", cmd, off);
	run(cmd, b, n, 0, o);
}

static void e_ldx(int reg, uint16_t se, uint64_t m)
{
	uint8_t b[12];
	xval(b, se, m);
	e_mem(2, reg, 0x00, b, 12);
}

static void e_rr(int src, int dst, int opmode)  /* op FPsrc,FPdst */
{
	uint8_t o[16];
	uint16_t cmd = src << 10 | dst << 7 | opmode;
	emit("\tdc.w\t$F200,$%04X\n", cmd);
	run(cmd, NULL, 0, 0, o);
}

static void e_cr(int reg, int rom)               /* FMOVECR #rom,FPn */
{
	uint8_t o[16];
	uint16_t cmd = 0x5C00 | reg << 7 | rom;
	emit("\tdc.w\t$F200,$%04X\n", cmd);
	run(cmd, NULL, 0, 0, o);
}

/* a new slot: FPSR at 0(A1), then the data written by the caller at 4(A1) */
static uint8_t *slot(const char *name)
{
	if (!keep) return dummy;
	if (nslots >= MAXSLOTS) fail("too many slots");
	uint8_t *s = expect + nslots * 16;
	memset(s, 0xA5, 16);
	snprintf(names[nslots], sizeof(names[0]), "%s", name);
	nslots++;
	return s;
}

static void e_stfpsr(uint8_t *s)                /* FMOVE.L FPSR,(A1) */
{
	uint8_t o[16];
	emit("\tdc.w\t$F211,$A800\n");
	if (run(0xA800, NULL, 0, 0, o) != 4) fail("FPSR store");
	memcpy(s, o, 4);
}

static void e_next(void)                        /* LEA 16(A1),A1 */
{
	emit("\tlea\t16(a1),a1\n");
}

/* FMOVE.fmt FPn,4(A1) (k: static k-factor, or Dn for fmt 7) */
static void e_store(uint8_t *s, int fmt, int reg, int k, uint32_t dn_val)
{
	uint8_t o[16];
	uint16_t cmd = 0x6000 | fmt << 10 | reg << 7 | (k & 0x7f);
	emit("\tdc.w\t$F229,$%04X,$0004\n", cmd);
	int n = run(cmd, NULL, 0, (uint16_t)dn_val, o);
	if (n > 12) fail("store too long");
	memcpy(s + 4, o, (size_t)n);
}

/* the result of an operation: FPSR, then FP0 as X */
static void result(const char *name, int reg)
{
	uint8_t *s = slot(name);
	e_stfpsr(s);
	e_store(s, 2, reg, 0, 0);
	e_next();
}

/* ---- operand values (extended) ---- */
struct xv { const char *n; uint16_t se; uint64_t m; };
static const struct xv V[] = {
	{"+0", 0x0000, 0},                       {"-0", 0x8000, 0},
	{"1", 0x3fff, 0x8000000000000000ull},    {"-1", 0xbfff, 0x8000000000000000ull},
	{"0.5", 0x3ffe, 0x8000000000000000ull},  {"pi", 0x4000, 0xc90fdaa22168c235ull},
	{"-2.5", 0xc000, 0xa000000000000000ull}, {"1e10", 0x4020, 0x9502f90000000000ull},
	{"1e-10", 0x3fdd, 0xdbe6fecebdedd5bfull},{"3.75", 0x4000, 0xf000000000000000ull},
	{"big", 0x7ffe, 0xffffffffffffffffull},  {"tiny", 0x0001, 0x8000000000000000ull},
	{"denorm", 0x0000, 0x0000000000000123ull},{"+inf", 0x7fff, 0},
	{"-inf", 0xffff, 0},                     {"qnan", 0x7fff, 0xc000000000000000ull},
	{"snan", 0x7fff, 0xa000000000000000ull}, {"100.25", 0x4005, 0xc880000000000000ull},
	{"-7e-3", 0xbff7, 0xe5604189374bc6a8ull},{"2^63", 0x403e, 0x8000000000000000ull},
};
#define NV ((int)(sizeof(V) / sizeof(V[0])))

struct op { const char *n; int mode; };
static const struct op MONO[] = {
	{"FMOVE", 0x00}, {"FINT", 0x01}, {"FSINH", 0x02}, {"FINTRZ", 0x03}, {"FSQRT", 0x04},
	{"FLOGNP1", 0x06}, {"FETOXM1", 0x08}, {"FTANH", 0x09}, {"FATAN", 0x0a}, {"FASIN", 0x0c},
	{"FATANH", 0x0d}, {"FSIN", 0x0e}, {"FTAN", 0x0f}, {"FETOX", 0x10}, {"FTWOTOX", 0x11},
	{"FTENTOX", 0x12}, {"FLOGN", 0x14}, {"FLOG10", 0x15}, {"FLOG2", 0x16}, {"FABS", 0x18},
	{"FCOSH", 0x19}, {"FNEG", 0x1a}, {"FACOS", 0x1c}, {"FCOS", 0x1d}, {"FGETEXP", 0x1e},
	{"FGETMAN", 0x1f}, {"FTST", 0x3a},
};
static const struct op DYA[] = {
	{"FDIV", 0x20}, {"FMOD", 0x21}, {"FADD", 0x22}, {"FMUL", 0x23}, {"FSGLDIV", 0x24},
	{"FREM", 0x25}, {"FSCALE", 0x26}, {"FSGLMUL", 0x27}, {"FSUB", 0x28}, {"FCMP", 0x38},
};

/* every check starts from FPCR 0 with FP0/FP1 loaded exactly */
static void setup(int a, int b, uint32_t fpcr)
{
	keep = nchk++ % stride == 0;
	e_fpcr(0);
	e_ldx(0, V[a].se, V[a].m);
	if (b >= 0) e_ldx(1, V[b].se, V[b].m);
	e_fpcr(fpcr);
	e_fpsr(0);
}

static void gen(void)
{
	char nm[64];
	static const int pick[] = {0, 2, 3, 4, 5, 6, 7, 8, 10, 11, 13, 15, 17, 18};
	const int npick = (int)(sizeof(pick) / sizeof(pick[0]));

	/* 1. monadic operations, every value, round to nearest extended */
	for (size_t o = 0; o < sizeof(MONO) / sizeof(MONO[0]); o++)
		for (int v = 0; v < NV; v++) {
			setup(v, -1, 0);
			e_rr(0, 0, MONO[o].mode);
			snprintf(nm, sizeof(nm), "%s %s", MONO[o].n, V[v].n);
			result(nm, 0);
		}
	/* FSINCOS: FP0 = sin, FP2 = cos */
	for (int v = 0; v < NV; v++) {
		setup(v, -1, 0);
		e_rr(0, 0, 0x32);
		snprintf(nm, sizeof(nm), "FSINCOS %s sin", V[v].n);
		result(nm, 0);
		snprintf(nm, sizeof(nm), "FSINCOS %s cos", V[v].n);
		result(nm, 2);
	}

	/* 2. dyadic operations FP1,FP0 */
	for (size_t o = 0; o < sizeof(DYA) / sizeof(DYA[0]); o++)
		for (int i = 0; i < npick; i++)
			for (int j = 0; j < npick; j++) {
				setup(pick[i], pick[j], 0);
				e_rr(1, 0, DYA[o].mode);
				snprintf(nm, sizeof(nm), "%s %s,%s", DYA[o].n, V[pick[j]].n, V[pick[i]].n);
				result(nm, 0);
			}

	/* 3. rounding modes and precisions */
	static const int rops[] = {0x22, 0x23, 0x20, 0x04, 0x00};
	static const char *rn[] = {"FADD", "FMUL", "FDIV", "FSQRT", "FMOVE"};
	static const int rpick[] = {2, 5, 7, 8, 9, 17, 18};
	for (int r = 0; r < 5; r++)
		for (int prec = 0; prec < 3; prec++)
			for (int mode = 0; mode < 4; mode++)
				for (int i = 0; i < 7; i++) {
					uint32_t fpcr = (uint32_t)(prec << 6 | mode << 4);
					int b = rpick[(i + 3) % 7];
					setup(rpick[i], b, fpcr);
					e_rr(rops[r] == 0x04 || rops[r] == 0x00 ? 0 : 1, 0, rops[r]);
					snprintf(nm, sizeof(nm), "%s %s p%d m%d", rn[r], V[rpick[i]].n, prec, mode);
					result(nm, 0);
				}

	/* 4. memory operands in every format: FMOVE.fmt and FADD.fmt d16(A2),FP0 */
	static const uint32_t L[] = {0, 1, 0xffffffffu, 0x7fffffffu, 0x80000000u, 123456789u};
	static const uint16_t W[] = {0, 1, 0xffff, 0x7fff, 0x8000, 1234};
	static const uint8_t B[] = {0, 1, 0xff, 0x7f, 0x80, 42};
	static const uint32_t S[] = {0x00000000u, 0x80000000u, 0x3f800000u, 0x40490fdbu, 0x7f800000u,
	                             0x7fc00000u, 0x00000001u, 0x7f7fffffu, 0xbf000000u};
	static const uint64_t D[] = {0, 0x8000000000000000ull, 0x3ff0000000000000ull, 0x400921fb54442d18ull,
	                             0x7ff0000000000000ull, 0x7ff8000000000000ull, 0x0000000000000001ull,
	                             0x7fefffffffffffffull, 0xc00c000000000000ull};
	static const uint8_t P[][12] = {
		{0x00,0x00,0x00,0x01, 0,0,0,0, 0,0,0,0},                    /* 1 */
		{0x80,0x00,0x00,0x03, 0x14,0x15,0x92,0x65, 0x35,0x89,0x79,0x32},   /* -3.1415926535897932 */
		{0x40,0x12,0x00,0x05, 0x50,0,0,0, 0,0,0,0},                 /* 5.5e-12 */
		{0x09,0x99,0x00,0x09, 0x99,0x99,0x99,0x99, 0x99,0x99,0x99,0x99},   /* 9.99..e999 */
		{0x7f,0xff,0x00,0x00, 0,0,0,0, 0,0,0,0},                    /* +inf */
		{0x00,0x00,0x00,0x00, 0,0,0,0, 0,0,0,0},                    /* +0 */
	};
	for (int op = 0; op < 2; op++) {
		int mode = op ? 0x22 : 0x00;
		const char *on = op ? "FADD" : "FMOVE";
		for (size_t i = 0; i < sizeof(L) / sizeof(L[0]); i++) {
			uint8_t b[4]; be32(b, L[i]);
			setup(5, -1, 0); e_mem(0, 0, mode, b, 4);
			snprintf(nm, sizeof(nm), "%s.L #$%08X", on, L[i]); result(nm, 0);
		}
		for (size_t i = 0; i < sizeof(W) / sizeof(W[0]); i++) {
			uint8_t b[2] = {(uint8_t)(W[i] >> 8), (uint8_t)W[i]};
			setup(5, -1, 0); e_mem(4, 0, mode, b, 2);
			snprintf(nm, sizeof(nm), "%s.W #$%04X", on, W[i]); result(nm, 0);
		}
		for (size_t i = 0; i < sizeof(B) / sizeof(B[0]); i++) {
			uint8_t b[2] = {B[i], 0};
			setup(5, -1, 0); e_mem(6, 0, mode, b, 1);
			snprintf(nm, sizeof(nm), "%s.B #$%02X", on, B[i]); result(nm, 0);
		}
		for (size_t i = 0; i < sizeof(S) / sizeof(S[0]); i++) {
			uint8_t b[4]; be32(b, S[i]);
			setup(5, -1, 0); e_mem(1, 0, mode, b, 4);
			snprintf(nm, sizeof(nm), "%s.S #$%08X", on, S[i]); result(nm, 0);
		}
		for (size_t i = 0; i < sizeof(D) / sizeof(D[0]); i++) {
			uint8_t b[8]; be32(b, (uint32_t)(D[i] >> 32)); be32(b + 4, (uint32_t)D[i]);
			setup(5, -1, 0); e_mem(5, 0, mode, b, 8);
			snprintf(nm, sizeof(nm), "%s.D #$%016llX", on, (unsigned long long)D[i]); result(nm, 0);
		}
		for (int v = 0; v < NV; v++) {
			uint8_t b[12]; xval(b, V[v].se, V[v].m);
			setup(5, -1, 0); e_mem(2, 0, mode, b, 12);
			snprintf(nm, sizeof(nm), "%s.X %s", on, V[v].n); result(nm, 0);
		}
		for (size_t i = 0; i < sizeof(P) / sizeof(P[0]); i++) {
			setup(5, -1, 0); e_mem(3, 0, mode, P[i], 12);
			snprintf(nm, sizeof(nm), "%s.P #%zu", on, i); result(nm, 0);
		}
	}

	/* 5. FMOVE FP0 out in every format, several values and rounding modes */
	static const int ofmt[] = {0, 1, 4, 5, 6, 2};
	static const char *ofn[] = {"L", "S", "W", "D", "B", "X"};
	for (int v = 0; v < NV; v++)
		for (int mode = 0; mode < 4; mode++)
			for (int f = 0; f < 6; f++) {
				setup(v, -1, (uint32_t)(mode << 4));
				uint8_t *s = slot("");
				if (keep) snprintf(names[nslots - 1], sizeof(names[0]), "FMOVE.%s %s m%d out", ofn[f], V[v].n, mode);
				e_store(s, ofmt[f], 0, 0, 0);
				e_stfpsr(s);
				e_next();
			}
	/* packed out: static k -3..17, dynamic k from D3 */
	static const int ks[] = {0, 1, 5, 17, 0x7d, 0x7f};
	for (int v = 0; v < NV; v++) {
		for (size_t k = 0; k < sizeof(ks) / sizeof(ks[0]); k++) {
			setup(v, -1, 0);
			uint8_t *s = slot("");
			if (keep) snprintf(names[nslots - 1], sizeof(names[0]), "FMOVE.P %s k%d out", V[v].n, ks[k]);
			e_store(s, 3, 0, ks[k], 0);
			e_stfpsr(s);
			e_next();
		}
		setup(v, -1, 0);
		emit("\tmoveq\t#%d,d3\n", 4 + (v % 9));
		uint8_t *s = slot("");
		if (keep) snprintf(names[nslots - 1], sizeof(names[0]), "FMOVE.P %s k=D3 out", V[v].n);
		e_store(s, 7, 0, 3 << 4, (uint32_t)(4 + (v % 9)));
		e_stfpsr(s);
		e_next();
	}

	/* 6. FMOVECR, every documented constant, two rounding modes and precisions */
	static const int rom[] = {0x00, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0x30, 0x31, 0x32, 0x33, 0x34,
	                          0x35, 0x36, 0x37, 0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f};
	for (size_t r = 0; r < sizeof(rom) / sizeof(rom[0]); r++)
		for (int m = 0; m < 4; m++) {
			uint32_t fpcr = (uint32_t)((m & 1 ? 0x10 : 0x30) | (m >> 1) << 6);
			setup(0, -1, fpcr);
			e_cr(0, rom[r]);
			snprintf(nm, sizeof(nm), "FMOVECR #$%02X fpcr %02X", rom[r], fpcr);
			result(nm, 0);
		}

	/* 7. conditionals: FPSR condition codes x 32 predicates, FScc to the slots */
	for (int cc = 0; cc < 16; cc++) {
		keep = nchk++ % stride == 0;
		e_fpcr(0);
		e_fpsr((uint32_t)cc << 24);
		uint8_t *s0 = slot(""), *s1 = slot("");
		if (keep) snprintf(names[nslots - 2], sizeof(names[0]), "FScc 0-15 cc=%X", cc);
		if (keep) snprintf(names[nslots - 1], sizeof(names[0]), "FScc 16-31 cc=%X", cc);
		uint8_t *bytes[2] = {s0, s1};               /* FScc (A1)+ fills both slots */
		for (int p = 0; p < 32; p++) {
			int r = fpe_cond(p);
			if (r < 0) fail("FScc exception");
			emit("\tdc.w\t$F259,$%04X\n", p);  /* FScc (A1)+ */
			bytes[p / 16][p % 16] = r ? 0xff : 0x00;
		}
		/* A1 is 32 bytes on: the FPSR after all (BSUN/IOP side effects) in a third slot */
		uint8_t *s2 = slot("");
		if (keep) snprintf(names[nslots - 1], sizeof(names[0]), "FScc cc=%X FPSR", cc);
		e_stfpsr(s2);
		e_next();
	}

	/* 8. FMOVEM.X: load FP0-FP7 from the table, store them in and out of order */
	keep = 1;
	{
		uint8_t img[96];
		for (int r = 0; r < 8; r++) xval(img + 12 * r, V[(r * 3 + 1) % NV].se, V[(r * 3 + 1) % NV].m);
		int off = add_val(img, 96);
		uint8_t o[128];
		e_fpcr(0);
		emit("\tdc.w\t$F22A,$D0FF,$%04X\n", off);          /* FMOVEM.X d16(A2),FP0-FP7 */
		run(0xD0FF, img, 96, 0, o);
		e_fpsr(0);
		/* FMOVEM.X FP0-FP7,(A1): six slots of 16 = 96 bytes */
		emit("\tdc.w\t$F211,$F0FF\n");
		int n = run(0xF0FF, NULL, 0, 0, o);
		if (n != 96) fail("FMOVEM out");
		for (int k = 0; k < 6; k++) {
			uint8_t *s = slot("");
			if (keep) snprintf(names[nslots - 1], sizeof(names[0]), "FMOVEM.X FP0-FP7 out, part %d", k);
			memcpy(s, o + 16 * k, 16);
		}
		emit("\tlea\t96(a1),a1\n");
		/* FMOVEM.X FP1/FP4/FP7,-(A1): predecrement list, bit 7 = FP0; 36 bytes end at the
		 * third slot's end, A1 then points 12 bytes into the first */
		emit("\tlea\t48(a1),a1\n");
		emit("\tdc.w\t$F221,$E049\n");
		n = run(0xE049, NULL, 0, 0, o);
		if (n != 36) fail("FMOVEM predecrement out");
		uint8_t buf[48];
		memset(buf, 0xA5, 48);
		memcpy(buf + 12, o, 36);
		for (int k = 0; k < 3; k++) {
			uint8_t *s = slot("");
			if (keep) snprintf(names[nslots - 1], sizeof(names[0]), "FMOVEM.X -(A1) part %d", k);
			memcpy(s, buf + 16 * k, 16);
		}
		emit("\tlea\t36(a1),a1\n");
		/* control registers: FMOVEM.L #imm,FPCR/FPSR then FPCR/FPSR/FPIAR out */
		uint8_t c[8];
		be32(c, 0x00000060); be32(c + 4, 0x0f00ff00);
		emit("\tdc.w\t$F23C,$9800,$0000,$0060,$0F00,$FF00\n");
		run(0x9800, c, 8, 0, o);
		emit("\tdc.w\t$F211,$B800\n");                    /* FMOVEM.L FPCR/FPSR,(A1) */
		n = run(0xB800, NULL, 0, 0, o);
		if (n != 8) fail("FMOVEM.L out");
		uint8_t *s = slot("FMOVEM.L FPCR/FPSR");
		memcpy(s, o, 8);
		e_next();
	}
	emit("\trts\n");
}

int main(int argc, char **argv)
{
	if (argc == 3 && !strcmp(argv[1], "-s")) stride = atoi(argv[2]);
	if (stride < 1) stride = 1;
	fpe_reset();
	printf("; generated by tools/fputest/gen_fputest.c - do not edit\n");
	printf("gen_code:\n");
	gen();
	printf("\n\tdata\n\teven\ngen_vals:\n");
	for (int i = 0; i < nvals; i += 2)
		printf("%s$%02X%02X%s", i % 32 == 0 ? "\tdc.w\t" : "", vals[i], i + 1 < nvals ? vals[i + 1] : 0,
		       (i % 32 == 30 || i + 2 >= nvals) ? "\n" : ",");
	printf("\teven\ngen_expect:\n");
	for (int i = 0; i < nslots * 16; i += 16) {
		printf("\tdc.l\t");
		for (int k = 0; k < 16; k += 4)
			printf("$%02X%02X%02X%02X%s", expect[i + k], expect[i + k + 1], expect[i + k + 2],
			       expect[i + k + 3], k < 12 ? "," : "\n");
	}
	printf("gen_names:\n");
	for (int i = 0; i < nslots; i++) {
		char clean[48];
		int k = 0;
		for (const char *p = names[i]; *p && k < 40; p++) clean[k++] = (*p == '"') ? '\'' : *p;
		clean[k] = 0;
		printf("\tdc.b\t\"%-40s\"\n", clean);
	}
	printf("GEN_SLOTS\tequ\t%d\n", nslots);
	fprintf(stderr, "gen_fputest: %d slots, %d operand bytes\n", nslots, nvals);
	return 0;
}
