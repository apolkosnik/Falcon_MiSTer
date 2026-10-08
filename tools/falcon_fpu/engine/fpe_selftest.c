/* fpe_selftest.c - proves the synthetic-EA technique of fpe_exec() against Hatari's fpp.c driven the
 * normal way.
 *
 *   golden   : fpuop_arithmetic(real opcode, cmd) with a real EA ((A1), (A1)+, -(A1), d16(A1), #imm,
 *              Dn) reading/writing a fake 68k RAM array through the cp accessors, normal regs.fp/fpsr/fpcr.
 *   candidate: fpe_exec(cmd, operand bytes, aux, iaddr) -> synthetic EA (A0) over a 256 byte buffer.
 *
 * Both sides are the SAME fpp.c, so this test verifies the EA mapping, the byte-order/extent handling,
 * the dynamic-list/k-factor (AUX) plumbing, the FPIAR/PC stub and the exception/unimplemented hooks of the
 * shim - NOT the arithmetic (Hatari's softfloat is taken on trust, it is the project's golden model).
 *
 * Per check the FP0-FP7 (all 80 bits), FPSR, FPCR, FPIAR, event flags/vector, result bytes and the whole
 * fake RAM image are compared.  Usage: fpe_selftest [-v] [-q]   (-v: print exp and got for every check,
 * -q: only failures and the summary). Exit status 0 = PASS.
 */
#include "sysconfig.h"
#include "sysdeps.h"
#include "main.h"
#include "hatari-glue.h"
#include "options_cpu.h"
#include "memory.h"
#include "newcpu.h"
#include "fpp.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "falcon_fpu_engine.h"
#include "fpe_internal.h"

/* ---------------------------------------------------------------- state helpers */
typedef struct { uint16_t se[8]; uint64_t m[8]; uint32_t fpcr, fpsr, fpiar; int pend; } St;
typedef struct { uint16_t cmd; uint8_t in[FPE_MAXIO]; int in_len; uint16_t aux; } Step;
typedef struct { int flags; uint32_t vec; int out_len; uint8_t out[FPE_MAXIO]; } Res;

#define IADDR 0x00012340u
#define BASE  0x100u
#define GSIZE 0x400u

static int verbose, quiet;
static int n_ok, n_fail;
static int cat_ok, cat_fail;
static const char *cat_name = "";
static void tally(int ok) { if (ok) { n_ok++; cat_ok++; } else { n_fail++; cat_fail++; } }

static void set_state(const St *s)
{
	fpe_clear_exception();
	for (int i = 0; i < 8; i++) fpe_set_fp(i, s->se[i], s->m[i]);
	fpe_set_fpcr(s->fpcr);
	fpe_set_fpsr(s->fpsr);
	fpe_set_fpiar(s->fpiar);
	for (int i = 0; i < 16; i++) regs.regs[i] = 0;
	regs.regs[1] = 0xDEADBE00u;                       /* D1 for the Dn cases */
	fpe_shim_clear_events();
}

static void get_state(St *s)
{
	for (int i = 0; i < 8; i++) fpe_get_fp(i, &s->se[i], &s->m[i]);
	s->fpcr = fpe_fpcr();
	s->fpsr = fpe_fpsr();
	s->fpiar = fpe_fpiar();
	s->pend = (int)regs.fp_exp_pend;
}

static int st_eq(const St *a, const St *b)
{
	for (int i = 0; i < 8; i++) if (a->se[i] != b->se[i] || a->m[i] != b->m[i]) return 0;
	return a->fpcr == b->fpcr && a->fpsr == b->fpsr && a->fpiar == b->fpiar && a->pend == b->pend;
}

static St base_state(void)
{
	static const struct { uint16_t se; uint64_t m; } v[8] = {
		{ 0x3fff, 0x8000000000000000ull }, { 0xc000, 0xa000000000000000ull },
		{ 0x4000, 0xc90fdaa22168c235ull }, { 0x0000, 0 },
		{ 0xffff, 0 },                     { 0x7fff, 0xc000000000000000ull },
		{ 0x0000, 0x0000000000001234ull }, { 0x3ffb, 0xcccccccccccccccdull } };
	St s; memset(&s, 0, sizeof s);
	for (int i = 0; i < 8; i++) { s.se[i] = v[i].se; s.m[i] = v[i].m; }
	return s;
}

static void d2x(double d, uint16_t *se, uint64_t *m)
{
	union { double d; uint64_t u; } c; c.d = d;
	if (d == 0) { *se = (c.u >> 63) ? 0x8000 : 0; *m = 0; return; }
	*se = (uint16_t)(((c.u >> 63) << 15) | (((c.u >> 52) & 0x7ff) - 1023 + 0x3fff));
	*m = (1ull << 63) | ((c.u & ((1ull << 52) - 1)) << 11);
}

/* ---------------------------------------------------------------- printing */
static void hex(char *d, size_t n, const uint8_t *p, int len)
{
	size_t o = 0; d[0] = 0;
	if (len <= 0) { snprintf(d, n, "-"); return; }
	for (int i = 0; i < len && o + 3 < n; i++) o += (size_t)snprintf(d + o, n - o, "%02x", p[i]);
}

static void summ(char *d, size_t n, const St *init, const St *s, const Res *r, int ns)
{
	char h[2 * FPE_MAXIO + 8], f[200]; size_t o;
	f[0] = 0; o = 0;
	for (int i = 0; i < 8; i++)
		if (s->se[i] != init->se[i] || s->m[i] != init->m[i])
			o += (size_t)snprintf(f + o, sizeof f - o, " fp%d=%04x:%016llx", i, s->se[i], (unsigned long long)s->m[i]);
	d[0] = 0; o = 0;
	for (int k = 0; k < ns; k++) {
		hex(h, sizeof h, r[k].out, r[k].out_len);
		o += (size_t)snprintf(d + o, n - o, "[fl=%d v=%u out(%d)=%.40s%s]", r[k].flags, r[k].vec, r[k].out_len, h,
		                      strlen(h) > 40 ? ".." : "");
	}
	snprintf(d + o, n - o, " fpsr=%08x fpcr=%08x fpiar=%08x%s%s", s->fpsr, s->fpcr, s->fpiar,
	         s->pend ? " PEND" : "", f);
}

/* ---------------------------------------------------------------- golden (normal fpp.c use) */
enum { GM_IND, GM_POST, GM_PRE, GM_D16, GM_IMM, GM_NMODES };
static const char *gm_name[] = { "(A1)", "(A1)+", "-(A1)", "d16(A1)", "#imm" };
static const uint16_t gm_opc[] = { 0xF211, 0xF219, 0xF221, 0xF229, 0xF23C };

static int fmt_size(int f)
{
	static const int sz[8] = { 4, 4, 12, 12, 2, 8, 1, 12 };
	return sz[f & 7];
}
static int pop8(unsigned v) { int n = 0; for (v &= 0xff; v; v >>= 1) n += v & 1; return n; }

/* independent expectation of the transfer size of an instruction */
static int exp_size(const Step *s)
{
	unsigned cls = s->cmd >> 13;
	if (cls == 3) return fmt_size((s->cmd >> 10) & 7);
	if (cls == 5) { int n = pop8((s->cmd >> 10) & 7); return 4 * (n ? n : 1); }
	if (cls == 7) { unsigned m = (s->cmd >> 11) & 3; return 12 * pop8((m & 1) ? s->aux : s->cmd); }
	return 0;
}
static int is_out(const Step *s) { unsigned c = s->cmd >> 13; return c == 3 || c == 5 || c == 7; }

static unsigned modes_for1(const Step *s)
{
	unsigned cls = s->cmd >> 13, m;
	switch (cls) {
	case 0: return 1u << GM_IND;                                   /* register to register */
	case 2: m = (1u << GM_IND) | (1u << GM_POST) | (1u << GM_D16);
	        if ((s->cmd & 0xfc00) != 0x5c00) m |= 1u << GM_IMM;     /* FMOVECR has no operand */
	        if ((s->cmd & 0xfc00) == 0x5c00) m = 1u << GM_IND;
	        return m;
	case 3: if (((s->cmd >> 10) & 7) == 7)          /* dynamic-k packed: fpp.c has size 0 in its (An)+/-(An) tables */
			return (1u << GM_IND) | (1u << GM_D16);
	        return (1u << GM_IND) | (1u << GM_POST) | (1u << GM_PRE) | (1u << GM_D16);
	case 4: return (1u << GM_IND) | (1u << GM_POST) | (1u << GM_D16) | (1u << GM_IMM);
	case 5: return (1u << GM_IND) | (1u << GM_POST) | (1u << GM_PRE) | (1u << GM_D16);
	case 6: return (1u << GM_IND) | (1u << GM_POST) | (1u << GM_D16);
	case 7: return (((s->cmd >> 11) & 3) < 2) ? (1u << GM_PRE) : ((1u << GM_IND) | (1u << GM_D16));
	}
	return 1u << GM_IND;
}

static unsigned modes_for(const Step *s, int ns)
{
	unsigned m = ~0u;
	for (int k = 0; k < ns; k++) m &= modes_for1(&s[k]);
	return m ? m : (1u << GM_IND);
}

static uint8_t gmem[GSIZE], gmem0[GSIZE];

/* run one step with the real EA mode gm; returns events in r (out bytes not filled: caller compares RAM) */
static void golden_step(const Step *s, int gm, Res *r, int *a1_ok)
{
	uint16_t is[16]; int nis = 0;
	int sz = exp_size(s);
	memset(gmem, 0xA5, sizeof gmem);
	memcpy(gmem + BASE, s->in, (size_t)s->in_len);
	memcpy(gmem0, gmem, sizeof gmem);
	regs.regs[9] = BASE;
	if (gm == GM_PRE) regs.regs[9] = BASE + (uint32_t)sz;
	if (gm == GM_D16) { regs.regs[9] = BASE - 0x20; is[nis++] = 0x20; }
	if (gm == GM_IMM) {
		if (s->in_len == 1) is[nis++] = s->in[0];                   /* byte immediate: low byte of a word */
		else for (int i = 0; i + 1 < s->in_len && nis < 16; i += 2) is[nis++] = (uint16_t)((s->in[i] << 8) | s->in[i + 1]);
	}
	regs.regs[(s->cmd >> 4) & 7] = (uae_u32)(uae_s32)(int16_t)s->aux;  /* the data register the CPU would hold */
	fpe_shim_use_memory(gmem, GSIZE, is, nis);
	fpe_shim_set_pc(IADDR + 4);
	fpe_shim_clear_events();
	fpuop_arithmetic(gm_opc[gm], s->cmd);
	r->flags = (fpe_ev_unimpl ? FPE_UNIMPL : 0) | (fpe_ev_exc ? FPE_EXCEPTION : 0);
	r->vec = fpe_ev_vector;
	r->out_len = 0;
	fpe_shim_use_buffer();
	*a1_ok = 1;
	if (!r->flags && (gm == GM_POST || gm == GM_PRE) && sz) {
		uint32_t want = (gm == GM_POST) ? BASE + (uint32_t)sz : BASE;
		if (s->cmd >> 13 == 2 || s->cmd >> 13 == 4 || s->cmd >> 13 == 6 || s->cmd >> 13 == 3 || s->cmd >> 13 == 5 || s->cmd >> 13 == 7)
			*a1_ok = (regs.regs[9] == want);
	}
}

/* ---------------------------------------------------------------- one check */
static void run_check(const char *name, const Step *steps, int ns, const St *init)
{
	St eg, gg; Res er[3], gr[3];
	unsigned modes = modes_for(steps, ns);
	char sa[600], sb[600];

	/* candidate: fpe_exec with the synthetic EA */
	set_state(init);
	for (int k = 0; k < ns; k++) {
		const Step *s = &steps[k];
		er[k].flags = fpe_exec(s->cmd, s->in, s->in_len, s->aux, IADDR, er[k].out, &er[k].out_len);
		er[k].vec = fpe_vector();
		if (!er[k].flags) er[k].vec = 0;
	}
	get_state(&eg);

	for (int gm = 0; gm < GM_NMODES; gm++) {
		int ok = 1, a1ok = 1;
		if (!(modes & (1u << gm))) continue;
		/* every step of a sequence must support the mode; sequences use register-form or IND/POST only */
		set_state(init);
		uint8_t eimg[GSIZE];
		for (int k = 0; k < ns && ok; k++) {
			int a;
			golden_step(&steps[k], gm, &gr[k], &a);
			a1ok &= a;
			/* expected RAM image = initial image with the candidate's result bytes at BASE */
			memcpy(eimg, gmem0, sizeof eimg);
			if (er[k].out_len) memcpy(eimg + BASE, er[k].out, (size_t)er[k].out_len);
			if (memcmp(eimg, gmem, sizeof eimg)) ok = 0;
			if (gr[k].flags != er[k].flags || (gr[k].flags && gr[k].vec != er[k].vec)) ok = 0;
			if (!gr[k].flags && is_out(&steps[k]) && er[k].out_len != exp_size(&steps[k])) ok = 0;
			if (gr[k].flags & FPE_UNIMPL) {}
			/* golden result bytes for the print: what is in RAM at BASE.. */
			gr[k].out_len = er[k].out_len;
			memcpy(gr[k].out, gmem + BASE, (size_t)er[k].out_len);
		}
		get_state(&gg);
		if (!st_eq(&gg, &eg)) ok = 0;
		if (!a1ok) ok = 0;
		tally(ok);
		if (!ok || verbose || !quiet) {
			char tag[48];
			snprintf(tag, sizeof tag, "%s%s", gm_name[gm], a1ok ? "" : " A1!");
			summ(sa, sizeof sa, init, &gg, gr, ns);
			summ(sb, sizeof sb, init, &eg, er, ns);
			if (ok && !verbose) printf("[ ok ] %-9s %-34s %-8s exp==got %s\n", cat_name, name, tag, sa);
			else {
				printf("[%s] %-9s %-34s %-8s\n   exp: %s\n   got: %s\n", ok ? " ok " : "FAIL", cat_name, name, tag, sa, sb);
			}
		}
	}
	set_state(init);                                 /* leave a clean state */
}

static void one(const char *name, uint16_t cmd, const uint8_t *in, int in_len, uint16_t aux, const St *init)
{
	Step s; memset(&s, 0, sizeof s);
	s.cmd = cmd; s.aux = aux; s.in_len = in_len;
	if (in_len) memcpy(s.in, in, (size_t)in_len);
	run_check(name, &s, 1, init);
}

static void category(const char *c)
{
	if (cat_ok || cat_fail) printf("  -- %s: %d ok, %d fail\n", cat_name, cat_ok, cat_fail);
	cat_ok = cat_fail = 0; cat_name = c;
}

/* ---------------------------------------------------------------- Dn (register direct) equivalence */
static void dn_check(const char *name, uint16_t cmd, const uint8_t *in, int in_len, const St *init)
{
	St eg, gg; uint8_t out[FPE_MAXIO]; int ol, fl, ok = 1;
	uint32_t d1;
	int cls = cmd >> 13;
	set_state(init);
	fl = fpe_exec(cmd, in, in_len, 0, IADDR, out, &ol);
	get_state(&eg);
	set_state(init);
	if (cls == 2 || cls == 4) {                                  /* Dn is the source */
		uint32_t v = 0;
		for (int i = 0; i < in_len; i++) v = (v << 8) | in[i];
		regs.regs[1] = in_len == 1 ? (0xDEADBE00u | v) : (in_len == 2 ? (0xDEAD0000u | v) : v);
	}
	fpe_shim_set_pc(IADDR + 4);
	fpe_shim_use_memory(gmem, GSIZE, NULL, 0);
	fpuop_arithmetic(0xF201, cmd);                               /* EA = D1 */
	fpe_shim_use_buffer();
	d1 = regs.regs[1];
	get_state(&gg);
	if (fl != (fpe_ev_unimpl ? FPE_UNIMPL : 0) + (fpe_ev_exc ? FPE_EXCEPTION : 0) || !st_eq(&gg, &eg)) ok = 0;
	if (cls == 3 || cls == 5) {                                  /* Dn is the destination */
		uint32_t m = 0;
		for (int i = 0; i < ol; i++) m = (m << 8) | out[i];
		if (ol == 1) ok &= ((d1 & 0xff) == m) && ((d1 & ~0xffu) == 0xDEADBE00u);
		else if (ol == 2) ok &= ((d1 & 0xffff) == m) && ((d1 >> 16) == 0xDEADu);
		else if (ol == 4) ok &= (d1 == m);
		else ok &= (ol == 0);
	}
	tally(ok);
	if (!ok || !quiet)
		printf("[%s] %-9s %-34s Dn       exp D1=%08x got bytes(%d)=%02x%02x%02x%02x fpsr %08x/%08x\n", ok ? " ok " : "FAIL",
		       cat_name, name, d1, ol, ol > 0 ? out[0] : 0, ol > 1 ? out[1] : 0, ol > 2 ? out[2] : 0, ol > 3 ? out[3] : 0,
		       gg.fpsr, eg.fpsr);
}

/* ---------------------------------------------------------------- operand pools */
enum { F_L, F_S, F_X, F_P, F_W, F_D, F_B, F_PD };           /* cmd size field values */

typedef struct { const char *n; uint16_t se; uint64_t m; } XV;
static XV xpool[40]; static int nx;
static void addx(const char *n, uint16_t se, uint64_t m) { xpool[nx].n = n; xpool[nx].se = se; xpool[nx].m = m; nx++; }
static void addd(const char *n, double d) { uint16_t se; uint64_t m; d2x(d, &se, &m); addx(n, se, m); }
static void init_pool(void)
{
	addx("+0", 0, 0); addx("-0", 0x8000, 0);
	addd("1.0", 1.0); addd("-1.0", -1.0); addd("1.5", 1.5); addd("2.5", 2.5); addd("-3.5", -3.5); addd("0.5", 0.5);
	addd("-0.5", -0.5); addd("pi", 3.14159265358979323846); addd("123456.789", 123456.789); addd("1e10", 1e10);
	addd("-300", -300.0); addd("70000.5", 70000.5); addd("1e-5", 1e-5); addd("1e-40", 1e-40); addd("1e-310", 1e-310);
	addd("1e300", 1e300); addd("1e-300", 1e-300);
	addx("+inf", 0x7fff, 0); addx("-inf", 0xffff, 0);
	addx("qNaN", 0x7fff, 0xc000000000000000ull); addx("sNaN", 0x7fff, 0x8000000000000001ull);
	addx("-qNaN", 0xffff, 0xc000000000000123ull);
	addx("denorm", 0x0000, 0x0000000000001000ull); addx("-denorm", 0x8000, 0x4000000000000000ull);
	addx("minnorm", 0x0001, 0x8000000000000000ull); addx("huge", 0x7ffe, 0xffffffffffffffffull);
	addx("2^1030", 0x4405, 0x8000000000000000ull); addx("2^-1055", 0x3be0, 0x8000000000000000ull);
	addx("unnormal", 0x3fff, 0x4000000000000000ull);
}

static void put32(uint8_t *p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = (uint8_t)v; }
static void put64(uint8_t *p, uint64_t v) { put32(p, (uint32_t)(v >> 32)); put32(p + 4, (uint32_t)v); }
static void xbytes(uint8_t *o, uint16_t se, uint64_t m, uint16_t pad) { o[0] = se >> 8; o[1] = (uint8_t)se; o[2] = pad >> 8; o[3] = (uint8_t)pad; put64(o + 4, m); }

/* packed decimal: sm/se signs, 3 exponent digits, integer digit, 16 fraction digits */
static void pk(uint8_t *o, int sm, int se, int e, int idig, const char *frac)
{
	uint32_t l0 = ((uint32_t)sm << 31) | ((uint32_t)se << 30) | (((uint32_t)(e / 100) & 15) << 24) |
	              (((uint32_t)((e / 10) % 10)) << 20) | (((uint32_t)(e % 10)) << 16) | ((uint32_t)idig & 15);
	uint32_t l1 = 0, l2 = 0;
	for (int i = 0; i < 8; i++) { l1 = (l1 << 4) | (uint32_t)(frac[i] - '0'); l2 = (l2 << 4) | (uint32_t)(frac[8 + i] - '0'); }
	put32(o, l0); put32(o + 4, l1); put32(o + 8, l2);
}
static void pkraw(uint8_t *o, uint32_t a, uint32_t b, uint32_t c) { put32(o, a); put32(o + 4, b); put32(o + 8, c); }

static const uint32_t fpcr_var[] = { 0x00, 0x10, 0x20, 0x30, 0x40, 0x80, 0x90, 0xA0, 0xB0 };  /* prec/round combos */
static const char *fpcr_nm[]    = { "RN/X", "RZ/X", "RM/X", "RP/X", "RN/S", "RN/D", "RZ/D", "RM/D", "RP/D" };

/* ---------------------------------------------------------------- tests */
static void t_fmove_in(void)
{
	St s = base_state(); uint8_t b[16]; char nm[96];
	category("FMOVE-in");
	{ static const uint8_t v[] = { 0x00, 0x01, 0x7f, 0x80, 0xff };
	  for (unsigned i = 0; i < sizeof v; i++) { b[0] = v[i]; snprintf(nm, sizeof nm, "FMOVE.B #%02x,FP3", v[i]); one(nm, 0x4000 | F_B << 10 | 3 << 7, b, 1, 0, &s); } }
	{ static const uint16_t v[] = { 0, 1, 0x7fff, 0x8000, 0xffff, 0x1234 };
	  for (unsigned i = 0; i < sizeof v / sizeof *v; i++) { b[0] = v[i] >> 8; b[1] = (uint8_t)v[i]; snprintf(nm, sizeof nm, "FMOVE.W #%04x,FP3", v[i]); one(nm, 0x4000 | F_W << 10 | 3 << 7, b, 2, 0, &s); } }
	{ static const uint32_t v[] = { 0, 1, 0xffffffffu, 0x7fffffffu, 0x80000000u, 0x12345678u };
	  for (unsigned i = 0; i < sizeof v / sizeof *v; i++) { put32(b, v[i]); snprintf(nm, sizeof nm, "FMOVE.L #%08x,FP3", v[i]); one(nm, 0x4000 | F_L << 10 | 3 << 7, b, 4, 0, &s); } }
	{ static const uint32_t v[] = { 0, 0x80000000u, 0x3fc00000u, 0xc0200000u, 0x7f800000u, 0xff800000u, 0x7fc00000u,
	                                0x7fa00001u, 0x00000001u, 0x00400000u, 0x80000001u, 0x7f7fffffu, 0x00800000u };
	  for (unsigned i = 0; i < sizeof v / sizeof *v; i++) { put32(b, v[i]); snprintf(nm, sizeof nm, "FMOVE.S #%08x,FP3", v[i]); one(nm, 0x4000 | F_S << 10 | 3 << 7, b, 4, 0, &s); } }
	{ static const uint64_t v[] = { 0, 0x8000000000000000ull, 0x3ff8000000000000ull, 0xc004000000000000ull,
	                                0x7ff0000000000000ull, 0xfff0000000000000ull, 0x7ff8000000000000ull,
	                                0x7ff0000000000001ull, 0x0000000000000001ull, 0x000fffffffffffffull,
	                                0x7fefffffffffffffull, 0x0010000000000000ull, 0xfff8000000000bcdull };
	  for (unsigned i = 0; i < sizeof v / sizeof *v; i++) { put64(b, v[i]); snprintf(nm, sizeof nm, "FMOVE.D #%016llx,FP3", (unsigned long long)v[i]); one(nm, 0x4000 | F_D << 10 | 3 << 7, b, 8, 0, &s); } }
	for (int i = 0; i < nx; i++) {
		xbytes(b, xpool[i].se, xpool[i].m, 0); snprintf(nm, sizeof nm, "FMOVE.X %s,FP3", xpool[i].n);
		one(nm, 0x4000 | F_X << 10 | 3 << 7, b, 12, 0, &s);
	}
	xbytes(b, 0x3fff, 0x8000000000000000ull, 0xBEEF); one("FMOVE.X 1.0 pad=beef,FP5", 0x4000 | F_X << 10 | 5 << 7, b, 12, 0, &s);
	/* packed */
	{ uint8_t p[12]; struct { const char *n; uint8_t d[12]; } pv[16]; int np = 0;
	  pk(pv[np].d, 0, 0, 0, 1, "5000000000000000"); pv[np++].n = "+1.5E+0";
	  pk(pv[np].d, 1, 1, 10, 1, "2345678901234567"); pv[np++].n = "-1.2345678901234567E-10";
	  pk(pv[np].d, 0, 0, 0, 0, "0000000000000000"); pv[np++].n = "+0";
	  pk(pv[np].d, 1, 0, 0, 0, "0000000000000000"); pv[np++].n = "-0";
	  pk(pv[np].d, 0, 0, 308, 9, "9999999999999999"); pv[np++].n = "+9.999..E+308";
	  pk(pv[np].d, 0, 1, 999, 1, "0000000000000000"); pv[np++].n = "+1E-999";
	  pk(pv[np].d, 0, 0, 999, 1, "0000000000000000"); pv[np++].n = "+1E+999 (overflow)";
	  pk(pv[np].d, 0, 0, 4, 1, "2345678000000000"); pv[np++].n = "+1.2345678E+4";
	  pkraw(pv[np].d, 0x7fff0000, 0, 0);          pv[np++].n = "+inf";
	  pkraw(pv[np].d, 0xffff0000, 0, 0);          pv[np++].n = "-inf";
	  pkraw(pv[np].d, 0x7fff0000, 0, 1);          pv[np++].n = "NaN";
	  pkraw(pv[np].d, 0x7fff0000, 0x40000000, 0); pv[np++].n = "NaN2";
	  pkraw(pv[np].d, 0x00010000, 0x50000000, 0); pv[np++].n = "bad digits";
	  for (int i = 0; i < np; i++) { memcpy(p, pv[i].d, 12); snprintf(nm, sizeof nm, "FMOVE.P %s,FP2", pv[i].n); one(nm, 0x4000 | F_P << 10 | 2 << 7, p, 12, 0, &s); } }
	/* FPCR precision/rounding does not change an exact conversion but exercise a few */
	for (int c = 0; c < 4; c++) { St t = s; t.fpcr = fpcr_var[c * 2 % 9]; xbytes(b, 0x3fff, 0xffffffffffffffffull, 0);
		snprintf(nm, sizeof nm, "FMOVE.X rounded-in fpcr=%02x", t.fpcr); one(nm, 0x4000 | F_X << 10 | 1 << 7, b, 12, 0, &t); }
}

static void t_fmove_out(void)
{
	char nm[128];
	category("FMOVE-out");
	static const int fmts[] = { F_B, F_W, F_L, F_S, F_D, F_X, F_P };
	static const char *fn[] = { "B", "W", "L", "S", "D", "X", "P{k=17}" };
	for (int i = 0; i < nx; i++) for (int f = 0; f < 7; f++) {
		int nc = (fmts[f] == F_X) ? 1 : (fmts[f] == F_P ? 2 : 4);
		for (int c = 0; c < nc; c++) {
			St s = base_state(); uint32_t cr = (fmts[f] == F_S || fmts[f] == F_D) ? fpcr_var[c * 2 + 1 > 8 ? 8 : c * 2 + 1] : (uint32_t)(c << 4);
			s.se[2] = xpool[i].se; s.m[2] = xpool[i].m; s.fpcr = cr;
			snprintf(nm, sizeof nm, "FMOVE.%s FP2=%s fpcr=%02x", fn[f], xpool[i].n, cr);
			one(nm, (uint16_t)(0x6000 | fmts[f] << 10 | 2 << 7 | (fmts[f] == F_P ? 17 : 0)), NULL, 0, 0, &s);
		}
	}
	/* packed: static k-factors, dynamic k (AUX), rounding modes, enabled-exception FPCR */
	static const int ks[] = { 0, 1, 3, 17, 0x7f /* -1 */, 0x60 /* -32 */, 0x40 /* -64 */, 0x3f /* 63: invalid */ };
	for (int i = 0; i < nx; i += 2) for (unsigned k = 0; k < sizeof ks / sizeof *ks; k++) for (int c = 0; c < 4; c++) {
		St s = base_state(); s.se[4] = xpool[i].se; s.m[4] = xpool[i].m; s.fpcr = (uint32_t)(c << 4);
		snprintf(nm, sizeof nm, "FMOVE.P FP4=%s {#%d} rm=%d", xpool[i].n, ks[k], c);
		one(nm, (uint16_t)(0x6000 | F_P << 10 | 4 << 7 | ks[k]), NULL, 0, 0, &s);
		snprintf(nm, sizeof nm, "FMOVE.P FP4=%s {D5=%d} rm=%d", xpool[i].n, (int8_t)(ks[k] << 1) >> 1, c);
		one(nm, (uint16_t)(0x6000 | F_PD << 10 | 4 << 7 | 5 << 4), NULL, 0, (uint16_t)(0xAB00 | ks[k]), &s);
	}
	/* register-direct forms use the same bytes */
	category("FMOVE-Dn");
	for (int i = 0; i < nx; i += 3) {
		St s = base_state(); s.se[2] = xpool[i].se; s.m[2] = xpool[i].m;
		for (int f = 0; f < 4; f++) { static const int ff[] = { F_B, F_W, F_L, F_S };
			snprintf(nm, sizeof nm, "FMOVE.%s FP2=%s,D1", fn[ff[f] == F_B ? 0 : ff[f] == F_W ? 1 : ff[f] == F_L ? 2 : 3], xpool[i].n);
			dn_check(nm, (uint16_t)(0x6000 | ff[f] << 10 | 2 << 7), NULL, 0, &s); }
	}
	{ St s = base_state(); uint8_t b[4];
	  for (int f = 0; f < 4; f++) { static const int ff[] = { F_B, F_W, F_L, F_S }; static const int ln[] = { 1, 2, 4, 4 };
		put32(b, f == 3 ? 0xc0200000u : 0xfffffff3u + (uint32_t)f * 0x1000000u);
		uint8_t in[4]; memcpy(in, ln[f] == 4 ? b : b + 4 - ln[f], 4);
		snprintf(nm, sizeof nm, "FMOVE.%s D1,FP3", fn[ff[f] == F_B ? 0 : ff[f] == F_W ? 1 : ff[f] == F_L ? 2 : 3]);
		dn_check(nm, (uint16_t)(0x4000 | ff[f] << 10 | 3 << 7), in, ln[f], &s); } }
	/* exceptions: enabled OPERR/DZ/OVFL/UNFL/INEX raise Exception(); a second instruction is blocked */
	category("exceptions");
	{ static const uint32_t en[] = { 0x2000, 0x0400, 0x1000, 0x0800, 0x0200, 0x4000, 0x8000 };
	  for (unsigned e = 0; e < sizeof en / sizeof *en; e++) {
		St s = base_state(); s.fpcr = en[e]; s.se[2] = 0x4008; s.m[2] = 0x9c40000000000000ull;     /* 20000 */
		uint8_t b[12]; Step st[2]; memset(st, 0, sizeof st);
		snprintf(nm, sizeof nm, "FMOVE.B 20000 en=%04x then FMOVE.W", en[e]);
		st[0].cmd = (uint16_t)(0x6000 | F_B << 10 | 2 << 7);
		st[1].cmd = (uint16_t)(0x6000 | F_W << 10 | 2 << 7);
		run_check(nm, st, 2, &s);
		/* FDIV by zero, FMUL overflow, FSQRT(-1), inexact */
		xbytes(b, 0, 0, 0); memset(&st[1], 0, sizeof st[1]);
		memcpy(st[0].in, b, 12); st[0].in_len = 12; st[0].cmd = (uint16_t)(0x4000 | F_X << 10 | 0 << 7 | 0x20);
		snprintf(nm, sizeof nm, "FDIV.X #0,FP0 en=%04x then FMOVE.L FP0", en[e]); st[1].cmd = (uint16_t)(0x6000 | F_L << 10);
		run_check(nm, st, 2, &s);
		xbytes(st[0].in, 0x7ffe, 0xffffffffffffffffull, 0); st[0].cmd = (uint16_t)(0x4000 | F_X << 10 | 0 << 7 | 0x23);
		snprintf(nm, sizeof nm, "FMUL.X huge,FP0 en=%04x", en[e]); st[1].cmd = (uint16_t)(0x6000 | F_X << 10);
		s.se[0] = 0x7ffe; s.m[0] = 0xffffffffffffffffull; run_check(nm, st, 2, &s);
		xbytes(st[0].in, 0xbfff, 0x8000000000000000ull, 0); st[0].cmd = (uint16_t)(0x4000 | F_X << 10 | 0 << 7 | 0x04);
		snprintf(nm, sizeof nm, "FSQRT.X -1,FP0 en=%04x", en[e]); st[1].cmd = (uint16_t)(0x6000 | F_X << 10);
		run_check(nm, st, 2, &s);
	  } }
}

static void t_fmovecr(void)
{
	char nm[64];
	category("FMOVECR");
	for (int off = 0; off < 0x80; off++) {
		if (off > 0x3f && off != 0x40 && off != 0x7f && off != 0x45) continue;
		St s = base_state(); one((snprintf(nm, sizeof nm, "FMOVECR #$%02x,FP1", off), nm), (uint16_t)(0x5c00 | 1 << 7 | off), NULL, 0, 0, &s);
		s.fpcr = 0x10 | 0x20; one((snprintf(nm, sizeof nm, "FMOVECR #$%02x,FP6 RP/S?", off), nm), (uint16_t)(0x5c00 | 6 << 7 | off), NULL, 0, 0, &s);
		s.fpcr = 0x80 | 0x10; one((snprintf(nm, sizeof nm, "FMOVECR #$%02x,FP7 RZ/D", off), nm), (uint16_t)(0x5c00 | 7 << 7 | off), NULL, 0, 0, &s);
	}
}

static void t_ctrl(void)
{
	char nm[80]; uint8_t b[12];
	category("FMOVE-ctrl");
	static const char *cn[8] = { "FPIAR", "FPIAR(m=1)", "FPSR", "FPSR,FPIAR", "FPCR", "FPCR,FPIAR", "FPCR,FPSR", "FPCR,FPSR,FPIAR" };
	for (int m = 0; m < 8; m++) {
		St s = base_state(); s.fpcr = 0x00f0 | 0x3400; s.fpsr = 0x0f0000f8 | 0x4000; s.fpiar = 0xCAFEF00Du;
		snprintf(nm, sizeof nm, "FMOVEM %s,<ea>", cn[m]);
		one(nm, (uint16_t)(0xA000 | m << 10), NULL, 0, 0, &s);
		static const uint32_t in[3] = { 0x0000ffffu, 0xfffffff8u, 0x13572468u };
		int n = pop8((unsigned)m) ? pop8((unsigned)m) : 1;
		for (int i = 0; i < n; i++) put32(b + 4 * i, in[i] ^ (uint32_t)(m * 0x01010101u));
		snprintf(nm, sizeof nm, "FMOVEM <ea>,%s", cn[m]);
		one(nm, (uint16_t)(0x8000 | m << 10), b, 4 * n, 0, &s);
	}
	/* FPSR with condition-code / exception bits, then read back */
	for (int i = 0; i < 6; i++) {
		static const uint32_t v[] = { 0x01000000, 0x04000000, 0x08000000, 0x02000000, 0x0f00ff00, 0xffffffff };
		St s = base_state(); put32(b, v[i]);
		snprintf(nm, sizeof nm, "FMOVE.L #%08x,FPSR", v[i]); one(nm, (uint16_t)(0x8000 | 2 << 10), b, 4, 0, &s);
		snprintf(nm, sizeof nm, "FMOVE.L #%08x,FPCR", v[i]); one(nm, (uint16_t)(0x8000 | 4 << 10), b, 4, 0, &s);
	}
	/* register-direct (Dn) */
	{ St s = base_state(); s.fpcr = 0x3410; s.fpsr = 0x0a0000f0; s.fpiar = 0x1234;
	  for (int m = 1; m < 8; m = (m == 4 ? 8 : m * 2)) {
		snprintf(nm, sizeof nm, "FMOVE %s,D1", cn[m]); dn_check(nm, (uint16_t)(0xA000 | m << 10), NULL, 0, &s);
		put32(b, 0x0000cafe); snprintf(nm, sizeof nm, "FMOVE D1,%s", cn[m]); dn_check(nm, (uint16_t)(0x8000 | m << 10), b, 4, &s);
	  } }
}

static void t_movem(void)
{
	char nm[96]; uint8_t b[FPE_MAXIO]; static const unsigned masks[] = { 0x00, 0x01, 0x80, 0x81, 0xff, 0xa5, 0x5a, 0x0f, 0xf0, 0x24 };
	category("FMOVEM.X");
	for (unsigned i = 0; i < sizeof masks / sizeof *masks; i++) {
		unsigned mk = masks[i]; int n = pop8(mk);
		St s = base_state();
		for (int r = 0; r < 8; r++) { s.se[r] = (uint16_t)(0x3ff0 + r); s.m[r] = 0x8000000000000000ull | ((uint64_t)(r + 1) * 0x0101010101010101ull >> 1); }
		for (int mode = 0; mode < 4; mode++) {
			const char *mn[] = { "static pre", "dyn pre", "static post", "dyn post" };
			uint16_t cmdlist = (mode & 1) ? (uint16_t)(3 << 4) : (uint16_t)mk;
			uint16_t aux = (mode & 1) ? (uint16_t)(0x5500 | mk) : 0;
			snprintf(nm, sizeof nm, "FMOVEM.X FP->mem %s mask=%02x", mn[mode], mk);
			one(nm, (uint16_t)(0xE000 | mode << 11 | cmdlist), NULL, 0, aux, &s);
			for (int r = 0; r < n; r++) xbytes(b + 12 * r, (uint16_t)(0x4000 + r * 3 + (r & 1) * 0x8000), 0x8000000000000000ull | (0x1111111111111111ull * (uint64_t)(r + 1) >> 1), 0);
			snprintf(nm, sizeof nm, "FMOVEM.X mem->FP %s mask=%02x", mn[mode], mk);
			one(nm, (uint16_t)(0xC000 | mode << 11 | cmdlist), b, 12 * n, aux, &s);
		}
	}
	/* special values through FMOVEM (no conversion, bit exact) */
	{ St s = base_state(); int k = 0;
	  for (int i = 0; i < nx && k < 8; i += 3, k++) xbytes(b + 12 * k, xpool[i].se, xpool[i].m, 0);
	  one("FMOVEM.X mem->FP specials (post)", (uint16_t)(0xC000 | 2 << 11 | 0xff), b, 12 * 8, 0, &s);
	  s.fpcr = 0x00000030; one("FMOVEM.X FP->mem RP post", (uint16_t)(0xE000 | 2 << 11 | 0xff), NULL, 0, 0, &s); }
	/* dynamic register list taken from AUX's low byte only (high byte garbage, bit 7 set) */
	{ St s = base_state(); one("FMOVEM.X dyn post aux=ffa7", (uint16_t)(0xE000 | 3 << 11 | 2 << 4), NULL, 0, 0xffa7, &s); }
}

static void t_arith(void)
{
	char nm[96]; uint8_t b[12];
	static const struct { const char *n; int op; } ops[] = { { "FADD", 0x22 }, { "FMUL", 0x23 }, { "FDIV", 0x20 }, { "FSQRT", 0x04 }, { "FSIN", 0x0e },
	                                                        { "FSUB", 0x28 }, { "FCMP", 0x38 }, { "FABS", 0x18 }, { "FNEG", 0x1a }, { "FINT", 0x01 } };
	category("arith");
	static const int vals[] = { 2, 4, 9, 0, 22, 12, 13, 18, 3 };
	for (unsigned o = 0; o < sizeof ops / sizeof *ops; o++) for (unsigned v = 0; v < sizeof vals / sizeof *vals; v++) for (int c = 0; c < 3; c++) {
		St s = base_state(); s.fpcr = fpcr_var[c == 0 ? 0 : c == 1 ? 4 : 8];
		xbytes(b, xpool[vals[v]].se, xpool[vals[v]].m, 0);
		snprintf(nm, sizeof nm, "%s.X %s,FP2 (%s)", ops[o].n, xpool[vals[v]].n, fpcr_nm[c == 0 ? 0 : c == 1 ? 4 : 8]);
		one(nm, (uint16_t)(0x4000 | F_X << 10 | 2 << 7 | ops[o].op), b, 12, 0, &s);
		snprintf(nm, sizeof nm, "%s FP0->FP2 (FP0=%s)", ops[o].n, xpool[vals[v]].n);
		s.se[0] = xpool[vals[v]].se; s.m[0] = xpool[vals[v]].m;
		one(nm, (uint16_t)(0x0000 | 0 << 10 | 2 << 7 | ops[o].op), NULL, 0, 0, &s);
	}
	{ St s = base_state(); put32(b, 7); one("FADD.L #7,FP2", (uint16_t)(0x4000 | F_L << 10 | 2 << 7 | 0x22), b, 4, 0, &s);
	  put32(b, 0x40490fdb); one("FMUL.S #pi,FP0", (uint16_t)(0x4000 | F_S << 10 | 0 << 7 | 0x23), b, 4, 0, &s);
	  put64(b, 0x4000000000000000ull); one("FDIV.D #2.0,FP2", (uint16_t)(0x4000 | F_D << 10 | 2 << 7 | 0x20), b, 8, 0, &s);
	  b[0] = 5; one("FADD.B #5,FP0", (uint16_t)(0x4000 | F_B << 10 | 0 << 7 | 0x22), b, 1, 0, &s);
	  b[0] = 0; b[1] = 9; one("FSQRT.W #9,FP1", (uint16_t)(0x4000 | F_W << 10 | 1 << 7 | 0x04), b, 2, 0, &s);
	  /* undefined opmodes */
	  one("opmode $7f (undefined)", 0x0000 | 0x7f, NULL, 0, 0, &s);
	  one("class 1 (undefined)", 0x2000, NULL, 0, 0, &s);
	  one("FMOVECR fmt!=X?", 0x5c00 | 0x40, NULL, 0, 0, &s);
	  s.fpsr = 0x01000000; one("FADD NaN cc then FADD", (uint16_t)(0x0022 | 1 << 10), NULL, 0, 0, &s); }
	category("cond");
	for (int cc = 0; cc < 32; cc++) for (int cs = 0; cs < 16; cs++) {
		St s = base_state(); s.fpsr = (uint32_t)cs << 24; int a, e;
		set_state(&s); e = fpe_cond(cc); fpe_clear_exception();
		set_state(&s); fpe_shim_clear_events(); a = fpp_cond(cc); fpe_clear_exception();
		tally(a == e);
		if (a != e) printf("[FAIL] cond cc=%d fpsr=%x exp %d got %d\n", cc, cs, a, e);
	}
	printf("  (cond: 512 predicate evaluations through fpe_cond vs fpp_cond)\n");
}

/* ---------------------------------------------------------------- fuzz */
static uint64_t rng = 0x9e3779b97f4a7c15ull;
static uint64_t rnd(void) { rng ^= rng << 13; rng ^= rng >> 7; rng ^= rng << 17; return rng; }

static void t_fuzz(int n)
{
	int quiet_save = quiet, ok0 = n_ok, f0 = n_fail; char nm[64];
	category("fuzz");
	quiet = 1;
	for (int i = 0; i < n; i++) {
		St s = base_state(); uint8_t b[FPE_MAXIO]; uint16_t cmd; int len = 0; uint16_t aux = (uint16_t)rnd();
		for (int r = 0; r < 8; r++) { s.se[r] = (uint16_t)rnd(); if ((rnd() & 3) == 0) s.se[r] = (uint16_t)(rnd() & 0x8000) | 0x3fff; s.m[r] = rnd() | ((rnd() & 3) ? (1ull << 63) : 0); }
		s.fpcr = (uint32_t)(rnd() & 0xfff0) & ((rnd() & 1) ? 0x00f0 : 0xffff); s.fpsr = (uint32_t)rnd() & 0x0ffffff8;
		switch (rnd() % 5) {
		case 0: { int f = (int)(rnd() % 7); static const int fm[] = { F_B, F_W, F_L, F_S, F_D, F_X, F_P }; static const int ln[] = { 1, 2, 4, 4, 8, 12, 12 };
		          len = ln[f]; for (int k = 0; k < len; k++) b[k] = (uint8_t)rnd(); if (fm[f] == F_X && (rnd() & 1)) { b[2] = b[3] = 0; }
		          if (fm[f] == F_P) { for (int k = 4; k < 12; k++) b[k] = (uint8_t)((rnd() % 10) << 4 | rnd() % 10); b[0] &= 0xcf; b[1] = 0x00 | (uint8_t)((rnd() % 10) << 4 | rnd() % 10); b[2] = 0; b[3] = (uint8_t)(rnd() % 10); if (rnd() % 8 == 0) b[0] = 0x7f, b[1] = 0xff; }
		          cmd = (uint16_t)(0x4000 | fm[f] << 10 | (rnd() & 7) << 7); static const int opm[] = { 0x00, 0x22, 0x23, 0x20, 0x04, 0x0e, 0x28, 0x18, 0x1a, 0x38, 0x0f, 0x1d, 0x01, 0x03 };
		          cmd |= (uint16_t)opm[rnd() % (sizeof opm / sizeof *opm)]; break; }
		case 1: { int f = (int)(rnd() % 8); cmd = (uint16_t)(0x6000 | f << 10 | (rnd() & 7) << 7 | (rnd() & 0x7f)); if (f == 7) cmd = (uint16_t)((cmd & ~0x70) | (rnd() & 7) << 4); break; }
		case 2: { int mode = (int)(rnd() & 3); unsigned mk = (unsigned)rnd() & 0xff; int nn = pop8(mk); cmd = (uint16_t)(0xE000 | mode << 11 | ((mode & 1) ? (rnd() & 7) << 4 : mk)); if (mode & 1) aux = (uint16_t)((aux & 0xff00) | mk); (void)nn; break; }
		case 3: { int mode = (int)(rnd() & 3); unsigned mk = (unsigned)rnd() & 0xff; int nn = pop8((mode & 1) ? aux : mk); cmd = (uint16_t)(0xC000 | mode << 11 | ((mode & 1) ? (rnd() & 7) << 4 : mk)); len = 12 * nn; for (int k = 0; k < len; k++) b[k] = (uint8_t)rnd(); break; }
		default: { cmd = (uint16_t)(0x5c00 | (rnd() & 7) << 7 | (rnd() & 0x7f)); break; }
		}
		snprintf(nm, sizeof nm, "fuzz #%d cmd=%04x", i, cmd);
		one(nm, cmd, b, len, aux, &s);
	}
	quiet = quiet_save;
	printf("  fuzz: %d random instructions, %d golden comparisons ok, %d fail\n", n, n_ok - ok0, n_fail - f0);
}

int main(int argc, char **argv)
{
	for (int i = 1; i < argc; i++) { if (!strcmp(argv[i], "-v")) verbose = 1; else if (!strcmp(argv[i], "-q")) quiet = 1; }
	fpe_reset();
	{ uint16_t se; uint64_t m; int bad = 0;                 /* reset state: FPn = NaN, control regs 0 */
	  for (int i = 0; i < 8; i++) { fpe_get_fp(i, &se, &m); if (se != 0x7fff || m != 0xffffffffffffffffull) bad = 1; }
	  if (fpe_fpcr() || fpe_fpsr() || fpe_fpiar()) bad = 1;
	  printf("[%s] reset: FP0-7 = NaN (7fff:ffffffffffffffff), FPCR=FPSR=FPIAR=0\n", bad ? "FAIL" : " ok "); if (bad) n_fail++; else n_ok++; }
	init_pool();
	t_fmove_in(); t_fmove_out(); t_fmovecr(); t_ctrl(); t_movem(); t_arith(); t_fuzz(3000);
	category("");
	printf("\nchecks: %d ok, %d fail\n%s\n", n_ok, n_fail, n_fail ? "FAIL" : "PASS");
	return n_fail ? 1 : 0;
}
