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
typedef struct { uint16_t se[8]; uint64_t m[8]; uint32_t fpcr, fpsr, fpiar; int pend; int fs; } St;
typedef struct { uint16_t cmd; uint8_t in[FPE_MAXIO]; int in_len; uint16_t aux; } Step;
typedef struct { int flags; uint32_t vec; int out_len; uint8_t out[FPE_MAXIO]; int st; uint32_t pvec; } Res;
/* optional independent expectations for the next run_check (-1 = unchecked) */
static int want_fl[3] = { -1, -1, -1 }, want_st[3] = { -1, -1, -1 };
static int want_vec[3] = { -1, -1, -1 };
static int64_t want_fpiar = -1;
static void clear_want(void) { for (int i = 0; i < 3; i++) want_fl[i] = want_st[i] = want_vec[i] = -1; want_fpiar = -1; }

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
	s->fs = (int)(regs.fpu_state | (regs.fpu_exp_state << 4));
}

static int st_eq(const St *a, const St *b)
{
	for (int i = 0; i < 8; i++) if (a->se[i] != b->se[i] || a->m[i] != b->m[i]) return 0;
	return a->fpcr == b->fpcr && a->fpsr == b->fpsr && a->fpiar == b->fpiar && a->pend == b->pend && a->fs == b->fs;
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
		o += (size_t)snprintf(d + o, n - o, "[fl=%d v=%u st=%x out(%d)=%.40s%s]", r[k].flags, r[k].vec, r[k].st, r[k].out_len, h,
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
	if ((s->cmd >> 13) >= 4) {         /* UM rule: pending exception is not tested by FMOVEM / control moves */
		uae_u32 pend = regs.fp_exp_pend;
		regs.fp_exp_pend = 0;
		fpuop_arithmetic(gm_opc[gm], s->cmd);
		if (!regs.fp_exp_pend) regs.fp_exp_pend = pend;
	} else {
		fpuop_arithmetic(gm_opc[gm], s->cmd);
	}
	r->st = fpe_status(); r->pvec = fpe_pending_vector();
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
static uint32_t fpe_vector_of(const Res *r) { return r->flags ? r->vec : r->pvec; }

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
		er[k].st = fpe_status(); er[k].pvec = fpe_pending_vector();
		er[k].vec = fpe_vector();
		if (!er[k].flags) er[k].vec = 0;
	}
	get_state(&eg);
	int wok = 1;
	for (int k = 0; k < ns; k++) {
		if (want_fl[k] >= 0 && er[k].flags != want_fl[k]) wok = 0;
		if (want_vec[k] >= 0 && fpe_vector_of(&er[k]) != (uint32_t)want_vec[k]) wok = 0;
		if (want_st[k] >= 0 && er[k].st != want_st[k]) wok = 0;
	}
	if (want_fpiar >= 0 && eg.fpiar != (uint32_t)want_fpiar) wok = 0;
	clear_want();

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
			if (gr[k].st != er[k].st || gr[k].pvec != er[k].pvec) ok = 0;
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
		if (!wok) ok = 0;
		tally(ok);
		if (!ok || verbose || !quiet) {
			char tag[48];
			snprintf(tag, sizeof tag, "%s%s%s", gm_name[gm], a1ok ? "" : " A1!", wok ? "" : " WANT!");
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


/* ---------------------------------------------------------------- milestone 4 */
static void expect(const char *name, int cond, const char *detail)
{
	tally(cond);
	if (!cond || !quiet) printf("[%s] %-9s %-52s %s\n", cond ? " ok " : "FAIL", cat_name, name, detail ? detail : "");
}

#define E_PEND 0x01
#define E_MID  0x02
#define E_PRE  0x04
#define E_EN   0x08
static Step mkstep(uint16_t cmd, uint16_t se, uint64_t m, int has_x)
{
	Step s; memset(&s, 0, sizeof s); s.cmd = cmd;
	if (has_x) { xbytes(s.in, se, m, 0); s.in_len = 12; }
	return s;
}

static void t_exc4(void)
{
	char nm[128];
	category("exc-pend");
	/* arithmetic that raises each enabled exception; then a 2nd arithmetic (must report it pre-instruction,
	 * not executing), then FMOVEM.X out (must execute: UM 6.4.2.2) */
	static const struct { const char *n; uint32_t en; uint16_t fse; uint64_t fm; uint16_t sse; uint64_t sm; int op; int vec; } c[] = {
		{ "SNAN  FADD sNaN",      0x4000, 0x3fff, 0x8000000000000000ull, 0x7fff, 0x8000000000000001ull, 0x22, 54 },
		{ "OPERR FSQRT -1",       0x2000, 0x3fff, 0x8000000000000000ull, 0xbfff, 0x8000000000000000ull, 0x04, 52 },
		{ "OVFL  FMUL huge*huge", 0x1000, 0x7ffe, 0xffffffffffffffffull, 0x7ffe, 0xffffffffffffffffull, 0x23, 53 },
		{ "UNFL  FMUL min*min",   0x0800, 0x0001, 0x8000000000000000ull, 0x0001, 0x8000000000000000ull, 0x23, 51 },
		{ "DZ    FDIV 1/0",       0x0400, 0x3fff, 0x8000000000000000ull, 0x0000, 0,                     0x20, 50 },
		{ "INEX2 FDIV 1/3",       0x0200, 0x3fff, 0x8000000000000000ull, 0x4000, 0xc000000000000000ull, 0x20, 49 },
		{ "prio  FMUL huge all",  0xff00, 0x7ffe, 0xffffffffffffffffull, 0x7ffe, 0xffffffffffffffffull, 0x23, 53 },
		{ "prio  FSQRT -1 all",   0xff00, 0x3fff, 0x8000000000000000ull, 0xbfff, 0x8000000000000000ull, 0x04, 52 },
		{ "prio  FDIV 0/0 all",   0xff00, 0x0000, 0,                     0x0000, 0,                     0x20, 52 },
		{ "prio  FADD sNaN all",  0xff00, 0x3fff, 0x8000000000000000ull, 0x7fff, 0x8000000000000001ull, 0x22, 54 },
		{ "prio  FDIV 1/0 all",   0xff00, 0x3fff, 0x8000000000000000ull, 0x0000, 0,                     0x20, 50 },
		{ "prio  FMUL min*min all", 0xff00, 0x0001, 0x8000000000000000ull, 0x0001, 0x8000000000000000ull, 0x23, 51 },
		{ "prio  FDIV 1/3 all",   0xff00, 0x3fff, 0x8000000000000000ull, 0x4000, 0xc000000000000000ull, 0x20, 49 },
		{ "none  FDIV 1/0 disabled", 0x0000, 0x3fff, 0x8000000000000000ull, 0x0000, 0,                  0x20, 0 },
		{ "none  FDIV 1/0 BSUN-only", 0x8000, 0x3fff, 0x8000000000000000ull, 0x0000, 0,                 0x20, 0 },
	};
	for (unsigned i = 0; i < sizeof c / sizeof *c; i++) {
		for (int v = 0; v < 3; v++) {
			St s = base_state(); Step st[3];
			s.fpcr = c[i].en; s.fpiar = 0xCAFEF00Du; s.se[0] = c[i].fse; s.m[0] = c[i].fm;
			st[0] = mkstep((uint16_t)(0x4000 | F_X << 10 | 0 << 7 | c[i].op), c[i].sse, c[i].sm, 1);
			/* step 1 variants: arithmetic / FMOVE.L out / FMOVEM.L control out */
			if (v == 0) st[1] = mkstep((uint16_t)(0x0000 | 1 << 10 | 2 << 7 | 0x22), 0, 0, 0);       /* FADD FP1,FP2 */
			else if (v == 1) st[1] = mkstep((uint16_t)(0x6000 | F_L << 10 | 1 << 7), 0, 0, 0);       /* FMOVE.L FP1 */
			else st[1] = mkstep((uint16_t)(0xA000 | 7 << 10), 0, 0, 0);                                 /* FMOVEM.L FPCR/FPSR/FPIAR */
			st[2] = mkstep((uint16_t)(0xE000 | 2 << 11 | 0x40), 0, 0, 0);                              /* FMOVEM.X FP1 */
			int en = (c[i].en & 0x7f00) ? E_EN : 0, vec = c[i].vec;
			want_fl[0] = 0; want_st[0] = vec ? (E_PEND | en) : en; want_vec[0] = vec ? vec : 0;
			if (vec && v < 2) { want_fl[1] = FPE_EXCEPTION; want_st[1] = E_PEND | E_PRE | en; want_vec[1] = vec; }
			else { want_fl[1] = 0; want_st[1] = vec ? (E_PEND | en) : en; }
			want_fl[2] = 0; want_st[2] = vec ? (E_PEND | en) : en;
			want_fpiar = vec || (c[i].en & 0x7f00) ? 0x12340 : 0xCAFEF00D;
			if (vec == 0 && (c[i].en & 0x7f00)) want_fpiar = 0x12340;
			snprintf(nm, sizeof nm, "%s en=%04x -> %s", c[i].n, c[i].en, v == 0 ? "FADD" : v == 1 ? "FMOVE.L out" : "FMOVEM.L ctrl");
			run_check(nm, st, 3, &s);
		}
	}
	/* conditional: pending exception reported first; BSUN */
	{ St s = base_state(); int r;
	  s.fpcr = 0x0400; s.fpsr = 0x00000400; set_state(&s); regs.fp_exp_pend = 50;
	  r = fpe_cond(1);
	  expect("FBcc with DZ pending: pre-instruction exception 50", r == -2 && fpe_vector() == 50 && (fpe_status() & E_PRE), NULL);
	  fpe_clear_exception(); r = fpe_cond(1); expect("FBcc after clear: evaluates (cc=1 EQ false)", r == 0, NULL);
	  s = base_state(); s.fpsr = 0x01000000; s.fpcr = 0x8000; set_state(&s); r = fpe_cond(0x11);
	  expect("FBcc NaN + BSUN enabled: BSUN 48 raised", r == -2 && fpe_vector() == 48 && (fpe_status() & E_PRE), NULL);
	  s = base_state(); s.fpsr = 0x01000000; s.fpcr = 0; set_state(&s); r = fpe_cond(0x11);
	  expect("FBcc NaN BSUN disabled: no exception, BSUN in FPSR", r >= 0 && (fpe_fpsr() & 0x8000), NULL);
	  s = base_state(); s.fpsr = 0x04000000; set_state(&s); regs.fpu_state = 0; r = fpe_cond(1);
	  expect("FBcc EQ with Z set true, null->idle", r == 1 && regs.fpu_state == 1, NULL);
	}
	category("exc-mid");
	{ static const struct { const char *n; int fmt; uint32_t en; uint16_t se; uint64_t m; int vec; } mv[] = {
		{ "FMOVE.B 20000 OPERR", F_B, 0x2000, 0x400c, 0x9c40000000000000ull, 52 },
		{ "FMOVE.W 1e10 OPERR",  F_W, 0x2000, 0x4021, 0x9502f90000000000ull, 52 },
		{ "FMOVE.S huge OVFL",   F_S, 0x1000, 0x7ffe, 0xffffffffffffffffull, 53 },
		{ "FMOVE.S tiny UNFL",   F_S, 0x0800, 0x3000, 0x8000000000000000ull, 51 },
		{ "FMOVE.L 1.5 INEX2",   F_L, 0x0200, 0x3fff, 0xc000000000000000ull, 49 },
		{ "FMOVE.D 1/3 INEX2",   F_D, 0x0200, 0x3ffd, 0xaaaaaaaaaaaaaaabull, 49 },
		{ "FMOVE.P 1/3 INEX2",   F_P, 0x0200, 0x3ffd, 0xaaaaaaaaaaaaaaabull, 49 },
		{ "FMOVE.S sNaN SNAN",   F_S, 0x4000, 0x7fff, 0x8000000000000001ull, 54 },
		{ "FMOVE.X pi (none)",   F_X, 0x7f00, 0x4000, 0xc90fdaa22168c235ull, 0 },
		{ "FMOVE.B 20000 disabled", F_B, 0x0000, 0x400c, 0x9c40000000000000ull, 0 },
	};
	  for (unsigned i = 0; i < sizeof mv / sizeof *mv; i++) {
		St s = base_state(); Step st[1];
		s.fpcr = mv[i].en; s.se[2] = mv[i].se; s.m[2] = mv[i].m; s.fpiar = 0xCAFEF00Du;
		st[0] = mkstep((uint16_t)(0x6000 | mv[i].fmt << 10 | 2 << 7 | (mv[i].fmt == F_P ? 17 : 0)), 0, 0, 0);
		int en = (mv[i].en & 0x7f00) ? E_EN : 0;
		want_fl[0] = mv[i].vec ? FPE_EXCEPTION : 0; want_vec[0] = mv[i].vec;
		want_st[0] = mv[i].vec ? (E_PEND | E_MID | en) : en;
		want_fpiar = en ? 0x12340 : 0xCAFEF00D;
		snprintf(nm, sizeof nm, "%s en=%04x (result stored, mid-instruction)", mv[i].n, mv[i].en);
		run_check(nm, st, 1, &s);
	  } }
	category("FPIAR");
	{ static const uint32_t ens[] = { 0x0000, 0x8000, 0x0100, 0x0200, 0x0400, 0x7f00 };
	  static const struct { const char *n; uint16_t cmd; int in; int upd; } fi[] = {
		{ "FADD.X", 0x4000 | F_X << 10 | 0x22, 1, 1 }, { "FMOVE.X in", 0x4000 | F_X << 10, 1, 1 },
		{ "FMOVECR", 0x5c00 | 3 << 7, 0, 1 },          { "FMOVE.X out", 0x6000 | F_X << 10, 0, 1 },
		{ "FMOVEM.X out", 0xE000 | 2 << 11 | 0x80, 0, 0 }, { "FMOVEM.L out", 0xA000 | 7 << 10, 0, 0 },
		{ "FMOVEM.X in", 0xC000 | 2 << 11 | 0x80, 2, 0 }, { "FMOVEM.L in", 0x8000 | 1 << 10, 3, 0 } };
	  for (unsigned i = 0; i < sizeof ens / sizeof *ens; i++) for (unsigned k = 0; k < sizeof fi / sizeof *fi; k++) {
		St s = base_state(); Step st[1]; s.fpcr = ens[i]; s.fpiar = 0xCAFEF00Du;
		st[0] = mkstep(fi[k].cmd, 0x3fff, 0x8000000000000000ull, fi[k].in == 1 || fi[k].in == 2);
		if (fi[k].in == 3) { st[0].in_len = 4; put32(st[0].in, 0x1111); }
		want_fpiar = ((ens[i] & 0x7f00) && fi[k].upd) ? 0x12340 : (fi[k].in == 3 ? 0x1111 : 0xCAFEF00D);
		if (fi[k].in == 3) want_fpiar = 0x1111;
		snprintf(nm, sizeof nm, "%s fpcr=%04x", fi[k].n, ens[i]);
		run_check(nm, st, 1, &s);
	  } }
}

/* ---- FSAVE / FRESTORE */
static void force_state(int null) { regs.fpu_state = null ? 0 : 1; regs.fpu_exp_state = 0; }
static void pr_frame(char *d, size_t n, const uint8_t *f, int len) { char h[2 * FPE_MAXIO + 8]; hex(h, sizeof h, f, len); snprintf(d, n, "len=%d %.60s%s", len, h, strlen(h) > 60 ? ".." : ""); }

static void save_case(const char *name, const St *init, int null, const Step *pre, int npre)
{
	uint8_t ef[FPE_MAXIO]; int el, efl, epend, est; St eg, gg; char da[200], nm[160];
	/* engine */
	set_state(init); force_state(null);
	for (int k = 0; k < npre; k++) { uint8_t o[FPE_MAXIO]; int ol; fpe_exec(pre[k].cmd, pre[k].in, pre[k].in_len, pre[k].aux, IADDR, o, &ol); }
	epend = regs.fp_exp_pend ? 1 : 0;
	int enull = regs.fpu_state == 0;
	efl = fpe_save(ef, &el); est = fpe_status(); get_state(&eg);
	int want_len = enull ? 4 : 60;
	static const char *gn[] = { "(A1)", "-(A1)", "d16(A1)" };
	static const uint16_t gop[] = { 0xF311, 0xF321, 0xF329 };
	for (int g = 0; g < 3; g++) {
		uint16_t is[2]; int nis = 0, ok = 1; uint8_t eimg[GSIZE];
		set_state(init); force_state(null);
		for (int k = 0; k < npre; k++) { Res r; int a; golden_step(&pre[k], GM_IND, &r, &a); }
		int gpend = regs.fp_exp_pend ? 1 : 0;
		memset(gmem, 0xA5, sizeof gmem); memcpy(gmem0, gmem, sizeof gmem);
		regs.regs[9] = g == 1 ? BASE + (uint32_t)want_len : (g == 2 ? BASE - 0x20 : BASE);
		if (g == 2) is[nis++] = 0x20;
		fpe_shim_use_memory(gmem, GSIZE, is, nis); fpe_shim_set_pc(IADDR + 2); fpe_shim_clear_events();
		fpuop_save(gop[g]);
		fpe_shim_use_buffer();
		get_state(&gg);
		/* expected image: Hatari's frame; with an exception pending the UM wants BIU bit 27 = 0 (Hatari writes
		 * 0x20000000 instead of 0x08000000 for an exception-state frame) */
		memcpy(eimg, gmem0, sizeof eimg);
		memcpy(eimg + BASE, gmem + BASE, (size_t)want_len);
		if (gpend && !enull) {
			uint32_t biu = (uint32_t)eimg[BASE + 56] << 24 | eimg[BASE + 57] << 16 | eimg[BASE + 58] << 8 | eimg[BASE + 59];
			biu = (biu & ~0x08000000u) | 0x20000000u; put32(eimg + BASE + 56, biu);
		}
		/* the CCR image holds the instruction opcode (F290 synthetic vs F291.. real EA): compare without its EA field */
		{ uint8_t a[FPE_MAXIO], b[FPE_MAXIO]; memcpy(a, ef, (size_t)want_len); memcpy(b, eimg + BASE, (size_t)want_len);
		  if (!enull && gpend) { a[5] &= (uint8_t)~0x3f; b[5] &= (uint8_t)~0x3f; }
		  if (el != want_len || efl != 0 || memcmp(a, b, (size_t)want_len)) ok = 0; }
		{ uint8_t chk[GSIZE]; memcpy(chk, gmem0, sizeof chk); memcpy(chk + BASE, ef, (size_t)el < sizeof chk - BASE ? (size_t)el : 0);
		  if (memcmp(chk + BASE + el, gmem + BASE + el, 16)) ok = 0; }          /* nothing written past the frame */
		/* state: pending absorbed, exception state cleared; same as golden after its own clear */
		if (eg.pend != 0 || epend != gpend) ok = 0;
		if (!enull) { uint32_t biu = (uint32_t)ef[56] << 24 | ef[57] << 16 | ef[58] << 8 | ef[59];
		              if ((epend && (biu & 0x08000000u)) || (!epend && !(biu & 0x08000000u))) ok = 0; }   /* UM bit 27 rule */
		if (ef[0] != (enull ? 0x00 : 0x1f) || ef[1] != 0x38) ok = 0;
		if (eg.fs != gg.fs || memcmp(eg.se, gg.se, sizeof eg.se) || eg.fpsr != gg.fpsr || eg.fpiar != gg.fpiar) ok = 0;
		(void)est;
		if (!ok && verbose) printf("dbg el=%d efl=%d img=%d eg.pend=%d epend=%d gpend=%d fs=%x/%x\n", el, efl, memcmp(ef, eimg + BASE, (size_t)want_len), eg.pend, epend, gpend, eg.fs, gg.fs);
		pr_frame(da, sizeof da, ef, el);
		snprintf(nm, sizeof nm, "FSAVE %s %s", name, gn[g]);
		tally(ok);
		if (!ok || !quiet) printf("[%s] %-9s %-44s pend=%d %s\n", ok ? " ok " : "FAIL", cat_name, nm, epend, da);
	}
}

static void restore_case(const char *name, const St *init, int null_before, const uint8_t *fr, int len,
                         int golden, int want_fmt, int want_pend)
{
	St eg, gg; int efl, ok = 1; char da[200], nm[160];
	set_state(init); force_state(null_before);
	St before; get_state(&before);
	efl = fpe_restore(fr, len); get_state(&eg);
	int estat = fpe_status();
	if (want_fmt >= 0 && ((efl & FPE_FORMAT) != 0) != (want_fmt != 0)) ok = 0;
	if (want_fmt == 1 && (efl != FPE_FORMAT || fpe_vector() != 14 || !st_eq(&before, &eg))) ok = 0;   /* state unchanged */
	if (want_pend >= 0 && ((estat & E_PEND) ? (int)fpe_pending_vector() : 0) != want_pend) ok = 0;
	if (golden) for (int g = 0; g < 2; g++) {
		memset(gmem, 0xA5, sizeof gmem); memcpy(gmem + BASE, fr, (size_t)len);
		set_state(init); force_state(null_before);
		regs.regs[9] = BASE;
		fpe_shim_use_memory(gmem, GSIZE, NULL, 0); fpe_shim_set_pc(IADDR + 2); fpe_shim_clear_events();
		fpuop_restore(g ? 0xF359 : 0xF351);
		int gfl = (fpe_ev_exc ? FPE_EXCEPTION : 0) | (fpe_ev_unimpl ? FPE_UNIMPL : 0) | (fpe_ev_fmt ? FPE_FORMAT : 0);
		int gst = fpe_status();
		fpe_shim_use_buffer(); get_state(&gg);
		if (gfl != efl || !st_eq(&gg, &eg) || gst != estat) ok = 0;
	}
	pr_frame(da, sizeof da, fr, len);
	snprintf(nm, sizeof nm, "FRESTORE %s", name);
	tally(ok);
	if (!ok || !quiet) printf("[%s] %-9s %-40s fl=%d vec=%u st=%x pendvec=%u fs=%02x %s\n", ok ? " ok " : "FAIL", cat_name, nm, efl,
	                         fpe_vector(), estat, fpe_pending_vector(), eg.fs, da);
}

static void mkframe(uint8_t *f, int ver, int size, uint32_t ccr, uint32_t biu, int total)
{
	memset(f, 0, FPE_MAXIO);
	f[0] = (uint8_t)ver; f[1] = (uint8_t)size;
	put32(f + 4, ccr);
	for (int i = 8; i < total - 4; i++) f[i] = (uint8_t)(i * 7);
	put32(f + total - 4, biu);
}

static void t_saverestore(void)
{
	St s = base_state(); Step pre[2]; uint8_t fr[FPE_MAXIO];
	category("FSAVE");
	s.fpiar = 0xCAFEF00Du;
	save_case("null state", &s, 1, NULL, 0);
	pre[0] = mkstep(0x0000 | 1 << 10 | 2 << 7 | 0x22, 0, 0, 0);
	save_case("idle (after FADD)", &s, 1, pre, 1);
	save_case("idle (state set)", &s, 0, NULL, 0);
	{ static const struct { uint32_t en; uint16_t fse; uint64_t fm; uint16_t sse; uint64_t sm; int op; } pv[] = {
		{ 0x0400, 0x3fff, 0x8000000000000000ull, 0x0000, 0, 0x20 }, { 0x2000, 0x3fff, 0x8000000000000000ull, 0xbfff, 0x8000000000000000ull, 0x04 },
		{ 0x1000, 0x7ffe, 0xffffffffffffffffull, 0x7ffe, 0xffffffffffffffffull, 0x23 }, { 0xff00, 0x3fff, 0x8000000000000000ull, 0x4000, 0xc000000000000000ull, 0x20 } };
	  for (unsigned i = 0; i < 4; i++) { St p = base_state(); char nm[64]; p.fpcr = pv[i].en; p.se[0] = pv[i].fse; p.m[0] = pv[i].fm;
		pre[0] = mkstep((uint16_t)(0x4000 | F_X << 10 | pv[i].op), pv[i].sse, pv[i].sm, 1);
		snprintf(nm, sizeof nm, "exception pending #%u (bit27=0)", i);
		save_case(nm, &p, 0, pre, 1); }
	  /* pending + other state: second FSAVE after absorbing is a plain idle frame */
	  St p = base_state(); p.fpcr = 0x0400; p.se[0] = 0x3fff; p.m[0] = 0x8000000000000000ull;
	  pre[0] = mkstep(0x4000 | F_X << 10 | 0x20, 0, 0, 1);
	  set_state(&p); { uint8_t o[FPE_MAXIO], f1[FPE_MAXIO], f2[FPE_MAXIO]; int ol, l1, l2, a, b;
		fpe_exec(pre[0].cmd, pre[0].in, pre[0].in_len, 0, IADDR, o, &ol);
		a = fpe_save(f1, &l1); b = fpe_save(f2, &l2);
		expect("second FSAVE: no longer pending (bit27=1)", !a && !b && l1 == 60 && l2 == 60 && !(f1[56] & 8) && (f2[56] & 8) && !(fpe_status() & E_PEND), NULL); } }

	category("FRESTORE");
	{ St r = base_state(); r.fpiar = 0xCAFEF00Du;
	  uint8_t nf1[4] = { 0, 0x38, 0, 0 }, nf2[4] = { 0, 0xff, 0xab, 0xcd }, nf3[4] = { 0, 0, 0, 0 };
	  restore_case("null $00380000 (from idle)", &r, 0, nf1, 4, 1, 0, 0);
	  restore_case("null $00ffabcd (from idle)", &r, 0, nf2, 4, 1, 0, 0);
	  restore_case("null $00000000 (from null)", &r, 1, nf3, 4, 1, 0, 0);
	  mkframe(fr, 0x1f, 0x38, 0x12345678, 0x540effff | 0x08000000, 60);
	  restore_case("idle $1F38 bit27=1", &r, 1, fr, 60, 1, 0, 0);
	  mkframe(fr, 0x1f, 0x18, 0x0badf00d, 0x540effff | 0x08000000, 28);
	  restore_case("idle $1F18 (68881) bit27=1", &r, 1, fr, 28, 1, 0, 0);
	  mkframe(fr, 0x1f, 0xd4, 0, 0, 216); restore_case("busy $1FD4 (skipped)", &r, 0, fr, 216, 1, 0, 0);
	  mkframe(fr, 0x1f, 0xb4, 0, 0, 184); restore_case("busy $1FB4 (skipped)", &r, 1, fr, 184, 1, 0, 0);
	  mkframe(fr, 0x1f, 0x20, 0, 0, 40);  restore_case("invalid $1F20", &r, 0, fr, 40, 1, 1, -1);
	  mkframe(fr, 0x1f, 0x00, 0, 0, 40);  restore_case("invalid $1F00", &r, 0, fr, 40, 1, 1, -1);
	  mkframe(fr, 0x55, 0x38, 0, 0, 60);  restore_case("invalid version $55", &r, 0, fr, 60, 0, 1, -1);
	  mkframe(fr, 0x40, 0x38, 0, 0, 60);  restore_case("68040 version $40 (rejected here)", &r, 0, fr, 60, 0, 1, -1);
	  mkframe(fr, 0x41, 0x30, 0, 0, 52);  restore_case("68040 version $41 (rejected here)", &r, 0, fr, 52, 0, 1, -1);
	  /* re-arming: bit 27 = 0 with a matching enabled FPSR/FPCR bit */
	  static const struct { uint32_t fpsr, fpcr; uint32_t biu; int vec; const char *n; } ra[] = {
		{ 0x0400, 0x0400, 0x540effff | 0x20000000, 50, "idle bit27=0, DZ enabled+set -> pending 50" },
		{ 0x0400, 0x0400, 0x540effff | 0x08000000, 0,  "idle bit27=1 -> not pending" },
		{ 0x0000, 0x0400, 0x540effff | 0x20000000, 0,  "idle bit27=0, nothing set -> not pending" },
		{ 0x2400, 0xff00, 0x540effff | 0x20000000, 52, "idle bit27=0, OPERR+DZ -> pending 52" },
		{ 0x4a00, 0xff00, 0x540effff | 0x20000000, 54, "idle bit27=0, SNAN prio -> pending 54" },
		{ 0x0200, 0x0200, 0x540effff,              49, "idle bit27=0 (plain), INEX -> pending 49" } };
	  for (unsigned i = 0; i < sizeof ra / sizeof *ra; i++) { St q = base_state(); q.fpsr = ra[i].fpsr; q.fpcr = ra[i].fpcr;
		mkframe(fr, 0x1f, 0x38, 0, ra[i].biu, 60);
		restore_case(ra[i].n, &q, 0, fr, 60, 1, 0, ra[i].vec); }
	  /* round trip: pending -> FSAVE -> (clear) -> FRESTORE re-arms -> next arithmetic reports it, FMOVEM does not */
	  { St q = base_state(); uint8_t f[FPE_MAXIO], o[FPE_MAXIO]; int l, ol, fl, ok;
		q.fpcr = 0x0400; q.se[0] = 0x3fff; q.m[0] = 0x8000000000000000ull;
		set_state(&q); Step d = mkstep(0x4000 | F_X << 10 | 0x20, 0, 0, 1);
		fpe_exec(d.cmd, d.in, d.in_len, 0, IADDR, o, &ol);
		fpe_save(f, &l);
		ok = !(fpe_status() & E_PEND);
		force_state(0); fpe_clear_exception();
		fl = fpe_restore(f, l);
		ok &= (fl == 0) && (fpe_status() & E_PEND) && fpe_pending_vector() == 50;
		fl = fpe_exec(0xE000 | 2 << 11 | 0x80, NULL, 0, 0, IADDR, o, &ol);
		ok &= (fl == 0) && ol == 12;
		fl = fpe_exec(0x0000 | 1 << 10 | 2 << 7 | 0x22, NULL, 0, 0, IADDR, o, &ol);
		ok &= (fl == FPE_EXCEPTION) && fpe_vector() == 50 && (fpe_status() & E_PRE);
		expect("round trip: pending survives FSAVE/FRESTORE", ok, NULL); } }
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
	t_fmove_in(); t_fmove_out(); t_fmovecr(); t_ctrl(); t_movem(); t_arith(); t_exc4(); t_saverestore(); t_fuzz(3000);
	category("");
	printf("\nchecks: %d ok, %d fail\n%s\n", n_ok, n_fail, n_fail ? "FAIL" : "PASS");
	return n_fail ? 1 : 0;
}
