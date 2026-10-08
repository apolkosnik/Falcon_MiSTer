/* fpe_shim.c - runs Hatari's WinUAE FPU front-end (cpu/fpp.c) without a 68k CPU core.
 *
 * Everything fpp.c expects from the CPU core is stubbed here:
 *   regs / currprefs / changed_prefs       real structs from Hatari's headers, only the FPU parts matter
 *   x_cp_get/put/next accessors            function pointers; the default set maps "address" to an offset in a
 *                                          private 256 byte buffer (see fpe_exec); the test swaps in a fake 68k RAM
 *   m68k_getpc()                           regs.pc (+ equal pc_p/pc_oldp), set per instruction to iaddr+4
 *   Exception(n) / op_illg(op)             recorded, then return (the caller decides what the bridge does)
 *   mmufixup, mmu030_*, savestate, logging dummies so fpp.c links
 * Hatari own files are compiled unmodified.
 */
#include "sysconfig.h"
#include "sysdeps.h"
#include "main.h"
#include "hatari-glue.h"
#include "options_cpu.h"
#include "memory.h"
#include "newcpu.h"
#include "fpp.h"
#include "cpummu030.h"
#include "savestate.h"
#include "log.h"

#include <string.h>

#include "falcon_fpu_engine.h"
#include "fpe_internal.h"

/* ---- CPU core state fpp.c touches ---------------------------------------------------- */
struct regstruct regs;
struct uae_prefs currprefs, changed_prefs;
struct mmufixup mmufixup[2];
uae_u16 mmu030_state[3];
uae_u32 mmu030_data_buffer_out;
uae_u32 mmu030_fmovem_store[2];

int      fpe_ev_unimpl, fpe_ev_exc;
uint32_t fpe_ev_vector, fpe_ev_opcode;

static uae_u8 dummy_code[16];          /* m68k_setpc()/get_real_address() target */

uae_u8 *memory_get_real_address(uaecptr a) { (void)a; return dummy_code; }

/* ---- hooks -------------------------------------------------------------------------- */
/* vectors 48..54 are FPU arithmetic exceptions; 4 (reserved opmode) and 11 (F-line) mean "this command is not an
 * instruction this FPU executes" and are reported as unimplemented */
void Exception(int nr)
{
	if (nr == 4 || nr == 11) fpe_ev_unimpl = 1; else fpe_ev_exc = 1;
	fpe_ev_vector = (uint32_t)nr;
}
void Exception_cpu_oldpc(int nr, uaecptr oldpc) { (void)oldpc; Exception(nr); }
uae_u32 op_illg(uae_u32 opcode)   { fpe_ev_unimpl = 1; fpe_ev_opcode = opcode; fpe_ev_vector = 11; return 0; }
void check_t0_trace(void)         { }
void set_cpu_caches(bool flush)   { (void)flush; }
void fpux_restore(int *v)         { (void)v; }
void Log_Printf(LOGTYPE t, const char *fmt, ...) { (void)t; (void)fmt; }
void fp_init_native(void) { }          /* native-FPU back-end is not built; fpu_mode is always softfloat */

/* savestate helpers are only referenced by restore_fpu()/save_fpu() */
void save_u16(uae_u16 d)  { (void)d; }
void save_u32(uae_u32 d)  { (void)d; }
uae_u16 restore_u16(void) { return 0; }
uae_u32 restore_u32(void) { return 0; }

/* ---- memory accessors ---------------------------------------------------------------- */
#define BUFSZ 256
static uae_u8  fpe_buf[BUFSZ];
static int     buf_rd_end, buf_wr_lo, buf_wr_hi, buf_err;

static uae_u8 *ext_mem; static uae_u32 ext_size;
static const uae_u16 *ext_is; static int ext_is_n, ext_is_pos;

/* default: private buffer */
static int b_ok(uaecptr a, int n) { if (a > BUFSZ || (uaecptr)n > BUFSZ - a) { buf_err = 1; return 0; } return 1; }
static uae_u32 b_get(uaecptr a, int n)
{
	uae_u32 v = 0;
	if (!b_ok(a, n)) return 0;
	if ((int)(a + n) > buf_rd_end) buf_rd_end = (int)(a + n);
	for (int i = 0; i < n; i++) v = (v << 8) | fpe_buf[a + i];
	return v;
}
static void b_put(uaecptr a, int n, uae_u32 v)
{
	if (!b_ok(a, n)) return;
	if ((int)a < buf_wr_lo) buf_wr_lo = (int)a;
	if ((int)(a + n) > buf_wr_hi) buf_wr_hi = (int)(a + n);
	for (int i = n - 1; i >= 0; i--) { fpe_buf[a + i] = (uae_u8)v; v >>= 8; }
}
static uae_u32 bg_b(uaecptr a) { return b_get(a, 1); }
static uae_u32 bg_w(uaecptr a) { return b_get(a, 2); }
static uae_u32 bg_l(uaecptr a) { return b_get(a, 4); }
static void bp_b(uaecptr a, uae_u32 v) { b_put(a, 1, v); }
static void bp_w(uaecptr a, uae_u32 v) { b_put(a, 2, v); }
static void bp_l(uaecptr a, uae_u32 v) { b_put(a, 4, v); }
/* no instruction stream: the 68030 evaluated every EA already */
static uae_u32 b_iw(void) { buf_err = 1; return 0; }
static uae_u32 b_il(void) { buf_err = 1; return 0; }
static uae_u32 REGPARAM3 b_disp(uae_u32 base, int idx) { (void)base; (void)idx; buf_err = 1; return 0; }

/* alternative: caller owned big-endian RAM + instruction stream (golden model in the selftest) */
static uae_u32 m_get(uaecptr a, int n)
{
	uae_u32 v = 0;
	if (a > ext_size || (uaecptr)n > ext_size - a) { buf_err = 1; return 0; }
	for (int i = 0; i < n; i++) v = (v << 8) | ext_mem[a + i];
	return v;
}
static void m_put(uaecptr a, int n, uae_u32 v)
{
	if (a > ext_size || (uaecptr)n > ext_size - a) { buf_err = 1; return; }
	for (int i = n - 1; i >= 0; i--) { ext_mem[a + i] = (uae_u8)v; v >>= 8; }
}
static uae_u32 mg_b(uaecptr a) { return m_get(a, 1); }
static uae_u32 mg_w(uaecptr a) { return m_get(a, 2); }
static uae_u32 mg_l(uaecptr a) { return m_get(a, 4); }
static void mp_b(uaecptr a, uae_u32 v) { m_put(a, 1, v); }
static void mp_w(uaecptr a, uae_u32 v) { m_put(a, 2, v); }
static void mp_l(uaecptr a, uae_u32 v) { m_put(a, 4, v); }
static uae_u32 m_iw(void) { if (ext_is_pos >= ext_is_n) { buf_err = 1; return 0; } return ext_is[ext_is_pos++]; }
static uae_u32 m_il(void) { uae_u32 h = m_iw(); return (h << 16) | m_iw(); }

/* the MMU030 fmovem path and the non-cp accessors are unreachable (mmu_model == 0) but must exist */
uae_u32 (*x_get_long)(uaecptr)  = mg_l;
void    (*x_put_long)(uaecptr, uae_u32) = mp_l;
uae_u32 (*x_cp_get_byte)(uaecptr) = bg_b;
uae_u32 (*x_cp_get_word)(uaecptr) = bg_w;
uae_u32 (*x_cp_get_long)(uaecptr) = bg_l;
void    (*x_cp_put_byte)(uaecptr, uae_u32) = bp_b;
void    (*x_cp_put_word)(uaecptr, uae_u32) = bp_w;
void    (*x_cp_put_long)(uaecptr, uae_u32) = bp_l;
uae_u32 (*x_cp_next_iword)(void) = b_iw;
uae_u32 (*x_cp_next_ilong)(void) = b_il;
uae_u32 (REGPARAM3 *x_cp_get_disp_ea_020)(uae_u32, int) REGPARAM = b_disp;

void fpe_shim_use_buffer(void)
{
	x_cp_get_byte = bg_b; x_cp_get_word = bg_w; x_cp_get_long = bg_l;
	x_cp_put_byte = bp_b; x_cp_put_word = bp_w; x_cp_put_long = bp_l;
	x_cp_next_iword = b_iw; x_cp_next_ilong = b_il; x_cp_get_disp_ea_020 = b_disp;
}

void fpe_shim_use_memory(uint8_t *mem, uint32_t size, const uint16_t *is, int n)
{
	ext_mem = mem; ext_size = size; ext_is = is; ext_is_n = n; ext_is_pos = 0;
	x_cp_get_byte = mg_b; x_cp_get_word = mg_w; x_cp_get_long = mg_l;
	x_cp_put_byte = mp_b; x_cp_put_word = mp_w; x_cp_put_long = mp_l;
	x_cp_next_iword = m_iw; x_cp_next_ilong = m_il;
	/* (d8,An,Xn) is not exercised by the test */
	x_cp_get_disp_ea_020 = b_disp;
}

void fpe_shim_set_pc(uint32_t pc)
{
	regs.pc = pc;
	regs.instruction_pc = pc;
	regs.pc_p = regs.pc_oldp = dummy_code;
}

void fpe_shim_clear_events(void)
{
	fpe_ev_unimpl = fpe_ev_exc = 0;
	fpe_ev_vector = fpe_ev_opcode = 0;
	buf_err = 0;
}

/* ---- init / reset -------------------------------------------------------------------- */
static int inited;

void fpe_shim_init(void)
{
	if (inited) return;
	inited = 1;
	memset(&regs, 0, sizeof regs);
	memset(&currprefs, 0, sizeof currprefs);
	currprefs.cpu_model = 68030;
	currprefs.mmu_model = 0;
	currprefs.fpu_model = 68882;
	currprefs.fpu_mode = 1;                 /* softfloat back-end */
	currprefs.fpu_no_unimplemented = false;
	changed_prefs = currprefs;
	mmufixup[0].reg = mmufixup[1].reg = -1;
	fpe_shim_use_buffer();
	fpe_shim_set_pc(0);
}

#define FRESTORE_A0 0xF350u            /* FRESTORE (A0) */

void fpe_reset(void)
{
	fpe_shim_init();
	fpu_reset();
	/* FRESTORE of a null frame = 68882 hardware reset state (FPn = NaN, FPCR/FPSR/FPIAR = 0) */
	fpe_shim_use_buffer();
	fpe_shim_clear_events();
	memset(fpe_buf, 0, 8);
	regs.regs[8] = 0;
	fpe_shim_set_pc(4);
	buf_rd_end = 0;
	fpuop_restore(FRESTORE_A0);
	regs.fp_exp_pend = 0;
	regs.fp_unimp_pend = 0;
	fpe_shim_clear_events();
}

/* ---- execution ----------------------------------------------------------------------- */
static int popcnt8(unsigned v) { int n = 0; for (v &= 0xff; v; v >>= 1) n += v & 1; return n; }

int fpe_exec(uint16_t cmd, const uint8_t *in, int in_len, uint16_t aux, uint32_t iaddr,
             uint8_t *out, int *out_len)
{
	uint32_t opcode = 0xF210;               /* FPU instruction, EA = (A0) */
	uint32_t a0 = 0;
	int flags = 0, n, lo, hi;
	unsigned cls = cmd >> 13;

	fpe_shim_init();
	fpe_shim_use_buffer();
	fpe_shim_clear_events();
	if (in_len < 0 || in_len > FPE_MAXIO) in_len = in_len < 0 ? 0 : FPE_MAXIO, flags |= FPE_ERROR;

	/* FMOVEM.X FPn,<ea> with a predecrement-format register list (mode 00/01): fpp.c writes
	 * downwards from A0, so use -(A0) with A0 = transfer size; the buffer then holds the memory
	 * image the CPU stores at ascending addresses.  Everything else is plain (A0) at 0. */
	if (cls == 7 && ((cmd >> 11) & 3) < 2) {
		unsigned list = (((cmd >> 11) & 3) == 1) ? (aux & 0xff) : (cmd & 0xff);
		opcode = 0xF220;
		a0 = 12u * (unsigned)popcnt8(list);
	}

	memset(fpe_buf, 0, sizeof fpe_buf);
	if (in_len > 0 && in) memcpy(fpe_buf, in, (size_t)in_len);
	buf_rd_end = 0; buf_wr_lo = BUFSZ; buf_wr_hi = 0;

	regs.regs[8] = a0;                                   /* A0 */
	regs.regs[(cmd >> 4) & 7] = (uae_u32)(uae_s32)(int16_t)aux; /* Dn for dynamic list / k-factor */
	fpe_shim_set_pc(iaddr + 4);                          /* fpp.c: pc = m68k_getpc() - 4 */

	fpuop_arithmetic(opcode, cmd);

	if (fpe_ev_unimpl) flags |= FPE_UNIMPL;
	if (fpe_ev_exc)    flags |= FPE_EXCEPTION;
	if (buf_err || buf_rd_end > in_len) flags |= FPE_ERROR;

	lo = buf_wr_lo; hi = buf_wr_hi;
	n = (hi > lo) ? hi - lo : 0;
	if (n && lo != 0) flags |= FPE_ERROR;                /* results must start at offset 0 */
	if (n > FPE_MAXIO) n = FPE_MAXIO, flags |= FPE_ERROR;
	if (out && n) memcpy(out, fpe_buf + (n ? lo : 0), (size_t)n);
	if (out_len) *out_len = n;
	return flags;
}

uint32_t fpe_fpsr(void)  { fpe_shim_init(); return fpp_get_fpsr(); }
uint32_t fpe_fpcr(void)  { fpe_shim_init(); return fpp_get_fpcr(); }
uint32_t fpe_fpiar(void) { fpe_shim_init(); return fpp_get_fpiar(); }
uint32_t fpe_vector(void){ return fpe_ev_vector; }

int fpe_cond(int cc)
{
	fpe_shim_init();
	fpe_shim_clear_events();
	return fpp_cond(cc);
}

void fpe_set_fpcr(uint32_t v)  { fpe_shim_init(); fpp_set_fpcr(v); }
void fpe_set_fpsr(uint32_t v)  { fpe_shim_init(); fpp_set_fpsr(v); }
void fpe_set_fpiar(uint32_t v) { fpe_shim_init(); fpp_set_fpiar(v); }
void fpe_clear_exception(void) { regs.fp_exp_pend = 0; regs.fp_unimp_pend = 0; }

void fpe_set_fp(int n, uint16_t sexp, uint64_t mant)
{
	fpe_shim_init();
	fpp_to_exten_fmovem(&regs.fp[n & 7], (uint32_t)sexp << 16, (uint32_t)(mant >> 32), (uint32_t)mant);
}

void fpe_get_fp(int n, uint16_t *sexp, uint64_t *mant)
{
	uae_u32 w1, w2, w3;
	fpe_shim_init();
	fpp_from_exten_fmovem(&regs.fp[n & 7], &w1, &w2, &w3);
	if (sexp) *sexp = (uint16_t)(w1 >> 16);
	if (mant) *mant = ((uint64_t)w2 << 32) | w3;
}
