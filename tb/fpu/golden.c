/* golden.c - see golden.h.  Compiled against Hatari's headers (run.sh) and linked
 * with libfpe.a (Hatari fpp.c + softfloat + the shim of tools/falcon_fpu/engine). */
#include "sysconfig.h"
#include "sysdeps.h"
#include "main.h"
#include "hatari-glue.h"
#include "options_cpu.h"
#include "memory.h"
#include "newcpu.h"
#include "fpp.h"

#include <string.h>
#include <stdlib.h>

#include "falcon_fpu_engine.h"
#include "fpe_internal.h"
#include "golden.h"

static uint8_t *gram;

void gold_init(void)
{
	if (!gram) gram = calloc(1, GOLD_RAM_SIZE);
	fpe_reset();
}

uint8_t *gold_ram(void) { return gram; }
void gold_reset(void)
{
	uint32_t keep[16];
	for (int i = 0; i < 16; i++) keep[i] = regs.regs[i];
	fpe_reset();                          /* the engine's reset uses A0 for its own FRESTORE */
	for (int i = 0; i < 16; i++) regs.regs[i] = keep[i];
}
void gold_setreg(int r, uint32_t v) { regs.regs[r & 15] = v; }
uint32_t gold_getreg(int r) { return regs.regs[r & 15]; }

static int ev(void)
{
	return (fpe_ev_unimpl ? GOLD_UNIMPL : 0) | (fpe_ev_exc ? GOLD_EXC : 0) | (fpe_ev_fmt ? GOLD_FMT : 0);
}

/* UM rule 1 (as the engine's fpe_exec): a pending enabled exception is reported by opclass 000/010/011
 * and the conditionals only; FMOVEM and the control register moves (opclass 100-111) execute normally,
 * while Hatari checks the pending exception for every opclass. */
int gold_exec(uint32_t iaddr, uint16_t op, uint16_t cmd, const uint16_t *ext, int next)
{
	fpe_shim_use_memory(gram, GOLD_RAM_SIZE, ext, next);
	fpe_shim_set_pc(iaddr + 4);           /* fpp.c: instruction address = m68k_getpc() - 4 */
	fpe_shim_clear_events();
	if ((cmd >> 13) >= 4) {
		uae_u32 pend = regs.fp_exp_pend;
		regs.fp_exp_pend = 0;
		fpuop_arithmetic(op, cmd);
		if (!regs.fp_exp_pend) regs.fp_exp_pend = pend;
	} else
		fpuop_arithmetic(op, cmd);
	fpe_shim_use_buffer();
	return ev();
}

int gold_scc(uint32_t iaddr, uint16_t op, uint16_t cond, const uint16_t *ext, int next)
{
	fpe_shim_use_memory(gram, GOLD_RAM_SIZE, ext, next);
	fpe_shim_set_pc(iaddr + 4);
	fpe_shim_clear_events();
	fpuop_scc(op, cond);
	fpe_shim_use_buffer();
	return ev();
}

/* FSAVE / FRESTORE with a real EA.  UM rule 2: an FSAVE with an exception pending writes BIU flags
 * bit 27 = 0 (Hatari writes 1 unless fpu_exp_state is set, which only FRESTORE does) and absorbs it. */
int gold_fsave(uint32_t iaddr, uint16_t op, const uint16_t *ext, int next)
{
	fpe_shim_use_memory(gram, GOLD_RAM_SIZE, ext, next);
	fpe_shim_set_pc(iaddr + 2);
	fpe_shim_clear_events();
	if (regs.fp_exp_pend) regs.fpu_exp_state = 2;
	fpuop_save(op);
	fpe_shim_use_buffer();
	return ev();
}
int gold_frestore(uint32_t iaddr, uint16_t op, const uint16_t *ext, int next)
{
	fpe_shim_use_memory(gram, GOLD_RAM_SIZE, ext, next);
	fpe_shim_set_pc(iaddr + 2);
	fpe_shim_clear_events();
	fpuop_restore(op);
	fpe_shim_use_buffer();
	return ev();
}

/* the exception handler's epilogue: FSAVE -(sp); set BIU flags bit 27; FRESTORE (sp)+ (the engine's
 * fpe_save/fpe_restore are Hatari's fpuop_save/fpuop_restore with rule 2 applied) */
void gold_epilogue(void)
{
	uint8_t fr[256];
	int n = 0;
	fpe_save(fr, &n);
	if (n > 56 && fr[0] != 0) fr[56] |= 0x08;
	fpe_restore(fr, n);
}

/* the frame an FSAVE writes right after a reset followed by FNOP (null -> idle) */
int gold_idle_frame(uint8_t *out)
{
	uint32_t keep[16];
	int n = 0;
	for (int i = 0; i < 16; i++) keep[i] = regs.regs[i];
	fpe_reset();
	fpe_cond(0);
	fpe_save(out, &n);
	fpe_reset();
	for (int i = 0; i < 16; i++) regs.regs[i] = keep[i];
	return n;
}

int gold_status(void) { return fpe_status(); }
int gold_pending(void) { return regs.fp_exp_pend != 0; }
int gold_fpu_state(void) { return (int)regs.fpu_state; }

/* the engine's conditional: Hatari FScc with pending-exception report, BSUN and null->idle; the 68882
 * keeps a pending exception when it is taken, so nothing is cleared here (see gold_epilogue) */
int gold_cond(int cc)
{
	return fpe_cond(cc);
}
int gold_last_vector(void) { return (int)fpe_vector(); }

uint32_t gold_fpsr(void) { return fpe_fpsr(); }
uint32_t gold_fpcr(void) { return fpe_fpcr(); }
uint32_t gold_fpiar(void) { return fpe_fpiar(); }
void gold_get_fp(int n, uint16_t *se, uint64_t *m) { fpe_get_fp(n, se, m); }
