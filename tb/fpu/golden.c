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

int gold_exec(uint32_t iaddr, uint16_t op, uint16_t cmd, const uint16_t *ext, int next)
{
	fpe_shim_use_memory(gram, GOLD_RAM_SIZE, ext, next);
	fpe_shim_set_pc(iaddr + 4);           /* fpp.c: instruction address = m68k_getpc() - 4 */
	fpe_shim_clear_events();
	fpuop_arithmetic(op, cmd);
	fpe_shim_use_buffer();
	return (fpe_ev_unimpl ? GOLD_UNIMPL : 0) | (fpe_ev_exc ? GOLD_EXC : 0);
}

void gold_scc(uint32_t iaddr, uint16_t op, uint16_t cond, const uint16_t *ext, int next)
{
	fpe_shim_use_memory(gram, GOLD_RAM_SIZE, ext, next);
	fpe_shim_set_pc(iaddr + 4);
	fpe_shim_clear_events();
	fpuop_scc(op, cond);
	fpe_shim_use_buffer();
}

int gold_cond(int cc)
{
	uint32_t s = fpe_fpsr();
	int r = fpe_cond(cc);
	fpe_set_fpsr(s);                      /* a BSUN status bit set by the test is not part of the state */
	fpe_clear_exception();
	return r;
}

uint32_t gold_fpsr(void) { return fpe_fpsr(); }
uint32_t gold_fpcr(void) { return fpe_fpcr(); }
uint32_t gold_fpiar(void) { return fpe_fpiar(); }
void gold_get_fp(int n, uint16_t *se, uint64_t *m) { fpe_get_fp(n, se, m); }
