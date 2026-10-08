/* fpe_smoke.c - golden-free smoke test of libfpe.a (builds for host and ARM; runs under qemu-arm).
 * Known-answer checks only: values are 68881 architecture facts, not taken from Hatari. */
#include <stdio.h>
#include <string.h>
#include "falcon_fpu_engine.h"

static int bad;
static void expect(const char *what, int cond) { printf("%-44s %s\n", what, cond ? "ok" : "FAIL"); bad |= !cond; }

int main(void)
{
	uint8_t out[FPE_MAXIO], in[FPE_MAXIO]; int n, fl;
	static const uint8_t pi_x[12] = { 0x40, 0x00, 0, 0, 0xC9, 0x0F, 0xDA, 0xA2, 0x21, 0x68, 0xC2, 0x35 };

	fpe_reset();
	fl = fpe_exec(0x5C00 | 0 << 7 | 0x00, NULL, 0, 0, 0x1000, out, &n);           /* FMOVECR #0,FP0 */
	expect("FMOVECR #0,FP0 completes", fl == 0 && n == 0);
	fl = fpe_exec(0x6800 | 0 << 7, NULL, 0, 0, 0x1004, out, &n);                  /* FMOVE.X FP0,<ea> */
	expect("FMOVE.X FP0 = pi, 12 bytes", fl == 0 && n == 12 && !memcmp(out, pi_x, 12));

	memset(in, 0, sizeof in); in[3] = 3;                                          /* FMOVE.L #3,FP1 */
	fl = fpe_exec(0x4000 | 1 << 7, in, 4, 0, 0x1008, out, &n);
	expect("FMOVE.L #3,FP1", fl == 0);
	fl = fpe_exec(0x0000 | 1 << 10 | 0 << 7 | 0x22, NULL, 0, 0, 0x100c, out, &n); /* FADD FP1,FP0 */
	fl = fpe_exec(0x6000 | 0 << 10 | 0 << 7, NULL, 0, 0, 0x1010, out, &n);        /* FMOVE.L FP0,<ea> */
	expect("FADD FP1,FP0; FMOVE.L FP0 = 6 (pi+3 rounded)", fl == 0 && n == 4 && !out[0] && !out[1] && !out[2] && out[3] == 6);

	fl = fpe_exec(0x6000 | 6 << 10 | 0 << 7, NULL, 0, 0, 0x1014, out, &n);       /* FMOVE.B FP0,<ea> */
	expect("FMOVE.B FP0 = 6", fl == 0 && n == 1 && out[0] == 6);

	/* FMOVEM.X FP0-FP1 to -(An) (predecrement-format mask: bit0 = FP0): ascending image FP0 then FP1 */
	fl = fpe_exec(0xE000 | 0 << 11 | 0x03, NULL, 0, 0, 0x1018, out, &n);
	expect("FMOVEM.X FP0/FP1,-(An): 24 bytes, FP0 (6.14) then FP1 (3.0)", fl == 0 && n == 24 && out[0] == 0x40 && out[1] == 0x01 && out[12] == 0x40 && out[13] == 0x00 && out[16] == 0xC0 && !out[17]);

	/* FMOVEM.L FPCR/FPSR/FPIAR,<ea>: 12 bytes; FPIAR holds the address of the last trapping-capable insn only if enabled */
	fl = fpe_exec(0xA000 | 7 << 10, NULL, 0, 0, 0x101c, out, &n);
	expect("FMOVEM.L FPCR/FPSR/FPIAR: 12 bytes", fl == 0 && n == 12);

	fl = fpe_exec(0x0000 | 0x7f, NULL, 0, 0, 0x1020, out, &n);                    /* undefined opmode */
	expect("undefined opmode flagged FPE_UNIMPL", (fl & FPE_UNIMPL) != 0);

	printf("FPSR=%08x FPCR=%08x FPIAR=%08x\n", fpe_fpsr(), fpe_fpcr(), fpe_fpiar());
	printf("%s\n", bad ? "FAIL" : "PASS");
	return bad;
}
