/* falcon_fpu_engine.h - MC68882 execution engine API (Hatari/WinUAE fpp.c under a thin shim).
 *
 * The engine never sees a 68k address.  Per FPU instruction the caller gives the
 * 16-bit coprocessor command word (cpGEN extension word), the operand bytes the CPU
 * transferred (big-endian, ascending memory order) and gets the bytes the CPU has to
 * store at ascending addresses.
 *
 * Register-direct operands (Dn as source/destination, #imm) use the same bytes as the
 * memory forms: B = 1 byte, W = 2, L/S = 4, D = 8, X/P = 12 (an immediate byte/word is
 * passed as 1/2 bytes, the main-register side merges the result itself).
 *
 * Not covered here (later milestones): FSAVE/FRESTORE frames, FBcc/FScc/FDBcc/FTRAPcc
 * predicates (fpe_cond() evaluates a condition against the current FPSR).
 */
#ifndef FALCON_FPU_ENGINE_H
#define FALCON_FPU_ENGINE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* fpe_exec() return flags */
#define FPE_UNIMPL    0x01  /* op_illg(): illegal/unsupported command -> F-line / protocol violation */
#define FPE_EXCEPTION 0x02  /* Exception(): an enabled FPU exception is pending, vector in fpe_vector() */
#define FPE_ERROR     0x04  /* shim error: operand underrun/overrun or out-of-range access */

#define FPE_MAXIO 128       /* maximum operand/result bytes (FMOVEM.X of 8 registers = 96) */

/* 68882 hardware reset / FRESTORE(null): FP0-FP7 = non-signaling NaN, FPCR=FPSR=FPIAR=0. */
void fpe_reset(void);

/* Execute one instruction.
 *   cmd    cpGEN command word (Hatari's "extra")
 *   in     operand bytes sent by the CPU (may be NULL when in_len == 0)
 *   aux    low word of the data register for a dynamic FMOVEM list or dynamic k-factor
 *   iaddr  address of the instruction's F-line opcode word (stored in FPIAR as fpp.c does)
 *   out    result bytes in ascending memory order (capacity FPE_MAXIO)
 *   out_len number of result bytes written to out
 * Returns FPE_* flags (0 = completed normally). */
int fpe_exec(uint16_t cmd, const uint8_t *in, int in_len, uint16_t aux, uint32_t iaddr,
             uint8_t *out, int *out_len);

uint32_t fpe_fpsr(void);
uint32_t fpe_fpcr(void);
uint32_t fpe_fpiar(void);
uint32_t fpe_vector(void);              /* vector of the last FPE_EXCEPTION / 11 for FPE_UNIMPL */

/* Condition predicate (cc = 0..31, the cpBcc/cpScc/... condition) against current FPSR.
 * Returns 1 true, 0 false, -2 BSUN exception raised (FPE_EXCEPTION state, vector 48). */
int fpe_cond(int cc);

/* State access (tests, FSAVE/FRESTORE later).  FP register = sign/exponent word + 64-bit mantissa. */
void fpe_set_fp(int n, uint16_t sexp, uint64_t mant);
void fpe_get_fp(int n, uint16_t *sexp, uint64_t *mant);
void fpe_set_fpcr(uint32_t v);
void fpe_set_fpsr(uint32_t v);
void fpe_set_fpiar(uint32_t v);
void fpe_clear_exception(void);         /* drop the pending enabled exception (after the CPU took it) */

#ifdef __cplusplus
}
#endif
#endif
