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
#define FPE_FORMAT    0x08  /* FRESTORE format error (vector 14 in fpe_vector()) */

#define FPE_MAXIO 256       /* maximum operand/result/frame bytes (busy frame = 216, FMOVEM.X of 8 regs = 96) */

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

/* Condition predicate (cc = 0..31, the cpBcc/cpScc/... condition) against current FPSR; runs like a real
 * conditional instruction (null->idle, pending exception reported first, BSUN).
 * Returns 1 true, 0 false, -2 an exception was raised instead (pre-instruction pending exception or BSUN:
 * vector in fpe_vector(), fpe_status() has FPE_ST_PRE). */
int fpe_cond(int cc);

/* Exception state, valid after fpe_exec / fpe_cond / fpe_save / fpe_restore.  Bits of fpe_status():
 *   FPE_ST_PEND    an enabled exception is pending (fp_exp_pend != 0), vector = fpe_pending_vector() (48..54).
 *                  The 68882 does not clear it when the CPU takes the exception; the caller clears it with
 *                  fpe_clear_exception(), or FSAVE absorbs it (see fpe_save).
 *   FPE_ST_MID     this instruction itself raised the exception (FMOVE out): result bytes are valid, the
 *                  exception is reported mid-instruction.  Always together with FPE_EXCEPTION.
 *   FPE_ST_PRE     this instruction was NOT executed: an earlier pending exception (or BSUN) was reported
 *                  before it (pre-instruction).  Always together with FPE_EXCEPTION.
 *   FPE_ST_ENABLED some arithmetic exception is enabled in FPCR (fpcr & 0x7f00) - FPIAR is then loaded.
 * "Pending for the next instruction" = PEND without MID/PRE (arithmetic that raised an enabled exception). */
#define FPE_ST_PEND    0x01
#define FPE_ST_MID     0x02
#define FPE_ST_PRE     0x04
#define FPE_ST_ENABLED 0x08
int      fpe_status(void);
uint32_t fpe_pending_vector(void);

/* FSAVE: frame image (ascending memory order; 68882 idle = $1F38 + 14 longs = 60 bytes, null = 4 bytes
 * $0038xxxx-style).  frame capacity FPE_MAXIO.  If an exception is pending the BIU flags long (last long of
 * the idle frame) has bit 27 = 0 (UM 6.4.2.2); the pending exception is absorbed by the frame
 * (fp_exp_pend cleared, as FSAVE does).  The CCR long (offset 4) of a frame saved after an exception holds
 * the opcode word with the synthetic EA ($F290 | cmd); the bridge may patch the EA field.  Returns FPE_* flags. */
int fpe_save(uint8_t *frame, int *len);

/* FRESTORE of a frame image (len = its size): null ($00xxxxxx), idle $1F38/$1F18, busy $1FD4/$1FB4 (accepted
 * and skipped, as Hatari does).  An idle frame with BIU bit 27 = 0 re-arms the pending exception from
 * FPSR&FPCR (check fpe_status()).  Bad version/size -> FPE_FORMAT (vector 14), state unchanged.  Returns flags. */
int fpe_restore(const uint8_t *frame, int len);

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
