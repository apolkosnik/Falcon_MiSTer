/* golden.h - Hatari's fpp.c driven the normal way (real opcodes with real EAs
 * over a fake 68k RAM and register file): the independent expected values for
 * the milestone 2 tests.  Plain C interface; implementation in golden.c. */
#ifndef TB_GOLDEN_H
#define TB_GOLDEN_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#define GOLD_RAM_SIZE 0x1000000u     /* guest addresses are used directly (RAM, and the ROM at $E00000) */
#define GOLD_UNIMPL   1
#define GOLD_EXC      2
#define GOLD_FMT      4            /* FRESTORE format error (vector 14) */
#define GOLD_RAM_SIZE_NOTE 0

void gold_init(void);
uint8_t *gold_ram(void);
void gold_reset(void);                              /* FRESTORE null: FPn = NaN, FPCR/FPSR/FPIAR = 0 */
void gold_setreg(int r, uint32_t v);                /* 0-7 D0-D7, 8-15 A0-A7 */
uint32_t gold_getreg(int r);
/* execute opcode+cmd (+ extension words) located at address iaddr; returns GOLD_* flags */
int gold_exec(uint32_t iaddr, uint16_t op, uint16_t cmd, const uint16_t *ext, int next);
/* FScc <ea> */
int gold_scc(uint32_t iaddr, uint16_t op, uint16_t cond, const uint16_t *ext, int next);
int gold_fsave(uint32_t iaddr, uint16_t op, const uint16_t *ext, int next);
int gold_frestore(uint32_t iaddr, uint16_t op, const uint16_t *ext, int next);
void gold_epilogue(void);
int gold_idle_frame(uint8_t *out);
int gold_status(void);
int gold_pending(void);
int gold_fpu_state(void);
/* condition predicate cc (0..31) as fpp.c evaluates it (side effects on FPSR included); -2: BSUN exception */
int gold_last_vector(void);                          /* vector of the last exception/unimplemented event */
int gold_cond(int cc);
uint32_t gold_fpsr(void);
uint32_t gold_fpcr(void);
uint32_t gold_fpiar(void);
void gold_get_fp(int n, uint16_t *sexp, uint64_t *mant);

#ifdef __cplusplus
}
#endif
#endif
