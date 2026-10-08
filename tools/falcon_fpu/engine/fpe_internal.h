/* fpe_internal.h - shim internals shared with fpe_selftest.c (not part of the public API) */
#ifndef FPE_INTERNAL_H
#define FPE_INTERNAL_H
#include <stdint.h>

/* per-instruction event record filled by the Exception()/op_illg() hooks */
extern int      fpe_ev_unimpl;     /* op_illg() called */
extern int      fpe_ev_exc;        /* Exception() called */
extern uint32_t fpe_ev_vector;     /* last vector */
extern uint32_t fpe_ev_opcode;     /* opcode given to op_illg */

void fpe_shim_init(void);                 /* idempotent: prefs, softfloat back-end */
void fpe_shim_set_pc(uint32_t pc);        /* what m68k_getpc() returns */
void fpe_shim_clear_events(void);
/* point the cp accessors back at the synthetic buffer (fpe_exec does this itself) */
void fpe_shim_use_buffer(void);
/* point the cp accessors at a caller supplied big-endian memory + instruction stream */
void fpe_shim_use_memory(uint8_t *mem, uint32_t size, const uint16_t *istream, int istream_words);
#endif
