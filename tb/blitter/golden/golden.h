/* API of the Hatari blitter golden model (golden_glue.c) */
#ifndef GOLDEN_H
#define GOLDEN_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
extern int gm_int_pending;
extern int gm_gpu_line;
void gm_init(void);
void gm_reset(void);
void gm_write(int off, int size, uint16_t val);
uint16_t gm_read(int off);
int gm_run_pass(void);
/* provided by the testbench */
uint16_t gm_mem_read(uint32_t addr);
void gm_mem_write(uint32_t addr, uint16_t val);
#ifdef __cplusplus
}
#endif
#endif
