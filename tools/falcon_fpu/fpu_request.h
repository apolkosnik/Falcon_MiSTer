/*
 * fpu_request.h - one mailbox request of the Falcon core's 68882 bridge,
 * executed by libfpe.  Shared by the HPS service (falcon_fpu.c) and the
 * system simulation (tb/system/sim_main.cpp), so both answer alike.
 *
 * Request kinds (mailbox +$102):
 *   1 execute  the cpGEN command CMD with the operand bytes; AUX = Dn of a
 *              dynamic list or k-factor; IADDR = the instruction address
 *              (posted by the bridge when it asked for the PC: FPIAR)
 *   2 reset    hardware reset / FRESTORE of a null frame
 *   3 condition  the predicate in CMD (the bridge asks only for IEEE-nonaware
 *              predicates with NAN set, and for the first conditional after
 *              a reset, which takes the FPU from null to idle)
 *   4 save     FSAVE: the result bytes are the frame image (format long first)
 *   5 restore  FRESTORE: AUX = format word, the operand bytes = the frame body
 *
 * Reply FLAGS (mailbox +$202):
 *   bit 0      the instruction is not implemented (F-line)
 *   bit 1      EXC PEND: an enabled exception is pending; the next opclass
 *              000/010/011 instruction or conditional takes it (pre-instruction)
 *              until FSAVE absorbs it (the 68882 keeps it when it is taken)
 *   bit 2      the condition is true
 *   bit 3      an arithmetic exception is enabled (FPCR[14:8]): the bridge
 *              asks for the instruction address (PC bit)
 *   bit 4      this request raised the exception itself: FMOVE out
 *              (mid-instruction) or a conditional (BSUN, pre-instruction)
 *   bits 15..8 vector of the exception (bit 4) or of the pending one (bit 1)
 */
#ifndef FPU_REQUEST_H
#define FPU_REQUEST_H

#include <stdint.h>
#include <string.h>

#include "engine/falcon_fpu_engine.h"

enum { FPU_KIND_EXEC = 1, FPU_KIND_RESET = 2, FPU_KIND_COND = 3,
       FPU_KIND_SAVE = 4, FPU_KIND_RESTORE = 5 };

#define FPU_REQ_MAX 240     /* request bytes: mailbox +$110..+$1FF */

static inline uint16_t fpu_request(uint16_t kind, uint16_t cmd, uint16_t aux, uint32_t iaddr,
                                   const uint8_t *in, int n, uint8_t *out, int *out_len)
{
	int flags = 0;

	*out_len = 0;
	if (n < 0) n = 0;
	if (n > FPU_REQ_MAX) n = FPU_REQ_MAX;
	switch (kind) {
	case FPU_KIND_RESET:
		fpe_reset();
		break;
	case FPU_KIND_COND: {
		int r = fpe_cond(cmd & 0x3f);
		if (r > 0) flags |= 4;
		break;
	}
	case FPU_KIND_SAVE:
		fpe_save(out, out_len);
		break;
	case FPU_KIND_RESTORE: {
		uint8_t frame[4 + FPU_REQ_MAX];
		frame[0] = (uint8_t)(aux >> 8);
		frame[1] = (uint8_t)aux;
		frame[2] = frame[3] = 0;
		memcpy(frame + 4, in, (size_t)n);
		fpe_restore(frame, 4 + n);
		break;
	}
	default:
		flags |= fpe_exec(cmd, in, n, aux, iaddr, out, out_len) & FPE_UNIMPL;
		break;
	}
	if (*out_len < 0 || *out_len > FPE_MAXIO) *out_len = 0;

	int st = fpe_status();
	uint32_t vec = 0;
	if (st & (FPE_ST_MID | FPE_ST_PRE)) {
		flags |= 0x10;
		vec = fpe_vector();
	} else if (st & FPE_ST_PEND)
		vec = fpe_pending_vector();
	if (st & FPE_ST_PEND) flags |= 2;
	if (st & FPE_ST_ENABLED) flags |= 8;
	return (uint16_t)(flags | (vec & 0xff) << 8);
}

#endif
