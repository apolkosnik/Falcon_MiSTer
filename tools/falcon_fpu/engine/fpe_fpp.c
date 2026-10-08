/* fpe_fpp.c - Hatari's fpp.c (src/cpu/fpp.c, included unmodified) and the engine's fast path for
 * register-to-register arithmetic.
 *
 * fpe_fast_arith() is fpuop_arithmetic()'s opclass 000 path (FPm,FPn) with the per-instruction
 * checks that cannot do anything under its conditions left out; every step that can change
 * state is Hatari's own code, called in the same order.  It takes the instruction only when
 *   - the opmode is $00-$3F: fault_if_nonexisting_opmode() refuses none of them on a 6888x
 *     (only $40 and up), and FMOVECR is opclass 010;
 *   - no exception is pending (fp_exception_pending() would do nothing) and no
 *     unimplemented-datatype exception either;
 *   - no exception is enabled in the FPCR: maybe_set_fpiar() would not touch FPIAR and
 *     fpsr_check_arithmetic_exception() would find nothing (FPSR & FPCR & $FF00 = 0);
 *   - the model is the engine's, 68030 + 68882 (fault_if_no_6888x(), fault_if_no_fpu() and
 *     fault_if_unimplemented_680x0() are constant false; no 68040/68060 FPIAR or MMU fixup).
 * Left out: those checks and fpuop_arithmetic2's dispatch.  Kept: fpu_state, fp_exception,
 * fpu_mmu_fixup, fpsr_clear_status(), the source fetch and its normalization, the destination
 * normalization of dyadic operations, the unimplemented-datatype check after them,
 * fp_arithmetic() (the operation, condition codes and status) and the register write.
 * Returns 0 when it did not take the instruction (the caller runs fpuop_arithmetic()). */
#include "fpp.c"

int fpe_fast_arith(uae_u32 opcode, uae_u16 extra, uaecptr pc)
{
	if ((extra & 0xE040) != 0 || regs.fp_exp_pend || regs.fp_unimp_pend || (regs.fpcr & 0xff00) ||
	    currprefs.fpu_model != 68882 || currprefs.cpu_model != 68030)
		return 0;

	/* fpuop_arithmetic() */
	regs.fpu_state = 1;
	regs.fp_exception = false;
	fpu_mmu_fixup = false;

	/* fpuop_arithmetic2(), case 0 */
	int reg = (extra >> 7) & 7;
	fpdata src, dst;
	fpsr_clear_status();
	/* get_fp_value(), FPx to FPx */
	src = regs.fp[(extra >> 10) & 7];
	normalize_or_fault_if_no_denormal_support(opcode, extra, 0, false, pc, &src);
	dst = regs.fp[reg];
	if (fp_is_dyadic(extra))
		normalize_or_fault_if_no_denormal_support_dst(opcode, extra, 0, false, pc, &dst, &src);
	if (regs.fp_unimp_pend) {
		fp_exception_pending(false);
		return 1;
	}
	if (fp_arithmetic(&src, &dst, extra))
		regs.fp[reg] = dst;
	return 1;
}
