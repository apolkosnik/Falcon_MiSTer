; t_absent.s - the FPU's own CpID 1 with the ARM service absent: every
; instruction form ends its first CIR access in BERR -> F-line (vector 11),
; exactly the behaviour before the bridge existed.
	include	"common.i"
	include	"cpall.i"

main:
	WAITGO
	lea	BUF,a0
	CPALL	1
	bra	finish
