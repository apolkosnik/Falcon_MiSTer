; t_cpid.s - coprocessor instructions of CpID 2..7: there is no such
; coprocessor, the bridge ends the first CIR access with BERR and the 68030
; takes the F-line exception (vector 11, MC68030 UM 10.4 / 7.1.4; a BERR on the
; first CIR access of an instruction means "no coprocessor").  Every
; instruction form: cpGEN, cpBcc, cpScc, cpDBcc, cpTRAPcc, cpSAVE, cpRESTORE.
; (CpID 0 is the 68030's own PMMU, not a coprocessor bus access.)
; With NOCPID1 undefined CpID 2..7 are tested.
	include	"common.i"
	include	"cpall.i"

main:
	WAITGO
	lea	BUF,a0
	CPALL	2
	CPALL	3
	CPALL	4
	CPALL	5
	CPALL	6
	CPALL	7
	bra	finish
