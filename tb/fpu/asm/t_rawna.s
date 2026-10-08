; t_rawna.s - raw CIR accesses with the service ABSENT: only the CIRs that are
; not the first access of a coprocessor instruction (response, control,
; operation word, operand, register select, instruction/operand address) may
; complete; the bridge refuses (BERR) just the command, condition and restore
; writes and the save read.  Here only the non-refused ones are touched.
	include	"common.i"

CIRB	equ	$00022000

main:
	moveq	#7,d0
	movec	d0,sfc
	movec	d0,dfc
	WAITGO
	lea	CIRB,a0
	moves.w	(a0),d1			; response
	moves.w	2(a0),d2
	moves.w	8(a0),d2
	moves.l	$10(a0),d2
	moves.w	$14(a0),d2
	moves.l	$18(a0),d2
	moves.l	$1C(a0),d2
	moveq	#0,d3
	moves.w	d3,2(a0)
	moves.w	d3,8(a0)
	moves.l	d3,$10(a0)
	moves.w	d3,$14(a0)
	moves.l	d3,$18(a0)
	moves.l	d3,$1C(a0)
	REC	T_RAW_DONE,#1
	bra	finish
