; t_straddle.s - FSAVE / FRESTORE back to back, so that the bench can take the
; service away (MAGIC cleared) in the middle of an FPU instruction.  The
; bridge may refuse an instruction only on its first CIR access; an instruction
; already under way must complete (MC68030 UM 10: later BERR = real bus error).
; The bench checks this on the pin-level bus log of the whole run.
	include	"common.i"

main:
	WAITGO
loop:	tst.l	STOPF
	bne	finish
	lea	BUF+128,a0
	move.l	#4,SKIP
	FNOP_				; the FPU is idle: FSAVE gives the 60 byte frame
	move.l	#2,SKIP			; FSAVE/FRESTORE are one word
	dc.w	$f320			; fsave -(a0)
	dc.w	$f358			; frestore (a0)+
	addq.l	#1,ITER
	bra	loop
