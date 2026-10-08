; t_busy.s - FSAVE while a request is outstanding returns the come-again
; format $0118 (UM 6.4.3) until the ARM replied, then the idle frame.  The
; bench holds the service (SIGSTOP) after FADD was released.
	include	"common.i"

main:
	lea	BUF,a0
	move.l	#3,(a0)
	move.l	#4,4(a0)
	lea	BUF2,a2
	fmove.l	(a0),fp0
	fmove.l	4(a0),fp1
	fmove.x	fp0,(a2)
	move.l	#1,MARK1
	WAITGO
	fadd.x	fp1,fp0			; background
	move.l	#1,MARK2
	lea	BUF3+128,a3
	move.l	a3,a4
	dc.w	$f323			; fsave -(a3): come again until the reply
	move.l	a4,d1
	sub.l	a3,d1
	REC	T_BUSY_DELTA,d1
	moveq	#0,d1
	move.w	(a3),d1
	REC	T_BUSY_FMT,d1
	fmove.x	fp0,(a2)
	bra	finish
