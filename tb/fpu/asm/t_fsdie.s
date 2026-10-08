; t_fsdie.s - FSAVE whose service stops answering (milestone 4): the ARM is held (SIGSTOP) before the
; FSAVE; after the bridge's watchdog the FSAVE ends with a null frame ($0000), without BERR and without an
; exception.  Later instructions work once the service is back (FNOP; FSAVE gives an idle frame).
	include	"common.i"

main:
	lea	BUF,a0
	move.l	#3,(a0)
	lea	BUF2,a2
	fmove.l	(a0),fp0
	fmove.x	fp0,(a2)
	move.l	#1,MARK1
	WAITGO
	lea	BUF3+128,a3
	move.l	a3,a4
	dc.w	$f323			; fsave -(a3): the service is held
	move.l	a4,d1
	sub.l	a3,d1
	REC	T_FSD_DELTA,d1
	moveq	#0,d1
	move.w	(a3),d1
	REC	T_FSD_FMT,d1
	move.l	#1,MARK2
wait2:	tst.l	STOPF			; the bench releases the service, then sets STOPF
	beq.s	wait2
	FNOP_
	lea	BUF4+128,a3
	move.l	a3,a4
	dc.w	$f323			; fsave -(a3)
	move.l	a4,d1
	sub.l	a3,d1
	REC	T_FSD_DELTA2,d1
	moveq	#0,d1
	move.w	(a3),d1
	REC	T_FSD_FMT2,d1
	bra	finish
