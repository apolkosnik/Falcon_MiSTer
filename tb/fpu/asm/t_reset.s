; t_reset.s - the RESET instruction resets the peripherals (the bridge gets
; dev_reset, as in falcon_system): the FPU returns to the null state
; (MC68881/MC68882 UM 6.4.2: null state after reset) while the ARM service
; stays present.
	include	"common.i"

main:
	WAITGO
	FNOP_
	lea	BUF+64,a0
	dc.w	$f320			; fsave -(a0)  (idle)
	moveq	#0,d1
	move.w	(a0),d1
	REC	T_RS_BEFORE,d1
	reset
	lea	BUF2+64,a0
	move.l	a0,a2
	dc.w	$f320			; fsave -(a0)  (null)
	moveq	#0,d1
	move.w	(a0),d1
	REC	T_RS_AFTER,d1
	move.l	a2,d1
	sub.l	a0,d1
	REC	T_RS_SIZE,d1
	move.l	EXCNT,d7
	FNOP_
	move.l	EXCNT,d1
	sub.l	d7,d1
	REC	T_RS_FNOP,d1
	bra	finish
