; t_wd.s - the reply watchdog (milestone 2).  An FPU instruction that waits for
; a reply the ARM does not give ends with the mid-instruction exception
; "coprocessor protocol violation", vector 13 (bridge header: ~100 ms); the
; machine does not hang.  MODE ($0F50): 0 = no service at all (C++ heartbeat),
; 1 = real service, held by the bench with SIGSTOP and released afterwards.
; Later instructions work again.
	include	"common.i"

MODE	equ	$0F50

main:
	lea	BUF,a0
	move.l	#3,(a0)
	lea	BUF2,a2
	tst.l	MODE
	beq.s	nosync
	fmove.l	(a0),fp0
	fmove.x	fp0,(a2)
	move.l	#1,MARK1
nosync:	WAITGO
	move.l	sp,SAVESP
	lea	enda(pc),a1
	move.l	a1,RESUME
	move.l	#1,ABORTMODE
wda:	fmove.x	fp0,(a2)		; waits for a reply that does not come
	move.l	#1,MARK2		; (not reached: the exception abandons it)
enda:	move.l	#1,MARK3
	lea	BUF,a0			; (the abort handler used a0)
	lea	BUF2,a2
	tst.l	MODE
	beq	finish
wait2:	tst.l	STOPF			; the bench releases the service, then sets STOPF
	beq.s	wait2
	move.l	#0,ABORTMODE
	fmove.l	(a0),fp0		; recovery: normal instructions work again
	fadd.x	fp0,fp0
	fmove.x	fp0,12(a2)
	move.l	#1,MARK4
	bra	finish
