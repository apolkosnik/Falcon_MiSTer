; t_bg.s - background execution (milestone 2).  A register-to-register FADD is
; released at once ($0900); the CPU runs non-FPU instructions while the ARM
; has not replied (the bench holds the service with SIGSTOP); the next FPU
; instruction gets "come again" ($8900) until the reply.
; Expected results are exact in extended precision: 3 + 4 = 7, 4 + 7 = 11.
	include	"common.i"

main:
	lea	BUF,a0
	move.l	#3,(a0)
	move.l	#4,4(a0)
	lea	BUF2,a2
	fmove.l	(a0),fp0
	fmove.l	4(a0),fp1
	fmove.x	fp0,(a2)		; waits for the ARM: everything before is done
	move.l	#1,MARK1
	WAITGO
bgi:	fadd.x	fp1,fp0			; background: released at once
	move.l	#1,MARK2		; the CPU goes on while the ARM is held
	moveq	#0,d1
	addq.l	#1,d1
	addq.l	#1,d1
	addq.l	#1,d1
	addq.l	#1,d1
	addq.l	#1,d1
	addq.l	#1,d1
	addq.l	#1,d1
	addq.l	#1,d1
	move.l	d1,MARK3		; 8 counted
	move.l	#1,MARK4
bg2:	fadd.x	fp0,fp1			; waits (come again) until the reply
	fmove.x	fp0,(a2)
	fmove.x	fp1,12(a2)
	bra	finish
