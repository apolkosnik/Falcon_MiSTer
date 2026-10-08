; t_irq.s - an interrupt during FSAVE's come-again (milestone 4): the ARM is held (SIGSTOP) while an
; FADD is outstanding, FSAVE reads the come-again format $0118 and the bench raises a level 3 interrupt.
; The 68030 takes it between two save CIR reads (the handler counts it), then FSAVE finishes with the idle frame.
	include	"common.i"

main:
	lea	h_irq,a0
	move.l	a0,(27*4).w		; level 3 autovector
	lea	BUF,a0
	move.l	#3,(a0)
	move.l	#4,4(a0)
	lea	BUF2,a2
	fmove.l	(a0),fp0
	fmove.l	4(a0),fp1
	fmove.x	fp0,(a2)		; waits for the ARM: everything before is done
	move.w	#$2000,sr		; interrupts enabled
	move.l	#1,MARK1
	WAITGO
	fadd.x	fp1,fp0			; background
	lea	BUF3+128,a3
	move.l	a3,a4
	dc.w	$f323			; fsave -(a3): come again until the reply
	move.l	a4,d1
	sub.l	a3,d1
	REC	T_IRQ_DELTA,d1
	moveq	#0,d1
	move.w	(a3),d1
	REC	T_IRQ_FMT,d1
	move.l	IRQCNT,d1
	REC	T_IRQ_CNT,d1
	bra	finish

h_irq:	addq.l	#1,IRQCNT
	rte
