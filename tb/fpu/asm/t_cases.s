; t_cases.s - the generated milestone 2 cases (obj/gen_cases.s, from tb/fpu/m2.cpp)
; Every case runs the real instruction sequence on the CPU; an unexpected
; exception (vector 11, 13, 14 ...) aborts that case only (habort in common.i).
	include	"common.i"
	include	"m2macros.i"

main:
	move.l	sp,SAVESP
	move.l	#1,ABORTMODE
	lea	h_exc,a0		; expected soft exceptions: vector 4 (refused opmodes)
	move.l	a0,(4*4).w
	lea	h_irq,a0		; level 3 autovector: the interrupt-during-come-again cases
	move.l	a0,(27*4).w
	lea	h_fpexc,a0		; vectors 48-54: BSUN and the FPU exceptions
	moveq	#6,d0
	lea	(48*4).w,a1
vl:	move.l	a0,(a1)+
	dbra	d0,vl
	WAITGO
	include	"gen_cases.s"
	jmp	finish

; FPU exception handler (vectors 48-54).  Like a real handler it absorbs the
; exception: FSAVE -(sp) (the 68882 keeps a pending exception until FSAVE takes it, and
; writes BIU flags bit 27 = 0 into the frame), set bit 27 so that FRESTORE does not
; re-arm it, FRESTORE (sp)+.  HMODE = 1: just return (the exception stays pending).
; A pre-instruction frame (format 0) is stepped over; a mid-instruction frame (format 9)
; resumes the instruction.
h_fpexc:
	logexc
	tst.l	SOFTEXC
	bne.s	hf0
	jmp	habort
hf0:	tst.l	HMODE
	bne.s	hf2
	dc.w	$f327			; fsave -(sp)
	tst.b	(sp)
	beq.s	hf1			; a null frame: nothing to change
	bset	#3,56(sp)
hf1:	dc.w	$f35f			; frestore (sp)+
hf2:	tst.l	d1
	bne.s	hf3
	move.l	SKIP,d2
	add.l	d2,2(a0)
hf3:	movem.l	(sp)+,d0-d2/a0-a1
	rte

; level 3 interrupt (vector 27): logged with its frame (format 9 when it hit a come-again), counted
h_irq:
	logexc
	addq.l	#1,IRQCNT
	movem.l	(sp)+,d0-d2/a0-a1
	rte
