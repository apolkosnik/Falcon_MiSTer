; t_ctxsw.s - a context switch inside a waiting FPU instruction.  An instruction that is told to
; come again ($8900) can be interrupted (MC68030 UM 10.4.8: a format $9 frame; the RTE reads the
; response CIR again).  A kernel's handler saves the FPU (FSAVE, FMOVEM), another process uses
; it, and the handler restores it (FMOVEM, FRESTORE) before the RTE.  The 68882 carries the
; pending instruction through the frame (UM 6.4.2, the idle frame's command/condition register):
; it executes after the RTE.  MODE ($0F50): where the instruction waits
;   0  FMOVE.D FP0,(d16,A3) behind a background FADD (its command written, not started)
;   1  FMOVE.D FP0,(d16,A3) waiting for its own result
;   2  FBLT behind a background FCMP (its condition written)
;   3  FMOVE.L (d16,A0),FP0 behind the reset of an FRESTORE of a null frame (the FPU is null)
	include	"common.i"

MODE	equ	$0F50
SWVEC	equ	$0F64		; long: format/vector word of the interrupt's frame
SWIA	equ	$0F68		; long: its instruction address
SWFMT	equ	$0F6C		; long: format word of the frame FSAVE gave the handler
SWCCR	equ	$0F70		; long: that frame's command/condition register word
PCB	equ	BUF3		; the interrupted process: frame 0, FP0-FP7 64, control registers 160
PCB2	equ	BUF4		; the other process
OTHER	equ	BUF+64		; the other process's result

main:
	lea	h_sw,a0
	move.l	a0,(27*4).w		; level 3 autovector
	lea	BUF,a0
	move.l	#3,(a0)
	move.l	#4,4(a0)
	move.l	#5,8(a0)
	lea	BUF2,a2
	lea	BUF2+32,a3
	move.l	#$5EED5EED,d0
	move.l	d0,40(a3)
	move.l	d0,44(a3)
	move.l	d0,48(a3)
	move.l	d0,52(a3)
	fmove.l	(a0),fp0
	fmove.l	4(a0),fp1
	fmove.x	fp0,(a2)		; waits for the ARM: everything before is done
	move.w	#$2000,sr		; interrupts enabled
	move.l	MODE,d0
	cmp.l	#1,d0
	beq	m1
	cmp.l	#2,d0
	beq	m2
	cmp.l	#3,d0
	beq	m3

m0:	move.l	#1,MARK1
	WAITGO
	fadd.x	fp1,fp0			; background: 3 + 4
pend0:	fmove.d	fp0,40(a3)		; come again behind it: interrupted here
	nop
	nop
	bra	tail

m1:	move.l	#1,MARK1
	WAITGO
pend1:	fmove.d	fp0,40(a3)		; come again until its result: interrupted here
	nop
	nop
	bra	tail

m2:	move.l	#1,MARK1
	WAITGO
	fcmp.x	fp1,fp0			; background: 3 - 4, N
pend2:	fblt.w	yes2			; come again behind it: interrupted here
	move.l	#$0BAD,d1
	bra.s	rec2
yes2:	move.l	#$600D,d1
rec2:	REC	T_CS_BR,d1
	bra	tail

m3:	move.l	#1,MARK1
	WAITGO
	lea	nullf,a1
	dc.w	$f351			; frestore (a1): the FPU is reset
pend3:	fmove.l	8(a0),fp0		; come again behind the reset: interrupted here
	nop
	nop
	fmove.d	fp0,40(a3)
	bra	fin3

tail:	fmove.d	fp1,48(a3)		; the registers came back
fin3:	REC	T_CS_VEC,SWVEC
	REC	T_CS_IA,SWIA
	REC	T_CS_FMT,SWFMT
	REC	T_CS_CCR,SWCCR
	move.l	IRQCNT,d1
	REC	T_CS_CNT,d1
	bra	finish

nullf:	dc.l	0

; the "kernel": the interrupt switches to another process and back
h_sw:	movem.l	d0/a0-a1,-(sp)
	addq.l	#1,IRQCNT
	moveq	#0,d0
	move.w	12+6(sp),d0
	move.l	d0,SWVEC
	move.l	12+8(sp),SWIA
	lea	PCB,a0
	dc.w	$f310			; fsave (a0)
	moveq	#0,d0
	move.w	(a0),d0
	move.l	d0,SWFMT
	move.w	4(a0),d0
	move.l	d0,SWCCR
	tst.b	(a0)
	beq.s	sw1			; a null frame: no registers to keep
	fmovem.x fp0-fp7,64(a0)
	fmovem.l fpcr/fpsr/fpiar,160(a0)
sw1:	lea	nullf,a1
	dc.w	$f351			; frestore (a1): the other process, from a null FPU
	fmove.l	#1000,fp0
	fmove.l	#7,fp1
	fdiv.x	fp1,fp0
	fmove.d	fp0,OTHER
	lea	PCB2,a1
	dc.w	$f311			; fsave (a1)
	tst.b	(a0)
	beq.s	sw2
	fmovem.l 160(a0),fpcr/fpsr/fpiar
	fmovem.x 64(a0),fp0-fp7
sw2:	dc.w	$f350			; frestore (a0)
	movem.l	(sp)+,d0/a0-a1
	rte
