; Bare test ROM (ap060): is $06D0 $0000 (CALLM on a 68020, Apollo's addiw.l)
; an illegal instruction?  FPSP060's CPU check relies on it.
;   $400 $D0E0600D; $404 d0: 60 = it trapped (a real 68060), 40 = it executed
	org	$E00000
	dc.l	$00010000
	dc.l	start
start:	move.w	#$2700,sr
	lea	trapped(pc),a1
	move.l	a1,$10			; illegal instruction
	move.l	a1,$2c			; line F
	move.l	a1,$f4			; unimplemented integer instruction
	move.l	a1,$08			; access error
	lea	$1000,a0
	move.l	sp,a2
	moveq	#60,d0
	dc.w	$06d0,$0000
	moveq	#40,d0
trapped:
	move.l	a2,sp
	move.l	d0,$404
	move.l	#$D0E0600D,$400
.h:	bra.s	.h
