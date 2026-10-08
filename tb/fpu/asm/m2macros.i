; m2macros.i - predicate sequences of the generated cases (tb/fpu/m2.cpp)
; FBcc.W over a MOVEQ: d7 = 1 when taken; the flag byte goes to \2
M2BCC	macro
	move.l	#4,SKIP
	move.l	#1,SOFTEXC
	moveq	#1,d7
\3:	dc.w	$F280+\1,$0004
	moveq	#0,d7
	clr.l	SOFTEXC
	move.b	d7,\2
	endm

; FDBcc D6 (count 5): d7 = 1 when the loop branches, d6 = 5 or 4
M2DBCC	macro
	move.l	#6,SKIP
	move.l	#1,SOFTEXC
	moveq	#5,d6
	moveq	#0,d7
\3:	dc.w	$F24E,\1
	dc.w	t\@-*
	bra.s	n\@
t\@:	moveq	#1,d7
n\@:	clr.l	SOFTEXC
	move.b	d7,\2
	move.b	d6,\2+1
	endm

; FTRAPcc (no operand): the handler counts vector 7
M2TRAP	macro
	move.l	#4,SKIP
	move.l	#1,SOFTEXC
	clr.l	TRAPCNT
\3:	dc.w	$F27C,\1
	clr.l	SOFTEXC
	move.b	TRAPCNT+3,\2
	endm
