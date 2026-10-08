; m2macros.i - predicate sequences of the generated cases (tb/fpu/m2.cpp)
; FBcc.W over a MOVEQ: d7 = 1 when taken; the flag byte goes to \2
M2BCC	macro
	moveq	#1,d7
	dc.w	$F280+\1,$0004
	moveq	#0,d7
	move.b	d7,\2
	endm

; FDBcc D6 (count 5): d7 = 1 when the loop branches, d6 = 5 or 4
M2DBCC	macro
	moveq	#5,d6
	moveq	#0,d7
	dc.w	$F24E,\1
	dc.w	t\@-*
	bra.s	n\@
t\@:	moveq	#1,d7
n\@:	move.b	d7,\2
	move.b	d6,\2+1
	endm

; FTRAPcc (no operand): the handler counts vector 7
M2TRAP	macro
	clr.l	TRAPCNT
	dc.w	$F27C,\1
	move.b	TRAPCNT+3,\2
	endm
