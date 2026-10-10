; t_timing_rom.s - the timing sequences as a ROM for the core (tb_bustime):
; copies timing_body.i to RAM at $20000 (TOS would load a program into RAM)
; and runs it there.  Test device at $FFFF00 (tb_bustime.sv).
TDEV	equ	$FFFF00

	org	$E00000
	dc.l	$8000
	dc.l	start

start:	move.w	#$2700,sr
	lea	8,a0
	move.w	#254-1,d0
.vec:	move.l	#unexp,(a0)+
	dbra	d0,.vec
	lea	body(pc),a0
	lea	$20000,a1
	move.w	#(body_end-body)/2-1,d0
.copy:	move.w	(a0)+,(a1)+
	dbra	d0,.copy
	jsr	$20000
	move.w	#$600D,TDEV
.halt:	bra.s	.halt

unexp:	move.w	#99,TDEV+16
	move.w	#$BAD0,TDEV
.halt:	bra.s	.halt

	even
body:
	include	"timing_body.i"
	even
body_end:
