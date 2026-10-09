; t_timing_tos.s - the timing sequences as a TOS program, for Hatari (the
; golden): copies timing_body.i to a 256-byte boundary (the instruction
; cache's mapping must be the same as on the core, t_timing_rom.s) and runs
; it in supervisor mode with interrupts masked.
	text
start:	pea	sup(pc)
	move.w	#38,-(sp)		; Supexec
	trap	#14
	addq.l	#6,sp
	clr.w	-(sp)			; Pterm0
	trap	#1

sup:	move.w	sr,-(sp)
	move.w	#$2700,sr
	lea	space(pc),a1
	move.l	a1,d0
	add.l	#255,d0
	and.l	#$FFFFFF00,d0
	move.l	d0,a1
	lea	body(pc),a0
	move.w	#(body_end-body)/2-1,d0
.copy:	move.w	(a0)+,(a1)+
	dbra	d0,.copy
	move.l	a1,d0
	sub.l	#body_end-body,d0
	move.l	d0,a1
	jsr	(a1)
	move.w	#$FFFF,$3F0.w		; the end (the golden harness quits Hatari)
	move.w	(sp)+,sr
	rts

	even
body:
	include	"timing_body.i"
	even
body_end:
space:	ds.b	body_end-body+256
