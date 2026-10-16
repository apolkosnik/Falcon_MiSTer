; blitbench.s - blitter speed in the full-system simulation (tb/system)
; A stand-in ROM, no TOS: five 320x200 blits of 4 interleaved planes (80
; words a line, 16,000 words each) from $40000 to $80000, and for each the
; system clocks (real time) and the processor clocks of Falcon time it took,
; from the timing counters (falcon_tstat, sim_main --tstat), printed in
; decimal to TEXT (sim_main --text); then $D0E0600D to DONE (--done).
;   copy    HOP source, LOP source: a read and a write per word
;   fill    HOP all ones: a write per word
;   or      LOP source OR destination: two reads and a write
;   shift   skew 4, FXSR, end masks $0FFF/$F000, LOP OR
;   nonhog  copy in non-hog mode (64 accesses, then the CPU's share)
; In Falcon mode at 16 MHz Hatari's time is two system clocks per processor
; clock of Falcon time.
;
;   vasmm68k_mot -Fbin -m68030 -no-opt -o blitbench.img blitbench.s

TEXT	equ	$100000
DONE	equ	$0FFFF0
BLT	equ	$FF8A00
SRCB	equ	$40000
DSTB	equ	$80000

	org	$E00000
	dc.l	$00008000		; reset: SSP
	dc.l	start			; reset: PC

start:	move.w	#$2700,sr
	lea	$8000,sp
	lea	TEXT,a5
	lea	SRCB,a0			; a source pattern, a line more for FXSR
	move.l	#$12345678,d0
	move.w	#8200-1,d1
.f:	move.l	d0,(a0)+
	rol.l	#3,d0
	addq.l	#1,d0
	dbra	d1,.f
	lea	BLT,a4

	lea	n_copy(pc),a0
	move.l	#$02030000,d0		; HOP, LOP, skew
	move.l	#$ffffffff,d1		; end masks 1, 2
	move.w	#$ffff,d2		; end mask 3
	move.b	#$c0,d3			; busy, hog
	bsr	runblit
	lea	n_fill(pc),a0
	move.l	#$00030000,d0
	move.b	#$c0,d3
	bsr	runblit
	lea	n_or(pc),a0
	move.l	#$02070000,d0
	move.b	#$c0,d3
	bsr	runblit
	lea	n_shft(pc),a0
	move.l	#$02078400,d0
	move.l	#$0fffffff,d1
	move.w	#$f000,d2
	move.b	#$c0,d3
	bsr	runblit
	lea	n_nhog(pc),a0
	move.l	#$02030000,d0
	move.l	#$ffffffff,d1
	move.w	#$ffff,d2
	move.b	#$80,d3			; busy
	bsr	runblit
	clr.b	(a5)
	move.l	#$D0E0600D,DONE
.end:	bra.s	.end

; a0 name, d0 HOP/LOP/skew, d1 end masks 1/2, d2 end mask 3, d3 control
runblit:
	move.w	#2,$20(a4)		; source X/Y increment
	move.w	#2,$22(a4)
	move.l	#SRCB,$24(a4)
	move.l	d1,$28(a4)
	move.w	d2,$2c(a4)
	move.w	#2,$2e(a4)		; destination X/Y increment
	move.w	#2,$30(a4)
	move.l	#DSTB,$32(a4)
	move.w	#80,$36(a4)		; X count
	move.w	#200,$38(a4)		; Y count
	move.l	d0,d4
	swap	d4
	move.w	d4,$3a(a4)		; HOP, LOP
	move.l	d0,d4
	lsr.w	#8,d4
	move.b	d4,$3d(a4)		; skew
	move.l	$fff008,d6		; RTIME
	move.l	$fff004,d7		; FTIME
	move.b	d3,$3c(a4)		; start
	nop
.w:	tst.b	$3c(a4)
	bmi.s	.w
	move.l	$fff008,d4
	move.l	$fff004,d5
	sub.l	d6,d4
	sub.l	d7,d5
.n:	move.b	(a0)+,(a5)+
	bne.s	.n
	move.b	#' ',-1(a5)
	move.l	d4,d0
	bsr.s	pdec
	move.b	#' ',(a5)+
	move.l	d5,d0
	bsr.s	pdec
	move.b	#10,(a5)+
	rts

; d0.l in decimal to (a5)+ (d1, d2 kept: the next blit's end masks)
pdec:	movem.l	d1-d2,-(sp)
	moveq	#0,d2
.d:	divul.l	#10,d1:d0
	add.w	#'0',d1
	move.w	d1,-(sp)
	addq.w	#1,d2
	tst.l	d0
	bne.s	.d
.o:	move.w	(sp)+,d1
	move.b	d1,(a5)+
	subq.w	#1,d2
	bne.s	.o
	movem.l	(sp)+,d1-d2
	rts

n_copy:	dc.b	'copy',0
n_fill:	dc.b	'fill',0
n_or:	dc.b	'or',0
n_shft:	dc.b	'shift',0
n_nhog:	dc.b	'nonhog',0
	even
