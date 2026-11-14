; Bare test ROM (-DTT for TT mode, where the SCU's masks come first): the end of a DMA sound frame at the ST MFP's Timer A.
; A 64-byte frame on repeat at 50066 Hz mono (1.28 ms a frame); Timer A counts events,
; data 1; its interrupts counted over about 20 ms.  Text at $1000 (--text 1000), $FFC
; holds $D0E0600D at the end (--done FFC).
;   vasmm68k_mot -Fbin -quiet -o snd_test.bin snd_test.s
	org	$E00000
	dc.l	$00010000
	dc.l	start
start:	move.w	#$2700,sr
	lea	$1000,a5
	moveq	#0,d6
	moveq	#0,d4			; interrupts
	move.l	#berr,$8.w
	move.l	#tahnd,$134.w		; vector $4D
	ifd	TT
	move.b	#$7f,$ffff8e01		; the SCU lets the MFP through (TOS: $14, VME $60)
	move.b	#$7f,$ffff8e0d
	endc
	lea	$fffffa01,a6
	move.b	#$48,$16(a6)		; VR $40, software end of interrupt
	lea	$ffff8901,a0
	clr.w	-1(a0)			; a word, as TOS writes it: on an STe or TT the high byte is nothing
	move.b	#$83,$20(a0)		; $FF8921: mono, 50066 Hz
	move.b	#$00,2(a0)		; base $002000
	move.b	#$20,4(a0)
	move.b	#$00,6(a0)
	move.b	#$00,$e(a0)		; end $002040
	move.b	#$20,$10(a0)
	move.b	#$40,$12(a0)
	clr.b	$18(a6)
	move.b	#1,$1e(a6)
	move.b	#8,$18(a6)		; Timer A: event count
	bset	#5,6(a6)
	bset	#5,$12(a6)
	move.b	#3,(a0)			; play, repeat
	move.w	#$2500,sr
	move.l	#40000,d2
w:	subq.l	#1,d2
	bne.s	w
	move.w	#$2700,sr
	clr.b	(a0)
	lea	t_n(pc),a1
	bsr.s	puts
	move.l	d4,d0
	bsr.s	hex8
	move.b	#10,(a5)+
	lea	t_be(pc),a1
	bsr.s	puts
	move.l	d6,d0
	bsr.s	hex8
	move.b	#10,(a5)+
	clr.b	(a5)
	move.l	#$D0E0600D,$FFC.w
hang:	bra.s	hang
tahnd:	addq.l	#1,d4
	move.b	#$df,$fffffa0f
	rte
berr:	addq.l	#1,d6
	move.l	$2(sp),d5		; where
	move.w	#$2700,sr
	lea	t_n(pc),a1
	lea	$1000,a5
	move.l	d5,d0
	bsr.s	hex8
	move.b	#'!',(a5)+
	clr.b	(a5)
	move.l	#$D0E0600D,$FFC.w
h2:	bra.s	h2
puts:	move.b	(a1)+,(a5)+
	bne.s	puts
	subq.l	#1,a5
	rts
hex8:	moveq	#7,d2
h1:	rol.l	#4,d0
	move.b	d0,d1
	and.b	#15,d1
	add.b	#'0',d1
	cmp.b	#'9',d1
	bls.s	h3
	addq.b	#7,d1
h3:	move.b	d1,(a5)+
	dbra	d2,h1
	rts
t_n:	dc.b	'Timer A interrupts ',0
t_be:	dc.b	'bus errors ',0
