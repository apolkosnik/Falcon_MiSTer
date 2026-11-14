; Bare test ROM (+serloop): SCC channel B set up as TOS 4.04 sets it (the values from a
; trace of its boot, but time constant 0): the vector with the status in it (WR9 = $09),
; receive, transmit and external/status interrupts on (WR1 = $17, WR15 = $20).
; Three characters go out, the second and third from the transmit interrupt, and come
; back by the receive interrupt.  The counts of each vector's interrupts are printed:
; transmit ($60), external ($62), receive ($64), special ($66).  Text at $1000, $FFC done.
;   vasmm68k_mot -Fbin -quiet -o scc_tos_test.bin scc_tos_test.s
W	macro
	move.b	#\1,(a0)
	move.b	#\2,(a0)
	endm
	org	$E00000
	dc.l	$00010000
	dc.l	start
start:	move.w	#$2700,sr
	lea	$1000,a5
	lea	$2000,a4		; received
	lea	$2100,a3		; counts: tx, ext, rx, special (longs)
	clr.l	(a3)
	clr.l	4(a3)
	clr.l	8(a3)
	clr.l	12(a3)
	move.l	#txint,$180.w
	move.l	#exint,$188.w
	move.l	#rxint,$190.w
	move.l	#spint,$198.w
	lea	msg+1(pc),a2		; the next character to send
	lea	$FFFF8C85,a0
	move.b	(a0),d0
	W	9,$40
	W	4,$44
	W	1,$04
	W	2,$60
	W	3,$c0
	W	5,$e2
	W	9,$01
	W	10,$00
	W	11,$50
	W	12,$00
	W	13,$00
	W	14,$02
	W	14,$03
	W	3,$c1
	W	5,$ea
	W	15,$20
	move.b	#$10,(a0)
	move.b	#$10,(a0)
	W	1,$17
	W	9,$09
	move.w	#$2400,sr
	move.b	msg(pc),2(a0)		; the first character
	move.l	#20000,d2
w:	subq.l	#1,d2
	bne.s	w
	move.w	#$2700,sr
	lea	t_tx(pc),a1
	moveq	#3,d7
rep:	bsr	puts
	move.l	(a3)+,d0
	bsr	hex8
	move.b	#10,(a5)+
	dbra	d7,rep
	bsr	puts			; "received "
	move.l	$2000,(a5)+
	move.b	#10,(a5)+
	clr.b	(a5)
	move.l	#$D0E0600D,$FFC.w
hang:	bra.s	hang

storm:	cmp.l	#500,d0			; an interrupt that will not go away: stop taking them
	bcs.s	st1
	or.w	#$0700,(sp)
st1:	rts
txint:	addq.l	#1,(a3)
	move.l	d0,-(sp)
	move.b	(a2)+,d0
	beq.s	tx1
	move.b	d0,2(a0)
	bra.s	tx2
tx1:	subq.l	#1,a2
	move.b	#$28,(a0)		; reset Tx interrupt pending
tx2:	move.b	#$38,(a0)		; reset highest IUS
	move.l	(a3),d0
	move.l	(sp)+,d0
	rte
exint:	addq.l	#1,4(a3)
	move.b	#$10,(a0)		; reset external/status interrupts
	move.b	#$38,(a0)
	cmp.l	#500,4(a3)
	bcs.s	ex1
	or.w	#$0700,(sp)
ex1:	rte
rxint:	addq.l	#1,8(a3)
	move.b	2(a0),(a4)+
	move.b	#$38,(a0)
	cmp.l	#500,8(a3)
	bcs.s	rx1
	or.w	#$0700,(sp)
rx1:	rte
spint:	addq.l	#1,12(a3)
	move.b	#$30,(a0)		; error reset
	move.b	2(a0),(a4)+
	move.b	#$38,(a0)
	cmp.l	#500,12(a3)
	bcs.s	sp1
	or.w	#$0700,(sp)
sp1:	rte
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
	bls.s	h2
	addq.b	#7,d1
h2:	move.b	d1,(a5)+
	dbra	d2,h1
	rts
msg:	dc.b	'SCC',0
t_tx:	dc.b	'transmit ',0
	dc.b	'external ',0
	dc.b	'receive  ',0
	dc.b	'special  ',0
	dc.b	'received ',0
