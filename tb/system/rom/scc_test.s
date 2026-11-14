; Bare test ROM (+serloop: the serial port's TxD wired to its RxD): the SCC's channel B,
; the Falcon's serial port, at $FFFF8C85/87.
;  - a register written and read back (WR12/RR12)
;  - three characters out at 57375 baud (the 3.672 MHz RTxC, time constant 0) and in
;    again, each taken by the receive interrupt: level 5, the vector from WR2 ($60)
; A line for each, "FAIL" on the wrong ones, their count last; text at $1000
; (--text 1000), $FFC holds $D0E0600D at the end (--done FFC).
;   vasmm68k_mot -Fbin -quiet -o scc_test.bin scc_test.s
W	macro
	move.b	#\1,(a0)
	move.b	#\2,(a0)
	endm

	org	$E00000
	dc.l	$00010000		; SSP
	dc.l	start
start:	move.w	#$2700,sr
	lea	$1000,a5		; text out
	moveq	#0,d6			; failures
	lea	$2000,a4		; what comes in
	moveq	#0,d4			; and how many
	move.l	#rxint,$180.w		; vector $60
	lea	$FFFF8C85,a0		; channel B control, its data at +2
	move.b	(a0),d0			; the pointer to register 0
	W	9,$40			; reset channel B
	W	4,$44			; x16 clock, 1 stop bit, no parity
	W	3,$C1			; receive 8 bits, enabled
	W	5,$EA			; send 8 bits, enabled, DTR and RTS
	W	11,$50			; both clocks from the baud rate generator
	W	12,$5A
	move.b	#12,(a0)
	move.b	(a0),d0			; RR12
	lea	t_rr12(pc),a1
	cmp.b	#$5A,d0
	bsr	result
	W	12,$00			; time constant 0
	W	13,$00
	W	14,$01			; the generator on, from RTxC
	W	2,$60			; the vector
	W	1,$10			; an interrupt for every character received
	W	9,$08			; master interrupt enable, the vector as written
	move.w	#$2400,sr

	lea	msg(pc),a2
send:	move.b	(a2)+,d1
	beq.s	sent
	move.l	#20000,d2
s1:	move.b	(a0),d0			; RR0 bit 2: transmit buffer empty
	btst	#2,d0
	bne.s	s2
	subq.l	#1,d2
	bne.s	s1
s2:	move.b	d1,2(a0)
	bra.s	send
sent:	move.l	#20000,d2
s3:	cmp.w	#3,d4
	beq.s	s4
	subq.l	#1,d2
	bne.s	s3
s4:	move.w	#$2700,sr
	lea	t_n(pc),a1
	cmp.w	#3,d4
	bsr	result
	lea	t_rx(pc),a1
	move.l	$2000,d0
	clr.b	d0
	cmp.l	#$53434300,d0		; "SCC"
	bsr	result

	lea	t_fail(pc),a1
	bsr	puts
	move.l	d6,d0
	bsr	hex8
	move.b	#10,(a5)+
	clr.b	(a5)
	move.l	#$D0E0600D,$FFC.w
hang:	bra.s	hang

rxint:	move.b	2(a0),(a4)+		; the character
	addq.w	#1,d4
	move.b	#$38,(a0)		; reset highest IUS
	rte

; a1 = the line's name, Z set = right
result:	beq.s	r1
	bsr	puts
	lea	t_bad(pc),a1
	addq.l	#1,d6
	bra.s	puts
r1:	bsr	puts
	lea	t_ok(pc),a1
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
t_rr12:	dc.b	'RR12 reads what WR12 was given  ',0
t_n:	dc.b	'three receive interrupts        ',0
t_rx:	dc.b	'the characters sent came back   ',0
t_ok:	dc.b	'ok',10,0
t_bad:	dc.b	'FAIL',10,0
t_fail:	dc.b	'failures ',0
