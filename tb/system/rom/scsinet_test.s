; Bare test ROM (TT mode, --net): the DaynaPORT at SCSI ID 4 through the TT's 5380
; ($FFFF8781 + 2 x register) in programmed I/O: INQUIRY, MAC address, enable, send a
; 60-byte frame, receive one frame.  The results go to $1000 as text (--text 1000);
; $FFC holds $D0E0600D at the end (--done FFC).
;   vasmm68k_mot -Fbin -quiet -o scsinet_test.bin rom/scsinet_test.s   (a vasm built with OUTBIN)
NCR	equ	$FFFF8781
	org	$E00000
	dc.l	$00010000		; SSP
	dc.l	start
start:	move.w	#$2700,sr
	lea	$1000,a5		; text out
	lea	NCR,a6
	clr.b	2*2(a6)			; MR: no DMA
	clr.b	1*2(a6)			; ICR
	clr.b	3*2(a6)			; TCR

	lea	c_inq(pc),a0		; INQUIRY: "Dayna   SCSI/Link       2.0f"
	bsr	cmd
	lea	t_inq(pc),a0
	bsr	puts
	move.b	dbuf,d0			; device type, then vendor, product, revision
	bsr	hex2
	move.b	#' ',(a5)+
	lea	dbuf+8,a0
	moveq	#27,d0
.inq:	move.b	(a0)+,(a5)+
	dbra	d0,.inq
	bsr	stat

	lea	c_mac(pc),a0		; MAC address
	bsr	cmd
	lea	t_mac(pc),a0
	bsr	puts
	lea	dbuf,a0
	moveq	#5,d3
.mac:	move.b	(a0)+,d0
	bsr	hex2
	dbra	d3,.mac
	bsr	stat

	lea	c_ena(pc),a0		; enable
	bsr	cmd
	lea	t_ena(pc),a0
	bsr	puts
	bsr	stat

	lea	frame(pc),a1		; send 60 bytes
	lea	c_send(pc),a0
	bsr	cmd
	lea	t_send(pc),a0
	bsr	puts
	bsr	stat

	lea	c_recv(pc),a0		; receive (1524 bytes allowed)
	bsr	cmd
	lea	t_recv(pc),a0
	bsr	puts
	move.w	d4,d0			; bytes in
	bsr	hex4
	move.b	#':',(a5)+
	lea	dbuf,a0
	moveq	#19,d3
.rcv:	move.b	#' ',(a5)+
	move.b	(a0)+,d0
	bsr	hex2
	dbra	d3,.rcv
	bsr	stat

	lea	c_recv(pc),a0		; a second receive: the queue is empty, six zeros
	bsr	cmd
	lea	t_recv(pc),a0
	bsr	puts
	move.w	d4,d0
	bsr	hex4
	bsr	stat

	clr.b	(a5)
	move.l	#$D0E0600D,$FFC.w
.end:	bra.s	.end

; one command, cdb at a0, data out from a1; data in to dbuf (d4 = count), d5 = status
cmd:	move.b	#$10,(a6)		; ODR: ID 4
	move.b	#$05,1*2(a6)		; ICR: data bus, SEL
.bsy:	btst	#6,4*2(a6)		; BSY from the target
	beq.s	.bsy
	clr.b	1*2(a6)			; SEL off
	lea	dbuf,a2
	moveq	#0,d4
	moveq	#-1,d5
.ph:	move.b	4*2(a6),d0
	btst	#6,d0			; bus free: done
	beq.s	.done
	btst	#5,d0			; REQ
	beq.s	.ph
	lsr.b	#2,d0
	and.b	#7,d0			; MSG C/D I/O
	move.b	d0,3*2(a6)		; TCR = the phase
	btst	#0,d0
	bne.s	.in
	cmp.b	#2,d0
	beq.s	.cmdb
	move.b	(a1)+,(a6)		; data out
	bra.s	.out
.cmdb:	move.b	(a0)+,(a6)		; command
.out:	move.b	#$11,1*2(a6)		; ACK + data bus
.w1:	btst	#5,4*2(a6)
	bne.s	.w1
	clr.b	1*2(a6)
	bra.s	.ph
.in:	move.b	(a6),d1			; the byte on the bus
	cmp.b	#1,d0
	bne.s	.nd
	move.b	d1,(a2)+		; data in
	addq.w	#1,d4
	bra.s	.ack
.nd:	cmp.b	#3,d0
	bne.s	.ack
	move.b	d1,d5			; status
.ack:	move.b	#$10,1*2(a6)		; ACK
.w2:	btst	#5,4*2(a6)
	bne.s	.w2
	clr.b	1*2(a6)
	bra.s	.ph
.done:	rts

stat:	lea	t_st(pc),a0
	bsr	puts
	move.b	d5,d0
	bsr	hex2
	move.b	#10,(a5)+
	rts

puts:	move.b	(a0)+,d0
	beq.s	.x
	move.b	d0,(a5)+
	bra.s	puts
.x:	rts

hex4:	move.w	d0,-(sp)
	lsr.w	#8,d0
	bsr	hex2
	move.w	(sp)+,d0
hex2:	move.b	d0,d1
	lsr.b	#4,d0
	bsr	hex1
	move.b	d1,d0
hex1:	and.b	#15,d0
	add.b	#'0',d0
	cmp.b	#'9',d0
	bls.s	.d
	addq.b	#7,d0
.d:	move.b	d0,(a5)+
	rts

c_inq:	dc.b	$12,0,0,0,36,0
c_mac:	dc.b	$09,0,0,0,18,0
c_ena:	dc.b	$0E,0,0,0,0,$80
c_send:	dc.b	$0A,0,0,0,60,0
c_recv:	dc.b	$08,0,0,$05,$F4,0
frame:	dc.b	$FF,$FF,$FF,$FF,$FF,$FF,$00,$80,$19,$4D,$53,$54,$08,$06
	dcb.b	46,$A5
t_inq:	dc.b	"inquiry: ",0
t_mac:	dc.b	"mac: ",0
t_ena:	dc.b	"enable",0
t_send:	dc.b	"send",0
t_recv:	dc.b	"receive ",0
t_st:	dc.b	" status ",0
	even
dbuf	equ	$2000
