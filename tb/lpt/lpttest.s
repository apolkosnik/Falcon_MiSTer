; LPTTEST.PRG: 200 numbered lines and then every byte value once, out of the
; printer port through the BIOS (Bconout to device 0), as any program prints.
; It says whether the printer was ready and whether the BIOS gave up on any
; byte, on the screen and in LPTTEST.LOG in the current directory.  From the
; AUTO folder it returns to the boot.
; vasmm68k_mot -Ftos -o LPTTEST.PRG lpttest.s
	lea	logbuf,a4
	pea	hello(pc)
	bsr	say
	clr.w	-(sp)			; Bcostat(0): the printer
	move.w	#8,-(sp)
	trap	#13
	addq.l	#4,sp
	tst.l	d0
	bne.s	go
	pea	notrdy(pc)
	bsr	say
	bra	done
go:	moveq	#0,d6			; bytes the BIOS gave up on
	moveq	#1,d7
line:	lea	ltext(pc),a3
	move.w	d7,d0			; the line number, three digits
	ext.l	d0
	divu	#100,d0
	bsr	digit
	swap	d0
	ext.l	d0
	divu	#10,d0
	bsr	digit
	swap	d0
	bsr	digit
tx:	move.b	(a3)+,d0
	beq.s	eol
	bsr	out
	bra.s	tx
eol:	addq.w	#1,d7
	cmp.w	#200,d7
	bls.s	line
	moveq	#0,d5			; every byte value
bin:	move.w	d5,d0
	bsr	out
	addq.w	#1,d5
	cmp.w	#256,d5
	bne.s	bin
	pea	sent(pc)
	bsr	say
	tst.l	d6
	beq.s	done
	pea	lost(pc)
	bsr	say
done:	clr.w	-(sp)			; Fcreate
	pea	logname(pc)
	move.w	#$3c,-(sp)
	trap	#1
	addq.l	#8,sp
	move.w	d0,d4
	bmi.s	quit
	pea	logbuf
	move.l	a4,d0
	sub.l	#logbuf,d0
	move.l	d0,-(sp)
	move.w	d4,-(sp)
	move.w	#$40,-(sp)		; Fwrite
	trap	#1
	lea	12(sp),sp
	move.w	d4,-(sp)
	move.w	#$3e,-(sp)		; Fclose
	trap	#1
	addq.l	#4,sp
quit:	clr.w	-(sp)
	trap	#1

digit:	move.l	d0,-(sp)
	add.b	#'0',d0
	bsr	out
	move.l	(sp)+,d0
	rts
out:	and.w	#$ff,d0
	move.w	d0,-(sp)
	clr.w	-(sp)
	move.w	#3,-(sp)		; Bconout
	trap	#13
	addq.l	#6,sp
	tst.l	d0
	bne.s	.ok
	addq.l	#1,d6
.ok:	rts
say:	move.l	4(sp),a0
.cp:	move.b	(a0)+,(a4)+
	bne.s	.cp
	subq.l	#1,a4
	move.l	4(sp),-(sp)
	move.w	#9,-(sp)		; Cconws
	trap	#1
	addq.l	#6,sp
	move.l	(sp)+,(sp)
	rts

logname: dc.b	"LPTTEST.LOG",0
hello:	dc.b	"LPTTEST: ",0
notrdy:	dc.b	"the printer is not ready",13,10,0
sent:	dc.b	"sent",13,10,0
lost:	dc.b	"the BIOS gave up on some bytes",13,10,0
ltext:	dc.b	" Atari printer port test: the quick brown fox jumps over the lazy dog",13,10,0
	even
	bss
logbuf:	ds.b	256
