; IKDISK.PRG - IKBD interrupts during disk transfers (AUTO folder program).
; Hooks the IKBD mouse vector (Kbdvbase) to count packets and their DX sum,
; then writes, reads back and compares a file over and over, with the buffer
; in ST-RAM and then in Fast RAM (Mxalloc mode 1, when there is any).
; Results at $3C0 (the post-mortem area, unused while nothing crashes):
;   $3C0  $D0E0600D when done        $3C4  mouse packets seen
;   $3C8  DX sum                     $3CC  compare errors
;   $3D0  passes                     $3D4  GEMDOS errors
;   $3D8  Fast RAM buffer address (0: none)

BUFSZ	equ	131072
PASSES	equ	3

	section	text
start:	move.l	4(sp),a5		; basepage: keep text+data+bss+256
	move.l	12(a5),d0
	add.l	20(a5),d0
	add.l	28(a5),d0
	add.l	#$1100,d0
	lea	(a5,d0.l),sp
	move.l	d0,-(sp)
	move.l	a5,-(sp)
	clr.w	-(sp)
	move.w	#$4a,-(sp)		; Mshrink
	trap	#1
	lea	12(sp),sp

	clr.l	-(sp)
	move.w	#$20,-(sp)		; Super
	trap	#1
	addq.l	#6,sp

	lea	$3c0,a0
	moveq	#7,d0
.clr:	clr.l	(a0)+
	dbra	d0,.clr

	move.w	#34,-(sp)		; Kbdvbase
	trap	#14
	addq.l	#2,sp
	move.l	d0,a0
	move.l	16(a0),oldmv
	move.l	a0,kbv
	move.l	#mousevec,16(a0)

	move.l	#BUFSZ,-(sp)		; ST-RAM buffer
	move.w	#$48,-(sp)		; Malloc
	trap	#1
	addq.l	#6,sp
	move.l	d0,bufst
	beq	fail

	move.w	#1,-(sp)		; Fast RAM only
	move.l	#BUFSZ,-(sp)
	move.w	#$44,-(sp)		; Mxalloc
	trap	#1
	addq.l	#8,sp
	tst.l	d0
	bmi.s	.nofr
	move.l	d0,buffr
	move.l	d0,$3d8
.nofr:
	move.l	bufst,a4
	bsr	passes
	move.l	buffr,d0
	beq.s	fin
	move.l	d0,a4
	bsr	passes

fin:	move.l	$4ba.w,d0		; let the last packets in: 200 ms
	add.l	#40,d0
.wt:	cmp.l	$4ba.w,d0
	bhi.s	.wt
	move.l	kbv,a0
	move.l	oldmv,16(a0)
	move.l	#$d0e0600d,$3c0
	clr.w	-(sp)			; Pterm0
	trap	#1

fail:	addq.l	#1,$3d4
	bra.s	fin

; PASSES times: fill (a4) with a pattern, write it, clear it, read it back,
; compare
passes:	moveq	#PASSES-1,d7
.pass:	move.l	a4,a0			; pattern: address xor pass
	move.l	#BUFSZ/4-1,d0
	move.l	d7,d1
	ror.l	#8,d1
.fill:	move.l	a0,d2
	eor.l	d1,d2
	move.l	d2,(a0)+
	subq.l	#1,d0
	bpl.s	.fill

	clr.w	-(sp)
	pea	fname
	move.w	#$3c,-(sp)		; Fcreate
	trap	#1
	addq.l	#8,sp
	tst.l	d0
	bmi	.err
	move.w	d0,d6
	move.l	a4,-(sp)
	move.l	#BUFSZ,-(sp)
	move.w	d6,-(sp)
	move.w	#$40,-(sp)		; Fwrite
	trap	#1
	lea	12(sp),sp
	cmp.l	#BUFSZ,d0
	bne	.err
	move.w	d6,-(sp)
	move.w	#$3e,-(sp)		; Fclose
	trap	#1
	addq.l	#4,sp

	move.l	a4,a0
	move.l	#BUFSZ/4-1,d0
.clr:	clr.l	(a0)+
	subq.l	#1,d0
	bpl.s	.clr

	clr.w	-(sp)
	pea	fname
	move.w	#$3d,-(sp)		; Fopen
	trap	#1
	addq.l	#8,sp
	tst.l	d0
	bmi	.err
	move.w	d0,d6
	move.l	a4,-(sp)
	move.l	#BUFSZ,-(sp)
	move.w	d6,-(sp)
	move.w	#$3f,-(sp)		; Fread
	trap	#1
	lea	12(sp),sp
	cmp.l	#BUFSZ,d0
	bne	.err
	move.w	d6,-(sp)
	move.w	#$3e,-(sp)		; Fclose
	trap	#1
	addq.l	#4,sp

	move.l	a4,a0
	move.l	#BUFSZ/4-1,d0
	move.l	d7,d1
	ror.l	#8,d1
.cmp:	move.l	a0,d2
	eor.l	d1,d2
	cmp.l	(a0)+,d2
	beq.s	.ok
	addq.l	#1,$3cc
.ok:	subq.l	#1,d0
	bpl.s	.cmp
	addq.l	#1,$3d0
	bra.s	.next
.err:	addq.l	#1,$3d4
.next:	dbra	d7,.pass
	rts

; the IKBD mouse vector: a0 = the packet (header, DX, DY)
mousevec:
	move.l	d0,-(sp)
	addq.l	#1,$3c4
	move.b	1(a0),d0
	ext.w	d0
	ext.l	d0
	add.l	d0,$3c8
	move.l	(sp)+,d0
	move.l	oldmv,-(sp)
	rts

	section	data
fname:	dc.b	'C:\IKDISK.DAT',0
	even

	section	bss
oldmv:	ds.l	1
kbv:	ds.l	1
bufst:	ds.l	1
buffr:	ds.l	1
