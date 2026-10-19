; FDWR.PRG - floppy writes from AUTO on A:.  Writes A:\FDWR_ST.TXT from an
; ST-RAM buffer and A:\FDWR_FR.TXT from a Fast RAM buffer (Mxalloc mode 1,
; skipped without Fast RAM), 6 KB each, reads them back and compares.
; Results at $3C0: $D0E0600D done, $3C4 errors (bit 0 ST create, 1 ST write,
; 2 ST read, 3 ST compare, 4-7 the same for Fast RAM), $3C8 the Fast RAM
; buffer (0: none), $3CC/$3D0 the first GEMDOS error code (ST, Fast RAM)
BUFSZ	equ	6144

	section	text
start:	move.l	4(sp),a5
	move.l	#$1000+BUFSZ*2,-(sp)
	move.l	a5,-(sp)
	clr.w	-(sp)
	move.w	#$4a,-(sp)		; Mshrink
	trap	#1
	lea	12(sp),sp
	lea	stack,sp
	clr.l	-(sp)
	move.w	#$20,-(sp)		; Super
	trap	#1
	addq.l	#6,sp
	lea	$3c0,a0
	moveq	#4,d0
.clr:	clr.l	(a0)+
	dbra	d0,.clr

	lea	bufst,a4
	lea	name_st,a3
	moveq	#0,d6			; error bit base
	lea	$3cc,a2
	bsr	onefile

	move.w	#1,-(sp)		; Fast RAM only
	move.l	#BUFSZ,-(sp)
	move.w	#$44,-(sp)		; Mxalloc
	trap	#1
	addq.l	#8,sp
	move.l	d0,$3c8
	tst.l	d0
	ble.s	.fin
	move.l	d0,a4
	lea	name_fr,a3
	moveq	#4,d6
	lea	$3d0,a2
	bsr	onefile
.fin:	move.l	#$d0e0600d,$3c0
	clr.w	-(sp)
	trap	#1

; write (a4) to the file named (a3), read it back into bufrd, compare;
; error bits from d6 up into $3C4, the first GEMDOS error to (a2)
onefile:
	move.l	a4,a0			; pattern
	move.w	#BUFSZ/4-1,d0
	move.l	#$12345678,d1
.f:	move.l	d1,(a0)+
	rol.l	#5,d1
	addq.l	#7,d1
	dbra	d0,.f
	clr.w	-(sp)
	move.l	a3,-(sp)
	move.w	#$3c,-(sp)		; Fcreate
	trap	#1
	addq.l	#8,sp
	tst.l	d0
	bmi.s	.e0
	move.w	d0,d5
	move.l	a4,-(sp)
	move.l	#BUFSZ,-(sp)
	move.w	d5,-(sp)
	move.w	#$40,-(sp)		; Fwrite
	trap	#1
	lea	12(sp),sp
	move.l	d0,d4
	move.w	d5,-(sp)
	move.w	#$3e,-(sp)		; Fclose
	trap	#1
	addq.l	#4,sp
	cmp.l	#BUFSZ,d4
	bne.s	.e1
	clr.w	-(sp)
	move.l	a3,-(sp)
	move.w	#$3d,-(sp)		; Fopen
	trap	#1
	addq.l	#8,sp
	tst.l	d0
	bmi.s	.e2
	move.w	d0,d5
	pea	bufrd
	move.l	#BUFSZ,-(sp)
	move.w	d5,-(sp)
	move.w	#$3f,-(sp)		; Fread
	trap	#1
	lea	12(sp),sp
	move.l	d0,d4
	move.w	d5,-(sp)
	move.w	#$3e,-(sp)		; Fclose
	trap	#1
	addq.l	#4,sp
	cmp.l	#BUFSZ,d4
	bne.s	.e2b
	move.l	a4,a0
	lea	bufrd,a1
	move.w	#BUFSZ/4-1,d0
.c:	cmpm.l	(a0)+,(a1)+
	bne.s	.e3
	dbra	d0,.c
	rts
.e0:	moveq	#0,d1
	bra.s	.err
.e1:	move.l	d4,d0
	moveq	#1,d1
	bra.s	.err
.e2b:	move.l	d4,d0
.e2:	moveq	#2,d1
	bra.s	.err
.e3:	moveq	#0,d0
	moveq	#3,d1
.err:	add.l	d6,d1
	bset	d1,$3c7
	tst.l	(a2)
	bne.s	.r
	move.l	d0,(a2)
.r:	rts

	section	data
name_st: dc.b	'A:\FDWR_ST.TXT',0
name_fr: dc.b	'A:\FDWR_FR.TXT',0
	even

	section	bss
bufst:	ds.b	BUFSZ
bufrd:	ds.b	BUFSZ
	ds.b	1024
stack:
