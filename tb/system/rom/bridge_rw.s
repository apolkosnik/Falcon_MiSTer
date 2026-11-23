; Bare test ROM (ap040/ap060 builds): the bus bridge's posted writes and
; burst line reads with their line buffer (falcon_cpubus040/060).
; Assemble: vasmm68k_mot -Fbin -m68040 -o bridge_rw.bin bridge_rw.s
; Run:      --done 400 --dump 400:10:FILE
;   $400  $D0E0600D when it stopped
;   $404  0: every check passed, else the number of the failed check
;   $408  the value read, $40C the value expected (a data check)
;
;   1-9    caches off: back-to-back posted long/word/byte writes (the next
;          write waits on the bus for the posted one), read back; a write
;          read back at once
;   10-14  writes that must end in a bus error, not be posted: ROM, the
;          reset vectors, a hole in the map, the supervisor page from user
;          mode; and a posted write right before one of them
;   20-27  caches on (translation off, no TTRs: write-through cachable):
;          line reads of both lines of a 32-byte block, lower line first
;          and upper line first (the second from the line buffer); ROM
;   30-31  the line buffer dropped by the processor's own write
;   40-42  the line buffer dropped by a blitter write (DMA)
;   60-61  the line buffer dropped by a line read of the vectors' 32 bytes
;          (two 64-bit reads into the same read RAM, rows A4:A3)
;          (all three run twice, the second time with the code in the
;          instruction cache: no instruction fetch replaces the buffer)
;   50-53  posted line writes: a copyback push of dirty lines, MOVE16,
;          read back uncached and through the blitter
;   98     an unexpected exception

DONE	equ	$400
BLT	equ	$FF8A00
ST1	equ	$00100000
ST2	equ	$00200000
ST3	equ	$00300000

failt	macro
	move.l	#\1,d7
	bra	fail
	endm

; \1: register read, \2: expected (register or immediate), \3: check number
vchk	macro
	cmp.l	\2,\1
	beq.s	.ok\@
	move.l	\1,$408
	move.l	\2,$40C
	failt	\3
.ok\@:
	endm

; a write that must end in a bus error: \1 the instruction, \2 check number
berr	macro
	move.l	sp,a5
	lea	.c\@(pc),a6
	moveq	#0,d6
	\1
	nop
	failt	\2
.c\@:	cmp.l	#1,d6
	beq.s	.k\@
	failt	\2
.k\@:
	endm

; the blitter: four words from (a0) to (a1), HOG mode, waits for the end.
; Inline (no BSR/RTS: with the data cache on, the RTS's read of the stack
; is a line read, which would replace the line buffer under test).  The
; data cache is off meanwhile: with no TTRs the I/O area is cachable, and a
; cachable read of a register is a line read (a bus error there); the
; instruction cache stays on, or its fetches would replace the line buffer
blit4	macro
	movec	cacr,d3
	move.l	#$00008000,d4		; the instruction cache stays on (IE)
	movec	d4,cacr
	lea	BLT,a2
	move.w	#2,$20(a2)		; source X increment
	move.w	#2,$22(a2)		; source Y increment
	move.l	a0,$24(a2)		; source address
	move.w	#$FFFF,$28(a2)		; end masks
	move.w	#$FFFF,$2A(a2)
	move.w	#$FFFF,$2C(a2)
	move.w	#2,$2E(a2)		; destination X increment
	move.w	#2,$30(a2)		; destination Y increment
	move.l	a1,$32(a2)		; destination address
	move.w	#4,$36(a2)		; X count
	move.w	#1,$38(a2)		; Y count
	move.b	#2,$3A(a2)		; HOP: source
	move.b	#3,$3B(a2)		; OP: source
	move.b	#0,$3D(a2)		; skew
	move.b	#$C0,$3C(a2)		; BUSY, HOG
.w\@:	tst.b	$3C(a2)
	bmi.s	.w\@
	movec	d3,cacr
	endm

	org	$E00000
	dc.l	$00010000
	dc.l	start
start:	move.w	#$2700,sr
	cinva	bc
	moveq	#0,d0
	movec	d0,cacr
	movec	d0,tc
	movec	d0,itt0
	movec	d0,itt1
	movec	d0,dtt0
	movec	d0,dtt1
	movec	d0,vbr
	lea	$8,a0			; every vector but the reset pair: unexp
	move.w	#256-2-1,d1
.v:	move.l	#unexp,(a0)+
	dbra	d1,.v
	move.l	#buserr,$8
	clr.l	$404
	clr.l	$408
	clr.l	$40C

;------------------------------------------------ 1-9 posted writes, caches off
	lea	ST1,a0
	move.l	#$11223344,(a0)+	; back to back: each waits for the last
	move.l	#$55667788,(a0)+
	move.l	#$99AABBCC,(a0)+
	move.l	#$DDEEFF00,(a0)+
	move.w	#$1234,(a0)+
	move.w	#$5678,(a0)+
	move.b	#$9A,(a0)+
	move.b	#$BC,(a0)+
	move.b	#$DE,(a0)+
	move.b	#$F0,(a0)+
	lea	ST1,a0
	move.l	(a0)+,d0
	vchk	d0,#$11223344,1
	move.l	(a0)+,d0
	vchk	d0,#$55667788,2
	move.l	(a0)+,d0
	vchk	d0,#$99AABBCC,3
	move.l	(a0)+,d0
	vchk	d0,#$DDEEFF00,4
	move.l	(a0)+,d0
	vchk	d0,#$12345678,5
	move.l	(a0)+,d0
	vchk	d0,#$9ABCDEF0,6
	; a write read back at once, and a read-modify-write
	move.l	#$CAFEBABE,ST1+$40
	move.l	ST1+$40,d0
	vchk	d0,#$CAFEBABE,7
	addq.l	#1,ST1+$40
	move.l	ST1+$40,d0
	vchk	d0,#$CAFEBABF,8
	; a long run of writes, then every one read back
	lea	ST1+$1000,a0
	move.w	#255,d1
	move.l	#$01020304,d2
.w9:	move.l	d2,(a0)+
	add.l	#$01010101,d2
	dbra	d1,.w9
	lea	ST1+$1000,a0
	move.w	#255,d1
	move.l	#$01020304,d2
.r9:	move.l	(a0)+,d0
	vchk	d0,d2,9
	add.l	#$01010101,d2
	dbra	d1,.r9

;------------------------------------------------ 10-14 writes that are bus errors
	berr	<move.l d0,$00E00100>,10	; ROM
	berr	<move.l d0,$00000004>,11	; the reset vectors
	berr	<move.l d0,$00F80000>,12	; nothing there
	move.l	#$5A5A5A5A,ST1+$80		; posted, then the error
	berr	<move.w d0,$00E00200>,13
	move.l	ST1+$80,d0
	vchk	d0,#$5A5A5A5A,13
	; user mode: the supervisor page
	move.l	sp,a5
	lea	.c14(pc),a6
	moveq	#0,d6
	lea	$D00000,a0
	move.l	a0,usp
	move.w	#$0700,sr		; user mode
	move.l	d0,$00000600
	nop
	failt	14			; (user mode: fail only writes, no supervisor needed)
.c14:	cmp.l	#1,d6
	beq.s	.k14
	failt	14
.k14:

;------------------------------------------------ 20-27 line reads, caches on
	; the data: ST2.. long words ST2+4*i = $A0000000+i
	lea	ST2,a0
	move.l	#$A0000000,d2
	move.w	#511,d1
.f20:	move.l	d2,(a0)+
	addq.l	#1,d2
	dbra	d1,.f20
	cinva	bc
	move.l	#$80008000,d0
	movec	d0,cacr
	; lower line first: $200000 is a burst, $200010 from the line buffer
	move.l	ST2+$00,d0
	vchk	d0,#$A0000000,20
	move.l	ST2+$1C,d0
	vchk	d0,#$A0000007,21
	move.l	ST2+$14,d0
	vchk	d0,#$A0000005,22
	; upper line first: $200030 a burst, $200020 from the buffer
	move.l	ST2+$38,d0
	vchk	d0,#$A000000E,23
	move.l	ST2+$24,d0
	vchk	d0,#$A0000009,24
	move.l	ST2+$20,d0
	vchk	d0,#$A0000008,25
	; ROM: a table at a 32-byte boundary, both lines
	lea	romtab(pc),a0
	move.l	12(a0),d0
	vchk	d0,#$C0DE0003,26
	move.l	20(a0),d0
	vchk	d0,#$C0DE0005,27

;------------------------------------------------ 30-42 the line buffer dropped by writes
; Twice, with other data the second time: the first pass brings this code
; into the instruction cache, so on the second no instruction fetch (a ROM
; line read) replaces the line buffer between the read that fills it and
; the read that must not use it; a stale buffer returns the first pass's.
	moveq	#1,d5
.p30:
	; 30-31 the processor's own write
	cinva	dc
	move.l	ST2+$40,d0		; $200040 a burst, $200050 in the buffer
	vchk	d0,#$A0000010,30
	move.l	#$12121212,d2
	add.l	d5,d2
	move.l	d2,ST2+$54		; write-through, no allocate: on the bus
	cinva	dc
	move.l	ST2+$54,d0		; must not come from the buffer
	vchk	d0,d2,31
	; 40-42 a DMA write (the blitter)
	move.l	d5,d2
	lsl.l	#8,d2
	add.l	#$BEEF0001,d2
	move.l	d2,ST3			; the blitter's source
	addq.l	#1,d2
	move.l	d2,ST3+4
	cinva	dc
	move.l	ST2+$60,d0		; $200060 a burst, $200070 in the buffer
	vchk	d0,#$A0000018,40
	lea	ST3,a0
	lea	ST2+$74,a1
	blit4			; 4 words (2 longs) ST3 -> $200074
	cinva	dc
	move.l	ST2+$78,d0
	vchk	d0,d2,42
	subq.l	#1,d2
	move.l	ST2+$74,d0
	vchk	d0,d2,41
	; 60-61 a line read of $10-$1F (the vectors' 32 bytes: not a burst, it
	; overwrites the read RAM's rows of A4 = 1, which hold $200090-$20009F)
	cinva	dc
	move.l	ST2+$80,d0		; $200080 a burst: all 32 bytes in the read RAM
	vchk	d0,#$A0000020,60
	move.l	$14,d1			; a line read of $10-$1F
	cinva	dc
	move.l	ST2+$98,d0		; must not come from the read RAM
	vchk	d0,#$A0000026,61
	dbra	d5,.p30

;------------------------------------------------ 50-53 posted line writes
	move.l	#$0000C020,d0		; DTT0: $00xxxxxx copyback
	movec	d0,dtt0
	lea	ST3+$100,a0		; allocate four lines, then dirty them
	move.w	#15,d1
.a50:	tst.l	(a0)+
	dbra	d1,.a50
	lea	ST3+$100,a0
	move.w	#15,d1
	move.l	#$D0000000,d2
.d50:	move.l	d2,(a0)+
	addq.l	#1,d2
	dbra	d1,.d50
	cpusha	dc			; four line writes, back to back
	; MOVE16: four lines from ST2 to ST3+$200
	lea	ST2+$100,a0
	lea	ST3+$200,a1
	move16	(a0)+,(a1)+
	move16	(a0)+,(a1)+
	move16	(a0)+,(a1)+
	move16	(a0)+,(a1)+
	moveq	#0,d0
	movec	d0,dtt0
	cpusha	dc
	moveq	#0,d0			; caches off: read memory
	movec	d0,cacr
	cinva	bc
	lea	ST3+$100,a0
	move.w	#15,d1
	move.l	#$D0000000,d2
.r50:	move.l	(a0)+,d0
	vchk	d0,d2,50
	addq.l	#1,d2
	dbra	d1,.r50
	lea	ST3+$200,a0
	move.w	#15,d1
	move.l	#$A0000040,d2
.r51:	move.l	(a0)+,d0
	vchk	d0,d2,51
	addq.l	#1,d2
	dbra	d1,.r51
	; the blitter reads what the line writes left in memory
	lea	ST3+$100,a0
	lea	ST3+$400,a1
	blit4
	move.l	ST3+$400,d0
	vchk	d0,#$D0000000,52
	move.l	ST3+$404,d0
	vchk	d0,#$D0000001,53

	move.l	#$D0E0600D,DONE
.h:	bra.s	.h

fail:	move.l	d7,$404
	move.l	#$D0E0600D,DONE
.h:	bra.s	.h

buserr:	move.l	a5,sp
	addq.l	#1,d6
	jmp	(a6)

unexp:	failt	98

	cnop	0,32
romtab:	dc.l	$C0DE0000,$C0DE0001,$C0DE0002,$C0DE0003
	dc.l	$C0DE0004,$C0DE0005,$C0DE0006,$C0DE0007
