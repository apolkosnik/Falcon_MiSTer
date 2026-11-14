; Bare test ROM (TT mode, --vme 1, 2 or 3 with -DM4, --ram 10): the ATW800/2 on the VME bus, as xVDI's
; set_adr() and the service pack's atwfb find it: the id, the bus errors around the card,
; its memory, the LUT, the VTG, the blitter's registers, the bus CPLD's version.
; A line for every read, "FAIL" on the wrong ones, their count last; text at $1000
; (--text 1000), $FFC holds $D0E0600D at the end (--done FFC).
; Then the 2D engine: each blit in one 2 KB area against the same done by the
; processor in another (the real card's rules: atari-sysv-sp1 tools/atw), a line
; with the bytes that differ.  -DNEG: the processor's fill repeats all 64 source
; bytes, which the card does not, so "fill" must FAIL.
;   vasmm68k_mot -Fbin -quiet [-DB=$FEA00000 -DO=$FEC00000 | -DM4] [-DNEG] -o vme_atw_test.bin rom/vme_atw_test.s
	ifnd	B
B	equ	$FEC00000		; the card
O	equ	$FEA00000		; where it is not
	endc
WL	equ	0			; write a long, a word, a byte
WW	equ	1
WB	equ	2
RL	equ	3			; read one, and the value expected
RW	equ	4
RB	equ	5
BE	equ	6			; read a byte: a bus error expected

T	macro
	dc.l	\1,\2,\3
	dc.b	\4
	endm

	org	$E00000
	dc.l	$00010000		; SSP
	dc.l	start
start:	move.w	#$2700,sr
	lea	$1000,a5		; text out
	move.l	#berr,$8.w
	moveq	#0,d6			; failures
	lea	tab(pc),a2
next:	move.l	(a2)+,d5		; what
	bmi	fin
	move.l	(a2)+,a0		; where
	move.l	(a2)+,d1		; value
	move.l	a2,a1			; its name
	addq.l	#8,a2
	moveq	#0,d7
	move.l	sp,a3
	lea	after(pc),a4
	moveq	#0,d0
	cmp.w	#RL,d5
	bcc.s	rd
	tst.w	d5
	bne.s	.w1
	move.l	d1,(a0)
	bra.s	after
.w1:	cmp.w	#WW,d5
	bne.s	.w2
	move.w	d1,(a0)
	bra.s	after
.w2:	move.b	d1,(a0)
	bra.s	after
rd:	bne.s	.r1
	move.l	(a0),d0
	bra.s	after
.r1:	cmp.w	#RW,d5
	bne.s	.r2
	move.w	(a0),d0
	bra.s	after
.r2:	move.b	(a0),d0
after:	nop				; (a posted write's bus error arrives by here)
	cmp.w	#RL,d5
	bcc.s	.rep
	tst.l	d7			; a write: a line only if it bus errored
	beq	next
.rep:	moveq	#7,d3
.lb:	move.b	(a1)+,(a5)+
	dbra	d3,.lb
	move.b	#' ',(a5)+
	tst.l	d7
	beq.s	.val
	lea	t_berr(pc),a0
	bsr	puts
	cmp.w	#BE,d5
	beq.s	.eol
	bra.s	.bad
.val:	bsr	hex8
	cmp.w	#BE,d5
	beq.s	.bad			; a bus error was expected
	cmp.l	d1,d0
	beq.s	.eol
.bad:	addq.l	#1,d6
	lea	t_fail(pc),a0
	bsr	puts
.eol:	move.b	#10,(a5)+
	bra	next

fin:	bsr	pict
	bsr	blits
	lea	t_fails(pc),a0
	bsr	puts
	move.l	d6,d0
	bsr	hex8
	move.b	#10,(a5)+
	clr.b	(a5)
	move.l	#$D0E0600D,$FFC.w
.h:	bra.s	.h

berr:	moveq	#1,d7
	move.l	a3,sp
	jmp	(a4)

; ---- the 2D engine ----
	ifd	M4
MEM	equ	$FEA00000
REG	equ	MEM+$3FF900
SRC	equ	MEM+$210000		; above 2 MB: the 4 MB layout's own
	else
MEM	equ	B
REG	equ	MEM+$1FF900
SRC	equ	MEM+$010000
	endc
AR1	equ	SRC+$10000		; the engine's
AR2	equ	SRC+$20000		; the processor's
BUSY	equ	$FF8			; the status read after the command

BL	macro				; command, flags (1: the source is in the area,
	dc.w	\1,\2			; 2: long enough to be seen busy), source,
	dc.l	\3,\4			; destination, strides, bytes, rows
	dc.w	\5,\6,\7,\8
	dc.b	\9
	endm

; the picture: what the scaler is told (tb_top logs it, "fb:"), 1024x768 in 32 bits from
; $011240 of the card -- bit 21 of the start counts in the 4 MB layout alone, where it is
; $211240 -- then 16 bits, then 640x480 in 8, then off
pict:	lea	REG-$100,a0		; the VTG
	clr.w	(a0)
	move.w	#1024,8(a0)
	move.w	#768,16(a0)
	move.w	#$1240,24(a0)
	move.w	#$0021,26(a0)
	move.w	#$0039,(a0)
	move.w	#$5555,24(a0)		; the start is taken as the VTG goes on, not after
	move.w	#$0029,(a0)
	move.w	#640,8(a0)
	move.w	#480,16(a0)
	move.w	#$0019,(a0)
	clr.w	(a0)
	rts

blits:	move.l	sp,a3
	lea	.berr(pc),a4
	lea	SRC,a0			; 2 KB of noise
	move.w	#2047,d0
	move.w	#$1234,d1
.p:	mulu	#$4E6D,d1
	add.w	#$3039,d1
	move.w	d1,d2
	lsr.w	#7,d2
	move.b	d2,(a0)+
	dbra	d0,.p
	lea	btab(pc),a2
.next:	move.w	(a2)+,d5		; the command
	bmi	.done
	move.w	(a2)+,d4
	lea	AR1,a0			; the two areas alike: $EE, or the noise
	lea	AR2,a1
	lea	SRC,a6
	move.w	#2047,d0
.f:	move.b	#$EE,d1
	btst	#0,d4
	beq.s	.f1
	move.b	(a6),d1
.f1:	addq.l	#1,a6
	move.b	d1,(a0)+
	move.b	d1,(a1)+
	dbra	d0,.f
	move.l	(a2)+,d0		; the source
	move.l	(a2)+,d1		; the destination
	lea	SRC,a6			; and the processor's
	btst	#0,d4
	beq.s	.s1
	lea	AR2,a6
.s1:	add.l	d0,a6
	add.l	#SRC-MEM,d0
	btst	#0,d4
	beq.s	.s2
	add.l	#AR1-SRC,d0
.s2:	lea	AR2,a1
	add.l	d1,a1
	add.l	#AR1-MEM,d1
	lea	REG,a0
	move.l	d0,(a0)
	move.l	d1,4(a0)
	move.l	(a2),8(a0)		; the strides
	move.l	4(a2),12(a0)		; bytes, rows
	move.w	d5,16(a0)
	move.w	16(a0),BUSY.w
	move.l	#400000,d0
.w:	btst	#0,17(a0)
	beq.s	.wd
	subq.l	#1,d0
	bne.s	.w
.wd:	movem.w	(a2)+,d0-d3		; the same by the processor: strides, bytes, rows
	move.w	d4,-(sp)
.row:	moveq	#0,d4
.byte:	move.l	d4,d7
	btst	#2,d5
	beq.s	.nf
	cmp.w	#64,d7			; a fill: 64 bytes, then the second 32 again
	bcs.s	.as
	ifd	NEG
	and.w	#63,d7
	else
	and.w	#31,d7
	add.w	#32,d7
	endc
	bra.s	.as
.nf:	btst	#1,d5
	beq.s	.as
	neg.l	d7			; descending, from the last byte
	move.b	(a6,d7.l),(a1,d7.l)
	bra.s	.nx
.as:	move.b	(a6,d7.l),(a1,d4.l)
.nx:	addq.l	#1,d4
	cmp.l	d2,d4
	bne.s	.byte
	add.l	d0,a6
	add.l	d1,a1
	subq.l	#1,d3
	bne.s	.row
	move.w	(sp)+,d4
	lea	AR1,a0
	lea	AR2,a1
	move.w	#2047,d0
	moveq	#0,d1			; the bytes that differ
.c:	cmpm.b	(a0)+,(a1)+
	beq.s	.c1
	addq.l	#1,d1
.c1:	dbra	d0,.c
	moveq	#7,d3
.lb:	move.b	(a2)+,(a5)+
	dbra	d3,.lb
	move.b	#' ',(a5)+
	move.l	d1,d0
	bsr	hex8
	tst.l	d1
	bne.s	.bad
	btst	#1,d4			; and it was busy
	beq.s	.eol
	btst	#0,BUSY+1.w
	bne.s	.eol
	lea	t_idle(pc),a0
	bsr	puts
.bad:	addq.l	#1,d6
	lea	t_fail(pc),a0
	bsr	puts
.eol:	move.b	#10,(a5)+
	bra	.next
.berr:	lea	t_blit(pc),a0
	bsr	puts
	addq.l	#1,d6
.done:	rts

;		cmd fl source dest  strides   bytes rows
btab:	BL	1,0,$001,$103,256,256,61,3,'copy    '
	BL	1,2,$000,$100,256,256,64,4,'copy alg'
	BL	3,0,$346,$4B0,-256,-256,61,3,'copy dsc'
	BL	5,2,$000,$045,0,256,200,4,'fill    '
	BL	5,0,$003,$7FF,0,0,1,1,'fill 1  '
	BL	5,0,$000,$100,0,64,64,3,'fill 64 '
	BL	1,3,$120,$100,256,256,200,4,'left 32 '
	BL	3,3,$4C7,$4E7,-256,-256,200,4,'right 32'
	BL	1,3,$101,$100,256,256,255,2,'left 1  '
	BL	3,3,$2FE,$2FF,-256,-256,255,2,'right 1 '
	BL	1,3,$200,$100,256,256,256,5,'up a row'
	BL	3,3,$5FF,$6FF,-256,-256,256,5,'down row'
	dc.w	-1

t_idle:	dc.b	' not busy',0
t_blit:	dc.b	'blits: bus error FAIL',10,0
	even

puts:	move.b	(a0)+,d2
	beq.s	.e
	move.b	d2,(a5)+
	bra.s	puts
.e:	rts

hex8:	moveq	#7,d3
	move.l	d0,d2
.l:	rol.l	#4,d2
	move.w	d2,d4
	and.w	#15,d4
	move.b	digits(pc,d4.w),(a5)+
	dbra	d3,.l
	rts
digits:	dc.b	'0123456789ABCDEF'
t_berr:	dc.b	'bus error',0
t_fail:	dc.b	' FAIL',0
t_fails: dc.b	'failures ',0
	even

	ifd	M4
; the 4 MB card at $FEA00000 (--vme 3): its mirrored 2 MB layout, then the 4 MB one
A	equ	$FEA00000
tab:	T	RL,A+$1FF218,$2063706D,'id 2M   '
	T	RL,A+$3FF218,$2063706D,'id 4M   '	; the mirror
	T	BE,A-1,0,'below   '
	T	BE,A+$400000,0,'above   '
	T	WL,A,$11112222,'w       '
	T	RL,A+$200000,$11112222,'mirror  '
	T	WL,A+$200004,$33334444,'w       '
	T	RL,A+4,$33334444,'mirror  '
	T	WW,A+$3FF000,$F81F,'w       '
	T	RW,A+$1FF000,$F81F,'lut mirr'
	T	WW,A+$1FF81E,3,'w       '		; register 15 = 3: 4 MB
	T	WL,A+$1FF218,$DEADBEEF,'w       '	; memory now
	T	RL,A+$1FF218,$DEADBEEF,'ram 2Mtp'
	T	RL,A+$3FF218,$2063706D,'id 4M   '
	T	WL,A+$200000,$55556666,'w       '
	T	RL,A,$11112222,'ram 0   '
	T	RL,A+$200000,$55556666,'ram 2M  '
	T	WL,A+$3FEFFC,$CAFEF00D,'w       '
	T	RL,A+$3FEFFC,$CAFEF00D,'ram top '
	T	RL,A+$1FEFFC,$00000000,'ram 2Mhi'	; not a mirror of it (never written: 0)
	T	RW,A+$3FF000,$F81F,'lut     '
	T	WW,A+$3FF910,$0001,'w       '
	T	RW,A+$3FF910,0,'blit cmd'
	T	RW,$FEFFFAD8,$0004,'cpld ver'
	T	WW,A+$3FF81E,1,'w       '		; and back
	T	RL,A+$1FF218,$2063706D,'id 2M   '
	T	RL,A+$200000,$11112222,'mirror  '
	T	WW,A+$3FF81E,3,'w       '		; the upper half kept its own
	T	RL,A+$200000,$55556666,'ram 2M  '
	dc.l	-1
	else
tab:	T	RL,B+$1FF218,$2063706D,'id      '	; " cpm"
	T	RL,B+$1FF200,$53657572,'id text '	; "Seur"
	T	BE,O+$1FF218,0,'other id'
	T	BE,O,0,'other   '
	T	BE,$FE000000,0,'vme 0   '
	T	BE,$FEE00000,0,'vme E0  '
	T	BE,B&$FFFFFF,0,'st map  '		; not in the 24-bit map
	; the memory
	T	WL,B,$12345678,'w       '
	T	WL,B+4,$9ABCDEF0,'w       '
	T	RL,B,$12345678,'ram 0   '
	T	RL,B+2,$56789ABC,'ram 2   '
	T	WB,B+1,$AA,'w       '
	T	WW,B+6,$BEEF,'w       '
	T	RL,B,$12AA5678,'ram byte'
	T	RL,B+4,$9ABCBEEF,'ram word'
	T	WL,B+$1FEFFC,$CAFEF00D,'w       '
	T	RL,B+$1FEFFC,$CAFEF00D,'ram top '
	T	WL,B+$1FF400,$0BADC0DE,'w       '	; between the registers: memory
	T	RL,B+$1FF400,$0BADC0DE,'ram gap '
	T	RL,B,$12AA5678,'ram 0   '
	; the LUT
	T	WW,B+$1FF000,$F81F,'w       '
	T	WW,B+$1FF1FE,$07E0,'w       '
	T	WB,B+$1FF003,$55,'w       '
	T	WB,B+$1FF002,$AA,'w       '
	T	RW,B+$1FF000,$F81F,'lut 0   '
	T	RW,B+$1FF002,$AA55,'lut 1   '
	T	RW,B+$1FF1FE,$07E0,'lut 255 '
	T	RL,B+$1FF000,$F81FAA55,'lut long'
	T	RB,B+$1FF1FF,$E0,'lut byte'
	; the VTG
	T	WW,B+$1FF800,$0019,'w       '
	T	WW,B+$1FF808,640,'w       '
	T	WW,B+$1FF810,480,'w       '
	T	WB,B+$1FF811,$E1,'w       '
	T	WW,B+$1FF81E,$1234,'w       '
	T	RW,B+$1FF800,$0019,'vtg ctrl'
	T	RW,B+$1FF808,$0280,'vtg hdi '
	T	RW,B+$1FF810,$01E1,'vtg vdi '
	T	RW,B+$1FF81E,$1234,'vtg mem '
	; the blitter's registers, the link
	T	WL,B+$1FF900,$00012345,'w       '
	T	RL,B+$1FF900,0,'blit src'		; write only: every offset reads the status
	T	WW,B+$1FF910,$0003,'w       '
	T	RW,B+$1FF910,$0000,'blit cmd'
	T	RW,B+$1FFAC0,0,'link    '
	; the bus CPLD
	T	RW,$FEFFFAD8,$0004,'cpld ver'
	T	RB,$FEFFFAD9,$04,'cpld low'
	T	RW,$FEFFFA80,0,'c011    '
	T	BE,$FEFFFA7E,0,'a16 low '
	T	BE,$FEFFFB00,0,'a16 high'
	; and the machine around it
	T	RL,B+$1FF218,$2063706D,'id      '
	dc.l	-1
	endc
