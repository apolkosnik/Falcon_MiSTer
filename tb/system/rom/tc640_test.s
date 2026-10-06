; Bare test ROM: 124 Beers Later 640x240 true colour mode on VGA.
; CPU fills a 768-word-stride buffer at $100000 (visible 640 words/line,
; modulo area words 640..767 = white), then the blitter copies a 64x64 block
; (checkerboard) into it at x=300,y=100 with a destination modulo.
	org	$E00000
	dc.l	$00010000		; SSP
	dc.l	start
start:	move.w	#$2700,sr
	move.l	#$3111,d0
	movec	d0,cacr		; caches and bursts on
	lea	$FFFF8200,a6
	move.b	#0,$0A(a6)		; sync mode
	move.w	#$018E,$82(a6)		; HHT
	move.w	#$011D,$84(a6)		; HBB
	move.w	#$002D,$86(a6)		; HBE
	move.w	#$0000,$88(a6)		; HDB (set again below as the demo does)
	move.w	#$0119,$8A(a6)		; HDE
	move.w	#$012E,$8C(a6)		; HSS
	move.w	#$0419,$A2(a6)		; VFT
	move.w	#$03FF,$A4(a6)		; VBB
	move.w	#$003F,$A6(a6)		; VBE
	move.w	#$003F,$A8(a6)		; VDB
	move.w	#$03FF,$AA(a6)		; VDE
	move.w	#$0415,$AC(a6)		; VSS
	move.w	#$0080,$0E(a6)		; line offset 128 words
	move.w	#$0280,$10(a6)		; line width 640 words
	move.w	#$0009,$C2(a6)		; VMD
	move.w	#$0186,$C0(a6)		; VCO
	move.w	#$0000,$66(a6)
	move.w	#$0100,$66(a6)		; true colour
	move.w	#$001C,$88(a6)		; HDB
	move.w	#$0135,$8A(a6)		; HDE
; screen base $100000 first, so frames show progress
	move.b	#$10,$01(a6)
	move.b	#$00,$03(a6)
	move.b	#$00,$0D(a6)
; fill buffer with tight loops: line y: 64 words red, 576 words blue (y<120)
; or green, the diagonal word x=2y white, then 128 modulo words white
	lea	$100000,a0
	moveq	#0,d1			; y
fy:	move.w	#63,d2
	move.w	#$F800,d0
f1:	move.w	d0,(a0)+
	dbf	d2,f1
	move.w	#575,d2
	move.w	#$001F,d0
	cmp.w	#120,d1
	blo.s	f2
	move.w	#$07E0,d0
f2:	move.w	d0,(a0)+
	dbf	d2,f2
	move.w	#127,d2
	move.w	#$FFFF,d0
f3:	move.w	d0,(a0)+
	dbf	d2,f3
	move.w	d1,d4
	add.w	d4,d4
	add.w	d4,d4			; byte offset of word 2y
	move.w	#$FFFF,-1536(a0,d4.w)
	addq.w	#1,d1
	cmp.w	#240,d1
	blo.s	fy
; source block 64x64 words checkerboard at $200000
	lea	$200000,a0
	moveq	#0,d1
sy:	moveq	#0,d3
sx:	move.w	d3,d0
	eor.w	d1,d0
	and.w	#8,d0
	beq.s	.k
	move.w	#$FFE0,(a0)+		; yellow
	bra.s	.n
.k:	move.w	#$F81F,(a0)+		; magenta
.n:	addq.w	#1,d3
	cmp.w	#64,d3
	blo.s	sx
	addq.w	#1,d1
	cmp.w	#64,d1
	blo.s	sy
; blitter: copy 64x64 words to screen x=300 y=100, dest stride 768 words
	lea	$FFFF8A00,a5
	move.w	#2,$20(a5)		; src x inc
	move.w	#2,$22(a5)		; src y inc
	move.l	#$200000,$24(a5)	; src addr
	move.w	#$FFFF,$28(a5)
	move.w	#$FFFF,$2A(a5)
	move.w	#$FFFF,$2C(a5)
	move.w	#2,$2E(a5)		; dst x inc
	move.w	#(768-63)*2,$30(a5)	; dst y inc
	move.l	#$100000+100*1536+300*2,$32(a5)
	move.w	#64,$36(a5)		; x count
	move.w	#64,$38(a5)		; y count
	move.b	#2,$3A(a5)		; HOP source
	move.b	#3,$3B(a5)		; OP copy
	move.b	#0,$3D(a5)		; skew
	move.b	#$C0,$3C(a5)		; start, hog
wb:	btst	#7,$3C(a5)
	bne.s	wb
loop:	bra.s	loop
