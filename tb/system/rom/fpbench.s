; Bare test ROM (ap040/ap060 builds): FPU throughput, one operation per
; build (vasm -DOP=n), timed by the simulator's done marker.
; Assemble: vasmm68k_mot -Fbin -m68040 -DOP=n [-DCB=1] -o fpbench_n.bin fpbench.s
;           (-DCB=1: DTT0 makes $00xxxxxx copyback)
; Run:      --done 400 --dump 400:10:FILE
;   $400  $D0E0600D when it stopped, $404 the loop count left (0),
;   $408/$40C the result (FP0 as a double)
;   OP 0 integer loop only, 1 FADD, 2 FMUL, 3 FDIV, 4 FSQRT,
;      5 FMOVE.D to memory, 6 FMOVE.D from memory, 7 FMOVE.X FPn,FPm,
;      8 FSGLMUL, 9 FCMP+FBcc, 10 MOVE.L pairs (a double's two longs),
;      11 MOVE.L, 12 FMOVE.S to memory, 13 FMOVE.X to memory
DONE	equ	$400
N	equ	4000
	org	$E00000
	dc.l	$00010000
	dc.l	start
start:	move.w	#$2700,sr
	cinva	bc
	moveq	#0,d0
	movec	d0,tc
	movec	d0,itt0
	movec	d0,itt1
	movec	d0,dtt0
	movec	d0,dtt1
	movec	d0,vbr
	lea	$8,a0
	move.w	#256-2-1,d1
.v:	move.l	#unexp,(a0)+
	dbra	d1,.v
	if CB
	move.l	#$0000C020,d0		; DTT0: $00xxxxxx copyback (-DCB=1)
	movec	d0,dtt0
	endif
	move.l	#$80008000,d0		; both caches on
	movec	d0,cacr
	clr.l	$404
	fmove.l	#0,fpcr
	fmove.d	c_a(pc),fp0
	fmove.d	c_b(pc),fp1
	fmove.d	c_c(pc),fp2
	fmove.d	fp0,$410
	fmove.d	fp1,$418
	fmove.d	fp2,$420
	lea	$2000,a0
	fmove.d	fp2,(a0)
	moveq	#9,d2
outer:	move.w	#N-1,d1
loop:
	if OP==1
	fadd.x	fp1,fp0
	fadd.x	fp1,fp0
	fadd.x	fp1,fp0
	fadd.x	fp1,fp0
	endif
	if OP==2
	fmul.x	fp1,fp0
	fmul.x	fp1,fp0
	fmul.x	fp1,fp0
	fmul.x	fp1,fp0
	endif
	if OP==3
	fdiv.x	fp1,fp0
	fdiv.x	fp1,fp0
	fdiv.x	fp1,fp0
	fdiv.x	fp1,fp0
	endif
	if OP==4
	fsqrt.x	fp2,fp0
	fsqrt.x	fp2,fp0
	fsqrt.x	fp2,fp0
	fsqrt.x	fp2,fp0
	endif
	if OP==5
	fmove.d	fp0,(a0)
	fmove.d	fp0,(a0)
	fmove.d	fp0,(a0)
	fmove.d	fp0,(a0)
	endif
	if OP==6
	fmove.d	(a0),fp0
	fmove.d	(a0),fp0
	fmove.d	(a0),fp0
	fmove.d	(a0),fp0
	endif
	if OP==7
	fmove.x	fp1,fp0
	fmove.x	fp2,fp0
	fmove.x	fp1,fp0
	fmove.x	fp2,fp0
	endif
	if OP==8
	fsglmul.x fp1,fp0
	fsglmul.x fp1,fp0
	fsglmul.x fp1,fp0
	fsglmul.x fp1,fp0
	endif
	if OP==9
	fcmp.x	fp1,fp0
	fbgt	.a
	nop
.a:	fcmp.x	fp0,fp1
	fbgt	.b
	nop
.b:
	endif
	if OP==10
	move.l	d0,(a0)
	move.l	d0,4(a0)
	move.l	d0,(a0)
	move.l	d0,4(a0)
	move.l	d0,(a0)
	move.l	d0,4(a0)
	move.l	d0,(a0)
	move.l	d0,4(a0)
	endif
	if OP==11
	move.l	d0,(a0)
	move.l	d0,(a0)
	move.l	d0,(a0)
	move.l	d0,(a0)
	endif
	if OP==12
	fmove.s	fp0,(a0)
	fmove.s	fp0,(a0)
	fmove.s	fp0,(a0)
	fmove.s	fp0,(a0)
	endif
	if OP==13
	fmove.x	fp0,(a0)
	fmove.x	fp0,(a0)
	fmove.x	fp0,(a0)
	fmove.x	fp0,(a0)
	endif
	dbra	d1,loop
	dbra	d2,outer
	move.w	d1,$404
	fmove.d	fp0,$408
	fmove.l	fpsr,$428
	cpusha	dc
	move.l	#$D0E0600D,DONE
	cpusha	dc		; (copyback: the marker too)
.h:	bra.s	.h
	cnop	0,8
c_a:	dc.l	$3FF00001,$0C6F7A0B	; 1.000001
c_b:	dc.l	$3FEFFFFD,$E7210BE9	; 0.999999
c_c:	dc.l	$40040000,$00000000	; 2.5
unexp:	move.l	#$EEEEEEEE,$404
	move.l	(sp),$408
	move.l	4(sp),$40C
	move.l	#$D0E0600D,DONE
	cpusha	dc
.h:	bra.s	.h
