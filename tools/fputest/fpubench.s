; fpubench.s - FPUBENCH.TOS: speed of the Falcon core's 68882 served by the
; ARM.  Each loop runs N instructions (unrolled by 10) and is timed with the
; 200 Hz system timer; printed: microseconds per instruction and instructions
; per second.  A register-to-register operation releases the CPU at once and
; finishes on the ARM, so a chain of them costs one round trip each; FMOVE
; out waits for the result.
;
; Assembled by build.sh: vasmm68k_mot -Ftos -m68030 -m68882 -no-opt

N		equ	100000			; instructions per test (multiple of 10, N/10 <= 32768)

	text
start:
	lea	title(pc),a0
	bsr	print
	pea	get_fpu(pc)		; the _FPU cookie: without an FPU every
	move.w	#38,-(sp)		; instruction would take the F-line trap
	trap	#14
	addq.l	#6,sp
	tst.w	fpu_cookie
	bne.s	have_fpu
	lea	nofpu(pc),a0
	bsr	print
	bra	done
have_fpu:
	fmove.l	#0,fpcr
	fmove.l	#3,fp1
	fmove.l	#7,fp0

	lea	n_nop(pc),a0
	lea	l_nop(pc),a1
	bsr	bench
	lea	n_add(pc),a0
	lea	l_add(pc),a1
	bsr	bench
	lea	n_mul(pc),a0
	lea	l_mul(pc),a1
	bsr	bench
	lea	n_div(pc),a0
	lea	l_div(pc),a1
	bsr	bench
	lea	n_sqrt(pc),a0
	lea	l_sqrt(pc),a1
	bsr	bench
	lea	n_sin(pc),a0
	lea	l_sin(pc),a1
	bsr	bench
	lea	n_etox(pc),a0
	lea	l_etox(pc),a1
	bsr	bench
	lea	n_ldin(pc),a0
	lea	l_ldin(pc),a1
	bsr	bench
	lea	n_stx(pc),a0
	lea	l_stx(pc),a1
	bsr	bench
	lea	n_stl(pc),a0
	lea	l_stl(pc),a1
	bsr	bench
	lea	n_cond(pc),a0
	lea	l_cond(pc),a1
	bsr	bench
	lea	n_mem(pc),a0
	lea	l_mem(pc),a1
	bsr	bench

done:	lea	t_key(pc),a0
	bsr	print
	move.w	#1,-(sp)		; Cconin
	trap	#1
	addq.l	#2,sp
	clr.w	-(sp)			; Pterm0
	trap	#1

; a0 = name, a1 = loop body (runs N/10 iterations of 10 instructions)
bench:	movem.l	d0-d7/a0-a6,-(sp)
	move.l	a1,a5
	bsr	print
	bsr	ticks
	move.l	d0,d6
	move.l	#N/10-1,d7
	lea	scratch,a4
	jsr	(a5)
	bsr	ticks
	sub.l	d6,d0			; 200 Hz ticks
	move.l	d0,d5
	; us per instruction x 100 = ticks * 5000 * 100 / N
	mulu.l	#500000,d0
	divu.l	#N,d0
	divul.l	#100,d1:d0		; d0 = integer us, d1 = hundredths
	bsr	pdec
	moveq	#'.',d0
	bsr	putc
	move.l	d1,d0
	cmp.l	#10,d0
	bge.s	b_2
	move.l	d0,-(sp)
	moveq	#'0',d0
	bsr	putc
	move.l	(sp)+,d0
b_2:	bsr	pdec
	lea	t_us(pc),a0
	bsr	print
	; per second = N * 200 / ticks
	move.l	#N*200,d0
	tst.l	d5
	beq.s	b_3
	divu.l	d5,d0
b_3:	bsr	pdec
	lea	t_ps(pc),a0
	bsr	print
	movem.l	(sp)+,d0-d7/a0-a6
	rts

; ---- loop bodies: d7 = iterations - 1, a4 = scratch memory ----
l_nop:	rept	10
	fnop
	endr
	dbra	d7,l_nop
	rts
l_add:	rept	10
	fadd.x	fp1,fp0
	endr
	dbra	d7,l_add
	rts
l_mul:	rept	10
	fmul.x	fp1,fp0
	endr
	dbra	d7,l_mul
	rts
l_div:	rept	10
	fdiv.x	fp1,fp0
	endr
	dbra	d7,l_div
	rts
l_sqrt:	rept	10
	fsqrt.x	fp1,fp0
	endr
	dbra	d7,l_sqrt
	rts
l_sin:	rept	10
	fsin.x	fp1,fp0
	endr
	dbra	d7,l_sin
	rts
l_etox:	rept	10
	fetox.x	fp1,fp0
	endr
	dbra	d7,l_etox
	rts
l_ldin:	rept	10
	fmove.l	d7,fp0
	endr
	dbra	d7,l_ldin
	rts
l_stx:	rept	10
	fmove.x	fp0,(a4)
	endr
	dbra	d7,l_stx
	rts
l_stl:	rept	10
	fmove.l	fp1,d0
	endr
	dbra	d7,l_stl
	rts
l_cond:	rept	10
	fsgt	d0
	endr
	dbra	d7,l_cond
	rts
l_mem:	rept	5
	fmove.d	(a4),fp0		; with the result of the previous loop
	fmul.d	(a4),fp0
	endr
	dbra	d7,l_mem
	rts

; ---- helpers ----
get_fpu:				; (supervisor) the _FPU cookie, 0 if none
	clr.l	fpu_cookie
	move.l	$5A0.w,d0
	beq.s	gf_out
	move.l	d0,a0
gf_l:	move.l	(a0)+,d0
	beq.s	gf_out
	move.l	(a0)+,d1
	cmp.l	#'_FPU',d0
	bne.s	gf_l
	move.l	d1,fpu_cookie
gf_out:	rts

ticks:	pea	rd_hz(pc)
	move.w	#38,-(sp)		; Supexec
	trap	#14
	addq.l	#6,sp
	move.l	hz,d0
	rts
rd_hz:	move.l	$4BA.w,hz
	rts

print:	movem.l	d0-d2/a0-a2,-(sp)
	move.l	a0,-(sp)
	move.w	#9,-(sp)		; Cconws
	trap	#1
	addq.l	#6,sp
	movem.l	(sp)+,d0-d2/a0-a2
	rts

putc:	movem.l	d0-d2/a0-a2,-(sp)
	and.w	#$FF,d0
	move.w	d0,-(sp)
	move.w	#2,-(sp)		; Cconout
	trap	#1
	addq.l	#4,sp
	movem.l	(sp)+,d0-d2/a0-a2
	rts

pdec:	movem.l	d0-d2/a0,-(sp)
	lea	decbuf+12,a0
	clr.b	-(a0)
pd_l:	move.l	d0,d1
	divul.l	#10,d2:d1
	add.b	#'0',d2
	move.b	d2,-(a0)
	move.l	d1,d0
	bne.s	pd_l
	bsr	print
	movem.l	(sp)+,d0-d2/a0
	rts

title:	dc.b	13,10,"FPUBENCH - Falcon core 68882 on the ARM (100000 each)",13,10,0
n_nop:	dc.b	"FNOP              ",0
n_add:	dc.b	"FADD FP1,FP0      ",0
n_mul:	dc.b	"FMUL FP1,FP0      ",0
n_div:	dc.b	"FDIV FP1,FP0      ",0
n_sqrt:	dc.b	"FSQRT FP1,FP0     ",0
n_sin:	dc.b	"FSIN FP1,FP0      ",0
n_etox:	dc.b	"FETOX FP1,FP0     ",0
n_ldin:	dc.b	"FMOVE.L D7,FP0    ",0
n_stx:	dc.b	"FMOVE.X FP0,(A4)  ",0
n_stl:	dc.b	"FMOVE.L FP1,D0    ",0
n_cond:	dc.b	"FSGT D0           ",0
n_mem:	dc.b	"FMOVE/FMUL.D (A4) ",0
t_us:	dc.b	" us  ",0
t_ps:	dc.b	" /s",13,10,0
t_key:	dc.b	"Press a key.",13,10,0
nofpu:	dc.b	"No FPU reported (_FPU cookie 0): is falcon_fpu running?",13,10,0
	even

	bss
fpu_cookie: ds.l	1
hz:	ds.l	1
decbuf:	ds.b	12
	even
scratch: ds.b	16
