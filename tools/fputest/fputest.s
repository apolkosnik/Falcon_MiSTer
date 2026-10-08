; fputest.s - FPUTEST.TOS: the Falcon core's 68882 (FPU served by the ARM)
; checked on the real machine.
;
; 1. The generated sequence (gen_fputest.c: every operation, operand format,
;    rounding mode, FMOVECR constant, condition predicate, FMOVEM) runs
;    PASSES times; every pass is compared with the results the same 68882
;    engine produced on the host.  Any difference is a transport/bridge
;    fault: the first ones are listed with their name, expected and got.
; 2. Exceptions: DZ enabled, FDIV by zero, the next FADD takes vector 50
;    (pre-instruction; FPIAR = the FDIV) and the handler's FSAVE/FRESTORE
;    with BIU bit 27 set clears it; OPERR enabled, FMOVE.L of 1e20 stores
;    $7FFFFFFF and takes vector 52 (mid-instruction, format 9).
;
; Assembled by build.sh: vasmm68k_mot -Ftos -m68030 -m68882 -no-opt
; Registers: the generated code uses D3, A1 (results) and A2 (operands).

	ifnd	PASSES
PASSES		equ	20
	endif
MAXSHOW		equ	6

	text
start:
	lea	title(pc),a0
	bsr	print

	; the _FPU cookie (the bridge reports a 68881/2 when falcon_fpu runs)
	pea	get_fpu(pc)
	move.w	#38,-(sp)		; Supexec
	trap	#14
	addq.l	#6,sp
	tst.w	fpu_cookie
	bne.s	have_fpu
	lea	nofpu(pc),a0
	bsr	print
	bra	the_end
have_fpu:
	lea	t_cookie(pc),a0
	bsr	print
	move.l	fpu_cookie,d0
	bsr	phex
	bsr	newline

	; ---- 1. the generated sequence, PASSES times ----
	lea	t_run(pc),a0
	bsr	print
	move.l	#GEN_SLOTS,d0
	bsr	pdec
	lea	t_slots(pc),a0
	bsr	print
	moveq	#PASSES,d0
	bsr	pdec
	lea	t_passes(pc),a0
	bsr	print
	bsr	ticks
	move.l	d0,t_start
	clr.l	nfail
	moveq	#PASSES-1,d7
pass:
	; fill the result buffer with $A5
	lea	results,a0
	move.l	#GEN_SLOTS*4-1,d0
	move.l	#$A5A5A5A5,d1
fill:	move.l	d1,(a0)+
	subq.l	#1,d0
	bpl.s	fill

	lea	results,a1
	lea	gen_vals,a2
	jsr	gen_code

	; compare
	lea	results,a3
	lea	gen_expect,a4
	moveq	#0,d6			; slot index
cmp_slot:
	moveq	#3,d1
	move.l	a3,a5
	move.l	a4,a6
cmp_l:	cmpm.l	(a5)+,(a6)+
	bne.s	cmp_bad
	dbra	d1,cmp_l
cmp_next:
	lea	16(a3),a3
	lea	16(a4),a4
	addq.l	#1,d6
	cmp.l	#GEN_SLOTS,d6
	blt.s	cmp_slot
	moveq	#'.',d0
	bsr	putc
	dbra	d7,pass
	bra	pass_done

cmp_bad:
	addq.l	#1,nfail
	cmp.l	#MAXSHOW,nfail
	bgt.s	cmp_next
	bsr	show_fail
	bra.s	cmp_next

pass_done:
	bsr	ticks
	sub.l	t_start,d0
	move.l	d0,t_total
	bsr	newline
	lea	t_time(pc),a0
	bsr	print
	move.l	t_total,d0
	mulu.l	#5,d0			; 200 Hz ticks -> ms
	divu.l	#PASSES,d0
	bsr	pdec
	lea	t_ms(pc),a0
	bsr	print
	tst.l	nfail
	bne.s	gen_failed
	lea	t_genok(pc),a0
	bsr	print
	bra.s	exc_tests
gen_failed:
	lea	t_genbad(pc),a0
	bsr	print
	move.l	nfail,d0
	bsr	pdec
	bsr	newline

	; ---- 2. exceptions ----
exc_tests:
	lea	t_dz(pc),a0
	bsr	print
	bsr	test_dz
	lea	t_operr(pc),a0
	bsr	print
	bsr	test_operr

	; ---- summary ----
	lea	t_sum_ok(pc),a0
	tst.l	nfail
	bne.s	sum_bad
	tst.l	exc_bad
	beq.s	sum_out
sum_bad:
	lea	t_sum_bad(pc),a0
sum_out:
	bsr	print
the_end:
	ifnd	NOKEY			; (NOKEY: simulation runs from AUTO)
	lea	t_key(pc),a0
	bsr	print
	move.w	#1,-(sp)		; Cconin
	trap	#1
	addq.l	#2,sp
	endif
	clr.w	-(sp)			; Pterm0
	trap	#1

; ---- a mismatch: slot d6 (a3 got, a4 expected) ----
show_fail:
	movem.l	d0-d2/a0-a1,-(sp)
	lea	t_slot(pc),a0
	bsr	print
	move.l	d6,d0
	bsr	pdec
	moveq	#' ',d0
	bsr	putc
	move.l	d6,d0
	mulu.l	#40,d0
	lea	gen_names,a0
	add.l	d0,a0
	moveq	#39,d1
sf_nm:	move.b	(a0)+,d0
	bsr	putc
	dbra	d1,sf_nm
	bsr	newline
	lea	t_exp(pc),a0
	bsr	print
	move.l	a4,a1
	bsr	show16
	lea	t_got(pc),a0
	bsr	print
	move.l	a3,a1
	bsr	show16
	movem.l	(sp)+,d0-d2/a0-a1
	rts

show16:	moveq	#3,d2
s16:	move.l	(a1)+,d0
	bsr	phex
	moveq	#' ',d0
	bsr	putc
	dbra	d2,s16
	bra	newline

; ---- exception tests ----
; vector d0 = number, a0 = handler; old vector saved in old_vec
setvec:	move.l	a0,-(sp)
	move.w	d0,-(sp)
	move.w	#5,-(sp)		; Setexc
	trap	#13
	addq.l	#8,sp
	rts

test_dz:
	clr.l	exc_cnt
	moveq	#50,d0
	lea	h_exc(pc),a0
	bsr	setvec
	move.l	d0,old_vec
	fmove.l	#0,fpcr
	fmove.l	#0,fpsr
	fmove.l	#1,fp0
	fmove.l	#$00000400,fpcr		; DZ enabled
dz_div:	fdiv.l	#0,fp0			; DZ: the exception is pending
dz_next:
	fadd.x	fp0,fp1			; takes it first, then executes
	fmove.l	fpiar,d4		; FPIAR of the FADD itself
	fmove.l	#0,fpcr
	moveq	#50,d0
	move.l	old_vec,a0
	bsr	setvec
	; exactly one exception, vector 50, PC = the FADD, FPIAR = the FDIV,
	; frame BIU bit 27 clear (pending), then FPIAR = the FADD
	moveq	#0,d5
	cmp.l	#1,exc_cnt
	bne.s	dz_bad
	move.w	exc_vw,d0
	and.w	#$0FFF,d0
	cmp.w	#50*4,d0
	bne.s	dz_bad
	lea	dz_next(pc),a0
	cmp.l	exc_pc,a0
	bne.s	dz_bad
	lea	dz_div(pc),a0
	cmp.l	exc_fpiar,a0
	bne.s	dz_bad
	move.l	exc_biu,d0
	btst	#27,d0
	bne.s	dz_bad
	lea	dz_next(pc),a0
	cmp.l	d4,a0
	bne.s	dz_bad
	lea	t_pass(pc),a0
	bra	print
dz_bad:	addq.l	#1,exc_bad
	bra	exc_report

test_operr:
	clr.l	exc_cnt
	moveq	#52,d0
	lea	h_exc(pc),a0
	bsr	setvec
	move.l	d0,old_vec
	fmove.l	#0,fpcr
	fmove.x	e20(pc),fp2
	move.l	#$12345678,op_dest
	fmove.l	#$00002000,fpcr		; OPERR enabled
op_mov:	fmove.l	fp2,op_dest		; integer overflow: OPERR, stored, then the exception
	fmove.l	#0,fpcr
	moveq	#52,d0
	move.l	old_vec,a0
	bsr	setvec
	cmp.l	#1,exc_cnt
	bne.s	op_bad
	move.w	exc_vw,d0
	and.w	#$0FFF,d0
	cmp.w	#52*4,d0
	bne.s	op_bad
	move.w	exc_vw,d0
	lsr.w	#8,d0
	lsr.w	#4,d0
	cmp.w	#9,d0			; format 9: coprocessor mid-instruction
	bne.s	op_bad
	cmp.l	#$7FFFFFFF,op_dest
	bne.s	op_bad
	lea	op_mov(pc),a0
	cmp.l	exc_fpiar,a0
	bne.s	op_bad
	lea	t_pass(pc),a0
	bra	print
op_bad:	addq.l	#1,exc_bad
	bra	exc_report

exc_report:
	lea	t_excfail(pc),a0
	bsr	print
	move.l	exc_cnt,d0
	bsr	phex
	moveq	#' ',d0
	bsr	putc
	moveq	#0,d0
	move.w	exc_vw,d0
	bsr	phex
	moveq	#' ',d0
	bsr	putc
	move.l	exc_pc,d0
	bsr	phex
	moveq	#' ',d0
	bsr	putc
	move.l	exc_fpiar,d0
	bsr	phex
	moveq	#' ',d0
	bsr	putc
	move.l	exc_biu,d0
	bsr	phex
	moveq	#' ',d0
	bsr	putc
	move.l	op_dest,d0
	bsr	phex
	bra	newline

; FPU exception handler: count, record, absorb the pending exception the way
; a 68882 handler does (FSAVE, BIU flags bit 27 set, FRESTORE)
h_exc:	movem.l	d0-d1,-(sp)
	addq.l	#1,exc_cnt
	move.l	8+2(sp),exc_pc
	move.w	8+6(sp),exc_vw
	fsave	-(sp)
	moveq	#0,d0
	move.b	1(sp),d0		; frame size: the BIU flags are its last long
	tst.b	(sp)
	beq.s	hx_null
	move.l	(sp,d0.w),exc_biu
	bset	#3,(sp,d0.w)		; bit 27 of the long
hx_null:
	frestore (sp)+
	fmove.l	fpiar,exc_fpiar
	movem.l	(sp)+,d0-d1
	rte

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

ticks:	pea	rd_hz(pc)		; d0 = _hz_200
	move.w	#38,-(sp)
	trap	#14
	addq.l	#6,sp
	move.l	hz,d0
	rts
rd_hz:	move.l	$4BA.w,hz
	rts

print:	movem.l	d0-d2/a0-a2,-(sp)	; a0 = C string
	move.l	a0,-(sp)
	move.w	#9,-(sp)		; Cconws
	trap	#1
	addq.l	#6,sp
	movem.l	(sp)+,d0-d2/a0-a2
	rts

putc:	movem.l	d0-d2/a0-a2,-(sp)	; d0.b
	and.w	#$FF,d0
	move.w	d0,-(sp)
	move.w	#2,-(sp)		; Cconout
	trap	#1
	addq.l	#4,sp
	movem.l	(sp)+,d0-d2/a0-a2
	rts

newline:
	move.l	a0,-(sp)
	lea	t_nl(pc),a0
	bsr	print
	move.l	(sp)+,a0
	rts

phex:	movem.l	d0-d2,-(sp)		; d0 as 8 hex digits
	move.l	d0,d1
	moveq	#7,d2
ph_l:	rol.l	#4,d1
	move.b	d1,d0
	and.b	#15,d0
	add.b	#'0',d0
	cmp.b	#'9',d0
	ble.s	ph_o
	addq.b	#7,d0
ph_o:	bsr	putc
	dbra	d2,ph_l
	movem.l	(sp)+,d0-d2
	rts

pdec:	movem.l	d0-d2/a0,-(sp)		; d0 unsigned decimal
	lea	decbuf+12,a0
	clr.b	-(a0)
pd_l:	move.l	d0,d1
	divul.l	#10,d2:d1		; d1 = quotient, d2 = remainder
	add.b	#'0',d2
	move.b	d2,-(a0)
	move.l	d1,d0
	bne.s	pd_l
	bsr	print
	movem.l	(sp)+,d0-d2/a0
	rts

	even
e20:	dc.w	$4041,$0000,$AD78,$EBC5,$AC62,$0000	; 1e20

title:	dc.b	13,10,"FPUTEST - Falcon core 68882 on the ARM (falcon_fpu)",13,10,0
nofpu:	dc.b	"No FPU reported (_FPU cookie 0): is falcon_fpu running?",13,10,0
t_cookie: dc.b	"_FPU cookie: $",0
t_run:	dc.b	"Generated sequence: ",0
t_slots: dc.b	" results per pass, ",0
t_passes: dc.b	" passes",13,10,0
t_time:	dc.b	"Time per pass: ",0
t_ms:	dc.b	" ms",13,10,0
t_genok: dc.b	"Generated sequence: PASS",13,10,0
t_genbad: dc.b	"Generated sequence: FAIL, mismatching results: ",0
t_slot:	dc.b	"slot ",0
t_exp:	dc.b	"  expected ",0
t_got:	dc.b	"  got      ",0
t_dz:	dc.b	"DZ exception (pre-instruction, FSAVE/FRESTORE): ",0
t_operr: dc.b	"OPERR exception (FMOVE out, mid-instruction): ",0
t_pass:	dc.b	"PASS",13,10,0
t_excfail: dc.b	"FAIL",13,10,"  count/vector/pc/fpiar/biu/dest: ",0
t_sum_ok: dc.b	13,10,"FPUTEST: ALL PASS",13,10,0
t_sum_bad: dc.b	13,10,"FPUTEST: FAILURES (see above)",13,10,0
t_key:	dc.b	"Press a key.",13,10,0
t_nl:	dc.b	13,10,0
	even

	include	"gen.s"

	bss
	even
fpu_cookie:	ds.l	1
hz:		ds.l	1
t_start:	ds.l	1
t_total:	ds.l	1
nfail:		ds.l	1
exc_bad:	ds.l	1
exc_cnt:	ds.l	1
exc_pc:		ds.l	1
exc_vw:		ds.w	1
		ds.w	1
exc_fpiar:	ds.l	1
exc_biu:	ds.l	1
old_vec:	ds.l	1
op_dest:	ds.l	1
decbuf:		ds.b	12
		even
results:	ds.b	GEN_SLOTS*16
