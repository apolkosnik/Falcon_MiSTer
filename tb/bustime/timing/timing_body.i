; timing_body.i - instruction timing sequences (docs/CPU_TIMING.md milestone 3)
;
; Shared by t_timing_rom.s (the core: tb/bustime) and t_timing_tos.s (Hatari,
; the golden).  Called in supervisor mode with interrupts masked.  Every test
; is a subroutine run twice: once to fill the instruction cache, then between
; two markers (move.w #n,$3F0.w); the cost of a test is the difference of the
; clock counts at its two markers.  Position independent; data in `tbuf`.
;
; With TSTAT defined (tools/cputime, milestone 4) every marker also calls
; tstat_mark, which reads the core's timing counters ($FFF000, falcon_tstat)
; into tstat_tab, and a last marker (73) closes the harness-only test's
; span.

MARK	macro
	move.w	#\1,$3F0.w
	ifd	TSTAT
	bsr	tstat_mark
	endif
	endm

TEST	macro				; TEST <subroutine>, <first marker>
	bsr	\1
	MARK	\2
	bsr	\1
	MARK	\2+1
	endm

timing:
	movem.l	d0-d7/a0-a6,-(sp)
	movec	cacr,d0
	move.l	d0,-(sp)
	move.l	#$0009,d0		; I-cache on (cleared), D-cache off
	movec	d0,cacr
	lea	tbuf(pc),a0
	lea	128(a0),a1
	lea	$E00000,a3
	lea	t_leaf(pc),a2
	moveq	#1,d1
	moveq	#3,d2
	move.l	$80.w,-(sp)		; trap #0
	lea	t_trap_h(pc),a4
	move.l	a4,$80.w

	TEST	t_nop,1
	TEST	t_moveq,3
	TEST	t_addl,5
	TEST	t_movel,7
	TEST	t_lea,9
	TEST	t_dbra,11
	TEST	t_bra,13
	TEST	t_bnt,15
	TEST	t_rdw,17
	TEST	t_rdl,19
	TEST	t_wrw,21
	TEST	t_wrl,23
	TEST	t_copy,25
	TEST	t_rdodd,27
	TEST	t_mulu,29
	TEST	t_muls,31
	TEST	t_divu,33
	TEST	t_divs,35
	TEST	t_mull,37
	TEST	t_divl,39
	TEST	t_lsl,41
	TEST	t_asr,43
	TEST	t_bsr,45
	TEST	t_jsr,47
	TEST	t_movem,49
	TEST	t_io,51
	TEST	t_rom,53
	TEST	t_addm,55
	TEST	t_clrm,57
	TEST	t_trap,59
	TEST	t_ext,61
	; instruction cache off: fetches from RAM
	moveq	#0,d0
	movec	d0,cacr
	TEST	t_nop,63
	TEST	t_rdw,65
	; data cache on
	move.l	#$0109,d0
	movec	d0,cacr
	TEST	t_rdl,67
	TEST	t_copy,69
	move.l	#$0009,d0		; the harness alone (instruction cache on)
	movec	d0,cacr
	TEST	t_empty,71
	ifd	TSTAT
	bsr	t_empty
	MARK	73
	endif

	move.l	(sp)+,$80.w
	move.l	(sp)+,d0
	movec	d0,cacr
	movem.l	(sp)+,d0-d7/a0-a6
	rts

; ---- the sequences
t_empty:
	rts
t_nop:	rept	50
	nop
	endr
	rts
t_moveq:
	rept	50
	moveq	#1,d0
	endr
	rts
t_addl:	rept	50
	add.l	d1,d0
	endr
	rts
t_movel:
	rept	50
	move.l	d1,d0
	endr
	rts
t_lea:	rept	50
	lea	8(a0),a5
	endr
	rts
t_dbra:	moveq	#49,d7
.l:	dbra	d7,.l
	rts
t_bra:	rept	25
	bra.s	*+4
	nop
	endr
	rts
t_bnt:	moveq	#1,d0
	rept	50
	beq.s	*+4
	endr
	rts
t_rdw:	rept	50
	move.w	(a0),d0
	endr
	rts
t_rdl:	rept	50
	move.l	(a0),d0
	endr
	rts
t_wrw:	rept	50
	move.w	d0,(a0)
	endr
	rts
t_wrl:	rept	50
	move.l	d0,(a0)
	endr
	rts
t_copy:	move.l	a0,a5
	move.l	a1,a6
	rept	25
	move.l	(a5)+,(a6)+
	endr
	rts
t_rdodd:
	rept	50
	move.w	1(a0),d0
	endr
	rts
t_mulu:	rept	20
	mulu.w	d2,d0
	endr
	rts
t_muls:	rept	20
	muls.w	d2,d0
	endr
	rts
t_divu:	rept	20
	move.l	#$12345,d0
	divu.w	d2,d0
	endr
	rts
t_divs:	rept	20
	move.l	#$12345,d0
	divs.w	d2,d0
	endr
	rts
t_mull:	rept	20
	mulu.l	d2,d0
	endr
	rts
t_divl:	rept	20
	move.l	#$12345678,d0
	divu.l	d2,d0
	endr
	rts
t_lsl:	rept	50
	lsl.l	#8,d0
	endr
	rts
t_asr:	rept	50
	asr.w	d1,d0
	endr
	rts
t_bsr:	rept	25
	bsr	t_leaf
	endr
	rts
t_jsr:	rept	25
	jsr	(a2)
	endr
	rts
t_leaf:	rts
t_movem:
	rept	5
	movem.l	d0-d7,-(sp)
	movem.l	(sp)+,d0-d7
	endr
	rts
t_io:	rept	20
	move.b	$FFFFFA01.w,d0
	endr
	rts
t_rom:	rept	50
	move.l	(a3),d0
	endr
	rts
t_addm:	rept	25
	add.l	d1,(a0)
	endr
	rts
t_clrm:	rept	50
	clr.l	(a0)
	endr
	rts
t_trap:	rept	10
	trap	#0
	endr
	rts
t_trap_h:
	rte
t_ext:	rept	50
	ext.l	d0
	swap	d0
	endr
	rts

	even
; tbuf at 2 mod 4 from the start (the body runs from a 256-byte boundary):
; 1(a0) then crosses a longword.  Hatari charges a misaligned word inside a
; longword as one access, the 68030 on the Falcon's 16-bit port always
; makes two byte cycles (docs/CPU_TIMING.md: the odd word); across a
; longword both take two.
	dcb.b	(2-(*-timing))&3,0
tbuf:	ds.b	256

	ifd	TSTAT
; the counters at marker n go to tstat_tab + 16 * n: FTIME (first, so the
; rest of the hook is the same every time), DEBT.w and RTIME (long reads:
; falcon_tstat latches the low word); the clocks forgiven at the debt cap
; follow from the three.  Without the
; counters (Hatari) tstat_io points to the ST palette instead: the same
; instructions (the same instruction cache footprint) and I/O reads of the
; same length (FAST16, no wait states), whose values are not used.
tstat_mark:
	movem.l	d0/a0-a1,-(sp)
	move.w	$3F0.w,d0
	lsl.w	#4,d0
	lea	tstat_tab(pc),a0
	add.w	d0,a0
	move.l	tstat_io(pc),a1
	move.l	4(a1),(a0)+		; FTIME
	move.w	12(a1),(a0)+		; DEBT
	move.l	8(a1),(a0)+		; RTIME
	movem.l	(sp)+,d0/a0-a1
	rts
tstat_io:
	dc.l	$FFFF8240		; $FFFFF000 with the counters
	even
tstat_tab:
	ds.b	16*74
	endif
