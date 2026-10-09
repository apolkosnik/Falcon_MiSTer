; cputime.s - CPUTIME.TOS: the Falcon core's CPU timing on the board
; (docs/CPU_TIMING.md, milestone 4).
;
; Runs the 35 instruction sequences of tb/bustime/timing/timing_body.i (and
; the harness alone) and, with the OSD option "Timing counters" on, reads
; the core's counters (falcon_tstat, $FFF000) at every marker.  Per test it
; prints the span from the test's first marker to the next test's first
; marker (the test and the next test's cache-filling run):
;   core    Falcon time, in processor clocks (Hatari's cycle count)
;   sim     the same span in the full-system simulation (ref.i, ref.sh:
;           Falcon time at every marker): Falcon time does not depend on the
;           memory's latency, so the two must be equal ('*' marks a span
;           that is not; the marker intervals that differ are listed)
;   real    real time, in processor clocks of the selected speed
;   lost    the part of it forgiven at the debt cap (the CPU behind for good)
;   real/core in percent
; and the same for whole sections.  How close Falcon time is to Hatari's
; is measured by tb/bustime/timing; the counter reads at the markers would
; distort that comparison here (docs/CPU_TIMING.md, milestone 4).
;
; Without the counters (option off, Hatari, a Falcon) it only runs the
; sequences, reading the ST palette where it would read the counters.
;
;   build.sh (vasmm68k_mot -Ftos -m68030 -no-opt)

TSTAT	equ	1			; timing_body.i: the counter reads
NT	equ	36			; spans: 35 tests and the harness alone

	text
start:	lea	title(pc),a0
	bsr	print
	pea	sup(pc)
	move.w	#38,-(sp)		; Supexec
	trap	#14
	addq.l	#6,sp
	tst.w	present
	bne.s	.on
	lea	t_off(pc),a0
	bsr	print
	bra.s	done
.on:	bsr	report
done:	lea	t_key(pc),a0
	bsr	print
	bsr	getkey
	clr.w	-(sp)			; Pterm0
	trap	#1

; ---- supervisor: find the counters, run the sequences ----
sup:
	ifd	STACKOFF		; (a test: the caller's stack elsewhere)
	sub.l	#STACKOFF,sp
	endif
	move.w	sr,-(sp)
	move.w	#$2700,sr
	; a bus error at $FFF000: no counters
	move.l	$8.w,-(sp)
	move.l	sp,berr_sp
	lea	berr_h(pc),a0
	move.l	a0,$8.w
	clr.w	present
	move.w	$FFFFF000.w,d0
	cmp.w	#$5453,d0
	bne.s	sup_none
	move.w	$FFFFF002.w,flags
	move.w	#1,present
sup_none:
	move.l	berr_sp,sp
	move.l	(sp)+,$8.w
	tst.w	present			; (in the body, before it is copied)
	beq.s	.io
	move.l	#$FFFFF000,tstat_io
.io:
	lea	ostack+512,a0		; (the stack for the sequences)
	move.l	a0,d0
	and.l	#$FFFFFF00,d0
	move.l	d0,own_sp
	; copy the body to a 256-byte boundary (as t_timing_tos.s: the
	; instruction cache maps it as in the golden)
	lea	space,a1
	move.l	a1,d0
	add.l	#255,d0
	and.l	#$FFFFFF00,d0
	move.l	d0,a1
	move.l	d0,copy
	lea	body(pc),a0
	move.w	#(body_end-body)/2-1,d0
.cp:	move.w	(a0)+,(a1)+
	dbra	d0,.cp
	; both caches off and cleared (the body sets its own: no data cache
	; lines from before), the caller's setting back afterwards
	movec	cacr,d0
	move.l	d0,-(sp)
	move.l	#$0809,d0		; (the instruction cache on for the loop below)
	movec	d0,cacr
	; start at the same ST-RAM slot phase every time: the slot waits
	; depend on Falcon time mod 4 (Hatari's rule), and what ran before
	; (TOS, the program's start) leaves it anywhere.  The loop runs from the
	; instruction cache with an I/O read only (no slot waits), so its length
	; is the same every time; it is odd, so FTIME passes 0 mod 4 within
	; four rounds.  synced: the rounds left (-1: it did not)
	moveq	#-1,d2
	tst.w	present
	beq.s	.go
	ifd	TIMERS			; (a test: MFP timers running, as under TOS)
	move.b	#TIMERS,$FFFFFA23.w	; timer C data
	move.b	#$10,$FFFFFA1D.w	; timer C: /4
	or.b	#$20,$FFFFFA09.w	; IERB: timer C
	or.b	#$20,$FFFFFA15.w	; IMRB
	endif
	ifd	PREDELAY		; (tests of the loop: other entry phases)
	rept	PREDELAY
	tst.b	$FFFFF000.w
	endr
	endif
	ifd	PRENOP
	rept	PRENOP
	nop
	endr
	endif
	moveq	#3,d1
	moveq	#15,d2
.sync:	move.w	$FFFFF006.w,d0		; FTIME, low word
	and.w	d1,d0
	dbeq	d2,.sync
.go:	move.w	d2,synced
	move.l	#$0808,d0
	movec	d0,cacr
	; the sequences on the program's own stack, at the same place in a
	; 256-byte block every time: the stack's lines in the data cache (the
	; marker hook's MOVEMs) then meet the test data the same way as in the
	; simulation, wherever TOS keeps its own stack
	move.l	sp,save_sp
	move.l	own_sp,sp
	move.l	copy,a1
	jsr	(a1)
	move.l	save_sp,sp
	move.w	#$FFFF,$3F0.w		; the end (a Hatari harness quits there)
	ifd	TIMERS
	clr.b	$FFFFFA1D.w
	and.b	#$DF,$FFFFFA09.w
	and.b	#$DF,$FFFFFA15.w
	move.b	#$DF,$FFFFFA0D.w	; (IPRB: timer C no longer pending)
	endif
	move.l	(sp)+,d0
	movec	d0,cacr
	move.w	(sp)+,sr
	ifd	STACKOFF
	add.l	#STACKOFF,sp
	endif
	rts
berr_h:	move.l	berr_sp,sp		; (drop the bus error frame)
	bra	sup_none

; ---- the report ----
; a2 = the counters at marker 0 (tstat_tab in the copy), 16 bytes per marker:
; FTIME, DEBT.w, RTIME
report:	movem.l	d0-d7/a0-a6,-(sp)
	move.l	copy,a2
	add.l	#tstat_tab-body,a2
	; processor clocks per system clock: 1/2 (16 MHz), 1/4 (8 MHz), 1 (turbo)
	moveq	#1,d0
	move.w	flags,d1
	btst	#0,d1
	beq.s	.per
	moveq	#2,d0
	btst	#1,d1
	bne.s	.per
	moveq	#4,d0
.per:	move.w	d0,period
	lea	t_turbo(pc),a0
	cmp.w	#1,d0
	beq.s	.mode
	lea	t_16(pc),a0
	cmp.w	#2,d0
	beq.s	.mode
	lea	t_8(pc),a0
.mode:	bsr	print
	lea	t_head(pc),a0
	bsr	print
	clr.w	nsim
	moveq	#1,d7
.t:	cmp.w	#21,d7			; a page break before test 21
	bne.s	.t1
	lea	t_more(pc),a0
	bsr	print
	bsr	getkey
	lea	t_head(pc),a0
	bsr	print
.t1:	move.l	d7,d0
	add.l	d0,d0
	subq.l	#1,d0			; markers 2k-1 to 2k+1
	move.l	d0,d1
	addq.l	#2,d1
	bsr	delta			; d0 = core, d1 = real, d2 = lost
	move.l	d7,d4
	subq.l	#1,d4			; (the name)
	lea	ref_mark(pc),a0
	move.l	d7,d3
	lsl.l	#3,d3
	add.l	d3,a0			; the simulation at marker 2k+1 ...
	move.l	(a0),d3
	sub.l	-8(a0),d3		; ... less at marker 2k-1
	move.l	d0,d5
	moveq	#' ',d0
	btst	#1,ref_valid+1(pc)
	beq.s	.t3
	cmp.l	d3,d5
	beq.s	.t2
	moveq	#'*',d0			; differs from the simulation
	bra.s	.t3
.t2:	addq.w	#1,nsim
.t3:	bsr	putc
	move.l	d4,d0
	mulu.w	#24,d0
	lea	names(pc),a0
	add.l	d0,a0
	bsr	print
	move.l	d5,d0
	bsr	row
	addq.w	#1,d7
	cmp.w	#NT,d7
	ble.s	.t
	; whole sections: markers 1-63, 63-71, 1-71 (spans 1-31, 32-35, 1-35)
	lea	t_sec(pc),a0
	bsr	print
	lea	n_sec1(pc),a0
	moveq	#1,d0
	moveq	#63,d1
	bsr	sect
	lea	n_sec2(pc),a0
	moveq	#63,d0
	moveq	#71,d1
	bsr	sect
	lea	n_sec3(pc),a0
	moveq	#1,d0
	moveq	#71,d1
	bsr	sect
	; against the simulation
	lea	t_nosim(pc),a0
	btst	#1,ref_valid+1(pc)
	beq	.s1
	lea	t_sim1(pc),a0
	bsr	print
	moveq	#0,d0
	move.w	nsim,d0
	moveq	#0,d1
	bsr	pnum
	lea	t_sim2(pc),a0
	bsr	print
	; where it differs: every marker interval (a test's own run, the next
	; test's cache-filling run), at most 12
	moveq	#0,d6
	moveq	#1,d7
.i:	move.l	d7,d0
	move.l	d7,d1
	addq.l	#1,d1
	bsr	delta
	lea	ref_mark(pc),a0
	move.l	(a0,d7.l*4),d3
	sub.l	-4(a0,d7.l*4),d3
	cmp.l	d3,d0
	beq.s	.i1
	move.l	d0,d5
	lea	t_iv1(pc),a0
	bsr	print
	move.l	d7,d0
	moveq	#0,d1
	bsr	pnum
	moveq	#'-',d0
	bsr	putc
	move.l	d7,d0
	addq.l	#1,d0
	bsr	pnum
	lea	t_iv2(pc),a0
	bsr	print
	move.l	d5,d0
	bsr	pnum
	lea	t_iv3(pc),a0
	bsr	print
	move.l	d3,d0
	bsr	pnum
	lea	t_nl(pc),a0
	bsr	print
	addq.l	#1,d6
	cmp.l	#12,d6
	bge.s	.s2
.i1:	addq.l	#1,d7
	cmp.l	#72,d7
	ble.s	.i
	bra.s	.s2
.s1:	bsr	print
	; no reference: Falcon time at every marker, for ref.sh
	moveq	#1,d7
.d:	lea	t_m(pc),a0
	bsr	print
	move.l	d7,d0
	moveq	#0,d1
	bsr	pnum
	moveq	#' ',d0
	bsr	putc
	moveq	#1,d0
	move.l	d7,d1
	bsr	delta
	moveq	#0,d1
	bsr	pnum
	lea	t_nl(pc),a0
	bsr	print
	addq.l	#1,d7
	cmp.l	#73,d7
	ble.s	.d
.s2:
	; the phase at the first marker (Falcon time mod 4: the ST-RAM slot)
	lea	t_phase(pc),a0
	bsr	print
	move.l	16(a2),d0
	and.l	#3,d0
	moveq	#0,d1
	bsr	pnum
	lea	t_sync(pc),a0
	bsr	print
	move.w	synced,d0
	ext.l	d0
	moveq	#0,d1
	bsr	pnum
	lea	t_nl(pc),a0
	bsr	print
	movem.l	(sp)+,d0-d7/a0-a6
	rts

; markers d0 to d1: d0 = Falcon clocks, d1 = real, d2 = lost (processor
; clocks); lost (forgiven) in system clocks = RTIME - period * FTIME - DEBT
delta:	movem.l	d3-d4/a0-a1,-(sp)
	lsl.l	#4,d0
	lsl.l	#4,d1
	lea	(a2,d0.l),a0
	lea	(a2,d1.l),a1
	move.l	(a1),d0
	sub.l	(a0),d0			; FTIME
	move.l	6(a1),d1
	sub.l	6(a0),d1		; RTIME (system clocks)
	moveq	#0,d3
	move.w	period,d3
	move.l	d0,d2
	mulu.l	d3,d2
	neg.l	d2
	add.l	d1,d2			; RTIME - period * FTIME
	moveq	#0,d4
	move.w	4(a1),d4
	sub.l	d4,d2
	moveq	#0,d4
	move.w	4(a0),d4
	add.l	d4,d2			; - DEBT: lost (system clocks)
	divu.l	d3,d1
	divu.l	d3,d2
	movem.l	(sp)+,d3-d4/a0-a1
	rts

; a0 = name, d0/d1 = first/last marker
sect:	movem.l	d0-d7/a0,-(sp)
	bsr	print
	lea	ref_mark(pc),a0
	move.l	-4(a0,d1.l*4),d3
	sub.l	-4(a0,d0.l*4),d3	; the simulation
	bsr	delta			; d0 core, d1 real, d2 lost
	bsr	row
	movem.l	(sp)+,d0-d7/a0
	rts

; one row: d0 = core, d3 = simulation, d1 = real, d2 = lost
row:	movem.l	d0-d7,-(sp)
	move.l	d0,d5
	move.l	d1,d6
	move.l	d2,d7
	moveq	#7,d1
	bsr	pnum			; core
	btst	#1,ref_valid+1(pc)
	beq.s	.nos
	move.l	d3,d0
	moveq	#8,d1
	bsr	pnum			; simulation
	bra.s	.r
.nos:	lea	t_nos(pc),a0
	bsr	print
.r:	move.l	d6,d0
	moveq	#9,d1
	bsr	pnum			; real
	move.l	d7,d0
	moveq	#8,d1
	bsr	pnum			; lost
	; real time as a share of Falcon time, in tenths of a percent
	move.l	d6,d0
	mulu.l	#1000,d0
	tst.l	d5
	beq.s	.pc
	divu.l	d5,d0
.pc:	divul.l	#10,d1:d0
	move.l	d1,-(sp)
	moveq	#6,d1
	bsr	pnum
	moveq	#'.',d0
	bsr	putc
	move.l	(sp)+,d0
	moveq	#0,d1
	bsr	pnum
	moveq	#'%',d0
	bsr	putc
	lea	t_nl(pc),a0
	bsr	print
	movem.l	(sp)+,d0-d7
	rts

; ---- console ----
getkey:	movem.l	d0-d2/a0-a2,-(sp)
	move.w	#1,-(sp)		; Cconin
	trap	#1
	addq.l	#2,sp
	movem.l	(sp)+,d0-d2/a0-a2
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

; d0 = signed number, d1 = field width (right aligned; 0: as wide as it is)
pnum:	movem.l	d0-d3/a0,-(sp)
	lea	numbuf+16,a0
	clr.b	-(a0)
	move.l	d0,d3			; (sign)
	bpl.s	.l
	neg.l	d0
.l:	move.l	d0,d2
	divul.l	#10,d0:d2		; d2 = quotient, d0 = remainder
	exg	d0,d2
	add.b	#'0',d2
	move.b	d2,-(a0)
	subq.l	#1,d1
	tst.l	d0
	bne.s	.l
	tst.l	d3
	bpl.s	.pad
	move.b	#'-',-(a0)
	subq.l	#1,d1
.pad:	tst.l	d1
	ble.s	.p
	move.b	#' ',-(a0)
	subq.l	#1,d1
	bra.s	.pad
.p:	bsr	print
	movem.l	(sp)+,d0-d3/a0
	rts

title:	dc.b	27,"E","CPUTIME 3 - Falcon core CPU timing (docs/CPU_TIMING.md)",13,10,0
t_off:	dc.b	"No timing counters: turn on the OSD option 'Timing counters' (Falcon core",13,10
	dc.b	"only) and run again.",13,10,0
t_16:	dc.b	"CPU: Falcon 16 MHz.  Spans in processor clocks.",13,10,0
t_8:	dc.b	"CPU: Falcon 8 MHz.  Spans in processor clocks.",13,10,0
t_turbo: dc.b	"CPU: 32 MHz turbo (not Falcon timing: the simulation does not apply).",13,10,0
t_head:	dc.b	"test                       core     sim     real    lost   real",13,10,0
t_sec:	dc.b	"sections",13,10,0
t_nos:	dc.b	"       -",0
t_nosim: dc.b	"(no simulation reference in this build; Falcon time at the markers:)",13,10,0
t_m:	dc.b	"M ",0
t_iv1:	dc.b	"  markers ",0
t_iv2:	dc.b	": core ",0
t_iv3:	dc.b	", sim ",0
t_sim1:	dc.b	"Falcon time equal to the simulation in ",0
t_sim2:	dc.b	" of 36 spans",13,10,0
t_more:	dc.b	"Press a key for more.",13,10,0
t_phase: dc.b	"slot phase at the first marker ",0
t_sync:	dc.b	", sync rounds left ",0
t_key:	dc.b	"Press a key.",13,10,0
t_nl:	dc.b	13,10,0
n_sec1:	dc.b	" 1-31 I-cache on       ",0
n_sec2:	dc.b	"32-35 I-cache off/D on ",0
n_sec3:	dc.b	" 1-35                  ",0
names:	; 24 bytes each
	dc.b	" 1 nop                 ",0
	dc.b	" 2 moveq               ",0
	dc.b	" 3 add.l Dn            ",0
	dc.b	" 4 move.l Dn           ",0
	dc.b	" 5 lea d(An)           ",0
	dc.b	" 6 dbra loop           ",0
	dc.b	" 7 bra.s               ",0
	dc.b	" 8 bcc not taken       ",0
	dc.b	" 9 move.w (An)         ",0
	dc.b	"10 move.l (An)         ",0
	dc.b	"11 move.w ->(An)       ",0
	dc.b	"12 move.l ->(An)       ",0
	dc.b	"13 move.l (An)+,(An)+  ",0
	dc.b	"14 move.w 1(An)        ",0
	dc.b	"15 mulu.w              ",0
	dc.b	"16 muls.w              ",0
	dc.b	"17 divu.w              ",0
	dc.b	"18 divs.w              ",0
	dc.b	"19 mulu.l              ",0
	dc.b	"20 divu.l              ",0
	dc.b	"21 lsl.l #8            ",0
	dc.b	"22 asr.w Dn            ",0
	dc.b	"23 bsr/rts             ",0
	dc.b	"24 jsr/rts             ",0
	dc.b	"25 movem.l             ",0
	dc.b	"26 MFP read            ",0
	dc.b	"27 ROM move.l          ",0
	dc.b	"28 add.l Dn,(An)       ",0
	dc.b	"29 clr.l (An)          ",0
	dc.b	"30 trap/rte            ",0
	dc.b	"31 ext/swap            ",0
	dc.b	"32 nop, no I-cache     ",0
	dc.b	"33 move.w (An), no I-c.",0
	dc.b	"34 move.l (An), D-cache",0
	dc.b	"35 copy, D-cache       ",0
	dc.b	"36 harness only        ",0
	even
	ifd	REFZERO			; (ref.sh: the same layout, no numbers)
ref_valid:	dc.w	0
ref_mark:	dcb.l	73,0
	else
	include	"ref.i"
	endif

	even
body:
	include	"timing_body.i"
	even
body_end:

	bss
present: ds.w	1
flags:	ds.w	1
period:	ds.w	1
nsim:	ds.w	1
synced:	ds.w	1
berr_sp: ds.l	1
save_sp: ds.l	1
own_sp:	ds.l	1
copy:	ds.l	1
numbuf:	ds.b	16
space:	ds.b	body_end-body+256
ostack:	ds.b	768
