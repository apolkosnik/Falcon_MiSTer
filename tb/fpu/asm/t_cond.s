; t_cond.s - coprocessor conditionals with the FPSR condition codes all zero
; (N = Z = I = NAN = 0 after reset): FScc, FBcc, FDBcc, FTRAPcc for every
; predicate 0..31 (MC68881/MC68882 UM conditional tests), FNOP (= FBF with
; displacement 0).  Needs the FPU present.
;
; Encodings (MC68030 UM 10.4 coprocessor instruction formats, as vasm emits
; them for fscc/fbcc/fdbcc/ftrapcc):
;   cpScc   $F240+ea,  condition word        FScc D1      = $F241,$00pp
;   cpBcc.W $F280+pp,  16-bit displacement   (relative to the displacement word)
;   cpDBcc  $F248+Dn,  condition word, displacement
;   cpTRAPcc $F27C (no operand), $F27A (.W), $F27B (.L), condition word first
	include	"common.i"

; FScc D1 -> byte in d1 (0 or $FF)
C_SCC	macro			; \1 = predicate
	moveq	#0,d1
	dc.w	$F241,\1
	and.l	#$FF,d1
	REC	T_FSCC+\1,d1
	endm

; FBcc.W over a MOVEQ: d2 = 1 when the branch was taken
C_BCC	macro
	moveq	#1,d2
	dc.w	$F280+\1,$0004
	moveq	#0,d2
	REC	T_FBCC+\1,d2
	endm

; FDBcc D3: count 5; when the condition is false D3 is decremented and the
; loop branches
C_DBCC	macro
	moveq	#5,d3
	moveq	#0,d2
	dc.w	$F24B,\1
	dc.w	fdt\@-*
	bra.s	fdn\@
fdt\@:	moveq	#1,d2
fdn\@:	REC	T_FDB_TAKEN+\1,d2
	REC	T_FDB_COUNT+\1,d3
	endm

; FTRAPcc (no operand): vector 7 when true
C_TRAP	macro
	move.l	EXCNT,d4
	dc.w	$F27C,\1
	move.l	EXCNT,d5
	sub.l	d4,d5
	REC	T_FTRAP+\1,d5
	endm

main:
	WAITGO
	move.l	#0,SKIP			; TRAPcc exceptions are post-instruction

;---- FNOP completes, three times, with no exception
	move.l	EXCNT,d7
	FNOP_
	FNOP_
	FNOP_
	moveq	#1,d6
	REC	T_FNOP_DONE,d6
	move.l	EXCNT,d1
	sub.l	d7,d1
	REC	T_FNOP_CNT,d1

;---- all 32 predicates, four instruction forms
p	set	0
	rept	32
	C_SCC	p
p	set	p+1
	endr
p	set	0
	rept	32
	C_BCC	p
p	set	p+1
	endr
p	set	0
	rept	32
	C_DBCC	p
p	set	p+1
	endr
p	set	0
	rept	32
	C_TRAP	p
p	set	p+1
	endr

;---- FScc to memory: byte store ($FF for T = $0F, $00 for F = $00), memory pre-set to $55
	lea	BUF,a0
	move.b	#$55,(a0)
	dc.w	$F250,$000F		; FST (a0)
	moveq	#0,d1
	move.b	(a0),d1
	REC	T_FSCC_MEM_T,d1
	move.b	#$55,(a0)
	dc.w	$F250,$0000		; FSF (a0)
	moveq	#0,d1
	move.b	(a0),d1
	REC	T_FSCC_MEM_F,d1

;---- FBcc.L (32-bit displacement): FBT taken, FBF not taken
	moveq	#1,d2
	dc.w	$F2CF			; FBT.L
	dc.l	$00000006		; relative to the displacement: skip the MOVEQ (2) after it (4+2)
	moveq	#0,d2
	REC	T_FBCC_L_T,d2
	moveq	#1,d2
	dc.w	$F2C0			; FBF.L
	dc.l	$00000006
	moveq	#0,d2			; executed: not taken
	REC	T_FBCC_L_F,d2

;---- FTRAPcc with operand: the operand is skipped whether or not it traps
	move.l	EXCNT,d7
	dc.w	$F27A,$000F,$1234	; FTRAPT.W #$1234 -> vector 7
ftwt:	move.l	EXCNT,d1
	sub.l	d7,d1
	REC	T_FTW_T,d1
	move.l	EXLOGP,a1
	REC	T_FTW_VEC,-32(a1)
	REC	T_FTW_FMT,-28(a1)
	move.l	-24(a1),d1
	cmp.l	#ftwt,d1
	seq	d1
	and.l	#1,d1
	REC	T_FTW_PC,d1
	move.l	-20(a1),d1
	cmp.l	#ftwt-6,d1
	seq	d1
	and.l	#1,d1
	REC	T_FTW_IA,d1
	move.l	EXCNT,d7
	dc.w	$F27A,$0000,$1234	; FTRAPF.W #$1234 -> nothing, operand skipped
	move.l	EXCNT,d1
	sub.l	d7,d1
	REC	T_FTW_F,d1
	move.l	EXCNT,d7
	dc.w	$F27B,$000F		; FTRAPT.L #$12345678
	dc.l	$12345678
ftlt:	move.l	EXCNT,d1
	sub.l	d7,d1
	REC	T_FTL_T,d1
	move.l	EXLOGP,a1
	move.l	-24(a1),d1
	cmp.l	#ftlt,d1
	seq	d1
	and.l	#1,d1
	REC	T_FTL_PC,d1
	move.l	EXCNT,d7
	dc.w	$F27B,$0000		; FTRAPF.L #$12345678
	dc.l	$12345678
	move.l	EXCNT,d1
	sub.l	d7,d1
	REC	T_FTL_F,d1
	bra	finish
