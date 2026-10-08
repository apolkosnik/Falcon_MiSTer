; t_raw.s - raw CIR accesses in CPU space through MOVES (DFC = SFC = 7), the
; way a debugger would: every defined CIR of CpID 1 is read or written once
; with its natural width.  The bench checks the bus width of every cycle
; (word CIRs DSACK1 only, $10/$18/$1C both) from the pin log.
; Responses checked against the MC68881/2 UM response primitives:
;   null primitive = $08xx, bit 0 = TF (true/false), CA = bit 15 = 0.
	include	"common.i"

CIRB	equ	$00022000		; CPU space: A19-16 = 2, A15-13 = CpID 1

main:
	move.l	#2,SKIP
	moveq	#7,d0
	movec	d0,sfc
	movec	d0,dfc
	WAITGO
	lea	CIRB,a0
;---- reads of every CIR in the idle/null state
	moves.w	(a0),d1			; $00 response
	and.l	#$FFFF,d1
	REC	T_RAW_RESP0,d1
	moves.w	2(a0),d2		; $02 control
	moves.w	6(a0),d2		; $06 restore
	moves.w	8(a0),d2		; $08 operation word
	moves.w	$E(a0),d2		; $0E condition
	moves.l	$10(a0),d2		; $10 operand
	moves.w	$14(a0),d2		; $14 register select
	moves.l	$18(a0),d2		; $18 instruction address
	moves.l	$1C(a0),d2		; $1C operand address
	moves.w	4(a0),d1		; $04 save (first-type read): null state -> format $0000
	and.l	#$FFFF,d1
	REC	T_RAW_SAVE,d1
;---- writes
	moveq	#0,d3
	moves.w	d3,8(a0)		; operation word
	moves.l	d3,$10(a0)		; operand
	moves.w	d3,$14(a0)		; register select
	moves.l	d3,$18(a0)		; instruction address
	moves.l	d3,$1C(a0)		; operand address
;---- condition T: response null with TF = 1
	move.w	#$000F,d3
	moves.w	d3,$E(a0)
	moves.w	(a0),d1
	and.l	#$FFFF,d1
	move.l	d1,d4
	lsr.l	#8,d4
	REC	T_RAW_TF1,d4
	and.l	#1,d1
	REC	T_RAW_TF1B,d1
;---- condition F: TF = 0
	moveq	#0,d3
	moves.w	d3,$E(a0)
	moves.w	(a0),d1
	and.l	#$FFFF,d1
	move.l	d1,d4
	lsr.l	#8,d4
	REC	T_RAW_TF0H,d4
	and.l	#1,d1
	REC	T_RAW_TF0,d1
;---- command word $0000 = FMOVE FP0,FP0 (register to register): released at once
;     ($0900: null, IA), then the idle response when the ARM has finished
	moveq	#0,d3
	moves.w	d3,$A(a0)
	moves.w	(a0),d1
	and.l	#$FFFF,d1
	REC	T_RAW_CMD,d1
	move.w	#4000,d5
rw1:	moves.w	(a0),d1
	and.l	#$FFFF,d1
	cmp.w	#$0802,d1
	beq.s	rw2
	dbra	d5,rw1
rw2:	REC	T_RAW_CMD2,d1
;---- control CIR abort write and a null restore
	move.w	#$0001,d3
	moves.w	d3,2(a0)
	moveq	#0,d3
	moves.w	d3,6(a0)
	REC	T_RAW_DONE,#1
	bra	finish
