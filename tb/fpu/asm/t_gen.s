; t_gen.s - command words the bridge refuses itself (milestone 2): opclass 001
; (reserved) and FMOVE to/from the control registers with an empty register
; list answer the pre-instruction exception primitive, vector 11 (F-line).
; The 68030 takes vector 11 with a format 0 frame whose stacked PC is the FPU
; instruction (MC68030 UM 10.4: pre-instruction exception); the handler steps
; over the instruction (SKIP = its length) and execution continues.
	include	"common.i"

GEN_B	macro			; \1 = start label, \2 = end label
	move.l	#\2-\1,SKIP
	move.l	EXCNT,d7
	moveq	#0,d6
	endm

GEN_A	macro			; \1 = case no, \2 = start label
	moveq	#1,d6
	move.l	EXLOGP,a1
	REC	T_GEN+\1*4+0,-32(a1)
	REC	T_GEN+\1*4+1,-28(a1)
	LASTPC_IS	\2
	REC	T_GEN+\1*4+2,d1
	REC	T_GEN+\1*4+3,d6
	endm

main:
	WAITGO
	lea	BUF,a0

	GEN_B	g0s,g0e
g0s:	dc.w	$F200,$2000
g0e:	GEN_A	0,g0s

	GEN_B	g1s,g1e
g1s:	dc.w	$F200,$2400
g1e:	GEN_A	1,g1s

	GEN_B	g2s,g2e
g2s:	dc.w	$F210,$8000
g2e:	GEN_A	2,g2s

	GEN_B	g3s,g3e
g3s:	dc.w	$F210,$A000
g3e:	GEN_A	3,g3s

	move.l	EXCNT,d1
	REC	T_GEN_COUNT,d1
	bra	finish
