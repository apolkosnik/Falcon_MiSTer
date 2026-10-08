; t_gen.s - cpGEN instructions are not implemented in milestone 1: the bridge
; answers the pre-instruction exception primitive with vector 11 (F-line).
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
g0s:	fmove.l	d0,fp0
g0e:	GEN_A	0,g0s

	GEN_B	g1s,g1e
g1s:	fadd.x	fp1,fp0
g1e:	GEN_A	1,g1s

	GEN_B	g2s,g2e
g2s:	fmovem.x fp0-fp3,(a0)
g2e:	GEN_A	2,g2s

	GEN_B	g3s,g3e
g3s:	fmove.l	#$12345678,fp0
g3e:	GEN_A	3,g3s

	GEN_B	g4s,g4e
g4s:	fmovem.l d0,fpcr
g4e:	GEN_A	4,g4s

	GEN_B	g5s,g5e
g5s:	fmove.x	fp0,-(a7)
g5e:	GEN_A	5,g5s

	GEN_B	g6s,g6e
g6s:	fmovecr	#0,fp0
g6e:	GEN_A	6,g6s

	GEN_B	g7s,g7e
g7s:	fmove.x	(BUF).l,fp1
g7e:	GEN_A	7,g7s

	GEN_B	g8s,g8e
g8s:	fsqrt.x	fp0,fp1
g8e:	GEN_A	8,g8s

	GEN_B	g9s,g9e
g9s:	fcmp.x	fp1,fp0
g9e:	GEN_A	9,g9s

	GEN_B	g10s,g10e
g10s:	ftst.x	fp0
g10e:	GEN_A	10,g10s

	GEN_B	g11s,g11e
g11s:	fmove.s	fp0,d1
g11e:	GEN_A	11,g11s

	GEN_B	g12s,g12e
g12s:	fmovem.l fpcr/fpsr/fpiar,(a0)
g12e:	GEN_A	12,g12s

	GEN_B	g13s,g13e
g13s:	fmove.d	(8,a0),fp7
g13e:	GEN_A	13,g13s

	move.l	EXCNT,d1
	REC	T_GEN_COUNT,d1		; 14 exceptions in all
	bra	finish
