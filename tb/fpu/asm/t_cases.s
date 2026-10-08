; t_cases.s - the generated milestone 2 cases (obj/gen_cases.s, from tb/fpu/m2.cpp)
; Every case runs the real instruction sequence on the CPU; an unexpected
; exception (vector 11, 13, 14 ...) aborts that case only (habort in common.i).
	include	"common.i"
	include	"m2macros.i"

main:
	move.l	sp,SAVESP
	move.l	#1,ABORTMODE
	WAITGO
	include	"gen_cases.s"
	jmp	finish
