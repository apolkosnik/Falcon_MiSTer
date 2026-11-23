; RVLAUNCH.PRG - AUTO launcher for \REVERT\REVERT.PRG
; \STRSTRCK as the current directory (as the desktop does)
	section	text
start:	move.l	4(sp),a5
	move.l	#$1000,-(sp)
	move.l	a5,-(sp)
	clr.w	-(sp)
	move.w	#$4a,-(sp)		; Mshrink
	trap	#1
	lea	12(sp),sp
	lea	stack,sp
	pea	path
	move.w	#$3b,-(sp)		; Dsetpath
	trap	#1
	addq.l	#6,sp
	clr.l	-(sp)			; environment
	pea	cmdl
	pea	prog
	clr.w	-(sp)			; load and go
	move.w	#$4b,-(sp)		; Pexec
	trap	#1
	lea	16(sp),sp
	clr.w	-(sp)
	trap	#1

	section	data
path:	dc.b	'\REVERT',0
prog:	dc.b	'REVERT.PRG',0
cmdl:	dc.b	0,0
	even

	section	bss
	ds.b	2048
stack:
