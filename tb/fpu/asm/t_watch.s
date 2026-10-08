; t_watch.s - presence watcher.  Executes FNOP in a loop; STATUS = 1 while it
; completes, 0 while it takes the Line-F exception (no FPU), ITER counts
; the loops.  The bench follows STATUS against the bridge's presence and
; ends the loop through STOPF.
	include	"common.i"

main:
	move.l	#4,SKIP			; FNOP is 4 bytes
	move.b	#2,STATUS
	WAITGO
loop:	tst.l	STOPF
	bne	finish
	clr.b	FLHIT
	FNOP_
	move.b	FLHIT,d0
	eori.b	#1,d0
	move.b	d0,STATUS
	addq.l	#1,ITER
	bra	loop
