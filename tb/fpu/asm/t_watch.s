; t_watch.s - presence watcher.  Executes FRESTORE (null frame) in a loop (its
; first CIR access is the restore CIR write: BERR without a service, and it
; does not wait for a reply, so it also works with the C++ heartbeat writer); STATUS = 1 while it
; completes, 0 while it takes the Line-F exception (no FPU), ITER counts
; the loops.  The bench follows STATUS against the bridge's presence and
; ends the loop through STOPF.
	include	"common.i"

main:
	move.l	#2,SKIP			; FRESTORE (a1) is 2 bytes
	clr.l	BUF			; null frame
	lea	BUF,a1
	move.b	#2,STATUS
	WAITGO
loop:	tst.l	STOPF
	bne	finish
	clr.b	FLHIT
	dc.w	$f351			; frestore (a1)
	move.b	FLHIT,d0
	eori.b	#1,d0
	move.b	d0,STATUS
	addq.l	#1,ITER
	bra	loop
