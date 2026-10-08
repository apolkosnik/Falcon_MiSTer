; common.i - harness shared by the FPU bridge test programs (vasm, Motorola syntax)
;
; Memory (24-bit guest space of falcon_cpubus):  RAM from 0, ROM at $E00000.
; The program image is the ROM: vector table at $E00000 (the first two
; vectors are what falcon_cpubus serves for reads at 0..7), code from $E00400.
; The vector table is copied to RAM at 0 so tests may patch vectors the way
; EmuTOS does.  Results go to a (tag, value) log in RAM that the bench reads
; and compares with the expected values it takes from the manuals.

	include	"tags.i"

SSP		equ	$00010000
GO		equ	$0F00		; long: bench sets non-zero when the test may start
STOPF		equ	$0F04		; long: bench sets non-zero to end a watch loop
DONE		equ	$0F08		; long: $D0E0600D when the program has finished
EXCNT		equ	$0F0C		; long: exceptions logged
EXLOGP		equ	$0F10		; long: next exception log entry
SKIP		equ	$0F14		; long: added to the stacked PC of format 0 frames
STATUS		equ	$0F18		; byte: watch loop result
FLHIT		equ	$0F1C		; byte: a Line-F (vector 11) exception happened
ITER		equ	$0F20		; long: watch loop iterations
UNEXP		equ	$0F24		; long: non-zero when an unexpected vector was taken
READY		equ	$0F28		; long: set when the program waits for GO
TRAPCNT		equ	$0F2C		; long: TRAPcc (vector 7) exceptions counted in case mode
CASEIX		equ	$0F30		; long: generated case being run
RESUME		equ	$0F34		; long: where an aborted case continues
SAVESP		equ	$0F38		; long: stack pointer at the start of the case program
ABORTMODE	equ	$0F3C		; long: non-zero: unexpected exceptions abort the current case
IRQCNT		equ	$0F5C		; long: interrupts taken by h_irq
HOLDW		equ	$0F60		; long: 1 while the program waits at a hold handshake (the bench clears it)
HMODE		equ	$0F58		; long: 0 the FPU exception handler absorbs the exception, 1 it only returns
SOFTEXC		equ	$0F54		; long: non-zero: an expected pre-instruction exception is stepped over (SKIP)
MARK1		equ	$0F40		; longs: progress markers of the hand written M2 programs
MARK2		equ	$0F44
MARK3		equ	$0F48
MARK4		equ	$0F4C
EXLOG		equ	$80000		; 32-byte entries: vector, format, PC, IA, SR, case index (8192 entries)
RECLOG		equ	$4000		; (tag, value) longs, ends with tag 0
BUF		equ	$8000
BUF2		equ	$8100
BUF3		equ	$8200
BUF4		equ	$8300

; record a result: REC tag,source
REC	macro
	move.l	#\1,(a5)+
	move.l	\2,(a5)+
	endm

; wait for the bench's go
WAITGO	macro
	move.l	#1,READY
w\@:	tst.l	GO
	beq.s	w\@
	endm

; FPU instructions as the words EmuTOS uses (bios/include/asmdefs.h)
FNOP_		macro
	dc.l	$F2800000
	endm

; ---------------------------------------------------------------- vectors
	org	$E00000
	dc.l	SSP
	dc.l	start
	rept	5			; 2-6
	dc.l	h_unexp
	endr
	dc.l	h_exc			; 7  TRAPcc
	rept	3			; 8-10
	dc.l	h_unexp
	endr
	dc.l	h_exc			; 11 Line-F
	rept	2			; 12-13
	dc.l	h_unexp
	endr
	dc.l	h_exc			; 14 format error
	rept	241			; 15-255
	dc.l	h_unexp
	endr

	org	$E00400
start:
	lea	$E00000,a0		; copy the vector table (1 KB) to RAM at 0
	lea	0,a1
	moveq	#31,d2
cpv:	movem.l	(a0)+,d0-d1/d3-d7/a2	; 8 longs per pass
	movem.l	d0-d1/d3-d7/a2,(a1)
	lea	32(a1),a1
	dbra	d2,cpv
	move.l	#EXLOG,EXLOGP
	move.l	#RECLOG,a5
	bra	main

; ---------------------------------------------------------------- exceptions
; Frame: 0 SR, 2 PC, 6 format/vector offset, 8 instruction address (format 2).
; Every exception is logged (vector, format, PC, IA, SR).  Format 0 frames
; (pre-instruction) are stepped over by SKIP; others return as they are.
logexc	macro
	movem.l	d0-d2/a0-a1,-(sp)
	lea	20(sp),a0
	move.l	EXLOGP,a1
	moveq	#0,d0
	move.w	6(a0),d0
	move.l	d0,d1
	and.l	#$0FFF,d0
	lsr.l	#2,d0
	lsr.l	#8,d1
	lsr.l	#4,d1
	move.l	d0,(a1)+		; vector
	move.l	d1,(a1)+		; format
	move.l	2(a0),(a1)+		; PC
	moveq	#0,d2
	cmp.l	#2,d1
	beq.s	yia\@
	cmp.l	#9,d1
	bne.s	nia\@
yia\@:	move.l	8(a0),d2
nia\@:	move.l	d2,(a1)+		; IA
	moveq	#0,d2
	move.w	(a0),d2
	move.l	d2,(a1)+		; SR
	move.l	CASEIX,(a1)+
	lea	8(a1),a1
	move.l	a1,EXLOGP
	addq.l	#1,EXCNT
	endm

h_exc:
	logexc
	tst.l	ABORTMODE
	beq.s	hx0
	cmp.l	#7,d0			; case mode: TRAPcc is counted, expected soft exceptions are stepped over
	bne.s	hxs
	cmp.l	#2,d1			; (a BSUN pre-instruction exception, vector 48, has format 0)
	bne.s	hxs
	addq.l	#1,TRAPCNT
	bra.s	hx2
hxs:	tst.l	SOFTEXC
	bne.s	hx0
	bra	habort
hx0:	cmp.l	#11,d0
	bne.s	hx1
	move.b	#1,FLHIT
hx1:	tst.l	d1
	bne.s	hx2
	move.l	SKIP,d2
	add.l	d2,2(a0)
hx2:	movem.l	(sp)+,d0-d2/a0-a1
	rte

h_unexp:
	logexc
	tst.l	ABORTMODE
	bne	habort
	move.l	#1,UNEXP
	bra	finish

habort:	movea.l	SAVESP,sp		; drop the exception frame, continue after the case
	movea.l	RESUME,a0
	jmp	(a0)

finish:
	move.l	#$D0E0600D,DONE
fin1:	stop	#$2700
	bra.s	fin1

; ---------------------------------------------------------------- helpers
; check the newest exception log entry: PC equals the given address -> d1 = 1/0
; (log entry layout: 32 bytes; newest = EXLOGP-32)
LASTPC_IS	macro
	move.l	EXLOGP,a1
	move.l	-24(a1),d1
	cmp.l	#\1,d1
	seq	d1
	and.l	#1,d1
	endm

; one instruction at a time: set SKIP to its length, run it, record the
; vector, frame format, "stacked PC is the instruction" and the number of
; exceptions it caused.  \1 = tag base, \2 = start label, \3 = end label.
; (the instruction sits between the two labels in the caller)
CPCHK	macro
	move.l	EXLOGP,a1
	REC	\1+0,-32(a1)
	REC	\1+1,-28(a1)
	LASTPC_IS	\2
	REC	\1+2,d1
	move.l	EXCNT,d1
	sub.l	d7,d1
	REC	\1+3,d1
	endm
