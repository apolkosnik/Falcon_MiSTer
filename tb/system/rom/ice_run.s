; Bare test ROM (ap040/ap060 builds): REVERT.PRG's ICE depacker on the real
; file.  The RAM image (rom/ice_ref.py ... RAMHEX, loaded with +ram=) holds
; the program at text $23DAC as GEMDOS loads it and every vector at FAIL.
; The processor is set up as EmuTOS sets up a 68040/68060 (bios/processor.S:
; caches on, ITT0 $00FFE000, DTT0 $00FFA040, DTT1 $00FF8000, TC 0) and runs
; the depacker in user mode, as REVERT does under TOS; its RTS returns to
; DONE through a pushed address.  Assemble with -DIMASK=n: 7 no interrupts,
; 3 the VBL (as under TOS), 1 the VBL and an HBL every line; their handlers
; are an RTE, every other vector is FAIL.
;   $400  $D0E0600D when it stopped; $404 the vector offset that stopped it
;         ($0080: TRAP #0 from DONE, the depacker returned; anything else is
;         an exception in the depacker); $408 a5, $40C a6, $410 the PC

	org	$E00000
	dc.l	$00010000		; SSP
	dc.l	start
start:	move.w	#$2700,sr
	cinva	bc
	moveq	#0,d0
	movec	d0,tc
	move.l	#$00FFE000,d0
	movec	d0,itt0
	moveq	#0,d0
	movec	d0,itt1
	move.l	#$00FFA040,d0
	movec	d0,dtt0
	move.l	#$00FF8000,d0
	movec	d0,dtt1
	move.l	#$80008000,d0		; DE, IE
	movec	d0,cacr
	lea	$00D00000,a0		; user stack
	move.l	#done,-(a0)		; the depacker's rts comes back here
	move.l	a0,usp
	lea	$00023DAC+$216,a4	; as the stub sets them (lea $23fc2(pc),a4 ...)
	move.l	a4,a5
	add.l	#$00559695,a5
	move.l	a4,a6
	add.l	#$00910430,a6
	move.w	#0,-(sp)			; a format 0 frame: format/vector,
	move.l	#$00023E5C,-(sp)	; PC: the depacker,
	move.l	#rtestub,$68		; level 2 (HBL) and 4 (VBL) autovectors: RTE
	move.l	#rtestub,$70
	move.w	#IMASK<<8,-(sp)		; SR: user mode, interrupt mask IMASK
	rte

done:	trap	#0			; back to supervisor
rtestub:	rte
	org	$E00100
fail:	move.w	6(sp),d0
	and.l	#$0FFF,d0
	move.l	d0,$404
	move.l	a5,$408
	move.l	a6,$40C
	move.l	2(sp),$410
	move.l	#$D0E0600D,$400
	cpusha	dc
.h:	bra.s	.h
