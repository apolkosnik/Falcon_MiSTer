; Bare test ROM (ap040/ap060 builds): the cost of stores with the MMU off.
; Runs the body of Starstruck's inflate loop ($01040290 in the demo: two
; bytes per pass, each with a byte load, a push and a pop) ITER times, the
; source in Fast RAM, the stack where PHASE says; then writes the done
; marker.  Assemble with -DPHASE=n:
;   0  no loop (the boot overhead)
;   1  stack in Fast RAM, default caching: write-through (MMU off)
;   2  stack in Fast RAM, DTT0 makes $01xxxxxx copyback
;   3  stack in ST-RAM (write-through), source still in Fast RAM
;   4  stack and source in ST-RAM
;   5  as 1, the loop copied to and run from Fast RAM
;   6  as 1, the loop in user mode (SR $0300, autovectors to an RTE)
;   7  as 1, plus zlib's "here" pattern: a long store to the stack frame
;      followed by byte loads of the same long word (the demo's $010402c4)
;   9  5 + 6 + 7 together (the demo's situation)
;  10  as 6 (user mode), DTT0 makes $01xxxxxx copyback
;  11  as 6 (user mode), stack and source in ST-RAM, code in ROM
;  12  as 9, with EmuTOS's TTRs: ITT0 all/write-through, DTT0 all/
;      supervisor/noncachable serialized, DTT1 all/user/write-through
;  13  as 12 but in supervisor mode (a demo after Super(): DTT0 applies)
;  14  as 13 with TTRFIX's mapping: DTT0 $FF00C040 (I/O noncachable),
;      DTT1 $00FFC000 (the rest write-through, any mode)
;  15  as 13 with TTRFIX's final mapping: DTT0 $0000A040 (ST block supervisor
;      noncachable), DTT1 $FF00C040 (I/O mirror noncachable), Fast RAM default
; The run's "simulated N ms" minus PHASE 0's is the loop's time.
ITER	equ	250000

	if (PHASE==5)||(PHASE==9)||(PHASE==12)||(PHASE==13)||(PHASE==14)||(PHASE==15)
CODE_FAST	equ	1
	else
CODE_FAST	equ	0
	endif
	if (PHASE==6)||(PHASE==9)||(PHASE==10)||(PHASE==11)||(PHASE==12)
USER	equ	1
	else
USER	equ	0
	endif
	if (PHASE==7)||(PHASE==9)||(PHASE==12)||(PHASE==13)||(PHASE==14)||(PHASE==15)
PATTERN	equ	1
	else
PATTERN	equ	0
	endif

	org	$E00000
	dc.l	$00010000		; SSP
	dc.l	start
start:	move.w	#$2700,sr
	cinva	bc
	move.l	#$80008000,d0		; IE, DE
	movec	d0,cacr
	moveq	#0,d0
	movec	d0,dtt0
	movec	d0,dtt1
	movec	d0,itt0
	movec	d0,itt1

	if (PHASE==2)||(PHASE==10)
	move.l	#$0100C020,d0		; base $01, mask $00, E, S ignored, CM copyback
	movec	d0,dtt0
	endif
	if (PHASE==12)||(PHASE==13)
	move.l	#$00FFE000,d0		; EmuTOS (bios/processor.S): code, all, write-through
	movec	d0,itt0
	move.l	#$00FFA040,d0		; supervisor data: noncachable serialized
	movec	d0,dtt0
	move.l	#$00FF8000,d0		; user data: write-through
	movec	d0,dtt1
	endif
	if PHASE==14
	move.l	#$00FFE000,d0
	movec	d0,itt0
	move.l	#$FF00C040,d0		; TTRFIX: I/O noncachable serialized
	movec	d0,dtt0
	move.l	#$00FFC000,d0		; the rest write-through, any mode
	movec	d0,dtt1
	endif
	if PHASE==15
	move.l	#$00FFE000,d0
	movec	d0,itt0
	move.l	#$0000A040,d0		; TTRFIX: the ST block, supervisor: noncachable
	movec	d0,dtt0
	move.l	#$FF00C040,d0		; the I/O mirror: noncachable
	movec	d0,dtt1
	endif

	if USER
	lea	$8,a0			; vectors 2-255: an RTE
	move.w	#254-1,d0
	lea	rte_stub(pc),a1
.v:	move.l	a1,(a0)+
	dbra	d0,.v
	lea	done(pc),a1
	move.l	a1,$80			; TRAP #0: the done marker (supervisor only: $0-$7FF)
	endif

	if (PHASE==4)||(PHASE==11)
	lea	$00100000,a3		; source
	else
	lea	$01000000,a3
	endif
	if (PHASE==3)||(PHASE==4)||(PHASE==11)
	lea	$00080000,sp		; stack
	else
	lea	$01100000,sp
	endif

	if CODE_FAST
	lea	loop(pc),a0		; the loop copied to Fast RAM
	lea	$01200000,a1
	move.l	#(loop_end-loop)/4-1,d0
.cp:	move.l	(a0)+,(a1)+
	dbra	d0,.cp
	cpusha	dc
	cinva	ic
	lea	$01200000,a4
	else
	lea	loop(pc),a4
	endif

	if USER
	move.l	sp,a0
	move.l	a0,usp
	lea	$00010000,sp		; the supervisor stack for the RTEs
	move.w	#$0300,sr		; user mode, HBL masked
	endif

	if PHASE>0
	jsr	(a4)
	endif
	if USER
	trap	#0
	endif
done:	move.l	d5,$404			; the checksum (keeps the loop honest)
	move.l	#$D0E0600D,$400
	cpusha	dc
.h:	bra.s	.h

rte_stub:
	rte

; the loop: a subroutine, position independent, long aligned
	cnop	0,4
loop:	lea	-$80(sp),sp		; a stack frame as zlib's inflate_fast has
	move.l	#ITER,d7
	moveq	#0,d4
	moveq	#0,d5
	moveq	#1,d2
.l:	addq.l	#1,a3			; the demo's loop, $01040290-$010402bc
	moveq	#0,d1
	move.b	(a3)+,d1
	move.l	d4,d0
	move.l	d2,-(sp)
	move.l	d1,d2
	lsl.l	d0,d2
	move.l	d2,d0
	move.l	(sp)+,d2
	add.l	d0,d5
	addq.l	#8,d4
	moveq	#0,d1
	move.b	(a3),d1
	move.l	d4,d0
	move.l	d2,-(sp)
	move.l	d1,d2
	lsl.l	d0,d2
	move.l	d2,d0
	move.l	(sp)+,d2
	add.l	d0,d5
	addq.l	#8,d4
	if PATTERN
	move.l	d4,$54(sp)		; here = lcode[...] ($010402c4)
	moveq	#0,d3
	move.b	$55(sp),d3		; here.bits
	lsr.l	d3,d5
	moveq	#0,d3
	move.b	$54(sp),d3		; here.op
	add.l	d3,d5
	endif
	and.l	#$1f,d4			; keep the shift counts small
	subq.l	#1,d7
	bne.s	.l
	lea	$80(sp),sp
	rts
	cnop	0,4
loop_end:
