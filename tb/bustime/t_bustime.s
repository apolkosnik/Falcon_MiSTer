; t_bustime.s - bus timing bench program (tb_bustime.sv, docs/CPU_TIMING.md)
;
; Runs from "ROM" at $E00000 like TOS and exercises every kind of bus cycle
; the Falcon bus model times: ST-RAM in all sizes and alignments, the
; read-ahead buffer and the posted write (read after write, a DMA write into
; the buffered word), ROM, the devices with Hatari wait states (MFP, YM,
; ACIA, FDC/DMA, DSP host port) and without (Videl, IDE), bus errors in
; CHIP16 and FAST16 space and from user mode, the four interrupt
; acknowledge kinds and TAS, with the caches off and on.  The bench checks
; the length of every cycle; the program checks the data.
;
; Test device (tb_bustime.sv) at $FFFF00:
;   +0  w  $600D: passed, $BAD0: failed (number in +16)
;   +2  w  console character
;   +4  w  interrupt request: level [2:0], kind [5:4] (0 MFP vectored,
;          1 DSP vectored, 2 autovector, 3 none: spurious), vector [15:8]
;   +6  w  release the interrupt request
;   +8  l  DMA write address; +12 w: DMA write data (starts the write)
;   +14 w  read: DMA write busy
;   +16 w  failing test number

TDEV	equ	$FFFF00
STACK	equ	$8000
berrs	equ	$4000			; variables in ST-RAM
irqs	equ	$4004
BUF	equ	$10000
VMFP	equ	$48
VDSP	equ	$49

	org	$E00000
	dc.l	STACK
	dc.l	start

start:	move.w	#$2700,sr
	lea	8,a0			; exception vectors in RAM
	move.w	#254-1,d0
.vec:	move.l	#unexp,(a0)+
	dbra	d0,.vec
	move.l	#buserr,8
	move.l	#irq_spur,$60
	move.l	#irq_av4,$70
	move.l	#irq_mfp,VMFP*4
	move.l	#irq_dsp,VDSP*4
	moveq	#0,d0
	movec	d0,cacr
	clr.l	berrs
	clr.l	irqs

	bsr	tests
	move.l	#$00003111,d0		; caches on: I, D, write allocate
	movec	d0,cacr
	bsr	tests
	moveq	#0,d0
	movec	d0,cacr

	move.w	#$600D,TDEV
.halt:	bra.s	.halt

failed:	move.w	d7,TDEV+16
	move.w	#$BAD0,TDEV
.halt:	bra.s	.halt

unexp:	moveq	#99,d7
	bra.s	failed

;--------------------------------------------------------------------------
tests:
; 1: RAM sizes and alignments
	moveq	#1,d7
	lea	BUF,a0
	moveq	#15,d0
.fill:	move.b	d0,(a0,d0.w)		; bytes 0..15 = 0..15
	dbra	d0,.fill
	move.l	#$00010203,d2
	moveq	#3,d0
	move.l	a0,a1
.rd:	cmp.l	(a1)+,d2
	bne	failed
	add.l	#$04040404,d2
	dbra	d0,.rd
	move.l	#$A1B2C3D4,1(a0)	; misaligned long
	cmp.w	#$00A1,(a0)
	bne	failed
	cmp.w	#$B2C3,2(a0)
	bne	failed
	cmp.w	#$D405,4(a0)
	bne	failed
	move.w	#$EEFF,3(a0)		; word across a longword boundary
	cmp.l	#$00A1B2EE,(a0)
	bne	failed
	cmp.l	#$FF050607,4(a0)
	bne	failed
	move.w	#$1234,9(a0)		; odd word inside a longword
	cmp.l	#$0812340B,8(a0)
	bne	failed
	move.l	#$61626364,6(a0)	; long across the 64-bit word boundary
	cmp.l	#$FF056162,4(a0)
	bne	failed
	cmp.l	#$6364340B,8(a0)
	bne	failed

; 2: sequential reads through the read-ahead buffer
	moveq	#2,d7
	lea	BUF+$400,a0
	moveq	#63,d0
	moveq	#0,d1
.f2:	move.l	d1,(a0)+
	add.l	#$01010101,d1
	dbra	d0,.f2
	lea	BUF+$400,a0
	moveq	#63,d0
	moveq	#0,d2
.s2:	add.l	(a0)+,d2
	dbra	d0,.s2
	cmp.l	#$E7E7E7E0,d2		; sum of i*$01010101, i = 0..63
	bne	failed

; 3: a read after a write to the buffered word
	moveq	#3,d7
	lea	BUF+$800,a0
	move.l	#$11112222,(a0)
	move.l	#$33334444,4(a0)
	move.l	(a0),d0			; the word is in the buffer
	move.w	#$ABCD,2(a0)
	cmp.l	#$1111ABCD,(a0)
	bne	failed
	move.b	#$EF,5(a0)
	cmp.l	#$33EF4444,4(a0)
	bne	failed
	tst.w	6(a0)

; 4: another master writes the buffered word
	moveq	#4,d7
	lea	BUF+$C00,a0
	move.l	#$55556666,(a0)
	move.l	(a0),d0			; in the buffer (and the data cache)
	move.l	a0,d0
	addq.l	#2,d0
	move.l	d0,TDEV+8
	move.w	#$BEEF,TDEV+12
.w4:	tst.w	TDEV+14
	bne.s	.w4
	cmp.l	#$5555BEEF,(a0)
	bne	failed

; 5: ROM
	moveq	#5,d7
	lea	romtab(pc),a0
	cmp.l	#$DEADBEEF,(a0)
	bne	failed
	cmp.w	#$CAFE,4(a0)
	bne	failed
	cmp.b	#$AD,1(a0)
	bne	failed
	cmp.w	#$EFCA,3(a0)		; odd word
	bne	failed
	cmp.l	#$ADBEEFCA,1(a0)	; odd long
	bne	failed

; 6: devices (the bench's device model stores what is written)
	moveq	#6,d7
	move.b	#$5A,$FFFA01		; MFP, odd byte
	cmp.b	#$5A,$FFFA01
	bne	failed
	move.w	#$1234,$FFFA02		; word: its odd byte is a register
	move.l	$FFFA00,d0		; long: two registers
	move.b	#$00,$FFFA26		; USART: no wait
	move.b	#7,$FF8800		; YM: one access per instruction
	move.b	#$3F,$FF8802
	move.b	$FF8800,d0
	lea	$FF8800,a1
	move.w	#$0102,d1
	movep.w	d1,0(a1)		; two YM accesses in one instruction
	move.l	#$01020304,d1
	movep.l	d1,0(a1)		; four bytes, two of them YM
	move.b	$FFFC00,d0		; ACIA (E clock)
	move.b	$FFFC02,d0
	move.w	$FFFC00,d0
	move.l	$FFFC00,d0		; two accesses: E clock once
	lea	$FFFC00,a2
	movep.w	0(a2),d0
	move.b	$FFFC01,d0		; odd byte: not a register
	move.w	#$0090,$FF8606		; FDC/DMA: sector count selected
	move.w	$FF8604,d0		;   ... read without a wait
	move.w	#$0080,$FF8606
	move.w	$FF8604,d0		;   ... with
	move.w	#$0001,$FF8604
	move.w	$FF8606,d0
	move.w	$FF860E,d0
	move.b	$FF8609,d0
	move.b	$FFA207,d0		; DSP host port
	move.w	$FFA206,d0
	move.l	$FFA204,d0
	move.w	#$0123,$FF8282		; Videl
	cmp.w	#$0123,$FF8282
	bne	failed
	move.w	$F00000,d0		; IDE (CHIP16)
	move.w	#'.',TDEV+2

; 7: bus errors (the handler resumes at a5 with the stack in a4; a NOP
;    after each faulting instruction keeps a posted write's fault there)
	moveq	#7,d7
	move.l	berrs,d6
	move.l	sp,a4
	lea	.b1(pc),a5
	tst.w	$400000			; beyond ST-RAM (CHIP16)
	nop
.b1:	lea	.b2(pc),a5
	tst.w	$E80000			; above the ROM (FAST16)
	nop
.b2:	lea	.b3(pc),a5
	move.w	d0,$E00100		; a ROM write
	nop
.b3:	lea	.b4(pc),a5
	move.w	d0,0			; a reset vector write
	nop
.b4:	lea	.b5(pc),a5
	tst.w	$FF0000			; below the I/O area
	nop
.b5:	lea	.b6(pc),a5
	tst.w	$FF8E00			; I/O without a device
	nop
.b6:	lea	.b7(pc),a5
	move.l	#STACK-$400,a0
	move.l	a0,usp
	andi.w	#$DFFF,sr		; user mode
	tst.b	$FFFA01			; supervisor-only I/O
	nop
.b7:	lea	.b8(pc),a5
	andi.w	#$DFFF,sr
	tst.w	$100			; supervisor-only RAM
	nop
.b8:	addq.l	#8,d6
	cmp.l	berrs,d6
	bne	failed

; 8: interrupt acknowledge: MFP and DSP vectored, autovector, spurious
	moveq	#8,d7
	move.l	irqs,d6
	move.w	#(VMFP<<8)|$06,TDEV+4
	bsr	takeirq
	move.w	#(VDSP<<8)|$16,TDEV+4
	bsr	takeirq
	move.w	#$24,TDEV+4
	bsr	takeirq
	move.w	#$36,TDEV+4
	bsr	takeirq
	addq.l	#4,d6
	cmp.l	irqs,d6
	bne	failed

; 9: TAS
	moveq	#9,d7
	lea	BUF+$1000,a0
	clr.b	(a0)
	tas	(a0)
	bne	failed
	cmp.b	#$80,(a0)
	bne	failed
	rts

; allow interrupts until one is taken (or give up)
takeirq:
	move.l	irqs,d5
	move.w	#$2300,sr
	move.w	#2000,d4
.w:	cmp.l	irqs,d5
	dbne	d4,.w
	move.w	#$2700,sr
	tst.w	d4
	bmi	failed
	rts

;--------------------------------------------------------------------------
buserr:	addq.l	#1,berrs
	move.l	a4,sp
	jmp	(a5)

irq_mfp:
irq_dsp:
irq_av4:
irq_spur:
	move.w	d0,TDEV+6
	addq.l	#1,irqs
	rte

romtab:	dc.l	$DEADBEEF,$CAFEF00D

