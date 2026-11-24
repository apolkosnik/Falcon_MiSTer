; Bare test ROM (ap040/ap060 builds, CT60 mode): the processor's memory
; paths for +buslat, the data cache off (-DDC=0: every data access a single
; bus transfer, as EmuTOS's supervisor data under DTT0) or on (-DDC=1).
; The instruction cache is on.
; Assemble: vasmm68k_mot -Fbin -m68040 -DDC=n -o membench_n.bin membench.s
; Run:      --fastram 2 --done 400 +buslat
;   reads:  16 KB of long words from Fast RAM ($01000000), ST-RAM
;           ($00100000), ROM ($E00000)
;   writes: 16 KB of long words to Fast RAM and to ST-RAM
;   $400: $D0E0600D when done
	org	$E00000
	dc.l	$00010000
	dc.l	start
start:	move.w	#$2700,sr
	cinva	bc
	moveq	#0,d0
	movec	d0,tc
	movec	d0,itt0
	movec	d0,itt1
	movec	d0,dtt0
	movec	d0,dtt1
	if DC
	move.l	#$80008000,d0		; data and instruction caches
	else
	move.l	#$00008000,d0		; the instruction cache only
	endif
	movec	d0,cacr
	lea	$01000000,a0		; Fast RAM reads
	bsr	rd16k
	lea	$00100000,a0		; ST-RAM reads
	bsr	rd16k
	lea	$00E00000,a0		; ROM reads
	bsr	rd16k
	lea	$01100000,a0		; Fast RAM writes
	bsr	wr16k
	lea	$00200000,a0		; ST-RAM writes
	bsr	wr16k
	move.l	#$D0E0600D,$400
	cpusha	dc
.h:	bra.s	.h

rd16k:	move.w	#16*1024/64-1,d1
.r:	rept	16
	move.l	(a0)+,d0
	endr
	dbra	d1,.r
	rts
wr16k:	move.w	#16*1024/64-1,d1
	moveq	#0,d0
.w:	rept	16
	move.l	d0,(a0)+
	endr
	dbra	d1,.w
	rts
