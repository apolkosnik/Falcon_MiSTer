; Bare test ROM (ap040/ap060 builds): the processor's memory paths under a
; streaming load, caches on (no TTRs, translation off: write-through
; cachable everywhere), for +buslat.  Each phase touches more than the
; data cache holds, so every line read misses it.
;   reads:  256 KB of long words from Fast RAM ($01000000), ST-RAM
;           ($00100000), ROM ($E00000)
;   writes: 64 KB of long words to Fast RAM and to ST-RAM (write-through,
;           no allocate: one single write each)
; $400: $D0E0600D when done; $404.. the CPU clock-less phase markers are
; not needed: the regions separate the phases.
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
	move.l	#$80008000,d0		; data and instruction caches
	movec	d0,cacr
	lea	$01000000,a0		; Fast RAM reads
	bsr	rd256k
	lea	$00100000,a0		; ST-RAM reads
	bsr	rd256k
	lea	$00E00000,a0		; ROM reads
	bsr	rd256k
	lea	$01100000,a0		; Fast RAM writes
	bsr	wr64k
	lea	$00200000,a0		; ST-RAM writes
	bsr	wr64k
	move.l	#$D0E0600D,$400
	cpusha	dc
.h:	bra.s	.h

rd256k:	move.w	#256*1024/64-1,d1
.r:	rept	16
	move.l	(a0)+,d0
	endr
	dbra	d1,.r
	rts
wr64k:	move.w	#64*1024/64-1,d1
	moveq	#0,d0
.w:	rept	16
	move.l	d0,(a0)+
	endr
	dbra	d1,.w
	rts
