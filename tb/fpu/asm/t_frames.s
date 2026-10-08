; t_frames.s - FSAVE / FRESTORE frames (MC68881/MC68882 UM 6.4.2 and the
; EmuTOS idle frame test).  Needs the FPU present.
	include	"common.i"

FILL	equ	$AAAAAAAA

; fill 64 longs at (a0) with FILL
fillbuf:
	move.w	#63,d0
fb1:	move.l	#FILL,(a0)+
	dbra	d0,fb1
	rts

main:
	WAITGO
	move.l	#2,SKIP			; FRESTORE (An) is one word

;---- 1. FSAVE right after reset: null frame, one long, format word 0
	lea	BUF,a0
	bsr	fillbuf
	lea	BUF+128,a0
	move.l	a0,a2
	dc.w	$f320			; fsave -(a0)
	move.l	a2,d1
	sub.l	a0,d1
	REC	T_FS_NULL_DELTA,d1
	moveq	#0,d1
	move.w	(a0),d1
	REC	T_FS_NULL_FMT,d1
	REC	T_FS_NULL_BEYOND,4(a0)

;---- 2. FNOP puts the FPU into idle: FSAVE -(a0) -> $1F38 + 14 longs
	lea	BUF2,a0
	bsr	fillbuf
	lea	BUF2+128,a0
	move.l	a0,a2
	FNOP_
	dc.w	$f320			; fsave -(a0)
	move.l	a2,d1
	sub.l	a0,d1
	REC	T_FS_IDLE_DELTA,d1
	moveq	#0,d1
	move.w	(a0),d1
	REC	T_FS_IDLE_FMT,d1
	moveq	#0,d1
	move.b	1(a0),d1
	REC	T_FS_IDLE_LEN,d1
	REC	T_FS_IDLE_BEYOND,60(a0)
	lea	4(a0),a3		; the 14 body longs
	move.l	#T_FS_IDLE_BODY,d3
	moveq	#13,d0
bl1:	move.l	d3,(a5)+
	move.l	(a3)+,(a5)+
	addq.l	#1,d3
	dbra	d0,bl1
	lea	BUF4,a3			; keep the idle frame for the restore test
	move.w	#14,d0
cp1:	move.l	(a0)+,(a3)+
	dbra	d0,cp1

;---- 3. FSAVE (a0): same frame, written upward, a0 unchanged
	lea	BUF3,a0
	bsr	fillbuf
	lea	BUF3,a0
	FNOP_
	dc.w	$f310			; fsave (a0)
	moveq	#0,d1
	move.w	(a0),d1
	REC	T_FS_CTRL_FMT,d1
	cmp.l	#BUF3,a0
	seq	d1
	and.l	#1,d1
	REC	T_FS_CTRL_A0,d1
	REC	T_FS_CTRL_BEYOND,60(a0)

;---- 4. FRESTORE of a null frame: FPU back to null
	clr.l	BUF
	lea	BUF,a1
	FNOP_
	dc.w	$f351			; frestore (a1)
	cmp.l	#BUF,a1
	seq	d1
	and.l	#1,d1
	REC	T_FR_NULL_A1,d1
	lea	BUF2+128,a0
	move.l	a0,a2
	dc.w	$f320			; fsave -(a0)
	moveq	#0,d1
	move.w	(a0),d1
	REC	T_FR_NULL_FMT,d1
	move.l	a2,d1
	sub.l	a0,d1
	REC	T_FR_NULL_DELTA,d1
	lea	BUF,a1
	FNOP_
	dc.w	$f359			; frestore (a1)+
	move.l	a1,d1
	sub.l	#BUF,d1
	REC	T_FR_NULL_ADV,d1

;---- 5. a null frame whose low byte is not 0 ($0012): still null (Hatari: version 0)
	move.l	#$00120000,BUF
	lea	BUF,a1
	FNOP_
	move.l	EXCNT,d7
	dc.w	$f351			; frestore (a1)
	move.l	EXCNT,d1
	sub.l	d7,d1
	REC	T_FR_NZ_CNT,d1
	lea	BUF2+128,a0
	dc.w	$f320			; fsave -(a0)
	moveq	#0,d1
	move.w	(a0),d1
	REC	T_FR_NZ_FMT,d1

;---- 6. FRESTORE of an idle frame: back to idle (from null), 60 bytes consumed
	lea	BUF4,a1			; the idle frame saved in step 2
	move.l	EXCNT,d7
	dc.w	$f359			; frestore (a1)+  (FPU is null here)
	move.l	a1,d1
	sub.l	#BUF4,d1
	REC	T_FR_IDLE_ADV,d1
	move.l	EXCNT,d1
	sub.l	d7,d1
	REC	T_FR_IDLE_CNT,d1
	lea	BUF3+128,a0
	move.l	a0,a2
	dc.w	$f320			; fsave -(a0)
	moveq	#0,d1
	move.w	(a0),d1
	REC	T_FR_IDLE_FMT,d1
	move.l	a2,d1
	sub.l	a0,d1
	REC	T_FR_IDLE_DELTA,d1

;---- 7. invalid formats: format error exception (vector 14), pre-instruction frame
	moveq	#0,d6
	lea	bad_tab,a4
bad1:	move.w	(a4)+,d5
	cmp.w	#$FFFF,d5
	beq	bad_done
	move.w	d5,BUF			; frame word, rest zero
	clr.w	BUF+2
	lea	BUF,a1
	clr.l	BUF+4
	move.l	EXCNT,d7
badi:	dc.w	$f351			; frestore (a1)
	moveq	#1,d4			; reached: execution continued after the handler
	move.l	EXCNT,d1
	sub.l	d7,d1
	move.l	d6,d3
	lsl.l	#3,d3			; 8 tags per case
	add.l	#T_BADF,d3
	move.l	d3,(a5)+
	move.l	d1,(a5)+		; +0 exceptions
	move.l	EXLOGP,a3
	addq.l	#1,d3
	move.l	d3,(a5)+
	move.l	-32(a3),(a5)+		; +1 vector
	addq.l	#1,d3
	move.l	d3,(a5)+
	move.l	-28(a3),(a5)+		; +2 frame format
	addq.l	#1,d3
	move.l	d3,(a5)+
	move.l	-24(a3),d1
	cmp.l	#badi,d1
	seq	d1
	and.l	#1,d1
	move.l	d1,(a5)+		; +3 stacked PC is the FRESTORE
	addq.l	#1,d3
	move.l	d3,(a5)+
	move.l	d4,(a5)+		; +4 continued
	addq.l	#1,d6
	bra	bad1
bad_done:

;---- 8. after the format errors the FPU still answers
	move.l	EXCNT,d7
	FNOP_
	move.l	EXCNT,d1
	sub.l	d7,d1
	REC	T_FR_AFTERBAD,d1

;---- 9. frames Hatari's fpuop_restore accepts on a 68882: 68881 idle $1F18, busy $1FD4 / $1FB4
;     (the format long is followed by 'length' bytes of zeros)
	moveq	#0,d6
	lea	odd_tab,a4
od1:	move.w	(a4)+,d5
	cmp.w	#$FFFF,d5
	beq	od_done
	lea	BUF,a0
	moveq	#63,d0
od0:	clr.l	(a0)+
	dbra	d0,od0
	move.w	d5,BUF
	lea	BUF,a1
	move.l	EXCNT,d7
	dc.w	$f359			; frestore (a1)+
	move.l	EXCNT,d1
	sub.l	d7,d1
	move.l	d6,d3
	lsl.l	#2,d3
	add.l	#T_ODDF,d3
	move.l	d3,(a5)+
	move.l	d1,(a5)+		; +0 exceptions
	addq.l	#1,d3
	move.l	d3,(a5)+
	move.l	a1,d1
	sub.l	#BUF,d1
	move.l	d1,(a5)+		; +1 bytes consumed
	addq.l	#1,d6
	bra	od1
od_done:
	bra	finish

; invalid frame format words: garbage ($1234)
bad_tab:
	dc.w	$1234,$FFFF
odd_tab:
	dc.w	$1F18,$1FD4,$1FB4,$FFFF
