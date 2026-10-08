; t_detect.s - the EmuTOS FPU detection (bios/processor.S _detect_fpu, 68030
; case) run unchanged on the real CPU and bridge.
;
; Expected (processor.S header): service alive -> d0 = $00060000 (68882 for
; sure); no FPU -> 0 (FRESTORE takes Line-F).
	include	"common.i"

main:
	WAITGO
	bsr	detect_fpu
	REC	T_COOKIE,d0
	bra	finish

; ---- replica of _detect_fpu (CONF_WITH_ADVANCED_CPU, no SFP004, mcpu = 30)
detect_fpu:
	movem.l	d2/a2,-(sp)
	move.l	sp,a0			; save the ssp
	moveq	#0,d0			; assume no FPU
	move.l	($2c).w,a1		; save the Line-F vector
	move.l	($08).w,a2
	move.l	#fexit,d1
	move.l	d1,($2c).w		; install temporary Line-F
	move.l	d1,($08).w
	nop				; flush pipelines

	moveq	#30,d1			; _mcpu = 68030 (the C code reads _mcpu)
	cmpi.w	#20,d1
	bmi	fexit

	cmpi.w	#60,d1
	bmi	no60
	nop				; (68060 PCR code not reachable here)
no60:	clr.l	-(sp)			; push NULL frame
	clr.l	-(sp)			; extra longs for 68060
	clr.l	-(sp)
	dc.w	$f35f			; FRESTORE_SP_PLUS: frestore (sp)+, reset FPU into NULL state
	dc.l	$f2800000		; FNOP: force it into IDLE state
	dc.w	$f327			; FSAVE_MINUS_SP: fsave -(sp), save the IDLE frame

	moveq	#$10,d0			; assume 68060 FPU (cookie 0x00100000)
	cmpi.w	#60,d1
	beq	fexit
	moveq	#$08,d0			; if not 060, maybe 040
	cmpi.w	#40,d1
	beq	fexit
	moveq	#$06,d0			; if neither, maybe a 68882 (0x00060000)
	move.b	1(sp),d1		; get offset to last long of IDLE frame
	cmpi.b	#$38,d1			; is it a 68882?
	beq	fexit			; yes, branch
	moveq	#$04,d0			; must be 68881

fexit:	move.l	a1,($2c).w		; restore Line-F
	move.l	a2,($08).w
	move.l	a0,sp
	nop				; flush pipelines
	swap	d0
	movem.l	(sp)+,d2/a2
	rts
