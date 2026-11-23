; Bare test ROM (ap060): FPSP060's get_cpu_typ (th-otto/fpsp fpspany/getcpu.S),
; transcribed for vasm; $404 = the CPU it reports (60 expected)
_longframe	equ	$59e		; TOS system variable
	org	$E00000
	dc.l	$00010000
	dc.l	start
start:	move.w	#$2700,sr
	bsr	get_cpu_typ
	move.l	d0,$404
	move.l	#$D0E0600D,$400
.h:	bra.s	.h
get_cpu_typ:
 move.l   $10.w,-(a7)
 move.l   $2c.w,-(a7)
 move.l   $f4.w,-(a7)
 move.l   sp,a0
 moveq    #0,d0
 nop
 lea      set_cpu_typ(pc),a1
 move.l   a1,$10.w
 move.l   a1,$2c.w
 move.l   a1,$f4.w

 move     ccr,d1
 moveq    #10,d0
 movec  cacr,d1
 move.l   d1,d0
 ori.w    #$0101,d1

 movec    d1,cacr
 movec    cacr,d1
 movec    d0,cacr
 btst     #0,d1
 beq.s    x040
 moveq    #20,d0
 btst     #8,d1
 beq    set_cpu_typ
 dc.w $f039,$4200,0,64
 moveq    #30,d0
 bra.s    set_cpu_typ



x040:
 moveq    #40,d0
 dc.w     $4e7a,$1808
 moveq    #60,d0

 dc.w    $06d0,$0000
 moveq    #40,d0

set_cpu_typ:
 move.l   a0,sp
 nop
 move.l   (a7)+,$f4.w
 move.l   (a7)+,$2c.w
 move.l   (a7)+,$10.w
 move.w   d0,_longframe.w
 rts
