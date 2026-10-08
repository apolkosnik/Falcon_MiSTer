; simrom.s - a stand-in ROM for running FPUTEST in the full-system simulation
; (tb/system) without booting TOS: it relocates the TOS program appended at
; `prg` to RAM and runs it in user mode with just the calls FPUTEST and
; FPUBENCH make:
;   GEMDOS (trap #1) Pterm0 0, Cconin 1, Cconout 2, Cconws 9
;   BIOS   (trap #13) Setexc 5
;   XBIOS  (trap #14) Supexec 38
; a cookie jar with _FPU when the FPU is there (detected as TOS does it:
; FRESTORE of a null frame ends in the F-line exception without an FPU; the
; core reports one once falcon_fpu's heartbeat is seen, so this retries for
; ~0.3 s), and _hz_200 counting VBLs.  Console output goes to TEXT; Pterm0 writes
; $D0E0600D to DONE (sim_main --text / --done).  Unexpected exceptions print
; their vector and end the run.
;
;   vasmm68k_mot -Fbin -m68030 -m68882 -no-opt -o simrom.img simrom.s

TEXT	equ	$100000
DONE	equ	$0FFFF0
BASE	equ	$20000			; the program's text segment
USTACK	equ	$0F0000

	org	$E00000
	dc.l	$00008000		; reset: SSP
	dc.l	start			; reset: PC

start:	move.w	#$2700,sr
	lea	$8000,sp
	; vectors 2..255: unexpected; then the ones used
	lea	8.w,a0
	move.w	#253,d0
sv:	move.l	#h_unexp,(a0)+
	dbra	d0,sv
	move.l	#h_vbl,$70.w		; level 4 autovector: VBL
	move.l	#h_trap1,$84.w
	move.l	#h_trap13,$B4.w
	move.l	#h_trap14,$B8.w
	; the FPU: FRESTORE of a null frame until it no longer takes F-line
	move.l	#h_nofpu,$2C.w
	clr.l	null_frame
	move.l	#100000,d7
fpu_wait:
	clr.w	no_fpu
	lea	null_frame,a0
	frestore (a0)
	tst.w	no_fpu
	beq.s	fpu_ok
	subq.l	#1,d7
	bne.s	fpu_wait
fpu_ok:	move.l	#h_unexp,$2C.w
	; cookie jar at $600: _FPU = $00020000 (68881/2) if found, as TOS 4.04
	lea	$600.w,a0
	move.l	a0,$5A0.w
	move.l	#'_FPU',(a0)+
	moveq	#0,d0
	tst.l	d7
	beq.s	fpu_none
	move.l	#$00020000,d0
fpu_none:
	move.l	d0,(a0)+
	clr.l	(a0)+
	move.l	#8,(a0)+
	clr.l	$4BA.w
	clr.l	DONE
	move.l	#TEXT,txtp
	clr.b	TEXT

	; ---- relocate the program ----
	lea	prg,a0
	move.l	2(a0),d1		; text
	move.l	6(a0),d2		; data
	move.l	10(a0),d3		; bss
	move.l	14(a0),d4		; symbols
	lea	28(a0),a1
	lea	BASE,a2
	move.l	d1,d0
	add.l	d2,d0
	subq.l	#1,d0
cp:	move.b	(a1)+,(a2)+
	subq.l	#1,d0
	bpl.s	cp
	move.l	d3,d0			; clear the bss
	beq.s	bss_done
	subq.l	#1,d0
cb:	clr.b	(a2)+
	subq.l	#1,d0
	bpl.s	cb
bss_done:
	add.l	d4,a1			; past the symbols: the relocation table
	move.l	(a1)+,d0
	beq.s	reloc_done
	lea	BASE,a2
	add.l	d0,a2
	move.l	#BASE,d5
	add.l	d5,(a2)
rl:	moveq	#0,d0
	move.b	(a1)+,d0
	beq.s	reloc_done
	cmp.b	#1,d0
	bne.s	rl_fix
	lea	254(a2),a2
	bra.s	rl
rl_fix:	add.l	d0,a2
	add.l	d5,(a2)
	bra.s	rl
reloc_done:

	; ---- run it in user mode ----
	lea	USTACK,a0
	move.l	a0,usp
	move.w	#$0000,-(sp)		; format 0 frame for RTE: vector word
	move.l	#BASE,-(sp)		; PC
	move.w	#$0300,-(sp)		; SR: user mode, IPL 3 (VBL on)
	rte

; ---- traps: the arguments are on the caller's (user) stack ----
h_trap1:
	move.l	usp,a0
	move.w	(a0),d0
	cmp.w	#9,d0
	beq.s	t_cconws
	cmp.w	#2,d0
	beq.s	t_cconout
	cmp.w	#1,d0
	beq.s	t_cconin
	tst.w	d0
	beq.s	t_pterm
	moveq	#-32,d0			; EINVFN
	rte
t_cconws:
	move.l	2(a0),a1
	move.l	txtp,a2
tw:	move.b	(a1)+,(a2)+
	bne.s	tw
	subq.l	#1,a2
	move.l	a2,txtp
	moveq	#0,d0
	rte
t_cconout:
	move.l	txtp,a2
	move.b	3(a0),(a2)+
	clr.b	(a2)
	move.l	a2,txtp
	moveq	#-1,d0
	rte
t_cconin:
	moveq	#' ',d0
	rte
t_pterm:
	move.l	#$D0E0600D,DONE
tp:	stop	#$2700
	bra.s	tp

h_trap13:
	move.l	usp,a0
	cmp.w	#5,(a0)
	bne.s	t13_x
	moveq	#0,d0
	move.w	2(a0),d0		; Setexc(vector, address)
	lsl.l	#2,d0
	move.l	d0,a1
	move.l	(a1),d0			; old
	move.l	4(a0),d1
	cmp.l	#-1,d1
	beq.s	t13_r
	move.l	d1,(a1)
t13_r:	rte
t13_x:	moveq	#-1,d0
	rte

h_trap14:
	move.l	usp,a0
	cmp.w	#38,(a0)
	bne.s	t14_x
	move.l	2(a0),a0		; Supexec(routine)
	movem.l	d1-d7/a1-a6,-(sp)
	jsr	(a0)
	movem.l	(sp)+,d1-d7/a1-a6
	rte
t14_x:	moveq	#-1,d0
	rte

h_nofpu:
	move.w	#1,no_fpu		; F-line from FRESTORE (A0): skip it
	addq.l	#2,2(sp)
	rte

h_vbl:	addq.l	#3,$4BA.w		; ~200 Hz from a 60-70 Hz VBL
	rte

h_unexp:
	move.w	#$2700,sr
	move.l	txtp,a2
	lea	m_unexp(pc),a1
ux:	move.b	(a1)+,(a2)+
	bne.s	ux
	subq.l	#1,a2
	move.w	6(sp),d0		; format/vector word
	and.w	#$0FFF,d0
	lsr.w	#2,d0
	bsr.s	hexw
	move.b	#' ',(a2)+
	move.l	2(sp),d1		; pc
	swap	d1
	move.w	d1,d0
	bsr.s	hexw
	swap	d1
	move.w	d1,d0
	bsr.s	hexw
	clr.b	(a2)
	move.l	#$D0E0600D,DONE
ul:	stop	#$2700
	bra.s	ul
hexw:	moveq	#3,d2			; d0.w as 4 hex digits at (a2)+
hx:	rol.w	#4,d0
	move.b	d0,d3
	and.b	#15,d3
	add.b	#'0',d3
	cmp.b	#'9',d3
	ble.s	hx1
	addq.b	#7,d3
hx1:	move.b	d3,(a2)+
	dbra	d2,hx
	rts

m_unexp: dc.b	13,10,"*** unexpected exception, vector/pc: ",0
	even
txtp	equ	$5F0			; output pointer (RAM)
no_fpu	equ	$5F4
null_frame equ	$5F8

	even
prg:	incbin	"prg.tos"
