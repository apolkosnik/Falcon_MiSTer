; cpall.i - CPALL k: the seven coprocessor instruction forms for CpID k, each
; expected to take the F-line exception on its first CIR access.
; Encodings (MC68030 UM 10.4):  1111 CpID(3) type ... :
;   cpGEN   $F000+k*$200, command word
;   cpBcc   $F080+k*$200+cc, displacement
;   cpScc   $F040+k*$200+ea, condition word    (ea = D1)
;   cpDBcc  $F048+k*$200+Dn, condition, disp   (Dn = D3)
;   cpTRAPcc $F078+opmode+k*$200 (opmode 2 = .W operand), condition, operand
;   cpSAVE  $F100+k*$200+ea                    (ea = (a0))
;   cpRESTORE $F140+k*$200+ea
CPALL	macro
	move.l	#4,SKIP
	move.l	EXCNT,d7
g\@:	dc.w	$F000+\1*$200,$0000
	CPCHK	T_CPID+\1*32+0,g\@
	move.l	#4,SKIP
	move.l	EXCNT,d7
b\@:	dc.w	$F080+\1*$200,$0004
	CPCHK	T_CPID+\1*32+4,b\@
	move.l	#4,SKIP
	move.l	EXCNT,d7
s\@:	dc.w	$F041+\1*$200,$0000
	CPCHK	T_CPID+\1*32+8,s\@
	move.l	#6,SKIP
	move.l	EXCNT,d7
d\@:	dc.w	$F04B+\1*$200,$0000,$0000
	CPCHK	T_CPID+\1*32+12,d\@
	move.l	#6,SKIP
	move.l	EXCNT,d7
t\@:	dc.w	$F07A+\1*$200,$0000,$1234
	CPCHK	T_CPID+\1*32+16,t\@
	move.l	#2,SKIP
	move.l	EXCNT,d7
v\@:	dc.w	$F110+\1*$200
	CPCHK	T_CPID+\1*32+20,v\@
	move.l	#2,SKIP
	move.l	EXCNT,d7
r\@:	dc.w	$F150+\1*$200
	CPCHK	T_CPID+\1*32+24,r\@
	endm
