; tags.i - result tags shared by the 68k programs and the C++ bench.
; The bench (tb_fpu.cpp) parses the "NAME equ VALUE" lines of this file, so
; both sides always use the same numbers.  Plain ASCII only.

; t_detect
T_COOKIE	equ	$0001	; _FPU cookie value returned by the EmuTOS _detect_fpu replica

; t_frames
T_FS_NULL_DELTA	equ	$0101	; FSAVE -(a0) after reset: bytes pushed
T_FS_NULL_FMT	equ	$0102	; format word stored
T_FS_NULL_BEYOND equ	$0103	; long just above the frame (must stay untouched)
T_FS_IDLE_DELTA	equ	$0104	; FSAVE -(a0) after FNOP: bytes pushed
T_FS_IDLE_FMT	equ	$0105	; format word stored
T_FS_IDLE_LEN	equ	$0106	; byte at frame offset 1 (what EmuTOS tests)
T_FS_IDLE_BEYOND equ	$0107
T_FS_CTRL_FMT	equ	$0108	; FSAVE (a0) (control mode) after FNOP: format word
T_FS_CTRL_A0	equ	$0109	; a0 unchanged by FSAVE (a0): 1
T_FS_CTRL_BEYOND equ	$010A	; long above the 60 bytes written upward
T_FR_NULL_FMT	equ	$010B	; FNOP; FRESTORE null; FSAVE -> format word
T_FR_NULL_DELTA	equ	$010C	; ... and its size
T_FR_NULL_A1	equ	$010D	; FRESTORE (a1) leaves a1 alone: 1
T_FR_NULL_ADV	equ	$010E	; FRESTORE (a1)+ of a null frame: bytes consumed
T_FR_NZ_CNT	equ	$010F	; FRESTORE of $0012 (null, version 0): exceptions taken
T_FR_NZ_FMT	equ	$0110	; ... FSAVE afterwards: format word
T_FR_IDLE_ADV	equ	$0111	; FRESTORE (a1)+ of an idle frame: bytes consumed
T_FR_IDLE_CNT	equ	$0112	; ... exceptions taken
T_FR_IDLE_FMT	equ	$0113	; ... FSAVE afterwards (from null): format word
T_FR_IDLE_DELTA	equ	$0114
T_BADF		equ	$0120	; + 8*i + {0 exceptions, 1 vector, 2 frame format, 3 stacked PC ok, 4 continued}
T_FR_AFTERBAD	equ	$0150	; FNOP after the format errors completes: exceptions since

; t_cond  (+ predicate 0..31)
T_FSCC		equ	$0200
T_FBCC		equ	$0240
T_FDB_TAKEN	equ	$0280
T_FDB_COUNT	equ	$02C0
T_FTRAP		equ	$0300
T_FNOP_CNT	equ	$0340	; exceptions taken by 3 FNOPs
T_FSCC_MEM_T	equ	$0350
T_FSCC_MEM_F	equ	$0351
T_FBCC_L_T	equ	$0352
T_FBCC_L_F	equ	$0353
T_FTW_T		equ	$0354	; FTRAPT.W #imm: exceptions
T_FTW_F		equ	$0355	; FTRAPF.W #imm: exceptions
T_FTL_T		equ	$0356
T_FTL_F		equ	$0357
T_FTW_FMT	equ	$0358	; frame format of the TRAPcc exception (2)
T_FTW_VEC	equ	$0359
T_FTW_PC	equ	$035A	; stacked PC = address after the operand: 1
T_FTW_IA	equ	$035B	; instruction address in the frame = the instruction: 1
T_FTL_PC	equ	$035C
T_FNOP_DONE	equ	$035D	; code after FNOP ran: 1

; t_gen  (+ 4*case + {0 vector, 1 frame format, 2 stacked PC ok, 3 continued})
T_GEN		equ	$0400
T_GEN_COUNT	equ	$04F0

; t_cpid / t_absent  (+ 32*CpID + 4*kind + {0 vector, 1 format, 2 PC ok, 3 exceptions})
T_CPID		equ	$0500

; t_reset
T_RS_BEFORE	equ	$0600	; FSAVE after FNOP: format word
T_RS_AFTER	equ	$0601	; FSAVE after the RESET instruction: format word
T_RS_FNOP	equ	$0602	; FNOP after RESET: exceptions taken
T_RS_SIZE	equ	$0603	; bytes pushed by the FSAVE after RESET

; t_raw  (raw CIR reads/writes through MOVES with DFC = 7)
T_RAW		equ	$0700

; t_frames: idle frame body (14 longs after the format long) OR-ed together
T_FS_IDLE_BODY	equ	$0170	; + n, n = 0..13: the 14 body longs
T_ODDF		equ	$0160	; + 4*i + {0 exceptions, 1 bytes consumed}

; t_raw / t_rawna  (raw CIR reads/writes through MOVES with DFC = SFC = 7)
T_RAW_RESP0	equ	$0700	; response CIR after reset (read)
T_RAW_SAVE	equ	$0701	; save CIR in the null state: format word
T_RAW_TF1	equ	$0702	; response after a condition word $000F (T): high byte
T_RAW_TF1B	equ	$0703	; ... TF bit
T_RAW_TF0	equ	$0704	; response after a condition word $0000 (F): TF bit
T_RAW_TF0H	equ	$0705	; ... high byte
T_RAW_CMD	equ	$0706	; response after a command word $0000: primitive (take exception, vector 11)
T_RAW_CMD2	equ	$0707	; the response CIR read once more: a taken-exception primitive is not repeated
T_RAW_DONE	equ	$07FF

; t_bg / t_busy / t_wd (milestone 2 hand written programs)
T_BG_FP0	equ	$0800
T_BUSY_FMT	equ	$0810
T_BUSY_DELTA	equ	$0811
T_WD_AFTER	equ	$0820

; t_irq / t_fsdie (milestone 4)
T_IRQ_DELTA	equ	$0830
T_IRQ_FMT	equ	$0831
T_IRQ_CNT	equ	$0832
T_FSD_DELTA	equ	$0840	; FSAVE whose service died: bytes pushed
T_FSD_FMT	equ	$0841	; ... format word (null, $0000)
T_FSD_DELTA2	equ	$0842	; FNOP; FSAVE after the service is back: bytes pushed
T_FSD_FMT2	equ	$0843	; ... format word
