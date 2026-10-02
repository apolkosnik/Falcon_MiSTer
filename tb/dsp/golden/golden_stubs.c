/* Minimal stubs so Hatari's dsp_cpu.c and dsp_core.c (the golden model)
   link outside of Hatari. Plain ASCII. */
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

FILE *TraceFile = NULL;
uint64_t LogTraceFlags = 0;
int ExceptionDebugMask = 0;

void Log_Printf(int nType, const char *psFormat, ...) { (void)nType; (void)psFormat; }
void Log_Trace(const char *format, ...) { (void)format; }
void Log_ResetMsgRepeat(void) { }
void DebugUI(int reason) { (void)reason; }

/* disassembler hooks (never used: tracing is off) */
void dsp56k_disasm_init(void) { }
uint16_t dsp56k_disasm(int mode, FILE *fp) { (void)mode; (void)fp; return 1; }
const char *dsp56k_getInstructionText(void) { return ""; }
void dsp56k_disasm_reg_save(void) { }
void dsp56k_disasm_reg_compare(FILE *fp) { (void)fp; }

/* crossbar handshake hooks: counted by the testbench */
int golden_sc1_calls = 0;
int golden_sc2_calls = 0;
uint32_t golden_sc2_last = 0;
/* Hatari crossbar.c dmaRecord.handshakeMode_Frame: set by
   Crossbar_DmaRecordInHandShakeMode_Frame (= DSP_SsiTransmit_SC2), cleared by
   the testbench when a transmit slot takes the word */
int golden_hs_record_frame = 0;
/* DSP_SsiTransmit_SC1 -> Crossbar_DmaPlayInHandShakeMode */
void DSP_SsiTransmit_SC1(void) { golden_sc1_calls++; }
void DSP_SsiTransmit_SC2(uint32_t frame) { golden_sc2_calls++; golden_sc2_last = frame; golden_hs_record_frame = frame ? 1 : 0; }
