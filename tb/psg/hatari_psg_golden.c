/*
 * Golden model of the YM2149 register interface for the falcon_psg
 * testbench: Hatari's REAL src/psg.c is compiled (included verbatim) and
 * PSG_Reset / PSG_Set_SelectRegister / PSG_Get_DataRegister /
 * PSG_Set_DataRegister are called directly.  The functions psg.c calls in
 * other Hatari modules (FDC, MFP, printer, status bar, DSP, joystick, CPU
 * wait states, tracing) are stubbed: they do not influence the register
 * values.  Sound_WriteReg / Sound_Update come from hatari_ym_golden.c
 * (Hatari's sound.c), so register writes reach the sound golden model.
 */
#include "psg.c"

uae_u8 IoMemBuf[0x10000];
uae_u8 *IOmemory = IoMemBuf;
uint32_t IoAccessCurrentAddress;
int nIoMemAccessSize;
int CurrentInstrCycles;
int OpcodeFamily;
uint64_t LogTraceFlags = 0;
struct regstruct regs;
MFP_STRUCT *pMFP_Main;
uint64_t Cycles_GetClockCounterOnWriteAccess(void) { return 0; }
void DSP_Reset(void) {}
void FDC_SetDriveSide(uint8_t a, uint8_t b) {}
uint8_t Joy_GetStickData(int id) { return 0; }
void Log_Trace(const char *f, ...) {}
void M68000_WaitState(int n) {}
void MFP_GPIP_Set_Line_Input(MFP_STRUCT *p, uint8_t l, uint8_t b) {}
bool Printer_TransferByteTo(uint8_t b) { return true; }
void SCC_Check_Lan_IsEnabled(void) {}
void Statusbar_SetFloppyLed(drive_index_t d, drive_led_t s) {}
void Video_GetPosition(int *a, int *b, int *c) { *a = *b = *c = 0; }

void golden_psg_reset(void) { PSG_Reset(); }
void golden_psg_select(int v) { PSG_Set_SelectRegister((uint8_t)v); }
int golden_psg_read(void) { return PSG_Get_DataRegister(); }
void golden_psg_data(int v) { PSG_Set_DataRegister((uint8_t)v); }
int golden_psg_reg(int r) { return PSGRegisters[r & 15]; }
