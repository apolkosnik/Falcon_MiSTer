/* Glue around the unmodified Hatari src/blitter.c (golden model).
 * Provides the Hatari globals/functions blitter.c needs and a small API for
 * the C++ testbench: register writes/reads through the same IoMem handler
 * dispatch as Hatari's ioMem.c (IoMem_wput/IoMem_bput/IoMem_wget) and the
 * blitter pass (Blitter_InterruptHandler) whenever Hatari scheduled one. */
#include <string.h>
#include "hatari_stubs.h"
#include "blitter.h"
#include "golden.h"

CNF_PARAMS ConfigureParams;
struct uae_prefs currprefs;
bool CpuRunCycleExact = false;
unsigned long currcycle;
int WaitStateCycles;
int nCyclesMainCounter;
uint64_t CyclesGlobalClockCounter;
int CurrentInstrCycles = 8;
bool cpu_bus_rmw;
int BusMode;
bool bDspEnabled = false;
void *pMFP_Main;
uint8_t gm_iomem[0x10000];
int nIoMemAccessSize;
uint32_t IoAccessCurrentAddress;

int gm_int_pending;
int gm_gpu_line;            /* MFP_GPIP_LINE_GPU_DONE: 1 = busy */
int gm_pass_accesses;

void CycInt_Process(void) { }
void CycInt_AddRelativeInterrupt(int c, int t, int h) { (void)c; (void)t; (void)h; gm_int_pending = 1; }
void CycInt_RemovePendingInterrupt(int h) { (void)h; gm_int_pending = 0; }
void CycInt_AcknowledgeInterrupt(void) { gm_int_pending = 0; }
void DSP_Run(int n) { (void)n; }
void MegaSTE_Cache_Flush(void) { }
void M68000_AddCycles_CE(int c) { (void)c; }
void M68000_SetBlitter_CE(bool on) { (void)on; }
uint32_t M68000_GetPC(void) { return 0; }
void Video_GetPosition(int *a, int *b, int *c) { *a = 0; *b = 0; *c = 0; }
void MemorySnapShot_Store(void *p, int s) { (void)p; (void)s; }
void MFP_GPIP_Set_Line_Input(void *m, uint8_t l, uint8_t b) { (void)m; if (l == MFP_GPIP_LINE_GPU_DONE) gm_gpu_line = b; }

uint16_t STMemory_DMA_ReadWord(uint32_t addr) { gm_pass_accesses++; return gm_mem_read(addr); }
void STMemory_DMA_WriteWord(uint32_t addr, uint16_t v) { gm_pass_accesses++; gm_mem_write(addr, v); }

typedef void (*hfn)(void);

static hfn rd_tab[0x3e];
static hfn wr_tab[0x3e];

static void nop_fn(void) { }

static void set2(hfn *t, int a, hfn f) { t[a] = f; t[a + 1] = f; }
static void set4(hfn *t, int a, hfn f) { set2(t, a, f); set2(t, a + 2, f); }

void gm_init(void)
{
	int i;
	static const hfn htr[16] = {
		Blitter_Halftone00_ReadWord, Blitter_Halftone01_ReadWord, Blitter_Halftone02_ReadWord, Blitter_Halftone03_ReadWord,
		Blitter_Halftone04_ReadWord, Blitter_Halftone05_ReadWord, Blitter_Halftone06_ReadWord, Blitter_Halftone07_ReadWord,
		Blitter_Halftone08_ReadWord, Blitter_Halftone09_ReadWord, Blitter_Halftone10_ReadWord, Blitter_Halftone11_ReadWord,
		Blitter_Halftone12_ReadWord, Blitter_Halftone13_ReadWord, Blitter_Halftone14_ReadWord, Blitter_Halftone15_ReadWord };
	static const hfn htw[16] = {
		Blitter_Halftone00_WriteWord, Blitter_Halftone01_WriteWord, Blitter_Halftone02_WriteWord, Blitter_Halftone03_WriteWord,
		Blitter_Halftone04_WriteWord, Blitter_Halftone05_WriteWord, Blitter_Halftone06_WriteWord, Blitter_Halftone07_WriteWord,
		Blitter_Halftone08_WriteWord, Blitter_Halftone09_WriteWord, Blitter_Halftone10_WriteWord, Blitter_Halftone11_WriteWord,
		Blitter_Halftone12_WriteWord, Blitter_Halftone13_WriteWord, Blitter_Halftone14_WriteWord, Blitter_Halftone15_WriteWord };

	ConfigureParams.System.nMachineType = MACHINE_FALCON;
	ConfigureParams.System.bAddressSpace24 = true;
	currprefs.cpu_model = 68030;
	CpuRunCycleExact = false;

	for (i = 0; i < 0x3e; i++) { rd_tab[i] = nop_fn; wr_tab[i] = nop_fn; }
	for (i = 0; i < 16; i++) { set2(rd_tab, 2 * i, htr[i]); set2(wr_tab, 2 * i, htw[i]); }
	set2(rd_tab, 0x20, Blitter_SourceXInc_ReadWord);      set2(wr_tab, 0x20, Blitter_SourceXInc_WriteWord);
	set2(rd_tab, 0x22, Blitter_SourceYInc_ReadWord);      set2(wr_tab, 0x22, Blitter_SourceYInc_WriteWord);
	set4(rd_tab, 0x24, Blitter_SourceAddr_ReadLong);      set4(wr_tab, 0x24, Blitter_SourceAddr_WriteLong);
	set2(rd_tab, 0x28, Blitter_Endmask1_ReadWord);        set2(wr_tab, 0x28, Blitter_Endmask1_WriteWord);
	set2(rd_tab, 0x2a, Blitter_Endmask2_ReadWord);        set2(wr_tab, 0x2a, Blitter_Endmask2_WriteWord);
	set2(rd_tab, 0x2c, Blitter_Endmask3_ReadWord);        set2(wr_tab, 0x2c, Blitter_Endmask3_WriteWord);
	set2(rd_tab, 0x2e, Blitter_DestXInc_ReadWord);        set2(wr_tab, 0x2e, Blitter_DestXInc_WriteWord);
	set2(rd_tab, 0x30, Blitter_DestYInc_ReadWord);        set2(wr_tab, 0x30, Blitter_DestYInc_WriteWord);
	set4(rd_tab, 0x32, Blitter_DestAddr_ReadLong);        set4(wr_tab, 0x32, Blitter_DestAddr_WriteLong);
	set2(rd_tab, 0x36, Blitter_WordsPerLine_ReadWord);    set2(wr_tab, 0x36, Blitter_WordsPerLine_WriteWord);
	set2(rd_tab, 0x38, Blitter_LinesPerBitblock_ReadWord); set2(wr_tab, 0x38, Blitter_LinesPerBitblock_WriteWord);
	rd_tab[0x3a] = Blitter_HalftoneOp_ReadByte;  wr_tab[0x3a] = Blitter_HalftoneOp_WriteByte;
	rd_tab[0x3b] = Blitter_LogOp_ReadByte;       wr_tab[0x3b] = Blitter_LogOp_WriteByte;
	rd_tab[0x3c] = Blitter_Control_ReadByte;     wr_tab[0x3c] = Blitter_Control_WriteByte;
	rd_tab[0x3d] = Blitter_Skew_ReadByte;        wr_tab[0x3d] = Blitter_Skew_WriteByte;

	memset(gm_iomem, 0, sizeof(gm_iomem));
	gm_int_pending = 0;
	gm_gpu_line = 0;
}

void gm_reset(void)
{
	Blitter_Reset();
	gm_int_pending = 0;
	gm_gpu_line = 0;
}

/* off = byte offset from $FF8A00; size 1 or 2 (as IoMem_bput / IoMem_wput) */
void gm_write(int off, int size, uint16_t val)
{
	uint32_t addr = 0xff8a00 + off;
	if (size == 1) {
		nIoMemAccessSize = SIZE_BYTE;
		IoMem_WriteByte(addr, (uint8_t)val);
		IoAccessCurrentAddress = addr;
		wr_tab[off]();
	} else {
		nIoMemAccessSize = SIZE_WORD;
		IoMem_WriteWord(addr, val);
		IoAccessCurrentAddress = addr;
		wr_tab[off]();
		if (wr_tab[off + 1] != wr_tab[off]) {
			IoAccessCurrentAddress = addr + 1;
			wr_tab[off + 1]();
		}
	}
}

/* word read as IoMem_wget */
uint16_t gm_read(int off)
{
	uint32_t addr = 0xff8a00 + off;
	nIoMemAccessSize = SIZE_WORD;
	IoAccessCurrentAddress = addr;
	rd_tab[off]();
	if (rd_tab[off + 1] != rd_tab[off]) {
		IoAccessCurrentAddress = addr + 1;
		rd_tab[off + 1]();
	}
	return IoMem_ReadWord(addr);
}

/* run one blitter pass if Hatari scheduled one; returns the number of bus
 * accesses of the pass or -1 if nothing was pending */
int gm_run_pass(void)
{
	if (!gm_int_pending)
		return -1;
	gm_pass_accesses = 0;
	Blitter_InterruptHandler();
	return gm_pass_accesses;
}
