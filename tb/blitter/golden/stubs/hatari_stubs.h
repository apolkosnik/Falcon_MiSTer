/* Minimal stand-ins for the Hatari headers included by src/blitter.c, so
 * that the unmodified Hatari blitter.c can be compiled as the golden model
 * of the Falcon_MiSTer blitter testbench.  Plain ASCII only. */
#ifndef HATARI_STUBS_H
#define HATARI_STUBS_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define SIZE_BYTE 1
#define SIZE_WORD 2
#define SIZE_LONG 4

#define LOG_TRACE_LEVEL(x) 0
#define LOG_TRACE_PRINT(...) do { } while (0)
#define TRACE_BLITTER 0

#define MACHINE_MEGA_STE 1
#define MACHINE_FALCON   2
typedef struct { int nMachineType; bool bAddressSpace24; } CNF_SYSTEM;
typedef struct { CNF_SYSTEM System; } CNF_PARAMS;
extern CNF_PARAMS ConfigureParams;

struct uae_prefs { int cpu_model; };
extern struct uae_prefs currprefs;
extern bool CpuRunCycleExact;
extern unsigned long currcycle;
#define CYCLE_UNIT 512

extern int WaitStateCycles;
extern int nCyclesMainCounter;
extern uint64_t CyclesGlobalClockCounter;
extern int CurrentInstrCycles;
extern bool cpu_bus_rmw;

#define BUS_MODE_CPU     0
#define BUS_MODE_BLITTER 1
extern int BusMode;

#define INT_CPU_CYCLE 1
#define INTERRUPT_BLITTER 1
void CycInt_Process(void);
void CycInt_AddRelativeInterrupt(int CycleTime, int CycleType, int Handler);
void CycInt_RemovePendingInterrupt(int Handler);
void CycInt_AcknowledgeInterrupt(void);

extern bool bDspEnabled;
void DSP_Run(int nHostCycles);

void MegaSTE_Cache_Flush(void);
void M68000_AddCycles_CE(int cycles);
void M68000_SetBlitter_CE(bool on);
uint32_t M68000_GetPC(void);
void Video_GetPosition(int *pFrameCycles, int *pHBL, int *pLineCycles);
void MemorySnapShot_Store(void *pData, int Size);

#define MFP_GPIP_LINE_GPU_DONE 3
#define MFP_GPIP_STATE_LOW  0
#define MFP_GPIP_STATE_HIGH 1
extern void *pMFP_Main;
void MFP_GPIP_Set_Line_Input(void *pMFP, uint8_t LineNr, uint8_t Bit);

uint16_t STMemory_DMA_ReadWord(uint32_t addr);
void STMemory_DMA_WriteWord(uint32_t addr, uint16_t value);

/* IO memory: big endian byte array indexed by the low 16 address bits */
extern uint8_t gm_iomem[0x10000];
extern int nIoMemAccessSize;
extern uint32_t IoAccessCurrentAddress;
static inline uint8_t IoMem_ReadByte(uint32_t a) { return gm_iomem[a & 0xffff]; }
static inline uint16_t IoMem_ReadWord(uint32_t a) { return (uint16_t)((gm_iomem[a & 0xffff] << 8) | gm_iomem[(a + 1) & 0xffff]); }
static inline uint32_t IoMem_ReadLong(uint32_t a) { return ((uint32_t)IoMem_ReadWord(a) << 16) | IoMem_ReadWord(a + 2); }
static inline void IoMem_WriteByte(uint32_t a, uint8_t v) { gm_iomem[a & 0xffff] = v; }
static inline void IoMem_WriteWord(uint32_t a, uint16_t v) { gm_iomem[a & 0xffff] = v >> 8; gm_iomem[(a + 1) & 0xffff] = v & 0xff; }
static inline void IoMem_WriteLong(uint32_t a, uint32_t v) { IoMem_WriteWord(a, v >> 16); IoMem_WriteWord(a + 2, v & 0xffff); }

#endif
