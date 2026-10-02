/*
 * Golden model for the falcon_psg testbench.
 * This file compiles Hatari's REAL src/sound.c (included verbatim below) so
 * the testbench compares the RTL against Hatari's own YM2149 code:
 *   Ym2149_Init / Ym2149_Reset, Sound_WriteReg, YM2149_DoSamples_250,
 *   YM2149_RndCompute, YmEnvWaves (YM2149_EnvBuild), ymout5 (interpolated
 *   YM_TABLE_MIXING table, normalised to 0..0x7fff), PWMaliasFilter.
 * Only the host side symbols sound.c references are stubbed (audio output,
 * recording, logging); none of them takes part in the YM computation.
 */
#include "sound.c"

/* ---- stubs for symbols outside the YM core ---- */
CNF_PARAMS ConfigureParams;
CLOCKS_STRUCT MachineClocks;
uint64_t CyclesGlobalClockCounter;
int nScreenRefreshRate = 50;
int SoundBufferSize = 1024;
bool bSoundWorking = false;
bool bRecordingWav = false;
bool bRecordingYM = false;
void Audio_Init(void) {}
void Audio_UnInit(void) {}
void Audio_Lock(void) {}
void Audio_Unlock(void) {}
bool Avi_AreWeRecording(void) { return false; }
bool Avi_RecordAudioStream(int16_t pSamples[][2], int SampleIndex, int SampleLength) { return true; }
void ClocksTimings_ConvertCycles(uint64_t a, uint64_t b, CLOCKS_CYCLES_STRUCT *c, uint64_t d) { c->Cycles = 0; c->Remainder = 0; }
uint32_t ClocksTimings_GetVBLPerSec(MACHINETYPE m, int r) { return 50; }
void Crossbar_Compute_Ratio(void) {}
void Crossbar_GenerateSamples(int a, int b) {}
void DmaSnd_GenerateSamples(int a, int b) {}
void DmaSnd_Init_Bass_and_Treble_Tables(void) {}
bool File_DoesFileExtensionMatch(const char *a, const char *b) { return false; }
void Log_AlertDlg(LOGTYPE t, const char *f, ...) {}
void Log_Printf(LOGTYPE t, const char *f, ...) {}
void MemorySnapShot_Store(void *p, int n) {}
bool WAVFormat_OpenFile(char *f) { return false; }
void WAVFormat_CloseFile(void) {}
void WAVFormat_Update(int16_t pSamples[][2], int Index, int Length) {}
bool YMFormat_BeginRecording(const char *f) { return false; }
void YMFormat_EndRecording(void) {}

/* ---- testbench interface ---- */
void golden_init(void)
{
	ConfigureParams.System.nMachineType = MACHINE_FALCON;
	YmVolumeMixing = YM_TABLE_MIXING;		/* configuration.c default */
	YM2149_LPF_Filter = YM2149_LPF_FILTER_PWM;	/* sound.c default */
	Ym2149_Init();
}

/* Ym2149_Reset plus the free running /2 noise prescaler, which Hatari never
 * clears but the RTL clears on reset */
void golden_reset(void) { Ym2149_Reset(); YM2149_Freq_div_2 = 0; }

/* Switch the mixing table (YM_TABLE_MIXING or YM_LINEAR_MIXING) */
void golden_set_mixing(int linear)
{
	YmVolumeMixing = linear ? YM_LINEAR_MIXING : YM_TABLE_MIXING;
	Ym2149_BuildVolumeTable();
}

void golden_write(int reg, int val) { Sound_WriteReg(reg, (uint8_t)val); }

/* One 250 kHz step; returns the sample stored in YM_Buffer_250 */
int golden_tick(void)
{
	int pos = YM_Buffer_250_pos_write;
	YM2149_DoSamples_250(1);
	return YM_Buffer_250[pos];
}

/* Channel output levels (5 bit) after the step, recomputed from Hatari's
 * state with the same expressions as YM2149_DoSamples_250 */
int golden_levels(void)
{
	ymu32 bt;
	ymu16 Env3Voices = YmEnvWaves[Env_shape][Env_pos] & EnvMask3Voices;
	ymu16 Tone3Voices;
	bt = (ToneA_val | mixerTA) & (Noise_val | mixerNA);
	Tone3Voices = bt & YM_MASK_1VOICE;
	bt = (ToneB_val | mixerTB) & (Noise_val | mixerNB);
	Tone3Voices |= (bt & YM_MASK_1VOICE) << 5;
	bt = (ToneC_val | mixerTC) & (Noise_val | mixerNC);
	Tone3Voices |= (bt & YM_MASK_1VOICE) << 10;
	Tone3Voices &= (Env3Voices | Vol3Voices);
	return Tone3Voices;
}

int golden_ymout5(int idx) { return ymout5[idx & 0x7fff]; }
int golden_envwave(int shape, int pos) { return YmEnvWaves[shape & 15][pos] & 0x1f; }
int golden_vol4to5(int v) { return YmVolume4to5[v & 15]; }
int golden_ymout1c5bit(int v) { return ymout1c5bit[v & 31]; }
unsigned golden_rndrack(void) { return RndRack; }
int golden_noise_val(void) { return Noise_val; }
int golden_rnd_step(void) { return YM2149_RndCompute(); }
void golden_set_rndrack(unsigned v) { RndRack = v; }
