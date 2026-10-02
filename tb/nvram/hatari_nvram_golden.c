/*
 * Golden model for the falcon_nvram testbench: Hatari's REAL
 * src/falcon/nvram.c is included verbatim.  Only its environment is
 * stubbed: the host clock (time() is redirected to golden_time(), the
 * testbench runs with TZ=UTC so localtime() is the calendar of that value),
 * the configuration, the I/O memory array and logging.
 */
#include <time.h>
#include <string.h>
#include <stdlib.h>
time_t golden_time(time_t *t);
#define time(x) golden_time(x)
#include "nvram.c"
#undef time

CNF_PARAMS ConfigureParams;
bool bUseVDIRes = false;
int VDIHeight = 400, VDIPlanes = 4;
uae_u8 IoMemBuf[0x10000];
uae_u8 *IOmemory = IoMemBuf;
uint64_t LogTraceFlags = 0;
struct regstruct regs;
void Log_Printf(LOGTYPE t, const char *f, ...) {}
void Log_Trace(const char *f, ...) {}
const char *Paths_GetHatariHome(void) { return "/nonexistent-hatari-home-for-tb"; }

static time_t g_now;
time_t golden_time(time_t *t) { if (t) *t = g_now; return g_now; }

static uint8_t pristine[64];
static int saved = 0;

void golden_nv_setup(void)
{
	setenv("TZ", "UTC", 1);
	tzset();
	if (!saved) { memcpy(pristine, nvram, 64); saved = 1; }
}

/* NvRam_Init with a fresh static image (as on a Hatari start without a
 * hatari.nvram file).  vga: monitor type, lang/kbd: TOS_LANG_* codes. */
void golden_nv_init(int vga, int lang, int kbd)
{
	memcpy(nvram, pristine, 64);
	ConfigureParams.System.nMachineType = MACHINE_FALCON;
	ConfigureParams.Screen.nMonitorType = vga ? MONITOR_TYPE_VGA : MONITOR_TYPE_RGB;
	ConfigureParams.Keyboard.nLanguage = lang;
	ConfigureParams.Keyboard.nKbdLayout = kbd;
	ConfigureParams.System.nRtcYear = 0;
	NvRam_Init();
}

void golden_nv_reset(void) { NvRam_Reset(); }
void golden_nv_set_time(long long t) { g_now = (time_t)t; }
int golden_nv_byte(int i) { return nvram[i & 63]; }

void golden_nv_select(int v) { IoMem_WriteByte(0xff8961, v); NvRam_Select_WriteByte(); }
int golden_nv_index(void) { NvRam_Select_ReadByte(); return IoMem_ReadByte(0xff8961); }
int golden_nv_read(void) { NvRam_Data_ReadByte(); return IoMem_ReadByte(0xff8963); }
void golden_nv_write(int v) { IoMem_WriteByte(0xff8963, v); NvRam_Data_WriteByte(); }
