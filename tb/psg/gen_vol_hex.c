/*
 * Generates rtl/falcon/falcon_psg_vol.hex: the 32*32*32 YM2149 mixing table
 * exactly as Hatari's sound.c builds it for the Falcon (YM_TABLE_MIXING:
 * interpolate_volumetable() over ym2149_fixed_vol.h, then
 * YM2149_Normalise_5bit_Table(level 0x7fff, not centered)).
 * Index = C<<10 | B<<5 | A (YM_MERGE_VOICE), one 15-bit value per line.
 */
#include <stdio.h>
void golden_init(void);
int golden_ymout5(int idx);
int main(int argc, char **argv)
{
	FILE *f = fopen(argv[1], "w");
	if (!f) return 1;
	golden_init();
	for (int i = 0; i < 32768; i++) {
		int v = golden_ymout5(i);
		if (v < 0 || v > 0x7fff) { fprintf(stderr, "value out of range\n"); return 1; }
		fprintf(f, "%04x\n", v);
	}
	fclose(f);
	return 0;
}
