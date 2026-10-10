/* Dump Hatari's 68030 opcode data table (cpudatatbl: length in bytes,
   brief/full extension positions disp020, branch flag) for all 65536
   opcodes, built as newcpu.c build_cpufunctbl does for the cycle-exact
   68030 (op_smalltbl_23, table68k handlers, lvl 3).  Output: one line per
   opcode "length d0 d1 branch".  gen.sh turns it into falcon_optbl.mem. */
#include "sysconfig.h"
#include "sysdeps.h"
#include "readcpu.h"
#include <stdio.h>
#include "smalltbl23.h"
struct d { int length, d0, d1, branch, ok; };
static struct d tbl[65536];
int main(void)
{
	int i, lvl = 3;
	init_table68k();
	for (i = 0; i < (int)(sizeof st23 / sizeof st23[0]); i++) {
		struct d *t = &tbl[st23[i].opcode];
		t->length = st23[i].length; t->d0 = st23[i].d0; t->d1 = st23[i].d1;
		t->branch = st23[i].branch; t->ok = 1;
	}
	for (i = 0; i < 65536; i++) {
		struct instr *t = &table68k[i];
		if (t->mnemo == i_ILLG) continue;
		if (t->unimpclev > 0 && lvl >= t->unimpclev) { tbl[i].ok = 0; continue; }
		if (t->clev > lvl) continue;
		if (t->handler != -1) tbl[i] = tbl[t->handler];
	}
	for (i = 0; i < 65536; i++)
		printf("%d %d %d %d\n", tbl[i].ok ? tbl[i].length : 0, tbl[i].d0, tbl[i].d1, tbl[i].branch);
	return 0;
}
