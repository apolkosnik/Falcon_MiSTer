// tb_lpt.cpp - falcon_lpt against the real falcon_lpt program (see run.sh).
#include "Vfalcon_lpt.h"
#include "verilated.h"
#include <dirent.h>
#include <fcntl.h>
#include <signal.h>
#include <algorithm>
#include <string>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

static Vfalcon_lpt *d;
static uint8_t *mem;                 // guest $EB8000..$EB9FFF
static const unsigned BASE = 0xEB8000, SIZE = 0x2000;
static int fails, checks;
static long clocks;
static int ack_in;                   // the memory answers two clocks after the request

static void check(bool ok, const char *what)
{
	checks++;
	if (!ok) { fails++; printf("FAIL: %s\n", what); }
	else printf("ok: %s\n", what);
}

static double wall(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec / 1e9;
}
static double t0;

static void tick(void)
{
	d->m_ack = 0;
	if (d->m_req && !ack_in) ack_in = 3;
	if (ack_in && --ack_in == 0 && d->m_req) {
		unsigned a = (unsigned)d->m_addr << 1;
		if (a < BASE || a >= BASE + SIZE) { printf("FAIL: access outside the ring's memory: %06x\n", a); fails++; }
		else if (d->m_we) { mem[a - BASE] = d->m_wdata >> 8; mem[a - BASE + 1] = d->m_wdata; }
		else d->m_rdata = mem[a - BASE] << 8 | mem[a - BASE + 1];
		d->m_ack = 1;
	}
	d->clk = 1; d->eval();
	d->clk = 0; d->eval();
	// keep to real time: the program's heartbeat is on the wall clock
	if (++clocks % 100 == 0) {
		double ahead = (double)clocks / TB_CLK_HZ - (wall() - t0);
		if (ahead > 0) usleep((useconds_t)(ahead * 1e6));
	}
}
static void run(double seconds) { for (long n = (long)(seconds * TB_CLK_HZ); n > 0; n--) tick(); }

// one byte as TOS sends it: data, /STROBE low, high
static void strobe(uint8_t b)
{
	d->data = b; tick();
	d->strobe_n = 0; tick(); tick();
	d->strobe_n = 1; tick();
}
// wait for BUSY low, at most `seconds`
static bool ready(double seconds)
{
	for (long n = (long)(seconds * TB_CLK_HZ); n > 0 && d->busy; n--) tick();
	return !d->busy;
}

static std::string outdir;
static std::vector<uint8_t> printed(void)      // everything in the job files, in name order
{
	std::vector<std::string> names;
	DIR *dir = opendir(outdir.c_str());
	struct dirent *e;
	while (dir && (e = readdir(dir))) if (strstr(e->d_name, ".prn")) names.push_back(e->d_name);
	if (dir) closedir(dir);
	std::sort(names.begin(), names.end());
	std::vector<uint8_t> v;
	for (auto &n : names) {
		FILE *f = fopen((outdir + "/" + n).c_str(), "rb");
		int c;
		while (f && (c = fgetc(f)) != EOF) v.push_back(c);
		if (f) fclose(f);
	}
	return v;
}
static unsigned rd16(unsigned o) { return mem[o] << 8 | mem[o + 1]; }

static pid_t start(const char *tool, const char *memfile)
{
	pid_t p = fork();
	if (p == 0) {
		execl(tool, tool, "-m", memfile, "-o", outdir.c_str(), "-U", "-t", "0.5", (char *)0);
		_exit(127);
	}
	return p;
}

int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	if (argc < 3) { fprintf(stderr, "usage: tb_lpt TOOL WORKDIR\n"); return 2; }
	std::string work = argv[2], memfile = work + "/lpt.mem";
	outdir = work + "/out";
	if (system(("rm -rf " + outdir).c_str())) return 2;
	int fd = open(memfile.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0666);
	if (fd < 0 || ftruncate(fd, SIZE)) { perror("memory file"); return 2; }
	mem = (uint8_t *)mmap(NULL, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	// memory as another core left it: no MAGIC, counts that mean nothing
	for (unsigned k = 0; k < SIZE; k++) mem[k] = 0xA5 ^ k;

	d = new Vfalcon_lpt;
	d->strobe_n = 1; d->strobe_drv = 0; d->data = 0xff; d->por = 1; d->reset = 1;
	t0 = wall();
	for (int k = 0; k < 8; k++) tick();
	d->por = 0; d->reset = 0;
	std::vector<uint8_t> sent;
	uint8_t x = 1;

	// 1. nobody on the HPS: a printer that is off, and bytes go nowhere
	run(1.5);
	check(d->busy, "no program: BUSY stays high");
	uint8_t before[16]; memcpy(before, mem, 16);
	for (int k = 0; k < 20; k++) strobe(k);
	run(0.2);
	check(!memcmp(before, mem, 16) && d->busy, "no program: strobed bytes are dropped, the memory is left alone");

	// 2. the program starts: BUSY falls, and a job arrives whole
	pid_t pid = start(argv[1], memfile.c_str());
	check(ready(1.0), "program started: BUSY falls within a second");
	// TOS on its way up: port A becomes an output, high as the PSG's reset left
	// it, then $07 is written to it (/STROBE low), later $27
	d->data = 0; d->strobe_drv = 1; run(0.01);
	d->strobe_n = 0; run(0.05);
	d->strobe_n = 1; run(0.05);
	check(rd16(4) == rd16(6) && !d->busy, "the first low after a reset is not a byte");
	// and again after a machine reset, with the program already there
	d->reset = 1; d->strobe_drv = 0; run(0.01); d->reset = 0;
	d->strobe_drv = 1; run(0.01);
	d->strobe_n = 0; run(0.05);
	d->strobe_n = 1; run(0.05);
	check(rd16(4) == rd16(6) && !d->busy, "nor after a later machine reset");
	unsigned head0 = rd16(4);
	bool ok = true;
	for (int k = 0; k < 5000 && ok; k++) {
		ok = ready(2.0);
		x = x * 5 + 3;
		strobe(x); sent.push_back(x);
	}
	check(ok, "5000 bytes strobed, BUSY honoured (the ring wraps twice)");
	run(1.2);                                    // the job's idle time
	check(printed() == sent, "the job file holds the 5000 bytes");

	// 3. a strobe while BUSY is high for a byte in flight is not taken twice or lost later
	ready(1.0);
	strobe(0x41); strobe(0x42);                  // the second arrives before the first is stored
	run(0.05);
	ready(1.0); strobe(0x43);
	sent.push_back(0x41); sent.push_back(0x43);
	run(1.2);
	check(printed() == sent, "a byte strobed against BUSY is dropped, its neighbours arrive");

	// 4. the program stops taking bytes: the ring fills and BUSY holds the machine
	kill(pid, SIGSTOP);
	int n = 0;
	double tstop = wall();
	while (n < 3000 && ready(0.002)) { x = x * 5 + 3; strobe(x); sent.push_back(x); n++; }
	printf("   ring took %d bytes in %.2f s\n", n, wall() - tstop);
	check(n == 2040 && d->busy, "program stopped: 2040 bytes, then BUSY");
	run(1.3);
	check(d->busy, "its heartbeat stands still: BUSY stays high");
	strobe(0x99); run(0.01);
	check(((rd16(4) - head0) & 0xffff) == 5002 + 2040, "no byte is stored meanwhile");
	kill(pid, SIGCONT);
	check(ready(1.0), "program continues: BUSY falls");
	for (int k = 0; k < 300; k++) { ready(2.0); x = x * 5 + 3; strobe(x); sent.push_back(x); }
	run(1.2);
	check(printed() == sent, "nothing lost or doubled across the stop");

	// 5. a machine reset is not this module's business; a new program picks the count up
	kill(pid, SIGTERM); waitpid(pid, NULL, 0);
	run(1.3);
	check(d->busy, "program gone: BUSY high");
	pid = start(argv[1], memfile.c_str());
	check(ready(1.5), "a new program: BUSY falls");
	for (int k = 0; k < 100; k++) { ready(2.0); x = x * 5 + 3; strobe(x); sent.push_back(x); }
	run(1.2);
	check(printed() == sent, "its first job holds only the new bytes");
	kill(pid, SIGTERM); waitpid(pid, NULL, 0);

	printf("%d checks, %d failed\nRESULT: %s\n", checks, fails, fails ? "FAIL" : "PASS");
	return fails ? 1 : 0;
}
