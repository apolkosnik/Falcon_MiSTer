/*
 * mbox_ping - HPS side of the Falcon core's mailbox latency probe
 * (rtl/falcon/falcon_mbox_test.sv, in a core built with
 * FALCON_BRINGUP="MBOX_TEST").
 *
 * Maps the mailbox (DDR3 0x30E80000, 512 KB) through /dev/mem, starts a run,
 * echoes every PING into PONG as fast as it can and prints the round-trip
 * times the FPGA measured: from the FPGA's PING write until the FPGA reads
 * the echo back.
 *
 *   mbox_ping [-n samples] [-c cpu] [-f] [-o file.csv] [-m file]
 *     -n N     samples, 1..32768 (default 10000)
 *     -c CPU   pin this process to one CPU core
 *     -f       SCHED_FIFO real-time priority
 *     -o FILE  also write every sample (cycles, PONG reads) as CSV
 *     -m FILE  map FILE instead of /dev/mem, offset 0 = mailbox (simulation)
 *
 * Mailbox words are 16 bit in the Falcon's big-endian byte order.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define MB_PHYS  0x30E80000u
#define MB_SIZE  0x80000u
#define MAX_N    32768

enum { O_CMD = 0x000, O_COUNT = 0x002, O_DONE = 0x004, O_STAT = 0x006,
       O_PING = 0x008, O_PONG = 0x00A, O_MAGIC = 0x00C, O_CLKMHZ = 0x00E,
       O_SMP = 0x100 };

static volatile uint8_t *mb;

static inline void barrier(void)
{
#if defined(__arm__)
	__asm__ volatile("dsb" ::: "memory");
#else
	__sync_synchronize();
#endif
}

static inline uint16_t rd16(unsigned off)
{
	uint16_t v = *(volatile uint16_t *)(mb + off);
	return (uint16_t)((v >> 8) | (v << 8));
}

static inline void wr16(unsigned off, uint16_t v)
{
	*(volatile uint16_t *)(mb + off) = (uint16_t)((v >> 8) | (v << 8));
	barrier();
}

static double now_s(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}

static int cmp_u32(const void *a, const void *b)
{
	uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
	return x < y ? -1 : x > y;
}

static double pct(const uint32_t *s, int n, double p)
{
	int i = (int)(p * (n - 1) + 0.5);
	return s[i];
}

int main(int argc, char **argv)
{
	int n = 10000, cpu = -1, fifo = 0, opt;
	const char *csv = NULL, *map = NULL;

	while ((opt = getopt(argc, argv, "n:c:fo:m:h")) != -1) {
		switch (opt) {
		case 'n': n = atoi(optarg); break;
		case 'c': cpu = atoi(optarg); break;
		case 'f': fifo = 1; break;
		case 'o': csv = optarg; break;
		case 'm': map = optarg; break;
		default:
			fprintf(stderr, "usage: %s [-n samples] [-c cpu] [-f] [-o file.csv] [-m file]\n", argv[0]);
			return 2;
		}
	}
	if (n < 1 || n > MAX_N) {
		fprintf(stderr, "-n must be 1..%d\n", MAX_N);
		return 2;
	}

	if (cpu >= 0) {
		cpu_set_t set;
		CPU_ZERO(&set);
		CPU_SET(cpu, &set);
		if (sched_setaffinity(0, sizeof(set), &set)) {
			perror("sched_setaffinity");
			return 1;
		}
	}
	if (fifo) {
		struct sched_param sp = { .sched_priority = 50 };
		if (sched_setscheduler(0, SCHED_FIFO, &sp)) {
			perror("sched_setscheduler");
			return 1;
		}
	}

	int fd = map ? open(map, O_RDWR) : open("/dev/mem", O_RDWR | O_SYNC);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", map ? map : "/dev/mem", strerror(errno));
		return 1;
	}
	void *p = mmap(NULL, MB_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, map ? 0 : MB_PHYS);
	if (p == MAP_FAILED) {
		perror("mmap");
		return 1;
	}
	mb = p;

	uint16_t magic = rd16(O_MAGIC);
	if (magic != 0x4D42) {
		fprintf(stderr, "no mailbox probe (MAGIC = %04X, expected 4D42): "
		        "load the core built with FALCON_BRINGUP=\"MBOX_TEST\"\n", magic);
		return 1;
	}
	unsigned mhz = rd16(O_CLKMHZ);
	if (!mhz) mhz = 32;

	/* a run id the probe has not seen: one above the last finished run */
	uint16_t run = (uint16_t)(rd16(O_DONE) + 1);
	if (!run) run = 1;

	/* resynchronise: the probe numbers a run from PONG + 1, so PONG = PING
	 * makes its first PING differ from the current one whatever an earlier
	 * run (e.g. a timed-out one, which leaves PING = PONG + 1) left behind */
	uint16_t last = rd16(O_PING);
	wr16(O_PONG, last);
	wr16(O_COUNT, (uint16_t)n);
	wr16(O_CMD, run);

	/* the probe looks at CMD about every 1 ms and gives up after ~1 s
	 * without an answer; allow for that plus slow round trips */
	double t0 = now_s(), limit = 3.0 + n * 0.01;
	unsigned long echoes = 0, quiet = 0;
	for (;;) {
		uint16_t v = rd16(O_PING);
		if (v != last) {
			wr16(O_PONG, v);
			last = v;
			echoes++;
			quiet = 0;
			continue;
		}
		if (++quiet < 1024) continue;
		quiet = 0;
		if (rd16(O_DONE) == run) break;
		if (now_s() - t0 > limit) {
			fprintf(stderr, "no end of run after %.1f s (%lu echoes): is the probe core running?\n",
			        now_s() - t0, echoes);
			return 1;
		}
	}
	double wall = now_s() - t0;
	uint16_t stat = rd16(O_STAT);
	if (stat) {
		fprintf(stderr, "the probe timed out waiting for an echo (STATUS %u) after %lu echoes\n",
		        stat, echoes);
		return 1;
	}

	static uint32_t cyc[MAX_N], sorted[MAX_N];
	static uint16_t polls[MAX_N];
	double sum = 0, psum = 0, rsum = 0, rmin = 1e30;
	for (int i = 0; i < n; i++) {
		unsigned o = O_SMP + 8u * i;
		cyc[i] = ((uint32_t)rd16(o) << 16) | rd16(o + 2);
		polls[i] = rd16(o + 4);
		sorted[i] = cyc[i];
		sum += cyc[i];
		psum += polls[i];
		if (polls[i]) {
			double r = (double)cyc[i] / polls[i];
			rsum += r;
			if (r < rmin) rmin = r;
		}
	}
	qsort(sorted, n, sizeof(sorted[0]), cmp_u32);

	double us = 1.0 / mhz;
	printf("mbox_ping: %d round trips, FPGA clock %u MHz, %s, %s, %.2f s\n",
	       n, mhz, cpu >= 0 ? "pinned" : "any CPU", fifo ? "SCHED_FIFO" : "SCHED_OTHER", wall);
	if (cpu >= 0) printf("  CPU %d\n", cpu);
	printf("round trip (us): min %.2f  median %.2f  p90 %.2f  p99 %.2f  p99.9 %.2f  max %.2f  mean %.2f\n",
	       sorted[0] * us, pct(sorted, n, 0.5) * us, pct(sorted, n, 0.9) * us,
	       pct(sorted, n, 0.99) * us, pct(sorted, n, 0.999) * us, sorted[n - 1] * us,
	       sum / n * us);
	printf("FPGA reads of PONG per round trip: mean %.1f; one read takes %.2f us (min %.2f)\n",
	       psum / n, rsum / n * us, rmin * us);

	if (csv) {
		FILE *f = fopen(csv, "w");
		if (!f) {
			perror(csv);
			return 1;
		}
		fprintf(f, "sample,cycles,us,pong_reads\n");
		for (int i = 0; i < n; i++)
			fprintf(f, "%d,%u,%.3f,%u\n", i, cyc[i], cyc[i] * us, polls[i]);
		fclose(f);
	}
	return 0;
}
