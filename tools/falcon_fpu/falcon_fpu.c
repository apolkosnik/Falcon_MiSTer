/*
 * falcon_fpu - HPS side of the Falcon core's MC68882 (docs/FPU_ARM.md)
 *
 * Milestone 1: presence only.  Writes MAGIC and VERSION into the FPU
 * mailbox and increments HEARTBEAT every 10 ms; the FPGA bridge
 * (rtl/falcon/falcon_fpu_bridge.sv) reports an FPU to the 68030 while the
 * heartbeat moves.  On SIGINT/SIGTERM MAGIC is cleared, so the FPU
 * disappears at once instead of after the bridge's 100 ms timeout.
 *
 *   falcon_fpu [-m file] [-c cpu] [-f] [-v]
 *     -m FILE  map FILE instead of /dev/mem, offset 0 = mailbox (simulation)
 *     -c CPU   pin to one CPU core (Main_MiSTer runs on CPU 1: use 0)
 *     -f       SCHED_FIFO real-time priority
 *     -v       print the heartbeat once a second
 *
 * Mailbox: DDR3 0x30E90000 (guest $E90000), 16-bit words in the Falcon's
 * big-endian byte order.
 */
#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#define FPU_MB_PHYS  0x30E90000u
#define FPU_MB_SIZE  0x10000u
#define FPU_MAGIC    0x4650u        /* "FP" */
#define FPU_VERSION  1u
#define HB_PERIOD_NS 10000000L      /* 10 ms */

enum { O_MAGIC = 0x000, O_HB = 0x002, O_VERSION = 0x004 };

static volatile uint8_t *mb;
static volatile sig_atomic_t quit;

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

static void on_signal(int sig)
{
	(void)sig;
	quit = 1;
}

int main(int argc, char **argv)
{
	int cpu = -1, fifo = 0, verbose = 0, opt;
	const char *map = NULL;

	while ((opt = getopt(argc, argv, "m:c:fvh")) != -1) {
		switch (opt) {
		case 'm': map = optarg; break;
		case 'c': cpu = atoi(optarg); break;
		case 'f': fifo = 1; break;
		case 'v': verbose = 1; break;
		default:
			fprintf(stderr, "usage: %s [-m file] [-c cpu] [-f] [-v]\n", argv[0]);
			return 2;
		}
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
	void *p = mmap(NULL, FPU_MB_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, map ? 0 : FPU_MB_PHYS);
	if (p == MAP_FAILED) {
		perror("mmap");
		return 1;
	}
	mb = p;

	struct sigaction sa;
	memset(&sa, 0, sizeof(sa));
	sa.sa_handler = on_signal;
	sigaction(SIGINT, &sa, NULL);
	sigaction(SIGTERM, &sa, NULL);

	uint16_t hb = rd16(O_HB);
	wr16(O_VERSION, FPU_VERSION);
	wr16(O_MAGIC, FPU_MAGIC);

	struct timespec next;
	clock_gettime(CLOCK_MONOTONIC, &next);
	unsigned long ticks = 0;
	while (!quit) {
		wr16(O_HB, ++hb);
		if (verbose && ++ticks % 100 == 0) {
			printf("falcon_fpu: heartbeat %u\n", hb);
			fflush(stdout);
		}
		next.tv_nsec += HB_PERIOD_NS;
		if (next.tv_nsec >= 1000000000L) {
			next.tv_nsec -= 1000000000L;
			next.tv_sec++;
		}
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
	}

	wr16(O_MAGIC, 0);
	return 0;
}
