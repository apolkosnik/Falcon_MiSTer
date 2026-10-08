/*
 * falcon_fpu - HPS side of the Falcon core's MC68882 (docs/FPU_ARM.md)
 *
 * The FPGA bridge (rtl/falcon/falcon_fpu_bridge.sv) answers the 68030's
 * coprocessor interface and posts one request per FPU instruction into a
 * DDR3 mailbox; this program executes it with Hatari's 68882 emulation
 * (engine/, libfpe) and writes the reply.  It also keeps MAGIC and a
 * heartbeat in the mailbox: while the heartbeat moves the core reports an
 * FPU.  On SIGINT/SIGTERM MAGIC is cleared, so the FPU disappears at once.
 *
 *   falcon_fpu [-m file] [-c cpu] [-f] [-v]
 *     -m FILE  map FILE instead of /dev/mem, offset 0 = mailbox (simulation)
 *     -c CPU   pin to one CPU core (Main_MiSTer runs on CPU 1: use 0)
 *     -f       SCHED_FIFO real-time priority
 *     -v       print a line per request
 *
 * Mailbox (DDR3 0x30E90000 = guest $E90000).  Words are 16 bit in the
 * Falcon's big-endian byte order; operand and result bytes are stored in
 * memory order (DDR3 byte k = guest byte k), so they are copied as is.
 *   +$000 MAGIC $4650  +$002 HEARTBEAT  +$004 VERSION (2)
 *   request (FPGA -> ARM), RSEQ written last:
 *   +$100 RSEQ  +$102 KIND (1 execute, 2 reset, 3 condition)  +$104 CMD  +$106 AUX
 *   +$108 NBYTES  +$10A IADDR[31:16]  +$10C IADDR[15:0]  +$110.. operand bytes
 *   reply (ARM -> FPGA), ASEQ written last:
 *   +$200 ASEQ  +$202 FLAGS  +$204 FPSR[31:16]  +$206 FPSR[15:0]
 *   +$208 FPCR[15:0]  +$20A NBYTES  +$210.. result bytes
 * FLAGS: bit 0 the instruction is not implemented (the bridge answers it
 * with the F-line exception), bit 1 an exception is pending (milestone 4;
 * for a condition request: the BSUN exception), bit 2 the condition is true.
 * A condition request (the bridge sends only IEEE-nonaware predicates with
 * NAN set: they set BSUN/IOP in the FPSR) carries the predicate in CMD.
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

#include "engine/falcon_fpu_engine.h"

#define FPU_MB_PHYS  0x30E90000u
#define FPU_MB_SIZE  0x10000u
#define FPU_MAGIC    0x4650u        /* "FP" */
#define FPU_VERSION  2u
#define HB_PERIOD_NS 10000000L      /* heartbeat every 10 ms */
#define SPIN_NS      1000000L       /* spin this long after a request ... */
#define NAP_NS       50000L         /* ... then poll every 50 us */
#define MAX_BYTES    96

enum { O_MAGIC = 0x000, O_HB = 0x002, O_VERSION = 0x004,
       O_RSEQ = 0x100, O_KIND = 0x102, O_CMD = 0x104, O_AUX = 0x106,
       O_RNBYTES = 0x108, O_IADDR = 0x10A, O_RDATA = 0x110,
       O_ASEQ = 0x200, O_FLAGS = 0x202, O_FPSR = 0x204, O_FPCR = 0x208,
       O_ANBYTES = 0x20A, O_ADATA = 0x210 };

enum { KIND_EXEC = 1, KIND_RESET = 2, KIND_COND = 3 };

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
}

static int64_t now_ns(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (int64_t)t.tv_sec * 1000000000LL + t.tv_nsec;
}

static void on_signal(int sig)
{
	(void)sig;
	quit = 1;
}

/* execute the request in the mailbox and write the reply */
static void serve(uint16_t seq, int verbose)
{
	uint8_t in[MAX_BYTES], out[MAX_BYTES];
	uint16_t kind = rd16(O_KIND), cmd = rd16(O_CMD), aux = rd16(O_AUX);
	int n = rd16(O_RNBYTES), out_len = 0, flags = 0;
	uint32_t iaddr = (uint32_t)rd16(O_IADDR) << 16 | rd16(O_IADDR + 2);

	if (n > MAX_BYTES) n = MAX_BYTES;
	for (int i = 0; i < n; i++) in[i] = mb[O_RDATA + i];

	if (kind == KIND_RESET)
		fpe_reset();
	else if (kind == KIND_COND) {
		int r = fpe_cond(cmd & 0x3f);
		if (r == -2) {              /* BSUN enabled: the bridge raises it now */
			flags = 2;
			fpe_clear_exception();
		} else if (r)
			flags = 4;
	} else
		flags = fpe_exec(cmd, in, n, aux, iaddr, out, &out_len);
	if (out_len < 0 || out_len > MAX_BYTES) out_len = 0;

	for (int i = 0; i < out_len; i++) mb[O_ADATA + i] = out[i];
	uint32_t fpsr = fpe_fpsr();
	wr16(O_FLAGS, (uint16_t)flags);
	wr16(O_FPSR, (uint16_t)(fpsr >> 16));
	wr16(O_FPSR + 2, (uint16_t)fpsr);
	wr16(O_FPCR, (uint16_t)fpe_fpcr());
	wr16(O_ANBYTES, (uint16_t)out_len);
	barrier();                      /* the reply is complete before ASEQ */
	wr16(O_ASEQ, seq);
	barrier();

	if (verbose) {
		printf("falcon_fpu: seq %04x kind %u cmd %04x aux %04x in %d -> out %d flags %x fpsr %08x\n",
		       seq, kind, cmd, aux, n, out_len, flags, fpsr);
		fflush(stdout);
	}
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

	fpe_reset();
	/* answer nothing that was posted before we ran: catch up with RSEQ */
	uint16_t seen = rd16(O_RSEQ);
	wr16(O_ASEQ, seen);
	uint16_t hb = rd16(O_HB);
	wr16(O_VERSION, FPU_VERSION);
	barrier();
	wr16(O_MAGIC, FPU_MAGIC);
	barrier();

	int64_t t = now_ns(), next_hb = t, last_req = t;
	while (!quit) {
		uint16_t seq = rd16(O_RSEQ);
		t = now_ns();
		if (seq != seen) {
			seen = seq;
			serve(seq, verbose);
			last_req = t;
		}
		if (t >= next_hb) {
			wr16(O_HB, ++hb);
			barrier();
			next_hb += HB_PERIOD_NS;
			if (next_hb < t) next_hb = t + HB_PERIOD_NS;
		}
		if (t - last_req > SPIN_NS) {   /* idle: stop spinning */
			struct timespec nap = { 0, NAP_NS };
			nanosleep(&nap, NULL);
		}
	}

	wr16(O_MAGIC, 0);
	barrier();
	return 0;
}
