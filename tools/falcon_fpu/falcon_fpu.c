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
 *   falcon_fpu [-m file] [-c cpu] [-f] [-d] [-s] [-v]
 *     -m FILE  map FILE instead of /dev/mem, offset 0 = mailbox (simulation)
 *     -c CPU   pin to one CPU core (Main_MiSTer runs on CPU 1: use 0)
 *     -f       SCHED_FIFO real-time priority
 *     -d       exit when the parent (Main_MiSTer) exits
 *     -s       timing statistics every 5 s and at the end (see stats_print)
 *     -v       print a line per request
 *
 * Main_MiSTer (support/falcon/falcon_fpu.cpp) starts it as "-c 0 -f -d"
 * when the Falcon core starts and stops it before the FPGA is loaded with
 * another core.  Only one instance may serve the mailbox (/dev/mem use takes
 * an exclusive lock on /tmp/falcon_fpu.lock).
 *
 * Polling: batches of POLL_BATCH reads of the request word with no system
 * call in between; between batches the clock: spin while requests keep
 * arriving, poll every NAP_NS after SPIN_NS without one.  Under SCHED_FIFO
 * a busy spin longer than RUN_NS is broken by a GIVE_NS sleep: about 10% of
 * CPU 0 stays with Linux, below the RT throttling limit
 * (sched_rt_runtime_us, 95%), which would otherwise stop the service for
 * 50 ms at a time.
 *
 * Mailbox (DDR3 0x30E90000 = guest $E90000).  Words are 16 bit in the
 * Falcon's big-endian byte order; operand and result bytes are stored in
 * memory order (DDR3 byte k = guest byte k), so they are copied as is.  The
 * mapping is uncached: every access is a bus transaction, so the mailbox is
 * read and written in aligned 32-bit words (two 16-bit fields each).
 *   +$000 MAGIC $4650  +$002 HEARTBEAT  +$004 VERSION (4)
 *   request (FPGA -> ARM), RSEQ written last:
 *   +$100 RSEQ  +$102 KIND  +$104 CMD  +$106 AUX  +$108 NBYTES
 *   +$10A IADDR[31:16]  +$10C IADDR[15:0]  +$110..+$1FF operand bytes
 *   reply (ARM -> FPGA), STATUS written last:
 *   +$200 STATUS  +$202 FLAGS  +$204 FPSR  +$208 FPCR[15:0]  +$20A NBYTES
 *   +$210.. result bytes
 * Request kinds, which fields each uses, STATUS and FLAGS: fpu_request.h.
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
#include <sys/file.h>
#include <sys/mman.h>
#include <sys/prctl.h>
#include <time.h>
#include <unistd.h>

#include "fpu_request.h"

#define FPU_MB_PHYS  0x30E90000u
#define FPU_MB_SIZE  0x10000u
#define FPU_MAGIC    0x4650u        /* "FP" */
#define FPU_VERSION  4u
#define HB_PERIOD_NS 10000000L      /* heartbeat every 10 ms */
#define SPIN_NS      1000000L       /* spin this long after a request ... */
#define NAP_NS       50000L         /* ... then poll every 50 us */
#define RUN_NS       1800000L       /* SCHED_FIFO: at most this long without sleeping ... */
#define GIVE_NS      200000L        /* ... then leave CPU 0 to Linux this long */
#define POLL_BATCH   32             /* request word reads between clock reads */
#define STATS_NS     5000000000LL   /* -s: report period */

/* Cortex-A9 MPCore global timer (Cyclone V HPS: PERIPHBASE 0xFFFEC000 + 0x200),
 * a free-running 64-bit counter readable without a system call (-s only) */
#define GTIMER_PAGE  0xFFFEC000u
#define GTIMER_OFF   0x200u

enum { O_MAGIC = 0x000, O_HB = 0x002, O_VERSION = 0x004,
       O_RSEQ = 0x100, O_CMD = 0x104, O_RNBYTES = 0x108, O_IADDRL = 0x10C, O_RDATA = 0x110,
       O_STATUS = 0x200, O_FPSR = 0x204, O_FPCR = 0x208, O_ADATA = 0x210 };

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

/* aligned 32-bit mailbox access: bytes off..off+3 in memory order (little-endian host) */
static inline uint32_t rd32(unsigned off) { return *(volatile uint32_t *)(mb + off); }
static inline void wr32(unsigned off, uint32_t v) { *(volatile uint32_t *)(mb + off) = v; }
/* the big-endian 16-bit field at off (lo) or off + 2 (hi) of such a word */
static inline uint16_t f_lo(uint32_t w) { return __builtin_bswap16((uint16_t)w); }
static inline uint16_t f_hi(uint32_t w) { return __builtin_bswap16((uint16_t)(w >> 16)); }
static inline uint32_t fields(uint16_t lo, uint16_t hi)
{
	return (uint32_t)__builtin_bswap16(lo) | (uint32_t)__builtin_bswap16(hi) << 16;
}
/* single fields (startup, heartbeat) */
static inline uint16_t rd16(unsigned off)
{
	return __builtin_bswap16(*(volatile uint16_t *)(mb + off));
}
static inline void wr16(unsigned off, uint16_t v)
{
	*(volatile uint16_t *)(mb + off) = __builtin_bswap16(v);
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

/* ---- -s: timing statistics ------------------------------------------------------------ */

static int stats;
static volatile uint32_t *gtimer;       /* NULL: clock_gettime */
static double tick_ns = 1.0;

static inline uint64_t ticks(void)
{
	if (!gtimer) return (uint64_t)now_ns();
	uint32_t hi, lo;
	do {
		hi = gtimer[1];
		lo = gtimer[0];
	} while (gtimer[1] != hi);
	return (uint64_t)hi << 32 | lo;
}

static void stats_init(int devmem)
{
	if (devmem) {
		int fd = open("/dev/mem", O_RDONLY | O_SYNC);
		void *p = fd < 0 ? MAP_FAILED : mmap(NULL, 4096, PROT_READ, MAP_SHARED, fd, GTIMER_PAGE);
		if (p != MAP_FAILED) gtimer = (volatile uint32_t *)((volatile uint8_t *)p + GTIMER_OFF);
	}
	if (gtimer) {
		/* its rate against CLOCK_MONOTONIC */
		int64_t t0 = now_ns();
		uint64_t g0 = ticks();
		struct timespec w = { 0, 50000000L };
		nanosleep(&w, NULL);
		int64_t t1 = now_ns();
		uint64_t g1 = ticks();
		if (g1 > g0) tick_ns = (double)(t1 - t0) / (double)(g1 - g0);
		else gtimer = NULL;             /* not running: use the clock */
	}
	printf("falcon_fpu: timing with %s (%.3f ns per tick)\n",
	       gtimer ? "the A9 global timer" : "clock_gettime", gtimer ? tick_ns : 1.0);
	/* the cost of one uncached mailbox read and write (an unused word, +$FFFC) */
	uint64_t a = ticks();
	for (int i = 0; i < 10000; i++) (void)rd32(0xFFFC);
	uint64_t b = ticks();
	for (int i = 0; i < 10000; i++) wr32(0xFFFC, 0);
	barrier();
	uint64_t c = ticks();
	printf("falcon_fpu: mailbox read %.0f ns, write %.0f ns (32 bit)\n",
	       (double)(b - a) * tick_ns / 10000, (double)(c - b) * tick_ns / 10000);
	fflush(stdout);
}

static const char *kind_name[6] = { "?", "exec", "reset", "cond", "save", "restore" };
static struct {
	uint64_t n[6], serve[6], engine[6];     /* per kind: requests, ticks in serve(), in the engine */
	uint64_t serve_max;
	uint64_t gap, ngap;                     /* STATUS written -> next RSEQ seen, back to back */
	uint64_t last_done;                     /* ticks when the last STATUS was written */
} st;

static void stats_print(const char *when)
{
	uint64_t n = 0;
	for (int k = 1; k < 6; k++) n += st.n[k];
	if (!n) return;
	printf("falcon_fpu: %s: %llu requests\n", when, (unsigned long long)n);
	for (int k = 1; k < 6; k++)
		if (st.n[k])
			printf("  %-7s %9llu  serve %6.2f us (engine %6.2f us, mailbox and the rest %5.2f us)\n",
			       kind_name[k], (unsigned long long)st.n[k],
			       st.serve[k] * tick_ns / 1000 / st.n[k], st.engine[k] * tick_ns / 1000 / st.n[k],
			       (st.serve[k] - st.engine[k]) * tick_ns / 1000 / st.n[k]);
	printf("  longest serve %.2f us\n", st.serve_max * tick_ns / 1000);
	if (st.ngap)
		printf("  back to back: STATUS written -> next RSEQ seen %.2f us (FPGA reply and post, 68030, polling)\n",
		       st.gap * tick_ns / 1000 / st.ngap);
	fflush(stdout);
	uint64_t keep = st.last_done;
	memset(&st, 0, sizeof(st));
	st.last_done = keep;
}

/* ---- requests ------------------------------------------------------------------------- */

/* execute the request in the mailbox (w0: its RSEQ/KIND word) and write the reply */
static void serve(uint32_t w0, int verbose)
{
	uint8_t in[FPU_REQ_MAX + 4], out[FPE_MAXIO + 4];
	uint16_t seq = f_lo(w0), kind = f_hi(w0), cmd = 0, aux = 0;
	int n = 0, out_len = 0;
	uint32_t iaddr = 0;
	uint64_t t0 = stats ? ticks() : 0, te0 = 0, te1 = 0;

	if (kind == FPU_KIND_EXEC || kind == FPU_KIND_COND || kind == FPU_KIND_RESTORE) {
		uint32_t w1 = rd32(O_CMD);                      /* CMD, AUX */
		cmd = f_lo(w1);
		aux = f_hi(w1);
	}
	if (fpu_kind_has_data(kind)) {
		uint32_t w2 = rd32(O_RNBYTES);                  /* NBYTES, IADDR[31:16] */
		n = f_lo(w2);
		if (n > FPU_REQ_MAX) n = FPU_REQ_MAX;
		if (kind == FPU_KIND_EXEC) iaddr = (uint32_t)f_hi(w2) << 16 | f_lo(rd32(O_IADDRL));
		for (int i = 0; i < n; i += 4) {
			uint32_t v = rd32(O_RDATA + i);
			memcpy(in + i, &v, 4);
		}
	}

	if (stats) te0 = ticks();
	uint16_t flags = fpu_request(kind, cmd, aux, iaddr, in, n, out, &out_len);
	uint32_t fpsr = fpe_fpsr();
	if (stats) te1 = ticks();

	if (out_len & 3) memset(out + out_len, 0, 4 - (out_len & 3));
	for (int i = 0; i < out_len; i += 4) {
		uint32_t v;
		memcpy(&v, out + i, 4);
		wr32(O_ADATA + i, v);
	}
	wr32(O_FPSR, __builtin_bswap32(fpsr));
	wr32(O_FPCR, fields((uint16_t)fpe_fpcr(), (uint16_t)out_len));
	barrier();                      /* the reply is complete before STATUS */
	wr32(O_STATUS, fields(fpu_status(seq, flags, fpsr), flags));
	barrier();

	if (stats) {
		uint64_t t1 = ticks();
		unsigned k = kind < 6 ? kind : 0;
		st.n[k]++;
		st.serve[k] += t1 - t0;
		st.engine[k] += te1 - te0;
		if (t1 - t0 > st.serve_max) st.serve_max = t1 - t0;
		uint64_t gap = t0 - st.last_done;
		if (st.last_done && gap * tick_ns < 100000.0) {         /* back to back (< 100 us) */
			st.gap += gap;
			st.ngap++;
		}
		st.last_done = t1;
	}
	if (verbose) {
		printf("falcon_fpu: seq %04x kind %u cmd %04x aux %04x in %d -> out %d flags %x fpsr %08x\n",
		       seq, kind, cmd, aux, n, out_len, flags, fpsr);
		fflush(stdout);
	}
}

int main(int argc, char **argv)
{
	int cpu = -1, fifo = 0, verbose = 0, parent = 0, opt;
	const char *map = NULL;

	while ((opt = getopt(argc, argv, "m:c:fdsvh")) != -1) {
		switch (opt) {
		case 'm': map = optarg; break;
		case 'c': cpu = atoi(optarg); break;
		case 'f': fifo = 1; break;
		case 'd': parent = 1; break;
		case 's': stats = 1; break;
		case 'v': verbose = 1; break;
		default:
			fprintf(stderr, "usage: %s [-m file] [-c cpu] [-f] [-d] [-s] [-v]\n", argv[0]);
			return 2;
		}
	}

	if (parent) {
		/* also when Main_MiSTer restarts itself for another core */
		if (prctl(PR_SET_PDEATHSIG, SIGTERM) || getppid() == 1) return 1;
	}
	if (!map) {
		int lk = open("/tmp/falcon_fpu.lock", O_RDWR | O_CREAT, 0644);
		if (lk < 0 || flock(lk, LOCK_EX | LOCK_NB)) {
			fprintf(stderr, "falcon_fpu: another instance is running\n");
			return 1;
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

	if (stats) stats_init(!map);
	fpe_reset();
	/* answer nothing that was posted before we ran: catch up with RSEQ */
	uint16_t seen = rd16(O_RSEQ);
	wr32(O_STATUS, fields(fpu_status(seen, 0, 0), 0));
	uint16_t hb = rd16(O_HB);
	wr16(O_VERSION, FPU_VERSION);
	barrier();
	wr16(O_MAGIC, FPU_MAGIC);
	barrier();

	int64_t t = now_ns(), next_hb = t, last_req = t, last_nap = t, next_stats = t + STATS_NS;
	while (!quit) {
		int got = 0;
		for (int i = 0; i < POLL_BATCH; i++) {
			uint32_t w0 = rd32(O_RSEQ);             /* RSEQ, KIND */
			if (f_lo(w0) != seen) {
				seen = f_lo(w0);
				serve(w0, verbose);
				got = 1;
			}
		}
		t = now_ns();
		if (got) last_req = t;
		if (t >= next_hb) {
			wr16(O_HB, ++hb);
			barrier();
			next_hb += HB_PERIOD_NS;
			if (next_hb < t) next_hb = t + HB_PERIOD_NS;
		}
		if (stats && t >= next_stats) {
			stats_print("last 5 s");
			next_stats = t + STATS_NS;
		}
		if (t - last_req > SPIN_NS) {   /* idle: stop spinning */
			struct timespec nap = { 0, NAP_NS };
			nanosleep(&nap, NULL);
			last_nap = t;
		} else if (fifo && t - last_nap > RUN_NS) {
			struct timespec give = { 0, GIVE_NS };
			nanosleep(&give, NULL);
			last_nap = now_ns();
		}
	}

	wr16(O_MAGIC, 0);
	barrier();
	if (stats) stats_print("at exit");
	return 0;
}
