// tb_fpu.cpp - Verilator co-simulation of the MC68882 FPU bridge, milestone 1.
//
// The design under test is the real RTL: AP68030 CPU + falcon_cpubus +
// falcon_memarb + falcon_fpu_bridge (tb_fpu_top.sv), running real 68k test
// programs (asm/*.s) from a ROM image at $E00000.  The DDR3 behind the
// arbiter is a C++ model with MiSTer DDRAM semantics (64-bit words, random
// BUSY and read latency); the mailbox window $E90000..$E9FFFF is a MAP_SHARED
// file, so the real ARM service (tools/falcon_fpu, built for the host) can
// serve it as a child process.  Heartbeats come either from a deterministic
// C++ writer or from that child.
//
// Time: the bridge parameter CLK_HZ is TB_CLK_HZ (200 kHz): the mailbox poll
// is every CLK_HZ/200 = 1000 clocks, the alive window 20 polls = 20000
// clocks, one simulated clock = 5 us.  Runs with the real child are paced
// to wall time (1 clock = 1/CLK_HZ s) so its 10 ms heartbeat is 10 ms of
// Falcon time as well.
//
// Expected values come from the manuals (cited at every check), never from
// the RTL: MC68881/MC68882 User's Manual (UM), MC68030 UM, EmuTOS
// bios/processor.S, and Hatari src/cpu/fpp.c as a cross-check.  A failing
// check stays failing.

#include <cerrno>
#include <csignal>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <map>
#include <set>
#include <random>
#include <string>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>
#include <vector>

#include "Vtb_fpu_top.h"
#include "verilated.h"
#include "m2.h"
#include "m3.h"
#include "golden.h"

#ifndef TB_CLK_HZ
#define TB_CLK_HZ 200000
#endif

static const unsigned CLK_HZ = TB_CLK_HZ;
static const unsigned POLL_CLKS = CLK_HZ / 200;        // bridge header: polls every CLK_HZ/200 clocks (5 ms)
static const unsigned ALIVE_POLLS = 20;                // bridge header: absent after 20 polls without a new heartbeat
static const unsigned HB_PERIOD = CLK_HZ / 100;        // the ARM service: 10 ms
static const unsigned MB_G = 0xE90000, MB_SIZE = 0x10000;
static const uint64_t DDR_BYTE0 = 0x30000000ull, DDR_SIZE = 0x1000000ull;
static const uint16_t MAGIC = 0x4650;
static const uint16_t VERSION = 4;                     // mailbox protocol version (bridge header)

//------------------------------------------------------------------ reporting
static int n_pass = 0, n_fail = 0;
static std::string cur_test;
static std::vector<std::string> failed_checks;

static void report(bool ok, const char *grp, const std::string &what, const char *src)
{
	if (ok) n_pass++;
	else {
		n_fail++;
		failed_checks.push_back(cur_test + " / " + what);
	}
	printf("  %s  [%s] %s  (%s)\n", ok ? "PASS" : "FAIL", grp, what.c_str(), src);
}

static std::string fmt(const char *f, ...)
{
	char b[1024];
	va_list ap;
	va_start(ap, f);
	vsnprintf(b, sizeof b, f, ap);
	va_end(ap);
	return b;
}

// compare numbers: prints "expected X got Y"
static bool check_eq(const char *grp, const std::string &what, uint64_t exp, uint64_t got, const char *src, int digits = 8)
{
	bool ok = exp == got;
	report(ok, grp, fmt("%s: expected 0x%0*llx got 0x%0*llx", what.c_str(), digits, (unsigned long long)exp, digits,
	                    (unsigned long long)got), src);
	return ok;
}
static bool check_true(const char *grp, const std::string &what, bool cond, const char *src)
{
	report(cond, grp, what, src);
	return cond;
}

//------------------------------------------------------------------ DUT, DDR3, mailbox
static Vtb_fpu_top *T = nullptr;
static uint64_t g_clk = 0;
static bool g_por = true, g_reset = true;
static unsigned g_ipl = 0;                         // interrupt request level (0 = none), autovectored
static std::vector<uint8_t> ddr;      // guest memory, offset = guest address (DDR3 byte k = guest byte k)
static uint8_t *MB = nullptr;         // mapped mailbox file
static std::string mb_path;
static std::mt19937_64 rng;
static uint32_t rnd(uint32_t n) { return n ? (uint32_t)(rng() % n) : 0; }

static inline uint16_t mb16(unsigned off)
{
	return __builtin_bswap16(__atomic_load_n((uint16_t *)(MB + off), __ATOMIC_ACQUIRE));
}
static inline void mb_w16(unsigned off, uint16_t v)
{
	__atomic_store_n((uint16_t *)(MB + off), __builtin_bswap16(v), __ATOMIC_RELEASE);
}

static inline bool in_mb(uint32_t off) { return off >= MB_G && off < MB_G + MB_SIZE; }
static inline uint8_t *ddr_ptr(uint32_t off) { return in_mb(off) ? MB + (off - MB_G) : &ddr[off]; }

static uint64_t ddr_read64(uint32_t off)
{
	uint64_t w = 0;
	if (in_mb(off)) {
		for (int p = 0; p < 4; p++) {
			uint16_t v = __atomic_load_n((uint16_t *)(ddr_ptr(off) + 2 * p), __ATOMIC_ACQUIRE);
			w |= (uint64_t)v << (16 * p);
		}
	} else
		memcpy(&w, ddr_ptr(off), 8);
	return w;
}
static void ddr_write64(uint32_t off, uint64_t din, uint8_t be)
{
	for (int p = 0; p < 4; p++) {
		unsigned m = (be >> (2 * p)) & 3;
		uint8_t *q = ddr_ptr(off) + 2 * p;
		uint16_t v = (uint16_t)(din >> (16 * p));
		if (m == 3) {
			if (in_mb(off)) __atomic_store_n((uint16_t *)q, v, __ATOMIC_RELEASE);
			else memcpy(q, &v, 2);
		} else if (m == 1) q[0] = (uint8_t)v;
		else if (m == 2) q[1] = (uint8_t)(v >> 8);
	}
}

// guest memory (big endian)
static uint32_t gr32(uint32_t a)
{
	return ((uint32_t)ddr[a] << 24) | ((uint32_t)ddr[a + 1] << 16) | ((uint32_t)ddr[a + 2] << 8) | ddr[a + 3];
}
// (a write the CPU polls for: the CPU's line cache, falcon_l2, is told at the
// next clock, as for another master's write)
static bool inv_pend = false;
static uint32_t inv_addr = 0;
static void gw32(uint32_t a, uint32_t v)
{
	ddr[a] = v >> 24; ddr[a + 1] = v >> 16; ddr[a + 2] = v >> 8; ddr[a + 3] = (uint8_t)v;
	inv_pend = true; inv_addr = a;
}

// DDR3 command / data model
static int rd_left = 0, rd_wait = 0;
static uint32_t rd_off = 0;
static int busy_run = 0;
static uint64_t nxt_dout = 0;
static bool nxt_ready = false, nxt_busy = false;
static uint64_t ddr_proto_err = 0, n_ddr_rd = 0, n_ddr_wr = 0;
static std::string ddr_first_err;

static void ddr_pre()
{
	bool rd = T->DDRAM_RD, wr = T->DDRAM_WE;
	if ((rd || wr) && !T->DDRAM_BUSY) {
		uint64_t byte = (uint64_t)T->DDRAM_ADDR * 8;
		bool inr = byte >= DDR_BYTE0 && byte < DDR_BYTE0 + DDR_SIZE;
		int burst = T->DDRAM_BURSTCNT;
		// (reads of 4: the CPU's line cache, falcon_l2, as the video's bursts)
		bool ok = !(rd && wr) && inr && rd_left == 0 && (burst == 1 || (rd && burst == 4));
		if (!ok) {
			ddr_proto_err++;
			if (ddr_first_err.empty())
				ddr_first_err = fmt("clk %llu rd=%d wr=%d byte=0x%llx burst=%d rd_left=%d", (unsigned long long)g_clk, rd, wr,
				                    (unsigned long long)byte, burst, rd_left);
		}
		if (inr && !(rd && wr)) {
			uint32_t off = (uint32_t)(byte - DDR_BYTE0);
			if (wr) {
				ddr_write64(off, T->DDRAM_DIN, T->DDRAM_BE);
				n_ddr_wr++;
			} else {
				rd_off = off; rd_left = burst; rd_wait = 1 + rnd(30);
				n_ddr_rd++;
			}
		}
	}
	nxt_ready = false;
	nxt_dout = (uint64_t)rng();
	if (rd_left > 0) {
		if (rd_wait > 1) rd_wait--;
		else if (rnd(6) == 0) { /* gap */ }
		else {
			nxt_dout = ddr_read64(rd_off);
			nxt_ready = true;
			rd_off += 8; rd_left--;
		}
	}
	nxt_busy = false;
	if (busy_run > 0) { busy_run--; nxt_busy = true; }
	else if (rnd(100) < 30) nxt_busy = true;
	if (rnd(300) == 0) busy_run = rnd(25);
}

//------------------------------------------------------------------ tags shared with the assembler
static std::map<std::string, uint32_t> TAG;
static void load_tags(const char *path)
{
	FILE *f = fopen(path, "r");
	if (!f) { fprintf(stderr, "cannot open %s\n", path); exit(2); }
	char line[512];
	while (fgets(line, sizeof line, f)) {
		char name[128], val[64];
		if (sscanf(line, "%127s equ %63s", name, val) == 2 && name[0] == 'T' && name[1] == '_') {
			TAG[name] = (uint32_t)strtoul(val[0] == '$' ? val + 1 : val, nullptr, val[0] == '$' ? 16 : 10);
		}
	}
	fclose(f);
}
static uint32_t Tg(const char *n)
{
	auto it = TAG.find(n);
	if (it == TAG.end()) { fprintf(stderr, "unknown tag %s\n", n); exit(2); }
	return it->second;
}

// guest control block (asm/common.i)
enum { A_GO = 0x0F00, A_STOP = 0x0F04, A_DONE = 0x0F08, A_EXCNT = 0x0F0C, A_EXLOGP = 0x0F10, A_STATUS = 0x0F18,
       A_ITER = 0x0F20, A_UNEXP = 0x0F24, A_READY = 0x0F28, A_CASEIX = 0x0F30, A_MARK1 = 0x0F40, A_MARK2 = 0x0F44, A_MARK3 = 0x0F48, A_MARK4 = 0x0F4C, A_MODE = 0x0F50, A_IRQCNT = 0x0F5C, A_HOLDW = 0x0F60, A_EXLOG = 0x80000, A_RECLOG = 0x4000 };

//------------------------------------------------------------------ monitors
// CIR bus cycles (CPU space type 2) seen on the 68030 pins
struct Cir {
	uint64_t clk0, clk1, instr;
	unsigned id, off, siz;
	bool rw;            // read
	uint32_t data;      // D31..D0 as moved (word CIRs: D31..D16)
	bool berr, ds0, ds1, present;
};
static std::vector<Cir> cirs;
static uint64_t instr_idx = 0, other_berr = 0, n_cycles = 0, n_halted_seen = 0;
static bool prev_as = false, cur_term = false, cur_is_cir = false, cur_ack_seen = false, prev_present = false;
static Cir cur;

// mailbox polls seen on the arbiter's d3 port, and the presence scoreboard
struct Poll { uint64_t clk; bool hb; uint16_t data; };
static std::vector<Poll> polls;
static unsigned g_prim_pc_seen = 0;                // take-exception primitives with the PC bit seen by the last bus_checks
static unsigned ca_run = 0;                       // consecutive come-again responses (a hang detector)
static unsigned n_kind[8];                       // request KIND words posted (1 execute, 2 reset, 3 condition)
static std::vector<uint64_t> req_start_magic;   // clock of each MAGIC read request
static bool prev_d3_req = false;
static bool pr_prev = false;
static uint64_t pr_fall = 0, pr_rise = 0;
static std::vector<uint64_t> pr_falls, pr_rises;
static std::vector<unsigned> pr_fall_unch;       // unchanged HB reads at each fall
static unsigned sb_alive = 0, sb_unch = 0;
static uint16_t sb_last_hb = 0;
static bool sb_exp = false, sb_next = false;
static uint64_t sb_eff = 0, sb_mismatch = 0;
static std::string sb_first_mismatch;
static int sb_last_was = 0;                      // 1 magic bad, 2 hb unchanged, 3 hb changed

static void monitors_reset()
{
	cirs.clear(); instr_idx = 0; other_berr = 0; n_cycles = 0; prev_as = false; cur_term = false; cur_is_cir = false;
	polls.clear(); req_start_magic.clear(); prev_d3_req = false; memset(n_kind, 0, sizeof n_kind); ca_run = 0;
	pr_prev = false; pr_fall = pr_rise = 0; pr_falls.clear(); pr_rises.clear(); pr_fall_unch.clear();
	sb_alive = 0; sb_unch = 0; sb_last_hb = 0; sb_exp = sb_next = false; sb_eff = 0; sb_mismatch = 0; sb_first_mismatch.clear();
	sb_last_was = 0;
	ddr_proto_err = 0; ddr_first_err.clear(); n_ddr_rd = n_ddr_wr = 0;
}

static void sample()
{
	if (T->o_dbg_inst) instr_idx++;
	bool as = !T->o_as_n && T->o_bus_oe;
	if (as && !prev_as) {
		cur = Cir();
		cur.clk0 = g_clk; cur.instr = instr_idx;
		uint32_t a = T->o_a;
		cur_is_cir = T->o_fc == 7 && ((a >> 16) & 0xF) == 2;
		cur.id = (a >> 13) & 7; cur.off = a & 0x1F; cur.siz = T->o_siz; cur.rw = T->o_rw;
		cur_term = false;
		cur_ack_seen = false;
		if (T->o_fc == 7 && ((a >> 16) & 0xF) == 0xF) g_ipl = 0;      // interrupt acknowledge: the source is gone
		n_cycles++;
	}
	if (as) {
		// the bridge decides BERR/no BERR when it processes the access (T_CIR): the value of
		// `present` it used is the one before the clock in which cp_ack/cp_berr appears
		if (cur_is_cir && T->o_cp_ack && !cur_ack_seen) { cur.present = prev_present; cur_ack_seen = true; }
		bool b = !T->o_berr_n, d0 = !T->o_dsack0_n, d1 = !T->o_dsack1_n;
		if (!cur_term && (b || d0 || d1)) {
			cur_term = true;
			cur.clk1 = g_clk; cur.berr = b; cur.ds0 = d0; cur.ds1 = d1;
			cur.data = cur.rw ? T->o_d_i : T->o_d_o;
			if (cur_is_cir) {
				cirs.push_back(cur);
				unsigned hi = cur.data >> 16;
				if (cur.rw && ((cur.off == 0x00 && (hi == 0x8900 || hi == 0xC900)) || (cur.off == 0x04 && hi == 0x0118))) ca_run++;
				else ca_run = 0;
			}
			else if (b) other_berr++;
		}
	}
	prev_as = as;

	// d3: mailbox polls
	bool rq = T->o_d3_req;
	if (rq && !prev_d3_req && !T->o_d3_we && T->o_d3_addr == (0xE90000 >> 1)) req_start_magic.push_back(g_clk);
	prev_d3_req = rq;
	if (g_clk >= sb_eff) sb_exp = sb_next;
	if (g_por) { sb_alive = 0; sb_unch = 0; sb_exp = sb_next = false; }
	else if (T->o_d3_ack && T->o_d3_we && T->o_d3_addr == (0xE90100 >> 1)) {
		// a request is posted (RSEQ written last): its KIND is in the mailbox, written now or
		// left by an earlier request (the bridge writes a header field only when it changes)
		unsigned kd = mb16(0x102);
		if (kd < 8) n_kind[kd]++;
	} else if (T->o_d3_ack && !T->o_d3_we) {
		unsigned wa = T->o_d3_addr;
		uint16_t d = T->o_d3_rdata;
		if (wa == (0xE90000 >> 1)) {                 // MAGIC
			polls.push_back({g_clk, false, d});
			if (d != MAGIC) sb_alive = 0;
		} else if (wa == (0xE90200 >> 1) && getenv("M2_SEQ")) {   // debugging: reply STATUS words
			static FILE *df = fopen("obj/d3_status.log", "w");
			if (df) fprintf(df, "%llu reply STATUS=%04x\n", (unsigned long long)g_clk, d);
		} else if (wa == (0xE90004 >> 1)) {          // VERSION: anything but VERSION means absent
			polls.push_back({g_clk, false, d});
			if (d != VERSION) sb_alive = 0;
		} else if (wa == (0xE90002 >> 1)) {          // HEARTBEAT
			polls.push_back({g_clk, true, d});
			if (d != sb_last_hb) { sb_alive = ALIVE_POLLS; sb_unch = 0; }
			else { if (sb_alive) sb_alive--; sb_unch++; }
			sb_last_hb = d;
		}
		sb_next = sb_alive != 0;
		sb_eff = g_clk + 1;
	}
	bool pr = T->o_present;
	if (!g_por && g_clk > sb_eff + 1 && pr != sb_exp) {
		sb_mismatch++;
		if (sb_first_mismatch.empty())
			sb_first_mismatch = fmt("clk %llu present=%d expected=%d", (unsigned long long)g_clk, pr, sb_exp);
	}
	if (pr && !pr_prev) { pr_rise = g_clk; pr_rises.push_back(g_clk); }
	if (!pr && pr_prev) { pr_fall = g_clk; pr_falls.push_back(g_clk); pr_fall_unch.push_back(sb_unch); }
	pr_prev = pr;
	prev_present = pr;
}

//------------------------------------------------------------------ heartbeat sources
static bool hb_on = false;
static uint16_t hb_val = 0;
static uint64_t hb_next = 0;
static uint64_t mb_hb_change_clk = 0, mb_magic_zero_clk = 0;
static uint16_t mb_prev_hb = 0, mb_prev_magic = 0;

static pid_t child = -1;
static bool child_exited = false;
static uint64_t child_exit_clk = 0;
static int child_status = 0;
static std::string host_exe;
static bool paced = false;
static struct timespec pace_t0;
static uint64_t pace_clk0 = 0;

static void hb_start(uint16_t hb0 = 1)
{
	mb_w16(0x004, VERSION);
	hb_val = hb0;
	mb_w16(0x002, hb_val);
	mb_w16(0x000, MAGIC);
	hb_next = g_clk + HB_PERIOD;
	hb_on = true;
}

static void child_poll()
{
	if (child > 0 && !child_exited) {
		int st;
		pid_t r = waitpid(child, &st, WNOHANG);
		if (r == child) { child_exited = true; child_exit_clk = g_clk; child_status = st; }
	}
}

static void pace()
{
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	double target = (double)(g_clk - pace_clk0) / CLK_HZ;
	double have = (now.tv_sec - pace_t0.tv_sec) + (now.tv_nsec - pace_t0.tv_nsec) * 1e-9;
	if (have < target) {
		double d = target - have;
		struct timespec ts;
		ts.tv_sec = (time_t)d;
		ts.tv_nsec = (long)((d - (double)ts.tv_sec) * 1e9);
		nanosleep(&ts, nullptr);
	}
}

static void tick()
{
	if (hb_on && g_clk >= hb_next) {
		mb_w16(0x002, ++hb_val);
		hb_next += HB_PERIOD;
	}
	uint16_t h = mb16(0x002), m = mb16(0x000);
	if (h != mb_prev_hb) { mb_hb_change_clk = g_clk; mb_prev_hb = h; }
	if (m != mb_prev_magic) { if (m == 0) mb_magic_zero_clk = g_clk; mb_prev_magic = m; }
	if (child > 0 && (g_clk & 15) == 0) child_poll();
	if (paced && (g_clk & 63) == 0) pace();
}

static void step()
{
	ddr_pre();
	T->por = g_por; T->reset = g_reset; T->ipl_n = (~g_ipl) & 7;
	T->tbinv_we = inv_pend; T->tbinv_addr = (inv_addr >> 3) & 0x1FFFFF; inv_pend = false;
	T->clk = 1; T->eval();
	T->DDRAM_DOUT = nxt_dout; T->DDRAM_DOUT_READY = nxt_ready; T->DDRAM_BUSY = nxt_busy;
	T->clk = 0; T->eval();
	g_clk++;
	sample();
	tick();
}
static void steps(uint64_t n) { while (n--) step(); }

template <class F> static bool run_until(F cond, uint64_t max_clks)
{
	uint64_t end = g_clk + max_clks;
	while (g_clk < end) {
		step();
		if (cond()) return true;
	}
	return cond();
}
static bool is_ready() { return gr32(A_READY) != 0; }
static bool is_done() { return gr32(A_DONE) == 0xD0E0600D; }
static bool present() { return T->o_present; }

//------------------------------------------------------------------ test set-up
static std::vector<uint8_t> read_file(const std::string &p)
{
	std::vector<uint8_t> v;
	FILE *f = fopen(p.c_str(), "rb");
	if (!f) { fprintf(stderr, "cannot open %s\n", p.c_str()); exit(2); }
	uint8_t b[4096];
	size_t n;
	while ((n = fread(b, 1, sizeof b, f)) > 0) v.insert(v.end(), b, b + n);
	fclose(f);
	return v;
}

static unsigned test_no = 0;
static std::string cir_log_dir = "obj";

static void begin_test(const char *name, const char *prog)
{
	cur_test = name;
	printf("\n== %s  (program %s) ==\n", name, prog);
	if (child > 0) { fprintf(stderr, "child still running\n"); exit(2); }
	delete T;
	T = new Vtb_fpu_top;
	rng.seed(0x46505531ull + 977 * (++test_no));
	std::fill(ddr.begin(), ddr.end(), 0);
	memset(MB, 0, MB_SIZE);
	std::vector<uint8_t> img = read_file(std::string("obj/") + prog + ".bin");
	memcpy(&ddr[0xE00000], img.data(), img.size());
	rd_left = 0; busy_run = 0; nxt_ready = false; nxt_busy = false; nxt_dout = 0;
	T->DDRAM_BUSY = 0; T->DDRAM_DOUT_READY = 0; T->DDRAM_DOUT = 0;
	hb_on = false; hb_val = 0; paced = false; g_ipl = 0;
	mb_prev_hb = mb_prev_magic = 0; mb_hb_change_clk = mb_magic_zero_clk = 0;
	child_exited = false; child_exit_clk = 0;
	g_clk = 0;
	monitors_reset();
	g_por = true; g_reset = true;
	steps(60);
}
static void release_reset()
{
	g_por = false; g_reset = false;
	steps(2);
}

static void set_pacing(bool on)
{
	paced = on;
	if (on) { clock_gettime(CLOCK_MONOTONIC, &pace_t0); pace_clk0 = g_clk; }
}

static void child_start()
{
	child_exited = false;
	child = fork();
	if (child == 0) {
		int nul = open("/dev/null", O_WRONLY);
		if (nul >= 0) { dup2(nul, 1); dup2(nul, 2); }
		execl(host_exe.c_str(), host_exe.c_str(), "-m", mb_path.c_str(), (char *)nullptr);
		_exit(127);
	}
	if (child < 0) { perror("fork"); exit(2); }
}
static void child_signal(int sig) { if (child > 0 && !child_exited) kill(child, sig); }
static void child_reap()
{
	if (child <= 0) return;
	if (!child_exited) {
		kill(child, SIGCONT);
		kill(child, SIGTERM);
		for (int i = 0; i < 200 && !child_exited; i++) {
			int st;
			pid_t r = waitpid(child, &st, WNOHANG);
			if (r == child) { child_exited = true; child_status = st; break; }
			usleep(10000);
		}
		if (!child_exited) {
			kill(child, SIGKILL);
			int st;
			waitpid(child, &st, 0);
			child_exited = true;
		}
	}
	child = -1;
}

//------------------------------------------------------------------ results
static std::map<uint32_t, std::vector<uint32_t>> recs;
static void collect()
{
	recs.clear();
	for (uint32_t a = A_RECLOG; a < A_RECLOG + 0x4000; a += 8) {
		uint32_t t = gr32(a);
		if (!t) break;
		recs[t].push_back(gr32(a + 4));
	}
}
static bool expect(const char *grp, uint32_t tag, const std::string &what, uint32_t exp, const char *src, int digits = 8)
{
	auto it = recs.find(tag);
	if (it == recs.end()) {
		report(false, grp, fmt("%s: expected 0x%0*x got <no result recorded> (tag 0x%x)", what.c_str(), digits, exp, tag), src);
		return false;
	}
	if (it->second.size() != 1) {
		report(false, grp, fmt("%s: expected 0x%0*x got %zu results for one tag", what.c_str(), digits, exp, it->second.size()), src);
		return false;
	}
	return check_eq(grp, what, exp, it->second[0], src, digits);
}

struct ExEnt { uint32_t vec, fmt, pc, ia, sr, ix; };
static std::vector<ExEnt> exlog()
{
	std::vector<ExEnt> v;
	uint32_t n = gr32(A_EXCNT);
	for (uint32_t i = 0; i < n && i < 8192; i++) {
		uint32_t a = A_EXLOG + 32 * i;
		v.push_back({gr32(a), gr32(a + 4), gr32(a + 8), gr32(a + 12), gr32(a + 16), gr32(a + 20)});
	}
	return v;
}

//------------------------------------------------------------------ common end-of-run checks
// check 7: bus level
static void bus_checks(const char *tag)
{
	char path[256];
	snprintf(path, sizeof path, "%s/cir_%s.log", cir_log_dir.c_str(), tag);
	FILE *lf = fopen(path, "w");
	if (lf) fprintf(lf, "# clk0 clk1 instr cpid off dir data term(BERR|DSACK1|DSACK1+0) present_at_req\n");
	unsigned n_berr = 0, n_wordbad = 0, n_longbad = 0, n_berr_notfirst = 0, n_berr_wrong = 0, n_nober_wrong = 0, n_term_odd = 0;
	std::string first_bad;
	for (size_t i = 0; i < cirs.size(); i++) {
		const Cir &c = cirs[i];
		bool first_in_instr = i == 0 || cirs[i - 1].instr != c.instr;
		// (re-reads of the save CIR within one FSAVE - the come-again polls - are not first accesses: no BERR on them)
		bool firsttype = (!c.rw && (c.off == 0x0A || c.off == 0x0E || c.off == 0x06)) || (c.rw && c.off == 0x04 && first_in_instr);
		bool lng = c.off == 0x10 || c.off == 0x18 || c.off == 0x1C;
		const char *term = c.berr ? "BERR" : (c.ds0 && c.ds1) ? "DSACK1+0" : c.ds1 ? "DSACK1" : "DSACK0";
		if (lf && i < 60000)
			fprintf(lf, "%llu %llu %llu cp%u $%02X %s %08X %s %d\n", (unsigned long long)c.clk0, (unsigned long long)c.clk1,
			        (unsigned long long)c.instr, c.id, c.off, c.rw ? "R" : "W", c.data, term, c.present);
		auto bad = [&](const char *why) {
			if (first_bad.empty())
				first_bad = fmt("%s: clk %llu cpid %u off $%02X %s data %08X term %s present %d", why, (unsigned long long)c.clk0, c.id,
				                c.off, c.rw ? "read" : "write", c.data, term, c.present);
		};
		if (c.berr) {
			n_berr++;
			if (c.ds0 || c.ds1) { n_term_odd++; bad("BERR together with DSACK"); }
			if (!first_in_instr) { n_berr_notfirst++; bad("BERR not on the first CIR access of the instruction"); }
			if (c.id == 1 && !(firsttype && !c.present)) { n_berr_wrong++; bad("CpID 1 BERR although FPU present or access not a first-type access"); }
		} else {
			if (c.id != 1) { n_nober_wrong++; bad("CpID != 1 access not ended with BERR"); }
			else if (firsttype && !c.present) { n_nober_wrong++; bad("first-type access completed although FPU absent"); }
			if (lng && !(c.ds0 && c.ds1)) { n_longbad++; bad("long CIR not terminated with DSACK1+DSACK0"); }
			if (!lng && !(c.ds1 && !c.ds0)) { n_wordbad++; bad("word CIR not terminated with DSACK1 only"); }
		}
	}
	if (lf) fclose(lf);
	printf("  bus log: %zu CIR cycles (%u BERR) in %s\n", cirs.size(), n_berr, path);
	const char *S = "MC68030 UM 10 (CIR protocol) / MC68881 UM 7 (CIR sizes) / falcon_cpubus.sv header";
	check_eq("bus", "CIR cycles ended in BERR anywhere but the first CIR access of an instruction", 0, n_berr_notfirst, S, 1);
	check_eq("bus", "BERR together with DSACK on the same cycle", 0, n_term_odd, S, 1);
	check_eq("bus", "CpID 1 cycles with BERR while the FPU is present (or not first-type)", 0, n_berr_wrong, S, 1);
	check_eq("bus", "cycles whose BERR/DSACK outcome contradicts presence / CpID", 0, n_nober_wrong, S, 1);
	check_eq("bus", "word CIR cycles not ended with DSACK1 only", 0, n_wordbad, "MC68881 UM 7.1 CIR widths: 16-bit except operand/IA/OA", 1);
	check_eq("bus", "$10/$18/$1C CIR cycles not ended with DSACK1+DSACK0", 0, n_longbad, "MC68881 UM 7.1 CIR widths: operand, instruction address, operand address are 32-bit", 1);
	check_eq("bus", "BERR on any non-CIR cycle (RAM/ROM)", 0, other_berr, "falcon_cpubus: RAM/ROM never ends in BERR here", 1);
	if (!first_bad.empty()) printf("        first offender: %s\n", first_bad.c_str());

	// frame transfers (MC68030 UM 10.5.4 / 10.5.5): after the save CIR read the
	// processor moves the frame body, size = length byte of the format word,
	// through the operand CIR in longs; cpRESTORE writes the same amount.
	unsigned n_saves = 0, n_rests = 0, n_frm_bad = 0;
	std::string frm_bad;
	for (size_t i = 0; i < cirs.size();) {
		size_t j = i;
		while (j < cirs.size() && cirs[j].instr == cirs[i].instr) j++;
		Cir f = cirs[i];
		for (size_t k = i; k < j; k++)                // FSAVE: the last save CIR read carries the frame format (earlier ones may be come-again $0118)
			if (cirs[k].rw && cirs[k].off == 0x04 && f.rw && f.off == 0x04) f = cirs[k];
		if (f.id == 1 && !f.berr && ((f.rw && f.off == 0x04) || (!f.rw && f.off == 0x06))) {
			unsigned fmtw = (f.data >> 16) & 0xFFFF;
			unsigned nops = 0;
			for (size_t k = i + 1; k < j; k++)
				if (cirs[k].off == 0x10 && cirs[k].rw == f.rw) nops++;
			// valid frames: null (version $00): no body; version $1F with size $38/$18/$D4/$B4 (Hatari fpuop_restore):
			// size/4 longs; anything else is refused (format error), no body
			unsigned sz = fmtw & 0xFF;
			unsigned exp_ops = (fmtw >> 8) == 0 ? 0 : ((fmtw >> 8) == 0x1F && (sz == 0x38 || sz == 0x18 || sz == 0xD4 || sz == 0xB4)) ? sz / 4 : 0;
			if (f.rw) n_saves++; else n_rests++;
			if (nops != exp_ops) {
				n_frm_bad++;
				if (frm_bad.empty())
					frm_bad = fmt("clk %llu %s format word %04X: %u operand CIR transfers, expected %u", (unsigned long long)f.clk0,
					              f.rw ? "FSAVE" : "FRESTORE", fmtw, nops, exp_ops);
			}
		}
		i = j;
	}
	// exception primitives (MC68030 UM 10.5.2): a "take exception" response ($1C..$1E in bits 12-8) is acknowledged with a write
	// to the control CIR ($02); with the PC bit (bit 14, e.g. $5C30 for BSUN) the CPU writes the instruction address to CIR $18 first
	unsigned n_prim = 0, n_prim_pc = 0, n_prim_bad = 0;
	std::string prim_bad;
	for (size_t i = 0; i < cirs.size();) {
		size_t j = i;
		while (j < cirs.size() && cirs[j].instr == cirs[i].instr) j++;
		for (size_t k = i; k < j; k++) {
			const Cir &c = cirs[k];
			unsigned hi = c.data >> 16;
			if (c.id != 1 || !c.rw || c.off != 0x00 || c.berr || (((hi >> 8) & 0x1F) < 0x1C || ((hi >> 8) & 0x1F) > 0x1E)) continue;
			bool pc = (hi >> 14) & 1;
			n_prim++;
			if (pc) n_prim_pc++;
			bool ia = false, xa = false;
			for (size_t m = k + 1; m < j; m++) {
				if (!cirs[m].rw && cirs[m].off == 0x18 && !xa) ia = true;
				if (!cirs[m].rw && cirs[m].off == 0x02) { xa = true; break; }
			}
			if (!xa || ia != pc) {
				n_prim_bad++;
				if (prim_bad.empty()) prim_bad = fmt("clk %llu response %04X: CIR $18 written %d (PC bit %d), exception acknowledge %d", (unsigned long long)c.clk0, hi, ia, pc, xa);
			}
		}
		i = j;
	}
	g_prim_pc_seen = n_prim_pc;
	check_eq("bus", fmt("take-exception primitives ($1C-$1E): %u seen, %u with the PC bit; wrong IA/XA dialogue", n_prim, n_prim_pc), 0, n_prim_bad,
	         "MC68030 UM 10.5.2: PC bit -> instruction address to CIR $18; exception acknowledge to the control CIR; bridge header", 1);
	if (!prim_bad.empty()) printf("        first: %s\n", prim_bad.c_str());

	// refused opmodes (bridge header): opclass 000/010 command words with opmode $6E-$77 are answered with the F-line
	// primitive ($1C0B), $78-$7F with vector 4 ($1C04), before any operand is transferred; while an earlier
	// instruction still executes the response is come-again ($8900) first (bridge header "Busy", MC68881 UM 7.2.6)
	unsigned n_ref = 0, n_ref_bad = 0;
	std::string ref_bad;
	for (size_t i = 0; i < cirs.size();) {
		size_t j = i;
		while (j < cirs.size() && cirs[j].instr == cirs[i].instr) j++;
		const Cir &w = cirs[i];
		unsigned cmdw = (w.data >> 16) & 0xFFFF;
		unsigned opm = cmdw & 0x7F, cls = cmdw >> 13;
		bool fmovecr = cls == 2 && ((cmdw >> 10) & 7) == 7;
		if (w.id == 1 && !w.rw && w.off == 0x0A && !w.berr && (cls == 0 || cls == 2) && !fmovecr && opm >= 0x6E) {
			n_ref++;
			unsigned want = opm < 0x78 ? 0x1C0B : 0x1C04, gotr = 0xFFFF;
			bool operand = false;
			for (size_t k = i + 1; k < j; k++) {
				if (cirs[k].off == 0x10) operand = true;
				if (cirs[k].off == 0x00 && cirs[k].rw && gotr == 0xFFFF && (cirs[k].data >> 16) != 0x8900)
					gotr = cirs[k].data >> 16;
			}
			if (operand || gotr != want) {
				n_ref_bad++;
				if (ref_bad.empty()) ref_bad = fmt("clk %llu cmd %04X: response %04X expected %04X, operand transfer %d", (unsigned long long)w.clk0, cmdw, gotr, want, operand);
			}
		}
		i = j;
	}
	check_eq("bus", fmt("refused opmodes ($6E-$7F, %u instructions): not answered with $1C0B/$1C04 (after any come-again) or with an operand transfer", n_ref), 0, n_ref_bad,
	         "bridge header: $6E-$77 F-line ($1C0B), $78-$7F vector 4 ($1C04), before any operand, come-again while busy; Hatari fault_if_nonexisting_opmode", 1);
	if (!ref_bad.empty()) printf("        first: %s\n", ref_bad.c_str());
	check_eq("bus", fmt("FSAVE/FRESTORE dialogues (%u saves, %u restores) whose operand CIR transfer count differs from the frame size", n_saves, n_rests), 0,
	         n_frm_bad, "MC68030 UM 10.5.4/10.5.5: body = size byte of the format word ($38 = 14 longs; version $00 = null: none); Hatari accepted sizes $38/$18/$D4/$B4", 1);
	if (!frm_bad.empty()) printf("        first: %s\n", frm_bad.c_str());
}

static void ddr_checks()
{
	check_eq("ddr", "DDRAM protocol violations (one command at a time, burst 1 or a read of 4, guest window)", 0, ddr_proto_err,
	         "MiSTer DDRAM semantics / falcon_memarb", 1);
	if (ddr_proto_err) printf("        first: %s\n", ddr_first_err.c_str());
}

static void presence_checks(bool strict_polls)
{
	check_eq("presence", "cycles where `present` differs from the header's rule (MAGIC ok and HB changed within 20 polls)", 0, sb_mismatch,
	         "falcon_fpu_bridge.sv header: present while HEARTBEAT changed within the last 20 polls", 1);
	if (sb_mismatch) printf("        first: %s\n", sb_first_mismatch.c_str());
	// poll period: CLK_HZ/200 clocks between the starts of successive polls
	if (strict_polls && req_start_magic.size() > 2) {
		uint64_t mn = ~0ull, mx = 0;
		for (size_t i = 1; i < req_start_magic.size(); i++) {
			uint64_t d = req_start_magic[i] - req_start_magic[i - 1];
			if (d < mn) mn = d;
			if (d > mx) mx = d;
		}
		check_true("presence", fmt("poll period min %llu max %llu clocks within [%u, %u] (CLK_HZ/200 = %u plus two DMA reads)",
		                          (unsigned long long)mn, (unsigned long long)mx, POLL_CLKS, POLL_CLKS + 400, POLL_CLKS),
		           mn >= POLL_CLKS && mx <= POLL_CLKS + 400, "bridge header: polled every 5 ms = CLK_HZ/200 clocks");
	}
}

// standard end of a program run
static bool finish_program(uint64_t max_clks, bool allow_unexp = false)
{
	bool ok = run_until(is_done, max_clks);
	check_true("run", fmt("program reached its done marker within %llu clocks (clk %llu, %llu instructions)", (unsigned long long)max_clks,
	                      (unsigned long long)g_clk, (unsigned long long)instr_idx), ok, "test harness");
	if (T->o_halted) check_true("run", "CPU not halted (no double bus fault)", false, "MC68030 UM 8.1.3");
	if (!allow_unexp) {
		check_eq("run", "unexpected-vector exceptions taken (UNEXP flag)", 0, gr32(A_UNEXP), "test harness", 1);
		if (gr32(A_UNEXP)) {
			for (auto &e : exlog()) printf("        exception: vector %u fmt %u pc %08x ia %08x sr %04x\n", e.vec, e.fmt, e.pc, e.ia, e.sr);
		}
	}
	collect();
	return ok;
}

//------------------------------------------------------------------ the condition table
// MC68881/MC68882 UM, "Conditional Tests" (table 3-21 predicates): with
// N = Z = I = NAN = 0.  Written out from the formulas:
//   $00 F 0   $01 EQ Z=0   $02 OGT !(NAN|Z|N)=1   $03 OGE Z|!(NAN|N)=1
//   $04 OLT N&!(NAN|Z)=0   $05 OLE Z|(N&!NAN)=0   $06 OGL !(NAN|Z)=1   $07 OR !NAN=1
//   $08 UN NAN=0   $09 UEQ NAN|Z=0   $0A UGT NAN|!(N|Z)=1   $0B UGE NAN|Z|!N=1
//   $0C ULT NAN|(N&!Z)=0   $0D ULE NAN|Z|N=0   $0E NE !Z=1   $0F T 1
//   $10-$1F: the IEEE-nonaware forms give the same results (BSUN only if NAN)
static const int TF[32] = {0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1,
                           0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 1, 1};
static const char *PRED_NAME[32] = {"F", "EQ", "OGT", "OGE", "OLT", "OLE", "OGL", "OR", "UN", "UEQ", "UGT", "UGE", "ULT", "ULE", "NE", "T",
                                    "SF", "SEQ", "GT", "GE", "LT", "LE", "GL", "GLE", "NGLE", "NGL", "NLE", "NLT", "NGE", "NGT", "SNE", "ST"};

// cross-check the table with Hatari's condition_table_6888x (first 32 entries = all cc clear)
static void hatari_crosscheck()
{
	const char *p = getenv("HATARI_FPP");
	std::string path = p ? p : "/home/pkilar/Devel/Atari/hatari/src/cpu/fpp.c";
	FILE *f = fopen(path.c_str(), "r");
	if (!f) { printf("  SKIP  [cond] Hatari cross-check: %s not readable\n", path.c_str()); return; }
	char line[1024];
	bool in = false;
	std::vector<int> v;
	while (fgets(line, sizeof line, f) && v.size() < 32) {
		if (strstr(line, "condition_table_6888x[]")) { in = true; continue; }
		if (!in) continue;
		for (char *c = line; *c; c++)
			if (*c == '0' || *c == '1') v.push_back(*c - '0');
	}
	fclose(f);
	bool ok = v.size() >= 32;
	std::string mine, theirs;
	for (int i = 0; i < 32 && ok; i++) {
		mine += char('0' + TF[i]);
		theirs += char('0' + v[i]);
		if (TF[i] != v[i]) ok = false;
	}
	report(ok, "cond", fmt("this bench's predicate table vs Hatari condition_table_6888x row 0: expected %s got %s", mine.c_str(), theirs.c_str()),
	       "hatari/src/cpu/fpp.c fpp_cond");
}

//------------------------------------------------------------------ scenarios
static const char *S_COOKIE = "EmuTOS bios/processor.S _detect_fpu header: 0x00060000 = 68882 for sure, 0 = no FPU";

static void sc_detect(const char *name, const char *mode)
{
	begin_test(name, "t_detect");
	bool real = !strcmp(mode, "real");
	uint32_t exp = 0;
	if (real) exp = 0x00060000;
	if (real) { set_pacing(true); child_start(); }
	else if (!strcmp(mode, "badmagic")) { hb_start(); mb_w16(0x000, 0x4651); }
	else if (!strcmp(mode, "hb0")) { mb_w16(0x004, VERSION); mb_w16(0x002, 0); mb_w16(0x000, MAGIC); }
	else if (!strcmp(mode, "badversion")) { hb_start(); mb_w16(0x004, 1); }
	release_reset();
	if (exp) {
		bool ok = run_until(present, 6 * POLL_CLKS + 5000);
		check_true("presence", fmt("`present` rises within a few polls of the service starting (clk %llu)", (unsigned long long)g_clk), ok,
		           "bridge header: polled every 5 ms");
	} else
		steps(5 * POLL_CLKS);
	if (!exp) check_eq("presence", "`present` with no usable service", 0, present(), "bridge header: no MAGIC / heartbeat -> absent", 1);
	steps(200);
	gw32(A_GO, 1);
	finish_program(300000);
	expect("detect", Tg("T_COOKIE"), fmt("_detect_fpu cookie (%s)", mode), exp, S_COOKIE);
	if (!exp) {
		// no FPU: the very first CIR access (FRESTORE's restore CIR write) ends in BERR, and nothing follows it
		check_eq("bus", "CIR cycles of the whole detection (just the failing FRESTORE access)", 1, cirs.size(),
		         "MC68030 UM 10: BERR on the first CIR access = no coprocessor", 1);
		if (cirs.size() >= 1) {
			check_eq("bus", "that access is a write to the restore CIR ($06)", 0x106, (cirs[0].rw ? 0x200u : 0x100u) | cirs[0].off,
			         "MC68881 UM 7.1.1 / MC68030 UM 10.5.5: cpRESTORE writes the restore CIR first", 3);
			check_eq("bus", "that access ended in BERR (1 = BERR)", 1, cirs[0].berr, "bridge header: first CIR access without service -> BERR", 1);
		}
	}
	if (real) {
		check_eq("service", "MAGIC word the real falcon_fpu wrote at mailbox +0", MAGIC, mb16(0), "tools/falcon_fpu header: MAGIC $4650", 4);
		check_eq("service", "VERSION word the real falcon_fpu wrote at mailbox +4", VERSION, mb16(4), "tools/falcon_fpu: FPU_VERSION 4 (bridge header: VERSION 4)", 4);
		check_true("service", fmt("HEARTBEAT advanced (now %u)", mb16(2)), mb16(2) != 0, "tools/falcon_fpu: +2 incremented every 10 ms");
		child_reap();
		check_true("service", "falcon_fpu_host exited cleanly on SIGTERM", child_exited && WIFEXITED(child_status) && WEXITSTATUS(child_status) == 0,
		           "tools/falcon_fpu: SIGTERM -> clears MAGIC, exit 0");
		check_eq("service", "MAGIC after SIGTERM", 0, mb16(0), "tools/falcon_fpu: clears MAGIC on SIGTERM", 4);
	}
	bus_checks(fmt("detect_%s", mode).c_str());
	ddr_checks();
	presence_checks(false);
}

// FNOP watch loop: STATUS follows presence
static uint32_t status_now() { return ddr[A_STATUS]; }

struct Watch {
	uint64_t t_status0 = 0;   // clock STATUS first became 0 after being 1
};

static bool watch_cpu = false;
static void watch_begin(const char *name, bool real)
{
	watch_cpu = real;
	begin_test(name, "t_watch");
	if (real) { set_pacing(true); child_start(); }
	else hb_start();
	release_reset();
	bool ok = run_until(present, 8 * POLL_CLKS);
	check_true("presence", fmt("`present` rises at the first poll that sees MAGIC and a new heartbeat (clk %llu)", (unsigned long long)g_clk), ok,
	           "bridge header: presence via mailbox heartbeat");
	if (!real) return;      // the C++ heartbeat writer cannot answer the reset request: a CPU instruction would wait for it
	ok = run_until(is_ready, 100000);
	check_true("run", fmt("program waiting for GO (clk %llu)", (unsigned long long)g_clk), ok, "test harness");
	gw32(A_GO, 1);
	ok = run_until([] { return status_now() == 1; }, 6000);
	check_eq("watch", "FRESTORE probe loop STATUS while the service is alive (1 = completes)", 1, status_now(), "MC68882 UM: FRESTORE completes", 1);
}

// run until `present` falls (or timeout); return clock of the fall, 0 if none
static uint64_t wait_fall(uint64_t max)
{
	size_t n = pr_falls.size();
	run_until([&] { return pr_falls.size() > n; }, max);
	return pr_falls.size() > n ? pr_falls.back() : 0;
}
static uint64_t wait_rise(uint64_t max)
{
	size_t n = pr_rises.size();
	run_until([&] { return pr_rises.size() > n; }, max);
	return pr_rises.size() > n ? pr_rises.back() : 0;
}

static void watch_after_fall(uint64_t fall, const char *why)
{
	// the CPU-visible result follows: STATUS goes to 0 (Line-F), never before presence fell
	uint64_t st1 = 0;
	for (int i = 0; i < 4000 && status_now() == 1; i++) step();
	st1 = g_clk;
	check_eq("watch", fmt("FNOP loop STATUS after `present` fell (%s): 0 = Line-F", why), 0, status_now(), "MC68030 UM 10: BERR on first CIR access = no coprocessor", 1);
	check_true("watch", fmt("STATUS became 0 %lld clocks after `present` fell (not before)", (long long)(st1 - fall)), st1 >= fall,
	           "no premature absence");
}

static void sc_watch_cpp_freeze()
{
	watch_begin("presence/C++ heartbeat frozen", false);
	steps(3 * HB_PERIOD);
	check_eq("presence", "`present` while the heartbeat runs", 1, present(), "bridge header", 1);
	size_t nf = pr_falls.size();
	hb_on = false;                                  // freeze: the last value stays in the mailbox
	uint64_t t_frozen = mb_hb_change_clk;           // clock of the last heartbeat write
	// still present until the alive window expires
	uint64_t fall = wait_fall(40 * POLL_CLKS);
	check_true("presence", fmt("`present` falls after the freeze (fall at clk %llu)", (unsigned long long)fall), pr_falls.size() > nf, "bridge header: absent after 20 polls");
	if (fall) {
		double ms = (double)(fall - t_frozen) * 1000.0 / CLK_HZ;
		// the first poll after the last write (<= 1 period) sees the new value, then 20 polls without change
		check_true("presence", fmt("freeze -> absent took %.1f ms (%llu clocks); expected 100..110 ms (20 polls of 5 ms, plus up to one poll and the DMA reads)", ms,
		                          (unsigned long long)(fall - t_frozen)),
		           fall - t_frozen >= 19 * POLL_CLKS && fall - t_frozen <= 22 * POLL_CLKS, "bridge header: 5 ms polls, absent after 20 polls without a new heartbeat");
		check_eq("presence", "HEARTBEAT polls without change when `present` fell", ALIVE_POLLS, pr_fall_unch.back(), "bridge header: absent after 20 polls", 2);
		// it was present just before
		if (watch_cpu) watch_after_fall(fall, "heartbeat frozen");
	}
	// heartbeat resumes: present again at the next poll that sees a change
	hb_on = true; hb_next = g_clk + HB_PERIOD;
	uint64_t t_resume = g_clk;
	uint64_t rise = wait_rise(6 * POLL_CLKS + HB_PERIOD);
	// the first new heartbeat is written HB_PERIOD after resuming; the next poll comes within POLL_CLKS and
	// needs its three mailbox reads (random DDR latency here, well under POLL_CLKS / 10); a missed poll
	// would add a whole POLL_CLKS
	check_true("presence", fmt("`present` rises again %llu clocks after the heartbeat resumed (limit %u)", (unsigned long long)(rise - t_resume),
	                           HB_PERIOD + POLL_CLKS + POLL_CLKS / 10),
	           rise != 0 && rise - t_resume <= HB_PERIOD + POLL_CLKS + POLL_CLKS / 10,
	           "bridge header: HEARTBEAT changed -> present at the first poll that sees it");
	if (watch_cpu) run_until([] { return status_now() == 1; }, 6000);
	if (watch_cpu) check_eq("watch", "FNOP loop STATUS after the heartbeat resumed", 1, status_now(), "MC68882 UM: FNOP completes", 1);
	if (watch_cpu) { gw32(A_STOP, 1); finish_program(30000); }
	bus_checks("watch_freeze");
	ddr_checks();
	presence_checks(true);
}

static void sc_watch_cpp_magic()
{
	watch_begin("presence/C++ MAGIC cleared", false);
	steps(2 * HB_PERIOD);
	size_t nf = pr_falls.size();
	mb_w16(0x000, 0);                                // what falcon_fpu does on SIGTERM
	uint64_t t0 = g_clk;
	uint64_t fall = wait_fall(3 * POLL_CLKS);
	check_true("presence", fmt("`present` falls on the next poll after MAGIC is cleared (%llu clocks later; limit %u)", (unsigned long long)(fall - t0), POLL_CLKS + 400),
	           pr_falls.size() > nf && fall - t0 <= POLL_CLKS + 400, "bridge header: polls every 5 ms; no MAGIC -> absent");
	if (fall) if (watch_cpu) watch_after_fall(fall, "MAGIC cleared");
	mb_w16(0x000, MAGIC);
	uint64_t t1 = g_clk;
	uint64_t rise = wait_rise(3 * HB_PERIOD);
	check_true("presence", fmt("`present` returns %llu clocks after MAGIC is back (heartbeat still running)", (unsigned long long)(rise - t1)), rise != 0 && rise - t1 <= POLL_CLKS + 400,
	           "bridge header: MAGIC ok and HEARTBEAT changed -> present");
	if (watch_cpu) run_until([] { return status_now() == 1; }, 6000);
	if (watch_cpu) check_eq("watch", "FNOP loop STATUS after MAGIC returned", 1, status_now(), "MC68882 UM: FNOP completes", 1);
	if (watch_cpu) { gw32(A_STOP, 1); finish_program(30000); }
	bus_checks("watch_magic");
	ddr_checks();
	presence_checks(true);
}

static void sc_watch_real()
{
	watch_begin("presence/real falcon_fpu_host: SIGSTOP, SIGCONT, SIGTERM", true);
	steps(2 * HB_PERIOD);
	check_eq("presence", "`present` while the real service runs", 1, present(), "tools/falcon_fpu: heartbeat every 10 ms", 1);
	// --- freeze with SIGSTOP
	size_t nf = pr_falls.size();
	child_signal(SIGSTOP);
	steps(4);
	uint64_t t_frozen = mb_hb_change_clk;           // last heartbeat the ARM program wrote
	uint64_t fall = wait_fall(40 * POLL_CLKS);
	check_true("presence", fmt("`present` falls after SIGSTOP (fall at clk %llu)", (unsigned long long)fall), pr_falls.size() > nf, "bridge header: absent after 20 polls");
	if (fall) {
		double ms = (double)(fall - t_frozen) * 1000.0 / CLK_HZ;
		check_true("presence", fmt("last heartbeat -> absent took %.1f ms (%llu clocks); expected 100..110 ms", ms, (unsigned long long)(fall - t_frozen)),
		           fall - t_frozen >= 19 * POLL_CLKS && fall - t_frozen <= 22 * POLL_CLKS, "bridge header: absent after 20 polls without a new heartbeat");
		check_eq("presence", "HEARTBEAT polls without change when `present` fell", ALIVE_POLLS, pr_fall_unch.back(), "bridge header", 2);
		watch_after_fall(fall, "SIGSTOP");
	}
	// --- SIGCONT: the heartbeat moves again
	child_signal(SIGCONT);
	uint64_t t_cont = g_clk;
	uint64_t rise = wait_rise(6 * POLL_CLKS + HB_PERIOD);
	check_true("presence", fmt("`present` rises %llu clocks after SIGCONT", (unsigned long long)(rise - t_cont)), rise != 0 && rise - t_cont <= 4 * POLL_CLKS,
	           "bridge header: HEARTBEAT changed -> present");
	run_until([] { return status_now() == 1; }, 6000);
	check_eq("watch", "FNOP loop STATUS after SIGCONT", 1, status_now(), "MC68882 UM: FNOP completes", 1);
	steps(2 * HB_PERIOD);
	// --- SIGTERM: MAGIC is cleared at once, the FPU disappears at the next poll
	nf = pr_falls.size();
	child_signal(SIGTERM);
	uint64_t te = g_clk;
	run_until([] { return child_exited; }, 4 * POLL_CLKS);
	check_true("service", "falcon_fpu_host exited after SIGTERM", child_exited && WIFEXITED(child_status) && WEXITSTATUS(child_status) == 0, "tools/falcon_fpu");
	check_eq("service", "MAGIC after SIGTERM", 0, mb16(0), "tools/falcon_fpu: SIGTERM -> MAGIC cleared", 4);
	uint64_t tz = mb_magic_zero_clk;
	fall = wait_fall(3 * POLL_CLKS);
	check_true("presence", fmt("`present` falls %lld clocks after MAGIC was cleared (SIGTERM at clk %llu); limit one poll = %u",
	                          (long long)(fall - tz), (unsigned long long)te, POLL_CLKS + 400),
	           pr_falls.size() > nf && fall >= tz && fall - tz <= POLL_CLKS + 400, "bridge header: absent at the next poll");
	if (fall) watch_after_fall(fall, "SIGTERM");
	child_reap();
	gw32(A_STOP, 1);
	finish_program(30000);
	bus_checks("watch_real");
	ddr_checks();
	presence_checks(true);
}

// program that executes FPU instructions: needs the real ARM service (the C++
// heartbeat writer cannot answer requests)
static void run_alive_program(const char *name, const char *prog, uint64_t max_clks, bool allow_unexp = false)
{
	begin_test(name, prog);
	set_pacing(true);
	child_start();
	release_reset();
	bool ok = run_until(present, 20 * POLL_CLKS);
	check_true("presence", fmt("`present` rises with the real service (clk %llu)", (unsigned long long)g_clk), ok, "bridge header");
	run_until(is_ready, 400000);
	steps(2000);
	gw32(A_GO, 1);
	finish_program(max_clks, allow_unexp);
}

static void sc_frames()
{
	run_alive_program("frames/FSAVE FRESTORE", "t_frames", 600000);
	const char *G = "frame";
	const char *HN = "Hatari fpp.c fpuop_save: a null frame is frame_id = (frame_size-4)<<16 = $0038xxxx for the 68882 (frame_size $3C); version byte $00 = null (MC68881/2 UM 6.4.2); bridge header";
	const char *UM_IDLE = "MC68882 UM 6.4.2: idle frame format $1F38 (version $1F, size $38) = 4 + 56 = 60 bytes; EmuTOS processor.S tests byte 1 = $38";
	expect(G, Tg("T_FS_NULL_DELTA"), "FSAVE -(a0) after reset: bytes pushed (null frame: format long only)", 4, HN);
	expect(G, Tg("T_FS_NULL_FMT"), "FSAVE after reset: format word", 0x0038, HN, 4);
	expect(G, Tg("T_FS_NULL_BEYOND"), "memory above the null frame untouched (pre-fill $AAAAAAAA)", 0xAAAAAAAA, HN);
	expect(G, Tg("T_FS_IDLE_DELTA"), "FNOP; FSAVE -(a0): bytes pushed", 60, UM_IDLE);
	expect(G, Tg("T_FS_IDLE_FMT"), "FNOP; FSAVE -(a0): format word (version $1F, size $38)", 0x1F38, UM_IDLE, 4);
	expect(G, Tg("T_FS_IDLE_LEN"), "FNOP; FSAVE -(a0): byte at offset 1 (EmuTOS 68882 test)", 0x38, UM_IDLE, 2);
	{   // the body is Hatari's: the golden FSAVE after a reset and an FNOP (null -> idle)
		uint8_t gf[256];
		int gn = gold_idle_frame(gf);
		check_eq(G, "golden idle frame size (reset; FNOP; FSAVE)", 60, gn, "Hatari fpp.c fpuop_save", 2);
		for (int n = 0; n < 14; n++) {
			uint32_t e = ((uint32_t)gf[4 + 4 * n] << 24) | ((uint32_t)gf[5 + 4 * n] << 16) | ((uint32_t)gf[6 + 4 * n] << 8) | gf[7 + 4 * n];
			uint32_t msk = 0xFFFFFFFFu;
			if (n == 0) msk = 0xFFC0FFFFu;      // CCR long: the engine's opcode word carries a synthetic EA (low 6 bits)
			auto it = recs.find(Tg("T_FS_IDLE_BODY") + n);
			uint32_t got = it == recs.end() || it->second.size() != 1 ? 0xDEADBEEFu : it->second[0];
			check_eq(G, fmt("idle frame body long %d (frame offset %d)%s", n, 4 + 4 * n, n == 0 ? ", EA field of the opcode half masked (engine opcode has a synthetic EA)" : ""),
			         e & msk, got & msk, "Hatari fpp.c fpuop_save golden image", 8);
		}
	}
	expect(G, Tg("T_FS_IDLE_BEYOND"), "memory above the 60-byte idle frame untouched", 0xAAAAAAAA, UM_IDLE);
	expect(G, Tg("T_FS_CTRL_FMT"), "FNOP; FSAVE (a0): format word", 0x1F38, UM_IDLE, 4);
	expect(G, Tg("T_FS_CTRL_A0"), "FSAVE (a0) leaves a0 unchanged (1 = yes)", 1, "MC68030 UM 10.5.4 (control addressing mode)", 1);
	expect(G, Tg("T_FS_CTRL_BEYOND"), "memory 60 bytes above a0 untouched after FSAVE (a0)", 0xAAAAAAAA, UM_IDLE);
	expect(G, Tg("T_FR_NULL_FMT"), "FNOP; FRESTORE null; FSAVE: format word (null again)", 0x0038, HN, 4);
	expect(G, Tg("T_FR_NULL_DELTA"), "... and the null frame is 4 bytes", 4, HN);
	expect(G, Tg("T_FR_NULL_A1"), "FRESTORE (a1) leaves a1 unchanged (1 = yes)", 1, "MC68030 UM 10.5.5", 1);
	expect(G, Tg("T_FR_NULL_ADV"), "FRESTORE (a1)+ of a null frame advances a1 by", 4, "MC68881 UM 6.4.2 null frame length; Hatari fpuop_restore");
	expect(G, Tg("T_FR_NZ_CNT"), "FRESTORE of $0012 (version 0 = null): exceptions taken", 0, "Hatari fpp.c fpuop_restore: frame_version 0 is a null frame");
	expect(G, Tg("T_FR_NZ_FMT"), "... FSAVE afterwards: format word", 0x0038, HN, 4);
	expect(G, Tg("T_FR_IDLE_ADV"), "FRESTORE (a1)+ of an idle 68882 frame: bytes consumed", 60, UM_IDLE);
	expect(G, Tg("T_FR_IDLE_CNT"), "... exceptions taken", 0, "MC68882 UM 6.4.2: a valid idle frame restores");
	expect(G, Tg("T_FR_IDLE_FMT"), "... FSAVE afterwards (from null): format word", 0x1F38, "MC68882 UM 6.4.2: restore of an idle frame leaves the FPU idle", 4);
	expect(G, Tg("T_FR_IDLE_DELTA"), "... and its size", 60, UM_IDLE);
	{
		uint32_t b0 = Tg("T_BADF");
		const char *S = "MC68030 UM 10.5.5 / Table 8-1: invalid format word -> format error, vector 14 (Hatari fpuop_restore Exception(14))";
		expect(G, b0 + 0, "FRESTORE of frame $1234: exceptions taken", 1, S, 1);
		expect(G, b0 + 1, "FRESTORE of frame $1234: exception vector (format error)", 14, S, 2);
		expect(G, b0 + 2, "FRESTORE of frame $1234: stack frame format (pre-instruction = 0)", 0, "MC68030 UM 10.4.2: pre-instruction exception, format 0", 1);
		expect(G, b0 + 3, "FRESTORE of frame $1234: stacked PC is the FRESTORE (1 = yes)", 1, "MC68030 UM 10.4.2", 1);
		expect(G, b0 + 4, "FRESTORE of frame $1234: execution continued after the handler (1 = yes)", 1, "test harness", 1);
	}
	expect(G, Tg("T_FR_AFTERBAD"), "FNOP after the format error: exceptions taken", 0, "MC68882 UM: FNOP completes");
	{
		static const struct { const char *n; uint32_t adv; } odd[3] = {{"$1F18 (68881 idle)", 4 + 0x18}, {"$1FD4 (busy)", 4 + 0xD4}, {"$1FB4 (busy)", 4 + 0xB4}};
		for (unsigned i = 0; i < 3; i++) {
			const char *S = "Hatari fpp.c fpuop_restore 6888x: version $1F frames of size $18/$38 (idle) and $B4/$D4 (busy, skipped) are accepted; the 68030 moves 'length' bytes after the format word";
			expect(G, Tg("T_ODDF") + 4 * i, fmt("FRESTORE of frame %s: exceptions taken", odd[i].n), 0, S, 1);
			expect(G, Tg("T_ODDF") + 4 * i + 1, fmt("FRESTORE (a1)+ of frame %s: bytes consumed", odd[i].n), odd[i].adv, S, 3);
		}
	}
	// the exception log must hold exactly the one vector-14 entry
	auto ex = exlog();
	check_eq(G, "exceptions in the whole program (the format error of $1234)", 1, ex.size(), "test harness", 1);
	bus_checks("frames");
	ddr_checks();
	presence_checks(false);
}

static void sc_cond()
{
	run_alive_program("conditionals/FScc FBcc FDBcc FTRAPcc", "t_cond", 3000000);
	const char *G = "cond";
	hatari_crosscheck();
	const char *UM = "MC68881/2 UM conditional tests (table in sec. 3); FPSR cc = 0 after reset, cross-checked with Hatari fpp_cond";
	expect(G, Tg("T_FNOP_CNT"), "3 x FNOP: exceptions taken", 0, "MC68882 UM: FNOP = FBF.W #0, completes");
	expect(G, Tg("T_FNOP_DONE"), "code after FNOP ran (1 = yes)", 1, "MC68882 UM", 1);
	for (unsigned p = 0; p < 32; p++) {
		int tf = TF[p];
		expect(G, Tg("T_FSCC") + p, fmt("FScc D1, predicate $%02X (%s) -> byte", p, PRED_NAME[p]), tf ? 0xFF : 0x00, UM, 2);
		expect(G, Tg("T_FBCC") + p, fmt("FBcc.W, predicate $%02X (%s): taken (1) / not taken (0)", p, PRED_NAME[p]), tf, UM, 1);
		expect(G, Tg("T_FDB_TAKEN") + p, fmt("FDBcc D3, predicate $%02X (%s): looped (1 = branch, i.e. condition false)", p, PRED_NAME[p]), !tf, UM, 1);
		expect(G, Tg("T_FDB_COUNT") + p, fmt("FDBcc D3 (=5), predicate $%02X (%s): D3 afterwards (5 if true, 4 if false)", p, PRED_NAME[p]), tf ? 5 : 4, UM, 1);
		expect(G, Tg("T_FTRAP") + p, fmt("FTRAPcc, predicate $%02X (%s): TRAPcc exceptions (1 = trapped)", p, PRED_NAME[p]), tf, UM, 1);
	}
	const char *U2 = "MC68881/2 UM FScc/FBcc/FTRAPcc; MC68030 UM 10.4 (cpScc/cpBcc/cpTRAPcc)";
	expect(G, Tg("T_FSCC_MEM_T"), "FST (a0): byte stored", 0xFF, U2, 2);
	expect(G, Tg("T_FSCC_MEM_F"), "FSF (a0): byte stored (pre-set $55)", 0x00, U2, 2);
	expect(G, Tg("T_FBCC_L_T"), "FBT.L: taken (1)", 1, U2, 1);
	expect(G, Tg("T_FBCC_L_F"), "FBF.L: taken (0 = not)", 0, U2, 1);
	expect(G, Tg("T_FTW_T"), "FTRAPT.W #imm: exceptions", 1, U2, 1);
	expect(G, Tg("T_FTW_VEC"), "FTRAPT.W: vector (TRAPcc)", 7, "MC68030 UM Table 8-1: vector 7 = cpTRAPcc", 2);
	expect(G, Tg("T_FTW_FMT"), "FTRAPT.W: stack frame format (post-instruction, six-word)", 2, "MC68030 UM 8.2.x: format $2", 1);
	expect(G, Tg("T_FTW_PC"), "FTRAPT.W: stacked PC is the next instruction, after the operand (1 = yes)", 1, "MC68030 UM 10.4.3", 1);
	expect(G, Tg("T_FTW_IA"), "FTRAPT.W: frame instruction address is the FTRAPT (1 = yes)", 1, "MC68030 UM 8.2.x format $2", 1);
	expect(G, Tg("T_FTW_F"), "FTRAPF.W #imm: exceptions (operand skipped)", 0, U2, 1);
	expect(G, Tg("T_FTL_T"), "FTRAPT.L #imm: exceptions", 1, U2, 1);
	expect(G, Tg("T_FTL_PC"), "FTRAPT.L: stacked PC after the long operand (1 = yes)", 1, "MC68030 UM 10.4.3", 1);
	expect(G, Tg("T_FTL_F"), "FTRAPF.L #imm: exceptions (operand skipped)", 0, U2, 1);
	check_eq(G, "exceptions in the whole program (TRAPcc taken: predicates true + 2 operand forms)", [] { int n = 0; for (int i = 0; i < 32; i++) n += TF[i]; return n + 2; }(),
	         exlog().size(), "predicate table + test harness", 2);
	for (auto &e : exlog())
		if (e.vec != 7) { check_true(G, fmt("unexpected exception vector %u", e.vec), false, "test harness"); break; }
	bus_checks("cond");
	ddr_checks();
	presence_checks(false);
}

static void sc_gen()
{
	run_alive_program("F-line from the bridge (reserved opclass, empty register list)", "t_gen", 600000);
	const char *G = "gen";
	const char *S = "bridge header: opclass 001 and an empty control register list answer F-line ($1C0B); MC68881 UM 7.? / MC68030 UM 10.4.2 pre-instruction exception";
	const char *names[4] = {"cpGEN command $2000 (opclass 001)", "cpGEN command $2400 (opclass 001)", "FMOVE <ea>,<empty list> ($8000)",
	                        "FMOVE <empty list>,<ea> ($A000)"};
	for (unsigned i = 0; i < 4; i++) {
		uint32_t b = Tg("T_GEN") + 4 * i;
		expect(G, b + 0, fmt("%s: exception vector (F-line)", names[i]), 11, S, 2);
		expect(G, b + 1, fmt("%s: stack frame format (pre-instruction)", names[i]), 0, S, 1);
		expect(G, b + 2, fmt("%s: stacked PC is the FPU instruction (1 = yes)", names[i]), 1, S, 1);
		expect(G, b + 3, fmt("%s: execution continued after the handler skipped it (1 = yes)", names[i]), 1, "test harness", 1);
	}
	expect(G, Tg("T_GEN_COUNT"), "exceptions taken in all", 4, S, 2);
	bus_checks("gen");
	ddr_checks();
	presence_checks(false);
}

static void cpid_checks(unsigned k, const char *G, const char *S)
{
	const char *kn[7] = {"cpGEN", "cpBcc", "cpScc", "cpDBcc", "cpTRAPcc.W", "cpSAVE (a0)", "cpRESTORE (a0)"};
	for (unsigned j = 0; j < 7; j++) {
		uint32_t b = Tg("T_CPID") + 32 * k + 4 * j;
		expect(G, b + 0, fmt("CpID %u %s: exception vector (F-line)", k, kn[j]), 11, S, 2);
		expect(G, b + 1, fmt("CpID %u %s: stack frame format (pre-instruction)", k, kn[j]), 0, S, 1);
		expect(G, b + 2, fmt("CpID %u %s: stacked PC is the instruction (1 = yes)", k, kn[j]), 1, S, 1);
		expect(G, b + 3, fmt("CpID %u %s: exactly one exception", k, kn[j]), 1, S, 1);
	}
}

static void sc_cpid(bool alive)
{
	if (alive) {
		begin_test("CpID 2..7 with the FPU present", "t_cpid");
		hb_start();
		release_reset();
		run_until(present, 6 * POLL_CLKS);
		check_eq("presence", "`present` (FPU alive, other CpIDs must still fail)", 1, present(), "bridge header", 1);
	} else {
		begin_test("CpID 2..7 with no service", "t_cpid");
		release_reset();
		steps(4 * POLL_CLKS);
		check_eq("presence", "`present` (no service)", 0, present(), "bridge header", 1);
	}
	run_until(is_ready, 100000);
	steps(100);
	gw32(A_GO, 1);
	finish_program(1200000);
	const char *S = "bridge header: CpID != 1 -> BERR on the first CIR access -> F-line (MC68030 UM 10.4 / 7.1.4)";
	for (unsigned k = 2; k <= 7; k++) cpid_checks(k, "cpid", S);
	// 6 CpIDs x 7 instructions -> 42 CIR cycles, all BERR
	check_eq("cpid", "CIR cycles in the run (one per instruction, each a BERR)", 42, cirs.size(), "bridge header: BERR on the first access ends the instruction", 2);
	unsigned nb = 0;
	for (auto &c : cirs) nb += c.berr;
	check_eq("cpid", "of which ended in BERR", 42, nb, "bridge header", 2);
	check_eq("cpid", "exceptions taken (42 F-line)", 42, exlog().size(), "test harness", 2);
	bus_checks(alive ? "cpid_alive" : "cpid_absent");
	ddr_checks();
	presence_checks(false);
}

static void sc_absent()
{
	begin_test("CpID 1 instructions with no service", "t_absent");
	release_reset();
	steps(4 * POLL_CLKS);
	check_eq("presence", "`present` (no service)", 0, present(), "bridge header", 1);
	gw32(A_GO, 1);
	finish_program(600000);
	cpid_checks(1, "absent", "bridge header: first CIR access without service -> BERR -> F-line; same as before the bridge (docs/FPU_ARM.md)");
	check_eq("absent", "CIR cycles in the run (one per instruction, each a BERR)", 7, cirs.size(), "bridge header", 1);
	bus_checks("absent");
	ddr_checks();
	presence_checks(false);
}

static void sc_reset()
{
	run_alive_program("RESET instruction", "t_reset", 400000);
	const char *G = "reset";
	const char *UM = "MC68881/2 UM 6.4.2: the FPU is in the null state after reset (RESET asserts the coprocessor reset)";
	expect(G, Tg("T_RS_BEFORE"), "FNOP; FSAVE: format word (idle)", 0x1F38, "MC68882 UM 6.4.2", 4);
	expect(G, Tg("T_RS_AFTER"), "FSAVE after the RESET instruction: format word (null, $0038 for the 68882)", 0x0038, "Hatari fpuop_save: null frame_id = (frame_size-4)<<16; MC68881/2 UM 6.4.2: null state after reset", 4);
	expect(G, Tg("T_RS_SIZE"), "... frame size", 4, UM);
	expect(G, Tg("T_RS_FNOP"), "FNOP after RESET (service still present): exceptions", 0, "bridge header: reset does not clear presence");
	bus_checks("reset");
	ddr_checks();
	presence_checks(false);
}

static void sc_raw()
{
	run_alive_program("raw CIR accesses (MOVES, DFC=7), FPU present", "t_raw", 600000);
	const char *G = "raw";
	const char *H = "bridge header: response primitives (MC68881/2 UM table 7-7): null/PF = $0802, null with TF = $08xT; the first conditional after a reset is asked of the ARM ($8900 until the reply)";
	expect(G, Tg("T_RAW_RESP0"), "response CIR after reset: null, processing finished", 0x0802, H, 4);
	expect(G, Tg("T_RAW_SAVE"), "save CIR in the null state (after the come-again $0118 reads): format word", 0x0038, "Hatari fpuop_save null frame_id $0038; bridge header: FSAVE reads $0118 until the save reply", 4);
	expect(G, Tg("T_RAW_TF1"), "response after condition word $000F (T): primitive high byte (null, CA=0)", 0x08, H, 2);
	expect(G, Tg("T_RAW_TF1B"), "... TF bit", 1, "MC68881/2 UM conditional tests: T is true", 1);
	expect(G, Tg("T_RAW_TF0H"), "response after condition word $0000 (F): primitive high byte (null, CA=0)", 0x08, H, 2);
	expect(G, Tg("T_RAW_TF0"), "... TF bit", 0, "MC68881/2 UM conditional tests: F is false", 1);
	expect(G, Tg("T_RAW_CMD"), "response after command word $0000 (FMOVE FP0,FP0): released, null with IA", 0x0900, "bridge header: reg-to-reg released at once ($0900); MC68881 UM 7.? null primitive IA", 4);
	expect(G, Tg("T_RAW_CMD2"), "... the response CIR later: idle (null, PF) once the ARM has finished", 0x0802, "bridge header", 4);
	expect(G, Tg("T_RAW_DONE"), "program ran to its end", 1, "test harness", 1);
	check_eq(G, "exceptions taken by raw accesses", 0, exlog().size(), "test harness", 1);
	check_eq(G, "CIR cycles with BERR (service present)", 0, [] { unsigned n = 0; for (auto &c : cirs) n += c.berr; return n; }(), "bridge header", 1);
	bus_checks("raw");
	ddr_checks();
	presence_checks(false);
}

static void sc_rawna()
{
	begin_test("raw CIR accesses (MOVES, DFC=7), no service", "t_rawna");
	release_reset();
	steps(4 * POLL_CLKS);
	run_until(is_ready, 100000);
	gw32(A_GO, 1);
	finish_program(600000);
	const char *G = "rawna";
	expect(G, Tg("T_RAW_DONE"), "program ran to its end", 1, "test harness", 1);
	check_eq(G, "CIR cycles (7 reads + 6 writes of non-refused CIRs)", 13, cirs.size(), "test harness", 2);
	unsigned nb = 0;
	for (auto &c : cirs) nb += c.berr;
	check_eq(G, "CIR cycles ended in BERR (only command/condition/restore writes and the save read may)", 0, nb,
	         "bridge header: BERR only on an instruction's first CIR access types; MC68030 UM 10", 1);
	bus_checks("rawna");
	ddr_checks();
	presence_checks(false);
}

// the service disappears (MAGIC cleared) at many different moments while the
// CPU runs FSAVE/FRESTORE back to back
static void sc_straddle()
{
	begin_test("presence lost in the middle of FPU instructions", "t_straddle");
	set_pacing(true);
	child_start();
	release_reset();
	run_until(present, 6 * POLL_CLKS);
	run_until(is_ready, 100000);
	gw32(A_GO, 1);
	steps(3000);
	unsigned reps = 0;
	for (; reps < 300; reps++) {
		if (!present() && !run_until(present, 3 * POLL_CLKS)) break;
		steps(rnd(1200) + 50);
		mb_w16(0x000, 0);                              // service gone
		if (!wait_fall(3 * POLL_CLKS)) break;
		steps(rnd(250) + 10);
		mb_w16(0x000, MAGIC);                          // service back
		if (!wait_rise(4 * POLL_CLKS)) break;
	}
	check_eq("straddle", "presence loss/return cycles completed", 300, reps, "test harness", 3);
	gw32(A_STOP, 1);
	finish_program(60000);
	{   // exceptions in the run: F-line (refused at the first access) and, for an instruction waiting for the ARM, the protocol violation
		unsigned nfl = 0, npv = 0, nother = 0;
		for (auto &e : exlog()) {
			if (e.vec == 11 && e.fmt == 0) nfl++;
			else if (e.vec == 13 && e.fmt == 9) npv++;
			else nother++;
		}
		check_eq("straddle", fmt("exceptions other than F-line (%u) and coprocessor protocol violation, format 9 (%u)", nfl, npv), 0, nother,
		         "bridge header: no FPU -> F-line; service stops answering or presence lost while an instruction waits -> $1D0D (vector 13)", 1);
	}
	// instructions whose CIR dialogue was cut by the loss of presence
	unsigned straddled = 0, refused = 0, completed_after = 0;
	for (size_t i = 0; i < cirs.size();) {
		size_t j = i;
		while (j < cirs.size() && cirs[j].instr == cirs[i].instr) j++;
		if (j - i >= 2 && !cirs[i].berr && cirs[i].present) {
			for (uint64_t f : pr_falls)
				if (f > cirs[i].clk0 && f < cirs[j - 1].clk1) { straddled++; if (!cirs[j - 1].berr) completed_after++; break; }
		}
		if (cirs[i].berr) refused++;
		i = j;
	}
	check_true("straddle", fmt("FPU instructions whose dialogue straddled the loss of presence: %u (all %u completed without BERR); refused at their first access: %u",
	                          straddled, completed_after, refused),
	           straddled >= 5 && completed_after == straddled && refused >= 5,
	           "MC68030 UM 10: BERR only on the first CIR access means 'no coprocessor'; bridge header");
	bus_checks("straddle");
	ddr_checks();
	presence_checks(false);
}

//------------------------------------------------------------------ milestone 2
static std::map<std::string, uint32_t> load_syms(const char *lst)
{
	std::map<std::string, uint32_t> m;
	FILE *f = fopen(lst, "r");
	if (!f) { fprintf(stderr, "cannot open %s\n", lst); exit(2); }
	char line[512];
	while (fgets(line, sizeof line, f)) {
		char nm[256], sec[8];
		unsigned a;
		if (sscanf(line, "%255s %2[0-9A-Za-z]:%x", nm, sec, &a) == 3) m[nm] = a;
	}
	fclose(f);
	return m;
}

// the 16-bit response words returned in CIR read cycles ($00) between two clocks
static unsigned count_resp(uint64_t from, uint64_t to, unsigned word)
{
	unsigned n = 0;
	for (auto &c : cirs)
		if (c.id == 1 && c.rw && c.off == 0x00 && c.clk0 >= from && c.clk0 <= to && (c.data >> 16) == word) n++;
	return n;
}
static const Cir *find_cir(uint64_t from, bool rw, unsigned off, int hiword = -1)
{
	for (auto &c : cirs)
		if (c.id == 1 && c.clk0 >= from && c.rw == rw && c.off == off && (hiword < 0 || (int)(c.data >> 16) == hiword)) return &c;
	return nullptr;
}
static std::string ram_hex(uint32_t a, int n)
{
	std::string s;
	char b[4];
	for (int i = 0; i < n; i++) { snprintf(b, sizeof b, "%02x", ddr[a + i]); s += b; }
	return s;
}

static std::vector<M2Exc> case_exc(const std::vector<ExEnt> &ex, int i)
{
	std::vector<M2Exc> v;
	for (auto &e : ex)
		if (e.ix == (uint32_t)i && !(e.vec == 7 && e.fmt == 2)) v.push_back({e.vec, e.fmt, e.pc, e.ia});   // (taken FTRAPcc are counted by the program)
	return v;
}

static void run_cases(bool irqmode)
{
	m2_build();
	const char *G = "m2";
	unsigned npass = 0, nfail = 0, nimp = 0, nimpok = 0, ngap = 0, tot_kind3 = 0, tot_gold3 = 0, tot_exec = 0, tot_prim_pc = 0;
	std::string gap_first;
	std::map<std::string, std::pair<unsigned, unsigned> > grp;     // per group: pass, fail
	FILE *fpl = fopen("obj/m2_fpiar.log", "w");
	for (int ch = 0; ch < m2_nchunks(); ch++) {
		bool any = false;
		for (int i = m2_chunk_first(ch); i < m2_chunk_end(ch); i++) any |= !m2_skip(i);
		if (!any || m2_chunk_irq(ch) != irqmode) continue;
		std::string prog = fmt("t_cases%d", ch);
		std::map<std::string, uint32_t> syms = load_syms(fmt("obj/%s.lst", prog.c_str()).c_str());
		begin_test(fmt("M2/M3 generated cases vs the Hatari golden, chunk %d (cases %d-%d)", ch, m2_chunk_first(ch), m2_chunk_end(ch) - 1).c_str(), prog.c_str());
		m2_load_ram(ddr.data(), ch);
		std::vector<uint8_t> rom = read_file(fmt("obj/%s.bin", prog.c_str()));
		m2_run_golden(syms, ch, &rom);
		set_pacing(true);
		child_start();
		release_reset();
		bool ok = run_until(present, 20 * POLL_CLKS);
		check_true("presence", "`present` rises with the real service", ok, "bridge header");
		run_until(is_ready, 800000);
		steps(2000);
		gw32(A_GO, 1);
		if (irqmode) {
			// hold the ARM at the case's hold point, wait until the instruction is in its come-again loop, raise the interrupt
			bool hw = run_until([] { return gr32(A_HOLDW) == 1; }, 800000);
			check_true("irqk", fmt("chunk %d: the program reached its hold point", ch), hw, "test harness");
			child_signal(SIGSTOP);
			steps(200);
			gw32(A_HOLDW, 0);
			bool ca = run_until([] { return ca_run >= 6; }, 100000);
			check_true("irqk", fmt("chunk %d: the FPU instruction is in its come-again loop (%u consecutive come-again responses, ARM held)", ch, ca_run), ca, "bridge header: Busy / FSAVE while busy");
			g_ipl = 3;
			uint64_t t_irq = g_clk;
			bool taken = run_until([] { return gr32(A_IRQCNT) != 0; }, 20000);
			check_true("irqk", fmt("chunk %d: the level 3 interrupt was taken %llu clocks after it was raised, with the instruction still waiting for the ARM", ch,
			                      (unsigned long long)(g_clk - t_irq)), taken && !is_done(),
			           "MC68030 UM 10.4.8: interrupt on a come-again null primitive with IA stacks a format $9 frame; AP68030 iack_cpmid");
			steps(1500);
			check_true("irqk", fmt("chunk %d: the instruction is still waiting after the handler returned (re-reads the response CIR)", ch), !is_done(), "MC68030 UM 10.4.8");
			child_signal(SIGCONT);
		}
		// run to the end; stop when no case makes progress for 5 watchdog periods
		uint32_t last = ~0u;
		uint64_t last_clk = g_clk;
		while (!is_done() && g_clk < 600000000ull) {
			steps(1000);
			uint32_t cx = gr32(A_CASEIX);
			if (cx != last) { last = cx; last_clk = g_clk; }
			else if (g_clk - last_clk > 20ull * CLK_HZ / 10 || ca_run > 30000) break;     // hang: no progress, or the CPU is told to come again for ~3 watchdog periods
		}
		check_true("run", fmt("chunk %d reached its done marker (clk %llu, case %u)", ch, (unsigned long long)g_clk, gr32(A_CASEIX)), is_done(), "test harness");
		collect();
		auto ex = exlog();
		for (int i = m2_chunk_first(ch); i < m2_chunk_end(ch); i++) {
			if (m2_skip(i)) continue;
			M2Result r = m2_compare(ddr.data(), i, case_exc(ex, i));
			std::string tag = std::string(m2_group(i)) + "] " + m2_name(i);
			bool imm = m2_expects_imm(i);
			if (imm) { nimp++; if (r.ok) nimpok++; }
			if (fpl && r.done) fprintf(fpl, "%d %s: FPIAR golden %08x device %08x\n", i, m2_name(i), r.fpiar_exp, r.fpiar_got);
			if (r.ok) {
				npass++;
				grp[m2_group(i)].first++;
				report(true, G, fmt("[%s: every FP register, FPCR, FPSR, integer registers, memory%s equal to the golden; expected %s got identical", tag.c_str(),
				                   r.nexc ? " and the expected exceptions" : "", r.summary.c_str()), irqmode ? "Hatari fpp.c golden; MC68030 UM 10.4.8: interrupt during come-again = format $9 frame (IA = the FPU instruction, PC = next), RTE completes the instruction; regression for AP68030 iack_cpmid (ap030_core.v, exec_a.vh, exec_c.vh)" : "Hatari fpp.c golden");
			} else {
				nfail++;
				grp[m2_group(i)].second++;
				std::string d;
				if (!r.done) {
					d = "case did not run to its end";
					for (auto &e : ex)
						if (e.ix == (uint32_t)i) d += fmt("; exception vector %u format %u at pc %08x", e.vec, e.fmt, e.pc);
				}
				for (auto &x : r.diffs) d += fmt("; %s expected %s got %s", x.what.c_str(), x.exp.c_str(), x.got.c_str());
				report(false, G, fmt("[%s: %s%s", tag.c_str(), r.summary.c_str(), (" -> " + d).c_str()), irqmode ? "Hatari fpp.c golden; MC68030 UM 10.4.8; without AP68030 iack_cpmid the interrupt stacks format 0 and RTE skips the rest of the instruction" : "Hatari fpp.c golden");
			}
			if (r.fpiar_diff && r.done) {
				ngap++;
				if (gap_first.empty()) gap_first = fmt("case %d %s: expected FPIAR %08x got %08x", i, m2_name(i), r.fpiar_exp, r.fpiar_got);
			}
		}
		{   // CIR $18: the instruction address is written exactly for the instructions run while an arithmetic exception is enabled
			std::vector<uint32_t> must, mustnot;
			m2_ia_sets(must, mustnot);
			std::set<uint32_t> obs;
			for (auto &c : cirs)
				if (c.id == 1 && !c.rw && c.off == 0x18 && !c.berr) obs.insert(c.data);
			unsigned miss = 0, extra = 0;
			std::string mfirst, efirst;
			for (uint32_t a : must) if (!obs.count(a)) { miss++; if (mfirst.empty()) mfirst = fmt("%08x", a); }
			for (uint32_t a : mustnot) if (obs.count(a)) { extra++; if (efirst.empty()) efirst = fmt("%08x", a); }
			check_eq(G, fmt("chunk %d: instructions with FPCR[14:8] enabled (or BSUN raised) without a CIR $18 write (of %zu)%s%s", ch, must.size(), mfirst.empty() ? "" : ", first ", mfirst.c_str()), 0, miss,
			         "bridge header: PC bit while an exception is enabled; Hatari maybe_set_fpiar", 1);
			check_eq(G, fmt("chunk %d: instructions with no exception enabled that wrote CIR $18 (of %zu)%s%s", ch, mustnot.size(), efirst.empty() ? "" : ", first ", efirst.c_str()), 0, extra,
			         "bridge header: no PC request with nothing (or only BSUN) enabled", 1);
		}
		tot_kind3 += n_kind[3];
		tot_exec += n_kind[1];
		tot_gold3 += m2_golden_cond_requests();
		bus_checks(fmt("m2_cases%d", ch).c_str());
		tot_prim_pc += g_prim_pc_seen;
		ddr_checks();
		check_eq("run", "ARM service still alive at the end (child not exited)", 0, child_exited ? 1 : 0, "tools/falcon_fpu", 1);
		child_reap();
	}
	if (fpl) fclose(fpl);
	for (auto &g : grp) printf("  group %-12s %u pass, %u fail\n", g.first.c_str(), g.second.first, g.second.second);
	printf("  summary: %u cases pass, %u fail; #imm/multi-register immediate cases %u of which %u pass\n", npass, nfail, nimp, nimpok);
	check_eq(G, "cases (that ran to their end) whose dumped FPIAR differs from the golden", 0, ngap, "Hatari fpp.c golden; bridge header: FPIAR not loaded from the instruction address yet", 1);
	if (ngap) printf("        first: %s\n", gap_first.c_str());
	if (!irqmode) check_true(G, fmt("BSUN/PC primitives ($5C30 etc.: PC bit, then CIR $18, then the exception acknowledge) seen in the CIR log: %u", tot_prim_pc), tot_prim_pc >= 20,
	           "bridge header: BSUN is a pre-instruction exception with the PC ($5C30); MC68030 UM 10.5.2");
	check_eq(G, fmt("condition requests (KIND 3) posted to the ARM (of %u execute requests): only IEEE-nonaware predicates with NAN set", tot_exec),
	         tot_gold3, tot_kind3, "bridge header: KIND 3 only for nonaware predicates with NAN; golden = count of fpp_cond evaluations with (cc & 0x10) and FPSR NAN", 1);
}

static void gen_m3(const char *)
{
	m3_build();
	for (int c = 0; c < m3_nchunks(); c++) {
		std::string p = fmt("obj/gen_m3_%d.s", c);
		FILE *f = fopen(p.c_str(), "w");
		if (!f) { perror("gen m3"); exit(2); }
		fprintf(f, "; generated by tb_fpu --gen (tb/fpu/m3.cpp): chunk %d\n", c);
		m3_emit_asm(f, c);
		fclose(f);
	}
}

static void sc_m3()
{
	m3_build();
	unsigned tot_items = 0, tot_bad = 0, tot_groups = 0, tot_gbad = 0;
	for (int ch = 0; ch < m3_nchunks(); ch++) {
		std::string prog = fmt("t_m3%d", ch);
		std::map<std::string, uint32_t> syms = load_syms(fmt("obj/%s.lst", prog.c_str()).c_str());
		begin_test(fmt("M3 arithmetic sweeps vs the Hatari golden, chunk %d of %d", ch, m3_nchunks()).c_str(), prog.c_str());
		m3_load_ram(ddr.data(), ch);
		m3_run_golden(syms, ch);
		set_pacing(true);
		child_start();
		release_reset();
		bool ok = run_until(present, 20 * POLL_CLKS);
		check_true("presence", "`present` rises with the real service", ok, "bridge header");
		run_until(is_ready, 800000);
		steps(2000);
		gw32(A_GO, 1);
		size_t last_n = 0;
		uint64_t last_clk = g_clk;
		while (!is_done() && g_clk < 900000000ull) {
			steps(2000);
			if (cirs.size() != last_n) { last_n = cirs.size(); last_clk = g_clk; }
			else if (g_clk - last_clk > 5ull * CLK_HZ / 20) break;
		}
		check_true("run", fmt("chunk %d reached its done marker (clk %llu, %zu CIR cycles)", ch, (unsigned long long)g_clk, cirs.size()), is_done() && m3_done(ddr.data(), ch), "test harness");
		auto ex = exlog();
		std::vector<M2Exc> ev;
		for (auto &e : ex) ev.push_back({e.vec, e.fmt, e.pc});
		std::string ed;
		bool eok;
		unsigned nsoft;
		std::vector<M3Group> gs = m3_compare(ddr.data(), ch, ev, ed, eok, nsoft);
		for (auto &g : gs) {
			tot_groups++;
			tot_items += g.items;
			tot_bad += g.bad;
			if (g.bad) tot_gbad++;
			if (!g.bad)
				report(true, "m3", fmt("%s: %u items, all 80-bit results, FPSR (cc, quotient, exception, accrued bytes) equal to the golden; expected identical got identical", g.name.c_str(), g.items), "Hatari fpp.c golden");
			else
				report(false, "m3", fmt("%s: %u of %u items differ%s", g.name.c_str(), g.bad, g.items, g.detail.c_str()), "Hatari fpp.c golden");
		}
		report(eok, "m3", fmt("exceptions at refused opmodes: %u expected (vector 11 / 4 at the instruction, format 0), got %zu%s", nsoft, ev.size(), eok ? ", identical" : (" -> " + ed).c_str()),
		       "bridge header: $6E-$77 F-line, $78-$7F vector 4; Hatari fault_if_nonexisting_opmode");
		bus_checks(fmt("m3_%d", ch).c_str());
		ddr_checks();
		check_eq("run", "ARM service still alive at the end (child not exited)", 0, child_exited ? 1 : 0, "tools/falcon_fpu", 1);
		child_reap();
	}
	printf("  m3 summary: %u groups (%u with differences), %u items, %u differing\n", tot_groups, tot_gbad, tot_items, tot_bad);
}

static void sc_cases() { run_cases(false); }
static void sc_irqk() { run_cases(true); }

// background execution, FSAVE while busy, watchdog -----------------------------------------------
static bool wait_mark(uint32_t addr, uint64_t max, uint64_t *when = nullptr)
{
	bool r = run_until([&] { return gr32(addr) != 0; }, max);
	if (when) *when = g_clk;
	return r;
}

static void real_start(const char *name, const char *prog)
{
	begin_test(name, prog);
	set_pacing(true);
	child_start();
}

static void sc_bg()
{
	real_start("background execution (service held with SIGSTOP)", "t_bg");
	release_reset();
	run_until(present, 20 * POLL_CLKS);
	bool ok = wait_mark(A_MARK1, 800000);
	check_true("bg", "program synchronised (first FPU instructions done)", ok, "test harness");
	steps(500);
	child_signal(SIGSTOP);                       // the ARM can no longer reply
	steps(200);
	gw32(A_GO, 1);
	uint64_t t_m2 = 0, t_m4 = 0;
	run_until([&] {
		if (!t_m2 && gr32(A_MARK2)) t_m2 = g_clk;
		if (!t_m4 && gr32(A_MARK4)) { t_m4 = g_clk; return true; }
		return false;
	}, 60000);
	check_true("bg", fmt("non-FPU instructions after the released FADD ran to the next FPU instruction (marker 2 at clk %llu, marker 4 at clk %llu) while the ARM was held",
	                    (unsigned long long)t_m2, (unsigned long long)t_m4), t_m2 && t_m4, "bridge header: reg-to-reg released at once ($0900); MC68882 UM 7.5 (reg-to-reg runs concurrently)");
	check_eq("bg", "instructions counted by the CPU while the FADD was outstanding", 8, gr32(A_MARK3), "test harness", 1);
	// the second FADD is told to come again until the reply
	run_until([&] { return count_resp(t_m4, ~0ull, 0x8900) >= 8; }, 60000);
	unsigned ca = count_resp(t_m4, ~0ull, 0x8900);
	check_true("bg", fmt("come-again responses ($8900) read by the second FPU instruction while no reply exists: %u (expected at least 8)", ca), ca >= 8,
	           "bridge header: a later instruction gets come-again $8900 until the reply; MC68882 UM 7.2.6");
	check_eq("bg", "DONE marker not yet set while the second FADD still waits", 0, is_done() ? 1 : 0, "test harness", 1);
	uint64_t t_cont = g_clk;
	child_signal(SIGCONT);
	finish_program(300000, true);
	// the first FADD command write ($0422) is answered with $0900
	const Cir *w1 = find_cir(1, false, 0x0A, 0x0422);
	const Cir *r1 = w1 ? find_cir(w1->clk1, true, 0x00) : nullptr;
	check_eq("bg", "first response read after the FADD command write", 0x0900, r1 ? (r1->data >> 16) : 0xFFFF,
	         "bridge header: R_REL $0900 = null, IA, CA=0 (released)", 4);
	check_true("bg", fmt("the CPU wrote marker 2 (clk %llu) before the service was released (SIGCONT at clk %llu)", (unsigned long long)t_m2, (unsigned long long)t_cont),
	           t_m2 && t_m2 < t_cont, "background execution");
	const Cir *w2 = find_cir(1, false, 0x0A, 0x00A2);
	const Cir *rl = nullptr;
	if (w2)
		for (auto &c : cirs)
			if (c.id == 1 && c.rw && c.off == 0 && c.clk0 > w2->clk1 && (c.data >> 16) != 0x8900) { rl = &c; break; }
	check_eq("bg", "response after the come-agains of the second FADD", 0x0900, rl ? (rl->data >> 16) : 0xFFFF, "bridge header: released once the reply is in", 4);
	check_true("bg", fmt("FP0 image expected 40010000e000000000000000 got %s", ram_hex(0x8100, 12).c_str()), ram_hex(0x8100, 12) == "40010000e000000000000000", "3 + 4 = 7 = 1.75 * 2^2");
	check_true("bg", fmt("FP1 image expected 40020000b000000000000000 got %s", ram_hex(0x810C, 12).c_str()), ram_hex(0x810C, 12) == "40020000b000000000000000", "4 + 7 = 11 = 1.375 * 2^3");
	bus_checks("bg");
	ddr_checks();
}

static void sc_busy()
{
	real_start("FSAVE while a request is outstanding", "t_busy");
	release_reset();
	run_until(present, 20 * POLL_CLKS);
	wait_mark(A_MARK1, 800000);
	steps(500);
	child_signal(SIGSTOP);
	steps(200);
	gw32(A_GO, 1);
	wait_mark(A_MARK2, 60000);
	uint64_t t0 = g_clk;
	run_until([&] {
		unsigned n = 0;
		for (auto &c : cirs)
			if (c.id == 1 && c.rw && c.off == 0x04 && c.clk0 >= t0 && (c.data >> 16) == 0x0118) n++;
		return n >= 6;
	}, 60000);
	unsigned nca = 0;
	for (auto &c : cirs)
		if (c.id == 1 && c.rw && c.off == 0x04 && c.clk0 >= t0 && (c.data >> 16) == 0x0118) nca++;
	check_true("busy", fmt("save CIR reads returning the come-again format $0118 while the FADD is outstanding: %u (expected at least 6)", nca), nca >= 6,
	           "MC68881 UM 6.4.3 / bridge header: FSAVE while busy -> $0118");
	check_eq("busy", "DONE marker not yet set while FSAVE waits", 0, is_done() ? 1 : 0, "test harness", 1);
	child_signal(SIGCONT);
	finish_program(300000, true);
	collect();
	expect("busy", Tg("T_BUSY_FMT"), "FSAVE after the come-agains: format word (idle)", 0x1F38, "MC68882 UM 6.4.2: idle frame $1F38", 4);
	expect("busy", Tg("T_BUSY_DELTA"), "FSAVE after the come-agains: frame size", 60, "MC68882 UM 6.4.2: 4 + 56 bytes");
	check_true("busy", fmt("FP0 image after FSAVE expected 40010000e000000000000000 got %s", ram_hex(0x8100, 12).c_str()), ram_hex(0x8100, 12) == "40010000e000000000000000",
	           "3 + 4 = 7: the background FADD finished before the idle frame");
	bus_checks("busy");
	ddr_checks();
}

static void sc_irq()
{
	real_start("interrupt during FSAVE come-again", "t_irq");
	release_reset();
	run_until(present, 20 * POLL_CLKS);
	wait_mark(A_MARK1, 800000);
	steps(500);
	child_signal(SIGSTOP);
	steps(200);
	gw32(A_GO, 1);
	uint64_t t0 = g_clk;
	auto nca = [&] {
		unsigned n = 0;
		for (auto &c : cirs)
			if (c.id == 1 && c.rw && c.off == 0x04 && c.clk0 >= t0 && (c.data >> 16) == 0x0118) n++;
		return n;
	};
	run_until([&] { return nca() >= 6; }, 60000);
	check_true("irq", fmt("FSAVE is in its come-again loop (%u reads of $0118)", nca()), nca() >= 6, "bridge header: FSAVE while busy reads $0118");
	g_ipl = 3;
	uint64_t t_irq = g_clk;
	bool taken = run_until([] { return gr32(0x0F5C) != 0; }, 20000);
	check_true("irq", fmt("the level 3 interrupt was taken %llu clocks after it was raised, while FSAVE still waits for the ARM",
	                     (unsigned long long)(g_clk - t_irq)), taken && !is_done(),
	           "MC68030 UM 10.5.4: come-again lets the processor service interrupts; bridge header: FSAVE come-again $0118 (UM 6.4.3)");
	unsigned n_before = nca();
	steps(2000);
	check_true("irq", fmt("FSAVE keeps polling after the interrupt handler (%u more reads of $0118)", nca() - n_before), nca() > n_before && !is_done(), "FSAVE restarts after RTE");
	child_signal(SIGCONT);
	finish_program(300000, true);
	collect();
	expect("irq", Tg("T_IRQ_CNT"), "interrupts taken (handler count)", 1, "test harness", 1);
	expect("irq", Tg("T_IRQ_FMT"), "FSAVE after the interrupt: format word (idle)", 0x1F38, "MC68882 UM 6.4.2", 4);
	expect("irq", Tg("T_IRQ_DELTA"), "FSAVE after the interrupt: frame size", 60, "MC68882 UM 6.4.2: 4 + 56 bytes");
	check_eq("irq", "exceptions other than the interrupt (UNEXP flag)", 0, gr32(A_UNEXP), "test harness", 1);
	bus_checks("irq");
	ddr_checks();
}

static void sc_fsdie()
{
	real_start("FSAVE whose service stops answering", "t_fsdie");
	release_reset();
	run_until(present, 20 * POLL_CLKS);
	wait_mark(A_MARK1, 800000);
	steps(500);
	child_signal(SIGSTOP);
	steps(200);
	gw32(A_GO, 1);
	uint64_t t_go = g_clk;
	const uint64_t WD = CLK_HZ / 10;
	bool got = wait_mark(A_MARK2, 3 * WD + 100000);
	uint64_t dt = g_clk - t_go;
	check_true("fsdie", fmt("FSAVE ended after %llu clocks without the service (watchdog CLK_HZ/10 = %llu)", (unsigned long long)dt, (unsigned long long)WD),
	           got && dt >= WD * 9 / 10 && dt <= WD + 20000, "bridge header: if the service dies meanwhile, FSAVE ends with a null frame");
	check_eq("fsdie", "exceptions taken while FSAVE waited", 0, gr32(A_EXCNT), "bridge header: null frame, no exception", 1);
	child_signal(SIGCONT);
	run_until(present, 6 * POLL_CLKS + HB_PERIOD);     // the held service stopped its heartbeat: presence returns after a poll
	check_eq("fsdie", "`present` after the service is released", 1, present(), "bridge header: heartbeat moves again", 1);
	steps(3000);
	gw32(A_STOP, 1);
	finish_program(300000, true);
	collect();
	expect("fsdie", Tg("T_FSD_FMT"), "FSAVE whose service died: format word (null frame)", 0x0000, "bridge header: FSAVE ends with a null frame $0000", 4);
	expect("fsdie", Tg("T_FSD_DELTA"), "... bytes pushed (format long only)", 4, "bridge header; version $00 = null frame", 1);
	expect("fsdie", Tg("T_FSD_FMT2"), "FNOP; FSAVE once the service is back: format word (idle)", 0x1F38, "MC68882 UM 6.4.2", 4);
	expect("fsdie", Tg("T_FSD_DELTA2"), "... frame size", 60, "MC68882 UM 6.4.2");
	check_eq("fsdie", "exceptions taken in the whole run", 0, gr32(A_EXCNT), "bridge header: no BERR, no exception", 1);
	for (auto &e : exlog()) printf("        exception: vector %u format %u pc %08x sr %04x\n", e.vec, e.fmt, e.pc, e.sr);
	bus_checks("fsdie");
	ddr_checks();
}

// variant: 0 service held with SIGSTOP, 1 no service at all (C++ heartbeat), 2 service held and MAGIC cleared (presence lost)
static void sc_wd(int variant)
{
	static const char *names[3] = {"watchdog: service held with SIGSTOP", "watchdog: no service answers (C++ heartbeat only)",
	                               "watchdog: presence lost while the CPU waits"};
	begin_test(names[variant], "t_wd");
	gw32(A_MODE, variant == 1 ? 0 : 1);
	if (variant == 1) hb_start();
	else { set_pacing(true); child_start(); }
	release_reset();
	run_until(present, 20 * POLL_CLKS);
	if (variant != 1) {
		bool ok = wait_mark(A_MARK1, 800000);
		check_true("wd", "program synchronised", ok, "test harness");
		steps(500);
		child_signal(SIGSTOP);
		steps(200);
	} else {
		run_until(is_ready, 800000);
		steps(500);
	}
	gw32(A_GO, 1);
	uint64_t t_go = g_clk;
	const uint64_t WD = CLK_HZ / 10;
	uint64_t t_clear = 0;
	if (variant == 2) {
		run_until([&] { return find_cir(t_go, false, 0x0A, 0x6800) != nullptr; }, 100000);
		steps(3000);
		mb_w16(0x000, 0);                              // presence lost
		t_clear = g_clk;
	}
	bool got = wait_mark(A_MARK3, 3 * WD + 100000);
	uint64_t t_exc = g_clk;
	check_true("wd", fmt("the waiting FMOVE FP0,(a2) ended (marker 3 at clk %llu)", (unsigned long long)t_exc), got, "bridge header: watchdog -> mid-instruction exception");
	check_eq("wd", "instruction after the FMOVE was not executed (marker 2)", 0, gr32(A_MARK2), "the exception abandons the instruction", 1);
	auto ex = exlog();
	check_eq("wd", "exceptions taken", 1, ex.size(), "bridge header: one protocol violation", 1);
	if (!ex.empty()) {
		check_eq("wd", "exception vector (coprocessor protocol violation)", 13, ex[0].vec, "MC68030 UM Table 8-1 vector 13; bridge header $1D0D", 2);
		check_eq("wd", "stack frame format (coprocessor mid-instruction)", 9, ex[0].fmt, "MC68030 UM 8.2: format $9", 1);
	}
	const Cir *cw = find_cir(t_go, false, 0x0A, 0x6800);
	const Cir *ack = cw ? find_cir(cw->clk1, false, 0x02) : nullptr;
	if (cw && ack) {
		uint64_t dt = ack->clk0 - cw->clk0;
		if (variant == 0)
			check_true("wd", fmt("command write to exception acknowledge took %llu clocks; expected about CLK_HZ/10 = %llu (within -10%% / +%u)", (unsigned long long)dt,
			                    (unsigned long long)WD, 6000u), dt >= WD * 9 / 10 && dt <= WD + 6000,
			           "bridge header: ~100 ms at the bridge's CLK_HZ");
		else if (variant == 1)
			check_true("wd", fmt("command write to exception acknowledge took %llu clocks; at most CLK_HZ/10 = %llu plus the dialog", (unsigned long long)dt, (unsigned long long)WD),
			           dt <= WD + 6000, "bridge header: ~100 ms");
		else
			check_true("wd", fmt("exception %llu clocks after MAGIC was cleared (limit one poll + dialog = %u, far below the watchdog %llu)", (unsigned long long)(ack->clk0 - t_clear),
			                    POLL_CLKS + 3000, (unsigned long long)WD), ack->clk0 >= t_clear && ack->clk0 - t_clear <= POLL_CLKS + 3000,
			           "bridge header: or presence lost");
		unsigned nca = count_resp(cw->clk1, ack->clk0, 0x8900), npr = count_resp(cw->clk1, ack->clk0, 0x1D0D);
		check_true("wd", fmt("response reads during the wait: %u come-again ($8900) then %u protocol violation ($1D0D)", nca, npr), nca >= 1 && npr == 1,
		           "bridge header: come-again until the watchdog, then $1D0D");
	} else
		check_true("wd", "command write and acknowledge found in the CIR log", false, "test harness");
	if (variant != 1) {
		if (variant == 2) mb_w16(0x000, MAGIC);
		child_signal(SIGCONT);
		if (variant == 2) {
			bool back = run_until(present, 6 * POLL_CLKS + HB_PERIOD);
			check_true("wd", "presence returns after MAGIC is restored and the service runs", back, "bridge header");
		}
		steps(3000);
		gw32(A_STOP, 1);
		finish_program(400000, true);
		check_eq("wd", "recovery: later instructions completed (marker 4)", 1, gr32(A_MARK4), "bridge header: later instructions work", 1);
		check_true("wd", fmt("recovery: FP0+FP0 image expected 40010000c000000000000000 got %s", ram_hex(0x810C, 12).c_str()),
		           ram_hex(0x810C, 12) == "40010000c000000000000000", "3 + 3 = 6 = 1.5 * 2^2");
	} else {
		finish_program(100000, true);
	}
	bus_checks(variant == 0 ? "wd_stop" : variant == 1 ? "wd_nosvc" : "wd_presence");
	ddr_checks();
}

//------------------------------------------------------------------ main
int main(int argc, char **argv)
{
	Verilated::commandArgs(argc, argv);
	setvbuf(stdout, nullptr, _IOLBF, 0);
	std::string only;
	for (int i = 1; i < argc; i++) {
		if (!strncmp(argv[i], "--gen=", 6)) {          // write the generated cases as 68k assembly and stop
			m2_build();
			for (int c = 0; c < m2_nchunks(); c++) {
				std::string p = fmt("%s_%d.s", argv[i] + 6, c);
				FILE *f = fopen(p.c_str(), "w");
				if (!f) { perror("gen"); return 2; }
				fprintf(f, "; generated by tb_fpu --gen (tb/fpu/m2.cpp): chunk %d, cases %d-%d\n", c, m2_chunk_first(c), m2_chunk_end(c) - 1);
				m2_emit_asm(f, c);
				fclose(f);
			}
			gen_m3(argv[i] + 6);
			printf("generated %d cases in %d chunks\n", m2_ncases(), m2_nchunks());
			FILE *nf = fopen("obj/gen_chunks.txt", "w");
			if (nf) { fprintf(nf, "%d %d\n", m2_nchunks(), m3_nchunks()); fclose(nf); }
			return 0;
		}
		if (!strncmp(argv[i], "--host=", 7)) host_exe = argv[i] + 7;
		else if (!strncmp(argv[i], "--only=", 7)) only = argv[i] + 7;
	}
	if (host_exe.empty()) host_exe = "obj/falcon_fpu_host";
	char abs[4096];
	if (host_exe[0] != '/' && getcwd(abs, sizeof abs)) host_exe = std::string(abs) + "/" + host_exe;
	load_tags("asm/tags.i");
	ddr.assign(DDR_SIZE, 0);

	char cwd[4096];
	if (!getcwd(cwd, sizeof cwd)) return 2;
	mb_path = std::string(cwd) + "/obj/mailbox.bin";
	int fd = open(mb_path.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
	if (fd < 0 || ftruncate(fd, MB_SIZE) != 0) { perror("mailbox file"); return 2; }
	void *p = mmap(nullptr, MB_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (p == MAP_FAILED) { perror("mmap"); return 2; }
	MB = (uint8_t *)p;
	signal(SIGPIPE, SIG_IGN);

	printf("FPU bridge milestone 1 co-simulation: CLK_HZ=%u poll=%u clocks alive window=%u polls heartbeat=%u clocks\n", CLK_HZ, POLL_CLKS,
	       ALIVE_POLLS, HB_PERIOD);
	struct { const char *name; void (*fn)(); } sc[] = {
	    {"detect_real", [] { sc_detect("EmuTOS detect/real falcon_fpu_host", "real"); }},
	    {"detect_nosvc", [] { sc_detect("EmuTOS detect/no service", "none"); }},
	    {"detect_badmagic", [] { sc_detect("EmuTOS detect/wrong MAGIC $4651", "badmagic"); }},
	    {"detect_hb0", [] { sc_detect("EmuTOS detect/MAGIC ok, heartbeat stuck at 0", "hb0"); }},
	    {"detect_badversion", [] { sc_detect("EmuTOS detect/VERSION 1 (old service)", "badversion"); }},
	    {"watch_freeze", sc_watch_cpp_freeze},
	    {"watch_magic", sc_watch_cpp_magic},
	    {"watch_real", sc_watch_real},
	    {"frames", sc_frames},
	    {"cond", sc_cond},
	    {"gen", sc_gen},
	    {"cpid_alive", [] { sc_cpid(true); }},
	    {"cpid_absent", [] { sc_cpid(false); }},
	    {"absent", sc_absent},
	    {"reset", sc_reset},
	    {"raw", sc_raw},
	    {"rawna", sc_rawna},
	    {"straddle", sc_straddle},
	    {"m2cases", sc_cases},
	    {"m3sweeps", sc_m3},
	    {"m2bg", sc_bg},
	    {"m2busy", sc_busy},
	    {"m4irq", sc_irq},
	    {"m4irqk", sc_irqk},
	    {"m4fsdie", sc_fsdie},
	    {"m2wd_stop", [] { sc_wd(0); }},
	    {"m2wd_nosvc", [] { sc_wd(1); }},
	    {"m2wd_presence", [] { sc_wd(2); }},
	};
	for (auto &s : sc) {
		if (!only.empty() && only != s.name) continue;
		s.fn();
		child_reap();
	}
	printf("\n== summary ==\n  checks: %d passed, %d failed\n", n_pass, n_fail);
	for (auto &f : failed_checks) printf("  FAILED: %s\n", f.c_str());
	printf("RESULT: %s\n", n_fail == 0 && n_pass > 0 ? "PASS" : "FAIL");
	return n_fail == 0 && n_pass > 0 ? 0 : 1;
}
