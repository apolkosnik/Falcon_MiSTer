// tb_mbox.cpp - Verilator co-simulation of the FPGA<->ARM mailbox latency
// probe (rtl/falcon/falcon_mbox_test.sv + falcon_memarb with FALCON_MBOX_TEST)
// against the REAL host tool tools/mbox_ping/mbox_ping.c.
//
// The DDR3 is a C++ model with MiSTer DDRAM semantics; the 512 KB mailbox
// window (DDR3 byte 0x30E80000 = guest $E80000) is a MAP_SHARED file that the
// forked tool maps too, so the tool sees exactly the bytes the RTL stores.
//
// Cycle relation checked for every sample (derived from falcon_mbox_test.sv):
//   Let tP be the clock edge at which the probe samples dma_ack of the PING
//   write and tR the edge at which it samples dma_ack of the PONG read that
//   returns the sequence number.  The counter is cleared at tP and counts one
//   per edge while in S_POLL, and the value stored is the one before the
//   increment of edge tR:   stored cycles = tR - tP - 1.
//   The stored PONG-read count is the number of PONG read acks in (tP, tR].
//   Timeout: the run ends at the first poll ack edge tE with tE-tP-1 >= CLK_HZ.

#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <cstdarg>
#include <string>
#include <vector>
#include <algorithm>
#include <random>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>

#include "Vtb_mbox_top.h"
#include "verilated.h"

#ifndef TB_CLK_HZ
#define TB_CLK_HZ 1000000
#endif

static const uint32_t CLK_HZ   = TB_CLK_HZ;
static const uint32_t IDLE_CLKS = CLK_HZ / 1000;
static const uint32_t MB_G     = 0xE80000;     // guest base of the mailbox
static const uint32_t MB_SIZE  = 0x80000;
static const uint64_t DDR_BYTE0 = 0x30000000ull;
static const uint32_t DDR_SIZE = 0x1000000;    // guest 24 bit space

enum { A_CMD = 0x000, A_COUNT = 0x002, A_DONE = 0x004, A_STAT = 0x006, A_PING = 0x008,
       A_PONG = 0x00A, A_MAGIC = 0x00C, A_CLK = 0x00E, A_SMP = 0x100 };

//------------------------------------------------------------------ reporting
static int g_pass = 0, g_fail = 0;
static uint64_t g_edge = 0;

static void chk_u(const char *name, uint64_t exp, uint64_t got, const char *src)
{
	bool ok = exp == got;
	if (ok) g_pass++; else g_fail++;
	printf("  [%s] %s: expected 0x%llx got 0x%llx  (%s) edge %llu\n", ok ? "PASS" : "FAIL", name,
	       (unsigned long long)exp, (unsigned long long)got, src, (unsigned long long)g_edge);
}
static void chk_s(const char *name, const std::string &exp, const std::string &got, const char *src)
{
	bool ok = exp == got;
	if (ok) g_pass++; else g_fail++;
	printf("  [%s] %s: expected '%s' got '%s'  (%s) edge %llu\n", ok ? "PASS" : "FAIL", name,
	       exp.c_str(), got.c_str(), src, (unsigned long long)g_edge);
}
static void chk_ok(const char *name, bool ok, const char *exp, const char *got, const char *src)
{
	if (ok) g_pass++; else g_fail++;
	printf("  [%s] %s: expected %s got %s  (%s) edge %llu\n", ok ? "PASS" : "FAIL", name, exp, got, src,
	       (unsigned long long)g_edge);
}

struct Bulk {
	const char *name, *src;
	uint64_t n = 0, bad = 0;
	Bulk(const char *nm, const char *s) : name(nm), src(s) {}
	void ok(bool good, const char *fmt = nullptr, ...)
	{
		n++;
		if (good) return;
		bad++;
		if (bad <= 5) {
			printf("    [bulk mismatch] %s #%llu edge %llu: ", name, (unsigned long long)n, (unsigned long long)g_edge);
			if (fmt) { va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap); }
			printf("\n");
		}
	}
	void end()
	{
		bool good = bad == 0;
		if (good) g_pass++; else g_fail++;
		printf("  [%s] %s: expected 0 mismatches got %llu of %llu checks  (%s)\n", good ? "PASS" : "FAIL", name,
		       (unsigned long long)bad, (unsigned long long)n, src);
	}
};

static std::mt19937_64 rng(0x4D424F58ull);
static uint32_t rnd(uint32_t n) { return n ? (uint32_t)(rng() % n) : 0; }

//------------------------------------------------------------------ DUT, DDR3, mailbox file
static Vtb_mbox_top *T;
static std::vector<uint8_t> ddr_mem;       // private DDR3 bytes, offset = guest address
static uint8_t *MB;                        // mapped shared file (raw DDR3 byte order)

static inline uint16_t gr16(unsigned off)
{
	return __builtin_bswap16(__atomic_load_n((uint16_t *)(MB + off), __ATOMIC_ACQUIRE));
}
static inline void gw16(unsigned off, uint16_t v)
{
	__atomic_store_n((uint16_t *)(MB + off), __builtin_bswap16(v), __ATOMIC_RELEASE);
}

static inline uint8_t *ddr_ptr(uint32_t off)
{
	if (off >= MB_G && off < MB_G + MB_SIZE) return MB + (off - MB_G);
	return &ddr_mem[off];
}
static inline bool in_mb(uint32_t off) { return off >= MB_G && off < MB_G + MB_SIZE; }

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

// ---- DDR3 command / data model state
static int rd_left = 0, rd_wait = 0;
static uint32_t rd_off = 0;
static int busy_run = 0;
static bool ddr_stalls = true;
static uint64_t nxt_dout = 0;
static bool nxt_ready = false, nxt_busy = false;
static Bulk b_ddr("DDRAM protocol: one command at a time, burst 1 (vid 4), address in 0x30000000..0x30FFFFFF",
                  "MiSTer DDRAM semantics / falcon_memarb");
static uint64_t n_ddr_rd = 0, n_ddr_wr = 0;

static void ddr_pre()
{
	bool rd = T->DDRAM_RD, wr = T->DDRAM_WE;
	if ((rd || wr) && !T->DDRAM_BUSY) {
		uint64_t byte = (uint64_t)T->DDRAM_ADDR * 8;
		bool inr = byte >= DDR_BYTE0 && byte < DDR_BYTE0 + DDR_SIZE;
		int burst = T->DDRAM_BURSTCNT;
		b_ddr.ok(!(rd && wr) && inr && rd_left == 0 && (wr ? burst == 1 : (burst == 1 || burst == 4)),
		         "rd=%d wr=%d byte=0x%llx burst=%d rd_left=%d", rd, wr, (unsigned long long)byte, burst, rd_left);
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
	nxt_dout = ((uint64_t)rng());
	if (rd_left > 0) {
		if (rd_wait > 1) rd_wait--;
		else if (ddr_stalls && rnd(6) == 0) { /* gap inside a burst */ }
		else {
			nxt_dout = ddr_read64(rd_off);
			nxt_ready = true;
			rd_off += 8; rd_left--;
		}
	}
	nxt_busy = false;
	if (ddr_stalls) {
		if (busy_run > 0) { busy_run--; nxt_busy = true; }
		else if (rnd(100) < 30) nxt_busy = true;
		if (rnd(300) == 0) busy_run = rnd(25);
	}
}

//------------------------------------------------------------------ other arbiter masters
static std::vector<uint8_t> shadow;        // guest-order shadow of the private regions

struct DmaM {
	int id;
	uint32_t base;
	int gapmax = 20;
	bool en = false;
	bool req = false, we = false;
	uint32_t h = 0;                         // halfword address (addr[23:1])
	uint8_t be = 3;
	uint16_t wd = 0, exp_rd = 0;
	int gap = 0;
	bool nreq, nwe; uint32_t nh; uint8_t nbe; uint16_t nwd;
	uint64_t nacks = 0, nrd = 0, nwr = 0;
	Bulk *chk;
	void issue()
	{
		nreq = true;
		nh = (base >> 1) + rnd(0x800);
		nwe = rnd(2);
		nbe = nwe ? 1 + rnd(3) : 3;
		nwd = (uint16_t)rng();
		uint32_t a = nh * 2;
		if (nwe) {
			if (nbe & 2) shadow[a] = nwd >> 8;
			if (nbe & 1) shadow[a + 1] = (uint8_t)nwd;
		} else
			exp_rd = (uint16_t)((shadow[a] << 8) | shadow[a + 1]);
	}
};
static DmaM dm[3] = { { 0, 0x200000 }, { 1, 0x300000 }, { 2, 0x400000 } };

struct CpuM {
	uint32_t base = 0x100000;
	int gapmax = 20;
	bool en = false;
	bool req = false, we = false;
	uint32_t w = 0;                         // longword address (addr[23:2])
	uint8_t be = 15;
	uint32_t wd = 0, exp_rd = 0;
	int gap = 0;
	bool nreq, nwe; uint32_t nw; uint8_t nbe; uint32_t nwd;
	uint64_t nacks = 0, nrd = 0, nwr = 0;
	Bulk *chk;
	void issue()
	{
		nreq = true;
		nw = (base >> 2) + rnd(0x400);
		nwe = rnd(2);
		nbe = nwe ? 1 + rnd(15) : 15;
		nwd = (uint32_t)rng();
		uint32_t a = nw * 4;
		if (nwe) {
			for (int i = 0; i < 4; i++)
				if (nbe & (8 >> i)) shadow[a + i] = (uint8_t)(nwd >> (24 - 8 * i));
		} else
			exp_rd = ((uint32_t)shadow[a] << 24) | (shadow[a + 1] << 16) | (shadow[a + 2] << 8) | shadow[a + 3];
	}
};
static CpuM cpu;
static std::vector<uint64_t> cpu_ack_edges;

struct VidM {
	uint32_t base = 0x500000;
	int gapmax = 40;
	bool en = false;
	bool req = false;
	uint32_t w = 0;                         // 64-bit word address (addr[23:3])
	int gap = 0, beats = 0;
	bool nreq; uint32_t nw;
	bool waiting_beats = false;
	uint64_t nbursts = 0;
	Bulk *chk;
};
static VidM vid;

static Bulk b_cpu("cpu read data equals C++ shadow", "private region 0x100000");
static Bulk b_d0("d0 read data equals C++ shadow", "private region 0x200000");
static Bulk b_d1("d1 read data equals C++ shadow", "private region 0x300000");
static Bulk b_d2("d2 read data equals C++ shadow", "private region 0x400000");
static Bulk b_vid("vid burst beats equal C++ shadow (guest byte order)", "private region 0x500000");

static void drive_d(int id, bool req, bool we, uint32_t a, uint8_t be, uint16_t wd)
{
	switch (id) {
	case 0: T->d0_req = req; T->d0_we = we; T->d0_addr = a; T->d0_be = be; T->d0_wdata = wd; break;
	case 1: T->d1_req = req; T->d1_we = we; T->d1_addr = a; T->d1_be = be; T->d1_wdata = wd; break;
	default: T->d2_req = req; T->d2_we = we; T->d2_addr = a; T->d2_be = be; T->d2_wdata = wd; break;
	}
}
static bool ack_d(int id) { return id == 0 ? T->d0_ack : id == 1 ? T->d1_ack : T->d2_ack; }
static uint16_t rdata_d(int id) { return id == 0 ? T->d0_rdata : id == 1 ? T->d1_rdata : T->d2_rdata; }

static void pre_dma(DmaM &m)
{
	m.nreq = m.req; m.nwe = m.we; m.nh = m.h; m.nbe = m.be; m.nwd = m.wd;
	if (m.req && ack_d(m.id)) {
		m.nacks++;
		if (m.we) m.nwr++;
		else {
			m.nrd++;
			m.chk->ok(rdata_d(m.id) == m.exp_rd, "d%d addr 0x%06x expected 0x%04x got 0x%04x", m.id, m.h * 2,
			          m.exp_rd, rdata_d(m.id));
		}
		m.nreq = false;
		m.gap = rnd(m.gapmax + 1);
		if (m.gap == 0 && m.en) m.issue();
	} else if (!m.req) {
		if (m.gap > 0) m.gap--;
		else if (m.en) m.issue();
	}
}
static void post_dma(DmaM &m)
{
	m.req = m.nreq; m.we = m.nwe; m.h = m.nh; m.be = m.nbe; m.wd = m.nwd;
	drive_d(m.id, m.req, m.we, m.h, m.be, m.wd);
}

static void pre_cpu(CpuM &m)
{
	m.nreq = m.req; m.nwe = m.we; m.nw = m.w; m.nbe = m.be; m.nwd = m.wd;
	if (m.req && T->cpu_ack) {
		m.nacks++;
		cpu_ack_edges.push_back(g_edge);
		if (m.we) m.nwr++;
		else {
			m.nrd++;
			m.chk->ok(T->cpu_rdata == m.exp_rd, "cpu addr 0x%06x expected 0x%08x got 0x%08x", m.w * 4, m.exp_rd,
			          T->cpu_rdata);
		}
		m.nreq = false;
		m.gap = rnd(m.gapmax + 1);
		if (m.gap == 0 && m.en) m.issue();
	} else if (!m.req) {
		if (m.gap > 0) m.gap--;
		else if (m.en) m.issue();
	}
}
static void post_cpu(CpuM &m)
{
	m.req = m.nreq; m.we = m.nwe; m.w = m.nw; m.be = m.nbe; m.wd = m.nwd;
	T->cpu_req = m.req; T->cpu_we = m.we; T->cpu_addr = m.w; T->cpu_be = m.be; T->cpu_wdata = m.wd;
}

static void pre_vid(VidM &m)
{
	m.nreq = m.req; m.nw = m.w;
	if (T->vid_valid) {
		uint32_t a = (m.w * 8) + 8 * m.beats;
		uint64_t e = 0;
		for (int j = 0; j < 8; j++) e = (e << 8) | shadow[a + j];
		m.chk->ok(T->vid_data == e, "vid beat %d addr 0x%06x expected 0x%016llx got 0x%016llx", m.beats, a,
		          (unsigned long long)e, (unsigned long long)T->vid_data);
		m.beats++;
		if (m.beats == 4) { m.waiting_beats = false; m.nbursts++; m.gap = rnd(m.gapmax + 1); }
	}
	if (m.req && T->vid_ack) { m.nreq = false; m.waiting_beats = true; m.beats = 0; }
	else if (!m.req && !m.waiting_beats) {
		if (m.gap > 0) m.gap--;
		else if (m.en) { m.nreq = true; m.nw = (m.base >> 3) + rnd(0x200); }
	}
}
static void post_vid(VidM &m)
{
	m.req = m.nreq; m.w = m.nw;
	T->vid_req = m.req; T->vid_addr = m.w;
}

//------------------------------------------------------------------ arbitration / command checker
struct Snap {
	bool vreq; uint32_t vaddr;
	bool dreq[4], dwe[4]; uint32_t dh[4]; uint8_t dbe[4]; uint16_t dwd[4];   // d0..d2 + d3 (mbx)
	bool creq, cwe; uint32_t cw; uint8_t cbe; uint32_t cwd;
	bool any_ack;
};
static Snap snap_now()
{
	Snap s;
	s.vreq = T->vid_req; s.vaddr = T->vid_addr;
	s.dreq[0] = T->d0_req; s.dwe[0] = T->d0_we; s.dh[0] = T->d0_addr; s.dbe[0] = T->d0_be; s.dwd[0] = T->d0_wdata;
	s.dreq[1] = T->d1_req; s.dwe[1] = T->d1_we; s.dh[1] = T->d1_addr; s.dbe[1] = T->d1_be; s.dwd[1] = T->d1_wdata;
	s.dreq[2] = T->d2_req; s.dwe[2] = T->d2_we; s.dh[2] = T->d2_addr; s.dbe[2] = T->d2_be; s.dwd[2] = T->d2_wdata;
	s.dreq[3] = T->mbx_req; s.dwe[3] = T->mbx_we; s.dh[3] = T->mbx_addr; s.dbe[3] = T->mbx_be; s.dwd[3] = T->mbx_wdata;
	s.creq = T->cpu_req; s.cwe = T->cpu_we; s.cw = T->cpu_addr; s.cbe = T->cpu_be; s.cwd = T->cpu_wdata;
	s.any_ack = T->d0_ack | T->d1_ack | T->d2_ack | T->mbx_ack | T->cpu_ack;
	return s;
}

static Bulk b_prio("every DDR command goes to the highest-priority requester (vid>d0>d1>d2>d3>cpu), with exact address/we/BE/data",
                   "falcon_memarb header + g_* mux");
static Bulk b_snoop("snoop port pulses for non-CPU writes with the byte address, never otherwise", "falcon_memarb snoop_we");
static uint64_t n_grant[6] = {0, 0, 0, 0, 0, 0};   // vid d0 d1 d2 d3 cpu
static bool g_in_reset = true;
static int g_since_reset = 0;
static Snap prev_snap;
static bool prev_cmd = false;

static void check_cmd_start()
{
	bool cmd = T->DDRAM_RD || T->DDRAM_WE;
	if (cmd && !prev_cmd && g_since_reset > 3) {
		const Snap &s = prev_snap;
		int owner = -1;          // 0 vid, 1..4 d0..d3, 5 cpu
		if (s.any_ack) owner = -2;
		else if (s.vreq) owner = 0;
		else if (s.dreq[0]) owner = 1;
		else if (s.dreq[1]) owner = 2;
		else if (s.dreq[2]) owner = 3;
		else if (s.dreq[3]) owner = 4;
		else if (s.creq) owner = 5;
		if (owner < 0) {
			b_prio.ok(false, "command started with owner=%d (-2: a master was acknowledged in the decision cycle, -1: nobody requested)", owner);
		} else {
			n_grant[owner]++;
			uint32_t word; bool we; int burst = 1; uint8_t be = 0xFF; int val[8]; bool snoop = false; uint32_t sa = 0;
			for (int k = 0; k < 8; k++) val[k] = -1;
			if (owner == 0) { word = s.vaddr; we = false; burst = 4; }
			else if (owner <= 4) {
				int d = owner - 1;
				uint32_t h = s.dh[d];
				word = h >> 2; we = s.dwe[d];
				int sub = h & 3;
				if (we) {
					be = 0;
					if (s.dbe[d] & 2) { be |= 1 << (2 * sub); val[2 * sub] = s.dwd[d] >> 8; }
					if (s.dbe[d] & 1) { be |= 1 << (2 * sub + 1); val[2 * sub + 1] = s.dwd[d] & 0xFF; }
					snoop = true; sa = (h << 1) & 0xFFFFFF;
				}
			} else {
				word = s.cw >> 1; we = s.cwe;
				int hh = s.cw & 1;
				if (we) {
					be = 0;
					for (int i = 0; i < 4; i++)
						if (s.cbe & (8 >> i)) { be |= 1 << (4 * hh + i); val[4 * hh + i] = (s.cwd >> (24 - 8 * i)) & 0xFF; }
				}
			}
			bool ok = T->DDRAM_ADDR == (0x6000000u | word) && T->DDRAM_BURSTCNT == burst &&
			          (bool)T->DDRAM_WE == we && (bool)T->DDRAM_RD == !we && T->DDRAM_BE == be;
			if (we)
				for (int k = 0; k < 8; k++)
					if (val[k] >= 0 && (int)((T->DDRAM_DIN >> (8 * k)) & 0xFF) != val[k]) ok = false;
			b_prio.ok(ok, "owner %d expected addr 0x%x burst %d we %d be 0x%02x, got addr 0x%x burst %d we %d rd %d be 0x%02x din 0x%016llx",
			          owner, 0x6000000u | word, burst, we, be, T->DDRAM_ADDR, T->DDRAM_BURSTCNT, T->DDRAM_WE,
			          T->DDRAM_RD, T->DDRAM_BE, (unsigned long long)T->DDRAM_DIN);
			bool sok = snoop ? (T->snoop_we && T->snoop_addr == sa) : !T->snoop_we;
			b_snoop.ok(sok, "owner %d expected snoop %d addr 0x%06x, got snoop_we %d addr 0x%06x", owner, snoop, sa,
			           T->snoop_we, T->snoop_addr);
		}
	}
	prev_cmd = cmd;
}

//------------------------------------------------------------------ d3 monitor
struct Txn { uint64_t e, req_e; bool we; uint32_t a; uint16_t wd, rd; };
static std::vector<Txn> txns;
static uint64_t d3_req_first = 0;
static bool d3_prev_req = false, d3_prev_ack = false, pend_b2b = false;
static Bulk b_b2b("after every d3 PONG read ack the d3 request stays asserted (reads are back to back)",
                  "falcon_mbox_test S_POLL access() on the ack edge");

static void monitor_d3()
{
	bool req = T->mbx_req, ack = T->mbx_ack;
	if (pend_b2b) b_b2b.ok(req, "d3 request dropped after a PONG read ack");
	pend_b2b = false;
	if (req && (!d3_prev_req || d3_prev_ack)) d3_req_first = g_edge;
	if (ack) {
		Txn x;
		x.e = g_edge; x.req_e = d3_req_first; x.we = T->mbx_we; x.a = (uint32_t)T->mbx_addr << 1;
		x.wd = T->mbx_wdata; x.rd = T->mbx_rdata;
		x.a -= MB_G;
		txns.push_back(x);
		if (!x.we && x.a == A_PONG) pend_b2b = true;
	}
	d3_prev_req = req; d3_prev_ack = ack;
}

//------------------------------------------------------------------ software echo (stand-in for the tool in COUNT/timeout tests)
static bool echo_en = false;
static uint16_t echo_last = 0, echo_pending = 0;
static int echo_dmax = 0, echo_wait = -1;
static void echo_pre()
{
	if (!echo_en) return;
	uint16_t v = gr16(A_PING);
	if (echo_wait < 0 && v != echo_last) { echo_wait = rnd(echo_dmax + 1); echo_pending = v; }
	if (echo_wait == 0) { gw16(A_PONG, echo_pending); echo_last = echo_pending; echo_wait = -1; }
	else if (echo_wait > 0) echo_wait--;
}

//------------------------------------------------------------------ clock step
static bool g_reset = true;

static void step()
{
	Snap s = snap_now();
	check_cmd_start();
	monitor_d3();
	pre_dma(dm[0]); pre_dma(dm[1]); pre_dma(dm[2]); pre_cpu(cpu); pre_vid(vid);
	ddr_pre();
	echo_pre();
	T->reset = g_reset;
	T->clk = 1; T->eval();
	post_dma(dm[0]); post_dma(dm[1]); post_dma(dm[2]); post_cpu(cpu); post_vid(vid);
	T->DDRAM_DOUT = nxt_dout; T->DDRAM_DOUT_READY = nxt_ready; T->DDRAM_BUSY = nxt_busy;
	T->clk = 0; T->eval();
	prev_snap = s;
	if (g_reset) { g_since_reset = 0; prev_cmd = false; } else g_since_reset++;
	g_edge++;
}
static void steps(uint64_t n) { while (n--) step(); }

static bool masters_idle()
{
	return !dm[0].req && !dm[1].req && !dm[2].req && !cpu.req && !vid.req && !vid.waiting_beats && rd_left == 0;
}
static void traffic(bool d0, bool d1, bool d2, bool c, bool v, int gapmax = 20, int vidgap = 40)
{
	dm[0].en = d0; dm[1].en = d1; dm[2].en = d2; cpu.en = c; vid.en = v;
	for (auto &m : dm) m.gapmax = gapmax;
	cpu.gapmax = gapmax; vid.gapmax = vidgap;
}
static void quiesce()
{
	traffic(false, false, false, false, false);
	for (int i = 0; i < 100000 && !masters_idle(); i++) step();
	steps(50);
}

//------------------------------------------------------------------ tool child process
static pid_t g_child = -1;
static void kill_child()
{
	if (g_child > 0) { kill(g_child, SIGKILL); int st; waitpid(g_child, &st, 0); g_child = -1; }
}
static double wall_s()
{
	struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
	return t.tv_sec + t.tv_nsec * 1e-9;
}
struct ToolRes { bool finished = false; int code = -1; int sig = 0; std::string out; uint64_t clocks = 0; };

static ToolRes run_tool(const std::string &tool, const std::vector<std::string> &args, uint64_t max_clk, double max_wall)
{
	ToolRes r;
	int pfd[2];
	if (pipe(pfd)) { perror("pipe"); exit(2); }
	std::vector<char *> av;
	av.push_back((char *)tool.c_str());
	for (auto &a : args) av.push_back((char *)a.c_str());
	av.push_back(nullptr);
	fflush(stdout);
	pid_t pid = fork();
	if (pid < 0) { perror("fork"); exit(2); }
	if (pid == 0) {
		dup2(pfd[1], 1); dup2(pfd[1], 2);
		close(pfd[0]); close(pfd[1]);
		int nul = open("/dev/null", O_RDONLY);
		if (nul >= 0) dup2(nul, 0);
		execv(tool.c_str(), av.data());
		_exit(127);
	}
	close(pfd[1]);
	fcntl(pfd[0], F_SETFL, O_NONBLOCK);
	g_child = pid;
	double t0 = wall_s();
	uint64_t c0 = g_edge;
	char buf[4096];
	for (;;) {
		for (int i = 0; i < 32; i++) step();
		ssize_t n;
		while ((n = read(pfd[0], buf, sizeof buf)) > 0) r.out.append(buf, n);
		int st;
		if (waitpid(pid, &st, WNOHANG) == pid) {
			g_child = -1;
			r.finished = true;
			if (WIFEXITED(st)) r.code = WEXITSTATUS(st); else { r.sig = WTERMSIG(st); }
			break;
		}
		if (g_edge - c0 > max_clk || wall_s() - t0 > max_wall) { kill_child(); break; }
	}
	while (true) {
		ssize_t n = read(pfd[0], buf, sizeof buf);
		if (n <= 0) break;
		r.out.append(buf, n);
	}
	close(pfd[0]);
	steps(300);                    // let the RTL finish its last accesses
	r.clocks = g_edge - c0;
	return r;
}

//------------------------------------------------------------------ run analysis
struct Samp { uint64_t tP, tR; uint32_t cyc; uint16_t hi, lo, polls_w, polls_obs; };
struct Run {
	bool ok = false, timed_out = false;
	size_t start = 0, end = 0;
	uint16_t run = 0, count_rd = 0, p0 = 0, stat_w = 0xFFFF, done_w = 0xFFFF;
	uint64_t start_ack = 0, done_ack = 0, tP_to = 0, tE = 0, tEprev = 0;
	std::vector<Samp> s;
	uint64_t pong_reads = 0;
};
static uint16_t g_last_run = 0;      // run id the probe finished last (its last_run register)

static bool run_fail(const char *what, size_t i)
{
	g_fail++;
	printf("  [FAIL] run structure: %s at txn %zu of %zu (edge %llu)\n", what, i, txns.size(), (unsigned long long)g_edge);
	return false;
}

static Run analyze_run(size_t from)
{
	Run r;
	size_t i = from, n = txns.size();
	while (i < n) {
		const Txn &x = txns[i];
		if (!x.we && x.a == A_CMD) {
			if (x.rd != 0 && x.rd != g_last_run) break;
			i++;
		} else { run_fail("unexpected access while idle", i); return r; }
	}
	if (i >= n) { run_fail("no run started", i); return r; }
	r.start = i; r.run = txns[i].rd; r.start_ack = txns[i].e;
	i++;
	if (i + 1 >= n || txns[i].we || txns[i].a != A_COUNT) { run_fail("COUNT read missing", i); return r; }
	r.count_rd = txns[i].rd; i++;
	if (txns[i].we || txns[i].a != A_PONG) { run_fail("initial PONG read missing", i); return r; }
	r.p0 = txns[i].rd; r.pong_reads = 1; i++;
	Bulk b_ping("PING write value = initial PONG + 1 + sample index", "falcon_mbox_test S_PONG0/S_SMP2");
	Bulk b_pre("PONG reads before the matching one return a different value", "S_POLL");
	Bulk b_smpaddr("sample words are written at +$100+8*i, +2, +4 in order", "falcon_mbox_test smp_a");
	for (uint32_t idx = 0;; idx++) {
		if (i >= n) { run_fail("run did not finish", i); return r; }
		if (txns[i].we && txns[i].a == A_STAT) break;
		uint16_t seq = (uint16_t)(r.p0 + 1 + idx);
		if (!(txns[i].we && txns[i].a == A_PING)) { run_fail("PING write expected", i); return r; }
		b_ping.ok(txns[i].wd == seq, "sample %u expected 0x%04x got 0x%04x", idx, seq, txns[i].wd);
		Samp sm; memset(&sm, 0, sizeof sm);
		sm.tP = txns[i].e;
		i++;
		bool matched = false;
		uint64_t prev_e = sm.tP;
		while (i < n && !txns[i].we && txns[i].a == A_PONG) {
			sm.polls_obs++; r.pong_reads++;
			if (txns[i].rd == seq) { matched = true; sm.tR = txns[i].e; i++; break; }
			b_pre.ok(txns[i].rd != seq);
			prev_e = txns[i].e;
			r.tE = txns[i].e;
			i++;
		}
		if (!matched) {
			if (i >= n) { run_fail("run did not finish (poll)", i); return r; }
			r.timed_out = true; r.tP_to = sm.tP; r.tEprev = prev_e;
			// r.tE is the last poll read; tEprev must be the one before it
			if (sm.polls_obs >= 2) {
				// recompute: edge of the poll before the last
				r.tEprev = txns[i - 2].e;
			} else r.tEprev = sm.tP;
			break;
		}
		if (i + 2 >= n) { run_fail("run did not finish (sample writes)", i); return r; }
		bool okw = true;
		for (int k = 0; k < 3; k++)
			okw &= txns[i + k].we && txns[i + k].a == (uint32_t)(A_SMP + 8 * idx + 2 * k);
		b_smpaddr.ok(okw, "sample %u", idx);
		sm.hi = txns[i].wd; sm.lo = txns[i + 1].wd; sm.polls_w = txns[i + 2].wd;
		sm.cyc = ((uint32_t)sm.hi << 16) | sm.lo;
		i += 3;
		r.s.push_back(sm);
	}
	b_ping.end(); b_pre.end(); b_smpaddr.end();
	if (i + 1 >= n) { run_fail("STATUS/DONE writes missing", i); return r; }
	if (!(txns[i].we && txns[i].a == A_STAT)) { run_fail("STATUS write expected", i); return r; }
	if (!(txns[i + 1].we && txns[i + 1].a == A_DONE)) { run_fail("DONE write expected right after STATUS", i + 1); return r; }
	r.stat_w = txns[i].wd; r.done_w = txns[i + 1].wd; r.done_ack = txns[i + 1].e;
	r.end = i + 2; r.ok = true;
	return r;
}

static std::vector<uint8_t> sample_init;     // initial content of the sample area (for untouched checks)

// checks common to every completed run
static std::vector<uint8_t> g_tail;      // sample-area bytes behind the last expected sample, before the run
static size_t g_tail_off = 0;
static void grab_tail(size_t nsamples)
{
	g_tail_off = A_SMP + 8 * nsamples;
	g_tail.assign(MB + g_tail_off, MB + g_tail_off + 64);
}

static void check_run(const Run &r, uint32_t exp_count, uint16_t exp_status, uint16_t exp_run, const char *tag, uint16_t count_w)
{
	char nm[160];
	if (!r.ok) return;
	snprintf(nm, sizeof nm, "%s: run id the probe started on (CMD read)", tag);
	chk_u(nm, exp_run, r.run, "id the host wrote into CMD");
	snprintf(nm, sizeof nm, "%s: COUNT word the probe read", tag);
	chk_u(nm, count_w, r.count_rd, "falcon_mbox_test S_COUNT read of what the host wrote");
	snprintf(nm, sizeof nm, "%s: number of samples the probe took (clamped COUNT)", tag);
	chk_u(nm, exp_count, r.s.size(), "falcon_mbox_test S_COUNT clamp");
	snprintf(nm, sizeof nm, "%s: STATUS word written by the probe", tag);
	chk_u(nm, exp_status, r.stat_w, "falcon_mbox_test S_STAT");
	snprintf(nm, sizeof nm, "%s: DONE word written by the probe", tag);
	chk_u(nm, exp_run, r.done_w, "falcon_mbox_test S_DONE = run id");
	snprintf(nm, sizeof nm, "%s: file STATUS", tag);
	chk_u(nm, exp_status, gr16(A_STAT), "shared file via DDR3 write path");
	snprintf(nm, sizeof nm, "%s: file DONE", tag);
	chk_u(nm, exp_run, gr16(A_DONE), "shared file via DDR3 write path");

	Bulk b_cyc("stored cycles == tR - tP - 1 (tP/tR = edges d3 ack seen for PING write / matching PONG read)", "falcon_mbox_test cyc counter");
	Bulk b_pol("stored PONG reads == d3 PONG read acks in (tP, tR]", "falcon_mbox_test polls");
	Bulk b_fil("file sample words (hi, lo, polls) equal what the RTL wrote on d3", "DDR3 byte order path");
	Bulk b_w3("4th word of each sample is not touched by the RTL", "falcon_mbox_test (never written)");
	uint64_t sumpol = 0, mn = ~0ull, mx = 0;
	for (size_t k = 0; k < r.s.size(); k++) {
		const Samp &m = r.s[k];
		uint64_t exp = m.tR - m.tP - 1;
		b_cyc.ok(m.cyc == exp, "sample %zu tP %llu tR %llu expected %llu got %u", k, (unsigned long long)m.tP,
		         (unsigned long long)m.tR, (unsigned long long)exp, m.cyc);
		b_pol.ok(m.polls_w == m.polls_obs, "sample %zu expected %u got %u", k, m.polls_obs, m.polls_w);
		unsigned o = A_SMP + 8 * (unsigned)k;
		b_fil.ok(gr16(o) == m.hi && gr16(o + 2) == m.lo && gr16(o + 4) == m.polls_w,
		         "sample %zu file %04x %04x %04x expected %04x %04x %04x", k, gr16(o), gr16(o + 2), gr16(o + 4), m.hi,
		         m.lo, m.polls_w);
		b_w3.ok(gr16(o + 6) == (uint16_t)((sample_init[o + 6 - A_SMP] << 8) | sample_init[o + 7 - A_SMP]),
		        "sample %zu word 3 changed", k);
		sumpol += m.polls_obs; mn = std::min<uint64_t>(mn, m.cyc); mx = std::max<uint64_t>(mx, m.cyc);
	}
	b_cyc.end(); b_pol.end(); b_fil.end(); b_w3.end();
	snprintf(nm, sizeof nm, "%s: PONG reads seen on d3 == 1 + sum of stored PONG-read counts", tag);
	uint64_t fsum = 0;
	for (size_t k = 0; k < r.s.size(); k++) fsum += gr16(A_SMP + 8 * (unsigned)k + 4);
	chk_u(nm, 1 + fsum, r.pong_reads, "monitor count vs file counts");
	printf("    info %s: %zu samples, cycles min %llu max %llu, PONG reads/sample mean %.2f\n", tag, r.s.size(),
	       (unsigned long long)mn, (unsigned long long)mx, r.s.empty() ? 0.0 : (double)sumpol / r.s.size());
	snprintf(nm, sizeof nm, "%s: 64 bytes after the last expected sample are untouched", tag);
	chk_ok(nm, memcmp(MB + g_tail_off, g_tail.data(), 64) == 0, "unchanged", memcmp(MB + g_tail_off, g_tail.data(), 64) == 0 ? "unchanged" : "changed", "falcon_mbox_test stops at count");
}

//------------------------------------------------------------------ file helpers
static std::string g_workdir, g_tool;

static void file_fill_init()
{
	memset(MB, 0, MB_SIZE);
	memset(MB + A_SMP, 0xEE, MB_SIZE - A_SMP);
	sample_init.assign(MB + A_SMP, MB + MB_SIZE);
}

static std::string slurp(const std::string &p)
{
	std::string s; FILE *f = fopen(p.c_str(), "r");
	if (!f) return s;
	char b[4096]; size_t n;
	while ((n = fread(b, 1, sizeof b, f)) > 0) s.append(b, n);
	fclose(f);
	return s;
}

//------------------------------------------------------------------ tests
static void power_cycle()
{
	quiesce();
	g_reset = true;
	steps(80);
	g_reset = false;
}

// pre-fill + power on + verify the power-on behaviour
static void test_power_on(const char *tag, bool refill, uint16_t stale)
{
	printf("\n== power on (%s): stale CMD 0x%04x ==\n", tag, stale);
	if (refill) {
		file_fill_init();
		gw16(A_COUNT, 3); gw16(A_DONE, stale); gw16(A_PING, 0x0500); gw16(A_PONG, 0x0777);
	}
	gw16(A_CMD, stale); gw16(A_MAGIC, 0xDEAD); gw16(A_CLK, 0xBEEF);
	uint16_t done0 = gr16(A_DONE), cnt0 = gr16(A_COUNT);
	quiesce();
	size_t t0 = txns.size();
	g_reset = true; steps(80); g_reset = false;
	uint64_t span = 6ull * IDLE_CLKS + 400;
	steps(span);
	const Txn *x = &txns[t0];
	bool have3 = txns.size() >= t0 + 3;
	chk_ok("power-on writes exist", have3, ">=3 d3 writes", have3 ? "3+" : "fewer", "falcon_mbox_test S_MAGIC,S_CLK,S_CLRC");
	if (!have3) return;
	chk_u("1st d3 access is a write of MAGIC", (1ull << 32) | (A_MAGIC << 16) | 0x4D42, ((uint64_t)x[0].we << 32) | (x[0].a << 16) | x[0].wd, "header: MAGIC $4D42 written at power on");
	chk_u("2nd d3 access is a write of CLKMHZ", (1ull << 32) | (A_CLK << 16) | (CLK_HZ / 1000000), ((uint64_t)x[1].we << 32) | (x[1].a << 16) | x[1].wd, "header: CLKMHZ = CLK_HZ/1e6");
	chk_u("3rd d3 access is a write of CMD = 0", (1ull << 32) | (A_CMD << 16) | 0, ((uint64_t)x[2].we << 32) | (x[2].a << 16) | x[2].wd, "header: CMD is cleared at power on");
	chk_u("file MAGIC", 0x4D42, gr16(A_MAGIC), "mailbox +$00C read through the swapped byte path");
	chk_u("file CLKMHZ", CLK_HZ / 1000000, gr16(A_CLK), "mailbox +$00E");
	chk_u("file CMD", 0, gr16(A_CMD), "stale run id must be cleared");
	chk_u("file DONE untouched by power on", done0, gr16(A_DONE), "no run started");
	chk_u("file COUNT untouched by power on", cnt0, gr16(A_COUNT), "no run started");
	size_t nrd = txns.size() - t0 - 3;
	chk_ok("idle CMD polls happened", nrd >= 5, ">=5 CMD reads in 6 poll periods", nrd >= 5 ? ">=5" : "fewer", "falcon_mbox_test S_IDLE");
	Bulk b_idle("every later d3 access is a CMD read returning 0 (no run starts by itself)", "falcon_mbox_test S_IDLE/S_CMD");
	Bulk b_per("CMD poll: request appears exactly CLK_HZ/1000 + 2 edges after the previous ack", "S_IDLE idle_t counter");
	for (size_t k = t0 + 3; k < txns.size(); k++) {
		b_idle.ok(!txns[k].we && txns[k].a == A_CMD && txns[k].rd == 0, "txn %zu we %d a 0x%x rd 0x%04x", k,
		          txns[k].we, txns[k].a, txns[k].rd);
		b_per.ok(txns[k].req_e - txns[k - 1].e == IDLE_CLKS + 2, "txn %zu expected %u got %llu", k, IDLE_CLKS + 2,
		         (unsigned long long)(txns[k].req_e - txns[k - 1].e));
	}
	b_idle.end(); b_per.end();
	g_last_run = 0;
}

static std::string csv_path() { return g_workdir + "/ping.csv"; }

static void check_tool_stats(const Run &r, const ToolRes &tr, int n, const char *tag)
{
	char nm[200];
	unsigned mhz = CLK_HZ / 1000000;
	int pn = -1; unsigned pm = 0;
	size_t p = tr.out.find("mbox_ping: ");
	if (p != std::string::npos) sscanf(tr.out.c_str() + p, "mbox_ping: %d round trips, FPGA clock %u MHz", &pn, &pm);
	snprintf(nm, sizeof nm, "%s: count printed by the tool", tag);
	chk_u(nm, n, (uint64_t)(unsigned)pn, "tool stdout 'N round trips'");
	snprintf(nm, sizeof nm, "%s: FPGA clock MHz printed by the tool (CLKMHZ through the RTL path)", tag);
	chk_u(nm, mhz, pm, "tool reads +$00E");
	if (!r.ok || (int)r.s.size() != n) return;
	// recompute the tool's statistics lines from the RTL's samples
	std::vector<uint32_t> cyc(n), sorted(n);
	double sum = 0, psum = 0, rsum = 0, rmin = 1e30;
	std::vector<uint16_t> pol(n);
	for (int i = 0; i < n; i++) {
		cyc[i] = r.s[i].cyc; pol[i] = r.s[i].polls_w; sorted[i] = cyc[i];
		sum += cyc[i]; psum += pol[i];
		if (pol[i]) { double q = (double)cyc[i] / pol[i]; rsum += q; if (q < rmin) rmin = q; }
	}
	std::sort(sorted.begin(), sorted.end());
	auto pct = [&](double q) { return (double)sorted[(int)(q * (n - 1) + 0.5)]; };
	double us = 1.0 / mhz;
	char l1[300], l2[300];
	snprintf(l1, sizeof l1, "round trip (us): min %.2f  median %.2f  p90 %.2f  p99 %.2f  p99.9 %.2f  max %.2f  mean %.2f",
	         sorted[0] * us, pct(0.5) * us, pct(0.9) * us, pct(0.99) * us, pct(0.999) * us, sorted[n - 1] * us, sum / n * us);
	snprintf(l2, sizeof l2, "FPGA reads of PONG per round trip: mean %.1f; one read takes %.2f us (min %.2f)", psum / n,
	         rsum / n * us, rmin * us);
	snprintf(nm, sizeof nm, "%s: tool round-trip statistics line", tag);
	chk_ok(nm, tr.out.find(l1) != std::string::npos, l1, tr.out.find(l1) != std::string::npos ? "same" : tr.out.c_str(), "recomputed from RTL samples");
	snprintf(nm, sizeof nm, "%s: tool PONG-reads statistics line", tag);
	chk_ok(nm, tr.out.find(l2) != std::string::npos, l2, tr.out.find(l2) != std::string::npos ? "same" : tr.out.c_str(), "recomputed from RTL samples");
	// CSV
	std::string csv = slurp(csv_path());
	Bulk b_csv("tool CSV row equals the sample the RTL wrote (cycles, us, pong_reads)", "mbox_ping.c -o");
	size_t pos = csv.find('\n');
	chk_ok("CSV header", csv.compare(0, pos, "sample,cycles,us,pong_reads") == 0, "sample,cycles,us,pong_reads", csv.substr(0, pos).c_str(), "mbox_ping.c");
	int rows = 0;
	while (pos != std::string::npos && pos + 1 < csv.size()) {
		size_t nx = csv.find('\n', pos + 1);
		std::string line = csv.substr(pos + 1, nx == std::string::npos ? std::string::npos : nx - pos - 1);
		pos = nx;
		int si; unsigned c, pr; double uu;
		if (sscanf(line.c_str(), "%d,%u,%lf,%u", &si, &c, &uu, &pr) == 4 && si == rows && si < n) {
			char e[100];
			snprintf(e, sizeof e, "%d,%u,%.3f,%u", si, r.s[si].cyc, r.s[si].cyc * us, r.s[si].polls_w);
			b_csv.ok(line == e, "expected '%s' got '%s'", e, line.c_str());
		} else
			b_csv.ok(false, "bad line '%s'", line.c_str());
		rows++;
	}
	b_csv.ok(rows == n, "rows expected %d got %d", n, rows);
	b_csv.end();
}

static void tool_run_test(const char *tag, int n, uint16_t exp_run)
{
	printf("\n== tool run: %s (n=%d) ==\n", tag, n);
	size_t t0 = txns.size();
	grab_tail(n);
	unlink(csv_path().c_str());
	ToolRes tr = run_tool(g_tool, { "-m", g_workdir + "/mailbox.bin", "-n", std::to_string(n), "-o", csv_path() },
	                      60ull * 1000 * 1000, 90.0);
	printf("  tool output: %s", tr.out.c_str());
	chk_ok("tool finished", tr.finished, "exits", tr.finished ? "exited" : "killed (timeout)", "real mbox_ping_host");
	chk_u("tool exit status", 0, tr.finished ? (uint64_t)tr.code : 0xFFFF, "mbox_ping.c main");
	Run r = analyze_run(t0);
	if (r.ok && r.timed_out) {
		char got[200];
		snprintf(got, sizeof got, "timed out: STATUS 1, no echo seen (initial PONG read 0x%04x, first PING written 0x%04x, file PING 0x%04x)",
		         r.p0, (uint16_t)(r.p0 + 1), gr16(A_PING));
		chk_ok("run completes with STATUS 0 (the tool must echo the first PING)", false, "STATUS 0, DONE = run id", got,
		       "tool reads PING at start as 'last'; probe writes PONG+1 which equals it when PING == PONG+1");
		g_last_run = r.done_w;
	} else if (r.ok) {
		check_run(r, n, 0, exp_run, tag, n);
		check_tool_stats(r, tr, n, tag);
		chk_u("CMD in the file is the run id the tool wrote", exp_run, gr16(A_CMD), "tool wr16(O_CMD)");
		g_last_run = r.done_w;
	}
}

static void cpp_run(const char *tag, uint16_t run, uint16_t count_written, uint32_t exp_count, int echo_dmax_)
{
	printf("\n== C++ driven run: %s (COUNT written %u, expected samples %u) ==\n", tag, count_written, exp_count);
	size_t t0 = txns.size();
	grab_tail(exp_count);
	echo_last = gr16(A_PING); echo_en = true; echo_dmax = echo_dmax_; echo_wait = -1;
	gw16(A_COUNT, count_written);
	gw16(A_CMD, run);
	uint64_t lim = 200ull * 1000 * 1000, c0 = g_edge;
	while (g_edge - c0 < lim) {
		step();
		if (!txns.empty() && txns.size() > t0 && txns.back().we && txns.back().a == A_DONE) break;
	}
	echo_en = false;
	steps(200);
	Run r = analyze_run(t0);
	if (r.ok) {
		check_run(r, exp_count, 0, run, tag, count_written);
		g_last_run = r.done_w;
	}
}

static void test_timeout(uint16_t run)
{
	printf("\n== timeout: CMD written from C++, nobody echoes ==\n");
	size_t t0 = txns.size();
	grab_tail(0);
	gw16(A_COUNT, 5);
	gw16(A_CMD, run);
	uint16_t ping0 = gr16(A_PING), pong0 = gr16(A_PONG);
	uint64_t c0 = g_edge;
	while (g_edge - c0 < 3ull * CLK_HZ + 20000) {
		step();
		if (txns.size() > t0 && txns.back().we && txns.back().a == A_DONE) break;
	}
	steps(200);
	Run r = analyze_run(t0);
	if (!r.ok) return;
	chk_ok("run ended through the timeout path", r.timed_out, "timed out", r.timed_out ? "timed out" : "completed", "falcon_mbox_test S_POLL");
	chk_u("STATUS written", 1, r.stat_w, "header: STATUS 1 = no answer within ~1 s");
	chk_u("DONE written", run, r.done_w, "header: DONE = run id");
	chk_u("file STATUS", 1, gr16(A_STAT), "DDR3 write path");
	chk_u("file DONE", run, gr16(A_DONE), "DDR3 write path");
	chk_u("samples recorded", 0, r.s.size(), "no PONG ever matched");
	chk_u("PING word written", (uint16_t)(pong0 + 1), gr16(A_PING), "S_PONG0: PONG + 1");
	(void)ping0;
	chk_u("last poll ack satisfies tE - tP - 1 >= CLK_HZ", 1, (r.tE - r.tP_to - 1) >= CLK_HZ, "S_POLL: cyc >= TMO_CLKS");
	chk_u("previous poll ack has tE - tP - 1 < CLK_HZ", 1, (r.tEprev - r.tP_to - 1) < CLK_HZ, "S_POLL: not timed out yet");
	printf("    info timeout: tP %llu, last poll ack %llu (tE-tP-1 = %llu, CLK_HZ %u)\n", (unsigned long long)r.tP_to,
	       (unsigned long long)r.tE, (unsigned long long)(r.tE - r.tP_to - 1), CLK_HZ);
	chk_ok("sample area untouched by a timed-out run", memcmp(MB + g_tail_off, g_tail.data(), 64) == 0, "unchanged",
	       memcmp(MB + g_tail_off, g_tail.data(), 64) == 0 ? "unchanged" : "changed", "no sample is stored without an echo");
	g_last_run = r.done_w;
}

static void tool_error_tests()
{
	printf("\n== tool error paths (no simulation needed) ==\n");
	std::string z = g_workdir + "/zero.bin";
	{
		int fd = open(z.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
		if (fd < 0 || ftruncate(fd, MB_SIZE)) { perror("zero file"); exit(2); }
		close(fd);
	}
	ToolRes tr = run_tool(g_tool, { "-m", z, "-n", "5" }, 200000, 20.0);
	printf("  tool output: %s", tr.out.c_str());
	chk_ok("tool with MAGIC missing exits", tr.finished, "exits", tr.finished ? "exited" : "hung", "mbox_ping.c MAGIC test");
	chk_u("exit status non-zero (tool returns 1)", 1, (uint64_t)tr.code, "mbox_ping.c");
	chk_ok("message names the missing mailbox probe", tr.out.find("no mailbox probe (MAGIC = 0000, expected 4D42)") != std::string::npos,
	       "no mailbox probe (MAGIC = 0000, expected 4D42)", tr.out.c_str(), "mbox_ping.c");
	tr = run_tool(g_tool, { "-m", g_workdir + "/mailbox.bin", "-n", "0" }, 200000, 20.0);
	chk_u("tool -n 0 exit status", 2, (uint64_t)tr.code, "mbox_ping.c: -n must be 1..32768");
	chk_ok("tool -n 0 message", tr.out.find("-n must be 1..32768") != std::string::npos, "-n must be 1..32768", tr.out.c_str(), "mbox_ping.c");
}

static void test_priority_starvation(uint16_t run)
{
	printf("\n== priorities under saturation: d2 > d3 > cpu ==\n");
	quiesce();
	// phase a: d2 and cpu request back to back; d3 has an idle CMD poll pending
	steps(IDLE_CLKS + 200);
	traffic(false, false, true, true, false, 0);
	uint64_t c0 = g_edge;
	while (dm[2].nacks < 3 && g_edge - c0 < 100000) step();
	size_t t3 = txns.size(); uint64_t cn = cpu.nacks, dn = dm[2].nacks;
	steps(20ull * IDLE_CLKS);
	chk_u("d3 acks while d2 saturates the arbiter", 0, txns.size() - t3, "d2 above d3: d3 starves");
	chk_u("cpu acks while d2 saturates the arbiter", 0, cpu.nacks - cn, "d2 above cpu");
	chk_ok("d2 got the bandwidth", dm[2].nacks - dn > 500, "> 500 d2 acks", dm[2].nacks - dn > 500 ? "> 500" : "fewer", "memarb");
	chk_ok("d3 request is pending during the starvation", T->mbx_req, "mbx_req=1", T->mbx_req ? "mbx_req=1" : "mbx_req=0", "falcon_mbox_test");
	// release d2: d3 (pending CMD poll) must be served before the cpu
	traffic(false, false, false, true, false, 0);
	size_t t3b = txns.size(); uint64_t cn2 = cpu.nacks;
	uint64_t c1 = g_edge, e_d3 = 0, e_cpu = 0;
	while (g_edge - c1 < 20000 && !(e_d3 && e_cpu)) {
		step();
		if (!e_d3 && txns.size() > t3b) e_d3 = g_edge;
		if (!e_cpu && cpu.nacks > cn2) e_cpu = g_edge;
	}
	chk_ok("after d2 stops, d3 and cpu are both served", e_d3 && e_cpu, "both acked", (e_d3 && e_cpu) ? "both acked" : "starved", "memarb");
	chk_ok("d3 served before cpu", e_d3 && e_d3 < e_cpu, "d3 ack edge < cpu ack edge", "see edges", "d3 above cpu");
	printf("    info: d3 first ack edge %llu, cpu first ack edge %llu\n", (unsigned long long)e_d3, (unsigned long long)e_cpu);
	// phase b: a d3 run with the cpu saturating: cpu must not get a slot while d3 requests
	size_t t0 = txns.size();
	grab_tail(300);
	echo_last = gr16(A_PING); echo_en = true; echo_dmax = 2; echo_wait = -1;
	gw16(A_COUNT, 300); gw16(A_CMD, run);
	uint64_t c2 = g_edge;
	while (g_edge - c2 < 5000000) {
		step();
		if (txns.size() > t0 && txns.back().we && txns.back().a == A_DONE) break;
	}
	echo_en = false;
	steps(300);
	Run r = analyze_run(t0);
	if (r.ok) {
		unsigned cin = 0;
		for (uint64_t e : cpu_ack_edges) if (e > r.start_ack && e < r.done_ack) cin++;
		chk_u("cpu acks between the run's first CMD ack and DONE ack (d3 request held high)", 0, cin, "d3 above cpu");
		chk_ok("cpu runs again after the run", cpu.nacks > cn2 && cpu_ack_edges.back() > r.done_ack, "cpu ack after DONE", "see edges", "memarb");
		check_run(r, 300, 0, run, "saturated cpu", 300);
		g_last_run = r.done_w;
	}
	quiesce();
}

//------------------------------------------------------------------ main
int main(int argc, char **argv)
{
	setvbuf(stdout, nullptr, _IOLBF, 0);
	Verilated::commandArgs(argc, argv);
	if (argc < 3) { fprintf(stderr, "usage: %s TOOL WORKDIR [quick]\n", argv[0]); return 2; }
	g_tool = argv[1]; g_workdir = argv[2];
	bool quick = argc > 3 && !strcmp(argv[3], "quick");
	signal(SIGINT, [](int) { kill_child(); _exit(130); });
	atexit(kill_child);

	std::string mbf = g_workdir + "/mailbox.bin";
	int fd = open(mbf.c_str(), O_RDWR | O_CREAT | O_TRUNC, 0644);
	if (fd < 0 || ftruncate(fd, MB_SIZE)) { perror("mailbox file"); return 2; }
	MB = (uint8_t *)mmap(nullptr, MB_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (MB == MAP_FAILED) { perror("mmap"); return 2; }

	ddr_mem.assign(DDR_SIZE, 0);
	shadow.assign(DDR_SIZE, 0);
	for (uint32_t base : { 0x100000u, 0x200000u, 0x300000u, 0x400000u, 0x500000u })
		for (uint32_t i = 0; i < 0x2000; i++) ddr_mem[base + i] = shadow[base + i] = (uint8_t)rng();
	dm[0].chk = &b_d0; dm[1].chk = &b_d1; dm[2].chk = &b_d2; cpu.chk = &b_cpu; vid.chk = &b_vid;

	T = new Vtb_mbox_top;
	T->clk = 0; T->reset = 1; T->vid_req = 0; T->d0_req = T->d1_req = T->d2_req = 0; T->cpu_req = 0;
	T->DDRAM_BUSY = 0; T->DDRAM_DOUT_READY = 0; T->DDRAM_DOUT = 0;
	T->eval();

	printf("tb_mbox: CLK_HZ %u (idle CMD poll every %u clocks, timeout %u clocks)\n", CLK_HZ, IDLE_CLKS, CLK_HZ);

	uint16_t run0 = 0x1235;
	test_power_on("fresh file with stale run id", true, 0x1234);

	tool_run_test("first run, no other traffic", 100, run0);
	if (quick) {
		tool_run_test("second invocation", 20, run0 + 1);
	} else {
		tool_run_test("second invocation, run id increments", 64, run0 + 1);

		// traffic on every other master, vid bursts included
		traffic(true, true, true, true, true, 150, 600);
		tool_run_test("with random traffic on cpu,d0,d1,d2,vid", 300, run0 + 2);
		steps(150000);              // keep all masters busy a while longer (vid, d0..d2, cpu, d3 idle polls)
		quiesce();
		printf("\n== traffic summary ==\n");
		for (int k = 0; k < 3; k++)
			printf("  d%d: %llu reads, %llu writes\n", k, (unsigned long long)dm[k].nrd, (unsigned long long)dm[k].nwr);
		printf("  cpu: %llu reads, %llu writes; vid bursts %llu\n", (unsigned long long)cpu.nrd,
		       (unsigned long long)cpu.nwr, (unsigned long long)vid.nbursts);
		printf("  grants: vid %llu d0 %llu d1 %llu d2 %llu d3 %llu cpu %llu; DDR reads %llu writes %llu\n",
		       (unsigned long long)n_grant[0], (unsigned long long)n_grant[1], (unsigned long long)n_grant[2],
		       (unsigned long long)n_grant[3], (unsigned long long)n_grant[4], (unsigned long long)n_grant[5],
		       (unsigned long long)n_ddr_rd, (unsigned long long)n_ddr_wr);
		chk_ok("traffic reached every master", dm[0].nacks > 100 && dm[1].nacks > 100 && dm[2].nacks > 100 && cpu.nacks > 100 && vid.nbursts > 50,
		       "> 100 ops per master, > 50 vid bursts", "see summary", "generators");

		test_priority_starvation(0x3001);

		// COUNT clamping, driven from C++ with a software echo
		cpp_run("COUNT 0 -> 1", 0x2001, 0, 1, 6);
		cpp_run("COUNT 2 (no clamp)", 0x2002, 2, 2, 6);
		cpp_run("COUNT 32769 -> 32768", 0x2003, 32769, 32768, 3);

		test_timeout(0x2004);
		// a tool run right after a timed-out run (PING/PONG left unequal by the timeout;
		// mbox_ping resynchronises with PONG = PING before it starts a run)
		tool_run_test("tool run after a timed-out run", 30, 0x2005);

		// power on again with a stale CMD in the DDR3
		test_power_on("second power on, CMD left at the last run id", false, gr16(A_CMD) ? gr16(A_CMD) : 0x2005);
		tool_run_test("tool run after the second power on", 30, gr16(A_DONE) + 1);
		tool_error_tests();
	}

	printf("\n== summary of bulk protocol checkers ==\n");
	b_ddr.end(); b_prio.end(); b_snoop.end(); b_b2b.end();
	b_cpu.end(); b_d0.end(); b_d1.end(); b_d2.end(); b_vid.end();
	printf("  reads compared: cpu %llu d0 %llu d1 %llu d2 %llu vid beats %llu\n", (unsigned long long)b_cpu.n,
	       (unsigned long long)b_d0.n, (unsigned long long)b_d1.n, (unsigned long long)b_d2.n, (unsigned long long)b_vid.n);
	if (quick) { /* nothing more */ }
	printf("\nchecks: %d passed, %d failed\n", g_pass, g_fail);
	bool ok = g_fail == 0;
	printf("RESULT: %s (CLK_HZ %u)\n", ok ? "PASS" : "FAIL", CLK_HZ);
	kill_child();
	T->final();
	delete T;
	return ok ? 0 : 1;
}
