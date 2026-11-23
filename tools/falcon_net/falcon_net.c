/*
 * falcon_net: the HPS side of the core's DaynaPORT (rtl/falcon/falcon_scsi.sv, SCSI ID 4).
 *
 * The core keeps two rings of 16 slots in DDR3 at 0x30EA0000 (guest $EA0000); 16-bit
 * words in guest (big endian) byte order:
 *   +$0000 MAGIC $444E ("DN", written here)   +$0002 HEARTBEAT (here, every second)
 *   +$0010 RX_HEAD (here)   +$0012 RX_TAIL (core)   +$0014 TX_HEAD (core)
 *   +$0016 TX_TAIL (here)   +$0018 ENABLED (core: the driver's 0E command)
 *   +$1000 + n*$800 receive slot n: +0 answer length, +4 the answer to a receive (08)
 *   +$9000 + n*$800 send slot n: +0 frame length, +2 CDB byte 5, +4 the frame
 * A receive answer is built as the sp1 Hatari fork's daynaport.c builds it: a 6-byte
 * header [length of frame + CRC, big endian][0 0 0][0], the frame (runts padded to 60)
 * and its CRC-32 (little endian).  Frames go to and from a TAP interface.
 *
 * With -a ADDR the guest gets ADDR on the uplink's network without NAT (the MiSTer's
 * kernel has none) and without bridging (a WiFi client cannot bridge): its DHCP
 * requests are answered here with ADDR, the uplink's netmask and gateway, and the
 * MiSTer routes to it with proxy ARP (a host route to ADDR on the TAP, proxy_arp on
 * both interfaces, ip_forward), all put back on exit.
 *
 *   falcon_net [-a ADDR] [-u wlan0] [-i dpnet0] [-g GW] [-n DNS] [-c CPU] [-v]
 *   falcon_net -m FILE ...          map FILE instead of /dev/mem (testing)
 */
#define _GNU_SOURCE
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/if_tun.h>
#include <net/if.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#define NET_PHYS   0x30EA0000u
#define NET_SIZE   0x11000u
#define RX_HEAD    0x10
#define RX_TAIL    0x12
#define TX_HEAD    0x14
#define TX_TAIL    0x16
#define ENABLED    0x18
#define RX_SLOT(n) (0x1000u + (n) * 0x800u)
#define TX_SLOT(n) (0x9000u + (n) * 0x800u)
#define FRAME_MAX  1518

static volatile uint8_t *mb;
static int verbose;
static volatile sig_atomic_t quit;
static const uint8_t guest_mac[6] = { 0x00, 0x80, 0x19, 0x4D, 0x53, 0x54 };   /* falcon_scsi G_MAC */

static unsigned rd16(unsigned o) { return mb[o] << 8 | mb[o + 1]; }
static void wr16(unsigned o, unsigned v) { mb[o] = v >> 8; mb[o + 1] = v; }

static uint32_t crc32(const uint8_t *p, int n)
{
	uint32_t crc = 0xffffffff;
	while (n--) {
		crc ^= *p++;
		for (int i = 0; i < 8; i++)
			crc = (crc >> 1) ^ (0xedb88320 & -(crc & 1));
	}
	return ~crc;
}

/* ------------------------------------------------------------------------- */
/* the receive ring */

static unsigned rx_head;

static void rx_queue(const uint8_t *frame, int n)
{
	uint8_t a[6 + FRAME_MAX + 4];
	if (!(rd16(ENABLED) & 1)) return;                       /* driver has it disabled */
	if (((rx_head + 1) & 15) == (rd16(RX_TAIL) & 15)) {     /* full: lost, as on a wire */
		if (verbose) fprintf(stderr, "falcon_net: receive ring full, %d bytes dropped\n", n);
		return;
	}
	if (n > FRAME_MAX) n = FRAME_MAX;
	memset(a, 0, sizeof a);
	memcpy(a + 6, frame, n);
	if (n < 60) n = 60;
	a[0] = (n + 4) >> 8;
	a[1] = (n + 4) & 0xff;
	uint32_t crc = crc32(a + 6, n);
	a[6 + n] = crc; a[7 + n] = crc >> 8; a[8 + n] = crc >> 16; a[9 + n] = crc >> 24;
	unsigned len = 6 + n + 4, slot = RX_SLOT(rx_head);
	for (unsigned i = 0; i < len; i++) mb[slot + 4 + i] = a[i];
	wr16(slot, len);
	__sync_synchronize();
	rx_head = (rx_head + 1) & 15;
	wr16(RX_HEAD, rx_head);
	if (verbose > 1) fprintf(stderr, "falcon_net: in %d bytes\n", n);
}

/* ------------------------------------------------------------------------- */
/* DHCP for the guest (-a) */

static uint32_t g_addr, g_mask, g_gw, g_dns, g_server;   /* network order */

static uint16_t ipsum(const uint8_t *p, int n)
{
	uint32_t s = 0;
	for (int i = 0; i < n; i += 2) s += p[i] << 8 | p[i + 1];
	while (s >> 16) s = (s & 0xffff) + (s >> 16);
	return ~s;
}

static int dhcp_answer(const uint8_t *f, int n)
{
	/* Ethernet IPv4 UDP 68 -> 67, BOOTREQUEST */
	if (!g_addr || n < 14 + 20 + 8 + 240) return 0;
	if (f[12] != 0x08 || f[13] != 0x00 || f[23] != 17) return 0;
	int ihl = (f[14] & 15) * 4;
	const uint8_t *u = f + 14 + ihl, *b = u + 8;
	if ((u[2] << 8 | u[3]) != 67 || b[0] != 1) return 0;
	if (b + 240 > f + n || memcmp(b + 236, "\x63\x82\x53\x63", 4)) return 0;
	int type = 0;
	for (const uint8_t *o = b + 240; o < f + n && *o != 255; o += (*o ? o[1] + 2 : 1))
		if (o[0] == 53) type = o[2];
	if (type != 1 && type != 3) return 1;                   /* other DHCP: dropped */

	uint8_t r[14 + 20 + 8 + 300];
	memset(r, 0, sizeof r);
	uint8_t *ip = r + 14, *ud = ip + 20, *bo = ud + 8;
	memcpy(r, b + 28, 6);                                   /* to the client's MAC */
	memcpy(r + 6, "\x02\x00\x46\x4e\x45\x54", 6);
	r[12] = 0x08;
	bo[0] = 2; bo[1] = 1; bo[2] = 6;
	memcpy(bo + 4, b + 4, 4);                               /* xid */
	memcpy(bo + 10, b + 10, 2);                             /* flags */
	memcpy(bo + 16, &g_addr, 4);                            /* yiaddr */
	memcpy(bo + 20, &g_server, 4);                          /* siaddr */
	memcpy(bo + 28, b + 28, 16);                            /* chaddr */
	memcpy(bo + 236, "\x63\x82\x53\x63", 4);
	uint8_t *o = bo + 240;
	*o++ = 53; *o++ = 1; *o++ = (type == 1) ? 2 : 5;        /* OFFER / ACK */
	*o++ = 54; *o++ = 4; memcpy(o, &g_server, 4); o += 4;
	*o++ = 51; *o++ = 4; *o++ = 0; *o++ = 1; *o++ = 0x51; *o++ = 0x80;   /* a day */
	*o++ = 1; *o++ = 4; memcpy(o, &g_mask, 4); o += 4;
	*o++ = 3; *o++ = 4; memcpy(o, &g_gw, 4); o += 4;
	*o++ = 6; *o++ = 4; memcpy(o, &g_dns, 4); o += 4;
	*o++ = 255;
	ud[0] = 0; ud[1] = 67; ud[2] = 0; ud[3] = 68;
	ud[4] = (8 + 300) >> 8; ud[5] = (8 + 300) & 0xff;       /* checksum 0: none */
	ip[0] = 0x45; ip[2] = (20 + 8 + 300) >> 8; ip[3] = (20 + 8 + 300) & 0xff;
	ip[8] = 64; ip[9] = 17;
	memcpy(ip + 12, &g_server, 4);
	memset(ip + 16, 0xff, 4);
	uint16_t s = ipsum(ip, 20);
	ip[10] = s >> 8; ip[11] = s;
	if (verbose) {
		char a[16];
		inet_ntop(AF_INET, &g_addr, a, sizeof a);
		fprintf(stderr, "falcon_net: DHCP %s -> %s %s\n", type == 1 ? "DISCOVER" : "REQUEST",
		        type == 1 ? "OFFER" : "ACK", a);
	}
	rx_queue(r, sizeof r);
	return 1;
}

/* ------------------------------------------------------------------------- */
/* proxy ARP routing (-a) */

static char ifname[IFNAMSIZ] = "dpnet0", upname[IFNAMSIZ] = "wlan0";
static char old_fwd = '0', old_pa_up = '0';
static int routed;

static char sysctl_rw(const char *path, char v)
{
	char old = '0';
	int fd = open(path, O_RDWR);
	if (fd < 0) { fprintf(stderr, "falcon_net: %s: %s\n", path, strerror(errno)); return old; }
	if (read(fd, &old, 1) != 1) old = '0';
	if (v && (lseek(fd, 0, SEEK_SET), write(fd, &v, 1)) != 1)
		fprintf(stderr, "falcon_net: %s: %s\n", path, strerror(errno));
	close(fd);
	return old;
}

static void run(const char *fmt, const char *a, const char *b)
{
	char cmd[256];
	snprintf(cmd, sizeof cmd, fmt, a, b);
	if (verbose) fprintf(stderr, "falcon_net: %s\n", cmd);
	if (system(cmd) != 0) fprintf(stderr, "falcon_net: failed: %s\n", cmd);
}

static void route_up(void)
{
	char a[16], s[16], p[96];
	inet_ntop(AF_INET, &g_addr, a, sizeof a);
	inet_ntop(AF_INET, &g_server, s, sizeof s);
	run("ip link set %s up%s", ifname, "");
	run("ip addr add %s/32 dev %s", s, ifname);
	run("ip route add %s/32 dev %s", a, ifname);
	old_fwd = sysctl_rw("/proc/sys/net/ipv4/ip_forward", '1');
	snprintf(p, sizeof p, "/proc/sys/net/ipv4/conf/%s/proxy_arp", upname);
	old_pa_up = sysctl_rw(p, '1');
	snprintf(p, sizeof p, "/proc/sys/net/ipv4/conf/%s/proxy_arp", ifname);
	sysctl_rw(p, '1');
	routed = 1;
}

static void route_down(void)
{
	char a[16], p[96];
	if (!routed) return;
	inet_ntop(AF_INET, &g_addr, a, sizeof a);
	run("ip route del %s/32 dev %s", a, ifname);
	sysctl_rw("/proc/sys/net/ipv4/ip_forward", old_fwd);
	snprintf(p, sizeof p, "/proc/sys/net/ipv4/conf/%s/proxy_arp", upname);
	sysctl_rw(p, old_pa_up);
	routed = 0;
}

/* the uplink's address and netmask; the gateway from /proc/net/route */
static int uplink(void)
{
	struct ifreq ifr;
	int s = socket(AF_INET, SOCK_DGRAM, 0);
	memset(&ifr, 0, sizeof ifr);
	memcpy(ifr.ifr_name, upname, IFNAMSIZ - 1);
	if (ioctl(s, SIOCGIFADDR, &ifr) < 0) { close(s); return -1; }
	g_server = ((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr.s_addr;
	if (ioctl(s, SIOCGIFNETMASK, &ifr) == 0 && !g_mask)
		g_mask = ((struct sockaddr_in *)&ifr.ifr_netmask)->sin_addr.s_addr;
	close(s);
	FILE *f = fopen("/proc/net/route", "r");
	char line[256], dev[IFNAMSIZ + 1];
	unsigned dst, gw;
	while (f && !g_gw && fgets(line, sizeof line, f))
		if (sscanf(line, "%16s %x %x", dev, &dst, &gw) == 3 && !strcmp(dev, upname) && dst == 0)
			g_gw = gw;
	if (f) fclose(f);
	if (!g_dns) g_dns = g_gw;
	return 0;
}

/* ------------------------------------------------------------------------- */

static int tap_open(void)
{
	struct ifreq ifr;
	int fd = open("/dev/net/tun", O_RDWR | O_NONBLOCK);
	if (fd < 0) { perror("falcon_net: /dev/net/tun"); return -1; }
	memset(&ifr, 0, sizeof ifr);
	ifr.ifr_flags = IFF_TAP | IFF_NO_PI;
	memcpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
	if (ioctl(fd, TUNSETIFF, &ifr) < 0) { perror("falcon_net: TUNSETIFF"); close(fd); return -1; }
	memcpy(ifname, ifr.ifr_name, IFNAMSIZ);
	return fd;
}

static void on_signal(int s) { (void)s; quit = 1; }

int main(int argc, char **argv)
{
	const char *memfile = NULL;
	int c, cpu = -1;
	while ((c = getopt(argc, argv, "a:u:i:g:n:c:m:v")) != -1) {
		switch (c) {
		case 'a': inet_pton(AF_INET, optarg, &g_addr); break;
		case 'u': snprintf(upname, sizeof upname, "%s", optarg); break;
		case 'i': snprintf(ifname, sizeof ifname, "%s", optarg); break;
		case 'g': inet_pton(AF_INET, optarg, &g_gw); break;
		case 'n': inet_pton(AF_INET, optarg, &g_dns); break;
		case 'c': cpu = atoi(optarg); break;
		case 'm': memfile = optarg; break;
		case 'v': verbose++; break;
		default:
			fprintf(stderr, "usage: falcon_net [-a ADDR] [-u wlan0] [-i dpnet0] [-g GW] [-n DNS] [-c CPU] [-m FILE] [-v]\n");
			return 2;
		}
	}
	if (cpu >= 0) {
		cpu_set_t set;
		CPU_ZERO(&set);
		CPU_SET(cpu, &set);
		sched_setaffinity(0, sizeof set, &set);
	}
	int mfd = memfile ? open(memfile, O_RDWR) : open("/dev/mem", O_RDWR | O_SYNC);
	if (mfd < 0) { perror("falcon_net: open memory"); return 1; }
	mb = mmap(NULL, NET_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, memfile ? 0 : NET_PHYS);
	if (mb == MAP_FAILED) { perror("falcon_net: mmap"); return 1; }

	if (g_addr && uplink() < 0) {
		fprintf(stderr, "falcon_net: no IPv4 address on %s\n", upname);
		return 1;
	}
	int tap = tap_open();
	if (tap < 0) return 1;
	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);
	if (g_addr) route_up();
	else run("ip link set %s up%s", ifname, "");

	/* start where the core is: nothing queued either way */
	rx_head = rd16(RX_TAIL) & 15;
	wr16(RX_HEAD, rx_head);
	unsigned tx_tail = rd16(TX_HEAD) & 15;
	wr16(TX_TAIL, tx_tail);
	wr16(0, 0x444E);
	fprintf(stderr, "falcon_net: DaynaPORT rings at 0x%08x, TAP %s%s\n", NET_PHYS, ifname,
	        g_addr ? ", DHCP and proxy ARP for the guest" : "");

	time_t beat = 0;
	unsigned hb = 0;
	uint8_t f[FRAME_MAX + 64];
	while (!quit) {
		struct pollfd p = { tap, POLLIN, 0 };
		poll(&p, 1, 1);
		int n;
		while ((n = read(tap, f, sizeof f)) > 0)
			if ((f[0] & 1) || !memcmp(f, guest_mac, 6))
				rx_queue(f, n);
		/* the core reloaded: it took our indices again; a core with older indices
		 * than ours never happens as long as only it moves its own */
		unsigned head = rd16(TX_HEAD) & 15;
		while (tx_tail != head) {
			unsigned slot = TX_SLOT(tx_tail), len = rd16(slot), flags = rd16(slot + 2);
			uint8_t *d = f;
			for (unsigned i = 0; i < len && i < sizeof f; i++) f[i] = mb[slot + 4 + i];
			if (flags && len >= 4) {                        /* framed: [len16][2 pad] frame */
				unsigned m = f[0] << 8 | f[1];
				if (m > 0 && m + 4 <= len) { d = f + 4; len = m; }
			}
			if (len > 0 && len <= FRAME_MAX && !dhcp_answer(d, len)) {
				if (write(tap, d, len) != (ssize_t)len && verbose)
					fprintf(stderr, "falcon_net: send %u bytes: %s\n", len, strerror(errno));
				else if (verbose > 1)
					fprintf(stderr, "falcon_net: out %u bytes\n", len);
			}
			tx_tail = (tx_tail + 1) & 15;
			wr16(TX_TAIL, tx_tail);
		}
		time_t now = time(NULL);
		if (now != beat) { beat = now; wr16(2, ++hb); }
	}
	wr16(0, 0);
	route_down();
	close(tap);
	return 0;
}
